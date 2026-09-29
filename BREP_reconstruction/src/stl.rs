//! Binary and ASCII STL import with deterministic vertex welding.

use crate::{Mesh, Vec3};
use std::collections::{BTreeMap, HashMap};
use std::fmt::{Display, Formatter};
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

/// The relative vertex-weld tolerance used when none is supplied.
pub const DEFAULT_RELATIVE_WELD_TOLERANCE: f64 = 1.0e-9;

/// Options controlling STL import.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StlReadOptions {
    /// Absolute distance used to merge vertices. `None` selects
    /// `bounding_box_diagonal * 1e-9`; `Some(0.0)` only merges bitwise-equal
    /// coordinates (with positive and negative zero treated as equal).
    pub weld_tolerance: Option<f64>,
}

/// Encoding detected while importing an STL file.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StlFormat {
    /// The 80-byte-header binary STL encoding.
    Binary,
    /// The line-oriented ASCII STL encoding.
    Ascii,
}

impl Display for StlFormat {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Binary => "binary",
            Self::Ascii => "ASCII",
        })
    }
}

/// An imported mesh and useful ingestion statistics.
#[derive(Clone, Debug, PartialEq)]
pub struct StlImport {
    /// Indexed triangle mesh with source winding preserved.
    pub mesh: Mesh,
    /// Encoding detected from the file contents.
    pub format: StlFormat,
    /// Number of facets in the source STL.
    pub source_triangle_count: usize,
    /// Number of source facet-corner vertices (three per facet).
    pub source_vertex_count: usize,
    /// Number of vertices remaining after deterministic welding.
    pub welded_vertex_count: usize,
    /// Absolute tolerance used for vertex welding.
    pub weld_tolerance: f64,
}

/// Failure to read, parse, or validate STL input.
#[derive(Debug)]
pub enum StlError {
    /// The source file could not be read.
    Io {
        /// Path passed to the importer.
        path: PathBuf,
        /// Underlying filesystem error.
        source: io::Error,
    },
    /// The byte stream is neither a valid binary nor a valid ASCII STL.
    InvalidFormat(String),
    /// The STL ends before all declared data is present.
    Truncated {
        /// Total byte count required by the binary facet count.
        expected: usize,
        /// Actual byte count supplied.
        actual: usize,
    },
    /// A numeric value or geometric facet is invalid.
    InvalidGeometry(String),
    /// An import option is outside its supported range.
    InvalidOptions(String),
}

impl Display for StlError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io { path, source } => {
                write!(f, "could not read STL '{}': {source}", path.display())
            }
            Self::InvalidFormat(message) => write!(f, "invalid STL format: {message}"),
            Self::Truncated { expected, actual } => write!(
                f,
                "truncated binary STL: expected {expected} bytes, found {actual}"
            ),
            Self::InvalidGeometry(message) => write!(f, "invalid STL geometry: {message}"),
            Self::InvalidOptions(message) => write!(f, "invalid STL options: {message}"),
        }
    }
}

impl std::error::Error for StlError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            _ => None,
        }
    }
}

/// Reads an STL file using [`StlReadOptions::default`].
pub fn read_stl(path: impl AsRef<Path>) -> Result<StlImport, StlError> {
    read_stl_with_options(path, &StlReadOptions::default())
}

/// Reads an STL file with explicit import options.
pub fn read_stl_with_options(
    path: impl AsRef<Path>,
    options: &StlReadOptions,
) -> Result<StlImport, StlError> {
    let path = path.as_ref();
    let bytes = fs::read(path).map_err(|source| StlError::Io {
        path: path.to_owned(),
        source,
    })?;
    parse_stl_bytes(&bytes, options)
}

/// Parses binary or ASCII STL bytes and creates an indexed mesh.
///
/// Binary structure is checked before the leading `solid` token because
/// binary STL headers are arbitrary and commonly begin with that word.
pub fn parse_stl_bytes(bytes: &[u8], options: &StlReadOptions) -> Result<StlImport, StlError> {
    validate_options(options)?;
    if bytes.is_empty() {
        return Err(StlError::InvalidFormat("the input is empty".into()));
    }

    let binary_length = declared_binary_length(bytes);
    let (format, triangles) = if binary_length == Some(bytes.len()) {
        (StlFormat::Binary, parse_binary(bytes)?)
    } else {
        match parse_ascii(bytes) {
            Ok(triangles) => (StlFormat::Ascii, triangles),
            Err(ascii_error) => {
                if let Some(expected) = binary_length {
                    if expected > bytes.len() && looks_like_binary(bytes) {
                        return Err(StlError::Truncated {
                            expected,
                            actual: bytes.len(),
                        });
                    }
                }
                return Err(ascii_error);
            }
        }
    };
    build_import(triangles, format, options)
}

fn validate_options(options: &StlReadOptions) -> Result<(), StlError> {
    if let Some(tolerance) = options.weld_tolerance {
        if !tolerance.is_finite() || tolerance < 0.0 {
            return Err(StlError::InvalidOptions(
                "weld_tolerance must be finite and non-negative".into(),
            ));
        }
    }
    Ok(())
}

fn declared_binary_length(bytes: &[u8]) -> Option<usize> {
    let count_bytes: [u8; 4] = bytes.get(80..84)?.try_into().ok()?;
    let count = u32::from_le_bytes(count_bytes) as usize;
    84_usize.checked_add(count.checked_mul(50)?)
}

fn looks_like_binary(bytes: &[u8]) -> bool {
    bytes
        .iter()
        .take(84)
        .any(|&byte| !matches!(byte, b'\t' | b'\n' | b'\r' | 0x20..=0x7e))
}

fn parse_binary(bytes: &[u8]) -> Result<Vec<[Vec3; 3]>, StlError> {
    let expected = declared_binary_length(bytes)
        .ok_or_else(|| StlError::InvalidFormat("binary header is incomplete".into()))?;
    if expected != bytes.len() {
        return Err(if expected > bytes.len() {
            StlError::Truncated {
                expected,
                actual: bytes.len(),
            }
        } else {
            StlError::InvalidFormat(format!(
                "binary STL has {} unexpected trailing bytes",
                bytes.len() - expected
            ))
        });
    }
    let count = (expected - 84) / 50;
    if count == 0 {
        return Err(StlError::InvalidGeometry(
            "the STL contains no triangles".into(),
        ));
    }
    let mut triangles = Vec::with_capacity(count);
    for facet in 0..count {
        let offset = 84 + facet * 50;
        let normal = read_binary_vec3(bytes, offset)?;
        if !normal.is_finite() {
            return Err(StlError::InvalidGeometry(format!(
                "facet {facet} has a non-finite normal"
            )));
        }
        triangles.push([
            read_binary_vec3(bytes, offset + 12)?,
            read_binary_vec3(bytes, offset + 24)?,
            read_binary_vec3(bytes, offset + 36)?,
        ]);
    }
    Ok(triangles)
}

fn read_binary_vec3(bytes: &[u8], offset: usize) -> Result<Vec3, StlError> {
    let component = |start: usize| -> Result<f64, StlError> {
        let raw: [u8; 4] = bytes
            .get(start..start + 4)
            .and_then(|slice| slice.try_into().ok())
            .ok_or_else(|| StlError::InvalidFormat("binary vector is incomplete".into()))?;
        Ok(f32::from_le_bytes(raw) as f64)
    };
    Ok(Vec3::new(
        component(offset)?,
        component(offset + 4)?,
        component(offset + 8)?,
    ))
}

fn parse_ascii(bytes: &[u8]) -> Result<Vec<[Vec3; 3]>, StlError> {
    let text = std::str::from_utf8(bytes)
        .map_err(|_| StlError::InvalidFormat("ASCII STL is not valid UTF-8".into()))?;
    let lines: Vec<(usize, &str)> = text
        .lines()
        .enumerate()
        .filter_map(|(index, line)| {
            let trimmed = line.trim();
            (!trimmed.is_empty()).then_some((index + 1, trimmed))
        })
        .collect();
    let Some(&(first_line, first_raw)) = lines.first() else {
        return Err(StlError::InvalidFormat("the input is empty".into()));
    };
    let first = first_raw.strip_prefix('\u{feff}').unwrap_or(first_raw);
    if !first
        .split_whitespace()
        .next()
        .is_some_and(|token| token.eq_ignore_ascii_case("solid"))
    {
        return Err(StlError::InvalidFormat(format!(
            "line {first_line}: ASCII STL must begin with 'solid'"
        )));
    }

    let mut cursor = 1;
    let mut triangles = Vec::new();
    while cursor < lines.len() {
        let (line_number, line) = lines[cursor];
        if line
            .split_whitespace()
            .next()
            .is_some_and(|token| token.eq_ignore_ascii_case("endsolid"))
        {
            cursor += 1;
            if cursor != lines.len() {
                return Err(StlError::InvalidFormat(format!(
                    "line {}: content follows 'endsolid'",
                    lines[cursor].0
                )));
            }
            break;
        }
        let normal = parse_prefixed_vec3(line_number, line, &["facet", "normal"])?;
        if !normal.is_finite() {
            return Err(StlError::InvalidGeometry(format!(
                "line {line_number}: facet normal is non-finite"
            )));
        }
        cursor += 1;
        expect_ascii_line(&lines, cursor, &["outer", "loop"])?;
        cursor += 1;
        let mut vertices = [Vec3::ZERO; 3];
        for vertex in &mut vertices {
            let &(number, source) = lines
                .get(cursor)
                .ok_or_else(|| StlError::InvalidFormat("ASCII STL ends inside a facet".into()))?;
            *vertex = parse_prefixed_vec3(number, source, &["vertex"])?;
            cursor += 1;
        }
        expect_ascii_line(&lines, cursor, &["endloop"])?;
        cursor += 1;
        expect_ascii_line(&lines, cursor, &["endfacet"])?;
        cursor += 1;
        triangles.push(vertices);
    }
    if triangles.is_empty() {
        return Err(StlError::InvalidGeometry(
            "the STL contains no triangles".into(),
        ));
    }
    Ok(triangles)
}

fn expect_ascii_line(
    lines: &[(usize, &str)],
    cursor: usize,
    expected: &[&str],
) -> Result<(), StlError> {
    let &(number, source) = lines
        .get(cursor)
        .ok_or_else(|| StlError::InvalidFormat("ASCII STL ends inside a facet".into()))?;
    let tokens: Vec<_> = source.split_whitespace().collect();
    if tokens.len() == expected.len()
        && tokens
            .iter()
            .zip(expected)
            .all(|(actual, expected)| actual.eq_ignore_ascii_case(expected))
    {
        Ok(())
    } else {
        Err(StlError::InvalidFormat(format!(
            "line {number}: expected '{}'",
            expected.join(" ")
        )))
    }
}

fn parse_prefixed_vec3(
    line_number: usize,
    source: &str,
    prefix: &[&str],
) -> Result<Vec3, StlError> {
    let tokens: Vec<_> = source.split_whitespace().collect();
    if tokens.len() != prefix.len() + 3
        || !tokens[..prefix.len()]
            .iter()
            .zip(prefix)
            .all(|(actual, expected)| actual.eq_ignore_ascii_case(expected))
    {
        return Err(StlError::InvalidFormat(format!(
            "line {line_number}: expected '{} x y z'",
            prefix.join(" ")
        )));
    }
    let mut values = [0.0; 3];
    for (index, token) in tokens[prefix.len()..].iter().enumerate() {
        values[index] = token.parse::<f64>().map_err(|_| {
            StlError::InvalidFormat(format!("line {line_number}: '{token}' is not a number"))
        })?;
    }
    let vector = Vec3::new(values[0], values[1], values[2]);
    if !vector.is_finite() {
        return Err(StlError::InvalidGeometry(format!(
            "line {line_number}: vector contains a non-finite value"
        )));
    }
    Ok(vector)
}

fn build_import(
    source: Vec<[Vec3; 3]>,
    format: StlFormat,
    options: &StlReadOptions,
) -> Result<StlImport, StlError> {
    let source_triangle_count = source.len();
    let source_vertex_count = source_triangle_count.checked_mul(3).ok_or_else(|| {
        StlError::InvalidGeometry("source vertex count exceeds platform limits".into())
    })?;
    let mut bbox_min = source[0][0];
    let mut bbox_max = source[0][0];
    for (facet, triangle) in source.iter().enumerate() {
        for &point in triangle {
            if !point.is_finite() {
                return Err(StlError::InvalidGeometry(format!(
                    "facet {facet} has a non-finite vertex"
                )));
            }
            bbox_min.x = bbox_min.x.min(point.x);
            bbox_min.y = bbox_min.y.min(point.y);
            bbox_min.z = bbox_min.z.min(point.z);
            bbox_max.x = bbox_max.x.max(point.x);
            bbox_max.y = bbox_max.y.max(point.y);
            bbox_max.z = bbox_max.z.max(point.z);
        }
        validate_triangle(*triangle, facet)?;
    }
    let extent = bbox_max - bbox_min;
    let diagonal = extent.length();
    if !extent.is_finite() || !diagonal.is_finite() || diagonal <= 0.0 {
        return Err(StlError::InvalidGeometry(
            "the bounding box is zero-sized or outside the supported numeric range".into(),
        ));
    }
    let weld_tolerance = options
        .weld_tolerance
        .unwrap_or(diagonal * DEFAULT_RELATIVE_WELD_TOLERANCE);
    if !weld_tolerance.is_finite() {
        return Err(StlError::InvalidGeometry(
            "the derived weld tolerance is outside the supported numeric range".into(),
        ));
    }
    let (vertices, triangles) = weld_vertices(&source, bbox_min, diagonal, weld_tolerance)?;
    let welded_vertex_count = vertices.len();
    Ok(StlImport {
        mesh: Mesh::new(vertices, triangles),
        format,
        source_triangle_count,
        source_vertex_count,
        welded_vertex_count,
        weld_tolerance,
    })
}

fn validate_triangle(points: [Vec3; 3], facet: usize) -> Result<(), StlError> {
    let ab = points[1] - points[0];
    let ac = points[2] - points[0];
    if !ab.is_finite() || !ac.is_finite() {
        return Err(StlError::InvalidGeometry(format!(
            "facet {facet} exceeds the supported numeric range"
        )));
    }
    let scale = [ab.x, ab.y, ab.z, ac.x, ac.y, ac.z]
        .into_iter()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    let cross = if scale > 0.0 {
        (ab / scale).cross(ac / scale)
    } else {
        Vec3::ZERO
    };
    if !cross.is_finite() || cross.length_squared() == 0.0 {
        return Err(StlError::InvalidGeometry(format!(
            "facet {facet} is degenerate"
        )));
    }
    Ok(())
}

fn weld_vertices(
    source: &[[Vec3; 3]],
    bbox_min: Vec3,
    diagonal: f64,
    tolerance: f64,
) -> Result<(Vec<Vec3>, Vec<[u32; 3]>), StlError> {
    if tolerance == 0.0 {
        return weld_exact(source);
    }
    // Leave headroom for the neighbor-cell offsets below.
    if diagonal / tolerance > (i64::MAX - 2) as f64 {
        return Err(StlError::InvalidOptions(
            "weld_tolerance is too small relative to the model size; use zero for exact welding"
                .into(),
        ));
    }

    let mut vertices = Vec::<Vec3>::new();
    let mut cells = BTreeMap::<[i64; 3], Vec<u32>>::new();
    let mut triangles = Vec::with_capacity(source.len());
    for (facet, points) in source.iter().enumerate() {
        let mut triangle = [0_u32; 3];
        for (corner, &point) in points.iter().enumerate() {
            let key = cell_key(point, bbox_min, tolerance)?;
            let mut matched = None;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for dz in -1..=1 {
                        let neighbor = [key[0] + dx, key[1] + dy, key[2] + dz];
                        if let Some(candidates) = cells.get(&neighbor) {
                            for &candidate in candidates {
                                let delta = (point - vertices[candidate as usize]) / tolerance;
                                if delta.length_squared() <= 1.0
                                    && matched.is_none_or(|current| candidate < current)
                                {
                                    matched = Some(candidate);
                                }
                            }
                        }
                    }
                }
            }
            let index = match matched {
                Some(index) => index,
                None => {
                    let index = u32::try_from(vertices.len()).map_err(|_| {
                        StlError::InvalidGeometry(
                            "the welded mesh exceeds the u32 vertex-index limit".into(),
                        )
                    })?;
                    vertices.push(point);
                    cells.entry(key).or_default().push(index);
                    index
                }
            };
            triangle[corner] = index;
        }
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0] {
            return Err(StlError::InvalidGeometry(format!(
                "facet {facet} becomes degenerate at weld tolerance {tolerance:e}"
            )));
        }
        validate_triangle(triangle.map(|index| vertices[index as usize]), facet)?;
        triangles.push(triangle);
    }
    Ok((vertices, triangles))
}

fn cell_key(point: Vec3, origin: Vec3, tolerance: f64) -> Result<[i64; 3], StlError> {
    let relative = point - origin;
    let cell = [relative.x, relative.y, relative.z].map(|value| (value / tolerance).floor());
    if cell
        .iter()
        .any(|&value| !value.is_finite() || value < 0.0 || value > (i64::MAX - 2) as f64)
    {
        return Err(StlError::InvalidOptions(
            "weld_tolerance cannot be represented by the spatial index".into(),
        ));
    }
    Ok(cell.map(|value| value as i64))
}

fn weld_exact(source: &[[Vec3; 3]]) -> Result<(Vec<Vec3>, Vec<[u32; 3]>), StlError> {
    let mut vertices = Vec::new();
    let mut indices = HashMap::<[u64; 3], u32>::new();
    let mut triangles = Vec::with_capacity(source.len());
    for (facet, points) in source.iter().enumerate() {
        let mut triangle = [0_u32; 3];
        for (corner, &point) in points.iter().enumerate() {
            let canonical_bits = |value: f64| if value == 0.0 { 0 } else { value.to_bits() };
            let key = [
                canonical_bits(point.x),
                canonical_bits(point.y),
                canonical_bits(point.z),
            ];
            let index = match indices.get(&key) {
                Some(&index) => index,
                None => {
                    let index = u32::try_from(vertices.len()).map_err(|_| {
                        StlError::InvalidGeometry(
                            "the welded mesh exceeds the u32 vertex-index limit".into(),
                        )
                    })?;
                    vertices.push(point);
                    indices.insert(key, index);
                    index
                }
            };
            triangle[corner] = index;
        }
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0] {
            return Err(StlError::InvalidGeometry(format!(
                "facet {facet} becomes degenerate after exact vertex welding"
            )));
        }
        validate_triangle(triangle.map(|index| vertices[index as usize]), facet)?;
        triangles.push(triangle);
    }
    Ok((vertices, triangles))
}

