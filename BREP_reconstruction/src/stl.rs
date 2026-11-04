//! Binary and ASCII STL import with deterministic vertex welding.

use crate::{Mesh, Vec3};
use std::collections::HashMap;
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

/// Encoding detected while importing an STL file (the kernel's STL format
/// layer, [`brep_kernel::parse_stl_triangles`]).
pub use brep_kernel::StlFormat;

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
    /// Source facets the reader left out, each with why. Only a facet with
    /// NO area is dropped — a zero scaled cross product in the file, or
    /// corners that weld to one vertex — so nothing with a surface is lost;
    /// `mesh` holds every other facet in file order.
    pub dropped_facets: Vec<DroppedFacet>,
}

/// One source facet the reader dropped, by its zero-based facet index.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DroppedFacet {
    /// Zero-based index of the facet in the file.
    pub facet: usize,
    /// Why it carries no area.
    pub reason: DroppedFacetReason,
}

/// Why a dropped facet has no area.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DroppedFacetReason {
    /// The facet's own corners are collinear or coincident: its scaled
    /// cross product is exactly zero in the file.
    ZeroArea,
    /// Two of its corners weld to one vertex at the weld tolerance.
    CollapsedByWeld,
}

impl Display for DroppedFacet {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self.reason {
            DroppedFacetReason::ZeroArea => {
                write!(f, "facet {} has zero area (collinear or coincident corners)", self.facet)
            }
            DroppedFacetReason::CollapsedByWeld => {
                write!(f, "facet {} has zero area after its corners weld together", self.facet)
            }
        }
    }
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

impl From<brep_kernel::StlError> for StlError {
    fn from(error: brep_kernel::StlError) -> Self {
        match error {
            brep_kernel::StlError::InvalidFormat(message) => Self::InvalidFormat(message),
            brep_kernel::StlError::Truncated { expected, actual } => {
                Self::Truncated { expected, actual }
            }
            brep_kernel::StlError::InvalidGeometry(message) => Self::InvalidGeometry(message),
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
/// The format layer (sniffing, the binary and ASCII grammars, and their
/// refusals) is the kernel's [`brep_kernel::parse_stl_triangles`]; this adds
/// the options, the zero-area drops and the deterministic weld.
pub fn parse_stl_bytes(bytes: &[u8], options: &StlReadOptions) -> Result<StlImport, StlError> {
    validate_options(options)?;
    let parsed = brep_kernel::parse_stl_triangles(bytes)?;
    let triangles = parsed
        .triangles
        .iter()
        .map(|triangle| triangle.map(|[x, y, z]| Vec3::new(x, y, z)))
        .collect();
    build_import(triangles, parsed.format, options)
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
    let mut dropped_facets = Vec::new();
    let mut kept = Vec::with_capacity(source.len());
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
        // A facet with no area at all (a tessellator's pole triangle, a
        // collapsed sliver) is dropped and named, not a reason to refuse
        // the file: it bounds nothing. Anything with area, however thin,
        // is kept — the converter's degenerate-coverage pass owns those.
        if facet_has_area(*triangle, facet)? {
            kept.push((facet, *triangle));
        } else {
            dropped_facets.push(DroppedFacet {
                facet,
                reason: DroppedFacetReason::ZeroArea,
            });
        }
    }
    if kept.is_empty() {
        return Err(StlError::InvalidGeometry(format!(
            "every one of the {source_triangle_count} facet(s) has zero area"
        )));
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
    let (vertices, triangles) =
        weld_vertices(&kept, bbox_min, diagonal, weld_tolerance, &mut dropped_facets)?;
    if triangles.is_empty() {
        return Err(StlError::InvalidGeometry(format!(
            "every one of the {source_triangle_count} facet(s) has zero area after welding"
        )));
    }
    dropped_facets.sort_by_key(|dropped| dropped.facet);
    let welded_vertex_count = vertices.len();
    Ok(StlImport {
        mesh: Mesh::new(vertices, triangles),
        format,
        source_triangle_count,
        source_vertex_count,
        welded_vertex_count,
        weld_tolerance,
        dropped_facets,
    })
}

/// Whether the facet has any area: its scaled cross product is not exactly
/// zero. Non-finite arithmetic is still an error.
fn facet_has_area(points: [Vec3; 3], facet: usize) -> Result<bool, StlError> {
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
    if !cross.is_finite() {
        return Err(StlError::InvalidGeometry(format!(
            "facet {facet} exceeds the supported numeric range"
        )));
    }
    Ok(cross.length_squared() != 0.0)
}

/// The kernel's tolerance weld ([`brep_kernel::VertexWeld`]: `floor` cells
/// of twice the tolerance, a match at `distance <= tolerance`, the
/// lowest-index matching vertex) over the kept facets' corners in order.
fn weld_vertices(
    source: &[(usize, [Vec3; 3])],
    bbox_min: Vec3,
    diagonal: f64,
    tolerance: f64,
    dropped: &mut Vec<DroppedFacet>,
) -> Result<(Vec<Vec3>, Vec<[u32; 3]>), StlError> {
    if tolerance == 0.0 {
        return weld_exact(source, dropped);
    }
    // The weld keys absolute cells floor(p / 2 tol); leave headroom for the
    // neighbour offsets on the largest coordinate the box reaches.
    let reach = diagonal + bbox_min.x.abs().max(bbox_min.y.abs()).max(bbox_min.z.abs());
    if !(reach / tolerance <= (i64::MAX - 2) as f64) {
        return Err(StlError::InvalidOptions(
            "weld_tolerance is too small relative to the model size; use zero for exact welding"
                .into(),
        ));
    }

    let mut weld = brep_kernel::VertexWeld::new(tolerance);
    let mut vertices = Vec::<Vec3>::new();
    let mut triangles = Vec::with_capacity(source.len());
    for &(facet, points) in source {
        let mut triangle = [0_u32; 3];
        for (corner, &point) in points.iter().enumerate() {
            let index = weld.weld([point.x, point.y, point.z]);
            if index == vertices.len() {
                vertices.push(point);
            }
            triangle[corner] = u32::try_from(index).map_err(|_| {
                StlError::InvalidGeometry(
                    "the welded mesh exceeds the u32 vertex-index limit".into(),
                )
            })?;
        }
        if triangle[0] == triangle[1]
            || triangle[1] == triangle[2]
            || triangle[2] == triangle[0]
            || !facet_has_area(triangle.map(|index| vertices[index as usize]), facet)?
        {
            dropped.push(DroppedFacet {
                facet,
                reason: DroppedFacetReason::CollapsedByWeld,
            });
            continue;
        }
        triangles.push(triangle);
    }
    Ok((vertices, triangles))
}

fn weld_exact(
    source: &[(usize, [Vec3; 3])],
    dropped: &mut Vec<DroppedFacet>,
) -> Result<(Vec<Vec3>, Vec<[u32; 3]>), StlError> {
    let mut vertices = Vec::new();
    let mut indices = HashMap::<[u64; 3], u32>::new();
    let mut triangles = Vec::with_capacity(source.len());
    for &(facet, points) in source {
        let mut triangle = [0_u32; 3];
        for (corner, &point) in points.iter().enumerate() {
            let canonical_bits = brep_kernel::canonical_bits;
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
        if triangle[0] == triangle[1]
            || triangle[1] == triangle[2]
            || triangle[2] == triangle[0]
            || !facet_has_area(triangle.map(|index| vertices[index as usize]), facet)?
        {
            dropped.push(DroppedFacet {
                facet,
                reason: DroppedFacetReason::CollapsedByWeld,
            });
            continue;
        }
        triangles.push(triangle);
    }
    Ok((vertices, triangles))
}

