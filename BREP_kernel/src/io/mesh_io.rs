use crate::Mesh;
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct StlReadResult {
    pub tri_count: u32,
    pub positions: Vec<f64>,
    /// Encoding the bytes were read as (`"binary"` or `"ascii"` in JSON).
    pub format: StlFormat,
}

#[derive(Clone, Debug, Serialize)]
pub struct ObjReadResult {
    pub positions: Vec<f64>,
    pub indices: Vec<u32>,
}

fn triangle_point(mesh: &Mesh, index: u32) -> Result<[f64; 3], String> {
    let offset = index as usize * 3;
    if offset + 2 >= mesh.positions.len() {
        return Err("write_binary_stl: triangle index outside position buffer".into());
    }
    Ok([
        mesh.positions[offset],
        mesh.positions[offset + 1],
        mesh.positions[offset + 2],
    ])
}

fn write_f32(output: &mut [u8], offset: usize, value: f64) {
    output[offset..offset + 4].copy_from_slice(&(value as f32).to_le_bytes());
}

/// Write the kernel mesh as binary STL.
pub fn write_binary_stl(mesh: &Mesh, name: &str) -> Result<Vec<u8>, String> {
    mesh.validate()?;
    let triangle_count = mesh.indices.len() / 3;
    let mut output = vec![0u8; 84 + 50 * triangle_count];
    let header = format!("binary STL - {name}");
    let header_bytes = header.as_bytes();
    let header_length = 79usize.min(header_bytes.len());
    output[..header_length].copy_from_slice(&header_bytes[..header_length]);
    output[80..84].copy_from_slice(&(triangle_count as u32).to_le_bytes());
    let mut offset = 84;
    for triangle in mesh.indices.chunks_exact(3) {
        let a = triangle_point(mesh, triangle[0])?;
        let b = triangle_point(mesh, triangle[1])?;
        let c = triangle_point(mesh, triangle[2])?;
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let mut normal = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
        if length > 1e-30 {
            for coordinate in &mut normal {
                *coordinate /= length;
            }
        }
        for coordinate in normal {
            write_f32(&mut output, offset, coordinate);
            offset += 4;
        }
        for point in [a, b, c] {
            for coordinate in point {
                write_f32(&mut output, offset, coordinate);
                offset += 4;
            }
        }
        output[offset..offset + 2].copy_from_slice(&0u16.to_le_bytes());
        offset += 2;
    }
    Ok(output)
}

/// Encoding detected while reading an STL.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum StlFormat {
    /// The 80-byte-header binary STL encoding.
    Binary,
    /// The line-oriented ASCII STL encoding.
    Ascii,
}

impl std::fmt::Display for StlFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Binary => "binary",
            Self::Ascii => "ASCII",
        })
    }
}

/// Why STL bytes are not a readable STL.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StlError {
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
}

impl std::fmt::Display for StlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidFormat(message) => write!(f, "invalid STL format: {message}"),
            Self::Truncated { expected, actual } => write!(
                f,
                "truncated binary STL: expected {expected} bytes, found {actual}"
            ),
            Self::InvalidGeometry(message) => write!(f, "invalid STL geometry: {message}"),
        }
    }
}

impl std::error::Error for StlError {}

/// The facets of an STL in file order, before any welding.
#[derive(Clone, Debug, PartialEq)]
pub struct StlTriangles {
    /// Encoding detected from the bytes.
    pub format: StlFormat,
    /// Each facet's three corners, in file order and winding.
    pub triangles: Vec<[[f64; 3]; 3]>,
}

/// Parse binary or ASCII STL bytes into their facets.
///
/// Binary structure is checked before the leading `solid` token because
/// binary STL headers are arbitrary and commonly begin with that word. A
/// binary file must hold exactly the bytes its facet count declares; a file
/// with no facets, or a facet normal or ASCII value that is not finite, is
/// refused. Corner finiteness of binary facets is the caller's check.
pub fn parse_stl_triangles(bytes: &[u8]) -> Result<StlTriangles, StlError> {
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
    Ok(StlTriangles { format, triangles })
}

/// Read binary or ASCII STL bytes ([`parse_stl_triangles`]) as a flat
/// triangle soup, refusing a corner that is not finite.
pub fn read_stl(data: &[u8]) -> Result<StlReadResult, String> {
    let parsed = parse_stl_triangles(data).map_err(|error| format!("read_stl: {error}"))?;
    let mut positions = Vec::with_capacity(parsed.triangles.len() * 9);
    for (facet, triangle) in parsed.triangles.iter().enumerate() {
        if triangle.iter().flatten().any(|value| !value.is_finite()) {
            return Err(format!(
                "read_stl: {}",
                StlError::InvalidGeometry(format!("facet {facet} has a non-finite vertex"))
            ));
        }
        positions.extend(triangle.iter().flatten());
    }
    Ok(StlReadResult {
        tri_count: parsed.triangles.len() as u32,
        positions,
        format: parsed.format,
    })
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

fn parse_binary(bytes: &[u8]) -> Result<Vec<[[f64; 3]; 3]>, StlError> {
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
        if !normal.iter().all(|value| value.is_finite()) {
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

fn read_binary_vec3(bytes: &[u8], offset: usize) -> Result<[f64; 3], StlError> {
    let component = |start: usize| -> Result<f64, StlError> {
        let raw: [u8; 4] = bytes
            .get(start..start + 4)
            .and_then(|slice| slice.try_into().ok())
            .ok_or_else(|| StlError::InvalidFormat("binary vector is incomplete".into()))?;
        Ok(f32::from_le_bytes(raw) as f64)
    };
    Ok([
        component(offset)?,
        component(offset + 4)?,
        component(offset + 8)?,
    ])
}

fn parse_ascii(bytes: &[u8]) -> Result<Vec<[[f64; 3]; 3]>, StlError> {
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
        // parse_prefixed_vec3 refuses a non-finite value, the facet
        // normal's included.
        parse_prefixed_vec3(line_number, line, &["facet", "normal"])?;
        cursor += 1;
        expect_ascii_line(&lines, cursor, &["outer", "loop"])?;
        cursor += 1;
        let mut vertices = [[0.0; 3]; 3];
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
) -> Result<[f64; 3], StlError> {
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
    if !values.iter().all(|value| value.is_finite()) {
        return Err(StlError::InvalidGeometry(format!(
            "line {line_number}: vector contains a non-finite value"
        )));
    }
    Ok(values)
}

/// Write ASCII STL: `solid {name}`, one facet per `(normal, corners)` item,
/// `endsolid {name}`. Values print with their own `Display`, so an `f32`
/// source keeps its shortest `f32` text.
pub fn write_ascii_stl<T, I>(name: &str, facets: I) -> String
where
    T: std::fmt::Display,
    I: IntoIterator<Item = ([T; 3], [[T; 3]; 3])>,
{
    let mut out = format!("solid {name}\n");
    for (normal, corners) in facets {
        out.push_str(&format!(
            "  facet normal {} {} {}\n    outer loop\n",
            normal[0], normal[1], normal[2]
        ));
        for v in &corners {
            out.push_str(&format!("      vertex {} {} {}\n", v[0], v[1], v[2]));
        }
        out.push_str("    endloop\n  endfacet\n");
    }
    out.push_str(&format!("endsolid {name}\n"));
    out
}

/// Resolve one OBJ face-vertex reference (`v`, `v/vt`, `v//vn`, `v/vt/vn`)
/// to a 0-based position index. OBJ indices are 1-based; negative values
/// count back from the vertices defined so far, which lets exporters emit
/// faces before the vertex table is complete.
fn obj_face_vertex(token: &str, vertex_count: usize, line_number: usize) -> Result<u32, String> {
    let mut parts = token.split('/');
    let vertex_text = parts.next().unwrap_or_default();
    if parts.count() > 2 {
        return Err(format!(
            "read_obj: line {line_number}: face vertex '{token}' has more than v/vt/vn parts"
        ));
    }
    let raw: i64 = vertex_text.parse().map_err(|_| {
        format!("read_obj: line {line_number}: face vertex '{token}' has no vertex index")
    })?;
    // Index 0 does not exist in either the 1-based or the relative scheme.
    let resolved = match raw {
        1.. => raw - 1,
        0 => {
            return Err(format!(
                "read_obj: line {line_number}: vertex index 0 is not valid OBJ"
            ))
        }
        _ => vertex_count as i64 + raw,
    };
    if resolved < 0 || resolved >= vertex_count as i64 {
        return Err(format!(
            "read_obj: line {line_number}: vertex index {raw} outside the \
             {vertex_count} vertices defined so far"
        ));
    }
    Ok(resolved as u32)
}

/// Parse Wavefront OBJ text into flat positions and triangle indices.
///
/// Faces with more than three vertices fan-triangulate so the result is
/// always a triangle list (exported polygons are overwhelmingly convex, and
/// the faceted importer welds and repairs downstream anyway). Texture and
/// normal references, grouping state, and material lines carry nothing the
/// kernel mesh keeps, so they are skipped; unknown keywords are OBJ
/// extensions, not errors.
pub fn read_obj(text: &str) -> Result<ObjReadResult, String> {
    let mut positions = Vec::<f64>::new();
    let mut indices = Vec::<u32>::new();
    for (line_index, raw_line) in text.lines().enumerate() {
        let line_number = line_index + 1;
        // '#' starts a comment; it may follow data on the same line.
        let line = raw_line.split('#').next().unwrap_or_default().trim();
        if line.is_empty() {
            continue;
        }
        let mut tokens = line.split_whitespace();
        match tokens.next().unwrap_or_default() {
            "v" => {
                let mut point = [0.0f64; 3];
                for coordinate in &mut point {
                    *coordinate = tokens
                        .next()
                        .ok_or_else(|| format!("read_obj: line {line_number}: vertex needs x y z"))?
                        .parse()
                        .map_err(|_| {
                            format!(
                                "read_obj: line {line_number}: vertex coordinate is not a number"
                            )
                        })?;
                }
                // A fourth value (w) or vertex-color extension may follow;
                // only the position matters here.
                positions.extend(point);
            }
            "f" => {
                // Negative indices resolve against the vertices defined
                // BEFORE this face line, so count them first.
                let vertex_count = positions.len() / 3;
                let corners = tokens
                    .map(|token| obj_face_vertex(token, vertex_count, line_number))
                    .collect::<Result<Vec<_>, _>>()?;
                if corners.len() < 3 {
                    return Err(format!(
                        "read_obj: line {line_number}: face needs at least 3 vertices"
                    ));
                }
                for corner in 1..corners.len() - 1 {
                    indices.extend([corners[0], corners[corner], corners[corner + 1]]);
                }
            }
            // vn/vt/vp geometry the mesh does not keep, o/g/s grouping,
            // usemtl/mtllib materials, and vendor extensions.
            _ => {}
        }
    }
    // An input with no geometry at all is far more likely a wrong-format
    // file than an intentionally empty model; say so instead of returning
    // an empty mesh that only fails further down the import chain.
    if positions.is_empty() {
        return Err("read_obj: no vertices found — not Wavefront OBJ text?".into());
    }
    if indices.is_empty() {
        return Err("read_obj: no faces found — nothing to build a mesh from".into());
    }
    Ok(ObjReadResult { positions, indices })
}

/// Write Wavefront OBJ positions, normals, and indexed triangle faces.
pub fn write_obj(mesh: &Mesh, name: &str) -> Result<String, String> {
    mesh.validate()?;
    let mut lines = vec![format!("# {name}"), format!("o {name}")];
    for point in mesh.positions.chunks_exact(3) {
        lines.push(format!("v {} {} {}", point[0], point[1], point[2]));
    }
    for normal in mesh.normals.chunks_exact(3) {
        lines.push(format!("vn {} {} {}", normal[0], normal[1], normal[2]));
    }
    for triangle in mesh.indices.chunks_exact(3) {
        let a = triangle[0] + 1;
        let b = triangle[1] + 1;
        let c = triangle[2] + 1;
        lines.push(format!("f {a}//{a} {b}//{b} {c}//{c}"));
    }
    Ok(lines.join("\n") + "\n")
}

