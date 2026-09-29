use crate::Mesh;
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct StlReadResult {
    pub tri_count: u32,
    pub positions: Vec<f64>,
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

fn read_u32(data: &[u8], offset: usize) -> Result<u32, String> {
    let bytes = data
        .get(offset..offset + 4)
        .ok_or_else(|| "read_binary_stl: truncated triangle count".to_string())?;
    Ok(u32::from_le_bytes(bytes.try_into().unwrap()))
}

fn read_f32(data: &[u8], offset: usize) -> Result<f32, String> {
    let bytes = data
        .get(offset..offset + 4)
        .ok_or_else(|| "read_binary_stl: truncated triangle".to_string())?;
    Ok(f32::from_le_bytes(bytes.try_into().unwrap()))
}

/// Parse binary STL triangle positions for round-trip and import validation.
pub fn read_binary_stl(data: &[u8]) -> Result<StlReadResult, String> {
    if data.len() < 84 {
        return Err("read_binary_stl: file is shorter than the STL header".into());
    }
    let triangle_count = read_u32(data, 80)?;
    let expected = 84usize
        .checked_add(50usize.saturating_mul(triangle_count as usize))
        .ok_or_else(|| "read_binary_stl: file size overflow".to_string())?;
    if data.len() < expected {
        return Err(format!(
            "read_binary_stl: expected {expected} bytes, received {}",
            data.len()
        ));
    }
    let mut positions = Vec::with_capacity(triangle_count as usize * 9);
    let mut offset = 84;
    for _ in 0..triangle_count {
        offset += 12;
        for _ in 0..9 {
            positions.push(read_f32(data, offset)? as f64);
            offset += 4;
        }
        offset += 2;
    }
    Ok(StlReadResult {
        tri_count: triangle_count,
        positions,
    })
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

