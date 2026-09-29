//! Binary glTF 2.0 (`.glb`) writer for DISPLAY MESHES — tessellated exchange
//! for viewers, web pages and downstream rendering. It is NOT a replacement for
//! the STEP lane: no exact surface, no topology, no feature tree leaves here.
//!
//! # The container
//!
//! A GLB is a 12-byte header (`glTF` magic, version 2, total length) followed by
//! chunks, each `[u32 length][u32 type][payload]`. This writer emits exactly the
//! two the spec defines for a self-contained file: chunk 0 `JSON` (padded with
//! spaces) and chunk 1 `BIN\0` (padded with zeros), both to a 4-byte boundary.
//! Every accessor element here is 4 bytes wide (`f32` positions/normals, `u32`
//! indices), so every buffer view starts 4-byte aligned by construction.
//!
//! The JSON is hand-rolled. The kernel takes no glTF dependency, and writing the
//! document directly is what makes the key order — and therefore the bytes — the
//! same on every run for the same input.
//!
//! # Axis convention — stated, and carried by the node, not the vertices
//!
//! glTF is right-handed with **+Y up**. So is this application's world: the
//! feature pipeline builds every frame against `world_up = (0, 1, 0)`
//! (`feature_pipeline/mod.rs`, the "standard world-up convention" the Plane
//! feature's documentation cites), the viewport camera's default up is `+Y`
//! (`BREP_render::view::ViewCamera`), and the ViewCube puts TOP/BOTTOM on ±Y
//! and FRONT/BACK on ±Z. The two conventions therefore AGREE and the mapping is
//! the identity — but it is written out as an explicit `"rotation"` on the root
//! node ([`GlbUpAxis::YUp`] → `[0,0,0,1]`) rather than left implicit, so the
//! file states which convention it was written under.
//!
//! [`GlbUpAxis::ZUp`] is the other half of that statement: a Z-up world (the
//! artifact renderer's camera is one) rotates -90° about X, so world +Z becomes
//! glTF +Y and world +Y becomes glTF -Z. Either way the ROTATION IS A NODE
//! TRANSFORM: vertex data is written in the caller's own coordinates, unrotated,
//! so a consumer that ignores the node still reads the model's own numbers and
//! one that honours it sees the model standing up.
//!
//! # Units
//!
//! glTF's linear unit is the METRE; this application models in millimetres (the
//! STEP writer's `MM`, the unit STL and OBJ leave implicit). The root node
//! therefore also carries `"scale"` — `unit_scale`, `0.001` for millimetres — by
//! the same "transform, not vertices" rule as the rotation. A viewer shows a
//! 100 mm part as 0.1 m, which is what it is; the vertex buffer still carries
//! `100`.
//!
//! # Colour
//!
//! Colours arrive as 8-bit **sRGB**, the spelling the metadata store and the
//! STEP writer's [`crate::StepColors`] use. glTF's `baseColorFactor` is
//! **linear**, so each channel is decoded with the sRGB electro-optical transfer
//! function on `byte / 255`:
//!
//! ```text
//! c <= 0.04045  ->  c / 12.92
//! c >  0.04045  ->  ((c + 0.055) / 1.055) ^ 2.4
//! ```
//!
//! which is the same decode the renderer applies before shading
//! (`BREP_render::color::srgb_to_linear`), so the file carries the colour the
//! viewport shows.
//!
//! Grouping: one glTF mesh per solid, one PRIMITIVE per distinct colour within
//! it. A solid whose faces all take the body colour is one primitive; a solid
//! carrying per-face colours ([`GlbSolid::face_colors`], keyed by the same
//! per-triangle face id the display mesh carries) is one primitive per distinct
//! colour, in first-appearance order. Materials are deduplicated across the
//! whole file and named after their hex, so two bodies painted alike share one
//! material. A group with no colour at all gets no material — the caller decides
//! whether "uncoloured" means the viewer's default or a colour of its own.
use crate::Mesh;

/// `glTF` as a little-endian `u32` — the first four bytes of every GLB.
pub const GLB_MAGIC: u32 = 0x4654_6C67;
/// `JSON` chunk type.
const CHUNK_JSON: u32 = 0x4E4F_534A;
/// `BIN\0` chunk type.
const CHUNK_BIN: u32 = 0x004E_4942;
/// `ARRAY_BUFFER` — vertex attribute buffer views.
const TARGET_ARRAY_BUFFER: u32 = 34962;
/// `ELEMENT_ARRAY_BUFFER` — index buffer views.
const TARGET_ELEMENT_ARRAY_BUFFER: u32 = 34963;
/// `FLOAT` accessor component type.
const COMPONENT_FLOAT: u32 = 5126;
/// `UNSIGNED_INT` accessor component type (indices; core glTF 2.0).
const COMPONENT_UNSIGNED_INT: u32 = 5125;

/// Which axis of the CALLER's world points up, and therefore what rotation the
/// root node carries to reach glTF's own right-handed +Y-up frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GlbUpAxis {
    /// +Y up — this application's world (see the module doc). Identity.
    YUp,
    /// +Z up — rotate -90° about X, so world +Z lands on glTF +Y.
    ZUp,
}

impl GlbUpAxis {
    /// The node rotation as glTF spells a quaternion: `[x, y, z, w]`.
    pub fn node_rotation(self) -> [f64; 4] {
        match self {
            GlbUpAxis::YUp => [0.0, 0.0, 0.0, 1.0],
            // -90° about X maps (x, y, z) -> (x, z, -y): +Z up becomes +Y up and
            // the world's +Y (into the scene) becomes glTF's -Z. det = +1, so
            // handedness — and therefore the sign of every volume — is kept.
            GlbUpAxis::ZUp => [
                -std::f64::consts::FRAC_1_SQRT_2,
                0.0,
                0.0,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        }
    }
}

/// One solid to write: its display mesh plus the colours it is shown in.
#[derive(Clone, Debug, Default)]
pub struct GlbSolid {
    /// The scene name — becomes the glTF node and mesh name.
    pub name: String,
    /// Positions / normals / triangle indices, and the per-TRIANGLE face id the
    /// `face_colors` keys match. An empty `face_ids` means "one group".
    pub mesh: Mesh,
    /// The body colour, 8-bit sRGB. `None` writes the primitive with no material.
    pub color: Option<[u8; 3]>,
    /// Per-face colour overrides, keyed by the face id in `mesh.face_ids`.
    /// A face listed here wins over `color` for its own triangles.
    pub face_colors: std::collections::BTreeMap<u32, [u8; 3]>,
}

/// What one written file contains — the counts a caller reports or asserts on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GlbReport {
    /// Meshes written (solids that carried at least one triangle).
    pub meshes: usize,
    /// Primitives across every mesh — one per distinct colour within a solid.
    pub primitives: usize,
    /// Distinct materials, deduplicated across the whole file.
    pub materials: usize,
    /// Triangles written.
    pub triangles: usize,
    /// Total file length in bytes.
    pub bytes: usize,
}

/// sRGB electro-optical transfer: one 8-bit sRGB channel -> linear 0..1.
fn srgb_byte_to_linear(byte: u8) -> f64 {
    let c = f64::from(byte) / 255.0;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// A JSON string literal, with the escapes the grammar requires. Scene names
/// carry `:` today and nothing stops a quote or a backslash tomorrow.
fn json_string(value: &str) -> String {
    let mut out = String::with_capacity(value.len() + 2);
    out.push('"');
    for ch in value.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

/// A finite f64 as a JSON number. Rust's `Display` never emits an exponent and
/// prints the shortest round-tripping form, so this is both valid JSON and
/// exact; a non-finite value is refused rather than written as `NaN`.
fn json_number(value: f64) -> Result<String, String> {
    if !value.is_finite() {
        return Err("write_glb: non-finite number".into());
    }
    Ok(format!("{value}"))
}

fn json_numbers(values: &[f64]) -> Result<String, String> {
    let parts: Result<Vec<String>, String> = values.iter().map(|v| json_number(*v)).collect();
    Ok(format!("[{}]", parts?.join(",")))
}

fn push_u32(bytes: &mut Vec<u8>, value: u32) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

/// Colour of one triangle: its face's override if it has one, else the body's.
fn triangle_color(solid: &GlbSolid, triangle: usize) -> Option<[u8; 3]> {
    solid
        .mesh
        .face_ids
        .get(triangle)
        .and_then(|face| solid.face_colors.get(face))
        .copied()
        .or(solid.color)
}

/// Write `solids` as one binary glTF 2.0 document.
///
/// `document_name` names the scene and its root node; `up` states the caller's
/// world up axis and `unit_scale` the caller's unit in metres (`0.001` for
/// millimetres) — both ride on the root node's transform, never on the vertices.
///
/// Solids with no triangles are SKIPPED (a glTF accessor must hold at least one
/// element); a set in which nothing has a triangle is refused, as the OBJ and
/// STL lanes refuse an empty scene.
pub fn write_glb(
    solids: &[GlbSolid],
    document_name: &str,
    up: GlbUpAxis,
    unit_scale: f64,
) -> Result<(Vec<u8>, GlbReport), String> {
    let mut bin: Vec<u8> = Vec::new();
    let mut buffer_views: Vec<String> = Vec::new();
    let mut accessors: Vec<String> = Vec::new();
    let mut meshes: Vec<String> = Vec::new();
    let mut nodes: Vec<String> = Vec::new();
    let mut palette: Vec<[u8; 3]> = Vec::new();
    let mut report = GlbReport::default();

    for solid in solids {
        solid.mesh.validate()?;
        let triangles = solid.mesh.indices.len() / 3;
        if triangles == 0 {
            continue;
        }
        let vertices = solid.mesh.positions.len() / 3;

        // Narrow to f32 ONCE, here: the accessor's min/max must be the values
        // the file actually stores, so they are taken after the narrowing.
        let positions: Vec<f32> = solid.mesh.positions.iter().map(|v| *v as f32).collect();
        let normals: Vec<f32> = solid.mesh.normals.iter().map(|v| *v as f32).collect();
        if !positions.iter().all(|v| v.is_finite()) || !normals.iter().all(|v| v.is_finite()) {
            return Err(format!(
                "write_glb: solid '{}' has a non-finite position or normal",
                solid.name
            ));
        }
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for point in positions.chunks_exact(3) {
            for axis in 0..3 {
                let value = f64::from(point[axis]);
                min[axis] = min[axis].min(value);
                max[axis] = max[axis].max(value);
            }
        }

        let position_view = buffer_views.len();
        buffer_views.push(format!(
            r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":{TARGET_ARRAY_BUFFER}}}"#,
            bin.len(),
            positions.len() * 4
        ));
        for value in &positions {
            bin.extend_from_slice(&value.to_le_bytes());
        }
        let position_accessor = accessors.len();
        accessors.push(format!(
            r#"{{"bufferView":{position_view},"componentType":{COMPONENT_FLOAT},"count":{vertices},"type":"VEC3","min":{},"max":{}}}"#,
            json_numbers(&min)?,
            json_numbers(&max)?
        ));

        let normal_view = buffer_views.len();
        buffer_views.push(format!(
            r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":{TARGET_ARRAY_BUFFER}}}"#,
            bin.len(),
            normals.len() * 4
        ));
        for value in &normals {
            bin.extend_from_slice(&value.to_le_bytes());
        }
        let normal_accessor = accessors.len();
        accessors.push(format!(
            r#"{{"bufferView":{normal_view},"componentType":{COMPONENT_FLOAT},"count":{vertices},"type":"VEC3"}}"#
        ));

        // Group triangles by the colour they are shown in, in FIRST-APPEARANCE
        // order so the primitive order follows the mesh rather than the palette.
        let mut groups: Vec<(Option<[u8; 3]>, Vec<u32>)> = Vec::new();
        for triangle in 0..triangles {
            let color = triangle_color(solid, triangle);
            let slot = match groups.iter().position(|(key, _)| *key == color) {
                Some(slot) => slot,
                None => {
                    groups.push((color, Vec::new()));
                    groups.len() - 1
                }
            };
            groups[slot]
                .1
                .extend_from_slice(&solid.mesh.indices[triangle * 3..triangle * 3 + 3]);
        }

        let mut primitives: Vec<String> = Vec::new();
        for (color, indices) in &groups {
            let index_view = buffer_views.len();
            buffer_views.push(format!(
                r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":{TARGET_ELEMENT_ARRAY_BUFFER}}}"#,
                bin.len(),
                indices.len() * 4
            ));
            for index in indices {
                push_u32(&mut bin, *index);
            }
            let index_accessor = accessors.len();
            accessors.push(format!(
                r#"{{"bufferView":{index_view},"componentType":{COMPONENT_UNSIGNED_INT},"count":{},"type":"SCALAR"}}"#,
                indices.len()
            ));
            let material = color.map(|rgb| match palette.iter().position(|c| *c == rgb) {
                Some(slot) => slot,
                None => {
                    palette.push(rgb);
                    palette.len() - 1
                }
            });
            let material_key = match material {
                Some(slot) => format!(r#","material":{slot}"#),
                None => String::new(),
            };
            primitives.push(format!(
                r#"{{"attributes":{{"POSITION":{position_accessor},"NORMAL":{normal_accessor}}},"indices":{index_accessor}{material_key},"mode":4}}"#
            ));
        }
        report.primitives += primitives.len();
        report.triangles += triangles;

        let mesh_index = meshes.len();
        meshes.push(format!(
            r#"{{"name":{},"primitives":[{}]}}"#,
            json_string(&solid.name),
            primitives.join(",")
        ));
        // Node 0 is the root; the solids are its children, numbered from 1 in
        // the order they were written.
        nodes.push(format!(
            r#"{{"name":{},"mesh":{mesh_index}}}"#,
            json_string(&solid.name)
        ));
    }

    if meshes.is_empty() {
        return Err("nothing to export: the scene has no triangles".into());
    }
    report.meshes = meshes.len();
    report.materials = palette.len();

    let materials: Vec<String> = palette
        .iter()
        .map(|rgb| {
            let linear = rgb.map(srgb_byte_to_linear);
            Ok(format!(
                r#"{{"name":{},"pbrMetallicRoughness":{{"baseColorFactor":[{},{},{},1],"metallicFactor":0,"roughnessFactor":0.5}}}}"#,
                json_string(&format!("color_{:02x}{:02x}{:02x}", rgb[0], rgb[1], rgb[2])),
                json_number(linear[0])?,
                json_number(linear[1])?,
                json_number(linear[2])?
            ))
        })
        .collect::<Result<Vec<String>, String>>()?;

    // The root: the axis rotation and the unit scale, in front of every solid.
    let children: Vec<String> = (1..=nodes.len()).map(|index| index.to_string()).collect();
    let root = format!(
        r#"{{"name":{},"rotation":{},"scale":[{s},{s},{s}],"children":[{}]}}"#,
        json_string(document_name),
        json_numbers(&up.node_rotation())?,
        children.join(","),
        s = json_number(unit_scale)?
    );
    nodes.insert(0, root);

    let mut json = String::new();
    json.push_str(r#"{"asset":{"version":"2.0","generator":"BREP glTF 2.0 writer"}"#);
    json.push_str(&format!(
        r#","scene":0,"scenes":[{{"name":{},"nodes":[0]}}]"#,
        json_string(document_name)
    ));
    json.push_str(&format!(r#","nodes":[{}]"#, nodes.join(",")));
    json.push_str(&format!(r#","meshes":[{}]"#, meshes.join(",")));
    if !materials.is_empty() {
        json.push_str(&format!(r#","materials":[{}]"#, materials.join(",")));
    }
    json.push_str(&format!(r#","accessors":[{}]"#, accessors.join(",")));
    json.push_str(&format!(r#","bufferViews":[{}]"#, buffer_views.join(",")));
    json.push_str(&format!(r#","buffers":[{{"byteLength":{}}}]"#, bin.len()));
    json.push('}');

    // Both chunks pad to 4 bytes: the JSON with SPACES, the binary with ZEROS.
    let mut json_bytes = json.into_bytes();
    while json_bytes.len() % 4 != 0 {
        json_bytes.push(b' ');
    }
    while bin.len() % 4 != 0 {
        bin.push(0);
    }

    let total = 12 + 8 + json_bytes.len() + 8 + bin.len();
    let mut out = Vec::with_capacity(total);
    push_u32(&mut out, GLB_MAGIC);
    push_u32(&mut out, 2);
    push_u32(&mut out, total as u32);
    push_u32(&mut out, json_bytes.len() as u32);
    push_u32(&mut out, CHUNK_JSON);
    out.extend_from_slice(&json_bytes);
    push_u32(&mut out, bin.len() as u32);
    push_u32(&mut out, CHUNK_BIN);
    out.extend_from_slice(&bin);
    report.bytes = out.len();
    Ok((out, report))
}

