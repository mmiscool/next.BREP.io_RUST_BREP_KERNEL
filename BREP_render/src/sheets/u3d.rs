//! A U3D (ECMA-363) writer — the 3D stream a PDF `/3D` annotation carries.
//!
//! Hand-rolled, no crate, wasm-clean, like the PDF writer beside it. It writes
//! exactly what a 3D PDF of a part needs and nothing else: MODEL nodes, each
//! owning either a triangle MESH (a CLOD mesh generator whose base mesh is the
//! whole mesh — no progressive resolution updates) or a LINE SET, one lit
//! texture shader and one material per colour, and a shading modifier per node
//! choosing them. No lights, cameras, textures, bones or animation: the PDF's
//! own view dictionaries carry the cameras and the lighting scheme.
//!
//! # The bit encoder
//!
//! Every block payload goes through the U3D bitstream (ECMA-363 §10): a 16-bit
//! arithmetic coder with static (uniform) and dynamic (adaptive) contexts.
//! Until the first compressed value, an uncompressed U8 leaves the coder in its
//! initial state and is emitted as its own eight bits, so a block holding only
//! uncompressed fields is plain little-endian bytes. [`BitWriter`] follows the
//! Intel reference implementation (`CIFXBitStreamX`, Apache-2.0) operation for
//! operation — the order of the renormalisation steps, when a histogram
//! halves, which contexts a field shares — because the reader on the other end
//! of a PDF is descended from that code, and an arithmetic coder that is
//! "equivalent" but not identical decodes to garbage. [`BitReader`] is the
//! matching decoder, kept for the round-trip tests.
//!
//! # The file
//!
//! The block order is the reference converter's own for the same content: the
//! file header, a priority update, a NODE modifier chain per model (the model
//! node and its shading modifier), a MODEL RESOURCE chain per model (the
//! generator's declaration, and — for a line set — its continuation), the
//! shaders, the materials, a second priority update, then each mesh's base-mesh
//! continuation.

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Block types (ECMA-363 §9)
// ---------------------------------------------------------------------------

const FILE_HEADER: u32 = 0x0044_3355;
const PRIORITY_UPDATE: u32 = 0xFFFF_FF15;
const MODIFIER_CHAIN: u32 = 0xFFFF_FF14;
const MODEL_NODE: u32 = 0xFFFF_FF22;
const CLOD_MESH_DECLARATION: u32 = 0xFFFF_FF31;
const CLOD_BASE_MESH_CONTINUATION: u32 = 0xFFFF_FF3B;
const LINE_SET_DECLARATION: u32 = 0xFFFF_FF37;
const LINE_SET_CONTINUATION: u32 = 0xFFFF_FF3F;
const SHADING_MODIFIER: u32 = 0xFFFF_FF45;
const LIT_TEXTURE_SHADER: u32 = 0xFFFF_FF53;
const MATERIAL_RESOURCE: u32 = 0xFFFF_FF54;

/// Modifier chain types.
const CHAIN_NODE: u32 = 0;
const CHAIN_MODEL_RESOURCE: u32 = 1;

/// The file header's character encoding: UTF-8 (the IANA MIBenum).
const UTF8: u32 = 106;

// ---------------------------------------------------------------------------
// Compression contexts — the reference implementation's numbering
// (`IFXACContext.h`). A context is a histogram; two fields that share a number
// share a histogram, so these are part of the format, not a local choice.
// ---------------------------------------------------------------------------

/// The static context of range `r`: every value `0..r` equally likely.
const STATIC_FULL: u32 = 0x400;
/// One past the largest static context: a range of `0x3FFF` or more is
/// written as an uncompressed value instead.
const MAX_RANGE: u32 = STATIC_FULL + 0x3FFF;
/// The context an uncompressed U8 is written through once the coder is live.
const CONTEXT_8: u32 = 0;

const CTX_BASE_SHADING_ID: u32 = 1;
const CTX_NUM_NEW_FACES: u32 = 1;
const CTX_LINE_SHADING_ID: u32 = 1;
const CTX_POSITION_DIFF_SIGNS: u32 = 20;
const CTX_POSITION_DIFF_MAG_X: u32 = 21;
const CTX_POSITION_DIFF_MAG_Y: u32 = 22;
const CTX_POSITION_DIFF_MAG_Z: u32 = 23;
const CTX_NUM_LOCAL_NORMALS: u32 = 40;
const CTX_NORMAL_DIFF_SIGNS: u32 = 41;
const CTX_NORMAL_DIFF_MAG_X: u32 = 42;
const CTX_NORMAL_DIFF_MAG_Y: u32 = 43;
const CTX_NORMAL_DIFF_MAG_Z: u32 = 44;
const CTX_NORMAL_LOCAL_INDEX: u32 = 45;

/// The static context for an index into an array of `count` entries.
fn static_range(count: u32) -> u32 {
    STATIC_FULL + count
}

/// A dynamic histogram halves every count once it has seen this many symbols
/// (the reference's `m_uElephant`), then gives the escape symbol one back.
const ELEPHANT: u32 = 0x1FFF;
/// Symbols above this are never added to a histogram.
const MAX_SYMBOL_IN_HISTOGRAM: u32 = 0xFFFF;

// ---------------------------------------------------------------------------
// The scene
// ---------------------------------------------------------------------------

/// A surface colour: one lit texture shader and one material in the file.
#[derive(Debug, Clone, PartialEq)]
pub struct Material {
    pub name: String,
    pub ambient: [f32; 3],
    pub diffuse: [f32; 3],
    pub specular: [f32; 3],
    pub emissive: [f32; 3],
    pub reflectivity: f32,
    pub opacity: f32,
}

/// A triangle mesh. `normals[i]` is the normal at `positions[i]`; each
/// triangle carries the index of the [`Model::shaders`] entry that colours it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Mesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub triangles: Vec<([u32; 3], u32)>,
}

/// Line segments between `positions`, each with its [`Model::shaders`] index.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Lines {
    pub positions: Vec<[f32; 3]>,
    pub segments: Vec<([u32; 2], u32)>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Geometry {
    Mesh(Mesh),
    Lines(Lines),
}

/// One model node and the resource it shows. `name` is the node name a PDF
/// `/3DNode` dictionary refers to; the resource is named `{name}.res`.
#[derive(Debug, Clone, PartialEq)]
pub struct Model {
    pub name: String,
    pub geometry: Geometry,
    /// Material index per shading id (at least one).
    pub shaders: Vec<usize>,
    /// Draw back faces too — a lone triangle (an arrowhead) that must not
    /// vanish when the model is turned round. A closed solid does not need it.
    pub two_sided: bool,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Scene {
    pub materials: Vec<Material>,
    pub models: Vec<Model>,
}

/// Why a scene cannot be written.
pub fn validate(scene: &Scene) -> Result<(), String> {
    let mut names = std::collections::HashSet::new();
    for model in &scene.models {
        if model.name.is_empty() || model.name.len() > 0xFFFF {
            return Err(format!("model name '{}' is empty or too long", model.name));
        }
        if !names.insert(model.name.as_str()) {
            return Err(format!("two models are named '{}'", model.name));
        }
        if model.shaders.is_empty() {
            return Err(format!("model '{}' has no shader", model.name));
        }
        if let Some(bad) = model.shaders.iter().find(|index| **index >= scene.materials.len()) {
            return Err(format!("model '{}' names material {bad}, which does not exist", model.name));
        }
        let shading = model.shaders.len() as u32;
        match &model.geometry {
            Geometry::Mesh(mesh) => {
                if mesh.positions.is_empty() || mesh.triangles.is_empty() {
                    return Err(format!("mesh '{}' is empty", model.name));
                }
                if mesh.normals.len() != mesh.positions.len() {
                    return Err(format!("mesh '{}' has a normal count unlike its position count", model.name));
                }
                let n = mesh.positions.len() as u32;
                if mesh.triangles.iter().any(|(t, s)| t.iter().any(|i| *i >= n) || *s >= shading) {
                    return Err(format!("mesh '{}' indexes past its arrays", model.name));
                }
            }
            Geometry::Lines(lines) => {
                if lines.positions.is_empty() || lines.segments.is_empty() {
                    return Err(format!("line set '{}' is empty", model.name));
                }
                let n = lines.positions.len() as u32;
                if lines.segments.iter().any(|(s, m)| s.iter().any(|i| *i >= n) || s[0] == s[1] || *m >= shading) {
                    return Err(format!("line set '{}' has a degenerate or out-of-range segment", model.name));
                }
            }
        }
    }
    for material in &scene.materials {
        if material.name.is_empty() || material.name.len() > 0xFFFF {
            return Err(format!("material name '{}' is empty or too long", material.name));
        }
    }
    Ok(())
}

/// Serialise `scene` as a U3D file.
pub fn write(scene: &Scene) -> Result<Vec<u8>, String> {
    validate(scene)?;
    let mut blocks: Vec<u8> = Vec::new();

    push_block(&mut blocks, PRIORITY_UPDATE, &uncompressed(|w| w.write_u32(0)));
    for model in &scene.models {
        let node = block_bytes(MODEL_NODE, &model_node(model));
        let shading = block_bytes(SHADING_MODIFIER, &shading_modifier(model, scene));
        push_block(&mut blocks, MODIFIER_CHAIN, &modifier_chain(&model.name, CHAIN_NODE, &[node, shading]));
    }
    for model in &scene.models {
        let resource = resource_name(model);
        let members = match &model.geometry {
            Geometry::Mesh(mesh) => vec![block_bytes(CLOD_MESH_DECLARATION, &mesh_declaration(&resource, mesh, model))],
            Geometry::Lines(lines) => {
                let quant = line_quantisation(lines);
                vec![
                    block_bytes(LINE_SET_DECLARATION, &line_declaration(&resource, lines, model, &quant)),
                    block_bytes(LINE_SET_CONTINUATION, &line_continuation(&resource, lines, &quant)),
                ]
            }
        };
        push_block(&mut blocks, MODIFIER_CHAIN, &modifier_chain(&resource, CHAIN_MODEL_RESOURCE, &members));
    }
    for material in &scene.materials {
        push_block(&mut blocks, LIT_TEXTURE_SHADER, &lit_texture_shader(material));
    }
    for material in &scene.materials {
        push_block(&mut blocks, MATERIAL_RESOURCE, &material_resource(material));
    }
    push_block(&mut blocks, PRIORITY_UPDATE, &uncompressed(|w| w.write_u32(0x100)));
    for model in &scene.models {
        if let Geometry::Mesh(mesh) = &model.geometry {
            push_block(&mut blocks, CLOD_BASE_MESH_CONTINUATION, &base_mesh(&resource_name(model), mesh));
        }
    }

    // The header last, now the file's size is known. Its declaration size is
    // the header block's own 36 bytes — what the reference writer puts there.
    const HEADER_BLOCK: u64 = 12 + 24;
    let file_size = HEADER_BLOCK + blocks.len() as u64;
    let header = uncompressed(|w| {
        w.write_u16(0); // major version
        w.write_u16(0); // minor version
        w.write_u32(0); // profile: base profile, compression on, no units
        w.write_u32(HEADER_BLOCK as u32);
        w.write_u64(file_size);
        w.write_u32(UTF8);
    });
    let mut out = Vec::with_capacity(file_size as usize);
    push_block(&mut out, FILE_HEADER, &header);
    out.extend_from_slice(&blocks);
    Ok(out)
}

fn resource_name(model: &Model) -> String {
    format!("{}.res", model.name)
}

/// A block: type, data size, metadata size (always 0 here), the data padded
/// to four bytes.
fn push_block(out: &mut Vec<u8>, block_type: u32, data: &[u8]) {
    out.extend_from_slice(&block_type.to_le_bytes());
    out.extend_from_slice(&(data.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(data);
    out.resize(out.len() + (4 - data.len() % 4) % 4, 0);
}

fn block_bytes(block_type: u32, data: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    push_block(&mut out, block_type, data);
    out
}

fn uncompressed(body: impl FnOnce(&mut BitWriter)) -> Vec<u8> {
    let mut w = BitWriter::default();
    body(&mut w);
    w.finish()
}

/// A modifier chain holding `members` (each already a whole block). The
/// member count sits on a four-byte boundary of the chain's data.
fn modifier_chain(name: &str, chain_type: u32, members: &[Vec<u8>]) -> Vec<u8> {
    let mut head = uncompressed(|w| {
        w.write_string(name);
        w.write_u32(chain_type);
        w.write_u32(0); // attributes: no bounding sphere, no bounding box
    });
    head.resize(head.len() + (4 - head.len() % 4) % 4, 0);
    head.extend_from_slice(&(members.len() as u32).to_le_bytes());
    for member in members {
        head.extend_from_slice(member);
    }
    head
}

const IDENTITY: [f32; 16] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

fn model_node(model: &Model) -> Vec<u8> {
    uncompressed(|w| {
        w.write_string(&model.name);
        w.write_u32(1); // one parent: the world
        w.write_string("");
        for value in IDENTITY {
            w.write_f32(value);
        }
        w.write_string(&resource_name(model));
        // Visibility: front faces (the reference's value), or front and back.
        w.write_u32(if model.two_sided { 3 } else { 1 });
    })
}

fn shading_modifier(model: &Model, scene: &Scene) -> Vec<u8> {
    uncompressed(|w| {
        w.write_string(&model.name);
        w.write_u32(1); // chain index: after the model node
        w.write_u32(0x0F); // applies to meshes, lines, points and glyphs
        w.write_u32(model.shaders.len() as u32);
        for material in &model.shaders {
            w.write_u32(1); // one shader in this shader list
            w.write_string(&scene.materials[*material].name);
        }
    })
}

/// The shading descriptions: every shader list uses positions and normals
/// only — no vertex colours, no texture layers.
fn write_shading_descriptions(w: &mut BitWriter, count: usize) {
    w.write_u32(count as u32);
    for index in 0..count {
        w.write_u32(0); // attributes: no per-vertex colours
        w.write_u32(0); // texture layer count
        w.write_u32(index as u32); // original shading id
    }
}

fn mesh_declaration(resource: &str, mesh: &Mesh, model: &Model) -> Vec<u8> {
    uncompressed(|w| {
        w.write_string(resource);
        w.write_u32(0); // chain index
        w.write_u32(0); // mesh attributes: normals present
        w.write_u32(mesh.triangles.len() as u32);
        w.write_u32(mesh.positions.len() as u32);
        w.write_u32(mesh.normals.len() as u32);
        w.write_u32(0); // diffuse colours
        w.write_u32(0); // specular colours
        w.write_u32(0); // texture coordinates
        write_shading_descriptions(w, model.shaders.len());
        // CLOD: the base mesh is the whole mesh.
        w.write_u32(mesh.positions.len() as u32);
        w.write_u32(mesh.positions.len() as u32);
        // Quality factors (informative) and inverse quantisation: the base
        // mesh is not quantised, so these are the reference's defaults.
        for _ in 0..3 {
            w.write_u32(1000);
        }
        for _ in 0..5 {
            w.write_f32(1.0);
        }
        // Normal crease, update and tolerance: merge nothing.
        w.write_f32(-1.0);
        w.write_f32(0.0);
        w.write_f32(0.0);
        w.write_u32(0); // bones
    })
}

fn base_mesh(resource: &str, mesh: &Mesh) -> Vec<u8> {
    let mut w = BitWriter::default();
    w.write_string(resource);
    w.write_u32(0); // chain index
    w.write_u32(mesh.triangles.len() as u32);
    w.write_u32(mesh.positions.len() as u32);
    w.write_u32(mesh.normals.len() as u32);
    w.write_u32(0);
    w.write_u32(0);
    w.write_u32(0);
    for p in &mesh.positions {
        for c in p {
            w.write_f32(*c);
        }
    }
    for n in &mesh.normals {
        for c in n {
            w.write_f32(*c);
        }
    }
    let positions = static_range(mesh.positions.len() as u32);
    let normals = static_range(mesh.normals.len() as u32);
    for (corners, shading) in &mesh.triangles {
        w.write_compressed_u32(CTX_BASE_SHADING_ID, *shading);
        for corner in corners {
            w.write_compressed_u32(positions, *corner);
            w.write_compressed_u32(normals, *corner);
        }
    }
    w.finish()
}

/// A line set's position quantisation (the reference's rule at quality
/// 1000): `2^18` steps across the bounding radius, capped so no coordinate
/// overflows a U32.
struct LineQuant {
    position: f32,
    normal: f32,
}

fn line_quantisation(lines: &Lines) -> LineQuant {
    let mut min = lines.positions[0];
    let mut max = lines.positions[0];
    for p in &lines.positions {
        for axis in 0..3 {
            min[axis] = min[axis].min(p[axis]);
            max[axis] = max[axis].max(p[axis]);
        }
    }
    let centre = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5, (min[2] + max[2]) * 0.5];
    let radius_squared = lines
        .positions
        .iter()
        .map(|p| (p[0] - centre[0]).powi(2) + (p[1] - centre[1]).powi(2) + (p[2] - centre[2]).powi(2))
        .fold(0.0f32, f32::max);
    let mut position = 262_144.0f32;
    if radius_squared > 0.0 {
        position /= radius_squared.sqrt();
    }
    let largest = min.iter().chain(max.iter()).map(|c| c.abs()).fold(0.0f32, f32::max);
    if largest > 0.0 {
        position = position.min(0xFFFF_FE00u32 as f32 / largest);
    }
    LineQuant { position, normal: 16_384.0 }
}

fn line_declaration(resource: &str, lines: &Lines, model: &Model, quant: &LineQuant) -> Vec<u8> {
    uncompressed(|w| {
        w.write_string(resource);
        w.write_u32(0); // chain index
        w.write_u32(0); // reserved
        w.write_u32(lines.segments.len() as u32);
        w.write_u32(lines.positions.len() as u32);
        w.write_u32(1); // one normal, shared by every line end
        w.write_u32(0);
        w.write_u32(0);
        w.write_u32(0);
        write_shading_descriptions(w, model.shaders.len());
        for _ in 0..3 {
            w.write_u32(0); // quality factors, as the reference writes them
        }
        w.write_f32(1.0 / quant.position);
        w.write_f32(1.0 / quant.normal);
        w.write_f32(1.0 / 16_384.0); // texture coordinates
        w.write_f32(1.0 / 16_384.0); // diffuse colours
        w.write_f32(1.0 / 16_384.0); // specular colours
        for _ in 0..3 {
            w.write_u32(0); // reserved line set parameters
        }
        w.write_u32(0); // bones
    })
}

/// The single line-end normal: every line end uses it, so every reconstructed
/// normal is exactly it and the predicted normal is either it or zero.
const LINE_NORMAL: [f32; 3] = [0.0, 0.0, 1.0];

/// Signs and magnitudes of a quantised difference (ECMA-363 §5.3.3).
fn quantise(v: [f32; 3], factor: f32) -> (u8, [u32; 3]) {
    let signs = (v[0] < 0.0) as u8 | ((v[1] < 0.0) as u8) << 1 | ((v[2] < 0.0) as u8) << 2;
    let mag = |c: f32| (0.5f32 + factor * c.abs()) as u32;
    (signs, [mag(v[0]), mag(v[1]), mag(v[2])])
}

fn reconstruct(predicted: [f32; 3], signs: u8, mag: [u32; 3], inverse: f32) -> [f32; 3] {
    let mut out = predicted;
    for axis in 0..3 {
        let step = inverse * mag[axis] as f32;
        out[axis] = if signs & (1 << axis) != 0 { predicted[axis] - step } else { predicted[axis] + step };
    }
    out
}

/// The line set's data, one block (a block of 4096 positions or more would be
/// split by the reference; one block reads the same and a PMI set is far
/// smaller).
///
/// Each position `i` is written as a difference from position `i − 1` (the
/// SPLIT position) and brings every segment whose other end is an earlier
/// position. The prediction is the RECONSTRUCTED previous position — what the
/// reader has — so quantisation error does not accumulate along the set.
fn line_continuation(resource: &str, lines: &Lines, quant: &LineQuant) -> Vec<u8> {
    let count = lines.positions.len() as u32;
    // Segments by their LATER end, with the earlier end: that is when each is
    // written.
    let mut by_later: Vec<Vec<(u32, u32)>> = vec![Vec::new(); lines.positions.len()];
    for ([a, b], shading) in &lines.segments {
        let (early, late) = if a < b { (*a, *b) } else { (*b, *a) };
        by_later[late as usize].push((early, *shading));
    }
    // Which positions have a segment already written — the predicted normal
    // at a split position is the line normal once one exists, else zero.
    let mut has_line = vec![false; lines.positions.len()];

    let mut w = BitWriter::default();
    w.write_string(resource);
    w.write_u32(0); // chain index
    w.write_u32(0); // start resolution
    w.write_u32(count); // end resolution
    let inverse = 1.0 / quant.position;
    let mut previous = [0.0f32; 3];
    for current in 0..count {
        let position = lines.positions[current as usize];
        let (split_range, split, predicted) = if current == 0 {
            (static_range(1), 0, [0.0f32; 3])
        } else {
            (static_range(current), current - 1, previous)
        };
        w.write_compressed_u32(split_range, split);
        let diff = [position[0] - predicted[0], position[1] - predicted[1], position[2] - predicted[2]];
        let (signs, mag) = quantise(diff, quant.position);
        w.write_compressed_u8(CTX_POSITION_DIFF_SIGNS, signs);
        w.write_compressed_u32(CTX_POSITION_DIFF_MAG_X, mag[0]);
        w.write_compressed_u32(CTX_POSITION_DIFF_MAG_Y, mag[1]);
        w.write_compressed_u32(CTX_POSITION_DIFF_MAG_Z, mag[2]);
        previous = reconstruct(predicted, signs, mag, inverse);
        if current == 0 {
            w.write_compressed_u32(CTX_NUM_LOCAL_NORMALS, 0);
            w.write_compressed_u32(CTX_NUM_NEW_FACES, 0);
            continue;
        }
        let new_lines = &by_later[current as usize];
        let predicted_normal = if has_line[split as usize] { LINE_NORMAL } else { [0.0; 3] };
        w.write_compressed_u32(CTX_NUM_LOCAL_NORMALS, 2 * new_lines.len() as u32);
        let normal_diff = [
            LINE_NORMAL[0] - predicted_normal[0],
            LINE_NORMAL[1] - predicted_normal[1],
            LINE_NORMAL[2] - predicted_normal[2],
        ];
        let (normal_signs, normal_mag) = quantise(normal_diff, quant.normal);
        for _ in 0..new_lines.len() * 2 {
            w.write_compressed_u8(CTX_NORMAL_DIFF_SIGNS, normal_signs);
            w.write_compressed_u32(CTX_NORMAL_DIFF_MAG_X, normal_mag[0]);
            w.write_compressed_u32(CTX_NORMAL_DIFF_MAG_Y, normal_mag[1]);
            w.write_compressed_u32(CTX_NORMAL_DIFF_MAG_Z, normal_mag[2]);
        }
        w.write_compressed_u32(CTX_NUM_NEW_FACES, new_lines.len() as u32);
        for (index, (early, shading)) in new_lines.iter().enumerate() {
            w.write_compressed_u32(CTX_LINE_SHADING_ID, *shading);
            w.write_compressed_u32(static_range(current), *early);
            for end in 0..2 {
                w.write_compressed_u32(CTX_NORMAL_LOCAL_INDEX, 2 * index as u32 + end);
            }
            has_line[*early as usize] = true;
        }
        if !new_lines.is_empty() {
            has_line[current as usize] = true;
        }
    }
    w.finish()
}

fn lit_texture_shader(material: &Material) -> Vec<u8> {
    uncompressed(|w| {
        w.write_string(&material.name);
        w.write_u32(1); // lighting enabled
        w.write_f32(0.0); // alpha test reference
        w.write_u32(0x617); // alpha test: always
        w.write_u32(0x606); // colour blend: alpha blend
        w.write_u32(1); // render pass 0 enabled
        w.write_u32(0); // no texture channels
        w.write_u32(0); // no alpha texture channels
        w.write_string(&material.name);
    })
}

fn material_resource(material: &Material) -> Vec<u8> {
    uncompressed(|w| {
        w.write_string(&material.name);
        w.write_u32(0x3F); // every colour, reflectivity and opacity present
        for colour in [material.ambient, material.diffuse, material.specular, material.emissive] {
            for c in colour {
                w.write_f32(c);
            }
        }
        w.write_f32(material.reflectivity);
        w.write_f32(material.opacity);
    })
}

// ---------------------------------------------------------------------------
// The bitstream
// ---------------------------------------------------------------------------

/// A dynamic histogram: symbol 0 is the escape, starting at frequency 1.
/// Cumulative frequencies come from a Fenwick tree, because a magnitude
/// context can hold symbols up to `0xFFFF` and a linear scan per symbol would
/// make a large line set quadratic.
#[derive(Debug, Clone)]
struct Histogram {
    counts: Vec<u32>,
    tree: Vec<u32>,
    total: u32,
}

impl Histogram {
    fn new() -> Self {
        let mut h = Histogram { counts: vec![0; 128], tree: vec![0; 129], total: 0 };
        h.bump(0, 1);
        h
    }

    fn bump(&mut self, symbol: usize, by: u32) {
        if symbol >= self.counts.len() {
            let mut size = self.counts.len();
            while size <= symbol {
                size *= 2;
            }
            let counts = std::mem::take(&mut self.counts);
            self.counts = vec![0; size];
            self.tree = vec![0; size + 1];
            self.total = 0;
            for (s, c) in counts.into_iter().enumerate() {
                if c > 0 {
                    self.bump(s, c);
                }
            }
        }
        self.counts[symbol] += by;
        self.total += by;
        let mut i = symbol + 1;
        while i < self.tree.len() {
            self.tree[i] += by;
            i += i & i.wrapping_neg();
        }
    }

    fn freq(&self, symbol: u32) -> u32 {
        self.counts.get(symbol as usize).copied().unwrap_or(0)
    }

    /// The summed frequency of every symbol below `symbol`.
    fn cum(&self, symbol: u32) -> u32 {
        let mut i = (symbol as usize).min(self.counts.len());
        let mut sum = 0;
        while i > 0 {
            sum += self.tree[i];
            i -= i & i.wrapping_neg();
        }
        sum
    }

    /// The symbol whose cumulative interval holds `target`.
    fn symbol_at(&self, target: u32) -> u32 {
        // The largest prefix whose sum is ≤ target.
        let mut position = 0usize;
        let mut remaining = target;
        let mut step = self.tree.len().next_power_of_two() / 2;
        while step > 0 {
            let next = position + step;
            if next < self.tree.len() && self.tree[next] <= remaining {
                position = next;
                remaining -= self.tree[next];
            }
            step /= 2;
        }
        position as u32
    }

    fn add(&mut self, symbol: u32) {
        if symbol > MAX_SYMBOL_IN_HISTOGRAM {
            return;
        }
        if self.total >= ELEPHANT {
            let halved: Vec<u32> = self.counts.iter().map(|c| c >> 1).collect();
            self.tree.iter_mut().for_each(|t| *t = 0);
            self.counts.iter_mut().for_each(|c| *c = 0);
            self.total = 0;
            for (s, c) in halved.into_iter().enumerate() {
                if c > 0 {
                    self.bump(s, c);
                }
            }
            self.bump(0, 1);
        }
        self.bump(symbol as usize, 1);
    }
}

const HALF: u32 = 0x8000;
const QUARTER: u32 = 0x4000;

/// The U3D bit writer (see the module doc).
#[derive(Debug, Clone)]
struct BitWriter {
    bytes: Vec<u8>,
    bit: usize,
    high: u32,
    low: u32,
    underflow: u32,
    compressed: bool,
    contexts: HashMap<u32, Histogram>,
}

impl Default for BitWriter {
    fn default() -> Self {
        BitWriter {
            bytes: Vec::new(),
            bit: 0,
            high: 0xFFFF,
            low: 0,
            underflow: 0,
            compressed: false,
            contexts: HashMap::new(),
        }
    }
}

/// Reverse the bit order of a byte.
fn swap8(value: u32) -> u32 {
    (value as u8).reverse_bits() as u32
}

impl BitWriter {
    fn write_bit(&mut self, value: u32) {
        let byte = self.bit / 8;
        if byte == self.bytes.len() {
            self.bytes.push(0);
        }
        if value & 1 != 0 {
            self.bytes[byte] |= 1 << (self.bit % 8);
        }
        self.bit += 1;
    }

    fn initial(&self) -> bool {
        self.high == 0xFFFF && self.low == 0 && self.underflow == 0
    }

    fn write_u8(&mut self, value: u8) {
        let symbol = swap8(value as u32);
        if self.initial() {
            for i in 0..8 {
                self.write_bit((value as u32) >> i);
            }
        } else {
            self.write_static(STATIC_FULL + 256, symbol + 1);
        }
    }

    fn write_u16(&mut self, value: u16) {
        self.write_u8(value as u8);
        self.write_u8((value >> 8) as u8);
    }

    fn write_u32(&mut self, value: u32) {
        self.write_u16(value as u16);
        self.write_u16((value >> 16) as u16);
    }

    fn write_u64(&mut self, value: u64) {
        self.write_u32(value as u32);
        self.write_u32((value >> 32) as u32);
    }

    fn write_f32(&mut self, value: f32) {
        self.write_u32(value.to_bits());
    }

    fn write_string(&mut self, text: &str) {
        self.write_u16(text.len() as u16);
        for byte in text.bytes() {
            self.write_u8(byte);
        }
    }

    /// Narrow the interval to `[cum, cum + freq)` of `total`, then emit every
    /// settled bit — the reference's renormalisation, step for step.
    fn encode(&mut self, cum: u32, freq: u32, total: u32) {
        let range = self.high + 1 - self.low;
        // `low - 1` wraps when low is 0, exactly as the reference's U32 does;
        // the sum lands back in range.
        self.high = self.low.wrapping_sub(1).wrapping_add(range * (cum + freq) / total);
        self.low += range * cum / total;
        let mut bit = self.low >> 15;
        while (self.high & HALF) == (self.low & HALF) {
            self.high &= !HALF;
            self.high += self.high + 1;
            self.write_bit(bit);
            while self.underflow > 0 {
                self.underflow -= 1;
                self.write_bit(!bit & 1);
            }
            self.low &= !HALF;
            self.low += self.low;
            bit = self.low >> 15;
        }
        while (self.high & QUARTER) == 0 && (self.low & QUARTER) == QUARTER {
            self.high &= !HALF;
            self.high <<= 1;
            self.low <<= 1;
            self.high |= HALF;
            self.high |= 1;
            self.low &= !HALF;
            self.underflow += 1;
        }
    }

    /// A symbol `1..=R` in the static context `STATIC_FULL + R`.
    fn write_static(&mut self, context: u32, symbol: u32) {
        let total = context - STATIC_FULL;
        debug_assert!(symbol >= 1 && symbol <= total);
        self.encode(symbol - 1, 1, total);
    }

    /// A symbol in a dynamic context; true when the escape was written
    /// instead.
    fn write_dynamic(&mut self, context: u32, symbol: u32) -> bool {
        let histogram = self.contexts.entry(context).or_insert_with(Histogram::new);
        let (mut symbol, total) = (symbol, histogram.total);
        let mut freq = histogram.freq(symbol);
        if freq == 0 {
            symbol = 0;
            freq = histogram.freq(0);
        }
        let cum = histogram.cum(symbol);
        histogram.add(symbol);
        self.encode(cum, freq, total);
        symbol == 0
    }

    fn write_compressed(&mut self, context: u32, value: u32, raw: impl Fn(&mut Self, u32)) {
        self.compressed = true;
        if context != CONTEXT_8 && context < MAX_RANGE {
            let escaped = if context > STATIC_FULL {
                self.write_static(context, value + 1);
                false
            } else {
                self.write_dynamic(context, value + 1)
            };
            if escaped {
                raw(self, value);
                if let Some(histogram) = self.contexts.get_mut(&context) {
                    histogram.add(value + 1);
                }
            }
        } else {
            raw(self, value);
        }
    }

    fn write_compressed_u32(&mut self, context: u32, value: u32) {
        self.write_compressed(context, value, |w, v| w.write_u32(v));
    }

    fn write_compressed_u8(&mut self, context: u32, value: u8) {
        self.write_compressed(context, value as u32, |w, v| w.write_u8(v as u8));
    }

    /// The block's bytes: a compressed block is flushed with an uncompressed
    /// U32 zero so every bit the reader needs is written.
    fn finish(mut self) -> Vec<u8> {
        if self.compressed {
            self.write_u32(0);
        }
        self.bytes.truncate(self.bit.div_ceil(8));
        self.bytes
    }
}



