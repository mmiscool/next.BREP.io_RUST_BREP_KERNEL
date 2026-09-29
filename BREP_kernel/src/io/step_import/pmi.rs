//! AP242 PMI on import — the read side of `io/step/pmi.rs`.
//!
//! Lifts a file's SEMANTIC PMI (dimensional locations / sizes / angular
//! locations with their values and ± / limit tolerances, datums, geometric
//! tolerances with datum systems, modifiers and zones), its SAVED VIEWS
//! (`DRAUGHTING_MODEL` + `CAMERA_MODEL_D3`) and the label positions of its
//! polyline presentation into a [`PmiState`] whose references name the faces
//! / edges / vertices the IMPORT3D feature will stamp — so PMI written by
//! this kernel, or by any AP242 CAD system, arrives as live, editable
//! annotations rather than dead geometry.
//!
//! Reference mapping: face `i` of built body `b` IS `#face_refs[b][i]` (the
//! importer's own pairing guarantee), so an `ADVANCED_FACE` maps to the
//! positional name `{body}_Face_{i}` (`{feature}_SOLID_0k` bodies for a
//! multi-body file). Edges map through their two adjacent faces to the
//! derived `{A}|{B}[n]` name the pipeline stamps; vertices to `{body}@x,y,z`.
//! 3D text callouts with no semantic element import as notes. Annotations no
//! saved view lists land in an "Imported PMI" view.

use super::bodies::{collect_step_solids, step_body_claimed_shells, StepBody};
use super::parse::{parse_data_section, Entity, Value};
use super::*;
use crate::feature_pipeline::pmi::annotations::fcf::characteristic_for_entity;
use crate::feature_pipeline::pmi::{
    PmiAnnotation, PmiCamera, PmiDisplay, PmiProjection, PmiState, PmiView,
};
use std::collections::BTreeMap;

/// Decode Part 21 string directives: `\X2\HHHH…\X0\` (UTF-16 code units),
/// `\X4\HHHHHHHH…\X0\`, `\X\HH` (ISO 8859-1), `\S\c`, `\N\` (newline).
pub(crate) fn decode_step_text(raw: &str) -> String {
    let mut out = String::new();
    let mut rest = raw;
    while let Some(index) = rest.find('\\') {
        out.push_str(&rest[..index]);
        let tail = &rest[index..];
        if let Some(after) = tail.strip_prefix("\\X2\\") {
            let end = after.find("\\X0\\").unwrap_or(after.len());
            let hex = &after[..end];
            let units: Vec<u16> = hex
                .as_bytes()
                .chunks(4)
                .filter_map(|chunk| u16::from_str_radix(std::str::from_utf8(chunk).ok()?, 16).ok())
                .collect();
            out.push_str(&String::from_utf16_lossy(&units));
            rest = &after[(end + 4).min(after.len())..];
        } else if let Some(after) = tail.strip_prefix("\\X4\\") {
            let end = after.find("\\X0\\").unwrap_or(after.len());
            let hex = &after[..end];
            for chunk in hex.as_bytes().chunks(8) {
                if let Some(ch) = std::str::from_utf8(chunk)
                    .ok()
                    .and_then(|text| u32::from_str_radix(text, 16).ok())
                    .and_then(char::from_u32)
                {
                    out.push(ch);
                }
            }
            rest = &after[(end + 4).min(after.len())..];
        } else if let Some(after) = tail.strip_prefix("\\X\\") {
            if let Some(byte) = after.get(..2).and_then(|hex| u8::from_str_radix(hex, 16).ok()) {
                out.push(byte as char);
                rest = &after[2..];
            } else {
                rest = after;
            }
        } else if let Some(after) = tail.strip_prefix("\\N\\") {
            out.push('\n');
            rest = after;
        } else if let Some(after) = tail.strip_prefix("\\S\\") {
            let mut chars = after.chars();
            if let Some(ch) = chars.next() {
                out.push(((ch as u32 + 128) as u8) as char);
            }
            rest = chars.as_str();
        } else if let Some(after) = tail.strip_prefix("\\\\") {
            out.push('\\');
            rest = after;
        } else {
            out.push('\\');
            rest = &tail[1..];
        }
    }
    out.push_str(rest);
    out
}

fn arg_string(args: &[Value], index: usize) -> String {
    match args.get(index) {
        Some(Value::Str(text)) => decode_step_text(text),
        _ => String::new(),
    }
}

fn arg_ref(args: &[Value], index: usize) -> Option<usize> {
    args.get(index).and_then(|value| value.as_ref_id().ok())
}

fn arg_refs(args: &[Value], index: usize) -> Vec<usize> {
    args.get(index)
        .and_then(|value| value.as_list().ok())
        .map(|items| items.iter().filter_map(|item| item.as_ref_id().ok()).collect())
        .unwrap_or_default()
}

/// The numeric payload of a measure select (`LENGTH_MEASURE(0.1)`,
/// `PLANE_ANGLE_MEASURE(1.5)`) or a bare real.
fn measure_value(value: &Value) -> Option<f64> {
    match value {
        Value::Typed(_, inner) => inner.first().and_then(measure_value),
        other => other.as_real().ok(),
    }
}

/// What a reference name opens with when the file named geometry this
/// importer did not mint: the STEP entity id of the shape aspect that named
/// it. Nothing resolves it, which is the point — the annotation reaches the
/// PMI panel as a row in error, naming the entity a reader can look up in the
/// file, instead of vanishing between the two formats.
pub(super) const UNRESOLVED_REF_PREFIX: &str = "STEP#";

/// The geometry a reference name points at, as the IMPORT3D feature names it.
pub(super) struct NameMap {
    faces: HashMap<usize, String>,
    edges: HashMap<usize, String>,
    vertices: HashMap<usize, String>,
    pub(super) length_scale: f64,
    /// Degrees per file plane-angle unit.
    angle_to_degrees: f64,
}

/// Degrees per plane-angle unit of the file (`SI_UNIT($,.RADIAN.)` →
/// 57.29…; a `CONVERSION_BASED_UNIT` of 0.01745 rad → 1).
fn angle_scale_degrees(entities: &HashMap<usize, Entity>) -> f64 {
    for entity in entities.values() {
        if !entity.has("PLANE_ANGLE_UNIT") {
            continue;
        }
        if let Some(args) = entity.find("CONVERSION_BASED_UNIT") {
            if let Some(measure) = arg_ref(args, 1).and_then(|id| entities.get(&id)) {
                if let Some(value) = measure
                    .find("PLANE_ANGLE_MEASURE_WITH_UNIT")
                    .or_else(|| measure.find("MEASURE_WITH_UNIT"))
                    .and_then(|args| args.first())
                    .and_then(measure_value)
                {
                    return value.to_degrees();
                }
            }
        }
    }
    1f64.to_degrees()
}

impl NameMap {
    /// The FLAT lane's map: every solid in the file, in file order, named by
    /// the IMPORT3D feature that is about to stamp them.
    fn build(text: &str, entities: &HashMap<usize, Entity>, feature_name: &str) -> Result<Self, String> {
        let imported = collect_step_solids(text)?;
        let scale = derive_length_scale_mm(entities);
        let resolver = Resolver::new(entities, scale);
        let count = imported.solids.len();
        let body_names = crate::feature_pipeline::imported_solid_names(count, feature_name);
        let mut map = Self {
            faces: HashMap::default(),
            edges: HashMap::default(),
            vertices: HashMap::default(),
            length_scale: scale,
            angle_to_degrees: angle_scale_degrees(entities),
        };
        map.add_bodies(entities, &resolver, &body_names, &imported.face_refs, "");
        Ok(map)
    }

    /// An EMPTY map at the file's global units — the base a structured lift
    /// adds one product at a time to ([`Self::add_bodies`]).
    pub(super) fn empty(entities: &HashMap<usize, Entity>, length_scale: f64) -> Self {
        Self {
            faces: HashMap::default(),
            edges: HashMap::default(),
            vertices: HashMap::default(),
            length_scale,
            angle_to_degrees: angle_scale_degrees(entities),
        }
    }

    /// Name one set of bodies — `face_refs[b]` is body `b`'s ordered face refs,
    /// `body_names[b]` the scene name the pipeline will stamp on it.
    ///
    /// `qualifier` is prepended to every name this call mints. The flat lane
    /// passes `""`; a STRUCTURED lift passes a per-product tag, because two
    /// part documents both call their first body `IMPORT3D1` and their names
    /// would otherwise collide in one map (`assembly_pmi.rs`).
    pub(super) fn add_bodies(
        &mut self,
        entities: &HashMap<usize, Entity>,
        resolver: &Resolver,
        body_names: &[String],
        face_refs: &[Vec<usize>],
        qualifier: &str,
    ) {
        for (body_index, body_faces) in face_refs.iter().enumerate() {
            let Some(body_name) = body_names.get(body_index) else {
                continue;
            };
            // Faces, positionally.
            for (face_index, face_ref) in body_faces.iter().enumerate() {
                self.faces
                    .entry(*face_ref)
                    .or_insert_with(|| format!("{qualifier}{body_name}_Face_{face_index}"));
            }
            // Edges: `{A}|{B}[n]` in first-encounter (face → loop → coedge)
            // order, with the two adjacent face names sorted — the pipeline's
            // `stamp_derived_edge_names` convention. The qualifier goes on the
            // FINISHED name, so the two face names it joins stay local.
            let mut adjacency: HashMap<usize, Vec<String>> = HashMap::default();
            let mut order: Vec<usize> = Vec::new();
            for (face_index, face_ref) in body_faces.iter().enumerate() {
                let face_name = format!("{body_name}_Face_{face_index}");
                for edge_ref in face_edge_refs(resolver, *face_ref) {
                    let uses = adjacency.entry(edge_ref).or_default();
                    if uses.is_empty() {
                        order.push(edge_ref);
                    }
                    uses.push(face_name.clone());
                    // Vertices, from the edge's ends.
                    if let Some(entity) = entities.get(&edge_ref) {
                        if let Some(args) = entity.find("EDGE_CURVE") {
                            for slot in [1usize, 2] {
                                if let Some(vertex_ref) = arg_ref(args, slot) {
                                    if let Ok(point) = resolver.step_vertex_point(vertex_ref) {
                                        self.vertices.entry(vertex_ref).or_insert_with(|| {
                                            format!("{qualifier}{body_name}@{},{},{}", trim_real(point.x), trim_real(point.y), trim_real(point.z))
                                        });
                                    }
                                }
                            }
                        }
                    }
                }
            }
            let mut counts: HashMap<String, usize> = HashMap::default();
            for edge_ref in order {
                let mut names = adjacency.remove(&edge_ref).unwrap_or_default();
                names.sort();
                names.dedup();
                let base = names.join("|");
                let n = counts.entry(base.clone()).or_default();
                self.edges
                    .entry(edge_ref)
                    .or_insert_with(|| format!("{qualifier}{base}[{n}]"));
                *n += 1;
            }
        }
    }

    fn name_of(&self, geometry_ref: usize) -> Option<String> {
        self.faces
            .get(&geometry_ref)
            .or_else(|| self.edges.get(&geometry_ref))
            .or_else(|| self.vertices.get(&geometry_ref))
            .cloned()
    }
}

/// A real with no trailing zeros (`10.5`, `0`, `-2`), the vertex-ref form.
fn trim_real(value: f64) -> String {
    let rounded = (value * 1e9).round() / 1e9;
    let text = format!("{rounded:.9}");
    let trimmed = text.trim_end_matches('0').trim_end_matches('.');
    if trimmed == "-0" || trimmed.is_empty() {
        "0".into()
    } else {
        trimmed.to_string()
    }
}

/// The EDGE_CURVE refs a face's bounds use, loop order.
fn face_edge_refs(resolver: &Resolver, face_ref: usize) -> Vec<usize> {
    let mut out = Vec::new();
    let Ok(face) = resolver.get(face_ref) else {
        return out;
    };
    let Some(args) = face.find("ADVANCED_FACE").or_else(|| face.find("FACE_SURFACE")) else {
        return out;
    };
    for bound_ref in arg_refs(args, 1) {
        let Ok(bound) = resolver.get(bound_ref) else { continue };
        let Some(bound_args) = bound
            .find("FACE_OUTER_BOUND")
            .or_else(|| bound.find("FACE_BOUND"))
        else {
            continue;
        };
        let Some(loop_ref) = arg_ref(bound_args, 1) else { continue };
        let Ok(edge_loop) = resolver.get(loop_ref) else { continue };
        let Some(loop_args) = edge_loop.find("EDGE_LOOP") else { continue };
        for oriented_ref in arg_refs(loop_args, 1) {
            let Ok(oriented) = resolver.get(oriented_ref) else { continue };
            let Some(oriented_args) = oriented.find("ORIENTED_EDGE") else { continue };
            if let Some(edge_ref) = arg_ref(oriented_args, 3) {
                out.push(edge_ref);
            }
        }
    }
    out
}

/// A body's ordered face refs, shell by shell (empty on a pairing mismatch).
pub(super) fn body_face_refs(resolver: &Resolver, body: StepBody, built: &BrepSolid) -> Vec<usize> {
    let Some(shells) = step_body_claimed_shells(resolver, body) else {
        return Vec::new();
    };
    if shells.len() != built.shells.len() {
        return Vec::new();
    }
    let mut out = Vec::new();
    for (shell_ref, built_shell) in shells.iter().zip(&built.shells) {
        let Some(faces) = super::bodies::shell_face_refs_pub(resolver, *shell_ref) else {
            return Vec::new();
        };
        if faces.len() != built_shell.faces.len() {
            return Vec::new();
        }
        out.extend(faces);
    }
    out
}

/// One semantic element found in the file.
struct Semantic {
    entity: usize,
    annotation: PmiAnnotation,
}

/// The aspect → geometry-name map (SHAPE_ASPECT / DATUM_FEATURE →
/// GEOMETRIC_ITEM_SPECIFIC_USAGE → geometry).
///
/// `pins` is the STRUCTURED lane's occurrence map: a shape aspect whose
/// `of_shape` is an OCCURRENCE's `PRODUCT_DEFINITION_SHAPE` — the file saying
/// "this tolerance is on THIS instance", not on the part — has its geometry
/// name tagged with that occurrence, which `assembly_pmi.rs` reads back off
/// the finished annotation. Empty for the flat lane, where there are no
/// occurrences to pin to.
fn aspect_names(
    entities: &HashMap<usize, Entity>,
    names: &NameMap,
    pins: &HashMap<usize, usize>,
) -> HashMap<usize, String> {
    let mut out = HashMap::default();
    for entity in entities.values() {
        let Some(args) = entity.find("GEOMETRIC_ITEM_SPECIFIC_USAGE") else {
            continue;
        };
        let (Some(aspect), Some(geometry)) = (arg_ref(args, 2), arg_ref(args, 4)) else {
            continue;
        };
        if let Some(name) = names.name_of(geometry) {
            out.entry(aspect).or_insert(name);
        }
    }
    // ITEM_IDENTIFIED_REPRESENTATION_USAGE with a single item (older files).
    for entity in entities.values() {
        let Some(args) = entity.find("ITEM_IDENTIFIED_REPRESENTATION_USAGE") else {
            continue;
        };
        let Some(aspect) = arg_ref(args, 2) else { continue };
        let geometry = match args.get(4) {
            Some(Value::Ref(id)) => Some(*id),
            Some(Value::Typed(_, inner)) => inner.first().and_then(|v| v.as_ref_id().ok()),
            _ => None,
        };
        if let Some(name) = geometry.and_then(|g| names.name_of(g)) {
            out.entry(aspect).or_insert(name);
        }
    }
    for (aspect, occurrence) in pins {
        if let Some(name) = out.get_mut(aspect) {
            *name = assembly_pmi::pin_occurrence(name, *occurrence);
        }
    }
    out
}

/// The datum letter behind a datum-ish reference (a DATUM, or a
/// DATUM_REFERENCE_COMPARTMENT / _ELEMENT / AP214 DATUM_REFERENCE wrapping one).
fn datum_letter(entities: &HashMap<usize, Entity>, id: usize, depth: usize) -> Option<(String, String)> {
    if depth > 4 {
        return None;
    }
    let entity = entities.get(&id)?;
    if let Some(args) = entity.find("DATUM") {
        return Some((arg_string(args, 4).to_ascii_uppercase(), String::new()));
    }
    if let Some(args) = entity.find("DATUM_REFERENCE") {
        return datum_letter(entities, arg_ref(args, 1)?, depth + 1);
    }
    for keyword in ["DATUM_REFERENCE_COMPARTMENT", "DATUM_REFERENCE_ELEMENT", "GENERAL_DATUM_REFERENCE"] {
        if let Some(args) = entity.find(keyword) {
            // Full form: (name, description, of_shape, pd, base, modifiers);
            // the abbreviated form some writers emit: (base, modifiers).
            let (base, modifiers) = if args.len() >= 6 {
                (args.get(4), args.get(5))
            } else {
                (args.first(), args.get(1))
            };
            let base_ref = match base {
                Some(Value::Ref(id)) => *id,
                Some(Value::Typed(_, inner)) => inner.first()?.as_ref_id().ok()?,
                _ => return None,
            };
            let (letter, _) = datum_letter(entities, base_ref, depth + 1)?;
            let modifier = modifiers
                .and_then(|value| value.as_list().ok())
                .map(|items| items.iter().filter_map(modifier_text).next().unwrap_or_default())
                .unwrap_or_default();
            return Some((letter, modifier));
        }
    }
    None
}

fn modifier_text(value: &Value) -> Option<String> {
    let name = match value {
        Value::Enum(name) => name.clone(),
        Value::Typed(_, inner) => match inner.first()? {
            Value::Enum(name) => name.clone(),
            _ => return None,
        },
        _ => return None,
    };
    match name.as_str() {
        "MAXIMUM_MATERIAL_REQUIREMENT" => Some("MMC".into()),
        "LEAST_MATERIAL_REQUIREMENT" => Some("LMC".into()),
        _ => None,
    }
}

/// The 'nominal value' / 'upper limit' / 'lower limit' items and the ± bounds
/// of a dimension entity.
struct DimensionValues {
    nominal: Option<f64>,
    upper_limit: Option<f64>,
    lower_limit: Option<f64>,
    plus_minus: Option<(f64, f64)>,
    /// A 'hole callout' descriptive item.
    callout: Option<String>,
    angle: bool,
}

fn dimension_values(entities: &HashMap<usize, Entity>, dimension: usize, names: &NameMap) -> DimensionValues {
    let mut values = DimensionValues {
        nominal: None,
        upper_limit: None,
        lower_limit: None,
        plus_minus: None,
        callout: None,
        angle: false,
    };
    let scale = |entity: &Entity, value: f64| -> f64 {
        if entity.has("PLANE_ANGLE_MEASURE_WITH_UNIT") {
            value * names.angle_to_degrees
        } else {
            value * names.length_scale
        }
    };
    for entity in entities.values() {
        if let Some(args) = entity.find("DIMENSIONAL_CHARACTERISTIC_REPRESENTATION") {
            if arg_ref(args, 0) != Some(dimension) {
                continue;
            }
            let Some(representation) = arg_ref(args, 1).and_then(|id| entities.get(&id)) else {
                continue;
            };
            let Some(rep_args) = representation
                .find("SHAPE_DIMENSION_REPRESENTATION")
                .or_else(|| representation.find("REPRESENTATION"))
            else {
                continue;
            };
            for item_ref in arg_refs(rep_args, 1) {
                let Some(item) = entities.get(&item_ref) else { continue };
                let name = item
                    .find("REPRESENTATION_ITEM")
                    .map(|args| arg_string(args, 0))
                    .unwrap_or_default();
                if let Some(args) = item.find("DESCRIPTIVE_REPRESENTATION_ITEM") {
                    if arg_string(args, 0) == "hole callout" {
                        values.callout = Some(arg_string(args, 1));
                    }
                    continue;
                }
                let Some(raw) = item
                    .find("MEASURE_WITH_UNIT")
                    .and_then(|args| args.first())
                    .and_then(measure_value)
                else {
                    continue;
                };
                if item.has("PLANE_ANGLE_MEASURE_WITH_UNIT") {
                    values.angle = true;
                }
                let value = scale(item, raw);
                match name.as_str() {
                    "upper limit" => values.upper_limit = Some(value),
                    "lower limit" => values.lower_limit = Some(value),
                    _ => {
                        if values.nominal.is_none() || name == "nominal value" {
                            values.nominal = Some(value);
                        }
                    }
                }
            }
        }
        if let Some(args) = entity.find("PLUS_MINUS_TOLERANCE") {
            if arg_ref(args, 1) != Some(dimension) {
                continue;
            }
            let Some(range) = arg_ref(args, 0).and_then(|id| entities.get(&id)) else {
                continue;
            };
            let Some(range_args) = range.find("TOLERANCE_VALUE") else {
                continue;
            };
            let bound = |slot: usize| -> Option<f64> {
                let measure = arg_ref(range_args, slot).and_then(|id| entities.get(&id))?;
                let raw = measure
                    .find("LENGTH_MEASURE_WITH_UNIT")
                    .or_else(|| measure.find("PLANE_ANGLE_MEASURE_WITH_UNIT"))
                    .or_else(|| measure.find("MEASURE_WITH_UNIT"))
                    .and_then(|args| args.first())
                    .and_then(measure_value)?;
                Some(scale(measure, raw))
            };
            if let (Some(lower), Some(upper)) = (bound(0), bound(1)) {
                values.plus_minus = Some((lower, upper));
            }
        }
    }
    values
}

/// Fold the values into a dimension annotation's tolerance block.
fn apply_values(params: &mut serde_json::Map<String, serde_json::Value>, values: &DimensionValues) {
    if let (Some(nominal), Some(upper), Some(lower)) = (values.nominal, values.upper_limit, values.lower_limit) {
        params.insert("tolMode".into(), "limits".into());
        params.insert("tolUpper".into(), serde_json::json!(round6(upper - nominal)));
        params.insert("tolLower".into(), serde_json::json!(round6(nominal - lower)));
    } else if let Some((lower, upper)) = values.plus_minus {
        if (lower.abs() - upper.abs()).abs() < 1e-9 {
            params.insert("tolMode".into(), "symmetric".into());
            params.insert("tolUpper".into(), serde_json::json!(round6(upper.abs())));
        } else {
            params.insert("tolMode".into(), "deviation".into());
            params.insert("tolUpper".into(), serde_json::json!(round6(upper.abs())));
            params.insert("tolLower".into(), serde_json::json!(round6(lower.abs())));
        }
    }
}

fn round6(value: f64) -> f64 {
    (value * 1e6).round() / 1e6
}

/// A saved view decoded from a `DRAUGHTING_MODEL` + `CAMERA_MODEL_D3`.
struct ViewIn {
    name: String,
    camera: Option<PmiCamera>,
    /// The callouts it shows (direct items + its annotation planes' elements).
    callouts: Vec<usize>,
}

fn read_camera(entities: &HashMap<usize, Entity>, resolver: &Resolver, camera_ref: usize) -> Option<(String, PmiCamera)> {
    let camera = entities.get(&camera_ref)?;
    let args = camera.find("CAMERA_MODEL_D3")?;
    let name = arg_string(args, 0);
    let frame = resolver.placement(arg_ref(args, 1)?).ok()?;
    let volume = entities.get(&arg_ref(args, 2)?)?;
    let volume_args = volume.find("VIEW_VOLUME")?;
    let parallel = volume_args.first().map(|v| v.enum_is("PARALLEL")).unwrap_or(true);
    let distance = resolver.length(volume_args.get(2)?.as_real().ok()?).max(1e-6);
    let window = entities.get(&arg_ref(volume_args, 8)?)?;
    let window_args = window.find("PLANAR_BOX")?;
    let width = resolver.length(window_args.get(1)?.as_real().ok()?);
    let height = resolver.length(window_args.get(2)?.as_real().ok()?).max(1e-9);
    let eye = frame.origin;
    let target = eye.add(frame.z.scale(distance));
    let projection = if parallel {
        PmiProjection::Orthographic {
            half_height: height * 0.5,
        }
    } else {
        PmiProjection::Perspective {
            fov_y_deg: 2.0 * (height * 0.5 / distance).atan().to_degrees(),
        }
    };
    let aspect = if height > 0.0 { width / height } else { 1.5 };
    Some((
        name,
        PmiCamera {
            eye: [eye.x, eye.y, eye.z],
            target: [target.x, target.y, target.z],
            up: [frame.y.x, frame.y.y, frame.y.z],
            projection,
            viewport: [800.0 * aspect, 800.0],
        },
    ))
}

/// The `draughting_callout` record of an entity, whatever SUBTYPE keyword a
/// producer wrote it under: the supertype itself (what the CAx-IF practice
/// recommends and what our own writer emits), or one of the AP214-era
/// draughting subtypes a vendor file carries instead —
/// `LEADER_DIRECTED_CALLOUT`, `PROJECTION_DIRECTED_CALLOUT`,
/// `DIMENSION_CURVE_DIRECTED_CALLOUT`. None of them adds an attribute AHEAD of
/// `contents`, so the contents list is slot 1 under every one of those
/// keywords, and matching on the suffix reads them all.
///
/// `io1-ac-214.stp` is the reason this is not simply `has("DRAUGHTING_CALLOUT")`:
/// its two annotations are `LEADER_DIRECTED_CALLOUT`s, so an exact-keyword test
/// lifted nothing at all from the file.
fn callout_contents(entity: &Entity) -> Option<&[Value]> {
    entity
        .records
        .iter()
        .find(|(keyword, _)| keyword.ends_with("CALLOUT"))
        .map(|(_, args)| args.as_slice())
}

/// Whether an entity is a `draughting_callout` of any subtype.
fn is_callout(entity: &Entity) -> bool {
    callout_contents(entity).is_some()
}

/// The literal text of a `text_literal` of any subtype
/// (`TEXT_LITERAL_WITH_EXTENT` and friends). `text_literal` is a
/// `geometric_representation_item`, so the INHERITED `name` is slot 0 and the
/// literal itself is slot 1 — reading slot 0 returns the producer's entity
/// label (`'C9C'`), not the annotation's words.
fn text_literal(entity: &Entity) -> Option<String> {
    entity
        .records
        .iter()
        .find(|(keyword, _)| keyword.starts_with("TEXT_LITERAL"))
        .map(|(_, args)| arg_string(args, 1))
        .filter(|text| !text.is_empty())
}

/// The sub-items one presentation entity collects, in the file's own list
/// order. Covers the simple and the COMPLEX spellings of the same thing: an
/// `annotation_occurrence` written as a complex instance carries its item on
/// the `STYLED_ITEM` record (slot 1), not on its own.
fn presentation_children(entity: &Entity) -> Vec<usize> {
    let mut out = Vec::new();
    let mut take = |args: &[Value], slot: usize| match args.get(slot) {
        Some(Value::Ref(target)) => out.push(*target),
        Some(Value::List(items)) => {
            out.extend(items.iter().filter_map(|item| item.as_ref_id().ok()))
        }
        _ => {}
    };
    for (keyword, args) in &entity.records {
        let slot = if keyword.ends_with("CALLOUT")
            || keyword.starts_with("COMPOSITE_TEXT")
            || keyword == "GEOMETRIC_CURVE_SET"
            || keyword == "STYLED_ITEM"
        {
            1
        } else if keyword.ends_with("ANNOTATION_OCCURRENCE")
            || keyword == "ANNOTATION_CURVE_OCCURRENCE"
            || keyword == "ANNOTATION_TEXT_OCCURRENCE"
            || keyword == "ANNOTATION_SYMBOL_OCCURRENCE"
        {
            2
        } else {
            continue;
        };
        take(args, slot);
    }
    out
}

/// The callouts of an ANNOTATION_PLANE (its elements) or a callout itself.
fn expand_item(entities: &HashMap<usize, Entity>, item: usize, out: &mut Vec<usize>) {
    let Some(entity) = entities.get(&item) else { return };
    if let Some(args) = entity.find("ANNOTATION_PLANE") {
        for element in arg_refs(args, 3) {
            expand_item(entities, element, out);
        }
    } else if is_callout(entity) || entity.has("ANNOTATION_OCCURRENCE") || entity.has("ANNOTATION_CURVE_OCCURRENCE") || entity.has("ANNOTATION_TEXT_OCCURRENCE") {
        out.push(item);
    }
}

/// callout → the ANNOTATION_PLANE listing it (nested planes flattened; the
/// first plane naming a callout wins).
fn plane_of_callouts(entities: &HashMap<usize, Entity>) -> HashMap<usize, usize> {
    let mut planes: Vec<(&usize, &Entity)> = entities.iter().filter(|(_, e)| e.has("ANNOTATION_PLANE")).collect();
    planes.sort_by_key(|(id, _)| **id);
    let mut out: HashMap<usize, usize> = HashMap::default();
    for (id, entity) in planes {
        let Some(args) = entity.find("ANNOTATION_PLANE") else { continue };
        let mut members = Vec::new();
        for element in arg_refs(args, 3) {
            expand_item(entities, element, &mut members);
        }
        for member in members {
            out.entry(member).or_insert(*id);
        }
    }
    out
}

/// The imported planar face whose plane an ANNOTATION_PLANE's placement
/// coincides with (normal parallel, origin on the plane, lowest face entity
/// first) — the `plane` reference the lifted annotation lays out in. `None`
/// when the plane matches no face: the annotation then aligns to its view.
fn plane_face_name(
    entities: &HashMap<usize, Entity>,
    resolver: &Resolver,
    names: &NameMap,
    plane: usize,
) -> Option<String> {
    let args = entities.get(&plane)?.find("ANNOTATION_PLANE")?;
    let item = arg_ref(args, 2)?;
    let placement = match entities.get(&item) {
        Some(entity) if entity.has("PLANE") => arg_ref(entity.find("PLANE")?, 1)?,
        Some(entity) if entity.has("AXIS2_PLACEMENT_3D") => item,
        _ => return None,
    };
    let frame = resolver.placement(placement).ok()?;
    let normal = frame.z.normalized().ok()?;
    let tolerance = 1e-4 * frame.origin.length().max(1.0);
    let mut faces: Vec<(&usize, &String)> = names.faces.iter().collect();
    faces.sort();
    for (face_id, name) in faces {
        let Some(face_args) = entities.get(face_id).and_then(|e| e.find("ADVANCED_FACE")) else { continue };
        let Some(surface) = arg_ref(face_args, 2).and_then(|id| entities.get(&id)) else { continue };
        let Some(plane_args) = surface.find("PLANE") else { continue };
        let Some(face_frame) = arg_ref(plane_args, 1).and_then(|p| resolver.placement(p).ok()) else { continue };
        let Ok(face_normal) = face_frame.z.normalized() else { continue };
        if face_normal.dot(normal).abs() < 1.0 - 1e-6 {
            continue;
        }
        if face_frame.origin.sub(frame.origin).dot(normal).abs() > tolerance {
            continue;
        }
        return Some(name.clone());
    }
    None
}

/// The PMI validation properties of a file (practice §10.3), keyed by the
/// presentation item they characterize (a callout or one of its subsets):
/// the equivalent unicode string and the polyline centre point.
#[derive(Default)]
struct ValidationProps {
    unicode: HashMap<usize, String>,
    centre: HashMap<usize, [f64; 3]>,
}

impl ValidationProps {
    fn read(entities: &HashMap<usize, Entity>, resolver: &Resolver) -> Self {
        let mut out = Self::default();
        for entity in entities.values() {
            let Some(args) = entity.find("PROPERTY_DEFINITION_REPRESENTATION") else { continue };
            let (Some(definition), Some(representation)) = (arg_ref(args, 0), arg_ref(args, 1)) else { continue };
            let Some(within) = entities
                .get(&definition)
                .and_then(|e| e.find("PROPERTY_DEFINITION"))
                .and_then(|a| arg_ref(a, 2))
            else {
                continue;
            };
            let Some(item) = entities
                .get(&within)
                .and_then(|e| e.find("CHARACTERIZED_ITEM_WITHIN_REPRESENTATION"))
                .and_then(|a| arg_ref(a, 2))
            else {
                continue;
            };
            let Some(items) = entities.get(&representation).and_then(|e| e.find("REPRESENTATION")) else { continue };
            for value in arg_refs(items, 1) {
                let Some(value_entity) = entities.get(&value) else { continue };
                if let Some(dri) = value_entity.find("DESCRIPTIVE_REPRESENTATION_ITEM") {
                    if arg_string(dri, 0) == "equivalent unicode string" {
                        out.unicode.entry(item).or_insert_with(|| arg_string(dri, 1));
                    }
                }
                if let Some(point) = value_entity.find("CARTESIAN_POINT") {
                    if arg_string(point, 0) == "polyline centre point" {
                        if let Ok(p) = resolver.point(value) {
                            out.centre.entry(item).or_insert([p.x, p.y, p.z]);
                        }
                    }
                }
            }
        }
        out
    }
}

/// The label position and text of a callout. Text: the callout's equivalent
/// unicode string (Graphic Presentation), else its `TEXT_LITERAL`
/// (character-based files). Position: the centre point of the callout's
/// text subset, else the callout's own centre point, else the text
/// literal's placement, else the first polyline point.
fn callout_label(
    entities: &HashMap<usize, Entity>,
    resolver: &Resolver,
    props: &ValidationProps,
    callout: usize,
) -> (Option<[f64; 3]>, String) {
    let unicode = props.unicode.get(&callout).cloned().unwrap_or_default();
    let mut position = entities
        .get(&callout)
        .and_then(|entity| callout_contents(entity))
        .map(|args| arg_refs(args, 1))
        .unwrap_or_default()
        .iter()
        .find_map(|subset| props.centre.get(subset).copied())
        .or_else(|| props.centre.get(&callout).copied());
    let mut fallback = None;
    // Every literal the callout collects, in the file's own list order: a
    // `composite_text` splits one annotation's words across several
    // `text_literal`s (io1-ac-214's note is two lines), and taking whichever
    // one the walk reached first would keep half the sentence.
    let mut literals: Vec<String> = Vec::new();
    let mut visit = |id: usize, entity: &Entity| {
        if let Some(literal) = text_literal(entity) {
            literals.push(literal);
            if position.is_none() {
                if let Some(frame) = entity
                    .records
                    .iter()
                    .find(|(keyword, _)| keyword.starts_with("TEXT_LITERAL"))
                    .and_then(|(_, args)| arg_ref(args, 2))
                    .and_then(|placement| resolver.placement(placement).ok())
                {
                    position = Some([frame.origin.x, frame.origin.y, frame.origin.z]);
                }
            }
        }
        if let Some(args) = entity.find("POLYLINE") {
            if fallback.is_none() {
                if let Some(point) = arg_refs(args, 1).first().and_then(|p| resolver.point(*p).ok()) {
                    fallback = Some([point.x, point.y, point.z]);
                }
            }
        }
        let _ = id;
    };
    // Depth first in LIST order (children pushed reversed onto the stack), so
    // the literals come out in the order the file wrote them.
    let mut stack = vec![callout];
    let mut seen: HashSet<usize> = HashSet::default();
    while let Some(id) = stack.pop() {
        if !seen.insert(id) {
            continue;
        }
        let Some(entity) = entities.get(&id) else { continue };
        visit(id, entity);
        let mut children = presentation_children(entity);
        children.reverse();
        stack.extend(children);
    }
    let text = if unicode.is_empty() {
        literals.join("\n")
    } else {
        unicode
    };
    (position.or(fallback), text)
}

/// Read a file's PMI into a [`PmiState`] whose references name the geometry
/// an IMPORT3D feature `feature_name` will stamp. `Ok(None)` when the file
/// carries no PMI.
pub fn read_step_pmi(text: &str, feature_name: &str) -> Result<Option<PmiState>, String> {
    if !file_mentions_pmi(text) {
        return Ok(None);
    }
    let entities = parse_data_section(text)?;
    let names = NameMap::build(text, &entities, feature_name)?;
    Ok(lift_pmi(text, &entities, &names, &HashMap::default(), &mut Vec::new()))
}

/// The cheap text test that keeps a PMI-less file off the parse. A superset of
/// the entity test — a mention inside a comment merely costs one parse.
pub(super) fn file_mentions_pmi(text: &str) -> bool {
    text.contains("DIMENSIONAL_")
        || text.contains("_TOLERANCE(")
        || text.contains("DATUM(")
        || text.contains("CAMERA_MODEL_D3(")
        || text.contains("CALLOUT(")
}

/// The lift itself, against an ALREADY-BUILT name map — so the flat lane
/// (one map for the whole file) and the structured lane (one map per product,
/// each entry tagged with its product, `assembly_pmi.rs`) read the file's PMI
/// through exactly the same code.
///
/// `diagnostics` collects references the file made that named geometry this
/// importer did not mint. Such an annotation is KEPT, carrying the file's own
/// entity id as its reference (`STEP#1234`), so it reaches the PMI panel as a
/// row with an error rather than disappearing between two file formats.
pub(super) fn lift_pmi(
    text: &str,
    entities: &HashMap<usize, Entity>,
    names: &NameMap,
    pins: &HashMap<usize, usize>,
    diagnostics: &mut Vec<String>,
) -> Option<PmiState> {
        let resolver = Resolver::new(entities, names.length_scale);
    let aspects = aspect_names(entities, names, pins);
    // A shape aspect whose geometry this importer did not mint still names
    // something: the file's own entity. `read_step_pmi`'s contract is that an
    // annotation is never dropped for want of a reference.
    let mut unresolved: Vec<usize> = Vec::new();
    let mut aspect_ref = |id: usize| -> String {
        match aspects.get(&id) {
            Some(name) => name.clone(),
            None => {
                unresolved.push(id);
                format!("{UNRESOLVED_REF_PREFIX}{id}")
            }
        }
    };
    let mut ids: BTreeMap<usize, &Entity> = entities.iter().map(|(id, entity)| (*id, entity)).collect();
    let mut state = PmiState::default();
    let mut semantics: Vec<Semantic> = Vec::new();
    let mut params = |pairs: Vec<(&str, serde_json::Value)>| -> serde_json::Map<String, serde_json::Value> {
        let mut map = serde_json::Map::new();
        for (key, value) in pairs {
            map.insert(key.to_string(), value);
        }
        map
    };

    // Datums first (frames reference their letters).
    let mut datum_ids: HashMap<usize, String> = HashMap::default();
    for (id, entity) in ids.iter() {
        let Some(args) = entity.find("DATUM") else { continue };
        let letter = arg_string(args, 4).to_ascii_uppercase();
        if letter.is_empty() {
            continue;
        }
        // The datum feature: SHAPE_ASPECT_RELATIONSHIP(relating = feature, related = datum).
        let feature = entities.values().find_map(|candidate| {
            let rel = candidate.find("SHAPE_ASPECT_RELATIONSHIP")?;
            (arg_ref(rel, 3) == Some(*id)).then(|| arg_ref(rel, 2)).flatten()
        });
        let Some(target) = feature.map(&mut aspect_ref) else {
            continue; // the datum names no shape aspect at all
        };
        let annotation_id = state.next_id("DTM");
        datum_ids.insert(*id, letter.clone());
        semantics.push(Semantic {
            entity: *id,
            annotation: PmiAnnotation {
                kind: "datum".into(),
                enabled: true,
                params: serde_json::Value::Object(params(vec![
                    ("id", annotation_id.into()),
                    ("target", target.into()),
                    ("letter", letter.into()),
                ])),
                label_world: None,
            },
        });
    }

    // Dimensions.
    for (id, entity) in ids.iter() {
        let mut fields: Option<(&str, serde_json::Map<String, serde_json::Value>)> = None;
        if let Some(args) = entity.find("ANGULAR_LOCATION").or_else(|| entity.find("DIMENSIONAL_LOCATION")) {
            let (Some(a), Some(b)) = (
                arg_ref(args, 2).map(&mut aspect_ref),
                arg_ref(args, 3).map(&mut aspect_ref),
            ) else {
                continue; // not a two-aspect location: not a dimension we read
            };
            let values = dimension_values(entities, *id, &names);
            if entity.has("ANGULAR_LOCATION") {
                let mut map = params(vec![("targets", serde_json::json!([a, b])), ("decimals", 1.into())]);
                if let Some(nominal) = values.nominal {
                    let kind = if nominal > 180.0 { "reflex" } else if nominal > 90.0 + 1e-9 { "obtuse" } else { "acute" };
                    map.insert("angleType".into(), kind.into());
                }
                apply_values(&mut map, &values);
                fields = Some(("angle", map));
            } else {
                let mut map = params(vec![("targets", serde_json::json!([a, b]))]);
                if let Some(orientation) = orientation_axis(entities, *id) {
                    map.insert("alignment".into(), orientation.into());
                }
                apply_values(&mut map, &values);
                fields = Some(("linear", map));
            }
        } else if let Some(args) = entity.find("DIMENSIONAL_SIZE") {
            let Some(target) = arg_ref(args, 0).map(&mut aspect_ref) else {
                continue; // the size names no shape aspect
            };
            let name = arg_string(args, 1).to_ascii_lowercase();
            let values = dimension_values(entities, *id, &names);
            if let Some(callout) = &values.callout {
                let _ = callout;
                let map = params(vec![("target", target.into()), ("showQuantity", true.into())]);
                fields = Some(("holeCallout", map));
            } else if name.contains("diameter") || name.contains("radius") {
                let mut map = params(vec![
                    ("target", target.into()),
                    ("displayStyle", if name.contains("radius") { "radius" } else { "diameter" }.into()),
                ]);
                apply_values(&mut map, &values);
                fields = Some(("radial", map));
            } else {
                let mut map = params(vec![("targets", serde_json::json!([target]))]);
                apply_values(&mut map, &values);
                fields = Some(("linear", map));
            }
        }
        if let Some((kind, mut map)) = fields {
            let prefix = match kind {
                "angle" => "ANG",
                "radial" => "RAD",
                "holeCallout" => "HOLE",
                _ => "DIM",
            };
            map.insert("id".into(), state.next_id(prefix).into());
            semantics.push(Semantic {
                entity: *id,
                annotation: PmiAnnotation {
                    kind: kind.into(),
                    enabled: true,
                    params: serde_json::Value::Object(map),
                    label_world: None,
                },
            });
        }
    }

    // Geometric tolerances.
    for (id, entity) in ids.iter() {
        let Some((kind_record, base_args)) = entity.records.iter().find_map(|(keyword, args)| {
            characteristic_for_entity(keyword).map(|c| (c, args))
        }) else {
            continue;
        };
        // The base attributes live on GEOMETRIC_TOLERANCE in a complex
        // instance, on the typed record itself in a simple one.
        let args = entity.find("GEOMETRIC_TOLERANCE").unwrap_or(base_args);
        if args.len() < 4 {
            continue;
        }
        let Some(target) = arg_ref(args, 3).map(&mut aspect_ref) else {
            continue; // the tolerance names no shape aspect
        };
        let magnitude = arg_ref(args, 2)
            .and_then(|m| entities.get(&m))
            .and_then(|m| m.find("LENGTH_MEASURE_WITH_UNIT").or_else(|| m.find("MEASURE_WITH_UNIT")))
            .and_then(|a| a.first())
            .and_then(measure_value)
            .map(|v| v * names.length_scale)
            .unwrap_or(0.1);
        let mut map = params(vec![
            ("target", target.into()),
            ("characteristic", kind_record.id.into()),
            ("zoneValue", serde_json::json!(round6(magnitude))),
        ]);
        if let Some(mod_args) = entity.find("GEOMETRIC_TOLERANCE_WITH_MODIFIERS") {
            if let Some(items) = mod_args.first().and_then(|v| v.as_list().ok()) {
                if let Some(modifier) = items.iter().filter_map(modifier_text).next() {
                    map.insert("materialCondition".into(), modifier.into());
                }
            }
        }
        if let Some(ref_args) = entity.find("GEOMETRIC_TOLERANCE_WITH_DATUM_REFERENCE") {
            let mut letters: Vec<(String, String)> = Vec::new();
            for system_ref in arg_refs(ref_args, 0) {
                let Some(system) = entities.get(&system_ref) else { continue };
                if let Some(system_args) = system.find("DATUM_SYSTEM") {
                    for constituent in arg_refs(system_args, 4) {
                        if let Some(pair) = datum_letter(entities, constituent, 0) {
                            letters.push(pair);
                        }
                    }
                } else if let Some(pair) = datum_letter(entities, system_ref, 0) {
                    letters.push(pair);
                }
            }
            for (slot, (letter, modifier)) in ["A", "B", "C"].iter().zip(letters) {
                map.insert(format!("datum{slot}"), letter.into());
                if !modifier.is_empty() {
                    map.insert(format!("datum{slot}Modifier"), modifier.into());
                }
            }
        }
        let cylindrical = entities.values().any(|zone| {
            zone.find("TOLERANCE_ZONE")
                .map(|zone_args| {
                    arg_refs(zone_args, 4).contains(id)
                        && arg_ref(zone_args, 5)
                            .and_then(|form| entities.get(&form))
                            .and_then(|form| form.find("TOLERANCE_ZONE_FORM"))
                            .map(|form_args| arg_string(form_args, 0).to_ascii_lowercase().contains("cylindrical"))
                            .unwrap_or(false)
                })
                .unwrap_or(false)
        });
        if cylindrical {
            map.insert("zoneDiameter".into(), true.into());
        }
        map.insert("id".into(), state.next_id("FCF").into());
        semantics.push(Semantic {
            entity: *id,
            annotation: PmiAnnotation {
                kind: "fcf".into(),
                enabled: true,
                params: serde_json::Value::Object(map),
                label_world: None,
            },
        });
    }
    ids.clear();

    // Presentation links: semantic entity (or its datum feature aspect) → callout.
    let mut callout_of: HashMap<usize, usize> = HashMap::default();
    for entity in entities.values() {
        let Some(args) = entity.find("DRAUGHTING_MODEL_ITEM_ASSOCIATION") else { continue };
        if let (Some(definition), Some(callout)) = (arg_ref(args, 2), arg_ref(args, 4)) {
            callout_of.entry(definition).or_insert(callout);
        }
    }
    // A datum's callout is linked through its datum feature aspect.
    let datum_feature_of: HashMap<usize, usize> = entities
        .values()
        .filter_map(|entity| {
            let rel = entity.find("SHAPE_ASPECT_RELATIONSHIP")?;
            Some((arg_ref(rel, 3)?, arg_ref(rel, 2)?))
        })
        .collect();
    let mut linked_callouts: HashSet<usize> = HashSet::default();
    let mut annotations: Vec<(Option<usize>, PmiAnnotation)> = Vec::new();
    // A callout's ANNOTATION_PLANE that coincides with a planar face becomes
    // the annotation's `plane` reference; any other plane leaves it aligned
    // to the view.
    let plane_of = plane_of_callouts(entities);
    let props = ValidationProps::read(entities, &resolver);
    let mut plane_faces: HashMap<usize, Option<String>> = HashMap::default();
    let mut plane_param = |callout: usize, annotation: &mut PmiAnnotation| {
        let Some(plane) = plane_of.get(&callout).copied() else { return };
        let face = plane_faces
            .entry(plane)
            .or_insert_with(|| plane_face_name(entities, &resolver, &names, plane))
            .clone();
        if let (Some(face), Some(object)) = (face, annotation.params.as_object_mut()) {
            object.insert("plane".into(), serde_json::Value::String(face));
        }
    };
    for semantic in semantics {
        let callout = callout_of
            .get(&semantic.entity)
            .copied()
            .or_else(|| datum_feature_of.get(&semantic.entity).and_then(|f| callout_of.get(f)).copied());
        let mut annotation = semantic.annotation;
        if let Some(callout) = callout {
            linked_callouts.insert(callout);
            let (position, _) = callout_label(entities, &resolver, &props, callout);
            annotation.label_world = position;
            plane_param(callout, &mut annotation);
        }
        annotations.push((callout, annotation));
    }
    // Free 3D text callouts become notes.
    for (id, entity) in entities.iter() {
        if !is_callout(entity) || linked_callouts.contains(id) {
            continue;
        }
        let (position, text) = callout_label(entities, &resolver, &props, *id);
        if text.trim().is_empty() {
            continue;
        }
        let annotation_id = state.next_id("NOTE");
        let mut annotation = PmiAnnotation {
            kind: "note".into(),
            enabled: true,
            params: serde_json::Value::Object(params(vec![("id", annotation_id.into()), ("text", text.into())])),
            label_world: position,
        };
        plane_param(*id, &mut annotation);
        annotations.push((Some(*id), annotation));
    }
    if annotations.is_empty() && !text.contains("CAMERA_MODEL_D3(") {
        return None;
    }

    // Saved views.
    let mut views_in: Vec<ViewIn> = Vec::new();
    let mut models: Vec<(&usize, &Entity)> = entities.iter().filter(|(_, e)| e.has("DRAUGHTING_MODEL")).collect();
    models.sort_by_key(|(id, _)| **id);
    for (_, model) in models {
        let Some(args) = model.find("DRAUGHTING_MODEL") else { continue };
        let items = arg_refs(args, 1);
        let camera = items
            .iter()
            .find_map(|item| read_camera(entities, &resolver, *item));
        let Some((camera_name, camera)) = camera else {
            continue; // the global model (no camera) is not a saved view
        };
        let mut callouts = Vec::new();
        for item in &items {
            expand_item(entities, *item, &mut callouts);
        }
        let model_name = arg_string(args, 0);
        views_in.push(ViewIn {
            name: if camera_name.trim().is_empty() { model_name } else { camera_name },
            camera: Some(camera),
            callouts,
        });
    }
    let mut placed: HashSet<usize> = HashSet::default();
    for view_in in views_in {
        let id = state.next_id("VIEW");
        let mut view = PmiView {
            id,
            name: view_in.name,
            camera: view_in.camera,
            display: PmiDisplay::default(),
            annotations: Vec::new(),
        };
        for (index, (callout, annotation)) in annotations.iter().enumerate() {
            if let Some(callout) = callout {
                if view_in.callouts.contains(callout) && !placed.contains(&index) {
                    view.annotations.push(annotation.clone());
                    placed.insert(index);
                }
            }
        }
        state.views.push(view);
    }
    let unplaced: Vec<PmiAnnotation> = annotations
        .iter()
        .enumerate()
        .filter(|(index, _)| !placed.contains(index))
        .map(|(_, (_, annotation))| annotation.clone())
        .collect();
    if !unplaced.is_empty() {
        let id = state.next_id("VIEW");
        state.views.push(PmiView {
            id,
            name: "Imported PMI".into(),
            camera: None,
            display: PmiDisplay::default(),
            annotations: unplaced,
        });
    }
    if state.views.is_empty() {
        return None;
    }
    for aspect in unresolved {
        diagnostics.push(format!(
            "PMI: shape aspect #{aspect} names geometry this import did not build; \
             the annotation is kept with an unresolved reference"
        ));
    }
    Some(state)
}

/// The world axis of an oriented dimension's 'orientation' placement (X / Y / Z).
fn orientation_axis(entities: &HashMap<usize, Entity>, dimension: usize) -> Option<&'static str> {
    for entity in entities.values() {
        let Some(args) = entity.find("DIMENSIONAL_CHARACTERISTIC_REPRESENTATION") else { continue };
        if arg_ref(args, 0) != Some(dimension) {
            continue;
        }
        let Some(representation) = arg_ref(args, 1).and_then(|id| entities.get(&id)) else { continue };
        let Some(rep_args) = representation.find("SHAPE_DIMENSION_REPRESENTATION") else { continue };
        for item_ref in arg_refs(rep_args, 1) {
            let Some(item) = entities.get(&item_ref) else { continue };
            let Some(placement) = item.find("AXIS2_PLACEMENT_3D") else { continue };
            if arg_string(placement, 0) != "orientation" {
                continue;
            }
            let Some(direction) = arg_ref(placement, 3).and_then(|id| entities.get(&id)) else { continue };
            let Some(list) = direction.find("DIRECTION").and_then(|d| d.get(1)).and_then(|v| v.as_list().ok()) else { continue };
            let components: Vec<f64> = list.iter().filter_map(|v| v.as_real().ok()).collect();
            let axis = components
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(index, _)| index)?;
            return Some(["X", "Y", "Z"][axis.min(2)]);
        }
    }
    None
}
