//! Constraint → mate mapping SHARED MACHINERY: selection-ref resolution
//! against the scene's component registry, the [`map_constraint`] dispatch,
//! orientation-preference handling (`preferredOppose` XOR
//! reverse/opposeNormals), first-solve initialization, and the rigid-pose math
//! shared with the write-back lane. The PER-TYPE `map` functions live with
//! their constraints ([`super::constraints`], one module per type).
//!
//! # Selection-ref conventions (cross-lane contract)
//!
//! - Face/edge refs are the NAMESPACED scene names (`ACOMP2:Extrude1_top`).
//! - A whole-component ref is the bare feature id (`ACOMP2`), or a nested
//!   chain (`ACOMP3:ACOMP1`) owned by the OUTERMOST component at this level.
//! - A vertex ref is `{solidName}@x,y,z` with the position in COMPONENT-LOCAL
//!   coordinates (stable under pose changes; kernel vertices carry no names).
//!   The picker builds it as `world_pick · component_transform⁻¹`.

use super::ConstraintEntry;
use crate::feature_pipeline::component::ComponentRecord;
use crate::feature_pipeline::{Env, SceneMap};
use crate::{
    resolve_edge_selection, resolve_face_selection, resolve_named_selection,
    resolve_vertex_selection, AffineTransform, MateAlign, MateKind, SelectionGeometry, Vec3,
};

/// A per-constraint failure carrying the requirements-§5 status word.
#[derive(Debug, Clone)]
pub(super) struct ConstraintFailure {
    pub status: &'static str,
    pub message: String,
}

impl ConstraintFailure {
    pub fn new(status: &'static str, message: impl Into<String>) -> Self {
        Self {
            status,
            message: message.into(),
        }
    }
    pub(super) fn unsupported(message: impl Into<String>) -> Self {
        Self::new("unsupported-selection", message)
    }
    pub(super) fn invalid(message: impl Into<String>) -> Self {
        Self::new("invalid-selection", message)
    }
}

// ===========================================================================
// Element resolution
// ===========================================================================

/// One resolved constraint element: the owning component plus the analytic
/// frame in WORLD space (as stored on the resident, world-posed solids) and in
/// the component's LOCAL space (the solver's mate-input space).
#[derive(Debug, Clone)]
pub(super) struct ResolvedElement {
    pub name: String,
    pub component: String,
    pub world: SelectionGeometry,
    pub local: SelectionGeometry,
}

/// Resolve one selection ref (conventions in the module doc). Every failure is
/// a typed status — never a panic; non-component geometry is fenced here
/// (requirements: solids from ordinary modeling features do not participate).
pub(super) fn resolve_element(
    scene: &SceneMap,
    name: &str,
) -> Result<ResolvedElement, ConstraintFailure> {
    let record = scene.owning_component(name).ok_or_else(|| {
        ConstraintFailure::invalid(format!(
            "selection '{name}' does not belong to an assembly component — only component geometry participates in constraints"
        ))
    })?;
    let world = resolve_world_geometry(scene, name, record)?;
    let inverse = record.transform.rigid_inverse().map_err(|error| {
        ConstraintFailure::new(
            "error",
            format!("component '{}': non-rigid pose: {error}", record.id),
        )
    })?;
    let local = world.transformed(&inverse).map_err(|error| {
        ConstraintFailure::new("error", format!("selection '{name}': {error}"))
    })?;
    Ok(ResolvedElement {
        name: name.to_string(),
        component: record.id.clone(),
        world,
        local,
    })
}

fn resolve_world_geometry(
    scene: &SceneMap,
    name: &str,
    record: &ComponentRecord,
) -> Result<SelectionGeometry, ConstraintFailure> {
    // Vertex ref: `{solid}@x,y,z`, component-local position.
    if let Some((solid_name, coords)) = name.split_once('@') {
        let handle = scene.resolve_solid(solid_name).ok_or_else(|| {
            ConstraintFailure::invalid(format!("vertex ref '{name}': unknown solid '{solid_name}'"))
        })?;
        let local = parse_triple(coords).ok_or_else(|| {
            ConstraintFailure::invalid(format!(
                "vertex ref '{name}': position must be 'x,y,z' numbers"
            ))
        })?;
        let world_query = record.transform.point(local);
        return crate::with_registered_solid_str(handle, |solid| {
            Ok(resolve_vertex_selection(solid, world_query))
        })
        .map_err(ConstraintFailure::invalid)?
        .map_err(|error| ConstraintFailure::new(error.status(), error.to_string()));
    }

    // Whole-component ref: the bare id, or a nested `ACOMP3:ACOMP1` chain
    // (representative point over the OUTER record's members under that
    // prefix). `owning_component` already proved the outermost segment.
    let (_, local_name) = crate::split_component_namespace(name);
    if crate::is_component_reference(local_name) {
        let prefix = format!("{name}:");
        let members: Vec<(String, u32)> = scene
            .component_solids(&record.id)
            .into_iter()
            .filter(|(member, _)| record.id == name || member.starts_with(&prefix))
            .collect();
        if members.is_empty() {
            return Err(ConstraintFailure::invalid(format!(
                "component ref '{name}' has no member solids"
            )));
        }
        return component_point(&members);
    }

    // Named topology: faces first, then edges (the deterministic-naming scheme
    // never collides across the kinds).
    if let Some(face) = scene.resolve_face(name) {
        return crate::with_registered_solid_str(face.handle, |solid| {
            Ok(resolve_face_selection(solid, face.face_id))
        })
        .map_err(ConstraintFailure::invalid)?
        .map_err(|error| ConstraintFailure::new(error.status(), error.to_string()));
    }
    if let Some(edge) = scene.resolve_edge(name) {
        return crate::with_registered_solid_str(edge.handle, |solid| {
            Ok(resolve_edge_selection(solid, edge.edge_id))
        })
        .map_err(ConstraintFailure::invalid)?
        .map_err(|error| ConstraintFailure::new(error.status(), error.to_string()));
    }
    // A member-solid name (rare — the app selects components, not solids):
    // resolve like a whole-component anchor over that one solid.
    if let Some(handle) = scene.resolve_solid(name) {
        return component_point(&[(name.to_string(), handle)]);
    }
    // Last resort: an unregistered per-solid name (should not happen for an
    // intact scene) — search the component's members by exact name.
    for (_member, handle) in scene.component_solids(&record.id) {
        let found = crate::with_registered_solid_str(handle, |solid| {
            Ok(resolve_named_selection(solid, name).ok())
        })
        .map_err(ConstraintFailure::invalid)?;
        if let Some(geometry) = found {
            return Ok(geometry);
        }
    }
    Err(ConstraintFailure::invalid(format!(
        "selection '{name}' not found in the scene"
    )))
}

/// Representative anchor of a solid set: center of the aggregate AABB over
/// topology vertices + surface control-point hulls (bounds the exact surfaces
/// without tessellation — same construction as `resolve_component_point`,
/// evaluated per resident member under one short borrow each).
fn component_point(members: &[(String, u32)]) -> Result<SelectionGeometry, ConstraintFailure> {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut any = false;
    for (member, handle) in members {
        let (lo, hi, non_empty) = crate::with_registered_solid_str(*handle, |solid| {
            let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            let mut non_empty = false;
            let mut include = |point: Vec3| {
                lo = Vec3::new(lo.x.min(point.x), lo.y.min(point.y), lo.z.min(point.z));
                hi = Vec3::new(hi.x.max(point.x), hi.y.max(point.y), hi.z.max(point.z));
                non_empty = true;
            };
            for vertex in &solid.vertices {
                include(vertex.point);
            }
            for shell in &solid.shells {
                for face in &shell.faces {
                    for row in &face.surface.control_points {
                        for control in row {
                            include(control.point()?);
                        }
                    }
                }
            }
            Ok((lo, hi, non_empty))
        })
        .map_err(|error| {
            ConstraintFailure::invalid(format!("component member '{member}': {error}"))
        })?;
        if non_empty {
            low = Vec3::new(low.x.min(lo.x), low.y.min(lo.y), low.z.min(lo.z));
            high = Vec3::new(high.x.max(hi.x), high.y.max(hi.y), high.z.max(hi.z));
            any = true;
        }
    }
    if !any {
        return Err(ConstraintFailure::invalid(
            "component has no geometry to anchor",
        ));
    }
    Ok(SelectionGeometry::Point {
        position: low.add(high).scale(0.5),
    })
}

fn parse_triple(coords: &str) -> Option<Vec3> {
    let mut parts = coords.split(',').map(str::trim);
    let x = parts.next()?.parse::<f64>().ok()?;
    let y = parts.next()?.parse::<f64>().ok()?;
    let z = parts.next()?.parse::<f64>().ok()?;
    if parts.next().is_some() {
        return None;
    }
    Some(Vec3::new(x, y, z))
}

// ===========================================================================
// Mate construction
// ===========================================================================

/// One mapped mate: local geometry sides keyed by owning component id.
#[derive(Debug, Clone)]
pub(super) struct MappedMate {
    pub body_a: String,
    pub body_b: String,
    pub kind: MateKind,
}

/// A constraint's whole mapping: its mates, the `inputParams` /
/// `persistentData` write-backs to commit ON SOLVE SUCCESS (first-solve
/// initialization), and the current measured value for the overlay label.
#[derive(Debug, Clone, Default)]
pub(super) struct MappedConstraint {
    pub mates: Vec<MappedMate>,
    pub pending_params: Vec<(String, serde_json::Value)>,
    pub pending_persistent: Vec<(String, serde_json::Value)>,
    /// `(value, unit)` — the measured current quantity (`"mm"` model units or
    /// `"deg"`), when the type has a natural scalar.
    pub measured: Option<(f64, &'static str)>,
    /// The evaluated target (distance/angle), for the overlay label suffix.
    pub target: Option<f64>,
    /// The element ROLES a multi-element type inferred, as index groups into
    /// the entry's `elements` (center: `[width pair, tab]`) — the overlay lane
    /// draws by role, not by pick order. Empty for the pairing types.
    pub groups: Vec<Vec<usize>>,
    /// A per-type sentence appended to the solved status message (center
    /// names the roles it inferred: "X centred between Y and Z"), so the
    /// panel shows what the constraint decided from the selection.
    pub note: Option<String>,
}

/// Map one validated, resolved constraint onto kernel mates. `resolved` is
/// the entry's elements in entry order (the lifecycle has already checked the
/// count against the type's range). `persistent` is the entry's live
/// `persistentData` — the orientation-preference cache (`preferredOppose`) is
/// captured/re-captured here immediately (it is a resolve-time cache, not a
/// solve result).
pub(super) fn map_constraint(
    entry: &mut ConstraintEntry,
    resolved: &[ResolvedElement],
    env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    // The pairing types take exactly two; center reads the whole slice.
    let pair = || -> Result<(&ResolvedElement, &ResolvedElement), ConstraintFailure> {
        match resolved {
            [a, b] => Ok((a, b)),
            _ => Err(ConstraintFailure::invalid(format!(
                "{} takes exactly two elements ({} given)",
                entry.constraint_type,
                resolved.len()
            ))),
        }
    };
    match entry.constraint_type.as_str() {
        "coincident" => pair().and_then(|(a, b)| super::constraints::coincident::map(a, b)),
        "touch_align" => {
            let (a, b) = pair()?;
            super::constraints::touch_align::map(entry, a, b)
        }
        "parallel" => {
            let (a, b) = pair()?;
            super::constraints::parallel::map(entry, a, b)
        }
        "distance" => {
            let (a, b) = pair()?;
            super::constraints::distance::map(entry, a, b, env)
        }
        "angle" => {
            let (a, b) = pair()?;
            super::constraints::angle::map(entry, a, b, env)
        }
        "concentric" => {
            let (a, b) = pair()?;
            super::constraints::concentric::map(entry, a, b)
        }
        "perpendicular" => pair().and_then(|(a, b)| super::constraints::perpendicular::map(a, b)),
        "tangent" => pair().and_then(|(a, b)| super::constraints::tangent::map(a, b)),
        "center" => super::constraints::center::map(entry, resolved),
        other => Err(ConstraintFailure::new(
            "error",
            format!("Unknown constraint type: {other}"),
        )),
    }
}

pub(super) fn mate(a: &ResolvedElement, b: &ResolvedElement, kind: MateKind) -> MappedMate {
    MappedMate {
        body_a: a.component.clone(),
        body_b: b.component.clone(),
        kind,
    }
}

/// The world direction a selection carries, if any (plane normal, axis/line
/// direction, circle axis).
pub(super) fn direction_of(geometry: &SelectionGeometry) -> Option<Vec3> {
    match *geometry {
        SelectionGeometry::Plane { normal, .. } => Some(normal),
        SelectionGeometry::Axis { direction, .. } | SelectionGeometry::Line { direction, .. } => {
            Some(direction)
        }
        SelectionGeometry::Circle { axis, .. } => Some(axis),
        SelectionGeometry::Sphere { .. } | SelectionGeometry::Point { .. } => None,
    }
}

pub(super) fn local_point(element: &ResolvedElement) -> [f64; 3] {
    let p = element.local.representative_point();
    [p.x, p.y, p.z]
}

pub(super) fn require_direction(element: &ResolvedElement) -> Result<(Vec3, Vec3), ConstraintFailure> {
    let world = direction_of(&element.world).ok_or_else(|| {
        ConstraintFailure::unsupported(format!(
            "selection '{}' carries no direction (needs a planar face, straight edge, axis face, or circular edge)",
            element.name
        ))
    })?;
    let local = direction_of(&element.local).expect("local mirrors world kind");
    Ok((world, local))
}

pub(super) fn require_axis(element: &ResolvedElement) -> Result<crate::MateAxis, ConstraintFailure> {
    element.local.mate_axis().ok_or_else(|| {
        ConstraintFailure::unsupported(format!(
            "selection '{}' carries no axis (needs a cylindrical/conical face, circular edge, or straight edge)",
            element.name
        ))
    })
}

pub(super) fn require_plane(element: &ResolvedElement) -> Result<crate::MatePlane, ConstraintFailure> {
    element.local.mate_plane().ok_or_else(|| {
        ConstraintFailure::unsupported(format!(
            "selection '{}' is not a planar face",
            element.name
        ))
    })
}

/// The order-independent selection-pair signature (duplicate detection AND the
/// orientation-preference recapture key).
pub(super) fn pair_signature(elements: &[String]) -> String {
    let mut pair: Vec<&str> = elements.iter().map(String::as_str).collect();
    pair.sort_unstable();
    pair.join("\n")
}

/// `preferredOppose` orientation preference: captured from the CURRENT world
/// directions on first resolve (so the initial solve preserves the assembled
/// facing), re-captured when the selection signature changes, and XOR'd with
/// the `reverse`/`opposeNormals` toggle into the mate's align sense.
pub(super) fn effective_align(
    entry: &mut ConstraintEntry,
    dir_a: Vec3,
    dir_b: Vec3,
    reverse: bool,
) -> MateAlign {
    let signature = pair_signature(&entry.elements());
    let dot = dir_a.dot(dir_b);
    let cached = entry
        .persistent("preferredOpposeSignature")
        .and_then(|value| value.as_str())
        .map(|stored| stored == signature)
        .unwrap_or(false)
        .then(|| entry.persistent("preferredOppose").and_then(|v| v.as_bool()))
        .flatten();
    let oppose = cached.unwrap_or_else(|| {
        let oppose = dot < 0.0;
        entry.set_persistent("preferredOppose", serde_json::Value::Bool(oppose));
        entry.set_persistent(
            "preferredOpposeSignature",
            serde_json::Value::String(signature),
        );
        oppose
    });
    entry.set_persistent(
        "lastOrientationDot",
        serde_json::json!(dot),
    );
    if oppose != reverse {
        MateAlign::AntiAligned
    } else {
        MateAlign::Aligned
    }
}





/// First-solve initialization: when `flag_key` is not yet set in
/// `persistentData`, the target ADOPTS the current measurement and both the
/// param and the flag are queued for commit on solve success.
#[allow(clippy::type_complexity)]
pub(super) fn first_solve_target(
    entry: &ConstraintEntry,
    param_key: &str,
    flag_key: &str,
    configured: f64,
    current: f64,
) -> (
    f64,
    (Vec<(String, serde_json::Value)>, Vec<(String, serde_json::Value)>),
) {
    let initialized = entry
        .persistent(flag_key)
        .and_then(|value| value.as_bool())
        .unwrap_or(false);
    if initialized {
        (configured, (Vec::new(), Vec::new()))
    } else {
        (
            current,
            (
                vec![(param_key.to_string(), serde_json::json!(current))],
                vec![(flag_key.to_string(), serde_json::Value::Bool(true))],
            ),
        )
    }
}





// ===========================================================================
// World-space measures (overlay labels + tolerance scaling)
// ===========================================================================

pub(super) fn angle_between_deg(a: Vec3, b: Vec3) -> f64 {
    let denominator = a.length() * b.length();
    if denominator <= 0.0 {
        return 0.0;
    }
    (a.dot(b) / denominator).clamp(-1.0, 1.0).acos().to_degrees()
}

pub(super) fn point_line_distance(point: Vec3, origin: Vec3, direction: Vec3) -> f64 {
    let offset = point.sub(origin);
    let along = offset.dot(direction) / direction.dot(direction).max(1e-300);
    offset.sub(direction.scale(along)).length()
}

/// Closest approach between two infinite lines (point-to-line when nearly
/// parallel — the skew formula degenerates there).
pub(super) fn line_line_distance(oa: Vec3, da: Vec3, ob: Vec3, db: Vec3) -> f64 {
    let cross = da.cross(db);
    let denominator = cross.length();
    if denominator < 1e-9 * da.length().max(db.length()).max(1.0) {
        return point_line_distance(ob, oa, da);
    }
    (ob.sub(oa).dot(cross) / denominator).abs()
}

/// The largest coordinate magnitude a constraint's resolved elements span —
/// the model-scale estimate behind the satisfied-vs-adjusted tolerance.
pub(super) fn elements_scale(elements: &[ResolvedElement]) -> f64 {
    let mut scale = 1.0f64;
    for point in elements.iter().map(|element| element.world.representative_point()) {
        scale = scale
            .max(point.x.abs())
            .max(point.y.abs())
            .max(point.z.abs());
    }
    scale
}

// ===========================================================================
// Rigid-pose math (bodies + write-back)
// ===========================================================================

/// Unit quaternion `[w,x,y,z]` from the rotation rows of a rigid transform
/// (Shepperd's method — numerically stable for every sign pattern).
pub(super) fn matrix_to_quaternion(transform: &AffineTransform) -> [f64; 4] {
    let m = &transform.elements;
    let (r00, r01, r02) = (m[0], m[1], m[2]);
    let (r10, r11, r12) = (m[4], m[5], m[6]);
    let (r20, r21, r22) = (m[8], m[9], m[10]);
    let trace = r00 + r11 + r22;
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [s / 4.0, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s]
    } else if r00 > r11 && r00 > r22 {
        let s = (1.0 + r00 - r11 - r22).sqrt() * 2.0;
        [(r21 - r12) / s, s / 4.0, (r01 + r10) / s, (r02 + r20) / s]
    } else if r11 > r22 {
        let s = (1.0 + r11 - r00 - r22).sqrt() * 2.0;
        [(r02 - r20) / s, (r01 + r10) / s, s / 4.0, (r12 + r21) / s]
    } else {
        let s = (1.0 + r22 - r00 - r11).sqrt() * 2.0;
        [(r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, s / 4.0]
    };
    normalize_quaternion(q)
}

pub(super) fn normalize_quaternion(q: [f64; 4]) -> [f64; 4] {
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if norm <= 0.0 || !norm.is_finite() {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]
}

/// Rigid transform from a solver pose. The quaternion is RENORMALIZED first —
/// the write-back contract (`component.rs` rejects non-rigid at 1e-8; a unit
/// quaternion's matrix is orthonormal to machine precision).
pub(super) fn pose_to_transform(
    rotation: [f64; 4],
    translation: [f64; 3],
) -> Result<AffineTransform, String> {
    let [w, x, y, z] = normalize_quaternion(rotation);
    AffineTransform::new([
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y), translation[0],
        2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x), translation[1],
        2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y), translation[2],
        0.0, 0.0, 0.0, 1.0,
    ])
}

/// The generic `inputParams.transform` pose write-back JSON: `{translate,
/// rotateEulerDeg}` with degrees in the intrinsic `XYZ` order (`R = Rx·Ry·Rz`
/// — `features::common::compose_trs_matrix`). This is the write-side half of
/// the ACOMP feature's reader contract (`assembly_component.rs` reads exactly
/// `transform.translate` + `transform.rotateEulerDeg`; a mismatched key would
/// silently zero the pose on the next history run).
///
/// `pub` (re-exported as `brep_kernel::transform_to_pose_params`) because it is
/// the ONE matrix → intrinsic-XYZ-Euler-degrees decomposition in the tree — the
/// solver write-back and the out-of-crate seams that author ACOMP poses share
/// it rather than re-deriving the convention (and its gimbal-lock branch), which
/// the readers on the other side (`assembly_component.rs`, `transform_bake`)
/// would then silently disagree with.
pub fn transform_to_pose_params(transform: &AffineTransform) -> serde_json::Value {
    let m = &transform.elements;
    // R = Rx(a)·Ry(b)·Rz(c) ⇒ m02 = sin b; a = atan2(−m12, m22);
    // c = atan2(−m01, m00); gimbal lock at |sin b| → 1 pins c = 0.
    let sb = m[2].clamp(-1.0, 1.0);
    let (a, b, c) = if sb.abs() < 1.0 - 1e-9 {
        (
            (-m[6]).atan2(m[10]),
            sb.asin(),
            (-m[1]).atan2(m[0]),
        )
    } else {
        (m[9].atan2(m[5]), sb.asin(), 0.0)
    };
    serde_json::json!({
        "translate": [m[3], m[7], m[11]],
        "rotateEulerDeg": [a.to_degrees(), b.to_degrees(), c.to_degrees()],
    })
}
