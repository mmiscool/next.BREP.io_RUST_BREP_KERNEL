//! Shared plumbing for the mechanical mates — [`super::gear`],
//! [`super::rack_pinion`] and [`super::screw`]: element poses, a reference
//! direction ⟂ an axis, the radius an axis selection carries, and the
//! reference capture that makes a new mate hold the poses it is mapped at.

use super::super::mapping::{matrix_to_quaternion, ConstraintFailure, ResolvedElement};
use crate::{MateKind, SelectionGeometry, Vec3};

/// The `(rotation [w,x,y,z], translation)` of the element's component.
pub(super) fn pose_of(element: &ResolvedElement) -> ([f64; 4], [f64; 3]) {
    let m = &element.transform.elements;
    (matrix_to_quaternion(&element.transform), [m[3], m[7], m[11]])
}

/// A unit direction ⟂ `axis` (any one: the spin reference only has to be
/// fixed to the body).
pub(super) fn perpendicular(axis: [f64; 3]) -> [f64; 3] {
    let n = Vec3::new(axis[0], axis[1], axis[2]);
    let helper = if n.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let r = n.cross(helper);
    let r = r.scale(1.0 / r.length());
    [r.x, r.y, r.z]
}

/// The radius of a cylindrical / conical face or circular edge selection.
pub(super) fn radius_of(element: &ResolvedElement) -> Option<f64> {
    match element.world {
        SelectionGeometry::Axis { radius, .. } => radius,
        SelectionGeometry::Circle { radius, .. } => Some(radius),
        _ => None,
    }
    .filter(|radius| radius.is_finite() && *radius > 0.0)
}

/// The references to map with: the ones the last successful solve stored
/// (`mechanicalReference`, while `mechanicalSignature` still matches the
/// elements and parameters), so a typed edit that turns one side is coupled
/// to the other by the next solve; otherwise captured at the elements'
/// current poses, so a new (or re-parameterised) mate moves nothing.
pub(super) fn held_here(
    mut kind: MateKind,
    a: &ResolvedElement,
    b: &ResolvedElement,
    entry: &super::super::ConstraintEntry,
) -> MateKind {
    let stored = (entry.persistent("mechanicalSignature").and_then(|v| v.as_str()) == Some(signature(entry).as_str()))
        .then(|| entry.persistent("mechanicalReference"))
        .flatten()
        .and_then(|v| serde_json::from_value::<Vec<f64>>(v.clone()).ok())
        .filter(|values| values.iter().all(|v| v.is_finite()));
    if !stored.is_some_and(|values| kind.set_references(&values)) {
        kind.rebase(pose_of(a), pose_of(b));
    }
    kind
}

/// What the stored references belong to: the elements and every parameter
/// that changes the coupling. A different signature re-holds at the current
/// poses.
pub(in crate::feature_pipeline::assembly) fn signature(entry: &super::super::ConstraintEntry) -> String {
    let params = &entry.input_params;
    let key = |name: &str| params.get(name).map(|v| v.to_string()).unwrap_or_default();
    format!(
        "{}|{}|{}|{}|{}|{}",
        entry.constraint_type,
        entry.elements().join(","),
        key("ratio"),
        key("distancePerRevolution"),
        key("pitch"),
        key("reverse")
    )
}

/// A positive finite parameter, or `None` for "derive it".
pub(super) fn positive(
    entry: &super::super::ConstraintEntry,
    key: &str,
    env: &crate::feature_pipeline::Env,
) -> Result<Option<f64>, ConstraintFailure> {
    let value = entry.number(key, env, 0.0).map_err(|error| ConstraintFailure::new("error", error))?;
    Ok((value.is_finite() && value > 0.0).then_some(value))
}
