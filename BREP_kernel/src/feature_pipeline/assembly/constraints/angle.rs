//! Angle (`ANGL`) — hold two direction-bearing elements at a target angle.
//! The stored param is the DISPLAY angle (exterior-remapped when the
//! toggle is on); a new constraint adopts the current orientation on first
//! solve, in that same display convention.

use super::super::mapping::{
    first_solve_target, mate, require_direction, ConstraintFailure, MappedConstraint,
    ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::assembly_resolve::linear;
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::{MateKind, Vec3};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "angle",
    label: "Angle",
    short_name: "ANGL",
    icon: "\u{2220}",
    long_name: "\u{2220} Angle",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: true,
    applicable,
};

/// Angle pairs direction-bearing faces/edges.
fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces + probe.edges)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(
        &DEF,
        serde_json::json!({
            "id": id_field(),
            "elements": elements_field(&["FACE", "EDGE"], 2, 2,
                "Select two direction-bearing elements to hold at an angle"),
            "angle": {
                "type": "number",
                "step": 1,
                "default_value": 90,
                "hint": "Signed target angle in degrees, including 0–360 (a new constraint adopts the current orientation)"
            },
            "exteriorAngle": {
                "type": "boolean",
                "default_value": false,
                "hint": "Measure the exterior (supplementary) angle instead"
            },
        }),
    )
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &mut ConstraintEntry,
    a: &ResolvedElement,
    b: &ResolvedElement,
    env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    let (wa, la) = require_direction(a)?;
    let (wb, lb) = require_direction(b)?;
    let exterior = entry.flag("exteriorAngle");
    // Keep the reference in A's local frame so it follows rigid assembly
    // motion, but never recapture it as the target crosses zero or 180 degrees.
    // Selection order matters for a signed angle.
    let signature = entry.elements().join("\n");
    let cached = (entry
        .persistent("angleAxisSignature")
        .and_then(|v| v.as_str())
        == Some(signature.as_str()))
    .then(|| entry.persistent("angleAxis").cloned())
    .flatten()
    .and_then(|v| serde_json::from_value::<[f64; 3]>(v).ok())
    .map(|v| Vec3::new(v[0], v[1], v[2]))
    .filter(|v| v.length().is_finite() && v.length() > 1e-9 && v.dot(la).abs() < 1e-8);
    let local_axis = if let Some(axis) = cached {
        axis.scale(1.0 / axis.length())
    } else {
        let mut axis = wa.cross(wb);
        if axis.length() < 1e-9 {
            let helper = if wa.x.abs() < 0.9 {
                Vec3::new(1.0, 0.0, 0.0)
            } else {
                Vec3::new(0.0, 1.0, 0.0)
            };
            axis = wa.cross(helper);
        }
        axis = axis.scale(1.0 / axis.length());
        let inverse = a
            .transform
            .rigid_inverse()
            .map_err(|e| ConstraintFailure::new("error", e))?;
        let local = linear(&inverse, axis);
        entry.set_persistent("angleAxis", serde_json::json!([local.x, local.y, local.z]));
        entry.set_persistent("angleAxisSignature", serde_json::json!(signature));
        local
    };
    let axis = linear(&a.transform, local_axis);
    let measured = axis.dot(wa.cross(wb)).atan2(wa.dot(wb)).to_degrees();
    let configured = entry
        .number("angle", env, 90.0)
        .map_err(|error| ConstraintFailure::new("error", error))?;
    // The stored param is the DISPLAY angle (exterior-remapped when the toggle
    // is on); first solve adopts the current orientation in that same display
    // convention, so the dialog shows what the user assembled.
    let current_display = if exterior { 180.0 - measured } else { measured };
    let (display_target, pending) = first_solve_target(
        entry,
        "angle",
        "initializedAngle",
        configured,
        current_display,
    );
    let interior = if exterior {
        180.0 - display_target
    } else {
        display_target
    };
    // Choose the equivalent measured sweep nearest the target so negative and
    // reflex values survive the overlay/session round-trip (including 360).
    let measured = measured + 360.0 * ((interior - measured) / 360.0).round();
    Ok(MappedConstraint {
        mates: vec![mate(
            a,
            b,
            MateKind::DirectedAngle {
                direction_a: [la.x, la.y, la.z],
                direction_b: [lb.x, lb.y, lb.z],
                axis_a: [local_axis.x, local_axis.y, local_axis.z],
                angle_deg: interior,
            },
        )],
        pending_params: pending.0,
        pending_persistent: pending.1,
        measured: Some((measured, "deg")),
        target: Some(display_target),
        angle_axis: Some(axis),
        ..Default::default()
    })
}
