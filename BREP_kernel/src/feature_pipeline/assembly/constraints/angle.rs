//! Angle (`ANGL`) — hold two direction-bearing elements at a target angle.
//! The stored param is the DISPLAY angle (exterior-remapped when the
//! toggle is on); a new constraint adopts the current orientation on first
//! solve, in that same display convention.

use super::super::mapping::{
    angle_between_deg, first_solve_target, mate, require_direction, ConstraintFailure,
    MappedConstraint, ResolvedElement,
};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::MateKind;

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
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE"], 2, 2,
            "Select two direction-bearing elements to hold at an angle"),
        "angle": {
            "type": "number",
            "step": 1,
            "default_value": 90,
            "hint": "Target angle in degrees (a new constraint adopts the current orientation)"
        },
        "exteriorAngle": {
            "type": "boolean",
            "default_value": false,
            "hint": "Measure the exterior (supplementary) angle instead"
        },
    }))
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
    let measured = angle_between_deg(wa, wb);
    let configured = entry
        .number("angle", env, 90.0)
        .map_err(|error| ConstraintFailure::new("error", error))?;
    // The stored param is the DISPLAY angle (exterior-remapped when the toggle
    // is on); first solve adopts the current orientation in that same display
    // convention, so the dialog shows what the user assembled.
    let current_display = if exterior { 180.0 - measured } else { measured };
    let (display_target, pending) =
        first_solve_target(entry, "angle", "initializedAngle", configured, current_display);
    // Interior target for the cosine-residual mate: wrap to [0, 360), fold
    // magnitudes past 180 (cosine-equivalent interior angle), exterior remap.
    let mut interior = display_target.abs() % 360.0;
    if interior > 180.0 {
        interior = 360.0 - interior;
    }
    if exterior {
        interior = 180.0 - interior;
    }
    Ok(MappedConstraint {
        mates: vec![mate(a, b, MateKind::Angle {
            direction_a: [la.x, la.y, la.z],
            direction_b: [lb.x, lb.y, lb.z],
            angle_deg: interior,
        })],
        pending_params: pending.0,
        pending_persistent: pending.1,
        measured: Some((measured, "deg")),
        target: Some(display_target),
        ..Default::default()
    })
}
