//! Gear (`GEAR`) — couple two components' spins about their own axes by a
//! ratio (the "gear" mechanical mate). Elements: two axis-bearing selections
//! (cylindrical or conical faces, circular edges, straight edges), one on
//! each component. `ratio` is B's turns per turn of A — teeth(A)/teeth(B); 0
//! takes it from the two radii (rA/rB), else 1. The default sense is an
//! external mesh (opposite spins); `reverse` turns them the same way. The
//! spins are read against references captured where the gears stand when the
//! solve starts, so a new Gear moves nothing and a drag of one gear turns the
//! other.

use super::super::mapping::{require_axis, ConstraintFailure, MappedConstraint, ResolvedElement, mate};
use super::mechanical::{held_here, perpendicular, positive, radius_of};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::MateKind;

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "gear",
    label: "Gear",
    short_name: "GEAR",
    icon: "\u{2699}",
    long_name: "\u{2699} Gear",
    min_elements: 2,
    max_elements: 2,
    duplicate_family: false,
    applicable,
};

fn applicable(probe: &SelectionProbe) -> bool {
    pair_applicable(probe, probe.faces + probe.edges)
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE"], 2, 2,
            "Select the two gears' axis faces or edges (cylindrical faces, circular or straight edges)"),
        "ratio": {
            "type": "number",
            "step": 0.1,
            "default_value": 0,
            "hint": "Turns of the second gear per turn of the first (teeth of the first / teeth of the second); 0 takes it from the two radii"
        },
        "reverse": {
            "type": "boolean",
            "default_value": false,
            "hint": "Turn both gears the same way (an internal mesh or a belt) instead of opposite ways"
        },
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &ConstraintEntry,
    a: &ResolvedElement,
    b: &ResolvedElement,
    env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    let (axis_a, axis_b) = (require_axis(a)?, require_axis(b)?);
    let ratio = match positive(entry, "ratio", env)? {
        Some(ratio) => ratio,
        None => match (radius_of(a), radius_of(b)) {
            (Some(ra), Some(rb)) => ra / rb,
            _ => 1.0,
        },
    };
    let kind = MateKind::Gear {
        axis_a,
        ref_a: perpendicular(axis_a.direction),
        world_ref_a: [0.0; 3],
        axis_b,
        ref_b: perpendicular(axis_b.direction),
        world_ref_b: [0.0; 3],
        ratio,
        reverse: entry.flag("reverse"),
    };
    Ok(MappedConstraint {
        mates: vec![mate(a, b, held_here(kind, a, b, entry))],
        note: Some(format!("Ratio {ratio:.4}")),
        ..Default::default()
    })
}
