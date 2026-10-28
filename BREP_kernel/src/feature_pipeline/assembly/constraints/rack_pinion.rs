//! Rack and Pinion (`RACK`) — turn the pinion's spin into the rack's travel
//! (the "rack and pinion" mechanical mate). Elements in pick order: the
//! pinion's axis selection (cylindrical face or circular edge), then a
//! selection on the rack whose direction is the travel (a straight edge, or a
//! planar face whose normal is the travel). `distancePerRevolution` is the
//! rack travel per pinion turn; 0 takes 2π × the pinion's radius (the pitch
//! circle when the pinion face is it). `reverse` flips the sense.

use super::super::mapping::{
    local_point, mate, require_axis, require_direction, ConstraintFailure, MappedConstraint,
    ResolvedElement,
};
use super::mechanical::{held_here, perpendicular, positive, radius_of};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::MateKind;

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "rack_pinion",
    label: "Rack and Pinion",
    short_name: "RACK",
    icon: "\u{21CC}",
    long_name: "\u{21CC} Rack and Pinion",
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
            "Select the pinion's axis face or circular edge, then a straight edge (or planar face) of the rack along its travel"),
        "distancePerRevolution": {
            "type": "number",
            "step": 0.1,
            "default_value": 0,
            "hint": "Rack travel per pinion turn; 0 takes 2π × the pinion's radius"
        },
        "reverse": {
            "type": "boolean",
            "default_value": false,
            "hint": "Move the rack the other way for the same turn"
        },
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &ConstraintEntry,
    pinion: &ResolvedElement,
    rack: &ResolvedElement,
    env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    let pinion_axis = require_axis(pinion)?;
    let (_, travel) = require_direction(rack)?;
    let per_turn = match positive(entry, "distancePerRevolution", env)? {
        Some(value) => value,
        None => match radius_of(pinion) {
            Some(radius) => std::f64::consts::TAU * radius,
            None => {
                return Err(ConstraintFailure::new(
                    "error",
                    "Set the rack travel per revolution: the pinion selection has no radius to take it from.",
                ))
            }
        },
    };
    let kind = MateKind::RackPinion {
        pinion_axis,
        pinion_ref: perpendicular(pinion_axis.direction),
        world_pinion_ref: [0.0; 3],
        rack_point: local_point(rack),
        rack_direction: [travel.x, travel.y, travel.z],
        world_rack_point: [0.0; 3],
        distance_per_revolution: per_turn,
        reverse: entry.flag("reverse"),
    };
    Ok(MappedConstraint {
        mates: vec![mate(pinion, rack, held_here(kind, pinion, rack, entry))],
        note: Some(format!("{per_turn:.4} per revolution")),
        ..Default::default()
    })
}
