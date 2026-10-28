//! Slot (`SLOT`) — a pin rides a slot (the "slot" mechanical mate).
//! Elements: the pin's axis (a cylindrical face, or a circular or straight
//! edge) and the slot's two end faces (the round ends' cylindrical faces, or
//! their circular edges), the ends on one component. The pin stays parallel
//! to the ends' axes and on the centreline between them, stopping at either
//! end; it is free along the slot unless `pinAlong` holds it at `along`
//! percent from the first end (from the second with `reverse`).

use super::super::mapping::{mate, require_axis, ConstraintFailure, MappedConstraint, ResolvedElement};
use super::{elements_field, id_field, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::{MateCurve, MateKind, Vec3, Vec4};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "slot",
    label: "Slot",
    short_name: "SLOT",
    icon: "\u{2334}",
    long_name: "\u{2334} Slot",
    min_elements: 3,
    max_elements: 3,
    duplicate_family: false,
    applicable,
};

fn applicable(probe: &SelectionProbe) -> bool {
    probe.faces + probe.edges == 3 && probe.components >= 2
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE"], 3, 3,
            "Select the pin (a cylindrical face or circular edge), then the slot's two round ends"),
        "pinAlong": {
            "type": "boolean",
            "default_value": false,
            "label": "Fix position",
            "hint": "Hold the pin at a set place along the slot instead of letting it slide"
        },
        "along": {
            "type": "number",
            "step": 1.0,
            "default_value": 0,
            "label": "Along (%)",
            "hint": "Where along the slot, in percent of its length from the first end picked"
        },
        "reverse": {
            "type": "boolean",
            "default_value": false,
            "hint": "Measure the position from the second end picked"
        },
    }))
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &ConstraintEntry,
    resolved: &[ResolvedElement],
    env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    let [pin, first, second] = resolved else {
        return Err(ConstraintFailure::invalid(format!(
            "Slot takes three elements: the pin and the slot's two ends ({} given)",
            resolved.len()
        )));
    };
    if first.component != second.component {
        return Err(ConstraintFailure::invalid(
            "Both slot ends must belong to one component",
        ));
    }
    if pin.component == first.component {
        return Err(ConstraintFailure::invalid(
            "The pin and the slot must be on different components",
        ));
    }
    let pin_axis = require_axis(pin)?;
    let (a, b) = (require_axis(first)?, require_axis(second)?);
    let normal = Vec3::new(a.direction[0], a.direction[1], a.direction[2]);
    let normal = normal.normalized().map_err(|_| ConstraintFailure::invalid("Slot end axis has no direction"))?;
    let start = Vec3::new(a.origin[0], a.origin[1], a.origin[2]);
    let other = Vec3::new(b.origin[0], b.origin[1], b.origin[2]);
    // The second end's axis, met in the plane of the first end's axis point.
    let end = other.sub(normal.scale(normal.dot(other.sub(start))));
    if end.sub(start).length() <= 1e-9 * start.length().max(end.length()).max(1.0) {
        return Err(ConstraintFailure::invalid(
            "The slot's two ends share one axis: pick the two round ends of the slot",
        ));
    }
    let point = |p: Vec3| Vec4::from_point(p, 1.0);
    let centreline = crate::NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![point(start), point(end)])
        .map_err(|error| ConstraintFailure::new("error", error))?;
    let along = if entry.flag("pinAlong") {
        Some(entry.number("along", env, 0.0).map_err(|error| ConstraintFailure::new("error", error))?)
    } else {
        None
    };
    let kind = MateKind::Slot {
        axis_a: pin_axis,
        curve_b: MateCurve { curve: centreline, t0: 0.0, t1: 1.0 },
        normal_b: [normal.x, normal.y, normal.z],
        along,
        reverse: entry.flag("reverse"),
    };
    Ok(MappedConstraint {
        mates: vec![mate(pin, first, kind)],
        ..Default::default()
    })
}
