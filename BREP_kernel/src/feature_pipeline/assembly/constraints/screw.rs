//! Screw (`SCRW`) — a nut advances along a screw by the pitch per turn (the
//! "screw" mechanical mate). Elements in pick order: the screw's axis
//! selection (cylindrical face, circular or straight edge) on one component,
//! then any selection on the nut component. The nut's turn and advance are
//! read relative to the screw, so the screw may move. It does not put the nut
//! on the axis: pair it with a Concentric. `pitch` is the advance per turn;
//! `reverse` makes it a left-hand thread.

use super::super::mapping::{
    local_point, mate, require_axis, ConstraintFailure, MappedConstraint, ResolvedElement,
};
use super::mechanical::{held_here, perpendicular, pose_of};
use super::{elements_field, id_field, pair_applicable, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::assembly::ConstraintEntry;
use crate::feature_pipeline::{Env, SelectionProbe};
use crate::{MateKind, Vec3};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "screw",
    label: "Screw",
    short_name: "SCRW",
    icon: "\u{21BB}",
    long_name: "\u{21BB} Screw",
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
        "elements": elements_field(&["FACE", "EDGE", "VERTEX"], 2, 2,
            "Select the screw's axis face or edge, then an element of the nut"),
        "pitch": {
            "type": "number",
            "step": 0.1,
            "default_value": 1,
            "hint": "Advance along the axis per turn"
        },
        "reverse": {
            "type": "boolean",
            "default_value": false,
            "hint": "Left-hand thread: advance the other way for the same turn"
        },
    }))
}

/// Rotate `v` by the unit quaternion `q = [w, x, y, z]` (or its inverse).
fn rotate(q: [f64; 4], v: Vec3, inverse: bool) -> Vec3 {
    let (w, u) = (q[0], Vec3::new(q[1], q[2], q[3]));
    let u = if inverse { u.scale(-1.0) } else { u };
    let t = u.cross(v).scale(2.0);
    v.add(t.scale(w)).add(u.cross(t))
}

pub(in crate::feature_pipeline::assembly) fn map(
    entry: &ConstraintEntry,
    screw: &ResolvedElement,
    nut: &ResolvedElement,
    env: &Env,
) -> Result<MappedConstraint, ConstraintFailure> {
    let axis_a = require_axis(screw)?;
    let pitch = entry
        .number("pitch", env, 1.0)
        .map_err(|error| ConstraintFailure::new("error", error))?;
    let ref_a = perpendicular(axis_a.direction);
    // The nut's reference is the screw's, carried into the nut's frame, so
    // the captured turn starts at zero.
    let world = rotate(pose_of(screw).0, Vec3::new(ref_a[0], ref_a[1], ref_a[2]), false);
    let ref_b = rotate(pose_of(nut).0, world, true);
    let kind = MateKind::Screw {
        axis_a,
        ref_a,
        ref_b: [ref_b.x, ref_b.y, ref_b.z],
        point_b: local_point(nut),
        pitch,
        reverse: entry.flag("reverse"),
        angle0: 0.0,
        offset0: 0.0,
    };
    Ok(MappedConstraint {
        mates: vec![mate(screw, nut, held_here(kind, screw, nut, entry))],
        note: Some(format!("Pitch {pitch}")),
        ..Default::default()
    })
}
