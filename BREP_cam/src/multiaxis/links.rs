//! Between-run traverses for tilted-axis strategies (lane C10b): the rapid
//! from one run's retract point to the next run's approach point is proven
//! by sampling the chord (axis interpolated) with the tool-frame drop; an
//! unproven chord falls back to a vertical clearance above the whole mesh
//! (lift keeping the axis, cross at the top with a vertical axis, descend
//! along the new axis), whose lift and descent legs are sampled the same
//! way. The caller counts fallbacks (`link-fallback-clearance`) and fails
//! the operation when even the fallback legs are unproven.

use super::contact::drop_along;
use super::slerp;
use super::tilt::ClPose;
use crate::model::{Feed, Motion, Point3};
use crate::{CamError, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};

/// The motions of one traverse and how it was proven.
#[derive(Clone, Debug, PartialEq)]
pub struct Traverse {
    pub motions: Vec<Motion>,
    /// The direct chord was unproven and the vertical clearance was used.
    pub fallback: bool,
    /// A fallback leg could not be proven either.
    pub unproven: bool,
}

/// Sample the straight move from `from` to `to` (axis interpolated) every
/// `step` and report whether the cutting part and shank stay clear.
pub fn chord_clear(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    cutter: &dyn CutterProfile,
    from: ClPose,
    to: ClPose,
    step: f64,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<bool, CamError> {
    let n = ((from.tip.distance(to.tip) / step.max(1e-6)).ceil() as usize).clamp(1, 10_000);
    for i in 0..=n {
        let f = i as f64 / n as f64;
        let p = from.tip.lerp(to.tip, f);
        let a = slerp(from.axis, to.axis, f);
        let d = drop_along(mesh, index, cutter, p, a, 0.0, tolerance, runner)?;
        if d.offset > tolerance.distance_eps {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The traverse from `from` (the previous run's retract pose) to `to` (the
/// next run's approach pose). `top_z` is a Z above the whole mesh plus the
/// clearance; `step` the sampling spacing (the cutter radius).
pub fn traverse(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    cutter: &dyn CutterProfile,
    from: ClPose,
    to: ClPose,
    step: f64,
    top_z: f64,
    path_id: u32,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Traverse, CamError> {
    let rapid = |p: Point3, a: Point3| {
        let mut m = Motion::rapid(p).with_path(path_id);
        m.tool_axis = Some(a);
        m
    };
    if chord_clear(mesh, index, cutter, from, to, step, tolerance, runner)? {
        return Ok(Traverse { motions: vec![rapid(to.tip, to.axis)], fallback: false, unproven: false });
    }
    let lift = ClPose { tip: Point3::new(from.tip.x, from.tip.y, top_z.max(from.tip.z)), axis: from.axis };
    let over = ClPose { tip: Point3::new(to.tip.x, to.tip.y, top_z.max(to.tip.z)), axis: Point3::Z };
    let mut unproven = false;
    if !chord_clear(mesh, index, cutter, from, lift, step, tolerance, runner)? {
        unproven = true;
    }
    if !chord_clear(mesh, index, cutter, over, to, step, tolerance, runner)? {
        unproven = true;
    }
    let mut lift_m = Motion::retract(lift.tip, Feed::RAPID).with_path(path_id);
    lift_m.tool_axis = Some(lift.axis);
    Ok(Traverse {
        motions: vec![lift_m, rapid(over.tip, over.axis), rapid(to.tip, to.axis)],
        fallback: true,
        unproven,
    })
}
