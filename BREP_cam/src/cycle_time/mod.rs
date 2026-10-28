//! Cycle-time estimate from the motion stream.
//!
//! Every motion is timed from its geometry and feed: rapids at the machine's
//! rapid rate, feed moves at their feed (capped by the machine's maximum
//! cutting feed), arcs by their true arc length, hole cycles as the plunge to
//! depth and the retract (pecks add their retracts), dwells by their seconds,
//! tool changes by the machine's tool-change time. With an acceleration limit
//! each move is a trapezoidal (or triangular) velocity profile from rest to
//! rest, which is conservative: real controllers blend corners. Times are in
//! seconds; feeds are per minute.

use crate::model::{ArcPlane, Cycle, Feed, Machine, Motion, MotionKind, Point3};
use serde::{Deserialize, Serialize};

/// Time of one motion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MotionTime {
    pub seconds: f64,
    /// Path length travelled by the tool reference point.
    pub length: f64,
    pub cutting: bool,
}

/// The estimate for a motion stream.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CycleTime {
    pub total_seconds: f64,
    pub cutting_seconds: f64,
    pub rapid_seconds: f64,
    pub dwell_seconds: f64,
    pub tool_change_seconds: f64,
    pub cutting_length: f64,
    pub rapid_length: f64,
    pub tool_changes: u32,
    /// One entry per motion, in order.
    pub per_motion: Vec<MotionTime>,
}

/// Seconds to travel `length` at `feed_per_minute`, from rest to rest under an
/// optional acceleration limit (units/s²): trapezoidal when the cruise speed is
/// reached, triangular otherwise.
pub fn move_seconds(length: f64, feed_per_minute: f64, acceleration: Option<f64>) -> f64 {
    if !(length > 0.0) || !(feed_per_minute > 0.0) {
        return 0.0;
    }
    let v = feed_per_minute / 60.0;
    match acceleration {
        Some(a) if a > 0.0 && a.is_finite() => {
            let ramp = v * v / a; // distance to accelerate and decelerate
            if length >= ramp {
                length / v + v / a
            } else {
                2.0 * (length / a).sqrt()
            }
        }
        _ => length / v,
    }
}

/// Arc length of an XY arc from `start` to `end` about `center` in the given
/// sense, including the helical rise (a full circle when start and end
/// coincide in XY; `pitch` fixes the turns of a helix).
pub fn arc_length(start: Point3, end: Point3, center: Point3, clockwise: bool, pitch: f64) -> f64 {
    use std::f64::consts::TAU;
    let r0 = ((start.x - center.x).powi(2) + (start.y - center.y).powi(2)).sqrt();
    let r1 = ((end.x - center.x).powi(2) + (end.y - center.y).powi(2)).sqrt();
    let radius = 0.5 * (r0 + r1);
    let a0 = (start.y - center.y).atan2(start.x - center.x);
    let a1 = (end.y - center.y).atan2(end.x - center.x);
    let mut sweep = (if clockwise { a0 - a1 } else { a1 - a0 }).rem_euclid(TAU);
    if sweep < 1e-12 {
        sweep = TAU;
    }
    let dz = end.z - start.z;
    if pitch.is_finite() && pitch.abs() > 0.0 && dz.abs() > 0.0 {
        sweep = TAU * dz.abs() / pitch.abs();
    }
    let planar = radius * sweep;
    (planar * planar + dz * dz).sqrt()
}

/// Estimate the time of `motions` on `machine`. The first positional motion
/// places the tool (no time); unknown feeds fall back to the machine limits.
pub fn estimate_cycle_time(motions: &[Motion], machine: &Machine) -> CycleTime {
    let mut out = CycleTime { per_motion: Vec::with_capacity(motions.len()), ..Default::default() };
    let rapid_rate = machine.feed.rapid;
    let max_cut = machine.feed.max_cutting;
    let accel = machine.feed.acceleration;
    let mut position: Option<Point3> = None;
    for motion in motions {
        let mut mt = MotionTime::default();
        match &motion.kind {
            MotionKind::Dwell { seconds } => {
                mt.seconds = seconds.max(0.0);
                out.dwell_seconds += mt.seconds;
            }
            MotionKind::ToolChange { .. } => {
                mt.seconds = machine.tool_change_seconds.max(0.0);
                out.tool_change_seconds += mt.seconds;
                out.tool_changes += 1;
            }
            MotionKind::Cycle { at, cycle } => {
                // Travel to the hole (rapid), plunge to depth at feed, retract at rapid, pecks added.
                let feed = feed_rate(motion.feed, max_cut).unwrap_or(max_cut);
                if let Some(p) = position {
                    let travel = p.distance(*at);
                    let t = move_seconds(travel, rapid_rate, accel);
                    mt.seconds += t;
                    mt.length += travel;
                    out.rapid_seconds += t;
                    out.rapid_length += travel;
                }
                let retract_z = cycle.retract_z().max(at.z);
                let depth_z = match cycle {
                    Cycle::BackBore { entry_z, .. } => *entry_z,
                    other => other.depth_z(),
                };
                let depth = (retract_z - depth_z).abs();
                let (mut feed_len, mut rapid_len) = (depth, depth);
                match cycle {
                    Cycle::Peck { peck, .. } if *peck > 0.0 => {
                        let pecks = (depth / peck).ceil().max(1.0);
                        // Each peck but the last retracts fully and rapids back down to the previous depth.
                        for k in 1..pecks as usize {
                            let reached = (k as f64 * peck).min(depth);
                            rapid_len += 2.0 * reached;
                        }
                    }
                    Cycle::ChipBreak { peck, back_off, .. } if *peck > 0.0 => {
                        let pecks = (depth / peck).ceil().max(1.0);
                        let back = if *back_off > 0.0 { *back_off } else { peck * 0.5 };
                        rapid_len += 2.0 * back * (pecks - 1.0);
                        feed_len += back * (pecks - 1.0);
                    }
                    Cycle::Ream { .. } | Cycle::Tap { .. } => {
                        // Feed out as well.
                        feed_len += depth;
                        rapid_len -= depth;
                    }
                    Cycle::Bore { feed_out: true, .. } => {
                        feed_len += depth;
                        rapid_len -= depth;
                    }
                    _ => {}
                }
                let dwell = match cycle {
                    Cycle::Drill { dwell_seconds, .. } | Cycle::Bore { dwell_seconds, .. } => dwell_seconds.max(0.0),
                    _ => 0.0,
                };
                let tf = move_seconds(feed_len, feed, accel);
                let tr = move_seconds(rapid_len.max(0.0), rapid_rate, accel);
                mt.seconds += tf + tr + dwell;
                mt.length += feed_len + rapid_len.max(0.0);
                mt.cutting = true;
                out.cutting_seconds += tf;
                out.cutting_length += feed_len;
                out.rapid_seconds += tr;
                out.rapid_length += rapid_len.max(0.0);
                out.dwell_seconds += dwell;
                position = Some(Point3::new(at.x, at.y, retract_z));
            }
            kind => {
                if let Some(end) = kind.end() {
                    if let Some(start) = position {
                        let length = match kind {
                            MotionKind::ArcCw { to, center, plane, pitch } | MotionKind::ArcCcw { to, center, plane, pitch } => {
                                if *plane == ArcPlane::Xy {
                                    arc_length(start, *to, *center, matches!(kind, MotionKind::ArcCw { .. }), *pitch)
                                } else {
                                    start.distance(*to)
                                }
                            }
                            _ => start.distance(end),
                        };
                        let is_rapid = matches!(motion.feed, Feed::Rapid(_));
                        let rate = if is_rapid { rapid_rate } else { feed_rate(motion.feed, max_cut).unwrap_or(max_cut) };
                        mt.seconds = move_seconds(length, rate, accel);
                        mt.length = length;
                        mt.cutting = kind.is_cutting();
                        if mt.cutting {
                            out.cutting_seconds += mt.seconds;
                            out.cutting_length += length;
                        } else {
                            out.rapid_seconds += mt.seconds;
                            out.rapid_length += length;
                        }
                    }
                    position = Some(end);
                }
            }
        }
        out.total_seconds += mt.seconds;
        out.per_motion.push(mt);
    }
    out
}

/// A feed move's rate per minute, capped by the machine; `None` for rapids.
fn feed_rate(feed: Feed, max_cutting: f64) -> Option<f64> {
    match feed {
        Feed::Rapid(_) => None,
        Feed::Rate(r) if r.is_finite() && r > 0.0 => Some(r.min(max_cutting)),
        Feed::Rate(_) => Some(max_cutting),
    }
}

