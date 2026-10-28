//! The machine's rotary state while a motion stream is replayed into stock.
//!
//! The Z-dexel stock lives in the PART frame (the frame the part and the
//! stock were modelled in, rotating with any table axis). Motions are given
//! in the MACHINE frame: after `RotaryPosition { C = -90 }` on a mill-turn,
//! a hole the part has at (0, 3) is drilled at machine (3, 0). Before any cut
//! this module maps machine points back into the part frame with the
//! kinematics chain's `Link::Part` placement (the convention the generators
//! use), and it refuses — by name, never by guessing — every state whose
//! part-frame cut a Z-dexel cannot represent:
//! - a nonzero rotary word with no machine chain to interpret it;
//! - an axis name the chain does not have, or angles outside its limits;
//! - a head (tool-carrying) joint away from zero (the tool tilts);
//! - a tool axis that is not vertical in the part frame (4th-axis indexing,
//!   radial live tooling);
//! - cylindrical interpolation (unwrapped radius/arc coordinates).
//! Under polar interpolation the program is written in the workpiece's own
//! XY, so its points are already in the part frame. Synchronized
//! linear+rotary moves are subdivided so every chord stays within the arc
//! chord tolerance in the part frame.

use crate::kinematics::{Carrier, Chain};
use crate::model::{Link, Motion, MotionKind, Point3, RigidTransform};
use crate::numeric::Tolerance;

/// Rotary state of the machine during a replay.
#[derive(Clone, Debug)]
pub struct RotaryFrame<'a> {
    chain: Option<&'a Chain>,
    /// Joint angles in degrees, in chain order (empty without a chain).
    angles: Vec<f64>,
    /// Without a chain: the last commanded angle per axis name.
    loose: Vec<(String, f64)>,
    /// Why the frame cannot be resolved right now (cleared by a resolvable RotaryPosition).
    unknown: Option<String>,
    polar: bool,
    cylindrical: bool,
}

impl<'a> RotaryFrame<'a> {
    pub fn new(chain: Option<&'a Chain>) -> Self {
        let angles = chain.map_or(Vec::new(), |c| vec![0.0; c.joints.len()]);
        Self { chain, angles, loose: Vec::new(), unknown: None, polar: false, cylindrical: false }
    }

    /// Angles a list of rotary words commands, starting from the current state.
    fn target(&self, axes: &[crate::model::RotaryWord]) -> Result<Vec<f64>, String> {
        match self.chain {
            Some(chain) => {
                let mut angles = self.angles.clone();
                for word in axes {
                    let k = chain
                        .joints
                        .iter()
                        .position(|j| j.name.eq_ignore_ascii_case(&word.name))
                        .ok_or_else(|| format!("rotary axis {:?} is not on the machine", word.name))?;
                    if !word.angle_deg.is_finite() {
                        return Err(format!("rotary axis {} angle is not finite", word.name));
                    }
                    angles[k] = word.angle_deg;
                }
                Ok(angles)
            }
            None => Ok(Vec::new()),
        }
    }

    /// Update the state after `motion` (call it after the motion was mapped).
    pub fn commit(&mut self, kind: &MotionKind) {
        match kind {
            MotionKind::RotaryPosition { axes } | MotionKind::LinearRotary { axes, .. } => {
                if self.chain.is_some() {
                    match self.target(axes) {
                        Ok(angles) => {
                            self.angles = angles;
                            self.unknown = None;
                        }
                        Err(reason) => self.unknown = Some(reason),
                    }
                } else {
                    for word in axes {
                        match self.loose.iter_mut().find(|(n, _)| n.eq_ignore_ascii_case(&word.name)) {
                            Some(entry) => entry.1 = word.angle_deg,
                            None => self.loose.push((word.name.clone(), word.angle_deg)),
                        }
                    }
                    self.unknown = self
                        .loose
                        .iter()
                        .find(|(_, a)| *a != 0.0)
                        .map(|(n, a)| format!("rotary axis {n} at {a} degrees needs the machine's kinematics to simulate"));
                }
            }
            MotionKind::PolarInterpolation { enabled } => self.polar = *enabled,
            MotionKind::CylindricalInterpolation { radius } => self.cylindrical = radius.is_some(),
            _ => {}
        }
    }

    /// Machine-to-part transform at `angles`, or why it cannot be used.
    fn part_from_machine(&self, angles: &[f64], tool_axis: Point3, tol: &Tolerance) -> Result<RigidTransform, String> {
        let Some(chain) = self.chain else {
            if tool_axis.distance(Point3::Z) > tol.angle_eps {
                return Err("the tool axis is not vertical (multi-axis motion needs multi-axis simulation)".into());
            }
            return Ok(RigidTransform::identity());
        };
        for (k, j) in chain.joints.iter().enumerate() {
            if j.carrier == Carrier::Head && angles[k] != 0.0 {
                return Err(format!("head axis {} at {} degrees tilts the tool", j.name, angles[k]));
            }
        }
        let place = chain.link_transform(Link::Part, angles, Point3::ZERO).map_err(|e| format!("{e}"))?;
        let to_part = place.inverse();
        let axis = to_part.apply_vector(tool_axis);
        if axis.distance(Point3::Z) > tol.angle_eps {
            return Err("the tool axis is not vertical in the part frame (indexed or radial milling needs multi-axis simulation)".into());
        }
        Ok(to_part)
    }

    /// Map the machine-frame segments of `motion` into the part frame, or say
    /// why the motion cannot be simulated in a Z-dexel stock.
    pub fn to_part(
        &self,
        motion: &Motion,
        segments: &[(Point3, Point3)],
        arc_chord: f64,
        tol: &Tolerance,
    ) -> Result<Vec<(Point3, Point3)>, String> {
        if let Some(reason) = &self.unknown {
            return Err(reason.clone());
        }
        if self.cylindrical {
            return Err("cylindrical interpolation programs unwrapped coordinates; not simulated in a Z-dexel stock".into());
        }
        let tool_axis = motion.tool_axis.and_then(|a| a.normalized()).unwrap_or(Point3::Z);
        if self.polar {
            // Written in the workpiece's own XY: already the part frame.
            if tool_axis.distance(Point3::Z) > tol.angle_eps {
                return Err("polar interpolation with a non-vertical tool axis".into());
            }
            return Ok(segments.to_vec());
        }
        match &motion.kind {
            MotionKind::LinearRotary { axes, .. } => {
                if self.chain.is_none() {
                    return Err("a synchronized linear+rotary move needs the machine's kinematics to simulate".into());
                }
                let target = self.target(axes)?;
                let mut out = Vec::new();
                for &(a, b) in segments {
                    // Enough steps that the rotation's chord error stays within `arc_chord`.
                    let sweep = self.angles.iter().zip(&target).map(|(p, q)| (q - p).abs()).fold(0.0, f64::max).to_radians();
                    // Largest distance of the move from any pivot bounds the swept radius.
                    let pivots = self.chain.map_or(0.0, |c| c.joints.iter().map(|j| j.origin.length()).fold(0.0, f64::max));
                    let reach = a.length().max(b.length()) + pivots + 1.0;
                    let step = 2.0 * (1.0 - (arc_chord / reach).min(1.0)).acos();
                    let n = if sweep > 0.0 && step > 0.0 { ((sweep / step).ceil() as usize).clamp(1, 1 << 16) } else { 1 };
                    let mut prev: Option<Point3> = None;
                    for i in 0..=n {
                        let f = i as f64 / n as f64;
                        let angles: Vec<f64> = self.angles.iter().zip(&target).map(|(p, q)| p + (q - p) * f).collect();
                        let to_part = self.part_from_machine(&angles, tool_axis, tol)?;
                        let p = to_part.apply_point(a.lerp(b, f));
                        if let Some(q) = prev {
                            out.push((q, p));
                        }
                        prev = Some(p);
                    }
                }
                Ok(out)
            }
            _ => {
                let to_part = self.part_from_machine(&self.angles, tool_axis, tol)?;
                Ok(segments.iter().map(|&(a, b)| (to_part.apply_point(a), to_part.apply_point(b))).collect())
            }
        }
    }
}
