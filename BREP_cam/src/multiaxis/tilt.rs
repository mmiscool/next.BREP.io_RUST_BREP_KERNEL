//! Collision avoidance by tilting (lane C10, row 5; spec section 8).
//!
//! For each CL point whose shank or holder collides with the protected
//! mesh ([`clearance_along`] over the chosen tool parts), the axis is tilted
//! about the travel tangent in steps up to `max_tilt_deg`, keeping the
//! pivot (the tip, or the ball centre for ball cutters) fixed; the smallest
//! tilt that clears the solid without gouging the cutting part wins; a point
//! with no clear tilt is reported unreachable, never kept.

use super::contact::{clearance_along, drop_along};
use crate::cutter::{CutterProfile, ToolPart, ToolProfile};
use crate::kinematics::rotate;
use crate::model::{Diagnostic, EmptyRegion, OperationReport, Point3};
use crate::{CamError, MeshSnapshot, Runner, Tolerance, TriangleIndex};

/// One CL point: tip and unit tool axis in setup coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClPose {
    pub tip: Point3,
    pub axis: Point3,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TiltOptions {
    /// Largest tilt away from the requested axis, degrees.
    pub max_tilt_deg: f64,
    /// Tilt increment, degrees.
    pub step_deg: f64,
    /// Tool parts the clearance test covers.
    pub parts: Vec<ToolPart>,
    /// Slab radius growth for tapered bands (see `clearance_along`).
    pub slab: f64,
    /// Distance along the axis from the tip to the point held fixed while
    /// tilting (the ball radius for ball cutters, 0 for a tip pivot).
    pub pivot_offset: f64,
    /// Largest accepted cutting-part gouge (`drop_along` offset) of a tilted
    /// pose; `None` skips the cutting-part check.
    pub gouge_tolerance: Option<f64>,
    /// Look this far beyond the tip when checking the cutting part.
    pub reach: f64,
    /// Fail the report when a pose stays blocked (`false`: the caller trims
    /// the path there and reports it; the poses are still marked unreachable
    /// with an `unreachable` empty region each).
    pub fail_unreachable: bool,
}

impl Default for TiltOptions {
    fn default() -> Self {
        Self {
            max_tilt_deg: 30.0,
            step_deg: 1.0,
            parts: vec![ToolPart::Shank, ToolPart::Holder],
            slab: 0.1,
            pivot_offset: 0.0,
            gouge_tolerance: Some(0.01),
            reach: 1.0,
            fail_unreachable: true,
        }
    }
}

impl TiltOptions {
    pub fn validate(&self) -> Result<(), CamError> {
        let bad = |m: &str| Err(CamError::InvalidInput(format!("tilt: {m}")));
        if !self.max_tilt_deg.is_finite() || self.max_tilt_deg < 0.0 || self.max_tilt_deg >= 90.0 {
            return bad("maximum tilt must be in [0, 90) degrees");
        }
        if !self.step_deg.is_finite() || self.step_deg <= 0.0 {
            return bad("tilt step must be positive");
        }
        if !self.slab.is_finite() || self.slab <= 0.0 {
            return bad("slab must be positive");
        }
        if !self.pivot_offset.is_finite() || self.pivot_offset < 0.0 {
            return bad("pivot offset must be finite and non-negative");
        }
        if !self.reach.is_finite() || self.reach < 0.0 {
            return bad("reach must be finite and non-negative");
        }
        if let Some(g) = self.gouge_tolerance {
            if !g.is_finite() || g < 0.0 {
                return bad("gouge tolerance must be finite and non-negative");
            }
        }
        if self.parts.is_empty() {
            return bad("no tool parts to check");
        }
        Ok(())
    }
}

/// The outcome for one CL point.
#[derive(Clone, Debug, PartialEq)]
pub struct TiltedPose {
    pub pose: ClPose,
    /// Signed tilt applied, degrees (0: untouched): about the travel
    /// tangent, or about the binormal or a diagonal when only a lean clears.
    pub tilt_deg: f64,
    /// Lateral clearance margin of the checked parts after tilting.
    pub gap: f64,
    /// No tilt within the limit cleared the tool: the pose is the requested
    /// one and must not be cut.
    pub unreachable: bool,
}

/// Unit travel tangent at `i` from the neighbouring tips; `None` when the
/// path has no extent there.
fn tangent(poses: &[ClPose], i: usize) -> Option<Point3> {
    let prev = if i > 0 { poses[i - 1].tip } else { poses[i].tip };
    let next = if i + 1 < poses.len() { poses[i + 1].tip } else { poses[i].tip };
    let d = next.sub(prev);
    let a = poses[i].axis;
    d.sub(a.scale(d.dot(a))).normalized()
}

/// Check one pose: the parts' clearance and, when asked, the cutting-part gouge.
fn check(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    profile: &ToolProfile,
    cutter: Option<&dyn CutterProfile>,
    pose: &ClPose,
    opts: &TiltOptions,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<(bool, f64), CamError> {
    let c = clearance_along(mesh, index, profile, &opts.parts, pose.tip, pose.axis, opts.slab, tolerance)?;
    if c.hit.is_some() {
        return Ok((false, c.min_gap));
    }
    if let (Some(g), Some(cutter)) = (opts.gouge_tolerance, cutter) {
        let d = drop_along(mesh, index, cutter, pose.tip, pose.axis, opts.reach, tolerance, runner)?;
        if d.offset > g {
            return Ok((false, c.min_gap));
        }
    }
    Ok((true, c.min_gap))
}

/// Tilt colliding poses away from the part. The returned vector has one
/// entry per input pose, in order; `report` receives one `unreachable`
/// empty region per blocked pose and a warning counting the tilted ones.
pub fn avoid_collisions(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    profile: &ToolProfile,
    cutter: Option<&dyn CutterProfile>,
    poses: &[ClPose],
    opts: &TiltOptions,
    tolerance: Tolerance,
    report: &mut OperationReport,
    runner: &mut Runner<'_>,
) -> Result<Vec<TiltedPose>, CamError> {
    opts.validate()?;
    let mut out = Vec::with_capacity(poses.len());
    let mut tilted = 0usize;
    let mut blocked = 0usize;
    runner.stage("tilt-avoid", "Tilt colliding poses", 0, poses.len() as u64)?;
    for (i, pose) in poses.iter().enumerate() {
        runner.work(1, "tilt-avoid", i as u64, poses.len() as u64)?;
        let axis = pose.axis.normalized().ok_or_else(|| CamError::InvalidInput("tilt: zero tool axis".into()))?;
        let pose = ClPose { tip: pose.tip, axis };
        let (clear, gap) = check(mesh, index, profile, cutter, &pose, opts, tolerance, runner)?;
        if clear {
            out.push(TiltedPose { pose, tilt_deg: 0.0, gap, unreachable: false });
            continue;
        }
        let pivot = pose.tip.add(axis.scale(opts.pivot_offset));
        let t = tangent(poses, i);
        let mut found: Option<TiltedPose> = None;
        if let Some(t) = t {
            let steps = (opts.max_tilt_deg / opts.step_deg).floor() as usize;
            'search: for k in 1..=steps {
                let deg = k as f64 * opts.step_deg;
                for sign in [1.0, -1.0] {
                    let a = rotate(t, (sign * deg).to_radians(), axis);
                    let tip = pivot.sub(a.scale(opts.pivot_offset));
                    let candidate = ClPose { tip, axis: a };
                    let (clear, gap) = check(mesh, index, profile, cutter, &candidate, opts, tolerance, runner)?;
                    if clear {
                        found = Some(TiltedPose { pose: candidate, tilt_deg: sign * deg, gap, unreachable: false });
                        break 'search;
                    }
                }
            }
            // Leaning sideways (about the travel tangent) cannot clear it:
            // lean forward or back (about the binormal), then diagonally,
            // within the same limit. Only poses the sideways search could
            // not place reach this, so every pose it finds is unchanged.
            if found.is_none() {
                if let Some(b) = axis.cross(t).normalized() {
                    let diagonals = [Some(b), t.add(b).normalized(), t.sub(b).normalized()];
                    'lean: for k in 1..=steps {
                        let deg = k as f64 * opts.step_deg;
                        for about in diagonals.iter().flatten() {
                            for sign in [1.0, -1.0] {
                                let a = rotate(*about, (sign * deg).to_radians(), axis);
                                let tip = pivot.sub(a.scale(opts.pivot_offset));
                                let candidate = ClPose { tip, axis: a };
                                let (clear, gap) = check(mesh, index, profile, cutter, &candidate, opts, tolerance, runner)?;
                                if clear {
                                    found = Some(TiltedPose { pose: candidate, tilt_deg: sign * deg, gap, unreachable: false });
                                    break 'lean;
                                }
                            }
                        }
                    }
                }
            }
        }
        match found {
            Some(p) => {
                tilted += 1;
                out.push(p);
            }
            None => {
                blocked += 1;
                report.empty_regions.push(EmptyRegion { reason: "unreachable".into(), target: None });
                out.push(TiltedPose { pose, tilt_deg: 0.0, gap, unreachable: true });
            }
        }
    }
    if tilted > 0 {
        report.warn(Diagnostic::new("tilted-for-clearance", format!("{tilted} CL points tilted to clear the shank or holder")));
    }
    if blocked > 0 && opts.fail_unreachable {
        let first = out.iter().find(|p| p.unreachable).map(|p| p.pose.tip);
        let mut d = Diagnostic::new(
            "unreachable-collision",
            format!("{blocked} CL points collide with the shank or holder at every tilt within {}°", opts.max_tilt_deg),
        );
        if let Some(p) = first {
            d = d.at(p);
        }
        report.fail(d);
    }
    Ok(out)
}
