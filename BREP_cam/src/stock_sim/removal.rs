//! Exact 3-axis material removal into a [`DexelGrid`].
//!
//! For every column under a move, the lowest point the cutter's CUTTING
//! profile reaches during the move is computed exactly: for a flat cutter in
//! closed form, for the other profiles by bracketed minimization of the convex
//! function `c_z(t) + height_at_radius(r(t))` over the covered part of the
//! move. The column is then cut between that floor and the top of the
//! cutting band, so shank and holder never remove material (they are
//! collision checks, module `collision`). Arcs and helices are discretised
//! into chords within a chord tolerance. Only the XY sampling is discrete.

use super::DexelGrid;
use crate::cancel::{Cancelled, Runner};
use crate::cutter::{Cutter, CutterKind, CutterProfile};
use crate::model::{ArcPlane, Bounds3, Cycle, Diagnostic, Motion, MotionKind, Point3};
use crate::numeric::Tolerance;
use serde::{Deserialize, Serialize};

/// Removal parameters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct RemovalOptions {
    /// Chord deviation allowed when discretising arcs and helices.
    pub arc_chord: f64,
    /// Iteration cap of the per-column floor minimization (a warning when hit).
    pub max_floor_iterations: u32,
}

impl Default for RemovalOptions {
    fn default() -> Self {
        Self { arc_chord: 0.01, max_floor_iterations: 200 }
    }
}

/// What one move removed.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MoveCut {
    pub removed_volume: f64,
    pub columns_touched: u64,
    /// Columns whose floor minimization hit the iteration cap.
    pub unconverged_columns: u64,
}

/// What a whole motion stream removed.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SimulationReport {
    pub removed_volume: f64,
    pub motions: u64,
    pub cutting_motions: u64,
    pub columns_touched: u64,
    pub unconverged_columns: u64,
    /// Motion indices that could not be fully simulated/certified (multi-axis,
    /// non-XY arcs, or asymmetric back-bore access in checked simulation).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub unsupported: Vec<usize>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<Diagnostic>,
}

/// The exact lowest Z the cutter's reference point reaches over column
/// `(x, y)` while moving linearly from `start` to `end`, plus the top of the
/// cutting band there. `None` when the cutter never covers the column. The
/// `bool` is `true` when the minimization converged within the cap.
pub fn column_floor(
    cutter: &Cutter,
    start: Point3,
    end: Point3,
    x: f64,
    y: f64,
    tol: &Tolerance,
    max_iterations: u32,
) -> Option<(f64, f64, bool)> {
    let radius = cutter.radius();
    let d = end.sub(start);
    let sx = start.x - x;
    let sy = start.y - y;
    // |s + t d|² <= R²  ->  a t² + b t + c <= 0.
    let a = d.x * d.x + d.y * d.y;
    let b = 2.0 * (sx * d.x + sy * d.y);
    let c = sx * sx + sy * sy - radius * radius;
    let (t0, t1) = if a <= f64::EPSILON * (1.0 + sx * sx + sy * sy) {
        if c > 0.0 {
            return None;
        }
        (0.0, 1.0)
    } else {
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return None;
        }
        let s = disc.sqrt();
        let lo = ((-b - s) / (2.0 * a)).max(0.0);
        let hi = ((-b + s) / (2.0 * a)).min(1.0);
        if lo > hi {
            return None;
        }
        (lo, hi)
    };
    let z_at = |t: f64| start.z + t * d.z;
    let r_at = |t: f64| {
        let px = sx + t * d.x;
        let py = sy + t * d.y;
        (px * px + py * py).sqrt().min(radius)
    };
    let f = |t: f64| z_at(t) + cutter.height_at_radius(r_at(t)).unwrap_or(0.0);
    let top = z_at(t0).max(z_at(t1)) + cutter.cutting_length();
    if cutter.kind() == CutterKind::Flat || cutter.kind() == CutterKind::Dovetail {
        // Height is zero everywhere under the flat bottom: the floor is the lower endpoint.
        return Some((z_at(t0).min(z_at(t1)), top, true));
    }
    // Convex on [t0, t1]: golden-section search, endpoints included.
    let mut lo = t0;
    let mut hi = t1;
    let mut best = f(lo).min(f(hi));
    if hi - lo <= 0.0 {
        return Some((best, top, true));
    }
    const PHI: f64 = 0.618_033_988_749_894_9;
    let mut m1 = hi - PHI * (hi - lo);
    let mut m2 = lo + PHI * (hi - lo);
    let mut f1 = f(m1);
    let mut f2 = f(m2);
    let mut converged = false;
    for _ in 0..max_iterations {
        best = best.min(f1).min(f2);
        if (f1 - f2).abs() <= 1e-3 * tol.distance_eps && (hi - lo) * d.length() <= 1e-3 * tol.distance_eps {
            converged = true;
            break;
        }
        if f1 < f2 {
            hi = m2;
            m2 = m1;
            f2 = f1;
            m1 = hi - PHI * (hi - lo);
            f1 = f(m1);
        } else {
            lo = m1;
            m1 = m2;
            f1 = f2;
            m2 = lo + PHI * (hi - lo);
            f2 = f(m2);
        }
        if hi - lo <= 1e-15 {
            converged = true;
            break;
        }
    }
    best = best.min(f1).min(f2);
    Some((best, top, converged))
}

/// Remove the material a linear move from `start` to `end` cuts.
pub fn cut_linear(
    grid: &mut DexelGrid,
    cutter: &Cutter,
    start: Point3,
    end: Point3,
    tol: &Tolerance,
    options: &RemovalOptions,
    runner: &mut Runner<'_>,
) -> Result<MoveCut, Cancelled> {
    let r = cutter.radius();
    let footprint = Bounds3::from_points([start, end]).expanded(r);
    let mut out = MoveCut::default();
    let Some([ix0, ix1, iy0, iy1]) = grid.columns_in(&footprint) else {
        return Ok(out);
    };
    let area = grid.cell_area();
    for iy in iy0..=iy1 {
        for ix in ix0..=ix1 {
            let (x, y) = grid.center(ix, iy);
            if let Some((floor, top, converged)) = column_floor(cutter, start, end, x, y, tol, options.max_floor_iterations) {
                if !converged {
                    out.unconverged_columns += 1;
                }
                let removed = grid.column_mut(ix, iy).cut(floor, top);
                if removed > 0.0 {
                    out.removed_volume += removed * area;
                    out.columns_touched += 1;
                }
            }
        }
        runner.work((ix1 - ix0 + 1) as u64, "stock-removal", 0, 0)?;
    }
    Ok(out)
}

/// G87 sweeps an upward-facing cutting edge from entry to finished back face.
/// The reference Z is that edge, not the bottom of a downward cutting band.
/// Only this interval is removed; neither approach nor shank clears stock.
/// The model has no asymmetric boring-head envelope: this is a removal
/// certificate, not a certificate of shifted entry/exit clearance.
pub(crate) fn cut_back_bore(grid: &mut DexelGrid, radius: f64, at: Point3, entry: f64, finished: f64, runner: &mut Runner<'_>) -> Result<MoveCut, Cancelled> {
    let mut out=MoveCut::default();
    let bounds=Bounds3::new(Point3::new(at.x-radius,at.y-radius,entry),Point3::new(at.x+radius,at.y+radius,finished));
    if let Some([x0,x1,y0,y1])=grid.columns_in(&bounds) {
        let area=grid.cell_area();
        for y in y0..=y1 {for x in x0..=x1 {
            let (cx,cy)=grid.center(x,y);
            if (cx-at.x).hypot(cy-at.y)<=radius {
                let removed=grid.column_mut(x,y).cut(entry,finished);
                if removed>0. {out.removed_volume+=removed*area;out.columns_touched+=1;}
            }
        }runner.work((x1-x0+1) as u64,"back-bore-removal",0,0)?;}
    }
    Ok(out)
}

/// Discretise an XY arc (or helix) from `start` to `end` about `center` into
/// chords within `chord` and return the chord endpoints after `start`.
/// `pitch` (Z per full turn) fixes the number of turns of a helix; without it
/// the sweep is the shorter arc in the given sense, a full circle when start
/// and end coincide in XY.
/// Chord points of an arc in any plane (G17/G18/G19), ending exactly at
/// `end`: XY arcs through [`arc_chords`], XZ and YZ arcs by angle in their own
/// plane ([`crate::arcfit::arc_point`]) with the same chord bound, so a
/// parallel finish fitted to vertical arcs is cut like any other motion. An
/// arc whose radius is zero in its plane is the straight move to `end`.
pub fn arc_chords_in_plane(start: Point3, end: Point3, center: Point3, plane: ArcPlane, clockwise: bool, pitch: f64, chord: f64) -> Vec<Point3> {
    if plane == ArcPlane::Xy {
        return arc_chords(start, end, center, clockwise, pitch, chord);
    }
    let Some((sweep, radius)) = crate::arcfit::arc_sweep(start, end, center, plane, clockwise) else {
        return vec![end];
    };
    let kind = if clockwise { MotionKind::ArcCw { to: end, center, plane, pitch } } else { MotionKind::ArcCcw { to: end, center, plane, pitch } };
    let max_step = 2.0 * (1.0 - (chord / radius).min(1.0)).acos();
    let n = if max_step > 0.0 { ((sweep / max_step).ceil().max(1.0) as usize).clamp(1, 1 << 16) } else { 1 };
    (1..=n).map(|i| if i == n { end } else { crate::arcfit::arc_point(start, &kind, i as f64 / n as f64).unwrap_or(end) }).collect()
}

pub fn arc_chords(start: Point3, end: Point3, center: Point3, clockwise: bool, pitch: f64, chord: f64) -> Vec<Point3> {
    use std::f64::consts::{PI, TAU};
    let r0 = ((start.x - center.x).powi(2) + (start.y - center.y).powi(2)).sqrt();
    let r1 = ((end.x - center.x).powi(2) + (end.y - center.y).powi(2)).sqrt();
    let radius = 0.5 * (r0 + r1);
    if !(radius > 0.0) {
        return vec![end];
    }
    let a0 = (start.y - center.y).atan2(start.x - center.x);
    let a1 = (end.y - center.y).atan2(end.x - center.x);
    // Signed sweep in the requested sense, in (0, 2π].
    let mut sweep = if clockwise { a0 - a1 } else { a1 - a0 };
    sweep = sweep.rem_euclid(TAU);
    if sweep < 1e-12 {
        sweep = TAU;
    }
    let dz = end.z - start.z;
    if pitch.is_finite() && pitch.abs() > 0.0 && dz.abs() > 0.0 {
        // A helix: the pitch fixes the total sweep (|dz| / pitch turns). The
        // chord list still ends exactly at `end`, so a pitch that disagrees
        // with the explicit end angle is absorbed in the last chord.
        sweep = TAU * dz.abs() / pitch.abs();
    }
    let max_step = 2.0 * (1.0 - (chord / radius).min(1.0)).acos();
    let n = if max_step > 0.0 { (sweep / max_step).ceil().max(1.0) as usize } else { 1 };
    let n = n.clamp(1, 1 << 16);
    let mut out = Vec::with_capacity(n);
    for i in 1..=n {
        let s = i as f64 / n as f64;
        if i == n {
            out.push(end);
            break;
        }
        let ang = if clockwise { a0 - s * sweep } else { a0 + s * sweep };
        let rr = r0 + (r1 - r0) * s;
        out.push(Point3::new(center.x + rr * ang.cos(), center.y + rr * ang.sin(), start.z + dz * s));
    }
    let _ = PI;
    out
}

/// Which motions remove material.
fn is_removing(kind: &MotionKind) -> bool {
    matches!(
        kind,
        MotionKind::Plunge { .. }
            | MotionKind::Cut { .. }
            | MotionKind::ArcCw { .. }
            | MotionKind::ArcCcw { .. }
            | MotionKind::LeadIn { .. }
            | MotionKind::LeadOut { .. }
            | MotionKind::Cycle { .. }
            | MotionKind::LinearRotary { .. }
    )
}

/// Run a motion stream through the stock. The first positional motion places
/// the tool without cutting; every removing motion cuts from the tool's
/// current position to its end. Rapids, links and retracts never cut (the
/// collision module checks them). Motions with a tilted tool axis and arcs
/// are recorded as unsupported and skipped; arcs in any plane are cut along
/// their chords.
pub fn simulate(
    grid: &mut DexelGrid,
    cutter: &Cutter,
    motions: &[Motion],
    tol: &Tolerance,
    options: &RemovalOptions,
    runner: &mut Runner<'_>,
) -> Result<SimulationReport, Cancelled> {
    simulate_impl(grid, cutter, motions, tol, options, None, runner)
}

/// [`simulate`] on a machine: rotary positioning (`RotaryPosition`,
/// `LinearRotary`) is applied through `chain` so every cut lands in the
/// stock's part frame. Motions whose part-frame cut a Z-dexel cannot
/// represent are reported as unsupported with a named warning (see
/// [`super::frame`]); without a chain, any nonzero rotary word is refused.
pub fn simulate_on_machine(
    grid: &mut DexelGrid,
    cutter: &Cutter,
    motions: &[Motion],
    tol: &Tolerance,
    options: &RemovalOptions,
    chain: &crate::kinematics::Chain,
    runner: &mut Runner<'_>,
) -> Result<SimulationReport, Cancelled> {
    simulate_impl(grid, cutter, motions, tol, options, Some(chain), runner)
}

/// Record an unsupported motion with its named reason (one warning per distinct reason).
pub(crate) fn refuse(report: &mut SimulationReport, index: usize, reason: String) {
    report.unsupported.push(index);
    if !report.warnings.iter().any(|w| w.code == "motion-not-simulated" && w.detail.as_deref() == Some(reason.as_str())) {
        report.warnings.push(Diagnostic::new("motion-not-simulated", "a motion cannot be simulated in the Z-dexel stock").detail(reason));
    }
}

fn simulate_impl(
    grid: &mut DexelGrid,
    cutter: &Cutter,
    motions: &[Motion],
    tol: &Tolerance,
    options: &RemovalOptions,
    chain: Option<&crate::kinematics::Chain>,
    runner: &mut Runner<'_>,
) -> Result<SimulationReport, Cancelled> {
    let mut report = SimulationReport { motions: motions.len() as u64, ..Default::default() };
    if matches!(cutter.kind(), CutterKind::Lollipop | CutterKind::Dovetail) {
        report.warnings.push(Diagnostic::new(
            "undercut-not-simulated",
            "undercut surfaces (lollipop upper ball, dovetail flank) are not simulated by the Z-dexel stock; only the lower profile removes material",
        ));
    }
    let mut frame = super::frame::RotaryFrame::new(chain);
    let mut position: Option<Point3> = None;
    for (index, motion) in motions.iter().enumerate() {
        runner.stage("stock-removal", "simulating motion", index as u64, motions.len() as u64)?;
        let Some(end) = motion.end() else {
            frame.commit(&motion.kind);
            continue;
        };
        let Some(start) = position else {
            position = Some(end);
            frame.commit(&motion.kind);
            continue;
        };
        if let MotionKind::Cycle { at, cycle: Cycle::BackBore { entry_z, depth_z, .. } } = &motion.kind {
            if !at.is_finite() || !entry_z.is_finite() || !depth_z.is_finite()
                || entry_z >= depth_z || cutter.kind() != crate::cutter::CutterKind::Flat
            {
                refuse(&mut report, index, "back boring requires finite upward depths and a flat cutting profile".into());
                position = Some(end);
                frame.commit(&motion.kind);
                continue;
            }
            // Transform both cutting-edge endpoints: an offset machine frame can
            // change Z as well as the bore's XY location.
            let world = [(Point3::new(at.x, at.y, *entry_z), Point3::new(at.x, at.y, *depth_z))];
            let part = match frame.to_part(motion, &world, options.arc_chord, tol) {
                Ok(part) => part,
                Err(reason) => {
                    refuse(&mut report, index, reason);
                    position = Some(end);
                    frame.commit(&motion.kind);
                    continue;
                }
            };

            for (entry, finished) in part {
                let cut = cut_back_bore(grid, cutter.radius(), entry, entry.z, finished.z, runner)?;
                report.removed_volume += cut.removed_volume;
                report.columns_touched += cut.columns_touched;
            }
            report.cutting_motions += 1;
            report.warnings.push(crate::model::Diagnostic::new("back-bore-access-not-certified", "G87 upward removal is simulated; asymmetric shifted head entry/exit clearance is not modeled"));
            position = Some(end);
            frame.commit(&motion.kind);
            continue;
        }
        let removing = is_removing(&motion.kind);
        let world: Vec<(Point3, Point3)> = match &motion.kind {
            MotionKind::ArcCw { to, center, plane, pitch } | MotionKind::ArcCcw { to, center, plane, pitch } if removing => {
                let clockwise = matches!(motion.kind, MotionKind::ArcCw { .. });
                let mut prev = start;
                arc_chords_in_plane(start, *to, *center, *plane, clockwise, *pitch, options.arc_chord)
                    .into_iter()
                    .map(|p| {
                        let seg = (prev, p);
                        prev = p;
                        seg
                    })
                    .collect()
            }
            MotionKind::Cycle { at, cycle } => {
                // A hole cycle plunges on the axis from the retract plane to the depth.
                let top = Point3::new(at.x, at.y, cycle.retract_z().max(at.z));
                let depth = cycle.depth_z();
                vec![(top, Point3::new(at.x, at.y, depth))]
            }
            _ => vec![(start, end)],
        };
        let part = match frame.to_part(motion, &world, options.arc_chord, tol) {
            Ok(part) => part,
            Err(reason) => {
                refuse(&mut report, index, reason);
                position = Some(end);
                frame.commit(&motion.kind);
                continue;
            }
        };
        if removing {
            report.cutting_motions += 1;
            for (a, b) in part {
                let cut = cut_linear(grid, cutter, a, b, tol, options, runner)?;
                report.removed_volume += cut.removed_volume;
                report.columns_touched += cut.columns_touched;
                report.unconverged_columns += cut.unconverged_columns;
            }
        }
        position = Some(end);
        frame.commit(&motion.kind);
    }
    if report.unconverged_columns > 0 {
        report.warnings.push(
            Diagnostic::new("floor-solve-cap", "per-column floor minimization hit its iteration cap")
                .detail(format!("{} columns", report.unconverged_columns)),
        );
    }
    Ok(report)
}
