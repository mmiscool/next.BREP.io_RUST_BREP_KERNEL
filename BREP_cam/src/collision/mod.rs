//! Collision checks during simulation: shank and holder against stock and
//! protected bodies, and any non-cutting move (rapid, link, retract) that
//! passes through material.
//!
//! The check runs in time order over each move with the Z-dexel stock: at a
//! sample parameter `t` the tool's envelope ([`ToolProfile`]) is compared
//! against the material of every column under its footprint, minus what the
//! move's own cutting band has removed up to `t` (the flutes reach a point
//! before the shank does on a descent, so that material is not a collision).
//! The first colliding sample is refined by bisection against the last clear
//! one, so a plunge with a too-short holder reports the depth where the
//! holder meets the stock. Protected bodies are checked the same way against
//! their own grid, with no removal credit. After the check the move is cut
//! exactly ([`crate::stock_sim::removal`]).

use crate::cancel::{Cancelled, Runner};
use crate::cutter::{Cutter, CutterProfile, ToolPart, ToolProfile};
use crate::model::{Bounds3, Cycle, Motion, MotionKind, Point3};
use crate::numeric::Tolerance;
use crate::stock_sim::frame::RotaryFrame;
use crate::stock_sim::removal::{arc_chords_in_plane, cut_back_bore, cut_linear, refuse, RemovalOptions, SimulationReport};
use crate::stock_sim::DexelGrid;
use serde::{Deserialize, Serialize};

/// Which part of the tool touched.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CollisionPart {
    /// The cutting part met material on a move that does not cut (rapid, link, retract).
    CuttingOnRapid,
    Shank,
    Holder,
}

/// What was touched.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CollisionAgainst {
    Stock,
    Protected,
}

/// One reported contact: the first time a part of the tool met material
/// during a motion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Collision {
    pub motion_index: usize,
    pub part: CollisionPart,
    pub against: CollisionAgainst,
    /// Tool reference point at first contact (machine coordinates).
    pub position: Point3,
    /// Column centre and the lowest colliding material Z.
    pub contact: Point3,
}

/// Collision check parameters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct CollisionOptions {
    /// Stop simulating at the first collision (the stock stays as it was before that move).
    pub stop_at_first: bool,
    /// Sample spacing along a move (`0`: the stock grid's cell size).
    pub sample_step: f64,
    /// Check rapids, links and retracts for passing through material.
    pub check_rapids: bool,
    /// Bisection steps refining the first-contact parameter.
    pub refine_steps: u32,
}

impl Default for CollisionOptions {
    fn default() -> Self {
        Self { stop_at_first: false, sample_step: 0.0, check_rapids: true, refine_steps: 30 }
    }
}

/// Result of a checked simulation.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CheckedSimulation {
    pub removal: SimulationReport,
    pub collisions: Vec<Collision>,
    /// Index of the motion the simulation stopped at (`stop_at_first`), if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stopped_at: Option<usize>,
}

/// Material in `column` that the tool at reference `c` touches with the
/// parts named by `bands` (absolute Z bands with their part), outside the
/// credit band `credit` already removed by this move. An overlap no deeper
/// than `tangent` is tangency, not contact. Returns the lowest colliding Z
/// and its part.
fn touch(column: &crate::stock_sim::Column, c_z: f64, bands: &[([f64; 2], ToolPart)], credit: Option<[f64; 2]>, tangent: f64) -> Option<(f64, ToolPart)> {
    let mut best: Option<(f64, ToolPart)> = None;
    for &([h0, h1], part) in bands {
        let (z0, z1) = (c_z + h0, c_z + h1);
        for &[a, b] in &column.intervals {
            // Overlap of material [a, b] with the band [z0, z1].
            let lo = a.max(z0);
            let hi = b.min(z1);
            if hi <= lo {
                continue;
            }
            // Remove the credit band.
            let pieces: Vec<[f64; 2]> = match credit {
                Some([ca, cb]) if cb > lo && ca < hi => {
                    let mut v = Vec::new();
                    if lo < ca {
                        v.push([lo, ca]);
                    }
                    if cb < hi {
                        v.push([cb, hi]);
                    }
                    v
                }
                _ => vec![[lo, hi]],
            };
            for [p0, p1] in pieces {
                if p1 - p0 > tangent && best.map_or(true, |(z, _)| p0 < z) {
                    best = Some((p0, part));
                }
            }
        }
    }
    best
}

/// Scan one move against `grid` in time order. `cutting` says whether the
/// move removes material (then the cutting part is excluded and credited);
/// `credit_cutter` is the cutter for the credit band. Returns the first
/// collision (lowest material Z at the earliest colliding sample, refined).
#[allow(clippy::too_many_arguments)]
fn scan_move(
    grid: &DexelGrid,
    against: CollisionAgainst,
    profile: &ToolProfile,
    cutter: &Cutter,
    cutting: bool,
    start: Point3,
    end: Point3,
    motion_index: usize,
    options: &CollisionOptions,
    tol: &Tolerance,
) -> Option<Collision> {
    let r_max = profile.max_radius();
    let footprint = Bounds3::from_points([start, end]).expanded(r_max);
    let [ix0, ix1, iy0, iy1] = grid.columns_in(&footprint)?;
    let d = end.sub(start);
    let length = d.length();
    let step = if options.sample_step > 0.0 { options.sample_step } else { grid.cell_x.min(grid.cell_y) };
    let samples = if length > 0.0 { ((length / step).ceil() as usize).clamp(1, 1 << 20) } else { 1 };
    let cut_len = cutter.cutting_length();
    let cut_radius = cutter.radius();
    // Per-column state: prefix cut credit [floor, top] accumulated along the move.
    let ncols = ((ix1 - ix0 + 1) * (iy1 - iy0 + 1)) as usize;
    let mut credit: Vec<Option<[f64; 2]>> = vec![None; ncols];
    // At the START of a move the tool sits where the previous move left it,
    // and that move cut every column it touched down to the tool's own
    // surface. The tool envelope and the cut surface agree there only to
    // round-off (C19-17/18: a tapered flank 4e-17 into the column it just
    // finished), so an overlap no deeper than `distance_eps` at t = 0 is
    // tangency. Every later sample, and the bisection, stay strict: a move
    // that drags into material is flagged as soon as it is any deeper.
    let start_tangent = tol.distance_eps.max(0.0);
    let collides_at = |t: f64, credit: &mut [Option<[f64; 2]>], update_credit: bool| -> Option<(f64, ToolPart, Point3)> {
        let tangent = if t == 0.0 { start_tangent } else { 0.0 };
        let c = start.add(d.scale(t));
        let mut hit: Option<(f64, ToolPart, Point3)> = None;
        let mut k = 0;
        for iy in iy0..=iy1 {
            for ix in ix0..=ix1 {
                let (x, y) = grid.center(ix, iy);
                let r = ((x - c.x).powi(2) + (y - c.y).powi(2)).sqrt();
                let column = grid.column(ix, iy);
                let slot = k;
                k += 1;
                if column.is_empty() || r > r_max {
                    continue;
                }
                // Removal credit from this move up to now.
                if cutting && update_credit && r <= cut_radius {
                    let floor = c.z + cutter.height_at_radius(r).unwrap_or(0.0);
                    let top = c.z + cut_len;
                    credit[slot] = Some(match credit[slot] {
                        Some([f, tp]) => [f.min(floor), tp.max(top)],
                        None => [floor, top],
                    });
                }
                let bands = profile.height_bands_reaching(r);
                let bands: Vec<([f64; 2], ToolPart)> = if cutting {
                    bands.into_iter().filter_map(|([h0, h1], part)| {
                        let lo = h0.max(cut_len);
                        (h1 > lo).then_some(([lo, h1], part))
                    }).collect()
                } else {
                    bands
                };
                if let Some((z, part)) = touch(column, c.z, &bands, if cutting { credit[slot] } else { None }, tangent) {
                    if hit.map_or(true, |(hz, _, _)| z < hz) {
                        hit = Some((z, part, Point3::new(x, y, z)));
                    }
                }
            }
        }
        hit
    };
    let mut last_clear = None;
    for k in 0..=samples {
        let t = k as f64 / samples as f64;
        if let Some((_, part, contact)) = collides_at(t, &mut credit, true) {
            // Refine between the last clear sample and this one (credit frozen).
            let (mut lo, mut hi) = (last_clear.unwrap_or(t), t);
            let mut best = (part, contact);
            if last_clear.is_some() {
                let mut frozen = credit.clone();
                for _ in 0..options.refine_steps {
                    let mid = 0.5 * (lo + hi);
                    match collides_at(mid, &mut frozen, false) {
                        Some((_, p, c)) => {
                            hi = mid;
                            best = (p, c);
                        }
                        None => lo = mid,
                    }
                    if (hi - lo) * length <= 1e-3 * tol.distance_eps {
                        break;
                    }
                }
            }
            let position = start.add(d.scale(hi));
            let part = match best.0 {
                ToolPart::Cutting => CollisionPart::CuttingOnRapid,
                ToolPart::Shank => CollisionPart::Shank,
                ToolPart::Holder => CollisionPart::Holder,
            };
            return Some(Collision { motion_index, part, against, position, contact: best.1 });
        }
        last_clear = Some(t);
    }
    None
}

/// Run a motion stream through the stock with collision checks. Every move is
/// checked against the stock as it is BEFORE the move (with the move's own
/// cutting credited) and against `protected`, then cut. With
/// `stop_at_first` the stock is left as it was before the colliding move.
/// Without a machine, a nonzero rotary position or a synchronized rotary move
/// is refused by name; use [`simulate_checked_on_machine`] for streams that
/// index or rotate the part.
#[allow(clippy::too_many_arguments)]
pub fn simulate_checked(
    grid: &mut DexelGrid,
    protected: Option<&DexelGrid>,
    cutter: &Cutter,
    profile: &ToolProfile,
    motions: &[Motion],
    tol: &Tolerance,
    removal: &RemovalOptions,
    options: &CollisionOptions,
    runner: &mut Runner<'_>,
) -> Result<CheckedSimulation, Cancelled> {
    checked_impl(grid, protected, cutter, profile, motions, tol, removal, options, None, runner)
}

/// [`simulate_checked`] on a machine: rotary positioning is applied through
/// `chain`, so checks and cuts happen in the stock's part frame (collision
/// positions are reported in that frame too). `protected` is a grid in the
/// same part frame (fixtures that rotate with the part, such as a chuck).
#[allow(clippy::too_many_arguments)]
pub fn simulate_checked_on_machine(
    grid: &mut DexelGrid,
    protected: Option<&DexelGrid>,
    cutter: &Cutter,
    profile: &ToolProfile,
    motions: &[Motion],
    tol: &Tolerance,
    removal: &RemovalOptions,
    options: &CollisionOptions,
    chain: &crate::kinematics::Chain,
    runner: &mut Runner<'_>,
) -> Result<CheckedSimulation, Cancelled> {
    checked_impl(grid, protected, cutter, profile, motions, tol, removal, options, Some(chain), runner)
}

#[allow(clippy::too_many_arguments)]
fn checked_impl(
    grid: &mut DexelGrid,
    protected: Option<&DexelGrid>,
    cutter: &Cutter,
    profile: &ToolProfile,
    motions: &[Motion],
    tol: &Tolerance,
    removal: &RemovalOptions,
    options: &CollisionOptions,
    chain: Option<&crate::kinematics::Chain>,
    runner: &mut Runner<'_>,
) -> Result<CheckedSimulation, Cancelled> {
    let mut out = CheckedSimulation { removal: SimulationReport { motions: motions.len() as u64, ..Default::default() }, ..Default::default() };
    let mut frame = RotaryFrame::new(chain);
    let mut position: Option<Point3> = None;
    'motions: for (index, motion) in motions.iter().enumerate() {
        runner.stage("stock-simulation", "checking and cutting motion", index as u64, motions.len() as u64)?;
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
                refuse(&mut out.removal, index, "back boring requires finite upward depths and a flat cutting profile".into());
                position = Some(end);
                frame.commit(&motion.kind);
                continue;
            }
            // Transform both cutting-edge endpoints: an offset machine frame can
            // change Z as well as the bore's XY location.
            let world = [(Point3::new(at.x, at.y, *entry_z), Point3::new(at.x, at.y, *depth_z))];
            let part = match frame.to_part(motion, &world, removal.arc_chord, tol) {
                Ok(part) => part,
                Err(reason) => {
                    refuse(&mut out.removal, index, reason);
                    position = Some(end);
                    frame.commit(&motion.kind);
                    continue;
                }
            };
            refuse(&mut out.removal, index, "asymmetric back-bore head entry/exit clearance is not modeled".into());
            for (entry, finished) in part {
                let cut = cut_back_bore(grid, cutter.radius(), entry, entry.z, finished.z, runner)?;
                out.removal.removed_volume += cut.removed_volume;
                out.removal.columns_touched += cut.columns_touched;
            }
            out.removal.cutting_motions += 1;
            out.removal.warnings.push(crate::model::Diagnostic::new("back-bore-access-not-certified", "G87 upward removal is simulated; asymmetric shifted head entry/exit clearance is not modeled"));
            position = Some(end);
            frame.commit(&motion.kind);
            continue;
        }
        let cutting = matches!(
            motion.kind,
            MotionKind::Plunge { .. }
                | MotionKind::Cut { .. }
                | MotionKind::ArcCw { .. }
                | MotionKind::ArcCcw { .. }
                | MotionKind::LeadIn { .. }
                | MotionKind::LeadOut { .. }
                | MotionKind::Cycle { .. }
                | MotionKind::LinearRotary { .. }
        );
        let world: Vec<(Point3, Point3)> = match &motion.kind {
            MotionKind::ArcCw { to, center, plane, pitch } | MotionKind::ArcCcw { to, center, plane, pitch } => {
                let clockwise = matches!(motion.kind, MotionKind::ArcCw { .. });
                let mut prev = start;
                arc_chords_in_plane(start, *to, *center, *plane, clockwise, *pitch, removal.arc_chord)
                    .into_iter()
                    .map(|p| {
                        let seg = (prev, p);
                        prev = p;
                        seg
                    })
                    .collect()
            }
            MotionKind::Cycle { at, cycle } => {
                let top = Point3::new(at.x, at.y, cycle.retract_z().max(at.z));
                let depth = cycle.depth_z();
                vec![(start, top), (top, Point3::new(at.x, at.y, depth))]
            }
            _ => vec![(start, end)],
        };
        let segments = match frame.to_part(motion, &world, removal.arc_chord, tol) {
            Ok(part) => part,
            Err(reason) => {
                refuse(&mut out.removal, index, reason);
                position = Some(end);
                frame.commit(&motion.kind);
                continue;
            }
        };
        if cutting {
            out.removal.cutting_motions += 1;
        }
        for (a, b) in segments {
            if cutting || options.check_rapids {
                let mut found: Vec<Collision> = Vec::new();
                if let Some(c) = scan_move(grid, CollisionAgainst::Stock, profile, cutter, cutting, a, b, index, options, tol) {
                    found.push(c);
                }
                if let Some(p) = protected {
                    if let Some(c) = scan_move(p, CollisionAgainst::Protected, profile, cutter, false, a, b, index, options, tol) {
                        found.push(c);
                    }
                }
                if !found.is_empty() {
                    out.collisions.extend(found);
                    if options.stop_at_first {
                        out.stopped_at = Some(index);
                        break 'motions;
                    }
                }
            }
            if cutting {
                let cut = cut_linear(grid, cutter, a, b, tol, removal, runner)?;
                out.removal.removed_volume += cut.removed_volume;
                out.removal.columns_touched += cut.columns_touched;
                out.removal.unconverged_columns += cut.unconverged_columns;
            }
        }
        position = Some(end);
        frame.commit(&motion.kind);
    }
    Ok(out)
}

