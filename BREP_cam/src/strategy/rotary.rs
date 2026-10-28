//! Rotary (4th-axis) strategies (lane C10, row 3; spec section 6).
//!
//! - [`wrap_toolpath`]: a flat 3-axis toolpath generated in the unwrapped
//!   frame `(u, v, w)` (axial position, arc length on the wrap cylinder,
//!   radial height above it) wrapped onto the cylinder; every wrapped CL
//!   point is certified against the real mesh with the tool-frame drop.
//! - [`generate_rotary_finish`]: rotary finishing with the tool axis radial
//!   to the rotary axis, line passes at angular steps or one spiral pass,
//!   every sample dropped radially with the exact drop cutter and chords
//!   refined until the midpoint contact is within the operation tolerance.
//!
//! Conventions: the rotary direction is the setup axis of the letter (`A`
//! = X, `B` = Y) through the setup origin; the flat `v` coordinate is the
//! rotary angle times the wrap radius; the material under the tool at
//! rotary angle `θ` sits at setup angle `−θ`, so a point is placed by
//! `rotate(d, −θ, ρ·Z)` and the tool axis is `rotate(d, −θ, Z)`. Heights
//! are radii in the unwrapped frame (`clearance` is the clearance radius).

use crate::kinematics::rotate;
use crate::model::*;
use crate::multiaxis::contact::drop_along;
use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};
use std::f64::consts::TAU;

/// Unit rotary direction of an axis letter.
pub fn rotary_direction(axis: &str) -> Result<Point3, CamError> {
    match axis {
        "A" | "a" => Ok(Point3::X),
        "B" | "b" => Ok(Point3::Y),
        other => Err(CamError::InvalidInput(format!("rotary axis must be A or B, got {other:?}"))),
    }
}

/// Unwrapped-frame reading of a flat point for a rotary direction: `(u, v, z)`.
fn unwrapped(d: Point3, p: Point3) -> (f64, f64, f64) {
    if d == Point3::X {
        (p.x, p.y, p.z)
    } else {
        (p.y, p.x, p.z)
    }
}

/// Place a point at axial position `u`, rotary angle `theta` (radians) and
/// radius `rho`; returns `(position, tool_axis)`.
pub fn place(d: Point3, u: f64, theta: f64, rho: f64) -> (Point3, Point3) {
    let radial = rotate(d, -theta, Point3::Z);
    (d.scale(u).add(radial.scale(rho)), radial)
}

/// Wrap one flat point: `(position, tool_axis, w)`.
fn wrap_point(d: Point3, wrap_radius: f64, p: Point3) -> (Point3, Point3, f64) {
    let (u, v, z) = unwrapped(d, p);
    let (pos, axis) = place(d, u, v / wrap_radius, z);
    (pos, axis, z - wrap_radius)
}

/// Wrap `flat` (a usable 3-axis toolpath in the unwrapped frame of
/// `params`) onto the cylinder. Arcs are refused (the inner strategy must
/// leave them as lines). A cut is split so each wrapped chord stays within a
/// quarter of the tolerance of the cylinder, and every wrapped cut is
/// certified with the tool-frame drop against `mesh` along its whole length
/// (sampled, not only at its ends: C16 sweep finding S3); a gouge beyond
/// `operation.tolerance` fails the operation.
pub fn wrap_toolpath(
    operation: &Operation,
    params: &RotaryWrapParams,
    flat: &Toolpath,
    cutter: &Cutter,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    operation.validate().map_err(CamError::InvalidInput)?;
    if !params.wrap_diameter.is_finite() || params.wrap_diameter <= 0.0 {
        return Err(CamError::InvalidInput("rotary wrap: wrap diameter must be positive".into()));
    }
    let d = rotary_direction(&params.axis)?;
    cutter.validate()?;
    tolerance.validate()?;
    if !flat.report.is_usable() {
        return Err(CamError::InvalidInput("rotary wrap: the flat toolpath is not usable".into()));
    }
    let wrap_radius = 0.5 * params.wrap_diameter;
    let mut out = Toolpath::new(operation.id.clone(), operation.tool.clone());
    out.report = flat.report.clone();
    let mut max_distortion = 0.0f64;
    let mut gouges = 0u64;
    let mut first_gouge: Option<Point3> = None;
    runner.stage("rotary-wrap", "Wrap and certify", 0, flat.motions.len() as u64)?;
    // Samples at most a tolerance apart (at least 0.01, at most a quarter radius).
    let step = operation.tolerance.max(0.01).min((0.25 * cutter.radius()).max(0.01));
    let mut prev_flat: Option<Point3> = None;
    for (i, m) in flat.motions.iter().enumerate() {
        runner.work(1, "rotary-wrap", i as u64, flat.motions.len() as u64)?;
        let mut mapped = m.clone();
        match &m.kind {
            MotionKind::ArcCw { .. } | MotionKind::ArcCcw { .. } => {
                return Err(CamError::InvalidInput("rotary wrap: linearise arcs before wrapping".into()));
            }
            MotionKind::Cycle { .. } => {
                return Err(CamError::InvalidInput("rotary wrap: canned cycles cannot be wrapped".into()));
            }
            other => {
                if let Some(p) = other.end() {
                    let (pos, axis, w) = wrap_point(d, wrap_radius, p);
                    if other.is_cutting() {
                        let (_, v, _) = unwrapped(d, p);
                        max_distortion = max_distortion.max((v * w / wrap_radius).abs());
                    }
                    mapped.kind = match other {
                        MotionKind::Rapid { .. } => MotionKind::Rapid { to: pos },
                        MotionKind::Plunge { .. } => MotionKind::Plunge { to: pos },
                        MotionKind::Cut { .. } => MotionKind::Cut { to: pos },
                        MotionKind::Retract { .. } => MotionKind::Retract { to: pos },
                        MotionKind::LeadIn { .. } => MotionKind::LeadIn { to: pos },
                        MotionKind::LeadOut { .. } => MotionKind::LeadOut { to: pos },
                        MotionKind::Link { .. } => MotionKind::Link { to: pos },
                        k => k.clone(),
                    };
                    mapped.tool_axis = Some(axis);
                    if let (MotionKind::Cut { .. }, Some(from)) = (other, prev_flat) {
                        // Pieces whose wrapped chords stay within tol/4 of the cylinder.
                        let (_, v0, z0) = unwrapped(d, from);
                        let (_, v1, z1) = unwrapped(d, p);
                        let dtheta = (v1 - v0).abs() / wrap_radius;
                        let rmax = z0.abs().max(z1.abs()).max(1e-9);
                        let s = (0.25 * operation.tolerance / rmax).min(1.0);
                        let per = 2.0 * (1.0 - s).acos();
                        let pieces = if per > 0.0 { ((dtheta / per).ceil() as usize).clamp(1, 100_000) } else { 1 };
                        for k in 1..=pieces {
                            let a = from.lerp(p, (k - 1) as f64 / pieces as f64);
                            let b = if k == pieces { p } else { from.lerp(p, k as f64 / pieces as f64) };
                            // Certify the whole piece, interpolated as the machine moves it.
                            let (pa, _, _) = wrap_point(d, wrap_radius, a);
                            let (pb, _, _) = wrap_point(d, wrap_radius, b);
                            let n = ((pa.distance(pb) / step).ceil() as usize).clamp(1, 100_000);
                            for j in 1..=n {
                                let (q, qa, _) = wrap_point(d, wrap_radius, a.lerp(b, j as f64 / n as f64));
                                let drop = drop_along(mesh, index, cutter, q, qa, cutter.radius(), tolerance, runner)?;
                                out.report.counts.samples += 1;
                                out.report.counts.candidates += drop.candidates;
                                if drop.offset > operation.tolerance {
                                    gouges += 1;
                                    first_gouge.get_or_insert(q);
                                }
                            }
                            if k < pieces {
                                let (q, qa, _) = wrap_point(d, wrap_radius, b);
                                let mut piece = m.clone();
                                piece.kind = MotionKind::Cut { to: q };
                                piece.tool_axis = Some(qa);
                                out.motions.push(piece);
                            }
                        }
                    } else if other.is_cutting() {
                        let drop = drop_along(mesh, index, cutter, pos, axis, cutter.radius(), tolerance, runner)?;
                        out.report.counts.samples += 1;
                        out.report.counts.candidates += drop.candidates;
                        if drop.offset > operation.tolerance {
                            gouges += 1;
                            first_gouge.get_or_insert(pos);
                        }
                    }
                    prev_flat = Some(p);
                }
            }
        }
        out.motions.push(mapped);
    }
    if max_distortion > 0.0 {
        out.report.approximations.push(ApproximationLimit {
            kind: "wrap-distortion".into(),
            bound: Some(max_distortion),
            detail: Some("arc length scales with radius away from the wrap diameter".into()),
        });
    }
    if gouges > 0 {
        let mut dg = Diagnostic::new("gouge-detected", format!("{gouges} wrapped cut samples gouge the part beyond the machining tolerance"));
        if let Some(p) = first_gouge {
            dg = dg.at(p);
        }
        out.report.fail(dg);
    }
    out.report.counts.motions = out.motions.len() as u64;
    Ok(out)
}

/// One sample of a rotary pass: axial position, rotary angle, tip radius.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Sample {
    u: f64,
    theta: f64,
    rho: f64,
}

#[derive(Clone, Copy)]
struct Dropper<'a> {
    mesh: &'a MeshSnapshot,
    index: Option<&'a dyn TriangleIndex>,
    cutter: &'a Cutter,
    d: Point3,
    stock_radius: f64,
    tolerance: Tolerance,
    samples: u64,
    candidates: u64,
}

impl Dropper<'_> {
    /// Tip radius where the cutter first touches along the radial at `(u, theta)`.
    fn drop(&mut self, u: f64, theta: f64, runner: &mut Runner<'_>) -> Result<f64, CamError> {
        let (tip, axis) = place(self.d, u, theta, self.stock_radius);
        let r = drop_along(self.mesh, self.index, self.cutter, tip, axis, self.stock_radius, self.tolerance, runner)?;
        self.samples += 1;
        self.candidates += r.candidates;
        Ok(self.stock_radius + r.offset)
    }

    /// Whether the move `a`-`b` dips below the drop height by more than
    /// `limit` anywhere along it. The move is linear in the unwrapped frame
    /// (axial position, angle and radius together, as the machine
    /// interpolates the rotary axis with the linear ones), sampled at most
    /// `limit` apart (at least 0.01, at most a quarter cutter radius) and held
    /// to half the limit, so a dip between two samples stays within it.
    fn chord_dips(&mut self, a: Sample, b: Sample, limit: f64, runner: &mut Runner<'_>) -> Result<bool, CamError> {
        let (pa, _) = place(self.d, a.u, a.theta, a.rho);
        let (pb, _) = place(self.d, b.u, b.theta, b.rho);
        let step = limit.max(0.01).min((0.25 * self.cutter.radius()).max(0.01));
        let n = ((pa.distance(pb) / step).ceil() as usize).clamp(1, 100_000);
        for k in 1..n {
            let f = k as f64 / n as f64;
            let (u, theta, rho) = (a.u + (b.u - a.u) * f, a.theta + (b.theta - a.theta) * f, a.rho + (b.rho - a.rho) * f);
            if self.drop(u, theta, runner)? - rho > 0.5 * limit {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// The samples to cut from `a` to `b` (`b` included) so no move dips
    /// below the drop height by more than `limit`: halved at the midpoint
    /// contact while a move dips, to `depth`. Where the drop height jumps
    /// (a step's face) halving never converges, so the last move gets a
    /// radial riser: up along the tool's own axis at the lower end, which
    /// cannot cut into the part, then the level move, which must certify.
    /// `None`: not even that certifies, and the pass must break here.
    fn certified(&mut self, a: Sample, b: Sample, limit: f64, depth: u32, runner: &mut Runner<'_>) -> Result<Option<Vec<Sample>>, CamError> {
        if !self.chord_dips(a, b, limit, runner)? {
            return Ok(Some(vec![b]));
        }
        if depth == 0 {
            if b.rho >= a.rho {
                let riser = Sample { rho: b.rho, ..a };
                return Ok((!self.chord_dips(riser, b, limit, runner)?).then(|| vec![riser, b]));
            }
            let riser = Sample { rho: a.rho, ..b };
            return Ok((!self.chord_dips(a, riser, limit, runner)?).then(|| vec![riser, b]));
        }
        let (u, theta) = (0.5 * (a.u + b.u), 0.5 * (a.theta + b.theta));
        let mid = Sample { u, theta, rho: self.drop(u, theta, runner)? };
        let Some(mut left) = self.certified(a, mid, limit, depth - 1, runner)? else { return Ok(None) };
        let Some(right) = self.certified(mid, b, limit, depth - 1, runner)? else { return Ok(None) };
        left.extend(right);
        Ok(Some(left))
    }
}

/// How deep a dipping rotary move is halved before it gets a riser.
const ROTARY_REFINE_DEPTH: u32 = 12;

/// Extend `passes` with `raw`, broken wherever a move cannot be certified
/// (`breaks` counts the breaks); passes of a single sample are dropped. The
/// moves are independent, so with the `parallel` feature they are certified
/// on the pool, each with its own dropper, and joined in order: the result is
/// the serial one.
fn push_certified(dropper: &mut Dropper<'_>, raw: &[Sample], limit: f64, passes: &mut Vec<Vec<Sample>>, breaks: &mut u64, runner: &mut Runner<'_>) -> Result<(), CamError> {
    let Some(&first) = raw.first() else { return Ok(()) };
    let moves: Vec<(Sample, Sample)> = raw.windows(2).map(|w| (w[0], w[1])).collect();
    let template = Dropper { samples: 0, candidates: 0, ..*dropper };
    let one = |&(a, b): &(Sample, Sample), worker: &mut Runner<'_>| {
        let mut own = Dropper { samples: 0, candidates: 0, ..template };
        let cut = own.certified(a, b, limit, ROTARY_REFINE_DEPTH, worker);
        (cut, own.samples, own.candidates)
    };
    #[cfg(feature = "parallel")]
    let certified = crate::par::map_ordered(&moves, runner, "rotary-certify", one)?;
    #[cfg(not(feature = "parallel"))]
    let certified: Vec<_> = moves.iter().map(|m| one(m, runner)).collect();
    let mut pass = vec![first];
    for ((_, b), (cut, samples, candidates)) in moves.iter().zip(certified) {
        dropper.samples += samples;
        dropper.candidates += candidates;
        match cut? {
            Some(cut) => pass.extend(cut),
            None => {
                *breaks += 1;
                if pass.len() > 1 {
                    passes.push(std::mem::take(&mut pass));
                }
                pass = vec![*b];
            }
        }
    }
    if pass.len() > 1 {
        passes.push(pass);
    }
    Ok(())
}

/// Generate rotary finishing. `mesh` is the complete protected mesh in
/// setup coordinates with the rotary axis through the origin.
pub fn generate_rotary_finish(
    operation: &Operation,
    params: &RotaryFinishParams,
    cutter: &Cutter,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    default_heights: &Heights,
    default_feeds: &FeedsSpeeds,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    operation.validate().map_err(CamError::InvalidInput)?;
    if !operation.enabled {
        return Err(CamError::InvalidInput("operation is disabled".into()));
    }
    params.validate().map_err(|e| CamError::InvalidInput(format!("rotary: {e}")))?;
    cutter.validate()?;
    tolerance.validate()?;
    let heights = operation.heights.as_ref().unwrap_or(default_heights);
    let feeds = operation.feeds.as_ref().unwrap_or(default_feeds);
    heights.validate().map_err(CamError::InvalidInput)?;
    feeds.validate().map_err(CamError::InvalidInput)?;
    if mesh.is_empty() {
        return Err(CamError::EmptyTarget { operation_id: operation.id.to_string() });
    }
    for solid in &operation.targets.target_solids {
        if !mesh.faces.iter().any(|f| &f.solid == solid) {
            return Err(CamError::UnresolvedTarget { solid: solid.clone(), face: None });
        }
    }
    let d = rotary_direction(&params.axis)?;
    let stock_radius = 0.5 * params.stock_diameter;
    // The mesh must fit inside the stock cylinder (it is where passes start).
    let max_r = mesh.triangles.iter().flat_map(|t| t.vertices()).map(|p| p.sub(d.scale(p.dot(d))).length()).fold(0.0, f64::max);
    if max_r > stock_radius + tolerance.distance_eps {
        return Err(CamError::InvalidInput("rotary: the part extends beyond the stock diameter".into()));
    }
    if heights.clearance <= stock_radius {
        return Err(CamError::InvalidInput("rotary: clearance radius must exceed the stock radius".into()));
    }
    let [u0, u1] = params.range.unwrap_or_else(|| {
        let (lo, hi) = mesh.triangles.iter().flat_map(|t| t.vertices()).map(|p| p.dot(d)).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)));
        [lo, hi]
    });
    if !(u1 > u0) {
        return Err(CamError::InvalidInput("rotary: empty axial range".into()));
    }
    let radial_allowance = operation.stock_to_leave.radial;
    if radial_allowance < 0.0 {
        return Err(CamError::InvalidInput("rotary: negative stock allowance".into()));
    }
    let mut dropper = Dropper { mesh, index, cutter, d, stock_radius, tolerance, samples: 0, candidates: 0 };
    let chord = operation.tolerance;
    let n_axial = ((u1 - u0) / params.sample_step).ceil().max(1.0);
    if n_axial > 1_000_000.0 {
        return Err(CamError::InvalidInput("rotary: sample work budget exceeded".into()));
    }
    let n_axial = n_axial as usize;
    // Build the passes as sample sequences in the unwrapped frame, every
    // move certified against the drop height (C16 sweep finding S2).
    let mut passes: Vec<Vec<Sample>> = Vec::new();
    let mut breaks = 0u64;
    if params.spiral {
        let pitch = params.stepover.distance;
        let turns = (u1 - u0) / pitch;
        let per_turn = (TAU * stock_radius / params.sample_step).ceil().max(4.0);
        let n = ((turns * per_turn).ceil() as usize).max(n_axial);
        if n > 5_000_000 {
            return Err(CamError::InvalidInput("rotary: spiral work budget exceeded".into()));
        }
        runner.stage("rotary-spiral", "Spiral samples", 0, n as u64 + 1)?;
        let mut raw = Vec::with_capacity(n + 1);
        for i in 0..=n {
            runner.work(1, "rotary-spiral", i as u64, n as u64 + 1)?;
            let f = i as f64 / n as f64;
            let u = u0 + (u1 - u0) * f;
            let theta = TAU * turns * f;
            raw.push(Sample { u, theta, rho: dropper.drop(u, theta, runner)? });
        }
        push_certified(&mut dropper, &raw, chord, &mut passes, &mut breaks, runner)?;
    } else {
        let dtheta = params.stepover.distance / stock_radius;
        let n_pass = (TAU / dtheta).ceil().max(1.0);
        if n_pass * n_axial as f64 > 5_000_000.0 {
            return Err(CamError::InvalidInput("rotary: pass work budget exceeded".into()));
        }
        let n_pass = n_pass as usize;
        let dtheta = TAU / n_pass as f64;
        runner.stage("rotary-lines", "Line passes", 0, n_pass as u64)?;
        for k in 0..n_pass {
            runner.work(1, "rotary-lines", k as u64, n_pass as u64)?;
            let theta = k as f64 * dtheta;
            let mut raw = Vec::with_capacity(n_axial + 1);
            for i in 0..=n_axial {
                let u = u0 + (u1 - u0) * i as f64 / n_axial as f64;
                raw.push(Sample { u, theta, rho: dropper.drop(u, theta, runner)? });
            }
            if params.zig_zag && k % 2 == 1 {
                raw.reverse();
            }
            push_certified(&mut dropper, &raw, chord, &mut passes, &mut breaks, runner)?;
        }
    }
    // Emit: radial rapid to clearance, plunge to the first contact, cuts,
    // retract; links at the clearance radius in rotary steps of at most 10°.
    let mut out = Toolpath::new(operation.id.clone(), operation.tool.clone());
    let clearance = heights.clearance;
    let mut at: Option<(f64, f64)> = None; // (u, theta) at the clearance radius
    for (pid, pass) in passes.iter().enumerate() {
        let first = pass[0];
        let path_id = pid as u32;
        match at {
            None => {
                let (p, a) = place(d, first.u, first.theta, clearance);
                let mut m = Motion::rapid(p).with_path(path_id);
                m.tool_axis = Some(a);
                out.motions.push(m);
            }
            Some((u, theta)) => {
                let steps = (((first.theta - theta).abs() / 10f64.to_radians()).ceil() as usize).max(1);
                for s in 1..=steps {
                    let f = s as f64 / steps as f64;
                    let (p, a) = place(d, u + (first.u - u) * f, theta + (first.theta - theta) * f, clearance);
                    let mut m = Motion::rapid(p).with_path(path_id);
                    m.tool_axis = Some(a);
                    out.motions.push(m);
                }
            }
        }
        let (p, a) = place(d, first.u, first.theta, first.rho + radial_allowance);
        let mut m = Motion::plunge(p, feeds.plunge_feed).with_path(path_id);
        m.tool_axis = Some(a);
        out.motions.push(m);
        for s in &pass[1..] {
            let (p, a) = place(d, s.u, s.theta, s.rho + radial_allowance);
            let mut m = Motion::cut(p, feeds.cutting_feed).with_path(path_id);
            m.tool_axis = Some(a);
            out.motions.push(m);
        }
        let last = pass[pass.len() - 1];
        let (p, a) = place(d, last.u, last.theta, clearance);
        let mut m = Motion::retract(p, Feed::RAPID).with_path(path_id);
        m.tool_axis = Some(a);
        out.motions.push(m);
        at = Some((last.u, last.theta));
        out.report.counts.passes += 1;
    }
    out.report.counts.triangles = mesh.triangle_count() as u64;
    out.report.counts.samples = dropper.samples;
    out.report.counts.candidates = dropper.candidates;
    out.report.counts.motions = out.motions.len() as u64;
    out.report.approximations.push(ApproximationLimit {
        kind: "chord-tolerance".into(),
        bound: Some(chord),
        detail: Some(format!("every rotary move certified to half the tolerance against the drop height, sampled every {:.3} (halved to depth {ROTARY_REFINE_DEPTH}, then a radial riser)", chord.max(0.01).min((0.25 * cutter.radius()).max(0.01)))),
    });
    if breaks > 0 {
        out.report.warn(Diagnostic::new("chord-split", format!("{breaks} rotary moves could not be certified against the drop height even with a riser: the passes break there, retract and re-enter")));
    }
    if passes.is_empty() {
        out.report.fail(Diagnostic::new("empty-after-clipping", "no rotary pass could be certified"));
    }
    out.report.finish();
    Ok(out)
}

