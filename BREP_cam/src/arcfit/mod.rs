//! Arc fitting (lane C4, row 2): replaces runs of straight cut chords with
//! G2/G3 arcs, planar or helical, in the XY, XZ or YZ plane.
//!
//! A fitted arc passes through the run's first and last point and stays
//! within `tolerance` of every sampled point it replaces, and every chord it
//! replaces stays within `tolerance` of the arc (its sagitta), so the posted
//! path never leaves the machining tolerance band of the certified polyline.
//! Fitting is greedy and deterministic: from an anchor, the arc is extended
//! point by point until a point fails, the arc is emitted, and the failing
//! point starts the next run. An arc replaces at least three chords (any three
//! points lie on some circle; the fourth verifies it). A run that no arc fits
//! stays lines.
//!
//! Plane frames follow the controller planes. In each frame `u × v = w`, `w`
//! is the plane normal, and "clockwise" is seen looking down `w`:
//! `Xy`: (u, v, w) = (X, Y, Z) (G17); `Xz`: (Z, X, Y) (G18); `Yz`: (Y, Z, X) (G19).

use crate::model::{ApproximationLimit, ArcPlane, Motion, MotionKind, Point3, Toolpath};
use crate::CamError;
use std::f64::consts::{PI, TAU};

/// Fitting limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArcFitOptions {
    /// Maximum deviation between the arc and the points/chords it replaces.
    pub tolerance: f64,
    /// Smallest radius worth an arc (controllers refuse tiny arcs).
    pub min_radius: f64,
    /// Largest radius worth an arc (beyond it a line is as good).
    pub max_radius: f64,
    /// Allow helical arcs (the third coordinate advances linearly with angle).
    pub helical: bool,
    /// Largest sweep of one arc in degrees (360 permits full circles).
    pub max_sweep_deg: f64,
    /// Planes to try, in order of preference.
    pub planes: [Option<ArcPlane>; 3],
}

impl Default for ArcFitOptions {
    fn default() -> Self {
        Self {
            tolerance: 0.01,
            min_radius: 0.05,
            max_radius: 10_000.0,
            helical: true,
            max_sweep_deg: 360.0,
            planes: [Some(ArcPlane::Xy), Some(ArcPlane::Xz), Some(ArcPlane::Yz)],
        }
    }
}

impl ArcFitOptions {
    pub fn with_tolerance(tolerance: f64) -> Self {
        Self { tolerance, ..Default::default() }
    }

    pub fn validate(&self) -> Result<(), CamError> {
        let bad = |m: &str| Err(CamError::InvalidInput(format!("arc fit: {m}")));
        if !self.tolerance.is_finite() || self.tolerance <= 0.0 {
            return bad("tolerance must be finite and positive");
        }
        if !self.min_radius.is_finite() || self.min_radius < 0.0 || !self.max_radius.is_finite() || self.max_radius <= self.min_radius {
            return bad("radius limits must satisfy 0 <= min < max");
        }
        if !self.max_sweep_deg.is_finite() || self.max_sweep_deg <= 0.0 || self.max_sweep_deg > 360.0 {
            return bad("maximum sweep must be in (0, 360] degrees");
        }
        if self.planes.iter().all(Option::is_none) {
            return bad("at least one plane must be allowed");
        }
        Ok(())
    }
}

/// One fitted arc through a run of points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FittedArc {
    pub from: Point3,
    pub to: Point3,
    /// Center; its normal coordinate equals `from`'s.
    pub center: Point3,
    pub plane: ArcPlane,
    pub clockwise: bool,
    pub radius: f64,
    /// Sweep in radians, positive.
    pub sweep: f64,
    /// Normal advance per full turn (0 for planar arcs).
    pub pitch: f64,
}

/// A piece of a fitted polyline.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Fitted {
    /// A straight chord to `to`.
    Line { to: Point3 },
    Arc(FittedArc),
}

/// Plane coordinates `(u, v, w)` of a point.
pub fn plane_coords(p: Point3, plane: ArcPlane) -> (f64, f64, f64) {
    match plane {
        ArcPlane::Xy => (p.x, p.y, p.z),
        ArcPlane::Xz => (p.z, p.x, p.y),
        ArcPlane::Yz => (p.y, p.z, p.x),
    }
}

/// Inverse of [`plane_coords`].
pub fn from_plane_coords(u: f64, v: f64, w: f64, plane: ArcPlane) -> Point3 {
    match plane {
        ArcPlane::Xy => Point3::new(u, v, w),
        ArcPlane::Xz => Point3::new(v, w, u),
        ArcPlane::Yz => Point3::new(w, u, v),
    }
}

/// Circle through three in-plane points: `(cu, cv, r)`, `None` when collinear.
/// The ends of a run are two of the three, so the start and end radii of an
/// emitted arc are exactly equal, as controllers require of a G2/G3 block.
fn circle_through(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Option<(f64, f64, f64)> {
    let (ax, ay) = a;
    let (bx, by) = b;
    let (cx, cy) = c;
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() < 1e-300 {
        return None;
    }
    let a2 = ax * ax + ay * ay;
    let b2 = bx * bx + by * by;
    let c2 = cx * cx + cy * cy;
    let ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    let uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    let r = (ux - ax).hypot(uy - ay);
    if !ux.is_finite() || !uy.is_finite() || !r.is_finite() {
        return None;
    }
    Some((ux, uy, r))
}

/// Try to fit one arc through `points[0..=last]` in `plane`.
fn fit_run(points: &[Point3], plane: ArcPlane, opts: &ArcFitOptions) -> Option<FittedArc> {
    let n = points.len();
    // Any three points lie on some circle; an arc needs a fourth to verify.
    if n < 4 {
        return None;
    }
    let uv: Vec<(f64, f64, f64)> = points.iter().map(|&p| plane_coords(p, plane)).collect();
    let (u0, v0, w0) = uv[0];
    let (u1, v1, _) = uv[n - 1];
    // The intermediate point farthest from the chord fixes the circle.
    let du = u1 - u0;
    let dv = v1 - v0;
    let chord = du.hypot(dv);
    let mut far = 1usize;
    let mut far_d = -1.0;
    for (k, &(u, v, _)) in uv.iter().enumerate().take(n - 1).skip(1) {
        let d = if chord > 0.0 { ((u - u0) * dv - (v - v0) * du).abs() / chord } else { (u - u0).hypot(v - v0) };
        if d > far_d {
            far_d = d;
            far = k;
        }
    }
    // A run that closes on itself (a full circle) has no usable chord: fit
    // through the points a third and two thirds of the way round instead.
    let (cu, cv, r) = if chord <= opts.tolerance {
        circle_through((u0, v0), (uv[n / 3].0, uv[n / 3].1), (uv[2 * n / 3].0, uv[2 * n / 3].1))?
    } else {
        circle_through((u0, v0), (uv[far].0, uv[far].1), (u1, v1))?
    };
    if r < opts.min_radius || r > opts.max_radius {
        return None;
    }
    // Radial deviation of every point.
    if uv.iter().any(|&(u, v, _)| ((u - cu).hypot(v - cv) - r).abs() > opts.tolerance) {
        return None;
    }
    // Angles must advance monotonically in one direction; the sweep is their sum.
    let angle = |u: f64, v: f64| (v - cv).atan2(u - cu);
    let mut sweep = 0.0;
    let mut sign = 0.0;
    let mut prev = angle(u0, v0);
    for &(u, v, _) in uv.iter().skip(1) {
        let a = angle(u, v);
        let mut step = a - prev;
        if step > PI {
            step -= TAU;
        } else if step <= -PI {
            step += TAU;
        }
        if step == 0.0 {
            return None;
        }
        let s = step.signum();
        if sign == 0.0 {
            sign = s;
        } else if s != sign {
            return None;
        }
        // Chord sagitta between consecutive samples.
        if r * (1.0 - (step.abs() * 0.5).cos()) > opts.tolerance {
            return None;
        }
        sweep += step.abs();
        prev = a;
    }
    if sweep > opts.max_sweep_deg.to_radians() + 1e-12 || sweep > TAU + 1e-12 {
        return None;
    }
    // Normal coordinate: constant (planar) or linear in the sweep (helical).
    let w_end = uv[n - 1].2;
    let pitch = if (w_end - w0).abs() <= opts.tolerance { 0.0 } else { (w_end - w0) / sweep * TAU };
    if pitch != 0.0 && !opts.helical {
        return None;
    }
    let mut turned = 0.0;
    prev = angle(u0, v0);
    for &(u, v, w) in uv.iter().skip(1) {
        let a = angle(u, v);
        let mut step = a - prev;
        if step > PI {
            step -= TAU;
        } else if step <= -PI {
            step += TAU;
        }
        turned += step.abs();
        prev = a;
        let expected = w0 + pitch * turned / TAU;
        if (w - expected).abs() > opts.tolerance {
            return None;
        }
    }
    Some(FittedArc {
        from: points[0],
        to: points[n - 1],
        center: from_plane_coords(cu, cv, w0, plane),
        plane,
        clockwise: sign < 0.0,
        radius: r,
        sweep,
        pitch,
    })
}

/// `p` lies on the arc's circle (in-plane radius within `tolerance`).
fn on_circle(p: Point3, arc: &FittedArc, tolerance: f64) -> bool {
    let (u, v, _) = plane_coords(p, arc.plane);
    let (cu, cv, _) = plane_coords(arc.center, arc.plane);
    ((u - cu).hypot(v - cv) - arc.radius).abs() <= tolerance
}

/// Every point of the run lies within `tolerance` of the chord between its ends.
fn line_fits(points: &[Point3], tolerance: f64) -> bool {
    let a = points[0];
    let b = points[points.len() - 1];
    points.iter().all(|&p| crate::filter::distance_to_chord(p, a, b) <= tolerance)
}

/// Fit arcs to a polyline. Returns the pieces in order; their ends chain from
/// `points[0]`. Every input point is an end of some piece or lies within
/// `tolerance` of the arc that replaced it; straight stretches keep every
/// point (filtering is the line filter's job, not the fitter's).
pub fn fit_polyline(points: &[Point3], opts: &ArcFitOptions) -> Result<Vec<Fitted>, CamError> {
    opts.validate()?;
    if let Some(p) = points.iter().find(|p| !p.is_finite()) {
        return Err(CamError::NonFinite { what: format!("arc fit point {p:?}") });
    }
    let n = points.len();
    let mut out = Vec::new();
    let mut i = 0usize;
    // Longest arc run starting at `start`, over every allowed plane.
    let longest = |start: usize| -> Option<(usize, FittedArc)> {
        let mut best: Option<(usize, FittedArc)> = None;
        for plane in opts.planes.iter().flatten() {
            // Extend greedily: the first failing end stops this plane.
            let mut end = None;
            let mut j = start + 3;
            while j < n {
                match fit_run(&points[start..=j], *plane, opts) {
                    Some(arc) => {
                        end = Some((j, arc));
                        j += 1;
                    }
                    None => break,
                }
            }
            if let Some((j, arc)) = end {
                if best.as_ref().map_or(true, |(bj, _)| j > *bj) {
                    best = Some((j, arc));
                }
            }
        }
        best
    };
    while i + 1 < n {
        let mut best = longest(i);
        // One-step lookahead: when the arc from the next point reaches further
        // and the current point is not on that arc's circle, the current point
        // is a corner that would bend the arc; keep it a line instead.
        if let Some((j, _)) = best {
            if let Some((k, next)) = longest(i + 1) {
                if k > j && !on_circle(points[i], &next, opts.tolerance) {
                    best = None;
                }
            }
        }
        match best {
            // A window that its own chord also fits is a (slightly kinked)
            // line, not an arc worth a G2/G3: keep its points as lines, one
            // step at a time, so the fitter never removes a point.
            Some((j, _)) if line_fits(&points[i..=j], opts.tolerance) => {
                out.push(Fitted::Line { to: points[i + 1] });
                i += 1;
            }
            Some((j, arc)) => {
                out.push(Fitted::Arc(arc));
                i = j;
            }
            None => {
                out.push(Fitted::Line { to: points[i + 1] });
                i += 1;
            }
        }
    }
    Ok(out)
}

fn is_cut_line(m: &Motion) -> bool {
    matches!(m.kind, MotionKind::Cut { .. })
}

fn same_run(a: &Motion, b: &Motion) -> bool {
    is_cut_line(a) && is_cut_line(b) && a.feed == b.feed && a.path_id == b.path_id && a.tool_axis == b.tool_axis
}

/// Fit arcs to every run of straight cut motions. Returns the new stream and
/// the number of arcs emitted. Other motions pass through unchanged. A run
/// whose tool axis differs from the previous motion's keeps its first chord
/// (the axis change) as a line, so no arc changes the tool axis.
pub fn fit_motions(motions: &[Motion], opts: &ArcFitOptions) -> Result<(Vec<Motion>, usize), CamError> {
    opts.validate()?;
    let axis_of = |m: &Motion| m.tool_axis.unwrap_or(Point3::Z);
    let mut out = Vec::with_capacity(motions.len());
    let mut arcs = 0usize;
    let mut position: Option<Point3> = None;
    let mut axis = Point3::Z;
    let mut i = 0usize;
    while i < motions.len() {
        let m = &motions[i];
        if !is_cut_line(m) {
            out.push(m.clone());
            if let Some(p) = m.end() {
                position = Some(p);
                axis = axis_of(m);
            }
            i += 1;
            continue;
        }
        if position.is_some() && axis_of(m) != axis {
            out.push(m.clone());
            position = m.end();
            axis = axis_of(m);
            i += 1;
            continue;
        }
        let mut j = i + 1;
        while j < motions.len() && same_run(&motions[j - 1], &motions[j]) {
            j += 1;
        }
        let run = &motions[i..j];
        let ends: Vec<Point3> = run.iter().map(|m| m.end().expect("cuts end somewhere")).collect();
        let template = &run[0];
        match position {
            Some(start) if run.len() >= 2 => {
                let mut points = Vec::with_capacity(ends.len() + 1);
                points.push(start);
                points.extend(ends.iter().copied());
                for piece in fit_polyline(&points, opts)? {
                    match piece {
                        Fitted::Line { to } => out.push(Motion { kind: MotionKind::Cut { to }, ..template.clone() }),
                        Fitted::Arc(arc) => {
                            arcs += 1;
                            let kind = if arc.clockwise {
                                MotionKind::ArcCw { to: arc.to, center: arc.center, plane: arc.plane, pitch: arc.pitch }
                            } else {
                                MotionKind::ArcCcw { to: arc.to, center: arc.center, plane: arc.plane, pitch: arc.pitch }
                            };
                            out.push(Motion { kind, ..template.clone() });
                        }
                    }
                }
            }
            _ => out.extend(run.iter().cloned()),
        }
        position = ends.last().copied();
        axis = axis_of(template);
        i = j;
    }
    Ok((out, arcs))
}

/// Fit arcs to a toolpath and record the deviation bound on its report.
pub fn fit_toolpath(path: &Toolpath, opts: &ArcFitOptions) -> Result<Toolpath, CamError> {
    let (motions, arcs) = fit_motions(&path.motions, opts)?;
    let mut out = path.clone();
    out.motions = motions;
    out.report.counts.motions = out.motions.len() as u64;
    if arcs > 0 {
        out.report.approximations.push(ApproximationLimit {
            kind: "arc-fit".into(),
            bound: Some(opts.tolerance),
            detail: Some(format!("{arcs} arcs fitted within {}", opts.tolerance)),
        });
    }
    Ok(out)
}

/// Evaluate an arc motion at fraction `t` of its sweep, given its start.
/// Used by linearization (posts without arc support, simulation).
pub fn arc_point(from: Point3, kind: &MotionKind, t: f64) -> Option<Point3> {
    let (to, center, plane, pitch, cw) = match kind {
        MotionKind::ArcCw { to, center, plane, pitch } => (*to, *center, *plane, *pitch, true),
        MotionKind::ArcCcw { to, center, plane, pitch } => (*to, *center, *plane, *pitch, false),
        _ => return None,
    };
    let (sweep, radius) = arc_sweep(from, to, center, plane, cw)?;
    let (cu, cv, _) = plane_coords(center, plane);
    let (u0, v0, w0) = plane_coords(from, plane);
    let a0 = (v0 - cv).atan2(u0 - cu);
    let a = a0 + if cw { -sweep * t } else { sweep * t };
    let w = w0 + pitch * sweep * t / TAU;
    Some(from_plane_coords(cu + radius * a.cos(), cv + radius * a.sin(), w, plane))
}

/// Positive sweep (radians) and radius of an arc from `from` to `to` about
/// `center`. A coincident start and end is a full circle. `None` when the
/// radius is zero.
pub fn arc_sweep(from: Point3, to: Point3, center: Point3, plane: ArcPlane, clockwise: bool) -> Option<(f64, f64)> {
    let (cu, cv, _) = plane_coords(center, plane);
    let (u0, v0, _) = plane_coords(from, plane);
    let (u1, v1, _) = plane_coords(to, plane);
    let r0 = (u0 - cu).hypot(v0 - cv);
    let r1 = (u1 - cu).hypot(v1 - cv);
    if r0 <= 0.0 || r1 <= 0.0 {
        return None;
    }
    let a0 = (v0 - cv).atan2(u0 - cu);
    let a1 = (v1 - cv).atan2(u1 - cu);
    let mut sweep = if clockwise { a0 - a1 } else { a1 - a0 };
    sweep = sweep.rem_euclid(TAU);
    if sweep < 1e-12 {
        sweep = TAU;
    }
    Some((sweep, 0.5 * (r0 + r1)))
}

