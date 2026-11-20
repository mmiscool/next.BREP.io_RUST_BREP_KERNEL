//! A convex prism slid along a straight line in its own plane: its whole sweep, exactly
//! (`BREP_SWEEP_SLIDE_PRISM`, on; =0 restores the engine).
//!
//! A convex region slid by t covers its Minkowski sum with the segment [0, t], which is convex: its
//! boundary is the region's boundary where the outward normal points back (against t) where the
//! slide starts, the boundary where it points ahead where the slide ends, and the two segments along
//! t joining them at the points where the normal is square to t. For an outline of lines and
//! circular arcs those pieces are lines and arcs again (an arc split at its two normals square to t),
//! so the sweep is that outline extruded through the prism's height: planes and cylinders, nothing
//! fitted. Straight-sided outlines are left to the planar-region route; a non-convex one, or any
//! motion but one slide in the prism's plane, to the general engine.
use crate::solid_sweep::{OutlinePiece, SolidMotion, SweepClip, SweptTool};
use crate::{extrude_profile_brep, make_arc, make_line, BrepSolid, KernelRefusal, NurbsCurve, Vec3};
use std::f64::consts::{FRAC_PI_2, TAU};

/// `BREP_SWEEP_SLIDE_PRISM` (on unless =0): the slid-prism route of `sweep_solid_within`.
pub(crate) fn slide_prism_on() -> bool {
    crate::switch!("BREP_SWEEP_SLIDE_PRISM", true)
}

type P = [f64; 2];

/// A boundary piece: a line, or an arc (centre, radius, angle at its start, angle at its end), run
/// from the first angle to the second (counter-clockwise where the second is the larger).
#[derive(Clone, Copy, Debug)]
enum Piece {
    Line(P, P),
    Arc(P, f64, f64, f64),
}

impl Piece {
    fn start(&self) -> P {
        match *self {
            Piece::Line(a, _) => a,
            Piece::Arc(c, r, a0, _) => [c[0] + r * a0.cos(), c[1] + r * a0.sin()],
        }
    }
    fn end(&self) -> P {
        match *self {
            Piece::Line(_, b) => b,
            Piece::Arc(c, r, _, a1) => [c[0] + r * a1.cos(), c[1] + r * a1.sin()],
        }
    }
    /// The unit direction of travel at the start and at the end.
    fn tangents(&self) -> (P, P) {
        match *self {
            Piece::Line(a, b) => {
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let l = dx.hypot(dy);
                ([dx / l, dy / l], [dx / l, dy / l])
            }
            Piece::Arc(_, _, a0, a1) => {
                let k = if a1 >= a0 { 1.0 } else { -1.0 };
                ([-k * a0.sin(), k * a0.cos()], [-k * a1.sin(), k * a1.cos()])
            }
        }
    }
    fn reversed(&self) -> Piece {
        match *self {
            Piece::Line(a, b) => Piece::Line(b, a),
            Piece::Arc(c, r, a0, a1) => Piece::Arc(c, r, a1, a0),
        }
    }
    fn shifted(&self, t: P) -> Piece {
        let m = |q: P| [q[0] + t[0], q[1] + t[1]];
        match *self {
            Piece::Line(a, b) => Piece::Line(m(a), m(b)),
            Piece::Arc(c, r, a0, a1) => Piece::Arc(m(c), r, a0, a1),
        }
    }
    /// Twice the signed area this piece adds to its loop's (exact for lines and arcs).
    fn twice_area(&self) -> f64 {
        match *self {
            Piece::Line(a, b) => cross(a, b),
            Piece::Arc(c, r, a0, a1) => r * r * (a1 - a0) + r * (c[0] * (a1.sin() - a0.sin()) - c[1] * (a1.cos() - a0.cos())),
        }
    }
}

fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// The whole sweep of a convex prism with arcs in its outline slid in its own plane, or None where
/// the tool or motion is not of that kind. Like the closed planar motion's region, it is the whole
/// sweep, so it holds whatever lies in the clip.
pub(crate) fn slide_prism(tool: &SweptTool, motion: &SolidMotion, clip: &SweepClip) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    let SweptTool::Prism { origin, normal, x_axis, outline, from, to } = tool else { return Ok(None) };
    let [step] = motion.steps.as_slice() else { return Ok(None) };
    // An invalid clip or range is the engine's to refuse, in its words.
    let good = |r: [f64; 2]| r[0].is_finite() && r[1].is_finite() && r[1] > r[0];
    if !(good(clip.radius) && clip.radius[0] >= 0.0 && good(clip.axial) && clip.angle.map_or(true, good)) {
        return Ok(None);
    }
    let Ok(clip_axis) = clip.axis_direction.normalized() else { return Ok(None) };
    if clip.reference.sub(clip_axis.scale(clip.reference.dot(clip_axis))).normalized().is_err() {
        return Ok(None);
    }
    if !(good([*from, *to]) && good([motion.from, motion.to])) || step.turn != 0.0 || !step.advance.is_finite() || step.advance == 0.0 {
        return Ok(None);
    }
    let (Ok(n), Ok(d)) = (normal.normalized(), step.axis_direction.normalized()) else { return Ok(None) };
    // Arcs present (a straight-sided outline is the planar-region route's), no ellipses.
    if !outline.iter().any(|p| matches!(p, OutlinePiece::Arc { .. })) || outline.iter().any(|p| matches!(p, OutlinePiece::Ellipse { .. })) {
        return Ok(None);
    }
    let Ok(xa) = x_axis.sub(n.scale(x_axis.dot(n))).normalized() else { return Ok(None) };
    let ya = n.cross(xa);
    let (s0, s1) = (d.scale(step.advance * motion.from), d.scale(step.advance * motion.to));
    let mut pieces: Vec<Piece> = Vec::with_capacity(outline.len());
    let mut size = 1.0f64;
    // The outline's extent (its widest arc, its longest side): what an out-of-plane drift is
    // measured against, whatever its distance from the origin.
    let mut span = 0.0f64;
    for p in outline {
        match *p {
            OutlinePiece::Line { from, to } => {
                size = size.max(from[0].abs()).max(from[1].abs()).max(to[0].abs()).max(to[1].abs());
                span = span.max((to[0] - from[0]).hypot(to[1] - from[1]));
                pieces.push(Piece::Line(from, to));
            }
            OutlinePiece::Arc { centre, radius, start, end } => {
                if !(radius > 0.0) || !(start.is_finite() && end.is_finite()) || start == end {
                    return Ok(None);
                }
                size = size.max(centre[0].abs() + radius).max(centre[1].abs() + radius);
                span = span.max(2.0 * radius);
                pieces.push(Piece::Arc(centre, radius, start, end));
            }
            OutlinePiece::Ellipse { .. } => return Ok(None),
        }
    }
    // Both poses within the prism's plane, measured against the outline's extent (neither a far
    // start nor a far outline widens the tolerance: the sweep is built in the plane).
    let travel = s1.sub(s0);
    let drift = 1e-12 * span.max(1e-300);
    if !(size.is_finite() && span > 0.0) || s0.dot(n).abs() > drift || s1.dot(n).abs() > drift || travel.length() <= 1e-9 * size {
        return Ok(None); // not a slide within the prism's plane
    }
    // (The closure and area tolerances stay with the outline's own coordinates, as the engine's.)
    let flat = |v: Vec3| -> P { [v.dot(xa), v.dot(ya)] };
    let (start, t) = (flat(s0), flat(travel));
    // Orient the loop counter-clockwise; then a convex loop runs every arc counter-clockwise and
    // turns left at every corner, once round in all.
    let area: f64 = pieces.iter().map(Piece::twice_area).sum();
    if !(area.abs() > 1e-12 * size * size) {
        return Ok(None);
    }
    if area < 0.0 {
        pieces = pieces.iter().rev().map(Piece::reversed).collect();
    }
    let mut turning = 0.0;
    for k in 0..pieces.len() {
        let (a, b) = (pieces[k], pieces[(k + 1) % pieces.len()]);
        let gap = [a.end()[0] - b.start()[0], a.end()[1] - b.start()[1]];
        if gap[0].hypot(gap[1]) > 1e-9 * size {
            return Ok(None); // not one closed chain in order
        }
        if let Piece::Arc(_, _, a0, a1) = a {
            if !(a1 > a0) || a1 - a0 > TAU + 1e-12 {
                return Ok(None); // an arc curving outward: a hollow
            }
            turning += a1 - a0;
        }
        let (_, out) = a.tangents();
        let (into, _) = b.tangents();
        let turn = cross(out, into).atan2(out[0] * into[0] + out[1] * into[1]);
        if turn < -1e-12 {
            return Ok(None); // a reflex corner
        }
        turning += turn;
    }
    if (turning - TAU).abs() > 1e-9 {
        return Ok(None); // not one convex loop
    }
    // Split the arcs where their outward normal is square to the slide; each piece faces ahead (+1)
    // or back (-1). Lines along the slide lie on the sweep's sides and are dropped.
    let tl = t[0].hypot(t[1]);
    let phi = t[1].atan2(t[0]);
    let mut tagged: Vec<(Piece, i32)> = Vec::new();
    for p in &pieces {
        match *p {
            Piece::Line(a, b) => {
                let dir = [b[0] - a[0], b[1] - a[1]];
                let side = (dir[1] * t[0] - dir[0] * t[1]) / (dir[0].hypot(dir[1]) * tl);
                if side.abs() > 1e-12 {
                    tagged.push((*p, if side > 0.0 { 1 } else { -1 }));
                }
            }
            Piece::Arc(c, r, a0, a1) => {
                let mut cuts = vec![a0, a1];
                for q in [phi + FRAC_PI_2, phi - FRAC_PI_2] {
                    let mut a = q + TAU * ((a0 - q) / TAU).ceil();
                    while a < a1 {
                        if a > a0 + 1e-12 && a < a1 - 1e-12 {
                            cuts.push(a);
                        }
                        a += TAU;
                    }
                }
                cuts.sort_by(f64::total_cmp);
                for w in cuts.windows(2) {
                    let side = (0.5 * (w[0] + w[1]) - phi).cos();
                    tagged.push((Piece::Arc(c, r, w[0], w[1]), if side > 0.0 { 1 } else { -1 }));
                }
            }
        }
    }
    // A convex loop faces ahead along one run and back along the other.
    let count = tagged.len();
    let changes = (0..count).filter(|&k| tagged[k].1 != tagged[(k + 1) % count].1).count();
    if changes != 2 {
        return Ok(None);
    }
    let Some(first) = (0..count).find(|&k| tagged[k].1 == 1 && tagged[(k + count - 1) % count].1 == -1) else { return Ok(None) };
    let order: Vec<(Piece, i32)> = (0..count).map(|k| tagged[(first + k) % count]).collect();
    let ahead_at = [start[0] + t[0], start[1] + t[1]];
    let fronts: Vec<Piece> = order.iter().filter(|(_, s)| *s == 1).map(|(p, _)| p.shifted(ahead_at)).collect();
    let backs: Vec<Piece> = order.iter().filter(|(_, s)| *s == -1).map(|(p, _)| p.shifted(start)).collect();
    let (Some(front_last), Some(back_last)) = (fronts.last(), backs.last()) else { return Ok(None) };
    let mut ring: Vec<Piece> = fronts.clone();
    ring.push(Piece::Line(front_last.end(), backs[0].start()));
    ring.extend(backs.iter().copied());
    ring.push(Piece::Line(back_last.end(), fronts[0].start()));
    // The loop at the prism's foot, extruded through its height: planes and cylinders.
    let foot = origin.add(n.scale(*from));
    let at = |q: P| foot.add(xa.scale(q[0])).add(ya.scale(q[1]));
    let mut curves: Vec<NurbsCurve> = Vec::with_capacity(ring.len());
    for p in &ring {
        let curve = match *p {
            Piece::Line(a, b) => {
                if (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-12 * size {
                    continue; // a side of zero length (a line along the slide already ends there)
                }
                make_line(at(a), at(b))
            }
            Piece::Arc(c, r, a0, a1) => make_arc(at(c), xa, ya, r, a0, a1),
        };
        match curve {
            Ok(c) => curves.push(c),
            Err(_) => return Ok(None),
        }
    }
    // (A part the kernel will not build leaves the sweep to the engine, never a refusal.)
    Ok(extrude_profile_brep(&curves, n, to - from).ok().map(|body| vec![body]))
}

