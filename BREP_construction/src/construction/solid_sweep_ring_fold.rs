//! A ring (a torus) slid along a straight line far enough over to one side
//! that the curve where its normal is square to the motion, seen along the
//! motion, folds: its inner outline makes two swallowtails, each with two
//! cusps and a crossing.
//!
//! The sweep is everything between the end ring's leading band and the start
//! ring's trailing band over the ring's shadow along the motion. The shadow's
//! edge is the outer outline and two arcs of the inner one (from one crossing
//! to the other, outside the swallowtails), and over it stands the outline
//! carried by the whole push. Inside each swallowtail the end ring's leading
//! side steps: from the crossing to one cusp (the side of the crossing nearer
//! the motion's end) the leading band folds away and the line along the
//! motion next enters the ring higher up, so a wall stands on that stretch of
//! outline up to where the line meets the band again (the outline's shadow on
//! the band), as tall as the two outline points at the crossing are apart and
//! nothing at the cusp. The start ring's trailing side steps the same way over
//! the crossing's other stretch, toward the start. Fourteen faces: each band
//! in two halves, seamed across the band in the middle of each arc; the outer
//! wall in two halves; the four arc walls (each arc split at its seam); the
//! four steps.

use super::*;

/// `BREP_SWEEP_RING_FOLD` (on unless =0): a ring slid along a line past the
/// lean at which its inner outline folds is built by the construction above,
/// instead of refused.
pub(super) fn ring_fold_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_FOLD", true)
}

/// Samples of a stretch of outline (and of its shadow) between its ends (the
/// steps' volume reads the same to 1e-15 from 64 up).
const STRETCH: usize = 128;

/// The ring's sweep along a straight push past its fold, or None where the
/// motion is not a straight push or the inner outline does not fold into two
/// swallowtails (the general split serves, or refuses).
#[allow(clippy::too_many_arguments)]
pub(super) fn ring_slid_past_fold(screw: &Screw, origin: Vec3, w: Vec3, u1: Vec3, u2: Vec3, centre: [f64; 2], radius: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let straight = screw.omega == 0.0 && screw.c2 == 0.0 && screw.frame.is_none() && screw.turns.is_none();
    if !straight {
        return Ok(None);
    }
    let push = screw.apply(screw.t1, origin).sub(screw.apply(screw.t0, origin));
    let Ok(m) = push.normalized() else { return Ok(None) };
    let length = push.length();
    let (rho_c, z_c) = (centre[0], centre[1]);
    if !(rho_c > radius && radius > 0.0) {
        return Ok(None);
    }
    // The ring's own size (where its centre circle sits along the axis says
    // nothing about it).
    let scale = rho_c + radius;
    let radial = |psi: f64| u1.scale(psi.cos()).add(u2.scale(psi.sin()));
    let tube_centre = |psi: f64| origin.add(w.scale(z_c)).add(radial(psi).scale(rho_c));
    let point = |psi: f64, v: f64| tube_centre(psi).add(radial(psi).scale(radius * v.cos())).add(w.scale(radius * v.sin()));
    let normal = |psi: f64, v: f64| radial(psi).scale(v.cos()).add(w.scale(v.sin()));
    // n·m = A cos v + B sin v = |(A, B)| cos(v − α): the leading band is
    // α − π/2 < v < α + π/2; branch 0 its lower edge, branch 1 its upper.
    let b_m = w.dot(m);
    let alpha = |psi: f64| b_m.atan2(radial(psi).dot(m));
    if (0..64).any(|k| radial(TAU * k as f64 / 64.0).dot(m).hypot(b_m) < 1e-9) {
        return Ok(None);
    }
    let branch_v = |psi: f64, branch: usize| alpha(psi) + if branch == 0 { -0.5 * PI } else { 0.5 * PI };
    let outline = |psi: f64, branch: usize| point(psi, branch_v(psi, branch));
    // Seen along the motion.
    let p1 = {
        let seed = if m.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
        seed.sub(m.scale(seed.dot(m))).normalized_or(seed)
    };
    let p2 = m.cross(p1);
    let seen = |x: Vec3| [x.dot(p1), x.dot(p2)];
    let seen_tangent = |psi: f64, branch: usize| {
        let h = 1e-6;
        let d = outline(psi + h, branch).sub(outline(psi - h, branch)).scale(0.5 / h);
        [d.dot(p1), d.dot(p2)]
    };
    // Cusps (the seen tangent turns back) and crossings of each branch.
    const N: usize = 2048;
    let psis: Vec<f64> = (0..N).map(|k| TAU * k as f64 / N as f64).collect();
    let mut folds: [Vec<f64>; 2] = [Vec::new(), Vec::new()];
    let mut crossings: [Vec<(f64, f64)>; 2] = [Vec::new(), Vec::new()];
    for branch in 0..2 {
        let tangents: Vec<[f64; 2]> = psis.iter().map(|&psi| seen_tangent(psi, branch)).collect();
        for k in 0..N {
            let (a, b) = (tangents[k], tangents[(k + 1) % N]);
            if a[0] * b[0] + a[1] * b[1] < 0.0 {
                // The tangent's component along its direction past the cusp
                // changes sign at it.
                let (lo, hi) = (psis[k], psis[k] + TAU / N as f64);
                let e = {
                    let t = seen_tangent(hi, branch);
                    let l = t[0].hypot(t[1]);
                    [t[0] / l, t[1] / l]
                };
                let f = |psi: f64| {
                    let t = seen_tangent(psi, branch);
                    t[0] * e[0] + t[1] * e[1]
                };
                let (mut a, mut b) = (lo, hi);
                if !(f(a) < 0.0 && f(b) > 0.0) {
                    return Err(refuse("ring_fold", "a cusp of the ring's outline is not bracketed".into()));
                }
                for _ in 0..100 {
                    let mid = 0.5 * (a + b);
                    if f(mid) < 0.0 { a = mid } else { b = mid }
                }
                folds[branch].push(0.5 * (a + b));
            }
        }
        let polyline: Vec<[f64; 2]> = psis.iter().map(|&psi| seen(outline(psi, branch))).collect();
        for i in 0..N {
            for j in i + 2..N {
                if i == 0 && j == N - 1 {
                    continue;
                }
                let (a, b, c, d) = (polyline[i], polyline[(i + 1) % N], polyline[j], polyline[(j + 1) % N]);
                let (r, s) = ([b[0] - a[0], b[1] - a[1]], [d[0] - c[0], d[1] - c[1]]);
                let den = r[0] * s[1] - r[1] * s[0];
                if den.abs() < 1e-300 {
                    continue;
                }
                let q = [c[0] - a[0], c[1] - a[1]];
                let (t, u) = ((q[0] * s[1] - q[1] * s[0]) / den, (q[0] * r[1] - q[1] * r[0]) / den);
                if (0.0..1.0).contains(&t) && (0.0..1.0).contains(&u) {
                    crossings[branch].push((psis[i] + t * TAU / N as f64, psis[j] + u * TAU / N as f64));
                }
            }
        }
    }
    let inner = match (folds[0].len(), folds[1].len()) {
        (0, 4) => 1,
        (4, 0) => 0,
        _ => return Ok(None),
    };
    let outer = 1 - inner;
    if crossings[outer].len() != 0 || crossings[inner].len() != 2 {
        return Ok(None);
    }
    // Each crossing exactly (Newton on the seen points), as (ψa, ψb) with the
    // swallowtail's two cusps between them going up from ψa.
    let mut tails: Vec<(f64, f64, [f64; 2])> = Vec::new();
    for &(guess_a, guess_b) in &crossings[inner] {
        let (mut a, mut b) = (guess_a, guess_b);
        for _ in 0..60 {
            let (x, y) = (seen(outline(a, inner)), seen(outline(b, inner)));
            let f = [x[0] - y[0], x[1] - y[1]];
            let (ta, tb) = (seen_tangent(a, inner), seen_tangent(b, inner));
            // J = [ta, −tb]
            let det = ta[0] * (-tb[1]) - (-tb[0]) * ta[1];
            if det.abs() < 1e-300 {
                return Err(refuse("ring_fold", "a crossing of the ring's outline is tangent".into()));
            }
            let da = (f[0] * (-tb[1]) - (-tb[0]) * f[1]) / det;
            let db = (ta[0] * f[1] - ta[1] * f[0]) / det;
            a -= da;
            b -= db;
            if da.abs().max(db.abs()) < 1e-15 {
                break;
            }
        }
        let a = a.rem_euclid(TAU);
        let b = a + (b - a).rem_euclid(TAU);
        let between: Vec<f64> = folds[inner].iter().map(|&c| a + (c - a).rem_euclid(TAU)).filter(|&c| c < b).collect();
        let (a, b, between) = if between.len() == 2 {
            (a, b, between)
        } else {
            // The other way round.
            let (a2, b2) = (b.rem_euclid(TAU), b.rem_euclid(TAU) + (a - b).rem_euclid(TAU));
            let other: Vec<f64> = folds[inner].iter().map(|&c| a2 + (c - a2).rem_euclid(TAU)).filter(|&c| c < b2).collect();
            if other.len() != 2 {
                return Ok(None);
            }
            (a2, b2, other)
        };
        let mut cusps = [between[0], between[1]];
        cusps.sort_by(|x, y| x.total_cmp(y));
        tails.push((a, b, cusps));
    }
    tails.sort_by(|x, y| x.0.total_cmp(&y.0));
    // The arcs: from one tail's ψb to the next's ψa; seams in their middles.
    let (t0, t1) = (tails[0], tails[1]);
    let arc_end_1 = t1.0;
    if !(t0.1 < arc_end_1 && t1.1 < t0.0 + TAU) {
        return Ok(None);
    }
    let seams = [0.5 * (t0.1 + t1.0), 0.5 * (t1.1 + t0.0 + TAU)];
    // Lead: the crossing's outline point further along the motion; the end
    // ring steps on the stretch from it to its cusp.
    let height = |x: Vec3| x.dot(m);
    // Where the line from an outline point along `sign`·m next enters the ring
    // (the tangency's double root divided out of the quartic).
    let shadow_t = |x: Vec3, sign: f64| -> Result<f64, KernelRefusal> {
        let e = m.scale(sign);
        let d0 = x.sub(origin.add(w.scale(z_c)));
        let (a1, a0) = (2.0 * d0.dot(e), d0.dot(d0));
        let (dw, ew) = (d0.dot(w), e.dot(w));
        let k = rho_c * rho_c - radius * radius;
        // |d|² + k = t² + a1 t + (a0 + k); |d|² − (d·w)² = (1 − ew²) t² + (a1 − 2 dw ew) t + (a0 − dw²)
        let g = [a0 + k, a1, 1.0];
        let h = [a0 - dw * dw, a1 - 2.0 * dw * ew, 1.0 - ew * ew];
        let f4 = g[2] * g[2];
        let f3 = 2.0 * g[2] * g[1];
        let f2 = g[1] * g[1] + 2.0 * g[2] * g[0] - 4.0 * rho_c * rho_c * h[2];
        // F = t² (f4 t² + f3 t + f2)
        // Near the crossing the next meeting is itself a tangency (a double
        // root): a discriminant within rounding of zero is zero.
        let disc = f3 * f3 - 4.0 * f4 * f2;
        let disc = if disc < 0.0 && disc > -1e-10 * (f3 * f3 + (4.0 * f4 * f2).abs()) { 0.0 } else { disc };
        if disc < 0.0 {
            return Err(refuse("ring_fold", format!("a line along the push from the folded outline does not meet the ring again (disc {disc:e} at {x:?} sign {sign} f {f4} {f3} {f2})")));
        }
        let root = disc.sqrt();
        let mut roots = [(-f3 - root) / (2.0 * f4), (-f3 + root) / (2.0 * f4)];
        roots.sort_by(|p, q| p.total_cmp(q));
        roots.into_iter().find(|&t| t > 1e-12 * scale).ok_or_else(|| refuse("ring_fold", format!("a line along the push from the folded outline does not meet the ring again (roots {roots:?} at {x:?} sign {sign})")))
    };
    let fit = |points: &[Vec3]| -> Result<NurbsCurve, KernelRefusal> {
        let params: Vec<f64> = (0..points.len()).map(|i| i as f64 / (points.len() - 1) as f64).collect();
        frag(crate::interpolate_curve(points, 5, &params), "ring_fold_curve")
    };
    let fit_uv = |points: &[[f64; 2]]| -> Result<NurbsCurve, KernelRefusal> {
        let params: Vec<f64> = (0..points.len()).map(|i| i as f64 / (points.len() - 1) as f64).collect();
        let xyz: Vec<Vec3> = points.iter().map(|p| Vec3::new(p[0], p[1], 0.0)).collect();
        frag(crate::interpolate_curve(&xyz, 5, &params), "ring_fold_pcurve")
    };
    let tube_arc = |psi: f64, v0: f64, v1: f64| -> Result<NurbsCurve, KernelRefusal> {
        let start = point(psi, v0);
        let axis = radial(psi).cross(w);
        let surface = frag(make_revolution(tube_centre(psi), axis, &frag(make_line(start, start.add(axis)), "line")?, v1 - v0), "tube_arc")?;
        frag(surface.iso_curve_v(0.0), "tube_arc")
    };
    let stretch_points = |a: f64, b: f64, branch: usize, count: usize| -> Vec<Vec3> { (0..=count).map(|i| outline(a + (b - a) * i as f64 / count as f64, branch)).collect() };
    // Poses: 0 the end (its leading band bounds the sweep), 1 the start (its
    // trailing band).
    let pose_time = [screw.t1, screw.t0];
    let at = |pose: usize, x: Vec3| screw.apply(pose_time[pose], x);
    let moved = |pose: usize, curve: &NurbsCurve| screw.move_curve(pose_time[pose], curve);
    let mut shell = Shell::new();
    let mut vertex_at: HashMap<(usize, i64, usize), u64> = HashMap::new();
    let mut vertex = |shell: &mut Shell, pose: usize, psi: f64, branch: usize| -> u64 {
        let turn = (TAU * 1e9).round() as i64;
        let k = ((psi.rem_euclid(TAU) * 1e9).round() as i64).rem_euclid(turn);
        *vertex_at.entry((pose, k, branch)).or_insert_with(|| shell.vertex(at(pose, outline(psi, branch))))
    };
    let (arc_n, stretch_n) = (STRETCH, STRETCH);
    let halves = [(seams[0], seams[1]), (seams[1], seams[0] + TAU)];
    let place = |x: f64, lo: f64| lo + (x - lo).rem_euclid(TAU);
    // Each half's swallowtail, in the half's ψ: (ψa, ψb, cusps, lead at ψa).
    let mut half_tail: Vec<(f64, f64, [f64; 2], bool)> = Vec::new();
    for &(lo, hi) in &halves {
        let Some(&(a0, b0, c0)) = tails.iter().find(|t| place(t.0, lo) < hi) else { return Ok(None) };
        let a = place(a0, lo);
        let (b, c) = (a + (b0 - a0), [a + (c0[0] - a0), a + (c0[1] - a0)]);
        if !(lo < a && b < hi) {
            return Ok(None);
        }
        half_tail.push((a, b, c, height(outline(a, inner)) > height(outline(b, inner))));
    }
    // The step at each crossing must stay short of the push (the start ring's
    // step then stands clear above the end ring's).
    let step_of = |h: usize| (height(outline(half_tail[h].0, inner)) - height(outline(half_tail[h].1, inner))).abs();
    if (0..2).any(|h| !(step_of(h) < length * (1.0 - 1e-6))) {
        return Err(refuse("ring_fold", "the ring's fold steps further than it is pushed".into()));
    }
    // Edges: (edge, its curve in the tool's frame, in the edge's direction).
    let mut edge_tool: HashMap<u64, NurbsCurve> = HashMap::new();
    let mut make_edge = |shell: &mut Shell, pose: usize, tool: NurbsCurve, from: u64, to: u64| -> Result<u64, KernelRefusal> {
        let id = shell.edge(moved(pose, &tool)?, from, to, false)?;
        edge_tool.insert(id, tool);
        Ok(id)
    };
    // Per pose and half: the band's inner path (edge, forward) in ψ order, the
    // step's stretch and shadow.
    struct Step {
        /// The stretch of outline (ψ order) and its edge.
        edge: u64,
        curve: NurbsCurve,
        /// Its shadow (the same order) and the heights to it over the push.
        shadow: u64,
        heights: Vec<f64>,
        /// At ψ order's start (true) or end, the crossing.
        crossing_first: bool,
    }
    let mut outer_edge: HashMap<(usize, usize), u64> = HashMap::new(); // (pose, half)
    let mut arc_edges: HashMap<(usize, usize, usize), u64> = HashMap::new(); // (pose, half, 0 tail | 1 head)
    let mut steps: HashMap<(usize, usize), Step> = HashMap::new();
    let mut paths: HashMap<(usize, usize), Vec<(u64, bool)>> = HashMap::new();
    for (h, &(lo, hi)) in halves.iter().enumerate() {
        let (a, b, c, lead_at_a) = half_tail[h];
        let outer_curve = fit(&stretch_points(lo, hi, outer, 2 * arc_n))?;
        let tail_curve = fit(&stretch_points(lo, a, inner, arc_n))?;
        let head_curve = fit(&stretch_points(b, hi, inner, arc_n))?;
        for pose in 0..2 {
            let id = {
                let (s, e) = (vertex(&mut shell, pose, lo, outer), vertex(&mut shell, pose, hi, outer));
                make_edge(&mut shell, pose, outer_curve.clone(), s, e)?
            };
            outer_edge.insert((pose, h), id);
            let (s, e) = (vertex(&mut shell, pose, lo, inner), vertex(&mut shell, pose, a, inner));
            let tail = make_edge(&mut shell, pose, tail_curve.clone(), s, e)?;
            let (s, e) = (vertex(&mut shell, pose, b, inner), vertex(&mut shell, pose, hi, inner));
            let head = make_edge(&mut shell, pose, head_curve.clone(), s, e)?;
            arc_edges.insert((pose, h, 0), tail);
            arc_edges.insert((pose, h, 1), head);
            // The end ring steps from the lead crossing, the start ring from
            // the other.
            let from_a = if pose == 0 { lead_at_a } else { !lead_at_a };
            let (p0, p1) = if from_a { (a, c[0]) } else { (c[1], b) };
            let sign = if pose == 0 { -1.0 } else { 1.0 };
            // At the crossing end the line's next meeting with the ring turns
            // into a tangency, so the shadow moves as the square root of ψ's
            // distance there (and closes onto the outline at the cusp): samples
            // spaced as (1 − cos πσ)/2 keep it smooth in σ at both ends.
            let xs: Vec<Vec3> = (0..=stretch_n).map(|k| outline(p0 + (p1 - p0) * 0.5 * (1.0 - (PI * k as f64 / stretch_n as f64).cos()), inner)).collect();
            let mut ys = Vec::with_capacity(xs.len());
            let mut heights = Vec::with_capacity(xs.len());
            let other = outline(if from_a { b } else { a }, inner);
            for (k, &x) in xs.iter().enumerate() {
                let cusp_end = if from_a { k == stretch_n } else { k == 0 };
                let crossing_end = if from_a { k == 0 } else { k == stretch_n };
                if cusp_end {
                    ys.push(x);
                    heights.push(0.0);
                    continue;
                }
                if crossing_end {
                    // The line along the push touches the ring at both of the
                    // crossing's outline points.
                    let t = other.sub(x).dot(m) * sign;
                    if !(t > 0.0) || other.sub(x.add(m.scale(sign * t))).length() > 1e-9 * scale {
                        return Err(refuse("ring_fold", "the fold's crossing does not lie along the push".into()));
                    }
                    ys.push(other);
                    heights.push(t / length);
                    continue;
                }
                let t = shadow_t(x, sign)?;
                ys.push(x.add(m.scale(sign * t)));
                heights.push(t / length);
            }
            let curve = fit(&xs)?;
            let shadow_curve = fit(&ys)?;
            let (cusp_psi, cross_psi, other_psi) = if from_a { (c[0], a, b) } else { (c[1], b, a) };
            let cusp_v = vertex(&mut shell, pose, cusp_psi, inner);
            let cross_v = vertex(&mut shell, pose, cross_psi, inner);
            let other_v = vertex(&mut shell, pose, other_psi, inner);
            let (seg, shadow) = if from_a {
                (make_edge(&mut shell, pose, curve.clone(), cross_v, cusp_v)?, make_edge(&mut shell, pose, shadow_curve, other_v, cusp_v)?)
            } else {
                (make_edge(&mut shell, pose, curve.clone(), cusp_v, cross_v)?, make_edge(&mut shell, pose, shadow_curve, cusp_v, other_v)?)
            };
            // ψ order along the band's inner edge: from a, the stretch then its
            // shadow back to b; else the shadow from a to the cusp, then the
            // stretch. A shadow edge runs in its stretch's order (crossing to
            // cusp from a, cusp to crossing to b), against the band's either way.
            let path = if from_a { vec![(tail, true), (seg, true), (shadow, false), (head, true)] } else { vec![(tail, true), (shadow, false), (seg, true), (head, true)] };
            paths.insert((pose, h), path);
            steps.insert((pose, h), Step { edge: seg, curve, shadow, heights, crossing_first: from_a });
        }
    }
    // Seam arcs across each band at each seam: band 0 (the end's leading) from
    // branch 0 to branch 1, band 1 (the start's trailing) from branch 1 to
    // branch 0 a turn on.
    let mut seam_arc: HashMap<(usize, usize), u64> = HashMap::new(); // (pose, seam)
    for (k, &psi) in seams.iter().enumerate() {
        let v0 = branch_v(psi, 0);
        for pose in 0..2 {
            let (from_v, to_v, from_b, to_b) = if pose == 0 { (v0, v0 + PI, 0, 1) } else { (v0 + PI, v0 + TAU, 1, 0) };
            let (s, e) = (vertex(&mut shell, pose, psi, from_b), vertex(&mut shell, pose, psi, to_b));
            let id = make_edge(&mut shell, pose, tube_arc(psi, from_v, to_v)?, s, e)?;
            seam_arc.insert((pose, k), id);
        }
    }
    let seam_index = |h: usize, at_hi: bool| if at_hi { 1 - h } else { h };
    // Band 0's v range over a half (from branch 0 to branch 1), band 1's
    // (from branch 1 to branch 0 a turn on), with a margin each side.
    let band_range = |lo: f64, hi: f64, band: usize| -> (f64, f64) {
        let mut low = f64::INFINITY;
        let mut high = f64::NEG_INFINITY;
        let start = alpha(lo);
        for k in 0..=256 {
            let psi = lo + (hi - lo) * k as f64 / 256.0;
            let a = start + (alpha(psi) - start + PI).rem_euclid(TAU) - PI;
            let (from, to) = if band == 0 { (a - 0.5 * PI, a + 0.5 * PI) } else { (a + 0.5 * PI, a + 1.5 * PI) };
            low = low.min(from);
            high = high.max(to);
        }
        let margin = (0.25 * (TAU - (high - low))).clamp(1e-3, 0.25);
        (low - margin, high + margin)
    };
    // The bands: per pose and half, the loop lower edge forward, seam at hi up,
    // upper edge back, seam at lo down (counter-clockwise in (ψ, v)).
    for pose in 0..2 {
        for (h, &(lo, hi)) in halves.iter().enumerate() {
            let (v_low, v_high) = band_range(lo, hi, pose);
            let theta_v = v_high - v_low;
            if theta_v >= TAU {
                return Err(refuse("ring_fold", "a band of the ring wraps all the way round its tube".into()));
            }
            let generatrix = tube_arc(lo, v_low, v_high)?;
            let surface = frag(make_revolution(origin, w, &generatrix, hi - lo), "ring_fold_band")?;
            let spans_u = (surface.knots_u.len() - 6) / 2 + 1;
            let spans_v = (generatrix.knots.len() - 6) / 2 + 1;
            let uv_of = |x: Vec3| -> Vec3 {
                let r = x.sub(origin);
                let psi = r.dot(u2).atan2(r.dot(u1));
                let mid = 0.5 * (lo + hi);
                let psi = mid + (psi - mid + PI).rem_euclid(TAU) - PI;
                let q = x.sub(tube_centre(psi));
                let v = q.dot(w).atan2(q.dot(radial(psi)));
                let mid_v = 0.5 * (v_low + v_high);
                let v = mid_v + (v - mid_v + PI).rem_euclid(TAU) - PI;
                Vec3::new(crate::circle_angle_to_parameter(spans_u, hi - lo, psi - lo), crate::circle_angle_to_parameter(spans_v, theta_v, v - v_low), 0.0)
            };
            let pcurve = |edge: u64, forward: bool| -> Result<NurbsCurve, KernelRefusal> {
                let tool = &edge_tool[&edge];
                let [d0, d1] = frag(tool.domain(), "domain")?;
                let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
                let uv = ts.iter().map(|&t| frag(tool.evaluate(t), "point").map(uv_of)).collect::<Result<Vec<_>, _>>()?;
                let curve = frag(crate::interpolate_curve(&uv, 5, &ts), "ring_fold_pcurve")?;
                if forward { Ok(curve) } else { frag(curve.reversed(), "reversed") }
            };
            // Band 0's lower edge is branch 0; band 1's is branch 1.
            let lower_branch = if pose == 0 { 0 } else { 1 };
            let edge_path = |branch: usize| -> Vec<(u64, bool)> { if branch == inner { paths[&(pose, h)].clone() } else { vec![(outer_edge[&(pose, h)], true)] } };
            let mut coedges = Vec::new();
            for (edge, forward) in edge_path(lower_branch) {
                coedges.push((edge, forward, pcurve(edge, forward)?));
            }
            let up = seam_arc[&(pose, seam_index(h, true))];
            coedges.push((up, true, pcurve(up, true)?));
            for (edge, forward) in edge_path(1 - lower_branch).into_iter().rev() {
                coedges.push((edge, !forward, pcurve(edge, !forward)?));
            }
            let down = seam_arc[&(pose, seam_index(h, false))];
            coedges.push((down, false, pcurve(down, false)?));
            shell.face(screw.move_surface(pose_time[pose], &surface)?, coedges);
        }
    }
    // Upright edges: at each seam on each branch, end to start; at each
    // crossing, lead end → trail end → lead start → trail start.
    let mut upright: HashMap<(usize, usize), (u64, Vec3, Vec3)> = HashMap::new(); // (seam, branch)
    for (k, &psi) in seams.iter().enumerate() {
        for branch in 0..2 {
            let (s, e) = (vertex(&mut shell, 0, psi, branch), vertex(&mut shell, 1, psi, branch));
            let (xa, xb) = (at(0, outline(psi, branch)), at(1, outline(psi, branch)));
            let id = shell.edge(frag(make_line(xa, xb), "line")?, s, e, false)?;
            upright.insert((k, branch), (id, xa, xb));
        }
    }
    // (edge, its start, its end) per crossing (per half), lead end up.
    let mut column: Vec<Vec<(u64, Vec3, Vec3)>> = Vec::new();
    for h in 0..2 {
        let (a, b, _, lead_at_a) = half_tail[h];
        let (lead, trail) = if lead_at_a { (a, b) } else { (b, a) };
        let points = [(0, lead), (0, trail), (1, lead), (1, trail)];
        let mut list = Vec::new();
        for k in 0..3 {
            let (pa, sa) = points[k];
            let (pb, sb) = points[k + 1];
            let (xa, xb) = (at(pa, outline(sa, inner)), at(pb, outline(sb, inner)));
            let (s, e) = (vertex(&mut shell, pa, sa, inner), vertex(&mut shell, pb, sb, inner));
            let id = shell.edge(frag(make_line(xa, xb), "line")?, s, e, false)?;
            list.push((id, xa, xb));
        }
        column.push(list);
    }
    // A face on the ruled surface c(s) + v·W over s in [0, 1] (c the pose's
    // `base` edge, ψ order), outward along the ring's normal at the base; its
    // top either the edge at v = 1 (`top`) or a shadow with its heights; its
    // sides (edge, start, end) at s = 0 and s = 1, from the base up.
    enum Top {
        Carried(u64),
        Shadow(u64, Vec<f64>),
    }
    let ruled = |shell: &mut Shell, base: u64, base_world: &NurbsCurve, along: Vec3, outward: Vec3, top: Top, sides: [Vec<(u64, Vec3, Vec3)>; 2]| -> Result<(), KernelRefusal> {
        let (c0, c1) = (frag(base_world.evaluate(0.0), "point")?, frag(base_world.evaluate(1.0), "point")?);
        let tangent = frag(base_world.evaluate(0.5 + 1e-4), "point")?.sub(frag(base_world.evaluate(0.5 - 1e-4), "point")?);
        let rev = tangent.cross(along).dot(outward) < 0.0;
        let curve = if rev { frag(base_world.reversed(), "reversed")? } else { base_world.clone() };
        let rows: Vec<Vec<Vec4>> = curve
            .control_points
            .iter()
            .map(|p| -> Result<Vec<Vec4>, KernelRefusal> {
                let x = frag(p.point(), "control_point")?;
                Ok(vec![*p, Vec4::from_point(x.add(along), p.w)])
            })
            .collect::<Result<_, _>>()?;
        let surface = frag(NurbsSurface::new(curve.degree, 1, curve.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], rows), "ring_fold_wall")?;
        let u_of = |s: f64| if rev { 1.0 - s } else { s };
        let v_at = |x: Vec3, s_end: Vec3| x.sub(s_end).dot(along) / along.dot(along);
        // The loop in (u, v), counter-clockwise: bottom, side at u = 1 up,
        // top back, side at u = 0 down.
        let (side_u1, side_u0, foot_u1, foot_u0) = if rev { (&sides[0], &sides[1], c0, c1) } else { (&sides[1], &sides[0], c1, c0) };
        let mut coedges = Vec::new();
        coedges.push((base, !rev, uv_line([0.0, 0.0], [1.0, 0.0])?));
        for &(edge, xa, xb) in side_u1 {
            let (va, vb) = (v_at(xa, foot_u1), v_at(xb, foot_u1));
            let up = vb > va;
            coedges.push((edge, up, if up { uv_line([1.0, va], [1.0, vb])? } else { uv_line([1.0, vb], [1.0, va])? }));
        }
        match top {
            Top::Carried(edge) => coedges.push((edge, rev, uv_line([1.0, 1.0], [0.0, 1.0])?)),
            Top::Shadow(edge, heights) => {
                let n = heights.len() - 1;
                // In ψ order (the shadow edge's own direction is chosen by its step).
                let mut uv: Vec<[f64; 2]> = (0..=n).map(|k| [u_of(k as f64 / n as f64), heights[k]]).collect();
                if !rev {
                    uv.reverse();
                }
                let pc = fit_uv(&uv)?;
                // The shadow edge runs in ψ order, like the base.
                coedges.push((edge, rev, pc));
            }
        }
        for &(edge, xa, xb) in side_u0.iter().rev() {
            let (va, vb) = (v_at(xa, foot_u0), v_at(xb, foot_u0));
            let down = vb < va;
            coedges.push((edge, down, if down { uv_line([0.0, va], [0.0, vb])? } else { uv_line([0.0, vb], [0.0, va])? }));
        }
        shell.face(surface, coedges);
        Ok(())
    };
    let world = |pose: usize, edge: u64| moved(pose, &edge_tool[&edge]);
    let outward_at = |psi: f64, branch: usize| normal(psi, branch_v(psi, branch));
    let back = push.scale(-1.0);
    for (h, &(lo, hi)) in halves.iter().enumerate() {
        let (a, b, _, lead_at_a) = half_tail[h];
        let (lo_seam, hi_seam) = (seam_index(h, false), seam_index(h, true));
        // The outer wall's half.
        let base = outer_edge[&(0, h)];
        ruled(&mut shell, base, &world(0, base)?, back, outward_at(0.5 * (lo + hi), outer), Top::Carried(outer_edge[&(1, h)]), [vec![upright[&(lo_seam, outer)]], vec![upright[&(hi_seam, outer)]]])?;
        // The arcs' walls: from a lead crossing its column runs lead end ->
        // trail end -> lead start; from a trail one, trail end -> lead start ->
        // trail start.
        let col = column[h].clone();
        let side_at = |lead: bool| if lead { vec![col[0], col[1]] } else { vec![col[1], col[2]] };
        let tail = arc_edges[&(0, h, 0)];
        ruled(&mut shell, tail, &world(0, tail)?, back, outward_at(0.5 * (lo + a), inner), Top::Carried(arc_edges[&(1, h, 0)]), [vec![upright[&(lo_seam, inner)]], side_at(lead_at_a)])?;
        let head = arc_edges[&(0, h, 1)];
        ruled(&mut shell, head, &world(0, head)?, back, outward_at(0.5 * (b + hi), inner), Top::Carried(arc_edges[&(1, h, 1)]), [side_at(!lead_at_a), vec![upright[&(hi_seam, inner)]]])?;
        // The steps: the end ring's up from the lead crossing's end point, the
        // start ring's down from the trail crossing's start point.
        for pose in 0..2 {
            let step = &steps[&(pose, h)];
            let (along, side) = if pose == 0 { (back, col[0]) } else { (push, col[2]) };
            let sides = if step.crossing_first { [vec![side], vec![]] } else { [vec![], vec![side]] };
            let [d0, d1] = frag(step.curve.domain(), "domain")?;
            let mid = frag(step.curve.evaluate(0.5 * (d0 + d1)), "point")?;
            let r = mid.sub(origin);
            let psi = r.dot(u2).atan2(r.dot(u1));
            ruled(&mut shell, step.edge, &world(pose, step.edge)?, along, outward_at(psi, inner), Top::Shadow(step.shadow, step.heights.clone()), sides)?;
        }
    }
    let body = shell.solid();
    let problems = body.validate();
    if !problems.is_empty() {
        return Err(refuse("ring_fold", format!("the folded ring's sweep does not validate ({} problems)", problems.len())));
    }
    Ok(Some(body))
}
