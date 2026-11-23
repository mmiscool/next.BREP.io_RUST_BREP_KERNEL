//! A RING (a torus about any axis) carried by a frame along a planar cubic,
//! read slice by slice: `BREP_SWEEP_PLANAR_RING` (on by default; `=0` restores the general route).
//!
//! The frame turns only about `up`, square to the curve's plane, so the motion
//! is a motion of each plane square to `up` in itself: every slice of the sweep
//! at a height `h` is what the ring's slice at `h` sweeps in that plane. The
//! ring is every point within its tube's radius `r` of its core circle, so the
//! sweep is every point whose distance to the nearest of the core circles the
//! frame carries (that minimised over the stations) is at most `r`. Where the
//! ring stands in the plane of its own motion (a wheel), its grazing set is
//! degenerate at the top and bottom of its tube and the general route refuses;
//! this read has no grazing set at all.
//!
//! Each slice is read on rays square to the curve at every station and on a
//! fan of rays round each end of the curve, each boundary point the root of
//! (distance to the carried circles) − `r` along its ray; a ray that does not
//! leave the slice exactly once is refused. Each slice's points are
//! interpolated as four cubics (each side, each end's fan), each piece on one
//! parameter line shared by every slice, and the slices are lofted. The sweep's extreme heights, where a
//! slice closes up onto a line, stand `GAP` inside the ring's own.
use crate::{interpolate_curve, loft_profile_brep, BrepSolid, FrameCurve, KernelRefusal, KernelStage, OutlinePiece, SolidTool, Vec3};

/// `BREP_SWEEP_PLANAR_RING`: on by default (`=0` off); on, [`crate::sweep_solid_along`]
/// builds a ring on a planar curve from its slices.
pub fn planar_ring_on() -> bool {
    crate::switch!("BREP_SWEEP_PLANAR_RING", true)
}

/// Rays along each side, rays round each end, slices each side of the
/// ring's middle height, and the extreme slices' distance in from the
/// ring's top and bottom (relative to the tube).
const SIDE: usize = 120;
const FAN: usize = 48;
const SLICES: usize = 40;
const GAP: f64 = 2e-5;

fn refuse(what: &'static str, message: impl Into<String>) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("sweepSolid: planar ring: {}", message.into()))
}

/// The ring's core circle (centre, unit axis, radius) and tube radius, where
/// `tool` is a revolved full circle off its axis.
fn ring_of(tool: &SolidTool) -> Option<(Vec3, Vec3, f64, f64)> {
    let SolidTool::Revolved { axis_origin, axis_direction, radial: _, meridian } = tool else { return None };
    let [OutlinePiece::Arc { centre, radius, start, end }] = meridian[..] else { return None };
    let axis = axis_direction.normalized().ok()?;
    let full = ((end - start).abs() - std::f64::consts::TAU).abs() <= 1e-12;
    if !(full && radius > 0.0 && centre[0] > radius && [centre[0], centre[1], radius].iter().all(|x| x.is_finite())) {
        return None;
    }
    Some((axis_origin.add(axis.scale(centre[1])), axis, centre[0], radius))
}

struct Motion {
    control: [Vec3; 4],
    up: Vec3,
    start: Vec3,
    start_tangent: Vec3,
    /// Parameters and arc lengths, densely.
    table: Vec<(f64, f64)>,
}

impl Motion {
    fn point(&self, s: f64) -> Vec3 {
        let [p0, p1, p2, p3] = self.control;
        let r = 1.0 - s;
        p0.scale(r * r * r).add(p1.scale(3.0 * r * r * s)).add(p2.scale(3.0 * r * s * s)).add(p3.scale(s * s * s))
    }
    fn first(&self, s: f64) -> Vec3 {
        let [p0, p1, p2, p3] = self.control;
        let r = 1.0 - s;
        p1.sub(p0).scale(3.0 * r * r).add(p2.sub(p1).scale(6.0 * r * s)).add(p3.sub(p2).scale(3.0 * s * s))
    }
    fn tangent(&self, s: f64) -> Vec3 {
        let d = self.first(s);
        d.scale(1.0 / d.length())
    }
    fn length(&self) -> f64 {
        self.table.last().map_or(0.0, |entry| entry.1)
    }
    fn at_length(&self, l: f64) -> f64 {
        let l = l.clamp(0.0, self.length());
        let i = self.table.partition_point(|entry| entry.1 < l).clamp(1, self.table.len() - 1);
        let ((t0, l0), (t1, l1)) = (self.table[i - 1], self.table[i]);
        if l1 > l0 { t0 + (t1 - t0) * (l - l0) / (l1 - l0) } else { t0 }
    }
    /// `q` carried back from station `s` to the start pose (the frame turns
    /// about `up` through the curve's point).
    fn to_start(&self, q: Vec3, s: f64) -> Vec3 {
        let t = self.tangent(s);
        let (cos, sin) = (self.start_tangent.dot(t), self.up.dot(self.start_tangent.cross(t)));
        let d = q.sub(self.point(s));
        // Turn by minus the frame's angle about `up`.
        let along = self.up.scale(d.dot(self.up));
        let flat = d.sub(along);
        let turned = flat.scale(cos).sub(self.up.cross(flat).scale(sin)).add(along);
        self.start.add(turned)
    }
}

struct Ring {
    centre: Vec3,
    axis: Vec3,
    core: f64,
    tube: f64,
}

impl Ring {
    fn to_core(&self, q: Vec3) -> f64 {
        let d = q.sub(self.centre);
        let h = d.dot(self.axis);
        let rho = d.sub(self.axis.scale(h)).length();
        (h * h + (rho - self.core).powi(2)).sqrt()
    }
}

/// The sweep of `tool` along `path` from its slices, `None` where the tool is
/// not a ring or the path is not a planar cubic square to `up`.
pub fn planar_ring_sweep(tool: &SolidTool, path: &FrameCurve) -> Option<Result<BrepSolid, KernelRefusal>> {
    let (centre, axis, core, tube) = ring_of(tool)?;
    let up = path.up.normalized().ok()?;
    let base = path.control[0];
    let size = path.control.iter().map(|p| p.sub(base).length()).fold(0.0, f64::max);
    if !(size > 0.0) || path.control.iter().any(|p| p.sub(base).dot(up).abs() > 1e-12 * size) {
        return None;
    }
    let n = 20000;
    let probe = Motion { control: path.control, up, start: base, start_tangent: up, table: Vec::new() };
    let first = probe.first(0.0);
    if !(first.length() > 1e-9 * size) {
        return None;
    }
    let mut table = vec![(0.0, 0.0)];
    let (mut previous, mut length) = (base, 0.0);
    for i in 1..=n {
        let t = i as f64 / n as f64;
        if !(probe.first(t).length() > 1e-6 * size) {
            return None;
        }
        let q = probe.point(t);
        length += q.sub(previous).length();
        previous = q;
        table.push((t, length));
    }
    let motion = Motion { start_tangent: first.scale(1.0 / first.length()), table, ..probe };
    Some(build(&motion, &Ring { centre, axis, core, tube }))
}

fn build(motion: &Motion, ring: &Ring) -> Result<BrepSolid, KernelRefusal> {
    let up = motion.up;
    let length = motion.length();
    // The ring's height above the curve's plane, its middle and half extent
    // (unchanged by turns about `up`), and its reach across the plane from
    // the frame's origin.
    let base = motion.start;
    let middle = ring.centre.sub(base).dot(up);
    let tilt = ring.axis.dot(up).clamp(-1.0, 1.0);
    let half = ring.core * (1.0 - tilt * tilt).sqrt() + ring.tube;
    let off = ring.centre.sub(base);
    let reach = off.sub(up.scale(off.dot(up))).length() + ring.core + ring.tube;
    let window = 2.0 * reach;
    let to_circles = |q: Vec3, near: f64| -> f64 {
        let (a, b) = ((near - window).max(0.0), (near + window).min(length));
        let steps = 64;
        let ts: Vec<f64> = (0..=steps).map(|k| motion.at_length(a + (b - a) * k as f64 / steps as f64)).collect();
        let values: Vec<f64> = ts.iter().map(|&t| ring.to_core(motion.to_start(q, t))).collect();
        let best = (0..=steps).min_by(|&i, &j| values[i].total_cmp(&values[j])).unwrap_or(0);
        let (mut lo, mut hi) = (ts[best.saturating_sub(1)], ts[(best + 1).min(steps)]);
        let g = 0.5 * (5f64.sqrt() - 1.0);
        let at = |t: f64| ring.to_core(motion.to_start(q, t));
        let (mut x1, mut x2) = (hi - g * (hi - lo), lo + g * (hi - lo));
        let (mut f1, mut f2) = (at(x1), at(x2));
        for _ in 0..48 {
            if f1 <= f2 {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - g * (hi - lo);
                f1 = at(x1);
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + g * (hi - lo);
                f2 = at(x2);
            }
        }
        values[best].min(f1).min(f2)
    };
    let far = reach + ring.tube;
    // Where a ray leaves the slice, to `steps` halvings; `checked`, that it
    // leaves exactly once.
    let leave = |from: Vec3, direction: Vec3, near: f64, steps: usize, checked: bool| -> Result<Vec3, KernelRefusal> {
        let inside = |r: f64| to_circles(from.add(direction.scale(r)), near) <= ring.tube;
        if !inside(0.0) || inside(far) {
            return Err(refuse("planar_ring_ray", "a slice does not hold the curve's point, or reaches past the ring"));
        }
        let (mut lo, mut hi) = (0.0, far);
        for _ in 0..steps {
            let mid = 0.5 * (lo + hi);
            if inside(mid) { lo = mid } else { hi = mid }
        }
        if !checked {
            return Ok(from.add(direction.scale(0.5 * (lo + hi))));
        }
        // The ray leaves the slice once: inside all the way in, outside all
        // the way out.
        let checks = 24;
        if (1..checks).any(|k| !inside(lo * k as f64 / checks as f64)) || (1..=checks).any(|k| inside(hi + (far - hi) * k as f64 / checks as f64)) {
            return Err(refuse("planar_ring_star", "a slice is not crossed once by a ray from the curve"));
        }
        Ok(from.add(direction.scale(0.5 * (lo + hi))))
    };
    let exit = |from: Vec3, direction: Vec3, near: f64| leave(from, direction, near, 52, true);
    let slice = |h: f64| -> Result<Vec<Vec3>, KernelRefusal> {
        let lift = up.scale(h);
        let station = |i: usize| length * i as f64 / SIDE as f64;
        let side = |i: usize, sign: f64| -> Result<Vec3, KernelRefusal> {
            let t = motion.at_length(station(i));
            let left = up.cross(motion.tangent(t));
            exit(motion.point(t).add(lift), left.scale(sign), station(i))
        };
        let mut points = Vec::with_capacity(2 * (SIDE + FAN));
        for i in 0..=SIDE {
            points.push(side(i, -1.0)?);
        }
        for (half_turn, (t, at)) in [(1.0, length), (0.0, 0.0)].into_iter().enumerate() {
            let tangent = motion.tangent(t);
            let left = up.cross(tangent);
            // From the right round the end to the left, or from the left
            // round the start to the right; the rays spaced evenly along the
            // end's boundary (read first on a coarse fan), not in angle: at
            // middle heights an end is a straight run past the curve's end and
            // then the ring's round front, which even angles barely reach.
            let from = motion.point(t).add(lift);
            let ray = |angle: f64| tangent.scale(angle.cos()).add(left.scale(angle.sin()));
            let angle_at = |w: f64| -std::f64::consts::FRAC_PI_2 + std::f64::consts::PI * (w + half_turn as f64);
            let coarse = 4 * FAN;
            let rough: Vec<Vec3> = (0..=coarse).map(|k| leave(from, ray(angle_at(k as f64 / coarse as f64)), at, 24, false)).collect::<Result<_, _>>()?;
            let mut along = vec![0.0];
            for k in 1..=coarse {
                along.push(along[k - 1] + rough[k].sub(rough[k - 1]).length());
            }
            let total = along[coarse];
            for k in 1..FAN {
                let target = total * k as f64 / FAN as f64;
                let j = along.partition_point(|&a| a < target).clamp(1, coarse);
                let w = (j as f64 - 1.0 + (target - along[j - 1]) / (along[j] - along[j - 1]).max(f64::MIN_POSITIVE)) / coarse as f64;
                points.push(exit(from, ray(angle_at(w)), at)?);
            }
            if half_turn == 0 {
                for i in (0..=SIDE).rev() {
                    points.push(side(i, 1.0)?);
                }
            }
        }
        Ok(points)
    };
    // Slices closer together towards the top and bottom, where a slice's width
    // grows as the square root of its distance in.
    let span = half - GAP * ring.tube;
    let heights: Vec<f64> = (-(SLICES as i64)..=SLICES as i64)
        .map(|j| {
            let u = j as f64 / SLICES as f64;
            middle + span * u.signum() * (1.0 - (1.0 - u.abs()).powi(2))
        })
        .collect();
    let rings: Vec<Vec<Vec3>> = heights.iter().map(|&h| slice(h)).collect::<Result<_, _>>()?;
    // Four curves a slice: the right side, the fan round the end, the left
    // side, the fan round the start, each sharing its end points with the
    // next. A side keeps its length from slice to slice while a fan shrinks
    // to nothing at the top and bottom, so each piece takes its own
    // parameters (the middle slice's chord lengths): shared across a whole
    // slice they would crowd a bottom slice's tiny fan into a third of the
    // curve and loop it at the corners.
    let n = SIDE + 1;
    let pieces = |points: &[Vec3]| -> [Vec<Vec3>; 4] {
        let fan = FAN - 1;
        let right = points[..n].to_vec();
        let end = points[n - 1..=n + fan].to_vec();
        let left = points[n + fan..2 * n + fan].to_vec();
        let mut start = points[2 * n + fan - 1..].to_vec();
        start.push(points[0]);
        [right, end, left, start]
    };
    let lines: Vec<Vec<f64>> = pieces(&rings[SLICES])
        .iter()
        .map(|piece| {
            let mut line = vec![0.0];
            for k in 1..piece.len() {
                line.push(line[k - 1] + piece[k].sub(piece[k - 1]).length());
            }
            let total = line[line.len() - 1];
            line.iter().map(|l| l / total).collect()
        })
        .collect();
    let fit = |points: &[Vec3], line: &[f64]| interpolate_curve(points, 3, line).map_err(|why| refuse("planar_ring_fit", why));
    let sections: Vec<Vec<_>> = rings
        .iter()
        .map(|points| pieces(points).iter().zip(&lines).map(|(piece, line)| fit(piece, line)).collect::<Result<Vec<_>, _>>())
        .collect::<Result<_, KernelRefusal>>()?;
    loft_profile_brep(&sections)
}

