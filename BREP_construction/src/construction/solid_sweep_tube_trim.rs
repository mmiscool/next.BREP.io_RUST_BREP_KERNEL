//! The tube route's self-trim (stage 1 and 2): where the motion carries the
//! ring back over K, K is cut along the curves where its points' depth under
//! the other poses falls to r, and only the uncovered part bounds the solid.
//!
//! K's domain is (σ, ψ), σ in [0, 4) round its four charts and ψ in [0, 2π),
//! periodic both ways. A lattice over it (aligned with the patches' sides) is
//! read for g = depth − r, depth the least distance to the core circle over
//! the poses away from the point's own time; the zero set is marched cell by
//! cell, each crossing solved on its lattice edge from exact K. Loops of the
//! zero set are returned with each point's mate: the pose and foot that reach
//! it, which name the point of K on the other side of the cut.
use super::Tube;
use crate::Vec3;

/// A point of a trim curve: where on K, where its mate is (the pose that
/// reaches it), and the 3D point.
#[derive(Clone, Copy, Debug)]
pub struct TrimPoint {
    pub sigma: f64,
    pub psi: f64,
    pub mate_time: f64,
    pub point: Vec3,
}

/// The lattice over K's domain: σ stations (each chart's chunks, `per_chunk`
/// cells each) and ψ stations (`per_quarter` cells each of four quarters).
pub struct Lattice {
    pub sigmas: Vec<f64>,
    pub psis: Vec<f64>,
    pub per_chunk: usize,
    pub per_quarter: usize,
}

impl Lattice {
    pub fn new(chunks: [usize; 4], per_chunk: usize, per_quarter: usize) -> Lattice {
        let mut sigmas = Vec::new();
        for (chart, &count) in chunks.iter().enumerate() {
            for c in 0..count {
                for k in 0..per_chunk {
                    sigmas.push(chart as f64 + (c as f64 + k as f64 / per_chunk as f64) / count as f64);
                }
            }
        }
        let quarters = 4 * per_quarter;
        let psis = (0..quarters).map(|k| std::f64::consts::TAU * k as f64 / quarters as f64).collect();
        Lattice { sigmas, psis, per_chunk, per_quarter }
    }
}

impl Tube {
    /// K at the domain point (σ, ψ), σ in [0, 4).
    pub fn k_at(&self, sigma: f64, psi: f64) -> Option<Vec3> {
        let chart = (sigma.floor() as i64).rem_euclid(4) as usize;
        self.k_point(chart, sigma - sigma.floor(), psi)
    }

    /// The time whose pose K's point (σ, ψ) belongs to.
    pub(super) fn own_time(&self, sigma: f64) -> f64 {
        let chart = (sigma.floor() as i64).rem_euclid(4) as usize;
        let s = sigma - sigma.floor();
        match chart {
            0 => self.from + (self.to - self.from) * s,
            1 => self.to,
            2 => self.to - (self.to - self.from) * s,
            _ => self.from,
        }
    }

    /// The least distance from `q` to the core circle over the poses outside
    /// (own − window, own + window), read to `tolerance` (pose intervals
    /// cleared by the speed bound or bisected), with the time that reaches it.
    pub fn far_depth(&self, q: Vec3, own: f64, window: f64, tolerance: f64) -> (f64, f64) {
        self.far_depth_until(q, own, window, tolerance, f64::NEG_INFINITY)
    }

    /// [`Tube::far_depth`], returning as soon as some pose is nearer than `stop`.
    pub fn far_depth_until(&self, q: Vec3, own: f64, window: f64, tolerance: f64, stop: f64) -> (f64, f64) {
        let speed = self.reach_speed(q);
        let mut spans: Vec<(f64, f64)> = Vec::new();
        let (a, b) = (own - window, own + window);
        if a > self.from {
            spans.push((self.from, a.min(self.to)));
        }
        if b < self.to {
            spans.push((b.max(self.from), self.to));
        }
        let mut best = (f64::INFINITY, own);
        let mut stack: Vec<(f64, f64)> = Vec::new();
        for (lo, hi) in spans {
            let count = 256usize;
            let h = (hi - lo) / count as f64;
            stack.extend((0..count).map(|k| (lo + h * k as f64, lo + h * (k + 1) as f64)));
        }
        while let Some((lo, hi)) = stack.pop() {
            let (fl, fh) = (self.reach(q, lo), self.reach(q, hi));
            if fl < best.0 {
                best = (fl, lo);
            }
            if fh < best.0 {
                best = (fh, hi);
            }
            if best.0 < stop {
                return best;
            }
            if fl.min(fh) - speed * (hi - lo) * 0.5 >= best.0 - tolerance || hi - lo <= 1e-13 * (self.to - self.from) {
                continue;
            }
            let m = 0.5 * (lo + hi);
            stack.push((lo, m));
            stack.push((m, hi));
        }
        best
    }

    /// The stretch about `own` the far reads leave out: within `window` of it,
    /// only as far as the distance keeps rising away from own (the own time's
    /// basin, stepped out by doublings from 1e-7 of the range). Near a fold the
    /// distance falls at once on the folded side, so the poses there, which
    /// hold the point, are read as competitors.
    pub fn own_basin(&self, q: Vec3, own: f64, window: f64) -> (f64, f64) {
        let range = self.to - self.from;
        // Each side's edge is the distance's local maximum, found exactly: the
        // doubling steps bracket it (it rose to the last step and fell at the
        // next), and a golden section on that bracket places it, so a far read
        // never lands on a cut that is not a stationary point.
        let side = |sign: f64| -> f64 {
            let mut before = 0.0f64;
            let mut last = (0.0f64, self.reach(q, own));
            let mut h = 1e-7 * range;
            // On past the window while the distance still rises (a slow pose
            // leaves the basin wide): a cut at the window is no stationary
            // point, and a far read would find its least on it. A side that
            // rises to the range's end is basin throughout.
            let _ = window;
            while h <= 2.0 * range {
                let t = own + sign * h;
                if t < self.from || t > self.to {
                    return if sign > 0.0 { self.to - own } else { own - self.from } + 1e-12 * range;
                }
                let value = self.reach(q, t);
                if value < last.1 {
                    let (mut a, mut b) = (before, h);
                    let ratio = 0.5 * (5f64.sqrt() - 1.0);
                    for _ in 0..60 {
                        if b - a <= 1e-15 * range {
                            break;
                        }
                        let (c, d) = (b - ratio * (b - a), a + ratio * (b - a));
                        if self.reach(q, own + sign * c) > self.reach(q, own + sign * d) {
                            b = d;
                        } else {
                            a = c;
                        }
                    }
                    return 0.5 * (a + b);
                }
                before = last.0;
                last = (h, value);
                h *= 2.0;
            }
            2.0 * range
        };
        (own - side(-1.0), own + side(1.0))
    }

    /// The far depth by sampling (`count` poses outside the window, the five
    /// least local minima refined by golden section): fast, NOT certified
    /// (a dip narrower than a pose step can be missed). Prototype only.
    pub fn far_depth_sampled(&self, q: Vec3, own: f64, window: f64, count: usize) -> (f64, f64) {
        let mut times: Vec<f64> = Vec::with_capacity(count + 1);
        let (lo, hi) = self.own_basin(q, own, window);
        for k in 0..=count {
            let t = self.from + (self.to - self.from) * k as f64 / count as f64;
            if t <= lo || t >= hi {
                times.push(t);
            }
        }
        if lo > self.from && lo < self.to {
            times.push(lo);
        }
        if hi > self.from && hi < self.to {
            times.push(hi);
        }
        times.sort_by(f64::total_cmp);
        let values: Vec<f64> = times.iter().map(|&t| self.reach(q, t)).collect();
        let n = times.len();
        if n == 0 {
            return (f64::INFINITY, own);
        }
        let mut best = (f64::INFINITY, own);
        for k in 0..n {
            if values[k] < best.0 {
                best = (values[k], times[k]);
            }
        }
        let mut minima: Vec<usize> = (0..n).filter(|&k| (k == 0 || values[k] <= values[k - 1]) && (k + 1 == n || values[k] <= values[k + 1])).collect();
        minima.sort_by(|&x, &y| values[x].total_cmp(&values[y]));
        let ratio = 0.5 * (5f64.sqrt() - 1.0);
        for &k in minima.iter().take(5) {
            let (mut a, mut b) = (times[k.saturating_sub(1)], times[(k + 1).min(n - 1)]);
            // never refine across the own-time window
            if a < hi && b > lo && (a - own).abs() > 0.0 {
                if times[k] <= lo { b = b.min(lo); } else { a = a.max(hi); }
            }
            for _ in 0..80 {
                if b - a <= 1e-14 * (self.to - self.from) {
                    break;
                }
                let (c, d) = (b - ratio * (b - a), a + ratio * (b - a));
                if self.reach(q, c) < self.reach(q, d) {
                    b = d;
                } else {
                    a = c;
                }
            }
            let t = 0.5 * (a + b);
            let value = self.reach(q, t);
            if value < best.0 {
                best = (value, t);
            }
        }
        best
    }

    /// The far depth near a known reaching time `guess` (a trim curve's
    /// competitor moves continuously along it): the least of 33 poses over
    /// guess ± `span` (outside the own window) refined by golden section.
    pub fn far_depth_near(&self, q: Vec3, own: f64, window: f64, guess: f64, span: f64) -> (f64, f64) {
        let lo = (guess - span).max(self.from);
        let hi = (guess + span).min(self.to);
        let (left, right) = self.own_basin(q, own, window);
        let allowed = |t: f64| t <= left || t >= right;
        let mut best = (f64::INFINITY, guess);
        let mut best_k = None;
        let count = 16;
        let samples: Vec<f64> = (0..=count).map(|k| lo + (hi - lo) * k as f64 / count as f64).collect();
        let mut values = vec![f64::INFINITY; count + 1];
        for (k, &t) in samples.iter().enumerate() {
            if allowed(t) {
                values[k] = self.reach(q, t);
                if values[k] < best.0 {
                    best = (values[k], t);
                    best_k = Some(k);
                }
            }
        }
        let Some(k) = best_k else { return self.far_depth_sampled(q, own, window, 1024) };
        // A least at the span's edge (not the range's) is no local minimum:
        // the competitor moved beyond the span, so read the whole range.
        if (k == 0 && lo > self.from) || (k == count && hi < self.to) {
            return self.far_depth_sampled(q, own, window, 1024);
        }
        // A bracket about the least sample, refined by parabolic steps
        // (the distance is smooth and nearly quadratic at its least) with a
        // golden step wherever a parabola would leave the bracket or stall.
        let (mut a, mut b) = (samples[k.saturating_sub(1)], samples[(k + 1).min(count)]);
        if !allowed(a) { a = samples[k]; }
        if !allowed(b) { b = samples[k]; }
        let (mut x, mut fx) = (best.1, best.0);
        let ratio = 0.5 * (3.0 - 5f64.sqrt());
        for _ in 0..40 {
            if b - a <= 1e-13 * (self.to - self.from) {
                break;
            }
            let (fa, fb) = (self.reach(q, a), self.reach(q, b));
            let denominator = (x - a) * (fx - fb) - (x - b) * (fx - fa);
            let mut u = if denominator.abs() > 0.0 { x - 0.5 * ((x - a) * (x - a) * (fx - fb) - (x - b) * (x - b) * (fx - fa)) / denominator } else { f64::NAN };
            if !(u > a && u < b) || (u - x).abs() < 1e-15 * (self.to - self.from) {
                u = if x - a > b - x { x - ratio * (x - a) } else { x + ratio * (b - x) };
            }
            let fu = self.reach(q, u);
            if fu < fx {
                if u < x { b = x; } else { a = x; }
                x = u;
                fx = fu;
            } else if u < x {
                a = u;
            } else {
                b = u;
            }
        }
        if fx < best.0 { (fx, x) } else { best }
    }

    /// Whether every pose outside the own window keeps `q` beyond `level`
    /// (Some(true), certified by the speed bound), some pose brings it within
    /// (Some(false)), or neither is settled within `budget` intervals (None).
    pub fn beyond(&self, q: Vec3, own: f64, window: f64, level: f64, budget: usize) -> Option<bool> {
        let speed = self.reach_speed(q);
        let mut stack: Vec<(f64, f64)> = Vec::new();
        let (lo, hi) = (own - window, own + window);
        for (a, b) in [(self.from, lo.min(self.to)), (hi.max(self.from), self.to)] {
            if b > a {
                let count = 64usize;
                let h = (b - a) / count as f64;
                stack.extend((0..count).map(|k| (a + h * k as f64, a + h * (k + 1) as f64)));
            }
        }
        let mut spent = 0usize;
        while let Some((a, b)) = stack.pop() {
            let (fa, fb) = (self.reach(q, a), self.reach(q, b));
            if fa < level || fb < level {
                return Some(false);
            }
            if fa.min(fb) - speed * (b - a) * 0.5 >= level {
                continue;
            }
            spent += 1;
            if spent > budget || b - a <= 1e-12 * (self.to - self.from) {
                return None;
            }
            let m = 0.5 * (a + b);
            stack.push((a, m));
            stack.push((m, b));
        }
        Some(true)
    }

    /// g = far depth − r at K's domain point, with the reaching time.
    pub(super) fn trim_value(&self, sigma: f64, psi: f64, window: f64, tolerance: f64) -> Option<(f64, f64, Vec3)> {
        let q = self.k_at(sigma, psi)?;
        let _ = tolerance;
        let (depth, time) = self.far_depth_sampled(q, self.own_time(sigma), window, 1024);
        Some((depth - self.minor, time, q))
    }
}

impl Tube {
    /// The domain point (σ, ψ) of K that the pose at `time` puts at `q` (q at
    /// distance r from that pose's core circle): on the sheet Σ + r·N or Σ − r·N
    /// inside the range, on the end torus's leading or the start torus's
    /// trailing half at the range's ends.
    pub fn mate_of(&self, q: Vec3, time: f64) -> Option<(f64, f64)> {
        if let Some(found) = self.mate_at(q, time) {
            return Some(found);
        }
        // A sampled time (a lattice read's least sample) is off the reaching
        // pose: the distance's least near it, by golden section over two
        // samples' spacing either side (clamped to the range), read again.
        let range = self.to - self.from;
        let spread = range / 512.0;
        let (mut lo, mut hi) = ((time - spread).max(self.from), (time + spread).min(self.to));
        let ratio = 0.5 * (5f64.sqrt() - 1.0);
        let (mut a, mut b) = (hi - ratio * (hi - lo), lo + ratio * (hi - lo));
        let (mut fa, mut fb) = (self.reach(q, a), self.reach(q, b));
        for _ in 0..80 {
            if !(hi - lo > 1e-15 * range) {
                break;
            }
            if fa <= fb {
                hi = b;
                (b, fb) = (a, fa);
                a = hi - ratio * (hi - lo);
                fa = self.reach(q, a);
            } else {
                lo = a;
                (a, fa) = (b, fb);
                b = lo + ratio * (hi - lo);
                fb = self.reach(q, b);
            }
        }
        let refined = 0.5 * (lo + hi);
        // At distance r from that pose's circle, or not its mate at all.
        if !((self.reach(q, refined) - self.minor).abs() <= 1e-6 * self.minor) {
            return None;
        }
        if let Some(found) = self.mate_at(q, refined) {
            return Some(found);
        }
        // A least inside the range is stationary, so its point is on a sheet
        // (± by its normal part): near a cusp the distance is too flat in time
        // for its lead part to settle within 1e-6 of r.
        if !((refined - self.from) > 1e-6 * range && (self.to - refined) > 1e-6 * range) {
            return None;
        }
        let local = self.motion.pose(refined).invert(q).sub(self.centre);
        let psi = local.dot(self.y).atan2(local.dot(self.x)).rem_euclid(std::f64::consts::TAU);
        let frame = self.frame(refined, psi)?;
        let sigma = if q.sub(frame.point).dot(frame.normal) >= 0.0 { (refined - self.from) / range } else { 2.0 + (self.to - refined) / range };
        Some((sigma, psi))
    }

    /// `mate_of` at an exact time.
    fn mate_at(&self, q: Vec3, time: f64) -> Option<(f64, f64)> {
        let local = self.motion.pose(time).invert(q).sub(self.centre);
        let psi = local.dot(self.y).atan2(local.dot(self.x)).rem_euclid(std::f64::consts::TAU);
        let frame = self.frame(time, psi)?;
        let offset = q.sub(frame.point);
        let (along_n, along_lead) = (offset.dot(frame.normal), offset.dot(frame.lead));
        let range = self.to - self.from;
        // Sheet or cap by where the point sits about its circle, not by an
        // exact time: a point off ±N (its lead part past 1e-6 of r) is on a
        // cap, whose pose must then be the range's end (the competitor read
        // lands a hair inside it).
        let on_sheet = along_lead.abs() <= 1e-6 * self.minor;
        let near_end = (self.to - time) <= 1e-6 * range;
        let near_start = (time - self.from) <= 1e-6 * range;
        let sigma = if !on_sheet && near_end && along_lead > 0.0 {
            1.0 + along_lead.atan2(along_n).clamp(0.0, std::f64::consts::PI) / std::f64::consts::PI
        } else if !on_sheet && near_start && along_lead < 0.0 {
            3.0 + (-along_lead).atan2(-along_n).clamp(0.0, std::f64::consts::PI) / std::f64::consts::PI
        } else if !on_sheet {
            return None; // off the sheets, yet not at an end of the range
        } else if along_n >= 0.0 {
            (time - self.from) / range
        } else {
            2.0 + (self.to - time) / range
        };
        Some((sigma, psi))
    }
}

/// The zero set of g over the lattice as closed loops of trim points (each
/// crossing solved on its lattice edge by bisection on exact K), or why not:
/// a cell with two crossings on each diagonal pair (a saddle) declines.
pub fn trim_loops(tube: &Tube, lattice: &Lattice, window: f64, tolerance: f64) -> Result<Vec<Vec<TrimPoint>>, String> {
    let (rows, columns) = (lattice.sigmas.len(), lattice.psis.len());
    let sigma_at = |i: usize| -> f64 { if i == rows { 4.0 } else { lattice.sigmas[i] } };
    let psi_at = |j: usize| -> f64 { if j == columns { std::f64::consts::TAU } else { lattice.psis[j] } };
    let mut values = vec![vec![f64::NAN; columns]; rows];
    // Each station's sign: certified against r by the speed bound, sampled
    // only where that stays unsettled (near a true tangency).
    for i in 0..rows {
        for j in 0..columns {
            let q = tube.k_at(lattice.sigmas[i], lattice.psis[j]).ok_or("K is singular at a lattice station")?;
            let own = tube.own_time(lattice.sigmas[i]);
            // Inside the own window the certified own-time read (covered by a
            // witness, clear by a second-order bound, or a tie that declines);
            // outside it the speed bound, sampled only where unsettled.
            let tie = 1e-9 * (tube.major + tube.minor);
            let near = tube.own_read(q, own, window, tie);
            if matches!(near, super::fold::OwnRead::Tie) {
                return Err(format!("a station's own-time read ties at (σ {:.6}, ψ {:.6})", lattice.sigmas[i], lattice.psis[j]));
            }
            let depth = if let super::fold::OwnRead::Covered(_) = near {
                tube.minor * (1.0 - 1e-3)
            } else {
                match tube.beyond(q, own, window, tube.minor, 4096) {
                    Some(true) => tube.minor * (1.0 + 1e-3),
                    Some(false) => tube.minor * (1.0 - 1e-3),
                    None => tube.far_depth_sampled(q, own, window, 1024).0,
                }
            };
            let g = depth - tube.minor;
            if g.abs() <= tolerance {
                return Err(format!("a lattice station ties at (σ {:.6}, ψ {:.6})", lattice.sigmas[i], lattice.psis[j]));
            }
            values[i][j] = g;
        }
    }
    let covered = |i: usize, j: usize| values[i % rows][j % columns] < 0.0;
    // A crossing on the edge from node (i, j) along σ (along = 0) or ψ (1).
    let mut crossings: std::collections::HashMap<(usize, usize, u8), TrimPoint> = std::collections::HashMap::new();
    let mut solve = |i: usize, j: usize, along: u8| -> Result<TrimPoint, String> {
        if let Some(found) = crossings.get(&(i, j, along)) {
            return Ok(*found);
        }
        let (s0, p0) = (sigma_at(i), psi_at(j));
        let (s1, p1) = if along == 0 { (sigma_at(i + 1), p0) } else { (s0, psi_at(j + 1)) };
        let at = |f: f64| (s0 + (s1 - s0) * f, p0 + (p1 - p0) * f);
        let sign0 = values[i % rows][j % columns] < 0.0;
        // The competing pose, read once at the edge's covered end, then
        // followed locally along the edge (it moves continuously).
        let covered_end = if sign0 { 0.0 } else { 1.0 };
        let (cs, cp) = at(covered_end);
        let (_, guess, _) = tube.trim_value(cs, cp, window, tolerance).ok_or("K is singular on a lattice edge")?;
        let range = tube.to - tube.from;
        let near = |s: f64, p: f64| -> Option<(f64, f64, Vec3)> {
            let (s, p) = (s.rem_euclid(4.0), p.rem_euclid(std::f64::consts::TAU));
            let q = tube.k_at(s, p)?;
            let (depth, time) = tube.far_depth_near(q, tube.own_time(s), window, guess, 0.01 * range);
            Some((depth - tube.minor, time, q))
        };
        let (mut lo, mut hi) = (0.0f64, 1.0f64);
        for _ in 0..48 {
            let mid = 0.5 * (lo + hi);
            let (s, p) = at(mid);
            let (g, _, _) = near(s, p).ok_or("K is singular on a lattice edge")?;
            if (g < 0.0) == sign0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let (s, p) = at(0.5 * (lo + hi));
        let (_, time, point) = near(s, p).ok_or("K is singular on a lattice edge")?;
        let found = TrimPoint { sigma: s, psi: p, mate_time: time, point };
        crossings.insert((i, j, along), found);
        Ok(found)
    };
    // Segments per cell: the cell (i, j) has corners (i, j), (i+1, j), (i+1, j+1), (i, j+1);
    // its edges: bottom (i, j, ψ), right (i+1, j, ψ... along σ at j+1), etc. Keys name edges.
    type Key = (usize, usize, u8);
    let mut links: std::collections::HashMap<Key, Vec<Key>> = std::collections::HashMap::new();
    for i in 0..rows {
        for j in 0..columns {
            let corners = [covered(i, j), covered(i + 1, j), covered(i + 1, j + 1), covered(i, j + 1)];
            // edges: 0 = (i,j)->(i+1,j) along σ at ψ j; 1 = (i+1,j)->(i+1,j+1) along ψ; 2 = (i,j+1)->(i+1,j+1) along σ at ψ j+1; 3 = (i,j)->(i,j+1) along ψ
            let keys: [Key; 4] = [(i, j, 0), ((i + 1) % rows, j, 1), (i, (j + 1) % columns, 0), (i, j, 1)];
            let cut = [corners[0] != corners[1], corners[1] != corners[2], corners[3] != corners[2], corners[0] != corners[3]];
            let cuts: Vec<usize> = (0..4).filter(|&e| cut[e]).collect();
            match cuts.as_slice() {
                [] => {}
                [a, b] => {
                    links.entry(keys[*a]).or_default().push(keys[*b]);
                    links.entry(keys[*b]).or_default().push(keys[*a]);
                }
                _ => return Err(format!("a saddle cell at (σ {:.6}, ψ {:.6})", sigma_at(i), psi_at(j))),
            }
        }
    }
    // Walk the links into loops.
    let mut seen: std::collections::HashSet<Key> = std::collections::HashSet::new();
    let mut loops = Vec::new();
    let mut keys: Vec<Key> = links.keys().copied().collect();
    keys.sort();
    for start in keys {
        if seen.contains(&start) {
            continue;
        }
        let mut chain = vec![start];
        seen.insert(start);
        let mut previous = start;
        let mut current = links[&start][0];
        while current != start {
            if !seen.insert(current) {
                return Err("a trim curve branches".into());
            }
            chain.push(current);
            let next = links[&current].iter().copied().find(|k| *k != previous).ok_or("a trim curve ends")?;
            previous = current;
            current = next;
        }
        let mut points = Vec::with_capacity(chain.len());
        for (i, j, along) in chain {
            points.push(solve(i, j, along)?);
        }
        loops.push(points);
    }
    Ok(loops)
}

/// A point of K's domain with where it is in 3D (shared by every face that uses it).
#[derive(Clone, Copy, Debug)]
struct Node {
    sigma: f64,
    psi: f64,
    point: Vec3,
}

/// A perimeter point of a patch: a patch corner (row a, column b of the patch
/// grid, both periodic) or a split point of a trim curve (loop, index).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Mark {
    Corner(usize, usize),
    Split(usize, usize),
}

/// A trim curve piece as one patch sees it: its points in the patch's (σ, ψ)
/// (unwrapped into the patch's own ranges), oriented with the kept side on
/// the left, the edge it is (loop, piece) and whether it runs with that edge.
#[derive(Clone, Debug)]
struct Piece {
    marks: [Mark; 2],
    domain: Vec<(f64, f64)>,
    edge: (usize, usize),
    forward: bool,
}

/// The trimmed body: the primary loop of each mated pair is densified (three
/// exact contour points a lattice cell), split where it crosses a patch side
/// on its own side or its mate's, and each piece becomes one edge of the face
/// it cuts on each side; every patch is clipped by the pieces it holds and the
/// kept parts of its sides, a face per closed loop.
#[allow(clippy::too_many_arguments)]
pub fn trimmed_tube(
    tube: &Tube,
    chunks: [usize; 4],
    lattice: &Lattice,
    per_chunk: usize,
    per_quarter: usize,
    loops: &[Vec<TrimPoint>],
    window: f64,
    fit: f64,
) -> Result<crate::BrepSolid, String> {
    use std::f64::consts::{FRAC_PI_2, TAU};
    let tolerance = 1e-12 * tube.minor;
    // A trim edge is an intersection curve: held to the kernel's own fit for
    // those (`intersection_fit`), not the patches' tighter one (an edge's
    // deviation moves neither the volume nor the area).
    let edge_fit = crate::KernelTolerances::for_scale(tube.major + tube.minor, 1e-7).intersection_fit.max(fit);
    // Patch rows: (chart, s0, s1, σ0, σ1) round K.
    let mut rows: Vec<(usize, f64, f64)> = Vec::new();
    for (chart, &count) in chunks.iter().enumerate() {
        for c in 0..count {
            rows.push((chart, c as f64 / count as f64, (c + 1) as f64 / count as f64));
        }
    }
    let sigma_bounds: Vec<f64> = rows.iter().map(|(chart, s0, _)| *chart as f64 + s0).chain(std::iter::once(4.0)).collect();
    let row_of = |sigma: f64| -> usize {
        let s = sigma.rem_euclid(4.0);
        (sigma_bounds.partition_point(|b| *b <= s).max(1) - 1).min(rows.len() - 1)
    };
    let column_of = |psi: f64| -> usize { ((psi.rem_euclid(TAU) / FRAC_PI_2).floor() as usize).min(3) };
    let unwrap = |value: f64, near: f64, period: f64| value + ((near - value) / period).round() * period;
    let cell_sigma = |sigma: f64| {
        let a = row_of(sigma);
        (sigma_bounds[a + 1] - sigma_bounds[a]) / per_chunk as f64
    };
    let cell_psi = TAU / (4 * per_quarter) as f64;
    let g_at = |sigma: f64, psi: f64| -> Option<(f64, f64, Vec3)> { tube.trim_value(sigma.rem_euclid(4.0), psi.rem_euclid(TAU), window, tolerance) };
    // A point of the contour near the chord from a to b at fraction f: g = 0
    // sought across the chord within a cell either side.
    let range = tube.to - tube.from;
    let g_near = |sigma: f64, psi: f64, guess: f64| -> Option<(f64, f64, Vec3)> {
        let (sigma, psi) = (sigma.rem_euclid(4.0), psi.rem_euclid(TAU));
        let q = tube.k_at(sigma, psi)?;
        let (depth, time) = tube.far_depth_near(q, tube.own_time(sigma), window, guess, 0.01 * range);
        Some((depth - tube.minor, time, q))
    };
    let contour_within = |a: (f64, f64), b: (f64, f64), f: f64, guess: f64, cells: f64| -> Result<(f64, f64, f64, Vec3), String> {
        let c = (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f);
        let (ds, dp) = (cell_sigma(c.0), cell_psi);
        // the chord's normal in cell units
        let (cs, cp) = ((b.0 - a.0) / ds, (b.1 - a.1) / dp);
        let length = cs.hypot(cp);
        let (ns, np) = if length > 0.0 { (-cp / length, cs / length) } else { (1.0, 0.0) };
        let at = |lambda: f64| (c.0 + lambda * ns * ds, c.1 + lambda * np * dp);
        // The local read first (the competitor near its guess); where it finds
        // no crossing (its competitor left the span, or the own basin's edge
        // moved), the whole-range read.
        let search = |global: bool| -> Result<(f64, f64, f64, Vec3), String> {
        let read = |s: f64, p: f64| if global { g_at(s, p) } else { g_near(s, p, guess) };
        let sign = |lambda: f64| -> Result<bool, String> {
            let (s, p) = at(lambda);
            Ok(read(s, p).ok_or("K is singular near a trim curve")?.0 < 0.0)
        };
        let (mut lo, mut hi) = (-cells, cells);
        let (mut slo, shi) = (sign(lo)?, sign(hi)?);
        if slo == shi {
            // Near a lens tip (a cusp) the other branch lies within the
            // reach: the nearest change of sign from the chord's point, out
            // in doublings from 2^-20 of the reach, is the chord's own branch
            // (both its ends lie on it).
            let here = sign(0.0)?;
            let mut lambda = cells * 2f64.powi(-20);
            let mut found = None;
            while lambda <= cells {
                if sign(lambda)? != here {
                    found = Some((0.0, lambda));
                    break;
                }
                if sign(-lambda)? != here {
                    found = Some((-lambda, 0.0));
                    break;
                }
                lambda *= 2.0;
            }
            if let Some((a0, a1)) = found {
                (lo, hi, slo) = (a0, a1, sign(a0)?);
            }
        }
        if slo == sign(hi)? {
            return Err(format!("a trim curve leaves its cell (across {cells} cells) at (σ {:.6}, ψ {:.6}) on the chord from (σ {:.6}, ψ {:.6}) to (σ {:.6}, ψ {:.6}) at {f}", c.0, c.1, a.0, a.1, b.0, b.1));
        }
        for _ in 0..30 {
            let mid = 0.5 * (lo + hi);
            if sign(mid)? == slo {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let (s, p) = at(0.5 * (lo + hi));
        let (_, time, point) = read(s, p).ok_or("K is singular near a trim curve")?;
        Ok((s, p, time, point))
        };
        search(false).or_else(|_| search(true))
    };
    let contour = |a: (f64, f64), b: (f64, f64), f: f64, guess: f64| -> Result<(f64, f64, f64, Vec3), String> { contour_within(a, b, f, guess, 1.0) };

    // A point's mate, with the time it was read at: a stored time (a
    // lattice's least sample, a local read's guess) that is not the reaching
    // pose's is replaced by the exact far read from the point's own time.
    let mate_read = |point: Vec3, time: f64, sigma: f64| -> Option<((f64, f64), f64)> {
        if let Some(m) = tube.mate_of(point, time) {
            return Some((m, time));
        }
        let _ = tolerance;
        let (depth, exact) = tube.far_depth_sampled(point, tube.own_time(sigma.rem_euclid(4.0)), window, 1024);
        let found = tube.mate_of(point, exact).map(|m| (m, exact));
        found
    };
    // Pair the loops by their mates.
    let wrap = |a: f64, b: f64, period: f64| {
        let d = (a - b).rem_euclid(period);
        d.min(period - d)
    };
    let mut mate_loop = vec![usize::MAX; loops.len()];
    for (k, l) in loops.iter().enumerate() {
        let first = l[0];
        let ((ms, mp), _) = mate_read(first.point, first.mate_time, first.sigma).ok_or("a mate is singular [0]")?;
        let owner = loops
            .iter()
            .enumerate()
            .flat_map(|(o, other)| other.iter().map(move |x| (o, x)))
            .map(|(o, x)| (o, wrap(x.sigma, ms, 4.0).hypot(wrap(x.psi, mp, TAU))))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .ok_or("no loop")?
            .0;
        mate_loop[k] = owner;
    }
    for k in 0..loops.len() {
        if mate_loop[mate_loop[k]] != k {
            return Err("the trim loops do not pair".into());
        }
    }
    // The curves to build: each mated pair's first loop (closed), and each
    // loop mated onto itself (a fold's double curve: its two branches join at
    // the cusps, the fixed points of the mate map) cut there into one open
    // branch whose mate is the other.
    let domain_gap = |a: (f64, f64), b: (f64, f64)| wrap(a.0, b.0, 4.0).hypot(wrap(a.1, b.1, TAU));
    let mut curves: Vec<(Vec<TrimPoint>, bool)> = Vec::new();
    for (k, l) in loops.iter().enumerate() {
        if mate_loop[k] > k {
            curves.push((l.clone(), false));
            continue;
        }
        if mate_loop[k] < k {
            continue;
        }
        let n = l.len();
        let gaps: Vec<f64> = l.iter().map(|p| mate_read(p.point, p.mate_time, p.sigma).map(|(m, _)| domain_gap(m, (p.sigma, p.psi))).unwrap_or(f64::INFINITY)).collect();
        let a = (0..n).min_by(|x, y| gaps[*x].total_cmp(&gaps[*y])).ok_or("an empty loop")?;
        let far = |i: usize| { let d = (i + n - a) % n; d.min(n - d) };
        let b = (0..n).filter(|i| far(*i) >= n / 4).min_by(|x, y| gaps[*x].total_cmp(&gaps[*y])).ok_or("a self-mated loop has one fixed point")?;
        // Each fixed point (a cusp, where the covered lens ends in a tip) found
        // by walking the contour from its nearest station: the tangent (the
        // gradient of g turned a quarter, in cell units) predicts a step, a
        // search along the normal brings it back onto g = 0, and the walk
        // follows whichever way the mate gap falls, halving its step when the
        // gap stops falling, until the step is 1e-9 of a cell.
        let walk = |i: usize| -> Result<TrimPoint, String> {
            let p0 = l[i];
            let guess = p0.mate_time;
            let g = |s: f64, p: f64| -> Result<f64, String> {
                g_near(s, p, guess).map(|x| x.0).ok_or_else(|| format!("K is singular near a cusp at (σ {s:.9}, ψ {p:.9}); start (σ {:.9}, ψ {:.9})", p0.sigma, p0.psi))
            };
            let (ds, dp) = (cell_sigma(p0.sigma), cell_psi);
            let gap_of = |s: f64, p: f64| -> Result<(f64, TrimPoint), String> {
                let (_, time, point) = g_near(s, p, guess).ok_or("K is singular near a cusp")?;
                // no mate (a corrected point whose least is off the guess): a gap that does not fall
                let gap = mate_read(point, time, s).map(|(m, _)| domain_gap(m, (s, p))).unwrap_or(f64::INFINITY);
                Ok((gap, TrimPoint { sigma: s.rem_euclid(4.0), psi: p.rem_euclid(TAU), mate_time: time, point }))
            };
            // back onto g = 0 along the gradient's direction, within `reach` cells
            let correct = |s: f64, p: f64, gs: f64, gp: f64, reach: f64| -> Result<Option<(f64, f64)>, String> {
                let length = gs.hypot(gp);
                if !(length.is_finite() && length > 0.0) || !(s.is_finite() && p.is_finite()) {
                    return Ok(None);
                }
                let (us, up) = (gs / length, gp / length);
                let at = |lambda: f64| (s + lambda * us * ds, p + lambda * up * dp);
                let (a0, a1) = (at(-reach), at(reach));
                let (f0, f1) = (g(a0.0, a0.1)?, g(a1.0, a1.1)?);
                if (f0 < 0.0) == (f1 < 0.0) {
                    return Ok(None);
                }
                let (mut lo, mut hi) = (-reach, reach);
                for _ in 0..40 {
                    let mid = 0.5 * (lo + hi);
                    let (x, y) = at(mid);
                    if (g(x, y)? < 0.0) == (f0 < 0.0) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                Ok(Some(at(0.5 * (lo + hi))))
            };
            let gradient = |s: f64, p: f64, h: f64| -> Result<(f64, f64), String> {
                let gs = (g(s + h * ds, p)? - g(s - h * ds, p)?) / (2.0 * h);
                let gp = (g(s, p + h * dp)? - g(s, p - h * dp)?) / (2.0 * h);
                Ok((gs, gp))
            };
            let (mut s0, mut q0) = (p0.sigma, p0.psi);
            let (mut best, mut best_point) = gap_of(s0, q0)?;
            let mut step: f64 = 0.25;
            let mut direction = 0.0f64;
            while step > 1e-9 {
                let (gs, gp) = gradient(s0, q0, (0.1 * step).max(1e-7))?;
                let (ts, tp) = (-gp, gs);
                let length = ts.hypot(tp);
                // a flat or undefined gradient (a sliver at the station): no direction to walk
                if !(length.is_finite() && length > 0.0) {
                    break;
                }
                let mut moved = false;
                let order: [f64; 2] = if direction < 0.0 { [-1.0, 1.0] } else { [1.0, -1.0] };
                for sign in order {
                    let (ps, pp) = (s0 + sign * step * ts / length * ds, q0 + sign * step * tp / length * dp);
                    let Some((cs, cp)) = correct(ps, pp, gs, gp, (2.0 * step).max(1e-6))? else { continue };
                    let (gap, point) = gap_of(cs, cp)?;
                    if gap < best {
                        best = gap;
                        best_point = point;
                        s0 = cs;
                        q0 = cp;
                        direction = sign;
                        moved = true;
                        break;
                    }
                }
                if !moved {
                    step *= 0.5;
                }
            }
            Ok(best_point)
        };
        let refine = |i: usize| walk(i);
        let (start, end) = (a.min(b), a.max(b));
        let mut branch: Vec<TrimPoint> = l[start..=end].to_vec();
        branch[0] = refine(start)?;
        let last = branch.len() - 1;
        branch[last] = refine(end)?;
        curves.push((branch, true));
    }

    // Each primary loop: dense exact points with mates, split marks, pieces.
    struct Dense {
        own: Vec<(f64, f64)>,
        mate: Vec<(f64, f64)>,
        point: Vec<Vec3>,
        split: Vec<bool>,
        time: Vec<f64>,
    }
    let mut dense_loops: Vec<(usize, Dense, bool)> = Vec::new();
    for (k, (l, open)) in curves.iter().enumerate() {
        let open = *open;
        let mut dense = Dense { own: Vec::new(), mate: Vec::new(), point: Vec::new(), split: Vec::new(), time: Vec::new() };
        let n = l.len();
        let mut previous: Option<(f64, f64)> = None;
        for i in 0..n {
            if open && i + 1 == n {
                // the open branch's last point (a cusp): a split, its own mate
                let a = l[i];
                let pa = match previous {
                    Some(p) => (unwrap(a.sigma, p.0, 4.0), unwrap(a.psi, p.1, TAU)),
                    None => (a.sigma, a.psi),
                };
                dense.own.push(pa);
                dense.point.push(a.point);
                dense.mate.push(pa);
                dense.split.push(true);
                dense.time.push(a.mate_time);
                break;
            }
            let (a, b) = (l[i], l[(i + 1) % n]);
            // a crossing found twice (on a lattice node, by both its edges): one point
            if domain_gap((a.sigma, a.psi), (b.sigma, b.psi)) <= 1e-12 && !(open && i + 2 == n) && !(open && i == 0) {
                continue;
            }
            let pa = match previous {
                Some(p) => (unwrap(a.sigma, p.0, 4.0), unwrap(a.psi, p.1, TAU)),
                None => (a.sigma, a.psi),
            };
            let pb = (unwrap(b.sigma, pa.0, 4.0), unwrap(b.psi, pa.1, TAU));
            // the lattice crossing itself: split where it sits on a patch side
            let on_sigma_side = sigma_bounds.iter().any(|s| wrap(*s, a.sigma, 4.0) <= 1e-12);
            let on_psi_side = (0..4).any(|q| wrap(q as f64 * FRAC_PI_2, a.psi, TAU) <= 1e-12);
            dense.own.push(pa);
            dense.point.push(a.point);
            // an open branch's first point is a cusp: its own mate
            let (mate, time) = if open && i == 0 { (pa, a.mate_time) } else { mate_read(a.point, a.mate_time, a.sigma).ok_or("a mate is singular [1]")? };
            dense.mate.push(mate);
            dense.split.push(on_sigma_side || on_psi_side || (open && i == 0));
            dense.time.push(time);
            for f in [0.25, 0.5, 0.75] {
                let (s, p, time, point) = contour(pa, pb, f, time).map_err(|e| format!("{e} [densify]"))?;
                dense.own.push((unwrap(s, pa.0, 4.0), unwrap(p, pa.1, TAU)));
                dense.point.push(point);
                let (mate, time) = mate_read(point, time, s).ok_or("a mate is singular [2]")?;
                dense.mate.push(mate);
                dense.split.push(false);
                dense.time.push(time);
            }
            previous = Some(pa);
        }
        // Unwrap the mates along the loop, then split where a mate crosses a patch side.
        for i in 1..dense.mate.len() {
            let before = dense.mate[i - 1];
            dense.mate[i] = (unwrap(dense.mate[i].0, before.0, 4.0), unwrap(dense.mate[i].1, before.1, TAU));
        }
        let mut i = 0;
        while i < dense.own.len() {
            if open && i + 1 >= dense.own.len() {
                break;
            }
            let j = (i + 1) % dense.own.len();
            let (ma, mb) = (dense.mate[i], dense.mate[j]);
            let mb = (unwrap(mb.0, ma.0, 4.0), unwrap(mb.1, ma.1, TAU));
            let crossing = |x: f64, y: f64, bounds: &[f64], period: f64| -> Option<f64> {
                bounds.iter().flat_map(|b| (-2..=2).map(move |w| b + w as f64 * period)).find(|b| (x - b) * (y - b) < 0.0 && (x - b).abs() > 1e-12 && (y - b).abs() > 1e-12)
            };
            let psi_bounds: Vec<f64> = (0..4).map(|q| q as f64 * FRAC_PI_2).collect();
            let found = crossing(ma.0, mb.0, &sigma_bounds, 4.0).map(|b| (0, b)).or_else(|| crossing(ma.1, mb.1, &psi_bounds, TAU).map(|b| (1, b)));
            if let Some((axis, bound)) = found {
                let (oa, ob) = (dense.own[i], dense.own[j]);
                let ob = (unwrap(ob.0, oa.0, 4.0), unwrap(ob.1, oa.1, TAU));
                let guess = dense.time[i];
                let mate_coord = |f: f64| -> Result<(f64, (f64, f64), (f64, f64), Vec3), String> {
                    let (s, p, time, point) = contour(oa, ob, f, guess).map_err(|e| format!("{e} [mate-split]"))?;
                    let (m, _) = mate_read(point, time, s).ok_or("a mate is singular [3]")?;
                    let m = (unwrap(m.0, ma.0, 4.0), unwrap(m.1, ma.1, TAU));
                    Ok((if axis == 0 { m.0 } else { m.1 } - bound, (unwrap(s, oa.0, 4.0), unwrap(p, oa.1, TAU)), m, point))
                };
                let (mut lo, mut hi) = (0.0f64, 1.0f64);
                let start = if axis == 0 { ma.0 } else { ma.1 } - bound;
                for _ in 0..50 {
                    let mid = 0.5 * (lo + hi);
                    if (mate_coord(mid)?.0 < 0.0) == (start < 0.0) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                let (_, own, mut mate, point) = mate_coord(0.5 * (lo + hi))?;
                if axis == 0 {
                    mate.0 = bound;
                } else {
                    mate.1 = bound;
                }
                dense.own.insert(i + 1, own);
                dense.mate.insert(i + 1, mate);
                dense.point.insert(i + 1, point);
                dense.split.insert(i + 1, true);
                dense.time.insert(i + 1, guess);
            }
            i += 1;
        }
        if !dense.split.iter().any(|s| *s) {
            return Err("a trim loop never crosses a patch side".into());
        }
        dense_loops.push((k, dense, open));
    }
    let _ = (fit, lattice);
    // Pieces between split points; each lies in one own patch and one mate patch.
    struct Raw {
        own_patch: (usize, usize),
        mate_patch: (usize, usize),
        own: Vec<(f64, f64)>,
        mate: Vec<(f64, f64)>,
        points: Vec<Vec3>,
        times: Vec<f64>,
        marks: [Mark; 2],
    }
    let mut raws: Vec<Raw> = Vec::new();
    #[allow(unused_mut)]
    for (index, (_, dense, open)) in dense_loops.iter().enumerate() {
        let n = dense.own.len();
        let first = dense.split.iter().position(|s| *s).unwrap();
        let mut start = first;
        loop {
            if *open && start + 1 >= n {
                break; // an open branch ends at its last cusp
            }
            let mut end = (start + 1) % n;
            let mut own = vec![dense.own[start]];
            let mut mate = vec![dense.mate[start]];
            let mut points = vec![dense.point[start]];
            let mut times = vec![dense.time[start]];
            loop {
                let last = *own.last().unwrap();
                let o = dense.own[end];
                own.push((unwrap(o.0, last.0, 4.0), unwrap(o.1, last.1, TAU)));
                let lm = *mate.last().unwrap();
                let m = dense.mate[end];
                mate.push((unwrap(m.0, lm.0, 4.0), unwrap(m.1, lm.1, TAU)));
                points.push(dense.point[end]);
                times.push(dense.time[end]);
                if dense.split[end] {
                    break;
                }
                end = (end + 1) % n;
            }
            let middle = own.len() / 2;
            let (om, mm) = (own[middle], mate[middle]);
            let (om, mm) = if own.len() == 2 { ((0.5 * (own[0].0 + own[1].0), 0.5 * (own[0].1 + own[1].1)), (0.5 * (mate[0].0 + mate[1].0), 0.5 * (mate[0].1 + mate[1].1))) } else { (om, mm) };
            raws.push(Raw {
                own_patch: (row_of(om.0), column_of(om.1)),
                mate_patch: (row_of(mm.0), column_of(mm.1)),
                own,
                mate,
                points,
                times,
                marks: [Mark::Split(index, start), Mark::Split(index, end)],
            });
            start = end;
            if start == first || (*open && start + 1 >= n) {
                break;
            }
        }
    }
    // Each piece refined until its curve passes within `fit` of the exact
    // contour: a gap that misses gets its exact midpoint (with its mate).
    for raw in raws.iter_mut() {
        // good[k]: gap k already within the fit (its stations unchanged since).
        let mut good = vec![false; raw.points.len() - 1];
        let mut settled = false;
        for _round in 0..32 {
            let mut params = vec![0.0];
            for pair in raw.points.windows(2) {
                params.push(params.last().unwrap() + pair[1].sub(pair[0]).length().max(1e-300));
            }
            let total = *params.last().unwrap();
            let params: Vec<f64> = params.iter().map(|t| t / total).collect();
            let curve = crate::interpolate_curve(&raw.points, 3.min(raw.points.len() - 1), &params)?;
            let mut inserts: Vec<(usize, (f64, f64), (f64, f64), f64, Vec3)> = Vec::new();
            for (k, fraction) in (0..raw.points.len() - 1).filter(|k| !good[*k]).flat_map(|k| [0.25, 0.5, 0.75].map(|f| (k, f))) {
                // a quarter point past a lens tip's reach (the contour turns
                // within the gap): the middle is read instead
                let (s_mid, p_mid, time, target) = match contour(raw.own[k], raw.own[k + 1], fraction, raw.times[k]) {
                    Ok(found) => found,
                    Err(_) if fraction != 0.5 => continue,
                    Err(e) => return Err(format!("{e} [refine]")),
                };
                let (lo, hi) = (params[k], params[k + 1]);
                let distance = |t: f64| curve.evaluate(t).map(|p| p.sub(target).length()).unwrap_or(f64::INFINITY);
                let mut best = (f64::INFINITY, lo);
                for j in 0..=32 {
                    let t = lo + (hi - lo) * j as f64 / 32.0;
                    let d = distance(t);
                    if d < best.0 {
                        best = (d, t);
                    }
                }
                let step = (hi - lo) / 32.0;
                let (mut a, mut b) = ((best.1 - step).max(lo), (best.1 + step).min(hi));
                let ratio = 0.5 * (5f64.sqrt() - 1.0);
                for _ in 0..60 {
                    let (c, d) = (b - ratio * (b - a), a + ratio * (b - a));
                    if distance(c) < distance(d) {
                        b = d;
                    } else {
                        a = c;
                    }
                }
                let miss_here = distance(0.5 * (a + b)).min(best.0);
                // A miss at the gap's quarter or middle point: the gap gets its
                // exact middle (once), which halves it.
                if miss_here > edge_fit && fraction == 0.5 || miss_here > edge_fit && !inserts.iter().any(|i| i.0 == k + 1) {
                    let (s_mid, p_mid, time, target) = if fraction == 0.5 { (s_mid, p_mid, time, target) } else { contour(raw.own[k], raw.own[k + 1], 0.5, raw.times[k])? };
                    let own = (unwrap(s_mid, raw.own[k].0, 4.0), unwrap(p_mid, raw.own[k].1, TAU));
                    let (m, _) = mate_read(target, time, s_mid).ok_or("a mate is singular [4]")?;
                    let mate = (unwrap(m.0, raw.mate[k].0, 4.0), unwrap(m.1, raw.mate[k].1, TAU));
                    if !inserts.iter().any(|i| i.0 == k + 1) {
                        inserts.push((k + 1, own, mate, time, target));
                    }
                }
            }
            if inserts.is_empty() {
                // Settled only by a round that read every gap: the interpolant
                // is global, so a point added elsewhere moves the whole curve.
                if good.iter().all(|g| !g) {
                    settled = true;
                    break;
                }
                good.iter_mut().for_each(|g| *g = false);
                continue;
            }
            // Every gap not split this round passed; a split gap's two halves are read next round.
            let split: std::collections::HashSet<usize> = inserts.iter().map(|i| i.0 - 1).collect();
            let mut next_good = Vec::with_capacity(good.len() + inserts.len());
            for k in 0..good.len() {
                if split.contains(&k) {
                    next_good.push(false);
                    next_good.push(false);
                } else {
                    next_good.push(true);
                }
            }
            good = next_good;
            for (at, own, mate, time, point) in inserts.into_iter().rev() {
                raw.own.insert(at, own);
                raw.mate.insert(at, mate);
                raw.times.insert(at, time);
                raw.points.insert(at, point);
            }
        }
        if !settled {
            return Err(format!("a trim edge does not settle within the fit {edge_fit:.1e} in 32 rounds"));
        }
    }
    let columns = 4usize;
    let patch_box = |a: usize, b: usize| -> ([f64; 2], [f64; 2]) { ([sigma_bounds[a], sigma_bounds[a + 1]], [b as f64 * FRAC_PI_2, (b + 1) as f64 * FRAC_PI_2]) };
    // A domain polyline moved into patch (a, b)'s own ranges.
    let localise = |line: &[(f64, f64)], a: usize, b: usize| -> Vec<(f64, f64)> {
        let (su, pv) = patch_box(a, b);
        let m = line[line.len() / 2];
        let (ds, dp) = (((0.5 * (su[0] + su[1]) - m.0) / 4.0).round() * 4.0, ((0.5 * (pv[0] + pv[1]) - m.1) / TAU).round() * TAU);
        line.iter().map(|(x, y)| (x + ds, y + dp)).collect()
    };
    // Orient a polyline so the kept side (g ≥ 0) is on its left.
    let kept_left = |line: &[(f64, f64)]| -> Result<bool, String> {
        let k = (line.len() / 2).max(1).min(line.len() - 1);
        let (a, b) = (line[k - 1], line[k]);
        let (ds, dp) = (cell_sigma(a.0), cell_psi);
        let (cs, cp) = ((b.0 - a.0) / ds, (b.1 - a.1) / dp);
        let length = cs.hypot(cp);
        if !(length > 0.0) {
            return Err("a trim piece has no length".into());
        }
        let m = (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1));
        let left = (m.0 - 0.05 * cp / length * ds, m.1 + 0.05 * cs / length * dp);
        Ok(g_at(left.0, left.1).ok_or("K is singular near a trim curve")?.0 >= 0.0)
    };
    let mut held: std::collections::HashMap<(usize, usize), Vec<Piece>> = std::collections::HashMap::new();
    for (e, raw) in raws.iter().enumerate() {
        for (patch, line) in [(raw.own_patch, &raw.own), (raw.mate_patch, &raw.mate)] {
            let mut domain = localise(line, patch.0, patch.1);
            let mut marks = raw.marks;
            let forward = kept_left(&domain)?;
            if !forward {
                domain.reverse();
                marks.swap(0, 1);
            }
            held.entry(patch).or_default().push(Piece { marks, domain, edge: (0, e), forward });
        }
    }
    // Topology.
    use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
    let mut next = 1u64;
    let mut id = || {
        next += 1;
        next - 1
    };
    let mut vertices: Vec<VertexRecord> = Vec::new();
    let mut vertex_ids: std::collections::HashMap<Mark, u64> = std::collections::HashMap::new();
    let mut vertex = |mark: Mark, point: Vec3, id: &mut dyn FnMut() -> u64| -> u64 {
        *vertex_ids.entry(mark).or_insert_with(|| {
            let vid = id();
            vertices.push(VertexRecord { id: vid, point });
            vid
        })
    };
    let mark_point = |mark: Mark| -> Result<Vec3, String> {
        match mark {
            Mark::Corner(a, b) => tube.k_at(sigma_bounds[a % rows.len()], (b % columns) as f64 * FRAC_PI_2).ok_or_else(|| "K is singular at a corner".to_string()),
            Mark::Split(l, i) => Ok(dense_loops[l].1.point[i]),
        }
    };
    let curve_through = |points: &[Vec3]| -> Result<crate::NurbsCurve, String> {
        let mut params = vec![0.0];
        for pair in points.windows(2) {
            params.push(params.last().unwrap() + pair[1].sub(pair[0]).length().max(1e-300));
        }
        let total = *params.last().unwrap();
        let params: Vec<f64> = params.iter().map(|t| t / total).collect();
        crate::interpolate_curve(points, 3.min(points.len() - 1), &params)
    };
    let mut edges: Vec<EdgeRecord> = Vec::new();
    let mut piece_edges: std::collections::HashMap<usize, u64> = std::collections::HashMap::new();
    let mut side_edges: std::collections::HashMap<(u8, usize, usize, Mark, Mark), (u64, Mark)> = std::collections::HashMap::new();
    let mut faces: Vec<FaceRecord> = Vec::new();
    let mut face_charts: Vec<usize> = Vec::new();
    let mut fits: std::collections::HashMap<(usize, usize), crate::NurbsSurface> = std::collections::HashMap::new();
    let mut orient_by: Option<(usize, [f64; 2], [f64; 2])> = None;
    for a in 0..rows.len() {
        for b in 0..columns {
            let (su, pv) = patch_box(a, b);
            let pieces = held.remove(&(a, b)).unwrap_or_default();
            let whole = pieces.is_empty();
            if pieces.is_empty() {
                let (g, _, _) = g_at(0.5 * (su[0] + su[1]), 0.5 * (pv[0] + pv[1])).ok_or("K is singular")?;
                if g < 0.0 {
                    continue; // covered whole
                }
            }
            let chart = rows[a].0;
            let surface = match fits.get(&(a, b)) {
                Some(found) => found.clone(),
                None => {
                    let (surface, ..) = super::fit_patch(tube, chart, [rows[a].1, rows[a].2], pv, fit).map_err(|e| e.message)?;
                    fits.insert((a, b), surface.clone());
                    surface
                }
            };
            // Perimeter positions in [0, 4): bottom (ψ = p0) 0..1, right (σ = s1) 1..2, top 2..3, left 3..4.
            let position = |(x, y): (f64, f64)| -> Result<f64, String> {
                let (fx, fy) = ((x - su[0]) / (su[1] - su[0]), (y - pv[0]) / (pv[1] - pv[0]));
                let near = |v: f64, t: f64| (v - t).abs() <= 1e-7;
                Ok(if near(fy, 0.0) {
                    fx.clamp(0.0, 1.0)
                } else if near(fx, 1.0) {
                    1.0 + fy.clamp(0.0, 1.0)
                } else if near(fy, 1.0) {
                    2.0 + (1.0 - fx).clamp(0.0, 1.0)
                } else if near(fx, 0.0) {
                    3.0 + (1.0 - fy).clamp(0.0, 1.0)
                } else {
                    return Err(format!("a trim piece ends inside patch ({a}, {b}) at ({fx:.6}, {fy:.6})"));
                })
            };
            let at_position = |t: f64| -> (f64, f64) {
                let f = t - t.floor();
                match t.floor() as i64 {
                    0 => (su[0] + (su[1] - su[0]) * f, pv[0]),
                    1 => (su[1], pv[0] + (pv[1] - pv[0]) * f),
                    2 => (su[1] - (su[1] - su[0]) * f, pv[1]),
                    _ => (su[0], pv[1] - (pv[1] - pv[0]) * f),
                }
            };
            // Chain the pieces end to end where they meet inside the patch
            // (a split made on the mate's side): only a chain's ends lie on
            // the perimeter.
            let mut chains: Vec<Vec<usize>> = Vec::new();
            {
                let starts: std::collections::HashMap<Mark, usize> = pieces.iter().enumerate().map(|(k, p)| (p.marks[0], k)).collect();
                let ends: std::collections::HashSet<Mark> = pieces.iter().map(|p| p.marks[1]).collect();
                let mut taken = vec![false; pieces.len()];
                for k in 0..pieces.len() {
                    if taken[k] || (ends.contains(&pieces[k].marks[0]) && position(pieces[k].domain[0]).is_err()) {
                        continue;
                    }
                    let mut chain = vec![k];
                    taken[k] = true;
                    let mut tail = pieces[k].marks[1];
                    while position(*pieces[*chain.last().unwrap()].domain.last().unwrap()).is_err() {
                        let next = *starts.get(&tail).ok_or_else(|| format!("a trim run stops inside patch ({a}, {b})"))?;
                        if taken[next] {
                            return Err(format!("a trim run closes inside patch ({a}, {b})"));
                        }
                        taken[next] = true;
                        chain.push(next);
                        tail = pieces[next].marks[1];
                    }
                    chains.push(chain);
                }
                if taken.iter().any(|t| !t) {
                    return Err(format!("a trim run of patch ({a}, {b}) never reaches its sides"));
                }
            }
            let corners = [Mark::Corner(a, b), Mark::Corner((a + 1) % rows.len(), b), Mark::Corner((a + 1) % rows.len(), (b + 1) % columns), Mark::Corner(a, (b + 1) % columns)];
            let mut stops: Vec<(f64, Mark)> = (0..4).map(|k| (k as f64, corners[k])).collect();
            for chain in &chains {
                let (first, last) = (&pieces[chain[0]], &pieces[*chain.last().unwrap()]);
                stops.push((position(first.domain[0])?, first.marks[0]));
                stops.push((position(*last.domain.last().unwrap())?, last.marks[1]));
            }
            stops.sort_by(|x, y| x.0.total_cmp(&y.0));
            stops.dedup_by(|x, y| (x.0 - y.0).abs() <= 1e-9 && x.1 == y.1);
            // Oriented segments: (from, to, kind). kind 0: a piece (index), 1: a side run (t0, t1).
            #[derive(Clone, Copy)]
            enum Run {
                Piece(usize),
                Side(f64, f64),
            }
            let mut segments: Vec<(Mark, Mark, Run)> = pieces.iter().enumerate().map(|(k, p)| (p.marks[0], p.marks[1], Run::Piece(k))).collect();
            let _ = &chains;
            for k in 0..stops.len() {
                let (t0, m0) = stops[k];
                let (mut t1, m1) = stops[(k + 1) % stops.len()];
                if t1 <= t0 {
                    t1 += 4.0;
                }
                if t1 - t0 <= 1e-12 {
                    continue;
                }
                let middle = at_position((0.5 * (t0 + t1)).rem_euclid(4.0));
                if g_at(middle.0, middle.1).ok_or("K is singular")?.0 >= 0.0 {
                    segments.push((m0, m1, Run::Side(t0, t1)));
                }
            }
            // Chain into loops.
            let mut used = vec![false; segments.len()];
            for start in 0..segments.len() {
                if used[start] {
                    continue;
                }
                let mut chain = vec![start];
                used[start] = true;
                let first = segments[start].0;
                let mut tail = segments[start].1;
                while tail != first {
                    let next = (0..segments.len()).find(|&k| !used[k] && segments[k].0 == tail).ok_or_else(|| format!("a face loop of patch ({a}, {b}) does not close"))?;
                    used[next] = true;
                    chain.push(next);
                    tail = segments[next].1;
                }
                // Coedges.
                let mut coedges = Vec::new();
                for &k in &chain {
                    let (m0, m1, run) = segments[k];
                    let v0 = vertex(m0, mark_point(m0)?, &mut id);
                    let v1 = vertex(m1, mark_point(m1)?, &mut id);
                    match run {
                        Run::Piece(index) => {
                            let piece = &pieces[index];
                            let e = piece.edge.1;
                            let edge_id = match piece_edges.get(&e) {
                                Some(found) => *found,
                                None => {
                                    let raw = &raws[e];
                                    let curve = curve_through(&raw.points)?;
                                    // Within the fit of the contour: its refinement above ended on a round
                                    // with every gap's quarter points and middle inside it.
                                    let [t0, t1] = curve.domain()?;
                                    let eid = id();
                                    let (s0, s1) = (vertex(raw.marks[0], mark_point(raw.marks[0])?, &mut id), vertex(raw.marks[1], mark_point(raw.marks[1])?, &mut id));
                                    edges.push(EdgeRecord { id: eid, curve, t0, t1, start_vertex_id: s0, end_vertex_id: s1, degenerate: false, name: None });
                                    piece_edges.insert(e, eid);
                                    eid
                                }
                            };
                            let chart = rows[a].0 as f64;
                            let uv: Vec<Vec3> = piece.domain.iter().map(|(x, y)| Vec3::new(x - chart, *y, 0.0)).collect();
                            let pcurve = curve_through(&uv)?;
                            coedges.push(CoedgeRecord { id: id(), edge_id, forward: piece.forward, pcurve });
                        }
                        Run::Side(t0, t1) => {
                            let side = t0.floor() as i64;
                            let (pa, pb) = (at_position(t0.rem_euclid(4.0)), at_position(if t1 - t1.floor() == 0.0 && t1.floor() as i64 == side + 1 { t1 - 1e-15 } else { t1.rem_euclid(4.0) }));
                            let pb = if t1 >= (side + 1) as f64 - 1e-15 { at_position(((side + 1) as f64 - 1e-15).rem_euclid(4.0)) } else { pb };
                            let pb = if t1 >= (side + 1) as f64 - 1e-15 { match side { 0 => (su[1], pv[0]), 1 => (su[1], pv[1]), 2 => (su[0], pv[1]), _ => (su[0], pv[0]) } } else { pb };
                            // the side line, named by the patch pair it parts
                            let key_line: (u8, usize, usize) = match side {
                                0 => (1, a, b),
                                1 => (0, (a + 1) % rows.len(), b),
                                2 => (1, a, (b + 1) % columns),
                                _ => (0, a, b),
                            };
                            let (lo, hi) = if m0 <= m1 { (m0, m1) } else { (m1, m0) };
                            let key = (key_line.0, key_line.1, key_line.2, lo, hi);
                            let (edge_id, edge_start) = match side_edges.get(&key) {
                                Some(found) => *found,
                                None => {
                                    // The face's own iso-curve along the side, cut to the run
                                    // (the neighbour across the side has the same one: both
                                    // patches interpolate the same boundary stations).
                                    let local = rows[a].0 as f64;
                                    let (iso, from_t, to_t) = match side {
                                        0 | 2 => (surface.iso_curve_v(pa.1)?, pa.0 - local, pb.0 - local),
                                        _ => (surface.iso_curve_u(pa.0 - local)?, pa.1, pb.1),
                                    };
                                    let (lo_t, hi_t) = (from_t.min(to_t), from_t.max(to_t));
                                    let [d0, d1] = iso.domain()?;
                                    let mut piece_curve = iso;
                                    // `split` refuses a cut within 1e-9 of an end: there the
                                    // whole curve stands (its end within 1e-9 of the run's).
                                    if lo_t > d0 + 1e-9 {
                                        piece_curve = piece_curve.split(lo_t)?.1;
                                    }
                                    let [_, e1] = piece_curve.domain()?;
                                    if hi_t < e1 - 1e-9 {
                                        piece_curve = piece_curve.split(hi_t)?.0;
                                    }
                                    let curve = if from_t > to_t { piece_curve.reversed()? } else { piece_curve };
                                    let [c0, c1] = curve.domain()?;
                                    let eid = id();
                                    edges.push(EdgeRecord { id: eid, curve, t0: c0, t1: c1, start_vertex_id: v0, end_vertex_id: v1, degenerate: false, name: None });
                                    side_edges.insert(key, (eid, m0));
                                    (eid, m0)
                                }
                            };
                            let chart = rows[a].0 as f64;
                            let pcurve = crate::make_line(Vec3::new(pa.0 - chart, pa.1, 0.0), Vec3::new(pb.0 - chart, pb.1, 0.0))?;
                            coedges.push(CoedgeRecord { id: id(), edge_id, forward: edge_start == m0, pcurve });
                        }
                    }
                }
                let surface = surface.clone();
                if whole && chart == 0 && orient_by.is_none() {
                    orient_by = Some((faces.len(), [rows[a].1, rows[a].2], pv));
                }
                faces.push(FaceRecord { id: id(), surface, same_sense: true, loops: vec![LoopRecord { id: id(), coedges }], name: None });
                face_charts.push(chart);
            }
        }
    }
    // Shells by connectivity through edges.
    let mut owner: std::collections::BTreeMap<u64, Vec<usize>> = std::collections::BTreeMap::new();
    for (f, face) in faces.iter().enumerate() {
        for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
            owner.entry(coedge.edge_id).or_default().push(f);
        }
    }
    let mut group: Vec<usize> = (0..faces.len()).collect();
    fn find(group: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while group[r] != r {
            r = group[r];
        }
        group[x] = r;
        r
    }
    for users in owner.values() {
        for pair in users.windows(2) {
            let (x, y) = (find(&mut group, pair[0]), find(&mut group, pair[1]));
            group[x] = y;
        }
    }
    let mut shells: std::collections::BTreeMap<usize, Vec<FaceRecord>> = std::collections::BTreeMap::new();
    for (f, face) in faces.into_iter().enumerate() {
        let root = find(&mut group, f);
        shells.entry(root).or_default().push(face);
    }
    let shells: Vec<ShellRecord> = shells.into_values().map(|faces| ShellRecord { id: id(), faces }).collect();
    let genus = shells.len() as i64;
    let _ = face_charts;
    let mut solid = crate::BrepSolid { mass_properties_cache: Default::default(), id: id(), vertices, edges, shells, genus, kind: Default::default(), cells: Vec::new() };
    // Outward on the sheet Σ + r·N is +N: a whole chart-0 face's normal at its
    // middle says whether the faces turn out.
    let (_, su, pv) = orient_by.ok_or("no whole face of the sheet Σ + r·N to orient by")?;
    let (um, vm) = (0.5 * (su[0] + su[1]), 0.5 * (pv[0] + pv[1]));
    let surface = &fits[&(row_of(su[0] + 1e-12), column_of(vm))];
    let normal = surface.normal(um, vm)?;
    let outward = tube.frame(tube.from + (tube.to - tube.from) * um, vm).ok_or("the core circle's sweep is singular")?.normal;
    let along = normal.dot(outward);
    if along < 0.0 {
        crate::orientation::flip_all_faces(&mut solid).map_err(|e| e.message)?;
    } else if !(along > 0.0) {
        return Err("the tube's normal is undecided".into());
    }
    Ok(solid)
}
