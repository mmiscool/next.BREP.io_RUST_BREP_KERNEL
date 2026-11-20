//! A parameterization-independent displacement certificate. Positive rational
//! control hulls bound each curve by a polyline; convex segment distance bounds
//! the two directed polyline distances, including their interiors.
use crate::{NurbsCurve, Vec3};

fn distance(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let d = b.sub(a);
    let t = if d.length_squared() == 0.0 {
        0.0
    } else {
        p.sub(a).dot(d) / d.length_squared()
    }
    .clamp(0.0, 1.0);
    p.sub(a.add(d.scale(t))).length()
}
fn polyline(c: NurbsCurve, flatness: f64, depth: usize, out: &mut Vec<Vec3>) -> Result<(), String> {
    if out.len() >= 4096 {
        return Err("analytic displacement certificate exhausted subdivision budget".into());
    }
    let [lo, hi] = c.domain()?;
    let a = c.evaluate(lo)?;
    let b = c.evaluate(hi)?;
    let mut flat = true;
    for control in &c.control_points {
        if control.w <= 0.0 {
            return Err("displacement certificate requires positive weights".into());
        }
        flat &= distance(control.point()?, a, b) <= flatness;
    }
    if flat {
        if out.is_empty() {
            out.push(a);
        }
        out.push(b);
        return Ok(());
    }
    if depth == 20 || out.len() >= 4096 {
        return Err("analytic displacement certificate exhausted subdivision budget".into());
    }
    let (left, right) = c.split((lo + hi) * 0.5)?;
    polyline(left, flatness, depth + 1, out)?;
    polyline(right, flatness, depth + 1, out)
}
/// A polyline with a box around each run of [`RUN`] segments, so the
/// segment nearest a point is found without reading every segment.
struct Target<'a> {
    points: &'a [Vec3],
    /// First segment, one past the last, and the box of their points.
    runs: Vec<(usize, usize, Vec3, Vec3)>,
    /// The largest coordinate magnitude of any point.
    magnitude: f64,
}

const RUN: usize = 32;

impl<'a> Target<'a> {
    fn new(points: &'a [Vec3]) -> Self {
        let segments = points.len().saturating_sub(1);
        let mut runs = Vec::with_capacity(segments.div_ceil(RUN));
        let mut start = 0;
        while start < segments {
            let end = (start + RUN).min(segments);
            let (mut lo, mut hi) = (points[start], points[start]);
            for p in &points[start..=end] {
                lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
                hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
            }
            runs.push((start, end, lo, hi));
            start = end;
        }
        let magnitude = points.iter().fold(0.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs()));
        Self { points, runs, magnitude }
    }

    /// The first segment at the least `distance` from `p`: the one
    /// `windows(2).min_by(distance)` picks. A run is skipped only when its
    /// box is farther than the best so far by more than the rounding of
    /// either reading (1e-12 of the coordinates' size, far above it), so no
    /// segment that could tie or win is skipped; within the scan order a
    /// segment replaces the best only when strictly nearer, as `min_by`
    /// keeps the first of equal ones.
    fn nearest(&self, p: Vec3) -> Option<(Vec3, Vec3)> {
        let pad = 1e-12 * (self.magnitude + p.x.abs().max(p.y.abs()).max(p.z.abs()));
        let mut best: Option<(usize, f64)> = None;
        for &(start, end, lo, hi) in &self.runs {
            if let Some((_, nearest)) = best {
                let gap = |v: f64, l: f64, h: f64| (l - v).max(v - h).max(0.0);
                let (dx, dy, dz) = (gap(p.x, lo.x, hi.x), gap(p.y, lo.y, hi.y), gap(p.z, lo.z, hi.z));
                if (dx * dx + dy * dy + dz * dz).sqrt() > nearest + pad + 1e-12 * nearest {
                    continue;
                }
            }
            for index in start..end {
                let reading = distance(p, self.points[index], self.points[index + 1]);
                if best.is_none_or(|(_, nearest)| reading.total_cmp(&nearest).is_lt()) {
                    best = Some((index, reading));
                }
            }
        }
        best.map(|(index, _)| (self.points[index], self.points[index + 1]))
    }
}

fn directed(
    a: Vec3,
    b: Vec3,
    target: &Target,
    bar: f64,
    depth: usize,
    work: &mut usize,
) -> Result<(), String> {
    // The budget counts the segments a full scan would read, as before.
    *work += target.points.len().saturating_sub(1);
    if *work > 32_000_000 {
        return Err("analytic displacement certificate exhausted comparison budget".into());
    }
    let mid = a.add(b).scale(0.5);
    let segment = target.nearest(mid).ok_or("empty displacement polyline")?;
    let segment = [segment.0, segment.1];
    if distance(a, segment[0], segment[1]).max(distance(b, segment[0], segment[1])) <= bar {
        return Ok(());
    }
    if distance(mid, segment[0], segment[1]) > bar || depth == 16 {
        return Err("analytic edge exceeds assembly displacement".into());
    }
    directed(a, mid, target, bar, depth + 1, work)?;
    directed(mid, b, target, bar, depth + 1, work)
}
pub(super) fn certify(
    original: &NurbsCurve,
    t0: f64,
    t1: f64,
    replacement: &NurbsCurve,
    movement: f64,
) -> Result<(), String> {
    let mut old = original.clone();
    let [lo, hi] = old.domain()?;
    if t1.max(t0) < hi - 1e-12 {
        old = old.split(t1.max(t0))?.0;
    }
    if t1.min(t0) > lo + 1e-12 {
        old = old.split(t1.min(t0))?.1;
    }
    let flatness = movement / 64.0;
    let mut before = Vec::new();
    let mut after = Vec::new();
    polyline(old, flatness, 0, &mut before)?;
    polyline(replacement.clone(), flatness, 0, &mut after)?;
    // Each curve and its polyline are within flatness in both directions:
    // the positive control hull bounds curve->chord; continuity of projection
    // onto the chord bounds chord->curve.
    let bar = movement - 2.0 * flatness;
    let mut work = 0;
    let (to_after, to_before) = (Target::new(&after), Target::new(&before));
    for (source, target) in [(&before, &to_after), (&after, &to_before)] {
        for pair in source.windows(2) {
            directed(pair[0], pair[1], target, bar, 0, &mut work)?;
        }
    }
    Ok(())
}
