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
fn directed(
    a: Vec3,
    b: Vec3,
    target: &[Vec3],
    bar: f64,
    depth: usize,
    work: &mut usize,
) -> Result<(), String> {
    *work += target.len().saturating_sub(1);
    if *work > 32_000_000 {
        return Err("analytic displacement certificate exhausted comparison budget".into());
    }
    let mid = a.add(b).scale(0.5);
    let segment = target
        .windows(2)
        .min_by(|p, q| distance(mid, p[0], p[1]).total_cmp(&distance(mid, q[0], q[1])))
        .ok_or("empty displacement polyline")?;
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
    for (source, target) in [(&before, &after), (&after, &before)] {
        for pair in source.windows(2) {
            directed(pair[0], pair[1], target, bar, 0, &mut work)?;
        }
    }
    Ok(())
}
