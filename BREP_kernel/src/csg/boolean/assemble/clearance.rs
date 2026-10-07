//! Whole-span exclusion for newly reintersected edge motions. Positive
//! rational control hulls contain both curves and their straight homotopy.
//! Work exhaustion and unsupported data decline the repair; neither samples
//! nor a local closest-point solve can authorize clearance.
//! The outward padding is engineering floating point, not interval arithmetic.

use crate::{FaceRecord, NurbsCurve, NurbsSurface, Vec4};
pub(super) fn motion_trace_enabled() -> bool {
    std::env::var_os("BREP_JOINT_MOTION_TRACE").is_some()
}
pub(super) fn trace_motion(value: serde_json::Value) {
    let Some(path) = std::env::var_os("BREP_JOINT_MOTION_TRACE") else {
        return;
    };
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    let Ok(_guard) = LOCK.lock() else {
        return;
    };
    use std::io::Write;
    let result = (|| -> std::io::Result<()> {
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)?;
        writeln!(file, "{value}")
    })();
    if let Err(e) = result {
        eprintln!("joint motion trace: {e}");
    }
}

fn split_surface(
    sf: &NurbsSurface,
    along_u: bool,
    t: f64,
) -> Result<(NurbsSurface, NurbsSurface), String> {
    let curves: Vec<NurbsCurve> = if along_u {
        (0..sf.control_points[0].len())
            .map(|j| {
                NurbsCurve::new(
                    sf.degree_u,
                    sf.knots_u.clone(),
                    sf.control_points.iter().map(|row| row[j]).collect(),
                )
            })
            .collect::<Result<_, _>>()?
    } else {
        sf.control_points
            .iter()
            .map(|row| NurbsCurve::new(sf.degree_v, sf.knots_v.clone(), row.clone()))
            .collect::<Result<_, _>>()?
    };
    let pairs = curves
        .iter()
        .map(|c| c.split(t))
        .collect::<Result<Vec<_>, _>>()?;
    let build = |right: bool| -> Result<NurbsSurface, String> {
        let curves: Vec<_> = pairs
            .iter()
            .map(|p| if right { &p.1 } else { &p.0 })
            .collect();
        let points = if along_u {
            (0..curves[0].control_points.len())
                .map(|i| curves.iter().map(|c| c.control_points[i]).collect())
                .collect()
        } else {
            curves.iter().map(|c| c.control_points.clone()).collect()
        };
        NurbsSurface::new(
            sf.degree_u,
            sf.degree_v,
            if along_u {
                curves[0].knots.clone()
            } else {
                sf.knots_u.clone()
            },
            if along_u {
                sf.knots_v.clone()
            } else {
                curves[0].knots.clone()
            },
            points,
        )
    };
    Ok((build(false)?, build(true)?))
}
fn trimmed_carrier_box(face: &FaceRecord) -> Result<NurbsSurface, String> {
    let mut sf = face.surface.clone();
    let closed = sf.closed_directions()?;
    if face.loops.is_empty() {
        return Ok(sf);
    }
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for c in face
        .loops
        .iter()
        .flat_map(|l| &l.coedges)
        .flat_map(|c| &c.pcurve.control_points)
    {
        if c.w <= 0.0 || !c.w.is_finite() {
            return Err("unsupported trim weights".into());
        }
        for (i, v) in [c.x / c.w, c.y / c.w].into_iter().enumerate() {
            if !v.is_finite() {
                return Err("nonfinite trim controls".into());
            }
            lo[i] = lo[i].min(v);
            hi[i] = hi[i].max(v);
        }
    }
    // A constant-weight bilinear carrier can be reboxed exactly on an
    // extrapolated UV rectangle. The four evaluated corners bound its whole
    // image there; the original finite patch hull does not.
    if !closed.0
        && !closed.1
        && sf.degree_u == 1
        && sf.degree_v == 1
        && sf.control_points.len() == 2
        && sf.control_points.iter().all(|r| r.len() == 2)
        && sf
            .control_points
            .iter()
            .flatten()
            .all(|p| p.w == sf.control_points[0][0].w && p.w > 0.0)
        && hi[0] > lo[0]
        && hi[1] > lo[1]
    {
        let mut points = Vec::new();
        for u in [lo[0], hi[0]] {
            let mut row = Vec::new();
            for v in [lo[1], hi[1]] {
                row.push(Vec4::from_point(sf.evaluate_extended(u, v)?, 1.0));
            }
            points.push(row);
        }
        return NurbsSurface::new(
            1,
            1,
            vec![lo[0], lo[0], hi[0], hi[0]],
            vec![lo[1], lo[1], hi[1], hi[1]],
            points,
        );
    }
    for axis in 0..2 {
        let along_u = axis == 0;
        let [a, b] = if along_u {
            sf.domain_u()?
        } else {
            sf.domain_v()?
        };
        let pad = 128.0 * f64::EPSILON * (1.0 + lo[axis].abs().max(hi[axis].abs()));
        let periodic = if along_u { closed.0 } else { closed.1 };
        if periodic {
            continue;
        }
        if lo[axis] < a || hi[axis] > b {
            return Err("trim outside nonperiodic carrier domain".into());
        }
        let start = lo[axis] - pad;
        let end = hi[axis] + pad;
        if !start.is_finite() || !end.is_finite() || start >= end || end <= a || start >= b {
            return Ok(face.surface.clone());
        }
        if start > a {
            if let Ok((_, right)) = split_surface(&sf, along_u, start) {
                if (if along_u {
                    right.domain_u()?
                } else {
                    right.domain_v()?
                })[0]
                    <= lo[axis]
                {
                    sf = right;
                }
            }
        }
        if end < b {
            if let Ok((left, _)) = split_surface(&sf, along_u, end) {
                if (if along_u {
                    left.domain_u()?
                } else {
                    left.domain_v()?
                })[1]
                    >= hi[axis]
                {
                    sf = left;
                }
            }
        }
    }
    Ok(sf)
}
#[derive(Clone, Copy)]
struct Bounds {
    lo: [f64; 3],
    hi: [f64; 3],
}
fn bounds<'a>(points: impl Iterator<Item = &'a Vec4>) -> Option<Bounds> {
    let mut b = Bounds {
        lo: [f64::INFINITY; 3],
        hi: [f64::NEG_INFINITY; 3],
    };
    for p in points {
        if !p.w.is_finite() || p.w <= 0. {
            return None;
        }
        for (i, x) in [p.x / p.w, p.y / p.w, p.z / p.w].into_iter().enumerate() {
            if !x.is_finite() {
                return None;
            }
            b.lo[i] = b.lo[i].min(x);
            b.hi[i] = b.hi[i].max(x);
        }
    }
    Some(b)
}
#[derive(Clone)]
enum CarrierRegion {
    Tensor(NurbsSurface),
    Native { surface: NurbsSurface, uv: Bounds },
}
impl CarrierRegion {
    fn bounds(&self) -> Option<Bounds> {
        match self {
            Self::Tensor(sf) => bounds(sf.control_points.iter().flatten()),
            Self::Native { surface, uv } => general_trim_image_bounds(surface, *uv),
        }
    }
    fn outside(&self, face: &FaceRecord) -> bool {
        match self {
            Self::Tensor(sf) => affine_patch_outside(face, sf),
            _ => false,
        }
    }
    fn split(&self, along_u: bool) -> Option<(Self, Self)> {
        match self {
            Self::Tensor(sf) => {
                let [a, b] = if along_u {
                    sf.domain_u().ok()?
                } else {
                    sf.domain_v().ok()?
                };
                let (a, b) = split_surface(sf, along_u, (a + b) * 0.5).ok()?;
                Some((Self::Tensor(a), Self::Tensor(b)))
            }
            Self::Native { surface, uv } => {
                let mut axis = if along_u { 0 } else { 1 };
                if uv.hi[axis] == uv.lo[axis] {
                    axis = 1 - axis;
                }
                let mid = uv.lo[axis] + (uv.hi[axis] - uv.lo[axis]) * 0.5;
                if mid <= uv.lo[axis] || mid >= uv.hi[axis] {
                    return None;
                }
                let (mut a, mut b) = (*uv, *uv);
                a.hi[axis] = mid;
                b.lo[axis] = mid;
                Some((
                    Self::Native {
                        surface: surface.clone(),
                        uv: a,
                    },
                    Self::Native {
                        surface: surface.clone(),
                        uv: b,
                    },
                ))
            }
        }
    }
}
/// Preserve actual native extensions as UV regions. A finite patch hull
/// cannot stand in for material whose positive trim hull leaves an open chart.
fn carrier_region(face: &FaceRecord) -> Result<CarrierRegion, String> {
    match trimmed_carrier_box(face) {
        Ok(sf) => return Ok(CarrierRegion::Tensor(sf)),
        Err(e) if e == "trim outside nonperiodic carrier domain" => {}
        Err(e) => return Err(e),
    }
    let mut uv = Bounds {
        lo: [f64::INFINITY, f64::INFINITY, 0.0],
        hi: [f64::NEG_INFINITY, f64::NEG_INFINITY, 0.0],
    };
    for h in face
        .loops
        .iter()
        .flat_map(|l| &l.coedges)
        .flat_map(|c| &c.pcurve.control_points)
    {
        if h.w <= 0.0 || ![h.x, h.y, h.w].iter().all(|v| v.is_finite()) {
            return Err("unsupported native trim hull".into());
        }
        for (i, x) in [h.x / h.w, h.y / h.w].into_iter().enumerate() {
            uv.lo[i] = uv.lo[i].min(x);
            uv.hi[i] = uv.hi[i].max(x);
        }
    }
    let closed = face.surface.closed_directions()?;
    for (axis, periodic, domain) in [
        (0, closed.0, face.surface.domain_u()?),
        (1, closed.1, face.surface.domain_v()?),
    ] {
        if periodic {
            uv.lo[axis] = domain[0];
            uv.hi[axis] = domain[1];
        }
        if !uv.lo[axis].is_finite() || !uv.hi[axis].is_finite() {
            return Err("nonfinite native trim hull".into());
        }
    }
    let region = CarrierRegion::Native {
        surface: face.surface.clone(),
        uv,
    };
    region
        .bounds()
        .ok_or_else(|| "unsupported native carrier enclosure".to_string())?;
    Ok(region)
}

fn merge(a: Bounds, b: Bounds) -> Bounds {
    let mut c = a;
    for i in 0..3 {
        c.lo[i] = c.lo[i].min(b.lo[i]);
        c.hi[i] = c.hi[i].max(b.hi[i]);
    }
    c
}
fn separate(a: Bounds, b: Bounds) -> bool {
    (0..3).any(|i| {
        let pad = 256.
            * f64::EPSILON
            * (1. + a.lo[i].abs().max(a.hi[i].abs()) + b.lo[i].abs().max(b.hi[i].abs()));
        a.hi[i] + pad < b.lo[i] || b.hi[i] + pad < a.lo[i]
    })
}
/// A proposed continuous, covering monotone parameter correspondence. Closest
/// UV/3D stations only choose the homotopy; every tile still needs a whole
/// positive-hull exclusion. No measured distance authorizes clearance.
fn covering_correspondence(
    old: &NurbsCurve,
    ar: [f64; 2],
    new: &NurbsCurve,
    br: [f64; 2],
) -> Option<Vec<(f64, f64)>> {
    let mut nodes: Vec<_> = (0..=64).map(|i| i as f64 / 64.0).collect();
    nodes.extend(
        old.knots
            .iter()
            .filter(|&&t| t > ar[0] && t < ar[1])
            .map(|&t| (t - ar[0]) / (ar[1] - ar[0])),
    );
    nodes.sort_by(f64::total_cmp);
    nodes.dedup();
    if nodes.len() > 4096 {
        return None;
    }
    let mut result = vec![(0.0, 0.0)];
    for &f in &nodes[1..nodes.len() - 1] {
        let p = old.evaluate(ar[0] + f * (ar[1] - ar[0])).ok()?;
        let t = crate::project_point_to_curve(new, p).ok()?.u;
        let g = (t - br[0]) / (br[1] - br[0]);
        if !g.is_finite() || g < result.last()?.1 || g > 1.0 {
            return None;
        }
        result.push((f, g));
    }
    result.push((1.0, 1.0));
    Some(result)
}
/// Bound complete paired image motion through the same covering map as the
/// safety sweep. Samples select a map; complete positive rational difference
/// bounds authorize every affine-map tile without its longitudinal box width.
pub(super) fn covering_motion_bound(old: &NurbsCurve, new: &NurbsCurve) -> Option<f64> {
    let ar = old.domain().ok()?;
    let br = new.domain().ok()?;
    let nodes = covering_correspondence(old, ar, new, br)?;
    let old_spans = super::image::bezier_spans(old)?;
    let new_spans = super::image::bezier_spans(new)?;
    let mut stack = Vec::new();
    for pair in nodes.windows(2) {
        let ((left, x), (right, y)) = (pair[0], pair[1]);
        let mut breaks = vec![left, right];
        if y > x {
            let (a, b) = (br[0] + x * (br[1] - br[0]), br[0] + y * (br[1] - br[0]));
            breaks.extend(
                new.knots
                    .iter()
                    .filter(|&&k| k > a && k < b)
                    .map(|&k| left + (right - left) * (k - a) / (b - a)),
            );
        }
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        stack.extend(
            breaks
                .windows(2)
                .filter(|s| s[0] < s[1])
                .map(|s| (s[0], s[1], 0_u32)),
        );
    }
    let mut work = 0;
    let mut maximum = 0.0_f64;
    while let Some((left, right, depth)) = stack.pop() {
        work += 1;
        if work > 8191 {
            return None;
        }
        let (a, b) = (
            ar[0] + left * (ar[1] - ar[0]),
            ar[0] + right * (ar[1] - ar[0]),
        );
        let (x, y) = (
            br[0] + mapped(&nodes, left) * (br[1] - br[0]),
            br[0] + mapped(&nodes, right) * (br[1] - br[0]),
        );
        let distance = if x == y {
            // A plateau covers a point, including both sides of a full-
            // multiplicity join. Positive hulls bound every pairing.
            let ac = super::image::restricted_bezier_controls(&old_spans, a, b)?;
            let bc = super::image::restricted_bezier_controls(&new_spans, x, y)?;
            let mut d = 0.0_f64;
            for h in &ac {
                for k in &bc {
                    let (p, q) = (h.point().ok()?, k.point().ok()?);
                    let distance = p.sub(q).length();
                    d = d.max(
                        distance
                            + 1024.0 * f64::EPSILON * (1.0 + distance + p.length() + q.length()),
                    );
                }
            }
            d + 1024.0 * f64::EPSILON * (1.0 + d)
        } else {
            match (
                super::image::restricted_single_span_controls(&old_spans, a, b),
                super::image::restricted_single_span_controls(&new_spans, x, y),
            ) {
                (Some(ac), Some(bc)) => super::image::bezier_difference_bound(ac, bc)?,
                _ => {
                    // Rounded partition endpoints can straddle a native knot.
                    // Keep both sides in positive hulls rather than snap away
                    // that tiny interval or create a degenerate NURBS domain.
                    let ac = super::image::restricted_bezier_controls(&old_spans, a, b)?;
                    let bc = super::image::restricted_bezier_controls(&new_spans, x, y)?;
                    let mut d = 0.0_f64;
                    for h in &ac {
                        for k in &bc {
                            let (p, q) = (h.point().ok()?, k.point().ok()?);
                            let distance = p.sub(q).length();
                            d = d.max(
                                distance
                                    + 1024.0
                                        * f64::EPSILON
                                        * (1.0 + distance + p.length() + q.length()),
                            );
                        }
                    }
                    d + 1024.0 * f64::EPSILON * (1.0 + d)
                }
            }
        };
        if !distance.is_finite() {
            return None;
        }
        if distance <= 4e-3 {
            maximum = maximum.max(distance);
        } else {
            if depth >= 24 {
                return None;
            }
            let mid = left + (right - left) * 0.5;
            if mid == left || mid == right {
                return None;
            }
            stack.push((left, mid, depth + 1));
            stack.push((mid, right, depth + 1));
        }
    }
    Some(maximum)
}

fn mapped(nodes: &[(f64, f64)], f: f64) -> f64 {
    if f <= 0.0 {
        return 0.0;
    }
    if f >= 1.0 {
        return 1.0;
    }
    let i = nodes.partition_point(|n| n.0 <= f) - 1;
    let (a, x) = nodes[i];
    let (b, y) = nodes[i + 1];
    x + (y - x) * (f - a) / (b - a)
}
fn span_bounds(source: &[NurbsCurve], range: [f64; 2], left: f64, right: f64) -> Option<Bounds> {
    let a = range[0] + (range[1] - range[0]) * left;
    let b = range[0] + (range[1] - range[0]) * right;
    let controls = super::image::restricted_bezier_controls(source, a, b)?;
    bounds(controls.iter())
}
/// An affine carrier rectangle is outside material only if it is outside
/// EVERY loop. Holes are deliberately not used as exclusions. Subdivision
/// replaces a positive rational curve by a control polygon only when its
/// complete hull avoids the rectangle: that homotopy preserves winding.
/// Implicit straight joint closures match the affine material-area policy.
fn affine_patch_outside(face: &FaceRecord, patch: &NurbsSurface) -> bool {
    if face.loops.is_empty()
        || !face.surface.is_affine().unwrap_or(false)
        || face.surface.closed_directions().ok() != Some((false, false))
    {
        return false;
    }
    let (Ok(u), Ok(v)) = (patch.domain_u(), patch.domain_v()) else {
        return false;
    };
    let lo = [u[0], v[0]];
    let hi = [u[1], v[1]];
    let center = [lo[0] + (hi[0] - lo[0]) * 0.5, lo[1] + (hi[1] - lo[1]) * 0.5];
    if center.iter().chain(&lo).chain(&hi).any(|v| !v.is_finite()) {
        return false;
    }
    let disjoint = |points: &[Vec4]| -> bool {
        if points.is_empty() || points.iter().any(|p| !(p.w > 0.0) || !p.w.is_finite()) {
            return false;
        }
        let mut a = [f64::INFINITY; 2];
        let mut b = [f64::NEG_INFINITY; 2];
        for p in points {
            for (i, x) in [p.x / p.w, p.y / p.w].into_iter().enumerate() {
                if !x.is_finite() {
                    return false;
                }
                a[i] = a[i].min(x);
                b[i] = b[i].max(x);
            }
        }
        (0..2).any(|i| {
            let pad = 256.0
                * f64::EPSILON
                * (1.0 + a[i].abs().max(b[i].abs()) + lo[i].abs().max(hi[i].abs()));
            b[i] + pad < lo[i] || hi[i] + pad < a[i]
        })
    };
    let mut work = 0usize;
    for loop_record in &face.loops {
        if loop_record.coedges.is_empty() {
            return false;
        }
        let mut polygon = Vec::<[f64; 2]>::new();
        for coedge in &loop_record.coedges {
            let mut pending = vec![(coedge.pcurve.clone(), 0usize)];
            while let Some((curve, depth)) = pending.pop() {
                work += 1;
                if work > 256 {
                    return false;
                }
                let Ok([a, b]) = curve.domain() else {
                    return false;
                };
                if curve.knots.iter().take(curve.degree + 1).any(|t| *t != a)
                    || curve
                        .knots
                        .iter()
                        .rev()
                        .take(curve.degree + 1)
                        .any(|t| *t != b)
                {
                    return false;
                }
                if disjoint(&curve.control_points) {
                    polygon.extend(curve.control_points.iter().map(|p| [p.x / p.w, p.y / p.w]));
                } else {
                    if depth >= 24 {
                        return false;
                    }
                    let Ok((left, right)) = curve.split((a + b) * 0.5) else {
                        return false;
                    };
                    pending.push((right, depth + 1));
                    pending.push((left, depth + 1));
                }
            }
        }
        // Curve subdivisions preserve endpoints. Consecutive polygons are
        // joined exactly as the affine face closes its trim gaps.
        let mut winding = 0i32;
        for i in 0..polygon.len() {
            let a = polygon[i];
            let b = polygon[(i + 1) % polygon.len()];
            if !disjoint(&[
                Vec4::from_point(crate::Vec3::new(a[0], a[1], 0.0), 1.0),
                Vec4::from_point(crate::Vec3::new(b[0], b[1], 0.0), 1.0),
            ]) {
                return false;
            }
            let cross = (b[0] - a[0]) * (center[1] - a[1]) - (b[1] - a[1]) * (center[0] - a[0]);
            let pad = 256.0
                * f64::EPSILON
                * (1.0
                    + (b[0] - a[0]).abs() * (center[1] - a[1]).abs()
                    + (b[1] - a[1]).abs() * (center[0] - a[0]).abs());
            if !cross.is_finite() || !pad.is_finite() {
                return false;
            }
            if !cross.is_finite() || !pad.is_finite() {
                return false;
            }
            if a[1] <= center[1] && b[1] > center[1] {
                if cross.abs() <= pad {
                    return false;
                }
                if cross > 0.0 {
                    winding += 1;
                }
            } else if a[1] > center[1] && b[1] <= center[1] {
                if cross.abs() <= pad {
                    return false;
                }
                if cross < 0.0 {
                    winding -= 1;
                }
            }
        }
        if winding != 0 {
            return false;
        }
    }
    true
}

fn swept_edge_clear(
    a: &NurbsCurve,
    ar: [f64; 2],
    b: &NurbsCurve,
    br: [f64; 2],
    face: &FaceRecord,
    budget: usize,
) -> Result<(bool, usize, &'static str), String> {
    let Some(a_spans) = super::image::bezier_spans(a) else {
        return Ok((false, 0, "unsupported"));
    };
    let Some(b_spans) = super::image::bezier_spans(b) else {
        return Ok((false, 0, "unsupported"));
    };
    let Some(mapping) = covering_correspondence(a, ar, b, br) else {
        return Ok((false, 0, "correspondence"));
    };
    if motion_trace_enabled() {
        trace_motion(
            serde_json::json!({"kind":"edge-motion","target":face,"old":a,"new":b,"old_range":ar,"new_range":br,"mapping":mapping}),
        );
    }
    let sf = carrier_region(face)?;
    let mut pending = vec![(0., 1., sf, 0usize)];
    for count in 0..budget {
        let Some((left, right, sf, depth)) = pending.pop() else {
            return Ok((true, count, "separated"));
        };
        let (Some(ba), Some(bb), Some(bs)) = (
            span_bounds(&a_spans, ar, left, right),
            span_bounds(
                &b_spans,
                br,
                mapped(&mapping, left),
                mapped(&mapping, right),
            ),
            sf.bounds(),
        ) else {
            return Ok((false, count, "unsupported"));
        };
        if separate(merge(ba, bb), bs) {
            continue;
        }
        if sf.outside(face) {
            continue;
        }
        if depth >= 48 {
            return Ok((false, count + 1, "depth"));
        }
        if depth % 3 == 0 {
            let middle = (left + right) / 2.;
            pending.push((left, middle, sf.clone(), depth + 1));
            pending.push((middle, right, sf, depth + 1));
        } else {
            let along_u = depth % 3 == 1;
            let Some((a, b)) = sf.split(along_u) else {
                return Ok((false, count + 1, "split"));
            };
            pending.push((left, right, a, depth + 1));
            pending.push((left, right, b, depth + 1));
        }
    }
    Ok((
        pending.is_empty(),
        budget,
        if pending.is_empty() {
            "separated"
        } else {
            "budget"
        },
    ))
}

pub(super) fn motion_clear(
    old: &crate::topology::EdgeRecord,
    curve: &NurbsCurve,
    faces: &[&FaceRecord],
    edges: &[crate::topology::EdgeRecord],
) -> bool {
    let Ok(range) = curve.domain() else {
        return false;
    };
    for face in faces {
        let vertex_incident = face.loops.iter().flat_map(|l| &l.coedges).any(|c| {
            edges.iter().find(|e| e.id == c.edge_id).is_some_and(|e| {
                [e.start_vertex_id, e.end_vertex_id]
                    .iter()
                    .any(|v| *v == old.start_vertex_id || *v == old.end_vertex_id)
            })
        });
        // Existing shared-vertex contacts cannot be excluded by disjoint hulls.
        // Carrier residual and assembler soundness checks cover those faces.
        if vertex_incident {
            continue;
        }
        let result = swept_edge_clear(&old.curve, [old.t0, old.t1], curve, range, face, 1024);
        if !matches!(&result, Ok((true, _, _))) {
            if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
                eprintln!(
                    "edge clearance: edge {} target {} {result:?}",
                    old.id, face.id
                );
            }
            return false;
        }
    }
    true
}

/// Whole-domain rational derivative bound, translated to a nearby origin to
/// avoid making the padding depend on the model's world coordinates.
fn derivative_u_bound(sf: &NurbsSurface) -> Option<[f64; 3]> {
    let origin = sf.control_points.first()?.first()?.point().ok()?;
    let coord = |h: &Vec4, i| match i {
        0 => h.x - origin.x * h.w,
        1 => h.y - origin.y * h.w,
        _ => h.z - origin.z * h.w,
    };
    let mut n = [0.0_f64; 3];
    let mut dn = [0.0_f64; 3];
    let mut dw = 0.0_f64;
    let mut minimum = f64::INFINITY;
    let mut maximum = 0.0_f64;
    for h in sf.control_points.iter().flatten() {
        if h.w <= 0.0 || !h.w.is_finite() {
            return None;
        }
        minimum = minimum.min(h.w);
        maximum = maximum.max(h.w);
        for i in 0..3 {
            n[i] = n[i].max(coord(h, i).abs());
        }
    }
    for row in 0..sf.control_points.len() - 1 {
        let span = sf.knots_u[row + sf.degree_u + 1] - sf.knots_u[row + 1];
        if span <= 0.0 {
            return None;
        }
        let factor = sf.degree_u as f64 / span;
        for column in 0..sf.control_points[row].len() {
            let (a, b) = (
                &sf.control_points[row][column],
                &sf.control_points[row + 1][column],
            );
            dw = dw.max(((b.w - a.w) * factor).abs());
            for i in 0..3 {
                dn[i] = dn[i].max(((coord(b, i) - coord(a, i)) * factor).abs());
            }
        }
    }
    let bound = std::array::from_fn(|i| {
        (dn[i] / minimum) * (maximum / minimum) + (n[i] / minimum) * (dw / minimum)
    });
    bound.iter().all(|x| x.is_finite()).then_some(bound)
}

/// Absolute partial bounds of a positive rational tensor patch. Translation
/// removes world-origin cancellation; derivative control hulls bound the
/// quotient expressions, including the mixed anchor partial.
fn partial_bounds(sf: &NurbsSurface) -> Option<([f64; 3], [f64; 3], [f64; 3])> {
    let origin = sf.control_points.first()?.first()?.point().ok()?;
    let cp: Vec<Vec<_>> = sf
        .control_points
        .iter()
        .map(|r| {
            r.iter()
                .map(|h| Vec4 {
                    x: h.x - origin.x * h.w,
                    y: h.y - origin.y * h.w,
                    z: h.z - origin.z * h.w,
                    w: h.w,
                })
                .collect()
        })
        .collect();
    let minimum = cp
        .iter()
        .flatten()
        .map(|p| p.w)
        .fold(f64::INFINITY, f64::min);
    if minimum <= 0.0 || !minimum.is_finite() {
        return None;
    }
    let maxima = |points: &[Vec4]| -> Option<[f64; 4]> {
        let mut bound = [0.0_f64; 4];
        for p in points {
            for (i, v) in [p.x, p.y, p.z, p.w].into_iter().enumerate() {
                if !v.is_finite() {
                    return None;
                }
                bound[i] = bound[i].max(v.abs());
            }
        }
        Some(bound)
    };
    let subtract = |a: Vec4, b: Vec4, f: f64| Vec4 {
        x: (a.x - b.x) * f,
        y: (a.y - b.y) * f,
        z: (a.z - b.z) * f,
        w: (a.w - b.w) * f,
    };
    let mut du = Vec::new();
    let mut dv = Vec::new();
    let mut mixed = Vec::new();
    for i in 0..cp.len() - 1 {
        let den = sf.knots_u[i + sf.degree_u + 1] - sf.knots_u[i + 1];
        if den <= 0.0 {
            return None;
        }
        let factor = sf.degree_u as f64 / den;
        for j in 0..cp[0].len() {
            du.push(subtract(cp[i + 1][j], cp[i][j], factor));
        }
        for j in 0..cp[0].len() - 1 {
            let den = sf.knots_v[j + sf.degree_v + 1] - sf.knots_v[j + 1];
            if den <= 0.0 {
                return None;
            }
            let a = subtract(cp[i + 1][j + 1], cp[i][j + 1], factor);
            let b = subtract(cp[i + 1][j], cp[i][j], factor);
            mixed.push(subtract(a, b, sf.degree_v as f64 / den));
        }
    }
    for row in &cp {
        for j in 0..row.len() - 1 {
            let den = sf.knots_v[j + sf.degree_v + 1] - sf.knots_v[j + 1];
            if den <= 0.0 {
                return None;
            }
            dv.push(subtract(row[j + 1], row[j], sf.degree_v as f64 / den));
        }
    }
    let n = maxima(&cp.into_iter().flatten().collect::<Vec<_>>())?;
    let u = maxima(&du)?;
    let v = maxima(&dv)?;
    let uv = maxima(&mixed)?;
    let first =
        |d: [f64; 4]| std::array::from_fn(|i| d[i] / minimum + n[i] * d[3] / minimum.powi(2));
    let mixed = std::array::from_fn(|i| {
        uv[i] / minimum
            + (u[i] * v[3] + v[i] * u[3] + n[i] * uv[3]) / minimum.powi(2)
            + 2.0 * n[i] * u[3] * v[3] / minimum.powi(3)
    });
    let u = first(u);
    let v = first(v);
    u.iter()
        .chain(&v)
        .chain(&mixed)
        .all(|x| x.is_finite())
        .then_some((u, v, mixed))
}

/// A Lipschitz enclosure of the actual native tangent/corner continuation.
/// Positive rational tensor derivative hulls cover every anchor. Periodic
/// excursions decline, since an approximate seam classification does not
/// justify a local derivative enclosure across a wrap.
fn general_trim_image_bounds(surface: &NurbsSurface, uv: Bounds) -> Option<Bounds> {
    let (u, v) = (surface.domain_u().ok()?, surface.domain_v().ok()?);
    let closed = surface.closed_directions().ok()?;
    if (closed.0 && (uv.lo[0] < u[0] || uv.hi[0] > u[1]))
        || (closed.1 && (uv.lo[1] < v[0] || uv.hi[1] > v[1]))
    {
        return None;
    }
    // Restrict to a rectangle containing every clamped native anchor. A
    // snapped or failed split keeps the wider positive hull.
    let mut local = surface.clone();
    for (axis, domain) in [(0, u), (1, v)] {
        let lo = uv.lo[axis].clamp(domain[0], domain[1]);
        let hi = uv.hi[axis].clamp(domain[0], domain[1]);
        if hi - lo <= 8.0 * crate::KNOT_IDENTITY_TOL {
            continue;
        }
        if lo > domain[0] {
            if let Ok((_, tail)) = split_surface(&local, axis == 0, lo) {
                if (if axis == 0 {
                    tail.domain_u().ok()?
                } else {
                    tail.domain_v().ok()?
                })[0]
                    <= lo
                {
                    local = tail;
                }
            }
        }
        if hi < domain[1] {
            if let Ok((head, _)) = split_surface(&local, axis == 0, hi) {
                if (if axis == 0 {
                    head.domain_u().ok()?
                } else {
                    head.domain_v().ok()?
                })[1]
                    >= hi
                {
                    local = head;
                }
            }
        }
    }
    if uv.lo[0] >= u[0] && uv.hi[0] <= u[1] && uv.lo[1] >= v[0] && uv.hi[1] <= v[1] {
        let wu = uv.hi[0] - uv.lo[0];
        let wv = uv.hi[1] - uv.lo[1];
        let thin_u = wu <= 8.0 * crate::KNOT_IDENTITY_TOL;
        let thin_v = wv <= 8.0 * crate::KNOT_IDENTITY_TOL;
        if !thin_u && !thin_v {
            return bounds(local.control_points.iter().flatten());
        }
        let (du, dv, _) = partial_bounds(&local)?;
        let mut result = if thin_u && !thin_v {
            let c = local.iso_curve_u((uv.lo[0] + uv.hi[0]) * 0.5).ok()?;
            let cp = super::image::restricted_controls(&c, uv.lo[1], uv.hi[1])?;
            bounds(cp.iter())?
        } else if thin_v && !thin_u {
            let c = local.iso_curve_v((uv.lo[1] + uv.hi[1]) * 0.5).ok()?;
            let cp = super::image::restricted_controls(&c, uv.lo[0], uv.hi[0])?;
            bounds(cp.iter())?
        } else {
            let p = surface
                .evaluate_extended((uv.lo[0] + uv.hi[0]) * 0.5, (uv.lo[1] + uv.hi[1]) * 0.5)
                .ok()?;
            Bounds {
                lo: [p.x, p.y, p.z],
                hi: [p.x, p.y, p.z],
            }
        };
        for i in 0..3 {
            let r = 0.5
                * ((if thin_u { du[i] * wu } else { 0.0 })
                    + (if thin_v { dv[i] * wv } else { 0.0 }));
            if !r.is_finite() {
                return None;
            }
            result.lo[i] -= r;
            result.hi[i] += r;
        }
        return Some(result);
    }
    let (du, dv, duv) = partial_bounds(&local)?;
    let excursion = |lo: f64, hi: f64, d: [f64; 2]| (d[0] - lo).max(hi - d[1]).max(0.0);
    let outu = excursion(uv.lo[0], uv.hi[0], u);
    let outv = excursion(uv.lo[1], uv.hi[1], v);
    let mut anchors = uv;
    for (axis, domain) in [(0, u), (1, v)] {
        anchors.lo[axis] = uv.lo[axis].clamp(domain[0], domain[1]);
        anchors.hi[axis] = uv.hi[axis].clamp(domain[0], domain[1]);
    }
    let mut b = general_trim_image_bounds(surface, anchors)?;
    for i in 0..3 {
        let radius = outu * du[i] + outv * dv[i] + outu * outv * duv[i];
        if !radius.is_finite() {
            return None;
        }
        let pad = 1024.0 * f64::EPSILON * (1.0 + b.lo[i].abs().max(b.hi[i].abs()) + radius);
        b.lo[i] -= radius + pad;
        b.hi[i] += radius + pad;
    }
    Some(b)
}

/// Bound the images of two trim spans and their homotopy on a linear-v
/// rational carrier. A positive UV hull rectangle is reboxed exactly across
/// v; restriction across u uses homogeneous knot insertion. Failed near-end
/// splits retain the wider hull, so they can only decline an exclusion.
fn trim_image_bounds(surface: &NurbsSurface, uv: Bounds) -> Option<Bounds> {
    // Uniform-weight open bilinear patches have an exact polynomial corner
    // extension. Bound the complete UV rectangle, even with zero-width axes.
    if surface.degree_u == 1
        && surface.degree_v == 1
        && surface.control_points.len() == 2
        && surface.control_points.iter().all(|r| r.len() == 2)
        && surface.closed_directions().ok()? == (false, false)
        && surface
            .control_points
            .iter()
            .flatten()
            .all(|p| p.w > 0.0 && p.w == surface.control_points[0][0].w)
    {
        let [u0, u1] = surface.domain_u().ok()?;
        let [v0, v1] = surface.domain_v().ok()?;
        if surface.knots_u != vec![u0, u0, u1, u1] || surface.knots_v != vec![v0, v0, v1, v1] {
            return None;
        }
        let mut corners = Vec::new();
        for u in [uv.lo[0], uv.hi[0]] {
            for v in [uv.lo[1], uv.hi[1]] {
                corners.push(Vec4::from_point(surface.evaluate_extended(u, v).ok()?, 1.0));
            }
        }
        return bounds(corners.iter());
    }
    if surface.degree_v != 1 {
        return general_trim_image_bounds(surface, uv);
    }
    if surface.control_points.iter().any(|r| r.len() != 2) {
        return None;
    }
    let mut sf = surface.clone();
    let (du, dv) = (sf.domain_u().ok()?, sf.domain_v().ok()?);
    let closed = sf.closed_directions().ok()?;
    let linear_v = sf.control_points.iter().all(|r| r[0].w == r[1].w);
    if (!closed.0 && (uv.lo[0] < du[0] || uv.hi[0] > du[1]))
        || (!closed.1 && !linear_v && (uv.lo[1] < dv[0] || uv.hi[1] > dv[1]))
    {
        return None;
    }
    let ur = if closed.0 && (uv.lo[0] < du[0] || uv.hi[0] > du[1]) {
        du
    } else {
        [uv.lo[0].max(du[0]), uv.hi[0].min(du[1])]
    };
    let thin = ur[1] - ur[0] <= 8.0 * crate::KNOT_IDENTITY_TOL;
    if !thin && ur[0] > du[0] {
        if let Ok((_, tail)) = split_surface(&sf, true, ur[0]) {
            if tail.domain_u().ok()?[0] <= ur[0] {
                sf = tail;
            }
        }
    }
    if !thin && ur[1] < du[1] {
        if let Ok((head, _)) = split_surface(&sf, true, ur[1]) {
            if head.domain_u().ok()?[1] >= ur[1] {
                sf = head;
            }
        }
    }
    let vr = if closed.1 { dv } else { [uv.lo[1], uv.hi[1]] };
    let mut controls = Vec::new();
    for row in &sf.control_points {
        let mut corners = Vec::new();
        for v in vr {
            let f = (v - dv[0]) / (dv[1] - dv[0]);
            let (a, b) = (row[0], row[1]);
            corners.push(Vec4 {
                x: a.x + f * (b.x - a.x),
                y: a.y + f * (b.y - a.y),
                z: a.z + f * (b.z - a.z),
                w: if a.w == b.w {
                    a.w
                } else {
                    a.w + f * (b.w - a.w)
                },
            });
        }
        controls.push(corners);
    }
    if thin {
        let reboxed = NurbsSurface::new(
            sf.degree_u,
            1,
            sf.knots_u.clone(),
            vec![0.0, 0.0, 1.0, 1.0],
            controls,
        )
        .ok()?;
        let mid = (ur[0] + ur[1]) * 0.5;
        let iso = reboxed.iso_curve_u(mid).ok()?;
        let mut bound = bounds(iso.control_points.iter())?;
        let derivative = derivative_u_bound(&reboxed)?;
        for i in 0..3 {
            let error = derivative[i] * (ur[1] - ur[0]) * 0.5;
            bound.lo[i] -= error;
            bound.hi[i] += error;
        }
        Some(bound)
    } else {
        bounds(controls.iter().flatten())
    }
}

/// All spans of the old/new trim images must exclude every nonincident face.
/// Unsupported domains, depth or work exhaustion reject the transaction.
/// Shared-vertex contacts stay with residual and topology qualification.
pub(super) fn trim_motion_clear(
    surface: &NurbsSurface,
    old: &NurbsCurve,
    new: &NurbsCurve,
    edge: &crate::topology::EdgeRecord,
    faces: &[&FaceRecord],
    edges: &[crate::topology::EdgeRecord],
) -> bool {
    let (Ok(ar), Ok(br)) = (old.domain(), new.domain()) else {
        return false;
    };
    let (Some(old_spans), Some(new_spans)) = (
        super::image::bezier_spans(old),
        super::image::bezier_spans(new),
    ) else {
        return false;
    };
    let Some(mapping) = covering_correspondence(old, ar, new, br) else {
        return false;
    };
    if motion_trace_enabled() {
        trace_motion(
            serde_json::json!({"kind":"trim-motion","edge":edge,"surface":surface,"old":old,"new":new,"old_range":ar,"new_range":br,"mapping":mapping}),
        );
    }
    for face in faces {
        let incident = face.loops.iter().flat_map(|l| &l.coedges).any(|c| {
            edges.iter().find(|e| e.id == c.edge_id).is_some_and(|e| {
                [e.start_vertex_id, e.end_vertex_id]
                    .iter()
                    .any(|v| *v == edge.start_vertex_id || *v == edge.end_vertex_id)
            })
        });
        if incident {
            continue;
        }
        let Ok(sf) = carrier_region(face) else {
            if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
                eprintln!(
                    "trim clearance: edge {} target {} held at line {}",
                    edge.id,
                    face.id,
                    line!()
                );
            }
            return false;
        };
        let mut pending = vec![(0.0, 1.0, sf, 0usize)];
        let mut work = 0usize;
        while let Some((left, right, sf, depth)) = pending.pop() {
            work += 1;
            if work > 1024 || depth >= 48 {
                if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
                    eprintln!(
                        "trim clearance: edge {} target {} held at line {}",
                        edge.id,
                        face.id,
                        line!()
                    );
                }
                return false;
            }
            let (Some(a), Some(b), Some(target)) = (
                span_bounds(&old_spans, ar, left, right),
                span_bounds(
                    &new_spans,
                    br,
                    mapped(&mapping, left),
                    mapped(&mapping, right),
                ),
                sf.bounds(),
            ) else {
                if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
                    eprintln!(
                        "trim clearance: edge {} target {} held at line {}",
                        edge.id,
                        face.id,
                        line!()
                    );
                }
                return false;
            };
            let Some(image) = trim_image_bounds(surface, merge(a, b)) else {
                if std::env::var("BREP_DEBUG_JOINTS").is_ok() {
                    eprintln!(
                        "trim clearance: edge {} target {} held at line {}",
                        edge.id,
                        face.id,
                        line!()
                    );
                }
                return false;
            };
            if separate(image, target) || sf.outside(face) {
                continue;
            }
            if depth % 3 == 0 {
                let middle = (left + right) * 0.5;
                pending.push((left, middle, sf.clone(), depth + 1));
                pending.push((middle, right, sf, depth + 1));
            } else {
                let along_u = depth % 3 == 1;
                let Some((a, b)) = sf.split(along_u) else {
                    return false;
                };
                pending.push((left, right, a, depth + 1));
                pending.push((left, right, b, depth + 1));
            }
        }
    }
    true
}

