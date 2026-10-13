//! Tensor-grid display lane for a single iso-parametric NURBS rectangle.
//!
//! Knot-span second partials choose the grid spacing; midpoint/centroid chord
//! and normal-turn checks certify the emitted triangles. The grid ends INSIDE
//! the trim. Four monotone strips join it to the original shared edge samples,
//! consuming each boundary segment exactly once without inserting stations.
//! No station seeding, ear clipping or Lawson flips are needed. Failure of any
//! shape, sample or tolerance check declines atomically to the existing path.
use super::*;
use std::cell::Cell;

const RECTANGLE_DEFAULT: bool = true;
const MAX_LINES: usize = 512;
const MAX_VERTICES: usize = 100_000;
thread_local! {
    static RECTANGLE_OVERRIDE: Cell<Option<bool>> = const { Cell::new(None) };
}
fn enabled() -> bool {
    RECTANGLE_OVERRIDE.with(|s| s.get()).unwrap_or_else(|| {
        #[cfg(not(target_arch = "wasm32"))]
        {
            std::env::var("BREP_TESS_NURBS_RECTANGLE")
                .map_or(RECTANGLE_DEFAULT, |v| v == "1" || v == "on")
        }
        #[cfg(target_arch = "wasm32")]
        {
            RECTANGLE_DEFAULT
        }
    })
}

fn supported_carrier(face: &FaceRecord) -> bool {
    match face.surface.analytic() {
        None => true,
        Some(crate::AnalyticSurface::Revolution { generatrix, .. }) => generatrix.degree > 1,
        _ => false,
    }
}

/// Shared sampling must see single-span inflections before either seam mate
/// consumes the chain. Only candidate NURBS rectangles request this guard;
/// the established road and primitive fixtures retain their samples.
pub(super) fn rectangle_edge_guard(face: &FaceRecord) -> bool {
    if !enabled()
        || !supported_carrier(face)
        || face.loops.len() != 1
        || face.loops[0].coedges.len() != 4
    {
        return false;
    }
    face.loops[0].coedges.iter().all(|c| {
        let Ok([t0, t1]) = c.pcurve.domain() else {
            return false;
        };
        let (Ok(a), Ok(b)) = (c.pcurve.evaluate(t0), c.pcurve.evaluate(t1)) else {
            return false;
        };
        let eps = 1e-12 * (a.x - b.x).abs().max((a.y - b.y).abs());
        let fixed = if (a.x - b.x).abs() <= eps {
            0
        } else if (a.y - b.y).abs() <= eps {
            1
        } else {
            return false;
        };
        c.pcurve.control_points.iter().all(|p| {
            p.w.is_finite()
                && p.w > 0.
                && if fixed == 0 {
                    (p.x / p.w - a.x).abs() <= eps
                } else {
                    (p.y / p.w - a.y).abs() <= eps
                }
        })
    })
}

fn decline(face_id: u32, reason: &str) -> Result<bool, String> {
    if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
        eprintln!("[nurbs-rectangle] face {face_id}: declined: {reason}");
    }
    tess_profile(|p| p.rectangle_declines += 1);
    Ok(false)
}

/// Ordered boundary sides, CCW in the rectangle's UV domain. Positive rational
/// pcurve controls must all lie on their iso line; checking just the endpoints
/// would admit a bowed trim and fill material outside it.
fn rectangle_sides(
    face: &FaceRecord,
    samples: &HashMap<u64, EdgeSamples>,
    chord: f64,
) -> Result<Option<([f64; 4], [Vec<FaceVertex>; 4])>, String> {
    let mut sides: [Vec<FaceVertex>; 4] = std::array::from_fn(|_| Vec::new());
    let mut ends = Vec::new();
    for c in &face.loops[0].coedges {
        let [t0, t1] = c.pcurve.domain()?;
        let a = c.pcurve.evaluate(t0)?;
        let b = c.pcurve.evaluate(t1)?;
        ends.extend([[a.x, a.y], [b.x, b.y]]);
    }
    let bounds = ends.iter().fold(
        [
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        ],
        |mut b, p| {
            b[0] = b[0].min(p[0]);
            b[1] = b[1].max(p[0]);
            b[2] = b[2].min(p[1]);
            b[3] = b[3].max(p[1]);
            b
        },
    );
    let [u0, u1, v0, v1] = bounds;
    if !(u1 > u0 && v1 > v0) {
        return Ok(None);
    }
    let domain_u = KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?.domain();
    let domain_v = KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?.domain();
    if u0 < domain_u[0] || u1 > domain_u[1] || v0 < domain_v[0] || v1 > domain_v[1] {
        return Ok(None);
    }
    let corners = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
    let eps = [1e-12 * (u1 - u0), 1e-12 * (v1 - v0)];
    let near =
        |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).abs() <= eps[0] && (a[1] - b[1]).abs() <= eps[1];
    for c in &face.loops[0].coedges {
        let [t0, t1] = c.pcurve.domain()?;
        let a = c.pcurve.evaluate(t0)?;
        let b = c.pcurve.evaluate(t1)?;
        let Some(side) = (0..4).find(|&i| {
            (near([a.x, a.y], corners[i]) && near([b.x, b.y], corners[(i + 1) % 4]))
                || (near([b.x, b.y], corners[i]) && near([a.x, a.y], corners[(i + 1) % 4]))
        }) else {
            return Ok(None);
        };
        if !sides[side].is_empty() {
            return Ok(None);
        }
        let varying = if side % 2 == 0 { 0 } else { 1 };
        let fixed = 1 - varying;
        for cp in &c.pcurve.control_points {
            if !cp.w.is_finite() || cp.w <= 0. {
                return Ok(None);
            }
            let q = [cp.x / cp.w, cp.y / cp.w];
            if (q[fixed] - corners[side][fixed]).abs() > eps[fixed]
                || q[varying]
                    < corners[side][varying].min(corners[(side + 1) % 4][varying]) - eps[varying]
                || q[varying]
                    > corners[side][varying].max(corners[(side + 1) % 4][varying]) + eps[varying]
            {
                return Ok(None);
            }
        }
        let Some(edge) = samples.get(&c.edge_id) else {
            return Ok(None);
        };
        if edge.fractions.len() < 2
            || edge.positions.len() != edge.fractions.len()
            || edge.worst_sag > chord
        {
            return Ok(None);
        }
        for (&f, &position) in edge.fractions.iter().zip(&edge.positions) {
            let f = if c.forward { f } else { 1. - f };
            let uv = c.pcurve.evaluate(t0 + f * (t1 - t0))?;
            let uv = [uv.x, uv.y];
            if ![position.x, position.y, position.z]
                .iter()
                .all(|x| x.is_finite())
                || !f.is_finite()
                || !uv.iter().all(|x| x.is_finite())
                || face.surface.evaluate(uv[0], uv[1])?.sub(position).length() > chord * 1e-5
            {
                return Ok(None);
            }
            sides[side].push(FaceVertex { uv, position });
        }
        let sign = if side < 2 { 1. } else { -1. };
        // Reverse the supplied chain as a whole when needed. Sorting it would
        // silently change the shared segments of a backtracking iso trim.
        if near(sides[side][0].uv, corners[(side + 1) % 4]) {
            sides[side].reverse();
        }
        if !near(sides[side][0].uv, corners[side])
            || !near(sides[side].last().unwrap().uv, corners[(side + 1) % 4])
            || sides[side].windows(2).any(|p| {
                sign * (p[1].uv[varying] - p[0].uv[varying]) <= 0.
                    || p[1].position.sub(p[0].position).length() == 0.
            })
        {
            return Ok(None);
        }
    }
    // The endpoints are the shared topological vertices, not independently
    // evaluated surface corners. Require exact agreement around the loop.
    for i in 0..4 {
        let a = sides[i].last().unwrap().position;
        let b = sides[(i + 1) % 4][0].position;
        if [a.x.to_bits(), a.y.to_bits(), a.z.to_bits()]
            != [b.x.to_bits(), b.y.to_bits(), b.z.to_bits()]
        {
            return Ok(None);
        }
    }
    Ok(Some((bounds, sides)))
}

fn knot_lines(knots: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    let mut lines = vec![lo];
    lines.extend(knots.iter().copied().filter(|&x| x > lo && x < hi));
    lines.push(hi);
    lines.dedup_by(|a, b| *a == *b);
    lines
}

/// Sample each knot span separately so an inflected multi-span carrier cannot
/// disappear at the whole-domain midpoint. Bound the quadratic diagonal by
/// allocating a quarter chord to each pure partial and half to the mixed one.
/// The bounds are estimates, not interval proofs; emitted triangles are checked
/// independently below, and a failed check refines or declines.
fn curvature_grid(
    face: &FaceRecord,
    bounds: [f64; 4],
    chord: f64,
) -> Result<Option<(Vec<f64>, Vec<f64>)>, String> {
    let [u0, u1, v0, v1] = bounds;
    let us = knot_lines(&face.surface.knots_u, u0, u1);
    let vs = knot_lines(&face.surface.knots_v, v0, v1);
    let mut nu = vec![3usize; us.len() - 1];
    let mut nv = vec![3usize; vs.len() - 1];
    for (i, u) in us.windows(2).enumerate() {
        for (j, v) in vs.windows(2).enumerate() {
            let du = u[1] - u[0];
            let dv = v[1] - v[0];
            for a in [0.0, 0.5, 1.0] {
                for b in [0.0, 0.5, 1.0] {
                    // Evaluate endpoints just inside this span (one-sided at knots).
                    let a: f64 = a;
                    let b: f64 = b;
                    let d = face.surface.derivatives_small(
                        u[0] + a.clamp(1e-8, 1. - 1e-8) * du,
                        v[0] + b.clamp(1e-8, 1. - 1e-8) * dv,
                        2,
                    )?;
                    let cross = d[1][0].cross(d[0][1]);
                    let length = cross.length();
                    if !length.is_finite() || length <= 1e-14 {
                        return Ok(None);
                    }
                    let normal = cross.scale(1. / length);
                    let normal_rate =
                        |x: Vec3| x.sub(normal.scale(x.dot(normal))).length() / length;
                    let turn_u = normal_rate(d[2][0].cross(d[0][1]).add(d[1][0].cross(d[1][1])));
                    let turn_v = normal_rate(d[1][1].cross(d[0][1]).add(d[1][0].cross(d[0][2])));
                    let su = d[2][0].length() + d[1][1].length() * dv / du;
                    let sv = d[0][2].length() + d[1][1].length() * du / dv;
                    let count = |width: f64, second: f64, turn: f64| {
                        (width * (second / (2. * chord)).sqrt())
                            .max(width * turn / ANGULAR_TOL)
                            .ceil() as usize
                    };
                    nu[i] = nu[i].max(count(du, su, turn_u));
                    nv[j] = nv[j].max(count(dv, sv, turn_v));
                }
            }
        }
    }
    let expand = |spans: &[f64], counts: &[usize]| -> Option<Vec<f64>> {
        if counts.iter().any(|&n| n >= MAX_LINES) || counts.iter().sum::<usize>() >= MAX_LINES {
            return None;
        }
        let mut lines = Vec::new();
        for (p, &n) in spans.windows(2).zip(counts) {
            for k in 0..n {
                lines.push(p[0] + (p[1] - p[0]) * k as f64 / n as f64);
            }
        }
        lines.push(*spans.last().unwrap());
        Some(lines)
    };
    Ok(expand(&us, &nu).zip(expand(&vs, &nv)))
}

/// A monotone triangulation of the strip between two station chains. Each
/// chain is in the same direction; `outer` walks CCW around the rectangle.
/// Prefer the shorter next 3D diagonal so a coarse boundary interval fans to
/// nearby interior stations, rather than keeping its corner until the next
/// boundary parameter and making an unnecessarily long curved diagonal.
fn strip(
    outer: &[usize],
    inner: &[usize],
    vertices: &[FaceVertex],
    triangles: &mut Vec<[usize; 3]>,
) {
    let distance = |a: usize, b: usize| {
        vertices[a]
            .position
            .sub(vertices[b].position)
            .length_squared()
    };
    let (mut i, mut j) = (0, 0);
    while i + 1 < outer.len() || j + 1 < inner.len() {
        if j + 1 == inner.len()
            || (i + 1 < outer.len()
                && distance(outer[i + 1], inner[j]) <= distance(outer[i], inner[j + 1]))
        {
            triangles.push([outer[i], outer[i + 1], inner[j]]);
            i += 1;
        } else {
            triangles.push([outer[i], inner[j + 1], inner[j]]);
            j += 1;
        }
    }
}

fn build_grid(
    face: &FaceRecord,
    sides: &[Vec<FaceVertex>; 4],
    us: &[f64],
    vs: &[f64],
) -> Result<(Vec<FaceVertex>, Vec<[usize; 3]>, usize), String> {
    let mut vertices = Vec::new();
    let mut boundary: [Vec<usize>; 4] = std::array::from_fn(|_| Vec::new());
    // Share each corner between its two strips; retain every original segment.
    let starts: [usize; 4] = std::array::from_fn(|i| sides[..i].iter().map(|s| s.len() - 1).sum());
    for i in 0..4 {
        for p in &sides[i][..sides[i].len() - 1] {
            boundary[i].push(vertices.len());
            vertices.push(*p);
        }
        boundary[i].push(starts[(i + 1) % 4]);
    }
    let boundary_count = vertices.len();
    let nu = us.len() - 2;
    let nv = vs.len() - 2;
    for &v in &vs[1..vs.len() - 1] {
        for &u in &us[1..us.len() - 1] {
            vertices.push(FaceVertex {
                uv: [u, v],
                position: face.surface.evaluate(u, v)?,
            });
        }
    }
    let at = |i: usize, j: usize| boundary_count + j * nu + i;
    let mut triangles = Vec::new();
    for j in 0..nv - 1 {
        for i in 0..nu - 1 {
            triangles.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            triangles.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }
    let inner = [
        (0..nu).map(|i| at(i, 0)).collect::<Vec<_>>(),
        (0..nv).map(|j| at(nu - 1, j)).collect(),
        (0..nu).rev().map(|i| at(i, nv - 1)).collect(),
        (0..nv).rev().map(|j| at(0, j)).collect(),
    ];
    for i in 0..4 {
        strip(&boundary[i], &inner[i], &vertices, &mut triangles);
    }
    Ok((vertices, triangles, boundary_count))
}

fn triangle_distance(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f64 {
    let n = b.sub(a).cross(c.sub(a));
    let nn = n.length_squared();
    if nn <= 0. {
        return f64::INFINITY;
    }
    let q = p.sub(n.scale(p.sub(a).dot(n) / nn));
    if [
        b.sub(a).cross(q.sub(a)).dot(n),
        c.sub(b).cross(q.sub(b)).dot(n),
        a.sub(c).cross(q.sub(c)).dot(n),
    ]
    .iter()
    .all(|&x| x >= 0.)
    {
        p.sub(q).length()
    } else {
        deflection_to_chord(p, a, b)
            .min(deflection_to_chord(p, b, c))
            .min(deflection_to_chord(p, c, a))
    }
}

#[derive(Clone, Copy)]
enum GridCheck {
    Passed,
    RefineGrid,
    RefineRim,
}

fn within_tolerance(
    face: &FaceRecord,
    vertices: &[FaceVertex],
    triangles: &[[usize; 3]],
    boundary_count: usize,
    bounds: [f64; 4],
    chord: f64,
) -> Result<GridCheck, String> {
    let normals: Vec<Vec3> = vertices
        .iter()
        .map(|p| face_normal_at(face, p.uv, bounds))
        .collect::<Result<_, _>>()?;
    for &[a, b, c] in triangles {
        let failure = if [a, b, c].iter().any(|&i| i < boundary_count) {
            GridCheck::RefineRim
        } else {
            GridCheck::RefineGrid
        };
        let points = [vertices[a], vertices[b], vertices[c]];
        let geometric = points[1]
            .position
            .sub(points[0].position)
            .cross(points[2].position.sub(points[0].position));
        let sense = if face.same_sense { 1. } else { -1. };
        if [a, b, c]
            .iter()
            .any(|&i| geometric.dot(normals[i]) * sense <= 0.)
        {
            if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
                eprintln!(
                    "[nurbs-rectangle] normal failure: {:?}",
                    points.map(|p| p.uv)
                );
            }
            return Ok(failure);
        }
        for (i, j) in [(a, b), (b, c), (c, a)] {
            // Both boundary endpoints: the shared sampler owns the angular
            // decision. All other edges use the generic interior angle bar.
            let uv = [
                (vertices[i].uv[0] + vertices[j].uv[0]) * 0.5,
                (vertices[i].uv[1] + vertices[j].uv[1]) * 0.5,
            ];
            let p = face.surface.evaluate(uv[0], uv[1])?;
            let sag = deflection_to_chord(p, vertices[i].position, vertices[j].position);
            if sag > chord
                || ((i >= boundary_count || j >= boundary_count)
                    && sag >= ANGULAR_SAG_FLOOR * chord
                    && normals[i].dot(normals[j]) < ANGULAR_TOL.cos())
            {
                if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
                    eprintln!("[nurbs-rectangle] edge failure: {:?} -> {:?}, sag={sag}, chord={chord}, cos={}", vertices[i].uv, vertices[j].uv, normals[i].dot(normals[j]));
                }
                return Ok(failure);
            }
        }
        // Interior probes in addition to edge midpoints catch a mixed-partial
        // saddle whose perimeter happens to be straight.
        for weights in [
            [1. / 3.; 3],
            [0.5, 0.25, 0.25],
            [0.25, 0.5, 0.25],
            [0.25, 0.25, 0.5],
        ] {
            let uv: [f64; 2] =
                std::array::from_fn(|k| points.iter().zip(weights).map(|(p, w)| p.uv[k] * w).sum());
            let p = face.surface.evaluate(uv[0], uv[1])?;
            if triangle_distance(
                p,
                points[0].position,
                points[1].position,
                points[2].position,
            ) > chord
            {
                if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
                    eprintln!(
                        "[nurbs-rectangle] interior failure: {:?}, probe={uv:?}",
                        points.map(|p| p.uv)
                    );
                }
                return Ok(failure);
            }
        }
    }
    Ok(GridCheck::Passed)
}

pub(super) fn tessellate_rectangle_watertight(
    face: &FaceRecord,
    samples: &HashMap<u64, EdgeSamples>,
    chord: f64,
    face_id: u32,
    mesh: &mut Mesh,
) -> Result<bool, String> {
    if !enabled() {
        return Ok(false);
    }
    // Revolution recognizes a general spline carrier, evaluated by its NURBS
    // net. Primitive planes/cylinders/cones/spheres/tori keep their own lanes.
    if !supported_carrier(face) {
        return Ok(false);
    }
    if face.loops.len() != 1
        || face.loops[0].coedges.len() != 4
        || face
            .surface
            .control_points
            .iter()
            .flatten()
            .any(|p| !p.w.is_finite() || p.w <= 0.)
    {
        return Ok(false);
    }
    let _timer = stage_timer(Stage::Rectangle);
    let Some((bounds, sides)) = rectangle_sides(face, samples, chord)? else {
        return decline(
            face_id,
            "not a nondegenerate iso rectangle with shared samples",
        );
    };
    let Some((mut us, mut vs)) = curvature_grid(face, bounds, chord)? else {
        return decline(face_id, "singular or over-budget curvature grid");
    };
    for pass in 0..4 {
        if us.len() > MAX_LINES
            || vs.len() > MAX_LINES
            || (us.len() - 2) * (vs.len() - 2) > MAX_VERTICES
        {
            return decline(face_id, "grid budget exhausted");
        }
        let (vertices, triangles, boundary_count) = build_grid(face, &sides, &us, &vs)?;
        let check = within_tolerance(face, &vertices, &triangles, boundary_count, bounds, chord)?;
        if matches!(check, GridCheck::Passed) {
            // Commit only a completely checked grid. A failed candidate has
            // never written into the caller's mesh, including its normals.
            let normals: Vec<_> = vertices
                .iter()
                .map(|p| face_normal_at(face, p.uv, bounds))
                .collect::<Result<_, _>>()?;
            let base = (mesh.positions.len() / 3) as u32;
            for (p, n) in vertices.iter().zip(normals) {
                mesh.positions
                    .extend([p.position.x, p.position.y, p.position.z]);
                mesh.normals.extend([n.x, n.y, n.z]);
            }
            for [a, mut b, mut c] in triangles {
                if !face.same_sense {
                    std::mem::swap(&mut b, &mut c);
                }
                mesh.indices
                    .extend([base + a as u32, base + b as u32, base + c as u32]);
                mesh.face_ids.push(face_id);
            }
            tess_profile(|p| {
                p.rectangle_faces += 1;
                p.rectangle_vertices += vertices.len() as u64;
            });
            if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
                eprintln!(
                    "[nurbs-rectangle] face {face_id}: {}x{} lines, pass={pass}",
                    us.len(),
                    vs.len()
                );
            }
            return Ok(true);
        }
        let bisect = |lines: &[f64]| {
            let mut out = Vec::with_capacity(2 * lines.len() - 1);
            for pair in lines.windows(2) {
                out.extend([pair[0], 0.5 * (pair[0] + pair[1])]);
            }
            out.push(*lines.last().unwrap());
            out
        };
        // A strip attached to a coarse straight rim may need to sit very close
        // to that rim. Preserve the already-checked interior spacing and add
        // only the two narrower boundary bands, rather than multiplying every
        // interior cell by four for a failure confined to the strips.
        let inset = |lines: &[f64]| {
            let mut out = Vec::with_capacity(lines.len() + 2);
            out.extend([lines[0], 0.5 * (lines[0] + lines[1])]);
            out.extend_from_slice(&lines[1..lines.len() - 1]);
            out.extend([
                0.5 * (lines[lines.len() - 2] + lines[lines.len() - 1]),
                *lines.last().unwrap(),
            ]);
            out
        };
        match check {
            GridCheck::RefineRim => {
                us = inset(&us);
                vs = inset(&vs);
            }
            GridCheck::RefineGrid => {
                us = bisect(&us);
                vs = bisect(&vs);
            }
            GridCheck::Passed => unreachable!(),
        }
    }
    decline(face_id, "triangle chord/normal check did not converge")
}

