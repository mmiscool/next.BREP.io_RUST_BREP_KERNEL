//! A ruled band: a cylinder or cone wall between two rims, whole (wrapping the period) or cut short
//! by a boolean, with no holes. Its straight rulings make the wall's sag across a triangle depend only on the
//! triangle's spread in u, and the rims' shared samples are already spaced for the chord. So the
//! band's polygon in the chart, which is monotone in u (one rim along increasing u, the seam at the
//! far end, the other rim back, the seam at the near end), is triangulated as a monotone polygon
//! from its own stations, every one of them kept (seam samples included), with no interior
//! stations, no refinement and no flips. Every triangle is then checked against the chord and the
//! normal-turn bars (the same check the NURBS rectangle uses); a band that misses either, or whose
//! polygon is not of that shape, declines atomically to the general road.
//!
//! The general road takes such a wall to the constrained Delaunay triangulation and then refines
//! across it: a large wall at a fine chord has rims of thousands of stations each, and the
//! refinement of the thin triangles between them is what never finished.
use super::*;

/// `BREP_TESS_RULED_BAND` (default off; `=1` meshes these walls as bands). Off by default
/// while it leaves a pinch open: on the body a marched Viviani section builds (a ball bored
/// by a cylinder tangent inside it), the band takes both cylinder walls and two edges at the
/// tangent point stay unmatched, where the general road's walls close.
pub fn ruled_band_enabled() -> bool {
    crate::switch!("BREP_TESS_RULED_BAND", false)
}

/// Triangulate a polygon monotone in u (counter-clockwise in the chart, vertices ordered
/// lexicographically by (u, v) along each chain) by the stack sweep. Returns counter-clockwise
/// triangles of indices into `polygon`, or `None` when the polygon is not monotone in u.
pub(super) fn monotone_triangles(polygon: &[[f64; 2]]) -> Option<Vec<[usize; 3]>> {
    let n = polygon.len();
    if n < 3 {
        return None;
    }
    let key = |i: usize| (polygon[i][0], polygon[i][1]);
    let less = |a: usize, b: usize| key(a).0 < key(b).0 || (key(a).0 == key(b).0 && key(a).1 < key(b).1);
    let first = (0..n).fold(0, |m, i| if less(i, m) { i } else { m });
    let last = (0..n).fold(0, |m, i| if less(m, i) { i } else { m });
    // Counter-clockwise from the first vertex runs along the lower chain to the last, then back
    // along the upper one. Each chain must advance strictly in the lexicographic order.
    let mut lower = vec![first];
    let mut i = first;
    while i != last {
        let next = (i + 1) % n;
        if !less(i, next) {
            return None;
        }
        lower.push(next);
        i = next;
    }
    let mut upper = vec![last];
    while i != first {
        let next = (i + 1) % n;
        if !less(next, i) {
            return None;
        }
        upper.push(next);
        i = next;
    }
    // Merge both chains into one sweep order, each vertex tagged with its chain (true = lower).
    upper.reverse(); // now first .. last along the upper chain
    let mut order: Vec<(usize, bool)> = Vec::with_capacity(n);
    let (mut a, mut b) = (0, 1); // lower[0] == upper[0] == first; take it once
    order.push((first, true));
    while a + 1 < lower.len() - 1 || b < upper.len() - 1 {
        let take_lower = b >= upper.len() - 1 || (a + 1 < lower.len() - 1 && less(lower[a + 1], upper[b]));
        if take_lower {
            a += 1;
            order.push((lower[a], true));
        } else {
            order.push((upper[b], false));
            b += 1;
        }
    }
    order.push((last, true));
    let cross = |o: usize, p: usize, q: usize| {
        let (o, p, q) = (polygon[o], polygon[p], polygon[q]);
        (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    };
    let mut out = Vec::with_capacity(n - 2);
    let mut emit = |p: usize, q: usize, r: usize| {
        let area = cross(p, q, r);
        if area > 0.0 {
            out.push([p, q, r]);
        } else {
            out.push([p, r, q]);
        }
    };
    let mut stack: Vec<(usize, bool)> = vec![order[0], order[1]];
    for &(v, side) in &order[2..n - 1] {
        let top = *stack.last().unwrap();
        if side != top.1 {
            // Opposite chains: fan v to every stacked vertex.
            for pair in stack.windows(2) {
                emit(v, pair[0].0, pair[1].0);
            }
            stack = vec![top, (v, side)];
        } else {
            let mut popped = stack.pop().unwrap();
            while let Some(&next) = stack.last() {
                // The diagonal from v to `next` lies inside when v, popped, next turn the way the
                // chain bends inward: left for the lower chain, right for the upper.
                let turn = cross(next.0, popped.0, v);
                let inside = if side { turn > 0.0 } else { turn < 0.0 };
                if !inside {
                    break;
                }
                emit(v, popped.0, next.0);
                popped = stack.pop().unwrap();
            }
            stack.push(popped);
            stack.push((v, side));
        }
    }
    let end = order[n - 1].0;
    for pair in stack.windows(2) {
        emit(end, pair[0].0, pair[1].0);
    }
    if out.len() != n - 2 || out.iter().any(|&[p, q, r]| cross(p, q, r) <= 0.0) {
        return None;
    }
    Some(out)
}

/// The shortest boundary segment a band may have, as a fraction of its median segment.
const CROWDED: f64 = 1e-4;

/// The shortest segment of `polygon`'s boundary over its median segment, in space.
fn crowding(polygon: &[FaceVertex]) -> f64 {
    let n = polygon.len();
    let mut lengths: Vec<f64> = (0..n).map(|i| polygon[(i + 1) % n].position.sub(polygon[i].position).length()).collect();
    lengths.sort_unstable_by(f64::total_cmp);
    let median = lengths[n / 2];
    if median > 0.0 { lengths[0] / median } else { 0.0 }
}

/// The sharpest corner a band may have, in degrees.
const CUSP_DEGREES: f64 = 10.0;

/// The smallest angle, in degrees, between the two boundary segments at any vertex of
/// `polygon`, measured in space (a seam corner reads 90).
fn sharpest_corner_degrees(polygon: &[FaceVertex]) -> f64 {
    let n = polygon.len();
    (0..n)
        .filter_map(|i| {
            let v = polygon[i].position;
            let a = polygon[(i + n - 1) % n].position.sub(v);
            let b = polygon[(i + 1) % n].position.sub(v);
            let (la, lb) = (a.length(), b.length());
            (la > 0.0 && lb > 0.0).then(|| (a.dot(b) / (la * lb)).clamp(-1.0, 1.0).acos().to_degrees())
        })
        .fold(180.0, f64::min)
}

/// Whether two vertices of `polygon` lie on one chart point, to 1e-9 of the domain's span.
fn pinched(polygon: &[[f64; 2]], domain: [f64; 4]) -> bool {
    let tolerance = 1e-9 * (domain[1] - domain[0]).abs().max((domain[3] - domain[2]).abs()).max(1e-300);
    let mut order: Vec<usize> = (0..polygon.len()).collect();
    order.sort_unstable_by(|&a, &b| polygon[a][0].total_cmp(&polygon[b][0]));
    for (k, &a) in order.iter().enumerate() {
        for &b in &order[k + 1..] {
            if polygon[b][0] - polygon[a][0] > tolerance {
                break;
            }
            if (polygon[b][1] - polygon[a][1]).abs() <= tolerance {
                return true;
            }
        }
    }
    false
}

/// Whether every triangle lies within `bar` of the surface and turns with it: the surface's normal
/// at each corner on the triangle's side, and its three edge middles and its centroid within `bar`
/// of the surface (their distance to it, by projection). On a ruled wall between rims of unequal
/// radii (a cone), a triangle between two rulings lies in their plane and so on the surface but for
/// the rims' own sag, though the surface's point at its parameter middle lies far from its chord:
/// distance to the surface, not to that point, is the reading.
fn band_within(face: &FaceRecord, vertices: &[FaceVertex], triangles: &[[usize; 3]], domain: [f64; 4], bar: f64)
    -> Result<bool, String> {
    let normals: Vec<Vec3> = vertices.iter().map(|v| face_normal_at(face, v.uv, domain)).collect::<Result<_, _>>()?;
    let sense = if face.same_sense { 1.0 } else { -1.0 };
    for &[a, b, c] in triangles {
        let (pa, pb, pc) = (vertices[a].position, vertices[b].position, vertices[c].position);
        let geometric = pb.sub(pa).cross(pc.sub(pa));
        if [a, b, c].iter().any(|&i| geometric.dot(normals[i]) * sense <= 0.0) {
            return Ok(false);
        }
        let centroid = pa.add(pb).add(pc).scale(1.0 / 3.0);
        for q in [pa.add(pb).scale(0.5), pb.add(pc).scale(0.5), pc.add(pa).scale(0.5), centroid] {
            if crate::project_point_to_surface(&face.surface, q)?.distance > bar {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// Split every interior edge (one not on the boundary, whose vertices are the first `boundary`)
/// that sags past `bar` at its middle, the new vertex on the surface at the edge's middle in the
/// chart, its two triangles made four; repeated while any interior edge sags past the bar, up to
/// half as many triangles again. Boundary edges are never split: they are the shared edges' samples.
fn split_sagging_diagonals(face: &FaceRecord, vertices: &mut Vec<FaceVertex>, triangles: &mut Vec<[usize; 3]>,
    boundary: usize, bar: f64) -> Result<(), String> {
    let cap = triangles.len() + triangles.len() / 2;
    let on_boundary = |i: usize, j: usize| {
        i < boundary && j < boundary && ((i + 1) % boundary == j || (j + 1) % boundary == i)
    };
    for _round in 0..64 {
        // Edge -> the triangles holding it.
        let mut holders: std::collections::HashMap<(usize, usize), Vec<usize>> = std::collections::HashMap::new();
        for (index, t) in triangles.iter().enumerate() {
            for (i, j) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                holders.entry((i.min(j), i.max(j))).or_default().push(index);
            }
        }
        // Sagging interior edges, each taken only when none of its triangles is taken already.
        let mut taken = vec![false; triangles.len()];
        let mut splits: Vec<((usize, usize), usize)> = Vec::new();
        let mut keys: Vec<&(usize, usize)> = holders.keys().collect();
        keys.sort_unstable();
        for &&(i, j) in &keys {
            if on_boundary(i, j) || holders[&(i, j)].iter().any(|&t| taken[t]) {
                continue;
            }
            let mid = [0.5 * (vertices[i].uv[0] + vertices[j].uv[0]), 0.5 * (vertices[i].uv[1] + vertices[j].uv[1])];
            let p = face.surface.evaluate(mid[0], mid[1])?;
            let middle = vertices[i].position.add(vertices[j].position).scale(0.5);
            if crate::project_point_to_surface(&face.surface, middle)?.distance > bar {
                for &t in &holders[&(i, j)] {
                    taken[t] = true;
                }
                let m = vertices.len();
                vertices.push(FaceVertex { uv: mid, position: p });
                splits.push(((i, j), m));
            }
        }
        if splits.is_empty() {
            return Ok(());
        }
        if triangles.len() + 2 * splits.len() > cap {
            return Ok(());
        }
        let mut at: std::collections::HashMap<usize, ((usize, usize), usize)> = std::collections::HashMap::new();
        for &(edge, m) in &splits {
            for &t in &holders[&edge] {
                at.insert(t, (edge, m));
            }
        }
        let mut next = Vec::with_capacity(triangles.len() + 2 * splits.len());
        for (index, &t) in triangles.iter().enumerate() {
            let Some(&((i, j), m)) = at.get(&index) else {
                next.push(t);
                continue;
            };
            // the triangle holding the edge i-j (either way round) becomes two, at m
            let k = (0..3).find(|&k| (t[k] == i && t[(k + 1) % 3] == j) || (t[k] == j && t[(k + 1) % 3] == i)).unwrap_or(0);
            let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            next.push([a, m, c]);
            next.push([m, b, c]);
        }
        *triangles = next;
    }
    Ok(())
}

fn decline(face_id: u32, why: &str) -> Result<bool, String> {
    if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
        eprintln!("[ruled-band] face {face_id}: declined: {why}");
    }
    Ok(false)
}

/// Mesh a ruled band face from its polygon; `Ok(false)` leaves it to the general road.
pub(super) fn tessellate_ruled_band(
    face: &FaceRecord,
    polygon: &[FaceVertex],
    domain: [f64; 4],
    closed_u: bool,
    chord_tolerance: f64,
    face_id: u32,
    mesh: &mut Mesh,
) -> Result<bool, String> {
    let _ = closed_u; // a partial wall (cut short by a boolean) is as monotone as a whole one
    if !(ruled_band_enabled() || ruled_band_forced())
        || !matches!(face.surface.analytic(), Some(crate::AnalyticSurface::RuledRevolution { .. }))
        || polygon.len() < 4
    {
        return Ok(false);
    }
    if !polygon_is_simple(polygon) {
        return decline(face_id, "polygon not simple");
    }
    let mut ccw: Vec<FaceVertex> = polygon.to_vec();
    if signed_area(&ccw) < 0.0 {
        ccw.reverse();
    }
    let uv: Vec<[f64; 2]> = ccw.iter().map(|p| p.uv).collect();
    // A boundary that passes through one chart point twice (a pinch: a section curve
    // crossing itself on the wall) is left to the general road, whose corner ears there
    // are the ones the neighbouring face clips from the same samples, which the pinch
    // pass in `orient` pairs up and removes.
    if pinched(&uv, domain) {
        return decline(face_id, "boundary passes through one point twice");
    }
    // A corner sharper than `CUSP_DEGREES` (a boolean cut tangent to the wall, which
    // reaches its vertex at zero angle: a pinch's sliver) is left to the general road too:
    // it clips that corner's ear from the corner's own two neighbours, as the face across
    // the sliver does, and the pinch pass removes the pair.
    let sharpest = sharpest_corner_degrees(&ccw);
    if sharpest < CUSP_DEGREES {
        return decline(face_id, &format!("a corner of {sharpest:.2} degrees (a cusp)"));
    }
    // Boundary samples crowded onto one point (an edge sampled down into a pinch or a
    // tangent touch, its segments a ten-thousandth of the face's usual ones) are left to
    // the general road too: the faces meeting there close only as that road meshes them.
    let crowding = crowding(&ccw);
    if crowding < CROWDED {
        return decline(face_id, &format!("boundary samples crowd a point (shortest segment {crowding:.1e} of the median)"));
    }
    let Some(triangles) = monotone_triangles(&uv) else {
        return decline(face_id, "polygon not monotone in u");
    };
    // The rims are the shared edges' own samples, spaced to the chord as a target, not a bound: a
    // segment may sag a little past it, and every route keeps those segments as they are. On a
    // ruled wall a diagonal's sag is set by its spread in u alone, like a rim segment's, so the
    // triangles are held to the chord or to the rims' own worst sag, whichever is larger: the band
    // is never worse than its boundary. Rims sagging past twice the chord are left to the general
    // road, which may add stations along them.
    let mut rim_sag = 0.0_f64;
    for (a, b) in ccw.iter().zip(ccw.iter().cycle().skip(1)) {
        let p = face.surface.evaluate(0.5 * (a.uv[0] + b.uv[0]), 0.5 * (a.uv[1] + b.uv[1]))?;
        rim_sag = rim_sag.max(deflection_to_chord(p, a.position, b.position));
    }
    if rim_sag > 2.0 * chord_tolerance {
        return decline(face_id, &format!("rim sag {rim_sag:.3e} past twice the chord"));
    }
    let bar = chord_tolerance.max(rim_sag);
    // A diagonal between the two chains may span a little more of u than any rim segment (their
    // stations interleave) and so sag a little more: such diagonals are split at their middle.
    let boundary = ccw.len();
    let (mut vertices, mut triangles) = (ccw.clone(), triangles);
    split_sagging_diagonals(face, &mut vertices, &mut triangles, boundary, bar)?;
    if !band_within(face, &vertices, &triangles, domain, bar)? {
        return decline(face_id, "a triangle leaves the surface by more than the chord, or turns against it");
    }
    let ccw = vertices;
    if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
        eprintln!("[ruled-band] face {face_id}: {} triangles ({} added stations), sharpest corner {sharpest:.2} degrees, crowding {crowding:.1e}", triangles.len(), ccw.len() - boundary);
    }
    let base = (mesh.positions.len() / 3) as u32;
    for vertex in &ccw {
        let normal = face_normal_at(face, vertex.uv, domain)?;
        mesh.positions.extend([vertex.position.x, vertex.position.y, vertex.position.z]);
        mesh.normals.extend([normal.x, normal.y, normal.z]);
    }
    for [a, mut b, mut c] in triangles {
        if !face.same_sense {
            std::mem::swap(&mut b, &mut c);
        }
        mesh.indices.extend([base + a as u32, base + b as u32, base + c as u32]);
        mesh.face_ids.push(face_id);
    }
    Ok(true)
}

