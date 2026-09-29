use super::*;
use super::bridge::point_in_polygon;

pub(in crate::watertight_tessellation) fn certified_planar_trim(
    v: &[FaceVertex],
    ts: &[[usize; 3]],
    outer: &[FaceVertex],
    holes: &[Vec<FaceVertex>],
) -> bool {
    if ts.is_empty() {
        return false;
    }
    let expected =
        signed_area(outer).abs() - holes.iter().map(|h| signed_area(h).abs()).sum::<f64>();
    if !expected.is_finite() || expected <= 0.0 {
        return false;
    }
    let (lo, hi) = outer.iter().chain(holes.iter().flatten()).fold(
        ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
        |(mut lo, mut hi), p| {
            for k in 0..2 {
                lo[k] = lo[k].min(p.uv[k]);
                hi[k] = hi[k].max(p.uv[k]);
            }
            (lo, hi)
        },
    );
    let scale = ((hi[0] - lo[0]) * (hi[1] - lo[1])).abs().max(expected);
    type K = (u64, u64);
    let key = |p: &FaceVertex| (p.uv[0].to_bits(), p.uv[1].to_bits());
    let ek = |a: K, b: K| if a <= b { (a, b) } else { (b, a) };
    let mut boundary = HashSet::new();
    let mut directed_boundary = HashSet::new();
    for ring in std::iter::once(outer).chain(holes.iter().map(Vec::as_slice)) {
        for i in 0..ring.len() {
            let edge = (key(&ring[i]), key(&ring[(i + 1) % ring.len()]));
            if !boundary.insert(ek(edge.0, edge.1)) || !directed_boundary.insert(edge) {
                return false;
            }
        }
    }
    let mut area = 0.0;
    let mut edges = HashMap::new();
    let mut directed_edges = HashMap::new();
    let mut unique = HashSet::new();
    for &[a, b, c] in ts {
        if a >= v.len() || b >= v.len() || c >= v.len() {
            return false;
        }
        let uv = [v[a].uv, v[b].uv, v[c].uv];
        let signed = triangle_area(uv[0], uv[1], uv[2]);
        if !signed.is_finite() || signed <= 0.0 {
            return false;
        }
        let ar = signed;
        area += ar;
        if ar > 64.0 * f64::EPSILON * scale {
            let p = [
                (uv[0][0] + uv[1][0] + uv[2][0]) / 3.0,
                (uv[0][1] + uv[1][1] + uv[2][1]) / 3.0,
            ];
            if !point_in_polygon(outer, p) || holes.iter().any(|h| point_in_polygon(h, p)) {
                return false;
            }
        }
        let mut t = [key(&v[a]), key(&v[b]), key(&v[c])];
        if t[0] == t[1] || t[1] == t[2] || t[2] == t[0] {
            return false;
        }
        t.sort_unstable();
        if !unique.insert(t) {
            return false;
        }
        for (x, y) in [(a, b), (b, c), (c, a)] {
            *edges.entry(ek(key(&v[x]), key(&v[y]))).or_insert(0usize) += 1;
            *directed_edges
                .entry((key(&v[x]), key(&v[y])))
                .or_insert(0usize) += 1;
        }
    }
    let roundoff = 64.0 * f64::EPSILON * scale * (ts.len() + boundary.len()) as f64;
    (area - expected).abs() <= roundoff
        && (area - expected).abs() <= expected * 1e-10
        && boundary.iter().all(|e| edges.get(e).copied() == Some(1))
        && edges
            .iter()
            .all(|(e, &n)| n == if boundary.contains(e) { 1 } else { 2 })
        && directed_boundary.iter().all(|&(a, b)| {
            directed_edges.get(&(a, b)).copied() == Some(1)
                && directed_edges.get(&(b, a)).copied().unwrap_or(0) == 0
        })
        && directed_edges.iter().all(|(&(a, b), &n)| {
            n == 1
                && (directed_boundary.contains(&(a, b))
                    || directed_edges.get(&(b, a)).copied() == Some(1))
        })
}

pub(in crate::watertight_tessellation) fn valid_planar_pslg(outer: &[FaceVertex], holes: &[Vec<FaceVertex>]) -> bool {
    let rings = std::iter::once(outer)
        .chain(holes.iter().map(Vec::as_slice))
        .collect::<Vec<_>>();
    if signed_area(outer) <= 0.0 || holes.iter().any(|hole| signed_area(hole) >= 0.0) {
        return false;
    }
    let (lo, hi) = rings.iter().flat_map(|ring| ring.iter()).fold(
        ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]),
        |(mut lo, mut hi), vertex| {
            for axis in 0..2 {
                lo[axis] = lo[axis].min(vertex.uv[axis]);
                hi[axis] = hi[axis].max(vertex.uv[axis]);
            }
            (lo, hi)
        },
    );
    let span = (hi[0] - lo[0]).abs().max((hi[1] - lo[1]).abs()).max(1e-12);
    let epsilon = 256.0 * f64::EPSILON * span * span;
    let on_segment = |point: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        triangle_area(a, b, point).abs() <= epsilon
            && point[0] >= a[0].min(b[0]) - epsilon
            && point[0] <= a[0].max(b[0]) + epsilon
            && point[1] >= a[1].min(b[1]) - epsilon
            && point[1] <= a[1].max(b[1]) + epsilon
    };
    for (ring_a, a) in rings.iter().enumerate() {
        for edge_a in 0..a.len() {
            let (a0, a1) = (a[edge_a].uv, a[(edge_a + 1) % a.len()].uv);
            if a0 == a1 {
                return false;
            }
            for (ring_b, b) in rings.iter().enumerate().skip(ring_a) {
                for edge_b in 0..b.len() {
                    if ring_a == ring_b
                        && (edge_a == edge_b
                            || (edge_a + 1) % a.len() == edge_b
                            || (edge_b + 1) % b.len() == edge_a)
                    {
                        continue;
                    }
                    let (b0, b1) = (b[edge_b].uv, b[(edge_b + 1) % b.len()].uv);
                    let cross = segments_properly_cross(a0, a1, b0, b1, epsilon);
                    let touch = on_segment(a0, b0, b1)
                        || on_segment(a1, b0, b1)
                        || on_segment(b0, a0, a1)
                        || on_segment(b1, a0, a1);
                    if cross || touch {
                        if std::env::var("BREP_DEBUG_PSLG").is_ok() {
                            eprintln!(
                                "[pslg] reject {} ringA={ring_a}(e{edge_a}) ringB={ring_b}(e{edge_b}) a=[{:.6},{:.6}]->[{:.6},{:.6}] b=[{:.6},{:.6}]->[{:.6},{:.6}]",
                                if cross { "CROSS" } else { "TOUCH" },
                                a0[0], a0[1], a1[0], a1[1], b0[0], b0[1], b1[0], b1[1]
                            );
                        }
                        return false;
                    }
                }
            }
        }
    }
    for hole in holes {
        if !point_in_polygon(outer, hole[0].uv) {
            return false;
        }
    }
    for a in 0..holes.len() {
        for b in a + 1..holes.len() {
            if point_in_polygon(&holes[a], holes[b][0].uv)
                || point_in_polygon(&holes[b], holes[a][0].uv)
            {
                return false;
            }
        }
    }
    true
}

pub(in crate::watertight_tessellation) struct DirectPlanar {
    pub(in crate::watertight_tessellation) vertices: Vec<FaceVertex>,
    pub(in crate::watertight_tessellation) triangles: Vec<[usize; 3]>,
}
#[derive(Clone, Copy)]
struct IndexedPoint {
    position: spade::Point2<f64>,
    authored: usize,
}
impl spade::HasPosition for IndexedPoint {
    type Scalar = f64;
    fn position(&self) -> spade::Point2<f64> {
        self.position
    }
}

pub(in crate::watertight_tessellation) fn direct_planar_cdt(
    mut outer: Vec<FaceVertex>,
    mut holes: Vec<Vec<FaceVertex>>,
) -> Option<DirectPlanar> {
    use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
    if outer.len() < 3 || holes.is_empty() || holes.iter().any(|h| h.len() < 3) {
        return None;
    }
    let canon = |r: &mut Vec<FaceVertex>| {
        let s = (0..r.len()).min_by(|&a, &b| {
            r[a].uv[0]
                .total_cmp(&r[b].uv[0])
                .then_with(|| r[a].uv[1].total_cmp(&r[b].uv[1]))
        })?;
        r.rotate_left(s);
        Some(())
    };
    canon(&mut outer)?;
    for h in &mut holes {
        canon(h)?;
    }
    holes.sort_by(|a, b| {
        a[0].uv[0]
            .total_cmp(&b[0].uv[0])
            .then_with(|| a[0].uv[1].total_cmp(&b[0].uv[1]))
            .then_with(|| a.len().cmp(&b.len()))
    });
    if !valid_planar_pslg(&outer, &holes) {
        return None;
    }
    let mut vertices = outer.clone();
    let mut ranges = vec![0..outer.len()];
    for h in &holes {
        let s = vertices.len();
        vertices.extend_from_slice(h);
        ranges.push(s..vertices.len());
    }
    let mut seen = HashSet::new();
    if vertices
        .iter()
        .any(|p| !seen.insert((p.uv[0].to_bits(), p.uv[1].to_bits())))
    {
        return None;
    }
    let first = 0;
    let second = (1..vertices.len()).max_by(|&a, &b| {
        let d = |i: usize| {
            let x = vertices[i].uv[0] - vertices[0].uv[0];
            let y = vertices[i].uv[1] - vertices[0].uv[1];
            x * x + y * y
        };
        d(a).total_cmp(&d(b))
    })?;
    let third = (0..vertices.len())
        .filter(|&i| i != first && i != second)
        .max_by(|&a, &b| {
            triangle_area(vertices[first].uv, vertices[second].uv, vertices[a].uv)
                .abs()
                .total_cmp(
                    &triangle_area(vertices[first].uv, vertices[second].uv, vertices[b].uv).abs(),
                )
        })?;
    let p = vertices[first].uv;
    let q = vertices[second].uv;
    let r = vertices[third].uv;
    let (a, b, c, d) = (q[0] - p[0], q[1] - p[1], r[0] - p[0], r[1] - p[1]);
    let det = a * d - b * c;
    if det == 0.0 {
        return None;
    }
    let o = vertices[first].position;
    let d1 = vertices[second].position.sub(o);
    let d2 = vertices[third].position.sub(o);
    let su = d1.scale(d).sub(d2.scale(b)).scale(1.0 / det);
    let sv = d2.scale(a).sub(d1.scale(c)).scale(1.0 / det);
    let ps = vertices
        .iter()
        .map(|x| x.position.sub(o).length())
        .fold(1e-6f64, f64::max);
    // The caller only sets `planar_trim` (and thus reaches here) for a face whose
    // analytic surface IS a Plane, so the uv->3d map is affine by construction;
    // the only deviation from the 3-point affine fit is ordinary boundary-sample
    // fit noise (edge points land a surface-fit tolerance off the nominal plane).
    // The historic 1e-9 relative gate demanded picometer planarity and so kept a
    // KNOWN-BROKEN ear-clip trim for any planar face with normal micron-scale fit
    // noise (this path runs only after the legacy trim already FAILED
    // certification). A standard 1e-6 relative planarity tolerance still rejects a
    // genuinely non-planar boundary, and the CDT output is re-certified by
    // `certified_planar_trim` below regardless, so relaxing this can only replace
    // a broken triangulation with a certified-correct one — never regress a face.
    if vertices.iter().any(|x| {
        o.add(su.scale(x.uv[0] - p[0]))
            .add(sv.scale(x.uv[1] - p[1]))
            .sub(x.position)
            .length()
            > ps * 1e-6
    }) {
        return None;
    }
    let mut cdt = ConstrainedDelaunayTriangulation::<IndexedPoint>::new();
    let mut handles = Vec::new();
    for (i, p) in vertices.iter().enumerate() {
        let h = cdt
            .insert(IndexedPoint {
                position: Point2::new(p.uv[0], p.uv[1]),
                authored: i,
            })
            .ok()?;
        if h.index() != i {
            return None;
        }
        handles.push(h);
    }
    if cdt.num_vertices() != vertices.len() {
        return None;
    }
    let mut boundary = HashSet::new();
    let mut directed_boundary = HashSet::new();
    for range in &ranges {
        for i in range.clone() {
            let n = if i + 1 == range.end {
                range.start
            } else {
                i + 1
            };
            let made = cdt.try_add_constraint(handles[i], handles[n]);
            if made.len() > 1 || !cdt.exists_constraint(handles[i], handles[n]) {
                return None;
            }
            boundary.insert((i.min(n), i.max(n)));
            directed_boundary.insert((i, n));
        }
    }
    let actual = cdt
        .undirected_edges()
        .filter(|e| e.data().is_constraint_edge())
        .map(|e| {
            let [a, b] = e.vertices();
            let (a, b) = (a.data().authored, b.data().authored);
            (a.min(b), a.max(b))
        })
        .collect::<HashSet<_>>();
    if actual != boundary {
        return None;
    }
    let mut triangles = Vec::new();
    for f in cdt.inner_faces() {
        let mut t = f.vertices().map(|x| x.data().authored);
        let uv = t.map(|i| vertices[i].uv);
        let m = [
            (uv[0][0] + uv[1][0] + uv[2][0]) / 3.0,
            (uv[0][1] + uv[1][1] + uv[2][1]) / 3.0,
        ];
        if !point_in_polygon(&outer, m) || holes.iter().any(|h| point_in_polygon(h, m)) {
            continue;
        }
        let ar = triangle_area(uv[0], uv[1], uv[2]);
        if ar == 0.0 {
            return None;
        }
        if ar < 0.0 {
            t.swap(1, 2);
        }
        let minimum = (0..3).min_by_key(|&index| t[index]).unwrap_or(0);
        t.rotate_left(minimum);
        triangles.push(t);
    }
    triangles.sort_unstable();
    if !certified_planar_trim(&vertices, &triangles, &outer, &holes) {
        return None;
    }
    let mut used = vec![false; vertices.len()];
    let mut dir = HashMap::new();
    let mut uniq = HashSet::new();
    for &[a, b, c] in &triangles {
        for i in [a, b, c] {
            used[i] = true;
        }
        let mut k = [a, b, c];
        k.sort_unstable();
        if !uniq.insert(k) {
            return None;
        }
        for e in [(a, b), (b, c), (c, a)] {
            *dir.entry(e).or_insert(0usize) += 1;
        }
    }
    if used.iter().any(|&x| !x) {
        return None;
    }
    for &(a, b) in &directed_boundary {
        if dir.get(&(a, b)).copied() != Some(1) || dir.get(&(b, a)).copied().unwrap_or(0) != 0 {
            return None;
        }
    }
    for (&(a, b), &n) in &dir {
        if n != 1 || (!directed_boundary.contains(&(a, b)) && dir.get(&(b, a)).copied() != Some(1))
        {
            return None;
        }
    }
    Some(DirectPlanar {
        vertices,
        triangles,
    })
}

/// The result of a constrained-Delaunay triangulation of a face's trim region:
/// the boundary vertices (outer loop then each hole, concatenated) and the
/// triangles indexing them, plus the constraint (loop) edges that refinement
/// must never split.
pub(in crate::watertight_tessellation) struct CdtMesh {
    pub(in crate::watertight_tessellation) vertices: Vec<FaceVertex>,
    pub(in crate::watertight_tessellation) triangles: Vec<[usize; 3]>,
    pub(in crate::watertight_tessellation) boundary_pairs: HashSet<(usize, usize)>,
}

/// Constrained-Delaunay triangulation of ANY (planar or curved) face's trim
/// region, from its outer loop + hole loops in UV. Generalises
/// [`direct_planar_cdt`]: it does NOT assume the surface is planar, and it feeds
/// the Delaunay predicate METRIC-SCALED uv (`scale = [√E, √G]`, the per-axis
/// first-fundamental-form magnitudes) so the triangulation is well-shaped on the
/// real surface rather than in a distorted parameter box — the fix for the
/// extreme-aspect / non-uniform-parameterization folds the ear-clip path
/// produces. Holes are constraints (no bridging), the region of a simple PSLG
/// triangulates without stalls or fans, and the result of a simple constrained
/// domain is always a valid, orientable triangulation (no junction odd cycle).
///
/// Boundary vertices are inserted verbatim as constraints, so they are never
/// moved or dropped — shared-edge coincidence (watertightness) is preserved
/// exactly. `refine_interior` (which works on triangle edges, skipping
/// `boundary_pairs`) then densifies the interior for curvature.
///
/// Returns `None` — the caller falls back to bridge+ear-clip — when the loops do
/// not form a simple PSLG or the Delaunay does not reproduce every constraint
/// edge, so a face this can't safely handle keeps its existing path bit-for-bit.
pub(in crate::watertight_tessellation) fn surface_cdt(outer: &[FaceVertex], holes: &[Vec<FaceVertex>], scale: [f64; 2]) -> Option<CdtMesh> {
    use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
    let decline = |why: &str| -> Option<CdtMesh> {
        if std::env::var("BREP_DEBUG_CDT").is_ok() {
            eprintln!("[cdt] decline: {why} (outer={}, holes={})", outer.len(), holes.len());
        }
        None
    };
    if outer.len() < 3 || holes.iter().any(|h| h.len() < 3) {
        return decline("degenerate loop");
    }
    if !valid_planar_pslg(outer, holes) {
        return decline("not a simple PSLG");
    }
    // Concatenate loops into one vertex list; record each loop's index range so
    // the constraint edges (and the caller's boundary_pairs) follow the loops.
    let mut vertices: Vec<FaceVertex> = outer.to_vec();
    let mut ranges = vec![0..outer.len()];
    for h in holes {
        let s = vertices.len();
        vertices.extend_from_slice(h);
        ranges.push(s..vertices.len());
    }
    // Reject exact uv-duplicates (a degenerate PSLG spade can't constrain).
    let mut seen = HashSet::new();
    if vertices
        .iter()
        .any(|p| !seen.insert((p.uv[0].to_bits(), p.uv[1].to_bits())))
    {
        return None;
    }
    let [su, sv] = [scale[0].max(1e-12), scale[1].max(1e-12)];
    let mut cdt = ConstrainedDelaunayTriangulation::<IndexedPoint>::new();
    let mut handles = Vec::with_capacity(vertices.len());
    for (i, p) in vertices.iter().enumerate() {
        // Metric-scaled uv: Delaunay quality is judged in ~unrolled 3D lengths.
        let h = cdt
            .insert(IndexedPoint {
                position: Point2::new(p.uv[0] * su, p.uv[1] * sv),
                authored: i,
            })
            .ok()?;
        if h.index() != i {
            return decline("coincident insert");
        }
        handles.push(h);
    }
    if cdt.num_vertices() != vertices.len() {
        return None;
    }
    let mut boundary_pairs: HashSet<(usize, usize)> = HashSet::new();
    let mut directed_boundary: HashSet<(usize, usize)> = HashSet::new();
    for range in &ranges {
        for i in range.clone() {
            let n = if i + 1 == range.end { range.start } else { i + 1 };
            let made = cdt.try_add_constraint(handles[i], handles[n]);
            if made.len() > 1 || !cdt.exists_constraint(handles[i], handles[n]) {
                return decline("constraint had to be split");
            }
            boundary_pairs.insert((i.min(n), i.max(n)));
            directed_boundary.insert((i, n));
        }
    }
    // Every authored constraint edge must survive as a CDT constraint (no more,
    // no fewer) — else the region is not the one we intended.
    let actual = cdt
        .undirected_edges()
        .filter(|e| e.data().is_constraint_edge())
        .map(|e| {
            let [a, b] = e.vertices();
            let (a, b) = (a.data().authored, b.data().authored);
            (a.min(b), a.max(b))
        })
        .collect::<HashSet<_>>();
    if actual != boundary_pairs {
        return decline("constraint set mismatch");
    }
    // Keep only faces inside the outer loop and outside every hole, determined
    // TOPOLOGICALLY by flooding from the outer (infinite) face and counting
    // CONSTRAINT-edge crossings: a face is in the trimmed region iff that parity
    // is odd (inside the outer, then back out of each hole). This is exact and
    // orientation-free, replacing the horizontal-ray parity test which grazes
    // the near-collinear edges of an axis-aligned rim/hole on a thin
    // metric-scaled wall and mis-counts (ABC 00000333 rings: it kept slivers
    // straddling the rim, so the manifold check rejected an otherwise clean CDT).
    let mut adjacency: HashMap<usize, [(Option<usize>, bool); 3]> = HashMap::new();
    let mut queue: std::collections::VecDeque<(usize, bool)> = std::collections::VecDeque::new();
    for face in cdt.inner_faces() {
        let face_index = face.fix().index();
        let mut entry = [(None, false); 3];
        for (slot, edge) in face.adjacent_edges().into_iter().enumerate() {
            let is_constraint = edge.is_constraint_edge();
            let neighbor = edge.rev().face();
            if neighbor.is_outer() {
                // Crossing from the outer face (parity 0) into this face.
                queue.push_back((face_index, is_constraint));
            } else if let Some(inner) = neighbor.as_inner() {
                entry[slot] = (Some(inner.index()), is_constraint);
            }
        }
        adjacency.insert(face_index, entry);
    }
    let mut inside: HashMap<usize, bool> = HashMap::new();
    while let Some((face_index, parity)) = queue.pop_front() {
        if inside.contains_key(&face_index) {
            continue;
        }
        inside.insert(face_index, parity);
        for &(neighbor, is_constraint) in &adjacency[&face_index] {
            if let Some(neighbor) = neighbor {
                if !inside.contains_key(&neighbor) {
                    queue.push_back((neighbor, parity ^ is_constraint));
                }
            }
        }
    }
    let mut triangles: Vec<[usize; 3]> = Vec::new();
    for f in cdt.inner_faces() {
        if !inside.get(&f.fix().index()).copied().unwrap_or(false) {
            continue;
        }
        let mut t = f.vertices().map(|x| x.data().authored);
        let uv = t.map(|i| vertices[i].uv);
        // Keep zero-area slivers (like the ear-clip path) so the manifold edge
        // count stays exact; the emission filter drops point-coincident ones
        // downstream. Orient CCW in uv (the same_sense emission convention).
        if triangle_area(uv[0], uv[1], uv[2]) < 0.0 {
            t.swap(1, 2);
        }
        triangles.push(t);
    }
    if triangles.len() < ranges.iter().map(|r| r.len()).sum::<usize>().saturating_sub(2) {
        // A valid trim triangulation has at least (boundary_verts - 2 - 2*holes)
        // triangles; far fewer means the inside filter rejected the real region
        // (e.g. an inverted outer loop). Decline and let ear-clip try.
        return decline("too few interior triangles");
    }
    // Manifold check: every kept triangle's directed edges are unique, every
    // boundary edge is used from ONE side only, and every interior edge is shared
    // by an opposite twin. A SLIT loop (an edge traversed both ways) or a filter
    // that kept an outside triangle both violate this — decline and let the
    // ear-clip path (which tolerates degenerate slit trims via slivers) own them.
    let mut dir: HashMap<(usize, usize), usize> = HashMap::new();
    for &[a, b, c] in &triangles {
        for e in [(a, b), (b, c), (c, a)] {
            *dir.entry(e).or_insert(0) += 1;
        }
    }
    for (&(a, b), &n) in &dir {
        if n != 1 {
            return decline("overlapping triangles (directed edge reused)");
        }
        let opp = dir.get(&(b, a)).copied().unwrap_or(0);
        if directed_boundary.contains(&(a, b)) {
            if opp != 0 {
                if std::env::var("BREP_DEBUG_CDT").is_ok() {
                    eprintln!(
                        "[cdt]   both-sides edge a={a} b={b} uv_a=({:.5},{:.5}) uv_b=({:.5},{:.5})",
                        vertices[a].uv[0], vertices[a].uv[1], vertices[b].uv[0], vertices[b].uv[1]
                    );
                }
                return decline("boundary edge used from both sides");
            }
        } else if opp != 1 {
            if std::env::var("BREP_DEBUG_CDT").is_ok() {
                eprintln!(
                    "[cdt]   no-twin edge a={a} b={b} uv_a=({:.5},{:.5}) uv_b=({:.5},{:.5})",
                    vertices[a].uv[0], vertices[a].uv[1], vertices[b].uv[0], vertices[b].uv[1]
                );
            }
            return decline("interior edge not shared by a twin");
        }
    }
    Some(CdtMesh {
        vertices,
        triangles,
        boundary_pairs,
    })
}

