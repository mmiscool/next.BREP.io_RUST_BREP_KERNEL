//! Regions from the mesh snapshot, in machine space and without the kernel:
//! the cross-section of the part at a Z, the projected outline of a face's
//! triangles, and the silhouette (projected union of every triangle).
//! Chaining of loose segments into loops is shared with the kernel-side
//! extraction.

use super::{boolean, crossings, BoolOp, Chain, Loop, PlanarError, Region, Segment, Vec2};
use crate::mesh::MeshSnapshot;
use crate::model::{Point2, Point3};
use crate::numeric::Tolerance;

/// Chain DIRECTED segments (each continues at its end) into closed loops.
/// Every segment must be used; an unmatched end is an [`PlanarError::OpenBoundary`].
pub fn chain_directed(segments: Vec<Segment>, tol: &Tolerance) -> Result<Vec<Loop>, PlanarError> {
    let n = segments.len();
    if n == 0 {
        return Ok(Vec::new());
    }
    // Merge endpoints within point_eps: ids over 2n points.
    let points: Vec<Point2> = segments.iter().flat_map(|s| [s.start, s.end]).collect();
    let vid = merge_points(&points, tol.point_eps);
    let mut outgoing: Vec<Vec<usize>> = vec![Vec::new(); 2 * n];
    for i in 0..n {
        outgoing[vid[2 * i]].push(i);
    }
    let mut used = vec![false; n];
    let mut loops = Vec::new();
    for first in 0..n {
        if used[first] {
            continue;
        }
        let start = vid[2 * first];
        let mut segs = Vec::new();
        let mut cur = first;
        loop {
            used[cur] = true;
            let mut s = segments[cur];
            // Snap to the merged vertex coordinates so the loop closes exactly.
            s.start = points[vid[2 * cur]];
            s.end = points[vid[2 * cur + 1]];
            segs.push(s);
            let v = vid[2 * cur + 1];
            if v == start {
                break;
            }
            let next = outgoing[v].iter().copied().find(|&k| !used[k]);
            match next {
                Some(k) => cur = k,
                None => return Err(PlanarError::OpenBoundary { at: points[2 * cur + 1] }),
            }
        }
        let mut l = Loop::new(segs);
        l.validate(tol)?;
        loops.push(l);
    }
    Ok(loops)
}

/// Chain UNDIRECTED segments (either end may continue, segments may be
/// reversed) into chains; closed chains are flagged. Every segment is used.
pub fn chain_undirected(segments: Vec<Segment>, tol: &Tolerance) -> Vec<Chain> {
    let n = segments.len();
    let points: Vec<Point2> = segments.iter().flat_map(|s| [s.start, s.end]).collect();
    let vid = merge_points(&points, tol.point_eps);
    let mut at: Vec<Vec<usize>> = vec![Vec::new(); 2 * n];
    for i in 0..n {
        at[vid[2 * i]].push(i);
        at[vid[2 * i + 1]].push(i);
    }
    let mut used = vec![false; n];
    let mut chains = Vec::new();
    for first in 0..n {
        if used[first] {
            continue;
        }
        used[first] = true;
        let mut segs = vec![segments[first]];
        let (mut head, mut tail) = (vid[2 * first], vid[2 * first + 1]);
        // Extend forward from the tail, then backward from the head.
        loop {
            let Some(&k) = at[tail].iter().find(|&&k| !used[k]) else { break };
            used[k] = true;
            let mut s = segments[k];
            if vid[2 * k] != tail {
                s = s.reversed();
                tail = vid[2 * k];
            } else {
                tail = vid[2 * k + 1];
            }
            s.start = segs.last().unwrap().end;
            segs.push(s);
        }
        loop {
            let Some(&k) = at[head].iter().find(|&&k| !used[k]) else { break };
            used[k] = true;
            let mut s = segments[k];
            if vid[2 * k + 1] != head {
                s = s.reversed();
                head = vid[2 * k + 1];
            } else {
                head = vid[2 * k];
            }
            s.end = segs[0].start;
            segs.insert(0, s);
        }
        let closed = segs.len() > 1 && head == tail;
        if closed {
            let last = segs.len() - 1;
            segs[0].start = segs[last].end;
        }
        chains.push(Chain { segments: segs, closed });
    }
    chains
}

/// Vertex ids for points, merged within `eps` (transitively), deterministic.
pub(crate) fn merge_points(points: &[Point2], eps: f64) -> Vec<usize> {
    let m = points.len();
    let mut parent: Vec<usize> = (0..m).collect();
    fn find(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut by_x: Vec<usize> = (0..m).collect();
    by_x.sort_by(|&a, &b| points[a].x.total_cmp(&points[b].x).then(a.cmp(&b)));
    for (k, &i) in by_x.iter().enumerate() {
        for &j in &by_x[k + 1..] {
            if points[j].x - points[i].x > eps {
                break;
            }
            if points[i].distance(points[j]) <= eps {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                if ri != rj {
                    let (lo, hi) = if ri < rj { (ri, rj) } else { (rj, ri) };
                    parent[hi] = lo;
                }
            }
        }
    }
    (0..m).map(|i| find(&mut parent, i)).collect()
}

/// Cross-section of the protected mesh at height `z`, restricted to the
/// faces for which `include(face index)` is true: closed loops with the
/// material on the left (outer loops counter-clockwise). Triangles lying in
/// the plane are ignored (a face exactly at `z` contributes its walls only).
pub fn section_at_z<F: Fn(u32) -> bool>(mesh: &MeshSnapshot, z: f64, include: F, tol: &Tolerance) -> Result<Region, PlanarError> {
    if !z.is_finite() {
        return Err(PlanarError::NonFinite { what: "section height".into() });
    }
    // The cut is taken just above `z`: a vertex is below it when it lies at
    // most `snap` above `z`. The classification is per vertex, so every
    // triangle sharing a vertex agrees and the crossings chain, also where a
    // tessellation leaves rounding noise on a height (a vendor STEP boss
    // whose wall's bottom vertices alternate between -6.0 and
    // -6.0000000000000036: cut at the lower value, half of them were above
    // and the section fell apart into separate bottom edges). Triangles are
    // never skipped by their normal: a nearly horizontal one that spans the
    // cut is cut like any other, and an exactly level one has every vertex
    // on one side.
    let snap = SECTION_SNAP * (1.0 + z.abs());
    let below = |p: &Point3| p.z <= z + snap;
    let mut segments: Vec<Segment> = Vec::new();
    for t in &mesh.triangles {
        if !include(t.face) || t.min_z() > z + snap || t.max_z() <= z + snap {
            continue;
        }
        let v = t.vertices();
        // Along the triangle's winding (its outward side on the left of each
        // edge seen from outside), the cut leaves the triangle where an edge
        // goes from above to below and enters it where one goes back up;
        // material lies on the left going from the first to the second.
        let mut down: Option<Point2> = None;
        let mut up: Option<Point2> = None;
        for k in 0..3 {
            let (a, b) = (v[k], v[(k + 1) % 3]);
            let (ba, bb) = (below(&a), below(&b));
            if ba == bb {
                continue;
            }
            let f = ((z - a.z) / (b.z - a.z)).clamp(0.0, 1.0);
            let c = a.lerp(b, f);
            let c = Point2::new(c.x, c.y);
            if ba {
                up = Some(c);
            } else {
                down = Some(c);
            }
        }
        let (Some(p), Some(q)) = (down, up) else { continue };
        if p.distance(q) <= tol.point_eps {
            continue;
        }
        segments.push(Segment::line(p, q));
    }
    let loops = chain_directed(segments, tol)?;
    Ok(Region { loops })
}

/// Relative height within which a mesh vertex counts as on a section's
/// plane (far above rounding noise, far below any machining tolerance).
const SECTION_SNAP: f64 = 1e-9;

/// Projected outline of a set of faces: the once-used edges of their
/// triangles, projected to XY, oriented with the face's upward side on the
/// left (downward faces are reversed). Returns the region and the Z range of
/// the triangles. Folded faces (a cylinder seen from above) are refused.
pub fn face_outline<F: Fn(u32) -> bool>(mesh: &MeshSnapshot, include: F, tol: &Tolerance) -> Result<(Region, [f64; 2]), PlanarError> {
    let mut edges: Vec<(Point3, Point3)> = Vec::new();
    let mut zmin = f64::INFINITY;
    let mut zmax = f64::NEG_INFINITY;
    let mut up = 0usize;
    let mut down = 0usize;
    for t in &mesh.triangles {
        if !include(t.face) {
            continue;
        }
        zmin = zmin.min(t.min_z());
        zmax = zmax.max(t.max_z());
        if tol.is_horizontal(t.normal) {
            continue; // vertical triangles have no projected area
        }
        if t.normal.z > 0.0 {
            up += 1;
        } else {
            down += 1;
        }
        let v = t.vertices();
        for k in 0..3 {
            edges.push((v[k], v[(k + 1) % 3]));
        }
    }
    if edges.is_empty() {
        return Err(PlanarError::InvalidInput("no triangles selected for the outline".into()));
    }
    if up > 0 && down > 0 {
        return Err(PlanarError::InvalidInput("the selected faces fold over in XY; use the silhouette".into()));
    }
    // Edges used twice (once each way) are interior; keep the singles.
    let key = |p: Point3| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
    let mut keyed: Vec<((u64, u64, u64), (u64, u64, u64), usize)> = edges.iter().enumerate().map(|(i, (a, b))| (key(*a), key(*b), i)).collect();
    let mut reversed: Vec<((u64, u64, u64), (u64, u64, u64))> = edges.iter().map(|(a, b)| (key(*b), key(*a))).collect();
    keyed.sort();
    reversed.sort();
    let mut segments = Vec::new();
    for (ka, kb, i) in keyed {
        if reversed.binary_search(&(ka, kb)).is_err() {
            let (a, b) = edges[i];
            let s = Segment::line(Point2::new(a.x, a.y), Point2::new(b.x, b.y));
            if s.start.distance(s.end) > tol.point_eps {
                segments.push(if down > 0 { s.reversed() } else { s });
            }
        }
    }
    let loops = chain_directed(segments, tol)?;
    Ok((Region { loops }, [zmin, zmax]))
}

/// The silhouette: the union of every non-vertical triangle's projection.
/// Expensive for large meshes (one n-ary union); callers cache it.
pub fn silhouette<F: Fn(u32) -> bool>(mesh: &MeshSnapshot, include: F, tol: &Tolerance) -> Result<Region, PlanarError> {
    let mut loops = Vec::new();
    for t in &mesh.triangles {
        if !include(t.face) || tol.is_horizontal(t.normal) {
            continue;
        }
        let v = t.vertices();
        let pts: Vec<Point2> = v.iter().map(|p| Point2::new(p.x, p.y)).collect();
        let mut l = Loop::polygon(&pts);
        if !l.is_ccw() {
            l = l.reversed();
        }
        if l.signed_area() > tol.area_eps && l.segments.iter().all(|s| s.start.distance(s.end) > tol.point_eps) {
            loops.push(l);
        }
    }
    if loops.is_empty() {
        return Ok(Region::empty());
    }
    boolean(&Region { loops }, &Region::empty(), BoolOp::Union, tol)
}

/// Winding number of `p` about `loops` alone (helper for consumers).
pub fn winding_of(loops: &[Loop], p: Point2, tol: &Tolerance) -> i32 {
    let canon: Vec<Segment> = loops.iter().flat_map(|l| l.segments.iter().flat_map(|s| s.canonical(tol))).collect();
    crossings(canon.iter(), p, true)
}
