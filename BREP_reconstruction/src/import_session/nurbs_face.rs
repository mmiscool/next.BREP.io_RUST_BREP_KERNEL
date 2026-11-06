//! A region the reconstruction left as triangles, rebuilt as ONE NURBS face
//! of the solid — exactly bounded by the edges its neighbours already have.
//!
//! The fitted surface must contain the region's boundary EXACTLY, or the
//! shell's edges would not lie on it and the solid could not validate. The
//! region's boundary in the built body is a chain of straight edges between
//! mesh vertices. Split at four corners, the four sides are polylines; a
//! cubic B-spline curve with a triple knot at every polyline vertex IS that
//! polyline, parametrized linearly on each segment. So the patch is a bicubic
//! tensor surface on the union of the sides' knots (triple interior knots),
//! whose four boundary rows of control points are the four sides: every
//! boundary edge of the region lies on the patch's domain edge, and its pcurve
//! is a straight parameter line. The interior control points are a smoothed
//! least-squares fit to the region's vertices, parametrized by a Tutte
//! (barycentric) embedding of the region on the unit square — valid for any
//! disk-shaped region, so the parametrization never folds.
//!
//! The patch replaces the region's facet faces in the solid; its interior
//! edges and vertices go with them. Nothing else in the solid changes. The
//! result must pass [`BrepSolid::validate`] like every other body, and is
//! refused with the reason otherwise.

use brep_kernel::{BrepSolid, CoedgeRecord, FaceRecord, LoopRecord, NurbsSurface, Vec3, Vec4};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

/// One region to rebuild as a NURBS face.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct NurbsFaceChoice {
    /// The region's source triangles (indices into the welded mesh).
    pub triangles: Vec<u32>,
    /// Largest vertex distance the patch may have from the mesh, mm.
    pub tolerance: f64,
}

/// What became of one [`NurbsFaceChoice`].
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct NurbsFaceOutcome {
    pub applied: bool,
    /// The patch's largest vertex deviation (applied), mm.
    pub max_deviation: Option<f64>,
    /// Control net size (applied).
    pub control_points: Option<[usize; 2]>,
    /// Why it was refused, in words.
    pub reason: Option<String>,
}

impl NurbsFaceOutcome {
    fn refused(reason: impl Into<String>) -> Self {
        Self { applied: false, max_deviation: None, control_points: None, reason: Some(reason.into()) }
    }
}

fn v3(p: &[f64]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

/// Distance from `p` to triangle `t`.
fn point_triangle_distance(p: Vec3, t: [Vec3; 3]) -> f64 {
    let [a, b, c] = t;
    let n = b.sub(a).cross(c.sub(a));
    let area2 = n.length();
    if area2 > 0.0 {
        let n = n.scale(1.0 / area2);
        let q = p.sub(n.scale(p.sub(a).dot(n)));
        let inside = [(a, b), (b, c), (c, a)].iter().all(|&(u, v)| v.sub(u).cross(q.sub(u)).dot(n) >= -1e-12 * area2);
        if inside {
            return p.sub(q).length();
        }
    }
    [(a, b), (b, c), (c, a)].iter().map(|&(u, v)| p.distance_to_segment(u, v, 0.0)).fold(f64::INFINITY, f64::min)
}

/// A coedge's (start, end) solid vertex ids.
fn coedge_ends(solid: &BrepSolid, coedge: &CoedgeRecord) -> Option<(u64, u64)> {
    let edge = solid.edges.iter().find(|e| e.id == coedge.edge_id)?;
    Some(if coedge.forward { (edge.start_vertex_id, edge.end_vertex_id) } else { (edge.end_vertex_id, edge.start_vertex_id) })
}

/// Rebuild each choice's region as one NURBS face of `solid` (in place),
/// returning one outcome per choice. A refused choice leaves the solid as it
/// was.
pub fn apply_nurbs_faces(
    solid: &mut BrepSolid,
    positions: &[f64],
    indices: &[u32],
    choices: &[NurbsFaceChoice],
) -> Vec<NurbsFaceOutcome> {
    choices
        .iter()
        .map(|choice| {
            let mut trial = solid.clone();
            match replace_region(&mut trial, positions, indices, choice) {
                Ok(outcome) => {
                    let issues = trial.validate();
                    if issues.is_empty() {
                        *solid = trial;
                        outcome
                    } else {
                        NurbsFaceOutcome::refused(format!(
                            "the solid with the NURBS face does not validate: {}",
                            issues.iter().take(2).map(|issue| format!("{issue:?}")).collect::<Vec<_>>().join("; ")
                        ))
                    }
                }
                Err(reason) => NurbsFaceOutcome::refused(reason),
            }
        })
        .collect()
}

fn replace_region(
    solid: &mut BrepSolid,
    positions: &[f64],
    indices: &[u32],
    choice: &NurbsFaceChoice,
) -> Result<NurbsFaceOutcome, String> {
    if !(choice.tolerance.is_finite() && choice.tolerance > 0.0) {
        return Err("the tolerance must be a positive distance".into());
    }
    let vertex_count = positions.len() / 3;
    let triangles: Vec<[u32; 3]> = choice
        .triangles
        .iter()
        .map(|&t| {
            let t = t as usize;
            if 3 * t + 2 >= indices.len() {
                return Err(format!("triangle {t} is outside the mesh"));
            }
            let tri = [indices[3 * t], indices[3 * t + 1], indices[3 * t + 2]];
            if tri.iter().any(|&v| v as usize >= vertex_count) {
                return Err(format!("triangle {t} names a vertex outside the mesh"));
            }
            Ok(tri)
        })
        .collect::<Result<_, _>>()?;
    if triangles.is_empty() {
        return Err("the region has no triangles".into());
    }
    let tri_points: Vec<[Vec3; 3]> = triangles.iter().map(|t| t.map(|v| v3(&positions[3 * v as usize..]))).collect();
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in positions.chunks_exact(3) {
        low = Vec3::new(low.x.min(p[0]), low.y.min(p[1]), low.z.min(p[2]));
        high = Vec3::new(high.x.max(p[0]), high.y.max(p[1]), high.z.max(p[2]));
    }
    let radius = (high.sub(low).length() * 1e-9).max(1e-12);
    // On the region: within a weld radius of one of its triangles.
    let on_region = |p: Vec3| tri_points.iter().any(|t| point_triangle_distance(p, *t) <= radius * 1e3);

    // The region's faces in the body: planar facets lying on its triangles
    // (the builder may have cut a triangle where it meets a fitted wall, so
    // a facet need not be a source triangle — but it lies on one).
    if solid.shells.len() != 1 {
        return Err(format!("the body has {} shells; a NURBS face is placed on single-shell bodies", solid.shells.len()));
    }
    let shell = &solid.shells[0];
    let point_of = |id: u64| solid.vertices.iter().find(|v| v.id == id).map(|v| v.point);
    let region_faces: Vec<usize> = shell
        .faces
        .iter()
        .enumerate()
        .filter(|(_, face)| {
            if face.surface.degree_u != 1 || face.surface.degree_v != 1 {
                return false;
            }
            let points: Vec<Vec3> = face
                .loops
                .iter()
                .flat_map(|l| &l.coedges)
                .filter_map(|c| coedge_ends(solid, c))
                .filter_map(|(a, _)| point_of(a))
                .collect();
            if points.is_empty() || !points.iter().all(|&p| on_region(p)) {
                return false;
            }
            let centre = points.iter().fold(Vec3::new(0.0, 0.0, 0.0), |acc, &p| acc.add(p)).scale(1.0 / points.len() as f64);
            on_region(centre)
        })
        .map(|(index, _)| index)
        .collect();
    if region_faces.is_empty() {
        return Err("the region's triangles are not faces of the built body (it was fitted, or merged with a neighbour)".into());
    }
    let region_face_set: HashSet<usize> = region_faces.iter().copied().collect();
    // The region's vertices and edges in the body.
    let mut region_edges: HashMap<u64, (u64, u64)> = HashMap::new();
    for &i in &region_faces {
        for coedge in shell.faces[i].loops.iter().flat_map(|l| &l.coedges) {
            let (a, b) = coedge_ends(solid, coedge).ok_or("a coedge names a missing edge")?;
            region_edges.insert(coedge.edge_id, (a.min(b), a.max(b)));
        }
    }
    let region_vertices: BTreeSet<u64> = region_edges.values().flat_map(|&(a, b)| [a, b]).collect();
    // Disk check: V - E + F == 1.
    if region_vertices.len() as i64 - region_edges.len() as i64 + region_faces.len() as i64 != 1 {
        return Err("the region is not a single disk-shaped patch (it has a hole or a handle); one NURBS face needs one boundary loop".into());
    }
    // Edge use: an edge used once by the region's faces is boundary.
    let mut uses: HashMap<u64, usize> = HashMap::new();
    for &i in &region_faces {
        for coedge in shell.faces[i].loops.iter().flat_map(|l| &l.coedges) {
            *uses.entry(coedge.edge_id).or_default() += 1;
        }
    }
    // Boundary coedges, chained by start vertex.
    let mut next: HashMap<u64, CoedgeRecord> = HashMap::new();
    for &i in &region_faces {
        for coedge in shell.faces[i].loops.iter().flat_map(|l| &l.coedges) {
            if uses[&coedge.edge_id] == 1 {
                let (a, _) = coedge_ends(solid, coedge).ok_or("a boundary coedge names a missing edge")?;
                if next.insert(a, coedge.clone()).is_some() {
                    return Err("the region's boundary touches itself at a vertex; one NURBS face needs a simple boundary loop".into());
                }
            }
        }
    }
    let Some(&start) = next.keys().min() else { return Err("the region has no boundary (a closed surface)".into()) };
    let mut loop_coedges = Vec::new();
    let mut at = start;
    loop {
        let coedge = next.get(&at).ok_or("the region's boundary does not close")?.clone();
        let (_, b) = coedge_ends(solid, &coedge).unwrap();
        loop_coedges.push(coedge);
        at = b;
        if at == start {
            break;
        }
        if loop_coedges.len() > next.len() {
            return Err("the region's boundary does not close".into());
        }
    }
    if loop_coedges.len() != next.len() {
        return Err("the region's boundary is more than one loop (the region has a hole)".into());
    }
    if loop_coedges.len() < 4 {
        return Err(format!("the region's boundary has {} edges; a four-sided patch needs at least four", loop_coedges.len()));
    }
    // Every boundary edge must be a straight line (a facet's edge is).
    let edge_of = |id: u64| solid.edges.iter().find(|e| e.id == id).unwrap();
    for coedge in &loop_coedges {
        let edge = edge_of(coedge.edge_id);
        let start = solid.vertices.iter().find(|v| v.id == edge.start_vertex_id).unwrap().point;
        let end = solid.vertices.iter().find(|v| v.id == edge.end_vertex_id).unwrap().point;
        let mid = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).map_err(|e| format!("boundary edge: {e}"))?;
        let chord = start.add(end).scale(0.5);
        if mid.sub(chord).length() > radius.max(1e-9 * end.sub(start).length()) * 10.0 {
            return Err("a boundary edge of the region is curved; the NURBS patch is bounded by straight facet edges only".into());
        }
    }
    // Boundary vertex positions, in loop order.
    let loop_vertices: Vec<u64> = loop_coedges.iter().map(|c| coedge_ends(solid, c).unwrap().0).collect();
    let boundary_points: Vec<Vec3> = loop_vertices.iter().map(|&id| point_of(id).unwrap()).collect();
    let n = boundary_points.len();

    // Corners: the four boundary vertices nearest the corners of the
    // region's principal-axis box, in loop order.
    let region_points: Vec<Vec3> = region_vertices.iter().filter_map(|&v| point_of(v)).collect();
    let (centroid, axes) = super::principal_frame_kernel(&region_points);
    let project = |p: Vec3| {
        let d = p.sub(centroid);
        [d.dot(axes[0]), d.dot(axes[1])]
    };
    let uv: Vec<[f64; 2]> = boundary_points.iter().map(|&p| project(p)).collect();
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in &uv {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let box_corners = [[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]];
    let mut corners: Vec<usize> = Vec::new();
    for corner in box_corners {
        let best = (0..n)
            .filter(|i| !corners.contains(i))
            .min_by(|&a, &b| {
                let da = (uv[a][0] - corner[0]).powi(2) + (uv[a][1] - corner[1]).powi(2);
                let db = (uv[b][0] - corner[0]).powi(2) + (uv[b][1] - corner[1]).powi(2);
                da.total_cmp(&db)
            })
            .unwrap();
        corners.push(best);
    }
    corners.sort_unstable();
    corners.dedup();
    if corners.len() != 4 {
        return Err("could not split the region's boundary into four sides".into());
    }
    // Sides as runs of boundary vertex indices (inclusive), loop order.
    let side = |s: usize| -> Vec<usize> {
        let (a, b) = (corners[s], corners[(s + 1) % 4]);
        let mut run = vec![a];
        let mut i = a;
        while i != b {
            i = (i + 1) % n;
            run.push(i);
        }
        run
    };
    let sides: Vec<Vec<usize>> = (0..4).map(side).collect();
    // Chord-length parameters along each side, 0..1.
    let params = |run: &[usize]| -> Result<Vec<f64>, String> {
        let mut t = vec![0.0];
        for w in run.windows(2) {
            t.push(t.last().unwrap() + boundary_points[w[1]].sub(boundary_points[w[0]]).length());
        }
        let total = *t.last().unwrap();
        if total <= 0.0 {
            return Err("a side of the region has no length".into());
        }
        Ok(t.into_iter().map(|x| x / total).collect())
    };
    // Side 0: (0,0)->(1,0); side 1: (1,0)->(1,1); side 2: (1,1)->(0,1);
    // side 3: (0,1)->(0,0). Square-boundary position of every boundary vertex.
    let p0 = params(&sides[0])?;
    let p1 = params(&sides[1])?;
    let p2 = params(&sides[2])?;
    let p3 = params(&sides[3])?;
    let mut square: HashMap<usize, [f64; 2]> = HashMap::new();
    for (k, &i) in sides[0].iter().enumerate() {
        square.insert(i, [p0[k], 0.0]);
    }
    for (k, &i) in sides[1].iter().enumerate() {
        square.insert(i, [1.0, p1[k]]);
    }
    for (k, &i) in sides[2].iter().enumerate() {
        square.insert(i, [1.0 - p2[k], 1.0]);
    }
    for (k, &i) in sides[3].iter().enumerate() {
        square.insert(i, [0.0, 1.0 - p3[k]]);
    }
    // Knots: the union of both opposite sides' vertex parameters.
    let merge = |a: &[f64], b: &[f64]| -> Vec<f64> {
        let mut all: Vec<f64> = a.iter().chain(b).copied().collect();
        all.sort_by(f64::total_cmp);
        let mut out: Vec<f64> = Vec::new();
        for x in all {
            if out.last().is_none_or(|&last| x - last > 1e-9) {
                out.push(x);
            } else if x == 1.0 {
                *out.last_mut().unwrap() = 1.0;
            }
        }
        *out.first_mut().unwrap() = 0.0;
        *out.last_mut().unwrap() = 1.0;
        out
    };
    let u_breaks = merge(&p0, &p2.iter().map(|t| 1.0 - t).collect::<Vec<_>>());
    let v_breaks = merge(&p1, &p3.iter().map(|t| 1.0 - t).collect::<Vec<_>>());
    let cu = 3 * (u_breaks.len() - 1) + 1;
    let cv = 3 * (v_breaks.len() - 1) + 1;
    if cu * cv > 40_000 {
        return Err(format!("the region's boundary needs a {cu} × {cv} control net; split the region"));
    }
    let knots = |breaks: &[f64]| {
        let mut k = vec![0.0; 4];
        for &b in &breaks[1..breaks.len() - 1] {
            k.extend([b, b, b]);
        }
        k.extend([1.0; 4]);
        k
    };
    let knots_u = knots(&u_breaks);
    let knots_v = knots(&v_breaks);
    // Control parameter of control index i (span s, offset r of 0..3).
    let control_param = |breaks: &[f64], i: usize| {
        let s = (i / 3).min(breaks.len() - 2);
        let r = i - 3 * s;
        breaks[s] + (breaks[s + 1] - breaks[s]) * r as f64 / 3.0
    };
    // A side polyline evaluated at its parameter t (linear per segment).
    let along = |run: &[usize], t_side: &[f64], t: f64| -> Vec3 {
        let k = t_side.partition_point(|&x| x <= t).clamp(1, run.len() - 1);
        let (t0, t1) = (t_side[k - 1], t_side[k]);
        let f = if t1 > t0 { ((t - t0) / (t1 - t0)).clamp(0.0, 1.0) } else { 0.0 };
        boundary_points[run[k - 1]].scale(1.0 - f).add(boundary_points[run[k]].scale(f))
    };
    let mut net: Vec<Option<Vec3>> = vec![None; cu * cv];
    let at = |i: usize, j: usize| i * cv + j;
    for i in 0..cu {
        let u = control_param(&u_breaks, i);
        net[at(i, 0)] = Some(along(&sides[0], &p0, u));
        net[at(i, cv - 1)] = Some(along(&sides[2], &p2, 1.0 - u));
    }
    for j in 0..cv {
        let v = control_param(&v_breaks, j);
        net[at(cu - 1, j)] = Some(along(&sides[1], &p1, v));
        net[at(0, j)] = Some(along(&sides[3], &p3, 1.0 - v));
    }

    // Tutte embedding of the region's vertices on the square.
    let boundary_mesh: HashMap<u64, [f64; 2]> = loop_vertices
        .iter()
        .enumerate()
        .map(|(k, &id)| (id, square[&k]))
        .collect();
    let mut neighbours: BTreeMap<u64, BTreeSet<u64>> = BTreeMap::new();
    for &(a, b) in region_edges.values() {
        neighbours.entry(a).or_default().insert(b);
        neighbours.entry(b).or_default().insert(a);
    }
    let mut param: HashMap<u64, [f64; 2]> = region_vertices.iter().map(|&v| (v, boundary_mesh.get(&v).copied().unwrap_or([0.5, 0.5]))).collect();
    for &v in &region_vertices {
        if !boundary_mesh.contains_key(&v) && !neighbours.contains_key(&v) {
            return Err("an isolated vertex in the region".into());
        }
    }
    let interior: Vec<u64> = region_vertices.iter().copied().filter(|v| !boundary_mesh.contains_key(v)).collect();
    for _ in 0..20_000 {
        let mut change = 0.0f64;
        for &v in &interior {
            let around = &neighbours[&v];
            let mut sum = [0.0; 2];
            for w in around {
                let p = param[w];
                sum[0] += p[0];
                sum[1] += p[1];
            }
            let new = [sum[0] / around.len() as f64, sum[1] / around.len() as f64];
            let old = param[&v];
            let relaxed = [old[0] + 1.8 * (new[0] - old[0]), old[1] + 1.8 * (new[1] - old[1])];
            change = change.max((relaxed[0] - old[0]).abs()).max((relaxed[1] - old[1]).abs());
            param.insert(v, relaxed);
        }
        if change < 1e-12 {
            break;
        }
    }

    // Least squares for the free control points, smoothed; the smoothest
    // weight that meets the tolerance is kept.
    let free: Vec<usize> = (0..cu * cv).filter(|&k| net[k].is_none()).collect();
    let free_index: HashMap<usize, usize> = free.iter().enumerate().map(|(k, &c)| (c, k)).collect();
    let samples: Vec<(Vec3, [f64; 2])> = region_vertices.iter().map(|&v| (point_of(v).unwrap(), param[&v])).collect();
    let rows: Vec<Vec<(usize, f64)>> = samples
        .iter()
        .map(|(_, uv)| {
            let (su, bu) = super::basis(&knots_u, 3, cu, uv[0]);
            let (sv, bv) = super::basis(&knots_v, 3, cv, uv[1]);
            let mut row = Vec::with_capacity(16);
            for (a, wu) in bu.iter().enumerate() {
                for (b, wv) in bv.iter().enumerate() {
                    if wu * wv != 0.0 {
                        row.push((at(su - 3 + a, sv - 3 + b), wu * wv));
                    }
                }
            }
            row
        })
        .collect();
    let trace = std::env::var("BREP_DEBUG_NURBS_FACE").is_ok();
    if trace {
        eprintln!("[nurbs-face] net {cu}x{cv} free {} samples {} boundary {n}", (0..cu * cv).filter(|&k| net[k].is_none()).count(), region_vertices.len());
    }
    // The Coons net of the four boundary rows, blended on the control
    // points' own (Greville) parameters: the patch the smoothing relaxes to.
    // Smoothing the displacement from it rather than the raw net keeps the
    // boundary's kinks fading inward; smoothing raw positions by index pulled
    // the first interior row straight across the staircase's reflex corners
    // and folded the patch there (the domed block: 28 reversed normals, a
    // second sheet 2.2e-3 mm from a boundary edge, which the importer's
    // projector then landed on — 21 s to read the STEP back).
    let pu: Vec<f64> = (0..cu).map(|i| control_param(&u_breaks, i)).collect();
    let pv: Vec<f64> = (0..cv).map(|j| control_param(&v_breaks, j)).collect();
    let coons: Vec<Vec3> = (0..cu * cv)
        .map(|c| {
            let (i, j) = (c / cv, c % cv);
            if let Some(p) = net[c] {
                return p;
            }
            let (u, v) = (pu[i], pv[j]);
            let g = |ii: usize, jj: usize| net[at(ii, jj)].unwrap();
            g(i, 0)
                .scale(1.0 - v)
                .add(g(i, cv - 1).scale(v))
                .add(g(0, j).scale(1.0 - u))
                .add(g(cu - 1, j).scale(u))
                .sub(g(0, 0).scale((1.0 - u) * (1.0 - v)))
                .sub(g(cu - 1, 0).scale(u * (1.0 - v)))
                .sub(g(0, cv - 1).scale((1.0 - u) * v))
                .sub(g(cu - 1, cv - 1).scale(u * v))
        })
        .collect();
    let ladder_started = web_time::Instant::now();
    let mut best: Option<(NurbsSurface, f64, f64)> = None;
    let mut folds = 0usize;
    // Each weight starts from the last one's displacements (the ladder only
    // loosens the smoothing, so the next solution is near).
    let mut warm = vec![Vec3::new(0.0, 0.0, 0.0); free.len()];
    for weight in [1e0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8] {
        let solved = if free.is_empty() {
            Vec::new()
        } else {
            solve_net(&coons, &free, &free_index, &rows, &samples, cu, cv, &pu, &pv, weight, &warm)
        };
        warm.clone_from(&solved);
        let mut full = coons.clone();
        for (k, &c) in free.iter().enumerate() {
            full[c] = coons[c].add(solved[k]);
        }
        let mut max = 0.0f64;
        let mut sum = 0.0f64;
        for ((point, _), row) in samples.iter().zip(&rows) {
            let s = row.iter().fold(Vec3::new(0.0, 0.0, 0.0), |acc, &(c, w)| acc.add(full[c].scale(w)));
            let d = s.sub(*point).length();
            max = max.max(d);
            sum += d * d;
        }
        let rms = (sum / samples.len() as f64).sqrt();
        let control_points: Vec<Vec<Vec4>> = (0..cu).map(|i| (0..cv).map(|j| Vec4::from_point(full[at(i, j)], 1.0)).collect()).collect();
        let surface = NurbsSurface::new(3, 3, knots_u.clone(), knots_v.clone(), control_points).map_err(|e| format!("NURBS surface: {e}"))?;
        let folded = reversed_normals(&surface, &u_breaks, &v_breaks, &tri_points);
        if trace {
            eprintln!("[nurbs-face] weight {weight:e} max {max:.3e} rms {rms:.3e} reversed normals {folded} at {:.2}s", ladder_started.elapsed().as_secs_f64());
        }
        if folded > 0 {
            folds += 1;
            continue;
        }
        let done = max <= choice.tolerance;
        if best.as_ref().is_none_or(|(_, m, _)| max < *m) || done {
            best = Some((surface, max, rms));
        }
        if done {
            break;
        }
    }
    let Some((surface, max_deviation, _)) = best else {
        return Err(format!("every NURBS face with this boundary folds over itself ({folds} tried); split the region or choose its corners differently"));
    };
    if max_deviation > choice.tolerance {
        return Err(format!(
            "the closest NURBS face with this boundary that does not fold is {max_deviation:.3e} mm from the mesh, over the {:.3e} mm tolerance",
            choice.tolerance
        ));
    }

    // The face: one loop of the region's boundary coedges, each with its
    // straight parameter line along the square.
    let mut next_id = 0u64;
    for vertex in &solid.vertices {
        next_id = next_id.max(vertex.id);
    }
    for edge in &solid.edges {
        next_id = next_id.max(edge.id);
    }
    for face in solid.shells.iter().flat_map(|s| &s.faces) {
        next_id = next_id.max(face.id);
        for l in &face.loops {
            next_id = next_id.max(l.id);
            for c in &l.coedges {
                next_id = next_id.max(c.id);
            }
        }
    }
    next_id += 1;
    let mut fresh = || {
        next_id += 1;
        next_id - 1
    };
    let mut coedges = Vec::new();
    for (k, coedge) in loop_coedges.iter().enumerate() {
        let a = square[&k];
        let b = square[&((k + 1) % n)];
        let pcurve = brep_kernel::make_line(Vec3::new(a[0], a[1], 0.0), Vec3::new(b[0], b[1], 0.0))?;
        coedges.push(CoedgeRecord { id: fresh(), edge_id: coedge.edge_id, forward: coedge.forward, pcurve });
    }
    // Orientation: the loop runs counter-clockwise on the square, so the
    // surface normal Su × Sv is the side the region's facets face when the
    // loop runs counter-clockwise around their outward normal — check it.
    let facet_normal = region_faces.iter().fold(Vec3::new(0.0, 0.0, 0.0), |acc, &i| {
        let face = &shell.faces[i];
        let sign = if face.same_sense { 1.0 } else { -1.0 };
        let normal = face.surface.normal(0.5 * (face.surface.knots_u[0] + face.surface.knots_u.last().unwrap()), 0.5 * (face.surface.knots_v[0] + face.surface.knots_v.last().unwrap()));
        acc.add(normal.map(|n| n.scale(sign)).unwrap_or(Vec3::new(0.0, 0.0, 0.0)))
    });
    let mid = surface.normal(0.5, 0.5).map_err(|e| format!("NURBS normal: {e}"))?;
    let same_sense = mid.dot(facet_normal) >= 0.0;
    let name = shell.faces[region_faces[0]].name.clone();
    let face = FaceRecord { id: fresh(), surface, same_sense, loops: vec![LoopRecord { id: fresh(), coedges }], name };

    // Drop the region's faces, interior edges and interior vertices.
    let interior_edges: HashSet<u64> = uses.iter().filter(|&(_, &n)| n == 2).map(|(&e, _)| e).collect();
    let kept_vertices: HashSet<u64> = loop_vertices.iter().copied().collect();
    let region_vertex_ids: HashSet<u64> = region_faces
        .iter()
        .flat_map(|&i| shell.faces[i].loops.iter().flat_map(|l| &l.coedges))
        .filter_map(|c| coedge_ends(solid, c))
        .flat_map(|(a, b)| [a, b])
        .collect();
    let first = region_faces[0];
    let shell = &mut solid.shells[0];
    let mut kept = Vec::with_capacity(shell.faces.len());
    for (index, old) in std::mem::take(&mut shell.faces).into_iter().enumerate() {
        if index == first {
            kept.push(face.clone());
        } else if !region_face_set.contains(&index) {
            kept.push(old);
        }
    }
    shell.faces = kept;
    solid.edges.retain(|e| !interior_edges.contains(&e.id));
    solid.vertices.retain(|v| !region_vertex_ids.contains(&v.id) || kept_vertices.contains(&v.id));
    solid.mass_properties_cache = Default::default();
    Ok(NurbsFaceOutcome { applied: true, max_deviation: Some(max_deviation), control_points: Some([cu, cv]), reason: None })
}

/// Samples of `surface` whose normal turns against the region's nearest
/// source triangle (by more than a right angle, against the orientation most
/// samples agree on): where the patch folds over itself. Sampled inside every
/// knot span, off its creases. A folded patch has a second sheet beside its
/// own boundary, which the importer's projector can land on.
fn reversed_normals(surface: &NurbsSurface, u_breaks: &[f64], v_breaks: &[f64], tri_points: &[[Vec3; 3]]) -> usize {
    let triangles: Vec<(Vec3, Vec3)> = tri_points
        .iter()
        .filter_map(|t| {
            let n = t[1].sub(t[0]).cross(t[2].sub(t[0]));
            let length = n.length();
            (length > 0.0).then(|| (t[0].add(t[1]).add(t[2]).scale(1.0 / 3.0), n.scale(1.0 / length)))
        })
        .collect();
    let mut dots = Vec::new();
    for su in u_breaks.windows(2) {
        for sv in v_breaks.windows(2) {
            for a in [0.1, 0.5, 0.9] {
                for b in [0.1, 0.5, 0.9] {
                    let (u, v) = (su[0] + a * (su[1] - su[0]), sv[0] + b * (sv[1] - sv[0]));
                    let (Ok(point), Ok(normal)) = (surface.evaluate(u, v), surface.normal(u, v)) else { continue };
                    let Some((_, n)) = triangles.iter().min_by(|x, y| x.0.sub(point).length().total_cmp(&y.0.sub(point).length())) else { continue };
                    dots.push(normal.dot(*n));
                }
            }
        }
    }
    let sign = if dots.iter().sum::<f64>() >= 0.0 { 1.0 } else { -1.0 };
    dots.iter().filter(|&&d| d * sign < 0.0).count()
}

/// The most conjugate-gradient steps one coordinate's solve takes.
const CG_ITERATIONS: usize = 1500;

/// Preconditioned conjugate gradients on the smoothed normal equations,
/// matrix-free. The unknowns are the free control points' DISPLACEMENTS
/// from `base` (the Coons net; the boundary's are zero), smoothed by their
/// second divided differences in the control parameters `pu`/`pv`, each
/// stencil weighted by its parameter extent (a discrete bending energy that
/// does not depend on how unevenly the knots are spaced).
#[allow(clippy::too_many_arguments)]
fn solve_net(
    base: &[Vec3],
    free: &[usize],
    free_index: &HashMap<usize, usize>,
    rows: &[Vec<(usize, f64)>],
    samples: &[(Vec3, [f64; 2])],
    cu: usize,
    cv: usize,
    pu: &[f64],
    pv: &[f64],
    weight: f64,
    start: &[Vec3],
) -> Vec<Vec3> {
    let m = free.len();
    let w = weight.sqrt();
    // Second divided differences of the net, both directions.
    let divided = |t: &[f64], k: usize| -> Option<[f64; 3]> {
        let (h1, h2) = (t[k] - t[k - 1], t[k + 1] - t[k]);
        if !(h1 > 0.0 && h2 > 0.0) {
            return None;
        }
        let extent = (0.5 * (h1 + h2)).sqrt();
        Some([2.0 / (h1 * (h1 + h2)), -2.0 / (h1 * h2), 2.0 / (h2 * (h1 + h2))].map(|c| c * extent * w))
    };
    let mut stencils: Vec<([usize; 3], [f64; 3])> = Vec::new();
    for i in 0..cu {
        for j in 1..cv - 1 {
            if let Some(co) = divided(pv, j) {
                stencils.push(([i * cv + j - 1, i * cv + j, i * cv + j + 1], co));
            }
        }
    }
    for j in 0..cv {
        for i in 1..cu - 1 {
            if let Some(co) = divided(pu, i) {
                stencils.push(([(i - 1) * cv + j, i * cv + j, (i + 1) * cv + j], co));
            }
        }
    }
    // The system's rows over the free unknowns only (samples, then
    // stencils), resolved once: the iterations below never look an index up.
    let free_rows: Vec<Vec<(usize, f64)>> = rows
        .iter()
        .map(|row| row.iter().filter_map(|&(c, wt)| free_index.get(&c).map(|&k| (k, wt))).collect())
        .chain(stencils.iter().map(|(st, coefficients)| {
            st.iter().zip(coefficients).filter_map(|(&c, &co)| free_index.get(&c).map(|&k| (k, co))).collect()
        }))
        .filter(|row: &Vec<(usize, f64)>| !row.is_empty())
        .collect();
    // A x for the free unknowns (one coordinate), A^T A applied. Fixed
    // displacements are zero, so the stencils add nothing to the right side.
    let apply = |x: &[f64]| -> Vec<f64> {
        let mut out = vec![0.0; m];
        for row in &free_rows {
            let s: f64 = row.iter().map(|&(k, wt)| wt * x[k]).sum();
            for &(k, wt) in row {
                out[k] += wt * s;
            }
        }
        out
    };
    // Jacobi preconditioner: the diagonal of the normal matrix.
    let mut diagonal = vec![0.0; m];
    for row in &free_rows {
        for &(k, wt) in row {
            diagonal[k] += wt * wt;
        }
    }
    let inverse: Vec<f64> = diagonal.iter().map(|&d| if d > 0.0 { 1.0 / d } else { 1.0 }).collect();
    let mut result = vec![Vec3::new(0.0, 0.0, 0.0); m];
    for axis in 0..3 {
        let coord = |p: Vec3| match axis {
            0 => p.x,
            1 => p.y,
            _ => p.z,
        };
        // Right-hand side A^T (b - A base): what the samples ask beyond the
        // Coons net.
        let mut rhs = vec![0.0; m];
        for ((point, _), row) in samples.iter().zip(rows) {
            let fixed: f64 = row.iter().map(|&(c, wt)| wt * coord(base[c])).sum();
            let r = coord(*point) - fixed;
            for &(c, wt) in row {
                if let Some(&k) = free_index.get(&c) {
                    rhs[k] += wt * r;
                }
            }
        }
        // From `start` (zero: the Coons net itself).
        let mut x: Vec<f64> = start.iter().map(|&p| coord(p)).collect();
        let ax = apply(&x);
        let mut r: Vec<f64> = rhs.iter().zip(&ax).map(|(b, a)| b - a).collect();
        let mut z: Vec<f64> = r.iter().zip(&inverse).map(|(a, b)| a * b).collect();
        let mut p = z.clone();
        let mut rz: f64 = r.iter().zip(&z).map(|(a, b)| a * b).sum();
        let scale: f64 = rhs.iter().map(|v| v * v).sum::<f64>().max(1e-300);
        for _ in 0..CG_ITERATIONS.min(4 * m + 50) {
            let rr: f64 = r.iter().map(|v| v * v).sum();
            if rr <= 1e-24 * scale {
                break;
            }
            let ap = apply(&p);
            let pap: f64 = p.iter().zip(&ap).map(|(a, b)| a * b).sum();
            if pap <= 0.0 {
                break;
            }
            let alpha = rz / pap;
            for k in 0..m {
                x[k] += alpha * p[k];
                r[k] -= alpha * ap[k];
            }
            for k in 0..m {
                z[k] = r[k] * inverse[k];
            }
            let next: f64 = r.iter().zip(&z).map(|(a, b)| a * b).sum();
            let beta = next / rz;
            rz = next;
            for k in 0..m {
                p[k] = z[k] + beta * p[k];
            }
        }
        for k in 0..m {
            match axis {
                0 => result[k].x = x[k],
                1 => result[k].y = x[k],
                _ => result[k].z = x[k],
            }
        }
    }
    result
}
