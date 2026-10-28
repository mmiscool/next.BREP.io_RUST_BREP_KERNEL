//! 5-axis swarf (lane C10, row 4; spec section 7): the side of the tool
//! along the rulings of the selected wall faces.
//!
//! Rulings come from the wall's own tessellation: the boundary loop of the
//! selected triangles is split into the lower and upper rails at the mid
//! height along the wall's "up" direction (setup +Z projected into the
//! wall); at each lower-rail vertex the ruling is the wall edge to an upper
//! vertex closest to the steepest-ascent direction, or that direction when
//! no such edge exists (a wall tessellated with interior vertices). The
//! tool axis is the ruling tilted by `tilt_deg` away from the wall about the
//! rail tangent; the axis line sits one cutter radius (plus the radial
//! allowance and 2ε) outside the wall along the vertex normal; the tip sits
//! on the lower rail for the last pass and `k/passes` up the ruling for the
//! earlier ones (top pass first). Every CL point is certified: the cutting
//! part by the tool-frame drop (gouge beyond the operation tolerance fails
//! the operation), the shank and holder by tilting (row 5). Heights are
//! distances along the tool axis for swarf (`clearance` for rapids,
//! `retract` for links).

use crate::cutter::{ToolPart, ToolProfile};
use crate::kinematics::rotate;
use crate::model::*;
use crate::multiaxis::contact::drop_along;
use crate::multiaxis::links::traverse;
use crate::multiaxis::tilt::{avoid_collisions, ClPose, TiltOptions};
use crate::{CamError, Cutter, CutterKind, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};
use std::collections::BTreeMap;

/// One ruling of the wall.
#[derive(Clone, Debug, PartialEq)]
pub struct Ruling {
    /// Lower-rail vertex.
    pub base: Point3,
    /// Unit direction up the wall.
    pub direction: Point3,
    /// Length to the upper rail.
    pub length: f64,
    /// Unit outward wall normal at the base.
    pub normal: Point3,
    /// Unit lower-rail tangent at the base (travel direction).
    pub tangent: Point3,
}

fn key(p: Point3, eps: f64) -> [i64; 3] {
    let q = |v: f64| (v / eps).round() as i64;
    [q(p.x), q(p.y), q(p.z)]
}

/// Extract the rulings of the selected wall triangles.
pub fn wall_rulings(mesh: &MeshSnapshot, wall: &[u32], tolerance: &Tolerance) -> Result<Vec<Ruling>, CamError> {
    let bad = |m: &str| CamError::InvalidInput(format!("swarf: {m}"));
    if wall.is_empty() {
        return Err(bad("no wall triangles selected"));
    }
    let eps = tolerance.point_eps.max(1e-9);
    // Vertices by quantised key, edges by vertex-index pair.
    let mut verts: Vec<Point3> = Vec::new();
    let mut vid: BTreeMap<[i64; 3], usize> = BTreeMap::new();
    let mut id_of = |p: Point3| -> usize {
        let k = key(p, eps);
        if let Some(&i) = vid.get(&k) {
            return i;
        }
        verts.push(p);
        vid.insert(k, verts.len() - 1);
        verts.len() - 1
    };
    let mut edge_count: BTreeMap<(usize, usize), u32> = BTreeMap::new();
    let mut tris: Vec<[usize; 3]> = Vec::with_capacity(wall.len());
    let mut normal_sum = Point3::ZERO;
    for &t in wall {
        let tri = &mesh.triangles[t as usize];
        let ids = [id_of(tri.a), id_of(tri.b), id_of(tri.c)];
        tris.push(ids);
        normal_sum = normal_sum.add(tri.normal.scale(tri.area()));
        for e in 0..3 {
            let (a, b) = (ids[e], ids[(e + 1) % 3]);
            *edge_count.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    // Vertex normals (area weighted) and wall edges per vertex.
    let mut vnormal: Vec<Point3> = vec![Point3::ZERO; verts.len()];
    let mut vedges: Vec<Vec<usize>> = vec![Vec::new(); verts.len()];
    let mut total_area = 0.0;
    for (k, ids) in tris.iter().enumerate() {
        let tri = &mesh.triangles[wall[k] as usize];
        total_area += tri.area();
        for e in 0..3 {
            vnormal[ids[e]] = vnormal[ids[e]].add(tri.normal.scale(tri.area()));
            vedges[ids[e]].push(ids[(e + 1) % 3]);
            vedges[ids[e]].push(ids[(e + 2) % 3]);
        }
    }
    // "Up" is setup +Z within the wall. A closed wall's normals cancel; its
    // up is +Z itself.
    let mean_normal = if normal_sum.length() > 1e-6 * total_area { normal_sum.normalized() } else { None };
    let up = match mean_normal {
        Some(n) => Point3::Z.sub(n.scale(Point3::Z.dot(n))).normalized().ok_or_else(|| bad("the wall is horizontal; swarf needs a wall"))?,
        None => Point3::Z,
    };
    // Boundary edges and the single loop through them.
    let mut adjacency: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (&(a, b), &n) in &edge_count {
        if n == 1 {
            adjacency.entry(a).or_default().push(b);
            adjacency.entry(b).or_default().push(a);
        }
    }
    if adjacency.is_empty() || adjacency.values().any(|v| v.len() != 2) {
        return Err(bad("the wall region has no closed boundary loops"));
    }
    // Walk every boundary loop.
    let mut loops: Vec<Vec<usize>> = Vec::new();
    let mut seen = vec![false; verts.len()];
    for &start in adjacency.keys() {
        if seen[start] {
            continue;
        }
        let mut ids = vec![start];
        seen[start] = true;
        let mut prev = start;
        let mut cur = adjacency[&start][0];
        while cur != start {
            ids.push(cur);
            seen[cur] = true;
            let next = adjacency[&cur].iter().copied().find(|&n| n != prev).ok_or_else(|| bad("boundary walk failed"))?;
            prev = cur;
            cur = next;
            if ids.len() > adjacency.len() {
                return Err(bad("boundary walk does not close"));
            }
        }
        loops.push(ids);
    }
    let height = |i: usize| verts[i].dot(up);
    // One loop: an open wall, split at the mid height into two chains. Two
    // loops: a closed wall (a cylinder), the lower loop is the whole rail.
    let (mut rail, closed, upper_mean): (Vec<usize>, bool, f64) = match loops.len() {
        1 => {
            let loop_ids = &loops[0];
            let heights: Vec<f64> = loop_ids.iter().map(|&i| height(i)).collect();
            let (lo, hi) = heights.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), &v| (l.min(v), h.max(v)));
            if hi - lo <= tolerance.distance_eps {
                return Err(bad("the wall has no height"));
            }
            let mid = 0.5 * (lo + hi);
            let lower: Vec<bool> = heights.iter().map(|&h| h < mid).collect();
            let n = loop_ids.len();
            let transitions = (0..n).filter(|&i| lower[i] != lower[(i + 1) % n]).count();
            if transitions != 2 {
                return Err(bad("the lower and upper rails are not two contiguous chains"));
            }
            let first_lower = (0..n).find(|&i| lower[i] && !lower[(i + n - 1) % n]).unwrap();
            let mut rail: Vec<usize> = Vec::new();
            let mut i = first_lower;
            while lower[i] {
                rail.push(loop_ids[i]);
                i = (i + 1) % n;
            }
            let (s, c) = loop_ids.iter().zip(&lower).filter(|(_, &l)| !l).fold((0.0, 0), |(s, c), (&i, _)| (s + height(i), c + 1));
            (rail, false, s / c.max(1) as f64)
        }
        2 => {
            let mean = |l: &Vec<usize>| l.iter().map(|&i| height(i)).sum::<f64>() / l.len() as f64;
            let (m0, m1) = (mean(&loops[0]), mean(&loops[1]));
            if (m0 - m1).abs() <= tolerance.distance_eps {
                return Err(bad("the two boundary loops are at the same height"));
            }
            let (low, high) = if m0 < m1 { (0, 1) } else { (1, 0) };
            (loops[low].clone(), true, mean(&loops[high]))
        }
        _ => return Err(bad("the wall region has more than two boundary loops")),
    };
    // Orient the rail so the wall normal is on the left of travel.
    if rail.len() >= 2 {
        let t = verts[rail[1]].sub(verts[rail[0]]);
        let n0 = vnormal[rail[0]].normalized().or(mean_normal).unwrap_or(Point3::X);
        if up.cross(t).dot(n0) < 0.0 {
            rail.reverse();
        }
    }
    let is_lower: Vec<bool> = {
        let mut v = vec![false; verts.len()];
        for &r in &rail {
            v[r] = true;
        }
        v
    };
    if closed && rail.len() >= 3 {
        let first = rail[0];
        rail.push(first);
    }
    let mut rulings = Vec::with_capacity(rail.len());
    for (k, &v) in rail.iter().enumerate() {
        let base = verts[v];
        let normal = vnormal[v].normalized().or(mean_normal).unwrap_or(Point3::X);
        let steepest = up.sub(normal.scale(up.dot(normal))).normalized().unwrap_or(up);
        let mut best: Option<(f64, Point3, f64)> = None;
        for &o in &vedges[v] {
            if is_lower[o] {
                continue;
            }
            let d = verts[o].sub(base);
            let Some(dir) = d.normalized() else { continue };
            let score = dir.dot(steepest);
            if best.map_or(true, |(s, _, _)| score > s) {
                best = Some((score, dir, d.length()));
            }
        }
        let (direction, length) = match best {
            Some((_, dir, len)) => (dir, len),
            None => {
                let rise = steepest.dot(up).max(1e-9);
                (steepest, (upper_mean - base.dot(up)).max(0.0) / rise)
            }
        };
        let last = rail.len() - 1;
        let (ni, pi) = if closed {
            (if k == last { 1 } else { k + 1 }, if k == 0 { last - 1 } else { k - 1 })
        } else {
            ((k + 1).min(last), k.saturating_sub(1))
        };
        let next = verts[rail[ni]];
        let prevv = verts[rail[pi]];
        let tangent = next.sub(prevv).normalized().unwrap_or(up.cross(normal));
        rulings.push(Ruling { base, direction, length, normal, tangent });
    }
    Ok(rulings)
}

/// Generate swarf. `mesh` is the complete protected mesh in setup
/// coordinates; `holder` is the tool's holder for the collision check.
pub fn generate(
    operation: &Operation,
    cutter: &Cutter,
    holder: Option<&Holder>,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    default_heights: &Heights,
    default_feeds: &FeedsSpeeds,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    operation.validate().map_err(CamError::InvalidInput)?;
    let params = match &operation.strategy {
        Strategy::Swarf(p) => p,
        _ => return Err(CamError::InvalidInput("swarf generator requires a swarf operation".into())),
    };
    if !operation.enabled {
        return Err(CamError::InvalidInput("operation is disabled".into()));
    }
    cutter.validate()?;
    tolerance.validate()?;
    if !matches!(cutter.kind(), CutterKind::Flat | CutterKind::Bull | CutterKind::Ball) {
        return Err(CamError::UnsupportedCutter { strategy: "swarf".into(), cutter: cutter.kind().as_str().into() });
    }
    let heights = operation.heights.as_ref().unwrap_or(default_heights);
    let feeds = operation.feeds.as_ref().unwrap_or(default_feeds);
    heights.validate().map_err(CamError::InvalidInput)?;
    feeds.validate().map_err(CamError::InvalidInput)?;
    if mesh.is_empty() {
        return Err(CamError::EmptyTarget { operation_id: operation.id.to_string() });
    }
    if operation.targets.drive_faces.is_empty() {
        return Err(CamError::InvalidInput("swarf needs the wall faces as drive faces".into()));
    }
    let mut wall_faces = Vec::new();
    for f in &operation.targets.drive_faces {
        let ids = mesh.resolve_faces(f);
        if ids.is_empty() {
            return Err(CamError::UnresolvedTarget { solid: f.solid.clone(), face: f.face.clone() });
        }
        wall_faces.extend(ids);
    }
    wall_faces.sort_unstable();
    wall_faces.dedup();
    let wall = mesh.triangles_of_faces(&wall_faces);
    let passes = params.passes.max(1);
    if !params.tilt_deg.is_finite() || params.tilt_deg < 0.0 || params.tilt_deg >= 90.0 {
        return Err(CamError::InvalidInput("swarf tilt must be in [0, 90) degrees".into()));
    }
    let rulings = wall_rulings(mesh, &wall, &tolerance)?;
    if rulings.len() < 2 {
        return Err(CamError::InvalidInput("swarf: the lower rail has fewer than two vertices".into()));
    }
    let offset = cutter.radius() + operation.stock_to_leave.radial.max(0.0) + 2.0 * tolerance.distance_eps;
    let profile = ToolProfile::new(cutter, holder, operation.tolerance);
    let mut out = Toolpath::new(operation.id.clone(), operation.tool.clone());
    out.report.counts.triangles = mesh.triangle_count() as u64;
    let top_z = mesh.bounds.max.z + heights.clearance;
    let mut prev_retract: Option<ClPose> = None;
    let mut fallbacks = 0u64;
    let mut unproven_traverses = 0u64;
    runner.stage("swarf-passes", "Swarf passes", 0, passes as u64)?;
    let mut gouges = 0u64;
    let mut first_gouge: Option<Point3> = None;
    let max_axis_step = 5f64.to_radians();
    for pass in 0..passes {
        runner.work(1, "swarf-passes", pass as u64, passes as u64)?;
        let fraction = (passes - 1 - pass) as f64 / passes as f64;
        // CL poses along the rail, densified so the axis turns ≤ 5° per chord.
        let pose_of = |r: &Ruling| -> ClPose {
            let mut axis = r.direction;
            if params.tilt_deg > 0.0 {
                let t = r.tangent;
                let candidate = rotate(t, params.tilt_deg.to_radians(), axis);
                axis = if candidate.dot(r.normal) >= axis.dot(r.normal) { candidate } else { rotate(t, -params.tilt_deg.to_radians(), axis) };
            }
            let tip = r.base.add(r.normal.scale(offset)).add(r.direction.scale(r.length * fraction));
            ClPose { tip, axis }
        };
        let mut poses: Vec<ClPose> = Vec::new();
        for w in rulings.windows(2) {
            let (a, b) = (pose_of(&w[0]), pose_of(&w[1]));
            if poses.is_empty() {
                poses.push(a);
            }
            let angle = a.axis.dot(b.axis).clamp(-1.0, 1.0).acos();
            let n = ((angle / max_axis_step).ceil() as usize).max(1);
            for s in 1..=n {
                let f = s as f64 / n as f64;
                let axis = slerp(a.axis, b.axis, f);
                poses.push(ClPose { tip: a.tip.lerp(b.tip, f), axis });
            }
        }
        let mut ordered = poses;
        if pass % 2 == 1 {
            ordered.reverse();
        }
        // Cutting part first: a gouging pose is a gouge, not a collision.
        let mut clear: Vec<ClPose> = Vec::with_capacity(ordered.len());
        for p in &ordered {
            let d = drop_along(mesh, index, cutter, p.tip, p.axis, cutter.radius(), tolerance, runner)?;
            out.report.counts.samples += 1;
            out.report.counts.candidates += d.candidates;
            if d.offset > operation.tolerance {
                gouges += 1;
                first_gouge.get_or_insert(p.tip);
            } else {
                clear.push(*p);
            }
        }
        // Shank and holder by tilting (tilts re-checked against the cutting part).
        let tilt_opts = TiltOptions { parts: vec![ToolPart::Shank, ToolPart::Holder], gouge_tolerance: Some(operation.tolerance), reach: cutter.radius(), ..Default::default() };
        let tilted = avoid_collisions(mesh, index, &profile, Some(cutter), &clear, &tilt_opts, tolerance, &mut out.report, runner)?;
        let cl: Vec<ClPose> = tilted.iter().filter(|t| !t.unreachable).map(|t| t.pose).collect();
        if cl.len() < 2 {
            out.report.empty_regions.push(EmptyRegion { reason: "unreachable".into(), target: operation.targets.drive_faces.first().cloned() });
            continue;
        }
        let path_id = pass;
        let first = cl[0];
        let approach = ClPose { tip: first.tip.add(first.axis.scale(heights.clearance)), axis: first.axis };
        match prev_retract {
            None => {
                let mut m = Motion::rapid(approach.tip).with_path(path_id);
                m.tool_axis = Some(first.axis);
                out.motions.push(m);
            }
            Some(prev) => {
                let t = traverse(mesh, index, cutter, prev, approach, cutter.radius(), top_z, path_id, tolerance, runner)?;
                if t.fallback {
                    fallbacks += 1;
                }
                if t.unproven {
                    unproven_traverses += 1;
                }
                out.motions.extend(t.motions);
            }
        }
        let mut m = Motion::plunge(first.tip, feeds.plunge_feed).with_path(path_id);
        m.tool_axis = Some(first.axis);
        out.motions.push(m);
        for p in &cl[1..] {
            let mut m = Motion::cut(p.tip, feeds.cutting_feed).with_path(path_id);
            m.tool_axis = Some(p.axis);
            out.motions.push(m);
        }
        let last = cl[cl.len() - 1];
        let retract_at = last.tip.add(last.axis.scale(heights.clearance));
        let mut m = Motion::retract(retract_at, Feed::RAPID).with_path(path_id);
        m.tool_axis = Some(last.axis);
        out.motions.push(m);
        prev_retract = Some(ClPose { tip: retract_at, axis: last.axis });
        out.report.counts.passes += 1;
    }
    if fallbacks > 0 {
        out.report.warn(Diagnostic::new("link-fallback-clearance", format!("{fallbacks} traverses between runs were unproven and use the vertical clearance")));
    }
    if unproven_traverses > 0 {
        out.report.fail(Diagnostic::new("link-unproven", format!("{unproven_traverses} vertical-clearance traverses still touch material")));
    }
    if gouges > 0 {
        let mut d = Diagnostic::new("gouge-detected", format!("{gouges} swarf CL points gouge beyond the machining tolerance"));
        if let Some(p) = first_gouge {
            d = d.at(p);
        }
        out.report.fail(d);
    }
    if out.motions.is_empty() && out.report.errors.is_empty() {
        out.report.fail(Diagnostic::new("empty-after-clipping", "no swarf pass could be machined"));
    }
    out.report.counts.motions = out.motions.len() as u64;
    out.report.finish();
    Ok(out)
}

fn slerp(a: Point3, b: Point3, t: f64) -> Point3 {
    let d = a.dot(b).clamp(-1.0, 1.0);
    let theta = d.acos();
    if theta < 1e-12 {
        return a;
    }
    let s = theta.sin();
    a.scale(((1.0 - t) * theta).sin() / s).add(b.scale((t * theta).sin() / s)).normalized().unwrap_or(b)
}

