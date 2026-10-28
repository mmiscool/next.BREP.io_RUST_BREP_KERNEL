//! Multi-axis contour (lane C10, row 4; spec section 7): the tool tip
//! follows the containment curves projected onto the drive faces with the
//! axis derived from the surface normal, lead (about the binormal, forward
//! positive) and tilt (about the tangent, left positive), smoothed so the
//! axis turns at most 5° per sample, then the cutter-location point is
//! built from the contact point for the cutter's corner radius and every
//! point is certified: cutting part by the tool-frame drop, shank and
//! holder by tilting (row 5). A curve sample with no drive triangle under
//! it ends the pass and is reported as an empty region. Heights are
//! distances along the tool axis (`clearance` for rapids).

use crate::cutter::{ToolPart, ToolProfile};
use crate::kinematics::rotate;
use crate::model::*;
use crate::multiaxis::contact::drop_along;
use crate::multiaxis::links::traverse;
use crate::multiaxis::tilt::{avoid_collisions, ClPose, TiltOptions};
use crate::{CamError, Cutter, CutterKind, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};

/// Largest axis change between consecutive samples after smoothing.
pub const MAX_AXIS_STEP_DEG: f64 = 5.0;

/// The highest drive triangle under `(x, y)`: surface point and unit normal.
pub fn surface_at(mesh: &MeshSnapshot, index: Option<&dyn TriangleIndex>, drive: &[bool], x: f64, y: f64) -> Option<(Point3, Point3)> {
    let mut ids = Vec::new();
    match index {
        Some(i) => i.query_cutter_xy(x, y, 0.0, &mut ids),
        None => ids.extend(0..mesh.triangle_count() as u32),
    }
    let mut best: Option<(Point3, Point3)> = None;
    for id in ids {
        if !drive[id as usize] {
            continue;
        }
        let t = &mesh.triangles[id as usize];
        let [a, b, c] = t.vertices();
        let det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
        if det.abs() <= 1e-18 {
            continue;
        }
        // Barycentric coordinates of (x, y).
        let u = ((x - a.x) * (c.y - a.y) - (c.x - a.x) * (y - a.y)) / det;
        let v = ((b.x - a.x) * (y - a.y) - (x - a.x) * (b.y - a.y)) / det;
        let eps = -1e-9;
        if u < eps || v < eps || u + v > 1.0 - eps {
            continue;
        }
        let z = a.z + u * (b.z - a.z) + v * (c.z - a.z);
        if best.map_or(true, |(p, _)| z > p.z) {
            best = Some((Point3::new(x, y, z), t.normal));
        }
    }
    best
}

/// Cutter-location point (tip) for a contact point `cc` with surface normal
/// `n`, tool axis `a`, cutter radius `big_r` and corner radius `r`.
pub fn cl_from_contact(cc: Point3, n: Point3, a: Point3, big_r: f64, r: f64) -> Point3 {
    let m = n.sub(a.scale(n.dot(a))).normalized().unwrap_or(Point3::ZERO);
    cc.add(n.scale(r)).add(m.scale(big_r - r)).sub(a.scale(r))
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

/// Limit the angle between consecutive axes to `max_rad`: alternating
/// forward and backward passes pull each axis toward its neighbour until no
/// pair exceeds the limit (bounded rounds); returns the number of changes.
pub fn smooth_axes(axes: &mut [Point3], max_rad: f64) -> usize {
    let mut changed = 0;
    let n = axes.len();
    for round in 0..64 {
        let mut round_changes = 0;
        for k in 1..n {
            let (i, j) = if round % 2 == 0 { (k - 1, k) } else { (n - k, n - k - 1) };
            let angle = axes[i].dot(axes[j]).clamp(-1.0, 1.0).acos();
            if angle > max_rad {
                axes[j] = slerp(axes[i], axes[j], (max_rad / angle) * (1.0 - 1e-9));
                round_changes += 1;
            }
        }
        changed += round_changes;
        if round_changes == 0 {
            break;
        }
    }
    changed
}

/// Generate multi-axis contour.
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
        Strategy::MultiAxisContour(p) => p,
        _ => return Err(CamError::InvalidInput("multi-axis contour generator requires a multi-axis contour operation".into())),
    };
    if !operation.enabled {
        return Err(CamError::InvalidInput("operation is disabled".into()));
    }
    cutter.validate()?;
    tolerance.validate()?;
    let corner = match cutter.kind() {
        CutterKind::Flat => 0.0,
        CutterKind::Ball => cutter.radius(),
        CutterKind::Bull => cutter.radius() - cutter.flat_radius_or_zero(),
        _ => return Err(CamError::UnsupportedCutter { strategy: "multi-axis contour".into(), cutter: cutter.kind().as_str().into() }),
    };
    let heights = operation.heights.as_ref().unwrap_or(default_heights);
    let feeds = operation.feeds.as_ref().unwrap_or(default_feeds);
    heights.validate().map_err(CamError::InvalidInput)?;
    feeds.validate().map_err(CamError::InvalidInput)?;
    if mesh.is_empty() {
        return Err(CamError::EmptyTarget { operation_id: operation.id.to_string() });
    }
    if params.containment.boundary.is_empty() {
        return Err(CamError::InvalidInput("multi-axis contour needs at least one boundary curve".into()));
    }
    for (name, v) in [("lead", params.lead_deg), ("tilt", params.tilt_deg)] {
        if !v.is_finite() || v.abs() >= 90.0 {
            return Err(CamError::InvalidInput(format!("multi-axis contour {name} must be within (-90, 90) degrees")));
        }
    }
    // Drive triangles: the drive faces, or every face of the target bodies.
    let mut drive = vec![false; mesh.triangle_count()];
    if operation.targets.drive_faces.is_empty() {
        for t in &mesh.triangles {
            let f = &mesh.faces[t.face as usize];
            drive[t.id as usize] = !f.protected_only && operation.targets.target_solids.contains(&f.solid);
        }
    } else {
        for f in &operation.targets.drive_faces {
            let ids = mesh.resolve_faces(f);
            if ids.is_empty() {
                return Err(CamError::UnresolvedTarget { solid: f.solid.clone(), face: f.face.clone() });
            }
            for t in mesh.triangles_of_faces(&ids) {
                drive[t as usize] = true;
            }
        }
    }
    if !drive.iter().any(|d| *d) {
        return Err(CamError::InvalidInput(format!("operation {} has no drive triangles", operation.id)));
    }
    let step = (cutter.radius() * 0.5).max(0.05);
    let big_r = cutter.radius() + operation.stock_to_leave.radial.max(0.0);
    let profile = ToolProfile::new(cutter, holder, operation.tolerance);
    let mut out = Toolpath::new(operation.id.clone(), operation.tool.clone());
    out.report.counts.triangles = mesh.triangle_count() as u64;
    let mut gouges = 0u64;
    let mut first_gouge: Option<Point3> = None;
    let top_z = mesh.bounds.max.z + heights.clearance;
    let mut prev_retract: Option<ClPose> = None;
    let mut fallbacks = 0u64;
    let mut unproven_traverses = 0u64;
    let loops = params.containment.boundary.len();
    runner.stage("contour-curves", "Multi-axis contour curves", 0, loops as u64)?;
    for (li, curve) in params.containment.boundary.iter().enumerate() {
        runner.work(1, "contour-curves", li as u64, loops as u64)?;
        if curve.len() < 2 {
            return Err(CamError::InvalidInput("a boundary curve needs at least two points".into()));
        }
        // Sample the closed curve.
        let mut samples: Vec<Point2> = Vec::new();
        let n = curve.len();
        for k in 0..n {
            let a = curve[k];
            let b = curve[(k + 1) % n];
            let len = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
            let m = ((len / step).ceil() as usize).max(1);
            for s in 0..m {
                let f = s as f64 / m as f64;
                samples.push(Point2::new(a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f));
            }
        }
        if samples.len() > 2_000_000 {
            return Err(CamError::InvalidInput("multi-axis contour sample budget exceeded".into()));
        }
        samples.push(samples[0]);
        // Contact points; a miss ends the run.
        let mut runs: Vec<Vec<(Point3, Point3)>> = vec![Vec::new()];
        let mut missed = 0usize;
        for s in &samples {
            out.report.counts.samples += 1;
            match surface_at(mesh, index, &drive, s.x, s.y) {
                Some(cc) => runs.last_mut().unwrap().push(cc),
                None => {
                    missed += 1;
                    if !runs.last().unwrap().is_empty() {
                        runs.push(Vec::new());
                    }
                }
            }
        }
        if missed > 0 {
            out.report.empty_regions.push(EmptyRegion { reason: "no-triangles".into(), target: operation.targets.drive_faces.first().cloned() });
            out.report.warn(Diagnostic::new("contour-off-surface", format!("{missed} samples of curve {li} have no drive surface under them")));
        }
        for run in runs.into_iter().filter(|r| r.len() >= 2) {
            // Axes from normals, lead and tilt; then smoothed.
            let mut axes: Vec<Point3> = Vec::with_capacity(run.len());
            for (k, (_, nrm)) in run.iter().enumerate() {
                let prev = run[k.saturating_sub(1)].0;
                let next = run[(k + 1).min(run.len() - 1)].0;
                let d = next.sub(prev);
                let t = d.sub(nrm.scale(d.dot(*nrm))).normalized().unwrap_or_else(|| nrm.cross(Point3::X).normalized().unwrap_or(Point3::Y));
                let b = nrm.cross(t);
                let mut a = rotate(b, params.lead_deg.to_radians(), *nrm);
                a = rotate(t, params.tilt_deg.to_radians(), a);
                axes.push(a.normalized().unwrap_or(*nrm));
            }
            let smoothed = smooth_axes(&mut axes, MAX_AXIS_STEP_DEG.to_radians());
            if smoothed > 0 {
                out.report.approximations.push(ApproximationLimit {
                    kind: "axis-smoothed".into(),
                    bound: Some(MAX_AXIS_STEP_DEG),
                    detail: Some(format!("{smoothed} axes limited to {MAX_AXIS_STEP_DEG}° per sample")),
                });
            }
            let poses: Vec<ClPose> = run.iter().zip(&axes).map(|((p, nrm), a)| ClPose { tip: cl_from_contact(*p, *nrm, *a, big_r, corner), axis: *a }).collect();
            // Cutting part first, then shank/holder by tilting about the corner centre.
            let mut clear: Vec<ClPose> = Vec::with_capacity(poses.len());
            for p in &poses {
                let d = drop_along(mesh, index, cutter, p.tip, p.axis, cutter.radius(), tolerance, runner)?;
                out.report.counts.candidates += d.candidates;
                if d.offset > operation.tolerance {
                    gouges += 1;
                    first_gouge.get_or_insert(p.tip);
                } else {
                    clear.push(*p);
                }
            }
            let tilt_opts = TiltOptions {
                parts: vec![ToolPart::Shank, ToolPart::Holder],
                gouge_tolerance: Some(operation.tolerance),
                reach: cutter.radius(),
                pivot_offset: corner,
                ..Default::default()
            };
            let tilted = avoid_collisions(mesh, index, &profile, Some(cutter), &clear, &tilt_opts, tolerance, &mut out.report, runner)?;
            let cl: Vec<ClPose> = tilted.iter().filter(|t| !t.unreachable).map(|t| t.pose).collect();
            if cl.len() < 2 {
                continue;
            }
            let path_id = out.report.counts.passes as u32;
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
    }
    if fallbacks > 0 {
        out.report.warn(Diagnostic::new("link-fallback-clearance", format!("{fallbacks} traverses between runs were unproven and use the vertical clearance")));
    }
    if unproven_traverses > 0 {
        out.report.fail(Diagnostic::new("link-unproven", format!("{unproven_traverses} vertical-clearance traverses still touch material")));
    }
    if gouges > 0 {
        let mut d = Diagnostic::new("gouge-detected", format!("{gouges} contour CL points gouge beyond the machining tolerance"));
        if let Some(p) = first_gouge {
            d = d.at(p);
        }
        out.report.fail(d);
    }
    if out.motions.is_empty() && out.report.errors.is_empty() {
        out.report.fail(Diagnostic::new("empty-after-clipping", "no contour curve lies over the drive surface"));
    }
    out.report.counts.motions = out.motions.len() as u64;
    out.report.finish();
    Ok(out)
}

trait FlatRadius {
    fn flat_radius_or_zero(&self) -> f64;
}

impl FlatRadius for Cutter {
    fn flat_radius_or_zero(&self) -> f64 {
        match self {
            Cutter::Bull(b) => b.flat_radius(),
            _ => 0.0,
        }
    }
}

