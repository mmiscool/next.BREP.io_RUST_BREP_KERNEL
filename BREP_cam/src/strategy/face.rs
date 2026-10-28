//! Facing (lane C5 row 3): parallel passes over the stock top down to the
//! finished face in depth levels. The pass area is the selected faces'
//! outlines (or the stock outline) grown by the cutter radius plus the pass
//! extension, so the cutter fully clears every edge; passes are zig-zag or
//! one-way at the pass angle. Every pass is certified against the full
//! protected mesh before linking.

use crate::model::{Operation, Point2, Point3, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{depth_levels, invalid, non_empty, OpContext, PlanarInput};
use crate::planar::{clip_line, Loop, Region, Vec2};
use crate::link::Pass;
use crate::{CamError, MeshSnapshot, Runner, Tolerance, TriangleIndex};

#[allow(clippy::too_many_arguments)]
pub fn generate(
    operation: &Operation,
    setup: &Setup,
    tool: &Tool,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    input: &PlanarInput,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    let params = match &operation.strategy {
        Strategy::Face(p) => p,
        _ => return Err(invalid("face", "requires Face parameters")),
    };
    let ctx = OpContext::new("face", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if !params.pass_extension.is_finite() || params.pass_extension < 0.0 {
        return Err(invalid("face", "pass extension must be finite and non-negative"));
    }
    if !params.pass_angle_deg.is_finite() {
        return Err(invalid("face", "pass angle is not finite"));
    }
    // The area to face: the selected regions, else the stock outline.
    let (area, face_z) = if !input.regions.is_empty() {
        let mut acc = Region::empty();
        let mut z = f64::INFINITY;
        for r in &input.regions {
            acc = if acc.is_empty() { r.region.clone() } else { acc.union(&r.region, tol)? };
            z = z.min(r.z);
        }
        (acc, Some(z))
    } else {
        let model_bounds = setup.machine_to_model().apply_bounds(&mesh.bounds);
        let stock = crate::stock::resolve_stock(setup, &model_bounds, None).map_err(|e| invalid("face", e))?;
        let bounds = stock.machine_bounds().ok_or_else(|| invalid("face", "the stock has no geometric outline; select faces to face"))?;
        (Region::new(vec![Loop::rectangle(Point2::new(bounds.min.x, bounds.min.y), Point2::new(bounds.max.x, bounds.max.y))], tol)?, None)
    };
    if area.is_empty() {
        return Err(invalid("face", "nothing to face: the selected regions are empty"));
    }
    let top = ctx.top_of_cut(&params.depths, input);
    let bottom = params.depths.bottom.or(ctx.heights.bottom).or(face_z).ok_or_else(|| invalid("face", "no bottom of cut: select faces or set the depth"))?
        + operation.stock_to_leave.axial;
    let levels = depth_levels(top, bottom, &params.depths, "face")?;
    let cover = area.offset(ctx.radius() + params.pass_extension, tol)?;
    let (lo, hi) = cover.bounds();
    let (s, c) = params.pass_angle_deg.to_radians().sin_cos();
    let u = Point2::new(c, s);
    let v = u.perp();
    // Extent of the cover along u and v.
    let corners = [lo, Point2::new(hi.x, lo.y), hi, Point2::new(lo.x, hi.y)];
    let umin = corners.iter().map(|p| p.dot(u)).fold(f64::INFINITY, f64::min) - 1.0;
    let umax = corners.iter().map(|p| p.dot(u)).fold(f64::NEG_INFINITY, f64::max) + 1.0;
    let vmin = corners.iter().map(|p| p.dot(v)).fold(f64::INFINITY, f64::min);
    let vmax = corners.iter().map(|p| p.dot(v)).fold(f64::NEG_INFINITY, f64::max);
    let step = params.stepover.distance;
    let count = ((vmax - vmin) / step).ceil().max(1.0);
    if !count.is_finite() || count > 1_000_000.0 {
        return Err(invalid("face", "pass budget exceeded"));
    }
    let count = count as usize;
    // The first and last pass sit half a cutter inside the cover edge when
    // possible, so the edge is fully swept with the least passes.
    let inset = ctx.radius().min(0.5 * (vmax - vmin));
    let (v0, v1) = (vmin + inset, vmax - inset);
    let n = if v1 > v0 { ((v1 - v0) / step).ceil().max(1.0) as usize } else { 1 };
    let _ = count;
    let mut report = crate::model::OperationReport::default();
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    runner.stage("face-levels", "Facing passes", 0, levels.len() as u64)?;
    for (li, &z) in levels.iter().enumerate() {
        let mut forward = true;
        for i in 0..=n {
            let vv = if n == 0 { v0 } else { v0 + (v1 - v0) * i as f64 / n as f64 };
            let a = u.scale(umin).add(v.scale(vv));
            let b = u.scale(umax).add(v.scale(vv));
            let mut spans: Vec<(Point2, Point2)> = clip_line(&cover, a, b, tol).into_iter().map(|(t0, t1)| (a.add(b.sub(a).scale(t0)), a.add(b.sub(a).scale(t1)))).collect();
            if !forward {
                spans.reverse();
            }
            for (p, q) in spans {
                let (p, q) = if forward { (p, q) } else { (q, p) };
                if p.distance(q) <= tol.sample_eps {
                    continue;
                }
                let pass = Pass::new(vec![Point3::new(p.x, p.y, z), Point3::new(q.x, q.y, z)], Some(path_id));
                ctx.certify(&pass.points, &mut report, runner)?;
                passes.push(pass);
                path_id += 1;
            }
            if params.zig_zag {
                forward = !forward;
            }
        }
        runner.work(1, "face-levels", li as u64 + 1, levels.len() as u64)?;
    }
    non_empty(&passes, "face")?;
    ctx.finish(&passes, false, report, runner)
}
