//! Circular (lane C5 row 7): concentric circular passes clearing a hole
//! (inside out, down to a centre circle no larger than the cutter radius so
//! the centre is covered) or the ring around a boss out to the given outer
//! radius (outside in), at every depth level. Passes go through the linker
//! (helix/ramp entry, proven links) and the arc fitter refits the circles.

use crate::link::Pass;
use crate::model::{Operation, OperationReport, Point3, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{circle_points, depth_levels, invalid, non_empty, OpContext, PlanarInput};
use crate::pocket2d::wall_margin;
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
        Strategy::Circular(p) => p,
        _ => return Err(invalid("circular", "requires Circular parameters")),
    };
    let ctx = OpContext::new("circular", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if input.circles.is_empty() {
        return Err(invalid("circular", "no circular features selected"));
    }
    let r = ctx.radius();
    let radial = operation.stock_to_leave.radial;
    let step = params.stepover.distance.min(2.0 * r);
    let mut report = OperationReport::default();
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    runner.stage("circular", "Circular passes", 0, input.circles.len() as u64)?;
    for (ci, circle) in input.circles.iter().enumerate() {
        circle.validate("circular")?;
        let top = params.depths.top.unwrap_or(circle.top);
        // The linker keeps links above the stock this operation has not cut.
        ctx.top_of_cut(&params.depths, input);
        ctx.note_material_top(top);
        let bottom = params.depths.bottom.unwrap_or(circle.bottom) + operation.stock_to_leave.axial;
        let levels = depth_levels(top, bottom, &params.depths, "circular")?;
        let c = circle.center;
        // Radii, first to cut first.
        let mut radii: Vec<f64> = Vec::new();
        if circle.internal {
            let wall = circle.radius - r - radial - wall_margin(tol, operation.tolerance);
            if wall <= tol.sample_eps {
                return Err(invalid("circular", format!("hole {ci}: the cutter does not fit the hole")));
            }
            let mut rho = wall;
            while rho > tol.sample_eps {
                radii.push(rho);
                if rho <= r {
                    break;
                }
                rho -= step;
            }
            if *radii.last().unwrap() > r {
                radii.push(r.min(wall));
            }
            radii.reverse();
        } else {
            let outer = circle.outer_radius.ok_or_else(|| invalid("circular", format!("boss {ci} needs an outer radius (the stock) to clear to")))?;
            let wall = circle.radius + r + radial + wall_margin(tol, operation.tolerance);
            let limit = outer + r;
            let mut rho = wall;
            while rho <= limit {
                radii.push(rho);
                rho += step;
            }
            if radii.is_empty() {
                radii.push(wall);
            }
            radii.reverse();
        }
        for &z in &levels {
            for &rho in &radii {
                // Climb: counter-clockwise inside, clockwise outside.
                let pts: Vec<Point3> = circle_points(c, rho, circle.internal, z, operation.tolerance, !circle.internal);
                let pass = Pass::new(pts, Some(path_id));
                path_id += 1;
                ctx.certify(&pass.points, &mut report, runner)?;
                passes.push(pass);
            }
        }
        runner.work(1, "circular", ci as u64 + 1, input.circles.len() as u64)?;
    }
    non_empty(&passes, "circular")?;
    ctx.finish(&passes, false, report, runner)
}
