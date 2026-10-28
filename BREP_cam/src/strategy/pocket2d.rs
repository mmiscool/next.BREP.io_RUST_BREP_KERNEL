//! 2D pocket (lane C5 row 4): contour-parallel clearing of planar regions
//! with islands, inside out, in depth levels, with a wall finishing pass
//! and radial/axial stock to leave. Offsets are exact (arcs preserved) and
//! the stepover is capped at the cutter radius so the core of every level
//! is covered. Entry (plunge, helix, ramp) and links come from the linker.

use crate::link::Pass;
use crate::model::{Diagnostic, Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{depth_levels, invalid, loop_pass, non_empty, orient_for, OpContext, PlanarInput};
use crate::planar::Region;
use crate::{CamError, MeshSnapshot, Runner, Tolerance, TriangleIndex};

/// Extra tool-centre distance kept off every wall: the machining tolerance
/// (chords and fitted arcs deviate within it, so they never enter material)
/// plus a provable hair above the certificate's contact epsilon.
pub fn wall_margin(tol: &Tolerance, machining_tolerance: f64) -> f64 {
    machining_tolerance.max(0.0) + 4.0 * tol.distance_eps
}

/// Contour-parallel loops of one region at wall distance `d0` and
/// `stepover`, outermost first: `levels[k]` is the region offset inward by
/// `d0 + k * stepover`, until the offset is empty.
pub fn clearing_levels(region: &Region, d0: f64, stepover: f64, tol: &Tolerance, strategy: &str) -> Result<Vec<Region>, CamError> {
    let mut out = Vec::new();
    for k in 0..100_000usize {
        let r = region.offset(-(d0 + k as f64 * stepover), tol)?;
        if r.is_empty() {
            break;
        }
        out.push(r);
    }
    if out.len() >= 100_000 {
        return Err(invalid(strategy, "clearing level budget exceeded"));
    }
    Ok(out)
}

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
        Strategy::Pocket2D(p) => p,
        _ => return Err(invalid("pocket2D", "requires Pocket2D parameters")),
    };
    let ctx = OpContext::new("pocket2D", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if input.regions.is_empty() {
        return Err(invalid("pocket2D", "no pocket regions selected"));
    }
    let r = ctx.radius();
    let mut report = OperationReport::default();
    let mut stepover = params.stepover.distance;
    if stepover > r {
        report.warn(Diagnostic::new("stepover-reduced", format!("pocket stepover {stepover} reduced to the cutter radius {r} for full coverage")));
        stepover = r;
    }
    let radial = operation.stock_to_leave.radial;
    let d_wall = r + radial + wall_margin(tol, operation.tolerance);
    let top = ctx.top_of_cut(&params.depths, input);
    // One job per (shell, level): the loops to cut at that depth.
    struct Job {
        z: f64,
        shell: usize,
        passes: Vec<Pass>,
    }
    let mut jobs: Vec<Job> = Vec::new();
    let mut path_id = 0u32;
    let mut shell_index = 0usize;
    runner.stage("pocket-offsets", "Pocket offsets", 0, input.regions.len() as u64)?;
    for (ri, planar) in input.regions.iter().enumerate() {
        let bottom = params.depths.bottom.or(ctx.heights.bottom).unwrap_or(planar.z) + operation.stock_to_leave.axial;
        let levels = depth_levels(top, bottom, &params.depths, "pocket2D")?;
        for shell in planar.region.shells(tol) {
            let mut region = shell.region();
            if !params.keep_islands {
                region = Region { loops: vec![shell.outer.clone()] };
            }
            let clearing = clearing_levels(&region, d_wall, stepover, tol, "pocket2D")?;
            if clearing.is_empty() {
                report.warn(Diagnostic::new("pocket-too-narrow", format!("a pocket of region {ri} is narrower than the cutter and was skipped")));
                continue;
            }
            let finish = if params.wall_finish && radial > 0.0 { Some(region.offset(-(r + wall_margin(tol, operation.tolerance)), tol)?) } else { None };
            for &z in &levels {
                let mut passes = Vec::new();
                // Inside out: innermost level first, the wall pass last.
                for level in clearing.iter().rev() {
                    for l in &level.loops {
                        let (oriented, mr) = orient_for(l, true, params.stepover.direction, false);
                        passes.push(loop_pass(&oriented, z, operation.tolerance, 0.0, path_id, Some(mr)));
                        path_id += 1;
                    }
                }
                if let Some(f) = &finish {
                    for l in &f.loops {
                        let (oriented, mr) = orient_for(l, true, params.stepover.direction, false);
                        passes.push(loop_pass(&oriented, z, operation.tolerance, 0.0, path_id, Some(mr)));
                        path_id += 1;
                    }
                }
                jobs.push(Job { z, shell: shell_index, passes });
            }
            shell_index += 1;
        }
        runner.work(1, "pocket-offsets", ri as u64 + 1, input.regions.len() as u64)?;
    }
    // Order: depth first (every level of a shell, then the next shell) or
    // level first (every shell at a depth, then the next depth).
    if params.depths.depth_first {
        jobs.sort_by(|a, b| a.shell.cmp(&b.shell).then(b.z.total_cmp(&a.z)));
    } else {
        jobs.sort_by(|a, b| b.z.total_cmp(&a.z).then(a.shell.cmp(&b.shell)));
    }
    let mut passes: Vec<Pass> = Vec::new();
    runner.stage("pocket-certify", "Certify pocket passes", 0, jobs.len() as u64)?;
    for (ji, job) in jobs.into_iter().enumerate() {
        for pass in job.passes {
            ctx.certify(&pass.points, &mut report, runner)?;
            passes.push(pass);
        }
        runner.work(1, "pocket-certify", ji as u64 + 1, 0)?;
    }
    non_empty(&passes, "pocket2D")?;
    ctx.finish(&passes, false, report, runner)
}
