//! 3D pocket clearing (lane C6 row 4): the classic contour-parallel roughing
//! of a part from its stock, level by level. Every level's tool-centre
//! feasible set (stock slice grown by the stepover, minus the part shadow
//! grown by the radius and allowances) is eroded by the stepover until it
//! vanishes; the loops are cut outside in where a shell starts in the air
//! around the stock, inside out (an entry at the innermost loop) where it is
//! a closed pocket. Each pass is certified against the full protected mesh;
//! rest machining takes the material from the simulated dexel stock.

use crate::adaptive::roughing::{finish_stock_aware, resolve_levels};
use crate::link::Pass;
use crate::model::{Diagnostic, Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{invalid, loop_pass, non_empty, orient_for, OpContext};
use crate::planar::Region;
use crate::stock::ResolvedStock;
use crate::stock_sim::DexelGrid;
use crate::{CamError, MeshSnapshot, Runner, Tolerance, TriangleIndex};

pub use crate::adaptive::levels::offset_rings;

#[allow(clippy::too_many_arguments)]
pub fn generate(
    operation: &Operation,
    setup: &Setup,
    tool: &Tool,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    stock: &ResolvedStock,
    rest: Option<&DexelGrid>,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    let params = match &operation.strategy {
        Strategy::Pocket3D(p) => p,
        _ => return Err(invalid("pocket3D", "requires Pocket3D parameters")),
    };
    let ctx = OpContext::new("pocket3D", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    let r = ctx.radius();
    let mut report = OperationReport::default();
    let mut stepover = params.stepover.distance;
    if stepover > r {
        report.warn(Diagnostic::new("stepover-reduced", format!("pocket stepover {stepover} reduced to the cutter radius {r} for full coverage")));
        stepover = r;
    }
    let rest_grid = if params.rest_machining { rest } else { None };
    let stock_used = match (params.rest_machining, rest) {
        (true, Some(_)) => &ResolvedStock::Rest(setup.id.clone()),
        _ => stock,
    };
    let input = resolve_levels(&ctx, &params.depths, 0.0, &params.containment, stock_used, rest_grid, stepover, runner)?;
    for w in &input.warnings {
        report.warn(w.clone());
    }
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    runner.stage("pocket3d-levels", "Pocket clearing levels", 0, input.levels.len() as u64)?;
    for (li, level) in input.levels.iter().enumerate() {
        if level.feasible.is_empty() || level.material.is_empty() {
            runner.work(1, "pocket3d-levels", li as u64 + 1, input.levels.len() as u64)?;
            continue;
        }
        for shell in level.feasible.shells(tol) {
            let region = shell.region();
            let rings = offset_rings(&region, stepover, operation.tolerance, tol, "pocket3D", runner)?;
            // Outside in when the outer loop starts in the air around the stock.
            let probe = shell.outer.segments[0].start;
            let in_air = level.stock.signed_distance(probe, tol) <= -(r - 4.0 * tol.distance_eps);
            let order: Vec<&Region> = if in_air { rings.iter().collect() } else { rings.iter().rev().collect() };
            for ring in order {
                for l in &ring.loops {
                    // Skip loops whose tool cuts nothing (entirely in air).
                    let touches = l.segments.iter().any(|s| level.material.signed_distance(s.midpoint(), tol) > -(r - 4.0 * tol.distance_eps));
                    if !touches {
                        continue;
                    }
                    let (oriented, mr) = orient_for(l, true, params.stepover.direction, false);
                    let pass = loop_pass(&oriented, level.z, operation.tolerance, 0.0, path_id, Some(mr));
                    ctx.certify(&pass.points, &mut report, runner)?;
                    passes.push(pass);
                    path_id += 1;
                }
            }
        }
        runner.work(1, "pocket3d-levels", li as u64 + 1, input.levels.len() as u64)?;
    }
    non_empty(&passes, "pocket3D")?;
    finish_stock_aware(&ctx, &passes, input.stock_top, None, report, input.reach.as_ref(), runner)
}
