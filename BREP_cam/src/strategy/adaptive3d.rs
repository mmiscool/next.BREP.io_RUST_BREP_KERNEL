//! 3D adaptive clearing (lane C6 row 3): constant-engagement roughing of a
//! part from its stock, level by level (depth levels, flat-area levels, fine
//! step-downs above flats). Per level the material is the stock slice minus
//! the part shadow and its allowance (or what the simulated stock still
//! holds for rest machining), the feasible set the stock grown by the load
//! minus the shadow grown by the radius and allowances; the solver starts in
//! the air around the stock and seeds helices in closed pockets. Every pass
//! is certified against the full protected mesh.

use crate::adaptive::clear::{clear_level, ClearInput};
use crate::adaptive::roughing::{finish_stock_aware, resolve_levels};
use crate::adaptive::strategy_support::{clear_params, engagement_summary, level_passes};
use crate::link::Pass;
use crate::model::{CutDirection, Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{invalid, non_empty, OpContext};
use crate::stock::ResolvedStock;
use crate::stock_sim::DexelGrid;
use crate::{CamError, MeshSnapshot, Runner, Tolerance, TriangleIndex};

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
        Strategy::Adaptive3D(p) => p,
        _ => return Err(invalid("adaptive3D", "requires Adaptive3D parameters")),
    };
    let ctx = OpContext::new("adaptive3D", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    let mut report = OperationReport::default();
    let cp = clear_params(&ctx, params.optimal_load, 0.0, CutDirection::Climb, &mut report);
    let rest_grid = if params.rest_machining { rest } else { None };
    let stock_used = match (params.rest_machining, rest) {
        (true, Some(_)) => &ResolvedStock::Rest(setup.id.clone()),
        _ => stock,
    };
    let input = resolve_levels(&ctx, &params.depths, params.fine_step_down, &params.containment, stock_used, rest_grid, cp.load, runner)?;
    for w in &input.warnings {
        report.warn(w.clone());
    }
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    let (mut max_theta, mut exceeded, mut loops, mut left) = (0.0f64, 0u64, 0usize, 0.0);
    let mut position = None;
    runner.stage("adaptive3d-levels", "Adaptive clearing levels", 0, input.levels.len() as u64)?;
    for (li, level) in input.levels.iter().enumerate() {
        if level.feasible.is_empty() || level.material.is_empty() {
            continue;
        }
        let result = clear_level(&ClearInput { feasible: &level.feasible, material: &level.material, entries: Vec::new(), start: position }, &cp, tol, runner)?;
        max_theta = max_theta.max(result.max_engagement);
        exceeded += result.exceeded;
        loops += result.loops;
        left += result.remaining_area;
        position = result.passes.last().and_then(|p| p.points.last().copied());
        passes.extend(level_passes(&ctx, &result, level.z, &mut path_id, &mut report, runner)?);
        runner.work(1, "adaptive3d-levels", li as u64 + 1, input.levels.len() as u64)?;
    }
    non_empty(&passes, "adaptive3D")?;
    engagement_summary(&mut report, cp.limit(), max_theta, exceeded, loops, left);
    finish_stock_aware(&ctx, &passes, input.stock_top, Some(cp.helix_diameter), report, input.reach.as_ref(), runner)
}
