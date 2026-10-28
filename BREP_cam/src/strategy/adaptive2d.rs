//! 2D adaptive clearing (lane C6 row 2): constant-engagement roughing of
//! planar regions with islands, in depth levels. Each region's material is
//! the region itself (or what the simulated stock still holds in it); the
//! tool-centre feasible set is the region eroded by the radius and the
//! allowances; predrilled holes (internal circles of the input at least the
//! tool's size) are the entries, else the solver seeds a helix at the
//! deepest point. The engagement never exceeds `arccos(1 - load / r)` except
//! where reported.

use crate::adaptive::clear::{clear_level, ClearInput, Entry};
use crate::adaptive::dilate::shrink;
use crate::adaptive::levels::grid_slab_region;
use crate::adaptive::roughing::finish_stock_aware;
use crate::adaptive::strategy_support::{clear_params, engagement_summary, level_passes};
use crate::link::Pass;
use crate::model::{Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{depth_levels, invalid, non_empty, OpContext, PlanarInput};
use crate::stock_sim::DexelGrid;
use crate::{CamError, MeshSnapshot, Runner, Tolerance, TriangleIndex};

#[allow(clippy::too_many_arguments)]
pub fn generate(
    operation: &Operation,
    setup: &Setup,
    tool: &Tool,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    input: &PlanarInput,
    rest: Option<&DexelGrid>,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    let params = match &operation.strategy {
        Strategy::Adaptive2D(p) => p,
        _ => return Err(invalid("adaptive2D", "requires Adaptive2D parameters")),
    };
    let ctx = OpContext::new("adaptive2D", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if input.regions.is_empty() {
        return Err(invalid("adaptive2D", "no regions selected"));
    }
    let r = ctx.radius();
    let mut report = OperationReport::default();
    let cp = clear_params(&ctx, params.optimal_load, params.minimum_cutting_radius, params.direction, &mut report);
    let radial = operation.stock_to_leave.radial;
    let margin = crate::pocket2d::wall_margin(tol, operation.tolerance);
    let top = ctx.top_of_cut(&params.depths, input);
    let stock_top = input.stock_top.unwrap_or(top).max(top);
    let rest_grid = if params.rest_machining { rest } else { None };
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    let (mut max_theta, mut exceeded, mut loops, mut left) = (0.0f64, 0u64, 0usize, 0.0);
    let mut limit = cp.limit();
    runner.stage("adaptive2d-regions", "Adaptive clearing regions", 0, input.regions.len() as u64)?;
    for (ri, planar) in input.regions.iter().enumerate() {
        let bottom = params.depths.bottom.or(ctx.heights.bottom).unwrap_or(planar.z) + operation.stock_to_leave.axial;
        let levels = depth_levels(top, bottom, &params.depths, "adaptive2D")?;
        let feasible = shrink(&planar.region, r + radial + margin, operation.tolerance, tol)?;
        if feasible.is_empty() {
            report.warn(crate::model::Diagnostic::new("region-too-narrow", format!("region {ri} is narrower than the cutter and was skipped")));
            continue;
        }
        // Predrilled holes inside the region are entries.
        let entries: Vec<Entry> = input
            .circles
            .iter()
            .filter(|c| c.internal && c.radius >= r - tol.distance_eps && feasible.contains(c.center, tol))
            .map(|c| Entry { center: c.center, cleared_radius: c.radius, helix: false })
            .collect();
        let mut position = None;
        for (li, &z) in levels.iter().enumerate() {
            let material = match rest_grid {
                Some(g) => {
                    let z_top = if li == 0 { top } else { levels[li - 1] };
                    grid_slab_region(g, z, z_top, tol)?.intersection(&planar.region, tol)?
                }
                None => planar.region.clone(),
            };
            if material.is_empty() {
                continue;
            }
            let result = clear_level(&ClearInput { feasible: &feasible, material: &material, entries: entries.clone(), start: position }, &cp, tol, runner)?;
            limit = result.limit;
            max_theta = max_theta.max(result.max_engagement);
            exceeded += result.exceeded;
            loops += result.loops;
            left += result.remaining_area;
            position = result.passes.last().and_then(|p| p.points.last().copied());
            passes.extend(level_passes(&ctx, &result, z, &mut path_id, &mut report, runner)?);
        }
        runner.work(1, "adaptive2d-regions", ri as u64 + 1, input.regions.len() as u64)?;
    }
    non_empty(&passes, "adaptive2D")?;
    engagement_summary(&mut report, limit, max_theta, exceeded, loops, left);
    finish_stock_aware(&ctx, &passes, stock_top, Some(cp.helix_diameter), report, None, runner)
}
