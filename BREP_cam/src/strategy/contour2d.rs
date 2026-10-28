//! 2D contour (lane C5 row 5): the cutter follows closed or open chains on
//! the outside or inside, with radial roughing passes, repeated finishing
//! passes, stock to leave, holding tabs and depth levels. Corners are rolled
//! (the exact offset's round joins). Leads and ramps come from the linker.

use crate::link::Pass;
use crate::model::{Diagnostic, Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{apply_tabs, chain_pass, depth_levels, invalid, loop_pass, non_empty, orient_for, OpContext, PlanarInput};
use crate::planar::{offset_chain, Loop, Region, Side};
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
        Strategy::Contour2D(p) => p,
        _ => return Err(invalid("contour2D", "requires Contour2D parameters")),
    };
    let ctx = OpContext::new("contour2D", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if input.chains.is_empty() {
        return Err(invalid("contour2D", "no contour chains selected"));
    }
    if params.roughing_passes > 0 && params.roughing_stepover <= 0.0 {
        return Err(invalid("contour2D", "roughing passes need a positive roughing stepover"));
    }
    for (i, t) in params.tabs.iter().enumerate() {
        if !t.at.is_finite() || !t.width.is_finite() || !t.height.is_finite() || t.width < 0.0 || t.height < 0.0 {
            return Err(invalid("contour2D", format!("tab {i} has an invalid position, width or height")));
        }
    }
    let r = ctx.radius();
    let radial = operation.stock_to_leave.radial;
    let top = ctx.top_of_cut(&params.depths, input);
    let mut report = OperationReport::default();
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    runner.stage("contour-offsets", "Contour offsets", 0, input.chains.len() as u64)?;
    for (ci, planar) in input.chains.iter().enumerate() {
        let bottom = params.depths.bottom.or(ctx.heights.bottom).unwrap_or(planar.z) + operation.stock_to_leave.axial;
        let levels = depth_levels(top, bottom, &params.depths, "contour2D")?;
        // Radial distances, outermost roughing pass first, then the finishing passes.
        let base = r + radial + wall_margin(tol, operation.tolerance);
        let mut distances: Vec<f64> = (1..=params.roughing_passes).rev().map(|k| base + k as f64 * params.roughing_stepover).collect();
        for _ in 0..params.finishing_passes.max(1) {
            distances.push(base);
        }
        // Each distance gives the loops/chains to follow (geometry is depth independent).
        let mut per_distance: Vec<Vec<Pass>> = Vec::new();
        if planar.chain.closed {
            let region = Region::from_unoriented(vec![Loop::new(planar.chain.segments.clone())], tol)?;
            for &d in &distances {
                let off = if params.outside { region.offset(d, tol)? } else { region.offset(-d, tol)? };
                if off.is_empty() {
                    report.warn(Diagnostic::new("contour-too-narrow", format!("chain {ci}: the inside offset at {d} is empty; the cutter does not fit")));
                    continue;
                }
                let mut group = Vec::new();
                for l in &off.loops {
                    // Outside: material inside the loop (left of CCW); inside: material outside.
                    let (oriented, mr) = orient_for(l, !params.outside, params.direction, false);
                    group.push(loop_pass(&oriented, 0.0, operation.tolerance, params.finishing_overlap, 0, Some(mr)));
                }
                per_distance.push(group);
            }
        } else {
            let side = if params.outside { Side::Right } else { Side::Left };
            for &d in &distances {
                let chains = offset_chain(&planar.chain, d, side, tol)?;
                if chains.is_empty() {
                    report.warn(Diagnostic::new("contour-too-narrow", format!("chain {ci}: the side offset at {d} vanished")));
                    continue;
                }
                let mut group = Vec::new();
                // Offset to the right of the chain: the material (the chain
                // side) is on the left of the pass; reversed for conventional.
                let conventional = params.direction == crate::model::CutDirection::Conventional;
                let material_right = (side == Side::Left) != conventional;
                for c in &chains {
                    let c = if conventional { c.reversed() } else { c.clone() };
                    group.push(chain_pass(&c, 0.0, operation.tolerance, 0, Some(material_right)));
                }
                per_distance.push(group);
            }
        }
        if per_distance.is_empty() {
            continue;
        }
        for &z in &levels {
            for group in &per_distance {
                for template in group {
                    let mut pass = Pass::new(template.points.iter().map(|p| crate::model::Point3::new(p.x, p.y, z)).collect(), Some(path_id));
                    path_id += 1;
                    ctx.certify(&pass.points, &mut report, runner)?;
                    apply_tabs(&mut pass, &params.tabs, bottom);
                    passes.push(pass);
                }
            }
        }
        runner.work(1, "contour-offsets", ci as u64 + 1, input.chains.len() as u64)?;
    }
    non_empty(&passes, "contour2D")?;
    ctx.finish(&passes, false, report, runner)
}
