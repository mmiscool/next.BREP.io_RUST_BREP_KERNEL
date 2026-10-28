//! Rest roughing (lane C6 row 5): contour-parallel clearing restricted to the
//! material a previous operation left, taken from the simulated dexel stock
//! (`RestSource::Stock`) or from what a larger reference tool could not reach
//! (`RestSource::ReferenceTool`). Per level the current tool's feasible set
//! is narrowed to the positions whose tool meets rest material, so the small
//! tool only runs where the big one could not; every pass is certified.

use crate::adaptive::dilate::grow;
use crate::adaptive::levels::{feasible_set, grid_slab_region, reference_tool_rest};
use crate::adaptive::roughing::{finish_stock_aware, resolve_levels};
use crate::link::Pass;
use crate::model::{Diagnostic, Operation, OperationReport, RestSource, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{invalid, loop_pass, non_empty, orient_for, OpContext};
use crate::planar::Region;
use crate::pocket3d::offset_rings;
use crate::stock::ResolvedStock;
use crate::stock_sim::DexelGrid;
use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};

/// Where the rest material comes from.
pub enum RestInput<'a> {
    /// The simulated stock after the previous operations.
    Stock(&'a DexelGrid),
    /// A larger tool that ran before; what it could not reach is the rest.
    ReferenceTool(&'a Cutter),
}

/// Rest roughing with `Pocket3D` parameters (depths, stepover, containment).
#[allow(clippy::too_many_arguments)]
pub fn generate(
    operation: &Operation,
    setup: &Setup,
    tool: &Tool,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    stock: &ResolvedStock,
    rest: RestInput<'_>,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    let params = match &operation.strategy {
        Strategy::Pocket3D(p) => p,
        _ => return Err(invalid("restRough", "requires Pocket3D parameters")),
    };
    let ctx = OpContext::new("restRough", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    let r = ctx.radius();
    let mut report = OperationReport::default();
    let mut stepover = params.stepover.distance;
    if stepover > r {
        report.warn(Diagnostic::new("stepover-reduced", format!("stepover {stepover} reduced to the cutter radius {r} for full coverage")));
        stepover = r;
    }
    let (source, grid, reference) = match rest {
        RestInput::Stock(g) => (RestSource::Stock, Some(g), None),
        RestInput::ReferenceTool(c) => {
            if c.radius() <= r + tol.distance_eps {
                return Err(invalid("restRough", "the reference tool must be larger than the current tool"));
            }
            (RestSource::ReferenceTool, None, Some(c))
        }
    };
    let input = resolve_levels(&ctx, &params.depths, 0.0, &params.containment, stock, None, stepover, runner)?;
    for w in &input.warnings {
        report.warn(w.clone());
    }
    let radial = operation.stock_to_leave.radial;
    let margin = crate::pocket2d::wall_margin(tol, operation.tolerance);
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    let mut rest_area = 0.0;
    runner.stage("rest-rough-levels", "Rest roughing levels", 0, input.levels.len() as u64)?;
    for (li, level) in input.levels.iter().enumerate() {
        if level.feasible.is_empty() || level.material.is_empty() {
            continue;
        }
        // The rest material of this level.
        let rest_material = match source {
            RestSource::Stock => {
                let left = grid_slab_region(grid.unwrap(), level.z, level.z_top, tol)?;
                left.intersection(&level.material, tol)?
            }
            RestSource::ReferenceTool => {
                let big = reference.unwrap().radius();
                let feasible_ref = feasible_set(&level.stock, &level.shadow, big, stepover, radial + margin, operation.tolerance, Some(&params.containment), tol)?;
                // The reference tool cleared up to its allowance line; what lies
                // beyond that reach is the rest (the allowance band is not).
                reference_tool_rest(&level.material, &feasible_ref, big + radial + margin, operation.tolerance, tol)?
            }
        };
        if rest_material.is_empty() {
            runner.work(1, "rest-rough-levels", li as u64 + 1, input.levels.len() as u64)?;
            continue;
        }
        rest_area += rest_material.area();
        // Tool positions whose tool meets rest material, inside the feasible set.
        let touching = grow(&rest_material, r, operation.tolerance, tol)?;
        let narrowed = level.feasible.intersection(&touching, tol)?;
        if narrowed.is_empty() {
            report.warn(Diagnostic::new("rest-unreachable", format!("rest material at Z={:.4} ({:.4} area) is out of the tool's reach", level.z, rest_material.area())));
            continue;
        }
        for shell in narrowed.shells(tol) {
            let region: Region = shell.region();
            let rings = offset_rings(&region, stepover, operation.tolerance, tol, "restRough", runner)?;
            let probe = shell.outer.segments[0].start;
            let in_air = level.stock.signed_distance(probe, tol) <= -(r - 4.0 * tol.distance_eps);
            let order: Vec<&Region> = if in_air { rings.iter().collect() } else { rings.iter().rev().collect() };
            for ring in order {
                for l in &ring.loops {
                    let touches = l.segments.iter().any(|s| rest_material.signed_distance(s.midpoint(), tol) > -(r - 4.0 * tol.distance_eps));
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
        runner.work(1, "rest-rough-levels", li as u64 + 1, input.levels.len() as u64)?;
    }
    if passes.is_empty() {
        return Err(invalid("restRough", format!("no rest material to remove ({rest_area:.4} area found, none reachable)")));
    }
    non_empty(&passes, "restRough")?;
    finish_stock_aware(&ctx, &passes, input.stock_top, None, report, input.reach.as_ref(), runner)
}
