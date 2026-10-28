//! Trace (lane C5 row 6): the tool centre follows a chain, or runs tangent
//! to it on its left or right, in depth levels. A trace cuts along the
//! selected curve by design, so it is certified against fixtures, protected
//! bodies and avoid faces only; the target body itself is not gouge-checked
//! and the report says so.

use crate::link::Pass;
use crate::model::{Diagnostic, Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{chain_pass, depth_levels, invalid, non_empty, OpContext, PlanarInput};
use crate::planar::{offset_chain, Side};
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
        Strategy::Trace(p) => p,
        _ => return Err(invalid("trace", "requires Trace parameters")),
    };
    let ctx = OpContext::new("trace", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if input.chains.is_empty() {
        return Err(invalid("trace", "no chains selected"));
    }
    if !(-1..=1).contains(&params.side) {
        return Err(invalid("trace", "side must be -1 (left), 0 (on) or 1 (right)"));
    }
    let top = ctx.top_of_cut(&params.depths, input);
    let mut report = OperationReport::default();
    report.warn(Diagnostic::new("trace-model-not-checked", "trace cuts along the selected curve by design; only fixtures, protected bodies and avoid faces are gouge-checked"));
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    runner.stage("trace", "Trace passes", 0, input.chains.len() as u64)?;
    for (ci, planar) in input.chains.iter().enumerate() {
        let bottom = params.depths.bottom.or(ctx.heights.bottom).unwrap_or(planar.z) + operation.stock_to_leave.axial;
        let levels = depth_levels(top, bottom, &params.depths, "trace")?;
        // A path left of the chain has the chain (the material) on its right.
        let material_right = if params.side == 0 { None } else { Some(params.side < 0) };
        let chains = if params.side == 0 {
            vec![planar.chain.clone()]
        } else {
            let d = ctx.radius() + operation.stock_to_leave.radial + wall_margin(tol, operation.tolerance);
            let side = if params.side < 0 { Side::Left } else { Side::Right };
            let c = offset_chain(&planar.chain, d, side, tol)?;
            if c.is_empty() {
                report.warn(Diagnostic::new("trace-offset-vanished", format!("chain {ci}: the side offset vanished")));
            }
            c
        };
        for &z in &levels {
            for c in &chains {
                let pass = chain_pass(c, z, operation.tolerance, path_id, material_right);
                path_id += 1;
                ctx.certify_with(&pass.points, true, &mut report, runner)?;
                passes.push(pass);
            }
        }
        runner.work(1, "trace", ci as u64 + 1, input.chains.len() as u64)?;
    }
    non_empty(&passes, "trace")?;
    ctx.finish(&passes, false, report, runner)
}
