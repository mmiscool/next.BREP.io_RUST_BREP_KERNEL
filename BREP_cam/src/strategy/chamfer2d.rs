//! 2D chamfer (lane C5 row 6): a chamfer mill or V-bit runs along the
//! selected top edges with its tip `width + tip_offset` below the edge and
//! its axis offset from the edge by `tip_offset * tan(half angle)`, so the
//! cone's flank cuts a chamfer of the given width (for a 45-degree cutter
//! the legs are equal). The chamfer removes model material by design, so
//! only fixtures, protected bodies and avoid faces are gouge-checked.

use crate::link::Pass;
use crate::model::{Diagnostic, Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{chain_pass, invalid, loop_pass, non_empty, OpContext, PlanarInput};
use crate::planar::{offset_chain, Loop, Region, Side};
use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};

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
        Strategy::Chamfer2D(p) => p,
        _ => return Err(invalid("chamfer2D", "requires Chamfer2D parameters")),
    };
    let ctx = OpContext::new("chamfer2D", operation, setup, tool, mesh, index, tolerance)?;
    // The linker keeps links above the stock this operation has not cut.
    ctx.top_of_cut(&Default::default(), input);
    let tol = &ctx.tolerance;
    let Cutter::Cone(cone) = &ctx.cutter else {
        return Err(CamError::UnsupportedCutter { strategy: "chamfer2D".into(), cutter: ctx.cutter.kind().to_string() });
    };
    if input.chains.is_empty() {
        return Err(invalid("chamfer2D", "no edges selected"));
    }
    let half = cone.half_angle_rad();
    let depth = params.width + params.tip_offset;
    if depth > cone.cone_height() + tol.distance_eps {
        return Err(invalid("chamfer2D", format!("chamfer width plus tip offset {depth} exceeds the cutter's cone height {}", cone.cone_height())));
    }
    let lateral = params.tip_offset * half.tan() + operation.stock_to_leave.radial;
    let mut report = OperationReport::default();
    report.warn(Diagnostic::new("chamfer-model-not-checked", "the chamfer removes model material by design; only fixtures, protected bodies and avoid faces are gouge-checked"));
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    runner.stage("chamfer", "Chamfer passes", 0, input.chains.len() as u64)?;
    for (ci, planar) in input.chains.iter().enumerate() {
        let z = planar.z - depth + operation.stock_to_leave.axial;
        if planar.chain.closed {
            // An outside edge: the region inside the loop is the part.
            let region = Region::from_unoriented(vec![Loop::new(planar.chain.segments.clone())], tol)?;
            let off = if lateral > tol.distance_eps { region.offset(lateral, tol)? } else { region.offset(0.0, tol)? };
            for l in &off.loops {
                // Climb on an outside edge: clockwise (material on the right).
                let pass = loop_pass(&l.reversed(), z, operation.tolerance, 0.0, path_id, Some(true));
                path_id += 1;
                ctx.certify_with(&pass.points, true, &mut report, runner)?;
                passes.push(pass);
            }
        } else {
            let chains = if lateral > tol.distance_eps { offset_chain(&planar.chain, lateral, Side::Right, tol)? } else { vec![planar.chain.clone()] };
            if chains.is_empty() {
                report.warn(Diagnostic::new("chamfer-offset-vanished", format!("chain {ci}: the offset vanished")));
            }
            for c in &chains {
                let pass = chain_pass(c, z, operation.tolerance, path_id, Some(false));
                path_id += 1;
                ctx.certify_with(&pass.points, true, &mut report, runner)?;
                passes.push(pass);
            }
        }
        runner.work(1, "chamfer", ci as u64 + 1, input.chains.len() as u64)?;
    }
    non_empty(&passes, "chamfer2D")?;
    ctx.finish(&passes, false, report, runner)
}
