//! Engrave and V-carve (lane C5 row 6). Engraving runs the tool centre
//! along chains in depth levels. V-carving fills closed regions with a
//! V-bit whose depth follows the local width: at depth `h` the bit of half
//! angle `a` is `h tan a` wide, so the region offset inward by `h tan a` at
//! machine Z `top - h` is the pass whose flank touches the outline; the
//! levels descend by the step-down until the offset empties (or the bottom
//! is reached), which carves the region without a medial axis. Both cut
//! into the model by design and are gouge-checked against fixtures,
//! protected bodies and avoid faces only.

use crate::link::Pass;
use crate::model::{Diagnostic, Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{chain_pass, depth_levels, invalid, loop_pass, non_empty, OpContext, PlanarInput};
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
        Strategy::Engrave(p) => p,
        _ => return Err(invalid("engrave", "requires Engrave parameters")),
    };
    let ctx = OpContext::new("engrave", operation, setup, tool, mesh, index, tolerance)?;
    // The linker keeps links above the stock this operation has not cut.
    ctx.top_of_cut(&params.depths, input);
    let tol = &ctx.tolerance;
    let mut report = OperationReport::default();
    report.warn(Diagnostic::new("engrave-model-not-checked", "engraving cuts into the model by design; only fixtures, protected bodies and avoid faces are gouge-checked"));
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    if params.v_carve {
        let Cutter::Cone(cone) = &ctx.cutter else {
            return Err(CamError::UnsupportedCutter { strategy: "engrave (v-carve)".into(), cutter: ctx.cutter.kind().to_string() });
        };
        if input.regions.is_empty() {
            return Err(invalid("engrave", "v-carve needs closed regions"));
        }
        let step = params.depths.step_down;
        if step <= 0.0 {
            return Err(invalid("engrave", "v-carve needs a positive step down"));
        }
        let t = cone.half_angle_rad().tan();
        runner.stage("v-carve", "V-carve levels", 0, input.regions.len() as u64)?;
        for (ri, planar) in input.regions.iter().enumerate() {
            let top = params.depths.top.unwrap_or(planar.z);
            let max_depth = params.depths.bottom.or(ctx.heights.bottom).map_or(cone.cone_height(), |b| (top - b).max(0.0)).min(cone.cone_height());
            let mut h = step.min(max_depth);
            let mut levels = 0u32;
            loop {
                let off = planar.region.offset(-(h * t + operation.stock_to_leave.radial), tol)?;
                if off.is_empty() {
                    break;
                }
                let z = top - h + operation.stock_to_leave.axial;
                for l in &off.loops {
                    let pass = loop_pass(l, z, operation.tolerance, 0.0, path_id, None);
                    path_id += 1;
                    ctx.certify_with(&pass.points, true, &mut report, runner)?;
                    passes.push(pass);
                }
                levels += 1;
                if h >= max_depth - 1e-9 || levels > 100_000 {
                    break;
                }
                h = (h + step).min(max_depth);
            }
            if levels == 0 {
                report.warn(Diagnostic::new("v-carve-empty", format!("region {ri} is narrower than the first V-carve level")));
            }
            runner.work(1, "v-carve", ri as u64 + 1, input.regions.len() as u64)?;
        }
    } else {
        if input.chains.is_empty() {
            return Err(invalid("engrave", "no chains selected"));
        }
        runner.stage("engrave", "Engrave passes", 0, input.chains.len() as u64)?;
        for (ci, planar) in input.chains.iter().enumerate() {
            let top = params.depths.top.unwrap_or(planar.z);
            let bottom = params.depths.bottom.or(ctx.heights.bottom).ok_or_else(|| invalid("engrave", "engraving needs a bottom depth"))? + operation.stock_to_leave.axial;
            let levels = depth_levels(top, bottom, &params.depths, "engrave")?;
            for &z in &levels {
                let pass = chain_pass(&planar.chain, z, operation.tolerance, path_id, None);
                path_id += 1;
                ctx.certify_with(&pass.points, true, &mut report, runner)?;
                passes.push(pass);
            }
            runner.work(1, "engrave", ci as u64 + 1, input.chains.len() as u64)?;
        }
    }
    non_empty(&passes, "engrave")?;
    ctx.finish(&passes, false, report, runner)
}
