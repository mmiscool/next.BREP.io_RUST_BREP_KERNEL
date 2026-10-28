//! Slot (lane C5 row 6): the cutter follows the slot centre line when the
//! slot is the cutter's width, else clears the slot's capsule outline
//! (centre line grown by half the width) like a pocket, in depth levels.

use crate::link::Pass;
use crate::model::{Diagnostic, Operation, OperationReport, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{chain_pass, depth_levels, invalid, loop_pass, non_empty, orient_for, OpContext, PlanarInput};
use crate::planar::chain_capsule;
use crate::pocket2d::{clearing_levels, wall_margin};
use crate::{CamError, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};

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
        Strategy::Slot(p) => p,
        _ => return Err(invalid("slot", "requires Slot parameters")),
    };
    let ctx = OpContext::new("slot", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if input.chains.is_empty() {
        return Err(invalid("slot", "no slot centre lines selected"));
    }
    if params.trochoidal {
        return Err(invalid("slot", "trochoidal slotting is not implemented; use a plain slot or 2D adaptive"));
    }
    let r = ctx.radius();
    let radial = operation.stock_to_leave.radial;
    let half = 0.5 * params.width - radial;
    if half < r - tol.distance_eps {
        return Err(invalid("slot", format!("slot width {} (minus radial stock) is narrower than the cutter diameter {}", params.width, 2.0 * r)));
    }
    let top = ctx.top_of_cut(&params.depths, input);
    let mut report = OperationReport::default();
    let mut passes: Vec<Pass> = Vec::new();
    let mut path_id = 0u32;
    runner.stage("slot", "Slot passes", 0, input.chains.len() as u64)?;
    for (ci, planar) in input.chains.iter().enumerate() {
        let bottom = params.depths.bottom.or(ctx.heights.bottom).unwrap_or(planar.z) + operation.stock_to_leave.axial;
        let levels = depth_levels(top, bottom, &params.depths, "slot")?;
        // Templates at z = 0.
        let full_width = half - r <= wall_margin(tol, operation.tolerance);
        let templates: Vec<Pass> = if full_width {
            // The cutter is the slot's width: one pass along the centre line.
            vec![chain_pass(&planar.chain, 0.0, operation.tolerance, 0, None)]
        } else {
            let capsule = chain_capsule(&planar.chain, half, tol)?;
            let clearing = clearing_levels(&capsule, r + wall_margin(tol, operation.tolerance), r, tol, "slot")?;
            if clearing.is_empty() {
                report.warn(Diagnostic::new("slot-too-narrow", format!("slot {ci} is narrower than the cutter and was skipped")));
                continue;
            }
            let mut t = Vec::new();
            for level in clearing.iter().rev() {
                for l in &level.loops {
                    let (oriented, mr) = orient_for(l, true, crate::model::CutDirection::Climb, false);
                    t.push(loop_pass(&oriented, 0.0, operation.tolerance, 0.0, 0, Some(mr)));
                }
            }
            t
        };
        // A full-width slot's walls are the cut itself: the certificate uses the
        // cutter shrunk by twice the machining tolerance (the tessellation's
        // chords and the exact contact lie within that band).
        let certifier = if full_width {
            match &ctx.cutter {
                crate::Cutter::Flat(_) => crate::Cutter::flat(2.0 * (r - 2.0 * operation.tolerance).max(tol.sample_eps), ctx.cutter.cutting_length(), ctx.cutter.shaft_length()),
                _ => return Err(invalid("slot", "a slot equal to the cutter width needs a flat cutter")),
            }
        } else {
            ctx.cutter
        };
        for &z in &levels {
            for template in &templates {
                let pass = Pass::new(template.points.iter().map(|p| crate::model::Point3::new(p.x, p.y, z)).collect(), Some(path_id));
                path_id += 1;
                ctx.certify_with_cutter(&pass.points, &certifier, false, &mut report, runner)?;
                passes.push(pass);
            }
        }
        runner.work(1, "slot", ci as u64 + 1, input.chains.len() as u64)?;
    }
    non_empty(&passes, "slot")?;
    ctx.finish(&passes, false, report, runner)
}
