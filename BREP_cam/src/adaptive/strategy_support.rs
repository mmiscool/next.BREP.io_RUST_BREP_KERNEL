//! What the adaptive strategies share: turning a level's clearing result into
//! certified passes and a report.

use super::clear::{ClearParams, LevelResult};
use crate::link::Pass;
use crate::model::{CutDirection, Diagnostic, OperationReport, Point3};
use crate::planar::machining::OpContext;
use crate::{CamError, Runner};

/// Solver parameters from the operation: load capped at the radius, the
/// linker's helix diameter (its default is the cutter radius), the chord
/// tolerance and the cut direction.
pub fn clear_params(ctx: &OpContext<'_>, optimal_load: f64, minimum_cutting_radius: f64, direction: CutDirection, report: &mut OperationReport) -> ClearParams {
    let r = ctx.radius();
    let mut load = optimal_load;
    if load > r {
        report.warn(Diagnostic::new("load-reduced", format!("optimal load {load} reduced to the cutter radius {r}")));
        load = r;
    }
    let ramp = &ctx.operation.linking.ramp;
    let helix = match ramp.kind {
        crate::model::RampKind::Plunge => 0.0,
        _ => {
            if ramp.helix_diameter > 0.0 {
                ramp.helix_diameter
            } else {
                r
            }
        }
    };
    ClearParams {
        radius: r,
        load,
        min_loop_radius: minimum_cutting_radius.max(0.0),
        max_loop_radius: 0.0,
        chord_tolerance: ctx.operation.tolerance,
        climb: direction != CutDirection::Conventional,
        helix_diameter: helix,
        max_connector: ctx.operation.linking.keep_down_distance,
        ..ClearParams::default()
    }
}

/// Certified passes of one level's result; the report gains the engagement
/// statistics and the solver's warnings.
pub fn level_passes(ctx: &OpContext<'_>, result: &LevelResult, z: f64, path_id: &mut u32, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<Vec<Pass>, CamError> {
    let mut out = Vec::with_capacity(result.passes.len());
    for lp in &result.passes {
        if lp.points.len() < 2 {
            continue;
        }
        let pts: Vec<Point3> = lp.points.iter().map(|q| Point3::new(q.x, q.y, z)).collect();
        let pass = Pass::new(pts, Some(*path_id));
        *path_id += 1;
        ctx.certify(&pass.points, report, runner)?;
        out.push(pass);
    }
    for w in &result.warnings {
        report.warn(w.clone());
    }
    report.counts.samples += result.samples;
    Ok(out)
}

/// The operation-wide engagement summary.
pub fn engagement_summary(report: &mut OperationReport, limit: f64, max: f64, exceeded: u64, loops: usize, left: f64) {
    report.warnings.push(
        Diagnostic::new("adaptive-engagement", format!("engagement limit {:.1} degrees, largest {:.1} degrees, {exceeded} samples over, {loops} trochoid loops, {left:.4} reachable area left", limit.to_degrees(), max.to_degrees()))
    );
}
