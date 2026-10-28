//! Optional feed optimisation and certified smoothing of linked toolpaths.

use crate::{CamError,model::Toolpath};
pub mod feed;
const MAX_MOTIONS:usize=200_000;
fn invalid(message:&str)->CamError {CamError::InvalidInput(format!("polish: {message}"))}
fn validate_path(path:&Toolpath)->Result<(),CamError> {
 if !path.report.is_usable() || path.first_non_finite().is_some() {return Err(invalid("requires a usable finite linked toolpath"));}
 if path.motions.len()>MAX_MOTIONS {return Err(invalid("input motion work budget exceeded"));}Ok(())
}
pub mod smoothing;

/// C4 per-operation hook, after linking and before publishing program motions.
/// `mesh` is the complete protected snapshot in linked-path coordinates.
/// Call once: repeated feed optimisation intentionally compounds reductions.
/// When enabled, this is the LAST geometry-changing step; any later filter or
/// fitter must independently re-certify its output. No operation is mutated.
pub fn apply(path:&Toolpath,operation:&crate::model::Operation,tool:&crate::model::Tool,mesh:&crate::MeshSnapshot,tolerance:crate::Tolerance,runner:&mut crate::Runner<'_>)->Result<Toolpath,CamError> {
 runner.token().check()?;
 let p=&operation.polish;
 if !p.feed_optimization.enabled && !p.smoothing.enabled {return Ok(path.clone());}
 if path.operation!=operation.id || path.tool!=operation.tool || path.tool!=tool.id {return Err(invalid("operation/tool/path identity mismatch"));}
 if p.smoothing.enabled && (!operation.tolerance.is_finite() || p.smoothing.tolerance>operation.tolerance) {return Err(invalid("smoothing tolerance exceeds operation tolerance"));}
 let reduced=feed::optimize(path,&p.feed_optimization,runner)?;
 smoothing::smooth(&reduced,&p.smoothing,tool,mesh,tolerance,runner)
}
