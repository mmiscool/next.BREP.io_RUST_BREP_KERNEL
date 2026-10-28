use crate::{CamError,Operation,Setup,Tool,MeshSnapshot,Tolerance,Runner,Motion,MotionKind,Feed};
use crate::holes::Hole;
/// Lathe drill/centre drill reuses the complete C8 strategy and safety checks.
/// Every selected hole must lie on spindle X=Y=0; radial drilling is a different
/// machine capability. Both alternative streams enter typed G18 lathe mode.
pub fn generate(op:&Operation,setup:&Setup,tool:&Tool,holes:&[Hole],mesh:&MeshSnapshot,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<crate::drill::strategy::DrillOutput,CamError> {
    let feeds=op.feeds.as_ref().unwrap_or(&tool.feeds);
    feeds.validate().map_err(CamError::InvalidInput)?;
    if feeds.css {return Err(CamError::InvalidInput("spindle-axis drilling requires fixed rpm".into()));}
    if holes.iter().any(|h|h.top.x.abs()>tolerance.distance_eps || h.top.y.abs()>tolerance.distance_eps) {return Err(CamError::InvalidInput("lathe drilling must be on the spindle axis".into()));}
    let mut output=crate::drill::strategy::generate(op,setup,tool,holes,mesh,tolerance,runner)?;
    for path in [&mut output.canned,&mut output.expanded] {path.motions.insert(0,Motion::new(MotionKind::LatheMode{diameter_mode:true},Feed::RAPID));path.motions.insert(1,Motion::new(MotionKind::SpindleOn{rpm:feeds.spindle_rpm,direction:feeds.spindle_direction},Feed::RAPID));path.report.counts.motions=path.motions.len() as u64;}
    Ok(output)
}
