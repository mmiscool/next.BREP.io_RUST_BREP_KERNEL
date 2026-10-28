use super::{TurningGeometry,profile::TurningProfile,generate::*,thread,thread_cycle};
use crate::{CamError,Operation,Setup,Tool,StockCylinder,Runner,Toolpath,Strategy,TurnThreadCycle,Motion,MotionKind,Feed};
#[derive(Clone,Debug,PartialEq)]
pub struct ThreadOutput {pub canned:Toolpath,pub expanded:Toolpath}
/// `nominal_radius` comes from the selected cylindrical thread feature. The
/// supplied protected profile is its axisymmetric root envelope, including all
/// shoulders and workholding; a helical flank solid is not an axial profile.
pub fn generate(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,stock:&StockCylinder,stickout:f64,profile:&TurningProfile,nominal_radius:f64,runner:&mut Runner<'_>)->Result<ThreadOutput,CamError> {
    runner.token().check()?;
    let p=match &op.strategy {Strategy::TurnThread(p)=>p,_=>return Err(invalid("threading requires TurnThread strategy"))};
    let insert=match g {TurningGeometry::Thread(t)=>t,_=>return Err(invalid("threading requires threading insert geometry"))};
    if !nominal_radius.is_finite() || nominal_radius<=0. || p.pitch<insert.pitch_min || p.pitch>insert.pitch_max || p.infeed_angle_deg>=insert.angle_deg/2. || (!p.internal && p.depth>=nominal_radius) {return Err(invalid("thread feature or infeed exceeds insert capability"));}
    if op.feeds.as_ref().unwrap_or(&tool.feeds).css {return Err(invalid("threading requires fixed spindle rpm"));}
    let (mut canned,envelope,feeds)=prepare(op,setup,tool,g,profile)?;
    let (low,high)=z_bounds(profile);let clearance=op.linking.safe_distance.max(op.tolerance);
    let extent=envelope.vertices.iter().map(|v|(v.z-envelope.reference_to_tip.z).abs()).fold(0.,f64::max);
    let heights=op.heights.as_ref().unwrap_or(&setup.heights);
    let start_z=heights.top.unwrap_or(high+extent+clearance);
    let end_z=heights.bottom.unwrap_or(low-extent-clearance);
    let depths=thread::pass_depths(p.depth,p.passes,p.constant_area)?;
    let offsets=thread::infeed_offsets(&depths,p.infeed_angle_deg,p.alternate_flanks)?;
    let allowance=op.tolerance*0.25;
    let radius=nominal_radius+if p.internal{-insert.nose_radius-allowance}else{insert.nose_radius+allowance};
    let clear=nominal_radius+if p.internal{-clearance-insert.nose_radius}else{clearance+insert.nose_radius};
    let cycle=TurnThreadCycle{start:point(radius,start_z),end:point(radius,end_z),pitch:p.pitch,depths,infeed_offsets:offsets,clearance_x:clear,internal:p.internal,spring_passes:p.spring_passes};
    let moves=thread_cycle::expand(&cycle,&feeds,runner)?;
    let mut expanded=canned.clone();expanded.motions.extend(moves);
    expanded.report.counts.passes=p.passes as u64+p.spring_passes as u64;
    let expanded=finish(expanded,&envelope,profile,stock,stickout,allowance,runner)?;
    canned.motions.push(Motion::rapid(point(clear,start_z)));
    canned.motions.push(Motion::new(MotionKind::TurnThreadCycle{cycle},Feed::Rate(p.pitch*feeds.spindle_rpm)));
    canned.report=expanded.report.clone();canned.report.counts.motions=canned.motions.len() as u64;
    Ok(ThreadOutput{canned,expanded})
}
