use crate::turning::{TurningGeometry,profile::TurningProfile,schedule::face_levels,generate::*};
use crate::{CamError,Operation,Setup,Tool,StockCylinder,Runner,Toolpath,Strategy,Motion,Feed};
/// Face from bar front to the profile's front plane. `end_x` is the physical
/// bore radius, zero for facing to centre. Insert orientation must be accessible;
/// the complete swept envelope, including holder/bar, is checked before return.
pub fn generate(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,stock:&StockCylinder,stickout:f64,profile:&TurningProfile,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    runner.token().check()?;
    let p=match &op.strategy {Strategy::TurnFace(p)=>p,_=>return Err(invalid("turn facing requires TurnFace strategy"))};
    let (mut path,envelope,feeds)=prepare(op,setup,tool,g,profile)?;
    let target=z_bounds(profile).1;let front=stock.base.z+stock.height;
    if p.end_x<0. || p.end_x>=stock.radius {return Err(invalid("facing bore radius is outside stock"));}
    let nose=nose(g)?;let pad=op.tolerance*0.25;let clearance=op.linking.safe_distance.max(op.tolerance);
    let levels=face_levels(front,target,p.step_over,p.finish_allowance,p.finish_pass,100_000)?;
    let outer=stock.radius+nose+clearance;let safe_z=front+nose+clearance;
    path.motions.push(Motion::rapid(point(outer,safe_z)));
    for (i,z) in levels.iter().enumerate() {
        runner.work(1,"turn-face",i as u64,levels.len() as u64)?;
        let z=z+nose+pad;
        path.motions.push(Motion::rapid(point(outer,z)));
        path.motions.push(Motion::cut(point(p.end_x,z),feeds.cutting_feed).with_path(i as u32));
        path.motions.push(Motion::retract(point(p.end_x,safe_z),Feed::RAPID));
        path.motions.push(Motion::rapid(point(outer,safe_z)));
    }
    path.report.counts.passes=levels.len() as u64;
    finish(path,&envelope,profile,stock,stickout,op.tolerance*0.25,runner)
}
