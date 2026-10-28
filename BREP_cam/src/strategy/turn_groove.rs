use crate::turning::{TurningGeometry,profile::TurningProfile,schedule::groove_centres,generate::*};
use crate::{CamError,Operation,Setup,Tool,StockCylinder,Runner,Toolpath,Strategy,Motion,Feed};
/// Rectangular groove with blade-matched corner radii; `centre_z` is selected
/// feature geometry in machine coordinates. Final profile must contain the
/// groove: collision checks refuse attempting to gouge a solid cylinder.
pub fn generate(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,stock:&StockCylinder,stickout:f64,profile:&TurningProfile,centre_z:f64,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    runner.token().check()?;
    let p=match &op.strategy {Strategy::TurnGroove(p)=>p,_=>return Err(invalid("turn grooving requires TurnGroove strategy"))};
    let blade=match g {TurningGeometry::Blade(b)=>b,_=>return Err(invalid("grooving requires blade geometry"))};
    if blade.holder_angle_deg!=0. || !centre_z.is_finite() || !p.depth.is_finite() || p.depth<=0. || p.depth>=stock.radius || p.finish_allowance<0. || p.finish_allowance>=p.depth {return Err(invalid("invalid axial blade groove geometry"));}
    let (mut path,envelope,feeds)=prepare(op,setup,tool,g,profile)?;
    let pad=op.tolerance*0.25;let width=p.width;
    let centres=groove_centres(width,blade.width,p.step_over,100_000)?;
    let outer=stock.radius+op.linking.safe_distance.max(op.tolerance);
    let floor=stock.radius-p.depth+pad;let rough=floor+p.finish_allowance;
    path.motions.push(Motion::rapid(point(outer,centre_z+centres[0])));
    for (i,z) in centres.iter().enumerate() {
        runner.work(1,"turn-groove",i as u64,centres.len() as u64)?;
        let z=centre_z+z;
        path.motions.push(Motion::rapid(point(outer,z)));
        path.motions.push(Motion::cut(point(rough,z),feeds.plunge_feed).with_path(i as u32));
        path.motions.push(Motion::retract(point(outer,z),Feed::RAPID));
    }
    if p.finish_pass {
        let first=centre_z+centres[0];let last=centre_z+centres[centres.len()-1];
        path.motions.push(Motion::rapid(point(outer,first)));
        path.motions.push(Motion::cut(point(floor,first),feeds.plunge_feed));
        path.motions.push(Motion::cut(point(floor,last),feeds.cutting_feed));
        path.motions.push(Motion::retract(point(outer,last),Feed::RAPID));
    }
    path.report.counts.passes=centres.len() as u64+p.finish_pass as u64;
    finish(path,&envelope,profile,stock,stickout,op.tolerance*0.25,runner)
}
