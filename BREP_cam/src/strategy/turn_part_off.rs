use crate::turning::{TurningGeometry,profile::TurningProfile,generate::*};
use crate::{CamError,Operation,Setup,Tool,StockCylinder,Runner,Toolpath,Strategy,Motion,Feed};
/// Part behind the final profile's back face. A requested chamfer is checked
/// against the final profile and refuses if that profile does not contain it.
pub fn generate(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,stock:&StockCylinder,stickout:f64,profile:&TurningProfile,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    runner.token().check()?;
    let p=match &op.strategy {Strategy::TurnPartOff(p)=>p,_=>return Err(invalid("part-off requires TurnPartOff strategy"))};
    let blade=match g {TurningGeometry::Blade(b)=>b,_=>return Err(invalid("part-off requires blade geometry"))};
    if blade.holder_angle_deg!=0. || ![p.end_x,p.chamfer,p.peck_depth,p.peck_retract,p.reduced_feed_radius,p.reduced_feed_factor].iter().all(|v|v.is_finite()) || p.end_x<0. || p.end_x>=stock.radius || p.chamfer<0. || p.chamfer>stock.radius-p.end_x || p.peck_depth<0. || p.peck_retract<0. || (p.peck_depth>0. && p.peck_retract<=0.) || p.reduced_feed_radius<0. || p.reduced_feed_factor<=0. || p.reduced_feed_factor>1. {return Err(invalid("invalid part-off geometry or feed reduction"));}
    let (mut path,envelope,feeds)=prepare(op,setup,tool,g,profile)?;
    let pad=op.tolerance*0.25;let back=z_bounds(profile).0;
    let z=back-blade.width*0.5-pad;
    let outer=stock.radius+op.linking.safe_distance.max(op.tolerance);
    path.motions.push(Motion::rapid(point(outer,z+p.chamfer)));
    if p.chamfer>0. {
        path.motions.push(Motion::cut(point(stock.radius,z+p.chamfer),feeds.plunge_feed));
        path.motions.push(Motion::cut(point(stock.radius-p.chamfer,z),feeds.cutting_feed));
        path.motions.push(Motion::retract(point(outer,z),Feed::RAPID));
    }
    let count=if p.peck_depth>0. {((stock.radius-p.end_x)/p.peck_depth).ceil()} else {1.};
    if !count.is_finite() || count>100_000. {return Err(invalid("part-off peck budget exceeded"));}
    let mut at=outer;
    for i in 1..=count as usize {
        runner.work(1,"turn-part-off",i as u64,count as u64)?;
        let x=if i==count as usize {p.end_x} else {(stock.radius-i as f64*p.peck_depth).max(p.end_x)};
        if at>p.reduced_feed_radius && x<p.reduced_feed_radius {path.motions.push(Motion::cut(point(p.reduced_feed_radius,z),feeds.plunge_feed));}
        let feed=feeds.plunge_feed*if x<p.reduced_feed_radius{p.reduced_feed_factor}else{1.};
        path.motions.push(Motion::cut(point(x,z),feed).with_path(i as u32));
        if i<count as usize {at=(x+p.peck_retract).min(outer);path.motions.push(Motion::retract(point(at,z),Feed::RAPID));}
    }
    path.motions.push(Motion::retract(point(outer,z),Feed::RAPID));
    path.report.counts.passes=count as u64;
    finish(path,&envelope,profile,stock,stickout,op.tolerance*0.25,runner)
}
