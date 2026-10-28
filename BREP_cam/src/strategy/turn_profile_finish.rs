use crate::{CamError,Operation,Setup,Tool,StockCylinder,Runner,Toolpath,Strategy,Motion,MotionKind,Feed,Point2,Tolerance,LeadKind};
use crate::planar::{offset_chain,Side};
use crate::turning::{TurningGeometry,profile::TurningProfile,profile_path::*,generate::*};
/// Finishing clears rough allowances. C5 computes the nose-centre curve; the
/// full insert, including non-nose portions, must pass the continuous XZ sweep.
/// None/linear radial leads are supported. Unsupported arc/vertical leads refuse.
pub fn generate(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,stock:&StockCylinder,stickout:f64,profile:&TurningProfile,input:&ProfileInput,tol:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    match &op.strategy {Strategy::TurnProfile(p) if p.finishing && !p.roughing=>{},_=>return Err(invalid("finish profile entry requires finishing only; use generate_combined for both"))};
    if op.stock_to_leave.radial!=0. || op.stock_to_leave.axial!=0. {return Err(invalid("profile finish needs zero stock allowance; combined generator clears rough allowances"));}
    for lead in [&op.linking.lead_in,&op.linking.lead_out] {
        if !matches!(lead.kind,LeadKind::None|LeadKind::Linear) || lead.vertical_radius!=0. {return Err(invalid("turning profile supports none or linear XZ leads"));}
    }
    let c=context(op,setup,tool,g,stock,profile,input,tol,runner)?;
    runner.work((c.contour.segments.len()*c.contour.segments.len()) as u64,"turn-profile-compensate",0,0)?;
    let compensated=offset_chain(&c.contour,c.nose+op.tolerance*0.5,if c.outside{Side::Left}else{Side::Right},&tol)?;
    if compensated.len()!=1 {return Err(invalid("nose compensation splits or removes the selected turning contour"));}
    let chain=&compensated[0];
    if chain.length()/op.tolerance>100_000. {return Err(invalid("compensated profile sample budget exceeded"));}
    let mut points=chain.discretize(op.tolerance*0.125);
    for p in &mut points {
        if p.x < -tol.point_eps {return Err(invalid("compensated turning profile crosses spindle axis"));}
        if p.x<0. {p.x=0.;}
    }
    if points.len()<2 {return Err(invalid("empty compensated turning contour"));}
    let first=points[0];let last=*points.last().unwrap();
    let sign=if c.outside{1.}else{-1.};
    let lead_point=|p:Point2,lead:&crate::Lead|->Result<Point2,CamError>{
        let length=if lead.kind==LeadKind::None{0.}else{lead.radius+lead.linear_length};
        let x=p.x+sign*length;
        if !x.is_finite() || x<0. {return Err(invalid("turning lead crosses spindle axis"));}
        Ok(Point2::new(x,p.y))
    };
    let entry=lead_point(first,&op.linking.lead_in)?;let exit=lead_point(last,&op.linking.lead_out)?;
    let mut path=c.path;
    path.motions.push(Motion::rapid(point(c.clear_x,c.safe_z)));
    path.motions.push(Motion::rapid(point(c.clear_x,entry.y)));
    path.motions.push(Motion::new(MotionKind::LeadIn{to:p3(entry)},Feed::Rate(c.feeds.cutting_feed)));
    path.motions.push(Motion::new(MotionKind::LeadIn{to:p3(first)},Feed::Rate(c.feeds.cutting_feed)));
    for (i,p) in points.iter().enumerate().skip(1) {runner.work(1,"turn-profile-finish",i as u64,points.len() as u64)?;path.motions.push(Motion::cut(p3(*p),c.feeds.cutting_feed));}
    path.motions.push(Motion::new(MotionKind::LeadOut{to:p3(exit)},Feed::Rate(c.feeds.cutting_feed)));
    path.motions.push(Motion::new(MotionKind::LeadOut{to:point(c.clear_x,exit.y)},Feed::Rate(c.feeds.cutting_feed)));
    path.motions.push(Motion::rapid(point(c.clear_x,c.safe_z)));
    path.report.counts.passes=1;
    finish(path,&c.envelope,profile,stock,stickout,op.tolerance*0.25,runner)
}
/// One operation's roughing followed by finishing; all construction remains
/// local until both complete, so failure/cancellation publishes no partial path.
pub fn generate_combined(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,stock:&StockCylinder,stickout:f64,profile:&TurningProfile,input:&ProfileInput,direction:RoughingDirection,tol:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let params=match &op.strategy {Strategy::TurnProfile(p) if p.roughing && p.finishing=>p,_=>return Err(invalid("combined profile needs roughing and finishing enabled"))};
    let mut rough=op.clone();let mut rp=params.clone();rp.finishing=false;rough.strategy=Strategy::TurnProfile(rp);
    let mut path=crate::turning::profile_rough::generate(&rough,setup,tool,g,stock,stickout,profile,input,direction,tol,runner)?;
    let mut final_op=op.clone();let mut fp=params.clone();fp.roughing=false;final_op.strategy=Strategy::TurnProfile(fp);
    final_op.stock_to_leave.radial=0.;final_op.stock_to_leave.axial=0.;
    let finishing=generate(&final_op,setup,tool,g,stock,stickout,profile,input,tol,runner)?;
    path.motions.extend(finishing.motions.into_iter().skip(2));
    path.report.counts.passes+=finishing.report.counts.passes;
    path.report.counts.motions=path.motions.len() as u64;runner.token().check()?;Ok(path)
}
