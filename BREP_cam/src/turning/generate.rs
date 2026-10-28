//! Shared turning stream preparation and continuous insert/section checks.
use super::{envelope,TurningGeometry,ToolEnvelope};
use super::profile::{TurningProfile,bar_section};
use crate::{CamError,Operation,Setup,Tool,StockCylinder,Runner,Toolpath,Motion,MotionKind,Feed,Point3,FeedsSpeeds};
pub(crate) fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}
pub(crate) fn prepare(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,profile:&TurningProfile)->Result<(Toolpath,ToolEnvelope,FeedsSpeeds),CamError> {
    op.validate().map_err(CamError::InvalidInput)?;
    if !op.enabled || op.setup!=setup.id || op.tool!=tool.id || profile.loops.is_empty() || profile.chord_bound>op.tolerance*0.25 {return Err(invalid("turning needs matching enabled operation and sufficiently accurate profile"));}
    if !profile.chord_bound.is_finite() || profile.chord_bound<0. || profile.loops.iter().any(|p|p.len()<3 || p.iter().any(|v|!v.is_finite() || v.y!=0. || v.x<0.)) {return Err(invalid("invalid protected turning section"));}
    if tool.holder.is_some() {return Err(invalid("turning needs an explicit XZ holder envelope; milling holder model unsupported"));}
    if (!matches!(op.strategy,crate::Strategy::TurnProfile(_)) && (op.stock_to_leave.radial!=0. || op.stock_to_leave.axial!=0.)) || op.tool_axis.map_or(false,|v|v.distance(Point3::Z)>1e-9) {return Err(invalid("turning stock allowances or tilted tool axis are unsupported by this strategy"));}
    let e=envelope(tool,g,op.tolerance*0.125)?;
    let feeds=op.feeds.as_ref().unwrap_or(&tool.feeds).clone();feeds.validate().map_err(CamError::InvalidInput)?;
    let mut path=Toolpath::new(op.id.clone(),tool.id.clone());
    path.motions.push(Motion::new(MotionKind::LatheMode{diameter_mode:true},Feed::RAPID));
    if feeds.css {
        let surface_speed=feeds.surface_speed.ok_or_else(||invalid("constant surface speed requires surface speed"))?;
        let max_rpm=feeds.surface_speed_limit_rpm.ok_or_else(||invalid("constant surface speed requires maximum rpm"))?;
        if !surface_speed.is_finite() || surface_speed<=0. || !max_rpm.is_finite() || max_rpm<=0. {return Err(invalid("invalid turning surface speed or rpm limit"));}
        path.motions.push(Motion::new(MotionKind::SpindleSurfaceSpeed{surface_speed,max_rpm,direction:feeds.spindle_direction},Feed::RAPID));
    } else {path.motions.push(Motion::new(MotionKind::SpindleOn{rpm:feeds.spindle_rpm,direction:feeds.spindle_direction},Feed::RAPID));}
    Ok((path,e,feeds))
}
pub(crate) fn finish(mut path:Toolpath,e:&ToolEnvelope,profile:&TurningProfile,stock:&StockCylinder,stickout:f64,contact_allowance:f64,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let bar=bar_section(stock,stickout)?;let mut previous=None;
    // Homothetic contact allowance, bounded in Euclidean distance at every
    // vertex. This is solely a collision tolerance, not a path offset engine.
    if !contact_allowance.is_finite() || contact_allowance<0. {return Err(invalid("invalid turning contact allowance"));}
    let mut check=e.clone();
    let centre=check.vertices.iter().fold(Point3::ZERO,|sum,p|sum.add(*p)).scale(1./check.vertices.len() as f64);
    let radius=check.vertices.iter().map(|p|p.distance(centre)).fold(0.,f64::max);
    if radius<=contact_allowance {return Err(invalid("turning tolerance exceeds envelope size"));}
    for p in &mut check.vertices {*p=centre.add(p.sub(centre).scale(1.-contact_allowance/radius));}
    for m in &path.motions {
        runner.work(1,"turning-sweep",0,0)?;
        if let Some(end)=m.end() {
            let start=previous.unwrap_or(end);
            let swept=e.swept_vertices(start,end)?;
            if swept.iter().any(|p|p.z<bar.chuck_z) {return Err(invalid("turning tool crosses the clamping plane"));}
            for material in &profile.loops {
                let mirrored:Vec<_>=material.iter().map(|p|Point3::new(-p.x,0.,p.z)).collect();
                if check.collides(start,end,material)? || check.collides(start,end,&mirrored)? {return Err(invalid("turning insert or holder intersects protected profile"));}
            }
            previous=Some(end);
        }
    }
    path.report.counts.motions=path.motions.len() as u64;path.report.finish();runner.token().check()?;Ok(path)
}
pub(crate) fn z_bounds(profile:&TurningProfile)->(f64,f64) {profile.loops.iter().flatten().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),p|(a.min(p.z),b.max(p.z)))}
pub(crate) fn nose(g:&TurningGeometry)->Result<f64,CamError> {match g {TurningGeometry::Insert(i)=>Ok(i.nose_radius),TurningGeometry::BoringBar(b)=>Ok(b.insert.nose_radius),_=>Err(invalid("facing needs a turning insert"))}}
pub(crate) fn point(x:f64,z:f64)->Point3 {Point3::new(x,0.,z)}
