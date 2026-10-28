//! Archimedean spiral with radial pitch in machine XY.
use crate::*;
use crate::drop::finish::{FinishContext,invalid,polyline,MAX_SOURCE_SEGMENTS};

pub fn source_path(p:&SpiralParams,outer_radius:f64,chord_error:f64)->Result<Vec<Point2>,CamError> {
    p.stepover.validate().map_err(CamError::InvalidInput)?;
    if !p.center.is_finite() || !outer_radius.is_finite() || outer_radius<=0. || !chord_error.is_finite() || chord_error<=0. {return Err(invalid("invalid spiral center, radius or chord tolerance"));}
    let pitch=p.stepover.distance/std::f64::consts::TAU;let sweep=outer_radius/pitch;
    // ||d²p/dtheta²|| <= R+2*pitch. Linear interpolation error <= M*dt²/8.
    let dt=(8.*chord_error/(outer_radius+2.*pitch)).sqrt().min(std::f64::consts::FRAC_PI_8);
    let n=(sweep/dt).ceil().max(1.);
    if !n.is_finite() || n>MAX_SOURCE_SEGMENTS as f64 {return Err(invalid("spiral source work budget exceeded"));}
    let mut points:Vec<_>=(0..=n as usize).map(|i|{let t=sweep*i as f64/n;let r=pitch*t;Point2::new(p.center.x+r*t.cos(),p.center.y+r*t.sin())}).collect();
    if !p.inside_out {points.reverse();}Ok(points)
}
pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Spiral(p)=>p,_=>return Err(invalid("spiral generator requires Spiral parameters"))};
    let ctx=FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance};let b=ctx.bounds(&p.containment)?;
    let outer=[b.min.x,b.max.x].iter().flat_map(|x|[b.min.y,b.max.y].map(|y|(x-p.center.x).hypot(y-p.center.y))).fold(0.,f64::max);
    let points=source_path(p,outer,operation.tolerance*0.25)?;
    ctx.generate(&p.containment,&[polyline(&points)?],runner)
}
