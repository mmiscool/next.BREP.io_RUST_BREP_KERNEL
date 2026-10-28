//! C-axis contour interpolation. Inputs are already compensated C5 CL chains.
//! Returned paths are single feed sections; the assembler owns safe entry/exit.
use super::{invalid,chain,validate_position};
use crate::{CamError,Point2,Point3,Tolerance,Runner,MeshSnapshot,Cutter,CutterProfile};
use crate::model::*;
use crate::planar::{Chain,Segment};
use crate::multiaxis::contact::drop_along;

// Machine tip, unwrapped C degrees, corresponding part-space tip.
type Sample=(Point3,f64,Point3);
const MAX_SAMPLES:usize=200_000;
fn points(chain:&Chain,t:Tolerance,chord:f64)->Result<Vec<Point2>,CamError> {
    t.validate()?;
    let mut c=chain.clone();c.validate(&t)?;
    if c.segments.len()>512 || !chord.is_finite() || chord<=t.distance_eps {return Err(invalid("contour budget or chord tolerance invalid"));}
    // C5 caps arc subdivision; verify that cap can still meet our bound.
    let mut count=0usize;
    for segment in &c.segments {
        let samples=segment.discretize(chord);count=count.saturating_add(samples.len());
        if count>MAX_SAMPLES {return Err(invalid("contour discretization budget exceeded"));}
        if segment.is_arc() {
            let (_,sweep)=segment.angles();let step=sweep.abs()/(samples.len()-1) as f64;
            if segment.radius()*(1.-(step*0.5).cos())>chord*(1.+1e-9) {return Err(invalid("arc subdivision cannot meet chord bound"));}
        }
    }
    let p=c.discretize(chord);
    if p.len()<2 || p.len()>MAX_SAMPLES {return Err(invalid("empty or excessive contour"));}Ok(p)
}
fn unwrap(mut angle:f64,previous:f64)->f64 {while angle-previous>180. {angle-=360.;}while angle-previous< -180. {angle+=360.;}angle}
fn polar_samples(c:&Chain,z:f64,error:f64,length:f64,t:Tolerance)->Result<Vec<Sample>,CamError> {
    let p=points(c,t,error*0.25)?;let mut out=Vec::new();let mut previous=0.;
    for w in p.windows(2) {
        if Segment::line(w[0],w[1]).distance(Point2::new(0.,0.))<=error {return Err(invalid("polar contour crosses spindle singularity"));}
        let distance=(w[1].x-w[0].x).hypot(w[1].y-w[0].y);
        let rmin=Segment::line(w[0],w[1]).distance(Point2::new(0.,0.));
        // Bounds tip displacement plus rotating tool envelope displacement.
        let n=(distance*(1.+length/rmin)/(error*0.25)).ceil().max(1.) as usize;
        if n.saturating_add(out.len()).saturating_add(1)>MAX_SAMPLES {return Err(invalid("polar sampling budget exceeded"));}
        for i in 0..=n {
            if i==0 && !out.is_empty() {continue;}
            let f=i as f64/n as f64;let x=w[0].x+(w[1].x-w[0].x)*f;let y=w[0].y+(w[1].y-w[0].y)*f;
            let a=unwrap(-y.atan2(x).to_degrees(),previous);previous=a;
            out.push((Point3::new(x.hypot(y),0.,z),a,Point3::new(x,y,z)));
        }
    }Ok(out)
}
/// Reorient C10's A-axis wrap into spindle Z (proper rotation, not reflection).
fn wrapped(z:f64,theta:f64,r:f64)->(Point3,Point3) {
    let (p,a)=crate::rotary::place(Point3::X,z,theta,r);
    (Point3::new(p.z,-p.y,p.x),Point3::new(a.z,-a.y,a.x))
}
fn cylindrical_samples(c:&Chain,radius:f64,error:f64,length:f64,t:Tolerance)->Result<Vec<Sample>,CamError> {
    let p=points(c,t,error*0.25)?;let mut out=Vec::new();
    for w in p.windows(2) {
        let dz=w[1].x-w[0].x;let dv=w[1].y-w[0].y;
        let n=((dz.abs()+dv.abs()*(1.+length/radius))/(error*0.25)).ceil().max(1.) as usize;
        if n.saturating_add(out.len()).saturating_add(1)>MAX_SAMPLES {return Err(invalid("cylindrical sampling budget exceeded"));}
        for i in 0..=n {
            if i==0 && !out.is_empty() {continue;}
            let f=i as f64/n as f64;let z=w[0].x+dz*f;let theta=(w[0].y+dv*f)/radius;
            out.push((Point3::new(radius,0.,z),theta.to_degrees(),wrapped(z,theta,radius).0));
        }
    }Ok(out)
}

/// Face XY contour at fixed Z. Both streams use radius X, never diameter X.
pub fn polar(op:&Operation,machine:&Machine,tool:&Tool,contour:&Chain,z:f64,part:&MeshSnapshot,fixed:&MeshSnapshot,t:Tolerance,runner:&mut Runner<'_>)->Result<(Toolpath,Toolpath),CamError> {
    runner.token().check()?;
    if !z.is_finite() {return Err(invalid("nonfinite face Z"));}
    let samples=polar_samples(contour,z,op.tolerance,tool.geometry.cutting_length+tool.geometry.shaft_length,t)?;
    generate(op,machine,tool,samples,None,part,fixed,t,runner)
}
/// Wrapped contour (C5 x=axial Z, y=arc length), at a fixed tip radius.
/// `part` rotates with C; `fixed` contains stationary fixtures in machine space.
pub fn cylindrical(op:&Operation,machine:&Machine,tool:&Tool,contour:&Chain,radius:f64,part:&MeshSnapshot,fixed:&MeshSnapshot,t:Tolerance,runner:&mut Runner<'_>)->Result<(Toolpath,Toolpath),CamError> {
    runner.token().check()?;
    if !radius.is_finite() || radius<=t.distance_eps {return Err(invalid("wrap radius must be positive"));}
    let samples=cylindrical_samples(contour,radius,op.tolerance,tool.geometry.cutting_length+tool.geometry.shaft_length,t)?;
    generate(op,machine,tool,samples,Some(radius),part,fixed,t,runner)
}
fn generate(op:&Operation,machine:&Machine,tool:&Tool,samples:Vec<Sample>,radius:Option<f64>,part:&MeshSnapshot,fixed:&MeshSnapshot,t:Tolerance,runner:&mut Runner<'_>)->Result<(Toolpath,Toolpath),CamError> {
    t.validate()?;op.validate().map_err(CamError::InvalidInput)?;chain(machine,Point3::ZERO)?;
    if !op.enabled || op.tool!=tool.id || tool.kind!=ToolKind::Flat || tool.holder.is_some() || part.is_empty() {return Err(invalid("requires enabled matching flat tool, explicit holder-free envelope and protected part mesh"));}
    if op.stock_to_leave.radial!=0. || op.stock_to_leave.axial!=0. {return Err(invalid("input is an already-compensated CL contour; operation allowances must be zero"));}
    let expected_axis=if radius.is_some(){Point3::X}else{Point3::Z};
    if op.tool_axis.is_some_and(|a|a.distance(expected_axis)>t.angle_eps) {return Err(invalid("operation tool axis disagrees with interpolation plane"));}
    let feeds=op.feeds.as_ref().unwrap_or(&tool.feeds);feeds.validate().map_err(CamError::InvalidInput)?;
    if feeds.css {return Err(invalid("live milling requires fixed tool spindle rpm"));}
    for solid in op.targets.protected_bodies() {
        if !part.faces.iter().chain(&fixed.faces).any(|f|f.solid==solid) {return Err(CamError::UnresolvedTarget{solid,face:None});}
    }
    let g=&tool.geometry;let length=g.cutting_length+g.shaft_length;
    Cutter::from_tool(tool)?;
    let machine_axis=if radius.is_some(){Point3::X}else{Point3::Z};
    let mut interpolated=Toolpath::new(op.id.clone(),tool.id.clone());let mut expanded=interpolated.clone();
    interpolated.motions.push(Motion::new(match radius {Some(r)=>MotionKind::CylindricalInterpolation{radius:Some(r)},None=>MotionKind::PolarInterpolation{enabled:true}},Feed::RAPID));
    runner.stage("millturn-contour","certify continuous tool envelope",0,samples.len() as u64)?;
    for (i,s) in samples.iter().enumerate() {
        runner.work(1,"millturn-contour",i as u64,samples.len() as u64)?;
        validate_position(machine,s.0,t)?;
        let MachineKind::MillTurn{c_axis,..}=&machine.kind else {unreachable!()};
        if c_axis.limits_deg.is_some_and(|[lo,hi]|s.1<lo || s.1>hi) {return Err(invalid("contour exceeds C travel"));}
        let axis=if radius.is_some(){wrapped(s.0.z,s.1.to_radians(),1.).1}else{Point3::Z};
        let next=samples.get(i+1).unwrap_or(s);
        // A conservative cylinder at the start encloses every translated and
        // rotated tool position over this synchronized feed interval.
        let angle=(next.1-s.1).abs().to_radians();
        let travel=(next.0.x-s.0.x).abs()+(next.0.z-s.0.z).abs()+s.0.x.max(next.0.x)*angle;
        let pad=travel+(length+g.diameter.max(g.shaft_diameter())*0.5)*angle+t.distance_eps;
        let bound=Cutter::flat(g.diameter.max(g.shaft_diameter())+2.*pad,length+2.*pad,0.);
        bound.validate()?;
        for (mesh,tip,ax) in [(part,s.2,axis),(fixed,s.0,machine_axis)] {
            if mesh.is_empty(){continue;}
            let result=drop_along(mesh,None,&bound,tip.sub(ax.scale(pad)),ax,op.tolerance+pad,t,runner)?;
            if result.offset>op.tolerance {return Err(invalid("continuous tool envelope gouges protected geometry"));}
        }
        let virtual_tip=match radius {None=>s.2,Some(r)=>Point3::new(r,s.1.to_radians()*r,s.0.z)};
        interpolated.motions.push(Motion::cut(virtual_tip,feeds.cutting_feed));
        let mut m=Motion::new(MotionKind::LinearRotary{to:s.0,axes:vec![RotaryWord{name:"C".into(),angle_deg:s.1}]},Feed::Rate(feeds.cutting_feed));m.tool_axis=Some(machine_axis);expanded.motions.push(m);
    }
    interpolated.motions.push(Motion::new(match radius {Some(_)=>MotionKind::CylindricalInterpolation{radius:None},None=>MotionKind::PolarInterpolation{enabled:false}},Feed::RAPID));
    for path in [&mut interpolated,&mut expanded] {path.report.counts.motions=path.motions.len() as u64;path.report.counts.samples=samples.len() as u64;path.report.counts.passes=1;path.report.finish();}
    runner.token().check()?;Ok((interpolated,expanded))
}
