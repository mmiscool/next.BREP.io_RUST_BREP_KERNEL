//! Continuous constant-pitch contour ramps on invariant closed cross-sections.
//! Changing topology/shape is an explicit refusal pending contour morphing.
use crate::{CamError,Cutter,CutterProfile,Diagnostic,Feed,MeshSnapshot,Motion,MotionKind,Operation,Point3,Runner,Setup,Strategy,Tool,Toolpath,Tolerance,Contour3dParams};
use crate::waterline::contour::{self,Limits};
use crate::push::PushContext;
use std::collections::BTreeMap;
fn error(s:&str)->CamError {CamError::InvalidInput(format!("ramp: {s}"))}

pub fn generate(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,limits:Limits,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let path=generate_unlinked(operation,setup,tool,mesh,tol,limits,runner)?;
    let cutter=Cutter::from_tool(tool)?;
    crate::strategy_linking::finish(&path,operation,&cutter,mesh,None,&setup.heights,&tool.feeds,tol,runner)
}
pub(crate) fn generate_unlinked(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,limits:Limits,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    runner.token().check()?;operation.validate().map_err(CamError::InvalidInput)?;
    let params=match &operation.strategy {Strategy::Ramp(p)=>p,_=>return Err(error("requires Ramp parameters"))};
    let angle=operation.linking.ramp.angle_deg;
    if !angle.is_finite() || angle<=0.0 || angle>=90.0 {return Err(error("maximum ramp angle must be strictly between 0 and 90 degrees"));}
    let mut source=operation.clone();source.strategy=Strategy::Contour3D(Contour3dParams{step_down:params.step_down,adaptive_step_down:false,fiber_step:0.5,direction:params.direction,containment:params.containment.clone()});
    let template=contour::generate_unlinked(&source,setup,tool,mesh,tol,limits,runner)?;
    let mut paths=BTreeMap::<u32,Vec<Point3>>::new();let mut previous=None;
    for motion in &template.motions {
        if let MotionKind::Cut{to}=motion.kind {
            let from=previous.ok_or_else(||error("contour has no cutting start"))?;
            let path=paths.entry(motion.path_id.ok_or_else(||error("contour has no path identity"))?).or_default();
            if path.is_empty() {path.push(from);} else if path.last().unwrap().distance(from)>tol.point_eps {return Err(error("clipped or open contours cannot ramp"));}
            path.push(to);
        }
        previous=motion.end().or(previous);
    }
    let mut levels=Vec::<(f64,Vec<Vec<Point3>>)>::new();
    for (_,path) in paths {
        if path.len()<4 || path[0].distance(*path.last().unwrap())>tol.point_eps {return Err(error("requires complete closed contours"));}
        if levels.last().is_none_or(|(z,_)|*z!=path[0].z) {levels.push((path[0].z,Vec::new()));}
        levels.last_mut().unwrap().1.push(path);
    }
    let first=levels.first().ok_or_else(||error("no contour loops"))?;let last=levels.last().unwrap();
    let (top,bottom)=(first.0,last.0);
    if top<=bottom {return Err(error("ramp needs a nonzero depth range"));}
    for (_,loops) in &levels {
        if loops.len()!=last.1.len() {return Err(error("changing loop topology cannot ramp"));}
        for (a,b) in loops.iter().zip(&last.1) {
            if a.len()!=b.len() || a.iter().zip(b).any(|(a,b)|a.distance_xy(*b)>tol.point_eps) {return Err(error("changing contour shape requires a morphing ramp"));}
        }
    }
    let cutter=Cutter::from_tool(tool)?;let heights=operation.heights.as_ref().unwrap_or(&setup.heights);let feeds=operation.feeds.as_ref().unwrap_or(&tool.feeds);
    let mut result=Toolpath::new(operation.id.clone(),tool.id.clone());result.report=template.report;
    result.report.counts.passes=0;result.report.counts.motions=0;
    for (id,base) in last.1.iter().enumerate() {
        let perimeter:f64=base.windows(2).map(|p|p[0].distance_xy(p[1])).sum();
        let maximum_drop=params.step_down.min(perimeter*angle.to_radians().tan());
        let turns=((top-bottom)/maximum_drop).ceil();
        if !turns.is_finite() || turns<1.0 || turns>limits.max_levels as f64 {return Err(error("ramp revolution budget exceeded"));}
        // Equal pitch on all turns, including the final one, reaches bottom exactly.
        let pitch=(top-bottom)/turns;let start=Point3::new(base[0].x,base[0].y,top);
        let safety=PushContext{mesh,cutter:&cutter,index:None,tolerance:tol,allowance:operation.stock_to_leave.radial};
        contour::verify_vertical(&safety,start,heights.clearance,runner)?;
        result.motions.push(Motion::rapid(Point3::new(start.x,start.y,heights.clearance)).with_path(id as u32));
        result.motions.push(Motion::plunge(start,feeds.plunge_feed).with_path(id as u32));
        let mut from=start;
        for turn in 0..=turns as usize {
            let mut distance=0.0;
            for pair in base.windows(2) {
                distance+=pair[0].distance_xy(pair[1]);
                let z=if turn==turns as usize {bottom}else{(top-pitch*(turn as f64+distance/perimeter)).max(bottom)};
                let to=Point3::new(pair[1].x,pair[1].y,z);
                verify(&safety,from,to,runner)?;
                result.motions.push(Motion::cut(to,feeds.cutting_feed).with_path(id as u32));from=to;
                result.report.counts.samples+=1;
            }
            result.report.counts.passes+=1;
        }
        contour::verify_vertical(&safety,from,heights.clearance,runner)?;
        result.motions.push(Motion::retract(Point3::new(from.x,from.y,heights.clearance),Feed::RAPID).with_path(id as u32));
    }
    result.report.warn(Diagnostic::new("ramp-cross-section","Constant cross-section ramp; one final level lap completes the bottom"));
    result.report.counts.motions=result.motions.len() as u64;result.report.finish();Ok(result)
}

fn verify(context:&PushContext<'_>,a:Point3,b:Point3,runner:&mut Runner<'_>)->Result<(),CamError> {
    let mut extended=context.cutter.clone();let extra=(a.z-b.z).abs();let radius=extended.radius().max(extended.shaft_radius());
    match &mut extended {
        Cutter::Flat(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(radius);},
        Cutter::Ball(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(radius);},
        Cutter::Bull(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(radius);},
        Cutter::Cone(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(radius);},
        Cutter::BallCone(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(radius);},
        Cutter::Frustum(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(radius);},
        Cutter::Lollipop(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(radius);},
        Cutter::Dovetail(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(radius);},
    }
    let sweep=PushContext{cutter:&extended,..*context};let z=a.z.min(b.z);
    if !sweep.segment(Point3::new(a.x,a.y,z),Point3::new(b.x,b.y,z),runner)?.is_empty() {return Err(error("descending segment intersects protected material"));}
    Ok(())
}
