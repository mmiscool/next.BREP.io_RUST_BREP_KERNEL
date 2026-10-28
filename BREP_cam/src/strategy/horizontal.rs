//! Exact-height flat finishing. Raster spans are clipped to upward planar
//! triangles; whole-chord drop bounds exclude inaccessible spans without
//! changing their Z. Full protected geometry remains in every clearance solve.
use crate::{CamError,Cutter,Diagnostic,Feed,MeshSnapshot,Motion,Operation,Point3,Runner,Setup,Strategy,Tool,Toolpath,Tolerance};
use crate::drop::{DropCutter,path::{Sampling,Dilated}};

fn error(s:&str)->CamError {CamError::InvalidInput(format!("horizontal: {s}"))}

pub fn generate(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let path=generate_unlinked(operation,setup,tool,mesh,tol,runner)?;
    let cutter=Cutter::from_tool(tool)?;
    crate::strategy_linking::finish(&path,operation,&cutter,mesh,None,&setup.heights,&tool.feeds,tol,runner)
}
pub(crate) fn generate_unlinked(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    runner.token().check()?;operation.validate().map_err(CamError::InvalidInput)?;tol.validate()?;
    let params=match &operation.strategy {Strategy::Horizontal(p)=>p,_=>return Err(error("requires Horizontal parameters"))};
    if !operation.enabled || operation.setup!=setup.id || operation.tool!=tool.id {return Err(error("disabled operation or setup/tool identity mismatch"));}
    if tool.holder.is_some() {return Err(error("holder verification is not available"));}
    if operation.tool_axis.is_some_and(|p|p.normalized().is_none_or(|p|p.distance(Point3::Z)>tol.angle_eps)) {return Err(error("requires machine +Z tool axis"));}
    if !params.flat_angle_deg.is_finite() || !(0.0..90.0).contains(&params.flat_angle_deg) {return Err(error("flat angle must be within 0..90 degrees"));}
    let cutter=Cutter::from_tool(tool)?;let expanded=Dilated{base:&cutter,pad:operation.stock_to_leave.radial};
    let _physical=DropCutter::new(mesh,&cutter,None,tol)?;
    let drop=DropCutter::from_profile(mesh,&expanded,None,tol)?;
    let heights=operation.heights.as_ref().unwrap_or(&setup.heights);heights.validate().map_err(CamError::InvalidInput)?;
    let feeds=operation.feeds.as_ref().unwrap_or(&tool.feeds);feeds.validate().map_err(CamError::InvalidInput)?;
    if heights.clearance<=mesh.bounds.max.z+operation.stock_to_leave.axial+tol.distance_eps {return Err(error("clearance must exceed protected mesh and allowance"));}
    for solid in operation.targets.protected_bodies() {if !mesh.faces.iter().any(|f|f.solid==solid) {return Err(CamError::UnresolvedTarget{solid,face:None});}}
    for face in operation.targets.drive_faces.iter().chain(&operation.targets.avoid_faces) {if mesh.resolve_faces(face).is_empty() {return Err(CamError::UnresolvedTarget{solid:face.solid.clone(),face:face.face.clone()});}}
    let boundary_region=crate::drop::containment::boundary_region(operation,&cutter,mesh,tol,&params.containment)?;
    let mut flats=Vec::new();
    for triangle in &mesh.triangles {
        runner.work(1,"horizontal-detect",triangle.id as u64,mesh.triangle_count() as u64)?;
        let source=&mesh.faces[triangle.face as usize];
        let matches=|r:&crate::TargetRef|r.solid==source.solid && r.face.as_ref().is_none_or(|f|*f==source.face);
        if source.protected_only || !operation.targets.target_solids.contains(&source.solid) || operation.targets.avoid_faces.iter().any(matches) || (!operation.targets.drive_faces.is_empty() && !operation.targets.drive_faces.iter().any(matches)) {continue;}
        if triangle.normal.z<(params.flat_angle_deg.to_radians().cos()-tol.angle_eps) {continue;}
        if triangle.max_z()-triangle.min_z()>tol.distance_eps {return Err(error("near-horizontal sloped faces cannot be cut at one exact Z; reduce flat angle"));}
        let z=triangle.a.z;
        if params.containment.z_range.is_some_and(|[lo,hi]|z<lo || z>hi) || heights.bottom.is_some_and(|lo|z<lo) || heights.top.is_some_and(|hi|z>hi) {continue;}
        if params.containment.slope_deg.is_some_and(|[lo,hi]|0.0<lo || 0.0>hi) {continue;}
        flats.push((z,triangle));
    }
    flats.sort_by(|a,b|b.0.total_cmp(&a.0).then(a.1.id.cmp(&b.1.id)));
    let mut result=Toolpath::new(operation.id.clone(),tool.id.clone());result.report.counts.triangles=mesh.triangle_count() as u64;
    let mut offset=0;let mut omitted=0u64;let mut path_id=0u32;
    while offset<flats.len() {
        let z=flats[offset].0;let mut end=offset+1;
        while end<flats.len() && (flats[end].0-z).abs()<=tol.distance_eps {end+=1;}
        let group=&flats[offset..end];offset=end;
        let ymin=group.iter().map(|(_,t)|t.bounds.min.y).fold(f64::INFINITY,f64::min);
        let ymax=group.iter().map(|(_,t)|t.bounds.max.y).fold(f64::NEG_INFINITY,f64::max);
        let passes=((ymax-ymin)/params.stepover.distance).ceil().max(1.0);
        if passes>100000.0 {return Err(error("raster pass budget exceeded"));}
        let cut_z=z+operation.stock_to_leave.axial;
        let settings=Sampling{floor_z:mesh.bounds.min.z-1.0,sample_spacing:params.stepover.distance,min_sample_spacing:tol.sample_eps,chord_error:operation.tolerance,..Sampling::default()};
        for pass in 0..passes as usize {
            let y=ymin+(pass as f64+0.5)*(ymax-ymin)/passes;
            let mut intervals=Vec::<(f64,f64)>::new();
            for (_,triangle) in group {
                let p=triangle.vertices();let mut xs=Vec::new();
                for (a,b) in [(p[0],p[1]),(p[1],p[2]),(p[2],p[0])] {if (a.y<=y && b.y>y)||(b.y<=y && a.y>y) {xs.push(a.x+(y-a.y)*(b.x-a.x)/(b.y-a.y));}}
                xs.sort_by(f64::total_cmp);if xs.len()==2 && xs[1]>xs[0] {intervals.push((xs[0],xs[1]));}
            }
            intervals.sort_by(|a,b|a.0.total_cmp(&b.0));let mut union=Vec::<(f64,f64)>::new();
            for (a,b) in intervals {if let Some(last)=union.last_mut() {if a<=last.1+tol.distance_eps {last.1=last.1.max(b);continue;}}union.push((a,b));}
            if pass%2==1 {union.reverse();}
            for (lo,hi) in union {
                let (a,b)=if pass%2==0 {(lo,hi)}else{(hi,lo)};
                for (a,b) in boundary_region.clip_boundary(Point3::new(a,y,cut_z),Point3::new(b,y,cut_z),runner)? {
                    let mut stack=vec![(a,b,0u32)];let mut spans=Vec::new();
                    while let Some((a,b,depth))=stack.pop() {
                        let bound=drop.segment_bound(a,b,&settings,&mut result.report,runner)?+operation.stock_to_leave.axial;
                        if bound<=cut_z+tol.distance_eps {spans.push((a,b));}
                        else if depth<24 && a.distance_xy(b)>tol.sample_eps.max(operation.tolerance) {
                            let mid=a.lerp(b,0.5);stack.push((mid,b,depth+1));stack.push((a,mid,depth+1));
                        } else {omitted+=1;}
                    }
                    for (a,b) in spans {
                        if a.distance(b)<=tol.distance_eps {continue;}
                        result.motions.push(Motion::rapid(Point3::new(a.x,a.y,heights.clearance)).with_path(path_id));
                        result.motions.push(Motion::plunge(a,feeds.plunge_feed).with_path(path_id));
                        result.motions.push(Motion::cut(b,feeds.cutting_feed).with_path(path_id));
                        result.motions.push(Motion::retract(Point3::new(b.x,b.y,heights.clearance),Feed::RAPID).with_path(path_id));
                        path_id=path_id.checked_add(1).ok_or_else(||error("path budget exceeded"))?;
                    }
                }
            }
        }
    }
    if result.motions.is_empty() {return Err(error("no reachable exact-height flat region"));}
    if omitted>0 {result.report.warn(Diagnostic::new("flat-obstruction",format!("{omitted} obstructed flat spans omitted after bounded clearance refinement")));}
    result.report.counts.passes=path_id as u64;result.report.counts.motions=result.motions.len() as u64;result.report.finish();Ok(result)
}
