//! Three-axis contour finishing. Mesh input is already in machine coordinates.
//! Results are committed only on success; errors/cancellation return no partial
//! motion stream. Every cut chord and vertical approach uses the protected mesh.
use crate::{CamError,Cutter,CutterProfile,Diagnostic,Feed,MeshBuilder,MeshSnapshot,Motion,
    Operation,Point3,Runner,Setup,Strategy,Tool,Toolpath,Tolerance};
use crate::model::{Containment,CutDirection};
use crate::push::PushContext;
use crate::waterline::{self,Sampling};

#[derive(Clone,Copy,Debug)]
pub struct Limits {
    pub max_levels:usize,
    pub max_refinements:u32,
    pub max_fibers:usize,
}
impl Default for Limits {
    fn default()->Self {Self {max_levels:1000,max_refinements:8,max_fibers:10000}}
}
fn error(s:&str)->CamError {CamError::InvalidInput(format!("contour: {s}"))}

/// Pure generation: the caller retains the previous accepted Toolpath on Err.
/// Raw stock removal and holder envelopes are supplied by the stock/verification
/// lane; this finishing strategy protects the complete supplied part/fixture mesh.
pub fn generate(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tolerance:Tolerance,limits:Limits,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let path=generate_unlinked(operation,setup,tool,mesh,tolerance,limits,runner)?;
    let cutter=Cutter::from_tool(tool)?;
    crate::strategy_linking::finish(&path,operation,&cutter,mesh,None,&setup.heights,&tool.feeds,tolerance,runner)
}
pub(crate) fn generate_unlinked(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tolerance:Tolerance,limits:Limits,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    operation.validate().map_err(CamError::InvalidInput)?; tolerance.validate()?;
    let params=match &operation.strategy {Strategy::Contour3D(p)=>p,_=>return Err(error("operation is not Contour3D"))};
    params.containment.validate().map_err(CamError::InvalidInput)?;
    if !operation.enabled {return Err(error("operation is disabled"));}
    if operation.setup!=setup.id || operation.tool!=tool.id {return Err(error("setup/tool identity mismatch"));}
    if operation.tool_axis.is_some_and(|p|p.normalized().is_none_or(|a|a.distance(Point3::new(0.0,0.0,1.0))>tolerance.angle_eps)) {
        return Err(error("only machine +Z cutter orientation is supported"));
    }
    if operation.stock_to_leave.axial!=0.0 {return Err(error("axial stock allowance requires a separate surface offset"));}
    if limits.max_refinements>16 {return Err(error("refinement depth exceeds 16"));}
    if tool.holder.is_some() {return Err(error("holder verification requires the collision-verification lane"));}
    let cutter=Cutter::from_tool(tool)?;
    let feeds=operation.feeds.as_ref().unwrap_or(&tool.feeds); feeds.validate().map_err(CamError::InvalidInput)?;
    let heights=operation.heights.as_ref().unwrap_or(&setup.heights); heights.validate().map_err(CamError::InvalidInput)?;
    if mesh.is_empty() {return Err(CamError::EmptyTarget{operation_id:operation.id.to_string()});}
    if heights.clearance<=mesh.bounds.max.z+tolerance.distance_eps {return Err(error("clearance must be above the full protected mesh"));}
    for name in operation.targets.protected_bodies() {
        if !mesh.faces.iter().any(|f|f.solid==name) {return Err(CamError::UnresolvedTarget{solid:name,face:None});}
    }
    for reference in operation.targets.drive_faces.iter().chain(&operation.targets.avoid_faces).chain(&setup.protected) {
        if mesh.resolve_faces(reference).is_empty() {return Err(CamError::UnresolvedTarget{solid:reference.solid.clone(),face:reference.face.clone()});}
    }
    // Full bodies drive interval topology; face selections only clip output.
    let mut builder=MeshBuilder::new(tolerance);
    let faces:Vec<_>=mesh.faces.iter().map(|f|builder.add_face(f.clone())).collect();
    for t in &mesh.triangles {
        let source=&mesh.faces[t.face as usize];
        if !source.protected_only && operation.targets.target_solids.contains(&source.solid) {
            builder.add_triangle(faces[t.face as usize],t.a,t.b,t.c).map_err(|e|error(&e.to_string()))?;
        }
    }
    let target=builder.finish();
    let boundary_region=crate::drop::containment::boundary_region(operation,&cutter,mesh,tolerance,&params.containment)?;
    if target.is_empty() {return Err(error("selected target bodies contain no machinable triangles"));}
    let [bottom,top]=params.containment.z_range.unwrap_or([heights.bottom.unwrap_or(target.bounds.min.z),heights.top.unwrap_or(target.bounds.max.z)]);
    let mut zs=waterline::levels(top,bottom,params.step_down,limits.max_levels)?;
    if operation.tolerance<=8.0*tolerance.distance_eps {return Err(error("machining tolerance must exceed eight numeric distance epsilons"));}
    let padding=operation.tolerance*0.25;
    let stock=operation.stock_to_leave.radial;
    let safety=PushContext{mesh,cutter:&cutter,index:None,tolerance,allowance:stock}; safety.validate()?;
    if params.adaptive_step_down && zs.len()>1 {
        let drive=PushContext{mesh:&target,cutter:&cutter,index:None,tolerance,allowance:stock+padding};
        let settings=Sampling{spacing:params.fiber_step,minimum:params.fiber_step.min(operation.tolerance)*0.25,max_fibers:limits.max_fibers,..Sampling::default()};
        zs=waterline::adaptive_z::refine(&zs,operation.tolerance,limits.max_levels,runner,|z,runner|Ok(waterline::generate(&drive,z,settings,runner)?.loops))?;
    }
    let mut path=Toolpath::new(operation.id.clone(),tool.id.clone()); let mut path_id=0u32;
    // Scratch motion stream stays local until every level has been verified.
    for (level_index,z) in zs.iter().copied().enumerate() {
        runner.stage("contour-level","Generate and verify contour level",level_index as u64,zs.len() as u64)?;
        let drive=PushContext{mesh:&target,cutter:&cutter,index:None,tolerance,allowance:stock+padding};
        // A section exactly through a mesh vertex/flat face has zero-width
        // contact components. Probe just below that critical plane, then restore
        // the requested Z and certify every emitted chord on the original mesh.
        let critical=target.triangles.iter().any(|t|t.vertices().iter().any(|p|(p.z-z).abs()<=tolerance.distance_eps));
        let probe_delta=(padding*padding/(2.*cutter.radius().max(padding))).max(tolerance.distance_eps*4.).min(padding*0.25);
        let probe_z=if critical {z-probe_delta}else{z};
        let mut step=params.fiber_step;
        let mut accepted=None;let mut failures=Vec::new();
        // Weave refusals refine the grid. Once refinement is spent (attempts
        // or the fiber budget), the finest step that failed to weave is rerun
        // closing the free gaps narrower than its grid: the tip of a sharp free
        // wedge stays unresolved at every spacing (the bracket at Z=-6.75).
        let (mut close,mut finest_weave,mut attempt)=(false,None::<f64>,0u32);
        loop {
            let settings=Sampling{spacing:step,minimum:step.min(operation.tolerance)*0.25,max_fibers:limits.max_fibers,adaptive:false,close_subgrid_gaps:close,..Sampling::default()};
            let mut generated=waterline::generate(&drive,probe_z,settings,runner);
            for phase in 1..=16 {
                if !matches!(&generated,Err(CamError::InvalidInput(s)) if s.contains("CL vertex coincides with grid intersection")) {break;}
                generated=waterline::generate_shifted(&drive,probe_z,settings,phase,runner);
            }
            if let Err(CamError::InvalidInput(s))=&generated {failures.push(format!("step {step}: {s}"));}
            let mut level=match generated {
                Ok(level)=>level,
                Err(CamError::InvalidInput(s)) if !close && s.starts_with("waterline weave:")=> {
                    finest_weave=Some(step);
                    if attempt<limits.max_refinements {attempt+=1;step*=0.5;} else {close=true;}
                    continue;
                },
                Err(CamError::InvalidInput(s)) if !close && s.contains("fiber budget exceeded") && finest_weave.is_some()=> {close=true;step=finest_weave.unwrap_or(step);continue;},
                Err(CamError::InvalidInput(s))=>return Err(error(&format!("Z={z}: {s}; refinement history {failures:?}"))),
                Err(e)=>return Err(error(&format!("section at Z={z}: {e}"))),
            };
            for boundary in &mut level.loops {for p in &mut boundary.points {p.z=z;}}
            let mut safe=true;
            'verify: for boundary in &level.loops {
                for p in boundary.points.windows(2) {
                    // Certification follows the same contained/selected spans
                    // that will be emitted. Irrelevant outside corners must not
                    // force refinement of a small contained machining patch.
                    for (a,b) in boundary_region.clip_boundary(p[0],p[1],runner)? {
                        if a.distance(b)<=tolerance.distance_eps || !selected(operation,&params.containment,&target,&cutter,a,b,stock+operation.tolerance,tolerance) {continue;}
                        let hits=safety.segment(a,b,runner).map_err(|e|error(&format!("chord at {a:?}: {e}")))?;
                        if let Some(hit)=hits.first() {failures.push(format!("unsafe contained chord at {a:?}, triangle {:?}",hit.lower_contact.triangle_id));safe=false;break 'verify;}
                    }
                }
            }
            if safe {accepted=Some(level);break;}
            failures.push(format!("step {step}: chord verification"));
            // A closed level keeps refining for its chords, bounded by the
            // fiber budget (whose error then ends the level).
            if attempt>=limits.max_refinements && !close {break;}
            attempt+=1;step*=0.5;
        }
        let level=accepted.ok_or_else(||error(&format!("Z={z}: cut-segment verification failed within the refinement budget; {failures:?}")))?;
        if critical {path.report.approximations.push(crate::ApproximationLimit{kind:"critical-section-probe".into(),bound:Some(probe_delta),detail:Some(format!("Section probed at Z={probe_z}; output remains Z={z}, with full original-mesh chord certification"))});}
        for warning in &level.warnings {path.report.warn(Diagnostic::new("waterline-level",warning));}
        let before=path.motions.len();
        // Stable area/coordinate order from weave, with explicit winding choice.
        for boundary in &level.loops {
            let mut points=boundary.points.clone();
            if params.direction==CutDirection::Conventional {points.reverse();}
            let mut spans=Vec::new();for pair in points.windows(2) {spans.extend(boundary_region.clip_boundary(pair[0],pair[1],runner)?);}
            for (a,b) in spans {
                if !selected(operation,&params.containment,&target,&cutter,a,b,stock+operation.tolerance,tolerance) {continue;}
                if a.distance(b)<=tolerance.distance_eps {continue;}
                // Joining consecutive clipped spans keeps full loops down. Any
                // gap or separate loop gets an independently verified approach.
                let previous=path.motions.last().and_then(|m|m.end());
                if previous!=Some(a) {
                    if let Some(p)=previous.filter(|p|p.z<heights.clearance) {
                        verify_vertical(&safety,p,heights.clearance,runner)?;
                        path.motions.push(Motion::retract(Point3::new(p.x,p.y,heights.clearance),Feed::default()));
                    }
                    verify_vertical(&safety,a,heights.clearance,runner)?;
                    path.motions.push(Motion::rapid(Point3::new(a.x,a.y,heights.clearance)).with_path(path_id));
                    path.motions.push(Motion::plunge(a,feeds.plunge_feed).with_path(path_id));
                }
                path.motions.push(Motion::cut(b,feeds.cutting_feed).with_path(path_id));
            }
            path_id=path_id.checked_add(1).ok_or_else(||error("path identity overflow"))?;
        }
        if path.motions.len()==before {path.report.warn(Diagnostic::new("empty-contour-region",format!("no selected contour at Z={z}")));}
        if let Some(p)=path.motions.last().and_then(|m|m.end()) {
            if p.z<heights.clearance {
                verify_vertical(&safety,p,heights.clearance,runner)?;
                path.motions.push(Motion::retract(Point3::new(p.x,p.y,heights.clearance),Feed::default()));
            }
        }
        path.report.counts.passes+=level.loops.len() as u64;
        path.report.counts.samples+=(level.x.len()+level.y.len()) as u64;
    }
    path.report.counts.triangles=mesh.triangle_count() as u64;
    path.report.counts.motions=path.motions.len() as u64;
    path.report.warn(Diagnostic::new("retract-linking","Links use verified full-clearance retracts; arc fitting and keep-down linking are deferred"));
    if path.motions.is_empty() {return Err(error("no reachable contour in the selected region"));}
    path.report.finish();
    Ok(path)
}

pub(crate) fn verify_vertical(context:&PushContext<'_>,point:Point3,safe:f64,runner:&mut Runner<'_>)->Result<(),CamError> {
    // For monotone cutter profiles, the vertical sweep is exactly the lower
    // profile plus a shaft extended by the retract distance. No Z samples.
    let mut extended=context.cutter.clone();
    let extra=(safe-point.z).max(0.0);
    let shaft_radius=extended.radius().max(extended.shaft_radius());
    match &mut extended {
        Cutter::Flat(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(shaft_radius);},
        Cutter::Ball(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(shaft_radius);},
        Cutter::Bull(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(shaft_radius);},
        Cutter::Cone(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(shaft_radius);},
        Cutter::BallCone(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(shaft_radius);},
        Cutter::Frustum(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(shaft_radius);},
        Cutter::Lollipop(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(shaft_radius);},
        Cutter::Dovetail(c)=>{c.shank.shaft_length+=extra;c.shank.shaft_radius=Some(shaft_radius);},
    }
    let check=PushContext{mesh:context.mesh,cutter:&extended,index:None,tolerance:context.tolerance,allowance:context.allowance};
    let delta=2.0*context.tolerance.distance_eps;
    if !check.segment(Point3::new(point.x-delta,point.y,point.z),Point3::new(point.x+delta,point.y,point.z),runner)?.is_empty() {
        return Err(error("vertical approach/retract intersects protected material"));
    }
    Ok(())
}

// Conservative surface selection: every possibly contacting triangle in the
// expanded segment envelope must be selected and within the slope interval.
// This may trim extra near a seam, but never machines an explicitly avoided face.
fn selected(operation:&Operation,containment:&Containment,mesh:&MeshSnapshot,cutter:&Cutter,a:Point3,b:Point3,allowance:f64,tol:Tolerance)->bool {
    if containment.slope_deg.is_none() && operation.targets.drive_faces.is_empty() && operation.targets.avoid_faces.is_empty() {return true;}
    let radius=cutter.radius()+allowance;
    let query=crate::Bounds3::new(Point3::new(a.x.min(b.x)-radius,a.y.min(b.y)-radius,a.z),
        Point3::new(a.x.max(b.x)+radius,a.y.max(b.y)+radius,a.z+cutter.total_height()));
    let mut count=0;
    for t in &mesh.triangles {
        if !query.intersects(&t.bounds) || t.normal.z.abs()>=1.0-tol.angle_eps {continue;}
        count+=1;let face=&mesh.faces[t.face as usize];
        let matches=|r:&crate::TargetRef|r.solid==face.solid && r.face.as_ref().is_none_or(|f|*f==face.face);
        if operation.targets.avoid_faces.iter().any(matches) || (!operation.targets.drive_faces.is_empty() && !operation.targets.drive_faces.iter().any(matches)) {return false;}
        if let Some([lo,hi])=containment.slope_deg {
            let slope=t.normal.z.abs().clamp(0.0,1.0).acos().to_degrees();
            if slope<lo-tol.angle_eps || slope>hi+tol.angle_eps {return false;}
        }
    }
    count>0
}

