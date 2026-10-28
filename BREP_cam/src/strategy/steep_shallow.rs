//! Steep waterlines plus shallow parallel finishing with spatial overlap.
//! Overlap is a symmetric Euclidean contact-surface band around threshold mesh
//! edges: each family extends half the requested distance onto the other side.
use std::collections::BTreeMap;
use crate::{CamError,Cutter,CutterProfile,Diagnostic,Feed,MeshSnapshot,Motion,MotionKind,Operation,Point3,Runner,Setup,Strategy,Tool,Toolpath,Tolerance,Contour3dParams,ParallelParams,ApproximationLimit};
use crate::drop::DropCutter;
use crate::push::{PushContext,Direction,Fiber};
use crate::waterline::contour::{self,Limits};

fn error(s:&str)->CamError {CamError::InvalidInput(format!("steep/shallow: {s}"))}
fn key(p:Point3)->[u64;3] {[p.x,p.y,p.z].map(|v|if v==0.0 {0}else{v.to_bits()})}
fn segment_distance(p:Point3,a:Point3,b:Point3)->f64 {
    let d=b.sub(a);let length=d.length_squared();
    if length==0.0 {p.distance(a)}else{p.distance(a.lerp(b,(p.sub(a).dot(d)/length).clamp(0.0,1.0)))}
}

struct Regions {classes:Vec<Option<bool>>,solids:Vec<String>,boundary:Vec<(Point3,Point3)>,boundary_solids:Vec<String>}
impl Regions {
    fn new(op:&Operation,mesh:&MeshSnapshot,threshold:f64,tol:Tolerance,runner:&mut Runner<'_>)->Result<Self,CamError> {
        let containment=match &op.strategy {Strategy::SteepShallow(p)=>&p.containment,_=>return Err(error("wrong strategy"))};
        let mut classes=vec![None;mesh.triangle_count()];
        let mut solids=vec![String::new();mesh.triangle_count()];
        let mut edges=BTreeMap::<(String,[u64;3],[u64;3]),(Point3,Point3,u8)>::new();
        for t in &mesh.triangles {
            runner.work(1,"steep-shallow-classify",t.id as u64,mesh.triangle_count() as u64)?;
            let f=&mesh.faces[t.face as usize];let matches=|r:&crate::TargetRef|r.solid==f.solid && r.face.as_ref().is_none_or(|s|*s==f.face);
            if f.protected_only || !op.targets.target_solids.contains(&f.solid) || t.normal.z < -tol.angle_eps || op.targets.avoid_faces.iter().any(matches) || (!op.targets.drive_faces.is_empty() && !op.targets.drive_faces.iter().any(matches)) {continue;}
            let slope=t.normal.z.clamp(0.0,1.0).acos().to_degrees();
            if containment.slope_deg.is_some_and(|[lo,hi]|slope<lo || slope>hi) {continue;}
            let steep=slope>threshold;classes[t.id as usize]=Some(steep);solids[t.id as usize]=f.solid.clone();
            for (a,b) in [(t.a,t.b),(t.b,t.c),(t.c,t.a)] {
                let ka=key(a);let kb=key(b);let (ka,kb)=if ka<kb {(ka,kb)}else{(kb,ka)};let k=(f.solid.clone(),ka,kb);
                edges.entry(k).or_insert((a,b,0)).2|=if steep {1}else{2};
            }
        }
        let mut boundary=Vec::new();let mut boundary_solids=Vec::new();
        for ((solid,_,_),(a,b,flags)) in edges {if flags==3 {boundary.push((a,b));boundary_solids.push(solid);}}
        Ok(Self{classes,solids,boundary,boundary_solids})
    }
    fn allowed(&self,id:u32,p:Point3,steep:bool,overlap:f64)->bool {
        let class=self.classes.get(id as usize).copied().flatten();
        if class==Some(steep) {return true;}
        class.is_some() && overlap>0.0 && self.boundary.iter().zip(&self.boundary_solids).any(|((a,b),solid)|self.solids.get(id as usize)==Some(solid) && segment_distance(p,*a,*b)<=overlap*0.5)
    }
}

/// Existing shared setup/tool/operation types, with full protected machine mesh.
pub fn generate(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,limits:Limits,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let path=generate_unlinked(operation,setup,tool,mesh,tol,limits,runner)?;
    let cutter=Cutter::from_tool(tool)?;
    crate::strategy_linking::finish(&path,operation,&cutter,mesh,None,&setup.heights,&tool.feeds,tol,runner)
}
pub(crate) fn generate_unlinked(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,limits:Limits,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    runner.token().check()?;operation.validate().map_err(CamError::InvalidInput)?;tol.validate()?;
    let params=match &operation.strategy {Strategy::SteepShallow(p)=>p,_=>return Err(error("requires SteepShallow parameters"))};
    if !params.threshold_angle_deg.is_finite() || !(0.0..=90.0).contains(&params.threshold_angle_deg) || !params.overlap.is_finite() || params.overlap<0.0 {return Err(error("invalid slope threshold or overlap distance"));}
    if !operation.enabled || operation.setup!=setup.id || operation.tool!=tool.id {return Err(error("disabled operation or setup/tool identity mismatch"));}
    if tool.holder.is_some() {return Err(error("holder verification is not available"));}
    if mesh.is_empty() {return Err(error("empty protected mesh"));}
    for reference in &setup.protected {if mesh.resolve_faces(reference).is_empty() {return Err(CamError::UnresolvedTarget{solid:reference.solid.clone(),face:reference.face.clone()});}}
    let cutter=Cutter::from_tool(tool)?;let drop=DropCutter::new(mesh,&cutter,None,tol)?;
    let regions=Regions::new(operation,mesh,params.threshold_angle_deg,tol,runner)?;
    if params.overlap>0.0 && regions.boundary.is_empty() && regions.classes.contains(&Some(true)) && regions.classes.contains(&Some(false)) {
        return Err(error("requested overlap has no shared threshold mesh edges; weld/resolve the selected surface boundary"));
    }
    let heights=operation.heights.as_ref().unwrap_or(&setup.heights);let feeds=operation.feeds.as_ref().unwrap_or(&tool.feeds);
    let index=crate::AnyIndex::build(mesh,crate::TreeOptions::default());
    let push=PushContext{mesh,cutter:&cutter,index:Some(&index),tolerance:tol,allowance:operation.stock_to_leave.radial+operation.tolerance*0.25};
    let safety=PushContext{allowance:operation.stock_to_leave.radial,..push};
    let mut result=Toolpath::new(operation.id.clone(),tool.id.clone());let mut family_counts=[0u64;2];let mut path_id=0u32;
    for steep in [true,false] {
        if !regions.classes.iter().any(|c|*c==Some(steep)) {continue;}
        let mut source=operation.clone();let mut containment=params.containment.clone();containment.slope_deg=None;
        if steep {
            source.strategy=Strategy::Contour3D(Contour3dParams{step_down:params.steep_step_down,adaptive_step_down:false,fiber_step:0.5,containment,..Contour3dParams::default()});
        } else {
            // Parallel clips the part surface Z range before cutter projection.
            source.strategy=Strategy::Parallel(ParallelParams{stepover:params.shallow_stepover.clone(),sample_step:0.5,adaptive_sampling:true,containment,..ParallelParams::default()});
        }
        let candidate=if steep {contour::generate_unlinked(&source,setup,tool,mesh,tol,limits,runner)?}else{crate::parallel::generate_unlinked(&source,&cutter,mesh,Some(&index),&setup.heights,&tool.feeds,tol,runner)?};
        result.report.warnings.extend(candidate.report.warnings);
        result.report.approximations.extend(candidate.report.approximations);
        result.report.counts.samples+=candidate.report.counts.samples;result.report.counts.candidates+=candidate.report.counts.candidates;result.report.counts.contacts+=candidate.report.counts.contacts;
        let mut cache=BTreeMap::<[u64;3],bool>::new();
        let mut classify=|p:Point3,runner:&mut Runner<'_>|->Result<bool,CamError> {
            if let Some(value)=cache.get(&key(p)) {return Ok(*value);}
            if cache.len()>=limits.max_fibers.saturating_mul(64) {return Err(error("contact classification budget exceeded"));}
            runner.work(1,"steep-shallow-contact",cache.len() as u64,0)?;
            let contact=if steep {nearest_contact(&push,p,runner)?}else{
                let hit=drop.point(p.x,p.y,mesh.bounds.min.z-1.0,runner)?;
                hit.point.contact.and_then(|c|Some((c.triangle_id?,c.point?)))
            };
            let (id,q)=contact.ok_or_else(||error("cannot resolve a candidate surface contact"))?;
            let allowed=regions.allowed(id,q,steep,params.overlap);
            cache.insert(key(p),allowed);Ok(allowed)
        };
        let mut accepted=Vec::new();let mut previous:Option<Point3>=None;
        let candidate_count=candidate.motions.len();
        for (motion_index,motion) in candidate.motions.into_iter().enumerate() {
            runner.work(1,"steep-shallow-clip",motion_index as u64,candidate_count as u64)?;
            if let MotionKind::Cut{to}=motion.kind {
                let from=previous.ok_or_else(||error("candidate cut has no start"))?;
                let mut stack=vec![(from,to,0u32)];
                while let Some((a,b,depth))=stack.pop() {
                    let mid=a.lerp(b,0.5);let ca=classify(a,runner)?;let cm=classify(mid,runner)?;let cb=classify(b,runner)?;
                    if ca && cm && cb {accepted.push((a,b));}
                    else if (ca||cm||cb) && a.distance(b)>operation.tolerance*0.1 && depth<24 {
                        stack.push((mid,b,depth+1));stack.push((a,mid,depth+1));
                    }
                }
            }
            previous=motion.end().or(previous);
        }
        result.report.counts.samples+=cache.len() as u64;
        let mut previous_cut=None;
        let accepted_count=accepted.len();
        for (span_index,(mut a,mut b)) in accepted.into_iter().enumerate() {
            runner.work(1,"steep-shallow-emit",span_index as u64,accepted_count as u64)?;
            if let Some([lo,hi])=params.containment.z_range.filter(|_|steep) {
                let dz=b.z-a.z;
                if dz==0.0 {if a.z<lo || a.z>hi {continue;}}
                else {let t0=((lo-a.z)/dz).min((hi-a.z)/dz).max(0.0);let t1=((lo-a.z)/dz).max((hi-a.z)/dz).min(1.0);if t1<=t0 {continue;}let end=a.lerp(b,t1);a=a.lerp(b,t0);b=end;}
            }
            if a.distance(b)<=tol.distance_eps {continue;}
            if previous_cut!=Some(a) {
                if let Some(p)=previous_cut {if steep {contour::verify_vertical(&safety,p,heights.clearance,runner)?;}result.motions.push(Motion::retract(Point3::new(p.x,p.y,heights.clearance),Feed::RAPID).with_path(path_id));}
                path_id=path_id.checked_add(1).ok_or_else(||error("path identity overflow"))?;
                if steep {contour::verify_vertical(&safety,a,heights.clearance,runner)?;}
                result.motions.push(Motion::rapid(Point3::new(a.x,a.y,heights.clearance)).with_path(path_id));
                result.motions.push(Motion::plunge(a,feeds.plunge_feed).with_path(path_id));
            }
            result.motions.push(Motion::cut(b,feeds.cutting_feed).with_path(path_id));previous_cut=Some(b);family_counts[if steep {0}else{1}]+=1;
        }
        if let Some(p)=previous_cut {if steep {contour::verify_vertical(&safety,p,heights.clearance,runner)?;}result.motions.push(Motion::retract(Point3::new(p.x,p.y,heights.clearance),Feed::RAPID).with_path(path_id));}
    }
    if result.motions.is_empty() {return Err(error("no reachable selected steep or shallow region"));}
    result.report.warn(Diagnostic::new("steep-shallow-counts",format!("steep={} shallow={} cut spans; overlap={} model units",family_counts[0],family_counts[1],params.overlap)));
    for (label,count) in ["steep","shallow"].iter().zip(family_counts) {if count==0 {result.report.warn(Diagnostic::new("empty-finishing-family",format!("no reachable {label} spans")));}}
    result.report.approximations.push(ApproximationLimit{kind:"contact-band-transition-resolution".into(),bound:Some(operation.tolerance*0.1),detail:Some("Symmetric Euclidean distance to shared steep/shallow mesh edges; sampled contact clipping; bound is transition bisection resolution, not a global feature-discovery or geodesic certificate".into())});
    result.report.counts.triangles=mesh.triangle_count() as u64;result.report.counts.passes=path_id as u64;result.report.counts.motions=result.motions.len() as u64;result.report.finish();
    runner.stage("steep-shallow-complete","Steep and shallow toolpath generated",result.report.counts.motions,result.report.counts.motions)?;Ok(result)
}

fn nearest_contact(context:&PushContext<'_>,p:Point3,runner:&mut Runner<'_>)->Result<Option<(u32,Point3)>,CamError> {
    let margin=2.0*(context.cutter.radius()+context.allowance)+1.0;let bounds=context.mesh.bounds.expanded(margin);
    let mut best=f64::INFINITY;let mut witness=None;
    for direction in [Direction::X,Direction::Y] {
        let (start,end)=match direction {Direction::X=>(Point3::new(bounds.min.x,p.y,p.z),Point3::new(bounds.max.x,p.y,p.z)),Direction::Y=>(Point3::new(p.x,bounds.min.y,p.z),Point3::new(p.x,bounds.max.y,p.z))};
        let fiber=context.fiber(&Fiber::new(direction,start,end),runner).map_err(|e|error(&format!("contact at {p:?}: {e}")))?;
        for interval in fiber.intervals {
            for (t,contact) in [(interval.lower,interval.lower_contact),(interval.upper,interval.upper_contact)] {
                let distance=start.lerp(end,t).distance_xy(p);
                if distance<best {if let (Some(id),Some(q))=(contact.triangle_id,contact.point) {best=distance;witness=Some((id,q));}}
            }
        }
    }
    Ok(witness)
}
