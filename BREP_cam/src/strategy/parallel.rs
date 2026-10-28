//! Parallel finishing over explicit containment loops or selected-face projection.
//! Source passes are clipped before projection; every cut chord is certified
//! against the complete protected mesh. Unproven links use full clearance.
use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, TriangleIndex, Tolerance, Runner};
use crate::drop::{DropCutter, path::{Span, Sampling, Dilated}};
use crate::model::*;

type Interval = (f64,f64);

/// Generate a new result transactionally. The caller retains its previous
/// accepted Toolpath if this returns an error (including cancellation).
/// `mesh` must include every selected target AND fixture in machine coordinates.
pub fn generate(operation: &Operation, cutter: &Cutter, mesh: &MeshSnapshot, index: Option<&dyn TriangleIndex>, default_heights: &Heights, default_feeds: &FeedsSpeeds, tolerance: Tolerance, runner: &mut Runner<'_>) -> Result<Toolpath,CamError> {
    let path=generate_unlinked(operation,cutter,mesh,index,default_heights,default_feeds,tolerance,runner)?;
    crate::strategy_linking::finish(&path,operation,cutter,mesh,index,default_heights,default_feeds,tolerance,runner)
}
pub(crate) fn generate_unlinked(operation: &Operation, cutter: &Cutter, mesh: &MeshSnapshot, index: Option<&dyn TriangleIndex>, default_heights: &Heights, default_feeds: &FeedsSpeeds, tolerance: Tolerance, runner: &mut Runner<'_>) -> Result<Toolpath,CamError> {
    operation.validate().map_err(CamError::InvalidInput)?;
    let params = match &operation.strategy { Strategy::Parallel(p) => p, _ => return Err(CamError::InvalidInput("parallel generator requires a parallel operation".into())) };
    if !operation.enabled { return Err(CamError::InvalidInput("operation is disabled".into())); }
    if let Some(axis) = operation.tool_axis {
        if axis.normalized().unwrap().distance(Point3::Z) > tolerance.angle_eps { return Err(CamError::InvalidInput("parallel finishing requires machine +Z tool axis; transform indexed setups first".into())); }
    }
    let heights = operation.heights.as_ref().unwrap_or(default_heights);
    let feeds = operation.feeds.as_ref().unwrap_or(default_feeds);
    heights.validate().map_err(CamError::InvalidInput)?; feeds.validate().map_err(CamError::InvalidInput)?;
    cutter.validate()?; tolerance.validate()?;
    if operation.stock_to_leave.radial < 0. || operation.stock_to_leave.axial < 0. {
        return Err(CamError::InvalidInput("parallel protected-surface finishing does not permit negative stock allowance".into()));
    }
    // Validate the physical shank before wrapping the bottom with allowance.
    let _validated = DropCutter::new(mesh,cutter,index,tolerance)?;
    if mesh.is_empty() { return Err(CamError::EmptyTarget {operation_id: operation.id.to_string()}); }
    for solid in operation.targets.protected_bodies() {
        if !mesh.faces.iter().any(|f| f.solid == solid) { return Err(CamError::UnresolvedTarget {solid,face:None}); }
    }
    for face in operation.targets.drive_faces.iter().chain(&operation.targets.avoid_faces) {
        if mesh.resolve_faces(face).is_empty() { return Err(CamError::UnresolvedTarget {solid:face.solid.clone(),face:face.face.clone()}); }
    }
    let selected = |face: &crate::mesh::FaceSource| {
        !face.protected_only && operation.targets.target_solids.contains(&face.solid)
            && (operation.targets.drive_faces.is_empty() || operation.targets.drive_faces.iter().any(|f| f.solid == face.solid && f.face.as_ref().map_or(true,|id|id == &face.face)))
    };
    let drive: Vec<_> = mesh.triangles.iter().filter(|t| selected(&mesh.faces[t.face as usize])).collect();
    if drive.is_empty() { return Err(CamError::InvalidInput(format!("operation {} has no reachable drive faces",operation.id))); }
    let angle = params.pass_angle_deg.to_radians(); let (sin,cos) = angle.sin_cos();
    let uv = |p:Point2| Point2::new(p.x*cos+p.y*sin,-p.x*sin+p.y*cos);
    let xy = |u:f64,v:f64| Point3::new(u*cos-v*sin,u*sin+v*cos,0.);
    let region=crate::drop::containment::region(operation,cutter,mesh,tolerance,&params.containment)?;
    let source=if region.boundary.is_empty() {&region.drive}else{&region.boundary};
    let points:Vec<_>=source.iter().flatten().copied().map(uv).collect();
    let radius=cutter.radius()+operation.stock_to_leave.radial;
    let expansion=(params.containment.offset+match params.containment.mode {ContainmentMode::ToolOutside=>radius,ContainmentMode::ToolInside=>-radius,_=>0.}).max(0.);
    let min_v=points.iter().map(|p|p.y).fold(f64::INFINITY,f64::min)-expansion;
    let max_v=points.iter().map(|p|p.y).fold(f64::NEG_INFINITY,f64::max)+expansion;
    let min_u=points.iter().map(|p|p.x).fold(f64::INFINITY,f64::min)-expansion;
    let max_u=points.iter().map(|p|p.x).fold(f64::NEG_INFINITY,f64::max)+expansion;
    let pass_count=((max_v-min_v)/params.stepover.distance).ceil();
    if !pass_count.is_finite() || pass_count>1_000_000. {return Err(CamError::InvalidInput("parallel pass budget exceeded".into()));}
    let floor = params.floor_z.or(heights.bottom).unwrap_or(mesh.bounds.min.z);
    if heights.clearance <= mesh.bounds.max.z + operation.stock_to_leave.axial + tolerance.distance_eps || heights.clearance <= floor {
        return Err(CamError::InvalidInput("clearance height must exceed the full protected mesh, allowance and floor".into()));
    }
    // The operation's tolerance bounds the material left: the scallop between
    // passes (the profile's height at half the stepover, its value on a flat
    // floor) plus the lift the adaptive sampler may give a segment's ends.
    // The sampler gets what the scallop leaves; a stepover whose scallop
    // alone uses most of the tolerance is reported, not silently exceeded.
    let mut result_warning: Option<Diagnostic> = None;
    let scallop = cutter.height_at_radius((0.5*params.stepover.distance).min(cutter.radius())).unwrap_or(0.0);
    let lift_budget = operation.tolerance - scallop;
    let chord_error = if lift_budget >= 0.25*operation.tolerance {
        lift_budget
    } else {
        result_warning = Some(Diagnostic::new("parallel-scallop-exceeds-tolerance", format!(
            "the stepover {} leaves a scallop of {scallop:.4} with this cutter, {} of the tolerance {}; reduce the stepover to meet the tolerance",
            params.stepover.distance, if scallop >= operation.tolerance { "more than all" } else { "most" }, operation.tolerance)));
        0.25*operation.tolerance
    };
    let settings = Sampling {sample_spacing:params.sample_step,min_sample_spacing:tolerance.sample_eps,
        chord_error,max_samples:1_000_000,floor_z:floor,..Default::default()};
    settings.validate()?;
    let expanded = Dilated {base:cutter,pad:operation.stock_to_leave.radial};
    let drop = DropCutter::from_profile(mesh,&expanded,index,tolerance)?;
    let mut result = Toolpath::new(operation.id.clone(),operation.tool.clone());
    if let Some(d) = result_warning.take() { result.report.warn(d); }
    result.report.counts.triangles = mesh.triangle_count() as u64;
    let mut spans = Vec::new();
    let mut selected_hit = vec![false;operation.targets.drive_faces.len()];
    runner.stage("parallel-region","Clip raster passes",0,pass_count as u64+1)?;
    for i in 0..=pass_count as u64 {
        let v = (min_v + i as f64*params.stepover.distance).min(max_v);
        // Preserve the established half-open raster schedule. Containment
        // itself is closed; the upper extent does not add a duplicate last pass.
        if v>=max_v {continue;}
        let clipped=region.clip(xy(min_u,v),xy(max_u,v),runner)?;
        let mut intervals:Vec<_>=clipped.into_iter().map(|(a,b)|(uv(Point2::new(a.x,a.y)).x,uv(Point2::new(b.x,b.y)).x)).collect();
        if params.zig_zag && i%2==1 { intervals.reverse(); }
        for (lo,hi) in intervals {
            if hi-lo <= tolerance.sample_eps { continue; }
            let (a,b) = if params.zig_zag && i%2==1 {(xy(hi,v),xy(lo,v))} else {(xy(lo,v),xy(hi,v))};
            for (j,f) in operation.targets.drive_faces.iter().enumerate() {
                if drive.iter().any(|t| {
                    let source = &mesh.faces[t.face as usize];
                    source.solid==f.solid && f.face.as_ref().map_or(true,|s|s==&source.face)
                        && !intersect(&[(lo,hi)],&scan(&t.vertices().iter().map(|p|uv(Point2::new(p.x,p.y))).collect::<Vec<_>>(),v)).is_empty()
                }) { selected_hit[j]=true; }
            }
            spans.push(Span::Line {start:a,end:b});
        }
    }
    if spans.is_empty() || selected_hit.iter().any(|hit| !hit) {
        return Err(CamError::InvalidInput(format!("operation {} has an empty or unreachable machining region/selected face",operation.id)));
    }
    runner.stage("parallel-project","Project raster passes",0,spans.len() as u64)?;
    // Uniform mode keeps requested sample positions but still certifies each
    // output chord. If certification needs refinement it fails explicitly.
    let (paths, report) = if params.adaptive_sampling { drop.adaptive(&spans,&settings,runner)? } else {
        let (mut paths,mut report) = drop.uniform(&spans,&settings,runner)?;
        for path in &mut paths {
            for j in 1..path.len() {
                let bound = drop.segment_bound(path[j-1].position(),path[j].position(),&settings,&mut report,runner)?;
                if bound > path[j-1].z.min(path[j].z)+settings.chord_error { return Err(CamError::InvalidInput("uniform cut chord cannot be certified; reduce sample spacing or enable adaptive sampling".into())); }
                path[j-1].z=path[j-1].z.max(bound); path[j].z=path[j].z.max(bound);
            }
        }
        (paths,report)
    };
    result.report=report;
    let mut previous:Option<Point3>=None;
    for (id,path) in paths.iter().enumerate() {
        if path.iter().all(|p|p.contact.is_none()) { return Err(CamError::InvalidInput(format!("operation {} pass {id} has no reachable surface",operation.id))); }
        let points:Vec<_> = path.iter().map(|p|Point3::new(p.x,p.y,p.z+operation.stock_to_leave.axial)).collect();
        if points.iter().any(|p|p.z>=heights.clearance) {return Err(CamError::InvalidInput("projected path exceeds clearance height".into()));}
        let start=points[0];
        let mut linked=false;
        if let Some(last)=previous {
            if params.zig_zag && last.distance(start)<=operation.linking.keep_down_distance && operation.stock_to_leave.radial==0. {
                let bound=drop.segment_bound(last,start,&settings,&mut result.report,runner)?+operation.stock_to_leave.axial;
                // Only permit a down link if it stays on this same clipped pass;
                // cross-region/stock links await the linking lane's stock proof.
                if last.distance_xy(start)<=tolerance.point_eps && last.z.min(start.z)>=bound {
                    result.motions.push(Motion::link(start,feeds.cutting_feed).with_path(id as u32)); linked=true;
                }
            }
        }
        if !linked {
            if let Some(last)=previous { result.motions.push(Motion::retract(Point3::new(last.x,last.y,heights.clearance),Feed::RAPID).with_path(id as u32)); }
            result.motions.push(Motion::rapid(Point3::new(start.x,start.y,heights.clearance)).with_path(id as u32));
            result.motions.push(Motion::plunge(start,feeds.plunge_feed).with_path(id as u32));
        }
        for &p in points.iter().skip(1) { result.motions.push(Motion::cut(p,feeds.cutting_feed).with_path(id as u32)); }
        previous=points.last().copied();
        runner.work(1,"parallel-link",id as u64+1,paths.len() as u64)?;
    }
    if let Some(last)=previous { result.motions.push(Motion::retract(Point3::new(last.x,last.y,heights.clearance),Feed::RAPID)); }
    result.report.counts.passes=paths.len() as u64; result.report.counts.motions=result.motions.len() as u64;
    result.report.finish();
    runner.stage("parallel-complete","Parallel toolpath generated",paths.len() as u64,paths.len() as u64)?;
    Ok(result)
}

fn crossings(loop_: &[Point2], v:f64) -> Vec<f64> {
    loop_.iter().zip(loop_.iter().cycle().skip(1)).take(loop_.len()).filter_map(|(a,b)| {
        if (a.y<=v && b.y>v)||(b.y<=v && a.y>v) {Some(a.x+(v-a.y)*(b.x-a.x)/(b.y-a.y))} else {None}
    }).collect()
}
fn scan(loop_: &[Point2],v:f64) -> Vec<Interval> {
    let mut x=crossings(loop_,v); x.sort_by(f64::total_cmp);
    x.chunks_exact(2).filter_map(|p|(p[1]>p[0]).then_some((p[0],p[1]))).collect()
}
fn union(mut intervals:Vec<Interval>) -> Vec<Interval> {
    intervals.sort_by(|a,b|a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    let mut out:Vec<Interval>=Vec::new();
    for (a,b) in intervals { if let Some(last)=out.last_mut() {if a<=last.1 {last.1=last.1.max(b);continue;}} out.push((a,b)); }
    out
}
fn intersect(a:&[Interval],b:&[Interval]) -> Vec<Interval> {
    union(a.iter().flat_map(|x|b.iter().filter_map(move |y| {let lo=x.0.max(y.0);let hi=x.1.min(y.1);(lo<hi).then_some((lo,hi))})).collect())
}
