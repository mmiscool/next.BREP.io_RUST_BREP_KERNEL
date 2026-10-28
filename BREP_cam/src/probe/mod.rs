//! Probe planning on picked geometry. Output stays local until fully checked.
use crate::*;
pub mod geometry;
pub mod inspection;
pub mod results;
mod safety;
fn invalid(s:&str)->CamError {CamError::InvalidInput(format!("probe: {s}"))}
pub fn nominals(path:&Toolpath)->Vec<ProbeCycle> {path.motions.iter().filter_map(|m|if let MotionKind::Probe {cycle}=&m.kind {Some(cycle.clone())}else{None}).collect()}
pub(crate) fn generate(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,runner:&mut Runner<'_>,kind:ProbeKind,points:&[ProbePoint],settings:&ProbeSettings,wcs:Option<&WorkOffset>)->Result<Toolpath,CamError> {
    runner.token().check()?;operation.validate().map_err(CamError::InvalidInput)?;tol.validate()?;settings.validate().map_err(CamError::InvalidInput)?;geometry::validate_pattern(kind,points)?;
    if !operation.enabled||operation.setup!=setup.id||operation.tool!=tool.id {return Err(invalid("disabled operation or setup/tool identity mismatch"));}
    if operation.tool_axis.is_some_and(|a|a.distance(Point3::Z)>tol.angle_eps)||tool.holder.is_some() {return Err(invalid("only a vertical stylus without an unmodelled holder is supported"));}
    if (tool.geometry.diameter-2.*settings.stylus_radius).abs()>tol.distance_eps || (tool.geometry.shaft_diameter()-2.*settings.shank_radius).abs()>tol.distance_eps || tool.geometry.overall_length()+tol.distance_eps<settings.shank_length+settings.stylus_radius {return Err(invalid("tool dimensions disagree with the physical probe stylus/shank"));}
    if operation.stock_to_leave.radial!=0.||operation.stock_to_leave.axial!=0. {return Err(invalid("probe targets are nominal surfaces, not stock offsets"));}
    if settings.overtravel>settings.approach_distance {return Err(invalid("overtravel exceeds approach distance"));}
    if mesh.is_empty() {return Err(invalid("empty protected mesh"));}
    let heights=operation.heights.as_ref().unwrap_or(&setup.heights);heights.validate().map_err(CamError::InvalidInput)?;
    if heights.clearance<=mesh.bounds.max.z+tol.distance_eps {return Err(invalid("clearance must be above the entire protected mesh"));}
    for target in operation.targets.protected_bodies() {if !mesh.faces.iter().any(|f|f.solid==target) {return Err(CamError::UnresolvedTarget {solid:target,face:None});}}
    let mut out=Toolpath::new(operation.id.clone(),tool.id.clone());let mut previous=None;
    for (i,p) in points.iter().enumerate() {
        runner.stage("probe-plan","Validate nominal contact and full approach",i as u64,points.len() as u64)?;p.validate().map_err(CamError::InvalidInput)?;safety::validate_pick(p,operation,mesh,tol)?;
        let target=p.position.add(p.normal.scale(settings.stylus_radius)).sub(Point3::Z.scale(settings.stylus_radius));let approach=target.add(p.normal.scale(settings.approach_distance));
        if approach.z>=heights.clearance || target.z>=heights.clearance {return Err(invalid("probe approach reaches/exceeds clearance"));}
        let safe=Point3::new(approach.x,approach.y,heights.clearance);
        if let Some(prev)=previous {safety::sweep(prev,safe,settings,mesh,tol,true,runner)?;}
        safety::sweep(safe,approach,settings,mesh,tol,true,runner)?;
        // Strict precontact check excludes any earlier tangency. The final
        // numerical epsilon at the nominal endpoint permits intended contact.
        let near=target.add(p.normal.scale(8.*tol.distance_eps));safety::sweep(approach,near,settings,mesh,tol,true,runner)?;safety::sweep(near,target,settings,mesh,tol,false,runner)?;
        let end=target.sub(p.normal.scale(settings.overtravel));safety::overtravel(target,end,settings,mesh,tol,p,runner)?;
        safety::sweep(target,approach,settings,mesh,tol,false,runner)?;
        out.motions.push(Motion::rapid(safe).with_path(i as u32));out.motions.push(Motion::new(MotionKind::Link {to:approach},Feed::rate(settings.feed)).with_path(i as u32));
        let cycle=ProbeCycle {target,approach,nominal:p.clone(),stylus_radius:settings.stylus_radius,overtravel:settings.overtravel,feed:settings.feed,kind,result_var:format!("{}_{}_{}",settings.result_prefix,operation.id.0.as_bytes().iter().map(|b|format!("{b:02x}")).collect::<String>(),p.id),point_index:i as u32,point_count:points.len() as u32,wcs:wcs.cloned()};
        out.motions.push(Motion::new(MotionKind::Probe {cycle},Feed::rate(settings.feed)).with_path(i as u32));out.motions.push(Motion::new(MotionKind::Link {to:approach},Feed::rate(settings.feed)).with_path(i as u32));out.motions.push(Motion::retract(safe,Feed::rate(settings.feed)).with_path(i as u32));previous=Some(safe);
    }
    out.report.counts.triangles=mesh.triangle_count() as u64;out.report.counts.samples=points.len() as u64;out.report.counts.passes=points.len() as u64;out.report.counts.motions=out.motions.len() as u64;
    out.report.warn(Diagnostic::new("probe-trigger-required","Probe motion requires a trigger-capable controller and abort on missing contact; nominal preview is not measured stock removal."));
    out.report.approximations.push(ApproximationLimit {kind:"probe-shank-capsule".into(),bound:Some(settings.shank_radius),detail:Some("Spherical stylus plus conservative capsule around the vertical cylindrical shank. Targets and safety use supplied mesh; no holder, machine travel or raw-stock collision claim.".into())});out.report.finish();Ok(out)
}
