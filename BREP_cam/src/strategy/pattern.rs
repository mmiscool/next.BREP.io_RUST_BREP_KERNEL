//! Source-operation expansion: transformed cut passes are independently
//! certified against the complete destination mesh and re-linked by C4.
use crate::{CamError,Point3,MeshSnapshot,Runner,Tolerance,Cutter};
use crate::model::*;
use crate::pattern::{Placement,transform_passes};
use crate::program::{OperationInput,SetupInput};
use crate::link::{LinkContext,LinkOptions};
use crate::planar::machining::OpContext;
fn invalid(s:&str)->CamError {CamError::InvalidInput(format!("pattern: {s}"))}
const MAX_INSTANCES:usize=1000;

pub fn placements(kind:&PatternKind)->Result<Vec<Placement>,CamError> {
    match kind {
        PatternKind::Linear{count,spacing,direction}=>{
            if *count==0 || *count as usize>MAX_INSTANCES || !spacing.is_finite() || *spacing<=0. {return Err(invalid("invalid linear count/spacing"));}
            let d=direction.normalized().ok_or_else(||invalid("invalid linear direction"))?;
            (0..*count).map(|i|Placement::rigid(RigidTransform::translation(d.scale(*spacing*i as f64)))).collect()
        }
        PatternKind::Circular{count,angle_deg,axis,origin}=>{
            if *count==0 || *count as usize>MAX_INSTANCES || !angle_deg.is_finite() || !origin.is_finite() {return Err(invalid("invalid circular count/angle/origin"));}
            let a=axis.normalized().ok_or_else(||invalid("invalid circular axis"))?;
            if a.cross(Point3::Z).length()>1e-10 {return Err(invalid("circular pattern requires spindle Z axis"));}
            (0..*count).map(|i|{
                let mut t=RigidTransform::axis_angle(a,(*angle_deg*i as f64).to_radians(),Point3::ZERO)?;
                t.translation=origin.sub(t.apply_vector(*origin));Placement::rigid(t)
            }).collect()
        }
        PatternKind::Mirror{origin,normal}=>Ok(vec![Placement::rigid(RigidTransform::identity())?,Placement::mirror(*origin,*normal)?]),
        PatternKind::WcsOffsets{..}=>Err(invalid("use wcs_setups for fixture-offset patterns")),
    }
}
fn sources<'a,'b>(op:&'b Operation,available:&'a [OperationInput])->Result<(&'b PatternParams,Vec<&'a OperationInput>),CamError> {
    let Strategy::Pattern(params)=&op.strategy else {return Err(invalid("requires Pattern strategy"));};
    if !op.enabled || params.sources.is_empty() || params.sources.len()>1000 {return Err(invalid("pattern needs enabled operation and bounded nonempty source list"));}
    let mut result=Vec::new();
    for (i,id) in params.sources.iter().enumerate() {
        if *id==op.id || params.sources[..i].contains(id) {return Err(invalid("cyclic or duplicate source operation"));}
        let found:Vec<_>=available.iter().filter(|s|s.operation.id==*id).collect();
        if found.len()!=1 {return Err(invalid("source identity is missing or ambiguous"));}
        let s=found[0];
        if !s.operation.enabled || matches!(s.operation.strategy,Strategy::Pattern(_)|Strategy::ManualNc(_)) || s.operation.tool!=s.tool.id || s.toolpath.tool!=s.tool.id || s.toolpath.operation!=s.operation.id {return Err(invalid("source must be an enabled resolved machining operation with matching identities"));}
        result.push(s);
    }
    Ok((params,result))
}
/// Generate separately identified operations in the requested stable order.
/// All output uses one common clearance above the full mesh, so transitions
/// between independently linked sections remain pure XY at clearance.
pub fn generate(op:&Operation,setup:&Setup,available:&[OperationInput],mesh:&MeshSnapshot,t:Tolerance,runner:&mut Runner<'_>)->Result<Vec<OperationInput>,CamError> {
    runner.token().check()?;t.validate()?;op.validate().map_err(CamError::InvalidInput)?;
    if op.setup!=setup.id {return Err(invalid("pattern setup identity mismatch"));}
    let (params,source)=sources(op,available)?;let places=placements(&params.kind)?;
    if source.len()*places.len()>MAX_INSTANCES {return Err(invalid("expanded operation budget exceeded"));}
    let mut common=mesh.bounds.max.z+op.linking.safe_distance.max(t.distance_eps*4.);
    for s in &source {for p in &places {
        let h=s.operation.heights.as_ref().unwrap_or(&setup.heights);
        common=common.max(p.point(Point3::new(0.,0.,h.clearance)).z);
    }}
    let mut ordered:Vec<_>=(0..places.len()).flat_map(|i|(0..source.len()).map(move|j|(i,j))).collect();
    if matches!(params.order,PatternOrder::ByTool){ordered.sort_by(|a,b|source[a.1].tool.id.as_str().cmp(source[b.1].tool.id.as_str()));}
    let mut output=Vec::new();let mut total_motions=0usize;
    for (i,j) in ordered {
        runner.work(1,"pattern-instance",output.len() as u64,(source.len()*places.len()) as u64)?;
        let id=format!("{}::instance:{i}::source:{j}",op.id);
        let generated=instance(&id,setup,source[j],&places[i],common,mesh,t,runner)?;
        total_motions=total_motions.saturating_add(generated.toolpath.motions.len());
        if total_motions>1_000_000 {return Err(invalid("expanded motion budget exceeded"));}
        output.push(generated);
    }
    runner.token().check()?;Ok(output)
}
pub(crate) fn instance(id:&str,setup:&Setup,source:&OperationInput,placement:&Placement,clearance:f64,mesh:&MeshSnapshot,t:Tolerance,runner:&mut Runner<'_>)->Result<OperationInput,CamError> {
    if source.operation.setup!=setup.id {return Err(invalid("source belongs to a different setup; use explicit WCS expansion"));}
    if source.tool.holder.is_some() {return Err(invalid("pattern holder sweep is not supported; supply a holder-free certified tool"));}
    let mut operation=source.operation.clone();operation.id=id.into();operation.setup=setup.id.clone();
    let mut heights=operation.heights.clone().unwrap_or_else(||setup.heights.clone());
    let shift=placement.point(Point3::ZERO).z;
    heights.clearance=clearance;heights.retract+=shift;heights.feed+=shift;heights.top=heights.top.map(|v|v+shift);heights.bottom=heights.bottom.map(|v|v+shift);
    operation.heights=Some(heights.clone());
    let feeds=operation.feeds.as_ref().unwrap_or(&source.tool.feeds);
    if source.toolpath.motions.iter().any(|m|m.kind.is_cutting() && m.feed!=Feed::Rate(feeds.cutting_feed) && !matches!(m.kind,MotionKind::Plunge{..}|MotionKind::LeadIn{..}|MotionKind::LeadOut{..})) {return Err(invalid("variable source cutting feed requires explicit regenerated operation"));}
    for protected in &setup.protected {if !mesh.faces.iter().any(|f|f.solid==protected.solid) {return Err(CamError::UnresolvedTarget{solid:protected.solid.clone(),face:protected.face.clone()});}}
    let passes=transform_passes(&source.toolpath,placement,operation.tolerance*0.25,runner)?;
    let context=OpContext::new("pattern",&operation,setup,&source.tool,mesh,None,t)?;
    let mut report=OperationReport::default();
    for p in &passes {context.certify_3d(&p.points,false,&mut report,runner)?;}
    let cutter=Cutter::from_tool(&source.tool)?;
    let linked=crate::link::with_current_pre_op_stock(|stock|{
    let link=LinkContext{mesh,cutter:&cutter,index:None,tolerance:t,heights:&heights,feeds,params:&operation.linking,options:LinkOptions{allow_reverse:false,reorder:false,chord_tolerance:operation.tolerance,..Default::default()},stock};
    crate::link::connect(&passes,&link,runner)})?;
    let mut toolpath=Toolpath::new(operation.id.clone(),source.tool.id.clone());toolpath.motions=linked.motions;toolpath.report=linked.report;
    toolpath.report.counts.samples+=report.counts.samples;toolpath.report.counts.candidates+=report.counts.candidates;toolpath.report.counts.passes=passes.len() as u64;toolpath.report.counts.motions=toolpath.motions.len() as u64;toolpath.report.finish();
    Ok(OperationInput{operation,tool:source.tool.clone(),toolpath})
}

/// Repeat source WCS-local CL geometry at physical fixture origins. The mesh is
/// complete PHYSICAL machine-space geometry. Each instance is checked there,
/// then converted back to WCS-local coordinates for the existing post contract.
/// WorkOffset number 1..6 denotes G54..G59; extended numbers remain post-defined
/// (Fanuc uses G54.1 P(number-6)). No motion is hidden in a WCS selection.
pub fn wcs_setups(op:&Operation,setup:&Setup,available:&[OperationInput],physical_mesh:&MeshSnapshot,t:Tolerance,runner:&mut Runner<'_>)->Result<Vec<SetupInput>,CamError> {
    runner.token().check()?;t.validate()?;op.validate().map_err(CamError::InvalidInput)?;
    if op.setup!=setup.id {return Err(invalid("pattern setup identity mismatch"));}
    let (params,source)=sources(op,available)?;
    let PatternKind::WcsOffsets{list}=&params.kind else {return Err(invalid("requires WcsOffsets pattern"));};
    if list.is_empty() || list.len().saturating_mul(source.len())>MAX_INSTANCES {return Err(invalid("invalid WCS instance budget"));}
    for (i,wcs) in list.iter().enumerate() {
        if wcs.number==0 || !wcs.origin.is_finite() || list[..i].iter().any(|w|w.number==wcs.number) {return Err(invalid("work offsets need unique positive numbers and finite physical origins"));}
    }
    let mut clearance=physical_mesh.bounds.max.z+op.linking.safe_distance.max(t.distance_eps*4.);
    for wcs in list {clearance=clearance.max(setup.heights.clearance+wcs.origin.z);for s in &source {clearance=clearance.max(s.operation.heights.as_ref().unwrap_or(&setup.heights).clearance+wcs.origin.z);}}
    let mut order:Vec<_>=(0..list.len()).flat_map(|i|(0..source.len()).map(move|j|(i,j))).collect();
    if matches!(params.order,PatternOrder::ByTool){order.sort_by(|a,b|source[a.1].tool.id.as_str().cmp(source[b.1].tool.id.as_str()));}
    let mut output:Vec<SetupInput>=Vec::new();let mut previous=None;let mut total_motions=0usize;
    for (i,j) in order {
        runner.work(1,"pattern-wcs",output.len() as u64,list.len() as u64)?;
        let wcs=&list[i];let placement=Placement::rigid(RigidTransform::translation(wcs.origin))?;
        let mut generated=instance(&format!("{}::offset:{i}::source:{j}",op.id),setup,source[j],&placement,clearance,physical_mesh,t,runner)?;
        if previous!=Some(i) {
            let mut destination=setup.clone();destination.id=format!("{}::offset:{i}::group:{}",op.id,output.len()).into();destination.wcs=wcs.clone();destination.heights.clearance=clearance-wcs.origin.z;
            output.push(SetupInput{setup:destination,operations:Vec::new()});previous=Some(i);
        }
        let group=output.last_mut().unwrap();generated.operation.setup=group.setup.id.clone();
        if let Some(h)=&mut generated.operation.heights {h.clearance-=wcs.origin.z;h.retract-=wcs.origin.z;h.feed-=wcs.origin.z;h.top=h.top.map(|z|z-wcs.origin.z);h.bottom=h.bottom.map(|z|z-wcs.origin.z);}
        for m in &mut generated.toolpath.motions {
            m.kind=match m.kind {
                MotionKind::ArcCw{to,center,plane,pitch}=>MotionKind::ArcCw{to:to.sub(wcs.origin),center:center.sub(wcs.origin),plane,pitch},
                MotionKind::ArcCcw{to,center,plane,pitch}=>MotionKind::ArcCcw{to:to.sub(wcs.origin),center:center.sub(wcs.origin),plane,pitch},
                MotionKind::Rapid{to}=>MotionKind::Rapid{to:to.sub(wcs.origin)},MotionKind::Cut{to}=>MotionKind::Cut{to:to.sub(wcs.origin)},MotionKind::Plunge{to}=>MotionKind::Plunge{to:to.sub(wcs.origin)},MotionKind::Retract{to}=>MotionKind::Retract{to:to.sub(wcs.origin)},MotionKind::Link{to}=>MotionKind::Link{to:to.sub(wcs.origin)},MotionKind::LeadIn{to}=>MotionKind::LeadIn{to:to.sub(wcs.origin)},MotionKind::LeadOut{to}=>MotionKind::LeadOut{to:to.sub(wcs.origin)},_=>return Err(invalid("linker emitted unsupported WCS motion")),
            };
        }
        total_motions=total_motions.saturating_add(generated.toolpath.motions.len());
        if total_motions>1_000_000 {return Err(invalid("expanded WCS motion budget exceeded"));}
        group.operations.push(generated);
    }
    runner.token().check()?;Ok(output)
}
