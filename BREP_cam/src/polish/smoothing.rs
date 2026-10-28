//! C4 arc fitting with a full-curve drop-cutter certificate. No new fitter.
//! Only horizontal, +Z straight cut runs are replaced. Others are retained.
use crate::{CamError,Cutter,CutterProfile,MeshSnapshot,Runner,Tolerance};
use crate::model::{ApproximationLimit,ArcPlane,Motion,MotionKind,Point3,SmoothingParams,Tool,Toolpath};
use crate::drop::{DropCutter,path::Dilated};
use super::{invalid,validate_path};

// Bound the entire arc interval by a disk about its midpoint, not sampled
// endpoint clearance. Adaptive subdivision tightens conservative bounds.
fn certify(from:Point3,m:&Motion,drop:&DropCutter<'_>,work:&mut usize,runner:&mut Runner<'_>)->Result<(),CamError> {
 let (to,center,cw)=match m.kind {MotionKind::ArcCw{to,center,..}=>(to,center,true),MotionKind::ArcCcw{to,center,..}=>(to,center,false),_=>return Ok(())};
 let (sweep,r)=crate::arcfit::arc_sweep(from,to,center,ArcPlane::Xy,cw).ok_or_else(||invalid("invalid fitted arc"))?;
 let mut stack=vec![(0.,1.,0u32)];
 while let Some((a,b,depth))=stack.pop() {
  runner.token().check()?;*work+=1;if *work>100_000 {return Err(invalid("smoothing certificate work budget exceeded"));}
  let mid=crate::arcfit::arc_point(from,&m.kind,(a+b)*0.5).ok_or_else(||invalid("invalid fitted arc point"))?;
  let pad=2.*r*(sweep*(b-a)*0.25).sin().abs();
  let profile=Dilated{base:drop.cutter(),pad};
  let wide=DropCutter::from_profile(drop.mesh(),&profile,None,drop.tolerance)?;
  let contact=wide.point(mid.x,mid.y,mid.z-2.*drop.tolerance.distance_eps,runner)?;
  if contact.point.z<=mid.z+drop.tolerance.distance_eps {continue;}
  // A point on the candidate itself gouges: no amount of refinement helps.
  if drop.point(mid.x,mid.y,mid.z-2.*drop.tolerance.distance_eps,runner)?.point.z>mid.z+drop.tolerance.distance_eps {return Err(invalid("smoothed arc gouges protected mesh"));}
  if depth>=20 {return Err(invalid("smoothed arc clearance could not be certified"));}
  let c=(a+b)*0.5;stack.push((c,b,depth+1));stack.push((a,c,depth+1));
 }
 Ok(())
}
/// Transactional smoothing of linked paths, with the complete protected mesh
/// in the same machine coordinates. The supplied tool must match the path.
/// Holders and non-vertical cutting axes are refused; no holder-sweep claim.
pub fn smooth(path:&Toolpath,opts:&SmoothingParams,tool:&Tool,mesh:&MeshSnapshot,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
 runner.token().check()?;if !opts.enabled {return Ok(path.clone());}
 if !opts.tolerance.is_finite() || opts.tolerance<=0. {return Err(invalid("smoothing tolerance must be finite and positive"));}
 validate_path(path)?;tolerance.validate()?;
 if tool.id!=path.tool || tool.holder.is_some() {return Err(invalid("smoothing requires the matching tool without a holder"));}
 let cutter=Cutter::from_tool(tool)?;
 if matches!(cutter.kind(),crate::cutter::CutterKind::Lollipop|crate::cutter::CutterKind::Dovetail) {return Err(invalid("smoothing requires a monotone bottom cutter profile"));}
 let drop=DropCutter::new(mesh,&cutter,None,tolerance)?;
 let fit=crate::arcfit::ArcFitOptions{tolerance:opts.tolerance*0.25,helical:false,planes:[Some(ArcPlane::Xy),None,None],..Default::default()};fit.validate()?;
 let mut out=path.clone();out.motions.clear();let mut at:Option<Point3>=None;let mut i=0;let mut arcs=0;let mut work=0;
 while i<path.motions.len() {
  runner.token().check()?;let m=&path.motions[i];
  if !matches!(m.kind,MotionKind::Cut{..}) {out.motions.push(m.clone());if let Some(p)=m.end(){at=Some(p);}i+=1;continue;}
  if m.tool_axis.map_or(false,|a|a.normalized().map_or(true,|a|a.distance(Point3::Z)>tolerance.angle_eps)) {return Err(invalid("smoothing requires +Z cutting axis"));}
  if m.feed.value().map_or(true,|f|f<=0.) {return Err(invalid("smoothing cut requires positive feed"));}
  let start=at.ok_or_else(||invalid("smoothing requires a linked start"))?;
  // Bound the fitter's quadratic work and cancellation latency. No chunk
  // crosses a feed, motion kind, path ID, tool-axis or exact Z boundary.
  let mut j=i;while j<path.motions.len() && j-i<128 {let n=&path.motions[j];if !matches!(n.kind,MotionKind::Cut{..}) || n.feed!=m.feed || n.path_id!=m.path_id || n.tool_axis!=m.tool_axis || n.end().unwrap().z!=start.z {break;}j+=1;}
  if j==i {out.motions.push(m.clone());at=m.end();i+=1;continue;}
  let mut chunk=vec![Motion::rapid(start)];chunk.extend_from_slice(&path.motions[i..j]);
  let (fitted,count)=crate::arcfit::fit_motions(&chunk,&fit)?;
  let mut prev=start;for n in fitted.iter().skip(1) {certify(prev,n,&drop,&mut work,runner)?;prev=n.end().unwrap();out.motions.push(n.clone());}
  arcs+=count;at=Some(prev);i=j;
 }
 if arcs>0 {out.report.counts.motions=out.motions.len() as u64;out.report.estimated_seconds=None;out.report.approximations.push(ApproximationLimit{kind:"certified-smoothing".into(),bound:Some(opts.tolerance),detail:Some(format!("{arcs} XY arcs; complete protected-mesh drop certificate, {work} bounds"))});}
 Ok(out)
}
