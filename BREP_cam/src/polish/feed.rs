//! Exact arclength feed windows. Both turn senses are conservative corner
//! triggers: the linked stream does not encode which side contains stock.
use crate::{CamError,Runner};
use crate::model::{Feed,FeedOptimizationParams,Motion,MotionKind,Point3,Toolpath};
use super::{invalid,validate_path,MAX_MOTIONS};

pub(crate) fn is_cut(m:&Motion)->bool {matches!(m.kind,MotionKind::Cut{..}|MotionKind::ArcCw{..}|MotionKind::ArcCcw{..})}
struct Edge {from:Point3, length:f64, radius:Option<f64>, start:Point3, end:Point3}
fn edge(from:Point3,m:&Motion)->Result<Edge,CamError> {
 let to=m.end().ok_or_else(||invalid("cut has no endpoint"))?;
 match m.kind {
  MotionKind::Cut{..}=>{let d=to.sub(from);let tangent=d.normalized().unwrap_or(Point3::ZERO);Ok(Edge{from,length:from.distance(to),radius:None,start:tangent,end:tangent})},
  MotionKind::ArcCw{center,plane,pitch,..}|MotionKind::ArcCcw{center,plane,pitch,..}=>{
   if pitch!=0. {return Err(invalid("feed optimisation requires planar arcs"));}
   let cw=matches!(m.kind,MotionKind::ArcCw{..});
   let (s,r)=crate::arcfit::arc_sweep(from,to,center,plane,cw).ok_or_else(||invalid("zero radius arc"))?;
   let (u,v,w)=crate::arcfit::plane_coords(from,plane);let (x,y,z)=crate::arcfit::plane_coords(to,plane);let (cu,cv,cw0)=crate::arcfit::plane_coords(center,plane);
   let eps=1e-9*r.max(1.);
   if ((u-cu).hypot(v-cv)-(x-cu).hypot(y-cv)).abs()>eps || (w-z).abs()>eps || (w-cw0).abs()>eps {return Err(invalid("inconsistent arc geometry"));}
   let sign=if cw {-1.}else{1.};
   let tangent=|a:f64,b:f64|crate::arcfit::from_plane_coords(-sign*(b-cv)/r,sign*(a-cu)/r,0.,plane);
   Ok(Edge{from,length:s*r,radius:Some(r),start:tangent(u,v),end:tangent(x,y)})
  },_=>Err(invalid("not a cut"))
 }
}
fn point(e:&Edge,m:&Motion,t:f64)->Point3 {
 if t>=1. {return m.end().unwrap();}
 if matches!(m.kind,MotionKind::Cut{..}) {e.from.lerp(m.end().unwrap(),t)}else{crate::arcfit::arc_point(e.from,&m.kind,t).unwrap()}
}
fn with_end(m:&Motion,to:Point3,slow:bool,ratio:f64)->Motion {
 let mut out=m.clone();match &mut out.kind {MotionKind::Cut{to:p}|MotionKind::ArcCw{to:p,..}|MotionKind::ArcCcw{to:p,..}=>*p=to,_=>{}}
 if slow {out.feed=Feed::Rate(m.feed.value().unwrap()*ratio);}out
}
/// Pure transactional pass. Existing per-motion rates are the base rates;
/// repeated invocation intentionally applies the requested ratio again.
pub fn optimize(path:&Toolpath,opts:&FeedOptimizationParams,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
 runner.token().check()?;
 if !opts.enabled {return Ok(path.clone());}
 if ![opts.max_angle_deg,opts.max_radius,opts.reduced_feed_ratio,opts.distance].iter().all(|x|x.is_finite()) || !(0.0..=180.0).contains(&opts.max_angle_deg) || opts.max_radius<0. || opts.reduced_feed_ratio<=0. || opts.reduced_feed_ratio>1. || opts.distance<0. {return Err(invalid("invalid feed optimisation parameters"));}
 validate_path(path)?;
 let mut out=path.clone();out.motions.clear();let mut at=None;let mut i=0;
 while i<path.motions.len() {
  runner.token().check()?;let m=&path.motions[i];
  if !is_cut(m) {out.motions.push(m.clone());if let Some(p)=m.end(){at=Some(p);}i+=1;continue;}
  let mut j=i+1;while j<path.motions.len() && is_cut(&path.motions[j]) && path.motions[j].path_id==m.path_id && path.motions[j].tool_axis==m.tool_axis {j+=1;}
  let run=&path.motions[i..j];let mut edges=Vec::with_capacity(run.len());let mut starts=vec![0.];let mut pos=at.ok_or_else(||invalid("linked cut requires a known start"))?;
  for m in run {runner.token().check()?;if m.feed.value().map_or(true,|f|f<=0. || f*opts.reduced_feed_ratio<=0.) {return Err(invalid("cut requires a positive finite rate"));}let e=edge(pos,m)?;let length=starts.last().unwrap()+e.length;if !length.is_finite(){return Err(invalid("path length overflow"));}starts.push(length);pos=m.end().unwrap();edges.push(e);}
  let total=*starts.last().unwrap();let mut windows:Vec<(f64,f64)>=Vec::new();
  let mut previous:Option<usize>=None;
  for k in 0..edges.len() {
   if edges[k].radius.map_or(false,|r|r<opts.max_radius) {windows.push(((starts[k]-opts.distance).max(0.),(starts[k+1]+opts.distance).min(total)));}
   if edges[k].length>0. {
    if previous.map_or(false,|p|edges[p].end.dot(edges[k].start).clamp(-1.,1.).acos().to_degrees()>opts.max_angle_deg) {windows.push(((starts[k]-opts.distance).max(0.),(starts[k]+opts.distance).min(total)));}
    previous=Some(k);
   }
  }
  windows.sort_by(|a,b|a.0.total_cmp(&b.0));let mut merged:Vec<(f64,f64)>=Vec::new();
  for (a,b) in windows {if a>=b {continue;}if let Some(last)=merged.last_mut(){if a<=last.1 {last.1=last.1.max(b);continue;}}merged.push((a,b));}
  let mut w=0;
  for (k,(e,m)) in edges.iter().zip(run).enumerate() {
   runner.token().check()?;let a=starts[k];let b=starts[k+1];
   if e.length==0. {out.motions.push(m.clone());continue;}
   while w<merged.len() && merged[w].1<=a {w+=1;}
   let mut cuts=vec![a];let mut q=w;
   while q<merged.len() && merged[q].0<b {for x in [merged[q].0,merged[q].1] {if x>a && x<b {cuts.push(x);}}q+=1;}
   cuts.push(b);cuts.sort_by(f64::total_cmp);cuts.dedup();
   let mut q=w;
   for interval in cuts.windows(2) {let mid=interval[0]+(interval[1]-interval[0])*0.5;while q<merged.len() && merged[q].1<=mid {q+=1;}let slow=q<merged.len() && merged[q].0<=mid && mid<merged[q].1;let to=point(e,m,(interval[1]-a)/e.length);out.motions.push(with_end(m,to,slow,opts.reduced_feed_ratio));if out.motions.len()>MAX_MOTIONS {return Err(invalid("feed output work budget exceeded"));}}
  }
  at=Some(pos);i=j;
 }
 if out.motions.len()>MAX_MOTIONS {return Err(invalid("feed output work budget exceeded"));}
 if out.motions!=path.motions {out.report.counts.motions=out.motions.len() as u64;out.report.estimated_seconds=None;}
 Ok(out)
}
