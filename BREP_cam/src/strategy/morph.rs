//! Morph between two boundary polylines with normalized arc-length pairing.
//! The caller supplies consistently oriented curves. Every intermediate curve
//! is dropped onto the CL surface; surface bridge lengths determine spacing.
use crate::*;
use crate::drop::{DropCutter,finish::{FinishContext,invalid,polyline,MAX_SOURCE_SEGMENTS}};
fn lengths(curve:&[Point2])->Result<Vec<f64>,CamError> {
    if curve.len()<2 || curve.iter().any(|p|!p.is_finite()) {return Err(invalid("morph requires two finite boundary curves"));}
    let mut s=vec![0.];for p in curve.windows(2) {let d=(p[1].x-p[0].x).hypot(p[1].y-p[0].y);if d<=1e-12 {return Err(invalid("morph boundary contains a degenerate segment"));}s.push(s[s.len()-1]+d);}
    let total=s[s.len()-1];for v in &mut s {*v/=total;}Ok(s)
}
fn at(curve:&[Point2],s:&[f64],t:f64)->Point2 {
    let k=s.partition_point(|x|*x<t).clamp(1,s.len()-1);let f=(t-s[k-1])/(s[k]-s[k-1]);
    Point2::new(curve[k-1].x+(curve[k].x-curve[k-1].x)*f,curve[k-1].y+(curve[k].y-curve[k-1].y)*f)
}
pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Morph(p)=>p,_=>return Err(invalid("morph requires Morph parameters"))};
    let ctx=FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance};ctx.validate()?;
    let first=lengths(&p.first_curve)?;let second=lengths(&p.second_curve)?;
    let mut stations=first.clone();stations.extend(&second);stations.extend((0..=32).map(|i|i as f64/32.));stations.sort_by(f64::total_cmp);stations.dedup();
    if stations.len()>2048 {return Err(invalid("morph boundary station budget exhausted"));}
    let pairs:Vec<_>=stations.iter().map(|t|(at(&p.first_curve,&first,*t),at(&p.second_curve,&second,*t))).collect();
    let point=|(a,b):(Point2,Point2),t:f64|Point2::new(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t);
    let drop=DropCutter::new(mesh,cutter,index,tolerance)?;
    // Resolve the cross-pass metric on the CL surface, not its XY shadow.
    // Refine until the maximum bridge length converges within tolerance/8.
    let mut n=16;let mut previous=None;let metric=loop {
        let mut metric=vec![0.;n+1];
        for &pair in &pairs {
            let a=point(pair,0.);let mut prev=drop.point(a.x,a.y,ctx.sampling().floor_z,runner)?.point.position();
            for k in 1..=n {let xy=point(pair,k as f64/n as f64);let q=drop.point(xy.x,xy.y,ctx.sampling().floor_z,runner)?.point.position();metric[k]=f64::max(metric[k],prev.distance(q));prev=q;}
        }
        for k in 1..=n {metric[k]+=metric[k-1];}
        if previous.map_or(false,|old:f64|(old-metric[n]).abs()<=operation.tolerance*0.125) {break metric;}
        if n>=512 {return Err(invalid("morph CL bridge metric failed to converge"));}previous=Some(metric[n]);n*=2;
    };
    let total=metric[n];if total<=tolerance.distance_eps {return Err(invalid("morph boundaries coincide"));}
    let count=(total/p.stepover.distance).ceil() as usize;
    if count.saturating_mul(pairs.len())>MAX_SOURCE_SEGMENTS {return Err(invalid("morph pass work budget exhausted"));}
    let mut paths=Vec::new();
    for i in 0..=count {
        let distance=(i as f64*p.stepover.distance).min(total);let k=metric.partition_point(|v|*v<distance).clamp(1,n);
        let f=if metric[k]>metric[k-1] {(distance-metric[k-1])/(metric[k]-metric[k-1])}else{0.};let t=(k as f64-1.+f)/n as f64;
        let points:Vec<_>=pairs.iter().map(|&pair|point(pair,t)).collect();paths.push(polyline(&points)?);
    }
    let mut out=ctx.generate(&p.containment,&paths,runner)?;
    out.report.approximations.push(ApproximationLimit {kind:"morph-cl-metric".into(),bound:Some(operation.tolerance*0.125),detail:Some("Normalized boundary arc length; maximum sampled CL bridge metric, converged in cross direction. Spacing may shrink where boundaries approach; caller supplies consistent curve orientation.".into())});Ok(out)
}
