//! Iso-parametric finishing. Cross-parameter distances are measured on the
//! surface, using the maximum metric across each pass. On a varying metric
//! surface the requested stepover is an upper bound, not a planar spacing.
use crate::*;
use crate::drop::{finish::{FinishContext,invalid,MAX_SOURCE_SEGMENTS},path::Span};

#[derive(Clone,Debug)]
pub struct UvDomain {pub u:[f64;2],pub v:[f64;2],pub trims:Vec<Vec<Point2>>}
#[derive(Clone,Copy,Debug)]
pub struct SurfaceSample {pub point:Point3,pub normal:Point3}
/// Implemented by the kernel adapter; every value is in machine coordinates.
/// Trims are closed UV polygons with even/odd fill (including holes). Empty
/// trims denote the full rectangular parameter domain.
pub trait ParametricFace {
    fn target(&self)->TargetRef;
    fn domain(&self)->UvDomain;
    fn sample(&self,u:f64,v:f64)->Result<SurfaceSample,CamError>;
}
fn sample(face:&dyn ParametricFace,a:f64,b:f64,along:bool,runner:&mut Runner<'_>)->Result<Point3,CamError> {
    runner.work(1,"flow-surface",0,0)?;
    let s=if along {face.sample(a,b)?}else{face.sample(b,a)?};
    if !s.point.is_finite() || !s.normal.is_finite() || s.normal.normalized().is_none() {return Err(invalid("flow sampler returned invalid point or normal"));}Ok(s.point)
}
fn metric_table(face:&dyn ParametricFace,along:bool,a:[f64;2],b:[f64;2],n:usize,runner:&mut Runner<'_>)->Result<Vec<f64>,CamError> {
    let mut lengths=vec![0.;n+1];
    // Convergence in cross parameter is checked below. Seventeen stations
    // measure the maximum cross-pass metric, including both endpoints.
    for j in 0..=16 {
        let u=a[0]+(a[1]-a[0])*j as f64/16.;let mut previous=sample(face,u,b[0],along,runner)?;
        for k in 1..=n {let p=sample(face,u,b[0]+(b[1]-b[0])*k as f64/n as f64,along,runner)?;lengths[k]=f64::max(lengths[k],p.distance(previous));previous=p;}
    }
    for k in 1..=n {lengths[k]+=lengths[k-1];}Ok(lengths)
}
fn trimmed_intervals(domain:&UvDomain,along:bool,cross:f64)->Vec<[f64;2]> {
    let a=if along {domain.u}else{domain.v};
    if domain.trims.is_empty() {return vec![a];}
    let mut hits=Vec::new();
    for l in &domain.trims {for (p,q) in l.iter().zip(l.iter().cycle().skip(1)).take(l.len()) {
        let (x,y,xx,yy)=if along {(p.x,p.y,q.x,q.y)}else{(p.y,p.x,q.y,q.x)};
        if (y>cross)!=(yy>cross) {hits.push(x+(xx-x)*(cross-y)/(yy-y));}
    }}
    hits.sort_by(f64::total_cmp);
    hits.chunks_exact(2).filter_map(|p|{let lo=p[0].max(a[0]);let hi=p[1].min(a[1]);(hi>lo).then_some([lo,hi])}).collect()
}
fn chord(face:&dyn ParametricFace,along:bool,cross:f64,a:f64,b:f64,error:f64,depth:usize,out:&mut Vec<Span>,runner:&mut Runner<'_>)->Result<(),CamError> {
    let p=sample(face,a,cross,along,runner)?;let q=sample(face,b,cross,along,runner)?;
    let mut deviation:f64=0.;
    for t in [0.25,0.5,0.75] {deviation=deviation.max(sample(face,a+(b-a)*t,cross,along,runner)?.distance(p.lerp(q,t)));}
    if deviation>error {
        if depth>=24 {return Err(invalid("flow surface chord refinement exhausted"));}
        let mid=(a+b)*0.5;chord(face,along,cross,a,mid,error,depth+1,out,runner)?;chord(face,along,cross,mid,b,error,depth+1,out,runner)?;
    }else if p.distance_xy(q)>1e-10 {out.push(Span::Line {start:p,end:q});}
    if out.len()>MAX_SOURCE_SEGMENTS {return Err(invalid("flow source work budget exhausted"));}Ok(())
}
pub fn source_paths(face:&dyn ParametricFace,p:&FlowParams,error:f64,runner:&mut Runner<'_>)->Result<Vec<Vec<Span>>,CamError> {
    p.stepover.validate().map_err(CamError::InvalidInput)?;
    if !error.is_finite() || error<=0. {return Err(invalid("flow requires positive chord tolerance"));}
    let d=face.domain();
    if [d.u,d.v].iter().any(|r|!r[0].is_finite()||!r[1].is_finite()||r[1]<=r[0]) || d.trims.iter().any(|l|l.len()<3||l.iter().any(|p|!p.is_finite())) {return Err(invalid("flow needs a finite trimmed parameter domain"));}
    let (a,b)=if p.along_u {(d.u,d.v)}else{(d.v,d.u)};
    let mut n=32;let mut table=metric_table(face,p.along_u,a,b,n,runner)?;
    loop {
        n*=2;let next=metric_table(face,p.along_u,a,b,n,runner)?;
        let change=(next[n]-table[table.len()-1]).abs();table=next;
        if change<=error*0.125 {break;}
        if n>=2048 {return Err(invalid("flow surface metric failed to converge"));}
    }
    let length=table[n];if length<=error*1e-6 {return Err(invalid("flow cross-parameter direction is degenerate"));}
    let count=(length/p.stepover.distance).ceil() as usize;
    if count>MAX_SOURCE_SEGMENTS {return Err(invalid("flow pass budget exhausted"));}
    let mut paths=Vec::new();let mut total=0;
    for pass in 0..=count {
        let distance=(pass as f64*p.stepover.distance).min(length);
        let k=table.partition_point(|v|*v<distance).clamp(1,n);
        let t=if table[k]>table[k-1] {(distance-table[k-1])/(table[k]-table[k-1])}else{0.};
        let mut cross=b[0]+(b[1]-b[0])*(k as f64-1.+t)/n as f64;
        // Half-open polygon scans use an interior representative at the final
        // trim edge; the displacement is far below the stated chord tolerance.
        if cross==b[1] && !d.trims.is_empty() {cross-=1e-12*(b[1]-b[0]);}
        for range in trimmed_intervals(&d,p.along_u,cross) {
            let mut path=Vec::new();
            // Seed intervals prevent an entire periodic curve aliasing a chord.
            for i in 0..16 {let lo=range[0]+(range[1]-range[0])*i as f64/16.;let hi=range[0]+(range[1]-range[0])*(i+1) as f64/16.;chord(face,p.along_u,cross,lo,hi,error,0,&mut path,runner)?;}
            total+=path.len();if total>MAX_SOURCE_SEGMENTS {return Err(invalid("flow source budget exhausted"));}
            if !path.is_empty() {paths.push(path);}
        }
    }
    if paths.is_empty() {return Err(invalid("flow face has no reachable XY passes"));}Ok(paths)
}
pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>,faces:&[&dyn ParametricFace])->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Flow(p)=>p,_=>return Err(invalid("flow requires Flow parameters"))};
    let ctx=FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance};ctx.validate()?;
    if faces.is_empty() {return Err(invalid("flow requires selected face samplers"));}
    let selected=|r:&TargetRef|operation.targets.target_solids.contains(&r.solid) && (operation.targets.drive_faces.is_empty() || operation.targets.drive_faces.iter().any(|s|s.solid==r.solid && (s.face.is_none()||s.face==r.face)));
    for target in &operation.targets.drive_faces {if !faces.iter().any(|f|{let r=f.target();r.solid==target.solid && (target.face.is_none()||r.face==target.face)}) {return Err(invalid("flow selected face has no sampler"));}}
    let mut paths=Vec::new();
    for face in faces {if !selected(&face.target()) || mesh.resolve_faces(&face.target()).is_empty() {return Err(invalid("flow sampler is not a selected mesh face"));}paths.extend(source_paths(*face,p,operation.tolerance*0.25,runner)?);}
    let mut out=ctx.generate(&p.containment,&paths,runner)?;
    out.report.approximations.push(ApproximationLimit {kind:"flow-surface-metric".into(),bound:Some(operation.tolerance*0.25),detail:Some("Adaptive chordal UV passes; cross distance converged on 17 stations. Varying metrics use maximum sampled spacing; final interval may be shorter. Mesh drop projection can increase surface pass separation.".into())});Ok(out)
}
