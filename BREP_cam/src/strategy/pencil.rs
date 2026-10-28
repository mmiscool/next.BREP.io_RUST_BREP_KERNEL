//! Pencil traces at concave mesh creases with two finite ball/facet contacts.
//! Fresh offset-plane intersection; no external CAM implementation is used.
use crate::*;
use crate::drop::{finish::{FinishContext,invalid,polyline,MAX_SOURCE_SEGMENTS},DropCutter};
use std::collections::BTreeMap;

type VertexKey=[u64;3];
fn key(p:Point3)->VertexKey {[p.x,p.y,p.z].map(|x|if x==0. {0} else {x.to_bits()})}
struct Trace {a:Point3,b:Point3,first:u32,second:u32,transverse:Point3}
/// A ball centre lies radius units along the outward facet normal. This 3D
/// predicate works for vertical walls as well as height-field facets.
fn clip_facet(t:&Triangle,a:Point3,b:Point3,r:f64,lo:&mut f64,hi:&mut f64)->bool {
    let n=t.normal;
    let contact=|p:Point3| {let center=p.add(Point3::Z.scale(r));center.sub(n.scale(n.dot(center.sub(t.a))))};
    let pa=contact(a);let pb=contact(b);
    for (u,v) in [(t.a,t.b),(t.b,t.c),(t.c,t.a)] {
        let side=|p:Point3|v.sub(u).cross(p.sub(u)).dot(n);
        let x=side(pa);let y=side(pb);let delta=y-x;
        if delta==0. {if x<0. {return false;}} else if delta>0. {*lo=lo.max(-x/delta);} else {*hi=hi.min(-x/delta);}
    }
    *lo<=*hi
}
fn traces(ctx:&FinishContext<'_>,p:&PencilParams,runner:&mut Runner<'_>)->Result<Vec<Trace>,CamError> {
    if ctx.cutter.kind()!=CutterKind::Ball {return Err(invalid("pencil bitangent tracing currently requires a ball cutter"));}
    if !(0.0..180.0).contains(&p.bitangency_angle_deg) || p.extra_passes>1000 {return Err(invalid("invalid pencil bitangency angle or pass budget"));}
    if ctx.operation.stock_to_leave.radial!=0. || ctx.operation.stock_to_leave.axial!=0. {return Err(invalid("pencil bitangent tracing requires zero stock allowance"));}
    let mut edges:BTreeMap<(VertexKey,VertexKey),Vec<(u32,Point3,Point3)>>=BTreeMap::new();
    for t in &ctx.mesh.triangles {
        let f=&ctx.mesh.faces[t.face as usize];
        if f.protected_only || !ctx.operation.targets.target_solids.contains(&f.solid) {continue;}
        for (a,b) in [(t.a,t.b),(t.b,t.c),(t.c,t.a)] {let (ka,kb)=(key(a),key(b));let k=if ka<kb {(ka,kb)} else {(kb,ka)};edges.entry(k).or_default().push((t.id,a,b));}
        runner.work(1,"pencil-adjacency",t.id as u64,ctx.mesh.triangle_count() as u64)?;
    }
    let r=ctx.cutter.radius();let mut out=Vec::new();
    for entries in edges.values() {
        if entries.len()!=2 {continue;}
        let (id,a,b)=entries[0];let other=entries[1].0;let t1=ctx.mesh.triangle(id);let t2=ctx.mesh.triangle(other);
        let (n1,n2)=(t1.normal,t2.normal);
        if n1.z < -ctx.tolerance.angle_eps || n2.z < -ctx.tolerance.angle_eps {continue;}
        let cosine=n1.dot(n2).clamp(-1.,1.);let angle=cosine.acos().to_degrees();
        if angle<=p.bitangency_angle_deg.max(ctx.tolerance.angle_eps.to_degrees()) || 1.+cosine<=ctx.tolerance.angle_eps {continue;}
        // At a concave crease the opposite facet extends into the outward
        // half-space; convex ridges cannot support a finite two-face ball.
        if t2.vertices().iter().map(|v|n1.dot(v.sub(a))).fold(f64::NEG_INFINITY,f64::max)<=ctx.tolerance.distance_eps {continue;}
        let d=b.sub(a);let len=d.x.hypot(d.y);if len<=ctx.tolerance.sample_eps {continue;}
        let transverse=Point3::new(-d.y/len,d.x/len,0.);
        // A bounded outward clearance above the adaptive sampling floor avoids the drop solver's first
        // contact at the top edge of a wall exactly tangent to the shank.
        let clearance=(ctx.operation.tolerance/32.).max(ctx.tolerance.sample_eps*2.+ctx.tolerance.distance_eps*4.);
        if n1.add(n2).scale(clearance/(1.+cosine)).length()>ctx.operation.tolerance*0.125 {return Err(invalid("pencil numeric clearance exceeds contact tolerance"));}
        let offset=n1.add(n2).scale((r+clearance)/(1.+cosine)).sub(Point3::Z.scale(r));
        let start=a.add(offset);let end=b.add(offset);
        let (mut lo,mut hi)=(0.,1.);
        if !clip_facet(t1,start,end,r,&mut lo,&mut hi)||!clip_facet(t2,start,end,r,&mut lo,&mut hi)||hi-lo<=ctx.tolerance.sample_eps/len {continue;}
        out.push(Trace {a:start.lerp(end,lo),b:start.lerp(end,hi),first:id,second:other,transverse});
        if out.len()>MAX_SOURCE_SEGMENTS {return Err(invalid("pencil trace work budget exceeded"));}
        runner.work(1,"pencil-bitangents",out.len() as u64,edges.len() as u64)?;
    }
    if out.is_empty() {return Err(invalid("no finite concave bitangent pencil traces satisfy the angle threshold"));}Ok(out)
}
pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Pencil(p)=>p,_=>return Err(invalid("pencil generator requires Pencil parameters"))};
    let ctx=FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance};ctx.validate()?;
    let raw=traces(&ctx,p,runner)?;let drop=DropCutter::new(mesh,cutter,index,tolerance)?;
    let mut source=Vec::new();let mut count=WorkCounts::default();let mut traces=Vec::new();let mut clipped=false;
    // A third wall bounds a two-plane crease near a pocket corner. Keep only
    // reachable portions at the bitangent height; whole-path certification
    // below still rejects any missed obstruction between sampling points.
    for trace in raw {
        let n=(trace.a.distance(trace.b)/operation.tolerance.min(cutter.radius()*0.25)).ceil().max(1.);
        if n>MAX_SOURCE_SEGMENTS as f64 || count.samples.saturating_add(n as u64+1)>MAX_SOURCE_SEGMENTS as u64 {return Err(invalid("pencil clearance sampling budget exceeded"));}
        let mut start=None;let mut last=None;
        for i in 0..=n as usize {
            let point=trace.a.lerp(trace.b,i as f64/n);
            let hit=drop.point(point.x,point.y,ctx.sampling().floor_z,runner)?;
            count.samples+=1;count.candidates+=hit.candidate_count;count.contacts+=hit.contact_count;
            let clear=hit.point.contact.is_some() && (hit.point.z-point.z).abs()<=operation.tolerance*0.25;
            if clear {if start.is_none(){start=Some(point);}last=Some(point);}else{clipped=true;}
            if !clear || i==n as usize {
                if let (Some(a),Some(b))=(start,last) {if a.distance_xy(b)>tolerance.sample_eps {traces.push(Trace{a,b,first:trace.first,second:trace.second,transverse:trace.transverse});}}
                start=None;last=None;
            }
        }
    }
    if traces.is_empty() {return Err(invalid("all pencil bitangent traces are blocked by protected geometry"));}
    for trace in &traces {
        for t in [0.,0.5,1.] {
            let a=trace.a.lerp(trace.b,t);let center=a.add(Point3::Z.scale(cutter.radius()));
            for id in [trace.first,trace.second] {let facet=mesh.triangle(id);if (facet.normal.dot(center.sub(facet.a))-cutter.radius()).abs()>(operation.tolerance/32.).max(tolerance.sample_eps*2.+tolerance.distance_eps*4.)+tolerance.distance_eps*4. {return Err(invalid("pencil offset planes lost bitangency"));}}
            let result=drop.point(a.x,a.y,ctx.sampling().floor_z,runner)?;
            count.samples+=1;count.candidates+=result.candidate_count;count.contacts+=result.contact_count;
            if (result.point.z-a.z).abs()>operation.tolerance {return Err(invalid("pencil bitangent is blocked by protected geometry"));}
        }
        for extra in 0..=p.extra_passes {
            let offset=if extra==0 {0.} else {((extra+1)/2) as f64*cutter.radius()*0.5*if extra%2==1 {1.} else {-1.}};
            let a=trace.a.add(trace.transverse.scale(offset));let b=trace.b.add(trace.transverse.scale(offset));
            source.push(polyline(&[Point2::new(a.x,a.y),Point2::new(b.x,b.y)])?);
        }
    }
    let mut result=ctx.generate(&p.containment,&source,runner)?;
    // Central traces must still be bitangent after whole-mesh certification.
    // Each source can clip into multiple groups: locate central segments by XY
    // rather than assuming a path-id mapping survives region clipping.
    for motion in &result.motions {
        if let MotionKind::Cut {to}=motion.kind {
            for trace in &traces {
                let d=trace.b.sub(trace.a);let len=d.x.hypot(d.y);
                let lateral=((to.x-trace.a.x)*d.y-(to.y-trace.a.y)*d.x).abs()/len;
                if lateral<=tolerance.point_eps {
                    let along=((to.x-trace.a.x)*d.x+(to.y-trace.a.y)*d.y)/(len*len);
                    if along < -tolerance.point_eps/len || along>1.+tolerance.point_eps/len {continue;}
                    let expected=trace.a.z+d.z*along;
                    if (to.z-expected).abs()>operation.tolerance+tolerance.distance_eps {return Err(invalid("pencil path lost its two-surface contact"));}
                }
            }
        }
    }
    result.report.approximations.push(ApproximationLimit{kind:"pencil-facet-clearance".into(),bound:Some((operation.tolerance/32.).max(tolerance.sample_eps*2.+tolerance.distance_eps*4.)),detail:Some("Outward normal clearance from each contact plane prevents ambiguous first contact at a vertical wall top; bounded inside operation tolerance".into())});
    if clipped {result.report.warn(Diagnostic::new("pencil-trimmed-contact","Two-facet traces were clipped where additional protected surfaces prevent bitangent contact"));}
    result.report.counts.samples+=count.samples;result.report.counts.candidates+=count.candidates;result.report.counts.contacts+=count.contacts;
    if p.extra_passes>0 {result.report.approximations.push(ApproximationLimit {kind:"pencil-extra-pass-spacing".into(),bound:Some(cutter.radius()*0.5),detail:Some("Extra passes alternate sides at half-tool-radius increments; current PencilParams has no spacing field".into())});}
    Ok(result)
}
