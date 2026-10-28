//! Analytic capsule/triangle distances: no point-only clearance sampling.
use crate::*;
fn point_segment(p:Point3,a:Point3,b:Point3)->f64 {let d=b.sub(a);let q=d.length_squared();p.distance(if q==0. {a}else{a.add(d.scale((p.sub(a).dot(d)/q).clamp(0.,1.)))})}
fn point_triangle(p:Point3,t:[Point3;3])->f64 {if t[1].sub(t[0]).cross(t[2].sub(t[0])).length_squared()<1e-24 {(0..3).map(|i|point_segment(p,t[i],t[(i+1)%3])).fold(f64::INFINITY,f64::min)}else{crate::verify::point_triangle_distance_squared(p,t[0],t[1],t[2]).max(0.).sqrt()}}
fn segments(a:Point3,b:Point3,c:Point3,d:Point3)->f64 {
    let u=b.sub(a);let v=d.sub(c);let w=a.sub(c);let aa=u.dot(u);let bb=u.dot(v);let cc=v.dot(v);let dd=u.dot(w);let ee=v.dot(w);let det=aa*cc-bb*bb;
    let mut best=point_segment(a,c,d).min(point_segment(b,c,d)).min(point_segment(c,a,b)).min(point_segment(d,a,b));
    if det>f64::EPSILON*aa*cc {let s=(bb*ee-cc*dd)/det;let t=(aa*ee-bb*dd)/det;if (0.0..=1.0).contains(&s)&&(0.0..=1.0).contains(&t) {best=best.min(a.add(u.scale(s)).distance(c.add(v.scale(t))));}}best
}
fn segment_triangle(a:Point3,b:Point3,t:[Point3;3])->f64 {
    let mut best=point_triangle(a,t).min(point_triangle(b,t));for i in 0..3 {best=best.min(segments(a,b,t[i],t[(i+1)%3]));}
    let n=t[1].sub(t[0]).cross(t[2].sub(t[0]));let den=n.dot(b.sub(a));if den!=0. {let u=n.dot(t[0].sub(a))/den;if (0.0..=1.0).contains(&u) {best=best.min(point_triangle(a.lerp(b,u),t));}}best
}
fn triangles(a:[Point3;3],b:[Point3;3])->f64 {let mut best=f64::INFINITY;for i in 0..3 {best=best.min(segment_triangle(a[i],a[(i+1)%3],b)).min(segment_triangle(b[i],b[(i+1)%3],a));}best}
pub(crate) fn sweep(a:Point3,b:Point3,s:&ProbeSettings,mesh:&MeshSnapshot,tol:Tolerance,strict:bool,r:&mut Runner<'_>)->Result<(),CamError> {check(a,b,s,mesh,tol,strict,None,r)}
pub(crate) fn overtravel(a:Point3,b:Point3,s:&ProbeSettings,mesh:&MeshSnapshot,tol:Tolerance,nominal:&ProbePoint,r:&mut Runner<'_>)->Result<(),CamError> {check(a,b,s,mesh,tol,false,Some(&nominal.face),r)}
fn check(a:Point3,b:Point3,s:&ProbeSettings,mesh:&MeshSnapshot,tol:Tolerance,strict:bool,ignore:Option<&TargetRef>,r:&mut Runner<'_>)->Result<(),CamError> {
    let ca=a.add(Point3::Z.scale(s.stylus_radius));let cb=b.add(Point3::Z.scale(s.stylus_radius));
    let z0=(s.stylus_radius*s.stylus_radius-s.shank_radius*s.shank_radius).sqrt();
    let q=[ca.add(Point3::Z.scale(z0)),ca.add(Point3::Z.scale(s.shank_length)),cb.add(Point3::Z.scale(s.shank_length)),cb.add(Point3::Z.scale(z0))];
    let margin=if strict {tol.distance_eps}else{-tol.distance_eps};
    for t in &mesh.triangles {
        r.work(1,"probe-sweep",t.id as u64,mesh.triangle_count() as u64)?;let f=&mesh.faces[t.face as usize];let skip=ignore.is_some_and(|p|!f.protected_only&&p.solid==f.solid&&p.face.as_ref()==Some(&f.face));
        if !skip && segment_triangle(ca,cb,t.vertices())<s.stylus_radius+margin {return Err(CamError::InvalidInput(format!("probe stylus sweep touches protected mesh triangle {}",t.id)));}
        let d=triangles([q[0],q[1],q[2]],t.vertices()).min(triangles([q[0],q[2],q[3]],t.vertices()));
        if d<s.shank_radius+tol.distance_eps {return Err(CamError::InvalidInput(format!("probe shank sweep touches protected mesh triangle {}",t.id)));}
    }Ok(())
}
pub(crate) fn validate_pick(p:&ProbePoint,op:&Operation,mesh:&MeshSnapshot,tol:Tolerance)->Result<(),CamError> {
    if p.face.face.is_none() || !op.targets.target_solids.contains(&p.face.solid) {return Err(CamError::InvalidInput("probe pick must name a selected target face".into()));}
    let mut found=false;
    for t in &mesh.triangles {let f=&mesh.faces[t.face as usize];if f.protected_only||f.solid!=p.face.solid||p.face.face.as_ref()!=Some(&f.face) {continue;}if point_triangle(p.position,t.vertices())<=tol.point_eps {found=true;}}
    if !found {return Err(CamError::InvalidInput(format!("probe nominal {} is not on its selected face",p.id)));}Ok(())
}
