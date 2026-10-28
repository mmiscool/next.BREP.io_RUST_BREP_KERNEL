use super::*;
use crate::{CamError, MeshSnapshot, Runner, Tolerance};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Recognition {
    pub holes: Vec<Hole>,
    /// Faces not supported as complete rotational hole walls, with reasons.
    pub rejected: Vec<String>,
}

struct Section {
    source: RevolvedFace,
    faces: Vec<String>,
    axis: Point3,
    bottom: Point3,
    top: Point3,
    radius: f64,
    cone: bool,
}

/// Recognize complete inward cylindrical walls and adjoining coaxial steps.
/// The analytic carrier is exact; extent, caps and full-circle coverage use
/// the adapter's tessellation. Partial/interrupted walls are refused explicitly.
pub fn recognize(carriers: &[RevolvedFace], mesh: &MeshSnapshot, tolerance: Tolerance, runner: &mut Runner<'_>) -> Result<Recognition, CamError> {
    tolerance.validate()?;
    runner.token().check()?;
    let mut result=Recognition::default();
    let mut sections=Vec::new();
    let mut walls=Vec::new();
    for (i,c) in carriers.iter().enumerate() {
        runner.work(1,"recognize-hole",i as u64,carriers.len() as u64)?;
        if !c.origin.is_finite() || !c.axis.is_finite() || !c.radius0.is_finite() || !c.radius1.is_finite() || !c.height.is_finite() || c.radius0<0. || c.radius1<0. || c.height.abs()<=tolerance.distance_eps || (c.axis.dot(c.axis)-1.).abs()>tolerance.angle_eps {
            return Err(CamError::InvalidInput("invalid analytic hole carrier".into()));
        }
        let axis=if c.axis.z<0. {c.axis.scale(-1.)} else {c.axis};
        let tris:Vec<_>=mesh.triangles.iter().filter(|t| {
            mesh.faces.get(t.face as usize).map_or(false,|f|f.solid==c.solid && f.face==c.face && !f.protected_only)
        }).collect();
        if tris.is_empty() {result.rejected.push(format!("{}/{}: missing target face mesh",c.solid,c.face));continue;}
        let mut low=f64::INFINITY;let mut high=f64::NEG_INFINITY;let mut inward=0.;
        for t in &tris {
            runner.work(1,"recognize-wall",0,0)?;
            for p in t.vertices() {let z=p.sub(c.origin).dot(axis);low=low.min(z);high=high.max(z);}
            let p=t.a.add(t.b).add(t.c).scale(1./3.).sub(c.origin);
            let radial=p.sub(axis.scale(p.dot(axis)));
            inward+=t.normal.dot(radial);
        }
        if inward>=0. || high-low<=tolerance.distance_eps {
            result.rejected.push(format!("{}/{}: exterior, degenerate or incomplete circular wall",c.solid,c.face));continue;
        }
        walls.push((c,axis,tris,low,high));
    }
    // Imported periodic surfaces may be partitioned at a seam. Completeness
    // belongs to the hole wall, not to each topological face. Merge only the
    // same analytic carrier with the same trimmed axial interval; separate
    // bodies, different radii and staggered partial walls cannot fill a gap.
    let mut joined=vec![false;walls.len()];
    for seed in 0..walls.len() {
        if joined[seed] {continue;}
        let (c,axis,seed_tris,low,high)=&walls[seed];
        let mut tris=seed_tris.clone();let mut faces=vec![c.face.clone()];joined[seed]=true;
        for j in seed+1..walls.len() {
            runner.work(1,"hole-wall-partitions",0,0)?;
            if joined[j] {continue;}
            let (other,other_axis,other_tris,other_low,other_high)=&walls[j];
            if c.solid==other.solid && c.face!=other.face
                && c.origin.distance(other.origin)<=tolerance.point_eps
                && c.axis.distance(other.axis)<=tolerance.angle_eps
                && axis.distance(*other_axis)<=tolerance.angle_eps
                && (c.radius0-other.radius0).abs()<=tolerance.distance_eps
                && (c.radius1-other.radius1).abs()<=tolerance.distance_eps
                && (c.height-other.height).abs()<=tolerance.distance_eps
                && (low-other_low).abs()<=tolerance.point_eps
                && (high-other_high).abs()<=tolerance.point_eps {
                joined[j]=true;faces.push(other.face.clone());tris.extend(other_tris.iter().copied());
            }
        }
        if !full_circle(&tris,c.origin,*axis,(low+high)*0.5,tolerance,runner)? {
            for face in faces {result.rejected.push(format!("{}/{face}: exterior, degenerate or incomplete circular wall",c.solid));}
            continue;
        }
        sections.push(Section {source:(*c).clone(),faces,axis:*axis,bottom:c.origin.add(axis.scale(*low)),top:c.origin.add(axis.scale(*high)),radius:c.radius0.min(c.radius1),cone:(c.radius0-c.radius1).abs()>tolerance.distance_eps});
    }
    let mut used=vec![false;sections.len()];
    for seed in 0..sections.len() {
        if used[seed] || sections[seed].cone {continue;}
        let s=&sections[seed];
        let mut group=vec![seed];used[seed]=true;
        let mut bottom=s.bottom;let mut top=s.top;
        loop {
            let mut added=false;
            for (j,c) in sections.iter().enumerate() {
                runner.work(1,"group-hole",0,0)?;
                if used[j] || c.source.solid!=s.source.solid || c.axis.dot(s.axis)<1.-tolerance.angle_eps {continue;}
                let d=c.bottom.sub(s.bottom);
                if d.sub(s.axis.scale(d.dot(s.axis))).distance(Point3::ZERO)>tolerance.point_eps {continue;}
                if c.bottom.distance(top)<=tolerance.point_eps || c.top.distance(bottom)<=tolerance.point_eps {
                    if c.bottom.sub(bottom).dot(s.axis)<0. {bottom=c.bottom;}
                    if c.top.sub(top).dot(s.axis)>0. {top=c.top;}
                    used[j]=true;group.push(j);added=true;
                }
            }
            if !added {break;}
        }
        let radius=group.iter().filter(|&&j|!sections[j].cone).map(|&j|sections[j].radius).fold(f64::INFINITY,f64::min);
        let cones=group.iter().any(|&j|sections[j].cone && sections[j].top.distance(top)<=tolerance.point_eps);
        let larger=group.iter().any(|&j|!sections[j].cone && sections[j].radius>radius+tolerance.distance_eps);
        // Geometry alone cannot establish machining intent. Spot-face is kept
        // as an explicit mapping option, not guessed from a shallow counterbore.
        let kind=if cones {HoleKind::Countersink} else if larger {HoleKind::Counterbore} else {HoleKind::Plain};
        let recess=group.iter().map(|&j|&sections[j]).find(|c|c.top.distance(top)<=tolerance.point_eps && (c.cone || c.radius>radius+tolerance.distance_eps)).map(|c| {
            let src=&c.source;
            let t=top.sub(src.origin).dot(src.axis)/src.height;
            let diameter=2.*(src.radius0+(src.radius1-src.radius0)*t);
            HoleRecess{diameter,depth:c.top.sub(c.bottom).dot(c.axis),included_angle_deg:if c.cone {Some(2.*((src.radius1-src.radius0)/src.height).abs().atan().to_degrees())} else {None}}
        });
        let cap=|p:Point3| mesh.triangles.iter().any(|t| {
            mesh.faces.get(t.face as usize).map_or(false,|f|f.solid==s.source.solid) &&
                ray_triangle(p.sub(s.axis.scale(tolerance.point_eps*4.)),s.axis,t,tolerance).map_or(false,|d|(d-tolerance.point_eps*4.).abs()<=tolerance.point_eps)
        });
        let through=!cap(bottom);
        let machinable=s.axis.distance(Point3::Z)<=tolerance.angle_eps && !cap(top);
        result.holes.push(Hole {solid:s.source.solid.clone(),faces:group.iter().flat_map(|&j|sections[j].faces.iter().cloned()).collect(),top,axis:s.axis,depth:top.sub(bottom).dot(s.axis),diameter:2.*radius,through,kind,recess,machinable,thread_candidate:None});
    }
    for (i,s) in sections.iter().enumerate() {if !used[i] {result.rejected.push(format!("{}/{}: cone has no adjoining cylinder",s.source.solid,s.source.face));}}
    runner.token().check()?;
    Ok(result)
}

fn ray_triangle(p:Point3,d:Point3,t:&crate::Triangle,tol:Tolerance)->Option<f64> {
    let e=t.b.sub(t.a);let f=t.c.sub(t.a);let h=d.cross(f);let det=e.dot(h);
    if det.abs()<=tol.area_eps {return None;}
    let q=p.sub(t.a);let u=q.dot(h)/det;let k=q.cross(e);let v=d.dot(k)/det;
    if u < -tol.angle_eps || v < -tol.angle_eps || u+v>1.+tol.angle_eps {return None;}
    Some(f.dot(k)/det)
}

fn full_circle(tris:&[&crate::Triangle],origin:Point3,axis:Point3,z:f64,tol:Tolerance,runner:&mut Runner<'_>)->Result<bool,CamError> {
    let e=if axis.x.abs()<0.8 {Point3::new(1.,0.,0.)} else {Point3::new(0.,1.,0.)};
    let u=axis.cross(e).normalized().unwrap();let v=axis.cross(u);
    let tau=std::f64::consts::TAU;
    let mut intervals=Vec::new();
    for t in tris {
        runner.work(1,"hole-circle-coverage",0,0)?;
        let pts=t.vertices();let mut hits=Vec::new();
        for i in 0..3 {
            let a=pts[i].sub(origin);let b=pts[(i+1)%3].sub(origin);
            let da=a.dot(axis)-z;let db=b.dot(axis)-z;
            if (da<=0. && db>0.) || (db<=0. && da>0.) {
                let p=a.lerp(b,da/(da-db));hits.push(p.dot(v).atan2(p.dot(u)).rem_euclid(tau));
            }
        }
        if hits.len()!=2 {continue;}
        let a=hits[0].min(hits[1]);let b=hits[0].max(hits[1]);
        if b-a>std::f64::consts::PI {intervals.push((0.,a));intervals.push((b,tau));} else {intervals.push((a,b));}
    }
    intervals.sort_by(|a,b|a.0.total_cmp(&b.0));
    let mut end=0.;
    for (a,b) in intervals {if a>end+tol.angle_eps {return Ok(false);}end=end.max(b);}
    Ok(end>=tau-tol.angle_eps)
}
