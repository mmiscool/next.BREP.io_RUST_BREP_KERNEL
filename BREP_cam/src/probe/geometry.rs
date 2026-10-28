//! Contact patterns from picked machine-space geometry; no mesh guessing.
use crate::*;
fn error(s:&str)->CamError {CamError::InvalidInput(format!("probe geometry: {s}"))}
fn point(id:usize,p:Point3,n:Point3,face:TargetRef)->ProbePoint {ProbePoint {id:format!("p{id}"),face,position:p,normal:n,lower_tolerance:-0.01,upper_tolerance:0.01}}
pub fn opposed(center:Point3,axis:Point3,width:f64,inside:bool,faces:[TargetRef;2])->Result<Vec<ProbePoint>,CamError> {
    if !center.is_finite()||!width.is_finite()||width<=0. {return Err(error("invalid opposed feature"));}let u=axis.normalized().ok_or_else(||error("zero feature axis"))?;
    Ok([1.,-1.].iter().zip(faces).enumerate().map(|(i,(&sign,face))|point(i,center.add(u.scale(sign*width*0.5)),u.scale(if inside {-sign}else{sign}),face)).collect())
}
pub fn round(center:Point3,radius:f64,axis:Point3,inside:bool,faces:[TargetRef;4])->Result<Vec<ProbePoint>,CamError> {
    let u=axis.normalized().ok_or_else(||error("zero round axis"))?;if u.z.abs()>1e-9 {return Err(error("round probing requires a vertical bore/boss"));}let v=Point3::Z.cross(u);
    let mut a=opposed(center,u,2.*radius,inside,[faces[0].clone(),faces[1].clone()])?;let mut b=opposed(center,v,2.*radius,inside,[faces[2].clone(),faces[3].clone()])?;for (i,p) in b.iter_mut().enumerate(){p.id=format!("p{}",i+2);}a.extend(b);Ok(a)
}
pub fn rectangle(center:Point3,width:f64,height:f64,inside:bool,faces:[TargetRef;4])->Result<Vec<ProbePoint>,CamError> {let mut a=opposed(center,Point3::X,width,inside,[faces[0].clone(),faces[1].clone()])?;let mut b=opposed(center,Point3::Y,height,inside,[faces[2].clone(),faces[3].clone()])?;for (i,p) in b.iter_mut().enumerate(){p.id=format!("p{}",i+2);}a.extend(b);Ok(a)}
pub fn angle(center:Point3,normal:Point3,spacing:f64,face:TargetRef)->Result<Vec<ProbePoint>,CamError> {let n=normal.normalized().ok_or_else(||error("zero angle normal"))?;if n.z.abs()>1e-9||!spacing.is_finite()||spacing<=0. {return Err(error("angle probing requires a vertical face and positive spacing"));}let t=Point3::Z.cross(n);Ok(vec![point(0,center.sub(t.scale(spacing*0.5)),n,face.clone()),point(1,center.add(t.scale(spacing*0.5)),n,face)])}
pub fn validate_pattern(kind:ProbeKind,p:&[ProbePoint])->Result<(),CamError> {
    let count=match kind {ProbeKind::SingleSurface=>1,ProbeKind::OutsideCorner|ProbeKind::InsideCorner|ProbeKind::Web|ProbeKind::Pocket|ProbeKind::Angle=>2,ProbeKind::Bore|ProbeKind::Boss|ProbeKind::Rectangle=>4,ProbeKind::Inspection=>return Ok(())};
    if p.len()!=count {return Err(error("wrong number of picked probe contacts"));}
    let horizontal=|n:Point3|n.z.abs()<1e-6;
    match kind {
        ProbeKind::SingleSurface=>{if [p[0].normal.x.abs(),p[0].normal.y.abs(),p[0].normal.z.abs()].into_iter().fold(0.,f64::max)<1.-1e-6 {return Err(error("WCS surface must be X, Y or Z; use inspection for arbitrary normals"));}},
        ProbeKind::OutsideCorner|ProbeKind::InsideCorner=>{if !horizontal(p[0].normal)||!horizontal(p[1].normal)||p[0].normal.dot(p[1].normal).abs()>1e-6 {return Err(error("corner requires orthogonal vertical faces"));}let sign=if kind==ProbeKind::InsideCorner {1.}else{-1.};if p[1].position.sub(p[0].position).dot(p[0].normal)*sign<=0.||p[0].position.sub(p[1].position).dot(p[1].normal)*sign<=0. {return Err(error("corner picks disagree with inside/outside kind"));}},
        ProbeKind::Angle=>{if !horizontal(p[0].normal)||p[0].normal.distance(p[1].normal)>1e-6||p[0].position.distance(p[1].position)<1e-6||p[1].position.sub(p[0].position).dot(p[0].normal).abs()>1e-6|| (p[1].position.z-p[0].position.z).abs()>1e-6 {return Err(error("angle picks must span one vertical face at equal Z"));}},
        _=>{for pair in p.chunks_exact(2) {let d=pair[0].position.sub(pair[1].position);if !horizontal(pair[0].normal)||pair[0].normal.add(pair[1].normal).length()>1e-6||d.length()<1e-6||d.cross(pair[0].normal).length()>1e-6 {return Err(error("centre probing needs opposed face pairs"));}let inside=d.dot(pair[0].normal)<0.;if (matches!(kind,ProbeKind::Bore|ProbeKind::Pocket)&&!inside)||(matches!(kind,ProbeKind::Boss|ProbeKind::Web)&&inside) {return Err(error("opposed normals disagree with inside/outside kind"));}}
            if p.len()==4 {let a=p[0].position.add(p[1].position).scale(0.5);let b=p[2].position.add(p[3].position).scale(0.5);if a.distance(b)>1e-6||p[0].normal.dot(p[2].normal).abs()>1e-6 {return Err(error("centre pairs must share a centre and perpendicular axes"));}if matches!(kind,ProbeKind::Bore|ProbeKind::Boss)&&(p[0].position.distance(p[1].position)-p[2].position.distance(p[3].position)).abs()>1e-6 {return Err(error("bore/boss diameters disagree"));}}
        }
    }Ok(())
}
