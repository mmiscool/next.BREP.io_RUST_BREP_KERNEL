//! Full rounds of convex prisms with a tangent line/circle outline.
//! Circle samples propose a carrier only: reconstructing every homogeneous
//! control and knot is the admission test. The inset keeps the line pieces
//! and shrinks each arc radius; half torus strips replace the arc walls.
use crate::{BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};
use std::f64::consts::{FRAC_PI_2, TAU};

#[derive(Clone)]
enum Piece {
    Line { a: Vec3, b: Vec3 },
    Arc { centre: Vec3, radial: Vec3, radius: f64, angle: f64 },
}
impl Piece {
    fn curve(&self, normal: Vec3) -> Result<NurbsCurve, String> {
        match *self {
            Self::Line { a,b } => crate::make_line(a,b),
            Self::Arc { centre,radial,radius,angle } => crate::make_arc(centre,radial,normal.cross(radial),radius,0.0,angle),
        }
    }
    fn tangents(&self, normal: Vec3) -> Option<(Vec3,Vec3)> {
        match *self {
            Self::Line { a,b } => {let t=b.sub(a).normalized().ok()?;Some((t,t))},
            Self::Arc { radial,angle,.. } => {
                let y=normal.cross(radial);
                Some((y,y.scale(angle.cos()).sub(radial.scale(angle.sin()))))
            }
        }
    }
    fn inset(&self, normal: Vec3, radius: f64) -> Option<Self> {
        let mid=normal.scale(-radius);
        Some(match *self {
            Self::Line { a,b } => {
                let inward=normal.cross(b.sub(a).normalized().ok()?).scale(radius);
                Self::Line { a:a.add(mid).add(inward),b:b.add(mid).add(inward) }
            },
            Self::Arc { centre,radial,radius:r,angle } => {
                if r <= radius {return None;}
                Self::Arc {centre:centre.add(mid),radial,radius:r-radius,angle}
            }
        })
    }
}

fn same_net(a:&NurbsCurve,b:&NurbsCurve,translation:Vec3,band:f64)->bool {
    if a.degree!=b.degree || a.knots.len()!=b.knots.len() || a.control_points.len()!=b.control_points.len() {return false;}
    let (Ok([a0,a1]),Ok([b0,b1]))=(a.domain(),b.domain()) else{return false;};
    if a.knots.iter().zip(&b.knots).any(|(a,b)| ((a-a0)/(a1-a0)-(b-b0)/(b1-b0)).abs()>128.0*f64::EPSILON) {return false;}
    let (wa,wb)=(a.control_points[0].w,b.control_points[0].w);
    if !(wa>0.0 && wb>0.0 && wa.is_finite() && wb.is_finite()) {return false;}
    let Ok(origin)=a.control_points[0].point() else {return false;};
    let extent=a.control_points.iter().filter_map(|p|p.point().ok()).map(|p|p.sub(origin).length()).fold(0.0,f64::max);
    let minimum=a.control_points.iter().map(|p|p.w/wa).chain(b.control_points.iter().map(|p|p.w/wb)).fold(1.0,f64::min);
    if !(extent>0.0 && extent.is_finite() && minimum>0.0) {return false;}
    // With positive normalized weights >= m, a weight change delta moves
    // the curve by at most (2*extent+band)*delta/m. Allocate half a band
    // to it; together with the position bound the error is below two bands.
    let weight_band=band*minimum/(4.0*extent+2.0*band);

    a.control_points.iter().zip(&b.control_points).all(|(a,b)| {
        a.w>0.0 && b.w>0.0 && (a.w/wa-b.w/wb).abs()<=weight_band &&
        match (a.point(),b.point()) {(Ok(a),Ok(b))=>a.add(translation).sub(b).length()<=band,_=>false}
    })
}

fn read(curve:&NurbsCurve,normal:Vec3,band:f64,angular:f64)->Option<Piece> {
    let [start,end]=curve.domain().ok()?;
    let a=curve.evaluate(start).ok()?;
    let b=curve.evaluate(end).ok()?;
    if curve.degree==1 && curve.control_points.len()==2 {
        return Some(Piece::Line {a,b});
    }
    if curve.degree!=2 {return None;}
    let circle=crate::analytic_surface::recognize_circle(curve,start,end,band)?;
    if circle.axis.cross(normal).length()>angular {return None;}
    let radial=a.sub(circle.centre).normalized().ok()?;
    let controls=curve.control_points.iter().map(|p|p.point().ok()).collect::<Option<Vec<_>>>()?;
    if controls.len()<3 || controls.len()%2!=1 {return None;}
    let mut angle=0.0;
    for i in (0..controls.len()-2).step_by(2) {
        let u=controls[i].sub(circle.centre).normalized().ok()?;
        let v=controls[i+2].sub(circle.centre).normalized().ok()?;
        let turn=u.cross(v).dot(normal).atan2(u.dot(v));
        if turn<=0.0 {return None;}
        angle+=turn;
    }
    if angle>TAU+angular {return None;}
    let piece=Piece::Arc {centre:circle.centre,radial,radius:circle.radius,angle:angle.min(TAU)};
    let reconstructed=piece.curve(normal).ok()?;
    same_net(curve,&reconstructed,Vec3::new(0.0,0.0,0.0),band).then_some(piece)
}

fn rim(solid:&BrepSolid,face:&crate::FaceRecord)->Option<Vec<NurbsCurve>> {
    face.loops[0].coedges.iter().map(|c| {
        let e=solid.edges.iter().find(|e|e.id==c.edge_id)?;
        let [lo,hi]=e.curve.domain().ok()?;
        if e.t0!=lo || e.t1!=hi {return None;}
        if c.forward {Some(e.curve.clone())} else {e.curve.reversed().ok()}
    }).collect()
}

pub(super) fn build(solid:&BrepSolid,picks:&[Vec3],radius:f64)->Option<Result<BrepSolid,KernelRefusal>> {
    if solid.shells.len()!=1 || solid.vertices.is_empty() || !radius.is_finite() || radius<=0.0 || picks.len()!=solid.edges.len() {return None;}
    let faces=&solid.shells[0].faces;
    if faces.len()<3 || faces.iter().any(|f|f.loops.len()!=1) {return None;}
    let mut selected=picks.iter().map(|p|super::edges::resolve_edge_by_point(solid,*p).ok()).collect::<Option<Vec<_>>>()?;
    selected.sort_unstable();selected.dedup();
    if selected.len()!=solid.edges.len() {return None;}
    let anchor=solid.vertices[0].point;
    let scale=solid.vertices.iter().map(|p|p.point.sub(anchor).length()).fold(radius,f64::max);
    let coordinate=solid.vertices.iter().map(|p|p.point.x.abs().max(p.point.y.abs()).max(p.point.z.abs())).fold(0.0,f64::max);
    let band=256.0*f64::EPSILON*(scale+coordinate);
    let angular=(band/radius).max(1e-12);
    if angular>1e-8 || radius<=100.0*band {return None;}
    for i in 0..faces.len() {
        let Some((normal,on))=super::full_round::plane_of(&faces[i]) else {continue;};
        for j in i+1..faces.len() {
            let Some((opposite,end))=super::full_round::plane_of(&faces[j]) else {continue;};
            if normal.add(opposite).length()>angular || (normal.dot(on.sub(end))-2.0*radius).abs()>band {continue;}
            let Some(mut curves)=rim(solid,&faces[i]) else {continue;};
            let Some(other)=rim(solid,&faces[j]) else {continue;};
            let count=curves.len();
            if count==0 || other.len()!=count || faces.len()!=count+2 || solid.edges.len()!=3*count || solid.vertices.len()!=2*count {continue;}
            if curves.iter().flat_map(|c|&c.control_points).any(|p|p.point().map_or(true,|p|normal.dot(p.sub(on)).abs()>band)) {continue;}
            let translation=normal.scale(-2.0*radius);
            if curves.iter().any(|c| !other.iter().any(|o|same_net(c,o,translation,band) || o.reversed().ok().is_some_and(|o|same_net(c,&o,translation,band)))) {continue;}
            // Every wall must be an affine sweep along the cap normal,
            // proven from its control net, over exactly the cap distance.
            if faces.iter().enumerate().any(|(k,f)| {
                if k==i || k==j {return false;}
                let Some(e)=f.surface.linear_extrusion() else {return true;};
                let delta=e.direction.scale(e.sweep_domain[1]-e.sweep_domain[0]);
                delta.cross(normal).length()>band || (delta.dot(normal).abs()-2.0*radius).abs()>band
            }) {continue;}
            let controls=curves.iter().flat_map(|c|c.control_points.iter()).map(|p|p.point().ok()).collect::<Option<Vec<_>>>()?;
            let area=(0..controls.len()).map(|k|controls[k].sub(on).cross(controls[(k+1)%controls.len()].sub(on)).dot(normal)).sum::<f64>();
            if area<0.0 {curves=curves.iter().rev().map(|c|c.reversed().ok()).collect::<Option<Vec<_>>>()?;}
            if angular*count as f64>1e-8 || (0..count).any(|k| {
                let a=&curves[k];let b=&curves[(k+1)%count];
                match (a.domain(),b.domain()) {
                    (Ok([_,hi]),Ok([lo,_]))=>match (a.evaluate(hi),b.evaluate(lo)) {(Ok(a),Ok(b))=>a.sub(b).length()>4.0*band,_=>true},
                    _=>true,
                }
            }) {continue;}
            let Some(pieces)=curves.iter().map(|c|read(c,normal,band,angular)).collect::<Option<Vec<_>>>() else {continue;};
            // A closed regular tangent chain with positive curvature and
            // one full tangent turn is convex. No point samples certify it.
            let turn=pieces.iter().map(|p|match p {Piece::Arc {angle,..}=>*angle,_=>0.0}).sum::<f64>();
            if (turn-TAU).abs()>angular*count as f64 {continue;}
            if (0..count).any(|k| {
                match (pieces[k].tangents(normal),pieces[(k+1)%count].tangents(normal)) {
                    (Some((_,a)),Some((b,_)))=>a.sub(b).length()>angular,
                    _=>true,
                }
            }) {continue;}
            if pieces.iter().all(|p|match p {Piece::Arc {radius:r,..}=>*r<=radius+band,_=>true}) {
                // All rounded polygon corners disappear from the inset.
                // Reconstruct the intersections of their supporting lines;
                // the polygon limit then supplies the spherical wedges.
                let Some(corners)=unrounded_corners(&pieces,normal,radius,angular) else {continue;};
                match super::plate_limit::inset_and_sew(&corners,normal,radius,band) {
                    Ok(Some(mut body))=>{
                        for f in body.shells.iter_mut().flat_map(|s|&mut s.faces) {
                            if let Some((n,_))=super::full_round::plane_of(f) {f.name=if n.dot(normal)>0.0 {faces[i].name.clone()} else {faces[j].name.clone()};}
                        }
                        return Some(Ok(body));
                    },
                    Ok(None)=>continue,
                    Err(error)=>return Some(Err(error)),
                }
            }
            let Some(inset)=pieces.iter().map(|p|p.inset(normal,radius)).collect::<Option<Vec<_>>>() else {continue;};
            let inner=inset.iter().map(|p|p.curve(normal).ok()).collect::<Option<Vec<_>>>()?;
            if (0..count).any(|k| {
                let a=&inner[k];let b=&inner[(k+1)%count];
                match (a.domain(),b.domain()) {
                    (Ok([_,hi]),Ok([lo,_]))=>match (a.evaluate(hi),b.evaluate(lo)) {(Ok(a),Ok(b))=>a.sub(b).length()>4.0*band,_=>true},
                    _=>true,
                }
            }) {continue;}
            return Some(sew(&inset,&inner,normal,radius,band).map(|mut body| {
                for f in body.shells.iter_mut().flat_map(|s|&mut s.faces) {
                    if let Some((n,_))=super::full_round::plane_of(f) {f.name=if n.dot(normal)>0.0 {faces[i].name.clone()} else {faces[j].name.clone()};}
                }
                body
            }));
        }
    }
    None
}

/// Only alternating straight segments and convex rounded corners represent
/// a rounded polygon. A semicircle, a mixed surviving/disappearing arc set,
/// or a sharp line/line join needs a different inset topology.
fn unrounded_corners(pieces:&[Piece],normal:Vec3,radius:f64,angular:f64)->Option<Vec<Vec3>> {
    let lines:Vec<_>=pieces.iter().filter_map(|p|match p {Piece::Line {a,b}=>Some((*a,*b)),_=>None}).collect();
    if lines.len()<3 || pieces.len()!=2*lines.len() {return None;}
    for i in 0..pieces.len() {
        if matches!(pieces[i],Piece::Line {..})==matches!(pieces[(i+1)%pieces.len()],Piece::Line {..}) {return None;}
        if let Piece::Arc {angle,..}=pieces[i] {if angle>=std::f64::consts::PI-angular {return None;}}
    }
    let mut corners=Vec::new();
    for i in 0..lines.len() {
        let (a,b)=lines[(i+lines.len()-1)%lines.len()];
        let (c,d)=lines[i];
        let t=b.sub(a).normalized().ok()?;
        let inward=normal.cross(d.sub(c).normalized().ok()?);
        let denominator=t.dot(inward);
        if denominator.abs()<=angular {return None;}
        let distance=c.sub(a).dot(inward)/denominator;
        corners.push(a.add(t.scale(distance)).sub(normal.scale(radius)));
    }
    Some(corners)
}

fn sew(pieces:&[Piece],inner:&[NurbsCurve],normal:Vec3,radius:f64,band:f64)->Result<BrepSolid,KernelRefusal> {
    let tolerance=(4.0*band).max(1e-7);
    if radius<=100.0*tolerance {return Err(KernelRefusal::unsupported(KernelStage::Sew,"plate_limit_scale","full-round plate: the radius is inside the sewing resolution"));}
    let mut sheets=Vec::new();
    for p in pieces {
        let surface=match *p {
            Piece::Line {a,b}=>{
                let out=b.sub(a).normalized().or_refuse(KernelStage::Sew,"plate_limit_tangent")?.cross(normal);
                let arc=crate::make_arc(a,out,normal,radius,-FRAC_PI_2,FRAC_PI_2).or_refuse(KernelStage::Sew,"plate_limit_arc")?;
                crate::make_extrusion(&arc,b.sub(a))
            },
            Piece::Arc {centre,radial,radius:r,angle}=>{
                let at=centre.add(radial.scale(r));
                let arc=crate::make_arc(at,radial,normal,radius,-FRAC_PI_2,FRAC_PI_2).or_refuse(KernelStage::Sew,"plate_limit_arc")?;
                crate::make_revolution(centre,normal,&arc,angle)
            }
        }.or_refuse(KernelStage::Sew,"plate_limit_strip")?;
        sheets.push(crate::face_from_surface(&surface)?);
    }
    let origin=inner[0].evaluate(inner[0].domain().or_refuse(KernelStage::Sew,"plate_limit_domain")?[0]).or_refuse(KernelStage::Sew,"plate_limit_point")?;
    let x=normal.perpendicular().or_refuse(KernelStage::Sew,"plate_limit_frame")?;let y=normal.cross(x);
    let (mut lo,mut hi)=([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for c in inner {for p in &c.control_points {
        let d=p.point().or_refuse(KernelStage::Sew,"plate_limit_point")?.sub(origin);
        let uv=[d.dot(x),d.dot(y)];for k in 0..2 {lo[k]=lo[k].min(uv[k]);hi[k]=hi[k].max(uv[k]);}
    }}
    for sign in [-1.0,1.0] {
        let shift=normal.scale(sign*radius);
        let rim=inner.iter().map(|c| {
            let cps=c.control_points.iter().map(|p|crate::Vec4 {x:p.x+shift.x*p.w,y:p.y+shift.y*p.w,z:p.z+shift.z*p.w,w:p.w}).collect();
            NurbsCurve::new(c.degree,c.knots.clone(),cps)
        }).collect::<Result<Vec<_>,_>>().or_refuse(KernelStage::Sew,"plate_limit_rim")?;
        let corner=origin.add(shift).add(x.scale(lo[0]-radius)).add(y.scale(lo[1]-radius));
        let plane=crate::make_plane(corner,x,y,hi[0]-lo[0]+2.0*radius,hi[1]-lo[1]+2.0*radius).or_refuse(KernelStage::Sew,"plate_limit_cap")?;
        sheets.push(crate::face_from_surface_trimmed(&plane,&[rim])?);
    }
    crate::solid_from_sheets(&sheets,tolerance)
}

