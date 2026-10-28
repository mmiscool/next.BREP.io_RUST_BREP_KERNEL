use crate::{CamError,Point3,Tool,ToolKind};
use serde::{Serialize,Deserialize};
use std::f64::consts::TAU;

#[derive(Clone,Copy,Debug,PartialEq,Eq,Serialize,Deserialize)]
pub enum InsertShape { C,D,V,T,W,R,S }

/// ISO shape letter determines the included corner angle. For a round insert,
/// edge_length means diameter and nose_radius must be half that diameter.
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct Insert {
    pub shape:InsertShape,
    pub edge_length:f64,
    pub nose_radius:f64,
    pub clearance_deg:f64,
    /// Rotation of the insert in XZ, in addition to holder orientation.
    pub lead_angle_deg:f64,
    pub holder_angle_deg:f64,
}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct Blade {pub width:f64,pub reach:f64,pub corner_radius:f64,pub holder_angle_deg:f64}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct ThreadInsert {pub pitch_min:f64,pub pitch_max:f64,pub angle_deg:f64,pub reach:f64,pub nose_radius:f64,pub holder_angle_deg:f64}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct BoringBar {pub diameter:f64,pub reach:f64,pub insert:Insert,pub insert_offset:Point3}

/// Geometry supplement for the shared Tool; not a replacement Tool model.
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(tag="kind",rename_all="camelCase")]
pub enum TurningGeometry {Insert(Insert),Blade(Blade),Thread(ThreadInsert),BoringBar(BoringBar)}

/// Conservative convex XZ envelope about its geometric reference. Insert
/// reference is its centre; blade reference is the middle of its cutting edge.
/// `reference_to_tip` maps the reference to the selected cutting tip centre.
/// Callers must translate by this offset when their toolpath uses that tip.
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct ToolEnvelope {
    pub vertices:Vec<Point3>,
    pub reference_to_tip:Point3,
    pub chord_bound:f64,
    /// Rake/clearance is a 3D accessibility condition, not a planar shrink.
    pub clearance_deg:f64,
}
impl ToolEnvelope {
    pub fn validate(&self)->Result<(),CamError> {
        if self.vertices.len()<3 || self.vertices.iter().any(|p|!p.is_finite() || p.y!=0.) || !self.reference_to_tip.is_finite() || self.reference_to_tip.y!=0. || !positive(self.chord_bound) {return Err(invalid("invalid turning envelope"));}
        Ok(())
    }
    pub fn support(&self,direction:Point3)->Result<f64,CamError> {
        self.validate()?;
        if !direction.is_finite() || direction.y!=0. {return Err(invalid("turning support direction must lie in XZ"));}
        Ok(self.vertices.iter().map(|p|p.dot(direction)).fold(f64::NEG_INFINITY,f64::max))
    }
}
fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}
fn positive(v:f64)->bool {v.is_finite() && v>0.}
fn rotate(p:Point3,angle:f64)->Point3 {let (s,c)=angle.sin_cos();Point3::new(c*p.x-s*p.z,0.,s*p.x+c*p.z)}
fn p(x:f64,z:f64)->Point3 {Point3::new(x,0.,z)}

/// Conservative polygonization uses intersections of adjacent arc tangents,
/// never inscribed chords. Maximum arc excess is bounded by `chord`.
pub fn envelope(tool:&Tool,geometry:&TurningGeometry,chord:f64)->Result<ToolEnvelope,CamError> {
    if !matches!(tool.kind,ToolKind::LatheInsert|ToolKind::BoringBar) || !positive(chord) {return Err(invalid("turning envelope needs a lathe tool and positive chord tolerance"));}
    let result=match geometry {
        TurningGeometry::Insert(g)=>insert_envelope(g,chord),
        TurningGeometry::Blade(g)=>{
            if !positive(g.width) || !positive(g.reach) || !g.corner_radius.is_finite() || g.corner_radius<0. || 2.*g.corner_radius>g.width.min(g.reach) || !g.holder_angle_deg.is_finite() {return Err(invalid("invalid grooving blade dimensions"));}
            let raw=vec![p(0.,-g.width/2.),p(g.reach,-g.width/2.),p(g.reach,g.width/2.),p(0.,g.width/2.)];
            let mut outline=rounded_polygon(&raw,g.corner_radius,chord)?;
            // Only the two cutting-edge corners are rounded. Preserve square
            // rear corners so the blade body is never under-enveloped.
            outline.extend([p(g.reach,-g.width/2.),p(g.reach,g.width/2.)]);
            let vertices=hull(outline).into_iter().map(|v|rotate(v,g.holder_angle_deg.to_radians())).collect();
            Ok(ToolEnvelope{vertices,reference_to_tip:Point3::ZERO,chord_bound:chord,clearance_deg:0.})
        }
        TurningGeometry::Thread(g)=>{
            if !positive(g.pitch_min) || g.pitch_max<g.pitch_min || !g.pitch_max.is_finite() || !positive(g.reach) || !g.angle_deg.is_finite() || g.angle_deg<=0. || g.angle_deg>=180. || !g.nose_radius.is_finite() || g.nose_radius<0. || !g.holder_angle_deg.is_finite() {return Err(invalid("invalid threading insert dimensions"));}
            let half=g.reach*(0.5*g.angle_deg.to_radians()).tan();
            let raw=vec![p(0.,0.),p(g.reach,-half),p(g.reach,half)];
            let vertices=rounded_polygon(&raw,g.nose_radius,chord)?.into_iter().map(|v|rotate(v,g.holder_angle_deg.to_radians())).collect();
            Ok(ToolEnvelope{vertices,reference_to_tip:rotate(p(g.nose_radius/(0.5*g.angle_deg.to_radians()).sin(),0.),g.holder_angle_deg.to_radians()),chord_bound:chord,clearance_deg:0.})
        }
        TurningGeometry::BoringBar(g)=>{
            if !positive(g.diameter) || !positive(g.reach) || !g.insert_offset.is_finite() || g.insert_offset.y!=0. {return Err(invalid("invalid boring bar dimensions"));}
            let mut e=insert_envelope(&g.insert,chord)?;
            for v in &mut e.vertices {*v=v.add(g.insert_offset);}
            e.reference_to_tip=e.reference_to_tip.add(g.insert_offset);
            e.vertices.extend([p(-g.diameter/2.,0.),p(g.diameter/2.,0.),p(g.diameter/2.,g.reach),p(-g.diameter/2.,g.reach)]);
            e.vertices=hull(e.vertices);Ok(e)
        }
    }?;
    result.validate()?;Ok(result)
}
fn insert_envelope(g:&Insert,chord:f64)->Result<ToolEnvelope,CamError> {
    if !positive(g.edge_length) || !g.nose_radius.is_finite() || g.nose_radius<0. || !g.clearance_deg.is_finite() || !(0. ..90.).contains(&g.clearance_deg) || !g.lead_angle_deg.is_finite() || !g.holder_angle_deg.is_finite() {return Err(invalid("invalid turning insert dimensions"));}
    let l=g.edge_length;let angle=(g.lead_angle_deg+g.holder_angle_deg).to_radians();
    if g.shape==InsertShape::R {
        if (g.nose_radius-l/2.).abs()>chord {return Err(invalid("round insert nose radius must equal half its diameter"));}
        let vertices=circle(p(0.,0.),l/2.,0.,TAU,chord)?.into_iter().map(|v|rotate(v,angle)).collect();
        return Ok(ToolEnvelope{vertices,reference_to_tip:Point3::ZERO,chord_bound:chord,clearance_deg:g.clearance_deg});
    }
    let raw=match g.shape {
        InsertShape::S=>vec![p(l/2.,l/2.),p(-l/2.,l/2.),p(-l/2.,-l/2.),p(l/2.,-l/2.)],
        InsertShape::T=>(0..3).map(|i|p(l/3_f64.sqrt()*(i as f64*TAU/3.).cos(),l/3_f64.sqrt()*(i as f64*TAU/3.).sin())).collect(),
        InsertShape::W=>{let t=40_f64.to_radians().tan();let k=2.*t/(3_f64.sqrt()+t);let r=l/(1.+k*k-k).sqrt();(0..6).map(|i|{let radius=if i%2==0{r}else{k*r};let a=i as f64*TAU/6.;p(radius*a.cos(),radius*a.sin())}).collect()},
        shape=>{let deg:f64=match shape {InsertShape::C=>80.,InsertShape::D=>55.,InsertShape::V=>35.,_=>unreachable!()};let a=l*(deg.to_radians()/2.).cos();let b=l*(deg.to_radians()/2.).sin();vec![p(a,0.),p(0.,b),p(-a,0.),p(0.,-b)]},
    };
    let centre=corner_centre(&raw,0,g.nose_radius)?;
    let vertices=rounded_polygon(&raw,g.nose_radius,chord)?.into_iter().map(|v|rotate(v,angle)).collect();
    Ok(ToolEnvelope{vertices,reference_to_tip:rotate(centre,angle),chord_bound:chord,clearance_deg:g.clearance_deg})
}
fn corner_centre(poly:&[Point3],i:usize,r:f64)->Result<Point3,CamError> {
    let at=poly[i];let a=poly[(i+poly.len()-1)%poly.len()].sub(at).normalized().ok_or_else(||invalid("degenerate insert edge"))?;
    let b=poly[(i+1)%poly.len()].sub(at).normalized().ok_or_else(||invalid("degenerate insert edge"))?;
    let bisector=a.add(b).normalized().ok_or_else(||invalid("degenerate insert corner"))?;
    let sin=(0.5*(1.-a.dot(b))).max(0.).sqrt();
    if sin<=0. {return Err(invalid("degenerate insert angle"));}
    Ok(at.add(bisector.scale(r/sin)))
}
fn rounded_polygon(poly:&[Point3],r:f64,chord:f64)->Result<Vec<Point3>,CamError> {
    if r==0. {return Ok(poly.to_vec());}
    let mut out=Vec::new();
    for i in 0..poly.len() {
        let at=poly[i];let prev=poly[(i+poly.len()-1)%poly.len()];let next=poly[(i+1)%poly.len()];
        let a=prev.sub(at).normalized().unwrap();let b=next.sub(at).normalized().unwrap();
        let centre=corner_centre(poly,i,r)?;
        let along=centre.sub(at).dot(a);
        if along>0.5*at.distance(prev) || along>0.5*at.distance(next) {return Err(invalid("nose radius cannot fit insert edges"));}
        let from=at.add(a.scale(along)).sub(centre);let to=at.add(b.scale(along)).sub(centre);
        let start=from.z.atan2(from.x);let sweep=(to.z.atan2(to.x)-start).rem_euclid(TAU);
        out.extend(circle(centre,r,start,sweep,chord)?);
    }
    Ok(out)
}
fn circle(centre:Point3,r:f64,start:f64,sweep:f64,chord:f64)->Result<Vec<Point3>,CamError> {
    let max_angle=2.*(r/(r+chord)).acos();
    let steps=(sweep/max_angle).ceil().max(1.);
    if !steps.is_finite() || steps>100_000. {return Err(invalid("turning envelope exceeds arc work budget"));}
    let n=steps as usize;let step=sweep/n as f64;
    let mut out=vec![centre.add(p(r*start.cos(),r*start.sin()))];
    for i in 0..n {let a=start+(i as f64+0.5)*step;let radius=r/(step*0.5).cos();out.push(centre.add(p(radius*a.cos(),radius*a.sin())));}
    let end=start+sweep;out.push(centre.add(p(r*end.cos(),r*end.sin())));Ok(out)
}
fn cross(a:Point3,b:Point3,c:Point3)->f64 {(b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x)}
fn hull(mut points:Vec<Point3>)->Vec<Point3> {
    points.sort_by(|a,b|a.x.total_cmp(&b.x).then(a.z.total_cmp(&b.z)));points.dedup();
    let chain=|iter:Vec<Point3>| {let mut c=Vec::new();for p in iter {while c.len()>=2 && cross(c[c.len()-2],c[c.len()-1],p)<=0. {c.pop();}c.push(p);}c};
    let mut lower=chain(points.clone());let mut upper=chain(points.into_iter().rev().collect());lower.pop();upper.pop();lower.extend(upper);lower
}

impl ToolEnvelope {
    /// Convex envelope swept from one TIP-reference position to another. This
    /// is collision construction only, not a profile offset/compensation engine.
    pub fn swept_vertices(&self,start:Point3,end:Point3)->Result<Vec<Point3>,CamError> {
        self.validate()?;
        if !start.is_finite() || !end.is_finite() || start.y!=0. || end.y!=0. {return Err(invalid("turning sweep must lie in XZ"));}
        Ok(hull(self.vertices.iter().flat_map(|v|[v.sub(self.reference_to_tip).add(start),v.sub(self.reference_to_tip).add(end)]).collect()))
    }
    /// Conservative collision against a filled, simple material polygon.
    /// Touching counts as collision; the caller explicitly supplies any allowed
    /// finishing tolerance through the profile representation.
    pub fn collides(&self,start:Point3,end:Point3,material:&[Point3])->Result<bool,CamError> {
        if material.len()<3 || material.iter().any(|p|!p.is_finite() || p.y!=0.) {return Err(invalid("material section needs finite XZ polygon vertices"));}
        let swept=self.swept_vertices(start,end)?;
        for i in 0..swept.len() {for j in 0..material.len() {
            if segments_touch(swept[i],swept[(i+1)%swept.len()],material[j],material[(j+1)%material.len()]) {return Ok(true);}
        }}
        Ok(inside(swept[0],material)||inside(material[0],&swept))
    }
}
fn segments_touch(a:Point3,b:Point3,c:Point3,d:Point3)->bool {
    if a.x.max(b.x)<c.x.min(d.x) || c.x.max(d.x)<a.x.min(b.x) || a.z.max(b.z)<c.z.min(d.z) || c.z.max(d.z)<a.z.min(b.z) {return false;}
    let [u,v,w,t]=[cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b)];
    ((u<=0. && v>=0.)||(v<=0. && u>=0.)) && ((w<=0. && t>=0.)||(t<=0. && w>=0.))
}
fn inside(p:Point3,poly:&[Point3])->bool {
    let mut yes=false;
    for i in 0..poly.len() {let a=poly[i];let b=poly[(i+1)%poly.len()];if (a.z>p.z)!=(b.z>p.z) && p.x<(b.x-a.x)*(p.z-a.z)/(b.z-a.z)+a.x {yes=!yes;}}
    yes
}
