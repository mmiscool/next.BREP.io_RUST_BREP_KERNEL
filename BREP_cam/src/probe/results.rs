//! Pure evaluation of one complete probe cycle.
use crate::*;
use std::collections::BTreeMap;
#[derive(Clone,Debug,PartialEq)]
pub struct Trigger {pub result_var:String,pub position:Point3}
#[derive(Clone,Debug,PartialEq)]
pub enum Evaluation {
    /// Machine = work + origin. Unobserved components retain their old value.
    Translation {updated:WorkOffset,delta:Point3,feature_radius:Option<f64>},
    /// Rotation measurement only: WorkOffset cannot represent rotation/pivot.
    Angle {wcs:u32,radians:f64},
}
fn invalid(s:&str)->CamError {CamError::InvalidInput(format!("probe results: {s}"))}
/// Measurements must be machine-space sphere-bottom trigger coordinates.
/// `eps` is an absolute distance consistency tolerance in program units.
/// This returns a proposal; it never changes a setup or emits controller code.
pub fn evaluate(cycles:&[ProbeCycle],triggers:&[Trigger],eps:f64)->Result<Evaluation,CamError> {
    if !eps.is_finite()||eps<=0.||cycles.is_empty()||cycles.len()!=triggers.len(){return Err(invalid("invalid tolerance or incomplete cycle"));}
    let first=&cycles[0];let wcs=first.wcs.as_ref().ok_or_else(||invalid("inspection has no WCS update"))?;
    if wcs.number==0||!wcs.origin.is_finite()||first.kind==ProbeKind::Inspection {return Err(invalid("invalid WCS intent"));}
    let points:Vec<_>=cycles.iter().map(|c|c.nominal.clone()).collect();super::geometry::validate_pattern(first.kind,&points)?;
    let mut measured=BTreeMap::new();for t in triggers {if !t.position.is_finite()||measured.insert(t.result_var.as_str(),t.position).is_some(){return Err(invalid("nonfinite or duplicate trigger"));}}
    let mut surfaces=Vec::new();let mut centers=Vec::new();let mut ids=std::collections::BTreeSet::new();
    for (i,c) in cycles.iter().enumerate() {
        if !c.is_finite()||c.kind!=first.kind||c.wcs.as_ref()!=Some(wcs)||c.point_index!=i as u32||c.point_count!=cycles.len() as u32||!ids.insert(&c.nominal.id)||c.result_var.is_empty()||(c.nominal.normal.length()-1.).abs()>1e-9 {return Err(invalid("inconsistent cycle metadata"));}
        let n=c.nominal.normal;let expected=c.nominal.position.add(n.scale(c.stylus_radius)).sub(Point3::Z.scale(c.stylus_radius));if c.target.distance(expected)>eps {return Err(invalid("inconsistent compensated target"));}
        let travel=c.approach.sub(c.target);let length=travel.dot(n);if length<=0.||travel.sub(n.scale(length)).length()>eps {return Err(invalid("invalid approach line"));}
        let tip=measured.remove(c.result_var.as_str()).ok_or_else(||invalid("missing or duplicate result variable"))?;let d=tip.sub(c.target);let along=d.dot(n);if d.sub(n.scale(along)).length()>eps||along>length+eps||along< -c.overtravel-eps {return Err(invalid("trigger outside search segment"));}
        let center=tip.add(Point3::Z.scale(c.stylus_radius));if !center.is_finite(){return Err(invalid("coordinate overflow"));}centers.push(center);surfaces.push(center.sub(n.scale(c.stylus_radius)));
    }
    if !measured.is_empty(){return Err(invalid("unexpected result variable"));}
    let normal_delta=|i:usize|surfaces[i].sub(points[i].position).dot(points[i].normal);
    let pair=|i:usize|points[i].normal.scale((normal_delta(i)-normal_delta(i+1))*0.5);
    let mut radius=None;
    let delta=match first.kind {
        ProbeKind::SingleSurface=>points[0].normal.scale(normal_delta(0)),
        ProbeKind::InsideCorner|ProbeKind::OutsideCorner=>points[0].normal.scale(normal_delta(0)).add(points[1].normal.scale(normal_delta(1))),
        ProbeKind::Web|ProbeKind::Pocket=>pair(0),
        ProbeKind::Rectangle=>pair(0).add(pair(2)),
        ProbeKind::Bore|ProbeKind::Boss=>{
            if cycles.iter().any(|c|(c.stylus_radius-first.stylus_radius).abs()>eps)||centers.iter().any(|p|(p.z-centers[0].z).abs()>eps){return Err(invalid("circle needs equal radii and equal Z"));}
            // Fit the circle of sphere CENTRES: nominal-normal compensation
            // would bias an off-centre bore because its actual normals changed.
            let a=centers[1].sub(centers[0]);let b=centers[2].sub(centers[0]);let det=a.x*b.y-a.y*b.x;
            if det.abs()<=eps*(a.length()+b.length()){return Err(invalid("degenerate circle contacts"));}
            let u=(a.x*a.x+a.y*a.y)*0.5;let v=(b.x*b.x+b.y*b.y)*0.5;let center=centers[0].add(Point3::new((u*b.y-v*a.y)/det,(a.x*v-b.x*u)/det,0.));let rho=center.distance(centers[0]);if centers.iter().any(|p|(center.distance(*p)-rho).abs()>eps){return Err(invalid("inconsistent circular feature"));}
            let r=if first.kind==ProbeKind::Bore {rho+first.stylus_radius}else{rho-first.stylus_radius};if r<=eps {return Err(invalid("invalid compensated feature radius"));}radius=Some(r);let nominal=points[0].position.add(points[1].position).scale(0.5);Point3::new(center.x-nominal.x,center.y-nominal.y,0.)
        },
        ProbeKind::Angle=>{
            if (first.stylus_radius-cycles[1].stylus_radius).abs()>eps {return Err(invalid("angle needs equal stylus radii"));}
            let tangent=centers[1].sub(centers[0]);let tangent=Point3::new(tangent.x,tangent.y,0.).normalized().ok_or_else(||invalid("degenerate angle contacts"))?;let mut n=tangent.cross(Point3::Z);if n.dot(points[0].normal)<0. {n=n.scale(-1.);}let dot=points[0].normal.dot(n);if dot<=1e-9{return Err(invalid("ambiguous face orientation"));}let radians=points[0].normal.cross(n).z.atan2(dot);if !radians.is_finite(){return Err(invalid("angle overflow"));}return Ok(Evaluation::Angle {wcs:wcs.number,radians});
        },
        ProbeKind::Inspection=>return Err(invalid("inspection never updates WCS")),
    };
    let updated=WorkOffset {number:wcs.number,origin:wcs.origin.add(delta)};if !delta.is_finite()||!updated.origin.is_finite()||radius.is_some_and(|r|!r.is_finite()){return Err(invalid("arithmetic overflow"));}Ok(Evaluation::Translation {updated,delta,feature_radius:radius})
}
