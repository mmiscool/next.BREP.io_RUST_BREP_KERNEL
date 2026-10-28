use crate::{CamError,Point3,Runner};
use crate::model::{RigidTransform,Toolpath,MotionKind,ArcPlane};
use crate::link::Pass;
use crate::multiaxis::indexed::linearize_arc;
fn invalid(s:&str)->CamError {CamError::InvalidInput(format!("pattern: {s}"))}
/// Derived placement. Reflections are deliberately not mislabeled as a proper
/// RigidTransform (which rejects a negative determinant).
#[derive(Clone,Debug)]
pub enum Placement { Rigid(RigidTransform), Reflection{origin:Point3,normal:Point3} }
impl Placement {
    pub fn rigid(t:RigidTransform)->Result<Self,CamError> {
        t.validate()?;
        if t.apply_vector(Point3::Z).distance(Point3::Z)>1e-10 {return Err(invalid("only +Z-preserving placements are supported; use indexed generation for tilted instances"));}
        Ok(Self::Rigid(t))
    }
    pub fn mirror(origin:Point3,normal:Point3)->Result<Self,CamError> {
        let n=normal.normalized().ok_or_else(||invalid("mirror normal is zero or invalid"))?;
        if !origin.is_finite() || n.z.abs()>1e-10 {return Err(invalid("mirror plane must be vertical and finite"));}
        Ok(Self::Reflection{origin,normal:n})
    }
    pub fn point(&self,p:Point3)->Point3 {match self {
        Self::Rigid(t)=>t.apply_point(p),Self::Reflection{origin,normal}=>p.sub(normal.scale(2.*p.sub(*origin).dot(*normal))),
    }}
    pub fn reverses(&self)->bool {matches!(self,Self::Reflection{..})}
}
/// Extract CUT geometry only, discard old leads/links and re-link later. Every
/// reflected pass reverses traversal, retaining the source material side. Arcs
/// become bounded chords before reflection; therefore reflected arc sense and
/// traversal reversal are both represented in the resulting ordered points.
/// Cycles, multi-axis/lathe/manual/control motions refuse rather than disappear.
pub fn transform_passes(path:&Toolpath,placement:&Placement,chord:f64,runner:&mut Runner<'_>)->Result<Vec<Pass>,CamError> {
    runner.token().check()?;
    if !path.report.is_usable() || !chord.is_finite() || chord<=0. || path.motions.len()>200_000 {return Err(invalid("unusable path or invalid transform work budget"));}
    let placement=match placement {Placement::Rigid(t)=>Placement::rigid(t.clone())?,Placement::Reflection{origin,normal}=>Placement::mirror(*origin,*normal)?};
    let mut out=Vec::new();let mut current:Option<Pass>=None;let mut at:Option<Point3>=None;let mut count=0usize;
    for m in &path.motions {
        runner.work(1,"pattern-transform",count as u64,0)?;
        if !m.kind.is_finite() || m.tool_axis.is_some_and(|a|a.distance(Point3::Z)>1e-10) {return Err(invalid("requires finite +Z source motions"));}
        let points=match &m.kind {
            MotionKind::Cut{to}=>Some(vec![*to]),
            MotionKind::ArcCw{to,center,plane,pitch}|MotionKind::ArcCcw{to,center,plane,pitch}=>{
                if *plane!=ArcPlane::Xy || *pitch!=0. {return Err(invalid("pattern requires planar XY arcs; helical/indexed arcs must be explicitly expanded"));}
                let start=at.ok_or_else(||invalid("arc has no start"))?;
                let radius=start.distance(*center);
                if radius<=0. || (to.distance(*center)-radius).abs()>1e-9*radius.max(1.) || (start.z-to.z).abs()>1e-9 || (start.z-center.z).abs()>1e-9 {return Err(invalid("source arc has inconsistent radius or plane"));}
                let pts=linearize_arc(start,*to,*center,*plane,*pitch,matches!(m.kind,MotionKind::ArcCw{..}),chord);
                // Independently pin the subdivision cap's sag bound.
                let r=start.distance(*center);
                if pts.is_empty() || pts.windows(2).any(|w|w[0].lerp(w[1],0.5).distance(*center)<r-chord-1e-9) || start.lerp(pts[0],0.5).distance(*center)<r-chord-1e-9 {return Err(invalid("arc chord budget cannot meet tolerance"));}
                Some(pts)
            }
            MotionKind::Rapid{..}|MotionKind::Plunge{..}|MotionKind::Retract{..}|MotionKind::Link{..}|MotionKind::LeadIn{..}|MotionKind::LeadOut{..}=>None,
            _=>return Err(invalid("source contains unsupported control/cycle/rotary/manual motions")),
        };
        if let Some(points)=points {
            let start=at.ok_or_else(||invalid("cut has no start"))?;
            if current.as_ref().is_some_and(|p|p.path_id!=m.path_id) {out.push(current.take().unwrap());}
            let p=current.get_or_insert_with(||Pass::new(vec![placement.point(start)],m.path_id));
            count+=points.len();if count>200_000 {return Err(invalid("transformed point budget exceeded"));}
            p.points.extend(points.into_iter().map(|p|placement.point(p)));
        } else if let Some(p)=current.take() {out.push(p);}
        if let Some(p)=m.end(){at=Some(p);}
    }
    if let Some(p)=current {out.push(p);}
    if out.is_empty(){return Err(invalid("source has no cut passes"));}
    if placement.reverses(){for p in &mut out {p.points.reverse();}}
    runner.token().check()?;Ok(out)
}
