//! Axisymmetric material removal in (radius,Z), using C5 planar booleans.
//! Sweeps are exact for the supplied convex polygon envelope. A rounded
//! insert's conservative chord bound remains part of the simulation accuracy;
//! this is not a helical-thread or live-tool (non-axisymmetric) simulator.
use crate::{CamError, Point2, Point3, Runner, Tolerance};
use crate::model::{Toolpath, MotionKind};
use crate::planar::{Region, Loop, Vec2};
use super::ToolEnvelope;

/// Derived stock section, never a second persisted Stock model.
#[derive(Clone, Debug)]
pub struct StockState { section: Region }
#[derive(Clone, Debug)]
pub struct Deviation {
    /// Meridian area of stock outside the target and target absent from stock.
    pub excess_area:f64, pub missing_area:f64,
    /// Exact regions for deviation display and subsequent rest machining.
    pub excess:Region, pub missing:Region,
}
fn invalid(s:&str)->CamError { CamError::InvalidInput(format!("turning simulation: {s}")) }
fn validate(r:&Region,t:&Tolerance)->Result<(),CamError> {
    if r.loops.iter().map(|l|l.segments.len()).sum::<usize>()>2048 {return Err(invalid("section exceeds segment budget"));}
    Region::new(r.loops.clone(),t)?;
    if r.loops.iter().flat_map(|l|&l.segments).any(|s|s.is_arc() || s.start.x<0. || s.end.x<0.) {return Err(invalid("requires nonnegative-radius polygon sections"));}
    if !r.is_empty() && r.area()<=0. {return Err(invalid("section must have positive winding"));}
    Ok(())
}
fn sweep(e:&ToolEnvelope,a:Point3,b:Point3,t:&Tolerance)->Result<Region,CamError> {
    let mut p:Vec<Point2>=Vec::new();
    for v in e.swept_vertices(a,b)? {
        let q=Point2::new(v.x,v.z);
        if p.last().is_none_or(|last|last.distance(q)>t.point_eps) {p.push(q);}
    }
    // Round-envelope trig endpoints can differ by a few ulps. Normalize the
    // cyclic closure at the planar engine's declared point resolution.
    while p.len()>1 && p[0].distance(*p.last().unwrap())<=t.point_eps {p.pop();}
    if p.len()<3 {return Err(invalid("swept envelope is below planar resolution"));}
    let original=Region::from_unoriented(vec![Loop::polygon(&p)],t)?;
    if p.iter().all(|p|p.x>=0.) {return Ok(original);}
    // Negative X contacts the opposite meridian of the same spinning stock.
    let reflected:Vec<_>=p.iter().rev().map(|p|Point2::new(-p.x,p.y)).collect();
    let reflected=Region::from_unoriented(vec![Loop::polygon(&reflected)],t)?;
    if p.iter().all(|p|p.x<=0.) {Ok(reflected)} else {Ok(original.union(&reflected,t)?)}
}
fn overlap(a:&Region,b:&Region,t:&Tolerance)->Result<bool,CamError> {
    Ok(!a.intersection(b,t)?.is_empty())
}
fn touching(a:&Region,b:&Region,t:&Tolerance)->Result<bool,CamError> {
    if overlap(a,b,t)? {return Ok(true);}
    for x in a.loops.iter().flat_map(|l|&l.segments) {
        for y in b.loops.iter().flat_map(|l|&l.segments) {
            if !x.intersect(y,t).is_empty() || x.distance(y.start)<=t.point_eps || y.distance(x.start)<=t.point_eps {return Ok(true);}
        }
    }
    Ok(false)
}
impl StockState {
    pub fn new(section:Region,tolerance:Tolerance)->Result<Self,CamError> {
        tolerance.validate()?;validate(&section,&tolerance)?;Ok(Self{section})
    }
    pub fn section(&self)->&Region {&self.section}
    /// Pappus/Green integral 2π∫r dA, exact for polygon sections including holes.
    pub fn volume(&self)->f64 {
        self.section.loops.iter().flat_map(|l|&l.segments).map(|s|{
            let (a,b)=(s.start,s.end);(a.x+b.x)*(a.x*b.y-b.x*a.y)*std::f64::consts::PI/3.
        }).sum()
    }
    pub fn compare(&self,target:&Region,t:Tolerance)->Result<Deviation,CamError> {
        t.validate()?;validate(target,&t)?;
        let excess=self.section.difference(target,&t)?;let missing=target.difference(&self.section,&t)?;
        Ok(Deviation{excess_area:excess.area(),missing_area:missing.area(),excess,missing})
    }
    /// Transactional replay. Holder/blade noncutting parts are separate envelopes
    /// at the SAME tip reference; they are checked against stock before removal.
    /// `protected` is the caller's tolerance-adjusted section; chuck fills z<=chuck_z.
    /// Only ordinary straight XZ moves are accepted. Controls are allowlisted;
    /// cycles, arcs, thread and rotary moves require a different simulator.
    pub fn simulate(&self,path:&Toolpath,start:Point3,cutter:&ToolEnvelope,holders:&[ToolEnvelope],protected:&Region,chuck_z:f64,t:Tolerance,runner:&mut Runner<'_>)->Result<Self,CamError> {
        t.validate()?;validate(protected,&t)?;cutter.validate()?;
        if !chuck_z.is_finite() || !start.is_finite() || start.y!=0. {return Err(invalid("invalid start or chuck plane"));}
        if path.motions.len()>100_000 {return Err(invalid("motion budget exceeded"));}
        runner.stage("turning-sim","replay",0,path.motions.len() as u64)?;
        let mut stock=self.section.clone();let mut at=start;
        for (i,m) in path.motions.iter().enumerate() {
            runner.work(1,"turning-sim",i as u64,path.motions.len() as u64)?;
            let (to,cut)=match m.kind {
                MotionKind::Cut{to}|MotionKind::Plunge{to}|MotionKind::LeadIn{to}|MotionKind::LeadOut{to}=>(to,true),
                MotionKind::Rapid{to}|MotionKind::Retract{to}|MotionKind::Link{to}=>(to,false),
                MotionKind::ToolChange{ref tool} if *tool==path.tool=>continue,
                MotionKind::Stop{..}|MotionKind::SpindleOrient{..}|MotionKind::SpindleSync{..}|MotionKind::LatheMode{..}|MotionKind::SpindleOn{..}|MotionKind::SpindleOff|MotionKind::SpindleSurfaceSpeed{..}|MotionKind::Coolant{..}|MotionKind::Comment{..}|MotionKind::Dwell{..}=>continue,
                _=>return Err(invalid("unsupported motion; requires expanded straight XZ turning moves")),
            };
            if !to.is_finite() || to.y!=0. || m.tool_axis.is_some_and(|a|a!=Point3::Z) {return Err(invalid("move is not straight XZ turning"));}
            for e in std::iter::once(cutter).chain(holders.iter()) {
                if e.swept_vertices(at,to)?.iter().any(|p|p.z<=chuck_z) {return Err(invalid("chuck collision"));}
            }
            let swept=sweep(cutter,at,to,&t)?;
            if overlap(&swept,protected,&t)? {return Err(invalid("protected profile gouge"));}
            for e in holders {
                let h=sweep(e,at,to,&t)?;
                if touching(&h,&stock,&t)? || touching(&h,protected,&t)? {return Err(invalid("holder/blade collision"));}
            }
            if cut {stock=stock.difference(&swept,&t)?;} else if overlap(&swept,&stock,&t)? {return Err(invalid("non-cutting move enters stock"));}
            validate(&stock,&t)?;at=to;
        }
        runner.stage("turning-sim","complete",path.motions.len() as u64,path.motions.len() as u64)?;
        Ok(Self{section:stock})
    }
}
