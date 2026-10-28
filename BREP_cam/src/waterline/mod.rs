//! Uniform and adaptive constant-Z fiber sampling and boundary weave.
pub mod weave;
pub mod adaptive_z;
#[path = "../strategy/horizontal.rs"]
pub mod horizontal;
#[path = "../strategy/ramp.rs"]
pub mod ramp;
#[path = "../strategy/steep_shallow.rs"]
pub mod steep_shallow;
#[path = "../strategy/contour.rs"]
pub mod contour;
use crate::{Bounds3,CamError,Point3,Runner};
use crate::push::{self,Direction,Fiber,PushContext};

#[derive(Clone,Copy,Debug)]
pub struct Sampling {
    pub spacing:f64,
    pub minimum:f64,
    pub flatness_cos:f64,
    pub max_depth:u32,
    pub max_fibers:usize,
    pub adaptive:bool,
    /// Close free gaps that fall inside one grid edge instead of refusing
    /// (see `weave::reconstruct_counted`). Off by default: refining the grid
    /// resolves an ordinary narrow gap; only the tip of a sharp free wedge
    /// stays unresolved at every spacing, and a caller that has run out of
    /// refinement may close it.
    pub close_subgrid_gaps:bool,
}
impl Default for Sampling {
    fn default()->Self {Self {spacing:0.5,minimum:0.01,flatness_cos:0.999,max_depth:16,max_fibers:10000,adaptive:false,close_subgrid_gaps:false}}
}
#[derive(Clone,Debug)]
pub struct Level {
    pub z:f64,
    pub x:Vec<Fiber>,
    pub y:Vec<Fiber>,
    pub loops:Vec<weave::Loop>,
    pub warnings:Vec<String>,
}
fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}

impl Sampling {
    pub fn validate(&self)->Result<(),CamError> {
        if !self.spacing.is_finite() || !self.minimum.is_finite() || self.minimum<=0.0 || self.spacing<self.minimum ||
            !self.flatness_cos.is_finite() || !(0.0..=1.0).contains(&self.flatness_cos) || self.max_depth>52 || self.max_fibers<4 {
            return Err(invalid("invalid waterline sampling controls"));
        }
        Ok(())
    }
}

pub fn levels(top:f64,bottom:f64,step:f64,max_levels:usize)->Result<Vec<f64>,CamError> {
    if !top.is_finite() || !bottom.is_finite() || !step.is_finite() || step<=0.0 || top<bottom {return Err(invalid("invalid waterline depth range or step-down"));}
    let count=((top-bottom)/step).ceil();
    if !count.is_finite() || count>=max_levels as f64 {return Err(invalid("waterline level budget exceeded"));}
    let mut result:Vec<_>=(0..count as usize).map(|i|top-i as f64*step).collect();
    result.push(bottom); Ok(result)
}

fn fiber(direction:Direction,coord:f64,z:f64,b:Bounds3)->Fiber {
    match direction {
        Direction::X=>Fiber::new(direction,Point3::new(b.min.x,coord,z),Point3::new(b.max.x,coord,z)),
        Direction::Y=>Fiber::new(direction,Point3::new(coord,b.min.y,z),Point3::new(coord,b.max.y,z)),
    }
}

fn flat(a:&Fiber,m:&Fiber,b:&Fiber,cos:f64)->bool {
    if a.intervals.len()!=m.intervals.len() || a.intervals.len()!=b.intervals.len() {return false;}
    for ((i,j),k) in a.intervals.iter().zip(&m.intervals).zip(&b.intervals) {
        for (ta,tm,tb) in [(i.lower,j.lower,k.lower),(i.upper,j.upper,k.upper)] {
            let p=a.start.lerp(a.end,ta); let q=m.start.lerp(m.end,tm); let r=b.start.lerp(b.end,tb);
            let u=q.sub(p); let v=r.sub(q);
            if u.dot(v)<cos*u.length()*v.length() {return false;}
        }
    }
    true
}

pub fn sample(context:&PushContext<'_>,direction:Direction,bounds:Bounds3,z:f64,settings:Sampling,runner:&mut Runner<'_>)->Result<(Vec<Fiber>,Vec<String>),CamError> {
    settings.validate()?;
    if !bounds.is_finite() || bounds.is_empty() || !z.is_finite() {return Err(invalid("invalid waterline bounds or level"));}
    let (lo,hi)=match direction {Direction::X=>(bounds.min.y,bounds.max.y),Direction::Y=>(bounds.min.x,bounds.max.x)};
    if hi<=lo {return Err(invalid("empty waterline grid extent"));}
    let count=((hi-lo)/settings.spacing).ceil().max(1.0);
    if count+1.0>settings.max_fibers as f64 {return Err(invalid("waterline fiber budget exceeded"));}
    let seeds:Vec<_>=(0..=count as usize).map(|i|fiber(direction,lo+(hi-lo)*i as f64/count,z,bounds)).collect();
    let initial=push::batch(context,direction,&seeds,runner)?;
    if !settings.adaptive {return Ok((initial,Vec::new()));}
    let mut stack:Vec<_>=initial.windows(2).rev().map(|p|(p[0].clone(),p[1].clone(),0u32)).collect();
    let mut output=vec![initial[0].clone()]; let mut warnings=Vec::new(); let mut evaluations=initial.len();
    while let Some((a,b,depth))=stack.pop() {
        runner.token().check()?;
        if evaluations>=settings.max_fibers {return Err(invalid("adaptive waterline fiber budget exceeded"));}
        let middle=(a.coordinate()+b.coordinate())*0.5;
        let m=context.fiber(&fiber(direction,middle,z,bounds),runner)?; evaluations+=1;
        if !flat(&a,&m,&b,settings.flatness_cos) {
            if b.coordinate()-a.coordinate()>settings.minimum && depth<settings.max_depth {
                stack.push((m.clone(),b,depth+1)); stack.push((a,m,depth+1)); continue;
            }
            warnings.push(format!("unresolved adaptive interval at {}..{}",a.coordinate(),b.coordinate()));
            output.push(m);
        }
        output.push(b);
    }
    output.sort_by(|a,b|a.coordinate().total_cmp(&b.coordinate()));
    Ok((output,warnings))
}

pub fn generate(context:&PushContext<'_>,z:f64,settings:Sampling,runner:&mut Runner<'_>)->Result<Level,CamError> {
    generate_shifted(context,z,settings,0,runner)
}
/// Bounded deterministic grid phases resolve endpoint/grid coincidences without
/// tightening spacing (which makes the same numeric degeneracy more likely).
pub(crate) fn generate_shifted(context:&PushContext<'_>,z:f64,settings:Sampling,phase:u32,runner:&mut Runner<'_>)->Result<Level,CamError> {
    context.validate()?; settings.validate()?;
    if context.mesh.is_empty() {return Err(invalid("waterline has no target triangles"));}
    runner.stage("waterline-level","Generate constant-Z boundary",0,1)?;
    // Unequal deterministic padding avoids coincident CL/grid vertices on
    // symmetric fixtures. Any remaining degeneracy is explicitly refused.
    use crate::CutterProfile;
    let margin=2.0*(context.cutter.radius()+context.allowance)+settings.spacing;
    let mut bounds=context.mesh.bounds.expanded(margin);
    bounds.min.x-=settings.spacing*(0.137+(phase as f64*0.6180339887498949).fract());
    bounds.min.y-=settings.spacing*(0.271+(phase as f64*0.4142135623730951).fract());
    let (x,mut warnings)=sample(context,Direction::X,bounds,z,settings,runner)?;
    let (y,other)=sample(context,Direction::Y,bounds,z,settings,runner)?; warnings.extend(other);
    let (loops,closed)=weave::reconstruct_counted(&x,&y,context.tolerance,settings.close_subgrid_gaps,runner)?;
    if closed>0 {warnings.push(format!("{closed} free gaps narrower than the fiber grid closed at Z={z}"));}
    if loops.is_empty() {warnings.push(format!("no material boundary at Z={z}"));}
    runner.stage("waterline-complete","Constant-Z boundary complete",1,1)?;
    Ok(Level {z,x,y,loops,warnings})
}

