//! Ordered, model-backed tool libraries. No machining profile is invented here.
use crate::*;
use serde::{Serialize,Deserialize};
use std::collections::BTreeSet;
pub mod interchange;
pub mod holders;

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum ProfileSupport {
    Exact,
    /// Reserved for a profile the cutter core explicitly approximates. None of
    /// the currently supported ToolKinds uses an approximate core profile.
    Approximate,
    Unavailable,
}

/// Validate the stored geometry, feeds and holder, then ask the shared cutter
/// engine about machining support. Unavailable profiles remain usable records
/// for turning workflows; callers must not treat them as milling cutters.
pub fn validate_tool(t:&Tool)->Result<ProfileSupport,String> {
    if t.id.as_str().trim().is_empty() {return Err("tool id is empty".into());}
    if t.number==0 {return Err("tool number must be positive".into());}
    let g=&t.geometry;
    for (name,value) in [("diameter",g.diameter),("cutting length",g.cutting_length),("shaft diameter",g.shaft_diameter()),("overall length",g.overall_length())] {
        if !value.is_finite()||value<=0. {return Err(format!("{name} must be finite and positive"));}
    }
    if !g.shaft_length.is_finite()||g.shaft_length<0. {return Err("shaft length must be finite and nonnegative".into());}
    if g.overall_length()<g.cutting_length+g.shaft_length {return Err("overall length is shorter than cutting plus shaft length".into());}
    for (name,value,zero) in [("corner radius",g.corner_radius,true),("ball diameter",g.ball_diameter,false),("tip diameter",g.tip_diameter,true),("thread pitch",g.thread_pitch,false)] {
        if let Some(v)=value {if !v.is_finite()||v<0.||(!zero&&v==0.) {return Err(format!("invalid {name}"));}}
    }
    if g.corner_radius.is_some_and(|v|v>g.diameter/2.)||g.ball_diameter.is_some_and(|v|v>g.diameter)||g.tip_diameter.is_some_and(|v|v>g.diameter) {return Err("end geometry exceeds the cutting diameter".into());}
    if g.included_angle_deg.is_some_and(|v|!v.is_finite()||v<=0.||v>=180.) {return Err("included angle must be between 0 and 180 degrees".into());}
    if g.flute_count==Some(0) {return Err("flute count must be positive".into());}
    if matches!(t.kind,ToolKind::Tap|ToolKind::ThreadMill)&&g.thread_pitch.is_none() {return Err("thread tool requires thread pitch".into());}
    if let Some(h)=&t.holder {h.validate()?;if h.segments.is_empty()||!h.height().is_finite() {return Err("holder profile is empty or its height overflows".into());}}
    t.feeds.validate()?;
    for v in [t.feeds.surface_speed,t.feeds.feed_per_tooth].into_iter().flatten() {if !v.is_finite()||v<=0. {return Err("feed metadata must be finite and positive".into());}}
    match Cutter::from_tool(t) {
        Ok(c)=>{c.validate().map_err(|e|e.to_string())?;Ok(ProfileSupport::Exact)},
        Err(CutterError::UnsupportedToolKind {..})=>Ok(ProfileSupport::Unavailable),
        Err(e)=>Err(e.to_string()),
    }
}

/// Libraries have a single length unit; all geometry and feed distances use it.
#[derive(Clone,Copy,Debug,Default,PartialEq,Eq,Serialize,Deserialize)]
#[serde(rename_all="kebab-case")]
pub enum LibraryUnits {#[default] Millimeters, Inches}
impl LibraryUnits {pub fn millimeters_per_unit(self)->f64 {match self {Self::Millimeters=>1.,Self::Inches=>25.4}}}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(rename_all="camelCase")]
pub struct ToolLibrary {pub name:String,pub units:LibraryUnits,pub tools:Vec<Tool>}
impl ToolLibrary {
    pub fn new(name:impl Into<String>)->Self {Self {name:name.into(),units:LibraryUnits::Millimeters,tools:Vec::new()}}
    pub fn validate(&self)->Result<(),String> {validate_program_tools(&self.tools)}
    pub fn insert(&mut self,tool:Tool)->Result<(),String> {
        validate_tool(&tool)?;
        if self.tools.iter().any(|t|t.id==tool.id||t.number==tool.number) {return Err("duplicate tool id or number".into());}
        self.tools.push(tool);Ok(())
    }
    pub fn replace(&mut self,tool:Tool)->Result<(),String> {
        let i=self.tools.iter().position(|t|t.id==tool.id).ok_or("unknown tool id")?;
        validate_tool(&tool)?;if self.tools.iter().enumerate().any(|(j,t)|j!=i&&t.number==tool.number) {return Err("duplicate tool number".into());}
        self.tools[i]=tool;Ok(())
    }
    /// All-or-nothing sequential renumbering in library order.
    pub fn renumber(&mut self,first:u32,increment:u32)->Result<(),String> {
        if first==0||increment==0 {return Err("numbering starts and increments must be positive".into());}
        let numbers:Result<Vec<_>,_>=(0..self.tools.len()).map(|i|u32::try_from(i).ok().and_then(|i|i.checked_mul(increment)).and_then(|n|first.checked_add(n)).ok_or("tool numbering overflow")).collect();
        for (tool,n) in self.tools.iter_mut().zip(numbers?) {tool.number=n;}Ok(())
    }
    pub fn search(&self,filter:&ToolFilter)->Result<Vec<&Tool>,String> {
        for v in [filter.min_diameter,filter.max_diameter].into_iter().flatten() {if !v.is_finite()||v<0. {return Err("invalid diameter filter".into());}}
        if matches!((filter.min_diameter,filter.max_diameter),(Some(a),Some(b)) if a>b) {return Err("reversed diameter range".into());}
        Ok(self.tools.iter().filter(|t|filter.kind.map_or(true,|k|k==t.kind)&&filter.min_diameter.map_or(true,|v|t.geometry.diameter>=v)&&filter.max_diameter.map_or(true,|v|t.geometry.diameter<=v)&&filter.flutes.map_or(true,|v|t.geometry.flute_count==Some(v))&&filter.material.as_ref().map_or(true,|v|t.material.eq_ignore_ascii_case(v))&&filter.text.as_ref().map_or(true,|v|format!("{} {} {}",t.name,t.description,t.id).to_lowercase().contains(&v.to_lowercase()))).collect())
    }
}
#[derive(Clone,Debug,Default)]
pub struct ToolFilter {pub kind:Option<ToolKind>,pub min_diameter:Option<f64>,pub max_diameter:Option<f64>,pub flutes:Option<u32>,pub material:Option<String>,pub text:Option<String>}
/// Check the actual program tool list, even when assembled from many libraries.
pub fn validate_program_tools(tools:&[Tool])->Result<(),String> {
    let mut ids=BTreeSet::new();let mut numbers=BTreeSet::new();
    for t in tools {validate_tool(t)?;if !ids.insert(&t.id)||!numbers.insert(t.number) {return Err("program contains duplicate tool ids or numbers".into());}}Ok(())
}
