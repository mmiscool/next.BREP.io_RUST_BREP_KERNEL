//! Typed probing intent. All positions/normals are in MACHINE coordinates.
use super::{Point3,TargetRef,WorkOffset};
use serde::{Deserialize,Serialize};
#[derive(Clone,Copy,Debug,PartialEq,Eq,Serialize,Deserialize)]
#[serde(rename_all="camelCase")]
pub enum ProbeKind {SingleSurface,OutsideCorner,InsideCorner,Web,Pocket,Bore,Boss,Rectangle,Angle,Inspection}
impl ProbeKind {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["singleSurface","outsideCorner","insideCorner","web","pocket","bore","boss","rectangle","angle","inspection"];
}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(rename_all="camelCase")]
pub struct ProbePoint {
    pub id:String,
    pub face:TargetRef,
    pub position:Point3,
    /// Unit normal toward free space (and the approaching stylus).
    pub normal:Point3,
    pub lower_tolerance:f64,
    pub upper_tolerance:f64,
}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(rename_all="camelCase",default)]
pub struct ProbeSettings {
    pub stylus_radius:f64,
    pub shank_radius:f64,
    /// Top of the vertical shank above the stylus centre.
    pub shank_length:f64,
    pub approach_distance:f64,
    /// Maximum travel beyond nominal contact; only executed if no trigger occurs.
    pub overtravel:f64,
    pub feed:f64,
    pub result_prefix:String,
}
impl Default for ProbeSettings {fn default()->Self {Self {stylus_radius:1.,shank_radius:0.25,shank_length:20.,approach_distance:2.,overtravel:0.5,feed:100.,result_prefix:"probe".into()}}}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(rename_all="camelCase",default)]
pub struct ProbeWcsParams {
    /// Serialized as `probeKind`: the strategy's own `kind` tag sits at the same level.
    #[serde(rename="probeKind")]
    pub kind:ProbeKind,
    pub points:Vec<ProbePoint>,pub settings:ProbeSettings,pub wcs:WorkOffset}
impl Default for ProbeWcsParams {fn default()->Self {Self {kind:ProbeKind::SingleSurface,points:Vec::new(),settings:ProbeSettings::default(),wcs:WorkOffset::default()}}}
#[derive(Clone,Debug,PartialEq,Default,Serialize,Deserialize)]
#[serde(rename_all="camelCase",default)]
pub struct ProbeGeometryParams {pub points:Vec<ProbePoint>,pub settings:ProbeSettings}
/// An expanded trigger move, never ordinary cutting. The reference is the
/// bottom of the stylus sphere; expected target = nominal + radius*normal - Z*radius.
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(rename_all="camelCase")]
pub struct ProbeCycle {
    pub target:Point3,
    pub approach:Point3,
    pub nominal:ProbePoint,
    pub stylus_radius:f64,
    pub overtravel:f64,
    pub feed:f64,
    pub kind:ProbeKind,
    pub result_var:String,
    pub point_index:u32,
    pub point_count:u32,
    /// Update intent for the completed cycle; never apply per point.
    pub wcs:Option<WorkOffset>,
}
impl ProbeSettings {
    pub fn validate(&self)->Result<(),String> {
        for v in [self.stylus_radius,self.shank_radius,self.shank_length,self.approach_distance,self.overtravel,self.feed] {if !v.is_finite()||v<=0. {return Err("probe dimensions, travel and feed must be finite and positive".into());}}
        if self.shank_radius>=self.stylus_radius || self.shank_length<=self.stylus_radius || self.result_prefix.is_empty() || !self.result_prefix.bytes().all(|c|c.is_ascii_alphanumeric()||c==b'_') {return Err("invalid probe shank or result prefix".into());}Ok(())
    }
}
impl ProbePoint {
    pub fn validate(&self)->Result<(),String> {
        if self.id.is_empty() || !self.id.bytes().all(|c|c.is_ascii_alphanumeric()||c==b'_') || self.face.solid.is_empty() || !self.position.is_finite() || !self.normal.is_finite() || (self.normal.length()-1.).abs()>1e-6 || !self.lower_tolerance.is_finite() || !self.upper_tolerance.is_finite() || self.lower_tolerance>self.upper_tolerance {return Err("invalid probe nominal point, normal, identity or tolerance".into());}Ok(())
    }
}
impl ProbeWcsParams {pub fn validate(&self)->Result<(),String> {self.settings.validate()?;if self.kind==ProbeKind::Inspection || self.wcs.number==0 || !self.wcs.origin.is_finite() {return Err("invalid probing WCS intent".into());}validate_points(&self.points)}}
impl ProbeGeometryParams {pub fn validate(&self)->Result<(),String> {self.settings.validate()?;validate_points(&self.points)}}
fn validate_points(points:&[ProbePoint])->Result<(),String> {if points.is_empty()||points.len()>10000 {return Err("probe needs 1..10000 nominal points".into());}let mut ids=std::collections::BTreeSet::new();for p in points {p.validate()?;if !ids.insert(&p.id) {return Err("duplicate probe point identity".into());}}Ok(())}
impl ProbeCycle {pub fn is_finite(&self)->bool {self.target.is_finite()&&self.approach.is_finite()&&self.nominal.validate().is_ok()&&self.stylus_radius.is_finite()&&self.stylus_radius>0.&&self.overtravel.is_finite()&&self.overtravel>0.&&self.feed.is_finite()&&self.feed>0.}}
