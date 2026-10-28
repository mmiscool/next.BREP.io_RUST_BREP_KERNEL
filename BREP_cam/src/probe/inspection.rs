//! Compare machine-space sphere-bottom trigger coordinates with nominal surfaces.
use crate::*;
use std::collections::BTreeMap;
#[derive(Clone,Debug,PartialEq,serde::Serialize,serde::Deserialize)]
#[serde(rename_all="camelCase")]
pub struct Comparison {
    pub id:String,
    pub measured_surface:Point3,
    pub normal_deviation:f64,
    /// Lateral displacement is reported separately; tolerance is along the normal.
    pub lateral_deviation:f64,
    pub lower_tolerance:f64,
    pub upper_tolerance:f64,
    pub in_tolerance:bool,
}
/// Strict CSV: header `id,x,y,z`, one finite measured trigger per nominal ID.
/// Coordinates use the program's machine frame and units. No WCS is modified.
pub fn compare_csv(nominals:&[ProbeCycle],csv:&str)->Result<Vec<Comparison>,CamError> {
    let error=|s:&str|CamError::InvalidInput(format!("probe results: {s}"));
    if nominals.is_empty(){return Err(error("empty nominal list"));}
    let mut lines=csv.lines();if lines.next().map(str::trim)!=Some("id,x,y,z"){return Err(error("expected id,x,y,z header"));}
    let mut measured=BTreeMap::new();
    for line in lines {let fields:Vec<_>=line.split(',').map(str::trim).collect();if fields.len()!=4||fields[0].is_empty(){return Err(error("malformed measured row"));}let number=|s:&str|s.parse::<f64>().ok().filter(|x|x.is_finite()).ok_or_else(||error("nonfinite or invalid coordinate"));let p=Point3::new(number(fields[1])?,number(fields[2])?,number(fields[3])?);if measured.insert(fields[0].to_string(),p).is_some(){return Err(error("duplicate measured ID"));}}
    let mut out=Vec::new();
    for c in nominals {c.nominal.validate().map_err(CamError::InvalidInput)?;if !c.stylus_radius.is_finite()||c.stylus_radius<=0.{return Err(error("invalid stylus radius"));}
        let tip=measured.remove(&c.nominal.id).ok_or_else(||error("missing measured ID or duplicate nominal ID"))?;let surface=tip.add(Point3::Z.scale(c.stylus_radius)).sub(c.nominal.normal.scale(c.stylus_radius));let delta=surface.sub(c.nominal.position);let deviation=delta.dot(c.nominal.normal);let lateral=delta.sub(c.nominal.normal.scale(deviation)).length();if !deviation.is_finite()||!lateral.is_finite(){return Err(error("coordinate overflow"));}
        out.push(Comparison {id:c.nominal.id.clone(),measured_surface:surface,normal_deviation:deviation,lateral_deviation:lateral,lower_tolerance:c.nominal.lower_tolerance,upper_tolerance:c.nominal.upper_tolerance,in_tolerance:deviation>=c.nominal.lower_tolerance&&deviation<=c.nominal.upper_tolerance});
    }
    if !measured.is_empty(){return Err(error("unexpected measured ID"));}Ok(out)
}
