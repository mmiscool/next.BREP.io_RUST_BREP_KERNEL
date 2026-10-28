//! Original starting values and metric cutting formulas; see the C12 design
//! record for provenance, units and limitations. Not a vendor cutting database.
use crate::*;
use crate::library::{LibraryUnits,validate_tool};
use serde::{Serialize,Deserialize};
#[derive(Clone,Copy,Debug,PartialEq,Eq,Serialize,Deserialize)]
#[serde(rename_all="kebab-case")]
pub enum WorkMaterial {Aluminium,MildSteel,AlloySteel,StainlessSteel,CastIron,Brass,Titanium,Plastics,Wood,Mdf}
#[derive(Clone,Copy,Debug,PartialEq,Serialize,Deserialize)]
pub struct CuttingData {pub surface_speed_m_min:f64,pub chip_load_mm:f64}
#[derive(Clone,Debug,PartialEq)]
pub struct FeedCalculation {pub feeds:FeedsSpeeds,pub requested_rpm:f64,pub requested_chip_load_mm:f64,pub chip_thinning_factor:f64,pub adjustments:Vec<String>}

/// These deliberately simple starting values are original engineering defaults.
/// Users can provide qualified grade/coating data to calculate_with_data instead.
pub fn default_data(tool:&Tool,units:LibraryUnits,material:WorkMaterial)->Result<CuttingData,String> {
    validate_tool(tool)?;
    let (speed,chip)=match material {
        WorkMaterial::Aluminium=>(60.,0.030),WorkMaterial::MildSteel=>(25.,0.020),WorkMaterial::AlloySteel=>(20.,0.015),WorkMaterial::StainlessSteel=>(15.,0.012),WorkMaterial::CastIron=>(20.,0.020),WorkMaterial::Brass=>(50.,0.025),WorkMaterial::Titanium=>(10.,0.010),WorkMaterial::Plastics=>(60.,0.040),WorkMaterial::Wood=>(100.,0.060),WorkMaterial::Mdf=>(80.,0.040),
    };
    let (speed_factor,chip_factor)=match tool.material.to_ascii_lowercase().as_str() {"carbide"|"solid carbide"=>(2.,1.25),"hss"|"high speed steel"=>(1.,1.),_=>return Err("default cutting data requires carbide or HSS tool material".into())};
    let diameter=tool.geometry.diameter*units.millimeters_per_unit();let size=if diameter<=3. {0.5}else if diameter<=10. {1.}else{1.5};
    let kind=match tool.kind {ToolKind::Drill|ToolKind::SpotDrill|ToolKind::CenterDrill|ToolKind::Reamer|ToolKind::BoringBar=>(0.7,0.5),ToolKind::Tap=>(0.3,1.),ToolKind::LatheInsert=>return Err("turning cutting data requires a turning feed-per-revolution contract".into()),ToolKind::Chamfer|ToolKind::VBit|ToolKind::Engraver=>(0.8,0.5),ToolKind::ThreadMill=>(0.7,0.5),_ =>(1.,1.)};
    Ok(CuttingData {surface_speed_m_min:speed*speed_factor*kind.0,chip_load_mm:chip*chip_factor*size*kind.1})
}
pub fn calculate(tool:&Tool,machine:&Machine,units:LibraryUnits,material:WorkMaterial,radial_engagement:Option<f64>)->Result<FeedCalculation,String> {
    if tool.kind==ToolKind::LatheInsert {
        if radial_engagement.is_some() {return Err("radial chip thinning does not apply to turning".into());}
        // This UI/API has no work diameter. Preserve the selected fixed rpm;
        // never mistake insert diameter for work diameter or silently enable CSS.
        let mut out=turning_at_rpm(tool,machine,units,default_turning_data(tool,material)?,tool.feeds.spindle_rpm)?;
        out.adjustments.push("turning uses selected fixed rpm; supply work diameter to calculate_turning_with_data for surface-speed-derived rpm".into());
        return Ok(out);
    }
    calculate_with_data(tool,machine,units,default_data(tool,units,material)?,radial_engagement)
}
/// Surface speed in m/min; chip load in mm/tooth; engagement and machine feeds
/// in the supplied model units. Informational surface_speed stays in m/min.
pub fn calculate_with_data(tool:&Tool,machine:&Machine,units:LibraryUnits,data:CuttingData,radial_engagement:Option<f64>)->Result<FeedCalculation,String> {
    validate_tool(tool)?;machine.validate()?;
    for v in [machine.spindle.min_rpm,machine.spindle.max_rpm,machine.feed.max_cutting,machine.feed.rapid,data.surface_speed_m_min,data.chip_load_mm] {if !v.is_finite()||v<0. {return Err("cutting data and machine limits must be finite and nonnegative".into());}}
    if data.surface_speed_m_min==0.||data.chip_load_mm==0.||machine.spindle.max_rpm==0. {return Err("cutting speed, chip load and maximum rpm must be positive".into());}
    if tool.kind==ToolKind::LatheInsert {return Err("turning requires feed per revolution".into());}
    let scale=units.millimeters_per_unit();let diameter=tool.geometry.diameter*scale;
    let requested_rpm=1000.*data.surface_speed_m_min/(std::f64::consts::PI*diameter);
    if !requested_rpm.is_finite()||requested_rpm<=0. {return Err("cutting speed calculation overflow".into());}
    let mut rpm=requested_rpm.clamp(machine.spindle.min_rpm,machine.spindle.max_rpm);let mut adjustments=Vec::new();
    if rpm!=requested_rpm {adjustments.push("spindle speed clamped to machine limits".into());}
    let mut factor=1.;
    if let Some(a)=radial_engagement {
        if !a.is_finite()||a<=0.||a>tool.geometry.diameter {return Err("radial engagement must be in (0, diameter]".into());}
        if tool.kind.is_hole_tool() {return Err("radial chip thinning does not apply to hole-making cycles".into());}
        let ratio=a/tool.geometry.diameter;
        if ratio<0.5 {factor=1./(2.*(ratio*(1.-ratio)).sqrt());if factor>4. {factor=4.;adjustments.push("radial chip thinning capped at four times base chip load".into());}}
    }
    let teeth=tool.geometry.flute_count.ok_or("feed calculation requires flute count")? as f64;
    let mut feed=if tool.kind==ToolKind::Tap {rpm*tool.geometry.thread_pitch.unwrap()}else{rpm*teeth*data.chip_load_mm/scale*factor};
    if !feed.is_finite()||feed<=0. {return Err("feed calculation overflow".into());}
    if feed>machine.feed.max_cutting {
        if tool.kind==ToolKind::Tap {
            rpm=machine.feed.max_cutting/tool.geometry.thread_pitch.unwrap();
            if rpm<machine.spindle.min_rpm||rpm<=0. {return Err("machine cannot maintain tapping pitch within spindle/feed limits".into());}
            feed=rpm*tool.geometry.thread_pitch.unwrap();adjustments.push("tapping rpm reduced to preserve pitch at machine feed limit".into());
        }else {feed=machine.feed.max_cutting;adjustments.push("cutting feed clamped to machine limit; achieved chip load reduced".into());}
    }
    let cycle=tool.kind.is_hole_tool();let feeds=FeedsSpeeds {spindle_rpm:rpm,spindle_direction:if tool.geometry.left_hand {SpindleDirection::Ccw}else{SpindleDirection::Cw},cutting_feed:feed,plunge_feed:if cycle {feed}else{feed*0.25},lead_feed:Some(if cycle {feed}else{feed*0.5}),ramp_feed:Some(if cycle {feed}else{feed*0.3}),retract_feed:None,surface_speed:Some(std::f64::consts::PI*diameter*rpm/1000.),feed_per_tooth:Some(feed/(rpm*teeth)),surface_speed_limit_rpm:None,css:false};
    feeds.validate()?;Ok(FeedCalculation {feeds,requested_rpm,requested_chip_load_mm:data.chip_load_mm,chip_thinning_factor:factor,adjustments})
}
/// Turning data is deliberately distinct from milling chip load: mm/revolution,
/// with no flute multiplier. Original starting values, not vendor recommendations.
#[derive(Clone,Copy,Debug,PartialEq,Serialize,Deserialize)]
pub struct TurningCuttingData {pub surface_speed_m_min:f64,pub feed_per_revolution_mm:f64}
pub fn default_turning_data(tool:&Tool,material:WorkMaterial)->Result<TurningCuttingData,String> {
    validate_tool(tool)?;
    if tool.kind!=ToolKind::LatheInsert {return Err("turning cutting data requires a lathe insert".into());}
    let (vc,feed)=match material {
        WorkMaterial::Aluminium=>(120.,0.12),WorkMaterial::MildSteel=>(80.,0.10),
        WorkMaterial::AlloySteel=>(60.,0.08),WorkMaterial::StainlessSteel=>(45.,0.06),
        WorkMaterial::CastIron=>(70.,0.10),WorkMaterial::Brass=>(100.,0.10),
        WorkMaterial::Titanium=>(30.,0.05),WorkMaterial::Plastics=>(80.,0.08),
        WorkMaterial::Wood=>(100.,0.10),WorkMaterial::Mdf=>(80.,0.08),
    };
    let factor=match tool.material.to_ascii_lowercase().as_str() {
        "carbide"|"solid carbide"=>1.,"hss"|"high speed steel"=>0.5,
        _=>return Err("default turning data requires carbide or HSS insert material".into()),
    };
    Ok(TurningCuttingData{surface_speed_m_min:vc*factor,feed_per_revolution_mm:feed*factor})
}
/// Fixed-rpm calculation at an explicit work diameter in model units. The
/// returned per-minute feed preserves f/rev at the achieved clamped rpm.
pub fn calculate_turning_with_data(tool:&Tool,machine:&Machine,units:LibraryUnits,work_diameter:f64,data:TurningCuttingData)->Result<FeedCalculation,String> {
    if !work_diameter.is_finite() || work_diameter<=0. {return Err("turning work diameter must be finite and positive".into());}
    let diameter=work_diameter*units.millimeters_per_unit();
    let rpm=1000.*data.surface_speed_m_min/(std::f64::consts::PI*diameter);
    let mut out=turning_at_rpm(tool,machine,units,data,rpm)?;
    out.feeds.surface_speed=Some(std::f64::consts::PI*diameter*out.feeds.spindle_rpm/1000.);
    Ok(out)
}
fn turning_at_rpm(tool:&Tool,machine:&Machine,units:LibraryUnits,data:TurningCuttingData,requested_rpm:f64)->Result<FeedCalculation,String> {
    validate_tool(tool)?;machine.validate()?;
    if tool.kind!=ToolKind::LatheInsert {return Err("turning feed calculation requires a lathe insert".into());}
    for v in [data.surface_speed_m_min,data.feed_per_revolution_mm,requested_rpm,machine.spindle.max_rpm,machine.feed.max_cutting] {
        if !v.is_finite() || v<=0. {return Err("turning cutting data, rpm and limits must be finite and positive".into());}
    }
    let per_rev=data.feed_per_revolution_mm/units.millimeters_per_unit();
    let max_rpm=machine.spindle.max_rpm.min(machine.feed.max_cutting/per_rev);
    if max_rpm<machine.spindle.min_rpm || max_rpm<=0. {return Err("machine cannot maintain turning feed per revolution within spindle/feed limits".into());}
    let rpm=requested_rpm.clamp(machine.spindle.min_rpm,max_rpm);let feed=rpm*per_rev;
    if !feed.is_finite() || feed<=0. {return Err("turning feed calculation overflow".into());}
    let mut adjustments=Vec::new();if rpm!=requested_rpm {adjustments.push("turning rpm clamped to spindle/feed limits while preserving feed per revolution".into());}
    let feeds=FeedsSpeeds{spindle_rpm:rpm,spindle_direction:tool.feeds.spindle_direction,cutting_feed:feed,plunge_feed:feed,lead_feed:Some(feed),ramp_feed:Some(feed),retract_feed:None,surface_speed:None,feed_per_tooth:None,surface_speed_limit_rpm:None,css:false};
    feeds.validate()?;
    // No milling chip load is implied by a turning calculation.
    Ok(FeedCalculation{feeds,requested_rpm,requested_chip_load_mm:0.,chip_thinning_factor:1.,adjustments})
}
