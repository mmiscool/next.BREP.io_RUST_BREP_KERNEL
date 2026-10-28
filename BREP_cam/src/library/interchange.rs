//! Versioned native JSON, documented Fusion JSON fields and RFC-style CSV.
use super::*;
use serde_json::{Value,json};
use std::collections::BTreeMap;
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(rename_all="camelCase",deny_unknown_fields)]
pub struct LibraryDocument {
    pub format:String,pub version:u32,pub library:ToolLibrary,
    #[serde(default)] pub properties:BTreeMap<String,Value>,
    #[serde(default)] pub tool_properties:BTreeMap<String,BTreeMap<String,Value>>,
}
impl LibraryDocument {
    pub fn new(library:ToolLibrary)->Self {Self {format:"brep-tool-library".into(),version:1,library,properties:BTreeMap::new(),tool_properties:BTreeMap::new()}}
    pub fn validate(&self)->Result<(),String> {
        if self.format!="brep-tool-library"||self.version!=1 {return Err("unsupported tool library format/version".into());}self.library.validate()?;
        if self.tool_properties.keys().any(|id|!self.library.tools.iter().any(|t|t.id.as_str()==id)) {return Err("properties refer to an unknown tool".into());}Ok(())
    }
    pub fn to_json(&self)->Result<String,String> {self.validate()?;serde_json::to_string_pretty(self).map(|s|s+"\n").map_err(|e|e.to_string())}
    pub fn from_json(s:&str)->Result<Self,String> {let v:Self=serde_json::from_str(s).map_err(|e|e.to_string())?;v.validate()?;Ok(v)}
}
#[derive(Clone,Debug,PartialEq)]
pub struct ImportIssue {pub row:usize,pub message:String}
#[derive(Clone,Debug,PartialEq)]
pub struct ImportReport {pub document:LibraryDocument,pub issues:Vec<ImportIssue>}
fn text<'a>(v:&'a Value,key:&str)->Option<&'a str> {v.get(key).and_then(Value::as_str)}
fn num(v:&Value,key:&str)->Result<f64,String> {let n=v.get(key).and_then(Value::as_f64).ok_or_else(||format!("missing/nonnumeric {key}"))?;if !n.is_finite(){return Err(format!("nonfinite {key}"));}Ok(n)}
fn optional(v:&Value,key:&str)->Result<Option<f64>,String> {if v.get(key).is_some() {num(v,key).map(Some)}else{Ok(None)}}
fn unsigned(v:f64,field:&str)->Result<u32,String> {if !v.is_finite()||v<0.||v>u32::MAX as f64||v.fract()!=0. {Err(format!("invalid integer {field}"))}else{Ok(v as u32)}}
fn fusion_scale(v:&Value)->Result<f64,String> {match text(v,"unit") {Some("millimeters")=>Ok(1.),Some("inches")=>Ok(25.4),_=>Err("missing/unsupported Fusion unit".into())}}
fn fusion_tool(v:&Value,row:usize)->Result<Tool,String> {
    let kind=match text(v,"type").ok_or("missing tool type")? {
        "flat end mill"=>ToolKind::Flat,"ball end mill"=>ToolKind::Ball,"bull nose end mill"=>ToolKind::Bull,"chamfer mill"=>ToolKind::Chamfer,"tapered mill"=>if text(v,"tapered-type")==Some("tapered_ball") {ToolKind::TaperedBall}else{ToolKind::TaperedFlat},"tapered ball end mill"=>ToolKind::TaperedBall,"face mill"=>ToolKind::FaceMill,"slot mill"=>ToolKind::Slot,"lollipop mill"=>ToolKind::Lollipop,"dovetail mill"=>ToolKind::Dovetail,"thread mill"=>ToolKind::ThreadMill,"drill"=>ToolKind::Drill,"spot drill"=>ToolKind::SpotDrill,"center drill"=>ToolKind::CenterDrill,"tap right hand"|"tap left hand"=>ToolKind::Tap,"reamer"=>ToolKind::Reamer,"boring bar"=>ToolKind::BoringBar,s=>return Err(format!("unsupported Fusion tool type: {s}")),
    };
    let scale=fusion_scale(v)?;let g=v.get("geometry").ok_or("missing geometry")?;let flute=num(g,"LCF")?*scale;let below=num(g,"LB")?*scale;
    if below<flute {return Err("LB is shorter than flute length".into());}
    let angle=optional(g,"SIG")?.or(optional(g,"TA")?);
    let geometry=ToolGeometry {diameter:num(g,"DC")?*scale,cutting_length:flute,shaft_length:below-flute,shaft_diameter:Some(num(g,"SFDM")?*scale),overall_length:Some(num(g,"OAL")?*scale),corner_radius:optional(g,"RE")?.map(|v|v*scale),included_angle_deg:angle,ball_diameter:optional(g,"ball-diameter")?.or(if kind==ToolKind::TaperedBall {optional(g,"RE")?.map(|r|2.*r)}else{None}).map(|v|v*scale),tip_diameter:optional(g,"tip-diameter")?.map(|v|v*scale),thread_pitch:optional(g,"TP")?.map(|v|v*scale),flute_count:Some(unsigned(num(g,"NOF")?,"NOF")?),left_hand:text(v,"type")==Some("tap left hand")||g.get("HAND").and_then(Value::as_bool)==Some(false)};
    let post=v.get("post-process").ok_or("missing post-process")?;
    let mut tool=Tool::new(text(v,"guid").map(str::to_owned).unwrap_or_else(||format!("fusion-{row}")),unsigned(num(post,"number")?,"number")?,kind,geometry);
    tool.name=text(v,"description").unwrap_or("").into();tool.description=tool.name.clone();tool.material=text(v,"BMC").unwrap_or("").into();
    tool.length_offset=optional(post,"length-offset")?.map(|v|unsigned(v,"length-offset")).transpose()?;tool.diameter_offset=optional(post,"diameter-offset")?.map(|v|unsigned(v,"diameter-offset")).transpose()?;
    if let Some(preset)=v.pointer("/start-values/presets/0") {
        let f=&mut tool.feeds;
        if let Some(n)=optional(preset,"n")? {f.spindle_rpm=n;}
        if let Some(n)=optional(preset,"v_f")? {f.cutting_feed=n*scale;}
        if let Some(n)=optional(preset,"v_f_plunge")? {f.plunge_feed=n*scale;}
        f.ramp_feed=optional(preset,"v_f_ramp")?.map(|n|n*scale);f.lead_feed=optional(preset,"v_f_leadIn")?.map(|n|n*scale);f.feed_per_tooth=optional(preset,"f_z")?.map(|n|n*scale);
        if let Some(mode)=preset.get("use-feed-per-revolution") {
            let mode=mode.as_bool().ok_or("invalid use-feed-per-revolution flag")?;
            if mode {
                f.cutting_feed=num(preset,"f_n")?*scale*f.spindle_rpm;
                if tool.kind.is_hole_tool() {f.plunge_feed=f.cutting_feed;}
                if let Some(n)=optional(preset,"f_n_leadIn")? {f.lead_feed=Some(n*scale*f.spindle_rpm);}
            }
        }
        if tool.kind==ToolKind::Tap {
            // A tap advances by its pitch per spindle revolution; independent
            // feed-per-minute preset values cannot change that relationship.
            f.cutting_feed=f.spindle_rpm*tool.geometry.thread_pitch.ok_or("tap requires pitch")?;
            f.plunge_feed=f.cutting_feed;f.ramp_feed=Some(f.cutting_feed);f.lead_feed=Some(f.cutting_feed);
        }
        tool.coolant=match text(preset,"tool-coolant") {None|Some("disabled")=>Coolant::Off,Some("flood")=>Coolant::Flood,Some("mist")=>Coolant::Mist,Some("tool")=>Coolant::ThroughTool,Some("flood tool")=>Coolant::FloodAndThroughTool,Some("flood mist")=>Coolant::FloodAndMist,Some("air")=>Coolant::Air,Some(x)=>return Err(format!("unsupported coolant: {x}"))};
    }
    tool.feeds.spindle_direction=if tool.geometry.left_hand {SpindleDirection::Ccw}else{SpindleDirection::Cw};
    if let Some(h)=v.get("holder") {
        let hs=if h.get("unit").is_some(){fusion_scale(h)?}else{scale};let segments=h.get("segments").and_then(Value::as_array).ok_or("external/invalid Fusion holder reference")?;
        let mut holder=Holder {id:format!("{}-holder",tool.id).into(),name:text(h,"description").unwrap_or("Imported holder").into(),segments:Vec::new()};
        for s in segments {holder.segments.push(HolderSegment {height:num(s,"height")?*hs,lower_diameter:num(s,"lower-diameter")?*hs,upper_diameter:num(s,"upper-diameter")?*hs});}tool.holder=Some(holder);
    }
    validate_tool(&tool)?;Ok(tool)
}
/// Import valid records, reporting each rejected row. Full source records and
/// root properties are retained so unmodeled fields/presets are not discarded.
pub fn import_fusion(input:&str)->Result<ImportReport,String> {
    let root:Value=serde_json::from_str(input).map_err(|e|e.to_string())?;let data=root.get("data").and_then(Value::as_array).ok_or("Fusion library requires a data array")?;
    let mut report=ImportReport {document:LibraryDocument::new(ToolLibrary::new("Fusion import")),issues:Vec::new()};
    let mut metadata=root.clone();metadata.as_object_mut().ok_or("invalid root")?.remove("data");report.document.properties.insert("fusion".into(),metadata);
    for (i,v) in data.iter().enumerate() {
        match fusion_tool(v,i+1).and_then(|t|{let id=t.id.to_string();report.document.library.insert(t)?;Ok(id)}) {
            Ok(id)=>{report.document.tool_properties.insert(id,BTreeMap::from([("fusion".into(),v.clone())]));},
            Err(message)=>report.issues.push(ImportIssue {row:i+1,message}),
        }
    }Ok(report)
}

/// CSV columns use snake_case model names. Required: id, number, kind,
/// diameter, cutting_length. Quoted fields support commas/newlines/doubled quotes.
pub fn import_csv(input:&str,units:LibraryUnits)->Result<ImportReport,String> {
    let rows=parse_csv(input.trim_start_matches('\u{feff}'))?;let header=rows.first().ok_or("CSV is empty")?;
    if header.iter().collect::<BTreeSet<_>>().len()!=header.len() {return Err("duplicate CSV column".into());}
    for name in ["id","number","kind","diameter","cutting_length"] {if !header.iter().any(|s|s==name) {return Err(format!("missing CSV column {name}"));}}
    let mut report=ImportReport {document:LibraryDocument::new(ToolLibrary::new("CSV import")),issues:Vec::new()};report.document.library.units=units;
    for (i,row) in rows.iter().enumerate().skip(1) {
        let result=(||->Result<(Tool,Value),String> {
            if row.len()!=header.len() {return Err("CSV column count differs from header".into());}
            let cells:BTreeMap<_,_>=header.iter().zip(row).map(|(k,v)|(k.as_str(),v.as_str())).collect();
            let required=|k:&str|->Result<f64,String>{cells.get(k).ok_or_else(||format!("missing {k}"))?.parse().map_err(|_|format!("invalid {k}"))};
            let opt=|k:&str|->Result<Option<f64>,String>{match cells.get(k) {Some(v) if !v.is_empty()=>v.parse().map(Some).map_err(|_|format!("invalid {k}")),_=>Ok(None)}};
            let kind=serde_json::from_value(Value::String(cells["kind"].into())).map_err(|_|"unsupported tool kind")?;
            let mut g=ToolGeometry::flat(required("diameter")?,required("cutting_length")?);g.shaft_length=opt("shaft_length")?.unwrap_or(0.);g.shaft_diameter=opt("shaft_diameter")?;g.overall_length=opt("overall_length")?;g.corner_radius=opt("corner_radius")?;g.included_angle_deg=opt("included_angle_deg")?;g.ball_diameter=opt("ball_diameter")?;g.tip_diameter=opt("tip_diameter")?;g.thread_pitch=opt("thread_pitch")?;g.flute_count=opt("flutes")?.map(|v|unsigned(v,"flutes")).transpose()?;
            let mut t=Tool::new(cells["id"],unsigned(required("number")?,"number")?,kind,g);t.name=cells.get("name").unwrap_or(&"").to_string();t.material=cells.get("material").unwrap_or(&"").to_string();
            if let Some(v)=opt("rpm")? {t.feeds.spindle_rpm=v;}if let Some(v)=opt("cutting_feed")? {t.feeds.cutting_feed=v;}if let Some(v)=opt("plunge_feed")? {t.feeds.plunge_feed=v;}
            validate_tool(&t)?;Ok((t,json!(cells)))
        })();
        match result.and_then(|(t,source)|{let id=t.id.to_string();report.document.library.insert(t)?;Ok((id,source))}) {
            Ok((id,source))=>{report.document.tool_properties.insert(id,BTreeMap::from([("csv".into(),source)]));},Err(message)=>report.issues.push(ImportIssue {row:i+1,message}),
        }
    }Ok(report)
}
fn parse_csv(input:&str)->Result<Vec<Vec<String>>,String> {
    let mut rows=Vec::new();let mut row=Vec::new();let mut field=String::new();let mut quoted=false;let mut closed=false;let mut chars=input.chars().peekable();
    while let Some(c)=chars.next() {
        if quoted {if c=='"' {if chars.peek()==Some(&'"'){chars.next();field.push('"');}else{quoted=false;closed=true;}}else{field.push(c);}continue;}
        match c {
            '"' if field.is_empty()&&!closed=>quoted=true,
            ','=>{row.push(std::mem::take(&mut field));closed=false;},
            '\n'|'\r'=>{if c=='\r'&&chars.peek()==Some(&'\n'){chars.next();}row.push(std::mem::take(&mut field));rows.push(std::mem::take(&mut row));closed=false;},
            _ if closed||c=='"'=>return Err("invalid quoting in CSV".into()),
            _=>field.push(c),
        }
    }
    if quoted {return Err("unterminated CSV quote".into());}
    if !field.is_empty()||!row.is_empty()||closed {row.push(field);rows.push(row);}Ok(rows)
}
