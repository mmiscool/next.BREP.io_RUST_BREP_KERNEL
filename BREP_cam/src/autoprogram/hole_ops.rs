//! Expand a hole rule into the pilot, recess and explicitly authorised thread.
use super::*;
use crate::{DrillCycle, DrillParams, ThreadMillParams, Point2};
use crate::holes::HoleKind;
use crate::planar::machining::Circle2d;

fn tool<'a>(input: &'a AutoInput<'_>, fits: impl Fn(&Tool)->bool) -> Option<&'a Tool> {
    input.tools.iter().filter(|t| fits(t)).min_by(|a,b|a.number.cmp(&b.number).then(a.id.as_str().cmp(b.id.as_str())))
}

pub(super) fn plan_hole(input:&AutoInput<'_>, feature:&Feature, rule:&OperationRule, machine:&Machine) -> Result<Vec<PlannedOperation>,String> {
    let FeatureKind::Hole(h)=&feature.kind else {return Err("hole sequence requires a recognised hole".into())};
    let mut pilot=h.clone();pilot.thread_candidate=None;pilot.kind=HoleKind::Plain;pilot.recess=None;
    let mut sink=crate::NullSink;let mut runner=Runner::detached(&mut sink);
    let suggestion=crate::holes::suggest(&[pilot.clone()],input.tools,1e-6,&mut runner).map_err(|e|format!("pilot: {e}"))?.remove(0);
    let drill=input.tools.iter().find(|t|t.id==suggestion.tool).unwrap();
    let mut out=Vec::new();
    let make=|suffix:&str,role:&str,t:&Tool,strategy:Strategy,geometry:OperationGeometry,mut notes:Vec<String>| {
        let mut op=Operation::new(format!("{}-{}-{suffix}",feature.id,rule.name),format!("hole {suffix}"),input.setup.id.clone(),t.id.clone(),strategy);
        op.targets=TargetSelection::solids([feature.solid.clone()]);op.tolerance=rule.tolerance;
        op.feeds=feeds_for(t,machine,input,None,&mut notes);
        PlannedOperation{operation:op,tool:t.id.clone(),geometry,feature:feature.id.clone(),role:role.into(),notes}
    };
    let mut params=suggestion.params;params.tip_depth=!h.through;
    let mut notes=Vec::new();
    if !h.through {notes.push("blind pilot: tip stops at the flat floor; full diameter stops short by the drill tip height".into());}
    if h.thread_candidate.as_ref().map_or(false,|t|!t.from_pmi) {notes.push("thread refused: diameter-only candidate needs explicit PMI thread intent".into());}
    if let Some(spot)=tool(input,|t|t.kind==ToolKind::SpotDrill && t.geometry.included_angle_deg.is_some() && t.geometry.diameter>=h.diameter*0.5) {
        let angle=spot.geometry.included_angle_deg.unwrap().to_radians()*0.5;
        let depth=(h.diameter*0.25/angle.tan()).min(h.depth*0.25).min(spot.geometry.cutting_length);
        out.push(make("spot","spot",spot,Strategy::Drill(DrillParams{depth:Some(depth),tip_depth:true,..Default::default()}),OperationGeometry::Holes(vec![pilot.clone()]),vec![]));
    }
    out.push(make("pilot","drill",drill,Strategy::Drill(params),OperationGeometry::Holes(vec![pilot.clone()]),notes));
    if h.kind!=HoleKind::Plain {
        let recess=h.recess.as_ref().ok_or("recess: recognised compound hole has no recess dimensions")?;
        match h.kind {
            HoleKind::Counterbore|HoleKind::SpotFace=>{
                let t=tool(input,|t|t.kind==ToolKind::Flat && t.geometry.diameter+2.*rule.tolerance<recess.diameter && t.geometry.cutting_length>=recess.depth).ok_or("recess: no flat cutter fits the counterbore/spot face")?;
                // Pocket2D supplies the linker's material-top proof for every
                // ring and depth; use it for circular recess regions too.
                let floor=h.top.z-recess.depth;
                let region=Region::new(vec![Loop::circle(Point2::new(h.top.x,h.top.y),recess.diameter*0.5,true)],&Tolerance::default()).map_err(|e|e.to_string())?;
                let params=Pocket2dParams{depths:Depths{top:Some(h.top.z),bottom:Some(floor),step_down:t.geometry.diameter*0.5,..Default::default()},stepover:Stepover{distance:t.geometry.diameter*0.4,direction:CutDirection::Climb},offset_pattern:true,wall_finish:true,keep_islands:true};
                out.push(make("recess","finish",t,Strategy::Pocket2D(params),OperationGeometry::Planar(PlanarInput{regions:vec![PlanarRegion{region,z:floor,upward:true}],stock_top:input.stock_top,..Default::default()}),vec![]));
            }
            HoleKind::Countersink=>{
                let angle=recess.included_angle_deg.ok_or("countersink: missing included angle")?;
                let t=tool(input,|t|t.kind==ToolKind::SpotDrill && t.geometry.diameter>=recess.diameter && t.geometry.included_angle_deg.map_or(false,|a|(a-angle).abs()<1e-6) && t.geometry.tip_diameter.unwrap_or(0.)==0.).ok_or("countersink: no pointed spot drill matches the recognised angle and diameter")?;
                let depth=recess.diameter*0.5/(angle.to_radians()*0.5).tan();
                if depth>h.depth || depth>t.geometry.cutting_length {return Err("countersink: tip would exceed the hole floor or cutting length".into());}
                out.push(make("countersink","drill",t,Strategy::Drill(DrillParams{depth:Some(depth),tip_depth:true,..Default::default()}),OperationGeometry::Holes(vec![pilot.clone()]),vec![]));
            }
            HoleKind::Plain=>unreachable!(),
        }
    }
    if let Some(thread)=h.thread_candidate.as_ref().filter(|t|t.from_pmi) {
        if let Some(t)=tool(input,|t|t.kind==ToolKind::Tap && (t.geometry.diameter-thread.nominal_diameter).abs()<1e-6 && t.geometry.thread_pitch.map_or(false,|p|(p-thread.pitch).abs()<1e-6) && t.geometry.cutting_length>=h.depth && !t.geometry.left_hand) {
            out.push(make("tap","tap",t,Strategy::Drill(DrillParams{cycle:DrillCycle::TapRigid,depth:Some(h.depth),tip_depth:true,..Default::default()}),OperationGeometry::Holes(vec![h.clone()]),vec![]));
        } else {
            let t=tool(input,|t|t.kind==ToolKind::ThreadMill && t.geometry.diameter<h.diameter && t.geometry.cutting_length>=h.depth && t.geometry.thread_pitch.map_or(false,|p|(p-thread.pitch).abs()<1e-6) && !t.geometry.left_hand).ok_or("thread: no matching right-hand tap or full-length thread mill (diameter, pitch, reach)")?;
            let circle=Circle2d::hole(Point2::new(h.top.x,h.top.y),thread.nominal_diameter*0.5,h.top.z,h.top.z-h.depth);
            let params=ThreadMillParams{depths:Depths{top:Some(circle.top),bottom:Some(circle.bottom),..Default::default()},pitch:thread.pitch,diameter:thread.nominal_diameter,..Default::default()};
            out.push(make("thread-mill","tap",t,Strategy::ThreadMill(params),OperationGeometry::Planar(PlanarInput{circles:vec![circle],stock_top:input.stock_top,..Default::default()}),vec!["thread milling intentionally cuts the PMI thread envelope; pilot-wall deviation is not a thread-form oracle".into()]));
        }
    }
    Ok(out)
}
