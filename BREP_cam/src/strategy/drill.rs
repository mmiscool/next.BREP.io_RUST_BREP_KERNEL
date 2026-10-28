//! Drilling strategy: equivalent canned and explicit streams, chosen by caller.
use crate::drill::expand_cycle;
use crate::holes::Hole;
use crate::model::*;
use crate::{CamError, Cutter, MeshSnapshot, Runner, Tolerance};

#[derive(Clone,Debug,PartialEq)]
pub struct DrillOutput {pub canned:Toolpath,pub expanded:Toolpath}
fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}

/// Convert a recognized hole and shared operation parameters into one complete
/// cycle. Depth references are tool tip or full-diameter shoulder. Breakthrough
/// is only meaningful for through holes; a blind hole never gains hidden depth.
pub fn cycle_for_hole(params:&DrillParams,hole:&Hole,tool:&Tool,heights:&Heights)->Result<Cycle,CamError> {
    heights.validate().map_err(CamError::InvalidInput)?;
    if !hole.machinable || hole.axis.distance(Point3::Z)>1e-9 || !hole.top.is_finite() || !hole.depth.is_finite() || hole.depth<=0. {
        return Err(invalid("drilling requires a finite hole machinable from setup +Z"));
    }
    let expected=match params.cycle {
        DrillCycle::TapRigid|DrillCycle::TapFloat=>tool.kind==ToolKind::Tap,
        DrillCycle::Ream=>matches!(tool.kind,ToolKind::Reamer|ToolKind::BoringBar),
        DrillCycle::Bore|DrillCycle::BoreFine|DrillCycle::BoreDwell|DrillCycle::BackBore=>tool.kind==ToolKind::BoringBar,
        _=>matches!(tool.kind,ToolKind::Drill|ToolKind::SpotDrill|ToolKind::CenterDrill),
    };
    if !expected {return Err(invalid("tool kind does not match drilling cycle"));}
    let depth=params.depth.unwrap_or(hole.depth);
    if !depth.is_finite() || depth<=0. || !params.break_through.is_finite() || params.break_through<0. || (!hole.through && params.break_through>0.) {
        return Err(invalid("invalid drilling depth or breakthrough on blind hole"));
    }
    let pointed=matches!(tool.kind,ToolKind::Drill|ToolKind::SpotDrill|ToolKind::CenterDrill);
    let shoulder=if !params.tip_depth && pointed {
        let angle=tool.geometry.included_angle_deg.ok_or_else(||invalid("shoulder depth requires drill included angle"))?;
        if !angle.is_finite() || angle<=0. || angle>=180. || !tool.geometry.diameter.is_finite() || tool.geometry.diameter<=0. {return Err(invalid("invalid drill point geometry"));}
        tool.geometry.diameter*0.5/(angle.to_radians()*0.5).tan()
    } else {0.};
    if !tool.geometry.cutting_length.is_finite() || depth+params.break_through+shoulder>tool.geometry.cutting_length {
        return Err(invalid("hole depth exceeds the tool cutting length"));
    }
    let depth_z=hole.top.z-depth-params.break_through-shoulder;
    let retract_z=heights.retract;
    if !depth_z.is_finite() || heights.feed<hole.top.z || retract_z<=hole.top.z || heights.bottom.map_or(false,|z|depth_z<z) {
        return Err(invalid("hole depth or entry violates operation heights"));
    }
    let c=match params.cycle {
        DrillCycle::Drill|DrillCycle::DrillDwell=>Cycle::Drill{depth_z,retract_z,dwell_seconds:if params.cycle==DrillCycle::DrillDwell{params.dwell_seconds}else{0.}},
        DrillCycle::Peck=>Cycle::Peck{depth_z,retract_z,peck:params.peck_depth},
        DrillCycle::ChipBreak=>Cycle::ChipBreak{depth_z,retract_z,peck:params.peck_depth,back_off:params.chip_break_retract},
        DrillCycle::TapRigid|DrillCycle::TapFloat=>Cycle::Tap{depth_z,retract_z,pitch:tool.geometry.thread_pitch.ok_or_else(||invalid("tapping requires tool thread pitch"))?,rigid:params.cycle==DrillCycle::TapRigid,left_hand:tool.geometry.left_hand},
        DrillCycle::Ream=>Cycle::Ream{depth_z,retract_z},
        DrillCycle::Bore|DrillCycle::BoreFine|DrillCycle::BoreDwell=>Cycle::Bore{depth_z,retract_z,dwell_seconds:params.dwell_seconds,shift:if params.cycle==DrillCycle::BoreFine{params.bore_shift}else{0.},feed_out:params.cycle==DrillCycle::BoreDwell},
        DrillCycle::BackBore=>Cycle::BackBore{depth_z,retract_z,entry_z:depth_z-params.back_bore_clearance,shift:params.bore_shift},
    };
    if params.cycle==DrillCycle::BoreFine && params.bore_shift<=0. {return Err(invalid("fine boring requires positive lateral shift"));}
    Ok(c)
}

/// Generate in machine coordinates. Mesh is the complete final part AND
/// protected fixtures (not uncut stock). Every explicit move is sweep checked;
/// the canned payload expands to those same moves. First position is clearance
/// above the first hole; a program assembler must connect the previous operation.
pub fn generate(operation:&Operation,setup:&Setup,tool:&Tool,holes:&[Hole],mesh:&MeshSnapshot,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<DrillOutput,CamError> {
    runner.token().check()?;tolerance.validate()?;
    operation.validate().map_err(CamError::InvalidInput)?;
    let params=match &operation.strategy {Strategy::Drill(p)=>p,_=>return Err(invalid("drill strategy requires Strategy::Drill"))};
    if !operation.enabled || operation.tool!=tool.id || operation.setup!=setup.id || holes.is_empty() {return Err(invalid("drill needs enabled operation, matching setup/tool and holes"));}
    if operation.stock_to_leave.radial!=0. || operation.stock_to_leave.axial!=0. || operation.tool_axis.map_or(false,|a|a.distance(Point3::Z)>tolerance.angle_eps) {return Err(invalid("drilling does not support stock allowance or tilted tool axis"));}
    if mesh.triangles.is_empty() {return Err(invalid("drilling requires the complete protected mesh"));}
    for solid in operation.targets.protected_bodies().iter().map(String::as_str).chain(setup.protected.iter().map(|t|t.solid.as_str())) {
        if !mesh.faces.iter().any(|f|f.solid==solid) {return Err(CamError::UnresolvedTarget{solid:solid.into(),face:None});}
    }
    let heights=operation.heights.as_ref().unwrap_or(&setup.heights);
    let feeds=operation.feeds.as_ref().unwrap_or(&tool.feeds);
    let points:Vec<_>=holes.iter().map(|h|h.top).collect();
    let order=if params.optimise_order {crate::drill::ordering::order(&points,points[0],100_000,runner)?} else {(0..holes.len()).collect()};
    let mut canned=Toolpath::new(operation.id.clone(),tool.id.clone());
    let mut expanded=canned.clone();
    let mut previous:Option<Point3>=None;
    for (path_id,&i) in order.iter().enumerate() {
        runner.work(1,"drill-hole",path_id as u64,holes.len() as u64)?;
        let hole=&holes[i];
        if !operation.targets.target_solids.contains(&hole.solid) || !mesh.faces.iter().any(|f|f.solid==hole.solid && !f.protected_only) {return Err(invalid("hole target missing from protected mesh"));}
        let cycle=cycle_for_hole(params,hole,tool,heights)?;
        let at=Point3::new(hole.top.x,hole.top.y,cycle.retract_z());
        let clear=Point3::new(at.x,at.y,heights.clearance);
        let mut links=Vec::new();
        if let Some(p)=previous {
            let direct=vec![Motion::rapid(p),Motion::link(at,feeds.cutting_feed)];
            let checked=if operation.linking.retract_policy==RetractPolicy::FullRetract {Err(invalid("full retract requested"))}
                else {verify_moves(&direct,tool,mesh,operation.tolerance,tolerance,runner)};
            match checked {
                Ok(())=>links.push(Motion::link(at,feeds.cutting_feed)),
                Err(CamError::Cancelled)=>return Err(CamError::Cancelled),
                Err(_)=>{links.push(Motion::retract(Point3::new(p.x,p.y,heights.clearance),Feed::RAPID));links.push(Motion::link(clear,feeds.cutting_feed));}
            }
        }
        else {links.push(Motion::rapid(clear));}
        links.push(Motion::rapid(at));
        let cycle_moves=expand_cycle(at,&cycle,feeds,runner)?;
        let pid=path_id as u32;
        canned.motions.extend(links.iter().cloned().map(|m|m.with_path(pid)));
        expanded.motions.extend(links.into_iter().map(|m|m.with_path(pid)));
        let feed=if let Cycle::Tap{pitch,..}=cycle {pitch*feeds.spindle_rpm} else {feeds.plunge_feed};
        canned.motions.push(Motion::new(MotionKind::Cycle{at,cycle},Feed::Rate(feed)).with_path(pid));
        expanded.motions.extend(cycle_moves.into_iter().map(|m|m.with_path(pid)));
        previous=Some(at);
    }
    let p=previous.unwrap();let last=Motion::retract(Point3::new(p.x,p.y,heights.clearance),Feed::RAPID);
    canned.motions.push(last.clone());expanded.motions.push(last);
    verify_moves(&expanded.motions,tool,mesh,operation.tolerance,tolerance,runner)?;
    for p in [&mut canned,&mut expanded] {p.report.counts.passes=holes.len() as u64;p.report.counts.motions=p.motions.len() as u64;p.report.finish();}
    runner.token().check()?;Ok(DrillOutput{canned,expanded})
}

/// Conservative swept envelope checks. The point geometry is conical for
/// drills, cylindrical for reamers/taps/boring bars; holder segments are checked
/// separately at their physical heights. Touch tolerance is split radially and
/// axially (one quarter each), keeping the combined deviation below tolerance.
pub(crate) fn verify_moves(moves:&[Motion],tool:&Tool,mesh:&MeshSnapshot,contact_tolerance:f64,tol:Tolerance,runner:&mut Runner<'_>)->Result<(),CamError> {
    use crate::push::PushContext;
    use crate::CutterProfile;
    let g=&tool.geometry;let pad=contact_tolerance*0.25;
    if !pad.is_finite() || contact_tolerance<8.*tol.distance_eps || g.diameter<=2.*pad {return Err(invalid("invalid drilling contact tolerance"));}
    let mut cutter=if matches!(tool.kind,ToolKind::Drill|ToolKind::SpotDrill|ToolKind::CenterDrill) {
        Cutter::cone(g.diameter-2.*pad,g.included_angle_deg.ok_or_else(||invalid("drill needs included angle"))?,g.cutting_length,g.shaft_length)
    } else if matches!(tool.kind,ToolKind::Tap|ToolKind::Reamer|ToolKind::BoringBar) {Cutter::flat(g.diameter-2.*pad,g.cutting_length,g.shaft_length)}
    else {return Err(invalid("tool kind is not supported by drilling"));};
    let shaft=g.shaft_diameter.unwrap_or(g.diameter)*0.5;
    match &mut cutter {Cutter::Cone(c)=>c.shank.shaft_radius=Some(shaft),Cutter::Flat(c)=>c.shank.shaft_radius=Some(shaft),_=>unreachable!()}
    cutter.validate()?;
    let mut bodies=vec![(0.,cutter)];
    if let Some(holder)=&tool.holder {
        let mut z=g.cutting_length+g.shaft_length;
        for s in &holder.segments {
            if !s.lower_diameter.is_finite() || !s.upper_diameter.is_finite() || !s.height.is_finite() || s.lower_diameter<0. || s.upper_diameter<0. || s.height<=0. {return Err(invalid("invalid holder segment"));}
            let diameter=s.lower_diameter.max(s.upper_diameter);
            let c=Cutter::flat(diameter,s.height,0.);c.validate()?;bodies.push((z,c));z+=s.height;
        }
    }
    for (offset,cutter) in bodies {
        let context=PushContext{mesh,cutter:&cutter,index:None,tolerance:tol,allowance:0.};
        let mut previous:Option<Point3>=None;
        for m in moves {
            runner.work(1,"drill-sweep",0,0)?;
            if let Some(mut b)=m.end() {
                b.z+=offset+pad;
                let a=previous.unwrap_or(b);
                if a.distance_xy(b)<=tol.distance_eps {
                    let p=Point3::new(b.x,b.y,a.z.min(b.z));
                    crate::waterline::contour::verify_vertical(&context,p,a.z.max(b.z),runner)?;
                } else if a.z==b.z {
                    if !context.segment(a,b,runner)?.is_empty() {return Err(invalid("drill traverse intersects protected material"));}
                } else {return Err(invalid("drilling refuses diagonal rapid/axial mixed move"));}
                previous=Some(b);
            }
        }
    }
    Ok(())
}
