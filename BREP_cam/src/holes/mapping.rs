use super::*;
use crate::{CamError, DrillCycle, DrillParams, Runner, Tool, ToolId, ToolKind};

#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct HoleGroup { pub indices:Vec<usize> }

/// Stable, non-transitive grouping against the first hole in each group.
/// Axis/accessibility/through status are preserved as well as diameter/depth/type.
pub fn group(holes:&[Hole],tolerance:f64,runner:&mut Runner<'_>)->Result<Vec<HoleGroup>,CamError> {
    validate_tolerance(tolerance)?;runner.token().check()?;
    let mut groups:Vec<HoleGroup>=Vec::new();
    for (i,h) in holes.iter().enumerate() {
        validate_hole(h)?;
        let mut matched=None;
        for (j,g) in groups.iter().enumerate() {
            runner.work(1,"hole-group",i as u64,holes.len() as u64)?;
            let a=&holes[g.indices[0]];
            if (a.diameter-h.diameter).abs()<=tolerance && (a.depth-h.depth).abs()<=tolerance && a.kind==h.kind && a.recess==h.recess && a.through==h.through && a.machinable==h.machinable && a.axis.distance(h.axis)<=1e-9 && a.thread_candidate==h.thread_candidate {matched=Some(j);break;}
        }
        if let Some(j)=matched {groups[j].indices.push(i);} else {groups.push(HoleGroup{indices:vec![i]});}
    }
    runner.token().check()?;Ok(groups)
}

/// A caller-supplied thread table (units must match the setup).
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct ThreadSize {pub designation:String,pub pilot_diameter:f64,pub nominal_diameter:f64,pub pitch:f64}

/// PMI, when supplied by a caller, wins. Diameter alone only adds a candidate;
/// ambiguous table matches refuse rather than selecting an arbitrary thread.
pub fn annotate_threads(holes:&mut [Hole],table:&[ThreadSize],tolerance:f64)->Result<(),CamError> {
    validate_tolerance(tolerance)?;
    for s in table {if !s.pilot_diameter.is_finite() || !s.nominal_diameter.is_finite() || !s.pitch.is_finite() || s.pitch<=0. || s.pilot_diameter<=0. || s.nominal_diameter<s.pilot_diameter {return Err(invalid("invalid thread table"));}}
    let mut candidates=Vec::with_capacity(holes.len());
    for h in holes.iter() {
        validate_hole(h)?;
        if h.thread_candidate.as_ref().map_or(false,|t|t.from_pmi) {candidates.push(h.thread_candidate.clone());continue;}
        let matching:Vec<_>=table.iter().filter(|s|(s.pilot_diameter-h.diameter).abs()<=tolerance).collect();
        if matching.len()>1 {return Err(invalid("ambiguous hole thread diameter table"));}
        candidates.push(matching.first().map(|s|ThreadCandidate{designation:s.designation.clone(),pitch:s.pitch,nominal_diameter:s.nominal_diameter,from_pmi:false}));
    }
    for (h,c) in holes.iter_mut().zip(candidates) {h.thread_candidate=c;}
    Ok(())
}

#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct Suggestion {
    pub group:HoleGroup,
    pub tool:ToolId,
    pub params:DrillParams,
    pub requires_confirmation:bool,
    pub notes:Vec<String>,
}

/// Suggest the axial pilot-hole operation for each group. Counterbores,
/// countersinks and spot faces additionally require a feature finishing operation;
/// this is stated in the suggestion, never represented as completed by drilling.
/// Only explicit PMI authorizes a tap suggestion; a diameter candidate does not.
pub fn suggest(holes:&[Hole],tools:&[Tool],tolerance:f64,runner:&mut Runner<'_>)->Result<Vec<Suggestion>,CamError> {
    let groups=group(holes,tolerance,runner)?;let mut result=Vec::new();
    for group in groups {
        let h=&holes[group.indices[0]];
        if !h.machinable {return Err(invalid("hole group is not machinable from setup +Z"));}
        let thread=h.thread_candidate.as_ref().filter(|t|t.from_pmi);
        let mut matching:Vec<_>=tools.iter().filter(|t| {
            let (kind,diameter)=if let Some(th)=thread {(ToolKind::Tap,th.nominal_diameter)} else {(ToolKind::Drill,h.diameter)};
            t.kind==kind && (t.geometry.diameter-diameter).abs()<=tolerance && t.geometry.cutting_length>=h.depth && thread.map_or(true,|th|t.geometry.thread_pitch.map_or(false,|p|(p-th.pitch).abs()<=tolerance))
        }).collect();
        matching.sort_by(|a,b|a.number.cmp(&b.number).then_with(||a.id.as_str().cmp(b.id.as_str())));
        let tool=matching.first().ok_or_else(||invalid("no matching tool with sufficient cutting length for hole group"))?;
        let mut notes=Vec::new();
        if h.thread_candidate.is_some() && thread.is_none() {notes.push("Diameter matches a thread pilot; confirm thread intent before adding tapping.".into());}
        if h.kind!=HoleKind::Plain {notes.push("Pilot drilling only; counterbore/countersink/spot-face finishing remains a separate operation.".into());}
        let params=DrillParams{cycle:if thread.is_some(){DrillCycle::TapRigid}else{DrillCycle::Drill},depth:Some(h.depth),optimise_order:true,tip_depth:false,..Default::default()};
        result.push(Suggestion{group,tool:tool.id.clone(),params,requires_confirmation:true,notes});
    }
    Ok(result)
}

fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}
fn validate_tolerance(t:f64)->Result<(),CamError> {if !t.is_finite() || t<=0. {Err(invalid("hole grouping tolerance must be positive"))} else {Ok(())}}
fn validate_hole(h:&Hole)->Result<(),CamError> {if !h.top.is_finite() || !h.axis.is_finite() || !h.depth.is_finite() || !h.diameter.is_finite() || h.depth<=0. || h.diameter<=0. {Err(invalid("invalid recognized hole"))} else {Ok(())}}


/// Apply an explicit shop rule to distinguish a spot face from a shallow
/// counterbore. Geometry cannot determine machining intent without this rule.
pub fn mark_spot_faces(holes:&mut [Hole],maximum_recess_depth:f64)->Result<(),CamError> {
    validate_tolerance(maximum_recess_depth)?;
    for h in holes {if h.kind==HoleKind::Counterbore && h.recess.as_ref().map_or(false,|r|r.depth<=maximum_recess_depth) {h.kind=HoleKind::SpotFace;}}
    Ok(())

}
