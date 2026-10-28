//! Indexed face/radial hole sections using the common recognized Hole and C8 cycles.
use super::{chain,invalid,validate_position};
use crate::{CamError, MeshSnapshot, Runner, Tolerance};
use crate::holes::Hole;
use crate::model::*;
use crate::drill::strategy::{self,DrillOutput};
use crate::multiaxis::indexed::{IndexedFrame,generate_indexed};

/// Recognition output stays in spindle coordinates. Index C so the hole lies
/// on machine +X; radial outward axes become +X, face axes remain +Z.
/// C8's `machinable=false` for non-Z holes is reassessed in the indexed frame.
pub fn index_hole(hole:&Hole,t:Tolerance)->Result<(f64,Hole),CamError> {
    t.validate()?;
    if !hole.top.is_finite() {return Err(invalid("nonfinite hole location"));}
    let axis=hole.axis.normalized().ok_or_else(||invalid("invalid hole axis"))?;
    let radius=hole.top.x.hypot(hole.top.y);
    let angle=if radius<=t.point_eps {0.} else {-hole.top.y.atan2(hole.top.x).to_degrees()};
    let rotation=RigidTransform::axis_angle(Point3::Z,angle.to_radians(),Point3::ZERO)?;
    let mut h=hole.clone();h.top=rotation.apply_point(hole.top);h.axis=rotation.apply_vector(axis);
    if h.axis.distance(Point3::Z)>t.angle_eps && (radius<=t.point_eps || h.axis.distance(Point3::X)>t.angle_eps) {return Err(invalid("only front-face and outward radial holes are C-indexable"));}
    h.top.y=0.;h.machinable=true;
    Ok((angle,h))
}

/// Generate ONE independently indexed section. `indexed_mesh` is the complete
/// protected mesh at this C index (rotating stock/chuck AND stationary fixtures).
/// The caller must park clear before the initial RotaryPosition; sections must
/// not be concatenated without verified indexing/connection moves. Heights are
/// distances on the tool axis: machine Z for face and machine X for radial.
/// A separate section is returned for each recognized hole rather than claiming
/// an unchecked rotating-fixture link is safe.
pub fn generate_hole(op:&Operation,setup:&Setup,machine:&Machine,tool:&Tool,hole:&Hole,indexed_mesh:&MeshSnapshot,t:Tolerance,runner:&mut Runner<'_>)->Result<DrillOutput,CamError> {
    chain(machine,Point3::ZERO)?;
    let (angle,h)=index_hole(hole,t)?;
    let MachineKind::MillTurn{c_axis,..}=&machine.kind else {return Err(invalid("C axis missing"));};
    if let Some([lo,hi])=c_axis.limits_deg {if angle<lo || angle>hi {return Err(invalid("hole index exceeds C travel"));}}
    if op.feeds.as_ref().unwrap_or(&tool.feeds).css {return Err(invalid("live drilling requires fixed tool spindle rpm"));}
    let axis=if h.axis.distance(Point3::Z)<=t.angle_eps {Point3::Z}else{Point3::X};
    let frame=IndexedFrame::new(axis,Point3::ZERO,&t)?;
    let mut local_h=h.clone();local_h.top=frame.local_of(h.top);local_h.axis=Point3::Z;
    let local_mesh=indexed_mesh.transformed(&frame.to_local);
    let mut local_op=op.clone();local_op.tool_axis=None;
    let local=strategy::generate(&local_op,setup,tool,&[local_h],&local_mesh,t,runner)?;
    let mut indexed_op=op.clone();indexed_op.tool_axis=Some(axis);
    let map=|path:Toolpath|->Result<Toolpath,CamError> {
        let mut out=generate_indexed(&indexed_op,indexed_mesh,t,|_,_|Ok(path))?;
        for m in &out.motions {if let Some(p)=m.end() {validate_position(machine,p,t)?;}}
        out.motions.insert(0,Motion::new(MotionKind::RotaryPosition{axes:vec![RotaryWord{name:"C".into(),angle_deg:angle}]},Feed::RAPID));
        out.report.counts.motions=out.motions.len() as u64;
        Ok(out)
    };
    Ok(DrillOutput{canned:map(local.canned)?,expanded:map(local.expanded)?})
}
