//! Live tooling on a spindle C axis; all X coordinates are physical radii.
use crate::{CamError, Point3, Tolerance};
use crate::model::{Machine, MachineKind};
use crate::kinematics::Chain;
fn invalid(s:&str)->CamError {CamError::InvalidInput(format!("mill-turn: {s}"))}
/// Use the common C10 chain; a plain XZ lathe is not a live-tool machine.
pub fn chain(machine:&Machine,work_origin:Point3)->Result<Chain,CamError> {
    if !matches!(machine.kind,MachineKind::MillTurn{..}) {return Err(invalid("requires a C-axis live-tool lathe"));}
    Chain::from_machine(machine,work_origin).map_err(|e|invalid(&e.to_string()))
}
/// Check the optional physical Y axis and the common linear travel envelope.
pub fn validate_position(machine:&Machine,p:Point3,t:Tolerance)->Result<(),CamError> {
    t.validate()?;machine.validate().map_err(|e|invalid(&e))?;
    let MachineKind::MillTurn{y_axis,..}=machine.kind else {return Err(invalid("requires a C-axis live-tool lathe"));};
    if !p.is_finite() || (!y_axis && p.y.abs()>t.point_eps) {return Err(invalid("position requires unavailable Y travel"));}
    if let Some(l)=&machine.travel {
        if p.x<l.min.x || p.x>l.max.x || p.y<l.min.y || p.y>l.max.y || p.z<l.min.z || p.z>l.max.z {return Err(invalid("linear travel exceeded"));}
    }
    Ok(())
}

pub mod drilling;

pub mod milling;
