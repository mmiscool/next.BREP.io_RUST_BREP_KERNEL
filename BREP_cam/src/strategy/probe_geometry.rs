//! Expanded in-process inspection of picked nominal face points.
use crate::*;
pub fn generate(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::ProbeGeometry(p)=>p,_=>return Err(CamError::InvalidInput("expected ProbeGeometry strategy".into()))};
    crate::probe::generate(operation,setup,tool,mesh,tol,runner,ProbeKind::Inspection,&p.points,&p.settings,None)
}
