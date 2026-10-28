//! Expanded WCS probe cycles; controller arithmetic belongs to the post.
use crate::*;
pub fn generate(operation:&Operation,setup:&Setup,tool:&Tool,mesh:&MeshSnapshot,tol:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::ProbeWcs(p)=>p,_=>return Err(CamError::InvalidInput("expected ProbeWcs strategy".into()))};
    crate::probe::generate(operation,setup,tool,mesh,tol,runner,p.kind,&p.points,&p.settings,Some(&p.wcs))
}
