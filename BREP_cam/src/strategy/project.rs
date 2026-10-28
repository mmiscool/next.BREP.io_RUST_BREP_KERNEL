//! Project machine-XY curves onto the complete protected mesh.
use crate::*;
use crate::drop::{finish::{FinishContext,polyline,invalid},path::Span};

pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Project(p)=>p,_=>return Err(invalid("project generator requires Project parameters"))};
    let curves:Result<Vec<_>,_>=p.curves.iter().map(|p|polyline(p)).collect();
    project_spans(operation,cutter,mesh,index,heights,feeds,tolerance,runner,&curves?)
}

/// Arc-capable entry point for callers that already have shared drop path spans.
/// The persisted ProjectParams currently holds only polyline curves.
pub fn project_spans(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>,curves:&[Vec<Span>])->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Project(p)=>p,_=>return Err(invalid("project generator requires Project parameters"))};
    FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance}.generate(&p.containment,curves,runner)
}
