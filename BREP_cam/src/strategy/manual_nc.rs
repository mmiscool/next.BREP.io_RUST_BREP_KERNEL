//! Explicit manual program entries: no inferred tool/spindle/coolant changes.
use crate::{CamError,Runner};
use crate::model::*;
fn invalid(s:&str)->CamError {CamError::InvalidInput(format!("manual NC: {s}"))}
pub fn generate(op:&Operation,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    runner.token().check()?;op.validate().map_err(CamError::InvalidInput)?;
    if !op.enabled {return Err(invalid("operation is disabled"));}
    let Strategy::ManualNc(params)=&op.strategy else {return Err(invalid("requires ManualNc strategy"));};
    let motion=match &params.kind {
        ManualNcKind::Stop=>MotionKind::Stop{optional:false},
        ManualNcKind::OptionalStop=>MotionKind::Stop{optional:true},
        ManualNcKind::Dwell{seconds}=>{if !seconds.is_finite() || *seconds<0. {return Err(invalid("dwell must be finite and nonnegative"));}MotionKind::Dwell{seconds:*seconds}},
        ManualNcKind::Comment{text}|ManualNcKind::PassThrough{text}=>{
            if text.len()>1_048_576 || text.contains('\0') {return Err(invalid("text is too large or contains NUL"));}
            if matches!(params.kind,ManualNcKind::Comment{..}) {MotionKind::Comment{text:text.clone()}} else {MotionKind::PassThrough{text:text.clone()}}
        }
        ManualNcKind::ToolChange{tool}=>{if tool.as_str().is_empty(){return Err(invalid("tool change needs a tool identity"));}MotionKind::ToolChange{tool:tool.clone()}},
    };
    let mut out=Toolpath::new(op.id.clone(),op.tool.clone());out.motions.push(Motion::new(motion,Feed::RAPID));
    out.report.counts.motions=1;out.report.finish();runner.token().check()?;Ok(out)
}
/// Typed entries for the common program stream. The owning assembler must insert
/// these at the requested position without its machining-operation preamble.
/// PassThrough is explicit user input; post dialect handling remains mandatory.
pub fn program_motions(op:&Operation,runner:&mut Runner<'_>)->Result<Vec<ProgramMotion>,CamError> {
    Ok(generate(op,runner)?.motions.into_iter().map(|motion|ProgramMotion{operation:op.id.clone(),motion}).collect())
}
