//! The sketch being edited: corner fillet and chamfer by point id (the same
//! edits the Fillet corner and Chamfer corner tools make on a click), and a
//! dimension's value (what typing into its label does).
use crate::automation::command::{app_command, parse_args, CommandSpec, Ctx, Outcome};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FilletCornerArgs {
    /// The corner: the id of the point two lines share (`__brepSketch` lists the
    /// sketch's points through the entity list).
    pub point: Value,
    /// The fillet radius (mm).
    pub radius: f64,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ChamferCornerArgs {
    /// The corner: the id of the point two lines share.
    pub point: Value,
    /// The distance from the corner along the first line (mm); along both
    /// when no `angle` is given.
    pub distance: f64,
    /// The cut's angle to the first line in degrees (Distance + angle); omit
    /// for an equal-distance chamfer.
    #[serde(default)]
    pub angle: Option<f64>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SetDimensionArgs {
    /// The dimension's constraint id (`__brepSketch` lists the dimensions).
    pub constraint: Value,
    /// The value as typed into its label: a number or an expression.
    pub value: String,
}

fn corner_result_schema() -> Value {
    json!({"type":"object","properties":{
        "geometryCount":{"type":"integer"},
        "constraintCount":{"type":"integer"}
    },"required":["geometryCount","constraintCount"],"additionalProperties":false})
}

fn counts(ctx: &Ctx<'_>) -> Value {
    let session = ctx.app.docs.engine().sketch_edit_session();
    json!({
        "geometryCount": session.map_or(0, |s| s.doc.geometries.len()),
        "constraintCount": session.map_or(0, |s| s.doc.constraints.len()),
    })
}

fn fillet_corner(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: FilletCornerArgs = parse_args(args)?;
    if !ctx.app.docs.engine().sketch_mode() {
        return Err("no sketch is being edited".to_string());
    }
    ctx.app.docs.engine_mut().sketch_fillet_corner(&a.point, a.radius)?;
    Ok(Outcome::Done(counts(ctx)))
}

fn chamfer_corner(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ChamferCornerArgs = parse_args(args)?;
    if !ctx.app.docs.engine().sketch_mode() {
        return Err("no sketch is being edited".to_string());
    }
    ctx.app.docs.engine_mut().sketch_chamfer_corner(&a.point, a.distance, a.angle)?;
    Ok(Outcome::Done(counts(ctx)))
}

fn set_dimension(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SetDimensionArgs = parse_args(args)?;
    if !ctx.app.docs.engine().sketch_mode() {
        return Err("no sketch is being edited".to_string());
    }
    if !ctx.app.docs.engine_mut().sketch_set_dimension_value(&a.constraint, &a.value) {
        return Err(format!("the dimension {} did not take the value {:?}", a.constraint, a.value));
    }
    Ok(Outcome::Done(counts(ctx)))
}

fn undo(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: crate::automation::command::NoArgs = parse_args(args)?;
    if !ctx.app.docs.engine().sketch_mode() {
        return Err("no sketch is being edited".to_string());
    }
    if !ctx.app.docs.engine_mut().sketch_undo() {
        return Err("nothing to undo in the sketch".to_string());
    }
    Ok(Outcome::Done(counts(ctx)))
}

fn redo(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: crate::automation::command::NoArgs = parse_args(args)?;
    if !ctx.app.docs.engine().sketch_mode() {
        return Err("no sketch is being edited".to_string());
    }
    if !ctx.app.docs.engine_mut().sketch_redo() {
        return Err("nothing to redo in the sketch".to_string());
    }
    Ok(Outcome::Done(counts(ctx)))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("sketch", "fillet_corner", "Round the corner two lines of the edited sketch share (its point id) with a tangent arc of `radius`: one undoable edit, the arc tangent to both lines with a radius dimension; refuses by name (radius too large, parallel lines, no shared corner) and then changes nothing.", FilletCornerArgs, fillet_corner, Mutate, MUTATE_NOWAIT, corner_result_schema),
    app_command!("sketch", "set_dimension", "Set a dimension of the edited sketch (its constraint id) to `value`, a number or an expression, as typing into its label does; one undoable edit, then the sketch re-solves.", SetDimensionArgs, set_dimension, Mutate, MUTATE_NOWAIT, corner_result_schema),
    app_command!("sketch", "undo_edit", "Undo the edited sketch's last edit (the sketch's own history, which Ctrl+Z drives while a sketch is open).", crate::automation::command::NoArgs, undo, Mutate, MUTATE_NOWAIT, corner_result_schema),
    app_command!("sketch", "redo_edit", "Redo the edited sketch's last undone edit.", crate::automation::command::NoArgs, redo, Mutate, MUTATE_NOWAIT, corner_result_schema),
    app_command!("sketch", "chamfer_corner", "Cut the corner two lines of the edited sketch share (its point id) with a straight line: `distance` along both lines, or with `angle` the distance along the first and the cut at that angle to it; one undoable edit with the dimension(s) as constraints; refuses by name and then changes nothing.", ChamferCornerArgs, chamfer_corner, Mutate, MUTATE_NOWAIT, corner_result_schema),
];
