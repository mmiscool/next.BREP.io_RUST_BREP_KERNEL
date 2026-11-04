//! Drawing sheets: the paper, what is placed on it, and the projection.
//!
//! A sheet places SAVED PMI views — capture one with `pmi_capture_view` first
//! — and carries DIMENSIONS of its own, anchored to those placements'
//! projected geometry rather than to any saved annotation. Neither adds
//! geometry to the model, so none of these commands re-runs the
//! history. They are the operations `panels::sheets` and the sheet viewport
//! perform, on the same engine methods, so an agent authors the same document
//! a person does. The sheet is exported through `document_export` with format
//! `sheet_svg`, beside every other output format.
use crate::automation::cmd_document::parse;
use crate::automation::command::{app_command, engine_call, read_engine, parse_args, schema_of, CommandSpec, Ctx, NoArgs, Outcome};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddSheetArgs {
    /// The sheet's name; a default (`Sheet N`) is minted when absent.
    #[serde(default)]
    pub name: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SheetIdArgs {
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpenSheetArgs {
    /// The sheet the viewport draws; absent CLOSES the sheet and returns the
    /// viewport to the 3D model.
    #[serde(default)]
    pub id: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateSheetArgs {
    pub id: String,
    /// The sheet's params as `sheet_catalogue`'s first schema defines them
    /// (`name`, `size`, `widthMm`, `heightMm`, `border`, `borderInsetMm`,
    /// `titleBlock`, `notes`, and `revisions` — the whole list of
    /// `{rev, date, description}` rows, in table order); absent keys are
    /// unchanged.
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PlaceViewArgs {
    /// The sheet to place on; the OPEN sheet (else the first) when absent.
    #[serde(default)]
    pub sheet_id: Option<String>,
    /// The saved PMI view's id. Its camera must be orthographic.
    pub view_id: String,
    /// Millimetres from the paper's top-left corner; the middle of the paper
    /// when absent.
    #[serde(default)]
    pub position: Option<[f64; 2]>,
    /// Paper millimetres per model unit (default 1).
    #[serde(default)]
    pub scale: Option<f64>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdatePlacedViewArgs {
    pub id: String,
    /// The placement's params as `sheet_catalogue`'s second schema defines
    /// them (`view`, `positionXMm`, `positionYMm`, `scale`, `projection`).
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MovePlacedViewArgs {
    pub id: String,
    /// The new position in paper millimetres.
    pub position: [f64; 2],
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddDimensionArgs {
    /// The sheet to dimension on; the OPEN sheet (else the first) when absent.
    #[serde(default)]
    pub sheet_id: Option<String>,
    /// `linear`, `radial` or `diametral`.
    pub kind: String,
    /// A linear dimension's measured direction: `horizontal`, `vertical` or
    /// `aligned` (the default).
    #[serde(default)]
    pub alignment: Option<String>,
    /// The anchor references — two for a linear dimension, one for a radial or
    /// diametral one. `{placement}:{kind}:{solid}#{entity}`, an edge's fraction
    /// after an `@`: `SV2:vertex:Box#1`, `SV2:edge:Box#Box_NX|Box_NZ[0]@0.5`,
    /// `SV2:circle:Pin#Pin_S|Pin_T[0]`. An empty list is allowed and gives an
    /// unresolved dimension whose anchors are picked afterwards.
    #[serde(default)]
    pub anchors: Vec<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateDimensionArgs {
    pub id: String,
    /// The dimension's params as `sheet_catalogue`'s third schema defines them
    /// (`kind`, `alignment`, `anchors`, `offsetMm`, `precision`, and the
    /// tolerance block `tolMode`, `tolUpper`, `tolLower`, `isReference`);
    /// absent keys are unchanged.
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MoveDimensionArgs {
    pub id: String,
    /// How far the dimension line sits from its anchors, paper millimetres.
    /// Signed: the sign picks the side.
    pub offset_mm: f64,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddOrdinateArgs {
    /// The sheet to draw the set on; the OPEN sheet (else the first) when absent.
    #[serde(default)]
    pub sheet_id: Option<String>,
    /// `horizontal` or `vertical` \u{2014} which paper component the values read.
    pub axis: String,
    /// The anchor every member is measured FROM. Absent gives a set with no
    /// datum, which draws nothing and says so.
    #[serde(default)]
    pub datum: Option<String>,
    /// The measured anchors, in reading order.
    #[serde(default)]
    pub members: Vec<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateOrdinateArgs {
    pub id: String,
    /// The set's params as `sheet_catalogue`'s fourth schema defines them
    /// (`axis`, `datum`, `members`, `offsetMm`, `precision`); absent keys are
    /// unchanged. A set with no `datum` is kept, unresolved.
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MoveOrdinateArgs {
    pub id: String,
    /// The baseline's signed distance from the datum, paper millimetres.
    pub offset_mm: f64,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NewDerivedArgs {
    /// The sheet to put it on; the OPEN sheet (else the first) when absent.
    #[serde(default)]
    pub sheet_id: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AnchorPickArgs {
    /// The anchor, `{placement}:{kind}:{solid}#{entity}` \u{2014} one of the
    /// open sheet's `drawing.views[].anchors[].ref`.
    pub anchor: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddSectionArgs {
    /// The placement the cutting line is drawn ON \u{2014} whose camera turns a
    /// paper line into a plane. Both anchors must belong to it.
    pub source: String,
    /// The two anchors the cutting line runs through.
    pub from: String,
    pub to: String,
    /// Where the section view sits, paper millimetres. Beside its source when
    /// absent.
    #[serde(default)]
    pub position: Option<[f64; 2]>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddRevisionArgs {
    /// The sheet whose revision table gains the row.
    pub id: String,
    /// The revision's letter; the one after the last row's (`A` on an empty
    /// table) when absent.
    #[serde(default)]
    pub rev: Option<String>,
    /// Its date; today's (`YYYY-MM-DD`, UTC) when absent.
    #[serde(default)]
    pub date: Option<String>,
    /// What changed; empty when absent.
    #[serde(default)]
    pub description: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateRevisionArgs {
    pub id: String,
    /// The row, counted from 0 in the table's own order.
    pub index: usize,
    /// `rev`, `date` and / or `description`; absent keys are unchanged.
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RevisionRowArgs {
    pub id: String,
    /// The row, counted from 0 in the table's own order.
    pub index: usize,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct RevisionAdded {
    /// The new row's index — the last.
    pub index: usize,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddDetailArgs {
    /// The placement the detail circle is drawn ON \u{2014} whose camera the detail
    /// looks through. Both anchors must belong to it, and it must be a plain
    /// placement: a detail of a section or of a detail is refused.
    pub source: String,
    /// The anchor at the circle's centre.
    pub centre: String,
    /// An anchor on the circle's rim; its distance from the centre on the
    /// source's paper, over the source's scale, is the radius in model units.
    pub rim: String,
    /// Where the detail view sits \u{2014} the paper point its circle's centre
    /// lands on, in millimetres. Beside the source when absent.
    #[serde(default)]
    pub position: Option<[f64; 2]>,
    /// Paper millimetres per model unit; twice the source's when absent.
    #[serde(default)]
    pub scale: Option<f64>,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct SheetAdded {
    /// The id the engine minted.
    pub id: String,
}

fn sheet_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(parse(&ctx.app.docs.engine_mut().sheet_state_json())))
}

fn sheet_catalogue(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    read_engine(ctx, args, |e| e.sheet_catalogue())
}

fn sheet_add(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddSheetArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().sheet_add(a.name.as_deref());
    serde_json::to_value(SheetAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_update(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: UpdateSheetArgs| e.sheet_update(&a.id, &a.params.to_string()))
}

fn sheet_delete(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: SheetIdArgs| e.sheet_delete(&a.id))
}

fn sheet_open(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: OpenSheetArgs| e.sheet_set_open(a.id.as_deref()))
}

fn sheet_place_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PlaceViewArgs = parse_args(args)?;
    let id = ctx
        .app
        .docs
        .engine_mut()
        .sheet_place_view(a.sheet_id.as_deref(), &a.view_id, a.position, a.scale)?;
    serde_json::to_value(SheetAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_update_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: UpdatePlacedViewArgs| e.sheet_update_view(&a.id, &a.params.to_string()))
}

fn sheet_move_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: MovePlacedViewArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    // A scripted move is ONE placement, so it is its own undo step: the
    // coalescing lane belongs to the pointer drag that emits a move per frame.
    engine.sheet_move_view(&a.id, a.position, false)?;
    engine.sheet_move_view_end();
    Ok(Outcome::Done(json!({})))
}

fn sheet_remove_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: SheetIdArgs| e.sheet_remove_view(&a.id))
}

fn sheet_add_dimension(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddDimensionArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().sheet_add_dimension(
        a.sheet_id.as_deref(),
        &a.kind,
        a.alignment.as_deref(),
        a.anchors,
    )?;
    serde_json::to_value(SheetAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_update_dimension(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: UpdateDimensionArgs| e.sheet_update_dimension(&a.id, &a.params.to_string()))
}

fn sheet_move_dimension(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: MoveDimensionArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    // A scripted move is ONE placement, so it is its own undo step: the
    // coalescing lane belongs to the pointer drag that emits a move per frame.
    engine.sheet_move_dimension(&a.id, a.offset_mm, false)?;
    engine.sheet_move_dimension_end();
    Ok(Outcome::Done(json!({})))
}

fn sheet_remove_dimension(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: SheetIdArgs| e.sheet_remove_dimension(&a.id))
}

fn sheet_add_ordinate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddOrdinateArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().sheet_add_ordinate(
        a.sheet_id.as_deref(),
        &a.axis,
        a.datum.as_deref(),
        a.members,
    )?;
    serde_json::to_value(SheetAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_update_ordinate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: UpdateOrdinateArgs| e.sheet_update_ordinate(&a.id, &a.params.to_string()))
}

fn sheet_move_ordinate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: MoveOrdinateArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    // A scripted move is ONE placement and its own undo step, the dimension
    // move's rule: coalescing belongs to the drag that emits one per frame.
    engine.sheet_move_ordinate(&a.id, a.offset_mm, false)?;
    engine.sheet_move_ordinate_end();
    Ok(Outcome::Done(json!({})))
}

fn sheet_remove_ordinate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: SheetIdArgs| e.sheet_remove_ordinate(&a.id))
}

fn sheet_add_section(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddSectionArgs = parse_args(args)?;
    let id = ctx
        .app
        .docs
        .engine_mut()
        .sheet_add_section(&a.source, &a.from, &a.to, a.position)?;
    serde_json::to_value(SheetAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_add_revision(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddRevisionArgs = parse_args(args)?;
    let index = ctx.app.docs.engine_mut().sheet_add_revision(
        &a.id,
        a.rev.as_deref(),
        a.date.as_deref(),
        a.description.as_deref(),
    )?;
    serde_json::to_value(RevisionAdded { index }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_update_revision(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: UpdateRevisionArgs| e.sheet_update_revision(&a.id, a.index, &a.params.to_string()))
}

fn sheet_remove_revision(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: RevisionRowArgs| e.sheet_remove_revision(&a.id, a.index))
}

fn sheet_add_detail(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddDetailArgs = parse_args(args)?;
    let id = ctx
        .app
        .docs
        .engine_mut()
        .sheet_add_detail(&a.source, &a.centre, &a.rim, a.position, a.scale)?;
    serde_json::to_value(SheetAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_new_section(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NewDerivedArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().sheet_new_section(a.sheet_id.as_deref())?;
    serde_json::to_value(SheetAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_new_detail(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NewDerivedArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().sheet_new_detail(a.sheet_id.as_deref())?;
    serde_json::to_value(SheetAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn sheet_anchor_pick(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AnchorPickArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().ref_select_pick_sheet_anchor(&a.anchor)?;
    Ok(Outcome::Done(serde_json::json!({})))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("sheets", "sheet_state", "The document's drawing sheets, which one the viewport is showing, which object's form is open (`openObject`) and which is selected (`selectedObject`), and the OPEN sheet's projection \u{2014} the paper in millimetres, each placed view's visible edge runs, EVERY annotation of the view it places, the anchor candidates a sheet mark can be placed on, and the sheet's own dimensions and ordinate sets.", NoArgs, sheet_state, Read, READ, super::sheet_output_schema::sheet_state_schema),
    app_command!("sheets", "sheet_catalogue", "The parameter schemas of a sheet, of a placed view, of a sheet dimension, of an ordinate set, of a section view and of a detail view \u{2014} what `sheet_update`, `sheet_update_view` (all three kinds of placement), `sheet_update_dimension` and `sheet_update_ordinate` take. The placed view's `view` field offers this document's saved PMI views.", NoArgs, sheet_catalogue, Read, READ, super::catalogue_output_schema::sheet_catalogue_schema),
    app_command!("sheets", "sheet_add", "Add an A3 drawing sheet and open it in the sheet viewport. Returns the sheet id.", AddSheetArgs, sheet_add, Mutate, MUTATE, schema_of::<SheetAdded>),
    app_command!("sheets", "sheet_update", "Patch a sheet's name, paper size (a preset, or Custom with `widthMm` / `heightMm`), border, title block, notes, or its whole `revisions` list (rows of `rev`, `date`, `description`, in table order); absent keys are unchanged.", UpdateSheetArgs, sheet_update, Mutate, MUTATE),
    app_command!("sheets", "sheet_add_revision", "Append a row to a sheet's REVISION TABLE \u{2014} what the sheet dialog's `+ Add revision` does. Absent fields are seeded: the letter after the last row's (`A` on an empty table), today's date, no description. The sheet draws its rows as a table beside the title block, in the order they were added. Returns the row's index.", AddRevisionArgs, sheet_add_revision, Mutate, MUTATE, schema_of::<RevisionAdded>),
    app_command!("sheets", "sheet_update_revision", "Patch one revision row's `rev`, `date` or `description` \u{2014} what typing into the sheet dialog's Revisions section does; absent keys are unchanged.", UpdateRevisionArgs, sheet_update_revision, Mutate, MUTATE),
    app_command!("sheets", "sheet_remove_revision", "Remove one revision row; the rows after it move up and keep their order. A sheet whose last row goes draws no revision table.", RevisionRowArgs, sheet_remove_revision, Mutate, DESTRUCTIVE),
    app_command!("sheets", "sheet_delete", "Delete a sheet and every view placed on it.", SheetIdArgs, sheet_delete, Mutate, DESTRUCTIVE),
    app_command!("sheets", "sheet_open", "Draw a sheet in the viewport instead of the 3D model; with no `id`, close the sheet and return to the 3D view.", OpenSheetArgs, sheet_open, Mutate, MUTATE),
    app_command!("sheets", "sheet_place_view", "Place a saved PMI view on a sheet (the open one when `sheet_id` is absent) at a position in paper millimetres and a scale in paper millimetres per model unit. The view's camera must be orthographic. Returns the placement id.", PlaceViewArgs, sheet_place_view, Mutate, MUTATE, schema_of::<SheetAdded>),
    app_command!("sheets", "sheet_update_view", "Replace a placed view's params: which saved view it draws, where it sits on the paper, its scale.", UpdatePlacedViewArgs, sheet_update_view, Mutate, MUTATE),
    app_command!("sheets", "sheet_move_view", "Move a placed view to a position in paper millimetres \u{2014} what dragging it in the sheet viewport does.", MovePlacedViewArgs, sheet_move_view, Mutate, MUTATE),
    app_command!("sheets", "sheet_remove_view", "Take one placed view off its sheet. Any sheet dimension anchored to it STAYS, unresolved with the reason \u{2014} deleting the sheet is what takes them.", SheetIdArgs, sheet_remove_view, Mutate, DESTRUCTIVE),
    app_command!("sheets", "sheet_add_dimension", "Add a dimension drawn ON the sheet, anchored to a placement's projected geometry. LINEAR takes two anchors and measures the MODEL distance between them in the view's projection plane, in model units at any sheet scale; RADIAL and DIAMETRAL take one CIRCLE anchor and read the model radius. An anchor is `{placement}:{kind}:{solid}#{entity}` \u{2014} `sheet_state` lists every one a placement offers under `drawing.views[].anchors`. Returns the dimension id.", AddDimensionArgs, sheet_add_dimension, Mutate, MUTATE, schema_of::<SheetAdded>),
    app_command!("sheets", "sheet_update_dimension", "Patch a sheet dimension's kind, alignment, anchors, offset, precision or TOLERANCE BLOCK \u{2014} the PMI dimension's own keys: `tolMode` (none / symmetric / deviation / limits), `tolUpper`, `tolLower` and `isReference` \u{2014} written with the kernel's own dimension text; absent keys are unchanged. A dimension with no block of its own whose anchors are the points a free linear or a radial PMI dimension of the placement's view measures writes that dimension's block, and its drawing names it as `toleranceFrom`.", UpdateDimensionArgs, sheet_update_dimension, Mutate, MUTATE),
    app_command!("sheets", "sheet_move_dimension", "Move a sheet dimension's line: how far it sits from its anchors in paper millimetres, signed \u{2014} what dragging its value box in the sheet viewport does.", MoveDimensionArgs, sheet_move_dimension, Mutate, MUTATE),
    app_command!("sheets", "sheet_remove_dimension", "Take one dimension off its sheet.", SheetIdArgs, sheet_remove_dimension, Mutate, DESTRUCTIVE),
    app_command!("sheets", "sheet_add_ordinate", "Add an ORDINATE SET drawn on the sheet: a datum anchor and any number of members, each reading its signed distance from the datum along one paper axis (`horizontal` reads x, right positive; `vertical` reads UP the page). Anchors are the same `{placement}:{kind}:{solid}#{entity}` references a sheet dimension takes. Returns the set id.", AddOrdinateArgs, sheet_add_ordinate, Mutate, MUTATE, schema_of::<SheetAdded>),
    app_command!("sheets", "sheet_update_ordinate", "Patch an ordinate set's axis, datum, members, baseline offset, precision or tolerance block (`tolMode`, `tolUpper`, `tolLower`, `isReference`, written on every member's value and never on the datum's); absent keys are unchanged. Clearing the DATUM keeps the set, unresolved: every value it draws is a distance from the datum, so until one is picked again it draws its id and the reason.", UpdateOrdinateArgs, sheet_update_ordinate, Mutate, MUTATE),
    app_command!("sheets", "sheet_move_ordinate", "Move an ordinate set's BASELINE: its signed distance from the datum in paper millimetres \u{2014} what dragging the datum's value box in the sheet viewport does. One number moves the whole run.", MoveOrdinateArgs, sheet_move_ordinate, Mutate, MUTATE),
    app_command!("sheets", "sheet_remove_ordinate", "Take one ordinate set off its sheet.", SheetIdArgs, sheet_remove_ordinate, Mutate, DESTRUCTIVE),
    app_command!("sheets", "sheet_add_section", "Add a SECTION VIEW of a placement: a second placement of the same saved PMI view whose camera looks along the normal of the plane through the two anchors, drawing the model CLIPPED at that plane with the cut faces hatched. Both anchors must be points of `source`, whose own viewing direction lies in the plane. The section line, its arrows and its letter are drawn on `source`. Returns the placement id.", AddSectionArgs, sheet_add_section, Mutate, MUTATE, schema_of::<SheetAdded>),
    app_command!("sheets", "sheet_add_detail", "Add a DETAIL VIEW of a placement: the region inside a circle picked on it \u{2014} centred on one anchor, its rim through another \u{2014} redrawn at a scale of its own (twice the source's by default) through the SOURCE's camera, the model's visible lines clipped to the circle. The circle and its letter are drawn on `source`; the detail is captioned `DETAIL B` over its scale. A detail of a detail and a detail of a section are refused by name. Returns the placement id.", AddDetailArgs, sheet_add_detail, Mutate, MUTATE, schema_of::<SheetAdded>),
    app_command!("sheets", "sheet_new_section", "Add a SECTION VIEW with no cutting line yet and open its form \u{2014} what the Drawing workbench's Section view button does. It draws its id and `no cutting line yet` until its `cut` field (two anchors of one placement) is set, through `sheet_update_view` or the form's reference picker; the first line names its source, whose saved view and scale it takes, beside it. Returns the placement id.", NewDerivedArgs, sheet_new_section, Mutate, MUTATE, schema_of::<SheetAdded>),
    app_command!("sheets", "sheet_new_detail", "Add a DETAIL VIEW with no circle yet and open its form \u{2014} what the Drawing workbench's Detail view button does. It is unresolved until its `centre` and `rim` fields (anchors of one placement) are set, through `sheet_update_view` or the form's reference picker; the first names its source, whose view it takes at twice its scale, beside it. Returns the placement id.", NewDerivedArgs, sheet_new_detail, Mutate, MUTATE, schema_of::<SheetAdded>),
    app_command!("sheets", "sheet_anchor_pick", "Pick an anchor BY NAME while the reference picker is up for a sheet object's reference row \u{2014} what clicking its marker (`sheet/anchor:<ref>`) does, for anchors that sit too close on the paper for a click to say which was meant. A list row gains it (in a row that is full for its object, the oldest pick makes room) and commits from the row's own `sheets/sheets:field:<path>#activate` (Done while it is focused); a single row is replaced and commits on the pick. Refused when no sheet reference is being picked.", AnchorPickArgs, sheet_anchor_pick, Mutate, MUTATE),
];
