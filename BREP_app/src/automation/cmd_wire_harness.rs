//! The WIRE HARNESS workbench: the connections between ports, and the routing
//! report the run produces.
//!
//! A connection names two PORT features and a wire diameter; the runner routes
//! it through the harness segments and (when bundles are on) builds the bundle
//! solids. These are the operations `panels::wire_harness` performs, on the
//! same engine methods.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::cmd_document::parse;
use brep_render::engine_state::ConnectionPatch;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddConnectionArgs {
    /// The A-side port feature id (`wire_harness_state`'s report lists the
    /// endpoints); empty leaves it unassigned.
    #[serde(default)]
    pub from: String,
    /// The B-side port feature id; empty leaves it unassigned.
    #[serde(default)]
    pub to: String,
    /// Wire diameter in mm. Non-positive falls back to 1.0.
    #[serde(default = "one")]
    pub diameter: f64,
}

fn one() -> f64 {
    1.0
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateConnectionArgs {
    pub id: String,
    #[serde(default)]
    pub name: Option<String>,
    #[serde(default)]
    pub from: Option<String>,
    #[serde(default)]
    pub to: Option<String>,
    /// Must be positive; a non-positive diameter is refused.
    #[serde(default)]
    pub diameter: Option<f64>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ConnectionIdArgs {
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BuildBundlesArgs {
    /// Build the bundle solids. The routing report is produced either way.
    pub on: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CutMarginArgs {
    /// Added to EVERY wire's routed length to give its BOM cut length (MF QTY),
    /// in model units. Zero or more; a negative margin is refused.
    pub margin: f64,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct ConnectionAdded {
    /// The id the engine minted (`wire-N`).
    pub id: String,
}

fn wire_harness_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(parse(&ctx.app.docs.engine().wire_harness_state_json())))
}

fn wire_harness_add_connection(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddConnectionArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().wire_harness_add_connection(&a.from, &a.to, a.diameter);
    serde_json::to_value(ConnectionAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn wire_harness_update_connection(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateConnectionArgs = parse_args(args)?;
    let patch = ConnectionPatch { name: a.name, from: a.from, to: a.to, diameter: a.diameter };
    ctx.app.docs.engine_mut().wire_harness_update_connection(&a.id, &patch)?;
    Ok(Outcome::Done(json!({})))
}

fn wire_harness_remove_connection(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ConnectionIdArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().wire_harness_remove_connection(&a.id)?;
    Ok(Outcome::Done(json!({})))
}

fn wire_harness_set_build_bundles(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: BuildBundlesArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().wire_harness_set_build_bundles(a.on);
    Ok(Outcome::Done(json!({})))
}

fn wire_harness_set_cut_margin(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: CutMarginArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().wire_harness_set_cut_margin(a.margin)?;
    Ok(Outcome::Done(json!({})))
}

/// Every harness wire as its BOM line — engine truth, the rows the BOM panel
/// draws and the export writes.
fn bom_wire_lines(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let lines: Vec<Value> = ctx
        .app
        .docs
        .engine()
        .wire_bom_lines()
        .iter()
        .map(|line| {
            json!({
                "id": line.id,
                "connectionId": line.connection_id,
                "stockPartNumber": line.stock_part_number,
                "length": line.length,
                "margin": line.margin,
                "mfQty": line.mf_qty,
                "status": line.state.as_str(),
                "reason": if line.mf_qty.is_some() { String::new() } else { line.state.explain() },
            })
        })
        .collect();
    Ok(Outcome::Done(json!({ "lines": lines })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "wire_harness_state", group: "harness", doc: "The document's harness block: every connection, whether bundle solids are built, and the last run's routing report (endpoints, segments, per-connection routes, bundles).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(wire_harness_state) },
    CommandSpec { name: "wire_harness_add_connection", group: "harness", doc: "Add a wire between two port features and re-run the routing. Either end may be empty and filled in later. Returns the connection id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddConnectionArgs>, result_schema: schema_of::<ConnectionAdded>, handler: Handler::App(wire_harness_add_connection) },
    CommandSpec { name: "wire_harness_update_connection", group: "harness", doc: "Patch one connection's name, ends or diameter; absent fields are unchanged.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateConnectionArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(wire_harness_update_connection) },
    CommandSpec { name: "wire_harness_remove_connection", group: "harness", doc: "Delete one connection and re-run the routing.", phase: Phase::Mutate, annotations: Annotations::DESTRUCTIVE, args_schema: schema_of::<ConnectionIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(wire_harness_remove_connection) },
    CommandSpec { name: "wire_harness_set_cut_margin", group: "harness", doc: "Set the cut margin added once to every wire's routed length to give its BOM cut length (MF QTY). The routing is not re-done; a negative margin is refused.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<CutMarginArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(wire_harness_set_cut_margin) },
    CommandSpec { name: "bom_wire_lines", group: "harness", doc: "Every harness wire as its BOM line: connection ID, stock part number, routed length, margin and MF QTY (length + margin) — or no number and the status saying why (routing, cancelled, rolled-back, no-run, or the router's own status).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(bom_wire_lines) },
    CommandSpec { name: "wire_harness_set_build_bundles", group: "harness", doc: "Turn the bundle solids on or off. Off still routes and reports — it just builds no geometry.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<BuildBundlesArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(wire_harness_set_build_bundles) },
];
