//! Render/UI settings: the same JSON the Settings panel edits.
use crate::automation::command::{app_command, read_engine, parse_args, CommandSpec, Ctx, NoArgs, Outcome};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SettingsSetArgs {
    /// A partial settings object; unknown or absent keys keep their values.
    pub patch: Value,
}

// Mirrors RenderSettings::to_json plus the live camera projection injected by
// EngineState::settings_json. UI field descriptors omit persisted settings, so
// they cannot alone describe this readback. Both commands return the full state.
fn settings_result_schema() -> Value {
    let mut properties = serde_json::Map::new();
    for key in [
        "disableRecoveryPrompt", "disableStartupModel", "debugGrabHandles",
        "showOccludedEdges", "flatShading", "wireframe", "showFaces", "showEdges",
        "showVertices", "overrideModelColors", "showWorkbenchToolbar",
        "pickDoubleSided", "assemblyAutoSolve", "showConstraintGraphics", "orthographic",
    ] {
        properties.insert(key.into(), json!({"type":"boolean"}));
    }
    for key in [
        "uiScale", "labelScale", "edgeWidthPx", "hiddenEdgeAlpha", "vertexSizePx",
        "axisLengthPx", "viewcubeSizePx",
    ] {
        properties.insert(key.into(), json!({"type":"number"}));
    }
    for key in [
        "background", "faceColor", "faceSelectedColor", "hoverColor", "edgeColor",
        "edgeSelectedColor", "vertexColor", "vertexSelectedColor", "sketchMovableColor",
        "sketchLockedColor", "sketchGeometryColor", "sketchPointColor",
        "sketchConstructionPointColor", "sketchUnderConstrainedPointColor",
        "sketchSelectedColor", "sketchHoveredColor", "sketchPreviewColor",
        "sketchConstraintColor", "sketchConflictColor", "renderQuality", "workbench", "bomColumns",
    ] {
        properties.insert(key.into(), json!({"type":"string"}));
    }
    properties.insert("theme".into(), json!({"type":"string","enum":["auto","light","dark"]}));
    properties.insert("faceColorMode".into(), json!({"type":"string","enum":["uniform","hashedBySolid"]}));
    properties.insert("toolbarStyle".into(), json!({"type":"string","enum":["Ribbon","Classic"]}));
    properties.insert("multiSelect".into(), json!({"type":"string","enum":["Ctrl+Click","Click toggles"]}));
    let required: Vec<_> = properties.keys().cloned().collect();
    json!({"type":"object","properties":{"settings":{
        "type":"object","properties":properties,"required":required,"additionalProperties":false
    }},"required":["settings"],"additionalProperties":false})
}

fn settings_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    read_engine(ctx, args, |e| json!({ "settings": e.settings_value() }))
}

fn settings_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SettingsSetArgs = parse_args(args)?;
    let text = serde_json::to_string(&a.patch).map_err(|e| e.to_string())?;
    ctx.app.docs.engine_mut().apply_settings_json(&text)?;
    Ok(Outcome::Done(json!({ "settings": ctx.app.docs.engine().settings_value() })))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("settings", "settings_get", "The render and UI settings (theme, colours, edge width, workbench, …).", NoArgs, settings_get, Read, READ, settings_result_schema),
    app_command!("settings", "settings_set", "Apply a partial settings patch (the keys `settings_get` returns).", SettingsSetArgs, settings_set, Mutate, MUTATE_NOWAIT, settings_result_schema),
];
