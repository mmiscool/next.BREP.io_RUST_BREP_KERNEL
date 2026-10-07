//! The app SHELL: the toolbar's chrome buttons and the dock's panes.
//!
//! Everything else in this layer drives the document; these drive the window
//! around it. They exist so the toolbar is fully reachable without a pointer —
//! `hit_keys` records the command each button names, and a test refuses a
//! toolbar button that names none. A widget an agent can only click is a widget
//! it cannot use the moment something scrolls or covers the rect.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::HELP_URL;
use crate::panels::dock::PaneKind;
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct OpenArgs {
    /// Open (true) or close the window.
    pub open: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WorkbenchButtonArgs {
    /// The button's `WorkbenchButton::id`, e.g. `sheetmetal.flat_pattern`.
    /// `describe_workbenches` lists the ids the active workbench offers.
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PaneArgs {
    /// A dock pane name as `describe_panes` lists it, e.g. `History`.
    pub pane: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PickArgs {
    /// Who asks: the file comes back only to this tag.
    pub tag: String,
    /// The chooser's heading. Default "Choose a file".
    #[serde(default)]
    pub title: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PickedArgs {
    pub tag: String,
}

fn file_pick(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PickArgs = parse_args(args)?;
    let (file, _, store) = ctx.app.file_docs_store();
    file.request_pick_file(store, &a.tag, a.title.as_deref().unwrap_or("Choose a file"))?;
    Ok(Outcome::Done(json!({ "tag": a.tag, "open": true })))
}

fn file_picked(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PickedArgs = parse_args(args)?;
    let (file, _, _) = ctx.app.file_docs_store();
    let picking = file.picking(&a.tag);
    Ok(Outcome::Done(match file.take_picked_file(&a.tag) {
        Some(picked) => json!({
            "picked": true, "name": picked.name, "path": picked.path, "size": picked.bytes.len(),
            "text": std::str::from_utf8(&picked.bytes).ok().filter(|t| t.len() <= 4096),
        }),
        None => json!({ "picked": false, "picking": picking }),
    }))
}

fn settings_window(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: OpenArgs = parse_args(args)?;
    ctx.app.set_settings_window_open(a.open);
    Ok(Outcome::Done(json!({ "open": a.open })))
}

fn part_properties_window(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: OpenArgs = parse_args(args)?;
    ctx.app.set_part_properties_window_open(a.open);
    Ok(Outcome::Done(json!({ "open": a.open })))
}

fn help_open(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    // In a window this opens the help site in a browser tab; headless there is
    // no browser to open it in, so the URL in the reply is the whole answer.
    ctx.egui.open_url(egui::OpenUrl::new_tab(HELP_URL));
    Ok(Outcome::Done(json!({ "url": HELP_URL })))
}

fn info_window(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: OpenArgs = parse_args(args)?;
    ctx.app.set_info_window_open(a.open);
    Ok(Outcome::Done(json!({ "open": a.open })))
}

fn diagnostics(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    // The app's ONE Diagnostics instance — the same rows the Info window draws
    // and a problem report carries, not a fresh probe of the device (which
    // could answer differently from what is actually rendering).
    Ok(Outcome::Done(ctx.app.diagnostics().json()))
}

fn bug_report_open(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let egui = ctx.egui.clone();
    ctx.app.begin_bug_report(&egui);
    Ok(Outcome::Done(json!({})))
}

fn workbench_button(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: WorkbenchButtonArgs = parse_args(args)?;
    // A CONDITIONAL button is refused exactly when the row would not be showing
    // it: Drawing's sheet buttons want paper on screen. Without this the command
    // surface would do what no click can, and a dimension would be made on a
    // sheet nobody is looking at.
    //
    // An offered button can still be DISABLED (an eCAD action its editor does
    // not allow yet): that is refused with the reason, never run as a silent
    // no-op. A MENU button is not pressed itself — its entries are. The gate
    // is `workbench::press_refusal`, where it is tested against each kind.
    let doc = ctx.app.docs.active();
    let state = crate::workbench::ButtonState { engine: &doc.engine, ecad: Some(&doc.ecad) };
    if let Some(refusal) = crate::workbench::press_refusal(&a.id, &state) {
        return Err(refusal);
    }
    if !ctx.app.dispatch_workbench_button(&a.id) {
        return Err(format!("no workbench button `{}` (see describe_workbenches)", a.id));
    }
    Ok(Outcome::Done(json!({})))
}

fn describe_workbenches(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let active = ctx.app.docs.engine().settings.workbench.clone();
    let doc = ctx.app.docs.active();
    let state = crate::workbench::ButtonState { engine: &doc.engine, ecad: Some(&doc.ecad) };
    let offered: Vec<&crate::workbench::WorkbenchButton> = crate::workbench::offered_buttons(&active, &state)
        .into_iter()
        .flat_map(|b| std::iter::once(b).chain(b.menu.iter().filter(|e| e.offered(&state))))
        .collect();
    let mut workbenches: Vec<Value> = crate::workbench::WORKBENCHES
        .iter()
        .map(|w| {
            json!({
                "id": w.id,
                "label": w.label,
                // What this workbench DECLARES of its own. The buttons every
                // workbench also carries are reported once, as `sharedButtons`
                // below, rather than eleven times here — that is the shape the
                // registry actually has (`workbench::shared_buttons`), and a
                // reader asking "whose button is this?" gets the true answer.
                "buttons": w.buttons.iter().map(|b| json!({ "id": b.id, "tooltip": b.tooltip })).collect::<Vec<_>>(),
                "panels": w.panels,
            })
        })
        .collect();
    workbenches.extend(crate::workbench::owned_workbenches(&doc.engine).into_iter().map(|w| serde_json::to_value(w).unwrap()));
    Ok(Outcome::Done(json!({
        "active": crate::workbench::resolved_id(&doc.engine, &active),
        "saved": active,
        "pluginActions": crate::workbench::plugin_actions(&doc.engine, &active),
        "pluginPanels": crate::workbench::plugin_panels(&doc.engine, &active),
        "pluginAnnotations": doc.engine.plugin_annotation_catalogue(),
        "workbenches": workbenches,
        // The buttons EVERY workbench's row carries whatever is active — the
        // sketch tools. Declared by no workbench, so they are listed here and
        // not under one.
        "sharedButtons": crate::workbench::shared_buttons().iter().map(|b| json!({ "id": b.id, "tooltip": b.tooltip })).collect::<Vec<_>>(),
        // A MENU's offered entries are pressable by id, so they are listed
        // after the menu; a DISABLED button is listed here AND under
        // `disabledButtons` with the reason a press would be refused with.
        "activeButtons": offered.iter().map(|b| b.id).collect::<Vec<_>>(),
        "disabledButtons": offered
            .iter()
            .filter_map(|b| b.disabled_reason(&state).map(|why| json!({ "id": b.id, "reason": why })))
            .collect::<Vec<_>>(),
    })))
}

fn describe_panes(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let _ = ctx;
    Ok(Outcome::Done(json!({ "panes": PaneKind::ALL.iter().map(|k| k.title()).collect::<Vec<_>>() })))
}

fn show_pane(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: PaneArgs = parse_args(args)?;
    let kind = PaneKind::ALL
        .iter()
        .copied()
        .find(|k| k.title().eq_ignore_ascii_case(&a.pane))
        .ok_or_else(|| format!("no pane `{}` (see describe_panes)", a.pane))?;
    // A workbench-CLAIMED pane is HIDDEN under a workbench that does not claim
    // it (it keeps its place in the tree — see `panels::dock`), and activating a
    // hidden tab is a silent no-op: egui_tiles moves the active tab straight
    // back to a visible one. So say so instead of reporting success for nothing.
    let workbench = ctx.app.docs.engine().settings.workbench.clone();
    let panels = crate::workbench::ButtonState::of(ctx.app.docs.engine());
    if !kind.visible_under_workbench(&workbench, &panels) {
        // Two ways to be hidden, and they want different answers: a CLAIMED
        // pane needs another workbench, a CONDITIONAL one needs a different
        // document. Telling a caller to switch workbench for the Qualify pane
        // would send them round every one of them.
        return Err(match kind {
            PaneKind::Qualify => format!(
                "the {} pane is on screen only while the part declares connection points —                  press the `{}` workbench button first",
                kind.title(),
                crate::workbench::wire_harness::DECLARE_POINT_BUTTON_ID,
            ),
            PaneKind::FamilyTable => format!(
                "the {} pane is on screen only while the document is a family seed (.fbrep)",
                kind.title(),
            ),
            PaneKind::Plm => format!("the {} pane is on screen only while the session is on a PLM", kind.title()),
            _ => format!(
                "the {} pane is claimed by another workbench (active: `{}`) — switch with settings_set {{patch:{{workbench}}}}",
                kind.title(),
                crate::workbench::resolve(&workbench).id
            ),
        });
    }
    ctx.app.show_pane(kind);
    Ok(Outcome::Done(json!({ "pane": kind.title() })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "file_pick", group: "shell", doc: "Open the file chooser for ANY file on this machine, on behalf of `tag` (what a panel's Add file… does). Natively it is the app's own explorer over the local files, even in a PLM session; drive it by its `pickfile:` hit keys. In the browser it is the page's file input. The chosen file is taken with `file_picked`.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<PickArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(file_pick) },
    CommandSpec { name: "file_picked", group: "shell", doc: "Take the file the chooser delivered for `tag`, once: `{picked, name, path, size, text}` (`text` when it is UTF-8 and at most 4 KB), or `{picked: false, picking}` while the chooser is still open or after a cancel.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<PickedArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(file_picked) },
    CommandSpec { name: "settings_window", group: "shell", doc: "Open or close the floating Settings window (the toolbar's gear). The settings themselves are read and written with `settings_get` / `settings_set`, no window needed.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<OpenArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(settings_window) },
    CommandSpec { name: "part_properties_window", group: "shell", doc: "Open or close the floating Part Properties window (the toolbar's tag button), which edits the active document's own BOM attributes. The attributes themselves are read and written with `part_attributes_get` / `part_attribute_set`, no window needed.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<OpenArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(part_properties_window) },
    CommandSpec { name: "help_open", group: "shell", doc: "Open the generated help site in a browser tab (the toolbar's Help button) and return its URL. Headless there is no browser, so the URL is the answer.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(help_open) },
    CommandSpec { name: "info_window", group: "shell", doc: "Open or close the floating Info window (the toolbar's \u{2139} button): the project + third-party licences, and this session's renderer diagnostics. Read the diagnostics themselves with `diagnostics`, no window needed.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<OpenArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(info_window) },
    CommandSpec { name: "diagnostics", group: "shell", doc: "What this session is running on: the renderer actually in use (WebGPU or the WebGL2 fallback in a browser; Vulkan/Metal/Direct3D 12 natively), the adapter, the texture ceiling that bounds the viewport, the app version and the platform. Captured at startup from the adapter that is drawing, never re-probed \u{2014} the same rows the Info window shows and every problem report carries.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(diagnostics) },
    CommandSpec { name: "bug_report_open", group: "shell", doc: "Begin the in-app problem report (the toolbar's Submit Bug button): captures the current frame, then opens the description form.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(bug_report_open) },
    CommandSpec { name: "workbench_button", group: "shell", doc: "Press one of the active workbench's toolbar buttons by id — the same dispatch a click runs (flat pattern, add component, interference, parts library, capture PMI view; while a sheet is open, PMI's Back to 3D and the five sheet-dimension constructions; and while a sketch is being edited, the ten draw tools and Auto-constrain, which EVERY workbench carries). A button whose condition is not met right now is refused rather than run; `describe_workbenches` lists what is offered as `activeButtons`. An offered button that is DISABLED is refused with the reason (`describe_workbenches` `disabledButtons`), and a MENU button is refused with its entries, which are pressed by their own ids.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<WorkbenchButtonArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(workbench_button) },
    CommandSpec { name: "describe_workbenches", group: "shell", doc: "Every workbench with its label, its toolbar buttons and the panels it claims, plus which one is active. `buttons` is what that workbench DECLARES; `sharedButtons` is the list EVERY workbench's row also carries (the sketch draw tools), reported once because no workbench owns it. `activeButtons` is what the toolbar row is offering right NOW \u{2014} a button declared conditional on document state (Drawing's sheet buttons want an open sheet; the sketch tools want a sketch being edited) is listed under its workbench but absent from `activeButtons` until its condition holds. A MENU button's offered entries follow it in `activeButtons`; `disabledButtons` lists the offered buttons a press would refuse right now, each with the reason. Switch with `settings_set {patch:{workbench}}`.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(describe_workbenches) },
    CommandSpec { name: "describe_panes", group: "shell", doc: "Every dock pane name `show_pane` accepts.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(describe_panes) },
    CommandSpec { name: "show_pane", group: "shell", doc: "Bring a dock pane to the front (History, Scene, BOM, PMI, …) so its widgets are laid out and its hit-rects are publishable.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<PaneArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(show_pane) },
];
