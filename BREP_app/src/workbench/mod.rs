//! Per-file WORKBENCH registry — a UI FILTER over feature-CREATION.
//!
//! A "workbench" trims the feature-creation UI (the "Add new feature" palette and
//! the selection context bar) and can gate workbench-specific toolbar buttons /
//! panels. It NEVER changes what the feature history executes, renders, or lets
//! the user edit: a document full of sheet-metal features opens and works
//! identically under Modeling. The ONLY things a workbench touches are the palette
//! (see [`includes_feature`]) and the context offers.
//!
//! EXTENSIBILITY is the whole point: a new workbench = one new file here + one
//! entry in [`WORKBENCHES`]. No enums to edit. Each file exposes a single
//! `'static` [`Workbench`] that OWNS its own inclusion decision (`includes`), its
//! extra toolbar buttons (data only), and the ids of existing panels it claims.
//!
//! `"All"` is special: it accepts every feature and its buttons are the DERIVED
//! union of every other workbench's buttons ([`workbench_buttons`]) — never
//! hand-maintained — so adding a workbench automatically grows it.
//!
//! # Shared buttons: a shell MODE's tools belong to no workbench
//!
//! A workbench filters feature CREATION. A shell MODE — sketch editing — is
//! entered from a surface no workbench filters, so its tools cannot be any one
//! workbench's without vanishing under the others. [`shared_buttons`] is the
//! second list every row carries: declared once ([`sketch::BUTTONS`]),
//! conditional on the mode, appended to every workbench's own buttons. That is
//! what makes the row the SINGLE home for "this mode's tools" — there is no
//! strip of its own to keep in step, and switching workbench mid-sketch keeps
//! the tools.

use brep_render::engine_state::EngineState;

pub mod all;
pub mod assembly;
pub mod diagram;
pub mod drawing;
pub mod ecad;
pub mod modeling;
pub mod pads;
pub mod pcb;
pub mod pmi;
pub mod sheet_metal;
pub mod sketch;
pub mod symbol;
pub mod wire_harness;

/// The minimal view of one catalogue entry a workbench predicate consumes: the
/// feature `type` CODE (e.g. `"E"`, `"S"`, `"SM.F"`), read from the existing
/// app-side catalogue accessor ([`brep_render::features::feature_catalogue`]).
/// Each workbench file classifies DIRECTLY off this code (no kernel-stamped
/// category — the kernel stays untouched). Borrows from the runtime catalogue
/// JSON, hence the lifetime — the registry itself stays `'static` because the
/// fn-pointer predicate is higher-ranked over the borrow.
pub struct FeatureInfo<'a> {
    pub type_code: &'a str,
}

/// What a button's predicates read: the document's engine and, for the eCAD
/// workbenches, the document's eCAD editors.
///
/// The editors are not engine state — they are egui widgets the app keeps per
/// document ([`ecad::Editors`]) — so a predicate that only took the engine
/// could not say whether eCAD's wire tool is armed, and an eCAD button would
/// have to be offered unconditionally and refused at dispatch instead: a row
/// drawing unavailable buttons as available. `ecad` is `None` where a document
/// has no editors, and then every eCAD button reads NOT offered.
#[derive(Clone, Copy)]
pub struct ButtonState<'a> {
    pub engine: &'a EngineState,
    pub ecad: Option<&'a ecad::Editors>,
}

impl<'a> ButtonState<'a> {
    /// The engine alone: every button but the eCAD workbenches' can answer.
    pub fn of(engine: &'a EngineState) -> Self {
        Self { engine, ecad: None }
    }

    /// The engine and the document's eCAD editors.
    pub fn with_ecad(engine: &'a EngineState, ecad: &'a ecad::Editors) -> Self {
        Self { engine, ecad: Some(ecad) }
    }
}

/// A workbench-specific toolbar button, DATA ONLY (no egui, no behavior): the
/// shared toolbar-button helper renders it and a click surfaces `id` out of the
/// toolbar to the shell, which dispatches on `id`.
///
/// A button may be CONDITIONAL on document state ([`Self::when`]) and may read
/// as PRESSED from it ([`Self::pressed`]). Both are predicates over the
/// document ([`ButtonState`]) rather than flags the toolbar sets, so the
/// declaration still owns the whole answer and the row stays a renderer:
/// Drawing's sheet buttons are offered only while a sheet is OPEN, and the armed
/// sketch draw tool is the lit button. A `None` predicate is the plain case —
/// always offered, never pressed — which is most buttons.
///
/// Three more readings came with the eCAD editors, whose actions carry them:
/// an offered button can be DISABLED with a reason ([`Self::disabled`]), its
/// label can be LIVE ([`Self::caption`]), and it can be a MENU of other buttons
/// ([`Self::menu`]).
pub struct WorkbenchButton {
    /// Stable id the shell matches on when the button is clicked.
    pub id: &'static str,
    /// The single unicode glyph shown on the square button.
    pub glyph: &'static str,
    /// Hover tooltip / human label.
    pub tooltip: &'static str,
    /// When this button is OFFERED at all. `None` = always.
    pub when: Option<fn(&ButtonState) -> bool>,
    /// When this button draws PRESSED (a toggle reflecting live state, the way
    /// the toolbar's wireframe and projection buttons do). `None` = a plain
    /// momentary button.
    pub pressed: Option<fn(&ButtonState) -> bool>,
    /// Why an OFFERED button cannot act right now, or `None` when it can. A
    /// disabled button is drawn greyed with the reason on hover, and the
    /// `workbench_button` command refuses it with the same reason — never a
    /// silent no-op. `None` = never disabled.
    ///
    /// This is eCAD's `enabled`, kept apart from `offered` the way eCAD keeps
    /// it: Finish wire is on the row whenever the wire tool is, and greyed
    /// until there is a wire to finish, rather than blinking in and out.
    pub disabled: Option<fn(&ButtonState) -> Option<&'static str>>,
    /// A LIVE label that replaces [`Self::tooltip`] where one is drawn, for a
    /// button whose words describe state (`Bend: vertical first`) or depend on
    /// the document (a copper layer's name). `None` = the tooltip.
    pub caption: Option<fn(&ButtonState) -> String>,
    /// A MENU button's entries: clicking it opens a menu of these, each with
    /// its live caption and pressed state, instead of surfacing its own id.
    /// The entries are buttons in every other respect — ids the lookup finds
    /// and the `workbench_button` command presses. A menu is offered while any
    /// entry is. Empty = an ordinary button.
    pub menu: &'static [WorkbenchButton],
}

impl WorkbenchButton {
    /// Whether `state` offers this button. The ONE reading of [`Self::when`] —
    /// the toolbar row, the `workbench_button` command and `describe_workbenches`
    /// all route through it, so a button an agent can call is exactly a button
    /// the user can see.
    pub fn offered(&self, state: &ButtonState) -> bool {
        self.when.is_none_or(|when| when(state))
            && (self.menu.is_empty() || self.menu.iter().any(|entry| entry.offered(state)))
    }

    /// Whether this button draws pressed in `state`.
    pub fn is_pressed(&self, state: &ButtonState) -> bool {
        self.pressed.is_some_and(|pressed| pressed(state))
    }

    /// Why this button cannot act in `state`, or `None` when it can.
    pub fn disabled_reason(&self, state: &ButtonState) -> Option<&'static str> {
        self.disabled.and_then(|disabled| disabled(state))
    }

    /// The words this button shows in `state`: its live caption, or its tooltip.
    pub fn label(&self, state: &ButtonState) -> String {
        match self.caption {
            Some(caption) => caption(state),
            None => self.tooltip.to_string(),
        }
    }
}

/// One workbench: a UI filter + optional toolbar buttons + claimed panels. Fully
/// `'static` — fn-pointer predicate and `&'static` slices, no `OnceLock`.
pub struct Workbench {
    /// Stable id, e.g. `"all"` / `"modeling"` / `"sheetMetal"` — the value stored
    /// in `RenderSettings.workbench` and published to the verifier.
    pub id: &'static str,
    /// Human label for the dropdown, e.g. `"Sheet Metal"`.
    pub label: &'static str,
    /// SVG catalog key used by the workbench switcher.
    pub glyph: &'static str,
    /// This workbench OWNS its inclusion decision: does a catalogue entry belong
    /// in this workbench's feature-creation UI?
    pub includes: fn(&FeatureInfo<'_>) -> bool,
    /// Extra toolbar buttons this workbench adds (data only). EMPTY in v1.
    pub buttons: &'static [WorkbenchButton],
    /// Ids of EXISTING panels this workbench CLAIMS. Claim-based, not hide-based:
    /// a panel is visible unless it is claimed by ≥1 workbench and the active one
    /// does not list it (see [`panel_visible`]). EMPTY in v1 (nothing claimed →
    /// every panel visible everywhere). Phase 2 adds claims here, editing NO other
    /// file — the same anti-rot property the button union gives.
    pub panels: &'static [&'static str],
}

/// The fallback / default workbench id. Boot-read validation is implicit:
/// [`resolve`] maps any unknown stored id to this, so there is no separate boot
/// step — every consumer routes through `resolve`.
pub const DEFAULT_WORKBENCH_ID: &str = "modeling";

/// Every workbench, in DROPDOWN ORDER: All, Modeling, Sheet Metal. The dropdown
/// iterates THIS — labels are never hardcoded. A new workbench is appended here.
pub static WORKBENCHES: &[&Workbench] = &[
    &all::ALL,
    &modeling::MODELING,
    &sheet_metal::SHEET_METAL,
    // Placeholders — established for the dropdown, fleshed out later.
    &wire_harness::WIRE_HARNESS,
    &assembly::ASSEMBLY,
    &pmi::PMI,
    &drawing::DRAWING,
    // The eCAD four: their editors are drawn in place of the 3D view.
    &diagram::DIAGRAM,
    &pcb::PCB,
    &symbol::SYMBOL,
    &pads::PADS,
];

/// Workbenches DECLARED but not yet in the dropdown — none today. A
/// workbench whose records and buttons land before anything draws it waits
/// here, held to every law below, and registering it is a move from this
/// list to [`WORKBENCHES`], which `every_declared_workbench_is_in_exactly_one_list`
/// keeps honest in both directions. The eCAD four waited here until their
/// editors were drawn: a workbench whose buttons act on an editor nobody can
/// see is worse than one that does nothing.
pub static UNREGISTERED: &[&Workbench] = &[];

/// Pure lookup by id — `None` if there is no such workbench.
pub fn workbench_by_id(id: &str) -> Option<&'static Workbench> {
    WORKBENCHES.iter().copied().find(|w| w.id == id)
}

/// Resolve a (possibly stale / unknown) stored id to a live workbench, falling
/// back to the default. This IS the boot/read validation — route ALL consumers
/// (dropdown display, palette/offer filters, buttons, panels) through here.
pub fn resolve(id: &str) -> &'static Workbench {
    workbench_by_id(id).unwrap_or_else(|| {
        workbench_by_id(DEFAULT_WORKBENCH_ID).expect("default workbench must be registered")
    })
}

/// The buttons EVERY workbench carries, whatever the active one is — the tools
/// of a SHELL MODE. A workbench is a filter over feature CREATION; a shell mode
/// is entered from a surface no workbench filters (the sketch tools' own
/// `editSketch`, on a History panel no workbench claims), so no workbench owns
/// its tools and every row has to offer them. Declared ONCE — [`sketch::BUTTONS`]
/// is the only entry today — and conditional on the mode, so out of it the rows
/// are exactly what each workbench declares.
///
/// This is the ONE reading: [`workbench_buttons`] appends it and
/// [`button_by_id`] searches it, so the row, `describe_workbenches` and the
/// `workbench_button` command agree about what exists.
pub fn shared_buttons() -> &'static [WorkbenchButton] {
    sketch::BUTTONS
}

/// Buttons one workbench LENDS to another: `(borrower id, the lent buttons)`.
/// The borrower's row shows the lender's own declaration — the same id, the
/// same picture, the same tooltip, dispatched by the same shell arm — so the
/// two rows cannot drift apart the way two declarations would.
///
/// A lent button is NOT one of the borrower's own: it is declared once, by its
/// owner, which is what keeps every law above intact. The prefix law
/// ([`button_prefix`]) reads a workbench's OWN buttons, so PCB showing
/// `assembly.add_component` breaks nothing; the artwork law sees one button
/// with one picture; [`button_by_id`] finds the one declaration; and All's
/// deduped union is built from OWN lists, so a lent button is in it exactly
/// once — via its owner. What a borrow changes is only [`workbench_buttons`]:
/// the borrower's row, and therefore [`offered_buttons`], `describe_workbenches`
/// and what `workbench_button` will press while it is active.
///
/// PCB borrows Assembly's ADD COMPONENT because a board is an assembly of
/// placed parts and the user asked for "the same one": inserting a component
/// is one action wherever it is reached from, and a second declaration would
/// be two ids for it.
pub static BORROWED: &[(&str, &[&WorkbenchButton])] =
    &[("pcb", &[assembly::ADD_COMPONENT])];

/// The buttons workbench `id` BORROWS ([`BORROWED`]) — empty for a workbench
/// that borrows none, and for `"all"`, whose union already carries every
/// lender's own declaration.
pub fn borrowed_buttons(id: &str) -> &'static [&'static WorkbenchButton] {
    BORROWED
        .iter()
        .find(|(borrower, _)| *borrower == id)
        .map_or(&[], |(_, buttons)| *buttons)
}

/// The active workbench's toolbar buttons: its OWN declared buttons — for
/// `"all"` the DEDUPED UNION of every workbench's, so "All" shows every icon
/// without a hand-maintained list — then the ones it BORROWS ([`BORROWED`]),
/// then the [`shared_buttons`] every workbench carries. The borrowed ones trail
/// the workbench's own, exactly as the shared tail trails both, so a row's own
/// declared order (eCAD's toolbar order, in PCB's case) is untouched by a
/// borrow.
pub fn workbench_buttons(id: &str) -> Vec<&'static WorkbenchButton> {
    let wb = resolve(id);
    let own = if wb.id == "all" {
        union_in(WORKBENCHES)
    } else {
        wb.buttons.iter().collect::<Vec<_>>()
    };
    own.into_iter()
        .chain(borrowed_buttons(wb.id).iter().copied())
        .chain(shared_buttons())
        .collect()
}

/// The active workbench's buttons that `state` actually OFFERS — [`workbench_buttons`]
/// filtered by each button's [`WorkbenchButton::when`]. This is what the toolbar row
/// draws and what `describe_workbenches` reports as active, so the two can never
/// disagree about which buttons exist right now.
pub fn offered_buttons(id: &str, state: &ButtonState) -> Vec<&'static WorkbenchButton> {
    workbench_buttons(id).into_iter().filter(|button| button.offered(state)).collect()
}

/// Look one button up by id across every workbench AND the shared list — the
/// dispatch side, which must answer for a button the active workbench does not
/// carry (the `workbench_button` command names an id, not a workbench) and for
/// one no workbench declares at all (the sketch tools).
pub fn button_by_id(id: &str) -> Option<&'static WorkbenchButton> {
    button_in(WORKBENCHES, id)
}

/// What every one of a workbench's OWN button ids leads with: its id,
/// lowercased, and a dot — `drawing.dim.radius`, `sheetmetal.flat_pattern`.
///
/// A button id is all the toolbar's return path and the `workbench_button`
/// command carry, so it must name ONE button across every workbench: with an
/// id declared twice, [`button_by_id`] returns the first declaration and asks
/// ITS `when`, and All's union keeps the first and drops the rest. The prefix
/// makes two workbenches' ids disjoint. It matters where two workbenches
/// take their buttons from one table: eCAD's editor actions (`view.*`,
/// `edit.*`, `sheet.*`, `board.*`) are Diagram's and PCB's alike, so each
/// leads them with its own — `diagram.sheet.tool.wire`,
/// `pcb.sheet.tool.wire` — while the symbol and pads editors' ids already
/// lead with `symbol.` and `pads.` and need nothing added. The shared sketch
/// tools belong to no workbench and lead with none of theirs (`sketch.`).
pub fn button_prefix(wb: &Workbench) -> String {
    format!("{}.", wb.id.to_ascii_lowercase())
}

/// [`button_by_id`] over any registry.
fn button_in(registry: &[&'static Workbench], id: &str) -> Option<&'static WorkbenchButton> {
    every_button_in(registry).find(|button| button.id == id)
}

/// Why the `workbench_button` command must NOT press `id` in `state`, or
/// `None` when it may. The command's whole gate, here rather than in the
/// command so it is tested against every kind of button: an unknown id, one
/// the row is not offering, a MENU (its entries are pressed, not it), and an
/// offered button that is DISABLED, which is refused with its reason — never
/// run as a silent no-op.
pub fn press_refusal(id: &str, state: &ButtonState) -> Option<String> {
    press_refusal_in(WORKBENCHES, id, state)
}

/// [`press_refusal`] over any registry.
fn press_refusal_in(registry: &[&'static Workbench], id: &str, state: &ButtonState) -> Option<String> {
    let Some(button) = button_in(registry, id) else {
        return Some(format!("no workbench button `{id}` (see describe_workbenches)"));
    };
    if !button.offered(state) {
        return Some(format!(
            "the `{id}` button is not offered right now (see describe_workbenches `activeButtons`)"
        ));
    }
    if !button.menu.is_empty() {
        let entries: Vec<&str> =
            button.menu.iter().filter(|e| e.offered(state)).map(|e| e.id).collect();
        return Some(format!("`{id}` opens a menu; press one of its entries: {}", entries.join(", ")));
    }
    button
        .disabled_reason(state)
        .map(|why| format!("the `{id}` button is disabled right now: {why}"))
}

/// Buttons that draw ONE picture although their ids differ, because they are
/// one user-facing action in different places — each group with the reason.
/// The artwork law (`every_button_draws_catalogued_artwork_shared_only_by_one_action`)
/// lets two buttons share a picture only when they are the same action behind
/// their workbench prefixes (Diagram's and PCB's Rotate) or are listed here.
/// A share between unrelated buttons still fails, which is what the law is for;
/// a deliberate reuse is a reviewable line here instead of a duplicated icon.
///
/// Two reuses were REFUSED rather than listed, because the picture already
/// means something else in this app: U+21BB ↻ is the BOM's outdated-vs-source
/// badge (`panels::assembly_components::OUTDATED_GLYPH`), not "rotate", and
/// U+2713 ✓ is a status tick (a resolved annotation, the switcher's active
/// entry), not "finish". Rotate and finish have pictures of their own.
pub const SYNONYMS: &[(&[&str], &str)] = &[
    (
        &[
            "sketch.tool.select",
            "diagram.sheet.tool.select",
            "pcb.sheet.tool.select",
            "pcb.board.tool.select",
            "symbol.tool.select",
            "pads.tool.select",
        ],
        "select and drag, in every editor",
    ),
    (&["sketch.tool.line", "symbol.tool.line", "pads.tool.line"], "draws a straight line"),
    (&["sketch.tool.rect", "symbol.tool.rectangle", "pads.tool.rectangle"], "draws a rectangle"),
    (&["sketch.tool.circle", "symbol.tool.circle", "pads.tool.circle"], "draws a circle"),
    (&["pcb.sheet.wire.finish", "pcb.board.route.finish"], "ends the path being drawn"),
];

/// Every DECLARED workbench: the dropdown's ([`WORKBENCHES`]), then the ones
/// not in it yet ([`UNREGISTERED`]). What the laws hold, so a workbench is
/// covered before it is offered.
pub fn declared_workbenches() -> Vec<&'static Workbench> {
    WORKBENCHES.iter().chain(UNREGISTERED).copied().collect()
}

/// [`button_by_id`] over every DECLARED workbench, registered or not — for the
/// eCAD module, whose buttons exist before their workbenches are offered.
pub(crate) fn button_by_declared_id(id: &str) -> Option<&'static WorkbenchButton> {
    button_in(&declared_workbenches(), id)
}

/// Every DECLARED button in the app: each workbench's own, then the shared
/// ones. The lookup + audit side ([`button_by_id`], the artwork and distinctness
/// tests) — NOT what any row draws, which is [`offered_buttons`].
pub fn every_button() -> impl Iterator<Item = &'static WorkbenchButton> {
    every_button_in(WORKBENCHES)
}

/// [`every_button`] over any registry.
///
/// A MENU button's entries are buttons too — the command presses them by id —
/// so each follows the menu that holds it.
fn every_button_in<'r>(
    registry: &'r [&'static Workbench],
) -> impl Iterator<Item = &'static WorkbenchButton> + 'r {
    registry
        .iter()
        .flat_map(|w| w.buttons.iter())
        .chain(shared_buttons())
        .flat_map(|button| std::iter::once(button).chain(button.menu.iter()))
}

/// All's own buttons over any registry: the union of every workbench's.
fn union_in(registry: &[&'static Workbench]) -> Vec<&'static WorkbenchButton> {
    dedupe_buttons(registry.iter().map(|w| w.buttons))
}

/// Collect buttons across lists, keeping FIRST occurrence per `id` and preserving
/// order. The union logic behind [`workbench_buttons`] for `"all"`.
fn dedupe_buttons<'a>(
    lists: impl Iterator<Item = &'a [WorkbenchButton]>,
) -> Vec<&'a WorkbenchButton> {
    let mut seen = std::collections::HashSet::new();
    let mut out = Vec::new();
    for list in lists {
        for button in list {
            if seen.insert(button.id) {
                out.push(button);
            }
        }
    }
    out
}

/// Whether a catalogue entry belongs in workbench `active_id`'s feature-creation
/// UI. The ONE entry point the palette + context-offer filters call: builds a
/// [`FeatureInfo`] from the feature `type_code` and runs the resolved workbench's
/// predicate (each workbench file owns its classification off the code).
pub fn includes_feature(active_id: &str, type_code: &str) -> bool {
    let info = FeatureInfo { type_code };
    (resolve(active_id).includes)(&info)
}

/// The Qualify pane's registration id: a part's declared connection points —
/// its port groups, their purpose, and the consistency reports the model
/// already produces.
///
/// Claimed by NO workbench, and deliberately: a connection point is part DATA
/// and belongs to whichever surface the user is on — the schematic symbol, the
/// pads, or the 3D model — so a claim list would have to name most of the
/// registry and would still hide the pane from the one workbench it was
/// forgotten in. What gates it is the DOCUMENT ([`PANEL_CONDITIONS`]), which is
/// the honest condition: a part that declares no connection points has nothing
/// to qualify.
pub const QUALIFY_PANEL_ID: &str = "qualify";

/// A panel whose visibility ALSO depends on the document, beside the claim.
///
/// The claim system answers "which workbench is this panel's". It cannot answer
/// "does this document have anything for it", because a [`Workbench`]'s
/// `panels` is a list of ids with no room for a predicate. This is that second
/// half, in the shape [`WorkbenchButton::when`] already uses — a predicate over
/// [`ButtonState`], which carries the whole engine — so a conditional PANEL is
/// declared exactly like a conditional BUTTON and read in exactly one place
/// ([`panel_visible`]).
///
/// Keyed by PANEL rather than by workbench because the condition is the
/// panel's own: Qualify wants a part with connection points wherever it is
/// drawn, not a different answer per workbench.
pub struct PanelCondition {
    pub panel: &'static str,
    /// Whether the document has anything for this panel. `false` hides it
    /// exactly as an unclaimed workbench does — in place, keeping its position.
    pub when: fn(&ButtonState) -> bool,
}

/// Every conditional panel.
pub static PANEL_CONDITIONS: &[PanelCondition] = &[
    PanelCondition { panel: QUALIFY_PANEL_ID, when: declares_connection_points },
    PanelCondition { panel: FAMILY_TABLE_PANEL_ID, when: is_family_document },
    PanelCondition { panel: PLM_PANEL_ID, when: crate::panels::plm_host::in_plm_session },
];

/// The PLM pane's registration id (`panels::plm_host`): every PLM panel, as
/// sections of one pane. Claimed by no workbench; on screen exactly while the
/// session's store is a PLM.
pub const PLM_PANEL_ID: &str = "plm";

/// The Family table pane's registration id: a family seed's member table
/// (`panels::family_table_editor`). Claimed by no workbench. It is on screen
/// exactly while the document being edited is a family seed (`.fbrep`),
/// whatever workbench the user is in.
pub const FAMILY_TABLE_PANEL_ID: &str = "familyTable";

/// Whether the active document is a family seed: its own `documentClass`
/// field, so a family that has never been saved answers too.
pub fn is_family_document(state: &ButtonState) -> bool {
    crate::document_class::is_family(state.engine)
}

/// Whether the document declares any connection point: a `ports` block with at
/// least one group in it. An EMPTY block reads as none — the block is omitted
/// on re-serialize when empty, so an empty array is a transient the panel's own
/// "remove the last group" leaves behind, and it must not keep the pane up.
pub fn declares_connection_points(state: &ButtonState) -> bool {
    state
        .engine
        .history
        .ports_block()
        .and_then(serde_json::Value::as_array)
        .is_some_and(|groups| !groups.is_empty())
}

/// Whether panel `panel_id` is visible in workbench `active_id`, for document
/// `state`. TWO halves, and this is the one reading of both.
///
/// Claim-based: a panel is visible UNLESS it is claimed by ≥1 workbench and the
/// active workbench does not list it. `"all"` sees every panel (mirrors the
/// button union).
///
/// Then CONDITIONAL: a panel [`PANEL_CONDITIONS`] names is visible only while
/// its predicate holds for this document. A panel with no condition — every one
/// but Qualify — is unaffected, so `state` costs the other callers nothing but
/// the argument.
pub fn panel_visible(active_id: &str, panel_id: &str, state: &ButtonState) -> bool {
    if !PANEL_CONDITIONS
        .iter()
        .filter(|condition| condition.panel == panel_id)
        .all(|condition| (condition.when)(state))
    {
        return false;
    }
    let claimed = WORKBENCHES.iter().any(|w| w.panels.contains(&panel_id));
    if !claimed {
        return true;
    }
    let wb = resolve(active_id);
    if wb.id == "all" {
        return true;
    }
    wb.panels.contains(&panel_id)
}

/// The workbench list as `{id, label}` JSON plus the resolved current id — the
/// `__brepWorkbench` logical-state global the headed verifier reads to drive the
/// dropdown and confirm the active workbench. `current` is the RESOLVED id (an
/// unknown stored id reads back as the default).
pub fn workbench_state_json(current_stored: &str) -> String {
    let available: Vec<serde_json::Value> = WORKBENCHES
        .iter()
        .map(|w| serde_json::json!({ "id": w.id, "label": w.label }))
        .collect();
    serde_json::json!({
        "current": resolve(current_stored).id,
        "available": available,
    })
    .to_string()
}

