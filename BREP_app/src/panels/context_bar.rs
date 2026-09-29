//! Context action toolbar — the **selection-driven** action bar (the engine-
//! native successor to the old app's floating selection action bar,
//! `SelectionFilter._syncSelectionActions` + `_getHistoryContextActionSpecs`).
//!
//! It is shown ONLY while something is selected (hidden otherwise) and its
//! buttons depend on the CURRENT selection (kinds + count read from
//! `selection_json`):
//!
//! * **Generic actions** (mirror the old selection action bar):
//!   - **Clear** — `clear_selection`.
//!   - **Hide** — `hide_selected` (toggles the visibility of EXACTLY what is
//!     selected: a selected face/edge/vertex hides just that sub-entity, a
//!     selected solid the whole solid; a second click shows it again).
//!   - **Edit owning feature** — for a SINGLE selected entity with a known
//!     producer, `creating_feature(name)` resolves the feature that built it;
//!     clicking rolls the model to that step (`roll_to`) and asks the shell to
//!     EXPAND that feature's inline dialog in the history tree. A wire-harness
//!     BUNDLE solid's producer is the harness tail, which is not a history
//!     feature: the button stays in place but DISABLED, its tooltip naming the
//!     spline the bundle follows and the Wire Harness panel as the things to
//!     edit instead ([`Selection::harness_bundle`]).
//! * **Feature-from-selection** — WHICH features a selection offers is answered
//!   by the KERNEL, per feature: each feature module defines `context_applicable`
//!   (aggregated in `feature_pipeline::context_offer`), a predicate over the
//!   [`SelectionProbe`] kind-counts this bar builds each frame. That is where
//!   nuance lives — e.g. Revolve wants a profile AND an axis edge, so a lone
//!   face no longer offers it. The pre-fill stays schema-derived: an offered
//!   feature's `References`-group `reference_selection` fields are filled from
//!   the selection in schema order under a CONSUMED set (each selected name
//!   lands in at most ONE field — face+edge → Revolve fills `profile` and
//!   `axis`). Clicking creates the feature (`add_feature`) with those fields
//!   pre-filled, then asks the shell to expand the new node for tweaking.
//! * **Constraint-from-selection** — the assembly-constraint mirror of the
//!   feature offers, shown when the Assembly Constraints panel is available in
//!   the active workbench (claim-based visibility). Each constraint type's
//!   `applicable` predicate ([`brep_kernel::ConstraintTypeDef`]) runs against
//!   the same probe: all-component selections only (the kernel rejects anything
//!   else), ONE component's solid(s) for Fixed, a two-element pair across TWO
//!   distinct components for the pairing types. Clicking adds the constraint
//!   with `elements` pre-seeded from the selection (the constraints panel's
//!   seeding helper) and opens its row in the panel.
//!
//! Like the other panels this owns NO model state — the selection + history live
//! in [`EngineState`], borrowed in; it only holds the per-frame `hits` map (widget
//! screen rects) + the last-drawn action ids the headed verifier reads.

use crate::automation::hit_keys::HitKeyDoc;
use super::action_rail::{action_rail, ActionItem};
use super::component_actions::{run_component_action, ComponentAction, ComponentActionRequest};
use crate::form;
use brep_render::brep_kernel::{self, SelectionProbe};
use brep_render::engine_state::EngineState;
use brep_render::features;
use brep_render::style::FieldKind;
use eframe::egui;
use serde_json::Value;
use std::collections::{HashMap, HashSet};

/// A request bubbled back to the shell after a context action ran: EXPAND (open
/// the inline dialog of) the feature with this id in the history tree. The
/// context bar mutates the engine directly but cannot reach the history panel's
/// private "expanded" state, so it returns the id for the shell to focus.
pub type FocusRequest = Option<String>;

/// What a context-bar frame hands back to the shell. The bar mutates the engine
/// directly, but two effects it cannot reach itself:
/// * `focus` — the history feature to EXPAND after a create / edit-owning action
///   (the history panel's expand state is private to it); and
/// * `info_targets` — the entity names to open PINNED Info windows for after the
///   Info action (the Info-window manager is shell-owned). One name per selected
///   entity, so a multi-select opens one window each.
#[derive(Default)]
pub struct ContextOutcome {
    pub focus: FocusRequest,
    pub info_targets: Vec<String>,
    /// A COMPONENT document-level flow the shell must run (Edit in place /
    /// Open Part) — set when the matching component action was clicked; the
    /// engine-mutating component actions (Move / Fix-Unfix / Delete) already
    /// applied inside the bar.
    pub component: Option<ComponentActionRequest>,
}

/// The context bar's transient UI state (the model lives in the engine).
#[derive(Default)]
pub struct ContextBarPanel {
    /// Per-frame widget screen rects, published for the headed verifier. Rebuilt
    /// each frame (there is no DOM — egui draws on the canvas).
    hits: HashMap<String, egui::Rect>,
    /// The generic action ids drawn THIS frame (`clear` / `hide` / `edit-owning`)
    /// — published so the verifier can assert WHICH actions the selection offered.
    shown_actions: Vec<String>,
    /// The feature TYPE CODES offered THIS frame (`E`, `F`, `CH`, …).
    shown_features: Vec<String>,
    /// The constraint TYPE ids offered THIS frame (`fixed`, `distance`, …).
    shown_constraints: Vec<String>,
    /// The PMI annotation type ids offered THIS frame (`linear`, `datum`, …).
    shown_pmi: Vec<String>,
    /// The COMPONENT action ids offered THIS frame (`move`, `open-part`, …)
    /// — non-empty exactly when the selection is a single component's members.
    shown_component_actions: Vec<String>,
    /// The single component the actions target this frame (its ACOMP id).
    shown_component_target: Option<String>,
}

impl ContextBarPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the context bar as a FLOATING panel over the viewport (nothing when
    /// nothing is selected — like the old app's floating selection action bar).
    /// Drawn at ctx level (not inside the scrollable side panel) so its buttons
    /// are always reachable regardless of side-panel scroll. Returns a
    /// [`ContextOutcome`] — the feature id the shell should expand in the history
    /// tree (after a create-from-selection or edit-owning action) plus any entity
    /// names the shell should open pinned Info windows for (after the Info action).
    pub fn card(&mut self, ui: &mut egui::Ui, state: &mut EngineState) -> ContextOutcome {
        self.hits.clear();
        self.shown_actions.clear();
        self.shown_features.clear();
        self.shown_constraints.clear();
        self.shown_pmi.clear();
        self.shown_component_actions.clear();
        self.shown_component_target = None;

        // Modeling context actions ONLY. Hidden with no selection (geometry OR a
        // label-selected constraint), and never during reference-selection (the
        // picker owns the selection), in sketch mode (the sketch context rail
        // replaces this one) or over an eCAD editor (below). Rendered through the SHARED single-column rail —
        // see [`super::action_rail`] — so it and the sketch context bar stay
        // identical. The constraint selection only counts (and only offers its
        // Delete action) in a workbench that shows the constraints panel — the
        // same claim gate as the constraint offers.
        let has_geometry = state.has_selection();
        let constraint_target = state.selected_constraint().filter(|_| {
            crate::workbench::panel_visible(
                &state.settings.workbench,
                crate::workbench::assembly::CONSTRAINTS_PANEL_ID,
                &crate::workbench::ButtonState::of(state),
            )
        });
        // Nor while an eCAD editor owns the central tile (Diagram, PCB,
        // Symbol, Pads): the card is anchored to that tile's corner and its
        // actions are all 3D ones, so a 3D selection carried across a workbench
        // switch — the Qualify panel's jump from a picked point, or a face picked
        // in Modeling — would otherwise stay drawn over the editor, beside the
        // editor's own tool card. The selection itself is kept for the return.
        if (!has_geometry && constraint_target.is_none())
            || state.ref_select_active()
            || state.sketch_mode()
            || crate::workbench::ecad::Target::of_workbench(&state.settings.workbench).is_some()
        {
            return ContextOutcome::default();
        }

        let sel = Selection::read(state);
        let comp = component_selection(&sel, state);
        let probe = selection_probe(&sel, &comp, all_on_sheet_metal(&sel, state));
        // The feature FENCE: a selection made ENTIRELY of component geometry
        // offers NO modeling-feature creation (the kernel rejects component
        // references anyway — don't offer dead ends). The constraint offers
        // are the complement: their predicates REQUIRE an all-component
        // selection, so the two sets never coexist.
        let offers = if comp.suppress_features() {
            Vec::new()
        } else {
            feature_offers(&probe, &sel, &state.settings.workbench)
        };
        let constraint_types =
            constraint_offers(&probe, &state.settings.workbench, &crate::workbench::ButtonState::of(state));
        // PMI annotation offers: the third applicability family — each type's
        // own predicate on the probe, plain and component geometry alike —
        // gated on the PMI panel's workbench visibility AND an active view
        // (creation is gated on a view to annotate).
        let pmi_types = pmi_offers(
            &probe,
            &state.settings.workbench,
            state.pmi_active_view().is_some(),
            &crate::workbench::ButtonState::of(state),
        );
        // A single component's member solid(s) selected → the COMPONENT action
        // set replaces the feature-creation offers — but ONLY in a workbench
        // that shows the assembly structure panel (claim-based: Assembly +
        // All). The component actions are that panel's row actions, so they
        // follow its visibility and never bleed into Modeling / Sheet Metal;
        // the STANDARD actions (Clear / Hide / Info / Edit owning) still apply
        // to a component selection in every workbench.
        let component_target = component_action_target(
            &comp,
            &state.settings.workbench,
            &crate::workbench::ButtonState::of(state),
        )
        .map(|id| {
            let fixed = state.component_info(id).map(|info| info.fixed).unwrap_or(false);
            (id.to_string(), fixed)
        });

        // Build the action items: the generic actions, then feature-from-selection.
        // Info (🕵 U+1F575, the previous app's "Inspector, Metadata & Mass Properties"
        // glyph, from the bundled Noto Sans Symbols 2 font) opens one PINNED Info
        // window per selected entity — unlike the other actions it drives no engine
        // mutation; the shell opens the windows from the returned targets.
        let mut items = vec![ActionItem::new(
            "action:clear",
            "\u{2716} Clear",
            "Clear the selection",
        )];
        self.shown_actions.push("clear".into());
        // Hide + Info act on selected GEOMETRY — with only a constraint
        // selected they would be no-ops, so they are not offered.
        if has_geometry {
            items.push(ActionItem::new("action:hide", "\u{1f441} Hide", "Hide/Show selection"));
            items.push(ActionItem::new(
                "action:info",
                "\u{1f575} Info",
                "Open a pinned Info window per selected entity",
            ));
            self.shown_actions.push("hide".into());
            self.shown_actions.push("info".into());
        }
        // The label-selected CONSTRAINT's action: delete it (the panel's row ✕,
        // reachable from the viewport).
        if let Some(cid) = &constraint_target {
            items.push(ActionItem::new(
                "action:delete-constraint",
                "\u{2715} Delete constraint",
                format!("Delete constraint {cid}"),
            ));
            self.shown_actions.push("delete-constraint".into());
        }
        if let Some(item) = edit_owning_item(&sel) {
            items.push(item);
            self.shown_actions.push("edit-owning".into());
        }
        // Component actions: shown INSTEAD of the feature offers when the
        // selection is exactly one component's member solid(s).
        if let Some((target, fixed)) = &component_target {
            for action in ComponentAction::ALL {
                items.push(ActionItem::new(
                    format!("component:{}", action.id()),
                    action.label(*fixed),
                    action.tooltip(),
                ));
                self.shown_component_actions.push(action.id().to_string());
            }
            self.shown_component_target = Some(target.clone());
        }
        // Constraint offers (all-component selections in a workbench that shows
        // the constraints panel): one button per applicable constraint type.
        for def in &constraint_types {
            items.push(ActionItem::new(
                format!("constraint:{}", def.type_id),
                def.long_name,
                format!("Add a {} constraint from the selection", def.label),
            ));
            self.shown_constraints.push(def.type_id.to_string());
        }
        for offer in &offers {
            items.push(ActionItem::new(
                format!("feature:{}", offer.type_code),
                offer.label.clone(),
                format!("Create {} from the selection", offer.label),
            ));
            self.shown_features.push(offer.type_code.clone());
        }
        for def in &pmi_types {
            items.push(ActionItem::new(
                format!("pmi:{}", def.type_id),
                def.long_name,
                format!("Add a {} to the active PMI view from the selection", def.label),
            ));
            self.shown_pmi.push(def.type_id.to_string());
        }

        // With only a constraint selected the geometry summary would read all
        // zeros — name the constraint instead.
        let summary = match (&constraint_target, has_geometry) {
            (Some(cid), false) => format!("Selected: constraint {cid}"),
            _ => sel.summary(),
        };
        let clicked = egui::Frame::popup(ui.style())
            .show(ui, |ui| {
                action_rail(
                    ui,
                    Some("Selection actions"),
                    Some(&summary),
                    &items,
                    &mut self.hits,
                )
            })
            .inner;

        // --- apply the intent (one engine mutation per frame) -----------------
        let mut outcome = ContextOutcome::default();
        match clicked.as_deref() {
            Some("action:clear") => {
                // Also drops a label-selected constraint (clear_selection folds
                // the constraint selection in).
                state.clear_selection();
            }
            Some("action:delete-constraint") => {
                if let Some(cid) = &constraint_target {
                    let _ = state.assembly_remove_constraint(cid);
                    state.constraint_deselect();
                }
            }
            Some("action:hide") => {
                state.hide_selected();
            }
            Some("action:info") => {
                // No engine mutation — hand the shell one target per selected entity
                // so it opens (or, on dedup, keeps) a pinned Info window for each.
                outcome.info_targets = sel.all_names();
            }
            Some("action:edit-owning") => {
                // A bundle's button is disabled; a scripted click by key lands
                // here anyway and gets the same answer as the tooltip.
                if let Some(spline) = sel.harness_bundle() {
                    state.push_notice(bundle_edit_hint(&spline));
                } else if let Some(fid) = sel.owning_feature.clone() {
                    if let Some(index) = feature_index(state, &fid) {
                        state.roll_to(index);
                    }
                    outcome.focus = Some(fid);
                }
            }
            Some(key) if key.starts_with("component:") => {
                if let Some((target, _)) = &component_target {
                    if let Some(action) = ComponentAction::from_id(&key["component:".len()..]) {
                        outcome.component = run_component_action(state, action, target);
                    }
                }
            }
            Some(key) if key.starts_with("constraint:") => {
                let type_id = &key["constraint:".len()..];
                if constraint_types.iter().any(|def| def.type_id == type_id) {
                    if let Err(error) = add_constraint_from_selection(state, type_id) {
                        state.push_notice(format!("Add constraint: {error}"));
                    }
                }
            }
            Some(key) if key.starts_with("feature:") => {
                let code = &key["feature:".len()..];
                if let Some(offer) = offers.iter().find(|o| o.type_code == code) {
                    outcome.focus = create_feature_from_selection(state, offer, &sel);
                }
            }
            Some(key) if key.starts_with("pmi:") => {
                let type_id = &key["pmi:".len()..];
                if pmi_types.iter().any(|def| def.type_id == type_id) {
                    // The add leaves the new annotation OPEN, so the dock's
                    // dialog door surfaces the PMI pane; there is nothing for
                    // this outcome to carry.
                    if let Err(error) = add_pmi_from_selection(state, type_id) {
                        state.push_notice(format!("Add PMI: {error}"));
                    }
                }
            }
            _ => {}
        }
        outcome
    }

    /// The published widget hit-rects (egui points) for the headed verifier —
    /// `action:clear|action:hide|action:edit-owning` + `feature:<TYPE>`.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// The bar's LOGICAL state for the verifier: whether it is shown + which
    /// generic actions, feature type-codes, and component actions it offered
    /// this frame (and the single component the latter target).
    pub fn state_json(&self) -> String {
        serde_json::json!({
            "shown": !self.hits.is_empty(),
            "actions": self.shown_actions,
            "features": self.shown_features,
            "constraints": self.shown_constraints,
            "pmi": self.shown_pmi,
            "componentActions": self.shown_component_actions,
            "componentTarget": self.shown_component_target,
        })
        .to_string()
    }
}

/// The COMPONENT view of the current selection: which ACOMP instances own the
/// selected entities, and whether the selection qualifies for the component
/// action set / the feature-offer fence.
struct ComponentSelection {
    /// Unique owning ACOMP ids across every selected NAMED entity, selection
    /// order.
    ids: Vec<String>,
    /// Whether EVERY selected named entity is component-owned (and at least one
    /// is selected; vertices carry no names, so any vertex disqualifies).
    all_component: bool,
    /// Whether the selection is member SOLIDS only (the shape a viewport
    /// component click produces).
    solids_only: bool,
}

impl ComponentSelection {
    /// The feature FENCE: suppress modeling-feature creation offers when the
    /// whole selection is component geometry.
    fn suppress_features(&self) -> bool {
        self.all_component && !self.ids.is_empty()
    }

    /// The single component the ACTION SET targets: exactly one owning
    /// component, selected via its member solid(s) alone.
    fn sole_target(&self) -> Option<&str> {
        (self.suppress_features() && self.solids_only && self.ids.len() == 1)
            .then(|| self.ids[0].as_str())
    }
}

/// Resolve the selection's component ownership through the engine's namespace
/// parse (`component_of_solid` accepts any namespaced entity name — solid,
/// face, or edge).
fn component_selection(sel: &Selection, state: &EngineState) -> ComponentSelection {
    let mut ids: Vec<String> = Vec::new();
    let mut all = true;
    let mut any = false;
    for name in sel.all_names() {
        any = true;
        match state.component_of_solid(&name) {
            Some(id) => {
                if !ids.contains(&id) {
                    ids.push(id);
                }
            }
            None => all = false,
        }
    }
    if sel.vertices > 0 {
        all = false;
    }
    ComponentSelection {
        ids,
        all_component: all && any,
        solids_only: !sel.solids.is_empty()
            && sel.sketches.is_empty()
            && sel.faces.is_empty()
            && sel.edges.is_empty()
            && sel.vertices == 0,
    }
}

/// The **Edit owning feature** button for `sel`, if the selection has an owning
/// feature: enabled and rolling to the producer, or — for a wire-harness bundle
/// solid, whose producer is the harness tail and not a history feature —
/// disabled with the tooltip saying what to edit instead.
fn edit_owning_item(sel: &Selection) -> Option<ActionItem> {
    sel.owning_feature.as_ref()?;
    let item = match sel.harness_bundle() {
        Some(spline) => ActionItem::new(
            "action:edit-owning",
            "Edit owning feature",
            bundle_edit_hint(&spline),
        )
        .enabled(false),
        None => ActionItem::new(
            "action:edit-owning",
            "Edit owning feature",
            "Roll to and edit the feature that created this",
        ),
    };
    Some(item)
}

/// What to edit instead of a bundle solid's (non-existent) owning feature.
fn bundle_edit_hint(spline: &str) -> String {
    format!(
        "This bundle is built by the wire harness, not by a feature. \
         Edit spline {spline} (the path it follows) or the wires in the Wire Harness panel instead"
    )
}

/// The current selection, resolved once per frame from `selection_json`, plus the
/// single-selection owning feature (for **Edit owning feature**).
struct Selection {
    solids: Vec<String>,
    /// Selected COMMITTED SKETCHES. A committed sketch presents in the scene as a
    /// solid (`is_sketch`), so it arrives in `selection_json`'s `solids` array; we
    /// partition it out here because its reference KIND is `SKETCH`, not `SOLID`
    /// (it must satisfy a `["FACE","SKETCH"]` profile field, and must NOT satisfy a
    /// `["SOLID"]` field like SM Cutout's `sheet`).
    sketches: Vec<String>,
    faces: Vec<String>,
    edges: Vec<String>,
    /// Selected construction PLANES / DATUM planes (their scene FRAME names, from
    /// `selection_json`'s `datums` array). Kept a SEPARATE bucket from `faces`:
    /// only [`kinds_present`](Self::kinds_present) / [`names_for_filter`](Self::
    /// names_for_filter) / the probe read it — NEVER the scene-solid consumers
    /// (`all_names`, Info, Hide, `component_selection`), which cannot resolve a
    /// datum frame name. A datum plane seats a sketch's `sketchPlane` exactly like
    /// a planar face (the kernel resolves either).
    planes: Vec<String>,
    vertices: usize,
    /// The producer feature id of a SINGLE-entity selection with a known producer.
    owning_feature: Option<String>,
}

impl Selection {
    /// When the single selected entity is (or belongs to) a wire-harness BUNDLE
    /// solid — its producer is the harness tail, never a history feature — the
    /// id of the spline the bundle follows (`WireHarness:SP1` → `SP1`). `None`
    /// for every other selection.
    fn harness_bundle(&self) -> Option<String> {
        if self.owning_feature.as_deref() != Some(brep_kernel::WIRE_HARNESS_FEATURE_ID) {
            return None;
        }
        let name = self
            .solids
            .first()
            .or_else(|| self.faces.first())
            .or_else(|| self.edges.first())?;
        // A face of the bundle is `WireHarness:SP1:Wall0`; the spline is the
        // segment between the prefix and the next colon.
        let rest = name.strip_prefix(brep_kernel::BUNDLE_SOLID_PREFIX)?;
        let spline = rest.split(':').next().filter(|id| !id.is_empty())?;
        Some(spline.to_string())
    }

    fn read(state: &EngineState) -> Self {
        let v: Value = serde_json::from_str(&state.selection_json()).unwrap_or(Value::Null);
        let names = |key: &str| -> Vec<String> {
            v[key]
                .as_array()
                .map(|a| a.iter().filter_map(|x| x.as_str().map(String::from)).collect())
                .unwrap_or_default()
        };
        // Partition the selected `solids` into REAL solids vs committed sketches: a
        // selected solid is a sketch iff its name is a committed sketch (the
        // sketch's selectable name is its id; visibility is irrelevant here).
        let sketch_ids: std::collections::HashSet<String> = state
            .committed_sketches()
            .into_iter()
            .map(|(id, _visible)| id)
            .collect();
        let (sketches, solids): (Vec<String>, Vec<String>) = names("solids")
            .into_iter()
            .partition(|name| sketch_ids.contains(name));
        let faces = names("faces");
        let edges = names("edges");
        // Construction planes / datum planes arrive under `datums` — a SEPARATE
        // bucket (never merged into `faces`): the scene-solid consumers cannot
        // resolve a datum frame name (see the `planes` field doc).
        let planes = names("datums");
        let vertices = v["vertices"].as_u64().unwrap_or(0) as usize;

        // A single selected entity → its owning feature (the old app's
        // Edit-owning-feature, generalized from FACE/PLANE to any single entity —
        // a lone selected sketch rolls to its `S` feature, a lone datum/plane to
        // its `D`/`P` feature). Datum planes count toward the single-selection
        // total too, else picking one shows no Edit-owning-feature button.
        let total = solids.len() + sketches.len() + faces.len() + edges.len() + planes.len();
        let single = if total == 1 && vertices == 0 {
            faces
                .first()
                .or_else(|| edges.first())
                .or_else(|| solids.first())
                .or_else(|| sketches.first())
                .or_else(|| planes.first())
                .cloned()
        } else {
            None
        };
        let owning_feature = single
            .as_deref()
            .and_then(|name| state.creating_feature(name))
            .map(|(id, _ty)| id);

        Self {
            solids,
            sketches,
            faces,
            edges,
            planes,
            vertices,
            owning_feature,
        }
    }

    /// The selectable KINDS currently present (vertices carry no names, and no
    /// primary reference is vertex-only, so they never drive feature actions).
    fn kinds_present(&self) -> Vec<&'static str> {
        let mut kinds = Vec::new();
        if !self.solids.is_empty() {
            kinds.push("SOLID");
        }
        if !self.sketches.is_empty() {
            kinds.push("SKETCH");
        }
        if !self.faces.is_empty() {
            kinds.push("FACE");
        }
        if !self.edges.is_empty() {
            kinds.push("EDGE");
        }
        // Datum planes and `P` planes both present as ONE kind, `PLANE` — the
        // schema filters spell it `["PLANE","FACE"]`, and both resolve as frames.
        if !self.planes.is_empty() {
            kinds.push("PLANE");
        }
        kinds
    }

    /// The selected names whose kind the reference `filter` accepts (de-duplicated,
    /// in solid→face→edge order). `PLANE`/`DATUM` map to selected datum/plane
    /// frames, `COMPONENT` to selected solids (the picker never yields a bare
    /// component here).
    fn names_for_filter(&self, filter: &[String]) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        let push = |src: &[String], out: &mut Vec<String>| {
            for name in src {
                if !out.iter().any(|n| n == name) {
                    out.push(name.clone());
                }
            }
        };
        for f in filter {
            match f.as_str() {
                "SOLID" | "COMPONENT" => push(&self.solids, &mut out),
                "SKETCH" => push(&self.sketches, &mut out),
                "FACE" => push(&self.faces, &mut out),
                // A `["PLANE","FACE"]` field prefills from EITHER a selected face
                // (via the FACE arm) or a selected datum/plane frame here; `DATUM`
                // is an alias for the same planes bucket.
                "PLANE" | "DATUM" => push(&self.planes, &mut out),
                "EDGE" => push(&self.edges, &mut out),
                _ => {}
            }
        }
        out
    }

    /// Every NAMED selected entity (solids → faces → edges), de-duplicated — one per
    /// pinned Info window the Info action opens. Vertices carry no name, and datum
    /// PLANES are deliberately EXCLUDED: this list feeds the scene-solid consumers
    /// (Info, Hide via `hide_selected`, `component_selection` via
    /// `component_of_solid`), none of which can resolve a datum frame name. A datum
    /// plane can be selected (`has_selection` now counts it, so the bar shows and
    /// offers Sketch), but it only reaches `kinds_present` / `names_for_filter` /
    /// the probe — never this list.
    fn all_names(&self) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for src in [&self.solids, &self.sketches, &self.faces, &self.edges] {
            for name in src {
                if !name.is_empty() && !out.iter().any(|n| n == name) {
                    out.push(name.clone());
                }
            }
        }
        out
    }

    fn summary(&self) -> String {
        format!(
            "Selected: {} solid, {} sketch, {} face, {} edge, {} plane, {} vertex",
            self.solids.len(),
            self.sketches.len(),
            self.faces.len(),
            self.edges.len(),
            self.planes.len(),
            self.vertices,
        )
    }
}

/// One reference field of an offered feature, in schema order — the pre-fill
/// targets [`prefill_references`] consumes the selection into.
struct OfferField {
    /// The JSON path of the `References`-group field.
    path: Vec<String>,
    /// That field's `selectionFilter` (which selected kinds map into it).
    filter: Vec<String>,
    /// Whether the field takes a list (vs a single name).
    multiple: bool,
}

/// One offered feature action.
struct Offer {
    /// The feature TYPE CODE (e.g. `E`, `F`, `CH`).
    type_code: String,
    /// The button label (the feature's long name).
    label: String,
    /// Every `References`-group field whose filter accepts a selected kind
    /// (schema order) — the create pre-fills them under a consumed set.
    fields: Vec<OfferField>,
}

/// Build the [`SelectionProbe`] the kernel applicability predicates run on:
/// the selection's kind counts, its component view, and whether it sits entirely
/// on sheet metal ([`all_on_sheet_metal`], the gate for the SM edit features).
fn selection_probe(
    sel: &Selection,
    comp: &ComponentSelection,
    all_sheet_metal: bool,
) -> SelectionProbe {
    SelectionProbe {
        solids: sel.solids.len(),
        sketches: sel.sketches.len(),
        faces: sel.faces.len(),
        edges: sel.edges.len(),
        planes: sel.planes.len(),
        vertices: sel.vertices,
        components: comp.ids.len(),
        all_component: comp.all_component,
        all_sheet_metal,
    }
}

/// Whether the selection sits ENTIRELY on sheet-metal bodies (and names at least
/// one entity) — the gate the SM edit features (Flange / Fillet / Chamfer) key
/// on. Mirrors [`component_selection`]'s all-or-nothing rule, including its
/// vertex convention: a vertex carries no name to resolve, so any vertex in the
/// selection disqualifies it.
fn all_on_sheet_metal(sel: &Selection, state: &EngineState) -> bool {
    let names = sel.all_names();
    !names.is_empty()
        && sel.vertices == 0
        && names.iter().all(|name| state.is_sheet_metal_object(name))
}

/// The feature actions to offer: every catalogue feature whose OWN
/// `context_applicable` predicate (kernel-defined, next to its schema —
/// `feature_pipeline::context_offer`) accepts the current selection probe. The
/// `workbench` argument only FURTHER RESTRICTS that set to the features the
/// active workbench includes; like the palette filter it is a pure UI trim over
/// CREATION and never affects the existing history / execution.
///
/// The pre-fill stays schema-derived: each offer carries EVERY
/// `References`-group `reference_selection` field whose `selectionFilter`
/// intersects a selected kind (schema order), and the create consumes the
/// selection into them ([`prefill_references`]).
fn feature_offers(probe: &SelectionProbe, sel: &Selection, workbench: &str) -> Vec<Offer> {
    let kinds = sel.kinds_present();
    if kinds.is_empty() {
        return Vec::new();
    }
    let catalogue = features::feature_catalogue();
    let mut out = Vec::new();
    if let Some(list) = catalogue.get("features").and_then(Value::as_array) {
        for feature in list {
            let Some(ty) = feature.get("type").and_then(Value::as_str) else {
                continue;
            };
            if ty.is_empty() {
                continue;
            }
            // Workbench UI filter: skip features this workbench does not include
            // (classified off the type code).
            if !crate::workbench::includes_feature(workbench, ty) {
                continue;
            }
            // The feature's own answer to "does this selection make me
            // meaningful?" — nuance (Revolve wants profile AND axis) lives in
            // the kernel predicate, not here.
            if !brep_kernel::feature_context_applicable(ty, probe) {
                continue;
            }
            // The pre-fill targets: every `References`-group reference field
            // accepting a selected kind. Primitives only carry the boolean-op
            // `targets` Reference (group `Boolean`), so they never collect any
            // (their predicates return false anyway).
            let fields: Vec<OfferField> = features::feature_form_fields(ty)
                .iter()
                .filter(|field| field.group == "References")
                .filter_map(|field| {
                    let FieldKind::Reference { filter, multiple } = &field.kind else {
                        return None;
                    };
                    filter
                        .iter()
                        .any(|f| kinds.iter().any(|k| *k == f.as_str()))
                        .then(|| OfferField {
                            path: field.path.clone(),
                            filter: filter.clone(),
                            multiple: *multiple,
                        })
                })
                .collect();
            out.push(Offer {
                type_code: ty.to_string(),
                label: features::feature_long_name(ty),
                fields,
            });
        }
    }
    out
}

/// The single component the context bar's COMPONENT action set targets, or
/// `None` when the selection shape doesn't qualify ([`ComponentSelection::
/// sole_target`]) OR the active workbench hides the assembly structure panel
/// (claim-based visibility, [`crate::workbench::panel_visible`]: Assembly +
/// All). The workbench gate is what keeps the Move / Edit-in-place / Open-Part
/// / Fix / Delete buttons — assembly UI — out of the Modeling context bar; the
/// feature FENCE (`suppress_features`) is intentionally NOT gated, since the
/// kernel rejects component references in every workbench.
fn component_action_target<'a>(
    comp: &'a ComponentSelection,
    workbench: &str,
    panels: &crate::workbench::ButtonState,
) -> Option<&'a str> {
    // The BOM is the assembly workbench's component list (it absorbed the
    // Structure panel): component actions target a selection only where that
    // list is on screen.
    let list_shown = crate::workbench::panel_visible(
        workbench,
        crate::workbench::assembly::BOM_PANEL_ID,
        panels,
    );
    list_shown.then(|| comp.sole_target()).flatten()
}

/// The PMI annotation actions to offer: every type whose `applicable`
/// predicate ([`brep_kernel::PMI_TYPES`]) accepts the probe — gated on the PMI
/// panel being available in the active workbench (PMI + All) and on an active
/// view (no view ⇒ no offers).
fn pmi_offers(
    probe: &SelectionProbe,
    workbench: &str,
    view_active: bool,
    panels: &crate::workbench::ButtonState,
) -> Vec<&'static brep_kernel::PmiTypeDef> {
    if !view_active
        || !crate::workbench::panel_visible(workbench, crate::workbench::pmi::PANEL_ID, panels)
    {
        return Vec::new();
    }
    brep_kernel::PMI_TYPES
        .iter()
        .filter(|def| (def.applicable)(probe))
        .collect()
}

/// Add a PMI annotation of `type_id` to the active view, its reference
/// fields pre-seeded from the selection by the schema's `selectionFilter`s
/// (faces / edges / datum planes / solids by name, vertices as `{solid}@x,y,z`
/// world refs). The annotation plane is never seeded — a selected face is
/// the geometry being annotated; the plane is picked in the form. The new
/// annotation's form opens (the engine sets the open annotation).
pub(crate) fn add_pmi_from_selection(state: &mut EngineState, type_id: &str) -> Result<String, String> {
    let catalogue = brep_kernel::pmi_schema_catalogue();
    let schema = catalogue
        .as_array()
        .and_then(|entries| entries.iter().find(|e| e.get("type").and_then(Value::as_str) == Some(type_id)))
        .cloned()
        .ok_or_else(|| format!("no schema for PMI type '{type_id}'"))?;
    let selection: Value = serde_json::from_str(&state.selection_json()).unwrap_or(Value::Null);
    let names = |key: &str| -> Vec<String> {
        selection[key]
            .as_array()
            .map(|items| items.iter().filter_map(|item| item.as_str().map(String::from)).collect())
            .unwrap_or_default()
    };
    let vertices: Vec<String> = state
        .emphasis
        .selected_vertices
        .iter()
        .map(|vertex| brep_render::engine_state::world_vertex_ref(&vertex.solid, vertex.position))
        .collect();
    let mut seeded = serde_json::Map::new();
    let mut consumed: Vec<String> = Vec::new();
    if let Some(fields) = schema.get("inputParamsSchema").and_then(Value::as_object) {
        for (key, spec) in fields {
            if spec.get("type").and_then(Value::as_str) != Some("reference_selection") || key == "plane" {
                continue;
            }
            let filter: Vec<String> = spec
                .get("selectionFilter")
                .and_then(Value::as_array)
                .map(|kinds| kinds.iter().filter_map(|k| k.as_str().map(String::from)).collect())
                .unwrap_or_default();
            let multiple = spec.get("multiple").and_then(Value::as_bool).unwrap_or(false);
            let cap = spec.get("maxSelections").and_then(Value::as_u64).unwrap_or(if multiple { 64 } else { 1 }) as usize;
            let mut picked: Vec<String> = Vec::new();
            for kind in &filter {
                let source: Vec<String> = match kind.to_ascii_uppercase().as_str() {
                    "FACE" => names("faces"),
                    "EDGE" => names("edges"),
                    "PLANE" | "DATUM" => names("datums"),
                    "SOLID" | "COMPONENT" => names("solids"),
                    "VERTEX" => vertices.clone(),
                    _ => Vec::new(),
                };
                for name in source {
                    if picked.len() < cap && !picked.contains(&name) && !consumed.contains(&name) {
                        picked.push(name);
                    }
                }
            }
            if picked.is_empty() {
                continue;
            }
            consumed.extend(picked.iter().cloned());
            let value = if multiple {
                Value::Array(picked.into_iter().map(Value::String).collect())
            } else {
                Value::String(picked.remove(0))
            };
            seeded.insert(key.clone(), value);
        }
    }
    state.pmi_add_annotation(None, type_id, &Value::Object(seeded).to_string())
}

/// The constraint actions to offer: every constraint type whose `applicable`
/// predicate ([`brep_kernel::CONSTRAINT_TYPES`], defined with the type table)
/// accepts the probe — gated on the Assembly Constraints panel being available
/// in the active workbench (claim-based visibility: Assembly + All).
fn constraint_offers(
    probe: &SelectionProbe,
    workbench: &str,
    panels: &crate::workbench::ButtonState,
) -> Vec<&'static brep_kernel::ConstraintTypeDef> {
    if !crate::workbench::panel_visible(
        workbench,
        crate::workbench::assembly::CONSTRAINTS_PANEL_ID,
        panels,
    ) {
        return Vec::new();
    }
    brep_kernel::CONSTRAINT_TYPES
        .iter()
        .filter(|def| (def.applicable)(probe))
        .collect()
}

/// Add a constraint of `type_id` from the selection: `elements` pre-seeded
/// through the constraints panel's seeding helper (filtered + capped by the
/// type's own schema), then the new row opened so the panel shows its dialog.
/// The engine's mutation path handles auto-solve exactly like a panel add.
pub(crate) fn add_constraint_from_selection(
    state: &mut EngineState,
    type_id: &str,
) -> Result<String, String> {
    let catalogue = brep_kernel::constraint_schema_catalogue();
    let schemas: Vec<Value> = catalogue.as_array().cloned().unwrap_or_default();
    let seed = super::assembly_constraints::seeded_elements(state, &schemas, type_id);
    let id = state.assembly_add_constraint(type_id, &seed.to_string())?;
    let _ = state.assembly_set_constraint_open(&id, true);
    Ok(id)
}

/// Consume the selection into an offer's reference fields, schema order: each
/// field takes the selected names its filter accepts that NO EARLIER field
/// consumed (first name for a single field, all remaining for a multiple) — so
/// face+edge → Revolve fills `profile` with the face and `axis` with the edge,
/// and Pattern's edge lands in `directionRef` without echoing into `axisRef`.
/// Returns `(path, value)` writes for [`form::set_at`].
fn prefill_references(fields: &[OfferField], sel: &Selection) -> Vec<(Vec<String>, Value)> {
    let mut consumed: HashSet<String> = HashSet::new();
    let mut writes = Vec::new();
    for field in fields {
        let names: Vec<String> = sel
            .names_for_filter(&field.filter)
            .into_iter()
            .filter(|name| !consumed.contains(name))
            .collect();
        if names.is_empty() {
            continue;
        }
        let value = if field.multiple {
            consumed.extend(names.iter().cloned());
            Value::Array(names.into_iter().map(Value::String).collect())
        } else {
            let name = names.into_iter().next().unwrap_or_default();
            consumed.insert(name.clone());
            Value::String(name)
        };
        writes.push((field.path.clone(), value));
    }
    writes
}

/// Create a feature of `offer.type_code` referencing the selection: build a
/// fresh descriptor whose `inputParams` are the schema defaults with an
/// engine-unique `id` and the matched reference fields pre-filled
/// ([`prefill_references`]), then append it (`add_feature`, which rolls to it).
/// Returns the new feature id (for the shell to expand its node).
fn create_feature_from_selection(
    state: &mut EngineState,
    offer: &Offer,
    sel: &Selection,
) -> Option<String> {
    let id = state.next_feature_id(&features::feature_short_name(&offer.type_code));
    let mut params = features::feature_default_params(&offer.type_code);
    if let Value::Object(map) = &mut params {
        map.insert("id".into(), Value::String(id.clone()));
    }

    for (path, value) in prefill_references(&offer.fields, sel) {
        form::set_at(&mut params, &path, value);
    }

    let feature = serde_json::json!({
        "type": offer.type_code,
        "inputParams": params,
        "persistentData": {},
    });
    if state.add_feature(&feature.to_string()).is_ok() {
        Some(id)
    } else {
        None
    }
}

/// The feature index carrying id `id` (the engine exposes index→id, so we scan).
fn feature_index(state: &EngineState, id: &str) -> Option<usize> {
    (0..state.history_len()).find(|&i| state.feature_id_at(i).as_deref() == Some(id))
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    // The generic actions were published from the day the bar was built and
    // documented by nobody, so `hit_keys_check` reported four undocumented keys
    // the moment a script left something selected while the bar was up — which
    // no script had done before the verifier migration.
    HitKeyDoc { panel: "context", prefix: "action:clear", meaning: "clear the selection", command: Some("select_clear") },
    HitKeyDoc { panel: "context", prefix: "action:hide", meaning: "hide or show the selected geometry", command: Some("set_visible") },
    HitKeyDoc { panel: "context", prefix: "action:info", meaning: "open one pinned Info window per selected entity (mass properties, topology, metadata)", command: None },
    HitKeyDoc { panel: "context", prefix: "action:delete-constraint", meaning: "delete the assembly constraint whose viewport label is selected", command: Some("assembly_remove_constraint") },
    HitKeyDoc { panel: "context", prefix: "action:edit-owning", meaning: "roll to and edit the feature that created the selection (disabled for a wire-harness bundle solid, which no history feature owns)", command: None },
    HitKeyDoc { panel: "context", prefix: "feature:", meaning: "add the offered feature from the selection (feature:type)", command: None },
    HitKeyDoc { panel: "context", prefix: "component:", meaning: "a component action", command: None },
    HitKeyDoc { panel: "context", prefix: "constraint:", meaning: "add the offered assembly constraint", command: None },
    HitKeyDoc { panel: "context", prefix: "pmi:", meaning: "add the offered PMI annotation (pmi:type)", command: None },
    HitKeyDoc { panel: "context", prefix: "Selected:", meaning: "the selection summary chip", command: None },
];
