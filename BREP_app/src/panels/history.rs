//! History panel — a feature **tree** that switches to a full-panel **form**.
//!
//! The panel has exactly two modes ([`PanelMode`]):
//!
//! * **Tree** — one row per feature: `[+/-] id LongName   N ms  [✎] [✕]`. The
//!   collapse box is a PURE ROLL control (roll the model to that step, nothing
//!   opens); a SINGLE CLICK on the row selects the feature
//!   (`EngineState::select_feature` — the geometry it made becomes the viewport
//!   selection, nothing rolls); the `✎` edit button and a DOUBLE CLICK on the
//!   row both open that feature's form (and roll to it); rows drag-reorder;
//!   `✕` deletes. Built on the reusable [`tree`] node helper (connector lines +
//!   `[+]`/`[-]` boxes) the rest of the sidebar reuses.
//! * **Form** — the whole panel is replaced by ONE feature's dialog, drawn by
//!   the shared [`crate::form_view`], with a single `Return to tree` button at
//!   the top right, beside the title. There is no OK/Cancel and no buffer:
//!   editing is LIVE and UNDO
//!   is the revert mechanism (`History::set_feature_params` checkpoints and
//!   coalesces per feature).
//!
//! Both ways into the form roll the model to the feature, exactly as expanding did;
//! returning to the tree rolls to the TIP so downstream features rebuild and the
//! edit becomes visible.
//!
//! This panel OWNS NO model state — it calls the ENGINE's history methods
//! (`state.*`) and reads the history + last-run report back to draw. The engine
//! core (`EngineState.history`) is the single source of truth. The panel holds
//! only transient UI state: the mode, an in-flight drag, the add-menu toggle,
//! and the per-frame `hits` map (widget screen rects) the
//! headed verifier reads to drive real clicks.
//!
//! The tree also shows HOW FAR the model is built: the rolled-to feature is the
//! last one EXECUTED, so a rollback BAR is drawn under its block, every feature
//! below that bar — not yet executed — is dimmed, and its `[+]`/`[-]` box reads
//! `[-]` down to the rolled-to step and `[+]` below it.

use crate::automation::hit_keys::HitKeyDoc;
use crate::form_view::{form_view, FormViewSpec};
use crate::panels::spline_anchors;
use crate::palette::{Palette, PaletteItem};
use crate::panels::tree::{self, TreeRow};
use brep_render::engine_state::EngineState;
use brep_render::features;
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

/// Who this panel is when it drives the viewport's dialog-row hover
/// (`EngineState::hover_entity_by_name`) — the owner tag that keeps its
/// highlight independent of the Scene tree's and the other form panes'.
const DIALOG_HOVER_OWNER: &str = "history";

/// The red of the per-feature delete affordance (theme-independent — it must read
/// as "destructive" in both light and dark).
const DELETE_RED: egui::Color32 = egui::Color32::from_rgb(0xd8, 0x54, 0x4f);

/// The red of a feature's error message node — a brighter, clearly-legible red for
/// wrapped body text (the delete red is tuned for a small glyph). Matches the
/// hardcoded-chrome-red convention of `DELETE_RED`.
const ERROR_RED: egui::Color32 = egui::Color32::from_rgb(0xff, 0x6b, 0x6b);

/// The amber of a feature's REPAIR note — the feature succeeded, with a result
/// the kernel changed on its way out (a self-crossing split and re-trimmed
/// rather than refused). Read as "look at this", not as "this failed", which is
/// why it is not [`ERROR_RED`].
const NOTE_AMBER: egui::Color32 = egui::Color32::from_rgb(0xe8, 0xa3, 0x3d);

/// The orange of a feature's PARTIAL FULFILMENT — the feature succeeded, on
/// fewer references than it was asked for (a face name that resolves to
/// nothing beside ones that do). Between the repair's amber and the error's
/// red: not a failure, but a different answer from the one the document asks,
/// which the user has to see without opening the feature.
const PARTIAL_ORANGE: egui::Color32 = egui::Color32::from_rgb(0xf2, 0x8c, 0x3c);

/// The height of the ROLLBACK BAR row — the horizontal rule painted after the
/// rolled-to feature ("the model is executed up to HERE"). Tall enough to read as
/// a break between the executed block above and the dimmed, not-yet-executed rows
/// below.
const ROLLBACK_BAR_H: f32 = 8.0;

/// The error message for feature `id` from the run report's `featureErrors` array,
/// or `None` when that feature ran clean. The kernel records each hard failure as
/// `"<feature id>: <message>"` (see `pipeline::SceneBuildReport`); this matches the
/// `"<id>: "` prefix (the delimiter after the exact id stops a shorter id from
/// matching a longer one) and returns just the message.
fn feature_error_message(report: &serde_json::Value, id: &str) -> Option<String> {
    let prefix = format!("{id}: ");
    report
        .get("featureErrors")?
        .as_array()?
        .iter()
        .filter_map(serde_json::Value::as_str)
        .find(|entry| entry.starts_with(&prefix))
        .map(|entry| entry[prefix.len()..].to_string())
}

/// The feature's error AS THE USER SHOULD READ IT: a Transform Face refusal
/// re-said about the model, other typed refusals with a readable cause and next step.
///
/// The kernel's text is written for the lane that raised it — `move_faces: the
/// translation inverts the solid (signed volume changed sign)` names a
/// function and a criterion — and it never names the face the user picked,
/// because the message is built below the layer that knows the selection and
/// this panel is above it. See [`brep_render::face_transform_help`]: an
/// unrecognised message comes back UNCHANGED, so this can only ever add.
///
/// BOTH places a failure is shown go through here — the tree's error leaf and
/// the form's banner — so one failure never wears two different wordings.
fn feature_error_for_user(report: &Value, state: &EngineState, id: &str) -> Option<String> {
    let message = feature_error_message(report, id)?;
    let index = feature_index_of(state, id)?;
    if !state.is_face_transform_feature(index) {
        return Some(explain_feature_refusal(&message, &report["featureRefusals"][id]));
    }
    let params: Value = serde_json::from_str(&state.feature_params_json(index)).ok()?;
    let faces = brep_render::face_transform_help::selected_face_names(params.get("faces"));
    let facts = brep_render::face_transform_help::RefusalFacts::from_report(report, id);
    let refusal = brep_render::face_transform_help::explain_refusal(&message, facts, &faces);
    Some(format!(
        "{} refused. {} {}",
        refusal.motion, refusal.reason, refusal.hint
    ))
}

/// Classify only from the structured report, never from diagnostic wording.
/// Unknown future classes and older text-only reports keep their original text.
fn explain_feature_refusal(message: &str, refusal: &Value) -> String {
    let guidance = match refusal.get("class").and_then(Value::as_str) {
        Some("invalid_input") =>
            "Invalid input. Check the selected references and parameter values.",
        Some("unsupported_geometry") =>
            "Unsupported geometry. This operation is not yet supported for the selected geometry.",
        Some("ill_posed") =>
            "Ambiguous operation. Adjust the selection or parameters to specify a unique result.",
        Some("non_convergence") =>
            "Solver did not converge. Try a smaller change or split the operation into simpler steps.",
        Some("tangent_node_singularity") =>
            "Tangent intersection. Adjust the geometry near the tangency or split the operation.",
        Some("degenerate_arrangement" | "non_integral_genus" | "invalid_result_topology" | "non_positive_volume") =>
            "Invalid solid result. The operation could not produce a valid solid; check for collapsed or zero-thickness regions.",
        Some("conservative_empty_overlap") =>
            "Uncertain overlap. The operation could not resolve touching boundaries; check the contact region.",
        Some("unsound_result") =>
            "Invalid geometry result. The operation produced intersecting or folded geometry, or an invalid boundary.",
        Some("internal") =>
            "Internal geometry error. Save the model and report this operation with the details below.",
        _ => return message.to_string(),
    };
    format!("{guidance}\n\n{message}")
}

/// What the kernel REPAIRED while building feature `id`, from the run report's
/// `featureNotes` array, joined when there is more than one. `None` when the
/// feature built what it was asked for unchanged, which is nearly always.
///
/// A repair is not an error — the feature succeeded — but it IS a different
/// answer from the one the lane built, so it is shown rather than swallowed.
/// The feature's PARTIAL FULFILMENT, when the run report carries one
/// (`featureFulfilment[id].summary`: "1 of 2 selected references applied;
/// `X` resolves to nothing on this model"). `None` when the feature did
/// everything it was asked, which is nearly always.
fn feature_fulfilment_message(report: &serde_json::Value, id: &str) -> Option<String> {
    report
        .get("featureFulfilment")?
        .get(id)?
        .get("summary")?
        .as_str()
        .map(str::to_string)
}

fn feature_note_message(report: &serde_json::Value, id: &str) -> Option<String> {
    let prefix = format!("{id}: ");
    let notes: Vec<String> = report
        .get("featureNotes")?
        .as_array()?
        .iter()
        .filter_map(serde_json::Value::as_str)
        .filter(|entry| entry.starts_with(&prefix))
        .map(|entry| entry[prefix.len()..].to_string())
        .collect();
    (!notes.is_empty()).then(|| notes.join("; "))
}

/// What the panel is showing: the feature TREE, or ONE feature's FORM filling
/// the whole panel. Exclusive by construction — there is no "expanded feature"
/// inside the tree any more, so there is nothing to reconcile between them.
#[derive(Default, Debug, Clone, PartialEq, Eq)]
pub enum PanelMode {
    /// The feature tree (the resting state).
    #[default]
    Tree,
    /// `feature_id`'s dialog, replacing the whole panel. Falls back to
    /// [`PanelMode::Tree`] the moment that id stops resolving (undo, delete,
    /// document load) — see the validity guard at the top of
    /// [`HistoryPanel::show`].
    Form { feature_id: String },
}

impl PanelMode {
    /// The open form's feature id, or `None` in tree mode.
    fn feature_id(&self) -> Option<&str> {
        match self {
            PanelMode::Tree => None,
            PanelMode::Form { feature_id } => Some(feature_id.as_str()),
        }
    }
}

/// The history panel's transient UI state (the model lives in the engine).
#[derive(Default)]
pub struct HistoryPanel {
    /// The per-frame map of egui widget screen rects, published to JS for the
    /// headed verifier. Rebuilt every frame.
    hits: HashMap<String, egui::Rect>,
    /// Tree, or one feature's full-panel form.
    mode: PanelMode,
    /// The feature id the panel last AUTO-ARMED a dimension gizmo for (gizmo-on-
    /// open). Compared to the OPEN FORM's feature id each frame: on a CHANGE
    /// (open a different feature's form, or return to the tree) the panel
    /// disarms the old gizmo and arms the dimension gizmo for the newly-opened
    /// feature IF it has dimensions — once per transition, so the in-viewport
    /// sphere/center toggle (transform↔dimension) isn't clobbered back to
    /// dimension each frame.
    gizmo_armed_for: Option<String>,
    /// The feature index currently being drag-reordered (None = not dragging).
    drag_src: Option<usize>,
    /// The reusable searchable command palette that `Add new feature` opens,
    /// populated from the kernel feature catalogue. Generic + engine-agnostic —
    /// this panel drives it and acts on the returned type code.
    palette: Palette,
    palette_display_loaded: bool,
    /// A schema `button` field click staged this frame — `(feature id, button
    /// key)` — applied AFTER the draw loop (so no engine mutation runs mid-render).
    /// E.g. `editSketch` on a SKETCH feature → `enter_sketch_mode`.
    pending_button: Option<(String, String)>,
    /// Set when the palette picked the ACOMP type: the insert flow must NOT
    /// open a bare feature dialog — the shell polls this
    /// ([`Self::take_insert_component_request`]) and opens the COMPONENT
    /// SELECTOR (the file dialog's insert mode) instead.
    pending_insert_component: bool,
    /// The SELECTED anchor of the open Spline form's anchor editor (the row
    /// the gizmo is armed on and the cage highlights). Reset when the open
    /// feature changes. See [`super::spline_anchors`].
    anchor_selected: Option<usize>,
    /// How many times a feature form has been OPENED through
    /// [`Self::open_form`] — monotonic, never reset. The dock's dialog door
    /// ([`super::dock::DialogTargets`]) keys on it beside the feature id, so
    /// RE-opening the form that is already open still reads as an open: the
    /// id alone does not change when the context bar's "Edit owning feature"
    /// names the feature whose form is already up, and the pane still has to
    /// come forward (the user is on another tab, which is the only reason they
    /// reached for that button).
    form_opens: u64,
}

impl HistoryPanel {
    /// Load once per panel (including document switches), save only user changes.
    pub fn sync_palette_display(&mut self, store: &dyn crate::store::ModelStore) {
        use crate::store::FEATURE_PALETTE_DISPLAY_KEY;
        if !self.palette_display_loaded {
            self.palette.display = store.read(FEATURE_PALETTE_DISPLAY_KEY)
                .and_then(|json| serde_json::from_str(&json).ok()).unwrap_or_default();
            self.palette_display_loaded = true;
        }
        if self.palette.take_display_change() {
            if let Ok(json) = serde_json::to_string(&self.palette.display) {
                let _ = store.write(FEATURE_PALETTE_DISPLAY_KEY, &json);
            }
        }
    }

    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the feature tree. While a reference-selection picker is active the
    /// shell HIDES this whole panel (the design doc's "hide the rest of the UI")
    /// and shows the picker in the top-right mode card ([`super::mode_bar`]), so
    /// this method is not called in that mode.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.hits.clear();

        // The panel's VISIBLE region (the enclosing dock pane's scroll viewport).
        // Every other rect below is a raw LAYOUT rect: a long feature list — or a
        // long form — runs past the pane's bottom, where egui clips it and it
        // stops being clickable even though the rect is still published. The
        // headed verifier intersects against this to
        // know when it must scroll a row into view first — without it a script
        // clicks dead space outside the panel and silently no-ops.
        self.hits.insert("panel:clip".into(), ui.clip_rect());

        // --- validity guard: a form whose SUBJECT is gone falls back, silently.
        // One check covers every hazard — undo that removed the feature, a delete
        // from another surface, a document load that changed the ids wholesale.
        // With no Cancel there is no dirty state to protect, so there is nothing
        // to ask the user about (params edits are already committed and undoable).
        if let Some(id) = self.mode.feature_id() {
            if feature_index_of(state, id).is_none() {
                self.mode = PanelMode::Tree;
            }
        }

        // Last-run report → per-feature timing + output solid names (parsed once).
        let report: Value = serde_json::from_str(&state.history_report_json()).unwrap_or(Value::Null);

        // Tight, tree-like row spacing so connector verticals read continuously.
        ui.spacing_mut().item_spacing.y = 2.0;

        // --- THE MODE SWITCH: the tree, or ONE feature's form -----------------
        match self.mode.clone() {
            PanelMode::Tree => {
                // No spline form is open: no anchor cage in the viewport.
                state.refresh_spline_edit_overlay(None, None);
                self.show_tree(ui, state, &report);
            }
            PanelMode::Form { feature_id } => self.show_form(ui, state, &report, &feature_id),
        }

        // --- apply a staged schema-button click (after the draw loop) ----------
        if let Some((fid, key)) = self.pending_button.take() {
            self.handle_feature_button(state, &fid, &key);
        }

        // --- gizmo-on-open (Phase 1) ------------------------------------------
        // Arm the DIMENSION gizmo for the feature whose FORM is open so its
        // draggable arrows appear on open (the reported bug), and DISARM on
        // return to the tree so gizmos don't leak. Runs only on an open/close
        // TRANSITION (the open feature changed since last frame) so it never
        // thrashes per frame — that lets the in-viewport sphere/center toggle flip
        // a feature to transform mode and STAY there (a per-frame re-arm would
        // snap it back to dimension). Guarded on the feature actually having
        // dimension annotations or a schema-declared Transform group.
        let open_feature = self.mode.feature_id().map(str::to_string);
        if self.gizmo_armed_for != open_feature {
            self.anchor_selected = None;
            state.disarm_transform();
            if let Some(id) = open_feature.clone() {
                if state.feature_dimension_annotations_json(&id) != "[]" {
                    // Has dimensions → dimension arrows (sphere-toggle to transform).
                    state.arm_dimension(&id);
                } else if state.feature_has_transform(&id) {
                    // No dimensions but transformable (Transform/datum/helix/…) →
                    // arm the TRANSFORM gizmo directly, so it isn't stranded without
                    // a gizmo now that the ◎ arm button is gone.
                    state.arm_transform(&id);
                } else if state.is_spline_feature(&id) {
                    // A spline's editing surface is its anchors: arm the gizmo on
                    // the first FREE one, so the form opens with a handle to drag
                    // (an attached anchor is placed by its port and has none).
                    let first_free = state
                        .spline_anchors(&id)
                        .iter()
                        .find(|row| row.attached.is_none())
                        .map(|row| row.index);
                    if let Some(index) = first_free {
                        if state.arm_spline_anchor(&id, index) {
                            self.anchor_selected = Some(index);
                        }
                    }
                }
            }
            self.gizmo_armed_for = open_feature;
        } else if let Some(id) = open_feature.as_deref() {
            // A TRANSFORM FACE has no gizmo until its faces give it a pivot —
            // arming it on open finds no pose and disarms — so it arms on the
            // frame the committed selection first resolves. Only this feature:
            // every other transform feature always has a pose, and a per-frame
            // re-arm would undo the ◎ toggle for them.
            if !state.transform_armed()
                && feature_index_of(state, id).is_some_and(|index| state.is_face_transform_feature(index))
                && state.face_transform_has_pivot(id)
            {
                state.arm_transform(id);
            }
        }
    }

    /// Draw ONE feature's dialog filling the whole panel, through the SHARED
    /// [`form_view`]. The panel supplies the schema + the live params and acts on
    /// the intents that come back — it is the engine side of a form view that
    /// holds no engine itself. Editing is LIVE (every change commits and
    /// re-runs); the single exit button returns to the tree and rolls to the
    /// TIP, so downstream features rebuild and the edit becomes visible — the
    /// same reason collapsing an inline dialog used to roll to the tip.
    fn show_form(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        report: &Value,
        id: &str,
    ) {
        // The guard in `show` already proved the id resolves.
        let Some(index) = feature_index_of(state, id) else {
            return;
        };
        let ty = state.feature_type_at(index).unwrap_or_else(|| "?".into());
        // NO glyph: an egui window title is plain text and cannot hold a widget,
        // so it is the one place an icon cannot be drawn as artwork — and with
        // no icon font there is nothing else to draw a private-use character
        // with. The id and name identify the form; the tree row behind it shows
        // the icon.
        let title = format!("{id}  {}", features::feature_plain_name(&ty));
        let fields = features::feature_form_fields(&ty);
        let mut params: Value =
            serde_json::from_str(&state.feature_params_json(index)).unwrap_or(Value::Null);
        // The schema-driven field-visibility hook: which params this feature's
        // dialog should hide for its CURRENT values (e.g. a SIMPLE hole hides the
        // countersink/counterbore fields). Recomputed every frame from the live
        // params, so it reacts to both opening the form and changing a field.
        let hidden = features::feature_hidden_params(&ty, &params);
        // The feature's error shows HERE, as a banner above the fields — and the
        // tree row keeps its own error leaf, so a failure is still visible while
        // scanning the tree.
        let error = feature_error_for_user(report, state, id);
        // One banner slot, by priority: the error when there is one (a failed
        // feature neither fulfilled nor repaired anything), else the partial
        // fulfilment (a different answer from the one asked), else the repair.
        let partial = feature_fulfilment_message(report, id);
        let note = feature_note_message(report, id);
        let banner = error
            .as_deref()
            .map(|message| (message, ERROR_RED))
            .or_else(|| partial.as_deref().map(|message| (message, PARTIAL_ORANGE)))
            .or_else(|| note.as_deref().map(|message| (message, NOTE_AMBER)));
        let outputs: Vec<String> = report
            .get("featureOutputs")
            .and_then(|m| m.get(id))
            .and_then(Value::as_array)
            .map(|a| a.iter().filter_map(|v| v.as_str().map(String::from)).collect())
            .unwrap_or_default();
        let trailing = [("Outputs", outputs)];

        // A SPLINE's editing surface is its anchor list — the schema cannot
        // express it, so it is the form's consumer section. Intent-out: the
        // drawer only records what was asked; the engine is driven below.
        let is_spline = ty == "SP";
        if is_spline {
            // A viewport click on an anchor dot selected it (and armed the
            // gizmo there): the row selection follows before the list draws.
            if let Some(index) = state.take_spline_anchor_pick() {
                self.anchor_selected = Some(index);
            }
        }
        let anchor_rows = if is_spline { state.spline_anchors(id) } else { Vec::new() };
        let anchor_intents: std::cell::RefCell<Vec<spline_anchors::AnchorIntent>> =
            std::cell::RefCell::new(Vec::new());
        let anchor_hits: std::cell::RefCell<Vec<(String, egui::Rect)>> =
            std::cell::RefCell::new(Vec::new());
        let anchor_selected = self.anchor_selected;
        let draw_anchors = |ui: &mut egui::Ui| {
            spline_anchors::draw(ui, &anchor_rows, anchor_selected, &anchor_intents, &anchor_hits);
        };

        let spec = FormViewSpec {
            title: &title,
            subtitle: None,
            fields: &fields,
            hidden: Some(&hidden),
            banner,
            trailing: Some(&trailing),
            exit_label: "Return to tree",
            extra: is_spline.then_some(("Anchors", &draw_anchors as &dyn Fn(&mut egui::Ui))),
            // A feature LIVES in the rolled history, so leaving its form rolls to
            // the tip (Q2). Declared here, acted on below via `out.roll_to_tip`.
            rollback: true,
            // History shows ONE form at a time, so its field keys stay exactly
            // the tree's — `field:sizeX`, `field:boolean.operation` — and every
            // verifier field flow keeps working unchanged.
            hits_prefix: "",
            // A PLM revision this user may not change: the form shows, disabled.
            read_only: state.history.locked(),
        };
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        for (key, rect) in anchor_hits.into_inner() {
            self.hits.insert(key, rect);
        }
        if is_spline {
            // ONE anchor intent per frame (each is a document edit + re-run).
            if let Some(intent) = anchor_intents.into_inner().into_iter().next() {
                self.anchor_selected = spline_anchors::apply(state, id, intent, self.anchor_selected);
            }
            state.refresh_spline_edit_overlay(Some(id), self.anchor_selected);
        } else {
            state.refresh_spline_edit_overlay(None, None);
        }

        // WHICH feature the form is showing, as a presence-only zero-size rect
        // beside the header's real rect (`form:feature`) — the same convention
        // the status bar's `busy` uses for "this is showing right now".
        let anchor = self
            .hits
            .get("form:feature")
            .map(|r| r.min)
            .unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(
            format!("form:feature:{id}"),
            egui::Rect::from_min_size(anchor, egui::Vec2::ZERO),
        );

        if out.changed {
            let _ = state.update_feature_params(id, &params.to_string());
        }
        // A button click (e.g. `editSketch`) binds to no param — stage it as a
        // deferred action keyed by (feature id, button key); `show` acts after the
        // draw so no engine mutation happens mid-render.
        if let Some(key) = out.button_clicked {
            self.pending_button = Some((id.to_string(), key));
        }
        if let Some(activate) = out.ref_activate {
            state.begin_ref_select(
                id,
                activate.path,
                activate.label,
                activate.filter,
                activate.multiple,
                activate.seed,
            );
        }
        // A hovered reference / `Outputs` line lights the entity it names in the
        // 3D view. Applied EVERY frame, independent of everything above (hovering
        // a line and editing a field legitimately land on the same frame): the
        // engine dedupes a held hover, and `dialog_hover_end` only ends a hover
        // THIS panel set, so the Scene tree beside it in a split dock keeps its
        // own.
        let hover_changed = match &out.hovered_entity {
            Some(name) => state.hover_entity_by_name(DIALOG_HOVER_OWNER, name),
            None => state.dialog_hover_end(DIALOG_HOVER_OWNER),
        };
        if hover_changed {
            // The viewport tile may have drawn (and consumed `state.dirty`) BEFORE
            // this pane in the dock, so without this the new highlight would wait
            // for the next pointer event to reach the screen.
            ui.ctx().request_repaint();
        }
        if out.exit_clicked {
            self.mode = PanelMode::Tree;
        }
        if out.roll_to_tip {
            // Finished editing → return the model to the TIP so the WHOLE history
            // runs and every downstream feature (e.g. a boolean that consumes this
            // one) reappears and reflects the edit. Without this the view stays
            // rolled at the just-edited feature and the result never updates — half
            // of the reported "edit the cylinder, close it, nothing changes" bug
            // (the other half was the stale cache, fixed in the kernel). The cache
            // makes this cheap: unchanged features replay instantly.
            //
            // The ROLL half is gated by `spec.rollback` (the form view's one-place
            // decision), not by an `if self is the history panel` here — a consumer
            // with no rollback simply never receives this intent.
            state.roll_to(state.history_len().saturating_sub(1));
        }
    }

    /// Draw the feature TREE — one row per feature, the rollback bar, and the
    /// add-feature palette.
    fn show_tree(&mut self, ui: &mut egui::Ui, state: &mut EngineState, report: &Value) {
        // --- the run's WORKING indicator is not here -------------------------
        // It is the status bar's (`panels::busy`), which every tab and workbench
        // shows; the tree header used to carry it, visible only on this tab.
        // What stays is the AFTERMATH of a cancel: the feature the run was
        // stuck on, until the next rebuild — a note about this tree, not about
        // work in flight.
        let cancelled = state.cancelled_run().map(str::to_string);

        // --- ROOT: `[-] Features` (always open) -------------------------------
        tree::node(
            ui,
            TreeRow {
                guides: &[],
                is_last: true,
                expandable: true,
                expanded: true,
                root: true,
                glyph: None,
                label: "Features",
                selected: false,
                highlighted: false,
                draggable: false,
                tint: None,
            },
            |ui| {
                if let Some(cancelled) = &cancelled {
                    let note = if cancelled.is_empty() {
                        "run cancelled — the next edit rebuilds".to_string()
                    } else {
                        format!("run cancelled at {cancelled} — edit or delete it to rebuild")
                    };
                    ui.colored_label(ERROR_RED, note);
                }
            },
        );

        let len = state.history_len();
        if len == 0 {
            let g = tree::child_guides(&[], true);
            tree::node(ui, TreeRow::leaf(&g, true, "(empty — add a feature)"), |_| {});
        }

        // Deferred engine mutations (applied after the draw loop so no borrow of
        // `self`/`state` is held across them).
        let mut roll: Option<usize> = None;
        let mut delete: Option<String> = None;
        let mut drag_move: Option<(usize, usize)> = None;
        // The feature a single click SELECTED — applied after the draw loop too.
        let mut select: Option<String> = None;
        // The feature whose FORM to open — `(index, id)`. Applied after the draw
        // loop, with the roll, so the mode flip and the roll are one step.
        let mut open_form: Option<(usize, String)> = None;
        let mut feature_rects: Vec<(usize, egui::Rect)> = Vec::with_capacity(len);

        let current = state.history_rollback();
        let selected_feature = state.selected_feature().map(str::to_string);
        // The error leaves, resolved to their USER wording BEFORE the draw loop —
        // `feature_error_for_user` needs `state`, which the loop has already lent
        // out. Computed for the failing features only, which is nearly always
        // none, so the common frame does no work here at all.
        let user_errors: std::collections::HashMap<String, String> = (0..len)
            .filter_map(|i| {
                let id = state.feature_id_at(i)?;
                let message = feature_error_for_user(report, state, &id)?;
                Some((id, message))
            })
            .collect();
        for i in 0..len {
            let ty = state.feature_type_at(i).unwrap_or_else(|| "?".into());
            let id = state.feature_id_at(i).unwrap_or_else(|| "(no id)".into());
            let is_last_feature = i + 1 == len;
            let ms = report
                .get("featureTimings")
                .and_then(|m| m.get(&id))
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            // The feature's glyph goes in the tree's OWN glyph column rather
            // than inline in the label, so the icons line up down the tree and
            // a catalogued COLOUR icon is drawn as its artwork instead of as a
            // one-colour font character (see `tree::node`). `feature_plain_name`
            // is `feature_long_name` without the glyph it would prepend.
            let glyph = features::feature_icon(&ty).map(String::from);
            let label = format!("{id}  {}", features::feature_plain_name(&ty));

            // Features AFTER the rollback point have NOT been executed: dim the
            // WHOLE row — header, timing, edit + delete buttons and the connector
            // lines — with egui's own disabled dimming, the app's existing
            // "not active" language (see `panels::toolbar_button`). Dimmed, NOT
            // disabled: a double click on one still rolls the model FORWARD to it.
            let pending = i > current;
            ui.scope(|ui| {
                if pending {
                    ui.set_opacity(ui.visuals().disabled_alpha());
                }
                // --- feature header row: [+/-] {glyph} id LongName  N ms [✎] [X] --
                // The per-type glyph sits in the tree's own glyph column, so it
                // is drawn as real COLOUR artwork from the icon catalog rather
                // than as a one-colour font character, and the icons line up in
                // a column down the tree. See `tree::node`.
                //
                // The `[+]`/`[-]` box is the BUILT-UP-TO marker: `[-]` down to the
                // rolled-to feature, `[+]` on the not-yet-executed ones below it —
                // the same boundary the rollback bar and the dimming draw. Clicking
                // one MOVES that boundary (a pure roll); nothing expands, because a
                // feature's fields no longer live in the tree.
                let mut del_rect = egui::Rect::NOTHING;
                let mut del_clicked = false;
                let mut edit_rect = egui::Rect::NOTHING;
                let mut edit_clicked = false;
                let resp = tree::node(
                    ui,
                    TreeRow::branch(&[], is_last_feature, !pending, &label)
                        .glyph(glyph.as_deref())
                        .selected(selected_feature.as_deref() == Some(id.as_str()))
                        .draggable(true),
                    |ui| {
                        // right-to-left: X first (rightmost), then the edit pencil,
                        // then the timing. Both buttons are `small()` so a third
                        // control costs the label as little width as possible.
                        let del = ui.add(
                            crate::icon_text::icon_button_colored(ui, "✕", Some(DELETE_RED))
                                .stroke(egui::Stroke::new(1.0, DELETE_RED))
                                .small(),
                        );
                        del_rect = del.rect;
                        del_clicked = del.clicked();
                        ui.add_space(4.0);
                        let edit = ui
                            .add(crate::icon_text::icon_button(ui, "✎").small())
                            .on_hover_text("Edit this feature");
                        edit_rect = edit.rect;
                        edit_clicked = edit.clicked();
                        ui.add_space(6.0);
                        // Not selectable: an egui label is drag-to-select by
                        // default, which senses clicks, and the timing sits in
                        // the middle of the row's blank space — selectable, it
                        // would be the one spot there that does not open the
                        // feature.
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(format!("{} ms", ms.round() as i64)).weak(),
                            )
                            .selectable(false),
                        );
                    },
                );
                // The published row rect IS the band — a script clicks what a
                // user can click, and that is the whole strip, not the name.
                self.hits.insert(format!("step:{i}"), resp.band.rect);
                self.hits.insert(format!("box:{i}"), resp.box_rect);
                self.hits.insert(format!("del:{i}"), del_rect);
                self.hits.insert(format!("edit:{i}"), edit_rect);
                feature_rects.push((i, resp.row_rect));

                if del_clicked {
                    delete = Some(id.clone());
                }
                // Collapse box → a PURE ROLL to that step (no dialog), so the model
                // can be rolled around without opening anything. A click ANYWHERE
                // ELSE on the row → SELECT the feature (the row means the whole
                // strip, not just its name, which is `NodeResponse::clicked`).
                // Edit button OR a DOUBLE click on the row → open that feature's
                // form AND roll to it (the button is the discoverable one, the
                // double click the one the hand already does; egui reports the
                // double click's second press as a click too, so it is read
                // first). Drag to ANOTHER row → reorder. A drag that ends back on
                // its OWN row is a click that egui timed out of the click window —
                // the drag-resolution block below routes it here-equivalently
                // (select).
                if resp.toggled {
                    roll = Some(i);
                }
                if edit_clicked || resp.double_clicked() {
                    open_form = Some((i, id.clone()));
                } else if resp.clicked() {
                    select = Some(id.clone());
                }
                // A press ANYWHERE on the row can turn into a drag — the band
                // senses drag too, because egui withdraws the click from a slow
                // or wobbly press whether or not the widget senses drag, and
                // the resolution below is what turns such a press back into
                // "select this row".
                if resp.drag_started() {
                    self.drag_src = Some(i);
                }

                // --- error node: shown under a FAILING feature, ALWAYS, so a
                // failure is visible while SCANNING the tree (the form's banner
                // shows the same message to whoever opens the feature), and gone
                // the moment the feature runs clean.
                // The SAME wording the form's banner shows — a Transform Face
                // refusal re-said about the model. Scanning the tree and opening
                // the feature must not give two accounts of one failure.
                if let Some(message) = user_errors.get(&id) {
                    let g = tree::child_guides(&[], is_last_feature);
                    tree::message_leaf(ui, &g, true, message, ERROR_RED);
                } else {
                    // --- partial node: the feature SUCCEEDED on fewer
                    // references than it was asked for. Its own leaf, in
                    // orange, because a partial and a repair can both be true
                    // of one feature and each has to be visible while scanning.
                    if let Some(message) = feature_fulfilment_message(report, &id) {
                        let g = tree::child_guides(&[], is_last_feature);
                        tree::message_leaf(ui, &g, true, &message, PARTIAL_ORANGE);
                    }
                    // --- repair node: the feature SUCCEEDED, with a result the
                    // kernel changed on its way out. Same leaf as the error, in
                    // amber, so a repair is visible while scanning the tree
                    // instead of only to whoever opens the feature.
                    if let Some(message) = feature_note_message(report, &id) {
                        let g = tree::child_guides(&[], is_last_feature);
                        tree::message_leaf(ui, &g, true, &message, NOTE_AMBER);
                    }
                }
            });

            // --- the EXECUTED-UP-TO boundary --------------------------------
            // The rolled-to feature IS executed (a run builds `features[0..=rollback]`
            // — see `brep_render::history`; the request's `stopAtId` stops AFTER that
            // feature), so the bar goes BELOW its whole block: everything above the
            // bar is live, everything below it is dimmed and not yet built. Drawn
            // OUTSIDE the dim scope (and at the tip too, where it simply reports that
            // the model is built to the end).
            if i == current {
                let bar = rollback_bar(ui);
                self.hits.insert("rollback:bar".into(), bar);
            }
        }

        // --- resolve an in-flight drag ----------------------------------------
        if let Some(src) = self.drag_src {
            let released = ui.input(|i| i.pointer.any_released());
            let ptr = ui.input(|i| i.pointer.interact_pos());
            match (ptr, released) {
                (Some(p), released) => {
                    let target = feature_rects
                        .iter()
                        .min_by(|a, b| {
                            let da = (a.1.center().y - p.y).abs();
                            let db = (b.1.center().y - p.y).abs();
                            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .map(|(idx, _)| *idx)
                        .unwrap_or(src);
                    if released {
                        if target == src {
                            // NOT a reorder — a press that STARTED and ENDED on the
                            // same row. egui reclassifies a press as a DRAG once it
                            // outlives `max_click_duration` (0.8 s) or drifts past
                            // `max_click_dist` (6 pt), so an ordinary human click on
                            // a label — which routinely lingers or wobbles a few
                            // pixels — fires `drag_started` and NEVER `clicked`.
                            // Routing that through the reorder arm below would do
                            // nothing at all (`drag_move` onto its own slot). A
                            // same-slot release IS the row click: select that
                            // feature, exactly like `resp.clicked()` does. egui
                            // never fires both a click and a drag for one press,
                            // so this can't select twice.
                            select = state.feature_id_at(src);
                        } else {
                            drag_move = Some((src, target));
                        }
                        self.drag_src = None;
                    } else if target != src {
                        // Draw an insertion indicator at the target row edge.
                        if let Some((_, rect)) = feature_rects.iter().find(|(idx, _)| *idx == target)
                        {
                            let y = if target >= src { rect.bottom() } else { rect.top() };
                            ui.painter().hline(
                                rect.x_range(),
                                y,
                                egui::Stroke::new(2.0, ui.visuals().selection.bg_fill),
                            );
                        }
                    }
                }
                (None, true) => self.drag_src = None,
                _ => {}
            }
        }

        // --- Add new feature (full-width) → open the searchable palette -------
        ui.add_space(6.0);
        let add = ui.add_sized(
            [ui.available_width(), 26.0],
            egui::Button::new("Add new feature"),
        );
        self.hits.insert("add:menu".into(), add.rect);
        if add.clicked() {
            // The active workbench TRIMS the creation palette (a UI filter only —
            // the history/execution surface is untouched).
            let items = feature_palette_items(&state.settings.workbench);
            self.palette.open(items, "Add feature", "Search features…");
        }

        // --- apply deferred engine mutations (one per frame) ------------------
        // A reorder does NOT open the moved feature's form: a drag is a
        // restructuring gesture, and replacing the whole panel with a dialog
        // after one would hide the tree the user was just arranging.
        if let Some((src, to)) = drag_move {
            self.move_feature(state, src, to);
        } else if let Some(id) = delete {
            // A deleted feature can't be the open form's subject (the form has no
            // delete affordance), and if another surface deletes it the validity
            // guard in `show` falls back to the tree next frame.
            state.delete_feature(&id);
        } else if let Some((i, id)) = open_form {
            // Opening ROLLS to that feature, exactly as expanding it used to:
            // the dialog and the model it describes must agree.
            self.open_form(id);
            state.roll_to(i);
        } else if let Some(i) = roll {
            state.roll_to(i);
        } else if let Some(id) = select {
            // Selecting never rolls: a click is a look, not a rebuild.
            state.select_feature(&id);
        }

        // --- the command palette (a ctx-level modal; drawn last) --------------
        // A pick returns the chosen feature TYPE CODE; add that feature to the
        // engine-owned history with schema-derived defaults + a unique id.
        let ctx = ui.ctx().clone();
        if let Some(type_code) = self.palette.show(&ctx) {
            self.add_feature_of_type(state, &type_code);
        }
        // The palette's own rects are NOT merged in here. It is a ctx-level
        // modal drawn over the whole window, so its rects are screen points
        // that owe nothing to this pane's scroll offset — and while they lived
        // in this blob, `click_widget` treated them as rows of this pane and
        // wheeled the feature list trying to bring `palette:input` inside
        // `panel:clip`, which it can never reach. They are published as their
        // own panel (`__brepPaletteHit`, no `panel:clip`) by the shell.
    }

    /// Move feature `from` to slot `to` via the engine's adjacent-swap reorder
    /// (each swap re-runs the truncated history — small N, and the ONE reorder
    /// primitive the engine exposes).
    fn move_feature(&mut self, state: &mut EngineState, from: usize, to: usize) {
        if from == to {
            return;
        }
        let mut cur = from;
        if to > from {
            while cur < to {
                state.reorder_feature(cur, false);
                cur += 1;
            }
        } else {
            while cur > to {
                state.reorder_feature(cur, true);
                cur -= 1;
            }
        }
    }

    /// Open the feature `id`'s FORM — the panel shows one form at a time, so
    /// this replaces whatever was open. The context action bar calls this via the
    /// shell after creating a feature from the selection or opening a selection's
    /// owning feature (both of which also rolled the model to that step), so the
    /// target feature's dialog is up for tweaking on the next frame. If the id
    /// does not resolve, the validity guard in [`Self::show`] drops straight back
    /// to the tree.
    pub fn focus_feature(&mut self, id: String) {
        self.open_form(id);
    }

    /// **The feature form's half of the dialog door.** Put the panel into
    /// `feature_id`'s form and count the open.
    ///
    /// EVERY path that opens a feature dialog goes through here — the tree's
    /// Edit button and row double click, the palette's add, the
    /// workbench toolbar's add, the context bar's create / edit-owning, and the
    /// BOM's Edit — so the dock has exactly one signal to watch and the pane is
    /// surfaced once, in one place ([`super::dock::DockState::surface_opened_dialogs`]).
    /// Writing `self.mode` directly anywhere else is the bug this method exists
    /// to prevent: the form would open with its tab still behind another one.
    fn open_form(&mut self, feature_id: String) {
        self.mode = PanelMode::Form { feature_id };
        self.form_opens = self.form_opens.wrapping_add(1);
    }

    /// The open feature form's subject and the open COUNT — what the dock's
    /// door compares frame to frame. `None` in tree mode.
    pub fn form_open(&self) -> (Option<&str>, u64) {
        (self.mode.feature_id(), self.form_opens)
    }

    /// Act on a schema `button` field click on a feature. `editSketch` on a SKETCH
    /// feature opens the engine-native sketcher on THAT feature (roll-to-before +
    /// plane orient) and returns the panel to the tree (the sketch-mode bar takes
    /// over the UI). `enter_sketch_mode` guards the feature is a sketch, so a
    /// stray click on a non-sketch is a harmless no-op.
    fn handle_feature_button(&mut self, state: &mut EngineState, feature_id: &str, key: &str) {
        match key {
            "editSketch" => match state.enter_sketch_mode(feature_id) {
                Ok(_) => self.mode = PanelMode::Tree,
                Err(_err) => {
                    #[cfg(not(target_arch = "wasm32"))]
                    eprintln!("Edit Sketch failed for '{feature_id}': {_err}");
                }
            },
            _ => {}
        }
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame (`DockState::ui`), whose widgets are not on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// The add-feature palette's rects, as their OWN panel. The palette is a
    /// ctx-level modal, not a row of this pane, so it publishes no
    /// `panel:clip` and nothing scrolls to reach it.
    pub fn palette_hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(self.palette.hits())
    }

    /// Whether the palette requested a COMPONENT INSERT this frame (the ACOMP
    /// palette entry routes to the component selector, never a bare feature
    /// dialog). Consumed by the shell, which opens the file dialog's insert
    /// mode.
    pub fn take_insert_component_request(&mut self) -> bool {
        std::mem::take(&mut self.pending_insert_component)
    }

    /// Append a feature of type `type_code` to the engine-owned history: build a
    /// fresh descriptor whose `inputParams` are the schema DEFAULTS
    /// ([`features::feature_default_params`]) with an engine-unique `id` assigned,
    /// hand it to `EngineState::add_feature` (which appends + rolls to it), and
    /// open the new feature's FORM. Works for ANY registered feature type — the
    /// catalogue drives both the palette and the defaults.
    ///
    /// EXCEPTION — `ACOMP` (assembly component): inserting an instance needs a
    /// parts-library payload first, so the palette pick surfaces an
    /// insert-component REQUEST to the shell (which opens the component
    /// selector) instead of appending an empty feature that could only fail.
    pub(crate) fn add_feature_of_type(&mut self, state: &mut EngineState, type_code: &str) {
        if type_code == "ACOMP" {
            self.pending_insert_component = true;
            return;
        }
        let id = state.next_feature_id(&features::feature_short_name(type_code));
        let mut params = features::feature_default_params(type_code);
        if let Value::Object(map) = &mut params {
            map.insert("id".into(), Value::String(id.clone()));
        }
        let feature = serde_json::json!({
            "type": type_code, "inputParams": params, "persistentData": {}
        });
        if state.add_feature(&feature.to_string()).is_ok() {
            self.open_form(id);
        }
    }
}

/// The history index of feature `id`, by linear scan over the engine's history.
/// `EngineState` exposes `feature_id_at` but no public `index_of`, and the panel
/// needs one for the form's per-frame validity guard (does the open form's
/// subject still exist?). Small N, once per frame.
fn feature_index_of(state: &EngineState, id: &str) -> Option<usize> {
    (0..state.history_len()).find(|i| state.feature_id_at(*i).as_deref() == Some(id))
}

/// Paint the ROLLBACK BAR: a full-width horizontal rule marking the step the model
/// is EXECUTED UP TO. It reuses the drag-reorder insertion indicator's look (a 2 px
/// line in the theme's selection accent — see the drag branch of
/// [`HistoryPanel::show`]) because it says the same thing: "the boundary is HERE".
/// Returns its row rect, which the panel publishes for the headed verifier.
fn rollback_bar(ui: &mut egui::Ui) -> egui::Rect {
    let (rect, _) = ui.allocate_exact_size(
        egui::vec2(ui.available_width(), ROLLBACK_BAR_H),
        egui::Sense::hover(),
    );
    ui.painter().hline(
        rect.x_range(),
        rect.center().y,
        egui::Stroke::new(2.0, ui.visuals().selection.bg_fill),
    );
    rect
}

/// Build one [`PaletteItem`] per registered feature from the kernel catalogue:
/// `id` = the feature TYPE CODE (e.g. `P.CU`), `label` = its long name (e.g.
/// `Primitive Cube`), `keywords` = the type code + short name (aliases the user
/// might type). The palette sorts them alphabetically by label on open.
///
/// `workbench` is the active workbench id: only entries that workbench INCLUDES
/// (each workbench classifies off the feature TYPE CODE) are offered. This is a
/// pure UI filter over CREATION — it does not touch the existing history, so a
/// document with sheet-metal features still shows and edits them in Modeling; only
/// the "Add new feature" list is trimmed.
fn feature_palette_items(workbench: &str) -> Vec<PaletteItem> {
    let catalogue = features::feature_catalogue();
    let mut items = Vec::new();
    if let Some(list) = catalogue.get("features").and_then(Value::as_array) {
        for feature in list {
            let ty = feature.get("type").and_then(Value::as_str).unwrap_or("");
            if ty.is_empty() {
                continue;
            }
            if !crate::workbench::includes_feature(workbench, ty) {
                continue;
            }
            // `feature_long_name` prepends the glyph; the palette sorts/searches
            // on a glyph-stripped key so it stays alphabetical.
            let long = features::feature_long_name(ty);
            let short = feature.get("shortName").and_then(Value::as_str).unwrap_or(ty);
            let mut keywords = vec![ty.to_string()];
            if short != ty {
                keywords.push(short.to_string());
            }
            items.push(PaletteItem::new(ty, long, keywords));
        }
    }
    items
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "history", prefix: "step:", meaning: "feature i's row (step:i) \u{2014} a click SELECTS the feature (what it made becomes the viewport selection), a double click opens its form and rolls to it", command: None },
    HitKeyDoc { panel: "history", prefix: "edit:", meaning: "open feature i's form (edit:i)", command: None },
    HitKeyDoc { panel: "history", prefix: "del:", meaning: "delete feature i (del:i)", command: None },
    HitKeyDoc { panel: "history", prefix: "box:", meaning: "feature i's visibility box (box:i)", command: None },
    HitKeyDoc { panel: "history", prefix: "add:menu", meaning: "open the add-feature palette", command: None },
    HitKeyDoc { panel: "history", prefix: "form:return", meaning: "close the open form and roll to the tip", command: None },
    HitKeyDoc { panel: "history", prefix: "form:read-only", meaning: "the open form's read-only banner (a PLM revision not checked out, or released): the fields take no edit", command: None },
    HitKeyDoc { panel: "history", prefix: "form:feature", meaning: "the open form's title row (form:feature) and its feature chip (form:feature:id)", command: None },
    HitKeyDoc { panel: "history", prefix: "form:feature:", meaning: "the open form's feature chip (form:feature:id)", command: None },
    HitKeyDoc { panel: "history", prefix: "form:section:", meaning: "toggle an accordion section of the open form", command: None },
    HitKeyDoc { panel: "history", prefix: "field:", meaning: "a field of the open form, by param path", command: None },
    HitKeyDoc { panel: "history", prefix: "form:trailing:", meaning: "a read-only line of the open form (form:trailing:Outputs:i) — hovering it lights that entity", command: None },
    HitKeyDoc { panel: "history", prefix: "rollback:bar", meaning: "the executed-up-to indicator", command: None },
    HitKeyDoc { panel: "history", prefix: "panel:clip", meaning: "the visible region of the pane", command: None },
];
