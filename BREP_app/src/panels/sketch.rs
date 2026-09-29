//! Sketch panel (S1) — the engine-native sketcher's mode surfaces.
//!
//! Thin surfaces over [`EngineState`] (the single brain — it owns the sketch
//! edit; this panel only triggers its methods and reads it back), all drawn while
//! [`EngineState::sketch_mode`]:
//!
//! * The sketch STATUS row ([`SketchPanel::show_status_bar`]): the sketch title +
//!   DOF readout + selection count + the in-progress click count + Lock-to-sketch
//!   (undo/redo is on the main toolbar, which routes to the sketch history while
//!   a sketch is open), rendered into the shell's persistent bottom status bar
//!   (the same bar that hosts the modeling selection filter out of sketch mode).
//! * The LEFT entity lists ([`SketchPanel::show_entity_lists`]) and the top-right
//!   CONTEXT actions ([`SketchPanel::context_card`]).
//!
//! There is NO draw-tools strip: the primitive draw tools (select / point /
//! line / rect / circle / …) and the one-shot Auto-constrain are workbench
//! buttons in the main toolbar row, declared once in [`crate::workbench::sketch`]
//! and offered by every workbench while a sketch is being edited. A mode's
//! tools live in one place, and that place is the row.
//!
//! Finish / Cancel (→ [`EngineState::exit_sketch_mode`]) live in the shared
//! top-right mode-exit card. Enter/exit rolls the model to the step before the
//! sketch and orients the camera onto the plane — all handled by the engine; the
//! shell just keeps the 3D viewport visible as the sketching surface.

use super::action_rail::{action_rail, ActionItem};
use crate::automation::hit_keys::HitKeyDoc;
use crate::panels::tree::{self, TreeRow};
use brep_render::engine_state::{EngineState, SketchEntityRow};
use brep_render::sketch::doc::id_key;
use eframe::egui;
use std::collections::HashMap;

/// The sketch panel's transient UI state (the sketch itself lives in the engine).
#[derive(Default)]
pub struct SketchPanel {
    /// Per-frame Sketch-actions card widget rects, published as `__brepSketchHit`.
    ctx_hits: HashMap<String, egui::Rect>,
    /// Per-frame entity-list widget rects, published as `__brepSketchListHit`.
    /// A SEPARATE blob from the card's: the list scrolls and therefore carries a
    /// `panel:clip`, while the card floats in the top-right overlay with no clip of
    /// its own. Merged into one panel the automation host would test the card's rect
    /// against the list's viewport and scroll for a widget that was never in it.
    list_hits: HashMap<String, egui::Rect>,
    /// Collapse state for the entity-list sections (default open = false).
    curves_collapsed: bool,
    points_collapsed: bool,
    constraints_collapsed: bool,
    /// Solver Settings section starts COLLAPSED (advanced/rarely-touched).
    solver_collapsed: bool,
}

impl SketchPanel {
    pub fn new() -> Self {
        Self {
            // Solver Settings is advanced — start it collapsed.
            solver_collapsed: true,
            ..Self::default()
        }
    }

    /// The sketch STATUS row, rendered into the shell's persistent bottom status
    /// bar while [`EngineState::sketch_mode`]: the sketch title + DOF readout +
    /// selection count + the in-progress click count + Lock-to-sketch. Draws
    /// directly into the bottom bar's `ui` (no inner panel) so the bar can host it
    /// in place of the modeling selection-filter row — demonstrating the status
    /// bar's dynamic, context-selected content.
    ///
    /// The "… N placed" readout came here when the draw-tools strip was folded
    /// into the toolbar row: it is STATUS, not a tool — how many clicks of the
    /// armed tool are buffered — so it belongs beside the selection count and not
    /// among the buttons. A tool that swallows the next three clicks has to say
    /// so, the same reason the sheet's armed-tool prompt is painted on the desk.
    pub fn show_status_bar(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        ui.horizontal_wrapped(|ui| {
            let id = state.sketch_edit_feature_id().unwrap_or("").to_string();
            ui.label(egui::RichText::new(format!("Sketch: {id}")).strong());
            ui.separator();
            // The conflict red is read from the display settings (one editable
            // palette entry drives the dot, the list rows, the glyphs and the
            // dimension text), so it must be taken BEFORE the session borrow.
            let conflict = conflict_color(state);
            if let Some(session) = state.sketch_edit_session() {
                dof_readout(ui, &session.diagnostics, conflict);
            }
            ui.separator();
            ui.label(
                egui::RichText::new(format!("{} selected", state.sketch_selection_count())).weak(),
            );
            let pending = state.sketch_pending_len();
            if pending > 0 {
                ui.separator();
                ui.label(
                    egui::RichText::new(format!("\u{2026} {pending} placed"))
                        .weak()
                        .italics(),
                );
            }
            ui.separator();
            // Undo/redo is NOT here — while a sketch is open the MAIN toolbar's
            // undo/redo pair drives the per-session sketch history (see
            // `toolbar::Toolbar::edit_actions`), alongside the Ctrl+Z / Ctrl+Y
            // keyboard router. Keeping one pair avoids a duplicate control.
            // Camera lock (on by default): hold the view flat to the sketch plane
            // and allow only panning. Toggling it back on re-faces the camera to
            // the plane; off frees orbiting.
            let mut locked = state.sketch_camera_locked();
            if ui
                .checkbox(&mut locked, "Lock to sketch")
                .on_hover_text(
                    "Face the sketch plane and pan only. Uncheck to orbit; \
                     re-check to snap back flat.",
                )
                .changed()
            {
                state.toggle_sketch_camera_lock();
            }
        });
    }

    /// The in-sketch CONTEXT actions — the selection-driven constraint palette
    /// (applicable constraints + Fix/Unfix + ◐ construction + 🧹 cleanup + 🗑
    /// delete) — rendered through the SAME shared rail as the modeling context
    /// bar ([`super::action_rail`]), so both look and behave identically. The
    /// shell draws it into the top-right overlay, below the mode-exit card. No-op
    /// unless a sketch is being edited AND something is selected.
    pub fn context_card(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.ctx_hits.clear();
        if !state.sketch_mode() || state.sketch_selection_count() == 0 {
            return;
        }

        let actions = state.sketch_applicable_constraints();
        let grounded = state.sketch_selection_all_grounded();
        let construction = state.sketch_selection_all_construction();

        let mut items: Vec<ActionItem> = Vec::new();
        for action in &actions {
            items.push(ActionItem::new(
                format!("constraint:{}", action.symbol),
                format!("{} {}", action.symbol, action.label),
                action.label.clone(),
            ));
        }
        if let Some(all_grounded) = grounded {
            let (label, tip) = if all_grounded {
                ("Unfix", "Remove the ground constraint")
            } else {
                ("Fix", "Ground (fix) the selected points")
            };
            items.push(ActionItem::new("fix", label, tip));
        }
        if let Some(all_construction) = construction {
            let tip = if all_construction {
                "Convert to regular geometry"
            } else {
                "Convert to construction geometry"
            };
            items.push(ActionItem::new("construction", "◐ Construction", tip));
        }
        items.push(ActionItem::new(
            "cleanup",
            "🧹 Clean",
            "Remove unused points",
        ));
        items.push(ActionItem::new(
            "delete",
            "🗑 Delete",
            "Delete the selected entities (Del / Backspace)",
        ));

        let subtitle = format!("{} selected", state.sketch_selection_count());
        let clicked = egui::Frame::popup(ui.style())
            .show(ui, |ui| {
                action_rail(
                    ui,
                    Some("Sketch actions"),
                    Some(&subtitle),
                    &items,
                    &mut self.ctx_hits,
                )
            })
            .inner;

        match clicked.as_deref() {
            Some(key) if key.starts_with("constraint:") => {
                state.sketch_add_constraint(&key["constraint:".len()..]);
            }
            Some("fix") => {
                state.sketch_toggle_ground();
            }
            Some("construction") => {
                state.sketch_toggle_construction();
            }
            Some("cleanup") => {
                state.sketch_cleanup_unused_points();
            }
            Some("delete") => {
                state.sketch_delete_selection();
            }
            _ => {}
        }
    }

    /// The sketch-mode LEFT panel: Curves / Points / Constraints as selectable,
    /// deletable, hover-synced rows (the port of the previous app's list sidebar).
    /// Drawn only while editing a sketch; a click selects (Ctrl/Cmd adds), a row
    /// hover highlights the entity on the canvas, and the ✕ deletes it.
    pub fn show_entity_lists(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.list_hits.clear();
        let hits = &mut self.list_hits;
        egui::ScrollArea::vertical().show(ui, |ui| {
            // The pane's VISIBLE region — the scroll viewport, not the layout
            // extent. A long entity list runs past the bottom, where egui clips it
            // and it stops being clickable even though its rect is still published;
            // an automation host intersects against this to know when it must
            // scroll a row into view first.
            hits.insert("panel:clip".into(), ui.clip_rect());
            ui.add_space(4.0);
            // Snapshot the rows first (immutable borrow), then render with `&mut
            // state` so per-row select/hover can call back into the engine.
            let curves = state.sketch_geometry_rows();
            let points = state.sketch_point_rows();
            let constraints = state.sketch_constraint_rows();
            entity_section(ui, &mut self.curves_collapsed, state, "Curves", "geometry", curves, hits);
            entity_section(ui, &mut self.points_collapsed, state, "Points", "point", points, hits);
            entity_section(
                ui,
                &mut self.constraints_collapsed,
                state,
                "Constraints",
                "constraint",
                constraints,
                hits,
            );

            ui.add_space(6.0);
            solver_settings_section(ui, &mut self.solver_collapsed, state);
        });
    }

    /// The sketch-mode OVERLAY widget rects (egui points), published as
    /// `__brepSketchHit`: the SKETCH ACTIONS card's constraint buttons plus its
    /// Fix / Construction / Clean / Delete actions, each under the key
    /// [`context_card`] returns when it is clicked.
    ///
    /// The DRAW TOOLS are no longer here. They are workbench buttons now
    /// ([`crate::workbench::sketch`]), so their rects are the toolbar's
    /// `workbench:btn:sketch.tool.*` — one row, one hit-key family, and the
    /// `workbench_button` command reaches them by id as well as by click.
    ///
    /// The card floats in the top-right overlay and does not scroll, so it needs
    /// no `panel:clip` — which is exactly why the entity list is [its own blob].
    ///
    /// Empty out of sketch mode: the shell stops calling the card there, so last
    /// frame's rects would otherwise be published forever as live widgets.
    ///
    /// [`context_card`]: Self::context_card
    /// [its own blob]: Self::list_hits_json
    pub fn hits_json(&self, state: &EngineState) -> String {
        if !state.sketch_mode() {
            return "{}".to_string();
        }
        crate::automation::hit_rects::hits_json(&self.ctx_hits)
    }

    /// The ENTITY LIST's widget rects (egui points), published as
    /// `__brepSketchListHit`: one key per row (`geometry:` / `point:` /
    /// `constraint:`, by entity id), its ✕ (`del:kind:id`), each section header, and
    /// the pane's `panel:clip`. Empty out of sketch mode, for the same reason as
    /// [`hits_json`].
    ///
    /// [`hits_json`]: Self::hits_json
    pub fn list_hits_json(&self, state: &EngineState) -> String {
        if !state.sketch_mode() {
            return "{}".to_string();
        }
        crate::automation::hit_rects::hits_json(&self.list_hits)
    }

    /// Verification mirror (wasm): the live sketch-mode state + the active session's
    /// solve summary, so the headed verifier can assert enter/exit (screenshots read
    /// black headless).
    pub fn published_json(&self, state: &EngineState) -> String {
        let session = state.sketch_edit_session();
        // S5 editable dimensions: the per-constraint labels (id/value/valueExpr/mode)
        // + a count, so the headless verifier can assert dims render + edit.
        let dim_labels: Vec<serde_json::Value> =
            serde_json::from_str(&state.sketch_dimension_labels_json()).unwrap_or_default();
        let dimensions: Vec<serde_json::Value> = dim_labels
            .iter()
            .map(|l| {
                serde_json::json!({
                    "id": l["id"],
                    "text": l["text"],
                    "value": l["value"],
                    "valueExpr": l["valueExpr"],
                    "mode": l["mode"],
                })
            })
            .collect();
        serde_json::json!({
            "sketchMode": state.sketch_mode(),
            "featureId": state.sketch_edit_feature_id(),
            "dof": session.map(|s| s.diagnostics.dof),
            "status": session.map(|s| s.diagnostics.status.clone()),
            "conflicting": session.map(|s| s.diagnostics.conflicting),
            // WHICH constraints conflict (the headless verifier asserts the exact
            // set, not just the flag).
            "conflictingConstraints": session
                .map(|s| s.diagnostics.conflicting_constraints.clone())
                .unwrap_or_default(),
            "points": session.map(|s| s.doc.points.len()),
            "geometries": session.map(|s| s.doc.geometries.len()),
            "constraints": session.map(|s| s.doc.constraints.len()),
            // S2 picking state (the headless verifier asserts hover/selection).
            "hovered": session.and_then(|s| s.hovered.clone()),
            "selectionCount": state.sketch_selection_count(),
            // Constraint selection (select + delete): how many selected refs are
            // constraints, so the verifier can assert a constraint pick + delete.
            "selectedConstraintCount": state.sketch_selected_constraint_count(),
            // S3a draw-tool state (the verifier asserts tool selection + that draws
            // landed): the active tool + the doc's point/geometry counts + the
            // in-progress click buffer length.
            "tool": state.sketch_active_tool(),
            "cameraLocked": state.sketch_camera_locked(),
            "pointCount": session.map(|s| s.doc.points.len()),
            "geometryCount": session.map(|s| s.doc.geometries.len()),
            "pendingLen": state.sketch_pending_len(),
            // S4 constraint palette: the active doc's constraint count + the ordered
            // list of applicable-constraint symbols for the live selection (the
            // headless verifier asserts the palette + that additions landed).
            "constraintCount": state.sketch_constraint_count(),
            "applicableConstraints": state
                .sketch_applicable_constraints()
                .iter()
                .map(|a| a.symbol.clone())
                .collect::<Vec<_>>(),
            // S5 editable dimensions.
            "dimensionCount": dimensions.len(),
            "dimensions": dimensions,
            // S6a per-session sketch undo/redo (the verifier asserts a draw is
            // undoable and that redo is available after an undo).
            "canUndo": state.sketch_can_undo(),
            "canRedo": state.sketch_can_redo(),
            // S6b-2 pickEdges: the number of linked external-reference edges (the
            // verifier asserts a link landed and round-trips a commit + re-enter).
            "externalRefCount": state.sketch_external_ref_count(),
            // S6b-3 handdraw: the live freehand stroke sample count (the verifier
            // asserts a stroke captures while dragging, then clears + emits on end).
            "handdrawPoints": state.sketch_handdraw_len(),
        })
        .to_string()
    }
}

/// The one conflict red every sketch surface uses: the editable
/// `sketchConflictColor` display setting, the same entry the engine paints the
/// glyphs, leaders and dimension text with.
fn conflict_color(state: &EngineState) -> egui::Color32 {
    let c = state.settings.sketch_colors().conflict;
    egui::Color32::from_rgb((c >> 16) as u8, (c >> 8) as u8, c as u8)
}

/// The DOF status readout — a colored dot + label, ported from the previous
/// sketcher's DOF readout. Red = conflicting, yellow = over-constrained, blue =
/// under-constrained (`N` DOF), green = fully constrained.
fn dof_readout(
    ui: &mut egui::Ui,
    diag: &brep_render::sketch::SketchDiagnostics,
    conflict: egui::Color32,
) {
    let (color, label): (egui::Color32, String) = if diag.conflicting {
        // Name HOW MANY constraints the solver implicated, so the readout and the
        // red rows/glyphs are visibly the same statement. The solver never raises
        // the flag without naming something, so the bare label is only a fallback
        // for a diagnostics blob that predates that guarantee.
        let n = diag.conflicting_constraints.len();
        let label = if n > 0 {
            format!("Conflicting constraints ({n})")
        } else {
            "Conflicting constraints".to_string()
        };
        (conflict, label)
    } else if diag.status == "over" || diag.redundant > 0 {
        let label = if diag.dof > 0 {
            format!(
                "Over-constrained ({} redundant, {} DOF)",
                diag.redundant, diag.dof
            )
        } else {
            format!("Over-constrained ({} redundant)", diag.redundant)
        };
        (egui::Color32::from_rgb(0xff, 0xcf, 0x5c), label)
    } else if diag.status == "under" || diag.dof > 0 {
        (
            egui::Color32::from_rgb(0x4a, 0xa3, 0xff),
            format!("Under-constrained — {} DOF", diag.dof),
        )
    } else {
        (
            egui::Color32::from_rgb(0x7e, 0xe0, 0xa6),
            "Fully constrained".to_string(),
        )
    };

    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
        ui.painter().circle_filled(rect.center(), 5.0, color);
        ui.label(egui::RichText::new(label).strong());
    });
}

/// One collapsible entity-list section, rendered with the shared `tree` widget so
/// it matches the Scene tree. A branch header (title + count) over leaf rows; each
/// leaf is a selectable label (Ctrl/Cmd adds), a row hover highlights the entity on
/// the canvas, and a trailing ✕ deletes it via the existing selection-delete path.
fn entity_section(
    ui: &mut egui::Ui,
    collapsed: &mut bool,
    state: &mut EngineState,
    title: &str,
    kind: &'static str,
    rows: Vec<SketchEntityRow>,
    hits: &mut HashMap<String, egui::Rect>,
) {
    let open = !*collapsed;
    let resp = tree::node(ui, TreeRow::branch(&[], true, open, title), |ui| {
        ui.add_space(6.0);
        ui.label(egui::RichText::new(format!("{}", rows.len())).weak());
    });
    hits.insert(format!("section:{title}"), resp.label.rect);
    if resp.toggled || resp.clicked() {
        *collapsed = !*collapsed;
    }
    if !open {
        return;
    }

    let base = tree::child_guides(&[], true);
    if rows.is_empty() {
        tree::node(ui, TreeRow::leaf(&base, true, "—"), |_| {});
        return;
    }

    let n = rows.len();
    // Read the palette ONCE, before the loop borrows `state` mutably per row.
    let conflict = conflict_color(state);
    let mut to_delete: Option<serde_json::Value> = None;
    for (i, row) in rows.iter().enumerate() {
        let last = i + 1 == n;
        let mut delete_clicked = false;
        let mut del_rect = egui::Rect::NOTHING;
        // A CONFLICTING constraint's row is painted in the same red as the status
        // dot and its canvas glyph, so the list answers "which ones?" directly.
        let tint = row.conflicting.then(|| conflict);
        let resp = tree::node(
            ui,
            TreeRow::leaf(&base, last, row.label.as_str())
                .selected(row.selected)
                .tint(tint),
            |ui| {
                let del = crate::icon_text::icon_button(ui, "✕").small();
                let resp = ui.add(del).on_hover_text("Delete");
                del_rect = resp.rect;
                if resp.clicked() {
                    delete_clicked = true;
                }
            },
        );
        // Keyed by the entity's own id, not its position or its label: a row's
        // index moves when a sibling is deleted and its label is display text, but
        // the id is what select/hover/delete are addressed by.
        let key = id_key(&row.id);
        hits.insert(format!("{kind}:{key}"), resp.label.rect);
        hits.insert(format!("del:{kind}:{key}"), del_rect);
        if delete_clicked {
            to_delete = Some(row.id.clone());
        } else if resp.clicked() {
            let additive = ui.input(|i| i.modifiers.command || i.modifiers.ctrl);
            state.sketch_select_entity(kind, row.id.clone(), additive);
        }
        if resp.label.hovered() {
            state.sketch_hover_entity(kind, row.id.clone());
        }
    }
    // Delete last so it can't invalidate the rows we are still iterating this frame.
    if let Some(id) = to_delete {
        state.sketch_select_entity(kind, id, false);
        state.sketch_delete_selection();
    }
}

/// The Solver Settings section (the port of the previous app's "Solver Settings" sidebar): a
/// couple of the `SolveSketchRequest` knobs (iteration cap + optional tolerance
/// override), plus Reset. Any change re-solves the sketch immediately. Defaults
/// reproduce the historical solve, so an untouched sketch is unaffected.
fn solver_settings_section(ui: &mut egui::Ui, collapsed: &mut bool, state: &mut EngineState) {
    let Some(mut settings) = state.sketch_solver_settings() else {
        return;
    };
    let open = !*collapsed;
    let resp = tree::node(ui, TreeRow::branch(&[], true, open, "Solver Settings"), |_| {});
    if resp.toggled || resp.clicked() {
        *collapsed = !*collapsed;
    }
    if !open {
        return;
    }

    let before = settings.clone();
    egui::Frame::group(ui.style()).show(ui, |ui| {
        // Iteration cap (always set; default 1000).
        let mut iters = settings.iterations.unwrap_or(1000);
        ui.horizontal(|ui| {
            ui.label("Max iterations");
            if ui
                .add(egui::DragValue::new(&mut iters).range(50..=20_000).speed(10.0))
                .changed()
            {
                settings.iterations = Some(iters);
            }
        });

        // Optional convergence tolerance override.
        let mut tol_on = settings.tolerance.is_some();
        ui.horizontal(|ui| {
            if ui.checkbox(&mut tol_on, "Override tolerance").changed() {
                settings.tolerance = tol_on.then_some(1e-6);
            }
            if let Some(mut tol) = settings.tolerance {
                if ui
                    .add(
                        egui::DragValue::new(&mut tol)
                            .range(1e-9..=1e-1)
                            .speed(1e-6)
                            .custom_formatter(|v, _| format!("{v:.1e}")),
                    )
                    .changed()
                {
                    settings.tolerance = Some(tol);
                }
            }
        });

        if ui.button("Reset to defaults").clicked() {
            settings = brep_render::sketch::SketchSolverSettings::default();
        }
    });

    if settings != before {
        state.sketch_set_solver_settings(settings);
    }
}

/// The hit keys these two surfaces publish (see `automation::hit_keys`). They are
/// separate PANELS because they are separate blobs: `sketch` is the floating
/// actions card, `sketchlist` the scrolling entity list that owns a `panel:clip`.
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "sketch", prefix: "constraint:", meaning: "apply the offered sketch constraint to the selection (constraint:glyph, e.g. constraint:⌒)", command: None },
    HitKeyDoc { panel: "sketch", prefix: "fix", meaning: "ground the selected points, or unground them when all are grounded", command: None },
    HitKeyDoc { panel: "sketch", prefix: "construction", meaning: "flip the selection between construction and regular geometry", command: None },
    HitKeyDoc { panel: "sketch", prefix: "cleanup", meaning: "remove points no geometry or constraint uses", command: None },
    HitKeyDoc { panel: "sketch", prefix: "delete", meaning: "delete the selected sketch entities", command: None },
    HitKeyDoc { panel: "sketchlist", prefix: "section:", meaning: "collapse or expand an entity-list section (section:Curves / section:Points / section:Constraints)", command: None },
    HitKeyDoc { panel: "sketchlist", prefix: "geometry:", meaning: "select a curve row by entity id; hold ctrl (modifiers_set) to add to the selection", command: None },
    HitKeyDoc { panel: "sketchlist", prefix: "point:", meaning: "select a point row by entity id; hold ctrl to add to the selection", command: None },
    HitKeyDoc { panel: "sketchlist", prefix: "constraint:", meaning: "select a constraint row by entity id; hold ctrl to add to the selection", command: None },
    HitKeyDoc { panel: "sketchlist", prefix: "del:", meaning: "delete one entity-list row (del:kind:id)", command: None },
    HitKeyDoc { panel: "sketchlist", prefix: "panel:clip", meaning: "the visible region of the pane", command: None },
];
