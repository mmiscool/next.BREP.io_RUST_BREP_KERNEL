//! 3D sketch panel — the engine-native 3D sketcher's mode surfaces, the
//! sibling of [`super::sketch`]: thin surfaces over [`EngineState`], all drawn
//! while [`EngineState::sketch3d_mode`]:
//!
//! * The STATUS row ([`Sketch3dPanel::show_status_bar`]): title + DOF readout
//!   + selection count + the in-progress click count + the working plane in
//!   force + what the last click landed on, in the shell's bottom status bar.
//! * The LEFT entity lists ([`Sketch3dPanel::show_entity_lists`]): Curves /
//!   Points / Constraints / References, selectable and deletable.
//! * The top-right CONTEXT actions ([`Sketch3dPanel::context_card`]): the
//!   selection-driven constraint palette plus Fix / Construction / Delete.
//!
//! The draw tools and the working-plane radio are workbench buttons
//! ([`crate::workbench::sketch3d`]); Finish / Cancel live in the shared
//! mode-exit card ([`super::mode_bar`]).

use super::action_rail::{action_rail, ActionItem};
use crate::automation::hit_keys::HitKeyDoc;
use crate::panels::tree::{self, TreeRow};
use brep_render::engine_state::EngineState;
use brep_render::sketch3d::{entity_constraint_ref, entity_geometry_ref, entity_point_ref};
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

/// The 3D sketch panel's transient UI state (the sketch itself lives in the engine).
#[derive(Default)]
pub struct Sketch3dPanel {
    /// Per-frame actions-card widget rects, published as `__brepSketch3dHit`.
    ctx_hits: HashMap<String, egui::Rect>,
    /// Per-frame entity-list widget rects, published as `__brepSketch3dListHit`.
    list_hits: HashMap<String, egui::Rect>,
    curves_collapsed: bool,
    points_collapsed: bool,
    constraints_collapsed: bool,
    references_collapsed: bool,
}

impl Sketch3dPanel {
    pub(crate) fn clear_context_hits(&mut self) { self.ctx_hits.clear(); }
    pub(crate) fn clear_list_hits(&mut self) { self.list_hits.clear(); }
    pub fn new() -> Self {
        Self::default()
    }

    /// The status row in the shell's bottom bar.
    pub fn show_status_bar(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        ui.horizontal_wrapped(|ui| {
            let id = state.sketch3d_edit_feature_id().unwrap_or("").to_string();
            ui.label(egui::RichText::new(format!("3D sketch: {id}")).strong());
            ui.separator();
            let conflict = conflict_color(state);
            if let Some(session) = state.sketch3d_edit_session() {
                dof_readout(ui, &session.diagnostics, conflict);
            }
            ui.separator();
            ui.label(
                egui::RichText::new(format!("{} selected", state.sketch3d_selection_count())).weak(),
            );
            let pending = state.sketch3d_pending_len();
            if pending > 0 {
                ui.separator();
                ui.label(
                    egui::RichText::new(format!("\u{2026} {pending} placed"))
                        .weak()
                        .italics(),
                );
            }
            ui.separator();
            let plane = state.sketch3d_work_plane().unwrap_or("XY").to_string();
            ui.label(egui::RichText::new(format!("Working plane {plane}")).weak())
                .on_hover_text("A click on empty space lands on this plane through the last placed point. Tab cycles XY, YZ, XZ.");
            if let Some(hit) = state
                .sketch3d_edit_session()
                .and_then(|_| last_hit_label(state))
            {
                ui.separator();
                ui.label(egui::RichText::new(hit).weak().italics());
            }
        });
    }

    /// The in-sketch CONTEXT actions: the constraint palette for the selection
    /// plus Fix / Construction / Delete. No-op unless something is selected.
    pub fn context_card(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.ctx_hits.clear();
        if !state.sketch3d_mode() || state.sketch3d_selection_count() == 0 {
            return;
        }
        let offers: Vec<Value> =
            serde_json::from_str(&state.sketch3d_applicable_constraints_json()).unwrap_or_default();
        let (points, geometries, constraints) = state
            .sketch3d_edit_session()
            .map(|s| s.selection_by_kind())
            .unwrap_or_default();

        let mut items: Vec<ActionItem> = Vec::new();
        for offer in &offers {
            let ctype = offer["type"].as_str().unwrap_or("").to_string();
            let label = offer["label"].as_str().unwrap_or("").to_string();
            if ctype == "fixed" {
                continue; // The Fix action below is the one button for grounding.
            }
            items.push(ActionItem::new(
                format!("constraint:{ctype}"),
                label.clone(),
                format!("{label}: {}", brep_render::brep_kernel::sketch3d_constraint_type(&ctype).map_or("", |t| t.meaning)),
            ));
        }
        if !points.is_empty() {
            let all_fixed = state
                .sketch3d_edit_session()
                .map_or(false, |s| points.iter().all(|id| s.doc.point(id).map_or(false, |p| p.fixed)));
            let (label, tip) = if all_fixed {
                ("Unfix", "Free the selected points")
            } else {
                ("Fix", "Ground the selected points where they are")
            };
            items.push(ActionItem::new("fix", label, tip));
        }
        if !geometries.is_empty() {
            items.push(ActionItem::new(
                "construction",
                "\u{25D0} Construction",
                "Toggle construction (a construction curve constrains but is not published)",
            ));
        }
        let _ = constraints;
        items.push(ActionItem::new(
            "delete",
            "\u{1F5D1} Delete",
            "Delete the selected entities (Del / Backspace)",
        ));

        let subtitle = format!("{} selected", state.sketch3d_selection_count());
        let clicked = if crate::viewport::touch::compact(ui.ctx()) {
            action_rail(ui, Some("3D sketch actions"), Some(&subtitle), &items, &mut self.ctx_hits)
        } else {
            egui::Frame::popup(ui.style())
            .show(ui, |ui| {
                action_rail(ui, Some("3D sketch actions"), Some(&subtitle), &items, &mut self.ctx_hits)
            })
            .inner
        };

        match clicked.as_deref() {
            Some(key) if key.starts_with("constraint:") => {
                if let Err(error) = state.sketch3d_add_constraint(&key["constraint:".len()..]) {
                    state.push_notice(format!("3D sketch: {error}"));
                }
            }
            Some("fix") => state.sketch3d_toggle_fixed(),
            Some("construction") => state.sketch3d_toggle_construction(),
            Some("delete") => state.sketch3d_delete_selection(),
            _ => {}
        }
    }

    /// The LEFT panel: Curves / Points / Constraints / References.
    pub fn show_entity_lists(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.list_hits.clear();
        let hits = &mut self.list_hits;
        egui::ScrollArea::vertical().show(ui, |ui| {
            hits.insert("panel:clip".into(), ui.clip_rect());
            ui.add_space(4.0);
            let rows: Value = state
                .sketch3d_edit_session()
                .map(|s| s.entity_rows_json())
                .unwrap_or_default();
            let selected = |kind: &str, id: &Value| -> bool {
                state.sketch3d_edit_session().map_or(false, |s| {
                    let r = match kind {
                        "point" => entity_point_ref(id),
                        "geometry" => entity_geometry_ref(id),
                        _ => entity_constraint_ref(id),
                    };
                    s.is_selected(&r)
                })
            };
            let curves: Vec<Row> = rows["geometries"]
                .as_array()
                .map(|a| {
                    a.iter()
                        .map(|g| Row {
                            id: g["id"].clone(),
                            label: format!(
                                "{} {}{}",
                                g["type"].as_str().unwrap_or("?"),
                                id_text(&g["id"]),
                                if g["construction"] == true { " (construction)" } else { "" }
                            ),
                            selected: selected("geometry", &g["id"]),
                            conflicting: false,
                        })
                        .collect()
                })
                .unwrap_or_default();
            let points: Vec<Row> = rows["points"]
                .as_array()
                .map(|a| {
                    a.iter()
                        .map(|p| {
                            let pos = &p["position"];
                            Row {
                                id: p["id"].clone(),
                                label: format!(
                                    "P{} ({:.2}, {:.2}, {:.2}){}{}",
                                    id_text(&p["id"]),
                                    pos[0].as_f64().unwrap_or(0.0),
                                    pos[1].as_f64().unwrap_or(0.0),
                                    pos[2].as_f64().unwrap_or(0.0),
                                    if p["fixed"] == true { " \u{23DA}" } else { "" },
                                    if p["externalReference"] == true { " ref" } else { "" }
                                ),
                                selected: selected("point", &p["id"]),
                                conflicting: false,
                            }
                        })
                        .collect()
                })
                .unwrap_or_default();
            let constraints: Vec<Row> = rows["constraints"]
                .as_array()
                .map(|a| {
                    a.iter()
                        .map(|c| Row {
                            id: c["id"].clone(),
                            label: match c["value"].as_f64() {
                                Some(v) => format!("{} {} = {v:.3}", c["type"].as_str().unwrap_or("?"), id_text(&c["id"])),
                                None => format!("{} {}", c["type"].as_str().unwrap_or("?"), id_text(&c["id"])),
                            },
                            selected: selected("constraint", &c["id"]),
                            conflicting: c["conflicting"] == true,
                        })
                        .collect()
                })
                .unwrap_or_default();
            entity_section(ui, &mut self.curves_collapsed, state, "Curves", "geometry", curves, hits);
            entity_section(ui, &mut self.points_collapsed, state, "Points", "point", points, hits);
            entity_section(ui, &mut self.constraints_collapsed, state, "Constraints", "constraint", constraints, hits);
            references_section(ui, &mut self.references_collapsed, state, hits);
        });
    }

    /// The actions card's widget rects (`__brepSketch3dHit`); empty out of 3D sketch mode.
    pub fn hits_json(&self, state: &EngineState) -> String {
        if !state.sketch3d_mode() {
            return "{}".to_string();
        }
        crate::automation::hit_rects::hits_json(&self.ctx_hits)
    }

    /// The entity lists' widget rects (`__brepSketch3dListHit`); empty out of 3D sketch mode.
    pub fn list_hits_json(&self, state: &EngineState) -> String {
        if !state.sketch3d_mode() {
            return "{}".to_string();
        }
        crate::automation::hit_rects::hits_json(&self.list_hits)
    }

    /// The `__brepSketch3d` state: the engine's own readback.
    pub fn published_json(&self, state: &EngineState) -> String {
        state.sketch3d_state_json()
    }
}

/// What the last placement landed on, for the status row.
fn last_hit_label(state: &EngineState) -> Option<String> {
    let st: Value = serde_json::from_str(&state.sketch3d_state_json()).ok()?;
    let hit = st["lastHit"].as_str()?;
    Some(match hit {
        "point" => "last click: an existing point".into(),
        "vertex" => "last click: a model vertex (linked)".into(),
        "edge" => "last click: a model edge (linked)".into(),
        "face" => "last click: a model face (linked)".into(),
        _ => "last click: the working plane".into(),
    })
}

fn id_text(id: &Value) -> String {
    match id {
        Value::String(s) => s.clone(),
        other => other.to_string(),
    }
}

fn conflict_color(state: &EngineState) -> egui::Color32 {
    let c = state.settings.sketch_colors().conflict;
    egui::Color32::from_rgb((c >> 16) as u8, (c >> 8) as u8, c as u8)
}

/// The DOF readout — the planar sketch's, over the 3D diagnostics (the same
/// contract: conflicting / over / under / well).
fn dof_readout(ui: &mut egui::Ui, diag: &brep_render::brep_kernel::Sketch3dDiagnostics, conflict: egui::Color32) {
    let (color, label): (egui::Color32, String) = if diag.conflicting {
        let n = diag.conflicting_constraints.len();
        (conflict, format!("Conflicting constraints ({n})"))
    } else if diag.status == "over" || diag.redundant > 0 {
        let label = if diag.dof > 0 {
            format!("Over-constrained ({} redundant, {} DOF)", diag.redundant, diag.dof)
        } else {
            format!("Over-constrained ({} redundant)", diag.redundant)
        };
        (egui::Color32::from_rgb(0xff, 0xcf, 0x5c), label)
    } else if diag.status == "under" || diag.dof > 0 {
        (egui::Color32::from_rgb(0x4a, 0xa3, 0xff), format!("Under-constrained \u{2014} {} DOF", diag.dof))
    } else {
        (egui::Color32::from_rgb(0x7e, 0xe0, 0xa6), "Fully constrained".to_string())
    };
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
        ui.painter().circle_filled(rect.center(), 5.0, color);
        ui.label(egui::RichText::new(label).strong());
    });
}

struct Row {
    id: Value,
    label: String,
    selected: bool,
    conflicting: bool,
}

fn entity_section(
    ui: &mut egui::Ui,
    collapsed: &mut bool,
    state: &mut EngineState,
    title: &str,
    kind: &'static str,
    rows: Vec<Row>,
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
        tree::node(ui, TreeRow::leaf(&base, true, "\u{2014}"), |_| {});
        return;
    }
    let n = rows.len();
    let conflict = conflict_color(state);
    let mut to_delete: Option<Value> = None;
    let entity_ref = |id: &Value| match kind {
        "point" => entity_point_ref(id),
        "geometry" => entity_geometry_ref(id),
        _ => entity_constraint_ref(id),
    };
    for (i, row) in rows.iter().enumerate() {
        let last = i + 1 == n;
        let mut delete_clicked = false;
        let mut del_rect = egui::Rect::NOTHING;
        let tint = row.conflicting.then_some(conflict);
        let resp = tree::node(
            ui,
            TreeRow::leaf(&base, last, row.label.as_str())
                .selected(row.selected)
                .tint(tint),
            |ui| {
                let del = crate::icon_text::icon_button(ui, "\u{2715}").small();
                let resp = ui.add(del).on_hover_text("Delete");
                del_rect = resp.rect;
                if resp.clicked() {
                    delete_clicked = true;
                }
            },
        );
        let key = brep_render::brep_kernel::sketch_id_key(&row.id);
        hits.insert(format!("{kind}:{key}"), resp.label.rect);
        hits.insert(format!("del:{kind}:{key}"), del_rect);
        if delete_clicked {
            to_delete = Some(row.id.clone());
        } else if resp.clicked() {
            let additive = ui.input(|i| i.modifiers.command || i.modifiers.ctrl);
            state.sketch3d_select_entity(entity_ref(&row.id), additive);
        }
    }
    if let Some(id) = to_delete {
        state.sketch3d_select_entity(entity_ref(&id), false);
        state.sketch3d_delete_selection();
    }
}

/// The linked model references: one row per record, with an Unlink ✕.
fn references_section(
    ui: &mut egui::Ui,
    collapsed: &mut bool,
    state: &mut EngineState,
    hits: &mut HashMap<String, egui::Rect>,
) {
    let st: Value = serde_json::from_str(&state.sketch3d_state_json()).unwrap_or_default();
    let refs: Vec<Value> = st["externalRefs"].as_array().cloned().unwrap_or_default();
    let open = !*collapsed;
    let resp = tree::node(ui, TreeRow::branch(&[], true, open, "References"), |ui| {
        ui.add_space(6.0);
        ui.label(egui::RichText::new(format!("{}", refs.len())).weak());
    });
    hits.insert("section:References".into(), resp.label.rect);
    if resp.toggled || resp.clicked() {
        *collapsed = !*collapsed;
    }
    if !open {
        return;
    }
    let base = tree::child_guides(&[], true);
    if refs.is_empty() {
        tree::node(ui, TreeRow::leaf(&base, true, "\u{2014}"), |_| {});
        return;
    }
    let n = refs.len();
    let mut unlink: Option<String> = None;
    for (i, record) in refs.iter().enumerate() {
        let name = record["name"].as_str().unwrap_or("?").to_string();
        let kind = record["kind"].as_str().unwrap_or("?");
        let label = format!("{name} ({kind})");
        let mut del_rect = egui::Rect::NOTHING;
        let mut clicked = false;
        let resp = tree::node(ui, TreeRow::leaf(&base, i + 1 == n, label.as_str()), |ui| {
            let del = crate::icon_text::icon_button(ui, "\u{2715}").small();
            let resp = ui.add(del).on_hover_text("Unlink this reference (its construction geometry goes with it)");
            del_rect = resp.rect;
            if resp.clicked() {
                clicked = true;
            }
        });
        hits.insert(format!("reference:{name}"), resp.label.rect);
        hits.insert(format!("unlink:{name}"), del_rect);
        if clicked {
            unlink = Some(name);
        }
    }
    if let Some(name) = unlink {
        state.sketch3d_unlink_reference(&name);
    }
}

/// The hit keys this panel publishes.
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "sketch3d", prefix: "constraint:", meaning: "add the named constraint to the 3D sketch selection (constraint:<type>, e.g. constraint:parallel)", command: None },
    HitKeyDoc { panel: "sketch3d", prefix: "fix", meaning: "ground / free the selected 3D sketch points", command: None },
    HitKeyDoc { panel: "sketch3d", prefix: "construction", meaning: "toggle construction on the selected 3D sketch curves", command: None },
    HitKeyDoc { panel: "sketch3d", prefix: "delete", meaning: "delete the 3D sketch selection", command: None },
    HitKeyDoc { panel: "sketch3dlist", prefix: "section:", meaning: "a 3D sketch entity-list section header (Curves, Points, Constraints, References)", command: None },
    HitKeyDoc { panel: "sketch3dlist", prefix: "geometry:", meaning: "a 3D sketch curve row, by id (click selects, Ctrl adds)", command: None },
    HitKeyDoc { panel: "sketch3dlist", prefix: "point:", meaning: "a 3D sketch point row, by id", command: None },
    HitKeyDoc { panel: "sketch3dlist", prefix: "constraint:", meaning: "a 3D sketch constraint row, by id", command: None },
    HitKeyDoc { panel: "sketch3dlist", prefix: "del:", meaning: "the \u{2715} on a 3D sketch entity row (del:<kind>:<id>)", command: None },
    HitKeyDoc { panel: "sketch3dlist", prefix: "reference:", meaning: "a linked model reference row, by name", command: None },
    HitKeyDoc { panel: "sketch3dlist", prefix: "unlink:", meaning: "the \u{2715} on a linked reference row: unlink it", command: None },
    HitKeyDoc { panel: "sketch3dlist", prefix: "panel:clip", meaning: "the entity list's visible region", command: None },
];
