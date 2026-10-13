//! The NURBS Solid CONTROL-CAGE editor — the app side of `brep_render`'s
//! `cage_edit`.
//!
//! Drawn as the **Cage** section of a NURBS Solid feature's form (the form
//! view's consumer section). The cage's points are handles in the VIEWPORT —
//! a click on a dot selects it, a Shift / Ctrl click toggles it into the
//! selection, and the move/rotate gizmo arms at the selection's centroid so a
//! drag moves (or turns) every selected point at once. This section carries
//! what a pointer cannot say: the cage's shape and whether it is the
//! primitive's or an edited one, the whole-loop selections (the ring or the
//! column of the selected points, everything, nothing), the LOOP INSERTIONS
//! (a ring above or below the selected point, a column to either side — knot
//! insertions that refine the cage without moving the body), the selected
//! point's typed position, and Reset back to the primitive.
//!
//! Intent-out like the anchor editor: [`draw`] reports what the user did
//! into a [`CageIntent`] list and [`apply`] acts on ONE of them against the
//! engine after the draw.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::engine_state::{CageSummary, EngineState};
use eframe::egui;
use std::cell::RefCell;

use crate::icon_text::icon_button;

/// What one frame of the cage section asked for.
#[derive(Debug, Clone, PartialEq)]
pub enum CageIntent {
    SelectAll,
    ClearSelection,
    /// Grow the selection to the whole ring of every selected ring point.
    SelectRings,
    /// Grow the selection to the whole column of every selected ring point.
    SelectColumns,
    /// Insert a ring beside the selected point: above (towards the top pole)
    /// or below.
    InsertRing { above: bool },
    /// Insert a column beside the selected point: the next around the ring,
    /// or the previous.
    InsertColumn { next: bool },
    /// Move one point (typed).
    SetPosition(usize, [f64; 3]),
    /// Drop the edited cage: back to the primitive.
    Reset,
}

/// The destructive red the other panels use for a reset affordance.
const RESET_RED: egui::Color32 = egui::Color32::from_rgb(0xd8, 0x54, 0x4f);

/// Draw the cage section into `ui`. Every widget rect is recorded under
/// `cage:{what}` in `hits`. `selected` is the engine's selection (linear
/// point indices) and `points` the cage's points in that order.
pub fn draw(
    ui: &mut egui::Ui,
    summary: Option<&CageSummary>,
    points: &[[f64; 3]],
    selected: &[usize],
    intents: &RefCell<Vec<CageIntent>>,
    hits: &RefCell<Vec<(String, egui::Rect)>>,
) {
    let push = |intent: CageIntent| intents.borrow_mut().push(intent);
    let hit = |key: &str, rect: egui::Rect| hits.borrow_mut().push((format!("cage:{key}"), rect));
    // Clamp to what is on screen (see the anchor editor's note on
    // right-aligned buttons past a scroll area's clip).
    let room = (ui.clip_rect().right() - ui.max_rect().left()).max(0.0);
    ui.set_width(ui.available_width().min(room));

    let Some(summary) = summary else {
        ui.label(
            egui::RichText::new("The cage could not be read: see the error above.").weak(),
        );
        return;
    };

    // --- what the cage is ----------------------------------------------------
    let shape = format!(
        "{} points: {} rings of {}, plus the two poles",
        summary.points, summary.rings, summary.segments
    );
    let shape_label = ui.label(shape).on_hover_text(
        "Click a point in the viewport to select it; Shift or Ctrl click adds or removes one. \
         The move/rotate gizmo sits at the selection's centre and moves every selected point.",
    );
    hit("shape", shape_label.rect);
    let state = if summary.persisted {
        "Edited cage: the primitive fields above are what it started from."
    } else {
        "The primitive's cage: the first move or insertion keeps a copy here."
    };
    ui.label(egui::RichText::new(state).weak());
    let selected_text = match selected.len() {
        0 => "Nothing selected".to_string(),
        1 => format!("Point {} selected", selected[0]),
        n => format!("{n} points selected"),
    };
    let selected_label = ui.label(egui::RichText::new(selected_text).strong());
    hit("selected", selected_label.rect);

    // --- the selection -------------------------------------------------------
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new("select").weak());
        let all = ui
            .add(icon_button(ui, "All").small())
            .on_hover_text("Select every point: the gizmo then moves the whole body");
        hit("select-all", all.rect);
        if all.clicked() {
            push(CageIntent::SelectAll);
        }
        let any = !selected.is_empty();
        let rings = ui
            .add_enabled(any, icon_button(ui, "Ring").small())
            .on_hover_text("Grow the selection to the whole ring of each selected point");
        hit("select-ring", rings.rect);
        if rings.clicked() {
            push(CageIntent::SelectRings);
        }
        let columns = ui
            .add_enabled(any, icon_button(ui, "Column").small())
            .on_hover_text("Grow the selection to the whole pole-to-pole column of each selected point");
        hit("select-column", columns.rect);
        if columns.clicked() {
            push(CageIntent::SelectColumns);
        }
        let clear = ui
            .add_enabled(any, icon_button(ui, "None").small())
            .on_hover_text("Clear the selection (the gizmo goes away)");
        hit("clear", clear.rect);
        if clear.clicked() {
            push(CageIntent::ClearSelection);
        }
    });

    // --- loop insertion ------------------------------------------------------
    let any = !selected.is_empty();
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new("insert ring").weak());
        let above = ui
            .add_enabled(any, icon_button(ui, "Above").small())
            .on_hover_text("Insert a ring of points between the selected point's ring and the next towards the top pole. The body does not move; the new ring is selected.");
        hit("insert-ring-above", above.rect);
        if above.clicked() {
            push(CageIntent::InsertRing { above: true });
        }
        let below = ui
            .add_enabled(any, icon_button(ui, "Below").small())
            .on_hover_text("Insert a ring of points between the selected point's ring and the next towards the bottom pole. The body does not move; the new ring is selected.");
        hit("insert-ring-below", below.rect);
        if below.clicked() {
            push(CageIntent::InsertRing { above: false });
        }
    });
    ui.horizontal_wrapped(|ui| {
        ui.label(egui::RichText::new("insert column").weak());
        let next = ui
            .add_enabled(any, icon_button(ui, "Next").small())
            .on_hover_text("Insert a pole-to-pole column of points between the selected point's column and the next around the ring. The body does not move; the new column is selected.");
        hit("insert-column-next", next.rect);
        if next.clicked() {
            push(CageIntent::InsertColumn { next: true });
        }
        let previous = ui
            .add_enabled(any, icon_button(ui, "Previous").small())
            .on_hover_text("Insert a pole-to-pole column of points between the selected point's column and the previous one. The body does not move; the new column is selected.");
        hit("insert-column-prev", previous.rect);
        if previous.clicked() {
            push(CageIntent::InsertColumn { next: false });
        }
    });

    // --- the selected point's position ---------------------------------------
    if let [index] = selected {
        if let Some(point) = points.get(*index) {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("at").weak());
                let mut position = *point;
                let mut changed = false;
                for (axis, name) in ["x", "y", "z"].iter().enumerate() {
                    let drag = ui.add(
                        egui::DragValue::new(&mut position[axis])
                            .speed(0.1)
                            .prefix(format!("{name} ")),
                    );
                    hit(name, drag.rect);
                    changed |= drag.changed();
                }
                if changed {
                    push(CageIntent::SetPosition(*index, position));
                }
            });
        }
    } else if selected.len() > 1 {
        let mut sum = [0.0; 3];
        let mut count = 0.0;
        for index in selected {
            if let Some(point) = points.get(*index) {
                for k in 0..3 {
                    sum[k] += point[k];
                }
                count += 1.0;
            }
        }
        if count > 0.0 {
            ui.label(
                egui::RichText::new(format!(
                    "centre ({:.2}, {:.2}, {:.2})",
                    sum[0] / count,
                    sum[1] / count,
                    sum[2] / count
                ))
                .weak(),
            );
        }
    }

    // --- reset ---------------------------------------------------------------
    let reset = ui
        .add_enabled(
            summary.persisted,
            crate::icon_text::icon_button_colored(ui, "Reset cage", Some(RESET_RED)).small(),
        )
        .on_hover_text("Drop the edited cage: the body returns to the primitive the fields above describe");
    hit("reset", reset.rect);
    if reset.clicked() {
        push(CageIntent::Reset);
    }
}

/// Act on ONE intent against the engine. Refusals surface as notices;
/// nothing here panics on a stale index.
pub fn apply(state: &mut EngineState, feature_id: &str, intent: CageIntent) {
    let notice = |state: &mut EngineState, result: Result<(), String>| {
        if let Err(error) = result {
            state.push_notice(error);
        }
    };
    match intent {
        CageIntent::SelectAll => state.cage_select_all(feature_id),
        CageIntent::ClearSelection => state.cage_clear_selection(feature_id),
        CageIntent::SelectRings => state.cage_select_rings_of_selection(feature_id),
        CageIntent::SelectColumns => state.cage_select_columns_of_selection(feature_id),
        CageIntent::InsertRing { above } => {
            let result = state.cage_insert_ring(feature_id, above).map(|_| ());
            notice(state, result);
        }
        CageIntent::InsertColumn { next } => {
            let result = state.cage_insert_column(feature_id, next).map(|_| ());
            notice(state, result);
        }
        CageIntent::SetPosition(index, position) => {
            let result = state.cage_set_point_position(feature_id, index, position);
            notice(state, result);
        }
        CageIntent::Reset => {
            let result = state.cage_reset(feature_id);
            notice(state, result);
        }
    }
}


/// The hit keys this section publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "history", prefix: "cage:", meaning: "a control of the open NURBS Solid's Cage section (cage:shape, :selected, :select-all, :select-ring, :select-column, :clear, :insert-ring-above, :insert-ring-below, :insert-column-next, :insert-column-prev, :x, :y, :z, :reset) — the points themselves are viewport handles, published as `cage/point:<i>`", command: None },
];
