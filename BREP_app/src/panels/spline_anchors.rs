//! The spline ANCHOR editor — the app side of `brep_render`'s `spline_edit`.
//!
//! Drawn as the **Anchors** section of a Spline feature's form (the form
//! view's consumer section): one row per through-point with its position,
//! its forward / backward straight-run distances, the flip toggle, its port
//! attachment (Attach… / the port's name with an A|B side switch and Detach)
//! and the reorder / insert / remove buttons; a footer adds an anchor at the
//! end. Selecting a row arms the move/rotate gizmo on that anchor in the
//! viewport, so an anchor is placed by dragging as well as by typing; the
//! direction cage the engine overlays shows which way each anchor travels.
//!
//! Intent-out like the form view: [`draw`] reports what the user did into a
//! [`AnchorIntent`] list and [`apply`] acts on ONE of them against the engine
//! after the draw. An attached anchor's placement belongs to its port, so its
//! position and flip are shown read-only there and the gizmo refuses it.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::brep_kernel::PortSide;
use brep_render::engine_state::{EngineState, SplineAnchorRow};
use eframe::egui;
use std::cell::RefCell;

use crate::icon_text::{icon_button, icon_button_colored, selectable_icon_label};

/// The row's glyphs — every one a catalogued icon (`assets/glyphs`), drawn as
/// artwork by [`crate::icon_text`]; nothing here relies on a font having the
/// character.
const GLYPH_SELECT: &str = "\u{25CE}"; // ◎ arm the gizmo on this anchor
const GLYPH_ATTACHED: &str = "\u{26D3}"; // ⛓ attached to a port / Attach…
const GLYPH_DETACH: &str = "\u{2702}"; // ✂ cut the attachment
const GLYPH_REMOVE: &str = "\u{2715}"; // ✕ remove the anchor
const GLYPH_INSERT: &str = "\u{FF0B}"; // ＋ insert / add an anchor
const GLYPH_UP: &str = "\u{2B06}"; // ⬆ earlier along the curve
const GLYPH_DOWN: &str = "\u{2B07}"; // ⬇ later along the curve

/// What one frame of the anchor list asked for.
#[derive(Debug, Clone, PartialEq)]
pub enum AnchorIntent {
    Select(usize),
    SetPosition(usize, [f64; 3]),
    SetForward(usize, f64),
    SetBackward(usize, f64),
    SetFlip(usize, bool),
    SetSide(usize, PortSide),
    Attach(usize),
    Detach(usize),
    /// Insert after the given anchor (`None` = append at the end).
    Add(Option<usize>),
    Remove(usize),
    MoveUp(usize),
    MoveDown(usize),
}

/// The destructive red the other panels use for a remove affordance.
const REMOVE_RED: egui::Color32 = egui::Color32::from_rgb(0xd8, 0x54, 0x4f);

/// Draw the anchor list into `ui`. Every widget rect is recorded under
/// `anchor:{index}:{what}` (and `anchors:add` for the footer) in `hits`.
pub fn draw(
    ui: &mut egui::Ui,
    rows: &[SplineAnchorRow],
    selected: Option<usize>,
    intents: &RefCell<Vec<AnchorIntent>>,
    hits: &RefCell<Vec<(String, egui::Rect)>>,
) {
    let push = |intent: AnchorIntent| intents.borrow_mut().push(intent);
    let hit = |key: String, rect: egui::Rect| hits.borrow_mut().push((key, rect));
    let count = rows.len();

    for row in rows {
        let i = row.index;
        let attached = row.attached.clone();
        egui::Frame::group(ui.style())
            .inner_margin(egui::Margin::same(4))
            .show(ui, |ui| {
                // The width to lay out in. `available_width` is measured from a
                // Ui whose `max_rect` an outer scroll area can make WIDER than
                // the visible clip, and row 1's ✕ / ＋ / ▲ / ▼ are RIGHT-aligned
                // inside this frame: a frame wider than the pane puts them past
                // the pane's edge, published and unclickable. `anchor:1:remove`
                // and `anchor:2:remove` sat at x 473 in a pane clipped at 419.7,
                // so `click_widget` aimed into the viewport and the press did
                // nothing — found by the wire-harness verifier migration
                // (2026-09-13), the same shape as the BOM's stranded actions
                // column. Clamp to what is actually on screen.
                let room = (ui.clip_rect().right() - ui.max_rect().left()).max(0.0);
                ui.set_width(ui.available_width().min(room));
                // --- row 1: select | reorder | insert | remove ---------------
                ui.horizontal(|ui| {
                    let select_text = format!("{GLYPH_SELECT} P{i}");
                    let label = selectable_icon_label(ui, selected == Some(i), &select_text)
                        .on_hover_text("Select this anchor: the move/rotate gizmo arms on it in the viewport (a free anchor); clicking its dot in the viewport does the same");
                    hit(format!("anchor:{i}:select"), label.rect);
                    if label.clicked() {
                        push(AnchorIntent::Select(i));
                    }
                    if let Some((port, side)) = &attached {
                        let attached_color = egui::Color32::from_rgb(0xff, 0xa8, 0x6b);
                        if let Some(art) = crate::icon_text::glyph(ui, GLYPH_ATTACHED, attached_color) {
                            ui.add(art);
                        }
                        ui.label(egui::RichText::new(port.as_str()).color(attached_color))
                            .on_hover_text("Attached to this port: the port places the anchor");
                        for candidate in [PortSide::A, PortSide::B] {
                            let button = ui
                                .selectable_label(*side == candidate, candidate.letter())
                                .on_hover_text(match candidate {
                                    PortSide::A => "Leave the port along its direction",
                                    PortSide::B => "Leave the port against its direction",
                                });
                            hit(format!("anchor:{i}:side{}", candidate.letter()), button.rect);
                            if button.clicked() && *side != candidate {
                                push(AnchorIntent::SetSide(i, candidate));
                            }
                        }
                        let detach_text = format!("{GLYPH_DETACH} Detach");
                        let detach = ui
                            .add(icon_button(ui, &detach_text).small())
                            .on_hover_text("Keep the anchor where the port put it, but stop following the port");
                        hit(format!("anchor:{i}:detach"), detach.rect);
                        if detach.clicked() {
                            push(AnchorIntent::Detach(i));
                        }
                    } else {
                        let attach_text = format!("{GLYPH_ATTACHED} Attach\u{2026}");
                        let attach = ui
                            .add(icon_button(ui, &attach_text).small())
                            .on_hover_text("Pick a port in the viewport: the anchor snaps to it and follows it");
                        hit(format!("anchor:{i}:attach"), attach.rect);
                        if attach.clicked() {
                            push(AnchorIntent::Attach(i));
                        }
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let remove = ui
                            .add_enabled(
                                count > 2,
                                icon_button_colored(ui, GLYPH_REMOVE, Some(REMOVE_RED)).small(),
                            )
                            .on_hover_text("Remove this anchor (a spline keeps at least two)");
                        hit(format!("anchor:{i}:remove"), remove.rect);
                        if remove.clicked() {
                            push(AnchorIntent::Remove(i));
                        }
                        let insert = ui
                            .add(icon_button(ui, GLYPH_INSERT).small())
                            .on_hover_text("Insert an anchor after this one");
                        hit(format!("anchor:{i}:insert"), insert.rect);
                        if insert.clicked() {
                            push(AnchorIntent::Add(Some(i)));
                        }
                        let down = ui
                            .add_enabled(i + 1 < count, icon_button(ui, GLYPH_DOWN).small())
                            .on_hover_text("Move this anchor later along the curve");
                        hit(format!("anchor:{i}:down"), down.rect);
                        if down.clicked() {
                            push(AnchorIntent::MoveDown(i));
                        }
                        let up = ui
                            .add_enabled(i > 0, icon_button(ui, GLYPH_UP).small())
                            .on_hover_text("Move this anchor earlier along the curve");
                        hit(format!("anchor:{i}:up"), up.rect);
                        if up.clicked() {
                            push(AnchorIntent::MoveUp(i));
                        }
                    });
                });
                // --- row 2: position ------------------------------------------
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("at").weak());
                    let mut position = row.position;
                    let mut changed = false;
                    for (axis, name) in ["x", "y", "z"].iter().enumerate() {
                        let drag = ui.add_enabled(
                            attached.is_none(),
                            egui::DragValue::new(&mut position[axis]).speed(0.1).prefix(format!("{name} ")),
                        );
                        hit(format!("anchor:{i}:{name}"), drag.rect);
                        changed |= drag.changed();
                    }
                    if changed {
                        push(AnchorIntent::SetPosition(i, position));
                    }
                    if attached.is_some() {
                        ui.label(egui::RichText::new("(placed by the port)").weak());
                    }
                });
                // --- row 3: straight runs + flip ------------------------------
                ui.horizontal(|ui| {
                    ui.label(egui::RichText::new("straight").weak());
                    let mut forward = row.forward;
                    let fwd = ui
                        .add(egui::DragValue::new(&mut forward).speed(0.1).range(0.0..=f64::INFINITY).prefix("fwd "))
                        .on_hover_text("How far the curve runs straight OUT of this anchor before bending");
                    hit(format!("anchor:{i}:fwd"), fwd.rect);
                    if fwd.changed() {
                        push(AnchorIntent::SetForward(i, forward));
                    }
                    let mut backward = row.backward;
                    let back = ui
                        .add(egui::DragValue::new(&mut backward).speed(0.1).range(0.0..=f64::INFINITY).prefix("back "))
                        .on_hover_text("How far the curve runs straight INTO this anchor after its bend");
                    hit(format!("anchor:{i}:back"), back.rect);
                    if back.changed() {
                        push(AnchorIntent::SetBackward(i, backward));
                    }
                    let mut flip = row.flip;
                    let flip_box = ui
                        .add_enabled(attached.is_none(), egui::Checkbox::new(&mut flip, "flip"))
                        .on_hover_text("Reverse this anchor's travel direction");
                    hit(format!("anchor:{i}:flip"), flip_box.rect);
                    if flip_box.changed() {
                        push(AnchorIntent::SetFlip(i, flip));
                    }
                    ui.label(
                        egui::RichText::new(format!(
                            "dir ({:.2}, {:.2}, {:.2})",
                            row.direction[0], row.direction[1], row.direction[2]
                        ))
                        .weak(),
                    );
                });
            });
    }

    let add_text = format!("{GLYPH_INSERT} Add anchor");
    let add = ui
        .add(icon_button(ui, &add_text))
        .on_hover_text("Append an anchor past the last one, along its direction");
    hit("anchors:add".to_string(), add.rect);
    if add.clicked() {
        push(AnchorIntent::Add(None));
    }
}

/// Act on ONE intent against the engine. Returns the anchor the editor should
/// show selected afterwards (`selected` carried over when the intent leaves
/// it alone). Refusals surface as notices; nothing here panics on a stale
/// index.
pub fn apply(
    state: &mut EngineState,
    feature_id: &str,
    intent: AnchorIntent,
    selected: Option<usize>,
) -> Option<usize> {
    let notice = |state: &mut EngineState, result: Result<(), String>| {
        if let Err(error) = result {
            state.push_notice(error);
        }
    };
    match intent {
        AnchorIntent::Select(i) => {
            // An attached anchor still selects (the cage highlights it); only
            // the gizmo is refused, since the port owns the pose.
            state.arm_spline_anchor(feature_id, i);
            Some(i)
        }
        AnchorIntent::SetPosition(i, position) => {
            let result = state.spline_set_anchor_position(feature_id, i, position);
            notice(state, result);
            selected
        }
        AnchorIntent::SetForward(i, forward) => {
            let result = state.spline_set_anchor_distances(feature_id, i, Some(forward), None);
            notice(state, result);
            selected
        }
        AnchorIntent::SetBackward(i, backward) => {
            let result = state.spline_set_anchor_distances(feature_id, i, None, Some(backward));
            notice(state, result);
            selected
        }
        AnchorIntent::SetFlip(i, flip) => {
            let result = state.spline_set_anchor_flip(feature_id, i, flip);
            notice(state, result);
            selected
        }
        AnchorIntent::SetSide(i, side) => {
            let result = state.spline_set_anchor_side(feature_id, i, side);
            notice(state, result);
            selected
        }
        AnchorIntent::Attach(i) => {
            state.begin_ref_select_for_spline_anchor(feature_id, i);
            Some(i)
        }
        AnchorIntent::Detach(i) => {
            let result = state.spline_detach_anchor(feature_id, i);
            notice(state, result);
            selected
        }
        AnchorIntent::Add(after) => match state.spline_add_anchor(feature_id, after) {
            Ok(index) => {
                state.arm_spline_anchor(feature_id, index);
                Some(index)
            }
            Err(error) => {
                state.push_notice(error);
                selected
            }
        },
        AnchorIntent::Remove(i) => {
            let result = state.spline_remove_anchor(feature_id, i);
            notice(state, result);
            match selected {
                Some(s) if s == i => None,
                Some(s) if s > i => Some(s - 1),
                other => other,
            }
        }
        AnchorIntent::MoveUp(i) => {
            let result = state.spline_move_anchor(feature_id, i, true);
            notice(state, result);
            match selected {
                Some(s) if s == i && i > 0 => Some(i - 1),
                Some(s) if s + 1 == i => Some(i),
                other => other,
            }
        }
        AnchorIntent::MoveDown(i) => {
            let result = state.spline_move_anchor(feature_id, i, false);
            notice(state, result);
            match selected {
                Some(s) if s == i => Some(i + 1),
                Some(s) if s == i + 1 => Some(i),
                other => other,
            }
        }
    }
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "history", prefix: "anchor:", meaning: "a spline anchor row control (anchor:i:select, :up, :down, :insert, :remove, :attach, :detach, :flip, :fwd, :back, :side, :name)", command: None },
    // NOT covered by the `anchor:` prefix above — the plural misses it by one
    // character, so the editor's Add anchor button published an undocumented
    // key for as long as it has existed. No script had opened the anchor editor
    // and run `hit_keys_check` before the verifier migration.
    HitKeyDoc { panel: "history", prefix: "anchors:add", meaning: "append an anchor to the open spline's list", command: None },
];
