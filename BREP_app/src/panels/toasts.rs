//! Transient toast notifications — a small overlay for engine notices (e.g. a
//! sketch solve that failed after an edit). The engine only QUEUES notices
//! ([`brep_render::engine_state::EngineState::take_notices`]); this drains them
//! and shows each as an auto-expiring card in the bottom-left corner of the
//! central tile. Time comes from egui (`ctx.input().time`) — never
//! `std::time::Instant`, which aborts on wasm.
//!
//! # The four rules (round five's audit, item 12)
//!
//! Six stacked cards once covered the lower third of a schematic, a click aimed
//! at a part under them selected nothing, and errors stayed up after they had
//! been dealt with. So:
//!
//! - **A bounded stack.** At most [`MAX_VISIBLE`] cards are drawn; older ones
//!   fold into one `+N older` chip above them.
//! - **Repeats coalesce.** A message equal to one already up does not add a
//!   card: that card counts it (`×3`), restarts its clock and moves to the
//!   bottom as the newest.
//! - **A resolved error retires.** When the document changes after a card was
//!   raised, the card is SUPERSEDED — whatever it complained about was acted on
//!   — and leaves once it has been up [`MIN_SHOWN`], instead of sitting out
//!   the whole [`TTL`]. A card raised again after that change is a new
//!   complaint about the new state, and stays.
//! - **A card eats clicks only inside itself.** Each card is its OWN area, so
//!   the gap between two cards, and everything around the stack, is the canvas.
//!   A click on a card dismisses it (the card says so on hover). The cards sit
//!   in the canvas's bottom-left corner, away from the middle where parts are
//!   placed, the ViewCube (bottom-right) and the eCAD card (top-right).
//!   (The old single area was non-interactable and still ate the clicks: egui's
//!   hit test stops at the topmost layer holding a widget that covers the
//!   pointer — `hit_test.rs`, "nothing behind this layer could ever be
//!   interacted with" — and the cards' labels were widgets on that layer.)
//!
//! # Severity
//!
//! Each card is drawn in its notice's [`Severity`]: red for an error, amber for
//! a warning, green for a success. Every card used to be red, so "Updated 2
//! part(s)" read as a failure (round seven, lane X).

use brep_render::engine_state::NoticeSeverity as Severity;
use eframe::egui;
use std::collections::HashMap;

/// Seconds a toast stays on screen before it fades out of the list.
const TTL: f64 = 6.0;
/// Seconds a SUPERSEDED toast (the document changed after it) stays, so a
/// message raised just before a fix can still be read.
const MIN_SHOWN: f64 = 1.5;
/// Most cards drawn at once; the rest fold into the `+N older` chip.
pub(crate) const MAX_VISIBLE: usize = 3;
/// Most messages KEPT (drawn or folded). Older ones drop off the top.
const MAX_KEPT: usize = 12;
/// A card's widest and narrowest, in points: one width for short messages, so
/// the stack reads as a column rather than a ragged edge.
const CARD_WIDTH: f32 = 360.0;
const CARD_MIN_WIDTH: f32 = 240.0;
/// Distance from the corner of the free space, and between cards.
const MARGIN: f32 = 12.0;
const GAP: f32 = 6.0;

/// One card: its text, how many times it was raised, when it was last raised,
/// and whether the document has changed since.
#[derive(Debug, Clone, PartialEq)]
struct Toast {
    text: String,
    severity: Severity,
    count: u32,
    born: f64,
    superseded: bool,
}

/// A queue of transient messages, oldest first.
#[derive(Default)]
pub struct Toasts {
    items: Vec<Toast>,
    /// `(document, top of its undo stack)` as of the last
    /// [`Self::note_document`] — the baseline a change is read against.
    document: Option<(u64, Option<u64>)>,
    /// This frame's card rects (`toast:<i>`, top to bottom, and `toast:more`)
    /// for the `__brepToastsHit` blob.
    hits: HashMap<String, egui::Rect>,
}

impl Toasts {
    /// Drop every queued toast (an automation host does this before a
    /// deterministic capture; the texts were already published as notices).
    pub fn dismiss_all(&mut self) -> usize {
        let n = self.items.len();
        self.items.clear();
        self.hits.clear();
        n
    }

    pub fn new() -> Self {
        Self::default()
    }

    /// Queue `messages` (drained from the engine this frame), stamped with `now`
    /// (`ctx.input().time`). A message already up is counted on its card, which
    /// becomes the newest and takes the repeat's severity.
    pub fn extend(&mut self, messages: impl IntoIterator<Item = (Severity, String)>, now: f64) {
        for (severity, text) in messages {
            let count = match self.items.iter().position(|t| t.text == text) {
                Some(at) => self.items.remove(at).count.saturating_add(1),
                None => 1,
            };
            self.items.push(Toast { text, severity, count, born: now, superseded: false });
        }
        let overflow = self.items.len().saturating_sub(MAX_KEPT);
        if overflow > 0 {
            self.items.drain(0..overflow);
        }
    }

    /// Tell the overlay which document is up and the step on top of its undo
    /// stack (`History::undo_top`). When that step changes on the SAME
    /// document — an edit, an undo, a redo — every card raised before now is
    /// superseded. A document switch only re-baselines: another model's edit
    /// resolves nothing here.
    ///
    /// The undo stack and not the document's edit key, because the key also
    /// moves on writes nobody made: ENTERING the PCB workbench moved it
    /// (measured, `ecad-toasts-corner`), which retired every card on the way
    /// in. What the user can undo is what the user did.
    pub fn note_document(&mut self, document: u64, key: Option<u64>, now: f64) {
        let previous = self.document.replace((document, key));
        if matches!(previous, Some((d, k)) if d == document && k != key) {
            for toast in self.items.iter_mut().filter(|t| t.born < now) {
                toast.superseded = true;
            }
        }
    }

    /// Drop what has had its time: every card after [`TTL`], a superseded one
    /// after [`MIN_SHOWN`].
    fn retire(&mut self, now: f64) {
        self.items.retain(|t| {
            let age = now - t.born;
            age < TTL && !(t.superseded && age >= MIN_SHOWN)
        });
    }

    /// How many kept messages are folded into the chip rather than drawn.
    fn folded(&self) -> usize {
        self.items.len().saturating_sub(MAX_VISIBLE)
    }

    /// The queued toast texts, oldest first, as a JSON array — the
    /// `__brepNotices` global the headed verifier reads to see a refusal the
    /// app only ever shows as a transient card. One entry per CARD: a repeat is
    /// counted on its card (see [`Self::state_json`]), not listed twice.
    pub fn texts_json(&self) -> String {
        serde_json::Value::Array(
            self.items.iter().map(|t| serde_json::Value::String(t.text.clone())).collect(),
        )
        .to_string()
    }

    /// The `__brepToasts` global: every kept card with its count and whether it
    /// is drawn or folded into the chip, oldest first.
    pub fn state_json(&self) -> String {
        let folded = self.folded();
        serde_json::json!({
            "cards": self.items.iter().enumerate().map(|(i, t)| serde_json::json!({
                "text": t.text,
                "severity": severity_name(t.severity),
                "count": t.count,
                "shown": i >= folded,
                "superseded": t.superseded,
            })).collect::<Vec<_>>(),
            "shown": self.items.len() - folded,
            "folded": folded,
        })
        .to_string()
    }

    /// The `__brepToastsHit` blob: this frame's card rects.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Draw the unexpired toasts. Call once per frame at ctx level (after the
    /// panels, so the cards float over the shell). `canvas` is the central
    /// tile's rect (the 3D view or the eCAD sheet, `None` before it has drawn)
    /// and `floor` the top of the status bar: the stack rises from the
    /// canvas's BOTTOM-LEFT corner. Not the right: the ViewCube stands in the
    /// 3D view's bottom-right, and the eCAD card in the sheet's top-right.
    pub fn show(&mut self, ctx: &egui::Context, canvas: Option<egui::Rect>, floor: f32) {
        let now = ctx.input(|i| i.time);
        self.retire(now);
        self.hits.clear();
        if self.items.is_empty() {
            return;
        }
        // Keep the frame loop alive so a toast expires on time without new input.
        ctx.request_repaint();

        // Before the canvas has a rect, the bottom centre of the window.
        let (pivot, x) = match canvas {
            Some(canvas) => (egui::Align2::LEFT_BOTTOM, canvas.left() + MARGIN),
            None => (egui::Align2::CENTER_BOTTOM, ctx.content_rect().center().x),
        };
        let mut bottom = canvas.map_or(floor, |c| c.bottom().min(floor)) - MARGIN;
        let folded = self.folded();
        let mut dismissed: Option<usize> = None;
        // Newest at the bottom, so the stack grows upward from the corner.
        for index in (folded..self.items.len()).rev() {
            let toast = &self.items[index];
            let text = match toast.count {
                1 => toast.text.clone(),
                n => format!("{}  ×{n}", toast.text),
            };
            let palette = Palette::of(toast.severity);
            let (rect, clicked) = card(ctx, egui::Id::new(("brep-toast", index)), pivot, egui::pos2(x, bottom), palette, |ui| {
                ui.set_min_width(CARD_MIN_WIDTH);
                ui.set_max_width(CARD_WIDTH);
                ui.add(egui::Label::new(egui::RichText::new(text).color(palette.text)).selectable(false));
            });
            self.hits.insert(format!("toast:{}", index - folded), rect);
            if clicked {
                dismissed = Some(index);
            }
            bottom = rect.top() - GAP;
        }
        if folded > 0 {
            let label = format!("+{folded} older");
            // The chip is the folded cards' count, not a message: drawn neutral.
            let palette = Palette::CHIP;
            let (rect, clicked) = card(ctx, egui::Id::new("brep-toast-more"), pivot, egui::pos2(x, bottom), palette, |ui| {
                ui.add(egui::Label::new(egui::RichText::new(label).small().color(palette.text)).selectable(false));
            });
            self.hits.insert("toast:more".into(), rect);
            if clicked {
                self.items.drain(0..folded);
                return;
            }
        }
        if let Some(index) = dismissed {
            self.items.remove(index);
        }
    }
}

/// The `__brepToastsHit` keys.
pub static HIT_KEYS: &[crate::automation::hit_keys::HitKeyDoc] = &[
    crate::automation::hit_keys::HitKeyDoc { panel: "toasts", prefix: "toast:more", meaning: "the `+N older` chip above the stack; a click dismisses the folded cards", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "toasts", prefix: "toast:", meaning: "a drawn toast card, 0 at the top of the stack; a click dismisses it", command: None },
];

/// The name `__brepToasts` publishes a card's severity by.
fn severity_name(severity: Severity) -> &'static str {
    match severity {
        Severity::Error => "error",
        Severity::Warning => "warning",
        Severity::Info => "info",
    }
}

/// A card's fill, border and text. Dark in either theme: the cards float over
/// the canvas, which is dark in both.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Palette {
    fill: egui::Color32,
    stroke: egui::Color32,
    text: egui::Color32,
}

impl Palette {
    const ERROR: Self = Self {
        fill: egui::Color32::from_rgb(0x3a, 0x24, 0x24),
        stroke: egui::Color32::from_rgb(0xff, 0x6b, 0x6b),
        text: egui::Color32::from_rgb(0xff, 0x9b, 0x9b),
    };
    const WARNING: Self = Self {
        fill: egui::Color32::from_rgb(0x38, 0x30, 0x1e),
        stroke: egui::Color32::from_rgb(0xe8, 0xb3, 0x4a),
        text: egui::Color32::from_rgb(0xf5, 0xd4, 0x8f),
    };
    const INFO: Self = Self {
        fill: egui::Color32::from_rgb(0x1f, 0x33, 0x29),
        stroke: egui::Color32::from_rgb(0x5c, 0xc0, 0x86),
        text: egui::Color32::from_rgb(0xb4, 0xe8, 0xc8),
    };
    const CHIP: Self = Self {
        fill: egui::Color32::from_rgb(0x2a, 0x2c, 0x30),
        stroke: egui::Color32::from_rgb(0x70, 0x76, 0x80),
        text: egui::Color32::from_rgb(0xc8, 0xcc, 0xd2),
    };

    fn of(severity: Severity) -> Self {
        match severity {
            Severity::Error => Self::ERROR,
            Severity::Warning => Self::WARNING,
            Severity::Info => Self::INFO,
        }
    }
}

/// One card in its own foreground area, placed by its `pivot` corner at `at`.
/// Returns its rect and whether it was clicked.
fn card(
    ctx: &egui::Context,
    id: egui::Id,
    pivot: egui::Align2,
    at: egui::Pos2,
    palette: Palette,
    add: impl FnOnce(&mut egui::Ui),
) -> (egui::Rect, bool) {
    let response = egui::Area::new(id)
        .order(egui::Order::Foreground)
        .pivot(pivot)
        .fixed_pos(at)
        .sense(egui::Sense::click())
        .show(ctx, |ui| {
            egui::Frame::popup(ui.style())
                .fill(palette.fill)
                .stroke(egui::Stroke::new(1.0, palette.stroke))
                .show(ui, add);
        })
        .response;
    let response = response.on_hover_cursor(egui::CursorIcon::PointingHand).on_hover_text("Click to dismiss");
    (response.rect, response.clicked())
}

