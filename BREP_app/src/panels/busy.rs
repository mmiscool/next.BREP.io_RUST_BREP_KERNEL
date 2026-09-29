//! The app-wide WORKING INDICATOR at the bottom right of the status bar: one
//! spinner, the label of the most important thing in flight, `+N more` when
//! several are, and a Cancel for work that can be cancelled.
//!
//! It used to be a spinner in the History tree's header, which only a user
//! looking at the History tab could see. It lives in the status bar now, which
//! every workbench, tab and mode draws, so "is anything happening?" has one
//! answer in one place.
//!
//! # How work reports in
//!
//! POLLED, not pushed: each frame the shell collects an [`Activity`] for every
//! piece of work in flight ([`engine_activities`] for a document's runner
//! queues, plus whatever panels report) and hands the list to
//! [`BusyIndicator::update`]. A source never has to remember to say "done" —
//! work that is no longer reported is over — so a panel that is closed, a
//! document that is closed or a runner that was cancelled cannot strand the
//! spinner. The indicator keeps only what the list cannot carry: when each
//! activity was FIRST seen (its elapsed time) and when the current busy spell
//! began (the anti-flicker delay). Time is egui's clock (`Input::time`), never
//! `Instant`, which panics on wasm32.
//!
//! The list is in PRIORITY order: the first activity is the one the label
//! names, the rest are `+N more` and the hover tooltip.
//!
//! # Cost
//!
//! Idle, it draws nothing and asks for no repaint. Busy but inside the delay it
//! asks for one repaint at the moment the delay ends (so a run that is still
//! going appears without waiting for input); shown, it repaints every frame so
//! the spinner turns and the elapsed time ticks.

use brep_render::engine_state::EngineState;
use eframe::egui;
use std::collections::HashMap;

/// How long work must be in flight before the indicator appears, in seconds.
/// A single param edit lands in a frame or two, and a spinner that blinks on
/// every keystroke is noise, not information.
pub const BUSY_DELAY: f64 = 0.2;

/// From how many seconds in an activity's label carries its elapsed time.
const ELAPSED_AFTER: f64 = 3.0;

/// One piece of work in flight.
#[derive(Debug, Clone, PartialEq)]
pub struct Activity {
    /// Stable identity across frames (its elapsed time is keyed on it), e.g.
    /// `run:<document id>`.
    pub key: String,
    /// The KIND of work, for automation: `run`, `query`, `topology`,
    /// `meshImport`, `stepProbe`, `sheetLines`, `download`, `search`, `boot`, `kicadRead`, `storeWrite`.
    pub kind: &'static str,
    /// What the user reads, e.g. `Rebuilding model — P.S16 (2/2)`.
    pub label: String,
    /// The document the work belongs to, when it belongs to one — Cancel is
    /// sent to THIS document's engine, which need not be the active one.
    pub document: Option<u64>,
    /// Whether the Cancel button offers to stop it.
    pub cancellable: bool,
}

impl Activity {
    pub fn new(key: impl Into<String>, kind: &'static str, label: impl Into<String>) -> Self {
        Self { key: key.into(), kind, label: label.into(), document: None, cancellable: false }
    }
}

/// The work a document's engine has in flight on its runner, most important
/// first. `title` names a BACKGROUND document (its work is not what the user
/// is looking at, so the label says whose it is); `None` for the active one.
pub fn engine_activities(engine: &EngineState, document: u64, title: Option<&str>) -> Vec<Activity> {
    let of = |what: &str| match title {
        Some(title) => format!("{what} ({title})"),
        None => what.to_string(),
    };
    let mut out = Vec::new();
    let mut push = |kind: &'static str, label: String, cancellable: bool| {
        out.push(Activity {
            key: format!("{kind}:{document}"),
            kind,
            label,
            document: Some(document),
            cancellable,
        });
    };
    if engine.run_pending() {
        // The runner posts which feature it is executing before each one; a
        // run that has not reached its first feature yet says only that it
        // is rebuilding.
        let preview = engine.expression_preview().map(|p| p.label.clone());
        let label = match (engine.run_progress(), title) {
            // A family-row preview rebuilding: name the row.
            (Some(p), None) if preview.is_some() => format!(
                "Previewing {} — {} ({}/{})",
                preview.as_deref().unwrap_or_default(),
                p.feature_id,
                p.index + 1,
                p.total
            ),
            (None, None) if preview.is_some() => {
                format!("Previewing {}", preview.as_deref().unwrap_or_default())
            }
            (Some(p), None) => {
                format!("Rebuilding model — {} ({}/{})", p.feature_id, p.index + 1, p.total)
            }
            (Some(p), Some(title)) => {
                format!("Rebuilding {title} — {} ({}/{})", p.feature_id, p.index + 1, p.total)
            }
            (None, None) => "Rebuilding model".to_string(),
            (None, Some(title)) => format!("Rebuilding {title}"),
        };
        push("run", label, true);
    }
    if engine.mesh_imports_pending() {
        push("meshImport", of("Reconstructing mesh"), false);
    }
    if engine.step_probes_pending() {
        push("stepProbe", of("Reading STEP structure"), false);
    }
    if engine.sheet_lines_pending() {
        push("sheetLines", of("Drawing sheet views"), false);
    }
    if engine.topology_pending() {
        push("topology", of("Loading exact geometry"), false);
    }
    if engine.queries_pending() {
        push("query", of("Measuring"), false);
    }
    out
}

/// The status bar's working indicator. See the module docs.
#[derive(Default)]
pub struct BusyIndicator {
    /// This frame's work, in priority order.
    activities: Vec<Activity>,
    /// When each activity (by key) was first seen in the current spell.
    since: HashMap<String, f64>,
    /// When the current busy spell began, `None` while idle.
    busy_since: Option<f64>,
    /// The clock at the last [`Self::update`].
    now: f64,
    /// A Cancel click, for the shell to act on after the draw.
    cancel: Option<Activity>,
    /// This frame's widget rects (`busy`, `busy:more`, `busy:cancel`) for the
    /// `__brepStatusHit` blob — published only while the indicator SHOWS.
    hits: HashMap<String, egui::Rect>,
}

impl BusyIndicator {
    pub fn new() -> Self {
        Self::default()
    }

    /// Take this frame's work (priority order) at egui time `now`.
    pub fn update(&mut self, activities: Vec<Activity>, now: f64) {
        self.now = now;
        if activities.is_empty() {
            self.busy_since = None;
            self.since.clear();
        } else {
            self.busy_since.get_or_insert(now);
            self.since.retain(|key, _| activities.iter().any(|a| &a.key == key));
            for activity in &activities {
                self.since.entry(activity.key.clone()).or_insert(now);
            }
        }
        self.activities = activities;
    }

    /// Whether the indicator is on screen: something has been in flight for
    /// at least [`BUSY_DELAY`].
    pub fn shown(&self) -> bool {
        self.busy_since.is_some_and(|since| self.now - since >= BUSY_DELAY)
    }

    /// Keep the frame loop honest: a repaint every frame while shown (the
    /// spinner turns, the elapsed time ticks), one at the end of the delay
    /// while waiting it out, and NOTHING while idle.
    pub fn request_repaint(&self, ctx: &egui::Context) {
        match self.busy_since {
            None => {}
            Some(_) if self.shown() => ctx.request_repaint(),
            Some(since) => ctx.request_repaint_after(std::time::Duration::from_secs_f64(
                (since + BUSY_DELAY - self.now).max(0.0),
            )),
        }
    }

    /// The Cancel the user clicked this frame, if any — the activity it was
    /// offered for.
    pub fn take_cancel(&mut self) -> Option<Activity> {
        self.cancel.take()
    }

    /// An activity's label as drawn: `…` and, past [`ELAPSED_AFTER`], its
    /// elapsed time.
    fn display_label(&self, activity: &Activity) -> String {
        let elapsed = self.since.get(&activity.key).map_or(0.0, |since| self.now - since);
        if elapsed >= ELAPSED_AFTER {
            format!("{}… {}", activity.label, format_elapsed(elapsed))
        } else {
            format!("{}…", activity.label)
        }
    }

    /// Draw the indicator into a RIGHT-TO-LEFT ui (the status bar's right
    /// end): from the edge inwards Cancel, `+N more`, the label, the spinner —
    /// so it reads spinner, label, `+N more`, Cancel. Draws nothing while not
    /// [`Self::shown`].
    pub fn show(&mut self, ui: &mut egui::Ui) {
        self.hits.clear();
        if !self.shown() {
            return;
        }
        let Some(primary) = self.activities.first().cloned() else { return };
        let tooltip = |ui: &mut egui::Ui, this: &Self| {
            for activity in &this.activities {
                ui.label(this.display_label(activity));
            }
        };
        if primary.cancellable {
            let cancel = ui.small_button("Cancel").on_hover_text(
                "Stop the run. The model keeps the last completed result; \
                 edit or delete the slow feature to rebuild.",
            );
            self.hits.insert("busy:cancel".into(), cancel.rect);
            if cancel.clicked() {
                self.cancel = Some(primary.clone());
            }
        }
        let more = self.activities.len() - 1;
        if more > 0 {
            let chip = ui.weak(format!("+{more} more")).on_hover_ui(|ui| tooltip(ui, self));
            self.hits.insert("busy:more".into(), chip.rect);
        }
        let label = ui
            .add(egui::Label::new(self.display_label(&primary)).truncate())
            .on_hover_ui(|ui| tooltip(ui, self));
        let spinner = ui.add(egui::Spinner::new().size(14.0));
        self.hits.insert("busy".into(), label.rect.union(spinner.rect));
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// `{shown, activities:[{kind, label, document, cancellable, elapsed}]}` —
    /// every activity in flight, shown or still inside the delay, in priority
    /// order. `label` is the bare label (no `…`, no elapsed time), so a
    /// script can pin it exactly.
    pub fn state_json(&self) -> String {
        let activities: Vec<serde_json::Value> = self
            .activities
            .iter()
            .map(|a| {
                serde_json::json!({
                    "kind": a.kind,
                    "label": a.label,
                    "document": a.document,
                    "cancellable": a.cancellable,
                    "elapsed": self.since.get(&a.key).map_or(0.0, |since| self.now - since),
                })
            })
            .collect();
        serde_json::json!({ "shown": self.shown(), "activities": activities }).to_string()
    }
}

/// `4 s`, `59 s`, `1 m 05 s`.
fn format_elapsed(seconds: f64) -> String {
    let whole = seconds.floor() as u64;
    if whole < 60 {
        format!("{whole} s")
    } else {
        format!("{} m {:02} s", whole / 60, whole % 60)
    }
}

pub static HIT_KEYS: &[crate::automation::hit_keys::HitKeyDoc] = &[
    crate::automation::hit_keys::HitKeyDoc { panel: "status", prefix: "busy", meaning: "the working indicator (spinner and label) at the right of the status bar; present only while it shows, and hovering it lists everything in flight", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "status", prefix: "busy:more", meaning: "the `+N more` chip beside the working indicator when several things are in flight; hovering it lists them", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "status", prefix: "busy:cancel", meaning: "cancel the in-flight history run named by the working indicator", command: None },
];

