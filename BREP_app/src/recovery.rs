//! Autosave unsaved documents under [`RECOVERY_KEY`] for recovery at next boot.
//!
//! Edits debounce writes by [`AUTOSAVE_DEBOUNCE`]. Saves, discards, and undo back
//! to a clean state remove entries immediately, preventing recovery of work the
//! user already saved or discarded during the debounce window.
//!
//! Recovery requires an explicit choice: automatically loading a failing model
//! could break every startup. The prompt remains until restore or discard so
//! edits to the initial seed document cannot overwrite the recovery offer.
//! Restored documents remain dirty and are autosaved again.
//!
//! Change detection uses document ID, [`Document::edit_key`] (the applied-run
//! generation and the history revision, so a block edit that runs no history
//! still re-arms the write), and dirty marker; metadata-only edits follow
//! [`Document::refresh_dirty_marker`] timing. Documents are serialized only when
//! a write is due.

use crate::automation::hit_keys::HitKeyDoc;
use std::collections::HashMap;

use eframe::egui;

use crate::document::{Document, Documents};
use crate::store::{ModelStore, RECOVERY_KEY};

/// Seconds a dirty set must hold still before its documents are written.
pub const AUTOSAVE_DEBOUNCE: f64 = 2.0;

/// The blob's schema number; a blob from a different one is ignored.
const SCHEMA: u64 = 1;

/// One autosaved document as it sits in the blob.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RecoveryEntry {
    /// The store identity the document had (`None` = untitled) — a copy of
    /// `Document::name`, one of the identity sites enumerated in
    /// `crate::store` (a path on the file stores, `part/…/rev/…` on a PLM
    /// store). The blob itself is [`KeySpace::Local`](crate::store::KeySpace):
    /// written on this machine first, mirrored to the server by a PLM session.
    #[serde(default)]
    pub name: Option<String>,
    /// The tab title at the time of the write.
    pub title: String,
    /// Wall-clock Unix seconds of the write (0 when unknown).
    #[serde(default)]
    pub saved_at: f64,
    /// The model request JSON — the same `.nbrep` text a Save writes.
    pub model: String,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct Blob {
    schema: u64,
    documents: Vec<RecoveryEntry>,
}

/// The per-frame autosave driver the shell owns.
#[derive(Default)]
pub struct Autosave {
    /// `(document id, edit key, dirty marker)` per open tab, as of the last
    /// tick — the change detector.
    captured: Vec<(u64, (u64, u64), bool)>,
    /// The egui clock time the current debounce window was (re)armed at.
    armed_at: Option<f64>,
    /// What the blob holds — `(name, model)` per entry — as far as this session
    /// knows: `None` at boot (the prompt owns the stored blob until it resolves),
    /// `Some(empty)` once it is known to be absent. Compared before every write
    /// so an unchanged dirty set never rewrites megabytes.
    written: Option<Vec<(Option<String>, String)>>,
    /// Store failures raised by a write, drained by the shell into the toasts.
    errors: Vec<String>,
}

impl Autosave {
    pub fn new() -> Self {
        Self::default()
    }

    /// Advance the autosave by one frame. `now` is the egui clock
    /// (`ctx.input().time`, seconds; never `Instant`, which traps on wasm).
    /// Returns the seconds until an armed write comes due, so the shell can ask
    /// for a repaint then — the frame loop is otherwise idle and the deadline
    /// would never be observed.
    pub fn tick(&mut self, docs: &Documents, store: &dyn ModelStore, now: f64) -> Option<f64> {
        let snapshot: Vec<(u64, (u64, u64), bool)> = docs
            .iter()
            .map(|doc| (doc.id(), doc.edit_key(), doc.dirty_marker()))
            .collect();
        if snapshot != self.captured {
            let dirty_before: Vec<u64> = dirty_ids(&self.captured);
            let dirty_now: Vec<u64> = dirty_ids(&snapshot);
            let shrank = dirty_before.iter().any(|id| !dirty_now.contains(id));
            self.captured = snapshot;
            if shrank {
                // A save, a discarded close, an undo to clean: whatever left the
                // set must stop being offered NOW. The write captures every
                // still-dirty document's current state too, so nothing armed
                // is left over.
                self.write(docs, store);
                self.armed_at = None;
            } else if !dirty_now.is_empty() {
                self.armed_at = Some(now);
            }
        }
        if let Some(armed) = self.armed_at {
            let remaining = AUTOSAVE_DEBOUNCE - (now - armed);
            if remaining > 0.0 {
                return Some(remaining);
            }
            self.write(docs, store);
            self.armed_at = None;
        }
        None
    }

    /// The prompt resolved (restore or discard removed the blob): the store is
    /// known empty, so the next dirty document is a fresh write.
    pub fn note_cleared(&mut self) {
        self.written = Some(Vec::new());
    }

    /// Store failures since the last drain, oldest first.
    pub fn take_errors(&mut self) -> Vec<String> {
        std::mem::take(&mut self.errors)
    }

    /// Write the dirty documents (or remove the blob when there are none),
    /// unless the store already holds exactly that.
    fn write(&mut self, docs: &Documents, store: &dyn ModelStore) {
        let entries: Vec<RecoveryEntry> = docs
            .iter()
            .filter(|doc| doc.dirty_marker())
            .map(|doc| RecoveryEntry {
                name: doc.name().map(str::to_string),
                title: doc.title(),
                saved_at: wall_clock(),
                model: doc.engine.history_request_json(),
            })
            .collect();
        let key: Vec<(Option<String>, String)> = entries
            .iter()
            .map(|entry| (entry.name.clone(), entry.model.clone()))
            .collect();
        if self.written.as_ref() == Some(&key) {
            return;
        }
        let result = if entries.is_empty() {
            store.remove(RECOVERY_KEY)
        } else {
            store.write(RECOVERY_KEY, &encode(&entries))
        };
        match result {
            Ok(()) => self.written = Some(key),
            Err(error) => self.errors.push(format!("autosave failed: {error}")),
        }
    }
}

fn dirty_ids(snapshot: &[(u64, (u64, u64), bool)]) -> Vec<u64> {
    snapshot
        .iter()
        .filter(|(_, _, dirty)| *dirty)
        .map(|(id, _, _)| *id)
        .collect()
}

fn encode(entries: &[RecoveryEntry]) -> String {
    serde_json::to_string(&Blob {
        schema: SCHEMA,
        documents: entries.to_vec(),
    })
    .expect("serialize recovery blob")
}

/// The autosaved documents worth offering: the blob's entries minus any whose
/// named store copy already holds the same model (a Save that landed before
/// the shrink write did — content is compared as parsed JSON so formatting
/// differences never manufacture a prompt). Empty when there is no blob, or it
/// is from another schema.
pub fn read_entries(store: &dyn ModelStore) -> Vec<RecoveryEntry> {
    let Some(text) = store.read(RECOVERY_KEY) else {
        return Vec::new();
    };
    let Ok(blob) = serde_json::from_str::<Blob>(&text) else {
        return Vec::new();
    };
    if blob.schema != SCHEMA {
        return Vec::new();
    }
    blob.documents
        .into_iter()
        .filter(|entry| !entry.model.is_empty())
        .filter(|entry| !already_saved(store, entry))
        .collect()
}

fn already_saved(store: &dyn ModelStore, entry: &RecoveryEntry) -> bool {
    let Some(name) = &entry.name else {
        return false;
    };
    let Some(saved) = store.read(name) else {
        return false;
    };
    match (
        serde_json::from_str::<serde_json::Value>(&saved),
        serde_json::from_str::<serde_json::Value>(&entry.model),
    ) {
        (Ok(saved), Ok(model)) => saved == model,
        _ => false,
    }
}

/// Open every entry as a tab of its own (dirty, under the name it had). An
/// entry the engine cannot load is reported, not restored. When the session is
/// still the pristine boot seed — one clean, untitled tab — that tab is
/// replaced rather than kept beside the restored work, for the same reason the
/// `?loadModel=` boot path replaces it: the user asked for THEIR document, and
/// a demo cube next to it is noise. Returns the restored count and the failures.
pub fn restore_entries(entries: &[RecoveryEntry], docs: &mut Documents) -> (usize, Vec<String>) {
    let pristine_seed =
        docs.len() == 1 && docs.active().name().is_none() && !docs.active().is_dirty();
    let mut restored = 0usize;
    let mut failures = Vec::new();
    for entry in entries {
        let mut engine = docs.spawn_engine();
        if let Err(error) = engine.load_model_and_fit(&entry.model) {
            failures.push(format!("could not restore \"{}\": {error}", entry.title));
            continue;
        }
        docs.open_document(Document::recovered(engine, entry.name.clone()));
        restored += 1;
    }
    if restored > 0 && pristine_seed {
        docs.close(0);
    }
    (restored, failures)
}

/// Wall-clock Unix seconds. `SystemTime` traps on wasm32; the browser's clock
/// is `Date.now()`.
pub fn wall_clock() -> f64 {
    #[cfg(target_arch = "wasm32")]
    {
        js_sys::Date::now() / 1000.0
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0)
    }
}

/// "just now" / "3 min ago" / "2 h ago" / "4 d ago" for a `saved_at` stamp,
/// or empty when the stamp is unknown.
fn age_label(saved_at: f64, now: f64) -> String {
    if saved_at <= 0.0 {
        return String::new();
    }
    let seconds = (now - saved_at).max(0.0);
    if seconds < 60.0 {
        "just now".to_string()
    } else if seconds < 3600.0 {
        format!("{} min ago", (seconds / 60.0).floor() as u64)
    } else if seconds < 86_400.0 {
        format!("{} h ago", (seconds / 3600.0).floor() as u64)
    } else {
        format!("{} d ago", (seconds / 86_400.0).floor() as u64)
    }
}

/// How the prompt resolved this frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Resolution {
    /// `Restore`: this many tabs were opened (the failures were toasted).
    Restored(usize),
    /// `Discard`: the blob is gone.
    Discarded,
}

/// The boot-time **Recover unsaved work?** modal. Armed with the entries
/// [`read_entries`] found; open until one of its two buttons resolves it.
#[derive(Default)]
pub struct RecoveryPanel {
    entries: Vec<RecoveryEntry>,
    hits: HashMap<String, egui::Rect>,
}

impl RecoveryPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Offer `entries` (nothing is shown for an empty list).
    pub fn arm(&mut self, entries: Vec<RecoveryEntry>) {
        self.entries = entries;
    }

    pub fn is_open(&self) -> bool {
        !self.entries.is_empty()
    }

    /// Forget the last frame's button rects (the shell calls this when the
    /// prompt is closed, so a verifier never reads a rect of a modal that is
    /// no longer there).
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// Draw the modal while entries are pending. Returns the resolution on the
    /// frame a button lands; `docs` gains the restored tabs and `store` loses
    /// the blob on either button. Failures to load an entry are queued on the
    /// active engine as notices.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        docs: &mut Documents,
        store: &dyn ModelStore,
    ) -> Option<Resolution> {
        self.hits.clear();
        if self.entries.is_empty() {
            return None;
        }
        let now = wall_clock();
        let count = self.entries.len();
        let mut restore = false;
        let mut discard = false;
        egui::Modal::new(egui::Id::new("brep-recovery")).show(ctx, |ui| {
            ui.set_width(440.0);
            ui.heading("Recover unsaved work?");
            ui.add_space(4.0);
            ui.label(format!(
                "The last session ended with {count} document{} unsaved. \
                 Autosaved copies are still here.",
                if count == 1 { "" } else { "s" }
            ));
            ui.add_space(6.0);
            // Bounded, so a long list never pushes Restore / Discard off-screen.
            let list_height = (ctx.content_rect().height() * 0.5).max(120.0);
            egui::ScrollArea::vertical()
                .id_salt("brep-recovery-entries")
                .max_height(list_height)
                .show(ui, |ui| {
                    for entry in &self.entries {
                        let age = age_label(entry.saved_at, now);
                        let line = if age.is_empty() {
                            format!("•  {}", entry.title)
                        } else {
                            format!("•  {}  ({age})", entry.title)
                        };
                        ui.label(line);
                    }
                });
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let r = ui.button("Restore");
                self.hits.insert("recovery:restore".into(), r.rect);
                if r.clicked() {
                    restore = true;
                }
                let d = ui.button("Discard");
                self.hits.insert("recovery:discard".into(), d.rect);
                if d.clicked() {
                    discard = true;
                }
            });
            ui.add_space(2.0);
            ui.weak("Restored documents open as new tabs with unsaved changes.");
        });
        // Esc / click-outside deliberately do NOT dismiss (see the module doc):
        // the offer survives until a button decides.
        if restore {
            let entries = std::mem::take(&mut self.entries);
            let (restored, failures) = restore_entries(&entries, docs);
            for failure in failures {
                docs.engine_mut().push_notice(failure);
            }
            let _ = store.remove(RECOVERY_KEY);
            Some(Resolution::Restored(restored))
        } else if discard {
            self.entries.clear();
            let _ = store.remove(RECOVERY_KEY);
            Some(Resolution::Discarded)
        } else {
            None
        }
    }

    /// `{open, entries:[{title, name, savedAt}]}` — the `__brepRecovery` global.
    pub fn state_json(&self) -> String {
        let entries: Vec<serde_json::Value> = self
            .entries
            .iter()
            .map(|entry| {
                serde_json::json!({
                    "title": entry.title,
                    "name": entry.name,
                    "savedAt": entry.saved_at,
                })
            })
            .collect();
        serde_json::json!({ "open": self.is_open(), "entries": entries }).to_string()
    }

    /// The modal's button rects (`recovery:restore` / `recovery:discard`) as
    /// `{key: [x, y, w, h]}`, empty while closed.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "recovery", prefix: "recovery:restore", meaning: "restore the recovered documents", command: None },
    HitKeyDoc { panel: "recovery", prefix: "recovery:discard", meaning: "discard the recovery blob", command: None },
];
