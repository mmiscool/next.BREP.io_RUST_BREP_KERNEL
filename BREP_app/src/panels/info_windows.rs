//! Pinned per-entity inspectors with editable metadata and read-only measurements.
//!
//! Each window keeps its target name and edit buffers independently of selection.
//! Opening an existing target keeps its window; closing it discards its buffers.
//! Metadata edits persist through [`EngineState`]. Lengths are millimetres.

use crate::color::rgb_to_hex as hex_string;
use crate::automation::hit_keys::HitKeyDoc;
use brep_render::engine_state::EngineState;
use eframe::egui;
use serde_json::Value;
use std::collections::{BTreeMap, HashMap};

/// Width cap for the attribute-NAME column of the metadata grid, in points.
/// Wide enough for the names the importer writes (`step_name`, `step_colour`)
/// and for a hand-typed one, narrow enough that the value editor and the remove
/// × always fit beside it in the window's default 300 pt width.
const KEY_COL_WIDTH: f32 = 120.0;

/// The two tabs of an Info window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tab {
    /// The editable name-keyed attribute editor.
    Metadata,
    /// The read-only measurements + provenance.
    Info,
}

/// ONE pinned per-entity inspector window. Its `target` is fixed at open time and
/// drives every engine query — later selection changes CANNOT retarget it (that is
/// the whole point). Owns its own tab + edit buffers so windows never share state.
struct PinnedInfoWindow {
    /// The object name this window is PINNED to (a solid / face / edge). Fixed at
    /// construction; never reassigned. Also the window title.
    target: String,
    /// Whether the window is still shown. Its own `×` clears this; the manager then
    /// prunes it. Never re-opened — a closed window is dropped, a re-request makes a
    /// fresh one.
    open: bool,
    /// The active tab.
    tab: Tab,
    /// Live edit buffers for the target's existing attributes (`key → value`),
    /// adopted from the engine store and kept in lock-step as the user types.
    values: BTreeMap<String, String>,
    /// The target's store record as it stood when [`Self::values`] was last in
    /// step with it — the middle term of the three-way compare in
    /// [`Self::adopt_outside_edits`]. Empty until the first draw, which is what
    /// makes that first draw the SEEDING one and keeps `open_for` engine-free.
    base: BTreeMap<String, String>,
    /// The "add attribute" row buffers (attribute name + value).
    new_key: String,
    new_value: String,
    /// Working colour for the `Set color` row — the one-click way to give an
    /// object that has no `color` attribute yet. A visible orange so the first
    /// click produces an obviously-applied colour rather than something that
    /// might be the default shade.
    new_color: [u8; 3],
    /// The window's default open position (cascaded per open-order so a multi-select
    /// open doesn't stack every window on the exact same spot).
    default_pos: [f32; 2],
    /// Per-frame interactive-widget screen rects (egui points), keyed with this
    /// window's target so the manager can publish them un-ambiguously for the headed
    /// verifier.
    hits: HashMap<String, egui::Rect>,
}

impl PinnedInfoWindow {
    fn new(target: impl Into<String>, default_pos: [f32; 2]) -> Self {
        Self {
            target: target.into(),
            open: true,
            tab: Tab::Info,
            values: BTreeMap::new(),
            base: BTreeMap::new(),
            new_key: String::new(),
            new_value: String::new(),
            new_color: [0xff, 0x88, 0x00],
            default_pos,
            hits: HashMap::new(),
        }
    }

    /// Bring the metadata edit buffers back in step with the store — on the
    /// first draw (when they are empty, so this seeds them) and on every draw
    /// after it.
    ///
    /// They used to be seeded ONCE and never re-read. The store has other
    /// writers: `automation::cmd_metadata`'s `metadata_set`,
    /// `metadata_set_many` and `metadata_remove` drive the SAME seam this
    /// window drives — "a colour an agent writes and a colour a person picks
    /// are the same edit", as that module's header puts it. A changed
    /// attribute therefore read its old value in an open window for the rest
    /// of the session, and one ADDED from outside never appeared at all, since
    /// the grid iterates the buffers rather than the record.
    ///
    /// A THREE-WAY compare, not a re-seed. [`Self::base`] is the record as it
    /// stood when the buffers were last in step with it, so:
    ///
    /// * a key whose store value still equals `base` did not move outside, and
    ///   its buffer is left exactly as the user left it;
    /// * a key that DID move outside is adopted only when its buffer still
    ///   reads `base` — a buffer the store has not confirmed is the user's own
    ///   unwritten edit and outranks a re-read;
    /// * a key `base` never carried is the user's too, and a removal elsewhere
    ///   does not sweep it up.
    ///
    /// The residual: an outside write to a field the user has focused, and has
    /// not typed into since it was last written through, replaces its text.
    /// That is a genuine conflict — somebody changed the value — and showing
    /// the new truth is the better half of it.
    ///
    /// Cost is one [`MetadataStore::record`] clone per open window per frame
    /// on the Metadata tab: a `BTreeMap` of a handful of short strings, the
    /// same read `published_json` already makes every frame in the browser
    /// build. It replaces a JSON round trip that ran once.
    fn adopt_outside_edits(&mut self, state: &EngineState) {
        let store = state.metadata.record(&self.target);
        if store == self.base {
            return;
        }
        for (key, value) in &store {
            if self.base.get(key) == Some(value) {
                continue; // did not move outside
            }
            let unwritten = self
                .values
                .get(key)
                .is_some_and(|shown| self.base.get(key) != Some(shown));
            if !unwritten {
                self.values.insert(key.clone(), value.clone());
            }
        }
        let base = &self.base;
        self.values
            .retain(|key, shown| store.contains_key(key) || base.get(key) != Some(&*shown));
        self.base = store;
    }

    /// Draw the window (if open) at ctx level. Folds the window's own close (`×`)
    /// back into `self.open` so the manager can prune it.
    fn show(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        self.hits.clear();
        if !self.open {
            return;
        }
        // `egui::Window::open` needs its own `&mut bool`; borrow a copy so the draw
        // closure can still take `&mut self`, then fold the close back in. The title
        // is the entity name; the manager's dedup guarantees it is unique, so the
        // derived egui window id never collides.
        let mut open = true;
        egui::Window::new(&self.target)
            .id(egui::Id::new(("brep-info-window", self.target.as_str())))
            .open(&mut open)
            .movable(true)
            .resizable(true)
            // Bounded default size + a fill ScrollArea (below) so the window is
            // FREELY resizable LARGER than its content (egui otherwise hugs the
            // window to content and refuses to grow).
            .default_size([300.0, 360.0])
            .default_pos(self.default_pos)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| self.body(ui, state));
            });
        self.open = open;
    }

    /// The window body: the tab strip, then the active tab. There is always a
    /// target (the window is opened FOR one), so no "select an object" hint.
    fn body(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        ui.horizontal(|ui| {
            let meta = ui.selectable_value(&mut self.tab, Tab::Metadata, "Metadata");
            let info = ui.selectable_value(&mut self.tab, Tab::Info, "Info");
            self.hits
                .insert(format!("{}:tab:metadata", self.target), meta.rect);
            self.hits.insert(format!("{}:tab:info", self.target), info.rect);
        });
        ui.separator();

        let name = self.target.clone();
        match self.tab {
            Tab::Metadata => self.metadata_tab(ui, state, &name),
            Tab::Info => info_tab(ui, state, &name),
        }
    }

    /// Tab 1 — the editable, name-keyed attribute editor. Each existing attribute is
    /// `key + editable value + ×`; the add row appends a new attribute. Every
    /// mutation writes straight through to the engine store (which persists with the
    /// model), keeping the local buffers in lock-step.
    fn metadata_tab(&mut self, ui: &mut egui::Ui, state: &mut EngineState, name: &str) {
        // Here rather than in `show`: this is the only tab the buffers are
        // drawn on, so the Info tab pays nothing for them.
        self.adopt_outside_edits(state);
        ui.label("Attributes (name-keyed; survive edits, rollback and re-tessellation):");
        ui.add_space(2.0);

        let keys: Vec<String> = self.values.keys().cloned().collect();
        let mut remove: Option<String> = None;
        egui::Grid::new(("info-metadata-grid", name))
            .num_columns(3)
            .striped(true)
            .show(ui, |ui| {
                for key in &keys {
                    // A `#RRGGBB` value gets a live colour PICKER beside its
                    // name — the form an imported STEP colour arrives in (kernel
                    // `io/appearance.rs`), and now the form a user can edit by
                    // eye instead of typing hex. Keyed on the VALUE, not the
                    // attribute name, so any colour-valued attribute is
                    // editable the same way; `color` is simply the one the
                    // renderer reads.
                    let swatch = self.values.get(key).and_then(|value| hex_color(value));
                    let mut picked: Option<String> = None;
                    let hits = &mut self.hits;
                    ui.horizontal(|ui| {
                        if let Some(color) = swatch {
                            let mut rgb = [color.r(), color.g(), color.b()];
                            let resp = ui.color_edit_button_srgb(&mut rgb);
                            if resp.changed() {
                                picked = Some(hex_string(rgb));
                            }
                            hits.insert(format!("{name}:swatch:{key}"), resp.rect);
                        }
                        // The key is CAPPED and truncates inside its cap. A
                        // grid column sizes to its widest cell, so an attribute
                        // name — user-typed, or carried in from a STEP import —
                        // otherwise widens this column and pushes the value
                        // editor and the remove × past the window's right edge,
                        // where the body's vertical-only ScrollArea clips them
                        // out of reach rather than scrolling to them. An elided
                        // Label tooltips its own full text, so the whole key is
                        // still one hover away.
                        ui.scope(|ui| {
                            ui.set_max_width(KEY_COL_WIDTH);
                            ui.add(egui::Label::new(key).truncate());
                        });
                    });
                    // Commit a picked colour through the SAME engine seam a typed
                    // value uses, so the viewport updates on the drag.
                    if let Some(hex) = picked {
                        state.set_metadata_attribute(name, key, &hex);
                        if let Some(slot) = self.values.get_mut(key) {
                            *slot = hex;
                        }
                    }
                    if let Some(value) = self.values.get_mut(key) {
                        let edit =
                            ui.add(egui::TextEdit::singleline(value).desired_width(140.0));
                        if edit.changed() {
                            state.set_metadata_attribute(name, key, value);
                        }
                        self.hits.insert(format!("{name}:value:{key}"), edit.rect);
                    }
                    let del = ui.button("\u{00d7}").on_hover_text("Remove attribute");
                    self.hits.insert(format!("{name}:remove:{key}"), del.rect);
                    if del.clicked() {
                        remove = Some(key.clone());
                    }
                    ui.end_row();
                }
            });
        if let Some(key) = remove {
            state.remove_metadata_attribute(name, &key);
            self.values.remove(&key);
        }

        if keys.is_empty() {
            ui.weak("(no attributes yet)");
        }

        // An object with no colour yet gets a one-click way to have one —
        // picking a model colour should not require knowing the `color` key or
        // hex syntax. Once set, the picker in the grid above edits it.
        if !self.values.contains_key(COLOR_KEY) {
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                let picker = ui.color_edit_button_srgb(&mut self.new_color);
                let set = ui
                    .button("Set color")
                    .on_hover_text("Give this object a `color` attribute (saved with the model)");
                self.hits.insert(format!("{name}:new-color:pick"), picker.rect);
                self.hits.insert(format!("{name}:new-color:set"), set.rect);
                if set.clicked() {
                    let hex = hex_string(self.new_color);
                    state.set_metadata_attribute(name, COLOR_KEY, &hex);
                    self.values.insert(COLOR_KEY.to_string(), hex);
                }
            });
        }

        ui.add_space(6.0);
        ui.separator();
        ui.label("Add attribute:");
        ui.horizontal(|ui| {
            let key = ui.add(
                egui::TextEdit::singleline(&mut self.new_key)
                    .hint_text("name")
                    .desired_width(110.0),
            );
            let val = ui.add(
                egui::TextEdit::singleline(&mut self.new_value)
                    .hint_text("value")
                    .desired_width(140.0),
            );
            let add = ui.button("Add");
            self.hits.insert(format!("{name}:new:key"), key.rect);
            self.hits.insert(format!("{name}:new:value"), val.rect);
            self.hits.insert(format!("{name}:new:add"), add.rect);
            let commit = add.clicked()
                || (val.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)));
            let trimmed = self.new_key.trim().to_string();
            if commit && !trimmed.is_empty() {
                state.set_metadata_attribute(name, &trimmed, &self.new_value);
                self.values.insert(trimmed, self.new_value.clone());
                self.new_key.clear();
                self.new_value.clear();
            }
        });

        ui.add_space(6.0);
        ui.weak(
            "Hint: `density` (mass per mm\u{00b3}) drives a solid's weight; `color` \
             (#RRGGBB) drives how it is shaded.",
        );
    }
}

/// The metadata attribute the renderer shades from — the kernel's own key, so
/// the Info window and a STEP import can never disagree about its spelling.
const COLOR_KEY: &str = brep_render::brep_kernel::COLOR_METADATA_KEY;

fn hex_color(value: &str) -> Option<egui::Color32> {
    crate::color::parse_hex_color(value.trim())
}

/// The shell-owned manager of the pinned Info windows: opens them (from the context
/// bar's Info action), draws them each frame, and prunes the ones the user closes.
#[derive(Default)]
pub struct InfoWindows {
    /// The open windows, in open order. Independent state per window.
    windows: Vec<PinnedInfoWindow>,
    /// Monotonic count of windows ever opened this session — used only to cascade
    /// each new window's default position so multi-select opens don't stack exactly.
    opened_count: usize,
}

impl InfoWindows {
    pub fn new() -> Self {
        Self::default()
    }

    /// Open one pinned window per name (a viewport multi-select yields N names → N
    /// windows). DEDUP: if a window is already open for a name, keep it — don't
    /// duplicate. Empty names are skipped. Engine-free (buffers seed lazily on the
    /// first draw) so it needs no `EngineState`.
    pub fn open_for(&mut self, names: &[String], viewport: Option<egui::Rect>) {
        for name in names {
            if name.is_empty() {
                continue;
            }
            // Already showing this exact entity → keep the existing window.
            if self.windows.iter().any(|w| w.open && w.target == *name) {
                continue;
            }
            // Cascade the default position off the open-order so a multi-select open
            // fans the windows out instead of stacking them on one spot.
            let k = (self.opened_count % 8) as f32;
            self.opened_count += 1;
            // Open from the viewport's LEFT edge, not its right. The ONLY way to
            // reach this window is the selection action bar, which floats over the
            // viewport's top-RIGHT corner in a Foreground area — so a window opened
            // there is drawn UNDER it and every widget the bar covers is unclickable
            // (the layer below never sees the press). That is where the
            // Add-attribute row landed: its `Add` button sat squarely behind the
            // bar, and clicking it by its published rect did nothing at all.
            let origin = viewport.map_or(egui::pos2(440.0, 56.0), |r| r.left_top() + egui::vec2(20.0, 8.0));
            let pos = [origin.x + 24.0 * k, origin.y + 24.0 * k];
            self.windows.push(PinnedInfoWindow::new(name.clone(), pos));
        }
    }

    /// Draw every open window at ctx level (like the file dialog / settings window),
    /// then drop the ones the user closed. Independent windows → independent state.
    pub fn show(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        for w in &mut self.windows {
            w.show(ctx, state);
        }
        self.prune();
    }

    /// Drop windows the user closed via their `×`. Called by `show` after drawing;
    /// exposed for the manager unit tests.
    fn prune(&mut self) {
        self.windows.retain(|w| w.open);
    }

    /// The FIXED target names of the currently-open windows, in open order — the
    /// unit-test seam proving pin-independence, dedup and prune with NO engine.
    #[cfg_attr(target_arch = "wasm32", allow(dead_code))]
    pub fn targets(&self) -> Vec<String> {
        self.windows
            .iter()
            .filter(|w| w.open)
            .map(|w| w.target.clone())
            .collect()
    }

    /// Composite state of every open window for the headed verifier — each record is
    /// `{ target, tab, metadata{…}, info{…} }`, all keyed by the window's FIXED
    /// target so the verifier can assert a window keeps its entity across selection
    /// changes. (wasm only; present-but-dead on native so `tab`/`target` count as
    /// read there too.)
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn published_json(&self, state: &mut EngineState) -> String {
        let windows: Vec<Value> = self
            .windows
            .iter()
            .filter(|w| w.open)
            .map(|w| {
                serde_json::json!({
                    "target": w.target,
                    "tab": match w.tab { Tab::Metadata => "metadata", Tab::Info => "info" },
                    "metadata": parse(&state.object_metadata_json(&w.target)),
                    "info": parse(&state.object_info_json(&w.target)),
                })
            })
            .collect();
        serde_json::json!({ "count": windows.len(), "windows": windows }).to_string()
    }

    /// The union of every open window's per-frame interactive-widget screen rects
    /// (egui points) as `{ key: [x, y, w, h] }`, each key prefixed with the window's
    /// target (`<name>:tab:info`, `<name>:value:<k>`, `<name>:new:add`, …).
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(self.windows.iter().flat_map(|window| &window.hits))
    }
}

/// Tab 2 — the READ-ONLY info: name / creating feature, then the kind-specific
/// measurements. NOTHING here is editable.
fn info_tab(ui: &mut egui::Ui, state: &mut EngineState, name: &str) {
    let info = parse(&state.object_info_json(name));
    if info.get("ok").and_then(Value::as_bool) != Some(true) {
        ui.label(
            info.get("message")
                .and_then(Value::as_str)
                .unwrap_or("no info for this object"),
        );
        return;
    }
    let kind = info.get("kind").and_then(Value::as_str).unwrap_or("");

    egui::Grid::new(("info-info-grid", name))
        .num_columns(2)
        .striped(true)
        .show(ui, |ui| {
            grid_row(ui, "Name / ID", name.to_string());
            grid_row(ui, "Kind", kind.to_string());
            grid_row(ui, "Creating feature", creating_feature_str(&info));
            match kind {
                "solid" => {
                    grid_row(ui, "Volume (mm\u{00b3})", num(getf(&info, "volume")));
                    grid_row(ui, "Surface area (mm\u{00b2})", num(getf(&info, "surfaceArea")));
                    grid_row(ui, "Edge length total (mm)", num(getf(&info, "edgeLengthTotal")));
                    grid_row(ui, "Density (mass/mm\u{00b3})", num(getf(&info, "density")));
                    grid_row(ui, "Weight (mass)", num(getf(&info, "weight")));
                }
                "face" => {
                    grid_row(ui, "Solid", getstr(&info, "solid"));
                    grid_row(ui, "Surface type", getstr(&info, "surfaceType"));
                    grid_row(ui, "Area (mm\u{00b2})", num(getf(&info, "area")));
                    grid_row(ui, "Edge length total (mm)", num(getf(&info, "edgeLengthTotal")));
                }
                "edge" => {
                    grid_row(ui, "Solid", getstr(&info, "solid"));
                    grid_row(ui, "Length (mm)", num(getf(&info, "length")));
                }
                _ => {}
            }
        });

    ui.add_space(6.0);
    ui.weak("Read-only. Set `density` on the Metadata tab to drive a solid's weight.");
}

/// The `creatingFeature` provenance as `id (type)`, or `—` when the object has no
/// known producer (a `null` provenance).
fn creating_feature_str(info: &Value) -> String {
    match info.get("creatingFeature") {
        Some(Value::Object(feature)) => {
            let id = feature.get("id").and_then(Value::as_str).unwrap_or("");
            let kind = feature.get("type").and_then(Value::as_str).unwrap_or("");
            if kind.is_empty() {
                id.to_string()
            } else {
                format!("{id} ({kind})")
            }
        }
        _ => "\u{2014}".to_string(),
    }
}

/// Parse an engine JSON string, defaulting to `null` on any error.
fn parse(json: &str) -> Value {
    serde_json::from_str(json).unwrap_or(Value::Null)
}

/// `label: value` row inside an `egui::Grid`.
fn grid_row(ui: &mut egui::Ui, label: &str, value: String) {
    ui.label(label);
    ui.label(value);
    ui.end_row();
}

/// A JSON number field (`0` for a missing / null field).
fn getf(value: &Value, key: &str) -> f64 {
    value.get(key).and_then(Value::as_f64).unwrap_or(0.0)
}

/// A JSON string field (empty for a missing / null field).
fn getstr(value: &Value, key: &str) -> String {
    value.get(key).and_then(Value::as_str).unwrap_or("").to_string()
}

/// Format a number readably (fixed 3 decimals, trailing-zero trimmed); `0` stays
/// `0`. `pub(crate)`: the interference window's volume labels reuse THIS
/// formatter (UI-consistency directive — one measurement format).
pub(crate) fn num(x: f64) -> String {
    if x == 0.0 {
        return "0".to_string();
    }
    let mut s = format!("{x:.3}");
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    s
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    // `:tab:` and not `:tab:info` — the window publishes BOTH tabs and only the
    // info one was documented, so the first script to open an Info window read
    // `{name}:tab:metadata` back as an undocumented key.
    HitKeyDoc { panel: "infowindows", prefix: ":tab:", meaning: "a window's tab ({name}:tab:info for mass properties and topology, {name}:tab:metadata for the key/value editor)", command: None },
    HitKeyDoc { panel: "infowindows", prefix: ":new:", meaning: "add a metadata key/value ({name}:new:key, :new:value, :new:add)", command: Some("metadata_set") },
    HitKeyDoc { panel: "infowindows", prefix: ":new-color:", meaning: "pick or set a colour ({name}:new-color:pick, :set)", command: Some("metadata_set") },
    HitKeyDoc { panel: "infowindows", prefix: ":value:", meaning: "edit a metadata value ({name}:value:key)", command: Some("metadata_set") },
    HitKeyDoc { panel: "infowindows", prefix: ":remove:", meaning: "remove a metadata key ({name}:remove:key)", command: Some("metadata_remove") },
    HitKeyDoc { panel: "infowindows", prefix: ":swatch:", meaning: "a colour swatch ({name}:swatch:key)", command: None },
];
