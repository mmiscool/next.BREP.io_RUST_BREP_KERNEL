//! The Player pane — the Instructions workbench's
//! ([`crate::workbench::instructions`]) in-app player: Back and Next through
//! the steps in reading order, each step shown through its view (the parts
//! earlier steps placed ghosted, the parts it adds solid and moving home
//! along their insertion arrows).
//!
//! Two modes, as the HTML player has:
//!
//! * **shop** — touch-first: large buttons, the step's parts as a checklist
//!   (with a serial field per unit of a serial-tracked part), tools, torque,
//!   callouts and notes, and a Sign off button on a step that needs it once
//!   every part is ticked;
//! * **customer** — pictures first: the step's title, its parts by find
//!   number and quantity, and its callouts; nothing else.
//!
//! The ticks, serials and sign-offs are the pane's, for this session: they
//! are a build's record, not the document's, so they never dirty it or
//! enter its undo. The jobs floor queue keeps the record for a real build.

use crate::automation::hit_keys::HitKeyDoc;
use crate::panels::instructions::PartsDrag;
use brep_instructions::model::{CalloutKind, Instructions, Step};
use brep_instructions::player::Mode;
use brep_render::engine_state::{EngineState, PlayTo};
use std::collections::{BTreeMap, BTreeSet, HashMap};

/// A shop-mode button's height: big enough for a gloved finger.
const TOUCH_HEIGHT: f32 = 44.0;

#[derive(Default)]
pub struct InstructionsPlayerPanel {
    hits: HashMap<String, egui::Rect>,
    /// The PLM client of the open session, for the tools' pictures.
    client: Option<std::rc::Rc<crate::plm::client::PlmClient>>,
    /// The tools' pictures decoded and loaded, by tool id.
    pictures: HashMap<String, egui::TextureHandle>,
    mode: Mode,
    /// Where to go on the next frame drawn (a toolbar button or a command).
    request: Option<PlayTo>,
    /// Ticked parts: step id → indices into the step's `adds`.
    checked: BTreeMap<String, BTreeSet<usize>>,
    /// Captured serials: occurrence path → serial.
    serials: BTreeMap<String, String>,
    /// Steps signed off.
    signed: BTreeSet<String>,
    error: Option<String>,
    /// What the last drop said ("2 parts joined the step").
    notice: Option<String>,
    /// A BOM row (or a not-yet-planned part) dropped on the shown step's
    /// title this frame: the step and the occurrences, applied after drawing.
    drop: Option<(String, Vec<String>)>,
}

impl InstructionsPlayerPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Go somewhere on the next frame the pane draws.
    pub fn request(&mut self, to: PlayTo) {
        self.request = Some(to);
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// What automation reads: the mode, the step shown and how far its parts
    /// have come home (null once they are), and the build's record.
    pub fn state_json(&self, engine: &EngineState) -> String {
        serde_json::json!({
            "mode": self.mode.as_str(),
            "step": engine.instructions_shown_step(),
            "insertion": engine.instructions_insertion_fraction(),
            "pending": self.request.is_some(),
            "checked": self.checked,
            "serials": self.serials,
            "signed": self.signed,
            "error": self.error,
            "notice": self.notice,
        })
        .to_string()
    }

    pub fn show(&mut self, ui: &mut egui::Ui, engine: &mut EngineState, client: Option<&std::rc::Rc<crate::plm::client::PlmClient>>) {
        self.client = client.cloned();
        self.hits.clear();
        self.hits.insert("panel:clip".into(), ui.clip_rect());
        if !engine.is_instructions() {
            ui.label("Open instructions to play them: New from assembly on the toolbar makes some.");
            return;
        }
        let mut go = self.request.take();
        let block = match engine.instructions() {
            Ok(Some(block)) => block,
            Ok(None) => return,
            Err(error) => {
                ui.colored_label(ui.visuals().error_fg_color, format!("The instructions cannot be read: {error}"));
                return;
            }
        };
        let order: Vec<&Step> = block.steps().map(|(_, step)| step).collect();
        let shown = engine.instructions_shown_step();
        let at = shown.as_deref().and_then(|id| order.iter().position(|step| step.id == id));

        ui.horizontal(|ui| {
            for mode in [Mode::Shop, Mode::Customer] {
                let label = if mode == Mode::Shop { "Shop" } else { "Customer" };
                let toggle = ui.selectable_label(self.mode == mode, label);
                self.hits.insert(format!("player:mode:{}", mode.as_str()), toggle.rect);
                if toggle.clicked() {
                    self.mode = mode;
                }
            }
        });
        if let Some(error) = &self.error {
            let label = ui.colored_label(ui.visuals().error_fg_color, error);
            self.hits.insert("player:error".into(), label.rect);
        }
        if order.is_empty() {
            ui.weak("No steps yet: press Wizard on the toolbar.");
            return;
        }
        let height = if self.mode == Mode::Shop { TOUCH_HEIGHT } else { 0.0 };
        ui.horizontal_wrapped(|ui| {
            let back = ui.add_enabled(at.is_some_and(|i| i > 0), egui::Button::new("◀ Back").min_size(egui::vec2(0.0, height)));
            self.hits.insert("player:back".into(), back.rect);
            if back.clicked() {
                go = Some(PlayTo::Back);
            }
            let next_label = if at.is_none() { "Start ▶" } else { "Next ▶" };
            let next = ui.add_enabled(at.is_none_or(|i| i + 1 < order.len()), egui::Button::new(next_label).min_size(egui::vec2(0.0, height)));
            self.hits.insert("player:next".into(), next.rect);
            if next.clicked() {
                go = Some(if at.is_none() { PlayTo::First } else { PlayTo::Next });
            }
            let replay = ui.add_enabled(at.is_some(), egui::Button::new("Replay").min_size(egui::vec2(0.0, height))).on_hover_text("Show this step's parts coming in again");
            self.hits.insert("player:replay".into(), replay.rect);
            if replay.clicked() {
                go = Some(PlayTo::Replay);
            }
            if let Some(i) = at {
                let count = ui.weak(format!("Step {} of {}", i + 1, order.len()));
                self.hits.insert("player:count".into(), count.rect);
            }
        });
        ui.separator();
        match at {
            Some(i) => self.step(ui, &block, order[i], i + 1),
            None => {
                ui.weak("Press Start to play the steps.");
            }
        }
        if let Some((step, occurrences)) = self.drop.take() {
            match engine.instructions_add_parts(&step, &occurrences) {
                Ok(report) => {
                    // The edit regenerated the step's view and put the shown
                    // step away: show it again, with the part in it.
                    let _ = engine.instructions_show_step(&step);
                    self.error = None;
                    self.notice = Some(match report.added {
                        0 => "No part joined the step".to_string(),
                        1 => "1 part joined the step".to_string(),
                        n => format!("{n} parts joined the step"),
                    });
                }
                Err(error) => self.error = Some(error),
            }
        }
        if let Some(to) = go {
            self.error = engine.instructions_play(to).err();
        }
    }

    fn step(&mut self, ui: &mut egui::Ui, block: &Instructions, step: &Step, number: usize) {
        if let Some(parent) = block.parent_id(&step.id).flatten().and_then(|parent| block.step(parent)) {
            ui.weak(&parent.title);
        }
        let title = ui.heading(format!("{number}. {}", step.title));
        self.hits.insert("player:title".into(), title.rect);
        // A part dragged here (a BOM row, a not-yet-planned part) joins the
        // shown step, as a drop on the step's row in the Instructions pane.
        if title.dnd_hover_payload::<PartsDrag>().is_some() {
            ui.painter().rect_stroke(title.rect, 2.0, ui.visuals().selection.stroke, egui::StrokeKind::Outside);
        }
        if egui::DragAndDrop::has_payload_of_type::<PartsDrag>(ui.ctx()) {
            if let Some(parts) = title.dnd_release_payload::<PartsDrag>() {
                self.drop = Some((step.id.clone(), parts.occurrences.clone()));
            }
        }
        if let Some(notice) = &self.notice {
            let label = ui.weak(notice);
            self.hits.insert("player:notice".into(), label.rect);
        }
        match self.mode {
            Mode::Shop => self.shop(ui, step),
            Mode::Customer => self.customer(ui, step),
        }
    }

    fn shop(&mut self, ui: &mut egui::Ui, step: &Step) {
        ui.label(egui::RichText::new("Parts").strong());
        let ticked = self.checked.entry(step.id.clone()).or_default();
        for (index, add) in step.adds.iter().enumerate() {
            let mut on = ticked.contains(&index);
            let tick = ui.checkbox(&mut on, egui::RichText::new(part_text(add)).size(16.0));
            self.hits.insert(format!("player:part:{index}"), tick.rect);
            if tick.changed() {
                if on {
                    ticked.insert(index);
                } else {
                    ticked.remove(&index);
                }
            }
            if add.serial_tracked {
                for (unit, occurrence) in add.occurrences.iter().enumerate() {
                    ui.horizontal(|ui| {
                        ui.label(format!("Serial {}", unit + 1));
                        let serial = self.serials.entry(occurrence.clone()).or_default();
                        let field = ui.add(egui::TextEdit::singleline(serial).min_size(egui::vec2(160.0, 0.0)));
                        self.hits.insert(format!("player:serial:{index}:{unit}"), field.rect);
                    });
                }
            }
        }
        if !step.tools.is_empty() {
            let tools: Vec<String> =
                step.tools.iter().map(|tool| if tool.spec.is_empty() { tool.name.clone() } else { format!("{} ({})", tool.name, tool.spec) }).collect();
            let label = ui.label(format!("Tools: {}", tools.join(", ")));
            self.hits.insert("player:tools".into(), label.rect);
            // The linked tools' pictures from the library (PNG, the type the
            // app decodes), asked for once per session, shown as they land.
            let ids: Vec<String> = step.tools.iter().filter_map(|tool| tool.tool_id.clone()).collect();
            if let Some(client) = &self.client {
                crate::plm::tool_pictures::request(client, &ids);
            }
            ui.horizontal_wrapped(|ui| {
                for id in &ids {
                    let texture = match self.pictures.get(id) {
                        Some(texture) => texture.clone(),
                        None => {
                            let Some(image) = crate::plm::tool_pictures::png_of(id).and_then(|bytes| crate::plm::thumbnail_view::decode(&bytes)) else { continue };
                            let texture = ui.ctx().load_texture(format!("tool-picture-{id}"), image, egui::TextureOptions::LINEAR);
                            self.pictures.insert(id.clone(), texture.clone());
                            texture
                        }
                    };
                    let image = ui.add(egui::Image::new(&texture).fit_to_exact_size(egui::vec2(64.0, 64.0)));
                    self.hits.insert(format!("player:tool:{id}:picture"), image.rect);
                }
            });
        }
        if let Some(torque) = &step.torque {
            let label = ui.label(egui::RichText::new(format!("Torque: {}", torque.text())).strong());
            self.hits.insert("player:torque".into(), label.rect);
        }
        self.callouts(ui, step);
        if !step.notes.is_empty() {
            let notes = ui.label(&step.notes);
            self.hits.insert("player:notes".into(), notes.rect);
        }
        if step.signoff.required {
            ui.add_space(6.0);
            if self.signed.contains(&step.id) {
                let label = ui.label(egui::RichText::new("✔ Signed off").strong());
                self.hits.insert("player:signed".into(), label.rect);
            } else {
                let ticked = self.checked.get(&step.id).map_or(0, BTreeSet::len);
                let serials_in = step
                    .adds
                    .iter()
                    .filter(|add| add.serial_tracked)
                    .flat_map(|add| &add.occurrences)
                    .all(|occurrence| self.serials.get(occurrence).is_some_and(|serial| !serial.trim().is_empty()));
                let ready = ticked == step.adds.len() && serials_in;
                let sign = ui
                    .add_enabled(ready, egui::Button::new("Sign off step").min_size(egui::vec2(0.0, TOUCH_HEIGHT)))
                    .on_disabled_hover_text("Tick every part (and enter its serials) first");
                self.hits.insert("player:signoff".into(), sign.rect);
                if sign.clicked() {
                    self.signed.insert(step.id.clone());
                }
            }
        }
    }

    fn customer(&mut self, ui: &mut egui::Ui, step: &Step) {
        for (index, add) in step.adds.iter().enumerate() {
            let label = ui.label(egui::RichText::new(part_text(add)).size(18.0));
            self.hits.insert(format!("player:part:{index}"), label.rect);
        }
        self.callouts(ui, step);
    }

    fn callouts(&mut self, ui: &mut egui::Ui, step: &Step) {
        for (index, callout) in step.callouts.iter().enumerate() {
            let (kind, colour) = match callout.kind {
                CalloutKind::Caution => ("⚠ Caution", ui.visuals().warn_fg_color),
                CalloutKind::Note => ("ℹ Note", ui.visuals().text_color()),
                CalloutKind::Check => ("✔ Check", ui.visuals().text_color()),
            };
            let label = ui.colored_label(colour, format!("{kind}: {}", callout.text));
            self.hits.insert(format!("player:callout:{index}"), label.rect);
        }
    }
}

/// `[2] widget × 4`: the find number, the part and the quantity.
fn part_text(add: &brep_instructions::model::Add) -> String {
    let mut text = String::new();
    if !add.find_number.is_empty() {
        text.push_str(&format!("[{}] ", add.find_number));
    }
    text.push_str(add.display_name());
    if add.quantity() > 1 {
        text.push_str(&format!(" × {}", add.quantity()));
    }
    text
}

pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "instructionsplayer", prefix: "panel:clip", meaning: "the visible region of the pane (scroll_into_view scrolls a player:* widget inside it)", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:mode:", meaning: "the mode toggle, by mode (shop, customer)", command: Some("instructions_play") },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:error", meaning: "why the last move was refused", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:back", meaning: "show the previous step", command: Some("instructions_play") },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:next", meaning: "show the next step (Start: the first)", command: Some("instructions_play") },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:replay", meaning: "show the step's parts coming in again", command: Some("instructions_play") },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:count", meaning: "where the shown step is: Step N of M", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:title", meaning: "the shown step's number and title; a BOM row or a not-yet-planned part dropped on it joins the shown step (instructions_add_parts), the outcome said under it (player:notice)", command: Some("instructions_add_parts") },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:notice", meaning: "what the last drop on the title said", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:part:", meaning: "a part the step adds, by index: a checklist tick in shop mode, a label in customer mode", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:serial:", meaning: "the serial field of one unit of a serial-tracked part, by part and unit index (shop mode)", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:tools", meaning: "the step's tools (shop mode)", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:tool:", meaning: "a linked tool's picture from the PLM tools library, `<tool id>:picture` (shown once fetched; PNG pictures only in the pane)", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:torque", meaning: "the step's torque (shop mode)", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:callout:", meaning: "a callout of the step, by index", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:notes", meaning: "the step's notes (shop mode)", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:signoff", meaning: "sign the step off, once every part is ticked (shop mode)", command: None },
    HitKeyDoc { panel: "instructionsplayer", prefix: "player:signed", meaning: "the step is signed off (shop mode)", command: None },
];
