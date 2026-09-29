//! The family table editor on a PLM store (plm-cad-integration-todo S9): what
//! the table pane adds when the family is a PLM revision.
//!
//! - **Generate** goes to the server ([`crate::plm::family::generate`]). The
//!   server generates from the family's SAVED revision, so a family with unsaved
//!   edits is refused with the reason and a **Save and generate** button: the
//!   shell saves, and Generate starts once the save has reached the server.
//! - **The member part type**: which part type new members are created in, shown
//!   with its numbering mode and settable (`member_part_type`).
//! - **The category's keys**: the family part's category schema, offered as
//!   columns the table does not have yet (family parameters are a subset of the
//!   category's attributes).
//! - **Cells the server will refuse**: a column that is a catalog attribute is
//!   type-checked by the server, so a cell holding an expression where a number
//!   is wanted fails its row. They are listed before Generate is pressed.
//!
//! Everything here talks to the server through futures the pane polls once per
//! frame, as the lifecycle panel does, so the web build works the same. With no
//! PLM none of it exists: [`PlmFamily::sync`] is given no client, and every
//! method is a no-op.

use std::rc::Rc;
use std::task::{Context, Poll, Waker};

use eframe::egui;

use crate::family_table::{FamilyTable, GenerateReport};
use crate::plm::client::PlmClient;
use crate::plm::family::{self, Attribute};
use crate::plm::PlmFuture;

/// What the pane knows about the family on the server.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Info {
    pub number: String,
    pub member_part_type: String,
    pub member_part_type_mode: String,
    /// The family part's category, and its schema (empty when uncategorized).
    pub category: String,
    pub schema: Vec<Attribute>,
}

pub(crate) struct Pending<T> {
    future: PlmFuture<T>,
}

impl<T> Pending<T> {
    pub(crate) fn new(future: PlmFuture<T>) -> Self {
        Self { future }
    }

    pub(crate) fn poll(&mut self) -> Option<Result<T, String>> {
        let mut cx = Context::from_waker(Waker::noop());
        match self.future.as_mut().poll(&mut cx) {
            Poll::Ready(result) => Some(result),
            Poll::Pending => None,
        }
    }
}

/// The PLM half of the family table pane.
#[derive(Default)]
pub struct PlmFamily {
    client: Option<Rc<PlmClient>>,
    key: Option<String>,
    info: Option<Info>,
    info_task: Option<Pending<Info>>,
    generate_task: Option<Pending<GenerateReport>>,
    set_type_task: Option<Pending<String>>,
    /// Generate was refused for unsaved edits; the pane offers Save and generate.
    pub refused_unsaved: bool,
    /// The user pressed Save and generate: the shell saves (take it with
    /// [`Self::take_save_request`]), and Generate starts once the save landed.
    save_request: bool,
    generate_after_save: bool,
    /// The member part type being typed.
    pub member_type_input: String,
    /// The last thing the server said, or why it could not be asked.
    pub message: Option<String>,
}

impl PlmFamily {
    /// Point the pane at `client` and the family's store `key` (both `None`
    /// on a file store). A change of family starts over and reads the server.
    pub fn sync(&mut self, client: Option<Rc<PlmClient>>, key: Option<&str>) {
        let key = if client.is_some() { key.and_then(family::revision_key) } else { None };
        let client = client.filter(|_| key.is_some());
        let same = self.key == key && self.client.as_ref().map(Rc::as_ptr) == client.as_ref().map(Rc::as_ptr);
        if same {
            return;
        }
        *self = Self { client, key, ..Self::default() };
        self.reload();
    }

    /// Whether this family is a PLM revision (so Generate is the server's).
    pub fn active(&self) -> bool {
        self.client.is_some()
    }

    pub fn info(&self) -> Option<&Info> {
        self.info.as_ref()
    }

    /// Something is on its way to or from the server.
    pub fn busy(&self) -> bool {
        self.info_task.is_some() || self.generate_task.is_some() || self.set_type_task.is_some() || self.generate_after_save
    }

    pub fn generating(&self) -> bool {
        self.generate_task.is_some() || self.generate_after_save
    }

    fn reload(&mut self) {
        let (Some(client), Some(key)) = (self.client.clone(), self.key.clone()) else { return };
        self.info_task = Some(Pending::new(Box::pin(async move { load_info(&client, &key).await })));
    }

    /// Generate pressed. `dirty`: the family has unsaved edits. `json`: the
    /// family document as it stands (which, when not dirty, is the saved one).
    pub fn request_generate(&mut self, dirty: bool, json: String) {
        let (Some(client), Some(key)) = (self.client.clone(), self.key.clone()) else { return };
        if self.generating() {
            return;
        }
        if dirty {
            self.refused_unsaved = true;
            self.message = Some(family::UNSAVED_FAMILY.into());
            return;
        }
        self.refused_unsaved = false;
        self.message = Some("generating on the server…".into());
        self.generate_task = Some(Pending::new(Box::pin(async move {
            family::generate(&client, &key, &json).await.map_err(|e| e.to_string())
        })));
    }

    /// Save and generate pressed.
    pub fn request_save_and_generate(&mut self) {
        if self.active() && !self.generating() {
            self.refused_unsaved = false;
            self.save_request = true;
            self.message = Some("saving the family…".into());
        }
    }

    /// Whether the shell should save the family now (once).
    pub fn take_save_request(&mut self) -> bool {
        let asked = std::mem::take(&mut self.save_request);
        if asked {
            self.generate_after_save = true;
        }
        asked
    }

    /// The shell's save failed: say so, and do not generate.
    pub fn save_failed(&mut self, error: String) {
        self.generate_after_save = false;
        self.message = Some(format!("the family was not saved, so nothing was generated: {error}"));
    }

    /// Set the member part type to what was typed.
    pub fn request_member_type(&mut self) {
        let (Some(client), Some(key)) = (self.client.clone(), self.key.clone()) else { return };
        let Some((part, _)) = family::key_ids(&key).map(|(p, r)| (p.to_string(), r.to_string())) else { return };
        let wanted = self.member_type_input.trim().to_string();
        self.set_type_task = Some(Pending::new(Box::pin(async move {
            family::set_member_part_type(&client, &part, &wanted).await.map_err(|e| e.to_string())?;
            Ok(wanted)
        })));
    }

    /// Poll what is in flight. `dirty` and `writes_pending` say whether a save
    /// asked for has reached the server. Returns a finished Generate's report.
    pub fn poll(&mut self, dirty: bool, writes_pending: usize, json: impl FnOnce() -> String) -> Option<GenerateReport> {
        if let Some(task) = self.info_task.as_mut() {
            if let Some(result) = task.poll() {
                self.info_task = None;
                match result {
                    Ok(info) => {
                        // Keep what the user is typing: only a field still
                        // showing the old answer (or nothing) follows the server.
                        let untouched = self.member_type_input.trim().is_empty()
                            || self.info.as_ref().is_some_and(|old| old.member_part_type == self.member_type_input.trim());
                        if untouched {
                            self.member_type_input = info.member_part_type.clone();
                        }
                        self.info = Some(info);
                    }
                    Err(error) => self.message = Some(format!("the family's server details could not be read: {error}")),
                }
            }
        }
        if let Some(task) = self.set_type_task.as_mut() {
            if let Some(result) = task.poll() {
                self.set_type_task = None;
                self.message = Some(match result {
                    Ok(set) => format!("new members are created as {set}"),
                    Err(error) => error,
                });
                self.reload();
            }
        }
        if self.generate_after_save && !self.save_request && !dirty && writes_pending == 0 {
            self.generate_after_save = false;
            self.request_generate(false, json());
        }
        let task = self.generate_task.as_mut()?;
        let result = task.poll()?;
        self.generate_task = None;
        match result {
            Ok(report) => {
                self.message = None;
                self.reload();
                Some(report)
            }
            Err(error) => {
                self.message = Some(format!("Generate was refused: {error}"));
                None
            }
        }
    }

    /// The pane's PLM lines: member part type, the category's keys to add, and
    /// the cells the server will refuse. Returns a key the user picked to add
    /// as a column.
    pub fn show(&mut self, ui: &mut egui::Ui, table: &FamilyTable, hits: &mut std::collections::HashMap<String, egui::Rect>) -> Option<String> {
        if !self.active() {
            return None;
        }
        let mut picked = None;
        ui.horizontal_wrapped(|ui| {
            ui.label("Members are created as part type");
            let field = ui.add(egui::TextEdit::singleline(&mut self.member_type_input).desired_width(110.0));
            hits.insert("family:plm_member_type".into(), field.rect);
            if let Some(info) = &self.info {
                if !info.member_part_type_mode.is_empty() {
                    ui.weak(format!("({} numbering)", info.member_part_type_mode));
                }
            }
            let changed = self.info.as_ref().is_some_and(|i| i.member_part_type != self.member_type_input.trim());
            let set = ui.add_enabled(changed && self.set_type_task.is_none(), egui::Button::new("Set"));
            hits.insert("family:plm_member_type_set".into(), set.rect);
            if set.clicked() {
                self.request_member_type();
            }
        });
        if let Some(info) = &self.info {
            let offered = family::keys_to_offer(&info.schema, table);
            if !offered.is_empty() {
                ui.horizontal_wrapped(|ui| {
                    ui.label("Category keys:");
                    for attribute in offered {
                        let button = ui.small_button(&attribute.key).on_hover_text(format!(
                            "Add {} ({}) as a column; it becomes each member's catalog value",
                            attribute.name, attribute.kind
                        ));
                        hits.insert(format!("family:plm_key:{}", attribute.key), button.rect);
                        if button.clicked() {
                            picked = Some(attribute.key.clone());
                        }
                    }
                });
            }
            let problems = family::attribute_column_problems(&info.schema, table);
            for (i, problem) in problems.iter().enumerate() {
                let label = ui.add(egui::Label::new(egui::RichText::new(problem).color(ui.visuals().warn_fg_color)).wrap());
                hits.insert(format!("family:plm_attribute_problem:{i}"), label.rect);
            }
        }
        if self.refused_unsaved {
            let button = ui.button("Save and generate").on_hover_text("Save the family to the server, then Generate from it");
            hits.insert("family:plm_save_and_generate".into(), button.rect);
            if button.clicked() {
                self.request_save_and_generate();
            }
        }
        if let Some(message) = &self.message {
            ui.add(egui::Label::new(egui::RichText::new(message).color(ui.visuals().weak_text_color())).wrap());
        }
        if self.busy() {
            ui.ctx().request_repaint_after(std::time::Duration::from_millis(30));
        }
        picked
    }

    /// For the pane's published state.
    pub fn state_json(&self, table: Option<&FamilyTable>) -> serde_json::Value {
        if !self.active() {
            return serde_json::Value::Null;
        }
        let problems = match (&self.info, table) {
            (Some(info), Some(table)) => family::attribute_column_problems(&info.schema, table),
            _ => Vec::new(),
        };
        let keys: Vec<&str> = match (&self.info, table) {
            (Some(info), Some(table)) => family::keys_to_offer(&info.schema, table).into_iter().map(|a| a.key.as_str()).collect(),
            _ => Vec::new(),
        };
        serde_json::json!({
            "key": self.key,
            "number": self.info.as_ref().map(|i| i.number.clone()),
            "memberPartType": self.info.as_ref().map(|i| i.member_part_type.clone()),
            "memberPartTypeMode": self.info.as_ref().map(|i| i.member_part_type_mode.clone()),
            "category": self.info.as_ref().map(|i| i.category.clone()),
            "keysToOffer": keys,
            "attributeProblems": problems,
            "refusedUnsaved": self.refused_unsaved,
            "busy": self.busy(),
            "message": self.message,
        })
    }
}

/// The family's number and member part type (`GET /api/parts/:id/family`),
/// and its category's schema (`GET /api/categories/:id/schema`).
async fn load_info(client: &PlmClient, key: &str) -> Result<Info, String> {
    let (part, _) = family::key_ids(key).ok_or_else(|| format!("`{key}` is not a PLM revision"))?;
    let head = family::part_head(client, part).await.map_err(|e| e.to_string())?;
    let view = family::family_view(client, part).await.map_err(|e| e.to_string())?;
    let schema = if head.category.is_empty() {
        Vec::new()
    } else {
        let response = client
            .call("GET", &format!("/api/categories/{}/schema", head.category), None)
            .await
            .map_err(|e| e.to_string())?;
        let schema: crate::panels::plm_parts::Schema =
            serde_json::from_slice(&response.body).map_err(|e| format!("the category's schema: {e}"))?;
        schema.attributes
    };
    Ok(Info {
        number: head.number,
        member_part_type: view.member_part_type,
        member_part_type_mode: view.member_part_type_mode,
        category: head.category,
        schema,
    })
}

