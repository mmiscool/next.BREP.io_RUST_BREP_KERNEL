//! Server field definitions and shared/personal BOM column configurations.
use super::bom_columns;
use crate::{column_tree::CellKind, plm::client::PlmClient};
use eframe::egui;
use serde::Deserialize;
use serde_json::{json, Value};
use std::{cell::RefCell, collections::BTreeMap, rc::Rc};
use web_time::{Duration, Instant};

#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct EditTarget {
    pub resource: String,
    pub key: String,
}
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Field {
    pub id: String,
    pub name: String,
    pub scope: String,
    pub part_type: Option<String>,
    pub key: String,
    pub editable: bool,
    pub edit: Option<EditTarget>,
    #[serde(rename = "type")]
    pub kind: String,
    pub unit: String,
    pub values: Vec<String>,
    pub cad_field: String,
}
impl Field {
    pub fn column_key(&self) -> String {
        if self.scope == "part" {
            return self.id.clone();
        }
        if self.scope == "occurrence" || self.key == "quantity" {
            format!("occurrence.{}", self.cad_field)
        } else {
            format!("part.{}", self.cad_field)
        }
    }
    pub fn cell_kind(&self) -> CellKind {
        if !self.editable {
            return CellKind::ReadOnly;
        }
        match self.kind.as_str() {
            "number" => CellKind::Numeric { step: 0.1 },
            "bool" => CellKind::Toggle,
            "enum" => CellKind::Choice {
                options: self.values.clone(),
            },
            _ => CellKind::Text,
        }
    }
}
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Layout {
    pub id: String,
    pub name: String,
    pub owner: Option<String>,
    pub columns: Vec<String>,
}
#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default)]
pub struct Config {
    pub fields: Vec<Field>,
    pub layouts: Vec<Layout>,
    pub selected: Option<String>,
    pub snapshots: BTreeMap<String, Value>,
}
type Slot = Rc<RefCell<Option<Result<Config, String>>>>;
#[derive(Default)]
pub struct BomConfiguration {
    pub config: Option<Config>,
    slot: Option<Slot>,
    last: Option<Instant>,
    keys: String,
    pub columns: Vec<String>,
    selected: String,
    name: String,
    dirty: bool,
    error: Option<String>,
    save: Option<Rc<RefCell<Option<Result<String, String>>>>>,
}
fn spawn(task: impl std::future::Future<Output = ()> + 'static) {
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(task);
    #[cfg(not(target_arch = "wasm32"))]
    crate::plm::native::spawn(task);
}
impl BomConfiguration {
    pub fn ensure(&mut self, client: &Rc<PlmClient>, keys: Vec<String>) {
        let keys = keys
            .iter()
            .filter_map(|key| {
                crate::plm::bom::revision_of_document(key)
                    .map(|(part, rev)| crate::plm::identity::document_key(&part, &rev))
            })
            .collect::<Vec<_>>()
            .join(",");
        if let Some(slot) = &self.slot {
            let answer = slot.borrow_mut().take();
            if let Some(answer) = answer {
                self.slot = None;
                match answer {
                    Ok(config) => {
                        let selected = config.selected.clone().unwrap_or_default();
                        let layout_changed = !self.dirty
                            && self
                                .config
                                .as_ref()
                                .and_then(|old| old.layouts.iter().find(|l| l.id == selected))
                                .map(|l| &l.columns)
                                != config
                                    .layouts
                                    .iter()
                                    .find(|l| l.id == selected)
                                    .map(|l| &l.columns);
                        if self.config.is_none()
                            || ((selected != self.selected || layout_changed)
                                && self.save.is_none())
                        {
                            self.columns = config
                                .layouts
                                .iter()
                                .find(|l| l.id == selected)
                                .map(|l| l.columns.clone())
                                .unwrap_or_else(|| {
                                    vec![
                                        "builtin.number".into(),
                                        "builtin.name".into(),
                                        "builtin.revision_label".into(),
                                        "builtin.quantity".into(),
                                        "builtin.total".into(),
                                    ]
                                });
                            self.name = config
                                .layouts
                                .iter()
                                .find(|l| l.id == selected)
                                .map(|l| l.name.clone())
                                .unwrap_or_default();
                            self.selected = selected;
                            self.dirty = false;
                        }
                        self.config = Some(config);
                        self.error = None;
                    }
                    Err(error) => self.error = Some(error),
                }
            }
        }
        if let Some(slot) = &self.save {
            let answer = slot.borrow_mut().take();
            if let Some(answer) = answer {
                self.save = None;
                match answer {
                    Ok(_) => {
                        self.last = None;
                        self.dirty = false;
                        self.error = None;
                    }
                    Err(error) => self.error = Some(error),
                }
            }
        }
        if self.save.is_some()
            || self.slot.is_some()
            || (keys == self.keys
                && self
                    .last
                    .is_some_and(|last| last.elapsed() < Duration::from_secs(3)))
        {
            return;
        }
        self.keys = keys.clone();
        self.last = Some(Instant::now());
        let slot: Slot = Rc::default();
        self.slot = Some(slot.clone());
        let client = client.clone();
        spawn(async move {
            let query = crate::plm::identity::segment(&keys);
            let answer = match client
                .call("GET", &format!("/api/bom/configuration?keys={query}"), None)
                .await
            {
                Ok(r) => serde_json::from_slice(&r.body).map_err(|e| e.to_string()),
                Err(e) => Err(e.to_string()),
            };
            *slot.borrow_mut() = Some(answer);
        });
    }
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        client: &Rc<PlmClient>,
        hits: &mut std::collections::HashMap<String, egui::Rect>,
    ) {
        ui.ctx().request_repaint_after(Duration::from_secs(1));
        let Some(config) = self.config.clone() else {
            ui.label("Loading BOM field configuration…");
            return;
        };
        ui.horizontal(|ui|{
            ui.label("Columns");
            let title=config.layouts.iter().find(|l|l.id==self.selected).map(|l|l.name.as_str()).unwrap_or("Default");
            let mut chosen=self.selected.clone();
            let combo=egui::ComboBox::from_id_salt("plm-bom-layout").selected_text(if self.dirty {"Custom columns"}else{title}).show_ui(ui,|ui|{
                let default=ui.selectable_value(&mut chosen,String::new(),"Default");hits.insert("bom:layout:default".into(),default.rect);
                for layout in &config.layouts{let r=ui.selectable_value(&mut chosen,layout.id.clone(),format!("{} ({})",layout.name,if layout.owner.is_none(){"Shared"}else{"Personal"}));hits.insert(format!("bom:layout:{}",layout.name),r.rect);}
            });hits.insert("bom:layout".into(),combo.response.rect);
            if chosen!=self.selected{
                let client=client.clone();let id=chosen.clone();
                let slot:Rc<RefCell<Option<Result<String,String>>>>=Rc::default();self.save=Some(slot.clone());
                spawn(async move{*slot.borrow_mut()=Some(client.call("PUT","/api/bom/selection",Some(json!({"id":id}).to_string().into_bytes())).await.map(|_|id).map_err(|e|e.to_string()));});
                self.columns=config.layouts.iter().find(|l|l.id==chosen).map(|l|l.columns.clone()).unwrap_or_else(||vec!["builtin.number".into(),"builtin.name".into(),"builtin.revision_label".into(),"builtin.quantity".into(),"builtin.total".into()]);
                self.name=config.layouts.iter().find(|l|l.id==chosen).map(|l|l.name.clone()).unwrap_or_default();
                self.selected=chosen;self.last=None;self.dirty=false;
            }
            let menu=egui::containers::menu::MenuButton::new("Choose fields…").config(egui::containers::menu::MenuConfig::new().close_behavior(egui::PopupCloseBehavior::CloseOnClickOutside)).ui(ui,|ui|{
                egui::ScrollArea::vertical().max_height(360.0).show(ui,|ui|{
                    for field in &config.fields {
                        let mut checked=self.columns.contains(&field.id);
                        let r=ui.checkbox(&mut checked,format!("{} — {}",field.name,if field.scope=="part"{"Part revision"}else if field.scope=="occurrence"{"Occurrence"}else{"Built-in"}));hits.insert(format!("bom:field:{}",field.id),r.rect);
                        if r.changed(){self.dirty=true;if checked{self.columns.push(field.id.clone());}else{self.columns.retain(|id|id!=&field.id);}}
                    }
                });
                ui.separator();
                for i in 0..self.columns.len(){
                    let field=config.fields.iter().find(|f|f.id==self.columns[i]);
                    if let Some(field)=field{ui.horizontal(|ui|{ui.label(&field.name);if ui.small_button("↑").clicked()&&i>0{self.columns.swap(i,i-1);self.dirty=true;}if ui.small_button("↓").clicked()&&i+1<self.columns.len(){self.columns.swap(i,i+1);self.dirty=true;}});}
                }
            });hits.insert("bom:choose-fields".into(),menu.0.rect);
            let popup=egui::containers::menu::MenuButton::new("Save configuration…").config(egui::containers::menu::MenuConfig::new().close_behavior(egui::PopupCloseBehavior::CloseOnClickOutside)).ui(ui,|ui|{
                ui.label("Personal configuration name");let input=ui.text_edit_singleline(&mut self.name);hits.insert("bom:layout-name".into(),input.rect);
                let button=ui.add_enabled(!self.name.trim().is_empty()&&self.save.is_none(),egui::Button::new("Save personal configuration"));hits.insert("bom:layout-save".into(),button.rect);
                if button.clicked(){
                    let slot:Rc<RefCell<Option<Result<String,String>>>>=Rc::default();self.save=Some(slot.clone());
                    let client=client.clone();let name=self.name.clone();let columns=self.columns.clone();
                    let id=config.layouts.iter().find(|l|l.id==self.selected&&l.owner.is_some()&&l.name==self.name).map(|l|l.id.clone()).unwrap_or_default();
                    spawn(async move{
                        let result=async{
                            let r=client.call("POST","/api/bom/layouts",Some(json!({"id":id,"name":name,"columns":columns,"shared":false}).to_string().into_bytes())).await.map_err(|e|e.to_string())?;
                            let layout:Layout=serde_json::from_slice(&r.body).map_err(|e|e.to_string())?;
                            client.call("PUT","/api/bom/selection",Some(json!({"id":layout.id}).to_string().into_bytes())).await.map_err(|e|e.to_string())?;
                            Ok(layout.id)
                        }.await;*slot.borrow_mut()=Some(result);
                    });
                    ui.close();
                }
            });hits.insert("bom:save-layout".into(),popup.0.rect);
        });
        if let Some(error) = &self.error {
            ui.colored_label(egui::Color32::LIGHT_RED, error);
        }
    }
    pub fn text(&self) -> Option<String> {
        let config = self.config.as_ref()?;
        // Keep hidden fields available for properties and the column chooser.
        let ordered = self
            .columns
            .iter()
            .filter_map(|id| config.fields.iter().find(|f| f.id == *id))
            .chain(
                config
                    .fields
                    .iter()
                    .filter(|f| !self.columns.contains(&f.id)),
            );
        Some(
            ordered
                .map(|f| {
                    format!(
                        "{}{}\n",
                        if self.columns.contains(&f.id) {
                            "*"
                        } else {
                            ""
                        },
                        f.column_key()
                    )
                })
                .collect(),
        )
    }
    pub fn field(&self, key: &str) -> Option<&Field> {
        self.config
            .as_ref()?
            .fields
            .iter()
            .find(|f| f.column_key() == key)
    }
    pub fn snapshot(&self, key: &str) -> Option<&Value> {
        let (part, rev) = crate::plm::bom::revision_of_document(key)?;
        self.config
            .as_ref()?
            .snapshots
            .get(&crate::plm::identity::document_key(&part, &rev))
    }
    pub fn occurrence(&self, key: &str, id: &str) -> Option<Value> {
        self.snapshot(key)?["occurrences"]
            .as_array()?
            .iter()
            .find(|o| o["id"].as_str() == Some(id))
            .map(|o| o["attributes"].clone())
    }
    pub fn adopt_layout(&mut self, columns: &[bom_columns::BomColumn]) {
        let Some(config) = &self.config else { return };
        self.columns = columns
            .iter()
            .filter(|c| c.shown)
            .filter_map(|c| {
                config
                    .fields
                    .iter()
                    .find(|f| f.column_key() == c.key())
                    .map(|f| f.id.clone())
            })
            .collect();
        self.dirty = true;
    }
}

impl BomConfiguration {
    pub fn attributes_ui(
        &mut self,
        ui: &mut egui::Ui,
        client: &Rc<PlmClient>,
        key: &str,
        hits: &mut std::collections::HashMap<String, egui::Rect>,
    ) {
        ui.ctx().request_repaint_after(Duration::from_secs(1));
        let Some(config) = self.config.clone() else {
            ui.label("Loading revision attributes…");
            return;
        };
        let Some(snapshot) = self.snapshot(key).cloned() else {
            ui.label("Loading revision attributes…");
            return;
        };
        let fields: Vec<Field> = config
            .fields
            .iter()
            .filter(|f| {
                f.scope == "part" && f.part_type.as_deref() == snapshot["part_type"].as_str()
            })
            .cloned()
            .collect();
        if fields.is_empty() {
            ui.label("No fields configured for this part type.");
        }
        let mut edit = None;
        egui::Grid::new(("plm-revision-fields", key))
            .striped(true)
            .show(ui, |ui| {
                for field in fields {
                    ui.label(if field.unit.is_empty() {
                        field.name.clone()
                    } else {
                        format!("{} ({})", field.name, field.unit)
                    });
                    let kind = if snapshot["editable"].as_bool() == Some(true) {
                        field.cell_kind()
                    } else {
                        CellKind::ReadOnly
                    };
                    let (value, rect) = crate::column_tree::value_editor(
                        ui,
                        egui::Id::new(("plm-revision-field", key, &field.key)),
                        &kind,
                        snapshot["attributes"].get(&field.key),
                        egui::vec2(180.0, ui.spacing().interact_size.y),
                    );
                    hits.insert(format!("field:{}", field.key), rect);
                    if let Some(value) = value {
                        edit = Some((field.key, value));
                    }
                    ui.end_row();
                }
            });
        if let Some((field, value)) = edit {
            if let Some((part, rev)) = crate::plm::bom::revision_of_document(key) {
                let slot: Rc<RefCell<Option<Result<String, String>>>> = Rc::default();
                self.save = Some(slot.clone());
                let client = client.clone();
                let path = format!(
                    "{}/attributes",
                    crate::plm::identity::revision_path(&part, &rev)
                );
                spawn(async move {
                    *slot.borrow_mut() = Some(
                        client
                            .call(
                                "PATCH",
                                &path,
                                Some(json!({"attributes":{field:value}}).to_string().into_bytes()),
                            )
                            .await
                            .map(|_| String::new())
                            .map_err(|e| e.to_string()),
                    );
                });
            }
        }
        if self.save.is_some() {
            ui.spinner();
        }
        if let Some(error) = &self.error {
            ui.colored_label(egui::Color32::LIGHT_RED, error);
        }
    }
}

