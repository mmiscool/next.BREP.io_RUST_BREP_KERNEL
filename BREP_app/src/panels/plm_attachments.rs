//! A PLM part's and revision's files, in the CAD app (plm-cad-integration-todo §3 S8).
//!
//! * **The list.** `GET /api/parts/:id/attachments?revision=` answers the part's files
//!   and the revision's, each with its name, kind (`drawing`, `datasheet`, `spec`,
//!   `image`, `other`), size and media type. Drawings matter most, so they sort first.
//! * **Download.** `GET /api/attachments/:id`. The bytes go to the host
//!   ([`PlmAttachmentsOutcome::downloaded`]), which saves them the way the app saves
//!   any file.
//! * **Attach.** The host hands in a file: one the user picked, or one the app just
//!   exported, such as a drawing PDF or a STEP for the revision being saved
//!   ([`PlmAttachmentsPanel::stage`]). The user confirms its kind and note and
//!   whether it belongs to the part or to this revision. It is sent with its media
//!   type (`call_typed`), so a PDF stays `application/pdf` and can be viewed inline.
//!
//! **Every refusal is the server's own sentence, shown as sent:**
//! * a released (or superseded, obsolete) revision refuses a file with `409`;
//! * a revision checked out by someone else refuses with `409`, naming them;
//! * a file over the server's limit is refused with `413`;
//! * an account outside the author group is refused with `403`.
//!
//! The server is the only judge. The panel only marks a revision that is not
//! editable, so the user is not surprised.
//!
//! The panel talks through [`PartAttachments`], which S1's client implements. It
//! polls its requests each frame with a waker that repaints, like S7's panel.
//! Nothing here is constructed without a server.

use crate::plm::PlmFuture;
use eframe::egui;
use std::collections::HashMap;
use std::task::Waker;

use super::plm_parts::Pending;

// --- the wire ------------------------------------------------------------------
//
// These mirror `BREP_plm/src/api/attachments.rs`. The golden tests below pin them
// against the REAL router. Every field defaults, so a server that adds a field
// breaks nothing.

/// One file on a part or a revision.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
#[serde(default)]
pub struct Attachment {
    pub id: String,
    pub name: String,
    pub media_type: String,
    pub size: u64,
    pub sha256: String,
    pub kind: String,
    pub note: String,
    pub uploaded_by_name: String,
    pub uploaded_at: u64,
    /// The server will show it in a browser rather than download it.
    pub inline: bool,
}

/// `GET /api/parts/:id/attachments?revision=`.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
#[serde(default)]
pub struct AttachmentList {
    pub part: Vec<Attachment>,
    pub revision: Vec<Attachment>,
    pub revision_label: String,
    /// Whether the revision can still take files (a draft). The server decides
    /// either way.
    pub revision_editable: bool,
    /// The kinds the server accepts.
    pub kinds: Vec<String>,
}

#[derive(serde::Deserialize)]
struct Uploaded {
    attachment: Attachment,
}

/// Where a file goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Target {
    /// The part, across all its revisions (a datasheet).
    Part,
    /// This revision only (its drawing, its exported STEP).
    Revision,
}

/// A file about to be attached.
#[derive(Clone, Debug, PartialEq)]
pub struct Upload {
    pub name: String,
    pub media_type: String,
    pub kind: String,
    pub note: String,
    pub bytes: Vec<u8>,
}

impl Upload {
    /// A file with its media type and kind worked out from its name
    /// ([`media_type_for`], [`kind_for`]).
    pub fn of(name: &str, bytes: Vec<u8>) -> Self {
        let media_type = media_type_for(name).to_string();
        Upload { name: name.to_string(), kind: kind_for(&media_type).to_string(), media_type, note: String::new(), bytes }
    }
}

/// The media type a CAD user's files are sent as, by extension. The server
/// shows only a few safe types inline; this only names the type.
pub fn media_type_for(name: &str) -> &'static str {
    let extension = name.rsplit_once('.').map(|(_, e)| e.to_ascii_lowercase()).unwrap_or_default();
    match extension.as_str() {
        "pdf" => "application/pdf",
        "step" | "stp" => "model/step",
        "stl" => "model/stl",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "svg" => "image/svg+xml",
        "dxf" => "image/vnd.dxf",
        "txt" | "md" => "text/plain",
        "csv" => "text/csv",
        "json" | "nbrep" | "fbrep" | "tbrep" => "application/json",
        _ => "application/octet-stream",
    }
}

/// The kind a file is attached as unless the user picks another: a PDF is a
/// drawing (what the app exports), an image an image, anything else `other`.
pub fn kind_for(media_type: &str) -> &'static str {
    if media_type == "application/pdf" {
        "drawing"
    } else if media_type.starts_with("image/") && media_type != "image/vnd.dxf" {
        "image"
    } else {
        "other"
    }
}

/// `12 B`, `3.4 KB`, `1.2 MB`: for the list.
pub fn human_size(bytes: u64) -> String {
    const KB: f64 = 1024.0;
    let b = bytes as f64;
    if b < KB {
        format!("{bytes} B")
    } else if b < KB * KB {
        format!("{:.1} KB", b / KB)
    } else if b < KB * KB * KB {
        format!("{:.1} MB", b / (KB * KB))
    } else {
        format!("{:.1} GB", b / (KB * KB * KB))
    }
}

/// Percent-encode a query or path value.
fn encode(text: &str) -> String {
    let mut out = String::new();
    for byte in text.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'.' | b'_' | b'~' => out.push(byte as char),
            _ => out.push_str(&format!("%{byte:02X}")),
        }
    }
    out
}

/// The PLM's attachment routes, as this panel needs them. An `Err` is the
/// sentence to show: the server's refusal, word for word, or why no answer
/// came.
pub trait PartAttachments {
    fn list(&self, part: &str, revision: &str) -> PlmFuture<AttachmentList>;
    fn download(&self, id: &str) -> PlmFuture<Vec<u8>>;
    fn upload(&self, part: &str, revision: Option<&str>, upload: &Upload) -> PlmFuture<Attachment>;
}

/// [`PartAttachments`] over S1's [`PlmClient`](crate::plm::client::PlmClient).
pub struct PlmAttachments {
    pub client: std::rc::Rc<crate::plm::client::PlmClient>,
}

impl PartAttachments for PlmAttachments {
    fn list(&self, part: &str, revision: &str) -> PlmFuture<AttachmentList> {
        let client = self.client.clone();
        let path = format!("/api/parts/{}/attachments?revision={}", encode(part), encode(revision));
        Box::pin(async move {
            let response = client.call("GET", &path, None).await.map_err(|e| e.to_string())?;
            serde_json::from_slice(&response.body)
                .map_err(|e| format!("the PLM answered {path} with something this app cannot read: {e}"))
        })
    }

    fn download(&self, id: &str) -> PlmFuture<Vec<u8>> {
        let client = self.client.clone();
        let path = format!("/api/attachments/{}", encode(id));
        Box::pin(async move { Ok(client.call("GET", &path, None).await.map_err(|e| e.to_string())?.body) })
    }

    fn upload(&self, part: &str, revision: Option<&str>, upload: &Upload) -> PlmFuture<Attachment> {
        let client = self.client.clone();
        let base = match revision {
            Some(rev) => format!("/api/parts/{}/revisions/{}/attachments", encode(part), encode(rev)),
            None => format!("/api/parts/{}/attachments", encode(part)),
        };
        let path = format!("{base}?name={}&kind={}&note={}", encode(&upload.name), encode(&upload.kind), encode(&upload.note));
        let (bytes, media_type) = (upload.bytes.clone(), upload.media_type.clone());
        Box::pin(async move {
            let response = client.call_typed("POST", &path, bytes, &media_type).await.map_err(|e| e.to_string())?;
            serde_json::from_slice::<Uploaded>(&response.body)
                .map(|u| u.attachment)
                .map_err(|e| format!("the PLM answered the upload with something this app cannot read: {e}"))
        })
    }
}

// --- the panel -------------------------------------------------------------------

/// What the host does after a frame.
#[derive(Default)]
pub struct PlmAttachmentsOutcome {
    /// The user asked to attach a file: open the file chooser, then
    /// [`PlmAttachmentsPanel::stage`] what they picked.
    pub pick_file: bool,
    /// A downloaded file: save it where the user chooses.
    pub downloaded: Option<(Attachment, Vec<u8>)>,
    /// A file the server accepted.
    pub attached: Option<Attachment>,
}

/// The files of one part and one of its revisions.
pub struct PlmAttachmentsPanel {
    pub part: String,
    pub revision: String,
    pub list: Option<AttachmentList>,
    /// A file waiting for the user to confirm it, and where it goes.
    pub staged: Option<(Upload, Target)>,
    /// The last refusal or failure, word for word.
    pub problem: Option<String>,
    /// What just happened, for the status line.
    pub notice: Option<String>,
    list_pending: Option<Pending<AttachmentList>>,
    download_pending: Option<(Attachment, Pending<Vec<u8>>)>,
    upload_pending: Option<Pending<Attachment>>,
    pub hits: HashMap<String, egui::Rect>,
    /// Whether the host has a file chooser to answer `pick_file` with. The
    /// shell has none yet, so "Attach a file…" is hidden and files come from
    /// the app's own exports ([`AttachmentsSection`]).
    pub can_pick_files: bool,
}

impl PlmAttachmentsPanel {
    /// The files of `part` and its `revision` (an id or a label).
    pub fn new(part: impl Into<String>, revision: impl Into<String>) -> Self {
        PlmAttachmentsPanel {
            part: part.into(),
            revision: revision.into(),
            list: None,
            staged: None,
            problem: None,
            notice: None,
            list_pending: None,
            download_pending: None,
            upload_pending: None,
            hits: HashMap::new(),
            can_pick_files: false,
        }
    }

    /// Ask for the list again.
    pub fn reload(&mut self, plm: &dyn PartAttachments) {
        self.list_pending = Some(Pending::new(plm.list(&self.part, &self.revision)));
    }

    /// Stage a file to attach: one the user picked, or one the app exported.
    /// It goes to the revision while the revision can take files (the file
    /// panel's "attach to the revision being saved"), else to the part.
    pub fn stage(&mut self, upload: Upload) {
        let target = if self.list.as_ref().is_some_and(|l| !l.revision_editable) { Target::Part } else { Target::Revision };
        self.problem = None;
        self.staged = Some((upload, target));
    }

    /// Send the staged file.
    pub fn attach(&mut self, plm: &dyn PartAttachments) {
        let Some((upload, target)) = self.staged.as_ref() else { return };
        let revision = (*target == Target::Revision).then_some(self.revision.as_str());
        self.problem = None;
        self.upload_pending = Some(Pending::new(plm.upload(&self.part, revision, upload)));
    }

    /// Download one file.
    pub fn download(&mut self, plm: &dyn PartAttachments, attachment: &Attachment) {
        self.problem = None;
        self.download_pending = Some((attachment.clone(), Pending::new(plm.download(&attachment.id))));
    }

    /// Whether any request is still out.
    pub fn busy(&self) -> bool {
        self.list_pending.is_some() || self.download_pending.is_some() || self.upload_pending.is_some()
    }

    /// Fold in every answer that has arrived.
    pub fn poll(&mut self, waker: &Waker, plm: &dyn PartAttachments, outcome: &mut PlmAttachmentsOutcome) {
        if let Some(answer) = self.list_pending.as_mut().and_then(|p| p.poll(waker)) {
            self.list_pending = None;
            match answer {
                Ok(list) => self.list = Some(list),
                Err(error) => self.problem = Some(error),
            }
        }
        if let Some(answer) = self.download_pending.as_mut().and_then(|(_, p)| p.poll(waker)) {
            let (attachment, _) = self.download_pending.take().expect("polled above");
            match answer {
                Ok(bytes) => {
                    self.notice = Some(format!("downloaded {} ({})", attachment.name, human_size(bytes.len() as u64)));
                    outcome.downloaded = Some((attachment, bytes));
                }
                Err(error) => self.problem = Some(error),
            }
        }
        if let Some(answer) = self.upload_pending.as_mut().and_then(|p| p.poll(waker)) {
            self.upload_pending = None;
            match answer {
                Ok(attachment) => {
                    self.staged = None;
                    self.notice = Some(format!("attached {} as {}", attachment.name, attachment.kind));
                    outcome.attached = Some(attachment);
                    self.reload(plm);
                }
                // Kept staged, so the user can retry: attach it to the part
                // instead, or once the lock is free.
                Err(error) => self.problem = Some(error),
            }
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui, plm: &dyn PartAttachments) -> PlmAttachmentsOutcome {
        self.hits.clear();
        let mut outcome = PlmAttachmentsOutcome::default();
        if self.list.is_none() && self.list_pending.is_none() && self.problem.is_none() {
            self.reload(plm);
        }
        let waker = repaint_waker(ui.ctx());
        self.poll(&waker, plm, &mut outcome);

        ui.horizontal(|ui| {
            if self.can_pick_files {
                let pick = ui.add_enabled(self.upload_pending.is_none(), egui::Button::new("Attach a file…"));
                self.hits.insert("plm_attachments:pick".into(), pick.rect);
                outcome.pick_file |= pick.clicked();
            }
            let reload = ui.add_enabled(self.list_pending.is_none(), egui::Button::new("Reload"));
            self.hits.insert("plm_attachments:reload".into(), reload.rect);
            if reload.clicked() {
                self.problem = None;
                self.reload(plm);
            }
            if self.busy() {
                ui.spinner();
            }
        });
        self.show_staged(ui, plm);
        if let Some(problem) = &self.problem {
            ui.colored_label(ui.visuals().error_fg_color, problem);
        } else if let Some(notice) = &self.notice {
            ui.weak(notice);
        }
        ui.separator();

        let Some(list) = self.list.clone() else { return outcome };
        let revision_title = if list.revision_editable {
            format!("Revision {}", list.revision_label)
        } else {
            format!("Revision {} (released: its files are frozen)", list.revision_label)
        };
        for (title, files) in [(revision_title, &list.revision), ("Part".to_string(), &list.part)] {
            ui.strong(title);
            if files.is_empty() {
                ui.weak("no files");
            }
            for attachment in sorted(files) {
                ui.horizontal(|ui| {
                    let row = ui.selectable_label(false, format!("{}  ·  {}  ·  {}", attachment.name, attachment.kind, human_size(attachment.size)));
                    let row = row.on_hover_text(format!(
                        "{}\nuploaded by {}{}",
                        attachment.media_type,
                        attachment.uploaded_by_name,
                        if attachment.note.is_empty() { String::new() } else { format!("\n{}", attachment.note) }
                    ));
                    self.hits.insert(format!("plm_attachments:file:{}", attachment.id), row.rect);
                    let button = ui.add_enabled(self.download_pending.is_none(), egui::Button::new("Download"));
                    self.hits.insert(format!("plm_attachments:download:{}", attachment.id), button.rect);
                    if button.clicked() || row.double_clicked() {
                        self.download(plm, attachment);
                    }
                });
            }
        }
        // Poll again: a request issued this frame has not registered its waker.
        self.poll(&waker, plm, &mut outcome);
        outcome
    }

    fn show_staged(&mut self, ui: &mut egui::Ui, plm: &dyn PartAttachments) {
        let kinds: Vec<String> = self
            .list
            .as_ref()
            .map(|l| l.kinds.clone())
            .filter(|k| !k.is_empty())
            .unwrap_or_else(|| ["datasheet", "drawing", "spec", "image", "other"].map(String::from).to_vec());
        let revision_editable = self.list.as_ref().is_none_or(|l| l.revision_editable);
        let mut send = false;
        let mut cancel = false;
        let hits = &mut self.hits;
        let Some((upload, target)) = self.staged.as_mut() else { return };
        ui.group(|ui| {
            ui.label(format!("{} ({}, {})", upload.name, upload.media_type, human_size(upload.bytes.len() as u64)));
            ui.horizontal(|ui| {
                let combo = egui::ComboBox::from_id_salt("plm_attachments_kind").selected_text(upload.kind.clone()).show_ui(ui, |ui| {
                    for kind in &kinds {
                        ui.selectable_value(&mut upload.kind, kind.clone(), kind);
                    }
                });
                hits.insert("plm_attachments:staged:kind".into(), combo.response.rect);
                let part = ui.radio_value(target, Target::Part, "on the part");
                hits.insert("plm_attachments:staged:part".into(), part.rect);
                let rev = ui.radio_value(target, Target::Revision, if revision_editable { "on this revision" } else { "on this revision (frozen)" });
                hits.insert("plm_attachments:staged:revision".into(), rev.rect);
            });
            let note = ui.add(egui::TextEdit::singleline(&mut upload.note).hint_text("note (optional)"));
            hits.insert("plm_attachments:staged:note".into(), note.rect);
            ui.horizontal(|ui| {
                let attach = ui.button("Attach");
                hits.insert("plm_attachments:staged:attach".into(), attach.rect);
                send = attach.clicked();
                let no = ui.button("Cancel");
                hits.insert("plm_attachments:staged:cancel".into(), no.rect);
                cancel = no.clicked();
            });
        });
        if cancel {
            self.staged = None;
        } else if send {
            self.attach(plm);
        }
    }
}

/// A waker that repaints: an answer is drawn the frame it arrives.
fn repaint_waker(ctx: &egui::Context) -> Waker {
    struct Repaint(egui::Context);
    impl std::task::Wake for Repaint {
        fn wake(self: std::sync::Arc<Self>) {
            self.0.request_repaint();
        }
    }
    Waker::from(std::sync::Arc::new(Repaint(ctx.clone())))
}

/// Drawings first, then by name.
fn sorted(files: &[Attachment]) -> Vec<&Attachment> {
    let mut out: Vec<&Attachment> = files.iter().collect();
    out.sort_by(|a, b| (a.kind != "drawing").cmp(&(b.kind != "drawing")).then_with(|| a.name.to_lowercase().cmp(&b.name.to_lowercase())));
    out
}

// --- the PLM pane's Files section ---------------------------------------------

/// A document the Files section shows: the active document, when it is a PLM
/// revision. The host reads `part` and `revision` out of its name.
pub struct SectionDoc<'a> {
    pub id: u64,
    pub part: &'a str,
    pub revision: &'a str,
    /// The file name an export is attached under, without extension: the
    /// part number and revision label (`CPART000000001-A`).
    pub stem: String,
}

/// An export the user asked for, made by [`AttachmentsSection::sync`], which
/// holds the documents mutably (drawing does not).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Export {
    /// Every drawing sheet as one PDF: the drawing a revision is released with.
    DrawingPdf,
    /// The model as STEP.
    Step,
}

/// The Files section of the PLM pane: one [`PlmAttachmentsPanel`] per open
/// PLM document, the exports it attaches, and the downloads it saves.
#[derive(Default)]
pub struct AttachmentsSection {
    panels: HashMap<u64, PlmAttachmentsPanel>,
    exports: Vec<(u64, String, Export)>,
    downloads: Vec<(Attachment, Vec<u8>)>,
    /// What the last export or save did, when it is not the panel's own news.
    pub status: Option<String>,
    /// Whether the shell has a file chooser (it does, since the file chooser
    /// landed): every panel shows "Attach a file…".
    pub can_pick_files: bool,
    /// The document whose panel asked for a file this frame.
    asked_pick: Option<u64>,
}

impl AttachmentsSection {
    /// The panel of document `id`, once it has been shown.
    pub fn panel(&self, id: u64) -> Option<&PlmAttachmentsPanel> {
        self.panels.get(&id)
    }

    pub fn panel_mut(&mut self, id: u64) -> Option<&mut PlmAttachmentsPanel> {
        self.panels.get_mut(&id)
    }

    /// Ask for an export of document `id`, attached under `stem`; made by the
    /// next [`Self::sync`].
    pub fn request_export(&mut self, id: u64, stem: &str, export: Export) {
        self.exports.push((id, stem.to_string(), export));
    }

    /// Whether a request is still out.
    pub fn busy(&self) -> bool {
        !self.exports.is_empty() || self.panels.values().any(PlmAttachmentsPanel::busy)
    }

    /// `__brepPlmDoc.files`, for scripts: document `id`'s staged file, the
    /// listed files' names, the refusal or notice, and the section's status.
    /// `null` before the section has been shown for it.
    pub fn state_json(&self, id: u64) -> serde_json::Value {
        let Some(panel) = self.panels.get(&id) else { return serde_json::Value::Null };
        let names = |files: &[Attachment]| files.iter().map(|a| a.name.clone()).collect::<Vec<_>>();
        serde_json::json!({
            "staged": panel.staged.as_ref().map(|(u, target)| serde_json::json!({
                "name": u.name, "mediaType": u.media_type, "kind": u.kind, "size": u.bytes.len(),
                "target": if *target == Target::Revision { "revision" } else { "part" },
            })),
            "revision": panel.list.as_ref().map(|l| names(&l.revision)),
            "part": panel.list.as_ref().map(|l| names(&l.part)),
            "problem": panel.problem,
            "notice": panel.notice,
            "status": self.status,
            "busy": panel.busy(),
        })
    }

    /// Once a frame, with the documents and the store: drop the panels of
    /// closed documents, make the exports asked for and stage them, and save
    /// what was downloaded through the store's export lane (a browser
    /// download; `<root>/models` natively).
    pub fn sync(&mut self, store: &dyn crate::store::ModelStore, docs: &mut crate::document::Documents) {
        let open: Vec<u64> = docs.iter().map(|d| d.id()).collect();
        self.panels.retain(|id, _| open.contains(id));
        for (id, stem, export) in std::mem::take(&mut self.exports) {
            let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else { continue };
            let made = match export {
                Export::DrawingPdf => doc.engine.export_document_pdf().map(|bytes| (format!("{stem}.pdf"), bytes)),
                Export::Step => doc.engine.export_step_text_named(&stem).map(|text| (format!("{stem}.step"), text.into_bytes())),
            };
            match (made, self.panels.get_mut(&id)) {
                (Ok((name, bytes)), Some(panel)) => {
                    panel.stage(Upload::of(&name, bytes));
                    self.status = None;
                }
                (Ok(_), None) => {}
                // The engine's own sentence: "the document has no sheets".
                (Err(error), _) => self.status = Some(format!("could not export: {error}")),
            }
        }
        for (attachment, bytes) in std::mem::take(&mut self.downloads) {
            self.status = Some(match store.export_file_named_bytes(&attachment.name, &bytes) {
                Ok(()) => format!("saved {} ({})", attachment.name, human_size(bytes.len() as u64)),
                Err(error) => format!("could not save {}: {error}", attachment.name),
            });
        }
    }

    /// Draw the section for `doc` (the active document, if it is a PLM
    /// revision). `put` publishes a widget's rect under the section's prefix.
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        doc: Option<SectionDoc<'_>>,
        plm: &dyn PartAttachments,
        put: &mut dyn FnMut(&str, egui::Rect),
    ) {
        let Some(doc) = doc else {
            ui.label("This document is not from the PLM.");
            return;
        };
        let panel = self.panels.entry(doc.id).or_insert_with(|| PlmAttachmentsPanel::new(doc.part, doc.revision));
        panel.can_pick_files = self.can_pick_files;
        let mut export = None;
        ui.horizontal(|ui| {
            let busy = panel.busy() || panel.staged.is_some();
            let pdf = ui.add_enabled(!busy, egui::Button::new("Attach drawing (PDF)"))
                .on_hover_text("This document's drawing sheets as one PDF, onto this revision");
            put("export:pdf", pdf.rect);
            if pdf.clicked() {
                export = Some(Export::DrawingPdf);
            }
            let step = ui.add_enabled(!busy, egui::Button::new("Attach STEP"))
                .on_hover_text("This document's model as STEP, onto this revision");
            put("export:step", step.rect);
            if step.clicked() {
                export = Some(Export::Step);
            }
        });
        if let Some(status) = &self.status {
            ui.weak(status);
        }
        let outcome = panel.show(ui, plm);
        for (key, rect) in &panel.hits {
            put(key.strip_prefix("plm_attachments:").unwrap_or(key), *rect);
        }
        if let Some(export) = export {
            self.exports.push((doc.id, doc.stem.clone(), export));
        }
        if let Some(download) = outcome.downloaded {
            self.downloads.push(download);
        }
        if outcome.pick_file {
            self.asked_pick = Some(doc.id);
        }
    }

    /// The documents that have a panel.
    pub fn panel_ids(&self) -> Vec<u64> {
        self.panels.keys().copied().collect()
    }

    /// The document whose "Attach a file…" was pressed, once.
    pub fn take_pick_request(&mut self) -> Option<u64> {
        self.asked_pick.take()
    }

    /// The file the chooser delivered for document `id`: staged on its panel,
    /// where the user confirms its kind, note and target as for any file.
    pub fn stage_picked(&mut self, id: u64, name: &str, bytes: Vec<u8>) {
        if let Some(panel) = self.panels.get_mut(&id) {
            panel.stage(Upload::of(name, bytes));
        }
    }
}

