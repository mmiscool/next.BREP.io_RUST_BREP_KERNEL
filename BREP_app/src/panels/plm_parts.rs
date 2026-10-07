//! PLM part creation and the catalog browser (plm-cad-integration-todo §3 S7).
//!
//! Three things a user does with a PLM's parts from the CAD app, none of which
//! the file stores have:
//!
//! * **New part.** The part type decides how its number is made
//!   ([`NumberMode`]): a counter hands one out and refuses a typed one; a free
//!   type takes whatever is typed; a pattern type takes a typed number that
//!   matches its regex; a script type hands a typed number (or none) to the
//!   administrator's `partNumber` script. The server is the judge in every mode.
//!   This dialog asks for what the mode needs and shows the server's refusal
//!   word for word. The document class comes from the extension
//!   (`.nbrep` / `.fbrep` / `.tbrep`), as it does on the file system, or from
//!   the chooser.
//! * **The catalog browser** for insertion: the category tree with its
//!   counts, a text search, and pages of [`PAGE`] parts behind **Load more**.
//!   A catalog of tens of thousands of parts is never asked for whole.
//!   Attribute filters are server prerequisite P7 (`attr.<key>=`, `.min`,
//!   `.max`, `include=attributes`). The filter row is built to that contract,
//!   and hidden until [`PartCatalog::attribute_filters`] says the server has
//!   it.
//! * **Save As** on a PLM store (D9) asks: a new part, or a new revision of this
//!   part ([`SaveAsChooser`]). The file panel acts on the answer.
//!
//! Everything talks to the server through [`PartCatalog`], which S1's client
//! implements. The panel holds each request's future and polls it every frame
//! with a waker that repaints, so it needs nothing from the host's executor.
//! Nothing here is constructed without a server: the file-based app never
//! reaches this panel.

use crate::document_class::DocumentClass;
use crate::plm::PlmFuture;
use eframe::egui;
use std::collections::HashMap;
use std::task::{Context, Poll, Waker};

/// Parts per catalog page. The server clamps to its own maximum.
pub const PAGE: usize = 100;

// --- the wire ------------------------------------------------------------------
//
// These mirror the server's answers (`BREP_plm/src/api/{accounts,catalog,
// parts}.rs`) and are pinned against the REAL router by the golden test
// below. Every field the app does not need is left out; every field it reads
// defaults, so a server adding one breaks nothing.

/// How a part type's numbers are made — `mode` on `GET /api/part-types`.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum NumberMode {
    #[default]
    Counter,
    Free,
    Pattern {
        #[serde(default)]
        regex: String,
    },
    Script {
        #[serde(default)]
        script: String,
    },
}

/// One row of `GET /api/part-types`.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct PartType {
    pub id: String,
    pub name: String,
    /// What a counter type hands out next (`CPART000000042`).
    #[serde(default)]
    pub next_number: String,
    #[serde(default)]
    pub mode: NumberMode,
}

/// An attribute a category defines — the category's own list on
/// `GET /api/categories`, or its effective schema (with `from`) on
/// `GET /api/categories/:id/schema`.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Attribute {
    pub key: String,
    pub name: String,
    /// `text`, `number`, `bool` or `enum`.
    #[serde(rename = "type", default)]
    pub kind: String,
    #[serde(default)]
    pub unit: String,
    #[serde(default)]
    pub values: Vec<String>,
    #[serde(default)]
    pub required: bool,
    /// The category that defines it (the schema route only).
    #[serde(default)]
    pub from: String,
}

/// One row of `GET /api/categories`: the whole tree, depth-first, siblings by
/// name — the order it is drawn in.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Category {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub parent: String,
    /// `Fasteners / Screws`.
    #[serde(default)]
    pub path: String,
    #[serde(default)]
    pub depth: usize,
    /// Parts in this category and every one beneath it.
    #[serde(default)]
    pub parts_within: usize,
    #[serde(default)]
    pub attributes: Vec<Attribute>,
}

/// `GET /api/categories/:id/schema`.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct Schema {
    pub category: String,
    #[serde(default)]
    pub path: String,
    #[serde(default)]
    pub attributes: Vec<Attribute>,
}

/// One part in a list (`PartRow` on the server).
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct PartRow {
    pub id: String,
    pub number: String,
    pub name: String,
    #[serde(default)]
    pub thumbnail_url: String,
    #[serde(default)]
    pub part_type: String,
    /// `normal`, `family` or `template`.
    #[serde(default)]
    pub document_class: String,
    #[serde(default)]
    pub category_path: String,
    #[serde(default)]
    pub revisions: usize,
    #[serde(default)]
    pub latest_label: String,
    #[serde(default)]
    pub latest_state: String,
    #[serde(default)]
    pub locked: bool,
    /// The newest revision, drafts included (P8): what Insert places (D13).
    #[serde(default)]
    pub latest_revision_id: String,
    /// The part's stored catalog values, when the page was asked with
    /// `include=attributes` (P7); empty otherwise.
    #[serde(default)]
    pub attributes: serde_json::Map<String, serde_json::Value>,
}

/// `GET /api/parts?limit=&after=`: a page and the cursor after it, `None` on
/// the last page.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct PartsPage {
    pub parts: Vec<PartRow>,
    pub next: Option<String>,
}

/// A revision as the create routes answer it.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct CreatedRevision {
    pub id: String,
    pub label: String,
}

/// A part as `POST /api/parts` answers it.
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct CreatedPart {
    pub id: String,
    pub number: String,
    pub name: String,
    #[serde(default)]
    pub revisions: Vec<CreatedRevision>,
}

impl CreatedPart {
    /// The store key of its first revision — where its document goes.
    pub fn first_revision_key(&self) -> Option<String> {
        self.revisions.first().map(|revision| {
            crate::store::DocumentIdentity::Revision {
                part: self.id.clone(),
                revision: revision.id.clone(),
            }
            .key()
        })
    }
}

/// `POST /api/parts`'s body.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize)]
pub struct NewPartRequest {
    pub part_type: String,
    pub name: String,
    pub description: String,
    pub category: String,
    /// Empty for a counter type (which refuses one) and for a script type
    /// asked to allocate.
    pub number: String,
    /// `normal`, `family` or `template`.
    pub document_class: String,
    /// The first revision's label; empty takes the server's (`A`).
    pub label: String,
    /// The user's workspace folder the SERVER links the new part into
    /// (empty: the top). The app never links a new part itself.
    pub workspace_folder: String,
}

/// How an attribute filter compares (server prerequisite P7).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Bound {
    /// `attr.<key>=<value>`: the stored value equals it, read as the schema
    /// reads form text. The same key twice is ANY of them.
    #[default]
    Equals,
    /// `attr.<key>.min=<n>`, inclusive; number attributes only.
    Min,
    /// `attr.<key>.max=<n>`, inclusive; number attributes only.
    Max,
}

/// One attribute filter — sent only to a server that takes them (P7).
/// Different keys are ALL of them; a part with no value never matches.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct AttributeFilter {
    pub key: String,
    pub bound: Bound,
    pub value: String,
}

/// What a catalog page asks for.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PartsQuery {
    pub q: String,
    /// A category id (it and everything beneath it); empty for all.
    pub category: String,
    pub after: Option<String>,
    pub attributes: Vec<AttributeFilter>,
    /// Ask for each row's catalog values (`include=attributes`, P7).
    pub include_attributes: bool,
}

impl PartsQuery {
    /// The query string for `GET /api/parts`, always paged.
    pub fn to_query(&self) -> String {
        let mut pairs = vec![format!("limit={PAGE}")];
        if !self.q.trim().is_empty() {
            pairs.push(format!("q={}", encode(self.q.trim())));
        }
        if !self.category.is_empty() {
            pairs.push(format!("category={}", encode(&self.category)));
        }
        if let Some(after) = &self.after {
            pairs.push(format!("after={}", encode(after)));
        }
        if self.include_attributes {
            pairs.push("include=attributes".into());
        }
        for filter in self.attributes.iter().filter(|f| !f.value.trim().is_empty()) {
            let suffix = match filter.bound {
                Bound::Equals => "",
                Bound::Min => ".min",
                Bound::Max => ".max",
            };
            pairs.push(format!("attr.{}{suffix}={}", encode(&filter.key), encode(filter.value.trim())));
        }
        pairs.join("&")
    }
}

/// Percent-encode everything but the unreserved characters (RFC 3986).
fn encode(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for byte in text.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'.' | b'_' | b'~' => out.push(byte as char),
            _ => out.push_str(&format!("%{byte:02X}")),
        }
    }
    out
}

/// The PLM's part and catalog routes, as this panel needs them. S1's client
/// implements it; the tests implement it over the real router and as a fake.
/// An `Err` is the sentence to show: the server's `{"error": …}` message, or
/// why no answer came.
pub trait PartCatalog {
    fn thumbnail(&self, _url: &str) -> PlmFuture<Option<Vec<u8>>> {
        Box::pin(async { Ok(None) })
    }
    fn part_types(&self) -> PlmFuture<Vec<PartType>>;
    fn categories(&self) -> PlmFuture<Vec<Category>>;
    fn schema(&self, category: &str) -> PlmFuture<Schema>;
    fn parts_page(&self, query: &PartsQuery) -> PlmFuture<PartsPage>;
    /// Refresh the visible extent of the catalog without dropping its rows.
    fn refresh_pages(&self, query: &PartsQuery, _minimum_rows: usize) -> PlmFuture<PartsPage> {
        self.parts_page(query)
    }
    fn create_part(&self, request: &NewPartRequest) -> PlmFuture<CreatedPart>;
    /// `POST /api/parts/:id/revisions` — Save As's "new revision of this part".
    fn create_revision(&self, part: &str, label: &str) -> PlmFuture<CreatedRevision>;
    /// Whether `GET /api/parts` takes attribute filters and
    /// `include=attributes` (server prerequisite P7). `false` hides the filter
    /// row; a server before P7 would ignore the parameters and answer
    /// unfiltered, which would look like a filter that matched everything.
    fn attribute_filters(&self) -> bool {
        false
    }
}

/// [`PartCatalog`] over S1's [`PlmClient`](crate::plm::client::PlmClient):
/// the routes, their JSON, and the server's refusal sentence as the error.
///
/// `attribute_filters` is whether this server has P7. The wiring sets it from
/// what the server says it supports; nothing is guessed from a failed call.
pub struct PlmPartCatalog {
    pub client: std::rc::Rc<crate::plm::client::PlmClient>,
    pub attribute_filters: bool,
}

impl PlmPartCatalog {
    /// Over a signed-in client, with attribute filters on exactly when the
    /// server says it has them (`features` on `/api/me`, P7).
    pub fn new(client: std::rc::Rc<crate::plm::client::PlmClient>) -> Self {
        let attribute_filters = client.me().is_some_and(|me| me.has("attribute-filters"));
        Self { client, attribute_filters }
    }

    fn json<T: serde::de::DeserializeOwned + 'static>(
        &self,
        method: &'static str,
        path: String,
        body: Option<Vec<u8>>,
    ) -> PlmFuture<T> {
        let client = self.client.clone();
        Box::pin(async move {
            let response = client.call(method, &path, body).await.map_err(|error| error.to_string())?;
            serde_json::from_slice(&response.body)
                .map_err(|error| format!("the PLM answered {path} with something this app cannot read: {error}"))
        })
    }
}

impl PartCatalog for PlmPartCatalog {
    fn thumbnail(&self, url: &str) -> PlmFuture<Option<Vec<u8>>> {
        crate::plm::thumbnail_view::fetch(self.client.clone(), url.to_string())
    }
    fn part_types(&self) -> PlmFuture<Vec<PartType>> {
        self.json("GET", "/api/part-types".into(), None)
    }
    fn categories(&self) -> PlmFuture<Vec<Category>> {
        self.json("GET", "/api/categories".into(), None)
    }
    fn schema(&self, category: &str) -> PlmFuture<Schema> {
        self.json("GET", format!("/api/categories/{}/schema", encode(category)), None)
    }
    fn parts_page(&self, query: &PartsQuery) -> PlmFuture<PartsPage> {
        self.json("GET", format!("/api/parts?{}", query.to_query()), None)
    }
    fn refresh_pages(&self, query: &PartsQuery, minimum_rows: usize) -> PlmFuture<PartsPage> {
        let client = self.client.clone();
        let mut query = query.clone();
        query.after = None;
        Box::pin(async move {
            let mut result = PartsPage::default();
            loop {
                let response = client.call("GET", &format!("/api/parts?{}", query.to_query()), None)
                    .await.map_err(|e| e.to_string())?;
                let page: PartsPage = serde_json::from_slice(&response.body).map_err(|e| e.to_string())?;
                result.parts.extend(page.parts);
                result.next = page.next;
                if result.parts.len() >= minimum_rows || result.next.is_none() { break; }
                if result.next == query.after { return Err("PLM catalog pagination did not advance".into()); }
                query.after = result.next.clone();
            }
            Ok(result)
        })
    }
    fn create_part(&self, request: &NewPartRequest) -> PlmFuture<CreatedPart> {
        self.json("POST", "/api/parts".into(), serde_json::to_vec(request).ok())
    }
    fn create_revision(&self, part: &str, label: &str) -> PlmFuture<CreatedRevision> {
        let body = serde_json::to_vec(&serde_json::json!({ "label": label })).ok();
        self.json("POST", format!("/api/parts/{}/revisions", encode(part)), body)
    }
    fn attribute_filters(&self) -> bool {
        self.attribute_filters
    }
}

/// Save As → "a new revision of this part" (D9): a new revision of `part`
/// holding `document` — the ACTIVE document, current edits included (S3's
/// New revision verb starts from the released document instead). Creates the
/// revision (`label` empty takes the server's suggestion), checks it OUT to
/// this client, and writes the document, leaving it checked out so the
/// revision opens editable with the lock already this user's. Returns its
/// store key. The server's refusal (another draft open, the author group) is
/// its sentence.
pub async fn save_as_new_revision(
    client: &crate::plm::client::PlmClient,
    part: &str,
    label: &str,
    document: &str,
) -> Result<String, String> {
    let body = |value: serde_json::Value| Some(serde_json::to_vec(&value).unwrap_or_default());
    let created = client
        .call("POST", &format!("/api/parts/{}/revisions", encode(part)), body(serde_json::json!({ "label": label })))
        .await
        .map_err(|error| error.to_string())?;
    let revision: CreatedRevision =
        serde_json::from_slice(&created.body).map_err(|error| format!("the new revision's answer: {error}"))?;
    let base = format!("/api/parts/{}/revisions/{}", encode(part), encode(&revision.id));
    client
        .call("POST", &format!("{base}/checkout"), body(serde_json::json!({ "client_id": "brep-app" })))
        .await
        .map_err(|error| error.to_string())?;
    let key = crate::plm::identity::document_key(part, &revision.id);
    client
        .call("PUT", &format!("/api/store/doc/{key}"), Some(document.as_bytes().to_vec()))
        .await
        .map_err(|error| format!("revision {} was created and checked out, but the document was not written: {error}", revision.label))?;
    Ok(key)
}

/// One request in flight: polled every frame until it answers.
pub struct Pending<T>(Option<PlmFuture<T>>);

impl<T> Pending<T> {
    pub fn new(future: PlmFuture<T>) -> Self {
        Self(Some(future))
    }

    /// The answer, once — `None` while it is still out (or after it was
    /// taken). `waker` is what a transport wakes when it has one: the panel's
    /// repaint, so an answer is drawn the frame it arrives.
    pub fn poll(&mut self, waker: &Waker) -> Option<Result<T, String>> {
        let future = self.0.as_mut()?;
        match future.as_mut().poll(&mut Context::from_waker(waker)) {
            Poll::Ready(outcome) => {
                self.0 = None;
                Some(outcome)
            }
            Poll::Pending => None,
        }
    }

    pub fn is_open(&self) -> bool {
        self.0.is_some()
    }
}

// --- New part --------------------------------------------------------------------

/// What the number field is for this part type.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NumberField {
    /// The counter hands one out; a typed number would be refused. `preview`
    /// is the next one.
    Allocated { preview: String },
    /// A number must be typed. `hint` says what the server will accept.
    Required { hint: String },
    /// The script may allocate one; a typed number is handed to it to judge.
    Optional { hint: String },
}

impl NumberField {
    pub fn of(part_type: &PartType) -> Self {
        match &part_type.mode {
            NumberMode::Counter => NumberField::Allocated {
                preview: part_type.next_number.clone(),
            },
            NumberMode::Free => NumberField::Required {
                hint: "any number, unique across the PLM".into(),
            },
            NumberMode::Pattern { regex } => NumberField::Required {
                hint: format!("must match {regex}"),
            },
            NumberMode::Script { .. } => NumberField::Optional {
                hint: "leave empty to have the numbering script assign one".into(),
            },
        }
    }
}

/// The New part dialog's fields.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NewPartForm {
    pub part_type: String,
    pub number: String,
    /// The name; a class extension on it (`bolts.fbrep`) sets the class.
    pub name: String,
    pub description: String,
    pub category: String,
    pub class: DocumentClass,
    pub label: String,
}

impl NewPartForm {
    /// The class the part will have: the name's extension when it carries
    /// one — the rule the file system follows — otherwise the chooser's.
    pub fn effective_class(&self) -> DocumentClass {
        DocumentClass::of_name(self.name.trim()).unwrap_or(self.class)
    }

    /// The request to send, or the sentence that says what is missing. Only
    /// what the dialog can know is checked here; the server checks the rest
    /// (uniqueness, the pattern, the script) and its refusal is shown as sent.
    pub fn request(&self, types: &[PartType]) -> Result<NewPartRequest, String> {
        let part_type = types
            .iter()
            .find(|t| t.id == self.part_type)
            .ok_or("choose a part type")?;
        let name = crate::document_class::strip_class_extension(self.name.trim())
            .trim()
            .to_string();
        if name.is_empty() {
            return Err("enter a name".into());
        }
        let number = self.number.trim().to_string();
        let number = match NumberField::of(part_type) {
            // The field is disabled, so this is only reachable by a script;
            // the server would refuse it with a less useful sentence.
            NumberField::Allocated { .. } if !number.is_empty() => {
                return Err(format!(
                    "{} numbers are assigned by its counter — leave the number empty",
                    part_type.name
                ))
            }
            NumberField::Allocated { .. } => String::new(),
            NumberField::Required { .. } if number.is_empty() => {
                return Err(format!("{} parts need a number — type one", part_type.name))
            }
            _ => number,
        };
        Ok(NewPartRequest {
            part_type: part_type.id.clone(),
            name,
            description: self.description.trim().to_string(),
            category: self.category.clone(),
            number,
            document_class: self.effective_class().slug().to_string(),
            label: self.label.trim().to_string(),
            workspace_folder: String::new(),
        })
    }
}

// --- the catalog browser ---------------------------------------------------------

/// The catalog browser's state: the tree, the query, the rows loaded so far,
/// and the cursor to the rest.
#[derive(Default)]
pub struct CatalogBrowser {
    pub(crate) thumbnails: crate::plm::thumbnail_view::ThumbnailCache,
    pub categories: Vec<Category>,
    pub query: PartsQuery,
    pub rows: Vec<PartRow>,
    /// `Some` while there is another page.
    pub next: Option<String>,
    pub selected: Option<String>,
    pub error: Option<String>,
    page: Option<Pending<PartsPage>>,
    tree: Option<Pending<Vec<Category>>>,
    /// The query the open page request belongs to, so an answer to a query
    /// the user has since changed is dropped instead of shown.
    asked: Option<PartsQuery>,
    refresh: Option<(PartsQuery, Pending<PartsPage>)>,
}

impl CatalogBrowser {
    /// Start over: the tree, and the first page of the current query.
    pub fn reload(&mut self, catalog: &dyn PartCatalog) {
        self.tree = Some(Pending::new(catalog.categories()));
        self.search(catalog);
    }

    /// Keep rows, selection, pagination and versioned textures visible while following changes.
    pub fn refresh(&mut self, catalog: &dyn PartCatalog) {
        if self.loading() || self.refresh.is_some() { return; }
        self.tree = Some(Pending::new(catalog.categories()));
        self.refresh = Some((self.query.clone(), Pending::new(catalog.refresh_pages(&self.query, self.rows.len().max(PAGE)))));
    }

    /// The first page of the current query; the rows so far are dropped.
    pub fn search(&mut self, catalog: &dyn PartCatalog) {
        self.refresh = None;
        self.rows.clear();
        self.next = None;
        self.error = None;
        self.query.after = None;
        self.ask(catalog);
    }

    /// The next page, appended. Nothing when there is none or one is out.
    pub fn load_more(&mut self, catalog: &dyn PartCatalog) {
        if self.loading() || self.refresh.is_some() || self.next.is_none() {
            return;
        }
        self.query.after = self.next.clone();
        self.ask(catalog);
    }

    fn ask(&mut self, catalog: &dyn PartCatalog) {
        self.page = Some(Pending::new(catalog.parts_page(&self.query)));
        self.asked = Some(self.query.clone());
    }

    pub fn loading(&self) -> bool {
        self.page.as_ref().is_some_and(Pending::is_open)
    }

    /// Fold in whatever has answered. Returns whether anything changed.
    pub fn poll(&mut self, waker: &Waker) -> bool {
        self.thumbnails.poll(waker);
        let mut changed = false;
        if let Some((query, pending)) = &mut self.refresh {
            if let Some(answer) = pending.poll(waker) {
                let mut expected = query.clone(); expected.after = None;
                let mut current = self.query.clone(); current.after = None;
                if expected.to_query() == current.to_query() {
                    match answer {
                        Ok(page) => {
                            if self.rows != page.parts { self.rows = page.parts; changed = true; }
                            self.next = page.next;
                            if self.selected.as_ref().is_some_and(|id| !self.rows.iter().any(|row| &row.id == id)) { self.selected = None; }
                            self.error = None;
                        }
                        Err(error) => self.error = Some(error),
                    }
                }
                self.refresh = None;
            }
        }
        if let Some(outcome) = self.tree.as_mut().and_then(|p| p.poll(waker)) {
            self.tree = None;
            changed = true;
            match outcome {
                Ok(categories) => self.categories = categories,
                Err(error) => self.error = Some(error),
            }
        }
        if let Some(outcome) = self.page.as_mut().and_then(|p| p.poll(waker)) {
            self.page = None;
            changed = true;
            // Stale means a DIFFERENT request, compared as sent: the filter
            // row adds empty filter slots to the query after it was asked,
            // which changes nothing on the wire and must not drop the answer.
            let current = self.asked.take().map(|asked| asked.to_query()) == Some(self.query.to_query());
            match outcome {
                Ok(page) if current => {
                    self.rows.extend(page.parts);
                    self.next = page.next;
                }
                Ok(_) => {}
                Err(error) => self.error = Some(error),
            }
        }
        changed
    }
}

// --- Save As on a PLM store (D9) ----------------------------------------------------

/// What Save As does with the active document on a PLM store.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SaveAsDecision {
    /// A new part (then the New part dialog).
    NewPart,
    /// A new revision of the part the document is a revision of.
    NewRevision { part: String },
}

/// The question Save As asks on a PLM store. A document that is not a
/// revision of a part (never saved to the PLM) can only become a new part,
/// so the question is not asked: the answer is [`SaveAsDecision::NewPart`].
pub struct SaveAsChooser;

impl SaveAsChooser {
    /// The choices for a document with store identity `identity`, read by the
    /// store's own rule (`ModelStore::identity`).
    pub fn choices(identity: Option<&crate::store::DocumentIdentity>) -> Vec<SaveAsDecision> {
        match identity {
            Some(crate::store::DocumentIdentity::Revision { part, .. }) => vec![
                SaveAsDecision::NewPart,
                SaveAsDecision::NewRevision { part: part.clone() },
            ],
            _ => vec![SaveAsDecision::NewPart],
        }
    }

    /// Draw the question; the choice on the frame it is made.
    pub fn show(ui: &mut egui::Ui, choices: &[SaveAsDecision], hits: &mut HashMap<String, egui::Rect>) -> Option<SaveAsDecision> {
        let mut chosen = None;
        ui.label("Save this document as:");
        for choice in choices {
            let (key, text) = match choice {
                SaveAsDecision::NewPart => ("plm_parts:saveas:new_part", "A new part"),
                SaveAsDecision::NewRevision { .. } => ("plm_parts:saveas:new_revision", "A new revision of this part"),
            };
            let button = ui.button(text);
            hits.insert(key.into(), button.rect);
            if button.clicked() {
                chosen = Some(choice.clone());
            }
        }
        chosen
    }
}

// --- the panel ----------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Tab {
    #[default]
    Catalog,
    NewPart,
}

/// What a frame of the panel hands back to the shell.
#[derive(Default, Debug, PartialEq)]
pub struct PlmPartsOutcome {
    /// A catalog part double-clicked for insertion.
    pub insert: Option<PartRow>,
    /// A part the New part dialog created — the shell writes the active
    /// document to [`CreatedPart::first_revision_key`].
    pub created: Option<CreatedPart>,
    /// A catalog part to link into the Workspace section's current folder
    /// (Link to workspace; S14 links it, following the newest revision).
    pub link: Option<PartRow>,
}

/// The PLM parts panel: the catalog browser and the New part dialog.
#[derive(Default)]
pub struct PlmPartsPanel {
    pub tab: Tab,
    pub browser: CatalogBrowser,
    pub form: NewPartForm,
    pub types: Vec<PartType>,
    /// The chosen category's effective schema: the keys a family's table may
    /// use as parameters (read-only here; the table editor is S9's).
    pub schema: Option<Schema>,
    /// The last create's refusal, word for word, or a form problem.
    pub problem: Option<String>,
    /// The folder the Workspace section shows: a new part is linked there,
    /// by the server (the PLM pane sets it before drawing this panel).
    pub workspace_folder: String,
    started: bool,
    types_pending: Option<Pending<Vec<PartType>>>,
    schema_pending: Option<Pending<Schema>>,
    create_pending: Option<Pending<CreatedPart>>,
    pub hits: HashMap<String, egui::Rect>,
}

impl PlmPartsPanel {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn refresh_catalog(&mut self, catalog: &dyn PartCatalog) {
        if self.started {
            self.browser.refresh(catalog);
        }
    }

    /// Fold in every answer that has arrived; `waker` repaints.
    fn poll(&mut self, waker: &Waker, outcome: &mut PlmPartsOutcome) {
        self.browser.poll(waker);
        if let Some(answer) = self.types_pending.as_mut().and_then(|p| p.poll(waker)) {
            self.types_pending = None;
            match answer {
                Ok(types) => {
                    if self.form.part_type.is_empty() {
                        if let Some(first) = types.first() {
                            self.form.part_type = first.id.clone();
                        }
                    }
                    self.types = types;
                }
                Err(error) => self.problem = Some(error),
            }
        }
        if let Some(answer) = self.schema_pending.as_mut().and_then(|p| p.poll(waker)) {
            self.schema_pending = None;
            self.schema = answer.ok();
        }
        if let Some(answer) = self.create_pending.as_mut().and_then(|p| p.poll(waker)) {
            self.create_pending = None;
            match answer {
                Ok(part) => {
                    self.problem = None;
                    self.form = NewPartForm {
                        part_type: self.form.part_type.clone(),
                        category: self.form.category.clone(),
                        ..NewPartForm::default()
                    };
                    outcome.created = Some(part);
                }
                // The server's sentence, as sent: a script's refusal is the
                // administrator's own words.
                Err(error) => self.problem = Some(error),
            }
        }
    }

    /// `{rows, more, category, loading, types, problem, schemaKeys}` — for
    /// scripts, through the PLM pane's state.
    pub fn state_json(&self) -> serde_json::Value {
        serde_json::json!({
            "rows": self.browser.rows.len(),
            "thumbnails": self.browser.rows.iter().filter(|r| self.browser.thumbnails.ready(&r.thumbnail_url)).map(|r| &r.id).collect::<Vec<_>>(),
            "numbers": self.browser.rows.iter().take(5).map(|r| r.number.clone()).collect::<Vec<_>>(),
            "more": self.browser.next.is_some(),
            "category": self.browser.query.category,
            "loading": self.busy(),
            "types": self.types.iter().map(|t| t.id.clone()).collect::<Vec<_>>(),
            "problem": self.problem,
            "schemaKeys": self.schema.as_ref().map(|s| s.attributes.iter().map(|a| a.key.clone()).collect::<Vec<_>>()).unwrap_or_default(),
        })
    }

    /// Whether a request this panel made is still out.
    pub fn busy(&self) -> bool {
        self.browser.loading()
            || self.browser.refresh.is_some()
            || self.browser.thumbnails.busy()
            || self.types_pending.as_ref().is_some_and(Pending::is_open)
            || self.schema_pending.as_ref().is_some_and(Pending::is_open)
            || self.create_pending.as_ref().is_some_and(Pending::is_open)
    }

    fn choose_category(&mut self, catalog: &dyn PartCatalog, id: &str) {
        self.browser.query.category = id.to_string();
        self.browser.query.attributes.clear();
        self.browser.query.include_attributes = catalog.attribute_filters();
        self.browser.search(catalog);
        self.form.category = id.to_string();
        self.schema = None;
        self.schema_pending = (!id.is_empty()).then(|| Pending::new(catalog.schema(id)));
    }

    pub fn show(&mut self, ui: &mut egui::Ui, catalog: &dyn PartCatalog) -> PlmPartsOutcome {
        self.hits.clear();
        let mut outcome = PlmPartsOutcome::default();
        if !self.started {
            self.started = true;
            self.browser.reload(catalog);
            self.types_pending = Some(Pending::new(catalog.part_types()));
        }
        let waker = repaint_waker(ui.ctx());
        self.poll(&waker, &mut outcome);

        ui.horizontal(|ui| {
            for (tab, text, key) in [
                (Tab::Catalog, "Catalog", "plm_parts:tab:catalog"),
                (Tab::NewPart, "New part", "plm_parts:tab:new"),
            ] {
                let button = ui.selectable_label(self.tab == tab, text);
                self.hits.insert(key.into(), button.rect);
                if button.clicked() {
                    self.tab = tab;
                }
            }
        });
        ui.separator();
        match self.tab {
            Tab::Catalog => self.show_catalog(ui, catalog, &mut outcome),
            Tab::NewPart => self.show_new_part(ui, catalog),
        }
        // Poll again: a request issued THIS frame has never been polled, and a
        // future registers its waker only when polled. Without this a real
        // transport's answer would wait for some unrelated repaint.
        let before = (self.browser.rows.len(), self.types.len(), self.problem.clone());
        self.poll(&waker, &mut outcome);
        if (self.browser.rows.len(), self.types.len(), self.problem.clone()) != before {
            ui.ctx().request_repaint();
        }
        outcome
    }

    fn show_catalog(&mut self, ui: &mut egui::Ui, catalog: &dyn PartCatalog, outcome: &mut PlmPartsOutcome) {
        let search = ui.add(egui::TextEdit::singleline(&mut self.browser.query.q).hint_text("Search number, name, MPN…"));
        self.hits.insert("plm_parts:search".into(), search.rect);
        if search.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
            self.browser.search(catalog);
        }
        if catalog.attribute_filters() {
            self.show_filters(ui, catalog);
        }
        // One line across the whole section, always there: selecting a row must not push the rows
        // down — a double click whose first click moved its own target
        // is two single clicks (plm-parts-catalog's Insert, headless).
        let selected = self.browser.selected.as_ref().and_then(|id| self.browser.rows.iter().find(|r| &r.id == id)).cloned();
        ui.horizontal(|ui| {
            let refresh = ui.button("Refresh");
            self.hits.insert("plm_parts:refresh".into(), refresh.rect);
            if refresh.clicked() {
                self.browser.reload(catalog);
            }
            if self.browser.loading() {
                ui.weak("Loading…");
            } else if self.browser.next.is_some() {
                ui.weak(format!("{} parts shown", self.browser.rows.len()));
                let more = ui.add_enabled(self.browser.refresh.is_none(), egui::Button::new("Load more"));
                self.hits.insert("plm_parts:more".into(), more.rect);
                if more.clicked() {
                    self.browser.load_more(catalog);
                }
            } else {
                ui.weak(format!("{} parts", self.browser.rows.len()));
            }
            if let Some(row) = selected {
                let link = ui.button(format!("Link {} to workspace", row.number));
                self.hits.insert("plm_parts:link".into(), link.rect);
                if link.clicked() {
                    outcome.link = Some(row);
                }
            }
        });
        let mut chosen: Option<String> = None;
        ui.columns(2, |columns| {
            let ui = &mut columns[0];
            let all = ui.selectable_label(self.browser.query.category.is_empty(), "All parts");
            self.hits.insert("plm_parts:category:".into(), all.rect);
            if all.clicked() {
                chosen = Some(String::new());
            }
            for category in &self.browser.categories {
                let text = format!(
                    "{}{} ({})",
                    "  ".repeat(category.depth + 1),
                    category.name,
                    category.parts_within
                );
                let row = ui.selectable_label(self.browser.query.category == category.id, text);
                self.hits.insert(format!("plm_parts:category:{}", category.id), row.rect);
                if row.clicked() {
                    chosen = Some(category.id.clone());
                }
            }

            let ui = &mut columns[1];
            // The controls come FIRST, above the rows: the rows are as long as
            // the pages loaded (the pane itself scrolls), and a Load more under
            // 200 rows is a button nobody — and no script — finds. No scroll
            // area of the list's own: a nested one would take the wheel meant
            // for the pane.
            for part in &self.browser.rows {
                ui.push_id(&part.id, |ui| {
                let text = format!(
                    "{}  {}  ({} {})",
                    part.number, part.name, part.latest_label, part.latest_state
                );
                let row = ui.horizontal(|ui| {
                    let preview = self.browser.thumbnails.show(ui, &part.thumbnail_url, |url| catalog.thumbnail(url));
                    ui.add_sized(
                        [ui.available_width(), crate::plm::thumbnail_view::SIDE],
                        egui::Button::selectable(self.browser.selected.as_deref() == Some(&part.id), text)
                            .wrap_mode(egui::TextWrapMode::Truncate),
                    ).union(preview)
                }).inner;
                self.hits.insert(format!("plm_parts:part:{}", part.id), row.rect);
                if row.clicked() {
                    self.browser.selected = Some(part.id.clone());
                }
                if row.double_clicked() {
                    outcome.insert = Some(part.clone());
                }
                });
            }
        });
        if let Some(id) = chosen {
            self.choose_category(catalog, &id);
        }
        if let Some(schema) = &self.schema {
            ui.separator();
            ui.weak(format!("{} — a family's parameters may be these attributes:", schema.path));
            for attribute in &schema.attributes {
                let unit = if attribute.unit.is_empty() { String::new() } else { format!(" [{}]", attribute.unit) };
                ui.weak(format!("  {} ({}){unit}", attribute.key, attribute.name));
            }
        }
        if let Some(error) = &self.browser.error {
            ui.colored_label(ui.visuals().error_fg_color, error);
        }
    }

    /// One field per attribute of the chosen category's schema (a number gets
    /// a min and a max). Typed only against a category, because only there is
    /// a key's type certain (P7 refuses a key whose type the categories
    /// disagree on). Enter searches.
    fn show_filters(&mut self, ui: &mut egui::Ui, catalog: &dyn PartCatalog) {
        let Some(schema) = &self.schema else {
            ui.weak("Choose a category to filter by its attributes.");
            return;
        };
        let mut search = false;
        let wanted: Vec<(String, String, Bound)> = schema
            .attributes
            .iter()
            .flat_map(|attribute| {
                if attribute.kind == "number" {
                    vec![
                        (attribute.key.clone(), format!("{} ≥", attribute.name), Bound::Min),
                        (attribute.key.clone(), format!("{} ≤", attribute.name), Bound::Max),
                    ]
                } else {
                    vec![(attribute.key.clone(), attribute.name.clone(), Bound::Equals)]
                }
            })
            .collect();
        let filters = &mut self.browser.query.attributes;
        filters.retain(|f| wanted.iter().any(|(key, _, bound)| *key == f.key && *bound == f.bound));
        ui.horizontal_wrapped(|ui| {
            for (key, label, bound) in &wanted {
                if !filters.iter().any(|f| f.key == *key && f.bound == *bound) {
                    filters.push(AttributeFilter { key: key.clone(), bound: *bound, value: String::new() });
                }
                let filter = filters.iter_mut().find(|f| f.key == *key && f.bound == *bound).unwrap();
                ui.label(label);
                let field = ui.add(egui::TextEdit::singleline(&mut filter.value).desired_width(80.0));
                let suffix = match bound {
                    Bound::Equals => "",
                    Bound::Min => ".min",
                    Bound::Max => ".max",
                };
                self.hits.insert(format!("plm_parts:filter:{key}{suffix}"), field.rect);
                if field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    search = true;
                }
            }
        });
        if search {
            self.browser.search(catalog);
        }
    }

    fn show_new_part(&mut self, ui: &mut egui::Ui, catalog: &dyn PartCatalog) {
        let current = self.types.iter().find(|t| t.id == self.form.part_type).cloned();
        egui::Grid::new("plm_parts:new").num_columns(2).show(ui, |ui| {
            ui.label("Part type");
            let combo = egui::ComboBox::from_id_salt("plm_parts:new:type")
                .selected_text(current.as_ref().map(|t| t.name.clone()).unwrap_or_default())
                .show_ui(ui, |ui| {
                    for part_type in &self.types {
                        ui.selectable_value(&mut self.form.part_type, part_type.id.clone(), &part_type.name);
                    }
                });
            self.hits.insert("plm_parts:new:type".into(), combo.response.rect);
            ui.end_row();

            ui.label("Number");
            let field = current.as_ref().map(NumberField::of);
            let (enabled, hint) = match &field {
                Some(NumberField::Allocated { preview }) => (false, format!("assigned: {preview}")),
                Some(NumberField::Required { hint }) | Some(NumberField::Optional { hint }) => (true, hint.clone()),
                None => (false, String::new()),
            };
            if !enabled {
                self.form.number.clear();
            }
            let number = ui.add_enabled(
                enabled,
                egui::TextEdit::singleline(&mut self.form.number).hint_text(hint),
            );
            self.hits.insert("plm_parts:new:number".into(), number.rect);
            ui.end_row();

            ui.label("Name");
            let name = ui.text_edit_singleline(&mut self.form.name);
            self.hits.insert("plm_parts:new:name".into(), name.rect);
            ui.end_row();

            ui.label("Document");
            let by_name = DocumentClass::of_name(self.form.name.trim()).is_some();
            ui.add_enabled_ui(!by_name, |ui| {
                ui.horizontal(|ui| {
                    for class in [DocumentClass::Normal, DocumentClass::Family, DocumentClass::Template] {
                        let button = ui.selectable_label(self.form.effective_class() == class, class.label());
                        self.hits.insert(format!("plm_parts:new:class:{}", class.slug()), button.rect);
                        if button.clicked() {
                            self.form.class = class;
                        }
                    }
                });
            });
            ui.end_row();

            ui.label("Category");
            let path = self
                .browser
                .categories
                .iter()
                .find(|c| c.id == self.form.category)
                .map(|c| c.path.clone())
                .unwrap_or_else(|| "uncategorized — choose one in Catalog".into());
            ui.weak(path);
            ui.end_row();

            ui.label("Revision label");
            ui.add(egui::TextEdit::singleline(&mut self.form.label).hint_text("empty: the server's (A)"));
            ui.end_row();
        });
        let busy = self.create_pending.as_ref().is_some_and(Pending::is_open);
        let create = ui.add_enabled(!busy, egui::Button::new(if busy { "Creating…" } else { "Create part" }));
        self.hits.insert("plm_parts:new:create".into(), create.rect);
        if create.clicked() {
            match self.form.request(&self.types) {
                Ok(mut request) => {
                    self.problem = None;
                    request.workspace_folder = self.workspace_folder.clone();
                    self.create_pending = Some(Pending::new(catalog.create_part(&request)));
                }
                Err(problem) => self.problem = Some(problem),
            }
        }
        if let Some(problem) = &self.problem {
            ui.colored_label(ui.visuals().error_fg_color, problem);
        }
    }
}

/// A waker that asks egui for a frame — a transport wakes it when an answer
/// arrives, and the next frame's poll takes it.
fn repaint_waker(ctx: &egui::Context) -> Waker {
    struct Repaint(egui::Context);
    impl std::task::Wake for Repaint {
        fn wake(self: std::sync::Arc<Self>) {
            self.0.request_repaint();
        }
    }
    Waker::from(std::sync::Arc::new(Repaint(ctx.clone())))
}

