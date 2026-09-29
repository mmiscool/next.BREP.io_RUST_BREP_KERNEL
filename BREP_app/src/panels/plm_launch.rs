//! "Open in CAD" from the PLM's web page: the hosted CAD app started with
//! `?open=part/<part>/rev/<revision>` opens that revision.
//!
//! The PLM page opens `/cad/app/web/index.html?open=<key>` in a new tab. By the
//! time the app's first frame runs, the web boot has already signed in with the
//! page's PLM session cookie and hydrated the store's index
//! (`plm::backend::open_web_session`). [`Launch`] then opens the key through the
//! one door every open lane uses (`FileDialog::open_document`, under the store's
//! own spelling of the key), so S3's PLM pane applies: read-only until Check out.
//!
//! * **A revision with a document** opens once its bytes are resident.
//! * **A revision with no document yet** is not in the index. The server is asked
//!   about the part; if the revision is there, a new empty document of the part's
//!   class opens at that key.
//! * **An unknown or forbidden key** shows the server's own sentence.
//! * **A hosted page that is not signed in** opens Settings on the PLM tab. Its
//!   sign-in reloads the page, and the query carries the key through the reload.
//! * **Anywhere else** (the native app, the public site) says the link needs the
//!   app the PLM hosts.
//!
//! Once the open is settled, the address bar loses `?open=`
//! (`history.replaceState`), so reloading the tab does not reopen the key on
//! top of the user's work. Only the sign-in case keeps it.

use crate::document::{Document, Documents, EMPTY_DOCUMENT};
use crate::panels::file::FileDialog;
use crate::plm::PlmFuture;
use crate::store::{ModelStore, Residency};

/// A revision key (`part/<part>/rev/<revision>`) from a page's query string
/// (`?open=…`, percent-encoded or not), or `None` when there is none or it is
/// not that shape.
pub fn open_key(search: &str) -> Option<String> {
    let query = search.strip_prefix('?').unwrap_or(search);
    let raw = query.split('&').find_map(|pair| pair.strip_prefix("open="))?;
    let key = decode(raw);
    let parts: Vec<&str> = key.split('/').collect();
    match parts.as_slice() {
        ["part", part, "rev", revision] if !part.is_empty() && !revision.is_empty() => Some(key.clone()),
        _ => None,
    }
}

fn decode(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        let hex = bytes
            .get(i + 1..i + 3)
            .and_then(|h| std::str::from_utf8(h).ok())
            .and_then(|h| u8::from_str_radix(h, 16).ok());
        match (bytes[i], hex) {
            (b'%', Some(byte)) => {
                out.push(byte);
                i += 3;
            }
            (b'+', _) => {
                out.push(b' ');
                i += 1;
            }
            (byte, _) => {
                out.push(byte);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// What one [`Launch::step`] asks of the shell.
#[derive(Debug, PartialEq)]
pub enum Step {
    /// Still waiting: for the document's bytes, or for the server.
    Pending,
    /// Nothing to do (settled earlier).
    Idle,
    /// The revision is open (or was already), under this name.
    Opened(String),
    /// The revision had no document yet: an empty one opened under this name.
    OpenedEmpty(String),
    /// Not signed in on a hosted page: open Settings on the PLM tab and show
    /// this sentence. Keep `?open=`, which the sign-in's reload carries.
    SignIn(String),
    /// Could not open: show this sentence (the server's, where it answered).
    Refused(String),
}

impl Step {
    /// Whether the address bar should drop `?open=` now.
    pub fn settles(&self) -> bool {
        matches!(self, Step::Opened(_) | Step::OpenedEmpty(_) | Step::Refused(_))
    }
}

/// The part as far as an empty-revision open needs it.
#[derive(Debug, Default, serde::Deserialize)]
#[serde(default)]
struct PartCheck {
    number: String,
    document_class: String,
    revisions: Vec<RevisionCheck>,
}

#[derive(Debug, Default, serde::Deserialize)]
#[serde(default)]
struct RevisionCheck {
    id: String,
    label: String,
}

enum State {
    Waiting,
    Checking(PlmFuture<PartCheck>),
    Done,
}

/// One `?open=` request, driven once a frame until it settles.
pub struct Launch {
    pub key: String,
    state: State,
}

impl Launch {
    pub fn new(key: impl Into<String>) -> Self {
        Launch { key: key.into(), state: State::Waiting }
    }

    /// The launch from the page's own address (wasm), if it carries one.
    #[cfg(target_arch = "wasm32")]
    pub fn from_page() -> Option<Self> {
        let search = web_sys::window()?.location().search().ok()?;
        open_key(&search).map(Launch::new)
    }

    pub fn is_done(&self) -> bool {
        matches!(self.state, State::Done)
    }

    /// Advance: open the revision when it can be, and say what the shell must
    /// show when it cannot.
    pub fn step(&mut self, store: &dyn ModelStore, docs: &mut Documents, file: &mut FileDialog) -> Step {
        if self.is_done() {
            return Step::Idle;
        }
        let Some(client) = store.plm_client() else {
            self.state = State::Done;
            return match store.plm_session() {
                Some(_) => Step::SignIn(format!(
                    "Sign in to the PLM to open {} — the page reloads onto it after you sign in.",
                    self.key
                )),
                None => Step::Refused(format!(
                    "{} can be opened only in the CAD app the PLM hosts (its Open in CAD button)",
                    self.key
                )),
            };
        };
        let name = store.canonical_identity(&self.key);
        match store.residency(&name) {
            Residency::Resident => {
                self.state = State::Done;
                file.open_document(docs, store, &name);
                // `open_document` focuses a tab that holds it already, or
                // opens it; either way the document is the active one now.
                return if docs.active().name() == Some(name.as_str()) {
                    Step::Opened(name)
                } else {
                    Step::Refused(format!("{} could not be opened: its document did not load", self.key))
                };
            }
            Residency::OnDisk => {
                // The read asks the mirror for the bytes; the next frames wait.
                let _ = store.read(&name);
                return Step::Pending;
            }
            Residency::Absent => {}
        }
        // Not in the index: no such revision, not ours to read, or a revision
        // with no document yet. The server says which.
        if let State::Waiting = self.state {
            let (part, _) = split(&self.key);
            let path = format!("/api/parts/{part}");
            let client = client.clone();
            self.state = State::Checking(Box::pin(async move {
                let response = client.call("GET", &path, None).await.map_err(|e| e.to_string())?;
                serde_json::from_slice(&response.body).map_err(|e| format!("the PLM's answer was not understood: {e}"))
            }));
        }
        let State::Checking(future) = &mut self.state else { return Step::Pending };
        let mut cx = std::task::Context::from_waker(std::task::Waker::noop());
        let std::task::Poll::Ready(answer) = future.as_mut().poll(&mut cx) else { return Step::Pending };
        self.state = State::Done;
        let (_, revision) = split(&self.key);
        match answer {
            Err(sentence) => Step::Refused(format!("{} could not be opened: {sentence}", self.key)),
            Ok(part) => match part.revisions.iter().find(|r| r.id == revision) {
                None => Step::Refused(format!("{} could not be opened: part {} has no revision {revision}", self.key, part.number)),
                Some(found) => {
                    let mut engine = docs.spawn_engine();
                    if let Err(e) = engine.set_history_json(EMPTY_DOCUMENT) {
                        return Step::Refused(format!("{} could not be opened: {e}", self.key));
                    }
                    let class = match part.document_class.as_str() {
                        "family" => crate::document_class::DocumentClass::Family,
                        "template" => crate::document_class::DocumentClass::Template,
                        _ => crate::document_class::DocumentClass::Normal,
                    };
                    crate::document_class::set_document_class(&mut engine, class);
                    let mut doc = Document::new(engine);
                    doc.set_name(Some(name.clone()));
                    docs.open_document(doc);
                    let _ = found;
                    Step::OpenedEmpty(name)
                }
            },
        }
    }
}

fn split(key: &str) -> (&str, &str) {
    let mut parts = key.split('/');
    let part = parts.nth(1).unwrap_or("");
    let revision = parts.nth(1).unwrap_or("");
    (part, revision)
}

/// Drop `?open=` from the address bar without reloading (wasm).
pub fn forget_query() {
    #[cfg(target_arch = "wasm32")]
    {
        use wasm_bindgen::{JsCast, JsValue};
        let Some(window) = web_sys::window() else { return };
        let location = window.location();
        let (Ok(path), Ok(search), Ok(hash)) = (location.pathname(), location.search(), location.hash()) else { return };
        let kept: Vec<&str> = search
            .trim_start_matches('?')
            .split('&')
            .filter(|pair| !pair.is_empty() && !pair.starts_with("open="))
            .collect();
        let query = if kept.is_empty() { String::new() } else { format!("?{}", kept.join("&")) };
        // `history.replaceState(null, "", url)`, reached by name: web-sys's
        // `History` feature is not enabled for this crate, and one call does
        // not warrant it.
        let url = JsValue::from_str(&format!("{path}{query}{hash}"));
        let Ok(history) = js_sys::Reflect::get(&window, &JsValue::from_str("history")) else { return };
        let Ok(replace) = js_sys::Reflect::get(&history, &JsValue::from_str("replaceState")) else { return };
        if let Ok(replace) = replace.dyn_into::<js_sys::Function>() {
            let _ = replace.call3(&history, &JsValue::NULL, &JsValue::from_str(""), &url);
        }
    }
}

