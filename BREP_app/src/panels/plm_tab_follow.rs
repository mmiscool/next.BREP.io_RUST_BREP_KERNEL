//! An open tab follows its PLM revision when another client saves it (the
//! S1 gap "a document another client wrote is not reloaded into an open tab").
//!
//! When the change feed names an open tab's revision (`panels::plm_follow`),
//! the revision's document is read. What the tab then does is
//! [`decide`]d from that document, the tab's clean baseline (what it last
//! opened or saved) and whether it holds unsaved edits:
//! * the server holds what the tab last opened or saved (our own save coming
//!   back, a check-in, a release): nothing;
//! * the tab is clean: it takes the server's document, with a notice;
//! * the tab has unsaved edits: the pane asks, **Keep my edits** (the
//!   default: nothing is thrown away unasked) or **Take their version**.
//!
//! A read is only asked, and only acted on, while the store has no write of
//! ours in flight: the feed can name our own key before our write has landed,
//! and the older document it would read must not replace the saved tab.
use crate::document::owned_save::source_hash;
use crate::panels::plm_parts::Pending;
use crate::plm::client::PlmClient;
use serde_json::Value;
use std::collections::HashMap;
use std::rc::Rc;
use std::task::Waker;

/// What a tab does with its revision's document as the server now holds it.
#[derive(Debug, Clone, PartialEq)]
pub enum Follow {
    /// The server holds what the tab last opened or saved.
    Nothing,
    /// Clean tab, newer server document: take it.
    Reload,
    /// Unsaved edits, newer server document: ask.
    Ask,
}

/// The rule (see the module doc). `baseline` is `None` for a tab never saved.
pub fn decide(server: &Value, baseline: Option<&Value>, current: &Value, dirty: bool) -> Follow {
    let theirs = source_hash(server);
    if baseline.is_some_and(|b| source_hash(b) == theirs) || source_hash(current) == theirs {
        return Follow::Nothing;
    }
    if dirty { Follow::Ask } else { Follow::Reload }
}

/// The tabs whose revisions the feed named, and the asks pending an answer.
#[derive(Default)]
pub struct TabFollow {
    reading: HashMap<u64, Pending<Option<Vec<u8>>>>,
    /// Tab id → the server's document, while the pane asks.
    asking: HashMap<u64, String>,
    /// Tabs reloaded since the session began (state, for automation).
    reloaded: u64,
}

impl TabFollow {
    /// The feed moved: read the revision of every open tab it named (every
    /// PLM tab when it could not say which). Not while a write is in flight.
    pub fn moved(&mut self, client: &Rc<PlmClient>, docs: &crate::document::Documents, keys: &[String], stale: bool, writing: bool) {
        if writing {
            return;
        }
        for doc in docs.iter() {
            let Some(key) = doc.name().and_then(crate::panels::plm_import::revision_key_of) else { continue };
            if self.reading.contains_key(&doc.id()) || !(stale || keys.iter().any(|k| crate::panels::plm_import::revision_key_of(k).as_deref() == Some(key.as_str()))) {
                continue;
            }
            let client = client.clone();
            self.reading.insert(
                doc.id(),
                Pending::new(Box::pin(async move { client.get_document(&key).await.map_err(|e| e.to_string()) })),
            );
        }
    }

    /// Apply the reads that answered. `writing`: a write of ours is in flight,
    /// so an answer may predate it and is dropped (the feed names the key
    /// again once the write lands).
    pub fn poll(&mut self, docs: &mut crate::document::Documents, waker: &Waker, writing: bool) {
        let answered: Vec<(u64, Result<Option<Vec<u8>>, String>)> = self
            .reading
            .iter_mut()
            .filter_map(|(id, pending)| pending.poll(waker).map(|answer| (*id, answer)))
            .collect();
        for (id, answer) in answered {
            self.reading.remove(&id);
            let (Ok(Some(bytes)), false) = (answer, writing) else { continue };
            let Ok(text) = String::from_utf8(bytes) else { continue };
            let Ok(server) = serde_json::from_str::<Value>(&text) else { continue };
            let Some(doc) = docs.iter_mut().find(|d| d.id() == id) else { continue };
            let Ok(current) = serde_json::from_str::<Value>(&doc.engine.history_request_json()) else { continue };
            match decide(&server, doc.saved_document().as_ref(), &current, doc.is_dirty()) {
                Follow::Nothing => {}
                Follow::Reload => {
                    if take(doc, &text) {
                        self.reloaded += 1;
                        doc.engine.push_notice(format!("{} was saved by another client: this tab now shows their version", doc.title()));
                    }
                }
                Follow::Ask => {
                    self.asking.insert(id, text);
                }
            }
        }
    }

    /// The ask for tab `id`, if one is open.
    pub fn asking(&self, id: u64) -> bool {
        self.asking.contains_key(&id)
    }

    /// The user's answer for tab `id`: `true` takes their version, `false`
    /// keeps the edits (asked again only when the server moves again).
    pub fn answer(&mut self, docs: &mut crate::document::Documents, id: u64, take_theirs: bool) {
        let Some(text) = self.asking.remove(&id) else { return };
        if !take_theirs {
            return;
        }
        if let Some(doc) = docs.iter_mut().find(|d| d.id() == id) {
            if take(doc, &text) {
                self.reloaded += 1;
                doc.engine.push_notice(format!("{} now shows the other client's version; your edits were discarded", doc.title()));
            }
        }
    }

    pub fn busy(&self) -> bool {
        !self.reading.is_empty()
    }

    pub fn state_json(&self, id: u64) -> Value {
        serde_json::json!({ "reading": self.reading.contains_key(&id), "asking": self.asking(id), "reloaded": self.reloaded })
    }
}

/// Replace the tab's document with `text` and take it as the clean baseline.
fn take(doc: &mut crate::document::Document, text: &str) -> bool {
    match doc.engine.set_history_json(text) {
        Ok(_) => {
            doc.mark_clean();
            true
        }
        Err(error) => {
            doc.engine.push_notice(format!("{} changed on the PLM, but this tab could not take it: {error}", doc.title()));
            false
        }
    }
}

