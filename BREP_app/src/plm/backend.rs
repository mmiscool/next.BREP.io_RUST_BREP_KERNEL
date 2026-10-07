//! The PLM as a store backend (plan S1 part 2): three [`StoreBackend`]s over
//! one signed-in [`PlmClient`], composed by S0's [`KeySpaceRouter`] (D1).
//!
//! - [`DocumentsBackend`] — the store routes. Its index is
//!   `GET /api/store/index`, every document `OnDisk` until read; a revision
//!   with no document yet is not listed, and `204` reads as absent.
//! - [`PreferencesBackend`] — the user's reserved keys on the server (P5),
//!   hydrated whole at sign-in: they are small, and the first frame needs the
//!   settings and the dock layout.
//! - [`RecoveryMirror`] — `@recovery` written to the machine FIRST, then
//!   mirrored to P5 in the background, latest value wins, so a network drop can
//!   never cost unsaved work and a slow server never holds up the next write.
//!
//! **A failed read backs off** (S0 record §6, item 1). Panels read every frame,
//! and the mirror asks the backend again after every failed `OnDisk` load. So
//! the documents backend remembers a failed key and answers from that memory,
//! without the network, until its backoff (1 s, doubling to 60 s) runs out: a
//! server that is down sees one probe per key per window, not one per frame.
//! A success, or a write of the key, clears it.
use super::client::{PlmClient, PlmError};
#[cfg(not(target_arch = "wasm32"))]
use super::PlmFuture;
use crate::store::mirror_store::{BackendFuture, Entry, IndexRow, KeySpaceRouter, MirrorStore, StoreBackend, DIR_PREFIX, PREFIX};
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;
use web_time::{Duration, Instant};

thread_local! {
    /// `@recovery` mirrors currently being sent in the background, across
    /// every session on this thread. A write-behind the mirror's own pending
    /// count cannot see, so the automation layer's idle check asks here too.
    static MIRRORS_IN_FLIGHT: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Whether an `@recovery` mirror is still on its way to the server.
pub(crate) fn mirror_in_flight() -> bool {
    MIRRORS_IN_FLIGHT.with(|n| n.get() > 0)
}

/// The first wait after a failed read, doubled on each further failure.
pub const FIRST_BACKOFF: Duration = Duration::from_secs(1);
/// The longest wait between two probes of a key that keeps failing.
pub const MAX_BACKOFF: Duration = Duration::from_secs(60);

/// Sign in with `config`'s token, run the version check, and compose the
/// session's backend: documents and preferences on the server, `@recovery`
/// on `local` first and mirrored. `Err` is the sentence the boot shows when it
/// falls back to the file stores.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn open_session(
    config: super::config::PlmConfig,
    local: Rc<dyn StoreBackend>,
) -> PlmFuture<Rc<dyn StoreBackend>> {
    Box::pin(async move {
        let client = Rc::new(PlmClient::new(Rc::new(super::transport::EhttpTransport::new(config.url.clone()))));
        let Some(token) = config.token.as_deref() else {
            return Err(PlmError::SignIn(format!("no token for {}: paste one, or sign in with a password", config.url)).to_string());
        };
        let me = client.sign_in_with_token(token).await.map_err(|e| e.to_string())?;
        Ok(compose(client, &config.url, &me.username, local))
    })
}

/// The web lane (D6): a page served by the PLM itself (`/cad/app/…`, P2)
/// signs in with the browser's own PLM session and stores documents and
/// preferences there, with `@recovery` in `local` (IndexedDB) first.
///
/// - `Ok(None)`: not served by a PLM. The page is the file app of today.
/// - `Err(sentence)`: served by a PLM that refused (not signed in, or a
///   version that does not serve this app). The boot falls back to IndexedDB
///   and shows the sentence; the PLM tab offers the sign-in.
#[cfg(target_arch = "wasm32")]
pub(crate) async fn open_web_session(local: Rc<dyn StoreBackend>) -> Result<Option<Rc<dyn StoreBackend>>, String> {
    let Some(window) = web_sys::window() else { return Ok(None) };
    let location = window.location();
    let (Ok(path), Ok(origin)) = (location.pathname(), location.origin()) else { return Ok(None) };
    if !served_by_plm(&path) {
        return Ok(None);
    }
    let client = Rc::new(PlmClient::new(Rc::new(super::transport::EhttpTransport::new(origin.clone()))));
    let me = client.sign_in_with_browser_session().await.map_err(|e| e.to_string())?;
    Ok(Some(compose(client, &origin, &me.username, local)))
}

/// Whether a page at `path` is the CAD app the PLM hosts (P2 serves it under
/// `/cad/app/`).
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))] // the web boot's
pub(crate) fn served_by_plm(path: &str) -> bool {
    path.starts_with("/cad/app/")
}

/// The three backends over one signed-in client.
/// Every save of an assembly revision republishes its uses list (S5's
/// publisher on the [`SavedHook`]), and every save uploads the picture staged
/// for it ([`crate::plm::thumbnail`]), after the list.
pub(crate) fn compose(client: Rc<PlmClient>, url: &str, username: &str, local: Rc<dyn StoreBackend>) -> Rc<dyn StoreBackend> {
    let on_saved = crate::plm::thumbnail::chain(crate::plm::uses::publish_on_save(), crate::plm::thumbnail::upload_on_save());
    compose_with(client, url, username, local, Some(on_saved))
}

/// [`compose`], with a hook run after every successful document write (see
/// [`SavedHook`]).
pub(crate) fn compose_with(
    client: Rc<PlmClient>,
    url: &str,
    username: &str,
    local: Rc<dyn StoreBackend>,
    on_saved: Option<SavedHook>,
) -> Rc<dyn StoreBackend> {
    let label = format!("PLM {url} as {username}");
    let preferences = Rc::new(PreferencesBackend { client: client.clone() });
    let mut documents = DocumentsBackend::new(client, label);
    documents.on_saved = on_saved;
    Rc::new(KeySpaceRouter {
        documents: Rc::new(documents),
        preferences: preferences.clone(),
        local: Rc::new(RecoveryMirror::new(local, preferences)),
    })
}

/// The store key of a mirror key (`brep-app:model:part/…/rev/…` →
/// `part/…/rev/…`).
fn document_key(key: &str) -> Result<&str, String> {
    if key.starts_with(DIR_PREFIX) {
        return Err("folders are not stored on the PLM; your folders are your workspace".into());
    }
    key.strip_prefix(PREFIX).ok_or_else(|| format!("`{key}` is not a document key"))
}

fn text(bytes: Vec<u8>, key: &str) -> Result<String, String> {
    String::from_utf8(bytes).map_err(|_| format!("{key}: the server's document is not UTF-8 text"))
}

/// Per-key memory of failed reads.
#[derive(Default)]
struct Backoff {
    failed: BTreeMap<String, (Instant, u32, String)>,
}

impl Backoff {
    /// The remembered failure, while its window is still open.
    fn holding(&self, key: &str, now: Instant) -> Option<String> {
        let (at, failures, why) = self.failed.get(key)?;
        (now < *at + window(*failures)).then(|| why.clone())
    }

    fn fail(&mut self, key: &str, now: Instant, why: &str) {
        let failures = self.failed.get(key).map_or(0, |(_, n, _)| *n) + 1;
        self.failed.insert(key.to_string(), (now, failures, why.to_string()));
    }

    fn clear(&mut self, key: &str) {
        self.failed.remove(key);
    }
}

/// How long a key that has failed `failures` times is left alone.
fn window(failures: u32) -> Duration {
    let doublings = failures.saturating_sub(1).min(16);
    (FIRST_BACKOFF * 2u32.pow(doublings)).min(MAX_BACKOFF)
}

/// Run after a document write the server ACCEPTED, with the signed-in
/// client, the store key (`part/…/rev/…`) and the bytes written: S5 hangs
/// the uses-list publisher here, which the plan wants right after the
/// document write and under the same checkout. It runs inside the write's
/// own future, so the mirror's per-key queue holds the key's next write
/// until it finishes. Its `Err` becomes the write's error, so its sentence
/// should say the document itself was saved.
pub(crate) type SavedHook = Rc<dyn Fn(Rc<PlmClient>, String, String) -> BackendFuture<()>>;

/// The store routes as a [`StoreBackend`].
pub(crate) struct DocumentsBackend {
    client: Rc<PlmClient>,
    label: String,
    backoff: Rc<RefCell<Backoff>>,
    /// The clock, replaceable so a test can move time.
    now: Rc<dyn Fn() -> Instant>,
    on_saved: Option<SavedHook>,
    /// What the store index said of each revision key, as last read: the
    /// BOM's part number, revision and lifecycle for a component, with no
    /// request per row ([`crate::store::ModelStore::plm_revision`]).
    index: Rc<RefCell<OrderedIndex>>,
}

impl DocumentsBackend {
    pub(crate) fn new(client: Rc<PlmClient>, label: String) -> Self {
        Self { client, label, backoff: Rc::default(), now: Rc::new(Instant::now), on_saved: None, index: Rc::default() }
    }
}

/// Metadata has its own response ordering, before the mirror sees an answer.
/// A full answer establishes absence even for keys never observed before.
#[derive(Default)]
struct OrderedIndex {
    rows: BTreeMap<String, super::client::IndexEntry>,
    clock: u64,
    freshness: BTreeMap<String, u64>,
    full_authority: u64,
}
impl OrderedIndex {
    fn issue(&mut self) -> u64 {
        self.clock += 1;
        self.clock
    }
    fn reconcile(&mut self, asked: Option<&[String]>, answer: &super::client::StoreIndex, issued: u64) {
        let listed: std::collections::BTreeSet<&str> = answer.entries.iter().map(|e| e.key.as_str()).collect();
        let gone: Vec<String> = match asked {
            Some(keys) => keys.iter().filter(|key| !listed.contains(key.as_str())).cloned().collect(),
            None => self.rows.keys().filter(|key| !listed.contains(key.as_str())).cloned().collect(),
        };
        for key in gone {
            if self.full_authority >= issued || self.freshness.get(&key).is_some_and(|&tick| tick >= issued) { continue; }
            self.freshness.insert(key.clone(), issued);
            self.rows.remove(&key);
        }
        for row in &answer.entries {
            if self.full_authority >= issued || self.freshness.get(&row.key).is_some_and(|&tick| tick >= issued) { continue; }
            self.freshness.insert(row.key.clone(), issued);
            self.rows.insert(row.key.clone(), row.clone());
        }
        if asked.is_none() { self.full_authority = self.full_authority.max(issued); }
    }
}

/// How many keys one delta read asks for (a URL stays short).
pub(crate) const INDEX_ROWS_PER_READ: usize = 100;

impl StoreBackend for DocumentsBackend {
    fn label(&self) -> String {
        self.label.clone()
    }

    fn plm_client(&self) -> Option<Rc<PlmClient>> {
        Some(self.client.clone())
    }

    fn plm_revision(&self, key: &str) -> Option<super::client::IndexEntry> {
        self.index.borrow().rows.get(key).cloned()
    }

    /// The change feed moved: re-read exactly the rows it named (a revision's
    /// lock and lifecycle changes name its keys, server `touch_part`), in
    /// reads of [`INDEX_ROWS_PER_READ`] keys; the whole index only when the
    /// feed was `stale`. A row the answer leaves out is a revision gone.
    fn refresh_plm_index(&self, keys: &[String], stale: bool) {
        if !stale && keys.is_empty() {
            return;
        }
        let issued = self.index.borrow_mut().issue();
        let (client, index) = (self.client.clone(), self.index.clone());
        let keys = keys.to_vec();
        detach(async move {
            if stale {
                if let Ok(fresh) = client.index().await {
                    index.borrow_mut().reconcile(None, &fresh, issued);
                }
                return;
            }
            for chunk in keys.chunks(INDEX_ROWS_PER_READ) {
                let Ok(rows) = client.index_rows(chunk).await else { return };
                index.borrow_mut().reconcile(Some(chunk), &rows, issued);
            }
        });
    }

    /// Every document's bytes. Only a caller that insists on a full hydrate
    /// uses it; the session hydrates through [`Self::load_index`].
    fn load_all(&self) -> BackendFuture<Vec<(String, String)>> {
        let client = self.client.clone();
        Box::pin(async move {
            let index = client.index().await.map_err(|e| e.to_string())?;
            let mut all = Vec::new();
            for entry in index.entries.into_iter().filter(|e| !e.content_hash.is_empty()) {
                if let Some(bytes) = client.get_document(&entry.key).await.map_err(|e| e.to_string())? {
                    all.push((format!("{PREFIX}{}", entry.key), text(bytes, &entry.key)?));
                }
            }
            Ok(all)
        })
    }

    fn load_index(&self) -> BackendFuture<Vec<(String, Entry)>> {
        let issued = self.index.borrow_mut().issue();
        let (client, keep) = (self.client.clone(), self.index.clone());
        Box::pin(async move {
            let index = client.index().await.map_err(|e| e.to_string())?;
            keep.borrow_mut().reconcile(None, &index, issued);
            Ok(index
                .entries
                .into_iter()
                // A revision with no document yet has nothing to open; it is
                // the revision picker's (S3), not the store's.
                .filter(|e| !e.content_hash.is_empty())
                .map(|e| (format!("{PREFIX}{}", e.key), Entry::OnDisk { bytes: e.size }))
                .collect())
        })
    }

    fn orders_writes_per_key(&self) -> bool {
        false
    }

    /// The rows a feed move names, read in chunks; this backend's own index
    /// map (what `plm_revision` answers) is brought up to date on the way —
    /// the asked keys the server no longer lists leave it.
    fn index_rows(&self, keys: &[String]) -> BackendFuture<Vec<IndexRow>> {
        let issued = self.index.borrow_mut().issue();
        let (client, index) = (self.client.clone(), self.index.clone());
        let keys = keys.to_vec();
        Box::pin(async move {
            let mut rows = Vec::new();
            for chunk in keys.chunks(INDEX_ROWS_PER_READ) {
                let answer = client.index_rows(chunk).await.map_err(|e| e.to_string())?;
                index.borrow_mut().reconcile(Some(chunk), &answer, issued);
                for entry in answer.entries {
                    if !entry.content_hash.is_empty() {
                        rows.push(IndexRow {
                            key: format!("{PREFIX}{}", entry.key),
                            bytes: entry.size,
                            content_hash: entry.content_hash,
                        });
                    }
                }
            }
            Ok(rows)
        })
    }
    /// The whole index, for a stale feed; this backend's own map is replaced by it.
    fn index_all(&self) -> BackendFuture<Vec<IndexRow>> {
        let issued = self.index.borrow_mut().issue();
        let (client, keep) = (self.client.clone(), self.index.clone());
        Box::pin(async move {
            let index = client.index().await.map_err(|e| e.to_string())?;
            keep.borrow_mut().reconcile(None, &index, issued);
            Ok(index
                .entries
                .into_iter()
                .filter(|e| !e.content_hash.is_empty())
                .map(|e| IndexRow { key: format!("{PREFIX}{}", e.key), bytes: e.size, content_hash: e.content_hash })
                .collect())
        })
    }
    fn get(&self, key: &str) -> BackendFuture<Option<String>> {
        let now = (self.now)();
        if let Some(why) = self.backoff.borrow().holding(key, now) {
            return Box::pin(async move { Err(why) });
        }
        let (client, backoff, clock, key) = (self.client.clone(), self.backoff.clone(), self.now.clone(), key.to_string());
        Box::pin(async move {
            let store_key = document_key(&key)?.to_string();
            match client.get_document(&store_key).await {
                Ok(bytes) => {
                    backoff.borrow_mut().clear(&key);
                    bytes.map(|b| text(b, &store_key)).transpose()
                }
                // Gone is an answer, not a failure: the key is absent.
                Err(PlmError::NotFound(_)) => {
                    backoff.borrow_mut().clear(&key);
                    Ok(None)
                }
                Err(e) => {
                    let why = e.to_string();
                    backoff.borrow_mut().fail(&key, clock(), &why);
                    Err(why)
                }
            }
        })
    }

    fn put(&self, key: &str, value: &str) -> BackendFuture<()> {
        self.backoff.borrow_mut().clear(key);
        let (client, key, value, on_saved) = (self.client.clone(), key.to_string(), value.to_string(), self.on_saved.clone());
        Box::pin(async move {
            let store_key = document_key(&key)?.to_string();
            client.put_document(&store_key, value.clone().into_bytes()).await.map_err(|e| e.to_string())?;
            match on_saved {
                Some(hook) => hook(client, store_key, value).await,
                None => Ok(()),
            }
        })
    }

    fn delete(&self, key: &str) -> BackendFuture<()> {
        self.backoff.borrow_mut().clear(key);
        let (client, key) = (self.client.clone(), key.to_string());
        Box::pin(async move {
            let store_key = document_key(&key)?;
            match client.delete_document(store_key).await {
                Ok(()) | Err(PlmError::NotFound(_)) => Ok(()),
                Err(e) => Err(e.to_string()),
            }
        })
    }
}

/// The reserved name (`@settings`) of a preference's mirror key
/// (`brep-app:settings`), for every name P5 holds: the preferences and
/// `@recovery`'s mirror.
fn preference_name(key: &str) -> Result<&'static str, String> {
    crate::store::PREFERENCE_KEYS
        .iter()
        .chain(crate::store::LOCAL_KEYS)
        .copied()
        .find(|name| MirrorStore::key(name) == key)
        .ok_or_else(|| format!("`{key}` is not a preference"))
}

/// The user's preferences on the server (P5) as a [`StoreBackend`].
pub(crate) struct PreferencesBackend {
    client: Rc<PlmClient>,
}

impl StoreBackend for PreferencesBackend {
    fn label(&self) -> String {
        "PLM preferences".into()
    }

    fn load_all(&self) -> BackendFuture<Vec<(String, String)>> {
        let client = self.client.clone();
        Box::pin(async move {
            let mut all = Vec::new();
            for name in crate::store::PREFERENCE_KEYS {
                if let Some(value) = client.get_preference(name).await.map_err(|e| e.to_string())? {
                    all.push((MirrorStore::key(name), value));
                }
            }
            Ok(all)
        })
    }

    fn orders_writes_per_key(&self) -> bool {
        false
    }

    fn get(&self, key: &str) -> BackendFuture<Option<String>> {
        let (client, key) = (self.client.clone(), key.to_string());
        Box::pin(async move { client.get_preference(preference_name(&key)?).await.map_err(|e| e.to_string()) })
    }

    fn put(&self, key: &str, value: &str) -> BackendFuture<()> {
        let (client, key, value) = (self.client.clone(), key.to_string(), value.to_string());
        Box::pin(async move { client.put_preference(preference_name(&key)?, value).await.map_err(|e| e.to_string()) })
    }

    fn delete(&self, key: &str) -> BackendFuture<()> {
        let (client, key) = (self.client.clone(), key.to_string());
        Box::pin(async move { client.delete_preference(preference_name(&key)?).await.map_err(|e| e.to_string()) })
    }
}

/// `@recovery` on the machine first, then on the server.
///
/// Reads and the hydrate are the LOCAL copy's: it is the one a network drop
/// cannot take away. A write lands locally and returns; the server copy is
/// sent in the background, and while one is in flight only the newest value
/// waits behind it, so a slow or dead server holds up nothing and is never
/// sent a stale snapshot after a newer one. A failed mirror is dropped: the
/// next write retries with newer bytes.
pub(crate) struct RecoveryMirror {
    local: Rc<dyn StoreBackend>,
    remote: Rc<dyn StoreBackend>,
    outbox: Rc<RefCell<Outbox>>,
}

#[derive(Default)]
struct Outbox {
    /// The newest value not yet sent (`None` value = a delete).
    waiting: Option<(String, Option<String>)>,
    sending: bool,
    /// Mirrors that failed, for a test or a status line to read.
    failures: u64,
}

impl RecoveryMirror {
    pub(crate) fn new(local: Rc<dyn StoreBackend>, remote: Rc<dyn StoreBackend>) -> Self {
        Self { local, remote, outbox: Rc::default() }
    }

    /// How many background mirrors have failed.
    #[cfg_attr(not(test), allow(dead_code))] // read by the tests; a status line may show it
    pub(crate) fn failures(&self) -> u64 {
        self.outbox.borrow().failures
    }

    fn mirror(&self, key: &str, value: Option<String>) {
        let mut outbox = self.outbox.borrow_mut();
        outbox.waiting = Some((key.to_string(), value));
        if outbox.sending {
            return;
        }
        outbox.sending = true;
        drop(outbox);
        MIRRORS_IN_FLIGHT.with(|n| n.set(n.get() + 1));
        let (outbox, remote) = (self.outbox.clone(), self.remote.clone());
        detach(async move {
            loop {
                let next = outbox.borrow_mut().waiting.take();
                let Some((key, value)) = next else {
                    outbox.borrow_mut().sending = false;
                    MIRRORS_IN_FLIGHT.with(|n| n.set(n.get().saturating_sub(1)));
                    return;
                };
                let sent = match value {
                    Some(value) => remote.put(&key, &value).await,
                    None => remote.delete(&key).await,
                };
                if sent.is_err() {
                    outbox.borrow_mut().failures += 1;
                }
            }
        });
    }
}

impl StoreBackend for RecoveryMirror {
    fn label(&self) -> String {
        self.local.label()
    }

    /// Only the local key space, read key by key: the local backend is the
    /// machine's whole store (every model in `<root>/models`, or a whole
    /// IndexedDB origin), and listing it would read every document in front
    /// of the first frame only for the router to drop them (S2's record §5).
    fn load_all(&self) -> BackendFuture<Vec<(String, String)>> {
        let local = self.local.clone();
        Box::pin(async move {
            let mut all = Vec::new();
            for name in crate::store::LOCAL_KEYS {
                let key = MirrorStore::key(name);
                if let Some(value) = local.get(&key).await? {
                    all.push((key, value));
                }
            }
            Ok(all)
        })
    }

    fn orders_writes_per_key(&self) -> bool {
        self.local.orders_writes_per_key()
    }

    fn get(&self, key: &str) -> BackendFuture<Option<String>> {
        self.local.get(key)
    }

    fn put(&self, key: &str, value: &str) -> BackendFuture<()> {
        let local = self.local.put(key, value);
        let (this, key, value) = (self.clone_handle(), key.to_string(), value.to_string());
        Box::pin(async move {
            local.await?;
            if mirrored(&key) {
                this.mirror(&key, Some(value));
            }
            Ok(())
        })
    }

    fn delete(&self, key: &str) -> BackendFuture<()> {
        let local = self.local.delete(key);
        let (this, key) = (self.clone_handle(), key.to_string());
        Box::pin(async move {
            local.await?;
            if mirrored(&key) {
                this.mirror(&key, None);
            }
            Ok(())
        })
    }
}

/// Whether a Local key is copied to the server. Only `@recovery` is (D1): the
/// other Local keys (`@plm_import`, the importer's ledgers) describe THIS
/// machine and never leave it. It matters twice over: the mirror's outbox
/// keeps only the newest waiting write, so a Local write of another key
/// would take the place of a `@recovery` copy not yet sent.
fn mirrored(key: &str) -> bool {
    key == MirrorStore::key(crate::store::RECOVERY_KEY)
}

impl RecoveryMirror {
    fn clone_handle(&self) -> Self {
        Self { local: self.local.clone(), remote: self.remote.clone(), outbox: self.outbox.clone() }
    }
}

/// Run `task` to completion in the background on the host's executor.
fn detach(task: impl std::future::Future<Output = ()> + 'static) {
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(task);
    #[cfg(not(target_arch = "wasm32"))]
    super::native::spawn(task);
}

