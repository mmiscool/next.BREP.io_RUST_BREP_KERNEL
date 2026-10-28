//! The player's data, uploaded at a PLM save: manufacturing instructions'
//! (their steps), or an assembly's motion study (the active study's baked
//! frames — `brep_render::engine_state::simulation_player`), both the same
//! `PlayerData` the PLM serves from the trusted template.
//!
//! The PLM serves the player per instructions revision from data the CAD app
//! makes (`brep_plm::player`): the page is the instructions crate's trusted
//! template, and only the `PlayerData` JSON comes from the app. That data is
//! keyed by the document it was made from, and an instructions revision does
//! not release without data made from the document it holds. So every
//! accepted save of an instructions revision is followed by
//! `PUT /api/parts/<p>/revisions/<r>/player-data?content_hash=<SHA-256 of the
//! saved bytes>`, the same way [`crate::plm::thumbnail`] follows it with the
//! picture:
//!
//! - [`stage`](crate::plm::player_data::stage), at the save site: the data is made from the engine on the UI
//!   thread (the step pictures and part thumbnails need its scene). A save
//!   made while a rebuild is still running waits for that run to land
//!   ([`tick`](crate::plm::player_data::tick)), so the data shows the model that was saved.
//! - `upload_on_save`, a `SavedHook` the PLM backend runs after each
//!   accepted write: when data was staged for exactly those bytes, it is
//!   uploaded in a task of its own. It never fails the save. A refusal is
//!   counted in [`stats`](crate::plm::player_data::stats) and said once as a notice ([`take_notice`](crate::plm::player_data::take_notice)); a `409`
//!   (the revision holds another document by now) reads "player data is from
//!   a different document". The next save makes it again.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::future::Future;
use std::pin::Pin;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

use brep_render::engine_state::EngineState;

use super::backend::SavedHook;
use super::client::{PlmClient, PlmError};
use super::thumbnail::content_hash;

/// What a `409` reads as: the server holds another document than the one the
/// data was made from.
pub const STALE: &str = "player data is from a different document";

/// What happened to the player data this session made, for scripts and tests.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize)]
pub struct Stats {
    /// Staged at a save of an instructions revision.
    pub staged: u32,
    /// Uploaded and accepted.
    pub uploaded: u32,
    /// Not made (the instructions could not be played) or superseded.
    pub skipped: u32,
    /// Refused or unreachable.
    pub failed: u32,
    /// The last refusal or reason, as the user should read it.
    pub last_error: String,
}

thread_local! {
    static STAGED: RefCell<BTreeMap<String, Staged>> = RefCell::new(BTreeMap::new());
    static WAITING: RefCell<Vec<Waiting>> = const { RefCell::new(Vec::new()) };
    static STATS: RefCell<Stats> = RefCell::new(Stats::default());
    static NOTICE: RefCell<Option<String>> = const { RefCell::new(None) };
}

/// This session's counts.
pub fn stats() -> Stats {
    STATS.with(|s| s.borrow().clone())
}

fn count(f: impl FnOnce(&mut Stats)) {
    STATS.with(|s| f(&mut s.borrow_mut()));
}

/// The refusal to say since the last call, once.
pub fn take_notice() -> Option<String> {
    NOTICE.with(|n| n.borrow_mut().take())
}

struct Staged {
    hash: String,
    data: Data,
}

/// A save made while its document was still rebuilding: made once the run
/// that was pending lands.
struct Waiting {
    doc: u64,
    after: u64,
    data: Data,
}

/// Player data on its way: made at the save, or once its run lands.
#[derive(Clone, Default)]
struct Data(Rc<RefCell<Slot>>);

#[derive(Default)]
struct Slot {
    /// `Some(None)`: nothing to upload.
    json: Option<Option<String>>,
    waker: Option<Waker>,
}

impl Data {
    fn fill(&self, json: Option<String>) {
        let mut slot = self.0.borrow_mut();
        slot.json = Some(json);
        if let Some(waker) = slot.waker.take() {
            waker.wake();
        }
    }
}

impl Future for Data {
    type Output = Option<String>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let mut slot = self.0.borrow_mut();
        match slot.json.take() {
            Some(json) => Poll::Ready(json),
            None => {
                slot.waker = Some(cx.waker().clone());
                Poll::Pending
            }
        }
    }
}

/// The player data of `engine`'s document, as JSON: its instructions, or its
/// active motion study as baked (a study not played to the end has none);
/// `None` (counted) when it cannot be played.
fn make(engine: &EngineState) -> Option<String> {
    let made = if engine.is_instructions() { engine.instructions_player_data() } else { engine.simulation_player_data_baked() };
    match made {
        Ok(data) => Some(data.to_json()),
        Err(why) => {
            count(|s| {
                s.skipped += 1;
                s.last_error = format!("no player data: {why}");
            });
            None
        }
    }
}

/// Stage the player data for the save of `document` under the PLM store key
/// `key` (`part/<p>/rev/<r>`) when `engine` holds instructions or a played
/// motion study. `doc` is the document's tab id, for a save made mid-rebuild
/// ([`tick`]).
pub fn stage(key: &str, document: &str, engine: &EngineState, doc: u64) {
    // Instructions always publish; an assembly publishes its active study
    // once it has been played or scrubbed (the bake is what is published).
    if !engine.is_instructions() && engine.simulation_playback_json().is_null() {
        return;
    }
    let data = Data::default();
    if engine.run_pending() {
        WAITING.with(|w| w.borrow_mut().push(Waiting { doc, after: engine.applied_generation(), data: data.clone() }));
    } else {
        data.fill(make(engine));
    }
    count(|s| s.staged += 1);
    STAGED.with(|s| s.borrow_mut().insert(key.to_string(), Staged { hash: content_hash(document), data }));
}

/// Make the data of the saves made mid-rebuild whose run has landed; called
/// once per frame. `engine_of` finds a tab's engine by id (`None`: the tab
/// closed, and its data is dropped).
pub fn tick<'a>(engine_of: impl Fn(u64) -> Option<&'a EngineState>) {
    if WAITING.with(|w| w.borrow().is_empty()) {
        return;
    }
    let mut ready: Vec<(Data, Option<String>)> = Vec::new();
    WAITING.with(|w| {
        w.borrow_mut().retain(|waiting| match engine_of(waiting.doc) {
            None => {
                ready.push((waiting.data.clone(), None));
                false
            }
            Some(engine) if engine.run_pending() || engine.applied_generation() == waiting.after => true,
            Some(engine) => {
                ready.push((waiting.data.clone(), make(engine)));
                false
            }
        })
    });
    for (data, json) in ready {
        data.fill(json);
    }
}

/// The data staged for exactly these bytes under `key`, taken. Data staged
/// for other bytes (a later save) is left for that save's own hook.
fn take(key: &str, document: &str) -> Option<Data> {
    let hash = content_hash(document);
    STAGED.with(|s| {
        let mut staged = s.borrow_mut();
        match staged.get(key) {
            Some(entry) if entry.hash == hash => staged.remove(key).map(|e| e.data),
            _ => None,
        }
    })
}

/// The hook: after an accepted document write, upload the data staged for it,
/// in a task of its own. Always `Ok`: player data never fails a save.
pub(crate) fn upload_on_save() -> SavedHook {
    Rc::new(|client, key, value| {
        let key = crate::plm::uses::revision_key_in(&key).unwrap_or(key);
        if let Some(data) = take(&key, &value) {
            let hash = content_hash(&value);
            crate::store::spawn_waking(async move { upload(&client, &key, &hash, data).await });
        }
        Box::pin(async { Ok(()) })
    })
}

async fn upload(client: &PlmClient, key: &str, hash: &str, data: Data) {
    let Some(json) = data.await else {
        count(|s| s.skipped += 1);
        return;
    };
    match put(client, key, hash, json).await {
        Ok(()) => count(|s| s.uploaded += 1),
        Err(error) => {
            let why = if error.status() == Some(409) { STALE.to_string() } else { format!("player data not uploaded: {error}") };
            log::warn!("{key}: {why}");
            count(|s| {
                s.failed += 1;
                s.last_error = why.clone();
            });
            NOTICE.with(|n| *n.borrow_mut() = Some(why));
        }
    }
}

/// Upload `json` as the player data of the instructions revision `key`, made
/// from the document whose hash is `hash`.
pub async fn put(client: &PlmClient, key: &str, hash: &str, json: String) -> Result<(), PlmError> {
    let (part, revision) = super::family::key_ids(key).ok_or_else(|| PlmError::Malformed(format!("'{key}' is not a revision key")))?;
    let path = format!("{}/player-data?content_hash={hash}", super::identity::revision_path(&part, &revision));
    client.call_typed("PUT", &path, json.into_bytes(), "application/json").await.map(drop)
}

