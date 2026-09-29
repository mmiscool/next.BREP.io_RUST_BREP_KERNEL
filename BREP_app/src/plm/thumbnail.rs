//! Thumbnails of PLM revisions (P9): the picture a save makes and uploads.
//!
//! The server never renders ([`brep_plm::thumbnail`]); the CAD app does, with
//! the CPU rasterizer every host shares ([`brep_render::thumbnail`]). The
//! parts of it here:
//!
//! - [`stage`], at the save site: the scene is copied out of the engine on the
//!   UI thread (well under a millisecond), and the raster runs off it. Natively
//!   on a thread of its own; in the browser, which has no threads, when the
//!   upload first asks for the picture, which is after the server has accepted
//!   the document, not in the frame that saved. A save made while a rebuild is
//!   still running waits for that run to land ([`tick`]), so the picture is of
//!   the model that was saved, not the one before.
//! - [`upload_on_save`], a [`SavedHook`] the PLM backend runs after each
//!   accepted document write: when a picture was staged for exactly those
//!   bytes (their SHA-256 is the server's `content_hash`), it is uploaded in a
//!   task of its own, so the key's next write never waits for it. It never
//!   fails the save: an upload the server refuses is counted and said in
//!   [`stats`], and the next save tries again.
//! - [`chain`] runs it after S5's uses publisher, which keeps its own error.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::future::Future;
use std::pin::Pin;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Waker};

use brep_render::engine_state::EngineState;
use brep_render::thumbnail::{self as raster, Capture};

use super::backend::SavedHook;
use super::client::PlmClient;

/// SHA-256 of a document's bytes, lowercase hex: the server's `content_hash`.
pub fn content_hash(document: &str) -> String {
    use sha2::{Digest, Sha256};
    Sha256::digest(document.as_bytes()).iter().map(|b| format!("{b:02x}")).collect()
}

/// What happened to the pictures this session made, for scripts and tests.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize)]
pub struct Stats {
    /// Pictures staged at a save.
    pub staged: u32,
    /// Uploaded and accepted.
    pub uploaded: u32,
    /// Nothing to picture (an empty scene) or superseded before it was sent.
    pub skipped: u32,
    /// Refused or unreachable.
    pub failed: u32,
    /// The last refusal, as the server said it.
    pub last_error: String,
    /// The last raster's cost, in milliseconds.
    pub last_render_ms: f64,
}

thread_local! {
    static STAGED: RefCell<BTreeMap<String, Staged>> = RefCell::new(BTreeMap::new());
    static WAITING: RefCell<Vec<Waiting>> = const { RefCell::new(Vec::new()) };
    static STATS: RefCell<Stats> = RefCell::new(Stats::default());
}

/// This session's counts.
pub fn stats() -> Stats {
    STATS.with(|s| s.borrow().clone())
}

fn count(f: impl FnOnce(&mut Stats)) {
    STATS.with(|s| f(&mut s.borrow_mut()));
}

struct Staged {
    hash: String,
    picture: Picture,
}

/// A save made while its document was still rebuilding: pictured once the
/// run that was pending lands.
struct Waiting {
    doc: u64,
    after: u64,
    picture: Picture,
}

/// A picture on its way: filled by a raster thread (native), or rastered on
/// first poll from the capture it holds (the browser).
#[derive(Clone, Default)]
pub struct Picture(Arc<Shared>);

#[derive(Default)]
struct Shared {
    capture: Mutex<Option<Capture>>,
    result: Mutex<Option<Option<Vec<u8>>>>,
    waker: Mutex<Option<Waker>>,
}

impl Picture {
    /// Raster `capture`: on a thread natively, later on this one in the browser.
    fn start(&self, capture: Capture) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let picture = self.clone();
            std::thread::spawn(move || picture.fill(render(&capture)));
        }
        #[cfg(target_arch = "wasm32")]
        {
            *self.0.capture.lock().unwrap_or_else(|p| p.into_inner()) = Some(capture);
            if let Some(waker) = self.0.waker.lock().unwrap_or_else(|p| p.into_inner()).take() {
                waker.wake();
            }
        }
    }

    fn fill(&self, png: Option<Vec<u8>>) {
        *self.0.result.lock().unwrap_or_else(|p| p.into_inner()) = Some(png);
        if let Some(waker) = self.0.waker.lock().unwrap_or_else(|p| p.into_inner()).take() {
            waker.wake();
        }
    }
}

impl Future for Picture {
    type Output = Option<Vec<u8>>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let capture = self.0.capture.lock().unwrap_or_else(|p| p.into_inner()).take();
        if let Some(capture) = capture {
            return Poll::Ready(render(&capture));
        }
        *self.0.waker.lock().unwrap_or_else(|p| p.into_inner()) = Some(cx.waker().clone());
        match self.0.result.lock().unwrap_or_else(|p| p.into_inner()).take() {
            Some(png) => Poll::Ready(png),
            None => Poll::Pending,
        }
    }
}

/// The raster, timed. `None` for an empty scene.
fn render(capture: &Capture) -> Option<Vec<u8>> {
    let started = crate::recovery::wall_clock();
    let png = raster::render_png(capture, raster::SIZE);
    let ms = (crate::recovery::wall_clock() - started) * 1000.0;
    // A raster thread has its own thread-locals; the count that matters is
    // the UI thread's, so a thread's figure rides back with the picture.
    #[cfg(target_arch = "wasm32")]
    count(|s| s.last_render_ms = ms);
    #[cfg(not(target_arch = "wasm32"))]
    LAST_RENDER_MS.store(ms.to_bits(), std::sync::atomic::Ordering::Relaxed);
    png
}

#[cfg(not(target_arch = "wasm32"))]
static LAST_RENDER_MS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Stage the picture of `engine`'s model for the save of `document` under
/// the PLM store key `key` (`part/<p>/rev/<r>`). `doc` is the document's tab
/// id, for a save made mid-rebuild ([`tick`]).
pub fn stage(key: &str, document: &str, engine: &EngineState, doc: u64) {
    let picture = Picture::default();
    if engine.run_pending() {
        WAITING.with(|w| w.borrow_mut().push(Waiting { doc, after: engine.applied_generation(), picture: picture.clone() }));
    } else {
        picture.start(raster::capture(&engine.scene));
    }
    count(|s| s.staged += 1);
    STAGED.with(|s| s.borrow_mut().insert(key.to_string(), Staged { hash: content_hash(document), picture }));
}

/// [`stage`] for a save through the store: only a PLM store, and only a
/// revision's document (`name` in either spelling), get a picture. The file
/// stores never do.
pub fn stage_save(store: &dyn crate::store::ModelStore, name: &str, document: &str, engine: &EngineState, doc: u64) {
    if store.plm_client().is_none() {
        return;
    }
    if let Some(key) = crate::plm::uses::revision_key_in(name) {
        stage(&key, document, engine, doc);
    }
}

/// Picture the saves made mid-rebuild whose run has landed; called once per
/// frame. `engine_of` finds a tab's engine by id (`None`: the tab closed,
/// and its picture is dropped).
pub fn tick<'a>(engine_of: impl Fn(u64) -> Option<&'a EngineState>) {
    if WAITING.with(|w| w.borrow().is_empty()) {
        return;
    }
    WAITING.with(|w| {
        w.borrow_mut().retain(|waiting| match engine_of(waiting.doc) {
            None => {
                waiting.picture.fill(None);
                false
            }
            Some(engine) if engine.run_pending() || engine.applied_generation() == waiting.after => true,
            Some(engine) => {
                waiting.picture.start(raster::capture(&engine.scene));
                false
            }
        })
    });
}

/// The picture staged for exactly these bytes under `key`, taken. One staged
/// for other bytes (a later save) is left for that save's own hook.
fn take(key: &str, document: &str) -> Option<Picture> {
    let hash = content_hash(document);
    STAGED.with(|s| {
        let mut staged = s.borrow_mut();
        match staged.get(key) {
            Some(entry) if entry.hash == hash => staged.remove(key).map(|e| e.picture),
            _ => None,
        }
    })
}

/// The hook: after an accepted document write, upload the picture staged for
/// it, in a task of its own. Always `Ok`: a thumbnail never fails a save.
pub(crate) fn upload_on_save() -> SavedHook {
    Rc::new(|client, key, value| {
        // One spelling for the staged picture and the upload's route.
        let key = crate::plm::uses::revision_key_in(&key).unwrap_or(key);
        if let Some(picture) = take(&key, &value) {
            let hash = content_hash(&value);
            detach(async move { upload(&client, &key, &hash, picture).await });
        }
        Box::pin(async { Ok(()) })
    })
}

/// Run `first`, then `then` whatever `first` answered; the answer is
/// `first`'s. The thumbnail upload follows S5's uses publisher this way.
pub(crate) fn chain(first: SavedHook, then: SavedHook) -> SavedHook {
    Rc::new(move |client: Rc<PlmClient>, key: String, value: String| {
        let before = first(client.clone(), key.clone(), value.clone());
        let then = then.clone();
        Box::pin(async move {
            let answer = before.await;
            let _ = then(client, key, value).await;
            answer
        })
    })
}

/// `PUT /api/parts/<p>/revisions/<r>/thumbnail` with the picture.
async fn upload(client: &PlmClient, key: &str, hash: &str, picture: Picture) {
    let Some(png) = picture.await else {
        count(|s| s.skipped += 1);
        return;
    };
    #[cfg(not(target_arch = "wasm32"))]
    count(|s| s.last_render_ms = f64::from_bits(LAST_RENDER_MS.load(std::sync::atomic::Ordering::Relaxed)));
    match put(client, key, hash, png).await {
        Ok(()) => count(|s| s.uploaded += 1),
        Err(why) => {
            log::warn!("thumbnail of {key} not uploaded: {why}");
            count(|s| {
                s.failed += 1;
                s.last_error = why;
            });
        }
    }
}

/// Upload `png` as the thumbnail of the revision `key`, picturing the
/// document whose hash is `hash`. The bake worker calls this directly.
pub async fn put(client: &PlmClient, key: &str, hash: &str, png: Vec<u8>) -> Result<(), String> {
    let (part, revision) = revision_of(key)?;
    let path = format!("/api/parts/{part}/revisions/{revision}/thumbnail?content_hash={hash}&renderer={}", raster::RENDERER);
    client.call_typed("PUT", &path, png, "image/png").await.map(|_| ()).map_err(|e| e.to_string())
}

fn revision_of(key: &str) -> Result<(&str, &str), String> {
    let mut parts = key.split('/');
    match (parts.next(), parts.next(), parts.next(), parts.next(), parts.next()) {
        (Some("part"), Some(p), Some("rev"), Some(r), None) if !p.is_empty() && !r.is_empty() => Ok((p, r)),
        _ => Err(format!("`{key}` is not a revision key")),
    }
}

fn detach(task: impl Future<Output = ()> + 'static) {
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(task);
    #[cfg(not(target_arch = "wasm32"))]
    super::native::spawn(task);
}

