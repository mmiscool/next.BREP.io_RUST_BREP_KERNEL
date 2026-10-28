//! Thumbnails of PLM revisions (P9): the picture a save makes and uploads.
//!
//! The server never renders (`brep_plm::thumbnail`); the CAD app does, with
//! the CPU rasterizer every host shares ([`brep_render::thumbnail`]). The
//! parts of it here:
//!
//! - [`stage`](crate::plm::thumbnail::stage), at the save site: the scene is copied out of the engine on the
//!   UI thread (well under a millisecond), and the raster runs off it. Natively
//!   on a thread of its own; in the browser, which has no threads, when the
//!   upload first asks for the picture, which is after the server has accepted
//!   the document, not in the frame that saved. A save made while a rebuild is
//!   still running waits for that run to land ([`tick`](crate::plm::thumbnail::tick)), so the picture is of
//!   the model that was saved, not the one before.
//! - `upload_on_save`, a `SavedHook` the PLM backend runs after each
//!   accepted document write: when a picture was staged for exactly those
//!   bytes (their SHA-256 is the server's `content_hash`), it is uploaded in a
//!   task of its own, so the key's next write never waits for it. It never
//!   fails the save: an upload the server refuses is counted and said in
//!   [`stats`](crate::plm::thumbnail::stats), and the next save tries again.
//! - `chain` runs it after S5's uses publisher, which keeps its own error.
//! - The RECENT-FILE pictures (`stage_recent`, [`persist_recent`](crate::plm::thumbnail::persist_recent),
//!   [`recent_textures`](crate::plm::thumbnail::recent_textures)): the File menu's recent list shows a small square of
//!   every document saved through ANY store (browser, native file, PLM), made
//!   by the same rasterizer and the same staging (a save mid-rebuild waits the
//!   same way) and kept in the local store under
//!   [`RECENT_THUMBNAILS_KEY`](crate::store::RECENT_THUMBNAILS_KEY) — one
//!   JSON map, name → base64 PNG, pruned to the recent list. The PLM upload
//!   path and its [`stats`](crate::plm::thumbnail::stats) are untouched by it: a PLM save stages two
//!   pictures, the server's and the menu's, and counts the server's.

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

struct Shared {
    /// The picture's edge, in pixels.
    size: u32,
    capture: Mutex<Option<Capture>>,
    result: Mutex<Option<Option<Vec<u8>>>>,
    waker: Mutex<Option<Waker>>,
}

impl Default for Shared {
    fn default() -> Self {
        Self { size: raster::SIZE, capture: Mutex::new(None), result: Mutex::new(None), waker: Mutex::new(None) }
    }
}

impl Picture {
    /// A picture `size` pixels on edge (the default is the server's [`raster::SIZE`]).
    fn sized(size: u32) -> Self {
        Self(Arc::new(Shared { size, ..Shared::default() }))
    }

    /// Done without waiting: `Some(None)` is a picture of nothing (an empty
    /// scene). In the browser the first look IS the raster, on this thread.
    fn peek(&self) -> Option<Option<Vec<u8>>> {
        let capture = self.0.capture.lock().unwrap_or_else(|p| p.into_inner()).take();
        if let Some(capture) = capture {
            return Some(render(&capture, self.0.size));
        }
        self.0.result.lock().unwrap_or_else(|p| p.into_inner()).take()
    }

    /// Raster `capture`: on a thread natively, later on this one in the browser.
    fn start(&self, capture: Capture) {
        #[cfg(not(target_arch = "wasm32"))]
        {
            let picture = self.clone();
            let size = self.0.size;
            std::thread::spawn(move || picture.fill(render(&capture, size)));
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
            return Poll::Ready(render(&capture, self.0.size));
        }
        *self.0.waker.lock().unwrap_or_else(|p| p.into_inner()) = Some(cx.waker().clone());
        match self.0.result.lock().unwrap_or_else(|p| p.into_inner()).take() {
            Some(png) => Poll::Ready(png),
            None => Poll::Pending,
        }
    }
}

/// The raster, timed. `None` for an empty scene.
fn render(capture: &Capture, size: u32) -> Option<Vec<u8>> {
    let started = crate::recovery::wall_clock();
    let png = raster::render_png(capture, size);
    let ms = (crate::recovery::wall_clock() - started) * 1000.0;
    // The timing is the SERVER picture's ([`stats`]); a recent-list raster
    // (`RECENT_SIZE`) never overwrites it.
    if size == raster::SIZE {
        // A raster thread has its own thread-locals; the count that matters is
        // the UI thread's, so a thread's figure rides back with the picture.
        #[cfg(target_arch = "wasm32")]
        count(|s| s.last_render_ms = ms);
        #[cfg(not(target_arch = "wasm32"))]
        LAST_RENDER_MS.store(ms.to_bits(), std::sync::atomic::Ordering::Relaxed);
    }
    png
}

#[cfg(not(target_arch = "wasm32"))]
static LAST_RENDER_MS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Stage the picture of `engine`'s model for the save of `document` under
/// the PLM store key `key` (`part/<p>/rev/<r>`). `doc` is the document's tab
/// id, for a save made mid-rebuild ([`tick`]).
pub fn stage(key: &str, document: &str, engine: &EngineState, doc: u64) {
    let picture = picture_of(engine, doc, raster::SIZE);
    count(|s| s.staged += 1);
    STAGED.with(|s| s.borrow_mut().insert(key.to_string(), Staged { hash: content_hash(document), picture }));
}

/// A picture of `engine`'s model, `size` on edge: started now, or once the
/// run that is pending lands ([`tick`]).
fn picture_of(engine: &EngineState, doc: u64, size: u32) -> Picture {
    let picture = Picture::sized(size);
    if engine.run_pending() {
        WAITING.with(|w| w.borrow_mut().push(Waiting { doc, after: engine.applied_generation(), picture: picture.clone() }));
    } else {
        picture.start(raster::capture(&engine.scene));
    }
    picture
}

/// The pictures of a save through the store. Every store's save gets the
/// File menu's recent-list picture (`stage_recent`); a PLM store's save of a
/// revision's document (`name` in either spelling) also gets the server's
/// ([`stage`]), and an instructions revision's save its player data
/// ([`crate::plm::player_data`]). The file stores upload nothing.
pub fn stage_save(store: &dyn crate::store::ModelStore, name: &str, document: &str, engine: &EngineState, doc: u64) {
    stage_recent(name, engine, doc);
    if store.plm_client().is_none() {
        return;
    }
    if let Some(key) = crate::plm::uses::revision_key_in(name) {
        stage(&key, document, engine, doc);
        crate::plm::player_data::stage(&key, document, engine, doc);
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
    let path = format!("{}/thumbnail?content_hash={hash}&renderer={}", super::identity::revision_path(&part, &revision), raster::RENDERER);
    client.call_typed("PUT", &path, png, "image/png").await.map(|_| ()).map_err(|e| e.to_string())
}

fn revision_of(key: &str) -> Result<(String, String), String> {
    super::family::key_ids(key).ok_or_else(|| format!("'{key}' is not a revision key"))
}

fn detach(task: impl Future<Output = ()> + 'static) {
    crate::store::spawn_waking(task);
}

// ---------------------------------------------------------------------------
// The File menu's recent-file pictures.
// ---------------------------------------------------------------------------

use crate::store::{ModelStore, RECENT_THUMBNAILS_KEY};
use eframe::egui;

/// A recent-file picture's edge, in pixels: small, because the browser store
/// is bounded, and drawn at [`RECENT_SIDE`] points (two pixels per point on a
/// HiDPI screen). A shaded 80-pixel PNG of a part is a few kilobytes; twenty
/// of them, the recent list's cap, stay well under a hundred.
pub const RECENT_SIZE: u32 = 80;
/// The side of the square a recent row draws its picture in, in points.
pub const RECENT_SIDE: f32 = 40.0;

/// What happened to the recent-file pictures this session made, for tests.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RecentStats {
    /// Pictures staged at a save.
    pub staged: u32,
    /// Written to the store.
    pub stored: u32,
    /// Nothing to picture, or the name was no longer a recent document.
    pub skipped: u32,
}

/// A picture on its way to the recent-thumbnail map.
struct Recent {
    name: String,
    picture: Picture,
}

thread_local! {
    static RECENT: RefCell<Vec<Recent>> = const { RefCell::new(Vec::new()) };
    static RECENT_STATS: RefCell<RecentStats> = RefCell::new(RecentStats::default());
    static RECENT_TEXTURES: RefCell<Option<RecentTextures>> = const { RefCell::new(None) };
}

/// This session's recent-picture counts.
pub fn recent_stats() -> RecentStats {
    RECENT_STATS.with(|s| s.borrow().clone())
}

fn count_recent(f: impl FnOnce(&mut RecentStats)) {
    RECENT_STATS.with(|s| f(&mut s.borrow_mut()));
}

/// Stage the recent-list picture of `engine`'s model for the save of `name`.
/// The raster runs off the UI thread natively and in the frame after the
/// save in the browser; [`persist_recent`] writes what it made.
pub fn stage_recent(name: &str, engine: &EngineState, doc: u64) {
    let picture = picture_of(engine, doc, RECENT_SIZE);
    count_recent(|s| s.staged += 1);
    RECENT.with(|r| r.borrow_mut().push(Recent { name: name.to_string(), picture }));
}

/// The recent-thumbnail map as stored: name → base64 PNG.
fn recent_map(store: &dyn ModelStore) -> BTreeMap<String, String> {
    store
        .read(RECENT_THUMBNAILS_KEY)
        .and_then(|text| serde_json::from_str(&text).ok())
        .unwrap_or_default()
}

/// Write the recent-list pictures that finished since the last frame into
/// the store, pruning the map to the names the recent list still holds.
/// Called once per frame. Never an error: a picture that could not be
/// rendered or written is dropped and the menu shows its placeholder.
pub fn persist_recent(store: &dyn ModelStore) {
    let done: Vec<(String, Option<Vec<u8>>)> = RECENT.with(|r| {
        let mut pending = r.borrow_mut();
        let mut done = Vec::new();
        pending.retain(|entry| match entry.picture.peek() {
            Some(png) => {
                done.push((entry.name.clone(), png));
                false
            }
            None => true,
        });
        done
    });
    if done.is_empty() {
        return;
    }
    let recent = crate::panels::file::recent_documents(store);
    let mut map = recent_map(store);
    map.retain(|name, _| recent.contains(name));
    for (name, png) in done {
        match png {
            Some(png) if recent.contains(&name) => {
                map.insert(name, base64_encode(&png));
                count_recent(|s| s.stored += 1);
            }
            Some(_) => count_recent(|s| s.skipped += 1),
            None => {
                // An empty scene: no picture, so the row shows its placeholder.
                map.remove(&name);
                count_recent(|s| s.skipped += 1);
            }
        }
    }
    let text = serde_json::to_string(&map).unwrap_or_else(|_| "{}".into());
    if let Err(why) = store.write(RECENT_THUMBNAILS_KEY, &text) {
        log::warn!("recent thumbnails not stored: {why}");
    }
}

/// The recent-file pictures as GPU textures, decoded once per store
/// mutation and reused while their bytes are unchanged.
struct RecentTextures {
    generation: u64,
    textures: Rc<BTreeMap<String, (String, egui::TextureHandle)>>,
}

/// The recent-file pictures the store holds right now, name → texture, for
/// the File menu to draw. A name with no entry has no picture.
pub fn recent_textures(ctx: &egui::Context, store: &dyn ModelStore) -> Rc<BTreeMap<String, (String, egui::TextureHandle)>> {
    let generation = store.mutation_generation();
    RECENT_TEXTURES.with(|cache| {
        let mut cache = cache.borrow_mut();
        if let Some(cached) = cache.as_ref().filter(|c| c.generation == generation) {
            return cached.textures.clone();
        }
        let previous = cache.take().map(|c| c.textures).unwrap_or_default();
        let mut textures = BTreeMap::new();
        for (name, encoded) in recent_map(store) {
            if let Some((kept, texture)) = previous.get(&name).filter(|(kept, _)| *kept == encoded) {
                textures.insert(name, (kept.clone(), texture.clone()));
                continue;
            }
            let Some(image) = base64_decode(&encoded).and_then(|png| decode_png(&png)) else { continue };
            let texture = ctx.load_texture(format!("recent-thumbnail:{name}"), image, egui::TextureOptions::LINEAR);
            textures.insert(name, (encoded, texture));
        }
        let textures = Rc::new(textures);
        *cache = Some(RecentTextures { generation, textures: textures.clone() });
        textures
    })
}

/// The preview a document carries (`thumbnail: {data, status}`, as a save
/// or a bake embeds it) as an egui image; `None` when it has none, or only
/// the empty placeholder.
pub(crate) fn embedded_image(document: &serde_json::Value) -> Option<egui::ColorImage> {
    let thumbnail = &document["thumbnail"];
    if thumbnail["status"].as_str().is_some_and(|status| status != "ready") {
        return None;
    }
    let encoded = thumbnail["data"].as_str().or_else(|| thumbnail["png"].as_str())?;
    decode_png(&base64_decode(encoded)?)
}

/// A stored PNG as an egui image; `None` for bytes that are not one.
fn decode_png(bytes: &[u8]) -> Option<egui::ColorImage> {
    let mut reader = image::ImageReader::with_format(std::io::Cursor::new(bytes), image::ImageFormat::Png);
    let mut limits = image::Limits::default();
    limits.max_image_width = Some(512);
    limits.max_image_height = Some(512);
    limits.max_alloc = Some(4 * 1024 * 1024);
    reader.limits(limits);
    let image = reader.decode().ok()?.to_rgba8();
    Some(egui::ColorImage::from_rgba_unmultiplied([image.width() as usize, image.height() as usize], image.as_raw()))
}

const BASE64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Standard base64 with padding: how a PNG rides in the string-valued store.
fn base64_encode(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let n = chunk.iter().enumerate().fold(0u32, |n, (i, b)| n | (u32::from(*b) << (16 - 8 * i)));
        for i in 0..4 {
            if i <= chunk.len() {
                out.push(BASE64[((n >> (18 - 6 * i)) & 63) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

fn base64_decode(text: &str) -> Option<Vec<u8>> {
    let text = text.trim_end_matches('=');
    let mut out = Vec::with_capacity(text.len() * 3 / 4);
    let mut acc = 0u32;
    let mut bits = 0u32;
    for c in text.bytes() {
        let value = BASE64.iter().position(|b| *b == c)? as u32;
        acc = (acc << 6) | value;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push(((acc >> bits) & 0xff) as u8);
        }
    }
    Some(out)
}

