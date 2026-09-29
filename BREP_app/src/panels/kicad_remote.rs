//! KiCad's official libraries, fetched from GitLab ONE FILE AT A TIME.
//!
//! The second source the importer reads. `kicad_import.rs` follows a symbol to
//! its footprint to its 3D model through a [`KicadFiles`](super::kicad_import::KicadFiles);
//! this module is that trait backed by KiCad's own repositories instead of the
//! user's disk, so the whole chain — the notes, the pad matching, KiCad's model
//! transform, the `Pins` port group — is the SAME code for both sources.
//!
//! # The three repositories, and why a tag is pinned
//!
//! KiCad keeps its libraries in three GitLab projects, one per link of the
//! chain: `kicad-symbols`, `kicad-footprints` and `kicad-packages3D`. Their
//! `master` branches are KiCad 10 and store a symbol library as a
//! `*.kicad_symdir` FOLDER, which this importer does not read; the 9.x release
//! tags still hold one `*.kicad_sym` file per library, which it does. So a
//! fetch names [`DEFAULT_TAG`] and never a branch. All three repositories carry
//! the same tag, which is what lets one tag address the whole chain.
//!
//! # Per file, not per archive
//!
//! Nothing here downloads a library archive. `kicad-packages3D` is gigabytes,
//! so its archive would be ZIP64 and would not fit in memory at once — both of
//! which the kernel's 3MF reader (`io/three_mf/zip.rs`) refuses BY DESIGN — and
//! `kicad-footprints` holds far more than that reader's `MAX_ENTRIES`. Only
//! `kicad-symbols` is small enough to unpack, and one mechanism beats two.
//! Fetching per file also means importing five parts downloads five models
//! rather than a library: the SOIC-8 chain below is 3 files, 308 KB.
//!
//! # The cache IS a KiCad install
//!
//! A fetched file is written under a cache root laid out exactly the way KiCad
//! lays out its own share directory — `symbols/`, `footprints/<lib>.pretty/`,
//! `3dmodels/<lib>.3dshapes/` — so a [`KicadLibrary`] pointing at that root is
//! an ordinary local library. A second import of the same part reads the disk
//! and asks GitLab nothing, and a user who points the local path at the cache
//! gets everything already downloaded.
//!
//! # One file, and then all of them
//!
//! A bulk sweep ([`super::kicad_bulk`]) is this module's [`fetch_file`] in a
//! loop over 223 libraries, so two things here are built for it rather than for
//! a single import: a 429 or a 5xx is waited out instead of failed
//! ([`retry_delay`]), and a request that went UNANSWERED is remembered apart
//! from a 404 ([`RemoteFiles::take_problems`]). The chain cannot tell those two
//! apart — `read` answers with an `Option` — and a sweep that recorded a rate
//! limit as "KiCad has no model for this part" would never look at it again.
//!
//! # Blocking on purpose
//!
//! [`KicadFiles::read`] is synchronous, so [`RemoteFiles`] fetches with
//! `ehttp::fetch_blocking`. Every caller runs it on a worker thread —
//! [`read_part`] here, and the window's own library reader — and takes the
//! answer through a channel the frame loop drains, the shape
//! `panels/step_parts.rs` and `kicad_import.rs`'s STEP reader already use.
//! Nothing this module does is quick enough for a frame: KiCad 9.0.9.1's
//! largest symbol library is 15.4 MB and a whole part's chain measured 1.16 s
//! of download and parse. That is also why this module is native-only: the
//! browser has no blocking fetch, and no filesystem to cache into.

#![cfg(not(target_arch = "wasm32"))]

use super::kicad_import::{
    part_from_symbol, FootprintChoice, KicadFiles, KicadLibrary, KicadPart, Folder,
};
use brep_ecad_core::Symbol;
use serde_json::Value;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Mutex;

/// GitLab's REST root. The public API needs no token; it is rate-limited to 500
/// requests a minute per address (`ratelimit-name: throttle_unauthenticated_api`),
/// which one import's handful of files never approaches.
pub const API: &str = "https://gitlab.com/api/v4";

/// The release the fetch reads. A 9.x tag, because 9.x is the format this
/// importer parses; see the module doc.
pub const DEFAULT_TAG: &str = "9.0.9.1";

/// Which of KiCad's three library repositories a file lives in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Repo {
    Symbols,
    Footprints,
    Packages3D,
}

impl Repo {
    /// The GitLab project path, which is what the API addresses a project by.
    pub fn project(self) -> &'static str {
        match self {
            Repo::Symbols => "kicad/libraries/kicad-symbols",
            Repo::Footprints => "kicad/libraries/kicad-footprints",
            Repo::Packages3D => "kicad/libraries/kicad-packages3D",
        }
    }

    /// The folder this repository's files are cached in, which is also the name
    /// KiCad's own share directory gives them.
    pub fn folder(self) -> &'static str {
        match self {
            Repo::Symbols => "symbols",
            Repo::Footprints => "footprints",
            Repo::Packages3D => "3dmodels",
        }
    }
}

/// Percent-encode one path segment for a GitLab route: everything outside RFC
/// 3986's unreserved set, `/` included, because the file routes take the whole
/// path as ONE segment.
pub fn encode(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    for byte in value.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => out.push(byte as char),
            _ => out.push_str(&format!("%{byte:02X}")),
        }
    }
    out
}

/// The URL of one file's raw bytes at `tag`.
pub fn raw_url(repo: Repo, tag: &str, path: &str) -> String {
    format!(
        "{API}/projects/{}/repository/files/{}/raw?ref={}",
        encode(repo.project()),
        encode(path),
        encode(tag)
    )
}

/// The URL of one page of a directory listing. `path` is empty for the
/// repository root. 100 is the API's page cap.
pub fn tree_url(repo: Repo, tag: &str, path: &str, page: u32) -> String {
    format!(
        "{API}/projects/{}/repository/tree?ref={}&path={}&per_page=100&page={}",
        encode(repo.project()),
        encode(tag),
        encode(path),
        page.max(1)
    )
}

/// The entry names of one `tree` page, and whether the header says another page
/// follows. The API answers `[{"name":…,"type":"blob"|"tree"},…]`, and an error
/// as `{"message":…}`.
pub fn parse_tree(body: &str) -> Result<Vec<String>, String> {
    let value: Value = serde_json::from_str(body).map_err(|error| format!("bad listing: {error}"))?;
    if let Some(message) = value.get("message").and_then(Value::as_str) {
        return Err(message.to_owned());
    }
    let entries = value.as_array().ok_or("the listing is not an array")?;
    Ok(entries
        .iter()
        .filter_map(|entry| entry.get("name").and_then(Value::as_str))
        .map(str::to_owned)
        .collect())
}

/// The `*.kicad_sym` library names a listing of the symbols repository's root
/// offers, without their extension and sorted — the list the download path
/// shows first.
pub fn symbol_libraries(names: &[String]) -> Vec<String> {
    let mut found: Vec<String> = names
        .iter()
        .filter_map(|name| name.strip_suffix(".kicad_sym"))
        .map(str::to_owned)
        .collect();
    found.sort();
    found
}

// ============================================================================
// The files
// ============================================================================

/// What fetches one URL's bytes. A trait object so the tests drive the whole
/// chain without a network.
pub type Fetcher = Box<dyn Fn(&str) -> Result<Vec<u8>, String> + Send + Sync>;

/// How many times one URL is asked for before a "come back later" becomes an
/// error. A bulk sweep is thousands of requests against an unauthenticated
/// limit of 500 a minute, so a 429 has to be waited out rather than recorded as
/// a failure — see [`retry_delay`].
pub const FETCH_ATTEMPTS: u32 = 4;

/// How long to wait before asking again, or `None` when the answer is final.
///
/// Only a 429 and a 5xx are worth waiting out; a 404 is the corpus telling the
/// truth about itself, and every other 4xx is this code's fault. GitLab's
/// `retry-after` is honoured when it is a plain seconds count, else the wait
/// doubles per attempt, and either way it is clamped to a minute so a wedged
/// server cannot park a sweep for an hour.
pub fn retry_delay(status: u16, retry_after: Option<&str>, attempt: u32) -> Option<std::time::Duration> {
    if status != 429 && !(500..600).contains(&status) {
        return None;
    }
    if attempt + 1 >= FETCH_ATTEMPTS {
        return None;
    }
    let seconds = retry_after
        .and_then(|value| value.trim().parse::<u64>().ok())
        .unwrap_or(1u64 << attempt.min(6));
    Some(std::time::Duration::from_secs(seconds.clamp(1, 60)))
}

/// Whether an error message from a [`Fetcher`] means the file is not there, as
/// against the request not having been answered.
///
/// This is a STRING test because [`Fetcher`] answers with one, and it is load
/// bearing: a bulk sweep records "KiCad has no model for this" as a settled
/// outcome it never retries, and a 429 or a dropped connection as a failure it
/// does. Getting the two the wrong way round writes lies into the sweep's
/// ledger, so [`http_fetcher`] and the tests' stub both spell a miss
/// `HTTP 404 …`.
pub fn looks_missing(error: &str) -> bool {
    // `HTTP 404 …` is what `http_fetcher` and the tests' stub say. `404 Tree Not
    // Found` is what GitLab puts in the BODY of a listing it does not have,
    // which `parse_tree` surfaces as the message.
    error.contains("HTTP 404") || error.contains("404 Tree Not Found") || error.contains("404 File Not Found")
}

/// `ehttp`'s blocking GET, which is what the app uses in earnest, waiting out a
/// 429 or a 5xx as [`retry_delay`] says.
pub fn http_fetcher() -> Fetcher {
    Box::new(|url: &str| http_get(url, &|delay| std::thread::sleep(delay)))
}

/// One URL, retried as [`retry_delay`] says. `wait` is a seam so a test can
/// pin the retry without sleeping.
pub fn http_get(url: &str, wait: &dyn Fn(std::time::Duration)) -> Result<Vec<u8>, String> {
    let mut attempt = 0;
    loop {
        let response = ehttp::fetch_blocking(&ehttp::Request::get(url)).map_err(|error| error.to_string())?;
        if response.ok {
            return Ok(response.bytes);
        }
        match retry_delay(response.status, response.headers.get("retry-after"), attempt) {
            Some(delay) => {
                wait(delay);
                attempt += 1;
            }
            None => return Err(format!("HTTP {} {}", response.status, response.status_text)),
        }
    }
}

/// KiCad's libraries as a [`KicadFiles`]: a write-through disk cache in front
/// of the three repositories.
///
/// Every path this is asked for is a path INSIDE [`root`](Self::root), because
/// the [`KicadLibrary`] it hands the chain names folders under it. A hit is
/// read from disk; a miss is fetched, written to disk, and read from there, so
/// a fetch happens once per file per cache.
pub struct RemoteFiles {
    root: PathBuf,
    tag: String,
    fetch: Fetcher,
    /// Directory listings already fetched, so one review does not list a
    /// `*.pretty` twice.
    listings: Mutex<HashMap<String, Vec<String>>>,
    /// The one `*.pretty` a footprint listing is narrowed to, when the user has
    /// chosen a library. A generic symbol's footprint search over the WHOLE
    /// repository is one tree call per library — 165 of them — where a narrowed
    /// one is two; the download path always narrows, and says so.
    scope: Option<String>,
    /// Every file this fetched, newest last, for the dialog's status line.
    fetched: Mutex<Vec<String>>,
    /// How many bytes those fetches brought down.
    bytes: Mutex<u64>,
    /// Every request that was NOT answered — a 429 that outlived its retries, a
    /// 5xx, a dropped connection, a listing that came back as an API error.
    ///
    /// A 404 is not one of these. [`KicadFiles::read`] answers with an `Option`,
    /// so the chain cannot tell "KiCad has no model here" from "the network did
    /// not answer" and reports both as a missing file; a bulk sweep MUST tell
    /// them apart, because the first is a settled outcome and the second is a
    /// failure to retry. This is where the difference is kept.
    problems: Mutex<Vec<String>>,
}

impl RemoteFiles {
    /// Read and write under `root`, fetching from `tag`.
    pub fn new(root: impl Into<PathBuf>, tag: impl Into<String>, fetch: Fetcher) -> Self {
        Self {
            root: root.into(),
            tag: tag.into(),
            fetch,
            listings: Mutex::new(HashMap::new()),
            scope: None,
            fetched: Mutex::new(Vec::new()),
            bytes: Mutex::new(0),
            problems: Mutex::new(Vec::new()),
        }
    }

    /// Narrow footprint listings to one `*.pretty` library.
    pub fn scoped_to(mut self, library: impl Into<String>) -> Self {
        self.scope = Some(library.into());
        self
    }

    /// The library whose folders address this cache. Handing it to
    /// `part_from_symbol` is what makes the remote chain the local one.
    pub fn library(&self) -> KicadLibrary {
        let folder = |repo: Repo| Folder {
            path: self.path_of(repo).to_string_lossy().into_owned(),
            from: format!("KiCad's {} library, {}", repo.project().rsplit('/').next().unwrap_or(""), self.tag),
        };
        KicadLibrary { footprints: folder(Repo::Footprints), models: folder(Repo::Packages3D) }
    }

    /// Where one repository's files are cached.
    pub fn path_of(&self, repo: Repo) -> PathBuf {
        self.root.join(repo.folder())
    }

    /// The files fetched so far, in the order they were fetched.
    pub fn fetched(&self) -> Vec<String> {
        self.fetched.lock().map(|list| list.clone()).unwrap_or_default()
    }

    /// The files fetched since the last time this was asked, emptying the list.
    /// A sweep of thousands of parts drains per part rather than keeping every
    /// path it ever downloaded.
    pub fn take_fetched(&self) -> Vec<String> {
        self.fetched.lock().map(|mut list| std::mem::take(&mut *list)).unwrap_or_default()
    }

    /// How many bytes came down the wire, cache hits excluded.
    pub fn fetched_bytes(&self) -> u64 {
        self.bytes.lock().map(|count| *count).unwrap_or(0)
    }

    /// The requests that went unanswered since this was last asked, emptying
    /// the list. Non-empty means what the chain just read cannot be trusted as
    /// KiCad's answer — see [`problems`](Self::problems).
    pub fn take_problems(&self) -> Vec<String> {
        self.problems.lock().map(|mut list| std::mem::take(&mut *list)).unwrap_or_default()
    }

    /// Note one unanswered request, unless it was a plain 404.
    fn note_problem(&self, what: &str, error: &str) {
        if looks_missing(error) {
            return;
        }
        if let Ok(mut list) = self.problems.lock() {
            if list.len() < 64 {
                list.push(format!("{what}: {error}"));
            }
        }
    }

    /// Make the three cache folders, so a listing of an empty cache is an empty
    /// folder rather than a missing one (`is_dir` gates the footprint chooser).
    pub fn prepare(&self) -> Result<(), String> {
        for repo in [Repo::Symbols, Repo::Footprints, Repo::Packages3D] {
            std::fs::create_dir_all(self.path_of(repo))
                .map_err(|error| format!("the KiCad cache folder {} was not made: {error}", self.path_of(repo).display()))?;
        }
        Ok(())
    }

    /// Fetch one repository file into the cache and answer with its bytes. Used
    /// for the symbol library the user picks, which starts the chain.
    pub fn fetch_file(&self, repo: Repo, path: &str) -> Result<Vec<u8>, String> {
        let local = self.path_of(repo).join(path);
        if let Ok(bytes) = std::fs::read(&local) {
            return Ok(bytes);
        }
        let bytes = (self.fetch)(&raw_url(repo, &self.tag, path)).map_err(|error| {
            self.note_problem(&format!("{}/{path}", repo.folder()), &error);
            format!("{}: {error}", repo.project())
        })?;
        if let Ok(mut count) = self.bytes.lock() {
            *count += bytes.len() as u64;
        }
        if let Some(dir) = local.parent() {
            std::fs::create_dir_all(dir).map_err(|error| format!("{}: {error}", dir.display()))?;
        }
        std::fs::write(&local, &bytes).map_err(|error| format!("{}: {error}", local.display()))?;
        if let Ok(mut list) = self.fetched.lock() {
            list.push(format!("{}/{path}", repo.folder()));
        }
        Ok(bytes)
    }

    /// One repository directory's entry names, fetched once and remembered.
    fn tree(&self, repo: Repo, path: &str) -> Vec<String> {
        let key = format!("{}/{path}", repo.folder());
        if let Some(cached) = self.listings.lock().ok().and_then(|map| map.get(&key).cloned()) {
            return cached;
        }
        let mut names = Vec::new();
        // A listing this walked only PART of the way is not this directory's
        // contents, and caching it would make every later question about that
        // directory answer from the truncated list. Only a walk that ran to a
        // short page is remembered.
        let mut whole = false;
        // The API caps a page at 100 and does not say how many there are in the
        // body; a short page is the last one.
        for page in 1..=64 {
            let body = match (self.fetch)(&tree_url(repo, &self.tag, path, page)) {
                Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
                Err(error) => {
                    self.note_problem(&format!("listing {key}"), &error);
                    break;
                }
            };
            match parse_tree(&body) {
                Ok(page_names) => {
                    let short = page_names.len() < 100;
                    names.extend(page_names);
                    if short {
                        whole = true;
                        break;
                    }
                }
                Err(error) => {
                    self.note_problem(&format!("listing {key}"), &error);
                    break;
                }
            }
        }
        if whole {
            if let Ok(mut map) = self.listings.lock() {
                map.insert(key, names.clone());
            }
        }
        names
    }

    /// Split a path the chain asked for into the repository it belongs to and
    /// the path inside it. Separators are normalised because `Path::join` uses
    /// the host's, and a repository path is always `/`.
    fn locate(&self, path: &str) -> Option<(Repo, String)> {
        let path = path.replace('\\', "/");
        let root = self.root.to_string_lossy().replace('\\', "/");
        let rest = path.strip_prefix(&root)?.trim_start_matches('/');
        for repo in [Repo::Symbols, Repo::Footprints, Repo::Packages3D] {
            if let Some(inside) = rest.strip_prefix(repo.folder()) {
                let inside = inside.trim_start_matches('/');
                return Some((repo, inside.to_owned()));
            }
        }
        None
    }
}

impl KicadFiles for RemoteFiles {
    fn read(&self, path: &str) -> Option<Vec<u8>> {
        let (repo, inside) = self.locate(path)?;
        if inside.is_empty() {
            return None;
        }
        self.fetch_file(repo, &inside).ok()
    }

    fn is_file(&self, path: &str) -> bool {
        // A `.wrl` beside a missing `.step` is the one question the chain asks
        // about a file it does not want to read; answering it from the listing
        // costs one tree call the chain is about to make anyway.
        let Some((repo, inside)) = self.locate(path) else {
            return false;
        };
        if self.path_of(repo).join(&inside).is_file() {
            return true;
        }
        let (dir, name) = inside.rsplit_once('/').unwrap_or(("", inside.as_str()));
        self.tree(repo, dir).iter().any(|entry| entry == name)
    }

    fn is_dir(&self, path: &str) -> bool {
        let Some((repo, inside)) = self.locate(path) else {
            return false;
        };
        if inside.is_empty() || self.path_of(repo).join(&inside).is_dir() {
            return true;
        }
        let (dir, name) = inside.rsplit_once('/').unwrap_or(("", inside.as_str()));
        self.tree(repo, dir).iter().any(|entry| entry == name)
    }

    fn list(&self, dir: &str) -> Vec<String> {
        let Some((repo, inside)) = self.locate(dir) else {
            return Vec::new();
        };
        if repo == Repo::Footprints && inside.is_empty() {
            if let Some(library) = &self.scope {
                return vec![library.clone()];
            }
        }
        self.tree(repo, &inside)
    }
}

// ============================================================================
// One import, off the UI thread
// ============================================================================

/// A part being read from KiCad's libraries: the chain, the STEP parse and
/// every fetch, all on one worker thread.
pub struct PartReader {
    answer: std::sync::mpsc::Receiver<(KicadPart, Vec<String>)>,
}

impl PartReader {
    /// The answer, once the worker has one. `None` while it works; a worker
    /// that died without one answers with a note rather than hanging.
    pub fn take(&self) -> Option<(KicadPart, Vec<String>)> {
        match self.answer.try_recv() {
            Ok(answer) => Some(answer),
            Err(std::sync::mpsc::TryRecvError::Empty) => None,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => Some((
                KicadPart {
                    notes: vec!["the KiCad download stopped without an answer".into()],
                    ..KicadPart::default()
                },
                Vec::new(),
            )),
        }
    }
}

/// Where one import's files come from: a KiCad install on disk, or KiCad's
/// repositories. The window offers one of each, and [`read_part`] reads either
/// the same way.
pub trait PartSource: KicadFiles + Send + 'static {
    /// The folders the chain follows a footprint and a model through.
    fn library(&self) -> KicadLibrary;
    /// What this had to download, for the status line. A local install
    /// downloads nothing.
    fn downloaded(&self) -> Vec<String> {
        Vec::new()
    }
}

impl PartSource for RemoteFiles {
    fn library(&self) -> KicadLibrary {
        RemoteFiles::library(self)
    }
    fn downloaded(&self) -> Vec<String> {
        self.fetched()
    }
}

/// Follow `symbol`'s chain through `source` on a worker thread: the footprint,
/// the 3D model, and the model's STEP parsed. Both halves are slow for their
/// own reason — the download waits on GitLab, the local read waits on the STEP
/// parser (529 ms for a real SOIC-8) — so both go off the UI thread.
pub fn read_part(symbol: Symbol, choice: FootprintChoice, source: impl PartSource) -> PartReader {
    let (send, answer) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let library = source.library();
        let part = part_from_symbol(symbol, &choice, &library, &source).parsed();
        // A window closed meanwhile has dropped the receiver; nobody waits.
        let _ = send.send((part, source.downloaded()));
    });
    PartReader { answer }
}

/// Where a fetched library is kept when the user names no folder: beside the
/// app's own state, so it survives a session and a user can point the LOCAL
/// path at it afterwards. The same `<config>/brep-app` base the model store
/// uses (`store::native_model::FileModelStore::new`), one folder along.
pub fn default_cache_root() -> PathBuf {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".config")))
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("brep-app").join("kicad-library")
}

