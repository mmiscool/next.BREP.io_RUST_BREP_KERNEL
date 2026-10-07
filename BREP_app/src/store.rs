//! The persistent-storage / filesystem seam — the ONE platform exception in
//! the engine-native UI. Settings, dock layout, and model documents all use the
//! same [`ModelStore::read`] / [`ModelStore::write`] API. Only its backend differs:
//! desktop writes files; the browser mirrors an ASYNC, swappable
//! [`mirror_store::StoreBackend`] — IndexedDB today, a remote server next — into
//! memory and writes behind it, which is what keeps this whole API synchronous.
//!
//! Desktop reads the real filesystem on every call. The browser CANNOT: IndexedDB
//! has no synchronous read on the main thread, so "go to the live filesystem every
//! time" is not implementable there — the mirror is not a convenience, it is the
//! only way this API can be synchronous at all. What the mirror owes in exchange
//! is that it never falls behind the storage it stands for, and the key space has
//! MORE than one writer: every other tab of the same origin. So each tab announces
//! the key it committed and re-reads the keys it is told about
//! ([`mirror_store::MirrorStore::refresh_key`], over the `brep-app:store` broadcast
//! in [`web_model`]). Save a part in one tab and the tab holding the assembly can
//! insert it — no reload.

/// Reserved persistent-object names used internally for application state.
pub const SETTINGS_KEY: &str = "@settings";
pub const FEATURE_PALETTE_DISPLAY_KEY: &str = "@feature_palette_display";
pub const DOCK_LAYOUT_KEY: &str = "@dock_layout";
/// Pinned explorer locations (a JSON array of navigable location strings),
/// persisted through the ordinary `read`/`write` CRUD like the other reserved
/// keys so the sidebar needs no dedicated trait surface.
pub const PINNED_KEY: &str = "@pinned";
/// The AUTOSAVE blob: every open document that had unsaved changes the last
/// time the shell's autosave fired (see `crate::recovery`), so a crash, a
/// closed tab or a reload can offer that work back. Written debounced, removed
/// the moment nothing is dirty; never listed as a document.
pub const RECOVERY_KEY: &str = "@recovery";
/// The KiCad library folders the KiCad import last used (`panels::kicad_import`).
/// Desktop only: the web build cannot read a KiCad library, so it never writes it.
pub const KICAD_LIBRARY_KEY: &str = "@kicad_library";
/// The adoption importer's ledgers (`plm::adoption::ImportLedgers`), per PLM
/// server and per imported folder: what a stopped import resumes from. It
/// never leaves the machine — it records THIS machine's runs against a
/// server, and the source tree it describes is on this machine.
pub const PLM_IMPORT_KEY: &str = "@plm_import";
/// Explicitly installed executable packages stay on this machine, including in PLM mode.
pub const PLUGINS_KEY: &str = "@plugins";
/// Explicitly saved JavaScript editor source, local even in PLM mode.
pub const JAVASCRIPT_DRAFT_KEY: &str = "@javascript_draft";
/// Recent document identities, shared through the user preference store.
pub const RECENT_DOCUMENTS_KEY: &str = "@recent_documents";

/// The reserved names that are a user's PREFERENCES (plm-cad-integration-todo
/// D1): state that follows the user, not the machine. A file backend keeps them
/// where it always has; a PLM backend serves them from its per-user store (P5).
///
/// `@pinned` is listed here as what it IS — explorer FOLDER locations, not
/// document identities — so in PLM mode it pins workspace folders (S14), never
/// `part/…/rev/…` keys.
pub const PREFERENCE_KEYS: &[&str] = &[
    SETTINGS_KEY,
    DOCK_LAYOUT_KEY,
    PINNED_KEY,
    FEATURE_PALETTE_DISPLAY_KEY,
    KICAD_LIBRARY_KEY,
    RECENT_DOCUMENTS_KEY,
];

/// The reserved names that never have to leave the machine to be safe: the
/// local-first copy of `@recovery` — a PLM session MIRRORS it to the server
/// after the local write (D1, S1's composition); the local copy is what a
/// network drop cannot take away — and `@plm_import`, which is never
/// mirrored.
pub const LOCAL_KEYS: &[&str] = &[RECOVERY_KEY, PLM_IMPORT_KEY, PLUGINS_KEY, JAVASCRIPT_DRAFT_KEY];

/// Which of the three key spaces a store name belongs to (D1, the round 10
/// answer). Every name is exactly one of them:
///
/// * **Documents** — model documents; the PLM's store routes
///   (`part/<part>/rev/<revision>`), a file path or a browser `/models/...`
///   name on the file backends.
/// * **Preferences** — [`PREFERENCE_KEYS`].
/// * **Local** — [`LOCAL_KEYS`].
///
/// The browser's explorer DIRECTORY rows are none of the three: they belong to
/// the file backends only (a PLM user's folders are the workspace, S14), so they
/// never reach this classifier — see `mirror_store::key_space`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeySpace {
    Documents,
    Preferences,
    Local,
}

impl KeySpace {
    /// The key space of a store NAME (what callers pass to
    /// [`ModelStore::read`] / [`ModelStore::write`]).
    pub fn of(name: &str) -> Self {
        if PREFERENCE_KEYS.contains(&name) {
            KeySpace::Preferences
        } else if LOCAL_KEYS.contains(&name) {
            KeySpace::Local
        } else {
            KeySpace::Documents
        }
    }
}

/// How much of a document a store holds right now — the synchronous answer to
/// "can `read` give me this?" that a store with an index-only hydrate owes.
///
/// A store that holds everything it lists (the native filesystem, a browser
/// origin hydrated whole) only ever answers `Absent` or `Resident`. A store
/// hydrated from an INDEX (the mirror's `OnDisk` entries — the PLM catalog is
/// the corpus that needs it) also answers `OnDisk`: the name exists and its
/// size is known, but its bytes are still on the backend, and a `read` of it
/// returns `None` while it asks for them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Residency {
    Absent,
    OnDisk,
    Resident,
}

/// A document as a caller can use it this frame: its bytes, still loading,
/// or not there ([`read_now`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReadNow {
    Ready(String),
    Loading,
    Absent,
}

/// Read `name`, telling "not loaded yet" from "gone".
///
/// Not simply `read` then `residency`: a `read` of an `OnDisk` entry asks the
/// backend for the bytes, and a backend that answers at once (an in-process
/// router, a local cache) lands them INSIDE that read. `read` has already
/// said `None`, and `residency` now says `Resident`. Read naively, that is a
/// document reported missing that is there. So a `None` that turned resident
/// is read again.
pub fn read_now(store: &dyn ModelStore, name: &str) -> ReadNow {
    if let Some(contents) = store.read(name) {
        return ReadNow::Ready(contents);
    }
    match store.residency(name) {
        Residency::Resident => store.read(name).map_or(ReadNow::Absent, ReadNow::Ready),
        Residency::OnDisk => ReadNow::Loading,
        Residency::Absent => ReadNow::Absent,
    }
}

/// A file selected outside the model store. Keeping bytes verbatim allows the
/// same upload channel to carry binary STL as well as text CAD formats.
pub struct ImportedFile {
    pub name: String,
    pub bytes: Vec<u8>,
}

/// One entry exposed to the common in-application filesystem browser.
#[derive(Clone, Debug, PartialEq)]
pub struct BrowserEntry {
    pub name: String,
    pub identity: String,
    pub is_dir: bool,
    /// File size in bytes for the Size column. `None` for directories or where
    /// the backend cannot report it.
    pub size: Option<u64>,
    /// Last-modified time as whole Unix seconds for the Date column. `None` where
    /// unavailable — the browser key/value backends store values, not timestamps.
    pub modified: Option<f64>,
}

/// A quick-access destination for the explorer's left sidebar.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BrowserPlace {
    /// Human label shown in the sidebar (e.g. `"Home"`, `"Models"`, a disk name).
    pub label: String,
    /// The navigable location — pass to [`ModelStore::browser_navigate`].
    pub location: String,
    /// Which built-in glyph the widget draws for this place.
    pub kind: PlaceKind,
}

/// The category of a [`BrowserPlace`], so the widget owns the icon styling.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlaceKind {
    Home,
    Documents,
    Downloads,
    Models,
    Root,
}
// A MODEL is a whole document
// (the engine-owned `HistoryRequest` JSON — one `.nbrep` recipe), and the
// file panel needs to *enumerate*, read, and write NAMED documents, plus (in the
// browser) hand a file to / take a file from the user's real filesystem. The same
// trait also stores reserved application-state blobs. Platform-specific behavior
// remains behind two `#[cfg]`-gated implementations.
//
// The trait is deliberately a **named-document CRUD** (`list` / `read` / `write`
// / `remove`) plus a poll-based **file-interchange** side-channel. That shape is
// exactly what a later **GitHub backend** needs: `list` = a repo directory
// listing, `read` = fetch a file's contents, `write` = create/update (commit)
// a file, `remove` = delete a file — the model name is the path within the repo.
// An async backend (GitHub over HTTP, or the browser File System Access API /
// OPFS whose main-thread API is async) slots in behind the SAME trait by driving
// its request on a background task and surfacing the result through the
// `begin_import` → `take_import` poll pattern used here for uploads, so the
// synchronous panel code never changes. GitHub itself is a deferred follow-up;
// this lands the seam it plugs into.

/// The media type an exported file is handed to the user with, from its own
/// extension.
///
/// A browser download IS its media type: the type on the `Blob` is what decides
/// whether the file opens in a viewer, is offered as a download, or is mailed on
/// correctly — the name alone is a hint. So the two export doors below take it
/// from here rather than each hard-coding one, which is what they used to do
/// (`model/gltf-binary` on everything the byte lane carried, because GLB was the
/// only thing it carried, and `application/octet-stream` on every text export).
///
/// Unknown extensions fall back to `application/octet-stream`, which is the
/// correct "some bytes, download them" answer.
pub fn media_type(file_name: &str) -> &'static str {
    let extension = std::path::Path::new(file_name)
        .extension()
        .map(|extension| extension.to_string_lossy().to_ascii_lowercase())
        .unwrap_or_default();
    match extension.as_str() {
        "pdf" => "application/pdf",
        "svg" => "image/svg+xml",
        "glb" => "model/gltf-binary",
        "json" => "application/json",
        // The PCB's fabrication bundle.
        "zip" => "application/zip",
        // A model document is JSON inside, but typed as opaque bytes: a browser
        // that sees a JSON type on a download may "correct" the name to
        // `bracket.nbrep.json`, and the name is the only thing that routes it.
        "nbrep" | "fbrep" | "tbrep" => "application/octet-stream",
        // The CAD text formats. `model/step+zip` is the registered type for a
        // zipped STEP; a plain `.step` file is `model/step`.
        "step" | "stp" => "model/step",
        "iges" | "igs" => "model/iges",
        "stl" => "model/stl",
        "obj" => "model/obj",
        "dxf" => "image/vnd.dxf",
        "csv" => "text/csv",
        "txt" => "text/plain",
        _ => "application/octet-stream",
    }
}

/// The application's single persistent-storage abstraction. Reserved application
/// keys and named model documents are read and written through the same methods.
/// All methods take `&self`; mutable backend state lives behind interior mutability.
pub trait ModelStore {
    /// A short human label of where documents persist, for the panel header
    /// (e.g. `"filesystem: ~/.config/brep-app/models"` or `"browser storage"`).
    fn backend_label(&self) -> String {
        "persistent storage".into()
    }

    /// The names of the documents currently available to **Open** (bare names,
    /// no extension). May be empty on a backend that cannot enumerate — then the
    /// panel falls back to the name field / import.
    fn list(&self) -> Vec<String> {
        Vec::new()
    }

    /// Read a stored document by name, or `None` if absent/unreadable.
    fn read(&self, name: &str) -> Option<String>;

    /// Create or overwrite the document `name` with `contents`. `Err` carries a
    /// message the panel surfaces in its status line.
    fn write(&self, name: &str, contents: &str) -> Result<(), String>;

    /// Delete the document `name` (best-effort; `Ok` if it is already gone).
    fn remove(&self, _name: &str) -> Result<(), String> {
        Ok(())
    }

    /// A monotonic count of this store's MUTATIONS — every `write`, `remove`
    /// and `browser_write` that landed, WHOEVER made it. A read never moves
    /// it; two reads that agree mean nothing was written in between.
    ///
    /// This is a cache-invalidation key, which is why it is REQUIRED rather
    /// than defaulted: a default would let a new backend (the GitHub one this
    /// trait is shaped for) compile while silently never invalidating
    /// anything, which is the failure it exists to stop. The outdated-parts
    /// badge (`panels::update_components`) used to key on the file dialog's
    /// own save counter, which counts Save / Save As / native Save-As and
    /// NOTHING else — so a KiCad re-import (`panels::kicad_bulk::write_part`)
    /// or the explorer's delete row could change a part's source under an open
    /// assembly, move no other generation, and leave the badge dark.
    fn mutation_generation(&self) -> u64;

    /// Whether `name` is absent, listed but not yet loaded, or readable now
    /// (see [`Residency`]). The default answers from `read`, which is exact for
    /// every store that holds what it lists; only an index-hydrated store
    /// overrides it. No caller branches on `OnDisk` yet — it is here so one
    /// CAN tell "not loaded yet" from "gone" without a second store API.
    fn residency(&self, name: &str) -> Residency {
        match self.read(name) {
            Some(_) => Residency::Resident,
            None => Residency::Absent,
        }
    }

    /// What kind of identity `identity` is on THIS store (see
    /// [`DocumentIdentity`] and the enumeration above it). Every file backend
    /// keys by path, so the default never parses: a browser folder named
    /// `part/7/rev` holds a document, not a revision.
    fn identity(&self, identity: &str) -> DocumentIdentity {
        DocumentIdentity::Path(identity.to_string())
    }

    /// The name a person sees for `identity`: the tab title, the name field.
    /// The default is the file rule, [`model_display_name`]; a PLM store knows
    /// the part number and revision label from its index, which the key alone
    /// does not carry.
    fn display_name(&self, identity: &str) -> String {
        model_display_name(identity)
    }

    /// The identity of `file_name` beside the document `identity` — or `None`
    /// on a store with no folders to be beside (the PLM, where a family member
    /// or a spun-out part is a new part instead, S7 / S9). The default is the
    /// file rule, [`sibling_identity`].
    fn sibling_of(&self, identity: &str, file_name: &str) -> Option<String> {
        Some(sibling_identity(identity, file_name))
    }

    /// The ONE spelling of the document `name` resolves to, as the explorer
    /// spells it (`BrowserEntry::identity`), so two names for one document
    /// compare equal: a bare `bolt` and `/home/u/.config/brep-app/models/bolt.nbrep`
    /// on the desktop, `sub/bolt` and `/models/sub/bolt.nbrep` in the browser.
    ///
    /// The adoption importer (S13) needs it. An assembly's `sourceKey`s are
    /// whatever name the part was inserted by, and the importer must match
    /// each one to the file it walked. The default is `name` unchanged, which
    /// is right for a store with one spelling per document (a PLM store's
    /// revision keys).
    fn canonical_identity(&self, name: &str) -> String {
        name.to_string()
    }

    /// Persistence failures that surfaced AFTER a `write` / `remove` already
    /// returned `Ok` — the price of a WRITE-BEHIND backend (see
    /// [`mirror_store`]). The app shell drains this once per frame into the toast
    /// overlay, so a save that never reached storage is never silent: the
    /// in-memory copy means nothing is lost mid-session, but the user has to know
    /// it will not survive a reload. Empty on a backend that writes synchronously
    /// (native files), which reports through `write`'s `Err` instead.
    fn take_persistence_errors(&self) -> Vec<String> {
        Vec::new()
    }

    /// The PLM this session is on, or is configured for and not on (why, when
    /// known): what the Settings PLM tab shows (plan S1 part 3). `None` is the
    /// file-based app, and the tab is absent.
    fn plm_session(&self) -> Option<crate::plm::connection::Session> {
        None
    }

    /// The signed-in PLM client of a session whose store IS the PLM, for the
    /// calls that are not store reads and writes (Generate, spin-out, the
    /// lifecycle verbs). `None` on every file store, and on a PLM configured
    /// but not connected.
    fn plm_client(&self) -> Option<std::rc::Rc<crate::plm::client::PlmClient>> {
        None
    }

    /// What the PLM's store index says of revision `key` (`part/…/rev/…`):
    /// its part number, label, lifecycle and lock, as the session last read
    /// them. `None` on every file store and for a key the index does not list.
    /// No request: this is the index the session already holds.
    fn plm_revision(&self, key: &str) -> Option<crate::plm::client::IndexEntry> {
        let _ = key;
        None
    }

    /// Bring the held index up to date after the change feed moved: `keys`
    /// are the revisions the feed named, `stale` means it could not say. A
    /// no-op on every file store.
    fn refresh_plm_index(&self, keys: &[String], stale: bool) {
        let _ = (keys, stale);
    }

    /// Remember a document that has already been durably written through a PLM
    /// request. Update the session mirror without issuing a second write.
    fn remember_saved_document(&self, name: &str, contents: &str) {
        let _ = (name, contents);
    }

    /// This machine's own files, browsable, for the file chooser (any file,
    /// any folder): the native file store itself, or the file store a PLM
    /// session keeps beside the server. `None` in the browser, which picks
    /// through [`Self::begin_pick_file`] instead.
    fn local_files(&self) -> Option<&dyn ModelStore> {
        None
    }

    /// The browser's file chooser for ANY file (not the model lane's
    /// `.nbrep` input): the choice arrives later through
    /// [`Self::take_picked_file`], under the file's own name.
    fn begin_pick_file(&self) -> Result<(), String> {
        Err("this platform has no file chooser".into())
    }

    /// The file [`Self::begin_pick_file`]'s chooser delivered, once.
    fn take_picked_file(&self) -> Option<ImportedFile> {
        None
    }

    /// Writes handed to a WRITE-BEHIND backend that have not settled yet — the
    /// status bar's working indicator reports them ("Saving to browser
    /// storage"). Always 0 on a backend that writes synchronously.
    fn pending_writes(&self) -> usize {
        0
    }

    // --- common file-browser filesystem --------------------------------------

    /// Current directory shown by the embedded explorer.
    fn browser_location(&self) -> String {
        self.backend_label()
    }

    /// Directories and matching files at the current explorer location.
    /// Directories are never filtered. The default implementation presents the
    /// backend's named model collection as a flat virtual directory.
    fn browser_entries(&self, _extensions: &[&str]) -> Vec<BrowserEntry> {
        self.list()
            .into_iter()
            .map(|name| BrowserEntry {
                identity: name.clone(),
                name,
                is_dir: false,
                size: None,
                modified: None,
            })
            .collect()
    }

    fn browser_enter(&self, _identity: &str) -> Result<(), String> {
        Err("this storage backend has no directories".into())
    }

    fn browser_up(&self) -> Result<(), String> {
        Ok(())
    }

    fn browser_home(&self) -> Result<(), String> {
        Ok(())
    }

    fn browser_root(&self) -> Result<(), String> {
        Ok(())
    }

    /// Navigate the explorer directly to `location` — a value previously returned
    /// by [`Self::browser_location`], a [`BrowserPlace::location`], a breadcrumb
    /// ancestor, or a user-typed path. `Err` if it is not a directory this backend
    /// can browse. Powers the breadcrumb, the sidebar, back/forward, and the
    /// path-edit field. Default: unsupported.
    fn browser_navigate(&self, _location: &str) -> Result<(), String> {
        Err("this storage backend cannot navigate to a path".into())
    }

    /// Quick-access places for the explorer's left sidebar (home, documents, the
    /// models root, disks…). Default: none, and the sidebar hides itself.
    fn browser_places(&self) -> Vec<BrowserPlace> {
        Vec::new()
    }

    /// Create a child directory at the current browser location.
    fn browser_create_dir(&self, _name: &str) -> Result<(), String> {
        Err("this storage backend cannot create directories".into())
    }

    /// Write a filename at the current browser location and return its stable
    /// identity for subsequent plain Save operations.
    fn browser_write(&self, name: &str, contents: &str) -> Result<String, String> {
        self.write(name, contents)?;
        Ok(name.to_string())
    }

    /// Files available to the in-app explorer for a foreign-format import.
    /// Names retain their extension. Browser storage returns none and offers an
    /// Upload button instead; desktop enumerates its application files directory.
    fn list_external_files(&self, _extensions: &[&str]) -> Vec<String> {
        Vec::new()
    }

    /// Read one entry returned by [`Self::list_external_files`].
    fn read_external_file(&self, _name: &str) -> Option<Vec<u8>> {
        None
    }

    // --- real-file interchange (the platform "fallback") ----------------------
    // The browser cannot silently write to an arbitrary path; these methods let
    // it move a document to/from the user's real filesystem. Desktop instead
    // browses and writes its application files directory directly.

    /// Whether this backend can exchange files with the user's real filesystem
    /// (browser download+upload). The panel shows the Upload affordance only when
    /// this is `true`.
    fn supports_file_interchange(&self) -> bool {
        false
    }

    /// Hand `contents` to the user as a file named after `name` (browser: a
    /// download; native w/ dialog: a Save-As). Returns the saved document's
    /// identity — the full path the user chose on native (so the caller can
    /// re-save straight to it), the bare download name on the web — or `None`
    /// when the user cancelled. No-op (`None`) by default.
    fn export_file(&self, name: &str, contents: &str) -> Result<Option<String>, String> {
        let _ = (name, contents);
        Ok(None)
    }

    /// Begin importing a real file — opens the platform picker. The result is
    /// retrieved later via [`Self::take_import`] (upload/read is async in the
    /// browser). No-op by default.
    fn begin_import(&self) -> Result<(), String> {
        Ok(())
    }

    /// Poll for a completed import, consuming it.
    /// `None` until a `begin_import` finishes. Default: never any.
    fn take_import(&self) -> Option<ImportedFile> {
        None
    }

    // --- format-typed interchange (CAD / mesh) --------------------------------
    // The model lanes above trade the `.nbrep` recipe; import/export of a
    // foreign format (STEP text, ASCII STL) needs a DIFFERENT picker filter and
    // must NOT mangle the file extension. These two methods add that lane while
    // leaving the model lanes byte-for-byte. They share the SAME `take_import`
    // pickup channel — the panel routes the result by its filename extension.

    /// Begin importing a real file behind a specific picker `filter` (a human
    /// label + dot-less extensions, e.g. `("STEP", &["step","stp"])`). The chosen
    /// file's contents + its FULL name (extension preserved, so the panel can
    /// route it) arrive via [`Self::take_import`]. Default: reuse [`Self::begin_import`].
    fn begin_import_filtered(&self, _filter: (&str, &[&str])) -> Result<(), String> {
        self.begin_import()
    }

    /// Hand `contents` to the user under EXACTLY `file_name` (extension included,
    /// no model-extension munging) — the Save-As / download for a foreign export
    /// format. The file's media type comes from its extension
    /// ([`media_type`]). Default no-op (interchange unsupported).
    fn export_file_named(&self, _file_name: &str, _contents: &str) -> Result<(), String> {
        Ok(())
    }

    /// [`Self::export_file_named`] for a BINARY format (GLB, a sheet PDF). Every
    /// text export could go through this one — text is bytes — but the text
    /// method stays because the browser's `Blob` takes a string directly and
    /// routing UTF-8 through a `Uint8Array` for every STEP file would be a copy
    /// for nothing. Both lanes type the file the same way, from its extension,
    /// so which lane an export takes is a question about copies and never about
    /// what the user receives. Default no-op, like its text sibling.
    fn export_file_named_bytes(&self, _file_name: &str, _contents: &[u8]) -> Result<(), String> {
        Ok(())
    }
}

/// Construct the platform's one persistent store for application state and models.
/// A native file store rooted at `dir` — the seam an automation host uses to keep
/// a session's settings, autosave and recovery blob out of the user's own config
/// directory.
///
/// Laid out the way the SHIPPED app lays out its own config directory: models
/// in `<dir>/models`, reserved application state beside them in `<dir>`. A
/// session whose store collapsed the two would show the app its own
/// `dock_layout.json` and `recovery.json` as documents in the Open dialog —
/// rows a user never sees, in the one place automation exists to drive.
///
/// A `plm.json` in `dir` (what the test-mcp PLM fixture writes) makes the
/// session a PLM session instead: see [`boot_native_store`]. Only the files
/// count here — never `BREP_PLM_URL` — so a host's own environment cannot leak
/// into a session it isolated. No `plm.json` is exactly the file store of
/// before the PLM existed.
#[cfg(not(target_arch = "wasm32"))]
pub fn native_store_at(dir: std::path::PathBuf) -> Box<dyn ModelStore> {
    match crate::plm::config::resolve(&dir, &|_| None, &Default::default()) {
        Ok(None) => Box::new(native_model::FileModelStore::rooted_at(dir)),
        Ok(Some(config)) => boot_native_store(dir, config),
        Err(e) => file_store_with_notice(dir, format!("PLM config: {e}")),
    }
}

/// The file store at `root`, saying on its first frame why it is not the PLM.
#[cfg(not(target_arch = "wasm32"))]
pub fn file_store_with_notice(root: std::path::PathBuf, reason: String) -> Box<dyn ModelStore> {
    Box::new(
        native_model::FileModelStore::rooted_at(root)
            .with_notice(format!("{reason} — this session uses the file stores"))
            .with_plm_reason(reason),
    )
}

#[cfg(not(target_arch = "wasm32"))]
pub use native_boot::{boot_native_store, check_native_plm, NativeFileBackend, BOOT_TIMEOUT};

pub fn default_model_store() -> Box<dyn ModelStore> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        Box::new(native_model::FileModelStore::new())
    }
    #[cfg(target_arch = "wasm32")]
    {
        // Already built and HYDRATED by [`hydrate_web_store`] inside the async
        // wasm entry point, so this is a hand-off, not a construction.
        web_model::take_boot_store()
    }
}

/// wasm: bring up browser persistence and pull the whole key space into memory.
/// MUST be awaited before `eframe::WebRunner::start`, because everything
/// downstream of it — [`default_model_store`], every `read` in `BrepApp::new` and
/// in the frame loop — is synchronous and assumes a complete mirror.
#[cfg(target_arch = "wasm32")]
pub async fn hydrate_web_store() {
    web_model::hydrate().await;
}

/// wasm: wake the reactive frame loop when an async file upload completes (see
/// [`web_model::set_repaint_ctx`]). Re-exported here so the app shell reaches it
/// as `store::set_repaint_ctx` without knowing the platform module.
#[cfg(target_arch = "wasm32")]
pub use web_model::set_repaint_ctx;

/// The document extension for a model recipe (`<name>.nbrep`) — the NORMAL
/// part's. A bare name with no class extension means this one; a family
/// (`.fbrep`) or template (`.tbrep`) keeps its extension in its name, so its
/// class survives every round trip (see [`crate::document_class`]).
pub const MODEL_EXT: &str = ".nbrep";

/// Every model document extension, dot-less, for the explorer's filters.
pub const MODEL_EXTENSIONS: &[&str] = &["nbrep", "fbrep", "tbrep"];

/// A document file name with its class extension made lowercase, and
/// `.nbrep` added when it has none: `Bolt` -> `Bolt.nbrep`,
/// `Bolt.FBREP` -> `Bolt.fbrep`.
pub fn model_file_name(name: &str) -> String {
    let class = crate::document_class::DocumentClass::of_name(name).unwrap_or_default();
    class.file_name(name)
}

// --- Desktop: the PLM session store ---------------------------------------------
/// The native half of plan S2: the mirror over a remote backend, hydrated
/// before the first frame, exactly as the wasm entry point hydrates IndexedDB
/// before `WebRunner::start`. Built ONLY when a PLM is configured; the
/// file-based app never reaches this module.
#[cfg(not(target_arch = "wasm32"))]
mod native_boot {
    use super::mirror_store::{BackendFuture, MirrorStore, StoreBackend, DIR_PREFIX, PREFIX};
    use super::native_model::FileModelStore;
    use super::{BrowserEntry, BrowserPlace, ModelStore, Residency};
    use crate::plm::config::PlmConfig;
    use crate::plm::PlmFuture;
    use std::path::{Path, PathBuf};
    use std::rc::Rc;

    /// How long the boot waits for sign-in and the index before giving up and
    /// starting on the file stores. The window is blank while it waits, so this
    /// is a bound on a hung server, not an expected duration.
    pub const BOOT_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(20);

    /// Every reserved name this backend stores: exactly the user's preferences
    /// and the machine's local keys (D1's split, `PREFERENCE_KEYS` ∪
    /// `LOCAL_KEYS`). DERIVED, not listed: a hand-kept copy once lacked
    /// `@plm_import`, and every PLM session refused to keep its import ledger.
    fn reserved() -> impl Iterator<Item = &'static str> {
        super::PREFERENCE_KEYS.iter().chain(super::LOCAL_KEYS).copied()
    }

    /// A [`StoreBackend`] over the native file store: the mirror's keys mapped
    /// back to the names `FileModelStore` has always used, so `@recovery` lands
    /// in `<root>/recovery.json` in a PLM session exactly as it does in a file
    /// session. It is the PLM session's LOCAL key space (D1: the local-first
    /// copy of `@recovery`), handed to the PLM client to compose.
    ///
    /// Every call is synchronous file I/O answered with a resolved future, so
    /// it lands writes of one key in call order by itself.
    pub struct NativeFileBackend {
        files: FileModelStore,
    }

    impl NativeFileBackend {
        pub fn rooted_at(root: PathBuf) -> Self {
            Self { files: FileModelStore::rooted_at(root) }
        }

        /// The store name a mirror key stands for; `None` for an explorer
        /// directory row, which this backend does not hold.
        fn name_of(key: &str) -> Option<String> {
            if let Some(name) = reserved().find(|name| MirrorStore::key(name) == key) {
                return Some(name.to_string());
            }
            if key.starts_with(DIR_PREFIX) {
                return None;
            }
            key.strip_prefix(PREFIX).map(str::to_string)
        }
    }

    fn ready<T: 'static>(value: Result<T, String>) -> BackendFuture<T> {
        Box::pin(std::future::ready(value))
    }

    impl StoreBackend for NativeFileBackend {
        fn label(&self) -> String {
            self.files.backend_label()
        }

        fn load_all(&self) -> BackendFuture<Vec<(String, String)>> {
            let reserved = reserved().map(|name| name.to_string());
            let documents = self.files.list();
            let all = reserved
                .chain(documents)
                .filter_map(|name| self.files.read(&name).map(|value| (MirrorStore::key(&name), value)))
                .collect();
            ready(Ok(all))
        }

        fn orders_writes_per_key(&self) -> bool {
            true
        }

        fn get(&self, key: &str) -> BackendFuture<Option<String>> {
            ready(Ok(Self::name_of(key).and_then(|name| self.files.read(&name))))
        }

        fn put(&self, key: &str, value: &str) -> BackendFuture<()> {
            ready(match Self::name_of(key) {
                Some(name) => self.files.write(&name, value),
                None => Err(format!("{key}: the local store keeps no explorer folders")),
            })
        }

        fn delete(&self, key: &str) -> BackendFuture<()> {
            ready(match Self::name_of(key) {
                Some(name) => self.files.remove(&name),
                None => Ok(()),
            })
        }
    }

    /// Connect to the configured PLM: sign in, check versions, and hand back
    /// the composed backend (documents and preferences on the server, `local`
    /// on this machine): S1's `plm::client::open_session`. A refusal is a
    /// sentence, and the session starts on the file stores saying it.
    fn plm_connect(config: PlmConfig, local: Rc<dyn StoreBackend>) -> PlmFuture<Rc<dyn StoreBackend>> {
        crate::plm::client::open_session(config, local)
    }

    /// The session store for a configured PLM, hydrated before this returns.
    /// Any failure — refused sign-in, a version the server does not serve, an
    /// index that will not load, no answer within [`BOOT_TIMEOUT`] — starts the
    /// session on the file stores at `root` instead, saying why on the first
    /// frame. A PLM never makes the app fail to start.
    pub fn boot_native_store(root: PathBuf, config: PlmConfig) -> Box<dyn ModelStore> {
        let url = config.url.clone();
        match boot_with(&root, config, plm_connect, BOOT_TIMEOUT) {
            Ok(store) => Box::new(store),
            Err(reason) => super::file_store_with_notice(root, format!("PLM at {url}: {reason}")),
        }
    }

    /// `brep-app --plm-check`: the boot's own sign-in and version check, and
    /// nothing else — no index, no store.
    pub fn check_native_plm(root: PathBuf, config: PlmConfig) -> Result<(), String> {
        let local: Rc<dyn StoreBackend> = Rc::new(NativeFileBackend::rooted_at(root));
        crate::plm::native::block_on_for(plm_connect(config, local), BOOT_TIMEOUT)
            .ok_or_else(|| format!("no answer within {} s", BOOT_TIMEOUT.as_secs()))?
            .map(|_| ())
    }

    /// [`boot_native_store`] with the connector and the time limit passed in,
    /// so a test can boot against a backend of its own.
    pub(super) fn boot_with(
        root: &Path,
        config: PlmConfig,
        connect: impl FnOnce(PlmConfig, Rc<dyn StoreBackend>) -> PlmFuture<Rc<dyn StoreBackend>>,
        timeout: std::time::Duration,
    ) -> Result<NativeMirrorStore, String> {
        use crate::plm::native::block_on_for;
        let late = || format!("no answer within {} s", timeout.as_secs());
        let local: Rc<dyn StoreBackend> = Rc::new(NativeFileBackend::rooted_at(root.to_path_buf()));
        let backend = block_on_for(connect(config, local), timeout).ok_or_else(late)??;
        // The index, not the documents: a catalog of thousands of parts must
        // not load at boot or stream into memory behind the first frame.
        // Document bodies remain OnDisk until read on demand.
        let entries = block_on_for(backend.load_index(), timeout)
            .ok_or_else(late)?
            .map_err(|e| format!("the store index would not load ({e})"))?;
        let wake: Rc<dyn Fn()> = Rc::new(crate::plm::native::request_wake);
        let core = MirrorStore::from_index(backend, entries, Some(wake));
        Ok(NativeMirrorStore { core, files: FileModelStore::rooted_at(root.to_path_buf()) })
    }

    /// The native PLM session store: the mirror (every CRUD and explorer
    /// method) plus this machine's real-file lane — a STEP or PDF export still
    /// lands in `<root>/models` and an external file is still read from disk,
    /// as in a file session.
    pub(super) struct NativeMirrorStore {
        pub(super) core: MirrorStore,
        files: FileModelStore,
    }

    impl ModelStore for NativeMirrorStore {
        // --- delegated to the mirror ------------------------------------------
        fn backend_label(&self) -> String {
            self.core.backend_label()
        }
        fn plm_client(&self) -> Option<std::rc::Rc<crate::plm::client::PlmClient>> {
            self.core.plm_client()
        }
        fn plm_revision(&self, key: &str) -> Option<crate::plm::client::IndexEntry> {
            self.core.plm_revision(key)
        }
        fn refresh_plm_index(&self, keys: &[String], stale: bool) {
            self.core.refresh_plm_index(keys, stale)
        }
        fn remember_saved_document(&self, name: &str, contents: &str) {
            self.core.remember_saved_document(name, contents)
        }
        fn list(&self) -> Vec<String> {
            self.core.list()
        }
        fn read(&self, name: &str) -> Option<String> {
            self.core.read(name)
        }
        fn residency(&self, name: &str) -> Residency {
            self.core.residency(name)
        }
        fn canonical_identity(&self, name: &str) -> String {
            self.core.canonical_identity(name)
        }
        fn write(&self, name: &str, contents: &str) -> Result<(), String> {
            self.core.write(name, contents)
        }
        fn remove(&self, name: &str) -> Result<(), String> {
            self.core.remove(name)
        }
        fn mutation_generation(&self) -> u64 {
            self.core.mutation_generation()
        }
        // The native queue these failures come from is run by the frame loop
        // itself (`BrepApp::ui`'s first line), not here: a store call behind an
        // early return would stall it.
        fn take_persistence_errors(&self) -> Vec<String> {
            self.core.take_persistence_errors()
        }
        fn plm_session(&self) -> Option<crate::plm::connection::Session> {
            use crate::plm::connection::{parse_label, Keep, Session, Status};
            let (url, username) = parse_label(&self.core.backend_label())?;
            Some(Session { status: Status::Connected { url, username }, keep: Keep::Files(self.files.root().to_path_buf()) })
        }
        fn pending_writes(&self) -> usize {
            self.core.pending_writes()
        }
        fn browser_location(&self) -> String {
            self.core.browser_location()
        }
        fn browser_entries(&self, extensions: &[&str]) -> Vec<BrowserEntry> {
            self.core.browser_entries(extensions)
        }
        fn browser_enter(&self, identity: &str) -> Result<(), String> {
            self.core.browser_enter(identity)
        }
        fn browser_up(&self) -> Result<(), String> {
            self.core.browser_up()
        }
        fn browser_home(&self) -> Result<(), String> {
            self.core.browser_home()
        }
        fn browser_root(&self) -> Result<(), String> {
            self.core.browser_root()
        }
        fn browser_navigate(&self, location: &str) -> Result<(), String> {
            self.core.browser_navigate(location)
        }
        fn browser_places(&self) -> Vec<BrowserPlace> {
            self.core.browser_places()
        }
        fn browser_create_dir(&self, name: &str) -> Result<(), String> {
            self.core.browser_create_dir(name)
        }
        fn browser_write(&self, name: &str, contents: &str) -> Result<String, String> {
            self.core.browser_write(name, contents)
        }

        // --- this machine's real files ----------------------------------------
        fn local_files(&self) -> Option<&dyn ModelStore> {
            Some(&self.files)
        }
        fn list_external_files(&self, extensions: &[&str]) -> Vec<String> {
            self.files.list_external_files(extensions)
        }
        fn read_external_file(&self, name: &str) -> Option<Vec<u8>> {
            self.files.read_external_file(name)
        }
        fn export_file_named(&self, file_name: &str, contents: &str) -> Result<(), String> {
            self.files.export_file_named(file_name, contents)
        }
        fn export_file_named_bytes(&self, file_name: &str, contents: &[u8]) -> Result<(), String> {
            self.files.export_file_named_bytes(file_name, contents)
        }
    }

}






// --- Document identity -----------------------------------------------------------
//
// A document's IDENTITY is the raw string the app keys it by: the name handed
// to `read` / `write`, kept on the open tab, and copied into everything that
// has to find the document again. On the file backends it is a path (native),
// a `/models/...` path (browser) or a bare name (the models folder). On a PLM
// store it is `part/<part id>/rev/<revision id>` — always a concrete revision,
// since the CAD app has no floating occurrences (D3).
//
// Every place the app keys a document by identity (plm-cad-integration-todo
// S0, enumerated 2026-09-25 by grep, not recalled):
//
// MINTED (a new identity comes into being):
//  1. `BrowserEntry.identity` — `browser_entries` rows; `browser_write`'s
//     return value (Save As, the STEP-to-part lane `panels/step_parts.rs`).
//  2. `Document::name` (`document.rs`) — the open tab's identity, set by Open
//     / Save / Save As (`panels/file.rs` `save_to`, `save_as`, the open path)
//     and by recovery. The ROOT: items 3 and 5 are copies of it.
//  3. The recovery blob's per-entry `name` (`recovery.rs` `RecoveryEntry`),
//     copied from `Document::name`, read back by `already_saved` and restored
//     onto the tab.
//  4. `sibling_identity` — a document beside another: a family's members
//     (`family_table.rs` `member_identity`, `generate_family`'s
//     `file_name_of`) and a spun-out part (`panels/file.rs`
//     `spin_out_target`). There are no folders on a PLM store: S7 / S9
//     replace these with a new part.
//  5. The ACOMP occurrence chain's `sourceKey` — `partsLibrary.<part>.sourceKey`
//     in an assembly document, written at insert (`panels/file.rs`
//     `insert_component_document`, `panels/step_parts.rs`) from the part's
//     identity; RESOLVED by `store.read(sourceKey)` in
//     `panels/update_components.rs`, `panels/parts_library.rs`
//     (`write_through`), `panels/component_actions.rs` (`part_source_key`,
//     Edit Part), `panels/bom.rs`, `panels/ecad_parts.rs` (open requests) and
//     `app.rs`. S13 rewrites file-path sourceKeys to revision keys.
//
// DERIVED (read off an identity's shape):
//  6. `model_display_name` (tab title, name field, duplicate-open check in
//     `panels/file.rs` `name_is_free`), `file_name_of`, and
//     `DocumentClass::of_name` (a document's class from its EXTENSION) — a
//     revision key has neither a file name nor an extension; on a PLM store
//     the class comes from the index (P3 / S7).
//
// NOT identities, though the plan's first draft listed them:
//  * `@pinned` holds explorer FOLDER locations, never documents — in PLM mode
//    it pins workspace folders (S14) and stays a Preferences key.
// `@recent_documents` holds document identities in most-recent-first order;
// it is a user preference and opens through the same file-dialog door.
//
// The seam: `ModelStore::identity`, `ModelStore::display_name` and
// `ModelStore::sibling_of`, whose defaults are exactly the path rules below,
// so a file backend is unchanged and a PLM backend supplies its own.

/// A document identity read for its shape — see the enumeration above.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DocumentIdentity {
    /// A native path, a browser `/models/...` path, or a bare name.
    Path(String),
    /// A PLM document key, `part/<part id>/rev/<revision id>`.
    Revision { part: String, revision: String },
}

impl DocumentIdentity {
    /// Read `key` as a PLM revision key — `part/<id>/rev/<id>`, both halves
    /// non-empty and slash-free, the same split the server's store routes make.
    ///
    /// Only a PLM-backed store may call this on its identities: the SHAPE is
    /// not proof, since a browser document at `models/part/7/rev/3.nbrep`
    /// lists as `part/7/rev/3`. That is why the file backends never parse.
    pub fn parse_revision_key(key: &str) -> Option<Self> {
        match key.split('/').collect::<Vec<_>>().as_slice() {
            ["part", part, "rev", revision] if !part.is_empty() && !revision.is_empty() => {
                Some(DocumentIdentity::Revision {
                    part: crate::plm::identity::unsegment(part)?,
                    revision: crate::plm::identity::unsegment(revision)?,
                })
            }
            _ => None,
        }
    }

    /// The raw string this identity is keyed by.
    pub fn key(&self) -> String {
        match self {
            DocumentIdentity::Path(path) => path.clone(),
            DocumentIdentity::Revision { part, revision } => crate::plm::identity::document_key(part, revision),
        }
    }
}

/// The identity of the file `file_name` in the same folder as the document
/// `identity`: a native path, a browser `/models/...` path, or — for a
/// document known only by a bare name — the bare `file_name`, which both
/// stores resolve into their models folder.
pub fn sibling_identity(identity: &str, file_name: &str) -> String {
    match identity.rfind(['/', '\\']) {
        Some(cut) => format!("{}{file_name}", &identity[..=cut]),
        None => file_name.to_string(),
    }
}

/// The last path component of a document identity (`/a/b/bolt.fbrep` ->
/// `bolt.fbrep`), extension kept.
pub fn file_name_of(identity: &str) -> String {
    identity.rsplit(['/', '\\']).next().unwrap_or(identity).to_string()
}

/// Strip the model extension (and any directory) from a filename to get the
/// bare display name — shared by both platform impls (and the file panel, which
/// shows the bare name while keeping the raw identity for re-saves).
pub(crate) fn model_display_name(file_name: &str) -> String {
    let base = file_name
        .rsplit(['/', '\\'])
        .next()
        .unwrap_or(file_name);
    // Only the NORMAL extension is implied: a family or template keeps its
    // extension in its name, which is what carries its class.
    match crate::document_class::DocumentClass::of_name(base) {
        Some(crate::document_class::DocumentClass::Normal) => {
            crate::document_class::strip_class_extension(base).to_string()
        }
        _ => base.to_string(),
    }
}

// --- Desktop: application models/files directory -------------------------------
#[cfg(not(target_arch = "wasm32"))]
mod native_model {
    use super::{
        model_display_name, BrowserEntry, BrowserPlace, ModelStore, PlaceKind, DOCK_LAYOUT_KEY,
        FEATURE_PALETTE_DISPLAY_KEY, KICAD_LIBRARY_KEY, PINNED_KEY, PLM_IMPORT_KEY, RECOVERY_KEY,
        SETTINGS_KEY,
    };
    use std::cell::{Cell, RefCell};
    use std::path::{Path, PathBuf};

    /// Persist models as `<config>/brep-app/models/<name>.nbrep` and reserved
    /// application state as files under `<config>/brep-app`. Enumerable (for the
    /// Open list) and unit-testable without a display.
    ///
    /// Open, Save As, import, and export are all driven by the application's
    /// common egui explorer; no OS-native dialog is involved.
    /// The PLM a config root names (D5's files only: what this machine keeps),
    /// as "not connected, reason unknown". A file that cannot be read is
    /// reported the same way, with the reason.
    fn configured_plm(root: &std::path::Path) -> Option<crate::plm::connection::Session> {
        use crate::plm::connection::{Keep, Session, Status};
        let status = match crate::plm::config::resolve(root, &|_| None, &Default::default()) {
            Ok(None) => return None,
            Ok(Some(config)) => Status::NotConnected { url: config.url, reason: None },
            Err(e) if root.join("plm.json").exists() => Status::NotConnected { url: String::new(), reason: Some(e) },
            Err(_) => return None,
        };
        Some(Session { status, keep: Keep::Files(root.to_path_buf()) })
    }

    pub struct FileModelStore {
        app_dir: PathBuf,
        dir: PathBuf,
        browser_dir: RefCell<PathBuf>,
        /// See [`ModelStore::mutation_generation`]. A plain `Cell` — this store
        /// is built once and held behind one `Box`, never cloned.
        mutations: Cell<u64>,
        /// Said once, on the first frame: why a session that asked for a PLM is
        /// on the file stores instead. Empty in every file-only session.
        notices: RefCell<Vec<String>>,
        /// The PLM configured in `app_dir` (read once, when the store is
        /// built) and why this session is not on it: the PLM tab's status.
        plm: RefCell<Option<crate::plm::connection::Session>>,
    }

    impl FileModelStore {
        pub fn new() -> Self {
            // The one config-directory rule, shared with the PLM's two files.
            let app_dir = crate::plm::config::app_config_dir();
            let plm = configured_plm(&app_dir);
            Self {
                dir: app_dir.join("models"),
                browser_dir: RefCell::new(app_dir.join("models")),
                app_dir,
                mutations: Cell::new(0),
                notices: RefCell::new(Vec::new()),
                plm: RefCell::new(plm),
            }
        }

        /// A store rooted at an explicit directory — used by the round-trip test.
        /// The tests stay headless and exercise the same egui explorer path.
        ///
        /// COLLAPSES the two directories [`Self::new`] keeps apart: models and
        /// reserved state land together. That is what a unit test wants and
        /// what a session must not have — see [`Self::rooted_at`].
        #[cfg_attr(not(test), allow(dead_code))]
        pub fn with_dir(dir: PathBuf) -> Self {
            Self {
                app_dir: dir.clone(),
                browser_dir: RefCell::new(dir.clone()),
                dir,
                mutations: Cell::new(0),
                notices: RefCell::new(Vec::new()),
                plm: RefCell::new(None),
            }
        }

        /// A store rooted at `root` with the SAME split [`Self::new`] makes
        /// under the user's config directory: models in `<root>/models`,
        /// reserved application state in `<root>`. The models directory is
        /// created now, so the explorer opens on a real location rather than on
        /// a path that does not exist yet.
        pub fn rooted_at(root: PathBuf) -> Self {
            let models = root.join("models");
            let _ = std::fs::create_dir_all(&models);
            let plm = configured_plm(&root);
            Self {
                app_dir: root,
                browser_dir: RefCell::new(models.clone()),
                dir: models,
                mutations: Cell::new(0),
                notices: RefCell::new(Vec::new()),
                plm: RefCell::new(plm),
            }
        }

        /// Queue `notice` for the first frame's notice drain (see
        /// [`ModelStore::take_persistence_errors`]).
        pub fn with_notice(self, notice: String) -> Self {
            self.notices.borrow_mut().push(notice);
            self
        }

        /// Keep why this session is not on its configured PLM, for the PLM
        /// tab (the notice above is drained on the first frame).
        pub fn with_plm_reason(self, reason: String) -> Self {
            if let Some(session) = self.plm.borrow_mut().as_mut() {
                if let crate::plm::connection::Status::NotConnected { url, reason: kept } = &mut session.status {
                    // The boot's sentence names the server first; the tab
                    // already does.
                    let own = format!("PLM at {url}: ");
                    *kept = Some(reason.strip_prefix(&own).map(str::to_string).unwrap_or(reason));
                }
            }
            self
        }

        /// The root D5's files live in.
        pub(crate) fn root(&self) -> &std::path::Path {
            &self.app_dir
        }

        /// Record that a write or a delete LANDED (see
        /// [`ModelStore::mutation_generation`]).
        fn bump(&self) {
            self.mutations.set(self.mutations.get().wrapping_add(1));
        }

        /// Resolve a name to a path. Explicit paths remain supported for existing
        /// callers; a bare name maps to `<dir>/<sanitized>.nbrep`.
        fn resolve(&self, name: &str) -> PathBuf {
            match name {
                SETTINGS_KEY => return self.app_dir.join("settings.json"),
                FEATURE_PALETTE_DISPLAY_KEY => return self.app_dir.join("feature_palette_display.json"),
                DOCK_LAYOUT_KEY => return self.app_dir.join("dock_layout.json"),
                PINNED_KEY => return self.app_dir.join("pinned.json"),
                RECOVERY_KEY => return self.app_dir.join("recovery.json"),
                KICAD_LIBRARY_KEY => return self.app_dir.join("kicad_library.json"),
                super::RECENT_DOCUMENTS_KEY => return self.app_dir.join("recent_documents.json"),
                PLM_IMPORT_KEY => return self.app_dir.join("plm_import.json"),
                super::PLUGINS_KEY => return self.app_dir.join("plugins.json"),
                super::JAVASCRIPT_DRAFT_KEY => return self.app_dir.join("javascript_draft.js"),
                _ => {}
            }
            if name.contains('/') || name.contains('\\') {
                return PathBuf::from(name);
            }
            let class = crate::document_class::DocumentClass::of_name(name).unwrap_or_default();
            let safe: String = crate::document_class::strip_class_extension(name)
                .chars()
                .map(|c| {
                    if c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.' {
                        c
                    } else {
                        '_'
                    }
                })
                .collect();
            self.dir.join(class.file_name(&safe))
        }

        fn home_dir() -> PathBuf {
            std::env::var_os("HOME")
                .map(PathBuf::from)
                .or_else(|| std::env::current_dir().ok())
                .unwrap_or_else(|| PathBuf::from("."))
        }

        fn matches_extension(path: &Path, extensions: &[&str]) -> bool {
            extensions.is_empty()
                || extensions.iter().any(|extension| {
                    let wanted = extension.trim_start_matches('.').to_ascii_lowercase();
                    let name = path
                        .file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .to_ascii_lowercase();
                    name.ends_with(&format!(".{wanted}"))
                })
        }
    }

    impl ModelStore for FileModelStore {
        fn backend_label(&self) -> String {
            format!("filesystem: {}", self.dir.display())
        }

        fn local_files(&self) -> Option<&dyn ModelStore> {
            Some(self)
        }

        // Writes are synchronous here, so nothing fails after `write` returned;
        // the only thing this lane ever carries is a boot notice.
        fn take_persistence_errors(&self) -> Vec<String> {
            std::mem::take(&mut *self.notices.borrow_mut())
        }

        fn plm_session(&self) -> Option<crate::plm::connection::Session> {
            self.plm.borrow().clone()
        }

        fn list(&self) -> Vec<String> {
            let mut names: Vec<String> = std::fs::read_dir(&self.dir)
                .into_iter()
                .flatten()
                .flatten()
                .filter_map(|entry| {
                    let name = entry.file_name().to_string_lossy().into_owned();
                    crate::document_class::DocumentClass::of_name(&name)
                        .is_some()
                        .then(|| model_display_name(&name))
                })
                .collect();
            names.sort();
            names
        }

        fn read(&self, name: &str) -> Option<String> {
            std::fs::read_to_string(self.resolve(name)).ok()
        }

        fn write(&self, name: &str, contents: &str) -> Result<(), String> {
            let path = self.resolve(name);
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent).map_err(|e| format!("create models dir: {e}"))?;
            }
            std::fs::write(&path, contents).map_err(|e| format!("write {}: {e}", path.display()))?;
            self.bump();
            Ok(())
        }

        fn mutation_generation(&self) -> u64 {
            self.mutations.get()
        }

        fn remove(&self, name: &str) -> Result<(), String> {
            match std::fs::remove_file(self.resolve(name)) {
                // A delete that found nothing changed nothing, so it does not
                // count: the generation tracks CONTENT, not calls.
                Ok(()) => {
                    self.bump();
                    Ok(())
                }
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
                Err(e) => Err(format!("remove: {e}")),
            }
        }

        fn canonical_identity(&self, name: &str) -> String {
            // `resolve` is where every read and write goes, so it IS the
            // document a name means; the explorer spells paths the same way.
            self.resolve(name).to_string_lossy().into_owned()
        }

        fn browser_location(&self) -> String {
            self.browser_dir.borrow().display().to_string()
        }

        fn browser_entries(&self, extensions: &[&str]) -> Vec<BrowserEntry> {
            let mut entries: Vec<BrowserEntry> = std::fs::read_dir(&*self.browser_dir.borrow())
                .into_iter()
                .flatten()
                .flatten()
                .filter_map(|entry| {
                    let path = entry.path();
                    let is_dir = path.is_dir();
                    if !(is_dir || (path.is_file() && Self::matches_extension(&path, extensions))) {
                        return None;
                    }
                    let meta = entry.metadata().ok();
                    let size = meta.as_ref().filter(|m| m.is_file()).map(|m| m.len());
                    let modified = meta
                        .as_ref()
                        .and_then(|m| m.modified().ok())
                        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
                        .map(|d| d.as_secs_f64());
                    Some(BrowserEntry {
                        name: entry.file_name().to_string_lossy().into_owned(),
                        identity: path.to_string_lossy().into_owned(),
                        is_dir,
                        size,
                        modified,
                    })
                })
                .collect();
            entries.sort_by(|a, b| {
                b.is_dir
                    .cmp(&a.is_dir)
                    .then_with(|| a.name.to_ascii_lowercase().cmp(&b.name.to_ascii_lowercase()))
            });
            entries
        }

        fn browser_enter(&self, identity: &str) -> Result<(), String> {
            let path = PathBuf::from(identity);
            if !path.is_dir() {
                return Err(format!("not a directory: {}", path.display()));
            }
            *self.browser_dir.borrow_mut() = path;
            Ok(())
        }

        fn browser_up(&self) -> Result<(), String> {
            let parent = self.browser_dir.borrow().parent().map(Path::to_path_buf);
            if let Some(parent) = parent {
                *self.browser_dir.borrow_mut() = parent;
            }
            Ok(())
        }

        fn browser_home(&self) -> Result<(), String> {
            *self.browser_dir.borrow_mut() = Self::home_dir();
            Ok(())
        }

        fn browser_root(&self) -> Result<(), String> {
            let current = self.browser_dir.borrow().clone();
            let root = current
                .ancestors()
                .last()
                .map(Path::to_path_buf)
                .unwrap_or_else(|| PathBuf::from(std::path::MAIN_SEPARATOR.to_string()));
            *self.browser_dir.borrow_mut() = root;
            Ok(())
        }

        fn browser_navigate(&self, location: &str) -> Result<(), String> {
            let path = PathBuf::from(location);
            if path.is_dir() {
                *self.browser_dir.borrow_mut() = path;
                Ok(())
            } else {
                Err(format!("not a directory: {location}"))
            }
        }

        fn browser_places(&self) -> Vec<BrowserPlace> {
            let home = Self::home_dir();
            let mut places = vec![BrowserPlace {
                label: "Home".into(),
                location: home.display().to_string(),
                kind: PlaceKind::Home,
            }];
            for (label, sub, kind) in [
                ("Documents", "Documents", PlaceKind::Documents),
                ("Downloads", "Downloads", PlaceKind::Downloads),
            ] {
                let path = home.join(sub);
                if path.is_dir() {
                    places.push(BrowserPlace {
                        label: label.into(),
                        location: path.display().to_string(),
                        kind,
                    });
                }
            }
            places.push(BrowserPlace {
                label: "Models".into(),
                location: self.dir.display().to_string(),
                kind: PlaceKind::Models,
            });
            places.push(BrowserPlace {
                label: "/".into(),
                location: "/".into(),
                kind: PlaceKind::Root,
            });
            places
        }

        fn browser_create_dir(&self, name: &str) -> Result<(), String> {
            let name = PathBuf::from(name);
            if name.components().count() != 1 {
                return Err("folder name must be one path component".into());
            }
            let path = self.browser_dir.borrow().join(name);
            std::fs::create_dir(&path)
                .map_err(|e| format!("create folder {}: {e}", path.display()))
        }

        fn browser_write(&self, name: &str, contents: &str) -> Result<String, String> {
            let file_name = PathBuf::from(name)
                .file_name()
                .ok_or("invalid file name")?
                .to_string_lossy()
                .into_owned();
            let file_name = super::model_file_name(&file_name);
            let path = self.browser_dir.borrow().join(file_name);
            std::fs::write(&path, contents)
                .map_err(|e| format!("write {}: {e}", path.display()))?;
            // This door writes the file ITSELF rather than going through
            // `write`, so it has to move the generation itself.
            self.bump();
            Ok(path.to_string_lossy().into_owned())
        }

        fn list_external_files(&self, extensions: &[&str]) -> Vec<String> {
            let wanted: Vec<String> = extensions
                .iter()
                .map(|extension| extension.trim_start_matches('.').to_ascii_lowercase())
                .collect();
            let mut names: Vec<String> = std::fs::read_dir(&self.dir)
                .into_iter()
                .flatten()
                .flatten()
                .filter_map(|entry| {
                    let path = entry.path();
                    path.is_file().then_some(path)
                })
                .filter_map(|path| {
                    let extension = path.extension()?.to_string_lossy().to_ascii_lowercase();
                    wanted.contains(&extension).then(|| {
                        path.file_name()
                            .unwrap_or_default()
                            .to_string_lossy()
                            .into_owned()
                    })
                })
                .collect();
            names.sort();
            names
        }

        fn read_external_file(&self, name: &str) -> Option<Vec<u8>> {
            let path = PathBuf::from(name);
            let path = if path.is_absolute() || path.components().count() > 1 {
                path
            } else {
                self.browser_dir.borrow().join(path)
            };
            std::fs::read(path).ok()
        }

        fn export_file_named(&self, file_name: &str, contents: &str) -> Result<(), String> {
            self.export_file_named_bytes(file_name, contents.as_bytes())
        }

        fn export_file_named_bytes(&self, file_name: &str, contents: &[u8]) -> Result<(), String> {
            std::fs::create_dir_all(&self.dir)
                .map_err(|e| format!("create models dir: {e}"))?;
            let name = PathBuf::from(file_name)
                .file_name()
                .ok_or("invalid export file name")?
                .to_owned();
            let path = self.dir.join(name);
            std::fs::write(&path, contents)
                .map_err(|e| format!("write {}: {e}", path.display()))
        }
    }
}

// --- The mirrored store: a synchronous facade over an ASYNC backend ------------
//
// The browser has no synchronous storage big enough for this application. The
// measured numbers that forced this module into existence: one imported STEP
// part serialises to a ~35 MB native BREP payload, and an imported assembly
// document runs 1.0x-12.8x its source STEP text — against `localStorage`'s
// ~5-10 MB per-origin quota. Every backend with room (IndexedDB today, a remote
// server tomorrow) is ASYNC, while [`ModelStore`] is synchronous and the egui
// frame loop that calls it is immediate-mode.
//
// The reconciliation is this module: an in-memory `BTreeMap` MIRROR of the whole
// key space, hydrated from the backend inside the already-async wasm entry
// point (`lib.rs::start`) BEFORE the app is constructed. Because hydration
// completes before the first `read` can happen, a synchronous read never has a
// "not loaded yet" state to represent, and none of the ~55 call sites change.
// Writes mutate the mirror synchronously (so the very next `read` sees them) and
// are pushed to the backend WRITE-BEHIND; a push that fails afterwards surfaces
// through [`ModelStore::take_persistence_errors`].
//
// Hydration is the only WHOLE-key-space read, not the only read. It cannot be:
// the backend is shared with every other tab of the origin, and a mirror that
// only ever loaded at boot goes stale the moment someone else saves — which is
// what made a part saved in one tab un-insertable in another until the user hit
// reload. [`MirrorStore::refresh_key`] re-reads ONE key through
// [`StoreBackend::get`] and folds it in, so the correction costs a single record
// rather than the corpus; what drives it is platform business, and lives in
// [`web_model`](super::web_model).
pub(crate) mod mirror_store {
    use super::{
        model_display_name, BrowserEntry, BrowserPlace, KeySpace, ModelStore, PlaceKind,
        Residency, DOCK_LAYOUT_KEY, FEATURE_PALETTE_DISPLAY_KEY, KICAD_LIBRARY_KEY, PINNED_KEY,
        PLM_IMPORT_KEY, RECOVERY_KEY, SETTINGS_KEY,
    };
    use std::cell::{Cell, RefCell};
    use std::collections::{BTreeMap, BTreeSet, VecDeque};
    use std::future::Future;
    use std::pin::Pin;
    use std::rc::Rc;

    /// The persisted key scheme, VERBATIM from the `localStorage` era so an
    /// existing origin's keys keep their meaning under any backend: model
    /// documents live at `brep-app:model:<relative name>`, explorer folders at
    /// `brep-app:dir:<relative path>/`, and the three reserved application blobs
    /// at `brep-app:settings` / `:dock_layout` / `:pinned` (see [`MirrorStore::key`]).
    pub(crate) const PREFIX: &str = "brep-app:model:";
    pub(crate) const DIR_PREFIX: &str = "brep-app:dir:";

    /// How many distinct persistence failures the error log keeps before the
    /// oldest is dropped. The log is a user-facing notice channel, not telemetry.
    const MAX_ERRORS: usize = 64;

    /// The future a [`StoreBackend`] hands back. Boxed (not `async fn` in the
    /// trait) because the store is used as `dyn StoreBackend`, and deliberately
    /// NOT `Send`: every host is single-threaded (the browser main thread; the
    /// native test executor below).
    pub(crate) type BackendFuture<T> = Pin<Box<dyn Future<Output = Result<T, String>>>>;

    /// One hydrated key's value in the mirror: the bytes themselves, or only
    /// the fact that they exist and how many there are.
    ///
    /// `OnDisk` is what makes an index-only hydrate possible: `list`,
    /// `browser_entries` and sizes answer from it with no bytes in memory, and
    /// the bytes arrive later — on the first `read`, or from
    /// [`MirrorStore::prefetch`] in the background.
    #[derive(Clone, Debug, PartialEq, Eq)]
    // `OnDisk` is built only by an index-only backend: the tests' today, the
    // PLM's in S1.
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) enum Entry {
        Resident(String),
        OnDisk { bytes: u64 },
    }

    /// One row of a PLM index answer: the mirror key, its size and the hash
    /// of the document the server holds (sha256, hex — what
    /// [`crate::plm::kicad::content_hash`] computes of the same bytes).
    pub(crate) struct IndexRow {
        pub(crate) key: String,
        pub(crate) bytes: u64,
        pub(crate) content_hash: String,
    }

    impl Entry {
        /// The value's size in bytes, whether or not it is loaded.
        pub(crate) fn len(&self) -> u64 {
            match self {
                Entry::Resident(value) => value.len() as u64,
                Entry::OnDisk { bytes } => *bytes,
            }
        }
    }

    /// The key space a BACKEND key belongs to (see [`KeySpace`]), or `None` for
    /// an explorer directory row (`brep-app:dir:`), which only the file backends
    /// hold — a PLM user's folders are the workspace (S14), not store keys.
    /// A key this module did not mint counts as a document.
    #[cfg_attr(not(test), allow(dead_code))] // routed by S1's PLM session
    pub(crate) fn key_space(key: &str) -> Option<KeySpace> {
        if key.starts_with(DIR_PREFIX) {
            return None;
        }
        for name in super::PREFERENCE_KEYS.iter().chain(super::LOCAL_KEYS) {
            if key == MirrorStore::key(name) {
                return Some(KeySpace::of(name));
            }
        }
        Some(KeySpace::Documents)
    }

    /// Where a [`MirrorStore`]'s bytes actually live — the swappable persistence
    /// seam.
    ///
    /// The contract is deliberately narrow and platform-free: **string keys,
    /// string values, `String` errors, async everywhere**. No `JsValue`, no
    /// `web_sys` type, no IndexedDB concept crosses it, and the mirror above it
    /// knows nothing about how a key is stored. [`IdbBackend`](super::web_model)
    /// is the only implementation today; the intended SECOND one is a remote
    /// HTTP/AJAX backend that pushes and pulls the same key space to a central
    /// server, and it must be able to land without touching this module.
    ///
    /// The obligations an implementation carries:
    ///
    /// * **Per-key FIFO — the mirror's, unless the backend says it is its own.**
    ///   Two `put`s of the same key must land in call order, whatever order
    ///   their futures are polled in. IndexedDB gets this for free (a
    ///   `readwrite` transaction commits in creation order, and
    ///   [`IdbBackend`](super::web_model) creates the transaction and issues the
    ///   request synchronously inside `put`), and says so through
    ///   [`Self::orders_writes_per_key`]. Every other backend — an HTTP one has
    ///   no such guarantee — gets the mirror's per-key send queue: a key's next
    ///   `put`/`delete` is not even ISSUED until its previous one has settled.
    /// * **`load_index` or `load_all` hydrates the whole key space** — every key
    ///   is present in the answer; `load_index` may leave values
    ///   [`Entry::OnDisk`].
    /// * **`get` sees other clients' writes.** The mirror calls it to correct
    ///   itself after someone else wrote the key, and to load an `OnDisk`
    ///   entry, so it must read THROUGH any per-connection cache of the
    ///   implementation's own.
    ///
    /// ## Index hydration
    ///
    /// A full hydrate holds EVERY saved document resident for the session: a
    /// non-issue for a browser origin (IndexedDB is local, and still hydrates
    /// whole through the `load_index` default), untenable for a remote catalog
    /// of thousands of parts. So the mirror's value is an [`Entry`], a backend
    /// may answer [`Self::load_index`] with sizes instead of bytes, and the
    /// bytes arrive on demand ([`ModelStore::read`] asks for them) or in the
    /// background ([`MirrorStore::prefetch`]). All of it sits behind THIS trait
    /// and behind [`ModelStore`], so no call site moved.
    pub(crate) trait StoreBackend {
        /// A short human label for the storage panel header (see
        /// [`ModelStore::backend_label`]).
        fn label(&self) -> String;

        /// The signed-in client of a backend that talks to a PLM (see
        /// [`ModelStore::plm_client`]). `None` for every local backend.
        fn plm_client(&self) -> Option<std::rc::Rc<crate::plm::client::PlmClient>> {
            None
        }

        /// See [`ModelStore::plm_revision`]. `None` for every local backend.
        fn plm_revision(&self, key: &str) -> Option<crate::plm::client::IndexEntry> {
            let _ = key;
            None
        }

        /// See [`ModelStore::refresh_plm_index`].
        fn refresh_plm_index(&self, keys: &[String], stale: bool) {
            let _ = (keys, stale);
        }

        /// The index rows of `keys` (store keys, `part/<p>/rev/<r>`), as the
        /// LAZY entries the mirror reconciles on a feed move: `(mirror key,
        /// [`Entry::OnDisk`], content hash)`. A key the server no longer
        /// lists is absent from the answer. `Err` from a backend that keeps no
        /// index: the mirror then leaves its entries alone.
        fn index_rows(&self, keys: &[String]) -> BackendFuture<Vec<IndexRow>> {
            let _ = keys;
            Box::pin(async { Err("this store keeps no index".to_string()) })
        }

        /// The whole index as [`Self::index_rows`] answers it, for a stale
        /// feed (a resync): every document the server lists.
        fn index_all(&self) -> BackendFuture<Vec<IndexRow>> {
            Box::pin(async { Err("this store keeps no index".to_string()) })
        }

        /// Pull the entire key space, values included.
        fn load_all(&self) -> BackendFuture<Vec<(String, String)>>;

        /// Pull the entire key space as METADATA: every key, with its bytes
        /// where they are cheap and only its size ([`Entry::OnDisk`]) where they
        /// are not. Called ONCE, during the async boot, before any
        /// [`MirrorStore`] exists.
        ///
        /// The default is [`Self::load_all`] with every value resident — exactly
        /// the hydrate a local backend has always done. A remote backend
        /// overrides it (the PLM's `GET /api/store/index` IS this call).
        fn load_index(&self) -> BackendFuture<Vec<(String, Entry)>> {
            let all = self.load_all();
            Box::pin(async move {
                Ok(all
                    .await?
                    .into_iter()
                    .map(|(key, value)| (key, Entry::Resident(value)))
                    .collect())
            })
        }

        /// `true` when this backend ALREADY lands two writes of one key in call
        /// order without help (IndexedDB), so the mirror issues every write
        /// the moment it is made. The default, `false`, puts the backend behind
        /// the mirror's per-key send queue — the safe answer for a backend that
        /// has not thought about it.
        fn orders_writes_per_key(&self) -> bool {
            false
        }

        /// Read ONE key's CURRENT value straight from the backend — `None` when
        /// it is absent.
        ///
        /// This is the LIVE read, and it exists because the mirror is not the
        /// only writer of its key space: a SECOND TAB of the same origin holds
        /// its own mirror over the same IndexedDB (and a remote backend would
        /// have other clients still). Hydration alone therefore goes stale the
        /// moment someone else saves — the user's "I saved a part in one tab and
        /// the other tab cannot insert it until I reload". [`MirrorStore::refresh_key`]
        /// answers that by re-reading exactly the key that changed. It is also
        /// how an [`Entry::OnDisk`] value is loaded.
        ///
        /// REQUIRED rather than defaulted, for the reason
        /// [`ModelStore::mutation_generation`] is: a `None`-returning default
        /// would let a new backend compile while silently making every external
        /// change look like a deletion.
        fn get(&self, key: &str) -> BackendFuture<Option<String>>;

        /// Persist `value` under `key`, creating or overwriting.
        fn put(&self, key: &str, value: &str) -> BackendFuture<()>;

        /// Delete `key`. Succeeding on an absent key is correct.
        fn delete(&self, key: &str) -> BackendFuture<()>;
    }

    /// A [`StoreBackend`] that sends each [`KeySpace`] to its own backend — the
    /// shape D1 gives a PLM session: documents to the store routes, preferences
    /// to the per-user store (P5), `@recovery` to the machine. Explorer
    /// directory rows travel with the documents, since only a file backend
    /// holds them; a documents backend with no folders refuses them itself.
    ///
    /// Hydration is the union of the three indexes. A key one backend returns
    /// that belongs to ANOTHER space is dropped rather than trusted: a backend
    /// is authoritative for its own space only.
    #[cfg_attr(not(test), allow(dead_code))] // composed by S1's PLM session
    pub(crate) struct KeySpaceRouter {
        pub(crate) documents: Rc<dyn StoreBackend>,
        pub(crate) preferences: Rc<dyn StoreBackend>,
        pub(crate) local: Rc<dyn StoreBackend>,
    }

    #[cfg_attr(not(test), allow(dead_code))]
    impl KeySpaceRouter {
        fn route(&self, key: &str) -> &Rc<dyn StoreBackend> {
            match key_space(key) {
                Some(KeySpace::Preferences) => &self.preferences,
                Some(KeySpace::Local) => &self.local,
                Some(KeySpace::Documents) | None => &self.documents,
            }
        }

        fn serves(space: KeySpace, key: &str) -> bool {
            match key_space(key) {
                Some(found) => found == space,
                None => space == KeySpace::Documents,
            }
        }

        fn spaces(&self) -> [(KeySpace, &Rc<dyn StoreBackend>); 3] {
            [
                (KeySpace::Documents, &self.documents),
                (KeySpace::Preferences, &self.preferences),
                (KeySpace::Local, &self.local),
            ]
        }
    }

    impl StoreBackend for KeySpaceRouter {
        fn label(&self) -> String {
            self.documents.label()
        }

        fn plm_client(&self) -> Option<std::rc::Rc<crate::plm::client::PlmClient>> {
            self.documents.plm_client()
        }

        fn plm_revision(&self, key: &str) -> Option<crate::plm::client::IndexEntry> {
            self.documents.plm_revision(key)
        }

        fn refresh_plm_index(&self, keys: &[String], stale: bool) {
            self.documents.refresh_plm_index(keys, stale)
        }
        fn index_rows(&self, keys: &[String]) -> BackendFuture<Vec<IndexRow>> {
            self.documents.index_rows(keys)
        }
        fn index_all(&self) -> BackendFuture<Vec<IndexRow>> {
            self.documents.index_all()
        }

        fn load_all(&self) -> BackendFuture<Vec<(String, String)>> {
            let requests: Vec<_> = self
                .spaces()
                .into_iter()
                .map(|(space, backend)| (space, backend.load_all()))
                .collect();
            Box::pin(async move {
                let mut all = Vec::new();
                for (space, request) in requests {
                    all.extend(
                        request
                            .await?
                            .into_iter()
                            .filter(|(key, _)| Self::serves(space, key)),
                    );
                }
                Ok(all)
            })
        }

        fn load_index(&self) -> BackendFuture<Vec<(String, Entry)>> {
            let requests: Vec<_> = self
                .spaces()
                .into_iter()
                .map(|(space, backend)| (space, backend.load_index()))
                .collect();
            Box::pin(async move {
                let mut all = Vec::new();
                for (space, request) in requests {
                    all.extend(
                        request
                            .await?
                            .into_iter()
                            .filter(|(key, _)| Self::serves(space, key)),
                    );
                }
                Ok(all)
            })
        }

        fn orders_writes_per_key(&self) -> bool {
            self.spaces()
                .iter()
                .all(|(_, backend)| backend.orders_writes_per_key())
        }

        fn get(&self, key: &str) -> BackendFuture<Option<String>> {
            self.route(key).get(key)
        }

        fn put(&self, key: &str, value: &str) -> BackendFuture<()> {
            self.route(key).put(key, value)
        }

        fn delete(&self, key: &str) -> BackendFuture<()> {
            self.route(key).delete(key)
        }
    }

    /// The backend installed when persistence could NOT be brought up (IndexedDB
    /// blocked in a private window, storage denied, the boot hydrate never ran).
    ///
    /// There is deliberately no quiet fallback to `localStorage`: a 5 MB backend
    /// silently standing in for a hundreds-of-megabytes one is the failure mode
    /// this whole change exists to end. Instead the session stays fully usable
    /// IN MEMORY — the mirror still holds everything written this session — while
    /// every write reports, loudly and repeatedly, that nothing is being saved.
    // Built by the browser's boot today; a native session falls back to the file stores instead.
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub(crate) struct UnavailableBackend {
        reason: String,
    }

    // See `UnavailableBackend`.
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    impl UnavailableBackend {
        pub(crate) fn new(reason: impl Into<String>) -> Self {
            Self {
                reason: reason.into(),
            }
        }
    }

    impl StoreBackend for UnavailableBackend {
        fn label(&self) -> String {
            format!("NOT SAVING — {} · use Download to keep your work", self.reason)
        }

        fn load_all(&self) -> BackendFuture<Vec<(String, String)>> {
            let reason = self.reason.clone();
            Box::pin(async move { Err(reason) })
        }

        fn get(&self, _key: &str) -> BackendFuture<Option<String>> {
            let reason = self.reason.clone();
            Box::pin(async move { Err(reason) })
        }

        fn put(&self, _key: &str, _value: &str) -> BackendFuture<()> {
            let reason = self.reason.clone();
            Box::pin(async move { Err(reason) })
        }

        fn delete(&self, _key: &str) -> BackendFuture<()> {
            let reason = self.reason.clone();
            Box::pin(async move { Err(reason) })
        }
    }

    /// Persistence failures that happened AFTER the synchronous `write` returned
    /// `Ok` — the price of write-behind. Nothing is lost mid-session (the mirror
    /// holds it), but the user MUST learn that it did not persist, so the log is
    /// drained into the toast overlay every frame.
    ///
    /// The cursor (rather than a `Vec::drain`) keeps the full history addressable
    /// for the `__brepStoreErrors` verification hook while still handing the UI
    /// each message exactly once.
    pub(crate) struct ErrorLog {
        log: RefCell<Vec<String>>,
        drained: Cell<usize>,
        /// Wakes the reactive frame loop so a failure recorded from an async
        /// callback is toasted THIS frame instead of waiting for stray input.
        wake: Option<Rc<dyn Fn()>>,
    }

    impl ErrorLog {
        fn new(wake: Option<Rc<dyn Fn()>>) -> Self {
            Self {
                log: RefCell::new(Vec::new()),
                drained: Cell::new(0),
                wake,
            }
        }

        /// Record one failure. An identical message that is still UNDRAINED is
        /// collapsed (a burst of failing writes shows one toast, not six); once
        /// the UI has shown it, the same message can be recorded again — every
        /// failed save is reported.
        pub(crate) fn record(&self, message: String) {
            {
                let mut log = self.log.borrow_mut();
                let undrained = log.len() > self.drained.get();
                if undrained && log.last().map(|last| *last == message).unwrap_or(false) {
                    return;
                }
                log.push(message);
                if log.len() > MAX_ERRORS {
                    log.remove(0);
                    self.drained.set(self.drained.get().saturating_sub(1));
                }
            }
            if let Some(wake) = &self.wake {
                wake();
            }
        }

        /// Messages recorded since the last drain (what the UI has not shown yet).
        fn drain(&self) -> Vec<String> {
            let log = self.log.borrow();
            let from = self.drained.get().min(log.len());
            self.drained.set(log.len());
            log[from..].to_vec()
        }

        /// Every message recorded this session (verification hook).
        // The browser's `__brepStoreErrors` verification hook.
        #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
        fn all(&self) -> Vec<String> {
            self.log.borrow().clone()
        }
    }

    /// Drive one write-behind push to completion.
    ///
    /// wasm: hand it to the browser's microtask queue, which is the whole point —
    /// the caller's `write` already returned. Native: the frame-driven executor
    /// ([`crate::plm::native`]), the same shape — polled once at once (so a
    /// backend whose futures are already resolved finishes inside `write`, as
    /// every test that does not care about timing assumes), parked if still
    /// pending, and polled again once its waker fires and the frame loop (or a
    /// test's [`test_executor::drive`]) runs the queue. Keeping the shape
    /// identical means the native tests exercise the REAL write-behind path
    /// (spawn, await, record the error) rather than a synchronous stand-in — and
    /// the parking is what lets a test hold one write open and watch what the
    /// per-key queue does with the next.
    #[cfg(target_arch = "wasm32")]
    fn spawn(task: impl Future<Output = ()> + 'static) {
        wasm_bindgen_futures::spawn_local(task);
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn spawn(task: impl Future<Output = ()> + 'static) {
        crate::plm::native::spawn(task);
    }


    /// A synchronous [`ModelStore`] over an asynchronous [`StoreBackend`]: the
    /// hydrated mirror plus write-behind. Cheap to clone — every field is shared,
    /// so a clone is another handle on the SAME session state (used by the
    /// verification hooks).
    #[derive(Clone)]
    pub(crate) struct MirrorStore {
        /// The whole key space, keyed EXACTLY as the backend keys it. A value
        /// is [`Entry::OnDisk`] until something asks for its bytes.
        entries: Rc<RefCell<BTreeMap<String, Entry>>>,
        /// The explorer's current virtual directory (`/` or `/models/...`).
        browser_dir: Rc<RefCell<String>>,
        backend: Rc<dyn StoreBackend>,
        errors: Rc<ErrorLog>,
        /// Pushes issued but not yet settled. Zero means everything written so
        /// far is durable — which is what makes a "save, reload, still there"
        /// check non-racy (see the `__brepStorePending` hook).
        pending: Rc<Cell<usize>>,
        /// See [`ModelStore::mutation_generation`]. SHARED like every other
        /// field: a clone is another handle on the same session state, and the
        /// `__brepStoreWrite` verification hook holds one — a write through
        /// that hook has to move the generation the app reads.
        mutations: Rc<Cell<u64>>,
        /// How many of our OWN pushes are still in flight, PER KEY. Used by
        /// [`Self::refresh_key`] to leave a key alone while this session is
        /// mid-write on it: pulling another tab's value in on top of a save we
        /// have already returned `Ok` for would show the user someone else's
        /// bytes under their own file name.
        in_flight: Rc<RefCell<BTreeMap<String, usize>>>,
        /// Repaint the reactive frame loop. The `ErrorLog` holds this for write
        /// failures; the store needs its own handle because an EXTERNAL change
        /// arrives on an async callback too, and a list that updates only when
        /// the user happens to move the mouse is the stale list all over again.
        wake: Option<Rc<dyn Fn()>>,
        /// `OnDisk` keys whose bytes have been asked of the backend and not yet
        /// answered, so a second `read` of one does not ask again.
        loading: Rc<RefCell<BTreeSet<String>>>,
        /// The per-key send queue (see [`StoreBackend::orders_writes_per_key`]):
        /// for each key with a write in flight, the writes made after it, not
        /// yet issued. A key is present exactly while its drain task runs.
        queues: Rc<RefCell<BTreeMap<String, VecDeque<QueuedWrite>>>>,
        /// A monotonic clock of authority: every local mutation of a key and
        /// every index request issued takes the next tick.
        clock: Rc<Cell<u64>>,
        /// Per key, the tick of the authority that last set it here — a local
        /// write, delete, remembered save, loaded body, or the ISSUE tick of
        /// the index answer applied to it. An index answer applies to a key
        /// only if it was issued after that tick (see [`Self::reconcile`]).
        freshness: Rc<RefCell<BTreeMap<String, u64>>>,
        /// A whole-index answer also speaks for keys never seen locally.
        full_index_authority: Rc<Cell<u64>>,
    }

    /// A write the per-key queue holds back: the document name its failure is
    /// reported under, and the backend call, made only when its turn comes.
    type QueuedWrite = (String, Box<dyn FnOnce() -> BackendFuture<()>>);

    impl MirrorStore {
        /// Build the session store from an already-hydrated key space.
        /// `wake` (`None` off-browser) is called when a write-behind failure is
        /// recorded, to repaint the reactive frame loop.
        // The browser's boot and the tests; the native boot hydrates from an index.
        #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
        pub(crate) fn new(
            backend: Rc<dyn StoreBackend>,
            entries: Vec<(String, String)>,
            wake: Option<Rc<dyn Fn()>>,
        ) -> Self {
            Self::from_index(
                backend,
                entries
                    .into_iter()
                    .map(|(key, value)| (key, Entry::Resident(value)))
                    .collect(),
                wake,
            )
        }

        /// Build the session store from an INDEX ([`StoreBackend::load_index`]):
        /// every key present, some values only [`Entry::OnDisk`].
        pub(crate) fn from_index(
            backend: Rc<dyn StoreBackend>,
            entries: Vec<(String, Entry)>,
            wake: Option<Rc<dyn Fn()>>,
        ) -> Self {
            Self {
                entries: Rc::new(RefCell::new(entries.into_iter().collect())),
                browser_dir: Rc::new(RefCell::new("/models".into())),
                backend,
                errors: Rc::new(ErrorLog::new(wake.clone())),
                pending: Rc::new(Cell::new(0)),
                mutations: Rc::new(Cell::new(0)),
                in_flight: Rc::new(RefCell::new(BTreeMap::new())),
                wake,
                loading: Rc::new(RefCell::new(BTreeSet::new())),
                queues: Rc::new(RefCell::new(BTreeMap::new())),
                clock: Rc::new(Cell::new(0)),
                freshness: Rc::new(RefCell::new(BTreeMap::new())),
                full_index_authority: Rc::new(Cell::new(0)),
            }
        }
        /// The next tick of the authority clock.
        fn tick(&self) -> u64 {
            let next = self.clock.get() + 1;
            self.clock.set(next);
            next
        }
        /// A local authority over `key` now: later index answers that were
        /// issued before this cannot touch it.
        fn stamp_local(&self, key: &str) {
            let tick = self.tick();
            self.freshness.borrow_mut().insert(key.to_string(), tick);
        }

        /// The persisted key for a document/reserved name. UNCHANGED from the
        /// `localStorage` implementation this replaced, so an existing origin's
        /// data keeps its identity.
        pub(crate) fn key(name: &str) -> String {
            match name {
                SETTINGS_KEY => "brep-app:settings".into(),
                FEATURE_PALETTE_DISPLAY_KEY => "brep-app:feature_palette_display".into(),
                DOCK_LAYOUT_KEY => "brep-app:dock_layout".into(),
                PINNED_KEY => "brep-app:pinned".into(),
                RECOVERY_KEY => "brep-app:recovery".into(),
                // Desktop-only today (the web build cannot read a KiCad
                // library), but a reserved name that fell through to the
                // document arm would key as `brep-app:model:@kicad_library` and
                // LIST as a document the day anything wrote it here.
                KICAD_LIBRARY_KEY => "brep-app:kicad_library".into(),
                super::RECENT_DOCUMENTS_KEY => "brep-app:recent_documents".into(),
                PLM_IMPORT_KEY => "brep-app:plm_import".into(),
                super::PLUGINS_KEY => "brep-app:plugins".into(),
                super::JAVASCRIPT_DRAFT_KEY => "brep-app:javascript_draft".into(),
                _ => format!("{PREFIX}{}", Self::model_relative(name)),
            }
        }

        /// A document name reduced to its store-relative form: no leading `/`, no
        /// `/models` root, no model extension.
        fn model_relative(name: &str) -> String {
            let name = name
                .trim_start_matches('/')
                .strip_prefix("models/")
                .unwrap_or_else(|| name.trim_start_matches('/'));
            // The key drops ONLY the normal extension (an existing origin's
            // keys keep their identity); a family or template keeps its own,
            // lowercased, so `bolt` the part and `bolt.fbrep` the family are
            // two keys.
            let name = name.trim_matches('/');
            match crate::document_class::DocumentClass::of_name(name) {
                Some(crate::document_class::DocumentClass::Normal) => {
                    crate::document_class::strip_class_extension(name).to_string()
                }
                Some(_) => super::model_file_name(name),
                None => name.to_string(),
            }
        }

        fn virtual_model_path(relative: &str) -> String {
            format!("/models/{}", super::model_file_name(relative.trim_matches('/')))
        }

        fn child_path(parent: &str, child: &str) -> String {
            if parent == "/" {
                format!("/{child}")
            } else {
                format!("{}/{child}", parent.trim_end_matches('/'))
            }
        }

        /// Hand one backend write to the executor, counting it in `pending` (and
        /// under its KEY in `in_flight`) and routing its eventual failure to the
        /// error log.
        ///
        /// `request` is the backend call itself, not its future: behind the
        /// per-key send queue it is made only once every earlier write of the
        /// same key has SETTLED, which is the FIFO an HTTP backend cannot give
        /// itself. A backend that orders its own writes
        /// ([`StoreBackend::orders_writes_per_key`]) has it called at once, so
        /// IndexedDB creates its transaction inside `write` exactly as before.
        fn push(
            &self,
            name: &str,
            key: &str,
            request: impl FnOnce() -> BackendFuture<()> + 'static,
        ) {
            self.pending.set(self.pending.get() + 1);
            *self.in_flight.borrow_mut().entry(key.to_string()).or_insert(0) += 1;
            if self.backend.orders_writes_per_key() {
                let settle = self.settler(key);
                let name = name.to_string();
                let request = request();
                spawn(async move { settle(&name, request.await) });
                return;
            }
            {
                let mut queues = self.queues.borrow_mut();
                if let Some(queue) = queues.get_mut(key) {
                    // A write of this key is still open: this one waits its turn.
                    queue.push_back((name.to_string(), Box::new(request)));
                    return;
                }
                queues.insert(key.to_string(), VecDeque::new());
            }
            // Nothing ahead of it: issue now, then drain whatever queues behind.
            let settle = self.settler(key);
            let queues = self.queues.clone();
            let key = key.to_string();
            let first = (name.to_string(), request());
            spawn(async move {
                let (name, request) = first;
                settle(&name, request.await);
                loop {
                    let next = {
                        let mut queues = queues.borrow_mut();
                        let next = queues.get_mut(&key).and_then(VecDeque::pop_front);
                        if next.is_none() {
                            queues.remove(&key);
                        }
                        next
                    };
                    let Some((name, request)) = next else {
                        break;
                    };
                    // A failed write does not cancel the next: each is its own
                    // save, and each failure is reported under its own name.
                    settle(&name, request().await);
                }
            });
        }

        /// What one settled write does to the session's books: out of
        /// `pending` and `in_flight`, and into the error log if it failed.
        fn settler(&self, key: &str) -> impl Fn(&str, Result<(), String>) + 'static {
            let pending = self.pending.clone();
            let in_flight = self.in_flight.clone();
            let errors = self.errors.clone();
            let key = key.to_string();
            move |name, outcome| {
                pending.set(pending.get().saturating_sub(1));
                {
                    let mut in_flight = in_flight.borrow_mut();
                    if let Some(count) = in_flight.get_mut(&key) {
                        *count -= 1;
                        if *count == 0 {
                            in_flight.remove(&key);
                        }
                    }
                }
                if let Err(message) = outcome {
                    errors.record(format!("'{name}' was NOT saved: {message}"));
                }
            }
        }

        /// Fold ONE externally-changed key into the mirror: `Some` is the value
        /// the backend now holds, `None` means it is gone.
        ///
        /// A change that is not a change (the value we already had) stops here —
        /// no generation bump, no repaint — so an echo costs nothing. A real one
        /// moves [`ModelStore::mutation_generation`], which is exactly what that
        /// counter promises ("every mutation that landed, WHOEVER made it") and
        /// what relights the outdated-components badge on an assembly whose part
        /// was just re-saved in another tab.
        // The browser's other-tab channel today; natively, the PLM change feed (S1).
        /// Fold an index answer into the entries. `asked`: the mirror keys a
        /// feed move named (a key missing from the answer was deleted on the
        /// server); `None`: the answer is the WHOLE index (a stale-feed
        /// resync), so every model key it does not list was deleted.
        ///
        /// The ordering policy, per key: an answer applies only if it was
        /// ISSUED (`issued`, the clock tick taken before the request left)
        /// after the authority that last set the key here — a local write or
        /// delete, a remembered save, a loaded body, or a newer answer. So an
        /// answer that left before a save and lands after it cannot evict the
        /// saved copy or delete the saved key, even with nothing in flight
        /// any more; and two answers landing out of order leave the newer
        /// one's result standing. A key with a write in flight or queued is
        /// skipped as well. A resident entry whose bytes hash to the row's
        /// content hash is left alone (the server holds what this app holds).
        /// Everything else becomes the server's row, lazily: a resident copy
        /// the server has moved on from is re-read on its next `read` (no
        /// body is fetched here), and an open document's unsaved edits live in
        /// its tab, not here. Returns whether anything changed.
        pub(crate) fn reconcile(&self, asked: Option<&[String]>, rows: Vec<IndexRow>, issued: u64) -> bool {
            let busy = |key: &str| {
                self.in_flight.borrow().contains_key(key)
                    || self.queues.borrow().contains_key(key)
                    || self.full_index_authority.get() >= issued
                    || self.freshness.borrow().get(key).is_some_and(|&tick| tick >= issued)
            };
            let listed: BTreeSet<&str> = rows.iter().map(|row| row.key.as_str()).collect();
            let mut changed = false;
            {
                let mut entries = self.entries.borrow_mut();
                let gone: Vec<String> = match asked {
                    Some(asked) => asked.iter().filter(|key| !listed.contains(key.as_str())).cloned().collect(),
                    None => entries
                        .keys()
                        .filter(|key| key.starts_with(PREFIX) && !listed.contains(key.as_str()))
                        .cloned()
                        .collect(),
                };
                for key in gone {
                    if busy(&key) {
                        continue;
                    }
                    self.freshness.borrow_mut().insert(key.clone(), issued);
                    changed |= entries.remove(&key).is_some();
                }
                for row in rows {
                    if busy(&row.key) {
                        continue;
                    }
                    self.freshness.borrow_mut().insert(row.key.clone(), issued);
                    let same = match entries.get(&row.key) {
                        Some(Entry::Resident(held)) => {
                            crate::plm::kicad::content_hash(held.as_bytes()) == row.content_hash
                        }
                        Some(Entry::OnDisk { bytes }) => *bytes == row.bytes,
                        None => false,
                    };
                    if same {
                        continue;
                    }
                    entries.insert(row.key, Entry::OnDisk { bytes: row.bytes });
                    changed = true;
                }
            }
            if asked.is_none() {
                self.full_index_authority.set(self.full_index_authority.get().max(issued));
            }
            if changed {
                self.mutations.set(self.mutations.get().wrapping_add(1));
                if let Some(wake) = &self.wake {
                    wake();
                }
            }
            changed
        }

        #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
        pub(crate) fn apply_external(&self, key: &str, value: Option<String>) {
            self.stamp_local(key);
            let changed = {
                let mut entries = self.entries.borrow_mut();
                match value {
                    Some(value) => {
                        let same = matches!(entries.get(key), Some(Entry::Resident(held)) if *held == value);
                        if same {
                            false
                        } else {
                            entries.insert(key.to_string(), Entry::Resident(value));
                            true
                        }
                    }
                    None => entries.remove(key).is_some(),
                }
            };
            if !changed {
                return;
            }
            self.mutations.set(self.mutations.get().wrapping_add(1));
            if let Some(wake) = &self.wake {
                wake();
            }
        }

        /// Re-read `key` from the backend and fold the result in — the LIVE read
        /// behind the mirror, driven by whatever tells this session that someone
        /// else wrote (on the web, the `brep-app:store` broadcast in
        /// [`web_model`](super::web_model)).
        ///
        /// A key this session is itself mid-write on is left ALONE: our own
        /// `write` already returned `Ok` against the mirror, and racing another
        /// tab's older bytes in on top of it would replace the user's save with a
        /// stranger's under the same name. Skipping costs no correctness. Which
        /// of two simultaneous saves of one file wins is settled by the backend's
        /// own ordering (IndexedDB commits same-store transactions in creation
        /// order), and it settles the same way for the mirror: if our put was
        /// created last it lands last, so the mirror already holds what the
        /// backend ends up holding — and the OTHER tab hears our commit and
        /// corrects itself.
        // See `apply_external`.
        #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
        pub(crate) fn refresh_key(&self, key: String) {
            if self.in_flight.borrow().contains_key(&key) {
                return;
            }
            let store = self.clone();
            let request = self.backend.get(&key);
            spawn(async move {
                match request.await {
                    Ok(value) => store.apply_external(&key, value),
                    // A failed re-read leaves the mirror as it was. It is NOT a
                    // persistence failure — nothing of the user's was lost — so it
                    // does not go to the toast channel; the stale row is the cost,
                    // and the next write to that key notifies again.
                    Err(_) => {}
                }
            });
        }

        /// Seed the log with a boot-time failure so the very first frame toasts it.
        // The browser's boot notice; the native boot reports through the file store it falls back to.
        #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
        pub(crate) fn report(&self, message: String) {
            self.errors.record(message);
        }

        /// Backend pushes issued but not yet settled (verification hook).
        pub(crate) fn pending(&self) -> usize {
            self.pending.get()
        }

        /// Every persistence failure this session (verification hook).
        // Browser verification hooks.
        #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
        pub(crate) fn error_history(&self) -> Vec<String> {
            self.errors.all()
        }

        /// Byte length of a stored document, or `None` if absent (verification
        /// hook — a multi-megabyte payload is not worth marshalling into JS just
        /// to measure it).
        // Browser verification hooks.
        #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
        pub(crate) fn len_of(&self, name: &str) -> Option<usize> {
            self.entries.borrow().get(&Self::key(name)).map(|v| v.len() as usize)
        }

        /// Ask the backend for ONE `OnDisk` key's bytes, unless that is already
        /// asked. The answer lands through [`Self::resolve`].
        fn load(&self, key: &str) {
            if !self.loading.borrow_mut().insert(key.to_string()) {
                return;
            }
            let store = self.clone();
            let key = key.to_string();
            let issued = self.tick();
            let request = self.backend.get(&key);
            spawn(async move {
                let outcome = request.await;
                store.loading.borrow_mut().remove(&key);
                store.resolve(&key, outcome, issued);
            });
        }

        /// Fold a loaded `OnDisk` value in — ONLY if the entry is still
        /// `OnDisk`. A write or a remove that happened while the bytes were on
        /// their way is newer than them, and wins.
        ///
        /// A failed load leaves the entry `OnDisk`, so the next `read` asks
        /// again. Like a failed refresh it does not toast: nothing of the
        /// user's was lost (S1 owns how a transport failure is shown).
        fn resolve(&self, key: &str, outcome: Result<Option<String>, String>, issued: u64) {
            let Ok(value) = outcome else {
                return;
            };
            // An index answered after this body request may name a newer
            // document while leaving the entry OnDisk (even at the same size).
            // Its authority also outranks this response; discard it so a next
            // demand read can fetch the current body.
            if self.full_index_authority.get() > issued
                || self.freshness.borrow().get(key).is_some_and(|&tick| tick > issued) {
                return;
            }
            {
                let mut entries = self.entries.borrow_mut();
                if !matches!(entries.get(key), Some(Entry::OnDisk { .. })) {
                    return;
                }
                self.stamp_local(key);
                match value {
                    Some(value) => entries.insert(key.to_string(), Entry::Resident(value)),
                    // Listed, then gone before we read it: the backend is
                    // authoritative, the listing was stale.
                    None => entries.remove(key),
                };
            }
            self.mutations.set(self.mutations.get().wrapping_add(1));
            if let Some(wake) = &self.wake {
                wake();
            }
        }

        /// Load every `OnDisk` entry in the background, ONE at a time, so a
        /// catalog of thousands of parts streams in behind the first frame
        /// instead of in front of it (or all at once in front of the user's own
        /// saves). A key a `read` already asked for is not asked twice, and a
        /// key written meanwhile is skipped.
        pub(crate) fn prefetch(&self) {
            let keys: Vec<String> = self
                .entries
                .borrow()
                .iter()
                .filter(|(_, entry)| matches!(entry, Entry::OnDisk { .. }))
                .map(|(key, _)| key.clone())
                .collect();
            if keys.is_empty() {
                return;
            }
            let store = self.clone();
            spawn(async move {
                for key in keys {
                    let wanted = matches!(store.entries.borrow().get(&key), Some(Entry::OnDisk { .. }));
                    if !wanted || !store.loading.borrow_mut().insert(key.clone()) {
                        continue;
                    }
                    let issued = store.tick();
                    let outcome = store.backend.get(&key).await;
                    store.loading.borrow_mut().remove(&key);
                    store.resolve(&key, outcome, issued);
                }
            });
        }
    }

    impl ModelStore for MirrorStore {
        fn backend_label(&self) -> String {
            self.backend.label()
        }

        fn plm_client(&self) -> Option<std::rc::Rc<crate::plm::client::PlmClient>> {
            self.backend.plm_client()
        }

        fn plm_revision(&self, key: &str) -> Option<crate::plm::client::IndexEntry> {
            self.backend.plm_revision(key)
        }

        /// A feed move: what the server now lists for `keys` — or, `stale`,
        /// for everything — is folded into the entries as LAZY `OnDisk`
        /// rows, so a document another client, the web app or Generate made
        /// after this app booted is listed and opens without a reload (bodies
        /// load on demand, as at boot). See [`Self::reconcile`] for what is
        /// never touched.
        fn refresh_plm_index(&self, keys: &[String], stale: bool) {
            // A backend that keeps no index answers `Err` and nothing moves.
            if !stale && keys.is_empty() {
                return;
            }
            let store = self.clone();
            // The answer speaks for the server AS OF NOW: ticked before the
            // request leaves, so anything local that happens while it is on
            // the wire outranks it.
            let issued = self.tick();
            if stale {
                let request = self.backend.index_all();
                spawn(async move {
                    if let Ok(rows) = request.await {
                        store.reconcile(None, rows, issued);
                    }
                });
            } else {
                let asked: Vec<String> = keys.iter().map(|key| format!("{PREFIX}{key}")).collect();
                let request = self.backend.index_rows(keys);
                spawn(async move {
                    if let Ok(rows) = request.await {
                        store.reconcile(Some(&asked), rows, issued);
                    }
                });
            }
        }
        fn remember_saved_document(&self, name: &str, contents: &str) {
            let key = Self::key(name);
            self.stamp_local(&key);
            self.entries.borrow_mut().insert(key, Entry::Resident(contents.into()));
            self.mutations.set(self.mutations.get().wrapping_add(1));
        }

        fn list(&self) -> Vec<String> {
            // `BTreeMap` iterates in key order, so the names come out sorted.
            self.entries
                .borrow()
                .keys()
                .filter_map(|key| key.strip_prefix(PREFIX).map(str::to_string))
                .collect()
        }

        fn read(&self, name: &str) -> Option<String> {
            let key = Self::key(name);
            let held = self.entries.borrow().get(&key).cloned();
            match held {
                Some(Entry::Resident(value)) => Some(value),
                // Listed but not loaded: ask for it and answer `None` NOW — the
                // API is synchronous. The bytes land through `resolve`, which
                // moves the generation and wakes the frame, so a caller keyed on
                // the generation reads again. `residency` tells the two `None`s
                // apart.
                Some(Entry::OnDisk { .. }) => {
                    self.load(&key);
                    None
                }
                None => None,
            }
        }

        fn canonical_identity(&self, name: &str) -> String {
            // The explorer's spelling of a document key (`browser_entries`);
            // a reserved name has no other spelling.
            match Self::key(name).strip_prefix(PREFIX) {
                Some(relative) => Self::virtual_model_path(relative),
                None => name.to_string(),
            }
        }

        fn residency(&self, name: &str) -> Residency {
            match self.entries.borrow().get(&Self::key(name)) {
                Some(Entry::Resident(_)) => Residency::Resident,
                Some(Entry::OnDisk { .. }) => Residency::OnDisk,
                None => Residency::Absent,
            }
        }

        fn write(&self, name: &str, contents: &str) -> Result<(), String> {
            let key = Self::key(name);
            self.stamp_local(&key);
            self.entries
                .borrow_mut()
                .insert(key.clone(), Entry::Resident(contents.to_string()));
            // The mirror is authoritative for this session, so `Ok` is honest:
            // every subsequent read sees the new bytes. Durability is the
            // backend's job and its failure arrives via `take_persistence_errors`.
            let backend = self.backend.clone();
            let (put_key, contents) = (key.clone(), contents.to_string());
            self.push(name, &key, move || backend.put(&put_key, &contents));
            self.mutations.set(self.mutations.get().wrapping_add(1));
            Ok(())
        }

        fn remove(&self, name: &str) -> Result<(), String> {
            let key = Self::key(name);
            self.stamp_local(&key);
            // The backend delete is pushed either way (the mirror may be
            // ahead of it), but only a delete that found something COUNTS —
            // the same rule `FileModelStore::remove` keeps.
            let existed = self.entries.borrow_mut().remove(&key).is_some();
            let backend = self.backend.clone();
            let delete_key = key.clone();
            self.push(name, &key, move || backend.delete(&delete_key));
            if existed {
                self.mutations.set(self.mutations.get().wrapping_add(1));
            }
            Ok(())
        }

        // `browser_write` needs no bump of its own: it routes through `write`.
        fn mutation_generation(&self) -> u64 {
            self.mutations.get()
        }

        fn pending_writes(&self) -> usize {
            self.pending()
        }

        fn take_persistence_errors(&self) -> Vec<String> {
            self.errors.drain()
        }

        fn browser_location(&self) -> String {
            // The RAW navigable path (breadcrumb / back-forward / path-edit rely on
            // this being feed-able straight back to `browser_navigate`); the human
            // "virtual filesystem" context lives in `backend_label`.
            self.browser_dir.borrow().clone()
        }

        fn browser_entries(&self, extensions: &[&str]) -> Vec<BrowserEntry> {
            let current = self.browser_dir.borrow().clone();
            if current == "/" {
                return vec![BrowserEntry {
                    name: "models".into(),
                    identity: "/models".into(),
                    is_dir: true,
                    size: None,
                    modified: None,
                }];
            }
            let relative_dir = current
                .strip_prefix("/models")
                .unwrap_or("")
                .trim_matches('/');
            let prefix = if relative_dir.is_empty() {
                String::new()
            } else {
                format!("{relative_dir}/")
            };
            let wanted: Vec<String> = extensions
                .iter()
                .map(|extension| extension.trim_start_matches('.').to_ascii_lowercase())
                .collect();
            let mut entries: BTreeMap<String, BrowserEntry> = BTreeMap::new();
            for (key, value) in self.entries.borrow().iter() {
                let (item, is_model) = if let Some(item) = key.strip_prefix(PREFIX) {
                    (item, true)
                } else if let Some(item) = key.strip_prefix(DIR_PREFIX) {
                    (item.trim_end_matches('/'), false)
                } else {
                    continue;
                };
                let Some(rest) = item.strip_prefix(&prefix) else {
                    continue;
                };
                if rest.is_empty() {
                    continue;
                }
                if let Some((child, _)) = rest.split_once('/') {
                    entries
                        .entry(child.to_string())
                        .or_insert_with(|| BrowserEntry {
                            name: child.to_string(),
                            identity: Self::child_path(&current, child),
                            is_dir: true,
                            size: None,
                            modified: None,
                        });
                } else if is_model {
                    let name = super::model_file_name(rest);
                    let lower = name.to_ascii_lowercase();
                    if wanted.is_empty()
                        || wanted
                            .iter()
                            .any(|extension| lower.ends_with(&format!(".{extension}")))
                    {
                        // Size = the mirrored JSON's byte length. `modified` stays
                        // None: the key space carries values, not timestamps.
                        entries.insert(
                            name.clone(),
                            BrowserEntry {
                                name,
                                identity: Self::virtual_model_path(item),
                                is_dir: false,
                                size: Some(value.len()),
                                modified: None,
                            },
                        );
                    }
                } else {
                    entries
                        .entry(rest.to_string())
                        .or_insert_with(|| BrowserEntry {
                            name: rest.to_string(),
                            identity: Self::child_path(&current, rest),
                            is_dir: true,
                            size: None,
                            modified: None,
                        });
                }
            }
            let mut entries: Vec<_> = entries.into_values().collect();
            entries.sort_by(|a, b| b.is_dir.cmp(&a.is_dir).then_with(|| a.name.cmp(&b.name)));
            entries
        }

        fn browser_enter(&self, identity: &str) -> Result<(), String> {
            if identity == "/models" || identity.starts_with("/models/") {
                *self.browser_dir.borrow_mut() = identity.trim_end_matches('/').to_string();
                Ok(())
            } else {
                Err("the browser virtual filesystem is rooted at /models".into())
            }
        }

        fn browser_up(&self) -> Result<(), String> {
            let current = self.browser_dir.borrow().clone();
            if current == "/" {
                return Ok(());
            }
            let parent = current
                .rsplit_once('/')
                .map(|(parent, _)| parent)
                .unwrap_or("");
            *self.browser_dir.borrow_mut() = if parent.is_empty() {
                "/".into()
            } else {
                parent.into()
            };
            Ok(())
        }

        fn browser_home(&self) -> Result<(), String> {
            *self.browser_dir.borrow_mut() = "/models".into();
            Ok(())
        }

        fn browser_root(&self) -> Result<(), String> {
            *self.browser_dir.borrow_mut() = "/".into();
            Ok(())
        }

        fn browser_navigate(&self, location: &str) -> Result<(), String> {
            let loc = location.trim_end_matches('/');
            let loc = if loc.is_empty() { "/" } else { loc };
            if loc == "/" || loc == "/models" || loc.starts_with("/models/") {
                *self.browser_dir.borrow_mut() = loc.to_string();
                Ok(())
            } else {
                Err("the browser virtual filesystem is rooted at /models".into())
            }
        }

        fn browser_places(&self) -> Vec<BrowserPlace> {
            vec![
                BrowserPlace {
                    label: "Models".into(),
                    location: "/models".into(),
                    kind: PlaceKind::Models,
                },
                BrowserPlace {
                    label: "/".into(),
                    location: "/".into(),
                    kind: PlaceKind::Root,
                },
            ]
        }

        fn browser_create_dir(&self, name: &str) -> Result<(), String> {
            if name.is_empty()
                || name == "."
                || name == ".."
                || name.contains('/')
                || name.contains('\\')
            {
                return Err("folder name must be one path component".into());
            }
            let current = self.browser_dir.borrow().clone();
            if !current.starts_with("/models") {
                return Err("folders can only be created under /models".into());
            }
            let relative = Self::child_path(&current, name)
                .trim_start_matches("/models/")
                .to_string();
            // A folder is a zero-length marker key; the explorer derives the tree
            // from the key prefixes alone.
            let key = format!("{DIR_PREFIX}{relative}/");
            self.entries
                .borrow_mut()
                .insert(key.clone(), Entry::Resident(String::new()));
            let backend = self.backend.clone();
            let put_key = key.clone();
            self.push(name, &key, move || backend.put(&put_key, ""));
            Ok(())
        }

        fn browser_write(&self, name: &str, contents: &str) -> Result<String, String> {
            let file = model_display_name(name);
            if file.is_empty() {
                return Err("invalid file name".into());
            }
            let current = self.browser_dir.borrow().clone();
            if !current.starts_with("/models") {
                return Err("select a folder under /models".into());
            }
            let identity = Self::child_path(&current, &super::model_file_name(&file));
            self.write(&identity, contents)?;
            Ok(identity)
        }
    }
}

// --- Browser: IndexedDB documents + download / upload interchange --------------
#[cfg(target_arch = "wasm32")]
mod web_model {
    use super::mirror_store::{
        BackendFuture, MirrorStore, StoreBackend, UnavailableBackend, DIR_PREFIX, PREFIX,
    };
    use super::{
        model_display_name, BrowserEntry, BrowserPlace, ImportedFile, ModelStore,
    };
    use std::cell::RefCell;
    use std::future::Future;
    use std::pin::Pin;
    use std::rc::Rc;
    use std::task::{Context, Poll, Waker};
    use wasm_bindgen::prelude::*;
    use wasm_bindgen::JsCast;

    // The origin-private persistent store on the web is **IndexedDB**. It replaced
    // `localStorage`, whose ~5-10 MB per-origin quota cannot hold a single native
    // BREP payload (a measured STEP import: ~35 MB), and it is the only browser
    // store that is both large and enumerable without a permission gesture (unlike
    // the File System Access API, which fails headless). Its API is ASYNC, hence
    // the mirror in [`mirror_store`](super::mirror_store); the key scheme is the
    // localStorage one, byte for byte. **Download + upload** below still bridge to
    // the user's REAL filesystem for portability.
    const DB_NAME: &str = "brep-app";
    const DB_VERSION: u32 = 1;
    /// The single key/value object store; keys are the `brep-app:*` strings.
    const STORE_NAME: &str = "kv";
    /// The cross-tab change channel. IndexedDB is shared by every tab of the
    /// origin but says NOTHING when another connection writes it, so each tab's
    /// hydrated mirror would sit at its boot-time contents until reloaded — save a
    /// part in one tab, and the tab holding the assembly cannot insert it. Every
    /// tab posts the key it just committed here and re-reads the keys it is told
    /// about; a [`BroadcastChannel`](web_sys::BroadcastChannel) never echoes to
    /// the object that sent the message, so a tab hears only the others.
    const CHANNEL_NAME: &str = "brep-app:store";

    thread_local! {
        /// The single hidden `<input type=file>`, created once and reused.
        static IMPORT_INPUT: RefCell<Option<web_sys::HtmlInputElement>> = const { RefCell::new(None) };
        /// The most recent completed upload, awaiting the panel's poll.
        static IMPORTED: RefCell<Option<ImportedFile>> = const { RefCell::new(None) };
        /// The file chooser's own input and slot ([`ModelStore::begin_pick_file`]):
        /// a picked attachment must never be taken by the model-import lane.
        static PICK_INPUT: RefCell<Option<web_sys::HtmlInputElement>> = const { RefCell::new(None) };
        static PICKED: RefCell<Option<ImportedFile>> = const { RefCell::new(None) };
        /// The egui context, so an ASYNC callback (a `FileReader` load, a failed
        /// IndexedDB write) can wake the reactive eframe loop (the app only
        /// repaints on input events + explicit requests). Without it a completed
        /// upload — or a "your model did not save" notice — sits unshown until the
        /// user happens to move the mouse. Seeded once at boot via [`set_repaint_ctx`].
        static REPAINT_CTX: RefCell<Option<eframe::egui::Context>> = const { RefCell::new(None) };
        /// The store built by [`hydrate`] inside the async wasm entry point,
        /// waiting for `BrepApp::new` to pick it up through
        /// [`default_model_store`](super::default_model_store). The hand-off is a
        /// thread-local rather than a closure capture so the app constructor keeps
        /// its platform-free signature.
        static BOOT_STORE: RefCell<Option<IdbModelStore>> = const { RefCell::new(None) };
        /// Why a page the PLM serves is on IndexedDB instead (the PLM refused
        /// its session), shown once on the first frame.
        static BOOT_NOTICE: RefCell<Option<String>> = const { RefCell::new(None) };
        /// The same reason, kept for the PLM tab after the notice is drained.
        static BOOT_REASON: RefCell<Option<String>> = const { RefCell::new(None) };
    }

    fn set_boot_notice(notice: String) {
        BOOT_REASON.with(|r| *r.borrow_mut() = Some(notice.clone()));
        BOOT_NOTICE.with(|n| *n.borrow_mut() = Some(format!("{notice} — this page uses the browser's storage")));
    }

    /// Register the egui context used to wake the frame loop when an async browser
    /// upload — or a write-behind persistence failure — completes. Called once from
    /// the app shell at construction.
    pub fn set_repaint_ctx(ctx: eframe::egui::Context) {
        REPAINT_CTX.with(|c| *c.borrow_mut() = Some(ctx));
    }

    /// Ask the reactive frame loop for a repaint. Reads `REPAINT_CTX` at CALL time,
    /// so it does not matter that the store is built (during boot) before the app
    /// shell registers the context.
    fn wake_frame_loop() {
        REPAINT_CTX.with(|c| {
            if let Some(ctx) = c.borrow().as_ref() {
                ctx.request_repaint();
            }
        });
    }

    // --- IndexedDB request/transaction -> Future ----------------------------------
    // Hand-rolled rather than pulling `indexed_db_futures`: this crate deliberately
    // keeps its wasm dependency tree thin (see the `ehttp` note in Cargo.toml about
    // the `getrandom` dep-tree problem), and the whole adapter is the ~60 lines
    // below. It owns its `Closure`s — dropping the future clears the handlers — so
    // a session of saves does not leak a pair of JS closures per write, which a
    // `Closure::forget()` sketch would.

    #[derive(Default)]
    struct Settled {
        outcome: Option<Result<JsValue, String>>,
        waker: Option<Waker>,
    }

    /// Which DOM event pair the future is listening to. Kept so `Drop` can detach
    /// the handlers (and with them the Rust closures' reference back to the target).
    enum EventSource {
        Request(web_sys::IdbRequest),
        Transaction(web_sys::IdbTransaction),
    }

    /// One IndexedDB completion, as a `Future`.
    struct IdbFuture {
        source: EventSource,
        settled: Rc<RefCell<Settled>>,
        /// Owned so the closures live exactly as long as the future.
        _handlers: Vec<Closure<dyn FnMut(web_sys::Event)>>,
    }

    fn settle(settled: &Rc<RefCell<Settled>>, outcome: Result<JsValue, String>) {
        let waker = {
            let mut settled = settled.borrow_mut();
            if settled.outcome.is_none() {
                settled.outcome = Some(outcome);
            }
            settled.waker.take()
        };
        if let Some(waker) = waker {
            waker.wake();
        }
    }

    fn request_error(request: &web_sys::IdbRequest) -> String {
        request
            .error()
            .ok()
            .flatten()
            .map(|error| format!("{}: {}", error.name(), error.message()))
            .unwrap_or_else(|| "IndexedDB request failed".into())
    }

    fn js_error(value: &JsValue) -> String {
        value
            .as_string()
            .or_else(|| js_sys::Reflect::get(value, &JsValue::from_str("message")).ok()?.as_string())
            .unwrap_or_else(|| format!("{value:?}"))
    }

    /// Resolve when `request` succeeds, with its `result`.
    fn on_request(request: web_sys::IdbRequest) -> IdbFuture {
        let settled = Rc::new(RefCell::new(Settled::default()));
        let success = {
            let settled = settled.clone();
            let request = request.clone();
            Closure::wrap(Box::new(move |_event: web_sys::Event| {
                let value = request.result().unwrap_or(JsValue::UNDEFINED);
                settle(&settled, Ok(value));
            }) as Box<dyn FnMut(web_sys::Event)>)
        };
        let failure = {
            let settled = settled.clone();
            let request = request.clone();
            Closure::wrap(Box::new(move |_event: web_sys::Event| {
                settle(&settled, Err(request_error(&request)));
            }) as Box<dyn FnMut(web_sys::Event)>)
        };
        request.set_onsuccess(Some(success.as_ref().unchecked_ref()));
        request.set_onerror(Some(failure.as_ref().unchecked_ref()));
        IdbFuture {
            source: EventSource::Request(request),
            settled,
            _handlers: vec![success, failure],
        }
    }

    /// Resolve when `transaction` COMMITS.
    ///
    /// A write must be awaited here, not on the `put` request's `onsuccess`: the
    /// request succeeds before the transaction commits, and a page reload can abort
    /// an uncommitted `readwrite` transaction. For a multi-megabyte payload that
    /// window is real, and "the save is durable" is exactly the claim the pending
    /// counter and the reload test rest on.
    fn on_transaction(transaction: web_sys::IdbTransaction) -> IdbFuture {
        let settled = Rc::new(RefCell::new(Settled::default()));
        let complete = {
            let settled = settled.clone();
            Closure::wrap(Box::new(move |_event: web_sys::Event| {
                settle(&settled, Ok(JsValue::UNDEFINED));
            }) as Box<dyn FnMut(web_sys::Event)>)
        };
        let describe = {
            let transaction = transaction.clone();
            move |fallback: &str| {
                transaction
                    .error()
                    .map(|error| format!("{}: {}", error.name(), error.message()))
                    .unwrap_or_else(|| fallback.to_string())
            }
        };
        let failure = {
            let settled = settled.clone();
            let describe = describe.clone();
            Closure::wrap(Box::new(move |_event: web_sys::Event| {
                settle(&settled, Err(describe("IndexedDB transaction failed")));
            }) as Box<dyn FnMut(web_sys::Event)>)
        };
        let abort = {
            let settled = settled.clone();
            Closure::wrap(Box::new(move |_event: web_sys::Event| {
                settle(
                    &settled,
                    Err(describe("IndexedDB transaction aborted (quota?)")),
                );
            }) as Box<dyn FnMut(web_sys::Event)>)
        };
        transaction.set_oncomplete(Some(complete.as_ref().unchecked_ref()));
        transaction.set_onerror(Some(failure.as_ref().unchecked_ref()));
        transaction.set_onabort(Some(abort.as_ref().unchecked_ref()));
        IdbFuture {
            source: EventSource::Transaction(transaction),
            settled,
            _handlers: vec![complete, failure, abort],
        }
    }

    impl Future for IdbFuture {
        type Output = Result<JsValue, String>;

        fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            let mut settled = self.settled.borrow_mut();
            if let Some(outcome) = settled.outcome.take() {
                return Poll::Ready(outcome);
            }
            settled.waker = Some(cx.waker().clone());
            Poll::Pending
        }
    }

    impl Drop for IdbFuture {
        fn drop(&mut self) {
            // Detach the handlers, breaking the target -> closure -> target cycle
            // so the closures we own are actually freed.
            match &self.source {
                EventSource::Request(request) => {
                    request.set_onsuccess(None);
                    request.set_onerror(None);
                }
                EventSource::Transaction(transaction) => {
                    transaction.set_oncomplete(None);
                    transaction.set_onerror(None);
                    transaction.set_onabort(None);
                }
            }
        }
    }

    // --- the IndexedDB backend -----------------------------------------------------

    /// [`StoreBackend`] over one IndexedDB object store. The ONLY place in the
    /// crate that knows what IndexedDB is.
    pub(super) struct IdbBackend {
        db: web_sys::IdbDatabase,
        /// The cross-tab notification channel (see [`CHANNEL_NAME`]), or `None`
        /// where the browser does not offer one — then this tab still saves and
        /// still reads, it just will not hear about another tab's saves.
        channel: Option<web_sys::BroadcastChannel>,
    }

    impl IdbBackend {
        /// Open (creating on first use) the application database. `Err` on any
        /// browser that blocks storage — private windows, "block all cookies"
        /// setups — which the caller turns into a loud, non-persisting session.
        ///
        /// `channel` is the tab's ONE [`BroadcastChannel`](web_sys::BroadcastChannel)
        /// handle, shared with [`hydrate`]'s listener: posting and listening on the
        /// same object is what keeps a tab from hearing its own writes back (the
        /// channel never delivers a message to the object that sent it).
        async fn open(channel: Option<web_sys::BroadcastChannel>) -> Result<Self, String> {
            let factory = web_sys::window()
                .ok_or("no window")?
                .indexed_db()
                .map_err(|e| js_error(&e))?
                .ok_or("no indexedDB on this window")?;
            let request = factory
                .open_with_u32(DB_NAME, DB_VERSION)
                .map_err(|e| js_error(&e))?;

            // Create the object store on first open / version bump. The request is
            // captured directly (rather than read off the event target) so no
            // `EventTarget` binding is needed.
            let upgrading = request.clone();
            let upgrade = Closure::wrap(Box::new(move |_event: web_sys::Event| {
                if let Ok(value) = upgrading.result() {
                    if let Ok(db) = value.dyn_into::<web_sys::IdbDatabase>() {
                        // Errors here mean the store already exists — harmless.
                        let _ = db.create_object_store(STORE_NAME);
                    }
                }
            }) as Box<dyn FnMut(web_sys::Event)>);
            request.set_onupgradeneeded(Some(upgrade.as_ref().unchecked_ref()));

            let opened = on_request(request.clone().unchecked_into::<web_sys::IdbRequest>()).await;
            request.set_onupgradeneeded(None);
            drop(upgrade);

            let db = opened?
                .dyn_into::<web_sys::IdbDatabase>()
                .map_err(|_| "IndexedDB open returned no database".to_string())?;
            Ok(Self { db, channel })
        }

        /// Tell the other tabs of this origin that `key` changed, so their mirrors
        /// can re-read it. Only the FILESYSTEM key space is announced: `@settings`,
        /// `@dock_layout`, `@recovery` and friends are per-tab working state, and
        /// syncing them live would have two windows fight over one layout.
        fn announce(channel: &Option<web_sys::BroadcastChannel>, key: &str) {
            if !key.starts_with(PREFIX) && !key.starts_with(DIR_PREFIX) {
                return;
            }
            if let Some(channel) = channel {
                // A failed post costs this tab nothing it can act on — the write
                // itself landed — so it is dropped rather than toasted.
                let _ = channel.post_message(&JsValue::from_str(key));
            }
        }

        /// Start a transaction and reach its object store. The transaction handle
        /// is returned so a write can await its COMMIT.
        fn transact(
            &self,
            mode: web_sys::IdbTransactionMode,
        ) -> Result<(web_sys::IdbTransaction, web_sys::IdbObjectStore), String> {
            let transaction = self
                .db
                .transaction_with_str_and_mode(STORE_NAME, mode)
                .map_err(|e| js_error(&e))?;
            let store = transaction.object_store(STORE_NAME).map_err(|e| js_error(&e))?;
            Ok((transaction, store))
        }
    }

    impl StoreBackend for IdbBackend {
        fn label(&self) -> String {
            "browser storage (IndexedDB) · download/upload for files".into()
        }

        /// IndexedDB commits `readwrite` transactions in creation order, and
        /// `put` / `delete` create theirs synchronously — so the mirror issues
        /// every write at once, exactly as it did before the send queue existed.
        fn orders_writes_per_key(&self) -> bool {
            true
        }

        fn load_all(&self) -> BackendFuture<Vec<(String, String)>> {
            // Two whole-store requests in ONE readonly transaction rather than a
            // cursor: `getAllKeys` and `getAll` both come back in key order, so
            // zipping them reconstructs the key space in one round trip each.
            let started = (|| -> Result<(IdbFuture, IdbFuture), String> {
                let (_transaction, store) = self.transact(web_sys::IdbTransactionMode::Readonly)?;
                let keys = store.get_all_keys().map_err(|e| js_error(&e))?;
                let values = store.get_all().map_err(|e| js_error(&e))?;
                Ok((on_request(keys), on_request(values)))
            })();
            Box::pin(async move {
                let (keys, values) = started?;
                let keys = js_sys::Array::from(&keys.await?);
                let values = js_sys::Array::from(&values.await?);
                if keys.length() != values.length() {
                    return Err("IndexedDB returned mismatched keys and values".into());
                }
                let mut entries = Vec::with_capacity(keys.length() as usize);
                for i in 0..keys.length() {
                    let (Some(key), Some(value)) =
                        (keys.get(i).as_string(), values.get(i).as_string())
                    else {
                        // A non-string entry is not ours; skipping it is safer than
                        // failing the whole hydrate.
                        continue;
                    };
                    entries.push((key, value));
                }
                Ok(entries)
            })
        }

        fn get(&self, key: &str) -> BackendFuture<Option<String>> {
            let started = (|| -> Result<IdbFuture, String> {
                let (_transaction, store) = self.transact(web_sys::IdbTransactionMode::Readonly)?;
                let request = store.get(&JsValue::from_str(key)).map_err(|e| js_error(&e))?;
                Ok(on_request(request))
            })();
            Box::pin(async move {
                // An absent key resolves to `undefined`, which is the `None` the
                // mirror turns into a removal.
                Ok(started?.await?.as_string())
            })
        }

        fn put(&self, key: &str, value: &str) -> BackendFuture<()> {
            // The transaction is created and the request issued SYNCHRONOUSLY, so
            // two writes of the same key commit in call order (IndexedDB commits
            // `readwrite` transactions in creation order) no matter how the futures
            // are later polled — the per-key FIFO obligation in `StoreBackend`.
            let started = (|| -> Result<IdbFuture, String> {
                let (transaction, store) =
                    self.transact(web_sys::IdbTransactionMode::Readwrite)?;
                store
                    .put_with_key(&JsValue::from_str(value), &JsValue::from_str(key))
                    .map_err(|e| js_error(&e))?;
                Ok(on_transaction(transaction))
            })();
            let channel = self.channel.clone();
            let key = key.to_string();
            Box::pin(async move {
                started?.await?;
                // AFTER the commit, never before: a tab told about the key reads
                // it back immediately, and must not be able to read the old bytes.
                Self::announce(&channel, &key);
                Ok(())
            })
        }

        fn delete(&self, key: &str) -> BackendFuture<()> {
            let started = (|| -> Result<IdbFuture, String> {
                let (transaction, store) =
                    self.transact(web_sys::IdbTransactionMode::Readwrite)?;
                store
                    .delete(&JsValue::from_str(key))
                    .map_err(|e| js_error(&e))?;
                Ok(on_transaction(transaction))
            })();
            let channel = self.channel.clone();
            let key = key.to_string();
            Box::pin(async move {
                started?.await?;
                Self::announce(&channel, &key);
                Ok(())
            })
        }
    }

    // --- boot ----------------------------------------------------------------------

    /// Bring up browser persistence and hydrate the whole key space into memory.
    ///
    /// Called from the async wasm entry point BEFORE `eframe::WebRunner::start`, so
    /// by the time any synchronous [`ModelStore::read`] can run, the mirror is
    /// complete — that ordering is the entire reason the trait stays synchronous.
    ///
    /// On failure there is NO quiet fallback: the session gets an
    /// [`UnavailableBackend`], which keeps the app usable in memory while saying so
    /// in the panel header and toasting every write that does not persist.
    pub(super) async fn hydrate() {
        let wake: Rc<dyn Fn()> = Rc::new(wake_frame_loop);
        let channel = web_sys::BroadcastChannel::new(CHANNEL_NAME).ok();
        let core = match IdbBackend::open(channel.clone()).await {
            Ok(backend) => {
                let mut backend: Rc<dyn StoreBackend> = Rc::new(backend);
                // A page the PLM serves (D6, `/cad/app/`) keeps its documents
                // and preferences there, with `@recovery` in IndexedDB first.
                // A refusal keeps the page on IndexedDB and says why; a page
                // served from anywhere else never asks.
                match crate::plm::backend::open_web_session(backend.clone()).await {
                    Ok(Some(plm)) => backend = plm,
                    Ok(None) => {}
                    Err(sentence) => set_boot_notice(format!("PLM: {sentence}")),
                }
                // `load_index`'s default IS `load_all` with every value
                // resident, so a browser origin still hydrates whole; the
                // index-only PLM keeps document bodies unloaded until an actual
                // read. Never prefetch the catalog: its bodies can exceed WASM memory.
                match backend.load_index().await {
                    Ok(entries) => {
                        let core = MirrorStore::from_index(backend, entries, Some(wake));
                        // Hydration is the LAST all-at-once read. From here the
                        // mirror keeps up with the other tabs one key at a time,
                        // so a part saved next door is insertable here without a
                        // reload.
                        listen_for_other_tabs(&core, channel);
                        core
                    }
                    // The database opened but would not read. Writing against a
                    // half-known key space could overwrite documents the mirror
                    // never saw, so treat it as unavailable rather than risk that.
                    Err(message) => unavailable(
                        format!("IndexedDB could not be read ({message})"),
                        Some(wake),
                    ),
                }
            }
            Err(message) => unavailable(format!("IndexedDB unavailable ({message})"), Some(wake)),
        };
        install_verification_hooks(&core);
        BOOT_STORE.with(|c| *c.borrow_mut() = Some(IdbModelStore { core }));
    }

    /// Subscribe this tab's mirror to the other tabs' commits: each message is a
    /// key one of them just wrote, and the mirror re-reads exactly that key.
    ///
    /// The handler is `forget()`-ed — one per tab, alive for the session, the same
    /// pattern the verification hooks use. It holds a `MirrorStore` clone, which is
    /// another handle on the SAME session state, not a copy of it.
    fn listen_for_other_tabs(core: &MirrorStore, channel: Option<web_sys::BroadcastChannel>) {
        let Some(channel) = channel else {
            return;
        };
        let core = core.clone();
        let on_message = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
            if let Some(key) = event.data().as_string() {
                core.refresh_key(key);
            }
        }) as Box<dyn FnMut(web_sys::MessageEvent)>);
        channel.set_onmessage(Some(on_message.as_ref().unchecked_ref()));
        on_message.forget();
        // The channel object itself must outlive this call — a dropped
        // `BroadcastChannel` stops delivering — and it does: `IdbBackend` holds the
        // same handle for the life of the session.
    }

    /// A session with no persistence: empty, in-memory, and loud about it.
    fn unavailable(reason: String, wake: Option<Rc<dyn Fn()>>) -> MirrorStore {
        let core = MirrorStore::new(
            Rc::new(UnavailableBackend::new(reason.clone())),
            Vec::new(),
            wake,
        );
        // Seed the notice channel so the FIRST frame tells the user, before they
        // have saved anything and discovered it the hard way.
        core.report(format!(
            "{reason} — this session will not be saved; use Download to keep your work"
        ));
        core
    }

    /// Hand the hydrated store to `BrepApp::new`. If boot never ran (no code path
    /// does that today), the app still starts — non-persisting and saying so.
    pub(super) fn take_boot_store() -> Box<dyn ModelStore> {
        let store = BOOT_STORE.with(|c| c.borrow_mut().take()).unwrap_or_else(|| {
            let wake: Rc<dyn Fn()> = Rc::new(wake_frame_loop);
            IdbModelStore {
                core: unavailable("storage was never initialised".into(), Some(wake)),
            }
        });
        Box::new(store)
    }

    // --- verification hooks ---------------------------------------------------------

    /// Publish `window.__brepStore*` handles onto the LIVE store, in the same
    /// spirit as the app shell's `__brep*` state globals: the headed verifier needs
    /// to write a payload far larger than any UI gesture can type, then prove it
    /// survived a reload. `__brepStorePending` is what makes that check non-racy —
    /// it reaches zero only once the backing transaction has COMMITTED.
    /// `__brepStoreRead` is the read side of the same seam: a script that saves
    /// through the UI has to be able to look at what landed.
    fn install_verification_hooks(core: &MirrorStore) {
        let Some(window) = web_sys::window() else {
            return;
        };
        let publish = |name: &str, value: &JsValue| {
            let _ = js_sys::Reflect::set(&window, &JsValue::from_str(name), value);
        };

        let store = core.clone();
        let write = Closure::wrap(Box::new(move |name: String, contents: String| -> JsValue {
            match store.write(&name, &contents) {
                Ok(()) => JsValue::NULL,
                Err(message) => JsValue::from_str(&message),
            }
        }) as Box<dyn FnMut(String, String) -> JsValue>);
        publish("__brepStoreWrite", write.as_ref());
        write.forget();

        let store = core.clone();
        let len = Closure::wrap(Box::new(move |name: String| -> f64 {
            store.len_of(&name).map(|n| n as f64).unwrap_or(-1.0)
        }) as Box<dyn FnMut(String) -> f64>);
        publish("__brepStoreLen", len.as_ref());
        len.forget();

        // The READ sibling of `__brepStoreWrite`. Without it a headed script can
        // only prove a save happened, not WHAT was saved: the assemblies sweep
        // used to read the document straight out of `localStorage`, which the
        // IndexedDB migration silently emptied.
        let store = core.clone();
        let read = Closure::wrap(Box::new(move |name: String| -> JsValue {
            match store.read(&name) {
                Some(contents) => JsValue::from_str(&contents),
                None => JsValue::NULL,
            }
        }) as Box<dyn FnMut(String) -> JsValue>);
        publish("__brepStoreRead", read.as_ref());
        read.forget();

        let store = core.clone();
        let list = Closure::wrap(Box::new(move || -> JsValue {
            JsValue::from_str(&serde_json::to_string(&store.list()).unwrap_or_default())
        }) as Box<dyn FnMut() -> JsValue>);
        publish("__brepStoreList", list.as_ref());
        list.forget();

        let store = core.clone();
        let errors = Closure::wrap(Box::new(move || -> JsValue {
            JsValue::from_str(&serde_json::to_string(&store.error_history()).unwrap_or_default())
        }) as Box<dyn FnMut() -> JsValue>);
        publish("__brepStoreErrors", errors.as_ref());
        errors.forget();

        // The DELETE sibling of `__brepStoreWrite`, so the two-tab check in
        // `web/verify_store.mjs` can prove the second half of live storage: a
        // document removed in one tab leaves the other tab's list as well.
        let store = core.clone();
        let remove = Closure::wrap(Box::new(move |name: String| -> JsValue {
            match store.remove(&name) {
                Ok(()) => JsValue::NULL,
                Err(message) => JsValue::from_str(&message),
            }
        }) as Box<dyn FnMut(String) -> JsValue>);
        publish("__brepStoreRemove", remove.as_ref());
        remove.forget();

        let store = core.clone();
        let pending = Closure::wrap(Box::new(move || -> f64 { store.pending() as f64 })
            as Box<dyn FnMut() -> f64>);
        publish("__brepStorePending", pending.as_ref());
        pending.forget();
    }

    // --- real-file interchange (download / upload) ----------------------------------
    // Free functions, not methods: they are pure browser plumbing with no store
    // state, and keeping them out of the store type leaves `IdbModelStore` as a
    // thin seam between the mirror and this lane.

    /// Lazily create the reusable hidden file input, wiring its `change` handler
    /// (which reads the chosen file and stashes it for `take_import`). `accept` is
    /// (re)applied every call so the picker's filter matches the current lane (the
    /// `.nbrep` model lane vs. a `.step` import lane).
    /// The file chooser's hidden input, created once: any file, its own name
    /// kept whole, its bytes into [`PICKED`].
    fn pick_input() -> Option<web_sys::HtmlInputElement> {
        if let Some(existing) = PICK_INPUT.with(|c| c.borrow().clone()) {
            return Some(existing);
        }
        let document = web_sys::window()?.document()?;
        let input: web_sys::HtmlInputElement = document.create_element("input").ok()?.dyn_into().ok()?;
        input.set_type("file");
        input.set_hidden(true);
        let input_for_cb = input.clone();
        let onchange = Closure::wrap(Box::new(move |_e: web_sys::Event| {
            let Some(file) = input_for_cb.files().and_then(|files| files.get(0)) else { return };
            let name = file.name();
            let Ok(reader) = web_sys::FileReader::new() else { return };
            let reader_for_load = reader.clone();
            let onload = Closure::wrap(Box::new(move |_e: web_sys::Event| {
                if let Ok(value) = reader_for_load.result() {
                    let bytes = js_sys::Uint8Array::new(&value).to_vec();
                    PICKED.with(|c| *c.borrow_mut() = Some(ImportedFile { name: name.clone(), bytes }));
                    wake_frame_loop();
                }
            }) as Box<dyn FnMut(web_sys::Event)>);
            reader.set_onload(Some(onload.as_ref().unchecked_ref()));
            onload.forget();
            let _ = reader.read_as_array_buffer(&file);
        }) as Box<dyn FnMut(web_sys::Event)>);
        input.set_onchange(Some(onchange.as_ref().unchecked_ref()));
        onchange.forget();
        if let Some(body) = document.body() {
            let _ = body.append_child(&input);
        }
        PICK_INPUT.with(|c| *c.borrow_mut() = Some(input.clone()));
        Some(input)
    }

    fn ensure_input(accept: &str) -> Option<web_sys::HtmlInputElement> {
        if let Some(existing) = IMPORT_INPUT.with(|c| c.borrow().clone()) {
            existing.set_accept(accept);
            return Some(existing);
        }
        let document = web_sys::window()?.document()?;
        let input: web_sys::HtmlInputElement =
            document.create_element("input").ok()?.dyn_into().ok()?;
        input.set_type("file");
        input.set_accept(accept);
        input.set_hidden(true);

        // Read bytes so binary STL is not corrupted at the browser boundary.
        let input_for_cb = input.clone();
        let onchange = Closure::wrap(Box::new(move |_e: web_sys::Event| {
            let Some(files) = input_for_cb.files() else { return };
            let Some(file) = files.get(0) else { return };
            let name = file.name();
            let Ok(reader) = web_sys::FileReader::new() else { return };
            let reader_for_load = reader.clone();
            let onload = Closure::wrap(Box::new(move |_e: web_sys::Event| {
                if let Ok(value) = reader_for_load.result() {
                    let bytes = js_sys::Uint8Array::new(&value).to_vec();
                    IMPORTED.with(|c| *c.borrow_mut() = Some(ImportedFile {
                        name: model_display_name(&name),
                        bytes,
                    }));
                    // Wake the reactive frame loop so the file panel polls
                    // `take_import` THIS frame, not on the next stray input event.
                    wake_frame_loop();
                }
            }) as Box<dyn FnMut(web_sys::Event)>);
            reader.set_onload(Some(onload.as_ref().unchecked_ref()));
            // One small per-import leak (the app runs for the page lifetime).
            onload.forget();
            let _ = reader.read_as_array_buffer(&file);
        }) as Box<dyn FnMut(web_sys::Event)>);
        input.set_onchange(Some(onchange.as_ref().unchecked_ref()));
        onchange.forget(); // created once — leak is bounded

        if let Some(body) = document.body() {
            let _ = body.append_child(&input);
        }
        IMPORT_INPUT.with(|c| *c.borrow_mut() = Some(input.clone()));
        Some(input)
    }

    /// Offer `contents` to the user as a download named EXACTLY `file_name`, via a
    /// Blob object-URL + a synthetic anchor click.
    fn download(file_name: &str, mime: &str, contents: &str) -> Result<(), String> {
        let parts = js_sys::Array::of1(&JsValue::from_str(contents));
        let options = web_sys::BlobPropertyBag::new();
        options.set_type(mime);
        let blob = web_sys::Blob::new_with_str_sequence_and_options(&parts, &options)
            .map_err(|_| "blob create failed".to_string())?;
        click_download(file_name, &blob)
    }

    /// [`download`] for BINARY content: the bytes are handed to the Blob as a
    /// `Uint8Array`, so nothing on the way out reinterprets them as UTF-8.
    fn download_bytes(file_name: &str, mime: &str, contents: &[u8]) -> Result<(), String> {
        let array = js_sys::Uint8Array::new_with_length(contents.len() as u32);
        array.copy_from(contents);
        let parts = js_sys::Array::of1(&array);
        let options = web_sys::BlobPropertyBag::new();
        options.set_type(mime);
        let blob = web_sys::Blob::new_with_u8_array_sequence_and_options(&parts, &options)
            .map_err(|_| "blob create failed".to_string())?;
        click_download(file_name, &blob)
    }

    /// The half both downloads share: an object URL for `blob`, a synthetic
    /// anchor click carrying EXACTLY `file_name`, then the URL released.
    fn click_download(file_name: &str, blob: &web_sys::Blob) -> Result<(), String> {
        let document = web_sys::window()
            .and_then(|w| w.document())
            .ok_or("no document")?;
        let url = web_sys::Url::create_object_url_with_blob(blob)
            .map_err(|_| "object url failed".to_string())?;
        let anchor: web_sys::HtmlAnchorElement = document
            .create_element("a")
            .map_err(|_| "anchor create failed".to_string())?
            .dyn_into()
            .map_err(|_| "anchor cast failed".to_string())?;
        anchor.set_href(&url);
        anchor.set_download(file_name);
        anchor.click();
        let _ = web_sys::Url::revoke_object_url(&url);
        Ok(())
    }

    /// The browser model store: the mirrored key space (which owns every CRUD and
    /// explorer method) plus this platform's real-file interchange lane.
    pub(super) struct IdbModelStore {
        core: MirrorStore,
    }

    impl ModelStore for IdbModelStore {
        // --- delegated to the mirror ------------------------------------------
        fn backend_label(&self) -> String {
            self.core.backend_label()
        }
        fn plm_client(&self) -> Option<std::rc::Rc<crate::plm::client::PlmClient>> {
            self.core.plm_client()
        }
        fn plm_revision(&self, key: &str) -> Option<crate::plm::client::IndexEntry> {
            self.core.plm_revision(key)
        }
        fn refresh_plm_index(&self, keys: &[String], stale: bool) {
            self.core.refresh_plm_index(keys, stale)
        }
        fn remember_saved_document(&self, name: &str, contents: &str) {
            self.core.remember_saved_document(name, contents)
        }
        fn list(&self) -> Vec<String> {
            self.core.list()
        }
        fn read(&self, name: &str) -> Option<String> {
            self.core.read(name)
        }
        fn residency(&self, name: &str) -> super::Residency {
            self.core.residency(name)
        }
        fn canonical_identity(&self, name: &str) -> String {
            self.core.canonical_identity(name)
        }
        fn write(&self, name: &str, contents: &str) -> Result<(), String> {
            self.core.write(name, contents)
        }
        fn remove(&self, name: &str) -> Result<(), String> {
            self.core.remove(name)
        }
        fn mutation_generation(&self) -> u64 {
            self.core.mutation_generation()
        }
        fn take_persistence_errors(&self) -> Vec<String> {
            let mut errors: Vec<String> = BOOT_NOTICE.with(|n| n.borrow_mut().take()).into_iter().collect();
            errors.extend(self.core.take_persistence_errors());
            errors
        }
        fn plm_session(&self) -> Option<crate::plm::connection::Session> {
            use crate::plm::connection::{parse_label, Keep, Session, Status};
            if let Some((url, username)) = parse_label(&self.core.backend_label()) {
                return Some(Session { status: Status::Connected { url, username }, keep: Keep::BrowserSession });
            }
            let location = web_sys::window()?.location();
            if !crate::plm::backend::served_by_plm(&location.pathname().ok()?) {
                return None;
            }
            let reason = BOOT_REASON.with(|r| r.borrow().clone());
            Some(Session { status: Status::NotConnected { url: location.origin().ok()?, reason }, keep: Keep::BrowserSession })
        }
        fn pending_writes(&self) -> usize {
            self.core.pending_writes()
        }
        fn browser_location(&self) -> String {
            self.core.browser_location()
        }
        fn browser_entries(&self, extensions: &[&str]) -> Vec<BrowserEntry> {
            self.core.browser_entries(extensions)
        }
        fn browser_enter(&self, identity: &str) -> Result<(), String> {
            self.core.browser_enter(identity)
        }
        fn browser_up(&self) -> Result<(), String> {
            self.core.browser_up()
        }
        fn browser_home(&self) -> Result<(), String> {
            self.core.browser_home()
        }
        fn browser_root(&self) -> Result<(), String> {
            self.core.browser_root()
        }
        fn browser_navigate(&self, location: &str) -> Result<(), String> {
            self.core.browser_navigate(location)
        }
        fn browser_places(&self) -> Vec<BrowserPlace> {
            self.core.browser_places()
        }
        fn browser_create_dir(&self, name: &str) -> Result<(), String> {
            self.core.browser_create_dir(name)
        }
        fn browser_write(&self, name: &str, contents: &str) -> Result<String, String> {
            self.core.browser_write(name, contents)
        }

        // --- the browser's real-file lane -------------------------------------
        fn supports_file_interchange(&self) -> bool {
            true
        }

        fn begin_pick_file(&self) -> Result<(), String> {
            let input = pick_input().ok_or("the browser's file chooser is unavailable")?;
            // Clear so choosing the same file again still fires `change`.
            input.set_value("");
            input.click();
            Ok(())
        }

        fn take_picked_file(&self) -> Option<ImportedFile> {
            PICKED.with(|c| c.borrow_mut().take())
        }

        /// Offer the document as a `<name>.nbrep` download. The returned
        /// identity is the bare name (the browser owns where the download lands).
        fn export_file(&self, name: &str, contents: &str) -> Result<Option<String>, String> {
            let name = model_display_name(name);
            let file_name = super::model_file_name(&name);
            download(&file_name, super::media_type(&file_name), contents)?;
            Ok(Some(name))
        }

        fn begin_import(&self) -> Result<(), String> {
            let input =
                ensure_input(".nbrep,.fbrep,.tbrep").ok_or("file input unavailable")?;
            // Clear so re-selecting the same file still fires `change`.
            input.set_value("");
            input.click();
            Ok(())
        }

        fn take_import(&self) -> Option<ImportedFile> {
            IMPORTED.with(|c| c.borrow_mut().take())
        }

        // Format-typed interchange: the same hidden input, refiltered to the
        // requested extensions. The onchange handler stashes the file's real name
        // (its extension survives `model_display_name`, since that only strips
        // `.nbrep`), so the panel routes STEP imports by extension.
        fn begin_import_filtered(&self, filter: (&str, &[&str])) -> Result<(), String> {
            let accept = filter
                .1
                .iter()
                .map(|ext| format!(".{ext}"))
                .collect::<Vec<_>>()
                .join(",");
            let input = ensure_input(&accept).ok_or("file input unavailable")?;
            input.set_value("");
            input.click();
            Ok(())
        }

        /// Offer `contents` as a download under EXACTLY `file_name` (extension
        /// kept) — the foreign-format sibling of [`Self::export_file`]. The
        /// media type is the file's own ([`super::media_type`]), so a sheet
        /// leaves as `image/svg+xml` and not as anonymous bytes.
        fn export_file_named(&self, file_name: &str, contents: &str) -> Result<(), String> {
            download(file_name, super::media_type(file_name), contents)
        }

        /// The same, for bytes: `model/gltf-binary` for a GLB, `application/pdf`
        /// for a drawing sheet. It used to be glTF whatever the file was, which
        /// is why this lane could only ever carry one format.
        fn export_file_named_bytes(&self, file_name: &str, contents: &[u8]) -> Result<(), String> {
            download_bytes(file_name, super::media_type(file_name), contents)
        }
    }
}



