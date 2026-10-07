//! Backup, restore and export.
//!
//! # A backup
//!
//! One `.tar.gz` holding, under a `manifest.json` that comes first:
//!
//! - `plm.sqlite` — a snapshot of the metadata and the audit log, taken with
//!   SQLite's `VACUUM INTO`, which reads one consistent transaction while the
//!   server keeps writing;
//! - `models/<part>/<revision>.json` — the document of every revision the snapshot records a hash for;
//! - `blobs/…` — every attachment the snapshot references, and every version
//!   of every workspace file;
//! - `scripts/…` — the administrator's scripts directory (without `.git`).
//!
//! The manifest lists every file with its size and SHA-256, and the snapshot's
//! row counts.
//!
//! # Consistent while the server runs
//!
//! The snapshot is one transaction, but documents and blobs are files that a
//! running server keeps changing: a draft's document is rewritten, a blob is
//! removed with its last reference. So a backup is taken OPTIMISTICALLY and
//! then proven: every document copied must hash to what the snapshot recorded,
//! and every blob to its name. A mismatch or a missing file means a write
//! landed in between, and the whole backup is taken again (up to
//! [`ATTEMPTS`] times). A released revision's document never changes and a
//! blob is never rewritten, so only drafts written during the copy can cause a
//! retry. This needs no lock on the server, which is why `brep-plm backup`
//! works from another process while `serve` owns the directory.
//!
//! A draft saved faster than a backup can be taken (an autosave loop) would
//! make that retry forever, so after [`ADJUST_AFTER`] tries a draft's
//! document is taken as copied and the snapshot's record of its hash and size
//! set to match: a draft in work at the snapshot, with the document it had a
//! moment later — a state the store passes through anyway between a CAD save
//! and its next. The manifest lists each such file under `adjusted`.
//!
//! The server's own backups (the Backups page, `--backup-dir`) take the
//! snapshot and the drafts' documents under the store's read lock instead:
//! writes wait those milliseconds, and nothing is ever retried or adjusted.
//!
//! # A restore
//!
//! Into an empty (or absent) directory only. Every entry's path is checked
//! (no absolute paths, no `..`), every file must be in the manifest with the
//! size and hash it lists, and every file the manifest lists must be present.
//! Anything wrong removes what was written and refuses.
//!
//! # An export
//!
//! Metadata only, for moving to another system: see [`export_json`] and the
//! operator guide's "Export format" (GUIDE.md).

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::io::{self, Read, Write};
use std::path::{Component, Path, PathBuf};

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};

use crate::db::State;
use crate::model::{Part, Revision};

/// The manifest's `format` field.
pub const FORMAT: &str = "brep-plm-backup";
/// Bumped when the archive's layout changes.
pub const VERSION: u32 = 1;
/// How many times a backup is retaken when a write lands during the copy.
pub const ATTEMPTS: usize = 6;
/// After this many tries a draft's moving document is taken as copied
/// ([`Manifest::adjusted`]) instead of retrying again.
pub const ADJUST_AFTER: usize = 3;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FileEntry {
    /// Relative to the data directory, `/`-separated.
    pub path: String,
    pub size: u64,
    pub sha256: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Manifest {
    pub format: String,
    pub version: u32,
    /// Unix seconds.
    pub created_at: u64,
    /// The SQLite schema version of the snapshot.
    pub schema_version: i64,
    /// Row counts in the snapshot, by table.
    pub counts: BTreeMap<String, i64>,
    /// The snapshot's change sequence number.
    pub seq: u64,
    /// How many tries it took to get a consistent copy.
    pub attempts: usize,
    /// Documents of revisions in work that kept changing while a backup was
    /// taken from outside the server: each was taken as copied, and the
    /// snapshot's hash and size for it set to match (see the module header).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub adjusted: Vec<String>,
    pub files: Vec<FileEntry>,
}

impl Manifest {
    pub fn total_size(&self) -> u64 {
        self.files.iter().map(|f| f.size).sum()
    }

    /// What `backup` and `restore` print when the backup took a draft's
    /// document as copied after it kept changing ([`Manifest::adjusted`]).
    /// Each such draft holds the document it had a moment AFTER the snapshot,
    /// and its recorded hash was set to match, so it reads as consistent; the
    /// notice is how an operator learns which drafts those are. `None` when
    /// nothing was adjusted, which is always the case for the server's own
    /// backups.
    pub fn adjusted_notice(&self) -> Option<String> {
        if self.adjusted.is_empty() {
            return None;
        }
        let mut notice = format!(
            "{} draft document(s) kept changing during the backup and were taken as copied, \
             a moment after the snapshot (their recorded hashes were set to match):",
            self.adjusted.len()
        );
        for path in &self.adjusted {
            notice.push_str("\n  ");
            notice.push_str(path);
        }
        Some(notice)
    }
}


fn now() -> u64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

fn other(message: impl ToString) -> io::Error {
    io::Error::other(message.to_string())
}

/// The lower-case hex SHA-256 of a file, read in pieces.
pub fn hash_file(path: &Path) -> io::Result<(u64, String)> {
    let mut file = fs::File::open(path)?;
    let mut hasher = Sha256::new();
    let mut buffer = vec![0u8; 256 * 1024];
    let mut size = 0u64;
    loop {
        let n = file.read(&mut buffer)?;
        if n == 0 {
            break;
        }
        size += n as u64;
        hasher.update(&buffer[..n]);
    }
    Ok((size, format!("{:x}", hasher.finalize())))
}

/// What a snapshot says must be in the backup besides itself.
struct Wanted {
    /// Document path (relative) → the hash the snapshot recorded, for
    /// revisions in work (their documents still change).
    drafts: BTreeMap<String, String>,
    /// The same for every other revision: frozen, their documents never change.
    frozen: BTreeMap<String, String>,
    blobs: BTreeSet<String>,
    seq: u64,
    counts: BTreeMap<String, i64>,
    schema_version: i64,
}

/// Read what the snapshot references, through a read-only connection so the
/// snapshot file is archived exactly as `VACUUM INTO` wrote it.
fn wanted(snapshot: &Path) -> io::Result<Wanted> {
    use rusqlite::{Connection, OpenFlags};
    let conn = Connection::open_with_flags(snapshot, OpenFlags::SQLITE_OPEN_READ_ONLY).map_err(other_sql)?;
    let mut drafts = BTreeMap::new();
    let mut frozen = BTreeMap::new();
    let mut blobs = BTreeSet::new();
    let mut take_attachments = |body: &Value| {
        if let Some(list) = body.get("attachments").and_then(Value::as_array) {
            for a in list {
                if let Some(sha) = a.get("sha256").and_then(Value::as_str) {
                    blobs.insert(sha.to_string());
                }
            }
        }
        // A revision's thumbnail (P9) is a blob like an attachment.
        if let Some(sha) = body.pointer("/thumbnail/sha256").and_then(Value::as_str) {
            blobs.insert(sha.to_string());
        }
    };
    {
        let mut stmt = conn.prepare("SELECT body FROM parts").map_err(other_sql)?;
        let rows = stmt.query_map([], |r| r.get::<_, String>(0)).map_err(other_sql)?;
        for body in rows {
            let body: Value = serde_json::from_str(&body.map_err(other_sql)?).map_err(other)?;
            take_attachments(&body);
        }
    }
    {
        let mut stmt = conn.prepare("SELECT part_id, id, body FROM revisions").map_err(other_sql)?;
        let rows = stmt
            .query_map([], |r| Ok((r.get::<_, String>(0)?, r.get::<_, String>(1)?, r.get::<_, String>(2)?)))
            .map_err(other_sql)?;
        for row in rows {
            let (part, revision, body) = row.map_err(other_sql)?;
            let body: Value = serde_json::from_str(&body).map_err(other)?;
            take_attachments(&body);
            if let Some(hash) = body.get("content_hash").and_then(Value::as_str).filter(|h| !h.is_empty()) {
                let path = crate::identity::model_path(&part, &revision).to_string_lossy().replace('\\', "/");
                let in_work = matches!(body.get("lifecycle").and_then(Value::as_str), Some("draft" | "inreview"));
                if in_work { drafts.insert(path, hash.to_string()) } else { frozen.insert(path, hash.to_string()) };
            }
        }
    }
    // Every version of every workspace file. A snapshot from a server older
    // than workspaces has no such table.
    let has_workspace: bool = conn
        .query_row("SELECT count(*) FROM sqlite_master WHERE type = 'table' AND name = 'workspace'", [], |r| r.get::<_, i64>(0))
        .map_err(other_sql)?
        > 0;
    if has_workspace {
        let mut stmt = conn.prepare("SELECT body FROM workspace").map_err(other_sql)?;
        let rows = stmt.query_map([], |r| r.get::<_, String>(0)).map_err(other_sql)?;
        for body in rows {
            let body: Value = serde_json::from_str(&body.map_err(other_sql)?).map_err(other)?;
            for v in body.get("versions").and_then(Value::as_array).into_iter().flatten() {
                if let Some(sha) = v.get("sha256").and_then(Value::as_str) {
                    blobs.insert(sha.to_string());
                }
            }
        }
    }
    let mut counts = BTreeMap::new();
    let tables: Vec<String> = {
        let mut stmt = conn
            .prepare("SELECT name FROM sqlite_master WHERE type = 'table' AND name NOT LIKE 'sqlite_%' AND name NOT LIKE '%search%' ORDER BY name")
            .map_err(other_sql)?;
        let names = stmt.query_map([], |r| r.get::<_, String>(0)).map_err(other_sql)?;
        names.collect::<Result<_, _>>().map_err(other_sql)?
    };
    for table in tables {
        if !table.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_') {
            continue;
        }
        let n: i64 = conn.query_row(&format!("SELECT count(*) FROM {table}"), [], |r| r.get(0)).unwrap_or(-1);
        counts.insert(table, n);
    }
    let seq = conn
        .query_row("SELECT value FROM meta WHERE key = 'seq'", [], |r| r.get::<_, String>(0))
        .ok()
        .and_then(|v| v.trim_matches('"').parse::<u64>().ok())
        .unwrap_or(0);
    let schema_version: i64 = conn.query_row("PRAGMA user_version", [], |r| r.get(0)).map_err(other_sql)?;
    Ok(Wanted { drafts, frozen, blobs, seq, counts, schema_version })
}

fn other_sql(error: rusqlite::Error) -> io::Error {
    other(error.to_string())
}

/// Every file under `dir`, relative, `/`-separated, skipping `.git`.
fn walk(dir: &Path) -> io::Result<Vec<String>> {
    let mut out = Vec::new();
    if !dir.exists() {
        return Ok(out);
    }
    let mut stack = vec![PathBuf::new()];
    while let Some(relative) = stack.pop() {
        for entry in fs::read_dir(dir.join(&relative))? {
            let entry = entry?;
            let name = entry.file_name();
            if name == ".git" {
                continue;
            }
            let path = relative.join(&name);
            let kind = entry.file_type()?;
            if kind.is_dir() {
                stack.push(path);
            } else if kind.is_file() {
                out.push(path.to_string_lossy().replace('\\', "/"));
            }
        }
    }
    out.sort();
    Ok(out)
}

/// A staged copy: the snapshot and the files, in a scratch directory.
struct Stage {
    dir: PathBuf,
}

impl Drop for Stage {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.dir);
    }
}

/// Copy `from` to `to` and return its size and hash; `None` when `from` is gone.
fn copy_hashed(from: &Path, to: &Path) -> io::Result<Option<(u64, String)>> {
    fs::create_dir_all(to.parent().expect("a staged file has a directory"))?;
    let mut source = match fs::File::open(from) {
        Ok(file) => file,
        Err(error) if error.kind() == io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(error),
    };
    let mut target = fs::File::create(to)?;
    let mut hasher = Sha256::new();
    let mut buffer = vec![0u8; 256 * 1024];
    let mut size = 0u64;
    loop {
        let n = source.read(&mut buffer)?;
        if n == 0 {
            break;
        }
        size += n as u64;
        hasher.update(&buffer[..n]);
        target.write_all(&buffer[..n])?;
    }
    Ok(Some((size, format!("{:x}", hasher.finalize()))))
}

/// One attempt: snapshot, copy, prove. `Retry` when a write landed.
pub enum Attempt {
    Done(Manifest),
    Retry(String),
}

/// Run `f` while writes to the store are held off, when the caller can hold
/// them: the server passes its store's read lock, a separate process cannot
/// and passes [`unheld`].
pub type Hold<'a> = &'a dyn Fn(&mut dyn FnMut() -> io::Result<Attempt>) -> io::Result<Attempt>;

/// For a caller that cannot hold writes off: just run it.
pub fn unheld(f: &mut dyn FnMut() -> io::Result<Attempt>) -> io::Result<Attempt> {
    f()
}

fn attempt(data: &Path, scripts: &Path, stage: &Path, hold: Hold<'_>, adjust: bool) -> io::Result<Attempt> {
    let _ = fs::remove_dir_all(stage);
    fs::create_dir_all(stage)?;
    let snapshot = stage.join(crate::db::SQLITE_FILE);
    let mut files = Vec::new();
    let mut found: Option<Wanted> = None;
    let mut adjusted: Vec<String> = Vec::new();
    // The part that must agree with itself: the snapshot, and the documents
    // that can still change. Back to back, and under the store's lock when
    // the caller has one, so the window a write can fall into is a few files.
    let first = hold(&mut || {
        use rusqlite::{Connection, OpenFlags};
        let source = data.join(crate::db::SQLITE_FILE);
        if !source.exists() {
            return Err(other(format!("{} has no {}", data.display(), crate::db::SQLITE_FILE)));
        }
        let conn = Connection::open_with_flags(&source, OpenFlags::SQLITE_OPEN_READ_ONLY).map_err(other_sql)?;
        conn.busy_timeout(std::time::Duration::from_secs(30)).map_err(other_sql)?;
        conn.execute("VACUUM INTO ?1", [snapshot.to_string_lossy().as_ref()]).map_err(other_sql)?;
        drop(conn);
        let wanted = wanted(&snapshot)?;
        let mut moved: Vec<(String, Option<(u64, String)>)> = Vec::new();
        for (path, expected) in &wanted.drafts {
            match copy_hashed(&data.join(path), &stage.join(path))? {
                None if adjust => moved.push((path.clone(), None)),
                None => return Ok(Attempt::Retry(format!("{path} is gone"))),
                Some((size, got)) if &got != expected => {
                    if !adjust {
                        return Ok(Attempt::Retry(format!("{path} changed during the copy")));
                    }
                    moved.push((path.clone(), Some((size, got.clone()))));
                    files.push(FileEntry { path: path.clone(), size, sha256: got });
                }
                Some((size, sha256)) => files.push(FileEntry { path: path.clone(), size, sha256 }),
            }
        }
        if !moved.is_empty() {
            adjust_snapshot(&snapshot, &moved)?;
            adjusted = moved.into_iter().map(|(p, _)| p).collect();
        }
        found = Some(wanted);
        Ok(Attempt::Done(Manifest {
            format: String::new(),
            version: 0,
            created_at: 0,
            schema_version: 0,
            counts: BTreeMap::new(),
            seq: 0,
            attempts: 0,
            adjusted: Vec::new(),
            files: Vec::new(),
        }))
    })?;
    if let Attempt::Retry(why) = first {
        return Ok(Attempt::Retry(why));
    }
    let wanted = found.expect("the first step read the snapshot");
    let (size, sha256) = hash_file(&snapshot)?;
    files.insert(0, FileEntry { path: crate::db::SQLITE_FILE.to_string(), size, sha256 });

    // Frozen documents and blobs never change once written, so they can be
    // copied with writes going on. A blob can only DISAPPEAR (its last
    // reference removed after the snapshot): then the backup is retaken.
    for (path, expected) in &wanted.frozen {
        match copy_hashed(&data.join(path), &stage.join(path))? {
            None => return Ok(Attempt::Retry(format!("{path} is gone"))),
            Some((_, got)) if &got != expected => {
                return Err(other(format!("{path} does not match the hash its released revision records")))
            }
            Some((size, sha256)) => files.push(FileEntry { path: path.clone(), size, sha256 }),
        }
    }
    for sha in &wanted.blobs {
        if !crate::attach::is_sha256(sha) {
            return Err(other(format!("the snapshot names a blob '{sha}' that is not a SHA-256")));
        }
        let path = format!("blobs/{}/{sha}", &sha[..2]);
        match copy_hashed(&data.join(&path), &stage.join(&path))? {
            None => return Ok(Attempt::Retry(format!("{path} is gone"))),
            Some((_, got)) if &got != sha => return Err(other(format!("{path} does not hash to its name"))),
            Some((size, sha256)) => files.push(FileEntry { path, size, sha256 }),
        }
    }
    for relative in walk(scripts)? {
        let path = format!("scripts/{relative}");
        if let Some((size, sha256)) = copy_hashed(&scripts.join(&relative), &stage.join(&path))? {
            files.push(FileEntry { path, size, sha256 });
        }
    }
    Ok(Attempt::Done(Manifest {
        format: FORMAT.into(),
        version: VERSION,
        created_at: now(),
        schema_version: wanted.schema_version,
        counts: wanted.counts,
        seq: wanted.seq,
        attempts: 0,
        adjusted,
        files,
    }))
}

/// Make the snapshot agree with draft documents copied after it: each
/// revision's recorded hash and size become those of the copied file, or
/// empty when the file had gone (a draft deleted meanwhile).
fn adjust_snapshot(snapshot: &Path, moved: &[(String, Option<(u64, String)>)]) -> io::Result<()> {
    let conn = rusqlite::Connection::open(snapshot).map_err(other_sql)?;
    for (path, now) in moved {
        let segments: Vec<_> = path.split('/').collect();
        let (part, revision) = match segments.as_slice() {
            ["models", part, revision] => (
                crate::identity::unsegment(part).ok_or_else(|| other("invalid model part path"))?,
                crate::identity::unsegment(revision.strip_suffix(".json").ok_or_else(|| other("invalid model revision path"))?).ok_or_else(|| other("invalid model revision path"))?,
            ),
            _ => return Err(other("invalid model path")),
        };
        let body: String = conn
            .query_row("SELECT body FROM revisions WHERE part_id = ?1 AND id = ?2", [&part, &revision], |r| r.get(0))
            .map_err(other_sql)?;
        let mut body: Value = serde_json::from_str(&body).map_err(other)?;
        let (size, hash) = now.clone().unwrap_or((0, String::new()));
        body["content_hash"] = json!(hash);
        body["size"] = json!(size);
        conn.execute("UPDATE revisions SET body = ?1 WHERE part_id = ?2 AND id = ?3", [body.to_string(), part, revision]).map_err(other_sql)?;
    }
    Ok(())
}

/// Take a backup of `data` (with its scripts directory) into `out`, a
/// `.tar.gz`, from outside the server: works while a server is running on
/// `data`, retrying when a draft's document is rewritten during the copy.
pub fn backup(data: &Path, scripts: Option<&Path>, out: &Path) -> io::Result<Manifest> {
    backup_held(data, scripts, out, &unheld)
}

/// [`backup`], with the snapshot and the draft documents copied under `hold`.
pub fn backup_held(data: &Path, scripts: Option<&Path>, out: &Path, hold: Hold<'_>) -> io::Result<Manifest> {
    let scripts = scripts.map(Path::to_path_buf).unwrap_or_else(|| data.join("scripts"));
    let parent = out.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = Stage { dir: parent.join(format!(".brep-backup-{}", crate::auth::new_id())) };
    let mut last = String::new();
    for tries in 1..=ATTEMPTS {
        // The last tries stop waiting for a quiet moment that is not coming.
        let adjust = tries > ADJUST_AFTER;
        match attempt(data, &scripts, &stage.dir, hold, adjust)? {
            Attempt::Retry(why) => {
                last = why;
                std::thread::sleep(std::time::Duration::from_millis(20 * tries as u64));
            }
            Attempt::Done(mut manifest) => {
                manifest.attempts = tries;
                write_archive(&stage.dir, &manifest, out)?;
                return Ok(manifest);
            }
        }
    }
    Err(other(format!(
        "could not take a consistent backup in {ATTEMPTS} tries — the store kept changing ({last}). \
         A backup taken by the server itself (the Backups page, or --backup-dir) holds writes off briefly and always succeeds"
    )))
}

fn write_archive(stage: &Path, manifest: &Manifest, out: &Path) -> io::Result<()> {
    let partial = out.with_extension("partial");
    {
        let file = fs::File::create(&partial)?;
        let gz = flate2::write::GzEncoder::new(io::BufWriter::new(file), flate2::Compression::default());
        let mut tar = tar::Builder::new(gz);
        let text = serde_json::to_vec_pretty(manifest).map_err(other)?;
        let mut header = tar::Header::new_gnu();
        header.set_size(text.len() as u64);
        header.set_mode(0o644);
        header.set_mtime(manifest.created_at);
        header.set_cksum();
        tar.append_data(&mut header, "manifest.json", text.as_slice())?;
        for entry in &manifest.files {
            let mut file = fs::File::open(stage.join(&entry.path))?;
            tar.append_file(&entry.path, &mut file)?;
        }
        let gz = tar.into_inner()?;
        let mut writer = gz.finish()?;
        writer.flush()?;
        writer.into_inner().map_err(|e| e.into_error())?.sync_all()?;
    }
    fs::rename(&partial, out)
}

/// A path from an archive, made safe to join to the target, or refused.
fn safe_relative(raw: &Path) -> io::Result<PathBuf> {
    let mut out = PathBuf::new();
    for part in raw.components() {
        match part {
            Component::Normal(p) => out.push(p),
            Component::CurDir => {}
            _ => return Err(other(format!("the archive names an unsafe path: {}", raw.display()))),
        }
    }
    if out.as_os_str().is_empty() {
        return Err(other("the archive names an empty path"));
    }
    Ok(out)
}

/// Read a backup's manifest without restoring it.
pub fn read_manifest(archive: &Path) -> io::Result<Manifest> {
    let file = fs::File::open(archive)?;
    let mut tar = tar::Archive::new(flate2::read::GzDecoder::new(io::BufReader::new(file)));
    let mut entries = tar.entries()?;
    let first = entries.next().ok_or_else(|| other("the archive is empty"))??;
    if first.path()?.as_ref() != Path::new("manifest.json") {
        return Err(other("the archive does not start with manifest.json — is it a brep-plm backup?"));
    }
    let manifest: Manifest = serde_json::from_reader(first).map_err(other)?;
    if manifest.format != FORMAT {
        return Err(other(format!("the archive's format is '{}', not '{FORMAT}'", manifest.format)));
    }
    if manifest.version > VERSION {
        return Err(other(format!("the archive is version {}, newer than this server's {VERSION}", manifest.version)));
    }
    Ok(manifest)
}

/// Restore `archive` into `data`, which must be empty or absent. Returns the
/// manifest. On any refusal, what was written is removed again.
pub fn restore(archive: &Path, data: &Path) -> io::Result<Manifest> {
    let existed = data.exists();
    if existed && fs::read_dir(data)?.next().is_some() {
        return Err(other(format!("{} is not empty — restore into a new directory", data.display())));
    }
    fs::create_dir_all(data)?;
    match restore_into(archive, data) {
        Ok(manifest) => Ok(manifest),
        Err(error) => {
            // Leave the directory as it was: gone if we made it, empty if not.
            if existed {
                if let Ok(entries) = fs::read_dir(data) {
                    for entry in entries.flatten() {
                        let path = entry.path();
                        let _ = if path.is_dir() { fs::remove_dir_all(&path) } else { fs::remove_file(&path) };
                    }
                }
            } else {
                let _ = fs::remove_dir_all(data);
            }
            Err(error)
        }
    }
}

fn restore_into(archive: &Path, data: &Path) -> io::Result<Manifest> {
    let file = fs::File::open(archive)?;
    let mut tar = tar::Archive::new(flate2::read::GzDecoder::new(io::BufReader::new(file)));
    let mut entries = tar.entries()?;
    let first = entries.next().ok_or_else(|| other("the archive is empty"))??;
    if first.path()?.as_ref() != Path::new("manifest.json") {
        return Err(other("the archive does not start with manifest.json — is it a brep-plm backup?"));
    }
    let manifest: Manifest = serde_json::from_reader(first).map_err(other)?;
    if manifest.format != FORMAT || manifest.version > VERSION {
        return Err(other("the archive is not a backup this server can restore"));
    }
    let listed: BTreeMap<&str, &FileEntry> = manifest.files.iter().map(|f| (f.path.as_str(), f)).collect();
    let mut seen = BTreeSet::new();
    for entry in entries {
        let mut entry = entry?;
        if !entry.header().entry_type().is_file() {
            return Err(other("the archive holds something that is not a file"));
        }
        let raw = entry.path()?.into_owned();
        let relative = safe_relative(&raw)?;
        let key = relative.to_string_lossy().replace('\\', "/");
        let Some(expected) = listed.get(key.as_str()) else {
            return Err(other(format!("{key} is in the archive but not in its manifest")));
        };
        if !seen.insert(key.clone()) {
            return Err(other(format!("{key} is in the archive twice")));
        }
        let target = data.join(&relative);
        fs::create_dir_all(target.parent().expect("a file has a directory"))?;
        let mut out = fs::File::create(&target)?;
        let mut hasher = Sha256::new();
        let mut buffer = vec![0u8; 256 * 1024];
        let mut size = 0u64;
        loop {
            let n = entry.read(&mut buffer)?;
            if n == 0 {
                break;
            }
            size += n as u64;
            hasher.update(&buffer[..n]);
            out.write_all(&buffer[..n])?;
        }
        out.sync_all()?;
        let sha = format!("{:x}", hasher.finalize());
        if size != expected.size || sha != expected.sha256 {
            return Err(other(format!("{key} does not match its manifest (size or SHA-256) — the backup is damaged")));
        }
    }
    for file in &manifest.files {
        if !seen.contains(&file.path) {
            return Err(other(format!("{} is in the manifest but missing from the archive", file.path)));
        }
    }
    {
        use rusqlite::{Connection, OpenFlags};
        let conn = Connection::open_with_flags(data.join(crate::db::SQLITE_FILE), OpenFlags::SQLITE_OPEN_READ_ONLY)
            .map_err(other_sql)?;
        let check: String = conn.query_row("PRAGMA integrity_check", [], |r| r.get(0)).map_err(other_sql)?;
        if check != "ok" {
            return Err(other(format!("the restored database fails its integrity check: {check}")));
        }
    }
    Ok(manifest)
}

// ===========================================================================
// Export
// ===========================================================================

/// The export format's `format` field.
pub const EXPORT_FORMAT: &str = "brep-plm-export";
pub const EXPORT_VERSION: u32 = 1;

fn revision_json(state: &State, part: &Part, revision: &Revision) -> Value {
    let mut value = serde_json::to_value(revision).unwrap_or(Value::Null);
    if let Some(object) = value.as_object_mut() {
        // Who holds a lock is live state, not history.
        object.remove("lock");
        object.insert("document_key".into(), json!(revision.document_key(&part.id)));
        let uses: Vec<Value> = revision
            .uses
            .iter()
            .map(|u| {
                let child = state.part(&u.part);
                let child_revision = child.and_then(|c| if u.revision.is_empty() { None } else { c.revision(&u.revision) });
                let mut line = serde_json::to_value(u).unwrap_or(Value::Null);
                line["part_number"] = json!(child.map(|c| c.number.clone()).unwrap_or_default());
                line["revision_label"] = json!(child_revision.map(|r| r.label.clone()).unwrap_or_default());
                line
            })
            .collect();
        object.insert("uses".into(), Value::Array(uses));
    }
    value
}

/// Every piece of metadata a migration to another system needs, as one JSON
/// value. Secrets and live state are left out: password verifiers, sessions,
/// API tokens, sign-in throttles, locks. Documents and attachment bytes are
/// not included (they are in a backup); each revision names its
/// `document_key` and `content_hash`, and each attachment its `sha256`.
pub fn export_json(state: &State) -> Value {
    let users: Vec<Value> = state
        .users
        .iter()
        .map(|u| {
            json!({
                "id": u.id, "username": u.username, "display_name": u.display_name,
                "email": u.email, "groups": u.groups, "active": u.active, "created_at": u.created_at,
            })
        })
        .collect();
    let parts: Vec<Value> = state
        .parts
        .iter()
        .map(|part| {
            let mut value = serde_json::to_value(part).unwrap_or(Value::Null);
            value["category_path"] = json!(crate::catalog::path_name(&state.categories, &part.category));
            value["revisions"] = Value::Array(part.revisions.iter().map(|r| revision_json(state, part, r)).collect());
            value
        })
        .collect();
    json!({
        "format": EXPORT_FORMAT,
        "version": EXPORT_VERSION,
        "exported_at": now(),
        "seq": state.seq,
        "settings": state.settings,
        "part_types": state.part_types,
        "categories": state.categories,
        "manufacturers": state.manufacturers,
        "suppliers": state.suppliers,
        "users": users,
        "parts": parts,
        "change_orders": state.change_orders,
    })
}

fn cell(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_string()
    }
}

/// One row per part: its identity, catalog, current state and preferred MPN.
pub fn parts_csv(state: &State) -> String {
    let mut out = String::from(
        "Number,Name,Type,Class,Category,Tags,Description,Latest rev,Latest state,Current release,Revisions,MPN,Manufacturer,Attributes\n",
    );
    for part in state.parts.iter() {
        let latest = part.latest();
        let preferred = part.sourcing.iter().find(|m| m.preferred).or(part.sourcing.first());
        let maker = preferred
            .and_then(|m| state.manufacturers.iter().find(|c| c.id == m.manufacturer))
            .map(|c| c.name.clone())
            .unwrap_or_default();
        let attributes: Vec<String> = part.attributes.iter().map(|(k, v)| {
            let text = match v { Value::String(s) => s.clone(), other => other.to_string() };
            format!("{k}={text}")
        }).collect();
        let row = [
            part.number.clone(),
            part.name.clone(),
            part.part_type.clone(),
            part.document_class.as_str().to_string(),
            crate::catalog::path_name(&state.categories, &part.category),
            part.tags.join("; "),
            part.description.clone(),
            latest.map(|r| r.label.clone()).unwrap_or_default(),
            latest.map(|r| r.lifecycle.as_str().to_string()).unwrap_or_default(),
            part.current_release().map(|r| r.label.clone()).unwrap_or_default(),
            part.revisions.len().to_string(),
            preferred.map(|m| m.mpn.clone()).unwrap_or_default(),
            maker,
            attributes.join("; "),
        ];
        out.push_str(&row.iter().map(|c| cell(c)).collect::<Vec<_>>().join(","));
        out.push('\n');
    }
    out
}

/// One row per uses-list line in the store: the whole assembly structure, flat.
pub fn structure_csv(state: &State) -> String {
    let mut out = String::from(
        "Parent number,Parent rev,Parent state,Child number,Child rev,Floating,Quantity,Unit,Find,Reference,Notes\n",
    );
    for part in state.parts.iter() {
        for revision in &part.revisions {
            for line in &revision.uses {
                let child = state.part(&line.part);
                let child_revision = child.and_then(|c| crate::bom::resolve(c, line));
                let row = [
                    part.number.clone(),
                    revision.label.clone(),
                    revision.lifecycle.as_str().to_string(),
                    child.map(|c| c.number.clone()).unwrap_or_default(),
                    child_revision.map(|r| r.label.clone()).unwrap_or_default(),
                    if line.revision.is_empty() { "yes".into() } else { String::new() },
                    crate::bom::qty(line.quantity),
                    line.unit.clone(),
                    line.find_number.clone(),
                    line.reference.clone(),
                    line.notes.clone(),
                ];
                out.push_str(&row.iter().map(|c| cell(c)).collect::<Vec<_>>().join(","));
                out.push('\n');
            }
        }
    }
    out
}

// ===========================================================================
// The server's backups: saved, scheduled, and downloaded
// ===========================================================================

/// How many saved backups are kept when `--backup-keep` is not given.
pub const DEFAULT_KEEP: usize = 7;
/// Saved backups are named `brep-plm-<UTC time>.tar.gz`.
pub const PREFIX: &str = "brep-plm-";
pub const SUFFIX: &str = ".tar.gz";

/// One backup this process took (or failed to).
#[derive(Debug, Clone, Serialize, Default)]
pub struct LastBackup {
    pub at: u64,
    /// The saved file's name, or empty for a download.
    pub file: String,
    pub size: u64,
    pub files: usize,
    pub attempts: usize,
    pub seq: u64,
    pub ok: bool,
    pub error: String,
    /// `schedule`, `button`, `download`.
    pub trigger: String,
}

#[derive(Debug, Default)]
pub struct Status {
    last: std::sync::Mutex<Option<LastBackup>>,
    running: std::sync::atomic::AtomicBool,
}

impl Status {
    pub fn last(&self) -> Option<LastBackup> {
        self.last.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }

    pub fn running(&self) -> bool {
        self.running.load(std::sync::atomic::Ordering::SeqCst)
    }

    /// Claim the one backup slot; `None` when a backup is already running.
    fn begin(&self) -> Option<Running<'_>> {
        use std::sync::atomic::Ordering;
        self.running.compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst).ok()?;
        Some(Running(self))
    }

    fn record(&self, last: LastBackup) {
        *self.last.lock().unwrap_or_else(|e| e.into_inner()) = Some(last);
    }
}

struct Running<'a>(&'a Status);

impl Drop for Running<'_> {
    fn drop(&mut self) {
        self.0.running.store(false, std::sync::atomic::Ordering::SeqCst);
    }
}

/// `YYYYMMDD-HHMMSS` in UTC, from Unix seconds.
pub fn stamp(seconds: u64) -> String {
    let days = (seconds / 86_400) as i64;
    let rest = seconds % 86_400;
    // Days to civil date (Howard Hinnant's algorithm).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + if month <= 2 { 1 } else { 0 };
    format!("{year:04}{month:02}{day:02}-{:02}{:02}{:02}", rest / 3600, rest % 3600 / 60, rest % 60)
}

/// Whether `name` is a saved backup's name — the only names ever joined to
/// the backup directory from a request.
pub fn is_backup_name(name: &str) -> bool {
    name.starts_with(PREFIX)
        && name.ends_with(SUFFIX)
        && name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'.' || b == b'_')
}

#[derive(Debug, Clone, Serialize)]
pub struct SavedBackup {
    pub name: String,
    pub size: u64,
    pub modified: u64,
}

/// The saved backups in `dir`, newest first.
pub fn saved(dir: &Path) -> Vec<SavedBackup> {
    let mut out = Vec::new();
    if let Ok(entries) = fs::read_dir(dir) {
        for entry in entries.flatten() {
            let name = entry.file_name().to_string_lossy().to_string();
            if !is_backup_name(&name) {
                continue;
            }
            let Ok(meta) = entry.metadata() else { continue };
            let modified = meta
                .modified()
                .ok()
                .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
                .map(|d| d.as_secs())
                .unwrap_or(0);
            out.push(SavedBackup { name, size: meta.len(), modified });
        }
    }
    // The name carries the time (and a zero-padded counter for two in one
    // second), so name order without the suffix is time order.
    let key = |n: &str| n.trim_end_matches(SUFFIX).to_string();
    out.sort_by(|a, b| key(&b.name).cmp(&key(&a.name)));
    out
}

impl crate::db::Db {
    /// Take a backup into `--backup-dir`, then keep only the newest
    /// `--backup-keep`. Refused while another backup runs.
    pub fn backup_now(&self, trigger: &str) -> Result<LastBackup, crate::Error> {
        let config = self.security().config.clone();
        let dir = config
            .backup_dir
            .clone()
            .ok_or_else(|| crate::Error::conflict("no backup directory is set (--backup-dir) — download a backup instead"))?;
        let _running = self
            .backups()
            .begin()
            .ok_or_else(|| crate::Error::conflict("a backup is already running"))?;
        let at = now();
        let mut name = format!("{PREFIX}{}{SUFFIX}", stamp(at));
        let mut n = 1;
        while dir.join(&name).exists() {
            n += 1;
            name = format!("{PREFIX}{}-{n:02}{SUFFIX}", stamp(at));
        }
        let result = self.backup_into(&dir.join(&name));
        let last = match &result {
            Ok(manifest) => LastBackup {
                at,
                file: name.clone(),
                size: fs::metadata(dir.join(&name)).map(|m| m.len()).unwrap_or(0),
                files: manifest.files.len(),
                attempts: manifest.attempts,
                seq: manifest.seq,
                ok: true,
                error: String::new(),
                trigger: trigger.into(),
            },
            Err(error) => LastBackup { at, ok: false, error: error.to_string(), trigger: trigger.into(), ..Default::default() },
        };
        self.backups().record(last.clone());
        self.record(
            "backup",
            crate::model::EntityRef { kind: "backup".into(), id: name.clone(), label: name.clone(), part_id: String::new() },
            if last.ok { format!("{trigger}: {} files, {} bytes", last.files, last.size) } else { format!("{trigger}: FAILED {}", last.error) },
        );
        if let Err(error) = result {
            return Err(crate::Error::internal(error));
        }
        let keep = if config.backup_keep == 0 { DEFAULT_KEEP } else { config.backup_keep };
        for old in saved(&dir).into_iter().skip(keep) {
            let _ = fs::remove_file(dir.join(&old.name));
        }
        Ok(last)
    }

    /// A backup of this store into `out`, with the snapshot and the draft
    /// documents taken under the store's read lock: writes wait for those
    /// few milliseconds, and the backup never has to be retaken for them.
    pub fn backup_into(&self, out: &Path) -> io::Result<Manifest> {
        let hold = |f: &mut dyn FnMut() -> io::Result<Attempt>| self.read(|_| f());
        backup_held(self.root(), Some(self.scripts().dir()), out, &hold)
    }

    /// Take a backup into a temporary file for a download. The caller streams
    /// it and removes it.
    pub fn backup_to_temp(&self) -> Result<(PathBuf, Manifest), crate::Error> {
        let _running = self
            .backups()
            .begin()
            .ok_or_else(|| crate::Error::conflict("a backup is already running"))?;
        let at = now();
        let path = std::env::temp_dir().join(format!("{PREFIX}{}-{}{SUFFIX}", stamp(at), crate::auth::new_id()));
        let result = self.backup_into(&path);
        let last = match &result {
            Ok(manifest) => LastBackup {
                at,
                size: fs::metadata(&path).map(|m| m.len()).unwrap_or(0),
                files: manifest.files.len(),
                attempts: manifest.attempts,
                seq: manifest.seq,
                ok: true,
                trigger: "download".into(),
                ..Default::default()
            },
            Err(error) => LastBackup { at, ok: false, error: error.to_string(), trigger: "download".into(), ..Default::default() },
        };
        self.backups().record(last.clone());
        self.record(
            "backup",
            crate::model::EntityRef { kind: "backup".into(), id: "download".into(), label: "download".into(), part_id: String::new() },
            if last.ok { format!("download: {} files, {} bytes", last.files, last.size) } else { format!("download: FAILED {}", last.error) },
        );
        let manifest = result.map_err(crate::Error::internal)?;
        Ok((path, manifest))
    }
}

/// Load a data directory's metadata from a snapshot, without opening the
/// store — what `brep-plm export` uses, so it works while a server runs.
pub fn load_snapshot(data: &Path) -> io::Result<State> {
    let dir = std::env::temp_dir().join(format!(".brep-export-{}", crate::auth::new_id()));
    let stage = Stage { dir };
    fs::create_dir_all(&stage.dir)?;
    let snapshot = stage.dir.join(crate::db::SQLITE_FILE);
    {
        use rusqlite::{Connection, OpenFlags};
        let source = data.join(crate::db::SQLITE_FILE);
        if !source.exists() {
            return Err(other(format!("{} has no {}", data.display(), crate::db::SQLITE_FILE)));
        }
        let conn = Connection::open_with_flags(&source, OpenFlags::SQLITE_OPEN_READ_ONLY).map_err(other_sql)?;
        conn.busy_timeout(std::time::Duration::from_secs(30)).map_err(other_sql)?;
        conn.execute("VACUUM INTO ?1", [snapshot.to_string_lossy().as_ref()]).map_err(other_sql)?;
    }
    let sql = crate::sql::Sql::open(&snapshot).map_err(other_sql)?;
    let state = sql.load().map_err(other_sql)?.unwrap_or_default();
    drop(sql);
    Ok(state)
}
