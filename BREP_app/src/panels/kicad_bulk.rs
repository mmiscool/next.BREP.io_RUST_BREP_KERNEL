//! Importing EVERY part KiCad has, over as many sittings as it takes.
//!
//! [`super::kicad_remote`] fetches one file. [`super::kicad_library`] follows one
//! symbol through the chain and saves one part. This module is the stage on top
//! of both: it walks every symbol library at a tag, and every symbol in each
//! one, through the SAME `part_from_symbol` → `import_part` chain, writing one
//! document per symbol.
//!
//! KiCad 9.0.9.1 is ~165 symbol libraries holding thousands of symbols whose 3D
//! models are gigabytes, and a whole part measured 1.16 s of download and parse.
//! So a full sweep is hours, and the three things that makes necessary are the
//! whole design of this file: it must show progress, it must stop when asked,
//! and it must be able to pick up where it left off WITHOUT paying again for
//! what it already did.
//!
//! # The ledger
//!
//! Resume is answered by a file, not by looking at what is on disk: one
//! append-only JSON-lines [`Ledger`] per tag, beside the fetch cache
//! (`<cache root>/bulk-import-<tag>.jsonl`). One line per part, keyed by the
//! symbol's `library_id` — `Timer:NE555D`, library nickname included, because
//! `NE555D` alone is not unique across 165 libraries.
//!
//! Append-only is not laziness, it is the cancellation story. A file rewritten
//! whole after every part is O(n) per part and is precisely the file that a
//! process dying mid-write truncates into nonsense. An append writes one line
//! and flushes; the worst a kill can leave is a torn LAST line, which
//! [`parse_ledger`] drops. So every key in the ledger is a part that finished,
//! and a part that did not finish is simply absent.
//!
//! **The write order is the invariant**: the document is written first, its
//! ledger line second ([`sweep`]). Cancelled between the two, the part is
//! redone next run and its document overwritten, which costs a repeat and
//! breaks nothing. The other order would record a part as done whose document
//! does not exist.
//!
//! # What counts as done
//!
//! The ledger's own record, and nothing else. Not the presence of the document:
//! two libraries hold symbols that would land on one document name, so a
//! document proves that SOMETHING wrote it, not that this part is done — and no
//! document can record the parts that legitimately have nothing to import. Not
//! a content hash either: a git tag is immutable, so a path's bytes cannot
//! change underneath a resume, which is what a hash would be checking for. The
//! tag is therefore doing the work a hash otherwise would, and that is why it
//! is in the ledger's FILE NAME: a sweep at another tag is another corpus and
//! starts its own ledger rather than inheriting conclusions about this one.
//!
//! Resume reads that one file once — thousands of short lines, no network, no
//! per-part `stat` — and skips every key it settled. The fetch cache then skips
//! the network a second time independently, which is what makes a part that was
//! interrupted midway through its conversion re-parse without re-downloading.
//!
//! # Failure is the normal case
//!
//! At this scale most parts are NOT clean. A power symbol names no footprint; a
//! footprint names no 3D model; a model exists only as `.wrl`, which BREP does
//! not read. None of those is an error — they are settled outcomes, recorded
//! with a [`Grade`] and never retried, and the part is still written with
//! whatever it does have.
//!
//! What IS retried is a request that went unanswered: a 429 that outlived its
//! backoff, a 5xx, a dropped connection, a library whose download failed. Those
//! are recorded [`Outcome::Failed`] and a later run redoes exactly them, with no
//! mode to select — resume retries failures by definition. This only works
//! because [`RemoteFiles::take_problems`](super::kicad_remote::RemoteFiles::take_problems)
//! distinguishes a 404 from an unanswered request: the chain itself cannot, and
//! without the distinction a rate limit would be written into the ledger as
//! "KiCad has no model for this part" and never looked at again.
//!
//! A forced sweep ignores the ledger and redoes everything, still serving files
//! from the fetch cache — at a fixed tag those files cannot be stale.

#![cfg(not(target_arch = "wasm32"))]

use super::kicad_import::{
    import_part, parse_step, part_from_symbol, read_symbol_library, FootprintChoice, Imported,
    KicadPart, ModelFile,
};
use super::kicad_remote::{Fetcher, RemoteFiles, Repo};
use crate::store::ModelStore;
use brep_render::engine_state::EngineState;
use serde_json::{json, Value};
use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// How many `.step` parses one library keeps, so the symbols that share a
/// footprint pay the 529 ms parse once. Cleared between libraries, which bounds
/// the held geometry to one library's worth and keeps the hit rate high: it is
/// a library's own symbols that share its packages.
const MODEL_MEMO: usize = 32;

/// How many notes one ledger line keeps, and how long each may be. The notes are
/// there to explain a grade, not to reproduce the part.
const LEDGER_NOTES: usize = 3;
const LEDGER_NOTE_CHARS: usize = 200;

/// How many failures the progress snapshot carries for the window to show.
const SHOWN_NOTES: usize = 8;

// ============================================================================
// What one part's outcome was
// ============================================================================

/// How complete a part that finished is. Every one of these is a SETTLED
/// answer at a fixed tag — a resume skips them all — and the part was written
/// either way.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Grade {
    /// Symbol, footprint and a 3D model.
    Full,
    /// Symbol and footprint; the footprint named no model, or named one the
    /// repository does not have (a `.wrl` with no `.step` beside it).
    NoModel,
    /// Symbol and footprint; the `.step` was there and did not parse.
    ModelUnreadable,
    /// The symbol alone: it names no footprint, or names one that is not there.
    NoFootprint,
}

impl Grade {
    /// The word the ledger stores.
    pub fn as_str(self) -> &'static str {
        match self {
            Grade::Full => "full",
            Grade::NoModel => "no-model",
            Grade::ModelUnreadable => "model-unreadable",
            Grade::NoFootprint => "no-footprint",
        }
    }

    /// The word back, for a ledger a later version reads.
    pub fn from_str(word: &str) -> Option<Self> {
        [Grade::Full, Grade::NoModel, Grade::ModelUnreadable, Grade::NoFootprint]
            .into_iter()
            .find(|grade| grade.as_str() == word)
    }
}

/// What a sweep concluded about one part.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// It was imported and written. Never done again unless forced.
    Done(Grade),
    /// Something did not answer. Done again by the next resume.
    Failed(String),
}

impl Outcome {
    pub fn settled(&self) -> bool {
        matches!(self, Outcome::Done(_))
    }
}

/// One part's line in the ledger.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Record {
    /// The symbol's `library_id`, `nickname:symbol`.
    pub key: String,
    pub outcome: Outcome,
    /// The document that was written, empty when nothing was.
    pub document: String,
    /// Why the grade is what it is, trimmed.
    pub notes: Vec<String>,
    /// What this part cost: files downloaded and their bytes. Zero when the
    /// fetch cache answered every read.
    pub files: usize,
    pub bytes: u64,
    /// Unix seconds, so a user can see how far a run got and when.
    pub at: u64,
}

impl Record {
    /// The line this record is stored as, newline included.
    pub fn to_line(&self) -> String {
        let mut value = json!({
            "key": self.key,
            "document": self.document,
            "files": self.files,
            "bytes": self.bytes,
            "at": self.at,
        });
        match &self.outcome {
            Outcome::Done(grade) => {
                value["outcome"] = json!("done");
                value["grade"] = json!(grade.as_str());
            }
            Outcome::Failed(reason) => {
                value["outcome"] = json!("failed");
                value["reason"] = json!(trim_note(reason));
            }
        }
        if !self.notes.is_empty() {
            value["notes"] = json!(self.notes);
        }
        format!("{value}\n")
    }

    /// One line back, or `None` when it is not a record this version reads —
    /// which is also how a torn last line is dropped.
    pub fn from_line(line: &str) -> Option<Self> {
        let value: Value = serde_json::from_str(line.trim()).ok()?;
        let key = value.get("key")?.as_str()?.to_owned();
        if key.is_empty() {
            return None;
        }
        let outcome = match value.get("outcome").and_then(Value::as_str)? {
            "done" => Outcome::Done(Grade::from_str(value.get("grade").and_then(Value::as_str)?)?),
            "failed" => Outcome::Failed(
                value.get("reason").and_then(Value::as_str).unwrap_or("it did not say").to_owned(),
            ),
            _ => return None,
        };
        Some(Record {
            key,
            outcome,
            document: value.get("document").and_then(Value::as_str).unwrap_or_default().to_owned(),
            notes: value
                .get("notes")
                .and_then(Value::as_array)
                .map(|list| list.iter().filter_map(Value::as_str).map(str::to_owned).collect())
                .unwrap_or_default(),
            files: value.get("files").and_then(Value::as_u64).unwrap_or(0) as usize,
            bytes: value.get("bytes").and_then(Value::as_u64).unwrap_or(0),
            at: value.get("at").and_then(Value::as_u64).unwrap_or(0),
        })
    }
}

/// One note, short enough to belong in a line that is read thousands at a time.
fn trim_note(note: &str) -> String {
    let flat = note.replace(['\n', '\r'], " ");
    match flat.char_indices().nth(LEDGER_NOTE_CHARS) {
        Some((at, _)) => format!("{}…", &flat[..at]),
        None => flat,
    }
}

/// Unix seconds, or 0 on a clock before the epoch.
fn now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|since| since.as_secs())
        .unwrap_or(0)
}

// ============================================================================
// The ledger
// ============================================================================

/// The records of a whole ledger file, latest line per key winning, and how many
/// lines were NOT records.
///
/// A dropped line is the expected shape of a kill mid-append: the last line is
/// half a JSON object. It is dropped rather than refused, because refusing the
/// whole ledger over one torn line would throw away every part the run did
/// finish — the exact opposite of the point.
pub fn parse_ledger(text: &str) -> (BTreeMap<String, Record>, usize) {
    let mut records = BTreeMap::new();
    let mut dropped = 0;
    for line in text.lines() {
        if line.trim().is_empty() {
            continue;
        }
        match Record::from_line(line) {
            Some(record) => {
                records.insert(record.key.clone(), record);
            }
            None => dropped += 1,
        }
    }
    (records, dropped)
}

/// A tag as a file name: a tag is typed by the user, so it is sanitised the way
/// the model store sanitises a document name.
fn tag_slug(tag: &str) -> String {
    let safe: String = tag
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.' { c } else { '_' })
        .collect();
    let trimmed = safe.trim_matches(['.', '_']).to_owned();
    if trimmed.is_empty() {
        "untagged".into()
    } else {
        trimmed
    }
}

/// What a sweep has already settled, and the handle it appends to.
pub struct Ledger {
    path: PathBuf,
    records: BTreeMap<String, Record>,
    dropped: usize,
    /// Held open for the whole run. `None` for a read-only ledger, which is what
    /// the window opens to count what a resume would skip.
    file: Option<std::fs::File>,
}

impl Ledger {
    /// Where a tag's ledger lives: beside the fetch cache, whose subfolders are
    /// KiCad's own names, so a file at the root collides with nothing.
    pub fn path_for(cache_root: &Path, tag: &str) -> PathBuf {
        cache_root.join(format!("bulk-import-{}.jsonl", tag_slug(tag)))
    }

    /// Read a tag's ledger without opening it for writing. A missing file is an
    /// empty ledger, which is what a first run has.
    pub fn read(cache_root: &Path, tag: &str) -> Self {
        let path = Ledger::path_for(cache_root, tag);
        let (records, dropped) = match std::fs::read_to_string(&path) {
            Ok(text) => parse_ledger(&text),
            Err(_) => (BTreeMap::new(), 0),
        };
        Self { path, records, dropped, file: None }
    }

    /// Read it and open it for appending. Fails loudly: a sweep whose ledger
    /// cannot be written is a sweep that cannot be resumed, and starting one
    /// anyway would waste hours.
    pub fn open(cache_root: &Path, tag: &str) -> Result<Self, String> {
        let mut ledger = Ledger::read(cache_root, tag);
        if let Some(dir) = ledger.path.parent() {
            std::fs::create_dir_all(dir).map_err(|error| format!("{}: {error}", dir.display()))?;
        }
        let file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&ledger.path)
            .map_err(|error| format!("{}: {error}", ledger.path.display()))?;
        ledger.file = Some(file);
        Ok(ledger)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Lines that were not records — a torn write, or a newer version's format.
    pub fn dropped(&self) -> usize {
        self.dropped
    }

    pub fn get(&self, key: &str) -> Option<&Record> {
        self.records.get(key)
    }

    /// Whether a resume skips this key.
    pub fn settled(&self, key: &str) -> bool {
        self.records.get(key).is_some_and(|record| record.outcome.settled())
    }

    /// How many parts are settled, and how many are waiting to be retried.
    pub fn counts(&self) -> (usize, usize) {
        let settled = self.records.values().filter(|record| record.outcome.settled()).count();
        (settled, self.records.len() - settled)
    }

    /// How many of each grade the ledger holds, in the order [`Grade`] declares.
    pub fn grades(&self) -> Vec<(Grade, usize)> {
        [Grade::Full, Grade::NoModel, Grade::ModelUnreadable, Grade::NoFootprint]
            .into_iter()
            .map(|grade| {
                let count = self
                    .records
                    .values()
                    .filter(|record| record.outcome == Outcome::Done(grade))
                    .count();
                (grade, count)
            })
            .collect()
    }

    /// Append one record and FLUSH it, so a process that exits immediately
    /// afterwards leaves it behind. One line, one write: there is no state in
    /// which half a record is on disk and the reader takes it.
    pub fn append(&mut self, record: Record) -> Result<(), String> {
        use std::io::Write;
        let line = record.to_line();
        if let Some(file) = &mut self.file {
            file.write_all(line.as_bytes()).map_err(|error| format!("{}: {error}", self.path.display()))?;
            file.flush().map_err(|error| format!("{}: {error}", self.path.display()))?;
        }
        self.records.insert(record.key.clone(), record);
        Ok(())
    }
}

// ============================================================================
// Where a part is written
// ============================================================================

/// Where an imported part goes. A seam for two reasons: the sweep runs on a
/// worker thread and the app's own [`ModelStore`] is not `Send`, and the suite
/// needs to watch thousands of writes without touching the user's config folder.
pub trait PartWriter {
    fn write(&self, document: &str, json: &str) -> Result<(), String>;
}

/// Any [`ModelStore`] as a writer — the single-part save, on the UI thread.
pub struct StoreWriter<'a>(pub &'a dyn ModelStore);

impl PartWriter for StoreWriter<'_> {
    fn write(&self, document: &str, json: &str) -> Result<(), String> {
        self.0.write(document, json)
    }
}

/// The app's own model store, built INSIDE each write so that nothing
/// non-`Send` has to cross into the worker. `FileModelStore::new` is a couple
/// of path joins; the write itself is the cost.
pub struct AppStoreWriter;

impl PartWriter for AppStoreWriter {
    fn write(&self, document: &str, json: &str) -> Result<(), String> {
        crate::store::default_model_store().write(document, json)
    }
}

/// Import one part into a FRESH document and write it. The one place a part
/// becomes a document, shared by the single save and the sweep so that a bulk
/// part is the same part a user would have imported by hand.
pub fn write_part(writer: &dyn PartWriter, document: &str, part: &KicadPart) -> Result<Imported, String> {
    let mut state = EngineState::new();
    let imported = import_part(&mut state, part)?;
    writer.write(document, &state.history_request_json())?;
    Ok(imported)
}

/// A document name for a bulk-imported part, library nickname INCLUDED:
/// `Timer:NE555D` becomes `Timer_NE555D`.
///
/// The single-part window drops the nickname, because a user who is importing
/// one part is looking at the name field and can see it. A sweep is not looking:
/// `Device:R`, `Device_Fan_Motor:R` and every other library's `R` would all land
/// on one document and overwrite each other, and the ledger would claim all of
/// them were imported.
pub fn bulk_document_name(library_id: &str) -> String {
    let cleaned: String = library_id
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.' { c } else { '_' })
        .collect();
    let trimmed = cleaned.trim_matches(['_', '.']).to_owned();
    if trimmed.is_empty() {
        "kicad-part".into()
    } else {
        trimmed
    }
}

/// How complete a part that came back is. Read off the part's own structure,
/// not off a count of its notes, so it is the same answer the review grid shows.
pub fn grade(part: &KicadPart) -> Grade {
    if part.model.is_some() {
        Grade::Full
    } else if part.footprint.is_none() {
        Grade::NoFootprint
    } else if part.notes.iter().any(|note| note.contains("did not import")) {
        Grade::ModelUnreadable
    } else {
        Grade::NoModel
    }
}

/// Parse the part's `.step`, reusing a parse this library already paid for.
///
/// The memo holds the GEOMETRY, and the part's own `(model …)` entry is put back
/// on the reused copy: two footprints share one `.step` at different rotations,
/// and `model_feature` places the bodies by that entry. Reusing it wholesale
/// would import the first footprint's orientation into the second's part.
fn settle_with_memo(part: &mut KicadPart, memo: &mut HashMap<String, ModelFile>) {
    let Some(step) = part.step.take() else {
        return;
    };
    if let Some(cached) = memo.get(&step.step_file) {
        let mut reused = cached.clone();
        reused.model = step.model.clone();
        reused.step_file = step.step_file.clone();
        part.settle_model(Ok(reused));
        return;
    }
    let parsed = parse_step(&step);
    if let Ok(model) = &parsed {
        if memo.len() < MODEL_MEMO {
            memo.insert(step.step_file.clone(), model.clone());
        }
    }
    part.settle_model(parsed);
}

// ============================================================================
// Progress
// ============================================================================

/// Everything the window shows about a sweep, as a whole snapshot. Snapshots
/// rather than deltas because the UI thread may miss any number of them: the
/// latest one is always the truth, and a dropped one costs nothing.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Progress {
    pub libraries_total: usize,
    pub libraries_done: usize,
    /// Libraries whose own `.kicad_sym` did not come down. Nothing is recorded
    /// for their symbols, so a resume walks them again.
    pub libraries_failed: usize,
    pub library: String,
    /// Symbols in the library being walked, known only once it is parsed —
    /// which is why the bar is two-level and the total is not a guess.
    pub symbols_total: usize,
    pub symbols_done: usize,
    pub symbol: String,
    pub imported: usize,
    pub skipped: usize,
    pub failed: usize,
    pub files: usize,
    pub bytes: u64,
    /// The last few things that went wrong, newest last.
    pub notes: Vec<String>,
    /// The sweep has stopped, for any reason.
    pub finished: bool,
    /// It stopped because it was asked to.
    pub cancelled: bool,
    /// It stopped because it could not go on — a ledger it cannot append to.
    pub error: Option<String>,
}

impl Progress {
    /// How far along, in `0.0..=1.0`: whole libraries plus the fraction of the
    /// one in hand. Honest about what is knowable — the symbol count of a
    /// library nobody has downloaded yet is not.
    pub fn fraction(&self) -> f32 {
        if self.libraries_total == 0 {
            return 0.0;
        }
        let within = if self.symbols_total == 0 {
            0.0
        } else {
            (self.symbols_done as f32 / self.symbols_total as f32).clamp(0.0, 1.0)
        };
        ((self.libraries_done as f32 + within) / self.libraries_total as f32).clamp(0.0, 1.0)
    }

    /// The line the window puts under the bar.
    pub fn line(&self) -> String {
        if let Some(error) = &self.error {
            return format!("the sweep stopped: {error}");
        }
        let head = if self.finished {
            if self.cancelled {
                "cancelled".to_string()
            } else {
                "finished".to_string()
            }
        } else if self.symbols_total == 0 {
            format!("{} \u{2014} reading its symbols", self.library)
        } else {
            format!("{} {}/{} \u{2014} {}", self.library, self.symbols_done, self.symbols_total, self.symbol)
        };
        format!(
            "{head} | library {}/{} | {} imported, {} already done, {} failed | {} files, {:.1} MB",
            self.libraries_done.min(self.libraries_total),
            self.libraries_total,
            self.imported,
            self.skipped,
            self.failed,
            self.files,
            self.bytes as f64 / 1_048_576.0
        )
    }

    fn note(&mut self, note: String) {
        self.notes.push(trim_note(&note));
        if self.notes.len() > SHOWN_NOTES {
            self.notes.remove(0);
        }
    }
}

// ============================================================================
// The sweep
// ============================================================================

/// What to sweep.
pub struct Sweep {
    /// Where files are cached and where the ledger lives.
    pub cache_root: PathBuf,
    pub tag: String,
    /// The symbol libraries to walk, in order. One name is how a user sweeps a
    /// single library, which is also the only size of sweep that has been
    /// measured end to end.
    pub libraries: Vec<String>,
    /// Redo parts the ledger has already settled.
    pub force: bool,
}

/// Walk `sweep` to the end, or until `cancel` is set.
///
/// Synchronous and thread-free on purpose: [`start`] is a thin wrapper that puts
/// this on a worker, and the suite drives THIS, so what the tests pin is the
/// resume, the recording and the cancellation rather than a thread's timing.
///
/// `report` is handed the snapshot after every part; the window's copy goes
/// down a channel.
pub fn sweep(
    plan: &Sweep,
    files: &RemoteFiles,
    ledger: &mut Ledger,
    writer: &dyn PartWriter,
    cancel: &AtomicBool,
    report: &mut dyn FnMut(&Progress),
) -> Progress {
    let mut progress = Progress { libraries_total: plan.libraries.len(), ..Progress::default() };
    report(&progress);
    for (index, library) in plan.libraries.iter().enumerate() {
        if cancel.load(Ordering::Relaxed) {
            progress.cancelled = true;
            break;
        }
        progress.libraries_done = index;
        progress.library = library.clone();
        progress.symbols_total = 0;
        progress.symbols_done = 0;
        progress.symbol.clear();
        report(&progress);

        let file = format!("{library}.kicad_sym");
        let read = files
            .fetch_file(Repo::Symbols, &file)
            .and_then(|bytes| read_symbol_library(&file, &bytes));
        let _ = files.take_problems();
        let symbols = match read {
            Ok((symbols, _warnings)) => symbols,
            Err(error) => {
                // Nothing is written to the ledger: this library's symbols are
                // unknown, so a resume walks it again from the top.
                progress.libraries_failed += 1;
                progress.note(format!("{library}: {error}"));
                progress.libraries_done = index + 1;
                report(&progress);
                continue;
            }
        };
        progress.symbols_total = symbols.len();
        report(&progress);

        let mut memo: HashMap<String, ModelFile> = HashMap::new();
        for symbol in symbols {
            if cancel.load(Ordering::Relaxed) {
                progress.cancelled = true;
                break;
            }
            let key = symbol.library_id.clone();
            if !plan.force && ledger.settled(&key) {
                progress.skipped += 1;
                progress.symbols_done += 1;
                continue;
            }
            progress.symbol = key.clone();
            report(&progress);

            // Start the part with a clean slate, so what is drained after it
            // belongs to it alone.
            let _ = files.take_fetched();
            let _ = files.take_problems();
            let before = files.fetched_bytes();

            let mut part = part_from_symbol(symbol, &FootprintChoice::Named, &files.library(), files);
            settle_with_memo(&mut part, &mut memo);

            let fetched = files.take_fetched();
            let problems = files.take_problems();
            let bytes = files.fetched_bytes().saturating_sub(before);
            let document = bulk_document_name(&key);
            let mut notes: Vec<String> =
                part.notes.iter().take(LEDGER_NOTES).map(|note| trim_note(note)).collect();

            // The document FIRST, its ledger line second. See the module doc.
            let outcome = if problems.is_empty() {
                match write_part(writer, &document, &part) {
                    Ok(_) => Outcome::Done(grade(&part)),
                    Err(error) => Outcome::Failed(error),
                }
            } else {
                // A part whose reads were not all answered is not a part whose
                // gaps are KiCad's. Nothing is written for it.
                Outcome::Failed(problems.join("; "))
            };
            let written = matches!(outcome, Outcome::Done(_));
            if let Outcome::Failed(reason) = &outcome {
                progress.failed += 1;
                progress.note(format!("{key}: {reason}"));
                notes.clear();
            } else {
                progress.imported += 1;
            }
            progress.files += fetched.len();
            progress.bytes += bytes;

            let record = Record {
                key,
                outcome,
                document: if written { document } else { String::new() },
                notes,
                files: fetched.len(),
                bytes,
                at: now(),
            };
            if let Err(error) = ledger.append(record) {
                // A ledger that cannot be appended to cannot be resumed, and a
                // sweep that cannot be resumed must not keep running for hours.
                progress.error = Some(error);
                progress.finished = true;
                report(&progress);
                return progress;
            }
            progress.symbols_done += 1;
            report(&progress);
        }
        if progress.cancelled {
            break;
        }
        progress.libraries_done = index + 1;
        report(&progress);
    }
    progress.symbol.clear();
    progress.finished = true;
    report(&progress);
    progress
}

/// A sweep running on a worker thread, and the window's view of it.
pub struct BulkRun {
    updates: std::sync::mpsc::Receiver<Progress>,
    cancel: Arc<AtomicBool>,
    /// The latest snapshot the worker sent.
    pub progress: Progress,
    /// The ledger this run appends to, for the window to name.
    pub ledger: PathBuf,
    /// What the ledger already held when the run started.
    pub resumed: usize,
}

impl BulkRun {
    /// Drain every snapshot waiting and keep the last. `true` when the view
    /// moved, which is what the frame loop repaints on.
    pub fn poll(&mut self) -> bool {
        let mut moved = false;
        loop {
            match self.updates.try_recv() {
                Ok(progress) => {
                    self.progress = progress;
                    moved = true;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => return moved,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    // The worker is gone. If it never said it had finished, say
                    // so here rather than showing a bar that never moves again.
                    if !self.progress.finished {
                        self.progress.finished = true;
                        if self.progress.error.is_none() && !self.progress.cancelled {
                            self.progress.error = Some("the sweep stopped without saying why".into());
                        }
                        moved = true;
                    }
                    return moved;
                }
            }
        }
    }

    /// Ask it to stop. Cooperative: the part in hand finishes and is recorded,
    /// then the sweep returns, so no part is left half-done. A download already
    /// in flight has to come back first, which is what "at worst one file"
    /// means.
    pub fn stop(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    pub fn stopping(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }

    pub fn finished(&self) -> bool {
        self.progress.finished
    }
}

/// Start a sweep on a worker thread.
///
/// The ledger is opened HERE, before the thread exists, so a cache folder that
/// cannot be written is an error the user sees instead of an hours-long run
/// whose progress is lost.
pub fn start(plan: Sweep, fetch: Fetcher, writer: Box<dyn PartWriter + Send>) -> Result<BulkRun, String> {
    let mut ledger = Ledger::open(&plan.cache_root, &plan.tag)?;
    let resumed = ledger.counts().0;
    let path = ledger.path().to_path_buf();
    let files = RemoteFiles::new(plan.cache_root.clone(), plan.tag.clone(), fetch);
    files.prepare()?;
    let cancel = Arc::new(AtomicBool::new(false));
    let (send, updates) = std::sync::mpsc::channel();
    let flag = Arc::clone(&cancel);
    std::thread::spawn(move || {
        let mut report = |progress: &Progress| {
            // A window that has been closed has dropped the receiver; nobody
            // waits, and the cancel flag is what stops the sweep.
            let _ = send.send(progress.clone());
        };
        sweep(&plan, &files, &mut ledger, writer.as_ref(), &flag, &mut report);
    });
    Ok(BulkRun {
        updates,
        cancel,
        progress: Progress::default(),
        ledger: path,
        resumed,
    })
}

