//! Opt-in diagnostic archive storage; never execution/admission evidence.
//! Two bounded segments retain metadata across process epochs. A torn final
//! record becomes an explicit gap; malformed complete records refuse opening.
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

const SEGMENT_RECORDS: usize = 512;
const MAX_FRAME: usize = 2048;
const MAX_SEGMENT: u64 = (SEGMENT_RECORDS * MAX_FRAME) as u64;

/// Deliberately no payload, message, path, source, or arguments field.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Event {
    pub event: String,
    pub event_sequence: String,
    pub epoch: String,
    pub correlation_id: String,
    pub id: Option<String>,
    pub elapsed_ms: u64,
    pub state: Option<String>,
    pub code: Option<String>,
    pub sequence: Option<String>,
}

impl Event {
    pub fn from_value(value: Value) -> Result<Self, String> {
        let event: Self = serde_json::from_value(value).map_err(|e| e.to_string())?;
        event.validate()?;
        Ok(event)
    }
    fn validate(&self) -> Result<(), String> {
        // Call sites supply names and opaque handles only. Bound each field so
        // disk/memory budgets also hold for malformed recovered records.
        for text in [Some(self.event.as_str()), Some(self.event_sequence.as_str()),
            Some(self.epoch.as_str()), Some(self.correlation_id.as_str()), self.id.as_deref(),
            self.state.as_deref(), self.code.as_deref(), self.sequence.as_deref()].into_iter().flatten() {
            if text.len() > 128 || text.chars().any(char::is_control) {
                return Err("diagnostic metadata field exceeds its contract".into());
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Entry {
    Event { event: Event },
    Gap { reason: String },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Record {
    pub version: u32,
    pub sequence: u64,
    pub entry: Entry,
}

pub struct Journal {
    directory: PathBuf,
    _lock: File,
    pub id: String,
    pub newly_created: bool,
    active: File,
    active_count: usize,
    records: VecDeque<Record>,
    next: u64,
    poisoned: bool,
}

fn private_file(path: &Path) -> Result<File, String> {
    if std::fs::symlink_metadata(path).is_ok_and(|m| !m.is_file() || m.file_type().is_symlink()) {
        return Err("diagnostic journal requires regular files".into());
    }
    let mut options = OpenOptions::new();
    options.read(true).append(true).create(true);
    #[cfg(unix)] {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    let file = options.open(path).map_err(|e| e.to_string())?;
    #[cfg(unix)] {
        use std::os::unix::fs::PermissionsExt;
        file.set_permissions(std::fs::Permissions::from_mode(0o600)).map_err(|e| e.to_string())?;
    }
    Ok(file)
}

fn sync_directory(path: &Path) -> Result<(), String> {
    #[cfg(unix)]
    File::open(path).and_then(|f| f.sync_all()).map_err(|e| e.to_string())?;
    #[cfg(not(unix))]
    let _ = path;
    Ok(())
}

fn read_segment(file: &mut File) -> Result<(Vec<Record>, u64, bool), String> {
    let mut bytes = Vec::new();
    Read::by_ref(file).take(MAX_SEGMENT + 1).read_to_end(&mut bytes).map_err(|e| e.to_string())?;
    if bytes.len() as u64 > MAX_SEGMENT { return Err("diagnostic segment exceeds budget".into()); }
    let mut valid = 0;
    let mut records = Vec::new();
    for line in bytes.split_inclusive(|b| *b == b'\n') {
        if line.len() > MAX_FRAME { return Err("diagnostic frame exceeds budget".into()); }
        if line.last() != Some(&b'\n') { return Ok((records, valid, true)); }
        let record: Record = serde_json::from_slice(line).map_err(|_| "invalid diagnostic frame")?;
        if record.version != 1 { return Err("unsupported diagnostic journal version".into()); }
        match &record.entry {
            Entry::Event { event } => event.validate()?,
            Entry::Gap { reason } if matches!(reason.as_str(), "truncated_write" | "writer_loss" | "process_boundary") => {},
            _ => return Err("invalid diagnostic gap reason".into()),
        }
        if records.len() >= SEGMENT_RECORDS { return Err("too many diagnostic frames".into()); }
        records.push(record);
        valid += line.len() as u64;
    }
    Ok((records, valid, false))
}

impl Journal {
    pub fn open(directory: &Path) -> Result<Self, String> {
        crate::host::private_dir(directory)?;
        let mut lock = private_file(&directory.join("archive.lock"))?;
        lock.try_lock().map_err(|_| "diagnostic archive is already in use")?;
        let mut id = String::new();
        Read::by_ref(&mut lock).take(129).read_to_string(&mut id).map_err(|e| e.to_string())?;
        let newly_created = id.is_empty();
        if id.is_empty() {
            id = uuid::Uuid::new_v4().to_string();
            lock.write_all(id.as_bytes()).and_then(|_| lock.sync_all()).map_err(|e| e.to_string())?;
        }
        if uuid::Uuid::parse_str(&id).is_err() { return Err("invalid diagnostic archive identity".into()); }
        let mut previous = private_file(&directory.join("previous.jsonl"))?;
        let (previous_records, _, previous_torn) = read_segment(&mut previous)?;
        drop(previous);
        // Only the active segment is ever appended. A torn sealed segment is
        // corruption, not an interrupted append that can safely be repaired.
        if previous_torn { return Err("sealed diagnostic segment is truncated".into()); }
        let mut active = private_file(&directory.join("active.jsonl"))?;
        let (active_records, valid_bytes, torn) = read_segment(&mut active)?;
        let active_count = active_records.len();
        let records: VecDeque<_> = previous_records.into_iter().chain(active_records).collect();
        let mut last = None;
        for record in &records {
            if last.is_some_and(|n: u64| n.checked_add(1) != Some(record.sequence)) {
                return Err("diagnostic sequence discontinuity".into());
            }
            last = Some(record.sequence);
        }
        let next = match last { Some(n) => n.checked_add(1).ok_or("diagnostic sequence exhausted")?, None => 0 };
        // Persist repair intent before discarding a torn suffix. If recovery
        // itself crashes, the next opener still reports a gap (duplicates are
        // conservative and permitted; silently losing the gap is not).
        let mut repair = private_file(&directory.join("repair.pending"))?;
        let pending = repair.metadata().map_err(|e| e.to_string())?.len();
        if pending > 1 { return Err("invalid diagnostic repair marker".into()); }
        if torn && pending == 0 {
            repair.write_all(b"1").and_then(|_| repair.sync_all()).map_err(|e| e.to_string())?;
            sync_directory(directory)?;
        }
        if torn { active.set_len(valid_bytes).and_then(|_| active.sync_all()).map_err(|e| e.to_string())?; }
        sync_directory(directory)?;
        let parent = directory.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
        sync_directory(parent)?;
        let mut journal = Self { directory: directory.to_owned(), _lock: lock, id, newly_created, active, active_count, records, next, poisoned: false };
        if torn || pending != 0 {
            journal.append_entry(Entry::Gap { reason: "truncated_write".into() })?;
            repair.set_len(0).and_then(|_| repair.sync_all()).map_err(|e| e.to_string())?;
        }
        Ok(journal)
    }

    pub fn records(&self) -> &VecDeque<Record> { &self.records }
    pub fn head(&self) -> u64 { self.next }

    pub fn writer_gap(&mut self) -> Result<(), String> {
        self.append_entry(Entry::Gap { reason: "writer_loss".into() })
    }

    pub fn restart_gap(&mut self) -> Result<(), String> {
        self.append_entry(Entry::Gap { reason: "process_boundary".into() })
    }

    pub fn append(&mut self, event: Event) -> Result<(), String> {
        event.validate()?;
        self.append_entry(Entry::Event { event })
    }

    fn append_entry(&mut self, entry: Entry) -> Result<(), String> {
        if self.poisoned { return Err("diagnostic journal write failed; reopen required".into()); }
        let next = self.next.checked_add(1).ok_or("diagnostic sequence exhausted")?;
        let record = Record { version: 1, sequence: self.next, entry };
        let mut bytes = serde_json::to_vec(&record).map_err(|e| e.to_string())?;
        bytes.push(b'\n');
        if bytes.len() > MAX_FRAME { return Err("diagnostic frame exceeds budget".into()); }
        // A failed rotation or append must never be followed by another write
        // using stale offsets. Readers can still inspect the confirmed prefix.
        self.poisoned = true;
        if self.active_count == SEGMENT_RECORDS {
            let previous = self.directory.join("previous.jsonl");
            std::fs::remove_file(&previous).map_err(|e| e.to_string())?;
            // Close the active handle before rename for Windows compatibility.
            let placeholder = self._lock.try_clone().map_err(|e| e.to_string())?;
            drop(std::mem::replace(&mut self.active, placeholder));
            std::fs::rename(self.directory.join("active.jsonl"), &previous).map_err(|e| e.to_string())?;
            self.active = private_file(&self.directory.join("active.jsonl"))?;
            sync_directory(&self.directory)?;
            self.active_count = 0;
            while self.records.len() > SEGMENT_RECORDS { self.records.pop_front(); }
        }
        self.active.write_all(&bytes).and_then(|_| self.active.sync_all()).map_err(|e| e.to_string())?;
        self.active_count += 1;
        self.records.push_back(record);
        self.next = next;
        self.poisoned = false;
        Ok(())
    }
}

