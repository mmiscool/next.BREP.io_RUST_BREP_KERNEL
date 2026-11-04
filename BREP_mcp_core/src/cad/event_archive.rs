//! Server-owned diagnostic archive. Event producers never wait for disk I/O.
use super::event_journal::{Event, Journal, Record};
use serde_json::{json, Value};
use std::collections::VecDeque;
use std::path::Path;
use std::sync::{Arc, Mutex, mpsc, atomic::{AtomicU64, Ordering}};
use std::thread::JoinHandle;
use std::time::Duration;

const QUEUE: usize = 128;
const PAGE: usize = 128;
struct Snapshot {
    id: String,
    records: VecDeque<Record>,
    head: u64,
    failed: bool,
}

pub struct Archive {
    sender: Option<mpsc::SyncSender<Event>>,
    worker: Option<JoinHandle<()>>,
    snapshot: Arc<Mutex<Snapshot>>,
    pending: Arc<AtomicU64>,
    lost: Arc<AtomicU64>,
    loss_total: AtomicU64,
}

impl Archive {
    pub fn open(directory: &Path) -> Result<Self, String> {
        let mut journal = Journal::open(directory)?;
        // Even a clean file cannot prove that the previous process's bounded
        // in-memory queue was drained. Conservatively mark every reopen.
        if !journal.newly_created { journal.restart_gap()?; }
        let snapshot = Arc::new(Mutex::new(Snapshot { id: journal.id.clone(), records: journal.records().clone(), head: journal.head(), failed: false }));
        let pending = Arc::new(AtomicU64::new(0));
        let lost = Arc::new(AtomicU64::new(0));
        let (sender, receiver) = mpsc::sync_channel(QUEUE);
        let (view, outstanding, losses) = (snapshot.clone(), pending.clone(), lost.clone());
        let worker = std::thread::Builder::new().name("brep-event-archive".into()).spawn(move || {
            run_writer(journal, receiver, view, outstanding, losses);
        }).map_err(|e| e.to_string())?;
        Ok(Self { sender: Some(sender), worker: Some(worker), snapshot, pending, lost, loss_total: AtomicU64::new(0) })
    }

    pub fn publish(&self, value: Value) {
        let event = Event::from_value(value);
        self.pending.fetch_add(1, Ordering::SeqCst);
        if let Ok(event) = event {
            match self.sender.as_ref().unwrap().try_send(event) {
                Ok(()) => return,
                Err(mpsc::TrySendError::Disconnected(_)) => self.snapshot.lock().unwrap().failed = true,
                Err(mpsc::TrySendError::Full(_)) => {},
            }
        }
        self.lost.fetch_add(1, Ordering::SeqCst);
        self.loss_total.fetch_add(1, Ordering::SeqCst);
        self.pending.fetch_sub(1, Ordering::SeqCst);
    }

    /// Snapshot contains only confirmed synced records. Pending queue entries
    /// are explicitly reported so an empty page is not a durability promise.
    pub fn resource(&self, uri: &str) -> Result<Value, String> {
        let cursor = parse_uri(uri)?;
        let view = self.snapshot.lock().unwrap();
        let oldest = view.records.front().map_or(view.head, |r| r.sequence);
        let (wanted, limit, gap) = match cursor {
            None => (oldest, PAGE, None),
            Some((id, seq, limit)) => {
                let reason = if id != view.id { Some("archive_changed") }
                    else if seq < oldest { Some("retention_gap") }
                    else if seq > view.head { Some("cursor_ahead") } else { None };
                (seq, limit, reason)
            }
        };
        let records: Vec<Value> = if gap.is_some() { vec![] } else {
            view.records.iter().filter(|r| r.sequence >= wanted).take(limit).map(|r| {
                // Wire cursors are decimal strings, never lossy JS numbers.
                json!({"sequence":r.sequence.to_string(),"entry":r.entry})
            }).collect()
        };
        let next = if gap.is_some() { oldest } else {
            records.last().map_or(wanted, |r| r["sequence"].as_str().unwrap().parse::<u64>().unwrap() + 1)
        };
        Ok(json!({"enabled":true,"archiveId":view.id,"records":records,
            "oldestSequence":oldest.to_string(),"headSequence":view.head.to_string(),
            "nextUri":format!("brep://events/archive/{}/{next}/{limit}",view.id),
            "hasMore":next<view.head,"pageLimit":PAGE,"capacity":1024,
            "gap":gap.map(|reason|json!({"reason":reason,"reconciliationRequired":true})),
            "persistence":{"status":if view.failed {"failed"} else {"active"},
                "pending":self.pending.load(Ordering::SeqCst),"unrecordedLoss":self.lost.load(Ordering::SeqCst),
                "droppedThisProcess":self.loss_total.load(Ordering::SeqCst),
                "reconciliationRequired":view.failed},
            "recovery":"Diagnostic history is incomplete by design. Gap records and cursor gaps require current-state reconciliation; never replay CAD work from events."}))
    }
}

impl Drop for Archive {
    fn drop(&mut self) {
        // No producer remains once the owning service drops. Drain the bounded
        // queue before releasing the archive lock; no async work is needed.
        drop(self.sender.take());
        if let Some(worker) = self.worker.take() { let _ = worker.join(); }
    }
}

fn parse_uri(uri: &str) -> Result<Option<(&str, u64, usize)>, String> {
    if uri == "brep://events/archive" { return Ok(None); }
    if uri.len() > 128 { return Err("archive URI exceeds limit".into()); }
    let parts: Vec<_> = uri.strip_prefix("brep://events/archive/").ok_or("unknown archive resource")?.split('/').collect();
    if parts.len() != 3 || uuid::Uuid::parse_str(parts[0]).is_err() { return Err("invalid archive cursor".into()); }
    let sequence = parts[1].parse::<u64>().map_err(|_| "invalid archive sequence")?;
    let limit = parts[2].parse::<usize>().map_err(|_| "invalid archive page limit")?;
    if sequence.to_string() != parts[1] || limit.to_string() != parts[2] || !(1..=PAGE).contains(&limit) {
        return Err("noncanonical archive cursor or page limit".into());
    }
    Ok(Some((parts[0], sequence, limit)))
}

fn run_writer(mut journal: Journal, receiver: mpsc::Receiver<Event>, view: Arc<Mutex<Snapshot>>, pending: Arc<AtomicU64>, lost: Arc<AtomicU64>) {
    let mut failed = false;
    loop {
        let received = receiver.recv_timeout(Duration::from_millis(20));
        let disconnected = matches!(received, Err(mpsc::RecvTimeoutError::Disconnected));
        let losses = lost.load(Ordering::SeqCst);
        if losses > 0 {
            if failed || journal.writer_gap().is_err() { failed = true; }
            else { lost.fetch_sub(losses, Ordering::SeqCst); }
        }
        let event = received.ok();
        let had_event = event.is_some();
        if let Some(event) = event {
            if failed || journal.append(event).is_err() {
                failed = true;
                lost.fetch_add(1, Ordering::SeqCst);
            }
        }
        {
            let mut snapshot = view.lock().unwrap();
            let oldest = journal.records().front().map_or(journal.head(), |r| r.sequence);
            while snapshot.records.front().is_some_and(|r| r.sequence < oldest) { snapshot.records.pop_front(); }
            let previous_head = snapshot.head;
            snapshot.records.extend(journal.records().iter().filter(|r| r.sequence >= previous_head).cloned());
            snapshot.head = journal.head();
            snapshot.failed = failed;
        }
        if had_event { pending.fetch_sub(1, Ordering::SeqCst); }
        if disconnected { break; }
    }
}

