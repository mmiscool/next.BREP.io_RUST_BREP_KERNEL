//! Diagnostic events (contract §14): bounded, structured, payload-free.
//!
//! Each event carries the process epoch, a stable correlation id, the
//! operation / execution id when known, monotonic elapsed milliseconds, a
//! state or phase, an error code and a sequence — never source, arguments,
//! feature contents, paths, geometry, messages or image bytes. The last
//! [`KEEP`] events are kept in memory ([`recent`]); with `BREP_CAD_TRACE=1`
//! each is also written to stderr as one JSON line (stdout is the stdio
//! transport's and is never written).
use serde_json::{json, Value};
use std::collections::VecDeque;
use std::sync::{Mutex, OnceLock};
use std::sync::atomic::{AtomicU64, Ordering};
static EVENT_SEQUENCE: AtomicU64 = AtomicU64::new(0);
use std::time::Instant;

pub const KEEP: usize = 1024;

fn ring() -> &'static Mutex<VecDeque<Value>> {
    static RING: OnceLock<Mutex<VecDeque<Value>>> = OnceLock::new();
    RING.get_or_init(|| Mutex::new(VecDeque::with_capacity(KEEP)))
}

fn origin() -> Instant {
    static ORIGIN: OnceLock<Instant> = OnceLock::new();
    *ORIGIN.get_or_init(Instant::now)
}

fn trace() -> bool {
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_CAD_TRACE").is_some_and(|v| v == "1"))
}

/// One event. `kind` is one of the contract's names (`operation_accepted`,
/// `queue_enter`, …); `id` an operation or execution handle; `state` a
/// state or phase; `code` an error code; `sequence` the record's sequence.
pub fn event(kind: &'static str, id: Option<&str>, state: Option<&str>, code: Option<&str>, sequence: Option<u64>) -> Value {
    let mut ring = ring().lock().unwrap();
    let e = json!({
        "event": kind,
        "eventSequence": EVENT_SEQUENCE.fetch_add(1, Ordering::Relaxed).to_string(),
        "epoch": super::wire::epoch(),
        "correlationId": id.map(str::to_string).unwrap_or_else(|| super::wire::epoch().to_string()),
        "id": id,
        "elapsedMs": origin().elapsed().as_millis() as u64,
        "state": state,
        "code": code,
        "sequence": sequence.map(|s| s.to_string()),
    });
    if trace() {
        eprintln!("{e}");
    }
    if ring.len() == KEEP {
        ring.pop_front();
    }
    ring.push_back(e.clone());
    e
}

/// The kept events, oldest first.
pub fn recent() -> Vec<Value> {
    ring().lock().unwrap().iter().cloned().collect()
}

/// Replay is local to this process epoch; it is not a durable event journal.
pub const PAGE_LIMIT: usize = 128;

pub fn replay(cursor: Option<&str>, limit: usize) -> Result<Value, String> {
    let ring = ring().lock().unwrap();
    page(&ring, EVENT_SEQUENCE.load(Ordering::Relaxed), super::wire::epoch(), cursor, limit)
}

fn page(ring: &VecDeque<Value>, head: u64, epoch: &str, cursor: Option<&str>, limit: usize) -> Result<Value, String> {
    if limit == 0 || limit > PAGE_LIMIT { return Err(format!("event limit must be 1..={PAGE_LIMIT}")); }
    let oldest = ring.front().and_then(|e| e["eventSequence"].as_str()).and_then(|s| s.parse::<u64>().ok()).unwrap_or(head);
    let (wanted, gap) = if let Some(cursor) = cursor {
        if cursor.len() > 43 { return Err("invalid event cursor".into()); }
        let (at_epoch, sequence) = cursor.split_once(':').ok_or("event cursor must contain epoch and sequence")?;
        if at_epoch.len()!=22 || !at_epoch.bytes().all(|b| b.is_ascii_alphanumeric() || b==b'_' || b==b'-') {
            return Err("invalid event cursor epoch".into());
        }
        let number = sequence.parse::<u64>().map_err(|_| "invalid event cursor sequence")?;
        if number.to_string()!=sequence { return Err("event cursor sequence must be canonical decimal".into()); }
        let reason = if at_epoch != epoch { Some("epoch_changed") } else if number < oldest { Some("retention_gap") } else if number > head { Some("cursor_ahead") } else { None };
        (number,reason)
    } else { (oldest,None) };
    let events:Vec<Value> = if gap.is_some() { vec![] } else {
        ring.iter().filter(|e| e["eventSequence"].as_str().and_then(|s| s.parse::<u64>().ok()).is_some_and(|n| n>=wanted)).take(limit).cloned().collect()
    };
    let next = if gap.is_some() { oldest } else { events.last().and_then(|e| e["eventSequence"].as_str()).and_then(|s| s.parse::<u64>().ok()).map_or(wanted,|n| n+1) };
    Ok(json!({"epoch":epoch,"events":events,"capacity":KEEP,"pageLimit":PAGE_LIMIT,
        "oldestSequence":oldest.to_string(),"headSequence":head.to_string(),
        "nextCursor":format!("{epoch}:{next}"),"nextUri":format!("brep://events/replay/{epoch}/{next}/{limit}"),
        "hasMore":next<head,"gap":gap.map(|reason| json!({"reason":reason,"reconciliationRequired":true})),
        "recovery":"On a gap, read current document and execution state before resuming; events are not durable or a complete transaction log."}))
}

/// Fixed path segments avoid query decoding ambiguities in cursor resources.
pub fn resource(uri: &str) -> Result<Value, String> {
    if uri.len()>128 { return Err("event replay URI is too long".into()); }
    if uri == "brep://events" { return replay(None,PAGE_LIMIT); }
    let path = uri.strip_prefix("brep://events/replay/").ok_or("unknown event resource")?;
    let parts:Vec<&str> = path.split('/').collect();
    if parts.len()!=3 { return Err("event replay URI requires epoch/sequence/limit".into()); }
    let limit = parts[2].parse::<usize>().map_err(|_| "invalid event limit")?;
    if limit.to_string()!=parts[2] { return Err("event limit must be canonical decimal".into()); }
    replay(Some(&format!("{}:{}",parts[0],parts[1])),limit)
}

