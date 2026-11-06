//! Optional exact-key access stream. Scope dictionaries retain full keys, never
//! hash-only identities. No data is retained unless both profile flags are set.
use std::collections::HashMap;
use std::hash::Hash;
use std::io::{BufWriter, Write};
use std::sync::{Mutex, OnceLock};
use std::sync::atomic::{AtomicU64, Ordering};
use serde::Serialize;

fn writer() -> Option<&'static Mutex<BufWriter<std::fs::File>>> {
    if !crate::profile::profile_enabled() { return None; }
    static WRITER: OnceLock<Option<Mutex<BufWriter<std::fs::File>>>> = OnceLock::new();
    WRITER.get_or_init(|| env_read!("BREP_PROFILE_REUSE").map(|path|
        Mutex::new(BufWriter::new(std::fs::File::create(std::path::Path::new(&*path)).expect("open reuse trace")))))
        .as_ref()
}
fn emit(value: impl Serialize) {
    if let Some(writer) = writer() {
        let mut writer = writer.lock().unwrap();
        serde_json::to_writer(&mut *writer, &value).expect("write reuse trace");
        writer.write_all(b"\n").expect("write reuse newline");
    }
}
pub(crate) struct Trace<K> {
    scope: u64,
    keys: HashMap<K, usize>,
    accesses: usize,
}
impl<K: Eq + Hash + Clone + Serialize> Trace<K> {
    pub(crate) fn open(kind: &'static str, context: impl FnOnce() -> serde_json::Value) -> Option<Self> {
        writer()?;
        static NEXT: AtomicU64 = AtomicU64::new(0);
        let scope = NEXT.fetch_add(1, Ordering::Relaxed);
        emit(("scope", scope, kind, context()));
        Some(Self { scope, keys: HashMap::new(), accesses: 0 })
    }
    pub(crate) fn access(&mut self, key: &K, caller: &'static std::panic::Location<'static>, hit: bool, success: bool) {
        let next = self.keys.len();
        let id = *self.keys.entry(key.clone()).or_insert_with(|| {
            emit(("key", self.scope, next, key)); next
        });
        emit(("access", self.scope, self.accesses, id, caller.file(), caller.line(), hit, success));
        self.accesses += 1;
    }
}
impl<K> Drop for Trace<K> {
    fn drop(&mut self) {
        emit(("end", self.scope, self.accesses, self.keys.len()));
        if let Some(writer) = writer() { writer.lock().unwrap().flush().expect("flush reuse trace"); }
    }
}
