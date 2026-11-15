//! Work counters behind `BREP_PROFILE` (observations only, never a geometry cache).

use crate::profile::profile_enabled;
use rustc_hash::FxHashSet as HashSet;
use serde::Serialize;
use std::cell::RefCell;


#[derive(Clone, Copy, Default, Serialize)]
pub struct Counts {
    pub sections: u64,
    pub projection_global: u64,
    pub projection_seeded: u64,
    pub projection_from_seed: u64,
    pub pcurve_native_passes: u64,
    pub pcurve_native_samples: u64,
    pub pcurve_probe_passes: u64,
    pub pcurve_probe_samples: u64,
    pub carried_uv_curves: u64,
    pub carried_uv_stations: u64,
    pub minted_pieces: u64,
    pub trace_calls: u64,
    pub trace_curves: u64,
    pub trace_samples: u64,
    pub fit_reports: u64,
    pub fit_calls: u64,
    pub refinement_rounds: u64,
    pub fit_samples: u64,
    pub fit_raised: u64,
    pub newton_iterations: u64,
    pub newton_unused: u64,
    pub newton_unused_first: u64,
}
thread_local! {
    pub static COUNTS: RefCell<Counts> = RefCell::new(Counts::default());
    pub static PREVIOUS_FACES: RefCell<HashSet<Vec<u8>>> = RefCell::new(HashSet::default());
    pub static PREVIOUS: RefCell<HashSet<Vec<u8>>> = RefCell::new(HashSet::default());
}
#[inline]
pub fn add(update: impl FnOnce(&mut Counts)) {
    if profile_enabled() { COUNTS.with(|c| update(&mut c.borrow_mut())); }
}
/// Successful construction routes, distinct from cached fit report reads.
pub fn constructed(route: &'static str, samples: usize, controls: usize) {
    if profile_enabled() {
        profile_line!("pcurve.construction route={} samples={} controls={}", route, samples, controls);
    }
}
pub fn counts() -> Counts { COUNTS.with(|c| *c.borrow()) }

pub struct Scope { label: &'static str, before: Counts }
impl Scope {
    pub fn open(label: &'static str) -> Option<Self> {
        profile_enabled().then(|| Self { label, before: counts() })
    }
}
impl Drop for Scope {
    fn drop(&mut self) {
        let a = serde_json::to_value(counts()).unwrap();
        let b = serde_json::to_value(self.before).unwrap();
        let delta: serde_json::Map<String, serde_json::Value> = a.as_object().unwrap().iter()
            .map(|(k, v)| (k.clone(), (v.as_u64().unwrap() - b[k].as_u64().unwrap()).into())).collect();
        profile_line!("{}.work {}", self.label, serde_json::Value::Object(delta));
    }
}
