//! Exclusive feature accounting under BREP_PROFILE. Existing nested detail
//! lines remain available; these rows alone sum to the history feature timer.
use std::cell::RefCell;
use std::collections::BTreeMap;
use web_time::Instant;

#[derive(Default)]
struct Account {
    rows: BTreeMap<&'static str, (u64, f64)>,
    recorded: f64,
}
thread_local! { static ACCOUNT: RefCell<Option<Account>> = const { RefCell::new(None) }; }

pub(crate) fn record(label: &'static str, ms: f64) {
    ACCOUNT.with(|state| {
        if let Some(account) = state.borrow_mut().as_mut() {
            let row = account.rows.entry(label).or_default();
            row.0 += 1;
            row.1 += ms;
            account.recorded += ms;
        }
    });
}

/// A containing scope records only time not already attributed by its children.
pub(crate) struct Scope(Option<(Instant, f64, &'static str)>);
impl Scope {
    pub(crate) fn new(label: &'static str) -> Self {
        Self(ACCOUNT.with(|state| state.borrow().as_ref().map(|a| (Instant::now(), a.recorded, label))))
    }
}
impl Drop for Scope {
    fn drop(&mut self) {
        if let Some((start, before, label)) = self.0 {
            let ms = start.elapsed().as_secs_f64() * 1000.0;
            let children = ACCOUNT.with(|state| state.borrow().as_ref().map_or(0.0, |a| a.recorded - before));
            record(label, ms - children);
        }
    }
}

pub(crate) struct Feature(bool);
impl Feature {
    pub(crate) fn begin() -> Self {
        let enabled = std::env::var_os("BREP_PROFILE").is_some();
        Self(enabled && ACCOUNT.with(|state| {
            let mut state = state.borrow_mut();
            if state.is_some() { return false; }
            *state = Some(Account::default());
            true
        }))
    }
    pub(crate) fn finish(self, id: &str, total_ms: f64) {
        if !self.0 { return; }
        let account = ACCOUNT.with(|state| state.borrow_mut().take()).unwrap();
        for (stage, (calls, ms)) in account.rows {
            eprintln!("feature.stage id={id} stage={stage} calls={calls} ms={ms:.6}");
        }
        eprintln!("feature.stage id={id} stage=dispatch.other calls=1 ms={:.6}", total_ms - account.recorded);
        eprintln!("feature.total id={id} ms={total_ms:.6}");
    }
}
impl Drop for Feature {
    fn drop(&mut self) {
        if self.0 { ACCOUNT.with(|state| *state.borrow_mut() = None); }
    }
}
