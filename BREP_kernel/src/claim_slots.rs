//! Work handed to idle pool threads that the caller never waits for.
//!
//! [`ClaimSlots::spawn`] puts one job per slot on the rayon pool (the last
//! slot first, since idle threads steal the oldest spawn first and the caller
//! walks the slots from the front). The caller then walks its own loop and,
//! at each slot, [`ClaimSlots::claim`]s it: a job a helper finished hands back
//! its result; a job not started or still running is taken from the helpers
//! and the caller does that work itself, as it would without helpers. Nothing
//! blocks, so a caller whose helpers are all busy elsewhere runs exactly as
//! fast as before, and a thread waiting on nothing never picks up unrelated
//! pool work in the middle of the caller's loop.
//!
//! A helper's panic makes its slot a miss (the caller repeats the work and
//! meets the panic itself). Dropping the slots cancels the jobs not started.
//! Without the `parallel` feature nothing is spawned and every claim misses.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

#[cfg_attr(not(feature = "parallel"), allow(dead_code))]
enum Slot<J, R> {
    Unstarted(J),
    Running,
    Done(R),
    Claimed,
}

pub(crate) struct ClaimSlots<J, R> {
    slots: Arc<Vec<Mutex<Slot<J, R>>>>,
    cancelled: Arc<AtomicBool>,
    /// Claims answered by a helper / taken back by the caller.
    pub(crate) taken: usize,
    pub(crate) claimed: usize,
}

impl<J, R> Drop for ClaimSlots<J, R> {
    fn drop(&mut self) {
        self.cancelled.store(true, Ordering::Relaxed);
    }
}

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

impl<J: Send + 'static, R: Send + 'static> ClaimSlots<J, R> {
    /// One slot per job; `work` runs on whichever pool thread gets there first.
    /// `None` without the `parallel` feature or on a one-thread pool.
    pub(crate) fn spawn(jobs: Vec<J>, work: fn(J) -> R) -> Option<Self> {
        #[cfg(not(feature = "parallel"))]
        {
            let _ = (jobs, work);
            None
        }
        #[cfg(feature = "parallel")]
        {
            if rayon::current_num_threads() < 2 {
                return None;
            }
            let count = jobs.len();
            let slots: Arc<Vec<Mutex<Slot<J, R>>>> =
                Arc::new(jobs.into_iter().map(|job| Mutex::new(Slot::Unstarted(job))).collect());
            let cancelled = Arc::new(AtomicBool::new(false));
            for index in (0..count).rev() {
                let (slots, cancelled) = (Arc::clone(&slots), Arc::clone(&cancelled));
                rayon::spawn(move || {
                    if cancelled.load(Ordering::Relaxed) {
                        return;
                    }
                    let job = {
                        let mut slot = lock(&slots[index]);
                        match std::mem::replace(&mut *slot, Slot::Running) {
                            Slot::Unstarted(job) => job,
                            other => {
                                *slot = other;
                                return;
                            }
                        }
                    };
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| work(job)));
                    let mut slot = lock(&slots[index]);
                    if matches!(*slot, Slot::Running) {
                        *slot = match result {
                            Ok(result) => Slot::Done(result),
                            Err(_) => Slot::Claimed,
                        };
                    }
                });
            }
            Some(Self { slots, cancelled, taken: 0, claimed: 0 })
        }
    }
}

impl<J, R> ClaimSlots<J, R> {
    pub(crate) fn len(&self) -> usize {
        self.slots.len()
    }

    /// Slot `index`'s result if a helper finished it; otherwise the slot is
    /// taken from the helpers and `None` says to do the work here. Never waits.
    pub(crate) fn claim(&mut self, index: usize) -> Option<R> {
        let result = match std::mem::replace(&mut *lock(&self.slots[index]), Slot::Claimed) {
            Slot::Done(result) => Some(result),
            _ => None,
        };
        if result.is_some() {
            self.taken += 1;
        } else {
            self.claimed += 1;
        }
        result
    }

}
