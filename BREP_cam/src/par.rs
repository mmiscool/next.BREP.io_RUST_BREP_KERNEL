//! Ordered parallel maps for independent contact solves (feature `parallel`).
//!
//! Items are processed in chunks on rayon's pool, each with its own runner on
//! the caller's cancel token (a worker sees a cancellation at its next
//! checkpoint), and the results are returned in item order — never completion
//! order — so a parallel build's output is byte-identical to the serial one.
//! Between chunks the calling thread publishes progress and checks the token.
//! Nothing here links or touches thread-local state: callers parallelise pure
//! contact solves only and do everything else on the calling thread.
#![cfg(feature = "parallel")]

use crate::cancel::{Cancelled, NullSink, Runner, WorkBudget};
use rayon::prelude::*;

/// The pool's width.
pub(crate) fn threads() -> usize {
    rayon::current_num_threads().max(1)
}

/// `f` over `items` in parallel; results in item order.
pub(crate) fn map_ordered<T: Sync, R: Send>(items: &[T], runner: &mut Runner<'_>, phase: &str, f: impl Fn(&T, &mut Runner<'_>) -> R + Sync) -> Result<Vec<R>, Cancelled> {
    let chunk = (rayon::current_num_threads() * 4).max(1);
    let mut out = Vec::with_capacity(items.len());
    for group in items.chunks(chunk) {
        runner.token().check()?;
        let token = runner.token().clone();
        let part: Vec<R> = group
            .par_iter()
            .map(|item| {
                let mut sink = NullSink;
                let mut worker = Runner::new(token.clone(), &mut sink, WorkBudget::default());
                f(item, &mut worker)
            })
            .collect();
        out.extend(part);
        runner.stage(phase, phase, out.len() as u64, items.len() as u64)?;
    }
    Ok(out)
}
