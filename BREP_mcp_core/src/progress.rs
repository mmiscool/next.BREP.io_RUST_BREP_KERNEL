//! Request-scoped waiting progress; never execution ownership or completion.
use std::{future::Future, time::Duration};
use rmcp::{model::ProgressNotificationParam, service::RequestContext, RoleServer};
use serde_json::Value;

const PERIOD: Duration = Duration::from_secs(1);
const MAX_UPDATES: usize = 600;

pub(crate) fn waits(name: &str, args: &Value) -> bool {
    match name {
        "run_script" => args.get("wait_ms").and_then(Value::as_u64).unwrap_or(60_000) > 0,
        "script_result" => args.get("wait_ms").and_then(Value::as_u64).unwrap_or(0) > 0,
        _ => false,
    }
}

/// Await the original future even if telemetry delivery fails or the request's
/// cancellation token fires. Accepted execution cancellation is an explicit
/// application operation, not a side effect of a notification channel closing.
pub(crate) async fn waiting<F: Future>(context: &RequestContext<RoleServer>, enabled: bool, future: F) -> F::Output {
    let Some(token) = context.meta.get_progress_token().filter(|_| enabled) else {
        return future.await;
    };
    drive(future, context.ct.cancelled(), PERIOD, MAX_UPDATES, |elapsed| {
        let notification = ProgressNotificationParam::new(token.clone(), elapsed)
            .with_message("Waiting for execution; progress is elapsed request-wait seconds, not completion percentage.");
        async move {
            matches!(tokio::time::timeout(PERIOD, context.peer.notify_progress(notification)).await, Ok(Ok(())))
        }
    }).await
}

async fn drive<F, C, S, N>(future: F, cancelled: C, period: Duration, maximum: usize, mut send: S) -> F::Output
where F: Future, C: Future<Output = ()>, S: FnMut(f64) -> N, N: Future<Output = bool> {
    tokio::pin!(future, cancelled);
    let start = tokio::time::Instant::now();
    let mut tick = tokio::time::interval_at(start + period, period);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    for _ in 0..maximum {
        tokio::select! {
            biased;
            value = &mut future => return value,
            _ = &mut cancelled => return future.await,
            _ = tick.tick() => {}
        }
        // Poll the operation while delivery is pending: a slow receiver cannot
        // delay its result. Dropping this future drops all notification work.
        tokio::select! {
            biased;
            value = &mut future => return value,
            _ = &mut cancelled => return future.await,
            delivered = send(start.elapsed().as_secs_f64()) => {
                if !delivered { return future.await; }
            }
        }
    }
    future.await
}

