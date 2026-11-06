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

const WAITING: &str = "Waiting for execution; progress is elapsed request-wait seconds, not completion percentage.";

/// Await the original future even if telemetry delivery fails or the request's
/// cancellation token fires. Accepted execution cancellation is an explicit
/// application operation, not a side effect of a notification channel closing.
///
/// Each update's message also says what the app is doing (`frame_info`: the
/// feature the run is on, the work still pending), read from the live session
/// in `slot` when there is one. That read is telemetry only: bounded, and
/// never part of the wait.
pub(crate) async fn waiting<F: Future>(context: &RequestContext<RoleServer>, enabled: bool, slot: &crate::tools::app::SessionSlot, future: F) -> F::Output {
    let Some(token) = context.meta.get_progress_token().filter(|_| enabled) else {
        return future.await;
    };
    drive(future, context.ct.cancelled(), PERIOD, MAX_UPDATES, |elapsed| {
        let token = token.clone();
        let slot = slot.clone();
        async move {
            let message = match app_status(&slot).await {
                Some(status) => format!("{WAITING} App: {status}."),
                None => WAITING.to_string(),
            };
            let notification = ProgressNotificationParam::new(token, elapsed).with_message(message);
            matches!(tokio::time::timeout(PERIOD, context.peer.notify_progress(notification)).await, Ok(Ok(())))
        }
    }).await
}

/// What the live session's app is doing now, in words, if it answers soon.
async fn app_status(slot: &crate::tools::app::SessionSlot) -> Option<String> {
    let session = slot.read().await.clone()?;
    let reply = tokio::time::timeout(Duration::from_millis(500), session.host.call("frame_info", serde_json::json!({}))).await.ok()?.ok()?;
    reply.result.as_ref().map(describe_status)
}

/// `frame_info` as one clause: idle, or the run's feature and what is pending.
fn describe_status(info: &Value) -> String {
    if info["idle"] == true {
        return "idle".into();
    }
    let mut parts = Vec::new();
    if let Some(p) = info["progress"].as_object() {
        let index = p.get("index").and_then(Value::as_u64).unwrap_or(0);
        let total = p.get("total").and_then(Value::as_u64).unwrap_or(0);
        let kind = p.get("feature_type").and_then(Value::as_str).unwrap_or("");
        let id = p.get("feature_id").and_then(Value::as_str).unwrap_or("");
        parts.push(format!("running feature {} of {total} ({kind} {id})", index + 1));
    }
    let pending: Vec<&str> = info["pending"]
        .as_object()
        .map(|o| o.iter().filter(|(_, v)| **v == true).map(|(k, _)| k.as_str()).collect())
        .unwrap_or_default();
    if !pending.is_empty() {
        parts.push(format!("pending: {}", pending.join(", ")));
    }
    if parts.is_empty() { "working".into() } else { parts.join("; ") }
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

