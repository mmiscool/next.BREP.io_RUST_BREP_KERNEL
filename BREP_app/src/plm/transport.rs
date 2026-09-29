//! The ehttp transport, both targets (plan S1).
//!
//! ehttp is already the app's HTTP client (`http.rs`): `ureq` on a worker
//! thread natively, the browser's `fetch` on wasm. The browser sends the PLM's
//! session cookie by itself on a same-origin page (D6), so the web lane needs
//! no cookie handling here. On native nothing keeps cookies, which is why the
//! native app signs in with a token.
//!
//! ehttp answers through a callback on its own thread (natively). The future
//! returned here is completed from that callback and wakes whatever waker last
//! polled it, so any executor that honours wakers drives it: S2's frame-driven
//! native queue, `wasm_bindgen_futures` on the web, a test's `block_on`.
use super::{PlmFuture, PlmRequest, PlmResponse, PlmTransport};
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Waker};

/// The PLM at `base` (`http://host:port`, no trailing slash) over ehttp.
pub struct EhttpTransport {
    pub base: String,
}

impl EhttpTransport {
    pub fn new(base: impl Into<String>) -> Self {
        Self { base: base.into().trim_end_matches('/').to_string() }
    }
}

impl PlmTransport for EhttpTransport {
    fn send(&self, request: PlmRequest) -> PlmFuture<PlmResponse> {
        let slot = Arc::new(Mutex::new(Slot::default()));
        let outgoing = ehttp::Request {
            method: request.method.to_string(),
            url: format!("{}{}", self.base, request.path),
            body: request.body,
            headers: ehttp::Headers { headers: request.headers },
            ..ehttp::Request::get("")
        };
        let filled = slot.clone();
        ehttp::fetch(outgoing, move |result| {
            let answer = result.map(|response| PlmResponse {
                status: response.status,
                headers: response.headers.headers,
                body: response.bytes,
            });
            let waker = {
                let mut slot = filled.lock().unwrap();
                slot.answer = Some(answer);
                slot.waker.take()
            };
            if let Some(waker) = waker {
                waker.wake();
            }
        });
        Box::pin(Answer { slot })
    }
}

#[derive(Default)]
struct Slot {
    answer: Option<Result<PlmResponse, String>>,
    waker: Option<Waker>,
}

/// Resolves when ehttp's callback has filled the slot.
struct Answer {
    slot: Arc<Mutex<Slot>>,
}

impl Future for Answer {
    type Output = Result<PlmResponse, String>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let mut slot = self.slot.lock().unwrap();
        match slot.answer.take() {
            Some(answer) => Poll::Ready(answer),
            None => {
                slot.waker = Some(cx.waker().clone());
                Poll::Pending
            }
        }
    }
}
