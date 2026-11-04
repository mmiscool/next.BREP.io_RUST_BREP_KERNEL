//! HTTP downloads delivered to the frame loop through a channel.

use eframe::egui;
use std::sync::mpsc::{channel, Receiver, Sender};

/// `text` percent-encoded as one URL query value or path segment: RFC 3986's
/// unreserved characters (`A-Z a-z 0-9 - . _ ~`) kept, every other byte `%XX`.
/// Not [`crate::plm::identity::segment`], which encodes `.` and `~` too
/// because a PLM key must not spell `..`; use that one for store keys.
pub(crate) fn query_component(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for byte in text.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'.' | b'_' | b'~' => out.push(byte as char),
            _ => out.push_str(&format!("%{byte:02X}")),
        }
    }
    out
}

/// Fetch text, falling back to lossy UTF-8 when the response has no text view.
pub(crate) fn fetch_text(ctx: &egui::Context, url: String) -> Receiver<Result<String, String>> {
    let (tx, rx) = channel();
    fetch_text_into(ctx, url, tx);
    rx
}

/// [`fetch_text`]'s request, answering into `tx` (for a caller that made its
/// channel before it knew whether to fetch: `offsite::fetch_text`).
pub(crate) fn fetch_text_into(ctx: &egui::Context, url: String, tx: Sender<Result<String, String>>) {
    fetch_into(ctx, url, text_of, tx);
}

fn text_of(response: ehttp::Response) -> String {
    response
        .text()
        .map(str::to_owned)
        .unwrap_or_else(|| String::from_utf8_lossy(&response.bytes).into_owned())
}

/// Fetch raw bytes, for example a thumbnail PNG.
pub(crate) fn fetch_bytes(ctx: &egui::Context, url: String) -> Receiver<Result<Vec<u8>, String>> {
    fetch(ctx, url, |response| response.bytes)
}

fn fetch<T: Send + 'static>(
    ctx: &egui::Context,
    url: String,
    decode: impl FnOnce(ehttp::Response) -> T + Send + 'static,
) -> Receiver<Result<T, String>> {
    let (tx, rx) = channel();
    fetch_into(ctx, url, decode, tx);
    rx
}

/// Wake the UI after delivering either a decoded response or an HTTP error.
fn fetch_into<T: Send + 'static>(
    ctx: &egui::Context,
    url: String,
    decode: impl FnOnce(ehttp::Response) -> T + Send + 'static,
    tx: Sender<Result<T, String>>,
) {
    let ctx = ctx.clone();
    ehttp::fetch(ehttp::Request::get(url), move |result| {
        let out = match result {
            Ok(response) if response.ok => Ok(decode(response)),
            Ok(response) => Err(format!("HTTP {} {}", response.status, response.status_text)),
            Err(error) => Err(error),
        };
        let _ = tx.send(out);
        ctx.request_repaint();
    });
}

