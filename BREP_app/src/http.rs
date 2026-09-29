//! HTTP downloads delivered to the frame loop through a channel.

use eframe::egui;
use std::sync::mpsc::{channel, Receiver};

/// Fetch text, falling back to lossy UTF-8 when the response has no text view.
pub(crate) fn fetch_text(ctx: &egui::Context, url: String) -> Receiver<Result<String, String>> {
    fetch(ctx, url, |response| {
        response
            .text()
            .map(str::to_owned)
            .unwrap_or_else(|| String::from_utf8_lossy(&response.bytes).into_owned())
    })
}

/// Fetch raw bytes, for example a thumbnail PNG.
pub(crate) fn fetch_bytes(ctx: &egui::Context, url: String) -> Receiver<Result<Vec<u8>, String>> {
    fetch(ctx, url, |response| response.bytes)
}

/// Wake the UI after delivering either a decoded response or an HTTP error.
fn fetch<T: Send + 'static>(
    ctx: &egui::Context,
    url: String,
    decode: impl FnOnce(ehttp::Response) -> T + Send + 'static,
) -> Receiver<Result<T, String>> {
    let (tx, rx) = channel();
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
    rx
}
