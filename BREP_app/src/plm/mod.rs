//! The PLM client: where the server is, how a request reaches it, and — in
//! `client` — sign-in, the refusals, the change feed and the version check.
//! Nothing here runs unless a server is configured; the file-based app never
//! constructs any of it.

pub mod backend;
pub mod bom;
pub mod client;
pub mod connection;
pub mod config;
pub mod transport;
// The pure halves of S5 (uses lists) and S13 (adoption): no transport, no
// store writes — the publisher and the importer drive them.
pub mod adoption;
pub mod kicad;
pub mod uses;
/// The bake worker (S10): open a queued member, build it, report.
pub mod bake;
/// Families and templates on a PLM store (S9): Generate, members, spin-out.
pub mod family;
/// Revision thumbnails (P9): the picture a save makes and uploads.
pub mod thumbnail;
pub mod thumbnail_view;
#[cfg(not(target_arch = "wasm32"))]
pub mod native;

/// One HTTP exchange with the PLM, whatever carries it. `path` is relative to
/// the server's base URL (`"/api/store/index"`); the client, not the
/// transport, adds the credential headers.
pub struct PlmRequest {
    pub method: &'static str,
    pub path: String,
    pub headers: Vec<(String, String)>,
    pub body: Vec<u8>,
}

/// The server's answer, whatever its status.
pub struct PlmResponse {
    pub status: u16,
    pub headers: Vec<(String, String)>,
    pub body: Vec<u8>,
}

/// Deliberately not `Send`, like the store seam's `BackendFuture`: every host
/// is single-threaded (the browser main thread; the native frame loop).
pub type PlmFuture<T> = std::pin::Pin<Box<dyn std::future::Future<Output = Result<T, String>>>>;

/// Carries a [`PlmRequest`] to the server. `Err` means the request never got
/// an answer (network, DNS, refused connection). Every HTTP status — 204, 4xx
/// and 5xx included — is `Ok`; the client maps them.
pub trait PlmTransport {
    fn send(&self, request: PlmRequest) -> PlmFuture<PlmResponse>;
}

pub mod identity;
