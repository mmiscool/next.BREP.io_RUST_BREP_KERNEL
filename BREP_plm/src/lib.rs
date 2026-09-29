//! The BREP PLM server.
//!
//! Parts have a permanent identity and a configurable number; revisions carry
//! the lifecycle and the CAD document; a draft is edited under an explicit
//! checkout; a released revision is immutable. The HTTP surface is in
//! [`api`], the rules are in [`db`] and [`lifecycle`], and the records are in
//! [`model`].
//!
//! # What is built
//!
//! The spine, plus the administrator's scripting layer ([`scripting`]): part
//! types number by counter, free text, pattern or script
//! ([`model::NumberMode`]); revision labels are free text; every part carries a
//! fixed [`model::DocumentClass`]; and four hooks — part number, revision
//! label, before release, after release — let an organization override the
//! defaults with JavaScript files in a directory it can keep in git.
//!
//! The catalog ([`catalog`]): a tree of categories, each adding typed
//! attributes to what it inherits from its parent; every part sits in one
//! category, carries free tags, and holds attribute values that are
//! type-checked when written and must be complete before a release.
//!
//! Sourcing ([`sourcing`]): manufacturers and suppliers, and on each part its
//! manufacturer parts with their supplier offers. Administrator settings
//! ([`model::Settings`]) decide whether a part may have several revisions in
//! work and whether a released part's catalog values lock.
//!
//! Families and templates ([`family`], [`seed`]).
//!
//! Assembly structure ([`bom`]): each revision's "uses" list — the one the CAD
//! app will publish on save — and the indented and flat BOM, the diff between
//! two revisions' lists, where-used, and an administrator's release gate on
//! unreleased children, all computed from it.
//!
//! Security ([`security`], `api::credentials`, [`tls`]): CSRF tokens on browser
//! sessions, scoped API tokens for everything else, a sign-in throttle, session
//! limits, password change and sign-out everywhere, response headers, `Secure`
//! cookies, optional native TLS, and a switch for the in-browser script editor.
//!
//! The audit log ([`audit`]): every change the store commits, found by diffing
//! what the change touched before and after, attributed to the request's
//! actor, committed with the change and read through `/api/audit`.
//!
//! Storage ([`sql`], [`table`]): the metadata in one SQLite file, each change
//! one transaction with its audit events; a trigram full-text index over what
//! a person searches a part by; paged part listings; and a one-time import
//! from the JSON-file store earlier versions wrote.
//!
//! Review and approval ([`review`]): submitting a revision opens a review
//! round judged by the settings' rule or the nearest override, reviewers
//! approve or reject, a rejection sends the revision back to Draft, release
//! waits for the rule, and every revision carries a threaded discussion. One
//! process holds a data directory at a time ([`dirlock`]).
//!
//! Change orders ([`eco`]): a set of revisions to release or obsolete, with
//! its own numbering, reviewed as a whole and released as a whole — every
//! item's gates and hooks, then all of them applied in one transaction, or
//! none.
//!
//! Library import is designed but not built — where the design named a field it
//! will need ([`model::Origin`], `external_ref`), the field is here and inert.
//!
//! # The document API
//!
//! `/api/store/*` is deliberately shaped to what the CAD app's `StoreBackend`
//! requires of a remote backend: a metadata-only index (so a client never
//! hydrates the corpus), a per-key read, a write, a delete, and a monotonic
//! change sequence with a "what moved" feed. The CAD app is not touched by
//! this crate; this is the surface it will adopt.

pub mod api;
pub mod attach;
pub mod audit;
pub mod auth;
pub mod backup;
pub mod bom;
pub mod cad;
pub mod catalog;
pub mod db;
pub mod dirlock;
pub mod eco;
pub mod family;
pub mod journal;
pub mod lifecycle;
pub mod replace;
pub mod review;
pub mod model;
pub mod scripting;
pub mod security;
pub mod seed;
pub mod sourcing;
pub mod sql;
pub mod table;
pub mod thumbnail;
pub mod tls;
pub mod version;
pub mod workspace;

use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::Json;

/// A refusal, with the status the API answers it with and the message a person
/// reads. Every refusal in this server carries a reason — a bare status code
/// tells the user nothing about what to do next.
#[derive(Debug, Clone)]
pub struct Error {
    pub status: StatusCode,
    pub message: String,
}

impl Error {
    pub fn bad_request(message: impl Into<String>) -> Self {
        Self { status: StatusCode::BAD_REQUEST, message: message.into() }
    }

    pub fn unauthorized(message: impl Into<String>) -> Self {
        Self { status: StatusCode::UNAUTHORIZED, message: message.into() }
    }

    pub fn forbidden(message: impl Into<String>) -> Self {
        Self { status: StatusCode::FORBIDDEN, message: message.into() }
    }

    pub fn not_found(what: impl std::fmt::Display) -> Self {
        Self { status: StatusCode::NOT_FOUND, message: format!("no such {what}") }
    }

    /// A rule refused it — a lock held elsewhere, an immutable revision, an
    /// exhausted number scheme. 409 is the status the CAD app's write-behind
    /// lane will surface as a notice.
    pub fn conflict(message: impl Into<String>) -> Self {
        Self { status: StatusCode::CONFLICT, message: message.into() }
    }

    /// Too many failed sign-ins: the caller must wait.
    pub fn too_many(message: impl Into<String>) -> Self {
        Self { status: StatusCode::TOO_MANY_REQUESTS, message: message.into() }
    }

    pub fn internal(error: impl std::fmt::Display) -> Self {
        Self {
            status: StatusCode::INTERNAL_SERVER_ERROR,
            message: error.to_string(),
        }
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.status, self.message)
    }
}

impl std::error::Error for Error {}

impl IntoResponse for Error {
    fn into_response(self) -> Response {
        (
            self.status,
            Json(serde_json::json!({ "error": self.message })),
        )
            .into_response()
    }
}
