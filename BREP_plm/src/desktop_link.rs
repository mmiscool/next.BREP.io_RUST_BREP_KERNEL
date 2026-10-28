//! Signing the desktop app in through the user's browser: a device
//! authorization flow in the spirit of RFC 8628.
//!
//! 1. The app registers ([`DesktopLinks::register`]) with no credential and
//!    gets three things back: a **link token** (the only one that goes in a
//!    URL), a **poll secret** (never in a URL), and a short **code** it shows.
//! 2. The app opens `/desktop-link?token=<link token>` in the default browser.
//!    Signed out, the page signs in first and comes back to the link.
//! 3. A signed-in person sees what asked (host, OS, address) and the code, and
//!    clicks Approve or Deny ([`DesktopLinks::decide`]). Nothing is approved by
//!    opening the page: a link someone else registered and sent to a victim
//!    must not hand the sender the victim's account.
//! 4. The app polls with its secret ([`DesktopLinks::poll`]); once approved,
//!    the handler mints an API token for the approver and hands it over, once.
//!
//! A decision kills the link token, so a URL read later decides nothing; the
//! link token never polls, so a URL read early collects nothing. Pending links
//! live in memory for [`TTL`] seconds: a restart forgets them, which costs a
//! user one more click. Only SHA-256 hashes of the token and the secret are
//! kept, and they are compared in constant time.

use std::collections::HashMap;
use std::sync::Mutex;

use serde::{Deserialize, Serialize};

use crate::model::{Actor, Timestamp};
use crate::security::{same_secret, token_hash};
use crate::Error;

/// How long a link waits for a decision and then for the app to collect it.
pub const TTL: u64 = 10 * 60;
/// How often the app should poll, in seconds.
pub const INTERVAL: u64 = 2;
/// Registrations one address may make in a minute.
pub const REGISTER_PER_MINUTE: usize = 10;
/// Links pending at once, all addresses together.
const MAX_PENDING: usize = 1000;
/// The longest host / OS / version text kept, in characters.
const MAX_TEXT: usize = 64;
/// The code's alphabet: no 0/O, 1/I/L, so it reads aloud and copies right.
const CODE_ALPHABET: &[u8] = b"ABCDEFGHJKMNPQRSTUVWXYZ23456789";

/// What the app says about itself, shown to the person deciding.
#[derive(Debug, Clone, Default, Deserialize, Serialize)]
pub struct Asker {
    #[serde(default)]
    pub host: String,
    #[serde(default)]
    pub os: String,
    #[serde(default)]
    pub app_version: String,
}

impl Asker {
    /// Trimmed, one line, at most [`MAX_TEXT`] characters each.
    fn tidy(self) -> Self {
        let tidy = |text: String| -> String {
            text.chars().map(|c| if c.is_control() { ' ' } else { c }).collect::<String>().trim().chars().take(MAX_TEXT).collect()
        };
        Self { host: tidy(self.host), os: tidy(self.os), app_version: tidy(self.app_version) }
    }
}

/// A registration's answer: the secrets exist only here, never in the store.
#[derive(Debug, Clone, Serialize)]
pub struct Registered {
    pub link_token: String,
    pub poll_secret: String,
    pub code: String,
    /// The link page, relative to the server: the app joins it to its URL.
    pub path: String,
    pub expires_in: u64,
    pub interval: u64,
}

/// A pending link as the confirm page sees it.
#[derive(Debug, Clone, Serialize)]
pub struct LinkView {
    pub code: String,
    pub host: String,
    pub os: String,
    pub app_version: String,
    /// The address the registration came from.
    pub ip: String,
    pub expires_in: u64,
}

/// What a poll finds.
#[derive(Debug, Clone)]
pub enum Poll {
    Pending,
    Denied,
    /// Approved by this person; the link is gone now, the handler mints.
    Approved { by: Actor, asker: Asker },
    /// No link with that secret: never was, expired, or already collected.
    Unknown,
}

#[derive(Clone)]
enum Decision {
    Pending,
    Denied,
    Approved(Actor),
}

#[derive(Clone)]
struct Link {
    /// Hash of the link token; empty once decided, so it matches nothing.
    token_hash: String,
    secret_hash: String,
    code: String,
    asker: Asker,
    ip: String,
    expires_at: Timestamp,
    decision: Decision,
}

/// Every pending link, and the registration counts per address.
#[derive(Default)]
pub struct DesktopLinks {
    links: Mutex<Vec<Link>>,
    registrations: Mutex<HashMap<String, Vec<Timestamp>>>,
}

impl std::fmt::Debug for DesktopLinks {
    /// Counts only: a link's hashes are no one's business in a log.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let pending = self.links.lock().map(|l| l.len()).unwrap_or(0);
        f.debug_struct("DesktopLinks").field("pending", &pending).finish()
    }
}

impl DesktopLinks {
    fn links(&self) -> std::sync::MutexGuard<'_, Vec<Link>> {
        self.links.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Register a link for `asker`, from `ip`. Refused with `429` past
    /// [`REGISTER_PER_MINUTE`] from one address, or `MAX_PENDING` in all.
    pub fn register(&self, asker: Asker, ip: Option<String>, now: Timestamp) -> Result<Registered, Error> {
        let ip = ip.unwrap_or_default();
        {
            let mut counts = self.registrations.lock().unwrap_or_else(|e| e.into_inner());
            counts.retain(|_, times| {
                times.retain(|&t| now.saturating_sub(t) < 60);
                !times.is_empty()
            });
            let times = counts.entry(ip.clone()).or_default();
            if times.len() >= REGISTER_PER_MINUTE {
                return Err(Error::too_many("too many sign-in requests from this address — wait a minute and try again"));
            }
            times.push(now);
        }
        let mut links = self.links();
        links.retain(|link| link.expires_at > now);
        if links.len() >= MAX_PENDING {
            return Err(Error::too_many("too many sign-in requests are waiting — try again in a few minutes"));
        }
        let link_token = crate::auth::random_token(32);
        let poll_secret = crate::auth::random_token(32);
        let code = new_code();
        links.push(Link {
            token_hash: token_hash(&link_token),
            secret_hash: token_hash(&poll_secret),
            code: code.clone(),
            asker: asker.tidy(),
            ip,
            expires_at: now + TTL,
            decision: Decision::Pending,
        });
        Ok(Registered {
            path: format!("/desktop-link?token={link_token}"),
            link_token,
            poll_secret,
            code,
            expires_in: TTL,
            interval: INTERVAL,
        })
    }

    /// The undecided, unexpired link `token` opens, for the confirm page.
    pub fn inspect(&self, token: &str, now: Timestamp) -> Option<LinkView> {
        let hash = token_hash(token);
        let links = self.links();
        let link = links.iter().find(|l| l.expires_at > now && !l.token_hash.is_empty() && same_secret(&l.token_hash, &hash))?;
        Some(LinkView {
            code: link.code.clone(),
            host: link.asker.host.clone(),
            os: link.asker.os.clone(),
            app_version: link.asker.app_version.clone(),
            ip: link.ip.clone(),
            expires_in: link.expires_at - now,
        })
    }

    /// Decide the link `token` opens: approve as `by`, or deny with `None`.
    /// The token is dead afterwards either way. Returns the asker, for the
    /// audit entry.
    pub fn decide(&self, token: &str, by: Option<Actor>, now: Timestamp) -> Result<Asker, Error> {
        let hash = token_hash(token);
        let mut links = self.links();
        let link = links
            .iter_mut()
            .find(|l| l.expires_at > now && !l.token_hash.is_empty() && same_secret(&l.token_hash, &hash))
            .ok_or_else(gone)?;
        link.token_hash.clear();
        link.decision = match by {
            Some(actor) => Decision::Approved(actor),
            None => Decision::Denied,
        };
        Ok(link.asker.clone())
    }

    /// What the app holding `secret` finds. An approval is handed out once:
    /// the link is removed as it is read.
    pub fn poll(&self, secret: &str, now: Timestamp) -> Poll {
        let hash = token_hash(secret);
        let mut links = self.links();
        links.retain(|link| link.expires_at > now);
        let Some(at) = links.iter().position(|l| same_secret(&l.secret_hash, &hash)) else {
            return Poll::Unknown;
        };
        match &links[at].decision {
            Decision::Pending => Poll::Pending,
            Decision::Denied => Poll::Denied,
            Decision::Approved(_) => {
                let link = links.remove(at);
                let Decision::Approved(by) = link.decision else { unreachable!() };
                Poll::Approved { by, asker: link.asker }
            }
        }
    }
}

/// The one refusal for a link token that opens nothing: never existed,
/// expired, or already decided. Which one is no business of a URL holder.
pub fn gone() -> Error {
    Error {
        status: axum::http::StatusCode::NOT_FOUND,
        message: "this sign-in link has expired or was already used — start the sign-in again from BREP".into(),
    }
}

fn new_code() -> String {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    (0..6).map(|_| CODE_ALPHABET[rng.gen_range(0..CODE_ALPHABET.len())] as char).collect()
}
