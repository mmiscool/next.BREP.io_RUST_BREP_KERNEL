//! The PLM tab's state and actions (plan S1 part 3): sign in with a pasted
//! token or a password, check the connection, forget the machine's sign-in.
//!
//! Every action runs on the host's executor and lands in a slot the tab polls
//! each frame; the tab drives the queue itself while an action is in flight,
//! because a session on the file stores (no PLM configured yet, or a refused
//! boot) has no store draining it.
//!
//! What an action leaves behind:
//!
//! - **Native.** A sign-in writes the token to `plm-token` beside `plm.json`
//!   (D5) and offers Connect now, which switches the running session when it
//!   is in a fresh start's state; otherwise the next start connects: the store
//!   is chosen before the first frame, and settings, the dock and recovery all
//!   read it there.
//!   Forget removes both files; the next start is the file app.
//! - **Web.** The page is served by the PLM (D6), so a sign-in is the PLM's own
//!   session cookie, and the page reloads onto it. Forget signs the session
//!   out and reloads.
use super::client::{DesktopLink, LinkPoll, Me, PlmClient, PlmError};
use super::transport::EhttpTransport;
use super::PlmTransport;
use std::cell::RefCell;
use std::rc::Rc;

/// What the tab knows about the PLM, before any action.
#[derive(Clone, Debug, PartialEq)]
pub enum Status {
    /// The session's store is the PLM.
    Connected { url: String, username: String },
    /// A PLM is configured (native) or serving this page (web), but this
    /// session is not on it. `reason` is why, when known: the boot's refusal,
    /// or a check's.
    NotConnected { url: String, reason: Option<String> },
}

/// The outcome of the last action, as the tab shows it.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    /// Done; the sentence says what happens next.
    Done(String),
    /// Refused or failed, in the server's (or the machine's) words.
    Failed(String),
}

/// A session's PLM, as its store reports it ([`crate::store::ModelStore::plm_session`]):
/// where it is, whether this session is on it, and where a sign-in is kept.
#[derive(Clone, Debug, PartialEq)]
pub struct Session {
    pub status: Status,
    pub keep: Keep,
}

/// The URL and user in the label `super::backend::compose` gives a PLM
/// store (`PLM <url> as <user>`), or `None` for any other store.
pub fn parse_label(label: &str) -> Option<(String, String)> {
    let rest = label.strip_prefix("PLM ")?;
    let (url, user) = rest.rsplit_once(" as ")?;
    Some((url.to_string(), user.to_string()))
}

/// Where the machine's sign-in lives: the config directory on native, the
/// PLM's own session cookie on the web.
#[derive(Clone, Debug, PartialEq)]
pub enum Keep {
    Files(std::path::PathBuf),
    BrowserSession,
}

pub struct PlmConnection {
    pub status: Status,
    keep: Keep,
    pub token_input: String,
    pub username: String,
    pub password: String,
    pending: Option<Rc<RefCell<Option<Outcome>>>>,
    pub last: Option<Outcome>,
    /// A web action finished that the page must reload onto.
    reload: bool,
    /// The server took this machine's sign-in (a sign-in or a check passed):
    /// the session may switch to it live (native Connect now).
    connectable: bool,
    /// Builds the transport for the server's URL: ehttp in the app, the
    /// in-process router in tests.
    transport: Rc<dyn Fn(&str) -> Rc<dyn PlmTransport>>,
    /// A browser sign-in's registration, in flight.
    registering: Option<Slot<Result<DesktopLink, PlmError>>>,
    /// A browser sign-in waiting for the person to approve it.
    pub link: Option<BrowserLink>,
    /// The link page a browser sign-in wants opened, for the tab to hand
    /// to `egui::Context::open_url` (the default browser natively; nothing in
    /// the headless automation host).
    open_request: Option<String>,
    /// Seconds on a monotonic clock: the poll schedule's time.
    clock: Rc<dyn Fn() -> f64>,
    /// A native sign-in that passed: `(username, token)`, for the shell to
    /// hand to the live session when its own token was revoked.
    fresh: Slot<(String, String)>,
    /// Forget ended this machine's sign-in on the server: the live session's
    /// token is gone, by the person's own choice.
    signed_out_here: Slot<()>,
}

type Slot<T> = Rc<RefCell<Option<T>>>;

/// What a browser sign-in found when it ended without a token.
const LINK_EXPIRED: &str = "the browser sign-in expired before it was approved — start it again";

/// The longest wait between polls after failures, in seconds.
const MAX_BACKOFF: f64 = 30.0;

/// A browser sign-in in progress (native): the code the tab shows, the link
/// the browser was sent to, and the poll schedule.
pub struct BrowserLink {
    pub code: String,
    pub url: String,
    /// The last poll's trouble, while it keeps trying.
    pub note: Option<String>,
    secret: String,
    poller: LinkPoller,
    in_flight: Option<Slot<Result<LinkPoll, PlmError>>>,
}

/// When a browser sign-in asks next, when it stops, and what each answer
/// means. Time is seconds on any monotonic clock.
#[derive(Clone, Debug, PartialEq)]
pub struct LinkPoller {
    expires_at: f64,
    interval: f64,
    next_at: f64,
    failures: u32,
}

/// Where a browser sign-in stands after an answer.
#[derive(Clone, Debug, PartialEq)]
pub enum LinkStep {
    /// Keep waiting; `note` is why the last poll failed, if it did.
    Wait { note: Option<String> },
    /// Approved: the machine's new token.
    Approved(String),
    /// Over without a token; the sentence says why.
    Ended(String),
}

impl LinkPoller {
    pub fn new(now: f64, expires_in: u64, interval: u64) -> Self {
        let interval = interval.max(1) as f64;
        Self { expires_at: now + expires_in as f64, interval, next_at: now + interval, failures: 0 }
    }

    pub fn expired(&self, now: f64) -> bool {
        now >= self.expires_at
    }

    /// Time to ask again.
    pub fn due(&self, now: f64) -> bool {
        !self.expired(now) && now >= self.next_at
    }

    /// Seconds until the next poll (or the end), for the repaint timer.
    pub fn wait(&self, now: f64) -> f64 {
        (self.next_at.min(self.expires_at) - now).max(0.0)
    }

    /// Take one poll's answer. A pending link is asked again after the
    /// server's interval; a failure the server may recover from (no answer,
    /// a 5xx, the rate limit) backs off, doubling up to `MAX_BACKOFF`, and
    /// keeps trying until the link expires; anything else ends it.
    pub fn answer(&mut self, now: f64, answer: Result<LinkPoll, PlmError>) -> LinkStep {
        match answer {
            Ok(LinkPoll::Approved(token)) => LinkStep::Approved(token),
            Ok(LinkPoll::Denied) => LinkStep::Ended("the sign-in was denied in the browser".into()),
            Ok(LinkPoll::Gone) => LinkStep::Ended(LINK_EXPIRED.into()),
            Ok(LinkPoll::Pending) => {
                self.failures = 0;
                self.next_at = now + self.interval;
                self.wait_or_end(now, None)
            }
            Err(error) if passing(&error) => {
                self.failures += 1;
                let wait = (self.interval * 2f64.powi(self.failures as i32)).min(MAX_BACKOFF);
                self.next_at = now + wait;
                self.wait_or_end(now, Some(format!("{error} — trying again")))
            }
            Err(error) => LinkStep::Ended(error.to_string()),
        }
    }

    fn wait_or_end(&self, now: f64, note: Option<String>) -> LinkStep {
        if self.expired(now) {
            LinkStep::Ended(LINK_EXPIRED.into())
        } else {
            LinkStep::Wait { note }
        }
    }
}

/// A failure that may pass on its own: no answer, a server error, the rate
/// limit.
fn passing(error: &PlmError) -> bool {
    match error {
        PlmError::Unreachable(_) | PlmError::Throttled(_) => true,
        PlmError::Refused { status, .. } => *status >= 500,
        _ => false,
    }
}

impl PlmConnection {
    pub fn new(status: Status, keep: Keep) -> Self {
        Self {
            status,
            keep,
            token_input: String::new(),
            username: String::new(),
            password: String::new(),
            pending: None,
            last: None,
            reload: false,
            connectable: false,
            transport: Rc::new(|url: &str| Rc::new(EhttpTransport::new(url.to_string())) as Rc<dyn PlmTransport>),
            registering: None,
            link: None,
            open_request: None,
            clock: Rc::new(monotonic_seconds),
            fresh: Rc::default(),
            signed_out_here: Rc::default(),
        }
    }

    pub fn url(&self) -> &str {
        match &self.status {
            Status::Connected { url, .. } | Status::NotConnected { url, .. } => url,
        }
    }

    pub fn keep(&self) -> &Keep {
        &self.keep
    }

    /// An action is in flight.
    pub fn busy(&self) -> bool {
        self.pending.is_some()
    }

    /// Call once per frame: drive the queue while busy, collect a finished
    /// action, and run a browser sign-in's polls. Returns true when something
    /// changed.
    pub fn poll(&mut self) -> bool {
        let polling = self.link.as_ref().is_some_and(|l| l.in_flight.is_some());
        if self.pending.is_none() && self.registering.is_none() && !polling && self.link.is_none() {
            return false;
        }
        #[cfg(not(target_arch = "wasm32"))]
        if self.pending.is_some() || self.registering.is_some() || polling {
            super::native::run_pending();
        }
        let mut changed = self.poll_link();
        let Some(slot) = &self.pending else { return changed };
        let Some(outcome) = slot.borrow_mut().take() else { return changed };
        changed = true;
        self.pending = None;
        if let (Keep::BrowserSession, Outcome::Done(_)) = (&self.keep, &outcome) {
            self.reload = true;
        }
        // A native sign-in or check that passed: the server takes this
        // machine, so Connect now is offered. Anything that failed withdraws it.
        self.connectable = matches!(self.keep, Keep::Files(_)) && matches!(outcome, Outcome::Done(_));
        if let (Status::NotConnected { url, .. }, Outcome::Failed(why)) = (&self.status, &outcome) {
            self.status = Status::NotConnected { url: url.clone(), reason: Some(why.clone()) };
        }
        self.last = Some(outcome);
        changed
    }

    /// The browser sign-in's part of a frame: a registration that answered
    /// opens the browser; a poll that answered moves the schedule; a due poll
    /// is sent. An approval signs in with the new token exactly as a pasted
    /// one would (the version check first, then `plm-token`).
    fn poll_link(&mut self) -> bool {
        let mut changed = false;
        if let Some(answer) = self.registering.as_ref().and_then(|slot| slot.borrow_mut().take()) {
            self.registering = None;
            changed = true;
            match answer {
                Ok(link) => self.open_link(link),
                Err(error) => self.last = Some(Outcome::Failed(format!("could not start the browser sign-in: {error}"))),
            }
        }
        let now = (self.clock)();
        let Some(link) = self.link.as_mut() else { return changed };
        let answered = link.in_flight.as_ref().and_then(|slot| slot.borrow_mut().take());
        let step = match answered {
            Some(answer) => {
                link.in_flight = None;
                Some(link.poller.answer(now, answer))
            }
            None if link.in_flight.is_none() && link.poller.expired(now) => Some(LinkStep::Ended(LINK_EXPIRED.into())),
            None => None,
        };
        match step {
            Some(LinkStep::Wait { note }) => {
                link.note = note;
                changed = true;
            }
            Some(LinkStep::Approved(token)) => {
                self.link = None;
                self.token_input = token;
                self.use_token();
                return true;
            }
            Some(LinkStep::Ended(why)) => {
                self.link = None;
                self.connectable = false;
                self.last = Some(Outcome::Failed(why));
                return true;
            }
            None => {}
        }
        if link.in_flight.is_none() && link.poller.due(now) {
            let slot: Slot<Result<LinkPoll, PlmError>> = Rc::default();
            link.in_flight = Some(slot.clone());
            let secret = link.secret.clone();
            let client = self.client();
            spawn(async move {
                let answer = client.poll_desktop_link(&secret).await;
                *slot.borrow_mut() = Some(answer);
            });
            changed = true;
        }
        changed
    }

    fn open_link(&mut self, link: DesktopLink) {
        let url = format!("{}{}", self.url().trim_end_matches('/'), link.path);
        self.open_request = Some(url.clone());
        let poller = LinkPoller::new((self.clock)(), link.expires_in, link.interval);
        self.link = Some(BrowserLink { code: link.code, url, note: None, secret: link.poll_secret, poller, in_flight: None });
    }

    /// Forget ended this machine's sign-in on the server since the last call.
    pub fn take_signed_out_here(&mut self) -> bool {
        self.signed_out_here.borrow_mut().take().is_some()
    }

    /// A native sign-in that passed since the last call: `(username, token)`.
    pub fn take_fresh_token(&mut self) -> Option<(String, String)> {
        self.fresh.borrow_mut().take()
    }

    /// Take the link page a browser sign-in wants opened (once).
    pub fn take_open_request(&mut self) -> Option<String> {
        self.open_request.take()
    }

    /// Seconds until the browser sign-in wants a frame (its next poll), or
    /// `None` when none is running.
    pub fn link_wait(&self) -> Option<f64> {
        let link = self.link.as_ref()?;
        Some(if link.in_flight.is_some() { 0.25 } else { link.poller.wait((self.clock)()) })
    }

    /// Start a browser sign-in (native): register with the server, ask for
    /// the link page to be opened in the default browser
    /// ([`Self::take_open_request`]), and wait for the person to approve
    /// it there. The server mints this machine's token on approval.
    pub fn sign_in_with_browser(&mut self) {
        let client = self.client();
        let slot: Slot<Result<DesktopLink, PlmError>> = Rc::default();
        self.registering = Some(slot.clone());
        self.link = None;
        self.last = None;
        let (host, os) = machine();
        spawn(async move {
            let answer = client.register_desktop_link(&host, &os).await;
            *slot.borrow_mut() = Some(answer);
        });
    }

    /// Stop waiting for the browser. The link expires on the server by
    /// itself; an approval after this is never collected.
    pub fn cancel_link(&mut self) {
        self.registering = None;
        self.link = None;
        self.open_request = None;
    }

    /// A browser sign-in is starting or waiting.
    pub fn linking(&self) -> bool {
        self.registering.is_some() || self.link.is_some()
    }

    /// Connect now is offered: the server took this machine's sign-in.
    pub fn connectable(&self) -> bool {
        self.connectable
    }

    /// The app's live switch answered: success is the store's own news (the
    /// next status says Connected); a refusal is shown as this tab's outcome.
    pub fn reconnected(&mut self, outcome: Result<String, String>) {
        self.connectable = outcome.is_err() && self.connectable;
        self.last = Some(match outcome {
            Ok(text) => Outcome::Done(text),
            Err(why) => Outcome::Failed(why),
        });
    }

    /// The store's status moved (a live switch landed): take it.
    pub fn set_status(&mut self, status: Status) {
        if std::mem::discriminant(&status) != std::mem::discriminant(&self.status) {
            self.status = status;
            self.connectable = false;
        }
    }

    /// Take the "reload the page" request a web action left.
    pub fn take_reload(&mut self) -> bool {
        std::mem::take(&mut self.reload)
    }

    fn client(&self) -> Rc<PlmClient> {
        Rc::new(PlmClient::new((self.transport)(self.url())))
    }

    fn start(&mut self, work: impl std::future::Future<Output = Outcome> + 'static) {
        let slot: Rc<RefCell<Option<Outcome>>> = Rc::default();
        let into = slot.clone();
        self.pending = Some(slot);
        self.last = None;
        spawn(async move {
            let outcome = work.await;
            *into.borrow_mut() = Some(outcome);
        });
    }

    /// Sign in with the pasted token: the server must accept it and serve
    /// this app before it is kept.
    pub fn use_token(&mut self) {
        self.cancel_link();
        let token = self.token_input.trim().to_string();
        let (client, keep, fresh) = (self.client(), self.keep.clone(), self.fresh.clone());
        self.token_input.clear();
        self.start(async move {
            match client.sign_in_with_token(&token).await {
                Ok(me) => keep_token(&keep, &token, &me, &fresh),
                Err(e) => Outcome::Failed(e.to_string()),
            }
        });
    }

    /// Sign in with a username and password: a token for this machine
    /// (native), or the PLM's session (web). The password is not kept.
    pub fn use_password(&mut self) {
        self.cancel_link();
        let (username, password) = (self.username.trim().to_string(), std::mem::take(&mut self.password));
        let (client, keep, fresh) = (self.client(), self.keep.clone(), self.fresh.clone());
        self.start(async move {
            match keep {
                Keep::Files(_) => match client.sign_in_for_token(&username, &password, &token_name()).await {
                    Ok((me, token)) => keep_token(&keep, &token, &me, &fresh),
                    Err(e) => Outcome::Failed(e.to_string()),
                },
                Keep::BrowserSession => match client.sign_in_with_password(&username, &password).await {
                    Ok(me) => Outcome::Done(format!("signed in as {}; reloading onto the PLM", me.username)),
                    Err(e) => Outcome::Failed(e.to_string()),
                },
            }
        });
    }

    /// Ask the server again with what this machine holds: the answer (or the
    /// refusal, naming both versions) replaces the tab's reason.
    pub fn check(&mut self) {
        let (client, keep) = (self.client(), self.keep.clone());
        self.start(async move {
            let signed_in = match &keep {
                Keep::Files(dir) => match held_token(dir) {
                    Ok(Some(token)) => client.sign_in_with_token(&token).await,
                    Ok(None) => return Outcome::Failed("no token on this machine: paste one, or sign in with a password".into()),
                    Err(e) => return Outcome::Failed(e),
                },
                Keep::BrowserSession => client.sign_in_with_browser_session().await,
            };
            match signed_in {
                Ok(me) => Outcome::Done(format!("the server accepts this machine as {} and serves this app", me.username)),
                Err(e) => Outcome::Failed(e.to_string()),
            }
        });
    }

    /// Forget this machine's sign-in: native removes `plm.json` and
    /// `plm-token` (the next start is the file app); web signs the session
    /// out and reloads.
    pub fn forget(&mut self) {
        match self.keep.clone() {
            Keep::Files(dir) => {
                self.connectable = false;
                self.cancel_link();
                // A Linked desktop's token is revoked on the server first, so
                // the sign-in ends there too (it leaves the account page's
                // list). The files go whatever the server says: signing out
                // must work offline.
                let held = held_token(&dir).ok().flatten();
                let client = self.client();
                let signed_out_here = self.signed_out_here.clone();
                self.start(async move {
                    // `Ok(true)`: the server revoked it (or it was gone);
                    // `Ok(false)`: it is a token a person made or a password
                    // sign-in minted, which the server keeps.
                    let told: Result<bool, String> = match held {
                        Some(token) => {
                            client.replace_token(&token);
                            match client.raw("POST", "/api/desktop/sign-out", Some(b"{}".to_vec())).await {
                                Ok(answer) if answer.status == 200 => Ok(serde_json::from_slice::<serde_json::Value>(&answer.body)
                                    .is_ok_and(|v| v["revoked"] == true)),
                                Ok(answer) if answer.status == 401 => Ok(true),
                                Ok(answer) => Err(format!("the server answered {}", answer.status)),
                                Err(e) => Err(e.to_string()),
                            }
                        }
                        None => Ok(false),
                    };
                    if told == Ok(true) {
                        *signed_out_here.borrow_mut() = Some(());
                    }
                    match (forget_files(&dir), told) {
                        (Err(e), _) => Outcome::Failed(e),
                        (Ok(()), Ok(true)) => Outcome::Done("signed out: this machine's PLM sign-in is forgotten and ended on the server; the next start is the file app".into()),
                        (Ok(()), Ok(false)) => Outcome::Done("this machine's PLM sign-in is forgotten; the next start is the file app".into()),
                        (Ok(()), Err(why)) => Outcome::Done(format!(
                            "this machine's PLM sign-in is forgotten, but the server could not be told ({why}): revoke it under Linked desktops on your PLM account page"
                        )),
                    }
                });
            }
            Keep::BrowserSession => {
                let client = self.client();
                self.start(async move {
                    match client.raw("POST", "/api/logout", Some(b"{}".to_vec())).await {
                        Ok(_) => Outcome::Done("signed out; reloading".into()),
                        Err(e) => Outcome::Failed(e.to_string()),
                    }
                });
            }
        }
    }
}

/// The token's name on the server's token list: which app, on which machine.
fn token_name() -> String {
    #[cfg(not(target_arch = "wasm32"))]
    {
        format!("brep-app {}", machine().0)
    }
    #[cfg(target_arch = "wasm32")]
    {
        "brep-app web".into()
    }
}

/// This machine's name and OS, as a browser sign-in shows them to the person
/// approving it (and as the server names the token it mints).
fn machine() -> (String, String) {
    let named = |text: String| Some(text.trim().to_string()).filter(|t| !t.is_empty());
    let host = std::env::var("HOSTNAME")
        .ok()
        .and_then(named)
        .or_else(|| std::env::var("COMPUTERNAME").ok().and_then(named));
    #[cfg(not(target_arch = "wasm32"))]
    let host = host.or_else(|| std::fs::read_to_string("/etc/hostname").ok().and_then(named));
    (host.unwrap_or_else(|| "desktop".into()), os_name(std::env::consts::OS).to_string())
}

/// The system as people name it, for the account page's Linked desktops.
fn os_name(os: &str) -> &str {
    match os {
        "windows" => "Windows",
        "linux" => "Linux",
        "macos" => "macOS",
        other => other,
    }
}

/// Seconds since the first call: monotonic, on native and in a browser.
fn monotonic_seconds() -> f64 {
    thread_local! {
        static START: web_time::Instant = web_time::Instant::now();
    }
    START.with(|start| start.elapsed().as_secs_f64())
}

fn keep_token(keep: &Keep, token: &str, me: &Me, fresh: &Slot<(String, String)>) -> Outcome {
    match keep {
        Keep::Files(dir) => match write_token(dir, token) {
            Ok(()) => Outcome::Done({
                *fresh.borrow_mut() = Some((me.username.clone(), token.to_string()));
                format!(
                "signed in as {}; the token is kept on this machine. Connect now switches this session to the PLM; otherwise the next start does",
                me.username
            )
            }),
            Err(e) => Outcome::Failed(format!("signed in, but the token could not be kept: {e}")),
        },
        Keep::BrowserSession => Outcome::Done(format!("signed in as {}; reloading onto the PLM", me.username)),
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn write_token(dir: &std::path::Path, token: &str) -> Result<(), String> {
    super::config::write_token(dir, token)
}

#[cfg(target_arch = "wasm32")]
fn write_token(_: &std::path::Path, _: &str) -> Result<(), String> {
    Err("a browser keeps no token file".into())
}

#[cfg(not(target_arch = "wasm32"))]
fn held_token(dir: &std::path::Path) -> Result<Option<String>, String> {
    Ok(super::config::resolve(dir, &|_| None, &Default::default())?.and_then(|c| c.token))
}

#[cfg(target_arch = "wasm32")]
fn held_token(_: &std::path::Path) -> Result<Option<String>, String> {
    Ok(None)
}

#[cfg(not(target_arch = "wasm32"))]
fn forget_files(dir: &std::path::Path) -> Result<(), String> {
    super::config::forget(dir).map(|_| ())
}

#[cfg(target_arch = "wasm32")]
fn forget_files(_: &std::path::Path) -> Result<(), String> {
    Err("a browser keeps no PLM files".into())
}

fn spawn(task: impl std::future::Future<Output = ()> + 'static) {
    crate::store::spawn_waking(task);
}

