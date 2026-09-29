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
//!   (D5) and says the next start connects: the store is chosen before the
//!   first frame, and settings, the dock and recovery all read it there.
//!   Forget removes both files; the next start is the file app.
//! - **Web.** The page is served by the PLM (D6), so a sign-in is the PLM's own
//!   session cookie, and the page reloads onto it. Forget signs the session
//!   out and reloads.
use super::client::{Me, PlmClient};
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

/// The URL and user in the label [`super::backend::compose`] gives a PLM
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

    /// Call once per frame: drive the queue while busy and collect a finished
    /// action. Returns true when something changed.
    pub fn poll(&mut self) -> bool {
        let Some(slot) = &self.pending else { return false };
        #[cfg(not(target_arch = "wasm32"))]
        super::native::run_pending();
        let Some(outcome) = slot.borrow_mut().take() else { return false };
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
        true
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
        let token = self.token_input.trim().to_string();
        let (client, keep) = (self.client(), self.keep.clone());
        self.token_input.clear();
        self.start(async move {
            match client.sign_in_with_token(&token).await {
                Ok(me) => keep_token(&keep, &token, &me),
                Err(e) => Outcome::Failed(e.to_string()),
            }
        });
    }

    /// Sign in with a username and password: a token for this machine
    /// (native), or the PLM's session (web). The password is not kept.
    pub fn use_password(&mut self) {
        let (username, password) = (self.username.trim().to_string(), std::mem::take(&mut self.password));
        let (client, keep) = (self.client(), self.keep.clone());
        self.start(async move {
            match keep {
                Keep::Files(_) => match client.sign_in_for_token(&username, &password, &token_name()).await {
                    Ok((me, token)) => keep_token(&keep, &token, &me),
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
                self.last = Some(match forget_files(&dir) {
                    Ok(()) => Outcome::Done("this machine's PLM sign-in is forgotten; the next start is the file app".into()),
                    Err(e) => Outcome::Failed(e),
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
        let host = std::env::var("HOSTNAME").ok().filter(|h| !h.is_empty());
        format!("brep-app {}", host.unwrap_or_else(|| "desktop".into()))
    }
    #[cfg(target_arch = "wasm32")]
    {
        "brep-app web".into()
    }
}

fn keep_token(keep: &Keep, token: &str, me: &Me) -> Outcome {
    match keep {
        Keep::Files(dir) => match write_token(dir, token) {
            Ok(()) => Outcome::Done(format!(
                "signed in as {}; the token is kept on this machine, and the next start connects",
                me.username
            )),
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
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(task);
    #[cfg(not(target_arch = "wasm32"))]
    super::native::spawn(task);
}

