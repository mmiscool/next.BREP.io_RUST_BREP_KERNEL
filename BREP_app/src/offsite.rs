//! Features that call another site, on a CAD app hosted by a PLM.
//!
//! A PLM serves the wasm app under a Content Security Policy whose
//! `connect-src` is its own origin plus the origins its administrator allows
//! (`brep-plm serve --cad-connect-src`, BREP_plm's `cad` module). A request to
//! any other origin is refused by the browser. `fetch` rejects with an opaque
//! network error, the same one it gives when offline or on a CORS refusal, so
//! the failure cannot say why. The app therefore asks BEFORE it fetches:
//!
//! * [`prefetch`] (at boot, wasm only) reads `/cad/config` when the page is
//!   served from under `/cad/app/`. Its `connect_src` is the administrator's
//!   list.
//! * [`refusal`] answers, for a URL a feature is about to fetch, the sentence
//!   to show instead: "STEP parts search is unavailable on this server:
//!   https://api.step.parts is not allowed". [`explain`] maps a failed fetch
//!   to the same sentence, for the one case where the policy arrived after the
//!   request left.
//!
//! The native app has no such policy, and neither does the public site or any
//! page not served by a PLM: there both functions let everything through.
//! The matching follows CSP's source-expression rules for what
//! `--cad-connect-src` accepts: a scheme, a host that may start with `*.`, and
//! an optional port (`*` for any).

/// What a hosted page may reach: its own origin and the listed ones.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Policy {
    /// The page's own origin, e.g. `https://plm.example.com`.
    pub page_origin: String,
    /// The administrator's `--cad-connect-src` origins.
    pub allowed: Vec<String>,
}

/// A URL's scheme, lower-case host and effective port.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Origin {
    scheme: String,
    host: String,
    port: u16,
}

impl Origin {
    fn parse(url: &str) -> Option<Origin> {
        let (scheme, rest) = url.split_once("://")?;
        let scheme = scheme.to_ascii_lowercase();
        let authority = rest.split(['/', '?', '#']).next().unwrap_or("");
        let authority = authority.rsplit('@').next().unwrap_or(authority);
        let (host, port) = split_port(authority);
        let port = match port {
            Some(port) => port.parse().ok()?,
            None => default_port(&scheme)?,
        };
        (!host.is_empty()).then(|| Origin { scheme, host: host.to_ascii_lowercase(), port })
    }

    fn display(&self) -> String {
        let host = if self.host.contains(':') { format!("[{}]", self.host) } else { self.host.clone() };
        match default_port(&self.scheme) {
            Some(port) if port == self.port => format!("{}://{host}", self.scheme),
            _ => format!("{}://{host}:{}", self.scheme, self.port),
        }
    }
}

fn default_port(scheme: &str) -> Option<u16> {
    match scheme {
        "https" | "wss" => Some(443),
        "http" | "ws" => Some(80),
        _ => None,
    }
}

/// `host[:port]`, minding an IPv6 literal's colons.
fn split_port(authority: &str) -> (&str, Option<&str>) {
    if let Some(rest) = authority.strip_prefix('[') {
        if let Some((host, after)) = rest.split_once(']') {
            return (host, after.strip_prefix(':'));
        }
    }
    match authority.rsplit_once(':') {
        Some((host, port)) if !host.contains(':') => (host, Some(port)),
        _ => (authority, None),
    }
}

/// Whether the CSP source `source` (`scheme://host[:port]`, host maybe
/// `*.suffix`, port maybe `*`) matches `url`. As in CSP, an `http` source also
/// matches `https`, and `ws`/`wss` follow `http`/`https`.
fn source_matches(source: &str, url: &Origin) -> bool {
    let Some((scheme, rest)) = source.split_once("://") else { return false };
    let scheme = scheme.to_ascii_lowercase();
    let scheme_ok = scheme == url.scheme
        || (scheme == "http" && matches!(url.scheme.as_str(), "https" | "ws" | "wss"))
        || (scheme == "https" && url.scheme == "wss");
    if !scheme_ok {
        return false;
    }
    let authority = rest.split('/').next().unwrap_or("");
    let (host, port) = split_port(authority);
    let host = host.to_ascii_lowercase();
    let host_ok = match host.strip_prefix("*.") {
        Some(suffix) => url.host.ends_with(&format!(".{suffix}")),
        None => host == url.host,
    };
    let port_ok = match port {
        Some("*") => true,
        Some(port) => port.parse::<u16>().ok() == Some(url.port),
        None => Some(url.port) == default_port(&url.scheme) || Some(url.port) == default_port(&scheme),
    };
    host_ok && port_ok
}

/// The sentence `feature` shows instead of fetching `url` under `policy`, or
/// `None` when the fetch may go. A URL with no scheme is relative, so it is
/// the page's own origin and always allowed.
pub fn refusal_for(policy: &Policy, feature: &str, url: &str) -> Option<String> {
    if !url.contains("://") {
        return None;
    }
    let Some(origin) = Origin::parse(url) else {
        return Some(format!("{feature} is unavailable on this server: {url} is not a web address"));
    };
    let own = Origin::parse(&policy.page_origin).is_some_and(|page| page == origin);
    let listed = policy.allowed.iter().any(|source| source_matches(source, &origin));
    (!own && !listed).then(|| format!("{feature} is unavailable on this server: {} is not allowed", origin.display()))
}

/// The `connect_src` list out of a `/cad/config` body. `None` when the body
/// has no such list (a PLM from before it existed): then nothing is known,
/// and nothing is refused.
pub fn parse_config(body: &str) -> Option<Vec<String>> {
    let value: serde_json::Value = serde_json::from_str(body).ok()?;
    let list = value.get("connect_src")?.as_array()?;
    Some(list.iter().filter_map(|v| v.as_str().map(str::to_string)).collect())
}

/// Start reading this page's policy (wasm, on a page under `/cad/app/`).
/// Idempotent; the native app has nothing to read.
pub fn prefetch() {
    #[cfg(target_arch = "wasm32")]
    web::prefetch();
}

/// The sentence to show instead of fetching `url` for `feature`, or `None`
/// when the fetch may go — including whenever the policy is not known (yet).
pub fn refusal(feature: &str, url: &str) -> Option<String> {
    #[cfg(target_arch = "wasm32")]
    {
        return web::policy().and_then(|policy| refusal_for(&policy, feature, url));
    }
    #[cfg(all(not(target_arch = "wasm32"), not(test)))]
    {
        let _ = (feature, url);
        None
    }
}



/// `crate::http::fetch_text` for a request that may leave before the policy
/// is known — the `?loadModel=` link, fetched at boot. On a hosted page it
/// waits for the policy, then answers the refusal sentence or fetches; on an
/// unhosted page it fetches at once. Wasm only, like its one caller.
#[cfg(target_arch = "wasm32")]
pub(crate) fn fetch_text(
    ctx: &eframe::egui::Context,
    feature: &'static str,
    url: String,
) -> std::sync::mpsc::Receiver<Result<String, String>> {
    let (tx, rx) = std::sync::mpsc::channel();
    let ctx = ctx.clone();
    web::when_known(move |policy| {
        if let Some(sentence) = policy.and_then(|p| refusal_for(&p, feature, &url)) {
            let _ = tx.send(Err(sentence));
            ctx.request_repaint();
            return;
        }
        web::fetch_text_into(ctx, url, tx);
    });
    rx
}

/// A failed fetch's message: the refusal sentence when the policy (which may
/// have arrived after the request left) forbids `url`, else `error` as is.
pub fn explain(feature: &str, url: &str, error: String) -> String {
    refusal(feature, url).unwrap_or(error)
}

#[cfg(target_arch = "wasm32")]
mod web {
    use super::{parse_config, Policy};
    use std::cell::RefCell;

    enum State {
        Unasked,
        Asking,
        /// `None`: not served by a PLM, or the PLM names no list.
        Known(Option<Policy>),
    }

    type Waiter = Box<dyn FnOnce(Option<Policy>)>;

    thread_local! {
        static STATE: RefCell<State> = const { RefCell::new(State::Unasked) };
        static WAITERS: RefCell<Vec<Waiter>> = const { RefCell::new(Vec::new()) };
    }

    /// Settle the policy and run whatever waited for it.
    fn settle(policy: Option<Policy>) {
        STATE.with(|s| *s.borrow_mut() = State::Known(policy.clone()));
        let waiting = WAITERS.with(|w| std::mem::take(&mut *w.borrow_mut()));
        for waiter in waiting {
            waiter(policy.clone());
        }
    }

    /// Run `f` with the policy once it is known: now, or when `/cad/config`
    /// answers.
    pub fn when_known(f: impl FnOnce(Option<Policy>) + 'static) {
        prefetch();
        let known = STATE.with(|s| match &*s.borrow() {
            State::Known(policy) => Some(policy.clone()),
            _ => None,
        });
        match known {
            Some(policy) => f(policy),
            None => WAITERS.with(|w| w.borrow_mut().push(Box::new(f))),
        }
    }

    /// `crate::http::fetch_text`'s request, answering into `tx`.
    pub fn fetch_text_into(
        ctx: eframe::egui::Context,
        url: String,
        tx: std::sync::mpsc::Sender<Result<String, String>>,
    ) {
        ehttp::fetch(ehttp::Request::get(url), move |result| {
            let out = match result {
                Ok(r) if r.ok => Ok(r.text().map(str::to_owned).unwrap_or_else(|| String::from_utf8_lossy(&r.bytes).into_owned())),
                Ok(r) => Err(format!("HTTP {} {}", r.status, r.status_text)),
                Err(error) => Err(error),
            };
            let _ = tx.send(out);
            ctx.request_repaint();
        });
    }

    pub fn prefetch() {
        let start = STATE.with(|s| matches!(*s.borrow(), State::Unasked));
        if !start {
            return;
        }
        let location = web_sys::window().map(|w| w.location());
        let path = location.as_ref().and_then(|l| l.pathname().ok()).unwrap_or_default();
        let origin = location.as_ref().and_then(|l| l.origin().ok()).unwrap_or_default();
        if !path.starts_with("/cad/app/") {
            settle(None);
            return;
        }
        STATE.with(|s| *s.borrow_mut() = State::Asking);
        ehttp::fetch(ehttp::Request::get("/cad/config"), move |result| {
            let policy = result
                .ok()
                .filter(|r| r.ok)
                .and_then(|r| r.text().and_then(parse_config))
                .map(|allowed| Policy { page_origin: origin.clone(), allowed });
            settle(policy);
        });
    }

    pub fn policy() -> Option<Policy> {
        prefetch();
        STATE.with(|s| match &*s.borrow() {
            State::Known(policy) => policy.clone(),
            _ => None,
        })
    }
}

