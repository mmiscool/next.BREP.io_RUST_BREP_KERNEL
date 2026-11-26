//! The app's one HTTP client, both targets: `ureq` on a background thread
//! natively (the same client BREP_mcp_core and BREP_plm use), the browser's
//! `fetch` on wasm. It replaced the `ehttp` crate, which was this same pair
//! behind the same request/response shapes, so every caller (the step.parts
//! library, the bug report, the update check, the PLM transport, report
//! follow-ups, `/cad/config`) behaves as before:
//!
//! * a reply of any status is `Ok` with `ok` set for 2xx, and its body is
//!   read (a 404's page included); `Err` is a transport failure only;
//! * [`fetch`] answers through a callback: on its own thread natively, on the
//!   page's task queue on wasm;
//! * [`fetch_blocking`] (native only) answers on the caller's thread.

/// Request or response headers, in order, duplicates kept.
#[derive(Clone, Debug, Default)]
pub struct Headers {
    pub headers: Vec<(String, String)>,
}

impl Headers {
    pub fn new(headers: &[(&str, &str)]) -> Self {
        Self { headers: headers.iter().map(|(k, v)| ((*k).to_owned(), (*v).to_owned())).collect() }
    }

    pub fn insert(&mut self, key: impl ToString, value: impl ToString) {
        self.headers.push((key.to_string(), value.to_string()));
    }

    /// The first value under `key`, compared case-insensitively.
    #[allow(dead_code)] // the PLM client's tests read cookies through it
    pub fn get(&self, key: &str) -> Option<&str> {
        self.headers.iter().find(|(k, _)| k.eq_ignore_ascii_case(key)).map(|(_, v)| v.as_str())
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn sort(&mut self) {
        self.headers.sort_by(|a, b| a.0.cmp(&b.0));
    }
}

impl<'h> IntoIterator for &'h Headers {
    type Item = &'h (String, String);
    type IntoIter = std::slice::Iter<'h, (String, String)>;

    fn into_iter(self) -> Self::IntoIter {
        self.headers.iter()
    }
}

/// One request. The browser sends it in `cors` mode, fetch's default.
#[derive(Clone, Debug)]
pub struct Request {
    pub method: String,
    pub url: String,
    pub body: Vec<u8>,
    pub headers: Headers,
}

impl Request {
    pub fn get(url: impl ToString) -> Self {
        Self { method: "GET".to_owned(), url: url.to_string(), body: vec![], headers: Headers::new(&[("Accept", "*/*")]) }
    }

    pub fn post(url: impl ToString, body: Vec<u8>) -> Self {
        Self {
            method: "POST".to_owned(),
            url: url.to_string(),
            body,
            headers: Headers::new(&[("Accept", "*/*"), ("Content-Type", "text/plain; charset=utf-8")]),
        }
    }
}

/// One reply, whatever its status.
#[derive(Clone)]
pub struct Response {
    /// The final URL, after redirects.
    #[allow(dead_code)]
    pub url: String,
    /// The status is 2xx.
    pub ok: bool,
    pub status: u16,
    pub status_text: String,
    pub headers: Headers,
    pub bytes: Vec<u8>,
}

impl Response {
    /// The body as text, when it is UTF-8.
    pub fn text(&self) -> Option<&str> {
        std::str::from_utf8(&self.bytes).ok()
    }
}

impl std::fmt::Debug for Response {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Response")
            .field("url", &self.url)
            .field("ok", &self.ok)
            .field("status", &self.status)
            .field("status_text", &self.status_text)
            .field("headers", &self.headers)
            .field("bytes", &format!("{} bytes", self.bytes.len()))
            .finish_non_exhaustive()
    }
}

/// A transport failure (no connection, DNS, TLS, CORS, a body that broke
/// off), as text.
pub type Error = String;
pub type Result<T> = std::result::Result<T, Error>;

/// Send `request`; `on_done` gets the reply.
pub fn fetch(request: Request, on_done: impl 'static + Send + FnOnce(Result<Response>)) {
    #[cfg(not(target_arch = "wasm32"))]
    std::thread::Builder::new()
        .name("brep-http".to_owned())
        .spawn(move || on_done(fetch_blocking(&request)))
        .expect("failed to spawn the HTTP request thread");
    #[cfg(target_arch = "wasm32")]
    wasm_bindgen_futures::spawn_local(async move {
        let result = web::fetch(&request).await.map_err(web::describe);
        on_done(result)
    });
}

/// Send `request` and wait for the reply on this thread.
#[cfg(not(target_arch = "wasm32"))]
pub fn fetch_blocking(request: &Request) -> Result<Response> {
    let mut outgoing = ureq::request(&request.method, &request.url);
    for (key, value) in &request.headers {
        outgoing = outgoing.set(key, value);
    }
    let reply = if request.body.is_empty() { outgoing.call() } else { outgoing.send_bytes(&request.body) };
    let (ok, reply) = match reply {
        Ok(reply) => (true, reply),
        // A 4xx/5xx still carries a body the caller may want (an error page,
        // a refusal's JSON).
        Err(ureq::Error::Status(_, reply)) => (false, reply),
        Err(ureq::Error::Transport(error)) => return Err(error.to_string()),
    };
    let url = reply.get_url().to_owned();
    let status = reply.status();
    let status_text = reply.status_text().to_owned();
    let mut headers = Headers::default();
    for key in &reply.headers_names() {
        if let Some(value) = reply.header(key) {
            headers.insert(key, value);
        }
    }
    // Name order, as the browser's Headers iterator gives them.
    headers.sort();
    let mut bytes = vec![];
    use std::io::Read as _;
    if let Err(error) = reply.into_reader().read_to_end(&mut bytes) {
        // A HEAD reply has no body to read.
        if !(request.method == "HEAD" && error.kind() == std::io::ErrorKind::UnexpectedEof) {
            return Err(format!("Failed to read response body: {error}"));
        }
    }
    Ok(Response { url, ok, status, status_text, headers, bytes })
}

#[cfg(target_arch = "wasm32")]
mod web {
    use super::{Headers, Request, Response};
    use wasm_bindgen::{JsCast, JsValue};
    use wasm_bindgen_futures::JsFuture;

    /// A thrown fetch exception as text. A `TypeError` is fetch's opaque
    /// network error: the details are only in the console, so it goes there.
    pub(super) fn describe(value: JsValue) -> String {
        value.as_string().unwrap_or_else(|| {
            if value.has_type::<js_sys::TypeError>() {
                web_sys::console::error_1(&value);
                "Failed to fetch, check the developer console for details".to_owned()
            } else {
                format!("{value:#?}")
            }
        })
    }

    pub(super) async fn fetch(request: &Request) -> Result<Response, JsValue> {
        let init = web_sys::RequestInit::new();
        init.set_method(&request.method);
        init.set_mode(web_sys::RequestMode::Cors);
        if !request.body.is_empty() {
            let body = js_sys::Uint8Array::from(request.body.as_slice());
            init.set_body(&body);
        }
        let outgoing = web_sys::Request::new_with_str_and_init(&request.url, &init)?;
        for (key, value) in &request.headers {
            outgoing.headers().set(key, value)?;
        }
        let window = web_sys::window().ok_or_else(|| JsValue::from_str("no window to fetch from"))?;
        let reply: web_sys::Response = JsFuture::from(window.fetch_with_request(&outgoing)).await?.dyn_into()?;
        let buffer = JsFuture::from(reply.array_buffer()?).await?;
        let bytes = js_sys::Uint8Array::new(&buffer).to_vec();
        let mut headers = Headers::default();
        // The Headers iterator yields [name, value] pairs, names lowercased
        // and sorted, duplicate names combined.
        let pairs = js_sys::try_iter(&reply.headers())?.ok_or_else(|| JsValue::from_str("headers have no iterator"))?;
        for pair in pairs {
            let pair: js_sys::Array = pair?.into();
            let name = pair.get(0).as_string().ok_or_else(|| JsValue::from_str("headers name"))?;
            let value = pair.get(1).as_string().ok_or_else(|| JsValue::from_str("headers value"))?;
            headers.insert(name, value);
        }
        Ok(Response {
            url: reply.url(),
            ok: reply.ok(),
            status: reply.status(),
            status_text: reply.status_text(),
            headers,
            bytes,
        })
    }
}
