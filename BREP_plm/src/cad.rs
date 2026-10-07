//! Hosting the wasm CAD app from a directory the administrator controls
//! (plm-cad-integration-todo D6, D7, §2 P2).
//!
//! `--cad-app <dir>` (default `<data>/cad-app`) holds the app's web bundle
//! exactly as `./build.sh app` leaves it in `BREP_app/`: `web/` (the page and
//! the history worker) and `pkg/` (the wasm module and its bindings) as
//! siblings. The layout is not a choice: `web/index.html` imports
//! `../pkg/brep_app.js` and spawns `./worker.js` by relative path. The server
//! answers it under [`PREFIX`], so the page is `/cad/app/web/index.html` and a
//! CAD upgrade is a file drop into that directory, not a PLM release.
//!
//! # The second Content Security Policy
//!
//! The PLM's own pages keep [`crate::security::CONTENT_SECURITY_POLICY`]. The
//! bundle cannot run under it, and [`policy`] is what it needs instead,
//! measured on the bundle rather than guessed:
//!
//! * `script-src 'wasm-unsafe-eval'` — the module is compiled with
//!   `WebAssembly.instantiateStreaming` on the page and again in the worker.
//! * `worker-src 'self'` — the history runner is a module worker,
//!   `new Worker("./worker.js", { type: "module" })`.
//! * **hashes of the page's inline blocks.** `web/index.html` carries an inline
//!   `<style>` and an inline module `<script>`; [`inline_hashes`] reads them
//!   from the file being served, so a new page from a new release is allowed
//!   without a PLM change and nothing needs `'unsafe-inline'`. An attacker who
//!   can already write that file is past any policy.
//! * `img-src 'self' data:` — the favicon is a `data:` SVG. Fonts are compiled
//!   into the wasm, so `font-src` is only `'self'`.
//! * `connect-src 'self'` plus the administrator's `--cad-connect-src`
//!   origins. Same-origin is the default (D6): the app's calls to the PLM are
//!   same-origin. Three app features reach other origins — the KiCad remote
//!   library (`https://gitlab.com`), STEP parts search (`https://api.step.parts`
//!   and the media hosts its results name) and the bug report
//!   (`https://next.brep.io`) — and are refused by the browser until the
//!   administrator names them. An air-gapped PLM leaves them out.
//!
//! Every file under the prefix carries the policy, not only the page: a
//! worker's policy is the one on the worker script's OWN response.
//!
//! # Caching
//!
//! wasm-pack does not fingerprint its output — `brep_app_bg.wasm` keeps its
//! name across releases — so a long `max-age` would strand every browser on
//! the old build after a file drop. Every file is `no-cache` with an `ETag`
//! (size and modification time), and a matching `If-None-Match` answers `304`,
//! so an unchanged 38 MB module is revalidated, not re-downloaded.
//!
//! # What may be served
//!
//! A path is refused (`404`) unless every component is a plain name: no `..`,
//! no empty component, no backslash, and nothing starting with `.` (so
//! `pkg/.gitignore` and any `.git/` are not served). The resolved file must
//! also canonicalize INSIDE the canonical directory, which refuses a symlink
//! that points out of it.

use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};

/// Where the bundle is served.
pub const PREFIX: &str = "/cad/app/";

/// The page a browser is sent to.
pub const ENTRY: &str = "/cad/app/web/index.html";

/// The directory's name under `--data` when `--cad-app` is not given.
pub const DEFAULT_DIR: &str = "cad-app";

/// The relative path under the bundle directory, if it is one this server may
/// serve. Checks the text only; [`resolve`] checks the filesystem.
pub fn safe_relative(path: &str) -> Option<PathBuf> {
    if path.is_empty() || path.contains('\\') || path.contains('\0') {
        return None;
    }
    let mut out = PathBuf::new();
    for part in path.split('/') {
        if part.is_empty() || part.starts_with('.') {
            return None;
        }
        out.push(part);
    }
    Some(out)
}

/// The file `path` names under `root`, or `None` when it must not be served:
/// an unsafe path, a missing file, a directory, or anything that resolves
/// outside `root` (a symlink out).
pub fn resolve(root: &Path, path: &str) -> Option<PathBuf> {
    let relative = safe_relative(path)?;
    let root = root.canonicalize().ok()?;
    let file = root.join(relative).canonicalize().ok()?;
    (file.starts_with(&root) && file.is_file()).then_some(file)
}

/// The `Content-Type` for a bundle file. `.wasm` must be `application/wasm`:
/// `instantiateStreaming` refuses anything else, and with `nosniff` on a wrong
/// script type is a hard module-load failure.
pub fn media_type(path: &Path) -> &'static str {
    let extension = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    match extension.as_str() {
        "wasm" => "application/wasm",
        "js" | "mjs" => "text/javascript; charset=utf-8",
        "html" | "htm" => "text/html; charset=utf-8",
        "css" => "text/css; charset=utf-8",
        "json" | "map" => "application/json",
        "svg" => "image/svg+xml",
        "png" => "image/png",
        "jpg" | "jpeg" => "image/jpeg",
        "gif" => "image/gif",
        "webp" => "image/webp",
        "ico" => "image/x-icon",
        "woff2" => "font/woff2",
        "woff" => "font/woff",
        "ttf" => "font/ttf",
        "otf" => "font/otf",
        // The installer scripts: read, not run, by a browser.
        "txt" | "md" | "sh" | "ps1" => "text/plain; charset=utf-8",
        _ => "application/octet-stream",
    }
}

/// The inline blocks of an HTML page, as CSP source expressions:
/// `(scripts, styles)`, each `'sha256-<base64>'` of the block's exact text.
/// A `<script src=…>` is not inline and is covered by `'self'`. Inline event
/// handlers and `style="…"` attributes are NOT hashed, so they stay refused.
pub fn inline_hashes(html: &str) -> (Vec<String>, Vec<String>) {
    (blocks(html, "script"), blocks(html, "style"))
}

fn blocks(html: &str, tag: &str) -> Vec<String> {
    use base64::engine::general_purpose::STANDARD;
    use base64::Engine;
    let lower = html.to_ascii_lowercase();
    let open = format!("<{tag}");
    let close = format!("</{tag}");
    let mut out = Vec::new();
    let mut at = 0;
    while let Some(found) = lower[at..].find(&open) {
        let start = at + found;
        let after_name = start + open.len();
        // `<scripts>` or `<styleX` is another tag.
        if !lower[after_name..].starts_with(|c: char| c == '>' || c.is_ascii_whitespace() || c == '/') {
            at = after_name;
            continue;
        }
        let Some(tag_end) = lower[after_name..].find('>').map(|i| after_name + i) else { break };
        let attributes = &lower[after_name..tag_end];
        let body_start = tag_end + 1;
        let Some(body_end) = lower[body_start..].find(&close).map(|i| body_start + i) else { break };
        let external = tag == "script" && attributes.split(|c: char| c.is_ascii_whitespace()).any(|a| a.starts_with("src="));
        if !external {
            let digest = Sha256::digest(html[body_start..body_end].as_bytes());
            out.push(format!("'sha256-{}'", STANDARD.encode(digest)));
        }
        at = body_end + close.len();
    }
    out
}

/// The bundle's Content Security Policy. `hashes` are the served page's
/// inline blocks ([`inline_hashes`]; empty for anything but a page);
/// `connect` the administrator's extra origins.
pub fn policy(hashes: &(Vec<String>, Vec<String>), connect: &[String]) -> String {
    let join = |extra: &[String]| extra.iter().map(|h| format!(" {h}")).collect::<String>();
    format!(
        "default-src 'none'; script-src 'self' 'wasm-unsafe-eval'{scripts}; \
style-src 'self'{styles}; img-src 'self' data:; connect-src 'self'{connect}; \
worker-src 'self'; font-src 'self'; manifest-src 'self'; form-action 'self'; \
frame-ancestors 'none'; base-uri 'none'",
        scripts = join(&hashes.0),
        styles = join(&hashes.1),
        connect = join(connect),
    )
}

/// Whether `origin` is something `connect-src` may name: a scheme, `://` and
/// a host, with no path, and nothing that could end the directive or add
/// another. Refused at start-up rather than written into every header.
pub fn check_connect_origin(origin: &str) -> Result<(), String> {
    let Some((scheme, host)) = origin.split_once("://") else {
        return Err(format!("'{origin}' is not an origin like https://gitlab.com"));
    };
    let good_scheme = matches!(scheme, "https" | "http" | "wss" | "ws");
    let good_host = !host.is_empty()
        && host.chars().all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '-' | ':' | '*' | '[' | ']'));
    if good_scheme && good_host {
        Ok(())
    } else {
        Err(format!("'{origin}' is not an origin like https://gitlab.com (a scheme and a host, no path)"))
    }
}

/// The native apps the directory may hold, as `(platform, path under the
/// directory)`. The installer scripts ([`INSTALLERS`]) download these.
pub const NATIVE: &[(&str, &str)] = &[
    ("linux-x86_64", "native/linux-x86_64/brep-app"),
    ("windows-x86_64", "native/windows-x86_64/brep-app.exe"),
];

/// The installer scripts, as `(name, path under the directory)`.
pub const INSTALLERS: &[(&str, &str)] = &[("linux", "install/install.sh"), ("windows", "install/install.ps1")];

/// A file the directory holds, for `/cad/config`: where it is served, its
/// size, and its SHA-256, so an installer can check what it downloaded.
#[derive(Debug, Clone, serde::Serialize, PartialEq, Eq)]
pub struct Listed {
    pub path: String,
    pub size: u64,
    pub sha256: String,
}

type HashCache = std::sync::Mutex<std::collections::HashMap<PathBuf, (u64, String, String)>>;

static HASHES: std::sync::OnceLock<HashCache> = std::sync::OnceLock::new();
static HASHED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// How many files [`listed`] has actually read to hash, in this process. A
/// test reads it to prove a second `/cad/config` does not re-read a 90 MB
/// executable.
pub fn files_hashed() -> usize {
    HASHED.load(std::sync::atomic::Ordering::Relaxed)
}

/// `relative` under `root` as a [`Listed`], or `None` when it is not there
/// (or may not be served). The hash is cached by the file's path and its
/// [`etag`], so it is computed once per file drop, not once per request.
/// Blocking: call it off the async runtime.
pub fn listed(root: &Path, relative: &str) -> Option<Listed> {
    let file = resolve(root, relative)?;
    let meta = std::fs::metadata(&file).ok()?;
    let tag = etag(&meta);
    let cache = HASHES.get_or_init(Default::default);
    let cached = cache.lock().ok()?.get(&file).filter(|(_, t, _)| *t == tag).map(|(_, _, h)| h.clone());
    let sha256 = match cached {
        Some(hash) => hash,
        None => {
            let mut hasher = Sha256::new();
            let mut reader = std::fs::File::open(&file).ok()?;
            std::io::copy(&mut reader, &mut hasher).ok()?;
            let hash = format!("{:x}", hasher.finalize());
            HASHED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            cache.lock().ok()?.insert(file.clone(), (meta.len(), tag, hash.clone()));
            hash
        }
    };
    Some(Listed { path: format!("{PREFIX}{relative}"), size: meta.len(), sha256 })
}

/// The `ETag` for a file: its size and modification time. A file drop
/// changes one or both.
pub fn etag(meta: &std::fs::Metadata) -> String {
    let modified = meta
        .modified()
        .ok()
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    format!("\"{:x}-{:x}\"", meta.len(), modified)
}

