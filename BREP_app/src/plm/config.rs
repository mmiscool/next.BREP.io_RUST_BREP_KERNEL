//! Where the native app finds its PLM server (plan D5).
//!
//! Two files beside the config the native app already keeps
//! (`$XDG_CONFIG_HOME/brep-app`, else `$HOME/.config/brep-app` — the directory
//! `FileModelStore` roots itself in):
//!
//! * `plm.json` — `{"url": "https://plm.example.com"}`. Not a secret.
//! * `plm-token` — the API token (`plm_…`) alone, at mode 0600. A secret, so it
//!   is its own file and never a field of `@settings`, which the explorer shows
//!   and an export could carry.
//!
//! Each field is overridden by an environment variable (`BREP_PLM_URL`,
//! `BREP_PLM_TOKEN`), which is overridden by a command-line flag (`--plm-url`,
//! `--plm-token-file`). The token's flag names a FILE, never the token itself:
//! a command line is readable by every user of the machine through `ps`.
//!
//! **No URL from any source means no PLM**, and [`resolve`] answers `Ok(None)`
//! having read nothing but two absent files — the file-based app of today. An
//! empty variable counts as unset, so `BREP_PLM_URL= brep-app` is the file app.
//!
//! [`write`] and [`forget`] are what `brep-app --plm-configure` / `--plm-forget`
//! run, so an installer script configures a machine with no prompt and no
//! hand-written JSON (plan S12).

use std::path::{Path, PathBuf};

/// The URL file's name inside the app config directory.
pub const URL_FILE: &str = "plm.json";
/// The token file's name inside the app config directory.
pub const TOKEN_FILE: &str = "plm-token";
/// Overrides `plm.json`'s `url`.
pub const URL_ENV: &str = "BREP_PLM_URL";
/// Overrides the token file.
pub const TOKEN_ENV: &str = "BREP_PLM_TOKEN";
/// Every API token the server issues starts with this (`BREP_plm`'s
/// `security::TOKEN_PREFIX`).
pub const TOKEN_PREFIX: &str = "plm_";

/// The native app's config directory — the ONE rule, which `FileModelStore`
/// (models, settings, recovery) and the two PLM files both use:
/// `$XDG_CONFIG_HOME/brep-app`, else on Windows `%APPDATA%\brep-app`, else
/// `$HOME/.config/brep-app`, else `./brep-app`. An empty variable counts as
/// unset.
///
/// Windows sets neither `XDG_CONFIG_HOME` nor `HOME`, so without its own arm
/// the app kept its files in whatever the working directory happened to be:
/// an installer's `--plm-configure` and a Start-menu launch would disagree.
pub fn app_config_dir() -> PathBuf {
    config_dir_from(&|name| std::env::var_os(name), cfg!(windows))
}

/// [`app_config_dir`] with the environment and platform passed in, so a test
/// can hold both still.
fn config_dir_from(env: &dyn Fn(&str) -> Option<std::ffi::OsString>, windows: bool) -> PathBuf {
    let var = |name: &str| env(name).filter(|v| !v.is_empty()).map(PathBuf::from);
    var("XDG_CONFIG_HOME")
        .or_else(|| if windows { var("APPDATA") } else { None })
        .or_else(|| var("HOME").map(|h| h.join(".config")))
        .unwrap_or_else(|| PathBuf::from("."))
        .join("brep-app")
}

/// Which layer a resolved value came from — shown by `--plm-show` so a user
/// can tell why the app is talking to the server it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    File,
    Env,
    Flag,
}

impl Source {
    pub fn describe(self) -> &'static str {
        match self {
            Source::File => "config file",
            Source::Env => "environment",
            Source::Flag => "command line",
        }
    }
}

/// The server this session connects to.
#[derive(Clone, PartialEq, Eq)]
pub struct PlmConfig {
    /// Normalised: `http://` or `https://`, no trailing slash.
    pub url: String,
    pub url_source: Source,
    /// `None` until the user signs in (S1's password-to-token flow writes it).
    pub token: Option<String>,
    pub token_source: Option<Source>,
}

// The token is a secret: a `{:?}` in a log line must not print it.
impl std::fmt::Debug for PlmConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PlmConfig")
            .field("url", &self.url)
            .field("url_source", &self.url_source)
            .field("token", &self.token.as_ref().map(|_| "<redacted>"))
            .field("token_source", &self.token_source)
            .finish()
    }
}

/// The command line's two overrides.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Flags {
    pub url: Option<String>,
    pub token_file: Option<PathBuf>,
}

/// Resolve the session's PLM from the config directory `dir`, the environment
/// (`env` stands in for `std::env::var` so tests need not mutate the process),
/// and `flags`. `Ok(None)`: no PLM, the file-based app. `Err`: a PLM was asked
/// for and cannot be used as written — the sentence names the file or variable.
pub fn resolve(
    dir: &Path,
    env: &dyn Fn(&str) -> Option<String>,
    flags: &Flags,
) -> Result<Option<PlmConfig>, String> {
    let env = |name: &str| env(name).filter(|v| !v.trim().is_empty());

    let (raw_url, url_source) = if let Some(url) = &flags.url {
        (url.clone(), Source::Flag)
    } else if let Some(url) = env(URL_ENV) {
        (url, Source::Env)
    } else if let Some(url) = read_url_file(dir)? {
        (url, Source::File)
    } else {
        return Ok(None);
    };
    let url = normalise_url(&raw_url).map_err(|e| format!("{} ({}): {e}", url_origin(url_source), raw_url))?;

    let token = if let Some(path) = &flags.token_file {
        Some((read_token_file(path, false)?.ok_or_else(|| format!("--plm-token-file {}: no such file", path.display()))?, Source::Flag))
    } else if let Some(token) = env(TOKEN_ENV) {
        Some((token, Source::Env))
    } else {
        read_token_file(&dir.join(TOKEN_FILE), true)?.map(|t| (t, Source::File))
    };
    let (token, token_source) = match token {
        Some((token, source)) => {
            let token = token.trim().to_string();
            check_token(&token).map_err(|e| format!("{}: {e}", token_origin(source)))?;
            (Some(token), Some(source))
        }
        None => (None, None),
    };
    Ok(Some(PlmConfig { url, url_source, token, token_source }))
}

/// Write `plm.json` (and, when given, `plm-token` at 0600) into `dir`, creating
/// it. With `token: None` an existing token file is LEFT ALONE — re-pointing a
/// machine at a moved server keeps its sign-in; [`forget`] is the way to drop it.
/// Each file is written whole to a sibling temp file and renamed over, so a
/// reader never sees half a file.
pub fn write(dir: &Path, url: &str, token: Option<&str>) -> Result<(), String> {
    let url = normalise_url(url)?;
    if let Some(token) = token {
        check_token(token.trim())?;
    }
    std::fs::create_dir_all(dir).map_err(|e| format!("cannot create {}: {e}", dir.display()))?;
    let json = serde_json::to_string_pretty(&serde_json::json!({ "url": url })).expect("a string map serialises");
    write_atomic(&dir.join(URL_FILE), format!("{json}\n").as_bytes(), false)?;
    if let Some(token) = token {
        write_atomic(&dir.join(TOKEN_FILE), format!("{}\n", token.trim()).as_bytes(), true)?;
    }
    Ok(())
}

/// Store only the token (S1's sign-in writes it after a password exchange).
pub fn write_token(dir: &Path, token: &str) -> Result<(), String> {
    check_token(token.trim())?;
    std::fs::create_dir_all(dir).map_err(|e| format!("cannot create {}: {e}", dir.display()))?;
    write_atomic(&dir.join(TOKEN_FILE), format!("{}\n", token.trim()).as_bytes(), true)
}

/// Remove both files: the next launch is the file-based app. Absent files are
/// not an error. Returns the paths actually removed.
pub fn forget(dir: &Path) -> Result<Vec<PathBuf>, String> {
    let mut removed = Vec::new();
    for name in [URL_FILE, TOKEN_FILE] {
        let path = dir.join(name);
        match std::fs::remove_file(&path) {
            Ok(()) => removed.push(path),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(format!("cannot remove {}: {e}", path.display())),
        }
    }
    Ok(removed)
}

/// `--plm-show`'s text: the effective server and where each part came from.
/// The token itself is never printed, only its first characters, the way the
/// server's own token list shows a `prefix`.
pub fn describe(config: Option<&PlmConfig>, dir: &Path) -> String {
    let Some(config) = config else {
        return format!("no PLM configured (no {URL_ENV}, no {} in {}): the file-based app\n", URL_FILE, dir.display());
    };
    let token = match (&config.token, config.token_source) {
        (Some(token), Some(source)) => {
            let shown: String = token.chars().take(TOKEN_PREFIX.len() + 4).collect();
            format!("{shown}… (from the {})", source.describe())
        }
        _ => "none — sign in from the app".to_string(),
    };
    format!("url:   {} (from the {})\ntoken: {token}\n", config.url, config.url_source.describe())
}

fn url_origin(source: Source) -> String {
    match source {
        Source::Flag => "--plm-url".into(),
        Source::Env => URL_ENV.into(),
        Source::File => URL_FILE.into(),
    }
}

fn token_origin(source: Source) -> String {
    match source {
        Source::Flag => "--plm-token-file".into(),
        Source::Env => TOKEN_ENV.into(),
        Source::File => TOKEN_FILE.into(),
    }
}

/// `http://` or `https://`, a host, no query or fragment; the trailing slash
/// is dropped so `format!("{url}{path}")` joins cleanly.
fn normalise_url(raw: &str) -> Result<String, String> {
    let url = raw.trim().trim_end_matches('/');
    let rest = url
        .strip_prefix("https://")
        .or_else(|| url.strip_prefix("http://"))
        .ok_or("the PLM address must start with http:// or https://")?;
    if rest.is_empty() || rest.starts_with('/') {
        return Err("the PLM address has no host".into());
    }
    if rest.contains(['?', '#', ' ']) {
        return Err("the PLM address must be the server's base URL, with no query, fragment or spaces".into());
    }
    Ok(url.to_string())
}

fn check_token(token: &str) -> Result<(), String> {
    if !token.starts_with(TOKEN_PREFIX) || token.len() <= TOKEN_PREFIX.len() {
        return Err(format!("not a PLM API token (they start with `{TOKEN_PREFIX}`)"));
    }
    if token.chars().any(|c| c.is_whitespace() || c.is_control()) {
        return Err("a PLM API token is one word; this one holds whitespace".into());
    }
    Ok(())
}

fn read_url_file(dir: &Path) -> Result<Option<String>, String> {
    let path = dir.join(URL_FILE);
    let text = match std::fs::read_to_string(&path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("cannot read {}: {e}", path.display())),
    };
    let value: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| format!("{} is not JSON: {e}", path.display()))?;
    match value.get("url") {
        Some(serde_json::Value::String(url)) if !url.trim().is_empty() => Ok(Some(url.clone())),
        _ => Err(format!("{} has no \"url\" string", path.display())),
    }
}

/// Read a token file. `strict_mode`: refuse one other users can read (unix) —
/// the app's own file must be 0600; a file named by a flag is the caller's
/// business and is only read.
fn read_token_file(path: &Path, strict_mode: bool) -> Result<Option<String>, String> {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(format!("cannot read {}: {e}", path.display())),
    };
    #[cfg(unix)]
    if strict_mode {
        use std::os::unix::fs::PermissionsExt;
        let mode = std::fs::metadata(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?.permissions().mode();
        if mode & 0o077 != 0 {
            return Err(format!(
                "{} is readable by other users (mode {:o}); run `chmod 600 {}` or sign in again",
                path.display(),
                mode & 0o777,
                path.display()
            ));
        }
    }
    #[cfg(not(unix))]
    let _ = strict_mode;
    Ok(Some(text))
}

fn write_atomic(path: &Path, bytes: &[u8], secret: bool) -> Result<(), String> {
    use std::io::Write;
    let tmp = path.with_extension(format!("tmp{}", std::process::id()));
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create(true).truncate(true);
    #[cfg(unix)]
    if secret {
        use std::os::unix::fs::OpenOptionsExt;
        // Created 0600, never chmod-ed down afterwards: there is no instant at
        // which the token sits in a file another user could open.
        options.mode(0o600);
    }
    #[cfg(not(unix))]
    let _ = secret;
    let result = options
        .open(&tmp)
        .and_then(|mut file| {
            file.write_all(bytes)?;
            file.sync_all()
        })
        .and_then(|()| std::fs::rename(&tmp, path));
    if let Err(e) = result {
        let _ = std::fs::remove_file(&tmp);
        return Err(format!("cannot write {}: {e}", path.display()));
    }
    Ok(())
}

