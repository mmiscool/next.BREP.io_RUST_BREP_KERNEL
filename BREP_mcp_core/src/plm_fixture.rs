//! The PLM fixture: a real `brep-plm serve` for one `test-mcp`
//! script.
//!
//! A script that declares `plm` gets, before its session starts:
//!
//! 1. `brep-plm serve --bind 127.0.0.1:0 --data <fresh dir>`, so every
//!    run starts from the server's own first-run seed and nothing leaks between
//!    scripts. The data directory is on tmpfs (`/dev/shm`) where there is one,
//!    and is removed when the fixture drops: the server runs SQLite at
//!    `synchronous=FULL`, and on a disk shared with parallel builds one fsync
//!    measured 1.2–1.7 s, twelve of them made start-up take 13 s, and a run
//!    timed out at 60 s. Durability is worthless to a throwaway fixture.
//!    `BREP_PLM_FIXTURE_DATA` names another parent directory (the run's
//!    `<out>/plm` is used where there is no tmpfs).
//! 2. The first-run administrator password, read from the one place the server
//!    ever shows it (its stdout), then a sign-in, and a token minted through
//!    that session exactly as a person mints one on the web page. Each account
//!    the script names is created the same way and gets its own token.
//! 3. The script's `setup` calls, through the API — never by writing the data
//!    directory, so the fixture can only build states the server would build.
//! 4. D5's two files in the session store (the app's config root):
//!    `plm.json` holding the URL and `plm-token` holding the app user's token
//!    (`app_token: false` writes `plm.json` alone: a machine not signed in).
//!
//! Steps then talk to the server with `plm_request` (handled by the runner, not
//! the app: it is the "second client" every PLM gate needs), and any string in
//! any step may use `${plm:NAME}`:
//!
//! - `${plm:url}` — the server's base URL;
//! - `${plm:token}` — the app user's token, `${plm:token:<user>}` any account's;
//! - any name a `bind` captured from an earlier answer.
//!
//! The server is killed when the fixture drops. Its stdout and stderr go to
//! `<out>/plm/serve.log`.
use crate::script::{PlmCall, PlmSpec};
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{mpsc, Mutex};
use std::time::{Duration, Instant};

/// The password a script's account gets when it names none.
pub const DEFAULT_PASSWORD: &str = "test-mcp-password";

/// How long the server may take to print its address and answer.
const START_TIMEOUT: Duration = Duration::from_secs(60);

pub struct PlmFixture {
    child: Child,
    /// `http://127.0.0.1:<port>`, no trailing slash.
    pub url: String,
    /// Which binary is serving, and when it was built: `<path> (built <UTC>)`.
    /// The run prints it, because "the newer of debug and release" can pick a
    /// stale build without anyone noticing.
    pub binary: String,
    /// Username → token secret. `admin` is always present.
    tokens: BTreeMap<String, String>,
    /// The account the app was configured as (empty: none).
    app_user: String,
    /// Values `bind` captured, plus the built-ins.
    vars: Mutex<BTreeMap<String, Value>>,
    /// The server's data directory, removed on drop.
    data: PathBuf,
    agent: ureq::Agent,
}

/// An answer from the server, whatever its status.
pub struct Answer {
    pub status: u16,
    pub headers: Vec<(String, String)>,
    pub body: Vec<u8>,
}

impl Answer {
    fn json(&self) -> Option<Value> {
        serde_json::from_slice(&self.body).ok()
    }

    /// The shape a `plm_request` step returns: `status`, `json` (the body
    /// parsed, or null), `text` (the body when it is not JSON), `headers`.
    pub fn to_value(&self) -> Value {
        let parsed = self.json();
        let text = if parsed.is_none() { Value::String(String::from_utf8_lossy(&self.body).into_owned()) } else { Value::Null };
        let headers: serde_json::Map<String, Value> =
            self.headers.iter().map(|(k, v)| (k.to_ascii_lowercase(), Value::String(v.clone()))).collect();
        json!({ "status": self.status, "json": parsed.unwrap_or(Value::Null), "text": text, "headers": headers, "size": self.body.len() })
    }
}

/// Where the `brep-plm` binary is: `BREP_PLM_BIN`, else the newer of the
/// checkout's debug and release builds (`./build.sh test-mcp` builds debug,
/// the same target `test-plm` keeps warm).
pub fn server_binary() -> Result<PathBuf, String> {
    if let Some(bin) = std::env::var_os("BREP_PLM_BIN") {
        let bin = PathBuf::from(bin);
        return if bin.is_file() { Ok(bin) } else { Err(format!("BREP_PLM_BIN={} is not a file", bin.display())) };
    }
    let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("../BREP_plm/target");
    let name = if cfg!(windows) { "brep-plm.exe" } else { "brep-plm" };
    let modified = |p: &PathBuf| std::fs::metadata(p).and_then(|m| m.modified()).ok();
    ["debug", "release"]
        .iter()
        .map(|profile| target.join(profile).join(name))
        .filter(|p| p.is_file())
        .max_by_key(modified)
        .ok_or_else(|| {
            format!(
                "no brep-plm binary under {} — build it with `cargo build --manifest-path BREP_plm/Cargo.toml --bin brep-plm` or set BREP_PLM_BIN",
                target.display()
            )
        })
}

/// What the server's stdout told the fixture while it started.
#[derive(Default)]
struct Startup {
    password: Option<String>,
    /// From `brep-plm: listening on <addr>`: the address the OS gave `:0`.
    listening: Option<String>,
}

impl PlmFixture {
    /// Start and seed a server for `spec`, its data under `out_dir/plm`.
    /// `store` is the session's store directory, which receives D5's files.
    pub fn start(spec: &PlmSpec, out_dir: &Path, store: &Path) -> Result<Self, String> {
        let bin = server_binary()?;
        let root = out_dir.join("plm");
        if root.exists() {
            std::fs::remove_dir_all(&root).map_err(|e| format!("clear {}: {e}", root.display()))?;
        }
        std::fs::create_dir_all(&root).map_err(|e| format!("{}: {e}", root.display()))?;
        let data = data_dir(&root)?;
        let log_path = root.join("serve.log");
        let mut command = Command::new(&bin);
        command
            .arg("serve")
            .arg("--bind")
            // `:0`: the OS picks the port and the server reports it, so no
            // other process can take it between choosing and binding.
            .arg("127.0.0.1:0")
            .arg("--data")
            .arg(&data)
            .args(&spec.args)
            .envs(&spec.env)
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = command.spawn().map_err(|e| format!("spawn {}: {e}", bin.display()))?;

        // Both pipes are drained into the log for the server's whole life, so
        // a chatty server can never block on a full pipe. Stdout lines are also
        // sent here until the fixture has what it needs.
        let log = std::sync::Arc::new(Mutex::new(
            std::fs::File::create(&log_path).map_err(|e| format!("{}: {e}", log_path.display()))?,
        ));
        let (tx, rx) = mpsc::channel::<String>();
        let stdout = child.stdout.take().ok_or("no stdout pipe")?;
        let stderr = child.stderr.take().ok_or("no stderr pipe")?;
        {
            let log = log.clone();
            std::thread::spawn(move || {
                for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                    let _ = writeln!(log.lock().unwrap(), "{line}");
                    let _ = tx.send(line);
                }
            });
        }
        std::thread::spawn(move || {
            for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                let _ = writeln!(log.lock().unwrap(), "stderr: {line}");
            }
        });

        let fail = |child: &mut Child, why: String| -> String {
            let _ = child.kill();
            let _ = child.wait();
            let _ = std::fs::remove_dir_all(&data);
            format!("{why} (server log: {})", log_path.display())
        };

        let deadline = Instant::now() + START_TIMEOUT;
        let mut seen = Startup::default();
        while seen.listening.is_none() {
            let left = deadline.saturating_duration_since(Instant::now());
            match rx.recv_timeout(left) {
                Ok(line) => read_startup_line(&line, &mut seen),
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    return Err(fail(
                        &mut child,
                        format!(
                            "brep-plm printed no `brep-plm: listening on` line within {} s (a binary from before that line cannot serve the fixture)",
                            START_TIMEOUT.as_secs()
                        ),
                    ))
                }
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    let status = child.wait().map(|s| s.to_string()).unwrap_or_default();
                    return Err(fail(&mut child, format!("brep-plm exited before serving ({status})")));
                }
            }
        }
        let Some(password) = seen.password else {
            return Err(fail(&mut child, "brep-plm did not print a first-run administrator password".into()));
        };
        let url = format!("http://{}", seen.listening.as_deref().unwrap_or_default());
        let agent = ureq::AgentBuilder::new().redirects(0).timeout(Duration::from_secs(60)).build();
        let mut fixture = Self {
            child,
            url,
            binary: describe_binary(&bin),
            tokens: BTreeMap::new(),
            app_user: String::new(),
            vars: Mutex::new(BTreeMap::new()),
            data: data.clone(),
            agent,
        };
        // The line is printed once bound; wait for an answer of any kind
        // before seeding all the same.
        loop {
            match fixture.send("GET", "/api/me", &[], None) {
                Ok(_) => break,
                Err(e) if Instant::now() > deadline => return Err(fail(&mut fixture.child, format!("brep-plm never answered: {e}"))),
                Err(_) => std::thread::sleep(Duration::from_millis(50)),
            }
        }
        if let Err(e) = fixture.seed(spec, &password) {
            return Err(fail(&mut fixture.child, format!("seeding the PLM: {e}")));
        }
        if let Err(e) = fixture.configure_app(&spec.app_user, spec.app_token, store) {
            return Err(fail(&mut fixture.child, e));
        }
        Ok(fixture)
    }

    /// Sign in as the first-run administrator, mint every account and token,
    /// then play `setup`.
    fn seed(&mut self, spec: &PlmSpec, password: &str) -> Result<(), String> {
        let login = self.send("POST", "/api/login", &[], Some(json_body(&json!({ "username": "admin", "password": password }))))?;
        if login.status != 200 {
            return Err(format!("admin sign-in answered {}: {}", login.status, String::from_utf8_lossy(&login.body)));
        }
        // The session cookie is the first `name=value` of Set-Cookie.
        let cookie = login
            .headers
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case("set-cookie"))
            .and_then(|(_, v)| v.split(';').next())
            .map(str::to_string)
            .ok_or("admin sign-in set no cookie")?;
        let me = self.send("GET", "/api/me", &[("Cookie", &cookie)], None)?;
        let csrf = me
            .json()
            .and_then(|v| v["csrf"].as_str().map(str::to_string))
            .ok_or_else(|| format!("/api/me gave the session no csrf ({})", me.status))?;
        let session = [("Cookie", cookie.as_str()), ("X-CSRF-Token", csrf.as_str())];
        let mint = |fixture: &Self, name: &str, scope: &str, user_id: Option<&str>| -> Result<String, String> {
            let mut body = json!({ "name": name, "scope": scope });
            if let Some(id) = user_id {
                body["user_id"] = json!(id);
            }
            let made = fixture.send("POST", "/api/tokens", &session, Some(json_body(&body)))?;
            made.json()
                .and_then(|v| v["token"].as_str().map(str::to_string))
                .ok_or_else(|| format!("minting a token for {name} answered {}: {}", made.status, String::from_utf8_lossy(&made.body)))
        };
        let admin = mint(self, "test-mcp admin", "full", None)?;
        self.tokens.insert("admin".into(), admin);
        for user in &spec.users {
            let body = json!({
                "username": user.username,
                "password": user.password.as_deref().unwrap_or(DEFAULT_PASSWORD),
                "groups": user.groups,
            });
            let made = self.send("POST", "/api/users", &session, Some(json_body(&body)))?;
            if made.status >= 400 {
                return Err(format!("creating {} answered {}: {}", user.username, made.status, String::from_utf8_lossy(&made.body)));
            }
            let users = self.send("GET", "/api/users", &session, None)?.json().unwrap_or(Value::Null);
            let id = users
                .as_array()
                .and_then(|list| list.iter().find(|u| u["username"] == json!(user.username)))
                .and_then(|u| u["id"].as_str())
                .ok_or_else(|| format!("{} is not in /api/users after creating it", user.username))?
                .to_string();
            let token = mint(self, &format!("test-mcp {}", user.username), user.scope.as_deref().unwrap_or("full"), Some(&id))?;
            self.tokens.insert(user.username.clone(), token);
        }
        {
            let mut vars = self.vars.lock().unwrap();
            vars.insert("url".into(), Value::String(self.url.clone()));
            for (user, token) in &self.tokens {
                vars.insert(format!("token:{user}"), Value::String(token.clone()));
            }
        }
        for (i, call) in spec.setup.iter().enumerate() {
            let answer = self.call(call)?;
            let ok = match call.status {
                Some(want) => answer.status == want,
                None => answer.status < 400,
            };
            if !ok {
                return Err(format!(
                    "setup {i} ({} {}) answered {}: {}",
                    call.method,
                    call.path,
                    answer.status,
                    String::from_utf8_lossy(&answer.body)
                ));
            }
        }
        Ok(())
    }

    /// Write D5's files for `user` into the store (nothing for an empty name).
    fn configure_app(&mut self, user: &str, with_token: bool, store: &Path) -> Result<(), String> {
        self.app_user = user.to_string();
        if user.is_empty() {
            return Ok(());
        }
        let token = self.tokens.get(user).ok_or_else(|| format!("app_user `{user}` is not an account the fixture made"))?.clone();
        self.vars.lock().unwrap().insert("token".into(), Value::String(token.clone()));
        std::fs::create_dir_all(store).map_err(|e| format!("{}: {e}", store.display()))?;
        let config = store.join("plm.json");
        std::fs::write(&config, serde_json::to_string_pretty(&json!({ "url": self.url })).unwrap() + "\n")
            .map_err(|e| format!("{}: {e}", config.display()))?;
        if with_token {
            let token_path = store.join("plm-token");
            write_secret(&token_path, &format!("{token}\n")).map_err(|e| format!("{}: {e}", token_path.display()))?;
        }
        Ok(())
    }

    /// The token of `user` (`admin`, a created account).
    pub fn token(&self, user: &str) -> Option<&str> {
        self.tokens.get(user).map(String::as_str)
    }

    /// Replace every `${plm:NAME}` in the strings of `value`. An unknown name
    /// is an error, so a typo cannot quietly send the literal text.
    pub fn substitute(&self, value: Value) -> Result<Value, String> {
        let vars = self.vars.lock().unwrap();
        substitute_vars(value, &vars)
    }

    /// Make one call (a `setup` entry or a `plm_request` step) and apply its
    /// `bind`s. Only a request that got no answer is an `Err`.
    pub fn call(&self, call: &PlmCall) -> Result<Answer, String> {
        let path = self.substitute(Value::String(call.path.clone()))?;
        let path = path.as_str().unwrap_or_default().to_string();
        let body = match (&call.body, &call.raw) {
            (Some(_), Some(_)) => return Err("a PLM call takes `body` or `raw`, not both".into()),
            (Some(body), None) => Some(json_body(&self.substitute(body.clone())?)),
            (None, Some(raw)) => {
                let raw = self.substitute(Value::String(raw.clone()))?;
                Some(raw.as_str().unwrap_or_default().as_bytes().to_vec())
            }
            (None, None) => None,
        };
        let who = call.as_user.as_deref().unwrap_or("admin");
        let bearer;
        let mut headers: Vec<(&str, &str)> = Vec::new();
        if who != "anonymous" {
            let token = self.tokens.get(who).ok_or_else(|| format!("`as: {who}` is not an account the fixture made"))?;
            bearer = format!("Bearer {token}");
            headers.push(("Authorization", &bearer));
        }
        if let Some(content_type) = call.content_type.as_deref().filter(|_| call.raw.is_some()) {
            headers.push(("Content-Type", content_type));
        }
        let method = call.method.to_ascii_uppercase();
        let answer = self.send(&method, &path, &headers, body)?;
        if !call.bind.is_empty() {
            let parsed = answer
                .json()
                .ok_or_else(|| format!("{method} {path} answered {} with no JSON to bind from", answer.status))?;
            self.bind(&parsed, &call.bind).map_err(|e| format!("{method} {path} answered {}: {e}", answer.status))?;
        }
        Ok(answer)
    }

    /// Capture values of `from` under names (`name → JSON pointer`), for
    /// `${plm:name}` in later steps: a `plm_request`'s answer, or any step's
    /// result (the step's own `bind`). A pointer that finds nothing fails.
    pub fn bind(&self, from: &Value, names: &BTreeMap<String, String>) -> Result<(), String> {
        let mut vars = self.vars.lock().unwrap();
        for (name, pointer) in names {
            let found = from.pointer(pointer).ok_or_else(|| format!("bind `{name}`: nothing at `{pointer}`"))?;
            vars.insert(name.clone(), found.clone());
        }
        Ok(())
    }

    fn send(&self, method: &str, path: &str, headers: &[(&str, &str)], body: Option<Vec<u8>>) -> Result<Answer, String> {
        let mut request = self.agent.request(method, &format!("{}{}", self.url, path));
        if body.is_some() {
            let is_json = body.as_deref().map(|b| serde_json::from_slice::<Value>(b).is_ok()).unwrap_or(false);
            if is_json {
                request = request.set("Content-Type", "application/json");
            }
        }
        for (name, value) in headers {
            request = request.set(name, value);
        }
        let sent = match body {
            Some(bytes) => request.send_bytes(&bytes),
            None => request.call(),
        };
        let response = match sent {
            Ok(response) => response,
            Err(ureq::Error::Status(_, response)) => response,
            Err(ureq::Error::Transport(e)) => return Err(format!("{method} {path}: {e}")),
        };
        let status = response.status();
        let mut headers = Vec::new();
        for name in response.headers_names() {
            for value in response.all(&name) {
                headers.push((name.clone(), value.to_string()));
            }
        }
        let mut body = Vec::new();
        response.into_reader().read_to_end(&mut body).map_err(|e| format!("{method} {path}: reading the answer: {e}"))?;
        Ok(Answer { status, headers, body })
    }
}

impl Drop for PlmFixture {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
        let _ = std::fs::remove_dir_all(&self.data);
    }
}

/// A fresh, empty data directory for one server: under
/// `BREP_PLM_FIXTURE_DATA`, else `/dev/shm`, else `<root>/data`.
fn data_dir(root: &Path) -> Result<PathBuf, String> {
    let parent = std::env::var_os("BREP_PLM_FIXTURE_DATA")
        .map(PathBuf::from)
        .or_else(|| Some(PathBuf::from("/dev/shm")).filter(|p| cfg!(target_os = "linux") && p.is_dir()));
    let data = match parent {
        Some(parent) => {
            static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
            let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
            parent.join(format!("brep-mcp-plm-{}-{n}-{nanos}", std::process::id()))
        }
        None => root.join("data"),
    };
    if data.exists() {
        std::fs::remove_dir_all(&data).map_err(|e| format!("clear {}: {e}", data.display()))?;
    }
    std::fs::create_dir_all(&data).map_err(|e| format!("{}: {e}", data.display()))?;
    Ok(data)
}

/// `<path> (built 2026-09-25T15:04:05Z)`, from the file's mtime.
fn describe_binary(bin: &Path) -> String {
    let built = std::fs::metadata(bin)
        .and_then(|m| m.modified())
        .ok()
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
        .map(|d| utc_stamp(d.as_secs()))
        .unwrap_or_else(|| "at an unknown time".into());
    let path = bin.canonicalize().unwrap_or_else(|_| bin.to_path_buf());
    format!("{} (built {built})", path.display())
}

/// Seconds since the epoch as an ISO-8601 UTC stamp (Howard Hinnant's
/// days-to-civil), so the fixture needs no date crate.
fn utc_stamp(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z", rem / 3_600, rem % 3_600 / 60, rem % 60)
}

fn json_body(value: &Value) -> Vec<u8> {
    serde_json::to_vec(value).unwrap_or_default()
}

/// Read one line of the server's start-up output.
fn read_startup_line(line: &str, seen: &mut Startup) {
    let trimmed = line.trim();
    if let Some(rest) = trimmed.strip_prefix("password") {
        seen.password = Some(rest.trim().to_string());
    } else if let Some(addr) = trimmed.strip_prefix("brep-plm: listening on ") {
        seen.listening = Some(addr.trim().trim_start_matches("http://").to_string());
    }
}

/// A string that is EXACTLY one variable becomes the variable's JSON value
/// (an object stays an object, a number a number), so an expectation can
/// compare a whole captured value; a variable inside longer text is spliced
/// in as text.
fn substitute_vars(value: Value, vars: &BTreeMap<String, Value>) -> Result<Value, String> {
    let text_of = |v: &Value| match v {
        Value::String(s) => s.clone(),
        other => other.to_string(),
    };
    if let Value::String(s) = &value {
        if let Some(name) = s.strip_prefix("${plm:").and_then(|rest| rest.strip_suffix('}')) {
            if !name.contains('}') && !name.contains("${") {
                if let Some(found) = vars.get(name) {
                    return Ok(found.clone());
                }
            }
        }
    }
    Ok(match value {
        Value::String(s) if s.contains("${plm:") => {
            let mut out = String::new();
            let mut rest = s.as_str();
            while let Some(start) = rest.find("${plm:") {
                out.push_str(&rest[..start]);
                let after = &rest[start + 6..];
                let end = after.find('}').ok_or_else(|| format!("unclosed `${{plm:` in `{s}`"))?;
                let name = &after[..end];
                let found = vars.get(name).ok_or_else(|| {
                    format!("`${{plm:{name}}}` is not known (have: {})", vars.keys().cloned().collect::<Vec<_>>().join(", "))
                })?;
                out.push_str(&text_of(found));
                rest = &after[end + 1..];
            }
            out.push_str(rest);
            Value::String(out)
        }
        Value::Array(a) => Value::Array(a.into_iter().map(|x| substitute_vars(x, vars)).collect::<Result<_, _>>()?),
        Value::Object(o) => {
            Value::Object(o.into_iter().map(|(k, x)| substitute_vars(x, vars).map(|x| (k, x))).collect::<Result<_, _>>()?)
        }
        other => other,
    })
}

/// A secret file: created 0600 on unix, so the token is never world-readable
/// even for the instant between create and chmod.
fn write_secret(path: &Path, text: &str) -> std::io::Result<()> {
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    options.open(path)?.write_all(text.as_bytes())
}

