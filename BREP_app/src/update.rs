//! The desktop app's updates: master's Linux and Windows builds, offered from
//! next.brep.io and installed only with the user's say-so.
//!
//! In the background the app hashes its own executable and asks the server's
//! update channel (`/api/public/app-update/<os>`) for the current build's
//! manifest. The manifest is signed with an ed25519 key whose public half is
//! `PUBLIC_KEY_HEX`; one that does not verify is ignored. When the signed
//! sha256 differs from this executable's, a notice offers **Update and
//! restart**. The download must match the signed length and sha256 before it
//! touches the executable, so a server or connection that hands over
//! anything else can only make an update fail.
//!
//! The executable is replaced by rename beside itself, which a running
//! program allows on both systems: on Linux the new file is renamed over the
//! old one; on Windows the running file is renamed to `<exe>.old` (deleted at
//! the next start) and the new one takes its name. The app then closes
//! through its normal close path, which writes unsaved work for recovery, and
//! starts the new executable with the same arguments.
//!
//! The check never runs for a build out of a cargo `target/` directory or
//! `~/.cargo/bin` (a developer's or a crates.io build, which is not the
//! channel's), with `BREP_NO_UPDATE_CHECK` set, or after **Don't check again**
//! (`update.json` in the app's config directory). `BREP_UPDATE_URL` points the
//! check at another server.

use std::path::{Path, PathBuf};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Mutex;
use std::time::Duration;

use eframe::egui;
use sha2::{Digest, Sha256};

/// The update signing key's public half.
pub const PUBLIC_KEY_HEX: &str = "a2bcdee9fcd8aefba42a566ff1f8eacd3f9ef5504e466580fbf291c81b622b94";

/// The first line of every signed message; the server's `FORMAT`.
pub const FORMAT: &str = "brep-app-update-v1";

/// The server the channel is on, unless `BREP_UPDATE_URL` says otherwise.
pub const DEFAULT_SERVER: &str = "https://next.brep.io";

/// How often a running app asks again (a CAD session can last days).
const CHECK_EVERY: Duration = Duration::from_secs(6 * 60 * 60);

/// A build the channel offers, as its manifest describes it.
#[derive(Clone, Debug, PartialEq, Eq, serde::Deserialize)]
pub struct Manifest {
    pub format: String,
    pub os: String,
    pub filename: String,
    pub sha256: String,
    pub bytes: u64,
    pub commit: String,
    pub built_at: String,
    pub signature: String,
    pub download_path: String,
}

/// What the manifest's signature covers. Byte-for-byte the server's
/// `desktop_update::signed_message`; both sides' tests pin one vector.
pub fn signed_message(os: &str, sha256: &str, bytes: u64, commit: &str, built_at: &str) -> String {
    format!("{FORMAT}\nos={os}\nsha256={sha256}\nbytes={bytes}\ncommit={commit}\nbuilt_at={built_at}\n")
}

fn hex_decode(s: &str) -> Option<Vec<u8>> {
    if s.len() % 2 != 0 {
        return None;
    }
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(s.get(i..i + 2)?, 16).ok()).collect()
}

impl Manifest {
    pub fn parse(json: &str) -> Result<Manifest, String> {
        serde_json::from_str(json).map_err(|e| format!("update manifest: {e}"))
    }

    /// Whether `public_key` signed this manifest, and it is for `os`.
    pub fn verify(&self, public_key_hex: &str, os: &str) -> Result<(), String> {
        if self.format != FORMAT {
            return Err(format!("update manifest format {:?} is not {FORMAT}", self.format));
        }
        if self.os != os {
            return Err(format!("update manifest is for {}, not {os}", self.os));
        }
        if self.sha256.len() != 64 || hex_decode(&self.sha256).is_none() {
            return Err("update manifest sha256 is not 64 hex digits".into());
        }
        let key = hex_decode(public_key_hex).ok_or("bad public key")?;
        let key = ed25519_compact::PublicKey::from_slice(&key).map_err(|e| e.to_string())?;
        let signature = hex_decode(&self.signature).ok_or("update manifest signature is not hex")?;
        let signature = ed25519_compact::Signature::from_slice(&signature).map_err(|e| e.to_string())?;
        let message = signed_message(&self.os, &self.sha256, self.bytes, &self.commit, &self.built_at);
        key.verify(message.as_bytes(), &signature)
            .map_err(|_| "update manifest signature does not verify".to_string())
    }

    /// "2026-11-04 05:00 UTC (commit 3c17b728d)", for the notice.
    pub fn describe(&self) -> String {
        let when = self.built_at.get(..16).unwrap_or(&self.built_at).replace('T', " ");
        let commit = self.commit.get(..9).unwrap_or(&self.commit);
        if commit.is_empty() {
            format!("built {when} UTC")
        } else {
            format!("built {when} UTC, commit {commit}")
        }
    }
}

/// The channel's name for this system, or `None` where it serves no build.
pub fn platform_os() -> Option<&'static str> {
    if cfg!(target_os = "linux") {
        Some("linux")
    } else if cfg!(windows) {
        Some("windows")
    } else {
        None
    }
}

/// A build cargo put where it builds (`…/target/release/brep-app`,
/// `…/target/<triple>/debug/…`, a test binary under `deps/`) or installed
/// (`~/.cargo/bin`): a developer's or a crates.io build, not the channel's.
pub fn is_dev_build(exe: &Path) -> bool {
    let parts: Vec<String> = exe.components().map(|c| c.as_os_str().to_string_lossy().to_ascii_lowercase()).collect();
    let cargo_target = parts.iter().enumerate().any(|(i, p)| {
        p == "target" && parts[i + 1..].iter().take(3).any(|q| q == "release" || q == "debug")
    });
    let cargo_bin = parts.windows(2).any(|w| w[0] == ".cargo" && w[1] == "bin");
    cargo_target || cargo_bin
}

fn settings_path(config_dir: &Path) -> PathBuf {
    config_dir.join("update.json")
}

/// Whether the user said **Don't check again**.
pub fn turned_off(config_dir: &Path) -> bool {
    std::fs::read_to_string(settings_path(config_dir))
        .ok()
        .and_then(|t| serde_json::from_str::<serde_json::Value>(&t).ok())
        .and_then(|v| v.get("check").and_then(|c| c.as_bool()))
        == Some(false)
}

pub fn turn_off(config_dir: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(config_dir)?;
    std::fs::write(settings_path(config_dir), "{\"check\": false}\n")
}

/// Why this run does not check, or `None` when it does.
pub fn disabled_reason(exe: &Path, env: &dyn Fn(&str) -> Option<String>, config_dir: &Path) -> Option<&'static str> {
    if platform_os().is_none() {
        Some("no build is published for this system")
    } else if env("BREP_NO_UPDATE_CHECK").is_some_and(|v| !v.is_empty()) {
        Some("BREP_NO_UPDATE_CHECK is set")
    } else if is_dev_build(exe) {
        Some("a cargo build")
    } else if turned_off(config_dir) {
        Some("turned off")
    } else {
        None
    }
}

pub fn sha256_file(path: &Path) -> std::io::Result<String> {
    let mut file = std::fs::File::open(path)?;
    let mut hasher = Sha256::new();
    std::io::copy(&mut file, &mut hasher)?;
    Ok(format!("{:x}", hasher.finalize()))
}

fn sibling(exe: &Path, suffix: &str) -> PathBuf {
    let mut name = exe.file_name().unwrap_or_default().to_os_string();
    name.push(suffix);
    exe.with_file_name(name)
}

/// What an earlier update left beside the executable: the Windows `.old`
/// (no longer running) and a `.new` an interrupted install wrote.
pub fn clean_leftovers(exe: &Path) {
    let _ = std::fs::remove_file(sibling(exe, ".old"));
    let _ = std::fs::remove_file(sibling(exe, ".new"));
}

/// Put `bytes` in place of `exe`, once they are what `manifest` signed.
pub fn install(exe: &Path, bytes: &[u8], manifest: &Manifest) -> Result<(), String> {
    if bytes.len() as u64 != manifest.bytes {
        return Err(format!("the download is {} bytes, the signed build {}", bytes.len(), manifest.bytes));
    }
    let sha = format!("{:x}", Sha256::digest(bytes));
    if sha != manifest.sha256 {
        return Err("the download's sha256 is not the signed build's".into());
    }
    let new = sibling(exe, ".new");
    let write = || -> std::io::Result<()> {
        std::fs::write(&new, bytes)?;
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(exe).map(|m| m.permissions().mode()).unwrap_or(0o755) | 0o111;
            std::fs::set_permissions(&new, std::fs::Permissions::from_mode(mode))?;
        }
        Ok(())
    };
    if let Err(e) = write() {
        let _ = std::fs::remove_file(&new);
        return Err(format!("cannot write beside {}: {e}", exe.display()));
    }
    swap(exe, &new).map_err(|e| {
        let _ = std::fs::remove_file(&new);
        format!("cannot replace {}: {e}", exe.display())
    })
}

/// A running executable can be renamed but, on Windows, not replaced: move it
/// aside first there, and back if the new one cannot take its place.
fn swap(exe: &Path, new: &Path) -> std::io::Result<()> {
    if cfg!(windows) {
        let old = sibling(exe, ".old");
        let _ = std::fs::remove_file(&old);
        std::fs::rename(exe, &old)?;
        if let Err(e) = std::fs::rename(new, exe) {
            let _ = std::fs::rename(&old, exe);
            return Err(e);
        }
        Ok(())
    } else {
        std::fs::rename(new, exe)
    }
}

/// The executable to start once the window has closed, set by an install.
static RELAUNCH: Mutex<Option<PathBuf>> = Mutex::new(None);

/// After the event loop returns: start the installed build with this run's
/// arguments, if an update was installed.
pub fn relaunch_if_installed() {
    let Some(exe) = RELAUNCH.lock().ok().and_then(|mut r| r.take()) else { return };
    if let Err(e) = std::process::Command::new(&exe).args(std::env::args_os().skip(1)).spawn() {
        eprintln!("brep-app: the update is installed but {} did not start: {e}", exe.display());
    }
}

fn server() -> String {
    std::env::var("BREP_UPDATE_URL")
        .ok()
        .filter(|u| !u.is_empty())
        .unwrap_or_else(|| DEFAULT_SERVER.to_string())
        .trim_end_matches('/')
        .to_string()
}

/// The build on offer, if it is signed and not this one.
fn check(server: &str, os: &str, own_sha: &str) -> Result<Option<Manifest>, String> {
    let response = ehttp::fetch_blocking(&ehttp::Request::get(format!("{server}/api/public/app-update/{os}")))?;
    if !response.ok {
        return Err(format!("update check: HTTP {} {}", response.status, response.status_text));
    }
    let manifest = Manifest::parse(&String::from_utf8_lossy(&response.bytes))?;
    manifest.verify(PUBLIC_KEY_HEX, os)?;
    Ok((manifest.sha256 != own_sha).then_some(manifest))
}

fn download_and_install(server: &str, exe: &Path, manifest: &Manifest) -> Result<(), String> {
    let response = ehttp::fetch_blocking(&ehttp::Request::get(format!("{server}{}", manifest.download_path)))?;
    if !response.ok {
        return Err(format!("download: HTTP {} {}", response.status, response.status_text));
    }
    install(exe, &response.bytes, manifest)
}

enum Msg {
    Available(Manifest),
    Installed,
    Failed(String),
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum Phase {
    Quiet,
    Offered(Manifest),
    Installing,
    Failed(String),
}

/// The background check and the notice it raises.
pub struct Updater {
    exe: Option<PathBuf>,
    rx: Option<Receiver<Msg>>,
    tx: Option<Sender<Msg>>,
    phase: Phase,
    /// The sha256 of a build the user said **Later** to: not offered again.
    declined: Option<String>,
    restart: bool,
}

impl Updater {
    /// An updater that never checks (tests, a dev build, turned off).
    pub fn inert() -> Updater {
        Updater { exe: None, rx: None, tx: None, phase: Phase::Quiet, declined: None, restart: false }
    }

    /// Start checking in the background, unless this run is not the channel's.
    pub fn start(ctx: &egui::Context) -> Updater {
        let Ok(exe) = std::env::current_exe() else { return Updater::inert() };
        // An earlier update's leftovers go even when checks are now off.
        if !is_dev_build(&exe) {
            clean_leftovers(&exe);
        }
        let config_dir = crate::plm::config::app_config_dir();
        if let Some(reason) = disabled_reason(&exe, &|n| std::env::var(n).ok(), &config_dir) {
            log::info!("update check off: {reason}");
            return Updater::inert();
        }
        let os = platform_os().expect("disabled_reason checks the platform");
        let (tx, rx) = channel();
        let (thread_tx, thread_exe, ctx) = (tx.clone(), exe.clone(), ctx.clone());
        let spawned = std::thread::Builder::new().name("brep-update-check".into()).spawn(move || {
            let Ok(own) = sha256_file(&thread_exe) else { return };
            let server = server();
            loop {
                match check(&server, os, &own) {
                    Ok(Some(manifest)) => {
                        if thread_tx.send(Msg::Available(manifest)).is_err() {
                            return;
                        }
                        ctx.request_repaint();
                    }
                    Ok(None) => {}
                    Err(e) => log::info!("{e}"),
                }
                std::thread::sleep(CHECK_EVERY);
            }
        });
        if spawned.is_err() {
            return Updater::inert();
        }
        Updater { exe: Some(exe), rx: Some(rx), tx: Some(tx), phase: Phase::Quiet, declined: None, restart: false }
    }

    fn drain(&mut self) {
        let Some(rx) = &self.rx else { return };
        while let Ok(msg) = rx.try_recv() {
            match msg {
                Msg::Available(m) => {
                    let busy = matches!(self.phase, Phase::Installing);
                    if !busy && self.declined.as_deref() != Some(&m.sha256) {
                        self.phase = Phase::Offered(m);
                    }
                }
                Msg::Installed => {
                    self.phase = Phase::Quiet;
                    if let (Ok(mut r), Some(exe)) = (RELAUNCH.lock(), &self.exe) {
                        *r = Some(exe.clone());
                    }
                    self.restart = true;
                }
                Msg::Failed(e) => self.phase = Phase::Failed(e),
            }
        }
    }

    fn begin_install(&mut self, ctx: &egui::Context, manifest: Manifest) {
        let (Some(exe), Some(tx)) = (self.exe.clone(), self.tx.clone()) else { return };
        self.phase = Phase::Installing;
        let ctx = ctx.clone();
        let _ = std::thread::Builder::new().name("brep-update-install".into()).spawn(move || {
            let msg = match download_and_install(&server(), &exe, &manifest) {
                Ok(()) => Msg::Installed,
                Err(e) => Msg::Failed(e),
            };
            let _ = tx.send(msg);
            ctx.request_repaint();
        });
    }

    /// Draw the notice. `true` once: an update is installed and the app
    /// should close (its normal close path) so the new build can start.
    pub fn show(&mut self, ctx: &egui::Context) -> bool {
        self.drain();
        if std::mem::take(&mut self.restart) {
            return true;
        }
        if self.phase == Phase::Quiet {
            return false;
        }
        let mut next = None;
        egui::Area::new(egui::Id::new("brep-update-notice"))
            .anchor(egui::Align2::RIGHT_BOTTOM, egui::vec2(-12.0, -36.0))
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.set_max_width(360.0);
                    match &self.phase {
                        Phase::Offered(m) => {
                            ui.strong("A newer BREP build is available");
                            ui.label(m.describe());
                            ui.horizontal(|ui| {
                                if ui
                                    .button("Update and restart")
                                    .on_hover_text("Unsaved work is kept and offered again when the app restarts.")
                                    .clicked()
                                {
                                    next = Some(Action::Install(m.clone()));
                                }
                                if ui.button("Later").clicked() {
                                    next = Some(Action::Later(m.sha256.clone()));
                                }
                                if ui.button("Don't check again").clicked() {
                                    next = Some(Action::Off);
                                }
                            });
                        }
                        Phase::Installing => {
                            ui.horizontal(|ui| {
                                ui.spinner();
                                ui.label("Downloading and checking the update…");
                            });
                        }
                        Phase::Failed(e) => {
                            ui.strong("The update was not installed");
                            ui.label(e);
                            ui.label(format!("You can download it from {}.", server()));
                            if ui.button("Close").clicked() {
                                next = Some(Action::Dismiss);
                            }
                        }
                        Phase::Quiet => {}
                    }
                });
            });
        match next {
            Some(Action::Install(m)) => self.begin_install(ctx, m),
            Some(Action::Later(sha)) => {
                self.declined = Some(sha);
                self.phase = Phase::Quiet;
            }
            Some(Action::Off) => {
                if let Err(e) = turn_off(&crate::plm::config::app_config_dir()) {
                    log::warn!("could not save the update setting: {e}");
                }
                self.phase = Phase::Quiet;
                self.rx = None;
            }
            Some(Action::Dismiss) => self.phase = Phase::Quiet,
            None => {}
        }
        if self.phase == Phase::Installing {
            ctx.request_repaint_after(Duration::from_millis(200));
        }
        false
    }
}

enum Action {
    Install(Manifest),
    Later(String),
    Off,
    Dismiss,
}

