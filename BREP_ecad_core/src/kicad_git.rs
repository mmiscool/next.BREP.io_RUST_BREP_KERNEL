//! KiCad's official libraries, cloned with git.
//!
//! KiCad keeps its libraries in three GitLab projects, one per link of the
//! symbol → footprint → 3D-model chain. An import clones each one, shallow and
//! at one pinned release tag, and then reads the clones as an ordinary folder
//! of KiCad files. `git` must be on the PATH; where it is not, or the clone
//! cannot reach GitLab, the caller asks for the folders instead.
//!
//! A clone is written to `<dest>.partial` and renamed to `<dest>` only once git
//! has finished, so a folder at `<dest>` is always a whole checkout and a clone
//! that was stopped or killed leaves nothing that reads as one. The tag is the
//! caller's to put in `dest`, so a different release is a different folder.
//!
//! At 9.0.9.1 the three checkouts are 220 MB, 195 MB and 5.2 GB (measured
//! 2026-10-24); the models are most of it.

#![cfg(not(target_arch = "wasm32"))]

use std::io::Read;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

/// The release every import reads. A 9.x tag: KiCad 10 (the repositories'
/// `master`) stores a symbol library as a `*.kicad_symdir` folder, which the
/// importers do not read; 9.x keeps one `*.kicad_sym` file per library.
pub const TAG: &str = "9.0.9.1";

/// Where the three projects live; a test points [`clone`] at a local copy.
pub const BASE_URL: &str = "https://gitlab.com/kicad/libraries";

/// One of KiCad's three library repositories.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Repo {
    Symbols,
    Footprints,
    Models,
}

impl Repo {
    pub const ALL: [Repo; 3] = [Repo::Symbols, Repo::Footprints, Repo::Models];

    /// The GitLab project's name, which is also the last segment of its URL.
    pub fn project(self) -> &'static str {
        match self {
            Repo::Symbols => "kicad-symbols",
            Repo::Footprints => "kicad-footprints",
            Repo::Models => "kicad-packages3D",
        }
    }

    /// The folder KiCad's own install keeps this repository's files in, so a
    /// root holding all three under these names is laid out like an install.
    pub fn install_folder(self) -> &'static str {
        match self {
            Repo::Symbols => "symbols",
            Repo::Footprints => "footprints",
            Repo::Models => "3dmodels",
        }
    }

    pub fn url(self, base: &str) -> String {
        format!("{}/{}.git", base.trim_end_matches('/'), self.project())
    }
}

/// `git --version`, or why git cannot be run.
pub fn git_version() -> Result<String, String> {
    let output = git()
        .arg("--version")
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
        .map_err(spawn_error)?;
    if !output.status.success() {
        return Err(format!("`git --version` failed ({})", output.status));
    }
    Ok(String::from_utf8_lossy(&output.stdout).trim().to_owned())
}

/// Clone `url` at `tag` into `dest`, unless `dest` is already there.
///
/// `progress` is handed each line git prints about the transfer, for a status
/// label. Setting `cancel` kills git; the partial clone is removed and the
/// answer is an error. `Ok(true)` when this call cloned, `Ok(false)` when the
/// folder was already present.
pub fn clone(
    url: &str,
    tag: &str,
    dest: &Path,
    progress: &mut dyn FnMut(&str),
    cancel: &AtomicBool,
) -> Result<bool, String> {
    if dest.is_dir() {
        return Ok(false);
    }
    let partial = partial_path(dest);
    if partial.exists() {
        std::fs::remove_dir_all(&partial).map_err(|e| format!("{}: {e}", partial.display()))?;
    }
    if let Some(parent) = dest.parent() {
        std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    }
    let mut child = clone_command(url, tag, &partial).spawn().map_err(spawn_error)?;

    // git writes its progress to stderr, rewriting a line with `\r`. A reader
    // thread keeps the pipe drained so git never blocks on it, and this thread
    // stays free to notice `cancel`.
    let mut stderr = child.stderr.take().expect("stderr is piped");
    let (send, lines) = std::sync::mpsc::channel::<String>();
    let reader = std::thread::spawn(move || {
        let mut buffer = [0u8; 4096];
        let mut line = Vec::new();
        while let Ok(read) = stderr.read(&mut buffer) {
            if read == 0 {
                break;
            }
            for &byte in &buffer[..read] {
                if byte == b'\r' || byte == b'\n' {
                    if !line.is_empty() {
                        let _ = send.send(String::from_utf8_lossy(&line).trim().to_owned());
                        line.clear();
                    }
                } else {
                    line.push(byte);
                }
            }
        }
        if !line.is_empty() {
            let _ = send.send(String::from_utf8_lossy(&line).trim().to_owned());
        }
    });

    let mut last = Vec::<String>::new();
    let status = loop {
        match lines.recv_timeout(Duration::from_millis(200)) {
            Ok(line) => {
                progress(&line);
                if last.len() == 4 {
                    last.remove(0);
                }
                last.push(line);
                continue;
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            // git closed stderr and is exiting; do not spin while it does.
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => std::thread::sleep(Duration::from_millis(20)),
        }
        if cancel.load(Ordering::Relaxed) {
            let _ = child.kill();
            let _ = child.wait();
            let _ = reader.join();
            let _ = std::fs::remove_dir_all(&partial);
            return Err(format!("the clone of {url} was stopped"));
        }
        if let Some(status) = child.try_wait().map_err(|e| e.to_string())? {
            let _ = reader.join();
            for line in lines.try_iter() {
                progress(&line);
                last.push(line);
            }
            break status;
        }
    };
    if !status.success() {
        let _ = std::fs::remove_dir_all(&partial);
        let said = last.iter().rev().take(3).rev().cloned().collect::<Vec<_>>().join("; ");
        return Err(format!("git clone {url} at {tag} failed ({status}): {said}"));
    }
    std::fs::rename(&partial, dest).map_err(|e| format!("moving {} to {}: {e}", partial.display(), dest.display()))?;
    Ok(true)
}

/// The `*.kicad_sym` library names in a symbols checkout, without their
/// extension, sorted.
pub fn symbol_libraries(dir: &Path) -> Result<Vec<String>, String> {
    let entries = std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut names: Vec<String> = entries
        .filter_map(Result::ok)
        .filter(|entry| entry.path().is_file())
        .filter_map(|entry| entry.file_name().to_str()?.strip_suffix(".kicad_sym").map(str::to_owned))
        .collect();
    names.sort();
    Ok(names)
}

/// The `git clone` that fetches `url` at `tag` into `partial`: shallow, the one
/// branch, progress on stderr for the caller to read.
///
/// `core.fsync=none`: the clone is a cache, written to `partial` and renamed
/// into place only when whole, and cloned again when it is not there, so
/// git's fsync of the pack buys it nothing. What it costs is a wait on the
/// disk's journal behind every other writer: on a host with the root disk at
/// 100% utilisation (cad-bot, 2026-10-26) the default clone of a three-file
/// repository made 3 fsync calls and took 129 s, the same clone without them
/// 5.5 s. The option precedes the subcommand so git hands it to index-pack and
/// the other helpers it spawns. The trade: an unclean shutdown inside the
/// seconds after the rename can leave a `dest` with a truncated pack, which
/// the user clears by deleting the folder.
fn clone_command(url: &str, tag: &str, partial: &Path) -> Command {
    let mut command = git();
    command
        .args(["-c", "core.autocrlf=false", "-c", "core.longpaths=true", "-c", "advice.detachedHead=false"])
        .args(["-c", "core.fsync=none"])
        .args(["clone", "--progress", "--depth", "1", "--single-branch", "--branch", tag, url])
        .arg(partial)
        .stdout(Stdio::null())
        .stderr(Stdio::piped());
    command
}

fn partial_path(dest: &Path) -> PathBuf {
    let mut name = dest.file_name().unwrap_or_default().to_os_string();
    name.push(".partial");
    dest.with_file_name(name)
}

/// A `git` command that cannot stop to ask for anything: no credential prompt
/// (the projects are public; a prompt means the URL is wrong) and, on Windows,
/// no console window flashing up beside a GUI program.
fn git() -> Command {
    let mut command = Command::new("git");
    command.env("GIT_TERMINAL_PROMPT", "0").stdin(Stdio::null());
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        const CREATE_NO_WINDOW: u32 = 0x0800_0000;
        command.creation_flags(CREATE_NO_WINDOW);
    }
    command
}

fn spawn_error(error: std::io::Error) -> String {
    if error.kind() == std::io::ErrorKind::NotFound {
        "git was not found on the PATH".into()
    } else {
        format!("git could not be started: {error}")
    }
}

