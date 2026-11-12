//! Register this executable as the system's opener for its three document
//! classes: `.nbrep` parts, `.fbrep` family seeds and `.tbrep` templates —
//! the DEFAULT opener for those — and as one AVAILABLE opener ("Open With")
//! for the files it imports (STEP, IGES, STL, 3MF, OBJ), never their default.
//! A file of an import type opened this way is imported into a new untitled
//! tab (`BrepApp::open_path_from_command_line`).
//!
//! `brep-app --install-file-associations` writes the platform's own association
//! records and points them at the RUNNING executable's path
//! (`std::env::current_exe`), so a build the user moved or renamed associates
//! itself rather than a path we guessed. `--uninstall-file-associations` removes
//! what the install wrote.
//!
//! Why this is per-user and never system-wide: writing `/usr/share` or
//! `HKEY_CLASSES_ROOT` needs privileges the app must not ask for, and a CAD
//! viewer that silently wants root is worse than one that associates for the
//! person who ran it. Everything below lands under `$XDG_DATA_HOME` (Linux) or
//! `HKEY_CURRENT_USER` (Windows).
//!
//! Every step reports what it did or why it could not, and a failure in one step
//! does not abandon the others: a desktop entry written but not indexed still
//! works after the next login, so "wrote the entry, could not run
//! update-desktop-database" is a useful outcome rather than an error to roll back.

use std::path::{Path, PathBuf};

/// The MIME type the association declares. `x-` because `.nbrep` is ours and
/// unregistered with IANA; the glob is what actually routes a double click.
pub const MIME_TYPE: &str = "application/x-nbrep";

/// Every class the association declares, one MIME type each in the same
/// family: `(extension without the dot, MIME type, what a file manager calls
/// it, the Windows ProgID)`.
pub const CLASSES: [(&str, &str, &str, &str); 3] = [
    ("nbrep", MIME_TYPE, "BREP model document", "BREP.Model"),
    ("fbrep", "application/x-fbrep", "BREP part family", "BREP.Family"),
    ("tbrep", "application/x-tbrep", "BREP part template", "BREP.Template"),
];

/// The files the app imports that the OS may offer it for, never as their
/// default opener: `(MIME type, its globs, what a file manager calls it)`.
/// The standard `model/*` types (IANA registers `model/step`, `model/iges`,
/// `model/stl`, `model/3mf`, `model/obj`); the install declares them with
/// their globs too, because an older shared-mime-info has no `model/step`
/// and a desktop entry naming an unknown type routes nothing. Declaring a
/// type the system already has merges the globs; it replaces nothing.
/// KiCad files are imported too but have no registered type, and KiCad's own
/// association owns them, so they are not offered here.
pub const IMPORT_TYPES: [(&str, &[&str], &str); 5] = [
    ("model/step", &["step", "stp", "p21"], "STEP 3D model"),
    ("model/iges", &["iges", "igs"], "IGES 3D model"),
    ("model/stl", &["stl"], "STL mesh"),
    ("model/3mf", &["3mf"], "3MF mesh package"),
    ("model/obj", &["obj"], "OBJ mesh"),
];

/// The Windows ProgID the import types are offered under (`OpenWithProgids`).
#[allow(dead_code)] // Windows only
const IMPORT_PROG_ID: &str = "BREP.Import";

/// The desktop entry's file name (Linux) — also the id `xdg-mime` takes.
const DESKTOP_FILE: &str = "brep-app.desktop";

/// What one step of the install did. Collected so the caller prints the whole
/// story: a partial install is the common case (no `update-mime-database` on a
/// minimal box) and the user needs to know which half took.
pub struct Step {
    pub what: String,
    pub outcome: Result<String, String>,
    /// A refresh the desktop would otherwise do at the next login
    /// (`update-mime-database`, `update-desktop-database`, `xdg-mime`). Its
    /// failure is a warning: the association's records were written and work
    /// without it. Only a record that could not be written is a failure.
    pub optional: bool,
}

impl Step {
    fn done(what: impl Into<String>, detail: impl Into<String>) -> Self {
        Step { what: what.into(), outcome: Ok(detail.into()), optional: false }
    }
    fn failed(what: impl Into<String>, why: impl Into<String>) -> Self {
        Step { what: what.into(), outcome: Err(why.into()), optional: false }
    }
    #[allow(dead_code)] // the refresh helpers exist on Linux only
    fn optional(mut self) -> Self {
        self.optional = true;
        self
    }
}

/// The steps that failed and are NOT optional: what the command's exit status
/// counts. A missing refresh tool is a warning, not one of these.
pub fn failures(steps: &[Step]) -> usize {
    steps.iter().filter(|s| s.outcome.is_err() && !s.optional).count()
}

/// Render the steps for the terminal, and say plainly whether the association
/// is expected to work now, later, or not at all.
pub fn report(action: &str, steps: &[Step]) -> String {
    let mut out = String::new();
    for step in steps {
        match &step.outcome {
            Ok(detail) if detail.is_empty() => out.push_str(&format!("  ok    {}\n", step.what)),
            Ok(detail) => out.push_str(&format!("  ok    {} — {}\n", step.what, detail)),
            Err(why) if step.optional => out.push_str(&format!("  WARN  {} — {}\n", step.what, why)),
            Err(why) => out.push_str(&format!("  FAIL  {} — {}\n", step.what, why)),
        }
    }
    let failed = failures(steps);
    let warned = steps.iter().filter(|s| s.outcome.is_err() && s.optional).count();
    out.push_str(&match (action, failed) {
        ("install", 0) if warned > 0 => format!(
            "\nThe association is written. {warned} refresh step(s) did not run, so the desktop may \
             notice .nbrep, .fbrep and .tbrep only at the next login; to finish now, run\n    \
             xdg-mime default {DESKTOP_FILE} {MIME_TYPE}\n"
        ),
        ("install", 0) => format!(
            "\n.nbrep, .fbrep and .tbrep documents now open in this build. MIME types {}.\n\
             It is also offered (Open With, not the default) for {}: they are imported into a new tab.\n",
            CLASSES.map(|(_, mime, _, _)| mime).join(", "),
            IMPORT_TYPES.map(|(mime, _, _)| mime).join(", ")
        ),
        ("install", _) => format!(
            "\n{failed} step(s) failed. What was written still counts: a desktop entry that could not be \
             indexed usually takes effect at the next login, and you can finish by hand with\n    \
             xdg-mime default {DESKTOP_FILE} {MIME_TYPE}\n"
        ),
        ("uninstall", 0) => "\nthe .nbrep / .fbrep / .tbrep association written by this build is gone.\n".to_string(),
        (_, _) => format!("\n{failed} step(s) failed; anything listed ok above was removed.\n"),
    });
    out
}

/// This executable's path, resolved through symlinks so the desktop entry names
/// the real binary rather than a link that may be replaced by another build.
fn exe_path() -> Result<PathBuf, String> {
    let exe = std::env::current_exe().map_err(|e| format!("cannot read my own path: {e}"))?;
    Ok(std::fs::canonicalize(&exe).unwrap_or(exe))
}

/// Install the association. Returns the steps, in the order attempted.
pub fn install() -> Vec<Step> {
    match exe_path() {
        Ok(exe) => platform::install(&exe),
        Err(e) => vec![Step::failed("locate this executable", e)],
    }
}

/// Remove what [`install`] wrote.
pub fn uninstall() -> Vec<Step> {
    platform::uninstall()
}

// --- Linux / BSD: XDG desktop entry + shared MIME info -------------------------
#[cfg(all(unix, not(target_os = "macos")))]
mod platform {
    use super::*;

    fn data_home() -> PathBuf {
        std::env::var_os("XDG_DATA_HOME")
            .map(PathBuf::from)
            .filter(|p| p.is_absolute())
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))
            .unwrap_or_else(|| PathBuf::from(".local/share"))
    }

    fn write(path: &Path, contents: &str, what: &str) -> Step {
        if let Some(parent) = path.parent() {
            if let Err(e) = std::fs::create_dir_all(parent) {
                return Step::failed(what, format!("create {}: {e}", parent.display()));
            }
        }
        match std::fs::write(path, contents) {
            Ok(()) => Step::done(what, path.display().to_string()),
            Err(e) => Step::failed(what, format!("write {}: {e}", path.display())),
        }
    }

    /// Run a refresh helper that may not be installed. It is an OPTIONAL
    /// step: a missing or failing tool is a WARN with the reason rather than a
    /// silent skip, because the user needs to know their desktop will only
    /// notice the new type after a re-login. It does not fail the command.
    /// `path` overrides `PATH` for the tests.
    fn run(program: &str, args: &[&str], path: Option<&std::ffi::OsStr>) -> Step {
        run_required(program, args, path).optional()
    }

    fn run_required(program: &str, args: &[&str], path: Option<&std::ffi::OsStr>) -> Step {
        let what = format!("{program} {}", args.join(" "));
        let mut command = std::process::Command::new(program);
        if let Some(path) = path {
            command.env("PATH", path);
        }
        match command.args(args).output() {
            Ok(out) if out.status.success() => Step::done(what, String::new()),
            Ok(out) => {
                let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
                Step::failed(what, if err.is_empty() { format!("exit {}", out.status) } else { err })
            }
            Err(e) => Step::failed(what, format!("{e} (not installed?)")),
        }
    }

    // Desktop Entry Exec quoting, followed by the key-file string escaping.
    // This is not shell quoting: the desktop launcher parses the argument itself.
    pub(super) fn exec_argument(exe: &Path) -> Result<String, String> {
        let text = exe.to_str().ok_or("desktop executable path must be UTF-8")?;
        if text.contains(['=', '\n', '\r', '\0']) {
            return Err("desktop executable path cannot contain '=', a newline or NUL".into());
        }
        let mut quoted = String::from("\"");
        for c in text.chars() {
            if c == '%' { quoted.push_str("%%"); continue; }
            if matches!(c, '\\' | '"' | '$' | '`') { quoted.push('\\'); }
            quoted.push(c);
        }
        quoted.push('"');
        Ok(quoted.replace('\\', "\\\\").replace('\t', "\\t"))
    }

    pub fn install(exe: &Path) -> Vec<Step> {
        install_at(exe, &data_home(), None)
    }

    /// [`install`] into `data` (an `XDG_DATA_HOME`), with the helpers found on
    /// `path` when given.
    pub(super) fn install_at(exe: &Path, data: &Path, path: Option<&std::ffi::OsStr>) -> Vec<Step> {
        let executable = match exec_argument(exe) {
            Ok(value) => value,
            Err(error) => return vec![Step::failed("desktop executable path", error)],
        };
        let data = data.to_path_buf();
        let mime_dir = data.join("mime");
        let mut steps = Vec::new();

        // The types themselves, one per class. `*.nbrep` (and its two
        // siblings) is the routing key; the comment is what a file manager
        // shows in its "Type" column.
        let types: String = CLASSES
            .iter()
            .map(|(ext, mime, comment, _)| {
                format!(
                    "  <mime-type type=\"{mime}\">\n    <comment>{comment}</comment>\n    <glob pattern=\"*.{ext}\"/>\n  </mime-type>\n"
                )
            })
            .collect();
        let import_types: String = IMPORT_TYPES
            .iter()
            .map(|(mime, globs, comment)| {
                let globs: String = globs.iter().map(|ext| format!("    <glob pattern=\"*.{ext}\"/>\n")).collect();
                format!("  <mime-type type=\"{mime}\">\n    <comment>{comment}</comment>\n{globs}  </mime-type>\n")
            })
            .collect();
        steps.push(write(
            &mime_dir.join("packages/brep-app.xml"),
            &format!(
                "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n\
                 <mime-info xmlns=\"http://www.freedesktop.org/standards/shared-mime-info\">\n\
                 {types}{import_types}</mime-info>\n"
            ),
            "declare the .nbrep / .fbrep / .tbrep MIME types",
        ));
        // The native classes first, then the import types: listing a type
        // here makes the app an available opener for it; only the
        // `xdg-mime default` calls below (native classes only) make it the
        // default.
        let mime_types: String = CLASSES
            .iter()
            .map(|(_, mime, _, _)| mime)
            .chain(IMPORT_TYPES.iter().map(|(mime, _, _)| mime))
            .map(|mime| format!("{mime};"))
            .collect();

        // The application. `%f` (a single local path) rather than `%F`: the
        // window opens one document per launch, and `%U` would hand us URLs we
        // cannot read.
        steps.push(write(
            &data.join("applications").join(DESKTOP_FILE),
            &format!(
                "[Desktop Entry]\n\
                 Type=Application\n\
                 Name=BREP\n\
                 Comment=Parametric B-rep CAD\n\
                 Exec={} %f\n\
                 Terminal=false\n\
                 Categories=Graphics;Engineering;3DGraphics;\n\
                 MimeType={mime_types}\n\
                 StartupNotify=true\n",
                executable
            ),
            "write the desktop entry",
        ));

        steps.push(run("update-mime-database", &[&mime_dir.to_string_lossy()], path));
        steps.push(run(
            "update-desktop-database",
            &[&data.join("applications").to_string_lossy()],
            path,
        ));
        for (_, mime, _, _) in CLASSES {
            steps.push(run("xdg-mime", &["default", DESKTOP_FILE, mime], path));
        }
        steps
    }

    pub fn uninstall() -> Vec<Step> {
        let data = data_home();
        let mime_dir = data.join("mime");
        let mut steps = Vec::new();
        for (path, what) in [
            (mime_dir.join("packages/brep-app.xml"), "remove the MIME type"),
            (data.join("applications").join(DESKTOP_FILE), "remove the desktop entry"),
        ] {
            steps.push(match std::fs::remove_file(&path) {
                Ok(()) => Step::done(what, path.display().to_string()),
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                    Step::done(what, format!("{} was already gone", path.display()))
                }
                Err(e) => Step::failed(what, format!("remove {}: {e}", path.display())),
            });
        }
        steps.push(run("update-mime-database", &[&mime_dir.to_string_lossy()], None));
        steps.push(run(
            "update-desktop-database",
            &[&data.join("applications").to_string_lossy()],
            None,
        ));
        steps
    }
}

// --- Windows: per-user file class under HKEY_CURRENT_USER ----------------------
// Written with `reg.exe` rather than a registry crate: no new dependency, and the
// same commands a user can read, check and undo by hand.
#[cfg(windows)]
mod platform {
    use super::*;

    fn reg(args: &[&str]) -> Step {
        let what = format!("reg {}", args.join(" "));
        match std::process::Command::new("reg").args(args).output() {
            Ok(out) if out.status.success() => Step::done(what, String::new()),
            Ok(out) => {
                let err = String::from_utf8_lossy(&out.stderr).trim().to_string();
                Step::failed(what, if err.is_empty() { format!("exit {}", out.status) } else { err })
            }
            Err(e) => Step::failed(what, e.to_string()),
        }
    }

    pub fn install(exe: &Path) -> Vec<Step> {
        let open = format!("\"{}\" \"%1\"", exe.display());
        let icon = format!("\"{}\",0", exe.display());
        let mut steps = Vec::new();
        for (ext, mime, comment, prog_id) in CLASSES {
            steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\{prog_id}"), "/ve", "/d", comment, "/f"]));
            steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\{prog_id}\DefaultIcon"), "/ve", "/d", &icon, "/f"]));
            steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\{prog_id}\shell\open\command"), "/ve", "/d", &open, "/f"]));
            steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\.{ext}"), "/ve", "/d", prog_id, "/f"]));
            steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\.{ext}"), "/v", "Content Type", "/d", mime, "/f"]));
        }
        // The import types: an "Open with" choice, never the default. Only
        // the extension's OpenWithProgids list gains a value; its default
        // ProgID (whatever owns .step today) is left alone.
        steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\{IMPORT_PROG_ID}"), "/ve", "/d", "BREP import", "/f"]));
        steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\{IMPORT_PROG_ID}\DefaultIcon"), "/ve", "/d", &icon, "/f"]));
        steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\{IMPORT_PROG_ID}\shell\open\command"), "/ve", "/d", &open, "/f"]));
        for (_, globs, _) in IMPORT_TYPES {
            for ext in globs {
                steps.push(reg(&["add", &format!(r"HKCU\Software\Classes\.{ext}\OpenWithProgids"), "/v", IMPORT_PROG_ID, "/t", "REG_NONE", "/f"]));
            }
        }
        steps
    }

    pub fn uninstall() -> Vec<Step> {
        let mut steps = Vec::new();
        for (ext, _, _, prog_id) in CLASSES {
            steps.push(reg(&["delete", &format!(r"HKCU\Software\Classes\.{ext}"), "/f"]));
            steps.push(reg(&["delete", &format!(r"HKCU\Software\Classes\{prog_id}"), "/f"]));
        }
        // Only the values the install added: another program's association
        // of the same extension stays.
        for (_, globs, _) in IMPORT_TYPES {
            for ext in globs {
                steps.push(reg(&["delete", &format!(r"HKCU\Software\Classes\.{ext}\OpenWithProgids"), "/v", IMPORT_PROG_ID, "/f"]));
            }
        }
        steps.push(reg(&["delete", &format!(r"HKCU\Software\Classes\{IMPORT_PROG_ID}"), "/f"]));
        steps
    }
}

// --- macOS: the association lives in a bundle's Info.plist ---------------------
// A bare binary cannot own a document type there, so this says so rather than
// writing something that silently does nothing.
#[cfg(target_os = "macos")]
mod platform {
    use super::*;

    pub fn install(_exe: &Path) -> Vec<Step> {
        vec![Step::failed(
            "register .nbrep / .fbrep / .tbrep",
            "macOS routes documents by a .app bundle's CFBundleDocumentTypes, which a bare binary has none of; \
             ship brep-app inside a bundle and declare the type there",
        )]
    }

    pub fn uninstall() -> Vec<Step> {
        vec![Step::done("register .nbrep / .fbrep / .tbrep", "nothing was ever written on macOS")]
    }
}

