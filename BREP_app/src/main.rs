//! Native entry: `brep-app [--mcp [--mcp-port PORT] [--session-root DIR]]`.
//!
//! Opens a real OS window (winit, via eframe) whose wgpu device drives the
//! shared `brep-render` engine. Needs a display + GPU to actually run; on a
//! headless box this still `cargo build`s (the CI/gate signal), and the web
//! target (`lib.rs` + `WebRunner`) is the interactive verification path.
//!
//! `--mcp` embeds the MCP server (src/mcp.rs): the window becomes a session an
//! AI agent drives over `http://127.0.0.1:PORT/mcp`, and the terminal prints
//! how to configure Claude Code or Codex for it.
//!
//! `--kicad-library` is the one flag that opens a DIFFERENT window: the minimal
//! KiCad importer (`panels/kicad_library.rs`), which downloads KiCad's own
//! libraries or reads the install on this machine, and writes each part it
//! imports into the model store as a document. No viewport, no workbench —
//! which is also why it runs where the CAD window might not.
//!
//! On Windows the executable is a GUI program (`windows_subsystem`), so a
//! Start-menu launch opens no console window beside the CAD window. Given any
//! argument it attaches to the console it was started from
//! ([`windows_console::attach_parent`]), so `--version`, `--plm-check`, the
//! other terminal commands and `--mcp`'s instructions still print there, and
//! their exit codes reach the caller. From Explorer there is no such console
//! and nothing changes.

// A GUI program on Windows: no console window of its own.
#![cfg_attr(windows, windows_subsystem = "windows")]

#[cfg(not(target_arch = "wasm32"))]
const USAGE: &str = "usage: brep-app [FILE] [--mcp] [--mcp-port PORT] [--session-root DIR]
       brep-app --kicad-library
       brep-app --install-file-associations | --uninstall-file-associations
       brep-app --plm-configure URL [--plm-token-file FILE] | --plm-forget | --plm-show | --plm-check
       brep-app --bake-worker [--plm-url URL] [--plm-token-file FILE] [--once | --poll SECS]

  FILE                open this document in a tab at startup (what a file
                      association hands us on a double click): a part (.nbrep),
                      a family seed (.fbrep) or a template (.tbrep)
  --mcp               embed the MCP server: agents drive this window over http://127.0.0.1:PORT/mcp
  --mcp-port PORT     the port to listen on (default 8765)
  --session-root DIR  where MCP sessions keep screenshots and call logs (default: the OS temp dir)
  --kicad-library     open the KiCad importer instead of the CAD window: download
                      KiCad's symbol, footprint and 3D-model libraries, or import
                      parts from the KiCad installed here, saving each as a document
  --install-file-associations
                      make THIS executable the opener for .nbrep, .fbrep and
                      .tbrep files, for the current user, and print what was
                      written
  --uninstall-file-associations
                      remove that association again
  --plm-url URL       connect this session to the PLM server at URL (overrides
                      BREP_PLM_URL, which overrides plm.json in the config dir)
  --plm-token-file FILE
                      read this session's PLM API token from FILE (overrides
                      BREP_PLM_TOKEN, which overrides the config dir's plm-token)
  --plm-configure URL write plm.json (and, with --plm-token-file, plm-token at mode
                      0600) into the config dir and exit: the no-prompt setup an
                      installer script runs. With no PLM configured the app is
                      the file-based app
  --plm-forget        remove plm.json and plm-token from the config dir and exit
  --plm-show          print the PLM server this launch would use, and from where,
                      and exit (the token is never printed)
  --plm-check         sign in to that server and check it serves this version of
                      the app, exactly as a window launch would, and exit: 0 when
                      it connects, 1 with the refusal when it does not
  --version           print `brep-app <version>` and exit
  --bake-worker       run as the PLM's headless bake worker, with no window:
                      build each queued family member or template copy, keep
                      it or report why it failed, one line per job. It finds
                      its server and token as --plm-url / --plm-token-file do
    --once            one pass over the queue, then exit
    --poll SECS       seconds between passes when the queue is empty (default 30)
  -h, --help          this text";

/// What the command line asked for.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, PartialEq, Default)]
struct Launch {
    mcp: bool,
    mcp_port: Option<u16>,
    session_root: Option<std::path::PathBuf>,
    help: bool,
    /// `--version`: print the app's version and exit.
    version: bool,
    /// A document to open at startup: the positional argument, which is also
    /// what a `.desktop` `Exec=… %f` or a Windows `shell\open\command` passes.
    open: Option<std::path::PathBuf>,
    /// `Some(true)` install, `Some(false)` uninstall. Neither opens a window.
    associations: Option<bool>,
    /// Open the KiCad importer instead of the CAD window.
    kicad_library: bool,
    /// `--plm-url` / `--plm-token-file`: this launch's PLM overrides (plan D5).
    plm: brep_app::plm::config::Flags,
    /// A terminal PLM config command; none of them opens a window.
    plm_command: Option<PlmCommand>,
}

/// `--plm-configure URL`, `--plm-forget`, `--plm-show`.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, PartialEq)]
enum PlmCommand {
    Configure(String),
    Forget,
    Show,
    Check,
}

#[cfg(not(target_arch = "wasm32"))]
fn parse_args(args: impl IntoIterator<Item = String>) -> Result<Launch, String> {
    let mut launch = Launch::default();
    let mut args = args.into_iter();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--mcp" => launch.mcp = true,
            "--mcp-port" => {
                let value = args.next().ok_or("--mcp-port needs a port number")?;
                launch.mcp_port = Some(value.parse().map_err(|_| format!("--mcp-port: `{value}` is not a port number"))?);
            }
            "--session-root" => {
                launch.session_root = Some(args.next().ok_or("--session-root needs a directory")?.into());
            }
            "--kicad-library" => launch.kicad_library = true,
            "--install-file-associations" => launch.associations = Some(true),
            "--uninstall-file-associations" => launch.associations = Some(false),
            "--plm-url" => launch.plm.url = Some(args.next().ok_or("--plm-url needs the server's URL")?),
            "--plm-token-file" => {
                launch.plm.token_file = Some(args.next().ok_or("--plm-token-file needs a file")?.into());
            }
            "--plm-configure" => {
                let url = args.next().ok_or("--plm-configure needs the server's URL")?;
                set_plm_command(&mut launch, PlmCommand::Configure(url))?;
            }
            "--plm-forget" => set_plm_command(&mut launch, PlmCommand::Forget)?,
            "--plm-show" => set_plm_command(&mut launch, PlmCommand::Show)?,
            "--plm-check" => set_plm_command(&mut launch, PlmCommand::Check)?,
            "-h" | "--help" => launch.help = true,
            "--version" => launch.version = true,
            // A lone path is the document to open. Anything else beginning with
            // `-` is a typo rather than a file name, and saying so beats opening
            // a window that silently ignored it.
            other if other.starts_with('-') => return Err(format!("unknown argument `{other}`")),
            path if launch.open.is_none() => launch.open = Some(path.into()),
            extra => return Err(format!("only one document can be opened at a time (`{extra}` is the second)")),
        }
    }
    if (launch.mcp_port.is_some() || launch.session_root.is_some()) && !launch.mcp {
        return Err("--mcp-port and --session-root need --mcp".into());
    }
    if launch.associations.is_some() && (launch.mcp || launch.open.is_some() || launch.kicad_library) {
        return Err("--install-file-associations / --uninstall-file-associations do not open a window".into());
    }
    // The importer is a window of its own: it has no document to open and no
    // automation session to be driven as.
    if launch.kicad_library && (launch.mcp || launch.open.is_some()) {
        return Err("--kicad-library opens the KiCad importer, not a document or an MCP session".into());
    }
    if let Some(command) = &launch.plm_command {
        if launch.mcp || launch.open.is_some() || launch.kicad_library || launch.associations.is_some() {
            return Err("--plm-configure / --plm-forget / --plm-show / --plm-check do not open a window".into());
        }
        // Configure writes the URL it names; a second URL would be ignored.
        if matches!(command, PlmCommand::Configure(_)) && launch.plm.url.is_some() {
            return Err("--plm-configure takes the URL itself; drop --plm-url".into());
        }
        if matches!(command, PlmCommand::Forget) && (launch.plm.url.is_some() || launch.plm.token_file.is_some()) {
            return Err("--plm-forget takes no --plm-url or --plm-token-file".into());
        }
    }
    Ok(launch)
}

#[cfg(not(target_arch = "wasm32"))]
fn set_plm_command(launch: &mut Launch, command: PlmCommand) -> Result<(), String> {
    if launch.plm_command.is_some() {
        return Err("give one of --plm-configure, --plm-forget, --plm-show, --plm-check".into());
    }
    launch.plm_command = Some(command);
    Ok(())
}

/// Run a terminal PLM config command against the app's config directory and
/// exit: 0 when it did what it says, 1 when it could not (the sentence is on
/// stderr). No prompt, so an installer script can run it unattended.
#[cfg(not(target_arch = "wasm32"))]
fn run_plm_command(command: &PlmCommand, flags: &brep_app::plm::config::Flags) -> ! {
    use brep_app::plm::config;
    let dir = config::app_config_dir();
    let result = match command {
        PlmCommand::Configure(url) => {
            let token = match &flags.token_file {
                Some(path) => std::fs::read_to_string(path)
                    .map(Some)
                    .map_err(|e| format!("--plm-token-file {}: {e}", path.display())),
                None => Ok(None),
            };
            token
                .and_then(|token| config::write(&dir, url, token.as_deref()))
                .and_then(|()| config::resolve(&dir, &|_| None, &config::Flags::default()))
                .map(|written| config::describe(written.as_ref(), &dir))
                .map(|shown| format!("wrote {}\n{shown}", dir.display()))
        }
        PlmCommand::Forget => config::forget(&dir).map(|removed| {
            if removed.is_empty() {
                format!("no PLM configured in {}: nothing to remove\n", dir.display())
            } else {
                removed.iter().map(|p| format!("removed {}\n", p.display())).collect()
            }
        }),
        PlmCommand::Show => config::resolve(&dir, &|name| std::env::var(name).ok(), flags)
            .map(|resolved| config::describe(resolved.as_ref(), &dir)),
        PlmCommand::Check => config::resolve(&dir, &|name| std::env::var(name).ok(), flags).and_then(|resolved| {
            let config = resolved.ok_or_else(|| config::describe(None, &dir).trim_end().to_string())?;
            let url = config.url.clone();
            brep_app::store::check_native_plm(dir.clone(), config)
                .map(|()| format!("connected to {url}: signed in, and it serves brep-app {}\n", env!("CARGO_PKG_VERSION")))
                .map_err(|e| format!("PLM at {url}: {e}"))
        }),
    };
    match result {
        Ok(text) => {
            print!("{text}");
            std::process::exit(0)
        }
        Err(e) => {
            eprintln!("brep-app: {e}");
            std::process::exit(1)
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    #[cfg(windows)]
    if std::env::args_os().len() > 1 {
        windows_console::attach_parent();
    }
    let _ = brep_app::logger::try_init();
    // The PLM bake worker (plan S10) takes its own flags and opens no window.
    if std::env::args().nth(1).as_deref() == Some("--bake-worker") {
        std::process::exit(brep_app::plm::bake::worker_main(std::env::args().skip(2)));
    }
    let launch = match parse_args(std::env::args().skip(1)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("brep-app: {e}\n{USAGE}");
            std::process::exit(2);
        }
    };
    if launch.help {
        println!("{USAGE}");
        return Ok(());
    }
    // One line an installer script compares with the server's
    // `min_client_version` (plan S12).
    if launch.version {
        println!("brep-app {}", env!("CARGO_PKG_VERSION"));
        return Ok(());
    }

    // The association commands are terminal: they write the platform's records
    // and exit, opening no window. A step that failed exits non-zero so a
    // packaging script can tell, while still printing what DID land.
    if let Some(install) = launch.associations {
        let (action, steps) = if install {
            ("install", brep_app::file_association::install())
        } else {
            ("uninstall", brep_app::file_association::uninstall())
        };
        print!("{}", brep_app::file_association::report(action, &steps));
        // A missing refresh tool is a warning; only a record that could not
        // be written fails the command.
        let failed = brep_app::file_association::failures(&steps);
        std::process::exit(if failed == 0 { 0 } else { 1 });
    }

    if let Some(command) = &launch.plm_command {
        run_plm_command(command, &launch.plm);
    }

    // The PLM this launch would use (plan D5). No URL from any source is
    // `Ok(None)`, having read nothing but two absent files: the file-based app,
    // built exactly as before the PLM existed. A PLM asked for but unusable as
    // written is said here and on the first frame, and the session starts on
    // the file stores — a PLM never makes the app fail to start.
    let plm = brep_app::plm::config::resolve(
        &brep_app::plm::config::app_config_dir(),
        &|name| std::env::var(name).ok(),
        &launch.plm,
    );
    match &plm {
        Ok(None) => {}
        Ok(Some(config)) => log::info!("PLM configured: {} (from the {})", config.url, config.url_source.describe()),
        Err(e) => eprintln!("brep-app: PLM config: {e}; this session uses the file stores"),
    }

    #[cfg(feature = "mcp")]
    let mcp = if launch.mcp {
        // Bind first: a port in use is a launch error, not a window without
        // its server. The instructions print from the address actually bound.
        let port = launch.mcp_port.unwrap_or(brep_app::mcp::DEFAULT_PORT);
        let listener = match brep_app::mcp::bind(port) {
            Ok(l) => l,
            Err(e) => {
                eprintln!("brep-app: {e}");
                std::process::exit(1);
            }
        };
        let session_root = launch.session_root.clone().unwrap_or_else(brep_app::mcp::default_session_root);
        println!("{}", brep_app::mcp::agent_instructions(&brep_app::mcp::url(&listener), &session_root));
        // The state registry publishes only for a host; this window has one.
        brep_app::automation::registry::set_enabled(true);
        Some(brep_app::mcp::Launch { listener, session_root })
    } else {
        None
    };
    #[cfg(not(feature = "mcp"))]
    if launch.mcp {
        eprintln!("brep-app: this build has no MCP server (built without the `mcp` feature)");
        std::process::exit(1);
    }

    let native_options = eframe::NativeOptions {
        // egui's frame render pass stays single-sample so the viewport blit
        // pipeline (also single-sample) matches; the engine does its own 4x MSAA
        // inside the offscreen render.
        multisampling: 0,
        // Toolbar controls wrap to the available width on native and web.
        ..Default::default()
    };
    eframe::run_native(
        "brep-app",
        native_options,
        Box::new(move |cc| {
            brep_app::fonts::install(&cc.egui_ctx);
            let plm_store = |store| brep_app::automation::AppOptions { store: Some(store), ..Default::default() };
            #[allow(unused_mut)]
            let mut app = match plm {
                Ok(None) => brep_app::app::BrepApp::new(cc)?,
                // Hydrated before the first frame, as the browser hydrates
                // IndexedDB before `WebRunner::start`: every read from here on
                // is synchronous and assumes a complete mirror.
                Ok(Some(config)) => {
                    let ctx = cc.egui_ctx.clone();
                    brep_app::plm::native::set_wake(move || ctx.request_repaint());
                    let store = brep_app::store::boot_native_store(brep_app::plm::config::app_config_dir(), config);
                    brep_app::app::BrepApp::new_with(cc, plm_store(store))?
                }
                Err(e) => {
                    let store = brep_app::store::file_store_with_notice(
                        brep_app::plm::config::app_config_dir(),
                        format!("PLM config: {e}"),
                    );
                    brep_app::app::BrepApp::new_with(cc, plm_store(store))?
                }
            };
            // A document named on the command line — or handed over by the file
            // association on a double click — opens before the first frame, so
            // the window never shows the seed document and then swap it.
            if let Some(path) = launch.open.as_deref() {
                app.open_path_from_command_line(path);
            }
            #[cfg(feature = "mcp")]
            if let Some(launch) = mcp {
                // The app's own diagnostics, not a second reading of the same
                // adapter: the banner a host sees and the Info window a user
                // sees must name the same hardware.
                let adapter = app.diagnostics().adapter_line();
                brep_app::mcp::start(app.automation().clone(), adapter, launch)?;
            }
            Ok(Box::new(app) as Box<dyn eframe::App>)
        }),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {}

/// The parent's console, for a GUI-subsystem executable run from a terminal
/// or a script. kernel32 is always linked, so the three calls are declared
/// here rather than taking a Windows API crate for them.
#[cfg(windows)]
mod windows_console {
    use std::ffi::c_void;

    const ATTACH_PARENT_PROCESS: u32 = u32::MAX; // (DWORD)-1
    const STD_OUTPUT_HANDLE: u32 = -11i32 as u32;
    const STD_ERROR_HANDLE: u32 = -12i32 as u32;
    const GENERIC_READ: u32 = 0x8000_0000;
    const GENERIC_WRITE: u32 = 0x4000_0000;
    const FILE_SHARE_READ: u32 = 1;
    const FILE_SHARE_WRITE: u32 = 2;
    const OPEN_EXISTING: u32 = 3;
    const INVALID_HANDLE_VALUE: *mut c_void = -1isize as *mut c_void;

    #[link(name = "kernel32")]
    extern "system" {
        fn AttachConsole(process: u32) -> i32;
        fn GetStdHandle(which: u32) -> *mut c_void;
        fn SetStdHandle(which: u32, handle: *mut c_void) -> i32;
        fn CreateFileW(
            name: *const u16,
            access: u32,
            share: u32,
            security: *mut c_void,
            disposition: u32,
            flags: u32,
            template: *mut c_void,
        ) -> *mut c_void;
    }

    /// Attach to the console of the process that started this one, if it has
    /// one. Output a caller REDIRECTED (a script capturing `--version`) keeps
    /// its pipe: only a stream with no handle is pointed at the console.
    pub fn attach_parent() {
        // SAFETY: plain Win32 calls with valid arguments; a failure is a
        // return value, and every failure leaves the process as it was.
        unsafe {
            if AttachConsole(ATTACH_PARENT_PROCESS) == 0 {
                return; // started from Explorer, the Start menu, a service
            }
            let name: Vec<u16> = "CONOUT$".encode_utf16().chain(Some(0)).collect();
            for which in [STD_OUTPUT_HANDLE, STD_ERROR_HANDLE] {
                let current = GetStdHandle(which);
                if current.is_null() || current == INVALID_HANDLE_VALUE {
                    let console = CreateFileW(
                        name.as_ptr(),
                        GENERIC_READ | GENERIC_WRITE,
                        FILE_SHARE_READ | FILE_SHARE_WRITE,
                        std::ptr::null_mut(),
                        OPEN_EXISTING,
                        0,
                        std::ptr::null_mut(),
                    );
                    if console != INVALID_HANDLE_VALUE {
                        SetStdHandle(which, console);
                    }
                }
            }
        }
    }
}

