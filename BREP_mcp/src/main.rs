//! `brep-mcp` — the BREP MCP automation server binary.
//!
//!   brep-mcp serve [--http PORT]      speak MCP over stdio (what `.mcp.json` runs), or streamable HTTP
//!   brep-mcp schema [--type T|--all] [--write DIR]
//!                                     the feature catalogue as JSON Schema / the generated docs
//!   brep-mcp test <scripts…>          run test-mcp scripts headlessly
//!   brep-mcp shot <doc> <png>         render one document to an image
//!   brep-mcp typescript status|provision
//!                                     report, or install, the pinned TypeScript compiler
//!   brep-mcp render-mesh <mesh.json> <out.png> [--normalize|--frame …]
//!                                     rasterise a triangle mesh on the CPU and
//!                                     measure its silhouette
use brep_mcp::server::BrepServer;
use clap::{Parser, Subcommand};
use serde_json::{json, Value};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "brep-mcp", version, about = "BREP MCP automation server")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Export and manage the reviewed Markdown skill bundle offline.
    Skills {
        #[command(subcommand)]
        command: SkillCommand,
    },
    /// Serve MCP over stdio, or over streamable HTTP with `--http`.
    Serve {
        /// Where sessions keep their store, shots and logs (default: the OS temp dir).
        #[arg(long)]
        session_root: Option<PathBuf>,
        /// Private persistent execution state for this server identity. Reuse
        /// after restart; interrupted work is reconciled, never replayed.
        #[arg(long, conflicts_with_all = ["instance", "ephemeral"])]
        state_dir: Option<PathBuf>,
        /// Stable named execution-state identity (default: headless). Use a
        /// different name for each concurrently running server.
        #[arg(long, conflicts_with = "ephemeral")]
        instance: Option<String>,
        /// Keep execution state only in memory for this process.
        #[arg(long)]
        ephemeral: bool,
        /// Opt-in private diagnostic metadata archive. Independent of execution
        /// state; bounded history and gaps, never automatic mutation replay.
        #[arg(long)]
        event_journal_dir: Option<PathBuf>,
        /// Serve the streamable-HTTP transport on this loopback port instead of
        /// stdio — the same transport `brep-app --mcp` exposes, with no window.
        /// 0 picks a free port; the URL is printed on stderr.
        #[arg(long, value_name = "PORT")]
        http: Option<u16>,
    },
    /// Print the kernel feature catalogue rendered as JSON Schema.
    Schema {
        /// One feature (type or shortName) instead of the whole catalogue.
        #[arg(long = "type")]
        feature_type: Option<String>,
        /// Also print the test-mcp script format schema.
        #[arg(long)]
        all: bool,
        /// Write the generated docs (commands / features, md + json) into this directory
        /// instead of printing.
        #[arg(long)]
        write: Option<PathBuf>,
    },
    /// Run test-mcp scripts through a headless session each.
    Test {
        scripts: Vec<PathBuf>,
        #[arg(long, default_value = "headless")]
        backend: String,
        #[arg(long)]
        out: Option<PathBuf>,
        #[arg(long)]
        update_baselines: bool,
        #[arg(long)]
        session_root: Option<PathBuf>,
        /// ORDER CHECK: play the whole list a second time in REVERSE, and fail
        /// if any script's session STARTED differently in the two passes.
        /// One script must not change what the next one measures; scripts run
        /// through one process, so only what a session owns is actually fresh.
        /// Artefacts go under `<out>/forward` and `<out>/reverse`. Costs two
        /// runs of the list — point it at a subset, not the whole glob.
        #[arg(long)]
        order_check: bool,
        /// Where a script's `video.out` is resolved — the repo root, for the
        /// documentation walkthroughs (`./build.sh docs-videos`). WITHOUT it a
        /// script that declares a video is still recorded and encoded, but the
        /// GIF only reaches the run's output directory: the `test-mcp` gate
        /// plays every script and must not rewrite documentation assets as a
        /// side effect of being run.
        #[arg(long, value_name = "DIR")]
        video_root: Option<PathBuf>,
    },
    /// Rasterise a triangle mesh `{positions, indices}` to a PNG on the CPU and
    /// report its silhouette — the native STEP-validation renderer. No host,
    /// no GPU, no window; `--help` prints the full option list. The same
    /// command is `BREP_mcp_core`'s `render_mesh` example, which step-validation
    /// calls directly so it need not build this binary's app host.
    // `disable_help_flag` lets `--help` reach the renderer's own parser,
    // which prints the framing modes and options.
    #[command(name = "render-mesh", disable_help_flag = true)]
    RenderMesh {
        /// Everything after `render-mesh`, passed through unparsed.
        #[arg(trailing_var_arg = true, allow_hyphen_values = true)]
        args: Vec<String>,
    },
    /// The TypeScript toolchain `run_script` checks TypeScript with.
    Typescript {
        #[command(subcommand)]
        command: TypescriptCommand,
    },
    /// Render a document to a PNG through the headless host.
    Shot {
        doc: PathBuf,
        png: PathBuf,
        #[arg(long, default_value = "ISO")]
        view: String,
        #[arg(long, default_value = "1280x800")]
        size: String,
        #[arg(long, default_value_t = 1.0)]
        ppp: f32,
        #[arg(long)]
        session_root: Option<PathBuf>,
    },
}

#[derive(Subcommand)]
enum TypescriptCommand {
    /// Report the compiler `run_script` would check TypeScript with: the same
    /// discovery and launcher probe `tools/list` advertises. Exits 1 when
    /// TypeScript is unavailable.
    Status,
    /// Install the pinned TypeScript into the user-local prefix discovery
    /// checks, with npm (`BREP_NPM` or PATH). Downloads; never run by a tool call.
    Provision,
}

#[derive(Subcommand)]
enum SkillCommand {
    /// Export this binary's reviewed bundle; refuses to replace an existing file.
    #[command(visible_alias = "fetch")]
    Bundle { #[arg(long)] output: PathBuf },
    /// Install or update the managed selection from this binary's exact bundle.
    #[command(visible_alias = "update")]
    Install {
        #[arg(long)] bundle: PathBuf,
        /// SHA256 obtained from the reviewed bundle, not an untrusted download.
        #[arg(long)] sha256: String,
        #[arg(long)] directory: PathBuf,
    },
    /// Verify the installed selection and report its version directory.
    Inspect { #[arg(long)] directory: PathBuf },
    /// Restore the previous installed version after verifying its reviewed pin.
    Rollback { #[arg(long)] directory: PathBuf, #[arg(long)] sha256: String },
}

fn skill_command(command: SkillCommand) -> Result<Value, String> {
    use brep_mcp_core::skill_distribution as distribution;
    let receipt = match command {
        SkillCommand::Bundle { output } => {
            let output_text=output.to_str().ok_or("bundle output path must be UTF-8")?;
            let digest=distribution::export(&output)?;
            return Ok(json!({"sha256":digest,"bundle":output_text}));
        }
        SkillCommand::Install { bundle, sha256, directory } => {
            distribution::install(&distribution::read_bundle(&bundle)?,&sha256,&directory)?
        }
        SkillCommand::Inspect { directory } => distribution::inspect(&directory)?,
        SkillCommand::Rollback { directory, sha256 } => distribution::rollback(&directory,&sha256)?,
    };
    serde_json::to_value(receipt).map_err(|e| e.to_string())
}

fn session_root(explicit: Option<PathBuf>) -> PathBuf {
    explicit
        .or_else(|| std::env::var_os("BREP_MCP_SESSION_ROOT").map(PathBuf::from))
        .unwrap_or_else(|| std::env::temp_dir().join("brep-mcp"))
}

/// Call a tool by name on the server's current tool set.
async fn call(server: &BrepServer, name: &str, args: Value) -> Result<brep_mcp::tools::ToolOutput, String> {
    let spec = server.state.tools.read().await.get(name).cloned().ok_or_else(|| format!("no tool `{name}`"))?;
    (spec.handler)(args).await
}

/// What the session looks like BEFORE a script's first step: the camera and the
/// published viewport rect + probe points, with the volatile frame counter
/// dropped. Two runs of the same script must start from the same one — the
/// script has not run yet, so anything that differs came from the script
/// BEFORE it (`--order-check`).
async fn boot_state(server: &BrepServer) -> Value {
    let mut out = json!({});
    for (key, tool, args) in [
        ("camera", "camera_get", json!({})),
        ("state", "state_get", json!({ "names": ["__brepView", "__brepProbe"] })),
    ] {
        out[key] = match call(server, tool, args).await {
            Ok(o) => {
                let mut v = o.json;
                if let Some(map) = v.as_object_mut() {
                    map.remove("frame");
                }
                v
            }
            Err(e) => json!({ "error": e }),
        };
    }
    out
}

/// Start a headless session for a script and regenerate the tool set.
async fn start_session(server: &BrepServer, args: Value) -> Result<(), String> {
    call(server, "session_start", args).await?;
    server.rebuild_tools().await;
    Ok(())
}

async fn stop_session(server: &BrepServer) {
    let _ = call(server, "session_stop", json!({})).await;
    server.rebuild_tools().await;
}

fn main() {
    // Started as a code-mode worker (the host spawning itself): run it, done.
    if brep_mcp_core::cad::code::worker_entry() {
        return;
    }
    let cli = Cli::parse();
    let rt = tokio::runtime::Runtime::new().expect("tokio runtime");
    let code = match cli.command {
        Command::Typescript { command: TypescriptCommand::Status } => match brep_mcp_core::cad::code::typescript_status() {
            Ok((node, tsc)) => { println!("TypeScript {} available: node {} runs {}", brep_mcp_core::cad::code::TSC_VERSION, node.display(), tsc.display()); 0 }
            Err(error) => { eprintln!("brep-mcp: typescript: {}", error.message); 1 }
        },
        Command::Typescript { command: TypescriptCommand::Provision } => match brep_mcp_core::cad::code::provision_typescript() {
            Ok(report) => { println!("{report}"); 0 }
            Err(error) => { eprintln!("brep-mcp: typescript provision: {error}"); 1 }
        },
        Command::Skills { command } => match skill_command(command) {
            Ok(value) => { println!("{value}"); 0 }
            Err(error) => { eprintln!("brep-mcp: skills: {error}"); 1 }
        },
        Command::Serve { session_root: root, state_dir, instance, ephemeral, event_journal_dir, http } => {
            use brep_mcp_core::launch_state::{Host, Policy};
            let persistence = match brep_mcp_core::launch_state::resolve(Host::Headless, &Policy { instance, state_dir, ephemeral }) {
                Ok(configuration) => configuration,
                Err(error) => { eprintln!("brep-mcp: execution-state configuration: {error}"); std::process::exit(1); }
            };
            if let Err(error) = persistence.prepare_named_parent() {
                eprintln!("brep-mcp: persistent state: {error}"); std::process::exit(1);
            }
            let server = BrepServer::new(session_root(root), brep_mcp::headless_backend()).with_commands(&brep_mcp::command_descriptions());
            let server = if let Some(directory) = &persistence.state_dir {
                match server.with_state_dir(directory) {
                    Ok(server) => server,
                    Err(error) => { eprintln!("brep-mcp: persistent state at {}: {error}. Use a distinct --instance or --state-dir for another server, or --ephemeral for temporary work.", directory.display()); std::process::exit(1); }
                }
            } else { server };
            match &persistence.state_dir {
                Some(directory) => eprintln!("brep-mcp: execution state {} ({:?})", directory.display(), persistence.source),
                None => eprintln!("brep-mcp: execution state is ephemeral ({:?})", persistence.source),
            }
            let server = if let Some(directory) = event_journal_dir {
                match server.with_event_journal(&directory) {
                    Ok(server) => server,
                    Err(error) => { eprintln!("brep-mcp: diagnostic archive: {error}"); std::process::exit(1); }
                }
            } else { server };
            let result = match http {
                // Streamable HTTP: bind first, so a busy port is a launch error
                // rather than a server nobody can reach, then print the URL the
                // way `brep-app --mcp` does.
                Some(port) => rt.block_on(async {
                    let listener = tokio::net::TcpListener::bind(("127.0.0.1", port))
                        .await
                        .map_err(|e| format!("cannot listen on 127.0.0.1:{port}: {e}"))?;
                    let bound = listener.local_addr().map_err(|e| e.to_string())?.port();
                    let credential = brep_mcp::http::Credential::load_or_create()?;
                    eprintln!("brep-mcp listening on {}", brep_mcp::http::url(bound));
                    eprintln!("every request needs `Authorization: Bearer <token>`, the token in {}", brep_mcp::http::Credential::path().display());
                    brep_mcp::http::serve_http(server, listener, credential).await.map_err(|e| e.to_string())
                }),
                None => rt.block_on(brep_mcp::server::serve_stdio(server)).map_err(|e| e.to_string()),
            };
            match result {
                Ok(()) => 0,
                Err(e) => {
                    eprintln!("brep-mcp: {e}");
                    1
                }
            }
        }
        Command::Schema { feature_type, all, write } => {
            if let Some(dir) = write {
                // The generated docs are rendered from a LIVE headless session:
                // the app's command registry, its state registry and every
                // panel's hit-key docs, exactly as a client would see them.
                let server = BrepServer::new(session_root(None), brep_mcp::headless_backend());
                let result: Result<Vec<String>, String> = rt.block_on(async {
                    let (tools, reg) = brep_mcp::generate::live_registries(&server).await?;
                    stop_session(&server).await;
                    brep_mcp::generate::write_with(&dir, &tools, &reg).map_err(|e| e.to_string())
                });
                match result {
                    Ok(files) => {
                        for f in files {
                            println!("wrote {}", dir.join(f).display());
                        }
                        std::process::exit(0);
                    }
                    Err(e) => {
                        eprintln!("brep-mcp: {e}");
                        std::process::exit(brep_mcp::script::exit::HOST_ERROR);
                    }
                }
            }
            let out = match feature_type {
                Some(ty) => match brep_mcp::schema::entry(&ty) {
                    Some(entry) => json!({
                        "schema": brep_mcp::schema::to_json_schema(&entry),
                        "defaults": brep_mcp::schema::defaults(&ty),
                    }),
                    None => {
                        eprintln!("brep-mcp: unknown feature type `{ty}`");
                        std::process::exit(2);
                    }
                },
                None if all => json!({
                    "features": brep_mcp::schema::all(),
                    "scriptFormat": brep_mcp::script::Script::json_schema(),
                }),
                None => brep_mcp::schema::all(),
            };
            println!("{}", serde_json::to_string_pretty(&out).unwrap());
            0
        }
        Command::Test { scripts, backend, out, update_baselines, session_root: root, order_check, video_root } => {
            if backend != "headless" {
                eprintln!("brep-mcp: backend `{backend}` is not available yet; using headless");
            }
            let server = BrepServer::new(session_root(root), brep_mcp::headless_backend()).with_commands(&brep_mcp::command_descriptions());
            let out_root = out.unwrap_or_else(|| PathBuf::from("target/test-mcp"));
            let baselines = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/baselines");
            let mut worst = brep_mcp::script::exit::PASS;
            // The ORDER CHECK plays the list forward and then backward through
            // the same process, recording what each script's session looked
            // like before its first step. Without it the list is played once.
            let mut passes: Vec<(&str, Vec<PathBuf>)> = vec![("forward", scripts.clone())];
            if order_check {
                passes.push(("reverse", scripts.iter().rev().cloned().collect()));
            }
            let mut boot_samples: Vec<brep_mcp::runner::BootSample> = Vec::new();
            for (pass, list) in &passes {
                let pass_root = if order_check { out_root.join(pass) } else { out_root.clone() };
                if order_check {
                    eprintln!("--- order check: {pass} pass ({} scripts) ---", list.len());
                }
                for path in list {
                    // A `.js` script is a PROGRAM, played through code mode
                    // (`brep_mcp::program`); a `.json` one is a step list.
                    let is_program = path.extension().is_some_and(|e| e == "js");
                    let loaded = std::fs::read_to_string(path).map_err(|e| e.to_string()).and_then(|t| {
                        if is_program {
                            brep_mcp::program::Program::parse(&t).map(|p| (p.script.clone(), Some(p)))
                        } else {
                            brep_mcp::script::Script::parse(&t).map(|s| (s, None))
                        }
                    });
                    let (script, program) = match loaded {
                        Ok(s) => s,
                        Err(e) => {
                            eprintln!("brep-mcp: {}: {e}", path.display());
                            worst = worst.max(brep_mcp::script::exit::TOOL_ERROR);
                            continue;
                        }
                    };
                    let out_dir = pass_root.join(&script.name);
                    // A script that only uses catalogue tools needs no app.
                    // Nor does one that only talks to its PLM fixture.
                    let needs_app = match &program {
                        Some(p) => p.needs_app(),
                        None => script.steps.iter().any(|s| {
                            !matches!(s.tool.as_str(), "feature_catalogue" | "feature_schema" | brep_mcp::runner::PLM_REQUEST | brep_mcp::runner::PLM_SERVER)
                        }),
                    };
                    // The session's STORE is a real directory beside the run's
                    // artefacts, so the file modal lists what this script saved
                    // and a `session_restart` comes back to it. It is also the
                    // app's config root, where the PLM fixture writes D5's files.
                    let store = if needs_app || script.plm.is_some() {
                        match brep_mcp::runner::prepare_store(&out_dir, &script.fixtures) {
                            Ok(s) => Some(s),
                            Err(e) => {
                                eprintln!("FAIL {} (store: {e})", script.name);
                                worst = worst.max(brep_mcp::script::exit::HOST_ERROR);
                                continue;
                            }
                        }
                    } else {
                        None
                    };
                    // THE PLM FIXTURE starts, and is seeded, before the app, so
                    // an app that reads D5's files at boot finds a live server.
                    // It lives until the end of this iteration (dropped = killed).
                    let plm = match (&script.plm, &store) {
                        (Some(spec), Some(store)) => match brep_mcp::plm_fixture::PlmFixture::start(spec, &out_dir, store) {
                            Ok(f) => {
                                eprintln!("  plm {} — {}", f.url, f.binary);
                                Some(std::sync::Arc::new(f))
                            }
                            Err(e) => {
                                eprintln!("FAIL {} (plm: {e})", script.name);
                                worst = worst.max(brep_mcp::script::exit::HOST_ERROR);
                                continue;
                            }
                        },
                        _ => None,
                    };
                    if let Some(store) = store.as_ref().filter(|_| needs_app) {
                        let args = json!({
                            "backend": "headless",
                            "width": script.size[0], "height": script.size[1], "ppp": script.ppp,
                            "seed": script.seed, "document": script.document, "record": false,
                            "store_root": store.display().to_string(),
                        });
                        if let Err(e) = rt.block_on(start_session(&server, args)) {
                            eprintln!("FAIL {} (host: {e})", script.name);
                            worst = worst.max(brep_mcp::script::exit::HOST_ERROR);
                            continue;
                        }
                        if order_check {
                            boot_samples.push(brep_mcp::runner::BootSample {
                                script: script.name.clone(),
                                pass: pass.to_string(),
                                state: rt.block_on(boot_state(&server)),
                            });
                        }
                    }
                    let opts = brep_mcp::runner::RunOptions {
                        out_dir,
                        baselines_dir: baselines.clone(),
                        update_baselines,
                        stop_on_fail: true,
                        video_root: video_root.clone(),
                        plm: plm.clone(),
                    };
                    let result = rt.block_on(async {
                        match &program {
                            Some(program) => brep_mcp::program::run(program, &server.state, &opts).await,
                            None => {
                                let tools = server.state.tools.read().await;
                                brep_mcp::runner::run(&script, &tools, &opts).await
                            }
                        }
                    });
                    if needs_app {
                        rt.block_on(stop_session(&server));
                    }
                    drop(plm);
                    let status = if result.passed { "PASS" } else { "FAIL" };
                    eprintln!("{status} {} ({} steps, exit {})", script.name, result.steps.len(), result.exit_code);
                    if let Some(v) = &result.video {
                        eprintln!(
                            "  video {} — {} frames, {}x{}, {:.1} KiB, plays {:.1} s, encoded in {} ms -> {}",
                            v.out,
                            v.frames,
                            v.size[0],
                            v.size[1],
                            v.bytes as f64 / 1024.0,
                            v.duration_ms as f64 / 1000.0,
                            v.encode_ms,
                            v.written.join(", ")
                        );
                    }
                    for step in result.steps.iter().filter(|s| !s.failures.is_empty()) {
                        for f in &step.failures {
                            eprintln!("  step {} `{}`: {f}", step.index, step.tool);
                        }
                    }
                    worst = worst.max(result.exit_code);
                }
            }
            if order_check {
                let failures = brep_mcp::runner::order_check_failures(&boot_samples);
                let scripts_checked = boot_samples.len() / passes.len().max(1);
                if failures.is_empty() {
                    eprintln!("PASS order check ({scripts_checked} scripts started identically in both passes)");
                } else {
                    for f in &failures {
                        eprintln!("FAIL order check: {f}");
                    }
                    worst = worst.max(brep_mcp::script::exit::EXPECTATION_FAILED);
                }
            }
            worst
        }
        Command::RenderMesh { args } => match brep_mcp::render_mesh::cli(&args) {
            Ok(out) => {
                if !out.is_empty() {
                    println!("{out}");
                }
                0
            }
            Err(e) => {
                eprintln!("brep-mcp render-mesh: {e}");
                2
            }
        },
        Command::Shot { doc, png, view, size, ppp, session_root: root } => {
            let (w, h) = size
                .split_once('x')
                .and_then(|(a, b)| Some((a.parse::<f32>().ok()?, b.parse::<f32>().ok()?)))
                .unwrap_or((1280.0, 800.0));
            let server = BrepServer::new(session_root(root), brep_mcp::headless_backend()).with_commands(&brep_mcp::command_descriptions());
            let result: Result<(), String> = rt.block_on(async {
                start_session(&server, json!({ "backend": "headless", "width": w, "height": h, "ppp": ppp, "document": doc.display().to_string(), "record": false })).await?;
                call(&server, "standard_view", json!({ "name": view })).await?;
                call(&server, "zoom_to_fit", json!({})).await?;
                call(&server, "wait_idle", json!({})).await?;
                let out = call(&server, "screenshot", json!({ "region": "viewport", "cursor": false, "max_width": 100000, "save_as": png.display().to_string() })).await?;
                eprintln!("wrote {} ({}x{})", png.display(), out.json["width"], out.json["height"]);
                stop_session(&server).await;
                Ok(())
            });
            match result {
                Ok(()) => 0,
                Err(e) => {
                    eprintln!("brep-mcp: {e}");
                    brep_mcp::script::exit::HOST_ERROR
                }
            }
        }
    };
    std::process::exit(code);
}
