//! The headless host: the whole `BrepApp` inside `egui_kittest::Harness` on a
//! wgpu device, stepped on demand on a dedicated thread (measured in §12). No
//! window, no display. Answers the `brep_mcp_core` host protocol; the app's
//! own envelope and reply types are converted at the boundary.
use crate::annotate::{Annotated, Annotation};
use brep_app::app::BrepApp;
use brep_app::automation::command::{Envelope as AppEnvelope, Reply as AppReply};
use brep_app::automation::AppOptions;
use brep_mcp_core::host::{HostConfig, HostHandle, HostInfo, HostRequest, Reply};
use egui_kittest::Harness;
use serde_json::Value;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub struct HeadlessConfig {
    pub width: f32,
    pub height: f32,
    pub ppp: f32,
    pub seed: bool,
    /// The session's private store directory.
    pub store_dir: PathBuf,
    /// Frames to step while waiting for one command's reply before giving up.
    pub max_steps: u32,
    /// HANG GUARD on the seed history landing before the first command
    /// ([`settle`]). Generous on purpose and with NO performance meaning:
    /// nothing about the app is asserted by how long the seed takes, only that
    /// it finishes at all. Sixty seconds to match the other liveness guards in
    /// the UI-side suites.
    pub settle_limit: Duration,
}

impl Default for HeadlessConfig {
    fn default() -> Self {
        Self {
            width: 1400.0,
            height: 960.0,
            ppp: 1.0,
            seed: true,
            store_dir: std::env::temp_dir().join("brep-mcp-store"),
            max_steps: 600,
            settle_limit: Duration::from_secs(60),
        }
    }
}

impl From<HostConfig> for HeadlessConfig {
    fn from(c: HostConfig) -> Self {
        Self { width: c.width, height: c.height, ppp: c.ppp, seed: c.seed, store_dir: c.store_dir, ..Self::default() }
    }
}

/// The app's reply in the server's shape: the same JSON, the blob beside it.
fn convert(mut r: AppReply) -> Reply {
    let blob = r.blob.take();
    let (id, frame) = (r.id, r.frame);
    match serde_json::to_value(&r).map_err(|e| e.to_string()).and_then(|v| Reply::from_app(v, blob)) {
        Ok(reply) => reply,
        Err(e) => Reply::err(id, frame, e),
    }
}

/// The last panic message, captured by the hook the host installs.
static LAST_PANIC: Mutex<Option<String>> = Mutex::new(None);

fn install_panic_hook() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            let msg = info
                .payload()
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| info.payload().downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_else(|| "panic".into());
            let loc = info.location().map(|l| format!(" at {}:{}", l.file(), l.line())).unwrap_or_default();
            *LAST_PANIC.lock().unwrap() = Some(format!("{msg}{loc}"));
            previous(info);
        }));
    });
}

fn take_panic() -> Option<String> {
    LAST_PANIC.lock().unwrap().take()
}

/// Spawn the host thread; returns once the app is built and its seed history
/// has settled, or with the build error.
pub fn spawn(cfg: impl Into<HeadlessConfig>) -> Result<HostHandle, String> {
    let cfg: HeadlessConfig = cfg.into();
    install_panic_hook();
    // Publishers are off in a plain native run; a host is what turns them on.
    brep_app::automation::registry::set_enabled(true);
    // ...and the registry is process-global while the app is not: the previous
    // script's session published into this same map and nothing has emptied it.
    // A reader would otherwise answer THIS session from the LAST one's frame
    // for any key this app has not published yet.
    brep_app::automation::registry::clear();
    let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<HostRequest>();
    let (ready_tx, ready_rx) = std::sync::mpsc::channel::<Result<HostInfo, String>>();
    let join = std::thread::Builder::new()
        .name("brep-mcp-headless".into())
        .spawn(move || {
            let built = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| build(&cfg)));
            let (mut harness, info) = match built {
                Ok(Ok(v)) => v,
                Ok(Err(e)) => {
                    let _ = ready_tx.send(Err(e));
                    return;
                }
                Err(_) => {
                    let _ = ready_tx.send(Err(format!("the app panicked while starting: {}", take_panic().unwrap_or_default())));
                    return;
                }
            };
            let queue = harness.state().app().automation().clone();
            let ppp = cfg.ppp;
            // Let the seed history land before the first command. A settle
            // that does not reach the idle contract is a FAILED START, not a
            // slow one. Sending `Ok(info)` regardless — which is what this did
            // until 2026-09-21 — handed the script an app with work still in
            // flight, and the script then failed further down, on whichever
            // expectation happened to touch the state that had not landed: a
            // hang reported as a wrong answer somewhere else.
            if let Err(why) = settle(&mut harness, &queue, ppp, cfg.settle_limit) {
                let _ = ready_tx.send(Err(format!("app was not settled before command 1 \u{2014} {why}")));
                return;
            }
            let _ = ready_tx.send(Ok(info));

            while let Some(req) = rx.blocking_recv() {
                match req {
                    HostRequest::Stop => break,
                    HostRequest::Command { envelope, reply } => {
                        let id = envelope.id;
                        let rx_reply = queue.submit(AppEnvelope { id, cmd: envelope.cmd, args: envelope.args });
                        let mut steps = 0u32;
                        let result = loop {
                            if let Ok(r) = rx_reply.try_recv() {
                                break convert(r);
                            }
                            if queue.is_poisoned() {
                                break Reply::err(id, queue.frame(), "session poisoned");
                            }
                            if steps >= cfg.max_steps {
                                break Reply::err(id, queue.frame(), format!("no reply after {} frames", cfg.max_steps));
                            }
                            if let Err(why) = step(&mut harness, &queue, ppp) {
                                queue.poison(why.clone());
                                break Reply::err(id, queue.frame(), why);
                            }
                            steps += 1;
                            if steps > 2 {
                                std::thread::sleep(Duration::from_millis(2));
                            }
                        };
                        let _ = reply.send(result);
                    }
                    // The DOCS ANNOTATION overlay (`crate::annotate`): set the
                    // caption and the ring, then draw one frame so the next
                    // capture — and anything that reads the session — sees it.
                    // The keys are checked HERE, against what the app is
                    // publishing right now, so a renamed control fails the
                    // walkthrough by name instead of dropping its highlight.
                    HostRequest::Annotate { spec, reply } => {
                        let result = (|| {
                            let note = Annotation::parse(&spec)?;
                            let missing = crate::annotate::unknown_keys(&note.highlight);
                            if !missing.is_empty() {
                                return Err(format!("annotate: {}", missing.join("; ")));
                            }
                            let highlighted = note.highlight.len();
                            harness.state_mut().note = note;
                            step(&mut harness, &queue, ppp)?;
                            Ok(serde_json::json!({
                                "frame": queue.frame(),
                                "highlighted": highlighted,
                            }))
                        })();
                        let _ = reply.send(result);
                    }
                    HostRequest::Screenshot { region, reply } => {
                        let result = (|| {
                            step(&mut harness, &queue, ppp)?;
                            let image = harness.render().map_err(|e| format!("headless render: {e}"))?;
                            let view = harness.state().app().view_rect();
                            let region: brep_app::automation::cmd_capture::Region =
                                serde_json::from_value(region).map_err(|e| format!("region: {e}"))?;
                            let color = egui::ColorImage {
                                size: [image.width() as usize, image.height() as usize],
                                source_size: egui::vec2(image.width() as f32, image.height() as f32),
                                pixels: image
                                    .pixels()
                                    .map(|p| egui::Color32::from_rgba_premultiplied(p.0[0], p.0[1], p.0[2], p.0[3]))
                                    .collect(),
                            };
                            brep_app::automation::cmd_capture::encode_capture(&color, ppp, view, &region)
                        })();
                        let _ = reply.send(result);
                    }
                }
            }
        })
        .map_err(|e| format!("spawn host thread: {e}"))?;
    match ready_rx.recv() {
        Ok(Ok(info)) => Ok(HostHandle::new(tx, info, join)),
        Ok(Err(e)) => Err(e),
        Err(_) => Err("the host thread ended before the app was ready".into()),
    }
}

/// The harness's state is [`Annotated`] — `BrepApp` plus the docs-annotation
/// overlay painted over it — so the app is reached through `state().app()`.
/// See `crate::annotate` for why the overlay lives inside an `update` rather
/// than in the image compositor.
type App = Harness<'static, Annotated>;

fn build(cfg: &HeadlessConfig) -> Result<(App, HostInfo), String> {
    std::fs::create_dir_all(&cfg.store_dir).map_err(|e| format!("store dir {}: {e}", cfg.store_dir.display()))?;
    let store_dir = cfg.store_dir.clone();
    let seed = cfg.seed;
    let adapter = Arc::new(Mutex::new(String::new()));
    let adapter_out = adapter.clone();
    let harness = Harness::builder()
        .with_size(egui::vec2(cfg.width, cfg.height))
        .with_pixels_per_point(cfg.ppp)
        .with_step_dt(1.0 / 60.0)
        .wgpu()
        .build_eframe(move |cc| {
            brep_app::fonts::install(&cc.egui_ctx);
            let store = brep_app::store::native_store_at(store_dir.clone());
            let app = BrepApp::new_with(cc, AppOptions { store: Some(store), seed })
                .expect("BrepApp::new_with under kittest");
            // The app's OWN diagnostics, not a second reading of the same
            // adapter: `HostInfo.adapter`, the `diagnostics` command and the
            // Info window must all name the same device (brep_app::diagnostics).
            *adapter_out.lock().unwrap() = app.diagnostics().adapter_line();
            Annotated::new(app)
        });
    let info = HostInfo {
        backend: "headless",
        adapter: adapter.lock().unwrap().clone(),
        platform: std::env::consts::OS,
        width: cfg.width,
        height: cfg.height,
        ppp: cfg.ppp,
    };
    Ok((harness, info))
}

/// One frame: drain input into the harness's RawInput, step, check for a panic.
fn step(harness: &mut App, queue: &Arc<brep_app::automation::queue::AutomationQueue>, ppp: f32) -> Result<(), String> {
    let frame = harness.ctx.cumulative_frame_nr();
    let view = harness.state().app().view_rect();
    let ctx = harness.ctx.clone();
    queue.drain_input(harness.input_mut(), frame, ppp, view, &ctx);
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| harness.step()));
    match r {
        Ok(()) => Ok(()),
        Err(_) => Err(format!("the app panicked: {}", take_panic().unwrap_or_else(|| "(no message)".into()))),
    }
}

/// Why a [`settle`] never reached the idle contract.
///
/// It exists because the two ways of not settling used to be spelled the same
/// as settling: the function returned `()`, the deadline simply ended the loop
/// and a failed frame simply returned. A caller could not tell a settled app
/// from an app that had run out of time, so it proceeded with the second one.
#[derive(Debug, Clone)]
pub enum SettleError {
    /// The hang guard expired. `pending` names the work still in flight, which
    /// is the whole diagnostic: "still pending: mesh_imports" says which part
    /// of the app is stuck, and an EMPTY list says the app had gone idle but
    /// not for the two consecutive frames the contract asks for.
    DeadlineExpired { limit: Duration, frames: u32, pending: Vec<&'static str> },
    /// A frame failed (the app panicked); the message is [`step`]'s.
    StepFailed(String),
}

impl std::fmt::Display for SettleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SettleError::DeadlineExpired { limit, frames, pending } => {
                let plural = if *frames == 1 { "" } else { "s" };
                write!(f, "the {} ms hang guard expired after {frames} frame{plural}; still pending: ", limit.as_millis())?;
                if pending.is_empty() {
                    write!(f, "nothing, but the app had not been idle for two consecutive frames")
                } else {
                    write!(f, "{}", pending.join(", "))
                }
            }
            SettleError::StepFailed(why) => write!(f, "a frame failed: {why}"),
        }
    }
}

/// What the app still has in flight, named.
///
/// The SAME six the app's own idle contract reads
/// (`brep_app::automation::cmd_frame::frame_info` — `pending` and the
/// `idle` computed from it, which is what every waiting tool polls). Settling
/// on fewer than the app calls idle is the same defect in a smaller costume:
/// the host would declare the app ready while `frame` still called it busy.
fn pending_work(harness: &App) -> Vec<&'static str> {
    let e = harness.state().app().docs_engine();
    [
        ("run", e.run_pending()),
        ("queries", e.queries_pending()),
        ("mesh_imports", e.mesh_imports_pending()),
        ("step_probes", e.step_probes_pending()),
        ("topology", e.topology_pending()),
        ("projection", e.sheet_lines_pending()),
    ]
    .into_iter()
    .filter_map(|(name, on)| on.then_some(name))
    .collect()
}

/// Step until the app is idle two frames running, or say why it is not.
///
/// `limit` is a HANG GUARD with no performance meaning: it is not a claim about
/// how long a seed run may take, only a refusal to step forever. Blowing it is
/// reported, never absorbed.
fn settle(
    harness: &mut App,
    queue: &Arc<brep_app::automation::queue::AutomationQueue>,
    ppp: f32,
    limit: Duration,
) -> Result<(), SettleError> {
    let start = Instant::now();
    let mut idle_frames = 0u32;
    let mut frames = 0u32;
    loop {
        // One frame ALWAYS runs before the deadline is consulted, so the
        // pending list in the error was read from a stepped app rather than
        // from whatever it happened to hold before the first frame.
        step(harness, queue, ppp).map_err(SettleError::StepFailed)?;
        frames += 1;
        let pending = pending_work(harness);
        if pending.is_empty() {
            idle_frames += 1;
            if idle_frames >= 2 {
                return Ok(());
            }
        } else {
            idle_frames = 0;
        }
        if start.elapsed() >= limit {
            return Err(SettleError::DeadlineExpired { limit, frames, pending });
        }
        if !pending.is_empty() {
            std::thread::sleep(Duration::from_millis(8));
        }
    }
}

#[allow(dead_code)]
fn _value(_: Value) {}
