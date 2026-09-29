//! The bake worker (plm-cad-integration-todo §3 S10).
//!
//! The PLM server never links the kernel. When it assembles a family member
//! or a template copy itself, it stores the document and queues a BAKE: a
//! headless CAD worker takes the job, opens the document in the engine,
//! evaluates its expressions, builds its features, and reports. This module is
//! that worker.
//!
//! - [`bake_document`] runs one document through a bare [`EngineState`] and
//!   its synchronous runner. That is the history run the app does on open,
//!   without egui, without a window and without a GPU.
//! - [`verdict`] reads the run's report. The engine LOADS a document whose
//!   expressions do not evaluate or whose features do not build, and reports
//!   each failure in `featureErrors`. So a load that succeeds is not a bake
//!   that succeeded: any feature error fails the job, with the sentences the
//!   engine wrote.
//!
//! - [`run_pass`] is the loop, one pass of it: claim the oldest free job
//!   (`POST /api/bake/next`), read the document the server assembled, bake it,
//!   and either keep it (`PUT /api/bake/jobs/:id/result`) or say why not
//!   (`POST /api/bake/jobs/:id/fail`). It signs in with a `worker`-scoped
//!   token, which reaches reads, `/api/bake/*` and `/api/store/doc/*` and
//!   nothing else.
//!
//! What goes back is the document the server sent, unchanged: the bake checks
//! it, it does not rewrite it. (The plan allows a rewrite. The engine's own
//! serialization adds the app's `workbench` field, and nothing needs it yet.)
//!
//! **The lease.** A claim lapses after 15 minutes (`BAKE_LEASE` on the
//! server), and the engine's run cannot be pre-empted. So while a job is held,
//! a [`Renewal`] thread beside the bake says "still baking"
//! (`POST /api/bake/jobs/:id/renew`) every third of a lease, and a bake of any
//! length keeps its claim. The thread stops the moment the job reports (its
//! guard is dropped), and a refused renewal (someone took the job over, it
//! was retried) is logged once and not repeated. A bake that still loses its
//! claim is refused when it reports, and the pass records the job as dropped.

use super::client::{PlmClient, PlmError};
use brep_render::engine_state::EngineState;
use serde::Deserialize;
use serde_json::{json, Value};

/// What one bake found.
#[derive(Debug, Clone, PartialEq)]
pub struct Bake {
    /// The engine's run report (`featureErrors`, `featureTimings`, …), as the
    /// app's `history_listing` shows it.
    pub report: Value,
    /// Solids the run produced.
    pub solids: usize,
    /// The member's thumbnail (P9), rendered from the baked scene; `None`
    /// when it drew nothing.
    pub thumbnail: Option<Vec<u8>>,
}

/// Open `document` in a fresh engine and run its history to the end.
/// `Err` is a document the engine refuses outright (not JSON, not a history).
pub fn bake_document(document: &str) -> Result<Bake, String> {
    let mut engine = EngineState::new();
    engine.set_history_json(document)?;
    // The default runner is synchronous, so the run has landed by now. Pump
    // anyway, so a runner that is not cannot hand back an empty report.
    while engine.run_pending() {
        engine.pump();
    }
    let report = serde_json::from_str(&engine.history_report_json()).unwrap_or(Value::Null);
    let solids = engine.scene.solids().len();
    let thumbnail = brep_render::thumbnail::render_png(&brep_render::thumbnail::capture(&engine.scene), brep_render::thumbnail::SIZE);
    Ok(Bake { report, solids, thumbnail })
}

/// Whether `bake` is a document to keep, or why not, in sentences a person can
/// act on. `number` and `revision` name the member in the sentence.
pub fn verdict(number: &str, revision: &str, bake: &Bake) -> Result<(), String> {
    let errors: Vec<&str> = bake.report["featureErrors"]
        .as_array()
        .map(|list| list.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    if errors.is_empty() {
        return Ok(());
    }
    Err(format!("{number} revision {revision} did not build: {}", errors.join("; ")))
}

/// A job as `POST /api/bake/next` hands it out (the fields this worker reads).
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct BakeJob {
    /// The revision's id: the job is the revision.
    pub id: String,
    pub number: String,
    pub revision_label: String,
    pub document_key: String,
    /// `family` or `template`.
    pub source: String,
    pub attempts: u32,
    pub lease_expires_at: Option<u64>,
}

/// What a pass did with one job.
#[derive(Debug, Clone, PartialEq)]
pub enum JobOutcome {
    /// Built: the document was kept and the revision left the queue.
    Done { job: BakeJob },
    /// Did not build, or could not be baked at all: reported failed with
    /// `error`, which the server shows beside the revision.
    Failed { job: BakeJob, error: String },
    /// The server refused the report (the claim lapsed, the revision was
    /// released meanwhile). Nothing was written; the job is the server's again.
    Dropped { job: BakeJob, why: String },
}

impl JobOutcome {
    pub fn job(&self) -> &BakeJob {
        match self {
            JobOutcome::Done { job } | JobOutcome::Failed { job, .. } | JobOutcome::Dropped { job, .. } => job,
        }
    }

    /// `done`, `failed` or `dropped`.
    pub fn status(&self) -> &'static str {
        match self {
            JobOutcome::Done { .. } => "done",
            JobOutcome::Failed { .. } => "failed",
            JobOutcome::Dropped { .. } => "dropped",
        }
    }

    pub fn to_json(&self) -> Value {
        let job = self.job();
        let mut out = json!({
            "id": job.id, "number": job.number, "revision": job.revision_label,
            "source": job.source, "status": self.status(),
        });
        match self {
            JobOutcome::Failed { error, .. } => out["error"] = json!(error),
            JobOutcome::Dropped { why, .. } => out["why"] = json!(why),
            JobOutcome::Done { .. } => {}
        }
        out
    }
}

/// Claim and bake up to `max_jobs` jobs, one at a time, stopping early when
/// the queue is empty. `client` must be signed in (a `worker`-scoped token is
/// enough). `Err` only when the server stopped answering or refused the worker
/// itself (`401`, `403`); a job that cannot be baked is a `Failed` outcome, not
/// an error.
pub async fn run_pass(client: &PlmClient, max_jobs: usize) -> Result<Vec<JobOutcome>, PlmError> {
    run_pass_with(client, max_jobs, None).await
}

/// [`run_pass`], renewing each held job's lease with `renewal` while it bakes.
pub async fn run_pass_with(client: &PlmClient, max_jobs: usize, renewal: Option<&Renewal>) -> Result<Vec<JobOutcome>, PlmError> {
    let mut outcomes = Vec::new();
    while outcomes.len() < max_jobs {
        let claimed = client.call("POST", "/api/bake/next", Some(b"{}".to_vec())).await?;
        if claimed.status == 204 {
            break;
        }
        let job: BakeJob = serde_json::from_slice(&claimed.body)
            .map_err(|e| PlmError::Malformed(format!("/api/bake/next: {e}")))?;
        // Held from the claim to the report; dropped (stopped) with it.
        let _renewing = renewal.map(|r| r.start(&job.id));
        outcomes.push(bake_job(client, job).await?);
    }
    Ok(outcomes)
}

/// One claimed job, start to report.
async fn bake_job(client: &PlmClient, job: BakeJob) -> Result<JobOutcome, PlmError> {
    let refusal = match client.get_document(&job.document_key).await {
        Ok(Some(bytes)) => match std::str::from_utf8(&bytes).map_err(|e| e.to_string()).and_then(bake_document) {
            Ok(bake) => match verdict(&job.number, &job.revision_label, &bake) {
                Ok(()) => return keep(client, job, bytes, bake.thumbnail).await,
                Err(sentence) => sentence,
            },
            Err(why) => format!(
                "{} revision {} is not a document the CAD app can open: {why}",
                job.number, job.revision_label
            ),
        },
        Ok(None) => format!("{} revision {} was queued with no document to bake", job.number, job.revision_label),
        Err(error @ (PlmError::Unreachable(_) | PlmError::SignIn(_) | PlmError::Forbidden(_))) => return Err(error),
        Err(error) => format!("{} revision {}: its document could not be read: {error}", job.number, job.revision_label),
    };
    report_failed(client, job, refusal).await
}

/// `PUT …/result` with the document as the server sent it, then its
/// thumbnail. A refused thumbnail does not fail the job: the member built.
async fn keep(client: &PlmClient, job: BakeJob, bytes: Vec<u8>, thumbnail: Option<Vec<u8>>) -> Result<JobOutcome, PlmError> {
    let path = format!("/api/bake/jobs/{}/result", job.id);
    let hash = super::thumbnail::content_hash(&String::from_utf8_lossy(&bytes));
    match client.call("PUT", &path, Some(bytes)).await {
        Ok(_) => {
            if let Some(png) = thumbnail {
                if let Err(why) = super::thumbnail::put(client, &job.document_key, &hash, png).await {
                    log::warn!("{} revision {}: baked, but its thumbnail was not kept: {why}", job.number, job.revision_label);
                }
            }
            Ok(JobOutcome::Done { job })
        }
        // Checked out by someone after the claim: the claim is still ours, so
        // say so on the job, where the person who checked it out will see it.
        Err(PlmError::Conflict(sentence)) if sentence.contains("checked out") => {
            report_failed(client, job, sentence).await
        }
        Err(error @ (PlmError::Unreachable(_) | PlmError::SignIn(_) | PlmError::Forbidden(_))) => Err(error),
        Err(error) => Ok(JobOutcome::Dropped { job, why: error.to_string() }),
    }
}

/// `POST …/fail` with a sentence a person can act on.
async fn report_failed(client: &PlmClient, job: BakeJob, error: String) -> Result<JobOutcome, PlmError> {
    let path = format!("/api/bake/jobs/{}/fail", job.id);
    let body = serde_json::to_vec(&json!({ "error": error })).unwrap_or_default();
    match client.call("POST", &path, Some(body)).await {
        Ok(_) => Ok(JobOutcome::Failed { job, error }),
        Err(refused @ (PlmError::Unreachable(_) | PlmError::SignIn(_) | PlmError::Forbidden(_))) => Err(refused),
        Err(refused) => Ok(JobOutcome::Dropped { job, why: format!("{error} (and the server refused the report: {refused})") }),
    }
}

/// One pass against the server at `url`, signed in with `token`, driven to
/// the end on this thread. What the automation command and
/// `brep-app --bake-worker` both run. `Err` is a sentence: the server did not
/// answer, refused the token, or does not serve this app.
#[cfg(not(target_arch = "wasm32"))]
pub fn pass_blocking(url: &str, token: &str, max_jobs: usize) -> Result<Vec<JobOutcome>, String> {
    let client = PlmClient::new(std::rc::Rc::new(super::transport::EhttpTransport::new(url)));
    let renewal = Renewal::new(url, token, RENEW_EVERY);
    block_on(async {
        client.sign_in_with_token(token).await.map_err(|e| e.to_string())?;
        run_pass_with(&client, max_jobs, Some(&renewal)).await.map_err(|e| e.to_string())
    })
}

/// How often a held job's lease is renewed: a third of the server's 15-minute
/// `BAKE_LEASE`, so two renewals can be lost before the claim lapses.
pub const RENEW_EVERY: std::time::Duration = std::time::Duration::from_secs(5 * 60);

/// There is no bake worker in a browser: nothing to renew with there.
#[cfg(target_arch = "wasm32")]
pub enum Renewal {}

#[cfg(target_arch = "wasm32")]
impl Renewal {
    fn start(&self, _: &str) {
        match *self {}
    }
}

/// Keeps a held job's lease alive while its bake runs. The bake is synchronous
/// and cannot yield, so the renewals come from a thread of their own, with the
/// worker's token (the async client is this thread's and cannot cross).
#[cfg(not(target_arch = "wasm32"))]
pub struct Renewal {
    url: String,
    token: String,
    every: std::time::Duration,
}

/// What one job's renewals did.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RenewLog {
    /// Renewals the server accepted.
    pub renewed: u32,
    /// The refusal that stopped them, in the server's words.
    pub refused: Option<String>,
}

#[cfg(not(target_arch = "wasm32"))]
impl Renewal {
    pub fn new(url: &str, token: &str, every: std::time::Duration) -> Self {
        Self { url: url.trim_end_matches('/').to_string(), token: token.trim().to_string(), every }
    }

    /// Renew `job` every interval until the returned guard is dropped (or
    /// [`Renewing::finish`]ed). The first renewal is one interval in: the
    /// claim was just taken.
    pub fn start(&self, job: &str) -> Renewing {
        let (stop, stopped) = std::sync::mpsc::channel::<()>();
        let (url, token, every) = (format!("{}/api/bake/jobs/{job}/renew", self.url), self.token.clone(), self.every);
        let job = job.to_string();
        let handle = std::thread::spawn(move || {
            let mut log = RenewLog::default();
            let mut unreachable_said = false;
            loop {
                // The guard's drop disconnects the channel, which ends the
                // wait AT ONCE rather than at the next tick.
                match stopped.recv_timeout(every) {
                    Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
                    _ => return log,
                }
                let request = ehttp::Request {
                    headers: ehttp::Headers::new(&[
                        ("Authorization", &format!("Bearer {token}")),
                        ("Content-Type", "application/json"),
                    ]),
                    ..ehttp::Request::post(url.clone(), b"{}".to_vec())
                };
                match ehttp::fetch_blocking(&request) {
                    Ok(response) if response.ok => log.renewed += 1,
                    Ok(response) => {
                        // Refused: the claim is not ours to keep any more.
                        // Say so once and stop; asking again changes nothing.
                        let answer = super::PlmResponse { status: response.status, headers: Vec::new(), body: response.bytes };
                        let sentence = PlmError::from_response(&answer).to_string();
                        eprintln!("bake worker: job {job}: the lease renewal was refused: {sentence}");
                        log.refused = Some(sentence);
                        return log;
                    }
                    Err(why) => {
                        // No answer: the next renewal may get one. Said once.
                        if !unreachable_said {
                            eprintln!("bake worker: job {job}: the lease renewal got no answer ({why}); still trying");
                            unreachable_said = true;
                        }
                    }
                }
            }
        });
        Renewing { stop: Some(stop), handle: Some(handle) }
    }
}

/// A job's renewals in progress; dropping it stops them promptly.
#[cfg(not(target_arch = "wasm32"))]
pub struct Renewing {
    stop: Option<std::sync::mpsc::Sender<()>>,
    handle: Option<std::thread::JoinHandle<RenewLog>>,
}

#[cfg(not(target_arch = "wasm32"))]
impl Renewing {
    /// Stop, and say what the renewals did.
    pub fn finish(mut self) -> RenewLog {
        self.stop.take();
        self.handle.take().and_then(|h| h.join().ok()).unwrap_or_default()
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl Drop for Renewing {
    fn drop(&mut self) {
        self.stop.take();
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

/// Drive `future` on this thread. ehttp answers on its own thread and wakes
/// the waker it was polled with, which unparks this one.
#[cfg(not(target_arch = "wasm32"))]
fn block_on<F: std::future::Future>(future: F) -> F::Output {
    use std::sync::Arc;
    use std::task::{Context, Poll, Wake, Waker};

    struct Unpark(std::thread::Thread);
    impl Wake for Unpark {
        fn wake(self: Arc<Self>) {
            self.0.unpark();
        }
    }

    let waker = Waker::from(Arc::new(Unpark(std::thread::current())));
    let mut context = Context::from_waker(&waker);
    let mut future = std::pin::pin!(future);
    loop {
        match future.as_mut().poll(&mut context) {
            Poll::Ready(output) => return output,
            Poll::Pending => std::thread::park(),
        }
    }
}

/// `brep-app --bake-worker`'s own flags (everything after `--bake-worker`).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, PartialEq)]
pub struct WorkerArgs {
    pub url: String,
    pub token: String,
    /// One pass, then exit.
    pub once: bool,
    /// Seconds to wait after a pass that found the queue empty.
    pub poll: u64,
}

/// The worker's usage lines, for `brep-app --help` and a bad flag.
pub const WORKER_USAGE: &str = "brep-app --bake-worker [--plm-url URL] [--plm-token-file FILE] [--once | --poll SECS]
  --bake-worker       run as the PLM's headless bake worker: take each queued
                      family member or template copy, build it, keep it or
                      report why it failed. One line per job. No window.
  --plm-url URL       the PLM server (else BREP_PLM_URL, else plm.json in the
                      config dir, as a window launch reads them)
  --plm-token-file FILE
                      a file holding the API token; a worker-scoped one is
                      enough (else BREP_PLM_TOKEN, else the config dir's
                      plm-token). There is no flag holding the token itself:
                      a long-running process's command line is readable by
                      every user of the machine
  --once              one pass over the queue, then exit
  --poll SECS         seconds between passes when the queue is empty (default 30)";

/// Read the worker's flags, then resolve the server and token exactly as a
/// window launch does (`plm::config::resolve`: flag, then environment, then
/// the files in `config_dir`). `env` stands in for `std::env::var`.
#[cfg(not(target_arch = "wasm32"))]
pub fn parse_worker_args(
    args: impl IntoIterator<Item = String>,
    config_dir: &std::path::Path,
    env: &dyn Fn(&str) -> Option<String>,
) -> Result<WorkerArgs, String> {
    let mut flags = super::config::Flags::default();
    let (mut once, mut poll) = (false, None);
    let mut args = args.into_iter();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--plm-url" => flags.url = Some(args.next().ok_or("--plm-url needs the server's address")?),
            "--plm-token-file" => flags.token_file = Some(args.next().ok_or("--plm-token-file needs a file")?.into()),
            "--once" => once = true,
            "--poll" => {
                let value = args.next().ok_or("--poll needs a number of seconds")?;
                poll = Some(value.parse::<u64>().map_err(|_| format!("--poll: `{value}` is not a number of seconds"))?);
            }
            other => return Err(format!("--bake-worker does not take `{other}`")),
        }
    }
    if once && poll.is_some() {
        return Err("--once and --poll do not go together".into());
    }
    let config = super::config::resolve(config_dir, env, &flags)?.ok_or_else(|| {
        format!(
            "the bake worker needs a server: --plm-url, BREP_PLM_URL or {}",
            config_dir.join(super::config::URL_FILE).display()
        )
    })?;
    let token = config.token.ok_or_else(|| {
        format!(
            "the bake worker needs a token: --plm-token-file, BREP_PLM_TOKEN or {}",
            config_dir.join(super::config::TOKEN_FILE).display()
        )
    })?;
    Ok(WorkerArgs { url: config.url, token, once, poll: poll.unwrap_or(30) })
}

/// `brep-app --bake-worker …`: returns the process's exit code. Non-zero only
/// when the worker cannot start or is refused: a bad flag (2), or a server that
/// does not answer, refuses the token or does not serve this app (3). A job
/// that fails to build is a line of output, not an exit code.
#[cfg(not(target_arch = "wasm32"))]
pub fn worker_main(args: impl IntoIterator<Item = String>) -> i32 {
    let args: Vec<String> = args.into_iter().collect();
    if args.iter().any(|a| a == "-h" || a == "--help") {
        println!("usage: {WORKER_USAGE}");
        return 0;
    }
    let env = |name: &str| std::env::var(name).ok();
    let args = match parse_worker_args(args, &super::config::app_config_dir(), &env) {
        Ok(args) => args,
        Err(e) => {
            eprintln!("brep-app: {e}\nusage: {WORKER_USAGE}");
            return 2;
        }
    };
    let client = PlmClient::new(std::rc::Rc::new(super::transport::EhttpTransport::new(&args.url)));
    match block_on(client.sign_in_with_token(&args.token)) {
        Ok(me) => println!("bake worker: signed in to {} as {}", args.url, me.username),
        Err(e) => {
            eprintln!("brep-app: the bake worker cannot start: {e}");
            return 3;
        }
    }
    let renewal = Renewal::new(&args.url, &args.token, RENEW_EVERY);
    loop {
        match block_on(run_pass_with(&client, usize::MAX, Some(&renewal))) {
            Ok(outcomes) => {
                for outcome in &outcomes {
                    println!("{}", worker_line(outcome));
                }
                if args.once {
                    return 0;
                }
                if outcomes.is_empty() {
                    std::thread::sleep(std::time::Duration::from_secs(args.poll));
                }
            }
            // Refused mid-run: the token was revoked, or its scope narrowed.
            Err(e @ (PlmError::SignIn(_) | PlmError::Forbidden(_))) => {
                eprintln!("brep-app: the bake worker was refused: {e}");
                return 3;
            }
            Err(e) => {
                eprintln!("bake worker: {e}");
                if args.once {
                    return 3;
                }
                std::thread::sleep(std::time::Duration::from_secs(args.poll));
            }
        }
    }
}

/// One line of the worker's output for one job.
pub fn worker_line(outcome: &JobOutcome) -> String {
    let job = outcome.job();
    match outcome {
        JobOutcome::Done { .. } => format!("{} {}: done", job.number, job.revision_label),
        JobOutcome::Failed { error, .. } => format!("{} {}: failed: {error}", job.number, job.revision_label),
        JobOutcome::Dropped { why, .. } => format!("{} {}: dropped: {why}", job.number, job.revision_label),
    }
}

