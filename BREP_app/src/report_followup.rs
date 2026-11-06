//! Remembering the problem reports this user submitted, and telling them when
//! one is resolved — with no account.
//!
//! `POST /api/report` answers with the new report's id. The app keeps each id
//! it submitted, with when and the first line of the user's description, on
//! this machine: `submitted-reports.json` in the app's config directory on the
//! desktop, `localStorage` in the browser. A few seconds after start and every
//! few hours after, it asks `GET /api/public/reports/status?ids=…`, which says
//! what the public bug list already shows: each report's status. When a report
//! reaches `fixed`, `wontfix` or `closed`, the user gets one notice; a report
//! the server no longer has is forgotten.
//!
//! Inert until the app's entry point starts it, so a test or a harness neither
//! records nor asks.

use std::sync::mpsc::Receiver;

use brep_render::engine_state::{EngineState, NoticeSeverity};
use eframe::egui;
use serde::{Deserialize, Serialize};

/// The status lookup on the public reports server.
const STATUS_URL: &str = "https://next.brep.io/api/public/reports/status";

/// The feature name [`crate::offsite`]'s refusal sentence starts with.
const FEATURE: &str = "Problem report status";

/// The first check after start, and the period after it (seconds).
const FIRST_CHECK_SECS: f64 = 5.0;
const CHECK_EVERY_SECS: f64 = 3.0 * 60.0 * 60.0;

/// The most reports remembered (the server answers at most 100 per lookup).
const MAX_KEPT: usize = 50;

/// One report this user submitted.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Submitted {
    pub id: String,
    /// RFC 3339, when this app submitted it.
    pub submitted_at: String,
    /// The first line of the user's own description.
    pub summary: String,
    /// The status last read from the server.
    #[serde(default)]
    pub status: String,
    /// The resolved status the user was last told about.
    #[serde(default)]
    pub notified: Option<String>,
}

/// The sentence for a report that reached a resolved status, or `None`.
fn resolution_notice(report: &Submitted, status: &str) -> Option<String> {
    let outcome = match status {
        "fixed" => "was marked fixed. Thank you for reporting it!",
        "wontfix" => "was closed as won't fix.",
        "closed" => "was closed.",
        _ => return None,
    };
    let date = report.submitted_at.get(..10).unwrap_or(&report.submitted_at);
    let summary = if report.summary.is_empty() { "no description".to_string() } else { report.summary.clone() };
    Some(format!("Your problem report from {date} (\u{201c}{summary}\u{201d}) {outcome}"))
}

/// Fold a status answer into `reports`: statuses updated, missing reports
/// dropped. Returns the notices to show; `reports` then records them as told.
pub fn apply(reports: &mut Vec<Submitted>, body: &str) -> Result<Vec<String>, String> {
    #[derive(Deserialize)]
    struct Row {
        id: String,
        status: String,
    }
    #[derive(Deserialize)]
    struct Answer {
        #[serde(default)]
        reports: Vec<Row>,
        #[serde(default)]
        missing: Vec<String>,
    }
    let answer: Answer = serde_json::from_str(body).map_err(|e| format!("report status: {e}"))?;
    reports.retain(|r| !answer.missing.contains(&r.id));
    let mut notices = Vec::new();
    for row in answer.reports {
        let Some(report) = reports.iter_mut().find(|r| r.id == row.id) else { continue };
        report.status = row.status.clone();
        if report.notified.as_deref() == Some(row.status.as_str()) {
            continue;
        }
        if let Some(notice) = resolution_notice(report, &row.status) {
            notices.push(notice);
            report.notified = Some(row.status);
        }
    }
    Ok(notices)
}

/// The first line of a description, trimmed to a notice's length.
pub fn summary_of(description: &str) -> String {
    let line = description.lines().map(str::trim).find(|l| !l.is_empty()).unwrap_or("");
    let mut out: String = line.chars().take(80).collect();
    if line.chars().count() > 80 {
        out.push('\u{2026}');
    }
    out
}

/// The report id out of `POST /api/report`'s `201` body.
pub fn id_of_reply(body: &[u8]) -> Option<String> {
    let v: serde_json::Value = serde_json::from_slice(body).ok()?;
    let id = v.get("id")?.as_str()?;
    let ok = !id.is_empty() && id.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_');
    ok.then(|| id.to_string())
}

/// `2026-11-04T05:06:07Z` for Unix seconds (the app has no date crate; days
/// to a civil date by Howard Hinnant's `civil_from_days`).
fn rfc3339(secs: i64) -> String {
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!("{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z", rem / 3600, rem % 3600 / 60, rem % 60)
}

fn status_url(reports: &[Submitted]) -> String {
    let ids: Vec<String> = reports.iter().map(|r| crate::http::query_component(&r.id)).collect();
    format!("{STATUS_URL}?ids={}", ids.join(","))
}

// ---------------------------------------------------------------- where the list lives

#[cfg(not(target_arch = "wasm32"))]
mod saved {
    use super::Submitted;

    fn path() -> std::path::PathBuf {
        crate::plm::config::app_config_dir().join("submitted-reports.json")
    }

    pub fn load() -> Vec<Submitted> {
        super::parse_saved(&std::fs::read_to_string(path()).unwrap_or_default())
    }

    pub fn save(reports: &[Submitted]) {
        let path = path();
        let write = || -> std::io::Result<()> {
            if let Some(dir) = path.parent() {
                std::fs::create_dir_all(dir)?;
            }
            std::fs::write(&path, serde_json::to_vec_pretty(reports).unwrap_or_default())
        };
        if let Err(e) = write() {
            log::warn!("could not save {}: {e}", path.display());
        }
    }
}

#[cfg(target_arch = "wasm32")]
mod saved {
    use super::Submitted;

    const KEY: &str = "brep-submitted-reports";

    fn storage() -> Option<web_sys::Storage> {
        web_sys::window()?.local_storage().ok().flatten()
    }

    pub fn load() -> Vec<Submitted> {
        let text = storage().and_then(|s| s.get_item(KEY).ok().flatten()).unwrap_or_default();
        super::parse_saved(&text)
    }

    pub fn save(reports: &[Submitted]) {
        if let (Some(storage), Ok(text)) = (storage(), serde_json::to_string(reports)) {
            let _ = storage.set_item(KEY, &text);
        }
    }
}

fn parse_saved(text: &str) -> Vec<Submitted> {
    serde_json::from_str(text).unwrap_or_default()
}

// ---------------------------------------------------------------- the frame's half

/// The remembered reports and the periodic status check.
pub struct ReportFollowup {
    started: bool,
    reports: Vec<Submitted>,
    rx: Option<Receiver<Result<String, String>>>,
    /// egui time of the next check.
    next_check: f64,
}

impl Default for ReportFollowup {
    fn default() -> Self {
        Self::new()
    }
}

impl ReportFollowup {
    /// Inert: records nothing and asks nothing until [`Self::start`].
    pub fn new() -> Self {
        ReportFollowup { started: false, reports: Vec::new(), rx: None, next_check: 0.0 }
    }

    /// Load the remembered reports; the first check follows shortly.
    pub fn start(&mut self, ctx: &egui::Context) {
        self.started = true;
        self.reports = saved::load();
        self.next_check = ctx.input(|i| i.time) + FIRST_CHECK_SECS;
        if !self.reports.is_empty() {
            ctx.request_repaint_after(std::time::Duration::from_secs_f64(FIRST_CHECK_SECS));
        }
    }

    /// Remember a report this app just submitted.
    pub fn record(&mut self, id: String, summary: String) {
        if !self.started || self.reports.iter().any(|r| r.id == id) {
            return;
        }
        let submitted_at = rfc3339(crate::recovery::wall_clock() as i64);
        self.reports.push(Submitted { id, submitted_at, summary, status: "new".into(), notified: None });
        if self.reports.len() > MAX_KEPT {
            let extra = self.reports.len() - MAX_KEPT;
            self.reports.drain(..extra);
        }
        saved::save(&self.reports);
    }

    /// The reports remembered, oldest first.
    pub fn reports(&self) -> &[Submitted] {
        &self.reports
    }

    /// Once a frame: start a due check, and turn an answer into notices.
    pub fn tick(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        if !self.started {
            return;
        }
        if let Some(rx) = &self.rx {
            match rx.try_recv() {
                Ok(Ok(body)) => {
                    self.rx = None;
                    match apply(&mut self.reports, &body) {
                        Ok(notices) => {
                            for notice in notices {
                                state.push_notice_as(NoticeSeverity::Info, notice);
                            }
                            saved::save(&self.reports);
                        }
                        Err(e) => log::info!("{e}"),
                    }
                }
                Ok(Err(e)) => {
                    self.rx = None;
                    log::info!("report status: {e}");
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => return,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => self.rx = None,
            }
        }
        let now = ctx.input(|i| i.time);
        if now < self.next_check {
            return;
        }
        self.next_check = now + CHECK_EVERY_SECS;
        ctx.request_repaint_after(std::time::Duration::from_secs_f64(CHECK_EVERY_SECS));
        // A PLM-hosted page that may not reach the report site skips quietly.
        if self.reports.is_empty() || crate::offsite::refusal(FEATURE, STATUS_URL).is_some() {
            return;
        }
        let (tx, rx) = std::sync::mpsc::channel();
        let ctx = ctx.clone();
        ehttp::fetch(ehttp::Request::get(status_url(&self.reports)), move |result| {
            let out = match result {
                Ok(resp) if resp.ok => Ok(String::from_utf8_lossy(&resp.bytes).into_owned()),
                Ok(resp) => Err(format!("HTTP {} {}", resp.status, resp.status_text)),
                Err(e) => Err(e),
            };
            let _ = tx.send(out);
            ctx.request_repaint();
        });
        self.rx = Some(rx);
    }
}

