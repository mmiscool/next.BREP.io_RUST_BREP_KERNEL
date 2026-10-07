//! The `test-mcp` script format (spec Appendix E): a session description plus
//! a list of tool calls, each with optional expectations on the tool's JSON
//! result and optional image capture / comparison.
//!
//! The script vocabulary is the tool list itself — a step names a tool and
//! passes its arguments verbatim — so the format never needs to know which
//! tools exist. Only the `expect` mini-language is defined here.
use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Script {
    pub name: String,
    /// What the script checks, for the reader; ignored by the runner.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    #[serde(default = "default_backend")]
    pub backend: String,
    #[serde(default = "default_size")]
    pub size: [f32; 2],
    #[serde(default = "default_ppp")]
    pub ppp: f32,
    /// Start on the seed model instead of an empty document.
    #[serde(default)]
    pub seed: bool,
    /// A `.nbrep` to open before the first step (repo-relative or absolute).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub document: Option<String>,
    /// Files copied into the session store's `models/` directory before the app
    /// starts (repo-relative or absolute), so the app's own Open / Import
    /// dialogs LIST them. This is what a browser verifier's file-chooser
    /// interception was: a real file the user's picker hands the app. A script
    /// that instead navigates the explorer to the repo's fixture tree would
    /// leave the assembly import's part documents THERE, in the checkout.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub fixtures: Vec<String>,
    #[serde(default = "default_timeout")]
    pub timeout_ms: u64,
    /// Record an annotated walkthrough GIF from the steps that carry a
    /// [`Frame`]. Absent = an ordinary script, and the runner records nothing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub video: Option<Video>,
    /// Run a real PLM server beside the session ([`crate::plm_fixture`]):
    /// `brep-plm serve` on a loopback port over a fresh data directory,
    /// seeded through its own API before the app starts. Absent = no server,
    /// and nothing about the session changes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plm: Option<PlmSpec>,
    pub steps: Vec<Step>,
}

/// The PLM a script runs against. Every account gets a `full` token (or the
/// scope named), minted through the administrator's session the way a person
/// would mint one on the web page.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PlmSpec {
    /// Accounts to create besides `admin`, each with its own token.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub users: Vec<PlmUser>,
    /// Calls made through the API once the accounts exist and before the app
    /// starts. Each must answer below 400 unless it names `status`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub setup: Vec<PlmCall>,
    /// The account the APP is configured as: the fixture writes the server's
    /// URL to `<store>/plm.json` and this account's token to `<store>/plm-token`
    /// (D5's two files, in the session store, which is the app's config root).
    /// An empty string configures nothing, so the app starts file-only while
    /// the server still runs for the script's own `plm_request` steps.
    #[serde(default = "default_plm_app_user")]
    pub app_user: String,
    /// Write `plm-token` for `app_user` (default). `false` writes `plm.json`
    /// alone: a machine pointed at the PLM that has not signed in yet, which
    /// is where the Settings PLM tab's sign-in starts.
    #[serde(default = "default_true")]
    pub app_token: bool,
    /// Extra arguments to `brep-plm serve`, after `--bind` and `--data`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub args: Vec<String>,
    /// Extra environment for the server process.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub env: std::collections::BTreeMap<String, String>,
}

fn default_plm_app_user() -> String {
    "admin".into()
}

fn default_true() -> bool {
    true
}

/// An account the fixture creates.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PlmUser {
    pub username: String,
    /// At least 8 characters; default `test-mcp-password`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub password: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub groups: Vec<String>,
    /// The token's scope: `full` (default), `read` or `worker`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scope: Option<String>,
}

/// One call to the PLM's API: a `setup` entry, and the arguments of a
/// `plm_request` step. Strings may use `${plm:NAME}` (see
/// [`crate::plm_fixture`]).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PlmCall {
    #[serde(default = "default_plm_method")]
    pub method: String,
    /// Relative to the server, e.g. `/api/store/index`.
    pub path: String,
    /// A JSON body.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body: Option<Value>,
    /// A raw text body, sent as-is (instead of `body`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub raw: Option<String>,
    /// The `Content-Type` a `raw` body is sent under (an upload's media type,
    /// e.g. `application/pdf`); without it a raw body carries none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub content_type: Option<String>,
    /// Whose token signs the call: `admin` (default), a created user, or
    /// `anonymous` for none.
    #[serde(default, rename = "as", skip_serializing_if = "Option::is_none")]
    pub as_user: Option<String>,
    /// Name values in the answer for later strings: `{"part": "/id"}` makes
    /// `${plm:part}` the answer's `id`. A pointer that finds nothing fails.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub bind: std::collections::BTreeMap<String, String>,
    /// The status this call must answer (setup only; a step asserts with
    /// `expect` on `/status`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub status: Option<u16>,
}

fn default_plm_method() -> String {
    "GET".into()
}

/// The walkthrough GIF a script records: where it goes and how it is encoded.
///
/// GIF, not a video codec, because the output is embedded in a markdown page
/// and in the generated help site and has to play there with no player, no
/// codec negotiation and no autoplay policy — and because the encoder is a
/// feature flag on `image`, which both this crate and the app already depend
/// on, rather than a new dependency or a system tool (there is no ffmpeg on the
/// build machine, and the docs build must work on a plain checkout).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Video {
    /// Where the GIF is written, relative to the video root (`brep-mcp test
    /// --video-root`, which `./build.sh docs-videos` sets to the repo root).
    /// Without a video root the GIF is written into the run's output directory
    /// only — which is what keeps the test gate unable to write into `docs/`.
    pub out: String,
    /// The lead drawn above every caption: the feature's name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub title: Option<String>,
    /// Pixel width of the recorded frames (the surface is 1400 points wide, so
    /// the default is a downscale). Height follows the aspect ratio.
    #[serde(default = "default_video_width")]
    pub width: u32,
    /// The GIF quantiser's speed knob, 1..=30: 1 weighs quality over time.
    #[serde(default = "default_video_speed")]
    pub speed: i32,
}

fn default_video_width() -> u32 {
    1000
}
fn default_video_speed() -> i32 {
    15
}
fn default_hold_ms() -> u32 {
    1600
}

/// One frame of the walkthrough, recorded BEFORE the step it is attached to
/// runs.
///
/// Before, deliberately: the frame a viewer needs is the one where the control
/// is still unpressed and ringed, with the caption saying what pressing it will
/// do. A frame showing a RESULT is attached to a step that does nothing (a
/// `wait_idle`).
///
/// Pacing is the `hold_ms` of each frame, not a frame rate. GIF stores a
/// per-frame delay, so holding a meaningful picture for two seconds costs one
/// frame, and a walkthrough is ten to twenty frames rather than three hundred.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Frame {
    /// What is happening, in one sentence.
    pub caption: String,
    /// Widget key(s) to ring, exactly as `hit_rects` publishes them. A key the
    /// app is not publishing FAILS the step.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub highlight: Option<Value>,
    /// Scene entities (`{kind, name}`, or an array) to ring in the 3D view —
    /// resolved through the app's own `locate`, so the ring lands where a click
    /// on them would. An entity that is not on screen FAILS the step.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub entity: Option<Value>,
    /// A surface point `[x, y]` in egui points, or an array of them, to ring.
    /// This is how a sketch click — which has neither a widget rect nor a named
    /// entity — is pointed at.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point: Option<Value>,
    /// How long this frame is held, in milliseconds (GIF's unit is 10 ms).
    #[serde(default = "default_hold_ms")]
    pub hold_ms: u32,
    /// Record the step's DRAG as motion instead of a single still: the
    /// walkthrough shows the pointer travelling and the model following it.
    /// Only meaningful on a `drag` / `drag_widget` / `drag_entity` step, and
    /// refused anywhere else ([`Script::parse`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion: Option<Motion>,
}

/// Film the drag this frame is attached to, rather than holding one picture of
/// the handle before it moves.
///
/// This is the ONE place the walkthroughs record motion, and it exists because
/// some of what the app offers cannot be shown any other way: a 3D gizmo's
/// arrowhead is dragged, and a still of a ringed arrowhead beside a caption
/// saying "drag it" teaches less than eight frames of the block actually
/// growing. Everything else in a walkthrough is still one held picture per
/// step — see the module docs on `Frame`.
///
/// The step's drag is NOT decomposed into several little drags: it is one
/// press, `frames` pointer moves with a capture after each, and one release,
/// so what the GIF shows is the drag the tool would have performed anyway and
/// the step's own `expect` reads the tool's ordinary result.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Motion {
    /// How many pictures are taken between the press and the release. The
    /// still of the handle before the press is captured as well and held for
    /// the frame's own `hold_ms`, so a `frames: 8` drag costs 9 GIF frames.
    #[serde(default = "default_motion_frames")]
    pub frames: u32,
    /// How long each MOTION frame is held, in milliseconds. Short — this is
    /// the one place a walkthrough is a film rather than a slide show, and at
    /// GIF's 10 ms granularity 120 ms reads as a steady pull.
    #[serde(default = "default_motion_hold_ms")]
    pub hold_ms: u32,
    /// How big the drawn pointer is in the recorded frames, as a multiple of
    /// the 12 px arrow an ordinary screenshot draws. The default is 3: at the
    /// walkthrough's downscale a 1x pointer is nine pixels tall and a reader
    /// cannot follow it.
    #[serde(default = "default_cursor_scale")]
    pub cursor_scale: u32,
}

fn default_motion_frames() -> u32 {
    8
}
fn default_motion_hold_ms() -> u32 {
    120
}
fn default_cursor_scale() -> u32 {
    3
}

/// The tools a [`Motion`] can film: the three drag compositions, which are the
/// only steps that have a press, a travel and a release to film.
pub const DRAG_TOOLS: [&str; 3] = ["drag", "drag_widget", "drag_entity"];

fn default_backend() -> String {
    "headless".into()
}
fn default_size() -> [f32; 2] {
    [1400.0, 960.0]
}
fn default_ppp() -> f32 {
    1.0
}
fn default_timeout() -> u64 {
    120_000
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Step {
    pub tool: String,
    #[serde(default)]
    pub args: Value,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub expect: Vec<Expect>,
    /// Save the step's image (if the tool produced one) under this name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub save: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub compare: Option<Compare>,
    /// A note for the reader; ignored by the runner.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    /// The step is expected to FAIL: the tool must return an error, and the
    /// error text must match this glob (`*` = anything).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expect_error: Option<String>,
    /// Capture values of this step's result for `${plm:NAME}` in later steps
    /// (`{"name": "/json/pointer"}`). Needs the script's `plm` block, which
    /// holds the variables; a pointer that finds nothing fails the step.
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub bind: std::collections::BTreeMap<String, String>,
    /// Record a walkthrough frame BEFORE this step runs. Only meaningful in a
    /// script that declares a [`Video`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub frame: Option<Frame>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Compare {
    /// Path under `BREP_mcp/tests/baselines/`.
    pub baseline: String,
    #[serde(default = "default_max_diff")]
    pub max_diff_ratio: f64,
}

fn default_max_diff() -> f64 {
    0.002
}

/// One expectation: a JSON pointer into the tool result plus exactly one
/// operator. `near` takes a relative tolerance `tol` (default 1e-6).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Expect {
    pub path: String,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub eq: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub ne: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gt: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gte: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lt: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lte: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub near: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tol: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contains: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub matches: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exists: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub len: Option<usize>,
}

/// Deserialize an optional field so that a WRITTEN `null` is `Some(Value::Null)`
/// and only an ABSENT key is `None`.
///
/// `Option<Value>` normally collapses the two, which makes `{"eq": null}` read
/// as "no operator given" — so the one way to assert that a field IS null (an
/// inactive PMI view, a solid with no colour override) parsed as a script
/// error. The distinction is exactly what this expectation format needs.
fn present<'de, D>(deserializer: D) -> Result<Option<Value>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    Value::deserialize(deserializer).map(Some)
}

/// Exit codes of `brep-mcp test`.
pub mod exit {
    pub const PASS: i32 = 0;
    pub const EXPECTATION_FAILED: i32 = 1;
    pub const TOOL_ERROR: i32 = 2;
    pub const HOST_ERROR: i32 = 3;
    pub const TIMEOUT: i32 = 4;
}

impl Script {
    pub fn parse(text: &str) -> Result<Self, String> {
        let s: Script = serde_json::from_str(text).map_err(|e| format!("script parse: {e}"))?;
        if s.steps.is_empty() {
            return Err("script has no steps".into());
        }
        for (i, step) in s.steps.iter().enumerate() {
            if let Some(frame) = &step.frame {
                if let Some(motion) = &frame.motion {
                    // A `motion` the runner cannot film is a silent nothing:
                    // the frame would record as an ordinary still and the
                    // author would be left looking at a GIF that does not move,
                    // wondering which end was wrong. Name it here instead.
                    if !DRAG_TOOLS.contains(&step.tool.as_str()) {
                        return Err(format!(
                            "step {i}: `frame.motion` films a DRAG, but this step is `{}` (one of {})",
                            step.tool,
                            DRAG_TOOLS.join(", ")
                        ));
                    }
                    if s.video.is_none() {
                        return Err(format!(
                            "step {i}: `frame.motion` only means something in a script that declares a `video`"
                        ));
                    }
                    if motion.frames == 0 {
                        return Err(format!("step {i}: `frame.motion.frames` must be at least 1"));
                    }
                    if motion.cursor_scale == 0 {
                        return Err(format!("step {i}: `frame.motion.cursor_scale` must be at least 1"));
                    }
                }
            }
            for (j, e) in step.expect.iter().enumerate() {
                if e.operator_count() != 1 {
                    return Err(format!(
                        "step {i} (`{}`) expect {j}: exactly one operator is required",
                        step.tool
                    ));
                }
                if !e.path.starts_with('/') && !e.path.is_empty() {
                    return Err(format!(
                        "step {i} expect {j}: `path` must be a JSON pointer starting with `/` (got `{}`)",
                        e.path
                    ));
                }
            }
        }
        Ok(s)
    }

    /// The JSON Schema of the format (`brep://script/format`), derived from
    /// these types.
    pub fn json_schema() -> Value {
        serde_json::to_value(schemars::schema_for!(Script)).unwrap_or(Value::Null)
    }
}

impl Expect {
    fn operator_count(&self) -> usize {
        [
            self.eq.is_some(),
            self.ne.is_some(),
            self.gt.is_some(),
            self.gte.is_some(),
            self.lt.is_some(),
            self.lte.is_some(),
            self.near.is_some(),
            self.contains.is_some(),
            self.matches.is_some(),
            self.exists.is_some(),
            self.len.is_some(),
        ]
        .iter()
        .filter(|b| **b)
        .count()
    }

    /// Evaluate against a tool result. `Ok(())` when satisfied; `Err(why)`
    /// otherwise, with the actual value in the message.
    pub fn check(&self, result: &Value) -> Result<(), String> {
        let actual = result.pointer(&self.path);
        if let Some(exists) = self.exists {
            return if actual.is_some() == exists {
                Ok(())
            } else {
                Err(format!("{}: exists == {} expected {exists}", self.path, actual.is_some()))
            };
        }
        let Some(actual) = actual else {
            return Err(format!("{}: no value at that path", self.path));
        };
        let num = |v: &Value| v.as_f64();
        let shown = {
            let t = actual.to_string();
            if t.chars().count() > 160 {
                format!("{}…", t.chars().take(160).collect::<String>())
            } else {
                t
            }
        };
        let fail = |what: &str| Err(format!("{}: {what}; actual {shown}", self.path));
        if let Some(e) = &self.eq {
            return if actual == e { Ok(()) } else { fail(&format!("expected {e}")) };
        }
        if let Some(e) = &self.ne {
            return if actual != e { Ok(()) } else { fail(&format!("expected not {e}")) };
        }
        if let Some(b) = self.gt {
            return match num(actual) { Some(a) if a > b => Ok(()), _ => fail(&format!("expected > {b}")) };
        }
        if let Some(b) = self.gte {
            return match num(actual) { Some(a) if a >= b => Ok(()), _ => fail(&format!("expected >= {b}")) };
        }
        if let Some(b) = self.lt {
            return match num(actual) { Some(a) if a < b => Ok(()), _ => fail(&format!("expected < {b}")) };
        }
        if let Some(b) = self.lte {
            return match num(actual) { Some(a) if a <= b => Ok(()), _ => fail(&format!("expected <= {b}")) };
        }
        if let Some(b) = self.near {
            let tol = self.tol.unwrap_or(1e-6);
            return match num(actual) {
                Some(a) if (a - b).abs() <= tol * b.abs().max(1.0) => Ok(()),
                _ => fail(&format!("expected within {tol} (relative) of {b}")),
            };
        }
        if let Some(needle) = &self.contains {
            let ok = match (actual, needle) {
                (Value::String(s), Value::String(n)) => s.contains(n.as_str()),
                (Value::Array(a), n) => a.contains(n),
                (Value::Object(o), Value::String(n)) => o.contains_key(n),
                _ => false,
            };
            return if ok { Ok(()) } else { fail(&format!("expected to contain {needle}")) };
        }
        if let Some(pattern) = &self.matches {
            let Some(s) = actual.as_str() else { return fail("expected a string to match") };
            return if glob_match(pattern, s) { Ok(()) } else { fail(&format!("expected to match `{pattern}`")) };
        }
        if let Some(n) = self.len {
            let l = match actual {
                Value::Array(a) => Some(a.len()),
                Value::String(s) => Some(s.chars().count()),
                Value::Object(o) => Some(o.len()),
                _ => None,
            };
            return match l { Some(l) if l == n => Ok(()), _ => fail(&format!("expected length {n}")) };
        }
        Err(format!("{}: no operator", self.path))
    }
}

/// `matches` uses a small glob (`*` any run, `?` one char) rather than a regex
/// crate: enough for ids like `E*` and messages like `*cannot determine*`.
pub fn glob_match(pattern: &str, text: &str) -> bool {
    fn rec(p: &[char], t: &[char]) -> bool {
        match (p.first(), t.first()) {
            (None, None) => true,
            (Some('*'), _) => rec(&p[1..], t) || (!t.is_empty() && rec(p, &t[1..])),
            (Some('?'), Some(_)) => rec(&p[1..], &t[1..]),
            (Some(a), Some(b)) if a == b => rec(&p[1..], &t[1..]),
            _ => false,
        }
    }
    let p: Vec<char> = pattern.chars().collect();
    let t: Vec<char> = text.chars().collect();
    rec(&p, &t)
}

