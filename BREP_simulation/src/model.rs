//! The `simulation` block: the motion studies of an assembly document.
//!
//! The block sits at the top level of the assembly document it animates, so a
//! study drives that document's own mates. Each study has a duration and a
//! frame rate (every output samples the same frames), motors and clips.
//! Constraints and PMI views are named by id; occurrences by path
//! (`ACOMP1:ACOMP3`).

use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashSet;

/// The document key the block is stored under.
pub const BLOCK_KEY: &str = "simulation";

/// The block's schema version.
pub const SCHEMA: u32 = 1;

/// The longest study, in seconds.
pub const MAX_DURATION: f64 = 600.0;

/// The frame rates a study may run at.
pub const FPS_RANGE: std::ops::RangeInclusive<u32> = 1..=120;

/// A new study's duration (s) and frame rate.
pub const DEFAULT_DURATION: f64 = 4.0;
pub const DEFAULT_FPS: u32 = 30;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Simulation {
    pub schema: u32,
    /// The study the workbench shows.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub active: Option<String>,
    #[serde(default)]
    pub studies: Vec<Study>,
    #[serde(default)]
    pub id_counter: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Study {
    pub id: String,
    pub name: String,
    /// Seconds.
    pub duration: f64,
    /// Frames per second: playback, scrubbing and every output use these frames.
    pub fps: u32,
    #[serde(default)]
    pub motors: Vec<Motor>,
    #[serde(default)]
    pub clips: Vec<Clip>,
    #[serde(default)]
    pub interference: Interference,
    /// The published motion (the baked frames of moving occurrences), for a
    /// manufacturing-instructions step to attach. Written by the workbench.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion: Option<Value>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Interference {
    /// Check every frame for parts that collide.
    pub enabled: bool,
}

impl Default for Interference {
    fn default() -> Self {
        Self { enabled: true }
    }
}

/// A mate driven over time. Its kind (distance or angle) is the constraint's.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Motor {
    pub id: String,
    /// The distance or angle constraint it drives.
    pub constraint: String,
    /// Seconds: the motor runs inside `[from, to]` and holds its last value after.
    pub from: f64,
    pub to: f64,
    pub drive: Drive,
}

/// How a motor's value moves.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "camelCase")]
pub enum Drive {
    /// From the mate's own value at `from`, at a constant rate (model units or
    /// degrees per second; negative runs backwards).
    Speed { speed: f64 },
    /// From `start` to `end` over the motor's span, along `shape`.
    Profile {
        shape: Shape,
        start: f64,
        end: f64,
        /// Trapezoid only: the fraction of the span spent accelerating (and
        /// again decelerating), `0 < accel ≤ 0.5`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        accel: Option<f64>,
    },
}

/// The motion profiles a motor can follow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Shape {
    /// Constant rate.
    Linear,
    /// Starts and stops smoothly (smoothstep).
    Ease,
    /// Accelerate, cruise, decelerate.
    Trapezoid,
}

impl Shape {
    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "linear" => Some(Self::Linear),
            "ease" => Some(Self::Ease),
            "trapezoid" => Some(Self::Trapezoid),
            _ => None,
        }
    }
}

/// The default trapezoid acceleration fraction.
pub const DEFAULT_ACCEL: f64 = 0.25;

/// One timeline row other than a motor, active over `[from, to]`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Clip {
    pub id: String,
    pub from: f64,
    pub to: f64,
    #[serde(flatten)]
    pub body: ClipBody,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum ClipBody {
    /// The constraint is off (suppressed) inside the span.
    Suppress { constraint: String },
    /// The camera moves through these PMI views' cameras, evenly over the
    /// span, and holds the last one after it.
    Camera { views: Vec<String> },
    /// The occurrences are hidden inside the span.
    Visibility { occurrences: Vec<String> },
    /// The occurrences' opacity ramps from `opacity[0]` to `opacity[1]` over
    /// the span.
    Transparency { occurrences: Vec<String>, opacity: [f64; 2] },
    /// The PMI view's section is on inside the span; with `sweep`, its planes
    /// move along their normals from `sweep[0]` to `sweep[1]` (model units).
    Section {
        view: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        sweep: Option<[f64; 2]>,
    },
    /// The exploded view's steps play over the span (collapse: backwards),
    /// and the last state holds after it.
    Explode {
        view: String,
        #[serde(default)]
        collapse: bool,
    },
}

impl ClipBody {
    /// The `kind` string.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::Suppress { .. } => "suppress",
            Self::Camera { .. } => "camera",
            Self::Visibility { .. } => "visibility",
            Self::Transparency { .. } => "transparency",
            Self::Section { .. } => "section",
            Self::Explode { .. } => "explode",
        }
    }
}

impl Default for Simulation {
    fn default() -> Self {
        Self { schema: SCHEMA, active: None, studies: Vec::new(), id_counter: 0 }
    }
}

impl Simulation {
    /// Read the block from its JSON, validated.
    pub fn from_value(value: &Value) -> Result<Self, String> {
        let block: Self = serde_json::from_value(value.clone()).map_err(|e| format!("simulation block: {e}"))?;
        block.validate()?;
        Ok(block)
    }

    /// Read the block out of a whole document, if it has one.
    pub fn from_document(document: &Value) -> Result<Option<Self>, String> {
        match document.get(BLOCK_KEY) {
            None | Some(Value::Null) => Ok(None),
            Some(block) => Self::from_value(block).map(Some),
        }
    }

    pub fn to_value(&self) -> Value {
        serde_json::to_value(self).expect("simulation block serializes")
    }

    /// A fresh id with `prefix` (`SIM` study, `MOT` motor, `CLP` clip).
    pub fn next_id(&mut self, prefix: &str) -> String {
        self.id_counter += 1;
        format!("{prefix}{}", self.id_counter)
    }

    /// Add an empty study named `name` (numbered when empty), make it active
    /// and return its id.
    pub fn add_study(&mut self, name: &str) -> String {
        let id = self.next_id("SIM");
        let name = if name.trim().is_empty() { format!("Study {}", self.studies.len() + 1) } else { name.trim().to_string() };
        self.studies.push(Study::new(&id, &name));
        self.active = Some(id.clone());
        id
    }

    pub fn study(&self, id: &str) -> Option<&Study> {
        self.studies.iter().find(|s| s.id == id)
    }

    pub fn study_mut(&mut self, id: &str) -> Option<&mut Study> {
        self.studies.iter_mut().find(|s| s.id == id)
    }

    /// The active study, else the first.
    pub fn active_study(&self) -> Option<&Study> {
        self.active.as_deref().and_then(|id| self.study(id)).or(self.studies.first())
    }

    /// The structural rules: a known schema, unique ids, and every study valid.
    pub fn validate(&self) -> Result<(), String> {
        if self.schema == 0 || self.schema > SCHEMA {
            return Err(format!("simulation schema {} is not supported (this build reads up to {SCHEMA})", self.schema));
        }
        let mut ids = HashSet::new();
        for study in &self.studies {
            for id in std::iter::once(&study.id).chain(study.motors.iter().map(|m| &m.id)).chain(study.clips.iter().map(|c| &c.id)) {
                if id.is_empty() {
                    return Err(format!("study '{}': an empty id", study.name));
                }
                if !ids.insert(id.as_str()) {
                    return Err(format!("duplicate id '{id}'"));
                }
            }
            study.validate()?;
        }
        if let Some(active) = &self.active {
            if self.study(active).is_none() {
                return Err(format!("the active study '{active}' does not exist"));
            }
        }
        Ok(())
    }
}

impl Study {
    pub fn new(id: &str, name: &str) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            duration: DEFAULT_DURATION,
            fps: DEFAULT_FPS,
            motors: Vec::new(),
            clips: Vec::new(),
            interference: Interference::default(),
            motion: None,
        }
    }

    /// Frames from 0 to the duration inclusive.
    pub fn frame_count(&self) -> usize {
        (self.duration * self.fps as f64).round() as usize + 1
    }

    /// The time of frame `index`, seconds.
    pub fn frame_time(&self, index: usize) -> f64 {
        (index as f64 / self.fps as f64).min(self.duration)
    }

    /// The frame nearest `time`.
    pub fn frame_at(&self, time: f64) -> usize {
        ((time.clamp(0.0, self.duration) * self.fps as f64).round() as usize).min(self.frame_count() - 1)
    }

    pub fn motor(&self, id: &str) -> Option<&Motor> {
        self.motors.iter().find(|m| m.id == id)
    }

    pub fn clip(&self, id: &str) -> Option<&Clip> {
        self.clips.iter().find(|c| c.id == id)
    }

    /// A study the sampler and every output can run: a sane duration and rate,
    /// spans inside the study, motors on distinct mates where they overlap,
    /// and clips that name what they act on.
    pub fn validate(&self) -> Result<(), String> {
        let name = &self.name;
        if !(self.duration.is_finite() && self.duration > 0.0 && self.duration <= MAX_DURATION) {
            return Err(format!("study '{name}': the duration must be over 0 and at most {MAX_DURATION} s, not {}", self.duration));
        }
        if !FPS_RANGE.contains(&self.fps) {
            return Err(format!("study '{name}': the frame rate must be {} to {} fps, not {}", FPS_RANGE.start(), FPS_RANGE.end(), self.fps));
        }
        let span = |what: &str, from: f64, to: f64| -> Result<(), String> {
            if !(from.is_finite() && to.is_finite() && from >= 0.0 && from < to && to <= self.duration + 1e-9) {
                return Err(format!("study '{name}': {what} must run inside 0–{} s with its start before its end, not {from}–{to}", self.duration));
            }
            Ok(())
        };
        for (index, motor) in self.motors.iter().enumerate() {
            let what = format!("motor '{}'", motor.id);
            span(&what, motor.from, motor.to)?;
            if motor.constraint.is_empty() {
                return Err(format!("study '{name}': {what} drives no mate"));
            }
            match &motor.drive {
                Drive::Speed { speed } if !speed.is_finite() => return Err(format!("study '{name}': {what} has no speed")),
                Drive::Profile { start, end, .. } if !(start.is_finite() && end.is_finite()) => {
                    return Err(format!("study '{name}': {what} needs a start and an end value"))
                }
                Drive::Profile { shape, accel: Some(accel), .. } if *shape == Shape::Trapezoid && !(*accel > 0.0 && *accel <= 0.5) => {
                    return Err(format!("study '{name}': {what} accelerates over 0–0.5 of its span, not {accel}"))
                }
                _ => {}
            }
            for other in &self.motors[..index] {
                if other.constraint == motor.constraint && other.from < motor.to && motor.from < other.to {
                    return Err(format!("study '{name}': motors '{}' and '{}' drive the same mate at the same time", other.id, motor.id));
                }
            }
        }
        for clip in &self.clips {
            let what = format!("{} clip '{}'", clip.body.kind(), clip.id);
            span(&what, clip.from, clip.to)?;
            let missing = match &clip.body {
                ClipBody::Suppress { constraint } => constraint.is_empty(),
                ClipBody::Camera { views } => views.is_empty() || views.iter().any(String::is_empty),
                ClipBody::Visibility { occurrences } | ClipBody::Transparency { occurrences, .. } => {
                    occurrences.is_empty() || occurrences.iter().any(String::is_empty)
                }
                ClipBody::Section { view, sweep } => view.is_empty() || sweep.is_some_and(|s| !(s[0].is_finite() && s[1].is_finite())),
                ClipBody::Explode { view, .. } => view.is_empty(),
            };
            if missing {
                return Err(format!("study '{name}': {what} names nothing to act on"));
            }
            if let ClipBody::Transparency { opacity, .. } = &clip.body {
                if opacity.iter().any(|o| !(0.0..=1.0).contains(o)) {
                    return Err(format!("study '{name}': {what} opacity must be 0–1, not {}–{}", opacity[0], opacity[1]));
                }
            }
        }
        Ok(())
    }
}

