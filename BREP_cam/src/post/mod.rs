//! Post-processing (lane C4, rows 5 and 6): posts are JavaScript files run
//! on the shared Boa interpreter (`BREP_script_core`, the same engine the
//! plugin host and the app's scripts use), with the host prelude in
//! `host.js` defining the post API. A post calls `post.define({...})` once
//! with its identity, properties, capabilities and callbacks (`onOpen`,
//! `onSection`, `onRapid`, `onLinear`, `onCircular`, `onCycle`, `onDwell`,
//! `onToolChange`, `onSpindle`, `onCoolant`, `onComment`, `onSectionEnd`,
//! `onClose`); the host walks the [`Program`] stream section by section and
//! returns the text. Arcs a post cannot take (plane, helix, sweep) are
//! split or linearized by the host within a declared tolerance; cycles a
//! post does not list are expanded into rapids, feeds and dwells.
//!
//! The types and the built-in post sources are always available; running a
//! post needs the `post-runtime` feature (Boa).

use crate::program::Program;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;

/// The host prelude evaluated before every post script.
pub const HOST_JS: &str = include_str!("host.js");

/// A post script shipped with the crate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BuiltinPost {
    /// Stable identifier (`fanuc`, `haas`, ...), also the script's `id`.
    pub id: &'static str,
    pub source: &'static str,
}

/// Every built-in post, in display order.
pub const BUILTIN_POSTS: &[BuiltinPost] = &[
    BuiltinPost { id: "fanuc", source: include_str!("../../posts/fanuc.js") },
    BuiltinPost { id: "haas", source: include_str!("../../posts/haas.js") },
    BuiltinPost { id: "grbl", source: include_str!("../../posts/grbl.js") },
    BuiltinPost { id: "linuxcnc", source: include_str!("../../posts/linuxcnc.js") },
    BuiltinPost { id: "mach", source: include_str!("../../posts/mach.js") },
    BuiltinPost { id: "heidenhain", source: include_str!("../../posts/heidenhain.js") },
    BuiltinPost { id: "siemens-840d", source: include_str!("../../posts/siemens-840d.js") },
    BuiltinPost { id: "marlin", source: include_str!("../../posts/marlin.js") },
    BuiltinPost { id: "okuma", source: include_str!("../../posts/okuma.js") },
    BuiltinPost { id: "mazak", source: include_str!("../../posts/mazak.js") },
    BuiltinPost { id: "fanuc-lathe", source: include_str!("../../posts/fanuc-lathe.js") },
    BuiltinPost { id: "haas-lathe", source: include_str!("../../posts/haas-lathe.js") },
    BuiltinPost { id: "okuma-lathe", source: include_str!("../../posts/okuma-lathe.js") },
    BuiltinPost { id: "mazak-lathe", source: include_str!("../../posts/mazak-lathe.js") },
];

/// Find a built-in post by id.
pub fn builtin(id: &str) -> Option<BuiltinPost> {
    BUILTIN_POSTS.iter().copied().find(|p| p.id == id)
}

/// One user-settable post property as the post declares it.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PropertySpec {
    pub title: String,
    #[serde(default)]
    pub description: String,
    /// `boolean`, `number`, `integer`, `string` or `enum`.
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(default)]
    pub default: Value,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub values: Option<Vec<Value>>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
}

/// What a post says about itself (`post.define` without the callbacks).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PostDefinition {
    pub id: String,
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub vendor: String,
    /// Output file extension without the dot.
    #[serde(default = "default_extension")]
    pub extension: String,
    /// `mm`, `in` or `auto` (the program's units).
    #[serde(default = "default_units")]
    pub units: String,
    /// `mill` or `lathe`: which programs the post is written for.
    #[serde(default = "default_machine")]
    pub machine: String,
    #[serde(default)]
    pub capabilities: Value,
    #[serde(default)]
    pub properties: BTreeMap<String, PropertySpec>,
}

fn default_extension() -> String {
    "nc".into()
}

fn default_units() -> String {
    "auto".into()
}

fn default_machine() -> String {
    "mill".into()
}

/// A request to post one program.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PostRequest {
    pub program: Program,
    /// Property values by name (typed, or strings the host coerces).
    #[serde(default)]
    pub properties: BTreeMap<String, Value>,
    /// Output file stem (`None`: the program name).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub file_name: Option<String>,
}

/// What a post produced.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PostOutput {
    pub text: String,
    /// `<stem>.<extension>`.
    pub file_name: String,
    pub extension: String,
    #[serde(default)]
    pub warnings: Vec<String>,
    #[serde(default)]
    pub notes: Vec<String>,
    /// The resolved property values the run used.
    #[serde(default)]
    pub properties: BTreeMap<String, Value>,
    /// `console.log` lines from the script.
    #[serde(default)]
    pub logs: Vec<String>,
}

/// Why a post run failed.
#[derive(Clone, Debug, PartialEq)]
pub struct PostError {
    pub message: String,
    pub logs: Vec<String>,
}

impl std::fmt::Display for PostError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for PostError {}

/// The output file name for a request and a definition.
pub fn output_file_name(request: &PostRequest, extension: &str) -> String {
    let stem = request.file_name.clone().unwrap_or_else(|| request.program.name.clone());
    let stem = if stem.trim().is_empty() { "program".to_string() } else { stem };
    let stem: String = stem.chars().map(|c| if c.is_alphanumeric() || "-_. ".contains(c) { c } else { '_' }).collect();
    format!("{stem}.{extension}")
}

#[cfg(feature = "post-runtime")]
mod runtime {
    use super::*;

    fn source_of(script: &str) -> String {
        let mut s = String::with_capacity(HOST_JS.len() + script.len() + 2);
        s.push_str(HOST_JS);
        s.push('\n');
        s.push_str(script);
        s.push('\n');
        s
    }

    fn call(script: &str, name: &str, function: &str, input: &Value) -> Result<(Value, Vec<String>), PostError> {
        let file = format!("{name}.js");
        match brep_script_core::call(&source_of(script), &file, function, input, &|_, _| Ok(())) {
            Ok(ok) => Ok((ok.value, ok.logs)),
            Err(e) => Err(PostError { message: e.message, logs: e.logs }),
        }
    }

    /// Evaluate a post script and return its definition without posting.
    pub fn describe(script: &str, name: &str) -> Result<PostDefinition, PostError> {
        let (value, logs) = call(script, name, "__brepDescribePost", &Value::Null)?;
        serde_json::from_value(value).map_err(|e| PostError { message: format!("{name}: invalid post definition: {e}"), logs })
    }

    /// Resolve the program through lane C10's kinematics when the machine has
    /// rotary axes: one chain per setup (its work offset origin), the rotary
    /// angles carried from one setup into the next, section ranges rebuilt the
    /// way `kinematics::post::resolve_program` builds them (an inserted block
    /// belongs to the section of the next sourced block, the trailing ones to
    /// the last section). The post's `capabilities.multiAxis` chooses the feed
    /// mode and TCP. `None` for three-axis machines, lathes and no machine.
    fn resolve_for_post(script: &str, name: &str, program: &Program) -> Result<Option<Value>, PostError> {
        use crate::kinematics::post::{resolve_motions, FeedMode, ResolveOptions};
        use crate::kinematics::{Chain, KinematicsOptions};
        use crate::model::MachineKind;
        let Some(machine) = &program.machine else { return Ok(None) };
        if !matches!(machine.kind, MachineKind::Mill4 { .. } | MachineKind::Mill5 { .. } | MachineKind::MillTurn { .. }) {
            return Ok(None);
        }
        // Live-tool streams (polar/cylindrical interpolation, synchronized
        // linear-rotary moves) program virtual coordinates and carry their
        // own rotary words: the resolver must not re-solve them.
        if program.motions.iter().any(|m| {
            matches!(m.motion.kind, crate::model::MotionKind::PolarInterpolation { .. } | crate::model::MotionKind::CylindricalInterpolation { .. } | crate::model::MotionKind::LinearRotary { .. })
        }) {
            return Ok(None);
        }
        let definition = describe(script, name)?;
        let caps = definition.capabilities.get("multiAxis").cloned().unwrap_or(Value::Null);
        let tcp = caps.get("tcp").and_then(Value::as_bool).unwrap_or(false);
        let feed_mode = match caps.get("feedMode").and_then(Value::as_str) {
            Some("inverseTime") => FeedMode::InverseTime,
            Some("dpm") => FeedMode::DegreesPerMinute,
            _ => FeedMode::Standard,
        };
        let base = ResolveOptions { kinematics: KinematicsOptions { tcp, ..Default::default() }, feed_mode, ..Default::default() };
        let fail = |m: String| PostError { message: format!("{name}: kinematics: {m}"), logs: Vec::new() };
        let point = |p: crate::model::Point3| serde_json::json!([p.x, p.y, p.z]);
        let mut joints: Vec<String> = Vec::new();
        let mut blocks: Vec<Value> = Vec::new();
        let mut sections: Vec<[usize; 2]> = vec![[0, 0]; program.sections.len()];
        let mut start_angles: Option<Vec<f64>> = None;
        let mut warnings: Vec<String> = Vec::new();
        for setup in &program.setups {
            let secs: Vec<&crate::program::Section> = program.sections.iter().filter(|s| s.setup == setup.id).collect();
            let (Some(first), Some(last)) = (secs.first(), secs.last()) else { continue };
            let lo = first.first_motion;
            let hi = last.first_motion + last.motion_count;
            if secs.windows(2).any(|w| w[1].first_motion != w[0].first_motion + w[0].motion_count) {
                return Err(fail(format!("setup {} sections are not contiguous in the program stream", setup.id)));
            }
            let chain = Chain::from_machine(machine, setup.wcs.origin).map_err(|e| fail(e.to_string()))?;
            let opts = ResolveOptions { start_angles: start_angles.clone(), ..base.clone() };
            let r = resolve_motions(&program.motions[lo..hi], &chain, &opts).map_err(|e| fail(e.to_string()))?;
            joints = r.joints.clone();
            warnings.extend(r.warnings.iter().map(|w| format!("{}: {}", w.code, w.message)));
            // Owner section of every block: by its own or the next sourced block's source.
            let section_of = |src: usize| secs.iter().position(|s| src + lo >= s.first_motion && src + lo < s.first_motion + s.motion_count);
            let mut owners: Vec<usize> = Vec::with_capacity(r.motions.len());
            for (k, b) in r.motions.iter().enumerate() {
                let src = b.source.or_else(|| r.motions[k..].iter().find_map(|q| q.source));
                let owner = src.and_then(section_of).unwrap_or(secs.len() - 1);
                owners.push(owner);
            }
            if owners.windows(2).any(|w| w[1] < w[0]) {
                return Err(fail("resolved blocks are out of section order".into()));
            }
            for (k, b) in r.motions.iter().enumerate() {
                let section = secs[owners[k]];
                let entry = &mut sections[section.index];
                if entry[1] == 0 {
                    entry[0] = blocks.len();
                }
                entry[1] += 1;
                blocks.push(serde_json::json!({
                    "source": b.source.map(|i| i + lo),
                    "operation": b.operation,
                    "motion": b.motion,
                    "linear": point(b.linear),
                    "center": b.center.map(point),
                    "rotary": b.rotary,
                    "inverseTime": b.inverse_time,
                    "rotaryFeed": b.rotary_feed,
                    "repositioned": b.repositioned,
                }));
            }
            if let Some(last) = r.motions.last() {
                start_angles = Some(last.rotary.iter().map(|w| w.angle_deg).collect());
            }
        }
        Ok(Some(serde_json::json!({ "joints": joints, "motions": blocks, "sections": sections, "warnings": warnings })))
    }

    /// Run a post over a program. Programs on machines with rotary axes are
    /// resolved through lane C10's kinematics first (see [`resolve_for_post`]).
    pub fn run(script: &str, name: &str, request: &PostRequest) -> Result<PostOutput, PostError> {
        let mut input = serde_json::to_value(request).map_err(|e| PostError { message: e.to_string(), logs: Vec::new() })?;
        if let Some(resolved) = resolve_for_post(script, name, &request.program)? {
            input["resolved"] = resolved;
        }
        let (value, logs) = call(script, name, "__brepRunPost", &input)?;
        #[derive(Deserialize)]
        #[serde(rename_all = "camelCase")]
        struct Raw {
            text: String,
            extension: String,
            #[serde(default)]
            warnings: Vec<String>,
            #[serde(default)]
            notes: Vec<String>,
            #[serde(default)]
            properties: BTreeMap<String, Value>,
        }
        let raw: Raw = serde_json::from_value(value).map_err(|e| PostError { message: format!("{name}: invalid post result: {e}"), logs: logs.clone() })?;
        Ok(PostOutput {
            file_name: output_file_name(request, &raw.extension),
            text: raw.text,
            extension: raw.extension,
            warnings: raw.warnings,
            notes: raw.notes,
            properties: raw.properties,
            logs,
        })
    }

    /// Run a built-in post by id.
    pub fn run_builtin(id: &str, request: &PostRequest) -> Result<PostOutput, PostError> {
        let post = builtin(id).ok_or_else(|| PostError { message: format!("unknown built-in post {id:?}"), logs: Vec::new() })?;
        run(post.source, post.id, request)
    }

    /// Definitions of every built-in post.
    pub fn describe_builtins() -> Result<Vec<PostDefinition>, PostError> {
        BUILTIN_POSTS.iter().map(|p| describe(p.source, p.id)).collect()
    }
}

#[cfg(feature = "post-runtime")]
pub use runtime::{describe, describe_builtins, run, run_builtin};

