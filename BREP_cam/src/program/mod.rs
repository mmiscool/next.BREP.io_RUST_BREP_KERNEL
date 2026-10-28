//! Program assembly (lane C4, row 4): one NC program from ordered operations
//! across setups.
//!
//! The [`Program`] is the single stream simulation and the post-processors
//! read: every motion keeps its operation identity, and the [`Section`] list
//! tells where each operation starts, which tool, work offset, spindle and
//! coolant it runs with, and what its report said. Assembly inserts the tool
//! changes, spindle and coolant switches and operation comments between the
//! operations' own streams, and polishes each operation's motions exactly once
//! on the way in ([`polish`]): the line filter within
//! `LinkingParams::smoothing_tolerance` and arc fitting when
//! `LinkingParams::fit_arcs`. Strategies hand over raw linked paths (their
//! `link::finish` runs with filter 0 and no arcs); previews and simulation read
//! the raw `OperationInput::toolpath`, the post reads `Program::motions`.

use crate::mesh::MeshSnapshot;
use crate::model::{
    Coolant, Feed, Heights, Machine, Motion, MotionKind, Operation, OperationId, OperationReport, OperationStatus,
    ProgramMotion, Setup, SetupId, SpindleDirection, StockToLeave, Strategy, Tool, ToolId, Toolpath, WorkOffset,
};
use crate::CamError;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub mod sheet;

/// Length unit of the program's coordinates and feeds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Units {
    #[default]
    Mm,
    In,
}

impl Units {
    pub fn as_str(&self) -> &'static str {
        match self {
            Units::Mm => "mm",
            Units::In => "in",
        }
    }
}

/// One operation's toolpath with the definitions it was generated from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct OperationInput {
    pub operation: Operation,
    pub tool: Tool,
    pub toolpath: Toolpath,
}

/// One setup and its operations in machining order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SetupInput {
    pub setup: Setup,
    pub operations: Vec<OperationInput>,
}

/// What the document hands to assembly.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProgramInput {
    /// Program name (file stem, header comment).
    #[serde(default)]
    pub name: String,
    /// Program number (`O1001`); the first setup's program info is the fallback.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub number: Option<u32>,
    #[serde(default)]
    pub units: Units,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub machine: Option<Machine>,
    #[serde(default)]
    pub setups: Vec<SetupInput>,
    /// Seconds charged per tool change in the time estimate.
    #[serde(default = "default_tool_change_seconds")]
    pub tool_change_seconds: f64,
    /// The complete protected mesh of each setup (part, fixtures, clamps) in
    /// that setup's linked-path coordinates: what lane C18's polish hook
    /// certifies smoothed arcs against. Only read for operations whose
    /// `Operation::polish` enables something; such an operation in a setup
    /// without a mesh is refused.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub protected: BTreeMap<SetupId, MeshSnapshot>,
}

fn default_tool_change_seconds() -> f64 {
    5.0
}

/// What assembly did to one operation's motions on the way into the stream.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Polish {
    /// Chord tolerance of the line filter (`0`: filter off).
    pub smoothing_tolerance: f64,
    /// Deviation allowed between fitted arcs and the chords (`0`: arc fitting off).
    pub arc_tolerance: f64,
    /// Points the line filter removed.
    pub points_removed: usize,
    /// Arcs fitted (each replaces a run of straight cuts).
    pub arcs_fitted: usize,
    /// Lane C18's feed optimisation ran (corner and small-radius windows at
    /// the reduced ratio) before the filter and the fitter.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub feed_optimization: bool,
    /// Lane C18's certified smoothing ran at this tolerance; it is the last
    /// geometry-changing step, so the line filter and the arc fitter above
    /// were skipped (`smoothingTolerance` and `arcTolerance` stay 0).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub certified_smoothing: Option<f64>,
}

/// Where one operation sits in the program stream and how it runs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Section {
    /// Zero-based section number.
    pub index: usize,
    pub operation: OperationId,
    pub name: String,
    /// Strategy name (the serde tag of the operation's strategy).
    pub strategy: String,
    pub setup: SetupId,
    pub setup_name: String,
    pub wcs: WorkOffset,
    pub tool: ToolId,
    pub tool_number: u32,
    /// Length offset register (`H`).
    pub length_offset: u32,
    /// Diameter offset register (`D`).
    pub diameter_offset: u32,
    /// The tool differs from the previous section's (always true for the first).
    pub tool_change: bool,
    /// Spindle speed (the rpm limit when `css` is on).
    pub spindle_rpm: f64,
    pub spindle_direction: SpindleDirection,
    /// Constant surface speed (turning): `surface_speed` in program units per minute.
    #[serde(default)]
    pub css: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface_speed: Option<f64>,
    /// The operation's cutting and plunge feeds (its override or the tool's).
    #[serde(default)]
    pub cutting_feed: f64,
    #[serde(default)]
    pub plunge_feed: f64,
    pub coolant: Coolant,
    pub heights: Heights,
    pub tolerance: f64,
    pub stock_to_leave: StockToLeave,
    /// Index of the section's first motion in `Program::motions` (the comment).
    pub first_motion: usize,
    /// Number of motions in the section, including the inserted ones.
    pub motion_count: usize,
    pub cutting_length: f64,
    /// Rough time estimate in seconds (lengths over feeds; no acceleration).
    pub estimated_seconds: f64,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub post_properties: BTreeMap<String, String>,
    /// Filtering and arc fitting applied to this section's motions.
    #[serde(default)]
    pub polish: Polish,
    pub report: OperationReport,
}

/// Totals over the program.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProgramSummary {
    pub motion_count: usize,
    pub cutting_length: f64,
    pub estimated_seconds: f64,
    pub tool_changes: usize,
    pub warning_count: usize,
    /// Points removed by the line filter over all sections.
    #[serde(default)]
    pub points_removed: usize,
    /// Arcs fitted over all sections.
    #[serde(default)]
    pub arcs_fitted: usize,
}

/// The assembled program: one motion stream plus its sections.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Program {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub number: Option<u32>,
    #[serde(default)]
    pub comment: String,
    pub units: Units,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub machine: Option<Machine>,
    /// Setups in program order.
    pub setups: Vec<Setup>,
    /// Tools in order of first use, each once.
    pub tools: Vec<Tool>,
    pub sections: Vec<Section>,
    pub motions: Vec<ProgramMotion>,
    pub summary: ProgramSummary,
}

impl Program {
    /// The motions of one section.
    pub fn section_motions(&self, section: &Section) -> &[ProgramMotion] {
        &self.motions[section.first_motion..section.first_motion + section.motion_count]
    }
}

fn invalid(m: String) -> CamError {
    CamError::InvalidInput(m)
}

/// The spindle as the stream last set it.
#[derive(Clone, Copy, Debug, PartialEq)]
enum SpindleState {
    Rpm(f64, SpindleDirection),
    /// Surface speed (program units per minute), rpm limit, direction.
    Css(f64, f64, SpindleDirection),
}

/// Straight-line length of a motion from `from` (arcs by their true length).
pub fn motion_length(from: Option<crate::model::Point3>, m: &Motion) -> f64 {
    let Some(to) = m.end() else { return 0.0 };
    let Some(from) = from else { return 0.0 };
    match &m.kind {
        MotionKind::ArcCw { center, plane, pitch, .. } | MotionKind::ArcCcw { center, plane, pitch, .. } => {
            let cw = matches!(m.kind, MotionKind::ArcCw { .. });
            match crate::arcfit::arc_sweep(from, to, *center, *plane, cw) {
                Some((sweep, radius)) => {
                    let arc = sweep * radius;
                    let rise = pitch * sweep / std::f64::consts::TAU;
                    arc.hypot(rise)
                }
                None => from.distance(to),
            }
        }
        MotionKind::Cycle { .. } => from.distance_xy(to),
        _ => from.distance(to),
    }
}

/// Polish one operation's motions for the program stream: the line filter
/// within `linking.smoothing_tolerance` (off at `0`), then arc fitting when
/// `linking.fit_arcs`, within the smoothing tolerance when it is set and the
/// operation's machining tolerance otherwise. Fitting touches only runs of
/// straight cuts, so a stream that was polished before passes through
/// unchanged with zero counts; arcs a strategy emitted (helical ramps) stay.
///
/// The deviation budget is honest but unproven here: a fitted arc stays
/// within `arc_tolerance` of chords that the strategy proved within the
/// operation tolerance of the surface; assembly has no mesh to re-prove the
/// arcs against.
pub fn polish(op: &Operation, motions: &[Motion]) -> Result<(Vec<Motion>, Polish), CamError> {
    let linking = &op.linking;
    let wrap = |e: CamError| invalid(format!("program: operation {}: {e}", op.id));
    let mut out = motions.to_vec();
    let mut polish = Polish::default();
    if linking.smoothing_tolerance > 0.0 {
        let (filtered, removed) = crate::filter::filter_motions(&out, linking.smoothing_tolerance).map_err(wrap)?;
        out = filtered;
        polish.smoothing_tolerance = linking.smoothing_tolerance;
        polish.points_removed = removed;
    }
    if linking.fit_arcs {
        let tolerance = if linking.smoothing_tolerance > 0.0 { linking.smoothing_tolerance } else { op.tolerance };
        let opts = crate::arcfit::ArcFitOptions::with_tolerance(tolerance);
        let (fitted, count) = crate::arcfit::fit_motions(&out, &opts).map_err(wrap)?;
        out = fitted;
        polish.arc_tolerance = tolerance;
        polish.arcs_fitted = count;
    }
    Ok((out, polish))
}

/// Assemble the program. Disabled operations are skipped; an operation whose
/// toolpath is not usable (pending, failed or cancelled) is an error, never
/// silently omitted. Tools are keyed by id; two tools with one id must be
/// identical.
pub fn assemble(input: &ProgramInput) -> Result<Program, CamError> {
    let mut sink = crate::NullSink;
    assemble_with(input, &mut crate::Runner::detached(&mut sink))
}

/// `assemble` with the caller's runner: lane C18's polish hook (feed
/// optimisation, certified smoothing) reports progress and honours
/// cancellation through it; nothing else in assembly does work worth a token.
pub fn assemble_with(input: &ProgramInput, runner: &mut crate::Runner<'_>) -> Result<Program, CamError> {
    if !input.tool_change_seconds.is_finite() || input.tool_change_seconds < 0.0 {
        return Err(invalid("program: tool change seconds must be finite and non-negative".into()));
    }
    if let Some(m) = &input.machine {
        m.validate().map_err(|e| invalid(format!("program: machine: {e}")))?;
    }
    let rapid_rate = input.machine.as_ref().map_or(30_000.0, |m| m.feed.rapid);
    let mut program = Program {
        name: input.name.clone(),
        number: input.number.or_else(|| input.setups.first().and_then(|s| s.setup.program.as_ref()).and_then(|p| p.number)),
        comment: input.setups.first().and_then(|s| s.setup.program.as_ref()).map(|p| p.comment.clone()).unwrap_or_default(),
        units: input.units,
        machine: input.machine.clone(),
        ..Default::default()
    };
    let mut tools: BTreeMap<ToolId, Tool> = BTreeMap::new();
    let mut tool_order: Vec<ToolId> = Vec::new();
    let mut current_tool: Option<ToolId> = None;
    let mut current_spindle: Option<SpindleState> = None;
    let mut current_coolant = Coolant::Off;
    let mut seen_ops: Vec<OperationId> = Vec::new();
    for setup_input in &input.setups {
        let setup = &setup_input.setup;
        setup.heights.validate().map_err(|e| invalid(format!("program: setup {}: {e}", setup.id)))?;
        if program.setups.iter().any(|s| s.id == setup.id) {
            return Err(invalid(format!("program: setup {} appears twice", setup.id)));
        }
        program.setups.push(setup.clone());
        for op_input in &setup_input.operations {
            let op = &op_input.operation;
            if !op.enabled {
                continue;
            }
            if op.setup != setup.id {
                return Err(invalid(format!("program: operation {} belongs to setup {}, not {}", op.id, op.setup, setup.id)));
            }
            if seen_ops.contains(&op.id) {
                return Err(invalid(format!("program: operation {} appears twice", op.id)));
            }
            seen_ops.push(op.id.clone());
            let tool = &op_input.tool;
            if tool.id != op.tool || op_input.toolpath.tool != tool.id || op_input.toolpath.operation != op.id {
                return Err(invalid(format!("program: operation {} and its tool/toolpath identities disagree", op.id)));
            }
            match tools.get(&tool.id) {
                Some(existing) if existing != tool => {
                    return Err(invalid(format!("program: tool {} is defined twice with different definitions", tool.id)));
                }
                Some(_) => {}
                None => {
                    tools.insert(tool.id.clone(), tool.clone());
                    tool_order.push(tool.id.clone());
                }
            }
            let path = &op_input.toolpath;
            if !path.report.is_usable() {
                return Err(invalid(format!("program: operation {} has no usable toolpath ({:?})", op.id, path.report.status)));
            }
            if let Some(i) = path.first_non_finite() {
                return Err(invalid(format!("program: operation {} motion {i} is not finite", op.id)));
            }
            let feeds = op.feeds.as_ref().unwrap_or(&tool.feeds);
            if matches!(op.strategy, Strategy::ManualNc(_)) {
                // Lane C15's manual NC entries: exactly the toolpath's own
                // entries, in order. No section comment, no implicit tool,
                // spindle or coolant state, no polish. An explicit tool change
                // is tracked like a machine would see it (it stops the
                // spindle and the coolant); a stop does the same on most
                // controls (Fanuc/Haas M00, an active M01), so the next
                // machining operation restarts them.
                if path.motions.is_empty() {
                    return Err(invalid(format!("program: manual NC operation {} has no entries", op.id)));
                }
                let first_motion = program.motions.len();
                let mut tool_change = false;
                let mut seconds = 0.0;
                for m in &path.motions {
                    match &m.kind {
                        MotionKind::Comment { .. } | MotionKind::PassThrough { .. } => {}
                        MotionKind::Stop { .. } => {
                            current_spindle = None;
                            current_coolant = Coolant::Off;
                        }
                        MotionKind::Dwell { seconds: s } => seconds += s,
                        MotionKind::ToolChange { tool: t } => {
                            if *t != tool.id {
                                return Err(invalid(format!(
                                    "program: manual NC operation {}: the tool change names {t}, not the operation's tool {} (the program carries only that definition)",
                                    op.id, tool.id
                                )));
                            }
                            tool_change = true;
                            seconds += input.tool_change_seconds;
                            current_tool = Some(t.clone());
                            current_spindle = None;
                            current_coolant = Coolant::Off;
                        }
                        other => {
                            return Err(invalid(format!(
                                "program: manual NC operation {} carries a {} motion; manual entries are stop, dwell, comment, pass-through and tool change",
                                op.id,
                                other.kind_name()
                            )));
                        }
                    }
                    program.motions.push(ProgramMotion { operation: op.id.clone(), motion: m.clone() });
                }
                let (spindle_rpm, spindle_direction) = match current_spindle {
                    Some(SpindleState::Rpm(rpm, direction)) => (rpm, direction),
                    Some(SpindleState::Css(_, max, direction)) => (max, direction),
                    None => (0.0, SpindleDirection::default()),
                };
                program.sections.push(Section {
                    index: program.sections.len(),
                    operation: op.id.clone(),
                    name: op.name.clone(),
                    strategy: strategy_name(op),
                    setup: setup.id.clone(),
                    setup_name: setup.name.clone(),
                    wcs: setup.wcs.clone(),
                    tool: tool.id.clone(),
                    tool_number: tool.number,
                    length_offset: tool.length_offset.unwrap_or(tool.number),
                    diameter_offset: tool.diameter_offset.unwrap_or(tool.number),
                    tool_change,
                    spindle_rpm,
                    spindle_direction,
                    css: matches!(current_spindle, Some(SpindleState::Css(..))),
                    surface_speed: match current_spindle {
                        Some(SpindleState::Css(v, _, _)) => Some(v),
                        _ => None,
                    },
                    cutting_feed: feeds.cutting_feed,
                    plunge_feed: feeds.plunge_feed,
                    coolant: current_coolant,
                    heights: op.heights.clone().unwrap_or_else(|| setup.heights.clone()),
                    tolerance: op.tolerance,
                    stock_to_leave: op.stock_to_leave.clone(),
                    first_motion,
                    motion_count: program.motions.len() - first_motion,
                    cutting_length: 0.0,
                    estimated_seconds: seconds,
                    post_properties: op.post_properties.clone(),
                    polish: Polish::default(),
                    report: path.report.clone(),
                });
                continue;
            }
            feeds.validate().map_err(|e| invalid(format!("program: operation {}: {e}", op.id)))?;
            op.linking.validate().map_err(|e| invalid(format!("program: operation {}: linking: {e}", op.id)))?;
            if !op.tolerance.is_finite() || op.tolerance <= 0.0 {
                return Err(invalid(format!("program: operation {}: tolerance must be finite and positive", op.id)));
            }
            // Lane C18's hook first (feed optimisation, then certified
            // smoothing), on the relinked path, once; then this crate's
            // line filter and arc fitter, unless certified smoothing ran:
            // it is the last geometry-changing step and nothing unchecked
            // may follow it.
            let (polished, polish) = if op.polish.feed_optimization.enabled || op.polish.smoothing.enabled {
                op.polish.validate().map_err(|e| invalid(format!("program: operation {}: polish: {e}", op.id)))?;
                // Only certified smoothing reads the protected mesh; feed
                // optimisation changes feeds, not geometry, and needs none.
                let empty = MeshSnapshot::empty();
                let mesh = match input.protected.get(&setup.id) {
                    Some(mesh) => mesh,
                    None if !op.polish.smoothing.enabled => &empty,
                    None => {
                        return Err(invalid(format!(
                            "program: operation {}: certified smoothing needs the complete protected mesh of setup {} (ProgramInput::protected)",
                            op.id, setup.id
                        )));
                    }
                };
                let applied = crate::polish::apply(path, op, tool, mesh, crate::Tolerance::default(), runner)
                    .map_err(|e| invalid(format!("program: operation {}: {e}", op.id)))?;
                if op.polish.smoothing.enabled {
                    let polish = Polish { feed_optimization: op.polish.feed_optimization.enabled, certified_smoothing: Some(op.polish.smoothing.tolerance), ..Default::default() };
                    (applied.motions, polish)
                } else {
                    let (out, mut polish) = polish(op, &applied.motions)?;
                    polish.feed_optimization = true;
                    (out, polish)
                }
            } else {
                polish(op, &path.motions)?
            };
            let first_motion = program.motions.len();
            let mut push = |m: Motion| program.motions.push(ProgramMotion { operation: op.id.clone(), motion: m });
            push(Motion::new(MotionKind::Comment { text: op.name.clone() }, Feed::RAPID));
            let tool_change = current_tool.as_ref() != Some(&tool.id);
            if tool_change {
                push(Motion::new(MotionKind::ToolChange { tool: tool.id.clone() }, Feed::RAPID));
                current_tool = Some(tool.id.clone());
                // A tool change stops the spindle and coolant on the machine.
                current_spindle = None;
                current_coolant = Coolant::Off;
            }
            // The spindle the operation wants: rpm, or constant surface speed
            // with its rpm limit (turning). A toolpath that sets the spindle
            // itself before its first move (turning streams do) is left alone.
            let wanted = if feeds.css {
                let (Some(v), Some(max)) = (feeds.surface_speed, feeds.surface_speed_limit_rpm) else {
                    return Err(invalid(format!("program: operation {}: constant surface speed needs a surface speed and an rpm limit", op.id)));
                };
                SpindleState::Css(v, max, feeds.spindle_direction)
            } else {
                SpindleState::Rpm(feeds.spindle_rpm, feeds.spindle_direction)
            };
            let prefix: Vec<&Motion> = path.motions.iter().take_while(|m| m.end().is_none()).collect();
            let own = prefix.iter().rev().find_map(|m| match &m.kind {
                MotionKind::SpindleOn { rpm, direction } => Some(SpindleState::Rpm(*rpm, *direction)),
                MotionKind::SpindleSurfaceSpeed { surface_speed, max_rpm, direction } => Some(SpindleState::Css(*surface_speed, *max_rpm, *direction)),
                _ => None,
            });
            match own {
                Some(state) => current_spindle = Some(state),
                None => {
                    if current_spindle != Some(wanted) {
                        push(match wanted {
                            SpindleState::Rpm(rpm, direction) => Motion::new(MotionKind::SpindleOn { rpm, direction }, Feed::RAPID),
                            SpindleState::Css(surface_speed, max_rpm, direction) => Motion::new(MotionKind::SpindleSurfaceSpeed { surface_speed, max_rpm, direction }, Feed::RAPID),
                        });
                        current_spindle = Some(wanted);
                    }
                }
            }
            let spindle = match current_spindle.expect("set above") {
                SpindleState::Rpm(rpm, direction) => (rpm, direction),
                SpindleState::Css(_, max, direction) => (max, direction),
            };
            if current_coolant != tool.coolant {
                push(Motion::new(MotionKind::Coolant { mode: tool.coolant }, Feed::RAPID));
                current_coolant = tool.coolant;
            }
            // Time and length over the operation's own motions.
            let mut at = None;
            let mut seconds = if tool_change { input.tool_change_seconds } else { 0.0 };
            let mut cutting = 0.0;
            for m in &polished {
                // A canned turning thread: every pass along the thread at the
                // synchronized feed, the clearance travel at rapid.
                let thread = match &m.kind {
                    MotionKind::TurnThreadCycle { cycle } => {
                        let passes = (cycle.depths.len() + cycle.spring_passes as usize) as f64;
                        let along = (cycle.start.z - cycle.end.z).abs() * passes;
                        let x = cycle.start.x + if cycle.internal { 1.0 } else { -1.0 } * cycle.depths.last().copied().unwrap_or(0.0);
                        let across = 2.0 * (cycle.clearance_x - x).abs() * passes;
                        Some((along, across))
                    }
                    _ => None,
                };
                let len = match thread {
                    Some((along, _)) => along,
                    None => motion_length(at, m),
                };
                if m.kind.is_cutting() {
                    cutting += len;
                }
                if let Some((_, across)) = thread {
                    seconds += across / rapid_rate * 60.0;
                }
                let rate = match m.feed {
                    Feed::Rapid(_) => rapid_rate,
                    Feed::Rate(r) => r.max(1e-9),
                };
                seconds += len / rate * 60.0;
                match &m.kind {
                    MotionKind::Dwell { seconds: s } => seconds += s,
                    MotionKind::Cycle { at: p, cycle } => {
                        // Feed down and back up at the operation's plunge feed.
                        let depth = (p.z - cycle.depth_z()).abs() + (cycle.retract_z() - cycle.depth_z()).abs();
                        seconds += depth / feeds.plunge_feed.max(1e-9) * 60.0;
                    }
                    _ => {}
                }
                if let Some(p) = m.end() {
                    at = Some(p);
                }
                push(m.clone());
            }
            let count = program.motions.len() - first_motion;
            program.sections.push(Section {
                index: program.sections.len(),
                operation: op.id.clone(),
                name: op.name.clone(),
                strategy: strategy_name(op),
                setup: setup.id.clone(),
                setup_name: setup.name.clone(),
                wcs: setup.wcs.clone(),
                tool: tool.id.clone(),
                tool_number: tool.number,
                length_offset: tool.length_offset.unwrap_or(tool.number),
                diameter_offset: tool.diameter_offset.unwrap_or(tool.number),
                tool_change,
                spindle_rpm: spindle.0,
                spindle_direction: spindle.1,
                css: matches!(current_spindle, Some(SpindleState::Css(..))),
                surface_speed: match current_spindle {
                    Some(SpindleState::Css(v, _, _)) => Some(v),
                    _ => None,
                },
                cutting_feed: feeds.cutting_feed,
                plunge_feed: feeds.plunge_feed,
                coolant: tool.coolant,
                heights: op.heights.clone().unwrap_or_else(|| setup.heights.clone()),
                tolerance: op.tolerance,
                stock_to_leave: op.stock_to_leave.clone(),
                first_motion,
                motion_count: count,
                cutting_length: cutting,
                estimated_seconds: seconds,
                post_properties: op.post_properties.clone(),
                polish,
                report: path.report.clone(),
            });
        }
    }
    if program.sections.is_empty() {
        return Err(invalid("program: no enabled operations with a toolpath".into()));
    }
    // Close: the last section ends spindle and coolant.
    let last_op = program.sections.last().map(|s| s.operation.clone()).expect("a section exists");
    if current_coolant != Coolant::Off {
        program.motions.push(ProgramMotion { operation: last_op.clone(), motion: Motion::new(MotionKind::Coolant { mode: Coolant::Off }, Feed::RAPID) });
    }
    program.motions.push(ProgramMotion { operation: last_op, motion: Motion::new(MotionKind::SpindleOff, Feed::RAPID) });
    if let Some(last) = program.sections.last_mut() {
        last.motion_count = program.motions.len() - last.first_motion;
    }
    program.tools = tool_order.iter().map(|id| tools[id].clone()).collect();
    program.summary = ProgramSummary {
        motion_count: program.motions.len(),
        cutting_length: program.sections.iter().map(|s| s.cutting_length).sum(),
        estimated_seconds: program.sections.iter().map(|s| s.estimated_seconds).sum(),
        tool_changes: program.sections.iter().filter(|s| s.tool_change).count(),
        warning_count: program.sections.iter().map(|s| s.report.warnings.len()).sum(),
        points_removed: program.sections.iter().map(|s| s.polish.points_removed).sum(),
        arcs_fitted: program.sections.iter().map(|s| s.polish.arcs_fitted).sum(),
    };
    let _ = OperationStatus::Ok;
    Ok(program)
}

/// The strategy's serde tag, by serializing just the tag.
pub fn strategy_name(op: &Operation) -> String {
    #[derive(Deserialize)]
    struct Tag {
        kind: String,
    }
    serde_json::to_value(&op.strategy)
        .ok()
        .and_then(|v| serde_json::from_value::<Tag>(v).ok())
        .map(|t| t.kind)
        .unwrap_or_else(|| "unknown".into())
}

