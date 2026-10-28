//! Simulation player model (lane C17, row 1): a timeline over a program's
//! motion stream with play/pause/step/scrub/speed and per-operation ranges,
//! and a driver that advances C7's Z-dexel (3-axis) or the tri-dexel
//! (multi-axis) incrementally with checkpoints so scrubbing backwards
//! replays from the nearest earlier state. Material removal is applied per
//! whole motion; the tool pose is interpolated inside the current motion for
//! display. Pure model: the app's panel and renderer sit on top.

use crate::arcfit::{arc_sweep, from_plane_coords, plane_coords};
use crate::cutter::ToolProfile;
use crate::cycle_time::estimate_cycle_time;
use crate::model::{Diagnostic, Machine, Motion, MotionKind, OperationId, Point3};
use crate::multiaxis::sim::tridexel::{TriDexel, TriDexelOptions};
use crate::program::Program;
use crate::stock_sim::mesh::{stock_mesh, StockMesh};
use crate::stock_sim::removal::{simulate, simulate_on_machine, RemovalOptions};
use crate::stock_sim::DexelGrid;
use crate::{CamError, Cutter, Runner, Tolerance};
use serde::{Deserialize, Serialize};

/// Exact axisymmetric stock plus a raster used only for viewport/deviation display.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TurningStock {
    pub section: crate::planar::Region,
    pub target: crate::planar::Region,
    pub view: DexelGrid,
}
impl TurningStock {
    pub fn volume(&self)->f64 {
        self.section.loops.iter().flat_map(|l|&l.segments).map(|s| {
            let (a,b)=(s.start,s.end);(a.x+b.x)*(a.x*b.y-b.x*a.y)*std::f64::consts::PI/3.
        }).sum()
    }
    /// Rasterize the exact section; this grid never drives turning removal.
    pub fn refresh_view(&mut self) {
        for iy in 0..self.view.ny { for ix in 0..self.view.nx {
            let (x,y)=self.view.center(ix,iy);let radius=x.hypot(y);let mut hits=Vec::new();
            for edge in self.section.loops.iter().flat_map(|l|&l.segments) {
                let (a,b)=(edge.start,edge.end);
                if (a.x<=radius && b.x>radius)||(b.x<=radius && a.x>radius) {
                    hits.push(a.y+(b.y-a.y)*(radius-a.x)/(b.x-a.x));
                }
            }
            hits.sort_by(f64::total_cmp);
            self.view.columns[(iy*self.view.nx+ix) as usize].intervals=hits.chunks_exact(2).filter(|p|p[1]>p[0]).map(|p|[p[0],p[1]]).collect();
        }}
    }
}
/// A turning range carries its physical XZ envelope and certified protected section.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TurningRange {
    pub tool: crate::ToolId,
    pub envelope: crate::turning::ToolEnvelope,
    pub holders: Vec<crate::turning::ToolEnvelope>,
    pub protected: crate::planar::Region,
    pub chuck_z: f64,
}

/// Motions of one operation inside the timeline.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct OperationRange {
    pub operation: OperationId,
    pub first: usize,
    pub count: usize,
}

/// The program's motions with cumulative times.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Timeline {
    pub motions: Vec<Motion>,
    pub operations: Vec<OperationRange>,
    /// Seconds at the END of each motion (cumulative, non-decreasing).
    pub end_times: Vec<f64>,
    pub total_seconds: f64,
}

/// A position on the timeline: inside motion `motion` at `fraction` of it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Position {
    pub motion: usize,
    pub fraction: f64,
}

impl Timeline {
    /// From a motion stream with one operation per range.
    pub fn new(motions: Vec<Motion>, operations: Vec<OperationRange>, machine: &Machine) -> Timeline {
        let times = estimate_cycle_time(&motions, machine);
        let mut end_times = Vec::with_capacity(motions.len());
        let mut acc = 0.0;
        for t in &times.per_motion {
            acc += t.seconds.max(0.0);
            end_times.push(acc);
        }
        Timeline { motions, operations, end_times, total_seconds: acc }
    }

    /// From an assembled program: its sections become the operation ranges.
    pub fn from_program(program: &Program, machine: &Machine) -> Timeline {
        let motions: Vec<Motion> = program.motions.iter().map(|m| m.motion.clone()).collect();
        let operations = program
            .sections
            .iter()
            .map(|s| OperationRange { operation: s.operation.clone(), first: s.first_motion, count: s.motion_count })
            .collect();
        Timeline::new(motions, operations, machine)
    }

    pub fn len(&self) -> usize {
        self.motions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.motions.is_empty()
    }

    /// Start time of a motion.
    pub fn start_time(&self, motion: usize) -> f64 {
        if motion == 0 || self.end_times.is_empty() {
            0.0
        } else {
            self.end_times[(motion - 1).min(self.end_times.len() - 1)]
        }
    }

    /// Seconds at a position.
    pub fn time_at(&self, p: Position) -> f64 {
        if self.motions.is_empty() {
            return 0.0;
        }
        let m = p.motion.min(self.motions.len() - 1);
        let start = self.start_time(m);
        start + (self.end_times[m] - start) * p.fraction.clamp(0.0, 1.0)
    }

    /// The position at `seconds` (clamped to the timeline).
    pub fn position_at(&self, seconds: f64) -> Position {
        if self.motions.is_empty() {
            return Position::default();
        }
        let t = seconds.clamp(0.0, self.total_seconds);
        // A time on a motion boundary is the START of the next motion; the
        // end of the stream is the end of the last motion.
        if t >= self.total_seconds {
            return Position { motion: self.motions.len() - 1, fraction: 1.0 };
        }
        if t <= 0.0 {
            return Position::default();
        }
        let m = self.end_times.partition_point(|e| *e <= t).min(self.motions.len() - 1);
        let start = self.start_time(m);
        let span = self.end_times[m] - start;
        let fraction = if span > 0.0 { ((t - start) / span).clamp(0.0, 1.0) } else { 0.0 };
        Position { motion: m, fraction }
    }

    /// Index of the operation range containing a motion.
    pub fn operation_of(&self, motion: usize) -> Option<usize> {
        self.operations.iter().position(|r| motion >= r.first && motion < r.first + r.count)
    }

    /// Tool tip and axis at a position (interpolated along the motion; arcs
    /// by angle in their plane). `None` before the first positional motion.
    pub fn pose_at(&self, p: Position) -> Option<(Point3, Point3)> {
        if self.motions.is_empty() {
            return None;
        }
        let m = p.motion.min(self.motions.len() - 1);
        let f = p.fraction.clamp(0.0, 1.0);
        let end = self.motions[m].end();
        let axis_end = self.motions[m].tool_axis.unwrap_or(Point3::Z);
        // Previous position and axis.
        let mut prev: Option<(Point3, Point3)> = None;
        for k in (0..m).rev() {
            if let Some(e) = self.motions[k].end() {
                prev = Some((e, self.motions[k].tool_axis.unwrap_or(Point3::Z)));
                break;
            }
        }
        match (end, prev) {
            (Some(to), Some((from, a0))) => {
                let axis = crate::multiaxis::slerp(a0.normalized().unwrap_or(Point3::Z), axis_end.normalized().unwrap_or(Point3::Z), f);
                let tip = match &self.motions[m].kind {
                    MotionKind::ArcCw { center, plane, .. } | MotionKind::ArcCcw { center, plane, .. } => {
                        let cw = matches!(self.motions[m].kind, MotionKind::ArcCw { .. });
                        match arc_sweep(from, to, *center, *plane, cw) {
                            Some((sweep, _)) => {
                                let (cu, cv, _) = plane_coords(*center, *plane);
                                let (u0, v0, w0) = plane_coords(from, *plane);
                                let (_, _, w1) = plane_coords(to, *plane);
                                let r = (u0 - cu).hypot(v0 - cv);
                                let a0 = (v0 - cv).atan2(u0 - cu);
                                let a = a0 + if cw { -sweep * f } else { sweep * f };
                                from_plane_coords(cu + r * a.cos(), cv + r * a.sin(), w0 + (w1 - w0) * f, *plane)
                            }
                            None => from.lerp(to, f),
                        }
                    }
                    _ => from.lerp(to, f),
                };
                Some((tip, axis))
            }
            (Some(to), None) => Some((to, axis_end)),
            (None, prev) => prev,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PlayState {
    #[default]
    Paused,
    Playing,
}

/// Transport state over a timeline.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Player {
    pub timeline: Timeline,
    pub position: Position,
    pub state: PlayState,
    /// Playback rate relative to real machining time.
    pub speed: f64,
    /// Pause when a collision is reported at the motion being entered. Row 1
    /// stores the preference only; the panel (row 3) consults
    /// `collision::simulate_checked` / `check_machine` and acts on it.
    pub stop_at_collision: bool,
}

impl Player {
    pub fn new(timeline: Timeline) -> Player {
        Player { timeline, position: Position::default(), state: PlayState::Paused, speed: 1.0, stop_at_collision: true }
    }

    pub fn play(&mut self) {
        if !self.is_at_end() {
            self.state = PlayState::Playing;
        }
    }

    pub fn pause(&mut self) {
        self.state = PlayState::Paused;
    }

    pub fn toggle(&mut self) {
        match self.state {
            PlayState::Playing => self.pause(),
            PlayState::Paused => self.play(),
        }
    }

    pub fn set_speed(&mut self, speed: f64) -> Result<(), CamError> {
        if !speed.is_finite() || speed <= 0.0 {
            return Err(CamError::InvalidInput("player speed must be finite and positive".into()));
        }
        self.speed = speed;
        Ok(())
    }

    pub fn is_at_end(&self) -> bool {
        self.timeline.is_empty() || (self.position.motion + 1 >= self.timeline.len() && self.position.fraction >= 1.0)
    }

    pub fn time(&self) -> f64 {
        self.timeline.time_at(self.position)
    }

    /// Fraction of the total time, `0..=1`.
    pub fn progress(&self) -> f64 {
        if self.timeline.total_seconds > 0.0 {
            self.time() / self.timeline.total_seconds
        } else {
            0.0
        }
    }

    /// Jump to the start of a motion.
    pub fn seek_motion(&mut self, motion: usize) {
        if self.timeline.is_empty() {
            return;
        }
        self.position = Position { motion: motion.min(self.timeline.len() - 1), fraction: 0.0 };
    }

    /// Jump to a time.
    pub fn scrub_time(&mut self, seconds: f64) {
        self.position = self.timeline.position_at(seconds);
    }

    /// Jump to a fraction of the total time.
    pub fn scrub(&mut self, fraction: f64) {
        self.scrub_time(fraction.clamp(0.0, 1.0) * self.timeline.total_seconds);
    }

    /// To the end of the current motion (or the start of the next when already there).
    pub fn step_forward(&mut self) {
        if self.timeline.is_empty() {
            return;
        }
        let last = self.timeline.len() - 1;
        if self.position.fraction >= 1.0 && self.position.motion < last {
            self.position = Position { motion: self.position.motion + 1, fraction: 1.0 };
        } else {
            self.position.fraction = 1.0;
        }
        if self.is_at_end() {
            self.state = PlayState::Paused;
        }
    }

    /// To the start of the current motion (or of the previous when already there).
    pub fn step_back(&mut self) {
        if self.position.fraction > 0.0 {
            self.position.fraction = 0.0;
        } else if self.position.motion > 0 {
            self.position = Position { motion: self.position.motion - 1, fraction: 0.0 };
        }
    }

    /// Advance by `dt` real seconds while playing; returns whether the
    /// position changed.
    pub fn advance(&mut self, dt: f64) -> bool {
        if self.state != PlayState::Playing || !dt.is_finite() || dt <= 0.0 || self.timeline.is_empty() {
            return false;
        }
        let before = self.position;
        let t = self.time() + dt * self.speed;
        self.position = self.timeline.position_at(t);
        if t >= self.timeline.total_seconds {
            self.position = Position { motion: self.timeline.len() - 1, fraction: 1.0 };
            self.state = PlayState::Paused;
        }
        self.position != before
    }

    /// Number of motions fully completed at the current position: what the
    /// driver must have applied.
    pub fn completed(&self) -> usize {
        if self.timeline.is_empty() {
            0
        } else if self.position.fraction >= 1.0 {
            (self.position.motion + 1).min(self.timeline.len())
        } else {
            self.position.motion
        }
    }

    pub fn pose(&self) -> Option<(Point3, Point3)> {
        self.timeline.pose_at(self.position)
    }
}

/// The stock the player drives.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum StockModel {
    /// C7's Z-dexel: exact 3-axis removal; tilted motions are reported, not cut.
    Dexel(DexelGrid),
    /// The tri-dexel: removal along any tool axis.
    TriDexel(TriDexel),
    Turning(TurningStock),
}

impl StockModel {
    /// Surface mesh for the viewport (the Z view for the tri-dexel).
    pub fn mesh(&self) -> StockMesh {
        match self {
            StockModel::Dexel(g) => stock_mesh(g),
            StockModel::TriDexel(t) => stock_mesh(&t.to_dexel_grid()),
            StockModel::Turning(t) => stock_mesh(&t.view),
        }
    }

    pub fn volume(&self) -> f64 {
        match self {
            StockModel::Dexel(g) => g.volume(),
            StockModel::TriDexel(t) => t.volume(),
            StockModel::Turning(t) => t.volume(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct DriverOptions {
    /// Keep a copy of the stock every this many applied motions (backward
    /// scrubbing replays from the nearest copy).
    pub checkpoint_every: usize,
    /// Largest number of copies kept; the oldest is evicted first (a rewind
    /// past the oldest replays from the initial stock).
    pub max_checkpoints: usize,
    pub removal: RemovalOptions,
    pub tridexel: TriDexelOptions,
}

impl Default for DriverOptions {
    fn default() -> Self {
        Self { checkpoint_every: 50, max_checkpoints: 16, removal: RemovalOptions::default(), tridexel: TriDexelOptions::default() }
    }
}

/// One operation's motion range in the timeline with the cutter that cuts it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RangeCutter {
    pub first: usize,
    pub count: usize,
    pub cutter: Option<Cutter>,
    pub profile: Option<ToolProfile>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turning: Option<TurningRange>,
}

/// How [`SimDriver::advance_ranges`] ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AdvanceOutcome {
    /// The stock is at the target.
    Reached,
    /// The motion budget ran out first; call again.
    Budget,
    /// `keep_going` said stop between two motions; the stock is consistent
    /// at `applied`.
    Stopped,
}

/// Advances the stock through the timeline's motions incrementally.
#[derive(Clone, Debug, PartialEq)]
pub struct SimDriver {
    initial: StockModel,
    pub stock: StockModel,
    /// Motions fully applied (`timeline.motions[..applied]`).
    pub applied: usize,
    checkpoints: Vec<(usize, StockModel)>,
    pub options: DriverOptions,
    pub removed_volume: f64,
    /// Motion indices the Z-dexel could not simulate (tilted axes, non-XY arcs).
    pub unsupported: Vec<usize>,
    /// Warnings with the number of applied motions when they were raised
    /// (trimmed on rewind so scrubbing never duplicates them).
    pub warnings: Vec<(usize, Diagnostic)>,
    /// Incremented whenever the stock changes (for viewport mesh refresh).
    pub version: u64,
    /// The machine's kinematic chain: rotary positioning in the stream is
    /// applied through it (C7's `simulate_on_machine`); without one a
    /// nonzero rotary word is refused, never cut in the wrong frame.
    pub chain: Option<crate::kinematics::Chain>,
}

/// The chain a simulation runs on: the machine's, built as generation and
/// C7's removal build it (work offset at the machine origin). A machine
/// without one (a lathe) gives none.
pub fn sim_chain(machine: Option<&Machine>) -> Option<crate::kinematics::Chain> {
    machine.and_then(|m| crate::kinematics::Chain::from_machine(m, Point3::ZERO).ok())
}

/// True when the stream positions or drives a rotary axis (or switches an
/// interpolation mode): those cuts are replayed in the part frame, which
/// only the Z-dexel removal does.
pub fn uses_rotary(motions: &[Motion]) -> bool {
    motions.iter().any(|m| {
        matches!(
            m.kind,
            MotionKind::RotaryPosition { .. }
                | MotionKind::LinearRotary { .. }
                | MotionKind::PolarInterpolation { .. }
                | MotionKind::CylindricalInterpolation { .. }
        )
    })
}

/// The machine state the motions before `from` leave: every rotary axis's
/// last commanded angle and the interpolation modes, as motions that put a
/// replay of `from..` in the same frame.
fn machine_state_before(motions: &[Motion], from: usize) -> Vec<Motion> {
    let mut words: Vec<crate::model::RotaryWord> = Vec::new();
    let mut polar = false;
    let mut cylindrical: Option<f64> = None;
    for m in &motions[..from] {
        match &m.kind {
            MotionKind::RotaryPosition { axes } | MotionKind::LinearRotary { axes, .. } => {
                for w in axes {
                    match words.iter_mut().find(|v| v.name.eq_ignore_ascii_case(&w.name)) {
                        Some(v) => v.angle_deg = w.angle_deg,
                        None => words.push(w.clone()),
                    }
                }
            }
            MotionKind::PolarInterpolation { enabled } => polar = *enabled,
            MotionKind::CylindricalInterpolation { radius } => cylindrical = *radius,
            _ => {}
        }
    }
    let mut out = Vec::new();
    if words.iter().any(|w| w.angle_deg != 0.0) {
        out.push(Motion::new(MotionKind::RotaryPosition { axes: words }, crate::model::Feed::RAPID));
    }
    if polar {
        out.push(Motion::new(MotionKind::PolarInterpolation { enabled: true }, crate::model::Feed::RAPID));
    }
    if cylindrical.is_some() {
        out.push(Motion::new(MotionKind::CylindricalInterpolation { radius: cylindrical }, crate::model::Feed::RAPID));
    }
    out
}

impl SimDriver {
    /// Simulate on a machine (see [`SimDriver::chain`]).
    pub fn with_chain(mut self, chain: Option<crate::kinematics::Chain>) -> SimDriver {
        self.chain = chain;
        self
    }

    pub fn new(stock: StockModel, options: DriverOptions) -> Result<SimDriver, CamError> {
        if options.checkpoint_every == 0 || options.max_checkpoints == 0 {
            return Err(CamError::InvalidInput("driver: checkpoint interval and count must be positive".into()));
        }
        options.tridexel.validate()?;
        Ok(SimDriver {
            initial: stock.clone(),
            stock,
            applied: 0,
            checkpoints: Vec::new(),
            options,
            removed_volume: 0.0,
            unsupported: Vec::new(),
            warnings: Vec::new(),
            version: 0,
            chain: None,
        })
    }

    /// Move the stock back to the latest checkpoint at or before `target`
    /// (or the initial stock), leaving `applied <= target`: the caller then
    /// replays forward, choosing the cutter per motion range. No-op when the
    /// target is at or beyond the applied count.
    pub fn rewind(&mut self, target: usize) {
        if target < self.applied {
            self.rewind_to(target);
        }
    }

    /// Restore the latest checkpoint at or before `target` (or the initial stock).
    fn rewind_to(&mut self, target: usize) {
        while let Some((at, _)) = self.checkpoints.last() {
            if *at <= target {
                break;
            }
            self.checkpoints.pop();
        }
        match self.checkpoints.last() {
            Some((at, s)) => {
                self.stock = s.clone();
                self.applied = *at;
            }
            None => {
                self.stock = self.initial.clone();
                self.applied = 0;
            }
        }
        self.removed_volume = self.initial.volume() - self.stock.volume();
        let applied = self.applied;
        self.unsupported.retain(|i| *i < applied);
        self.warnings.retain(|(at, _)| *at <= applied);
        self.version += 1;
    }

    /// Bring the stock to the state after `target` motions of `timeline`
    /// with ONE cutter. A target behind the applied count rewinds to the
    /// nearest checkpoint first and replays with that same cutter: streams
    /// whose tool changes must call [`Self::rewind`] and then advance range
    /// by range with each range's cutter.
    pub fn advance_to(
        &mut self,
        timeline: &Timeline,
        target: usize,
        cutter: &Cutter,
        profile: &ToolProfile,
        tolerance: &Tolerance,
        runner: &mut Runner<'_>,
    ) -> Result<(), CamError> {
        let target = target.min(timeline.len());
        if target < self.applied {
            self.rewind_to(target);
        }
        while self.applied < target {
            let next_checkpoint = ((self.applied / self.options.checkpoint_every) + 1) * self.options.checkpoint_every;
            let stop = target.min(next_checkpoint);
            self.apply_slice(timeline, self.applied, stop, cutter, profile, tolerance, runner)?;
            if stop == next_checkpoint && stop < timeline.len() {
                self.checkpoints.push((stop, self.stock.clone()));
                if self.checkpoints.len() > self.options.max_checkpoints {
                    self.checkpoints.remove(0);
                }
            }
        }
        Ok(())
    }

    /// Apply motions `[from, to)` from the current position.
    fn apply_slice(
        &mut self,
        timeline: &Timeline,
        from: usize,
        to: usize,
        cutter: &Cutter,
        profile: &ToolProfile,
        tolerance: &Tolerance,
        runner: &mut Runner<'_>,
    ) -> Result<(), CamError> {
        // The machine state the earlier motions left (rotary angles,
        // interpolation modes), then a positioning motion to the tool's
        // current position: the slice's first motion cuts from there in the
        // frame a straight run would have.
        let mut slice: Vec<Motion> = machine_state_before(&timeline.motions, from);
        for k in (0..from).rev() {
            if let Some(e) = timeline.motions[k].end() {
                let mut m = Motion::rapid(e);
                m.tool_axis = timeline.motions[k].tool_axis;
                slice.push(m);
                break;
            }
        }
        let offset = slice.len();
        slice.extend(timeline.motions[from..to].iter().cloned());
        // Live milling after turning in one setup cuts the turned stock: its
        // Z-dexel view, kept in the same machine frame as the section.
        if let StockModel::Turning(turned) = &self.stock {
            self.stock = StockModel::Dexel(turned.view.clone());
        }
        let before = self.stock.volume();
        match &mut self.stock {
            StockModel::Dexel(grid) => {
                let report = match &self.chain {
                    Some(chain) => simulate_on_machine(grid, cutter, &slice, tolerance, &self.options.removal, chain, runner)?,
                    None => simulate(grid, cutter, &slice, tolerance, &self.options.removal, runner)?,
                };
                for i in report.unsupported {
                    if i >= offset {
                        self.unsupported.push(from + i - offset);
                    }
                }
                self.warnings.extend(report.warnings.into_iter().map(|w| (to, w)));
            }
            StockModel::TriDexel(td) => {
                let report = td.simulate(profile, &slice, &self.options.tridexel, tolerance, runner)?;
                self.warnings.extend(report.warnings.into_iter().map(|w| (to, w)));
            }
            StockModel::Turning(_) => return Err(CamError::InvalidInput("turning stock requires a turning range".into())),
        }
        self.removed_volume += before - self.stock.volume();
        self.applied = to;
        self.version += 1;
        Ok(())
    }

    /// Bring the stock toward `target` motions, at most `budget` motions
    /// per call, each motion cut by the cutter of the range that holds it
    /// (a motion outside every range uses the first range's cutter). A
    /// target behind the applied count rewinds to the nearest checkpoint
    /// first. `keep_going(applied)` is asked between motions, never inside
    /// one, so a stop leaves the stock consistent. This is the one stepping
    /// function the app's inline fallback and the background runner share.
    pub fn advance_ranges(
        &mut self,
        timeline: &Timeline,
        ranges: &[RangeCutter],
        target: usize,
        budget: usize,
        tolerance: &Tolerance,
        keep_going: &mut dyn FnMut(usize) -> bool,
    ) -> Result<AdvanceOutcome, CamError> {
        let target = target.min(timeline.len());
        self.rewind(target);
        if ranges.is_empty() && self.applied < target {
            return Err(CamError::InvalidInput("driver: no cutter for the timeline's motions".into()));
        }
        let mut done = 0usize;
        while self.applied < target {
            if done >= budget {
                return Ok(AdvanceOutcome::Budget);
            }
            if done > 0 && !keep_going(self.applied) {
                return Ok(AdvanceOutcome::Stopped);
            }
            let at = self.applied;
            let range = ranges.iter().find(|r| at >= r.first && at < r.first + r.count).unwrap_or(&ranges[0]);
            let mut sink = crate::NullSink;
            if let Some(turning) = &range.turning {
                let StockModel::Turning(stock)=&mut self.stock else {return Err(CamError::InvalidInput("turning range requires axisymmetric stock".into()));};
                if range.cutter.is_some() || range.profile.is_some() {return Err(CamError::InvalidInput("turning range cannot also carry a mill cutter".into()));}
                let start=timeline.motions[..at].iter().rev().find_map(|m|m.end()).or_else(||timeline.motions[at..].iter().find_map(|m|m.end())).ok_or_else(||CamError::InvalidInput("turning timeline has no position".into()))?;
                let state=crate::turning::sim::StockState::new(stock.section.clone(),*tolerance)?;
                let mut path=crate::Toolpath::new("turning-replay",turning.tool.clone());path.motions.push(timeline.motions[at].clone());
                let next=state.simulate(&path,start,&turning.envelope,&turning.holders,&turning.protected,turning.chuck_z,*tolerance,&mut Runner::detached(&mut sink))?;
                let before=stock.volume();stock.section=next.section().clone();stock.refresh_view();
                self.removed_volume+=before-stock.volume();self.applied=at+1;self.version+=1;
                if self.applied%self.options.checkpoint_every==0 {
                    self.checkpoints.push((self.applied,self.stock.clone()));
                    if self.checkpoints.len()>self.options.max_checkpoints {self.checkpoints.remove(0);}
                }
            } else {
                let cutter=range.cutter.as_ref().ok_or_else(||CamError::InvalidInput("mill range has no cutter".into()))?;
                let profile=range.profile.as_ref().ok_or_else(||CamError::InvalidInput("mill range has no profile".into()))?;
                self.advance_to(timeline, at + 1, cutter, profile, tolerance, &mut Runner::detached(&mut sink))?;
            }
            done += 1;
        }
        Ok(AdvanceOutcome::Reached)
    }

    pub fn checkpoint_count(&self) -> usize {
        self.checkpoints.len()
    }
}
