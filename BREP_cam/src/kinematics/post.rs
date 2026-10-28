//! Post resolution (lane C10, row 6; spec section 9): the rotary resolver
//! the post runtime calls before it walks a program.
//!
//! Input: the program's motion stream (setup coordinates, `tool_axis` per
//! motion) and the machine's [`Chain`]. Output: one [`ResolvedMotion`] per
//! emitted block, in order, each carrying the linear words, the rotary
//! words and (for feed moves with rotary motion) the inverse-time or
//! degrees-per-minute feed word:
//! - angles unwound and on the nearest branch (sequence rules), joint
//!   limits enforced (`LimitViolation` is an error, never a wrapped angle);
//! - a `WorkPlane` motion inserts a `RotaryPosition` with the angles that
//!   orient the tool along its `z_axis`; inside the work plane the linear
//!   words are the plane-local coordinates (what G68.2 / CYCLE800 / PLANE
//!   SPATIAL expect) and the rotaries hold;
//! - a feed move whose solution sits on the other branch is preceded by
//!   retract (along the current axis), rotate at the retracted distance,
//!   and plunge back (`repositioned` blocks);
//! - with TCP off, blocks whose joint interpolation would move the tip off
//!   the straight chord by more than `linearize_tolerance` are subdivided;
//!   every block is also limited to `max_segment_deg` of rotary travel;
//! - `TcpMode { enabled: true }` is inserted before the first tilted move
//!   outside a work plane when TCP is on, and switched off at the end.

use super::sequence::choose_pose;
use super::{Chain, KinematicsError, KinematicsOptions, Pose};
use crate::model::{Diagnostic, Feed, Motion, MotionKind, OperationId, Point3, ProgramMotion, RigidTransform, RotaryWord};
use crate::program::Program;

/// How the post expresses feed on moves with rotary motion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FeedMode {
    /// The feed word is passed through unchanged.
    #[default]
    Standard,
    /// G93: `inverse_time = 1 / minutes` per block.
    InverseTime,
    /// `rotary_feed` in degrees per minute per block.
    DegreesPerMinute,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ResolveOptions {
    pub kinematics: KinematicsOptions,
    pub feed_mode: FeedMode,
    /// Fastest rotary rate (deg/min) used to time pure rotations and to
    /// bound the time of mixed moves.
    pub rotary_max_deg_per_min: f64,
    /// Largest tip deviation from the straight chord tolerated for one block
    /// under joint interpolation (TCP off).
    pub linearize_tolerance: f64,
    /// Largest rotary travel per block, degrees.
    pub max_segment_deg: f64,
    /// Retract distance along the tool axis before a branch change.
    pub reposition_distance: f64,
    /// A feed move whose nearest solution needs more rotary travel than this
    /// (degrees) cannot be cut continuously: it is repositioned instead.
    pub reposition_travel_deg: f64,
    /// Rotary position before the program (`None`: zero).
    pub start_angles: Option<Vec<f64>>,
}

impl Default for ResolveOptions {
    fn default() -> Self {
        Self {
            kinematics: KinematicsOptions::default(),
            feed_mode: FeedMode::Standard,
            rotary_max_deg_per_min: 3_600.0,
            linearize_tolerance: 0.01,
            max_segment_deg: 5.0,
            reposition_distance: 10.0,
            reposition_travel_deg: 90.0,
            start_angles: None,
        }
    }
}

impl ResolveOptions {
    pub fn validate(&self, chain: &Chain) -> Result<(), KinematicsError> {
        self.kinematics.validate()?;
        let bad = |m: &str| Err(KinematicsError::InvalidInput(format!("post resolve: {m}")));
        if !self.rotary_max_deg_per_min.is_finite() || self.rotary_max_deg_per_min <= 0.0 {
            return bad("rotary rate must be positive");
        }
        if !self.linearize_tolerance.is_finite() || self.linearize_tolerance <= 0.0 {
            return bad("linearize tolerance must be positive");
        }
        if !self.max_segment_deg.is_finite() || self.max_segment_deg <= 0.0 {
            return bad("maximum segment must be positive");
        }
        if !self.reposition_distance.is_finite() || self.reposition_distance < 0.0 {
            return bad("reposition distance must be finite and non-negative");
        }
        if !self.reposition_travel_deg.is_finite() || self.reposition_travel_deg <= self.max_segment_deg {
            return bad("reposition travel must exceed the maximum segment");
        }
        if let Some(s) = &self.start_angles {
            if s.len() != chain.joints.len() || s.iter().any(|a| !a.is_finite()) {
                return bad("start angles do not match the chain");
            }
        }
        Ok(())
    }
}

/// One emitted block.
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedMotion {
    /// Index of the source motion; `None` for inserted blocks.
    pub source: Option<usize>,
    pub operation: OperationId,
    /// The motion in setup coordinates (subdivided or inserted ones included).
    pub motion: Motion,
    /// X/Y/Z words: machine axes (TCP off), the tip (TCP on), or the
    /// work-plane-local coordinates inside a `WorkPlane` section.
    pub linear: Point3,
    /// Arc centre in the same coordinates as `linear`, for arc motions.
    pub center: Option<Point3>,
    /// Rotary words in the chain's joint order.
    pub rotary: Vec<RotaryWord>,
    /// `1 / minutes` for feed moves with rotary motion (`FeedMode::InverseTime`).
    pub inverse_time: Option<f64>,
    /// Degrees per minute for feed moves with rotary motion (`FeedMode::DegreesPerMinute`).
    pub rotary_feed: Option<f64>,
    /// Part of an inserted retract-rotate-plunge sequence.
    pub repositioned: bool,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ResolvedProgram {
    /// Rotary axis names in word order.
    pub joints: Vec<String>,
    pub motions: Vec<ResolvedMotion>,
    /// `[first, count]` into `motions` per input section.
    pub sections: Vec<[usize; 2]>,
    pub warnings: Vec<Diagnostic>,
    pub repositions: usize,
    /// Blocks added by subdivision.
    pub subdivided: usize,
}

struct State<'a> {
    chain: &'a Chain,
    opts: &'a ResolveOptions,
    angles: Vec<f64>,
    tip: Option<Point3>,
    axis: Point3,
    work_plane: Option<RigidTransform>,
    tcp_announced: bool,
    out: Vec<ResolvedMotion>,
    repositions: usize,
    subdivided: usize,
    warnings: Vec<Diagnostic>,
}

fn words(chain: &Chain, angles: &[f64]) -> Vec<RotaryWord> {
    chain.joints.iter().zip(angles).map(|(j, a)| RotaryWord { name: j.name.clone(), angle_deg: *a }).collect()
}

/// Spherical interpolation of unit vectors.
fn slerp(a: Point3, b: Point3, t: f64) -> Point3 {
    let d = a.dot(b).clamp(-1.0, 1.0);
    let theta = d.acos();
    if theta < 1e-12 {
        return a;
    }
    let s = theta.sin();
    a.scale(((1.0 - t) * theta).sin() / s).add(b.scale((t * theta).sin() / s)).normalized().unwrap_or(b)
}

fn with_end(kind: &MotionKind, to: Point3) -> MotionKind {
    match kind {
        MotionKind::Rapid { .. } => MotionKind::Rapid { to },
        MotionKind::Plunge { .. } => MotionKind::Plunge { to },
        MotionKind::Cut { .. } => MotionKind::Cut { to },
        MotionKind::Retract { .. } => MotionKind::Retract { to },
        MotionKind::LeadIn { .. } => MotionKind::LeadIn { to },
        MotionKind::LeadOut { .. } => MotionKind::LeadOut { to },
        MotionKind::Link { .. } => MotionKind::Link { to },
        other => other.clone(),
    }
}

impl<'a> State<'a> {
    fn push(&mut self, source: Option<usize>, op: &OperationId, motion: Motion, linear: Point3, center: Option<Point3>, feed_words: (Option<f64>, Option<f64>), repositioned: bool) {
        let rotary = words(self.chain, &self.angles);
        self.out.push(ResolvedMotion {
            source,
            operation: op.clone(),
            motion,
            linear,
            center,
            rotary,
            inverse_time: feed_words.0,
            rotary_feed: feed_words.1,
            repositioned,
        });
    }

    fn pass_through(&mut self, source: usize, op: &OperationId, m: &Motion) {
        let linear = match (self.tip, &self.work_plane) {
            (Some(t), Some(wp)) => wp.apply_point(t),
            (Some(t), None) => self.chain.linear_for(t, &self.angles, &self.opts.kinematics),
            (None, _) => Point3::ZERO,
        };
        self.push(Some(source), op, m.clone(), linear, None, (None, None), false);
    }

    /// Feed words of a block from `t0` to `t1` whose rotaries move `travel_deg`.
    fn feed_words(&self, feed: Feed, t0: Option<Point3>, t1: Point3, travel_deg: f64) -> (Option<f64>, Option<f64>) {
        let Some(rate) = feed.value() else { return (None, None) };
        if travel_deg <= 1e-9 || self.opts.feed_mode == FeedMode::Standard {
            return (None, None);
        }
        let chord = t0.map_or(0.0, |p| p.distance(t1));
        let minutes = (chord / rate.max(1e-9)).max(travel_deg / self.opts.rotary_max_deg_per_min);
        match self.opts.feed_mode {
            FeedMode::InverseTime => (Some(1.0 / minutes.max(1e-12)), None),
            FeedMode::DegreesPerMinute => (None, Some(travel_deg / minutes.max(1e-12))),
            FeedMode::Standard => (None, None),
        }
    }

    /// Largest tip deviation of joint-interpolated motion between two poses
    /// from the straight chord between their tips (TCP off only).
    fn deviation(&self, p0: &Pose, p1: &Pose, t0: Point3, t1: Point3) -> f64 {
        if self.opts.kinematics.tcp {
            return 0.0;
        }
        let mut worst = 0.0f64;
        for f in [0.25, 0.5, 0.75] {
            let angles: Vec<f64> = p0.angles_deg.iter().zip(&p1.angles_deg).map(|(a, b)| a + (b - a) * f).collect();
            let pose = Pose { linear: p0.linear.lerp(p1.linear, f), angles_deg: angles, branch: p1.branch, singular: false };
            if let Ok((tip, _)) = self.chain.forward(&pose, &self.opts.kinematics) {
                worst = worst.max(tip.distance(t0.lerp(t1, f)));
            }
        }
        worst
    }

    /// One moving motion outside a work plane.
    fn emit_move(&mut self, source: usize, op: &OperationId, m: &Motion, to: Point3, axis: Point3) -> Result<(), KinematicsError> {
        let kin = &self.opts.kinematics;
        let (pose, travel) = choose_pose(self.chain, to, Some(axis), Some(&self.angles), source, kin)?;
        let is_feed = !m.feed.is_rapid();
        // The nearest solution lies on the other branch (or across a
        // singularity): the rotaries cannot sweep that far while cutting.
        if is_feed && self.tip.is_some() && travel > self.opts.reposition_travel_deg {
            self.reposition(source, op, m, to, axis, &pose)?;
            return Ok(());
        }
        // Subdivide: by rotary travel, then (TCP off) by tip deviation. The
        // first move of a stream has no start to interpolate from.
        let steps = if self.tip.is_some() { ((travel / self.opts.max_segment_deg - 1e-9).ceil() as usize).max(1) } else { 1 };
        let from_tip = self.tip;
        let from_axis = self.axis;
        let mut pieces: Vec<(Point3, Point3)> = Vec::with_capacity(steps);
        for s in 1..=steps {
            let f = s as f64 / steps as f64;
            let tip = from_tip.map_or(to, |p| p.lerp(to, f));
            let ax = slerp(from_axis, axis, f);
            pieces.push((tip, ax));
        }
        // Refine each piece by deviation (TCP off), depth-limited.
        let mut queue: Vec<(Point3, Point3, u32)> = Vec::new();
        let mut prev_tip = from_tip;
        let mut prev_axis = from_axis;
        let mut emitted = 0usize;
        for (tip, ax) in pieces {
            queue.push((tip, ax, 8));
            while let Some((t, a, depth)) = queue.pop() {
                let (p1, travel) = choose_pose(self.chain, t, Some(a), Some(&self.angles), source, kin)?;
                let p0 = Pose { linear: self.chain.linear_for(prev_tip.unwrap_or(t), &self.angles, kin), angles_deg: self.angles.clone(), branch: p1.branch, singular: false };
                let dev = match prev_tip {
                    Some(pt) if depth > 0 => self.deviation(&p0, &p1, pt, t),
                    _ => 0.0,
                };
                if dev > self.opts.linearize_tolerance {
                    // Split: push the far half first so the near half is popped next.
                    let mid_t = prev_tip.unwrap().lerp(t, 0.5);
                    let mid_a = slerp(prev_axis, a, 0.5);
                    queue.push((t, a, depth - 1));
                    queue.push((mid_t, mid_a, depth - 1));
                    continue;
                }
                let feed_words = self.feed_words(m.feed, prev_tip, t, travel);
                let mut motion = m.clone();
                motion.kind = with_end(&m.kind, t);
                motion.tool_axis = Some(a);
                self.angles = p1.angles_deg.clone();
                self.push(Some(source), op, motion, p1.linear, None, feed_words, false);
                emitted += 1;
                prev_tip = Some(t);
                prev_axis = a;
            }
        }
        self.subdivided += emitted.saturating_sub(1);
        self.tip = Some(to);
        self.axis = axis;
        Ok(())
    }

    /// Retract along the current axis, rotate to the new branch at the
    /// retracted distance, plunge back to the target.
    fn reposition(&mut self, source: usize, op: &OperationId, m: &Motion, to: Point3, axis: Point3, target: &Pose) -> Result<(), KinematicsError> {
        let kin = &self.opts.kinematics;
        let d = self.opts.reposition_distance;
        let cur_tip = self.tip.unwrap();
        let up = cur_tip.add(self.axis.scale(d));
        let mut retract = Motion::retract(up, Feed::RAPID);
        retract.tool_axis = Some(self.axis);
        retract.path_id = m.path_id;
        let linear = self.chain.linear_for(up, &self.angles, kin);
        self.push(None, op, retract, linear, None, (None, None), true);
        // Rotate at the retracted distance above the target.
        let over = to.add(axis.scale(d));
        let (pose, _) = choose_pose(self.chain, over, Some(axis), Some(&target.angles_deg), source, kin)?;
        let mut rotate = Motion::rapid(over);
        rotate.tool_axis = Some(axis);
        rotate.path_id = m.path_id;
        self.angles = pose.angles_deg.clone();
        self.push(None, op, rotate, pose.linear, None, (None, None), true);
        // Plunge back along the new axis at the motion's feed.
        let mut plunge = Motion::new(MotionKind::Plunge { to }, m.feed);
        plunge.tool_axis = Some(axis);
        plunge.path_id = m.path_id;
        let linear = self.chain.linear_for(to, &self.angles, kin);
        self.push(Some(source), op, plunge, linear, None, (None, None), true);
        self.repositions += 1;
        self.tip = Some(to);
        self.axis = axis;
        Ok(())
    }
}

/// Resolve a motion stream. Sections are not computed; see [`resolve_program`].
pub fn resolve_motions(motions: &[ProgramMotion], chain: &Chain, opts: &ResolveOptions) -> Result<ResolvedProgram, KinematicsError> {
    opts.validate(chain)?;
    let kin = &opts.kinematics;
    let mut st = State {
        chain,
        opts,
        angles: opts.start_angles.clone().unwrap_or_else(|| vec![0.0; chain.joints.len()]),
        tip: None,
        axis: Point3::Z,
        work_plane: None,
        tcp_announced: false,
        out: Vec::with_capacity(motions.len()),
        repositions: 0,
        subdivided: 0,
        warnings: Vec::new(),
    };
    let mut linearised = 0usize;
    for (i, pm) in motions.iter().enumerate() {
        let op = &pm.operation;
        // An arc under a rotated table, with neither TCP nor a work plane,
        // cannot be one block: the machine interpolates the joints, not the
        // part-frame circle. It is cut as chords within the linearisation
        // tolerance (what a post without arcs does), each a linear move.
        let table_rotated = chain.joints.iter().zip(&st.angles).any(|(j, a)| j.carrier == super::Carrier::Table && a.abs() > 1e-9);
        let expanded: Vec<Motion> = match (&pm.motion.kind, st.tip) {
            (MotionKind::ArcCw { to, center, plane, .. } | MotionKind::ArcCcw { to, center, plane, .. }, Some(from)) if !kin.tcp && table_rotated && st.work_plane.is_none() => {
                let cw = matches!(pm.motion.kind, MotionKind::ArcCw { .. });
                let (sweep, radius) = crate::arcfit::arc_sweep(from, *to, *center, *plane, cw).ok_or_else(|| KinematicsError::InvalidInput(format!("motion {i}: arc of zero radius")))?;
                let tol = opts.linearize_tolerance.max(1e-6);
                let max_step = if tol >= radius { std::f64::consts::PI } else { 2.0 * (1.0 - tol / radius).acos() };
                let n = ((sweep / max_step.max(1e-6)).ceil() as usize).clamp(1, 100_000);
                linearised += 1;
                (1..=n)
                    .map(|k| {
                        let q = if k == n { *to } else { crate::arcfit::arc_point(from, &pm.motion.kind, k as f64 / n as f64).unwrap_or(*to) };
                        Motion { kind: MotionKind::Cut { to: q }, ..pm.motion.clone() }
                    })
                    .collect()
            }
            _ => vec![pm.motion.clone()],
        };
        for m in &expanded {
        match &m.kind {
            MotionKind::WorkPlane { origin, x_axis, z_axis } => {
                let z = z_axis.normalized().ok_or(KinematicsError::InvalidAxis)?;
                let at = st.tip.unwrap_or(*origin);
                let (pose, _) = choose_pose(chain, at, Some(z), Some(&st.angles), i, kin)?;
                // The tool does not move in machine coordinates while the
                // rotaries position; the setup-frame tip follows.
                let machine_linear = st.tip.map_or(pose.linear, |t| chain.linear_for(t, &st.angles, kin));
                st.angles = pose.angles_deg.clone();
                if !kin.tcp {
                    let probe = Pose { linear: machine_linear, angles_deg: st.angles.clone(), branch: pose.branch, singular: false };
                    if let Ok((tip, _)) = chain.forward(&probe, kin) {
                        st.tip = Some(tip);
                    }
                }
                st.axis = z;
                st.push(Some(i), op, m.clone(), machine_linear, None, (None, None), false);
                let rp = Motion::new(MotionKind::RotaryPosition { axes: words(chain, &st.angles) }, Feed::RAPID);
                st.push(None, op, rp, machine_linear, None, (None, None), false);
                st.work_plane = Some(RigidTransform::from_frame(*origin, *x_axis, z).map_err(|e| KinematicsError::InvalidInput(format!("motion {i}: work plane: {e}")))?);
            }
            MotionKind::WorkPlaneOff => {
                st.pass_through(i, op, m);
                st.work_plane = None;
            }
            MotionKind::ArcCw { to, center, .. } | MotionKind::ArcCcw { to, center, .. } => {
                let axis = m.tool_axis.unwrap_or(Point3::Z).normalized().ok_or(KinematicsError::InvalidAxis)?;
                if axis.distance(st.axis) > 1e-9 && st.tip.is_some() {
                    return Err(KinematicsError::InvalidInput(format!("motion {i}: an arc cannot change the tool axis; linearise it")));
                }
                if let Some(wp) = &st.work_plane {
                    let (linear, c) = (wp.apply_point(*to), wp.apply_point(*center));
                    st.tip = Some(*to);
                    st.push(Some(i), op, m.clone(), linear, Some(c), (None, None), false);
                    continue;
                }
                let table_rotated = chain.joints.iter().zip(&st.angles).any(|(j, a)| j.carrier == super::Carrier::Table && a.abs() > 1e-9);
                if !kin.tcp && table_rotated {
                    return Err(KinematicsError::InvalidInput(format!("motion {i}: an arc under a rotated table needs TCP or a work plane")));
                }
                let (pose, _) = choose_pose(chain, *to, Some(axis), Some(&st.angles), i, kin)?;
                let c = chain.linear_for(*center, &pose.angles_deg, kin);
                st.angles = pose.angles_deg.clone();
                st.tip = Some(*to);
                st.axis = axis;
                st.push(Some(i), op, m.clone(), pose.linear, Some(c), (None, None), false);
            }
            MotionKind::Cycle { at, .. } => {
                let axis = m.tool_axis.unwrap_or(Point3::Z).normalized().ok_or(KinematicsError::InvalidAxis)?;
                if let Some(wp) = &st.work_plane {
                    st.tip = Some(*at);
                    let linear = wp.apply_point(*at);
                    st.push(Some(i), op, m.clone(), linear, None, (None, None), false);
                    continue;
                }
                let (pose, _) = choose_pose(chain, *at, Some(axis), Some(&st.angles), i, kin)?;
                st.angles = pose.angles_deg.clone();
                st.tip = Some(*at);
                st.axis = axis;
                st.push(Some(i), op, m.clone(), pose.linear, None, (None, None), false);
            }
            other => match other.end() {
                None => st.pass_through(i, op, m),
                Some(to) => {
                    let axis = m.tool_axis.unwrap_or(Point3::Z).normalized().ok_or(KinematicsError::InvalidAxis)?;
                    if let Some(wp) = &st.work_plane {
                        let linear = wp.apply_point(to);
                        st.tip = Some(to);
                        st.axis = axis;
                        st.push(Some(i), op, m.clone(), linear, None, (None, None), false);
                        continue;
                    }
                    if kin.tcp && !st.tcp_announced && !chain.joints.is_empty() && 1.0 - axis.dot(Point3::Z) > kin.cos_eps {
                        let tcp = Motion::new(MotionKind::TcpMode { enabled: true }, Feed::RAPID);
                        let linear = st.tip.map_or(Point3::ZERO, |t| chain.linear_for(t, &st.angles, kin));
                        st.push(None, op, tcp, linear, None, (None, None), false);
                        st.tcp_announced = true;
                    }
                    st.emit_move(i, op, m, to, axis)?;
                }
            },
        }
        }
    }
    if linearised > 0 {
        st.warnings.push(Diagnostic::new("arc-linearised", format!("{linearised} arcs under a rotated table (no TCP, no work plane) are cut as chords within {} of the arc", opts.linearize_tolerance)));
    }
    if st.tcp_announced {
        let op = motions.last().map(|m| m.operation.clone()).unwrap_or_default();
        let linear = st.tip.map_or(Point3::ZERO, |t| chain.linear_for(t, &st.angles, kin));
        st.push(None, &op, Motion::new(MotionKind::TcpMode { enabled: false }, Feed::RAPID), linear, None, (None, None), false);
    }
    if st.repositions > 0 {
        st.warnings.push(Diagnostic::new("rotary-repositioned", format!("{} branch changes resolved by retract, rotate and plunge", st.repositions)));
    }
    Ok(ResolvedProgram {
        joints: chain.joints.iter().map(|j| j.name.clone()).collect(),
        motions: st.out,
        sections: Vec::new(),
        warnings: st.warnings,
        repositions: st.repositions,
        subdivided: st.subdivided,
    })
}

/// Resolve a whole program; `sections` maps each program section to its
/// resolved block range.
pub fn resolve_program(program: &Program, chain: &Chain, opts: &ResolveOptions) -> Result<ResolvedProgram, KinematicsError> {
    let mut resolved = resolve_motions(&program.motions, chain, opts)?;
    let mut sections = Vec::with_capacity(program.sections.len());
    for s in &program.sections {
        let lo = s.first_motion;
        let hi = s.first_motion + s.motion_count;
        let mut first: Option<usize> = None;
        let mut last: Option<usize> = None;
        for (k, r) in resolved.motions.iter().enumerate() {
            // Inserted blocks belong to the section of the next sourced block.
            let src = r.source.or_else(|| resolved.motions[k..].iter().find_map(|q| q.source));
            if let Some(src) = src {
                if src >= lo && src < hi {
                    first.get_or_insert(k);
                    last = Some(k);
                }
            }
        }
        match (first, last) {
            (Some(f), Some(l)) => sections.push([f, l - f + 1]),
            _ => sections.push([resolved.motions.len(), 0]),
        }
    }
    resolved.sections = sections;
    Ok(resolved)
}
