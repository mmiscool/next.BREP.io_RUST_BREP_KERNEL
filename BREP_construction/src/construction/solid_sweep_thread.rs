//! A screw thread swept by a generating motion: what a hob (a worm) cuts from a gear's blank
//! as the two turn in mesh.
//!
//! The tool is a thread wound on a pitch surface of revolution whose radius may change along the
//! axis (a cylinder for a worm, a cone for a tapered worm). In any plane through its axis its
//! section is a row of trapezoids one lead apart, each `2·half_width` wide along the axis at the
//! pitch radius, its flanks leaning by `flank_tan` (the axial width shrinks by `2·flank_tan` per
//! unit of radius outward), from `root` below the pitch radius to `tip` above it. Its two end
//! planes square to the axis bound it.
//!
//! The motion is [`crate::TwoTurns`] as a case states it: the tool turned about its own axis and
//! then the result turned about the gear's. A point x of the gear is in the tool at t where
//! x carried back by the motion lies in the thread ([`ThreadTool::margin`]), and in the sweep
//! where that holds at some t ([`swept_margin`]). The tool's turn about its own axis keeps every
//! point's distance from that axis, so the times a point can be in the tool are found from the
//! gear's turn alone ([`window`]).

use crate::{TwoTurns, Vec3};
use std::f64::consts::{PI, TAU};

/// A screw thread on a pitch surface of revolution, between two end planes square to its axis.
#[derive(Clone, Copy, Debug)]
pub struct ThreadTool {
    /// A point of the axis, where the axial coordinate s is 0.
    pub axis_point: Vec3,
    /// The axis's direction (unit).
    pub axis_direction: Vec3,
    /// The direction square to the axis where a point's angle about the axis is 0; angles run
    /// right-handed about `axis_direction`.
    pub reference: Vec3,
    /// The thread's advance along the axis per turn, positive for a right-hand thread.
    pub lead: f64,
    /// The pitch radius at s = 0, and its change per unit of s (0 for a cylindrical worm).
    pub pitch_radius: f64,
    pub taper: f64,
    /// Half the thread's axial width at the pitch radius.
    pub half_width: f64,
    /// The flanks' lean: the axial half-width falls by this much per unit of radius outward.
    pub flank_tan: f64,
    /// How far the tip stands above, and the root lies below, the pitch radius.
    pub tip: f64,
    pub root: f64,
    /// The end planes, as axial coordinates.
    pub start: f64,
    pub end: f64,
}

/// The labels of the thread's bounds, as bits of [`ThreadTool::margin`]'s binding set: the two
/// flanks, the tip and root surfaces, and the two end planes.
pub const LEADING_FLANK: u32 = 1 << 0;
pub const TRAILING_FLANK: u32 = 1 << 1;
pub const TIP: u32 = 1 << 2;
pub const ROOT: u32 = 1 << 3;
pub const START: u32 = 1 << 4;
pub const END: u32 = 1 << 5;

/// Bounds within this of the least are taken to bind together.
const TIGHT: f64 = 1e-7;

impl ThreadTool {
    /// The least slack of the thread's inequalities at the tool-frame point `y` (positive
    /// inside, as a length), and which of them bind there.
    pub fn margin(&self, y: Vec3) -> (f64, u32) {
        let d = y.sub(self.axis_point);
        let s = d.dot(self.axis_direction);
        let radial = d.sub(self.axis_direction.scale(s));
        let r = radial.length();
        let across = self.axis_direction.cross(self.reference);
        let theta = radial.dot(across).atan2(radial.dot(self.reference));
        let pitch = self.pitch_radius + self.taper * s;
        // The axial offset from the nearest thread centre: the centre at angle θ lies at
        // s = lead·θ/2π (mod lead).
        let along = s - self.lead * theta / TAU;
        let offset = along - self.lead.abs() * (along / self.lead.abs()).round();
        let half = self.half_width - (r - pitch) * self.flank_tan;
        let slack = [half - offset, half + offset, pitch + self.tip - r, r - (pitch - self.root), s - self.start, self.end - s];
        let least = slack.iter().copied().fold(f64::INFINITY, f64::min);
        let bind = slack.iter().enumerate().filter(|(_, &v)| v - least <= TIGHT).fold(0u32, |m, (i, _)| m | 1 << i);
        (least, bind)
    }

    /// Whether the thread can be read by sampling: its width a positive fraction of its lead (at
    /// least 1e-5 of a turn), its section of positive depth, its ends in order; `Err` naming
    /// what is not. The cases' worms are a quarter of a turn wide.
    pub fn check(&self) -> Result<(), String> {
        let fraction = 2.0 * self.half_width / self.lead.abs();
        if !(self.lead.is_finite() && self.lead != 0.0 && fraction >= 1e-5) {
            return Err(format!("the thread's width is {fraction:e} of its lead, under the 1e-5 a sweep can resolve"));
        }
        if !(self.tip + self.root > 0.0 && self.end > self.start && self.pitch_radius > 0.0) {
            return Err("the thread's section or its ends are empty".into());
        }
        // Near its axis a point's angle about it turns without bound, so the thread must keep
        // clear of the axis (a hundredth of its depth): its angle then changes at a rate sampling
        // can bound.
        if !(self.inner_radius() >= 0.01 * (self.tip + self.root)) {
            return Err(format!("the thread comes within {:e} of its axis, under a hundredth of its depth", self.inner_radius()));
        }
        Ok(())
    }

    /// The least distance of the thread from its axis: its root under the pitch radius at the end
    /// where the taper brings it nearest.
    pub fn inner_radius(&self) -> f64 {
        (self.pitch_radius + self.taper * self.start).min(self.pitch_radius + self.taper * self.end) - self.root
    }

    /// The farthest the thread reaches from its axis.
    pub fn reach(&self) -> f64 {
        let at = |s: f64| self.pitch_radius + self.taper * s + self.tip;
        at(self.start).max(at(self.end))
    }
}

/// `x` carried back by the motion at `t`: the point of the tool that stands at `x` then.
pub fn carried_back(motion: &TwoTurns, x: Vec3, t: f64) -> Vec3 {
    let back = |p: Vec3, (point, direction): (Vec3, Vec3), angle: f64| point.add(p.sub(point).rotate_about(direction, angle));
    let unit = |d: Vec3| d.normalized().unwrap_or(d);
    let y = back(x, (motion.axes[1].0, unit(motion.axes[1].1)), -motion.rates[1] * t);
    back(y, (motion.axes[0].0, unit(motion.axes[0].1)), -motion.rates[0] * t)
}

/// The thread's margin at the gear point `x` at time `t`.
pub fn margin_at(tool: &ThreadTool, motion: &TwoTurns, x: Vec3, t: f64) -> (f64, u32) {
    tool.margin(carried_back(motion, x, t))
}

/// Samples of the gear's turn in [`window`].
const WINDOW_SAMPLES: usize = 720;

/// The stretches of the motion's time in which `x` comes within the thread's reach of its axis
/// (and [`WINDOW_BAND`]):
/// the tool's own turn keeps that distance, so it depends on the gear's turn alone. Each is
/// widened by a sample either way. Empty where `x` never comes within reach.
pub fn window(tool: &ThreadTool, motion: &TwoTurns, x: Vec3) -> Vec<(f64, f64)> {
    let rate = motion.rates[1];
    let (from, to) = (motion.from, motion.to);
    if rate == 0.0 || !(to > from) {
        return if within(tool, motion, x, from) { vec![(from, to)] } else { Vec::new() };
    }
    // One sample per 1/720 of a gear turn, finer where the point stands far from the gear's axis
    // (so it moves no more than a quarter of the reach between samples), or over a short motion.
    let (point, direction) = (motion.axes[1].0, motion.axes[1].1.normalized().unwrap_or(motion.axes[1].1));
    let off = x.sub(point);
    let rho = off.sub(direction.scale(off.dot(direction))).length();
    // The steps are set by the thread's depth, not its reach, so a shallow pass of a wide tool is
    // not stepped over.
    let depth = tool.tip + tool.root + WINDOW_BAND;
    let angle = (TAU / WINDOW_SAMPLES as f64).min(0.25 * depth / rho.max(1e-12));
    let step = (angle / rate.abs()).min((to - from) / 8.0);
    let n = ((to - from) / step).ceil() as usize;
    let at = |i: usize| (from + step * i as f64).min(to);
    let mut out: Vec<(f64, f64)> = Vec::new();
    let mut open: Option<f64> = None;
    for i in 0..=n {
        let inside = within(tool, motion, x, at(i));
        match (inside, open) {
            (true, None) => open = Some(at(i.saturating_sub(1))),
            (false, Some(start)) => {
                out.push((start, at(i)));
                open = None;
            }
            _ => {}
        }
    }
    if let Some(start) = open {
        out.push((start, to));
    }
    out
}

/// How far past the thread's reach a window still reads: the margin is exact there for points
/// that far outside the sweep, and only its sign counts beyond.
pub const WINDOW_BAND: f64 = 1.0;

/// Whether `x` at `t` lies within the thread's reach of its axis, and [`WINDOW_BAND`].
fn within(tool: &ThreadTool, motion: &TwoTurns, x: Vec3, t: f64) -> bool {
    let y = carried_back(motion, x, t);
    let d = y.sub(tool.axis_point);
    let s = d.dot(tool.axis_direction);
    let reach = tool.reach() + WINDOW_BAND;
    d.sub(tool.axis_direction.scale(s)).length() <= reach && s >= tool.start - reach && s <= tool.end + reach
}

/// The time step that resolves the thread within a window: an eighth of the time the tool's own
/// turn takes to carry the thread's width past a point (a fortieth of a turn at most), or of the
/// gear's turn through that width.
fn phase_step(tool: &ThreadTool, motion: &TwoTurns) -> f64 {
    let width = (2.0 * tool.half_width / tool.lead.abs()).min(1.0);
    let own = (TAU / motion.rates[0].abs().max(1e-12)) * (width / 8.0).min(1.0 / 40.0) * 40.0;
    let across = (2.0 * tool.half_width / tool.reach().max(1e-12)) / motion.rates[1].abs().max(1e-12);
    own.min(across) / 40.0
}

/// How fast the margin at the gear point `x` can change in time, and the time step that keeps that
/// change within an eighth of the thread's least extent (its width or its depth), no longer than
/// [`phase_step`]. In the tool's frame the point moves at most `ρ·|ω₁|` (ρ its distance from the
/// gear's axis), so its height along the tool's axis and its radius change no faster; its angle
/// about the tool's axis changes at most `|ω₀| + ρ·|ω₁| / r_in` while it is in the thread, at least
/// `r_in` from the axis. Each slack is a sum of those with coefficients 1, the lead over a turn,
/// the flanks' slope and the taper, and their minimum changes no faster than the largest.
fn time_scale(tool: &ThreadTool, motion: &TwoTurns, x: Vec3) -> (f64, f64) {
    let (point, axis) = (motion.axes[1].0, motion.axes[1].1.normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0)));
    let rel = x.sub(point);
    let rho = rel.sub(axis.scale(rel.dot(axis))).length();
    let speed = rho * motion.rates[1].abs();
    let inner = tool.inner_radius();
    let turning = motion.rates[0].abs() + speed / inner;
    let rate = speed * (1.0 + tool.flank_tan.abs() + tool.taper.abs()) + tool.lead.abs() / TAU * turning;
    let extent = (2.0 * tool.half_width).min(tool.tip + tool.root);
    let step = phase_step(tool, motion).min(0.125 * extent / rate.max(1e-300));
    (step, rate)
}

/// The most samples one window takes.
const MAX_SAMPLES: usize = 2_000_000;

/// The best margin over the motion at the gear point `x` (positive where the sweep holds it) and
/// what binds there: each window sampled at [`phase_step`], the three best samples refined by
/// golden sections over their neighbours. Exact for points within [`WINDOW_BAND`] of the sweep;
/// `(-1.0, 0)` where `x` never comes within reach, and negative (of no set size) anywhere
/// farther out. The tool must pass [`ThreadTool::check`].
pub fn swept_margin(tool: &ThreadTool, motion: &TwoTurns, x: Vec3) -> (f64, u32) {
    let (step, _) = time_scale(tool, motion, x);
    let mut top = (-1.0, 0u32);
    let mut first = true;
    for (a, b) in window(tool, motion, x) {
        // A thread `check` admits never needs more; the cap only bounds one it refuses.
        let n = ((b - a) / step).ceil().clamp(2.0, MAX_SAMPLES as f64) as usize;
        let samples: Vec<f64> = (0..=n).map(|i| margin_at(tool, motion, x, a + (b - a) * i as f64 / n as f64).0).collect();
        let mut order: Vec<usize> = (0..=n).collect();
        order.sort_by(|&i, &j| samples[j].total_cmp(&samples[i]));
        for &i in order.iter().take(3) {
            // The sample itself stands, whatever its refinement finds.
            let sampled = margin_at(tool, motion, x, a + (b - a) * i as f64 / n as f64);
            if first || sampled.0 > top.0 {
                top = sampled;
                first = false;
            }
            let (mut l, mut h) = (a + (b - a) * i.saturating_sub(1) as f64 / n as f64, a + (b - a) * (i + 1).min(n) as f64 / n as f64);
            let ratio = 0.5 * (5f64.sqrt() - 1.0);
            let (mut c, mut d) = (h - ratio * (h - l), l + ratio * (h - l));
            let (mut fc, mut fd) = (margin_at(tool, motion, x, c).0, margin_at(tool, motion, x, d).0);
            for _ in 0..48 {
                if fc > fd {
                    h = d;
                    (d, fd) = (c, fc);
                    c = h - ratio * (h - l);
                    fc = margin_at(tool, motion, x, c).0;
                } else {
                    l = c;
                    (c, fc) = (d, fd);
                    d = l + ratio * (h - l);
                    fd = margin_at(tool, motion, x, d).0;
                }
            }
            let found = margin_at(tool, motion, x, 0.5 * (l + h));
            if first || found.0 > top.0 {
                top = found;
                first = false;
            }
        }
    }
    top
}

/// Whether the sweep holds `x` (the sign of [`swept_margin`]), read with less work: true at the
/// first positive sample; a sample is refined only where it could rise above zero between its
/// neighbours, by the bound on the margin's rate of change ([`time_scale`]).
pub fn swept_holds(tool: &ThreadTool, motion: &TwoTurns, x: Vec3) -> bool {
    let (step, rate) = time_scale(tool, motion, x);
    for (a, b) in window(tool, motion, x) {
        let n = ((b - a) / step).ceil().clamp(2.0, MAX_SAMPLES as f64) as usize;
        let time = |i: usize| a + (b - a) * i as f64 / n as f64;
        let mut samples = Vec::with_capacity(n + 1);
        for i in 0..=n {
            let m = margin_at(tool, motion, x, time(i)).0;
            if m > 0.0 {
                return true;
            }
            samples.push(m);
        }
        // Between neighbouring samples the margin rises at most `rate` times half their spacing
        // above the higher of them.
        let reach = rate * (b - a) / n as f64;
        let mut order: Vec<usize> = (0..=n).filter(|&i| samples[i] + reach >= 0.0).collect();
        order.sort_by(|&i, &j| samples[j].total_cmp(&samples[i]));
        for &i in order.iter().take(3) {
            let (mut l, mut h) = (time(i.saturating_sub(1)), time((i + 1).min(n)));
            let ratio = 0.5 * (5f64.sqrt() - 1.0);
            let (mut c, mut d) = (h - ratio * (h - l), l + ratio * (h - l));
            let (mut fc, mut fd) = (margin_at(tool, motion, x, c).0, margin_at(tool, motion, x, d).0);
            for _ in 0..48 {
                if fc > 0.0 || fd > 0.0 {
                    return true;
                }
                if fc > fd {
                    h = d;
                    (d, fd) = (c, fc);
                    c = h - ratio * (h - l);
                    fc = margin_at(tool, motion, x, c).0;
                } else {
                    l = c;
                    (c, fc) = (d, fd);
                    d = l + ratio * (h - l);
                    fd = margin_at(tool, motion, x, d).0;
                }
            }
            if fc > 0.0 || fd > 0.0 || margin_at(tool, motion, x, 0.5 * (l + h)).0 > 0.0 {
                return true;
            }
        }
    }
    false
}

/// The same reading with no window and no refinement: every sample of the whole motion at
/// `samples` steps. The check mode's oracle for [`swept_margin`] (it shares only the tool's
/// own margin with it).
pub fn swept_margin_brute(tool: &ThreadTool, motion: &TwoTurns, x: Vec3, samples: usize) -> f64 {
    (0..=samples)
        .map(|i| margin_at(tool, motion, x, motion.from + (motion.to - motion.from) * i as f64 / samples as f64).0)
        .fold(f64::NEG_INFINITY, f64::max)
}

/// A gear's blank: the ring about the gear's axis (the motion's second axis) between radii
/// `inner` and `outer`, from `bottom` up to its face, which stands at `face` at the inner edge and
/// `face_outer` at the outer (heights along that axis): flat where they are equal, a cone else.
#[derive(Clone, Copy, Debug)]
pub struct RingBlank {
    pub inner: f64,
    pub outer: f64,
    pub bottom: f64,
    pub face: f64,
    pub face_outer: f64,
}

impl RingBlank {
    /// The face's height at radius `r` (the cone run on past the ring's edges).
    pub fn face_at(&self, r: f64) -> f64 {
        self.face + (self.face_outer - self.face) * (r - self.inner) / (self.outer - self.inner)
    }
}

/// How a gear's spaces are sliced: on cylinders about its axis (a face gear, its teeth across its
/// face) or on planes square to it (a wheel, its teeth across its rim).
#[derive(Clone, Copy, Debug, PartialEq)]
enum Family {
    Cylinders,
    Planes,
}

/// The gear axis's frame: its point, its direction and two directions square to it.
fn gear_frame(motion: &TwoTurns) -> (Vec3, Vec3, Vec3, Vec3) {
    let (point, d) = (motion.axes[1].0, motion.axes[1].1.normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0)));
    let seed = if d.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let e1 = seed.sub(d.scale(seed.dot(d))).normalized().unwrap_or(seed);
    (point, d, e1, d.cross(e1))
}

/// How far above the face the air over a space is counted in (never on the face's plane), as a
/// share of the thread's depth (tip + root): 0.15 for the cases' worms, 5 deep.
pub const AIR_STEP: f64 = 0.03;

/// One cylindrical slice of a tooth space at radius `r` about the gear's axis, as `count` points
/// evenly spaced by length round its closed boundary: the cut below the face, and the air above
/// it over the space (from `AIR_STEP` of the thread's depth above the face), between the middles
/// of the tooth tops
/// either side. `seed`: an angle near the space. Returns the points and the space's middle angle.
fn slice(tool: &ThreadTool, motion: &TwoTurns, blank: &RingBlank, family: Family, pitch: f64, r: f64, seed: f64, fallback: Option<(f64, f64)>, count: usize) -> Result<(Vec<Vec3>, f64, f64), String> {
    let (o, d, e1, e2) = gear_frame(motion);
    // A slice's point at angle `f` and depth coordinate `z`: on a cylinder of radius `r`, `z` is
    // the height; on the plane at height `r`, the distance from the axis (the rim its face).
    let place = |f: f64, z: f64| match family {
        Family::Cylinders => o.add(e1.scale(r * f.cos())).add(e2.scale(r * f.sin())).add(d.scale(z)),
        Family::Planes => o.add(e1.scale(z * f.cos())).add(e2.scale(z * f.sin())).add(d.scale(r)),
    };
    let cut = |f: f64, z: f64| swept_holds(tool, motion, place(f, z));
    let (face, floor, weight) = match family {
        Family::Cylinders => (blank.face_at(r), blank.bottom, r),
        Family::Planes => (blank.outer, blank.inner, blank.outer),
    };
    // The space's run nearest the seed, and the tooth tops either side, read just under the face;
    // where the teeth come to a point below it (the whole circle cut there), lower down, at the
    // tips' level.
    // A pitch and a half either way, so the tooth tops either side of the space nearest the
    // seed lie within the search even where the spaces are wide and the tops narrow (at a rim).
    let n = 720;
    let (lo, hi) = (seed - 1.5 * pitch, seed + 1.5 * pitch);
    let at = |k: usize| lo + (hi - lo) * k as f64 / n as f64;
    // Offsets are shares of the thread's depth, so the trace is the same at any scale.
    let depth = tool.tip + tool.root;
    let mut level = face - 0.004 * depth;
    let mut cuts: Vec<bool> = (0..=n).map(|k| cut(at(k), level)).collect();
    while cuts.iter().all(|&c| c) && level > floor {
        level -= 0.02 * depth;
        cuts = (0..=n).map(|k| cut(at(k), level)).collect();
    }
    let mid = n / 2;
    let distance = |a: usize, b: usize| if mid < a { a - mid } else if mid > b { mid - b } else { 0 };
    let (mut best, mut k): (Option<(usize, usize)>, usize) = (None, 0);
    while k <= n {
        if cuts[k] {
            let start = k;
            while k <= n && cuts[k] {
                k += 1;
            }
            let end = k - 1;
            if start > 0 && end < n && best.is_none_or(|(a, b)| distance(start, end) < distance(a, b)) {
                best = Some((start, end));
            }
        } else {
            k += 1;
        }
    }
    // A slice the tool never reaches (beyond the blank, past the tool's reach) is the air step's
    // floor alone, laid on its neighbour's middle and strips (`fallback`).
    let (centre, delta) = match (best, fallback) {
        (None, Some(fallback)) if cuts.iter().all(|&c| !c) => fallback,
        (best, _) => {
            let (start, end) = best.ok_or_else(|| format!("no tooth space on the face at r = {r}"))?;
            let (mut left, mut right) = (start, end);
            while left > 0 && !cuts[left - 1] {
                left -= 1;
            }
            while right < n && !cuts[right + 1] {
                right += 1;
            }
            if left == 0 || right == n {
                return Err(format!("the tooth tops beside the space at r = {r} reach past the search"));
            }
            let (ml, el, er, mr) = (0.5 * (at(left) + at(start - 1)), at(start), at(end), 0.5 * (at(end + 1) + at(right)));
            // The slice runs from half a pitch before the space's middle to half a pitch after it (the
            // tooth middles either side), and `delta` past each, so where the cut runs across a middle
            // (above the tips, where they come to a point below the face) a cell divider through it
            // crosses the space rather than meets one of its walls. The strips either side of a middle,
            // `delta` each way, must hold the bottom as a graph in angle: on a flat tooth top a share of
            // its width (past it the bottom steps down into the space); where the teeth come to a point
            // below the face, over the smooth tip, at least a fiftieth of a pitch.
            let lowered = level < face - 0.004 * depth;
            let delta = if lowered { (0.3 * (el - ml).min(mr - er)).max(0.02 * pitch) } else { 0.3 * (el - ml).min(mr - er) };
            let centre = 0.5 * (el + er);
            (centre, delta)
        }
    };
    let (ml, mr) = (centre - 0.5 * pitch - delta, centre + 0.5 * pitch + delta);
    // From just below the blank's bottom (all of the cut counted), but no lower than the tool
    // can reach (so a thick blank does not spread the grid's rows past the cut), to a band over
    // the air step. The tool lies within its reach of its axis segment, and the gear's turn keeps
    // heights along the gear's axis.
    let lowest = match family {
        Family::Cylinders => {
            let height = |s: f64| tool.axis_point.add(tool.axis_direction.scale(s)).sub(o).dot(d);
            height(tool.start).min(height(tool.end)) - tool.reach()
        }
        // The least distance of the tool's axis segment from the gear's axis, less the reach.
        Family::Planes => {
            let flat = |v: Vec3| v.sub(d.scale(v.dot(d)));
            let (a, u) = (flat(tool.axis_point.sub(o)), flat(tool.axis_direction));
            let s = if u.dot(u) > 0.0 { (-a.dot(u) / u.dot(u)).clamp(tool.start, tool.end) } else { tool.start };
            a.add(u.scale(s)).length() - tool.reach()
        }
    };
    let (z0, z1) = (floor.max(lowest) - 0.05 * depth, face + 0.2 * depth);
    let inside = |f: f64, z: f64| -> bool {
        if z >= z1 || f <= ml || f >= mr || z <= z0 {
            return false;
        }
        if z > face + AIR_STEP * depth {
            return true;
        }
        cut(f, z)
    };
    // The grid's cells, the outer boundary traced cell to cell (Moore), each boundary cell pushed
    // to the boundary by bisection towards its outside neighbours.
    let (nf, nz) = (220isize, 160isize);
    let fx = |i: isize| ml + (mr - ml) * (i as f64 + 0.5) / nf as f64;
    let zy = |j: isize| z0 + (z1 - z0) * (j as f64 + 0.5) / nz as f64;
    // Cells are read when the trace first asks for them (0 unread, 1 held, 2 not): it touches the
    // boundary's neighbourhood only, a small share of the grid.
    let grid: Vec<std::cell::Cell<u8>> = (0..nf * nz).map(|_| std::cell::Cell::new(0)).collect();
    let held = |i: isize, j: isize| {
        if i < 0 || j < 0 || i >= nf || j >= nz {
            return false;
        }
        let cell = &grid[(i * nz + j) as usize];
        if cell.get() == 0 {
            cell.set(if inside(fx(i), zy(j)) { 1 } else { 2 });
        }
        cell.get() == 1
    };
    let mut first = None;
    'find: for j in (0..nz).rev() {
        for i in 0..nf {
            if held(i, j) {
                first = Some((i, j));
                break 'find;
            }
        }
    }
    let first = first.ok_or_else(|| format!("the space at r = {r} is empty"))?;
    let dirs = [(1isize, 0isize), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)];
    let mut cells = vec![first];
    let (mut c, mut back) = (first, 4usize);
    for _ in 0..(4 * nf * nz) {
        let mut moved = false;
        for k in 1..=8 {
            let dir = (back + k) % 8;
            let q = (c.0 + dirs[dir].0, c.1 + dirs[dir].1);
            if held(q.0, q.1) {
                back = (dir + 4) % 8;
                c = q;
                moved = true;
                break;
            }
        }
        if !moved || c == first {
            break;
        }
        cells.push(c);
    }
    if cells.len() < 8 {
        return Err(format!("the space at r = {r} is too small to trace"));
    }
    let mut points: Vec<(f64, f64)> = Vec::with_capacity(cells.len());
    for &(i, j) in &cells {
        let (mut dx, mut dy) = (0.0, 0.0);
        for step in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            if !held(i + step.0, j + step.1) {
                dx += step.0 as f64;
                dy += step.1 as f64;
            }
        }
        let (cf, cz) = (fx(i), zy(j));
        // A cell a column (or row) thin, out on opposite sides, is pushed towards the frame where
        // it touches it, else towards its first outside neighbour.
        if dx == 0.0 && dy == 0.0 {
            let outside: Vec<(isize, isize)> = [(-1, 0), (1, 0), (0, 1), (0, -1)].into_iter().filter(|s| !held(i + s.0, j + s.1)).collect();
            let frame = outside.iter().find(|s| (s.0 == -1 && i == 0) || (s.0 == 1 && i == nf - 1) || (s.1 == 1 && j == nz - 1));
            if let Some(&(a, b)) = frame.or(outside.first()) {
                (dx, dy) = (a as f64, b as f64);
            }
        }
        let norm = (dx * dx + dy * dy).sqrt();
        if norm == 0.0 {
            points.push((cf, cz));
            continue;
        }
        let (sf, sz) = (dx / norm * (mr - ml) / nf as f64 * 1.5, dy / norm * (z1 - z0) / nz as f64 * 1.5);
        if inside(cf + sf, cz + sz) {
            points.push((cf + sf, cz + sz));
            continue;
        }
        // Eighteen halvings put the point within 1.5/2^18 of a cell of the boundary.
        let (mut a, mut b) = (0.0f64, 1.0f64);
        for _ in 0..18 {
            let m = 0.5 * (a + b);
            if inside(cf + sf * m, cz + sz * m) {
                a = m;
            } else {
                b = m;
            }
        }
        points.push((cf + sf * a, cz + sz * a));
    }
    let loop_points = periodic_loop(&points, weight, pitch, (ml, mr), delta, z1, (mr - ml) / nf as f64, (z1 - z0) / nz as f64, count)?;
    Ok((loop_points.into_iter().map(|(f, z)| place(f, z)).collect(), centre, delta))
}

/// A slice's traced boundary `points` (angle, height), in the window `lo..hi` (a pitch and twice
/// `delta` wide) under the top `z1`, made into `count` points: down the left wall, along the bottom
/// (the cut, or the air step's floor), up the right wall, back along the top. The bottom's run
/// within `2·delta` of the right end is its run at the left end turned a pitch, so the space and
/// its copy a pitch on are the same where a cell divider through a tooth middle crosses them, and
/// the cells' divider faces match when turned. Counts are fixed shares of `count`, so the slices'
/// points correspond. `cell`: the grid's cell sizes, by which points on the frame are told.
#[allow(clippy::too_many_arguments)]
fn periodic_loop(points: &[(f64, f64)], r: f64, pitch: f64, (lo, hi): (f64, f64), delta: f64, z1: f64, cell_f: f64, cell_z: f64, count: usize) -> Result<Vec<(f64, f64)>, String> {
    let on_frame = |q: (f64, f64)| q.0 - lo < 1e-3 * cell_f || hi - q.0 < 1e-3 * cell_f || z1 - q.1 < 1e-3 * cell_z;
    let n = points.len();
    let start = (0..n).find(|&k| on_frame(points[k])).ok_or_else(|| format!("the slice at r = {r} does not reach its frame"))?;
    // The longest run off the frame is the bottom.
    let (mut best, mut k) = ((0usize, 0usize), 0usize);
    while k < n {
        if on_frame(points[(start + k) % n]) {
            k += 1;
            continue;
        }
        let first = k;
        while k < n && !on_frame(points[(start + k) % n]) {
            k += 1;
        }
        if k - first > best.1 {
            best = (first, k - first);
        }
    }
    let mut bottom: Vec<(f64, f64)> = (0..best.1).map(|i| points[(start + best.0 + i) % n]).collect();
    if bottom.len() < 4 {
        return Err(format!("the slice at r = {r} has no bottom"));
    }
    if bottom[0].0 > bottom[bottom.len() - 1].0 {
        bottom.reverse();
    }
    let strip = 2.0 * delta;
    if !(bottom[0].0 < lo + 3.0 * cell_f && bottom[bottom.len() - 1].0 > hi - 3.0 * cell_f) {
        return Err(format!("the bottom of the slice at r = {r} does not run from wall to wall"));
    }
    // The bottom from the left wall to where it first leaves the left strip, by length: its turn
    // a pitch on is the bottom's run into the right wall.
    let enter = bottom.iter().position(|q| q.0 > lo + strip).ok_or_else(|| format!("the slice at r = {r} has no middle"))?;
    if enter == 0 {
        return Err(format!("the slice at r = {r} has no left strip"));
    }
    let (a, b) = (bottom[enter - 1], bottom[enter]);
    let exit = (lo + strip, a.1 + (b.1 - a.1) * (lo + strip - a.0) / (b.0 - a.0));
    let mut left_path = bottom[..enter].to_vec();
    left_path.push(exit);
    let first = bottom[0];
    let target = (first.0 + pitch, first.1);
    let end = (enter..bottom.len())
        .min_by(|&x, &y| {
            let d = |q: (f64, f64)| (r * (q.0 - target.0)).hypot(q.1 - target.1);
            d(bottom[x]).total_cmp(&d(bottom[y]))
        })
        .unwrap_or(bottom.len() - 1);
    let [_, walls, _, _, _, _] = sides(count);
    let (before, after) = strip_counts(count);
    let (strips, top) = (before + after, (count / 12).max(2));
    let middle_count = count.checked_sub(2 * walls + 2 * strips + top).filter(|&m| m >= 8).ok_or_else(|| format!("{count} points are too few for a slice"))?;
    let resample = |path: &[(f64, f64)], n: usize| -> Vec<(f64, f64)> {
        let mut length = vec![0.0];
        for k in 1..path.len() {
            length.push(length[k - 1] + (r * (path[k].0 - path[k - 1].0)).hypot(path[k].1 - path[k - 1].1));
        }
        let total = length[length.len() - 1];
        let mut piece = 0;
        (0..n)
            .map(|k| {
                let s = total * k as f64 / n as f64;
                while piece + 2 < length.len() && length[piece + 1] < s {
                    piece += 1;
                }
                let w = (s - length[piece]) / (length[piece + 1] - length[piece]).max(1e-300);
                let (a, b) = (path[piece], path[piece + 1]);
                (a.0 + (b.0 - a.0) * w, a.1 + (b.1 - a.1) * w)
            })
            .collect()
    };
    // The strip's crest: its highest point, or where it is flat (a tooth top's floor) its middle.
    let (low, high) = left_path.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), q| (a.min(q.1), b.max(q.1)));
    let crest = if high - low < 1e-6 * (z1 - low) {
        left_path.iter().enumerate().min_by(|a, b| (a.1 .0 - lo - delta).abs().total_cmp(&(b.1 .0 - lo - delta).abs())).map_or(0, |(k, _)| k)
    } else {
        left_path.iter().enumerate().max_by(|a, b| a.1 .1.total_cmp(&b.1 .1)).map_or(0, |(k, _)| k)
    };
    if crest == 0 || crest + 1 >= left_path.len() {
        return Err(format!("the left strip of the slice at r = {r} has its crest at an end"));
    }
    let mut left = resample(&left_path[..=crest], before);
    left.extend(resample(&left_path[crest..], after));
    let mut middle = vec![exit];
    middle.extend_from_slice(&bottom[enter..end]);
    middle.push(target);
    if middle.len() < 3 {
        return Err(format!("the slice at r = {r} is narrower than its strips"));
    }
    let mut out = Vec::with_capacity(count);
    out.extend((0..walls).map(|k| (lo, z1 + (first.1 - z1) * k as f64 / walls as f64)));
    out.extend_from_slice(&left);
    out.extend(resample(&middle, middle_count));
    out.extend(left.iter().map(|&(f, z)| (f + pitch, z)));
    out.extend((0..walls).map(|k| (hi, exit.1 + (z1 - exit.1) * k as f64 / walls as f64)));
    out.extend((0..top).map(|k| (hi + (lo - hi) * k as f64 / top as f64, z1)));
    Ok(out)
}

/// The indices at which a slice of `count` points ([`periodic_loop`]) breaks: the left wall, the
/// left strip, the bottom from the left strip's crest, the right strip's part after its crest,
/// the right wall and the top start there. (A crest is a tooth's tip where the teeth come to a
/// point below the face, a sharp ridge kept an edge of the loft rather than a crease in a face.)
fn sides(count: usize) -> [usize; 6] {
    let (walls, top, (before, after)) = ((count / 18).max(2), (count / 12).max(2), strip_counts(count));
    [0, walls, walls + before, count - top - walls - after, count - top - walls, count - top]
}

/// The points of a strip before and after its crest.
fn strip_counts(count: usize) -> (usize, usize) {
    let strips = (count / 15).max(4);
    (strips / 2, strips - strips / 2)
}

/// A closed polyline as degree-1 curves, one from each of `breaks` to the next (the last back to
/// the first), so its corners there are edges of the loft rather than creases inside a face.
fn closed_polyline(points: &[Vec3], breaks: &[usize]) -> Result<Vec<crate::NurbsCurve>, String> {
    let n = points.len();
    let open = |from: usize, to: usize| -> Result<crate::NurbsCurve, String> {
        let m = to - from;
        let mut knots = vec![0.0];
        knots.extend((0..=m).map(|k| k as f64 / m as f64));
        knots.push(1.0);
        crate::NurbsCurve::new(1, knots, (from..=to).map(|k| crate::Vec4::from_point(points[k % n], 1.0)).collect())
    };
    (0..breaks.len()).map(|k| open(breaks[k], if k + 1 < breaks.len() { breaks[k + 1] } else { n + breaks[0] })).collect()
}

/// One tooth space the thread cuts from `blank` under `motion`, as a solid to cut from it: traced
/// on cylinders about the gear's axis at `radii` (seeded from the middle one outward, starting
/// near `seed` there), `count` points each, the end slices (outside the ring) put on their tangent
/// planes, lofted. `pitch`: the angle between neighbouring spaces.
pub fn tooth_space(tool: &ThreadTool, motion: &TwoTurns, blank: &RingBlank, pitch: f64, radii: &[f64], seed: f64, count: usize) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    Ok(traced_space(tool, motion, blank, Family::Cylinders, pitch, radii, seed, count)?.0)
}

/// [`tooth_space`] and, at each of `radii`, the angle a cell divider after the space crosses it:
/// the tooth middle after the space, less than half its strip on, so the divider crosses a tooth
/// tip's flank and not its crest.
fn traced_space(tool: &ThreadTool, motion: &TwoTurns, blank: &RingBlank, family: Family, pitch: f64, radii: &[f64], seed: f64, count: usize) -> Result<(crate::BrepSolid, Vec<f64>), crate::KernelRefusal> {
    let fail = |message: String| crate::KernelRefusal::unsupported(crate::KernelStage::Classify, "thread_space", format!("threadSweep: {message}"));
    tool.check().map_err(fail)?;
    if radii.len() < 2 || count < 8 {
        return Err(fail("a space needs two slices or more, of eight points or more".into()));
    }
    let mid = radii.len() / 2;
    let mut sections: Vec<Vec<Vec3>> = vec![Vec::new(); radii.len()];
    let mut centres = vec![0.0; radii.len()];
    let (first, centre, delta) = slice(tool, motion, blank, family, pitch, radii[mid], seed, None, count).map_err(fail)?;
    sections[mid] = first;
    centres[mid] = divider_angle(centre, delta, pitch);
    let middle_delta = delta;
    let (mut near, mut strip) = (centre, middle_delta);
    for k in (0..mid).rev() {
        let (points, c, delta) = slice(tool, motion, blank, family, pitch, radii[k], near, Some((near, strip)), count).map_err(fail)?;
        sections[k] = points;
        (centres[k], near, strip) = (divider_angle(c, delta, pitch), c, delta);
    }
    (near, strip) = (centre, middle_delta);
    for k in mid + 1..radii.len() {
        let (points, c, delta) = slice(tool, motion, blank, family, pitch, radii[k], near, Some((near, strip)), count).map_err(fail)?;
        sections[k] = points;
        (centres[k], near, strip) = (divider_angle(c, delta, pitch), c, delta);
    }
    // The end slices on planes (the loft's end caps must be planar) square to the radius through
    // their middle: the inner one no farther out than the bore at the slice's widest angle from
    // that radius (so no part of the ring lies behind it), the outer one no nearer than the rim.
    let (o, d, _, _) = gear_frame(motion);
    let (low, high) = match family {
        Family::Cylinders => (blank.inner, blank.outer),
        Family::Planes => (blank.bottom, blank.face),
    };
    if !(radii[0] < low && radii[radii.len() - 1] > high) {
        return Err(fail(format!("the end slices ({} and {}) must lie beyond the blank ({low} and {high})", radii[0], radii[radii.len() - 1])));
    }
    // (Plane slices are planar already.)
    let ends: &[(usize, bool)] = if family == Family::Cylinders { &[(0, false), (radii.len() - 1, true)] } else { &[] };
    for &(k, outer) in ends {
        let centre = sections[k].iter().fold(Vec3::new(0.0, 0.0, 0.0), |a, q| a.add(*q)).scale(1.0 / count as f64);
        let off = centre.sub(o);
        let normal = off.sub(d.scale(off.dot(d))).normalized().map_err(|e| fail(e))?;
        let widest = sections[k]
            .iter()
            .map(|q| {
                let rel = q.sub(o);
                let flat = rel.sub(d.scale(rel.dot(d)));
                (flat.dot(normal) / flat.length()).clamp(-1.0, 1.0)
            })
            .fold(1.0f64, f64::min);
        let distance = if outer { radii[k].max(blank.outer) } else { radii[k].min(blank.inner * widest * (1.0 - 1e-9)) };
        for q in sections[k].iter_mut() {
            let rel = q.sub(o);
            let along = d.scale(rel.dot(d));
            let flat = rel.sub(along);
            *q = o.add(along).add(flat.scale(distance / flat.dot(normal)));
        }
    }
    let profiles: Vec<Vec<crate::NurbsCurve>> = sections.iter().map(|s| closed_polyline(s, &sides(count))).collect::<Result<_, _>>().map_err(fail)?;
    // The end slices (on their planes, so not turned copies at their two ends) are lofted on
    // their own, so the skin between the inner slices, all of the ring, is theirs alone.
    let skin = if profiles.len() >= 4 { crate::loft_topology::loft_profile_brep_broken(&profiles, &[1, profiles.len() - 2])? } else { crate::loft_profile_brep(&profiles)? };
    Ok((skin, centres))
}

/// [`thread_cut_gear`] at the kernel's own resolution: 15 slices (13 across the ring from a
/// thirtieth of its width inside the bore to as far outside the rim, and one a twelfth of its width
/// beyond each edge), 180 points each, the first space sought near half a pitch.
pub fn thread_gear(tool: &ThreadTool, motion: &TwoTurns, blank: &RingBlank, teeth: usize) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    let width = blank.outer - blank.inner;
    let (from, to) = (blank.inner - width / 30.0, blank.outer + width / 30.0);
    let mut radii = vec![blank.inner - width / 12.0];
    radii.extend((0..13).map(|k| from + (to - from) * k as f64 / 12.0));
    radii.push(blank.outer + width / 12.0);
    thread_cut_gear(tool, motion, blank, teeth, &radii, 0.5 * TAU / teeth.max(1) as f64, 180)
}

/// The wheel the thread cuts across the rim of `blank` (a ring from `inner`, 0 for a disk, to its
/// rim at `outer`, between the planes at `bottom` and `face`; `face_outer` unused), its `teeth`
/// spaces alike, at the kernel's own resolution: plane slices square to the wheel's axis (13
/// from 0.7/12 of its thickness below it to as far above, one a twelfth beyond each side; failing
/// that, two other layouts, and then the same nearer the blank), 180 points each. See
/// [`thread_cut_wheel`].
pub fn thread_wheel(tool: &ThreadTool, motion: &TwoTurns, blank: &RingBlank, teeth: usize) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    let thick = blank.face - blank.bottom;
    // The cut of the space from the cell can miss a crossing where a slice stands close to a
    // cap (the cases' worm wheel at 13 slices from 0.4/12 of the thickness beyond it does), so
    // a few layouts are tried in turn, the first that builds taken; each result is validated by
    // the cut and the sew as always.
    // The end slices stand a twelfth of the thickness beyond the blank, or nearer (a 24th, a 48th)
    // where the tool does not reach so far.
    // The layouts are first tried with the booleans' perturbation ladder off (a layout whose cut
    // is degenerate refuses at once, where the ladder spent a minute on it, and the next layout
    // is tried); only if every layout refuses are they tried again in turn with it.
    let layouts = [(13usize, 0.7), (17, 0.3), (11, 0.5)];
    let mut last = None;
    for perturbation in [false, true] {
        for end in [thick / 12.0, thick / 24.0, thick / 48.0] {
            for (inner, beyond) in layouts {
                let (from, to) = (blank.bottom - end * beyond, blank.face + end * beyond);
                let mut levels = vec![blank.bottom - end];
                levels.extend((0..inner).map(|k| from + (to - from) * k as f64 / (inner - 1) as f64));
                levels.push(blank.face + end);
                match thread_cut_wheel_with(tool, motion, blank, teeth, &levels, 0.5 * TAU / teeth.max(1) as f64, 180, perturbation) {
                    Ok(wheel) => return Ok(wheel),
                    Err(refusal) => last = Some(refusal),
                }
            }
        }
    }
    Err(last.expect("layouts tried"))
}

/// [`thread_cut_gear`] for a wheel cut across its rim: one space traced on the planes at
/// `levels` (heights along the wheel's axis) and lofted, the blank's cell about it (a wedge, or
/// for a ring an annular sector, at each height between radial dividers through that height's
/// divider angle, lofted from the blank's bottom to its face, the dividers leaning with the teeth)
/// cut by it, and the cell's faces less those on its dividers copied round the axis and sewn.
pub fn thread_cut_wheel(tool: &ThreadTool, motion: &TwoTurns, blank: &RingBlank, teeth: usize, levels: &[f64], seed: f64, count: usize) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    thread_cut_wheel_with(tool, motion, blank, teeth, levels, seed, count, true)
}

/// [`thread_cut_wheel`] with its booleans' perturbation ladder on or off.
#[allow(clippy::too_many_arguments)]
fn thread_cut_wheel_with(tool: &ThreadTool, motion: &TwoTurns, blank: &RingBlank, teeth: usize, levels: &[f64], seed: f64, count: usize, perturbation: bool) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    let boolean_options = crate::BooleanOptions::default();
    let boolean = |a: &crate::BrepSolid, b: &crate::BrepSolid, operation: crate::BooleanOperation| {
        if perturbation { crate::boolean_operation(a, b, operation, &boolean_options) } else { crate::without_perturbation(|| crate::boolean_operation(a, b, operation, &boolean_options)) }
    };
    let fail = |message: String| crate::KernelRefusal::unsupported(crate::KernelStage::Classify, "thread_wheel", format!("threadSweep: {message}"));
    if teeth < 2 || !(blank.inner >= 0.0 && blank.outer > blank.inner && blank.face > blank.bottom) {
        return Err(fail("a wheel needs two teeth or more and a blank of positive size".into()));
    }
    let pitch = TAU / teeth as f64;
    let (space, dividers) = traced_space(tool, motion, blank, Family::Planes, pitch, levels, seed, count)?;
    let (o, d, e1, e2) = gear_frame(motion);
    let middle: Vec<(f64, f64)> = levels.iter().copied().zip(dividers.iter().copied()).collect();
    let at = |z: f64| -> (f64, f64) {
        let k = (1..middle.len()).find(|&k| middle[k].0 >= z).unwrap_or(middle.len() - 1);
        let (a, b) = (middle[k - 1], middle[k]);
        (z, a.1 + (b.1 - a.1) * (z - a.0) / (b.0 - a.0))
    };
    // The wedge reaches a 24th of the thickness past the blank's sides, and a quarter past its rim,
    // so the blank's own faces bound the cell: none of the wedge's lies on them.
    let thick = blank.face - blank.bottom;
    let (low, high) = (blank.bottom - thick / 24.0, blank.face + thick / 24.0);
    let gap = 0.02 * thick;
    let mut plan = vec![at(low)];
    plan.extend(middle.iter().copied().filter(|m| m.0 > low + gap && m.0 < high - gap));
    plan.push(at(high));
    if plan.len() < 4 {
        return Err(fail("the slices give fewer than four divider angles across the wheel".into()));
    }
    // One section per height: out from the axis along the divider before the space, round past
    // the rim to the one after it, and back in.
    let line = |a: Vec3, b: Vec3| crate::NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![crate::Vec4::from_point(a, 1.0), crate::Vec4::from_point(b, 1.0)]);
    let reach = 1.25 * blank.outer;
    let mut sections = Vec::with_capacity(plan.len());
    for &(z, a) in &plan {
        let b = a - pitch;
        let centre = o.add(d.scale(z));
        let point = |rho: f64, angle: f64| centre.add(e1.scale(rho * angle.cos())).add(e2.scale(rho * angle.sin()));
        sections.push(vec![
            line(centre, point(reach, b)).map_err(fail)?,
            crate::make_arc(centre, e1, e2, reach, b, a).map_err(fail)?,
            line(point(reach, a), centre).map_err(fail)?,
        ]);
    }
    let wedge = crate::loft_profile_brep(&sections)?;
    // The blank itself, exact: its section revolved about the axis, the seam on the far side.
    let away = 0.5 * (plan[0].1 + plan[plan.len() - 1].1) - 0.5 * pitch + PI;
    let radial = e1.scale(away.cos()).add(e2.scale(away.sin()));
    let at_rz = |rho: f64, z: f64| o.add(radial.scale(rho)).add(d.scale(z));
    let corners = [at_rz(blank.inner, blank.bottom), at_rz(blank.outer, blank.bottom), at_rz(blank.outer, blank.face), at_rz(blank.inner, blank.face)];
    let profile: Vec<crate::NurbsCurve> = (0..4).map(|k| line(corners[k], corners[(k + 1) % 4])).collect::<Result<_, _>>().map_err(fail)?;
    let whole = crate::revolve_profile_brep(&profile, o, d, TAU).map_err(fail)?;
    let piece = boolean(&whole, &wedge, crate::BooleanOperation::Intersect)?;
    // The wedge's divider faces: those its radial lines sweep (their points spread across the
    // radius and over the height).
    let [piece_shell] = wedge.shells.as_slice() else {
        return Err(fail("the wheel's cell is not one shell".into()));
    };
    let samples = |solid: &crate::BrepSolid, face: &crate::topology::FaceRecord| -> Vec<Vec3> {
        let vertex = |id: u64| solid.vertices.iter().find(|v| v.id == id).map(|v| v.point);
        let mut points = Vec::new();
        for e in face.loops.iter().flat_map(|l| &l.coedges).filter_map(|c| solid.edges.iter().find(|e| e.id == c.edge_id)) {
            points.extend([vertex(e.start_vertex_id), vertex(e.end_vertex_id)].into_iter().flatten());
            points.extend((1..4).filter_map(|k| e.curve.evaluate(e.t0 + (e.t1 - e.t0) * k as f64 / 4.0).ok()));
        }
        points
    };
    let radius = |q: Vec3| {
        let rel = q.sub(o);
        rel.sub(d.scale(rel.dot(d))).length()
    };
    let height = |q: Vec3| q.sub(o).dot(d);
    let divider_surfaces: Vec<&crate::NurbsSurface> = piece_shell
        .faces
        .iter()
        .filter(|face| {
            let points = samples(&wedge, face);
            let spread = |f: &dyn Fn(Vec3) -> f64| points.iter().map(|&q| f(q)).fold(f64::NEG_INFINITY, f64::max) - points.iter().map(|&q| f(q)).fold(f64::INFINITY, f64::min);
            spread(&radius) > 0.5 * reach && spread(&height) > 0.5 * thick
        })
        .map(|face| &face.surface)
        .collect();
    if divider_surfaces.len() != 2 {
        return Err(fail(format!("the wheel's cell has {} divider faces, not two", divider_surfaces.len())));
    }
    let cell = boolean(&piece, &space, crate::BooleanOperation::Subtract)?;
    let [shell] = cell.shells.as_slice() else {
        return Err(fail("the cut cell is not one shell".into()));
    };
    let tolerance = 1e-7 * blank.outer;
    let on = |q: Vec3| divider_surfaces.iter().any(|surface| crate::project_point_to_surface(surface, q).is_ok_and(|f| f.distance < tolerance));
    let mut kept = Vec::new();
    let mut dropped = 0;
    for face in &shell.faces {
        let points = samples(&cell, face);
        if !points.is_empty() && points.iter().all(|&q| on(q)) {
            dropped += 1;
        } else {
            kept.push(face.id);
        }
    }
    if dropped == 0 {
        return Err(fail("no face of the cut cell lies on its dividers".into()));
    }
    let (wheel, report, _) = crate::sew_rotated_copies(&cell, &kept, o, d, pitch, teeth, (2e-8 * blank.outer).max(1e-6))?;
    if let Some(issue) = report.issues.first() {
        return Err(fail(format!("the cells do not sew into one valid solid: {issue}")));
    }
    Ok(wheel)
}

/// The angle of a cell divider at a slice whose space's middle is `centre` and strips `delta`.
fn divider_angle(centre: f64, delta: f64, pitch: f64) -> f64 {
    centre + 0.5 * pitch + 0.4 * delta
}

/// The gear the thread cuts from `blank` under `motion`, its `teeth` spaces alike: one space traced
/// and lofted ([`tooth_space`]), the blank's cell about it cut by it,
/// and the cell's faces, less those on its dividers, copied round the gear's axis and sewn.
/// The cell lies between two upright surfaces through the middles of the teeth either side of
/// the space (cubics through those middles in plan, the middles at the blank's radii read
/// linearly off the neighbouring slices), and is the blank's own between them, built directly.
pub fn thread_cut_gear(tool: &ThreadTool, motion: &TwoTurns, blank: &RingBlank, teeth: usize, radii: &[f64], seed: f64, count: usize) -> Result<crate::BrepSolid, crate::KernelRefusal> {
    let fail = |message: String| crate::KernelRefusal::unsupported(crate::KernelStage::Classify, "thread_gear", format!("threadSweep: {message}"));
    if teeth < 2 || !(blank.inner > 0.0 && blank.outer > blank.inner && blank.face.min(blank.face_outer) > blank.bottom) {
        return Err(fail("a gear needs two teeth or more and a ring of positive size".into()));
    }
    let pitch = TAU / teeth as f64;
    let (space, dividers) = traced_space(tool, motion, blank, Family::Cylinders, pitch, radii, seed, count)?;
    let (o, d, e1, e2) = gear_frame(motion);
    // The divider after the space by radius, with the ring's own radii as ends.
    let middle: Vec<(f64, f64)> = radii.iter().copied().zip(dividers.iter().copied()).collect();
    let at = |r: f64| -> (f64, f64) {
        let k = (1..middle.len()).find(|&k| middle[k].0 >= r).unwrap_or(middle.len() - 1);
        let (a, b) = (middle[k - 1], middle[k]);
        (r, a.1 + (b.1 - a.1) * (r - a.0) / (b.0 - a.0))
    };
    let gap = 0.02 * (blank.outer - blank.inner);
    let mut plan = vec![at(blank.inner)];
    plan.extend(middle.iter().copied().filter(|m| m.0 > blank.inner + gap && m.0 < blank.outer - gap));
    plan.push(at(blank.outer));
    if plan.len() < 4 {
        return Err(fail("the slices give fewer than four tooth middles inside the ring".into()));
    }
    let floor = o.add(d.scale(blank.bottom));
    let point = |r: f64, a: f64| floor.add(e1.scale(r * a.cos())).add(e2.scale(r * a.sin()));
    let divider = |shift: f64| -> Result<crate::NurbsCurve, String> {
        let points: Vec<Vec3> = plan.iter().map(|&(r, a)| point(r, a - shift)).collect();
        let mut parameters = vec![0.0];
        for k in 1..points.len() {
            parameters.push(parameters[k - 1] + points[k].sub(points[k - 1]).length());
        }
        let total = parameters[parameters.len() - 1];
        crate::interpolate_curve(&points, 3, &parameters.iter().map(|t| t / total).collect::<Vec<_>>())
    };
    let (ahead, behind) = (divider(0.0).map_err(fail)?, divider(pitch).map_err(fail)?);
    // The cell's floor: out along `ahead`, back round the rim, in along `behind`, round the bore.
    let (a_in, a_out) = (plan[0].1, plan[plan.len() - 1].1);
    let (b_in, b_out) = (a_in - pitch, a_out - pitch);
    let rim = crate::make_arc(floor, e1, e2, blank.outer, b_out, a_out).map_err(fail)?;
    let bore = crate::make_arc(floor, e1, e2, blank.inner, b_in, a_in).map_err(fail)?;
    let section = vec![ahead.clone(), rim.reversed().map_err(fail)?, behind.reversed().map_err(fail)?, bore];
    let top = blank.face.max(blank.face_outer);
    let piece = crate::extrude_profile_brep(&section, d, top - blank.bottom)?;
    // A conical face: the ring above it, wider than the cell and reaching over it, cut away too.
    let mut operands = vec![piece];
    if blank.face_outer != blank.face {
        let depth = tool.tip + tool.root;
        let width = blank.outer - blank.inner;
        let (near, far) = (blank.inner - 0.25 * width, blank.outer + 0.25 * width);
        // Its profile (and so its seam) on the side of the axis away from the cell, so the seam
        // crosses neither divider.
        let away = 0.5 * (a_in + a_out) + PI;
        let radial = e1.scale(away.cos()).add(e2.scale(away.sin()));
        let at = |rho: f64, z: f64| o.add(radial.scale(rho)).add(d.scale(z));
        let corners = [at(near, blank.face_at(near)), at(far, blank.face_at(far)), at(far, top + depth), at(near, top + depth)];
        let profile: Vec<crate::NurbsCurve> = (0..4)
            .map(|k| crate::NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![crate::Vec4::from_point(corners[k], 1.0), crate::Vec4::from_point(corners[(k + 1) % 4], 1.0)]))
            .collect::<Result<_, _>>()
            .map_err(fail)?;
        operands.push(crate::revolve_profile_brep(&profile, o, d, TAU).map_err(fail)?);
    }
    operands.push(space.clone());
    // The space holds all the cut between the tooth middles and a little past them, and the
    // dividers run through those middles, so the space alone is cut from the cell.
    // Each cut in turn: the cap (when the face is a cone), then the space.
    let mut cell = operands[0].clone();
    for cutter in &operands[1..] {
        cell = crate::boolean_operation(&cell, cutter, crate::BooleanOperation::Subtract, &crate::BooleanOptions::default())?;
    }
    // The faces on the dividers: every vertex and edge sample of theirs over one of the curves.
    let tolerance = 1e-7 * blank.outer;
    let on = |q: Vec3| {
        let flat = q.sub(d.scale(q.sub(floor).dot(d)));
        [&ahead, &behind].iter().any(|c| crate::project_point_to_curve(c, flat).is_ok_and(|f| f.distance < tolerance))
    };
    let [shell] = cell.shells.as_slice() else {
        return Err(fail("the cut cell is not one shell".into()));
    };
    let vertex = |id: u64| cell.vertices.iter().find(|v| v.id == id).map(|v| v.point);
    let edge = |id: u64| cell.edges.iter().find(|e| e.id == id);
    let mut kept = Vec::new();
    let mut dropped = 0;
    for face in &shell.faces {
        let mut points = Vec::new();
        for e in face.loops.iter().flat_map(|l| &l.coedges).filter_map(|c| edge(c.edge_id)) {
            points.extend([vertex(e.start_vertex_id), vertex(e.end_vertex_id)].into_iter().flatten());
            points.extend((1..4).filter_map(|k| e.curve.evaluate(e.t0 + (e.t1 - e.t0) * k as f64 / 4.0).ok()));
        }
        if !points.is_empty() && points.iter().all(|&q| on(q)) {
            dropped += 1;
        } else {
            kept.push(face.id);
        }
    }
    if dropped == 0 {
        return Err(fail("no face of the cut cell lies on its dividers".into()));
    }
    // A divider's cut curve and its twin a pitch back are marched apart, each to the boolean's
    // fit tolerance (1e-7), so they are sewn to a few times that, scaled to the gear.
    let (gear, report, _) = crate::sew_rotated_copies(&cell, &kept, o, d, pitch, teeth, (2e-8 * blank.outer).max(1e-6))?;
    if let Some(issue) = report.issues.first() {
        return Err(fail(format!("the cells do not sew into one valid solid: {issue}")));
    }
    Ok(gear)
}

/// Whether `angle` is within `PI` of 0 once reduced: for tests and callers that compare phases.
#[doc(hidden)]
pub fn wrapped(angle: f64) -> f64 {
    (angle + PI).rem_euclid(TAU) - PI
}

