use super::*;

/// The largest tangent break, in radians, a chained path may have at a joint
/// before the joint is a CORNER.
///
/// This is a REAL geometric boundary, not a tuning knob, and it now selects the
/// LANE rather than deciding a refusal. The skinning builder lofts consecutive
/// stations, so a corner between two stations would be rendered as a smooth
/// blend across it — the tube cuts the corner, and near a sharp one the section
/// sweeps through itself. So a break inside this band is skinned as ONE tube
/// (line→fillet→line, spline pieces, an arc train), and a break past it is a
/// corner, which is built a different way or not at all:
///   - every segment STRAIGHT (a polyline, open or CLOSED) → the MITRE lane: the
///     section is swept along each segment and the two sweeps are trimmed to
///     the joint's bisector plane, sharing one face loop there
///     ([`Self::cornered_polyline`], `sweep_topology/miter.rs`). A closed one is
///     a picture FRAME: its closing joint is a mitre like the rest and it has no
///     caps at all;
///   - a corner at a CURVED segment → refused by name. A curve has no single
///     direction for a bisector plane to bisect, so there is no plane both
///     sides can be trimmed to.
///
/// ~1.15° (0.02 rad). Loose enough to absorb the tangent disagreement of two
/// curves a sketch chained on endpoint coincidence, tight enough that anything
/// a user would call a corner is refused rather than silently rounded off.
///
/// This is INDEPENDENT of the chainer's join tolerance, and deliberately so.
/// `common::chain_path_segments` joins on POSITION (`1e-5 * scale`); this gate
/// measures DIRECTION. Two collinear segments meeting with a positional gap have
/// a zero tangent break, and two segments meeting exactly at a point can still
/// break 90°. So a run the chainer happily orders head-to-tail may still be
/// refused by a consumer of this classification — which is not an inconsistency
/// but the SW/SWP split itself: the chainer's job is to decide what order the
/// picks form a run in, and this gate's job is to decide whether that run is one
/// the skinning builder can carry a section along.
pub const MAX_JOINT_TANGENT_BREAK: f64 = 0.02;

/// How much the CURVATURE VECTOR may disagree across a joint, relative to the
/// curvatures being compared, before the joint is G1 rather than G2.
///
/// Dimensionless and deliberately tiny: the two sides of a G2 joint are the same
/// analytic curve evaluated from two parameterizations (two arcs of one circle,
/// two collinear lines), so they agree to rounding — a dozen orders of magnitude
/// inside this bar. Everything else separates by a FULL curvature: a line
/// meeting a fillet of radius `r` jumps by `1/r`, an S-bend of two equal arcs by
/// `2/r`. There is no population in between to tune against, which is why this
/// number needs no tuning.
const MAX_JOINT_CURVATURE_BREAK: f64 = 1e-9;

/// How far a sampled path point may sit off its own fitted plane, relative to
/// the path's extent, before the path is SPATIAL rather than planar.
///
/// Read only by the CLOSED lane, and the reason it is tight: a planar closed
/// path transports a section round the loop and back to itself exactly (the
/// frame's only rotation is about the plane normal, which comes back to where it
/// started), while a spatial one returns the section rotated by the path's
/// HOLONOMY. Calling a spatial path planar would mean skinning that mismatch
/// into the ring instead of refusing it, so the bar admits only paths that are
/// planar to rounding — a sketch-authored chain, or resident edges that share a
/// plane — and refuses the rest by name.
const MAX_PLANARITY_DEVIATION: f64 = 1e-7;

/// How continuous a path is AT ONE JOINT — the classification every sweep
/// refusal reads instead of re-deriving the same angle from the curves.
///
/// The three levels are the standard geometric ones, and each is decided by a
/// measurement [`PathJoint`] carries beside it:
///   - `G0` — the position matches and nothing else: the tangent turns by more
///     than [`MAX_JOINT_TANGENT_BREAK`]. A corner.
///   - `G1` — the unit tangents agree; the curvature VECTORS do not. A line
///     meeting a fillet arc, two arcs of different radius, an S-bend of two
///     equal arcs curving opposite ways.
///   - `G2` — the unit tangents AND the curvature vectors agree. Two arcs of ONE
///     circle (the shape a sketch publishes for a circle), two collinear lines.
///
/// G1 and G2 are not distinguished by any refusal today — the skinning builder
/// needs G1 and is indifferent to the curvature jump, because it interpolates
/// the stations rather than the segments. The distinction is recorded because it
/// is the difference between a path whose swept surface is curvature-continuous
/// and one whose is not, which is what a later curvature-driven station law
/// (remaining work item 2) has to refine against.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointContinuity {
    /// Position only — a corner.
    G0,
    /// Tangent continuous, curvature discontinuous.
    G1,
    /// Tangent and curvature continuous.
    G2,
}

impl JointContinuity {
    /// The `G0` / `G1` / `G2` spelling, for a message or a record.
    pub fn label(self) -> &'static str {
        match self {
            JointContinuity::G0 => "G0",
            JointContinuity::G1 => "G1",
            JointContinuity::G2 => "G2",
        }
    }
}

/// ONE joint of a resolved path: which two segments meet there, how continuously
/// ([`JointContinuity`]), and the two measurements that decided it.
///
/// `before`/`after` are INDICES into the path's segments, and the CLOSING joint
/// of a closed path is `(n-1, 0)` — including the single-segment case `(0, 0)`,
/// a lone closed curve meeting itself.
#[derive(Debug, Clone, Copy)]
pub struct PathJoint {
    /// The segment ARRIVING at the joint (at its own domain end).
    pub before: usize,
    /// The segment DEPARTING from it (at its own domain start).
    pub after: usize,
    /// The angle between the arriving and departing UNIT TANGENTS, in radians.
    /// The measured angle a corner refusal quotes.
    pub tangent_break: f64,
    /// `|Δk|` of the CURVATURE VECTOR across the joint, in 1/length. A vector
    /// rather than a magnitude, so two equal arcs curving opposite ways (an
    /// S-bend) read `2/r` rather than zero.
    pub curvature_break: f64,
    /// The level those two measurements put the joint at.
    pub continuity: JointContinuity,
    /// True for the joint that CLOSES a closed path (the last segment's end
    /// meeting the first segment's start). An open path has none.
    pub closing: bool,
}

/// A resolved sweep path AND its classification: the ordered, head-to-tail
/// curves, the name of each, whether the run is CLOSED, the PLANE it lies in if
/// it lies in one, and the continuity of every joint — the closing one included.
///
/// This is the sweep's path type, and the classification is computed ONCE, here,
/// where the path is resolved. Before it, three refusals each derived their own
/// answer to the same question from the same curves: the builder's corner gate
/// measured a tangent break while sampling, the sampler decided a path was
/// closed by comparing its first and last station, and `SW`'s `translate` mode
/// inferred closure from the fact that a closed path's advances through the
/// profile must sum to zero. They agreed, but nothing made them agree, and none
/// of the three could SAY what it had found — a closed path was only ever
/// reported as the consequence the refusing rule happened to notice.
///
/// ORDER AND ORIENTATION are the caller's: this classifies the chain it is
/// given, segment order and each segment's direction untouched, so the first
/// segment still decides which way the sweep runs.
#[derive(Debug, Clone)]
pub struct SweepPath {
    /// The ordered, head-to-tail world curves.
    pub curves: Vec<NurbsCurve>,
    /// Each curve's source name, parallel to `curves`. A short or empty slice is
    /// legal — [`SweepPath::name`] degrades to `<unnamed>` — because a caller
    /// that builds a chain itself (the tube's bend runs, the wire harness'
    /// spline spans) has no per-segment user-facing name to give.
    pub names: Vec<String>,
    /// The last segment's end returns to the first segment's start.
    pub closed: bool,
    /// The UNIT NORMAL of the plane the whole path lies in, when it lies in one
    /// to within [`MAX_PLANARITY_DEVIATION`] of its own extent.
    ///
    /// `None` for a spatial path, and also for one whose sampled polygon
    /// encloses no signed area to derive a normal from — a straight run, or a
    /// figure-of-eight whose lobes cancel. Only the CLOSED lane reads this, and
    /// a straight run is never closed.
    pub planar: Option<Vec3>,
    /// Every joint, in order: the interior ones `(0,1), (1,2), …`, then the
    /// CLOSING one `(n-1, 0)` when the path is closed.
    pub joints: Vec<PathJoint>,
    /// Every segment is a STRAIGHT LINE — a degree-1 curve with two control
    /// points, which is the test `SW`'s `translate` mode applies to a segment
    /// before it reduces it to a chord.
    ///
    /// Read with [`Self::corner`] by the MITRE lane: a cornered run of straight
    /// segments is a polyline, and a polyline corner is the one corner a sweep
    /// can join exactly (see [`Self::cornered_polyline`]). A corner at a CURVED
    /// segment is not — there is no single bisector plane a curve's section can
    /// be trimmed to — so the straightness of the segments, not the size of the
    /// break, is what separates the two.
    pub straight: bool,
    /// Each segment's SCREW AXIS direction, parallel to `curves`: `Some` for a
    /// segment its builder KNOWS is a helix about that axis, `None` for every
    /// other curve. Unit length; its sign is irrelevant.
    ///
    /// Not measured here, and deliberately so: a helix's published curve is a
    /// cubic FIT, and a fit is not a helix to any bar a classification could
    /// hold it to without a tolerance keyed to the fit density — and a TAPERED
    /// helix is not a constant-slope curve at all. The feature that wound the
    /// helix hands its axis over instead ([`Self::with_screw_axes`]).
    ///
    /// Read only by the `Rigid` placement (SW `pathAlign`): on a helix the path's
    /// own motion is the SCREW about this axis, and the rotation-minimizing frame
    /// every other curve is carried by rolls away from that screw by the helix's
    /// integrated torsion — 169.75° over three turns of a radius-5, pitch-5 coil.
    pub screw_axes: Vec<Option<Vec3>>,
}

impl SweepPath {
    /// Classify `curves` (ordered head-to-tail) into a path. `names` may be
    /// shorter than `curves`, or empty.
    pub fn new(curves: Vec<NurbsCurve>, names: Vec<String>) -> Result<Self, KernelRefusal> {
        if curves.is_empty() {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "chain_empty",
                "sweepSolid: path chain is empty",
            ));
        }
        // --- Endpoints, and the SCALE every position test below is relative to.
        //     Same form as the chainer's (`1e-5 * scale`, floored at 1.0 so a
        //     path at the origin gets an absolute band): the two have to agree
        //     about closure, since the chainer STOPS extending a run when its
        //     tail returns to its head and this is what then reports that the
        //     run it produced is a ring.
        let mut endpoints = Vec::with_capacity(curves.len());
        for curve in &curves {
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
            endpoints.push((curve.evaluate(t0).or_refuse(KernelStage::Classify, "evaluate")?, curve.evaluate(t1).or_refuse(KernelStage::Classify, "evaluate")?));
        }
        let first_start = endpoints[0].0;
        let scale = endpoints
            .iter()
            .flat_map(|(a, b)| [a, b])
            .map(|point| point.sub(first_start).length())
            .fold(1.0_f64, f64::max);
        let join_tolerance = 1e-5 * scale;

        // --- Sample the run once, for the planarity fit and for the travelled
        //     distance the closure test needs a floor from. 16 per curve is the
        //     sampling density `profile_anchor` uses to judge a profile's own
        //     plane, so a path and a profile are held to the same resolution.
        let mut samples = Vec::with_capacity(16 * curves.len() + 1);
        for curve in &curves {
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
            for index in 0..16 {
                samples.push(curve.evaluate(t0 + (t1 - t0) * index as f64 / 16.0).or_refuse(KernelStage::Classify, "evaluate")?);
            }
        }
        samples.push(endpoints[curves.len() - 1].1);
        let travel: f64 = samples
            .windows(2)
            .map(|pair| pair[1].sub(pair[0]).length())
            .sum();

        // --- CLOSURE. The run returns to where it started, and has actually
        //     gone somewhere: a zero-length curve trivially ends where it began,
        //     and calling that a ring would send it into the closed lane instead
        //     of the zero-length refusal that names it.
        let closed = travel > join_tolerance
            && endpoints[curves.len() - 1]
                .1
                .sub(first_start)
                .length()
                <= join_tolerance;

        // --- PLANARITY. Newell over the sampled polygon gives the candidate
        //     normal (exact for a planar sample set, whatever the winding), and
        //     the samples are then CHECKED against it — a normal that fits
        //     nothing is no answer. A degenerate normal (no enclosed area) is
        //     `None` rather than an error: an open straight run is a perfectly
        //     good path that simply has no one plane.
        let planar = match crate::polygon::newell_normal(&samples).normalized() {
            Ok(normal) => {
                let deviation = samples
                    .iter()
                    .map(|point| point.sub(first_start).dot(normal).abs())
                    .fold(0.0_f64, f64::max);
                (deviation <= MAX_PLANARITY_DEVIATION * scale).then_some(normal)
            }
            Err(_) => None,
        };

        // --- JOINTS, interior then closing. A closed single-curve path has
        //     exactly one joint — the curve meeting itself.
        let mut joints = Vec::with_capacity(curves.len());
        for index in 1..curves.len() {
            joints.push(Self::joint(&curves, index - 1, index, false)?);
        }
        if closed {
            joints.push(Self::joint(&curves, curves.len() - 1, 0, true)?);
        }

        // --- STRAIGHTNESS. The same test `SW`'s `translate` loop applies before
        //     it reduces a segment to its chord: a degree-1 curve with two
        //     control points. Recorded per PATH rather than per segment because
        //     every consumer asks about the whole run — the mitre lane needs all
        //     of them straight, and one curved segment is what sends a cornered
        //     run back to the refusal.
        let straight = curves
            .iter()
            .all(|curve| curve.degree == 1 && curve.control_points.len() == 2);

        let screw_axes = vec![None; curves.len()];
        Ok(Self {
            curves,
            names,
            closed,
            planar,
            joints,
            straight,
            screw_axes,
        })
    }

    /// Attach each segment's SCREW AXIS ([`Self::screw_axes`]), parallel to the
    /// curves. A zero axis is refused rather than silently read as "no axis".
    pub fn with_screw_axes(mut self, axes: Vec<Option<Vec3>>) -> Result<Self, KernelRefusal> {
        if axes.len() != self.curves.len() {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "screw_axes_count",
                format!(
                    "sweepSolid: {} screw axes for a {}-segment path",
                    axes.len(),
                    self.curves.len()
                ),
            ));
        }
        self.screw_axes = axes
            .into_iter()
            .enumerate()
            .map(|(index, axis)| {
                axis.map(|axis| {
                    axis.normalized().map_err(|_| {
                        KernelRefusal::input(
                            KernelStage::Collect,
                            "screw_axis",
                            format!(
                                "sweepSolid: path segment '{}' has a degenerate screw axis",
                                self.name(index)
                            ),
                        )
                    })
                })
                .transpose()
            })
            .collect::<Result<_, _>>()?;
        Ok(self)
    }

    /// Classify a borrowed chain — the entry a caller assembling curves for one
    /// build uses ([`Self::new`] takes them by value for a caller that owns them).
    pub fn from_curves(curves: &[NurbsCurve], names: &[String]) -> Result<Self, KernelRefusal> {
        Self::new(curves.to_vec(), names.to_vec())
    }

    /// One joint's measurements and level. `before` is read at its domain END and
    /// `after` at its domain START — the tangent the path ARRIVES with against
    /// the one it DEPARTS with, which is the only pair that measures the joint
    /// itself: sampling a station further along either side would fold that
    /// segment's own curvature into the break and let a genuine corner through
    /// on a curved segment.
    fn joint(
        curves: &[NurbsCurve],
        before: usize,
        after: usize,
        closing: bool,
    ) -> Result<PathJoint, KernelRefusal> {
        let arriving = Self::frame_at(&curves[before], true, before, "end")?;
        let departing = Self::frame_at(&curves[after], false, after, "start")?;
        // Both unit, so the dot is the cosine of the break.
        let tangent_break = arriving.0.dot(departing.0).clamp(-1.0, 1.0).acos();
        let curvature_break = departing.1.sub(arriving.1).length();
        let curvature_bar = MAX_JOINT_CURVATURE_BREAK
            * (1.0 + arriving.1.length() + departing.1.length());
        let continuity = if tangent_break > MAX_JOINT_TANGENT_BREAK {
            JointContinuity::G0
        } else if curvature_break > curvature_bar {
            JointContinuity::G1
        } else {
            JointContinuity::G2
        };
        Ok(PathJoint {
            before,
            after,
            tangent_break,
            curvature_break,
            continuity,
            closing,
        })
    }

    /// `(unit tangent, curvature vector)` at one end of one segment.
    ///
    /// The curvature VECTOR is `k = (r' × r'') × r' / |r'|⁴` — magnitude `κ`,
    /// pointing at the centre of curvature — so it is invariant to the
    /// parameterization (a rational arc is not parameterized in its own angle)
    /// and it carries the bend's DIRECTION, which the magnitude alone does not.
    /// A degree-1 segment has `r'' = 0` and so `k = 0`, which is the honest
    /// reading: a line's curvature is zero.
    fn frame_at(
        curve: &NurbsCurve,
        at_end: bool,
        index: usize,
        which: &str,
    ) -> Result<(Vec3, Vec3), KernelRefusal> {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
        let parameter = if at_end { t1 } else { t0 };
        let derivatives = curve.derivatives(parameter, 2).or_refuse(KernelStage::Classify, "derivatives")?;
        let speed = derivatives[1].length();
        let tangent = derivatives[1].normalized().map_err(|_| {
            KernelRefusal::input(
                KernelStage::Classify,
                "path_tangent",
                format!("sweepSolid: path tangent is degenerate at the {which} of segment {index}"),
            )
        })?;
        let curvature = derivatives[1]
            .cross(derivatives[2])
            .cross(derivatives[1])
            .scale(1.0 / speed.powi(4));
        Ok((tangent, curvature))
    }

    /// The number of segments.
    pub fn len(&self) -> usize {
        self.curves.len()
    }

    /// No segments — impossible through [`Self::new`], which refuses one.
    pub fn is_empty(&self) -> bool {
        self.curves.is_empty()
    }

    /// Segment `index`'s name, degrading to `<unnamed>` for a caller that named
    /// none.
    pub fn name(&self, index: usize) -> &str {
        self.names
            .get(index)
            .map(String::as_str)
            .unwrap_or("<unnamed>")
    }

    /// The first CORNER (a `G0` joint), if the path has one — what a builder that
    /// skins between stations refuses on.
    pub fn corner(&self) -> Option<&PathJoint> {
        self.joints
            .iter()
            .find(|joint| joint.continuity == JointContinuity::G0)
    }

    /// This path is a CORNERED POLYLINE: every segment straight, with at least
    /// one `G0` joint. The MITRE lane's selector, and the one predicate both the
    /// builder and the two features read — the builder to pick the lane, the
    /// features to know how many faces to name (one wall per segment × section
    /// edge, rather than one per section edge).
    ///
    /// Each clause is a separate geometric reason:
    ///   - STRAIGHT, because a mitre trims both sides to ONE plane, and only a
    ///     straight segment has a single direction for that plane to bisect;
    ///   - a CORNER, because a tangent-continuous polyline (collinear segments)
    ///     is already carried by the skinning lane, which must keep it.
    ///
    /// CLOSED IS NOT A CLAUSE, and that is this predicate's whole content: a
    /// closed cornered polyline — a picture FRAME — is mitred at every joint
    /// INCLUDING the closing one, and comes out as one wall per (segment ×
    /// section curve) with no caps at all. On a PLANAR loop the joint rotations
    /// are all about the path plane's normal and sum to the loop's total turning,
    /// `±2π`, so the section carried once round comes back as itself and the
    /// closing mitre is built exactly like the others (`sweep_topology/miter.rs`
    /// derives it, and measures the residual it predicts to be zero). A SPATIAL
    /// loop's section returns rolled by the path's own holonomy, and the lane
    /// closes it with a counter-twist shared over the sides — the same closure
    /// policy the smooth ring lane applies ([`crate::SweepClosure`]).
    ///
    /// So this answers "is this the MITRE lane", and [`Self::closed`] answers
    /// "does the result have caps". Both features read both.
    pub fn cornered_polyline(&self) -> bool {
        self.straight && self.corner().is_some()
    }
}

