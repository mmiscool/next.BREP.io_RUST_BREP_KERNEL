//! An OUTWARD shell of ONE face of a partial REVOLVE, built in the meridian
//! half-plane and revolved once.
//!
//! The configuration: a solid revolved through less than a full turn, whose one
//! RETAINED face is the revolve of a circular arc, every other face opened, and
//! the shell grown outward. A torus band grown past its own major radius is the
//! reason this lane exists: its offset folds in a BAND across the trim, and the
//! imprint pipeline can neither use the carve's three-piece division nor — with
//! the chord of a circular segment as the opening — close the regular case
//! either (measured on 2026-09-17: the same wedge grown by 0.5, no fold
//! anywhere, reads "completion produced non-integral genus", because an outward
//! wall on a CURVED opening face is the unpadded source face and never reaches
//! the grown offset).
//!
//! ## What the shell owes, and why the skin is never carved
//!
//! An outward shell is the grown solid less the source, where the grown solid
//! is the source with its retained faces offset and its OPENED faces held in
//! place, extended along themselves. Everything is planar in the meridian
//! half-plane: with the tube centre `T`, the skin radius `r` and the grown
//! radius `r + |d|`, the region is the annulus between the two circles, cut by
//! the profile's two straight edges at the ends of the arc (extended outward
//! until they meet the grown circle) and by the AXIS.
//!
//! Where the grown circle crosses the axis — `R + (r + |d|)·cos φ < 0`, the
//! fold band — its normals have passed through the axis and come out at the
//! azimuth half a turn away. That REFLECTED part lies in front of both azimuth
//! openings, and an opening removes everything in front of it; so the region
//! stops at the axis, and the two end caps meet along the chord the band cuts
//! there. The SKIN over the band stays: every point between it and the axis is
//! within `|d|` of it along its own outward normal and in front of no opening.
//! Carving it away as well would close a shell too — two bodies, each with a
//! pinch cone — and would drop exactly that material: on the R = 3, r = 2 ring
//! grown by 4 the difference is `(π/2)(35√3/3 − 4π)` = 12.00228, and the point
//! ρ = 0.5, z = 0 at the mid azimuth is in neither body. It is the rule the
//! apex-cone carve already ships: the skin whole, the offset trimmed where it
//! collapses.
//!
//! A full-revolution thicken of the same sheet builds `S′ ∪ lemon`
//! (`offset/thicken/band.rs`); this lane builds `S′` clipped by the openings,
//! and the lemon is precisely what the azimuth openings remove.
//!
//! ## What it declines
//!
//! Anything that is not exactly this configuration goes back to the imprint
//! pipeline untouched, with the reason logged under `BREP_OS_DEBUG`: an inward
//! shell, more than one retained face, a full revolution, a generatrix that is
//! not a circle, a trim that is not the whole face, material outside the tube, a
//! neighbouring profile edge that is not straight, an extended edge that does
//! not leave the tube or that reaches the axis, two extended edges that cross,
//! and a fold that reaches an END of the grown arc rather than lying inside it —
//! that last is the carve's single-chord lane, and it keeps it.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use crate::thicken::band::{
    fit_meridian_circle, outward_at, whole_face, wrap_into, MERIDIAN_RESIDUAL_BAR,
};
use crate::{make_arc, make_line, NurbsCurve};
use std::f64::consts::{PI, TAU};

/// How far a sampled point may sit off the geometry it is tested against,
/// relative to the model scale.
const LINEAR_BAR: f64 = 1e-9;
/// How far a cap's azimuth may sit from the revolve's own start or end, in
/// radians.
const AZIMUTH_BAR: f64 = 1e-9;
/// How far the grown circle may pass the axis and still be called TANGENT to
/// it, relative to the model scale.
const TANGENT_BAR: f64 = 1e-9;

/// Whether the lane takes a shell, and why not when it does not.
pub(super) enum Recognition {
    Wedge(RevolvedWedge),
    Declined(String),
}

/// One end of the skin arc in the start cap's half-plane.
#[derive(Clone, Copy, Debug)]
struct ArcEnd {
    /// Meridian angle about the tube centre, unwrapped with the arc.
    angle: f64,
    /// Where the extended profile edge meets the grown circle.
    grown_angle: f64,
    /// The opened face the profile edge at this end revolves to.
    wall: u64,
}

/// The recognized configuration, in the start cap's half-plane.
#[derive(Clone, Debug)]
pub(super) struct RevolvedWedge {
    origin: Vec3,
    axis: Vec3,
    /// Unit radial direction of the start cap's half-plane.
    radial: Vec3,
    sweep: f64,
    centre: Vec3,
    major: f64,
    axial: f64,
    skin: f64,
    grown: f64,
    /// The arc's two ends, `low.angle < high.angle`.
    low: ArcEnd,
    high: ArcEnd,
    /// The fold band on the grown arc, `[start, end]`, when the grown circle
    /// reaches the axis inside the arc; equal ends when it only touches it.
    band: Option<[f64; 2]>,
    retained: u64,
    start_cap: u64,
    end_cap: u64,
}

impl RevolvedWedge {
    pub(super) fn report(&self) -> String {
        format!(
            "R = {:.6}, s ∈ [{:.6}, {:.6}], skin [{:.6}, {:.6}] rad, grown [{:.6}, {:.6}] rad, \
             fold band {}, sweep {:.6}",
            self.major,
            self.skin,
            self.grown,
            self.low.angle,
            self.high.angle,
            self.low.grown_angle,
            self.high.grown_angle,
            self.band
                .map(|[a, b]| format!("[{a:.6}, {b:.6}] rad"))
                .unwrap_or_else(|| "none".to_string()),
            self.sweep
        )
    }

    fn at(&self, s: f64, phi: f64) -> Vec3 {
        self.centre
            .add(self.radial.scale(s * phi.cos()))
            .add(self.axis.scale(s * phi.sin()))
    }

    fn meridian_angle(&self, point: Vec3) -> f64 {
        let delta = point.sub(self.centre);
        delta.dot(self.axis).atan2(delta.dot(self.radial))
    }
}

/// A profile edge walked the way its cap's loop walks it.
struct LoopCurve {
    edge_id: u64,
    curve: NurbsCurve,
    t0: f64,
    t1: f64,
    forward: bool,
}

impl LoopCurve {
    fn at(&self, fraction: f64) -> Result<Vec3, KernelRefusal> {
        let fraction = if self.forward { fraction } else { 1.0 - fraction };
        self.curve.evaluate(self.t0 + (self.t1 - self.t0) * fraction).or_refuse(KernelStage::Refine, "evaluate")
    }
}

fn declined(reason: impl Into<String>) -> Result<Recognition, KernelRefusal> {
    Ok(Recognition::Declined(reason.into()))
}

/// The cap faces: planes that contain the revolve axis.
fn plane_contains_axis(face: &FaceRecord, origin: Vec3, axis: Vec3, linear: f64) -> bool {
    let Some(crate::AnalyticSurface::Plane {
        origin: plane_origin,
        u_dir,
        v_dir,
        ..
    }) = face.surface.analytic()
    else {
        return false;
    };
    let Ok(normal) = u_dir.cross(*v_dir).normalized() else {
        return false;
    };
    normal.dot(axis).abs() <= LINEAR_BAR && plane_origin.sub(origin).dot(normal).abs() <= linear
}

/// Recognize the configuration, or say why this shell is not it.
///
/// Never an error for a shape it does not take: the imprint pipeline owns the
/// input checks and every refusal, so a declined shell reaches it exactly as it
/// would have without this lane.
pub(super) fn recognize(
    source: &BrepSolid,
    opening_face_ids: &[u64],
    distance: f64,
) -> Result<Recognition, KernelRefusal> {
    if !(distance < -MINIMUM_DISTANCE) || !distance.is_finite() {
        return declined(format!(
            "not a finite outward distance past {MINIMUM_DISTANCE:e}, where the shell refuses"
        ));
    }
    let faces = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    if opening_face_ids
        .iter()
        .any(|id| !faces.iter().any(|face| face.id == *id))
    {
        return declined("an opening face is not in the source");
    }
    let retained = faces
        .iter()
        .filter(|face| !opening_face_ids.contains(&face.id))
        .collect::<Vec<_>>();
    let [face] = retained[..] else {
        return declined(format!("{} retained faces, not one", retained.len()));
    };
    let face = *face;
    let Some(crate::AnalyticSurface::Revolution { frame, sweep, .. }) = face.surface.analytic()
    else {
        return declined("the retained face is not a general revolution");
    };
    let (origin, axis, sweep) = (frame.origin, frame.axis, *sweep);
    if sweep >= TAU - AZIMUTH_BAR {
        return declined("a full revolution has no azimuth openings to stop the fold at the axis");
    }
    let linear = LINEAR_BAR * crate::solid_scale(source);
    if source.edges.iter().any(|edge| edge.degenerate) {
        return declined("the source has a degenerate edge, so it touches the axis");
    }
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let loops = face
        .loops
        .iter()
        .map(|record| {
            record
                .coedges
                .iter()
                .map(|coedge| coedge.pcurve.clone())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    if !whole_face(&loops, [u0, u1], [v0, v1])? {
        return declined("the retained trim is not the whole face");
    }

    // The two caps, at azimuth 0 and at the sweep.
    let azimuth = |point: Vec3| -> Option<f64> {
        let delta = point.sub(origin);
        let planar = delta.sub(axis.scale(delta.dot(axis)));
        if planar.length() <= linear {
            return None;
        }
        let theta = planar.dot(frame.y_axis).atan2(planar.dot(frame.x_axis));
        Some(if theta < 0.0 { theta + TAU } else { theta })
    };
    let face_edges = |face: &FaceRecord| {
        face.loops
            .iter()
            .flat_map(|record| &record.coedges)
            .map(|coedge| coedge.edge_id)
            .collect::<HashSet<_>>()
    };
    let edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let cap_azimuth = |cap: &FaceRecord| -> Result<Option<f64>, KernelRefusal> {
        let edge = cap
            .loops
            .first()
            .and_then(|record| record.coedges.first())
            .and_then(|coedge| edge_by_id.get(&coedge.edge_id))
            .ok_or_else(|| format!("offset_shell: cap face {} has no edge", cap.id)).or_refuse(KernelStage::Refine, "offset_shell_cap_face_has_no_edge")?;
        for index in 0..=8 {
            let point = edge
                .curve
                .evaluate(edge.t0 + (edge.t1 - edge.t0) * index as f64 / 8.0).or_refuse(KernelStage::Refine, "evaluate")?;
            if let Some(theta) = azimuth(point) {
                return Ok(Some(theta));
            }
        }
        Ok(None)
    };
    let mut start_cap = None;
    let mut end_cap = None;
    for cap in faces
        .iter()
        .filter(|candidate| plane_contains_axis(candidate, origin, axis, linear))
    {
        let Some(theta) = cap_azimuth(cap)? else {
            return declined(format!("cap face {} lies on the axis", cap.id));
        };
        let from_start = theta.min(TAU - theta);
        if from_start <= AZIMUTH_BAR && start_cap.is_none() {
            start_cap = Some(*cap);
        } else if (theta - sweep).abs() <= AZIMUTH_BAR && end_cap.is_none() {
            end_cap = Some(*cap);
        } else {
            return declined(format!(
                "plane face {} contains the axis at azimuth {theta:.9} rad, neither the revolve's \
                 start nor its end ({sweep:.9} rad)",
                cap.id
            ));
        }
    }
    let (Some(start_cap), Some(end_cap)) = (start_cap, end_cap) else {
        return declined("the source has no planar cap at both ends of the sweep");
    };
    if start_cap.loops.len() != 1 || end_cap.loops.len() != 1 {
        return declined("a cap carries more than one loop");
    }
    let profile_count = start_cap.loops[0].coedges.len();
    if end_cap.loops[0].coedges.len() != profile_count || faces.len() != profile_count + 2 {
        return declined(format!(
            "the source is not a plain revolve of one profile: {} faces for a {}-edge start cap \
             and a {}-edge end cap",
            faces.len(),
            profile_count,
            end_cap.loops[0].coedges.len()
        ));
    }
    let (start_edges, end_edges) = (face_edges(start_cap), face_edges(end_cap));
    for side in faces
        .iter()
        .filter(|candidate| candidate.id != start_cap.id && candidate.id != end_cap.id)
    {
        let edges = face_edges(side);
        if edges.is_disjoint(&start_edges) || edges.is_disjoint(&end_edges) {
            return declined(format!(
                "face {} does not run from one cap to the other, so the source is not a plain \
                 revolve",
                side.id
            ));
        }
    }

    // The profile, as the start cap's loop walks it.
    let profile = start_cap.loops[0]
        .coedges
        .iter()
        .map(|coedge| {
            let edge = edge_by_id
                .get(&coedge.edge_id)
                .ok_or_else(|| format!("offset_shell: missing edge {}", coedge.edge_id)).or_refuse(KernelStage::Refine, "offset_shell_missing_edge")?;
            Ok(LoopCurve {
                edge_id: edge.id,
                curve: edge.curve.clone(),
                t0: edge.t0,
                t1: edge.t1,
                forward: coedge.forward,
            })
        })
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let retained_edges = face_edges(face);
    let arc_indices = profile
        .iter()
        .enumerate()
        .filter(|(_, curve)| retained_edges.contains(&curve.edge_id))
        .map(|(index, _)| index)
        .collect::<Vec<_>>();
    let [arc_index] = arc_indices[..] else {
        return declined(format!(
            "the retained face meets the start cap along {} edges, not one",
            arc_indices.len()
        ));
    };
    let arc = &profile[arc_index];
    let samples = (0..=64)
        .map(|index| arc.at(index as f64 / 64.0))
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let Some((centre, skin, residual)) = fit_meridian_circle(&samples) else {
        return declined("the retained meridian is straight");
    };
    if residual > MERIDIAN_RESIDUAL_BAR {
        return declined(format!(
            "the retained meridian is not a circle: residual {residual:.3e} of its radius"
        ));
    }
    let delta = centre.sub(origin);
    let axial = delta.dot(axis);
    let planar = delta.sub(axis.scale(axial));
    let major = planar.length();
    if major <= skin + linear {
        return declined("the skin reaches the axis itself");
    }
    let radial = planar.scale(1.0 / major);
    if azimuth(centre).map_or(true, |theta| theta.min(TAU - theta) > AZIMUTH_BAR) {
        return declined("the start cap's arc is not in the revolve's start half-plane");
    }

    // The skin's material must be on the tube centre's side: the grown circle
    // is then `skin + |d|` about the same centre.
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let middle = face.surface.evaluate(um, vm).or_refuse(KernelStage::Refine, "evaluate")?;
    let sense = if face.same_sense { 1.0 } else { -1.0 };
    let normal = face.surface.normal(um, vm).or_refuse(KernelStage::Refine, "normal")?.scale(sense);
    let Some(outward) = outward_at(middle, origin, axis, major, axial) else {
        return declined("the retained face's mid sample sits on the axis");
    };
    if normal.dot(outward) <= 0.0 {
        return declined("the retained face's material is outside its tube");
    }
    let grown = skin - distance;

    // Every profile edge inside the tube disc: nothing else of the source can
    // reach the region between the skin and the grown circle.
    for curve in &profile {
        for index in 0..=32 {
            let point = curve.at(index as f64 / 32.0)?;
            if point.sub(centre).length() > skin + linear {
                return declined(format!(
                    "profile edge {} leaves the tube disc, so the source is not inside its skin",
                    curve.edge_id
                ));
            }
        }
    }

    let mut wedge = RevolvedWedge {
        origin,
        axis,
        radial,
        sweep,
        centre,
        major,
        axial,
        skin,
        grown,
        low: ArcEnd {
            angle: 0.0,
            grown_angle: 0.0,
            wall: 0,
        },
        high: ArcEnd {
            angle: 0.0,
            grown_angle: 0.0,
            wall: 0,
        },
        band: None,
        retained: face.id,
        start_cap: start_cap.id,
        end_cap: end_cap.id,
    };
    // The arc's own run, unwrapped, and which neighbour sits at which end.
    let mut run = 0.0;
    let mut previous = wedge.meridian_angle(samples[0]);
    let first = previous;
    for sample in &samples[1..] {
        let current = wedge.meridian_angle(*sample);
        let mut step = current - previous;
        while step > PI {
            step -= TAU;
        }
        while step < -PI {
            step += TAU;
        }
        run += step;
        previous = current;
    }
    let count = profile.len();
    let before = &profile[(arc_index + count - 1) % count];
    let after = &profile[(arc_index + 1) % count];
    let wall_of = |curve: &LoopCurve| -> Result<u64, KernelRefusal> {
        faces
            .iter()
            .find(|candidate| {
                candidate.id != start_cap.id && face_edges(candidate).contains(&curve.edge_id)
            })
            .map(|candidate| candidate.id)
            .ok_or_else(|| format!("offset_shell: profile edge {} bounds no side face", curve.edge_id))
            .or_refuse(KernelStage::Refine, "offset_shell_profile_edge_no_side_face")
    };
    // Each neighbour's far end, and the straightness that lets it be extended.
    let mut ends = Vec::with_capacity(2);
    for (curve, arrives) in [(before, true), (after, false)] {
        let (near, far) = if arrives {
            (curve.at(1.0)?, curve.at(0.0)?)
        } else {
            (curve.at(0.0)?, curve.at(1.0)?)
        };
        let length = near.sub(far).length();
        if length <= linear {
            return declined(format!("profile edge {} has no length", curve.edge_id));
        }
        let direction = near.sub(far).scale(1.0 / length);
        for index in 1..8 {
            let point = curve.at(index as f64 / 8.0)?;
            let off = point.sub(far);
            if off.sub(direction.scale(off.dot(direction))).length() > linear {
                return declined(format!(
                    "profile edge {} next to the retained arc is not straight",
                    curve.edge_id
                ));
            }
        }
        ends.push((near, direction, wall_of(curve)?));
    }
    let (start_end, end_end) = (ends[0], ends[1]);
    let (low_end, high_end, low_angle) = if run > 0.0 {
        (start_end, end_end, first)
    } else {
        (end_end, start_end, first + run)
    };
    let high_angle = low_angle + run.abs();

    // Extend each end's edge outward until it meets the grown circle.
    let mut extended = [(0.0, Vec3::default()); 2];
    for (slot, (near, direction, _)) in [low_end, high_end].into_iter().enumerate() {
        let offset = near.sub(centre);
        let along = direction.dot(offset);
        if along <= linear {
            return declined("a profile edge at the arc's end does not leave the tube outward");
        }
        let reach = -along + (along * along + grown * grown - offset.dot(offset)).sqrt();
        let meet = near.add(direction.scale(reach));
        if meet.sub(origin).dot(radial) <= linear {
            return declined("an extended profile edge reaches the axis");
        }
        let mut turn = wedge.meridian_angle(meet) - wedge.meridian_angle(near);
        while turn > PI {
            turn -= TAU;
        }
        while turn < -PI {
            turn += TAU;
        }
        extended[slot] = (turn, meet);
    }
    let segment = |a: Vec3, b: Vec3| {
        let flat = |point: Vec3| {
            let delta = point.sub(origin);
            (delta.dot(radial), delta.dot(axis))
        };
        (flat(a), flat(b))
    };
    if segments_meet(
        segment(low_end.0, extended[0].1),
        segment(high_end.0, extended[1].1),
        linear,
    ) {
        return declined("the two extended profile edges cross");
    }
    wedge.low = ArcEnd {
        angle: low_angle,
        grown_angle: low_angle + extended[0].0,
        wall: low_end.2,
    };
    wedge.high = ArcEnd {
        angle: high_angle,
        grown_angle: high_angle + extended[1].0,
        wall: high_end.2,
    };
    let span = wedge.high.grown_angle - wedge.low.grown_angle;
    if !(span > AZIMUTH_BAR && span < TAU) {
        return declined(format!("the grown arc's span {span:.9} rad is not a proper arc"));
    }

    // The fold: where the grown circle crosses the axis. Both ends of the grown
    // arc are off the axis, so a band copy that meets the arc lies wholly
    // inside it.
    let tangent = TANGENT_BAR * crate::solid_scale(source);
    if grown >= major - tangent {
        let fold = (-major / grown).clamp(-1.0, 1.0).acos();
        let width = if grown <= major + tangent {
            0.0
        } else {
            2.0 * (PI - fold)
        };
        let lead = if width == 0.0 { PI } else { fold };
        wedge.band = wrap_into(lead, wedge.low.grown_angle, wedge.high.grown_angle)
            .map(|start| [start, start + width]);
        if let Some([_, end]) = wedge.band {
            if end >= wedge.high.grown_angle {
                return declined("the fold band reaches the grown arc's end");
            }
        }
    }
    Ok(Recognition::Wedge(wedge))
}

/// Do two closed 2D segments share a point?
fn segments_meet(
    ((ax, ay), (bx, by)): ((f64, f64), (f64, f64)),
    ((cx, cy), (dx, dy)): ((f64, f64), (f64, f64)),
    linear: f64,
) -> bool {
    let (rx, ry) = (bx - ax, by - ay);
    let (sx, sy) = (dx - cx, dy - cy);
    let denominator = rx * sy - ry * sx;
    let (qx, qy) = (cx - ax, cy - ay);
    let r_length = rx.hypot(ry);
    let s_length = sx.hypot(sy);
    if denominator.abs() <= LINEAR_BAR * r_length * s_length {
        // Parallel. Collinear segments meet where their projections overlap.
        if (qx * ry - qy * rx).abs() > linear * r_length {
            return false;
        }
        let project = |x: f64, y: f64| (x * rx + y * ry) / (r_length * r_length);
        let (c, d) = (project(qx, qy), project(dx - ax, dy - ay));
        let (low, high) = (c.min(d), c.max(d));
        let slack = linear / r_length;
        return high >= -slack && low <= 1.0 + slack;
    }
    let t = (qx * sy - qy * sx) / denominator;
    let u = (qx * ry - qy * rx) / denominator;
    let (t_slack, u_slack) = (linear / r_length, linear / s_length);
    t >= -t_slack && t <= 1.0 + t_slack && u >= -u_slack && u <= 1.0 + u_slack
}

/// Build the half-plane region and revolve it through the sweep.
pub(super) fn build(
    wedge: &RevolvedWedge,
    source: &BrepSolid,
) -> Result<OffsetShellResultRecord, KernelRefusal> {
    const SKIN: &str = "skin";
    const WALL_LOW: &str = "wall-low";
    const WALL_HIGH: &str = "wall-high";
    const GROWN: &str = "grown";
    const AXIS: &str = "axis";
    const CAP_START: &str = "cap-start";
    const CAP_END: &str = "cap-end";
    let (radial, axis) = (wedge.radial, wedge.axis);
    let arc = |radius: f64, from: f64, to: f64| make_arc(wedge.centre, radial, axis, radius, from, to);
    let mut profile: Vec<NurbsCurve> = Vec::with_capacity(6);
    let mut tags: Vec<&str> = Vec::with_capacity(6);
    // The skin, from the arc's low end to its high end.
    profile.push(arc(wedge.skin, wedge.low.angle, wedge.high.angle).or_refuse(KernelStage::Refine, "arc")?);
    tags.push(SKIN);
    // Out along the high end's profile edge to the grown circle.
    profile.push(make_line(
        wedge.at(wedge.skin, wedge.high.angle),
        wedge.at(wedge.grown, wedge.high.grown_angle),
    ).or_refuse(KernelStage::Refine, "make_line")?);
    tags.push(WALL_HIGH);
    // Back along the grown circle, stopping at the axis if it reaches it.
    let pinch = |angle: f64| {
        wedge
            .origin
            .add(axis.scale(wedge.axial + wedge.grown * angle.sin()))
    };
    match wedge.band {
        Some([band_start, band_end]) => {
            profile.push(arc(wedge.grown, band_end, wedge.high.grown_angle).or_refuse(KernelStage::Refine, "arc")?.reversed().or_refuse(KernelStage::Refine, "reversed")?);
            tags.push(GROWN);
            if band_end > band_start {
                profile.push(make_line(pinch(band_end), pinch(band_start)).or_refuse(KernelStage::Refine, "make_line")?);
                tags.push(AXIS);
            }
            profile.push(arc(wedge.grown, wedge.low.grown_angle, band_start).or_refuse(KernelStage::Refine, "arc")?.reversed().or_refuse(KernelStage::Refine, "reversed")?);
            tags.push(GROWN);
        }
        None => {
            profile.push(
                arc(wedge.grown, wedge.low.grown_angle, wedge.high.grown_angle).or_refuse(KernelStage::Refine, "arc")?.reversed().or_refuse(KernelStage::Refine, "reversed")?,
            );
            tags.push(GROWN);
        }
    }
    // And in along the low end's profile edge to the skin.
    profile.push(make_line(
        wedge.at(wedge.grown, wedge.low.grown_angle),
        wedge.at(wedge.skin, wedge.low.angle),
    ).or_refuse(KernelStage::Refine, "make_line")?);
    tags.push(WALL_LOW);

    let side_names = tags
        .iter()
        .map(|tag| Some(tag.to_string()))
        .collect::<Vec<_>>();
    let cap_names = vec![Some(CAP_START.to_string()), Some(CAP_END.to_string())];
    let mut solid = crate::revolve_profile_brep_named(
        &profile,
        wedge.origin,
        axis,
        wedge.sweep,
        &side_names,
        &cap_names,
    )
    .map_err(|error| {
        format!("offset_shell: the revolved wedge's half-plane region could not be revolved: {error}")
    }).or_refuse(KernelStage::Refine, "revolve_profile_brep_named")?;

    // Provenance, positionally parallel to the face order, and the names the
    // pipeline's own assembly would give: a source face keeps its name, an
    // offset appends `_Offset`, a wall takes its opening's, duplicates `_1`, ….
    let source_names = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| (face.id, face.name.clone()))
        .collect::<HashMap<_, _>>();
    let mut used = HashSet::<String>::default();
    let mut face_images = Vec::new();
    for face in solid.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        let tag = face.name.take().unwrap_or_default();
        let (role, source_face_id) = match tag.as_str() {
            SKIN => (OffsetFaceRole::Source, wedge.retained),
            GROWN => (OffsetFaceRole::Offset, wedge.retained),
            WALL_LOW => (OffsetFaceRole::Wall, wedge.low.wall),
            WALL_HIGH => (OffsetFaceRole::Wall, wedge.high.wall),
            CAP_START => (OffsetFaceRole::Wall, wedge.start_cap),
            CAP_END => (OffsetFaceRole::Wall, wedge.end_cap),
            other => {
                return Err(KernelRefusal::internal(KernelStage::Refine, "offset_shell_wedge_attribution", format!(
                    "offset_shell: the revolved wedge built a face it cannot attribute ({other:?})"
                )))
            }
        };
        face.name = source_names
            .get(&source_face_id)
            .cloned()
            .flatten()
            .map(|base| {
                let base = if role == OffsetFaceRole::Offset {
                    format!("{base}_Offset")
                } else {
                    base
                };
                if used.insert(base.clone()) {
                    return base;
                }
                (1usize..)
                    .map(|suffix| format!("{base}_{suffix}"))
                    .find(|candidate| used.insert(candidate.clone()))
                    .expect("an unbounded suffix search ends")
            });
        face_images.push(OffsetShellFaceImageRecord {
            role,
            source_face_id,
        });
    }

    // The construction checks itself against the face it was built from: the
    // source skin's own mid sample is on the result's boundary. A revolve turned
    // the wrong way, or a region taken on the wrong side of the skin, fails here
    // rather than leaving the lane as a validating solid in the wrong place.
    let skin_face = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == wedge.retained)
        .ok_or_else(|| "offset_shell: the retained face went missing".to_string()).or_refuse(KernelStage::Refine, "offset_shell_the_retained_face_went_missing")?;
    let [u0, u1] = skin_face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = skin_face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let probe = skin_face
        .surface
        .evaluate(0.5 * (u0 + u1), 0.5 * (v0 + v1)).or_refuse(KernelStage::Refine, "evaluate")?;
    let scale = crate::solid_scale(source);
    let class = classify_point(probe, &solid, LINEAR_BAR * scale * 10.0).or_refuse(KernelStage::Refine, "classify_point")?.class;
    if class != PointClass::On {
        return Err(KernelRefusal::internal(KernelStage::Refine, "offset_shell_wedge_skin", format!(
            "offset_shell: the revolved wedge does not carry its own source skin — the skin's mid \
             sample classifies {class:?} against it ({})",
            wedge.report()
        )));
    }
    let solid = crate::accept_sound(solid, "offsetShell")?;
    Ok(OffsetShellResultRecord { solid, face_images })
}
