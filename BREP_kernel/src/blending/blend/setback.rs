//! Asymmetric chamfer stations: per-face CHORDAL setbacks in the edge's
//! normal plane.
//!
//! A two-distance chamfer puts its rail on face `i` at the point `P_i` of
//! that face which lies in the normal plane of the edge at the spine point
//! `C` and whose chord to `C` is `d_i` long:
//!
//! ```text
//!   (P_i − C)·T = 0,   |P_i − C| = d_i
//! ```
//!
//! (OpenCASCADE's two-distance chamfer is the same chord: on a half cylinder
//! of radius 10 its rail sits at chord 3, not arc 3, from the edge.)  The two
//! faces solve independently, two unknowns each.  A distance-angle chamfer
//! keeps face 1's equation and puts face 2's point on the ray leaving `P_1`
//! at the given angle from face 1's TANGENT at `C` (the section's direction
//! into face 1), tilted toward face 2.
//!
//! The stations replace the rolling ball's in the closed rim's march; the
//! fit, the support pcurves and the §6.9 surgery are the rim's own, with the
//! chamfer's linear section.  The law is active only inside
//! [`with_setback_law`], which the asymmetric chamfer entries open.

use std::cell::Cell;

use super::stations::{edge_uv_on_face, BlendMate, Station};
use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::{KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec3};

/// What the chamfer measures on the two faces it joins, keyed by face id.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum SetbackLaw {
    /// Chord `distances[k]` from the spine point on the face `faces[k]`.
    Distances { faces: [u64; 2], distances: [f64; 2] },
    /// Chord `distance` on the face `first`; the bevel leaves that face's
    /// tangent at the spine point at `angle` (radians), toward the other face.
    Angle { first: u64, distance: f64, angle: f64 },
}

impl SetbackLaw {
    /// The larger of the two setbacks (the angle law's second is not known
    /// before the section is solved; its first stands in).  The march reads
    /// it only as a size: its scale and the edge-length probes.
    pub(crate) fn reach(&self) -> f64 {
        match *self {
            Self::Distances { distances, .. } => distances[0].max(distances[1]),
            Self::Angle { distance, .. } => distance,
        }
    }
}

thread_local! {
    static LAW: Cell<Option<SetbackLaw>> = const { Cell::new(None) };
}

/// Run `action` with `law` governing the closed rim's stations.
pub(crate) fn with_setback_law<T>(law: SetbackLaw, action: impl FnOnce() -> T) -> T {
    struct Restore(Option<SetbackLaw>);
    impl Drop for Restore {
        fn drop(&mut self) {
            LAW.with(|slot| slot.set(self.0));
        }
    }
    let _restore = Restore(LAW.with(|slot| slot.replace(Some(law))));
    action()
}

/// The law in force on this thread, if any.
pub(super) fn active() -> Option<SetbackLaw> {
    LAW.with(Cell::get)
}

/// The refusal for a rim the setback stations cannot take.
pub(super) fn unsupported(what: &'static str, message: impl Into<String>) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, message.into())
}

const NEWTON_ITERATIONS: usize = 40;

/// How one face's rail point is pinned in the section, besides lying in it.
#[derive(Clone, Copy)]
enum Pin {
    /// `|P − C| = d`.
    Chord(f64),
    /// `(P − origin)·normal = 0`: on the line through `origin` in the
    /// section plane whose in-plane normal is `normal`.
    Ray { origin: Vec3, normal: Vec3 },
}

fn wrapped(face: &FaceRecord, mut uv: [f64; 2]) -> [f64; 2] {
    if let Ok((closed_u, closed_v)) = face.surface.closed_directions() {
        for (index, closed, domain) in [(0, closed_u, face.surface.domain_u()), (1, closed_v, face.surface.domain_v())] {
            if let (true, Ok([low, high])) = (closed, domain) {
                uv[index] = low + (uv[index] - low).rem_euclid(high - low);
            }
        }
    }
    uv
}

fn inside(face: &FaceRecord, uv: [f64; 2]) -> bool {
    let uv = wrapped(face, uv);
    crate::parameter_point_in_face(face, crate::Vec2 { x: uv[0], y: uv[1] }, 1e-9)
        .map(|class| class != crate::PolygonClass::Outside)
        .unwrap_or(false)
}

/// The surface point and its two partials.
fn frame(surface: &NurbsSurface, uv: [f64; 2]) -> Result<(Vec3, Vec3, Vec3), KernelRefusal> {
    surface.deriv1_extended(uv[0], uv[1]).or_refuse(KernelStage::Refine, "setback_surface")
}

/// The (u, v) step whose image in the tangent plane is closest to `along`.
fn tangent_step(su: Vec3, sv: Vec3, along: Vec3) -> Option<[f64; 2]> {
    let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let det = a * c - b * b;
    if !(det.abs() > 1e-300) {
        return None;
    }
    let (p, q) = (su.dot(along), sv.dot(along));
    Some([(c * p - b * q) / det, (a * q - b * p) / det])
}

/// The unit direction INTO `face` at its edge point `uv`, square to the
/// edge tangent `tangent`: the tangent-plane direction whose small step
/// stays inside the face's trim.
fn into_face(face: &FaceRecord, uv: [f64; 2], tangent: Vec3, probe: f64) -> Result<Vec3, KernelRefusal> {
    let (_, su, sv) = frame(&face.surface, uv)?;
    let normal = su.cross(sv).normalized().or_refuse(KernelStage::Refine, "setback_normal")?;
    let across = normal.cross(tangent).normalized().or_refuse(KernelStage::Refine, "setback_across")?;
    let mut verdicts = [false; 2];
    for (slot, sign) in [1.0, -1.0].into_iter().enumerate() {
        if let Some(step) = tangent_step(su, sv, across.scale(sign * probe)) {
            verdicts[slot] = inside(face, [uv[0] + step[0], uv[1] + step[1]]);
        }
    }
    match verdicts {
        [true, false] => Ok(across),
        [false, true] => Ok(across.scale(-1.0)),
        _ => Err(unsupported(
            "setback_side",
            format!("asymmetric chamfer: cannot tell which side of the edge face {} lies on", face.id),
        )),
    }
}

/// `uv` shifted by whole periods of its carrier's closed directions to sit
/// nearest `reference`.
fn nearest_image(surface: &NurbsSurface, mut uv: [f64; 2], reference: [f64; 2]) -> [f64; 2] {
    if let Ok((closed_u, closed_v)) = surface.closed_directions() {
        for (index, closed, domain) in [(0, closed_u, surface.domain_u()), (1, closed_v, surface.domain_v())] {
            if let (true, Ok([low, high])) = (closed, domain) {
                let period = high - low;
                uv[index] -= ((uv[index] - reference[index]) / period).round() * period;
            }
        }
    }
    uv
}

/// Newton for one face's rail point in the section plane at `(spine, tangent)`.
fn solve_side(
    surface: &NurbsSurface,
    seed: [f64; 2],
    spine: Vec3,
    tangent: Vec3,
    pin: Pin,
    tolerance: f64,
) -> Result<([f64; 2], Vec3), KernelRefusal> {
    let mut uv = seed;
    for _ in 0..NEWTON_ITERATIONS {
        let (point, su, sv) = frame(surface, uv)?;
        let chord = point.sub(spine);
        let plane = chord.dot(tangent);

        let (second, gu, gv) = match pin {
            Pin::Chord(distance) => {
                let length = chord.length();
                if !(length > 1e-300) {
                    return Err(KernelRefusal::non_convergence(KernelStage::Refine, "setback_on_edge", "asymmetric chamfer: a rail point fell onto the edge"));
                }
                (length - distance, chord.dot(su) / length, chord.dot(sv) / length)
            }
            Pin::Ray { origin, normal } => (point.sub(origin).dot(normal), su.dot(normal), sv.dot(normal)),
        };
        if plane.abs().max(second.abs()) <= tolerance {
            return Ok((uv, point));
        }
        let (a, b, c, d) = (su.dot(tangent), sv.dot(tangent), gu, gv);
        let det = a * d - b * c;
        if !(det.abs() > 1e-300) || !det.is_finite() {
            break;
        }
        uv[0] -= (d * plane - b * second) / det;
        uv[1] -= (a * second - c * plane) / det;
        if !uv[0].is_finite() || !uv[1].is_finite() {
            break;
        }
    }
    Err(KernelRefusal::non_convergence(
        KernelStage::Refine,
        "setback_station",
        "asymmetric chamfer: a rail point did not converge in its section",
    ))
}

/// The section frame and both faces' into-directions at `t`, for the
/// first station of a march (and the angle law at every station).
struct Section {
    spine: Vec3,
    tangent: Vec3,
}

fn section(edge: &EdgeRecord, t: f64) -> Result<Section, KernelRefusal> {
    let (spine, tangent) = edge
        .curve
        .point_and_unit_tangent_extended(t, edge.t0, edge.t1)
        .or_refuse(KernelStage::Refine, "setback_section")?;
    Ok(Section { spine, tangent })
}

/// Seed `uv` (on the edge) stepped `distance` along `direction` in the
/// tangent plane, refined by a few chord-length corrections.
fn displaced_seed(surface: &NurbsSurface, uv: [f64; 2], direction: Vec3, distance: f64) -> Result<[f64; 2], KernelRefusal> {
    let (_, su, sv) = frame(surface, uv)?;
    let step = tangent_step(su, sv, direction.scale(distance))
        .ok_or_else(|| unsupported("setback_seed", "asymmetric chamfer: a degenerate face chart at the edge"))?;
    Ok([uv[0] + step[0], uv[1] + step[1]])
}

/// The mates' setback pins, in the march's mate order.
fn mate_distances(law: &SetbackLaw, first: &BlendMate, second: &BlendMate) -> Result<[Option<f64>; 2], KernelRefusal> {
    let ids = [first.face.id, second.face.id];
    match *law {
        SetbackLaw::Distances { faces, distances } => {
            let of = |id: u64| faces.iter().position(|face| *face == id).map(|index| distances[index]);
            match (of(ids[0]), of(ids[1])) {
                (Some(a), Some(b)) if ids[0] != ids[1] => Ok([Some(a), Some(b)]),
                _ => Err(unsupported(
                    "setback_faces",
                    format!("asymmetric chamfer: the rim's faces {ids:?} are not the faces the distances name {faces:?}"),
                )),
            }
        }
        SetbackLaw::Angle { first: id, distance, .. } => {
            if ids[0] == id {
                Ok([Some(distance), None])
            } else if ids[1] == id {
                Ok([None, Some(distance)])
            } else {
                Err(unsupported(
                    "setback_faces",
                    format!("asymmetric chamfer: the rim's faces {ids:?} do not include face {id} the distance names"),
                ))
            }
        }
    }
}

/// The setback stations' solver state along one edge.
struct Marcher<'a> {
    edge: &'a EdgeRecord,
    mates: [&'a BlendMate<'a>; 2],
    distances: [Option<f64>; 2],
    angle: f64,
    tolerance: f64,
}

impl Marcher<'_> {
    /// `t` brought back onto the edge: a closed edge's parameter wraps (the
    /// anchored march runs once round from its seam crossing), an open one's
    /// clamps.
    fn on_edge_parameter(&self, t: f64) -> f64 {
        let (t0, t1) = (self.edge.t0, self.edge.t1);
        if self.edge.start_vertex_id == self.edge.end_vertex_id && t1 > t0 {
            t0 + (t - t0).rem_euclid(t1 - t0)
        } else {
            t.clamp(t0.min(t1), t0.max(t1))
        }
    }

    /// One station at `t` from the previous station's (u, v), or (first
    /// station) from the edge's own pcurves displaced into each face.
    fn station(&self, t: f64, previous: Option<[[f64; 2]; 2]>) -> Result<Station, KernelRefusal> {
        let Section { spine, tangent } = section(self.edge, t)?;
        let surfaces = [&self.mates[0].face.surface, &self.mates[1].face.surface];
        // The into-directions at the spine are needed for the first seeds and
        // for the angle law's ray at every station.
        let need_into = previous.is_none() || self.distances.iter().any(Option::is_none);
        let mut into = [Vec3::default(); 2];
        let mut on_edge = [[0.0; 2]; 2];
        if need_into {
            let probe = 1e-4 * self.distances.iter().flatten().fold(0.0f64, |a, b| a.max(*b)).max(1e-6);
            let past_vertex = self.edge.start_vertex_id != self.edge.end_vertex_id && self.on_edge_parameter(t) != t;
            for k in 0..2 {
                on_edge[k] = edge_uv_on_face(self.mates[k].coedge, self.edge, self.on_edge_parameter(t))?;
                into[k] = match previous {
                    // Past an open edge's vertex the trim cannot be probed at
                    // the spine: the carrier's own across-direction at the
                    // spine's foot, on the side its previous rail point lies.
                    Some(uv) if past_vertex => {
                        let at = foot(surfaces[k], uv[k], spine, self.tolerance)?;
                        let across = unit_normal(surfaces[k], at)?
                            .cross(tangent)
                            .normalized()
                            .or_refuse(KernelStage::Refine, "setback_across")?;
                        let (rail, _, _) = frame(surfaces[k], uv[k])?;
                        if across.dot(rail.sub(spine)) >= 0.0 { across } else { across.scale(-1.0) }
                    }
                    _ => into_face(self.mates[k].face, on_edge[k], tangent, probe)?,
                };
            }
        }
        let mut uv = [[0.0; 2]; 2];
        let mut points = [Vec3::default(); 2];
        // The chord-pinned face first: the angle law's ray starts at its point.
        let order = if self.distances[0].is_some() { [0, 1] } else { [1, 0] };
        for k in order {
            let pin = match self.distances[k] {
                Some(distance) => Pin::Chord(distance),
                None => {
                    let other = 1 - k;
                    let e1 = into[other];
                    let e2 = into[k].sub(e1.scale(into[k].dot(e1))).normalized().or_refuse(KernelStage::Refine, "setback_angle_basis")?;
                    let ray = e1.scale(-self.angle.cos()).add(e2.scale(self.angle.sin()));
                    Pin::Ray { origin: points[other], normal: tangent.cross(ray).normalized().or_refuse(KernelStage::Refine, "setback_ray")? }
                }
            };
            let seed = match previous {
                Some(uv) => uv[k],
                None => {
                    let guess = match pin {
                        Pin::Chord(distance) => distance,
                        // The planar section's d2 (sine rule) seeds the ray.
                        Pin::Ray { .. } => {
                            let other = self.distances[1 - k].unwrap_or(0.0);
                            let theta = into[0].dot(into[1]).clamp(-1.0, 1.0).acos();
                            let gamma = std::f64::consts::PI - theta - self.angle;
                            if gamma > 1e-6 { other * self.angle.sin() / gamma.sin() } else { other }
                        }
                    };
                    displaced_seed(surfaces[k], on_edge[k], into[k], guess)?
                }
            };
            let (mut solved, point) = solve_side(surfaces[k], seed, spine, tangent, pin, self.tolerance)?;
            // A closed carrier answers on any periodic image; the track
            // stays on the one nearest its last station.
            if let Some(previous) = previous {
                solved = nearest_image(&self.mates[k].face.surface, solved, previous[k]);
            }
            uv[k] = solved;
            points[k] = point;
        }
        // The twin root on the far side of the edge is not the chamfer's.
        if need_into {
            for k in 0..2 {
                if !(points[k].sub(spine).dot(into[k]) > 0.0) {
                    return Err(unsupported(
                        "setback_far_side",
                        format!("asymmetric chamfer: the rail on face {} solved on the far side of the edge", self.mates[k].face.id),
                    ));
                }
            }
        }
        Ok(Station {
            uv1: uv[0],
            uv2: uv[1],
            p1: points[0],
            p2: points[1],
            center: spine,
            weight: 1.0,
            apex: points[0].add(points[1]).scale(0.5),
        })
    }

    /// Stations at `count + 1` uniform parameters, each seeded by the last.
    fn uniform(&self, from: f64, span: f64, count: usize) -> Result<Vec<Station>, KernelRefusal> {
        let mut stations: Vec<Station> = Vec::with_capacity(count + 1);
        for index in 0..=count {
            let t = from + span * index as f64 / count as f64;
            let previous = stations.last().map(|station| [station.uv1, station.uv2]);
            stations.push(self.station(t, previous)?);
        }
        Ok(stations)
    }
}

/// The distance from `point` to the rail, by Newton on the foot parameter
/// from `u` (a geometric miss: the rail's parameterisation is not the
/// edge's, so `|rail(u) − point|` would also read the slip along the rail).
fn distance_to_rail(rail: &NurbsCurve, point: Vec3, mut u: f64, domain: [f64; 2]) -> Result<f64, KernelRefusal> {
    let h = 1e-6 * (domain[1] - domain[0]);
    for _ in 0..8 {
        let at = |u: f64| rail.evaluate(u.clamp(domain[0], domain[1])).or_refuse(KernelStage::Refine, "setback_fit_eval");
        let (here, ahead, behind) = (at(u)?, at(u + h)?, at(u - h)?);
        let d1 = ahead.sub(behind).scale(0.5 / h);
        let gap = here.sub(point);
        let slope = d1.dot(d1);
        if !(slope.abs() > 1e-300) {
            break;
        }
        let step = gap.dot(d1) / slope;
        u = (u - step).clamp(domain[0], domain[1]);
        if step.abs() <= 1e-15 * (domain[1] - domain[0]) {
            break;
        }
    }
    Ok(rail.evaluate(u).or_refuse(KernelStage::Refine, "setback_fit_eval")?.sub(point).length())
}

/// The largest distance between the rails fitted through `stations` (solved
/// at the increasing edge parameters `ts`) and the stations solved halfway
/// between them.
fn fit_miss(marcher: &Marcher, ts: &[f64], stations: &[Station]) -> Result<f64, KernelRefusal> {
    let count = stations.len() - 1;
    let (first, last) = (ts[0], ts[count]);
    let fraction_of = |t: f64| (t - first) / (last - first);
    let fit = |points: Vec<Vec3>| -> Result<NurbsCurve, KernelRefusal> {
        let parameters: Vec<f64> = ts.iter().map(|t| fraction_of(*t)).collect();
        let samples: Vec<crate::Vec4> = points.into_iter().map(|p| crate::Vec4::from_point(p, 1.0)).collect();
        crate::fit::interpolate_homogeneous(&samples, 3, &parameters).or_refuse(KernelStage::Refine, "setback_fit")
    };
    let rails = [fit(stations.iter().map(|s| s.p1).collect())?, fit(stations.iter().map(|s| s.p2).collect())?];
    let mut miss = 0.0f64;
    for index in 0..count {
        let t = 0.5 * (ts[index] + ts[index + 1]);
        let previous = [stations[index].uv1, stations[index].uv2];
        let mid = marcher.station(t, Some(previous))?;
        let domain = rails[0].domain().or_refuse(KernelStage::Refine, "setback_fit_domain")?;
        let u = domain[0] + (domain[1] - domain[0]) * fraction_of(t);
        for (rail, point) in rails.iter().zip([mid.p1, mid.p2]) {
            miss = miss.max(distance_to_rail(rail, point, u, domain)?);
        }
    }
    Ok(miss)
}

/// The parameter where face 1's rail crosses the meridian `u = lock`, from a
/// station solved at `seed_t`: Newton on (t, v) with u frozen, face 1's pin.
fn anchored_t(marcher: &Marcher, seed_t: f64, lock: f64) -> Result<f64, KernelRefusal> {
    let seed = marcher.station(seed_t, None)?;
    let surface = &marcher.mates[0].face.surface;
    let pin = |t: f64, v: f64| -> Result<[f64; 2], KernelRefusal> {
        let Section { spine, tangent } = section(marcher.edge, t)?;
        let (point, _, _) = frame(surface, [lock, v])?;
        let chord = point.sub(spine);
        let distance = marcher.distances[0].ok_or_else(|| unsupported("setback_anchor", "asymmetric chamfer: an anchored rim needs its seam face chord-pinned"))?;
        Ok([chord.dot(tangent), chord.length() - distance])
    };
    let (mut t, mut v) = (seed_t, seed.uv1[1]);
    let span = marcher.edge.t1 - marcher.edge.t0;
    for _ in 0..NEWTON_ITERATIONS {
        let residual = pin(t, v)?;
        if residual[0].abs().max(residual[1].abs()) <= marcher.tolerance {
            return Ok(t);
        }
        let (ht, hv) = (1e-7 * span.abs().max(1e-12), 1e-7);
        let at_t = pin(t + ht, v)?;
        let at_v = pin(t, v + hv)?;
        let (a, b) = ((at_t[0] - residual[0]) / ht, (at_v[0] - residual[0]) / hv);
        let (c, d) = ((at_t[1] - residual[1]) / ht, (at_v[1] - residual[1]) / hv);
        let det = a * d - b * c;
        if !(det.abs() > 1e-300) {
            break;
        }
        t -= (d * residual[0] - b * residual[1]) / det;
        v -= (a * residual[1] - c * residual[0]) / det;
    }
    Err(KernelRefusal::non_convergence(KernelStage::Refine, "setback_anchor", "asymmetric chamfer: the rail's seam crossing did not converge"))
}

/// The closed rim's stations under the setback law: uniform in the edge
/// parameter from the rail's crossing of face 1's seam meridian (`anchor_u`,
/// when the rim's loop is seam-structured), doubled until cubic rails
/// through them hold the solved mid-stations to `rail_bar`; the last
/// station carries the carriers' period.
pub(super) fn march_closed(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    law: &SetbackLaw,
    anchor_u: Option<f64>,
    rail_bar: f64,
) -> Result<Vec<Station>, KernelRefusal> {
    let distances = mate_distances(law, first, second)?;
    let angle = match *law {
        SetbackLaw::Angle { angle, .. } => angle,
        SetbackLaw::Distances { .. } => 0.0,
    };
    let scale = crate::solid_scale(solid).max(1.0);
    let marcher = Marcher { edge, mates: [first, second], distances, angle, tolerance: 1e-13 * scale };
    let span = edge.t1 - edge.t0;
    let (from, lock) = match anchor_u {
        None => (edge.t0, None),
        Some(base) => {
            // Seed where the edge's own pcurve on face 1 is nearest the
            // meridian (any periodic image of it).
            let period = {
                let [u0, u1] = first.face.surface.domain_u().or_refuse(KernelStage::Refine, "setback_domain")?;
                u1 - u0
            };
            let mut best = (edge.t0, f64::INFINITY, base);
            for index in 0..=64 {
                let t = edge.t0 + span * index as f64 / 64.0;
                let uv = edge_uv_on_face(first.coedge, edge, t)?;
                let lock = base + ((uv[0] - base) / period).round() * period;
                if (uv[0] - lock).abs() < best.1 {
                    best = (t, (uv[0] - lock).abs(), lock);
                }
            }
            (anchored_t(&marcher, best.0, best.2)?, Some((best.2, period)))
        }
    };
    // The march's own rail bar, and no looser than 1e-9 of the model scale:
    // the rails bound the removed volume, which is read to 1e-9 of the part.
    let bar = rail_bar.min(1e-9 * scale);
    let mut count = 32usize;
    let mut misses = Vec::new();
    loop {
        let mut stations = marcher.uniform(from, span, count)?;
        let ts: Vec<f64> = (0..=count).map(|index| from + span * index as f64 / count as f64).collect();
        let miss = fit_miss(&marcher, &ts, &stations)?;
        misses.push(miss);
        if std::env::var_os("BREP_DEBUG_SETBACK").is_some() {
            eprintln!("setback march: {} stations, rail miss {miss:.3e} (bar {bar:.3e})", stations.len());
        }
        crate::blend::record_count("count.setback.stations", stations.len() as u64);
        if miss <= bar {
            if std::env::var_os("BREP_DEBUG_SETBACK").is_some() {
                for (label, mate) in [("first", first), ("second", second)] {
                    eprintln!(
                        "setback mate {label}: face {} closed {:?} u {:?} v {:?} analytic {:?}",
                        mate.face.id,
                        mate.face.surface.closed_directions(),
                        mate.face.surface.domain_u(),
                        mate.face.surface.domain_v(),
                        mate.face.surface.analytic().map(|a| a.kind_label())
                    );
                }
                let n = stations.len() - 1;
                for i in [0, 1, n / 4, n / 2, n - 1, n] {
                    let st = &stations[i];
                    eprintln!("setback station {i}: uv1 {:?} uv2 {:?} p1 {:?} p2 {:?}", st.uv1, st.uv2, st.p1, st.p2);
                }
                eprintln!("setback anchor {lock:?} from t {from}");
            }
            // Each track as a whole on the periodic image of its face's edge
            // pcurve at the start station (the image the surgery trims on).
            for (k, mate) in [first, second].into_iter().enumerate() {
                let reference = edge_uv_on_face(mate.coedge, edge, marcher.on_edge_parameter(from))?;
                let start = if k == 0 { stations[0].uv1 } else { stations[0].uv2 };
                let moved = nearest_image(&mate.face.surface, start, reference);
                let shift = [moved[0] - start[0], moved[1] - start[1]];
                if shift != [0.0, 0.0] {
                    for station in &mut stations {
                        let uv = if k == 0 { &mut station.uv1 } else { &mut station.uv2 };
                        uv[0] += shift[0];
                        uv[1] += shift[1];
                    }
                }
            }
            if let Some((lock, period)) = lock {
                // Both ends ON the seam meridian, the last one period on.
                // The meridian's image the march started on, and the laps
                // the march itself wound.
                let last = stations.len() - 1;
                let start = lock + ((stations[0].uv1[0] - lock) / period).round() * period;
                let laps = ((stations[last].uv1[0] - start) / period).round();
                stations[0].uv1[0] = start;
                stations[last].uv1[0] = start + laps * period;
            }
            return Ok(stations);
        }
        if count >= 4096 {
            return Err(KernelRefusal::non_convergence(
                KernelStage::Refine,
                "setback_rails",
                format!("asymmetric chamfer: {} stations still leave the rails {miss:.3e} off the solved sections (bar {bar:.3e}; by doubling {misses:?})", stations.len()),
            ));
        }
        count *= 2;
    }
}

/// The open edge's stations under the setback law, at the parameters
/// `parameters(count)` gives for `count` intervals (the open march's own,
/// overshoot past both vertices included): solved from the middle station
/// (the best-conditioned, inside the edge) outward, each seeded by its
/// neighbour, and doubled from `count` until cubic rails through them hold
/// the solved mid-stations to `rail_bar` (no looser than 1e-9 of the model
/// scale).  Returns the stations and the parameters' vertex indices.
pub(super) fn march_open(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    law: &SetbackLaw,
    mut count: usize,
    parameters: &dyn Fn(usize) -> (Vec<f64>, [usize; 2]),
    rail_bar: f64,
) -> Result<(Vec<Station>, [usize; 2]), KernelRefusal> {
    let distances = mate_distances(law, first, second)?;
    let angle = match *law {
        SetbackLaw::Angle { angle, .. } => angle,
        SetbackLaw::Distances { .. } => 0.0,
    };
    let scale = crate::solid_scale(solid).max(1.0);
    let marcher = Marcher { edge, mates: [first, second], distances, angle, tolerance: 1e-13 * scale };
    let bar = rail_bar.min(1e-9 * scale);
    let mut misses = Vec::new();
    loop {
        let (ts, vertex_indices) = parameters(count);
        let middle = ts.len() / 2;
        let mut solved: Vec<Option<Station>> = vec![None; ts.len()];
        solved[middle] = Some(marcher.station(ts[middle], None)?);
        for index in (0..middle).rev() {
            let next = solved[index + 1].expect("solved outward");
            solved[index] = Some(marcher.station(ts[index], Some([next.uv1, next.uv2]))?);
        }
        for index in middle + 1..ts.len() {
            let previous = solved[index - 1].expect("solved outward");
            solved[index] = Some(marcher.station(ts[index], Some([previous.uv1, previous.uv2]))?);
        }
        let stations: Vec<Station> = solved.into_iter().flatten().collect();
        let miss = fit_miss(&marcher, &ts, &stations)?;
        misses.push(miss);
        if std::env::var_os("BREP_DEBUG_SETBACK").is_some() {
            eprintln!("setback open march: {} stations, rail miss {miss:.3e} (bar {bar:.3e})", stations.len());
        }
        crate::blend::record_count("count.setback.stations", stations.len() as u64);
        if miss <= bar {
            return Ok((stations, vertex_indices));
        }
        if count >= 4096 {
            return Err(KernelRefusal::non_convergence(
                KernelStage::Refine,
                "setback_rails",
                format!("asymmetric chamfer: {} stations still leave the rails {miss:.3e} off the solved sections (bar {bar:.3e}; by doubling {misses:?})", stations.len()),
            ));
        }
        count *= 2;
    }
}

/// The law resolved to the SIDES of a smooth chain: the distance (or the
/// angle law's missing one, `None`) for every segment's first and second
/// mate.  A chain keeps its first side along its segments while the faces
/// change, so the faces the law names are read where they occur and the
/// side carries the setback on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct SideLaw {
    distances: [Option<f64>; 2],
    angle: f64,
}

thread_local! {
    static SIDES: Cell<Option<SideLaw>> = const { Cell::new(None) };
}

/// Restores the previous side law when dropped.
pub(super) struct SidesGuard(Option<SideLaw>, bool);

impl Drop for SidesGuard {
    fn drop(&mut self) {
        if self.1 {
            SIDES.with(|slot| slot.set(self.0));
        }
    }
}

/// Under an active law, resolve it on the chain whose segments join the
/// face pairs `pairs` (first mate, second mate) and put it in force for the
/// chain's stations until the guard drops; without a law, nothing.
pub(super) fn chain_sides(pairs: &[[u64; 2]]) -> Result<SidesGuard, KernelRefusal> {
    let Some(law) = active() else {
        return Ok(SidesGuard(None, false));
    };
    let mut resolved: Option<[Option<f64>; 2]> = None;
    let mut agree = |sides: [Option<f64>; 2]| -> Result<(), KernelRefusal> {
        match resolved {
            Some(held) if held != sides => Err(unsupported(
                "setback_faces",
                "asymmetric chamfer: the chain's faces put the law's setbacks on both sides",
            )),
            _ => {
                resolved = Some(sides);
                Ok(())
            }
        }
    };
    let angle = match law {
        SetbackLaw::Distances { faces, distances } => {
            for pair in pairs {
                if pair[0] == faces[0] || pair[1] == faces[1] {
                    agree([Some(distances[0]), Some(distances[1])])?;
                }
                if pair[0] == faces[1] || pair[1] == faces[0] {
                    agree([Some(distances[1]), Some(distances[0])])?;
                }
            }
            0.0
        }
        SetbackLaw::Angle { first, distance, angle } => {
            for pair in pairs {
                if pair[0] == first {
                    agree([Some(distance), None])?;
                }
                if pair[1] == first {
                    agree([None, Some(distance)])?;
                }
            }
            angle
        }
    };
    let distances = resolved.ok_or_else(|| {
        unsupported("setback_faces", format!("asymmetric chamfer: no segment of the chain joins the faces the law names ({law:?})"))
    })?;
    let previous = SIDES.with(|slot| slot.replace(Some(SideLaw { distances, angle })));
    Ok(SidesGuard(previous, true))
}

/// The side law in force for the chain stations on this thread, if any.
pub(super) fn sides() -> Option<SideLaw> {
    SIDES.with(Cell::get)
}

/// The surface parameters of the point of `surface` nearest `target`, by
/// Gauss-Newton from `uv`.
fn foot(surface: &NurbsSurface, mut uv: [f64; 2], target: Vec3, tolerance: f64) -> Result<[f64; 2], KernelRefusal> {
    for _ in 0..NEWTON_ITERATIONS {
        let (point, su, sv) = frame(surface, uv)?;
        let Some(step) = tangent_step(su, sv, target.sub(point)) else {
            break;
        };
        uv = [uv[0] + step[0], uv[1] + step[1]];
        if (step[0] * su.length()).abs().max((step[1] * sv.length()).abs()) <= tolerance {
            break;
        }
    }
    Ok(uv)
}

fn unit_normal(surface: &NurbsSurface, uv: [f64; 2]) -> Result<Vec3, KernelRefusal> {
    let (_, su, sv) = frame(surface, uv)?;
    su.cross(sv).normalized().or_refuse(KernelStage::Refine, "setback_normal")
}

/// The rolling ball's own reading of the two into-directions at the spine:
/// the ball's centre leaves the edge along `rho1·n1 + rho2·n2`, whose shadow
/// in face 1's tangent plane is `rho2·(n2 − (n1·n2)·n1)` (and in turn).  The
/// chain's signed radii thereby say which side of the edge each face lies on
/// without reading a trim.
fn ball_into(normals: [Vec3; 2], rho: [f64; 2], tangent: Vec3) -> Result<[Vec3; 2], KernelRefusal> {
    let mut into = [Vec3::default(); 2];
    for k in 0..2 {
        let other = normals[1 - k];
        let own = normals[k];
        let shadow = other.sub(own.scale(own.dot(other))).scale(rho[1 - k].signum());
        let shadow = shadow.sub(tangent.scale(shadow.dot(tangent)));
        into[k] = shadow.normalized().or_refuse(KernelStage::Refine, "setback_into")?;
    }
    Ok(into)
}

/// The side law's station in the section `(spine, tangent)` from the seed
/// `seed` (the march's own: a neighbour, or the edge's pcurves): the chord-
/// pinned side first, then the other on its chord or the angle law's ray.
fn side_station(
    law: SideLaw,
    surfaces: [&NurbsSurface; 2],
    rho: [f64; 2],
    seed: [f64; 4],
    spine: Vec3,
    tangent: Vec3,
    tolerance: f64,
) -> Result<([f64; 4], [Vec3; 2]), KernelRefusal> {
    let seeds = [[seed[0], seed[1]], [seed[2], seed[3]]];
    // Into-directions at the spine, read on the feet of the spine.
    let mut feet = [[0.0; 2]; 2];
    let mut normals = [Vec3::default(); 2];
    for k in 0..2 {
        feet[k] = foot(surfaces[k], seeds[k], spine, tolerance)?;
        normals[k] = unit_normal(surfaces[k], feet[k])?;
    }
    let into = ball_into(normals, rho, tangent)?;
    let order = if law.distances[0].is_some() { [0, 1] } else { [1, 0] };
    let mut uv = [[0.0; 2]; 2];
    let mut points = [Vec3::default(); 2];
    for k in order {
        let (pin, reach) = match law.distances[k] {
            Some(distance) => (Pin::Chord(distance), distance),
            None => {
                let other = 1 - k;
                let e1 = into[other];
                let e2 = into[k].sub(e1.scale(into[k].dot(e1))).normalized().or_refuse(KernelStage::Refine, "setback_angle_basis")?;
                let ray = e1.scale(-law.angle.cos()).add(e2.scale(law.angle.sin()));
                let normal = tangent.cross(ray).normalized().or_refuse(KernelStage::Refine, "setback_ray")?;
                let theta = into[0].dot(into[1]).clamp(-1.0, 1.0).acos();
                let gamma = std::f64::consts::PI - theta - law.angle;
                let near = law.distances[other].unwrap_or(0.0);
                (Pin::Ray { origin: points[other], normal }, if gamma > 1e-6 { near * law.angle.sin() / gamma.sin() } else { near })
            }
        };
        // A seed on the edge (the march's first station) is stepped into
        // its face first: the chord's gradient vanishes at the spine.
        let (at_seed, _, _) = frame(surfaces[k], seeds[k])?;
        let start = if at_seed.sub(spine).length() < 0.5 * reach {
            displaced_seed(surfaces[k], feet[k], into[k], reach)?
        } else {
            seeds[k]
        };
        let (solved, point) = solve_side(surfaces[k], start, spine, tangent, pin, tolerance).map_err(|error| {
            if std::env::var("BREP_BLEND_STATION_TRACE").ok().as_deref() == Some("1") {
                let feet_points: Vec<Vec3> = (0..2).filter_map(|j| frame(surfaces[j], feet[j]).ok().map(|f| f.0)).collect();
                let seed_points: Vec<Vec3> = (0..2).filter_map(|j| frame(surfaces[j], seeds[j]).ok().map(|f| f.0)).collect();
                eprintln!(
                    "setback side FAILED k {k} spine {spine:?} tangent {tangent:?} into {into:?} feet {feet_points:?} seeds {seed_points:?} start {start:?} feet_uv {feet:?} normals {normals:?}"
                );
            }
            error
        })?;
        if !(point.sub(spine).dot(into[k]) > 0.0) {
            return Err(unsupported(
                "setback_far_side",
                "asymmetric chamfer: a chain rail solved on the far side of the edge",
            ));
        }
        uv[k] = nearest_image(surfaces[k], solved, seeds[k]);
        points[k] = point;
    }
    Ok(([uv[0][0], uv[0][1], uv[1][0], uv[1][1]], points))
}

/// `solve_station_centered` under a side law: the solved (u, v, a, b) and
/// the station's "centre", the spine point (a chamfer's section has no ball).
pub(super) fn solve_centered_by_sides(
    law: SideLaw,
    surfaces: [&NurbsSurface; 2],
    rho: [f64; 2],
    seed: [f64; 4],
    spine: Vec3,
    tangent: Vec3,
    scale: f64,
) -> Result<([f64; 4], Vec3), KernelRefusal> {
    let (uv, _) = side_station(law, surfaces, rho, seed, spine, tangent, 1e-13 * scale.max(1.0))?;
    Ok((uv, spine))
}

/// `tangency_residual` under a side law: the pins' residual at `uv`, the
/// two rail points and the spine as the centre.
pub(super) fn residual_by_sides(
    law: SideLaw,
    surfaces: [&NurbsSurface; 2],
    uv: [f64; 4],
    spine: Vec3,
    tangent: Vec3,
) -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal> {
    let p = [frame(surfaces[0], [uv[0], uv[1]])?.0, frame(surfaces[1], [uv[2], uv[3]])?.0];
    let mut residual = [0.0; 4];
    for k in 0..2 {
        let chord = p[k].sub(spine);
        residual[2 * k] = chord.dot(tangent);
        residual[2 * k + 1] = match law.distances[k] {
            Some(distance) => chord.length() - distance,
            // The ray's residual needs the into-directions; the station
            // solve enforces it, the reading reports the plane alone.
            None => 0.0,
        };
    }
    Ok((residual, p[0], p[1], spine))
}

/// A chain ladder's rail bar under a side law: no looser than 1e-9 of the
/// model scale, as the closed and open marches' (the rails bound the removed
/// volume); the bar unchanged without one.
pub(super) fn chain_bar(solid: &BrepSolid, bar: f64) -> f64 {
    match sides() {
        Some(_) => bar.min(1e-9 * crate::solid_scale(solid).max(1.0)),
        None => bar,
    }
}

/// A chain station's section past its segment's end under a side law: on
/// the NEIGHBOURING segment's edge, the same arc length past the shared
/// vertex, with the tangent in `path[index]`'s edge sense.  An open curve
/// extends along its end tangent line, and a chord pin read from that line
/// can lose the carrier (a seam-split rim's arc turns away from it within a
/// few stations); the rolling ball touches the extended carriers anywhere.
/// Only a neighbour between the SAME two faces continues this segment's
/// carriers' intersection (a line running into an arc leaves its planes,
/// whose intersection the tangent line already is).  `None` without a side
/// law, inside the segment, at an open chain's free end, past a neighbour
/// on other faces, or past the neighbour's own length.
pub(super) fn continued_section(
    path: &[(&EdgeRecord, bool, [u64; 2])],
    index: usize,
    open: bool,
    t: f64,
) -> Result<Option<(Vec3, Vec3)>, KernelRefusal> {
    if sides().is_none() {
        return Ok(None);
    }
    let (edge, forward, faces) = path[index];
    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    if t >= low && t <= high {
        return Ok(None);
    }
    let boundary = if t < low { low } else { high };
    // Past the chain-START end of this segment, or past its END?
    let before = (t < low) == forward;
    let count = path.len();
    let neighbour = match (before, open) {
        (true, false) => (index + count - 1) % count,
        (false, false) => (index + 1) % count,
        (true, true) if index > 0 => index - 1,
        (false, true) if index + 1 < count => index + 1,
        _ => return Ok(None),
    };
    let speed = |curve: &NurbsCurve, at: f64| -> Result<f64, KernelRefusal> {
        Ok(curve.derivatives(at, 1).or_refuse(KernelStage::Refine, "setback_continuation")?[1].length())
    };
    let own_speed = speed(&edge.curve, boundary)?;
    let (other, other_forward, other_faces) = path[neighbour];
    let same_faces = |a: [u64; 2], b: [u64; 2]| a == b || a == [b[1], b[0]];
    if !same_faces(faces, other_faces) {
        return Ok(None);
    }
    // The neighbour's end at the shared vertex, and the way into it.
    let (shared, inward) = match (before, other_forward) {
        (true, true) => (other.t1, -1.0),
        (true, false) => (other.t0, 1.0),
        (false, true) => (other.t0, 1.0),
        (false, false) => (other.t1, -1.0),
    };
    let other_speed = speed(&other.curve, shared)?;
    if !(own_speed > 1e-300) || !(other_speed > 1e-300) {
        return Ok(None);
    }
    let along = inward * (t - boundary).abs() * own_speed / other_speed;
    let at = shared + along;
    let (other_low, other_high) = (other.t0.min(other.t1), other.t0.max(other.t1));
    if at < other_low || at > other_high {
        return Ok(None);
    }
    let (point, tangent) = other
        .curve
        .point_and_unit_tangent_extended(at, other.t0, other.t1)
        .or_refuse(KernelStage::Refine, "setback_continuation")?;
    // Chain direction on the neighbour, then this edge's own sense.
    let chain_tangent = if other_forward { tangent } else { tangent.scale(-1.0) };
    Ok(Some((point, if forward { chain_tangent } else { chain_tangent.scale(-1.0) })))
}
