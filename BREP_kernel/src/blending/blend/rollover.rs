//! A march's contact track may leave its mate across an edge whose other
//! face is TANGENT to the mate. That is not a run-out: the neighbour
//! continues the same smooth wall and the rolling ball rolls straight onto
//! it (`check_support_extent` already admits the reach for exactly this
//! reason). Our march has one carrier pair per stripe, so a track that
//! crosses a tangent edge runs onto the carrier's extension and the lane
//! that owns the stripe has, until now, answered with whatever its fallback
//! said about the shape — on the reported bore mouth, the edge-preserving
//! construction's "ambiguous preserved boundary edge".
//!
//! This module is the row's first two pieces: the DETECTION — which
//! stations bracket a crossing, which boundary edge of the mate the track
//! crosses there, and which face lies across it — typed as a named refusal
//! the ladder falls through exactly as before (the body is the cutter's
//! either way), and the CROSSING STATION itself, solved with the rail frozen
//! on the crossed edge. The split, the re-march on the swapped carrier and
//! the sewing are not built; the validation record of 2026-10-11 measures
//! them.
use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::{fit, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, PolygonClass, Vec3};
use super::stations::{
    apex_point, branch_hop, closed_rail_interval_misses, continue_station, raw_normal, solve_station, station_parameters,
    tangency_state, BlendMate, Continued, FittedRows, Station, NEWTON_ITERATIONS, REFINE_ROUNDS, SEED_INTERVALS,
};

/// The slug of the rollover refusal (`UnsupportedGeometry`).
pub(crate) const TANGENT_ROLLOVER_WHAT: &str = "tangent_rollover";


/// One crossing of a mate's boundary by the march's contact track, onto a
/// tangent neighbour that carries the ball.
#[derive(Clone, Debug)]
pub(super) struct RailCrossing {
    /// 0: the first mate's track, 1: the second's.
    pub(crate) side: usize,
    pub(crate) mate_face_id: u64,
    /// The crossed boundary edge of the mate.
    pub(crate) edge_id: u64,
    /// The face across that edge, tangent to the mate along it.
    pub(crate) neighbour_face_id: u64,
    /// The stations bracketing the crossing, in march order.
    pub(crate) stations: [usize; 2],
    /// Whether the track LEAVES the mate here (inside before, outside after)
    /// or comes back onto it.
    pub(crate) leaving: bool,
    /// Where the track crosses the edge, on the mate's surface. Located by
    /// the bracket's chord against the edge's pcurve: within the chord's
    /// sagitta of the exact crossing, which the frozen-rail solve refines.
    pub(crate) point: Vec3,
    /// The crossed edge's pcurve parameter at the crossing (on the mate).
    pub(crate) edge_parameter: f64,
    /// How far along the bracket's chord the crossing sits, in `[0, 1]`.
    pub(crate) fraction: f64,
    /// The crossing station, solved with this rail frozen on the crossed
    /// edge (`solve_crossing`): its march parameter and the station. `None`
    /// when that solve did not converge, in which case `point` stays the
    /// chord's reading.
    pub(crate) solved: Option<(f64, Station)>,
}

/// One piece of a stripe between two rollover crossings: the carrier face
/// per side, and its stations, the crossing stations included at both ends
/// (so consecutive pieces share one station, the rib).
#[derive(Clone, Debug)]
pub(super) struct StripePiece {
    /// The carrier face id each side's track lies on in this piece: the
    /// mate, or the neighbour the track rolled onto.
    pub(crate) faces: [u64; 2],
    pub(crate) stations: Vec<Station>,
    /// Indices into the plan's crossings of the station at each end.
    pub(crate) from: usize,
    pub(crate) to: usize,
}

/// What a rollover would take: every crossing, solved, and the stripe split
/// at them.
#[derive(Clone, Debug)]
pub(super) struct RolloverPlan {
    pub(crate) crossings: Vec<RailCrossing>,
    /// Empty unless every crossing solved.
    pub(crate) pieces: Vec<StripePiece>,
    /// Per crossing: the first station of the rolled piece, solved on the
    /// SWAPPED carrier pair (the neighbour in the crossed side's slot, its
    /// signed radius adjusted to the neighbour's raw normal) one step past
    /// the crossing and seeded from the crossing station; `None` when the
    /// crossing did not solve or the station on the neighbour did not.
    pub(crate) rolled: Vec<Option<(f64, Station)>>,
}

/// A mate's parameter point brought into its carrier's base period in every
/// closed direction, as the lane's support-exit probe does before it
/// classifies.
fn wrapped_uv(face: &FaceRecord, mut uv: [f64; 2]) -> [f64; 2] {
    if let Ok((closed_u, closed_v)) = face.surface.closed_directions() {
        if closed_u {
            if let Ok([low, high]) = face.surface.domain_u() {
                uv[0] = low + (uv[0] - low).rem_euclid(high - low);
            }
        }
        if closed_v {
            if let Ok([low, high]) = face.surface.domain_v() {
                uv[1] = low + (uv[1] - low).rem_euclid(high - low);
            }
        }
    }
    uv
}

/// The station's track parameter on `side`.
fn track_uv(station: &Station, side: usize) -> [f64; 2] {
    if side == 0 {
        station.uv1
    } else {
        station.uv2
    }
}

/// Inside the mate's trim or not; a point the classifier cannot place is
/// read as outside, as the support-exit probe reads it. A point ON the
/// boundary counts as inside, so a track that touches the boundary without
/// crossing it is not a crossing.
fn inside(face: &FaceRecord, uv: [f64; 2]) -> bool {
    let uv = wrapped_uv(face, uv);
    !crate::parameter_point_in_face(face, crate::Vec2 { x: uv[0], y: uv[1] }, 1e-6)
        .map(|class| class == PolygonClass::Outside)
        .unwrap_or(true)
}

/// Proper crossing of the segments `a→b` and `c→d` in the parameter plane:
/// the fractions along each, when both lie in `[0, 1]`.
fn segments_cross(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> Option<(f64, f64)> {
    let r = [b[0] - a[0], b[1] - a[1]];
    let s = [d[0] - c[0], d[1] - c[1]];
    let denominator = r[0] * s[1] - r[1] * s[0];
    if denominator.abs() <= 1e-18 {
        return None;
    }
    let ac = [c[0] - a[0], c[1] - a[1]];
    let lambda = (ac[0] * s[1] - ac[1] * s[0]) / denominator;
    let mu = (ac[0] * r[1] - ac[1] * r[0]) / denominator;
    let band = 1e-9;
    ((-band..=1.0 + band).contains(&lambda) && (-band..=1.0 + band).contains(&mu))
        .then(|| (lambda.clamp(0.0, 1.0), mu.clamp(0.0, 1.0)))
}

/// Samples per boundary pcurve when the chord between two stations is
/// tested against the mate's loops.
const PCURVE_SAMPLES: usize = 64;

/// Which boundary edge of `face` the chord `from → to` (both already in the
/// base period) crosses, with the chord fraction, the edge's pcurve
/// parameter there and the crossing's parameter point. A chord that leaves
/// the face takes the FIRST boundary it meets, one that comes back the
/// LAST. Pcurves are tested in every period image a closed direction
/// admits, so an unwrapped boolean pcurve is found from a wrapped station.
fn boundary_hit(
    solid: &BrepSolid,
    face: &FaceRecord,
    from: [f64; 2],
    to: [f64; 2],
    leaving: bool,
) -> Option<(u64, f64, f64, [f64; 2])> {
    let (closed_u, closed_v) = face.surface.closed_directions().ok()?;
    let period_u = closed_u.then(|| face.surface.domain_u().ok().map(|[a, b]| b - a)).flatten();
    let period_v = closed_v.then(|| face.surface.domain_v().ok().map(|[a, b]| b - a)).flatten();
    let shifts_u: Vec<f64> = match period_u {
        Some(period) => vec![-period, 0.0, period],
        None => vec![0.0],
    };
    let shifts_v: Vec<f64> = match period_v {
        Some(period) => vec![-period, 0.0, period],
        None => vec![0.0],
    };
    let mut best: Option<(u64, f64, f64, [f64; 2])> = None;
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            if !solid.edges.iter().any(|edge| edge.id == coedge.edge_id) {
                continue;
            }
            let Ok([s0, s1]) = coedge.pcurve.domain() else {
                continue;
            };
            let samples: Vec<([f64; 2], f64)> = (0..=PCURVE_SAMPLES)
                .filter_map(|k| {
                    let s = s0 + (s1 - s0) * k as f64 / PCURVE_SAMPLES as f64;
                    coedge.pcurve.evaluate(s).ok().map(|uv| ([uv.x, uv.y], s))
                })
                .collect();
            if samples.len() < 2 {
                continue;
            }
            for &du in &shifts_u {
                for &dv in &shifts_v {
                    for pair in samples.windows(2) {
                        let c = [pair[0].0[0] + du, pair[0].0[1] + dv];
                        let d = [pair[1].0[0] + du, pair[1].0[1] + dv];
                        let Some((lambda, mu)) = segments_cross(from, to, c, d) else {
                            continue;
                        };
                        let better = match best {
                            None => true,
                            Some((_, found, _, _)) => {
                                if leaving {
                                    lambda < found
                                } else {
                                    lambda > found
                                }
                            }
                        };
                        if better {
                            let s = pair[0].1 + (pair[1].1 - pair[0].1) * mu;
                            let uv = [c[0] + (d[0] - c[0]) * mu, c[1] + (d[1] - c[1]) * mu];
                            best = Some((coedge.edge_id, lambda, s, uv));
                        }
                    }
                }
            }
        }
    }
    best
}

/// The face other than `face` with a coedge on `edge_id`.
fn face_across<'a>(solid: &'a BrepSolid, face: &FaceRecord, edge_id: u64) -> Option<&'a FaceRecord> {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|other| {
            other.id != face.id
                && other
                    .loops
                    .iter()
                    .flat_map(|lp| &lp.coedges)
                    .any(|coedge| coedge.edge_id == edge_id)
        })
}

/// Whether `face` and `neighbour` are tangent at `point`: their outward
/// normals agree to the solid's angular band, as `check_support_extent`'s
/// tangent-neighbour scan reads them.
fn tangent_at(solid: &BrepSolid, face: &FaceRecord, neighbour: &FaceRecord, point: Vec3) -> bool {
    let angular = crate::KernelTolerances::for_solid(solid, 1e-7).angular;
    match (
        crate::fillet::face_plane_normal(face, point),
        crate::fillet::face_plane_normal(neighbour, point),
    ) {
        (Ok((_, here)), Ok((_, there))) => here.dot(there) > 0.0 && here.cross(there).length() <= angular,
        _ => false,
    }
}

/// Every crossing of a mate's boundary by the march's track onto a tangent
/// neighbour that carries a ball of `radius`, on both sides, in march order.
/// A crossing onto a SHARP neighbour, onto a tangent one the ball cannot
/// roll onto (`neighbour_carries_ball`), or one whose boundary edge the
/// chord does not identify, is not reported: those exits keep the lane's
/// existing route.
pub(super) fn tangent_crossings(
    solid: &BrepSolid,
    stations: &[Station],
    mates: [&BlendMate<'_>; 2],
    radius: f64,
) -> Result<Vec<RailCrossing>, KernelRefusal> {
    let mut found = Vec::new();
    for (side, mate) in mates.into_iter().enumerate() {
        let face = mate.face;
        let classes: Vec<bool> = stations.iter().map(|station| inside(face, track_uv(station, side))).collect();
        for index in 0..stations.len().saturating_sub(1) {
            let (before, after) = (classes[index], classes[index + 1]);
            if before == after {
                continue;
            }
            let leaving = before && !after;
            let from = wrapped_uv(face, track_uv(&stations[index], side));
            let to = wrapped_uv(face, track_uv(&stations[index + 1], side));
            // A chord that straddles a seam of a closed carrier is not a
            // chord of the track; such a bracket is left unidentified.
            let straddles = |closed: bool, a: f64, b: f64, domain: Result<[f64; 2], String>| -> bool {
                closed && domain.map(|[low, high]| (a - b).abs() > 0.5 * (high - low)).unwrap_or(true)
            };
            let (closed_u, closed_v) = face.surface.closed_directions().or_refuse(KernelStage::Classify, "closed_directions")?;
            if straddles(closed_u, from[0], to[0], face.surface.domain_u())
                || straddles(closed_v, from[1], to[1], face.surface.domain_v())
            {
                continue;
            }
            let Some((edge_id, fraction, edge_parameter, uv)) = boundary_hit(solid, face, from, to, leaving) else {
                continue;
            };
            let Some(neighbour) = face_across(solid, face, edge_id) else {
                continue;
            };
            let point = face.surface.evaluate(uv[0], uv[1]).or_refuse(KernelStage::Classify, "evaluate")?;
            if !tangent_at(solid, face, neighbour, point) {
                continue;
            }
            // The ball's side: centre = p + rho * n_raw, and the outward
            // normal is n_raw on a same_sense face.
            let ball_outside = (mate.rho != 0.0).then(|| (mate.rho > 0.0) == face.same_sense);
            if !crate::fillet::neighbour_carries_ball_sided(solid, face, neighbour, radius, ball_outside) {
                continue;
            }
            found.push(RailCrossing {
                side,
                mate_face_id: face.id,
                edge_id,
                neighbour_face_id: neighbour.id,
                stations: [index, index + 1],
                leaving,
                point,
                edge_parameter,
                fraction,
                solved: None,
            });
        }
    }
    Ok(found)
}

/// The STOPPED-END crossings of an open stripe (user bug F10, 2026-11-02):
/// on each side, the last crossing before the finish vertex's station at
/// which the track LEAVES its mate onto a tangent neighbour and after
/// which it never returns before that station. Stations past the vertex
/// (the overshoot, marched on the carriers' extensions) are not read. At
/// most one crossing per side, side 0 first.
pub(super) fn stopped_end_crossings(
    solid: &BrepSolid,
    stations: &[Station],
    mates: [&BlendMate<'_>; 2],
    radius: f64,
    vertex_indices: [usize; 2],
) -> Result<Vec<RailCrossing>, KernelRefusal> {
    let finish = vertex_indices[1].min(stations.len().saturating_sub(1));
    if finish <= vertex_indices[0] {
        return Ok(Vec::new());
    }
    let crossings = tangent_crossings(solid, &stations[..=finish], mates, radius)?;
    let mut found = Vec::new();
    for side in 0..2 {
        let on_side: Vec<&RailCrossing> = crossings.iter().filter(|crossing| crossing.side == side).collect();
        // The last crossing before the vertex must be a leaving one.
        if let Some(last) = on_side.last() {
            if last.leaving && last.stations[0] > vertex_indices[0] {
                found.push((*last).clone());
            }
        }
    }
    Ok(found)
}

/// One side of an open stripe that rolled onto its tangent neighbour at a
/// stopped end: what the surgery needs to split the rail there.
#[derive(Clone, Debug)]
pub(crate) struct RolledSide {
    pub(crate) neighbour_face_id: u64,
    /// The crossed tangent edge of the mate.
    pub(crate) edge_id: u64,
    /// Index of the crossing station in the tail's station list.
    pub(crate) station: usize,
    /// The crossing contact, on the crossed edge.
    pub(crate) point: Vec3,
    /// The crossing station's ROW parameter (the rails' and pcurves'), set
    /// once the rows are fitted; the surgery splits the rail there.
    pub(crate) row_parameter: f64,
}

/// One piece of a stopped-end cap's leg: the plane/carrier intersection on
/// ONE face, from `from` to `to` (the chain runs rim -> sharp vertex, and
/// crosses a face boundary where the carrier does not reach the vertex: F10's
/// leg from the rim on face 19 crosses edge 56 onto face 26, which holds
/// vertex 34).
#[derive(Clone, Debug)]
pub(crate) struct LegPiece {
    pub(crate) edge_id: u64,
    pub(crate) face_id: u64,
    pub(crate) from: u64,
    pub(crate) to: u64,
    /// On `face_id`, parameterised from `from` to `to`.
    pub(crate) pcurve: NurbsCurve,
}

/// What the surgery splices for one rolled side: the crossing and the
/// cap's leg chain on the neighbours.
pub(crate) struct TailSplice<'a> {
    pub(crate) rolled: &'a RolledSide,
    pub(crate) legs: Vec<LegPiece>,
}

/// An open stripe's stations with its stopped-end tail re-marched on the
/// swapped carriers (see [`stopped_end_tail`]).
pub(crate) struct StoppedEndTail {
    pub(crate) stations: Vec<Station>,
    pub(crate) ts: Vec<f64>,
    pub(crate) vertex_indices: [usize; 2],
    /// The crossing stations' indices: the rows and pcurves break there.
    pub(crate) breaks: Vec<usize>,
    pub(crate) sides: [Option<RolledSide>; 2],
}

const STOPPED_END_WHAT: &str = "stopped_end_rollover";

/// The stripe's stations with the tail past its stopped-end crossings
/// re-marched on the swapped carriers (user bug F10, 2026-11-02). The
/// crossings are solved in march order, each with the rail frozen on its
/// tangent edge and the other side on whatever carrier it is on by then (a
/// mate, or the neighbour it already rolled onto); the stations after a
/// crossing are marched at the ORIGINAL march's parameters — the vertex
/// station and the overshoot included — on the pair with that side swapped
/// to its neighbour, its signed radius adjusted to the neighbour's raw
/// normal (C1), continuing from the crossing station re-read on the new
/// pair (which must agree with the frozen-rail solve: the contact lies on
/// the tangent edge, on both faces). Stations before the first crossing are
/// the march's own.
#[allow(clippy::too_many_arguments)]
pub(crate) fn stopped_end_tail(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    mates: [&BlendMate<'_>; 2],
    radius_at: &dyn Fn(f64) -> f64,
    scale: f64,
    stations: &[Station],
    station_ts: &[f64],
    vertex_indices: [usize; 2],
    mut crossings: Vec<RailCrossing>,
) -> Result<StoppedEndTail, KernelRefusal> {
    let not_built = |what: String| KernelRefusal::unsupported(KernelStage::Refine, STOPPED_END_WHAT, format!("blend stopped-end rollover: {what}"));
    // The native parameter of every station: the dense reading's, else read
    // back off the centres (every centre lies in its station's section plane).
    let ts: Vec<f64> = if station_ts.len() == stations.len() {
        station_ts.to_vec()
    } else {
        stations.iter().map(|station| edge_parameter_near(edge, station.center)).collect::<Result<_, _>>()?
    };
    crossings.sort_by_key(|crossing| crossing.stations[0]);
    let face_by_id = |id: u64| -> Result<&FaceRecord, KernelRefusal> {
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "rollover_face", format!("blend stopped-end rollover: face {id} vanished")))
    };
    // The carriers each side is on, as owned mates (the neighbour borrows
    // the mate's coedge and loop: only its face and ρ are read here).
    let mut current: [BlendMate<'_>; 2] = [
        BlendMate { face: mates[0].face, coedge: mates[0].coedge, loop_index: mates[0].loop_index, rho: mates[0].rho },
        BlendMate { face: mates[1].face, coedge: mates[1].coedge, loop_index: mates[1].loop_index, rho: mates[1].rho },
    ];
    let mut sides: [Option<RolledSide>; 2] = [None, None];
    let mut out_stations: Vec<Station> = Vec::with_capacity(stations.len() + 2);
    let mut out_ts: Vec<f64> = Vec::with_capacity(stations.len() + 2);
    let mut breaks = Vec::new();
    // Stations the crossing solves read their seeds from: the original list
    // with a rolled side's parameter points moved onto its neighbour.
    let mut work: Vec<Station> = stations.to_vec();
    let mut next_target = 0usize;
    for (k, crossing) in crossings.iter().enumerate() {
        let side = crossing.side;
        if sides[side].is_some() {
            return Err(not_built(format!("side {side} leaves its carrier twice before the vertex")));
        }
        // The crossing station: the frozen-rail solve, or — where its Newton
        // is singular (the trimmed F10 model's coarser rung) — a bisection of
        // the march between the bracket's stations on the crossed side's
        // membership of its mate, the contact then snapped onto the edge.
        let (t_cross, station) = match solve_crossing(edge, [&current[0], &current[1]], radius_at, &work, crossing, scale) {
            Ok(solved) => solved,
            Err(error) => {
                if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                    eprintln!("stopped-end rollover: frozen-rail solve declined ({}); bisecting the march", error.message);
                }
                bisect_crossing(solid, edge, [&current[0], &current[1]], radius_at, &work, &ts, crossing, scale)?
            }
        };
        if k == 0 {
            // The march's own stations up to the crossing.
            let before = crossing.stations[0];
            out_stations.extend_from_slice(&stations[..=before]);
            out_ts.extend_from_slice(&ts[..=before]);
            next_target = before + 1;
        } else if let Some(&last) = out_ts.last() {
            if t_cross <= last {
                return Err(not_built(format!("crossing stations out of order ({t_cross:.6} after {last:.6})")));
            }
        }
        while next_target < ts.len() && ts[next_target] <= t_cross {
            next_target += 1;
        }
        // The crossing station, the rolled side's parameter point read on the
        // neighbour; the pair swapped from here on.
        let neighbour = face_by_id(crossing.neighbour_face_id)?;
        let point = if side == 0 { station.p1 } else { station.p2 };
        let projection = crate::project_point_to_surface(&neighbour.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        let on_mate = track_uv(&station, side);
        let mate_raw = raw_normal(&current[side].face.surface, on_mate[0], on_mate[1])?;
        let neighbour_raw = raw_normal(&neighbour.surface, projection.u, projection.v)?;
        let rho = if mate_raw.dot(neighbour_raw) < 0.0 { -current[side].rho } else { current[side].rho };
        current[side] = BlendMate { face: neighbour, coedge: current[side].coedge, loop_index: current[side].loop_index, rho };
        let mut first = station;
        if side == 0 {
            first.uv1 = [projection.u, projection.v];
        } else {
            first.uv2 = [projection.u, projection.v];
        }
        let surfaces = [&current[0].face.surface, &current[1].face.surface];
        let signs = [current[0].rho.signum(), current[1].rho.signum()];
        let frame = PieceFrame { edge, surfaces, signs, radius_at, scale };
        let seed_uv = [first.uv1[0], first.uv1[1], first.uv2[0], first.uv2[1]];
        let check = frame.station(t_cross, seed_uv)?;
        let agreement = check.center.sub(first.center).length().max(check.p1.sub(first.p1).length()).max(check.p2.sub(first.p2).length());
        if agreement > 1e-6 * scale {
            return Err(KernelRefusal::non_convergence(
                KernelStage::Refine,
                STOPPED_END_WHAT,
                format!("blend stopped-end rollover: the crossing station re-read on face {} moves by {agreement:.3e}", neighbour.id),
            ));
        }
        breaks.push(out_stations.len());
        sides[side] = Some(RolledSide { neighbour_face_id: neighbour.id, edge_id: crossing.edge_id, station: out_stations.len(), point, row_parameter: 0.0 });
        out_stations.push(first);
        out_ts.push(t_cross);
        // March on the new pair to the next crossing (exclusive) or the end.
        let stop = crossings.get(k + 1).map(|next| {
            // The next crossing's own parameter is read after its solve; the
            // targets up to its bracket's first station are marched now.
            ts[next.stations[0]]
        });
        let mut left = Continued { t: t_cross, uv: seed_uv, center: Some(first.center) };
        while next_target < ts.len() && stop.is_none_or(|stop| ts[next_target] <= stop) {
            let t = ts[next_target];
            let (node, marched) = frame.continue_from(&left, t)?;
            out_stations.push(marched);
            out_ts.push(node.t);
            // The working seed list follows the rolled side onto the neighbour.
            if side == 0 {
                work[next_target].uv1 = marched.uv1;
            } else {
                work[next_target].uv2 = marched.uv2;
            }
            left = node;
            next_target += 1;
        }
    }
    // The vertex stations: the start one unchanged, the finish one at its
    // original parameter.
    let t_finish = ts[vertex_indices[1]];
    let finish = out_ts
        .iter()
        .position(|t| (t - t_finish).abs() <= 1e-12 * (1.0 + t_finish.abs()))
        .ok_or_else(|| not_built("the vertex station was not marched on the neighbour".to_string()))?;
    Ok(StoppedEndTail { stations: out_stations, ts: out_ts, vertex_indices: [vertex_indices[0], finish], breaks, sides })
}

/// The crossing station by bisection: the march continued on the current
/// pair from the bracket's first station to the parameter where the crossed
/// side's parameter point leaves its mate, to 1e-12 of the bracket's span;
/// the crossed contact is then within the bisection's reach of the tangent
/// edge and is snapped onto it (its parameter point re-read on the mate).
#[allow(clippy::too_many_arguments)]
fn bisect_crossing(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    mates: [&BlendMate<'_>; 2],
    radius_at: &dyn Fn(f64) -> f64,
    stations: &[Station],
    ts: &[f64],
    crossing: &RailCrossing,
    scale: f64,
) -> Result<(f64, Station), KernelRefusal> {
    let side = crossing.side;
    let [before, after] = crossing.stations;
    let surfaces = [&mates[0].face.surface, &mates[1].face.surface];
    let signs = [mates[0].rho.signum(), mates[1].rho.signum()];
    let frame = PieceFrame { edge, surfaces, signs, radius_at, scale };
    let start = &stations[before];
    let mut left = Continued { t: ts[before], uv: [start.uv1[0], start.uv1[1], start.uv2[0], start.uv2[1]], center: Some(start.center) };
    let (mut lo, mut hi) = (ts[before], ts[after]);
    let mut inside_station = *start;
    let mut outside_station: Option<Station> = None;
    let span = (hi - lo).abs().max(1e-300);
    for _ in 0..60 {
        if (hi - lo).abs() <= 1e-12 * span {
            break;
        }
        let mid = 0.5 * (lo + hi);
        let (node, station) = frame.continue_from(&left, mid)?;
        if inside(mates[side].face, track_uv(&station, side)) == crossing.leaving {
            // Still on the mate (leaving) / still off it (returning): advance.
            lo = mid;
            left = node;
            inside_station = station;
        } else {
            hi = mid;
            outside_station = Some(station);
        }
    }
    let tangent_edge = solid
        .edges
        .iter()
        .find(|candidate| candidate.id == crossing.edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "rollover_tangent_edge", format!("blend stopped-end rollover: tangent edge {} vanished", crossing.edge_id)))?;
    // The membership test reads the mate's PCURVE, which stands off the 3-D
    // edge by the operand's trim band; refine the bracket on the contact's
    // signed offset from the edge itself, which vanishes at the crossing.
    let contact_of = |station: &Station| if side == 0 { station.p1 } else { station.p2 };
    let offset_of = |station: &Station, across: Vec3| -> Result<f64, KernelRefusal> {
        let contact = contact_of(station);
        let foot = crate::project_point_to_curve(&tangent_edge.curve, contact).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
        Ok(contact.sub(foot.point).dot(across))
    };
    let mut t_cross = lo;
    let mut station = inside_station;
    if let Some(outside) = outside_station {
        // Inside -> outside across the edge, the reference direction.
        if let Ok(across) = contact_of(&outside).sub(contact_of(&inside_station)).normalized() {
            let start = &stations[before];
            let mut left = Continued { t: ts[before], uv: [start.uv1[0], start.uv1[1], start.uv2[0], start.uv2[1]], center: Some(start.center) };
            let (mut a, mut b) = (ts[before], ts[after]);
            let (_, station_a) = frame.continue_from(&left, a)?;
            let (_, station_b) = frame.continue_from(&left, b)?;
            let (mut fa, mut fb) = (offset_of(&station_a, across)?, offset_of(&station_b, across)?);
            if fa * fb < 0.0 {
                for _ in 0..80 {
                    // Regula falsi with bisection safeguard.
                    let mut t = b - fb * (b - a) / (fb - fa);
                    if !(t > a.min(b) && t < a.max(b)) {
                        t = 0.5 * (a + b);
                    }
                    let (node, candidate) = frame.continue_from(&left, t)?;
                    let f = offset_of(&candidate, across)?;
                    t_cross = t;
                    station = candidate;
                    if f.abs() <= 1e-10 * (1.0 + scale) {
                        break;
                    }
                    if (f < 0.0) == (fa < 0.0) {
                        a = t;
                        fa = f;
                        left = node;
                    } else {
                        b = t;
                        fb = f;
                    }
                    if (b - a).abs() <= 1e-13 * span {
                        break;
                    }
                }
            }
        }
    }
    // Snap the crossed contact onto the tangent edge and re-read its
    // parameter point on the mate.
    let contact = contact_of(&station);
    let foot = crate::project_point_to_curve(&tangent_edge.curve, contact).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
    if foot.distance > 1e-6 * (1.0 + scale) {
        return Err(KernelRefusal::non_convergence(
            KernelStage::Refine,
            STOPPED_END_WHAT,
            format!("blend stopped-end rollover: the bisected crossing stands {:.3e} off tangent edge {}", foot.distance, crossing.edge_id),
        ));
    }
    let on_edge = tangent_edge.curve.evaluate(foot.u).or_refuse(KernelStage::Refine, "evaluate")?;
    let on_mate = crate::project_point_to_surface(&mates[side].face.surface, on_edge).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    if side == 0 {
        station.p1 = on_edge;
        station.uv1 = [on_mate.u, on_mate.v];
    } else {
        station.p2 = on_edge;
        station.uv2 = [on_mate.u, on_mate.v];
    }
    Ok((t_cross, station))
}

/// The edge parameter whose normal plane holds `point`: the foot of the
/// point on the edge, from the nearest of 256 samples by Newton on
/// `(P − C(t)) · C'(t) = 0`. A station's contacts and centre all lie in its
/// section plane, so this reads a station's parameter back from any of
/// them.
pub(super) fn edge_parameter_near(edge: &EdgeRecord, point: Vec3) -> Result<f64, KernelRefusal> {
    const SAMPLES: usize = 256;
    let span = edge.t1 - edge.t0;
    let mut best = (edge.t0, f64::INFINITY);
    for k in 0..=SAMPLES {
        let t = edge.t0 + span * k as f64 / SAMPLES as f64;
        let distance = edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?.sub(point).length();
        if distance < best.1 {
            best = (t, distance);
        }
    }
    use crate::projection::{curve_foot_newton, CurveFootPolicy, FootDomain, FootMode};
    let (t, _) = curve_foot_newton(&edge.curve, point, best.0, [edge.t0, edge.t1],
        CurveFootPolicy {
            domain: FootDomain::Extended,
            iterations: 12,
            slope_guard: 1e-18,
            step_limit: f64::INFINITY,
            parameter_tolerance: 1e-14 * (1.0 + span.abs()),
            mode: FootMode::Stationary { positive_slope: false, relative_step: false },
        }).or_refuse(KernelStage::Refine, "derivatives_extended")?;
    Ok(t)
}

/// The crossing station: the tangency system solved with the crossed rail
/// FROZEN on the crossed edge. Unknowns are the march parameter `t`, the
/// crossed edge's pcurve parameter `s` on the mate (the rail's parameter
/// point is the pcurve at `s`) and the other side's `(a, b)`; the four
/// residuals are the station solver's own, centre mismatch and section
/// plane. Seeded from the bracket's chord. Returns the parameter and the
/// station, built as the march builds its own.
pub(super) fn solve_crossing(
    edge: &EdgeRecord,
    mates: [&BlendMate<'_>; 2],
    radius_at: &dyn Fn(f64) -> f64,
    stations: &[Station],
    crossing: &RailCrossing,
    scale: f64,
) -> Result<(f64, Station), KernelRefusal> {
    let side = crossing.side;
    let other = 1 - side;
    let mate = mates[side];
    let pcurve = &mate
        .face
        .loops
        .iter()
        .flat_map(|lp| &lp.coedges)
        .find(|coedge| coedge.edge_id == crossing.edge_id)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "crossed_edge_pcurve", "blend rollover: the crossed edge has no pcurve on the mate"))?
        .pcurve;
    let surfaces = [&mates[0].face.surface, &mates[1].face.surface];
    let signs = [mates[0].rho.signum(), mates[1].rho.signum()];
    let [before, after] = crossing.stations;
    let seed_other = {
        let a = track_uv(&stations[before], other);
        let b = track_uv(&stations[after], other);
        [a[0] + (b[0] - a[0]) * crossing.fraction, a[1] + (b[1] - a[1]) * crossing.fraction]
    };
    let mut x = [
        edge_parameter_near(edge, crossing.point)?,
        crossing.edge_parameter,
        seed_other[0],
        seed_other[1],
    ];
    let assemble = |x: &[f64; 4]| -> Result<[f64; 4], KernelRefusal> {
        let on_edge = pcurve.evaluate(x[1]).or_refuse(KernelStage::Refine, "pcurve")?;
        let mut uv = [0.0; 4];
        uv[2 * side] = on_edge.x;
        uv[2 * side + 1] = on_edge.y;
        uv[2 * other] = x[2];
        uv[2 * other + 1] = x[3];
        Ok(uv)
    };
    let state_at = |x: &[f64; 4]| -> Result<super::stations::TangencyState, KernelRefusal> {
        let (section_point, section_tangent) = edge
            .curve
            .point_and_unit_tangent_extended(x[0], edge.t0, edge.t1)
            .map_err(|error| format!("blend rollover: edge {}: {error}", edge.id))
            .or_refuse(KernelStage::Refine, "point_and_unit_tangent_extended")?;
        let radius = radius_at(x[0]);
        tangency_state(
            surfaces[0],
            surfaces[1],
            [signs[0] * radius, signs[1] * radius],
            assemble(x)?,
            section_point,
            section_tangent,
        )
    };
    let tolerance = 1e-11 * (1.0 + scale);
    for _ in 0..NEWTON_ITERATIONS {
        let state = state_at(&x)?;
        let error = state.residual.iter().map(|value| value.abs()).fold(0.0, f64::max);
        if error <= tolerance {
            let uv = assemble(&x)?;
            let cos_alpha = signs[0] * signs[1] * state.n1.dot(state.n2);
            let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
            if weight <= 1e-6 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "tangent_faces", "blend: faces are tangent at a station (α = π)"));
            }
            let apex = apex_point(state.p1, state.n1, state.p2, state.n2, state.center)?;
            return Ok((
                x[0],
                Station {
                    uv1: [uv[0], uv[1]],
                    uv2: [uv[2], uv[3]],
                    p1: state.p1,
                    p2: state.p2,
                    center: state.center,
                    weight,
                    apex,
                },
            ));
        }
        let mut jacobian = [[0.0f64; 4]; 4];
        let step = 1e-7;
        for column in 0..4 {
            let mut probe = x;
            probe[column] += step;
            let probed = state_at(&probe)?;
            for row in 0..4 {
                jacobian[row][column] = (probed.residual[row] - state.residual[row]) / step;
            }
        }
        let delta = fit::solve_small::<4>(jacobian, state.residual, 4)
            .map_err(|error| format!("blend rollover: the frozen-rail Newton is singular ({error})"))
            .or_refuse(KernelStage::Refine, "solve_small")?;
        for (value, correction) in x.iter_mut().zip(delta) {
            *value -= correction;
        }
    }
    Err(KernelRefusal::non_convergence(KernelStage::Refine, "rollover_crossing", "blend rollover: the crossing station did not converge"))
}

/// The first station of a rolled piece: the tangency system on the SWAPPED
/// carrier pair — the neighbour in the crossed side's slot — one step past
/// the crossing station in the direction the track rolls, seeded from the
/// crossing station (its parameter point on the neighbour is the crossing
/// point's projection). The neighbour's signed radius is the mate's,
/// flipped when the neighbour's raw normal opposes the mate's at the
/// crossing (C1: "ρ sign-adjusted to the neighbour's normal"). This is the
/// seed the re-march of the piece would continue from.
pub(super) fn seed_on_neighbour(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    mates: [&BlendMate<'_>; 2],
    radius_at: &dyn Fn(f64) -> f64,
    crossing: &RailCrossing,
    scale: f64,
) -> Result<(f64, Station), KernelRefusal> {
    let (t_cross, station) = crossing
        .solved
        .as_ref()
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "rollover_unsolved", "blend rollover: the crossing station did not solve"))?;
    let side = crossing.side;
    let other = 1 - side;
    let neighbour = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == crossing.neighbour_face_id)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "rollover_neighbour", "blend rollover: the neighbour face vanished"))?;
    let mate = mates[side];
    let on_mate = track_uv(station, side);
    let projection = crate::project_point_to_surface(&neighbour.surface, crossing.point)
        .or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let mate_raw = raw_normal(&mate.face.surface, on_mate[0], on_mate[1])?;
    let neighbour_raw = raw_normal(&neighbour.surface, projection.u, projection.v)?;
    let flip = if mate_raw.dot(neighbour_raw) < 0.0 { -1.0 } else { 1.0 };
    let signs = {
        let mut signs = [mates[0].rho.signum(), mates[1].rho.signum()];
        signs[side] *= flip;
        signs
    };
    let mut surfaces = [&mates[0].face.surface, &mates[1].face.surface];
    surfaces[side] = &neighbour.surface;
    let step = (edge.t1 - edge.t0) / 64.0;
    let t = if crossing.leaving { t_cross + step } else { t_cross - step };
    let (section_point, section_tangent) = edge
        .curve
        .point_and_unit_tangent_extended(t, edge.t0, edge.t1)
        .map_err(|error| format!("blend rollover: edge {}: {error}", edge.id))
        .or_refuse(KernelStage::Refine, "point_and_unit_tangent_extended")?;
    let radius = radius_at(t);
    let mut seed = [0.0; 4];
    seed[2 * side] = projection.u;
    seed[2 * side + 1] = projection.v;
    let other_uv = track_uv(station, other);
    seed[2 * other] = other_uv[0];
    seed[2 * other + 1] = other_uv[1];
    let rho = [signs[0] * radius, signs[1] * radius];
    let uv = solve_station(surfaces[0], surfaces[1], rho, seed, section_point, section_tangent, scale)?;
    let state = tangency_state(surfaces[0], surfaces[1], rho, uv, section_point, section_tangent)?;
    let cos_alpha = signs[0] * signs[1] * state.n1.dot(state.n2);
    let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
    if weight <= 1e-6 {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "tangent_faces", "blend: faces are tangent at a station (α = π)"));
    }
    let apex = apex_point(state.p1, state.n1, state.p2, state.n2, state.center)?;
    Ok((
        t,
        Station {
            uv1: [uv[0], uv[1]],
            uv2: [uv[2], uv[3]],
            p1: state.p1,
            p2: state.p2,
            center: state.center,
            weight,
            apex,
        },
    ))
}

/// The CLOSED march's stations split at its solved crossings, in march
/// order: a piece runs from one crossing station to the next, the last
/// wrapping through the seam back to the first. The crossed side's carrier
/// toggles at each crossing — onto the neighbour where the track leaves
/// the mate, back onto the mate where it returns; the other side keeps its
/// mate. Empty unless every crossing solved.
pub(super) fn split_closed_at_crossings(
    stations: &[Station],
    mates: [&BlendMate<'_>; 2],
    crossings: &[RailCrossing],
) -> Vec<StripePiece> {
    if crossings.is_empty() || crossings.iter().any(|crossing| crossing.solved.is_none()) {
        return Vec::new();
    }
    let mate_ids = [mates[0].face.id, mates[1].face.id];
    let count = crossings.len();
    let mut pieces = Vec::with_capacity(count);
    for index in 0..count {
        let from = &crossings[index];
        let to = &crossings[(index + 1) % count];
        let mut faces = mate_ids;
        if from.leaving {
            faces[from.side] = from.neighbour_face_id;
        }
        let mut piece = vec![from.solved.as_ref().map(|(_, station)| *station).expect("solved")];
        let [_, start] = from.stations;
        let [end, _] = to.stations;
        if index + 1 < count {
            piece.extend_from_slice(&stations[start..=end]);
        } else {
            // The wrapping piece: the closed march's last station repeats its
            // first (one lap), so the seam station is taken once.
            piece.extend_from_slice(&stations[start..stations.len() - 1]);
            piece.extend_from_slice(&stations[..=end]);
        }
        piece.push(to.solved.as_ref().map(|(_, station)| *station).expect("solved"));
        pieces.push(StripePiece {
            faces,
            stations: piece,
            from: index,
            to: (index + 1) % count,
        });
    }
    pieces
}

/// The closed lane's rollover plan: the tangent crossings of both tracks,
/// each solved with its rail frozen on the crossed edge, and the stripe
/// split at them. `None` when no track rolls over a tangent neighbour.
pub(super) fn plan_closed(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    stations: &[Station],
    mates: [&BlendMate<'_>; 2],
    radius_at: &dyn Fn(f64) -> f64,
    scale: f64,
) -> Result<Option<RolloverPlan>, KernelRefusal> {
    let radius = radius_at(0.5 * (edge.t0 + edge.t1)).abs();
    let mut crossings = tangent_crossings(solid, stations, mates, radius)?;
    if crossings.is_empty() {
        return Ok(None);
    }
    for crossing in &mut crossings {
        match solve_crossing(edge, mates, radius_at, stations, crossing, scale) {
            Ok((t, station)) => {
                crossing.point = if crossing.side == 0 { station.p1 } else { station.p2 };
                crossing.solved = Some((t, station));
            }
            Err(error) => {
                if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                    eprintln!("closed-rim rollover: crossing station did not solve: {error}");
                }
            }
        }
    }
    let pieces = split_closed_at_crossings(stations, mates, &crossings);
    let rolled = crossings
        .iter()
        .map(|crossing| {
            if crossing.solved.is_none() {
                return None;
            }
            match seed_on_neighbour(solid, edge, mates, radius_at, crossing, scale) {
                Ok(seeded) => Some(seeded),
                Err(error) => {
                    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                        eprintln!("closed-rim rollover: the station on the neighbour did not solve: {error}");
                    }
                    None
                }
            }
        })
        .collect();
    Ok(Some(RolloverPlan { crossings, pieces, rolled }))
}

/// One piece of the split stripe, marched on ITS carriers and fitted: the
/// stations between its two crossing stations (both included, so consecutive
/// pieces share one station — the rib — to the bit), their march parameters,
/// and the open rows through them with a support pcurve per side on the
/// piece's own carrier. A rolled piece's carrier on the crossed side is the
/// neighbour; its end stations are the crossing stations with that side's
/// parameter point read on the neighbour.
pub(super) struct RolledPiece {
    /// The carrier face id per side.
    pub(super) faces: [u64; 2],
    pub(super) stations: Vec<Station>,
    /// The march parameter of every station, increasing; a piece that wraps
    /// the closed march's seam runs past `edge.t1` the way the closed march
    /// does. The surgery reads the stations, not their parameters; the row's
    /// test reads these as the march's evidence.
    #[allow(dead_code)]
    pub(super) ts: Vec<f64>,
    pub(super) rows: FittedRows,
    /// Indices into the plan's crossings of the station at each end.
    pub(super) from: usize,
    pub(super) to: usize,
    /// The side whose carrier is the neighbour, when this piece rolled.
    pub(super) rolled_side: Option<usize>,
    /// The index of the closed march's seam station (its first station, on
    /// the anchored carrier's seam meridian) within this piece, when the
    /// piece wraps through it: the surgery splits the piece's rail there so
    /// the anchored carrier keeps its seam-in-one-loop structure.
    pub(super) seam_station: Option<usize>,
}

/// Stations a rolled piece is seeded with between its crossing stations,
/// before the rail bar refines them: the closed march's density over the
/// piece's share of the lap, and never fewer than this.
const ROLLED_PIECE_MIN_INTERVALS: usize = 8;

/// The station ceiling of one piece's refinement.
const ROLLED_PIECE_STATION_CEILING: usize = (SEED_INTERVALS << 6) + 1;

/// One piece's march: its carriers, signed radii and section plane, the
/// station solver the closed march uses, under the same branch guard.
struct PieceFrame<'a> {
    edge: &'a EdgeRecord,
    surfaces: [&'a NurbsSurface; 2],
    signs: [f64; 2],
    radius_at: &'a dyn Fn(f64) -> f64,
    scale: f64,
}

impl PieceFrame<'_> {
    fn rho(&self, t: f64) -> [f64; 2] {
        let radius = (self.radius_at)(t);
        [self.signs[0] * radius, self.signs[1] * radius]
    }

    fn section(&self, t: f64) -> Result<(Vec3, Vec3), KernelRefusal> {
        self.edge
            .curve
            .point_and_unit_tangent_extended(t, self.edge.t0, self.edge.t1)
            .map_err(|error| format!("blend rollover: edge {}: {error}", self.edge.id))
            .or_refuse(KernelStage::Refine, "point_and_unit_tangent_extended")
    }

    /// The station at `uv` (already solved) at parameter `t`.
    fn station(&self, t: f64, uv: [f64; 4]) -> Result<Station, KernelRefusal> {
        let (section_point, section_tangent) = self.section(t)?;
        let state = tangency_state(self.surfaces[0], self.surfaces[1], self.rho(t), uv, section_point, section_tangent)?;
        let cos_alpha = self.signs[0] * self.signs[1] * state.n1.dot(state.n2);
        let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
        if weight <= 1e-6 {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "tangent_faces", "blend: faces are tangent at a station (α = π)"));
        }
        let apex = apex_point(state.p1, state.n1, state.p2, state.n2, state.center)?;
        Ok(Station {
            uv1: [uv[0], uv[1]],
            uv2: [uv[2], uv[3]],
            p1: state.p1,
            p2: state.p2,
            center: state.center,
            weight,
            apex,
        })
    }

    /// The station at `t` continued from `left`, under the closed march's
    /// branch guard (a trip halves the seed, never refuses).
    fn continue_from(&self, left: &Continued, t: f64) -> Result<(Continued, Station), KernelRefusal> {
        let rho_at = |t: f64| self.rho(t);
        let solve = |t: f64, seed: [f64; 4]| -> Result<([f64; 4], Vec3), KernelRefusal> {
            let (section_point, section_tangent) = self.section(t)?;
            let uv = solve_station(self.surfaces[0], self.surfaces[1], self.rho(t), seed, section_point, section_tangent, self.scale)?;
            let state = tangency_state(self.surfaces[0], self.surfaces[1], self.rho(t), uv, section_point, section_tangent)?;
            Ok((uv, state.center))
        };
        let hop = |left: &Continued, right: &Continued| branch_hop(self.surfaces, self.edge, &rho_at, left, right);
        let node = continue_station(&solve, &hop, left, t, 0)?;
        let station = self.station(node.t, node.uv)?;
        Ok((node, station))
    }
}

/// A closed-carrier parameter track made continuous: every station's `uv`
/// on `side` is moved by whole periods to lie within half a period of the
/// station before it. The closed march unwraps its own track, but a piece
/// assembled across the march's seam joins stations a period apart.
fn unwrap_track(stations: &mut [Station], side: usize, surface: &NurbsSurface) -> Result<(), KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let periods = [
        closed_u.then(|| surface.domain_u().map(|[a, b]| b - a)).transpose().or_refuse(KernelStage::Refine, "domain_u")?,
        closed_v.then(|| surface.domain_v().map(|[a, b]| b - a)).transpose().or_refuse(KernelStage::Refine, "domain_v")?,
    ];
    for index in 1..stations.len() {
        let previous = track_uv(&stations[index - 1], side);
        let mut current = track_uv(&stations[index], side);
        for (axis, period) in periods.iter().enumerate() {
            if let Some(period) = period {
                if *period > 0.0 {
                    let laps = ((current[axis] - previous[axis]) / period).round();
                    current[axis] -= laps * period;
                }
            }
        }
        if side == 0 {
            stations[index].uv1 = current;
        } else {
            stations[index].uv2 = current;
        }
    }
    Ok(())
}

/// March parameters made increasing along a closed edge: a drop of more
/// than half the span between consecutive stations is a lap.
fn unwrap_parameters(ts: &mut [f64], span: f64) {
    for index in 1..ts.len() {
        while ts[index] < ts[index - 1] - 0.5 * span {
            ts[index] += span;
        }
    }
}

/// Every piece of the plan marched on its own carriers and fitted: a piece
/// on the mates keeps the closed march's stations between its crossing
/// stations (their parameters read back from the ball centres, which lie in
/// the section planes); a rolled piece's interior is re-marched on the
/// swapped pair from its first crossing station, the crossed side's signed
/// radius adjusted to the neighbour's raw normal there. Every piece is then
/// refined where its fitted rails stand off its carriers between stations
/// by more than `rail_bar` (the closed lane's own reading, at 0.25/0.5/0.75
/// of every span), one midpoint station per failing interval and round,
/// within the closed march's rounds. `anchor_u` is the closed march's
/// anchor: when it is set, the march's first station lies on the first
/// carrier's seam meridian and the piece that wraps through it records
/// where.
#[allow(clippy::too_many_arguments)]
pub(super) fn march_pieces(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    plan: &RolloverPlan,
    stations: &[Station],
    mates: [&BlendMate<'_>; 2],
    radius_at: &dyn Fn(f64) -> f64,
    scale: f64,
    rail_bar: f64,
    chamfer: bool,
    anchor_u: Option<f64>,
) -> Result<Vec<RolledPiece>, KernelRefusal> {
    if plan.pieces.is_empty() {
        return Err(KernelRefusal::unsupported(
            KernelStage::Refine,
            TANGENT_ROLLOVER_WHAT,
            "blend rollover: a crossing station did not solve, so the stripe was not split",
        ));
    }
    let span = edge.t1 - edge.t0;
    let mate_ids = [mates[0].face.id, mates[1].face.id];
    let face_by_id = |id: u64| -> Result<&FaceRecord, KernelRefusal> {
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == id)
            .ok_or(KernelRefusal::internal(KernelStage::Refine, "rollover_carrier", "blend rollover: a piece's carrier face vanished"))
    };
    let count = plan.pieces.len();
    let mut pieces = Vec::with_capacity(count);
    for (index, piece) in plan.pieces.iter().enumerate() {
        let from = &plan.crossings[piece.from];
        let to = &plan.crossings[piece.to];
        let (t_from, from_station) = from.solved.as_ref().expect("the plan holds solved crossings only");
        let (t_to, to_station) = to.solved.as_ref().expect("the plan holds solved crossings only");
        let (t_from, mut t_to) = (*t_from, *t_to);
        // The piece that runs past the march's end (its parameters wrap).
        let wraps = t_to <= t_from;
        if t_to <= t_from {
            t_to += span;
        }
        let rolled_side = (0..2).find(|&side| piece.faces[side] != mate_ids[side]);
        let faces = [face_by_id(piece.faces[0])?, face_by_id(piece.faces[1])?];
        let surfaces = [&faces[0].surface, &faces[1].surface];
        let mut signs = [mates[0].rho.signum(), mates[1].rho.signum()];
        if let Some(side) = rolled_side {
            // C1: ρ sign-adjusted to the neighbour's normal, read where the
            // track left the mate.
            let on_mate = track_uv(from_station, side);
            let mate_raw = raw_normal(&mates[side].face.surface, on_mate[0], on_mate[1])?;
            let projection = crate::project_point_to_surface(surfaces[side], from.point)
                .or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            let neighbour_raw = raw_normal(surfaces[side], projection.u, projection.v)?;
            if mate_raw.dot(neighbour_raw) < 0.0 {
                signs[side] = -signs[side];
            }
        }
        let frame = PieceFrame { edge, surfaces, signs, radius_at, scale };
        // The end stations on THIS piece's carriers: the crossing station,
        // with the rolled side's parameter point read on the neighbour (the
        // contact lies on the crossed edge, on both faces).
        let on_carriers = |station: &Station, point: Vec3| -> Result<Station, KernelRefusal> {
            let Some(side) = rolled_side else { return Ok(*station) };
            let projection = crate::project_point_to_surface(surfaces[side], point)
                .or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            let mut moved = *station;
            if side == 0 {
                moved.uv1 = [projection.u, projection.v];
            } else {
                moved.uv2 = [projection.u, projection.v];
            }
            Ok(moved)
        };
        let first = on_carriers(from_station, from.point)?;
        let last = on_carriers(to_station, to.point)?;
        let (mut marched, mut ts): (Vec<Station>, Vec<f64>) = match rolled_side {
            None => {
                // The closed march's own stations, with their parameters read
                // back off the centres.
                let mut ts = Vec::with_capacity(piece.stations.len());
                for (k, station) in piece.stations.iter().enumerate() {
                    ts.push(if k == 0 {
                        t_from
                    } else if k + 1 == piece.stations.len() {
                        t_to
                    } else {
                        edge_parameter_near(edge, station.center)?
                    });
                }
                unwrap_parameters(&mut ts, span);
                let mut stations = piece.stations.clone();
                let n = stations.len();
                stations[0] = first;
                stations[n - 1] = last;
                // A short piece (the closed march placed few stations between
                // these crossings) is bisected by continuation until it
                // carries the rolled pieces' minimum, so its rows can be fitted
                // and refined like theirs.
                while stations.len() - 1 < ROLLED_PIECE_MIN_INTERVALS {
                    let mut denser = Vec::with_capacity(2 * stations.len());
                    let mut denser_ts = Vec::with_capacity(denser.capacity());
                    for k in 0..stations.len() - 1 {
                        denser.push(stations[k]);
                        denser_ts.push(ts[k]);
                        let left = Continued {
                            t: ts[k],
                            uv: [stations[k].uv1[0], stations[k].uv1[1], stations[k].uv2[0], stations[k].uv2[1]],
                            center: Some(stations[k].center),
                        };
                        let (node, station) = frame.continue_from(&left, 0.5 * (ts[k] + ts[k + 1]))?;
                        denser.push(station);
                        denser_ts.push(node.t);
                    }
                    denser.push(*stations.last().expect("stations"));
                    denser_ts.push(*ts.last().expect("parameters"));
                    stations = denser;
                    ts = denser_ts;
                }
                (stations, ts)
            }
            Some(_) => {
                let fraction = (t_to - t_from) / span;
                let intervals = ((stations.len().saturating_sub(1)) as f64 * fraction).ceil().max(ROLLED_PIECE_MIN_INTERVALS as f64) as usize;
                let mut stations = vec![first];
                let mut ts = vec![t_from];
                let seed_uv = [first.uv1[0], first.uv1[1], first.uv2[0], first.uv2[1]];
                // The crossing station itself solves on the swapped pair: the
                // same ball, the same contacts (the rolled contact is ON the
                // tangent edge), so the march continues from it.
                let mut left = Continued { t: t_from, uv: seed_uv, center: Some(first.center) };
                {
                    let check = frame.station(t_from, seed_uv)?;
                    let agreement = check.center.sub(first.center).length().max(check.p1.sub(first.p1).length()).max(check.p2.sub(first.p2).length());
                    if agreement > 1e-6 * scale {
                        return Err(KernelRefusal::non_convergence(
                            KernelStage::Refine,
                            "rollover_rib",
                            format!("blend rollover: the crossing station re-read on the neighbour moves by {agreement:.3e}"),
                        ));
                    }
                }
                // Lane N census row 5 (with `sided_ball_neighbour_on`): a
                // ROLLED piece that wraps past the march's end carries the
                // anchored seam too — the rim's seam vertex lies on the
                // rolled stretch (lCount_408: the round on the far side of
                // the boss from the cylinder's seam is crossed there). A
                // station is marched AT the seam parameter, between the
                // uniform ones, and the surgery splits the through rail
                // there as it does for a piece on the mates.
                let seam_t = (wraps && anchor_u.is_some() && crate::fillet::sided_ball_neighbour_on()).then_some(edge.t1).filter(|t| *t > t_from && *t < t_to);
                // A uniform station within a hair of the seam IS the seam
                // station: no second one beside it.
                let hair = 1e-9 * span.abs().max(1.0);
                let mut seam_pending = seam_t;
                for k in 1..intervals {
                    let t = t_from + (t_to - t_from) * k as f64 / intervals as f64;
                    if let Some(at) = seam_pending.filter(|at| *at <= t + hair) {
                        seam_pending = None;
                        if at < t - hair {
                            let (node, station) = frame.continue_from(&left, at)?;
                            stations.push(station);
                            ts.push(node.t);
                            left = node;
                        }
                    }
                    let (node, station) = frame.continue_from(&left, t)?;
                    stations.push(station);
                    ts.push(node.t);
                    left = node;
                }
                if let Some(at) = seam_pending {
                    let (node, station) = frame.continue_from(&left, at)?;
                    stations.push(station);
                    ts.push(node.t);
                }
                stations.push(last);
                ts.push(t_to);
                (stations, ts)
            }
        };
        for side in 0..2 {
            unwrap_track(&mut marched, side, surfaces[side])?;
        }
        // The seam station of the wrapping piece: the closed march's first
        // station, one past the run from the crossing's bracket to the
        // march's end.
        let seam_station = match anchor_u {
            Some(_) if index + 1 == count && rolled_side.is_none() => {
                let [_, start] = from.stations;
                Some(1 + (stations.len() - 1 - start))
            }
            // The rolled wrapping piece's seam station (above): found again
            // by its parameter point below, like every other.
            Some(_) if wraps && rolled_side.is_some() && crate::fillet::sided_ball_neighbour_on() => Some(0),
            _ => None,
        };
        // Refinement: the fitted rails against the carriers between
        // stations, the closed lane's reading, one midpoint per failing
        // interval and round.
        let mut rows = None;
        for round in 0..=REFINE_ROUNDS {
            let parameters = station_parameters(&marched);
            let fitted = super::edge::fit_open_rows(&marched, &parameters, true, None, None)?;
            let misses = closed_rail_interval_misses(&fitted, &marched, &parameters, surfaces)?;
            let failing: Vec<usize> = misses.iter().enumerate().filter(|(_, &miss)| !(miss <= rail_bar)).map(|(k, _)| k).collect();
            let worst = misses.iter().copied().fold(0.0_f64, f64::max);
            if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                eprintln!(
                    "closed-rim rollover: piece {index} ({}) round {round}: {} stations, {} interval(s) over the rail bar {rail_bar:.3e}, worst {worst:.3e}",
                    if rolled_side.is_some() { "rolled" } else { "on the mates" },
                    marched.len(),
                    failing.len()
                );
            }
            if failing.is_empty() {
                rows = Some(());
                break;
            }
            if round == REFINE_ROUNDS || marched.len() + failing.len() > ROLLED_PIECE_STATION_CEILING {
                return Err(KernelRefusal::non_convergence(
                    KernelStage::Refine,
                    super::miter::MARCHED_FIT_OFF_CARRIERS_WHAT,
                    format!(
                        "blend rollover: piece {index}'s rails stand {worst:.9e} off their carriers (bar {rail_bar:.9e}) after {} stations and {round} local rounds; bounded refinement exhausted",
                        marched.len()
                    ),
                ));
            }
            let mut refined = Vec::with_capacity(marched.len() + failing.len());
            let mut refined_ts = Vec::with_capacity(refined.capacity());
            for k in 0..marched.len() - 1 {
                refined.push(marched[k]);
                refined_ts.push(ts[k]);
                if failing.contains(&k) {
                    let left = Continued {
                        t: ts[k],
                        uv: [marched[k].uv1[0], marched[k].uv1[1], marched[k].uv2[0], marched[k].uv2[1]],
                        center: Some(marched[k].center),
                    };
                    let (node, station) = frame.continue_from(&left, 0.5 * (ts[k] + ts[k + 1]))?;
                    refined.push(station);
                    refined_ts.push(node.t);
                }
            }
            refined.push(*marched.last().expect("a piece has stations"));
            refined_ts.push(*ts.last().expect("a piece has parameters"));
            marched = refined;
            ts = refined_ts;
            for side in 0..2 {
                unwrap_track(&mut marched, side, surfaces[side])?;
            }
        }
        debug_assert!(rows.is_some());
        // The seam station's index moves with the insertions before it: find
        // it again by its parameter point on the anchored carrier.
        let seam_station = match (seam_station, anchor_u) {
            (Some(_), Some(lock)) => {
                let period = {
                    let [a, b] = surfaces[0].domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
                    b - a
                };
                marched.iter().enumerate().skip(1).find(|(k, station)| {
                    *k + 1 < marched.len() && ((station.uv1[0] - lock) / period - ((station.uv1[0] - lock) / period).round()).abs() * period <= 1e-9 * (1.0 + period)
                }).map(|(k, _)| k)
            }
            _ => None,
        };
        let parameters = station_parameters(&marched);
        let mut rows = super::edge::fit_open_rows(&marched, &parameters, chamfer, None, None)?;
        // The support pcurves interpolate the stations' (u, v) and are read
        // nowhere between them; each is refitted to its rail's own projected
        // track at the pcurve floor, as the closed lane refits its own.
        rows.cr_pcurve = super::edge::refit_closed_support_pcurve(&rows.cr, &rows.cr_pcurve, surfaces[0])?;
        rows.cs_pcurve = super::edge::refit_closed_support_pcurve(&rows.cs, &rows.cs_pcurve, surfaces[1])?;
        pieces.push(RolledPiece {
            faces: piece.faces,
            stations: marched,
            ts,
            rows,
            from: piece.from,
            to: piece.to,
            rolled_side,
            seam_station,
        });
    }
    Ok(pieces)
}

/// The rollover's body: the marched pieces sewn into the solid
/// (`rollover_surgery`). The pieces are handed to the test capture whatever
/// the surgery answers.
#[allow(clippy::too_many_arguments)]
pub(super) fn build(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    mates: [&BlendMate<'_>; 2],
    plan: &RolloverPlan,
    pieces: Vec<RolledPiece>,
    anchor_u: Option<f64>,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let built = super::rollover_surgery::rollover_surgery(solid, edge, mates, plan, &pieces, anchor_u, name);
    built
}

/// The named refusal for a track that rolls over: what crosses what, where,
/// and that the roll is not built.
pub(super) fn refusal(plan: &RolloverPlan, radius: f64) -> KernelRefusal {
    let crossings = &plan.crossings;
    let first = &crossings[0];
    let more = match crossings.len() {
        1 => String::new(),
        n => format!(" and {} more crossing(s)", n - 1),
    };
    let solved = if first.solved.is_some() { "the crossing station solved" } else { "located by the bracket's chord" };
    let rolled = plan.rolled.iter().filter(|seed| seed.is_some()).count();
    KernelRefusal::unsupported(
        KernelStage::Classify,
        TANGENT_ROLLOVER_WHAT,
        format!(
            "blend: the radius {radius} rail on face {} crosses the tangent edge {} onto face {} at \
             ({:.6}, {:.6}, {:.6}){more} ({solved}; {} piece(s); {rolled} seeded on the neighbour): re-marching the rolled pieces on the \
             neighbour and sewing them on the shared rib is not built, \
             so this lane declines",
            first.mate_face_id, first.edge_id, first.neighbour_face_id, first.point.x, first.point.y, first.point.z,
            plan.pieces.len()
        ),
    )
}

