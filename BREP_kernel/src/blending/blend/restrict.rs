//! Restricting a sewn stripe against a THIRD face it crosses.
//!
//! The march plus §6.9 surgery re-trims *the blended edge's two MATING faces
//! and nothing else*.  That is complete only while the swept volume — the
//! sliver a convex blend removes — touches nothing but those two mates.  Let a
//! third face cross it (a through hole under the rail, a pocket, a neighbouring
//! boss) and the blend wall comes out whole and running through the other face,
//! the mate keeps an inner loop that no longer lies inside its own trimmed
//! region, and the answer is self-intersecting.  `check_blend_interference`
//! (`fillet/analyze.rs`) states that precondition and refuses when it fails;
//! this module RESTRICTS the stripe instead, so the refusal — and the cutter
//! composition behind it — is not reached.
//!
//! # Where the crossing points come from
//!
//! Each end of the trim curve is the point where the blend surface, the mate
//! and the obstacle all meet: a three-surface point (Golovanov §4.5).  It is
//! not solved as one.  Two of the three pairwise intersection curves are
//! already in hand as TOPOLOGY — blend ∩ mate is the contact RAIL the march
//! just fitted, and mate ∩ obstacle is an EDGE the input solid already carried
//! — so the triple point drops to a curve-curve intersection between two
//! existing curves.  That is cheaper, better conditioned, and it lands the
//! point exactly ON both curves instead of a fit-accuracy away from each.
//!
//! An obstacle that meets NEITHER mate — a void wholly under the blend's arc
//! — has no such edge, but it has its OWN edges, and those cross the blend
//! surface: each crossing is again a curve/surface point, and the trim is a
//! closed chain of pieces through them (2026-09-26, [`plan_void`]).
//!
//! # The trim curve
//!
//! With both endpoints known exactly, the curve between them is TRACED by the
//! kernel's own surface/surface marcher, seeded once in the middle of the
//! crossing window.  The march stops on the blend carrier's parametric domain
//! boundary, and that boundary IS the rail, so the traced branch's two ends are
//! the two crossings — the curve is clipped to the face by construction rather
//! than by a filter, and it comes back with a parameter pair on each carrier.
//! The exact crossings are then committed over the traced endpoints, because a
//! curve-curve intersection of two existing curves is sharper than anything the
//! march produces.  A pcurve that jumps a large fraction of the obstacle's
//! domain crosses that carrier's seam and is refused by name for an OPEN piece;
//! the two closed shapes that legitimately wrap a seam — the eaten arc through
//! the rim's own seam foot, and a closed piece hung on the seam — build their
//! pcurve unwrapped and fold it (see below).
//!
//! A per-SECTION root solve was tried first — for each `u` across the window,
//! the single root of the obstacle's signed distance along that blend section
//! — and it is wrong at the ends.  On the measured through hole the trim curve
//! leaves each crossing and turns BACK in `u`: the bore is widest at z = 10,
//! half a millimetre past the rail at z = 9.5, so the curve has a vertical
//! tangent there and is not a graph over `u` at all.  A fit through per-section
//! roots cuts that turn off and converges at FIRST order — 1.7e-3 at 49
//! stations, still 5.7e-5 at 1483 — while the traced curve clears the kernel's
//! own SSI fit contract on its first pass.  The section solve survives as the
//! seed and as a structural guard: a section with no root, or more than one, is
//! a shape this construction does not describe and is refused by name.
//!
//! # What it rebuilds
//!
//! Three loops, sharing one new edge:
//!
//! * the BLEND's loop swaps the rail coedge for `rail-low, trim, rail-high`;
//! * the OBSTACLE's loop swaps the arc the blend ate for the same trim edge;
//! * the MATE's two loops MERGE — the rail's own loop and the loop the obstacle
//!   bounded become one, because the rail now runs into the hole and out of it.
//!
//! # The eaten arc through the rim's own vertex (2026-09-26)
//!
//! A circular rim is one closed edge whose start/end vertex foots the
//! obstacle's seam.  When the arc the blend eats is the COMPLEMENT of the run
//! between the crossings — which for a `+Z` bore under a blend toward `+Y` is
//! every crossing there is, because `make_cylinder_brep` seats the foot at
//! `cy + r` — the vertex goes with the arc and the seam is left footed on
//! nothing.  The run between the crossings is then the KEPT piece, the two
//! outer pieces go, the trim takes their place in the obstacle's loop, and
//! the loop is open at both seam coedges: the state a fully blended bore mouth
//! leaves when its rail wraps the carrier, and the same repair closes it
//! (`network::plan_carrier_seam_split` — the trim split at the meridian, the
//! seam trimmed to the crossing).  For that the trim's pcurve on the obstacle
//! is built UNWRAPPED across the seam ([`unwrapped_pcurve`]), from the
//! committed 3D curve's own samples.
//!
//! # The eaten run across several boundary edges (2026-09-26)
//!
//! A rail that crosses two DIFFERENT edges of one mate loop, once each, eats a
//! run of that loop: the end of one edge, whole edges, the start of another,
//! each with its own obstacle face across it.  The trim is then a CHAIN of
//! pieces, one per obstacle face, joined where consecutive obstacle faces'
//! shared edge crosses the blend — again an existing edge against the blend
//! surface, a curve/surface point ([`plan_chain`], [`apply_chain`]).  Each
//! piece is seeded on a junction, because an end piece on a wall normal to
//! the blended edge IS a blend section and the section-root seed above has no
//! sign change to find there.
//!
//! # A concave stripe's pad over a third face (2026-09-26)
//!
//! The pad closes OVER the obstacle: the mate loses the rim under the pad and
//! the obstacle's region is carried up onto the pad along it, so the obstacle
//! grows where a convex blend's shrinks.  A rim that straddles the pad's rail
//! is the single-edge class; a rim wholly under the pad is the void class
//! below with "under the arc" read on the pad's material side, the hole's
//! seam EXTENDED along its own line to the pad, and one closed piece hung on
//! that junction.
//!
//! # A closure over an obstacle (2026-09-26)
//!
//! Corner patches, chamfer facets and cones and horn sectors are not stripes
//! and have no rails; they go through the void lane below
//! ([`restrict_closures`]), with the faces they are sewn to as mates and
//! their convexity read from the original solid by majority over probes just
//! outside them.
//!
//! # The void that touches neither mate (2026-09-26)
//!
//! Found by the solid's edges against the blend surface, away from the rails.
//! What the sliver removes is read against the blend's outward normal: a
//! vertex whose foot lies inside the blend face and which sits on the outer
//! side is under the arc.  Edges with both ends under it go; edges with one
//! end under it and one crossing are junction edges, trimmed to the crossing;
//! each affected face's loop is walked with those eaten ends re-tagged, and
//! every break that appears is a gap closed by the piece of blend ∩ face
//! between its two junctions.  The blend gains an inner loop per cycle the
//! pieces close, and the void's shell merges into the blend's.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{
    build_pcurve_on_surface_range, intersect_curve_surface, intersect_curves, interpolate_curve,
    project_point_to_surface, KernelTolerances, NurbsCurve, Vec3,
};

use super::edge::trim_edge_at;

/// The faces one sewn stripe contributed, as the orchestrator knows them.
pub(in crate::blend) struct StripeFaces {
    pub(in crate::blend) blend_face: u64,
    pub(in crate::blend) mates: [u64; 2],
    /// Which mates the stripe consumed whole (`SewnStripe::consumed`).  A
    /// consumed mate is no longer in the solid, so there is nothing on it to
    /// restrict; it stays in `mates` because it is still not an obstacle.
    pub(in crate::blend) consumed: [bool; 2],
    /// A CONVEX stripe removes a sliver; a concave one adds a pad.  Both are
    /// restricted (the pad closes OVER the obstacle: the obstacle's region is
    /// carried up onto the pad instead of being notched, 2026-09-26).
    pub(in crate::blend) convex: bool,
    /// The blend's size (radius, or chamfer distance): everything the stripe
    /// adds or removes lies within this distance of its surface — the farthest
    /// point is the sharp edge itself, at `(√2 − 1)·r` for a right dihedral —
    /// which bounds what "under the arc" can mean for a void's vertices.
    pub(in crate::blend) size: f64,
}

/// How many obstacles one stripe/mate pair may carry before the search is
/// called a mis-detection rather than a shape.
const OBSTACLE_BUDGET: usize = 8;

/// Sections used only to prove the root along one blend section is UNIQUE.
const UNIQUENESS_SAMPLES: usize = 24;

/// Refinement passes, and the traced-point count at which the fit is called
/// impossible rather than under-refined.
const REFINEMENTS: usize = 7;
const STATION_BUDGET: usize = 40000;

/// Samples used to MEASURE a fit against its carriers.
const VERIFY: usize = 97;

/// Restrict every sewn stripe against the third faces it crosses.
///
/// `Ok(fragments)` lists the faces a restriction CUT OFF a mate (the own-loop
/// class splits its mate in two): pieces of an input face, not faces the
/// network built, so the caller must not take them for closures.  `Err` is a
/// NAMED refusal: a crossing was found that this construction does not
/// describe, and the caller should treat the whole network result as refused.
pub(in crate::blend) fn restrict_third_faces(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    stripes: &[StripeFaces],
    policy: &KernelTolerances,
) -> Result<Vec<u64>, KernelRefusal> {
    let mut fragments = Vec::new();
    for stripe in stripes {
        let mut mates = Vec::with_capacity(2);
        for side in 0..2 {
            let mate = stripe.mates[side];
            if !stripe.consumed[side] && !mates.contains(&mate) {
                mates.push(mate);
            }
        }
        for mate in mates {
            for _ in 0..OBSTACLE_BUDGET {
                let Some(plan) = plan_restriction(
                    result,
                    stripe.blend_face,
                    mate,
                    &stripe.mates,
                    stripe.convex,
                    stripe.size,
                    policy,
                )?
                else {
                    break;
                };
                match plan {
                    Plan::Single(plan) => fragments.extend(apply_restriction(result, take_id, plan)?),
                    Plan::Chain(plan) => apply_chain(result, take_id, plan)?,
                }
            }
        }
        // A void that touches NEITHER mate crosses the blend's interior without
        // crossing its rails: found by its EDGES against the blend surface.
        for _ in 0..OBSTACLE_BUDGET {
            let Some(plan) = plan_void(
                result,
                stripe.blend_face,
                &stripe.mates,
                stripe.convex,
                stripe.size,
                policy,
            )?
            else {
                break;
            };
            apply_void(result, take_id, plan)?;
        }
    }
    Ok(fragments)
}

/// What one pass of the planner found on a (blend, mate) pair.
enum Plan {
    /// The eaten run lies on ONE boundary edge of the mate.
    Single(Restriction),
    /// The eaten run spans several boundary edges of one mate loop, each with
    /// its own obstacle face; the trim is a CHAIN of pieces joined where the
    /// obstacle faces' shared edges cross the blend (2026-09-26).
    Chain(ChainRestriction),
}

/// One link of a chain restriction: one boundary edge of the mate loop the
/// blend eats (wholly, or from a crossing to its end) and the obstacle face
/// across it.
struct ChainLink {
    edge: u64,
    obstacle_face: u64,
    /// The run of the edge the blend ate, in the edge's own direction;
    /// `[t0, t1]` for an interior link.
    eaten: [f64; 2],
}

/// Where two consecutive obstacle faces' shared edge crosses the blend: the
/// eaten vertex between two links is replaced by this point.
struct Junction {
    edge: u64,
    /// The eaten vertex the junction edge ends on today.
    eaten_vertex: u64,
    parameter: f64,
    point: Vec3,
}

/// One traced piece of the chain, between two of `[crossing 0, junctions…,
/// crossing 1]`, with its pcurves on the blend and on its obstacle face.
struct ChainPiece {
    curve: NurbsCurve,
    on_blend: NurbsCurve,
    on_obstacle: NurbsCurve,
}

/// A crossing whose eaten run spans more than one boundary edge of the mate.
struct ChainRestriction {
    blend_face: u64,
    mate_face: u64,
    rail_edge: u64,
    rail_parameters: [f64; 2],
    rail_loop: usize,
    /// The crossing points, matched to `rail_parameters`.
    points: [Vec3; 2],
    /// Links in walk order from `points[0]` to `points[1]`.
    links: Vec<ChainLink>,
    /// `links.len() - 1` junctions, between consecutive links.
    junctions: Vec<Junction>,
    /// `links.len()` pieces, piece `i` on `links[i].obstacle_face`, running
    /// from `points[0]`/`junctions[i-1]` to `junctions[i]`/`points[1]`.
    pieces: Vec<ChainPiece>,
}

/// One obstacle, resolved against one (blend face, mate face) pair.
struct Restriction {
    blend_face: u64,
    mate_face: u64,
    obstacle_face: u64,
    /// The rail: the edge the blend and the mate share.
    rail_edge: u64,
    /// Where the obstacle's boundary crosses the rail, in the rail's own
    /// parameter, ordered.
    rail_parameters: [f64; 2],
    /// The mate's loop that carries the rail, and the loop the obstacle bounds.
    rail_loop: usize,
    obstacle_loop: usize,
    /// The obstacle's edge is on the rail's OWN loop: dropping the rail's
    /// middle and the eaten run leaves two closed chains, and the mate is cut
    /// into two faces instead of having two loops merged.
    split_mate: bool,
    /// The obstacle's boundary edge on the mate, its position in that loop, and
    /// the sub-range of it the blend ate.
    obstacle_edge: u64,
    obstacle_position: usize,
    /// The obstacle edge's parameters bounding the run of it the blend KEPT
    /// when `through_seam`, else the run it ate, in the edge's own direction.
    obstacle_parameters: [f64; 2],
    /// The eaten run is the COMPLEMENT of `obstacle_parameters` on a closed
    /// boundary edge: it passes through that edge's own start/end vertex,
    /// where the obstacle's seam edge is footed.  The seam is then re-footed
    /// on the trim curve by the carrier seam split (`network.rs`), the same
    /// repair a fully blended bore mouth gets when its rail wraps the carrier.
    through_seam: bool,
    /// The two crossing points, matched to `rail_parameters`.
    points: [Vec3; 2],
    /// The trim curve, its pcurve on the blend and its pcurve on the obstacle,
    /// all running from `points[0]` to `points[1]`.
    trim_curve: NurbsCurve,
    trim_on_blend: NurbsCurve,
    trim_on_obstacle: NurbsCurve,
    /// The worst distance the fitted trim curve sits from either carrier.
    deviation: f64,
}

/// The edges a face's loops use, with the loop each one sits in.
fn face_edges(face: &FaceRecord) -> Vec<(usize, usize, u64)> {
    face.loops
        .iter()
        .enumerate()
        .flat_map(|(loop_index, loop_record)| {
            loop_record
                .coedges
                .iter()
                .enumerate()
                .map(move |(position, coedge)| (loop_index, position, coedge.edge_id))
        })
        .collect()
}

fn face_of<'a>(solid: &'a BrepSolid, id: u64) -> Option<&'a FaceRecord> {
    solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == id)
}

fn edge_of<'a>(solid: &'a BrepSolid, id: u64) -> Option<&'a EdgeRecord> {
    solid.edges.iter().find(|edge| edge.id == id)
}

/// The outward normal of `face` at (u, v): the surface normal, flipped when the
/// face uses its carrier reversed.
fn outward_normal(face: &FaceRecord, u: f64, v: f64) -> Result<Vec3, KernelRefusal> {
    let normal = face.surface.normal(u, v).or_refuse(KernelStage::Refine, "normal")?;
    Ok(if face.same_sense { normal } else { normal.scale(-1.0) })
}

/// Signed distance from `point` to the obstacle's carrier: POSITIVE on the
/// material side (behind the face's outward normal is material, so a point
/// outside the solid across that face reads positive).
///
/// The sign is read from the face's own orientation rather than assumed, so a
/// bore (material outside the cylinder) and a boss (material inside it) are the
/// same expression.
fn material_side_distance(face: &FaceRecord, point: Vec3) -> Result<f64, KernelRefusal> {
    let projection = project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let normal = outward_normal(face, projection.u, projection.v)?;
    let offset = point.sub(projection.point);
    // `-` because the outward normal points AWAY from material.
    Ok(-offset.dot(normal))
}

/// Walk a loop's pcurves into one uv polygon, in traversal order.
///
/// A coedge's pcurve is already parameterized in that coedge's traversal
/// direction, so the polygon is the concatenation of each pcurve sampled from
/// its own domain start to its own domain end.
fn loop_polygon(face: &FaceRecord, loop_index: usize) -> Result<Vec<[f64; 2]>, KernelRefusal> {
    const PER_COEDGE: usize = 24;
    let mut polygon = Vec::new();
    for coedge in &face.loops[loop_index].coedges {
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        for step in 0..PER_COEDGE {
            let q = q0 + (q1 - q0) * step as f64 / PER_COEDGE as f64;
            let point = coedge.pcurve.evaluate(q).or_refuse(KernelStage::Refine, "evaluate")?;
            polygon.push([point.x, point.y]);
        }
    }
    if polygon.len() < 3 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "loop_polygon", "blend restriction: a loop has no uv polygon"));
    }
    Ok(polygon)
}

/// Even-odd parity of `point` against a uv polygon.
fn polygon_contains(polygon: &[[f64; 2]], point: [f64; 2]) -> bool {
    let mut inside = false;
    let mut previous = polygon[polygon.len() - 1];
    for current in polygon {
        let straddles = (current[1] > point[1]) != (previous[1] > point[1]);
        if straddles {
            let t = (point[1] - current[1]) / (previous[1] - current[1]);
            if point[0] < current[0] + t * (previous[0] - current[0]) {
                inside = !inside;
            }
        }
        previous = *current;
    }
    inside
}

/// The normalized position of `parameter` along an edge's own span.
fn along(edge: &EdgeRecord, parameter: f64) -> f64 {
    (parameter - edge.t0) / (edge.t1 - edge.t0)
}

/// The uv a coedge's pcurve reaches at `along` of its edge's span.
fn pcurve_at(coedge: &CoedgeRecord, along: f64) -> Result<[f64; 2], KernelRefusal> {
    let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let walked = if coedge.forward { along } else { 1.0 - along };
    let point = coedge.pcurve.evaluate(q0 + (q1 - q0) * walked).or_refuse(KernelStage::Refine, "evaluate")?;
    Ok([point.x, point.y])
}

/// Plan the next restriction for one (blend, mate) pair, or `Ok(None)` when
/// nothing on that mate crosses the blend's rail.
fn plan_restriction(
    solid: &BrepSolid,
    blend_face_id: u64,
    mate_face_id: u64,
    stripe_mates: &[u64; 2],
    convex: bool,
    size: f64,
    policy: &KernelTolerances,
) -> Result<Option<Plan>, KernelRefusal> {
    let blend = face_of(solid, blend_face_id)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "blend_face", "blend restriction: the blend face is missing"))?;
    let mate = face_of(solid, mate_face_id)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "mate_face", "blend restriction: the mate face is missing"))?;
    let blend_edges: Vec<u64> = face_edges(blend).into_iter().map(|(_, _, id)| id).collect();

    // Every rail: an edge the blend and the mate share.  After one obstacle has
    // been restricted the rail is in pieces, so this is a set, not a single id.
    let mut rails: Vec<(usize, usize, u64)> = face_edges(mate)
        .into_iter()
        .filter(|(_, _, id)| blend_edges.contains(id))
        .collect();
    rails.sort_by_key(|(_, _, id)| *id);

    for (rail_loop, _rail_position, rail_id) in rails {
        let rail = edge_of(solid, rail_id)
            .ok_or(KernelRefusal::internal(KernelStage::Refine, "rail_edge", "blend restriction: a rail edge vanished"))?;
        let band = policy.pcurve_consistency;
        let mut hits: Vec<(usize, usize, u64, f64, f64, Vec3)> = Vec::new();
        // Crossings of an edge of the rail's OWN loop: an obstacle that opens
        // the mate's outer boundary (a bore breaking out through the notch a
        // profile cut left) rather than sitting inside it as a hole.  Kept
        // apart, and read only when nothing else crosses the rail.
        let mut own_loop_hits: Vec<(usize, usize, u64, f64, f64, Vec3)> = Vec::new();
        for (loop_index, position, edge_id) in face_edges(mate) {
            if edge_id == rail_id {
                continue;
            }
            let other = edge_of(solid, edge_id)
                .ok_or(KernelRefusal::internal(KernelStage::Refine, "mate_boundary_edge", "blend restriction: a mate boundary edge vanished"))?;
            for crossing in intersect_curves(&rail.curve, &other.curve, band).or_refuse(KernelStage::Refine, "intersect_curves")? {
                let inside_rail = within(crossing.s, rail.t0, rail.t1);
                let inside_other = within(crossing.t, other.t0, other.t1);
                if !inside_rail || !inside_other {
                    continue;
                }
                // A TANGENTIAL meeting is not a crossing: the rail grazes the
                // obstacle's boundary and passes no material through it, so
                // the rails alone still trim the blend correctly.  This is the
                // pinched bore mouth, where the corner setbacks pull the top
                // face in until its boundary just touches the rim — refusing
                // it would drop a selection the network already builds.
                if crossing.tangential {
                    continue;
                }
                let hit = (loop_index, position, edge_id, crossing.s, crossing.t, crossing.point);
                if loop_index == rail_loop {
                    own_loop_hits.push(hit);
                } else {
                    hits.push(hit);
                }
            }
        }
        if hits.is_empty() {
            // The rail crosses ONE edge of its own loop twice: the run of that
            // edge between the crossings is eaten and the mate is cut in two
            // (2026-09-27).  Anything else on the own loop is left to
            // `check_blend_interference`, exactly as before this class.
            if own_loop_hits.len() == 2 && own_loop_hits[0].2 == own_loop_hits[1].2 {
                match plan_from_hits(
                    solid,
                    blend,
                    mate,
                    rail,
                    rail_loop,
                    &own_loop_hits,
                    stripe_mates,
                    convex,
                    Some(size),
                    policy,
                ) {
                    Ok(plan) => return Ok(Some(Plan::Single(plan))),
                    Err(reason) => {
                        if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                            eprintln!("blend restriction: own-loop crossing not resolved — {reason}");
                        }
                    }
                }
            }
            continue;
        }
        // One obstacle at a time: a rail may cross several, and each
        // restriction re-scans what is left of it.
        let mut edges: Vec<u64> = hits.iter().map(|hit| hit.2).collect();
        edges.sort_unstable();
        edges.dedup();
        // Two boundary edges of ONE loop crossed once each: the eaten run
        // spans several edges and the trim is a chain.
        if hits.len() == 2 && edges.len() == 2 && hits[0].0 == hits[1].0 {
            return plan_chain(
                solid,
                blend,
                mate,
                rail,
                rail_loop,
                [hits[0], hits[1]],
                stripe_mates,
                convex,
                policy,
            )
            .map(|plan| Some(Plan::Chain(plan)));
        }
        for edge_id in edges {
            let group: Vec<_> = hits.iter().copied().filter(|hit| hit.2 == edge_id).collect();
            if group.len() != 2 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "rail_obstacle_crossings", format!(
                    "blend restriction: the blend's rail {} crosses edge {edge_id} of face {} at \
                     {} points (expected two); a stripe that enters an obstacle without leaving \
                     it is not restricted directly",
                    rail.id,
                    mate.id,
                    group.len()
                )));
            }
        }
        let first = hits[0].2;
        let group: Vec<_> = hits.iter().copied().filter(|hit| hit.2 == first).collect();
        return plan_from_hits(
            solid,
            blend,
            mate,
            rail,
            rail_loop,
            &group,
            stripe_mates,
            convex,
            None,
            policy,
        )
        .map(|plan| Some(Plan::Single(plan)));
    }
    Ok(None)
}

/// Strictly inside a span, by a margin that keeps a crossing at a rail END out
/// of the class (there the trim would degenerate into the stripe's own cap).
fn within(parameter: f64, low: f64, high: f64) -> bool {
    let margin = 1e-6 * (high - low).abs();
    parameter > low + margin && parameter < high - margin
}

fn plan_from_hits(
    solid: &BrepSolid,
    blend: &FaceRecord,
    mate: &FaceRecord,
    rail: &EdgeRecord,
    rail_loop: usize,
    hits: &[(usize, usize, u64, f64, f64, Vec3)],
    stripe_mates: &[u64; 2],
    convex: bool,
    own_loop_size: Option<f64>,
    policy: &KernelTolerances,
) -> Result<Restriction, KernelRefusal> {
    if hits.len() != 2 {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "rail_obstacle_crossings", format!(
            "blend restriction: the blend's rail {} crosses the rest of face {} at {} points \
             (expected two); a stripe entering or leaving more than one obstacle is not \
             restricted directly",
            rail.id,
            mate.id,
            hits.len()
        )));
    }
    let (loop_a, position_a, edge_a, _, _, _) = hits[0];
    let (loop_b, position_b, edge_b, _, _, _) = hits[1];
    if loop_a != loop_b || edge_a != edge_b || position_a != position_b {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "rail_obstacle_span", format!(
            "blend restriction: the blend's rail {} enters face {}'s obstacle on edge {edge_a} \
             and leaves it on edge {edge_b}; a crossing spanning more than one boundary edge is \
             not restricted directly",
            rail.id, mate.id
        )));
    }
    // Order by the RAIL's parameter: the trim curve runs the way the rail does.
    let mut ordered = [hits[0], hits[1]];
    if ordered[0].3 > ordered[1].3 {
        ordered.swap(0, 1);
    }
    let rail_parameters = [ordered[0].3, ordered[1].3];
    let points = [ordered[0].5, ordered[1].5];
    if (rail_parameters[1] - rail_parameters[0]).abs() <= 1e-6 * (rail.t1 - rail.t0).abs() {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "rail_obstacle_tangent", format!(
            "blend restriction: the blend's rail {} touches face {}'s obstacle at one point \
             rather than crossing it; a tangential contact removes nothing and is not restricted",
            rail.id, mate.id
        )));
    }

    let obstacle_edge = edge_of(solid, edge_a)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "obstacle_boundary_edge", "blend restriction: the obstacle boundary edge vanished"))?;
    let closed = obstacle_edge.start_vertex_id == obstacle_edge.end_vertex_id;

    // Which run of the obstacle's boundary did the blend EAT?  The one outside
    // the rail's own loop: that loop is the mate's material after the surgery,
    // and the rail is its new boundary there.
    let low = ordered[0].4.min(ordered[1].4);
    let high = ordered[0].4.max(ordered[1].4);
    let inner_mid = 0.5 * (low + high);
    let (obstacle_parameters, through_seam) = if let Some(size) = own_loop_size {
        // On the rail's own loop that loop's polygon crosses itself (the rail
        // runs over the notch), so parity says nothing.  The BLEND decides:
        // the eaten run lies under its arc, and the edge's own ends — the
        // loop's vertices on either side — do not, or more of the loop than
        // this one edge went with it.
        let under = |parameter: f64| -> Result<Option<bool>, KernelRefusal> {
            let point = obstacle_edge.curve.evaluate(parameter).or_refuse(KernelStage::Refine, "evaluate")?;
            under_the_arc(blend, point, convex, size, policy)
        };
        if closed
            || under(inner_mid)? != Some(true)
            || under(obstacle_edge.t0)? == Some(true)
            || under(obstacle_edge.t1)? == Some(true)
        {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "own_loop_eaten_run", format!(
                "blend restriction: the run of edge {} between the rail crossings on face {} is \
                 not the only part of it under the blend; the mate is not cut in two by it",
                obstacle_edge.id, mate.id
            )));
        }
        ([low, high], false)
    } else if !polygon_contains(
        &loop_polygon(mate, rail_loop)?,
        pcurve_at(&mate.loops[loop_a].coedges[position_a], along(obstacle_edge, inner_mid))?,
    ) {
        ([low, high], false)
    } else if closed
        && !polygon_contains(
            &loop_polygon(mate, rail_loop)?,
            pcurve_at(&mate.loops[loop_a].coedges[position_a], along(obstacle_edge, obstacle_edge.t0))?,
        )
    {
        // The complement of [low, high] on a closed edge runs through its own
        // start/end vertex, the foot of the obstacle's seam.  The kept run is
        // [low, high]; the eaten one takes the vertex with it and the seam is
        // re-footed on the trim curve where it crosses the seam meridian.
        // Measured 2026-09-26 on the through-hole document with the bore
        // along +Z instead of +Y: `make_cylinder_brep` puts the rim's seam
        // foot at `cy + r`, inside every eaten arc that crosses the rail at
        // all, and the identical crossing trimmed cleanly with the foot on
        // the kept side.  One class, two seam positions.
        ([low, high], true)
    } else {
        return Err(KernelRefusal::internal(KernelStage::Refine, "obstacle_boundary", format!(
            "blend restriction: neither run of edge {} between the rail crossings lies outside \
             the mate's own loop on face {}; the obstacle's boundary is not resolved",
            obstacle_edge.id, mate.id
        )));
    };

    // The obstacle: the OTHER face using that boundary edge.
    let mut obstacle_faces: Vec<u64> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face.id != mate.id)
        .filter(|face| {
            face.loops
                .iter()
                .any(|l| l.coedges.iter().any(|c| c.edge_id == obstacle_edge.id))
        })
        .map(|face| face.id)
        .collect();
    obstacle_faces.sort_unstable();
    obstacle_faces.dedup();
    let [obstacle_face] = obstacle_faces[..] else {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "obstacle_faces", format!(
            "blend restriction: edge {} is shared by {} faces besides the mate; the obstacle is \
             not a single face",
            obstacle_edge.id,
            obstacle_faces.len()
        )));
    };
    if obstacle_face == blend.id || stripe_mates.contains(&obstacle_face) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "obstacle_own_support", format!(
            "blend restriction: the face across edge {} is the stripe's own blend or mate ({}), \
             not a third face; a stripe crossing its own supports is not restricted here",
            obstacle_edge.id, obstacle_face
        )));
    }
    // A CONCAVE stripe's pad over the obstacle is the same three-loop
    // rebuild (2026-09-26): the mate loses the arc under the pad, the
    // obstacle's region along that arc is carried UP onto the pad surface —
    // its carrier is untrimmed, so the trim lies within it — and the blend
    // takes the trim for its rail's middle.  The obstacle grows where a
    // convex one shrinks; nothing below distinguishes the two.
    let obstacle = face_of(solid, obstacle_face)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "obstacle_face", "blend restriction: the obstacle face is missing"))?;

    let (trim_curve, trim_on_blend, trim_on_obstacle, deviation) = solve_trim_curve(
        blend,
        obstacle,
        rail,
        rail_parameters,
        points,
        through_seam,
        policy,
    )?;

    Ok(Restriction {
        blend_face: blend.id,
        mate_face: mate.id,
        obstacle_face,
        rail_edge: rail.id,
        rail_parameters,
        rail_loop,
        obstacle_loop: loop_a,
        split_mate: own_loop_size.is_some(),
        obstacle_edge: obstacle_edge.id,
        obstacle_position: position_a,
        obstacle_parameters,
        through_seam,
        points,
        trim_curve,
        trim_on_blend,
        trim_on_obstacle,
        deviation,
    })
}

/// The uv a coedge's pcurve reaches at its walk's start or end.
fn walk_uv(coedge: &CoedgeRecord, at_end: bool) -> Result<[f64; 2], KernelRefusal> {
    let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let point = coedge.pcurve.evaluate(if at_end { q1 } else { q0 }).or_refuse(KernelStage::Refine, "evaluate")?;
    Ok([point.x, point.y])
}

/// The vertex a coedge's walk starts or ends on.
fn walk_vertex(solid: &BrepSolid, coedge: &CoedgeRecord, at_end: bool) -> Result<u64, KernelRefusal> {
    let edge = edge_of(solid, coedge.edge_id)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "missing_edge", "blend restriction: a loop references a missing edge"))?;
    Ok(if coedge.forward == at_end { edge.end_vertex_id } else { edge.start_vertex_id })
}

/// The one face besides `mate` that uses `edge_id`, checked to be a third face.
fn obstacle_across(
    solid: &BrepSolid,
    mate: &FaceRecord,
    blend: &FaceRecord,
    stripe_mates: &[u64; 2],
    edge_id: u64,
) -> Result<u64, KernelRefusal> {
    let mut faces: Vec<u64> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face.id != mate.id)
        .filter(|face| face.loops.iter().any(|l| l.coedges.iter().any(|c| c.edge_id == edge_id)))
        .map(|face| face.id)
        .collect();
    faces.sort_unstable();
    faces.dedup();
    let [face] = faces[..] else {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "obstacle_faces", format!(
            "blend restriction: edge {edge_id} is shared by {} faces besides the mate; the \
             obstacle is not a single face",
            faces.len()
        )));
    };
    if face == blend.id || stripe_mates.contains(&face) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "obstacle_own_support", format!(
            "blend restriction: the face across edge {edge_id} is the stripe's own blend or mate \
             ({face}), not a third face; a stripe crossing its own supports is not restricted here"
        )));
    }
    Ok(face)
}

/// Plan a CHAIN restriction: the rail crosses two different boundary edges of
/// one mate loop, once each, so the run it eats spans several edges — a
/// rectangular pocket's corner under the rail, where the eaten run is the end
/// of one wall's rim, the whole of the next wall's, and the start of the
/// third's.  Each link has its own obstacle face, consecutive obstacle faces
/// share the edge that leaves the eaten vertex between them, and that edge's
/// crossing of the blend is where the trim changes faces.  The junction is a
/// curve/surface point (an existing edge against the blend), not a
/// three-surface solve, for the reason the module header gives.
#[allow(clippy::too_many_arguments)]
fn plan_chain(
    solid: &BrepSolid,
    blend: &FaceRecord,
    mate: &FaceRecord,
    rail: &EdgeRecord,
    rail_loop: usize,
    hits: [(usize, usize, u64, f64, f64, Vec3); 2],
    stripe_mates: &[u64; 2],
    convex: bool,
    policy: &KernelTolerances,
) -> Result<ChainRestriction, KernelRefusal> {
    if !convex {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "concave_stripe_obstacles", format!(
            "blend restriction: the concave stripe on face {} crosses several boundary edges of \
             face {}; a pad that runs over a third face closes OVER it rather than being notched \
             by it, and is not restricted directly",
            blend.id, mate.id
        )));
    }
    let mut ordered = hits;
    if ordered[0].3 > ordered[1].3 {
        ordered.swap(0, 1);
    }
    let rail_parameters = [ordered[0].3, ordered[1].3];
    let points = [ordered[0].5, ordered[1].5];
    let loop_index = ordered[0].0;
    let loop_record = &mate.loops[loop_index];
    let count = loop_record.coedges.len();
    let (pos_a, pos_b) = (ordered[0].1, ordered[1].1);
    let interior_from = |step: isize| -> Vec<usize> {
        let mut run = Vec::new();
        let mut at = (pos_a as isize + step).rem_euclid(count as isize) as usize;
        while at != pos_b {
            run.push(at);
            at = (at as isize + step).rem_euclid(count as isize) as usize;
        }
        run
    };
    let ahead = interior_from(1);
    let behind = interior_from(-1);
    // Which way round the loop is the eaten run?  The one that lies OUTSIDE
    // the mate's rail loop: an interior edge's midpoint, or for two adjacent
    // links the vertex they share, tested against the rail loop's polygon.
    let polygon = loop_polygon(mate, rail_loop)?;
    let outside = |interior: &[usize], increasing: bool| -> Result<bool, KernelRefusal> {
        let uv = match interior.first() {
            Some(&first) => pcurve_at(&loop_record.coedges[first], 0.5)?,
            None => walk_uv(&loop_record.coedges[pos_a], increasing)?,
        };
        Ok(!polygon_contains(&polygon, uv))
    };
    let (interior, increasing) = match (outside(&ahead, true)?, outside(&behind, false)?) {
        (true, false) => (ahead, true),
        (false, true) => (behind, false),
        (both_out, _) => {
            return Err(KernelRefusal::internal(KernelStage::Refine, "eaten_run", format!(
                "blend restriction: the blend's rail {} crosses edges {} and {} of face {}, but \
                 {} run of the loop between them lies outside the mate's rail loop; the eaten \
                 run is not resolved",
                rail.id,
                ordered[0].2,
                ordered[1].2,
                mate.id,
                if both_out { "each" } else { "neither" }
            )));
        }
    };
    let positions: Vec<usize> =
        std::iter::once(pos_a).chain(interior).chain(std::iter::once(pos_b)).collect();

    // The links: edge, obstacle face, eaten run.
    let mut links = Vec::with_capacity(positions.len());
    for (index, &position) in positions.iter().enumerate() {
        let coedge = &loop_record.coedges[position];
        let edge = edge_of(solid, coedge.edge_id)
            .ok_or(KernelRefusal::internal(KernelStage::Refine, "mate_boundary_edge", "blend restriction: a mate boundary edge vanished"))?;
        let obstacle_face = obstacle_across(solid, mate, blend, stripe_mates, edge.id)?;
        // The walk leaves a link at its walk END when the run goes with the
        // loop, at its walk START against it; the eaten run of the first link
        // is from the crossing to that end, of the last from the arrival end
        // to the crossing.
        let leaves_at_end = increasing;
        let eaten = if index == 0 {
            let t = ordered[0].4;
            if coedge.forward == leaves_at_end { [t, edge.t1] } else { [edge.t0, t] }
        } else if index + 1 == positions.len() {
            let t = ordered[1].4;
            if coedge.forward == leaves_at_end { [edge.t0, t] } else { [t, edge.t1] }
        } else {
            [edge.t0, edge.t1]
        };
        links.push(ChainLink { edge: edge.id, obstacle_face, eaten });
    }
    for pair in links.windows(2) {
        if pair[0].obstacle_face == pair[1].obstacle_face {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "obstacle_run", format!(
                "blend restriction: consecutive edges {} and {} of face {}'s eaten run both \
                 belong to obstacle face {}; a trim that stays on one face across a boundary \
                 edge is not restricted directly",
                pair[0].edge, pair[1].edge, mate.id, pair[0].obstacle_face
            )));
        }
    }

    // The junctions: the edge the two obstacle faces share at each eaten
    // vertex, and where it crosses the blend.
    let mut junctions = Vec::with_capacity(links.len() - 1);
    for (index, pair) in positions.windows(2).enumerate() {
        let vertex = walk_vertex(solid, &loop_record.coedges[pair[0]], increasing)?;
        let (face_a, face_b) = (links[index].obstacle_face, links[index + 1].obstacle_face);
        let uses = |face_id: u64, edge_id: u64| -> bool {
            face_of(solid, face_id).is_some_and(|face| {
                face.loops.iter().any(|l| l.coedges.iter().any(|c| c.edge_id == edge_id))
            })
        };
        let mut candidates: Vec<&EdgeRecord> = solid
            .edges
            .iter()
            .filter(|edge| edge.start_vertex_id == vertex || edge.end_vertex_id == vertex)
            .filter(|edge| edge.id != links[index].edge && edge.id != links[index + 1].edge)
            .filter(|edge| uses(face_a, edge.id) && uses(face_b, edge.id))
            .collect();
        candidates.dedup_by_key(|edge| edge.id);
        let [junction_edge] = candidates[..] else {
            return Err(KernelRefusal::internal(KernelStage::Refine, "eaten_junction", format!(
                "blend restriction: {} edges leave the eaten vertex {vertex} of face {} shared \
                 by obstacle faces {face_a} and {face_b} (expected one); the junction between \
                 the two trim pieces is not resolved",
                candidates.len(),
                mate.id
            )));
        };
        let mut crossings: Vec<(f64, Vec3)> =
            intersect_curve_surface(&junction_edge.curve, &blend.surface, policy.intersection_fit).or_refuse(KernelStage::Refine, "intersect_curve_surface")?
                .into_iter()
                .filter(|hit| within(hit.t, junction_edge.t0, junction_edge.t1))
                .map(|hit| (hit.t, hit.point))
                .collect();
        crossings.dedup_by(|a, b| (a.0 - b.0).abs() <= 1e-9 * (junction_edge.t1 - junction_edge.t0).abs());
        let [(parameter, _)] = crossings[..] else {
            return Err(KernelRefusal::internal(KernelStage::Refine, "eaten_junction", format!(
                "blend restriction: edge {} leaving the eaten vertex {vertex} crosses the blend \
                 face {} {} times inside itself (expected once); the junction is not resolved",
                junction_edge.id,
                blend.id,
                crossings.len()
            )));
        };
        junctions.push(Junction {
            edge: junction_edge.id,
            eaten_vertex: vertex,
            parameter,
            point: junction_edge.curve.evaluate(parameter).or_refuse(KernelStage::Refine, "evaluate")?,
        });
    }

    // The pieces, each seeded on a junction — a point that lies exactly on
    // both surfaces and strictly inside the blend face — and clipped to its
    // two ends.
    let mut pieces = Vec::with_capacity(links.len());
    for (index, link) in links.iter().enumerate() {
        let obstacle = face_of(solid, link.obstacle_face)
            .ok_or(KernelRefusal::internal(KernelStage::Refine, "obstacle_face", "blend restriction: an obstacle face is missing"))?;
        let start = if index == 0 { points[0] } else { junctions[index - 1].point };
        let end = if index + 1 == links.len() { points[1] } else { junctions[index].point };
        let seed = if index < junctions.len() {
            junctions[index].point
        } else {
            junctions[index - 1].point
        };
        pieces.push(trace_piece(blend, obstacle, [start, end], seed, policy)?);
    }

    Ok(ChainRestriction {
        blend_face: blend.id,
        mate_face: mate.id,
        rail_edge: rail.id,
        rail_parameters,
        rail_loop,
        points,
        links,
        junctions,
        pieces,
    })
}

/// Trace one piece of a chain between two known endpoints, seeded at `seed`,
/// clipped to the ends by projecting each onto the traced polyline's
/// segments, fitted, and verified against both carriers.
fn trace_piece(
    blend: &FaceRecord,
    obstacle: &FaceRecord,
    ends: [Vec3; 2],
    seed: Vec3,
    policy: &KernelTolerances,
) -> Result<ChainPiece, KernelRefusal> {
    let fit_band = policy.intersection_fit;
    let drift_band = policy.pcurve_consistency;
    // The first step is a fraction of the piece's reach: its ends' distance,
    // or for a CLOSED piece (both ends the one junction) the obstacle
    // carrier's extent — a bore mouth traced at a 32nd of nothing came back
    // as 1193 points, and the march of a later blend along that edge could
    // not converge on it.
    let reach = {
        let (low, high) = surface_box(&obstacle.surface)?;
        let extent = high.sub(low).length() / 4.0;
        ends[0].sub(ends[1]).length().max(extent).max(1.0)
    };
    let mut step = reach / 32.0;
    let mut worst = f64::INFINITY;
    let mut traced = 0usize;
    for _pass in 0..REFINEMENTS {
        let options = crate::SurfaceIntersectionOptions {
            tolerance: fit_band,
            seed_points: vec![seed],
            seed_only: true,
            maximum_step: Some(step),
            maximum_steps: STATION_BUDGET,
            ..Default::default()
        };
        let branches = crate::intersect_surfaces(&blend.surface, &obstacle.surface, &options).or_refuse(KernelStage::Refine, "intersect_surfaces")?;
        let [branch] = &branches[..] else {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "trim_branches", format!(
                "blend restriction: the blend and obstacle face {} meet in {} branches through \
                 the chain's junction; a trim piece that is not one arc is not restricted directly",
                obstacle.id,
                branches.len()
            )));
        };
        if branch.points.len() < 2 {
            return Err(KernelRefusal::internal(KernelStage::Refine, "trim_piece_points", format!(
                "blend restriction: the trim piece traced against obstacle face {} has {} points",
                obstacle.id,
                branch.points.len()
            )));
        }
        let spatial = clip_polyline(&branch.points, branch.closed, ends, drift_band)?;
        traced = spatial.len();
        let parameters = chord_parameters(&spatial)?;
        let degree = 3.min(spatial.len() - 1);
        let curve = interpolate_curve(&spatial, degree, &parameters).or_refuse(KernelStage::Refine, "interpolate_curve")?;
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let on_blend =
            build_pcurve_on_surface_range(&blend.surface, &curve, t0, t1, true, fit_band).or_refuse(KernelStage::Refine, "build_pcurve_on_surface_range")?;
        let closed_piece = ends[0].sub(ends[1]).length() <= drift_band;
        let periodic = {
            let (u, v) = obstacle.surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
            u || v
        };
        // A closed piece round a periodic carrier wraps its seam once: built
        // unwrapped from the curve's own samples and folded into the domain.
        let on_obstacle = if closed_piece && periodic {
            fold_into_domain(&obstacle.surface, unwrapped_pcurve(&obstacle.surface, &curve, spatial.len(), degree)?)?
        } else {
            build_pcurve_on_surface_range(&obstacle.surface, &curve, t0, t1, true, fit_band).or_refuse(KernelStage::Refine, "build_pcurve_on_surface_range")?
        };
        let [ou0, ou1] = obstacle.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [ov0, ov1] = obstacle.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let [op0, op1] = on_obstacle.domain().or_refuse(KernelStage::Refine, "domain")?;
        for index in 0..VERIFY {
            if closed_piece && periodic {
                break;
            }
            let a = on_obstacle.evaluate(op0 + (op1 - op0) * index as f64 / VERIFY as f64).or_refuse(KernelStage::Refine, "evaluate")?;
            let b = on_obstacle.evaluate(op0 + (op1 - op0) * (index + 1) as f64 / VERIFY as f64).or_refuse(KernelStage::Refine, "evaluate")?;
            if (b.x - a.x).abs() > 0.25 * (ou1 - ou0).abs() || (b.y - a.y).abs() > 0.25 * (ov1 - ov0).abs()
            {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "trim_seam_crossing", format!(
                    "blend restriction: a chain piece's pcurve on obstacle face {} jumps a large \
                     fraction of its domain, so it crosses that carrier's seam; a seam-crossing \
                     trim piece is not restricted directly",
                    obstacle.id
                )));
            }
        }
        let mut deviation: f64 = 0.0;
        let mut drift: f64 = 0.0;
        for index in 0..=VERIFY {
            let fraction = index as f64 / VERIFY as f64;
            let on_curve = curve.evaluate(t0 + (t1 - t0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
            for (surface, pcurve) in [(&blend.surface, &on_blend), (&obstacle.surface, &on_obstacle)] {
                deviation = deviation.max(project_point_to_surface(surface, on_curve).or_refuse(KernelStage::Refine, "project_point_to_surface")?.distance);
                let [p0, p1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                let node = pcurve.evaluate(p0 + (p1 - p0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
                let (u, v) = wrap_into_domain(surface, node.x, node.y)?;
                drift = drift.max(surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?.sub(on_curve).length());
            }
        }
        worst = deviation.max(drift);
        if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
            eprintln!(
                "blend restriction: chain piece on face {} traced {traced} points at step \
                 {step:.4} — off-carrier {deviation:.3e} drift {drift:.3e} (bands {fit_band:.3e} \
                 / {drift_band:.3e})",
                obstacle.id
            );
        }
        if deviation <= fit_band && drift <= drift_band {
            return Ok(ChainPiece { curve, on_blend, on_obstacle });
        }
        step *= 0.5;
    }
    Err(KernelRefusal::non_convergence(KernelStage::Refine, "trim_piece_fit", format!(
        "blend restriction: a chain piece against obstacle face {} could not be fitted to its \
         carriers — {traced} traced points still miss by {worst:.3e}, past the {fit_band:.3e} \
         an SSI fit is allowed",
        obstacle.id
    )))
}

/// Clip a traced polyline to the run between two points that lie on it (to
/// within `band`): each end is projected onto the polyline's SEGMENTS, the
/// segment is cut there, and the exact end is committed — cutting at the
/// nearest vertex instead would bend the last span by up to half a step.
fn clip_polyline(
    polyline: &[Vec3],
    closed: bool,
    ends: [Vec3; 2],
    band: f64,
) -> Result<Vec<Vec3>, KernelRefusal> {
    let n = polyline.len();
    let segments = if closed { n } else { n - 1 };
    // (segment index, fraction along it, distance) of each end's foot.
    let foot = |point: Vec3| -> (usize, f64, f64) {
        let mut best = (0usize, 0.0f64, f64::INFINITY);
        for index in 0..segments {
            let a = polyline[index];
            let b = polyline[(index + 1) % n];
            let ab = b.sub(a);
            let length2 = ab.dot(ab);
            let fraction = if length2 > 0.0 { (point.sub(a).dot(ab) / length2).clamp(0.0, 1.0) } else { 0.0 };
            let on = a.add(ab.scale(fraction));
            let distance = on.sub(point).length();
            if distance < best.2 {
                best = (index, fraction, distance);
            }
        }
        best
    };
    let feet = [foot(ends[0]), foot(ends[1])];
    // A CLOSED piece: both ends are one point on a closed branch, and the
    // piece is the whole branch, walked from that foot round to it again.
    if closed && ends[0].sub(ends[1]).length() <= band {
        if feet[0].2 > band {
            return Err(KernelRefusal::internal(KernelStage::Refine, "trim_loop_junction", format!(
                "blend restriction: the traced trim loop passes {:.3e} from its junction (band \
                 {band:.3e}); the loop does not reach it",
                feet[0].2
            )));
        }
        let mut spatial = vec![ends[0]];
        let mut index = feet[0].0;
        for _ in 0..n {
            index = (index + 1) % n;
            let point = polyline[index];
            if point.sub(*spatial.last().unwrap()).length() > band && point.sub(ends[0]).length() > band {
                spatial.push(point);
            }
        }
        spatial.push(ends[0]);
        if spatial.len() < 4 {
            return Err(KernelRefusal::internal(KernelStage::Refine, "trim_loop", "blend restriction: the trim loop collapsed"));
        }
        return Ok(spatial);
    }
    for (index, (_, _, distance)) in feet.iter().enumerate() {
        if *distance > band {
            return Err(KernelRefusal::internal(KernelStage::Refine, "trim_piece_end", format!(
                "blend restriction: the traced trim piece passes {distance:.3e} from its end \
                 ({:.6}, {:.6}, {:.6}) (band {band:.3e}); the piece does not reach that end",
                ends[index].x, ends[index].y, ends[index].z
            )));
        }
    }
    // Walk from foot 0 to foot 1 the way that stays on the polyline; on an
    // open branch that is the parameter order, on a closed one the shorter way.
    let key = |f: &(usize, f64, f64)| f.0 as f64 + f.1;
    let (first, second, reversed) = if key(&feet[0]) <= key(&feet[1]) {
        (feet[0], feet[1], false)
    } else {
        (feet[1], feet[0], true)
    };
    let mut run: Vec<Vec3> = Vec::new();
    let mut index = first.0;
    let last = second.0;
    loop {
        let next = (index + 1) % n;
        if index == last {
            break;
        }
        run.push(polyline[next]);
        index = next;
        if run.len() > n {
            return Err(KernelRefusal::internal(KernelStage::Refine, "trim_piece_clip", "blend restriction: the trim piece's clip walked the whole branch"));
        }
    }
    // Drop a vertex that the exact end would sit on top of.
    let mut spatial = Vec::with_capacity(run.len() + 2);
    spatial.push(if reversed { ends[1] } else { ends[0] });
    for point in run {
        if point.sub(*spatial.last().unwrap()).length() > band {
            spatial.push(point);
        }
    }
    let far = if reversed { ends[0] } else { ends[1] };
    if spatial.len() > 1 && far.sub(*spatial.last().unwrap()).length() <= band {
        spatial.pop();
    }
    spatial.push(far);
    if reversed {
        spatial.reverse();
    }
    if spatial.len() < 2 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "trim_piece", "blend restriction: the trim piece collapsed to a point"));
    }
    Ok(spatial)
}

/// Solve the trim curve in the blend's own domain, between two known endpoints.
///
/// `through_seam`: the curve crosses the obstacle carrier's seam meridian, so
/// its pcurve on the obstacle is built UNWRAPPED — continuous across the
/// seam, running outside the carrier's u domain by up to one period — for
/// the carrier seam split to fold.  A fit through wrapped samples would be
/// a fit through a jump.
#[allow(clippy::too_many_arguments)]
fn solve_trim_curve(
    blend: &FaceRecord,
    obstacle: &FaceRecord,
    rail: &EdgeRecord,
    rail_parameters: [f64; 2],
    points: [Vec3; 2],
    through_seam: bool,
    policy: &KernelTolerances,
) -> Result<(NurbsCurve, NurbsCurve, NurbsCurve, f64), KernelRefusal> {
    // The rail on the BLEND is an isoparm of its carrier: one of the two
    // parameters is constant along it, and that constant is the trim curve's
    // own start and end.
    let rail_coedge = blend
        .loops
        .iter()
        .flat_map(|l| &l.coedges)
        .find(|c| c.edge_id == rail.id)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "blend_rail", "blend restriction: the blend face does not use its own rail"))?;
    let ends = [
        pcurve_at(rail_coedge, along(rail, rail_parameters[0]))?,
        pcurve_at(rail_coedge, along(rail, rail_parameters[1]))?,
    ];
    let [du0, du1] = blend.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [dv0, dv1] = blend.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let span_u = du1 - du0;
    let span_v = dv1 - dv0;
    let iso_band = 1e-6;
    let along_u = (ends[0][1] - ends[1][1]).abs() <= iso_band * span_v.abs();
    let along_v = (ends[0][0] - ends[1][0]).abs() <= iso_band * span_u.abs();
    if along_u == along_v {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "rail_isoparm", format!(
            "blend restriction: the contact rail {} is not an isoparm of the blend carrier \
             (its pcurve runs ({:.9}, {:.9}) to ({:.9}, {:.9})); the trim curve needs the \
             blend's own section direction",
            rail.id, ends[0][0], ends[0][1], ends[1][0], ends[1][1]
        )));
    }
    // `section` is the constant coordinate of the rail; `run` varies along it.
    let section_is_v = along_u;
    let section0 = if section_is_v { ends[0][1] } else { ends[0][0] };
    let (section_low, section_high) = if section_is_v { (dv0, dv1) } else { (du0, du1) };
    // The far side of the section: the OTHER rail's end of the domain.
    let section1 = if (section0 - section_low).abs() >= (section0 - section_high).abs() {
        section_low
    } else {
        section_high
    };
    let run = [
        if section_is_v { ends[0][0] } else { ends[0][1] },
        if section_is_v { ends[1][0] } else { ends[1][1] },
    ];

    let at = |run_value: f64, section_value: f64| -> Result<Vec3, KernelRefusal> {
        if section_is_v {
            blend.surface.evaluate(run_value, section_value).or_refuse(KernelStage::Refine, "evaluate")
        } else {
            blend.surface.evaluate(section_value, run_value).or_refuse(KernelStage::Refine, "evaluate")
        }
    };
    // The parameters must reproduce the crossing points, or the rail's pcurve
    // and its 3D fit disagree and everything below is built on the wrong uv.
    let seat_band = (policy.pcurve_consistency * 10.0).max(1e-9);
    for (index, point) in points.iter().enumerate() {
        let seated = at(run[index], section0)?;
        let miss = seated.sub(*point).length();
        if miss > seat_band {
            return Err(KernelRefusal::internal(KernelStage::Refine, "rail_seat", format!(
                "blend restriction: the rail crossing at ({:.6}, {:.6}, {:.6}) sits {miss:.3e} \
                 from the blend carrier's own point there (band {seat_band:.3e}); the rail's \
                 pcurve and its 3D fit disagree too far to trim against",
                point.x, point.y, point.z
            )));
        }
    }

    // Each section: one root of the obstacle's signed distance, and only one.
    let root_on_section = |run_value: f64| -> Result<f64, KernelRefusal> {
        let mut previous = material_side_distance(obstacle, at(run_value, section0)?)?;
        if previous >= 0.0 {
            return Err(KernelRefusal::internal(KernelStage::Refine, "crossing_window", format!(
                "blend restriction: the blend's rail at run {run_value:.6} is already on the \
                 material side of the obstacle, so the crossing window is not the region the \
                 blend ate"
            )));
        }
        let mut bracket: Option<[f64; 2]> = None;
        let mut crossings = 0usize;
        for step in 1..=UNIQUENESS_SAMPLES {
            let s = section0 + (section1 - section0) * step as f64 / UNIQUENESS_SAMPLES as f64;
            let value = material_side_distance(obstacle, at(run_value, s)?)?;
            if (value >= 0.0) != (previous >= 0.0) {
                crossings += 1;
                if bracket.is_none() {
                    let previous_s =
                        section0 + (section1 - section0) * (step - 1) as f64 / UNIQUENESS_SAMPLES as f64;
                    bracket = Some([previous_s, s]);
                }
            }
            previous = value;
        }
        let Some([mut low, mut high]) = bracket else {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "severed_stripe", format!(
                "blend restriction: the blend's section at run {run_value:.6} never reaches the \
                 material side of the obstacle; the obstacle cuts the whole section and the \
                 stripe would be severed, which is not restricted directly"
            )));
        };
        if crossings != 1 {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "section_crossings", format!(
                "blend restriction: the blend's section at run {run_value:.6} crosses the \
                 obstacle {crossings} times; only a single crossing per section is restricted \
                 directly"
            )));
        }
        // Bisection: the bracket is a genuine sign change, so it converges
        // without a derivative and cannot leave the section.
        for _ in 0..80 {
            let middle = 0.5 * (low + high);
            let value = material_side_distance(obstacle, at(run_value, middle)?)?;
            if value >= 0.0 {
                high = middle;
            } else {
                low = middle;
            }
            if (high - low).abs() <= 1e-15 * (section1 - section0).abs().max(1.0) {
                break;
            }
        }
        Ok(0.5 * (low + high))
    };

    // The curve between the two crossings is TRACED, not solved section by
    // section.  A section solve was tried first and is wrong at the ends: on
    // the measured through hole the trim curve leaves each crossing and turns
    // BACK in the run parameter — the bore's widest point sits at z = 10, just
    // past the rail at z = 9.5 — so it has a vertical tangent there and is not
    // a graph over the run at all.  A fit through per-section roots cuts that
    // turn off and converges at first order (1.7e-3 at 49 stations, still
    // 5.7e-5 at 1483).  The kernel's own marcher walks arc length instead and
    // STOPS on the carrier's domain boundary, which is the rail: seeded once
    // in the middle, its two ends are the two crossings.
    let fit_band = policy.intersection_fit;
    let drift_band = policy.pcurve_consistency;
    let seed_run = 0.5 * (run[0] + run[1]);
    let seed_point = at(seed_run, root_on_section(seed_run)?)?;
    let reach = points[0].sub(points[1]).length().max(1.0);
    let mut step = reach / 32.0;
    let mut outcome: Option<(NurbsCurve, NurbsCurve, NurbsCurve, f64)> = None;
    let mut worst = f64::INFINITY;
    let mut traced = 0usize;
    for _pass in 0..REFINEMENTS {
        let options = crate::SurfaceIntersectionOptions {
            tolerance: fit_band,
            seed_points: vec![seed_point],
            seed_only: true,
            maximum_step: Some(step),
            maximum_steps: STATION_BUDGET,
            ..Default::default()
        };
        let branches = crate::intersect_surfaces(&blend.surface, &obstacle.surface, &options).or_refuse(KernelStage::Refine, "intersect_surfaces")?;
        let [branch] = &branches[..] else {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "trim_branches", format!(
                "blend restriction: the blend and obstacle face {} meet in {} branches through \
                 the crossing window; a trim curve that is not one arc is not restricted \
                 directly",
                obstacle.id,
                branches.len()
            )));
        };
        if branch.closed || branch.points.len() < 4 {
            return Err(KernelRefusal::internal(KernelStage::Refine, "trim_curve_ends", format!(
                "blend restriction: the trim curve traced against obstacle face {} came back \
                 {} with {} points; it does not run from one rail crossing to the other",
                obstacle.id,
                if branch.closed { "closed" } else { "open" },
                branch.points.len()
            )));
        }
        // The march stops on the blend carrier's own domain boundary, which is
        // the rail.  Both ends must therefore BE the crossings; anything else
        // means the curve left the face somewhere this lane does not describe.
        let ends = [branch.points[0], branch.points[branch.points.len() - 1]];
        let straight = ends[0].sub(points[0]).length().max(ends[1].sub(points[1]).length());
        let swapped = ends[0].sub(points[1]).length().max(ends[1].sub(points[0]).length());
        let reversed = swapped < straight;
        let landing = straight.min(swapped);
        if landing > drift_band {
            return Err(KernelRefusal::internal(KernelStage::Refine, "trim_curve_drift", format!(
                "blend restriction: the traced trim curve stops {landing:.3e} from the rail \
                 crossings it must join (band {drift_band:.3e}); it leaves the blend face \
                 somewhere other than the rail"
            )));
        }
        let mut spatial: Vec<Vec3> = branch.points.clone();
        if reversed {
            spatial.reverse();
        }
        // Commit the ends EXACTLY: they are curve-curve intersections of two
        // existing curves, which is sharper than anything the march produces.
        let last = spatial.len() - 1;
        spatial[0] = points[0];
        spatial[last] = points[1];
        traced = spatial.len();

        let parameters = chord_parameters(&spatial)?;
        let degree = 3.min(spatial.len() - 1);
        let trim_curve = interpolate_curve(&spatial, degree, &parameters).or_refuse(KernelStage::Refine, "interpolate_curve")?;
        let [t0, t1] = trim_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let trim_on_blend = build_pcurve_on_surface_range(
            &blend.surface,
            &trim_curve,
            t0,
            t1,
            true,
            fit_band,
        ).or_refuse(KernelStage::Refine, "build_pcurve_on_surface_range")?;
        let trim_on_obstacle = if through_seam {
            unwrapped_pcurve(&obstacle.surface, &trim_curve, spatial.len(), degree)?
        } else {
            build_pcurve_on_surface_range(
                &obstacle.surface,
                &trim_curve,
                t0,
                t1,
                true,
                fit_band,
            ).or_refuse(KernelStage::Refine, "build_pcurve_on_surface_range")?
        };
        let [ou0, ou1] = obstacle.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [ov0, ov1] = obstacle.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let [op0, op1] = trim_on_obstacle.domain().or_refuse(KernelStage::Refine, "domain")?;
        for index in 0..VERIFY {
            if through_seam {
                break;
            }
            let a = trim_on_obstacle.evaluate(op0 + (op1 - op0) * index as f64 / VERIFY as f64).or_refuse(KernelStage::Refine, "evaluate")?;
            let b =
                trim_on_obstacle.evaluate(op0 + (op1 - op0) * (index + 1) as f64 / VERIFY as f64).or_refuse(KernelStage::Refine, "evaluate")?;
            if (b.x - a.x).abs() > 0.25 * (ou1 - ou0).abs()
                || (b.y - a.y).abs() > 0.25 * (ov1 - ov0).abs()
            {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "trim_seam_crossing", format!(
                    "blend restriction: the trim curve's pcurve on obstacle face {} jumps a \
                     large fraction of its domain, so it crosses that carrier's seam; a \
                     seam-crossing trim is not restricted directly",
                    obstacle.id
                )));
            }
        }

        // What the fit is worth, measured: how far the committed edge sits from
        // each carrier it is supposed to lie ON, and how far each pcurve's
        // image sits from that edge.
        let mut deviation: f64 = 0.0;
        let mut drift: f64 = 0.0;
        let mut off_carrier = [0.0f64; 2];
        let mut per_carrier = [0.0f64; 2];
        for index in 0..=VERIFY {
            let fraction = index as f64 / VERIFY as f64;
            let on_curve = trim_curve.evaluate(t0 + (t1 - t0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
            for (which, (surface, pcurve)) in [
                (&blend.surface, &trim_on_blend),
                (&obstacle.surface, &trim_on_obstacle),
            ]
            .into_iter()
            .enumerate()
            {
                let off = project_point_to_surface(surface, on_curve).or_refuse(KernelStage::Refine, "project_point_to_surface")?.distance;
                off_carrier[which] = off_carrier[which].max(off);
                deviation = deviation.max(off);
                let [p0, p1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                let node = pcurve.evaluate(p0 + (p1 - p0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
                let (u, v) = wrap_into_domain(surface, node.x, node.y)?;
                let miss = surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?.sub(on_curve).length();
                per_carrier[which] = per_carrier[which].max(miss);
                drift = drift.max(miss);
            }
        }
        worst = deviation.max(drift);
        if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
            eprintln!(
                "blend restriction: traced {traced} points at step {step:.4} — off-carrier \
                 blend={:.3e} obstacle={:.3e} drift blend={:.3e} obstacle={:.3e} \
                 (bands {fit_band:.3e} / {drift_band:.3e})",
                off_carrier[0], off_carrier[1], per_carrier[0], per_carrier[1],
            );
        }
        if deviation <= fit_band && drift <= drift_band {
            outcome = Some((trim_curve, trim_on_blend, trim_on_obstacle, worst));
            break;
        }
        step *= 0.5;
    }
    let Some((trim_curve, trim_on_blend, trim_on_obstacle, measured)) = outcome else {
        return Err(KernelRefusal::non_convergence(KernelStage::Refine, "trim_curve_fit", format!(
            "blend restriction: the trim curve could not be fitted to its carriers — {traced} \
             traced points still miss by {worst:.3e}, past the {fit_band:.3e} an SSI fit is \
             allowed; the restriction would be a new defect rather than a fix"
        )));
    };
    Ok((trim_curve, trim_on_blend, trim_on_obstacle, measured))
}

/// Chord-length parameters normalized to [0, 1].
fn chord_parameters(points: &[Vec3]) -> Result<Vec<f64>, KernelRefusal> {
    let mut total = 0.0;
    let mut running = vec![0.0];
    for pair in points.windows(2) {
        total += pair[1].sub(pair[0]).length();
        running.push(total);
    }
    if !(total > 0.0) {
        return Err(KernelRefusal::internal(KernelStage::Refine, "trim_stations", "blend restriction: the trim curve's stations collapse to a point"));
    }
    Ok(running.into_iter().map(|value| value / total).collect())
}

/// A pcurve through the traced points' projections, with each periodic
/// coordinate UNWRAPPED onto its predecessor's sheet — the pcurve may run
/// outside the carrier's domain by up to one period, which is what the
/// carrier seam split expects of a curve that crosses the meridian.  The
/// parameters are the 3D curve's own, so the two stay affinely aligned.
fn unwrapped_pcurve(
    surface: &crate::NurbsSurface,
    curve: &NurbsCurve,
    traced: usize,
    degree: usize,
) -> Result<NurbsCurve, KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let periods = [
        if closed_u { u1 - u0 } else { 0.0 },
        if closed_v { v1 - v0 } else { 0.0 },
    ];
    // Interpolating the projections of the TRACED points at their own
    // parameters left the pcurve's image 7e-5 .. 2.5e-4 off the edge between
    // them (the two interpolants bend differently in their two spaces), so
    // the nodes are the committed 3D curve's own samples, eight per traced
    // step.
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let count = (traced * 8).clamp(16, 4096);
    let parameters: Vec<f64> =
        (0..=count).map(|index| t0 + (t1 - t0) * index as f64 / count as f64).collect();
    let mut nodes: Vec<Vec3> = Vec::with_capacity(parameters.len());
    for &t in &parameters {
        let projection = project_point_to_surface(surface, curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        let mut uv = [projection.u, projection.v];
        if let Some(previous) = nodes.last() {
            let previous = [previous.x, previous.y];
            for axis in 0..2 {
                if periods[axis] > 0.0 {
                    let turns = ((uv[axis] - previous[axis]) / periods[axis]).round();
                    uv[axis] -= turns * periods[axis];
                }
            }
        }
        nodes.push(Vec3::new(uv[0], uv[1], 0.0));
    }
    interpolate_curve(&nodes, degree, &parameters).or_refuse(KernelStage::Refine, "interpolate_curve")
}

/// `(u, v)` folded into the carrier's domain along its periodic directions,
/// for evaluating a node of an unwrapped pcurve.
fn wrap_into_domain(
    surface: &crate::NurbsSurface,
    u: f64,
    v: f64,
) -> Result<(f64, f64), KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let fold = |value: f64, low: f64, high: f64| -> f64 {
        let period = high - low;
        if !(period > 0.0) {
            return value;
        }
        value - period * ((value - low) / period).floor()
    };
    Ok((
        if closed_u { fold(u, u0, u1) } else { u },
        if closed_v { fold(v, v0, v1) } else { v },
    ))
}

/// Shift an unwrapped pcurve by whole periods so that it lies inside the
/// carrier's domain (a closed piece spans exactly one period from its seam).
fn fold_into_domain(surface: &crate::NurbsSurface, mut pcurve: NurbsCurve) -> Result<NurbsCurve, KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let slack = 1e-6;
    for (axis, closed, low, high) in [(0usize, closed_u, u0, u1), (1usize, closed_v, v0, v1)] {
        if !closed {
            continue;
        }
        let period = high - low;
        let coordinate = |c: &crate::Vec4| if axis == 0 { c.x / c.w } else { c.y / c.w };
        let (mut min, mut max) = (f64::INFINITY, f64::NEG_INFINITY);
        for c in &pcurve.control_points {
            min = min.min(coordinate(c));
            max = max.max(coordinate(c));
        }
        let shift = period * ((min - low) / period + slack).floor();
        if shift != 0.0 {
            for c in pcurve.control_points.iter_mut() {
                if axis == 0 { c.x -= shift * c.w } else { c.y -= shift * c.w }
            }
            min -= shift;
            max -= shift;
        }
        if min < low - slack * period || max > high + slack * period {
            return Err(KernelRefusal::internal(KernelStage::Refine, "trim_piece_wrap", format!(
                "blend restriction: a closed trim piece spans {:.6} .. {:.6} on a carrier whose \
                 domain is {low:.6} .. {high:.6}; it wraps more than once",
                min, max
            )));
        }
    }
    Ok(pcurve)
}

/// Split one coedge into three at two normalized positions along its edge.
fn split_coedge_twice(
    coedge: &CoedgeRecord,
    first: f64,
    second: f64,
    edges: [u64; 3],
    take_id: &mut dyn FnMut() -> u64,
) -> Result<[CoedgeRecord; 3], KernelRefusal> {
    let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let walk = |value: f64| if coedge.forward { value } else { 1.0 - value };
    let mut cuts = [walk(first), walk(second)];
    if cuts[0] > cuts[1] {
        cuts.swap(0, 1);
    }
    let (head, rest) = coedge.pcurve.split(q0 + (q1 - q0) * cuts[0]).or_refuse(KernelStage::Refine, "split")?;
    let (middle, tail) = rest.split(q0 + (q1 - q0) * cuts[1]).or_refuse(KernelStage::Refine, "split")?;
    // The walk meets the edge pieces in the edge's own order when forward.
    let order = if coedge.forward {
        [edges[0], edges[1], edges[2]]
    } else {
        [edges[2], edges[1], edges[0]]
    };
    Ok([
        CoedgeRecord {
            id: coedge.id,
            edge_id: order[0],
            forward: coedge.forward,
            pcurve: head,
        },
        CoedgeRecord {
            id: take_id(),
            edge_id: order[1],
            forward: coedge.forward,
            pcurve: middle,
        },
        CoedgeRecord {
            id: take_id(),
            edge_id: order[2],
            forward: coedge.forward,
            pcurve: tail,
        },
    ])
}

/// Cut an edge in three at two parameters, minting the pieces and the two
/// crossing vertices, and splice every loop that uses it.
fn cut_edge_in_three(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    edge_id: u64,
    parameters: [f64; 2],
    vertices: [u64; 2],
) -> Result<[u64; 3], KernelRefusal> {
    let edge = edge_of(result, edge_id)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "edge_cut", "blend restriction: an edge vanished before its cut"))?
        .clone();
    let (low_curve, rest) = edge.curve.split(parameters[0]).or_refuse(KernelStage::Refine, "split")?;
    let (middle_curve, high_curve) = rest.split(parameters[1]).or_refuse(KernelStage::Refine, "split")?;
    let ids = [take_id(), take_id(), take_id()];
    for (index, (curve, span, ends)) in [
        (low_curve, [edge.t0, parameters[0]], [edge.start_vertex_id, vertices[0]]),
        (middle_curve, [parameters[0], parameters[1]], [vertices[0], vertices[1]]),
        (high_curve, [parameters[1], edge.t1], [vertices[1], edge.end_vertex_id]),
    ]
    .into_iter()
    .enumerate()
    {
        result.edges.push(EdgeRecord {
            id: ids[index],
            curve,
            t0: span[0],
            t1: span[1],
            start_vertex_id: ends[0],
            end_vertex_id: ends[1],
            degenerate: false,
            name: edge.name.clone(),
        });
    }
    let first = along(&edge, parameters[0]);
    let second = along(&edge, parameters[1]);
    let uses: usize = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .map(|l| l.coedges.iter().filter(|c| c.edge_id == edge_id).count())
        .sum();
    if uses != 2 {
        return Err(KernelRefusal::unsupported(KernelStage::Sew, "non_manifold_edge", format!(
            "blend restriction: edge {edge_id} has {uses} coedges, not two; a seam or a \
             non-manifold use is not cut here"
        )));
    }
    let mut spliced = 0usize;
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            for loop_record in &mut face.loops {
                let Some(position) = loop_record.coedges.iter().position(|c| c.edge_id == edge_id)
                else {
                    continue;
                };
                let pieces =
                    split_coedge_twice(&loop_record.coedges[position], first, second, ids, take_id)?;
                loop_record.coedges.splice(position..=position, pieces);
                spliced += 1;
            }
        }
    }
    if spliced != 2 {
        return Err(KernelRefusal::internal(KernelStage::Sew, "edge_splice", format!(
            "blend restriction: edge {edge_id} spliced into {spliced} loops, not two"
        )));
    }
    result.edges.retain(|candidate| candidate.id != edge_id);
    Ok(ids)
}

/// Returns the face the own-loop class cut off the mate, if it ran.
fn apply_restriction(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    plan: Restriction,
) -> Result<Option<u64>, KernelRefusal> {
    // The two crossing vertices, shared by every piece below.
    let crossing_vertices = [take_id(), take_id()];
    for (index, id) in crossing_vertices.iter().enumerate() {
        result.vertices.push(VertexRecord {
            id: *id,
            point: plan.points[index],
        });
    }
    // Which crossing does the obstacle's LOW parameter meet?
    let obstacle_edge = edge_of(result, plan.obstacle_edge)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "obstacle_boundary_edge", "blend restriction: the obstacle boundary edge vanished"))?;
    let low_point = obstacle_edge.curve.evaluate(plan.obstacle_parameters[0]).or_refuse(KernelStage::Refine, "evaluate")?;
    let low_meets_first =
        low_point.sub(plan.points[0]).length() <= low_point.sub(plan.points[1]).length();
    let obstacle_vertices = if low_meets_first {
        [crossing_vertices[0], crossing_vertices[1]]
    } else {
        [crossing_vertices[1], crossing_vertices[0]]
    };

    let rail_pieces = cut_edge_in_three(
        result,
        take_id,
        plan.rail_edge,
        plan.rail_parameters,
        crossing_vertices,
    )?;
    let obstacle_pieces = cut_edge_in_three(
        result,
        take_id,
        plan.obstacle_edge,
        plan.obstacle_parameters,
        obstacle_vertices,
    )?;

    // The trim edge, running from crossing 0 to crossing 1.
    let trim_edge = take_id();
    let [t0, t1] = plan.trim_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    result.edges.push(EdgeRecord {
        id: trim_edge,
        curve: plan.trim_curve.clone(),
        t0,
        t1,
        start_vertex_id: crossing_vertices[0],
        end_vertex_id: crossing_vertices[1],
        degenerate: false,
        name: None,
    });

    // --- The blend face: the rail's middle piece becomes the trim edge. ---
    replace_coedge(
        result,
        plan.blend_face,
        rail_pieces[1],
        trim_edge,
        &plan.trim_on_blend,
        crossing_vertices,
    )?;
    // The eaten pieces of the obstacle's boundary: the middle one, or — when
    // the eaten run passes through the edge's own vertex — the two outer ones.
    let eaten: Vec<u64> = if plan.through_seam {
        vec![obstacle_pieces[0], obstacle_pieces[2]]
    } else {
        vec![obstacle_pieces[1]]
    };

    // --- The obstacle face: the eaten run becomes the same trim edge. ---
    if plan.through_seam {
        replace_eaten_run(
            result,
            take_id,
            plan.obstacle_face,
            &eaten,
            obstacle_pieces[1],
            trim_edge,
            &plan.trim_on_obstacle,
            crossing_vertices,
        )?;
    } else {
        replace_coedge(
            result,
            plan.obstacle_face,
            obstacle_pieces[1],
            trim_edge,
            &plan.trim_on_obstacle,
            crossing_vertices,
        )?;
    }

    // --- The mate: the rail's middle piece and the eaten run both go, and the
    //     two loops they left open become one — or, when both were on one
    //     loop, that loop falls into two and the mate into two faces. ---
    let split_off = if plan.split_mate {
        Some(split_mate_loop(result, take_id, plan.mate_face, rail_pieces[1], &eaten)?)
    } else {
        merge_mate_loops(result, plan.mate_face, rail_pieces[1], &eaten)?;
        None
    };

    result
        .edges
        .retain(|edge| edge.id != rail_pieces[1] && !eaten.contains(&edge.id));

    // --- Through the seam: the obstacle's loop is now open at both seam
    //     coedges, which still end on the eaten vertex.  That is exactly the
    //     state a fully blended bore mouth leaves when its rail wraps the
    //     carrier, and the same repair closes it: split the trim where it
    //     crosses the meridian and trim the seam edge to the crossing. ---
    if plan.through_seam {
        if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
            dump_loop_walk(result, plan.obstacle_face, "obstacle before seam split");
        }
        let Some(split) =
            super::network::plan_carrier_seam_split(result, &[trim_edge], plan.obstacle_face)?
        else {
            return Err(KernelRefusal::unsupported(KernelStage::Sew, "seam_foot", format!(
                "blend restriction: the trim against obstacle face {} eats the foot of its seam \
                 and no seam split could re-foot it on the trim curve",
                plan.obstacle_face
            )));
        };
        super::network::apply_carrier_seam_split(result, take_id, split)?;
    }
    for face in [plan.blend_face, plan.obstacle_face, plan.mate_face].into_iter().chain(split_off) {
        check_loops_closed(result, face)?;
    }
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        eprintln!(
            "blend restriction: face {} trimmed against obstacle face {} along edge {} \
             (mate {} loops {} and {}, position {}) — fit {:.3e}",
            plan.blend_face,
            plan.obstacle_face,
            trim_edge,
            plan.mate_face,
            plan.rail_loop,
            plan.obstacle_loop,
            plan.obstacle_position,
            plan.deviation,
        );
    }
    Ok(split_off)
}

/// Drop the eaten pieces of a closed boundary edge from a face's loop and put
/// the trim edge where the walk reaches the KEPT piece, facing so that it ends
/// where the kept piece starts.  The seam coedges on either side are left
/// ending on the eaten vertex; the carrier seam split re-foots them.
fn replace_eaten_run(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    face_id: u64,
    eaten: &[u64],
    kept: u64,
    trim_edge: u64,
    pcurve: &NurbsCurve,
    trim_vertices: [u64; 2],
) -> Result<(), KernelRefusal> {
    let kept_edge = edge_of(result, kept)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "kept_piece", "blend restriction: the kept piece vanished"))?
        .clone();
    let face = result
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .find(|face| face.id == face_id)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "obstacle_face", "blend restriction: the obstacle face vanished before its splice"))?;
    for loop_record in &mut face.loops {
        let Some(kept_position) = loop_record.coedges.iter().position(|c| c.edge_id == kept)
        else {
            continue;
        };
        let kept_coedge = &loop_record.coedges[kept_position];
        let kept_start = if kept_coedge.forward {
            kept_edge.start_vertex_id
        } else {
            kept_edge.end_vertex_id
        };
        let forward = trim_vertices[1] == kept_start;
        if !forward && trim_vertices[0] != kept_start {
            return Err(KernelRefusal::internal(KernelStage::Sew, "kept_piece", format!(
                "blend restriction: the trim edge meets neither end of the kept piece {kept} \
                 on face {face_id}"
            )));
        }
        let trim = CoedgeRecord {
            id: kept_coedge.id,
            edge_id: trim_edge,
            forward,
            pcurve: if forward { pcurve.clone() } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
        };
        let mut rebuilt: Vec<CoedgeRecord> = Vec::with_capacity(loop_record.coedges.len());
        for (position, coedge) in loop_record.coedges.iter().enumerate() {
            if eaten.contains(&coedge.edge_id) {
                continue;
            }
            if position == kept_position {
                let mut inserted = trim.clone();
                inserted.id = take_id();
                rebuilt.push(inserted);
            }
            rebuilt.push(coedge.clone());
        }
        if rebuilt.len() + eaten.len() != loop_record.coedges.len() + 1 {
            return Err(KernelRefusal::internal(KernelStage::Sew, "eaten_pieces", format!(
                "blend restriction: face {face_id}'s loop did not carry every eaten piece of \
                 its boundary"
            )));
        }
        loop_record.coedges = rebuilt;
        return Ok(());
    }
    Err(KernelRefusal::internal(KernelStage::Sew, "kept_piece", format!("blend restriction: face {face_id} does not use the kept piece {kept}")))
}

/// Split one coedge in two at a normalized position along its edge.
fn split_coedge_once(
    coedge: &CoedgeRecord,
    along: f64,
    edges: [u64; 2],
    take_id: &mut dyn FnMut() -> u64,
) -> Result<[CoedgeRecord; 2], KernelRefusal> {
    let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let walked = if coedge.forward { along } else { 1.0 - along };
    let (head, tail) = coedge.pcurve.split(q0 + (q1 - q0) * walked).or_refuse(KernelStage::Refine, "split")?;
    let order = if coedge.forward { [edges[0], edges[1]] } else { [edges[1], edges[0]] };
    Ok([
        CoedgeRecord { id: coedge.id, edge_id: order[0], forward: coedge.forward, pcurve: head },
        CoedgeRecord { id: take_id(), edge_id: order[1], forward: coedge.forward, pcurve: tail },
    ])
}

/// Cut an edge in two at `parameter`, minting the pieces and the cut vertex
/// (already recorded), and splice every loop that uses it.  Returns the
/// `[low, high]` piece ids.
fn cut_edge_in_two(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    edge_id: u64,
    parameter: f64,
    vertex: u64,
) -> Result<[u64; 2], KernelRefusal> {
    let edge = edge_of(result, edge_id)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "edge_cut", "blend restriction: an edge vanished before its cut"))?
        .clone();
    let (low_curve, high_curve) = edge.curve.split(parameter).or_refuse(KernelStage::Refine, "split")?;
    let ids = [take_id(), take_id()];
    result.edges.push(EdgeRecord {
        id: ids[0],
        curve: low_curve,
        t0: edge.t0,
        t1: parameter,
        start_vertex_id: edge.start_vertex_id,
        end_vertex_id: vertex,
        degenerate: false,
        name: edge.name.clone(),
    });
    result.edges.push(EdgeRecord {
        id: ids[1],
        curve: high_curve,
        t0: parameter,
        t1: edge.t1,
        start_vertex_id: vertex,
        end_vertex_id: edge.end_vertex_id,
        degenerate: false,
        name: edge.name.clone(),
    });
    let along = along(&edge, parameter);
    let mut spliced = 0usize;
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            for loop_record in &mut face.loops {
                let Some(position) = loop_record.coedges.iter().position(|c| c.edge_id == edge_id)
                else {
                    continue;
                };
                let pieces = split_coedge_once(&loop_record.coedges[position], along, ids, take_id)?;
                loop_record.coedges.splice(position..=position, pieces);
                spliced += 1;
            }
        }
    }
    if spliced != 2 {
        return Err(KernelRefusal::internal(KernelStage::Sew, "edge_splice", format!(
            "blend restriction: edge {edge_id} spliced into {spliced} loops, not two"
        )));
    }
    result.edges.retain(|candidate| candidate.id != edge_id);
    Ok(ids)
}

/// Replace one contiguous (cyclic) block of coedges in a face's loop by a
/// chain of new edges, oriented so the walk closes: the first inserted coedge
/// starts where the coedge before the block ends.  `inserts` are
/// `(edge id, pcurve in the edge's direction, start vertex, end vertex)` in
/// chain order; the chain is used as given or reversed whole.
fn splice_block(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    face_id: u64,
    removed: &[u64],
    inserts: &[(u64, NurbsCurve, u64, u64)],
) -> Result<(), KernelRefusal> {
    let walk_end = |solid: &BrepSolid, coedge: &CoedgeRecord| -> Result<u64, KernelRefusal> {
        walk_vertex(solid, coedge, true)
    };
    let face = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "splice_face", "blend restriction: a face vanished before its splice"))?;
    let Some(loop_index) = face
        .loops
        .iter()
        .position(|l| l.coedges.iter().any(|c| removed.contains(&c.edge_id)))
    else {
        return Err(KernelRefusal::internal(KernelStage::Sew, "eaten_pieces", format!("blend restriction: face {face_id} carries none of the eaten pieces")));
    };
    let coedges = face.loops[loop_index].coedges.clone();
    let n = coedges.len();
    let block_start = (0..n)
        .find(|&i| removed.contains(&coedges[i].edge_id) && !removed.contains(&coedges[(i + n - 1) % n].edge_id))
        .ok_or_else(|| format!("blend restriction: face {face_id}'s eaten run is its whole loop")).or_refuse(KernelStage::Refine, "ok_or_else")?;
    let mut rotated = coedges;
    rotated.rotate_left(block_start);
    let block_length = rotated.iter().take_while(|c| removed.contains(&c.edge_id)).count();
    if block_length != removed.len() || rotated[block_length..].iter().any(|c| removed.contains(&c.edge_id)) {
        return Err(KernelRefusal::internal(KernelStage::Sew, "eaten_pieces", format!(
            "blend restriction: face {face_id}'s eaten pieces are not one contiguous run of its loop"
        )));
    }
    let rest: Vec<CoedgeRecord> = rotated[block_length..].to_vec();
    let previous_end = walk_end(result, rest.last().ok_or(KernelRefusal::internal(KernelStage::Sew, "eaten_run", "blend restriction: a loop is only its eaten run"))?)?;
    let forward_chain = inserts[0].2 == previous_end;
    let reversed_chain = inserts[inserts.len() - 1].3 == previous_end;
    if !forward_chain && !reversed_chain {
        return Err(KernelRefusal::internal(KernelStage::Sew, "trim_chain", format!(
            "blend restriction: the trim chain meets neither end of face {face_id}'s remaining loop"
        )));
    }
    let mut new_coedges: Vec<CoedgeRecord> = Vec::with_capacity(inserts.len());
    if forward_chain {
        for (edge_id, pcurve, _, _) in inserts {
            new_coedges.push(CoedgeRecord { id: take_id(), edge_id: *edge_id, forward: true, pcurve: pcurve.clone() });
        }
    } else {
        for (edge_id, pcurve, _, _) in inserts.iter().rev() {
            new_coedges.push(CoedgeRecord { id: take_id(), edge_id: *edge_id, forward: false, pcurve: pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? });
        }
    }
    let mut rebuilt = new_coedges;
    rebuilt.extend(rest);
    let face = result
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .find(|face| face.id == face_id)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "splice_face", "blend restriction: a face vanished before its splice"))?;
    face.loops[loop_index].coedges = rebuilt;
    Ok(())
}

/// Carry out a chain restriction.  Geometry first (every junction and piece
/// is already solved in the plan), then the topology in an order that never
/// leaves a loop holding a dead id: junction edges trimmed, the rail cut in
/// three and the end links in two, the blend loop spliced, each obstacle loop
/// spliced, the mate's loops merged.
fn apply_chain(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    plan: ChainRestriction,
) -> Result<(), KernelRefusal> {
    let crossing_vertices = [take_id(), take_id()];
    for (index, id) in crossing_vertices.iter().enumerate() {
        result.vertices.push(VertexRecord { id: *id, point: plan.points[index] });
    }
    let mut junction_vertices = Vec::with_capacity(plan.junctions.len());
    for junction in &plan.junctions {
        let id = take_id();
        result.vertices.push(VertexRecord { id, point: junction.point });
        trim_edge_at(result, junction.edge, junction.parameter, junction.eaten_vertex, id)?;
        junction_vertices.push(id);
    }
    // The chain's vertices in order: crossing 0, junctions, crossing 1.
    let mut chain_vertices = vec![crossing_vertices[0]];
    chain_vertices.extend(junction_vertices.iter().copied());
    chain_vertices.push(crossing_vertices[1]);

    let rail_pieces = cut_edge_in_three(
        result,
        take_id,
        plan.rail_edge,
        plan.rail_parameters,
        crossing_vertices,
    )?;

    // The eaten pieces of the mate's boundary, one per link, in chain order.
    let mut eaten: Vec<u64> = Vec::with_capacity(plan.links.len());
    let last = plan.links.len() - 1;
    for (index, link) in plan.links.iter().enumerate() {
        if index == 0 || index == last {
            let edge = edge_of(result, link.edge)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "chain_link", "blend restriction: a chain link vanished"))?
                .clone();
            let crossing = if index == 0 { crossing_vertices[0] } else { crossing_vertices[1] };
            let eats_high = (link.eaten[1] - edge.t1).abs() <= (link.eaten[0] - edge.t0).abs();
            let parameter = if eats_high { link.eaten[0] } else { link.eaten[1] };
            let [low, high] = cut_edge_in_two(result, take_id, link.edge, parameter, crossing)?;
            eaten.push(if eats_high { high } else { low });
        } else {
            eaten.push(link.edge);
        }
    }

    // The trim edges, piece i from chain vertex i to i + 1.
    let mut inserts: Vec<(u64, NurbsCurve, u64, u64)> = Vec::with_capacity(plan.pieces.len());
    let mut on_obstacle: Vec<NurbsCurve> = Vec::with_capacity(plan.pieces.len());
    for (index, piece) in plan.pieces.iter().enumerate() {
        let id = take_id();
        let [t0, t1] = piece.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        result.edges.push(EdgeRecord {
            id,
            curve: piece.curve.clone(),
            t0,
            t1,
            start_vertex_id: chain_vertices[index],
            end_vertex_id: chain_vertices[index + 1],
            degenerate: false,
            name: None,
        });
        inserts.push((id, piece.on_blend.clone(), chain_vertices[index], chain_vertices[index + 1]));
        on_obstacle.push(piece.on_obstacle.clone());
    }

    // The blend: the rail's middle piece becomes the whole chain.
    splice_block(result, take_id, plan.blend_face, &[rail_pieces[1]], &inserts)?;
    // Each obstacle face: its eaten piece becomes its piece of the chain.
    for (index, link) in plan.links.iter().enumerate() {
        let insert = (inserts[index].0, on_obstacle[index].clone(), inserts[index].2, inserts[index].3);
        splice_block(result, take_id, link.obstacle_face, &[eaten[index]], &[insert])?;
    }
    // The mate: the rail's middle piece and the whole eaten run go, and the
    // two loops they left open become one.
    merge_mate_loops(result, plan.mate_face, rail_pieces[1], &eaten)?;
    result
        .edges
        .retain(|edge| edge.id != rail_pieces[1] && !eaten.contains(&edge.id));
    for face in std::iter::once(plan.blend_face)
        .chain(std::iter::once(plan.mate_face))
        .chain(plan.links.iter().map(|link| link.obstacle_face))
    {
        check_loops_closed(result, face)?;
    }
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        eprintln!(
            "blend restriction: face {} trimmed against obstacle faces {:?} along a chain of {} \
             pieces (mate {}, {} junctions)",
            plan.blend_face,
            plan.links.iter().map(|link| link.obstacle_face).collect::<Vec<_>>(),
            plan.pieces.len(),
            plan.mate_face,
            plan.junctions.len(),
        );
    }
    Ok(())
}

/// A void wholly under the blend's arc: an obstacle that meets NEITHER mate,
/// so no rail crosses it and the module's first two classes never see it.
/// Its edges cross the blend surface instead, and the trim is a CLOSED chain
/// of pieces around the void — an inner loop on the blend, and on each
/// obstacle face the piece that closes the gap the sliver tore in its loop.
struct VoidRestriction {
    blend_face: u64,
    junctions: Vec<Junction>,
    /// Edges wholly inside the sliver: both ends removed, no crossing.
    removed_edges: Vec<u64>,
    /// Faces all of whose loops lie wholly inside the sliver.
    removed_faces: Vec<u64>,
    /// Faces that keep part of a loop, with the pieces that close their gaps:
    /// `(face, from junction index, to junction index, piece)`, the piece
    /// running from the first junction to the second.
    pieces: Vec<(u64, usize, usize, ChainPiece)>,
}

/// Is `point` on the side of the blend the sliver was removed from — beyond
/// the blend face, under its arc?  Requires the point's foot on the carrier to
/// lie inside the blend FACE, so a point far from the stripe reads as neither.
fn under_the_arc(
    blend: &FaceRecord,
    point: Vec3,
    convex: bool,
    size: f64,
    policy: &KernelTolerances,
) -> Result<Option<bool>, KernelRefusal> {
    let projection = project_point_to_surface(&blend.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    if !uv_inside_face(blend, projection.u, projection.v, policy.model)? {
        return Ok(None);
    }
    // Nothing the stripe adds or removes lies farther from its surface than
    // its size; a point behind the pad deep in the part projects onto the pad
    // just as a rim under it does, and only the distance tells them apart.
    if projection.distance > size * (1.0 + 1e-6) + policy.model {
        return Ok(None);
    }
    let normal = outward_normal(blend, projection.u, projection.v)?;
    // A convex blend removes what lies beyond its wall; a concave one ADDS
    // the pad, so what lies under the pad — on its inner side — is what goes.
    Ok(Some((point.sub(projection.point).dot(normal) > 0.0) == convex))
}

/// Whether `(u, v)` lies inside a face's trimmed region, at a uv band
/// equivalent to the model band there (capped, as the classifier caps it).
fn uv_inside_face(face: &FaceRecord, u: f64, v: f64, spatial: f64) -> Result<bool, KernelRefusal> {
    let derivatives = face.surface.derivatives(u, v, 1).or_refuse(KernelStage::Refine, "derivatives")?;
    let band = crate::tolerance::surface_uv_tolerance(
        spatial,
        derivatives[1][0].length(),
        derivatives[0][1].length(),
    );
    let cap = match (face.surface.domain_u(), face.surface.domain_v()) {
        (Ok([a0, a1]), Ok([b0, b1])) => ((a1 - a0).min(b1 - b0) * 0.05).max(1e-12),
        _ => f64::INFINITY,
    };
    Ok(matches!(
        crate::parameter_point_in_face(face, crate::Vec2 { x: u, y: v }, band.min(cap)).or_refuse(KernelStage::Refine, "parameter_point_in_face")?,
        crate::PolygonClass::Inside
    ))
}

/// The axis-aligned box of a surface over its domain, sampled.
fn surface_box(surface: &crate::NurbsSurface) -> Result<(Vec3, Vec3), KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let (mut low, mut high) = (Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY));
    const N: usize = 12;
    for i in 0..=N {
        for j in 0..=N {
            let p = surface.evaluate(u0 + (u1 - u0) * i as f64 / N as f64, v0 + (v1 - v0) * j as f64 / N as f64).or_refuse(KernelStage::Refine, "evaluate")?;
            low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
            high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
        }
    }
    Ok((low, high))
}

/// Plan a void restriction for one stripe, or `Ok(None)` when no edge of the
/// solid crosses the blend face away from its rails.
fn plan_void(
    solid: &BrepSolid,
    blend_face_id: u64,
    stripe_mates: &[u64],
    convex: bool,
    size: f64,
    policy: &KernelTolerances,
) -> Result<Option<VoidRestriction>, KernelRefusal> {
    let blend = face_of(solid, blend_face_id).ok_or(KernelRefusal::internal(KernelStage::Refine, "blend_face", "blend restriction: the blend face is missing"))?;
    // The blend's own edges (rails and sections) are the stripe's boundary,
    // and so are their vertices; a mate's OTHER edges may well be under the
    // arc — a hole rim wholly under a pad is one — so they stay candidates.
    let own: Vec<u64> = face_edges(blend).into_iter().map(|(_, _, id)| id).collect();
    let own_vertices: Vec<u64> = own
        .iter()
        .filter_map(|id| edge_of(solid, *id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let (low, high) = surface_box(&blend.surface)?;
    let margin = policy.model * 10.0;
    let near = |p: Vec3| -> bool {
        p.x >= low.x - margin && p.x <= high.x + margin
            && p.y >= low.y - margin && p.y <= high.y + margin
            && p.z >= low.z - margin && p.z <= high.z + margin
    };
    // Candidates: edges with a sample inside the blend carrier's box, not the
    // blend's own and not meeting the blend at one of its own vertices — an
    // edge that does meets the blend on its boundary, which is no void.
    let mut candidates: Vec<&EdgeRecord> = Vec::new();
    for edge in &solid.edges {
        if own.contains(&edge.id)
            || edge.degenerate
            || own_vertices.contains(&edge.start_vertex_id)
            || own_vertices.contains(&edge.end_vertex_id)
        {
            continue;
        }
        let mut any = false;
        for k in 0..=6 {
            let t = edge.t0 + (edge.t1 - edge.t0) * k as f64 / 6.0;
            if near(edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?) {
                any = true;
                break;
            }
        }
        if any {
            candidates.push(edge);
        }
    }
    if candidates.is_empty() {
        return Ok(None);
    }
    // Crossings of each candidate with the blend FACE (inside its trims).
    // In-span crossings of each candidate with the blend FACE, and the
    // crossings of its curve BEYOND its span (an edge may have to EXTEND up
    // to the blend: a hole's seam under a pad).
    let mut crossings: Vec<Vec<(f64, Vec3)>> = Vec::with_capacity(candidates.len());
    let mut beyond: Vec<Vec<(f64, Vec3)>> = Vec::with_capacity(candidates.len());
    let mut any_crossing = false;
    // A hit within the pcurve band of the blend's own boundary is a graze of
    // that boundary (a mate's edge meeting the rail), not a crossing.
    let boundary: Vec<&EdgeRecord> = own.iter().filter_map(|id| edge_of(solid, *id)).collect();
    let grazes = |point: Vec3| -> Result<bool, KernelRefusal> {
        for edge in &boundary {
            if crate::project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?.distance <= policy.pcurve_consistency {
                return Ok(true);
            }
        }
        Ok(false)
    };
    for edge in &candidates {
        let mut hits: Vec<(f64, Vec3)> = Vec::new();
        let mut outside: Vec<(f64, Vec3)> = Vec::new();
        for hit in intersect_curve_surface(&edge.curve, &blend.surface, policy.intersection_fit).or_refuse(KernelStage::Refine, "intersect_curve_surface")? {
            if !uv_inside_face(blend, hit.u, hit.v, policy.model)? || grazes(hit.point)? {
                continue;
            }
            let bucket = if within(hit.t, edge.t0, edge.t1) { &mut hits } else { &mut outside };
            if bucket.iter().any(|(t, _)| (t - hit.t).abs() <= 1e-9 * (edge.t1 - edge.t0).abs()) {
                continue;
            }
            bucket.push((hit.t, edge.curve.evaluate(hit.t).or_refuse(KernelStage::Refine, "evaluate")?));
        }
        any_crossing |= !hits.is_empty() || !outside.is_empty();
        crossings.push(hits);
        beyond.push(outside);
    }
    if !any_crossing {
        return Ok(None);
    }
    // Which candidate endpoints are under the arc.
    let mut removed_vertices: Vec<u64> = Vec::new();
    let mut side: rustc_hash::FxHashMap<u64, Option<bool>> = rustc_hash::FxHashMap::default();
    for edge in &candidates {
        for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
            if side.contains_key(&vertex) {
                continue;
            }
            let point = solid
                .vertices
                .iter()
                .find(|v| v.id == vertex)
                .ok_or(KernelRefusal::internal(KernelStage::Refine, "candidate_vertex", "blend restriction: a candidate edge's vertex is missing"))?
                .point;
            let verdict = if own_vertices.contains(&vertex) {
                None
            } else {
                under_the_arc(blend, point, convex, size, policy)?
            };
            if verdict == Some(true) {
                removed_vertices.push(vertex);
            }
            side.insert(vertex, verdict);
        }
    }
    let removed = |vertex: u64| side.get(&vertex).copied().flatten() == Some(true);
    // Classify the candidates.
    let mut junctions: Vec<Junction> = Vec::new();
    let mut removed_edges: Vec<u64> = Vec::new();
    for ((edge, hits), outside) in candidates.iter().zip(&crossings).zip(&beyond) {
        let (a, b) = (removed(edge.start_vertex_id), removed(edge.end_vertex_id));
        if std::env::var("BREP_DEBUG_NETWORK").is_ok() && (!hits.is_empty() || !outside.is_empty() || a || b) {
            let p0 = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
            let p1 = edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Refine, "evaluate")?;
            let [c0, c1] = edge.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            eprintln!(
                "blend restriction: void candidate edge {} span [{:.4}, {:.4}] of curve [{c0:.4}, {c1:.4}] \
                 ({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3}) under={a}/{b} sides={:?}/{:?} hits={:?} beyond={:?}",
                edge.id, edge.t0, edge.t1, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z,
                side.get(&edge.start_vertex_id), side.get(&edge.end_vertex_id),
                hits.iter().map(|h| h.0).collect::<Vec<_>>(), outside.iter().map(|h| h.0).collect::<Vec<_>>()
            );
        }
        match (hits.len(), a, b) {
            (0, false, false) => {}
            (0, true, true) => removed_edges.push(edge.id),
            (1, true, false) | (1, false, true) => junctions.push(Junction {
                edge: edge.id,
                eaten_vertex: if a { edge.start_vertex_id } else { edge.end_vertex_id },
                parameter: hits[0].0,
                point: hits[0].1,
            }),
            // One end under the arc and no crossing inside the edge: the edge
            // must EXTEND past that end to reach the blend — a hole's seam
            // running down from a rim the pad closes over.  Its curve's
            // crossing beyond the eaten end is the junction.
            (0, true, false) | (0, false, true) => {
                let past: Vec<&(f64, Vec3)> = outside
                    .iter()
                    .filter(|(t, _)| if a { *t < edge.t0 } else { *t > edge.t1 })
                    .collect();
                let [hit] = past[..] else {
                    return unresolved_void(format!(
                        "edge {} of the solid keeps one end under the arc of blend face {} and its \
                         curve meets the blend {} times beyond that end (expected once, to extend to)",
                        edge.id,
                        blend.id,
                        past.len()
                    ));
                };
                junctions.push(Junction {
                    edge: edge.id,
                    eaten_vertex: if a { edge.start_vertex_id } else { edge.end_vertex_id },
                    parameter: hit.0,
                    point: hit.1,
                });
            }
            (count, _, _) => {
                return unresolved_void(format!(
                    "edge {} crosses the blend face {} {count} time(s) with its ends {}/{} under \
                     the arc; a void edge that dips through the blend and back",
                    edge.id,
                    blend.id,
                    if a { "under" } else { "clear" },
                    if b { "under" } else { "clear" }
                ));
            }
        }
    }
    if junctions.is_empty() {
        return unresolved_void(format!(
            "edges of the solid cross the blend face {} but none leaves the arc on one side and \
             enters it on the other",
            blend.id
        ));
    }
    // Affected faces: those using a junction or removed edge, none of them
    // the stripe's own.
    let touched: Vec<u64> = junctions.iter().map(|j| j.edge).chain(removed_edges.iter().copied()).collect();
    let mut affected: Vec<u64> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face.loops.iter().any(|l| l.coedges.iter().any(|c| touched.contains(&c.edge_id))))
        .map(|face| face.id)
        .collect();
    affected.sort_unstable();
    affected.dedup();
    if affected.contains(&blend.id) {
        return unresolved_void(format!(
            "an edge crossing the blend face {} belongs to the blend itself",
            blend.id
        ));
    }
    // A mate may lose a whole loop to the void (a hole rim wholly under the
    // pad); a junction ON a mate is a rail-crossing class, not this one.
    for mate in stripe_mates {
        if junctions.iter().any(|j| face_of(solid, *mate).is_some_and(|face| face_edges(face).iter().any(|(_, _, id)| *id == j.edge))) {
            return unresolved_void(format!(
                "a junction edge of the void under blend face {} lies on mate face {mate}",
                blend.id
            ));
        }
    }
    // Per affected face: the loops after the cut, and the gaps to close.
    let mut removed_faces: Vec<u64> = Vec::new();
    let mut pieces: Vec<(u64, usize, usize, ChainPiece)> = Vec::new();
    for &face_id in &affected {
        let face = face_of(solid, face_id).ok_or(KernelRefusal::internal(KernelStage::Refine, "affected_face", "blend restriction: an affected face is missing"))?;
        let mut loops_kept = 0usize;
        let mut gaps: Vec<(usize, usize)> = Vec::new();
        let is_mate = stripe_mates.contains(&face_id);
        for loop_record in &face.loops {
            // Effective walk vertices after the trims: a junction edge's eaten
            // end becomes its junction.  `followed_removal` marks a kept
            // coedge that comes right after a removed run.
            let mut kept: Vec<(u64, u64, bool)> = Vec::new();
            let mut pending_removal = loop_record
                .coedges
                .last()
                .is_some_and(|c| removed_edges.contains(&c.edge_id));
            for coedge in &loop_record.coedges {
                if removed_edges.contains(&coedge.edge_id) {
                    pending_removal = true;
                    continue;
                }
                let edge = edge_of(solid, coedge.edge_id).ok_or(KernelRefusal::internal(KernelStage::Refine, "missing_edge", "blend restriction: a loop references a missing edge"))?;
                let (mut start, mut end) = if coedge.forward {
                    (edge.start_vertex_id, edge.end_vertex_id)
                } else {
                    (edge.end_vertex_id, edge.start_vertex_id)
                };
                if let Some(junction) = junctions.iter().position(|j| j.edge == coedge.edge_id) {
                    // Tag the eaten end with a junction marker id.
                    let marker = u64::MAX - junction as u64;
                    if start == junctions[junction].eaten_vertex { start = marker; }
                    if end == junctions[junction].eaten_vertex { end = marker; }
                } else if removed(start) || removed(end) {
                    return unresolved_void(format!(
                        "edge {} of face {face_id} keeps an end under the arc without crossing \
                         the blend",
                        coedge.edge_id
                    ));
                }
                kept.push((start, end, pending_removal));
                pending_removal = false;
            }
            if kept.is_empty() {
                continue;
            }
            loops_kept += 1;
            for index in 0..kept.len() {
                let end = kept[index].1;
                let (next_start, _, followed_removal) = kept[(index + 1) % kept.len()];
                if end == next_start {
                    // The walk still closes here; but if a removed run sat
                    // between the two, its place is taken by a CLOSED piece
                    // through the one junction they share (a rim the pad
                    // closes over, hung on its seam).
                    if followed_removal {
                        let Some(junction) = marker_index(end) else {
                            return unresolved_void(format!(
                                "face {face_id} lost a run of its loop between coedges meeting at \
                                 vertex {end}, which is not a junction"
                            ));
                        };
                        gaps.push((junction, junction));
                    }
                    continue;
                }
                let (Some(from), Some(to)) = (marker_index(end), marker_index(next_start)) else {
                    return unresolved_void(format!(
                        "face {face_id}'s loop breaks at vertices {end} and {next_start}, not at \
                         two junctions"
                    ));
                };
                gaps.push((from, to));
            }
        }
        if loops_kept == 0 {
            removed_faces.push(face_id);
            continue;
        }
        if is_mate {
            // A mate keeps its rail loop and loses the loop under the pad;
            // nothing on it is traced.
            if !gaps.is_empty() {
                return unresolved_void(format!("mate face {face_id} would need a trim piece"));
            }
            continue;
        }
        for (from, to) in gaps {
            let piece = trace_piece(
                blend,
                face,
                [junctions[from].point, junctions[to].point],
                junctions[from].point,
                policy,
            )?;
            pieces.push((face_id, from, to, piece));
        }
    }
    if pieces.is_empty() {
        return unresolved_void(format!(
            "the void under the blend face {} leaves no gap to close",
            blend.id
        ));
    }
    // Every junction must be the end of exactly two pieces for the blend's
    // inner loop(s) to close.
    for (index, junction) in junctions.iter().enumerate() {
        let uses: usize = pieces
            .iter()
            .map(|(_, a, b, _)| usize::from(*a == index) + usize::from(*b == index))
            .sum();
        if uses != 2 {
            return unresolved_void(format!(
                "junction on edge {} at ({:.6}, {:.6}, {:.6}) ends {uses} trim pieces (expected two)",
                junction.edge, junction.point.x, junction.point.y, junction.point.z
            ));
        }
    }
    Ok(Some(VoidRestriction { blend_face: blend.id, junctions, removed_edges, removed_faces, pieces }))
}

/// Restrict the selection's CLOSURES — corner patches, chamfer facets and
/// cones, horn sectors — against the voids under them (2026-09-26).  A
/// closure is not a stripe: it has no rails to scan a mate's loop with, so
/// only the void lane applies, with the faces it is sewn to standing in for
/// mates.  Whether it removes or adds material is read from the original
/// solid: a point just outside the closure, along its outward normal, lies
/// INSIDE the original where the closure removed material (convex) and
/// outside it where the closure added a pad (concave).
pub(in crate::blend) fn restrict_closures(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    closures: &[u64],
    original: &BrepSolid,
    size: f64,
    policy: &KernelTolerances,
) -> Result<usize, KernelRefusal> {
    if closures.is_empty() {
        return Ok(0);
    }
    let classifier = crate::SolidClassifier::new(original, policy.model).or_refuse(KernelStage::Refine, "new")?;
    let mut applied = 0usize;
    for &closure in closures {
        let Some(face) = face_of(result, closure) else { continue };
        let own: Vec<u64> = face_edges(face).into_iter().map(|(_, _, id)| id).collect();
        let neighbours: Vec<u64> = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|other| other.id != closure)
            .filter(|other| face_edges(other).iter().any(|(_, _, id)| own.contains(id)))
            .map(|other| other.id)
            .collect();
        // Convexity, by majority over a grid of the closure's interior: one
        // probe can land inside the very void under the closure (the cone's
        // middle sits over the bore), and read as air.
        let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let (mut inside, mut outside) = (0usize, 0usize);
        const GRID: usize = 7;
        for i in 1..GRID {
            for j in 1..GRID {
                let (u, v) = (
                    u0 + (u1 - u0) * i as f64 / GRID as f64,
                    v0 + (v1 - v0) * j as f64 / GRID as f64,
                );
                if !uv_inside_face(face, u, v, policy.model)? {
                    continue;
                }
                let point = face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
                let normal = outward_normal(face, u, v)?;
                let probe = point.add(normal.scale(size * 0.05));
                match classifier.classify(probe).or_refuse(KernelStage::Refine, "classify")?.class {
                    crate::PointClass::In => inside += 1,
                    crate::PointClass::Out => outside += 1,
                    crate::PointClass::On => {}
                }
            }
        }
        if inside == outside {
            if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                eprintln!("blend restriction: closure face {closure}: convexity undecided ({inside} in, {outside} out), left alone");
            }
            continue;
        }
        let convex = inside > outside;
        if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
            eprintln!(
                "blend restriction: closure face {closure} same_sense={} probes {inside} in / {outside} out -> convex={convex}; neighbours {:?}",
                face.same_sense, neighbours
            );
        }
        for _ in 0..OBSTACLE_BUDGET {
            let Some(plan) = plan_void(result, closure, &neighbours, convex, size, policy)? else {
                break;
            };
            apply_void(result, take_id, plan)?;
            applied += 1;
        }
    }
    Ok(applied)
}

/// A void the planner cannot describe is NOT a refusal of the network's
/// result: the result is left untrimmed, exactly as before this lane existed,
/// and `check_blend_interference` decides whether it stands — a crossing it
/// sees still routes the group to the cutter, a graze it does not see is
/// nothing.  Refusing here instead sank chains that built before the lane
/// (a mate's edge grazing the blend's own boundary on the rounded-cone wall
/// document, 2026-09-26).  The reason is printed under `BREP_DEBUG_NETWORK`.
fn unresolved_void(reason: String) -> Result<Option<VoidRestriction>, KernelRefusal> {
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        eprintln!("blend restriction: void not resolved — {reason}");
    }
    Ok(None)
}

/// The junction index a tagged walk vertex stands for.
fn marker_index(tagged: u64) -> Option<usize> {
    (tagged > u64::MAX / 2).then(|| (u64::MAX - tagged) as usize)
}

/// Signed area of a loop's uv polygon (shoelace over the sampled pcurves).
fn loop_signed_area(face: &FaceRecord, loop_index: usize) -> Result<f64, KernelRefusal> {
    let polygon = loop_polygon(face, loop_index)?;
    let mut area = 0.0;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        area += a[0] * b[1] - b[0] * a[1];
    }
    Ok(0.5 * area)
}

/// Carry out a void restriction.
fn apply_void(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    plan: VoidRestriction,
) -> Result<(), KernelRefusal> {
    // Junction vertices, and the junction edges trimmed to them.
    let mut junction_vertices = Vec::with_capacity(plan.junctions.len());
    for junction in &plan.junctions {
        let id = take_id();
        result.vertices.push(VertexRecord { id, point: junction.point });
        trim_edge_at(result, junction.edge, junction.parameter, junction.eaten_vertex, id)?;
        junction_vertices.push(id);
    }
    // Removed edges leave every loop; removed faces leave their shells.
    for shell in &mut result.shells {
        shell.faces.retain(|face| !plan.removed_faces.contains(&face.id));
        for face in &mut shell.faces {
            for loop_record in &mut face.loops {
                loop_record.coedges.retain(|c| !plan.removed_edges.contains(&c.edge_id));
            }
            // A loop the void took whole — a hole rim on a mate, under the
            // pad — leaves the face.
            face.loops.retain(|loop_record| !loop_record.coedges.is_empty());
        }
    }
    result.shells.retain(|shell| !shell.faces.is_empty());
    result.edges.retain(|edge| !plan.removed_edges.contains(&edge.id));

    // The piece edges.
    let mut piece_edges: Vec<(u64, u64, u64)> = Vec::with_capacity(plan.pieces.len()); // (edge, from vertex, to vertex)
    for (_, from, to, piece) in &plan.pieces {
        let id = take_id();
        let [t0, t1] = piece.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        result.edges.push(EdgeRecord {
            id,
            curve: piece.curve.clone(),
            t0,
            t1,
            start_vertex_id: junction_vertices[*from],
            end_vertex_id: junction_vertices[*to],
            degenerate: false,
            name: None,
        });
        piece_edges.push((id, junction_vertices[*from], junction_vertices[*to]));
    }

    // Each affected face: close every break in its loops with its piece.  The
    // sense each piece takes on its obstacle face is recorded: the blend's
    // coedge on that piece must be the OPPOSITE, which is what makes the
    // result a manifold, and the blend's inner loop is built from that.
    let mut obstacle_forward: Vec<Option<bool>> = vec![None; plan.pieces.len()];
    let mut affected: Vec<u64> = plan.pieces.iter().map(|(face, ..)| *face).collect();
    affected.sort_unstable();
    affected.dedup();
    for face_id in &affected {
        let loops: Vec<LoopRecord> = face_of(result, *face_id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "affected_face", "blend restriction: an affected face vanished"))?
            .loops
            .clone();
        let mut rebuilt_loops = Vec::with_capacity(loops.len());
        let mut closed_inserted: Vec<usize> = Vec::new();
        for loop_record in loops {
            let mut rebuilt: Vec<CoedgeRecord> = Vec::with_capacity(loop_record.coedges.len() + 2);
            let n = loop_record.coedges.len();
            for index in 0..n {
                let coedge = &loop_record.coedges[index];
                rebuilt.push(coedge.clone());
                let end = walk_vertex(result, coedge, true)?;
                let next_start = walk_vertex(result, &loop_record.coedges[(index + 1) % n], false)?;
                if end == next_start {
                    // A closed piece hung on this junction goes in here.
                    if let Some(k) = plan.pieces.iter().enumerate().position(|(k, (face, from, to, _))| {
                        *face == *face_id && from == to && piece_edges[k].1 == end && !closed_inserted.contains(&k)
                    }) {
                        closed_inserted.push(k);
                        // Both senses start and end on the same vertex; the
                        // loop's uv walk decides: the piece's pcurve must
                        // START where the preceding coedge's pcurve ends —
                        // on a periodic carrier the two senses start at the
                        // two representatives of the seam.
                        let previous_end = walk_uv(coedge, true)?;
                        let pcurve = &plan.pieces[k].3.on_obstacle;
                        let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                        let (a, b) = (pcurve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?, pcurve.evaluate(q1).or_refuse(KernelStage::Refine, "evaluate")?);
                        let gap = |p: Vec3| ((p.x - previous_end[0]).powi(2) + (p.y - previous_end[1]).powi(2)).sqrt();
                        let forward = gap(a) <= gap(b);
                        obstacle_forward[k] = Some(forward);
                        rebuilt.push(CoedgeRecord {
                            id: take_id(),
                            edge_id: piece_edges[k].0,
                            forward,
                            pcurve: if forward { pcurve.clone() } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
                        });
                    }
                    continue;
                }
                let Some(k) = plan.pieces.iter().enumerate().position(|(k, (face, ..))| {
                    *face == *face_id
                        && ((piece_edges[k].1 == end && piece_edges[k].2 == next_start)
                            || (piece_edges[k].2 == end && piece_edges[k].1 == next_start))
                }) else {
                    return Err(KernelRefusal::internal(KernelStage::Sew, "face_break", format!(
                        "blend restriction: face {face_id} breaks between vertices {end} and \
                         {next_start} and no trim piece closes it"
                    )));
                };
                let forward = piece_edges[k].1 == end;
                obstacle_forward[k] = Some(forward);
                let pcurve = &plan.pieces[k].3.on_obstacle;
                rebuilt.push(CoedgeRecord {
                    id: take_id(),
                    edge_id: piece_edges[k].0,
                    forward,
                    pcurve: if forward { pcurve.clone() } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
                });
            }
            let candidate = LoopRecord { coedges: rebuilt, ..loop_record };
            rebuilt_loops.push(candidate);
        }
        let face = result
            .shells
            .iter_mut()
            .flat_map(|shell| &mut shell.faces)
            .find(|face| face.id == *face_id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "affected_face", "blend restriction: an affected face vanished"))?;
        face.loops = rebuilt_loops;
    }

    // The blend: each piece with the sense opposite to its obstacle's, the
    // pieces chained through their junctions into one inner loop per cycle.
    let blend_sense: Vec<bool> = obstacle_forward
        .iter()
        .enumerate()
        .map(|(k, sense)| {
            sense.map(|forward| !forward).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Refine,
                    "trim_piece_obstacle",
                    format!("blend restriction: trim piece {} was placed on no obstacle face", piece_edges[k].0),
                )
            })
        })
        .collect::<Result<_, KernelRefusal>>()?;
    let walk = |k: usize| -> (u64, u64) {
        if blend_sense[k] { (piece_edges[k].1, piece_edges[k].2) } else { (piece_edges[k].2, piece_edges[k].1) }
    };
    let mut unused: Vec<usize> = (0..plan.pieces.len()).collect();
    let mut inner_loops: Vec<Vec<CoedgeRecord>> = Vec::new();
    while let Some(first) = unused.first().copied() {
        unused.retain(|k| *k != first);
        let mut cycle: Vec<usize> = vec![first];
        let (start, mut at) = walk(first);
        while at != start {
            let Some(position) = unused.iter().position(|k| walk(*k).0 == at) else {
                return Err(KernelRefusal::internal(KernelStage::Refine, "void_trim_loop", format!(
                    "blend restriction: the void's trim pieces do not close into a loop on the blend \
                     (no piece leaves vertex {at} with the sense its obstacle face requires)"
                )));
            };
            let k = unused.remove(position);
            at = walk(k).1;
            cycle.push(k);
        }
        let mut coedges = Vec::with_capacity(cycle.len());
        for k in cycle {
            let pcurve = &plan.pieces[k].3.on_blend;
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: piece_edges[k].0,
                forward: blend_sense[k],
                pcurve: if blend_sense[k] { pcurve.clone() } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
            });
        }
        inner_loops.push(coedges);
    }
    // Merge the shells the void's faces lived in into the blend's shell: the
    // void now opens into the part.
    let blend_shell = result
        .shells
        .iter()
        .position(|shell| shell.faces.iter().any(|face| face.id == plan.blend_face))
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "blend_face", "blend restriction: the blend face is missing"))?;
    let mut moved: Vec<FaceRecord> = Vec::new();
    for (index, shell) in result.shells.iter_mut().enumerate() {
        if index == blend_shell {
            continue;
        }
        if shell.faces.iter().any(|face| affected.contains(&face.id)) {
            moved.append(&mut shell.faces);
        }
    }
    result.shells[blend_shell].faces.extend(moved);
    result.shells.retain(|shell| !shell.faces.is_empty());
    let blend = result
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .find(|face| face.id == plan.blend_face)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "blend_face", "blend restriction: the blend face is missing"))?;
    for coedges in inner_loops {
        blend.loops.push(LoopRecord { id: take_id(), coedges });
    }
    for face in std::iter::once(plan.blend_face).chain(affected.iter().copied()) {
        check_loops_closed(result, face)?;
    }
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        for face in std::iter::once(plan.blend_face).chain(affected.iter().copied()) {
            dump_loop_walk(result, face, "after the void");
        }
        eprintln!(
            "blend restriction: face {} trimmed against a void: {} junctions, {} pieces over faces \
             {:?}, {} edges and {} faces removed",
            plan.blend_face,
            plan.junctions.len(),
            plan.pieces.len(),
            affected,
            plan.removed_edges.len(),
            plan.removed_faces.len(),
        );
    }
    Ok(())
}

/// Debug: every coedge of a face's loops with its walk and its pcurve's u at
/// the walk's start and end.
fn dump_loop_walk(result: &BrepSolid, face_id: u64, label: &str) {
    let Some(face) = face_of(result, face_id) else { return };
    for (index, loop_record) in face.loops.iter().enumerate() {
        eprintln!("blend restriction: {label}: face {face_id} loop {index}");
        for coedge in &loop_record.coedges {
            let Some(edge) = edge_of(result, coedge.edge_id) else { continue };
            let (start, end) = if coedge.forward {
                (edge.start_vertex_id, edge.end_vertex_id)
            } else {
                (edge.end_vertex_id, edge.start_vertex_id)
            };
            let [q0, q1] = coedge.pcurve.domain().unwrap_or([0.0, 0.0]);
            let (a, b) = (
                coedge.pcurve.evaluate(q0).unwrap_or_default(),
                coedge.pcurve.evaluate(q1).unwrap_or_default(),
            );
            eprintln!(
                "  edge {} forward={} walk {start}->{end} pcurve ({:.6}, {:.6}) -> ({:.6}, {:.6})",
                coedge.edge_id, coedge.forward, a.x, a.y, b.x, b.y
            );
        }
    }
}

/// Swap one coedge for the trim edge, keeping the loop's traversal sense.
fn replace_coedge(
    result: &mut BrepSolid,
    face_id: u64,
    old_edge: u64,
    trim_edge: u64,
    pcurve: &NurbsCurve,
    trim_vertices: [u64; 2],
) -> Result<(), KernelRefusal> {
    let old = edge_of(result, old_edge)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "replaced_piece", "blend restriction: the piece being replaced vanished"))?;
    // The trim edge runs `trim_vertices[0] -> trim_vertices[1]`, and the piece
    // it replaces runs between the same two vertices, so the walk decides the
    // sense: a coedge that STARTS at `trim_vertices[0]` uses the trim forward.
    let (old_start, old_end) = (old.start_vertex_id, old.end_vertex_id);
    let face = result
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .find(|face| face.id == face_id)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "splice_face", "blend restriction: a face vanished before its splice"))?;
    for loop_record in &mut face.loops {
        let Some(position) = loop_record.coedges.iter().position(|c| c.edge_id == old_edge) else {
            continue;
        };
        let walked_from = if loop_record.coedges[position].forward {
            old_start
        } else {
            old_end
        };
        let forward = walked_from == trim_vertices[0];
        loop_record.coedges[position] = CoedgeRecord {
            id: loop_record.coedges[position].id,
            edge_id: trim_edge,
            forward,
            pcurve: if forward { pcurve.clone() } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
        };
        return Ok(());
    }
    Err(KernelRefusal::internal(KernelStage::Sew, "face_edge", format!(
        "blend restriction: face {face_id} does not use edge {old_edge}"
    )))
}

/// Merge the mate's rail loop and obstacle loop into one, dropping the two
/// coedges the restriction removed.
fn merge_mate_loops(
    result: &mut BrepSolid,
    mate_face: u64,
    rail_middle: u64,
    eaten: &[u64],
) -> Result<(), KernelRefusal> {
    let face = result
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .find(|face| face.id == mate_face)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend restriction: the mate face vanished before its merge"))?;
    let Some(rail_loop) = face
        .loops
        .iter()
        .position(|l| l.coedges.iter().any(|c| c.edge_id == rail_middle))
    else {
        return Err(KernelRefusal::internal(KernelStage::Sew, "mate_rail_piece", "blend restriction: the mate lost the rail's middle piece"));
    };
    let Some(obstacle_loop) = face
        .loops
        .iter()
        .position(|l| l.coedges.iter().any(|c| eaten.contains(&c.edge_id)))
    else {
        return Err(KernelRefusal::internal(KernelStage::Sew, "mate_obstacle_arc", "blend restriction: the mate lost the obstacle's eaten arc"));
    };
    if rail_loop == obstacle_loop {
        return Err(
            KernelRefusal::internal(KernelStage::Sew, "mate_loops_joined", "blend restriction: the rail and the obstacle arc are already in one loop on the \
             mate; the merge has nothing to join"),
        );
    }
    // Rotate each loop so the removed coedge sits LAST, then drop it: what is
    // left of each is an open chain, and the two chains join end to end.
    let mut rail_chain = face.loops[rail_loop].coedges.clone();
    let rail_at = rail_chain
        .iter()
        .position(|c| c.edge_id == rail_middle)
        .expect("located above");
    let rail_length = rail_chain.len();
    rail_chain.rotate_left((rail_at + 1) % rail_length);
    rail_chain.pop();
    // The eaten pieces are one contiguous block of the loop (cyclically):
    // rotate so the walk starts right after that block, then drop the block.
    let mut obstacle_chain = face.loops[obstacle_loop].coedges.clone();
    let obstacle_length = obstacle_chain.len();
    let Some(block_end) = (0..obstacle_length).find(|&index| {
        eaten.contains(&obstacle_chain[index].edge_id)
            && !eaten.contains(&obstacle_chain[(index + 1) % obstacle_length].edge_id)
    }) else {
        return Err(
            KernelRefusal::internal(KernelStage::Sew, "eaten_run", "blend restriction: the obstacle's eaten run is the whole of its loop on the mate"),
        );
    };
    obstacle_chain.rotate_left((block_end + 1) % obstacle_length);
    obstacle_chain.retain(|c| !eaten.contains(&c.edge_id));
    rail_chain.extend(obstacle_chain);
    face.loops[rail_loop].coedges = rail_chain;
    face.loops.remove(obstacle_loop);
    Ok(())
    // Whether the two chains meet end to end is not asserted here: the caller
    // walks every touched loop through `check_loops_closed`, which names the
    // pair of vertices that failed to meet rather than a bare orientation
    // claim.
}

/// Cut the mate in two along the rail's own loop: drop the rail's middle piece
/// and the eaten run, which leaves two open chains that each close on the two
/// crossing vertices.  The mate keeps the chain that walks on from the rail's
/// middle; the other becomes a new face on the same carrier, named as the
/// boolean names a split fragment (`{name}_1`, the first free suffix).  Any
/// other loop of the mate — a hole — goes with the chain whose uv polygon holds
/// it.  Returns the new face's id.
fn split_mate_loop(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    mate_face: u64,
    rail_middle: u64,
    eaten: &[u64],
) -> Result<u64, KernelRefusal> {
    let [eaten_piece] = eaten[..] else {
        return Err(KernelRefusal::internal(KernelStage::Sew, "mate_split_run", "blend restriction: a mate split eats exactly one piece of its boundary"));
    };
    let used_names: std::collections::HashSet<String> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter_map(|face| face.name.clone())
        .collect();
    let (face_id, loop_id) = (take_id(), take_id());
    let (shell_index, face_index) = result
        .shells
        .iter()
        .enumerate()
        .find_map(|(s, shell)| shell.faces.iter().position(|face| face.id == mate_face).map(|f| (s, f)))
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend restriction: the mate face vanished before its split"))?;
    let face = &mut result.shells[shell_index].faces[face_index];
    let Some(rail_loop) = face
        .loops
        .iter()
        .position(|l| l.coedges.iter().any(|c| c.edge_id == rail_middle) && l.coedges.iter().any(|c| c.edge_id == eaten_piece))
    else {
        return Err(KernelRefusal::internal(KernelStage::Sew, "mate_split_loop", "blend restriction: the rail's middle and the eaten run are not on one loop of the mate"));
    };
    let mut walk = face.loops[rail_loop].coedges.clone();
    let rail_at = walk.iter().position(|c| c.edge_id == rail_middle).expect("located above");
    walk.rotate_left(rail_at);
    let eaten_at = walk.iter().position(|c| c.edge_id == eaten_piece).expect("located above");
    let kept: Vec<CoedgeRecord> = walk[1..eaten_at].to_vec();
    let other: Vec<CoedgeRecord> = walk[eaten_at + 1..].to_vec();
    if kept.is_empty() || other.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Sew, "mate_split_chain", "blend restriction: the rail's middle and the eaten run are adjacent; the mate is not cut in two"));
    }
    face.loops[rail_loop].coedges = kept;
    let mut split = FaceRecord {
        id: face_id,
        surface: face.surface.clone(),
        same_sense: face.same_sense,
        loops: vec![LoopRecord { id: loop_id, coedges: other }],
        name: face.name.clone().map(|base| {
            (1usize..)
                .map(|suffix| format!("{base}_{suffix}"))
                .find(|candidate| !used_names.contains(candidate))
                .expect("an unbounded suffix search ends")
        }),
    };
    // A hole goes with the piece whose boundary holds it.
    let polygon = loop_polygon(&split, 0)?;
    let kept_loop = face.loops[rail_loop].id;
    let mut stays = Vec::with_capacity(face.loops.len());
    for loop_record in std::mem::take(&mut face.loops) {
        if loop_record.id != kept_loop
            && polygon_contains(&polygon, walk_uv(&loop_record.coedges[0], false)?)
        {
            split.loops.push(loop_record);
        } else {
            stays.push(loop_record);
        }
    }
    face.loops = stays;
    result.shells[shell_index].faces.push(split);
    Ok(face_id)
}

/// Every loop of a face must walk vertex to vertex: each coedge's finishing
/// vertex is the next coedge's starting one.  Checked here rather than left to
/// `validate`, because a merge that joins the wrong ends still validates as
/// two-use incidence.
fn check_loops_closed(result: &BrepSolid, face_id: u64) -> Result<(), KernelRefusal> {
    let face = face_of(result, face_id)
        .ok_or(KernelRefusal::internal(KernelStage::Validate, "loop_check_face", "blend restriction: a face vanished before its loop check"))?;
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        let mut walk: Vec<(u64, u64)> = Vec::with_capacity(loop_record.coedges.len());
        for coedge in &loop_record.coedges {
            let edge = edge_of(result, coedge.edge_id).ok_or_else(|| {
                format!(
                    "blend restriction: face {face_id} loop {loop_index} uses missing edge {}",
                    coedge.edge_id
                )
            }).or_refuse(KernelStage::Refine, "ok_or_else")?;
            walk.push(if coedge.forward {
                (edge.start_vertex_id, edge.end_vertex_id)
            } else {
                (edge.end_vertex_id, edge.start_vertex_id)
            });
        }
        for index in 0..walk.len() {
            let next = (index + 1) % walk.len();
            if walk[index].1 != walk[next].0 {
                return Err(KernelRefusal::internal(KernelStage::Validate, "loop_closure", format!(
                    "blend restriction: face {face_id} loop {loop_index} does not close — \
                     coedge {} finishes at vertex {} and the next starts at {}",
                    loop_record.coedges[index].edge_id, walk[index].1, walk[next].0
                )));
            }
        }
    }
    Ok(())
}
