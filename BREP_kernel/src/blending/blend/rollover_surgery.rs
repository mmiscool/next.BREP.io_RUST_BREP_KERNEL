//! The pieces of the split stripe (`rollover::march_pieces`) are sewn
//! into the solid directly, the §6.9 way, with no tool solid and no
//! boolean:
//!
//! * the THROUGH mate (the side whose track never left its face) swaps the
//!   blended edge's coedge for the pieces' rails on it, in loop order, the
//!   piece that wraps the anchored carrier's seam split at its seam station,
//!   and its seam edges trimmed to that station — the closed lane's own rim
//!   replacement (`replace_rim_in_face`, `trim_seam_edges`);
//! * every crossed TANGENT EDGE is cut at its crossing vertices, and the run
//!   between a leaving and a returning crossing — the run the ball rolled
//!   over — is consumed: dropped from the crossed mate's loop and, on the
//!   neighbour, replaced by the rolled piece's rail (the "bite");
//! * the crossed mate's rim loop is deleted and the loop that held the
//!   tangent edges is RE-TRACED through the plane pieces' rails: at a
//!   crossing vertex whose next run was consumed the trace follows the rail
//!   that departs there, so the rim's inner loop merges into the outer loop
//!   along the rails, and every closed trace is one face of the carrier (the
//!   reported bore mouth's top plane comes back in two pieces, the corner
//!   triangle past both pinches its own face);
//! * one blend face per piece, bounded by its two rails and the two RIBS it
//!   shares with its neighbours — each rib the section at the crossing
//!   station, one curve for both pieces because both interpolate that
//!   station to the bit.
//!
//! What this surgery does not build it refuses by name (`tangent_rollover`):
//! crossings on both sides, a rolled piece leaving across one edge and
//! returning across another, a crossed mate with further inner loops, or a
//! seam-structured through mate the march did not anchor.
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};
use super::rollover::{edge_parameter_near, RolledPiece, RolloverPlan, TANGENT_ROLLOVER_WHAT};
use super::stations::{raw_normal, station_parameters, BlendMate, Station};

fn not_built(what: impl Into<String>) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Sew, TANGENT_ROLLOVER_WHAT, format!("blend rollover: {} is not built, so this lane declines", what.into()))
}

/// The vertices a loop traverses `coedge` between: (start, end) in
/// traversal order.
fn coedge_ends(solid: &BrepSolid, coedge: &CoedgeRecord) -> Result<(u64, u64), KernelRefusal> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == coedge.edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "rollover_coedge_edge", format!("blend rollover: coedge {} names a missing edge {}", coedge.id, coedge.edge_id)))?;
    Ok(if coedge.forward { (edge.start_vertex_id, edge.end_vertex_id) } else { (edge.end_vertex_id, edge.start_vertex_id) })
}

/// Split `edge_id` at the given vertices (each already in the solid, with
/// its point on the edge) into consecutive sub-edges in increasing edge
/// parameter, and every coedge on the edge, in every face, into the matching
/// sub-coedges with their pcurves split at the same physical points. Returns
/// the sub-edge ids in edge-parameter order; the first sub-coedge of each
/// run keeps the old coedge's id. The old edge record is removed.
pub(super) fn split_edge_at_vertices(
    result: &mut BrepSolid,
    edge_id: u64,
    cuts: &[(u64, Vec3)],
    take_id: &mut dyn FnMut() -> u64,
) -> Result<Vec<u64>, KernelRefusal> {
    let position = result
        .edges
        .iter()
        .position(|edge| edge.id == edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "rollover_split_edge", format!("blend rollover: edge {edge_id} to split is missing")))?;
    let edge = result.edges[position].clone();
    let span = edge.t1 - edge.t0;
    let mut parameters: Vec<(f64, u64, Vec3)> = Vec::with_capacity(cuts.len());
    for &(vertex, point) in cuts {
        let t = edge_parameter_near(&edge, point)?;
        let fraction = (t - edge.t0) / span;
        if !(fraction > 1e-9 && fraction < 1.0 - 1e-9) {
            return Err(not_built(format!("a crossing at the end of tangent edge {edge_id} (fraction {fraction:.3e})")));
        }
        parameters.push((t, vertex, point));
    }
    parameters.sort_by(|a, b| a.0.total_cmp(&b.0));
    if parameters.windows(2).any(|pair| pair[1].0 - pair[0].0 <= 1e-9 * span.abs()) {
        return Err(not_built(format!("two crossings coincide on tangent edge {edge_id}")));
    }
    // The sub-edges.
    let mut bounds: Vec<(f64, u64)> = vec![(edge.t0, edge.start_vertex_id)];
    bounds.extend(parameters.iter().map(|(t, vertex, _)| (*t, *vertex)));
    bounds.push((edge.t1, edge.end_vertex_id));
    let mut sub_ids = Vec::with_capacity(bounds.len() - 1);
    for pair in bounds.windows(2) {
        let id = take_id();
        result.edges.push(EdgeRecord {
            id,
            curve: edge.curve.clone(),
            t0: pair[0].0,
            t1: pair[1].0,
            start_vertex_id: pair[0].1,
            end_vertex_id: pair[1].1,
            degenerate: false,
            name: edge.name.clone(),
        });
        sub_ids.push(id);
    }
    result.edges.remove(position);
    // The coedges: each split at the cut points on its own carrier.
    let band = crate::KernelTolerances::for_solid(result, 1e-7).pcurve_consistency;
    let mut fresh_coedges: Vec<CoedgeRecord> = Vec::new();
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            let FaceRecord { surface, loops, .. } = &mut *face;
            for loop_record in loops.iter_mut() {
                let Some(index) = loop_record.coedges.iter().position(|coedge| coedge.edge_id == edge_id) else {
                    continue;
                };
                let old = loop_record.coedges[index].clone();
                let [q0, q1] = old.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                // The pcurve parameters of the cuts, in pcurve order: a
                // reversed coedge meets the cuts in reverse edge order.
                let mut qs: Vec<f64> = Vec::with_capacity(parameters.len());
                for (t, _, point) in &parameters {
                    let fraction = (t - edge.t0) / span;
                    let seed = if old.forward { q0 + (q1 - q0) * fraction } else { q1 - (q1 - q0) * fraction };
                    qs.push(super::edge::trim_parameter_at_point(surface, &old.pcurve, seed, *point, band)?);
                }
                qs.sort_by(f64::total_cmp);
                let mut rest = old.pcurve.clone();
                let mut pcurves = Vec::with_capacity(qs.len() + 1);
                for q in &qs {
                    let (low, high) = rest.split(*q).or_refuse(KernelStage::Refine, "split")?;
                    pcurves.push(low);
                    rest = high;
                }
                pcurves.push(rest);
                // Sub-edges in traversal order.
                let ordered: Vec<u64> = if old.forward { sub_ids.clone() } else { sub_ids.iter().rev().copied().collect() };
                let replacements: Vec<CoedgeRecord> = ordered
                    .into_iter()
                    .zip(pcurves)
                    .enumerate()
                    .map(|(k, (sub_edge, pcurve))| CoedgeRecord { id: if k == 0 { old.id } else { 0 }, edge_id: sub_edge, forward: old.forward, pcurve })
                    .collect();
                fresh_coedges.extend(replacements.iter().filter(|coedge| coedge.id == 0).cloned());
                loop_record.coedges.splice(index..=index, replacements);
            }
        }
    }
    // Fresh ids for the extra coedges.
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            for loop_record in &mut face.loops {
                for coedge in &mut loop_record.coedges {
                    if coedge.id == 0 {
                        coedge.id = take_id();
                    }
                }
            }
        }
    }
    let _ = fresh_coedges;
    Ok(sub_ids)
}

/// A chord joining two crossing vertices across a face: the edge, its start
/// and end vertex in the EDGE's direction, and its pcurve on the face in
/// that direction.
pub(super) struct Chord {
    pub(super) edge_id: u64,
    pub(super) start: u64,
    pub(super) end: u64,
    pub(super) pcurve: NurbsCurve,
}

/// Re-trace a loop after the coedges on the `consumed` edges are dropped
/// and the `chords` added: from every surviving coedge not yet traced, the
/// walk follows the surviving coedge that departs the current end vertex,
/// or — where none does, the next run having been consumed — the chord that
/// touches that vertex, oriented to depart it. Every closed walk is one
/// loop; every chord must be used exactly once and every walk must close,
/// or the trace refuses by name. The chord coedges take fresh ids.
pub(super) fn retrace_loop(
    solid: &BrepSolid,
    loop_record: &LoopRecord,
    consumed: &[u64],
    chords: &[Chord],
    take_id: &mut dyn FnMut() -> u64,
) -> Result<Vec<Vec<CoedgeRecord>>, KernelRefusal> {
    struct Item {
        coedge: CoedgeRecord,
        start: u64,
        end: u64,
    }
    let mut surviving: Vec<Item> = Vec::new();
    for coedge in &loop_record.coedges {
        if consumed.contains(&coedge.edge_id) {
            continue;
        }
        let (start, end) = coedge_ends(solid, coedge)?;
        surviving.push(Item { coedge: coedge.clone(), start, end });
    }
    let mut visited = vec![false; surviving.len()];
    let mut chord_used = vec![false; chords.len()];
    let mut loops = Vec::new();
    for seed in 0..surviving.len() {
        if visited[seed] {
            continue;
        }
        let origin = surviving[seed].start;
        let mut traced: Vec<CoedgeRecord> = Vec::new();
        let mut current = seed;
        let mut guard = 0usize;
        loop {
            visited[current] = true;
            traced.push(surviving[current].coedge.clone());
            let mut at = surviving[current].end;
            // A chord departs where no surviving coedge does.
            if !surviving.iter().any(|item| item.start == at) {
                let Some(index) = chords.iter().enumerate().position(|(k, chord)| !chord_used[k] && (chord.start == at || chord.end == at)) else {
                    return Err(not_built(format!("re-tracing a loop whose run from vertex {at} was consumed with no rail departing there")));
                };
                chord_used[index] = true;
                let chord = &chords[index];
                let forward = chord.start == at;
                traced.push(CoedgeRecord {
                    id: take_id(),
                    edge_id: chord.edge_id,
                    forward,
                    pcurve: if forward { chord.pcurve.clone() } else { chord.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
                });
                at = if forward { chord.end } else { chord.start };
            }
            if at == origin {
                break;
            }
            let Some(next) = surviving.iter().position(|item| item.start == at) else {
                return Err(not_built(format!("re-tracing a loop that does not continue from vertex {at}")));
            };
            if visited[next] {
                return Err(not_built(format!("re-tracing a loop that revisits a coedge at vertex {at}")));
            }
            current = next;
            guard += 1;
            if guard > surviving.len() + chords.len() + 1 {
                return Err(not_built("re-tracing a loop that does not close"));
            }
        }
        loops.push(traced);
    }
    if let Some(unused) = chord_used.iter().position(|used| !used) {
        return Err(not_built(format!("re-tracing a loop that leaves the rail on edge {} unused", chords[unused].edge_id)));
    }
    Ok(loops)
}

/// A pcurve moved by whole carrier periods so its parameter midpoint lies in
/// the carrier's base domain in every closed direction.
fn into_base_period(pcurve: &NurbsCurve, surface: &crate::NurbsSurface) -> Result<NurbsCurve, KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let middle = pcurve.evaluate(0.5 * (q0 + q1)).or_refuse(KernelStage::Refine, "evaluate")?;
    let mut shifted = pcurve.clone();
    for (closed, domain, coordinate) in [
        (closed_u, surface.domain_u(), middle.x),
        (closed_v, surface.domain_v(), middle.y),
    ] {
        if !closed {
            continue;
        }
        let [low, high] = domain.or_refuse(KernelStage::Refine, "domain")?;
        let period = high - low;
        let laps = ((coordinate - low) / period).floor();
        if laps != 0.0 {
            for point in shifted.control_points.iter_mut() {
                if coordinate == middle.x {
                    point.x -= laps * period * point.w;
                } else {
                    point.y -= laps * period * point.w;
                }
            }
        }
    }
    Ok(shifted)
}

/// The side-`side` contact of a station.
fn contact(station: &Station, side: usize) -> Vec3 {
    if side == 0 { station.p1 } else { station.p2 }
}

fn uv_of(station: &Station, side: usize) -> [f64; 2] {
    if side == 0 { station.uv1 } else { station.uv2 }
}

/// The rollover's body: the pieces sewn into `solid` as the module
/// documentation describes.
#[allow(clippy::too_many_arguments)]
pub(super) fn rollover_surgery(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    mates: [&BlendMate<'_>; 2],
    plan: &RolloverPlan,
    pieces: &[RolledPiece],
    anchor_u: Option<f64>,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let crossings = &plan.crossings;
    let side = crossings[0].side;
    if crossings.iter().any(|crossing| crossing.side != side) {
        return Err(not_built("a stripe whose BOTH tracks roll over a tangent neighbour"));
    }
    let through = 1 - side;
    let crossed_mate = mates[side];
    let through_mate = mates[through];
    if through_mate.face.loops[through_mate.loop_index].coedges.len() > 1 && anchor_u.is_none() {
        return Err(not_built("a seam-structured through carrier the march did not anchor"));
    }
    if crossed_mate.face.loops[crossed_mate.loop_index].coedges.len() != 1 {
        return Err(not_built("a rim that shares its loop with other edges of the crossed mate"));
    }
    if crossed_mate.face.loops.len() != 2 {
        return Err(not_built(format!("a crossed mate with {} loops (the rim's and the one holding the tangent edges are the two built)", crossed_mate.face.loops.len())));
    }
    for piece in pieces {
        if piece.rolled_side.is_some() && crossings[piece.from].edge_id != crossings[piece.to].edge_id {
            return Err(not_built("a rolled piece that leaves its mate across one tangent edge and returns across another"));
        }
    }
    let mut result = solid.clone();
    let mut take_id = super::edge::fresh_id_source(solid);

    // ---- vertices: the crossing stations' contacts, and the seam station ----
    let mut through_vertex = Vec::with_capacity(crossings.len());
    let mut crossed_vertex = Vec::with_capacity(crossings.len());
    for crossing in crossings {
        let (_, station) = crossing.solved.as_ref().ok_or_else(|| not_built("an unsolved crossing station"))?;
        let w = take_id();
        result.vertices.push(VertexRecord { id: w, point: contact(station, through) });
        through_vertex.push(w);
        let v = take_id();
        result.vertices.push(VertexRecord { id: v, point: contact(station, side) });
        crossed_vertex.push(v);
    }
    let wrapping = pieces.iter().position(|piece| piece.seam_station.is_some());
    let seam = match (anchor_u, wrapping) {
        (Some(_), Some(index)) => {
            let piece = &pieces[index];
            let station = &piece.stations[piece.seam_station.expect("the wrapping piece")];
            let id = take_id();
            result.vertices.push(VertexRecord { id, point: contact(station, through) });
            Some((index, id, *station))
        }
        (Some(_), None) => return Err(not_built("an anchored march whose seam station fell on no piece")),
        (None, _) => None,
    };

    // ---- the rails: one edge per piece and side (the wrapping piece's
    // through rail in two, split at the seam station) ----
    struct Rails {
        /// (edge id, fit-space window, pcurve on the through carrier in the
        /// base period), in u order.
        through: Vec<(u64, [f64; 2], NurbsCurve)>,
        crossed: (u64, [f64; 2], NurbsCurve),
    }
    let mut rails: Vec<Rails> = Vec::with_capacity(pieces.len());
    for (index, piece) in pieces.iter().enumerate() {
        let [u_start, u_end] = piece.rows.u_domain;
        let (through_curve, through_pcurve, crossed_curve, crossed_pcurve) = if through == 0 {
            (&piece.rows.cr, &piece.rows.cr_pcurve, &piece.rows.cs, &piece.rows.cs_pcurve)
        } else {
            (&piece.rows.cs, &piece.rows.cs_pcurve, &piece.rows.cr, &piece.rows.cr_pcurve)
        };
        let mut through_pieces = Vec::new();
        match seam {
            Some((wrapping, seam_vertex, _)) if wrapping == index => {
                let parameters = station_parameters(&piece.stations);
                let k = piece.seam_station.expect("the wrapping piece");
                let u_seam = u_start + (u_end - u_start) * parameters[k];
                let (rail_a, rail_b) = through_curve.split(u_seam).or_refuse(KernelStage::Refine, "split")?;
                let (pc_a, pc_b) = through_pcurve.split(u_seam).or_refuse(KernelStage::Refine, "split")?;
                let edge_a = take_id();
                result.edges.push(EdgeRecord { id: edge_a, curve: rail_a, t0: u_start, t1: u_seam, start_vertex_id: through_vertex[piece.from], end_vertex_id: seam_vertex, degenerate: false, name: None });
                through_pieces.push((edge_a, [u_start, u_seam], into_base_period(&pc_a, &through_mate.face.surface)?));
                let edge_b = take_id();
                result.edges.push(EdgeRecord { id: edge_b, curve: rail_b, t0: u_seam, t1: u_end, start_vertex_id: seam_vertex, end_vertex_id: through_vertex[piece.to], degenerate: false, name: None });
                through_pieces.push((edge_b, [u_seam, u_end], into_base_period(&pc_b, &through_mate.face.surface)?));
            }
            _ => {
                let id = take_id();
                result.edges.push(EdgeRecord { id, curve: through_curve.clone(), t0: u_start, t1: u_end, start_vertex_id: through_vertex[piece.from], end_vertex_id: through_vertex[piece.to], degenerate: false, name: None });
                through_pieces.push((id, [u_start, u_end], into_base_period(through_pcurve, &through_mate.face.surface)?));
            }
        }
        let crossed_id = take_id();
        result.edges.push(EdgeRecord { id: crossed_id, curve: crossed_curve.clone(), t0: u_start, t1: u_end, start_vertex_id: crossed_vertex[piece.from], end_vertex_id: crossed_vertex[piece.to], degenerate: false, name: None });
        let crossed_face = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == piece.faces[side])
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "rollover_crossed_carrier", "blend rollover: a piece's crossed-side carrier vanished"))?;
        rails.push(Rails { through: through_pieces, crossed: (crossed_id, [u_start, u_end], into_base_period(crossed_pcurve, &crossed_face.surface)?) });
    }

    // ---- the ribs: the section at every crossing station, from the v = 0
    // end (side 0's contact) to the v = 1 end ----
    let mut rib_edge = Vec::with_capacity(crossings.len());
    for (index, _) in crossings.iter().enumerate() {
        let piece_index = pieces.iter().position(|piece| piece.from == index).ok_or_else(|| not_built("a crossing no piece starts at"))?;
        let piece = &pieces[piece_index];
        let [u_start, _] = piece.rows.u_domain;
        let curve = piece.rows.surface.iso_curve_u(u_start).or_refuse(KernelStage::Refine, "iso_curve_u")?;
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let (v0_vertex, v1_vertex) = if through == 0 { (through_vertex[index], crossed_vertex[index]) } else { (crossed_vertex[index], through_vertex[index]) };
        let id = take_id();
        result.edges.push(EdgeRecord { id, curve, t0, t1, start_vertex_id: v0_vertex, end_vertex_id: v1_vertex, degenerate: false, name: None });
        rib_edge.push(id);
    }

    // ---- the through mate: the rim's coedge becomes the through rails,
    // starting at the seam when the carrier has one ----
    let mut ordered: Vec<(u64, [f64; 2], NurbsCurve)> = Vec::new();
    match seam {
        Some((wrapping, _, _)) => {
            ordered.push(rails[wrapping].through[1].clone());
            for k in 1..pieces.len() {
                let index = (wrapping + k) % pieces.len();
                ordered.extend(rails[index].through.iter().cloned());
            }
            ordered.push(rails[wrapping].through[0].clone());
        }
        None => {
            for rail in &rails {
                ordered.extend(rail.through.iter().cloned());
            }
        }
    }
    let through_uv_new = match seam {
        Some((_, _, station)) => uv_of(&station, through),
        None => uv_of(&pieces[0].stations[0], through),
    };
    let mut repaired = super::edge::replace_rim_in_face(&mut result, edge, through_mate, &ordered, through_uv_new)?;
    let new_edge_ids: Vec<u64> = rails
        .iter()
        .flat_map(|rail| rail.through.iter().map(|(id, ..)| *id).chain(std::iter::once(rail.crossed.0)))
        .chain(rib_edge.iter().copied())
        .collect();
    let through_face_edges: Vec<u64> = through_mate
        .face
        .loops
        .iter()
        .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.edge_id))
        .collect();
    if let Some((_, seam_vertex, station)) = seam {
        let target = (seam_vertex, uv_of(&station, through)[1], contact(&station, through));
        repaired |= super::edge::trim_seam_edges(&mut result, edge, &new_edge_ids, &through_face_edges, target, target)?;
    }

    // ---- the crossed tangent edges: cut at the crossing vertices ----
    let mut crossed_edges: Vec<u64> = crossings.iter().map(|crossing| crossing.edge_id).collect();
    crossed_edges.sort_unstable();
    crossed_edges.dedup();
    // Per rolled piece: the sub-edge its roll consumed.
    let mut consumed: Vec<(usize, u64)> = Vec::new();
    for edge_id in &crossed_edges {
        let cuts: Vec<(u64, Vec3)> = crossings
            .iter()
            .enumerate()
            .filter(|(_, crossing)| crossing.edge_id == *edge_id)
            .map(|(index, crossing)| (crossed_vertex[index], crossing.point))
            .collect();
        let sub_edges = split_edge_at_vertices(&mut result, *edge_id, &cuts, &mut take_id)?;
        for (index, piece) in pieces.iter().enumerate() {
            if piece.rolled_side.is_none() || crossings[piece.from].edge_id != *edge_id {
                continue;
            }
            let (a, b) = (crossed_vertex[piece.from], crossed_vertex[piece.to]);
            let sub = sub_edges
                .iter()
                .copied()
                .find(|id| {
                    result.edges.iter().any(|edge| edge.id == *id && ((edge.start_vertex_id == a && edge.end_vertex_id == b) || (edge.start_vertex_id == b && edge.end_vertex_id == a)))
                })
                .ok_or_else(|| not_built("a rolled piece whose crossings do not bound one run of the tangent edge"))?;
            consumed.push((index, sub));
        }
    }

    // ---- the crossed mate: drop the rim loop, re-trace the other through
    // the plane pieces' rails, one face per trace ----
    let consumed_ids: Vec<u64> = consumed.iter().map(|(_, id)| *id).collect();
    let chords: Vec<Chord> = pieces
        .iter()
        .enumerate()
        .filter(|(_, piece)| piece.rolled_side.is_none())
        .map(|(index, piece)| Chord { edge_id: rails[index].crossed.0, start: crossed_vertex[piece.from], end: crossed_vertex[piece.to], pcurve: rails[index].crossed.2.clone() })
        .collect();
    let traced = {
        let face = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == crossed_mate.face.id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "rollover_crossed_mate", "blend rollover: the crossed mate vanished"))?;
        let outer = 1 - crossed_mate.loop_index;
        retrace_loop(&result, &face.loops[outer], &consumed_ids, &chords, &mut take_id)?
    };
    if traced.is_empty() {
        return Err(not_built("a crossed mate whose loop vanished entirely"));
    }
    let outer_sign = {
        let face = result.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == crossed_mate.face.id).expect("found above");
        super::chain::signed_area(&face.loops[1 - crossed_mate.loop_index].coedges)?.signum()
    };
    for coedges in &traced {
        let area = super::chain::signed_area(coedges)?;
        if area.signum() != outer_sign || area == 0.0 {
            return Err(not_built(format!("a re-traced loop that winds against its face (area {area:.3e})")));
        }
    }
    {
        let shell_index = result
            .shells
            .iter()
            .position(|shell| shell.faces.iter().any(|face| face.id == crossed_mate.face.id))
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "rollover_shell", "blend rollover: the crossed mate's shell vanished"))?;
        let face_index = result.shells[shell_index].faces.iter().position(|face| face.id == crossed_mate.face.id).expect("found above");
        let template = result.shells[shell_index].faces[face_index].clone();
        let mut traced = traced.into_iter();
        let first = traced.next().expect("non-empty");
        result.shells[shell_index].faces[face_index].loops = vec![LoopRecord { id: take_id(), coedges: first }];
        for (k, coedges) in traced.enumerate() {
            result.shells[shell_index].faces.push(FaceRecord {
                id: take_id(),
                surface: template.surface.clone(),
                same_sense: template.same_sense,
                loops: vec![LoopRecord { id: take_id(), coedges }],
                // The cutter's boolean named the piece it split off `_1`;
                // the roll keeps that name for the same piece.
                name: template.name.as_ref().map(|value| format!("{value}_{}", k + 1)),
            });
        }
    }

    // ---- the neighbours: the consumed run becomes the rolled rail ----
    for (piece_index, sub_edge) in &consumed {
        let piece = &pieces[*piece_index];
        let neighbour_id = piece.faces[side];
        let (crossed_id, _, ref pcurve) = rails[*piece_index].crossed;
        let (a, b) = (crossed_vertex[piece.from], crossed_vertex[piece.to]);
        let mut bitten = false;
        for shell in &mut result.shells {
            for face in &mut shell.faces {
                if face.id != neighbour_id {
                    continue;
                }
                for loop_record in &mut face.loops {
                    let Some(index) = loop_record.coedges.iter().position(|coedge| coedge.edge_id == *sub_edge) else { continue };
                    let old = &loop_record.coedges[index];
                    let (start, _) = {
                        let sub = result.edges.iter().find(|edge| edge.id == *sub_edge).expect("the sub-edge exists");
                        if old.forward { (sub.start_vertex_id, sub.end_vertex_id) } else { (sub.end_vertex_id, sub.start_vertex_id) }
                    };
                    let forward = start == a;
                    if !forward && start != b {
                        return Err(not_built("a neighbour run whose ends are not the roll's crossings"));
                    }
                    loop_record.coedges[index] = CoedgeRecord {
                        id: old.id,
                        edge_id: crossed_id,
                        forward,
                        pcurve: if forward { pcurve.clone() } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
                    };
                    bitten = true;
                }
            }
        }
        if !bitten {
            return Err(not_built(format!("a neighbour (face {neighbour_id}) that does not border the consumed run")));
        }
    }

    // ---- the blend faces ----
    let shell_index = result
        .shells
        .iter()
        .position(|shell| shell.faces.iter().any(|face| face.id == through_mate.face.id))
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "rollover_shell", "blend rollover: the through mate's shell vanished"))?;
    for (index, piece) in pieces.iter().enumerate() {
        let [u_start, u_end] = piece.rows.u_domain;
        let rail = &rails[index];
        // The v = 0 side is side 0's rail, the v = 1 side is side 1's.
        let (side0, side1): (Vec<(u64, [f64; 2])>, Vec<(u64, [f64; 2])>) = if through == 0 {
            (rail.through.iter().map(|(id, window, _)| (*id, *window)).collect(), vec![(rail.crossed.0, rail.crossed.1)])
        } else {
            (vec![(rail.crossed.0, rail.crossed.1)], rail.through.iter().map(|(id, window, _)| (*id, *window)).collect())
        };
        let line = |u0: f64, v0: f64, u1: f64, v1: f64| crate::sweep_topology::parameter_line(u0, v0, u1, v1).or_refuse(KernelStage::Refine, "parameter_line");
        let mut coedges = Vec::new();
        for (id, window) in &side0 {
            coedges.push(CoedgeRecord { id: take_id(), edge_id: *id, forward: true, pcurve: line(window[0], 0.0, window[1], 0.0)? });
        }
        coedges.push(CoedgeRecord { id: take_id(), edge_id: rib_edge[piece.to], forward: true, pcurve: line(u_end, 0.0, u_end, 1.0)? });
        for (id, window) in side1.iter().rev() {
            coedges.push(CoedgeRecord { id: take_id(), edge_id: *id, forward: false, pcurve: line(window[1], 1.0, window[0], 1.0)? });
        }
        coedges.push(CoedgeRecord { id: take_id(), edge_id: rib_edge[piece.from], forward: false, pcurve: line(u_start, 1.0, u_start, 0.0)? });
        // Outward orientation: against side 0's carrier's outward normal at
        // the v = 0 rail, as the closed lane reads it.
        let mid_u = 0.5 * (u_start + u_end);
        let blend_normal = raw_normal(&piece.rows.surface, mid_u, 0.0)?;
        let side0_face = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == piece.faces[0])
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "rollover_carrier", "blend rollover: a piece's first carrier vanished"))?;
        let station_uv = piece.rows.cr_pcurve.evaluate(mid_u).or_refuse(KernelStage::Refine, "evaluate")?;
        let n0 = raw_normal(&side0_face.surface, station_uv.x, station_uv.y)?;
        let out0 = if side0_face.same_sense { n0 } else { n0.scale(-1.0) };
        let same_sense = blend_normal.dot(out0) >= 0.0;
        if !same_sense {
            coedges.reverse();
            for coedge in &mut coedges {
                coedge.forward = !coedge.forward;
                coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
            }
        }
        result.shells[shell_index].faces.push(FaceRecord {
            id: take_id(),
            surface: piece.rows.surface.clone(),
            same_sense,
            loops: vec![LoopRecord { id: take_id(), coedges }],
            name: name.map(|value| if index == 0 { value.to_string() } else { format!("{value}_{index}") }),
        });
    }

    // ---- sweep: the rim, the consumed runs and the rim's vertex ----
    let used_edges: std::collections::HashSet<u64> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    result.edges.retain(|candidate| used_edges.contains(&candidate.id));
    let used_vertices: std::collections::HashSet<u64> = result.edges.iter().flat_map(|candidate| [candidate.start_vertex_id, candidate.end_vertex_id]).collect();
    result.vertices.retain(|candidate| used_vertices.contains(&candidate.id));
    if repaired {
        let problems = result.validate();
        if !problems.is_empty() {
            return Err(KernelRefusal::internal(KernelStage::Validate, "neighbour_trim_closure", format!(
                "blend rollover: the repaired neighbour trim did not close ({} issue(s), first: {})",
                problems.len(),
                problems.first().map(|issue| issue.message.as_str()).unwrap_or("unknown")
            )));
        }
    }
    Ok(result)
}
