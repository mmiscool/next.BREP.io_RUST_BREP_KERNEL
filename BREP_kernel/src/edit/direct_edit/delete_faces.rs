//! Deleting a SET of faces — the multi-select half of Golovanov §6.12.
//!
//! [`delete_face_and_heal`](super::delete_face_and_heal) removes ONE face and
//! heals by re-intersecting its neighbours. That is the right heal for a
//! transition strip, and the wrong question entirely for the case the app
//! sends most often: a user has selected every face of a POCKET — its walls and
//! its floor — and wants the pocket gone.
//!
//! Such a selection is not a strip between neighbours; it is a PATCH. Its free
//! boundary is not four edges belonging to four other faces, it is one whole
//! HOLE LOOP of one surviving face — the mouth the pocket was sunk through. So
//! the heal is not to re-intersect anything, it is to CAP: drop the patch, drop
//! the hole loop, and the face the pocket was sunk into is the whole face
//! again. Nothing is refit; every surviving carrier and loop is bit-identical
//! to what it was.
//!
//! This is the same closed form [`cap_through_wall`](super::closed_heal) already
//! uses for a bore's wall, stated for an arbitrary set of faces instead of one:
//! a bore wall is simply the patch whose free boundary is TWO hole loops, in the
//! two faces the drill went in and out of. A blind pocket, a blind bore (wall +
//! floor disc), and a BOSS (a pad's wall + top, whose mouth is a hole loop in
//! the face it was grown from) are all the same operation.
//!
//! ## The lane gate
//!
//! Structural, decided before any geometry is touched:
//!
//! * a survivor loop that touches the selection is either ENTIRELY consumed by
//!   it — the ordinary case, a hole loop that goes with the patch — or CUT, and
//!   then the runs it keeps must splice back into one closed loop with the
//!   other cut loops on the same carrier.
//!
//! The second half of that is the case where the opening has no hole loop to
//! consume. A bore whose mouth is TANGENT to the boundary of the face it opens
//! through pinches that face, and the arrangement splits it in two rather than
//! hand back a loop that touches itself; the mouth then arrives as runs of two
//! faces' loops. Dropping those runs rejoins the pieces into the face they were
//! before the mouth pinched them — the same cap, stated for a free boundary
//! that cuts rather than encircles. A transition STRIP also cuts its
//! neighbours' loops and it does not splice: the runs stop at a genuine gap
//! between two different carriers, which is what re-intersection is for, so the
//! splice walk is the gate rather than a separate test (`splice_fragments`).
//!
//! The gate DECLINES one shape before it looks at a loop: a selection that
//! encloses every survivor it borders, so that no survivor would keep a loop
//! (`encloses_every_survivor`). A cap needs a face around the opening to drop
//! the opening out of, and there is none — the selection is not a pocket but,
//! say, every blend on a cube with all twelve edges rounded, whose six sides
//! are each bounded by strips alone. The lanes after this one read it; one that
//! none of them reads is refused by name rather than taken apart one face at a
//! time.
//!
//! ## The bridged rejoin
//!
//! The splice above hands one cut run over to the next AT A SHARED VERTEX,
//! which is what a pinched face's runs do. A band across a MIRRORED union does
//! not. A fillet groove that runs over the top of a part and down its side is
//! flanked, on both sides, by faces that are cosurface TWINS — the same plane,
//! the same fillet cylinder, reflected — and their runs stop at the setback and
//! tangent lines the band interrupted, a whole band's width apart.
//!
//! Those stops are not a gap between two DIFFERENT carriers, which is what
//! re-intersection is for. They are one edge with its middle taken out: at
//! such a stop the circuit doubles back on the single edge the two survivors
//! still share. So the circuit is cut at every CORNER — every handover where
//! it leaves one carrier for another — and the corners pair off: the circuit
//! leaves carrier A for carrier B at one and comes back at the other, and their
//! two spine edges are one straight line, each running away from its own
//! corner. The BRIDGE is that line's missing middle, and with it every
//! carrier's runs close into one loop again. Nothing is intersected and nothing
//! is guessed — every bridge is an edge the solid already had, continued.
//!
//! A rejoined group is the one place this lane refits. Its faces are twins
//! whose patches were fitted apart (a mirrored plane is the same plane with
//! reflected control points and a flipped `same_sense`, which bit-identity
//! reads as two carriers), so the host's carrier grows over everything it now
//! holds and every pcurve is rebuilt on it — exactly, for each carrier the
//! growth accepts. A transition strip is untouched by all of this: its four
//! corners wear four DIFFERENT carrier pairs and no two of them pair off, so it
//! goes on to the chain that re-intersects it.
//!
//! A selection that fails that test is not thereby ONE question. A user who
//! selects four counterbored holes AND the block's corner fillets has asked for
//! two different heals at once, and the whole-set gate says "not a patch" only
//! because the fillets leave part of the top face's outer loop standing. So a
//! selection the gate declines is split where it is ALREADY split — into
//! edge-connected COMPONENTS, each of which is put to the same gate on its own.
//! Patch components cap, strip components chain, and the two coexist in one
//! operation. Caps go FIRST: a cap is a pure topological drop that refits
//! nothing, so it cannot disturb a strip the chain has yet to reach, whereas a
//! chain rebuilds its neighbours' carriers and would move the loops a later cap
//! resolves its positions against.
//!
//! Once the gate says "patch", this lane OWNS the answer, refusals included —
//! the checks below name what is wrong instead of falling back to a heal that
//! was written for a different shape:
//!
//! * a consumed loop must be a HOLE in its face, not the loop that bounds the
//!   face's material. Selecting a pocket's three walls but not its floor makes
//!   the floor's OUTER loop the free boundary; capping that would leave a face
//!   with no boundary at all, so the refusal names the floor and says to select
//!   it too. A rejoined face answers the same question by AREA: it has to take
//!   in the region its free boundary encloses, not give up its own.
//! * the shell must stay CONNECTED. `validate()` deliberately does not ask
//!   (see `faces_are_connected`), and a patch whose removal severs the body is
//!   a thing this operation cannot represent.
//! * the GENUS must come out of the Euler accounting as a whole number ≥ 0.
//!   Capping a blind pocket leaves the genus alone; capping a through feature
//!   drops it by one. Neither is assumed — both fall out of the same count.
//!
//! A ONE-FACE selection is put to the same gate, and this is the one place the
//! refusals above are read as "not a cap" instead: see [`delete_one_face`]. It
//! is also why `closed_heal.rs`'s own `cap_through_wall` is now reached only
//! from inside `delete_face_and_heal` — every route through this module answers
//! a bore's wall with the general cap, which produces the same solid.

use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// A selection that passed the structural gate: the faces to lift off, the
/// survivor loops their free boundary consumes, and the edges that go with
/// them.
struct FacePatch {
    /// The shell the patch lives in (all its faces share one).
    shell_index: usize,
    /// Ids of the selected faces.
    face_ids: HashSet<u64>,
    /// `(shell, face position, loop index)` of every survivor loop the patch's
    /// free boundary consumes whole — the hole loops that go with it.
    dropped_loops: Vec<(usize, usize, usize)>,
    /// The survivor faces whose loops the free boundary only CUTS, grouped and
    /// rebuilt — the pinch-split half of the cap (see the module docs).
    merges: Vec<FaceMerge>,
    /// Every edge the patch uses: its interior edges and its free boundary.
    /// All of them lose both their coedges, so all of them go.
    edges: HashSet<u64>,
    /// The edges the rejoin lays ACROSS the openings the patch leaves in
    /// survivor–survivor edges — the piece of a setback line, or of a
    /// tangent line, that the deleted band interrupted. Empty for a pure
    /// cap, where every survivor loop closes on the topology it already has.
    bridges: Vec<EdgeRecord>,
}

/// Survivor faces on ONE carrier that the free boundary's removal rejoins into
/// a single face.
///
/// The host is the first face in shell order — the convention
/// `merge_same_surface_faces` already follows: it keeps its id and its name,
/// and the others' names disappear the way the deleted face's name does.
struct FaceMerge {
    /// Face positions in the patch's shell, ascending; the first is the host.
    faces: Vec<usize>,
    /// The host's loops after the splice: the widest loop first (`validate`
    /// reads loop 0 as the face's outer one), then the rest — every rebuilt
    /// loop, plus every loop of every face in the group the boundary never
    /// touched.
    loops: Vec<LoopRecord>,
    /// The carrier those loops were re-fitted to, when the rejoin had to GROW
    /// one to span the opening. `None` for the pinch rejoin, where the group's
    /// faces already share one carrier bit-for-bit and every pcurve is the one
    /// it was.
    surface: Option<NurbsSurface>,
}

/// A maximal RUN of coedges that a CUT survivor loop keeps, in that loop's own
/// traversal order.
struct Fragment {
    /// The survivor face's position in the patch's shell.
    face: usize,
    coedges: Vec<CoedgeRecord>,
    /// Traversal-start vertex of the first coedge.
    start: u64,
    /// Traversal-end vertex of the last coedge.
    end: u64,
}

/// How many coedges a given edge gets from the selection and from the rest of
/// the solid.
#[derive(Clone, Copy, Default)]
struct EdgeCensus {
    selected: usize,
    kept: usize,
}

fn census(solid: &BrepSolid, face_ids: &HashSet<u64>) -> HashMap<u64, EdgeCensus> {
    let mut counts: HashMap<u64, EdgeCensus> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let selected = face_ids.contains(&face.id);
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let entry = counts.entry(coedge.edge_id).or_default();
            if selected {
                entry.selected += 1;
            } else {
                entry.kept += 1;
            }
        }
    }
    counts
}

/// Whether every survivor the selection borders is bounded by the selection
/// ALONE: each face outside it that shares an edge with it has every coedge of
/// every loop on such an edge.
///
/// That is the one shape a cap cannot be. A cap drops the free boundary out of
/// the face AROUND the patch, and here no survivor keeps a loop to be that
/// face. A cube with all twelve edges rounded, every blend selected, is the
/// shape that asks: each of its six sides is bounded by four strips and
/// nothing else, and un-rounding it is the blend network's question.
///
/// False when the selection borders nothing at all — a whole closed shell —
/// which the patch gate declines on its own terms.
fn encloses_every_survivor(
    solid: &BrepSolid,
    selected: &HashSet<u64>,
    counts: &HashMap<u64, EdgeCensus>,
) -> bool {
    let on_selection = |coedge: &CoedgeRecord| {
        counts
            .get(&coedge.edge_id)
            .is_some_and(|count| count.selected > 0)
    };
    let mut bordered = false;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if selected.contains(&face.id) {
            continue;
        }
        let mut coedges = face.loops.iter().flat_map(|loop_record| &loop_record.coedges);
        if !coedges.clone().any(on_selection) {
            continue;
        }
        if !coedges.all(on_selection) {
            return false;
        }
        bordered = true;
    }
    bordered
}

/// [`encloses_every_survivor`] for a selection, and the faces it encloses.
/// `None` when the selection does not enclose every survivor it borders.
pub(super) fn enclosed_survivors(solid: &BrepSolid, face_ids: &[u64]) -> Option<Vec<u64>> {
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let counts = census(solid, &selected);
    if !encloses_every_survivor(solid, &selected, &counts) {
        return None;
    }
    Some(
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| {
                !selected.contains(&face.id)
                    && face
                        .loops
                        .iter()
                        .flat_map(|loop_record| &loop_record.coedges)
                        .any(|coedge| counts.get(&coedge.edge_id).is_some_and(|count| count.selected > 0))
            })
            .map(|face| face.id)
            .collect(),
    )
}

/// The refusal for a selection that encloses every survivor it borders and
/// that no lane after the cap has read: nothing is left to cap against, and
/// one face at a time is not an answer to a set whose every neighbour is going
/// with it.
///
/// Two shapes of it have a reason of their own, and the refusal names it:
///
/// * the selection is every face of a closed shell but ONE — five sides of a
///   box. It is a patch on the sixth whose free boundary is that face's whole
///   outer loop: removing it leaves one face with no boundary at all, and one
///   face bounds no solid.
/// * a CURVED face is among the enclosed — the eight vertex blends of a box
///   whose twelve strips are selected. Once the strips go, the walls meet in
///   sharp edges, and a sphere rolled against three planes stands off each of
///   them (at `r·√2` on a cube): nothing is left to bound it, so it cannot stay.
fn enclosed_selection_refusal(
    solid: &BrepSolid,
    face_ids: &[u64],
    op: &str,
) -> Option<KernelRefusal> {
    let enclosed = enclosed_survivors(solid, face_ids)?;
    let labels = |ids: &[u64]| -> String {
        let mut labels: Vec<String> = ids
            .iter()
            .take(4)
            .filter_map(|face_id| find_face(solid, *face_id))
            .map(|(shell, position)| face_label(&solid.shells[shell].faces[position]))
            .collect();
        if ids.len() > labels.len() {
            labels.push(format!("{} more", ids.len() - labels.len()));
        }
        labels.join(", ")
    };
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let all_but_one = enclosed.len() == 1
        && solid.shells.iter().any(|shell| {
            shell.faces.iter().any(|face| face.id == enclosed[0])
                && shell
                    .faces
                    .iter()
                    .all(|face| selected.contains(&face.id) || face.id == enclosed[0])
        });
    if all_but_one {
        return Some(KernelRefusal::ill_posed(
            KernelStage::Classify,
            "all_but_one",
            format!(
                "{op}: the selection is every face of the closed shell but {} — a patch whose free \
                 boundary is that face's whole outer loop, so deleting it would leave one face with \
                 no boundary at all, and one face bounds no solid to heal to",
                labels(&enclosed)
            ),
        ));
    }
    let tolerance = (solid_model_scale(solid) * 1e-6).max(1e-7);
    let curved: Vec<u64> = enclosed
        .iter()
        .copied()
        .filter(|face_id| {
            find_face(solid, *face_id).is_some_and(|(shell, position)| {
                plane_of_surface(&solid.shells[shell].faces[position].surface, tolerance, op).is_err()
            })
        })
        .collect();
    if !curved.is_empty() {
        return Some(KernelRefusal::ill_posed(
            KernelStage::Classify,
            "enclosed_curved",
            format!(
                "{op}: {} {} bounded by the selection alone — a vertex blend whose strips are all \
                 selected cannot stay once they go, because the walls re-intersect in sharp edges \
                 that stand off it; select {} as well",
                labels(&curved),
                if curved.len() == 1 { "is a curved face" } else { "are curved faces" },
                if curved.len() == 1 { "it" } else { "them" }
            ),
        ));
    }
    Some(KernelRefusal::unsupported(
        KernelStage::Classify,
        "enclosed_survivors",
        format!(
            "{op}: every face the selection borders ({}) is bounded by the selection alone, so no \
             face is left around it to cap the opening against, and the selection does not read \
             as a corner blend, a closed band or a planar blend network — refusing rather than \
             deleting it one face at a time (deferred)",
            labels(&enclosed)
        ),
    ))
}

/// Decide whether `face_ids` is a PATCH — a set whose free boundary consumes,
/// or rejoins, whole loops of the faces around it — and if so gather what its
/// removal takes with it. `None` routes the caller to the one-face-at-a-time
/// chain.
fn classify_patch(solid: &BrepSolid, face_ids: &[u64]) -> Option<FacePatch> {
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let mut shells = face_ids
        .iter()
        .filter_map(|face_id| find_face(solid, *face_id))
        .map(|(shell_index, _)| shell_index);
    let shell_index = shells.next()?;
    if shells.any(|other| other != shell_index) {
        // A patch is a piece of ONE shell's surface.
        return None;
    }

    let counts = census(solid, &selected);
    let patch_edges: HashSet<u64> = counts
        .iter()
        .filter(|(_, count)| count.selected > 0)
        .map(|(edge_id, _)| *edge_id)
        .collect();
    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    // Every edge the patch touches is used exactly twice, or there is no cap
    // question here: the splice below reads "the other side of this free
    // boundary edge" as ONE survivor coedge, and a non-manifold edge has no
    // such reading. `validate()` rejects such a solid outright; this is the
    // statement that the lane depends on it. DEGENERATE edges are exempt — a
    // pole (a drill point's apex, a dome's north pole) is one coedge of one
    // face by construction, and it is not a cell of the complex the Euler count
    // or the splice reads.
    if patch_edges.iter().any(|edge_id| {
        let count = counts[edge_id];
        !edges.get(edge_id).is_some_and(|edge| edge.degenerate)
            && count.selected + count.kept != 2
    }) {
        return None;
    }
    // Nothing to cap AGAINST: every survivor the selection borders would lose
    // every loop it has. `cap_preconditions` would only refuse it ("the whole
    // boundary of ..."), and it is not a pocket at all — it is what a blend
    // network looks like when every face it touches is bounded by blends. So
    // the lane declines here, and the lanes after it read the set.
    if encloses_every_survivor(solid, &selected, &counts) {
        return None;
    }

    let mut dropped_loops = Vec::new();
    let mut fragments: Vec<Fragment> = Vec::new();
    for (shell_position, shell) in solid.shells.iter().enumerate() {
        for (face_position, face) in shell.faces.iter().enumerate() {
            if selected.contains(&face.id) {
                continue;
            }
            for (loop_index, loop_record) in face.loops.iter().enumerate() {
                let consumed: Vec<bool> = loop_record
                    .coedges
                    .iter()
                    .map(|coedge| patch_edges.contains(&coedge.edge_id))
                    .collect();
                if !consumed.iter().any(|flag| *flag) {
                    continue;
                }
                if consumed.iter().all(|flag| *flag) {
                    dropped_loops.push((shell_position, face_position, loop_index));
                    continue;
                }
                // The selection stops part-way along this survivor's loop.
                // Usually that means a strip between neighbours — the chain's
                // question. But a face the free boundary is TANGENT to has no
                // hole loop to consume: the tangency pinches the face and the
                // arrangement splits it, so the opening's rim arrives as runs
                // of several faces' loops instead. Those runs splice back into
                // one loop; a strip's runs do not, and `splice_fragments` is
                // where the two part company.
                if shell_position != shell_index {
                    return None;
                }
                fragments.extend(loop_fragments(face_position, loop_record, &consumed, &edges)?);
            }
        }
    }
    if dropped_loops.is_empty() && fragments.is_empty() {
        // Nothing outside the selection borders it — the "patch" is a whole
        // closed shell. Capping has nothing to cap onto.
        return None;
    }
    let (merges, bridges) = splice_fragments(solid, shell_index, &fragments, &patch_edges)?;

    Some(FacePatch {
        shell_index,
        face_ids: selected,
        dropped_loops,
        merges,
        edges: patch_edges,
        bridges,
    })
}

/// The traversal-start and -end vertices of a coedge.
fn coedge_ends(coedge: &CoedgeRecord, edges: &HashMap<u64, &EdgeRecord>) -> Option<(u64, u64)> {
    let edge = edges.get(&coedge.edge_id)?;
    Some(if coedge.forward {
        (edge.start_vertex_id, edge.end_vertex_id)
    } else {
        (edge.end_vertex_id, edge.start_vertex_id)
    })
}

/// The uv endpoints of a coedge's pcurve, in traversal order (a pcurve runs
/// with its coedge, not with its edge — see `coedge_sample`). Raw parameters:
/// a coedge riding a periodic seam carries values just past the domain, and
/// those are exactly what the next coedge has to continue from.
fn pcurve_ends(coedge: &CoedgeRecord) -> Option<([f64; 2], [f64; 2])> {
    let [q0, q1] = coedge.pcurve.domain().ok()?;
    let start = coedge.pcurve.evaluate(q0).ok()?;
    let end = coedge.pcurve.evaluate(q1).ok()?;
    Some(([start.x, start.y], [end.x, end.y]))
}

/// Cut `loop_record` at every consumed coedge and return the runs that survive,
/// each in the loop's own traversal order.
///
/// The runs are read off the CYCLIC loop order and never re-linked by vertex
/// adjacency: a loop may legitimately visit one vertex twice, and re-linking
/// would scramble it.
fn loop_fragments(
    face: usize,
    loop_record: &LoopRecord,
    consumed: &[bool],
    edges: &HashMap<u64, &EdgeRecord>,
) -> Option<Vec<Fragment>> {
    let count = loop_record.coedges.len();
    let first_cut = consumed.iter().position(|flag| *flag)?;
    let mut fragments = Vec::new();
    let mut run: Vec<CoedgeRecord> = Vec::new();
    for step in 1..=count {
        let index = (first_cut + step) % count;
        if consumed[index] {
            if !run.is_empty() {
                fragments.push(fragment(face, std::mem::take(&mut run), edges)?);
            }
            continue;
        }
        run.push(loop_record.coedges[index].clone());
    }
    // The walk ends ON `first_cut`, which flushes the last run.
    debug_assert!(run.is_empty());
    Some(fragments)
}

fn fragment(
    face: usize,
    coedges: Vec<CoedgeRecord>,
    edges: &HashMap<u64, &EdgeRecord>,
) -> Option<Fragment> {
    let start = coedge_ends(coedges.first()?, edges)?.0;
    let end = coedge_ends(coedges.last()?, edges)?.1;
    Some(Fragment {
        face,
        coedges,
        start,
        end,
    })
}

/// Splice the cut loops' surviving runs back into closed loops, and group the
/// faces they came from into the faces they become.
///
/// A run stops where a consumed edge took over; the loop closes again through
/// whichever run STARTS at that vertex. Everything here answers "not a cap"
/// (`None`, so the caller's own lane gets the question) rather than refusing,
/// because each way the reading can fail is a different shape:
///
/// * no run starts at a run's end — the free boundary leaves a genuine gap
///   between neighbours, which is the transition strip the chain re-intersects;
/// * two runs start there, or one run would have to serve two loops — the
///   survivor stays pinched, and this operation has no representation for it;
/// * the faces a rebuilt loop draws from do not share ONE carrier — a coedge
///   moves with the pcurve it was fitted to, and that pcurve means nothing on
///   another surface;
/// * a rebuilt loop does not close in PARAMETER space. Matching vertex ids are
///   not enough: a periodic wall's rim closes on one vertex while its uv ends
///   sit a full period apart, and splicing there would hand back a loop that
///   only looks closed.
fn splice_fragments(
    solid: &BrepSolid,
    shell_index: usize,
    fragments: &[Fragment],
    patch_edges: &HashSet<u64>,
) -> Option<(Vec<FaceMerge>, Vec<EdgeRecord>)> {
    if fragments.is_empty() {
        return Some((Vec::new(), Vec::new()));
    }
    let mut by_start: HashMap<u64, usize> = HashMap::default();
    for (index, fragment) in fragments.iter().enumerate() {
        if by_start.insert(fragment.start, index).is_some() {
            return None;
        }
    }

    // --- Walk the runs into closed circuits --------------------------------
    let mut used = vec![false; fragments.len()];
    let mut circuits: Vec<Vec<usize>> = Vec::new();
    for seed in 0..fragments.len() {
        if used[seed] {
            continue;
        }
        used[seed] = true;
        let mut circuit = vec![seed];
        let mut end = fragments[seed].end;
        while end != fragments[seed].start {
            let next = *by_start.get(&end)?;
            if used[next] {
                return None;
            }
            used[next] = true;
            circuit.push(next);
            end = fragments[next].end;
        }
        circuits.push(circuit);
    }

    let scale = solid_model_scale(solid);
    let carriers = carrier_classes(solid, shell_index, fragments, scale)?;
    let corners = circuit_corners(fragments, &circuits, &carriers)?;
    let (rebuilt, bridges) = if corners.is_empty() {
        // Every circuit stays on one carrier all the way round: the pinch
        // rejoin, which closes on the topology it already has.
        (
            circuits
                .iter()
                .map(|circuit| {
                    let mut coedges = Vec::new();
                    let mut faces = Vec::new();
                    for index in circuit {
                        coedges.extend(fragments[*index].coedges.iter().cloned());
                        faces.push(fragments[*index].face);
                    }
                    faces.sort_unstable();
                    faces.dedup();
                    (coedges, faces)
                })
                .collect(),
            Vec::new(),
        )
    } else {
        rejoin_across_corners(solid, fragments, &circuits, &carriers, &corners)?
    };

    // --- Faces that share a rebuilt loop become ONE face -------------------
    let mut parent: HashMap<usize, usize> = HashMap::default();
    for (_, faces) in &rebuilt {
        for face in faces {
            parent.entry(*face).or_insert(*face);
        }
    }
    for (_, faces) in &rebuilt {
        let mut members = faces.iter();
        let anchor = root(&mut parent, *members.next()?);
        for face in members {
            let other = root(&mut parent, *face);
            if other != anchor {
                parent.insert(other, anchor);
            }
        }
    }
    let mut grouped: HashMap<usize, Vec<usize>> = HashMap::default();
    for face in parent.keys().copied().collect::<Vec<usize>>() {
        let group = root(&mut parent, face);
        grouped.entry(group).or_default().push(face);
    }

    // The carriers have to be the SAME patch, not merely the same plane: a
    // pcurve is fitted to one parametrization. Faces split apart by the
    // arrangement carry the identical surface, so the band is a numeric
    // formality, not a fit tolerance. A group the BRIDGES rejoined is the one
    // exception — its faces are cosurface twins whose patches were fitted
    // apart — and it pays for that by re-fitting every pcurve onto one grown
    // carrier below.
    let carrier_tolerance = (scale * 1e-9).max(1e-12);
    // Past every id the solid has AND every id the bridges just took.
    let mut next_loop_id = rebuilt
        .iter()
        .flat_map(|(coedges, _)| coedges.iter().map(|coedge| coedge.id))
        .chain(bridges.iter().map(|bridge| bridge.id))
        .fold(max_topology_id(solid), u64::max)
        + 1;
    let mut edges: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    for bridge in &bridges {
        edges.insert(bridge.id, bridge.clone());
    }
    let mut groups: Vec<usize> = grouped.keys().copied().collect();
    groups.sort_unstable();
    let mut merges = Vec::with_capacity(groups.len());
    for group in groups {
        let mut faces = grouped[&group].clone();
        faces.sort_unstable();
        let host = &solid.shells[shell_index].faces[faces[0]];
        for other in faces.iter().skip(1) {
            let face = &solid.shells[shell_index].faces[*other];
            let one_patch = face.same_sense == host.same_sense
                && crate::face_merge::same_surface(
                    &face.surface,
                    &host.surface,
                    carrier_tolerance,
                );
            // Without a bridge the group has to be ONE patch: the pinch rejoin
            // moves loops between its faces with nothing refit, and a pcurve
            // means nothing on another parametrization. A BRIDGED group is the
            // exception, and it pays for it below.
            let rejoinable =
                one_patch || (!bridges.is_empty() && carriers[other] == carriers[&faces[0]]);
            if !rejoinable {
                return None;
            }
        }
        let mut loops: Vec<LoopRecord> = Vec::new();
        let mut bridged = false;
        for (coedges, members) in &rebuilt {
            if root(&mut parent, members[0]) != group {
                continue;
            }
            bridged |= coedges
                .iter()
                .any(|coedge| bridges.iter().any(|bridge| bridge.id == coedge.edge_id));
            loops.push(LoopRecord {
                id: next_loop_id,
                coedges: coedges.clone(),
            });
            next_loop_id += 1;
        }
        // Every loop the free boundary never touched comes along to the host.
        for face_position in &faces {
            let face = &solid.shells[shell_index].faces[*face_position];
            for loop_record in &face.loops {
                if loop_record
                    .coedges
                    .iter()
                    .any(|coedge| patch_edges.contains(&coedge.edge_id))
                {
                    continue;
                }
                loops.push(loop_record.clone());
            }
        }
        // A rejoined group rides ONE carrier grown over everything it now
        // holds, with every pcurve re-fitted to it; a pinch rejoin keeps the
        // host's patch and every pcurve bit-identical.
        let surface = if bridged {
            let mut candidate = FaceRecord {
                id: host.id,
                surface: host.surface.clone(),
                same_sense: host.same_sense,
                loops: loops.clone(),
                name: host.name.clone(),
            };
            regrow_and_refit_carrier(&mut candidate, &edges, scale, "delete_faces_and_heal").ok()?;
            // The pcurve fit interpolates whatever it is handed and reports no
            // error for a curve that MISSES the carrier, so this is where a
            // bridge laid on the wrong line is caught — before it becomes a
            // face whose new boundary is not on its own surface.
            let source_carriers: HashMap<u64, &NurbsSurface> = faces
                .iter()
                .flat_map(|position| {
                    let face = &solid.shells[shell_index].faces[*position];
                    face.loops.iter().flat_map(move |loop_record| {
                        loop_record
                            .coedges
                            .iter()
                            .map(move |coedge| (coedge.id, &face.surface))
                    })
                })
                .collect();
            if !loops_ride_carrier(
                &candidate.surface,
                &candidate.loops,
                &edges,
                &source_carriers,
                scale,
            ) {
                return None;
            }
            loops = candidate.loops;
            Some(candidate.surface)
        } else {
            None
        };
        let carrier = surface.as_ref().unwrap_or(&host.surface);
        for loop_record in &loops {
            if !closes_in_parameter_space(carrier, &loop_record.coedges) {
                return None;
            }
        }
        // `validate` reads loop 0 as the face's OUTER loop, so the widest leads.
        let outer = widest_loop_on(carrier, host.same_sense, host.id, &loops)?;
        loops.swap(0, outer);
        merges.push(FaceMerge {
            faces,
            loops,
            surface,
        });
    }
    Some((merges, bridges))
}


// ---------------------------------------------------------------------------
// The BRIDGED rejoin — a free boundary that walks off one carrier and back on
// to another (see the module docs).
// ---------------------------------------------------------------------------

/// Where the free boundary's circuit walks off one carrier and on to the next:
/// the vertex it turns at, and the survivor–survivor edge both sides ride into
/// it.
///
/// That edge is the whole point. A corner is not an arbitrary meeting of two
/// carriers, it is the far end of an edge the deleted band INTERRUPTED — the
/// arriving run's last coedge and the leaving run's first are the same edge
/// walked once each way, because at a corner the circuit doubles back on the
/// one edge the two survivors still share. The bridge is that edge's own line,
/// continued across the band to the corner where it resumes.
struct Corner {
    /// Fragment arriving at the corner, and the one leaving it.
    incoming: usize,
    outgoing: usize,
    vertex: u64,
    /// The edge both flanking coedges ride.
    spine: u64,
}

/// Whether every new coedge lies on `surface`, and each retained coedge is
/// at least as close to it as to its original carrier.
///
/// The rejoin's whole claim is that one grown carrier holds everything the
/// group now carries: its twin's loops, and the bridges between them. Nothing
/// upstream tests that claim — `faces_are_cosurface` matches carriers, not the
/// EXTENT a patch was grown to, and the pcurve fit interpolates a curve that
/// misses the surface as readily as one that lies on it. Imported trims can
/// already stand off their carrier. Rejoining preserves those edges verbatim;
/// require their pointwise residual not to increase rather than rejecting a
/// valid bridge because of an unrelated, inherited trim discrepancy. New
/// bridge coedges have no source carrier and must meet the absolute tolerance.
fn loops_ride_carrier(
    surface: &NurbsSurface,
    loops: &[LoopRecord],
    edges: &HashMap<u64, EdgeRecord>,
    source_carriers: &HashMap<u64, &NurbsSurface>,
    scale: f64,
) -> bool {
    let tolerance = (scale * 1e-7).max(1e-9);
    for coedge in loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        let Some(edge) = edges.get(&coedge.edge_id) else {
            return false;
        };
        if edge.degenerate {
            continue;
        }
        for step in 0..=8 {
            let t = edge.t0 + (edge.t1 - edge.t0) * step as f64 / 8.0;
            let Ok(point) = edge.curve.evaluate(t) else {
                return false;
            };
            let Ok(projection) = crate::project_point_to_surface(surface, point) else {
                return false;
            };
            if projection.distance <= tolerance {
                continue;
            }
            let Some(source) = source_carriers.get(&coedge.id) else {
                return false;
            };
            match crate::project_point_to_surface(source, point) {
                Ok(previous) if projection.distance <= previous.distance + tolerance => {}
                _ => return false,
            }
        }
    }
    true
}

/// Which carrier each fragment's face rides, as a class index — cosurface
/// GEOMETRICALLY (`face_merge::faces_are_cosurface`), because the far side of
/// a band is routinely a mirrored twin: the same surface with reflected
/// control points and a flipped `same_sense`, which bit-identity reads as two.
fn carrier_classes(
    solid: &BrepSolid,
    shell_index: usize,
    fragments: &[Fragment],
    scale: f64,
) -> Option<HashMap<usize, usize>> {
    let tolerance = (scale * 1e-9).max(1e-12);
    let mut classes: HashMap<usize, usize> = HashMap::default();
    let mut representatives: Vec<usize> = Vec::new();
    for fragment in fragments {
        if classes.contains_key(&fragment.face) {
            continue;
        }
        let face = &solid.shells[shell_index].faces[fragment.face];
        let mut found = None;
        for (index, representative) in representatives.iter().enumerate() {
            let other = &solid.shells[shell_index].faces[*representative];
            // A carrier this test cannot read is not thereby the same one:
            // an Err means "cannot say", which for a CLASS is a `false`. It
            // must not abort the classification, or a pinch rejoin that the
            // bit-identical half already answers would be lost to it.
            if crate::face_merge::faces_are_cosurface(other, face, tolerance).unwrap_or(false) {
                found = Some(index);
                break;
            }
        }
        let class = found.unwrap_or_else(|| {
            representatives.push(fragment.face);
            representatives.len() - 1
        });
        classes.insert(fragment.face, class);
    }
    Some(classes)
}

/// Every handover in `circuits` where the circuit changes carrier.
///
/// `None` — not an empty list — when a handover changes carrier WITHOUT the
/// two runs sharing an edge there. That is a genuine gap between neighbours
/// rather than an interrupted edge, which is what re-intersection is for, so
/// the whole classification declines and the caller's chain takes it.
fn circuit_corners(
    fragments: &[Fragment],
    circuits: &[Vec<usize>],
    carriers: &HashMap<usize, usize>,
) -> Option<Vec<Corner>> {
    let mut corners = Vec::new();
    for circuit in circuits {
        for (position, index) in circuit.iter().copied().enumerate() {
            let next = circuit[(position + 1) % circuit.len()];
            if next == index || carriers[&fragments[index].face] == carriers[&fragments[next].face]
            {
                continue;
            }
            let arriving = fragments[index].coedges.last()?.edge_id;
            let leaving = fragments[next].coedges.first()?.edge_id;
            if arriving != leaving {
                return None;
            }
            corners.push(Corner {
                incoming: index,
                outgoing: next,
                vertex: fragments[index].end,
                spine: arriving,
            });
        }
    }
    Some(corners)
}

/// Cut the circuits at their corners, pair the corners off, lay a bridge along
/// each pair's shared line, and walk the pieces back into one closed loop per
/// carrier.
///
/// Two corners pair when the circuit leaves carrier A for carrier B at one and
/// comes back at the other, AND their spine edges are one straight line that
/// the bridge continues — each spine running AWAY from its own corner, so the
/// bridge covers exactly the piece the deleted band took and no more. Anything
/// else declines: a transition strip's four corners have four different carrier
/// pairs and no partner between them, which is how a strip keeps going to the
/// chain that re-intersects it.
fn rejoin_across_corners(
    solid: &BrepSolid,
    fragments: &[Fragment],
    circuits: &[Vec<usize>],
    carriers: &HashMap<usize, usize>,
    corners: &[Corner],
) -> Option<(Vec<(Vec<CoedgeRecord>, Vec<usize>)>, Vec<EdgeRecord>)> {
    // --- Pair the corners --------------------------------------------------
    let signature = |corner: &Corner| {
        (
            carriers[&fragments[corner.incoming].face],
            carriers[&fragments[corner.outgoing].face],
        )
    };
    let mut partner = vec![usize::MAX; corners.len()];
    for (index, corner) in corners.iter().enumerate() {
        let (arriving, leaving) = signature(corner);
        let mut matches = corners
            .iter()
            .enumerate()
            .filter(|(other, candidate)| *other != index && signature(candidate) == (leaving, arriving));
        let (found, _) = matches.next()?;
        if matches.next().is_some() {
            return None;
        }
        partner[index] = found;
    }
    // The pairing has to be an involution, or the loops below cannot close.
    if (0..corners.len()).any(|index| partner[partner[index]] != index) {
        return None;
    }

    // --- Lay one bridge per pair -------------------------------------------
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let mut next_id = max_topology_id(solid) + 1;
    let mut bridges: Vec<EdgeRecord> = Vec::new();
    // corner -> (bridge edge, whether leaving this corner runs with the edge)
    let mut bridge_at: HashMap<usize, (u64, bool)> = HashMap::default();
    for index in 0..corners.len() {
        if bridge_at.contains_key(&index) {
            continue;
        }
        let other = partner[index];
        let bridge = bridge_edge(solid, &corners[index], &corners[other], next_id, tolerance)?;
        next_id += 1;
        bridge_at.insert(index, (bridge.id, true));
        bridge_at.insert(other, (bridge.id, false));
        bridges.push(bridge);
    }

    // --- Cut the circuits into arcs ----------------------------------------
    // An arc is a maximal run of fragments on one carrier, from the corner
    // that starts it to the corner that ends it.
    let ends_at: HashMap<usize, usize> = corners
        .iter()
        .enumerate()
        .map(|(index, corner)| (corner.incoming, index))
        .collect();
    let mut arcs: Vec<(usize, Vec<usize>, usize)> = Vec::new();
    for circuit in circuits {
        let length = circuit.len();
        let Some(first) = (0..length).find(|position| ends_at.contains_key(&circuit[*position]))
        else {
            // A circuit with no corner never left its carrier: it is a whole
            // rejoined loop already, and mixing it with the arcs would lose it.
            return None;
        };
        let mut open: Vec<usize> = Vec::new();
        let mut start = ends_at[&circuit[first]];
        for step in 1..=length {
            let index = circuit[(first + step) % length];
            open.push(index);
            if let Some(corner) = ends_at.get(&index) {
                arcs.push((start, std::mem::take(&mut open), *corner));
                start = *corner;
            }
        }
        if !open.is_empty() {
            return None;
        }
    }
    let starts_at: HashMap<usize, usize> = arcs
        .iter()
        .enumerate()
        .map(|(index, arc)| (arc.0, index))
        .collect();
    if starts_at.len() != arcs.len() {
        return None;
    }

    // --- Walk arc → bridge → arc into one loop per opening -----------------
    let mut used = vec![false; arcs.len()];
    let mut rebuilt: Vec<(Vec<CoedgeRecord>, Vec<usize>)> = Vec::new();
    for seed in 0..arcs.len() {
        if used[seed] {
            continue;
        }
        used[seed] = true;
        let mut coedges: Vec<CoedgeRecord> = Vec::new();
        let mut faces: Vec<usize> = Vec::new();
        let mut current = seed;
        loop {
            for fragment in &arcs[current].1 {
                coedges.extend(fragments[*fragment].coedges.iter().cloned());
                faces.push(fragments[*fragment].face);
            }
            let (edge_id, forward) = bridge_at[&arcs[current].2];
            coedges.push(CoedgeRecord {
                id: next_id,
                edge_id,
                forward,
                // A placeholder: the group's whole loop is re-fitted onto one
                // grown carrier before it is ever read.
                pcurve: make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0)).ok()?,
            });
            next_id += 1;
            let next = *starts_at.get(&partner[arcs[current].2])?;
            if next == seed {
                break;
            }
            if used[next] {
                return None;
            }
            used[next] = true;
            current = next;
        }
        faces.sort_unstable();
        faces.dedup();
        rebuilt.push((coedges, faces));
    }
    Some((rebuilt, bridges))
}

/// The bridge between two paired corners: the straight piece of their shared
/// line that the deleted band took out of it.
///
/// Refuses (`None`) unless both spines are STRAIGHT, both lie on the line
/// through the two corner points, and each runs away from its own corner —
/// which together say the two spines are one interrupted edge and the bridge
/// is exactly its missing middle. A curved spine is deferred rather than
/// guessed at: continuing an arc needs its centre and sense, not two points.
fn bridge_edge(
    solid: &BrepSolid,
    from: &Corner,
    to: &Corner,
    id: u64,
    tolerance: f64,
) -> Option<EdgeRecord> {
    let start = edge_point(solid, from.vertex).ok()?;
    let end = edge_point(solid, to.vertex).ok()?;
    let span = end.sub(start);
    let length = span.length();
    if length <= tolerance {
        return None;
    }
    let direction = span.scale(1.0 / length);
    for (corner, anchor, sense) in [(from, start, -1.0f64), (to, end, 1.0f64)] {
        let spine = solid.edges.iter().find(|edge| edge.id == corner.spine)?;
        let mut reach = 0.0f64;
        for step in 0..=8 {
            let t = spine.t0 + (spine.t1 - spine.t0) * step as f64 / 8.0;
            let point = spine.curve.evaluate(t).ok()?;
            let offset = point.sub(anchor);
            let along = offset.dot(direction);
            if offset.sub(direction.scale(along)).length() > tolerance {
                return None;
            }
            if along * sense < -tolerance {
                // The spine reaches INTO the gap the bridge is to cover: these
                // two are not one interrupted edge.
                return None;
            }
            if along * sense > reach * sense {
                reach = along;
            }
        }
        if reach * sense <= tolerance {
            return None;
        }
    }
    let curve = make_line(start, end).ok()?;
    let [t0, t1] = curve.domain().ok()?;
    Some(EdgeRecord {
        id,
        curve,
        t0,
        t1,
        start_vertex_id: from.vertex,
        end_vertex_id: to.vertex,
        degenerate: false,
        name: None,
    })
}

/// Union-find root, over a set small enough that path compression would cost
/// more than it saves.
fn root(parent: &mut HashMap<usize, usize>, mut node: usize) -> usize {
    while let Some(next) = parent.get(&node).copied() {
        if next == node {
            break;
        }
        node = next;
    }
    node
}

/// Whether a rebuilt loop's coedges hand over to one another in the carrier's
/// PARAMETER space, all the way round.
fn closes_in_parameter_space(surface: &NurbsSurface, coedges: &[CoedgeRecord]) -> bool {
    let (u_span, v_span) = match parameter_spans(surface) {
        Some(spans) => spans,
        None => return false,
    };
    // Loose enough that an honest fit's round-off passes, far tighter than the
    // half-period a seam jump would show.
    let tolerance = (u_span + v_span) * 1e-6;
    let mut ends: Vec<([f64; 2], [f64; 2])> = Vec::with_capacity(coedges.len());
    for coedge in coedges {
        match pcurve_ends(coedge) {
            Some(pair) => ends.push(pair),
            None => return false,
        }
    }
    for index in 0..ends.len() {
        let leaving = ends[index].1;
        let arriving = ends[(index + 1) % ends.len()].0;
        let gap = (leaving[0] - arriving[0]).hypot(leaving[1] - arriving[1]);
        if gap > tolerance {
            return false;
        }
    }
    true
}

/// The `[u, v]` domain spans of a carrier.
fn parameter_spans(surface: &NurbsSurface) -> Option<(f64, f64)> {
    let u = crate::KnotVector::new(surface.knots_u.clone(), surface.degree_u)
        .ok()?
        .domain();
    let v = crate::KnotVector::new(surface.knots_v.clone(), surface.degree_v)
        .ok()?
        .domain();
    Some((u[1] - u[0], v[1] - v[0]))
}

/// Signed parameter-space area of ONE loop, read on `host`'s carrier.
fn single_loop_area(host: &FaceRecord, loop_record: &LoopRecord) -> Result<f64, KernelRefusal> {
    parameter_space_area(&FaceRecord {
        id: host.id,
        surface: host.surface.clone(),
        same_sense: host.same_sense,
        loops: vec![loop_record.clone()],
        name: None,
    })
    .or_refuse(KernelStage::Classify, "parameter_space_area")
}

/// The index of the loop that bounds the material — the widest in parameter
/// space, read on whichever carrier the loops now ride (a bridged rejoin fits
/// them to a GROWN one the host does not have yet).
fn widest_loop_on(
    surface: &NurbsSurface,
    same_sense: bool,
    id: u64,
    loops: &[LoopRecord],
) -> Option<usize> {
    let host = FaceRecord {
        id,
        surface: surface.clone(),
        same_sense,
        loops: Vec::new(),
        name: None,
    };
    let mut widest = (0usize, f64::NEG_INFINITY);
    for (index, loop_record) in loops.iter().enumerate() {
        let area = single_loop_area(&host, loop_record).ok()?.abs();
        if area > widest.1 {
            widest = (index, area);
        }
    }
    (widest.1 > f64::NEG_INFINITY).then_some(widest.0)
}

/// `V - E + F - H` on the reduced complex `validate()`'s Euler check uses:
/// degenerate (pole) edges and the vertices only they reference are not
/// independent cells, and each loop past a face's first is a hole.
pub(super) fn euler_characteristic(solid: &BrepSolid) -> i64 {
    let referenced: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate)
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let vertices = solid
        .vertices
        .iter()
        .filter(|vertex| referenced.contains(&vertex.id))
        .count() as i64;
    let edges = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let faces = solid
        .shells
        .iter()
        .map(|shell| shell.faces.len())
        .sum::<usize>() as i64;
    let holes = solid.bounding_hole_count();
    vertices - edges + faces - holes
}

/// How a face is named in a refusal: its persistent name when it has one, its
/// id otherwise.
fn face_label(face: &FaceRecord) -> String {
    match &face.name {
        Some(name) => format!("`{name}`"),
        None => format!("face {}", face.id),
    }
}

/// Group the wholly-consumed loops by the face they belong to, because "would
/// this face keep a loop" is a question about the face, not about one loop.
fn dropped_per_face(patch: &FacePatch) -> HashMap<(usize, usize), Vec<usize>> {
    let mut per_face: HashMap<(usize, usize), Vec<usize>> = HashMap::default();
    for (shell_position, face_position, loop_index) in &patch.dropped_loops {
        per_face
            .entry((*shell_position, *face_position))
            .or_default()
            .push(*loop_index);
    }
    per_face
}

/// The conditions that make a classified patch a CAP rather than a gap this
/// operation cannot close.
///
/// Separate from [`cap_face_patch`] because the two callers need the same
/// question answered in different registers. A SET selection has already said
/// "this is one patch", so a failure here is a refusal that names what is
/// wrong. A ONE-FACE selection has said nothing of the kind, and the same
/// failure only means "not a cap": a blind bore's wall and a dome boss's base
/// fillet wear the same shape here, and both belong to
/// [`delete_face_and_heal`](super::delete_face_and_heal) — one for the refusal
/// that names the floor, the other because plane x sphere re-intersects and
/// heals.
fn cap_preconditions(solid: &BrepSolid, patch: &FacePatch, op: &str) -> Result<(), KernelRefusal> {
    // --- Every consumed loop must be a HOLE in its face --------------------
    for ((shell_position, face_position), loop_indices) in &dropped_per_face(patch) {
        let face = &solid.shells[*shell_position].faces[*face_position];
        let rebuilt = *shell_position == patch.shell_index
            && patch
                .merges
                .iter()
                .any(|merge| merge.faces.contains(face_position));
        if !rebuilt && loop_indices.len() >= face.loops.len() {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "whole_boundary",
                format!(
                    "{op}: the selection is the whole boundary of {} — it is part of the \
                     pocket, not the face the pocket was sunk into. Select it as well \
                     (a patch is capped by the face AROUND it, which has to keep a loop).",
                    face_label(face)
                ),
            ));
        }
        if face.loops.len() < 2 {
            // A face whose only loop is consumed is caught above; a rebuilt
            // face's remaining loops are the splice's business, not a winding
            // question about the loops it no longer has.
            continue;
        }
        let mut areas = Vec::with_capacity(face.loops.len());
        for index in 0..face.loops.len() {
            areas.push(loop_signed_area(face, index)?);
        }
        let host = (0..areas.len())
            .max_by(|a, b| areas[*a].abs().total_cmp(&areas[*b].abs()))
            .expect("the face has at least two loops here");
        for loop_index in loop_indices {
            if *loop_index == host || areas[host] * areas[*loop_index] >= 0.0 {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "outer_loop_consumed",
                    format!(
                        "{op}: the loop the selection would leave open in {} bounds that \
                         face's material rather than a hole in it — capping it would erase \
                         the face (deferred)",
                        face_label(face)
                    ),
                ));
            }
        }
    }

    // --- A rejoined face must have GROWN by the region it takes in ---------
    // The spliced analogue of the winding test above: the free boundary
    // enclosed a region of the shared carrier, and capping fills that region
    // in. Parameter-space areas over one carrier are additive, so a loop that
    // wound the other way shows up here as a merged face that would have
    // swallowed its own material instead of the opening.
    for merge in &patch.merges {
        let host = &solid.shells[patch.shell_index].faces[merge.faces[0]];
        if let Some(surface) = &merge.surface {
            // The group was rejoined across bridges, so its loops ride a GROWN
            // carrier and its members rode twins of it: parameter-space areas
            // are no longer one currency and the same question is asked in
            // mm². The rejoined face has to take in the openings the bridges
            // closed, so it is strictly larger than the faces it replaces, and
            // it has to keep the host's winding (a loop that came back the
            // other way would enclose the complement).
            let rejoined = FaceRecord {
                id: host.id,
                surface: surface.clone(),
                same_sense: host.same_sense,
                loops: merge.loops.clone(),
                name: None,
            };
            let mut before = 0.0;
            for face_position in &merge.faces {
                before += crate::face_area(&solid.shells[patch.shell_index].faces[*face_position])
                    .or_refuse(KernelStage::Classify, "face_area")?;
            }
            let after = crate::face_area(&rejoined).or_refuse(KernelStage::Classify, "face_area")?;
            let winding = parameter_space_area(&rejoined)
                .or_refuse(KernelStage::Classify, "parameter_space_area")?;
            let host_winding = loop_signed_area(host, 0)?;
            if after <= before * (1.0 + 1e-9) || winding * host_winding <= 0.0 {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "rejoin_shrinks_grown",
                    format!(
                        "{op}: rejoining {} across the openings its free boundary leaves \
                         would not take in the region they enclose (area {before} -> {after}, \
                         winding {host_winding} -> {winding}) — that boundary bounds material \
                         rather than an opening (deferred)",
                        face_label(host)
                    ),
                ));
            }
            continue;
        }
        let mut before = 0.0;
        for face_position in &merge.faces {
            let face = &solid.shells[patch.shell_index].faces[*face_position];
            for loop_record in &face.loops {
                before += single_loop_area(host, loop_record)?;
            }
        }
        let mut after = 0.0;
        for loop_record in &merge.loops {
            after += single_loop_area(host, loop_record)?;
        }
        if (after - before) * before.signum() <= before.abs() * 1e-9 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "rejoin_shrinks",
                format!(
                    "{op}: rejoining {} would not take in the region its free boundary \
                     encloses (parameter-space area {before} -> {after}) — that boundary \
                     bounds material rather than an opening (deferred)",
                    face_label(host)
                ),
            ));
        }
    }
    Ok(())
}

/// Lift `patch` off the solid and close the opening it was sunk through: drop
/// the hole loops it consumed whole, and rejoin the faces whose loops it only
/// cut.
fn cap_face_patch(solid: &BrepSolid, patch: &FacePatch, op: &str) -> Result<BrepSolid, KernelRefusal> {
    if !patch.bridges.is_empty() {
        if let Some(directory) = census_directory() {
            let mut face_ids: Vec<u64> = patch.face_ids.iter().copied().collect();
            face_ids.sort_unstable();
            let rejoined: Vec<u64> = patch
                .merges
                .iter()
                .flat_map(|merge| &merge.faces)
                .map(|position| solid.shells[patch.shell_index].faces[*position].id)
                .collect();
            // The census reads its operation as text (`closed_heal_census.rs`
            // takes a `-> Result<BrepSolid, String>`), so the class is dropped
            // here and here only, on the diagnostic copy.
            let operation = |body: &BrepSolid| -> Result<BrepSolid, String> {
                let patch = classify_patch(body, &face_ids).ok_or("the copy is no longer a patch")?;
                cap_face_patch(body, &patch, op).map_err(String::from)
            };
            record_rejoin_census(&directory, solid, &face_ids, &rejoined, &operation);
        }
    }
    cap_preconditions(solid, patch, op)?;
    let per_face = dropped_per_face(patch);

    let mut healed = solid.clone();

    // --- Rejoin first, by the positions resolved against THIS topology -----
    // A merge writes the host's whole loop list, so it subsumes any dropped
    // loop of a face in its group; the drop below skips those faces.
    let mut merged_away: HashSet<u64> = HashSet::default();
    for merge in &patch.merges {
        let faces = &mut healed.shells[patch.shell_index].faces;
        faces[merge.faces[0]].loops = merge.loops.clone();
        if let Some(surface) = &merge.surface {
            faces[merge.faces[0]].surface = surface.clone();
        }
        for face_position in merge.faces.iter().skip(1) {
            merged_away.insert(faces[*face_position].id);
        }
    }

    // --- Drop the hole loops, then the patch, then their edges -------------
    // Loops go by descending index within each face, so no removal disturbs a
    // position resolved against the original topology; dropping a loop cannot
    // move a face, and the faces go by id.
    for ((shell_position, face_position), loop_indices) in &per_face {
        if *shell_position == patch.shell_index
            && patch
                .merges
                .iter()
                .any(|merge| merge.faces.contains(face_position))
        {
            continue;
        }
        let mut loop_indices = loop_indices.clone();
        loop_indices.sort_unstable_by(|a, b| b.cmp(a));
        for loop_index in loop_indices {
            healed.shells[*shell_position].faces[*face_position]
                .loops
                .remove(loop_index);
        }
    }
    for shell in &mut healed.shells {
        shell
            .faces
            .retain(|face| !patch.face_ids.contains(&face.id) && !merged_away.contains(&face.id));
    }
    healed.edges.retain(|edge| !patch.edges.contains(&edge.id));
    healed.edges.extend(patch.bridges.iter().cloned());
    let used: HashSet<u64> = healed
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    healed.vertices.retain(|vertex| used.contains(&vertex.id));

    // --- The two things `validate()` will not ask for us -------------------
    if !faces_are_connected(&healed.shells[patch.shell_index].faces) {
        return Err(KernelRefusal::unsupported(
            KernelStage::Validate,
            "severs_shell",
            format!(
                "{op}: the selected faces are what joins two otherwise separate parts of \
                 the body — removing them would sever the solid, which this operation \
                 cannot represent (deferred)"
            ),
        ));
    }
    // Genus is not assumed either way: capping a blind pocket leaves it alone,
    // capping a through feature drops it by one, and both fall out of the same
    // Euler count over the reduced complex.
    let shift = euler_characteristic(solid) - euler_characteristic(&healed);
    if shift % 2 != 0 {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "odd_euler_shift",
            format!(
                "{op}: the selection does not close into whole handles \
                 (Euler characteristic shifts by an odd {shift}) — refusing rather than \
                 emitting a solid whose genus is a guess"
            ),
        ));
    }
    healed.genus += shift / 2;
    if healed.genus < 0 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Validate,
            "negative_genus",
            format!(
                "{op}: capping the selection leaves genus {}, so the solid's stated genus \
                 did not account for the feature it carries (deferred)",
                healed.genus
            ),
        ));
    }

    let issues = healed.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!("{op}: the capped solid failed validation: {issues:?}"),
        ));
    }
    Ok(healed)
}

/// Delete ONE face: the cap when the face's own shape says it caps,
/// [`delete_face_and_heal`](super::delete_face_and_heal)'s re-intersection
/// otherwise.
///
/// The cap gate is consulted FIRST and in full — see [`cap_preconditions`] for
/// why the failures have to read as "not a cap" here rather than as refusals.
fn delete_one_face(solid: &BrepSolid, face_id: u64, op: &str) -> Result<BrepSolid, KernelRefusal> {
    if let Some(patch) = classify_patch(solid, &[face_id]) {
        if cap_preconditions(solid, &patch, op).is_ok() {
            return cap_face_patch(solid, &patch, op);
        }
    }
    // A closed strip whose rims are RUNS of edges rather than one closed edge
    // each: `delete_face_and_heal_impl` reads only `[seam+, rim_a, seam-,
    // rim_b]` and refuses anything else on its coedge count, but the band lane
    // is stated for a run and heals it identically (`closed_band.rs`). The gate
    // declines every shape the closed strip CAN read, so that lane keeps every
    // case it already answered — and this is asked after the patch gate, so a
    // through feature's wall still caps rather than being sharpened against
    // two survivors that were never meant to meet.
    if let Some(band) = classify_closed_band(solid, &[face_id]) {
        return heal_closed_band(solid, &band, op);
    }
    delete_face_and_heal_impl(solid, face_id)
}

/// The selection's edge-connected COMPONENTS, in the order their faces appear
/// in `face_ids` (both between components and within one), so a chained
/// component heals in exactly the order the caller listed it.
///
/// Two selected faces belong to the same component when they SHARE AN EDGE.
/// That is the relation the patch gate is a question about: a patch's free
/// boundary is a property of one contiguous piece of surface, and two pieces
/// that touch nothing of each other cannot make one another's boundary any
/// less whole.
pub(super) fn connected_components(solid: &BrepSolid, face_ids: &[u64]) -> Vec<Vec<u64>> {
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let mut adjacency: HashMap<u64, Vec<u64>> = HashMap::default();
    let mut by_edge: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if !selected.contains(&face.id) {
            continue;
        }
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            by_edge.entry(coedge.edge_id).or_default().push(face.id);
        }
    }
    for sharing in by_edge.values() {
        for a in sharing {
            for b in sharing {
                if a != b {
                    adjacency.entry(*a).or_default().push(*b);
                }
            }
        }
    }

    let position: HashMap<u64, usize> = face_ids
        .iter()
        .enumerate()
        .map(|(index, face_id)| (*face_id, index))
        .collect();
    let mut seen: HashSet<u64> = HashSet::default();
    let mut components: Vec<Vec<u64>> = Vec::new();
    for start in face_ids {
        if !seen.insert(*start) {
            continue;
        }
        let mut component = vec![*start];
        let mut stack = vec![*start];
        while let Some(face_id) = stack.pop() {
            for next in adjacency.get(&face_id).into_iter().flatten() {
                if seen.insert(*next) {
                    component.push(*next);
                    stack.push(*next);
                }
            }
        }
        component.sort_unstable_by_key(|face_id| position[face_id]);
        components.push(component);
    }
    components
}

/// Delete a SET of faces and heal, in one operation.
///
/// Two lanes, chosen by the selection's own shape (see the module docs):
///
/// * a PATCH — a set whose free boundary consumes whole hole loops of the
///   faces around it (a pocket's walls + floor, a boss's wall + top, a bore's
///   wall) — is CAPPED: the patch and those loops go, and nothing else is
///   touched;
/// * anything else is healed one face at a time by
///   [`delete_face_and_heal`](super::delete_face_and_heal), which extends and
///   re-intersects each face's neighbours. Face ids are stable across a heal,
///   so the chain re-uses the ids it was given.
///
/// A single-face selection is put to the SAME gate, in full, and falls through
/// to the chain when it does not hold — see [`delete_one_face`]. A lone face
/// can be a patch on its own: a bore's wall is one, and so is a bore whose
/// mouth is tangent to something, where the rim arrives as runs of two
/// pinch-split faces' loops instead of as a hole loop.
///
/// A contiguous selection that is neither a patch nor a corner but a CLOSED
/// BAND — an annulus whose free boundary is one closed run on each of two
/// survivors, like a nut pocket sunk over a bore that keeps going — is healed by
/// re-intersecting those two survivors (`closed_band.rs`), the same answer the
/// one-face closed strip gets.
///
/// A selection can be BOTH — holes and fillets picked together — and then it is
/// neither lane as a whole. Such a selection is split into edge-connected
/// components and each component takes the lane its own shape asks for; the
/// caps run before the chain (see the module docs). A selection that IS one
/// patch never reaches that split, so every set the gate already accepted is
/// answered by exactly the code it was answered by before.
pub fn delete_faces_and_heal(solid: &BrepSolid, face_ids: &[u64]) -> Result<BrepSolid, KernelRefusal> {
    // The soundness floor, once, on the finished body. `validate()` is an
    // incidence test and says nothing about a shell passing through itself;
    // a re-intersected rim is exactly what can produce one, and a fold is
    // wrong in AREA where it is right in volume, so nothing that measures
    // volume would notice. See `healing::accept_sound`.
    crate::accept_sound(delete_faces_and_heal_impl(solid, face_ids)?, "deleteFace")
}

/// The whole selection's heal, without the soundness floor: see
/// [`delete_faces_and_heal`].
fn delete_faces_and_heal_impl(solid: &BrepSolid, face_ids: &[u64]) -> Result<BrepSolid, KernelRefusal> {
    let op = "delete_faces_and_heal";
    let mut seen: HashSet<u64> = HashSet::default();
    let face_ids: Vec<u64> = face_ids
        .iter()
        .copied()
        .filter(|face_id| seen.insert(*face_id))
        .collect();
    if face_ids.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "selection",
            format!("{op}: no faces selected"),
        ));
    }
    for face_id in &face_ids {
        if find_face(solid, *face_id).is_none() {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "face_id",
                format!("{op}: no face with id {face_id}"),
            ));
        }
    }
    // A strip a fillet capped takes its planar end caps with it, picked or not
    // (`blend_network_heal.rs`), so every lane below is asked about the strip
    // and its caps together.
    let face_ids =
        with_owned_caps(solid, &face_ids, op)?;
    if face_ids.len() == 1 {
        // One face is still a network when caps left standing by a second
        // round sit at its ends — a first round's cylinder whose end arcs the
        // second round's strips ran up to (`blend_network_heal.rs`). Every
        // other face is the one-face chain's, exactly as before.
        return match read_blend_network(solid, &face_ids, op) {
            NetworkRead::Network(network) => Ok(coalesce_healed_edges(
                &heal_blend_network(solid, &network, op)
                    ?,
            )),
            // The gate's refusal is typed at its origin; it rides through.
            NetworkRead::Refused(reason) => Err(reason),
            NetworkRead::NotANetwork => {
                Ok(coalesce_healed_edges(&delete_one_face(solid, face_ids[0], op)?))
            }
        };
    }
    if let Some(patch) = classify_patch(solid, &face_ids) {
        return Ok(coalesce_healed_edges(&cap_face_patch(solid, &patch, op)?));
    }
    if let Some(group) = classify_corner_blend_group(solid, &face_ids) {
        return Ok(coalesce_healed_edges(
            &heal_corner_blend_group(solid, &group, op)?,
        ));
    }
    if let Some(band) = classify_closed_band(solid, &face_ids) {
        return Ok(coalesce_healed_edges(
            &heal_closed_band(solid, &band, op)?,
        ));
    }
    match read_blend_network(solid, &face_ids, op) {
        NetworkRead::Network(network) => {
            return Ok(coalesce_healed_edges(
                &heal_blend_network(solid, &network, op)
                    ?,
            ));
        }
        // A network with a face beside it that cannot stay is refused by name:
        // one face at a time has no better answer for a network.
        NetworkRead::Refused(reason) => {
            return Err(reason)
        }
        NetworkRead::NotANetwork => {}
    }

    // Not one patch. Ask each edge-connected component of the selection the
    // same question on its own, so a mixed selection is answered rather than
    // forced through whichever single lane the whole set happened to miss.
    let components = connected_components(solid, &face_ids);
    let mut healed = solid.clone();
    if components.len() < 2 {
        // A set that encloses every face it borders, and that no lane above
        // read, is refused by name: the patch gate declined it for having
        // nothing to cap against, and the chain would take it apart one face
        // at a time with every neighbour still going.
        if let Some(reason) = enclosed_selection_refusal(solid, &face_ids, op) {
            return Err(reason);
        }
        // One contiguous piece that is neither a patch nor a corner: the
        // chain, exactly as before, in the caller's own order.
        for face_id in &face_ids {
            healed = delete_one_face(&healed, *face_id, op)?;
        }
        return Ok(coalesce_healed_edges(&healed));
    }

    // Caps first — each classified against the RUNNING solid, because a patch
    // carries loop positions and those are only valid for the solid they were
    // resolved against. Capping one component cannot unmake another's patch:
    // it only drops faces and hole loops the other component does not border.
    let mut chained: Vec<Vec<u64>> = Vec::new();
    for component in components {
        if component.len() == 1 {
            // A lone face goes to `delete_one_face` — the same gate, and the
            // same fall-through, a one-face selection gets.
            chained.push(component);
            continue;
        }
        match classify_patch(&healed, &component) {
            Some(patch) => healed = cap_face_patch(&healed, &patch, op)?,
            None => chained.push(component),
        }
    }
    // A corner blend is a component the chain cannot take a face at a time
    // (`corner_heal.rs`), and like the chain it refits carriers, so it runs in
    // the same phase — after every cap, before the faces the chain still owns.
    for (index, component) in chained.iter().enumerate() {
        if component.len() > 1 {
            if let Some(group) = classify_corner_blend_group(&healed, component) {
                healed = heal_corner_blend_group(&healed, &group, op)
                    ?;
                continue;
            }
            if let Some(band) = classify_closed_band(&healed, component) {
                healed =
                    heal_closed_band(&healed, &band, op)?;
                continue;
            }
        }
        // A lone face with caps standing at its ends is a network, as it is when
        // it is the whole selection.
        match read_blend_network(&healed, component, op) {
            NetworkRead::Network(network) => {
                healed = heal_blend_network(&healed, &network, op)
                    ?;
                continue;
            }
            NetworkRead::Refused(reason) => {
                return Err(reason)
            }
            NetworkRead::NotANetwork => {}
        }
        // The same refusal, asked of everything still selected rather than of
        // this piece: twelve strips without their vertex blends are twelve
        // separate pieces, and each one alone borders survivors that the rest
        // of the set goes on to enclose.
        let remaining: Vec<u64> = chained[index..].iter().flatten().copied().collect();
        if let Some(reason) = enclosed_selection_refusal(&healed, &remaining, op) {
            return Err(reason);
        }
        for face_id in component {
            healed = delete_one_face(&healed, *face_id, op)?;
        }
    }
    Ok(coalesce_healed_edges(&healed))
}

/// Collapse the tangent continuation edges the heal leaves behind.
///
/// Deleting a face REJOINS survivors: a cap drops a hole loop, and a rejoin
/// merges cosurface twins and bridges the edge the band interrupted. Either
/// way two edges that were separate only because something stood between them
/// end up adjacent on the same two faces — three collinear pieces of one box
/// edge where the mirrored union and the deleted fillet band split it, for
/// instance. The boolean has closed exactly this since it started merging
/// coplanar faces (`csg::boolean`); the heal grew the same need when it grew
/// the rejoin, and did not get the same finish.
///
/// The pre-coalesce solid is a complete, valid answer on its own — the merge
/// only removes vertices the user never asked for — so anything short of a
/// clean merge keeps it rather than failing the delete.
/// `merge_curve_continuation_edges` is conservative by construction (it accepts
/// a pair only when both 3D curves and both p-curves rejoin within the pcurve
/// contract and neither face's parameter area moves) and it deliberately does
/// NOT validate, because the offset shell coalesces while its boundaries are
/// still open. A heal's result is closed, so validate it here: each merge drops
/// one vertex and one edge, which leaves the Euler characteristic — and so the
/// genus already stamped on the solid — exactly where it was.
fn coalesce_healed_edges(solid: &BrepSolid) -> BrepSolid {
    let tolerance = (solid_model_scale(solid) * 1e-7).max(1e-9);
    match crate::merge_curve_continuation_edges(solid, tolerance) {
        Ok(merged) if merged.validate().len() <= solid.validate().len() => merged,
        _ => solid.clone(),
    }
}

