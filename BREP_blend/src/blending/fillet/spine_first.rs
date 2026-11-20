//! The SPINE-FIRST composition (reported 2026-11-16): a convex edge — the spine — that dies into two selected concave
//! edges whose shared face (the floor) is CURVED: a block's top edge running
//! into a cylinder it is unioned with, its two concave edges the block's
//! vertical wall and top face meeting the cylinder.
//!
//! On a planar floor that corner is the runout composition's (`runout.rs`),
//! but only where the corner analysis sees it: a closed edge in the selection
//! (a cylinder's rim) blinds that analysis, so no composition there closes a
//! planar-floor corner either (a block's edge dying through a cylinder's flat
//! top cap, or the cap's rim arc dying into the block's flat wall). There the
//! planar floor is this composition's too (`planar_floors`).
//! On a curved floor the runout composition's ring does not build (the miter of the
//! two concave stripes over the sharp spine edge leaves its carriers) and the
//! cutter composition left the corner as bare edge fillets — the concave arc's
//! wall overhanging the spine's round as a lip, or, at a smaller radius, the
//! spine's wall dropped from the selection.
//!
//! What the rolling ball does there is simple once the spine is rounded: the
//! spine's wall ends on the floor, and its edge with the floor continues the
//! two concave edges TANGENTLY (the wall is tangent to both of the spine's
//! faces), so the concave edges and that bridge form one smooth chain and the
//! concave ball rolls round the rounded spine. So the selection is built in
//! two stages: the spines alone, then every other selected edge with the
//! bridges added to the chain they close.
//!
//! The composition is taken only by the shape it is built for, read before
//! anything is built, and only where the ordinary lanes have no closure for
//! the corner: the floor is curved (or the corner analysis is blind), and
//! each spine's other end is free or a convex STAR (every other selected edge
//! there convex: a box corner). A PARTIAL star, one of whose edges is left
//! sharp, closes in a miter of the rounds that meet there, which only a stage
//! building them together closes: its other edges join the spines' stage. A star
//! end splits the second stage in two (`split_rest`): the spine's round
//! already ends at that corner, so the star's other edges reach it along the
//! round's arc on the far face, an arc of a round of their own radius whose
//! ball does not move. The network cannot march that arc and the cutter lane
//! cannot march the concave chain, so the concave chain goes first on its
//! own, then the rest with those arcs added to their runs. Any stage that
//! refuses or drops a wall sends the selection on to the lanes it took before.
use super::*;
use crate::topology::BrepSolid;

/// One spine's runout: the spine (an index into the selection), the vertex
/// where it dies into two concave edges, and the floor face those share.
struct Runout {
    spine: usize,
    vertex: u64,
    point: Vec3,
    floor: u64,
}

/// The non-degenerate edges at `vertex`.
fn edges_at(solid: &BrepSolid, vertex: u64) -> Vec<u64> {
    solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate && (edge.start_vertex_id == vertex || edge.end_vertex_id == vertex))
        .map(|edge| edge.id)
        .collect()
}

fn is_planar(solid: &BrepSolid, face_id: u64) -> bool {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .is_some_and(|face| matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. })))
}

/// The selection's spine runouts on a curved floor (or any floor, with
/// `planar_floors`), or `None` when the selection is not this composition's
/// shape (no such spine, a spine whose other end meets another selected edge,
/// or, without `planar_floors`, one dying into a planar floor).
fn detect(solid: &BrepSolid, selected_ids: &[u64], radius: f64, planar_floors: bool) -> Option<(Vec<Runout>, bool, Vec<usize>)> {
    // Closed edges (a cylinder's rim) have no corner to share: they are left
    // out of the reading and built with the rest in the second stage.
    let open: Vec<usize> = (0..selected_ids.len())
        .filter(|index| {
            solid
                .edges
                .iter()
                .find(|edge| edge.id == selected_ids[*index])
                .is_some_and(|edge| edge.start_vertex_id != edge.end_vertex_id)
        })
        .collect();
    let open_ids: Vec<u64> = open.iter().map(|index| selected_ids[*index]).collect();
    let mates = crate::blend::selection_mates(solid, &open_ids, radius).ok()?;
    let convex = crate::blend::selection_convexity(solid, &open_ids, radius)?;
    let mut runouts = Vec::new();
    // Whether some spine's far end is a convex star (other selected convex
    // edges meet it there): the second stage is then split (`split_rest`).
    let mut split = false;
    // A star with an edge left sharp (a box corner two of whose three edges
    // are picked): the rounds meet there in a miter, which only a stage that
    // builds them together closes, so the star's other edges join the
    // spines' stage (their own far ends then reach the rest as a star's do).
    let mut with_spines = Vec::new();
    for (slot, &index) in open.iter().enumerate() {
        if !convex[slot] {
            continue;
        }
        let (edge, spine_faces) = (mates[slot].0, mates[slot].1.map(|face| face.id));
        // Each end: a runout (`Some(floor)`), free (no other selected edge
        // there) or anything else.
        enum End {
            Runout(u64),
            Free,
            /// Every other selected edge there is convex: the selection
            /// indices of those edges when an edge there is left sharp
            /// (a partial star), empty when every edge there is selected.
            Star(Vec<usize>),
            Other,
        }
        let mut ends = Vec::with_capacity(2);
        for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
            let around = edges_at(solid, vertex);
            let selected_here: Vec<usize> = around
                .iter()
                .filter_map(|id| open_ids.iter().position(|open_id| open_id == id))
                .filter(|other| *other != slot)
                .collect();
            if selected_here.is_empty() && around.iter().all(|id| !selected_ids.contains(id) || *id == edge.id) {
                ends.push((vertex, End::Free));
                continue;
            }
            if !selected_here.is_empty()
                && selected_here.iter().all(|other| convex[*other])
                && around.iter().all(|id| !selected_ids.contains(id) || open_ids.contains(id))
            {
                let full = around.iter().all(|id| selected_ids.contains(id));
                let partial = if full { Vec::new() } else { selected_here.iter().map(|other| open[*other]).collect() };
                ends.push((vertex, End::Star(partial)));
                continue;
            }
            // Three edges at the vertex, all selected: the spine and two
            // concave edges sharing a face the spine does not touch.
            if around.len() != 3 || selected_here.len() != 2 || selected_here.iter().any(|other| convex[*other]) {
                ends.push((vertex, End::Other));
                continue;
            }
            let [a, b] = [mates[selected_here[0]].1.map(|face| face.id), mates[selected_here[1]].1.map(|face| face.id)];
            let shared: Vec<u64> = a.iter().copied().filter(|id| b.contains(id)).collect();
            match shared[..] {
                [floor] if !spine_faces.contains(&floor) => ends.push((vertex, End::Runout(floor))),
                _ => ends.push((vertex, End::Other)),
            }
        }
        if !ends.iter().any(|(_, end)| matches!(end, End::Runout(_))) {
            continue;
        }
        for (vertex, end) in ends {
            match end {
                // A planar floor is the runout composition's where the
                // corner analysis can see it.
                End::Runout(floor) if !planar_floors && is_planar(solid, floor) => return None,
                End::Runout(floor) => {
                    let point = solid.vertices.iter().find(|candidate| candidate.id == vertex)?.point;
                    runouts.push(Runout { spine: index, vertex, point, floor });
                }
                End::Free => {}
                End::Star(partial) => {
                    split = true;
                    with_spines.extend(partial);
                }
                End::Other => return None,
            }
        }
    }
    (!runouts.is_empty()).then_some((runouts, split, with_spines))
}

/// The spine's bridge: the edge of the first stage's body between the floor
/// and a face that body added (the spine's wall), nearest the runout vertex.
/// Its midpoint selects it.
fn bridge_point(input: &BrepSolid, staged: &BrepSolid, runout: &Runout) -> Option<Vec3> {
    let old_faces: Vec<u64> = input.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).collect();
    let faces_of = |edge_id: u64| -> Vec<u64> {
        staged
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| face.loops.iter().any(|lp| lp.coedges.iter().any(|coedge| coedge.edge_id == edge_id)))
            .map(|face| face.id)
            .collect()
    };
    let mut best: Option<(f64, Vec3)> = None;
    for edge in staged.edges.iter().filter(|edge| !edge.degenerate) {
        let faces = faces_of(edge.id);
        if faces.len() != 2 || !faces.contains(&runout.floor) || faces.iter().all(|face| old_faces.contains(face)) {
            continue;
        }
        let Ok(middle) = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)) else {
            continue;
        };
        let distance = middle.sub(runout.point).length();
        if best.is_none_or(|(nearest, _)| distance < nearest) {
            best = Some((distance, middle));
        }
    }
    best.map(|(_, point)| point)
}

/// Build `edge_points` spine-first when the selection has this shape: `None`
/// when it does not, or when a stage did not build every wall (the caller's
/// lanes then answer as before). `planar_floors` admits spines dying into a
/// planar floor too: only where the corner analysis is blind to the corner.
#[allow(clippy::too_many_arguments)]
pub(super) fn spine_first_composition(
    solid: &BrepSolid,
    selected_ids: &[u64],
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
    planar_floors: bool,
) -> Option<(BrepSolid, BlendSelection)> {
    let debug = std::env::var("BREP_DEBUG_NETWORK").is_ok();
    let (runouts, split, with_spines) = detect(solid, selected_ids, radius, planar_floors)?;
    let mut spines: Vec<usize> = runouts.iter().map(|runout| runout.spine).chain(with_spines).collect();
    spines.sort_unstable();
    spines.dedup();
    if debug {
        for runout in &runouts {
            eprintln!(
                "spine-first: spine #{} dies into the floor {} at vertex {} ({:.4}, {:.4}, {:.4})",
                runout.spine, runout.floor, runout.vertex, runout.point.x, runout.point.y, runout.point.z
            );
        }
    }
    // Per-edge names, when given, must name every selected edge: a short list
    // leaves the selection to the other lanes, which read names by `get`.
    if edge_names.is_some_and(|names| names.len() < edge_points.len()) {
        return None;
    }
    let name_of = |index: usize| edge_names.and_then(|names| names.get(index)).cloned().unwrap_or_default();
    let pick_names = |indices: &[usize]| edge_names.map(|_| indices.iter().map(|index| name_of(*index)).collect::<Vec<_>>());
    // Stage 1: the spines alone (with the other edges of a partial star).
    let spine_points: Vec<Vec3> = spines.iter().map(|index| edge_points[*index]).collect();
    let spine_names = pick_names(&spines);
    if debug {
        eprintln!("spine-first: the first stage picks {:?}", spine_names.clone().unwrap_or_default());
    }
    let staged = match fillet_edges_reported(solid, &spine_points, spine_names.as_deref(), radius, false, name) {
        Ok((staged, selection)) if selection.is_complete() => staged,
        Ok(_) | Err(_) => {
            if debug {
                eprintln!("spine-first: the spines did not build alone; the selection takes the other lanes");
            }
            return None;
        }
    };
    // Stage 2: every other selected edge, with each spine's bridge.
    let rest: Vec<usize> = (0..edge_points.len()).filter(|index| !spines.contains(index)).collect();
    // Each edge is picked again on the first stage's body by its midpoint
    // there: a pick near the spine's end may lie on the stretch its round
    // took. An edge the first stage replaced keeps its pick.
    let mut points: Vec<Vec3> = rest
        .iter()
        .map(|index| {
            staged
                .edges
                .iter()
                .find(|edge| edge.id == selected_ids[*index] && !edge.degenerate)
                .and_then(|edge| edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).ok())
                .unwrap_or(edge_points[*index])
        })
        .collect();
    let mut names = pick_names(&rest);
    for runout in &runouts {
        let Some(bridge) = bridge_point(solid, &staged, runout) else {
            if debug {
                eprintln!("spine-first: no bridge edge on the floor {} by vertex {}", runout.floor, runout.vertex);
            }
            return None;
        };
        points.push(bridge);
        if let Some(names) = names.as_mut() {
            names.push(format!("{}:FOOT", name_of(runout.spine)));
        }
    }
    let result = if split {
        split_rest(solid, &staged, selected_ids, edge_points, &rest, points, names, &name_of, edge_names.is_some(), radius, name, debug)?
    } else {
        build_stage(&staged, &points, names.as_deref(), radius, name, "the second stage", debug)?
    };
    let n = edge_points.len();
    Some((result, BlendSelection { requested: n, applied: (0..n).collect(), rejected: Vec::new(), reason: None }))
}

/// One stage: `None` (and a debug line) unless it builds every wall.
fn build_stage(
    body: &BrepSolid,
    points: &[Vec3],
    names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
    what: &str,
    debug: bool,
) -> Option<BrepSolid> {
    if debug {
        eprintln!("spine-first: {what} picks {:?}", names.map(|names| names.to_vec()).unwrap_or_default());
    }
    match fillet_edges_reported(body, points, names, radius, false, name) {
        Ok((result, selection)) if selection.is_complete() => Some(result),
        Ok((_, selection)) => {
            if debug {
                let named: Vec<String> = selection
                    .rejected
                    .iter()
                    .map(|index| names.and_then(|names| names.get(*index)).cloned().unwrap_or_else(|| format!("#{index} at {:?}", points.get(*index))))
                    .collect();
                eprintln!("spine-first: {what} dropped {:?} ({named:?})", selection.rejected);
            }
            None
        }
        Err(error) => {
            if debug {
                eprintln!("spine-first: {what} refused: {error}");
            }
            None
        }
    }
}

/// The second stage when a spine's far end is a convex STAR: the spine's
/// round already ends there, so the star's other edges reach it along the
/// round's arc on the far face, an arc of a round of their own radius whose
/// ball does not move (the network cannot march it; the cutter lane rounds it
/// as a sphere). The concave chain, which only the network marches, is built
/// on its own first: every selected concave edge with the spines' bridges.
/// Then every other edge, with each new edge the first two stages left
/// continuing one of them smoothly (the far arcs, and the concave rounds'
/// arcs on a face whose rim is selected) added to its run.
#[allow(clippy::too_many_arguments)]
fn split_rest(
    input: &BrepSolid,
    staged: &BrepSolid,
    selected_ids: &[u64],
    edge_points: &[Vec3],
    rest: &[usize],
    points: Vec<Vec3>,
    names: Option<Vec<String>>,
    name_of: &dyn Fn(usize) -> String,
    named: bool,
    radius: f64,
    name: Option<&str>,
    debug: bool,
) -> Option<BrepSolid> {
    // `points`/`names` are `rest` in order, then the bridges.
    let concave: Vec<bool> = rest
        .iter()
        .map(|index| {
            let id = selected_ids[*index];
            let closed = input.edges.iter().find(|edge| edge.id == id).is_none_or(|edge| edge.start_vertex_id == edge.end_vertex_id);
            !closed && crate::blend::selection_convexity(input, &[id], radius).is_some_and(|convex| !convex[0])
        })
        .collect();
    let (mut concave_points, mut concave_names) = (Vec::new(), Vec::new());
    let (mut other_slots, mut other_names) = (Vec::new(), Vec::new());
    for (slot, point) in points.iter().enumerate() {
        let name = names.as_ref().map(|names| names[slot].clone()).unwrap_or_default();
        if slot >= rest.len() || concave[slot] {
            concave_points.push(*point);
            concave_names.push(name);
        } else {
            other_slots.push(slot);
            other_names.push(name);
        }
    }
    let concave_staged = build_stage(staged, &concave_points, named.then_some(&concave_names[..]), radius, name, "the concave chain", debug)?;
    // Every other edge, picked again on that body by its midpoint there.
    let mut other_points: Vec<Vec3> = other_slots
        .iter()
        .map(|slot| {
            concave_staged
                .edges
                .iter()
                .find(|edge| edge.id == selected_ids[rest[*slot]] && !edge.degenerate)
                .and_then(|edge| edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).ok())
                .unwrap_or(edge_points[rest[*slot]])
        })
        .collect();
    for (point, source) in smooth_runs(input, &concave_staged, &other_points) {
        other_points.push(point);
        other_names.push(format!("{}:RUN", name_of(rest[other_slots[source]])));
        if debug {
            eprintln!("spine-first: run {} at ({:.4}, {:.4}, {:.4})", other_names.last().unwrap(), point.x, point.y, point.z);
        }
    }
    if debug {
        eprintln!("spine-first: the last stage takes {} edges ({} added runs)", other_points.len(), other_points.len() - other_slots.len());
    }
    // The far arcs are rounds of the stage's own radius: the ball's
    // ceiling there IS the radius, which the misfit probe reads a hair under
    // it (0.9987 r at r 0.1 on the reported model) and takes for a fold that
    // pre-empts the lanes that round the arc. Off for this stage, but only
    // when the selected edges themselves read no misfit: a selected edge
    // the ball truly folds on keeps its refusal or relief.
    let selected_here: Vec<u64> = other_points[..other_slots.len()]
        .iter()
        .filter_map(|point| super::edges::resolve_edge_by_point(&concave_staged, *point).ok())
        .collect();
    let _misfit_off = crate::blend::relief::first_misfit(&concave_staged, &selected_here, radius)
        .is_none()
        .then(crate::blend::relief::MisfitOff::enter);
    build_stage(&concave_staged, &other_points, named.then_some(&other_names[..]), radius, name, "the last stage", debug)
}

/// The edges of `body` that `input` did not have and that continue one of the
/// edges `picked` names smoothly (G1, the two leaving their shared vertex in
/// opposite directions) without being smooth themselves: each one's midpoint
/// and the index of the picked edge whose run it extends.
fn smooth_runs(input: &BrepSolid, body: &BrepSolid, picked: &[Vec3]) -> Vec<(Vec3, usize)> {
    let old: Vec<u64> = input.edges.iter().map(|edge| edge.id).collect();
    let mut run: Vec<(u64, usize)> = picked
        .iter()
        .enumerate()
        .filter_map(|(slot, point)| super::edges::resolve_edge_by_point(body, *point).ok().map(|id| (id, slot)))
        .collect();
    let leaving = |edge: &crate::topology::EdgeRecord, vertex: u64| -> Option<Vec3> {
        let at_start = edge.start_vertex_id == vertex;
        let tangent = edge.curve.unit_tangent(if at_start { edge.t0 } else { edge.t1 }, edge.t0, edge.t1).ok()?;
        Some(if at_start { tangent } else { tangent.scale(-1.0) })
    };
    let creased = |edge: &crate::topology::EdgeRecord| -> bool {
        let Ok(middle) = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)) else {
            return false;
        };
        let normals: Vec<Vec3> = body
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| face.loops.iter().any(|lp| lp.coedges.iter().any(|coedge| coedge.edge_id == edge.id)))
            .filter_map(|face| {
                let uv = crate::project_point_to_surface(&face.surface, middle).ok()?;
                let normal = face.surface.normal(uv.u, uv.v).ok()?;
                Some(if face.same_sense { normal } else { normal.scale(-1.0) })
            })
            .collect();
        normals.len() == 2 && normals[0].dot(normals[1]) < 1.0 - 1e-6
    };
    let mut added = Vec::new();
    loop {
        let mut grew = false;
        for at in 0..run.len() {
            let (id, source) = run[at];
            let Some(edge) = body.edges.iter().find(|edge| edge.id == id) else {
                continue;
            };
            for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
                let Some(own) = leaving(edge, vertex) else {
                    continue;
                };
                let candidates: Vec<&crate::topology::EdgeRecord> = body
                    .edges
                    .iter()
                    .filter(|other| {
                        !other.degenerate
                            && other.start_vertex_id != other.end_vertex_id
                            && (other.start_vertex_id == vertex || other.end_vertex_id == vertex)
                            && !old.contains(&other.id)
                            && !run.iter().any(|(taken, _)| *taken == other.id)
                    })
                    .collect();
                for other in candidates {
                    let smooth = leaving(other, vertex).is_some_and(|theirs| own.dot(theirs) <= -(1.0 - 1e-6));
                    if smooth && creased(other) {
                        if let Ok(middle) = other.curve.evaluate(0.5 * (other.t0 + other.t1)) {
                            run.push((other.id, source));
                            added.push((middle, source));
                            grew = true;
                        }
                    }
                }
            }
        }
        if !grew {
            break;
        }
    }
    added
}
