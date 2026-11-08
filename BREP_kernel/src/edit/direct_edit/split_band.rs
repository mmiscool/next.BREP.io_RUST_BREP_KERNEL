//! Closed blend rings whose neighbours arrive CUT INTO PIECES.
//!
//! The closed-band lane (`closed_band.rs`) counts survivor FACES and asks for
//! exactly two. Two ordinary shapes break that count without changing the
//! question, and each is healed here:
//!
//! ## A band between split walls ([`classify_split_band`])
//!
//! CAD exporters cut a periodic face at two meridians, so an imported fillet
//! ring between a cylinder and a cone is two half-tori between two half
//! cylinders and two half cones: four survivor faces on two carriers. The
//! free boundary is one closed circuit per CARRIER, made of one open run per
//! face, handed over head to tail at the meridian vertices.
//!
//! The heal is the closed strip's: both carriers are extended and
//! re-intersected. The healed rim is cut at every handover vertex, each face's
//! run collapses onto its own pieces of the rim, and the meridian edges that
//! ended on a handover grow to the rim.
//!
//! ## A ring that overran the strips beside it ([`reduce_ring_over_strips`])
//!
//! Fillet a bore's mouth and the four edges around it with a radius large
//! enough that the two footprints overlap: the mouth's torus cuts each edge
//! strip's rim in two and leaves the planar face as four corner triangles.
//! With every one of those blends selected, no lane reads the whole: each strip
//! has three rims, and the torus is a twelve-sided face.
//!
//! Without the ring, the rest IS an ordinary blend network, so the ring goes
//! first. The planar pieces are gathered into one face whose hole is the ring's
//! whole outer circuit, and each strip's rim is straightened back across the
//! ring. The ring then reads as a closed band between that hole and the bore
//! and takes the existing closed-band heal, whose result is validated. The
//! strips and vertex blends left over go to the lanes that already read them.
//!
//! The intermediate solid exists only to be handed to the band heal: until the
//! ring is gone its hole loop wears the strip edges, so it is not validated on
//! its own. Everything the gate does not recognise declines to the lanes that
//! ran before this one.

use super::delete_faces::euler_characteristic;
use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// How densely a run is sampled for branch scoring and to read its path.
const RUN_SAMPLES: usize = 8;

/// One survivor's open stretch of a split band's circuit.
pub(super) struct SplitRim {
    face: u64,
    loop_index: usize,
    /// Coedge positions in the loop, in traversal order.
    run: Vec<usize>,
    start: u64,
    end: u64,
}

/// A selection the gate read as a closed band between two split carriers.
pub(super) struct SplitBand {
    shell_index: usize,
    faces: HashSet<u64>,
    edges: HashSet<u64>,
    groups: [Vec<SplitRim>; 2],
}

fn traversed(coedge: &CoedgeRecord, edges: &HashMap<u64, &EdgeRecord>) -> Option<(u64, u64)> {
    let edge = edges.get(&coedge.edge_id)?;
    Some(if coedge.forward {
        (edge.start_vertex_id, edge.end_vertex_id)
    } else {
        (edge.end_vertex_id, edge.start_vertex_id)
    })
}

/// A ruled surface of revolution, as `(axis point, unit axis, radius law)`.
fn ruled_form(surface: &NurbsSurface) -> Option<(Vec3, Vec3, f64, f64)> {
    let AnalyticSurface::RuledRevolution { frame, rho0, rho1, height } = surface.analytic()? else {
        return None;
    };
    let slope = (rho1 - rho0) / height;
    Some((frame.origin, frame.axis, *rho0, slope))
}

/// Whether two analytic carriers are one geometric surface: halves of a
/// revolution are fitted apart with their own frames, so compare the shapes.
fn same_carrier(a: &NurbsSurface, b: &NurbsSurface, tolerance: f64) -> bool {
    if let (Some((oa, axis_a, rho_a, slope_a)), Some((ob, axis_b, rho_b, slope_b))) =
        (ruled_form(a), ruled_form(b))
    {
        let off_axis = ob.sub(oa);
        if axis_a.cross(axis_b).length() > 1e-9
            || off_axis.sub(axis_a.scale(off_axis.dot(axis_a))).length() > tolerance
        {
            return false;
        }
        // The radius law in A's axial coordinate, read through B's.
        let sign = axis_a.dot(axis_b);
        let shift = off_axis.dot(axis_a);
        let rho_b_at = |z: f64| rho_b + slope_b * (z - shift) * sign;
        let rho_a_at = |z: f64| rho_a + slope_a * z;
        return (rho_a_at(0.0) - rho_b_at(0.0)).abs() <= tolerance
            && (rho_a_at(1.0) - rho_b_at(1.0)).abs() <= tolerance;
    }
    let (Some(AnalyticSurface::Plane { .. }), Some(AnalyticSurface::Plane { .. })) =
        (a.analytic(), b.analytic())
    else {
        return false;
    };
    match (
        plane_of_surface(a, tolerance, "split band"),
        plane_of_surface(b, tolerance, "split band"),
    ) {
        (Ok(first), Ok(second)) => {
            first.normal.cross(second.normal).length() <= 1e-9
                && second.origin.sub(first.origin).dot(first.normal).abs() <= tolerance
        }
        _ => false,
    }
}

/// Does this selection read as a closed band whose two walls are split across
/// several faces? `None` leaves it to the lanes after this one.
pub(super) fn classify_split_band(solid: &BrepSolid, face_ids: &[u64]) -> Option<SplitBand> {
    if face_ids.len() < 2 {
        return None;
    }
    let faces: HashSet<u64> = face_ids.iter().copied().collect();
    let shell_index = find_face(solid, face_ids[0])?.0;
    for face_id in face_ids {
        if find_face(solid, *face_id)?.0 != shell_index {
            return None;
        }
    }
    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut uses: HashMap<u64, (usize, usize)> = HashMap::default();
    let mut keeper: HashMap<u64, u64> = HashMap::default();
    let mut owners: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let selected = faces.contains(&face.id);
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let entry = uses.entry(coedge.edge_id).or_default();
            if selected {
                entry.0 += 1;
                owners.entry(coedge.edge_id).or_default().push(face.id);
            } else {
                entry.1 += 1;
                keeper.insert(coedge.edge_id, face.id);
            }
        }
    }
    let band_edges: HashSet<u64> = uses
        .iter()
        .filter(|(_, (selected, _))| *selected > 0)
        .map(|(edge_id, _)| *edge_id)
        .collect();
    for edge_id in &band_edges {
        let (selected, kept) = uses[edge_id];
        if edges.get(edge_id)?.degenerate || selected + kept != 2 {
            return None;
        }
    }

    // One edge-connected annulus.
    let mut reached: HashSet<u64> = HashSet::default();
    let mut stack = vec![face_ids[0]];
    reached.insert(face_ids[0]);
    while let Some(face_id) = stack.pop() {
        let (shell, position) = find_face(solid, face_id)?;
        for coedge in solid.shells[shell].faces[position].loops.iter().flat_map(|l| &l.coedges) {
            for other in owners.get(&coedge.edge_id).into_iter().flatten() {
                if reached.insert(*other) {
                    stack.push(*other);
                }
            }
        }
    }
    if reached.len() != faces.len() {
        return None;
    }
    let vertices: HashSet<u64> = band_edges
        .iter()
        .flat_map(|edge_id| [edges[edge_id].start_vertex_id, edges[edge_id].end_vertex_id])
        .collect();
    let holes: usize = face_ids
        .iter()
        .filter_map(|face_id| find_face(solid, *face_id))
        .map(|(shell, position)| solid.shells[shell].faces[position].loops.len() - 1)
        .sum();
    if vertices.len() as i64 - band_edges.len() as i64 + faces.len() as i64 - holes as i64 != 0 {
        return None;
    }

    // Survivors grouped by carrier: exactly two carriers, one of them split.
    let mut survivors: Vec<u64> = band_edges
        .iter()
        .filter(|edge_id| uses[*edge_id].1 == 1)
        .map(|edge_id| keeper[edge_id])
        .collect();
    survivors.sort_unstable();
    survivors.dedup();
    if survivors.len() < 3 {
        return None;
    }
    let tolerance = (solid_model_scale(solid) * 1e-6).max(1e-9);
    let surface_of = |id: u64| -> Option<&NurbsSurface> {
        let (shell, position) = find_face(solid, id)?;
        Some(&solid.shells[shell].faces[position].surface)
    };
    let mut carriers: Vec<Vec<u64>> = Vec::new();
    for survivor in survivors {
        let surface = surface_of(survivor)?;
        surface.analytic()?;
        match carriers
            .iter_mut()
            .find(|group| surface_of(group[0]).is_some_and(|first| same_carrier(first, surface, tolerance)))
        {
            Some(group) => group.push(survivor),
            None => carriers.push(vec![survivor]),
        }
    }
    if carriers.len() != 2 {
        return None;
    }

    // One open run per face, and each carrier's runs close one circuit.
    let mut groups: Vec<Vec<SplitRim>> = Vec::with_capacity(2);
    for carrier in &carriers {
        let mut rims: Vec<SplitRim> = Vec::with_capacity(carrier.len());
        for &neighbour in carrier {
            let (shell, position) = find_face(solid, neighbour)?;
            if shell != shell_index {
                return None;
            }
            let face = &solid.shells[shell].faces[position];
            let mut found: Option<SplitRim> = None;
            for (loop_index, loop_record) in face.loops.iter().enumerate() {
                let flags: Vec<bool> = loop_record
                    .coedges
                    .iter()
                    .map(|coedge| band_edges.contains(&coedge.edge_id))
                    .collect();
                if !flags.iter().any(|flag| *flag) {
                    continue;
                }
                let count = flags.len();
                let starts: Vec<usize> = (0..count)
                    .filter(|index| flags[*index] && !flags[(index + count - 1) % count])
                    .collect();
                if found.is_some() || starts.len() != 1 {
                    return None;
                }
                let run: Vec<usize> = (0..count)
                    .map(|step| (starts[0] + step) % count)
                    .take_while(|index| flags[*index])
                    .collect();
                let start = traversed(&loop_record.coedges[run[0]], &edges)?.0;
                let end = traversed(&loop_record.coedges[*run.last()?], &edges)?.1;
                if start == end {
                    return None;
                }
                found = Some(SplitRim { face: neighbour, loop_index, run, start, end });
            }
            rims.push(found?);
        }
        let by_start: HashMap<u64, usize> =
            rims.iter().enumerate().map(|(index, rim)| (rim.start, index)).collect();
        if by_start.len() != rims.len() {
            return None;
        }
        // The walk has to come home having visited every run exactly once.
        let mut steps = 1usize;
        let mut walk = *by_start.get(&rims[0].end)?;
        while walk != 0 && steps <= rims.len() {
            walk = *by_start.get(&rims[walk].end)?;
            steps += 1;
        }
        if steps != rims.len() {
            return None;
        }
        groups.push(rims);
    }
    let second = groups.pop()?;
    let first = groups.pop()?;
    Some(SplitBand { shell_index, faces, edges: band_edges, groups: [first, second] })
}

fn missing(what: &str, id: u64, op: &str) -> KernelRefusal {
    KernelRefusal::internal(KernelStage::Collect, "topology", format!("{op}: missing {what} {id}"))
}

/// Heal a [`SplitBand`]: see the module docs.
pub(super) fn heal_split_band(
    solid: &BrepSolid,
    band: &SplitBand,
    op: &str,
) -> Result<BrepSolid, KernelRefusal> {
    let original = solid;
    let mut solid = solid.clone();
    let scale = solid_model_scale(&solid);
    let tolerance = (scale * 1e-6).max(1e-9);
    let pcurve_tolerance = (scale * 1e-7).max(1e-9);
    let refusal = |slug: &'static str, reason: String| {
        KernelRefusal::unsupported(
            KernelStage::Sew,
            slug,
            format!("{op}: the selection is a closed band between split walls, and {reason} (deferred)"),
        )
    };

    let edge_map: HashMap<u64, EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge.clone())).collect();
    let vertex_point: HashMap<u64, Vec3> =
        solid.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();

    // Sample every run in its own traversal order.
    let run_points = |rim: &SplitRim| -> Result<Vec<Vec3>, KernelRefusal> {
        let (shell, position) = find_face(&solid, rim.face).ok_or_else(|| missing("face", rim.face, op))?;
        let loop_record = &solid.shells[shell].faces[position].loops[rim.loop_index];
        let mut points = Vec::with_capacity(rim.run.len() * RUN_SAMPLES);
        for index in &rim.run {
            let coedge = &loop_record.coedges[*index];
            let edge = edge_map.get(&coedge.edge_id).ok_or_else(|| missing("edge", coedge.edge_id, op))?;
            for step in 0..RUN_SAMPLES {
                let fraction = step as f64 / RUN_SAMPLES as f64;
                let fraction = if coedge.forward { fraction } else { 1.0 - fraction };
                points.push(
                    edge.curve
                        .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)
                        .or_refuse(KernelStage::Classify, "evaluate")?,
                );
            }
        }
        Ok(points)
    };
    let mut band_points = Vec::new();
    let mut per_rim: HashMap<u64, Vec<Vec3>> = HashMap::default();
    for rim in band.groups.iter().flatten() {
        let points = run_points(rim)?;
        band_points.extend(points.iter().copied());
        per_rim.insert(rim.face, points);
    }

    // Extend, then re-intersect the two carriers.
    for rim in band.groups.iter().flatten() {
        extend_ruled_neighbour_over(&mut solid, rim.face, &band_points, tolerance)?;
    }
    let surface = |solid: &BrepSolid, id: u64| -> Result<NurbsSurface, KernelRefusal> {
        let (shell, position) = find_face(solid, id).ok_or_else(|| missing("face", id, op))?;
        Ok(solid.shells[shell].faces[position].surface.clone())
    };
    let surface_a = surface(&solid, band.groups[0][0].face)?;
    let surface_b = surface(&solid, band.groups[1][0].face)?;
    let curves = intersect_analytic_pair(&surface_a, &surface_b, tolerance).ok_or_else(|| {
        refusal("analytic_pair", "its walls are not a recognized analytic pair".into())
    })?;
    let mut centre = Vec3::default();
    for point in &band_points {
        centre = centre.add(*point);
    }
    let centre = centre.scale(1.0 / band_points.len().max(1) as f64);
    let extent = 2.0 * band_points.iter().map(|point| point.sub(centre).length()).fold(0.0f64, f64::max);
    let mut best: Option<(f64, NurbsCurve)> = None;
    for curve in curves {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
        if curve.evaluate(t0).or_refuse(KernelStage::Classify, "evaluate")?
            .sub(curve.evaluate(t1).or_refuse(KernelStage::Classify, "evaluate")?)
            .length()
            > tolerance
        {
            continue;
        }
        let mut worst = 0.0f64;
        for step in 0..32 {
            let point = curve
                .evaluate(t0 + (t1 - t0) * step as f64 / 32.0)
                .or_refuse(KernelStage::Classify, "evaluate")?;
            let nearest =
                band_points.iter().map(|at| at.sub(point).length()).fold(f64::INFINITY, f64::min);
            worst = worst.max(nearest);
        }
        if best.as_ref().map_or(true, |(score, _)| worst < *score) {
            best = Some((worst, curve));
        }
    }
    let (_, rim) = best
        .filter(|(score, _)| *score <= extent)
        .ok_or_else(|| refusal("closed_rim", "its walls do not re-intersect in a closed rim near it".into()))?;

    // --- Cut the rim at every handover vertex ---------------------------------
    let first_point = *vertex_point
        .get(&band.groups[0][0].start)
        .ok_or_else(|| missing("vertex", band.groups[0][0].start, op))?;
    let rim = align_closed_rim_origin(&rim, first_point, tolerance, op)?;
    let [t0, t1] = rim.domain().or_refuse(KernelStage::Sew, "domain")?;
    let period = t1 - t0;
    let rim_at = |t: f64| rim.evaluate(t).or_refuse(KernelStage::Sew, "evaluate");
    let stations_of = |point: Vec3| -> Result<f64, KernelRefusal> {
        let hit = project_point_to_curve(&rim, point).or_refuse(KernelStage::Sew, "project")?;
        let station = (hit.u - t0).rem_euclid(period);
        Ok(if station > period * (1.0 - 1e-9) { 0.0 } else { station })
    };
    let mut handovers: Vec<(u64, f64)> = Vec::new();
    for rim_run in band.groups.iter().flatten() {
        for vertex in [rim_run.start, rim_run.end] {
            if !handovers.iter().any(|(id, _)| *id == vertex) {
                let point = *vertex_point.get(&vertex).ok_or_else(|| missing("vertex", vertex, op))?;
                handovers.push((vertex, stations_of(point)?));
            }
        }
    }
    handovers.sort_by(|a, b| a.1.total_cmp(&b.1));
    // Handovers that land on one rim point (a meridian on each wall) share it.
    let mut stations: Vec<f64> = Vec::new();
    let mut station_of: HashMap<u64, usize> = HashMap::default();
    for (vertex, station) in &handovers {
        let joined = match stations.last() {
            Some(last) => {
                rim_at(t0 + *last)?.sub(rim_at(t0 + *station)?).length() <= tolerance
            }
            None => false,
        };
        if !joined {
            let wraps = stations.first().is_some_and(|first| {
                rim_at(t0 + *first)
                    .and_then(|a| Ok(a.sub(rim_at(t0 + *station)?).length()))
                    .is_ok_and(|gap| gap <= tolerance)
            });
            if wraps {
                station_of.insert(*vertex, 0);
                continue;
            }
            stations.push(*station);
        }
        station_of.insert(*vertex, stations.len() - 1);
    }
    let pieces = stations.len();
    if pieces < 2 {
        return Err(refusal("handover", "its circuits hand over at fewer than two rim points".into()));
    }

    let mut next_id = max_topology_id(&solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };
    let vertex_ids: Vec<u64> = (0..pieces).map(|_| alloc()).collect();
    let mut points = Vec::with_capacity(pieces);
    for (index, station) in stations.iter().enumerate() {
        let point = rim_at(t0 + *station)?;
        points.push(point);
        solid.vertices.push(VertexRecord { id: vertex_ids[index], point });
    }
    let mut piece_curves: Vec<NurbsCurve> = Vec::with_capacity(pieces);
    let mut rest = rim.clone();
    for station in stations.iter().skip(1) {
        let (head, tail) = rest.split(t0 + *station).or_refuse(KernelStage::Sew, "split")?;
        piece_curves.push(head);
        rest = tail;
    }
    piece_curves.push(rest);
    // The rim was re-origined on the first handover run's start, which is the
    // first station only when that vertex sorts first; `stations[0]` is zero.
    if stations[0].abs() > 1e-9 * period {
        return Err(KernelRefusal::internal(
            KernelStage::Sew,
            "rim_origin",
            format!("{op}: the healed rim does not start on a handover vertex"),
        ));
    }
    let mut new_edges: Vec<u64> = Vec::with_capacity(pieces);
    for piece in 0..pieces {
        let id = alloc();
        let [a, b] = piece_curves[piece].domain().or_refuse(KernelStage::Sew, "domain")?;
        solid.edges.push(EdgeRecord {
            id,
            curve: piece_curves[piece].clone(),
            t0: a,
            t1: b,
            start_vertex_id: vertex_ids[piece],
            end_vertex_id: vertex_ids[(piece + 1) % pieces],
            degenerate: false,
            name: None,
        });
        new_edges.push(id);
    }

    // --- Each run collapses onto its own pieces of the rim --------------------
    let placeholder = make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0)).or_refuse(KernelStage::Sew, "make_line")?;
    for rim_run in band.groups.iter().flatten() {
        let from = station_of[&rim_run.start];
        let to = station_of[&rim_run.end];
        if from == to {
            return Err(refusal("run_collapse", format!("the run on face {} collapses to a point", rim_run.face)));
        }
        // Which way round: the run's own middle sample names the arc it covers.
        let samples = &per_rim[&rim_run.face];
        let middle = stations_of(samples[samples.len() / 2])?;
        let arc_contains = |from: f64, to: f64, at: f64| -> bool {
            (at - from).rem_euclid(period) < (to - from).rem_euclid(period)
        };
        let forward = arc_contains(stations[from], stations[to], middle);
        let order: Vec<usize> = if forward {
            std::iter::successors(Some(from), |piece| Some((piece + 1) % pieces))
                .take_while(|piece| *piece != to)
                .collect()
        } else {
            std::iter::successors(Some((from + pieces - 1) % pieces), |piece| {
                Some((piece + pieces - 1) % pieces)
            })
            .take_while(|piece| (piece + 1) % pieces != to)
            .collect()
        };
        let mut coedges: Vec<CoedgeRecord> = Vec::with_capacity(order.len());
        for piece in order {
            coedges.push(CoedgeRecord {
                id: alloc(),
                edge_id: new_edges[piece],
                forward,
                pcurve: placeholder.clone(),
            });
        }
        let (shell, position) = find_face(&solid, rim_run.face).ok_or_else(|| missing("face", rim_run.face, op))?;
        let loop_record = &mut solid.shells[shell].faces[position].loops[rim_run.loop_index];
        let count = loop_record.coedges.len();
        let last = *rim_run.run.last().expect("a run has a coedge");
        for step in 1..=(count - rim_run.run.len()) {
            coedges.push(loop_record.coedges[(last + step) % count].clone());
        }
        loop_record.coedges = coedges;
    }

    // --- The meridians that ended on a handover grow to the rim ---------------
    let mut extended: HashMap<u64, (bool, bool)> = HashMap::default();
    for edge in &mut solid.edges {
        if band.edges.contains(&edge.id) || new_edges.contains(&edge.id) {
            continue;
        }
        let start_hit = station_of.get(&edge.start_vertex_id).copied();
        let end_hit = station_of.get(&edge.end_vertex_id).copied();
        if start_hit.is_none() && end_hit.is_none() {
            continue;
        }
        if start_hit.is_some() && end_hit.is_some() {
            return Err(refusal("edge_collapse", format!("edge {} would span the rim", edge.id)));
        }
        let Some(_) = edge.curve.straight_segment(tolerance) else {
            return Err(refusal("curved_meridian", format!("edge {} reaching the band is curved", edge.id)));
        };
        let station = start_hit.or(end_hit).expect("one end hit");
        let target = points[station];
        let anchor = edge
            .curve
            .evaluate(if start_hit.is_some() { edge.t1 } else { edge.t0 })
            .or_refuse(KernelStage::Sew, "evaluate")?;
        if start_hit.is_some() {
            edge.start_vertex_id = vertex_ids[station];
            edge.curve = make_line(target, anchor).or_refuse(KernelStage::Sew, "make_line")?;
        } else {
            edge.end_vertex_id = vertex_ids[station];
            edge.curve = make_line(anchor, target).or_refuse(KernelStage::Sew, "make_line")?;
        }
        [edge.t0, edge.t1] = edge.curve.domain().or_refuse(KernelStage::Sew, "domain")?;
        extended.insert(edge.id, (start_hit.is_some(), end_hit.is_some()));
    }

    // --- Pcurves --------------------------------------------------------------
    let edges_by_id: HashMap<u64, EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge.clone())).collect();
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            if band.faces.contains(&face.id) {
                continue;
            }
            let ruled = match face.surface.analytic() {
                Some(AnalyticSurface::RuledRevolution { frame, height, .. }) => {
                    Some((frame.origin, frame.axis, *height))
                }
                _ => None,
            };
            let mut touched: HashSet<u64> = HashSet::default();
            for coedge in face.loops.iter_mut().flat_map(|loop_record| &mut loop_record.coedges) {
                if new_edges.contains(&coedge.edge_id) {
                    touched.insert(coedge.edge_id);
                    continue;
                }
                let Some(&(start_moved, _)) = extended.get(&coedge.edge_id) else {
                    continue;
                };
                // A straight meridian keeps its two-point pcurve on its own
                // seam branch; only the moved end's v changes.
                match ruled {
                    Some((origin, axis, height))
                        if coedge.pcurve.degree == 1 && coedge.pcurve.control_points.len() == 2 =>
                    {
                        let edge = &edges_by_id[&coedge.edge_id];
                        let moved = if start_moved {
                            edge.curve.evaluate(edge.t0)
                        } else {
                            edge.curve.evaluate(edge.t1)
                        }
                        .or_refuse(KernelStage::Sew, "evaluate")?;
                        let v_new = (moved.sub(origin).dot(axis) / height).clamp(0.0, 1.0);
                        let at_start = start_moved == coedge.forward;
                        let index = if at_start { 0 } else { 1 };
                        let control = &mut coedge.pcurve.control_points[index];
                        control.y = v_new * control.w;
                    }
                    _ => {
                        touched.insert(coedge.edge_id);
                    }
                }
            }
            if !touched.is_empty() {
                refit_touched_pcurves(face, &edges_by_id, &touched, false, pcurve_tolerance, op)?;
            }
        }
    }

    // --- Drop the band --------------------------------------------------------
    solid.shells[band.shell_index].faces.retain(|face| !band.faces.contains(&face.id));
    solid.edges.retain(|edge| !band.edges.contains(&edge.id));
    let used: HashSet<u64> =
        solid.edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect();
    solid.vertices.retain(|vertex| used.contains(&vertex.id));

    let (before, after) = (euler_characteristic(original), euler_characteristic(&solid));
    if before != after {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "euler",
            format!("{op}: closing the split band changes the Euler characteristic {before} -> {after}"),
        ));
    }
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validation",
            format!("{op}: split-band heal failed validation: {issues:?}"),
        ));
    }
    Ok(solid)
}

/// A ring blend that overran selected strips, reduced to a solid in which it
/// is an ordinary closed band: `(ring id, the solid)`.
fn ring_over_strips(solid: &BrepSolid, selected: &HashSet<u64>, ring_id: u64) -> Option<BrepSolid> {
    let tolerance = (solid_model_scale(solid) * 1e-6).max(1e-9);
    let (shell_index, ring_position) = find_face(solid, ring_id)?;
    let ring = &solid.shells[shell_index].faces[ring_position];
    if ring.loops.len() != 1 {
        return None;
    }
    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut owners: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            owners.entry(coedge.edge_id).or_default().push(face.id);
        }
    }
    let across = |edge: u64, from: u64| -> Option<u64> {
        let sharing = owners.get(&edge)?;
        if sharing.len() != 2 {
            return None;
        }
        Some(if sharing[0] == from { sharing[1] } else { sharing[0] })
    };

    // The ring's own loop: survivors are planar pieces and one curved wall.
    let face_of = |id: u64| -> Option<&FaceRecord> {
        let (shell, position) = find_face(solid, id)?;
        Some(&solid.shells[shell].faces[position])
    };
    let mut pieces: Vec<u64> = Vec::new();
    let mut strips: Vec<u64> = Vec::new();
    let mut wall: Option<u64> = None;
    for coedge in &ring.loops[0].coedges {
        let other = across(coedge.edge_id, ring_id)?;
        if other == ring_id {
            continue; // the ring's own seam
        }
        let other_face = face_of(other)?;
        if selected.contains(&other) {
            if !strips.contains(&other) {
                strips.push(other);
            }
        } else if matches!(other_face.surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
            if !pieces.contains(&other) {
                pieces.push(other);
            }
        } else if wall.is_none_or(|known| known == other) {
            wall = Some(other);
        } else {
            return None;
        }
    }
    wall?;
    if pieces.len() < 2 || strips.is_empty() {
        return None;
    }
    pieces.sort_unstable();
    let first_surface = &face_of(pieces[0])?.surface;
    if !pieces.iter().all(|piece| face_of(*piece).is_some_and(|face| same_carrier(first_surface, &face.surface, tolerance))) {
        return None;
    }

    // Every edge of every planar piece is on the ring or on a selected strip,
    // so the pieces hold nothing else that a rejoin would have to keep.
    for piece in &pieces {
        let face = face_of(*piece)?;
        if face.loops.len() != 1 {
            return None;
        }
        for coedge in &face.loops[0].coedges {
            let other = across(coedge.edge_id, *piece)?;
            if other != ring_id && !strips.contains(&other) {
                return None;
            }
        }
    }

    // The ring's circuit on the planar side: everything but its seam and wall.
    let ring_loop = &ring.loops[0].coedges;
    let count = ring_loop.len();
    let on_circuit = |coedge: &CoedgeRecord| -> bool {
        across(coedge.edge_id, ring_id).is_some_and(|other| pieces.contains(&other) || strips.contains(&other))
    };
    let start = (0..count).find(|index| on_circuit(&ring_loop[*index]) && !on_circuit(&ring_loop[(index + count - 1) % count]))?;
    let circuit: Vec<&CoedgeRecord> = (0..count)
        .map(|step| &ring_loop[(start + step) % count])
        .take_while(|coedge| on_circuit(coedge))
        .collect();
    if circuit.len() != ring_loop.iter().filter(|coedge| on_circuit(coedge)).count() {
        return None;
    }
    let circuit_start = traversed(circuit[0], &edges)?.0;
    if traversed(circuit[circuit.len() - 1], &edges)?.1 != circuit_start {
        return None;
    }

    // Each strip: one run of [rim, ring edges…, rim] straightened into one edge.
    let mut result = solid.clone();
    let mut next_id = max_topology_id(solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };
    let mut dropped_edges: HashSet<u64> = HashSet::default();
    let mut outer: Vec<(u64, u64, u64)> = Vec::new(); // (edge id, from, to) as walked by the strip
    let mut new_edges: Vec<EdgeRecord> = Vec::new();
    for strip_id in &strips {
        let strip = face_of(*strip_id)?;
        if strip.loops.len() != 1 {
            return None;
        }
        let coedges = &strip.loops[0].coedges;
        let count = coedges.len();
        let in_run = |coedge: &CoedgeRecord| -> bool {
            across(coedge.edge_id, *strip_id).is_some_and(|other| other == ring_id || pieces.contains(&other))
        };
        let first = (0..count).find(|index| in_run(&coedges[*index]) && !in_run(&coedges[(index + count - 1) % count]))?;
        let run: Vec<usize> = (0..count)
            .map(|step| (first + step) % count)
            .take_while(|index| in_run(&coedges[*index]))
            .collect();
        if run.len() != coedges.iter().filter(|coedge| in_run(coedge)).count() || run.len() < 3 {
            return None;
        }
        let is_rim = |coedge: &CoedgeRecord| across(coedge.edge_id, *strip_id).is_some_and(|other| pieces.contains(&other));
        let (head, tail) = (&coedges[run[0]], &coedges[*run.last()?]);
        if !is_rim(head) || !is_rim(tail) {
            return None;
        }
        let from = traversed(head, &edges)?.0;
        let to = traversed(tail, &edges)?.1;
        let vertex = |id: u64| solid.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point);
        let (a, b) = (vertex(from)?, vertex(to)?);
        // Every rim on the run must lie on the straight line it heals to.
        let direction = b.sub(a).normalized().ok()?;
        for index in &run {
            let coedge = &coedges[*index];
            if !is_rim(coedge) {
                continue;
            }
            let edge = edges[&coedge.edge_id];
            for point in [edge.curve.evaluate(edge.t0).ok()?, edge.curve.evaluate(edge.t1).ok()?] {
                let delta = point.sub(a);
                if delta.sub(direction.scale(delta.dot(direction))).length() > tolerance {
                    return None;
                }
            }
            dropped_edges.insert(coedge.edge_id);
        }
        let curve = make_line(a, b).ok()?;
        let [t0, t1] = curve.domain().ok()?;
        let edge = EdgeRecord {
            id: alloc(),
            curve,
            t0,
            t1,
            start_vertex_id: from,
            end_vertex_id: to,
            degenerate: false,
            name: None,
        };
        let pcurve = build_pcurve_on_surface_range(&strip.surface, &edge.curve, t0, t1, true, (solid_model_scale(solid) * 1e-7).max(1e-9)).ok()?;
        let (shell, position) = find_face(&result, *strip_id)?;
        let loop_record = &mut result.shells[shell].faces[position].loops[0];
        let mut rebuilt = vec![CoedgeRecord { id: alloc(), edge_id: edge.id, forward: true, pcurve }];
        let last = *run.last()?;
        for step in 1..=(count - run.len()) {
            rebuilt.push(loop_record.coedges[(last + step) % count].clone());
        }
        loop_record.coedges = rebuilt;
        outer.push((edge.id, from, to));
        new_edges.push(edge);
    }

    // The planar pieces become one face: the new edges chain into its outer
    // loop, opposite each strip's sense, and the ring's circuit is its hole.
    let mut outer_chain: Vec<(u64, u64, u64)> = Vec::with_capacity(outer.len());
    let (first_edge, first_from, first_to) = *outer.first()?;
    // Each strip walked its edge from → to, so the plane walks to → from.
    outer_chain.push((first_edge, first_to, first_from));
    while outer_chain.len() < outer.len() {
        let tail = outer_chain.last()?.2;
        let next = outer.iter().find(|(_, _, to)| *to == tail)?;
        outer_chain.push((next.0, next.2, next.1));
    }
    if outer_chain.last()?.2 != outer_chain[0].1 {
        return None;
    }
    let host_id = pieces[0];
    let host_surface = face_of(host_id)?.surface.clone();
    let scale = solid_model_scale(solid);
    let by_id: HashMap<u64, &EdgeRecord> = new_edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut outer_coedges = Vec::with_capacity(outer_chain.len());
    for (edge_id, _, _) in &outer_chain {
        let edge = by_id[edge_id];
        let pcurve = build_pcurve_on_surface_range(&host_surface, &edge.curve, edge.t0, edge.t1, false, (scale * 1e-7).max(1e-9)).ok()?;
        outer_coedges.push(CoedgeRecord { id: alloc(), edge_id: *edge_id, forward: false, pcurve });
    }
    let placeholder = make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0)).ok()?;
    let mut hole = Vec::with_capacity(circuit.len());
    for coedge in circuit.iter().rev() {
        hole.push(CoedgeRecord {
            id: alloc(),
            edge_id: coedge.edge_id,
            forward: !coedge.forward,
            pcurve: placeholder.clone(),
        });
    }
    let (shell, position) = find_face(&result, host_id)?;
    result.shells[shell].faces[position].loops = vec![
        LoopRecord { id: alloc(), coedges: outer_coedges },
        LoopRecord { id: alloc(), coedges: hole },
    ];
    result.shells[shell_index].faces.retain(|face| !pieces[1..].contains(&face.id));
    result.edges.retain(|edge| !dropped_edges.contains(&edge.id));
    result.edges.extend(new_edges);
    Some(result)
}

/// Take a ring blend that overran the strips beside it out of a selection: the
/// solid without the ring, and what is left to delete. `None` when no selected
/// face reads as one.
pub(super) fn reduce_ring_over_strips(
    solid: &BrepSolid,
    face_ids: &[u64],
    op: &str,
) -> Option<Result<(BrepSolid, Vec<u64>), KernelRefusal>> {
    if face_ids.len() < 2 {
        return None;
    }
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    for ring_id in face_ids {
        let Some(reduced) = ring_over_strips(solid, &selected, *ring_id) else {
            continue;
        };
        let band = classify_closed_band(&reduced, &[*ring_id])?;
        let remaining: Vec<u64> = face_ids.iter().copied().filter(|id| id != ring_id).collect();
        return Some(heal_closed_band(&reduced, &band, op).map(|healed| (healed, remaining)));
    }
    None
}
