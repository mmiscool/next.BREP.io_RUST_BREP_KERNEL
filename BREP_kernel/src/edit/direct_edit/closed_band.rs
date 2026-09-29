//! Deleting a CLOSED BAND of faces — the multi-face analogue of the closed
//! transition strip ([`heal_closed_transition`](super::closed_heal)).
//!
//! A closed strip is one face whose two rims are closed edges, each belonging
//! to one neighbour; deleting it re-intersects the two neighbours in the rim
//! they would have met in. The same question arrives with MANY faces as soon
//! as the ring between the two neighbours is not one surface: a hexagonal nut
//! pocket sunk into a plate that a round bore passes through. The pocket's
//! walls, steps and floor sit between exactly two survivors — the plate face
//! the hexagon opens through and the bore that continues below the floor —
//! and deleting them means the bore runs on up to the plate.
//!
//! The patch cap in `delete_faces.rs` does not answer that. Its free boundary
//! consumes the plate's hexagonal hole loop whole, but on the bore it only
//! CUTS the loop — a periodic wall's loop carries its seam and its far rim as
//! well — and the runs that loop keeps do not splice into anything. That is not
//! a gap the cap misses; the opening is not there to cap: the bore's carrier
//! passes straight through the region the band enclosed.
//!
//! ONE face asks it too, whenever its rims are not single closed edges. The
//! closed strip reads exactly `[seam+, rim_a, seam-, rim_b]` and refuses every
//! other boundary by its coedge count, so a strip whose rim something CUT —
//! an import's marched mouth, a boolean, an imprint — never reached a heal at
//! all, though it is the same strip between the same two survivors. Here a rim
//! is a run of edges by construction, so the one-face case needs nothing of its
//! own beyond being let in (`one_face_the_closed_strip_cannot_read`).
//!
//! ## The gate
//!
//! Structural, decided before any geometry is touched:
//!
//! * the band is one edge-connected piece of one shell, every edge it uses is
//!   manifold, and it is an ANNULUS (`V - E + F - H = 0`);
//! * its free boundary belongs to exactly TWO survivor faces, and on each it is
//!   ONE closed run of one loop — a whole hole loop, or a closed stretch of a
//!   loop that goes on to carry a seam.
//!
//! ## The heal
//!
//! Exactly the closed strip's, stated for a run of edges instead of one: both
//! carriers are extended over the band, re-intersected in closed form, the
//! rim nearest the band is re-origined on the curved neighbour's seam, and in
//! each neighbour the run collapses to one coedge on that rim. The run's start
//! vertex is where a periodic neighbour's seam meets it, so the straight edges
//! that ended there grow to the new rim, as a strip's seam meridians do. The
//! band, its edges and every vertex only they used go. Genus is left alone and
//! verified by the Euler count, not by `validate()`'s parity check.

use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// How densely each boundary edge of the band is sampled for the carrier
/// extension and the branch pick — the closed strip's own figure.
const RIM_EDGE_SAMPLES: usize = 8;

/// How densely a candidate re-intersection is sampled when scoring it against
/// the band.
const BRANCH_SAMPLES: usize = 32;

/// One of the band's two free-boundary circuits, seen from the survivor that
/// owns it.
pub(super) struct BandRim {
    /// The survivor face.
    pub(super) neighbour: u64,
    /// The loop of that face the run lives in.
    pub(super) loop_index: usize,
    /// Coedge positions of the run in that loop, in traversal order.
    pub(super) run: Vec<usize>,
    /// Traversal-start (= end) vertex of the run.
    pub(super) start_vertex: u64,
}

/// A selection the gate read as a closed band between two survivors.
pub(super) struct ClosedBand {
    pub(super) shell_index: usize,
    pub(super) faces: HashSet<u64>,
    /// Every edge the band's faces use: interior edges and both circuits.
    pub(super) edges: HashSet<u64>,
    pub(super) rims: [BandRim; 2],
}

fn traversal_ends(coedge: &CoedgeRecord, edges: &HashMap<u64, &EdgeRecord>) -> Option<(u64, u64)> {
    let edge = edges.get(&coedge.edge_id)?;
    Some(if coedge.forward {
        (edge.start_vertex_id, edge.end_vertex_id)
    } else {
        (edge.end_vertex_id, edge.start_vertex_id)
    })
}

/// Is this ONE face a closed strip [`heal_closed_transition`](super::closed_heal)
/// cannot read?
///
/// That lane reads exactly one shape: a single loop of FOUR coedges,
/// `[seam+, rim_a, seam-, rim_b]`, so each rim is one closed edge. Every other
/// boundary it refuses outright ("only 4-sided transition faces"), whatever the
/// strip's geometry is — and a rim arrives cut in two whenever something
/// crossed it: an import's marched mouth, a boolean, an imprint. The strip is
/// the same strip and the heal is the same heal, stated for a RUN of edges,
/// which is this lane.
///
/// Requiring ONE loop keeps a face with a hole out: the annulus count below
/// would read that hole as the band's second circuit, and a face with a hole is
/// the patch lane's question or the chain's, both of which are asked first.
fn one_face_the_closed_strip_cannot_read(solid: &BrepSolid, face_id: u64) -> bool {
    let Some((shell, position)) = find_face(solid, face_id) else {
        return false;
    };
    let face = &solid.shells[shell].faces[position];
    face.loops.len() == 1 && face.loops[0].coedges.len() != 4
}

/// Does this selection read as a closed band between exactly two survivors?
/// Structural only — the geometry is gated in [`heal_closed_band`].
pub(super) fn classify_closed_band(solid: &BrepSolid, face_ids: &[u64]) -> Option<ClosedBand> {
    let faces: HashSet<u64> = face_ids.iter().copied().collect();
    // One face is the closed strip's own lane — as long as the closed strip
    // can read it (`one_face_the_closed_strip_cannot_read`). Everything below
    // is the gate the multi-face selection has always met, so a one-face
    // selection that is not a band still fails it: a 4-sided chamfer never
    // reaches here, and a plain face counts `V - E + F - H = 1`, not 0.
    if faces.is_empty()
        || (faces.len() < 2 && !one_face_the_closed_strip_cannot_read(solid, face_ids[0]))
    {
        return None;
    }
    let mut shells = face_ids
        .iter()
        .map(|face_id| find_face(solid, *face_id).map(|(shell, _)| shell));
    let shell_index = shells.next()??;
    for shell in shells {
        if shell? != shell_index {
            return None;
        }
    }

    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    // (selected uses, kept uses) per edge, and the survivor that keeps it.
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
        let edge = edges.get(edge_id)?;
        // A pole inside the band is a cell the annulus count below does not
        // read; a non-manifold edge has no "other side".
        if edge.degenerate || selected + kept != 2 {
            return None;
        }
    }

    // --- One connected piece -----------------------------------------------
    let mut reached: HashSet<u64> = HashSet::default();
    let mut stack = vec![face_ids[0]];
    reached.insert(face_ids[0]);
    while let Some(face_id) = stack.pop() {
        let (shell, position) = find_face(solid, face_id)?;
        for coedge in solid.shells[shell].faces[position]
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
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

    // --- An annulus ----------------------------------------------------------
    let vertices: HashSet<u64> = band_edges
        .iter()
        .flat_map(|edge_id| {
            let edge = edges[edge_id];
            [edge.start_vertex_id, edge.end_vertex_id]
        })
        .collect();
    let holes: usize = face_ids
        .iter()
        .filter_map(|face_id| find_face(solid, *face_id))
        .map(|(shell, position)| solid.shells[shell].faces[position].loops.len() - 1)
        .sum();
    let chi = vertices.len() as i64 - band_edges.len() as i64 + faces.len() as i64 - holes as i64;
    if chi != 0 {
        return None;
    }

    // --- Two survivors, one closed run each ----------------------------------
    let mut survivors: Vec<u64> = band_edges
        .iter()
        .filter(|edge_id| uses[*edge_id].1 == 1)
        .map(|edge_id| keeper[edge_id])
        .collect();
    survivors.sort_unstable();
    survivors.dedup();
    if survivors.len() != 2 {
        return None;
    }
    let mut rims: Vec<BandRim> = Vec::with_capacity(2);
    for neighbour in survivors {
        let (shell, position) = find_face(solid, neighbour)?;
        if shell != shell_index {
            return None;
        }
        let face = &solid.shells[shell].faces[position];
        let mut found: Option<BandRim> = None;
        for (loop_index, loop_record) in face.loops.iter().enumerate() {
            let flags: Vec<bool> = loop_record
                .coedges
                .iter()
                .map(|coedge| band_edges.contains(&coedge.edge_id))
                .collect();
            if !flags.iter().any(|flag| *flag) {
                continue;
            }
            if found.is_some() {
                // The band meets this survivor along two loops.
                return None;
            }
            let count = flags.len();
            let run: Vec<usize> = if flags.iter().all(|flag| *flag) {
                (0..count).collect()
            } else {
                let starts: Vec<usize> = (0..count)
                    .filter(|index| flags[*index] && !flags[(index + count - 1) % count])
                    .collect();
                if starts.len() != 1 {
                    return None;
                }
                (0..count)
                    .map(|step| (starts[0] + step) % count)
                    .take_while(|index| flags[*index])
                    .collect()
            };
            let first = traversal_ends(&loop_record.coedges[run[0]], &edges)?;
            let last = traversal_ends(&loop_record.coedges[*run.last()?], &edges)?;
            if first.0 != last.1 {
                // The run stops somewhere: a strip's gap, not a closed rim.
                return None;
            }
            found = Some(BandRim {
                neighbour,
                loop_index,
                run,
                start_vertex: first.0,
            });
        }
        rims.push(found?);
    }
    let second = rims.pop()?;
    let first = rims.pop()?;
    Some(ClosedBand {
        shell_index,
        faces,
        edges: band_edges,
        rims: [first, second],
    })
}

/// Points along a rim run, in the run's traversal order.
fn run_samples(
    solid: &BrepSolid,
    rim: &BandRim,
    edges: &HashMap<u64, EdgeRecord>,
    op: &str,
) -> Result<Vec<Vec3>, KernelRefusal> {
    let (shell, position) = find_face(solid, rim.neighbour).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "face",
            format!("{op}: missing face {}", rim.neighbour),
        )
    })?;
    let loop_record = &solid.shells[shell].faces[position].loops[rim.loop_index];
    let mut points = Vec::with_capacity(rim.run.len() * RIM_EDGE_SAMPLES);
    for index in &rim.run {
        let coedge = &loop_record.coedges[*index];
        let edge = edges.get(&coedge.edge_id).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "edge",
                format!("{op}: missing edge {}", coedge.edge_id),
            )
        })?;
        for step in 0..RIM_EDGE_SAMPLES {
            let fraction = step as f64 / RIM_EDGE_SAMPLES as f64;
            let fraction = if coedge.forward { fraction } else { 1.0 - fraction };
            points.push(
                edge.curve
                    .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)
                    .or_refuse(KernelStage::Classify, "evaluate")?,
            );
        }
    }
    Ok(points)
}

/// Twice the vector area of a closed polygon: its direction is the sense the
/// polygon turns in, whatever point it starts from.
pub(super) fn turning_sense(points: &[Vec3]) -> Vec3 {
    let mut centre = Vec3::default();
    for point in points {
        centre = centre.add(*point);
    }
    let centre = centre.scale(1.0 / points.len().max(1) as f64);
    let mut sum = Vec3::default();
    for (index, point) in points.iter().enumerate() {
        let next = points[(index + 1) % points.len()];
        sum = sum.add(point.sub(centre).cross(next.sub(centre)));
    }
    sum
}

/// Heal a [`ClosedBand`]: see the module docs.
pub(super) fn heal_closed_band(
    solid: &BrepSolid,
    band: &ClosedBand,
    op: &str,
) -> Result<BrepSolid, KernelRefusal> {
    if let Some(directory) = census_directory() {
        record_closed_band_census(&directory, solid, band);
    }
    let original = solid;
    let mut solid = solid.clone();
    let scale = solid_model_scale(&solid);
    // The closed strip's own tolerances.
    let tolerance = (scale * 1e-6).max(1e-9);
    let pcurve_tolerance = (scale * 1e-7).max(1e-9);
    let neighbour_ids = [band.rims[0].neighbour, band.rims[1].neighbour];
    // The text every refusal of this lane wears; the class is minted at each
    // site, since a deferral, an ambiguity and a consistency miss share it.
    let refusal = |reason: &str| {
        format!(
            "{op}: the selection is a closed band between faces {} and {}, and {reason} (deferred)",
            neighbour_ids[0], neighbour_ids[1]
        )
    };
    let missing_face = |id: u64| {
        KernelRefusal::internal(KernelStage::Collect, "face", format!("{op}: missing face {id}"))
    };

    // --- Both carriers analytic ------------------------------------------
    for &neighbour_id in &neighbour_ids {
        let (shell, position) = find_face(&solid, neighbour_id).ok_or_else(|| missing_face(neighbour_id))?;
        if solid.shells[shell].faces[position].surface.analytic().is_none() {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "fitted_neighbour",
                refusal(&format!(
                    "neighbour {neighbour_id} is a fitted surface this lane has no closed-form \
                 re-intersection for"
                )),
            ));
        }
    }

    let edge_map: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    let rim_points = [
        run_samples(&solid, &band.rims[0], &edge_map, op)?,
        run_samples(&solid, &band.rims[1], &edge_map, op)?,
    ];
    let band_points: Vec<Vec3> = rim_points.iter().flatten().copied().collect();
    let senses = [turning_sense(&rim_points[0]), turning_sense(&rim_points[1])];

    // --- Extend, then re-intersect -------------------------------------------
    for &neighbour_id in &neighbour_ids {
        extend_ruled_neighbour_over(&mut solid, neighbour_id, &band_points, tolerance)?;
    }
    let surface_of = |solid: &BrepSolid, id: u64| -> Result<NurbsSurface, KernelRefusal> {
        let (shell, position) = find_face(solid, id).ok_or_else(|| missing_face(id))?;
        Ok(solid.shells[shell].faces[position].surface.clone())
    };
    let surface_a = surface_of(&solid, neighbour_ids[0])?;
    let surface_b = surface_of(&solid, neighbour_ids[1])?;
    let curves = intersect_analytic_pair(&surface_a, &surface_b, tolerance).ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "analytic_pair",
            refusal("its neighbours are not a recognized analytic pair"),
        )
    })?;
    // The healed rim is the closed branch that runs through the band: score
    // each by its WORST sample's distance to the band's rims.
    let mut centre = Vec3::default();
    for point in &band_points {
        centre = centre.add(*point);
    }
    let centre = centre.scale(1.0 / band_points.len().max(1) as f64);
    let band_extent = 2.0
        * band_points
            .iter()
            .map(|point| point.sub(centre).length())
            .fold(0.0f64, f64::max);
    let mut best: Option<(f64, NurbsCurve)> = None;
    for curve in curves {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
        let evaluate = |t: f64| curve.evaluate(t).or_refuse(KernelStage::Classify, "evaluate");
        if evaluate(t0)?.sub(evaluate(t1)?).length() > tolerance {
            continue;
        }
        let mut worst = 0.0f64;
        for step in 0..BRANCH_SAMPLES {
            let point = evaluate(t0 + (t1 - t0) * step as f64 / BRANCH_SAMPLES as f64)?;
            let nearest = band_points
                .iter()
                .map(|band_point| band_point.sub(point).length())
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(nearest);
        }
        census_push("candidates", || serde_json::json!({ "worst_sampled": worst }));
        if best.as_ref().map_or(true, |(score, _)| worst < *score) {
            best = Some((worst, curve));
        }
    }
    let (score, new_curve) = best.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "closed_rim",
            refusal("its neighbours do not re-intersect in a closed rim"),
        )
    })?;
    if score > band_extent {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "rim_beyond_band",
            refusal(&format!(
                "its neighbours' nearest closed re-intersection passes {score:.6} from the band, \
             beyond the band's own extent {band_extent:.6}"
            )),
        ));
    }

    // --- Re-origin the rim on the curved neighbour's seam ---------------------
    // The run's start vertex on a periodic neighbour is where its seam edge
    // meets the band, so that is where the rim has to start.
    let surfaces = [surface_a, surface_b];
    let curved = [0usize, 1].map(|index| {
        !matches!(surfaces[index].analytic(), Some(AnalyticSurface::Plane { .. }))
    });
    let seam_rim = (0..2).find(|index| curved[*index]).ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "no_curved_neighbour",
            refusal("neither neighbour is curved"),
        )
    })?;
    let seam = solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == band.rims[seam_rim].start_vertex)
        .map(|vertex| vertex.point)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "vertex",
                format!("{op}: missing vertex {}", band.rims[seam_rim].start_vertex),
            )
        })?;
    let new_curve = align_closed_rim_origin(&new_curve, seam, tolerance, op)?;
    let [nt0, nt1] = new_curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let rim_at = |t: f64| new_curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate");
    let new_point = rim_at(nt0)?;
    let mut new_samples = Vec::with_capacity(BRANCH_SAMPLES);
    for step in 0..BRANCH_SAMPLES {
        let fraction = step as f64 / BRANCH_SAMPLES as f64;
        new_samples.push(rim_at(nt0 + (nt1 - nt0) * fraction)?);
    }
    let new_sense = turning_sense(&new_samples);

    // --- The rim's station on each curved neighbour, and where it crosses ----
    // --- that carrier's own periodic seam ------------------------------------
    // A face's seam EDGE need not sit on its carrier's seam: an imported bore
    // can carry its seam edge at u = ½ and cross u = 0 at a vertex that splits
    // each rim in two. A trim pcurve cannot jump across the branch cut in the
    // middle of a coedge, so the healed rim is split there as well.
    let mut stations: [Option<f64>; 2] = [None, None];
    let mut splits: Vec<f64> = Vec::new();
    for index in 0..2 {
        if !curved[index] {
            continue;
        }
        let surface = &surfaces[index];
        let neighbour = band.rims[index].neighbour;
        let station = match surface.analytic() {
            Some(AnalyticSurface::RuledRevolution { frame, height, .. }) => {
                let axial = new_point.sub(frame.origin).dot(frame.axis);
                if !(-tolerance..=height + tolerance).contains(&axial) {
                    return Err(KernelRefusal::internal(
                        KernelStage::Sew,
                        "rim_escapes_carrier",
                        refusal(&format!(
                            "the healed rim escapes neighbour {neighbour}'s extended carrier \
                         (station {axial:.6} of {height:.6})"
                        )),
                    ));
                }
                Some((axial / height).clamp(0.0, 1.0))
            }
            _ => rim_iso_station(surface, &new_curve, tolerance),
        };
        let v_new = station.ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Classify,
                "rim_not_iso",
                refusal(&format!(
                    "the healed rim is not a `v = const` iso-curve of neighbour {neighbour}'s carrier"
                )),
            )
        })?;
        stations[index] = Some(v_new);
        let [u_low, u_high] = surface.domain_u().or_refuse(KernelStage::Classify, "domain")?;
        let eps = 1e-6 * (u_high - u_low).abs().max(1e-12);
        let start_u = crate::project_point_to_surface(surface, new_point)
            .or_refuse(KernelStage::Classify, "project")?
            .u;
        if (start_u - u_low).abs() <= eps || (start_u - u_high).abs() <= eps {
            continue;
        }
        if !surface.closed_directions().or_refuse(KernelStage::Classify, "domain")?.0 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "neighbour_not_closed",
                refusal(&format!(
                    "neighbour {neighbour} is not closed around the rim it would carry"
                )),
            ));
        }
        let crossing = surface.evaluate(u_low, v_new).or_refuse(KernelStage::Classify, "evaluate")?;
        let hit = project_point_to_curve(&new_curve, crossing).or_refuse(KernelStage::Classify, "project")?;
        if hit.distance > tolerance {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "rim_misses_seam",
                refusal(&format!(
                    "the healed rim misses neighbour {neighbour}'s seam by {:.3e}",
                    hit.distance
                )),
            ));
        }
        splits.push(hit.u);
    }
    splits.sort_by(f64::total_cmp);
    let mut bounds = vec![nt0];
    for split in splits {
        let point = rim_at(split)?;
        let previous = rim_at(*bounds.last().expect("seeded"))?;
        if point.sub(previous).length() > tolerance && point.sub(new_point).length() > tolerance {
            bounds.push(split);
        }
    }
    bounds.push(nt1);
    let pieces = bounds.len() - 1;

    let mut next_id = max_topology_id(&solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };
    let new_vertex_id = alloc();
    let mut piece_vertices = vec![new_vertex_id];
    solid.vertices.push(VertexRecord {
        id: new_vertex_id,
        point: new_point,
    });
    for bound in &bounds[1..pieces] {
        let id = alloc();
        solid.vertices.push(VertexRecord {
            id,
            point: rim_at(*bound)?,
        });
        piece_vertices.push(id);
    }
    // Each piece is its own curve over its whole domain, so a planar
    // neighbour's pcurve is the exact affine preimage rather than a fit.
    let mut piece_curves: Vec<NurbsCurve> = Vec::with_capacity(pieces);
    let mut rest = new_curve.clone();
    for bound in &bounds[1..pieces] {
        let (head, tail) = rest.split(*bound).or_refuse(KernelStage::Sew, "split")?;
        piece_curves.push(head);
        rest = tail;
    }
    piece_curves.push(rest);
    let mut new_edges: Vec<u64> = Vec::with_capacity(pieces);
    for piece in 0..pieces {
        let id = alloc();
        let [t0, t1] = piece_curves[piece].domain().or_refuse(KernelStage::Sew, "domain")?;
        solid.edges.push(EdgeRecord {
            id,
            curve: piece_curves[piece].clone(),
            t0,
            t1,
            start_vertex_id: piece_vertices[piece],
            end_vertex_id: piece_vertices[(piece + 1) % pieces],
            degenerate: false,
            name: None,
        });
        new_edges.push(id);
    }

    // --- Each run collapses onto the rim --------------------------------------
    for (index, rim) in band.rims.iter().enumerate() {
        let sense = senses[index].dot(new_sense);
        if sense.abs() <= 1e-6 * senses[index].length() * new_sense.length() {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "rim_sense",
                refusal(&format!(
                    "the rim it leaves in neighbour {} does not turn about the healed rim",
                    rim.neighbour
                )),
            ));
        }
        let forward = sense > 0.0;
        let order: Vec<usize> = if forward {
            (0..pieces).collect()
        } else {
            (0..pieces).rev().collect()
        };
        let surface = &surfaces[index];
        let mut coedges: Vec<CoedgeRecord> = Vec::with_capacity(pieces);
        match stations[index] {
            None => {
                for piece in order {
                    let mut pcurve = build_pcurve_on_surface(surface, &piece_curves[piece])
                        .or_refuse(KernelStage::Sew, "pcurve_fit")?;
                    if !forward {
                        pcurve = pcurve.reversed().or_refuse(KernelStage::Sew, "reverse")?;
                    }
                    // The rim has to lie inside the plane's patch, or the fit
                    // has clamped onto its border.
                    let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
                    for step in 0..=RIM_EDGE_SAMPLES {
                        let fraction = step as f64 / RIM_EDGE_SAMPLES as f64;
                        let uv = pcurve
                            .evaluate(q0 + (q1 - q0) * fraction)
                            .or_refuse(KernelStage::Sew, "evaluate")?;
                        let on_plane = surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate")?;
                        if project_point_to_curve(&new_curve, on_plane)
                            .or_refuse(KernelStage::Sew, "project")?
                            .distance
                            > tolerance
                        {
                            return Err(KernelRefusal::unsupported(
                                KernelStage::Sew,
                                "rim_off_planar_patch",
                                refusal(&format!(
                                    "the healed rim leaves the patch of planar neighbour {}",
                                    rim.neighbour
                                )),
                            ));
                        }
                    }
                    coedges.push(CoedgeRecord {
                        id: alloc(),
                        edge_id: new_edges[piece],
                        forward,
                        pcurve,
                    });
                }
            }
            Some(v_new) => {
                let [u_low, u_high] = surface.domain_u().or_refuse(KernelStage::Sew, "domain")?;
                let span = (u_high - u_low).abs().max(1e-12);
                let eps = 1e-6 * span;
                let u_of = |point: Vec3| -> Result<f64, KernelRefusal> {
                    Ok(crate::project_point_to_surface(surface, point)
                        .or_refuse(KernelStage::Sew, "project")?
                        .u)
                };
                // Which way the traversal runs in the carrier's own u,
                // measured an eighth of a turn along it.
                let eighth = (nt1 - nt0) / 8.0;
                let ahead = rim_at(if forward { nt0 + eighth } else { nt1 - eighth })?;
                let mut u = u_of(new_point)?;
                let ascending = (u_of(ahead)? - u + 0.5 * span).rem_euclid(span) > 0.5 * span;
                for piece in order {
                    let end_parameter = if forward { bounds[piece + 1] } else { bounds[piece] };
                    if ascending && (u - u_high).abs() <= eps {
                        u = u_low;
                    }
                    if !ascending && (u - u_low).abs() <= eps {
                        u = u_high;
                    }
                    let end_raw = u_of(rim_at(end_parameter)?)?;
                    let delta = if ascending {
                        let delta = (end_raw - u).rem_euclid(span);
                        if delta <= eps { span } else { delta }
                    } else {
                        let delta = (u - end_raw).rem_euclid(span);
                        -(if delta <= eps { span } else { delta })
                    };
                    let end_u = u + delta;
                    if end_u < u_low - eps || end_u > u_high + eps {
                        return Err(KernelRefusal::internal(
                            KernelStage::Sew,
                            "seam_inside_coedge",
                            refusal(&format!(
                                "the healed rim crosses neighbour {}'s seam inside one coedge",
                                rim.neighbour
                            )),
                        ));
                    }
                    coedges.push(CoedgeRecord {
                        id: alloc(),
                        edge_id: new_edges[piece],
                        forward,
                        pcurve: make_line(Vec3::new(u, v_new, 0.0), Vec3::new(end_u, v_new, 0.0))
                            .or_refuse(KernelStage::Sew, "make_line")?,
                    });
                    u = end_u;
                }
            }
        }
        let (shell, position) = find_face(&solid, rim.neighbour).ok_or_else(|| missing_face(rim.neighbour))?;
        let loop_record = &mut solid.shells[shell].faces[position].loops[rim.loop_index];
        let count = loop_record.coedges.len();
        let last = *rim.run.last().expect("a run has a coedge");
        for step in 1..=(count - rim.run.len()) {
            coedges.push(loop_record.coedges[(last + step) % count].clone());
        }
        loop_record.coedges = coedges;
    }

    // --- The edges that ended on a run's start vertex grow to the rim ---------
    let run_vertices: HashSet<u64> = band
        .edges
        .iter()
        .filter_map(|edge_id| edge_map.get(edge_id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let collapsed: HashSet<u64> = band
        .rims
        .iter()
        .zip(curved)
        .filter(|(_, curved)| *curved)
        .map(|(rim, _)| rim.start_vertex)
        .collect();
    let mut extended: HashMap<u64, (bool, bool)> = HashMap::default();
    let mut refit: HashSet<u64> = HashSet::default();
    for edge in &mut solid.edges {
        if band.edges.contains(&edge.id) || new_edges.contains(&edge.id) {
            continue;
        }
        let start_hit = run_vertices.contains(&edge.start_vertex_id);
        let end_hit = run_vertices.contains(&edge.end_vertex_id);
        if !(start_hit || end_hit) {
            continue;
        }
        if (start_hit && !collapsed.contains(&edge.start_vertex_id))
            || (end_hit && !collapsed.contains(&edge.end_vertex_id))
        {
            return Err(KernelRefusal::unsupported(
                KernelStage::Sew,
                "edge_off_seam",
                refusal(&format!(
                    "edge {} meets the band away from its neighbours' seam",
                    edge.id
                )),
            ));
        }
        if start_hit {
            edge.start_vertex_id = new_vertex_id;
        }
        if end_hit {
            edge.end_vertex_id = new_vertex_id;
        }
        if start_hit && end_hit {
            return Err(KernelRefusal::unsupported(
                KernelStage::Sew,
                "edge_collapse",
                refusal(&format!("edge {} would collapse onto the rim", edge.id)),
            ));
        }
        if edge.curve.straight_segment(tolerance).is_some() {
            let redundant = edge.curve.control_points.len() > 2;
            let anchor = if start_hit {
                edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?
            } else {
                edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?
            };
            let (from, to) = if start_hit { (new_point, anchor) } else { (anchor, new_point) };
            edge.curve = make_line(from, to).or_refuse(KernelStage::Sew, "make_line")?;
            [edge.t0, edge.t1] = edge.curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            if redundant {
                refit.insert(edge.id);
            }
        } else {
            let projection =
                project_point_to_curve(&edge.curve, new_point).or_refuse(KernelStage::Sew, "project")?;
            if projection.distance > tolerance {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "curved_edge_short",
                    refusal(&format!(
                        "curved edge {} does not reach the healed rim (nearest point {:.3e} away)",
                        edge.id, projection.distance
                    )),
                ));
            }
            if start_hit {
                edge.t0 = projection.u;
            } else {
                edge.t1 = projection.u;
            }
            if edge.t1 <= edge.t0 {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "curved_edge_collapse",
                    refusal(&format!("curved edge {} would collapse", edge.id)),
                ));
            }
            refit.insert(edge.id);
        }
        extended.insert(edge.id, (start_hit, end_hit));
    }
    let edges_by_id: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    // A grown edge has to stay on every carrier it borders. Two curved
    // neighbours whose seams stand at different azimuths would pull one seam
    // off its own wall to reach the rim's single start vertex.
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if !extended.contains_key(&coedge.edge_id) {
                continue;
            }
            let edge = &edges_by_id[&coedge.edge_id];
            let middle = edge
                .curve
                .evaluate(0.5 * (edge.t0 + edge.t1))
                .or_refuse(KernelStage::Validate, "evaluate")?;
            let distance = crate::project_point_to_surface(&face.surface, middle)
                .or_refuse(KernelStage::Validate, "project")?
                .distance;
            if distance > tolerance {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Validate,
                    "grown_edge_off_face",
                    refusal(&format!(
                        "growing edge {} to the healed rim takes it {distance:.3e} off face {}",
                        edge.id, face.id
                    )),
                ));
            }
        }
    }
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            let ruled_station = match face.surface.analytic() {
                Some(AnalyticSurface::RuledRevolution { frame, height, .. }) => {
                    Some((new_point.sub(frame.origin).dot(frame.axis) / height).clamp(0.0, 1.0))
                }
                _ => None,
            };
            let mut touched: HashSet<u64> = HashSet::default();
            for coedge in face.loops.iter_mut().flat_map(|loop_record| &mut loop_record.coedges) {
                let Some(&(start_moved, end_moved)) = extended.get(&coedge.edge_id) else {
                    continue;
                };
                // A straight meridian on a ruled wall keeps its two-point
                // pcurve pinned to its seam branch; only the moved end's v
                // changes. Everything else is refit on the carrier.
                match ruled_station {
                    Some(v_new)
                        if !refit.contains(&coedge.edge_id)
                            && coedge.pcurve.degree == 1
                            && coedge.pcurve.control_points.len() == 2 =>
                    {
                        let start_index = if coedge.forward { 0 } else { 1 };
                        if start_moved {
                            let control = &mut coedge.pcurve.control_points[start_index];
                            control.y = v_new * control.w;
                        }
                        if end_moved {
                            let control = &mut coedge.pcurve.control_points[1 - start_index];
                            control.y = v_new * control.w;
                        }
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

    // --- Drop the band ----------------------------------------------------------
    solid.shells[band.shell_index]
        .faces
        .retain(|face| !band.faces.contains(&face.id));
    solid.edges.retain(|edge| !band.edges.contains(&edge.id));
    let used: HashSet<u64> = solid
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    solid.vertices.retain(|vertex| used.contains(&vertex.id));

    // Deleting an annulus and closing it with one rim leaves the handle count
    // alone. `validate()` only checks that parity, so ask the count itself.
    let before = super::delete_faces::euler_characteristic(original);
    let after = super::delete_faces::euler_characteristic(&solid);
    if before != after {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "euler",
            refusal(&format!(
                "closing it changes the Euler characteristic {before} -> {after}"
            )),
        ));
    }
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validation",
            format!("{op}: closed-band heal failed validation: {issues:?}"),
        ));
    }
    Ok(solid)
}
