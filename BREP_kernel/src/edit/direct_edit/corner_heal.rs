//! Deleting a whole CORNER BLEND — a vertex blend and the strips that meet on
//! it — as ONE question.
//!
//! Round the three edges that meet at a box corner and the fillet leaves four
//! faces: three cylindrical STRIPS, one per edge, and the spherical CORNER the
//! rolling ball leaves where they run together. Selecting all four and asking
//! for them gone is "un-round this corner", and it is the commonest thing a
//! user does to a fillet after making it.
//!
//! One face at a time it cannot be answered, and the refusal was honest. A
//! strip's four neighbours are its two walls, the face it runs out into, and —
//! at the corner end — the vertex blend. The heal a strip asks for is "extend
//! the two walls, re-intersect them, and clip the recovered sharp edge against
//! both end caps"; here the recovered edge is the box's own sharp edge, and it
//! passes the corner sphere at `r·√2` — it never meets it. So
//! [`heal_open_transition_mixed`](super::open_heal) reported exactly what it
//! found:
//!
//! ```text
//! delete_face_and_heal: no re-intersection branch of the extended carriers
//! spans the deleted strip — refusing rather than emitting an invalid solid
//! ```
//!
//! No chain order fixes that. Deleting one strip out of a rounded corner is
//! not an under-solved problem, it is an ILL-POSED one: the corner sphere is
//! tangent to the strip that is leaving and to the two that are staying, and
//! what bounds it once its neighbour is gone is not a question the input
//! answers. The set is the question, and the set has a closed form.
//!
//! ## The construction
//!
//! The group is read off its own topology, never guessed:
//!
//! * the CORNER is the selected face with three boundary edges, and each of
//!   those edges is shared with another selected face;
//! * each of those three is a STRIP: a four-sided face, four distinct edges,
//!   four distinct neighbours, exactly one of them the corner. Opposite the
//!   corner edge is the strip's far CAP; flanking it are the two WALLS;
//! * the three strips' walls must be three distinct carriers, each carrying
//!   two strips. That is what makes the group a corner rather than three
//!   fillets that happen to touch: the walls close a cycle.
//!
//! Then, for each strip, exactly the single-strip heal's own closed form: the
//! two walls re-intersect in the recovered sharp edge, and the far cap clips
//! one end of it. What replaces the corner sphere is the OTHER end — the
//! recovered edge meets the third wall (the one this strip does not use) in a
//! point, and all three strips must name the SAME point. That agreement is the
//! whole gate on the geometry: three lines, each solved from a different pair
//! of planes, meeting at one vertex to within the plane tolerance. Disagree
//! and the group is refused.
//!
//! The topology that follows is the single-strip heal repeated: the corner
//! sphere's three vertices collapse onto the recovered corner vertex, each
//! strip's two far vertices collapse onto its own recovered far corner, every
//! surviving edge that ended on a collapsed vertex is widened along its own
//! line, each wall swaps its two strip rims for the two sharp edges that now
//! meet on it, each cap drops the transverse blend edge it wore, and every
//! touched carrier is re-trimmed and its pcurves refit.
//!
//! ## Nothing is left for the coalescer
//!
//! `delete_faces_and_heal` finishes every lane through
//! `merge_curve_continuation_edges`, which collapses the tangent continuation
//! edges a heal leaves behind. This lane leaves none, and that is structural
//! rather than lucky: a wall's two strip rims are REPLACED by the two recovered
//! sharp edges rather than having new pieces appended beside them, and every
//! surviving side edge is WIDENED along its own line rather than continued by a
//! second edge. So there is no continuation pair to find. Measured on both the
//! plain cube and the reported shelled document: the coalesced result is
//! bit-for-bit the counts the heal produced (6/12/8 and 14/33/21 faces / edges
//! / vertices), and a further coalesce takes 0 edges and 0 vertices.
//!
//! ## Curved walls and caps
//!
//! When a wall or a cap is a cylinder, a cone, a sphere or a torus there is no
//! closed form, and the construction is the same one with the closed forms
//! replaced ([`heal_curved_corner_blend_group`]). The corner vertex is the three
//! walls' triple point and each far corner the triple point of its strip's
//! walls and cap, solved by Newton on the INFINITE analytic carriers
//! (`triple_point.rs`, which reports iterations, residual, conditioning and a
//! first-order position bound, and refuses a corner the model tolerance cannot
//! place). Each sharp edge is the piece of its walls' intersection between the
//! two, taken from `intersect_analytic_pair` and rebuilt as the exact arc when
//! it is circular. A curved carrier is KEPT: grown exactly along its axis where
//! the recovered edges overrun its patch, and only the new and moved edges get
//! new trims.
//!
//! More than one triple point exists — two planes and a cylinder meet twice —
//! so each root is checked to be the corner the blend was cut from: it lies
//! PAST every rim of the strips it ends (measured along the wall, away from the
//! wall's own face), the corner solve returns the same point from every
//! strip's own seed, and every other root on the walls' intersection inside the
//! group's reach that is also past those rims lies farther from the seed.
//! Finally every new or moved edge must lie on every face it bounds, and on a
//! curved face's patch: relocation widens or rebuilds edges without being told
//! what they lie on, and a strip whose end touches a fourth face moves one off.
//!
//! ## Scope
//!
//! Three strips meeting on one three-sided vertex blend, whose walls close a
//! cycle of three carriers: planes in closed form, and analytic walls and caps
//! through the triple solve. Refused by name: a FITTED wall or cap (no
//! closed-form re-intersection here, though the solver itself reads a patch), a
//! wall pair `intersect_analytic_pair` has no form for, a closed intersection
//! whose piece crosses its origin and is not a circle, and a strip end that
//! touches a fourth face. A four-way star corner is not taken (its corner face
//! is four-sided); a planar star heals through the blend network lane.

use super::*;
use crate::project_point_to_surface;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// One blend strip of a corner group, read off its own loop.
pub(super) struct CornerStrip {
    /// The strip face itself.
    face_id: u64,
    /// The strip's boundary edge ids, in loop order.
    boundary: [u64; 4],
    /// The face across each boundary edge, in the same order.
    neighbours: [u64; 4],
    /// Index into `boundary` of the edge shared with the corner face.
    corner_index: usize,
    /// The two indices flanking `corner_index` — the walls whose
    /// re-intersection is the recovered sharp edge.
    walls: [usize; 2],
    /// The index opposite `corner_index` — the face the strip runs out into.
    cap_index: usize,
}

impl CornerStrip {
    pub(super) fn wall_ids(&self) -> [u64; 2] {
        [self.neighbours[self.walls[0]], self.neighbours[self.walls[1]]]
    }

    pub(super) fn cap_id(&self) -> u64 {
        self.neighbours[self.cap_index]
    }
}

/// A selection that reads as one vertex blend plus every strip that meets on
/// it.
pub(super) struct CornerBlendGroup {
    /// The shell the group lives in (all four faces share one).
    shell_index: usize,
    /// The vertex blend.
    corner_id: u64,
    /// The corner face's boundary edges.
    corner_edges: Vec<u64>,
    /// The strips, in the corner's own loop order.
    pub(super) strips: Vec<CornerStrip>,
}

/// Read one strip of a corner group off its loop, or decline.
fn read_corner_strip(
    solid: &BrepSolid,
    strip_id: u64,
    corner_id: u64,
    selected: &HashSet<u64>,
) -> Option<CornerStrip> {
    let (shell_index, face_index) = find_face(solid, strip_id)?;
    let face = &solid.shells[shell_index].faces[face_index];
    if face.loops.len() != 1 || face.loops[0].coedges.len() != 4 {
        return None;
    }
    let mut boundary = [0u64; 4];
    for (index, coedge) in face.loops[0].coedges.iter().enumerate() {
        boundary[index] = coedge.edge_id;
    }
    if boundary.iter().copied().collect::<HashSet<u64>>().len() != 4 {
        return None;
    }
    let mut neighbours = [0u64; 4];
    for (index, &edge_id) in boundary.iter().enumerate() {
        neighbours[index] = other_face_of_edge(solid, edge_id, strip_id).ok()?;
    }
    if neighbours.iter().copied().collect::<HashSet<u64>>().len() != 4 {
        return None;
    }
    let corner_index = neighbours.iter().position(|id| *id == corner_id)?;
    let walls = [(corner_index + 1) % 4, (corner_index + 3) % 4];
    let cap_index = (corner_index + 2) % 4;
    // Everything the strip leans on has to SURVIVE — a wall or a cap that is
    // itself selected is a bigger question than one corner.
    for index in [walls[0], walls[1], cap_index] {
        if selected.contains(&neighbours[index]) {
            return None;
        }
    }
    Some(CornerStrip {
        face_id: strip_id,
        boundary,
        neighbours,
        corner_index,
        walls,
        cap_index,
    })
}

/// Does this selection read as one vertex blend plus every strip that meets on
/// it? Structural only — the geometry is gated in
/// [`heal_corner_blend_group`].
pub(super) fn classify_corner_blend_group(
    solid: &BrepSolid,
    face_ids: &[u64],
) -> Option<CornerBlendGroup> {
    // One corner + three strips. A four-way star corner wears a four-sided
    // corner face, which the one-at-a-time chain already reads its own way.
    if face_ids.len() != 4 {
        return None;
    }
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    if selected.len() != 4 {
        return None;
    }
    for &corner_id in face_ids {
        let Some((shell_index, face_index)) = find_face(solid, corner_id) else {
            return None;
        };
        let corner = &solid.shells[shell_index].faces[face_index];
        if corner.loops.len() != 1 || corner.loops[0].coedges.len() != 3 {
            continue;
        }
        let corner_edges: Vec<u64> = corner.loops[0]
            .coedges
            .iter()
            .map(|coedge| coedge.edge_id)
            .collect();
        if corner_edges.iter().copied().collect::<HashSet<u64>>().len() != 3 {
            continue;
        }
        let mut strips: Vec<CornerStrip> = Vec::with_capacity(3);
        for &edge_id in &corner_edges {
            let Ok(strip_id) = other_face_of_edge(solid, edge_id, corner_id) else {
                break;
            };
            if !selected.contains(&strip_id) {
                break;
            }
            let Some(strip) = read_corner_strip(solid, strip_id, corner_id, &selected) else {
                break;
            };
            // Every face of the group lives in the corner's own shell.
            if find_face(solid, strip_id).map(|(shell, _)| shell) != Some(shell_index) {
                break;
            }
            strips.push(strip);
        }
        if strips.len() != 3 {
            continue;
        }
        if strips
            .iter()
            .map(|strip| strip.face_id)
            .collect::<HashSet<u64>>()
            .len()
            != 3
        {
            continue;
        }
        // The walls close a CYCLE: three carriers, each carrying two of the
        // three strips. Anything else is not one corner.
        let mut wall_uses: HashMap<u64, usize> = HashMap::default();
        for strip in &strips {
            for wall in strip.wall_ids() {
                *wall_uses.entry(wall).or_insert(0) += 1;
            }
        }
        if wall_uses.len() != 3 || wall_uses.values().any(|count| *count != 2) {
            continue;
        }
        // A cap that is also a wall would be re-trimmed against a boundary it
        // is simultaneously rewriting; that is a different question.
        if strips
            .iter()
            .any(|strip| wall_uses.contains_key(&strip.cap_id()))
        {
            continue;
        }
        return Some(CornerBlendGroup {
            shell_index,
            corner_id,
            corner_edges,
            strips,
        });
    }
    None
}

/// Re-derive a side edge from its two flanking PLANES.
///
/// `relocate_open_side_edge` widens along the stored curve, and rebuilds only a
/// two-control-point line; `rederive_side_edge_on_carriers` goes through
/// `intersect_analytic_pair`, which declines plane × plane by design (there is
/// no closed form there because there is one HERE). A shelled solid's rim weld
/// leaves its straight edges as degree-1 curves with three control points, so
/// both of those decline an edge that is a plain line between two planes. This
/// is that line: the flanking pair re-intersected exactly, re-trimmed from the
/// endpoint it keeps to the corner it now reaches.
pub(super) fn rederive_side_edge_on_planes(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    planes: &HashMap<u64, Plane>,
    start_target: Option<(u64, Vec3)>,
    end_target: Option<(u64, Vec3)>,
    plane_tolerance: f64,
    tolerance: f64,
) -> Result<Option<EdgeRecord>, KernelRefusal> {
    let mut flanking: Vec<u64> = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            if face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .any(|coedge| coedge.edge_id == edge.id)
            {
                flanking.push(face.id);
            }
        }
    }
    if flanking.len() != 2 {
        return Ok(None);
    }
    let (Some(plane_a), Some(plane_b)) = (planes.get(&flanking[0]), planes.get(&flanking[1])) else {
        return Ok(None);
    };
    let Some(line) = intersect_planes(plane_a, plane_b) else {
        return Ok(None);
    };
    let on_line = |point: Vec3| -> bool {
        let along = point.sub(line.point).dot(line.dir);
        point.sub(line.point.add(line.dir.scale(along))).length() <= plane_tolerance
    };
    let start_point = match start_target {
        Some((_, point)) => point,
        None => edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?,
    };
    let end_point = match end_target {
        Some((_, point)) => point,
        None => edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?,
    };
    if !on_line(start_point) || !on_line(end_point) {
        return Ok(None);
    }
    if start_point.sub(end_point).length() <= tolerance {
        return Ok(None);
    }
    let mut record = edge.clone();
    record.curve = make_line(start_point, end_point).or_refuse(KernelStage::Sew, "make_line")?;
    record.t0 = 0.0;
    record.t1 = 1.0;
    if let Some((vertex, _)) = start_target {
        record.start_vertex_id = vertex;
    }
    if let Some((vertex, _)) = end_target {
        record.end_vertex_id = vertex;
    }
    Ok(Some(record))
}

/// Every edge and vertex a corner group wears, and its region gate: the
/// centroid of its vertices and three times their spread about it.
fn corner_group_region(
    solid: &BrepSolid,
    group: &CornerBlendGroup,
    tolerance: f64,
    op: &str,
) -> Result<(HashSet<u64>, Vec<u64>, Vec3, f64), KernelRefusal> {
    let group_edges: HashSet<u64> = group
        .corner_edges
        .iter()
        .copied()
        .chain(
            group
                .strips
                .iter()
                .flat_map(|strip| strip.boundary.iter().copied()),
        )
        .collect();
    let mut group_vertices: Vec<u64> = Vec::new();
    for edge in &solid.edges {
        if group_edges.contains(&edge.id) {
            group_vertices.push(edge.start_vertex_id);
            group_vertices.push(edge.end_vertex_id);
        }
    }
    group_vertices.sort_unstable();
    group_vertices.dedup();
    if group_vertices.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Collect,
            "group_vertices",
            format!("{op}: corner blend group has no vertices"),
        ));
    }
    let mut centre = Vec3::default();
    for &vertex_id in &group_vertices {
        centre = centre.add(edge_point(solid, vertex_id)?);
    }
    centre = centre.scale(1.0 / group_vertices.len() as f64);
    let mut reach = 0.0f64;
    for &vertex_id in &group_vertices {
        reach = reach.max(edge_point(solid, vertex_id)?.sub(centre).length());
    }
    Ok((group_edges, group_vertices, centre, reach * 3.0 + tolerance))
}

/// The recovered geometry of one strip: the sharp edge its two walls make, and
/// where its far cap clips it.
struct StripPlan {
    /// The recovered far corner (sharp edge × the strip's cap plane).
    far_point: Vec3,
    /// The recovered corner vertex, read from this strip's own line.
    corner_point: Vec3,
}

/// Heal a corner blend group: the strips' walls re-intersect into three sharp
/// edges meeting at one recovered vertex, and the vertex blend goes with them.
pub(super) fn heal_corner_blend_group(
    solid: &BrepSolid,
    group: &CornerBlendGroup,
    op: &str,
) -> Result<BrepSolid, KernelRefusal> {
    let mut solid = solid.clone();
    let scale = solid_model_scale(&solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let missing = |what: &'static str, id: u64| {
        KernelRefusal::internal(KernelStage::Collect, what, format!("{op}: missing {what} {id}"))
    };

    // --- Carriers: every wall and every cap must be planar in this slice ---
    let mut planes: HashMap<u64, Plane> = HashMap::default();
    for strip in &group.strips {
        for neighbour_id in strip
            .wall_ids()
            .into_iter()
            .chain(std::iter::once(strip.cap_id()))
        {
            if planes.contains_key(&neighbour_id) {
                continue;
            }
            let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing("neighbour", neighbour_id))?;
            let Ok(plane) = plane_of_surface(
                &solid.shells[ns].faces[nf].surface,
                plane_tolerance,
                op,
            ) else {
                // A curved wall or cap has no closed form here: the general
                // triple-surface solve takes the whole group.
                return heal_curved_corner_blend_group(&solid, group, op).map(|(healed, _)| healed);
            };
            planes.insert(neighbour_id, plane);
        }
    }
    let wall_ids: HashSet<u64> = group
        .strips
        .iter()
        .flat_map(|strip| strip.wall_ids())
        .collect();

    // --- The group's own region gate (centre + reach), from its vertices ---
    let (group_edges, group_vertices, centre, reach) =
        corner_group_region(&solid, group, tolerance, op)?;

    // --- Each strip's recovered edge, in closed form -----------------------
    let mut plans: Vec<StripPlan> = Vec::with_capacity(3);
    for strip in &group.strips {
        let [wall_a, wall_b] = strip.wall_ids();
        let line = intersect_planes(&planes[&wall_a], &planes[&wall_b]).ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Classify,
                "parallel_walls",
                format!(
                    "{op}: the walls flanking blend face {} are parallel — they cannot \
                 re-intersect into a sharp edge",
                    strip.face_id
                ),
            )
        })?;
        let far_point = intersect_line_plane(&line, &planes[&strip.cap_id()])
            .filter(|point| point.sub(centre).length() <= reach)
            .ok_or_else(|| {
                KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "cap_not_crossed",
                    format!(
                        "{op}: the sharp edge recovered for blend face {} does not cross the \
                     face it runs out into",
                        strip.face_id
                    ),
                )
            })?;
        // The corner vertex is where this strip's recovered edge meets the
        // wall it does NOT use — the third carrier of the cycle.
        let third = wall_ids
            .iter()
            .copied()
            .find(|id| *id != wall_a && *id != wall_b)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "wall_cycle",
                    format!("{op}: the corner's walls do not close a cycle"),
                )
            })?;
        let corner_point = intersect_line_plane(&line, &planes[&third])
            .filter(|point| point.sub(centre).length() <= reach)
            .ok_or_else(|| {
                KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "third_wall_not_reached",
                    format!(
                        "{op}: the sharp edge recovered for blend face {} never reaches the \
                     corner's third wall",
                        strip.face_id
                    ),
                )
            })?;
        plans.push(StripPlan {
            far_point,
            corner_point,
        });
    }

    // The three lines were each solved from a different pair of planes; they
    // have to name ONE vertex. This is the gate on the geometry.
    let mut corner_point = Vec3::default();
    for plan in &plans {
        corner_point = corner_point.add(plan.corner_point);
    }
    corner_point = corner_point.scale(1.0 / plans.len() as f64);
    for plan in &plans {
        let drift = plan.corner_point.sub(corner_point).length();
        if drift > plane_tolerance {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "corner_drift",
                format!(
                    "{op}: the blend strips' recovered edges miss each other by {drift:.6} — \
                 the selection is not one corner"
                ),
            ));
        }
    }
    for (index, plan) in plans.iter().enumerate() {
        if plan.far_point.sub(corner_point).length() <= tolerance {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "edge_collapse",
                format!(
                    "{op}: the sharp edge recovered for blend face {} collapses to the corner \
                 vertex",
                    group.strips[index].face_id
                ),
            ));
        }
    }

    // --- New vertices and the three new sharp edges ------------------------
    let mut next_id = max_topology_id(&solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };
    let corner_vertex = alloc();
    let mut new_vertices: Vec<(u64, Vec3)> = vec![(corner_vertex, corner_point)];
    let mut vertex_points: HashMap<u64, Vec3> = HashMap::default();
    vertex_points.insert(corner_vertex, corner_point);
    let mut sharp_edges: Vec<EdgeRecord> = Vec::with_capacity(3);
    // Old vertex -> the recovered vertex it collapses onto.
    let mut collapse: HashMap<u64, u64> = HashMap::default();
    for (index, strip) in group.strips.iter().enumerate() {
        let far_vertex = alloc();
        new_vertices.push((far_vertex, plans[index].far_point));
        vertex_points.insert(far_vertex, plans[index].far_point);
        let sharp_edge_id = alloc();
        sharp_edges.push(EdgeRecord {
            id: sharp_edge_id,
            curve: make_line(plans[index].far_point, corner_point).or_refuse(KernelStage::Sew, "make_line")?,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: far_vertex,
            end_vertex_id: corner_vertex,
            degenerate: false,
            name: None,
        });
        // The strip's far vertices are the ends of its cap edge; its corner
        // vertices are the ends of the edge it shares with the vertex blend.
        for (boundary_index, target) in [
            (strip.cap_index, far_vertex),
            (strip.corner_index, corner_vertex),
        ] {
            let edge_id = strip.boundary[boundary_index];
            let edge = solid
                .edges
                .iter()
                .find(|edge| edge.id == edge_id)
                .ok_or_else(|| missing("edge", edge_id))?;
            collapse.insert(edge.start_vertex_id, target);
            collapse.insert(edge.end_vertex_id, target);
        }
    }
    // Every vertex the group wore has to go somewhere; an unmapped one would
    // be left dangling by the prune below.
    for &vertex_id in &group_vertices {
        if !collapse.contains_key(&vertex_id) {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "vertex_unmapped",
                format!(
                    "{op}: corner blend vertex {vertex_id} is not on a cap or corner edge \
                 (unexpected strip ordering)"
                ),
            ));
        }
    }

    // --- Relocate every surviving edge that ended on a collapsed vertex ----
    let unmoved = solid.edges.clone();
    let mut relocated: HashSet<u64> = HashSet::default();
    let pending = pending_edge_relocations(&solid, &group_edges, &collapse, &vertex_points);
    for (index, start_target, end_target) in pending {
        let mut record = solid.edges[index].clone();
        let widened = relocate_open_side_edge(
            &mut record,
            start_target,
            end_target,
            plane_tolerance,
            tolerance,
            op,
        )?;
        if !widened {
            // The flanking pair is two planes wherever the group's own walls
            // and caps meet, and that is the exact line the edge lies on.
            match rederive_side_edge_on_planes(
                &solid,
                &solid.edges[index],
                &planes,
                start_target,
                end_target,
                plane_tolerance,
                tolerance,
            )? {
                Some(rebuilt) => record = rebuilt,
                None => {
                    record = rederive_side_edge_on_carriers(
                        &solid,
                        &solid.edges[index],
                        start_target,
                        end_target,
                        plane_tolerance,
                        tolerance,
                        op,
                    )?
                }
            }
        }
        relocated.insert(record.id);
        solid.edges[index] = record;
    }
    moved_edges_lie_on_their_faces(&unmoved, &solid, &relocated, &planes, plane_tolerance, op)?;

    // Index the (now relocated) edges for the loop rewrites.
    let mut edges_by_id: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    for edge in &sharp_edges {
        edges_by_id.insert(edge.id, edge.clone());
    }
    // A coedge on a relocated edge already carries its new vertex; one on a
    // strip rim still carries the vertex that is about to collapse.
    let resolve = |vertex_id: u64| -> u64 { collapse.get(&vertex_id).copied().unwrap_or(vertex_id) };

    // --- Walls: swap each strip rim for the sharp edge that replaces it ----
    for (index, strip) in group.strips.iter().enumerate() {
        let sharp_edge_id = sharp_edges[index].id;
        let far_vertex = sharp_edges[index].start_vertex_id;
        for &wall_index in &strip.walls {
            let rim_edge = strip.boundary[wall_index];
            let neighbour_id = strip.neighbours[wall_index];
            let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing("wall", neighbour_id))?;
            let face = &mut solid.shells[ns].faces[nf];
            let (loop_index, coedge_index) = locate_coedge(face, rim_edge).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "wall_coedge",
                    format!("{op}: wall {neighbour_id} does not use edge {rim_edge}"),
                )
            })?;
            let coedges = &face.loops[loop_index].coedges;
            let count = coedges.len();
            let previous = &coedges[(coedge_index + count - 1) % count];
            let next = &coedges[(coedge_index + 1) % count];
            let unresolved = || {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "loop_connectivity",
                    format!("{op}: could not resolve loop connectivity"),
                )
            };
            let required_from = coedge_to_vertex(previous, &edges_by_id)
                .map(resolve)
                .ok_or_else(unresolved)?;
            let required_to = coedge_from_vertex(next, &edges_by_id)
                .map(resolve)
                .ok_or_else(unresolved)?;
            let forward = if required_from == far_vertex && required_to == corner_vertex {
                true
            } else if required_from == corner_vertex && required_to == far_vertex {
                false
            } else {
                return Err(KernelRefusal::internal(
                    KernelStage::Sew,
                    "wall_loop",
                    format!(
                        "{op}: the recovered sharp edge does not close wall {neighbour_id}'s loop \
                     (unexpected connectivity)"
                    ),
                ));
            };
            let new_coedge = CoedgeRecord {
                id: alloc(),
                edge_id: sharp_edge_id,
                forward,
                // Placeholder; the re-trim/refit pass below recomputes it.
                pcurve: make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0))
                    .or_refuse(KernelStage::Sew, "make_line")?,
            };
            face.loops[loop_index].coedges[coedge_index] = new_coedge;
        }
    }

    // --- Caps: drop the transverse blend edge the strip ran out on ---------
    for strip in &group.strips {
        let cap_edge = strip.boundary[strip.cap_index];
        let neighbour_id = strip.cap_id();
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing("cap", neighbour_id))?;
        let face = &mut solid.shells[ns].faces[nf];
        let (loop_index, coedge_index) = locate_coedge(face, cap_edge).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "cap_coedge",
                format!("{op}: cap {neighbour_id} does not use edge {cap_edge}"),
            )
        })?;
        face.loops[loop_index].coedges.remove(coedge_index);
        if face.loops[loop_index].coedges.is_empty() {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "emptied_loop",
                format!("{op}: healing emptied a cap face loop"),
            ));
        }
    }

    // --- Prune the group, its edges and its vertices -----------------------
    let removed_faces: HashSet<u64> = group
        .strips
        .iter()
        .map(|strip| strip.face_id)
        .chain(std::iter::once(group.corner_id))
        .collect();
    solid.shells[group.shell_index]
        .faces
        .retain(|face| !removed_faces.contains(&face.id));
    solid.edges.retain(|edge| !group_edges.contains(&edge.id));
    let removed_vertices: HashSet<u64> = group_vertices.iter().copied().collect();
    solid
        .vertices
        .retain(|vertex| !removed_vertices.contains(&vertex.id));
    for edge in &sharp_edges {
        solid.edges.push(edge.clone());
    }
    for &(vertex_id, point) in &new_vertices {
        solid.vertices.push(VertexRecord {
            id: vertex_id,
            point,
        });
    }

    // --- Re-trim and refit every wall and cap ------------------------------
    let final_edges: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    let mut touched: HashSet<u64> = HashSet::default();
    let mut retrim_order: Vec<u64> = planes.keys().copied().collect();
    retrim_order.sort_unstable();
    for neighbour_id in retrim_order {
        let plane = &planes[&neighbour_id];
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing("neighbour", neighbour_id))?;
        retrim_planar_face(
            &mut solid.shells[ns].faces[nf],
            plane,
            &final_edges,
            scale,
            op,
        )
        ?;
        // The planar re-trim maps each edge's WHOLE curve; every edge of the
        // face that represents a strict SUBRANGE of its curve — a widened side
        // edge, but equally an untouched edge an earlier boolean trimmed —
        // needs the range fitter so its pcurve spans exactly `[t0, t1]`.
        touched.clear();
        touched.extend(
            solid.shells[ns].faces[nf]
                .loops
                .iter()
                .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.edge_id)),
        );
        refit_touched_pcurves(
            &mut solid.shells[ns].faces[nf],
            &final_edges,
            &touched,
            true,
            tolerance,
            op,
        )?;
    }
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validation",
            format!("{op}: corner blend heal failed validation: {issues:?}"),
        ));
    }
    Ok(solid)
}

// ---------------------------------------------------------------------------
// Curved walls and caps: the triple-surface solve.
// ---------------------------------------------------------------------------

/// What the curved corner lane measured on a heal it completed.
#[derive(Clone, Debug)]
pub(super) struct CurvedCornerReport {
    /// The recovered corner vertex: the three walls' triple point, seeded at
    /// the vertex blend's own vertices.
    pub(super) corner: TriplePoint,
    /// The largest distance between that solve and the same solve seeded at
    /// each strip's edge on the vertex blend.
    pub(super) seed_spread: f64,
    /// Each strip's recovered far corner (its two walls and its cap), in the
    /// corner's loop order.
    pub(super) far: Vec<TriplePoint>,
    /// The least distance a recovered corner lies PAST a rim of the strips it
    /// ends, measured along the wall away from the wall's own face.
    pub(super) least_setback: f64,
    /// The worst distance from a sample of a moved side edge to the carrier of
    /// a face it bounds.
    pub(super) incidence: f64,
    /// The worst distance from a sample of a new or moved edge to the carrier
    /// patch of a curved face it bounds, after that carrier was extended.
    pub(super) coverage: f64,
    /// How much farther from its seed the nearest admissible RIVAL root lies
    /// than the root taken, over every solve; `None` when no rival is
    /// admissible within the group's reach.
    pub(super) rival_margin: Option<f64>,
}

/// How far `point` lies past a strip's rim on `wall`, along the wall and away
/// from the wall's own face.
///
/// A blend strip leaves its wall tangentially, so the direction from a rim
/// point INTO the strip — read as the chord to the strip's other rim, with the
/// wall's normal and the rim's tangent taken out — is the direction the wall
/// continues past its trim toward the sharp edge the blend replaced. That holds
/// for a convex blend and a concave one alike: in the section, both the chord
/// and the sharp edge lie on the same side of the rim. A root with a negative
/// reading is on the wrong side of the rim — a triple point the blend was not
/// cut from.
fn setback_past_rim(
    point: Vec3,
    rim: &EdgeRecord,
    other_rim: &EdgeRecord,
    wall: &TripleCarrier,
    op: &str,
) -> Result<f64, KernelRefusal> {
    let (t, foot) = nearest_on_edge(rim, point)?;
    let tangent = rim
        .curve
        .deriv1(t)
        .or_refuse(KernelStage::Classify, "tangent")?
        .1
        .normalized()
        .or_refuse(KernelStage::Classify, "tangent")?;
    let normal = wall.normal_at(foot)?;
    let (_, across) = nearest_on_edge(other_rim, foot)?;
    let mut inward = across.sub(foot);
    inward = inward.sub(normal.scale(inward.dot(normal)));
    inward = inward.sub(tangent.scale(inward.dot(tangent)));
    let inward = inward.normalized().map_err(|_| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "rims_coincide",
            format!(
                "{op}: blend rim {} has no direction into its strip (its rims coincide)",
                rim.id
            ),
        )
    })?;
    Ok(point.sub(foot).dot(inward))
}

/// The least setback of `point` past both rims of `strip`.
fn strip_setback(
    solid: &BrepSolid,
    strip: &CornerStrip,
    point: Vec3,
    carriers: &HashMap<u64, TripleCarrier>,
    op: &str,
) -> Result<f64, KernelRefusal> {
    let edge = |id: u64| -> Result<&EdgeRecord, KernelRefusal> {
        solid.edges.iter().find(|edge| edge.id == id).ok_or_else(|| {
            KernelRefusal::internal(KernelStage::Collect, "edge", format!("{op}: missing edge {id}"))
        })
    };
    let mut least = f64::INFINITY;
    for side in 0..2 {
        let rim = edge(strip.boundary[strip.walls[side]])?;
        let other = edge(strip.boundary[strip.walls[1 - side]])?;
        let wall = &carriers[&strip.neighbours[strip.walls[side]]];
        least = least.min(setback_past_rim(point, rim, other, wall, op)?);
    }
    Ok(least)
}

/// The recovered sharp edge of one strip: a piece of its two walls'
/// intersection from the far corner to the corner vertex.
struct CurvedStripPlan {
    curve: NurbsCurve,
    t0: f64,
    t1: f64,
    /// Whether the curve runs from the far corner (true) or from the corner
    /// vertex (false) as its parameter increases.
    from_far: bool,
}

/// Build the arc of the circle through `from`, `via` and `to` that starts at
/// `from`, passes `via` and ends at `to`, parameterized over `[0, 1]`.
fn arc_through(
    center: Vec3,
    normal: Vec3,
    radius: f64,
    from: Vec3,
    via: Vec3,
    to: Vec3,
) -> Result<NurbsCurve, KernelRefusal> {
    let radial = from.sub(center);
    let x_axis = radial
        .sub(normal.scale(radial.dot(normal)))
        .normalized()
        .or_refuse(KernelStage::Sew, "arc_axis")?;
    let angle = |point: Vec3, y_axis: Vec3| {
        let delta = point.sub(center);
        let theta = delta.dot(y_axis).atan2(delta.dot(x_axis));
        if theta < 0.0 {
            theta + std::f64::consts::TAU
        } else {
            theta
        }
    };
    let mut y_axis = normal.cross(x_axis);
    if angle(via, y_axis) > angle(to, y_axis) {
        y_axis = y_axis.scale(-1.0);
    }
    crate::make_arc(center, x_axis, y_axis, radius, 0.0, angle(to, y_axis)).or_refuse(KernelStage::Sew, "make_arc")
}

/// The piece of the walls' intersection that joins `far` to `corner` beside the
/// strip, or a refusal naming what was found.
#[allow(clippy::too_many_arguments)]
fn plan_curved_strip_edge(
    strip: &CornerStrip,
    branches: &[NurbsCurve],
    far: Vec3,
    corner: Vec3,
    witness: Vec3,
    plane_tolerance: f64,
    tolerance: f64,
    op: &str,
) -> Result<CurvedStripPlan, KernelRefusal> {
    let mut nearest = [f64::INFINITY; 2];
    let mut plans: Vec<CurvedStripPlan> = Vec::new();
    for branch in branches {
        let (Ok(on_far), Ok(on_corner)) = (
            project_point_to_curve(branch, far),
            project_point_to_curve(branch, corner),
        ) else {
            continue;
        };
        nearest = [
            nearest[0].min(on_far.distance),
            nearest[1].min(on_corner.distance),
        ];
        if on_far.distance > plane_tolerance || on_corner.distance > plane_tolerance {
            continue;
        }
        let on_witness = project_point_to_curve(branch, witness).or_refuse(KernelStage::Classify, "project")?;
        let (low, high) = (on_far.u.min(on_corner.u), on_far.u.max(on_corner.u));
        let [d0, d1] = branch.domain().or_refuse(KernelStage::Classify, "domain")?;
        let evaluate = |t: f64| branch.evaluate(t).or_refuse(KernelStage::Classify, "evaluate");
        let closed = evaluate(d0)?.sub(evaluate(d1)?).length() <= tolerance;
        let inside = on_witness.u >= low && on_witness.u <= high;
        if !inside && !closed {
            continue;
        }
        // A circular piece is rebuilt as the exact arc from one corner to the
        // other past the strip: a planar face maps a whole rational arc onto
        // its plane exactly, where a sub-range of the section circle would be
        // refitted. Across the origin of a closed branch there is no other
        // honest construction.
        let via = on_witness.point;
        let circle = circle_through(far, via, corner).filter(|(center, normal, radius)| {
            (0..16).all(|index| {
                branch
                    .evaluate(d0 + (d1 - d0) * index as f64 / 16.0)
                    .map(|sample| {
                        let radial = sample.sub(*center);
                        (radial.length() - radius).abs() <= tolerance
                            && radial.dot(*normal).abs() <= tolerance
                    })
                    .unwrap_or(false)
            })
        });
        let plan = match circle {
            Some((center, normal, radius)) => CurvedStripPlan {
                curve: arc_through(center, normal, radius, far, via, corner)?,
                t0: 0.0,
                t1: 1.0,
                from_far: true,
            },
            None if inside => CurvedStripPlan {
                curve: branch.clone(),
                t0: low,
                t1: high,
                from_far: on_far.u < on_corner.u,
            },
            None => {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "origin_crossing",
                    format!(
                        "{op}: the sharp edge recovered for blend face {} crosses the origin of \
                     its walls' closed intersection and that intersection is not a circle \
                     (deferred)",
                        strip.face_id
                    ),
                ))
            }
        };
        plans.push(plan);
    }
    match plans.len() {
        0 => Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "no_joining_piece",
            format!(
                "{op}: no piece of the walls' intersection beside blend face {} joins its \
             recovered corners (nearest branch {:.3e} from the far corner and {:.3e} from the \
             corner vertex)",
                strip.face_id, nearest[0], nearest[1]
            ),
        )),
        _ => {
            // One branch found twice is one branch; two different pieces are a
            // question the strip does not answer.
            let middle = |plan: &CurvedStripPlan| {
                plan.curve
                    .evaluate(0.5 * (plan.t0 + plan.t1))
                    .or_refuse(KernelStage::Classify, "evaluate")
            };
            let first = middle(&plans[0])?;
            for plan in &plans[1..] {
                let other = middle(plan)?;
                if other.sub(first).length() > plane_tolerance {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "piece_ambiguous",
                        format!(
                            "{op}: two different pieces of the walls' intersection join the \
                         recovered corners of blend face {} — refusing rather than choosing one",
                            strip.face_id
                        ),
                    ));
                }
            }
            Ok(plans.swap_remove(0))
        }
    }
}

/// Heal a corner blend group whose walls or caps are not all planes.
///
/// The planar lane's construction with the closed forms replaced: the corner
/// vertex is the three walls' triple point, each far corner the triple point
/// of its strip's two walls and cap ([`solve_triple_point`], on the INFINITE
/// analytic carriers), and each sharp edge the piece of its walls'
/// intersection between them. Every root is checked to be the one the blend
/// was cut from — past every rim it ends, and the only such root on its branch
/// within the group's reach — and the curved carriers are kept, extended where
/// the recovered edges overrun them, with that coverage measured.
pub(super) fn heal_curved_corner_blend_group(
    source: &BrepSolid,
    group: &CornerBlendGroup,
    op: &str,
) -> Result<(BrepSolid, CurvedCornerReport), KernelRefusal> {
    let scale = solid_model_scale(source);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let debug = std::env::var("BREP_DEBUG_HEAL").is_ok();
    let missing = |what: &'static str, id: u64| {
        KernelRefusal::internal(KernelStage::Collect, what, format!("{op}: missing {what} {id}"))
    };
    let edge_of = |id: u64| -> Result<&EdgeRecord, KernelRefusal> {
        source
            .edges
            .iter()
            .find(|edge| edge.id == id)
            .ok_or_else(|| missing("edge", id))
    };

    // --- Carriers ---------------------------------------------------------
    let mut wall_ids: Vec<u64> = group.strips.iter().flat_map(|strip| strip.wall_ids()).collect();
    wall_ids.sort_unstable();
    wall_ids.dedup();
    let mut neighbour_ids: Vec<u64> = wall_ids
        .iter()
        .copied()
        .chain(group.strips.iter().map(|strip| strip.cap_id()))
        .collect();
    neighbour_ids.sort_unstable();
    neighbour_ids.dedup();
    let mut planes: HashMap<u64, Plane> = HashMap::default();
    let mut carriers: HashMap<u64, TripleCarrier> = HashMap::default();
    for &neighbour_id in &neighbour_ids {
        let (ns, nf) = find_face(source, neighbour_id).ok_or_else(|| missing("neighbour", neighbour_id))?;
        let surface = &source.shells[ns].faces[nf].surface;
        let role = if wall_ids.contains(&neighbour_id) { "wall" } else { "cap" };
        if let Ok(plane) = plane_of_surface(surface, plane_tolerance, op) {
            planes.insert(neighbour_id, plane);
        } else if surface.analytic().is_none() {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "fitted_carrier",
                format!(
                    "{op}: the corner blend's {role} face {neighbour_id} is a fitted patch — the \
                 corner lane re-intersects analytic carriers only (deferred)"
                ),
            ));
        }
        carriers.insert(
            neighbour_id,
            TripleCarrier::of(surface).map_err(|reason| {
                reason.with_message(|reason| {
                    format!("{op}: the corner blend's {role} face {neighbour_id} cannot be read ({reason})")
                })
            })?,
        );
    }

    // --- The group's own region gate (centre + reach), from its vertices ---
    let (group_edges, group_vertices, centre, reach) = corner_group_region(source, group, tolerance, op)?;
    let policy = TriplePolicy {
        tolerance,
        reach,
        position_bar: plane_tolerance,
    };
    let edge_middle = |id: u64| -> Result<Vec3, KernelRefusal> {
        let edge = edge_of(id)?;
        edge.curve
            .evaluate(0.5 * (edge.t0 + edge.t1))
            .or_refuse(KernelStage::Classify, "evaluate")
    };

    // --- The corner vertex: the walls' triple point ------------------------
    let walls: [&TripleCarrier; 3] = [
        &carriers[&wall_ids[0]],
        &carriers[&wall_ids[1]],
        &carriers[&wall_ids[2]],
    ];
    let corner_seed = {
        let mut sum = Vec3::default();
        let mut count = 0usize;
        for &edge_id in &group.corner_edges {
            let edge = edge_of(edge_id)?;
            for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
                sum = sum.add(edge_point(source, vertex)?);
                count += 1;
            }
        }
        sum.scale(1.0 / count.max(1) as f64)
    };
    let corner = solve_triple_point(walls, corner_seed, &policy).map_err(|refusal| {
        refusal.into_refusal(|reason| {
            format!(
                "{op}: the corner blend's three walls ({}) do not meet at one corner near it: {}",
                carrier_kinds(walls),
                reason
            )
        })
    })?;
    // The same root from every strip's own edge on the vertex blend.
    let mut seed_spread = 0.0f64;
    for strip in &group.strips {
        let seed = edge_middle(strip.boundary[strip.corner_index])?;
        let again = solve_triple_point(walls, seed, &policy).map_err(|refusal| {
            refusal.into_refusal(|reason| {
                format!(
                    "{op}: the corner blend's three walls do not meet near blend face {}'s end: {}",
                    strip.face_id,
                    reason
                )
            })
        })?;
        let spread = again.point.sub(corner.point).length();
        if spread > plane_tolerance {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Refine,
                "seed_spread",
                format!(
                    "{op}: the corner blend's walls meet at different corners from different seeds \
                 ({spread:.3e} apart) — refusing rather than choosing one"
                ),
            ));
        }
        seed_spread = seed_spread.max(spread);
    }
    let corner_point = corner.point;

    // --- Each strip's far corner: its walls and its cap ---------------------
    let mut least_setback = f64::INFINITY;
    for strip in &group.strips {
        let setback = strip_setback(source, strip, corner_point, &carriers, op)?;
        if setback <= tolerance {
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "corner_short_of_rim",
                format!(
                    "{op}: the walls' triple point lies {setback:.3e} short of blend face {}'s rim — \
                 it is not the corner the blend was cut from",
                    strip.face_id
                ),
            ));
        }
        least_setback = least_setback.min(setback);
    }
    let mut far: Vec<TriplePoint> = Vec::with_capacity(3);
    for strip in &group.strips {
        let [wall_a, wall_b] = strip.wall_ids();
        let triple: [&TripleCarrier; 3] = [
            &carriers[&wall_a],
            &carriers[&wall_b],
            &carriers[&strip.cap_id()],
        ];
        let seed = edge_middle(strip.boundary[strip.cap_index])?;
        let solved = solve_triple_point(triple, seed, &policy).map_err(|refusal| {
            refusal.into_refusal(|reason| {
                format!(
                    "{op}: the walls flanking blend face {} do not meet the face it runs out into \
                 ({}): {}",
                    strip.face_id,
                    carrier_kinds(triple),
                    reason
                )
            })
        })?;
        let setback = strip_setback(source, strip, solved.point, &carriers, op)?;
        if setback <= tolerance {
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "far_short_of_rim",
                format!(
                    "{op}: the far corner recovered for blend face {} lies {setback:.3e} short of its \
                 rim — it is not the corner the blend was cut from",
                    strip.face_id
                ),
            ));
        }
        least_setback = least_setback.min(setback);
        if solved.point.sub(corner_point).length() <= tolerance {
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "curved_edge_collapse",
                format!(
                    "{op}: the sharp edge recovered for blend face {} collapses to the corner vertex",
                    strip.face_id
                ),
            ));
        }
        far.push(solved);
    }

    // --- Grow the curved carriers over everything the heal will put on them -
    let mut solid = source.clone();
    let mut reach_points: Vec<Vec3> = vec![corner_point];
    reach_points.extend(far.iter().map(|solved| solved.point));
    for strip in &group.strips {
        for &edge_id in &strip.boundary {
            reach_points.extend(arc_length_stations(edge_of(edge_id)?, 9, true)?);
        }
    }
    for &neighbour_id in &neighbour_ids {
        if !planes.contains_key(&neighbour_id) {
            extend_ruled_neighbour_over(&mut solid, neighbour_id, &reach_points, tolerance)?;
        }
    }
    let surface_of = |solid: &BrepSolid, id: u64| -> Result<NurbsSurface, KernelRefusal> {
        let (ns, nf) = find_face(solid, id).ok_or_else(|| missing("face", id))?;
        Ok(solid.shells[ns].faces[nf].surface.clone())
    };

    // --- Each strip's sharp edge, and the census of rival roots -------------
    let mut rival_margin: Option<f64> = None;
    let mut plans: Vec<CurvedStripPlan> = Vec::with_capacity(3);
    for (index, strip) in group.strips.iter().enumerate() {
        let [wall_a, wall_b] = strip.wall_ids();
        let far_point = far[index].point;
        let branches: Vec<NurbsCurve> = match (planes.get(&wall_a), planes.get(&wall_b)) {
            (Some(_), Some(_)) => {
                // Two planes: the exact line, long enough to carry every root
                // the census below has to see.
                let direction = corner_point
                    .sub(far_point)
                    .normalized()
                    .or_refuse(KernelStage::Sew, "edge_direction")?;
                vec![make_line(
                    corner_point.sub(direction.scale(2.0 * reach)),
                    corner_point.add(direction.scale(2.0 * reach)),
                )
                .or_refuse(KernelStage::Sew, "make_line")?]
            }
            _ => intersect_analytic_pair(
                &surface_of(&solid, wall_a)?,
                &surface_of(&solid, wall_b)?,
                tolerance,
            )
            .ok_or_else(|| {
                KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "no_closed_form_walls",
                    format!(
                        "{op}: the walls flanking blend face {} ({} × {}) have no closed-form \
                     re-intersection (deferred)",
                        strip.face_id,
                        carrier_kind(&carriers[&wall_a]),
                        carrier_kind(&carriers[&wall_b])
                    ),
                )
            })?,
        };
        // A RIVAL is another root on the walls' intersection: another point
        // where it meets the third wall or the cap, inside the group's reach,
        // and past the rims the root has to be past. The solve's root has to
        // be nearer its seed — the deleted corner's own geometry — than every
        // rival; one the seed cannot tell apart from it is refused.
        let third = wall_ids
            .iter()
            .copied()
            .find(|id| *id != wall_a && *id != wall_b)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "curved_wall_cycle",
                    format!("{op}: the corner's walls do not close a cycle"),
                )
            })?;
        let far_seed = edge_middle(strip.boundary[strip.cap_index])?;
        for branch in &branches {
            let domain = branch.domain().or_refuse(KernelStage::Classify, "domain")?;
            for (carrier_id, root, seed, is_corner) in [
                (third, corner_point, corner_seed, true),
                (strip.cap_id(), far_point, far_seed, false),
            ] {
                for rival in carrier_crossings(branch, domain, &carriers[&carrier_id], centre, reach) {
                    if rival.sub(root).length() <= plane_tolerance {
                        continue;
                    }
                    let setback = if is_corner {
                        let mut least = f64::INFINITY;
                        for other in &group.strips {
                            least = least.min(strip_setback(source, other, rival, &carriers, op)?);
                        }
                        least
                    } else {
                        strip_setback(source, strip, rival, &carriers, op)?
                    };
                    if setback <= tolerance {
                        continue;
                    }
                    let margin = rival.sub(seed).length() - root.sub(seed).length();
                    rival_margin = Some(rival_margin.map_or(margin, |known: f64| known.min(margin)));
                    if margin <= plane_tolerance {
                        let point = |p: Vec3| format!("({:.6}, {:.6}, {:.6})", p.x, p.y, p.z);
                        let label = if is_corner { "corner vertex" } else { "far corner" };
                        return Err(KernelRefusal::ill_posed(
                            KernelStage::Refine,
                            "rival_root",
                            format!(
                                "{op}: the walls flanking blend face {} reach two admissible {label}s, \
                             {} and {}, and the second is no farther from the deleted corner \
                             ({margin:.3e}) — refusing rather than choosing one",
                                strip.face_id,
                                point(root),
                                point(rival)
                            ),
                        ));
                    }
                }
            }
        }
        let witness = edge_middle(strip.boundary[strip.walls[0]])?;
        let plan = if planes.contains_key(&wall_a) && planes.contains_key(&wall_b) {
            CurvedStripPlan {
                curve: make_line(far_point, corner_point).or_refuse(KernelStage::Sew, "make_line")?,
                t0: 0.0,
                t1: 1.0,
                from_far: true,
            }
        } else {
            plan_curved_strip_edge(
                strip,
                &branches,
                far_point,
                corner_point,
                witness,
                plane_tolerance,
                tolerance,
                op,
            )?
        };
        plans.push(plan);
    }
    // --- New vertices and the three new sharp edges ------------------------
    let mut next_id = max_topology_id(&solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };
    let corner_vertex = alloc();
    let mut new_vertices: Vec<(u64, Vec3)> = vec![(corner_vertex, corner_point)];
    let mut vertex_points: HashMap<u64, Vec3> = HashMap::default();
    vertex_points.insert(corner_vertex, corner_point);
    let mut sharp_edges: Vec<EdgeRecord> = Vec::with_capacity(3);
    let mut far_vertices: Vec<u64> = Vec::with_capacity(3);
    let mut collapse: HashMap<u64, u64> = HashMap::default();
    for (index, strip) in group.strips.iter().enumerate() {
        let far_vertex = alloc();
        far_vertices.push(far_vertex);
        new_vertices.push((far_vertex, far[index].point));
        vertex_points.insert(far_vertex, far[index].point);
        let plan = &plans[index];
        let (start_vertex_id, end_vertex_id) = if plan.from_far {
            (far_vertex, corner_vertex)
        } else {
            (corner_vertex, far_vertex)
        };
        sharp_edges.push(EdgeRecord {
            id: alloc(),
            curve: plan.curve.clone(),
            t0: plan.t0,
            t1: plan.t1,
            start_vertex_id,
            end_vertex_id,
            degenerate: false,
            name: None,
        });
        for (boundary_index, target) in [
            (strip.cap_index, far_vertex),
            (strip.corner_index, corner_vertex),
        ] {
            let edge = edge_of(strip.boundary[boundary_index])?;
            collapse.insert(edge.start_vertex_id, target);
            collapse.insert(edge.end_vertex_id, target);
        }
    }
    for &vertex_id in &group_vertices {
        if !collapse.contains_key(&vertex_id) {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "curved_vertex_unmapped",
                format!(
                    "{op}: corner blend vertex {vertex_id} is not on a cap or corner edge \
                 (unexpected strip ordering)"
                ),
            ));
        }
    }
    // Each sharp edge's ends must BE its vertices, to the incidence bar.
    for edge in &sharp_edges {
        for (t, vertex) in [(edge.t0, edge.start_vertex_id), (edge.t1, edge.end_vertex_id)] {
            let miss = edge
                .curve
                .evaluate(t)
                .or_refuse(KernelStage::Sew, "evaluate")?
                .sub(vertex_points[&vertex])
                .length();
            if miss > plane_tolerance {
                return Err(KernelRefusal::internal(
                    KernelStage::Sew,
                    "edge_end_miss",
                    format!("{op}: a recovered sharp edge ends {miss:.3e} from its corner"),
                ));
            }
        }
    }

    // --- Relocate every surviving edge that ended on a collapsed vertex ----
    let mut relocated: HashSet<u64> = HashSet::default();
    let unmoved = solid.edges.clone();
    let pending = pending_edge_relocations(&solid, &group_edges, &collapse, &vertex_points);
    for (index, start_target, end_target) in pending {
        let mut record = solid.edges[index].clone();
        let widened = relocate_open_side_edge(
            &mut record,
            start_target,
            end_target,
            plane_tolerance,
            tolerance,
            op,
        )?;
        if !widened {
            match rederive_side_edge_on_planes(
                &solid,
                &solid.edges[index],
                &planes,
                start_target,
                end_target,
                plane_tolerance,
                tolerance,
            )? {
                Some(rebuilt) => record = rebuilt,
                None => {
                    record = rederive_side_edge_on_carriers(
                        &solid,
                        &solid.edges[index],
                        start_target,
                        end_target,
                        plane_tolerance,
                        tolerance,
                        op,
                    )?
                }
            }
        }
        relocated.insert(record.id);
        solid.edges[index] = record;
    }
    let incidence = moved_edges_lie_on_their_faces(&unmoved, &solid, &relocated, &planes, plane_tolerance, op)?;

    let mut edges_by_id: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    for edge in &sharp_edges {
        edges_by_id.insert(edge.id, edge.clone());
    }
    let resolve = |vertex_id: u64| -> u64 { collapse.get(&vertex_id).copied().unwrap_or(vertex_id) };

    // --- Walls: swap each strip rim for the sharp edge that replaces it ----
    for (index, strip) in group.strips.iter().enumerate() {
        let sharp = &sharp_edges[index];
        for &wall_index in &strip.walls {
            let rim_edge = strip.boundary[wall_index];
            let neighbour_id = strip.neighbours[wall_index];
            let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing("wall", neighbour_id))?;
            let face = &mut solid.shells[ns].faces[nf];
            let (loop_index, coedge_index) = locate_coedge(face, rim_edge).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "curved_wall_coedge",
                    format!("{op}: wall {neighbour_id} does not use edge {rim_edge}"),
                )
            })?;
            let coedges = &face.loops[loop_index].coedges;
            let count = coedges.len();
            let previous = &coedges[(coedge_index + count - 1) % count];
            let next = &coedges[(coedge_index + 1) % count];
            let unresolved = || {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "curved_loop_connectivity",
                    format!("{op}: could not resolve loop connectivity"),
                )
            };
            let required_from = coedge_to_vertex(previous, &edges_by_id)
                .map(resolve)
                .ok_or_else(unresolved)?;
            let required_to = coedge_from_vertex(next, &edges_by_id)
                .map(resolve)
                .ok_or_else(unresolved)?;
            let forward = if required_from == sharp.start_vertex_id
                && required_to == sharp.end_vertex_id
            {
                true
            } else if required_from == sharp.end_vertex_id && required_to == sharp.start_vertex_id
            {
                false
            } else {
                return Err(KernelRefusal::internal(
                    KernelStage::Sew,
                    "curved_wall_loop",
                    format!(
                        "{op}: the recovered sharp edge does not close wall {neighbour_id}'s loop \
                     (unexpected connectivity)"
                    ),
                ));
            };
            face.loops[loop_index].coedges[coedge_index] = CoedgeRecord {
                id: alloc(),
                edge_id: sharp.id,
                forward,
                // Placeholder; the re-trim/refit pass below recomputes it.
                pcurve: make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0))
                    .or_refuse(KernelStage::Sew, "make_line")?,
            };
        }
    }

    // --- Caps: drop the transverse blend edge the strip ran out on ---------
    for strip in &group.strips {
        let cap_edge = strip.boundary[strip.cap_index];
        let neighbour_id = strip.cap_id();
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing("cap", neighbour_id))?;
        let face = &mut solid.shells[ns].faces[nf];
        let (loop_index, coedge_index) = locate_coedge(face, cap_edge).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "curved_cap_coedge",
                format!("{op}: cap {neighbour_id} does not use edge {cap_edge}"),
            )
        })?;
        face.loops[loop_index].coedges.remove(coedge_index);
        if face.loops[loop_index].coedges.is_empty() {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "curved_emptied_loop",
                format!("{op}: healing emptied a cap face loop"),
            ));
        }
    }

    // --- Prune the group, its edges and its vertices -----------------------
    let removed_faces: HashSet<u64> = group
        .strips
        .iter()
        .map(|strip| strip.face_id)
        .chain(std::iter::once(group.corner_id))
        .collect();
    solid.shells[group.shell_index]
        .faces
        .retain(|face| !removed_faces.contains(&face.id));
    solid.edges.retain(|edge| !group_edges.contains(&edge.id));
    let removed_vertices: HashSet<u64> = group_vertices.iter().copied().collect();
    solid
        .vertices
        .retain(|vertex| !removed_vertices.contains(&vertex.id));
    for edge in &sharp_edges {
        solid.edges.push(edge.clone());
    }
    for &(vertex_id, point) in &new_vertices {
        solid.vertices.push(VertexRecord {
            id: vertex_id,
            point,
        });
    }

    // --- A curved face's PATCH must reach every new or moved edge ----------
    // The sharp edges lie on their walls by construction (both ends are
    // triple points and the curve is the walls' own intersection), and a moved
    // edge was gated on its faces' carriers above. Neither says that the PATCH
    // a kept curved face is trimmed on reaches them.
    let final_edges: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    let mut touched: HashSet<u64> = relocated;
    touched.extend(sharp_edges.iter().map(|edge| edge.id));
    let mut coverage = 0.0f64;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if planes.contains_key(&face.id) || !neighbour_ids.contains(&face.id) {
            continue;
        }
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if !touched.contains(&coedge.edge_id) {
                continue;
            }
            let edge = &final_edges[&coedge.edge_id];
            // The distance is to the PATCH, so the patch's own edge sets how densely.
            let intervals = sample_intervals(Some(&face.surface), edge, face.id, op)?;
            for sample in 0..=intervals {
                let t = edge.t0 + (edge.t1 - edge.t0) * sample as f64 / intervals as f64;
                let point = edge.curve.evaluate(t).or_refuse(KernelStage::Validate, "evaluate")?;
                let distance = project_point_to_surface(&face.surface, point)
                    .or_refuse(KernelStage::Validate, "project")?
                    .distance;
                coverage = coverage.max(distance);
                if distance > plane_tolerance {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Validate,
                        "carrier_short",
                        format!(
                            "{op}: the carrier of curved face {} does not reach recovered edge {} \
                         ({distance:.3e} off it) — refusing rather than trimming it on an \
                         extrapolation",
                            face.id, edge.id
                        ),
                    ));
                }
            }
        }
    }

    // --- Re-trim planes, refit curved carriers ----------------------------
    for &neighbour_id in &neighbour_ids {
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing("neighbour", neighbour_id))?;
        let face = &mut solid.shells[ns].faces[nf];
        match planes.get(&neighbour_id) {
            Some(plane) => {
                retrim_planar_face(face, plane, &final_edges, scale, op)?;
                let face_edges: HashSet<u64> = face
                    .loops
                    .iter()
                    .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.edge_id))
                    .collect();
                refit_touched_pcurves(face, &final_edges, &face_edges, true, tolerance, op)?;
            }
            None => {
                // The carrier is kept: untouched or grown exactly along its
                // axis, both in its own parameterization, so only the new and
                // moved edges need fresh trims.
                refit_touched_pcurves(face, &final_edges, &touched, false, tolerance, op)?;
            }
        }
    }
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "curved_validation",
            format!("{op}: curved corner blend heal failed validation: {issues:?}"),
        ));
    }
    let report = CurvedCornerReport {
        corner,
        seed_spread,
        far,
        least_setback,
        incidence,
        coverage,
        rival_margin,
    };
    if debug {
        let describe = |solved: &TriplePoint| {
            format!(
                "({:.9}, {:.9}, {:.9}) in {} iterations, residual {:.3e}, σ_min {:.3e}, bound \
                 {:.3e}, sensitivity {:.3e}, {:.3e} from its seed",
                solved.point.x,
                solved.point.y,
                solved.point.z,
                solved.iterations,
                solved.residual,
                solved.sigma_min,
                solved.error_bound,
                solved.sensitivity,
                solved.seed_distance
            )
        };
        eprintln!("HEAL curved corner: corner {}", describe(&report.corner));
        for solved in &report.far {
            eprintln!("HEAL curved corner: far {}", describe(solved));
        }
        eprintln!(
            "HEAL curved corner: seed spread {:.3e}, least setback {:.3e}, incidence {:.3e}, \
             coverage {:.3e}, rival margin {:?}",
            report.seed_spread,
            report.least_setback,
            report.incidence,
            report.coverage,
            report.rival_margin
        );
    }
    Ok((solid, report))
}
