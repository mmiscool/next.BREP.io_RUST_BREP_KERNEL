use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// How densely a healed rim is sampled when asking whether it follows one of
/// its neighbour's parameter lines. Coarse on purpose: a rim that drifts off a
/// `v = const` line does so over the whole turn, not between two samples.
const RIM_STATION_SAMPLES: usize = 16;

/// How many stations of equal arc length each rim of a closed strip is read at.
const RIM_ARC_STATIONS: usize = 8;

/// Which closed re-intersection a closed heal keeps, decided by GEOMETRY.
///
/// Each candidate is rated by the WORST distance from any of `stations` — the
/// strip's two rims at equal arc length — to the candidate curve, measured by
/// projection; the least rating wins. The recovered rim is the one the strip
/// was blended from, so it runs beside both rims all the way round, and a
/// branch that passes near them only somewhere has a rim station far from it.
///
/// Refuses (`Err((chosen, rival))`) when a DISTINCT candidate — one that does
/// not lie on the winner within `tolerance` — rates within `tolerance` of the
/// least rating: nothing about the strip tells two such rims apart, and
/// keeping whichever the intersector listed first would make the heal a
/// property of that order. The decision reads every candidate before it
/// decides, so it does not depend on the order they arrive in either.
///
/// This is the closed strip's form of the open heal's `nearest_branch`. It
/// replaces two picks that were not geometric: the analytic lane's nearest
/// curve MIDPOINT (a point placed by the intersector's own parameterisation)
/// to the strip surface's `(0.5, 0.5)` point, and the marched lane's
/// `nearest_section`, which measures to 24 parameter samples of each section.
pub(super) fn nearest_closed_rim(
    candidates: &[NurbsCurve],
    stations: &[Vec3],
    tolerance: f64,
) -> Result<Option<usize>, (usize, usize)> {
    let rating = |curve: &NurbsCurve| {
        stations
            .iter()
            .map(|station| {
                project_point_to_curve(curve, *station)
                    .map(|projection| projection.distance)
                    .unwrap_or(f64::INFINITY)
            })
            .fold(0.0f64, f64::max)
    };
    let ratings: Vec<f64> = candidates.iter().map(rating).collect();
    let Some(chosen) = (0..candidates.len()).min_by(|a, b| ratings[*a].total_cmp(&ratings[*b])) else {
        return Ok(None);
    };
    // One curve lies on another when samples of each project onto the other.
    let lies_on = |curve: &NurbsCurve, other: &NurbsCurve| {
        let Ok([t0, t1]) = curve.domain() else {
            return false;
        };
        (0..16).all(|index| {
            curve
                .evaluate(t0 + (t1 - t0) * index as f64 / 16.0)
                .and_then(|point| project_point_to_curve(other, point))
                .map(|projection| projection.distance <= tolerance)
                .unwrap_or(false)
        })
    };
    for rival in 0..candidates.len() {
        if rival == chosen || ratings[rival] - ratings[chosen] > tolerance {
            continue;
        }
        let same_rim =
            lies_on(&candidates[rival], &candidates[chosen]) && lies_on(&candidates[chosen], &candidates[rival]);
        if !same_rim {
            return Err((chosen, rival));
        }
    }
    Ok(Some(chosen))
}

/// The closed heal's refusal for two rims [`nearest_closed_rim`] cannot tell
/// apart.
fn closed_rim_tie(
    op: &str,
    neighbour_ids: &[u64; 2],
    candidates: &[NurbsCurve],
    chosen: usize,
    rival: usize,
    stations: &[Vec3],
) -> KernelRefusal {
    let worst = |curve: &NurbsCurve| {
        stations
            .iter()
            .map(|station| project_point_to_curve(curve, *station).map(|p| p.distance).unwrap_or(f64::INFINITY))
            .fold(0.0f64, f64::max)
    };
    KernelRefusal::ill_posed(
        KernelStage::Classify,
        "rim_tie",
        format!(
            "{op}: two closed re-intersections of faces {} and {} are equally near the strip's rims \
         (worst rim station {:.6} and {:.6} away) — refusing rather than choosing one by the order \
         the intersector returned them",
            neighbour_ids[0],
            neighbour_ids[1],
            worst(&candidates[chosen]),
            worst(&candidates[rival])
        ),
    )
}

/// The constant `v` of a closed rim that lies on a `v = const` ISO-CURVE of
/// `surface`, or `None` when it does not lie on one.
///
/// This is the general form of the axial `(z / height)` station a
/// `RuledRevolution` states in closed form, and it is what lets a SPHERE,
/// TORUS or general revolution neighbour be rebound by the same code: the
/// healed rim is sampled, each sample projected onto the carrier, and the
/// station accepted only when
///
/// * every sample sits ON the carrier (within `tolerance`) — a rim that has
///   left the neighbour is not its rim at all, and
/// * their `v` values agree to a millionth of the v-domain — otherwise the
///   rim crosses parameter lines and a straight `v = const` pcurve would be a
///   fiction the pcurve/edge consistency check would (rightly) reject.
///
/// Returning `None` rather than a best-fit station is the point: the caller
/// refuses, and no solid is emitted whose trim curve does not follow the
/// boundary it claims to.
pub(super) fn rim_iso_station(surface: &NurbsSurface, rim: &NurbsCurve, tolerance: f64) -> Option<f64> {
    let [t0, t1] = rim.domain().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let v_span = (v1 - v0).abs().max(1e-12);
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    let mut total = 0.0f64;
    for index in 0..RIM_STATION_SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / RIM_STATION_SAMPLES as f64;
        let point = rim.evaluate(t).ok()?;
        let projection = crate::project_point_to_surface(surface, point).ok()?;
        if projection.distance > tolerance {
            return None;
        }
        low = low.min(projection.v);
        high = high.max(projection.v);
        total += projection.v;
    }
    if high - low > 1e-6 * v_span {
        return None;
    }
    Some((total / RIM_STATION_SAMPLES as f64).clamp(v0.min(v1), v0.max(v1)))
}

/// Centre, unit normal and radius of the circle through three points, or
/// `None` when they are collinear. Exact (the circumcentre in closed form).
pub(super) fn circle_through(a: Vec3, b: Vec3, c: Vec3) -> Option<(Vec3, Vec3, f64)> {
    let u = b.sub(a);
    let v = c.sub(a);
    let n = u.cross(v);
    let n_squared = n.dot(n);
    if n_squared <= 1e-24 {
        return None;
    }
    let offset = v
        .scale(u.dot(u))
        .sub(u.scale(v.dot(v)))
        .cross(n)
        .scale(0.5 / n_squared);
    let center = a.add(offset);
    Some((center, n.normalized().ok()?, a.sub(center).length()))
}

/// Put a healed closed rim's PARAMETER ORIGIN on the neighbour's seam.
///
/// A closed rim edge starts and ends at one vertex, and on a periodic carrier
/// that vertex is not free: it is where the rim crosses the surface's seam, so
/// the seam meridian that ends there can still reach it. The re-intersection
/// that produced the rim does not know this. `intersect_plane_quadric` builds a
/// ruled-revolution or torus section with `frame_circle`, whose origin is the
/// carrier frame's own azimuth — the same azimuth `make_cylinder_brep` and
/// friends put the seam at, so those align by construction. Its SPHERE arm
/// instead starts the circle at `plane.normal.perpendicular()`, an azimuth
/// unrelated to the sphere's seam; a half-ball's equator comes back rotated a
/// quarter turn from where the dome's meridian ends.
///
/// So: leave the curve untouched when it already starts on the seam (the
/// aligned-by-construction cases stay byte-for-byte identical), and otherwise
/// rebuild it as the SAME circle re-origined at the seam. Everything is
/// verified — the rim must be a circle to within `tolerance` at every sample,
/// and every sample of the original must lie on the rebuilt curve — and any
/// miss refuses. A section that is not a circle (an oblique plane's ellipse)
/// has no such closed form and is refused rather than approximated.
pub(super) fn align_closed_rim_origin(
    curve: &NurbsCurve,
    seam: Vec3,
    tolerance: f64,
    op: &str,
) -> Result<NurbsCurve, KernelRefusal> {
    const SAMPLES: usize = 16;
    let evaluate = |t: f64| curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate");
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let start = evaluate(t0)?;
    let crossing = project_point_to_curve(curve, seam).or_refuse(KernelStage::Sew, "project")?;
    let target = evaluate(crossing.u)?;
    if start.sub(target).length() <= tolerance {
        return Ok(curve.clone());
    }
    let mut samples = Vec::with_capacity(SAMPLES);
    for index in 0..SAMPLES {
        samples.push(evaluate(t0 + (t1 - t0) * index as f64 / SAMPLES as f64)?);
    }
    let off_seam = start.sub(target).length();
    let decline = || {
        KernelRefusal::unsupported(
            KernelStage::Sew,
            "rim_origin",
            format!(
                "{op}: the healed rim starts {off_seam:.3e} from the neighbour's seam and is not \
             a circle that can be re-origined there — refusing rather than binding a closed \
             rim to a vertex off the seam (deferred)"
            ),
        )
    };
    let (center, normal, radius) =
        circle_through(samples[0], samples[SAMPLES / 3], samples[2 * SAMPLES / 3])
            .ok_or_else(decline)?;
    for sample in &samples {
        let radial = sample.sub(center);
        if (radial.length() - radius).abs() > tolerance || radial.dot(normal).abs() > tolerance {
            return Err(decline());
        }
    }
    let radial = target.sub(center);
    let x_axis = radial
        .sub(normal.scale(radial.dot(normal)))
        .normalized()
        .or_refuse(KernelStage::Sew, "rim_axis")?;
    let y_axis = normal.cross(x_axis);
    let rebuilt = crate::make_arc(center, x_axis, y_axis, radius, 0.0, std::f64::consts::TAU)
        .or_refuse(KernelStage::Sew, "make_arc")?;
    // The rebuilt rim must be the same point set, not merely a similar one.
    for sample in &samples {
        if project_point_to_curve(&rebuilt, *sample)
            .or_refuse(KernelStage::Sew, "project")?
            .distance
            > tolerance
        {
            return Err(decline());
        }
    }
    Ok(rebuilt)
}

/// Signed area of ONE of a face's trim loops in the carrier's own parameter
/// space, through the kernel's existing producer: `parameter_space_area`
/// integrates Green's theorem over a face's coedge pcurves exactly (Gauss
/// quadrature per knot span, so a rational arc is exact, not sampled), and a
/// face carrying the single loop is how every other caller asks it for one
/// loop — `offset_shell::carriers`, `healing::face_merge` and `thicken` all
/// spell it this way.
///
/// The sign is what this is for: on a validly trimmed face the loop that
/// bounds the material and the loops that punch holes in it wind OPPOSITE
/// ways. Which absolute sign means "outer" depends on the carrier's uv
/// handedness and on `same_sense`, and neither is worth assuming here — the
/// only claim made downstream is that two loops disagree.
pub(super) fn loop_signed_area(face: &FaceRecord, loop_index: usize) -> Result<f64, KernelRefusal> {
    parameter_space_area(&FaceRecord {
        id: face.id,
        surface: face.surface.clone(),
        same_sense: face.same_sense,
        loops: vec![face.loops[loop_index].clone()],
        name: None,
    })
    .or_refuse(KernelStage::Classify, "loop_area")
}

/// `(shell, face, loop)` of the loop `rim` occupies in `neighbour_id`, when
/// that loop is a HOLE the rim owns ALONE — the neighbour-side signature of a
/// through feature's wall.
///
/// Three things have to hold, and each rules out a transition strip:
///
/// * the rim appears in exactly ONE of the neighbour's loops, and is that
///   loop's only coedge — a fillet's rim on a periodic wall shares its loop
///   with the wall's seam and the wall's other rim, so it fails here;
/// * the neighbour carries at least one further loop — a cap disc whose only
///   loop is the rim has no material left once the loop goes, so it is a
///   multi-face gap, not a cap;
/// * the rim's loop winds AGAINST the neighbour's largest loop, i.e. it is a
///   hole in it rather than the loop that bounds its material.
fn rim_hole_loop(
    solid: &BrepSolid,
    neighbour_id: u64,
    rim: u64,
    op: &str,
) -> Result<Option<(usize, usize, usize)>, KernelRefusal> {
    let (shell, face_pos) = find_face(solid, neighbour_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "face",
            format!("{op}: missing face {neighbour_id}"),
        )
    })?;
    let face = &solid.shells[shell].faces[face_pos];
    if face.loops.len() < 2 {
        return Ok(None);
    }
    let mut carrying = face
        .loops
        .iter()
        .enumerate()
        .filter(|(_, loop_record)| {
            loop_record
                .coedges
                .iter()
                .any(|coedge| coedge.edge_id == rim)
        })
        .map(|(index, _)| index);
    let Some(loop_index) = carrying.next() else {
        return Ok(None);
    };
    if carrying.next().is_some() || face.loops[loop_index].coedges.len() != 1 {
        return Ok(None);
    }
    let mut areas = Vec::with_capacity(face.loops.len());
    for index in 0..face.loops.len() {
        areas.push(loop_signed_area(face, index)?);
    }
    let host = (0..areas.len())
        .max_by(|a, b| areas[*a].abs().total_cmp(&areas[*b].abs()))
        .expect("at least two loops");
    if host == loop_index || areas[host] * areas[loop_index] >= 0.0 {
        return Ok(None);
    }
    Ok(Some((shell, face_pos, loop_index)))
}

/// Both rims' hole loops, when BOTH rims are through-wall rims; `None` as soon
/// as either is not — a strip with one hole rim and one ordinary rim is a
/// transition strip (a fillet round a hole's mouth is exactly that), and it
/// belongs to the re-intersection path.
fn through_wall_hole_loops(
    solid: &BrepSolid,
    rims: &[u64],
    neighbour_ids: &[u64; 2],
    op: &str,
) -> Result<Option<[(usize, usize, usize); 2]>, KernelRefusal> {
    let mut found = Vec::with_capacity(2);
    for (index, rim) in rims.iter().enumerate() {
        match rim_hole_loop(solid, neighbour_ids[index], *rim, op)? {
            Some(position) => found.push(position),
            None => return Ok(None),
        }
    }
    Ok(Some([found[0], found[1]]))
}

/// The face id of a BLIND pocket's floor, when the strip could be that
/// pocket's wall: one rim is a hole loop (the mouth, in the face the pocket
/// was sunk into) and the other is the ENTIRE boundary of a single-loop
/// neighbour (the floor).
///
/// This is a NAME for a refusal, not a decision to refuse. A dome boss on a
/// plate wears the same shape — hole loop on the plate, one rim loop on the
/// dome — and its base fillet re-intersects (plane x sphere) perfectly well.
/// So the caller reads this only once the re-intersection has already failed,
/// to say WHICH configuration failed instead of naming the intersector.
fn blind_pocket_floor(
    solid: &BrepSolid,
    rims: &[u64],
    neighbour_ids: &[u64; 2],
    op: &str,
) -> Result<Option<u64>, KernelRefusal> {
    for near in 0..2usize {
        let far = 1 - near;
        if rim_hole_loop(solid, neighbour_ids[near], rims[near], op)?.is_none() {
            continue;
        }
        let (shell, face_pos) = find_face(solid, neighbour_ids[far]).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "face",
                format!("{op}: missing face {}", neighbour_ids[far]),
            )
        })?;
        let floor = &solid.shells[shell].faces[face_pos];
        if floor.loops.len() == 1
            && floor.loops[0].coedges.len() == 1
            && floor.loops[0].coedges[0].edge_id == rims[far]
        {
            return Ok(Some(neighbour_ids[far]));
        }
    }
    Ok(None)
}

/// Whether every face is reachable from the first by walking shared edges.
///
/// `validate()` deliberately does not ask this — a closed complex is checked
/// incidence by incidence — so a shell that has fallen into two closed pieces
/// passes it. The cap is the one heal that can create that, which is why the
/// question is asked here and not in the shared validator.
pub(super) fn faces_are_connected(faces: &[FaceRecord]) -> bool {
    if faces.is_empty() {
        return true;
    }
    let mut by_edge: HashMap<u64, Vec<usize>> = HashMap::default();
    for (index, face) in faces.iter().enumerate() {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            by_edge.entry(coedge.edge_id).or_default().push(index);
        }
    }
    let mut seen: HashSet<usize> = HashSet::default();
    let mut stack = vec![0usize];
    seen.insert(0);
    while let Some(index) = stack.pop() {
        for coedge in faces[index]
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            for neighbour in by_edge.get(&coedge.edge_id).into_iter().flatten() {
                if seen.insert(*neighbour) {
                    stack.push(*neighbour);
                }
            }
        }
    }
    seen.len() == faces.len()
}

/// Heal after deleting the WALL OF A THROUGH FEATURE by capping it: the wall
/// face, its private edges (two rims and the doubled seam) and their vertices
/// go, and each neighbour loses the hole loop the wall was bored through. The
/// neighbours' carriers and outer loops are untouched — a plane with its hole
/// loop removed is the whole plane again — so nothing is refit and the capped
/// faces are bit-identical to what they were before the feature was cut.
///
/// Two things are checked before the result is handed back, because a POST
/// joining two otherwise separate parts of a body wears exactly a through
/// wall's topological signature and capping it would SEVER the solid:
///
/// * the shell must still be CONNECTED — this is the exact statement of "the
///   wall was not the only thing holding the body together", and it is
///   checked directly because `validate()` does not test connectivity: two
///   closed surfaces wearing one shell record satisfy every incidence rule it
///   has;
/// * the GENUS must stay non-negative once decremented. Closing a through
///   hole removes exactly one handle, so the new genus is the old one less
///   one; a negative result means the input's own genus disagreed with its
///   topology, and `validate()`'s Euler check would only report that
///   afterwards, in terms of a formula rather than of the operation.
fn cap_through_wall(
    solid: &BrepSolid,
    shell_index: usize,
    face_index: usize,
    boundary: &[(u64, bool)],
    holes: [(usize, usize, usize); 2],
    op: &str,
) -> Result<BrepSolid, KernelRefusal> {
    let mut solid = solid.clone();
    let wall_edges: HashSet<u64> = boundary.iter().map(|(edge_id, _)| *edge_id).collect();

    // The hole loops go FIRST, by the positions already resolved against this
    // very topology. Dropping a loop cannot move a face, whereas dropping the
    // wall face renumbers every face behind it in its shell — and the two
    // neighbours are distinct faces (checked upstream), so neither removal
    // disturbs the other's index.
    for (shell, face_pos, loop_index) in holes {
        solid.shells[shell].faces[face_pos].loops.remove(loop_index);
    }
    solid.shells[shell_index].faces.remove(face_index);
    solid.edges.retain(|edge| !wall_edges.contains(&edge.id));
    let used: HashSet<u64> = solid
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    solid.vertices.retain(|vertex| used.contains(&vertex.id));

    if !faces_are_connected(&solid.shells[shell_index].faces) {
        return Err(KernelRefusal::unsupported(
            KernelStage::Validate,
            "sever",
            format!(
                "{op}: the deleted wall joins two otherwise separate parts of the body — \
             capping it would sever the solid, which this operation cannot represent \
             (deferred)"
            ),
        ));
    }
    solid.genus -= 1;
    if solid.genus < 0 {
        return Err(KernelRefusal::input(
            KernelStage::Validate,
            "genus",
            format!(
                "{op}: capping the wall leaves genus {}, so the solid's stated genus did not \
             account for the through feature it carries (deferred)",
                solid.genus
            ),
        ));
    }
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "cap_validation",
            format!("{op}: through-wall cap failed validation: {issues:?}"),
        ));
    }
    Ok(solid)
}

/// Heal after deleting a CLOSED transition strip (a fillet or chamfer that
/// runs around a full rim): the two closed rim edges each border one
/// neighbour; the neighbours' ANALYTIC carriers re-intersect in exactly one
/// closed curve — the re-sharpened rim. Both carriers extend exactly (a
/// plane widens, a ruled revolution grows along its axis with every pcurve
/// remapped affinely), so a fillet-then-heal round-trip reproduces the
/// original body bit-for-bit up to ids.
pub(super) fn heal_closed_transition(
    solid: &BrepSolid,
    shell_index: usize,
    face_index: usize,
    boundary: &[(u64, bool)],
) -> Result<BrepSolid, KernelRefusal> {
    if let Some(directory) = census_directory() {
        record_closed_heal_census(&directory, solid, shell_index, face_index, boundary);
    }
    let op = "delete_face_and_heal";
    let missing_neighbour = |id: u64| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "neighbour",
            format!("{op}: missing neighbour {id}"),
        )
    };
    let missing_rim = |id: u64| {
        KernelRefusal::internal(KernelStage::Collect, "rim", format!("{op}: missing rim {id}"))
    };
    let mut solid = solid.clone();
    let scale = solid_model_scale(&solid);
    let tolerance = (scale * 1e-6).max(1e-9);
    // The MARCHED lane's two numbers are the OPEN chain's, not this lane's:
    // `march_tolerance` is the corrector's convergence radius and the polyline
    // fit's chord tolerance, `plane_tolerance` the residual gate a fitted
    // section must stay inside. Every closed form below keeps `tolerance`, so
    // no rim that is answered exactly moves by a bit.
    let march_tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let face_id = solid.shells[shell_index].faces[face_index].id;

    // Classify the strip boundary: one OPEN seam edge used twice with
    // opposite senses, two distinct CLOSED rims.
    let mut counts: HashMap<u64, usize> = HashMap::default();
    for (edge_id, _) in boundary {
        *counts.entry(*edge_id).or_default() += 1;
    }
    let seam_id = *counts
        .iter()
        .find(|(_, count)| **count == 2)
        .map(|(id, _)| id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "seam",
                format!("{op}: closed transition without a doubled seam"),
            )
        })?;
    let rims: Vec<u64> = boundary
        .iter()
        .map(|(edge_id, _)| *edge_id)
        .filter(|edge_id| *edge_id != seam_id)
        .collect();
    if rims.len() != 2 || rims[0] == rims[1] {
        return Err(KernelRefusal::internal(
            KernelStage::Collect,
            "rims",
            format!("{op}: closed transition needs exactly two rims"),
        ));
    }
    for rim in &rims {
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.id == *rim)
            .ok_or_else(|| missing_rim(*rim))?;
        if edge.start_vertex_id != edge.end_vertex_id {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "open_rim",
                format!("{op}: rim {rim} is not closed"),
            ));
        }
    }

    // The two neighbours and their analytic carriers.
    let mut neighbour_ids = [0u64; 2];
    for (index, rim) in rims.iter().enumerate() {
        neighbour_ids[index] = other_face_of_edge(&solid, *rim, face_id)?;
    }
    if neighbour_ids[0] == neighbour_ids[1] {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "same_neighbour",
            format!("{op}: both rims border the same neighbour (deferred)"),
        ));
    }

    // Not every closed strip is a TRANSITION. When each rim is its
    // neighbour's whole hole loop, the strip is the WALL OF A THROUGH
    // FEATURE — a drilled hole's bore — and the neighbours were never meant
    // to meet: they are the two faces the feature punched through. Deleting
    // that wall means UN-punching it, so the heal is to cap: drop the wall
    // and both hole loops, and the neighbours close back over the opening.
    // This is checked BEFORE the re-intersection because it is a topological
    // signature, not a fallback — a strip with this shape has no sharp rim
    // to recover, and a pair of neighbours that happened to re-intersect
    // somewhere far away would be "sharpened" onto geometry the strip never
    // touched.
    if let Some(holes) = through_wall_hole_loops(&solid, &rims, &neighbour_ids, op)? {
        census_note("lane", || serde_json::json!("cap"));
        return cap_through_wall(&solid, shell_index, face_index, boundary, holes, op);
    }
    // A BLIND bore wears HALF that signature: the mouth rim is a hole loop,
    // but the far rim is the floor disc's whole boundary, so capping it would
    // leave the floor with no loop at all. That is a two-face gap this
    // one-face-at-a-time operation cannot close — but it is NOT grounds to
    // refuse here, because the same half-signature also fits a blend that
    // re-intersects perfectly well: a dome boss on a plate puts a hole loop on
    // the plate and a single rim loop on the dome, and its base fillet
    // sharpens against a plane x sphere circle. So the reading is only
    // prepared now and spent on the refusal path below, where the
    // re-intersection has already had its say.
    let blind_floor = blind_pocket_floor(&solid, &rims, &neighbour_ids, op)?;
    // The deferral's text; each site mints the class, and the blind-pocket
    // reading keeps the class of the refusal it re-words.
    let deferral = |reason: &str| match blind_floor {
        Some(floor) => format!(
            "{op}: the wall's far rim is the whole boundary of face {floor} — this is a \
             blind pocket, and its wall and floor are one PATCH: select that face too \
             and the whole pocket caps in a single delete"
        ),
        None => format!("{op}: {reason} (deferred)"),
    };

    // The strip's spatial extent: stations of BOTH rims at equal ARC LENGTH,
    // in the strip loop's own traversal order (`arc_length_stations`). The
    // healed rim lies within this region. These points size the neighbours'
    // extensions and the region gate, seed the march, and decide which
    // re-intersection is the rim, so they are read off where the rims ARE: a
    // step in the rim's PARAMETER would put them wherever its speed law does,
    // and one rim carries any number of speed laws. A closed rim's last station
    // is its first, so it is dropped.
    let mut strip_points: Vec<Vec3> = Vec::new();
    // Each rim's stations, and the sense they turn in along the EDGE's own
    // direction: what the rebind below reads the new rim's direction against.
    let mut rim_senses: Vec<Vec3> = Vec::with_capacity(2);
    for rim in &rims {
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.id == *rim)
            .ok_or_else(|| missing_rim(*rim))?;
        let forward = boundary
            .iter()
            .find(|(edge_id, _)| edge_id == rim)
            .map(|(_, forward)| *forward)
            .unwrap_or(true);
        let mut stations = arc_length_stations(edge, RIM_ARC_STATIONS + 1, forward)?;
        stations.pop();
        let sense = turning_sense(&stations);
        rim_senses.push(if forward { sense } else { sense.scale(-1.0) });
        strip_points.extend(stations);
    }
    // What each neighbour's carrier IS, read exactly as the open chain reads
    // its four (`open_heal.rs`): a plane, a recognized analytic, or a FITTED
    // patch that is neither. This one reading then picks the extension, the
    // intersector and the rebind, so the three cannot disagree about a face.
    let mut carriers: Vec<OpenNeighbourCarrier> = Vec::with_capacity(2);
    for &neighbour_id in &neighbour_ids {
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing_neighbour(neighbour_id))?;
        let surface = &solid.shells[ns].faces[nf].surface;
        carriers.push(match surface.analytic() {
            Some(AnalyticSurface::Plane { .. }) => OpenNeighbourCarrier::Planar(
                plane_of_surface(surface, plane_tolerance, op)?,
            ),
            Some(_) => OpenNeighbourCarrier::Curved,
            None => OpenNeighbourCarrier::FreeForm,
        });
    }
    // Whether either neighbour is fitted decides the whole lane below: an
    // all-analytic pair keeps the closed forms it has always had, bit for bit.
    let fitted_pair = carriers
        .iter()
        .any(|carrier| matches!(carrier, OpenNeighbourCarrier::FreeForm));

    // EXTEND-FIRST: a trimmed carrier stops at the strip's near rim, so
    // intersecting the trimmed surfaces finds nothing (the fillet removed
    // exactly the region where they meet). Grow every ruled-revolution
    // neighbour along its axis to cover the strip BEFORE intersecting;
    // planes are analytically unbounded and need no growth for the
    // intersection itself (retrim handles their extents afterwards).
    //
    // A FITTED neighbour is continued by its own terminal Bezier span through
    // `extend_freeform_neighbour_over` — the same call the open chain makes,
    // with the same verified coverage. That is what item 2 of the heal-tail
    // plan asks for and the only continuation that leaves a real surface with
    // a real domain behind the trim: `evaluate_extended` is a tangent plane a
    // Newton can converge onto anywhere.
    for (index, &neighbour_id) in neighbour_ids.iter().enumerate() {
        extend_ruled_neighbour_over(&mut solid, neighbour_id, &strip_points, tolerance)?;
        if matches!(carriers[index], OpenNeighbourCarrier::FreeForm) {
            extend_freeform_neighbour_over(
                &mut solid,
                neighbour_id,
                &strip_points,
                march_tolerance,
            )?;
        }
    }
    let neighbour_surface = |id: u64, solid: &BrepSolid| -> Result<NurbsSurface, KernelRefusal> {
        let (shell, face) = find_face(solid, id).ok_or_else(|| {
            KernelRefusal::internal(KernelStage::Collect, "face", format!("{op}: missing face {id}"))
        })?;
        Ok(solid.shells[shell].faces[face].surface.clone())
    };
    let surface_a = neighbour_surface(neighbour_ids[0], &solid)?;
    let surface_b = neighbour_surface(neighbour_ids[1], &solid)?;
    if std::env::var("BREP_DEBUG_HEAL").is_ok() {
        eprintln!(
            "HEAL neighbours {:?} analytic a={:?} b={:?}",
            neighbour_ids,
            surface_a.analytic().map(std::mem::discriminant),
            surface_b.analytic().map(std::mem::discriminant)
        );
    }
    // The strip's own region gate, used by both lanes below.
    let strip_centre = {
        let face = &solid.shells[shell_index].faces[face_index];
        face.surface.evaluate(0.5, 0.5).or_refuse(KernelStage::Classify, "evaluate")?
    };
    let strip_reach = strip_points
        .iter()
        .map(|point| point.sub(strip_centre).length())
        .fold(0.0f64, f64::max)
        * 3.0
        + tolerance;
    let new_curve = if !fitted_pair {
        let curves = intersect_analytic_pair(&surface_a, &surface_b, tolerance).ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Classify,
                "analytic_pair",
                deferral("neighbours are not a recognized analytic pair"),
            )
        })?;
        let closed: Vec<_> = curves
            .into_iter()
            .filter(|curve| {
                let [t0, t1] = curve.domain().unwrap_or([0.0, 1.0]);
                curve
                    .evaluate(t0)
                    .and_then(|a| curve.evaluate(t1).map(|b| a.sub(b).length()))
                    .map(|gap| gap <= tolerance)
                    .unwrap_or(false)
            })
            .collect();
        census_note("lane", || serde_json::json!("analytic"));
        census_note("candidates", || {
            let rows: Vec<serde_json::Value> = closed
                .iter()
                .map(|curve| {
                    let [t0, t1] = curve.domain().unwrap_or([0.0, 1.0]);
                    let mid = curve.evaluate(0.5 * (t0 + t1)).unwrap_or_default();
                    let worst = strip_points
                        .iter()
                        .map(|point| {
                            project_point_to_curve(curve, *point)
                                .map(|projection| projection.distance)
                                .unwrap_or(f64::INFINITY)
                        })
                        .fold(0.0f64, f64::max);
                    serde_json::json!({ "midpoint_to_centre": mid.sub(strip_centre).length(), "worst_station": worst })
                })
                .collect();
            serde_json::Value::Array(rows)
        });
        // Pick the re-intersection the strip was blended from (a cone x plane
        // pair can yield two circles; the healed rim is the one beside both
        // rims), by geometry, refusing a tie.
        match nearest_closed_rim(&closed, &strip_points, tolerance) {
            Ok(Some(index)) => closed[index].clone(),
            Ok(None) => {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "closed_rim",
                    deferral("neighbours do not re-intersect in a closed rim"),
                ))
            }
            Err((chosen, rival)) => return Err(closed_rim_tie(op, &neighbour_ids, &closed, chosen, rival, &strip_points)),
        }
    } else {
        // A fitted pair has no closed form. `reintersect_carriers` is the
        // shared, surface-type-blind seam the open chain already goes through:
        // it tries the analytic lane first (which declines here) and marches
        // otherwise, with a residual gate the caller sets. Seed it on the
        // strip's own rims — the rim being recovered runs right beside them.
        //
        // A PLANAR partner goes in as a patch covering the region gate rather
        // than as its own trimmed carrier: the marcher clamps to both domains,
        // and a plane patch that stops at the rim leaves the branch lying on
        // its own boundary (`planar_region_patch`). A plane has no extent of
        // its own — only its trim does — so widening it is exact.
        let widen = |carrier: &OpenNeighbourCarrier,
                     surface: &NurbsSurface|
         -> Result<NurbsSurface, KernelRefusal> {
            match carrier {
                OpenNeighbourCarrier::Planar(plane) => {
                    planar_region_patch(plane, strip_centre, strip_reach)
                }
                _ => Ok(surface.clone()),
            }
        };
        let first = widen(&carriers[0], &surface_a)?;
        let second = widen(&carriers[1], &surface_b)?;
        let policy = MarchPolicy {
            tolerance: march_tolerance,
            residual_tolerance: plane_tolerance,
            seeds: strip_points.clone(),
        };
        // The marcher's own verdict classes the refusal: carriers that do not
        // meet or a section that drifts off them are the pair declining a
        // closed form; a lower-level failure inside the march is its own.
        let rim = reintersect_carriers(&first, &second, &policy).map_err(|refusal| {
            let message = deferral(&format!(
                "the extended carriers do not re-intersect ({})",
                refusal.describe()
            ));
            match refusal {
                ReintersectRefusal::Failed(_) => {
                    KernelRefusal::internal(KernelStage::Refine, "marched_reintersection", message)
                }
                _ => KernelRefusal::unsupported(KernelStage::Refine, "marched_reintersection", message),
            }
        })?;
        census_note("lane", || serde_json::json!("marched"));
        census_note("candidates", || {
            let rows: Vec<serde_json::Value> = rim
                .sections
                .iter()
                .map(|section| {
                    let [t0, t1] = section.curve.domain().unwrap_or([0.0, 1.0]);
                    let samples: Vec<Vec3> = (0..24)
                        .map(|index| section.curve.evaluate(t0 + (t1 - t0) * index as f64 / 23.0).unwrap_or_default())
                        .collect();
                    let sampled = strip_points
                        .iter()
                        .map(|point| samples.iter().map(|sample| sample.sub(*point).length()).fold(f64::INFINITY, f64::min))
                        .fold(0.0f64, f64::max);
                    let projected = strip_points
                        .iter()
                        .map(|point| {
                            project_point_to_curve(&section.curve, *point)
                                .map(|projection| projection.distance)
                                .unwrap_or(f64::INFINITY)
                        })
                        .fold(0.0f64, f64::max);
                    serde_json::json!({ "closed": section.closed, "worst_sampled": sampled, "worst_station": projected, "points": section.polyline.len() })
                })
                .collect();
            serde_json::Value::Array(rows)
        });
        // Which branch is THIS rim is decided from the boundary it replaces,
        // never inside the intersector, and by the same geometric rating the
        // closed forms get: the worst distance from a rim station to the
        // branch, which separates two branches that pass near each other where
        // a midpoint pick does not.
        let curves = rim.curves();
        let section = match nearest_closed_rim(&curves, &strip_points, tolerance) {
            Ok(Some(index)) => &rim.sections[index],
            Ok(None) => {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "marched_closed_rim",
                    deferral("the extended carriers do not re-intersect"),
                ))
            }
            Err((chosen, rival)) => return Err(closed_rim_tie(op, &neighbour_ids, &curves, chosen, rival, &strip_points)),
        };
        if !section.closed {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "open_branch",
                deferral("the extended carriers' nearest re-intersection branch is not a closed rim"),
            ));
        }
        section.curve.clone()
    };

    // The rim's shared vertex must land on the SEAM of every periodic
    // neighbour, because that is where those neighbours' seam meridians end.
    // A planar neighbour has no seam and imposes nothing; two seamed
    // neighbours that disagree are not a configuration this heal can serve.
    let seam_target: Option<Vec3> = {
        let mut wanted: Option<Vec3> = None;
        for (index, rim) in rims.iter().enumerate() {
            let neighbour_id = neighbour_ids[index];
            let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing_neighbour(neighbour_id))?;
            let surface = &solid.shells[ns].faces[nf].surface;
            if matches!(surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
                continue;
            }
            if surface.analytic().is_none() {
                // A FITTED patch open in both directions has no seam either:
                // its rim is a closed loop inside its own parameter square and
                // no meridian ends anywhere on it, so it constrains nothing.
                // One that IS closed in a direction does have a branch cut, and
                // nothing here knows where — refused by name rather than
                // guessed at, because binding a rim to the wrong azimuth is
                // wrong by up to a diameter while looking plausible.
                let (closed_u, closed_v) =
                    surface.closed_directions().or_refuse(KernelStage::Classify, "domain")?;
                if closed_u || closed_v {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "fitted_seam",
                        format!(
                            "{op}: fitted neighbour {neighbour_id} is closed in its own parameter \
                         square, so its rim must start on a seam this heal cannot locate \
                         (deferred)"
                        ),
                    ));
                }
                continue;
            }
            let edge = solid
                .edges
                .iter()
                .find(|edge| edge.id == *rim)
                .ok_or_else(|| missing_rim(*rim))?;
            let seam_point = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Classify, "evaluate")?;
            match wanted {
                None => wanted = Some(seam_point),
                Some(known) if known.sub(seam_point).length() <= tolerance => {}
                Some(_) => {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "seams_disagree",
                        format!(
                            "{op}: the strip's two curved neighbours put their seams in different \
                         places — the healed rim cannot start on both (deferred)"
                        ),
                    ))
                }
            }
        }
        wanted
    };
    let new_curve = match seam_target {
        Some(seam) => align_closed_rim_origin(&new_curve, seam, tolerance, op)?,
        None => new_curve,
    };

    // New shared rim edge + its seam vertex.
    let mut next_id = max_topology_id(&solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };
    let [nt0, nt1] = new_curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let new_vertex_id = alloc();
    let new_point = new_curve.evaluate(nt0).or_refuse(KernelStage::Sew, "evaluate")?;
    solid.vertices.push(VertexRecord {
        id: new_vertex_id,
        point: new_point,
    });
    let new_edge_id = alloc();
    solid.edges.push(EdgeRecord {
        id: new_edge_id,
        curve: new_curve.clone(),
        t0: nt0,
        t1: nt1,
        start_vertex_id: new_vertex_id,
        end_vertex_id: new_vertex_id,
        degenerate: false,
        name: None,
    });

    // The sense the new rim turns in along its own direction, read at equal
    // arc length like the old rims' senses.
    let new_sense = {
        let record = solid
            .edges
            .iter()
            .find(|edge| edge.id == new_edge_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "healed_rim",
                    format!("{op}: missing healed rim {new_edge_id}"),
                )
            })?;
        let mut stations = arc_length_stations(record, RIM_ARC_STATIONS + 1, true)?;
        stations.pop();
        turning_sense(&stations)
    };

    // Rebind each neighbour's rim coedge onto the new edge, extending the
    // carrier when the new rim lies outside its domain.
    for (index, rim) in rims.iter().enumerate() {
        let neighbour_id = neighbour_ids[index];
        let (shell, face_pos) = find_face(&solid, neighbour_id).ok_or_else(|| missing_neighbour(neighbour_id))?;
        // Direction of the loop's walk along the OLD rim, preserved for the
        // new one: coaxial closed rims keep azimuth direction under nearest
        // projection.
        let face = &solid.shells[shell].faces[face_pos];
        let old_coedge = face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .find(|coedge| coedge.edge_id == *rim)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "rim_coedge",
                    format!("{op}: neighbour lost its rim coedge"),
                )
            })?;
        let old_forward = old_coedge.forward;
        // The new coedge runs round the new rim in the sense the old one ran
        // round the old rim. Both senses are read off the curves at equal arc
        // length (`turning_sense`, the closed band's reading), so they are
        // properties of where the rims are and not of how either is
        // parameterised: a quarter of the old rim's PARAMETER range is a
        // quarter turn only for a uniform speed law, and a re-weighted rim put
        // that point past the half turn, bound the new coedge backwards, and
        // failed validation as two coedges in the same sense.
        let old_sense = if old_forward {
            rim_senses[index]
        } else {
            rim_senses[index].scale(-1.0)
        };
        let agreement = old_sense.dot(new_sense);
        if agreement.abs() <= 1e-6 * old_sense.length() * new_sense.length() {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Sew,
                "rim_sense",
                format!(
                    "{op}: the healed rim does not turn about the same axis as rim {rim} of neighbour \
                 {neighbour_id}, so which way its coedge runs cannot be read from the old one \
                 (deferred)"
                ),
            ));
        }
        let new_forward = agreement > 0.0;
        census_push("rebind", || {
            serde_json::json!({ "rim": rim, "agreement": agreement, "forward": new_forward })
        });

        let is_plane = matches!(face.surface.analytic(), Some(AnalyticSurface::Plane { .. }));
        if is_plane {
            let plane = plane_of_surface(&face.surface, tolerance, op)?;
            let face = &mut solid.shells[shell].faces[face_pos];
            for coedge in face
                .loops
                .iter_mut()
                .flat_map(|loop_record| &mut loop_record.coedges)
            {
                if coedge.edge_id == *rim {
                    coedge.edge_id = new_edge_id;
                    coedge.forward = new_forward;
                }
            }
            let edges_by_id: HashMap<u64, EdgeRecord> = solid
                .edges
                .iter()
                .map(|edge| (edge.id, edge.clone()))
                .collect();
            let face = &mut solid.shells[shell].faces[face_pos];
            retrim_planar_face(face, &plane, &edges_by_id, scale, op)?;
            continue;
        }
        // The rebound rim is the carrier's `v = const` ISO-CURVE at the new
        // station, and the whole rebind is that one number. A ruled
        // revolution states it in closed form; every other revolution-family
        // carrier — sphere, torus, general revolution — is MEASURED off the
        // carrier by projecting the healed rim onto it. Nothing else about
        // this branch is carrier-specific, so the ruled-only gate that used
        // to stand here was narrower than the intersector at line 100: for a
        // Sphere×Plane or Torus×Plane pair `intersect_analytic_pair` returns
        // the exact closed rim and the heal then refused to bind it.
        // A rim produced by a CLOSED FORM between two revolution-family
        // carriers is a `v = const` iso-curve of each by construction, and
        // then the whole rebind is that one number. A rim the MARCHER produced
        // is not one in general: it is the section of a fitted carrier and it
        // crosses parameter lines, so laying a straight iso pcurve under it
        // would be a trim that is wrong while looking plausible. There the
        // trim is FITTED — from the same edge curve, over the same subrange,
        // on both incident faces.
        let mut ruled_arm = false;
        let station = match face.surface.analytic() {
            Some(AnalyticSurface::RuledRevolution { frame, height, .. }) => {
                if fitted_pair && rim_iso_station(&face.surface, &new_curve, tolerance).is_none() {
                    None
                } else {
                    ruled_arm = true;
                    // The pre-extension covered the strip, so the new rim's
                    // station is inside the (already extended) carrier.
                    let axial = new_point.sub(frame.origin).dot(frame.axis);
                    if !(-tolerance..=height + tolerance).contains(&axial) {
                        return Err(KernelRefusal::internal(
                            KernelStage::Sew,
                            "rim_escapes_carrier",
                            format!(
                                "{op}: healed rim escapes the extended carrier \
                             (station {axial:.6} of {height:.6})"
                            ),
                        ));
                    }
                    Some((axial / *height).clamp(0.0, 1.0))
                }
            }
            Some(_) => rim_iso_station(&face.surface, &new_curve, tolerance),
            // A fitted carrier's parameter lines are its fitter's, not the
            // geometry's; the rim is bound by a fit, never by a station.
            None => None,
        };
        let Some(v_new) = station else {
            if !fitted_pair {
                // An all-analytic pair keeps its own refusals, word for word.
                return Err(match face.surface.analytic() {
                    Some(_) => KernelRefusal::unsupported(
                        KernelStage::Sew,
                        "rim_not_iso",
                        format!(
                            "{op}: the healed rim is not a `v = const` iso-curve of neighbour \
                         {neighbour_id}'s carrier — refusing rather than binding it to a \
                         parameter line it does not follow (deferred)"
                        ),
                    ),
                    None => KernelRefusal::unsupported(
                        KernelStage::Sew,
                        "freeform_neighbour",
                        format!(
                            "{op}: curved neighbour {neighbour_id} is a free-form surface (deferred)"
                        ),
                    ),
                });
            }
            // On a PERIODIC carrier the rim still has to start on the seam —
            // a pcurve that begins mid-domain traces the rim from the wrong
            // azimuth — so measure that here as the iso path measures it.
            let [u_low, u_high] = face.surface.domain_u().or_refuse(KernelStage::Sew, "domain")?;
            let (closed_u, _) = face.surface.closed_directions().or_refuse(KernelStage::Sew, "domain")?;
            if closed_u {
                let rim_start = crate::project_point_to_surface(&face.surface, new_point)
                    .or_refuse(KernelStage::Sew, "project")?;
                let u_span = (u_high - u_low).abs().max(1e-12);
                if (rim_start.u - u_low).abs() > 1e-6 * u_span
                    && (rim_start.u - u_high).abs() > 1e-6 * u_span
                {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Sew,
                        "rim_off_seam_fitted",
                        format!(
                            "{op}: the healed rim starts at u={:.6} of periodic neighbour \
                         {neighbour_id}'s [{u_low:.6}, {u_high:.6}] domain, not at its seam — \
                         a fitted pcurve would cross the seam branch (deferred)",
                            rim_start.u
                        ),
                    ));
                }
            }
            let surface = face.surface.clone();
            let face = &mut solid.shells[shell].faces[face_pos];
            for coedge in face
                .loops
                .iter_mut()
                .flat_map(|loop_record| &mut loop_record.coedges)
            {
                if coedge.edge_id == *rim {
                    coedge.edge_id = new_edge_id;
                    coedge.forward = new_forward;
                    coedge.pcurve = build_pcurve_on_surface_range(
                        &surface,
                        &new_curve,
                        nt0,
                        nt1,
                        new_forward,
                        march_tolerance,
                    )
                    .or_refuse(KernelStage::Sew, "pcurve_fit")?;
                }
            }
            continue;
        };
        let [u_low, u_high] = face.surface.domain_u().or_refuse(KernelStage::Sew, "domain")?;
        // A `v = const` pcurve running the WHOLE u range is only the rim's
        // trace if the rim starts where that u range starts. On a carrier
        // whose seam sits at u_low that is what `align_closed_rim_origin`
        // arranged, but arranging is not knowing: measure it. A rim that
        // begins mid-domain traces the same circle from a different azimuth,
        // and the straight pcurve would be that circle rotated — geometry
        // that is wrong by up to a diameter while looking perfectly plausible.
        let rim_start = crate::project_point_to_surface(&face.surface, new_point)
            .or_refuse(KernelStage::Sew, "project")?;
        let u_span = (u_high - u_low).abs().max(1e-12);
        let at_u_end = (rim_start.u - u_low).abs() <= 1e-6 * u_span
            || (rim_start.u - u_high).abs() <= 1e-6 * u_span;
        if !at_u_end {
            return Err(KernelRefusal::unsupported(
                KernelStage::Sew,
                "rim_off_seam",
                format!(
                    "{op}: the healed rim starts at u={:.6} of neighbour {neighbour_id}'s                  [{u_low:.6}, {u_high:.6}] domain, not at its seam — a whole-domain iso                  pcurve would trace the rim from the wrong azimuth (deferred)",
                    rim_start.u
                ),
            ));
        }
        // Which way the rim's traversal runs in the carrier's OWN u.
        //
        // For a ruled revolution the answer is known by construction: the rim
        // came from `frame_circle` on the same frame `make_revolution` built
        // the carrier from, so the coedge's sense IS the u sense. Nowhere else
        // is that guaranteed — a sphere's u may wind the opposite way round
        // the same axis — and a `v = const` pcurve laid down the wrong way
        // traces the rim mirrored, matching at both ends and a full diameter
        // out at the quarter turns. So measure it: project the quarter point
        // of the TRAVERSAL and see which end of the u domain it sits nearer.
        let ascending = if ruled_arm {
            new_forward
        } else {
            let [rim_t0, rim_t1] = new_curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            let fraction = if new_forward { 0.25 } else { 0.75 };
            let quarter = new_curve
                .evaluate(rim_t0 + (rim_t1 - rim_t0) * fraction)
                .or_refuse(KernelStage::Sew, "evaluate")?;
            let quarter_u = crate::project_point_to_surface(&face.surface, quarter)
                .or_refuse(KernelStage::Sew, "project")?
                .u;
            (quarter_u - u_low).abs() < (quarter_u - u_high).abs()
        };
        let face = &mut solid.shells[shell].faces[face_pos];
        for coedge in face
            .loops
            .iter_mut()
            .flat_map(|loop_record| &mut loop_record.coedges)
        {
            if coedge.edge_id == *rim {
                coedge.edge_id = new_edge_id;
                coedge.forward = new_forward;
                // The new rim is the v-iso at its own station.
                let (from_u, to_u) = if ascending {
                    (u_low, u_high)
                } else {
                    (u_high, u_low)
                };
                coedge.pcurve = make_line(Vec3::new(from_u, v_new, 0.0), Vec3::new(to_u, v_new, 0.0))
                    .or_refuse(KernelStage::Sew, "make_line")?;
            }
        }
    }

    // Collapse the old rim seam vertices onto the new vertex and extend the
    // straight edges that ended there (a wall's meridian grows to the new
    // rim station).
    let mut collapsed: HashSet<u64> = HashSet::default();
    for rim in &rims {
        if let Some(edge) = solid.edges.iter().find(|edge| edge.id == *rim) {
            collapsed.insert(edge.start_vertex_id);
        }
    }
    let strip_edges: HashSet<u64> = boundary.iter().map(|(edge_id, _)| *edge_id).collect();
    let mut extended_edges: HashMap<u64, (bool, bool)> = HashMap::default();
    // Extended edges whose pcurves must be REFIT against their faces' carriers
    // rather than repaired by sliding one control point's v. Two kinds reach
    // it. A CURVED meridian widened along itself: a rational arc is not linear
    // in the surface's v, so the straight two-point pcurve that serves a line
    // meridian would cut the chord in parameter space. And a straight meridian
    // that had to be REBUILT because it was stored with more than two control
    // points: its pcurve carries the same redundant control, so the two-point
    // repair below does not apply to it either.
    let mut refit_edges: HashSet<u64> = HashSet::default();
    for edge in &mut solid.edges {
        if strip_edges.contains(&edge.id) || edge.id == new_edge_id {
            continue;
        }
        let start_hit = collapsed.contains(&edge.start_vertex_id);
        let end_hit = collapsed.contains(&edge.end_vertex_id);
        if !(start_hit || end_hit) {
            continue;
        }
        if start_hit {
            edge.start_vertex_id = new_vertex_id;
        }
        if end_hit {
            edge.end_vertex_id = new_vertex_id;
        }
        if edge.curve.straight_segment(tolerance).is_some() {
            // A STRAIGHT meridian (a cylinder's or cone's wall seam) is
            // rebuilt from its endpoints: the prolonged line is the same line,
            // so this is exact. Straightness is measured, not read off the
            // control count — see `NurbsCurve::straight_segment`.
            let redundant_controls = edge.curve.control_points.len() > 2;
            let anchor = if start_hit {
                edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?
            } else {
                edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?
            };
            let (from, to) = if start_hit {
                (new_point, anchor)
            } else {
                (anchor, new_point)
            };
            edge.curve = make_line(from, to).or_refuse(KernelStage::Sew, "make_line")?;
            [edge.t0, edge.t1] = edge.curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            if redundant_controls {
                refit_edges.insert(edge.id);
            }
        } else {
            // A CURVED seam meridian — a dome's is an arc — must be WIDENED
            // along its own curve. Rebuilding it as a line, which is what this
            // branch used to do unconditionally, would replace the arc with
            // its chord: an edge that no longer lies on either carrier, i.e.
            // exactly the silently-wrong solid the fail-safe contract forbids.
            // When the stored curve does not reach the new station there is
            // nothing honest to widen into, so refuse.
            let projection =
                project_point_to_curve(&edge.curve, new_point).or_refuse(KernelStage::Sew, "project")?;
            if projection.distance > tolerance {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "curved_seam_short",
                    format!(
                        "{op}: curved seam edge {} does not reach the healed rim (nearest \
                     point {:.3e} away) — refusing rather than replacing it with a straight \
                     chord (deferred)",
                        edge.id, projection.distance
                    ),
                ));
            }
            if start_hit {
                edge.t0 = projection.u;
            } else {
                edge.t1 = projection.u;
            }
            let [d0, d1] = edge.curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            if edge.t1 - edge.t0 <= 1e-9 * (d1 - d0).max(1e-12) {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "curved_seam_collapse",
                    format!(
                        "{op}: healing would collapse curved seam edge {} to zero length",
                        edge.id
                    ),
                ));
            }
            refit_edges.insert(edge.id);
        }
        extended_edges.insert(edge.id, (start_hit, end_hit));
    }
    // Refit every pcurve of those edges over its new range, on whatever
    // carrier each referencing face has. Same mechanism the open chain's
    // `refit_touched_pcurves` uses, and for the same reason.
    if !refit_edges.is_empty() {
        let edges_by_id: HashMap<u64, EdgeRecord> = solid
            .edges
            .iter()
            .map(|edge| (edge.id, edge.clone()))
            .collect();
        for shell in &mut solid.shells {
            for face in &mut shell.faces {
                refit_touched_pcurves(face, &edges_by_id, &refit_edges, false, tolerance, op)?;
            }
        }
    }
    // The pcurves referencing an extended edge still stop at the OLD rim
    // station; pull the affected endpoint to the new station. The meridian
    // pcurves are straight two-point lines pinned to a seam branch — moving
    // only the endpoint's v keeps the branch pinning intact.
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            let station = match face.surface.analytic() {
                Some(AnalyticSurface::RuledRevolution { frame, height, .. }) => {
                    Some((new_point.sub(frame.origin).dot(frame.axis) / height).clamp(0.0, 1.0))
                }
                _ => None,
            };
            let Some(v_new) = station else { continue };
            for coedge in face
                .loops
                .iter_mut()
                .flat_map(|loop_record| &mut loop_record.coedges)
            {
                if refit_edges.contains(&coedge.edge_id) {
                    continue; // already refit against the carrier above
                }
                let Some(&(start_moved, end_moved)) = extended_edges.get(&coedge.edge_id) else {
                    continue;
                };
                if coedge.pcurve.degree != 1 || coedge.pcurve.control_points.len() != 2 {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Sew,
                        "non_line_pcurve",
                        format!("{op}: extended edge has a non-line pcurve (deferred)"),
                    ));
                }
                // Traversal start maps to the edge start when forward.
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
        }
    }

    // Drop the strip and its private topology.
    solid.shells[shell_index].faces.remove(face_index);
    solid.edges.retain(|edge| !strip_edges.contains(&edge.id));
    let used: HashSet<u64> = solid
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    solid.vertices.retain(|vertex| used.contains(&vertex.id));

    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validation",
            format!("{op}: closed-transition heal failed validation: {issues:?}"),
        ));
    }
    Ok(solid)
}
