use super::*;
use crate::RevolutionFrame;
use crate::{KernelRefusal, KernelStage, OrRefuse};

// ---------------------------------------------------------------------------
// Move Face by CARRIER RE-INTERSECTION — the operator's construction.
// ---------------------------------------------------------------------------
//
// > "I need the infinite faces of the move face selection to go and intersect
// > themselves with the rest of the solid as the thing gets moved."
//
// `face_move.rs` ENUMERATES CASES. It classifies each boundary edge by the
// carrier pair it joins and picks one of four actions, and every configuration
// nobody enumerated becomes a refusal — which is the root cause the
// 2026-09-21 capability matrix convicts in three places (an in-plane slide
// that refuses because a NEIGHBOUR is curved; a bore wall pushed along its own
// invariant axis; acceptance that is not monotone in the motion).
//
// This road has no case table over configurations. It has exactly three steps,
// and each one asks the same question of every face regardless of what kind of
// surface it carries:
//
//   1. **Carriers.** Every face's carrier is taken UNBOUNDED — stretched to
//      cover the whole body plus the motion — and the selected faces' carriers
//      are moved. Nothing is trimmed at this stage; the body supplies the
//      extent later, which is the whole point.
//   2. **Corners.** Every vertex the moved group touches goes where the rigid
//      motion would put it, corrected by the LEAST amount that lands it on
//      every carrier it must lie on. That one sentence replaces the old road's
//      three-plane solve, its two rim affines, its "ride the fixed edge" lane
//      and its invariant-carrier lane. A corner whose carriers do not meet
//      near the rigid guess refuses, by name.
//   3. **Edges and trims.** Every edge whose geometry the motion disturbed is
//      rebuilt as the section of its own two carriers between its own two
//      re-solved corners, and every face the rebuild touched regrows its
//      carrier around its new boundary and refits its pcurves.
//
// ## What is dispatched on, and what is not
//
// Step 3 does dispatch on the SURFACE PAIR — a plane pair sections in a line, a
// recognized analytic pair sections in closed form
// (`intersect_analytic_pair`), anything else is marched. That is a geometry
// library's business and it is exhaustive by fallback: the marcher has no
// "unsupported pair". What this road does NOT dispatch on is the
// CONFIGURATION — which side of the edge moved, whether the neighbour is
// curved, whether the push is parallel to an axis, how many fixed faces meet
// at a corner. Those are the distinctions the old road's refusals are made of.
//
// ## Honest scope of this slice
//
// * **Topology is preserved.** A motion that needs the topology to CHANGE — a
//   bore breaking out through a new face, a migrating exit hole — is not
//   attempted here; it refuses. Slice 1 measures the geometry road.
// * **Free-form carriers are not stretched.** A fitted NURBS patch keeps its
//   own domain, so a section that leaves it refuses instead of extrapolating.
// * The entry is `move_faces` under `BREP_FACE_MOVE_ROAD=carrier`. Nothing is
//   rewired: with the variable unset, not one byte of the old road's behaviour
//   changes.

/// Which road `move_faces` takes. The old one unless `BREP_FACE_MOVE_ROAD` is
/// set to `carrier` — so an unset environment is the tree's behaviour exactly.
pub(super) fn carrier_road_selected() -> bool {
    std::env::var("BREP_FACE_MOVE_ROAD").as_deref() == Ok("carrier")
}

const OP: &str = "move_faces(carrier)";

/// How a vertex is allowed to sit off the carriers it was solved against,
/// and how far a rebuilt edge may sit off its own section, as a multiple of
/// the model tolerance. Deliberately the same slack the old road grants its
/// own re-solved corners (`10.0 * tolerance`).
const RESIDUAL_SLACK: f64 = 10.0;

/// The section two carriers share, in the only two shapes a caller here needs.
enum Section {
    /// A plane pair. `intersect_analytic_pair` deliberately declines this one
    /// (its doc says so: a synthesized over-long segment interacts badly with
    /// the shared-parameter pcurve contract), and marching it would fit a
    /// straight line to a polyline and lose the exactness every planar closed
    /// form in the capability matrix is measured to. So a plane pair is built
    /// as the straight segment between the corners the solve already placed —
    /// which IS the section restricted to them, because both corners were
    /// solved onto both planes.
    Straight,
    /// Anything else: the exact closed form where the pair has one, the marched
    /// trace where it does not. `reintersect_carriers` owns that choice.
    Curve(NurbsCurve),
}

/// Move a face group by re-intersecting its carriers with the rest of the
/// solid. See the module header for the construction; `move_faces` routes here
/// under `BREP_FACE_MOVE_ROAD=carrier`.
pub(super) fn move_faces_by_carrier(
    solid: &BrepSolid,
    moved: &HashSet<u64>,
    translation: Vec3,
) -> Result<BrepSolid, KernelRefusal> {
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let motion = translate_affine(translation).or_refuse(KernelStage::Collect, "translate_affine")?;

    let mut face_lookup: HashMap<u64, (usize, usize)> = HashMap::default();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            face_lookup.entry(face.id).or_insert((shell_index, face_index));
        }
    }

    let mut faces_of_edge: HashMap<u64, Vec<u64>> = HashMap::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    faces_of_edge.entry(coedge.edge_id).or_default().push(face.id);
                }
            }
        }
    }
    for edge in &solid.edges {
        let uses = faces_of_edge.get(&edge.id).map(Vec::as_slice).unwrap_or(&[]);
        let expected = if edge.degenerate { 1 } else { 2 };
        if uses.len() != expected {
            return Err(KernelRefusal::input(KernelStage::Collect, "non_manifold", format!(
                "{OP}: edge {} is used {} times (non-manifold input)",
                edge.id,
                uses.len()
            )));
        }
    }

    // --- 1. The carriers, unbounded, and already moved ---------------------
    let (centre, radius) = body_ball(solid)?;
    // The reach a carrier has to cover for the body to be able to supply the
    // extent: the whole body, wherever the motion carries it, with room to
    // spare. This is what "infinite" means in a kernel that stores patches.
    let reach = (radius + translation.length()) * 4.0 + scale;
    let mut carriers: HashMap<u64, NurbsSurface> = HashMap::default();
    // Which SELECTED faces the motion actually moves the CARRIER of.
    //
    // A motion can be a real motion of the face and the identity on its
    // carrier: a plane slid inside itself, a cylinder pushed along its own
    // axis, a face spun about its own normal. The contract already says what
    // those answer — "the identity, geometry and topology unchanged" — and the
    // old road refuses two of them, which is category 4 of the capability
    // matrix. This is the reading that makes them the identity here, and it is
    // a MEASUREMENT applied to every selected face alike, not a list of the
    // configurations it holds in: sample the face's own patch, carry the
    // samples by the motion, and ask whether they are still on the carrier
    // they came from. Nothing in it mentions what kind of surface that is.
    //
    // The threshold is much tighter than the model tolerance on purpose. At
    // model tolerance a 1e-9 push of a box top reads as invariant and the road
    // answers with the input body, which is a DIFFERENT answer from the one the
    // old road gives (it moves the face by 1e-9). Whether a sub-tolerance drag
    // ought to be the identity is a contract question and not this slice's to
    // settle, so the reading is set where it cannot decide it: five orders
    // below the smallest motion any cell asks for, and four above the
    // floating-point noise a genuinely invariant carrier's own samples carry.
    let invariance_tolerance = (scale * 1e-11).max(1e-13);
    let mut moves_carrier: HashSet<u64> = HashSet::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            let stretched = unbounded_carrier(&face.surface, centre, reach, plane_tolerance)?;
            let carrier = if moved.contains(&face.id)
                && !carrier_invariant_under_motion(
                    &face.surface,
                    &stretched,
                    motion,
                    invariance_tolerance,
                ).or_refuse(KernelStage::Classify, "carrier_invariant_under_motion")?
            {
                moves_carrier.insert(face.id);
                transform_surface(&stretched, motion).or_refuse(KernelStage::Classify, "transform_surface")?
            } else {
                stretched
            };
            carriers.insert(face.id, carrier);
        }
    }

    // --- 2. The corners ----------------------------------------------------
    let mut vertex_faces: HashMap<u64, HashSet<u64>> = HashMap::default();
    for edge in &solid.edges {
        if let Some(uses) = faces_of_edge.get(&edge.id) {
            for vertex_id in [edge.start_vertex_id, edge.end_vertex_id] {
                vertex_faces.entry(vertex_id).or_default().extend(uses.iter().copied());
            }
        }
    }
    let mut new_vertex: HashMap<u64, Vec3> = HashMap::default();
    for vertex in &solid.vertices {
        let Some(adjacent) = vertex_faces.get(&vertex.id) else {
            continue;
        };
        if !adjacent.iter().any(|face_id| moved.contains(face_id)) {
            continue;
        }
        // THE RULE, and there is only one: the corner goes where the rigid
        // motion would put it, corrected by the least amount that lands it on
        // every carrier it must lie on. When every adjacent face moved, the
        // rigid guess is already on all of them and the correction is zero.
        // When the motion leaves a carrier invariant — a plane slid in itself,
        // a cylinder pushed along its own axis — the guess is off it and the
        // correction is exactly the slide back, which is why an identity
        // motion comes out as the identity here with nothing written for it.
        let mut at: Vec<&NurbsSurface> = Vec::with_capacity(adjacent.len());
        let mut ids: Vec<u64> = adjacent.iter().copied().collect();
        ids.sort_unstable();
        for face_id in &ids {
            at.push(carriers.get(face_id).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Refine,
                    "carrier_lookup",
                    format!("{OP}: no carrier for face {face_id} at vertex {}", vertex.id),
                )
            })?);
        }
        // The corner is only forced to move by a carrier the motion CHANGES.
        // When every carrier meeting here is one the motion maps onto itself,
        // the constraint set is the one the corner already satisfies and the
        // rigid guess would be a displacement nothing asked for.
        let seed = if ids.iter().any(|face_id| moves_carrier.contains(face_id)) {
            vertex.point.add(translation)
        } else {
            vertex.point
        };
        if std::env::var("BREP_FACE_MOVE_CARRIER_DEBUG").is_ok() {
            eprintln!("CORNER v{} at faces {:?}  seed {:?}", vertex.id, ids, seed);
            for (face_id, surface) in ids.iter().zip(at.iter()) {
                let kind = match surface.analytic() {
                    Some(AnalyticSurface::Plane { .. }) => "plane",
                    Some(AnalyticSurface::RuledRevolution { .. }) => "ruled",
                    Some(AnalyticSurface::Sphere { .. }) => "sphere",
                    Some(AnalyticSurface::Torus { .. }) => "torus",
                    Some(AnalyticSurface::Revolution { .. }) => "revolution",
                    None => "free-form",
                };
                match crate::project_point_to_surface(surface, seed) {
                    Ok(projection) => eprintln!(
                        "    face {face_id} [{kind}] foot {:?} dist {:.6e} uv ({:.6},{:.6}) normal {:?}",
                        projection.point,
                        projection.point.sub(seed).length(),
                        projection.u,
                        projection.v,
                        surface.normal(projection.u, projection.v),
                    ),
                    Err(error) => eprintln!("    face {face_id} [{kind}] projection FAILED: {error}"),
                }
            }
        }
        let corner = solve_corner(&at, seed, tolerance).map_err(|why| {
            // Named the way the user picked the faces, not by internal id. The
            // capability matrix graded every refusal this operation writes and
            // split them exactly on this: the messages the FEATURE writes name
            // faces the user can see, and the messages the PRIMITIVE writes
            // name `edge 18`, `vertex 9`, `vertex 21` — ids a user cannot
            // select and which change between runs of the same document.
            let named: Vec<String> = ids
                .iter()
                .map(|face_id| face_label(solid, &face_lookup, *face_id))
                .collect();
            let split = if ids.len() > 3 {
                " — more than three carriers met there, and a corner like that cannot stay \
                 ONE corner once any of them moves: the right answer splits it, which is a \
                 topology change this operation does not make"
            } else {
                ""
            };
            why.with_message(|why| format!(
                "{OP}: the {} carriers meeting at the corner of {} no longer share a point \
                 after the motion ({why}){split}",
                ids.len(),
                named.join(", ")
            ))
        })?;
        new_vertex.insert(vertex.id, corner);
    }

    // --- 3. The edges ------------------------------------------------------
    // An edge is DIRTY when the motion disturbed its geometry: either a face it
    // bounds moved, or a corner it ends on did. The second half is what lets a
    // FIXED edge between two fixed carriers be re-trimmed (and, when it must,
    // re-derived from those same two carriers past its old span) instead of
    // being the thing a push refuses for not re-crossing within its span.
    let mut result = solid.clone();
    let mut dirty: HashSet<u64> = HashSet::default();
    let mut rebuilt: HashMap<u64, (NurbsCurve, f64, f64)> = HashMap::default();
    let mut order: Vec<&EdgeRecord> = Vec::new();
    for edge in &solid.edges {
        let uses = faces_of_edge.get(&edge.id).map(Vec::as_slice).unwrap_or(&[]);
        let moved_uses = uses.iter().filter(|face_id| moves_carrier.contains(*face_id)).count();
        let corner_moved = |vertex_id: u64| {
            new_vertex
                .get(&vertex_id)
                .is_some_and(|point| point.sub(point_of(solid, vertex_id)).length() > tolerance)
        };
        let ends_moved =
            corner_moved(edge.start_vertex_id) || corner_moved(edge.end_vertex_id);
        // Nothing changed here, so nothing is rebuilt: an edge whose carriers
        // the motion leaves alone and whose corners did not move IS the edge it
        // was, exactly, and re-deriving it could only add fit noise to an
        // answer the contract says is the input.
        if moved_uses == 0 && !ends_moved {
            continue;
        }
        dirty.insert(edge.id);
        order.push(edge);
    }

    // 3a. THE CLOSED RIMS FIRST, because they carry their own corner.
    //
    // A rim whose two ends are the same vertex — a bore's mouth — is a whole
    // conic, and its carriers meet in a CURVE, not a point: no third face meets
    // at that vertex, so nothing determines where along the rim it sits. The
    // section itself does. `intersect_plane_quadric` anchors its answer at the
    // carrier frame's own `x_axis`, which is the carrier's seam, and the vertex
    // is where the face's seam meets the rim — so taking the vertex FROM the
    // rebuilt section is not a convention invented here, it is the only reading
    // on which the vertex and the seam edge that ends on it can agree.
    //
    // It is also what makes the two bore cells come out right for the same
    // reason rather than by opposite rules: a bore pushed ACROSS its axis
    // carries its seam with it (the frame moved, so the anchor moved), and a
    // bore pushed ALONG its axis, or a cap slid in its own plane, leaves the
    // seam exactly where it was (the frame's azimuth did not move).
    for edge in &order {
        if edge.start_vertex_id != edge.end_vertex_id || edge.degenerate {
            continue;
        }
        let uses = faces_of_edge.get(&edge.id).map(Vec::as_slice).unwrap_or(&[]);
        let (Some(&first), Some(&second)) = (uses.first(), uses.get(1)) else {
            continue;
        };
        if first == second {
            continue;
        }
        let section = carrier_section(
            edge,
            carrier_of(&carriers, first)?,
            carrier_of(&carriers, second)?,
            tolerance,
        )?;
        let Section::Curve(curve) = section else {
            return Err(KernelRefusal::internal(KernelStage::Refine, "closed_rim_straight", format!(
                "{OP}: the closed rim {} sections in a straight line, which cannot close — \
                 refusing",
                edge.id
            )));
        };
        let matched = match_marched_rim_direction(curve, edge)?;
        let [d0, d1] = matched.domain().or_refuse(KernelStage::Refine, "domain")?;
        let anchor = matched.evaluate(d0).or_refuse(KernelStage::Refine, "evaluate")?;
        new_vertex.insert(edge.start_vertex_id, anchor);
        rebuilt.insert(edge.id, (matched, d0, d1));
    }

    // 3b. Everything else.
    for edge in &order {
        if rebuilt.contains_key(&edge.id) {
            continue;
        }
        let uses = faces_of_edge.get(&edge.id).map(Vec::as_slice).unwrap_or(&[]);
        let moved_uses = uses.iter().filter(|face_id| moves_carrier.contains(*face_id)).count();
        let old_start = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
        let old_end = edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Refine, "evaluate")?;
        let start_new = new_vertex
            .get(&edge.start_vertex_id)
            .copied()
            .unwrap_or_else(|| point_of(solid, edge.start_vertex_id));
        let end_new = new_vertex
            .get(&edge.end_vertex_id)
            .copied()
            .unwrap_or_else(|| point_of(solid, edge.end_vertex_id));
        let first = *uses.first().ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Refine,
                "edge_faces",
                format!("{OP}: edge {} has no faces", edge.id),
            )
        })?;
        let second = uses.get(1).copied();

        // A rigid shortcut that carries a PROOF, not a case: when every carrier
        // this edge lies on moved by the same rigid motion AND both its corners
        // came out at their own rigid images, the section of the moved carriers
        // between them IS the rigid image of the old edge — so mapping it keeps
        // the parametrisation, the weights and the knot vector exact. Where
        // either half of that fails, the general path answers instead, and the
        // three moved faces of a corner lift are exactly where it fails: two
        // moved planes still meet a FIXED one, so their shared edge's corner
        // slides ALONG the edge and is not the rigid image of anything.
        let rigid_ends = start_new.sub(old_start.add(translation)).length() <= tolerance
            && end_new.sub(old_end.add(translation)).length() <= tolerance;
        if moved_uses == uses.len() && rigid_ends {
            rebuilt.insert(
                edge.id,
                (transform_curve(&edge.curve, motion).or_refuse(KernelStage::Refine, "transform_curve")?, edge.t0, edge.t1),
            );
            continue;
        }

        // An edge with only ONE carrier — a seam meridian a face uses twice, or
        // a degenerate apex point — has no carrier PAIR to re-intersect. It
        // lies on that one carrier and moves exactly as the carrier does, and
        // the corners it ends on re-trim it. This is the arm a bore's own seam
        // takes whenever the cap it ends against is the thing that moved.
        let carrier_pair = match second {
            Some(second) if second != first => Some(second),
            _ => None,
        };
        let (curve, t0, t1) = match carrier_pair {
            None => {
                let carried = if moves_carrier.contains(&first) {
                    transform_curve(&edge.curve, motion).or_refuse(KernelStage::Refine, "transform_curve")?
                } else {
                    edge.curve.clone()
                };
                // A SEAM that has to GROW. The carrier is unbounded here, but
                // the stored seam curve is not: it spans whatever extent the
                // face happened to be trimmed to, and a corner the motion
                // carries past that end projects onto the end instead of onto
                // the seam. That is not a limit of the geometry — a cylinder's
                // generatrix is a straight line and it runs as far as its
                // carrier does — so a STRAIGHT seam is rebuilt as the segment
                // between its own two re-solved corners, which both lie on the
                // carrier by construction.
                //
                // MEASURED, and it is why this is here rather than in a later
                // slice: pushing a holed cap along the bore's own axis builds
                // to `s = 10` on the old road, and refused here from
                // `s ≈ 1.250` — the magnitude at which the cap passes the end
                // of the bore's stored generatrix. The refusal quoted
                // `3.000e0`, which is exactly that overrun at `s = 2`. The
                // capability matrix's own cell for it sits at `s = 1`, where
                // both roads agree, so the cell table could not see it and the
                // swept column is what found it.
                let curve = if carried.straight_segment(tolerance).is_some() {
                    make_line(start_new, end_new).or_refuse(KernelStage::Refine, "make_line")?
                } else {
                    carried
                };
                let t0 = parameter_on(&curve, start_new, tolerance, edge.id, "start")?;
                let t1 = parameter_on(&curve, end_new, tolerance, edge.id, "end")?;
                if (t1 - t0).abs() <= 1e-12 {
                    return Err(KernelRefusal::unsupported(KernelStage::Refine, "seam_collapse", format!(
                        "{OP}: the corners of the single-carrier edge {} land on the same point \
                         of it — refusing",
                        edge.id
                    )));
                }
                (curve, t0, t1)
            }
            Some(second) => rebuild_edge(
                edge,
                carrier_of(&carriers, first)?,
                carrier_of(&carriers, second)?,
                start_new,
                end_new,
                tolerance,
            )?,
        };
        rebuilt.insert(edge.id, (curve, t0, t1));
    }

    // The two guards the translation road's category-3 refusals are made of,
    // stated on the rebuilt chord rather than on a plan: a moved face that
    // lands exactly on its neighbour collapses an edge, and one that passes
    // beyond it reverses one. Both are read from the geometry, so they hold
    // for any carrier pair and not only for the straight rebuilds the old road
    // could check.
    for edge in &solid.edges {
        let Some((curve, t0, t1)) = rebuilt.get(&edge.id) else {
            continue;
        };
        if edge.start_vertex_id == edge.end_vertex_id || edge.degenerate {
            continue;
        }
        let old_start = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
        let old_end = edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Refine, "evaluate")?;
        let new_start = curve.evaluate(*t0).or_refuse(KernelStage::Refine, "evaluate")?;
        let new_end = curve.evaluate(*t1).or_refuse(KernelStage::Refine, "evaluate")?;
        let old_chord = old_end.sub(old_start);
        let new_chord = new_end.sub(new_start);
        if new_chord.length() <= tolerance {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, "edge_collapse", format!(
                "{OP}: the translation collapses edge {} to zero length (a moved face lands \
                 exactly on its neighbour)",
                edge.id
            )));
        }
        if old_chord.length() > tolerance && new_chord.dot(old_chord) < 0.0 {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, EDGE_INVERSION, format!(
                "{OP}: the translation inverts edge {} (a moved face passes beyond its \
                 neighbour)",
                edge.id
            )));
        }
    }

    for edge in &mut result.edges {
        if let Some((curve, t0, t1)) = rebuilt.get(&edge.id) {
            edge.curve = curve.clone();
            edge.t0 = *t0;
            edge.t1 = *t1;
        }
    }
    for vertex in &mut result.vertices {
        if let Some(point) = new_vertex.get(&vertex.id) {
            vertex.point = *point;
        }
    }

    // --- 3b. The trims -----------------------------------------------------
    // Every face the rebuild touched regrows its carrier around its own new
    // boundary and refits every pcurve on it. A MOVED face's carrier is the
    // moved one; a FIXED face's is its own, GROWN to cover wherever the new
    // boundary went — which is the same "extend the neighbour" the delete/heal
    // road has always done, and is what makes "the pushed cap does not re-cross
    // fixed edge N within its span" not a thing that can be said here.
    let final_edges: HashMap<u64, EdgeRecord> =
        result.edges.iter().map(|edge| (edge.id, edge.clone())).collect();
    let mut touched_faces: Vec<(usize, usize)> = Vec::new();
    for (shell_index, shell) in result.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            let face_dirty = moves_carrier.contains(&face.id)
                || face.loops.iter().any(|loop_record| {
                    loop_record.coedges.iter().any(|c| dirty.contains(&c.edge_id))
                });
            if face_dirty {
                touched_faces.push((shell_index, face_index));
            }
        }
    }
    for (shell_index, face_index) in touched_faces {
        let face = &mut result.shells[shell_index].faces[face_index];
        if moves_carrier.contains(&face.id) {
            face.surface = transform_surface(&face.surface, motion).or_refuse(KernelStage::Sew, "transform_surface")?;
        }
        regrow_and_refit_carrier(face, &final_edges, scale, OP)?;
    }

    // --- 4. Acceptance -----------------------------------------------------
    // Unchanged from the old road, deliberately: this slice does not get to
    // move the bar it is measured against.
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "validate", format!("{OP}: moved solid failed validation: {issues:?}")));
    }
    if let (Ok(before), Ok(after)) = (solid_signed_volume(solid), solid_signed_volume(&result)) {
        if before * after <= 0.0 {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, SOLID_INVERSION, format!(
                "{OP}: the translation inverts the solid (signed volume changed sign) — refusing"
            )));
        }
    }
    Ok(result)
}

/// Does the motion map this face's carrier onto ITSELF?
///
/// Sixteen points of the face's own patch, carried by the motion and projected
/// back onto the carrier they came from (the STRETCHED one, so a sample that
/// leaves the face's own trim is still asked about the surface rather than
/// about the patch's edge). Nothing here reads the surface's type, so it
/// answers for a plane slid in itself, a cylinder or cone pushed along its
/// axis, a sphere or torus translated along nothing at all, and a fitted
/// free-form patch that happens to be periodic, by the same arithmetic.
fn carrier_invariant_under_motion(
    patch: &NurbsSurface,
    carrier: &NurbsSurface,
    motion: AffineTransform,
    tolerance: f64,
) -> Result<bool, String> {
    const STATIONS: usize = 4;
    let [u0, u1] = patch.domain_u()?;
    let [v0, v1] = patch.domain_v()?;
    for i in 0..STATIONS {
        for j in 0..STATIONS {
            let u = u0 + (u1 - u0) * i as f64 / (STATIONS - 1) as f64;
            let v = v0 + (v1 - v0) * j as f64 / (STATIONS - 1) as f64;
            let carried = motion.point(patch.evaluate(u, v)?);
            let foot = crate::project_point_to_surface(carrier, carried)?;
            if foot.point.sub(carried).length() > tolerance {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// A face the way the user picked it: its persistent name where it has one,
/// and its id either way so a log can still be followed.
fn face_label(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
) -> String {
    match face_lookup
        .get(&face_id)
        .and_then(|&(shell, face)| solid.shells[shell].faces[face].name.as_deref())
    {
        Some(name) => format!("`{name}` (face {face_id})"),
        None => format!("face {face_id}"),
    }
}

fn carrier_of<'a>(
    carriers: &'a HashMap<u64, NurbsSurface>,
    face_id: u64,
) -> Result<&'a NurbsSurface, KernelRefusal> {
    carriers.get(&face_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Refine,
            "carrier_missing",
            format!("{OP}: no carrier for face {face_id}"),
        )
    })
}

fn point_of(solid: &BrepSolid, vertex_id: u64) -> Vec3 {
    solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == vertex_id)
        .map(|vertex| vertex.point)
        .unwrap_or_default()
}

fn translate_affine(translation: Vec3) -> Result<AffineTransform, String> {
    AffineTransform::new([
        1.0, 0.0, 0.0, translation.x, //
        0.0, 1.0, 0.0, translation.y, //
        0.0, 0.0, 1.0, translation.z, //
        0.0, 0.0, 0.0, 1.0,
    ])
}

/// The centre and radius of a ball containing every vertex of the body — the
/// scale at which "cover the rest of the solid" is measured.
fn body_ball(solid: &BrepSolid) -> Result<(Vec3, f64), KernelRefusal> {
    if solid.vertices.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "empty_solid", format!("{OP}: the body has no vertices")));
    }
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in &solid.vertices {
        low = Vec3::new(
            low.x.min(vertex.point.x),
            low.y.min(vertex.point.y),
            low.z.min(vertex.point.z),
        );
        high = Vec3::new(
            high.x.max(vertex.point.x),
            high.y.max(vertex.point.y),
            high.z.max(vertex.point.z),
        );
    }
    let centre = low.add(high).scale(0.5);
    let radius = high.sub(low).length() * 0.5;
    Ok((centre, radius.max(1.0)))
}

/// A face's carrier with its trim thrown away — stretched to cover a ball of
/// radius `reach` about `centre`, which is the body and everywhere the motion
/// can take it.
///
/// This is the step the operator's instruction names. A plane rebuilds as a
/// patch that covers the ball; a cylinder, cone or fillet band prolongs along
/// its own axis with the SAME frame and the SAME generatrix direction, so the
/// stretch is exact and not a fit; a sphere and a torus are already complete
/// and are returned untouched. A free-form patch is returned as it stands — it
/// has no exact prolongation, and inventing one would be extrapolating a fit,
/// so a section that needs more of it refuses downstream instead.
fn unbounded_carrier(
    surface: &NurbsSurface,
    centre: Vec3,
    reach: f64,
    plane_tolerance: f64,
) -> Result<NurbsSurface, KernelRefusal> {
    match surface.analytic() {
        Some(AnalyticSurface::Sphere { .. }) | Some(AnalyticSurface::Torus { .. }) => {
            return Ok(surface.clone())
        }
        Some(AnalyticSurface::RuledRevolution {
            frame,
            rho0,
            rho1,
            height,
        }) => {
            let (frame, rho0, rho1, height) = (frame.clone(), *rho0, *rho1, *height);
            return stretch_ruled(&frame, rho0, rho1, height, centre, reach);
        }
        Some(AnalyticSurface::Revolution {
            frame,
            sweep,
            generatrix,
            ..
        }) => {
            // A partial sweep with a STRAIGHT generatrix is a cylinder or cone
            // band (this is how a fillet band recognizes). Its prolongation is
            // the same one, over the same sweep.
            let sweep = *sweep;
            let frame = frame.clone();
            if let Some((start, end)) = generatrix.straight_segment(plane_tolerance) {
                let (rho0, z0) = axial_radial(&frame, start);
                let (rho1, z1) = axial_radial(&frame, end);
                let height = z1 - z0;
                if height.abs() > 1e-12 {
                    let base = RevolutionFrame {
                        origin: frame.origin.add(frame.axis.scale(z0)),
                        ..frame.clone()
                    };
                    return stretch_ruled_sweep(&base, rho0, rho1, height, centre, reach, sweep);
                }
            }
            return Ok(surface.clone());
        }
        _ => {}
    }
    // A plane — tagged or merely geometric — rebuilds as a patch covering the
    // ball. Everything else (a fitted free-form patch) keeps its own domain.
    match plane_of_surface(surface, plane_tolerance, OP) {
        Ok(plane) => {
            let foot = centre.sub(plane.normal.scale(centre.sub(plane.origin).dot(plane.normal)));
            let origin = foot
                .sub(plane.u_dir.scale(reach))
                .sub(plane.v_dir.scale(reach));
            crate::make_plane(origin, plane.u_dir, plane.v_dir, 2.0 * reach, 2.0 * reach)
                .or_refuse(KernelStage::Classify, "make_plane")
        }
        Err(_) => Ok(surface.clone()),
    }
}

/// A point in the frame's own cylindrical reading: its radius from the axis
/// and its height along it. `RevolutionFrame::cylindrical` says the same thing
/// and is private to the geometry module.
fn axial_radial(frame: &RevolutionFrame, point: Vec3) -> (f64, f64) {
    let delta = point.sub(frame.origin);
    let z = delta.dot(frame.axis);
    (delta.sub(frame.axis.scale(z)).length(), z)
}

fn stretch_ruled(
    frame: &RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    centre: Vec3,
    reach: f64,
) -> Result<NurbsSurface, KernelRefusal> {
    stretch_ruled_sweep(
        frame,
        rho0,
        rho1,
        height,
        centre,
        reach,
        std::f64::consts::TAU,
    )
}

/// Prolong a straight generatrix along the axis so the carrier spans the ball,
/// and revolve it again through the same sweep. Exact: the frame, the axis and
/// the generatrix's slope are the carrier's own.
fn stretch_ruled_sweep(
    frame: &RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    centre: Vec3,
    reach: f64,
    sweep: f64,
) -> Result<NurbsSurface, KernelRefusal> {
    if height.abs() <= 1e-12 {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "flat_ruled_carrier", format!("{OP}: a ruled carrier with no height cannot be stretched")));
    }
    let axial = centre.sub(frame.origin).dot(frame.axis);
    let low = (axial - reach).min(0.0);
    let high = (axial + reach).max(height);
    let rho_at = |z: f64| rho0 + (rho1 - rho0) * z / height;
    // A cone's generatrix crosses its own apex: prolonging past it would fold
    // the carrier through itself and give a second nappe with a negative
    // radius. Stop at the apex, which is where the carrier genuinely ends.
    let (low, high) = if (rho1 - rho0).abs() > 1e-12 {
        let apex_z = -rho0 * height / (rho1 - rho0);
        if apex_z > 0.0 {
            (low.max(apex_z), high)
        } else {
            (low, high.min(apex_z).max(low))
        }
    } else {
        (low, high)
    };
    let (low, high) = if high - low > 1e-12 * height.abs() {
        (low, high)
    } else {
        (0.0, height)
    };
    let base = frame.origin.add(frame.axis.scale(low));
    let start = base.add(frame.x_axis.scale(rho_at(low)));
    let end = frame
        .origin
        .add(frame.axis.scale(high))
        .add(frame.x_axis.scale(rho_at(high)));
    let generatrix = make_line(start, end).or_refuse(KernelStage::Classify, "make_line")?;
    make_revolution(base, frame.axis, &generatrix, sweep)
        .or_refuse(KernelStage::Classify, "make_revolution")
}

/// Where a corner goes: as near the rigid guess `seed` as a point can be while
/// lying on every carrier in `at`.
///
/// Each carrier is linearized at the current iterate's foot — the tangent plane
/// `n·(x − q) = 0` — and the correction from the seed is the damped
/// least-squares solution of the stacked system. Three transverse carriers give
/// the unique triple point; a rank-deficient set (a rim's two carriers, which
/// meet in a CURVE) leaves a free direction along which the solution stays
/// where the seed put it, which is exactly the coherent answer: the corner
/// rides the rim the way the motion carried it rather than sliding to the
/// nearest point of the new rim, which for a translated bore would flip it to
/// the far side.
fn solve_corner(
    at: &[&NurbsSurface],
    seed: Vec3,
    tolerance: f64,
) -> Result<Vec3, KernelRefusal> {
    // Enough to make the 3×3 normal matrix invertible in the directions no
    // carrier constrains, and small enough that the bias it leaves at the fixed
    // point (≈ λ·|correction|) is orders below the residual gate.
    const DAMPING: f64 = 1e-12;
    let mut x = seed;
    let mut worst = f64::INFINITY;
    for _ in 0..64 {
        let mut normal_matrix = [[0.0f64; 3]; 3];
        let mut rhs = [0.0f64; 3];
        worst = 0.0;
        for surface in at {
            let projection = crate::project_point_to_surface(surface, x).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            let foot = projection.point;
            let normal = surface.normal(projection.u, projection.v).or_refuse(KernelStage::Refine, "normal")?;
            worst = worst.max(foot.sub(x).length());
            // n·(seed + d) = n·foot  ⇒  n·d = n·(foot − seed)
            let row = [normal.x, normal.y, normal.z];
            let value = normal.dot(foot.sub(seed));
            for i in 0..3 {
                for j in 0..3 {
                    normal_matrix[i][j] += row[i] * row[j];
                }
                rhs[i] += row[i] * value;
            }
        }
        if worst <= tolerance {
            return Ok(x);
        }
        // Damping is what lets a RANK-DEFICIENT corner — a rim's two carriers,
        // which meet in a curve — keep the seed's position along the direction
        // nothing constrains. It is also a bias of order `λ·|correction|` on
        // the answer, so it is spent only where it buys something: three
        // transverse carriers determine the corner outright and are solved
        // exactly. (Measured: with the damping always on, a three-face corner
        // lift came out 1.4e-13 against its closed form instead of 1e-16.)
        let scale3 = (normal_matrix[0][0] + normal_matrix[1][1] + normal_matrix[2][2]) / 3.0;
        let full_rank = determinant3(&normal_matrix).abs() > 1e-9 * scale3.powi(3).max(1e-300);
        if !full_rank {
            for i in 0..3 {
                normal_matrix[i][i] += DAMPING * scale3.max(1.0);
            }
        }
        let delta = solve3(&normal_matrix, &rhs)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Refine,
                    "corner_singular",
                    "the carriers give no correction",
                )
            })?;
        let next = seed.add(Vec3::new(delta[0], delta[1], delta[2]));
        if next.sub(x).length() <= tolerance * 1e-3 {
            x = next;
            break;
        }
        x = next;
    }
    // One last reading, so the refusal quotes the distance rather than the
    // iteration count.
    let mut residual = 0.0f64;
    for surface in at {
        let projection = crate::project_point_to_surface(surface, x).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        residual = residual.max(projection.point.sub(x).length());
    }
    if residual <= tolerance * RESIDUAL_SLACK {
        return Ok(x);
    }
    // Say WHICH of the two ways it failed, because they want different things
    // from the reader. If the carriers' normals all agree at the corner they
    // share a tangent plane, and three tangent surfaces constrain a point in
    // ONE direction and pin nothing in the other two — no motion of any size
    // has an answer there, and the reader needs to know it is the shape and
    // not the distance. If they are transverse and still do not meet, it is
    // the motion that carried them apart, and a smaller one will work.
    let mut spread: f64 = 0.0;
    let mut normals: Vec<Vec3> = Vec::with_capacity(at.len());
    for surface in at {
        let projection = crate::project_point_to_surface(surface, x).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        normals.push(surface.normal(projection.u, projection.v).or_refuse(KernelStage::Refine, "normal")?);
    }
    for (index, first) in normals.iter().enumerate() {
        for second in &normals[index + 1..] {
            let sine = first.cross(*second).length();
            let cosine = first.dot(*second).abs();
            spread = spread.max(sine.atan2(cosine));
        }
    }
    if spread <= 1e-6 {
        return Err(KernelRefusal::ill_posed(KernelStage::Refine, "tangent_corner", format!(
            "they share a common TANGENT PLANE there (their normals agree to {spread:.1e} rad), \
             so together they fix the corner in one direction and pin it in neither of the \
             other two; the nearest point to all of them is still {residual:.3e} away"
        )));
    }
    Err(KernelRefusal::unsupported(KernelStage::Refine, "corner_apart", format!(
        "the nearest point to all of them is {residual:.3e} off at least one, against a \
         tolerance of {:.3e} (their normals there are {spread:.3e} rad apart, so they are \
         transverse and it is the MOTION that carried them apart); worst reading while \
         iterating {worst:.3e}",
        tolerance * RESIDUAL_SLACK
    )))
}

/// Solve a symmetric 3×3 system by Cramer's rule. `None` when it is singular
/// even after damping, which means the carriers constrain nothing at all.
fn solve3(matrix: &[[f64; 3]; 3], rhs: &[f64; 3]) -> Option<[f64; 3]> {
    let det = determinant3(matrix);
    if det.abs() <= f64::MIN_POSITIVE {
        return None;
    }
    let mut out = [0.0f64; 3];
    for column in 0..3 {
        let mut replaced = *matrix;
        for row in 0..3 {
            replaced[row][column] = rhs[row];
        }
        out[column] = determinant3(&replaced) / det;
    }
    if out.iter().all(|value| value.is_finite()) {
        Some(out)
    } else {
        None
    }
}

fn determinant3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Rebuild one edge as the section of its own two carriers, delimited by its
/// own two re-solved corners.
fn rebuild_edge(
    edge: &EdgeRecord,
    first: &NurbsSurface,
    second: &NurbsSurface,
    start_new: Vec3,
    end_new: Vec3,
    tolerance: f64,
) -> Result<(NurbsCurve, f64, f64), KernelRefusal> {
    let section = carrier_section(edge, first, second, tolerance)?;
    match section {
        Section::Straight => {
            let curve = make_line(start_new, end_new).or_refuse(KernelStage::Refine, "make_line")?;
            Ok((curve, 0.0, 1.0))
        }
        Section::Curve(curve) => {
            let start_t = parameter_on(&curve, start_new, tolerance, edge.id, "start")?;
            let end_t = parameter_on(&curve, end_new, tolerance, edge.id, "end")?;
            if (end_t - start_t).abs() <= 1e-12 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "section_collapse", format!(
                    "{OP}: the corners of edge {} land on the same point of their section \
                     — refusing",
                    edge.id
                )));
            }
            if start_t < end_t {
                Ok((curve, start_t, end_t))
            } else {
                let reversed = curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
                let [d0, d1] = reversed.domain().or_refuse(KernelStage::Refine, "domain")?;
                // `reversed` maps t ↦ d0 + d1 − t on the same domain.
                Ok((reversed, d0 + d1 - start_t, d0 + d1 - end_t))
            }
        }
    }
}

/// The section of two carriers, chosen to run near the boundary it replaces.
fn carrier_section(
    edge: &EdgeRecord,
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<Section, KernelRefusal> {
    let plane_tolerance = tolerance * 10.0;
    if plane_of_surface(first, plane_tolerance, OP).is_ok()
        && plane_of_surface(second, plane_tolerance, OP).is_ok()
    {
        return Ok(Section::Straight);
    }
    let seeds = edge_seeds(edge, 9)?;
    let policy = MarchPolicy {
        tolerance,
        residual_tolerance: tolerance * 100.0,
        seeds: seeds.clone(),
    };
    let found = match reintersect_carriers(first, second, &policy) {
        Ok(found) => found,
        Err(ReintersectRefusal::Separated) => {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "carriers_separated", format!(
                "{OP}: the carriers of edge {} no longer meet after the motion — refusing",
                edge.id
            )))
        }
        // The marched lane's fit missed the caller's residual gate: a march
        // that did not conform, so a budget class rather than a defect.
        Err(other @ ReintersectRefusal::Residual { .. }) => {
            return Err(KernelRefusal::non_convergence(
                KernelStage::Refine,
                "section_residual",
                format!("{OP}: edge {}: {}", edge.id, other.describe()),
            ));
        }
        Err(other) => {
            return Err(KernelRefusal::internal(
                KernelStage::Refine,
                "reintersect",
                format!("{OP}: edge {}: {}", edge.id, other.describe()),
            ));
        }
    };
    let section = found
        .nearest_section(&seeds)
        .map_err(|error| {
            KernelRefusal::internal(
                KernelStage::Refine,
                "nearest_section",
                format!("{OP}: edge {}: {error}", edge.id),
            )
        })?;
    Ok(Section::Curve(section.curve.clone()))
}

/// The parameter at which `curve` passes through `point`, refusing rather than
/// snapping when it does not pass through it at all.
fn parameter_on(
    curve: &NurbsCurve,
    point: Vec3,
    tolerance: f64,
    edge_id: u64,
    which: &str,
) -> Result<f64, KernelRefusal> {
    let projection = project_point_to_curve(curve, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
    let off = projection.point.sub(point).length();
    if off > tolerance * RESIDUAL_SLACK {
        return Err(KernelRefusal::internal(KernelStage::Refine, "corner_off_section", format!(
            "{OP}: the re-solved {which} corner of edge {edge_id} is {off:.3e} off the section \
             its carriers make — refusing"
        )));
    }
    Ok(projection.u)
}


// ===========================================================================
// THE ROUTING READING — which cells need the REBUILD road, measured rather
// than enumerated.
// ===========================================================================

/// What a Move Face would have to do to the TOPOLOGY, read off the motion
/// before anything is built.
#[derive(Debug, Clone, Default)]
pub struct RouteReading {
    /// Faces that do not bound the moved group today and whose trimmed region
    /// a moved carrier would pass through — the faces that would have to
    /// receive a loop they do not have.
    pub new_loop_faces: Vec<(u64, Option<String>)>,
    /// Faces whose section against a moved carrier could not be COMPUTED.
    ///
    /// These are not "no", they are "unknown", and they are reported rather
    /// than folded into the negative: an intersection that fails silently
    /// would make this predicate answer NO for a face it never managed to ask
    /// about, which is the difference between a measurement and an instrument
    /// that cannot report a negative. A caller that routes on this must treat
    /// a non-empty list as a refusal rather than as a clean read.
    pub unreadable: Vec<(u64, Option<String>, String)>,
}

impl RouteReading {
    /// The routing decision itself: a motion that would put a boundary on a
    /// face that has none cannot be answered by re-solving the existing
    /// topology's geometry, because there is no loop there to re-solve.
    pub fn needs_rebuild(&self) -> bool {
        !self.new_loop_faces.is_empty()
    }

    /// Whether every face was actually ASKED. A reading with unreadable faces
    /// has not established the negative for those faces.
    pub fn is_complete(&self) -> bool {
        self.unreadable.is_empty()
    }
}

/// Which faces a moved selection's carriers would open a NEW hole in.
///
/// # What this measures, and why it is not a case table
///
/// The carrier road re-solves the EXISTING topology's geometry against the
/// moved carriers. That is right for as long as the moved group keeps the
/// neighbours it has: every boundary it owns is a loop that is already there,
/// and re-solving moves it. It is wrong the moment the motion carries a
/// boundary onto a face the group does not currently touch, because there is
/// no loop on that face to re-solve — which is the migrating mouth, and it is
/// why that cell refuses on both roads today.
///
/// So the question "does this motion need the rebuild road" is exactly "does
/// some moved carrier, after the motion, pass through the TRIMMED REGION of a
/// face it does not currently bound". It reads no surface type and no
/// configuration; a plane, a bore wall and an imported free-form patch are all
/// asked the same question.
///
/// # The two halves, and which one already existed
///
/// The SECTION half is the kernel's own `intersect_surfaces`. Note that
/// [`carrier_section`] above is NOT usable here: it seeds its march from the
/// edge it is replacing (`edge_seeds`), and the whole point of this question is
/// faces that share no edge with the moved group, so there is no seed to take.
///
/// The TRIMMED-REGION half is `parameter_point_in_face`, and nothing on the
/// carrier road called it before this: the road only ever asked about carriers
/// it already had an edge on, where containment is not in question. A section
/// that crosses a face's CARRIER but misses its trim is the ordinary case — a
/// bore sliding along inside its own slab passes through the plane of every
/// wall in the body and opens a hole in none of them — so the trim test is
/// what separates a migration from a motion that merely moves.
///
/// Faces the group ALREADY bounds are excluded: a section there is the first
/// half's job and is not a topology change.
pub fn route_reading(
    solid: &BrepSolid,
    moved: &[u64],
    motion: &AffineTransform,
) -> Result<RouteReading, KernelRefusal> {
    let tolerances = crate::KernelTolerances::for_solid(solid, 1e-7);
    let tolerance = tolerances.model.max(1e-9);
    let scale = crate::solid_scale(solid).max(1.0);
    let moved_set: HashSet<u64> = moved.iter().copied().collect();

    // Faces the moved group already bounds: they share an edge with it, so a
    // section there is the re-solve's business rather than a new loop.
    let mut adjacent: HashSet<u64> = HashSet::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if moved_set.contains(&face.id) {
            continue;
        }
        let touches = face.loops.iter().flat_map(|wire| &wire.coedges).any(|use_| {
            solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .filter(|other| moved_set.contains(&other.id))
                .any(|other| {
                    other
                        .loops
                        .iter()
                        .flat_map(|wire| &wire.coedges)
                        .any(|theirs| theirs.edge_id == use_.edge_id)
                })
        });
        if touches {
            adjacent.insert(face.id);
        }
    }

    let mut centre = Vec3::new(0.0, 0.0, 0.0);
    for vertex in &solid.vertices {
        centre = centre.add(vertex.point);
    }
    centre = centre.scale(1.0 / (solid.vertices.len().max(1)) as f64);
    // HYGIENE, NOT THE FIX. The carrier only has to reach the body and
    // wherever the motion takes it; `scale * 4` stretched it four times beyond
    // the body for no reason. Bounding does not make a section findable — the
    // seeds do that — so this is tidiness.
    let travel = motion.point(centre).sub(centre).length();
    let reach = scale + travel + tolerance * 10.0;

    // The carriers BEFORE and AFTER the motion, both prolonged the same way so
    // the two readings differ only by the motion.
    let mut before_carriers: Vec<NurbsSurface> = Vec::new();
    let mut after_carriers: Vec<NurbsSurface> = Vec::new();
    for face_id in moved {
        let Some((shell, position)) = find_face(solid, *face_id) else {
            return Err(KernelRefusal::input(KernelStage::Collect, "face_id", format!("route_reading: no face with id {face_id}")));
        };
        let surface = &solid.shells[shell].faces[position].surface;
        let unbounded = unbounded_carrier(surface, centre, reach, tolerance * 10.0)?;
        after_carriers.push(transform_surface(&unbounded, *motion).or_refuse(KernelStage::Classify, "transform_surface")?);
        before_carriers.push(unbounded);
    }

    let mut unreadable: Vec<(u64, Option<String>, String)> = Vec::new();
    let candidates: Vec<&FaceRecord> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| !moved_set.contains(&face.id) && !adjacent.contains(&face.id))
        .collect();

    let before = crossed_faces(
        &before_carriers,
        &candidates,
        scale,
        tolerance,
        &mut unreadable,
    );
    let after = crossed_faces(
        &after_carriers,
        &candidates,
        scale,
        tolerance,
        &mut unreadable,
    );

    // THE DISCRIMINATOR: the crossing has to be NEW.
    //
    // "The moved carrier crosses this face's trim" is satisfied by any
    // prolonged planar carrier against any wall of the body — an unbounded
    // plane crosses everything, which is what unbounded means. Measured: it
    // fired on `pocket-floor-down` and `pocket-wall-out`, which merely deepen
    // and widen a pocket and put no new hole anywhere.
    //
    // What separates a migration from that is whether the crossing EXISTED
    // BEFORE. A pocket floor's horizontal carrier already crossed the body's
    // vertical walls; moving it down changes nothing about that, and what kept
    // the face off those walls was its own trim, which the re-solve maintains.
    // A bore at `x = 12` does NOT cross a step floor spanning `x ∈ [30, 60]`,
    // and after a 30 mm slide it does. That is the migration, and it is the
    // only thing the existing topology has no loop to re-solve.
    //
    // Chosen over "the section must lie inside the moved face's own trim,
    // transported by the motion", which also fixes the pocket cells and reads
    // well, because that rule fails on a migration into a THICKER region: move
    // the same bore from the thin side of the step to the thick side and its
    // new mouth lands beyond the transported trim, so it would go quiet. No
    // cell in the matrix exercises that, so it would have looked clean and
    // been wrong — the same shape as the `0 LOST` reading that was true of the
    // cells and false of the road.
    //
    // A differential test is also harder to fool than an absolute one: the
    // BEFORE and AFTER readings share every source of error except the motion,
    // which is why the before pass is seeded identically. An unseeded before
    // pass would report a spurious empty from the search blindness above and
    // then fire on everything.
    //
    // # THE KNOWN LIMIT OF THIS DISCRIMINATOR
    //
    // It goes quiet whenever the UNMOVED carrier ALREADY sectioned the face's
    // trim, because then the crossing is not new — even if no material was
    // there and a mouth appears anyway. The case to look at is a bore moved
    // OBLIQUELY whose unbounded cylinder already clips the target floor's trim
    // at the start while the bore itself does not yet reach it: before
    // non-empty, after non-empty, quiet, and a mouth opens. `step-bore-across`
    // escapes it only because `x = 12` is genuinely outside `x ∈ [30, 60]`; a
    // cell starting nearer the step would show it.
    //
    // Stated rather than fixed: nothing in the 54 cells exercises it, so there
    // is no measurement to fix it against, and a rule tuned against a case
    // nobody has measured is how a predicate stops meaning anything.
    let mut new_loop_faces: Vec<(u64, Option<String>)> = candidates
        .iter()
        .filter(|face| after.contains(&face.id) && !before.contains(&face.id))
        .map(|face| (face.id, face.name.clone()))
        .collect();

    new_loop_faces.sort();
    unreadable.sort();
    unreadable.dedup();
    Ok(RouteReading {
        new_loop_faces,
        unreadable,
    })
}

/// Which of `candidates` have a carrier's section running through their own
/// TRIMMED region.
///
/// Seeded from the FACE rather than the carrier, because the carrier is
/// unbounded by construction and `intersect_surfaces` returns the same
/// `Ok(empty)` for "the bounds prove there is no section" as for "the grid
/// search did not find one". Measured on `stepped2`: a prolonged carrier
/// against two parallel planes of the same kind, 18 apart and at near-identical
/// distances along it, is found on one at every seed density tried (1, 8, 40)
/// and on the other at none — while an explicit seed finds both. Every other
/// in-kernel caller of `intersect_surfaces` seeds for the same reason.
///
/// The seeds are the nearest face samples PROJECTED onto the carrier. A grid
/// sample will essentially never land on the section — the section is a curve
/// and the samples are a lattice — so a seed has to be near enough to REFINE
/// rather than already correct.
fn crossed_faces(
    carriers: &[NurbsSurface],
    candidates: &[&FaceRecord],
    scale: f64,
    tolerance: f64,
    unreadable: &mut Vec<(u64, Option<String>, String)>,
) -> HashSet<u64> {
    let mut crossed: HashSet<u64> = HashSet::default();
    for face in candidates {
        for carrier in carriers {
            let mut nearest: Vec<(f64, Vec3)> = Vec::new();
            if let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v())
            {
                for ui in 0..=10 {
                    for vi in 0..=10 {
                        let u = u0 + (u1 - u0) * ui as f64 / 10.0;
                        let v = v0 + (v1 - v0) * vi as f64 / 10.0;
                        let Ok(point) = face.surface.evaluate(u, v) else {
                            continue;
                        };
                        let Ok(projection) = crate::project_point_to_surface(carrier, point) else {
                            continue;
                        };
                        if projection.distance <= scale * 0.1 {
                            if let Ok(on_carrier) = carrier.evaluate(projection.u, projection.v) {
                                nearest.push((projection.distance, on_carrier));
                            }
                        }
                    }
                }
            }
            nearest.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            let options = crate::SurfaceIntersectionOptions {
                tolerance,
                seed_points: nearest.into_iter().take(12).map(|(_, p)| p).collect(),
                ..Default::default()
            };
            let curves = match crate::intersect_surfaces(carrier, &face.surface, &options) {
                Ok(curves) => curves,
                Err(error) => {
                    // NOT a negative: record it so a caller can tell "not
                    // crossed" from "never successfully asked".
                    unreadable.push((face.id, face.name.clone(), error));
                    continue;
                }
            };
            let mut opens = false;
            for curve in &curves {
                // `params_b` is the section's parameters on the FACE's own
                // surface, so the trim question is asked directly rather than
                // by projecting a 3D point back.
                for uv in &curve.params_b {
                    let point = crate::Vec2 { x: uv[0], y: uv[1] };
                    if matches!(
                        crate::parameter_point_in_face(face, point, tolerance),
                        Ok(crate::PolygonClass::Inside)
                    ) {
                        opens = true;
                        break;
                    }
                }
                if opens {
                    break;
                }
            }
            if opens {
                crossed.insert(face.id);
                break;
            }
        }
    }
    crossed
}
