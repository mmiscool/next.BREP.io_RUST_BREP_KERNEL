use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// Which recovered branch an open heal keeps, decided by GEOMETRY.
///
/// `corners[i]` are candidate `i`'s crossings of the two caps, and `contacts`
/// the strip's own contact with each cap (the midpoint of the boundary edge it
/// shares with that cap). A candidate is rated by the worse of its two corners'
/// distances from those contacts, and the least rating wins: the recovered edge
/// is the one the strip was cut from, so it runs past the strip on both caps.
///
/// Refuses (`Err((chosen, rival))`) when a DISTINCT candidate — one whose corners
/// do not both coincide with the winner's within `tolerance` — rates within
/// `tolerance` of the least rating. Nothing about the strip tells two such
/// branches apart, and keeping whichever the intersector happened to list first
/// would make the heal a property of that order. Candidates whose corners do
/// coincide are one branch found twice, and the first is kept. The decision
/// reads every candidate before it decides, so it does not depend on the order
/// they arrive in either.
pub(super) fn nearest_branch(
    corners: &[[Vec3; 2]],
    contacts: [Vec3; 2],
    tolerance: f64,
) -> Result<Option<usize>, (usize, usize)> {
    let rating = |pair: &[Vec3; 2]| {
        pair[0]
            .sub(contacts[0])
            .length()
            .max(pair[1].sub(contacts[1]).length())
    };
    let Some(chosen) = (0..corners.len()).min_by(|a, b| rating(&corners[*a]).total_cmp(&rating(&corners[*b])))
    else {
        return Ok(None);
    };
    let least = rating(&corners[chosen]);
    for rival in 0..corners.len() {
        let same_branch = corners[rival][0].sub(corners[chosen][0]).length() <= tolerance
            && corners[rival][1].sub(corners[chosen][1]).length() <= tolerance;
        if !same_branch && rating(&corners[rival]) - least <= tolerance {
            return Err((chosen, rival));
        }
    }
    Ok(Some(chosen))
}

/// Heal after deleting an OPEN blend strip (a fillet/chamfer along an open
/// edge chain) when at least one flanking neighbour is a CURVED analytic
/// carrier — the curved generalization of the all-planar path above, for
/// curved PRIMARIES (plane × cylinder / ruled revolution) and for curved
/// LATERAL CAPS (a strip between two planes ending against a cylindrical
/// wall) alike.
///
/// Extend-first (mirroring `heal_closed_transition`): the primary pairing is
/// CHOSEN, not assumed — each opposite neighbour pair is tried on a clone
/// (curved primaries grown over the strip's sampled extent, the pair
/// re-intersected in closed form via `intersect_analytic_pair`, plane×plane
/// synthesized from `intersect_planes`), and exactly one pairing must yield a
/// branch crossing BOTH remaining caps' ANALYTIC carriers within the strip's
/// region gate (planar caps use exact curve×plane bisection, curved caps the
/// Newton curve×surface intersector). The strip's corner topology then
/// collapses onto the recovered corners on those cap crossings: chain-end
/// side edges are widened along their own underlying curves when those reach
/// the corner, or re-derived in closed form from their flanking carriers when
/// the stored curve stops short (their far vertices are preserved and the
/// strip-corner vertices are rebound to the new edge's vertices), the
/// primaries swap their strip rim for the new edge, the caps drop their
/// transverse blend edge, and every touched pcurve is refit against the
/// surviving carriers. Refuses (never emits a bad solid) when a neighbour is
/// free-form, when no pairing (or both pairings) re-intersects across the
/// strip, or when a side edge cannot be carried onto its recovered corner.
pub(super) fn heal_open_transition_mixed(
    solid: &BrepSolid,
    shell_index: usize,
    face_index: usize,
    boundary: &[(u64, bool)],
    neighbour_ids: &[u64; 4],
) -> Result<BrepSolid, KernelRefusal> {
    if let Some(directory) = census_directory() {
        record_open_heal_census(&directory, solid, shell_index, face_index, boundary, neighbour_ids);
    }
    let op = "delete_face_and_heal";
    let solid = solid.clone();
    let scale = solid_model_scale(&solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    let face_id = solid.shells[shell_index].faces[face_index].id;
    let debug = std::env::var("BREP_DEBUG_HEAL").is_ok();
    let missing_neighbour = |id: u64| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "neighbour",
            format!("{op}: missing neighbour {id}"),
        )
    };
    let missing_edge = |id: u64| {
        KernelRefusal::internal(KernelStage::Collect, "edge", format!("{op}: missing edge {id}"))
    };

    // --- Classify the four neighbour carriers ------------------------------
    let mut carriers: Vec<OpenNeighbourCarrier> = Vec::with_capacity(4);
    for &neighbour_id in neighbour_ids {
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing_neighbour(neighbour_id))?;
        let surface = &solid.shells[ns].faces[nf].surface;
        if let Ok(plane) = plane_of_surface(surface, plane_tolerance, op) {
            carriers.push(OpenNeighbourCarrier::Planar(plane));
        } else if surface.analytic().is_some() {
            carriers.push(OpenNeighbourCarrier::Curved);
        } else {
            // A FITTED patch. It grows by `extend_natural` (a real surface with
            // a real domain) and re-intersects through the marched lane, so the
            // heal no longer stops at the first imported B-spline neighbour.
            carriers.push(OpenNeighbourCarrier::FreeForm);
        }
    }
    if debug {
        let kinds: Vec<&str> = carriers
            .iter()
            .map(|carrier| match carrier {
                OpenNeighbourCarrier::Planar(_) => "plane",
                OpenNeighbourCarrier::Curved => "curved",
                OpenNeighbourCarrier::FreeForm => "free-form",
            })
            .collect();
        eprintln!("HEAL open mixed: strip {face_id} neighbours {neighbour_ids:?} kinds {kinds:?}");
    }
    // --- Strip corners, centre, and reach (as in the planar path) ----------
    let boundary_edge_ids: HashSet<u64> = boundary.iter().map(|(edge_id, _)| *edge_id).collect();
    let transition_vertices: Vec<u64> = boundary
        .iter()
        .map(|(edge_id, forward)| {
            let edge = solid
                .edges
                .iter()
                .find(|edge| edge.id == *edge_id)
                .ok_or_else(|| missing_edge(*edge_id))?;
            Ok(if *forward {
                edge.start_vertex_id
            } else {
                edge.end_vertex_id
            })
        })
        .collect::<Result<Vec<u64>, KernelRefusal>>()?;
    if transition_vertices.iter().collect::<HashSet<_>>().len() != 4 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "degenerate_transition",
            format!(
                "{op}: transition face has repeated corner vertices (deferred: degenerate transition)"
            ),
        ));
    }
    let mut f_center = Vec3::default();
    for &vertex_id in &transition_vertices {
        f_center = f_center.add(edge_point(&solid, vertex_id)?);
    }
    f_center = f_center.scale(0.25);
    let mut f_reach = 0.0f64;
    for &vertex_id in &transition_vertices {
        f_reach = f_reach.max(edge_point(&solid, vertex_id)?.sub(f_center).length());
    }
    let f_reach = f_reach * 3.0 + tolerance;

    // --- Extend-first, then re-intersect: choose the primary pairing -------
    // Any neighbour may be curved. The primaries are the OPPOSITE pair whose
    // (extended) carriers re-intersect in a branch crossing BOTH remaining
    // caps' carriers within the strip's region gate. Each candidate pairing
    // is tried on a clone (carrier extension mutates the solid) and exactly
    // one pairing must qualify — mirroring the planar path's `plan_heal`.
    //
    // The strip is sampled at nine stations of equal ARC LENGTH per boundary
    // edge (`arc_length_stations`), in the loop's own traversal order. These
    // points decide how far a curved or fitted primary is grown and seed the
    // march, and a sample taken at uniform PARAMETER would make both depend on
    // how the rails happen to be parameterised rather than on where they are.
    let mut strip_points: Vec<Vec3> = Vec::new();
    // The strip's contact with each neighbour, as the midpoint of the boundary
    // edge they share: what a recovered corner is measured against below.
    let mut contacts: Vec<Vec3> = Vec::with_capacity(4);
    for (edge_id, forward) in boundary {
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.id == *edge_id)
            .ok_or_else(|| missing_edge(*edge_id))?;
        strip_points.extend(arc_length_stations(edge, 9, *forward)?);
        contacts.push(
            edge_point(&solid, edge.start_vertex_id)?
                .add(edge_point(&solid, edge.end_vertex_id)?)
                .scale(0.5),
        );
    }
    let neighbour_surface = |id: u64, solid: &BrepSolid| -> Result<NurbsSurface, KernelRefusal> {
        let (shell, face) = find_face(solid, id).ok_or_else(|| {
            KernelRefusal::internal(KernelStage::Collect, "face", format!("{op}: missing face {id}"))
        })?;
        Ok(solid.shells[shell].faces[face].surface.clone())
    };
    struct MixedHealPlan {
        solid: BrepSolid,
        primary: [usize; 2],
        lateral: [usize; 2],
        branch: NurbsCurve,
        crossings: [(f64, Vec3); 2],
    }
    let mut plans: Vec<MixedHealPlan> = Vec::new();
    // Why a pairing was dropped, kept so a refusal can name the reason rather
    // than only the symptom ("no branch spans the strip").
    let mut refusals: Vec<String> = Vec::new();
    for start in 0..2usize {
        let primary = [start, start + 2];
        let lateral = [(start + 1) % 4, (start + 3) % 4];
        let mut candidate = solid.clone();
        let mut extension_refusal: Option<()> = None;
        for &index in &primary {
            let grown = match carriers[index] {
                OpenNeighbourCarrier::Planar(_) => Ok(()),
                OpenNeighbourCarrier::Curved => extend_ruled_neighbour_over(
                    &mut candidate,
                    neighbour_ids[index],
                    &strip_points,
                    tolerance,
                ),
                // A fitted carrier is continued by its own terminal Bézier
                // span, far enough to cover the strip and no further, with the
                // coverage VERIFIED against the grown domain.
                OpenNeighbourCarrier::FreeForm => extend_freeform_neighbour_over(
                    &mut candidate,
                    neighbour_ids[index],
                    &strip_points,
                    tolerance,
                ),
            };
            if let Err(message) = grown {
                // Only the FREE-FORM lane records a reason. The analytic lanes
                // dropped a pairing silently before this slice and must keep
                // dropping it silently, so their refusal text stays byte-identical.
                if matches!(carriers[index], OpenNeighbourCarrier::FreeForm) {
                    if debug {
                        eprintln!("HEAL open mixed: primaries {primary:?} not extendable: {message}");
                    }
                    refusals.push(message.message);
                }
                extension_refusal = Some(());
                break;
            }
        }
        if extension_refusal.is_some() {
            continue;
        }
        let Ok(surface_a) = neighbour_surface(neighbour_ids[primary[0]], &candidate) else {
            continue;
        };
        let Ok(surface_b) = neighbour_surface(neighbour_ids[primary[1]], &candidate) else {
            continue;
        };
        let branches: Vec<NurbsCurve> = match (&carriers[primary[0]], &carriers[primary[1]]) {
            (OpenNeighbourCarrier::Planar(plane_a), OpenNeighbourCarrier::Planar(plane_b)) => {
                // `intersect_analytic_pair` declines plane×plane by design;
                // synthesize the closed-form line here, long enough to cross
                // both caps anywhere within the region gate.
                let Some(line) = intersect_planes(plane_a, plane_b) else {
                    continue;
                };
                let anchor = line
                    .point
                    .add(line.dir.scale(f_center.sub(line.point).dot(line.dir)));
                let half = f_reach * 2.0;
                let Ok(segment) = make_line(
                    anchor.sub(line.dir.scale(half)),
                    anchor.add(line.dir.scale(half)),
                ) else {
                    continue;
                };
                vec![segment]
            }
            (OpenNeighbourCarrier::FreeForm, _) | (_, OpenNeighbourCarrier::FreeForm) => {
                // No closed form exists for a fitted pair. `reintersect_carriers`
                // is the shared surface-type-blind seam: it tries the analytic
                // lane first (which declines here) and marches otherwise, with a
                // residual gate on the fitted section. Seed it on the strip's own
                // boundary — the branch being recovered runs right beside it.
                //
                // A PLANAR partner goes in as a patch covering the region gate
                // rather than as its own trimmed carrier: the marcher clamps to
                // both domains, and a plane patch that stops at the sharp edge
                // leaves the branch on its own boundary (see
                // `planar_region_patch`).
                let widen = |carrier: &OpenNeighbourCarrier,
                             surface: &NurbsSurface|
                 -> Option<NurbsSurface> {
                    match carrier {
                        OpenNeighbourCarrier::Planar(plane) => {
                            planar_region_patch(plane, f_center, f_reach).ok()
                        }
                        _ => Some(surface.clone()),
                    }
                };
                let (Some(first), Some(second)) = (
                    widen(&carriers[primary[0]], &surface_a),
                    widen(&carriers[primary[1]], &surface_b),
                ) else {
                    continue;
                };
                let policy = MarchPolicy {
                    tolerance,
                    residual_tolerance: plane_tolerance,
                    seeds: strip_points.clone(),
                };
                match reintersect_carriers(&first, &second, &policy) {
                    Ok(rim) => rim.curves(),
                    Err(refusal) => {
                        if debug {
                            eprintln!(
                                "HEAL open mixed: primaries {primary:?} do not re-intersect: {}",
                                refusal.describe()
                            );
                        }
                        refusals.push(format!(
                            "the fitted carriers of faces {} and {} do not re-intersect ({})",
                            neighbour_ids[primary[0]],
                            neighbour_ids[primary[1]],
                            refusal.describe()
                        ));
                        continue;
                    }
                }
            }
            _ => {
                let Some(branches) = intersect_analytic_pair(&surface_a, &surface_b, tolerance)
                else {
                    continue;
                };
                branches
            }
        };
        let Ok(cap_surface_first) = neighbour_surface(neighbour_ids[lateral[0]], &candidate) else {
            continue;
        };
        let Ok(cap_surface_second) = neighbour_surface(neighbour_ids[lateral[1]], &candidate)
        else {
            continue;
        };
        if debug {
            eprintln!(
                "HEAL open mixed: trying primaries {:?} laterals {:?} -> {} branch(es)",
                [neighbour_ids[primary[0]], neighbour_ids[primary[1]]],
                [neighbour_ids[lateral[0]], neighbour_ids[lateral[1]]],
                branches.len()
            );
        }
        // Every branch that crosses both caps inside the region gate is a
        // candidate; `nearest_branch` picks among them by geometry.
        let mut candidates: Vec<(NurbsCurve, [(f64, Vec3); 2])> = Vec::new();
        for curve in branches {
            let Some(first) = curve_cap_crossing_near(
                &curve,
                &carriers[lateral[0]],
                &cap_surface_first,
                f_center,
                f_reach,
                tolerance,
            ) else {
                continue;
            };
            let Some(second) = curve_cap_crossing_near(
                &curve,
                &carriers[lateral[1]],
                &cap_surface_second,
                f_center,
                f_reach,
                tolerance,
            ) else {
                continue;
            };
            let Ok(mid) = curve.evaluate((first.0 + second.0) * 0.5) else {
                continue;
            };
            if mid.sub(f_center).length() > f_reach {
                continue;
            }
            candidates.push((curve, [first, second]));
        }
        let corners: Vec<[Vec3; 2]> = candidates
            .iter()
            .map(|(_, crossings)| [crossings[0].1, crossings[1].1])
            .collect();
        let best = match nearest_branch(&corners, [contacts[lateral[0]], contacts[lateral[1]]], tolerance) {
            Ok(Some(index)) => Some(candidates.swap_remove(index)),
            Ok(None) => None,
            Err((chosen, rival)) => {
                let point = |p: Vec3| format!("({:.6}, {:.6}, {:.6})", p.x, p.y, p.z);
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "branch_tie",
                    format!(
                        "{op}: two re-intersection branches of faces {} and {} are equally near the \
                     strip's contacts with faces {} and {} (corners {} and {} against {} and {}) \
                     — refusing rather than choosing one by the order the intersector returned \
                     them",
                        neighbour_ids[primary[0]],
                        neighbour_ids[primary[1]],
                        neighbour_ids[lateral[0]],
                        neighbour_ids[lateral[1]],
                        point(corners[chosen][0]),
                        point(corners[chosen][1]),
                        point(corners[rival][0]),
                        point(corners[rival][1])
                    ),
                ));
            }
        };
        if let Some((branch, crossings)) = best {
            plans.push(MixedHealPlan {
                solid: candidate,
                primary,
                lateral,
                branch,
                crossings,
            });
        }
    }
    let plan = match plans.len() {
        1 => plans.pop().unwrap(),
        0 => {
            let detail = if refusals.is_empty() {
                String::new()
            } else {
                format!(" ({})", refusals.join("; "))
            };
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "no_spanning_branch",
                format!(
                    "{op}: no re-intersection branch of the extended carriers spans the deleted \
                 strip — refusing rather than emitting an invalid solid{detail}"
                ),
            ))
        }
        _ => {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "pairing_ambiguous",
                format!(
                    "{op}: healing is ambiguous — both opposite neighbour pairs re-intersect \
                 across the deleted strip"
                ),
            ))
        }
    };
    let mut solid = plan.solid;
    let primary = plan.primary;
    let lateral = plan.lateral;
    let branch_curve = plan.branch;
    let crossings = plan.crossings;
    let [branch_t0, branch_t1] = branch_curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let branch_span = (branch_t1 - branch_t0).max(1e-12);
    let snap = |t: f64| -> f64 {
        if (t - branch_t0).abs() <= 1e-9 * branch_span {
            branch_t0
        } else if (t - branch_t1).abs() <= 1e-9 * branch_span {
            branch_t1
        } else {
            t
        }
    };
    let t_first = snap(crossings[0].0);
    let t_second = snap(crossings[1].0);
    let point_first = branch_curve.evaluate(t_first).or_refuse(KernelStage::Sew, "evaluate")?;
    let point_second = branch_curve.evaluate(t_second).or_refuse(KernelStage::Sew, "evaluate")?;
    if (t_second - t_first).abs() <= 1e-9 * branch_span
        || point_first.sub(point_second).length() <= tolerance
    {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "corners_coincide",
            format!("{op}: recovered corners coincide — the carriers do not bound a clean edge"),
        ));
    }
    if debug {
        eprintln!(
            "HEAL open mixed: sharp edge t=[{t_first:.6},{t_second:.6}] between \
             ({:.4},{:.4},{:.4}) and ({:.4},{:.4},{:.4})",
            point_first.x,
            point_first.y,
            point_first.z,
            point_second.x,
            point_second.y,
            point_second.z
        );
    }

    // --- New vertices at the chain ends and the new sharp edge --------------
    let mut next_id = max_topology_id(&solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };
    let vertex_first = alloc();
    let vertex_second = alloc();
    let mut lateral_vertex: HashMap<usize, u64> = HashMap::default();
    lateral_vertex.insert(lateral[0], vertex_first);
    lateral_vertex.insert(lateral[1], vertex_second);
    let vertex_points: HashMap<u64, Vec3> =
        [(vertex_first, point_first), (vertex_second, point_second)]
            .into_iter()
            .collect();
    // Edge parameter must increase from t0 to t1.
    let (edge_t0, edge_t1, start_vertex, end_vertex) = if t_first < t_second {
        (t_first, t_second, vertex_first, vertex_second)
    } else {
        (t_second, t_first, vertex_second, vertex_first)
    };
    let sharp_edge_id = alloc();
    let sharp_edge = EdgeRecord {
        id: sharp_edge_id,
        curve: branch_curve.clone(),
        t0: edge_t0,
        t1: edge_t1,
        start_vertex_id: start_vertex,
        end_vertex_id: end_vertex,
        degenerate: false,
        name: None,
    };

    // --- Collapse each strip corner onto its lateral's recovered corner ----
    let lateral_set: HashSet<usize> = lateral.iter().copied().collect();
    let mut collapse: HashMap<u64, u64> = HashMap::default();
    for index in 0..4usize {
        let previous = (index + 3) % 4;
        let lateral_index = if lateral_set.contains(&previous) {
            previous
        } else if lateral_set.contains(&index) {
            index
        } else {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "corner_flank",
                format!(
                    "{op}: transition corner is not flanked by a lateral face (unexpected \
                 neighbour ordering)"
                ),
            ));
        };
        collapse.insert(transition_vertices[index], lateral_vertex[&lateral_index]);
    }

    // Relocate every non-strip edge that ends on a collapsed corner: widen
    // along its own stored curve when that reaches the corner, otherwise
    // re-derive the edge in closed form from its two flanking carriers.
    let mut relocated: HashSet<u64> = HashSet::default();
    let unmoved = solid.edges.clone();
    let pending = pending_edge_relocations(&solid, &boundary_edge_ids, &collapse, &vertex_points);
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
            record = rederive_side_edge_on_carriers(
                &solid,
                &solid.edges[index],
                start_target,
                end_target,
                plane_tolerance,
                tolerance,
                op,
            )?;
        }
        if debug {
            eprintln!(
                "HEAL open mixed: side edge {} {} (trim [{:.6},{:.6}])",
                record.id,
                if widened {
                    "widened/rebuilt"
                } else {
                    "re-derived"
                },
                record.t0,
                record.t1
            );
        }
        relocated.insert(record.id);
        solid.edges[index] = record;
    }
    // The planar neighbours, each on the one plane the gate reads it as and the
    // re-trim below rebuilds it on.
    let retrimmed: HashMap<u64, Plane> = neighbour_ids
        .iter()
        .zip(&carriers)
        .filter_map(|(&id, carrier)| match carrier {
            OpenNeighbourCarrier::Planar(plane) => Some((id, *plane)),
            _ => None,
        })
        .collect();
    moved_edges_lie_on_their_faces(&unmoved, &solid, &relocated, &retrimmed, plane_tolerance, op)?;

    // Index the (now relocated) edges for loop rewrites.
    let mut edges_by_id: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    edges_by_id.insert(sharp_edge_id, sharp_edge.clone());

    // --- Primary faces: swap the strip rim for the new sharp edge ----------
    for &primary_index in &primary {
        let primary_edge = boundary[primary_index].0;
        let neighbour_id = neighbour_ids[primary_index];
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing_neighbour(neighbour_id))?;
        let face = &mut solid.shells[ns].faces[nf];
        let (loop_index, coedge_index) = locate_coedge(face, primary_edge).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "primary_coedge",
                format!("{op}: neighbour {neighbour_id} does not use edge {primary_edge}"),
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
        let required_from = coedge_to_vertex(previous, &edges_by_id).ok_or_else(unresolved)?;
        let required_to = coedge_from_vertex(next, &edges_by_id).ok_or_else(unresolved)?;
        let forward = if required_from == start_vertex && required_to == end_vertex {
            true
        } else if required_from == end_vertex && required_to == start_vertex {
            false
        } else {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "primary_loop",
                format!("{op}: new edge does not close the primary loop (unexpected connectivity)"),
            ));
        };
        let new_coedge = CoedgeRecord {
            id: alloc(),
            edge_id: sharp_edge_id,
            forward,
            // Placeholder; the retrim/refit pass below recomputes it.
            pcurve: make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0))
                .or_refuse(KernelStage::Sew, "make_line")?,
        };
        face.loops[loop_index].coedges[coedge_index] = new_coedge;
    }

    // --- Lateral (cap) faces: drop the transverse blend coedge -------------
    for &lateral_index in &lateral {
        let lateral_edge = boundary[lateral_index].0;
        let neighbour_id = neighbour_ids[lateral_index];
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing_neighbour(neighbour_id))?;
        let face = &mut solid.shells[ns].faces[nf];
        let (loop_index, coedge_index) = locate_coedge(face, lateral_edge).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "lateral_coedge",
                format!("{op}: neighbour {neighbour_id} does not use edge {lateral_edge}"),
            )
        })?;
        face.loops[loop_index].coedges.remove(coedge_index);
        if face.loops[loop_index].coedges.is_empty() {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "emptied_loop",
                format!("{op}: healing emptied a lateral face loop"),
            ));
        }
    }

    // --- Prune the strip, its edges, and its corner vertices ----------------
    solid.shells[shell_index]
        .faces
        .retain(|face| face.id != face_id);
    solid
        .edges
        .retain(|edge| !boundary_edge_ids.contains(&edge.id));
    let removed_vertices: HashSet<u64> = transition_vertices.iter().copied().collect();
    solid.edges.push(sharp_edge);
    solid
        .vertices
        .retain(|vertex| !removed_vertices.contains(&vertex.id));
    solid.vertices.push(VertexRecord {
        id: vertex_first,
        point: point_first,
    });
    solid.vertices.push(VertexRecord {
        id: vertex_second,
        point: point_second,
    });

    // --- Re-trim / refit every affected neighbour ---------------------------
    let final_edges: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    let mut touched: HashSet<u64> = relocated.clone();
    touched.insert(sharp_edge_id);
    for &neighbour_id in neighbour_ids {
        let (ns, nf) = find_face(&solid, neighbour_id).ok_or_else(|| missing_neighbour(neighbour_id))?;
        match retrimmed.get(&neighbour_id) {
            Some(plane) => {
                retrim_planar_face(
                    &mut solid.shells[ns].faces[nf],
                    plane,
                    &final_edges,
                    scale,
                    op,
                )
                ?;
                // The planar retrim maps each edge's WHOLE curve; EVERY edge
                // of the face that represents a strict subrange of its curve
                // (widened side edges, but equally untouched edges an earlier
                // boolean trimmed) needs the range fitter so its pcurve spans
                // exactly [t0, t1].
                let face_edges: HashSet<u64> = solid.shells[ns].faces[nf]
                    .loops
                    .iter()
                    .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.edge_id))
                    .collect();
                refit_touched_pcurves(
                    &mut solid.shells[ns].faces[nf],
                    &final_edges,
                    &face_edges,
                    true,
                    tolerance,
                    op,
                )?;
            }
            None => {
                // Carrier untouched, exactly extended, or naturally extended —
                // all three keep the original parameterization, so the face's
                // existing pcurves are still its own trims and only the widened
                // and newly created edges need fresh ones. This is what makes
                // the natural extension usable here at all: it appends a span
                // rather than refining the net, so nothing already drawn on the
                // face has to be remapped.
                refit_touched_pcurves(
                    &mut solid.shells[ns].faces[nf],
                    &final_edges,
                    &touched,
                    false,
                    tolerance,
                    op,
                )?;
            }
        }
    }

    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validation",
            format!("{op}: open-chain mixed heal failed validation: {issues:?}"),
        ));
    }
    Ok(solid)
}
