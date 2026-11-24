use super::*;
use crate::{KernelStage, OrRefuse};

/// A midpoint alone cannot distinguish a straight surface ruling from a curved
/// profile that happens to cross its endpoint chord at half-span.  Sample each
/// eighth so both half-spans must lie on the carrier before a synthetic chord
/// is accepted as edge geometry.
pub(super) const RULING_INTERIOR_STATIONS: [f64; 7] = [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875];

pub(in crate::step_import) fn ruling_is_on_surface_at_interior_stations(
    mut station_is_on_surface: impl FnMut(f64) -> bool,
) -> bool {
    RULING_INTERIOR_STATIONS
        .into_iter()
        .all(&mut station_is_on_surface)
}

pub(in crate::step_import) fn prefer_exact_general_ruling(analytic: Option<&crate::AnalyticSurface>) -> bool {
    matches!(
        analytic,
        None | Some(crate::AnalyticSurface::Revolution { .. })
    )
}

fn closed_shell_face_refs(resolver: &Resolver, shell_ref: usize) -> Result<Vec<Value>, String> {
    resolver
        .get(shell_ref)?
        .find("CLOSED_SHELL")
        .and_then(|args| args.get(1))
        .ok_or_else(|| format!("step_import: #{shell_ref} is not a CLOSED_SHELL"))?
        .as_list()
        .map(|faces| faces.to_vec())
}

fn build_shell_faces_in_namespace(
    builder: &mut SolidBuilder<'_>,
    face_refs: &[Value],
) -> Result<Vec<FaceRecord>, String> {
    let mut pending = Vec::with_capacity(face_refs.len());
    for face_value in face_refs {
        let face_ref = face_value.as_ref_id()?;
        pending.push(
            builder
                .collect_face(face_ref)
                .map_err(|error| format!("step_import: face #{face_ref}: {error}"))?,
        );
    }
    builder.close_vertex_loop_tori(&mut pending)?;
    builder.relocate_periodic_rim_seams(&pending)?;
    builder.split_seam_crossing_edges(&mut pending)?;
    // Phase 1.75: a planar face whose straight boundary sits off its plane
    // (mesh-derived region fits) becomes exactly planar triangles BEFORE the
    // reconcile step below can bend its edges onto one plane or the other.
    builder.split_off_plane_planar_faces(&mut pending)?;
    builder.seat_vertices_on_conic_ends()?;
    builder.reconcile_edges_onto_surfaces(&pending)?;
    let mut faces = Vec::with_capacity(pending.len());
    // The planar split can have minted more pending faces than STEP faces, so
    // each pending face carries its own source reference.
    for pending_face in pending {
        let face_ref = pending_face.face_ref;
        faces.push(
            builder
                .finish_face(pending_face)
                .map_err(|error| format!("step_import: face #{face_ref}: {error}"))?,
        );
    }
    builder.report_supplied_pcurves();
    Ok(faces)
}

fn brep_with_voids_volume_tolerance(solid: &BrepSolid) -> f64 {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut coordinate_scale = 0.0f64;
    for vertex in &solid.vertices {
        lo = Vec3::new(
            lo.x.min(vertex.point.x),
            lo.y.min(vertex.point.y),
            lo.z.min(vertex.point.z),
        );
        hi = Vec3::new(
            hi.x.max(vertex.point.x),
            hi.y.max(vertex.point.y),
            hi.z.max(vertex.point.z),
        );
        coordinate_scale = coordinate_scale
            .max(vertex.point.x.abs())
            .max(vertex.point.y.abs())
            .max(vertex.point.z.abs());
    }
    let extent = if solid.vertices.is_empty() {
        0.0
    } else {
        hi.sub(lo).length()
    };
    let face_count = solid
        .shells
        .iter()
        .map(|shell| shell.faces.len())
        .sum::<usize>()
        .max(1) as f64;
    256.0 * f64::EPSILON * face_count * coordinate_scale.max(extent) * extent * extent
}

/// The slug every `BREP_WITH_VOIDS` certification refuses under: the file's
/// shells are not the disjoint closed shells the entity promises
/// (`RefusalClass::InvalidInput`). Classified at the origin since
/// 2026-10-03; every message is the text it always was.
const BREP_WITH_VOIDS_INPUT: &str = "brep_with_voids";

fn brep_with_voids_input(message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Validate, BREP_WITH_VOIDS_INPUT, message)
}

/// `validate()`'s verdict on a reconstructed body, typed as the boolean types
/// its own (`RefusalClass::InvalidResultTopology`); the message is the text
/// the importer always produced.
fn validation_refusal(issues: usize, message: String) -> KernelRefusal {
    KernelRefusal::new(
        crate::RefusalClass::InvalidResultTopology { issues: issues as u32 },
        KernelStage::Validate,
        message,
    )
}

fn certify_brep_with_voids_topology(solid: &BrepSolid) -> Result<(), KernelRefusal> {
    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut claimed_edges = HashSet::default();
    let mut claimed_vertices = HashSet::default();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        let mut uses: HashMap<u64, Vec<bool>> = HashMap::default();
        for face in &shell.faces {
            for face_loop in &face.loops {
                for coedge in &face_loop.coedges {
                    uses.entry(coedge.edge_id).or_default().push(coedge.forward);
                }
            }
        }
        let shell_edges: HashSet<u64> = uses.keys().copied().collect();
        if shell_edges
            .iter()
            .any(|edge_id| claimed_edges.contains(edge_id))
        {
            return Err(brep_with_voids_input(format!(
                "step_import: BREP_WITH_VOIDS shell {shell_index} shares an edge reference with another shell"
            )));
        }
        let mut shell_vertices = HashSet::default();
        for (edge_id, senses) in &uses {
            let edge = edges.get(edge_id).ok_or_else(|| {
                brep_with_voids_input(format!("step_import: BREP_WITH_VOIDS shell references missing edge {edge_id}"))
            })?;
            if !edge.degenerate && (senses.len() != 2 || senses[0] == senses[1]) {
                return Err(brep_with_voids_input(format!(
                    "step_import: BREP_WITH_VOIDS shell {shell_index} edge {edge_id} has incidence {senses:?}"
                )));
            }
            shell_vertices.insert(edge.start_vertex_id);
            shell_vertices.insert(edge.end_vertex_id);
        }
        if shell_vertices
            .iter()
            .any(|vertex_id| claimed_vertices.contains(vertex_id))
        {
            return Err(brep_with_voids_input(format!(
                "step_import: BREP_WITH_VOIDS shell {shell_index} shares a vertex reference with another shell"
            )));
        }
        claimed_edges.extend(shell_edges);
        claimed_vertices.extend(shell_vertices);
    }
    Ok(())
}

pub(in crate::step_import) fn certify_brep_with_voids_containment(solid: &BrepSolid) -> Result<(), KernelRefusal> {
    let edge_map: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let vertex_map: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let shell_vertex_ids = |shell: &ShellRecord| -> Result<HashSet<u64>, String> {
        let mut ids = HashSet::default();
        for edge_id in shell
            .faces
            .iter()
            .flat_map(|face| &face.loops)
            .flat_map(|face_loop| &face_loop.coedges)
            .map(|coedge| coedge.edge_id)
        {
            let edge = edge_map.get(&edge_id).ok_or_else(|| {
                format!("step_import: BREP_WITH_VOIDS shell references missing edge {edge_id}")
            })?;
            ids.insert(edge.start_vertex_id);
            ids.insert(edge.end_vertex_id);
        }
        Ok(ids)
    };
    let bounds = |ids: &HashSet<u64>| -> Result<(Vec3, Vec3), String> {
        if ids.is_empty() {
            return Err("step_import: BREP_WITH_VOIDS shell has no vertices".into());
        }
        let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for id in ids {
            let point = vertex_map.get(id).ok_or_else(|| {
                format!("step_import: BREP_WITH_VOIDS shell references missing vertex {id}")
            })?;
            lo = Vec3::new(lo.x.min(point.x), lo.y.min(point.y), lo.z.min(point.z));
            hi = Vec3::new(hi.x.max(point.x), hi.y.max(point.y), hi.z.max(point.z));
        }
        Ok((lo, hi))
    };

    let outer_ids = shell_vertex_ids(&solid.shells[0]).map_err(brep_with_voids_input)?;
    let (outer_lo, outer_hi) = bounds(&outer_ids).map_err(brep_with_voids_input)?;
    let extent = outer_hi.sub(outer_lo).length();
    let tolerance = (extent * 1e-9).max(1e-12);
    // Pick the containment-certification lane. A straight-edged polyhedron admits
    // the fast, fully-sound halfspace test: a convex planar outer shell means a
    // void vertex on the inner side of every face plane is inside. A body with any
    // curved face or edge takes the sampled lane below, ray-classifying void
    // samples against the true outer shell — sound for an arbitrary closed outer,
    // just costlier. (OCC performs no containment check here at all; this keeps a
    // real one while no longer rejecting the many valid curved BREP_WITH_VOIDS.)
    let all_faces_planar = solid.shells.iter().all(|shell| {
        shell.faces.iter().all(|face| {
            matches!(
                face.surface.analytic(),
                Some(crate::AnalyticSurface::Plane { .. })
            )
        })
    });
    let all_edges_lines = solid.edges.iter().all(|edge| {
        edge.degenerate || (edge.curve.degree == 1 && edge.curve.control_points.len() == 2)
    });
    let polyhedral = all_faces_planar && all_edges_lines;

    // Polyhedral lane: the outer shell must be the convex intersection of its
    // oriented face halfspaces, so every outer vertex lies on the inner side of
    // every face plane. Curved bodies skip this and use the sampled lane.
    if polyhedral {
        for face in &solid.shells[0].faces {
            let Some(crate::AnalyticSurface::Plane {
                origin,
                u_dir,
                v_dir,
                ..
            }) = face.surface.analytic()
            else {
                unreachable!("all faces planar in the polyhedral lane");
            };
            let mut outward = u_dir.cross(*v_dir).normalized().or_refuse(KernelStage::Validate, "brep_with_voids_normal")?;
            if !face.same_sense {
                outward = outward.scale(-1.0);
            }
            for vertex_id in &outer_ids {
                let signed_distance = vertex_map[vertex_id].sub(*origin).dot(outward);
                if signed_distance > tolerance {
                    return Err(brep_with_voids_input(format!(
                        "step_import: BREP_WITH_VOIDS outer shell is not convex at face {} vertex {vertex_id} (outside halfspace by {signed_distance})",
                        face.id
                    )));
                }
            }
        }
    }
    let outer = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: solid.id,
        vertices: solid.vertices.clone(),
        edges: solid.edges.clone(),
        shells: vec![solid.shells[0].clone()],
        genus: 0,
        kind: crate::BodyKind::Solid,
    };
    // Build the outer-shell point classifier ONCE and reuse it for every void
    // vertex/edge sample. `classify_point` rebuilds this per call, which for a
    // curved void (many samples) is O(samples x outer-faces) and can blow the
    // import timeout; a single build gives byte-identical results far faster.
    let outer_classifier = crate::SolidClassifier::new(&outer, tolerance).or_refuse(KernelStage::Validate, "brep_with_voids_classifier")?;
    let mut void_bounds = Vec::new();
    for (shell_index, shell) in solid.shells.iter().enumerate().skip(1) {
        let ids = shell_vertex_ids(shell).map_err(brep_with_voids_input)?;
        let (lo, hi) = bounds(&ids).map_err(brep_with_voids_input)?;
        let bbox_clearances = [
            lo.x - outer_lo.x,
            outer_hi.x - hi.x,
            lo.y - outer_lo.y,
            outer_hi.y - hi.y,
            lo.z - outer_lo.z,
            outer_hi.z - hi.z,
        ];
        // Fast bbox pre-filter — sound only when the outer bbox equals its vertex
        // bbox, i.e. a straight-edged polyhedron. A curved outer surface can bulge
        // past its vertices, so the sampled lane skips this and trusts the
        // classify_point checks below.
        if polyhedral {
            let strictly_inside_box = lo.x > outer_lo.x + tolerance
                && lo.y > outer_lo.y + tolerance
                && lo.z > outer_lo.z + tolerance
                && hi.x < outer_hi.x - tolerance
                && hi.y < outer_hi.y - tolerance
                && hi.z < outer_hi.z - tolerance;
            if !strictly_inside_box {
                return Err(brep_with_voids_input(format!(
                    "step_import: BREP_WITH_VOIDS void {shell_index} is not strictly inside the outer bounds"
                )));
            }
        }
        for vertex_id in &ids {
            let point = vertex_map[vertex_id];
            let class = outer_classifier.classify(point).or_refuse(KernelStage::Validate, "brep_with_voids_classify")?.class;
            if class != crate::PointClass::In {
                let projection_distance = outer.shells[0]
                    .faces
                    .iter()
                    .filter_map(|face| {
                        crate::project_point_to_surface(&face.surface, point)
                            .ok()
                            .map(|projection| projection.distance)
                    })
                    .fold(f64::INFINITY, f64::min);
                return Err(brep_with_voids_input(format!(
                    "step_import: BREP_WITH_VOIDS void {shell_index} vertex {vertex_id} at {point:?} is {class:?} relative to outer shell (classifier tolerance {tolerance}, nearest carrier projection {projection_distance}, bbox clearances {bbox_clearances:?})"
                )));
            }
        }
        // Curved lane only: a curved void edge can bow through the outer boundary
        // between two contained endpoints, so sample each void edge's interior.
        // Straight void edges cannot, so the polyhedral lane needs only vertices.
        if !polyhedral {
            const VOID_EDGE_SAMPLES: usize = 5;
            let mut sampled: HashSet<u64> = HashSet::default();
            for face in &shell.faces {
                for face_loop in &face.loops {
                    for coedge in &face_loop.coedges {
                        if !sampled.insert(coedge.edge_id) {
                            continue;
                        }
                        let edge = edge_map[&coedge.edge_id];
                        if edge.degenerate {
                            continue;
                        }
                        for step in 1..VOID_EDGE_SAMPLES {
                            let fraction = step as f64 / VOID_EDGE_SAMPLES as f64;
                            let t = edge.t0 + (edge.t1 - edge.t0) * fraction;
                            let Ok(point) = edge.curve.evaluate(t) else {
                                continue;
                            };
                            if outer_classifier.classify(point).or_refuse(KernelStage::Validate, "brep_with_voids_classify")?.class == crate::PointClass::Out {
                                return Err(brep_with_voids_input(format!(
                                    "step_import: BREP_WITH_VOIDS void {shell_index} edge {} bows outside the outer shell",
                                    coedge.edge_id
                                )));
                            }
                        }
                    }
                }
            }
        }
        void_bounds.push((lo, hi));
    }
    // Void-void disjointness via vertex bboxes — again exact only for polyhedra,
    // so it stays in the polyhedral lane. Overlapping voids are a pathological
    // authoring error OCC does not screen for either; the per-void containment
    // above is the substantive curved-lane guarantee.
    if polyhedral {
        for first in 0..void_bounds.len() {
            for second in (first + 1)..void_bounds.len() {
                let (a_lo, a_hi) = void_bounds[first];
                let (b_lo, b_hi) = void_bounds[second];
                let separated = a_hi.x + tolerance < b_lo.x
                    || b_hi.x + tolerance < a_lo.x
                    || a_hi.y + tolerance < b_lo.y
                    || b_hi.y + tolerance < a_lo.y
                    || a_hi.z + tolerance < b_lo.z
                    || b_hi.z + tolerance < a_lo.z;
                if !separated {
                    return Err(brep_with_voids_input(format!(
                        "step_import: BREP_WITH_VOIDS void bounds {} and {} overlap",
                        first + 1,
                        second + 1
                    )));
                }
            }
        }
    }
    Ok(())
}

/// Reconstruct the outer shell and voids, optionally carrying the per-edge capture.
pub(in crate::step_import) fn build_brep_with_voids_captured(
    resolver: &Resolver,
    body_ref: usize,
    capture: bool,
) -> Result<(BrepSolid, Option<readings::TrimCapture>, readings::BodyReadings), StepBodyError> {
    let args = resolver
        .get(body_ref)?
        .find("BREP_WITH_VOIDS")
        .ok_or("step_import: body is not BREP_WITH_VOIDS")?;
    let outer_ref = args
        .get(1)
        .ok_or("step_import: BREP_WITH_VOIDS missing outer shell")?
        .as_ref_id()?;
    let void_values = args
        .get(2)
        .ok_or("step_import: BREP_WITH_VOIDS missing void shell list")?
        .as_list()?;
    if void_values.is_empty() {
        return Err(brep_with_voids_input("step_import: BREP_WITH_VOIDS has no void shells".into()).into());
    }

    let mut shell_specs = vec![(outer_ref, true)];
    let mut shell_refs = HashSet::default();
    shell_refs.insert(outer_ref);
    for value in void_values {
        let oriented_ref = value.as_ref_id()?;
        let oriented = resolver
            .get(oriented_ref)?
            .find("ORIENTED_CLOSED_SHELL")
            .ok_or_else(|| {
                format!("step_import: void #{oriented_ref} is not ORIENTED_CLOSED_SHELL")
            })?;
        let base_ref = oriented
            .get(2)
            .ok_or("step_import: ORIENTED_CLOSED_SHELL missing base shell")?
            .as_ref_id()?;
        let orientation = oriented
            .get(3)
            .ok_or("step_import: ORIENTED_CLOSED_SHELL missing orientation")?;
        let same_sense = if orientation.enum_is("T") {
            true
        } else if orientation.enum_is("F") {
            false
        } else {
            return Err(brep_with_voids_input("step_import: ORIENTED_CLOSED_SHELL orientation is not .T./.F.".into()).into());
        };
        if !shell_refs.insert(base_ref) {
            return Err(brep_with_voids_input(format!(
                "step_import: BREP_WITH_VOIDS repeats CLOSED_SHELL #{base_ref}"
            )).into());
        }
        shell_specs.push((base_ref, same_sense));
    }

    let mut all_face_refs = HashSet::default();
    let mut face_lists = Vec::with_capacity(shell_specs.len());
    for (shell_ref, _) in &shell_specs {
        let faces = closed_shell_face_refs(resolver, *shell_ref)?;
        for face in &faces {
            let face_ref = face.as_ref_id()?;
            if !all_face_refs.insert(face_ref) {
                return Err(brep_with_voids_input(format!(
                    "step_import: BREP_WITH_VOIDS face #{face_ref} belongs to multiple shells"
                )).into());
            }
        }
        face_lists.push(faces);
    }

    let mut builder = SolidBuilder {
        resolver,
        next_id: 1,
        vertices: Vec::new(),
        edges: Vec::new(),
        vertex_of_ref: HashMap::default(),
        edge_of_ref: HashMap::default(),
        edge_of_vertex_pair: HashMap::default(),
        vertex_index: HashMap::default(),
        edge_index: HashMap::default(),
        curve_ref_of_edge: HashMap::default(),
        supplied_report: SuppliedReport::default(),
        bounded: Vec::new(),
        readings: capture.then(readings::TrimCapture::default),
        face_refs: HashMap::default(),
        reconciled_edges: HashSet::default(),
        body_faces: all_face_refs.len(),
        open_shell: false,
    };
    let mut shells = Vec::with_capacity(face_lists.len());
    for ((_, same_sense), faces) in shell_specs.iter().zip(&face_lists) {
        let shell_faces = build_shell_faces_in_namespace(&mut builder, faces)?;
        let mut shell = ShellRecord {
            id: builder.fresh(),
            faces: shell_faces,
        };
        if !same_sense {
            crate::orientation::flip_shell_faces(&mut shell)?;
        }
        shells.push(shell);
    }
    let solid_id = builder.fresh();
    let captured = builder.readings.take();
    let bounded = std::mem::take(&mut builder.bounded);
    let closure_context = builder.closure_context();
    let mut solid = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: solid_id,
        vertices: builder.vertices,
        edges: builder.edges,
        shells,
        genus: 0,
        kind: crate::BodyKind::Solid,
    };
    solid = finalize_imported_solid(solid)?;
    solid.genus = euler_genus(&solid);
    certify_brep_with_voids_topology(&solid)?;
    let issues = solid.validate();
    if !issues.is_empty() {
        dump_invalid_solid_for_triage(&solid);
        return Err(validation_refusal(
            issues.len(),
            format!("step_import: reconstructed BREP_WITH_VOIDS failed validation: {issues:?}"),
        )
        .into());
    }
    let tolerance = brep_with_voids_volume_tolerance(&solid);
    let _caller = crate::mass_caller("step_import.brep_with_voids");
    let mut shell_volumes = solid
        .shells
        .iter()
        .map(crate::mass_properties::shell_signed_volume)
        .collect::<Result<Vec<_>, _>>()?;
    if shell_volumes
        .iter()
        .any(|volume| !volume.is_finite() || volume.abs() <= tolerance)
    {
        return Err(brep_with_voids_input(format!(
            "step_import: BREP_WITH_VOIDS shell volume is below tolerance {tolerance}: {shell_volumes:?}"
        )).into());
    }
    let authored_roles = shell_volumes[0] > tolerance
        && shell_volumes
            .iter()
            .skip(1)
            .all(|volume| *volume < -tolerance);
    let globally_reversed = shell_volumes[0] < -tolerance
        && shell_volumes
            .iter()
            .skip(1)
            .all(|volume| *volume > tolerance);
    if globally_reversed {
        crate::orientation::flip_all_faces(&mut solid)?;
        for volume in &mut shell_volumes {
            *volume = -*volume;
        }
    } else if !authored_roles && shell_volumes[0] > tolerance {
        // A void encloses positive volume: its base shell already faces into
        // the void and the `.F.` wrapper turned it outward again (one CAD
        // exporter writes every void this way), or a `.T.` wrapper was given a
        // base shell authored as a void. A void can only remove material, so
        // the shell is turned to face the void; the containment check below
        // still refuses a "void" that does not lie inside the outer shell.
        for (shell, volume) in solid.shells.iter_mut().zip(&mut shell_volumes).skip(1) {
            if *volume > tolerance {
                crate::orientation::flip_shell_faces(shell)?;
                *volume = -*volume;
            }
        }
    } else if !authored_roles {
        return Err(brep_with_voids_input(format!(
            "step_import: BREP_WITH_VOIDS has mixed outer/void orientations {shell_volumes:?}"
        )).into());
    }
    certify_brep_with_voids_containment(&solid)?;
    let outer_volume = shell_volumes[0];
    let net_volume = shell_volumes.iter().sum::<f64>();
    if outer_volume <= tolerance || !net_volume.is_finite() || net_volume <= tolerance {
        return Err(brep_with_voids_input(format!(
            "step_import: BREP_WITH_VOIDS invalid outer/net volume {outer_volume}/{net_volume} (tolerance {tolerance})"
        )).into());
    }
    solid.genus = euler_genus(&solid);
    // The closure gate, after every other acceptance check: a body whose
    // shells' trims do not enclose zero vector area is refused, typed
    // (`builder/closure.rs`); the capture lane holds the refusal and keeps the
    // body so the residual can be attributed.
    let closure = closure_context.certify_shell_closure(&solid, outer_ref, capture)?;
    Ok((solid, captured, readings::BodyReadings { bounded, closure }))
}

pub(in crate::step_import) fn build_solid_inner(
    resolver: &Resolver,
    manifold_ref: usize,
    capture: bool,
) -> Result<(BrepSolid, Option<readings::TrimCapture>, readings::BodyReadings), StepBodyError> {
    let entity = resolver.get(manifold_ref)?;
    // MANIFOLD_SOLID_BREP(name, #shell) and FACETED_BREP(name, #shell) share the
    // same shape: a single CLOSED_SHELL as the second argument.
    let shell_ref = entity
        .find("MANIFOLD_SOLID_BREP")
        .or_else(|| entity.find("FACETED_BREP"))
        .and_then(|args| args.get(1))
        .ok_or("step_import: solid brep missing shell")?
        .as_ref_id()?;
    build_solid_from_shell_captured(resolver, shell_ref, capture)
}

/// Reconstruct one shell, optionally carrying what the importer did to
/// every edge (`capture`), for [`crate::import_step_trim_readings`].
pub(in crate::step_import) fn build_solid_from_shell_captured(
    resolver: &Resolver,
    shell_ref: usize,
    capture: bool,
) -> Result<(BrepSolid, Option<readings::TrimCapture>, readings::BodyReadings), StepBodyError> {
    let shell_entity = resolver.get(shell_ref)?;
    let open_shell = shell_entity.find("CLOSED_SHELL").is_none();
    let face_refs = shell_entity
        .find("CLOSED_SHELL")
        .or_else(|| shell_entity.find("OPEN_SHELL"))
        .and_then(|args| args.get(1))
        .ok_or("step_import: shell is not CLOSED_SHELL/OPEN_SHELL")?
        .as_list()?
        .to_vec();

    let mut builder = SolidBuilder {
        resolver,
        next_id: 1,
        vertices: Vec::new(),
        edges: Vec::new(),
        vertex_of_ref: HashMap::default(),
        edge_of_ref: HashMap::default(),
        edge_of_vertex_pair: HashMap::default(),
        vertex_index: HashMap::default(),
        edge_index: HashMap::default(),
        curve_ref_of_edge: HashMap::default(),
        supplied_report: SuppliedReport::default(),
        bounded: Vec::new(),
        readings: capture.then(readings::TrimCapture::default),
        face_refs: HashMap::default(),
        reconciled_edges: HashSet::default(),
        body_faces: face_refs.len(),
        open_shell,
    };

    // Phase 1: resolve every face's surface + coedge bounds (this also builds
    // and caches all shared edges/vertices). Phase 1.5: re-seat OCC
    // full-cylinder rim circles onto the surface seam meridian — a mutation of
    // shared edges/vertices that must precede any pcurve derivation. Phase 2:
    // assemble each face's loops/pcurves from the (now consistent) topology.
    // Phase wall-clock on stderr when `BREP_PROFILE` is set — the switch the
    // boolean's `boolean.<stage>_ms` lines use.
    let profile = crate::profile::profile_enabled();
    let mut lap_started = web_time::Instant::now();
    let mut laps: Vec<(&str, f64)> = Vec::new();
    let lap = |name: &'static str, started: &mut web_time::Instant, laps: &mut Vec<(&str, f64)>| {
        if profile {
            let now = web_time::Instant::now();
            laps.push((name, (now - *started).as_secs_f64() * 1_000.0));
            *started = now;
        }
    };
    let mut pending = Vec::with_capacity(face_refs.len());
    for face_value in &face_refs {
        let face_ref = face_value.as_ref_id()?;
        pending.push(
            builder
                .collect_face(face_ref)
                .map_err(|error| format!("step_import: face #{face_ref}: {error}"))?,
        );
    }
    lap("collect_faces", &mut lap_started, &mut laps);
    builder.close_vertex_loop_tori(&mut pending)?;
    builder.relocate_periodic_rim_seams(&pending)?;
    builder.split_seam_crossing_edges(&mut pending)?;
    lap("seams", &mut lap_started, &mut laps);
    // Phase 1.75: a planar face whose straight boundary sits off its plane
    // (mesh-derived region fits) becomes exactly planar triangles BEFORE the
    // reconcile step below can bend its edges onto one plane or the other.
    builder.split_off_plane_planar_faces(&mut pending)?;
    lap("planar_split", &mut lap_started, &mut laps);
    builder.seat_vertices_on_conic_ends()?;
    builder.reconcile_edges_onto_surfaces(&pending)?;
    lap("reconcile", &mut lap_started, &mut laps);
    // `BREP_STEP_PREFIT`: a large body's projection fits start on idle
    // threads; the loop below takes the finished ones (`builder/prefit.rs`).
    let mut prefits = builder.spawn_prefits(&pending)?;
    if prefits.is_some() {
        lap("prefit", &mut lap_started, &mut laps);
    }
    // `BREP_STEP_ALIAS_VERDICTS`: the helpers' aliasing verdicts, taken with
    // their faces' fits and read by the face loop (`builder/alias_verdicts.rs`).
    let alias_verdicts = super::alias_verdicts::MemoScope::open(prefits.as_ref().is_some_and(|prefits| prefits.verdicts()));
    let mut faces = Vec::with_capacity(pending.len());
    // The planar split can have minted more pending faces than STEP faces, so
    // each pending face carries its own source reference.
    let face_count = pending.len();
    // `BREP_PROFILE_WALL` (profile-wall builds): the face loop's time split by
    // what the pcurve work under it was doing.
    let _wall = crate::wall_profile::Stage::open("step_finish_faces");
    // The lap totals below are this loop's alone: an earlier build on this
    // thread that failed mid-loop must not be counted here.
    if profile {
        super::loops::take_loop_laps();
        crate::pcurve::take_station_fit_laps();
    }
    for (index, pending_face) in pending.into_iter().enumerate() {
        let face_ref = pending_face.face_ref;
        let face_started = profile.then(web_time::Instant::now);
        let offered = prefits.as_mut().map(|prefits| prefits.claim(index));
        let finished = builder.finish_face(pending_face);
        drop(offered);
        // Under the profile switch, a face that takes over a second is named
        // with its place in the shell, and so is a face that fails: how much of
        // a refused body's time went before the face that refused it.
        if let Some(started) = face_started {
            let elapsed = started.elapsed();
            if elapsed.as_secs_f64() > 1.0 || finished.is_err() {
                profile_line!(
                    "step_import.face shell=#{shell_ref} face=#{face_ref} index={index}/{face_count} ms={:.1} ok={} body_ms_so_far={:.1}",
                    elapsed.as_secs_f64() * 1_000.0,
                    finished.is_ok(),
                    lap_started.elapsed().as_secs_f64() * 1_000.0
                );
            }
        }
        faces.push(finished.map_err(|error| format!("step_import: face #{face_ref}: {error}"))?);
    }
    lap("finish_faces", &mut lap_started, &mut laps);
    if profile {
        let [ask, floor, refine, readings, supplied, analytic] = super::loops::take_loop_laps();
        profile_line!("step_import.loop_laps ask_fit={ask:.1} floor_refit={floor:.1} refine={refine:.1} readings={readings:.1} supplied={supplied:.1} analytic={analytic:.1}");
        let [setup, probes, inserts, repair, interpolant, residual, other] = crate::pcurve::take_station_fit_laps();
        profile_line!("step_import.station_laps setup={setup:.1} probes={probes:.1} inserts={inserts:.1} repair={repair:.1} interpolant={interpolant:.1} residual={residual:.1} tracking_probes={other:.1}");
    }
    if let Some(prefits) = prefits.take() {
        prefits.report(shell_ref);
    }
    alias_verdicts.report(shell_ref);
    drop(alias_verdicts);

    builder.report_supplied_pcurves();
    let shell_id = builder.fresh();
    let solid_id = builder.fresh();
    let captured = builder.readings.take();
    let bounded = std::mem::take(&mut builder.bounded);
    let closure_context = builder.closure_context();
    let mut solid = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: solid_id,
        vertices: builder.vertices,
        edges: builder.edges,
        shells: vec![ShellRecord {
            id: shell_id,
            faces,
        }],
        genus: 0,
        // An open shell is a sheet where the caller asked for sheets.
        kind: if open_shell && resolver.sheets { crate::BodyKind::Sheet } else { crate::BodyKind::Solid },
    };
    solid = finalize_imported_solid(solid)?;
    solid.genus = euler_genus(&solid);
    lap("finalize", &mut lap_started, &mut laps);

    let issues = solid.validate();
    lap("validate", &mut lap_started, &mut laps);
    if !issues.is_empty() {
        // Debug escape hatch: dump the invalid solid for offline triage
        // before refusing (validation issues reference face/coedge ids that
        // only exist on this in-flight solid).
        dump_invalid_solid_for_triage(&solid);
        // Strict again: the vendor quirks that once needed a tolerance lane
        // are now REPAIRED structurally (coincident parallel edges credited
        // in the Euler count; pinched vertices split into their fans), so
        // any residual issue is a genuine import defect.
        return Err(validation_refusal(
            issues.len(),
            format!("step_import: reconstructed solid failed validation: {:?}", issues),
        )
        .into());
    }
    // A sheet bounds nothing: no material side to restore, and no closure to
    // certify. Its faces keep the senses the file gives them.
    if solid.kind == crate::BodyKind::Sheet {
        return Ok((solid, captured, readings::BodyReadings { bounded, closure: ClosureNote::Unjudged }));
    }
    // A face whose outer loop winds against the sense the file gives it
    // takes its loop's: the loop's edges agree with its neighbours' (the shell
    // validated), so the sense is the file's slip. SolidWorks wrote one face
    // of a keycap `.F.` with its loop counter-clockwise about its carrier's
    // own normal, which points out of the solid; read as given, its volume
    // term had the wrong sign (the part 3% large; OpenCASCADE's reader heals
    // the face the same way).
    resense_faces_by_winding(&mut solid);
    // Vendor face senses can leave the whole shell pointing inward; a closed
    // shell's material side is defined by its signed volume, so restore the
    // outward convention.
    //
    // Only the SIGN is read, so the sign reader answers it: its verdict against
    // zero is the exact integral's (it integrates exactly itself inside its
    // margin), without the full mass properties, which were most of a large
    // part's import time. A reading that fails falls back to the exact
    // integral, as the orientation read always did.
    let _caller = crate::mass_caller("step_import.face_sense_flip");
    let signed = crate::solid_signed_volume_for_sign(&solid, 0.0).or_else(|_| crate::solid_signed_volume(&solid));
    if let Ok(volume) = signed {
        if volume < 0.0 {
            crate::orientation::flip_all_faces(&mut solid)?;
        }
    }
    lap("signed_volume", &mut lap_started, &mut laps);
    // The closure gate, after validation and the orientation flip: a body
    // whose shell's trims do not enclose zero vector area is refused, typed
    // (`builder/closure.rs`); the capture lane holds the refusal and keeps the
    // body so the residual can be attributed edge by edge.
    let closure = closure_context.certify_shell_closure(&solid, shell_ref, capture)?;
    lap("closure", &mut lap_started, &mut laps);
    if profile {
        let line: Vec<String> = laps.iter().map(|(name, ms)| format!("{name}={ms:.2}")).collect();
        profile_line!("step_import.profile shell=#{shell_ref} faces={} {}", face_refs.len(), line.join(" "));
    }
    Ok((solid, captured, readings::BodyReadings { bounded, closure }))
}

/// Debug escape hatch shared by BOTH validation gates: dump the invalid solid
/// for offline triage before refusing, because the validation issues reference
/// face/coedge ids that exist only on this in-flight solid — there is no other
/// way to look at what failed.
///
/// FIRST failure wins, meaning the first solid to reach EITHER gate in this
/// process. A multi-body file can reach them more than once, and last-wins
/// overwrote the dump that belonged to the failure the caller is shown — which
/// is exactly how a BREP_WITH_VOIDS pcurve failure came to be triaged against
/// an unrelated opportunistic shell's solid. The flag is a per-process
/// debugging lever, so a per-process latch is the right scope.
///
/// The scope is the PROCESS, so a batch tool importing hundreds of files with
/// the flag set dumps the first failure of the run, not one per file. That is
/// the single-file triage case this lever is for; a batch wanting all of them
/// should set the flag per child, or the dump should learn numbered paths.
///
/// "First to reach a gate" is not always "the reported `first_error`": an
/// OPPORTUNISTIC surface-model body that fails here is skipped silently
/// (`bodies.rs`) without setting `first_error`, so on such a file the dump can
/// belong to a body no message names. Dumping every failing body under
/// numbered paths would remove the ambiguity; nothing needs that yet.
fn dump_invalid_solid_for_triage(solid: &BrepSolid) {
    use std::sync::atomic::{AtomicBool, Ordering};
    static DUMPED: AtomicBool = AtomicBool::new(false);
    let Ok(path) = std::env::var("BREP_STEP_DUMP_INVALID") else {
        return;
    };
    if DUMPED.swap(true, Ordering::Relaxed) {
        return;
    }
    if let Ok(json) = serde_json::to_string(solid) {
        let _ = std::fs::write(&path, json);
    }
}

/// Validation gate shared by the assembly and flat import paths, with the
/// pinch repair folded in: the duplicate-vertex weld can join two umbrella
/// fans at one point (an odd Euler characteristic no local check sees);
/// splitting the fans restores the manifold before judging the solid.
fn finalize_imported_solid(mut solid: BrepSolid) -> Result<BrepSolid, StepBodyError> {
    // The split's refusal is a `KernelRefusal`; until 2026-10-03 the `?` here
    // lost its class to `String`. It rides typed now, and the body lanes
    // count it as a failed body exactly as they counted its text.
    if crate::split_pinched_vertices(&mut solid)? > 0 {
        solid.genus = euler_genus(&solid);
    }
    // Every curve of the body was read from the file (or projected onto a
    // read surface from a read edge curve): the strict edge/pcurve bar
    // judges only kernel-built pairs (`NurbsCurve::is_imported`). Parasolid
    // and IGES reach this through their STEP translation.
    for edge in &mut solid.edges {
        edge.curve.set_imported(true);
    }
    for coedge in solid.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()).flat_map(|face| face.loops.iter_mut()).flat_map(|lp| lp.coedges.iter_mut()) {
        coedge.pcurve.set_imported(true);
    }
    Ok(solid)
}

/// Each face on a carrier closed in neither direction whose largest loop
/// winds in its parameters against its sense (counter-clockwise for a face
/// that keeps its carrier's normal, clockwise for one that reverses it) has
/// its sense turned to the loop's. A loop too small to read a winding from
/// is left alone. Each pcurve is read at its knots and at even steps within
/// every knot span (cross review: a detour packed into a narrow span), and
/// the area taken about the loop's first point (cross review: products of
/// large parameters cancel). Returns how many faces turned.
pub(in crate::step_import) fn resense_faces_by_winding(solid: &mut crate::BrepSolid) -> usize {
    const STEPS: usize = 16;
    const SPAN_STEPS: usize = 8;
    let mut turned = 0;
    for face in solid.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        if !matches!(face.surface.closed_directions(), Ok((false, false))) {
            continue;
        }
        let (Ok([u0, u1]), Ok([v0, v1])) = (face.surface.domain_u(), face.surface.domain_v()) else { continue };
        let mut largest = 0.0f64;
        for lp in &face.loops {
            let mut uv: Vec<[f64; 2]> = Vec::new();
            for coedge in &lp.coedges {
                let Ok([t0, t1]) = coedge.pcurve.domain() else { continue };
                let mut ts: Vec<f64> = (0..STEPS).map(|k| t0 + (t1 - t0) * k as f64 / STEPS as f64).collect();
                for w in coedge.pcurve.knots.windows(2) {
                    let (a, b) = (w[0].max(t0), w[1].min(t1));
                    if b > a {
                        ts.extend((0..SPAN_STEPS).map(|k| a + (b - a) * k as f64 / SPAN_STEPS as f64));
                    }
                }
                ts.retain(|t| *t < t1);
                ts.sort_by(f64::total_cmp);
                ts.dedup();
                uv.extend(ts.into_iter().filter_map(|t| coedge.pcurve.evaluate(t).ok()).map(|p| [p.x, p.y]));
            }
            let n = uv.len();
            if n < 3 {
                continue;
            }
            let o = uv[0];
            let at = |i: usize| [uv[i % n][0] - o[0], uv[i % n][1] - o[1]];
            let area = 0.5 * (0..n).map(|i| at(i)[0] * at(i + 1)[1] - at(i + 1)[0] * at(i)[1]).sum::<f64>();
            if area.abs() > largest.abs() {
                largest = area;
            }
        }
        if largest.abs() > 1e-6 * (u1 - u0) * (v1 - v0) && (largest > 0.0) != face.same_sense {
            face.same_sense = !face.same_sense;
            turned += 1;
        }
    }
    turned
}
