use super::*;

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

pub(in crate::step_import) fn build_solid(resolver: &Resolver, manifold_ref: usize) -> Result<BrepSolid, String> {
    let debug = std::env::var("BREP_DEBUG_STEP_LOOP").is_ok();
    let started = debug.then(web_time::Instant::now);
    if debug {
        eprintln!("SOLID #{manifold_ref} BEGIN");
    }
    let result = build_solid_inner(resolver, manifold_ref, false).map(|(solid, _, _)| solid);
    if let Some(started) = started {
        eprintln!(
            "SOLID #{manifold_ref} END {:?} ok={}",
            started.elapsed(),
            result.is_ok()
        );
    }
    result
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
    builder.relocate_periodic_rim_seams(&pending)?;
    builder.split_seam_crossing_edges(&mut pending)?;
    // Phase 1.75: a planar face whose straight boundary sits off its plane
    // (mesh-derived region fits) becomes exactly planar triangles BEFORE the
    // reconcile step below can bend its edges onto one plane or the other.
    builder.split_off_plane_planar_faces(&mut pending)?;
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

fn certify_brep_with_voids_topology(solid: &BrepSolid) -> Result<(), String> {
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
            return Err(format!(
                "step_import: BREP_WITH_VOIDS shell {shell_index} shares an edge reference with another shell"
            ));
        }
        let mut shell_vertices = HashSet::default();
        for (edge_id, senses) in &uses {
            let edge = edges.get(edge_id).ok_or_else(|| {
                format!("step_import: BREP_WITH_VOIDS shell references missing edge {edge_id}")
            })?;
            if !edge.degenerate && (senses.len() != 2 || senses[0] == senses[1]) {
                return Err(format!(
                    "step_import: BREP_WITH_VOIDS shell {shell_index} edge {edge_id} has incidence {senses:?}"
                ));
            }
            shell_vertices.insert(edge.start_vertex_id);
            shell_vertices.insert(edge.end_vertex_id);
        }
        if shell_vertices
            .iter()
            .any(|vertex_id| claimed_vertices.contains(vertex_id))
        {
            return Err(format!(
                "step_import: BREP_WITH_VOIDS shell {shell_index} shares a vertex reference with another shell"
            ));
        }
        claimed_edges.extend(shell_edges);
        claimed_vertices.extend(shell_vertices);
    }
    Ok(())
}

pub(in crate::step_import) fn certify_brep_with_voids_containment(solid: &BrepSolid) -> Result<(), String> {
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

    let outer_ids = shell_vertex_ids(&solid.shells[0])?;
    let (outer_lo, outer_hi) = bounds(&outer_ids)?;
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
            let mut outward = u_dir.cross(*v_dir).normalized()?;
            if !face.same_sense {
                outward = outward.scale(-1.0);
            }
            for vertex_id in &outer_ids {
                let signed_distance = vertex_map[vertex_id].sub(*origin).dot(outward);
                if signed_distance > tolerance {
                    return Err(format!(
                        "step_import: BREP_WITH_VOIDS outer shell is not convex at face {} vertex {vertex_id} (outside halfspace by {signed_distance})",
                        face.id
                    ));
                }
            }
        }
    }
    let outer = BrepSolid {
        mass_properties_cache: Default::default(),
        id: solid.id,
        vertices: solid.vertices.clone(),
        edges: solid.edges.clone(),
        shells: vec![solid.shells[0].clone()],
        genus: 0,
    };
    // Build the outer-shell point classifier ONCE and reuse it for every void
    // vertex/edge sample. `classify_point` rebuilds this per call, which for a
    // curved void (many samples) is O(samples x outer-faces) and can blow the
    // import timeout; a single build gives byte-identical results far faster.
    let outer_classifier = crate::SolidClassifier::new(&outer, tolerance)?;
    let mut void_bounds = Vec::new();
    for (shell_index, shell) in solid.shells.iter().enumerate().skip(1) {
        let ids = shell_vertex_ids(shell)?;
        let (lo, hi) = bounds(&ids)?;
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
                return Err(format!(
                    "step_import: BREP_WITH_VOIDS void {shell_index} is not strictly inside the outer bounds"
                ));
            }
        }
        for vertex_id in &ids {
            let point = vertex_map[vertex_id];
            let class = outer_classifier.classify(point)?.class;
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
                return Err(format!(
                    "step_import: BREP_WITH_VOIDS void {shell_index} vertex {vertex_id} at {point:?} is {class:?} relative to outer shell (classifier tolerance {tolerance}, nearest carrier projection {projection_distance}, bbox clearances {bbox_clearances:?})"
                ));
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
                            if outer_classifier.classify(point)?.class == crate::PointClass::Out {
                                return Err(format!(
                                    "step_import: BREP_WITH_VOIDS void {shell_index} edge {} bows outside the outer shell",
                                    coedge.edge_id
                                ));
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
                    return Err(format!(
                        "step_import: BREP_WITH_VOIDS void bounds {} and {} overlap",
                        first + 1,
                        second + 1
                    ));
                }
            }
        }
    }
    Ok(())
}

pub(in crate::step_import) fn build_brep_with_voids(resolver: &Resolver, body_ref: usize) -> Result<BrepSolid, String> {
    build_brep_with_voids_captured(resolver, body_ref, false).map(|(solid, _, _)| solid)
}

/// [`build_brep_with_voids`], optionally carrying the per-edge capture.
pub(in crate::step_import) fn build_brep_with_voids_captured(
    resolver: &Resolver,
    body_ref: usize,
    capture: bool,
) -> Result<(BrepSolid, Option<readings::TrimCapture>, Vec<readings::BoundedTrim>), String> {
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
        return Err("step_import: BREP_WITH_VOIDS has no void shells".into());
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
            return Err("step_import: ORIENTED_CLOSED_SHELL orientation is not .T./.F.".into());
        };
        if !shell_refs.insert(base_ref) {
            return Err(format!(
                "step_import: BREP_WITH_VOIDS repeats CLOSED_SHELL #{base_ref}"
            ));
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
                return Err(format!(
                    "step_import: BREP_WITH_VOIDS face #{face_ref} belongs to multiple shells"
                ));
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
    };
    let mut shells = Vec::with_capacity(face_lists.len());
    for ((_, same_sense), faces) in shell_specs.iter().zip(&face_lists) {
        let shell_faces = build_shell_faces_in_namespace(&mut builder, faces)?;
        let mut shell = ShellRecord {
            id: builder.fresh(),
            faces: shell_faces,
        };
        if !same_sense {
            crate::offset_shell::flip_shell_faces(&mut shell)?;
        }
        shells.push(shell);
    }
    let solid_id = builder.fresh();
    let captured = builder.readings.take();
    let bounded = std::mem::take(&mut builder.bounded);
    let mut solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: solid_id,
        vertices: builder.vertices,
        edges: builder.edges,
        shells,
        genus: 0,
    };
    solid = finalize_imported_solid(solid)?;
    solid.genus = euler_genus(&solid);
    certify_brep_with_voids_topology(&solid)?;
    let issues = solid.validate();
    if !issues.is_empty() {
        dump_invalid_solid_for_triage(&solid);
        return Err(format!(
            "step_import: reconstructed BREP_WITH_VOIDS failed validation: {issues:?}"
        ));
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
        return Err(format!(
            "step_import: BREP_WITH_VOIDS shell volume is below tolerance {tolerance}: {shell_volumes:?}"
        ));
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
        crate::offset_shell::flip_all_faces(&mut solid)?;
        for volume in &mut shell_volumes {
            *volume = -*volume;
        }
    } else if !authored_roles {
        return Err(format!(
            "step_import: BREP_WITH_VOIDS has mixed outer/void orientations {shell_volumes:?}"
        ));
    }
    certify_brep_with_voids_containment(&solid)?;
    let outer_volume = shell_volumes[0];
    let net_volume = shell_volumes.iter().sum::<f64>();
    if outer_volume <= tolerance || !net_volume.is_finite() || net_volume <= tolerance {
        return Err(format!(
            "step_import: BREP_WITH_VOIDS invalid outer/net volume {outer_volume}/{net_volume} (tolerance {tolerance})"
        ));
    }
    solid.genus = euler_genus(&solid);
    Ok((solid, captured, bounded))
}

pub(in crate::step_import) fn build_solid_inner(
    resolver: &Resolver,
    manifold_ref: usize,
    capture: bool,
) -> Result<(BrepSolid, Option<readings::TrimCapture>, Vec<readings::BoundedTrim>), String> {
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

/// Reconstruct one `BrepSolid` from a single CLOSED_SHELL (or OPEN_SHELL)
/// reference — the shared core behind MANIFOLD_SOLID_BREP / FACETED_BREP import
/// and behind each shell of a SHELL_BASED_SURFACE_MODEL. A genuinely open shell
/// reaches the same validation gate and is rejected there, so this stays safe to
/// call on any shell reference.
pub(in crate::step_import) fn build_solid_from_shell(resolver: &Resolver, shell_ref: usize) -> Result<BrepSolid, String> {
    build_solid_from_shell_captured(resolver, shell_ref, false).map(|(solid, _, _)| solid)
}

/// [`build_solid_from_shell`], optionally carrying what the importer did to
/// every edge (`capture`), for [`crate::import_step_trim_readings`].
pub(in crate::step_import) fn build_solid_from_shell_captured(
    resolver: &Resolver,
    shell_ref: usize,
    capture: bool,
) -> Result<(BrepSolid, Option<readings::TrimCapture>, Vec<readings::BoundedTrim>), String> {
    let shell_entity = resolver.get(shell_ref)?;
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
    };

    // Phase 1: resolve every face's surface + coedge bounds (this also builds
    // and caches all shared edges/vertices). Phase 1.5: re-seat OCC
    // full-cylinder rim circles onto the surface seam meridian — a mutation of
    // shared edges/vertices that must precede any pcurve derivation. Phase 2:
    // assemble each face's loops/pcurves from the (now consistent) topology.
    // Phase wall-clock on stderr when `BREP_PROFILE` is set — the switch the
    // boolean's `boolean.<stage>_ms` lines use.
    let profile = std::env::var("BREP_PROFILE").is_ok();
    let mut lap_started = web_time::Instant::now();
    let mut laps: Vec<(&str, f64)> = Vec::new();
    let mut lap = |name: &'static str, started: &mut web_time::Instant, laps: &mut Vec<(&str, f64)>| {
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
    builder.relocate_periodic_rim_seams(&pending)?;
    builder.split_seam_crossing_edges(&mut pending)?;
    lap("seams", &mut lap_started, &mut laps);
    // Phase 1.75: a planar face whose straight boundary sits off its plane
    // (mesh-derived region fits) becomes exactly planar triangles BEFORE the
    // reconcile step below can bend its edges onto one plane or the other.
    builder.split_off_plane_planar_faces(&mut pending)?;
    lap("planar_split", &mut lap_started, &mut laps);
    builder.reconcile_edges_onto_surfaces(&pending)?;
    lap("reconcile", &mut lap_started, &mut laps);
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
    lap("finish_faces", &mut lap_started, &mut laps);

    builder.report_supplied_pcurves();
    let shell_id = builder.fresh();
    let solid_id = builder.fresh();
    let captured = builder.readings.take();
    let bounded = std::mem::take(&mut builder.bounded);
    let mut solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: solid_id,
        vertices: builder.vertices,
        edges: builder.edges,
        shells: vec![ShellRecord {
            id: shell_id,
            faces,
        }],
        genus: 0,
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
        return Err(format!(
            "step_import: reconstructed solid failed validation: {:?}",
            issues
        ));
    }
    // Vendor face senses can leave the whole shell pointing inward; a closed
    // shell's material side is defined by its signed volume, so restore the
    // outward convention.
    let _caller = crate::mass_caller("step_import.face_sense_flip");
    if let Ok(volume) = crate::solid_signed_volume(&solid) {
        if volume < 0.0 {
            crate::offset_shell::flip_all_faces(&mut solid)?;
        }
    }
    lap("signed_volume", &mut lap_started, &mut laps);
    if profile {
        let line: Vec<String> = laps.iter().map(|(name, ms)| format!("{name}={ms:.2}")).collect();
        eprintln!("step_import.profile shell=#{shell_ref} faces={} {}", face_refs.len(), line.join(" "));
    }
    Ok((solid, captured, bounded))
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
fn finalize_imported_solid(mut solid: BrepSolid) -> Result<BrepSolid, String> {
    if crate::split_pinched_vertices(&mut solid)? > 0 {
        solid.genus = euler_genus(&solid);
    }
    Ok(solid)
}
