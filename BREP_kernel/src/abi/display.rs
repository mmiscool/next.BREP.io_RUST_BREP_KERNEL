use super::*;

/// Everything the native renderer needs to display one resident solid.
pub struct DisplaySolidPayload {
    /// Faces in shell/face order — the index into this Vec IS the watertight
    /// mesh's `face_ids` value for that face's triangles.
    pub faces: Vec<(u64, Option<String>)>,
    /// Watertight display mesh (chord-tolerance driven).
    pub mesh: Mesh,
    /// Non-degenerate edges `(edge_id, name, polyline)`, sorted by edge id.
    pub edges: Vec<(u64, Option<String>, Vec<Vec3>)>,
    /// Topology vertices `(vertex_id, point)`.
    pub vertices: Vec<(u64, Vec3)>,
    /// The chord tolerance actually used.
    pub chord_tolerance: f64,
}

/// The app's display chord tolerance (`BetterSolid._kernelTessellationOptions`):
/// vertex |coord| extent — falling back to the control-point hull / sqrt(2) for
/// vertex-free solids (full spheres/tori) — times 1.5e-3, times the render-LOD
/// factor (1.0 = the app's "Normal" preset).
pub fn display_chord_tolerance(solid: &BrepSolid, lod_factor: f64) -> f64 {
    let mut extent = 0.0f64;
    for vertex in &solid.vertices {
        extent = extent
            .max(vertex.point.x.abs())
            .max(vertex.point.y.abs())
            .max(vertex.point.z.abs());
    }
    if extent <= 0.0 {
        let mut control_point_extent = 0.0f64;
        for shell in &solid.shells {
            for face in &shell.faces {
                for row in &face.surface.control_points {
                    for cp in row {
                        let w = if cp.w != 0.0 { cp.w } else { 1.0 };
                        control_point_extent = control_point_extent
                            .max((cp.x / w).abs())
                            .max((cp.y / w).abs())
                            .max((cp.z / w).abs());
                    }
                }
            }
        }
        extent = control_point_extent / std::f64::consts::SQRT_2;
    }
    extent.max(1e-9) * 1.5e-3 * lod_factor
}

/// Native display payload for a resident solid: watertight mesh + face list (in
/// mesh `face_ids` order) + edge polylines + vertices, all in one registry
/// borrow. `lod_factor` scales the per-solid display chord tolerance (1.0 = the
/// app's "Normal" preset; higher = coarser mesh). Callers pass a sanitized,
/// finite, positive value — [`display_chord_tolerance`] multiplies it in directly.
pub fn display_payload_handle_native(
    handle: u32,
    lod_factor: f64,
) -> Result<DisplaySolidPayload, String> {
    with_registered_solid_str(handle, |solid| {
        let chord = display_chord_tolerance(solid, lod_factor);
        let mesh = tessellate_brep_watertight(solid, chord)?;
        let mut faces = Vec::new();
        for shell in &solid.shells {
            for face in &shell.faces {
                faces.push((face.id, face.name.clone()));
            }
        }
        let edge_names: std::collections::HashMap<u64, String> = solid
            .edges
            .iter()
            .filter_map(|edge| edge.name.as_ref().map(|name| (edge.id, name.clone())))
            .collect();
        let edges = sample_edge_polylines(solid, chord)?
            .into_iter()
            .map(|(id, points)| (id, edge_names.get(&id).cloned(), points))
            .collect();
        let vertices = solid
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect();
        Ok(DisplaySolidPayload {
            faces,
            mesh,
            edges,
            vertices,
            chord_tolerance: chord,
        })
    })
}

/// Native display payload for a solved sketch PROFILE — the SHEET-SOLID view of a
/// committed sketch: a planar FACE mesh + its named boundary EDGES + corner
/// VERTICES, so a sketch is pickable / selectable / measurable through the exact
/// same display path as a real solid. NO solid is registered — the payload is
/// synthesized directly from the profile, so it owns no scene handle (the display
/// carries `source_handle = 0`).
///
/// The FACE triangulates each region (outer boundary minus holes) via the
/// watertight planar path, mapped to world through the profile's frame; each
/// triangle rides face id 0 (one logical sheet face). EDGES are each boundary
/// curve sampled to a world polyline, named from the profile's `{sketchId}:G{gid}`
/// edge names; VERTICES are the loop corners. An empty profile (no closed region)
/// yields an empty payload (no face, no edges) — an open/underconstrained sketch
/// has no sheet.
pub fn sketch_profile_display_payload(
    profile: &crate::feature_pipeline::SketchProfile,
) -> DisplaySolidPayload {
    /// Samples per boundary curve — enough to resolve arcs/circles; straight
    /// segments oversample harmlessly.
    const SEGMENTS: usize = 24;
    let origin = profile.origin;
    let x_axis = profile.x_axis;
    let y_axis = profile.y_axis;
    let normal = profile.z_axis;
    let to_uv = |p: Vec3| {
        let d = p.sub(origin);
        [d.dot(x_axis), d.dot(y_axis)]
    };

    let mut mesh = Mesh::default();
    let mut edges: Vec<(u64, Option<String>, Vec<Vec3>)> = Vec::new();
    let mut vertices: Vec<(u64, Vec3)> = Vec::new();
    let mut next_edge_id: u64 = 0;
    let mut next_vertex_id: u64 = 0;
    let mut sketch_id: Option<String> = None;

    for region in &profile.regions {
        let Some((outer, holes)) = region.split_first() else {
            continue;
        };
        // Boundary vertices (uv, world) for triangulation: sample [t0, t1) per
        // curve so the next curve contributes the shared corner exactly once.
        let boundary = |lp: &crate::feature_pipeline::ProfileLoop| -> Vec<([f64; 2], Vec3)> {
            let mut out = Vec::new();
            for curve in &lp.curves {
                let Ok([t0, t1]) = curve.domain() else { continue };
                for step in 0..SEGMENTS {
                    let t = t0 + (t1 - t0) * step as f64 / SEGMENTS as f64;
                    if let Ok(p) = curve.evaluate(t) {
                        out.push((to_uv(p), p));
                    }
                }
            }
            out
        };
        let outer_b = boundary(outer);
        let holes_b: Vec<Vec<([f64; 2], Vec3)>> = holes.iter().map(boundary).collect();
        for [a, b, c] in watertight_tessellation::triangulate_planar_region(&outer_b, &holes_b) {
            let base = (mesh.positions.len() / 3) as u32;
            for p in [a, b, c] {
                mesh.positions.extend([p.x, p.y, p.z]);
                mesh.normals.extend([normal.x, normal.y, normal.z]);
            }
            mesh.indices.extend([base, base + 1, base + 2]);
            mesh.face_ids.push(0);
        }

        // Named boundary EDGES + corner VERTICES for every loop of this region.
        for lp in region {
            for (index, curve) in lp.curves.iter().enumerate() {
                let Ok([t0, t1]) = curve.domain() else { continue };
                let mut polyline = Vec::with_capacity(SEGMENTS + 1);
                for step in 0..=SEGMENTS {
                    let t = t0 + (t1 - t0) * step as f64 / SEGMENTS as f64;
                    if let Ok(p) = curve.evaluate(t) {
                        polyline.push(p);
                    }
                }
                let name = lp.edge_names.get(index).cloned().flatten();
                if sketch_id.is_none() {
                    // `{sketchId}:G{gid}` → the sheet's face inherits `{sketchId}`.
                    sketch_id = name
                        .as_deref()
                        .and_then(|n| n.split_once(":G").map(|(id, _)| id.to_string()));
                }
                if polyline.len() >= 2 {
                    edges.push((next_edge_id, name, polyline));
                    next_edge_id += 1;
                }
                if let Ok(p) = curve.evaluate(t0) {
                    vertices.push((next_vertex_id, p));
                    next_vertex_id += 1;
                }
            }
        }
    }

    // One logical sheet face (id 0), only if the mesh has triangles.
    let faces = if mesh.face_ids.is_empty() {
        Vec::new()
    } else {
        let face_name = sketch_id.map(|id| format!("{id}:FACE"));
        vec![(0u64, face_name)]
    };
    DisplaySolidPayload {
        faces,
        mesh,
        edges,
        vertices,
        chord_tolerance: 0.0,
    }
}

/// Native display payload for a whole committed SKETCH: its profile SHEET (when
/// the sketch closes a region) PLUS every model SEGMENT the sheet does not
/// already draw.
///
/// [`sketch_profile_display_payload`] draws a sketch through its closed profile,
/// and that is the only display a committed sketch had. A sketch that closes
/// NOTHING — one line, an open chain, a trajectory drawn alongside a closed loop —
/// publishes no profile at all, so it was handed to the sheet builder as `None`
/// and drew nothing: invisible in 3D, and unpickable there (the 2026-09-02 report,
/// *"Sketch with single edge not visible in 3D"*). Its geometry was never missing,
/// only unaccepted — the SKETCH feature already publishes one world curve per
/// model segment under `{sketchId}:G{gid}`, which is exactly what `segments`
/// carries here.
///
/// Each such segment draws as one NAMED edge (the same name a downstream
/// `reference_selection` stores, so picking the drawn line resolves) with its
/// endpoints as vertices. A segment whose name the sheet already drew is SKIPPED,
/// so a closed sketch draws each boundary edge exactly once and a mixed sketch
/// draws its loop and its open chain side by side. Endpoints are deduped against
/// the points already drawn — consecutive segments of one chain share a junction.
///
/// `points` are the sketch's standalone MODEL points (world space): a sketch
/// with points only — a hole-placement sketch — has no segment at all, so it
/// drew nothing and was invisible and unpickable in 3D. Each point draws as a
/// vertex, deduped against the endpoints the segments already drew, so a point
/// that is also a segment corner draws once.
///
/// Like its sheet sibling this registers no solid: the payload is synthesized
/// directly from the profile, the curves and the points.
pub fn sketch_display_payload(
    profile: Option<&crate::feature_pipeline::SketchProfile>,
    segments: &[(String, Vec<NurbsCurve>)],
    points: &[Vec3],
) -> DisplaySolidPayload {
    /// Samples per segment for a simple curve — the sheet builder's own
    /// resolution. A curve with many knot spans (a fitted helix has ~64 per
    /// turn) gets four samples per span instead, so a long fitted curve draws
    /// as itself rather than as a coarse polygon; capped so a pathological
    /// knot vector cannot flood the display.
    const SEGMENTS: usize = 24;
    const SAMPLES_PER_SPAN: usize = 4;
    const MAX_SEGMENTS: usize = 4096;
    /// Two drawn endpoints closer than this are the same corner (a chain's
    /// junction points are the SAME solved sketch point, so they agree exactly).
    const SAME_POINT: f64 = 1e-9;

    let mut payload = match profile {
        Some(profile) => sketch_profile_display_payload(profile),
        None => DisplaySolidPayload {
            faces: Vec::new(),
            mesh: Mesh::default(),
            edges: Vec::new(),
            vertices: Vec::new(),
            chord_tolerance: 0.0,
        },
    };

    let drawn: std::collections::HashSet<String> = payload
        .edges
        .iter()
        .filter_map(|(_, name, _)| name.clone())
        .collect();
    let mut next_edge_id = payload
        .edges
        .iter()
        .map(|(id, _, _)| id + 1)
        .max()
        .unwrap_or(0);
    let mut next_vertex_id = payload.vertices.iter().map(|(id, _)| id + 1).max().unwrap_or(0);

    for (name, curves) in segments {
        if drawn.contains(name) {
            continue;
        }
        for curve in curves {
            let Ok([t0, t1]) = curve.domain() else { continue };
            let spans = curve
                .knots
                .windows(2)
                .filter(|pair| pair[1] > pair[0])
                .count();
            let segments = (spans * SAMPLES_PER_SPAN).clamp(SEGMENTS, MAX_SEGMENTS);
            let mut polyline = Vec::with_capacity(segments + 1);
            for step in 0..=segments {
                let t = t0 + (t1 - t0) * step as f64 / segments as f64;
                if let Ok(point) = curve.evaluate(t) {
                    polyline.push(point);
                }
            }
            if polyline.len() < 2 {
                continue;
            }
            for end in [polyline[0], polyline[polyline.len() - 1]] {
                if payload
                    .vertices
                    .iter()
                    .any(|(_, point)| point.sub(end).length() <= SAME_POINT)
                {
                    continue;
                }
                payload.vertices.push((next_vertex_id, end));
                next_vertex_id += 1;
            }
            payload.edges.push((next_edge_id, Some(name.clone()), polyline));
            next_edge_id += 1;
        }
    }

    // Standalone points: one vertex each, skipping any position a segment
    // endpoint (or an earlier point) already drew.
    for &point in points {
        if payload
            .vertices
            .iter()
            .any(|(_, drawn)| drawn.sub(point).length() <= SAME_POINT)
        {
            continue;
        }
        payload.vertices.push((next_vertex_id, point));
        next_vertex_id += 1;
    }
    payload
}

/// Full mass properties of a resident solid, scaled to `density` (the native,
/// non-wasm sibling of [`mass_properties_handle`] for the in-process renderer):
/// volume + surface area + centroid + centroidal inertia tensor + principal
/// axes/moments (Golovanov §8.11). The underlying geometry is unit-density; here
/// `mass = density * volume` and every inertia quantity scales linearly with
/// density (centroid + principal axes are density-independent). `density` is in
/// mass units per mm³ (the kernel's length convention is millimetres); pass
/// `1.0` for the raw geometric result (`mass == volume`). Reads the solid in one
/// registry borrow; the topology never crosses a boundary.
pub fn mass_properties_handle_native(
    handle: u32,
    density: f64,
) -> Result<DensityMassProperties, String> {
    with_registered_solid_str(handle, |solid| {
        Ok(solid_mass_properties_full(solid)?.with_density(density))
    })
}

/// Surface area and volume of a resident solid — the native sibling of
/// [`mass_properties_handle_native`] for a caller that reads NOTHING but
/// those two numbers (`examples/case_replay.rs` records volume and area per
/// solid and discards the rest). The full entry runs a second station sweep
/// over every trimmed face for the nine moment integrands, which costs as
/// much again as the area/volume sweep: on `anotherBooleanFail`'s imported
/// 32-face body 2.6 s against 1.3 s, and 5.3 s of that case's 12.9 s replay
/// went to moments nobody read. The two numbers come out of the same
/// `solid_mass_properties` call the full entry starts with, so they are
/// bit-identical to its `volume` and `surface_area`; the full entry's
/// refusal of a non-positive volume is kept so a caller switching over sees
/// the same `Err` on the same solids. Reads the solid in one registry borrow.
pub fn mass_measures_handle_native(handle: u32) -> Result<MassProperties, String> {
    with_registered_solid_str(handle, |solid| {
        let _caller = crate::mass_caller("abi.mass_measures");
        let properties = solid_mass_properties(solid)?;
        if properties.volume.abs() <= 1e-30 {
            return Err("solid_mass_properties_full: non-positive volume".into());
        }
        Ok(properties)
    })
}

/// Topology validation issues of a resident solid as `(severity, message)`
/// pairs — the native sibling of the JSON validators, for in-process
/// qualification tooling (`examples/case_replay.rs`). An empty Vec means the
/// incidence checks passed; that is NOT a correctness proof: `validate()` tests
/// neither connectivity nor face-vs-face self-intersection, and a validating
/// solid can still be the wrong solid. Reads the solid in one registry borrow.
pub fn validate_handle_native(handle: u32) -> Result<Vec<(String, String)>, String> {
    with_registered_solid_str(handle, |solid| {
        Ok(solid
            .validate()
            .into_iter()
            .map(|issue| (issue.severity.to_string(), issue.message))
            .collect())
    })
}

/// Topology TOTALS of a resident solid as
/// `(faces, edges, non_degenerate_edges, vertices)`, read straight off the
/// record: every face of every shell, every edge, every edge that is not a
/// collapsed pole boundary, every vertex.
///
/// The two edge counts answer different questions and both are worth
/// recording. The RAW total is what a reader means by "15 edges" when looking
/// at the record, and it is the right thing to diff between two runs. The
/// NON-DEGENERATE count is the one a derivation can state from the intended
/// construction, because it excludes the seam and pole boundaries the kernel
/// chose as scaffolding rather than anything the shape has.
///
/// These are what a case gate compares. A solid can keep its volume to the
/// last digit while gaining or losing edges — `inbox-20260909-tube-joint-edge-splits`
/// carried "7 faces / 17 edges / 11 vertices" in prose for a month with every
/// local assertion in its suite passing — so the totals are recorded and
/// diffed.
///
/// Since 2026-09-20 they can also be STATED. The two checks answer different
/// questions and both are kept — a diff against the baseline says "did this
/// change", a claim says "was it ever right" — and it is the NON-degenerate
/// count that a claim may state, for the reason above. Reads the solid in one
/// registry borrow.
pub fn topology_counts_native(handle: u32) -> Result<(usize, usize, usize, usize), String> {
    with_registered_solid_str(handle, |solid| {
        Ok((
            solid.shells.iter().map(|shell| shell.faces.len()).sum(),
            solid.edges.len(),
            solid.edges.iter().filter(|edge| !edge.degenerate).count(),
            solid.vertices.len(),
        ))
    })
}

/// Connectivity of a resident solid — the check `validate()` does NOT make.
/// Per-shell edge-connected face components, edges shared across shells, and
/// pinch vertices. Pure topology, no geometry, no tolerance; see
/// [`crate::solid_connectivity`] for which part is a gate and which is a
/// diagnostic. Reads the solid in one registry borrow.
pub fn connectivity_handle_native(handle: u32) -> Result<crate::ConnectivityReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::solid_connectivity(solid)))
}

/// Closedness and Euler parity of a resident solid — the THIRD check
/// `validate()` does not make on its own. Its Euler comparison reduces, on a
/// freshly derived solid, to exactly "is the characteristic even", and it is
/// switched off on any solid carrying a degenerate edge; this asks the question
/// directly and on every solid. Pure counting, one registry borrow.
pub fn euler_handle_native(handle: u32) -> Result<crate::EulerReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::solid_euler(solid)))
}

/// Face-loop self-crossing of a resident solid — the check no OTHER detector
/// here can make. `validate()` is silent (the incidence is perfect), the
/// face-versus-face scan has no pair to confirm (both triangles are on one
/// face), connectivity and Euler parity count the same cells either way; the
/// face's AREA is the only other tell. Pure parameter-space work on the trims
/// the face already carries — no tessellation — so it is cheap enough to be a
/// default. Reads the solid in one registry borrow.
pub fn loop_crossings_handle_native(handle: u32) -> Result<crate::LoopCrossingReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::loop_self_crossings(solid)))
}

/// The vector-area closure of a resident solid: does each closed shell's
/// TRIMS enclose the zero vector area a closed shell owes, and if not, which
/// faces own the residual. One pass over the trims and the edge curves — no
/// tessellation, no projection — so unlike the crossing scan it is cheap
/// enough for a default gate. Reads the solid in one registry borrow.
pub fn vector_area_handle_native(handle: u32) -> Result<crate::VectorAreaReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::shell_vector_areas(solid)))
}

/// Face-versus-face self-intersection of a resident solid — the OTHER check
/// `validate()` does not make (its one "self-intersects" string is a uv-wire
/// warning). Opt-in and expensive: it tessellates the solid and walks a
/// triangle BVH, so no default gate calls it. `chord_tolerance` overrides the
/// per-solid display density when positive. Reads the solid in one registry
/// borrow.
pub fn self_intersections_handle_native(
    handle: u32,
    chord_tolerance: f64,
) -> Result<crate::SelfIntersectionReport, String> {
    with_registered_solid_str(handle, |solid| {
        let mut options = crate::SelfIntersectionOptions::for_solid(solid);
        if chord_tolerance > 0.0 && chord_tolerance.is_finite() {
            options.chord_tolerance = chord_tolerance;
        }
        crate::solid_self_intersections(solid, options)
    })
}

/// The Transform feature's BBOX_CENTER pivot for a resident source solid.
/// Share the kernel's vertex-bbox definition with viewport controls; display
/// tessellation bounds would give a different pivot for curved solids.
pub fn transform_pivot_native(handle: u32) -> Result<[f64; 3], String> {
    with_registered_solid_str(handle, |solid| {
        Ok(crate::feature_pipeline::transform_bbox_center(solid))
    })
}

/// Total 3D arc length (mm) of every non-degenerate edge of a resident solid —
/// the Properties panel's "total edge length" measurement. Native sibling of the
/// mass-properties accessors; one short registry borrow, topology never crosses
/// the boundary.
pub fn solid_edge_length_total_native(handle: u32) -> Result<f64, String> {
    with_registered_solid_str(handle, solid_edge_length_total)
}

/// `(area, boundary_edge_total_length, surface_type)` of a resident solid's
/// named face (mm² / mm / classification): the face's surface area, the summed
/// arc length of its boundary edges, and a short label for its underlying
/// carrier surface (`"Plane"`, `"Cylinder"`, `"Cone"`, `"Sphere"`, `"Torus"`,
/// `"Surface of revolution"`, or `"NURBS"` when the exact rational patch is not
/// a recognized analytic carrier). Errs if no face carries `face_name`.
pub fn face_measurements_native(
    handle: u32,
    face_name: &str,
) -> Result<(f64, f64, &'static str), String> {
    with_registered_solid_str(handle, |solid| {
        let face = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.name.as_deref() == Some(face_name))
            .ok_or_else(|| format!("face '{face_name}' not found"))?;
        let surface_type = face
            .surface
            .analytic()
            .map(|analytic| analytic.kind_label())
            .unwrap_or("NURBS");
        Ok((
            face_area(face)?,
            face_boundary_length(solid, face)?,
            surface_type,
        ))
    })
}

/// 3D arc length (mm) of a resident solid's named edge. Errs if no edge carries
/// `edge_name`.
pub fn edge_length_native(handle: u32, edge_name: &str) -> Result<f64, String> {
    with_registered_solid_str(handle, |solid| {
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.name.as_deref() == Some(edge_name))
            .ok_or_else(|| format!("edge '{edge_name}' not found"))?;
        edge_arc_length(edge)
    })
}

/// Escape hatch: pull a resident solid's full topology across the boundary (for
/// STEP export or JSON-only lanes during the migration). Prefer handle-native
/// ops — this re-incurs the serialization cost the registry exists to avoid.
#[wasm_bindgen]
pub fn solid_handle_to_buffer(handle: u32) -> Result<WasmSolidBuffer, JsValue> {
    with_registered_solid(handle, |solid| solid_buffer(solid, "{}".into()))
}

/// Native: export the CURRENT resident solids (by handle) to an ISO-10303-21
/// STEP document. Reads each resident solid out of the thread-local registry —
/// the SAME registry [`display_payload_handle_native`] reads — clones them into
/// a `Vec<BrepSolid>`, and hands the batch to [`export_step`]. The engine-native
/// app's Export→STEP lane calls this with the handles the pipeline left resident
/// after the last history run, so the topology never crosses a boundary as JSON.
/// `String` error (JsValue-free — it links + runs on native and wasm alike).
pub fn export_step_handles(
    handles: &[u32],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    if handles.is_empty() {
        return Err("export_step_handles: no solids to export".into());
    }
    let mut solids = Vec::with_capacity(handles.len());
    for &handle in handles {
        let solid: BrepSolid = with_registered_solid_str(handle, |solid| Ok(solid.clone()))?;
        solids.push(solid);
    }
    export_step(&solids, name, unit, timestamp)
}

/// [`export_step_handles`] with each body's SCENE NAME and an optional PMI
/// block: the engine-native Export→STEP lane. Names key the
/// `MANIFOLD_SOLID_BREP`s and the PMI reference resolution.
pub fn export_step_named_handles(
    named: &[(String, u32)],
    name: &str,
    unit: &str,
    timestamp: &str,
    pmi: Option<&crate::StepPmi<'_>>,
    colors: Option<&crate::StepColors>,
) -> Result<crate::StepExportReport, String> {
    if named.is_empty() {
        return Err("export_step_named_handles: no solids to export".into());
    }
    let mut solids = Vec::with_capacity(named.len());
    for (solid_name, handle) in named {
        let solid: BrepSolid = with_registered_solid_str(*handle, |solid| Ok(solid.clone()))?;
        solids.push((solid_name.clone(), solid));
    }
    let borrowed: Vec<(String, &BrepSolid)> = solids
        .iter()
        .map(|(solid_name, solid)| (solid_name.clone(), solid))
        .collect();
    crate::export_step_report_named(&borrowed, name, unit, timestamp, pmi, colors)
}

/// STRUCTURED export: the resident scene PLUS the document's assembly model, as
/// an AP242 file with real product structure — one `PRODUCT` per parts-library
/// entry, one `NEXT_ASSEMBLY_USAGE_OCCURRENCE` per placed instance.
///
/// `named` is the whole resident scene (component-owned bodies included — the
/// tree builder filters them, because their geometry is written unposed in the
/// part's own product instead). `components` is one entry per ACOMP instance of
/// the ROOT document — `(component id, parts-library entry name, live pose)` —
/// which only the caller knows, since a solved or gizmo-moved instance's pose
/// lives in the app's projection of the scene.
pub fn export_step_assembly_handles(
    document_name: &str,
    named: &[(String, u32)],
    components: &[(String, String, crate::Mat4)],
    unit: &str,
    timestamp: &str,
    pmi: Option<&crate::StepPmi<'_>>,
    colors: Option<&crate::StepColors>,
) -> Result<crate::StepExportReport, String> {
    if named.is_empty() {
        return Err("export_step_assembly_handles: no solids to export".into());
    }
    let mut solids = Vec::with_capacity(named.len());
    for (solid_name, handle) in named {
        let solid: BrepSolid = with_registered_solid_str(*handle, |solid| Ok(solid.clone()))?;
        solids.push((solid_name.clone(), solid));
    }
    let assembly = crate::assembly_export_tree(document_name, solids, components)?;
    crate::export_step_assembly_report(&assembly, unit, timestamp, pmi, colors)
}

/// Native: export the CURRENT resident solids (by handle) to an IGES 5.3
/// document of trimmed NURBS surfaces. The IGES analogue of
/// [`export_step_handles`] — reads the resident solids out of the thread-local
/// registry, clones them into a `Vec<BrepSolid>`, and hands the batch to
/// [`export_iges`]. `String` error (JsValue-free — links on native and wasm).
pub fn export_iges_handles(
    handles: &[u32],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    if handles.is_empty() {
        return Err("export_iges_handles: no solids to export".into());
    }
    let mut solids = Vec::with_capacity(handles.len());
    for &handle in handles {
        let solid: BrepSolid = with_registered_solid_str(handle, |solid| Ok(solid.clone()))?;
        solids.push(solid);
    }
    export_iges(&solids, name, unit, timestamp)
}
