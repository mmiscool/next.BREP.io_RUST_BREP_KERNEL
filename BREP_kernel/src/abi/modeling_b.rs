use super::*;

#[derive(Deserialize)]
pub(crate) struct TangentLoftRequest {
    pub(crate) sections: Vec<Vec<NurbsCurve>>,
    pub(crate) start_direction: Vec3,
    pub(crate) end_direction: Vec3,
}

#[wasm_bindgen]
pub fn loft_profile_brep_closed_json(request_json: &str) -> Result<String, JsValue> {
    let request: LoftBrepRequest = json_in(request_json)?;
    let solid = loft_profile_brep_closed(&request.sections).map_err(javascript_refusal)?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn loft_profile_brep_closed_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: LoftBrepRequest = json_in(request_json)?;
    let solid = loft_profile_brep_closed(&request.sections).map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn loft_profile_brep_tangent_json(request_json: &str) -> Result<String, JsValue> {
    let request: TangentLoftRequest = json_in(request_json)?;
    let solid = loft_profile_brep_tangent(
        &request.sections,
        request.start_direction,
        request.end_direction,
    )
    .map_err(javascript_refusal)?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn loft_profile_brep_tangent_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: TangentLoftRequest = json_in(request_json)?;
    let solid = loft_profile_brep_tangent(
        &request.sections,
        request.start_direction,
        request.end_direction,
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn loft_profile_brep_guided_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: GuidedLoftRequest = json_in(request_json)?;
    let solid = guided_loft_dispatch(&request).map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn sweep_profile_along_path_json(request_json: &str) -> Result<String, JsValue> {
    let request: SweepRequest = json_in(request_json)?;
    let solid = sweep_profile_along_path(&request.profile, &request.path, request.name.as_deref())
        .map_err(javascript_refusal)?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn sweep_profile_along_path_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: SweepRequest = json_in(request_json)?;
    let solid = sweep_profile_along_path(&request.profile, &request.path, request.name.as_deref())
        .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[derive(Deserialize)]
pub(crate) struct TwistedSweepRequest {
    pub(crate) profile: Vec<NurbsCurve>,
    pub(crate) path: NurbsCurve,
    /// Total twist about the path tangent, radians, linear in arc length.
    pub(crate) twist_angle: f64,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

#[wasm_bindgen]
pub fn sweep_profile_twisted_json(request_json: &str) -> Result<String, JsValue> {
    let request: TwistedSweepRequest = json_in(request_json)?;
    let solid = sweep_profile_twisted(
        &request.profile,
        &request.path,
        request.twist_angle,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn sweep_profile_twisted_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: TwistedSweepRequest = json_in(request_json)?;
    let solid = sweep_profile_twisted(
        &request.profile,
        &request.path,
        request.twist_angle,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn sweep_profile_helix_json(request_json: &str) -> Result<String, JsValue> {
    let request: HelixSweepRequest = json_in(request_json)?;
    let solid = sweep_profile_helix(
        &request.profile,
        request.axis_origin,
        request.axis_direction,
        request.helix_radius,
        request.pitch,
        request.turns,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn sweep_profile_helix_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: HelixSweepRequest = json_in(request_json)?;
    let solid = sweep_profile_helix(
        &request.profile,
        request.axis_origin,
        request.axis_direction,
        request.helix_radius,
        request.pitch,
        request.turns,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn transform_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: TransformBrepRequest = json_in(request_json)?;
    let transform = AffineTransform::new(request.matrix).map_err(javascript_error)?;
    let solid = transform_brep(&request.solid, transform, request.reverse_orientation)
        .map_err(javascript_error)?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn mirror_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: MirrorBrepRequest = json_in(request_json)?;
    let solid = mirror_brep(&request.solid, request.plane_point, request.plane_normal)
        .map_err(javascript_error)?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn mirror_brep_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: MirrorBrepRequest = json_in(request_json)?;
    let solid = mirror_brep(&request.solid, request.plane_point, request.plane_normal)
        .map_err(javascript_error)?;
    solid_buffer(&solid, "{}".into())
}

/// Cut a solid by a plane (Golovanov §6.4). Returns `{ "below": <solid>,
/// "above": <solid> }` — the −n and +n pieces of the split. Errors with
/// "plane does not intersect the solid" when the plane misses the body.
#[wasm_bindgen]
pub fn split_solid_by_plane_json(request_json: &str) -> Result<String, JsValue> {
    let request: SplitByPlaneRequest = json_in(request_json)?;
    let (below, above) =
        split_solid_by_plane(&request.solid, request.plane_point, request.plane_normal)
            .map_err(javascript_error)?;
    let result = serde_json::json!({ "below": below, "above": above });
    json_out(&result)
}

/// Cut a solid by an analytic tool surface (Golovanov §6.4). The request is
/// `{ "solid": <solid>, "tool": { "type": "cylinder"|"sphere"|"cone"|"torus"|
/// "plane", ... } }`. Returns `{ "pieces": [<inside>, <outside>] }` (for a
/// plane, `[below, above]`). Errors with "tool surface does not divide the
/// solid" when the tool does not cleanly cut the body.
#[wasm_bindgen]
pub fn split_solid_by_surface_json(request_json: &str) -> Result<String, JsValue> {
    let request: SplitBySurfaceRequest = json_in(request_json)?;
    let pieces = split_solid_by_surface(&request.solid, &request.tool).map_err(javascript_error)?;
    let result = serde_json::json!({ "pieces": pieces });
    json_out(&result)
}

/// Cut a solid by the analytic carrier of a selected face's `surface` (Golovanov
/// §6.4). The face may be a plane, cylinder, cone, or sphere; the kernel
/// recognizes its analytic type and extends it to span the body. Returns
/// `{ "pieces": [<below/inside>, <above/outside>] }`. Errors on non-analytic
/// faces or when the carrier does not cleanly divide the body.
#[wasm_bindgen]
pub fn split_solid_by_face_surface_json(request_json: &str) -> Result<String, JsValue> {
    let request: SplitByFaceSurfaceRequest = json_in(request_json)?;
    let pieces =
        split_solid_by_face_surface(&request.solid, &request.surface).map_err(javascript_error)?;
    let result = serde_json::json!({ "pieces": pieces });
    json_out(&result)
}

#[wasm_bindgen]
pub fn solid_mass_properties_json(solid_json: &str) -> Result<String, JsValue> {
    let solid: BrepSolid = json_in(solid_json)?;
    let properties = solid_mass_properties(&solid).map_err(javascript_error)?;
    json_out(&properties)
}

#[wasm_bindgen]
pub fn solid_mass_properties_full_json(solid_json: &str) -> Result<String, JsValue> {
    let solid: BrepSolid = json_in(solid_json)?;
    let properties = solid_mass_properties_full(&solid).map_err(javascript_error)?;
    json_out(&properties)
}

#[wasm_bindgen]
pub fn face_metrics_json(face_json: &str) -> Result<String, JsValue> {
    let face: FaceRecord = json_in(face_json)?;
    let metrics = FaceMetrics {
        parameter_space_area: parameter_space_area(&face).map_err(javascript_error)?,
        area: face_area(&face).map_err(javascript_error)?,
        volume_contribution: face_volume_contribution(&face).map_err(javascript_error)?,
        trim_polygons: trim_polygons(&face).map_err(javascript_error)?,
    };
    json_out(&metrics)
}

#[wasm_bindgen]
pub fn tessellate_face_json(request_json: &str) -> Result<String, JsValue> {
    let request: TessellateFaceRequest = json_in(request_json)?;
    let mesh = tessellate_face(
        &request.face,
        TessellationOptions {
            slabs_per_span_u: request.slabs_per_span_u,
            steps_per_span_v: request.steps_per_span_v,
        },
        request.face.id as u32,
    )
    .map_err(javascript_error)?;
    json_out(&mesh)
}

#[wasm_bindgen]
pub fn tessellate_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: TessellateBrepRequest = json_in(request_json)?;
    let mesh = tessellate_brep(
        &request.solid,
        TessellationOptions {
            slabs_per_span_u: request.slabs_per_span_u,
            steps_per_span_v: request.steps_per_span_v,
        },
    )
    .map_err(javascript_error)?;
    json_out(&mesh)
}

#[wasm_bindgen]
pub fn classify_points_json(request_json: &str) -> Result<String, JsValue> {
    let request: ClassifyPointsRequest = json_in(request_json)?;
    let classifications = request
        .points
        .into_iter()
        .map(|point| classify_point(point, &request.solid, request.tolerance))
        .collect::<Result<Vec<_>, _>>()
        .map_err(javascript_error)?;
    json_out(&classifications)
}

#[wasm_bindgen]
pub fn arrange_segments_json(request_json: &str) -> Result<String, JsValue> {
    let request: ArrangeSegmentsRequest = json_in(request_json)?;
    let regions =
        arrange_segments(&request.segments, request.tolerance).map_err(javascript_error)?;
    json_out(&regions)
}

#[wasm_bindgen]
pub fn build_pcurve_json(request_json: &str) -> Result<String, JsValue> {
    let request: BuildPcurveRequest = json_in(request_json)?;
    let pcurve =
        build_pcurve_on_surface(&request.surface, &request.curve).map_err(javascript_error)?;
    json_out(&pcurve)
}

#[wasm_bindgen]
pub fn build_imprints_json(request_json: &str) -> Result<String, JsValue> {
    let request: BuildImprintsRequest = json_in(request_json)?;
    let result = build_imprints(&request.solid_a, &request.solid_b, &request.options)
        .map_err(javascript_refusal)?;
    json_out(&result)
}

#[wasm_bindgen]
pub fn apply_edge_splits_json(request_json: &str) -> Result<String, JsValue> {
    let request: ApplyEdgeSplitsRequest = json_in(request_json)?;
    let result = apply_edge_splits(&request.solid, request.operand, &request.imprint)
        .map_err(javascript_refusal)?;
    json_out(&result)
}

#[wasm_bindgen]
pub fn fragment_faces_json(request_json: &str) -> Result<String, JsValue> {
    let request: FragmentFacesRequest = json_in(request_json)?;
    let result = fragment_solid(&request.solid, request.operand, &request.imprint)
        .map_err(javascript_refusal)?;
    json_out(&result)
}

#[wasm_bindgen]
pub fn boolean_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: BooleanRequest = json_in(request_json)?;
    let result = boolean_operation(
        &request.first,
        &request.second,
        request.operation,
        &request.options,
    )
    .map_err(javascript_refusal)?;
    json_out(&result)
}

#[wasm_bindgen]
pub fn boolean_brep_diagnostics_json(request_json: &str) -> Result<String, JsValue> {
    let request: BooleanRequest = json_in(request_json)?;
    let result = boolean_operation_with_diagnostics(
        &request.first,
        &request.second,
        request.operation,
        &request.options,
    )
    .map_err(javascript_refusal)?;
    json_out(&result)
}

#[wasm_bindgen]
pub fn offset_surface_json(request_json: &str) -> Result<String, JsValue> {
    let request: OffsetSurfaceRequest = json_in(request_json)?;
    let surface = offset_surface(&request.face, request.distance, request.planar_extension)
        .map_err(javascript_refusal)?;
    json_out(&surface)
}

#[derive(Deserialize)]
pub(crate) struct SolveAssemblyRequest {
    pub(crate) bodies: Vec<AssemblyBody>,
    pub(crate) mates: Vec<AssemblyMate>,
    #[serde(default)]
    pub(crate) options: AssemblySolveOptions,
}

#[wasm_bindgen]
pub fn solve_assembly_json(request_json: &str) -> Result<String, JsValue> {
    let request: SolveAssemblyRequest = json_in(request_json)?;
    let solution = solve_assembly(&request.bodies, &request.mates, &request.options)
        .map_err(javascript_error)?;
    json_out(&solution)
}

#[derive(Deserialize)]
pub(crate) struct SegmentMeshRequest {
    /// Flat xyz triples.
    pub(crate) positions: Vec<f64>,
    /// Triangle indices; empty = raw soup (STL-style, 3 positions/triangle).
    #[serde(default)]
    pub(crate) indices: Vec<u32>,
    #[serde(default)]
    pub(crate) options: SegmentOptions,
}

#[wasm_bindgen]
pub fn segment_mesh_faces_json(request_json: &str) -> Result<String, JsValue> {
    let request: SegmentMeshRequest = json_in(request_json)?;
    let segmentation = segment_mesh_faces(&request.positions, &request.indices, &request.options)
        .map_err(javascript_error)?;
    json_out(&segmentation)
}

#[wasm_bindgen]
pub fn mesh_regions_to_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: SegmentMeshRequest = json_in(request_json)?;
    let solid = mesh_regions_to_brep(&request.positions, &request.indices, &request.options)
        .map_err(javascript_error)?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn mesh_regions_to_brep_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: SegmentMeshRequest = json_in(request_json)?;
    let solid = mesh_regions_to_brep(&request.positions, &request.indices, &request.options)
        .map_err(javascript_error)?;
    solid_buffer(&solid, "{}".into())
}

#[derive(Deserialize)]
pub(crate) struct ThickenSheetRequest {
    /// The sheet as an untrimmed surface patch over its full domain.
    pub(crate) surface: NurbsSurface,
    /// Signed thickness (positive = along the sheet normal Su×Sv).
    pub(crate) thickness: f64,
    /// Split the thickness half per side (sheet becomes the mid-surface).
    #[serde(default)]
    pub(crate) symmetric: bool,
}

#[wasm_bindgen]
pub fn thicken_face_sheet_json(request_json: &str) -> Result<String, JsValue> {
    let request: ThickenSheetRequest = json_in(request_json)?;
    let bodies = thicken_face_sheet(&request.surface, request.thickness, request.symmetric)
        .map_err(javascript_refusal)?;
    json_out(&bodies)
}

#[wasm_bindgen]
pub fn thicken_face_sheet_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: ThickenSheetRequest = json_in(request_json)?;
    let bodies = thicken_face_sheet(&request.surface, request.thickness, request.symmetric)
        .map_err(javascript_refusal)?;
    solid_buffer(single_body(&bodies, "thicken_face_sheet")?, "{}".into())
}

#[derive(Deserialize)]
pub(crate) struct ThickenTrimmedSheetRequest {
    pub(crate) surface: NurbsSurface,
    /// Pcurve trim loops on the sheet: outer loop first (counter-clockwise),
    /// optional holes (clockwise).
    pub(crate) loops: Vec<Vec<NurbsCurve>>,
    pub(crate) thickness: f64,
    #[serde(default)]
    pub(crate) symmetric: bool,
}

#[wasm_bindgen]
pub fn thicken_trimmed_sheet_json(request_json: &str) -> Result<String, JsValue> {
    let request: ThickenTrimmedSheetRequest = json_in(request_json)?;
    let bodies = thicken_trimmed_sheet(
        &request.surface,
        &request.loops,
        request.thickness,
        request.symmetric,
    )
    .map_err(javascript_refusal)?;
    json_out(&bodies)
}

#[wasm_bindgen]
pub fn thicken_trimmed_sheet_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: ThickenTrimmedSheetRequest = json_in(request_json)?;
    let bodies = thicken_trimmed_sheet(
        &request.surface,
        &request.loops,
        request.thickness,
        request.symmetric,
    )
    .map_err(javascript_refusal)?;
    solid_buffer(single_body(&bodies, "thicken_trimmed_sheet")?, "{}".into())
}

/// The one body a single-solid BUFFER export can carry.
///
/// A thicken whose trim a fold BAND crosses produces one body per regular
/// piece, and this buffer shape holds exactly one solid. Returning the first
/// and dropping the rest would hand a caller half a result it could not know
/// was half, so the count is named and the `_json` export — which carries the
/// whole list — is pointed at instead.
fn single_body<'a>(bodies: &'a [BrepSolid], operation: &str) -> Result<&'a BrepSolid, JsValue> {
    match bodies {
        [body] => Ok(body),
        _ => Err(javascript_error(format!(
            "{operation}: the selected trim thickens into {} bodies and this buffer carries one \
             — call {operation}_json, which returns them all",
            bodies.len()
        ))),
    }
}

#[wasm_bindgen]
pub fn offset_face_carrier_json(request_json: &str) -> Result<String, JsValue> {
    let request: OffsetFaceCarrierRequest = json_in(request_json)?;
    let carrier = offset_face_carrier(
        &request.solid,
        request.face_id,
        request.distance,
        request.planar_extension,
    )
    .map_err(javascript_refusal)?;
    json_out(&carrier)
}

#[wasm_bindgen]
pub fn offset_shell_json(request_json: &str) -> Result<String, JsValue> {
    let request: OffsetShellRequest = json_in(request_json)?;
    let result = offset_shell(&request.solid, &request.opening_face_ids, request.distance)
        .map_err(javascript_refusal)?;
    json_out(&result)
}

#[wasm_bindgen]
pub fn offset_shell_diagnostics_json(request_json: &str) -> Result<String, JsValue> {
    let request: OffsetShellDiagnosticsRequest = json_in(request_json)?;
    let result = offset_shell_with_diagnostics(
        &request.solid,
        &request.opening_face_ids,
        request.distance,
        request.tolerances,
    )
    .map_err(javascript_refusal)?;
    json_out(&result)
}

#[wasm_bindgen]
pub fn export_step_json(request_json: &str) -> Result<String, JsValue> {
    let request: ExportStepRequest = json_in(request_json)?;
    export_step(
        &request.solids,
        &request.name,
        &request.unit,
        &request.timestamp,
    )
    .map_err(javascript_error)
}

#[wasm_bindgen]
pub fn merge_curve_continuations_json(request_json: &str) -> Result<String, JsValue> {
    let request: CoalesceRequest = json_in(request_json)?;
    let solid = merge_curve_continuation_edges(&request.solid, request.tolerance)
        .map_err(|refusal| javascript_error(refusal.message))?;
    json_out(&solid)
}

#[wasm_bindgen]
pub fn solve_sketch_json(request_json: &str) -> Result<String, JsValue> {
    solve_sketch_from_json(request_json).map_err(javascript_error)
}
