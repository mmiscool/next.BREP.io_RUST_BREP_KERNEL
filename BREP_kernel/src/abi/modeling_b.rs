use super::*;

#[derive(Deserialize)]
pub(crate) struct TangentLoftRequest {
    pub(crate) sections: Vec<Vec<NurbsCurve>>,
    pub(crate) start_direction: Vec3,
    pub(crate) end_direction: Vec3,
}

#[wasm_bindgen]
pub fn loft_profile_brep_closed_json(request_json: &str) -> Result<String, JsValue> {
    let request: LoftBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = loft_profile_brep_closed(&request.sections).map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn loft_profile_brep_closed_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: LoftBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = loft_profile_brep_closed(&request.sections).map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn loft_profile_brep_tangent_json(request_json: &str) -> Result<String, JsValue> {
    let request: TangentLoftRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = loft_profile_brep_tangent(
        &request.sections,
        request.start_direction,
        request.end_direction,
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn loft_profile_brep_tangent_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: TangentLoftRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    let request: GuidedLoftRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = guided_loft_dispatch(&request).map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn sweep_profile_along_path_json(request_json: &str) -> Result<String, JsValue> {
    let request: SweepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = sweep_profile_along_path(&request.profile, &request.path, request.name.as_deref())
        .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn sweep_profile_along_path_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: SweepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    let request: TwistedSweepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = sweep_profile_twisted(
        &request.profile,
        &request.path,
        request.twist_angle,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn sweep_profile_twisted_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: TwistedSweepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    let request: HelixSweepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn sweep_profile_helix_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: HelixSweepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    let request: TransformBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let transform = AffineTransform::new(request.matrix).map_err(javascript_error)?;
    let solid = transform_brep(&request.solid, transform, request.reverse_orientation)
        .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn mirror_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: MirrorBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = mirror_brep(&request.solid, request.plane_point, request.plane_normal)
        .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn mirror_brep_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: MirrorBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = mirror_brep(&request.solid, request.plane_point, request.plane_normal)
        .map_err(javascript_error)?;
    solid_buffer(&solid, "{}".into())
}

/// Cut a solid by a plane (Golovanov §6.4). Returns `{ "below": <solid>,
/// "above": <solid> }` — the −n and +n pieces of the split. Errors with
/// "plane does not intersect the solid" when the plane misses the body.
#[wasm_bindgen]
pub fn split_solid_by_plane_json(request_json: &str) -> Result<String, JsValue> {
    let request: SplitByPlaneRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let (below, above) =
        split_solid_by_plane(&request.solid, request.plane_point, request.plane_normal)
            .map_err(javascript_error)?;
    let result = serde_json::json!({ "below": below, "above": above });
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

/// Cut a solid by an analytic tool surface (Golovanov §6.4). The request is
/// `{ "solid": <solid>, "tool": { "type": "cylinder"|"sphere"|"cone"|"torus"|
/// "plane", ... } }`. Returns `{ "pieces": [<inside>, <outside>] }` (for a
/// plane, `[below, above]`). Errors with "tool surface does not divide the
/// solid" when the tool does not cleanly cut the body.
#[wasm_bindgen]
pub fn split_solid_by_surface_json(request_json: &str) -> Result<String, JsValue> {
    let request: SplitBySurfaceRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let pieces = split_solid_by_surface(&request.solid, &request.tool).map_err(javascript_error)?;
    let result = serde_json::json!({ "pieces": pieces });
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

/// Cut a solid by the analytic carrier of a selected face's `surface` (Golovanov
/// §6.4). The face may be a plane, cylinder, cone, or sphere; the kernel
/// recognizes its analytic type and extends it to span the body. Returns
/// `{ "pieces": [<below/inside>, <above/outside>] }`. Errors on non-analytic
/// faces or when the carrier does not cleanly divide the body.
#[wasm_bindgen]
pub fn split_solid_by_face_surface_json(request_json: &str) -> Result<String, JsValue> {
    let request: SplitByFaceSurfaceRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let pieces =
        split_solid_by_face_surface(&request.solid, &request.surface).map_err(javascript_error)?;
    let result = serde_json::json!({ "pieces": pieces });
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn solid_mass_properties_json(solid_json: &str) -> Result<String, JsValue> {
    let solid: BrepSolid =
        serde_json::from_str(solid_json).map_err(|error| javascript_error(error.to_string()))?;
    let properties = solid_mass_properties(&solid).map_err(javascript_error)?;
    serde_json::to_string(&properties).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn solid_mass_properties_full_json(solid_json: &str) -> Result<String, JsValue> {
    let solid: BrepSolid =
        serde_json::from_str(solid_json).map_err(|error| javascript_error(error.to_string()))?;
    let properties = solid_mass_properties_full(&solid).map_err(javascript_error)?;
    serde_json::to_string(&properties).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn face_metrics_json(face_json: &str) -> Result<String, JsValue> {
    let face: FaceRecord =
        serde_json::from_str(face_json).map_err(|error| javascript_error(error.to_string()))?;
    let metrics = FaceMetrics {
        parameter_space_area: parameter_space_area(&face).map_err(javascript_error)?,
        area: face_area(&face).map_err(javascript_error)?,
        volume_contribution: face_volume_contribution(&face).map_err(javascript_error)?,
        trim_polygons: trim_polygons(&face).map_err(javascript_error)?,
    };
    serde_json::to_string(&metrics).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn tessellate_face_json(request_json: &str) -> Result<String, JsValue> {
    let request: TessellateFaceRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let mesh = tessellate_face(
        &request.face,
        TessellationOptions {
            slabs_per_span_u: request.slabs_per_span_u,
            steps_per_span_v: request.steps_per_span_v,
        },
        request.face.id as u32,
    )
    .map_err(javascript_error)?;
    serde_json::to_string(&mesh).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn tessellate_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: TessellateBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let mesh = tessellate_brep(
        &request.solid,
        TessellationOptions {
            slabs_per_span_u: request.slabs_per_span_u,
            steps_per_span_v: request.steps_per_span_v,
        },
    )
    .map_err(javascript_error)?;
    serde_json::to_string(&mesh).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn classify_points_json(request_json: &str) -> Result<String, JsValue> {
    let request: ClassifyPointsRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let classifications = request
        .points
        .into_iter()
        .map(|point| classify_point(point, &request.solid, request.tolerance))
        .collect::<Result<Vec<_>, _>>()
        .map_err(javascript_error)?;
    serde_json::to_string(&classifications).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn arrange_segments_json(request_json: &str) -> Result<String, JsValue> {
    let request: ArrangeSegmentsRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let regions =
        arrange_segments(&request.segments, request.tolerance).map_err(javascript_error)?;
    serde_json::to_string(&regions).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn build_pcurve_json(request_json: &str) -> Result<String, JsValue> {
    let request: BuildPcurveRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let pcurve =
        build_pcurve_on_surface(&request.surface, &request.curve).map_err(javascript_error)?;
    serde_json::to_string(&pcurve).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn build_imprints_json(request_json: &str) -> Result<String, JsValue> {
    let request: BuildImprintsRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let result = build_imprints(&request.solid_a, &request.solid_b, &request.options)
        .map_err(javascript_refusal)?;
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn apply_edge_splits_json(request_json: &str) -> Result<String, JsValue> {
    let request: ApplyEdgeSplitsRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let result = apply_edge_splits(&request.solid, request.operand, &request.imprint)
        .map_err(javascript_refusal)?;
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn fragment_faces_json(request_json: &str) -> Result<String, JsValue> {
    let request: FragmentFacesRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let result = fragment_solid(&request.solid, request.operand, &request.imprint)
        .map_err(javascript_refusal)?;
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn boolean_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: BooleanRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let result = boolean_operation(
        &request.first,
        &request.second,
        request.operation,
        &request.options,
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn boolean_brep_diagnostics_json(request_json: &str) -> Result<String, JsValue> {
    let request: BooleanRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let result = boolean_operation_with_diagnostics(
        &request.first,
        &request.second,
        request.operation,
        &request.options,
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn offset_surface_json(request_json: &str) -> Result<String, JsValue> {
    let request: OffsetSurfaceRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let surface = offset_surface(&request.face, request.distance, request.planar_extension)
        .map_err(javascript_refusal)?;
    serde_json::to_string(&surface).map_err(|error| javascript_error(error.to_string()))
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
    let request: SolveAssemblyRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solution = solve_assembly(&request.bodies, &request.mates, &request.options)
        .map_err(javascript_error)?;
    serde_json::to_string(&solution).map_err(|error| javascript_error(error.to_string()))
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
    let request: SegmentMeshRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let segmentation = segment_mesh_faces(&request.positions, &request.indices, &request.options)
        .map_err(javascript_error)?;
    serde_json::to_string(&segmentation).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn mesh_regions_to_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: SegmentMeshRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = mesh_regions_to_brep(&request.positions, &request.indices, &request.options)
        .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn mesh_regions_to_brep_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: SegmentMeshRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    let request: ThickenSheetRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let bodies = thicken_face_sheet(&request.surface, request.thickness, request.symmetric)
        .map_err(javascript_refusal)?;
    serde_json::to_string(&bodies).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn thicken_face_sheet_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: ThickenSheetRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    let request: ThickenTrimmedSheetRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let bodies = thicken_trimmed_sheet(
        &request.surface,
        &request.loops,
        request.thickness,
        request.symmetric,
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&bodies).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn thicken_trimmed_sheet_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: ThickenTrimmedSheetRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    let request: OffsetFaceCarrierRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let carrier = offset_face_carrier(
        &request.solid,
        request.face_id,
        request.distance,
        request.planar_extension,
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&carrier).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn offset_shell_json(request_json: &str) -> Result<String, JsValue> {
    let request: OffsetShellRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let result = offset_shell(&request.solid, &request.opening_face_ids, request.distance)
        .map_err(javascript_refusal)?;
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn offset_shell_diagnostics_json(request_json: &str) -> Result<String, JsValue> {
    let request: OffsetShellDiagnosticsRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let result = offset_shell_with_diagnostics(
        &request.solid,
        &request.opening_face_ids,
        request.distance,
        request.tolerances,
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn export_step_json(request_json: &str) -> Result<String, JsValue> {
    let request: ExportStepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
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
    let request: CoalesceRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = merge_curve_continuation_edges(&request.solid, request.tolerance)
        .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn solve_sketch_json(request_json: &str) -> Result<String, JsValue> {
    solve_sketch_from_json(request_json).map_err(javascript_error)
}
