use super::*;

#[wasm_bindgen]
pub fn extrude_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: ExtrudeBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = extrude_profile_brep(&request.profile, request.direction, request.distance)
        .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn revolve_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: RevolveBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = revolve_profile_brep_named(
        &request.profile,
        request.axis_point,
        request.axis_direction,
        request.angle,
        &request.side_names,
        &request.cap_names,
    )
    .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn fillet_edge_json(request_json: &str) -> Result<String, JsValue> {
    let request: EdgeBlendRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let edge_id = resolve_blend_edge(&request).map_err(javascript_error)?;
    let solid = fillet_edge(
        &request.solid,
        edge_id,
        request.radius,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn fillet_edge_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: EdgeBlendRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let edge_id = resolve_blend_edge(&request).map_err(javascript_error)?;
    let solid = fillet_edge(
        &request.solid,
        edge_id,
        request.radius,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn chamfer_edge_json(request_json: &str) -> Result<String, JsValue> {
    let request: EdgeBlendRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let edge_id = resolve_blend_edge(&request).map_err(javascript_error)?;
    let solid = chamfer_edge(
        &request.solid,
        edge_id,
        request.radius,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn chamfer_edge_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: EdgeBlendRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let edge_id = resolve_blend_edge(&request).map_err(javascript_error)?;
    let solid = chamfer_edge(
        &request.solid,
        edge_id,
        request.radius,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

/// Convex-corner (vertex/"star") blend request — Golovanov §6.9.7. The three
/// incident edges must ALREADY be filleted; `corner_point` is the original
/// sharp-corner coordinate (trimmed away by the edge fillets, so it names the
/// corner geometrically — the vertex itself no longer exists).
#[derive(Deserialize)]
pub(crate) struct CornerBlendRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) corner_point: Vec3,
    pub(crate) radius: f64,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

/// Multi-edge fillet/chamfer request: the object plus one 3D point on each
/// selected edge. The kernel fillets them all and (for fillets) rounds the
/// convex vertices where >=3 of them meet — the whole operation in one call.
#[derive(Deserialize)]
pub(crate) struct FilletEdgesRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) edge_points: Vec<Vec3>,
    pub(crate) radius: f64,
    #[serde(default)]
    pub(crate) chamfer: bool,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

#[wasm_bindgen]
pub fn fillet_edges_json(request_json: &str) -> Result<String, JsValue> {
    let request: FilletEdgesRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = fillet_edges(
        &request.solid,
        &request.edge_points,
        None,
        request.radius,
        request.chamfer,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn fillet_edges_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: FilletEdgesRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = fillet_edges(
        &request.solid,
        &request.edge_points,
        None,
        request.radius,
        request.chamfer,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[derive(serde::Deserialize)]
pub(crate) struct FilletEdgesVariableRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) edge_points: Vec<Vec3>,
    /// (edge-fraction in [0,1], radius) stops applied along each edge.
    pub(crate) radii: Vec<(f64, f64)>,
    #[serde(default)]
    pub(crate) chamfer: bool,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

#[wasm_bindgen]
pub fn fillet_edges_variable_json(request_json: &str) -> Result<String, JsValue> {
    let request: FilletEdgesVariableRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = fillet_edges_variable(
        &request.solid,
        &request.edge_points,
        None,
        &request.radii,
        request.chamfer,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn fillet_edges_variable_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: FilletEdgesVariableRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = fillet_edges_variable(
        &request.solid,
        &request.edge_points,
        None,
        &request.radii,
        request.chamfer,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[derive(serde::Deserialize)]
pub(crate) struct ChamferEdgesAsymmetricRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) edge_points: Vec<Vec3>,
    /// Setback along face 1 (into_first).
    pub(crate) d1: f64,
    /// Setback along face 2 (into_second).
    pub(crate) d2: f64,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

#[wasm_bindgen]
pub fn chamfer_edges_asymmetric_json(request_json: &str) -> Result<String, JsValue> {
    let request: ChamferEdgesAsymmetricRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = chamfer_edges_asymmetric(
        &request.solid,
        &request.edge_points,
        None,
        request.d1,
        request.d2,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn chamfer_edges_asymmetric_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: ChamferEdgesAsymmetricRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = chamfer_edges_asymmetric(
        &request.solid,
        &request.edge_points,
        None,
        request.d1,
        request.d2,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[derive(serde::Deserialize)]
pub(crate) struct ChamferEdgesAngleRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) edge_points: Vec<Vec3>,
    /// Setback along face 1 (into_first).
    pub(crate) d1: f64,
    /// Angle (radians) between the chamfer face and face 1.
    pub(crate) angle: f64,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

#[wasm_bindgen]
pub fn chamfer_edges_angle_json(request_json: &str) -> Result<String, JsValue> {
    let request: ChamferEdgesAngleRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = chamfer_edges_angle(
        &request.solid,
        &request.edge_points,
        None,
        request.d1,
        request.angle,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn chamfer_edges_angle_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: ChamferEdgesAngleRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = chamfer_edges_angle(
        &request.solid,
        &request.edge_points,
        None,
        request.d1,
        request.angle,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn round_convex_corner_json(request_json: &str) -> Result<String, JsValue> {
    let request: CornerBlendRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = round_convex_corner(
        &request.solid,
        request.corner_point,
        request.radius,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn round_convex_corner_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: CornerBlendRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = round_convex_corner(
        &request.solid,
        request.corner_point,
        request.radius,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn loft_brep_json(request_json: &str) -> Result<String, JsValue> {
    let request: LoftBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = loft_profile_brep(&request.sections).map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn extrude_brep_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: ExtrudeBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = extrude_profile_brep(&request.profile, request.direction, request.distance)
        .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn extrude_profile_brep_draft_json(request_json: &str) -> Result<String, JsValue> {
    let request: ExtrudeDraftRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = extrude_profile_brep_draft(
        &request.profile,
        request.direction,
        request.distance,
        request.draft_angle,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn extrude_profile_brep_draft_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: ExtrudeDraftRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = extrude_profile_brep_draft(
        &request.profile,
        request.direction,
        request.distance,
        request.draft_angle,
        request.name.as_deref(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn rib_from_profile_json(request_json: &str) -> Result<String, JsValue> {
    let request: RibRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = rib_from_profile(
        &request.solid,
        &request.profile,
        request.thickness,
        request.extrude_dir,
        request.plane_normal,
        request.extrusion,
        &request.names(),
    )
    .map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn rib_from_profile_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: RibRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = rib_from_profile(
        &request.solid,
        &request.profile,
        request.thickness,
        request.extrude_dir,
        request.plane_normal,
        request.extrusion,
        &request.names(),
    )
    .map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn revolve_brep_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: RevolveBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = revolve_profile_brep_named(
        &request.profile,
        request.axis_point,
        request.axis_direction,
        request.angle,
        &request.side_names,
        &request.cap_names,
    )
    .map_err(javascript_error)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn loft_brep_buffer(request_json: &str) -> Result<WasmSolidBuffer, JsValue> {
    let request: LoftBrepRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = loft_profile_brep(&request.sections).map_err(javascript_refusal)?;
    solid_buffer(&solid, "{}".into())
}

#[wasm_bindgen]
pub fn loft_profile_brep_guided_json(request_json: &str) -> Result<String, JsValue> {
    let request: GuidedLoftRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = guided_loft_dispatch(&request).map_err(javascript_refusal)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}
