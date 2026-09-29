use super::*;

#[derive(Deserialize)]
pub(crate) struct CurveEvaluationRequest {
    pub(crate) curve: NurbsCurve,
    pub(crate) parameters: Vec<f64>,
}

#[derive(Deserialize)]
pub(crate) struct CurveDerivativeRequest {
    pub(crate) curve: NurbsCurve,
    pub(crate) parameter: f64,
    pub(crate) derivative_count: usize,
}

#[derive(Deserialize)]
pub(crate) struct CurveProjectionRequest {
    pub(crate) curve: NurbsCurve,
    pub(crate) point: Vec3,
}

#[derive(Deserialize)]
pub(crate) struct SurfaceProjectionRequest {
    pub(crate) surface: NurbsSurface,
    pub(crate) point: Vec3,
}

#[derive(Deserialize)]
pub(crate) struct CurveSurfaceIntersectionRequest {
    pub(crate) curve: NurbsCurve,
    pub(crate) surface: NurbsSurface,
    #[serde(default = "default_intersection_tolerance")]
    pub(crate) tolerance: f64,
}

#[derive(Deserialize)]
pub(crate) struct CurveCurveIntersectionRequest {
    pub(crate) first: NurbsCurve,
    pub(crate) second: NurbsCurve,
    #[serde(default = "default_intersection_tolerance")]
    pub(crate) tolerance: f64,
}

#[derive(Deserialize)]
pub(crate) struct SurfaceSurfaceIntersectionRequest {
    pub(crate) first: NurbsSurface,
    pub(crate) second: NurbsSurface,
    #[serde(default)]
    pub(crate) options: SurfaceIntersectionOptions,
}

pub(crate) fn default_intersection_tolerance() -> f64 {
    1e-7
}

#[derive(Deserialize)]
pub(crate) struct LineRequest {
    pub(crate) start: Vec3,
    pub(crate) end: Vec3,
}

#[derive(Deserialize)]
pub(crate) struct ArcRequest {
    pub(crate) center: Vec3,
    pub(crate) x_axis: Vec3,
    pub(crate) y_axis: Vec3,
    pub(crate) radius: f64,
    pub(crate) start_angle: f64,
    pub(crate) end_angle: f64,
}

#[derive(Deserialize)]
pub(crate) struct CircleRequest {
    pub(crate) center: Vec3,
    pub(crate) normal: Vec3,
    pub(crate) radius: f64,
}

#[derive(Deserialize)]
pub(crate) struct ParabolaRequest {
    pub(crate) vertex: Vec3,
    pub(crate) axis: Vec3,
    pub(crate) latus_direction: Vec3,
    pub(crate) focal: f64,
    pub(crate) t0: f64,
    pub(crate) t1: f64,
}

#[derive(Deserialize)]
pub(crate) struct HyperbolaRequest {
    pub(crate) center: Vec3,
    pub(crate) major_axis: Vec3,
    pub(crate) minor_axis: Vec3,
    pub(crate) a: f64,
    pub(crate) b: f64,
    pub(crate) t0: f64,
    pub(crate) t1: f64,
}

#[derive(Deserialize)]
pub(crate) struct BandedSolveRequest {
    pub(crate) matrix: Vec<Vec<f64>>,
    pub(crate) rhs: Vec<f64>,
    pub(crate) bandwidth: usize,
}

#[derive(Deserialize)]
pub(crate) struct DenseSolveRequest {
    pub(crate) matrix: Vec<Vec<f64>>,
    pub(crate) rhs: Vec<f64>,
}

#[derive(Deserialize)]
pub(crate) struct SegmentIntersectionRequest {
    pub(crate) a1: Vec2,
    pub(crate) b1: Vec2,
    pub(crate) a2: Vec2,
    pub(crate) b2: Vec2,
    pub(crate) tolerance: f64,
}

#[derive(Deserialize)]
pub(crate) struct PointInPolygonRequest {
    pub(crate) point: Vec2,
    pub(crate) polygon: Vec<Vec2>,
    pub(crate) tolerance: f64,
}

#[derive(Deserialize)]
pub(crate) struct ParameterPointInFaceRequest {
    pub(crate) face: FaceRecord,
    pub(crate) point: Vec2,
    pub(crate) tolerance: f64,
}

#[derive(Deserialize)]
pub(crate) struct ThinnedCurveRequest {
    pub(crate) points: Vec<Vec3>,
    #[serde(default = "default_curve_fit_degree")]
    pub(crate) degree: usize,
    #[serde(default = "default_curve_fit_maximum_points")]
    pub(crate) maximum_points: usize,
}

pub(crate) fn default_curve_fit_degree() -> usize {
    3
}

pub(crate) fn default_curve_fit_maximum_points() -> usize {
    200
}

/// A typed refusal crossing the wasm ABI: the flattened refusal object
/// (`{"class":…,…,"stage":…,"message":…}`) as the error's text, so a JS
/// consumer can dispatch on the class and still show the message.
pub(crate) fn javascript_refusal(refusal: crate::KernelRefusal) -> JsValue {
    let text = serde_json::to_string(&refusal).unwrap_or_else(|_| refusal.message.clone());
    javascript_error(text)
}

pub(crate) fn javascript_error(error: String) -> JsValue {
    JsValue::from_str(&error)
}

#[wasm_bindgen]
pub fn curve_evaluate_json(request_json: &str) -> Result<String, JsValue> {
    let request: CurveEvaluationRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let curve = NurbsCurve::new(
        request.curve.degree,
        request.curve.knots,
        request.curve.control_points,
    )
    .map_err(javascript_error)?;
    let points: Result<Vec<Vec3>, String> = request
        .parameters
        .into_iter()
        .map(|parameter| curve.evaluate(parameter))
        .collect();
    serde_json::to_string(&points.map_err(javascript_error)?)
        .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn project_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: CurveProjectionRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let projection =
        project_point_to_curve(&request.curve, request.point).map_err(javascript_error)?;
    serde_json::to_string(&projection).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn project_surface_json(request_json: &str) -> Result<String, JsValue> {
    let request: SurfaceProjectionRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let projection =
        project_point_to_surface(&request.surface, request.point).map_err(javascript_error)?;
    serde_json::to_string(&projection).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn intersect_curve_surface_json(request_json: &str) -> Result<String, JsValue> {
    let request: CurveSurfaceIntersectionRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let intersections =
        intersect_curve_surface(&request.curve, &request.surface, request.tolerance)
            .map_err(javascript_error)?;
    serde_json::to_string(&intersections).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn intersect_curves_json(request_json: &str) -> Result<String, JsValue> {
    let request: CurveCurveIntersectionRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let intersections = intersect_curves(&request.first, &request.second, request.tolerance)
        .map_err(javascript_error)?;
    serde_json::to_string(&intersections).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn intersect_surfaces_json(request_json: &str) -> Result<String, JsValue> {
    let request: SurfaceSurfaceIntersectionRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let intersections = intersect_surfaces(&request.first, &request.second, &request.options)
        .map_err(javascript_error)?;
    serde_json::to_string(&intersections).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn curve_derivatives_json(request_json: &str) -> Result<String, JsValue> {
    let request: CurveDerivativeRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let curve = NurbsCurve::new(
        request.curve.degree,
        request.curve.knots,
        request.curve.control_points,
    )
    .map_err(javascript_error)?;
    serde_json::to_string(
        &curve
            .derivatives(request.parameter, request.derivative_count)
            .map_err(javascript_error)?,
    )
    .map_err(|error| javascript_error(error.to_string()))
}
