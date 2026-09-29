use super::*;

#[wasm_bindgen]
pub fn line_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: LineRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(&make_line(request.start, request.end).map_err(javascript_error)?)
        .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn arc_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: ArcRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(
        &make_arc(
            request.center,
            request.x_axis,
            request.y_axis,
            request.radius,
            request.start_angle,
            request.end_angle,
        )
        .map_err(javascript_error)?,
    )
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn circle_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: CircleRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(
        &make_circle(request.center, request.normal, request.radius).map_err(javascript_error)?,
    )
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn parabola_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: ParabolaRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(
        &make_parabola(
            request.vertex,
            request.axis,
            request.latus_direction,
            request.focal,
            request.t0,
            request.t1,
        )
        .map_err(javascript_error)?,
    )
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn hyperbola_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: HyperbolaRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(
        &make_hyperbola(
            request.center,
            request.major_axis,
            request.minor_axis,
            request.a,
            request.b,
            request.t0,
            request.t1,
        )
        .map_err(javascript_error)?,
    )
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn uniform_clamped_knots_json(
    control_point_count: usize,
    degree: usize,
) -> Result<String, JsValue> {
    serde_json::to_string(
        &uniform_clamped_knots(control_point_count, degree).map_err(javascript_error)?,
    )
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn solve_banded_json(request_json: &str) -> Result<String, JsValue> {
    let request: BandedSolveRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(
        &solve_banded(&request.matrix, &request.rhs, request.bandwidth)
            .map_err(javascript_error)?,
    )
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn solve_dense_json(request_json: &str) -> Result<String, JsValue> {
    let request: DenseSolveRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(&solve_dense(request.matrix, request.rhs).map_err(javascript_error)?)
        .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn segment_intersection_json(request_json: &str) -> Result<String, JsValue> {
    let request: SegmentIntersectionRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(&segment_intersection(
        request.a1,
        request.b1,
        request.a2,
        request.b2,
        request.tolerance,
    ))
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn point_in_polygon_json(request_json: &str) -> Result<String, JsValue> {
    let request: PointInPolygonRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(&point_in_polygon(
        request.point,
        &request.polygon,
        request.tolerance,
    ))
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn parameter_point_in_face_json(request_json: &str) -> Result<String, JsValue> {
    let request: ParameterPointInFaceRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    let classification = parameter_point_in_face(&request.face, request.point, request.tolerance)
        .map_err(javascript_error)?;
    let result = match classification {
        PolygonClass::Inside => "in",
        PolygonClass::Outside => "out",
        PolygonClass::Boundary => "boundary",
    };
    serde_json::to_string(result).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn interpolate_curve_thinned_json(request_json: &str) -> Result<String, JsValue> {
    let request: ThinnedCurveRequest =
        serde_json::from_str(request_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(
        &interpolate_curve_thinned(&request.points, request.degree, request.maximum_points)
            .map_err(javascript_error)?,
    )
    .map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn validate_brep_json(solid_json: &str) -> Result<String, JsValue> {
    let solid: BrepSolid =
        serde_json::from_str(solid_json).map_err(|error| javascript_error(error.to_string()))?;
    serde_json::to_string(&solid.validate()).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn validate_brep_detailed_json(solid_json: &str) -> Result<String, JsValue> {
    let solid: BrepSolid =
        serde_json::from_str(solid_json).map_err(|error| javascript_error(error.to_string()))?;
    let policy = KernelTolerances::for_solid(&solid, 1e-7);
    serde_json::to_string(&solid.validate_detailed(&policy))
        .map_err(|error| javascript_error(error.to_string()))
}
