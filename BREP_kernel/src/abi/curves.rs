use super::*;

#[wasm_bindgen]
pub fn line_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: LineRequest = json_in(request_json)?;
    json_out(&make_line(request.start, request.end).map_err(javascript_error)?)
}

#[wasm_bindgen]
pub fn arc_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: ArcRequest = json_in(request_json)?;
    json_out(
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
}

#[wasm_bindgen]
pub fn circle_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: CircleRequest = json_in(request_json)?;
    json_out(
        &make_circle(request.center, request.normal, request.radius).map_err(javascript_error)?,
    )
}

#[wasm_bindgen]
pub fn parabola_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: ParabolaRequest = json_in(request_json)?;
    json_out(
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
}

#[wasm_bindgen]
pub fn hyperbola_curve_json(request_json: &str) -> Result<String, JsValue> {
    let request: HyperbolaRequest = json_in(request_json)?;
    json_out(
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
}

#[wasm_bindgen]
pub fn uniform_clamped_knots_json(
    control_point_count: usize,
    degree: usize,
) -> Result<String, JsValue> {
    json_out(
        &uniform_clamped_knots(control_point_count, degree).map_err(javascript_error)?,
    )
}

#[wasm_bindgen]
pub fn solve_banded_json(request_json: &str) -> Result<String, JsValue> {
    let request: BandedSolveRequest = json_in(request_json)?;
    json_out(
        &solve_banded(&request.matrix, &request.rhs, request.bandwidth)
            .map_err(javascript_error)?,
    )
}

#[wasm_bindgen]
pub fn solve_dense_json(request_json: &str) -> Result<String, JsValue> {
    let request: DenseSolveRequest = json_in(request_json)?;
    json_out(&solve_dense(request.matrix, request.rhs).map_err(javascript_error)?)
}

#[wasm_bindgen]
pub fn segment_intersection_json(request_json: &str) -> Result<String, JsValue> {
    let request: SegmentIntersectionRequest = json_in(request_json)?;
    json_out(&segment_intersection(
        request.a1,
        request.b1,
        request.a2,
        request.b2,
        request.tolerance,
    ))
}

#[wasm_bindgen]
pub fn point_in_polygon_json(request_json: &str) -> Result<String, JsValue> {
    let request: PointInPolygonRequest = json_in(request_json)?;
    json_out(&point_in_polygon(
        request.point,
        &request.polygon,
        request.tolerance,
    ))
}

#[wasm_bindgen]
pub fn parameter_point_in_face_json(request_json: &str) -> Result<String, JsValue> {
    let request: ParameterPointInFaceRequest = json_in(request_json)?;
    let classification = parameter_point_in_face(&request.face, request.point, request.tolerance)
        .map_err(javascript_error)?;
    let result = match classification {
        PolygonClass::Inside => "in",
        PolygonClass::Outside => "out",
        PolygonClass::Boundary => "boundary",
    };
    json_out(result)
}

#[wasm_bindgen]
pub fn interpolate_curve_thinned_json(request_json: &str) -> Result<String, JsValue> {
    let request: ThinnedCurveRequest = json_in(request_json)?;
    json_out(
        &interpolate_curve_thinned(&request.points, request.degree, request.maximum_points)
            .map_err(javascript_error)?,
    )
}

#[wasm_bindgen]
pub fn validate_brep_json(solid_json: &str) -> Result<String, JsValue> {
    let solid: BrepSolid = json_in(solid_json)?;
    json_out(&solid.validate())
}

#[wasm_bindgen]
pub fn validate_brep_detailed_json(solid_json: &str) -> Result<String, JsValue> {
    let solid: BrepSolid = json_in(solid_json)?;
    let policy = KernelTolerances::for_solid(&solid, 1e-7);
    json_out(&solid.validate_detailed(&policy))
}
