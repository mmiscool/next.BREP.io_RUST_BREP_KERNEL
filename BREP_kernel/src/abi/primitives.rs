use super::*;

#[wasm_bindgen]
pub fn box_brep_json(
    corner_x: f64,
    corner_y: f64,
    corner_z: f64,
    size_x: f64,
    size_y: f64,
    size_z: f64,
) -> Result<String, JsValue> {
    let solid = make_box_brep(
        Vec3::new(corner_x, corner_y, corner_z),
        size_x,
        size_y,
        size_z,
    )
    .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn pyramid_brep_json(
    corner_x: f64,
    corner_y: f64,
    corner_z: f64,
    side_length: f64,
    height: f64,
    sides: u32,
) -> Result<String, JsValue> {
    let solid = make_pyramid_brep(
        Vec3::new(corner_x, corner_y, corner_z),
        side_length,
        height,
        sides as usize,
    )
    .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn cylinder_brep_json(
    base_x: f64,
    base_y: f64,
    base_z: f64,
    axis_x: f64,
    axis_y: f64,
    axis_z: f64,
    radius: f64,
    height: f64,
) -> Result<String, JsValue> {
    let solid = make_cylinder_brep(
        Vec3::new(base_x, base_y, base_z),
        Vec3::new(axis_x, axis_y, axis_z),
        radius,
        height,
    )
    .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn cone_brep_json(radius_bottom: f64, radius_top: f64, height: f64) -> Result<String, JsValue> {
    let solid = make_cone_brep(
        Vec3::default(),
        Vec3::new(0.0, 1.0, 0.0),
        radius_bottom,
        radius_top,
        height,
    )
    .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn sphere_brep_json(radius: f64) -> Result<String, JsValue> {
    let solid = make_sphere_brep(Vec3::default(), radius, Vec3::new(0.0, 1.0, 0.0))
        .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

/// Import an ISO-10303-21 (STEP Part 21) document and return the reconstructed
/// kernel solids as a JSON array of `BrepSolid` records. Round-trips exactly
/// what `export_step` writes; unsupported entities yield a clear error string.
#[wasm_bindgen]
pub fn import_step_json(step_text: &str) -> Result<String, JsValue> {
    let solids = import_step(step_text).map_err(javascript_error)?;
    serde_json::to_string(&solids).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn torus_brep_json(major_radius: f64, minor_radius: f64) -> Result<String, JsValue> {
    let solid = make_torus_brep(
        Vec3::default(),
        Vec3::new(0.0, 1.0, 0.0),
        major_radius,
        minor_radius,
    )
    .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}
