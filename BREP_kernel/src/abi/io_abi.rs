use super::*;

#[derive(Serialize)]
pub(crate) struct Capabilities {
    pub(crate) abi_version: u32,
    pub(crate) operations: &'static [&'static str],
}

#[wasm_bindgen]
pub fn capabilities_json() -> String {
    serde_json::to_string(&Capabilities {
        abi_version: 1,
        operations: &[
            "box_mesh",
            "cylinder_mesh",
            "mesh_validate",
            "mesh_volume",
            "nurbs_curve_evaluate",
            "nurbs_curve_derivatives",
            "nurbs_curve_handle_typed_arrays",
            "make_line",
            "make_arc",
            "make_circle",
            "make_parabola",
            "make_hyperbola",
            "uniform_clamped_knots",
            "solve_dense",
            "solve_banded",
            "interpolate_curve_thinned",
            "nurbs_surface_evaluate",
            "nurbs_surface_derivatives",
            "nurbs_surface_normal",
            "nurbs_surface_handle_typed_arrays",
            "brep_solid_resident_handle",
            "brep_topology_validate",
            "brep_topology_validate_detailed",
            "make_box_brep",
            "make_pyramid_brep",
            "make_cylinder_brep",
            "make_cone_brep",
            "make_sphere_brep",
            "make_torus_brep",
            "extrude_profile_brep",
            "extrude_profile_brep_draft",
            "rib_from_profile",
            "revolve_profile_brep",
            "fillet_edge",
            "chamfer_edge",
            "fillet_edges",
            "fillet_edges_variable",
            "chamfer_edges_asymmetric",
            "chamfer_edges_angle",
            "round_convex_corner",
            "loft_profile_brep",
            "loft_profile_brep_guided",
            "loft_profile_brep_guided_frame",
            "loft_profile_brep_tangent",
            "thicken_face_sheet",
            "thicken_trimmed_sheet",
            "segment_mesh_faces",
            "mesh_regions_to_brep",
            "solve_assembly",
            "loft_profile_brep_closed",
            "sweep_profile_along_path",
            "sweep_profile_helix",
            "sweep_profile_twisted",
            "transform_brep",
            "mirror_brep",
            "split_solid_by_plane",
            "split_solid_by_surface",
            "split_solid_by_face_surface",
            "parameter_space_area",
            "trim_polygons",
            "face_mass_properties",
            "solid_mass_properties",
            "solid_mass_properties_full",
            "solid_mass_properties_density",
            "solid_buffer_abi",
            "tessellate_watertight",
            "tessellate_watertight_face_stride",
            "tessellate_watertight_face_stride_shared_samples",
            "tessellate_face",
            "tessellate_brep",
            "project_point_to_curve",
            "project_point_to_surface",
            "intersect_curve_surface",
            "intersect_curves",
            "intersect_surfaces",
            "classify_point",
            "classify_points",
            "parameter_point_in_face",
            "arrange_segments",
            "segment_intersection",
            "point_in_polygon",
            "build_pcurve_on_surface",
            "carrier_preview_patch",
            "build_imprints",
            "apply_edge_splits",
            "fragment_faces",
            "boolean_operation",
            "boolean_operation_diagnostics",
            "offset_surface",
            "offset_face_carrier",
            "merge_curve_continuation_edges",
            "merge_same_surface_faces",
            "offset_shell",
            "offset_shell_diagnostics",
            "delete_face_and_heal",
            "move_faces",
            "sew_solid",
            "export_step",
            "import_step",
            "export_iges",
            "import_iges",
            "write_binary_stl",
            "read_binary_stl",
            "mesh_to_faceted_brep",
            "import_stl_solid",
            "write_obj",
            "read_obj",
            "import_obj_solid",
            "solve_sketch",
        ],
    })
    .expect("serialize capabilities")
}

#[wasm_bindgen]
pub fn box_mesh_json(x: f64, y: f64, z: f64) -> String {
    serde_json::to_string(&make_box(x, y, z)).expect("serialize box")
}

#[wasm_bindgen]
pub fn cylinder_mesh_json(radius: f64, height: f64, segments: u32) -> String {
    serde_json::to_string(&make_cylinder(radius, height, segments as usize))
        .expect("serialize cylinder")
}

#[wasm_bindgen]
pub fn mesh_volume_json(mesh_json: &str) -> Result<f64, JsValue> {
    let mesh: Mesh =
        serde_json::from_str(mesh_json).map_err(|error| JsValue::from_str(&error.to_string()))?;
    // Volume is a signed-tetrahedron sum over positions only; do NOT require a
    // normal buffer (mesh-only solids, e.g. sheet-metal, have none).
    mesh.validate_geometry()
        .map_err(|error| JsValue::from_str(&error))?;
    Ok(mesh.signed_volume())
}

#[wasm_bindgen]
pub fn write_stl_bytes(mesh_json: &str, name: &str) -> Result<Vec<u8>, JsValue> {
    let mesh: Mesh =
        serde_json::from_str(mesh_json).map_err(|error| javascript_error(error.to_string()))?;
    write_binary_stl(&mesh, name).map_err(javascript_error)
}

#[wasm_bindgen]
pub fn read_stl_json(data: &[u8]) -> Result<String, JsValue> {
    let result = read_binary_stl(data).map_err(javascript_error)?;
    serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
}

/// Import binary-STL bytes as a faceted (planar-triangle) BREP solid.
/// `tolerance <= 0` derives the vertex-weld band from the model's
/// bounding-box diagonal.
#[wasm_bindgen]
pub fn import_stl_solid(data: &[u8], tolerance: f64) -> Result<String, JsValue> {
    crate::panic_hook::set_once();
    let stl = read_binary_stl(data).map_err(javascript_error)?;
    let solid = mesh_to_faceted_brep(&stl.positions, None, tolerance).map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}

/// Diagnostic carrier preview: re-express a surface over an inflated domain
/// (open directions inflate about the centre via extended evaluation; closed
/// directions keep their full period). `inflate <= 0` defaults to 2.
#[wasm_bindgen]
pub fn carrier_preview_patch_json(surface_json: &str, inflate: f64) -> Result<String, JsValue> {
    let surface: NurbsSurface =
        serde_json::from_str(surface_json).map_err(|error| javascript_error(error.to_string()))?;
    let inflate = if inflate.is_finite() && inflate > 0.0 {
        inflate
    } else {
        2.0
    };
    let preview = carrier_preview_patch(&surface, inflate).map_err(javascript_error)?;
    serde_json::to_string(&preview).map_err(|error| javascript_error(error.to_string()))
}

#[wasm_bindgen]
pub fn write_obj_text(mesh_json: &str, name: &str) -> Result<String, JsValue> {
    let mesh: Mesh =
        serde_json::from_str(mesh_json).map_err(|error| javascript_error(error.to_string()))?;
    write_obj(&mesh, name).map_err(javascript_error)
}

/// Import Wavefront OBJ text as a faceted (planar-triangle) BREP solid.
/// `tolerance <= 0` derives the vertex-weld band from the model's
/// bounding-box diagonal.
#[wasm_bindgen]
pub fn import_obj_solid(text: &str, tolerance: f64) -> Result<String, JsValue> {
    crate::panic_hook::set_once();
    let obj = read_obj(text).map_err(javascript_error)?;
    let solid = mesh_to_faceted_brep(&obj.positions, Some(&obj.indices), tolerance)
        .map_err(javascript_error)?;
    serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
}
