//! The feature-schema catalogue — kernel-owned feature DEFINITIONS.
//!
//! Each feature's schema lives WITH its own code (`features/<feat>.rs` exposes
//! `schema()` — name, display-builder flag, `inputParamsSchema` — exactly the
//! shape the caller's feature classes declared). This module only AGGREGATES them, in
//! the registry's presentation order, and serves the catalogue to the caller's feature editor
//! via [`feature_schemas_json`]. The caller's feature registry initializes from this;
//! there is no caller-side schema. Function-valued UI fields (button actions,
//! custom widgets) cannot cross the JSON boundary — the caller's hook code merges
//! those back per-param at registry init.

use wasm_bindgen::prelude::*;

/// Assemble the catalogue from every feature's own `schema()`. The assembly
/// constraint schemas join the export under their own `assemblyConstraints`
/// namespace so the dialog engine renders them with the same machinery.
pub fn feature_schema_catalogue() -> serde_json::Value {
    serde_json::json!({
        "version": 1,
        "assemblyConstraints": super::assembly::constraint_schema_catalogue(),
        "features": [
            super::features::datum::schema(),
            super::features::plane::schema(),
            super::features::cube::schema(),
            super::features::cylinder::schema(),
            super::features::cone::schema(),
            super::features::sphere::schema(),
            super::features::torus::schema(),
            super::features::pyramid::schema(),
            super::features::import3d::schema(),
            super::features::sketch::schema(),
            super::features::spline::schema(),
            super::features::waypoint::schema(),
            super::features::helix::schema(),
            super::features::extrude::schema(),
            super::features::boolean::schema(),
            super::features::fillet::schema(),
            super::features::chamfer::schema(),
            super::features::offset_shell::schema(),
            super::features::offset_face::schema(),
            super::features::push_face::schema(),
            super::features::transform_face::schema(),
            super::features::delete_face::schema(),
            super::features::refit_faces::schema(),
            super::features::thicken::schema(),
            super::features::sheet_metal_tab::schema(),
            super::features::sheet_metal_contour_flange::schema(),
            super::features::sheet_metal_flange::schema(),
            super::features::sheet_metal_hem::schema(),
            super::features::sheet_metal_corner::schema_fillet(),
            super::features::sheet_metal_corner::schema_chamfer(),
            super::features::sheet_metal_cutout::schema(),
            super::features::loft::schema(),
            super::features::mirror::schema(),
            super::features::split::schema(),
            super::features::revolve::schema(),
            super::features::rib::schema(),
            super::features::sweep::schema(),
            super::features::path_sweep::schema(),
            super::features::hole::schema(),
            super::features::tube::schema(),
            super::features::transform::schema(),
            super::features::pattern::schema(),
            super::features::assembly_component::schema()
        ]
    })
}

/// wasm boundary: the whole feature-schema catalogue as JSON.
#[wasm_bindgen]
pub fn feature_schemas_json() -> String {
    feature_schema_catalogue().to_string()
}

/// The DIALOG FIELD-VISIBILITY hook, kernel-owned like the schemas themselves.
///
/// Given a feature's `type` (its `shortName`, e.g. `"H"`) and the dialog's
/// CURRENT param values, return the param keys that should be HIDDEN for those
/// values — the choke point every schema-driven dialog consults to decide which
/// fields to draw. A feature opts in by exposing its own `hidden_params(values)`
/// beside its `schema()` (see `features::hole::hidden_params`); the mechanism is
/// uniform across every dialog, and a feature that declares no hook (or an
/// unknown type) hides nothing, so all its fields stay visible.
///
/// The form engine is immediate-mode and re-runs this every frame against the
/// live values, which is what makes one pure function serve both "the dialog was
/// just opened" and "a field just changed" — there is no separate event to wire.
pub fn feature_hidden_params(feature_type: &str, values: &serde_json::Value) -> Vec<String> {
    let keys: Vec<&'static str> = match feature_type {
        "H" => super::features::hole::hidden_params(values),
        _ => Vec::new(),
    };
    keys.into_iter().map(str::to_owned).collect()
}

