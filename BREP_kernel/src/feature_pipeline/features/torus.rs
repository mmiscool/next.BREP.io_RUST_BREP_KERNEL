//! P.T — Primitive Torus. Ported from the retired `Primitive Torus` feature.
//!
//! The feature params map `mR=majorRadius`, `tR=tubeRadius`, `arcDegrees=arc`.
//!
//! Arc handling (the SIGN handling below diverges — the earlier behavior took
//! `|arc|` and could not reverse):
//! - A falsy `arc` (0, null, missing, NaN) falls back to a FULL turn (360°), not
//!   the partial path.
//! - `sweep` is the arc MAGNITUDE, clamped to `[1e-6, 360]`; it builds the geometry.
//! - `full = |sweep - 360| <= 1e-9`. When full, we call
//!   `make_torus_brep((0,0,0),(0,1,0), major, minor)` — direction is irrelevant.
//! - When NOT full, we revolve the tube-circle profile — four quarter arcs centered
//!   at `(majorRadius, 0, 0)` in the X/Y plane — about the +Y axis through the
//!   origin by `sweep` radians via [`crate::revolve_profile_brep_named`], passing
//!   the names applied POSITIONALLY.
//! - A NEGATIVE `arc` reverses the sweep DIRECTION: the +|arc| solid is spun a half
//!   turn about X (`common::rotate_half_turn_about_x`) onto the opposite side. The
//!   revolve helper only accepts a positive angle in `(0, 2π]`, so the sign never
//!   reaches it.
//!
//! Name fidelity (contract rule 2): the full torus is a single analytic face; the
//! solid AND its one face are both named `${featureID}_Side`. The partial torus
//! has 6 faces named
//! `[`${featureID}_Side` ×4, `${featureID}_Cap0`, `${featureID}_Cap1`]` — all four
//! revolved tube-quarters share the SAME `_Side` name, and `Cap0`/`Cap1` are the
//! start/end caps (metadata roles `startCap`/`endCap`, matching
//! `revolve_profile_brep_named`'s `cap_names = [start, end]` order).

use crate::feature_pipeline::features::{common, transform_bake};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_arc, make_torus_brep, revolve_profile_brep_named, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // majorRadius = |mR|, tubeRadius = |tR|.
    let major_radius = ctx.number("majorRadius")?.abs();
    let tube_radius = ctx.number("tubeRadius")?.abs();

    // sweep = max(1e-6, min(360, |Number(arc) || 360|)) — the `|| 360`
    // fallback fires for a falsy arc (0 / null / missing / non-finite).
    let raw = match ctx.param("arc") {
        None | Some(serde_json::Value::Null) => 360.0,
        Some(_) => ctx.number("arc")?,
    };
    let raw = if !raw.is_finite() || raw == 0.0 { 360.0 } else { raw };
    // The sweep MAGNITUDE builds the geometry (`revolve_profile_brep_named` requires
    // a positive angle in (0, 2π]); a NEGATIVE arc reverses the sweep DIRECTION,
    // realized by a half-turn flip after the build. A full turn is direction-
    // agnostic, so the sign is moot there.
    let sweep = raw.abs().min(360.0).max(1e-6);
    let full = (sweep - 360.0).abs() <= 1e-9;
    let reverse = raw < 0.0 && !full;

    let solid = if full {
        let mut solid = make_torus_brep(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            major_radius,
            tube_radius,
        )?;

        // Single analytic face, named `${featureID}_Side`.
        let face = solid
            .shells
            .get_mut(0)
            .and_then(|shell| shell.faces.get_mut(0))
            .ok_or("torus builder produced no face")?;
        face.name = Some(format!("{}_Side", ctx.id));
        solid
    } else {
        // Partial arc: revolve the tube circle — four quarter arcs centered at
        // `(majorRadius, 0, 0)` in the X/Y plane — about
        // the +Y axis through the origin by `sweep` radians.
        let tube_center = Vec3::new(major_radius, 0.0, 0.0);
        let x_axis = Vec3::new(1.0, 0.0, 0.0);
        let y_axis = Vec3::new(0.0, 1.0, 0.0);
        let quarter = std::f64::consts::FRAC_PI_2;
        let profile = [
            make_arc(tube_center, x_axis, y_axis, tube_radius, 0.0, quarter)?,
            make_arc(tube_center, x_axis, y_axis, tube_radius, quarter, 2.0 * quarter)?,
            make_arc(tube_center, x_axis, y_axis, tube_radius, 2.0 * quarter, 3.0 * quarter)?,
            make_arc(tube_center, x_axis, y_axis, tube_radius, 3.0 * quarter, 4.0 * quarter)?,
        ];
        // Named positionally: all four revolved quarters are `${name}_Side`,
        // then the start/end caps are `${name}_Cap0` / `${name}_Cap1`.
        let side_names = vec![Some(format!("{}_Side", ctx.id)); 4];
        let cap_names = [
            Some(format!("{}_Cap0", ctx.id)),
            Some(format!("{}_Cap1", ctx.id)),
        ];
        revolve_profile_brep_named(
            &profile,
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            sweep.to_radians(),
            &side_names,
            &cap_names,
        )?
    };

    // A negative arc spins the partial torus a half turn about X, sweeping it to the
    // opposite side (proper rotation → names + winding preserved; the tube-circle
    // profile is invariant under the half turn, so this is exactly the -|arc| sweep).
    let solid = if reverse {
        common::rotate_half_turn_about_x(solid)?
    } else {
        solid
    };

    // Bake the `transform` param BEFORE the boolean; the bake preserves the
    // stamped names (geometry-only rewrite).
    let solid = transform_bake::apply(ctx, solid)?;

    Ok(common::finalize_solid(ctx, solid, &ctx.id))
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// never offered from a selection (no reference inputs).
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "P.T",
    "shortName": "P.T",
    "longName": "Primitive Torus",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the feature"
        },
        "majorRadius": {
            "type": "number",
            "default_value": 10,
            "hint": "Distance from center to the centerline of the tube (R)"
        },
        "tubeRadius": {
            "type": "number",
            "default_value": 2,
            "hint": "Radius of the tube (r)"
        },
        "arc": {
            "type": "number",
            "default_value": 360,
            "hint": "Sweep angle of the torus in degrees [-360, 360]; a negative angle reverses the sweep direction"
        },
        "transform": common::primitive_transform_schema(),
        "boolean": common::optional_boolean_schema()
    }
})
}

