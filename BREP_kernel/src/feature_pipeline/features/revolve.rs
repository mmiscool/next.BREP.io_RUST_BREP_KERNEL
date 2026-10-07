//! R — Revolve. Ported from the retired `Revolve` feature.
//!
//! A closed planar profile lying in a half-plane through the axis is swept `angle`
//! degrees about that axis into a solid.
//!
//! Profile + axis both resolve from the scene, HEADLESS (the `extrude.rs`
//! template): `profile` names a SKETCH (`scene.resolve_profile`) or a resident
//! solid FACE (`scene.resolve_face` → `face_profile::face_profile`, the same
//! [`SketchProfile`] shape — a face profile consumes no sketch); `axis` names a
//! SKETCH construction/model line (`scene.resolve_axis`, published by the sketch)
//! or a resident solid EDGE (`scene.resolve_edge` → `common::edge_axis`).
//!
//! `revolve_profile_brep_named` (revolve_topology.rs) stamps the FINAL face names
//! itself, carrying `side_names` across winding normalization + axis-lying-curve
//! skips: `side_names[i] = {edgeName}_RV` (aligned with the outer loop curves);
//! `cap_names = [{id}:L{loopId}_START, {id}:L{loopId}_END]` (a partial revolution
//! only), one pair PER REGION keyed by that region's outer loop's stable identity
//! (see the sketch feature's `loop_ids`) so a cap name never moves when a
//! DIFFERENT loop is edited; the un-keyed `{id}_START/_END` is registered as the
//! CONTAINER standing for all of them. Face order out is
//! `[side faces (non-axis)..., start_cap, end_cap]`.
//!
//! Each profile REGION revolves to its own solid about the SAME oriented axis
//! (region holes revolve the SAME sweep into cutters —
//! `common::subtract_region_holes`, cutter faces stamped `{id}:HOLE:L{loopId}`
//! from the hole loop's own identity, for the same reason)
//! and the region solids are unioned into ONE result. A hole touching/crossing
//! the axis is rejected loudly. The optional `boolean` param folds via
//! `common::finalize_solid`.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{Axis, FeatureContext, FeatureRefusal, FeatureResult, SketchProfile};
use crate::{KernelStage, OrRefuse};
use crate::revolve_profile_brep_named;
use crate::Vec3;
use serde_json::Value;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    // Normalize a `{sketch}:FACE` display-sheet pick to the sketch profile base.
    let name = common::normalize_profile_alias(
        common::first_reference_name(ctx.param("profile"))
            .ok_or("revolve: missing `profile` reference selection")?,
    );
    // Sketch profile first; otherwise a resident FACE becomes a profile via
    // `face_profile`. `from_face` skips the consumed-sketch bookkeeping.
    let mut face_profile_storage: Option<SketchProfile> = None;
    let (profile, from_face): (&SketchProfile, bool) = match ctx.scene.resolve_profile(&name) {
        Some(profile) => (profile, false),
        None => match ctx.scene.resolve_face(&name) {
            Some(face) => {
                let built = super::face_profile::face_profile(face)
                    .map_err(|error| format!("revolve: {error}"))?;
                (&*face_profile_storage.insert(built), true)
            }
            None => {
                return Err(format!(
                    "revolve: profile '{name}' not found (no sketch profile or resident face)"
                ).into());
            }
        },
    };
    let axis_name = common::first_reference_name(ctx.param("axis"))
        .ok_or("revolve: missing `axis` reference selection")?;
    let axis = resolve_axis(ctx, &axis_name)?;
    // Orient the axis so a POSITIVE angle starts the sweep toward the profile
    // plane's +z (`resolveOrientedRevolveAxisDirection` port — the SAME rule the
    // editor's angle widget draws its arc with, so arc and geometry always
    // rotate the same way regardless of which way the axis line was drawn).
    let oriented = orient_axis_toward_profile_front(profile, &axis);

    // `angle` is DEGREES, signed (port `resolveSignedRevolveSweep`): clamp the
    // magnitude to 360, floor at 1e-8 rad, and NEGATE the axis for a negative angle.
    let degrees = ctx.number("angle")?;
    let magnitude = degrees.abs().min(360.0);
    let radians = (magnitude * std::f64::consts::PI / 180.0).max(1e-8);
    let direction = if degrees < 0.0 {
        oriented.scale(-1.0)
    } else {
        oriented
    };

    // FINAL names (the kernel stamps them below the boundary). Caps are
    // `{featureId}_START/_END` for BOTH profile branches — the deliberate Rust
    // respelling of the legacy `sourceProfileBaseName` caps, already re-pinned
    // browser-side ("R13_END is the Rust spelling of the legacy P.CU1_PY_END",
    // test_generated_history_20260725060008). Legacy histories that still
    // reference `{faceName}_START/_END` need the same owner-side re-pin.
    // Per-loop cap names keyed off each region's STABLE loop identity, with the
    // un-keyed `{id}_START/_END` as the containers standing for all of them (see
    // `common::CapNames`). A full revolution emits no caps at all — the names are
    // built either way and simply go unused.
    let caps = common::CapNames::new(&ctx.id, &profile.regions);

    // One revolved solid per REGION (about the SAME oriented axis), unioned
    // below. Each region's holes revolve the SAME sweep into cutters and
    // subtract (both start from the same profile plane, so a partial revolve's
    // cutter caps land coplanar with the outer's caps — the exact boolean's
    // coplanar merge handles them). A hole that touches or crosses the axis
    // would revolve into a self-intersecting "solid", not a cutter — rejected
    // loudly first.
    let mut region_solids = Vec::with_capacity(profile.regions.len());
    for (region_index, region) in profile.regions.iter().enumerate() {
        let outer = region.first().ok_or("revolve: profile has no outer loop")?;
        if outer.curves.len() < 2 {
            return Err(format!(
                "revolve: outer loop needs >= 2 curves, got {}",
                outer.curves.len()
            ).into());
        }
        for (index, hole) in region.iter().enumerate().skip(1) {
            reject_axis_touching_hole(profile, hole, &common::hole_key(hole, index), &axis)?;
        }
        let side_names: Vec<Option<String>> = outer
            .edge_names
            .iter()
            .map(|edge| edge.as_ref().map(|name| format!("{name}_RV")))
            .collect();

        let (start, end) = &caps.per_loop[region_index];
        let cap_names = [Some(start.clone()), Some(end.clone())];
        let mut solid = revolve_profile_brep_named(
            &outer.curves,
            axis.point,
            direction,
            radians,
            &side_names,
            &cap_names,
        )?;

        if region.len() > 1 {
            solid = common::subtract_region_holes(
                solid,
                profile,
                region,
                "revolve",
                &ctx.id,
                None,
                &mut |loop_index, _depth| {
                    // Hole walls carry the hole loop's OWN `{edgeName}_RV` names
                    // (the `revolveProfilesWithHoles` behavior — each hole
                    // revolved with its `faceNames`); caps stay unnamed and pick
                    // up the `{id}:HOLE:{index}` cutter stamp.
                    let hole = &region[loop_index];
                    let hole_side_names: Vec<Option<String>> = hole
                        .edge_names
                        .iter()
                        .map(|edge| edge.as_ref().map(|name| format!("{name}_RV")))
                        .collect();
                    revolve_profile_brep_named(
                        &hole.curves,
                        axis.point,
                        direction,
                        radians,
                        &hole_side_names,
                        &[],
                    )
                    // The revolve stack is still stringly: its text is kept, classed
                    // Internal at this consuming site until it mints its own refusals.
                    .or_refuse(KernelStage::Fragment, "revolve_profile_brep")
                },
            )?;
        }
        region_solids.push(solid);
    }
    let solid = common::union_region_solids(region_solids)
        .map_err(|error| format!("revolve: {error}"))?;

    // Role metadata (sidewall/cap convention): walk the ACTUAL faces so a full
    // revolve (no caps) or a skipped axis wall stamps nothing phantom.
    common::stamp_sweep_roles_multi(&solid, "_RV", &caps.starts(), &caps.ends());

    let mut result = common::finalize_solid_grouped(ctx, solid, &ctx.id, &caps.containers());
    if result.error.is_some() {
        return Ok(result);
    }
    // A FACE profile has no sketch to consume — the source solid stays resident.
    if !from_face && consume_profile_sketch(ctx) {
        let sketch_name = name.strip_suffix(":PROFILE").unwrap_or(&name).to_string();
        if !result.removed.contains(&sketch_name) {
            result.removed.push(sketch_name);
        }
    }
    Ok(result)
}

/// Reject a hole loop that touches or crosses the revolve axis — revolving it
/// would produce a self-intersecting solid of revolution, not a hole cutter.
/// EXACT via the convex-hull property: the signed in-plane distance from the
/// axis line is linear in position, and `{signed >= tol}` is a half-plane
/// (convex), so all control points strictly on one side puts the whole curve
/// strictly on that side.
fn reject_axis_touching_hole(
    profile: &SketchProfile,
    hole: &crate::feature_pipeline::ProfileLoop,
    key: &str,
    axis: &Axis,
) -> Result<(), String> {
    // In-plane direction perpendicular to the axis (the signed-radial axis).
    let side = profile
        .z_axis
        .cross(axis.direction)
        .normalized()
        .map_err(|_| "revolve: axis is perpendicular to the profile plane".to_string())?;
    let mut min_abs = f64::INFINITY;
    let mut has_positive = false;
    let mut has_negative = false;
    for curve in &hole.curves {
        for cp in &curve.control_points {
            let point = cp.point()?;
            let signed = point.sub(axis.point).dot(side);
            if signed > 0.0 {
                has_positive = true;
            }
            if signed < 0.0 {
                has_negative = true;
            }
            min_abs = min_abs.min(signed.abs());
        }
    }
    const AXIS_TOLERANCE: f64 = 1e-9;
    if (has_positive && has_negative) || min_abs <= AXIS_TOLERANCE {
        return Err(format!(
            "revolve: hole loop {key} touches or crosses the revolve axis \
             (min radial distance {min_abs:.3e}) — revolving it would self-intersect; \
             a hole must stay strictly on one side of the axis"
        ).into());
    }
    Ok(())
}

/// `resolveOrientedRevolveAxisDirection` port: flip the axis direction when
/// `cross(axis, radial) · plane_z < 0`, where `radial` is the profile center's
/// offset from the axis (axis component removed) — so a positive angle's
/// initial motion carries the profile toward its plane's +z side. A profile
/// centered ON the axis (degenerate for a revolve anyway) keeps the raw
/// direction. The center is the outer loop's control-point average — the
/// convex-hull property keeps it on the profile's side of the axis.
fn orient_axis_toward_profile_front(profile: &SketchProfile, axis: &Axis) -> Vec3 {
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    let mut count = 0usize;
    if let Some(outer) = profile.regions.first().and_then(|region| region.first()) {
        for curve in &outer.curves {
            for cp in &curve.control_points {
                let w = if cp.w.abs() > 1e-12 { cp.w } else { 1.0 };
                sum = sum.add(Vec3::new(cp.x / w, cp.y / w, cp.z / w));
                count += 1;
            }
        }
    }
    if count == 0 {
        return axis.direction;
    }
    let center = sum.scale(1.0 / count as f64);
    let offset = center.sub(axis.point);
    let radial = offset.sub(axis.direction.scale(offset.dot(axis.direction)));
    if radial.dot(radial) <= 1e-18 {
        return axis.direction;
    }
    if axis.direction.cross(radial).dot(profile.z_axis) < 0.0 {
        axis.direction.scale(-1.0)
    } else {
        axis.direction
    }
}

/// Resolve the axis from a published sketch line or a resident solid edge.
fn resolve_axis(ctx: &FeatureContext, name: &str) -> Result<Axis, String> {
    if let Some(axis) = ctx.scene.resolve_axis(name) {
        return Ok(axis);
    }
    if let Some(edge) = ctx.scene.resolve_edge(name) {
        return common::edge_axis(edge);
    }
    Err(format!(
        "revolve: axis '{name}' not found (no sketch line or resident edge)"
    ).into())
}

/// `consumeProfileSketch` (default true) — remove the sketch after revolving.
fn consume_profile_sketch(ctx: &FeatureContext) -> bool {
    !matches!(ctx.param("consumeProfileSketch"), Some(Value::Bool(false)))
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// needs BOTH a profile (face or sketch, `profile`) AND an axis
/// edge (`axis`) - a profile alone cannot revolve, so it is not offered.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.has_profile() && probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "R",
    "shortName": "R",
    "longName": "Revolve",
    "ribbonPath": "Home/Sweep/Revolve",
    "commandSize": "Large",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the revolve feature"
        },
        "profile": {
            "type": "reference_selection",
            "selectionFilter": [
                "SKETCH",
                "FACE"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Select the profile (face) to revolve"
        },
        "consumeProfileSketch": {
            "type": "boolean",
            "default_value": true,
            "hint": "Remove the referenced sketch after creating the revolve. Turn off to keep it in the scene."
        },
        "axis": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Select the axis to revolve about"
        },
        "angle": {
            "type": "number",
            "default_value": 360,
            "hint": "Revolve angle"
        },
        "boolean": common::optional_boolean_schema()
    }
})
}

