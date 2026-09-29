//! Sheet-metal tab: a single closed sketch or planar face profile becomes
//! the root flat of a [`SheetTree`], evaluated at full fold. Disjoint regions,
//! including islands inside holes, are rejected.
//!
//! [`outline_from_outer_loop`] projects exact curves into the profile frame,
//! orients them CCW, and splits short loops to form a polygon skeleton.
//! Curved boundaries remain exact through `Flat::outline_curves`; flanges
//! and hems require straight outline edges. Inner loops become `Flat::holes`
//! and survive later folding, cutting, and unfolding.
//!
//! `thickness` sets the extrusion distance. `placementMode` defaults to
//! `forward` (+normal), with `reverse` and `midplane` alternatives. A face
//! profile uses its outward normal. `bendRadius` and `neutralFactor` become
//! tree defaults for later operations. Sketch profiles are consumed; source
//! solids used for face profiles remain resident.


use std::collections::BTreeMap;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::sheet_metal::{self, Edge, Flat, SheetTree};
use crate::feature_pipeline::{FeatureContext, FeatureResult, ProfileLoop, SketchProfile};
use crate::NurbsCurve;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // Resolve sketch aliases first, then fall back to a planar face profile.
    let name = common::normalize_profile_alias(
        common::first_reference_name(ctx.param("profile"))
            .ok_or("sheet-metal tab: missing `profile` reference selection")?,
    );
    let mut face_profile_storage: Option<SketchProfile> = None;
    let (profile, from_face): (&SketchProfile, bool) = match ctx.scene.resolve_profile(&name) {
        Some(profile) => (profile, false),
        None => match ctx.scene.resolve_face(&name) {
            Some(face) => {
                let built = super::face_profile::face_profile(face)
                    .map_err(|error| format!("sheet-metal tab: {error}"))?;
                (&*face_profile_storage.insert(built), true)
            }
            None => {
                return Err(format!(
                    "sheet-metal tab: profile '{name}' not found (no sketch profile or resident face)"
                ));
            }
        },
    };

    // A single region maps to a tab (no-fallback): a tab roots ONE sheet-metal
    // bend tree, so disjoint regions are geometric nonsense here (sketch one
    // tab per region — an island inside a hole classifies as its own region
    // and lands here too). Interior HOLE loops are welcome: they bake into the
    // tree as exact local-plane curve loops below.
    if profile.regions.len() > 1 {
        return Err(
            "sheet-metal tab: profile has multiple disjoint regions; a tab roots one \
             sheet-metal body — sketch one tab per region"
                .into(),
        );
    }
    let region = profile
        .regions
        .first()
        .ok_or("sheet-metal tab: profile has no outer loop")?;
    let outer = region.first().ok_or("sheet-metal tab: profile has no outer loop")?;

    // Outer loop → CCW polygon skeleton + exact-curve overrides for the curved
    // segments + the segment Edge records (ids `{feature}:e{i}`).
    let (outline, edges, outline_curves) = outline_from_outer_loop(&ctx.id, profile, outer)?;

    let thickness = ctx.number("thickness")?;
    if !(thickness > 0.0) {
        return Err(format!("sheet-metal tab: thickness must be positive, got {thickness}"));
    }
    let bend_radius = ctx.number("bendRadius").unwrap_or(0.0).max(0.0);
    let k_factor = ctx.number("neutralFactor").unwrap_or(0.5).clamp(0.0, 1.0);
    let mode = ctx
        .string("placementMode")
        .unwrap_or_else(|| "forward".to_string());

    // Profile plane → world root frame, offset along the normal per placement mode.
    let origin = profile.origin;
    let x_axis = profile.x_axis;
    let y_axis = profile.y_axis;
    let normal = profile.z_axis;
    let half_t = thickness * 0.5;
    let shift = match mode.as_str() {
        "reverse" => -half_t,
        "midplane" => 0.0,
        _ => half_t, // forward (default): material on the +normal side
    };
    let placed_origin = origin.add(normal.scale(shift));
    let root_transform = [
        [placed_origin.x, placed_origin.y, placed_origin.z],
        [x_axis.x, x_axis.y, x_axis.z],
        [y_axis.x, y_axis.y, y_axis.z],
        [normal.x, normal.y, normal.z],
    ];

    // Store exact holes in the tree so every later evaluation retains them.
    let holes = region
        .iter()
        .skip(1)
        .map(|hole| common::profile_loop_uv_curves(profile, hole).map(sheet_metal::Hole::through))
        .collect::<Result<Vec<_>, _>>()?;
    let tree = SheetTree {
        thickness,
        root: Flat {
            id: format!("{}:flat_root", ctx.id),
            outline,
            edges,
            holes,
            hole_bends: Vec::new(),
            outline_curves,
        },
        default_inside_radius: bend_radius,
        default_k_factor: k_factor,
        root_transform,
    };

    let added = sheet_metal::register_folded(tree, &ctx.id)?;
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.added.push(added);
    // Consume sketch display groups; face-profile source solids remain resident.
    if !from_face {
        common::consume_sketch_always(&name, &mut result);
    }
    Ok(result)
}

/// Project exact curves into the profile plane and orient by sampled area.
/// Split every curve at mid-domain until there are at least three segments
/// (1 → 2 → 4, 2 → 4), fixing circular tabs' e0..e3 naming. Segment starts
/// form the skeleton; curved segments retain exact per-edge curve overrides.
fn outline_from_outer_loop(
    feature_id: &str,
    profile: &SketchProfile,
    outer: &ProfileLoop,
) -> Result<(Vec<[f64; 2]>, Vec<Edge>, BTreeMap<String, NurbsCurve>), String> {
    let mut curves = common::profile_loop_uv_curves(profile, outer)?;
    if curves.is_empty() {
        return Err("sheet-metal tab: profile outer loop has no curves".into());
    }
    let area = sampled_loop_area(&curves)?;
    if !(area.abs() > 1e-12) {
        return Err("sheet-metal tab: profile outer loop encloses no area".into());
    }
    // The flat outline must wind CCW — the evaluator takes "right of a→b" as
    // the outward side (evaluate.rs), so a CW profile would build inside-out.
    if area < 0.0 {
        curves = curves
            .iter()
            .rev()
            .map(NurbsCurve::reversed)
            .collect::<Result<_, _>>()?;
    }
    while curves.len() < 3 {
        let mut split = Vec::with_capacity(curves.len() * 2);
        for curve in &curves {
            let [t0, t1] = curve.domain()?;
            let (first, second) = curve.split((t0 + t1) * 0.5)?;
            split.push(first);
            split.push(second);
        }
        curves = split;
    }

    let mut outline = Vec::with_capacity(curves.len());
    let mut edges = Vec::with_capacity(curves.len());
    let mut outline_curves = BTreeMap::new();
    for (i, curve) in curves.iter().enumerate() {
        let domain = curve.domain()?;
        let start = curve.evaluate(domain[0])?;
        outline.push([start.x, start.y]);
        let id = format!("{feature_id}:e{i}");
        if !curve_is_straight(curve)? {
            outline_curves.insert(id.clone(), curve.clone());
        }
        edges.push(Edge { id, bend: None });
    }
    Ok((outline, edges, outline_curves))
}

/// Signed area of a closed curve loop, from 64 samples per curve (shoelace).
/// Positive = CCW in the local `(u, v)` plane.
fn sampled_loop_area(curves: &[NurbsCurve]) -> Result<f64, String> {
    let mut points = Vec::with_capacity(curves.len() * 64);
    for curve in curves {
        let [t0, t1] = curve.domain()?;
        for step in 0..64 {
            let p = curve.evaluate(t0 + (t1 - t0) * step as f64 / 64.0)?;
            points.push([p.x, p.y]);
        }
    }
    let mut area2 = 0.0;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        area2 += a[0] * b[1] - b[0] * a[1];
    }
    Ok(area2 * 0.5)
}

/// True iff a curve is (numerically) straight — every interior sample lies on
/// the chord between its endpoints (7 interior samples: a single mid-domain
/// probe would pass an S-curve crossing its chord at mid). Deciding the
/// straight-segment vs exact-override split, so it biases toward "curved":
/// a false "curved" is still exact geometry, a false "straight" silently
/// chords.
fn curve_is_straight(curve: &NurbsCurve) -> Result<bool, String> {
    let domain = curve.domain()?;
    let start = curve.evaluate(domain[0])?;
    let end = curve.evaluate(domain[1])?;
    let chord = end.sub(start);
    let chord_len = chord.length();
    for step in 1..8 {
        let sample = curve.evaluate(domain[0] + (domain[1] - domain[0]) * step as f64 / 8.0)?;
        if chord_len <= 1e-9 {
            // Zero-length chord: straight only if the whole curve is one point
            // (a CLOSED single-curve loop lands here and is decidedly curved).
            if sample.sub(start).length() > 1e-9 {
                return Ok(false);
            }
            continue;
        }
        let t = sample.sub(start).dot(chord) / (chord_len * chord_len);
        let projection = start.add(chord.scale(t));
        if sample.sub(projection).length() > 1e-6 * chord_len.max(1.0) {
            return Ok(false);
        }
    }
    Ok(true)
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected profile (face or sketch) drives `profile`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.has_profile()
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SM.TAB",
    "shortName": "SM.TAB",
    "longName": "SM Tab",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the sheet metal tab"
        },
        "profile": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "SKETCH"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Closed sketch or face defining the tab footprint"
        },
        "thickness": {
            "type": "number",
            "default_value": 1,
            "min": 0,
            "hint": "Sheet metal thickness. Also used as the tab extrusion distance."
        },
        "placementMode": {
            "type": "options",
            "options": [
                "forward",
                "reverse",
                "midplane"
            ],
            "default_value": "forward",
            "hint": "Controls whether material is added forward, backward, or split about the sketch plane."
        },
        "bendRadius": {
            "type": "number",
            "default_value": 0.125,
            "min": 0,
            "hint": "Default bend radius captured with the sheet-metal base feature."
        },
        "neutralFactor": {
            "type": "number",
            "default_value": 0.5,
            "min": 0,
            "max": 1,
            "step": 0.01,
            "hint": "Neutral factor used for flat pattern bend allowance (0-1)."
        }
    }
})
}

