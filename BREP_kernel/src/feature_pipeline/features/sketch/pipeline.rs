use super::*;
use crate::feature_pipeline::ScenePoint;
use super::chain::{chain_segments, emit_chain, plan_length};
use super::frame::{pre_evaluate_expressions, resolve_frame};
use super::geometry::{circle_loop, ellipse_loop, Loop, SegKind, Segment, SegmentIds};
use super::project::projected_reference_paths;
use super::regions::{assemble_loops, classify_regions, emit_loop};

/// What a sketch produces: an optional profile (none for an axis-only/open/empty
/// sketch), any unresolved plane reference, the named axis lines it publishes, an
/// optional path chain (its ordered geometry, open or closed) for sweep/rib, and
/// its resolved plane frame (published under the sketch id).
struct SketchOutput {
    profile: Option<SketchProfile>,
    unresolved: Vec<String>,
    axes: Vec<(String, Axis)>,
    path: Option<Vec<NurbsCurve>>,
    /// The SOURCE NAME (`{id}:G{gid}`) of each curve in `path`, in the same
    /// order — the chain's segments come from named model geometries, and a
    /// per-SEGMENT consumer (the sweep, which builds one portion per path
    /// segment) names the faces of each portion after the segment that made it.
    /// `None` for a segment with no named geometry behind it.
    path_segment_names: Option<Vec<Option<String>>>,
    /// One world curve per NAMED model segment, published under its own
    /// `{id}:G{gid}` name, so a per-EDGE consumer (the tube) resolves EACH picked
    /// sketch edge to its own curve. A single ordered `path` chain cannot
    /// represent a BRANCHING sketch (a Y-junction has no one chain), so per-edge
    /// resolution is what makes 3-way-junction tubes work.
    per_geometry: Vec<(String, Vec<NurbsCurve>)>,
    /// Projected reference paths under `{id}:REF:{source}` — model edges
    /// (component edges included) projected onto the sketch plane as read-only
    /// construction input (see `project.rs`).
    projected: Vec<(String, Vec<NurbsCurve>)>,
    frame: Frame,
    points: Vec<(String, ScenePoint)>,
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(output) => {
            let mut result = FeatureResult::pass_through(ctx.id.clone(), ctx.feature_type.clone());
            if let Some(profile) = output.profile {
                result.profiles.push((ctx.id.clone(), profile));
            }
            if let Some(path) = output.path {
                result.paths.push((ctx.id.clone(), path));
            }
            if let Some(names) = output.path_segment_names {
                result.path_segment_names.push((ctx.id.clone(), names));
            }
            // Per-geometry paths under `{id}:G{gid}` (see [`SketchOutput::per_geometry`]).
            for entry in output.per_geometry {
                result.paths.push(entry);
            }
            // Projected reference paths under `{id}:REF:{source}` (read-only
            // in-context construction input; see [`SketchOutput::projected`]).
            for entry in output.projected {
                result.paths.push(entry);
            }
            // Publish the resolved plane frame under the sketch id, so a consumer
            // of an OPEN sketch (SM.CF path, sweep trajectory) gets the authored
            // plane instead of re-deriving one from the path (sign-ambiguous).
            result.frames.push((ctx.id.clone(), output.frame));
            result.axes = output.axes;
            result.points = output.points;
            result.unresolved = output.unresolved;
            result
        }
        Err(error) => ctx.fail(error),
    }
}

// ---------------------------------------------------------------------------
// Solve + geometry reconstruction
// ---------------------------------------------------------------------------

/// A canonical key for a point id: byte-matches `Map` key identity (number `3` and
/// string `"3"` are DISTINCT keys, exactly as `pointById.get(g.points[k])` sees).
fn point_key(value: &Value) -> String {
    value.to_string()
}

fn build(ctx: &FeatureContext) -> Result<SketchOutput, String> {
    let (frame, mut unresolved) = resolve_frame(ctx)?;

    // Projected reference geometry, resolved BEFORE the fresh-sketch early
    // return so a brand-new sketch already shows its references on first edit
    // (the same reason the frame publishes there).
    let projected = projected_reference_paths(ctx, &frame, &mut unresolved)?;

    // A brand-new sketch has NO `persistentData.sketch` yet: the app's `add_feature`
    // seeds an empty `persistentData: {}`, and only the FIRST commit (`exit_sketch_mode`)
    // writes the doc. That is a legitimate fresh-sketch state, not an error — there is
    // no geometry to extract, but the resolved plane FRAME must STILL publish (via
    // `execute` → `result.frames`, the Ok path) so the live sketcher + downstream
    // consumers find the sketch's plane on the VERY FIRST edit. Without it,
    // `enter_sketch_mode` has no resolved frame in `construction_frames` to seed the
    // session, so the empty sketcher opens at the world origin instead of on its
    // face/datum — the off-location live plane on a newly-created sketch's first entry.
    // A PRESENT-but-corrupt doc (a non-object `sketch`) still errors loudly (no silent
    // fallback on bad data).
    let mut sketch = match ctx.persistent.get("sketch") {
        Some(value) if value.is_object() => value.clone(),
        Some(value) => {
            return Err(format!(
                "sketch: `persistentData.sketch` must be an object, got {value}"
            ));
        }
        None => {
            return Ok(SketchOutput {
                profile: None,
                unresolved,
                axes: Vec::new(),
                path: None,
                path_segment_names: None,
                per_geometry: Vec::new(),
                projected,
                frame,
                points: Vec::new(),
            });
        }
    };

    pre_evaluate_expressions(ctx, &mut sketch)?;
    pin_origin_point(&mut sketch, &frame);

    // Solve to final coordinates (committed solve: exact, polished).
    let request = SolveSketchRequest {
        sketch,
        iterations: Some(2000),
        remove_implied_duplicates: true,
        tolerance: None,
        distance_slide_threshold_ratio: None,
        distance_slide_step_ratio: None,
        distance_slide_min_step: None,
        polish: Some(true),
    };
    let solved = solve_sketch(&request).map_err(|error| format!("sketch: solve failed: {error}"))?;
    let solved_sketch = solved
        .get("sketch")
        .ok_or("sketch: solver returned no sketch")?;

    // id -> solved (x, y).
    let mut points: std::collections::HashMap<String, [f64; 2]> = std::collections::HashMap::new();
    if let Some(array) = solved_sketch.get("points").and_then(|p| p.as_array()) {
        for point in array {
            let (Some(id), Some(x), Some(y)) = (
                point.get("id"),
                point.get("x").and_then(|v| v.as_f64()),
                point.get("y").and_then(|v| v.as_f64()),
            ) else {
                continue;
            };
            points.insert(point_key(id), [x, y]);
        }
    }
    let point_of = |id: &Value| -> Option<[f64; 2]> { points.get(&point_key(id)).copied() };

    // Publish every solved point as a named world point `{sketchId}:P{pid}`
    // (hole centers / placement references resolve these from the scene, the
    // committed-sketch display draws them). The point's own `construction` flag
    // rides along: a construction point (the sketcher's ◐ toggle, a materialized
    // external reference) still resolves by name but is NOT model geometry, so a
    // placement consumer skips it — the point analogue of a construction line,
    // which publishes as an axis yet never models a profile edge.
    let mut published_points: Vec<(String, ScenePoint)> = Vec::new();
    if let Some(array) = solved_sketch.get("points").and_then(|p| p.as_array()) {
        for point in array {
            let (Some(id), Some(x), Some(y)) = (
                point.get("id"),
                point.get("x").and_then(|v| v.as_f64()),
                point.get("y").and_then(|v| v.as_f64()),
            ) else {
                continue;
            };
            // The part-origin reference exists only inside the sketch.
            if is_sketch_origin_point_id(id) {
                continue;
            }
            let construction = point.get("construction").and_then(|v| v.as_bool()) == Some(true);
            published_points.push((
                format!("{}:P{}", ctx.id, id_display(id)),
                ScenePoint { position: frame.to_3d(x, y), construction },
            ));
        }
    }

    // Reconstruct model geometry: straight/arc SEGMENTS get chained into loops;
    // whole CIRCLES and ELLIPSES are self-contained loops emitted directly.
    // Every LINE (incl. construction) is ALSO published as a named axis line (a
    // revolve/sweep axis is commonly a construction line).
    let mut segments: Vec<Segment> = Vec::new();
    let mut whole_loops: Vec<Loop> = Vec::new();
    let mut axes: Vec<(String, Axis)> = Vec::new();
    if let Some(geometries) = solved_sketch.get("geometries").and_then(|g| g.as_array()) {
        for geometry in geometries {
            let geom_type = geometry.get("type").and_then(|v| v.as_str()).unwrap_or("");
            let ids = geometry.get("points").and_then(|p| p.as_array());
            // Source edge name `{sketchId}:G{gid}` — the
            // string a downstream reference_selection stores; sidewalls stamp `{name}_E`.
            let edge_name = geometry
                .get("id")
                .map(|id| format!("{}:G{}", ctx.id, id_display(id)));
            // The geometry's own numeric id + the loop id a previous commit
            // persisted on it — the raw material for the loop's STABLE identity
            // (see `loop_ids`). Both are optional: a non-numeric id or a
            // never-committed geometry simply carries `None`.
            let segment_ids = SegmentIds {
                gid: geometry.get("id").and_then(numeric_id),
                loop_id: geometry.get("loopId").and_then(numeric_id),
            };

            // Publish a line (construction included) as a named axis line.
            if geom_type == "line" {
                if let Some(ids) = ids {
                    if ids.len() == 2 {
                        if let (Some(a), Some(b)) = (point_of(&ids[0]), point_of(&ids[1])) {
                            let point = frame.to_3d(a[0], a[1]);
                            if let Ok(direction) = frame.to_3d(b[0], b[1]).sub(point).normalized() {
                                if let Some(name) = edge_name.clone() {
                                    axes.push((name, Axis { point, direction }));
                                }
                            }
                        }
                    }
                }
            }

            // Construction geometry constrains but never models a profile edge.
            if geometry.get("construction").and_then(|v| v.as_bool()) == Some(true) {
                continue;
            }
            match (geom_type, ids) {
                ("line", Some(ids)) if ids.len() == 2 => {
                    let (Some(a), Some(b)) = (point_of(&ids[0]), point_of(&ids[1])) else {
                        continue;
                    };
                    segments.push(Segment {
                        a,
                        b,
                        kind: SegKind::Line,
                        name: edge_name,
                        ids: segment_ids,
                    });
                }
                ("arc", Some(ids)) if ids.len() == 3 => {
                    let (Some(center), Some(start), Some(end)) =
                        (point_of(&ids[0]), point_of(&ids[1]), point_of(&ids[2]))
                    else {
                        continue;
                    };
                    let radius = (start[0] - center[0]).hypot(start[1] - center[1]);
                    if radius <= JOIN_TOL {
                        return Err("sketch: degenerate arc (zero radius)".into());
                    }
                    let start_angle = (start[1] - center[1]).atan2(start[0] - center[0]);
                    let end_angle = (end[1] - center[1]).atan2(end[0] - center[0]);
                    // CCW sweep into [0, 2π); start≈end ⇒ full circle (legacy:882-884).
                    let mut sweep = (end_angle - start_angle).rem_euclid(TAU);
                    if sweep < 1e-6 {
                        sweep = TAU;
                    }
                    segments.push(Segment {
                        a: start,
                        b: end,
                        kind: SegKind::Arc {
                            center,
                            radius,
                            start_angle,
                            sweep,
                        },
                        name: edge_name,
                        ids: segment_ids,
                    });
                }
                ("circle", Some(ids)) if ids.len() == 2 => {
                    let (Some(center), Some(rim)) = (point_of(&ids[0]), point_of(&ids[1])) else {
                        continue;
                    };
                    let radius = (rim[0] - center[0]).hypot(rim[1] - center[1]);
                    if radius <= JOIN_TOL {
                        return Err("sketch: degenerate circle (zero radius)".into());
                    }
                    whole_loops.push(circle_loop(center, radius, edge_name, segment_ids));
                }
                ("ellipse", Some(ids)) if ids.len() >= 3 => {
                    // Points: [center, majorAxisEnd, minorAxisEnd] (legacy:1052-1069).
                    // û = major direction; v̂ ⟂ û; a = |majorEnd − center|;
                    // b = the PROJECTION of (minorEnd − center) onto v̂.
                    let (Some(center), Some(major_end), Some(minor_end)) =
                        (point_of(&ids[0]), point_of(&ids[1]), point_of(&ids[2]))
                    else {
                        continue;
                    };
                    let mut ux = major_end[0] - center[0];
                    let mut uy = major_end[1] - center[1];
                    let a = ux.hypot(uy);
                    if a <= JOIN_TOL {
                        return Err("sketch: degenerate ellipse (zero-length major axis)".into());
                    }
                    ux /= a;
                    uy /= a;
                    let (px, py) = (-uy, ux);
                    let b = ((minor_end[0] - center[0]) * px + (minor_end[1] - center[1]) * py)
                        .abs();
                    if b <= JOIN_TOL {
                        return Err(
                            "sketch: degenerate ellipse (minor axis end collinear with the major axis)"
                                .into(),
                        );
                    }
                    whole_loops.push(ellipse_loop(
                        center,
                        [a * ux, a * uy],
                        [b * px, b * py],
                        edge_name,
                        segment_ids,
                    ));
                }
                ("bezier", Some(ids)) if ids.len() >= 4 && (ids.len() - 1) % 3 == 0 => {
                    let mut controls = Vec::with_capacity(ids.len());
                    for id in ids {
                        let Some(point) = point_of(id) else {
                            return Err("sketch: bezier references a missing point".into());
                        };
                        controls.push(point);
                    }
                    let a = controls[0];
                    let b = *controls.last().unwrap();
                    segments.push(Segment {
                        a,
                        b,
                        kind: SegKind::Bezier { controls },
                        name: edge_name,
                        ids: segment_ids,
                    });
                }
                ("line" | "arc" | "circle" | "bezier" | "ellipse", _) => {
                    // Right type, wrong point count — a corrupt geometry.
                    return Err(format!("sketch: malformed '{geom_type}' geometry"));
                }
                (other, _) => {
                    return Err(format!(
                        "sketch: profile geometry type '{other}' is not yet migrated to the Rust pipeline"
                    ));
                }
            }
        }
    }

    // Per-geometry world curves (one per NAMED model segment), published so a
    // per-EDGE consumer (the tube) resolves each picked sketch edge to its OWN
    // curve — the ordered `path` chain below cannot represent a branching sketch.
    // Built from `&segments` BEFORE they are moved into `build_profile`.
    let mut per_geometry: Vec<(String, Vec<NurbsCurve>)> = Vec::new();
    for segment in &segments {
        if let Some(name) = &segment.name {
            per_geometry.push((name.clone(), vec![segment.to_curve(&frame, false)?]));
        }
    }

    // Chain the model segments into maximal endpoint-connected runs, then split
    // by closure (loops and open chains built side by side): closed runs feed the
    // PROFILE, open runs
    // feed the PATH. Mixed sketches (a closed profile plus a separate open
    // sweep trajectory) are legal; neither invalidates the other.
    let mut loop_plans: Vec<Vec<(usize, bool)>> = Vec::new();
    let mut open_plans: Vec<Vec<(usize, bool)>> = Vec::new();
    for chain in chain_segments(&segments) {
        if chain.closed {
            if chain.entries.len() < 2 {
                return Err("sketch: degenerate profile loop (< 2 edges)".into());
            }
            loop_plans.push(chain.entries);
        } else {
            open_plans.push(chain.entries);
        }
    }

    // PATH: the longest open chain wins (a sweep trajectory drawn alongside a
    // profile); with no open geometry a LONE closed chain doubles as the path
    // (a closed sketch is a legal sweep path too — existing behavior).
    let path_plan: Option<(&Vec<(usize, bool)>, bool)> = if let Some(plan) = open_plans
        .iter()
        .max_by(|a, b| plan_length(a, &segments).total_cmp(&plan_length(b, &segments)))
    {
        Some((plan, false))
    } else if loop_plans.len() == 1 {
        Some((&loop_plans[0], true))
    } else {
        None
    };
    let mut path = None;
    let mut path_segment_names = None;
    if let Some((plan, closed)) = path_plan {
        path = Some(emit_chain(plan, &segments, &frame, closed)?);
        // The published chain's per-segment SOURCE names, in chain order — what
        // lets the sweep name each portion after the path segment that built it.
        path_segment_names = Some(
            plan.iter()
                .map(|&(index, _)| segments[index].name.clone())
                .collect(),
        );
    }

    // PROFILE from the closed loops (+ whole circles/ellipses). No closed loop
    // at all is an open/axis-only/empty sketch — no profile, not an error (the
    // path and axes above still publish). The loops group into disjoint REGIONS
    // by even-odd containment depth (outer + holes per region).
    let profile = build_profile(segments, loop_plans, whole_loops, &frame)?;

    Ok(SketchOutput {
        profile,
        unresolved,
        axes,
        path,
        path_segment_names,
        per_geometry,
        projected,
        frame,
        points: published_points,
    })
}

/// Build the closed-loop profile from the planned loops + circle/ellipse loops.
/// `Ok(None)` when the sketch has no closed loop at all (open/axis-only/empty).
fn build_profile(
    segments: Vec<Segment>,
    loop_plans: Vec<Vec<(usize, bool)>>,
    mut whole_loops: Vec<Loop>,
    frame: &Frame,
) -> Result<Option<SketchProfile>, String> {
    let mut loops = assemble_loops(segments, loop_plans);
    loops.append(&mut whole_loops);
    if loops.is_empty() {
        return Ok(None);
    }
    // Decide each loop's STABLE identity over the sketch as a whole, BEFORE the
    // region grouping — a loop's id must not depend on how the loops happen to
    // nest (see `loop_ids`). The cap/hole names downstream embed it.
    super::loop_ids::resolve(&mut loops);
    let mut regions: Vec<Vec<ProfileLoop>> = Vec::new();
    for region in classify_regions(loops)? {
        let mut emitted = Vec::with_capacity(region.len());
        for loop_data in &region {
            emitted.push(emit_loop(loop_data, frame)?);
        }
        regions.push(emitted);
    }
    Ok(Some(SketchProfile {
        origin: frame.origin,
        x_axis: frame.x_axis,
        y_axis: frame.y_axis,
        z_axis: frame.z_axis,
        regions,
    }))
}

/// Re-project the part-origin reference point ([`SKETCH_ORIGIN_POINT_ID`]) onto
/// the RESOLVED frame, so a sketch whose plane moved since it was last edited
/// (a face or datum upstream changed) still constrains to the true origin, not
/// to where it projected when the editor last saw it. Its role flags are forced
/// too: the origin is always fixed, construction and an external reference.
fn pin_origin_point(sketch: &mut Value, frame: &Frame) {
    let Some(points) = sketch.get_mut("points").and_then(|p| p.as_array_mut()) else {
        return;
    };
    let offset = Vec3::new(0.0, 0.0, 0.0).sub(frame.origin);
    for point in points {
        if !point.get("id").is_some_and(is_sketch_origin_point_id) {
            continue;
        }
        point["x"] = Value::from(offset.dot(frame.x_axis));
        point["y"] = Value::from(offset.dot(frame.y_axis));
        point["fixed"] = Value::Bool(true);
        point["construction"] = Value::Bool(true);
        point["externalReference"] = Value::Bool(true);
    }
}

/// An id JSON value as a plain number, for the ids that must be ORDERED rather
/// than displayed (geometry ids and the persisted `loopId` — see `loop_ids`).
/// A numeric string counts; anything else (a uuid, a float) is `None` and the
/// loop falls back to the next id source.
fn numeric_id(value: &Value) -> Option<u64> {
    match value {
        Value::Number(number) => number.as_u64(),
        Value::String(text) => text.parse().ok(),
        _ => None,
    }
}

/// Display an id JSON value like the template literal `G${id}` (integral
/// numbers without a decimal point; strings verbatim).
fn id_display(value: &Value) -> String {
    match value {
        Value::Number(number) => {
            if let Some(int) = number.as_i64() {
                int.to_string()
            } else if let Some(uint) = number.as_u64() {
                uint.to_string()
            } else {
                number.to_string()
            }
        }
        Value::String(text) => text.clone(),
        other => other.to_string(),
    }
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "S",
    "shortName": "S",
    "longName": "Sketch",
    "ribbonPath": "Home/Reference/Sketch",
    "commandSize": "Large",
    "displayBuilder": true,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the sketch feature"
        },
        "sketchPlane": {
            "type": "reference_selection",
            "selectionFilter": [
                "PLANE",
                "FACE"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Select the plane or face for the sketch",
            "selectionValidationMessage": "Cannot use this sketch's own profile face as its sketch plane."
        },
        "projectedEdges": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE"
            ],
            "multiple": true,
            "default_value": null,
            "hint": "Model edges projected onto the sketch plane as read-only reference geometry"
        },
        "editSketch": {
            "type": "button",
            "label": "Edit Sketch",
            "default_value": null,
            "hint": "Launch the 2D sketch editor"
        }
    }
})
}
