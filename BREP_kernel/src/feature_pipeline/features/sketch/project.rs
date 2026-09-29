use super::*;

// ---------------------------------------------------------------------------
// Projected reference geometry — the in-context PROJECT lane
// ---------------------------------------------------------------------------

/// Resolve the sketch's optional `projectedEdges` reference_selection: each
/// name is a resident model edge — NAMESPACED COMPONENT edges included — whose
/// curve is orthographically projected onto the sketch plane and published as
/// a read-only reference PATH under `{sketchId}:REF:{edge name}`.
///
/// - Resolution is LIVE against the scene each run, so the projection follows
///   a component re-pose exactly like the plane attach (no baked geometry);
///   the reference name in `inputParams` also makes the cache's consumed-set
///   walk re-dirty this sketch whenever the source edge's producer re-runs.
/// - The published names are `{id}:`-prefixed, so the consumed-sketch purge in
///   `SceneMap::apply` removes them with the sketch; and their FIRST segment
///   is the sketch id, so they are never component-owned — downstream
///   consumers of the projected path are correctly unfenced (derived sketch
///   geometry, not component geometry).
/// - A miss is an `unresolved` entry (contract rule 1), never an error.
pub(super) fn projected_reference_paths(
    ctx: &FeatureContext,
    frame: &Frame,
    unresolved: &mut Vec<String>,
) -> Result<Vec<(String, Vec<NurbsCurve>)>, String> {
    let names = common::reference_names(ctx.param("projectedEdges"));
    if names.is_empty() {
        return Ok(Vec::new());
    }
    let normal = frame.z_axis.normalized()?;
    let mut paths = Vec::with_capacity(names.len());
    for name in names {
        let Some(edge) = ctx.scene.resolve_edge(&name) else {
            unresolved.push(name);
            continue;
        };
        // The edge curve trimmed to the edge, matching the edge-as-path
        // convention (`common::resolve_path` resolves through the same
        // `common::edge_curve`): projecting the untrimmed record curve would
        // publish a reference LONGER than the edge the user picked.
        let curve = common::edge_curve(edge)
            .map_err(|error| format!("sketch: projected {error}"))?;
        let projected = project_curve(&curve, frame.origin, normal)?;
        paths.push((format!("{}:REF:{name}", ctx.id), vec![projected]));
    }
    Ok(paths)
}

/// Project one curve onto the plane `(origin, n̂)`: every control point
/// `p → p − ((p−origin)·n̂)·n̂`, weights/knots/degree kept. Orthographic
/// projection is an AFFINE map, so the control-point image is exact for
/// rational curves too. An edge running along the plane normal degenerates to
/// a point-curve — published as-is (still a valid reference anchor).
fn project_curve(curve: &NurbsCurve, origin: Vec3, normal: Vec3) -> Result<NurbsCurve, String> {
    let mut control_points = Vec::with_capacity(curve.control_points.len());
    for cp in &curve.control_points {
        let point = cp.point()?;
        let signed = point.sub(origin).dot(normal);
        control_points.push(Vec4::from_point(point.sub(normal.scale(signed)), cp.w));
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
}
