use super::*;

// ---------------------------------------------------------------------------
// Plane frame — LIVE resolution in Rust (headless), persisted basis as fallback
// ---------------------------------------------------------------------------

/// Resolve the sketch's plane frame, live-first (matching `_getOrCreateBasis`,
/// which recomputes from the reference and only keeps the persisted basis when the
/// reference is missing). Order: `sketchPlane` reference → DATUM/PLANE scene frame
/// → resident face frame → persisted `persistentData.basis` → identity XY. Returns
/// the frame plus any `unresolved` reference name (named but nothing resolved and
/// no basis to fall back on).
pub(super) fn resolve_frame(ctx: &FeatureContext) -> Result<(Frame, Vec<String>), String> {
    if let Some(name) = common::first_reference_name(ctx.param("sketchPlane")) {
        if let Some(frame) = ctx.scene.resolve_frame(&name) {
            return Ok((frame, Vec::new()));
        }
        if let Some(face) = ctx.scene.resolve_face(&name) {
            return Ok((common::face_frame(face)?, Vec::new()));
        }
        // Named but unresolved: fall back to the persisted basis if present, else
        // default to identity and report the miss (contract rule 1).
        if let Some(frame) = persisted_basis_frame(ctx) {
            return Ok((frame, Vec::new()));
        }
        return Ok((identity_frame(), vec![name]));
    }
    if let Some(frame) = persisted_basis_frame(ctx) {
        return Ok((frame, Vec::new()));
    }
    Ok((identity_frame(), Vec::new()))
}

fn identity_frame() -> Frame {
    Frame {
        origin: Vec3::new(0.0, 0.0, 0.0),
        x_axis: Vec3::new(1.0, 0.0, 0.0),
        y_axis: Vec3::new(0.0, 1.0, 0.0),
        z_axis: Vec3::new(0.0, 0.0, 1.0),
    }
}

/// The persisted `persistentData.basis` as a full frame (already computed upstream
/// worldUp-convention axes at authoring time), or `None` when absent.
fn persisted_basis_frame(ctx: &FeatureContext) -> Option<Frame> {
    let basis = ctx.persistent.get("basis")?;
    let read = |key: &str, default: Vec3| read_vec3(basis.get(key), default);
    Some(Frame {
        origin: read("origin", Vec3::new(0.0, 0.0, 0.0)),
        x_axis: read("x", Vec3::new(1.0, 0.0, 0.0)),
        y_axis: read("y", Vec3::new(0.0, 1.0, 0.0)),
        z_axis: read("z", Vec3::new(0.0, 0.0, 1.0)),
    })
}

/// Read a `[x, y, z]` JSON array as a `Vec3`, falling back to `default`.
fn read_vec3(value: Option<&Value>, default: Vec3) -> Vec3 {
    let Some(array) = value.and_then(|v| v.as_array()) else {
        return default;
    };
    let component = |index: usize, fallback: f64| {
        array.get(index).and_then(|v| v.as_f64()).unwrap_or(fallback)
    };
    Vec3::new(
        component(0, default.x),
        component(1, default.y),
        component(2, default.z),
    )
}


// ---------------------------------------------------------------------------
// Expression pre-evaluation
// ---------------------------------------------------------------------------

/// A JSON value that may be an expression string; evaluate it against the shared
/// env. A number passes through; a STRING is an expression and MUST resolve to a
/// finite number — a failure is an `Err`, never a silently kept string. That kept
/// string used to reach the solver's `coerce_coord`, which takes the leading
/// numeric PREFIX (`"20*Hh"` → `20`), so an unresolved variable produced a valid,
/// watertight solid of the wrong size with no error anywhere.
fn eval_expr(ctx: &FeatureContext, what: &str, value: &Value) -> Result<Option<f64>, String> {
    let Value::String(source) = value else {
        return Ok(None);
    };
    // An empty/whitespace slot is "unset", not an expression: the live editor
    // REMOVES `valueExpr` for a plain-number dimension rather than blanking it,
    // but a hand-written doc may carry `""`.
    if source.trim().is_empty() {
        return Ok(None);
    }
    let number = ctx
        .env
        .eval(source)
        .map_err(|error| format!("sketch: {what} = `{source}`: {error}"))?;
    if !number.is_finite() {
        return Err(format!("sketch: {what} = `{source}` evaluated to {number}"));
    }
    Ok(Some(number))
}

/// Walk points (`x`/`y`) and constraints (`value`/`valueExpr`), substituting any
/// expression string with its evaluated number BEFORE solving. Diameter-style
/// dimension expressions are halved to a radius. An expression that does not
/// resolve FAILS the sketch — the solver must never see a string coordinate.
pub(super) fn pre_evaluate_expressions(
    ctx: &FeatureContext,
    sketch: &mut Value,
) -> Result<(), String> {
    if let Some(points) = sketch.get_mut("points").and_then(|p| p.as_array_mut()) {
        for point in points {
            let id = point
                .get("id")
                .map(|id| id.to_string())
                .unwrap_or_else(|| "?".to_string());
            for key in ["x", "y"] {
                let Some(value) = point.get(key) else { continue };
                let what = format!("point {id} `{key}`");
                if let Some(number) = eval_expr(ctx, &what, &value.clone())? {
                    point[key] = Value::from(number);
                }
            }
        }
    }
    if let Some(constraints) = sketch.get_mut("constraints").and_then(|c| c.as_array_mut()) {
        for constraint in constraints {
            let id = constraint
                .get("id")
                .map(|id| id.to_string())
                .unwrap_or_else(|| "?".to_string());
            let is_diameter = constraint.get("type").and_then(|v| v.as_str()) == Some("⟺")
                && constraint.get("displayStyle").and_then(|v| v.as_str()) == Some("diameter")
                && constraint.get("valueExprMode").and_then(|v| v.as_str()) == Some("diameter");
            // `valueExpr` is the authored dimension; `value` is the last solved
            // number. A `valueExpr` that no longer resolves is an ERROR — falling
            // back to the stale `value` is the same silent wrong answer, and the
            // live editor already refuses a bad expression at entry time, so the
            // only way to get here is an expression-sheet edit that broke it.
            let expr = constraint
                .get("valueExpr")
                .cloned()
                .map(|value| eval_expr(ctx, &format!("constraint {id} `valueExpr`"), &value))
                .transpose()?
                .flatten()
                .map(|n| if is_diameter { n * 0.5 } else { n });
            let evaluated = match expr {
                Some(number) => Some(number),
                None => constraint
                    .get("value")
                    .cloned()
                    .map(|value| eval_expr(ctx, &format!("constraint {id} `value`"), &value))
                    .transpose()?
                    .flatten(),
            };
            if let Some(number) = evaluated {
                constraint["value"] = Value::from(number);
            }
        }
    }
    Ok(())
}
