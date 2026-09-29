use super::*;

// ---------------------------------------------------------------------------
// Implied-duplicate constraint removal (ConstraintSolver private helpers)
// ---------------------------------------------------------------------------

fn ordered_pair_key(a: i64, b: i64) -> String {
    if a <= b {
        format!("{a},{b}")
    } else {
        format!("{b},{a}")
    }
}

fn constraint_signature(
    constraint_type: &str,
    points: &[Value],
    canonical: &HashMap<i64, i64>,
) -> Option<String> {
    let canon = |value: &Value| -> Option<i64> {
        let pid = js_parse_int(Some(value))?;
        Some(*canonical.get(&pid).unwrap_or(&pid))
    };
    if constraint_type == "⇌" && points.len() >= 4 {
        let p0 = canon(&points[0])?;
        let p1 = canon(&points[1])?;
        let p2 = canon(&points[2])?;
        let p3 = canon(&points[3])?;
        let a = ordered_pair_key(p0, p1);
        let b = ordered_pair_key(p2, p3);
        return Some(if a <= b {
            format!("⇌:{a}|{b}")
        } else {
            format!("⇌:{b}|{a}")
        });
    }
    if constraint_type == "⏛" && points.len() >= 3 {
        let p0 = canon(&points[0])?;
        let p1 = canon(&points[1])?;
        let p2 = canon(&points[2])?;
        return Some(format!("⏛:{}|{}", ordered_pair_key(p0, p1), p2));
    }
    None
}

fn build_coincident_canonical_map(constraints: &[Value]) -> HashMap<i64, i64> {
    fn find(parent: &mut HashMap<i64, i64>, x: i64) -> i64 {
        let p = *parent.entry(x).or_insert(x);
        if p == x {
            return x;
        }
        let root = find(parent, p);
        parent.insert(x, root);
        root
    }

    let mut parent: HashMap<i64, i64> = HashMap::default();
    for constraint in constraints {
        let Some(obj) = constraint.as_object() else {
            continue;
        };
        if truthy(obj.get("temporary")) {
            continue;
        }
        if obj.get("type").and_then(Value::as_str) != Some("≡") {
            continue;
        }
        let Some(points) = obj.get("points").and_then(Value::as_array) else {
            continue;
        };
        if points.len() < 2 {
            continue;
        }
        let (Some(p0), Some(p1)) = (js_parse_int(points.first()), js_parse_int(points.get(1)))
        else {
            continue;
        };
        let ra = find(&mut parent, p0);
        let rb = find(&mut parent, p1);
        if ra != rb {
            if ra < rb {
                parent.insert(rb, ra);
            } else {
                parent.insert(ra, rb);
            }
        }
    }
    let keys: Vec<i64> = parent.keys().copied().collect();
    let mut out = HashMap::default();
    for key in keys {
        let root = find(&mut parent, key);
        out.insert(key, root);
    }
    out
}

fn remove_constraints_duplicated_by_implied_geometry(sketch: &mut Value) {
    let Some(obj) = sketch.as_object_mut() else {
        return;
    };
    let Some(geometries) = obj.get("geometries").and_then(Value::as_array).cloned() else {
        return;
    };
    let Some(constraints) = obj.get("constraints").and_then(Value::as_array).cloned() else {
        return;
    };

    let canonical = build_coincident_canonical_map(&constraints);

    let mut implied: std::collections::HashSet<String> = std::collections::HashSet::new();
    for geometry in &geometries {
        let Some(geo) = geometry.as_object() else {
            continue;
        };
        let Some(points) = geo.get("points").and_then(Value::as_array) else {
            continue;
        };
        let geometry_type = geo.get("type").and_then(Value::as_str).unwrap_or("");
        if geometry_type == "arc" && points.len() >= 3 {
            let candidate = vec![
                points[0].clone(),
                points[1].clone(),
                points[0].clone(),
                points[2].clone(),
            ];
            if let Some(key) = constraint_signature("⇌", &candidate, &canonical) {
                implied.insert(key);
            }
        } else if is_spline_geometry_type(geometry_type) && points.len() >= 4 {
            let seg_count = (points.len() - 1) / 3;
            let last_anchor_index = seg_count * 3;
            let mut i = 3usize;
            while i < last_anchor_index {
                let prev_handle = &points[i - 1];
                let anchor = &points[i];
                let next_handle = &points[i + 1];
                let nullish = |v: &Value| matches!(v, Value::Null);
                if !nullish(prev_handle) && !nullish(anchor) && !nullish(next_handle) {
                    let same = |a: &Value, b: &Value| point_key(a) == point_key(b);
                    if !same(prev_handle, anchor)
                        && !same(next_handle, anchor)
                        && !same(prev_handle, next_handle)
                    {
                        let candidate =
                            vec![prev_handle.clone(), next_handle.clone(), anchor.clone()];
                        if let Some(key) = constraint_signature("⏛", &candidate, &canonical) {
                            implied.insert(key);
                        }
                    }
                }
                i += 3;
            }
        }
    }
    if implied.is_empty() {
        return;
    }

    let filtered: Vec<Value> = constraints
        .into_iter()
        .filter(|constraint| {
            let Some(c) = constraint.as_object() else {
                return true;
            };
            if truthy(c.get("temporary")) {
                return true;
            }
            let constraint_type = c.get("type").and_then(Value::as_str).unwrap_or("");
            let Some(points) = c.get("points").and_then(Value::as_array) else {
                return true;
            };
            match constraint_signature(constraint_type, points, &canonical) {
                Some(key) => !implied.contains(&key),
                None => true,
            }
        })
        .collect();
    obj.insert("constraints".into(), Value::Array(filtered));
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

pub fn solve_sketch(request: &SolveSketchRequest) -> Result<Value, String> {
    let mut settings = SketchSolverSettings::default();
    if let Some(tolerance) = request.tolerance {
        settings.tolerance = tolerance;
    }
    if let Some(value) = request.distance_slide_threshold_ratio {
        if value.is_finite() && value >= 0.0 {
            settings.distance_slide_threshold_ratio = value;
        }
    }
    if let Some(value) = request.distance_slide_step_ratio {
        if value.is_finite() && value >= 0.0 {
            settings.distance_slide_step_ratio = value;
        }
    }
    if let Some(value) = request.distance_slide_min_step {
        if value.is_finite() && value >= 0.0 {
            settings.distance_slide_min_step = value;
        }
    }
    if let Some(polish) = request.polish {
        settings.newton_polish = polish;
    }

    let mut sketch = request.sketch.clone();
    if request.remove_implied_duplicates {
        remove_constraints_duplicated_by_implied_geometry(&mut sketch);
    }

    let iterations = request.iterations.unwrap_or(1500);
    let mut engine = Engine::new(&sketch, settings)?;
    let solved = engine.solve(iterations)?;

    let mut response = Map::new();
    response.insert("sketch".into(), solved);
    Ok(Value::Object(response))
}

pub fn solve_sketch_from_json(request_json: &str) -> Result<String, String> {
    let request: SolveSketchRequest =
        serde_json::from_str(request_json).map_err(|error| error.to_string())?;
    let response = solve_sketch(&request)?;
    serde_json::to_string(&response).map_err(|error| error.to_string())
}
