use super::*;

fn coerce_coord(value: Option<&Value>) -> f64 {
    match value {
        Some(Value::Number(number)) => number.as_f64().unwrap_or(f64::NAN),
        Some(Value::String(text)) => parse_float_prefix(text),
        _ => f64::NAN,
    }
}

impl Engine {
    pub(super) fn new(sketch: &Value, settings: SketchSolverSettings) -> Result<Self, String> {
        let sketch_obj = sketch
            .as_object()
            .ok_or_else(|| "sketch must be an object".to_string())?;
        let raw_points = sketch_obj
            .get("points")
            .and_then(Value::as_array)
            .ok_or_else(|| "sketch.points must be an array".to_string())?;
        let mut points = Vec::with_capacity(raw_points.len());
        for raw in raw_points {
            let obj = raw.as_object();
            let id = obj
                .and_then(|o| o.get("id"))
                .cloned()
                .unwrap_or(Value::Null);
            let x = coerce_coord(obj.and_then(|o| o.get("x")));
            let y = coerce_coord(obj.and_then(|o| o.get("y")));
            let fixed = truthy(obj.and_then(|o| o.get("fixed")));
            let construction = obj.and_then(|o| o.get("construction")) == Some(&Value::Bool(true));
            let external_reference =
                obj.and_then(|o| o.get("externalReference")) == Some(&Value::Bool(true));
            points.push(EnginePoint {
                id,
                x,
                y,
                fixed,
                construction,
                external_reference,
            });
        }
        let mut point_index = HashMap::with_capacity_and_hasher(points.len(), Default::default());
        for (index, point) in points.iter().enumerate() {
            point_index.insert(point_key(&point.id), index);
        }
        let geometries = sketch_obj
            .get("geometries")
            .and_then(Value::as_array)
            .map(|list| list.to_vec())
            .unwrap_or_default();
        let raw_constraints = sketch_obj
            .get("constraints")
            .and_then(Value::as_array)
            .map(|list| list.to_vec())
            .unwrap_or_default();
        let mut constraints = Vec::with_capacity(raw_constraints.len());
        for raw in raw_constraints {
            match raw {
                Value::Object(map) => constraints.push(HotConstraint::from_map(map, &point_index)?),
                other => {
                    return Err(format!(
                        "sketch.constraints entries must be objects (got {other})"
                    ))
                }
            }
        }
        Ok(Self {
            points,
            point_index,
            geometries,
            constraints,
            settings,
            pass_token: String::new(),
            sig_scratch_before: Vec::new(),
            sig_scratch_after: Vec::new(),
        })
    }

    pub(super) fn tolerance(&self) -> f64 {
        self.settings.tolerance
    }

    fn fill_signature(points: &[EnginePoint], indices: &[Option<usize>], out: &mut Vec<SigEntry>) {
        out.clear();
        for index in indices {
            match index {
                None => out.push(None),
                Some(i) => {
                    let p = &points[*i];
                    out.push(Some((sig_bits(p.x), sig_bits(p.y), p.fixed)));
                }
            }
        }
    }

    fn signature_string(&self, indices: &[Option<usize>], entries: &[SigEntry]) -> String {
        let mut out = String::new();
        for (slot, entry) in entries.iter().enumerate() {
            match entry {
                None => out.push_str("null;"),
                Some((xb, yb, fixed)) => {
                    let index = indices[slot].expect("sig entry without point index");
                    let p = &self.points[index];
                    out.push_str(&format!(
                        "{}:{},{},{};",
                        fmt_id(&p.id),
                        fmt_number(f64::from_bits(*xb)),
                        fmt_number(f64::from_bits(*yb)),
                        if *fixed { 1 } else { 0 }
                    ));
                }
            }
        }
        out
    }

    fn all_points_signature(&self) -> Vec<(u64, u64, bool)> {
        self.points
            .iter()
            .map(|p| (sig_bits(p.x), sig_bits(p.y), p.fixed))
            .collect()
    }

    fn tidy_decimals_of_points(&mut self, decimals: i32, reset_fixed: bool) {
        let k = 10f64.powi(decimals);
        for point in &mut self.points {
            if reset_fixed {
                point.fixed = false;
            }
            if point.x.is_nan() {
                point.x = 0.0;
            }
            if point.y.is_nan() {
                point.y = 0.0;
            }
            point.x = js_round(point.x * k) / k;
            point.y = js_round(point.y * k) / k;
        }
    }

    pub(super) fn req(&self, indices: &[Option<usize>], slot: usize) -> Result<usize, String> {
        indices
            .get(slot)
            .copied()
            .flatten()
            .ok_or_else(|| "Cannot read properties of undefined (reading 'x')".to_string())
    }

    pub(super) fn calculate_angle(&self, from: usize, to: usize) -> f64 {
        let dx = self.points[to].x - self.points[from].x;
        let dy = self.points[to].y - self.points[from].y;
        let angle = dy.atan2(dx) * 180.0 / std::f64::consts::PI;
        (angle + 360.0) % 360.0
    }

    pub(super) fn distance(&self, a: usize, b: usize) -> f64 {
        let dx = self.points[a].x - self.points[b].x;
        let dy = self.points[a].y - self.points[b].y;
        (dx * dx + dy * dy).sqrt()
    }

    pub(super) fn rotate_point(&mut self, center_x: f64, center_y: f64, point: usize, angle_deg: f64) {
        let angle_rad = (angle_deg % 360.0) * (std::f64::consts::PI / 180.0);
        let x2 = self.points[point].x;
        let y2 = self.points[point].y;
        let cos = angle_rad.cos();
        let sin = angle_rad.sin();
        self.points[point].x = (x2 - center_x) * cos - (y2 - center_y) * sin + center_x;
        self.points[point].y = (x2 - center_x) * sin + (y2 - center_y) * cos + center_y;
    }

    /// A curvature constraint's point list plus the SECOND control point on
    /// each side — `P₃ᵢ±₂`, read off the spline's own polygon. Those are the
    /// points its relaxation moves, so they belong to its move signature; see
    /// the note in [`Engine::process_single_constraint`]. A side that does not
    /// resolve contributes a `None` slot, which the signature renders as the
    /// same "null;" a missing constraint point gets.
    fn curvature_signature_indices(&self, indices: &[Option<usize>]) -> Vec<Option<usize>> {
        let mut out = indices.to_vec();
        for (first, second) in [(0usize, 1usize), (2, 3)] {
            let pair = (
                indices.get(first).copied().flatten(),
                indices.get(second).copied().flatten(),
            );
            if let (Some(a), Some(b)) = pair {
                if let Some(side) = self.spline_curvature_side(a, b) {
                    out.push(side.second);
                }
            }
        }
        out
    }

    /// `participateInConstraint` from constraintDefinitions.
    pub(super) fn participate_in_constraint(&self, ctype: CType, indices: &[usize]) -> bool {
        self.constraints.iter().any(|c| {
            c.ctype == ctype
                && indices
                    .iter()
                    .all(|needed| c.point_idx.iter().any(|slot| *slot == Some(*needed)))
        })
    }

    /// Find a ⟺ constraint whose points include both indices; returns its
    /// Number-coerced value. Mirrors `solverObject.constraints.find(...)`.
    pub(super) fn find_distance_constraint_on_pair(&self, a: usize, b: usize) -> Option<f64> {
        for c in &self.constraints {
            if c.ctype != CType::Distance {
                continue;
            }
            let has_a = c.point_idx.iter().any(|slot| *slot == Some(a));
            let has_b = c.point_idx.iter().any(|slot| *slot == Some(b));
            if has_a && has_b {
                return Some(c.value_cv);
            }
        }
        None
    }
}

impl Engine {
    fn apply_constraint(
        &mut self,
        constraint: &mut HotConstraint,
        indices: &[Option<usize>],
        constraint_value: f64,
    ) -> CResult {
        match constraint.ctype {
            CType::Horizontal => {
                let a = self.req(indices, 0)?;
                let b = self.req(indices, 1)?;
                self.c_horizontal(constraint, a, b)
            }
            CType::Vertical => {
                let a = self.req(indices, 0)?;
                let b = self.req(indices, 1)?;
                self.c_vertical(constraint, a, b)
            }
            CType::Distance => {
                let a = self.req(indices, 0)?;
                let b = self.req(indices, 1)?;
                self.c_distance(constraint, a, b, constraint_value)
            }
            CType::PointLine => self.c_point_line_distance(constraint, indices, constraint_value),
            CType::EqualDistance => {
                let a = self.req(indices, 0)?;
                let b = self.req(indices, 1)?;
                let c = self.req(indices, 2)?;
                let d = self.req(indices, 3)?;
                self.c_equal_distance(constraint, a, b, c, d)
            }
            CType::Parallel => self.c_parallel(constraint, indices),
            CType::Perpendicular => self.c_perpendicular(constraint, indices),
            CType::Angle => self.c_angle(constraint, indices, constraint_value),
            CType::Coincident => {
                let a = self.req(indices, 0)?;
                let b = self.req(indices, 1)?;
                self.c_coincident(constraint, a, b)
            }
            CType::PointOnLine => {
                let a = self.req(indices, 0)?;
                let b = self.req(indices, 1)?;
                let c = self.req(indices, 2)?;
                self.c_point_on_line(constraint, a, b, c)
            }
            CType::Midpoint => {
                let a = self.req(indices, 0)?;
                let b = self.req(indices, 1)?;
                let c = self.req(indices, 2)?;
                self.c_midpoint(constraint, a, b, c)
            }
            CType::Ground => {
                let a = self.req(indices, 0)?;
                self.points[a].fixed = true;
                Ok(Value::Null)
            }
            CType::Tangent => self.c_tangent(constraint, indices),
            CType::SplineCurvature => self.c_spline_curvature(constraint, indices),
            CType::SplineFootTangent => self.c_spline_foot_tangent(constraint, indices),
            CType::Concentric => {
                let a = self.req(indices, 0)?;
                let b = self.req(indices, 1)?;
                self.c_concentric(constraint, a, b)
            }
            CType::EqualRadius => {
                let c1 = self.req(indices, 0)?;
                let b1 = self.req(indices, 1)?;
                let c2 = self.req(indices, 2)?;
                let b2 = self.req(indices, 3)?;
                self.c_equal_radius(constraint, c1, b1, c2, b2)
            }
            CType::Collinear => self.c_collinear(constraint, indices),
            CType::Symmetric => self.c_symmetric(constraint, indices),
            CType::Other => {
                Err("constraintFunctions[constraint.type] is not a function".to_string())
            }
        }
    }

    fn process_constraints_of_type(&mut self, filter: &Filter) -> Result<(), String> {
        for index in 0..self.constraints.len() {
            if !self.constraints[index].matches_filter(filter) {
                continue;
            }
            let mut constraint = std::mem::take(&mut self.constraints[index]);
            let result = self.process_single_constraint(&mut constraint);
            self.constraints[index] = constraint;
            result?;
        }
        Ok(())
    }

    fn process_single_constraint(&mut self, constraint: &mut HotConstraint) -> Result<(), String> {
        let constraint_value = constraint.value_parsed;

        // The previous engine captured `points` (the resolved point list) once and
        // used it for both the before and after signatures — even when the ∠
        // constraint swapped `constraint.points` mid-run. Mirror that by
        // snapshotting the resolved indices here.
        let indices = constraint.point_idx.clone();
        // The points the "did anything move?" signature has to cover. For every
        // constraint but one that is exactly its own point list. A CURVATURE
        // constraint also reads — and nudges — the control point beyond each
        // handle, derived from the spline's polygon rather than named in the
        // record: leave those out and a pass that moved only a second control
        // reads as a pass that changed nothing, which sets `status_solved`, arms
        // the shortcut below, and freezes the constraint one nudge into its
        // convergence (measured: it stopped at half the curvature violation).
        // A FOOT tangency has the same problem for the same reason: its point
        // list names the spline by its leading pair, and it moves span controls
        // that pair never mentions.
        let signature_indices = match constraint.ctype {
            CType::SplineCurvature => self.curvature_signature_indices(&indices),
            CType::SplineFootTangent => self.foot_signature_indices(&indices),
            _ => indices.clone(),
        };

        // `before` signature in bit form; the string is only built when a
        // comparison against an input JSON signature requires it.
        let mut before = std::mem::take(&mut self.sig_scratch_before);
        Self::fill_signature(&self.points, &signature_indices, &mut before);

        let same_solve_value = match constraint.prev_solve {
            PrevSolve::Num(previous) => {
                previous == constraint_value || (previous.is_nan() && constraint_value.is_nan())
            }
            _ => false,
        };
        let distance_slide_pending =
            is_distance_ctype(constraint.ctype) && constraint.throttle_true;

        if same_solve_value && !distance_slide_pending && constraint.status_solved {
            let signature_matches = match &constraint.prev_points {
                PrevPoints::Bits(bits, _) => bits.as_slice() == before.as_slice(),
                PrevPoints::Str(text) => {
                    self.signature_string(&signature_indices, &before) == *text
                }
                PrevPoints::Missing | PrevPoints::PresentNonString => false,
            };
            if signature_matches {
                self.sig_scratch_before = before;
                return Ok(());
            }
        }

        constraint.status_solved = false;
        constraint.status_written = true;
        constraint.set_error(Value::Null);
        let outcome = self.apply_constraint(constraint, &indices, constraint_value);
        if let Err(message) = outcome {
            constraint.set_error(Value::String(message));
        }
        let swapped_points = constraint.point_idx != indices;

        let mut after = std::mem::take(&mut self.sig_scratch_after);
        Self::fill_signature(&self.points, &signature_indices, &mut after);
        constraint.prev_solve = PrevSolve::Num(constraint_value);
        constraint.prev_solve_written = true;
        if before == after {
            constraint.status_solved = true;
            let text = self.signature_string(&signature_indices, &after);
            // After an ∠ point swap the stored bits would no longer line up
            // with the constraint's slots, so keep only the string form (the
            // previous engine compared strings that then simply mismatch).
            constraint.prev_points = if swapped_points {
                PrevPoints::Str(text)
            } else {
                PrevPoints::Bits(after.clone(), text)
            };
            constraint.prev_points_written = true;
        }
        self.sig_scratch_before = before;
        self.sig_scratch_after = after;
        Ok(())
    }

    fn has_pending_distance_target_slides(&self) -> bool {
        let slide_tolerance = if self.settings.tolerance.is_finite() {
            self.settings.tolerance
        } else {
            1e-8
        };
        self.constraints.iter().any(|c| {
            if !is_distance_ctype(c.ctype) || !c.throttle_true {
                return false;
            }
            let (Some(requested), Some(applied)) = (c.req_target, c.app_target) else {
                return false;
            };
            (requested - applied).abs() > slide_tolerance
        })
    }

    fn push_implied_temp_constraints(&mut self) -> Result<(), String> {
        // JavaScript: Math.max(0, ...ids where Number.isFinite(+id), else 0) + 1.
        let mut max_id = 0.0f64;
        for constraint in &self.constraints {
            let id = js_number(constraint.raw.get("id"));
            if id.is_finite() && id > max_id {
                max_id = id;
            }
        }
        let mut next_temp_id = max_id + 1.0;

        let mut point_index =
            HashMap::with_capacity_and_hasher(self.points.len(), Default::default());
        for (index, point) in self.points.iter().enumerate() {
            point_index.insert(point_key(&point.id), index);
        }

        let mut pending: Vec<(&'static str, Vec<Value>)> = Vec::new();
        for geometry in &self.geometries {
            let Some(obj) = geometry.as_object() else {
                continue;
            };
            let geometry_type = obj.get("type").and_then(Value::as_str).unwrap_or("");
            let points = obj.get("points").and_then(Value::as_array);
            if geometry_type == "arc" {
                let ids = points.cloned().unwrap_or_default();
                let pick = |slot: usize| ids.get(slot).cloned().unwrap_or(Value::Null);
                pending.push(("⇌", vec![pick(0), pick(1), pick(0), pick(2)]));
            } else if is_spline_geometry_type(geometry_type) {
                let Some(ids) = points else { continue };
                if ids.len() < 4 {
                    continue;
                }
                let seg_count = (ids.len() - 1) / 3;
                let last_anchor_index = seg_count * 3;
                let mut i = 3usize;
                while i < last_anchor_index {
                    let prev_handle = ids.get(i - 1).cloned().unwrap_or(Value::Null);
                    let anchor = ids.get(i).cloned().unwrap_or(Value::Null);
                    let next_handle = ids.get(i + 1).cloned().unwrap_or(Value::Null);
                    let nullish = |v: &Value| matches!(v, Value::Null);
                    if !nullish(&prev_handle) && !nullish(&anchor) && !nullish(&next_handle) {
                        let same = |a: &Value, b: &Value| point_key(a) == point_key(b);
                        if !same(&prev_handle, &anchor)
                            && !same(&next_handle, &anchor)
                            && !same(&prev_handle, &next_handle)
                        {
                            pending.push(("⏛", vec![prev_handle, next_handle, anchor]));
                        }
                    }
                    i += 3;
                }
            }
        }
        for (constraint_type, points) in pending {
            let mut map = Map::new();
            map.insert("id".into(), json_num(next_temp_id));
            next_temp_id += 1.0;
            map.insert("type".into(), Value::String(constraint_type.to_string()));
            map.insert("points".into(), Value::Array(points));
            map.insert("temporary".into(), Value::Bool(true));
            map.insert("labelX".into(), json_num(0.0));
            map.insert("labelY".into(), json_num(0.0));
            self.constraints
                .push(HotConstraint::from_map(map, &point_index)?);
        }
        Ok(())
    }

    pub(super) fn solve(&mut self, iterations: u32) -> Result<Value, String> {
        let decimals_places = 6;
        let cycle_id = GLOBAL_DISTANCE_SOLVE_CYCLE_ID.fetch_add(1, Ordering::SeqCst) + 1;

        self.push_implied_temp_constraints()?;
        // The coordinates as the caller stored them, before the tidy below
        // rounds them onto the relaxation's six-decimal grid. The polish starts
        // from these wherever relaxation never moved a coordinate off that grid.
        let stored: Vec<(f64, f64)> = self.points.iter().map(|p| (p.x, p.y)).collect();
        self.tidy_decimals_of_points(decimals_places, true);

        // Ground first, then everything.
        self.pass_token = format!("{cycle_id}:pre");
        self.process_constraints_of_type(&Filter::Type(CType::Ground, "⏚"))?;
        self.process_constraints_of_type(&Filter::All)?;

        let order = [
            Filter::Type(CType::PointOnLine, "⏛"),
            Filter::Type(CType::Horizontal, "━"),
            Filter::Type(CType::Vertical, "│"),
            Filter::Type(CType::Midpoint, "⋯"),
            Filter::Type(CType::Distance, "⟺"),
            Filter::Type(CType::PointLine, POINT_LINE_DISTANCE_TYPE),
            Filter::Type(CType::EqualDistance, "⇌"),
            Filter::Type(CType::Angle, "∠"),
            Filter::Type(CType::Perpendicular, "⟂"),
            Filter::Type(CType::Parallel, "∥"),
            Filter::Type(CType::Collinear, COLLINEAR_TYPE),
            Filter::Type(CType::Concentric, CONCENTRIC_TYPE),
            Filter::Type(CType::EqualRadius, EQUAL_RADIUS_TYPE),
            Filter::Type(CType::Symmetric, SYMMETRIC_TYPE),
            Filter::Type(CType::Tangent, TANGENT_TYPE),
            // Curvature AFTER tangency: a `ϰ` carries its own G1 stage, and
            // running it behind `⌒` lets a sketch that states both on one anchor
            // settle on the tangent direction first.
            Filter::Type(CType::SplineCurvature, SPLINE_CURVATURE_TYPE),
            // Foot-parameter tangency behind both: `∿` states where a curve
            // TOUCHES something, and running it after the anchor lanes lets a
            // sketch that states both settle the anchor's direction first — the
            // same reasoning that put `ϰ` behind `⌒`.
            Filter::Type(CType::SplineFootTangent, SPLINE_FOOT_TANGENT_TYPE),
            Filter::Type(CType::EqualDistance, "⇌"),
            Filter::Type(CType::Distance, "⟺"),
            Filter::Type(CType::PointLine, POINT_LINE_DISTANCE_TYPE),
            Filter::Type(CType::EqualDistance, "⇌"),
            Filter::Type(CType::Distance, "⟺"),
            Filter::Type(CType::PointLine, POINT_LINE_DISTANCE_TYPE),
            Filter::Type(CType::PointOnLine, "⏛"),
            Filter::Type(CType::Horizontal, "━"),
            Filter::Type(CType::Vertical, "│"),
        ];
        let coincident = Filter::Type(CType::Coincident, "≡");
        let horizontal = Filter::Type(CType::Horizontal, "━");
        let ascii_pipe = Filter::Type(CType::Other, "|");

        let mut prev = self.all_points_signature();
        for iteration in 0..iterations {
            self.pass_token = format!("{cycle_id}:{iteration}");
            for step in &order {
                self.process_constraints_of_type(step)?;
                self.process_constraints_of_type(&coincident)?;
                self.process_constraints_of_type(&horizontal)?;
                self.process_constraints_of_type(&ascii_pipe)?;
                self.tidy_decimals_of_points(decimals_places, false);
                self.process_constraints_of_type(&horizontal)?;
                self.process_constraints_of_type(&ascii_pipe)?;
                self.tidy_decimals_of_points(decimals_places, false);
            }
            let current = self.all_points_signature();
            if current == prev && !self.has_pending_distance_target_slides() {
                break;
            }
            prev = current;
        }

        // Global least-squares polish. Relaxation above only reaches its ~1e-5
        // residual floor (and the per-iteration 6-decimal `tidy` snaps points to
        // that floor); this drives the constraint residuals to machine precision
        // so downstream features receive exact geometry. It runs AFTER the final
        // `tidy` pass and its result is deliberately NOT re-rounded, so the gained
        // precision survives into the emitted points below.
        //
        // The tidy also ROUNDS THE INPUT, and the polish cannot recover a digit
        // it was never given. When relaxation moved NOTHING — every coordinate
        // is still bit for bit its stored value rounded — the sketch was already
        // at rest on the grid, and the polish starts from the stored digits
        // instead. Without that, a sketch already consistent to machine
        // precision comes out on a DIFFERENT consistent configuration: an arc
        // drawn through (7, 0), (6, −√3), (5, 0) had −√3 rounded to −1.732051,
        // its implied equal-radius row read 1.66e-7, and the polish settled on a
        // circle of radius 2 + 8.3e-8 about a centre 5.6e-8 away — which moved
        // the thicken-torus-fold-band-full-revolution row 4.0e-6 off a closed
        // form its lib fixture reads to 1.3e-8.
        //
        // All or nothing, never coordinate by coordinate. Once relaxation has
        // moved a point, it moved it to satisfy a row against its partners'
        // ROUNDED values, and handing those partners back their stored digits
        // reopens that row by up to half a grid step: restored per coordinate,
        // two spline fixtures ended with an anchor 2.1e-7 and 2.5e-7 off its
        // rim. Such a sketch keeps the grid start it always had.
        //
        // The relaxation itself is unchanged, so only the polish's starting
        // point moves, by at most the grid's half step — inside relaxation's own
        // floor, which is also why a polish SKIPPED for cost leaves the restored
        // points standing unpolished without harm. A sketch whose every stored
        // coordinate is on the grid restores to itself, and so does any sketch
        // this solver saved: its output rounds back to the grid point it was
        // polished from.
        if self.settings.newton_polish {
            let k = 10f64.powi(decimals_places);
            let on_grid = |stored: f64, now: f64| {
                stored.is_finite() && (js_round(stored * k) / k).to_bits() == now.to_bits()
            };
            let at_rest = self
                .points
                .iter()
                .zip(&stored)
                .all(|(point, &(x, y))| on_grid(x, point.x) && on_grid(y, point.y));
            if at_rest {
                for (point, (x, y)) in self.points.iter_mut().zip(stored) {
                    point.x = x;
                    point.y = y;
                }
            }
        }
        let polish_complete = self.newton_polish();

        // The polish moved the points, so every `∿` constraint's persisted touch
        // point now describes a configuration that no longer exists. Re-solve the
        // feet on the configuration the sketch actually ends on, so the seed the
        // NEXT solve (or the next drag frame) starts from belongs to the geometry
        // it is emitted beside.
        self.refresh_spline_foot_seeds();

        // Constraint diagnostics (degrees of freedom, over/under status,
        // per-point + per-geometry mobility). Computed on the SOLVED
        // configuration; purely additive read-only overlay information.
        let diagnostics = self.compute_diagnostics(polish_complete);

        let points: Vec<Value> = self
            .points
            .iter()
            .map(|p| {
                let mut map = Map::new();
                map.insert("id".into(), p.id.clone());
                map.insert("x".into(), json_num(p.x));
                map.insert("y".into(), json_num(p.y));
                map.insert("fixed".into(), Value::Bool(p.fixed));
                map.insert("construction".into(), Value::Bool(p.construction));
                map.insert(
                    "externalReference".into(),
                    Value::Bool(p.external_reference),
                );
                Value::Object(map)
            })
            .collect();
        let geometries: Vec<Value> = self.geometries.clone();
        let constraints: Vec<Value> = std::mem::take(&mut self.constraints)
            .into_iter()
            .filter(|c| !c.temporary)
            .map(|c| c.into_value())
            .collect();

        let mut sketch = Map::new();
        sketch.insert("points".into(), Value::Array(points));
        sketch.insert("geometries".into(), Value::Array(geometries));
        sketch.insert("constraints".into(), Value::Array(constraints));
        sketch.insert("diagnostics".into(), diagnostics);
        Ok(Value::Object(sketch))
    }
}
