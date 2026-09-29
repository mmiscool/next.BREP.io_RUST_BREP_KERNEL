use super::*;

/// Below this handle length a spline's tangent — and therefore its curvature —
/// has no direction to state. The relaxation reports the degenerate handle by
/// name; the residual model drops the row rather than contributing a zero one,
/// which would read as a redundant equation.
pub(super) const SPLINE_HANDLE_EPS: f64 = 1e-12;

/// One side of a spline curvature joint: the `(anchor, handle)` pair plus the
/// control point beyond the handle, and where that anchor sits in its own
/// control polygon. Built by [`Engine::spline_curvature_side`].
#[derive(Clone, Copy, Debug)]
pub(super) struct SplineCurvatureSide {
    /// Index into `Engine::geometries` of the spline this side belongs to.
    pub geometry: usize,
    /// Polygon slot `3i` of the anchor.
    pub anchor_slot: usize,
    /// Span count `n` of that spline (`3n + 1` control points).
    pub seg_count: usize,
    /// Engine point index of the on-curve anchor.
    pub anchor: usize,
    /// Engine point index of the adjacent off-curve handle (`P₃ᵢ±₁`).
    pub handle: usize,
    /// Engine point index of the control beyond the handle (`P₃ᵢ±₂`); `None`
    /// only for a document whose polygon names a point that does not exist.
    pub second: Option<usize>,
}

impl SplineCurvatureSide {
    /// Whether the anchor is an INTERIOR anchor of its spline — the only place
    /// the engine's own implied `⏛` already holds the two sides' tangents
    /// together, so the only place a curvature constraint needs no G1 row of its
    /// own.
    fn interior(&self) -> bool {
        self.anchor_slot > 0 && self.anchor_slot < 3 * self.seg_count
    }
}

impl Engine {
    // -------------------------------------------------------------------
    // New point-based constraints (Tangent / Concentric / Equal-radius /
    // Collinear / Symmetric). All geometry math lives here in Rust; the app
    // only builds the `{type, points}` record and renders the result.
    // -------------------------------------------------------------------

    /// True when a `line` geometry exists whose points reference both `a` and
    /// `b`. Used to tell a line/circle tangent apart from a circle/circle one
    /// without extra constraint fields.
    pub(super) fn is_line_pair(&self, a: usize, b: usize) -> bool {
        let key_a = point_key(&self.points[a].id);
        let key_b = point_key(&self.points[b].id);
        for geometry in &self.geometries {
            let Some(obj) = geometry.as_object() else {
                continue;
            };
            if obj.get("type").and_then(Value::as_str) != Some("line") {
                continue;
            }
            let Some(points) = obj.get("points").and_then(Value::as_array) else {
                continue;
            };
            let has_a = points.iter().any(|p| point_key(p) == key_a);
            let has_b = points.iter().any(|p| point_key(p) == key_b);
            if has_a && has_b {
                return true;
            }
        }
        false
    }

    /// If `(a, b)` is an adjacent (anchor, handle) pair of a spline geometry
    /// — in either slot order — return the canonical `(anchor, handle)`
    /// indices.
    ///
    /// A spline (`"bezier"`/`"spline"` geometry) with `3n + 1` control points
    /// has anchors at slots `3i` and handles in between; the adjacent pairs
    /// are `(ids[3i], ids[3i - 1])` for `i > 0` and `(ids[3i], ids[3i + 1])`
    /// for `i < n` — two at every interior anchor, one at each end. The cubic
    /// Bézier hodograph gives the TANGENT convention: `B'(0) = 3(P1 − P0)`
    /// and `B'(1) = 3(P_last − P_{last-1})`, so the tangent direction of a
    /// span at either of its anchors is exactly anchor→adjacent-handle. At an
    /// interior anchor the implied `⏛` (`push_implied_temp_constraints`)
    /// keeps the two handles collinear through the anchor, so the two spans'
    /// tangents agree there modulo 180° and a tangency written against either
    /// pair constrains the curve's tangent at that anchor; the pair the
    /// constraint records is the side whose handle the relaxation rotates.
    /// A tangency constraint on a spline constrains that segment's DIRECTION
    /// only (G1). CURVATURE continuity (G2) is the separate
    /// [`SPLINE_CURVATURE_TYPE`] constraint, which reads the same pairs through
    /// [`Engine::spline_curvature_side`] and adds the second control point.
    pub(super) fn spline_anchor_info(&self, a: usize, b: usize) -> Option<(usize, usize)> {
        self.spline_curvature_side(a, b)
            .map(|side| (side.anchor, side.handle))
    }

    /// The same adjacent `(anchor, handle)` pair [`spline_anchor_info`] accepts,
    /// resolved far enough to state the CURVATURE there: which spline it belongs
    /// to, where the anchor sits in that control polygon, and the control point
    /// BEYOND the handle (`P₃ᵢ±₂`) that the second derivative needs.
    ///
    /// The cubic hodograph gives `B''` at an anchor from three control points —
    /// anchor, adjacent handle, and the one past it — so this is the whole
    /// geometric input of a G2 row. The `second` slot always exists for a
    /// well-formed `3n + 1` polygon (an anchor's handle is never the last
    /// control of its span); it is `None` only when that control's id resolves
    /// to no point, which is a corrupt document, not a case. Returning it as an
    /// option keeps [`spline_anchor_info`] — which does not need it — answering
    /// for exactly the pairs it always did.
    pub(super) fn spline_curvature_side(&self, a: usize, b: usize) -> Option<SplineCurvatureSide> {
        let key_a = point_key(&self.points[a].id);
        let key_b = point_key(&self.points[b].id);
        for (gi, geometry) in self.geometries.iter().enumerate() {
            let Some(obj) = geometry.as_object() else {
                continue;
            };
            let geometry_type = obj.get("type").and_then(Value::as_str).unwrap_or("");
            if !is_spline_geometry_type(geometry_type) {
                continue;
            }
            let Some(points) = obj.get("points").and_then(Value::as_array) else {
                continue;
            };
            if points.len() < 4 {
                continue;
            }
            let seg_count = (points.len() - 1) / 3;
            for i in 0..=seg_count {
                let anchor_slot = 3 * i;
                let anchor_key = point_key(&points[anchor_slot]);
                if anchor_key != key_a && anchor_key != key_b {
                    continue;
                }
                let mut handle_slots = [None, None];
                if i > 0 {
                    handle_slots[0] = Some(anchor_slot - 1);
                }
                if i < seg_count {
                    handle_slots[1] = Some(anchor_slot + 1);
                }
                for handle_slot in handle_slots.into_iter().flatten() {
                    let handle_key = point_key(&points[handle_slot]);
                    let (anchor, handle) = if anchor_key == key_a && handle_key == key_b {
                        (a, b)
                    } else if anchor_key == key_b && handle_key == key_a {
                        (b, a)
                    } else {
                        continue;
                    };
                    // One step further along the polygon in the same direction:
                    // anchor 3i, handle 3i±1, second 3i±2.
                    let second_slot = 2 * handle_slot as isize - anchor_slot as isize;
                    let second = points
                        .get(second_slot as usize)
                        .and_then(|id| self.point_index_of(id));
                    return Some(SplineCurvatureSide {
                        geometry: gi,
                        anchor_slot,
                        seg_count,
                        anchor,
                        handle,
                        second,
                    });
                }
            }
        }
        None
    }

    /// The engine point index of a document point id (JavaScript `Map`
    /// SameValueZero identity, the same reading the constraint point lists get).
    pub(super) fn point_index_of(&self, id: &Value) -> Option<usize> {
        self.point_index.get(&point_key(id)).copied()
    }

    /// Rotate a spline's tangent segment (anchor→adjacent handle) by
    /// `angle_deg`. The handle is the natural orientation DOF: rotate it about
    /// the anchor whenever it is free, so the anchor (typically pinned by a
    /// coincident/ground, or shared with the neighbouring span) stays put;
    /// otherwise swing the free anchor about the fixed handle. No-op when both
    /// are fixed. At an interior anchor the implied `⏛` carries the rotation
    /// to the other side's handle over the following passes.
    fn rotate_spline_tangent_segment(&mut self, anchor: usize, handle: usize, angle_deg: f64) {
        if !self.points[handle].fixed {
            let (cx, cy) = (self.points[anchor].x, self.points[anchor].y);
            self.rotate_point(cx, cy, handle, angle_deg);
        } else if !self.points[anchor].fixed {
            let (cx, cy) = (self.points[handle].x, self.points[handle].y);
            self.rotate_point(cx, cy, anchor, angle_deg);
        }
    }

    /// Move `boundary` along the current center→boundary direction so that
    /// distance(center, boundary) == target (target clamped to >= 0).
    fn snap_radius(&mut self, center: usize, boundary: usize, target: f64) {
        let target = target.max(0.0);
        let dx = self.points[boundary].x - self.points[center].x;
        let dy = self.points[boundary].y - self.points[center].y;
        let r = (dx * dx + dy * dy).sqrt();
        if r <= 1e-12 {
            self.points[boundary].x = self.points[center].x + target;
            self.points[boundary].y = self.points[center].y;
            return;
        }
        let scale = target / r;
        self.points[boundary].x = self.points[center].x + dx * scale;
        self.points[boundary].y = self.points[center].y + dy * scale;
    }

    fn translate_point(&mut self, index: usize, dx: f64, dy: f64) {
        self.points[index].x += dx;
        self.points[index].y += dy;
    }

    /// Nudge a circle/arc so distance(center, boundary) moves toward `target`,
    /// distributing the correction over whichever of {center, boundary} is free.
    /// When the boundary is pinned (fixed, or held by another constraint such as
    /// a coincidence), the center absorbs the change over iterations — so equal
    /// radius can still resize a circle whose rim point is locked to geometry.
    /// `damp` in (0,1] controls the step size. Returns true if it could move.
    pub(super) fn adjust_circle_radius(
        &mut self,
        center: usize,
        boundary: usize,
        target: f64,
        damp: f64,
    ) -> bool {
        let target = target.max(0.0);
        let dx = self.points[boundary].x - self.points[center].x;
        let dy = self.points[boundary].y - self.points[center].y;
        let r = (dx * dx + dy * dy).sqrt();
        let free_b = !self.points[boundary].fixed;
        let free_c = !self.points[center].fixed;
        let n = (free_b as i32 + free_c as i32) as f64;
        if n == 0.0 {
            return false;
        }
        if r <= 1e-12 {
            // Degenerate: push the boundary (or center) out along +x.
            if free_b {
                self.points[boundary].x = self.points[center].x + target;
            } else {
                self.points[center].x = self.points[boundary].x - target;
            }
            return true;
        }
        let ux = dx / r;
        let uy = dy / r;
        let step = (target - r) * damp / n;
        if free_b {
            self.points[boundary].x += ux * step;
            self.points[boundary].y += uy * step;
        }
        if free_c {
            self.points[center].x -= ux * step;
            self.points[center].y -= uy * step;
        }
        true
    }

    /// Concentric: pull the two circle/arc centers together. Unlike Coincident,
    /// this does NOT lock the points as fixed (a concentric circle center should
    /// stay free to move as a pair).
    pub(super) fn c_concentric(&mut self, constraint: &mut HotConstraint, a: usize, b: usize) -> CResult {
        let tolerance = self.tolerance();
        let dx = self.points[a].x - self.points[b].x;
        let dy = self.points[a].y - self.points[b].y;
        if dx.hypot(dy) <= tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }
        let fixed_a = self.points[a].fixed;
        let fixed_b = self.points[b].fixed;
        if !fixed_a && !fixed_b {
            let cx = (self.points[a].x + self.points[b].x) / 2.0;
            let cy = (self.points[a].y + self.points[b].y) / 2.0;
            self.points[a].x = cx;
            self.points[a].y = cy;
            self.points[b].x = cx;
            self.points[b].y = cy;
            constraint.set_error(Value::Null);
        } else if !fixed_a {
            self.points[a].x = self.points[b].x;
            self.points[a].y = self.points[b].y;
            constraint.set_error(Value::Null);
        } else if !fixed_b {
            self.points[b].x = self.points[a].x;
            self.points[b].y = self.points[a].y;
            constraint.set_error(Value::Null);
        } else {
            constraint.set_error(Value::String(format!(
                "Concentric constraint not satisfied: centers {} and {} are both fixed",
                fmt_id(&self.points[a].id),
                fmt_id(&self.points[b].id)
            )));
        }
        Ok(Value::Null)
    }

    /// Pick which circle "leads" an equal-radius pair when neither is
    /// dimensioned or pinned, so interactive dragging works bidirectionally.
    ///
    /// The app moves a dragged boundary point and then re-solves without
    /// pinning it, so at solve time one radius has just jumped. We remember the
    /// radii from the previous solve (`_eqRadLast{1,2}`) and, on the first pass
    /// of each new solve (`_eqRadCycle`), treat whichever radius changed more as
    /// the reference the user is dragging; the other follows it. Returns 1 (use
    /// circle 1), 2 (use circle 2), or 0 (average — nothing is being dragged).
    fn equal_radius_leader(
        &self,
        constraint: &mut HotConstraint,
        r1: f64,
        r2: f64,
        tolerance: f64,
    ) -> i64 {
        let cycle = self.pass_token.split(':').next().unwrap_or("").to_string();
        let stored_cycle = constraint
            .raw
            .get("_eqRadCycle")
            .and_then(Value::as_str)
            .map(str::to_string);
        if stored_cycle.as_deref() == Some(cycle.as_str()) {
            return constraint
                .raw
                .get("_eqRadRef")
                .and_then(Value::as_f64)
                .map(|v| v as i64)
                .unwrap_or(0);
        }
        // First pass of a new solve: detect the just-dragged radius.
        let last1 = constraint
            .raw
            .get("_eqRadLast1")
            .and_then(Value::as_f64)
            .unwrap_or(r1);
        let last2 = constraint
            .raw
            .get("_eqRadLast2")
            .and_then(Value::as_f64)
            .unwrap_or(r2);
        let d1 = (r1 - last1).abs();
        let d2 = (r2 - last2).abs();
        let bias = tolerance.max(1e-6);
        let leader = if d1 > d2 + bias {
            1
        } else if d2 > d1 + bias {
            2
        } else {
            0
        };
        constraint
            .raw
            .insert("_eqRadRef".into(), json_num(leader as f64));
        constraint
            .raw
            .insert("_eqRadCycle".into(), Value::String(cycle));
        leader
    }

    /// Equal radius: dist(c1,b1) == dist(c2,b2). Reference selection order:
    /// explicit radius (Distance) dimension > pinned/fixed boundary >
    /// drag-leader (the boundary the user just moved) > average of both.
    pub(super) fn c_equal_radius(
        &mut self,
        constraint: &mut HotConstraint,
        c1: usize,
        b1: usize,
        c2: usize,
        b2: usize,
    ) -> CResult {
        let tolerance = self.tolerance();
        let r1_dim = self.find_distance_constraint_on_pair(c1, b1);
        let r2_dim = self.find_distance_constraint_on_pair(c2, b2);
        let r1 = self.distance(c1, b1);
        let r2 = self.distance(c2, b2);
        // A circle's radius is immovable only when BOTH its center and boundary
        // are fixed (otherwise the free one can absorb the change).
        let c1_locked = self.points[c1].fixed && self.points[b1].fixed;
        let c2_locked = self.points[c2].fixed && self.points[b2].fixed;

        // (target radius, move circle1?, move circle2?)
        let (target, move1, move2) = match (r1_dim, r2_dim) {
            (Some(_), Some(_)) => {
                let message = "Both circles have a radius dimension applied to them".to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            }
            (Some(value), None) => (value.abs(), false, true),
            (None, Some(value)) => (value.abs(), true, false),
            (None, None) => {
                if c1_locked && c2_locked {
                    if (r1 - r2).abs() <= tolerance {
                        constraint.set_error(Value::Null);
                    } else {
                        constraint.set_error(Value::String(format!(
                            "Equal radius: circles {} and {} are both fully fixed",
                            fmt_id(&self.points[c1].id),
                            fmt_id(&self.points[c2].id)
                        )));
                    }
                    return Ok(Value::Null);
                } else if c1_locked {
                    (r1, false, true)
                } else if c2_locked {
                    (r2, true, false)
                } else {
                    match self.equal_radius_leader(constraint, r1, r2, tolerance) {
                        1 => (r1, false, true),
                        2 => (r2, true, false),
                        _ => ((r1 + r2) / 2.0, true, true),
                    }
                }
            }
        };

        let mut dev = 0.0f64;
        if move1 {
            dev = dev.max((r1 - target).abs());
        }
        if move2 {
            dev = dev.max((r2 - target).abs());
        }
        if dev <= tolerance {
            constraint.set_error(Value::Null);
        } else {
            constraint.set_error(Value::String(format!(
                "Equal radius constraint not satisfied\n        {} != {}",
                fmt_number(r1),
                fmt_number(r2)
            )));
            let mut moved_any = false;
            if move1 {
                moved_any |= self.adjust_circle_radius(c1, b1, target, 0.5);
            }
            if move2 {
                moved_any |= self.adjust_circle_radius(c2, b2, target, 0.5);
            }
            if !moved_any {
                constraint.set_error(Value::String(format!(
                    "Equal radius: circles {} and {} cannot move",
                    fmt_id(&self.points[c1].id),
                    fmt_id(&self.points[c2].id)
                )));
            }
        }

        // Remember the resulting radii so the next solve can tell which one the
        // user drags.
        let out1 = self.distance(c1, b1);
        let out2 = self.distance(c2, b2);
        constraint.raw.insert("_eqRadLast1".into(), json_num(out1));
        constraint.raw.insert("_eqRadLast2".into(), json_num(out2));
        Ok(Value::Null)
    }

    /// Collinear: every point lies on the line through the first two. Reuses the
    /// tested Point-on-Line relaxation for each extra point.
    pub(super) fn c_collinear(
        &mut self,
        constraint: &mut HotConstraint,
        indices: &[Option<usize>],
    ) -> CResult {
        let resolved: Vec<usize> = indices.iter().filter_map(|slot| *slot).collect();
        if resolved.len() < 3 {
            constraint.set_error(Value::String(
                "Collinear constraint requires at least 3 points".into(),
            ));
            return Ok(Value::Null);
        }
        let a = resolved[0];
        let b = resolved[1];
        for &pk in &resolved[2..] {
            self.c_point_on_line(constraint, a, b, pk)?;
        }
        // Recompute overall satisfaction (overrides the per-point error set by
        // the last Point-on-Line correction).
        let tolerance = self.tolerance();
        let dx = self.points[b].x - self.points[a].x;
        let dy = self.points[b].y - self.points[a].y;
        let len = dx.hypot(dy);
        let mut max_dev = 0.0f64;
        if len > tolerance {
            for &pk in &resolved[2..] {
                let cx = self.points[pk].x - self.points[a].x;
                let cy = self.points[pk].y - self.points[a].y;
                max_dev = max_dev.max(((cx * dy - cy * dx) / len).abs());
            }
        }
        if len > tolerance && max_dev <= tolerance {
            constraint.set_error(Value::Null);
        } else {
            constraint.set_error(Value::String(format!(
                "Collinear constraint not satisfied. Max dist: {:.4}",
                max_dev
            )));
        }
        Ok(Value::Null)
    }

    /// Symmetric: points p and q are mirror images across the axis line a→b.
    pub(super) fn c_symmetric(
        &mut self,
        constraint: &mut HotConstraint,
        indices: &[Option<usize>],
    ) -> CResult {
        let a = self.req(indices, 0)?;
        let b = self.req(indices, 1)?;
        let p = self.req(indices, 2)?;
        let q = self.req(indices, 3)?;
        let tolerance = self.tolerance();
        let dx = self.points[b].x - self.points[a].x;
        let dy = self.points[b].y - self.points[a].y;
        let len_sq = dx * dx + dy * dy;
        if len_sq < tolerance * tolerance {
            constraint.set_error(Value::String(
                "Symmetric constraint axis is degenerate".into(),
            ));
            return Ok(Value::Null);
        }
        // Reflection of (px,py) across the infinite line a→b.
        let ax = self.points[a].x;
        let ay = self.points[a].y;
        let reflect = |px: f64, py: f64| -> (f64, f64) {
            let t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
            let proj_x = ax + t * dx;
            let proj_y = ay + t * dy;
            (2.0 * proj_x - px, 2.0 * proj_y - py)
        };
        let (target_q_x, target_q_y) = reflect(self.points[p].x, self.points[p].y);
        let (target_p_x, target_p_y) = reflect(self.points[q].x, self.points[q].y);
        let err_p = (self.points[p].x - target_p_x).hypot(self.points[p].y - target_p_y);
        let err_q = (self.points[q].x - target_q_x).hypot(self.points[q].y - target_q_y);
        if err_p.max(err_q) <= tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(format!(
            "Symmetric constraint not satisfied. Error: {:.4}",
            err_p.max(err_q)
        )));
        if !self.points[p].fixed {
            self.points[p].x += (target_p_x - self.points[p].x) * 0.5;
            self.points[p].y += (target_p_y - self.points[p].y) * 0.5;
        }
        if !self.points[q].fixed {
            self.points[q].x += (target_q_x - self.points[q].x) * 0.5;
            self.points[q].y += (target_q_y - self.points[q].y) * 0.5;
        }
        Ok(Value::Null)
    }

    /// Tangent dispatch. Spline anchor pairs (anchor, adjacent handle) — at
    /// the ends or at interior anchors — are recognized first, in either pair
    /// slot: spline/spline joins the two tangents, spline/line aligns the
    /// tangent with the line, spline/circle makes the tangent perpendicular
    /// to the radial direction at the anchor. Otherwise the legacy dispatch
    /// applies: line/circle when the first pair is a line geometry,
    /// circle/circle in all remaining cases. (A pair that is simultaneously a
    /// spline anchor pair and a line — every handle guide the sketcher draws
    /// is one — is read as the spline.)
    pub(super) fn c_tangent(&mut self, constraint: &mut HotConstraint, indices: &[Option<usize>]) -> CResult {
        let p0 = self.req(indices, 0)?;
        let p1 = self.req(indices, 1)?;
        let p2 = self.req(indices, 2)?;
        let p3 = self.req(indices, 3)?;
        match (self.spline_anchor_info(p0, p1), self.spline_anchor_info(p2, p3)) {
            (Some((a1, h1)), Some((a2, h2))) => {
                self.c_tangent_spline_spline(constraint, a1, h1, a2, h2)
            }
            (Some((anchor, handle)), None) => {
                if self.is_line_pair(p2, p3) {
                    self.c_tangent_spline_line(constraint, anchor, handle, p2, p3)
                } else {
                    self.c_tangent_spline_circle(constraint, anchor, handle, p2, p3)
                }
            }
            (None, Some((anchor, handle))) => {
                if self.is_line_pair(p0, p1) {
                    self.c_tangent_spline_line(constraint, anchor, handle, p0, p1)
                } else {
                    self.c_tangent_spline_circle(constraint, anchor, handle, p0, p1)
                }
            }
            (None, None) => {
                if self.is_line_pair(p0, p1) {
                    self.c_tangent_line_circle(constraint, p0, p1, p2, p3)
                } else {
                    self.c_tangent_circle_circle(constraint, p0, p1, p2, p3)
                }
            }
        }
    }

    /// A tangent side (two-point segment) may rotate only when it is not
    /// fully fixed and not pinned to an axis by a horizontal/vertical
    /// constraint (mirrors the ∠ constraint's movement guards).
    fn tangent_side_movable(&self, a: usize, b: usize) -> bool {
        let pinned = self.points[a].fixed && self.points[b].fixed;
        let axis_locked = self.participate_in_constraint(CType::Horizontal, &[a, b])
            || self.participate_in_constraint(CType::Vertical, &[a, b]);
        !(pinned || axis_locked)
    }

    /// Shared angular relaxation step for the spline tangent variants: rotate
    /// toward `target_angle` (degrees, relative to `current_angle`) with the
    /// same 1.5°-per-pass clamp the ∠ constraint uses. Returns the clamped
    /// rotation to apply to side 1 (side 2 gets the negative), split in half
    /// when both sides can move.
    fn tangent_rotation_split(
        delta_raw: f64,
        side1_moving: bool,
        side2_moving: bool,
    ) -> (f64, f64) {
        let max_step = 1.5;
        let mut delta = delta_raw;
        if delta.abs() > max_step {
            delta = js_sign(delta) * max_step;
        }
        if side1_moving && side2_moving {
            (delta / 2.0, -delta / 2.0)
        } else if side1_moving {
            (delta, 0.0)
        } else if side2_moving {
            (0.0, -delta)
        } else {
            (0.0, 0.0)
        }
    }

    /// Spline-anchor / line tangent: the spline's tangent direction at the
    /// anchor (anchor→handle) must be parallel to line `a→b`. Direction only
    /// (G1) — there is no curvature to match against a straight line anyway.
    fn c_tangent_spline_line(
        &mut self,
        constraint: &mut HotConstraint,
        anchor: usize,
        handle: usize,
        a: usize,
        b: usize,
    ) -> CResult {
        let tolerance = self.tolerance();
        if self.distance(anchor, handle) < tolerance {
            constraint.set_error(Value::String(
                "Tangent: spline tangent is degenerate".into(),
            ));
            return Ok(Value::Null);
        }
        if self.distance(a, b) < tolerance {
            constraint.set_error(Value::String("Tangent: line is degenerate".into()));
            return Ok(Value::Null);
        }
        let seg_angle = self.calculate_angle(anchor, handle);
        let line_angle = self.calculate_angle(a, b);
        let current = normalize_angle(seg_angle - line_angle);
        // Parallel modulo 180: pick the closer of 0°/180° (∥ constraint rule).
        let target = if current > 90.0 && current <= 270.0 {
            180.0
        } else if current > 270.0 {
            360.0
        } else {
            0.0
        };
        let delta_raw = shortest_angle_delta(target, current);
        if delta_raw.abs() < tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(format!(
            "Tangent constraint not satisfied\n        spline tangent {} not parallel to line {}",
            fmt_number(seg_angle),
            fmt_number(line_angle)
        )));

        let seg_moving = self.tangent_side_movable(anchor, handle);
        let line_moving = self.tangent_side_movable(a, b);
        if !seg_moving && !line_moving {
            constraint.set_error(Value::String(
                "Tangent: spline tangent and line are both fully fixed".into(),
            ));
            return Ok(Value::Null);
        }
        let (rot_seg, rot_line) = Self::tangent_rotation_split(delta_raw, seg_moving, line_moving);
        if rot_seg != 0.0 && !rot_seg.is_nan() {
            self.rotate_spline_tangent_segment(anchor, handle, rot_seg);
        }
        if rot_line != 0.0 && !rot_line.is_nan() {
            // Same pivot policy as the ∠ constraint applies to a line.
            if self.points[a].fixed {
                let (cx, cy) = (self.points[a].x, self.points[a].y);
                self.rotate_point(cx, cy, b, rot_line);
            } else if self.points[b].fixed {
                let (cx, cy) = (self.points[b].x, self.points[b].y);
                self.rotate_point(cx, cy, a, rot_line);
            } else {
                let mid_x = (self.points[a].x + self.points[b].x) / 2.0;
                let mid_y = (self.points[a].y + self.points[b].y) / 2.0;
                self.rotate_point(mid_x, mid_y, a, rot_line);
                self.rotate_point(mid_x, mid_y, b, rot_line);
            }
        }
        Ok(Value::Null)
    }

    /// Spline-anchor / circle (or arc) tangent: the spline's tangent at the
    /// anchor (anchor→handle) must be perpendicular to the radial direction
    /// center→anchor, i.e. the curve passes its anchor along the circle's
    /// tangent there. Pair it with a coincident (spline anchor ≡ rim/arc
    /// point) so the anchor actually lies on the circle. G1 only: this
    /// constraint does not match the spline's curvature to 1/r — that is the
    /// `ϰ` constraint ([`Engine::c_spline_curvature`]), which emits this same
    /// row plus a curvature row.
    fn c_tangent_spline_circle(
        &mut self,
        constraint: &mut HotConstraint,
        anchor: usize,
        handle: usize,
        center: usize,
        boundary: usize,
    ) -> CResult {
        let tolerance = self.tolerance();
        if self.distance(anchor, handle) < tolerance {
            constraint.set_error(Value::String(
                "Tangent: spline tangent is degenerate".into(),
            ));
            return Ok(Value::Null);
        }
        if self.distance(center, anchor) < tolerance {
            constraint.set_error(Value::String(
                "Tangent: spline anchor coincides with the circle center".into(),
            ));
            return Ok(Value::Null);
        }
        let (delta_raw, seg_angle, radial_angle) = self.spline_circle_g1_delta(anchor, handle, center);
        if delta_raw.abs() < tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(format!(
            "Tangent constraint not satisfied\n        spline tangent {} not perpendicular to radius {}",
            fmt_number(seg_angle),
            fmt_number(radial_angle)
        )));

        if !self.rotate_spline_and_circle(anchor, handle, center, boundary, delta_raw) {
            constraint.set_error(Value::String(
                "Tangent: spline tangent and circle are both fully fixed".into(),
            ));
        }
        Ok(Value::Null)
    }

    /// The rotation (degrees) that would take the spline tangent
    /// `anchor→handle` onto perpendicular-to-radius at the anchor, with the two
    /// angles the message quotes. Perpendicular modulo 180: the closer of
    /// 90°/270° (the `⟂` rule), so either tangent orientation is accepted.
    /// Shared by the `⌒` and `ϰ` spline/circle lanes — one G1 target, so
    /// "curvature continuous" cannot drift onto a different tangent than
    /// "tangent" would have chosen.
    fn spline_circle_g1_delta(&self, anchor: usize, handle: usize, center: usize) -> (f64, f64, f64) {
        let seg_angle = self.calculate_angle(anchor, handle);
        let radial_angle = self.calculate_angle(center, anchor);
        let current = normalize_angle(seg_angle - radial_angle);
        let target = if current <= 180.0 { 90.0 } else { 270.0 };
        (shortest_angle_delta(target, current), seg_angle, radial_angle)
    }

    /// Apply the G1 rotation split between the spline tangent and the circle:
    /// the handle swings about the anchor, the circle rotates RIGIDLY (center
    /// and boundary together) about the tangency point so its radius is
    /// preserved. `false` when neither side can move — the caller names that in
    /// its own words.
    fn rotate_spline_and_circle(
        &mut self,
        anchor: usize,
        handle: usize,
        center: usize,
        boundary: usize,
        delta_raw: f64,
    ) -> bool {
        let seg_moving = self.tangent_side_movable(anchor, handle);
        let circle_moving = !self.points[center].fixed && !self.points[boundary].fixed;
        if !seg_moving && !circle_moving {
            return false;
        }
        let (rot_seg, rot_circle) =
            Self::tangent_rotation_split(delta_raw, seg_moving, circle_moving);
        if rot_seg != 0.0 && !rot_seg.is_nan() {
            self.rotate_spline_tangent_segment(anchor, handle, rot_seg);
        }
        if rot_circle != 0.0 && !rot_circle.is_nan() {
            let (cx, cy) = (self.points[anchor].x, self.points[anchor].y);
            self.rotate_point(cx, cy, center, rot_circle);
            self.rotate_point(cx, cy, boundary, rot_circle);
        }
        true
    }

    /// Spline-anchor / spline-anchor tangent (G1 join): the two tangent
    /// directions must be parallel. Typically combined with a coincident on
    /// the two anchors so the curves share the point.
    fn c_tangent_spline_spline(
        &mut self,
        constraint: &mut HotConstraint,
        a1: usize,
        h1: usize,
        a2: usize,
        h2: usize,
    ) -> CResult {
        let tolerance = self.tolerance();
        if self.distance(a1, h1) < tolerance || self.distance(a2, h2) < tolerance {
            constraint.set_error(Value::String(
                "Tangent: spline tangent is degenerate".into(),
            ));
            return Ok(Value::Null);
        }
        let (delta_raw, angle1, angle2) = self.spline_spline_g1_delta(a1, h1, a2, h2);
        if delta_raw.abs() < tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(format!(
            "Tangent constraint not satisfied\n        spline tangents {} and {} not parallel",
            fmt_number(angle1),
            fmt_number(angle2)
        )));

        if !self.rotate_spline_and_spline(a1, h1, a2, h2, delta_raw) {
            constraint.set_error(Value::String(
                "Tangent: both spline tangents are fully fixed".into(),
            ));
        }
        Ok(Value::Null)
    }

    /// The rotation (degrees) that would make the two spline tangents parallel,
    /// with the two angles the message quotes. Parallel modulo 180 (the `∥`
    /// rule): two curve ends meeting smoothly have their anchor→handle
    /// directions OPPOSED, so 180° is as good a target as 0°. Shared by the `⌒`
    /// and `ϰ` spline/spline lanes.
    fn spline_spline_g1_delta(&self, a1: usize, h1: usize, a2: usize, h2: usize) -> (f64, f64, f64) {
        let angle1 = self.calculate_angle(a1, h1);
        let angle2 = self.calculate_angle(a2, h2);
        let current = normalize_angle(angle1 - angle2);
        let target = if current > 90.0 && current <= 270.0 {
            180.0
        } else if current > 270.0 {
            360.0
        } else {
            0.0
        };
        (shortest_angle_delta(target, current), angle1, angle2)
    }

    /// Apply the G1 rotation split between two spline tangents. `false` when
    /// neither side can move.
    fn rotate_spline_and_spline(
        &mut self,
        a1: usize,
        h1: usize,
        a2: usize,
        h2: usize,
        delta_raw: f64,
    ) -> bool {
        let m1 = self.tangent_side_movable(a1, h1);
        let m2 = self.tangent_side_movable(a2, h2);
        if !m1 && !m2 {
            return false;
        }
        let (rot1, rot2) = Self::tangent_rotation_split(delta_raw, m1, m2);
        if rot1 != 0.0 && !rot1.is_nan() {
            self.rotate_spline_tangent_segment(a1, h1, rot1);
        }
        if rot2 != 0.0 && !rot2.is_nan() {
            self.rotate_spline_tangent_segment(a2, h2, rot2);
        }
        true
    }

    /// Line/circle tangent: perpendicular distance(center, line a→b) == radius,
    /// where radius = dist(center, boundary).
    fn c_tangent_line_circle(
        &mut self,
        constraint: &mut HotConstraint,
        a: usize,
        b: usize,
        center: usize,
        boundary: usize,
    ) -> CResult {
        let tolerance = self.tolerance();
        let dx = self.points[b].x - self.points[a].x;
        let dy = self.points[b].y - self.points[a].y;
        let len = (dx * dx + dy * dy).sqrt();
        if len < tolerance {
            constraint.set_error(Value::String("Tangent: line is degenerate".into()));
            return Ok(Value::Null);
        }
        let nx = -dy / len;
        let ny = dx / len;
        let signed = (self.points[center].x - self.points[a].x) * nx
            + (self.points[center].y - self.points[a].y) * ny;
        let side = if signed >= 0.0 { 1.0 } else { -1.0 };
        let d = signed.abs();
        let radius = self.distance(center, boundary);
        let err = d - radius;
        if err.abs() <= tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(format!(
            "Tangent constraint not satisfied\n        distance {} != radius {}",
            fmt_number(d),
            fmt_number(radius)
        )));

        let center_free = !self.points[center].fixed;
        let boundary_free = !self.points[boundary].fixed;
        let handles = (center_free as i32) + (boundary_free as i32);
        if handles == 0 {
            constraint.set_error(Value::String(
                "Tangent: circle center and boundary are both fixed".into(),
            ));
            return Ok(Value::Null);
        }
        let per = err / handles as f64 * 0.5;
        // Increase radius toward the line (uses the original center position).
        if boundary_free {
            self.snap_radius(center, boundary, radius + per);
        }
        // Move the center toward the line to reduce the perpendicular distance.
        if center_free {
            self.points[center].x -= nx * side * per;
            self.points[center].y -= ny * side * per;
        }
        Ok(Value::Null)
    }

    /// Circle/circle tangent: dist(centers) == r1 + r2 (external) or |r1 - r2|
    /// (internal), chosen by the initial configuration. Circles translate
    /// rigidly (center + boundary together) so radii are preserved.
    fn c_tangent_circle_circle(
        &mut self,
        constraint: &mut HotConstraint,
        c1: usize,
        b1: usize,
        c2: usize,
        b2: usize,
    ) -> CResult {
        let tolerance = self.tolerance();
        let r1 = self.distance(c1, b1);
        let r2 = self.distance(c2, b2);
        let cdx = self.points[c2].x - self.points[c1].x;
        let cdy = self.points[c2].y - self.points[c1].y;
        let d = (cdx * cdx + cdy * cdy).sqrt();
        let external = r1 + r2;
        let internal = (r1 - r2).abs();
        let target = if (d - external).abs() <= (d - internal).abs() {
            external
        } else {
            internal
        };
        let err = d - target;
        if err.abs() <= tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(format!(
            "Tangent constraint not satisfied\n        center distance {} != {}",
            fmt_number(d),
            fmt_number(target)
        )));

        let (ux, uy) = if d > 1e-12 {
            (cdx / d, cdy / d)
        } else {
            (1.0, 0.0)
        };
        // A circle can be translated only when both its center and boundary are
        // free, so its radius is preserved.
        let c1_movable = !self.points[c1].fixed && !self.points[b1].fixed;
        let c2_movable = !self.points[c2].fixed && !self.points[b2].fixed;
        let handles = (c1_movable as i32) + (c2_movable as i32);
        if handles == 0 {
            constraint.set_error(Value::String(
                "Tangent: both circles are fully constrained".into(),
            ));
            return Ok(Value::Null);
        }
        let per = err / handles as f64 * 0.5;
        if c1_movable {
            self.translate_point(c1, ux * per, uy * per);
            self.translate_point(b1, ux * per, uy * per);
        }
        if c2_movable {
            self.translate_point(c2, -ux * per, -uy * per);
            self.translate_point(b2, -ux * per, -uy * per);
        }
        Ok(Value::Null)
    }

    // -------------------------------------------------------------------
    // Curvature continuity (`ϰ`, SPLINE_CURVATURE_TYPE).
    //
    // G2: the curve's signed curvature agrees across a joint, or matches a
    // circle/arc's 1/ρ at a spline end. The constraint ALWAYS carries the
    // tangency its join needs — at an interior anchor through the engine's
    // own implied `⏛`, everywhere else as a row of its own — so "curvature
    // continuous" implies "tangent".
    // -------------------------------------------------------------------

    /// Curvature (G2) continuity. The point list is the SAME shape `⌒` uses —
    /// two pairs, `[pairA0, pairA1, pairB0, pairB1]` — and is dispatched the
    /// same way: each pair is tried as a spline (anchor, handle) pair first.
    ///
    /// * two spline pairs → the joint lane ([`Self::c_curvature_joint`]): the
    ///   two sides of one spline's interior anchor, or two spline ENDS held
    ///   together by a coincident (including one spline closed on itself).
    /// * one spline pair + a circle/arc `(center, boundary)` → the circle lane.
    /// * one spline pair + a LINE is refused by name: a line's curvature is
    ///   zero everywhere, which the plan does not derive a residual for, and
    ///   silently reading the line's two ends as a circle would be a lie.
    pub(super) fn c_spline_curvature(
        &mut self,
        constraint: &mut HotConstraint,
        indices: &[Option<usize>],
    ) -> CResult {
        let p0 = self.req(indices, 0)?;
        let p1 = self.req(indices, 1)?;
        let p2 = self.req(indices, 2)?;
        let p3 = self.req(indices, 3)?;
        match (
            self.spline_curvature_side(p0, p1),
            self.spline_curvature_side(p2, p3),
        ) {
            (Some(side1), Some(side2)) => self.c_curvature_joint(constraint, side1, side2),
            (Some(side), None) => self.c_curvature_circle(constraint, side, p2, p3),
            (None, Some(side)) => self.c_curvature_circle(constraint, side, p0, p1),
            (None, None) => {
                constraint.set_error(Value::String(
                    "Curvature: no spline anchor and handle pair in the constraint".into(),
                ));
                Ok(Value::Null)
            }
        }
    }

    /// Whether the engine's own implied `⏛` already holds this joint's two
    /// tangents together — the exact predicate `push_implied_temp_constraints`
    /// uses: one spline, an INTERIOR anchor `3i` with `0 < i < n`, both flanking
    /// handles present and the three ids pairwise distinct. Only then does a
    /// curvature constraint need no G1 row of its own. A spline closed on itself
    /// joins slot `0` to slot `3n`, which that loop never reaches, so the
    /// closure joint gets its own G1 row like any two-spline join.
    pub(super) fn joint_g1_is_implied(
        &self,
        side1: &SplineCurvatureSide,
        side2: &SplineCurvatureSide,
    ) -> bool {
        side1.geometry == side2.geometry
            && side1.anchor_slot == side2.anchor_slot
            && side1.interior()
            && side2.interior()
            && side1.anchor == side2.anchor
            && side1.handle != side2.handle
            && side1.handle != side1.anchor
            && side2.handle != side2.anchor
    }

    /// G2 at a joint: the two sides' signed curvatures must agree. With
    /// `κ_out` the curvature read along each side's own anchor→handle
    /// direction, the arriving side's curvature is `−κ_out`, so the condition is
    /// `κ_out1 + κ_out2 = 0` — symmetric in the two sides, which is why the slot
    /// order of the two pairs does not change the solve.
    ///
    /// Relaxation moves the free SECOND control points along the joint normal
    /// toward the matched curvature (half the violation each when both are free,
    /// 0.5 damping). It never touches the handles, so it cannot disturb the
    /// tangency the joint already has; the LM polish provides exactness.
    fn c_curvature_joint(
        &mut self,
        constraint: &mut HotConstraint,
        side1: SplineCurvatureSide,
        side2: SplineCurvatureSide,
    ) -> CResult {
        let tolerance = self.tolerance();
        let (Some(second1), Some(second2)) = (side1.second, side2.second) else {
            constraint.set_error(Value::String(
                "Curvature: the spline control polygon is incomplete".into(),
            ));
            return Ok(Value::Null);
        };
        if side1.handle == side2.handle {
            constraint.set_error(Value::String(
                "Curvature: the two sides of the joint are the same handle".into(),
            ));
            return Ok(Value::Null);
        }
        let (Some((k1, a1)), Some((k2, a2))) = (
            self.spline_side_curvature(&side1, second1),
            self.spline_side_curvature(&side2, second2),
        ) else {
            constraint.set_error(Value::String(
                "Curvature: spline tangent is degenerate".into(),
            ));
            return Ok(Value::Null);
        };
        if a1 < tolerance || a2 < tolerance {
            constraint.set_error(Value::String(
                "Curvature: spline tangent is degenerate".into(),
            ));
            return Ok(Value::Null);
        }

        // The joint's own G1, unless the implied `⏛` is already holding it.
        let implied_g1 = self.joint_g1_is_implied(&side1, &side2);
        let (g1_delta, angle1, angle2) = if implied_g1 {
            (0.0, 0.0, 0.0)
        } else {
            self.spline_spline_g1_delta(side1.anchor, side1.handle, side2.anchor, side2.handle)
        };

        // The residual is in length units (see the plan: the LM polish weighs
        // raw residuals against each other), so the SATISFIED test divides it
        // back out by the same length scale — an absolute bar on a length¹
        // residual would tighten with the model's size.
        let residual = joint_curvature_residual(k1, k2, a1, a2);
        let length = 0.5 * (a1 + a2);
        let g1_ok = g1_delta.abs() < tolerance;
        if g1_ok && residual.abs() <= tolerance * length {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }

        let involved = [
            side1.anchor,
            side1.handle,
            second1,
            side2.anchor,
            side2.handle,
            second2,
        ];
        if involved.iter().all(|&i| self.points[i].fixed) {
            constraint.set_error(Value::String(
                "Curvature: the whole joint is fully fixed".into(),
            ));
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(if !g1_ok {
            format!(
                "Curvature constraint not satisfied\n        spline tangents {} and {} not parallel",
                fmt_number(angle1),
                fmt_number(angle2)
            )
        } else {
            format!(
                "Curvature constraint not satisfied\n        spline curvatures {} and {} differ",
                fmt_number(-k1),
                fmt_number(k2)
            )
        }));

        if !g1_ok {
            self.rotate_spline_and_spline(
                side1.anchor,
                side1.handle,
                side2.anchor,
                side2.handle,
                g1_delta,
            );
        }
        // `κ_out1 + κ_out2 = 0`: with both second controls free each moves half
        // the violation (the plan's split); with one frozen the free side takes
        // the frozen side's curvature, which is the same fixed point the mean
        // reaches, in a quarter of the passes.
        let free1 = !self.points[second1].fixed;
        let free2 = !self.points[second2].fixed;
        let (target1, target2) = if free1 && free2 {
            ((k1 - k2) / 2.0, (k2 - k1) / 2.0)
        } else {
            (-k2, -k1)
        };
        if free1 {
            self.nudge_second_control(side1.anchor, side1.handle, second1, target1, CURVATURE_DAMP);
        }
        if free2 {
            self.nudge_second_control(side2.anchor, side2.handle, second2, target2, CURVATURE_DAMP);
        }
        Ok(Value::Null)
    }

    /// G2 between a spline END (or any anchor — the math is the same) and a
    /// circle/arc: the spline's curvature there must be the circle's `σ/ρ`, with
    /// the side sign `σ` frozen from the configuration the solve arrives at,
    /// exactly like the existing tangent side constants. Emits the tangency too,
    /// because a spline-to-circle join has no implied G1 anywhere.
    ///
    /// Pair it with a coincident (anchor ≡ rim point) so the anchor actually
    /// lies on the circle; curvature and tangency alone do not place it.
    fn c_curvature_circle(
        &mut self,
        constraint: &mut HotConstraint,
        side: SplineCurvatureSide,
        center: usize,
        boundary: usize,
    ) -> CResult {
        if self.is_line_pair(center, boundary) {
            constraint.set_error(Value::String(
                "Curvature: a line has no curvature to match".into(),
            ));
            return Ok(Value::Null);
        }
        let tolerance = self.tolerance();
        let Some(second) = side.second else {
            constraint.set_error(Value::String(
                "Curvature: the spline control polygon is incomplete".into(),
            ));
            return Ok(Value::Null);
        };
        let Some((curvature, handle_len)) = self.spline_side_curvature(&side, second) else {
            constraint.set_error(Value::String(
                "Curvature: spline tangent is degenerate".into(),
            ));
            return Ok(Value::Null);
        };
        if handle_len < tolerance {
            constraint.set_error(Value::String(
                "Curvature: spline tangent is degenerate".into(),
            ));
            return Ok(Value::Null);
        }
        let radius = self.distance(center, boundary);
        if radius < tolerance {
            constraint.set_error(Value::String("Curvature: the circle is degenerate".into()));
            return Ok(Value::Null);
        }
        if self.distance(center, side.anchor) < tolerance {
            constraint.set_error(Value::String(
                "Curvature: spline anchor coincides with the circle center".into(),
            ));
            return Ok(Value::Null);
        }

        let (g1_delta, seg_angle, radial_angle) =
            self.spline_circle_g1_delta(side.anchor, side.handle, center);
        let sigma = circle_curvature_sign(
            self.point_xy(side.anchor),
            self.point_xy(side.handle),
            self.point_xy(center),
        );
        let residual = circle_curvature_residual(curvature, radius, sigma);
        let g1_ok = g1_delta.abs() < tolerance;
        // Same dimensionless reading as the joint lane: the residual carries a
        // factor ρ, so the bar does too.
        if g1_ok && residual.abs() <= tolerance * radius {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }

        let involved = [side.anchor, side.handle, second, center, boundary];
        if involved.iter().all(|&i| self.points[i].fixed) {
            constraint.set_error(Value::String(
                "Curvature: the spline end and the circle are fully fixed".into(),
            ));
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(if !g1_ok {
            format!(
                "Curvature constraint not satisfied\n        spline tangent {} not perpendicular to radius {}",
                fmt_number(seg_angle),
                fmt_number(radial_angle)
            )
        } else {
            format!(
                "Curvature constraint not satisfied\n        spline curvature {} != {}",
                fmt_number(curvature),
                fmt_number(sigma / radius)
            )
        }));

        if !g1_ok {
            self.rotate_spline_and_circle(side.anchor, side.handle, center, boundary, g1_delta);
        }
        self.nudge_second_control(
            side.anchor,
            side.handle,
            second,
            sigma / radius,
            CURVATURE_DAMP,
        );
        Ok(Value::Null)
    }

    /// `(κ, |handle − anchor|)` for one side of a joint, read from the live
    /// points. `None` for a degenerate handle.
    fn spline_side_curvature(
        &self,
        side: &SplineCurvatureSide,
        second: usize,
    ) -> Option<(f64, f64)> {
        spline_end_curvature(
            self.point_xy(side.anchor),
            self.point_xy(side.handle),
            self.point_xy(second),
        )
    }

    /// Move a spline's SECOND control point (`P₃ᵢ±₂`) along the end normal so
    /// the curvature there moves toward `target` (signed in the anchor→handle
    /// orientation). `κ = (2/3)·h/a²` with `h` the signed perpendicular offset
    /// of that control from the tangent line and `a` the handle length, so the
    /// target offset is `h* = (3/2)·target·a²`; `damp` in `(0, 1]` scales the
    /// step. The handle and anchor are never touched, so the tangent — and the
    /// target itself — is unchanged by the move. Returns whether it moved.
    fn nudge_second_control(
        &mut self,
        anchor: usize,
        handle: usize,
        second: usize,
        target: f64,
        damp: f64,
    ) -> bool {
        if self.points[second].fixed {
            return false;
        }
        let [ax, ay] = self.point_xy(anchor);
        let [hx, hy] = self.point_xy(handle);
        let (tx, ty) = (hx - ax, hy - ay);
        let length = tx.hypot(ty);
        if !(length > SPLINE_HANDLE_EPS) {
            return false;
        }
        let (ux, uy) = (tx / length, ty / length);
        let (nx, ny) = (-uy, ux);
        let offset = (self.points[second].x - hx) * nx + (self.points[second].y - hy) * ny;
        let step = (1.5 * target * length * length - offset) * damp;
        if !step.is_finite() || step == 0.0 {
            return false;
        }
        self.points[second].x += nx * step;
        self.points[second].y += ny * step;
        true
    }

    /// One point's coordinates as an array, for the shared curvature formulas.
    pub(super) fn point_xy(&self, index: usize) -> [f64; 2] {
        [self.points[index].x, self.points[index].y]
    }
}

/// Relaxation damping for the curvature nudge (the plan's 0.5).
const CURVATURE_DAMP: f64 = 0.5;

/// Signed curvature of the cubic Bézier end whose first three control points in
/// polygon order are `anchor, handle, second`, measured along the ANCHOR→HANDLE
/// direction, together with the handle length `a = |handle − anchor|`:
///
/// ```text
/// κ = (2/3)·cross(H − A, H2 − H) / |H − A|³
/// ```
///
/// This is the cubic hodograph at that anchor (`B' = 3(H − A)`,
/// `B'' = 6(H2 − 2H + A)`, `κ = cross(B', B'')/|B'|³`) — see the plan. The
/// span's own parameter direction agrees with anchor→handle at a START anchor
/// and reverses at an END anchor, and reversing a curve negates its SIGNED
/// curvature: so this is the curvature read "leaving the anchor along the
/// handle", and a side that ARRIVES at a joint contributes `−κ`.
///
/// `None` when the handle is degenerate — there is no tangent to sign against.
pub(super) fn spline_end_curvature(
    anchor: [f64; 2],
    handle: [f64; 2],
    second: [f64; 2],
) -> Option<(f64, f64)> {
    let (tx, ty) = (handle[0] - anchor[0], handle[1] - anchor[1]);
    let length = tx.hypot(ty);
    if !(length > SPLINE_HANDLE_EPS) {
        return None;
    }
    let (dx, dy) = (second[0] - handle[0], second[1] - handle[1]);
    let curvature = (2.0 / 3.0) * (tx * dy - ty * dx) / (length * length * length);
    Some((curvature, length))
}

/// The G2 residual at a joint from the two sides' outward curvatures and handle
/// lengths: `(κ₁ − κ₂)·ℓ²` with `κ₁ = −k1` (side 1 arrives at the joint),
/// `κ₂ = k2` and `ℓ = (a1 + a2)/2`.
///
/// LENGTH¹ units, deliberately: the LM polish minimizes the RAW residuals
/// through `(JᵀJ + λI)Δ = −Jᵀr`, so an atom in curvature or area units would be
/// weighted against the length atoms by an accident of scale. (The conflict
/// diagnosis no longer needs it — it divides each residual by its own Jacobian
/// row norm — but the polish still does.)
pub(super) fn joint_curvature_residual(k1: f64, k2: f64, a1: f64, a2: f64) -> f64 {
    let length = 0.5 * (a1 + a2);
    (-k1 - k2) * length * length
}

/// The G2 residual between a spline end of curvature `k` and a circle of radius
/// `rho` on side `sigma`: `(κρ − σ)·ρ`, also in length¹ units.
pub(super) fn circle_curvature_residual(k: f64, rho: f64, sigma: f64) -> f64 {
    (k * rho - sigma) * rho
}

/// The frozen side sign `σ = ±1`: which side of the spline's end tangent the
/// circle's centre sits on. A curve that turns toward the centre matches the
/// circle (`κρ = σ`); one that turns away hugs it from the far side and would
/// need `−σ`. Read from the configuration the solve arrives at, exactly like the
/// line/circle tangent's side constant — at tangency the centre sits square off
/// the tangent, so the sign is unambiguous there.
pub(super) fn circle_curvature_sign(anchor: [f64; 2], handle: [f64; 2], center: [f64; 2]) -> f64 {
    let (tx, ty) = (handle[0] - anchor[0], handle[1] - anchor[1]);
    let (cx, cy) = (center[0] - anchor[0], center[1] - anchor[1]);
    if tx * cy - ty * cx >= 0.0 {
        1.0
    } else {
        -1.0
    }
}
