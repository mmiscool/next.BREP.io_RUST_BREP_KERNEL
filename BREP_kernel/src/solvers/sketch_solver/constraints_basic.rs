use super::*;

impl Engine {
    // -------------------------------------------------------------------
    // Constraint functions
    // -------------------------------------------------------------------

    pub(super) fn c_horizontal(&mut self, constraint: &mut HotConstraint, a: usize, b: usize) -> CResult {
        let tolerance = self.tolerance();
        let y0 = self.points[a].y;
        let y1 = self.points[b].y;
        if (y0 - y1).abs() < tolerance {
            constraint.set_error(Value::Null);
        } else {
            constraint.set_error(Value::String(format!(
                "Horizontal constraint not satisfied\n        {} != {}",
                fmt_number(y0),
                fmt_number(y1)
            )));
        }
        let fixed_a = self.points[a].fixed;
        let fixed_b = self.points[b].fixed;
        if !fixed_a && !fixed_b {
            let avg = (y0 + y1) / 2.0;
            self.points[a].y = avg;
            self.points[b].y = avg;
        } else if !fixed_a {
            self.points[a].y = y1;
        } else if !fixed_b {
            self.points[b].y = y0;
        }
        Ok(Value::Null)
    }

    pub(super) fn c_vertical(&mut self, constraint: &mut HotConstraint, a: usize, b: usize) -> CResult {
        let tolerance = self.tolerance();
        let x0 = self.points[a].x;
        let x1 = self.points[b].x;
        if (x0 - x1).abs() < tolerance * 2.0 {
            constraint.set_error(Value::Null);
        } else {
            constraint.set_error(Value::String(format!(
                "Vertical constraint not satisfied\n        {} != {}",
                fmt_number(x0),
                fmt_number(x1)
            )));
        }
        let fixed_a = self.points[a].fixed;
        let fixed_b = self.points[b].fixed;
        if !fixed_a && !fixed_b {
            let avg = (x0 + x1) / 2.0;
            self.points[a].x = avg;
            self.points[b].x = avg;
        } else if !fixed_a {
            self.points[a].x = x1;
        } else if !fixed_b {
            self.points[b].x = x0;
        }
        Ok(Value::Null)
    }

    fn resolve_distance_target(
        &self,
        constraint: &mut HotConstraint,
        requested_target: f64,
    ) -> f64 {
        if !is_distance_ctype(constraint.ctype) || !requested_target.is_finite() {
            return requested_target;
        }
        let tolerance = self.tolerance();
        let previous_requested = constraint.req_target;
        let mut applied_target = constraint
            .app_target
            .unwrap_or(previous_requested.unwrap_or(requested_target));
        let target_changed = previous_requested
            .map(|prev| (requested_target - prev).abs() > tolerance)
            .unwrap_or(false);
        if previous_requested.is_none() {
            constraint.set_throttle(false);
            applied_target = requested_target;
        } else if target_changed {
            let rel =
                relative_delta_ratio(requested_target, previous_requested.unwrap(), tolerance);
            if rel > self.settings.distance_slide_threshold_ratio {
                constraint.set_throttle(true);
            } else {
                constraint.set_throttle(false);
                applied_target = requested_target;
            }
        }
        constraint.set_req_target(requested_target);
        if !constraint.throttle_truthy {
            constraint.set_app_target(requested_target);
            return requested_target;
        }
        if constraint.pass_token.as_deref() == Some(self.pass_token.as_str()) {
            return constraint.app_target.unwrap_or(applied_target);
        }
        let delta = requested_target - applied_target;
        let abs_delta = delta.abs();
        if abs_delta <= tolerance {
            constraint.set_throttle(false);
            constraint.set_app_target(requested_target);
            constraint.set_pass_token(Some(self.pass_token.clone()));
            return requested_target;
        }
        let max_step = self
            .settings
            .distance_slide_min_step
            .max(abs_delta * self.settings.distance_slide_step_ratio);
        let step = abs_delta.min(max_step);
        applied_target += js_sign(delta) * step;
        if (requested_target - applied_target).abs() <= tolerance {
            applied_target = requested_target;
            constraint.set_throttle(false);
        }
        constraint.set_app_target(applied_target);
        constraint.set_pass_token(Some(self.pass_token.clone()));
        applied_target
    }

    pub(super) fn c_distance(
        &mut self,
        constraint: &mut HotConstraint,
        a: usize,
        b: usize,
        constraint_value: f64,
    ) -> CResult {
        let tolerance = self.tolerance();
        let mut target_distance = constraint_value;
        let mut dx = self.points[b].x - self.points[a].x;
        let mut dy = self.points[b].y - self.points[a].y;
        let mut current_distance = self.distance(a, b);

        if constraint_value.is_nan() {
            target_distance = current_distance;
            constraint.set_value(current_distance);
            if is_distance_ctype(constraint.ctype) {
                constraint.set_req_target(current_distance);
                constraint.set_app_target(current_distance);
                constraint.set_throttle(false);
                constraint.set_pass_token(None);
            }
        }

        target_distance = self.resolve_distance_target(constraint, target_distance);

        let diff = round_to_decimals(target_distance.abs() - current_distance, 4);
        if diff.abs() == 0.0 {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }
        constraint.set_error(Value::String(format!(
            "Distance constraint not satisfied\n        {} != {}",
            fmt_number(target_distance),
            fmt_number(current_distance)
        )));

        if current_distance == 0.0 {
            current_distance = 1.0;
            dx = 1.0;
            dy = 1.0;
        }

        let ratio = diff / current_distance;
        let mut offset_x = dx * ratio * 0.5;
        let mut offset_y = dy * ratio * 0.5;
        let direction = if target_distance >= 0.0 { 1.0 } else { -1.0 };

        let max_move = 1.0;
        let mut move_distance = (offset_x * offset_x + offset_y * offset_y).sqrt();
        if move_distance == 0.0 || move_distance.is_nan() {
            move_distance = tolerance;
        }
        if move_distance > max_move {
            let scale = max_move / move_distance;
            offset_x *= scale;
            offset_y *= scale;
        }

        let fixed_a = self.points[a].fixed;
        let fixed_b = self.points[b].fixed;
        if !fixed_a && !fixed_b {
            self.points[a].x -= offset_x * direction;
            self.points[a].y -= offset_y * direction;
            self.points[b].x += offset_x * direction;
            self.points[b].y += offset_y * direction;
        } else if !fixed_a {
            self.points[a].x -= offset_x * 2.0 * direction;
            self.points[a].y -= offset_y * 2.0 * direction;
        } else if !fixed_b {
            self.points[b].x += offset_x * 2.0 * direction;
            self.points[b].y += offset_y * 2.0 * direction;
        } else {
            let message = format!(
                "points {} and {} are both fixed",
                fmt_id(&self.points[a].id),
                fmt_id(&self.points[b].id)
            );
            constraint.set_error(Value::String(message.clone()));
            return Ok(Value::String(message));
        }
        Ok(Value::Null)
    }

    pub(super) fn c_point_line_distance(
        &mut self,
        constraint: &mut HotConstraint,
        indices: &[Option<usize>],
        constraint_value: f64,
    ) -> CResult {
        let a = indices.first().copied().flatten();
        let b = indices.get(1).copied().flatten();
        let c = indices.get(2).copied().flatten();
        let (Some(a), Some(b), Some(c)) = (a, b, c) else {
            constraint.set_error(Value::String(
                "Line to Point Distance requires 3 points".into(),
            ));
            return Ok(Value::Null);
        };
        let tolerance = self.tolerance();

        let dx = self.points[b].x - self.points[a].x;
        let dy = self.points[b].y - self.points[a].y;
        let len_sq = dx * dx + dy * dy;
        if len_sq < tolerance * tolerance {
            // Degenerate line: fallback to point-point distance from A to C.
            return self.c_distance(constraint, a, c, constraint_value);
        }

        let len = len_sq.sqrt();
        let nx = -dy / len;
        let ny = dx / len;
        let t = ((self.points[c].x - self.points[a].x) * dx
            + (self.points[c].y - self.points[a].y) * dy)
            / len_sq;
        let signed_distance =
            (self.points[c].x - self.points[a].x) * nx + (self.points[c].y - self.points[a].y) * ny;

        let mut target_distance = if constraint_value.is_finite() {
            constraint_value
        } else {
            f64::NAN
        };
        if !target_distance.is_finite() {
            target_distance = signed_distance.abs();
            constraint.set_value(target_distance);
            constraint.set_req_target(target_distance);
            constraint.set_app_target(target_distance);
            constraint.set_throttle(false);
            constraint.set_pass_token(None);
            let init_sign = js_sign(signed_distance);
            let seeded = if init_sign == 0.0 { 1.0 } else { init_sign };
            constraint.set_line_sign(seeded);
        }

        target_distance = self.resolve_distance_target(constraint, target_distance);

        let mut side = constraint.line_sign;
        if !side.is_finite() || side == 0.0 {
            side = js_sign(signed_distance);
            if !side.is_finite() || side == 0.0 {
                side = 1.0;
            }
        }
        if target_distance < 0.0 {
            side = -1.0;
        }
        constraint.set_line_sign(side);

        let target_signed_distance = target_distance.abs() * side;
        let err = signed_distance - target_signed_distance;
        if err.abs() <= tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }

        constraint.set_error(Value::String(format!(
            "Line to Point Distance not satisfied\n        {} != {}",
            fmt_number(target_distance),
            fmt_number(signed_distance.abs())
        )));

        let mut denom = 0.0;
        if !self.points[c].fixed {
            denom += 1.0;
        }
        if !self.points[a].fixed {
            denom += (1.0 - t) * (1.0 - t);
        }
        if !self.points[b].fixed {
            denom += t * t;
        }
        if denom <= 0.0 {
            constraint.set_error(Value::String(format!(
                "points {}, {}, and {} are all fixed",
                fmt_id(&self.points[a].id),
                fmt_id(&self.points[b].id),
                fmt_id(&self.points[c].id)
            )));
            return Ok(Value::Null);
        }

        let factor = 1.0 / denom;
        let corr_x = err * nx;
        let corr_y = err * ny;

        if !self.points[c].fixed {
            self.points[c].x -= corr_x * factor;
            self.points[c].y -= corr_y * factor;
        }
        if !self.points[a].fixed {
            self.points[a].x += corr_x * (1.0 - t) * factor;
            self.points[a].y += corr_y * (1.0 - t) * factor;
        }
        if !self.points[b].fixed {
            self.points[b].x += corr_x * t * factor;
            self.points[b].y += corr_y * t * factor;
        }
        Ok(Value::Null)
    }

    pub(super) fn c_equal_distance(
        &mut self,
        constraint: &mut HotConstraint,
        a: usize,
        b: usize,
        c: usize,
        d: usize,
    ) -> CResult {
        let line1_distance = self.find_distance_constraint_on_pair(a, b);
        let line2_distance = self.find_distance_constraint_on_pair(c, d);

        let mut avg_distance = f64::NAN;
        let mut line1_moving = false;
        let mut line2_moving = false;
        match (line1_distance, line2_distance) {
            (None, None) => {
                let distance_ab = self.distance(b, a);
                let distance_cd = self.distance(d, c);
                avg_distance = (distance_ab + distance_cd) / 2.0;
                line1_moving = true;
                line2_moving = true;
            }
            (Some(value), None) => {
                avg_distance = value;
                line2_moving = true;
            }
            (None, Some(value)) => {
                avg_distance = value;
                line1_moving = true;
            }
            (Some(_), Some(_)) => {
                let message = "Both lines have a distance constraint applied to them".to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            }
        }

        if line1_moving {
            let result = self.c_distance(constraint, a, b, avg_distance)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        }
        if line2_moving {
            let result = self.c_distance(constraint, c, d, avg_distance)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        }
        Ok(Value::Null)
    }

    pub(super) fn c_parallel(&mut self, constraint: &mut HotConstraint, indices: &[Option<usize>]) -> CResult {
        let p0 = self.req(indices, 0)?;
        let p1 = self.req(indices, 1)?;
        let p2 = self.req(indices, 2)?;
        let p3 = self.req(indices, 3)?;
        let line1_vertical = self.participate_in_constraint(CType::Vertical, &[p0, p1]);
        let line1_horizontal = self.participate_in_constraint(CType::Horizontal, &[p0, p1]);
        let line2_vertical = self.participate_in_constraint(CType::Vertical, &[p2, p3]);
        let line2_horizontal = self.participate_in_constraint(CType::Horizontal, &[p2, p3]);

        if line1_vertical {
            if line2_vertical {
                let message = "Both lines have a vertical constraint applied to them".to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            } else if line2_horizontal {
                let message =
                    "One line has a vertical constraint and the other has a horizontal constraint"
                        .to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            }
            let result = self.c_vertical(constraint, p2, p3)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        } else if line1_horizontal {
            if line2_vertical {
                let message =
                    "One line has a vertical constraint and the other has a horizontal constraint"
                        .to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            } else if line2_horizontal {
                let message = "Both lines have a horizontal constraint applied to them".to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            }
            let result = self.c_horizontal(constraint, p2, p3)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        } else if line2_vertical {
            let result = self.c_vertical(constraint, p0, p1)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        } else if line2_horizontal {
            let result = self.c_horizontal(constraint, p0, p1)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        } else {
            let line1_angle = self.calculate_angle(p0, p1);
            let line2_angle = self.calculate_angle(p2, p3);
            let angle_difference = ((line1_angle - line2_angle) + 360.0) % 360.0;
            let mut new_set_angle = 0.0;
            if angle_difference > 90.0 {
                new_set_angle = 180.0;
            }
            if angle_difference > 180.0 {
                new_set_angle = 180.0;
            }
            if angle_difference > 270.0 {
                new_set_angle = 360.0;
            }
            return self.c_angle(constraint, indices, new_set_angle);
        }
        Ok(Value::Null)
    }

    pub(super) fn c_perpendicular(
        &mut self,
        constraint: &mut HotConstraint,
        indices: &[Option<usize>],
    ) -> CResult {
        let p0 = self.req(indices, 0)?;
        let p1 = self.req(indices, 1)?;
        let p2 = self.req(indices, 2)?;
        let p3 = self.req(indices, 3)?;
        let line1_vertical = self.participate_in_constraint(CType::Vertical, &[p0, p1]);
        let line1_horizontal = self.participate_in_constraint(CType::Horizontal, &[p0, p1]);
        let line2_vertical = self.participate_in_constraint(CType::Vertical, &[p2, p3]);
        let line2_horizontal = self.participate_in_constraint(CType::Horizontal, &[p2, p3]);

        if line1_vertical {
            if line2_vertical {
                let message = "Both lines have a vertical constraint applied to them".to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            } else if line2_horizontal {
                let message =
                    "One line has a vertical constraint and the other has a horizontal constraint"
                        .to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            }
            let result = self.c_horizontal(constraint, p2, p3)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        } else if line1_horizontal {
            if line2_vertical {
                let message =
                    "One line has a vertical constraint and the other has a horizontal constraint"
                        .to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            } else if line2_horizontal {
                let message = "Both lines have a horizontal constraint applied to them".to_string();
                constraint.set_error(Value::String(message.clone()));
                return Ok(Value::String(message));
            }
            let result = self.c_vertical(constraint, p2, p3)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        } else if line2_vertical {
            let result = self.c_horizontal(constraint, p0, p1)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        } else if line2_horizontal {
            let result = self.c_vertical(constraint, p0, p1)?;
            if truthy(Some(&result)) {
                return Ok(result);
            }
        } else {
            let line1_angle = self.calculate_angle(p0, p1);
            let line2_angle = self.calculate_angle(p2, p3);
            let difference = ((line1_angle - line2_angle) + 360.0) % 360.0;
            let new_target_angle = if difference <= 180.0 { 90.0 } else { 270.0 };
            return self.c_angle(constraint, indices, new_target_angle);
        }
        Ok(Value::Null)
    }

    pub(super) fn c_angle(
        &mut self,
        constraint: &mut HotConstraint,
        indices: &[Option<usize>],
        constraint_value: f64,
    ) -> CResult {
        let p1 = self.req(indices, 0)?;
        let p2 = self.req(indices, 1)?;
        let p3 = self.req(indices, 2)?;
        let p4 = self.req(indices, 3)?;
        let tolerance = self.tolerance();

        let line1_angle = self.calculate_angle(p1, p2);
        let line2_angle = self.calculate_angle(p3, p4);
        let difference_between_angles = line1_angle - line2_angle;

        if constraint.value_nullish {
            // Seed with the current measured angle (normalize into [0, 360)).
            constraint.set_value(round_to_decimals(
                normalize_angle(difference_between_angles),
                4,
            ));
        } else {
            let value_number = constraint
                .value_num
                .unwrap_or_else(|| js_number(constraint.raw.get("value")));
            if value_number < 0.0 {
                constraint.set_value(value_number.abs());
                // Swap the raw point ids and their resolved indices [2,3,1,0].
                let pick_id = |slot: usize| {
                    constraint
                        .point_ids
                        .get(slot)
                        .cloned()
                        .unwrap_or(Value::Null)
                };
                let pick_idx = |slot: usize| indices.get(slot).copied().flatten();
                let new_ids = vec![pick_id(2), pick_id(3), pick_id(1), pick_id(0)];
                let new_idx = vec![pick_idx(2), pick_idx(3), pick_idx(1), pick_idx(0)];
                constraint.point_ids = new_ids;
                constraint.point_idx = new_idx;
                constraint.points_written = true;
                // The stored bits signature no longer corresponds slot-wise;
                // keep only the string form for future comparisons.
                if let PrevPoints::Bits(_, text) = &constraint.prev_points {
                    constraint.prev_points = PrevPoints::Str(text.clone());
                }
                return Ok(Value::Null);
            } else if value_number > 360.0 {
                constraint.set_value(normalize_angle(value_number));
                return Ok(Value::Null);
            }
        }

        let current_angle = normalize_angle(difference_between_angles);
        let mut desired_angle = if constraint_value.is_finite() {
            constraint_value
        } else {
            constraint.value_parsed
        };
        if !desired_angle.is_finite() {
            desired_angle = current_angle;
        }
        let target_angle = normalize_angle(desired_angle);

        let delta_raw = shortest_angle_delta(target_angle, current_angle);

        if delta_raw.abs() < tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }

        if delta_raw.abs() > tolerance {
            constraint.set_error(Value::String(format!(
                "Angle constraint not satisfied\n            {} != {}\n            Diff: {:.4}\n            ",
                fmt_number(target_angle),
                fmt_number(current_angle),
                delta_raw.abs()
            )));
        } else {
            constraint.set_error(Value::Null);
        }

        let mut line1_moving = !(self.points[p1].fixed && self.points[p2].fixed);
        let mut line2_moving = !(self.points[p3].fixed && self.points[p4].fixed);

        if self.participate_in_constraint(CType::Horizontal, &[p1, p2]) {
            line1_moving = false;
        }
        if self.participate_in_constraint(CType::Horizontal, &[p3, p4]) {
            line2_moving = false;
        }
        if self.participate_in_constraint(CType::Vertical, &[p1, p2]) {
            line1_moving = false;
        }
        if self.participate_in_constraint(CType::Vertical, &[p3, p4]) {
            line2_moving = false;
        }

        if !line1_moving && !line2_moving {
            return Ok(Value::Null);
        }

        let max_step = 1.5;
        let mut delta = delta_raw;
        if delta.abs() > max_step {
            delta = js_sign(delta) * max_step;
        }

        let mut rotation_line1 = 0.0;
        let mut rotation_line2 = 0.0;
        if line1_moving && line2_moving {
            rotation_line1 = delta / 2.0;
            rotation_line2 = -delta / 2.0;
        } else if line1_moving {
            rotation_line1 = delta;
        } else if line2_moving {
            rotation_line2 = -delta;
        }

        if line1_moving && rotation_line1 != 0.0 && !rotation_line1.is_nan() {
            if self.points[p1].fixed {
                let (cx, cy) = (self.points[p1].x, self.points[p1].y);
                self.rotate_point(cx, cy, p2, rotation_line1);
            } else if self.points[p2].fixed {
                let (cx, cy) = (self.points[p2].x, self.points[p2].y);
                self.rotate_point(cx, cy, p1, rotation_line1);
            } else {
                let mid_x = (self.points[p1].x + self.points[p2].x) / 2.0;
                let mid_y = (self.points[p1].y + self.points[p2].y) / 2.0;
                self.rotate_point(mid_x, mid_y, p1, rotation_line1);
                self.rotate_point(mid_x, mid_y, p2, rotation_line1);
            }
        }

        if line2_moving && rotation_line2 != 0.0 && !rotation_line2.is_nan() {
            if self.points[p3].fixed {
                let (cx, cy) = (self.points[p3].x, self.points[p3].y);
                self.rotate_point(cx, cy, p4, rotation_line2);
            } else if self.points[p4].fixed {
                let (cx, cy) = (self.points[p4].x, self.points[p4].y);
                self.rotate_point(cx, cy, p3, rotation_line2);
            } else {
                let mid_x = (self.points[p3].x + self.points[p4].x) / 2.0;
                let mid_y = (self.points[p3].y + self.points[p4].y) / 2.0;
                self.rotate_point(mid_x, mid_y, p3, rotation_line2);
                self.rotate_point(mid_x, mid_y, p4, rotation_line2);
            }
        }
        Ok(Value::Null)
    }

    pub(super) fn c_coincident(&mut self, constraint: &mut HotConstraint, a: usize, b: usize) -> CResult {
        if self.points[a].fixed && self.points[b].fixed {
            if self.participate_in_constraint(CType::Ground, &[a])
                && self.participate_in_constraint(CType::Ground, &[b])
            {
                constraint.set_error(Value::String("Both points are fixed".into()));
            }
            return Ok(Value::Null);
        }

        if self.points[a].x == self.points[b].x && self.points[a].y == self.points[b].y {
            constraint.set_error(Value::Null);
        } else if !self.points[a].fixed && !self.points[b].fixed {
            let avg_x = (self.points[a].x + self.points[b].x) / 2.0;
            let avg_y = (self.points[a].y + self.points[b].y) / 2.0;
            self.points[a].x = avg_x;
            self.points[a].y = avg_y;
            self.points[b].x = avg_x;
            self.points[b].y = avg_y;
        } else if !self.points[a].fixed {
            self.points[a].x = self.points[b].x;
            self.points[a].y = self.points[b].y;
            self.points[a].fixed = true;
        } else if !self.points[b].fixed {
            self.points[b].x = self.points[a].x;
            self.points[b].y = self.points[a].y;
            self.points[b].fixed = true;
        }

        if self.points[a].fixed || self.points[b].fixed {
            self.points[a].fixed = true;
            self.points[b].fixed = true;
        }
        Ok(Value::Null)
    }

    pub(super) fn c_point_on_line(
        &mut self,
        constraint: &mut HotConstraint,
        a: usize,
        b: usize,
        c: usize,
    ) -> CResult {
        let tolerance = self.tolerance();
        let dx = self.points[b].x - self.points[a].x;
        let dy = self.points[b].y - self.points[a].y;
        let len_sq = dx * dx + dy * dy;

        if len_sq < tolerance {
            let dist = self.distance(a, c);
            if dist > tolerance {
                constraint.set_error(Value::String(
                    "Point on Line: Line is degenerate (points too close) and Point C is not coincident."
                        .into(),
                ));
                if !self.points[c].fixed {
                    self.points[c].x = self.points[a].x;
                    self.points[c].y = self.points[a].y;
                } else if !self.points[a].fixed {
                    self.points[a].x = self.points[c].x;
                    self.points[a].y = self.points[c].y;
                    if !self.points[b].fixed {
                        self.points[b].x = self.points[c].x;
                        self.points[b].y = self.points[c].y;
                    }
                }
            } else {
                constraint.set_error(Value::Null);
            }
            return Ok(Value::Null);
        }

        let t = ((self.points[c].x - self.points[a].x) * dx
            + (self.points[c].y - self.points[a].y) * dy)
            / len_sq;
        let proj_x = self.points[a].x + t * dx;
        let proj_y = self.points[a].y + t * dy;
        let err_x = self.points[c].x - proj_x;
        let err_y = self.points[c].y - proj_y;
        let err_dist = (err_x * err_x + err_y * err_y).sqrt();

        if err_dist < tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }

        constraint.set_error(Value::String(format!(
            "Point on Line not satisfied. Dist: {:.4}",
            err_dist
        )));

        let mut denom = 0.0;
        if !self.points[c].fixed {
            denom += 1.0;
        }
        if !self.points[a].fixed {
            denom += (1.0 - t) * (1.0 - t);
        }
        if !self.points[b].fixed {
            denom += t * t;
        }
        if denom == 0.0 {
            return Ok(Value::Null);
        }

        let k = 1.0;
        let factor = k / denom;

        let mut expansion_x = 0.0;
        let mut expansion_y = 0.0;
        if (!self.points[a].fixed || !self.points[b].fixed) && len_sq > tolerance {
            let len = len_sq.sqrt();
            let ux = dx / len;
            let uy = dy / len;
            let exp_force = (err_dist / len) * 0.1 * factor;
            expansion_x = ux * exp_force;
            expansion_y = uy * exp_force;
        }

        if !self.points[c].fixed {
            self.points[c].x -= err_x * factor;
            self.points[c].y -= err_y * factor;
        }
        if !self.points[a].fixed {
            self.points[a].x += err_x * (1.0 - t) * factor;
            self.points[a].y += err_y * (1.0 - t) * factor;
            self.points[a].x -= expansion_x;
            self.points[a].y -= expansion_y;
        }
        if !self.points[b].fixed {
            self.points[b].x += err_x * t * factor;
            self.points[b].y += err_y * t * factor;
            self.points[b].x += expansion_x;
            self.points[b].y += expansion_y;
        }
        Ok(Value::Null)
    }

    pub(super) fn c_midpoint(
        &mut self,
        constraint: &mut HotConstraint,
        a: usize,
        b: usize,
        c: usize,
    ) -> CResult {
        let tolerance = self.tolerance();
        let rx = 2.0 * self.points[c].x - self.points[a].x - self.points[b].x;
        let ry = 2.0 * self.points[c].y - self.points[a].y - self.points[b].y;

        if rx.abs() < tolerance && ry.abs() < tolerance {
            constraint.set_error(Value::Null);
            return Ok(Value::Null);
        }

        constraint.set_error(Value::String(format!(
            "Midpoint constraint not satisfied. Error: {:.4}",
            rx.hypot(ry)
        )));

        let mut denom = 0.0;
        if !self.points[a].fixed {
            denom += 1.0;
        }
        if !self.points[b].fixed {
            denom += 1.0;
        }
        if !self.points[c].fixed {
            denom += 4.0;
        }
        if denom == 0.0 {
            constraint.set_error(Value::String(
                "All points fixed in Midpoint constraint".into(),
            ));
            return Ok(Value::Null);
        }

        let alpha_x = -rx / denom;
        let alpha_y = -ry / denom;
        if !self.points[a].fixed {
            self.points[a].x -= alpha_x;
            self.points[a].y -= alpha_y;
        }
        if !self.points[b].fixed {
            self.points[b].x -= alpha_x;
            self.points[b].y -= alpha_y;
        }
        if !self.points[c].fixed {
            self.points[c].x += alpha_x * 2.0;
            self.points[c].y += alpha_y * 2.0;
        }
        Ok(Value::Null)
    }
}
