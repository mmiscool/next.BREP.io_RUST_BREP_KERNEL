use super::*;

impl HotConstraint {
    pub(super) fn from_map(
        map: Map<String, Value>,
        point_index: &HashMap<PointKey, usize>,
    ) -> Result<Self, String> {
        let type_str = map.get("type").and_then(Value::as_str).map(str::to_string);
        let ctype = type_str.as_deref().map(ctype_from).unwrap_or(CType::Other);
        let points = map
            .get("points")
            .and_then(Value::as_array)
            .ok_or_else(|| "constraint.points must be an array".to_string())?;
        let point_ids: Vec<Value> = points.to_vec();
        let point_idx: Vec<Option<usize>> = point_ids
            .iter()
            .map(|id| point_index.get(&point_key(id)).copied())
            .collect();

        let raw_value = map.get("value");
        let value_parsed = js_parse_float(raw_value);
        let value_nullish = matches!(raw_value, None | Some(Value::Null));
        let value_cv = cv_from_raw(raw_value);

        let status_solved = map.get("status").and_then(Value::as_str) == Some("solved");
        let prev_solve = match map.get("_previousSolveValue") {
            None => PrevSolve::Missing,
            Some(Value::Null) => PrevSolve::Num(f64::NAN),
            Some(Value::Number(number)) => PrevSolve::Num(number.as_f64().unwrap_or(f64::NAN)),
            Some(_) => PrevSolve::OtherType,
        };
        let prev_points = match map.get("previousPointValues") {
            None => PrevPoints::Missing,
            Some(Value::String(text)) => PrevPoints::Str(text.clone()),
            Some(_) => PrevPoints::PresentNonString,
        };

        let get_finite = |key: &str| -> Option<f64> {
            match map.get(key) {
                Some(Value::Number(number)) => number.as_f64().filter(|f| f.is_finite()),
                _ => None,
            }
        };

        Ok(Self {
            temporary: truthy(map.get("temporary")),
            req_target: get_finite("_distanceRequestedTarget"),
            app_target: get_finite("_distanceAppliedTarget"),
            throttle_true: map.get("_distanceThrottleActive") == Some(&Value::Bool(true)),
            throttle_truthy: truthy(map.get("_distanceThrottleActive")),
            pass_token: map
                .get("_distanceLastAppliedPassToken")
                .and_then(Value::as_str)
                .map(str::to_string),
            line_sign: js_number(map.get("_linePointDistanceSign")),
            foot_t: get_finite("_splineFootT"),
            ctype,
            type_str,
            point_ids,
            point_idx,
            value_num: None,
            value_parsed,
            value_nullish,
            value_cv,
            status_solved,
            prev_solve,
            prev_points,
            error: None,
            raw: map,
            ..Default::default()
        })
    }

    pub(super) fn set_value(&mut self, value: f64) {
        self.value_num = Some(value);
        self.value_parsed = value;
        self.value_nullish = false;
        self.value_cv = value;
        self.value_written = true;
    }

    pub(super) fn set_error(&mut self, error: Value) {
        self.error = Some(error);
        self.error_written = true;
    }

    pub(super) fn set_throttle(&mut self, active: bool) {
        self.throttle_true = active;
        self.throttle_truthy = active;
        self.throttle_written = true;
    }

    pub(super) fn set_req_target(&mut self, value: f64) {
        self.req_target = if value.is_finite() { Some(value) } else { None };
        self.req_target_written = true;
    }

    pub(super) fn set_app_target(&mut self, value: f64) {
        self.app_target = if value.is_finite() { Some(value) } else { None };
        self.app_target_written = true;
    }

    pub(super) fn set_pass_token(&mut self, token: Option<String>) {
        self.pass_token = token;
        self.pass_token_written = true;
    }

    pub(super) fn set_line_sign(&mut self, value: f64) {
        self.line_sign = value;
        self.line_sign_written = true;
    }

    /// Persist the foot-parameter tangency's touch point as `span + t`. Written
    /// on every pass that resolves a foot — including the passes that find it
    /// already satisfied — so the seed tracks the configuration the solve is
    /// actually on, and so an interactive drag (relaxation only, `polish:false`)
    /// hands the next frame the touch point this frame ended with instead of the
    /// one the drag started from.
    pub(super) fn set_foot_t(&mut self, value: f64) {
        self.foot_t = if value.is_finite() { Some(value) } else { None };
        self.foot_t_written = true;
    }

    pub(super) fn matches_filter(&self, filter: &Filter) -> bool {
        match filter {
            Filter::All => true,
            Filter::Type(ctype, text) => {
                if *ctype != CType::Other {
                    self.ctype == *ctype
                } else {
                    self.ctype == CType::Other && self.type_str.as_deref() == Some(*text)
                }
            }
        }
    }

    pub(super) fn into_value(mut self) -> Value {
        if self.points_written {
            self.raw.insert(
                "points".into(),
                Value::Array(std::mem::take(&mut self.point_ids)),
            );
        }
        if self.value_written {
            if let Some(v) = self.value_num {
                self.raw.insert("value".into(), json_num(v));
            }
        }
        if self.status_written {
            self.raw.insert(
                "status".into(),
                Value::String(if self.status_solved {
                    "solved".into()
                } else {
                    String::new()
                }),
            );
        }
        if self.error_written {
            self.raw
                .insert("error".into(), self.error.take().unwrap_or(Value::Null));
        }
        if self.prev_solve_written {
            let value = match self.prev_solve {
                PrevSolve::Num(v) => json_num(v),
                _ => Value::Null,
            };
            self.raw.insert("_previousSolveValue".into(), value);
        }
        if self.prev_points_written {
            match &self.prev_points {
                PrevPoints::Bits(_, text) | PrevPoints::Str(text) => {
                    self.raw
                        .insert("previousPointValues".into(), Value::String(text.clone()));
                }
                _ => {}
            }
        }
        if self.req_target_written {
            self.raw.insert(
                "_distanceRequestedTarget".into(),
                self.req_target.map(json_num).unwrap_or(Value::Null),
            );
        }
        if self.app_target_written {
            self.raw.insert(
                "_distanceAppliedTarget".into(),
                self.app_target.map(json_num).unwrap_or(Value::Null),
            );
        }
        if self.throttle_written {
            self.raw.insert(
                "_distanceThrottleActive".into(),
                Value::Bool(self.throttle_true),
            );
        }
        if self.pass_token_written {
            self.raw.insert(
                "_distanceLastAppliedPassToken".into(),
                self.pass_token
                    .take()
                    .map(Value::String)
                    .unwrap_or(Value::Null),
            );
        }
        if self.line_sign_written {
            self.raw
                .insert("_linePointDistanceSign".into(), json_num(self.line_sign));
        }
        if self.foot_t_written {
            self.raw.insert(
                "_splineFootT".into(),
                self.foot_t.map(json_num).unwrap_or(Value::Null),
            );
        }
        Value::Object(self.raw)
    }
}
