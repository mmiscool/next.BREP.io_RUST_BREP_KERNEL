use super::*;

// ---------------------------------------------------------------------------
// JavaScript numeric semantics helpers
// ---------------------------------------------------------------------------

/// JavaScript `Math.round` (ties round toward +Infinity).
pub(super) fn js_round(x: f64) -> f64 {
    if !x.is_finite() {
        return x;
    }
    let floor = x.floor();
    let diff = x - floor;
    if diff >= 0.5 {
        floor + 1.0
    } else {
        floor
    }
}

/// mathHelpersMod.roundToDecimals.
pub(super) fn round_to_decimals(number: f64, decimals: i32) -> f64 {
    let k = 10f64.powi(decimals);
    js_round(number * k) / k
}

/// JavaScript `Math.sign` (preserves signed zero, NaN passes through).
pub(super) fn js_sign(x: f64) -> f64 {
    if x.is_nan() || x == 0.0 {
        x
    } else if x > 0.0 {
        1.0
    } else {
        -1.0
    }
}

/// JavaScript `parseFloat` applied to a JSON value (numbers pass through, strings are
/// parsed from their leading numeric prefix, everything else is NaN).
pub(super) fn js_parse_float(value: Option<&Value>) -> f64 {
    match value {
        Some(Value::Number(number)) => number.as_f64().unwrap_or(f64::NAN),
        Some(Value::String(text)) => parse_float_prefix(text),
        _ => f64::NAN,
    }
}

pub(super) fn parse_float_prefix(text: &str) -> f64 {
    let trimmed = text.trim_start();
    let bytes = trimmed.as_bytes();
    let mut end = 0usize;
    let mut seen_digit = false;
    let mut seen_dot = false;
    let mut seen_exp = false;
    let mut index = 0usize;
    if index < bytes.len() && (bytes[index] == b'+' || bytes[index] == b'-') {
        index += 1;
    }
    if trimmed[index..].starts_with("Infinity") {
        let sign = if bytes.first() == Some(&b'-') {
            -1.0
        } else {
            1.0
        };
        return sign * f64::INFINITY;
    }
    while index < bytes.len() {
        let b = bytes[index];
        if b.is_ascii_digit() {
            seen_digit = true;
            end = index + 1;
        } else if b == b'.' && !seen_dot && !seen_exp {
            seen_dot = true;
        } else if (b == b'e' || b == b'E') && seen_digit && !seen_exp {
            let mut next = index + 1;
            if next < bytes.len() && (bytes[next] == b'+' || bytes[next] == b'-') {
                next += 1;
            }
            if next < bytes.len() && bytes[next].is_ascii_digit() {
                seen_exp = true;
                index = next;
                end = index + 1;
                continue;
            } else {
                break;
            }
        } else {
            break;
        }
        index += 1;
    }
    if !seen_digit {
        return f64::NAN;
    }
    trimmed[..end].parse::<f64>().unwrap_or(f64::NAN)
}

/// JavaScript `Number(...)` coercion for the value shapes that appear in sketches.
pub(super) fn js_number(value: Option<&Value>) -> f64 {
    match value {
        None => f64::NAN,
        Some(Value::Null) => 0.0,
        Some(Value::Bool(flag)) => {
            if *flag {
                1.0
            } else {
                0.0
            }
        }
        Some(Value::Number(number)) => number.as_f64().unwrap_or(f64::NAN),
        Some(Value::String(text)) => {
            let trimmed = text.trim();
            if trimmed.is_empty() {
                0.0
            } else if trimmed == "Infinity" || trimmed == "+Infinity" {
                f64::INFINITY
            } else if trimmed == "-Infinity" {
                f64::NEG_INFINITY
            } else {
                trimmed.parse::<f64>().unwrap_or(f64::NAN)
            }
        }
        Some(_) => f64::NAN,
    }
}

/// Coerce a raw constraint `value` the way the distance path saw it:
/// null/undefined behave like the "seed from current" branch (NaN here),
/// everything else uses Number coercion.
pub(super) fn cv_from_raw(value: Option<&Value>) -> f64 {
    match value {
        None | Some(Value::Null) => f64::NAN,
        other => js_number(other),
    }
}

/// JavaScript `parseInt` for canonical constraint-point ids.
pub(super) fn js_parse_int(value: Option<&Value>) -> Option<i64> {
    match value {
        Some(Value::Number(number)) => {
            let float = number.as_f64()?;
            if float.is_finite() {
                Some(float.trunc() as i64)
            } else {
                None
            }
        }
        Some(Value::String(text)) => {
            let trimmed = text.trim_start();
            let bytes = trimmed.as_bytes();
            let mut index = 0usize;
            let mut sign = 1i64;
            if index < bytes.len() && (bytes[index] == b'+' || bytes[index] == b'-') {
                if bytes[index] == b'-' {
                    sign = -1;
                }
                index += 1;
            }
            let start = index;
            while index < bytes.len() && bytes[index].is_ascii_digit() {
                index += 1;
            }
            if index == start {
                return None;
            }
            trimmed[start..index].parse::<i64>().ok().map(|v| sign * v)
        }
        _ => None,
    }
}

/// JavaScript truthiness for JSON values.
pub(super) fn truthy(value: Option<&Value>) -> bool {
    match value {
        None | Some(Value::Null) => false,
        Some(Value::Bool(flag)) => *flag,
        Some(Value::Number(number)) => {
            let float = number.as_f64().unwrap_or(f64::NAN);
            float != 0.0 && !float.is_nan()
        }
        Some(Value::String(text)) => !text.is_empty(),
        Some(_) => true,
    }
}

/// Number formatting for signatures / error strings (JavaScript-like: integral values
/// print without a decimal point, negative zero prints as "0").
pub(super) fn fmt_number(x: f64) -> String {
    if x.is_nan() {
        return "NaN".to_string();
    }
    if x == 0.0 {
        return "0".to_string();
    }
    format!("{}", x)
}

/// Canonical sketch entity key. Numeric IDs normalize integral floats and signed
/// zero; string IDs are preserved. Other JSON values use their JSON spelling.
pub fn fmt_id(value: &Value) -> String {
    match value {
        Value::String(text) => text.clone(),
        Value::Number(number) => number
            .as_f64()
            .map(fmt_number)
            .unwrap_or_else(|| number.to_string()),
        Value::Bool(flag) => flag.to_string(),
        Value::Null => "null".to_string(),
        other => other.to_string(),
    }
}

/// JSON-safe number (JSON.stringify turns NaN/Infinity into null).
pub(super) fn json_num(x: f64) -> Value {
    serde_json::Number::from_f64(x)
        .map(Value::Number)
        .unwrap_or(Value::Null)
}

pub(super) fn normalize_angle(angle: f64) -> f64 {
    ((angle % 360.0) + 360.0) % 360.0
}

pub(super) fn shortest_angle_delta(target: f64, current: f64) -> f64 {
    let delta = normalize_angle(target - current);
    if delta > 180.0 {
        delta - 360.0
    } else {
        delta
    }
}

pub(super) fn relative_delta_ratio(a: f64, b: f64, floor: f64) -> f64 {
    let denom = a.abs().max(b.abs()).max(floor);
    (a - b).abs() / denom
}

/// Normalized bit pattern for signature comparison: collapses -0 into +0 and
/// all NaNs into one canonical NaN so bit equality matches JavaScript string equality.
pub(super) fn sig_bits(x: f64) -> u64 {
    if x == 0.0 {
        0u64
    } else if x.is_nan() {
        f64::NAN.to_bits()
    } else {
        x.to_bits()
    }
}
