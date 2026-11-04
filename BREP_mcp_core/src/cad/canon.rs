//! RFC 8785 canonical JSON (contract §7.1) and SHA-256.
//!
//! Object keys sort by UTF-16 code units; arrays keep their order; strings
//! keep their code points (no Unicode normalisation) under JSON's minimal
//! escaping; numbers print as ECMAScript's `Number.prototype.toString`; no
//! insignificant whitespace. Integers outside ±2^53 are refused rather than
//! rounded: an identity must never depend on a lossy number.
use serde_json::Value;
use sha2::{Digest, Sha256};

pub fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes).iter().map(|b| format!("{b:02x}")).collect()
}

pub fn canonical(value: &Value) -> Result<String, String> {
    let mut out = String::new();
    write(value, &mut out, 0)?;
    Ok(out)
}

const MAX_DEPTH: usize = 64;

fn write(value: &Value, out: &mut String, depth: usize) -> Result<(), String> {
    if depth > MAX_DEPTH {
        return Err(format!("JSON deeper than {MAX_DEPTH}"));
    }
    match value {
        Value::Null => out.push_str("null"),
        Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        Value::Number(n) => out.push_str(&number(n)?),
        Value::String(s) => string(s, out),
        Value::Array(items) => {
            out.push('[');
            for (i, item) in items.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                write(item, out, depth + 1)?;
            }
            out.push(']');
        }
        Value::Object(map) => {
            let mut keys: Vec<&String> = map.keys().collect();
            keys.sort_by(|a, b| a.encode_utf16().cmp(b.encode_utf16()));
            out.push('{');
            for (i, key) in keys.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                string(key, out);
                out.push(':');
                write(&map[key.as_str()], out, depth + 1)?;
            }
            out.push('}');
        }
    }
    Ok(())
}

fn string(s: &str, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\u{08}' => out.push_str("\\b"),
            '\u{0c}' => out.push_str("\\f"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

const SAFE: u64 = (1 << 53) - 1;

fn number(n: &serde_json::Number) -> Result<String, String> {
    if let Some(i) = n.as_i64() {
        if i.unsigned_abs() > SAFE {
            return Err(format!("integer {i} is outside the exact JavaScript range"));
        }
        return Ok(i.to_string());
    }
    if let Some(u) = n.as_u64() {
        if u > SAFE {
            return Err(format!("integer {u} is outside the exact JavaScript range"));
        }
        return Ok(u.to_string());
    }
    let f = n.as_f64().ok_or("not a finite number")?;
    es_number(f)
}

/// `Number.prototype.toString` (ECMA-262 Number::toString, radix 10).
pub fn es_number(f: f64) -> Result<String, String> {
    if !f.is_finite() {
        return Err("NaN and Infinity are not JSON".into());
    }
    if f == 0.0 {
        return Ok("0".into());
    }
    let sign = if f < 0.0 { "-" } else { "" };
    // Rust's `{:e}` is the shortest round-trip digit string: "d.ddde±x".
    let sci = format!("{:e}", f.abs());
    let (mantissa, exp) = sci.split_once('e').ok_or("unexpected float format")?;
    let digits: String = mantissa.chars().filter(|c| *c != '.').collect();
    let k = digits.len() as i32;
    let n = exp.parse::<i32>().map_err(|e| e.to_string())? + 1;
    let body = if k <= n && n <= 21 {
        format!("{digits}{}", "0".repeat((n - k) as usize))
    } else if 0 < n && n <= 21 {
        format!("{}.{}", &digits[..n as usize], &digits[n as usize..])
    } else if -6 < n && n <= 0 {
        format!("0.{}{digits}", "0".repeat((-n) as usize))
    } else {
        let e = n - 1;
        let e = if e >= 0 { format!("+{e}") } else { e.to_string() };
        if k == 1 {
            format!("{digits}e{e}")
        } else {
            format!("{}.{}e{e}", &digits[..1], &digits[1..])
        }
    };
    Ok(format!("{sign}{body}"))
}

