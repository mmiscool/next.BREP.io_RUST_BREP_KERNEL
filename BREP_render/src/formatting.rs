//! Numeric formatting shared by annotations and app panels.

/// Fixed precision without trailing fractional zeros. Preserves signed zero;
/// callers that use the result as an identifier may choose to normalize it.
pub fn compact_decimal(value: f64, precision: usize) -> String {
    let text = format!("{value:.precision$}");
    if precision == 0 {
        return text;
    }
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

