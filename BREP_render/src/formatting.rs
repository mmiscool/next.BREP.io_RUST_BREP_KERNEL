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

/// The grid every viewport-gizmo drag writes to, per unit shown (mm, degrees).
const DRAG_GRID_PER_UNIT: f64 = 1000.0;

/// Below this a drag has not moved a stored number: it is pose or solver noise.
const DRAG_UNMOVED: f64 = 1e-6;

/// `value` on the 0.001 drag grid. Divides by the exact integer rather than
/// multiplying by `0.001`, which is not representable and would bring back the
/// `12.300000000000001` the grid exists to remove. Never returns `-0`.
pub fn snap_drag_value(value: f64) -> f64 {
    if !value.is_finite() {
        return value;
    }
    (value * DRAG_GRID_PER_UNIT).round() / DRAG_GRID_PER_UNIT + 0.0
}

/// One stored number after a drag: on the grid if the drag moved it, otherwise
/// exactly what it was, so that dragging X leaves a typed Y of `1.23456` alone.
pub fn snap_drag_moved(before: f64, after: f64) -> f64 {
    if (after - before).abs() <= DRAG_UNMOVED {
        before
    } else {
        snap_drag_value(after)
    }
}

