use super::*;

/// One loft column interpolated through `points` at the shared `parameters`,
/// BROKEN at the station indices `breaks`: each run of stations between two
/// breaks (and the ends) is fitted on its own — the loft's cubic, on the same
/// global parameters restricted to the run — and the runs are joined C0 by a
/// knot of full multiplicity at the break station, which every run
/// interpolates exactly.
///
/// Why: a path whose curvature JUMPS at a joint (a line into an arc) carries
/// a section whose rate of turn jumps there, and one C² cubic through
/// stations on both sides overshoots by a term FIRST order in the local
/// spacing — the station sampler then has to cluster stations at every such
/// joint, and a chain of enough of them reaches the station cap with the wall
/// still off tolerance. Fitted per run, each run interpolates a smooth
/// function and keeps the cubic's `h⁴` law up to the joint.
///
/// Every column of one loft shares `parameters` and `breaks`, so every column
/// gets the same knot vector — what a skin needs. A run of 2 or 3 stations is
/// fitted at degree 1 or 2 and elevated exactly to 3. `breaks` must be
/// strictly increasing and interior (`0 < b < points.len() − 1`); empty
/// `breaks` is one run. Open columns only: a ring is passed unrolled, its
/// station 0 repeated at the end with the wrap parameter, so its seam is a
/// break like any other.
pub fn interpolate_column_broken(points: &[Vec3], parameters: &[f64], breaks: &[usize]) -> Result<NurbsCurve, String> {
    let count = points.len();
    if count < 2 || parameters.len() != count {
        return Err("interpolate_column_broken: points and parameters must have matching length >= 2".into());
    }
    if breaks.windows(2).any(|pair| pair[1] <= pair[0]) || breaks.iter().any(|&b| b == 0 || b >= count - 1) {
        return Err(format!("interpolate_column_broken: breaks {breaks:?} must be strictly increasing and interior to {count} stations"));
    }
    let mut bounds = Vec::with_capacity(breaks.len() + 2);
    bounds.push(0);
    bounds.extend_from_slice(breaks);
    bounds.push(count - 1);
    let mut knots: Vec<f64> = Vec::new();
    let mut poles: Vec<Vec4> = Vec::new();
    for (index, run) in bounds.windows(2).enumerate() {
        let (first, last) = (run[0], run[1]);
        let piece = crate::interpolate_curve(&points[first..=last], 3, &parameters[first..=last])?;
        let piece = if piece.degree < 3 { crate::coons::compat::elevate_degree(&piece, 3)? } else { piece };
        if piece.degree != 3 {
            return Err(format!("interpolate_column_broken: run {index} came back at degree {}", piece.degree));
        }
        if index == 0 {
            // [a, a, a, a, interior…] — everything but the closing end knots.
            knots.extend_from_slice(&piece.knots[..piece.knots.len() - 4]);
            poles.extend_from_slice(&piece.control_points);
        } else {
            // The break: three copies of its parameter (C0), then this run's
            // interior knots; its first pole IS the previous run's last.
            knots.extend_from_slice(&piece.knots[1..piece.knots.len() - 4]);
            poles.extend_from_slice(&piece.control_points[1..]);
        }
        if index + 2 == bounds.len() {
            knots.extend_from_slice(&piece.knots[piece.knots.len() - 4..]);
        }
    }
    NurbsCurve::new(3, knots, poles)
}
