use crate::{DiagnosticSeverity, KernelDiagnostics, KernelStage, KnotVector, NurbsCurve, Vec3, Vec4};

pub fn solve_dense(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Result<Vec<f64>, String> {
    let count = rhs.len();
    if count == 0 || matrix.len() != count || matrix.iter().any(|row| row.len() != count) {
        return Err("solve_dense: matrix must be square and match RHS".into());
    }
    let scale = matrix
        .iter()
        .flatten()
        .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
    if scale == 0.0 {
        return Err("solve_dense: singular matrix".into());
    }
    for column in 0..count {
        let pivot = (column..count)
            .max_by(|&a, &b| matrix[a][column].abs().total_cmp(&matrix[b][column].abs()))
            .unwrap();
        if matrix[pivot][column].abs() <= 1e-13 * scale {
            return Err("solve_dense: singular matrix".into());
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        let diagonal = matrix[column][column];
        for row in column + 1..count {
            let factor = matrix[row][column] / diagonal;
            matrix[row][column] = 0.0;
            for entry in column + 1..count {
                matrix[row][entry] -= factor * matrix[column][entry];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    let mut result = vec![0.0; count];
    for row in (0..count).rev() {
        let remainder: f64 = (row + 1..count)
            .map(|column| matrix[row][column] * result[column])
            .sum();
        result[row] = (rhs[row] - remainder) / matrix[row][row];
    }
    Ok(result)
}

/// `solve_dense` for fixed-size systems on the stack — identical pivoting and
/// thresholds, no allocation. `count <= N` solves the leading `count`-sized
/// block (for callers that shrink the system by fixing parameters).
pub fn solve_small<const N: usize>(
    mut matrix: [[f64; N]; N],
    mut rhs: [f64; N],
    count: usize,
) -> Result<[f64; N], String> {
    if count == 0 || count > N {
        return Err("solve_small: invalid system size".into());
    }
    let scale = matrix
        .iter()
        .take(count)
        .flat_map(|row| row.iter().take(count))
        .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
    if scale == 0.0 {
        return Err("solve_small: singular matrix".into());
    }
    for column in 0..count {
        let pivot = (column..count)
            .max_by(|&a, &b| matrix[a][column].abs().total_cmp(&matrix[b][column].abs()))
            .unwrap();
        if matrix[pivot][column].abs() <= 1e-13 * scale {
            return Err("solve_small: singular matrix".into());
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        let diagonal = matrix[column][column];
        for row in column + 1..count {
            let factor = matrix[row][column] / diagonal;
            matrix[row][column] = 0.0;
            for entry in column + 1..count {
                matrix[row][entry] -= factor * matrix[column][entry];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    let mut result = [0.0; N];
    for row in (0..count).rev() {
        let remainder: f64 = (row + 1..count)
            .map(|column| matrix[row][column] * result[column])
            .sum();
        result[row] = (rhs[row] - remainder) / matrix[row][row];
    }
    Ok(result)
}

/// Solve a B-spline interpolation (collocation) system. The collocation matrix
/// `N_j(τ_i)` produced by the knot-averaging schemes below is banded with
/// half-bandwidth ≤ `degree` (Schoenberg–Whitney) and totally positive, so a
/// no-pivot BANDED elimination is O(n·degree²) — vs the O(n³) of the general
/// dense solver — and is numerically identical to it on these systems.
///
/// The banded result is accepted only when its in-band residual is tiny;
/// otherwise (a pathological / near-singular system) it falls back to the
/// partial-pivoting dense solver. This keeps large STEP-import pcurve fits
/// (which push `n` toward ~500) from turning each solve into a ~500³ blowup.
pub fn solve_collocation(
    matrix: &[Vec<f64>],
    rhs: &[f64],
    degree: usize,
) -> Result<Vec<f64>, String> {
    let count = rhs.len();
    let bandwidth = degree.max(1).min(count.saturating_sub(1).max(1));
    if let Ok(solution) = solve_banded(matrix, rhs, bandwidth) {
        let scale = rhs
            .iter()
            .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
        let tolerance = 1e-9 * scale.max(1.0);
        let mut residual = 0.0_f64;
        for row in 0..count {
            let lo = row.saturating_sub(bandwidth);
            let hi = (row + bandwidth).min(count - 1);
            let mut accumulated = 0.0;
            for col in lo..=hi {
                accumulated += matrix[row][col] * solution[col];
            }
            residual = residual.max((accumulated - rhs[row]).abs());
        }
        if residual <= tolerance {
            return Ok(solution);
        }
    }
    solve_dense(matrix.to_vec(), rhs.to_vec())
}

/// A collocation matrix held as its band only: row `r` stores columns
/// `r - bandwidth ..= r + bandwidth`, the entries [`solve_banded`]'s no-pivot
/// elimination can read or fill. [`solve_collocation`] on a dense
/// `(n+1)²` matrix allocates and copies O(n²) per axis for an O(n·degree²)
/// solve; at the 2000-station ceiling of the pcurve fitters that dominated
/// the solve (28 s of an 81 s `abc_00000011` import, 4051 builds, measured
/// 2026-09-26). [`solve_collocation_banded`] performs the SAME operations in
/// the same order on the same entries, so its result is bit-identical.
pub(crate) struct CollocationBand {
    count: usize,
    bandwidth: usize,
    data: Vec<f64>,
}

impl CollocationBand {
    /// Zeroed, for `count` rows at [`solve_collocation`]'s bandwidth for
    /// `degree`.
    pub(crate) fn new(count: usize, degree: usize) -> Self {
        let bandwidth = degree.max(1).min(count.saturating_sub(1).max(1));
        CollocationBand { count, bandwidth, data: vec![0.0; count * (2 * bandwidth + 1)] }
    }

    fn slot(&self, row: usize, column: usize) -> Option<usize> {
        let offset = column as isize - row as isize + self.bandwidth as isize;
        (offset >= 0 && offset <= 2 * self.bandwidth as isize)
            .then(|| row * (2 * self.bandwidth + 1) + offset as usize)
    }

    /// Store an entry; `false` when it lies outside the band (the caller then
    /// solves densely, as [`solve_collocation`] would after its banded check).
    pub(crate) fn set(&mut self, row: usize, column: usize, value: f64) -> bool {
        match self.slot(row, column) {
            Some(index) => {
                self.data[index] = value;
                true
            }
            None => value == 0.0,
        }
    }

    fn get(&self, row: usize, column: usize) -> f64 {
        self.slot(row, column).map_or(0.0, |index| self.data[index])
    }

    fn to_dense(&self) -> Vec<Vec<f64>> {
        (0..self.count)
            .map(|row| (0..self.count).map(|column| self.get(row, column)).collect())
            .collect()
    }
}

/// [`solve_collocation`] on a [`CollocationBand`]: the banded elimination with
/// its in-band residual check, and the dense partial-pivoting fallback
/// (materialised only then). Bit-identical to [`solve_collocation`] on the
/// dense form of the same matrix.
pub(crate) fn solve_collocation_banded(band: &CollocationBand, rhs: &[f64]) -> Result<Vec<f64>, String> {
    let count = rhs.len();
    if count == 0 || band.count != count {
        return Err("solve_banded: matrix must be square and match RHS".into());
    }
    let bandwidth = band.bandwidth;
    if let Ok(solution) = solve_band_storage(band, rhs) {
        let scale = rhs
            .iter()
            .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
        let tolerance = 1e-9 * scale.max(1.0);
        let mut residual = 0.0_f64;
        for row in 0..count {
            let lo = row.saturating_sub(bandwidth);
            let hi = (row + bandwidth).min(count - 1);
            let mut accumulated = 0.0;
            for col in lo..=hi {
                accumulated += band.get(row, col) * solution[col];
            }
            residual = residual.max((accumulated - rhs[row]).abs());
        }
        if residual <= tolerance {
            return Ok(solution);
        }
    }
    solve_dense(band.to_dense(), rhs.to_vec())
}

fn solve_band_storage(band: &CollocationBand, rhs: &[f64]) -> Result<Vec<f64>, String> {
    let count = rhs.len();
    let bandwidth = band.bandwidth;
    let width = 2 * bandwidth + 1;
    let mut matrix = band.data.clone();
    let mut rhs = rhs.to_vec();
    // Row `r`, column `c` lives at `r * width + (c + bandwidth - r)`.
    let at = |row: usize, column: usize| row * width + column + bandwidth - row;
    for column in 0..count {
        let pivot = matrix[at(column, column)];
        if pivot.abs() <= 1e-300 {
            return Err("solve_banded: singular matrix".into());
        }
        let maximum_row = (column + bandwidth).min(count - 1);
        for row in column + 1..=maximum_row {
            let factor = matrix[at(row, column)] / pivot;
            if factor == 0.0 {
                continue;
            }
            let maximum_column = (column + bandwidth).min(count - 1);
            for entry in column..=maximum_column {
                matrix[at(row, entry)] -= factor * matrix[at(column, entry)];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    let mut result = vec![0.0; count];
    for row in (0..count).rev() {
        let mut value = rhs[row];
        let maximum_column = (row + bandwidth).min(count - 1);
        for column in row + 1..=maximum_column {
            value -= matrix[at(row, column)] * result[column];
        }
        if matrix[at(row, row)].abs() <= 1e-300 {
            return Err("solve_banded: singular matrix".into());
        }
        result[row] = value / matrix[at(row, row)];
    }
    Ok(result)
}

pub fn solve_banded(
    matrix: &[Vec<f64>],
    rhs: &[f64],
    bandwidth: usize,
) -> Result<Vec<f64>, String> {
    let count = rhs.len();
    if count == 0 || matrix.len() != count || matrix.iter().any(|row| row.len() != count) {
        return Err("solve_banded: matrix must be square and match RHS".into());
    }
    let mut matrix = matrix.to_vec();
    let mut rhs = rhs.to_vec();
    for column in 0..count {
        let pivot = matrix[column][column];
        if pivot.abs() <= 1e-300 {
            return Err("solve_banded: singular matrix".into());
        }
        let maximum_row = (column + bandwidth).min(count - 1);
        for row in column + 1..=maximum_row {
            let factor = matrix[row][column] / pivot;
            if factor == 0.0 {
                continue;
            }
            let maximum_column = (column + bandwidth).min(count - 1);
            for entry in column..=maximum_column {
                matrix[row][entry] -= factor * matrix[column][entry];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    let mut result = vec![0.0; count];
    for row in (0..count).rev() {
        let mut value = rhs[row];
        let maximum_column = (row + bandwidth).min(count - 1);
        for column in row + 1..=maximum_column {
            value -= matrix[row][column] * result[column];
        }
        if matrix[row][row].abs() <= 1e-300 {
            return Err("solve_banded: singular matrix".into());
        }
        result[row] = value / matrix[row][row];
    }
    Ok(result)
}

/// Global B-spline interpolation (The NURBS Book A9.1), matching the
/// reference implementation's supplied-parameter path.
/// Interpolate HOMOGENEOUS points (Vec4, rational data) at the given
/// parameters with the same averaged-knot scheme as `interpolate_curve`.
/// Parameters must start at 0 and end at 1. Rows fitted with identical
/// parameters share knots — the contract the blend-surface rows rely on.
pub fn interpolate_homogeneous(
    points: &[Vec4],
    degree: usize,
    parameters: &[f64],
) -> Result<NurbsCurve, String> {
    if points.len() < 2 || parameters.len() != points.len() {
        return Err(
            "interpolate_homogeneous: points and parameters must have matching length >= 2".into(),
        );
    }
    if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err("interpolate_homogeneous: parameters must be strictly increasing".into());
    }
    let n = points.len() - 1;
    let degree = degree.min(n);
    let m = n + degree + 1;
    let mut knots = vec![0.0; m + 1];
    for knot in &mut knots[m - degree..=m] {
        *knot = 1.0;
    }
    for j in 1..=n.saturating_sub(degree) {
        knots[j + degree] = parameters[j..j + degree].iter().sum::<f64>() / degree as f64;
    }
    let knot_vector = KnotVector::new(knots.clone(), degree)?;
    let mut matrix = vec![vec![0.0; n + 1]; n + 1];
    for (row, &parameter) in parameters.iter().enumerate() {
        let span = knot_vector.find_span(parameter);
        let basis = knot_vector.basis_functions(span, parameter);
        for (offset, value) in basis.into_iter().enumerate() {
            matrix[row][span - degree + offset] = value;
        }
    }
    let solve_axis = |axis: fn(&Vec4) -> f64| {
        solve_collocation(
            &matrix,
            &points.iter().map(axis).collect::<Vec<_>>(),
            degree,
        )
    };
    let xs = solve_axis(|point| point.x)?;
    let ys = solve_axis(|point| point.y)?;
    let zs = solve_axis(|point| point.z)?;
    let ws = solve_axis(|point| point.w)?;
    NurbsCurve::new(
        degree,
        knots,
        (0..=n)
            .map(|index| Vec4 {
                x: xs[index],
                y: ys[index],
                z: zs[index],
                w: ws[index],
            })
            .collect(),
    )
}

/// Global B-spline interpolation (The NURBS Book A9.1) through `points` at the
/// supplied strictly increasing `parameters`.
///
/// The result is CLAMPED over exactly `[parameters[0], parameters[n]]` — the
/// caller's own parameter interval, not a normalized `[0, 1]`. A9.1's end knots
/// are `ū_0` and `ū_n`; hardcoding `0.0`/`1.0` there is only correct when the
/// caller already normalized, and silently emits a knot vector that DESCENDS at
/// the clamp when it did not (STEP import's `reconcile_edges_onto_surfaces`
/// re-fits an edge over its own domain — on ABC 00000290 that is `[1.0,
/// 1.2499]`, producing interior knots up to 1.234 followed by the four 1.0 end
/// knots, i.e. a 0.234 descent that `validate_knots` rightly rejected). Since
/// the averaged interior knots always lie strictly between `parameters[0]` and
/// `parameters[n]`, taking the ends from the parameters makes the vector valid
/// for ANY strictly increasing input, and is bit-identical for the already-
/// normalized callers.
pub fn interpolate_curve(
    points: &[Vec3],
    degree: usize,
    parameters: &[f64],
) -> Result<NurbsCurve, String> {
    if points.len() < 2 || parameters.len() != points.len() {
        return Err(
            "interpolate_curve: points and parameters must have matching length >= 2".into(),
        );
    }
    if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err("interpolate_curve: parameters must be strictly increasing".into());
    }
    let n = points.len() - 1;
    let degree = degree.min(n);
    let m = n + degree + 1;
    let mut knots = vec![parameters[0]; m + 1];
    for knot in &mut knots[m - degree..=m] {
        *knot = parameters[n];
    }
    for j in 1..=n.saturating_sub(degree) {
        knots[j + degree] = parameters[j..j + degree].iter().sum::<f64>() / degree as f64;
    }
    let knot_vector = KnotVector::new(knots.clone(), degree)?;
    // The band alone ([`CollocationBand`]); the dense matrix only when a basis
    // value falls outside it, which the dense banded solve could not use either.
    let mut band = CollocationBand::new(n + 1, degree);
    let mut dense: Option<Vec<Vec<f64>>> = None;
    for (row, &parameter) in parameters.iter().enumerate() {
        let span = knot_vector.find_span(parameter);
        let basis = knot_vector.basis_functions(span, parameter);
        for (offset, value) in basis.into_iter().enumerate() {
            let column = span - degree + offset;
            if !band.set(row, column, value) {
                dense.get_or_insert_with(|| band.to_dense())[row][column] = value;
            } else if let Some(dense) = dense.as_mut() {
                dense[row][column] = value;
            }
        }
    }
    let solve_axis = |axis: fn(Vec3) -> f64| {
        let rhs = points.iter().copied().map(axis).collect::<Vec<_>>();
        match &dense {
            Some(matrix) => solve_collocation(matrix, &rhs, degree),
            None => solve_collocation_banded(&band, &rhs),
        }
    };
    let xs = solve_axis(|point| point.x)?;
    let ys = solve_axis(|point| point.y)?;
    let zs = solve_axis(|point| point.z)?;
    NurbsCurve::new(
        degree,
        knots,
        (0..=n)
            .map(|index| Vec4::from_point(Vec3::new(xs[index], ys[index], zs[index]), 1.0))
            .collect(),
    )
}

/// Cubic interpolation with PRESCRIBED end derivatives (Piegl–Tiller §9.2.2):
/// n+1 points plus two tangent rows give n+3 clamped control points. The
/// derivative conditions use the exact clamped end forms
/// C'(t0) = p/(u_{p+1}−t0)·(Q1−Q0) and C'(t1) = p/(t1−u_{m−p−1})·(Qn−Qn−1),
/// so the requested tangents are reproduced exactly — the §5.8 loft tangency
/// building block.
pub fn interpolate_curve_with_end_tangents(
    points: &[Vec3],
    parameters: &[f64],
    start_tangent: Vec3,
    end_tangent: Vec3,
) -> Result<NurbsCurve, String> {
    if points.len() < 2 || parameters.len() != points.len() {
        return Err(
            "interpolate_curve_with_end_tangents: points and parameters must match, >= 2".into(),
        );
    }
    if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err("interpolate_curve_with_end_tangents: parameters must increase".into());
    }
    let degree = 3usize;
    let n = points.len() - 1;
    let control_count = n + 3;
    let t0 = parameters[0];
    let t1 = parameters[n];
    // Clamped knots with n−1 interior values averaged over parameter runs
    // (the tangent rows consume the two extra controls).
    let mut knots = vec![t0; degree + 1];
    for j in 0..n.saturating_sub(1) {
        let window = &parameters[j + 1..(j + degree).min(n) + 1];
        knots.push(window.iter().sum::<f64>() / window.len() as f64);
    }
    knots.extend(std::iter::repeat(t1).take(degree + 1));
    if knots.len() != control_count + degree + 1 {
        return Err(format!(
            "interpolate_curve_with_end_tangents: internal knot count {} for {} controls",
            knots.len(),
            control_count
        ));
    }
    let knot_vector = KnotVector::new(knots.clone(), degree)?;
    let mut matrix = vec![vec![0.0; control_count]; control_count];
    let mut rhs_points = vec![Vec3::default(); control_count];
    // Row 0: C(t0) = P0; row 1: start tangent; rows 2..=n: interior + end
    // interpolation; row n+1... reorganized: standard layout is
    // [P0, T0, P1..Pn-1, T1, Pn].
    matrix[0][0] = 1.0;
    rhs_points[0] = points[0];
    let start_span = knots[degree + 1] - t0;
    matrix[1][0] = -(degree as f64) / start_span;
    matrix[1][1] = (degree as f64) / start_span;
    rhs_points[1] = start_tangent;
    for (index, &parameter) in parameters.iter().enumerate().take(n).skip(1) {
        let row = index + 1;
        let span = knot_vector.find_span(parameter);
        let basis = knot_vector.basis_functions(span, parameter);
        for (offset, value) in basis.into_iter().enumerate() {
            matrix[row][span - degree + offset] = value;
        }
        rhs_points[row] = points[index];
    }
    let end_span = t1 - knots[control_count - 1];
    matrix[control_count - 2][control_count - 2] = -(degree as f64) / end_span;
    matrix[control_count - 2][control_count - 1] = (degree as f64) / end_span;
    rhs_points[control_count - 2] = end_tangent;
    matrix[control_count - 1][control_count - 1] = 1.0;
    rhs_points[control_count - 1] = points[n];
    let solve_axis = |axis: fn(Vec3) -> f64| {
        solve_dense(
            matrix.clone(),
            rhs_points.iter().copied().map(axis).collect::<Vec<_>>(),
        )
    };
    let xs = solve_axis(|point| point.x)?;
    let ys = solve_axis(|point| point.y)?;
    let zs = solve_axis(|point| point.z)?;
    NurbsCurve::new(
        degree,
        knots,
        (0..control_count)
            .map(|index| Vec4::from_point(Vec3::new(xs[index], ys[index], zs[index]), 1.0))
            .collect(),
    )
}

pub fn interpolate_curve_thinned(
    points: &[Vec3],
    degree: usize,
    maximum_points: usize,
) -> Result<NurbsCurve, String> {
    if maximum_points < 2 {
        return Err("interpolate_curve_thinned: maximum point count must be at least 2".into());
    }
    if points.len() <= maximum_points {
        let parameters = chord_parameters(points);
        return interpolate_curve(points, degree, &parameters);
    }
    let step = (points.len() - 1) as f64 / (maximum_points - 1) as f64;
    let thinned = (0..maximum_points)
        .map(|index| points[(index as f64 * step).round() as usize])
        .collect::<Vec<_>>();
    let parameters = chord_parameters(&thinned);
    interpolate_curve(&thinned, degree, &parameters)
}

fn chord_parameters(points: &[Vec3]) -> Vec<f64> {
    if points.len() < 2 {
        return vec![0.0; points.len()];
    }
    let mut parameters = vec![0.0; points.len()];
    for index in 1..points.len() {
        parameters[index] = parameters[index - 1] + points[index].sub(points[index - 1]).length();
    }
    let length = parameters[points.len() - 1];
    if length <= 1e-15 {
        for (index, parameter) in parameters.iter_mut().enumerate() {
            *parameter = index as f64 / (points.len() - 1) as f64;
        }
    } else {
        for parameter in &mut parameters {
            *parameter /= length;
        }
    }
    parameters
}

/// Douglas–Peucker against `tolerance`, as INDICES into `points`.
///
/// The indices are what a FIT needs: they carry the correspondence between an
/// interpolation station and the input station it came from, and that
/// correspondence is what lets [`fit_polyline`] measure its own result against
/// the input points BETWEEN its stations without a global projection per point.
/// [`simplify_polyline`] is a thin wrapper over this; the arithmetic is
/// unchanged.
pub fn simplify_polyline_indices(points: &[Vec3], tolerance: f64) -> Vec<usize> {
    if points.len() <= 2 {
        return (0..points.len()).collect();
    }
    let mut keep = vec![false; points.len()];
    keep[0] = true;
    keep[points.len() - 1] = true;
    let mut stack = vec![(0usize, points.len() - 1)];
    while let Some((start, end)) = stack.pop() {
        if end - start < 2 {
            continue;
        }
        let a = points[start];
        let direction = points[end].sub(a);
        let length_squared = direction.length_squared().max(1e-300);
        let mut worst = None;
        let mut worst_distance = tolerance;
        for (index, point) in points.iter().enumerate().take(end).skip(start + 1) {
            let fraction = point.sub(a).dot(direction) / length_squared;
            let fraction = fraction.clamp(0.0, 1.0);
            let distance = point.sub(a.add(direction.scale(fraction))).length();
            if distance > worst_distance {
                worst_distance = distance;
                worst = Some(index);
            }
        }
        if let Some(index) = worst {
            keep[index] = true;
            stack.push((start, index));
            stack.push((index, end));
        }
    }
    keep.into_iter()
        .enumerate()
        .filter_map(|(index, keep)| keep.then_some(index))
        .collect()
}

pub fn simplify_polyline(points: &[Vec3], tolerance: f64) -> Vec<Vec3> {
    simplify_polyline_indices(points, tolerance)
        .into_iter()
        .map(|index| points[index])
        .collect()
}

pub fn interpolate_curve_local(
    points: &[Vec3],
    parameters: &[f64],
    tension: f64,
) -> Result<NurbsCurve, String> {
    if points.len() != parameters.len() || points.len() < 2 {
        return Err(
            "interpolate_curve_local: points and parameters must have matching length >= 2".into(),
        );
    }
    if points.len() == 2 {
        return interpolate_curve(points, 1, parameters);
    }
    let count = points.len();
    let mut tangents = vec![Vec3::default(); count];
    tangents[0] = points[1]
        .sub(points[0])
        .scale(1.0 / (parameters[1] - parameters[0]));
    tangents[count - 1] = points[count - 1]
        .sub(points[count - 2])
        .scale(1.0 / (parameters[count - 1] - parameters[count - 2]));
    for index in 1..count - 1 {
        let previous_interval = parameters[index] - parameters[index - 1];
        let next_interval = parameters[index + 1] - parameters[index];
        let total = previous_interval + next_interval;
        let previous_secant = points[index]
            .sub(points[index - 1])
            .scale(1.0 / previous_interval);
        let next_secant = points[index + 1]
            .sub(points[index])
            .scale(1.0 / next_interval);
        let mut tangent =
            points[index - 1]
                .scale(-next_interval / (previous_interval * total))
                .add(points[index].scale(
                    (next_interval - previous_interval) / (previous_interval * next_interval),
                ))
                .add(points[index + 1].scale(previous_interval / (next_interval * total)));
        if previous_secant.dot(next_secant) <= 0.0
            || tangent.dot(previous_secant) <= 0.0
            || tangent.dot(next_secant) <= 0.0
        {
            tangent = Vec3::default();
        } else {
            let maximum = 3.0 * previous_secant.length().min(next_secant.length());
            if tangent.length() > maximum {
                tangent = tangent.normalized()?.scale(maximum);
            }
        }
        tangents[index] = tangent;
    }
    if tension != 1.0 {
        for tangent in &mut tangents {
            *tangent = tangent.scale(tension);
        }
    }
    let mut control_points = vec![Vec4::from_point(points[0], 1.0)];
    for index in 0..count - 1 {
        let interval = parameters[index + 1] - parameters[index];
        control_points.extend([
            Vec4::from_point(
                points[index].add(tangents[index].scale(interval / 3.0)),
                1.0,
            ),
            Vec4::from_point(
                points[index + 1].sub(tangents[index + 1].scale(interval / 3.0)),
                1.0,
            ),
            Vec4::from_point(points[index + 1], 1.0),
        ]);
    }
    let mut knots = vec![parameters[0]; 4];
    for parameter in &parameters[1..count - 1] {
        knots.extend([*parameter; 3]);
    }
    knots.extend([parameters[count - 1]; 4]);
    NurbsCurve::new(3, knots, control_points)
}

/// How far past the caller's own station budget the ladder may climb.
///
/// The budget a caller passes is its declaration of what it can afford
/// DOWNSTREAM, and downstream cost is not linear in a section curve's span
/// count: measured on the torus×torus singular pair, 4x the stations (80 → 320)
/// took one imprint from 1.4 s to over 200 s, because every pcurve fit on the
/// section then needs more samples than `MAX_PCURVE_SAMPLES` allows and takes
/// the raised pass, 400 times over. So the ladder may DOUBLE the caller's
/// budget and no more. With the input's own band as the floor
/// ([`PolylineFitReport::floor`]) that meets 275 of the 282 fits in the case
/// population; the other 7 are all `offset_shell` sections that stop at 192
/// stations holding 2.33e-7 to 3.42e-6, and they REPORT it
/// ([`PolylineFitExit::StationCeiling`]) instead of widening to it. Meeting every
/// bar is not this bound's job — capping the cost of the ones it cannot at 2x
/// what the caller chose, rather than at an absolute this module invented, is.
const LADDER_BUDGET_FACTOR: usize = 2;

/// Absolute station ceiling, whatever the caller's budget.
///
/// The ladder doubles the interpolation budget from what the caller asked for
/// while the MEASUREMENT says the curve misses its tolerance, and stops here —
/// or at the input's own station count, whichever comes first.
///
/// 1024 is a COST cap, not a geometric one. As SHIPPED it is a BACKSTOP that
/// nothing in the corpus reaches, because [`LADDER_BUDGET_FACTOR`] binds first
/// for every caller here (80 → 160 for the imprint driver, 96 → 192 for
/// `offset_shell`). The reading below is the ladder let OFF that bound, kept
/// because it is what the ceiling exists for. Measured over the 55-row case
/// population: 282 fits, 21 of which miss at the caller's own budget; 16 of
/// those reach their tolerance inside the ladder, and 5 do not — three at this
/// ceiling with 3,070 control points and two stalled at 2,302 — every one of
/// them an `offset_shell` section on the LOCAL interpolation lane, which
/// converges at h² and cannot reach 1e-7 on a curved section at any station
/// count this input offers. The lane is the fix there, not the ceiling; raising
/// the ceiling buys accuracy in the fourth significant figure and pays for it
/// in control points on every consumer downstream. A fit that stops here
/// reports [`PolylineFitExit::StationCeiling`] with the deviation it actually
/// holds, the way a pcurve fit reports its sample ceiling.
pub const MAX_FIT_STATIONS: usize = 1024;

/// Newton steps spent per point when the measurement locates the curve's
/// nearest point to an input station. Three is enough on a seed that is already
/// inside the right span (the seed comes from the station correspondence, not
/// from a search), and the best iterate is kept, so a non-convergent step can
/// only cost time.
const MEASURE_NEWTON_STEPS: usize = 3;

/// Fractions of every station span the measurement probes BETWEEN the input's
/// own stations — the same three the pcurve refinement probes (`probe_residual`
/// in `geometry/pcurve.rs`), and for the same reason: an interpolant that
/// passes exactly through every station it was built from can still bulge in
/// between, and that bulge is what becomes topology.
const MEASURE_SPAN_FRACTIONS: [f64; 3] = [0.25, 0.5, 0.75];

/// A rung must beat the one below it by this factor to be worth the next
/// doubling. Global cubic interpolation improves 16x per doubling and the local
/// lane 4x, so anything above 0.9 says the input — not the station count — is
/// what binds, and the ladder stops with [`PolylineFitExit::Stalled`].
const STALL_FACTOR: f64 = 0.9;

/// What a polyline fit ACHIEVED, returned beside the curve so a budget exit can
/// never be mistaken for a met tolerance.
///
/// [`fit_polyline`] is handed a tolerance by every caller that fits a marched
/// section. Until 2026-09-13 that tolerance only ever reached
/// [`simplify_polyline`] and the conditioning floor — the INPUT side — and the
/// returned curve was never compared against it, while `maximum_points`
/// DECIMATED the input before interpolation. A cap on the number of
/// interpolation points is not a tolerance: on the 5,925-point marched base
/// circle of an offset apex cone the delivered rim sagged 2.179e-3 against a
/// requested 1e-7, four orders, silently, and the shell's volume read 4.23e-7
/// relative where its analytically recognized sibling read 2.40e-11.
///
/// This report is a MEASUREMENT taken at a construction, not a band. Nothing
/// may read `deviation` as the acceptance tolerance of the curve it describes;
/// its consumers are the [`PolylineFitLedger`] (operation diagnostics) and the
/// ladder's own stopping decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PolylineFitReport {
    /// Largest distance from an input point to the returned curve, over every
    /// point of the input run. THE acceptance measurement: the points the
    /// decimation dropped are the ones that read it.
    pub deviation: f64,
    /// Root mean square of the same sample set.
    pub rms: f64,
    /// Largest distance from the curve to the input POLYLINE, sampled between
    /// the fit's own stations. Reported, never compared against the tolerance:
    /// see [`measure_polyline_fit`] — its floor is the input's chord sag.
    pub between_stations: f64,
    /// The deviation the caller asked for.
    pub tolerance: f64,
    /// The deviation the ladder actually chased: `tolerance.max(dropped_band)`.
    ///
    /// A fit cannot demonstrate fidelity finer than its own INPUT's band. The
    /// conditioning that produced the station pool drops points up to
    /// `dropped_band` from the polyline through the survivors, so a curve that
    /// reproduces the pool to 1e-8 while the pool stands 3.75e-7 from the input
    /// is not a 1e-8 curve — it is a 3.75e-7 curve with an expensive interior.
    /// Refining past the band buys control points and nothing else, and control
    /// points are not free: the imprint's cost in a section curve's span count
    /// is superlinear (measured on the torus×torus singular pair, where 4x the
    /// stations cost two orders of magnitude of `process_curve` time).
    pub floor: f64,
    /// Interpolation stations in the returned curve.
    pub stations: usize,
    /// Control points in the returned curve — what the fit costs downstream.
    pub controls: usize,
    /// Points in the input run.
    pub input_points: usize,
    /// Stations the input offers after [`simplify_polyline`] and conditioning:
    /// the ceiling the ladder can climb to on this input.
    pub pool_points: usize,
    /// Largest distance from an input point DROPPED by the simplification to
    /// the polyline through the stations that survived it — the part of
    /// `deviation` no interpolation can remove, since those points are not
    /// stations any more. The analogue of [`crate::PcurveFitReport::off_surface`]:
    /// a value near `tolerance` says the input's own band explains the
    /// residual. Zero when the simplification dropped nothing.
    pub dropped_band: f64,
    /// Rungs the ladder spent. One means rung zero met the tolerance, or
    /// nothing was refined.
    pub rungs: usize,
    /// Whether the returned curve came from the LOCAL interpolation lane. Not
    /// always the lane the caller asked for: a ladder that ends unmet is
    /// re-run in the other lane and the better MEASURED result is returned
    /// (see [`fit_polyline`]).
    pub local_lane: bool,
    /// Whether that is the other lane — the caller's own ladder could not meet
    /// the tolerance and this one measured better.
    pub lane_switched: bool,
    /// Why the ladder stopped.
    pub exit: PolylineFitExit,
}

impl PolylineFitReport {
    /// Whether the returned curve reaches the floor the fit chased — the
    /// caller's tolerance, or the input's own band where that is coarser (see
    /// [`PolylineFitReport::floor`]).
    pub fn met_tolerance(&self) -> bool {
        self.deviation <= self.floor
    }

    /// Whether the caller's own tolerance was reached, band or no band.
    pub fn met_requested_tolerance(&self) -> bool {
        self.deviation <= self.tolerance
    }

    /// Whether every station the input offers is an interpolation station — in
    /// which case `deviation` is the interpolation SOLVE's residual and not a
    /// statement about the curve between the input's samples.
    ///
    /// This matters for reading a met tolerance honestly. A fit has no oracle
    /// but its input: on the exact-circle bench the pool-exhausted rung reads
    /// 8.9e-16 in both lanes while the curves are 5.61e-7 (local) and 2.36e-13
    /// (global) from the true circle. So a saturated `Converged` says "this
    /// curve reproduces every point the marcher produced", which is the most a
    /// fit can claim — not "this curve is within the tolerance of the section".
    pub fn saturated(&self) -> bool {
        self.stations >= self.pool_points
    }
}

/// Why a polyline fit's refinement stopped. Only
/// [`PolylineFitExit::Converged`] means the tolerance was met; every other
/// variant is a BUDGET exit whose deviation was measured on the curve actually
/// returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PolylineFitExit {
    /// A measured rung came in at or under the tolerance.
    Converged,
    /// [`MAX_FIT_STATIONS`] bound with the tolerance unmet and the input still
    /// holding stations the ladder never used.
    StationCeiling,
    /// Every station the input offers is an interpolation station and the
    /// tolerance is still unmet. More points cannot help; a different lane, a
    /// finer march or a looser bar can.
    InputExhausted,
    /// A doubling failed to improve the deviation by [`STALL_FACTOR`], so the
    /// input rather than the station count is what binds. The better of the two
    /// curves is returned.
    Stalled,
    /// No refinement was attempted: [`fit_polyline_at`] fits exactly one rung,
    /// and `BREP_POLYLINE_FIT_REFINE=0` makes [`fit_polyline`] do the same.
    Unrefined,
}

#[derive(Clone, Debug)]
pub struct PolylineFit {
    pub curve: NurbsCurve,
    pub parameters: Vec<f64>,
    pub kept: Vec<Vec3>,
    /// What this fit achieved against the tolerance it was given.
    pub report: PolylineFitReport,
}

/// The interpolation stations a fit is ENTITLED to on this input: the points
/// [`simplify_polyline`] keeps at `tolerance`, conditioned against
/// near-duplicates (a station a fraction of a step from its neighbour, or from
/// either end, rings the interpolant). Indices into `points`, ascending.
///
/// The decimation to a point BUDGET is deliberately not here
/// ([`decimate_stations`]): the ladder lifts the budget and never the pool, so
/// this runs once per fit however many rungs it takes.
/// The station pool a full refit of `points` would interpolate from: the
/// existing [`station_pool`], exposed unchanged for the section refiner's
/// debug trace, which compares it against the seeded refit's forced seeds.
pub(crate) fn polyline_station_pool(points: &[Vec3], tolerance: f64) -> Vec<usize> {
    station_pool(points, tolerance)
}

fn station_pool(points: &[Vec3], tolerance: f64) -> Vec<usize> {
    let simplified = simplify_polyline_indices(points, tolerance);
    if simplified.len() > 2 {
        let total: f64 = simplified
            .windows(2)
            .map(|pair| points[pair[1]].sub(points[pair[0]]).length())
            .sum();
        let floor = (tolerance * 0.01).max(total * 1e-4);
        let first = points[simplified[0]];
        let last = points[simplified[simplified.len() - 1]];
        let mut conditioned = vec![simplified[0]];
        for index in &simplified[1..simplified.len() - 1] {
            let point = points[*index];
            if point.sub(first).length() > floor
                && point.sub(last).length() > floor
                && point.sub(points[*conditioned.last().unwrap()]).length() > floor
            {
                conditioned.push(*index);
            }
        }
        conditioned.push(simplified[simplified.len() - 1]);
        conditioned
    } else {
        let mut distinct: Vec<usize> = Vec::new();
        for index in simplified {
            if distinct.last().is_none_or(|previous: &usize| {
                points[index].sub(points[*previous]).length() > tolerance * 0.01
            }) {
                distinct.push(index);
            }
        }
        distinct
    }
}

/// `maximum_points` stations spread evenly over the pool, both ends kept.
fn decimate_stations(pool: &[usize], maximum_points: usize) -> Vec<usize> {
    if pool.len() <= maximum_points {
        return pool.to_vec();
    }
    let step = (pool.len() - 1) as f64 / (maximum_points - 1) as f64;
    (0..maximum_points)
        .map(|index| pool[(index as f64 * step).round() as usize])
        .collect()
}

/// Normalized chord-length parameters for `stations`, or `None` when they span
/// no length at all.
fn station_parameters(points: &[Vec3], stations: &[usize]) -> Option<Vec<f64>> {
    let total: f64 = stations
        .windows(2)
        .map(|pair| points[pair[1]].sub(points[pair[0]]).length())
        .sum();
    if total <= 0.0 {
        return None;
    }
    let mut parameters = vec![0.0; stations.len()];
    let mut accumulated = 0.0;
    for index in 1..stations.len() {
        accumulated += points[stations[index]]
            .sub(points[stations[index - 1]])
            .length();
        parameters[index] = accumulated / total;
    }
    *parameters.last_mut().unwrap() = 1.0;
    Some(parameters)
}

fn distance_to_segment(point: Vec3, from: Vec3, to: Vec3) -> f64 {
    let direction = to.sub(from);
    let length_squared = direction.length_squared();
    if length_squared <= 0.0 {
        return point.sub(from).length();
    }
    let fraction = (point.sub(from).dot(direction) / length_squared).clamp(0.0, 1.0);
    point.sub(from.add(direction.scale(fraction))).length()
}

/// Distance from `point` to `curve`, minimized inside `[lower, upper]` from a
/// seed that is already in the right span.
///
/// NOT [`crate::project_point_to_curve`]: that one sweeps `4.max(degree + 2)`
/// samples of EVERY span before its Newton — 1,700 evaluations on a
/// 286-control fit, per query, where this is a fixed four — and it stops
/// refining at a 1e-7 residual, which is exactly the bar this measurement has
/// to resolve.
fn distance_to_curve_near(
    curve: &NurbsCurve,
    point: Vec3,
    seed: f64,
    lower: f64,
    upper: f64,
) -> Result<f64, String> {
    let mut parameter = seed.clamp(lower, upper);
    let mut best = f64::INFINITY;
    for _ in 0..MEASURE_NEWTON_STEPS {
        let derivatives = curve.derivatives_small(parameter, 2)?;
        let residual = derivatives[0].sub(point);
        best = best.min(residual.length_squared());
        let slope = derivatives[1].dot(residual);
        let curvature = derivatives[2].dot(residual) + derivatives[1].length_squared();
        if curvature.abs() <= f64::MIN_POSITIVE {
            break;
        }
        let next = (parameter - slope / curvature).clamp(lower, upper);
        if next == parameter {
            break;
        }
        parameter = next;
    }
    let residual = curve.evaluate(parameter)?.sub(point).length_squared();
    Ok(best.min(residual).sqrt())
}

/// Deviation of `curve` from the input run: `(max, rms, between_stations)`.
///
/// `max` and `rms` are taken AT the input's own stations — the distance from
/// every input point to the curve. That is the acceptance measurement, and it
/// is the one the decimation defeats: a curve interpolating 96 of 5,925 marched
/// points reproduces those 96 exactly and misses the other 5,829 by however far
/// the interpolant sags, 2.179e-3 on the apex cone's base circle.
///
/// `between_stations` is the same curve sampled at [`MEASURE_SPAN_FRACTIONS`] of
/// every station span and measured against the input POLYLINE. It is reported
/// and deliberately NOT compared against the tolerance, because its floor is
/// the input's own sampling: a polyline through exact samples of a circle of
/// radius R taken every h cuts the corner by h²/(8R), so the perfect curve —
/// the circle itself — reads 9.6e-7 against a 7.2e-3-step march of a
/// radius-6.798 section. Measuring acceptance against chords would make the
/// bar unreachable for being right. What it does catch is a curve that wanders
/// between the points it was given by much MORE than that sag.
///
/// `stations` are indices into `points` and `parameters` their curve
/// parameters, so every query starts from a seed inside the right span and
/// nothing here searches the whole curve.
fn measure_polyline_fit(
    curve: &NurbsCurve,
    points: &[Vec3],
    stations: &[usize],
    parameters: &[f64],
    arc: &[f64],
) -> Result<(f64, f64, f64), String> {
    let mut worst = 0.0_f64;
    let mut sum_squares = 0.0_f64;
    let mut samples = 0usize;
    let mut between = 0.0_f64;
    let last = stations.len() - 1;
    for span in 0..last {
        let (from, to) = (stations[span], stations[span + 1]);
        let (t0, t1) = (parameters[span], parameters[span + 1]);
        // One span each side, so a station's nearest point may sit past its own
        // span's end without the bracket cutting the minimization short.
        let lower = parameters[span.saturating_sub(1)];
        let upper = parameters[(span + 2).min(last)];
        // AT the input's stations: every input point of this span, the left end
        // included (the right end belongs to the next span, and the very last
        // one is taken after the loop).
        let length = arc[to] - arc[from];
        for index in from..to {
            let seed = if length > 0.0 {
                t0 + (t1 - t0) * (arc[index] - arc[from]) / length
            } else {
                t0
            };
            let distance = distance_to_curve_near(curve, points[index], seed, lower, upper)?;
            worst = worst.max(distance);
            sum_squares += distance * distance;
            samples += 1;
        }
        // BETWEEN them: the curve's own span samples against the input run, in
        // the index window this span covers, widened by one segment each side.
        let window_start = from.saturating_sub(1);
        let window_end = (to + 1).min(points.len() - 1);
        for fraction in MEASURE_SPAN_FRACTIONS {
            let sample = curve.evaluate(t0 + (t1 - t0) * fraction)?;
            let mut nearest = f64::INFINITY;
            for index in window_start..window_end {
                nearest = nearest.min(distance_to_segment(
                    sample,
                    points[index],
                    points[index + 1],
                ));
            }
            if nearest.is_finite() {
                between = between.max(nearest);
            }
        }
    }
    let distance = distance_to_curve_near(
        curve,
        points[stations[last]],
        parameters[last],
        parameters[last.saturating_sub(1)],
        parameters[last],
    )?;
    worst = worst.max(distance);
    sum_squares += distance * distance;
    samples += 1;
    let rms = (sum_squares / samples as f64).sqrt();
    Ok((worst, rms, between))
}

/// Largest distance from an input point the simplification DROPPED to the
/// polyline through the stations that survived — the input-side floor under
/// every rung of the ladder. See [`PolylineFitReport::dropped_band`].
fn dropped_band(points: &[Vec3], pool: &[usize]) -> f64 {
    let mut worst = 0.0_f64;
    for span in 0..pool.len() - 1 {
        let (from, to) = (pool[span], pool[span + 1]);
        for index in from + 1..to {
            worst = worst.max(distance_to_segment(
                points[index],
                points[from],
                points[to],
            ));
        }
    }
    worst
}

/// Cumulative chord length of `points`, in the input's own order.
fn cumulative_chord(points: &[Vec3]) -> Vec<f64> {
    let mut arc = vec![0.0; points.len()];
    for index in 1..points.len() {
        arc[index] = arc[index - 1] + points[index].sub(points[index - 1]).length();
    }
    arc
}

/// ONE rung: interpolate `stations` and measure the result against the input.
fn fit_rung(
    points: &[Vec3],
    pool: &[usize],
    arc: &[f64],
    maximum_points: usize,
    local_interpolation: bool,
    degree: usize,
) -> Result<(NurbsCurve, Vec<usize>, Vec<f64>, f64, f64, f64), String> {
    let stations = decimate_stations(pool, maximum_points);
    fit_station_set(points, stations, arc, local_interpolation, degree)
}

fn fit_station_set(
    points: &[Vec3],
    stations: Vec<usize>,
    arc: &[f64],
    local_interpolation: bool,
    degree: usize,
) -> Result<(NurbsCurve, Vec<usize>, Vec<f64>, f64, f64, f64), String> {
    let parameters = station_parameters(points, &stations)
        .ok_or_else(|| "fit_polyline: degenerate polyline".to_string())?;
    let kept = stations.iter().map(|index| points[*index]).collect::<Vec<_>>();
    // A run that closes on itself is fitted PERIODICALLY on the global lane:
    // an open interpolant errs most at its ends, and a closed run's ends are
    // the march's origin, not a feature of the curve.
    let closed = kept.len() > degree + 1
        && kept[0].sub(kept[kept.len() - 1]).length() <= 1e-12 * (1.0 + kept[0].length());
    let curve = if local_interpolation {
        interpolate_curve_local(&kept, &parameters, 1.0)?
    } else if closed
        && degree > 3
        && degree % 2 == 1
        && std::env::var("BREP_SECTION_FIT_PERIODIC").as_deref() != Ok("0")
    {
        interpolate_curve_closed_of_degree(&kept[..kept.len() - 1], &parameters, degree)?
    } else {
        interpolate_curve(&kept, degree.min(kept.len() - 1), &parameters)?
    };
    let (deviation, rms, between) =
        measure_polyline_fit(&curve, points, &stations, &parameters, arc)?;
    Ok((curve, stations, parameters, deviation, rms, between))
}

/// Fit ONE rung at the given station budget and measure it — no refinement.
///
/// This is [`fit_polyline`] before the ladder: the bench that pins the fit's
/// convergence orders drives it directly, because the ladder by design does
/// NOT return a 96-station curve when 96 stations miss the tolerance. Its
/// report always exits [`PolylineFitExit::Unrefined`] when the tolerance is
/// unmet.
pub fn fit_polyline_at(
    points: &[Vec3],
    tolerance: f64,
    maximum_points: usize,
    local_interpolation: bool,
) -> Result<PolylineFit, String> {
    let fit = fit_polyline_inner(points, tolerance, maximum_points, local_interpolation, false, 3)?;
    record_polyline_fit(&fit.report);
    Ok(fit)
}

/// Interpolate a curve through a polyline and REFINE it until the curve itself
/// is within `tolerance` of that polyline.
///
/// `tolerance` is the deviation the RESULT must reach; `maximum_points` is
/// where the station ladder STARTS, not where it ends. Rung zero is exactly
/// the fit this routine returned before the ladder existed — same
/// simplification, same conditioning, same decimation, same parameters — so a
/// section that already met its tolerance returns the identical curve, to the
/// bit. Only a MEASURED miss spends anything: the budget doubles, each rung is
/// measured, and the best is kept, up to [`MAX_FIT_STATIONS`] or the input's
/// own station count.
///
/// Where it cannot get there it says so, in [`PolylineFit::report`], rather
/// than returning silently — the interpolant becomes a trim boundary and every
/// mass property downstream is read off it, so "within a step of the marched
/// points" is not good enough. This is the same contract
/// [`offset::fold_locus::fit_locus_pcurve`] already keeps for a traced locus
/// and [`crate::PcurveFitReport`] for a pcurve.
///
/// THE LANE IS ALSO MEASURED. `local_interpolation` is the lane the ladder
/// STARTS in, not the lane that is returned: where the caller's own ladder ends
/// unmet, the other lane's ladder is run on the same input and the better
/// MEASURED result is returned. Neither lane wins in general and the two fail
/// differently — measured on the exact-circle bench and the kink bench below:
///
/// * on a SMOOTH section the global cubic converges at h⁴ and the local lane at
///   h², so at the 96 stations `offset_shell` asks for the local lane delivers
///   286 control points 2.179e-3 from the input and the global one delivers 96
///   at 3.700e-6 — 589x better with a third of the controls, and the local lane
///   cannot reach 1e-7 there at any station count;
/// * at a KINK — a rim through a corner — the local lane's monotonicity limiter
///   holds the corner where the global cubic rings across it: 8.377e-2 against
///   4.095e-1 at the same 32 stations, 4.9x.
///
/// So the flag is a caller's PREFERENCE — which ladder is paid for first — and
/// never a decision about which curve is delivered. `offset_shell` asks for the
/// global lane since 2026-09-13 because its sections are mostly smooth and the
/// global rungs are three times cheaper per station; its corner sections reach
/// the local lane through this fallback instead. This is the
/// same shape as `degenerate_apex_frustum`'s branch selection in
/// `geometry/analytic_surface/intersect.rs`: a deterministic choice between two
/// constructions on their measured error, never a tolerance, and it can only
/// return the more accurate of the two.
///
/// Escape hatch: `BREP_POLYLINE_FIT_REFINE=0` fits rung zero alone, for
/// bisecting a moved number against the pre-ladder tree.
///
/// [`offset::fold_locus::fit_locus_pcurve`]: crate::offset
pub fn fit_polyline(
    points: &[Vec3],
    tolerance: f64,
    maximum_points: usize,
    local_interpolation: bool,
) -> Result<PolylineFit, String> {
    fit_polyline_of_degree(points, tolerance, maximum_points, local_interpolation, 3)
}

/// [`fit_polyline`] with the GLOBAL lane's interpolation degree chosen by the
/// caller (the local lane is cubic Hermite whatever is asked). A marched
/// section's stations lie on both carriers to the march's residual, and a
/// degree-`p` interpolant through them errs by O(h^(p+1)) between stations: on
/// `BadBoolean`'s sphere×bore rim at its own 135 stations the cubic sits
/// 2.5e-7 off a carrier with a mean signed bias of −1.2e-8, the quintic 8.5e-9
/// and +6e-11 — the SAME stations, knots and control-point count, so nothing
/// downstream that samples per knot pays for it (2026-09-27).
pub fn fit_polyline_of_degree(
    points: &[Vec3],
    tolerance: f64,
    maximum_points: usize,
    local_interpolation: bool,
    degree: usize,
) -> Result<PolylineFit, String> {
    fit_polyline_of_degree_with_recovery(points, tolerance, maximum_points, local_interpolation, degree)
        .map(|(fit, _)| fit)
}

/// Internal provenance for the SSI caller: recovering fidelity to the march
/// points does not establish accuracy between them against the carriers.
/// Keep this out of the public fit/report API.
pub(crate) fn fit_polyline_of_degree_with_recovery(
    points: &[Vec3],
    tolerance: f64,
    maximum_points: usize,
    local_interpolation: bool,
    degree: usize,
) -> Result<(PolylineFit, bool), String> {
    let refine = std::env::var("BREP_POLYLINE_FIT_REFINE").as_deref() != Ok("0");
    let asked = fit_polyline_inner(points, tolerance, maximum_points, local_interpolation, refine, degree)?;
    if asked.report.met_tolerance() || !refine {
        record_polyline_fit(&asked.report);
        return Ok((asked, false));
    }
    let mut other =
        fit_polyline_inner(points, tolerance, maximum_points, !local_interpolation, refine, degree)?;
    let spent = asked.report.rungs + other.report.rungs;
    let mut best = if other.report.deviation < asked.report.deviation {
        other.report.lane_switched = true;
        other
    } else {
        asked
    };
    // Uniform decimation can spend its whole budget away from a short,
    // highly curved part of the run. Retry a missed fit by adding the worst
    // eligible station per span, within the same station ceiling. Only a
    // measured converged fit replaces the already available fallback.
    let mut spent = spent;
    let mut recovered = false;
    if !best.report.met_tolerance() {
        let (adaptive, rounds) = fit_polyline_adaptive(points, tolerance, maximum_points, degree)?;
        spent += rounds;
        if let Some(mut adaptive) = adaptive {
            adaptive.report.lane_switched = local_interpolation;
            best = adaptive;
            recovered = true;
        }
    }
    // Include discarded attempts in the work ledger.
    best.report.rungs = spent;
    record_polyline_fit(&best.report);
    Ok((best, recovered))
}

/// Redistribute the existing station budget after both uniform ladders miss.
/// Measurements still include every input point, not just eligible stations.
/// This proves fidelity to the sampled run, not accuracy between march points.
fn fit_polyline_adaptive(
    points: &[Vec3],
    tolerance: f64,
    maximum_points: usize,
    degree: usize,
) -> Result<(Option<PolylineFit>, usize), String> {
    let pool = station_pool(points, tolerance);
    let arc = cumulative_chord(points);
    let band = dropped_band(points, &pool);
    let floor = tolerance.max(band);
    let budget = maximum_points.max(2);
    let ceiling = pool.len().min(MAX_FIT_STATIONS)
        .min(budget.saturating_mul(LADDER_BUDGET_FACTOR)).max(budget);
    let mut stations = decimate_stations(&pool, budget);
    // Limit work independently of the station ceiling. A difficult run keeps
    // the existing measured fallback when this recovery does not converge.
    for round in 0..=8 {
        let Ok((curve, current, parameters, deviation, rms, between)) =
            fit_station_set(points, stations, &arc, false, degree)
        else { return Ok((None, round + 1)); };
        stations = current;
        if deviation <= floor {
            let report = PolylineFitReport {
                deviation, rms, between_stations: between, tolerance, floor,
                stations: stations.len(), controls: curve.control_points.len(),
                input_points: points.len(), pool_points: pool.len(), dropped_band: band,
                rungs: round + 1, local_lane: false, lane_switched: false,
                exit: PolylineFitExit::Converged,
            };
            return Ok((Some(PolylineFit {
                curve, parameters, kept: stations.iter().map(|&i| points[i]).collect(), report,
            }), round + 1));
        }
        if stations.len() >= ceiling || round == 8 { return Ok((None, round + 1)); }
        let mut candidates = Vec::new();
        for span in 0..stations.len() - 1 {
            let (from, to) = (stations[span], stations[span + 1]);
            let (t0, t1) = (parameters[span], parameters[span + 1]);
            let lower = parameters[span.saturating_sub(1)];
            let upper = parameters[(span + 2).min(stations.len() - 1)];
            let mut candidate: Option<(usize, f64)> = None;
            for &index in pool.iter().filter(|&&i| i > from && i < to) {
                let length = arc[to] - arc[from];
                let seed = if length > 0.0 {
                    t0 + (t1 - t0) * (arc[index] - arc[from]) / length
                } else { t0 };
                let error = distance_to_curve_near(&curve, points[index], seed, lower, upper)?;
                if candidate.is_none_or(|(_, previous)| error > previous) {
                    candidate = Some((index, error));
                }
            }
            if let Some((index, error)) = candidate {
                if error > floor { candidates.push((index, error)); }
            }
        }
        candidates.sort_by(|a, b| b.1.total_cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
        let added = candidates.len().min(16).min(ceiling - stations.len());
        if added == 0 { return Ok((None, round + 1)); }
        stations.extend(candidates.iter().take(added).map(|&(index, _)| index));
        stations.sort_unstable();
    }
    unreachable!("the bounded refinement returns on its last round")
}

/// The section refiner's refit (`csg/imprint/driver.rs`): every SEED station is
/// kept, and the set grows only where the fit is measured to miss.
///
/// The refiner inserts true intersection points into a run whose standing fit
/// kept only some of the run's stations. Refitting the whole run from its own
/// count interpolates every march station the standing fit had dropped as well;
/// on the skew-cylinder loop that took 160 kept stations to 519 (consistent with
/// ~122 inserts if the old refit kept every run point; the inserts themselves were
/// not counted). Here `seeds` (indices into `points`, any order) are the stations
/// the standing fit kept plus every inserted point, the bend witnesses, and the
/// run's two ends are added. AN ACCEPTED seeded fit interpolates every one of
/// them; the fallback below is the original full refit with its own pool and its
/// own conditioning, which may not. The set then grows exactly as
/// [`fit_polyline_adaptive`] grows its stations: the worst eligible pool station
/// of each span measured over the floor, at most 16 a round, at most 9 rounds.
/// It never passes the ceiling [`fit_polyline_inner`] derives for `full_budget`,
/// and it stands only when its deviation, measured over every input point,
/// meets the same floor (the tolerance, or the input's dropped band where
/// coarser).
///
/// Otherwise the result is EXACTLY `fit_polyline_of_degree(points, tolerance,
/// full_budget, local_interpolation, degree)`, the full-run refit the refiner
/// made before, both recovery lanes included. That covers every seeded
/// measurement that errs or reads non-finite (deviation, floor or a span's
/// error). So a seeded fit can stand only where it meets the same floor, and
/// every other section keeps the old curve, to the bit, or the old call's own
/// error.
/// How [`fit_polyline_seeded_with_path`] produced its fit: the seeded set
/// accepted at the floor, or the original full refit, and why. Returned on
/// every call (not a trace switch), so callers and controls can rely on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SeededPath {
    /// The seeded set met the floor after `rounds` fits.
    Accepted { rounds: usize },
    /// The result IS `fit_polyline_of_degree` at the full budget.
    Fallback(SeededFallback),
}

/// Why a seeded refit fell back to the original full refit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SeededFallback {
    NonFiniteInput,
    Degenerate,
    RefineDisabled,
    PoolTooSmall,
    NonFiniteBandOrArc,
    NonFiniteFloor,
    SeedsOverCeiling,
    /// A forced station is not in the pool `station_pool` builds for the full
    /// refit (simplified or conditioned away): the seeded set cannot respect
    /// the pool contract, so the attempt is declined.
    SeedsOutsideStationPool,
    StationSetFailed,
    NonFiniteFitReading,
    SpanUnreadable,
    CeilingReached,
    RoundsSpent,
    NothingToAdd,
    /// The accepted candidate's curve could not be evaluated, finite, at its
    /// own domain ends: the endpoint readings do not exist, so it is declined.
    EndpointUnreadable,
    /// A tight witness that is ALREADY a station reads over its tight
    /// tolerance at its own interpolation parameter: no station can be added
    /// to fix it, so the candidate is declined.
    TightStationOverTolerance,
}

/// What one representative candidate call spent, for cost accounting apart
/// from the fallback's own report (whose bits must stay the full refit's).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct RepresentativeCost {
    /// Pool representatives the candidate STARTED from (previous kept
    /// stations that are pool members, plus both run ends); every other point
    /// of the run, raw inserts included, is a witness, never forced.
    pub(crate) representatives: usize,
    /// Witnesses: every point of the run.
    pub(crate) witnesses: usize,
    /// `fit_station_set` interpolations the candidate made (at most 9).
    pub(crate) candidate_fits: usize,
    /// Whether the original full refit was then called (a fallback).
    pub(crate) full_refit: bool,
    /// MEASURED, not an acceptance condition (2026-10-04 clarification): on an
    /// accepted candidate, the distance of its curve evaluated at its own
    /// domain start and end from the run's first and last points. The
    /// existing constructors do not guarantee raw bit equality there (a
    /// periodic solve interpolates the distinct stations and reaches the seam
    /// by knot insertion), so no bit condition is imposed; the endpoint
    /// stations keep their indices and order, and both ends are witnesses of
    /// the all-run deviation like every other point. NaN when not accepted.
    pub(crate) start_offset: f64,
    pub(crate) end_offset: f64,
}

/// TRACE ONLY: the near-duplicate floor `station_pool` conditions its pool
/// with, max(1% of the tolerance, 1e-4 of the simplified run's length), or 1%
/// of the tolerance for a run that simplifies to two points. It MIRRORS
/// `station_pool`'s expression (which is not edited); private controls pin it
/// against `station_pool`'s own conditioning. Production decisions use
/// `station_pool` membership itself, never this mirror.
pub(crate) fn polyline_station_pool_floor(points: &[Vec3], tolerance: f64) -> f64 {
    let simplified = simplify_polyline_indices(points, tolerance);
    if simplified.len() > 2 {
        let total: f64 = simplified.windows(2).map(|pair| points[pair[1]].sub(points[pair[0]]).length()).sum();
        (tolerance * 0.01).max(total * 1e-4)
    } else {
        tolerance * 0.01
    }
}

pub(crate) fn fit_polyline_seeded_with_path(
    points: &[Vec3],
    seeds: &[usize],
    tolerance: f64,
    full_budget: usize,
    local_interpolation: bool,
    degree: usize,
) -> Result<(PolylineFit, SeededPath), String> {
    let full_refit = |reason: SeededFallback| {
        fit_polyline_of_degree(points, tolerance, full_budget, local_interpolation, degree)
            .map(|fit| (fit, SeededPath::Fallback(reason)))
    };
    // Inspect unreadable inputs before max/reductions can hide a NaN.
    if !tolerance.is_finite() || points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite()) {
        return full_refit(SeededFallback::NonFiniteInput);
    }
    if points.len() < 3 {
        return full_refit(SeededFallback::Degenerate);
    }
    if std::env::var("BREP_POLYLINE_FIT_REFINE").as_deref() == Ok("0") {
        return full_refit(SeededFallback::RefineDisabled);
    }
    let pool = station_pool(points, tolerance);
    if pool.len() < 2 {
        return full_refit(SeededFallback::PoolTooSmall);
    }
    let arc = cumulative_chord(points);
    let band = dropped_band(points, &pool);
    if !band.is_finite() || arc.iter().any(|value| !value.is_finite()) {
        return full_refit(SeededFallback::NonFiniteBandOrArc);
    }
    let floor = tolerance.max(band);
    if !floor.is_finite() {
        return full_refit(SeededFallback::NonFiniteFloor);
    }
    // The ceiling `fit_polyline_inner` derives for the same budget: the seeded
    // set may grow to it and never past it.
    let budget = full_budget.max(2);
    let ceiling = pool
        .len()
        .min(MAX_FIT_STATIONS)
        .min(budget.saturating_mul(LADDER_BUDGET_FACTOR))
        .max(budget);
    let mut stations = seeds
        .iter()
        .copied()
        .filter(|&index| index < points.len())
        .chain([0, points.len() - 1])
        .collect::<Vec<_>>();
    stations.sort_unstable();
    stations.dedup();
    if stations.len() > ceiling {
        return full_refit(SeededFallback::SeedsOverCeiling);
    }
    // ELIGIBILITY: THE ORIGINAL STATION-POOL CONTRACT. The full refit
    // interpolates `pool`, i.e. `station_pool`'s own simplification and
    // near-duplicate conditioning of this run. Every forced station must be
    // a member of that pool. If one is not (simplified or conditioned away),
    // the seeded set cannot respect the contract, and the attempt is DECLINED
    // to the identical full refit. Seeds are never dropped or moved, and no
    // new threshold is read: membership is `station_pool`'s own answer.
    // Measured on the helmet's 2036x3009 section (2026-10-04, 42f031ae6): an
    // accepted seeded fit with one seed outside the pool met its polyline
    // floor at 6.373e-8 while its carrier mid-span rang to 1.945e-4. That
    // seed is an insert SIMPLIFIED away (traced on c47bedc2a: insert
    // simplified 1, conditioned 0), not conditioned. The round's closest
    // seed pair (station-insert, 8.585e-8, same 59 seeds) sits under the
    // 4.881e-7 near-duplicate floor, but no seed was conditioned out; which
    // seeds that pair joins is not traced. On the bend control the declined
    // insert is the other case, conditioned out against a non-seed run point.
    // Membership covers both.
    if stations.iter().any(|index| pool.binary_search(index).is_err()) {
        return full_refit(SeededFallback::SeedsOutsideStationPool);
    }
    for round in 0..=8 {
        let Ok((curve, current, parameters, deviation, rms, between)) =
            fit_station_set(points, stations, &arc, local_interpolation, degree)
        else {
            return full_refit(SeededFallback::StationSetFailed);
        };
        stations = current;
        if !deviation.is_finite() || !rms.is_finite() || !between.is_finite() {
            return full_refit(SeededFallback::NonFiniteFitReading);
        }
        if deviation <= floor {
            let report = PolylineFitReport {
                deviation,
                rms,
                between_stations: between,
                tolerance,
                floor,
                stations: stations.len(),
                controls: curve.control_points.len(),
                input_points: points.len(),
                pool_points: pool.len(),
                dropped_band: band,
                rungs: round + 1,
                local_lane: local_interpolation,
                lane_switched: false,
                exit: PolylineFitExit::Converged,
            };
            record_polyline_fit(&report);
            let kept = stations.iter().map(|&index| points[index]).collect();
            return Ok((PolylineFit { curve, parameters, kept, report }, SeededPath::Accepted { rounds: round + 1 }));
        }
        if stations.len() >= ceiling {
            return full_refit(SeededFallback::CeilingReached);
        }
        if round == 8 {
            return full_refit(SeededFallback::RoundsSpent);
        }
        let mut candidates = Vec::new();
        for span in 0..stations.len() - 1 {
            let (from, to) = (stations[span], stations[span + 1]);
            let (t0, t1) = (parameters[span], parameters[span + 1]);
            let lower = parameters[span.saturating_sub(1)];
            let upper = parameters[(span + 2).min(stations.len() - 1)];
            let mut candidate: Option<(usize, f64)> = None;
            for &index in pool.iter().filter(|&&i| i > from && i < to) {
                let length = arc[to] - arc[from];
                let seed = if length > 0.0 { t0 + (t1 - t0) * (arc[index] - arc[from]) / length } else { t0 };
                // A span this entry cannot read sends the section to the
                // original refit; the shared measurement itself is unchanged.
                let Ok(error) = distance_to_curve_near(&curve, points[index], seed, lower, upper) else {
                    return full_refit(SeededFallback::SpanUnreadable);
                };
                if !error.is_finite() {
                    return full_refit(SeededFallback::SpanUnreadable);
                }
                if candidate.is_none_or(|(_, previous)| error > previous) {
                    candidate = Some((index, error));
                }
            }
            if let Some((index, error)) = candidate {
                if error > floor {
                    candidates.push((index, error));
                }
            }
        }
        candidates.sort_by(|a, b| b.1.total_cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
        let added = candidates.len().min(16).min(ceiling - stations.len());
        if added == 0 {
            return full_refit(SeededFallback::NothingToAdd);
        }
        stations.extend(candidates.iter().take(added).map(|&(index, _)| index));
        stations.sort_unstable();
    }
    // Every pass of the loop above returns; this is the same fallback, named.
    full_refit(SeededFallback::RoundsSpent)
}

/// REFINER-SPECIFIC candidate (2026-10-04, design review of 8f8c17368): the
/// seeded refit's contract with forced membership replaced by REPRESENTATIVES.
/// `seeds` (the previous fit's kept stations) that are members of
/// `station_pool(points, tolerance)` start the set, with both ends; seeds the
/// pool does not keep, and every refiner insert, are NOT forced as stations.
/// Instead EVERY point of `points` is a fidelity witness: the candidate is
/// accepted only when the interpolant's deviation over all of them (the same
/// `measure_polyline_fit` reading the full refit takes) is within `tolerance`
/// itself, NOT within max(tolerance, dropped_band). The chord band a
/// simplified or conditioned pool leaves is a property of the polyline
/// through the pool, not inherited uncertainty of points that lie on the
/// carriers; it is reported as `dropped_band` and never used as the bar here.
/// Growth is the existing rule: per span, the pool member farthest from the
/// curve, if over `tolerance`, at most 16 per round, 9 rounds, the same
/// ceiling. The caller enables this only for a run whose every ORIGINAL
/// point lies on both carriers. Anything that fails (unreadable, ceiling,
/// rounds, nothing a pool member can add while a witness is still over)
/// returns the identical `fit_polyline_of_degree(points, tolerance,
/// full_budget, local_interpolation, degree)`.
pub(crate) fn fit_polyline_representatives_with_path(
    points: &[Vec3],
    seeds: &[usize],
    tolerance: f64,
    full_budget: usize,
    local_interpolation: bool,
    degree: usize,
) -> Result<(PolylineFit, SeededPath, RepresentativeCost), String> {
    fit_polyline_representatives_with_witnesses(points, seeds, &[], tolerance, tolerance, full_budget, local_interpolation, degree)
}

/// [`fit_polyline_representatives_with_path`] with TIGHT witnesses (root
/// ruling, 2026-10-04): `tight` indices of `points` (the refiner's own
/// inserted true-carrier points) must lie within `tight_tolerance` of the
/// curve (the refiner's existing carrier floor), while every other point is
/// held to `tolerance` as before. Measured on e90c30c94's bend: round 2's
/// inserts were already within 1e-7 of round 1's 63-station curve, so the
/// candidate added nothing, the carrier worst stayed 1.762e-8 against the
/// 1e-8 floor, and the bounded retry ended on the full refit. A tight
/// witness over its tolerance is REPRESENTED: the pool member nearest it (by
/// arc) inside its span becomes a station; if its span has none to add, the
/// candidate declines to the identical full refit (NothingToAdd). Inserts are
/// never forced as stations and no original point is held tighter. With
/// `tight` empty this is exactly the original candidate.
#[allow(clippy::too_many_arguments)]
pub(crate) fn fit_polyline_representatives_with_witnesses(
    points: &[Vec3],
    seeds: &[usize],
    tight: &[usize],
    tight_tolerance: f64,
    tolerance: f64,
    full_budget: usize,
    local_interpolation: bool,
    degree: usize,
) -> Result<(PolylineFit, SeededPath, RepresentativeCost), String> {
    let mut cost = RepresentativeCost { witnesses: points.len(), start_offset: f64::NAN, end_offset: f64::NAN, ..RepresentativeCost::default() };
    let full_refit = |reason: SeededFallback, mut cost: RepresentativeCost| {
        cost.full_refit = true;
        fit_polyline_of_degree(points, tolerance, full_budget, local_interpolation, degree)
            .map(|fit| (fit, SeededPath::Fallback(reason), cost))
    };
    if !tolerance.is_finite()
        || !tight_tolerance.is_finite()
        || points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite())
    {
        return full_refit(SeededFallback::NonFiniteInput, cost);
    }
    if points.len() < 3 {
        return full_refit(SeededFallback::Degenerate, cost);
    }
    if std::env::var("BREP_POLYLINE_FIT_REFINE").as_deref() == Ok("0") {
        return full_refit(SeededFallback::RefineDisabled, cost);
    }
    let mut tight: Vec<usize> = tight.iter().copied().filter(|&index| index < points.len()).collect();
    tight.sort_unstable();
    tight.dedup();
    let pool = station_pool(points, tolerance);
    if pool.len() < 2 || pool[0] != 0 || pool[pool.len() - 1] != points.len() - 1 {
        return full_refit(SeededFallback::PoolTooSmall, cost);
    }
    let arc = cumulative_chord(points);
    let band = dropped_band(points, &pool);
    if !band.is_finite() || arc.iter().any(|value| !value.is_finite()) {
        return full_refit(SeededFallback::NonFiniteBandOrArc, cost);
    }
    let budget = full_budget.max(2);
    let ceiling = pool
        .len()
        .min(MAX_FIT_STATIONS)
        .min(budget.saturating_mul(LADDER_BUDGET_FACTOR))
        .max(budget);
    let mut stations = seeds
        .iter()
        .copied()
        .filter(|index| pool.binary_search(index).is_ok())
        .chain([0, points.len() - 1])
        .collect::<Vec<_>>();
    stations.sort_unstable();
    stations.dedup();
    cost.representatives = stations.len();
    if stations.len() > ceiling {
        return full_refit(SeededFallback::SeedsOverCeiling, cost);
    }
    for round in 0..=8 {
        cost.candidate_fits += 1;
        let Ok((curve, current, parameters, deviation, rms, between)) =
            fit_station_set(points, stations, &arc, local_interpolation, degree)
        else {
            return full_refit(SeededFallback::StationSetFailed, cost);
        };
        stations = current;
        if !deviation.is_finite() || !rms.is_finite() || !between.is_finite() {
            return full_refit(SeededFallback::NonFiniteFitReading, cost);
        }
        // Every tight witness against the curve, near its own span (the same
        // bracketed reader the growth below uses). Any unreadable read declines.
        let mut tight_over: Vec<usize> = Vec::new();
        for &index in &tight {
            let span = match stations.binary_search(&index) {
                Ok(position) => {
                    // A station is interpolated, but finite precision is not a
                    // proof: read it at its own interpolation parameter.
                    let distance = curve.evaluate(parameters[position]).map(|p| p.sub(points[index]).length());
                    match distance {
                        Ok(distance) if distance.is_finite() => {
                            if distance > tight_tolerance {
                                return full_refit(SeededFallback::TightStationOverTolerance, cost);
                            }
                        }
                        _ => return full_refit(SeededFallback::SpanUnreadable, cost),
                    }
                    continue;
                }
                // Both run ends are always stations; an index outside the
                // stations' range cannot occur, and is read as unreadable.
                Err(at) if at == 0 || at >= stations.len() => return full_refit(SeededFallback::SpanUnreadable, cost),
                Err(at) => at - 1,
            };
            let (from, to) = (stations[span], stations[span + 1]);
            let (t0, t1) = (parameters[span], parameters[span + 1]);
            let length = arc[to] - arc[from];
            let seed = if length > 0.0 { t0 + (t1 - t0) * (arc[index] - arc[from]) / length } else { t0 };
            let lower = parameters[span.saturating_sub(1)];
            let upper = parameters[(span + 2).min(stations.len() - 1)];
            let Ok(error) = distance_to_curve_near(&curve, points[index], seed, lower, upper) else {
                return full_refit(SeededFallback::SpanUnreadable, cost);
            };
            if !error.is_finite() {
                return full_refit(SeededFallback::SpanUnreadable, cost);
            }
            if error > tight_tolerance {
                tight_over.push(index);
            }
        }
        if deviation <= tolerance && tight_over.is_empty() {
            // The curve at its own domain ends, read and recorded against the
            // run's first and last points (the same reading the controls take
            // on the full refit). Unreadable ends decline; the offsets are
            // reported, not judged by a new bit condition.
            let ends = curve
                .domain()
                .and_then(|[t0, t1]| Ok((curve.evaluate(t0)?, curve.evaluate(t1)?)));
            let Ok((start, end)) = ends else {
                return full_refit(SeededFallback::EndpointUnreadable, cost);
            };
            cost.start_offset = start.sub(points[0]).length();
            cost.end_offset = end.sub(points[points.len() - 1]).length();
            if !(cost.start_offset.is_finite() && cost.end_offset.is_finite()) {
                cost.start_offset = f64::NAN;
                cost.end_offset = f64::NAN;
                return full_refit(SeededFallback::EndpointUnreadable, cost);
            }
            let report = PolylineFitReport {
                deviation,
                rms,
                between_stations: between,
                tolerance,
                floor: tolerance,
                stations: stations.len(),
                controls: curve.control_points.len(),
                input_points: points.len(),
                pool_points: pool.len(),
                dropped_band: band,
                rungs: round + 1,
                local_lane: local_interpolation,
                lane_switched: false,
                exit: PolylineFitExit::Converged,
            };
            record_polyline_fit(&report);
            let kept = stations.iter().map(|&index| points[index]).collect();
            return Ok((PolylineFit { curve, parameters, kept, report }, SeededPath::Accepted { rounds: round + 1 }, cost));
        }
        if stations.len() >= ceiling {
            return full_refit(SeededFallback::CeilingReached, cost);
        }
        if round == 8 {
            return full_refit(SeededFallback::RoundsSpent, cost);
        }
        let mut candidates = Vec::new();
        for span in 0..stations.len() - 1 {
            let (from, to) = (stations[span], stations[span + 1]);
            let (t0, t1) = (parameters[span], parameters[span + 1]);
            let lower = parameters[span.saturating_sub(1)];
            let upper = parameters[(span + 2).min(stations.len() - 1)];
            let mut candidate: Option<(usize, f64)> = None;
            for &index in pool.iter().filter(|&&i| i > from && i < to) {
                let length = arc[to] - arc[from];
                let seed = if length > 0.0 { t0 + (t1 - t0) * (arc[index] - arc[from]) / length } else { t0 };
                let Ok(error) = distance_to_curve_near(&curve, points[index], seed, lower, upper) else {
                    return full_refit(SeededFallback::SpanUnreadable, cost);
                };
                if !error.is_finite() {
                    return full_refit(SeededFallback::SpanUnreadable, cost);
                }
                if candidate.is_none_or(|(_, previous)| error > previous) {
                    candidate = Some((index, error));
                }
            }
            if let Some((index, error)) = candidate {
                if error > tolerance {
                    candidates.push((index, error));
                }
            }
        }
        // A tight witness over its tolerance is represented by the pool member
        // nearest it by arc inside its span (not already chosen), if any.
        for &index in &tight_over {
            let at = stations.binary_search(&index).unwrap_err();
            let (from, to) = (stations[at - 1], stations[at]);
            let nearest = pool
                .iter()
                .copied()
                .filter(|&member| member > from && member < to)
                .min_by(|&a, &b| (arc[a] - arc[index]).abs().total_cmp(&(arc[b] - arc[index]).abs()).then_with(|| a.cmp(&b)));
            if let Some(member) = nearest {
                // Already an ordinary candidate: raise it to the tight priority
                // so the first 16 cannot pass it over.
                match candidates.iter_mut().find(|(chosen, _)| *chosen == member) {
                    Some(entry) => entry.1 = f64::INFINITY,
                    None => candidates.push((member, f64::INFINITY)),
                }
            }
        }
        candidates.sort_by(|a, b| b.1.total_cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
        let added = candidates.len().min(16).min(ceiling - stations.len());
        if added == 0 {
            // A witness is still over its tolerance and no pool member in any
            // span can represent it: decline, never force the raw point.
            return full_refit(SeededFallback::NothingToAdd, cost);
        }
        stations.extend(candidates.iter().take(added).map(|&(index, _)| index));
        stations.sort_unstable();
    }
    full_refit(SeededFallback::RoundsSpent, cost)
}


fn fit_polyline_inner(
    points: &[Vec3],
    tolerance: f64,
    maximum_points: usize,
    local_interpolation: bool,
    refine: bool,
    degree: usize,
) -> Result<PolylineFit, String> {
    let pool = station_pool(points, tolerance);
    if pool.len() < 2 {
        return Err("fit_polyline: degenerate polyline".into());
    }
    let arc = cumulative_chord(points);
    let band = dropped_band(points, &pool);
    let floor = tolerance.max(band);
    let mut budget = maximum_points.max(2);
    let ceiling = pool
        .len()
        .min(MAX_FIT_STATIONS)
        .min(budget.saturating_mul(LADDER_BUDGET_FACTOR))
        .max(budget);
    let (mut curve, mut stations, mut parameters, mut deviation, mut rms, mut between) =
        fit_rung(points, &pool, &arc, budget, local_interpolation, degree)?;
    let mut rungs = 1usize;
    let mut exit = if deviation <= floor {
        PolylineFitExit::Converged
    } else {
        PolylineFitExit::Unrefined
    };
    while refine && deviation > floor {
        if budget >= ceiling {
            exit = if ceiling >= pool.len() {
                PolylineFitExit::InputExhausted
            } else {
                PolylineFitExit::StationCeiling
            };
            break;
        }
        budget = (budget * 2).min(ceiling);
        let rung = fit_rung(points, &pool, &arc, budget, local_interpolation, degree)?;
        rungs += 1;
        if rung.3 >= deviation * STALL_FACTOR {
            if rung.3 < deviation {
                (curve, stations, parameters, deviation, rms, between) = rung;
            }
            // A rung can stall INSIDE the tolerance — it bought little because
            // there was little left to buy. The exit names what the curve is,
            // so `met_tolerance()` and `exit` can never disagree.
            exit = if deviation <= floor {
                PolylineFitExit::Converged
            } else {
                PolylineFitExit::Stalled
            };
            break;
        }
        (curve, stations, parameters, deviation, rms, between) = rung;
        if deviation <= floor {
            exit = PolylineFitExit::Converged;
        }
    }
    let report = PolylineFitReport {
        deviation,
        rms,
        between_stations: between,
        tolerance,
        floor,
        stations: stations.len(),
        controls: curve.control_points.len(),
        input_points: points.len(),
        pool_points: pool.len(),
        dropped_band: band,
        rungs,
        local_lane: local_interpolation,
        lane_switched: false,
        exit,
    };
    Ok(PolylineFit {
        curve,
        parameters,
        kept: stations.iter().map(|index| points[*index]).collect(),
        report,
    })
}

/// Per-operation tally of polyline fits and how many missed their tolerance.
///
/// The same shape, and for the same reason, as [`crate::PcurveFitLedger`]:
/// [`fit_polyline`] is called from the imprint driver and from the offset
/// reintersection, neither of which has a diagnostics record to write into. An
/// operation that returns [`KernelDiagnostics`] opens a [`PolylineFitScope`];
/// every fit made on that thread while the scope is open is tallied here, and
/// closing it hands the tally back for [`PolylineFitLedger::report_into`].
/// Scopes nest — closing a child folds it into its parent — and a scope DROPPED
/// without being closed is discarded, so a refused boolean attempt does not
/// leave its fits on the caller's tally. With no scope open, recording is a
/// no-op.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PolylineFitLedger {
    /// Fits recorded, met ones included.
    pub fits: u64,
    /// Fits whose returned curve did not reach its tolerance.
    pub unmet: u64,
    /// Unmet fits that stopped at [`MAX_FIT_STATIONS`].
    pub station_ceiling: u64,
    /// Unmet fits that had already used every station the input offers.
    pub input_exhausted: u64,
    /// Unmet fits whose last doubling bought nothing.
    pub stalled: u64,
    /// Unmet fits that never refined (a single-rung fit, or the escape hatch).
    pub unrefined: u64,
    /// Fits that spent more than one rung, met or not.
    pub refined: u64,
    /// Fits that met their tolerance only with every station the input offers —
    /// a SATURATED reading (see [`PolylineFitReport::saturated`]).
    pub met_at_pool: u64,
    /// Fits returned from the lane the caller did NOT ask for, because the
    /// caller's own ladder ended unmet and the other lane measured better.
    pub lane_switched: u64,
    /// Extra rungs spent over every fit — what the ladder cost.
    pub extra_rungs: u64,
    /// Largest deviation among the unmet fits; zero when there are none.
    pub worst_unmet_deviation: f64,
    /// Largest [`PolylineFitReport::dropped_band`] over every fit.
    pub worst_dropped_band: f64,
}

impl PolylineFitLedger {
    fn record(&mut self, report: &PolylineFitReport) {
        self.fits += 1;
        self.extra_rungs += (report.rungs - 1) as u64;
        if report.rungs > 1 {
            self.refined += 1;
        }
        self.worst_dropped_band = self.worst_dropped_band.max(report.dropped_band);
        if report.lane_switched {
            self.lane_switched += 1;
        }
        if report.met_tolerance() {
            if report.saturated() {
                self.met_at_pool += 1;
            }
            return;
        }
        self.unmet += 1;
        match report.exit {
            PolylineFitExit::Converged => {}
            PolylineFitExit::StationCeiling => self.station_ceiling += 1,
            PolylineFitExit::InputExhausted => self.input_exhausted += 1,
            PolylineFitExit::Stalled => self.stalled += 1,
            PolylineFitExit::Unrefined => self.unrefined += 1,
        }
        self.worst_unmet_deviation = self.worst_unmet_deviation.max(report.deviation);
    }

    fn fold(&mut self, child: &PolylineFitLedger) {
        self.fits += child.fits;
        self.unmet += child.unmet;
        self.station_ceiling += child.station_ceiling;
        self.input_exhausted += child.input_exhausted;
        self.stalled += child.stalled;
        self.unrefined += child.unrefined;
        self.refined += child.refined;
        self.met_at_pool += child.met_at_pool;
        self.lane_switched += child.lane_switched;
        self.extra_rungs += child.extra_rungs;
        self.worst_unmet_deviation = self.worst_unmet_deviation.max(child.worst_unmet_deviation);
        self.worst_dropped_band = self.worst_dropped_band.max(child.worst_dropped_band);
    }

    /// Write the tally into an operation's diagnostics: the `fit.polyline.*`
    /// counters always, plus one `fit.polyline.unmet_tolerance` event at
    /// [`DiagnosticSeverity::Degraded`] when any fit missed its tolerance.
    /// Degraded, not Error: the curve is still the kernel's best interpolant of
    /// the points it was given and the solid stays shippable, but it no longer
    /// claims the accuracy the caller asked for.
    pub fn report_into(&self, diagnostics: &mut KernelDiagnostics) {
        diagnostics.count_n("fit.polyline.fits", self.fits);
        diagnostics.count_n("fit.polyline.unmet_tolerance", self.unmet);
        diagnostics.count_n("fit.polyline.exit.station_ceiling", self.station_ceiling);
        diagnostics.count_n("fit.polyline.exit.input_exhausted", self.input_exhausted);
        diagnostics.count_n("fit.polyline.exit.stalled", self.stalled);
        diagnostics.count_n("fit.polyline.exit.unrefined", self.unrefined);
        diagnostics.count_n("fit.polyline.refined", self.refined);
        diagnostics.count_n("fit.polyline.met_at_pool", self.met_at_pool);
        diagnostics.count_n("fit.polyline.lane_switched", self.lane_switched);
        diagnostics.count_n("fit.polyline.extra_rungs", self.extra_rungs);
        diagnostics.measure_max("fit.polyline.worst_unmet_deviation", self.worst_unmet_deviation);
        diagnostics.measure_max("fit.polyline.worst_dropped_band", self.worst_dropped_band);
        if self.unmet > 0 {
            diagnostics.event(
                DiagnosticSeverity::Degraded,
                KernelStage::Intersect,
                "fit.polyline.unmet_tolerance",
                format!(
                    "{} of {} polyline fits did not reach the tolerance they were given \
                     (station ceiling {}, input exhausted {}, stalled {}, unrefined {}); \
                     worst deviation {:.3e}; the simplification's own band reaches {:.3e}",
                    self.unmet,
                    self.fits,
                    self.station_ceiling,
                    self.input_exhausted,
                    self.stalled,
                    self.unrefined,
                    self.worst_unmet_deviation,
                    self.worst_dropped_band,
                ),
            );
        }
    }
}

thread_local! {
    /// The open [`PolylineFitScope`]s on this thread, innermost last.
    static POLYLINE_FIT_SCOPES: std::cell::RefCell<Vec<PolylineFitLedger>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// An open tally of the polyline fits made on this thread — see
/// [`PolylineFitLedger`]. Open it where an operation starts, [`close`] it where
/// the operation returns its diagnostics; a `?` that unwinds past it drops the
/// tally.
///
/// [`close`]: PolylineFitScope::close
pub struct PolylineFitScope {
    /// Not `Send`: the scope must close on the thread that opened it.
    _thread_bound: std::marker::PhantomData<*const ()>,
}

impl PolylineFitScope {
    pub fn open() -> Self {
        POLYLINE_FIT_SCOPES.with(|scopes| scopes.borrow_mut().push(PolylineFitLedger::default()));
        Self {
            _thread_bound: std::marker::PhantomData,
        }
    }

    /// Take the tally, folding it into the enclosing scope if there is one.
    pub fn close(self) -> PolylineFitLedger {
        let ledger = POLYLINE_FIT_SCOPES.with(|scopes| {
            let mut scopes = scopes.borrow_mut();
            let ledger = scopes.pop().unwrap_or_default();
            if let Some(parent) = scopes.last_mut() {
                parent.fold(&ledger);
            }
            ledger
        });
        std::mem::forget(self);
        ledger
    }
}

impl Drop for PolylineFitScope {
    fn drop(&mut self) {
        POLYLINE_FIT_SCOPES.with(|scopes| {
            scopes.borrow_mut().pop();
        });
    }
}

fn record_polyline_fit(report: &PolylineFitReport) {
    // Per-fit trace for the fit bench and the lane census, the way
    // `BREP_DEBUG_PCURVE_FIT` traces a pcurve fit.
    if std::env::var_os("BREP_DEBUG_POLYLINE_FIT").is_some() {
        eprintln!("POLYLINE-FIT {report:?}");
    }
    POLYLINE_FIT_SCOPES.with(|scopes| {
        if let Some(open) = scopes.borrow_mut().last_mut() {
            open.record(report);
        }
    });
}



/// Cox–de Boor basis over a RAW (possibly unclamped) knot array — the local
/// helper the periodic interpolation needs; `KnotVector` validation rightly
/// rejects unclamped arrays, so this stays private to the fit module.
fn raw_basis(knots: &[f64], degree: usize, span: usize, parameter: f64) -> Vec<f64> {
    let mut basis = vec![0.0; degree + 1];
    let mut left = vec![0.0; degree + 1];
    let mut right = vec![0.0; degree + 1];
    basis[0] = 1.0;
    for j in 1..=degree {
        left[j] = parameter - knots[span + 1 - j];
        right[j] = knots[span + j] - parameter;
        let mut saved = 0.0;
        for r in 0..j {
            let denominator = right[r + 1] + left[j - r];
            let temp = if denominator.abs() > 0.0 {
                basis[r] / denominator
            } else {
                0.0
            };
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        basis[j] = saved;
    }
    basis
}

/// Boehm single-knot insertion on raw arrays (degree fixed by caller).
fn raw_insert_knot(knots: &mut Vec<f64>, controls: &mut Vec<Vec3>, degree: usize, parameter: f64) {
    // span: last index with knots[span] <= parameter, clamped to the valid
    // control range (the textbook find_span clamp — inserting at the domain
    // end otherwise indexes one past the control array).
    let span = knots
        .iter()
        .rposition(|&knot| knot <= parameter + 1e-14)
        .unwrap()
        .min(controls.len() - 1);
    let mut fresh = Vec::with_capacity(controls.len() + 1);
    fresh.extend_from_slice(&controls[..=span - degree]);
    for i in span - degree + 1..=span {
        let denominator = knots[i + degree] - knots[i];
        let alpha = if denominator.abs() > 0.0 {
            (parameter - knots[i]) / denominator
        } else {
            0.0
        };
        fresh.push(
            controls[i - 1]
                .scale(1.0 - alpha)
                .add(controls[i].scale(alpha)),
        );
    }
    fresh.extend_from_slice(&controls[span..]);
    *controls = fresh;
    knots.insert(span + 1, parameter);
}

/// EXACT closed (periodic) cubic interpolation. `points` are the S >= 4
/// distinct stations (first NOT repeated); `parameters` has S+1 strictly
/// increasing values whose last entry closes the period. The cyclic
/// collocation system is solved densely (S is small for lofts), and the
/// periodic B-spline is re-expressed in CLAMPED form by Boehm-inserting the
/// domain ends to full multiplicity — the representation every kernel
/// consumer expects — so the seam is C² by construction, not by welding.
pub fn interpolate_curve_closed(points: &[Vec3], parameters: &[f64]) -> Result<NurbsCurve, String> {
    interpolate_curve_closed_of_degree(points, parameters, 3)
}

/// [`interpolate_curve_closed`] at an odd `degree` (3 or 5): the cyclic knot
/// line carries `degree` knots either side of the period, and interpolating at
/// the knots is well posed for an odd degree. A closed marched section fitted
/// OPEN errs most at its ends — the march's origin, where the loop was cut —
/// and that end error is what the periodic fit removes (2026-09-27).
pub fn interpolate_curve_closed_of_degree(
    points: &[Vec3],
    parameters: &[f64],
    degree: usize,
) -> Result<NurbsCurve, String> {
    if degree % 2 == 0 || degree < 3 {
        return Err("interpolate_curve_closed: the degree must be odd and at least 3".into());
    }
    let station_count = points.len();
    if station_count < degree + 1 {
        return Err("interpolate_curve_closed: need more stations than the degree".into());
    }
    if parameters.len() != station_count + 1 {
        return Err(
            "interpolate_curve_closed: parameters must have one more entry than points".into(),
        );
    }
    if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err("interpolate_curve_closed: parameters must increase".into());
    }
    let period = parameters[station_count] - parameters[0];
    // Cyclic knot line u_j = t_{j mod S} + floor(j/S)·T for j in −p..S+p,
    // stored with offset p (the degree): raw[k] = u_{k−p}.
    let cyclic = |j: i64| -> f64 {
        let s = station_count as i64;
        let wrap = j.div_euclid(s);
        parameters[j.rem_euclid(s) as usize] + wrap as f64 * period
    };
    let raw_knots: Vec<f64> = (-(degree as i64)..=(station_count as i64 + degree as i64)).map(cyclic).collect();
    // Collocation: row i evaluates the cubic basis at t_i; the span in the
    // raw array is the one containing t_i (raw index i+p == u_i).
    let mut matrix = vec![vec![0.0; station_count]; station_count];
    for i in 0..station_count {
        let span = i + degree;
        let basis = raw_basis(&raw_knots, degree, span, parameters[i]);
        for (offset, value) in basis.iter().enumerate() {
            // Control j = span − degree + offset in unclamped indexing, i.e.
            // cyclic control (i + offset − p) mod S.
            let index = (i as i64 + offset as i64 - degree as i64).rem_euclid(station_count as i64) as usize;
            matrix[i][index] += value;
        }
    }
    let solve_axis = |axis: fn(Vec3) -> f64| {
        solve_dense(
            matrix.clone(),
            points.iter().copied().map(axis).collect::<Vec<_>>(),
        )
    };
    let xs = solve_axis(|point| point.x)?;
    let ys = solve_axis(|point| point.y)?;
    let zs = solve_axis(|point| point.z)?;
    let cyclic_controls: Vec<Vec3> = (0..station_count)
        .map(|index| Vec3::new(xs[index], ys[index], zs[index]))
        .collect();
    // Window covering [t_0, t_S]: controls D_{−p..S−1} cyclically.
    let mut window_controls: Vec<Vec3> = (-(degree as i64)..(station_count as i64))
        .map(|j| cyclic_controls[j.rem_euclid(station_count as i64) as usize])
        .collect();
    let mut window_knots = raw_knots.clone();
    // Clamp both domain ends to full multiplicity (degree insertions each —
    // the ends currently sit at multiplicity 1).
    for _ in 0..degree {
        raw_insert_knot(
            &mut window_knots,
            &mut window_controls,
            degree,
            parameters[0],
        );
    }
    for _ in 0..degree {
        raw_insert_knot(
            &mut window_knots,
            &mut window_controls,
            degree,
            parameters[station_count],
        );
    }
    // Slice out the clamped sub-curve over [t_0, t_S]: knots from the first
    // occurrence of t_0 through the last of t_S, controls aligned so that
    // control k pairs with knot span k..k+degree+1.
    let first = window_knots
        .iter()
        .position(|&knot| (knot - parameters[0]).abs() < 1e-12)
        .ok_or("interpolate_curve_closed: clamp lost the start knot")?;
    let last = window_knots
        .iter()
        .rposition(|&knot| (knot - parameters[station_count]).abs() < 1e-12)
        .ok_or("interpolate_curve_closed: clamp lost the end knot")?;
    let clamped_knots: Vec<f64> = window_knots[first..=last].to_vec();
    let control_count = clamped_knots.len() - degree - 1;
    let clamped_controls: Vec<Vec4> = window_controls[first..first + control_count]
        .iter()
        .map(|point| Vec4::from_point(*point, 1.0))
        .collect();
    NurbsCurve::new(degree, clamped_knots, clamped_controls)
}

