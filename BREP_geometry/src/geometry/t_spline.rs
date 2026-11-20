//! Analysis-suitable T-spline surfaces (Golovanov 2020 §3.9) and their exact
//! conversion to one tensor-product NURBS.
//!
//! A T-spline is a point-based rational sum
//! `S(u, v) = Σ w_i p_i N_i(u) M_i(v) / Σ w_i N_i(u) M_i(v)` in which every
//! control point (anchor) carries its own local knot vectors. Conventions:
//!
//! - **Index-space T-mesh.** Knot values `s_0 ≤ … ≤ s_Mu` and `t_0 ≤ … ≤ t_Mv`
//!   sit on integer index lines; the mesh is a set of axis-aligned unit edges
//!   between integer points. Repeated knots are zero-length index edges.
//! - **Clamped ends.** `s_0 = … = s_d` and `s_{Mu−d} = … = s_Mu` (same in `v`);
//!   the index lines `0..=d` and `Mu−d..=Mu` are complete (added
//!   automatically), so a full grid is exactly the clamped tensor NURBS.
//! - **Odd degrees** `1, 3, 5`; anchors are mesh vertices (points touched by
//!   both a horizontal and a vertical edge) with index in
//!   `[(d+1)/2, M−(d+1)/2]` in each direction.
//! - **Local knots by ray casting**: from the anchor along `±u`, the first
//!   `(d+1)/2` index positions where the ray meets a vertical edge (an edge
//!   ending on the ray counts), plus the anchor: `d + 2` knots.
//! - **Analysis-suitable**: no horizontal T-junction extension meets a
//!   vertical one (face extension: `(d+1)/2` perpendicular edges into the
//!   missing edge's direction; edge extension: `(d−1)/2` the other way).
//!   Other meshes are refused by name.
//!
//! Conversion: every local knot vector is a subsequence of the global index
//! knot vector, so each blending function is refined exactly (Boehm knot
//! insertion) onto the global B-splines and the homogeneous points summed.

use std::collections::HashSet;

use crate::{NurbsSurface, Vec3, Vec4};

/// Largest index extent of a T-mesh in either direction (bounds the
/// converted tensor net at `MAX_T_MESH_LINES²` points).
pub const MAX_T_MESH_LINES: usize = 512;

/// Smallest admitted ratio of the least to the largest weight (see
/// `bezier_triangle::MIN_WEIGHT_RATIO`: the converted net's weights are
/// convex combinations of the control weights).
pub const MIN_T_SPLINE_WEIGHT_RATIO: f64 = 1e-9;

/// Smallest gap between two distinct knot values. The tensor evaluator drops
/// a basis-recursion term whose knot span is at or below an absolute 1e-12,
/// so a nonzero span below that would convert to a surface that evaluates
/// differently; this keeps three orders of margin.
pub const MIN_KNOT_SPAN: f64 = 1e-9;

/// Largest admitted magnitude of a knot value or control point coordinate
/// (differences of admitted values stay finite).
pub const MAX_T_SPLINE_VALUE: f64 = 1e100;

/// A straight run of T-mesh edges in index space, end indices inclusive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TMeshSegment {
    /// Along `u` on index row `j`, from column `i0` to column `i1`.
    Horizontal { j: usize, i0: usize, i1: usize },
    /// Along `v` on index column `i`, from row `j0` to row `j1`.
    Vertical { i: usize, j0: usize, j1: usize },
}

/// Why a T-mesh or T-spline was refused.
#[derive(Clone, Debug, PartialEq)]
pub enum TSplineRefusal {
    /// Only odd degrees 1, 3, 5 have vertex anchors.
    Degree { direction: char, degree: usize },
    /// Knot values not finite, beyond [`MAX_T_SPLINE_VALUE`], or decreasing.
    Knots { direction: char, index: usize },
    /// Two distinct knot values at most [`MIN_KNOT_SPAN`] apart: the converted
    /// NURBS evaluator treats such a span as empty.
    KnotSpan { direction: char, index: usize, span: f64 },
    /// The first or last `degree + 1` knots differ, or nothing is left between them.
    NotClamped { direction: char },
    /// An interior knot value repeated more than `degree` times.
    Multiplicity { direction: char, value: f64 },
    /// Too few or too many index lines.
    Extent { direction: char, lines: usize },
    /// A segment outside the index range or of zero length.
    Segment { segment: TMeshSegment },
    /// An interior point where a segment ends without meeting a perpendicular edge.
    Dangling { i: usize, j: usize },
    /// An interior L corner: the faces around it are not rectangles.
    Corner { i: usize, j: usize },
    /// A ray from an anchor left the mesh before collecting its knots.
    Ray { i: usize, j: usize },
    /// A horizontal and a vertical T-junction extension intersect.
    NotAnalysisSuitable { horizontal: (usize, usize), vertical: (usize, usize) },
    /// The number of control points does not match the anchors.
    PointCount { expected: usize, found: usize },
    /// A control point or weight that is not finite.
    NonFinite { index: usize },
    /// A control point farther than [`MAX_T_SPLINE_VALUE`] from the origin: the
    /// derivative arithmetic would overflow.
    Magnitude { index: usize },
    /// A weight that is not positive.
    Weight { index: usize, weight: f64 },
    /// Weights spanning more than [`MIN_T_SPLINE_WEIGHT_RATIO`].
    WeightRatio { min: f64, max: f64 },
    /// A parameter outside the domain.
    Parameter { u: f64, v: f64 },
    /// The tensor constructor refused the converted net.
    Conversion { reason: String },
}

impl std::fmt::Display for TSplineRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Degree { direction, degree } => {
                write!(f, "t-spline: degree {degree} in {direction}; only odd degrees 1, 3, 5 are supported")
            }
            Self::Knots { direction, index } => {
                write!(f, "t-spline: {direction} knot {index} is not finite, beyond {MAX_T_SPLINE_VALUE}, or decreases")
            }
            Self::KnotSpan { direction, index, span } => write!(
                f,
                "t-spline: {direction} knots {index} and {} are {span} apart; distinct knots must be more than {MIN_KNOT_SPAN} apart",
                index + 1
            ),
            Self::NotClamped { direction } => write!(
                f,
                "t-spline: the {direction} knots must repeat their end values degree + 1 times with a nonzero span between"
            ),
            Self::Multiplicity { direction, value } => {
                write!(f, "t-spline: interior {direction} knot {value} repeats more than the degree")
            }
            Self::Extent { direction, lines } => write!(
                f,
                "t-spline: {lines} {direction} index lines is outside the supported range (at most {MAX_T_MESH_LINES})"
            ),
            Self::Segment { segment } => write!(f, "t-spline: segment {segment:?} is empty or outside the index range"),
            Self::Dangling { i, j } => {
                write!(f, "t-spline: an edge ends at ({i}, {j}) without meeting a perpendicular edge")
            }
            Self::Corner { i, j } => {
                write!(f, "t-spline: interior L corner at ({i}, {j}); every T-mesh face must be a rectangle")
            }
            Self::Ray { i, j } => write!(f, "t-spline: the knot ray from anchor ({i}, {j}) left the mesh"),
            Self::NotAnalysisSuitable { horizontal, vertical } => write!(
                f,
                "t-spline: not analysis-suitable: the extension of the horizontal T-junction {horizontal:?} meets \
                 that of the vertical T-junction {vertical:?}"
            ),
            Self::PointCount { expected, found } => {
                write!(f, "t-spline: the mesh has {expected} anchors, {found} control points were given")
            }
            Self::NonFinite { index } => write!(f, "t-spline: control point {index} is not finite"),
            Self::Magnitude { index } => {
                write!(f, "t-spline: control point {index} lies beyond {MAX_T_SPLINE_VALUE} from the origin")
            }
            Self::Weight { index, weight } => {
                write!(f, "t-spline: control point {index} has weight {weight}; weights must be positive")
            }
            Self::WeightRatio { min, max } => write!(
                f,
                "t-spline: weights span {min}..{max}; the smallest must be at least {MIN_T_SPLINE_WEIGHT_RATIO} of the largest"
            ),
            Self::Parameter { u, v } => write!(f, "t-spline: ({u}, {v}) is outside the domain"),
            Self::Conversion { reason } => write!(f, "t-spline: tensor conversion refused: {reason}"),
        }
    }
}

impl std::error::Error for TSplineRefusal {}

/// One anchor: its index-space position and its local knot vectors, given
/// as index lines (`d + 2` each, increasing) into the global knots.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TAnchor {
    pub i: usize,
    pub j: usize,
    pub lines_u: Vec<usize>,
    pub lines_v: Vec<usize>,
}

/// A validated analysis-suitable T-mesh with its anchors.
#[derive(Clone, Debug)]
pub struct TMesh {
    degree_u: usize,
    degree_v: usize,
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
    /// Unit edge `(i, j) – (i + 1, j)`.
    horizontal: HashSet<(usize, usize)>,
    /// Unit edge `(i, j) – (i, j + 1)`.
    vertical: HashSet<(usize, usize)>,
    anchors: Vec<TAnchor>,
}

fn check_knots(direction: char, degree: usize, knots: &[f64]) -> Result<(), TSplineRefusal> {
    if degree % 2 == 0 || degree > 5 {
        return Err(TSplineRefusal::Degree { direction, degree });
    }
    let lines = knots.len();
    if lines < 2 * degree + 2 || lines > MAX_T_MESH_LINES {
        return Err(TSplineRefusal::Extent { direction, lines });
    }
    for (index, pair) in knots.windows(2).enumerate() {
        if !(pair[0].abs() <= MAX_T_SPLINE_VALUE && pair[1].abs() <= MAX_T_SPLINE_VALUE) || pair[1] < pair[0] {
            return Err(TSplineRefusal::Knots { direction, index: index + 1 });
        }
        let span = pair[1] - pair[0];
        // At or below the bound: the tensor constructor's knot identity
        // tolerance (1e-9) treats such knots as equal.
        if span > 0.0 && span <= MIN_KNOT_SPAN {
            return Err(TSplineRefusal::KnotSpan { direction, index, span });
        }
    }
    let last = lines - 1;
    if knots[0] != knots[degree] || knots[last - degree] != knots[last] || !(knots[degree] < knots[last - degree]) {
        return Err(TSplineRefusal::NotClamped { direction });
    }
    if knots[degree + 1] == knots[0] || knots[last - degree - 1] == knots[last] {
        return Err(TSplineRefusal::NotClamped { direction });
    }
    let mut start = degree + 1;
    while start < last - degree {
        let mut end = start;
        while end + 1 < last - degree && knots[end + 1] == knots[start] {
            end += 1;
        }
        let interior_run = end - start + 1;
        if interior_run > degree {
            return Err(TSplineRefusal::Multiplicity { direction, value: knots[start] });
        }
        start = end + 1;
    }
    Ok(())
}

impl TMesh {
    /// Builds and validates a T-mesh: knots per direction, the interior
    /// segments (the clamped border lines are added), odd degrees, rectangular
    /// faces, analysis suitability; then casts every anchor's knot rays.
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        segments: &[TMeshSegment],
    ) -> Result<Self, TSplineRefusal> {
        check_knots('u', degree_u, &knots_u)?;
        check_knots('v', degree_v, &knots_v)?;
        let (mu, mv) = (knots_u.len() - 1, knots_v.len() - 1);
        let mut horizontal = HashSet::new();
        let mut vertical = HashSet::new();
        for segment in segments {
            match *segment {
                TMeshSegment::Horizontal { j, i0, i1 } => {
                    if !(i0 < i1 && i1 <= mu && j <= mv) {
                        return Err(TSplineRefusal::Segment { segment: *segment });
                    }
                    horizontal.extend((i0..i1).map(|i| (i, j)));
                }
                TMeshSegment::Vertical { i, j0, j1 } => {
                    if !(j0 < j1 && j1 <= mv && i <= mu) {
                        return Err(TSplineRefusal::Segment { segment: *segment });
                    }
                    vertical.extend((j0..j1).map(|j| (i, j)));
                }
            }
        }
        // The clamped border: rows 0..=dv and mv-dv..=mv, columns likewise.
        for j in (0..=degree_v).chain(mv - degree_v..=mv) {
            horizontal.extend((0..mu).map(|i| (i, j)));
        }
        for i in (0..=degree_u).chain(mu - degree_u..=mu) {
            vertical.extend((0..mv).map(|j| (i, j)));
        }
        let mut mesh = Self { degree_u, degree_v, knots_u, knots_v, horizontal, vertical, anchors: Vec::new() };
        mesh.check_faces()?;
        mesh.check_analysis_suitable()?;
        mesh.anchors = mesh.cast_anchors()?;
        Ok(mesh)
    }

    pub fn degree_u(&self) -> usize {
        self.degree_u
    }

    pub fn degree_v(&self) -> usize {
        self.degree_v
    }

    pub fn knots_u(&self) -> &[f64] {
        &self.knots_u
    }

    pub fn knots_v(&self) -> &[f64] {
        &self.knots_v
    }

    /// Anchors in row-major order (`j`, then `i`): the control point order.
    pub fn anchors(&self) -> &[TAnchor] {
        &self.anchors
    }

    /// Parameter domain `[s_d, s_{Mu−d}] × [t_d, t_{Mv−d}]`.
    pub fn domain(&self) -> ([f64; 2], [f64; 2]) {
        let (mu, mv) = (self.knots_u.len() - 1, self.knots_v.len() - 1);
        (
            [self.knots_u[self.degree_u], self.knots_u[mu - self.degree_u]],
            [self.knots_v[self.degree_v], self.knots_v[mv - self.degree_v]],
        )
    }

    /// Edges at a point: (left, right, down, up).
    fn arms(&self, i: usize, j: usize) -> (bool, bool, bool, bool) {
        (
            i > 0 && self.horizontal.contains(&(i - 1, j)),
            self.horizontal.contains(&(i, j)),
            j > 0 && self.vertical.contains(&(i, j - 1)),
            self.vertical.contains(&(i, j)),
        )
    }

    /// A vertical edge passes through or ends at `(i, j)`.
    fn touches_vertical(&self, i: usize, j: usize) -> bool {
        let (_, _, down, up) = self.arms(i, j);
        down || up
    }

    fn touches_horizontal(&self, i: usize, j: usize) -> bool {
        let (left, right, _, _) = self.arms(i, j);
        left || right
    }

    fn check_faces(&self) -> Result<(), TSplineRefusal> {
        let (mu, mv) = (self.knots_u.len() - 1, self.knots_v.len() - 1);
        for j in 1..mv {
            for i in 1..mu {
                let (left, right, down, up) = self.arms(i, j);
                let count = [left, right, down, up].iter().filter(|arm| **arm).count();
                match count {
                    0 | 4 | 3 => {}
                    1 => return Err(TSplineRefusal::Dangling { i, j }),
                    2 if (left && right) || (down && up) => {}
                    _ => return Err(TSplineRefusal::Corner { i, j }),
                }
            }
        }
        Ok(())
    }

    /// Index positions where a ray from `(i, j)` along `step` meets a
    /// perpendicular edge, at most `count` of them.
    fn ray(&self, i: usize, j: usize, along_u: bool, forward: bool, count: usize) -> Vec<usize> {
        let (mu, mv) = (self.knots_u.len() - 1, self.knots_v.len() - 1);
        let mut hits = Vec::with_capacity(count);
        let mut position = if along_u { i } else { j };
        let limit = if along_u { mu } else { mv };
        while hits.len() < count {
            if forward {
                if position == limit {
                    break;
                }
                position += 1;
            } else {
                if position == 0 {
                    break;
                }
                position -= 1;
            }
            let hit = if along_u { self.touches_vertical(position, j) } else { self.touches_horizontal(i, position) };
            if hit {
                hits.push(position);
            }
        }
        hits
    }

    /// The closed index interval a T-junction's extension covers along its line.
    fn extension(&self, i: usize, j: usize, along_u: bool, missing_forward: bool) -> (usize, usize) {
        let degree = if along_u { self.degree_u } else { self.degree_v };
        let at = if along_u { i } else { j };
        let face = self.ray(i, j, along_u, missing_forward, (degree + 1) / 2);
        let edge = self.ray(i, j, along_u, !missing_forward, (degree - 1) / 2);
        let far = |hits: &Vec<usize>| hits.last().copied().unwrap_or(at);
        let (a, b) = (far(&face), far(&edge));
        (a.min(b), a.max(b))
    }

    fn check_analysis_suitable(&self) -> Result<(), TSplineRefusal> {
        let (mu, mv) = (self.knots_u.len() - 1, self.knots_v.len() - 1);
        // (junction, line index, interval)
        let mut horizontal = Vec::new();
        let mut vertical = Vec::new();
        for j in 1..mv {
            for i in 1..mu {
                let (left, right, down, up) = self.arms(i, j);
                if [left, right, down, up].iter().filter(|arm| **arm).count() != 3 {
                    continue;
                }
                if !left || !right {
                    // Missing a horizontal edge: a horizontal T-junction.
                    horizontal.push(((i, j), self.extension(i, j, true, !right)));
                } else {
                    vertical.push(((i, j), self.extension(i, j, false, !up)));
                }
            }
        }
        for &((hi, hj), (i0, i1)) in &horizontal {
            for &((vi, vj), (j0, j1)) in &vertical {
                if i0 <= vi && vi <= i1 && j0 <= hj && hj <= j1 {
                    return Err(TSplineRefusal::NotAnalysisSuitable { horizontal: (hi, hj), vertical: (vi, vj) });
                }
            }
        }
        Ok(())
    }

    fn cast_anchors(&self) -> Result<Vec<TAnchor>, TSplineRefusal> {
        let (mu, mv) = (self.knots_u.len() - 1, self.knots_v.len() - 1);
        let (hu, hv) = ((self.degree_u + 1) / 2, (self.degree_v + 1) / 2);
        let mut anchors = Vec::new();
        for j in hv..=mv - hv {
            for i in hu..=mu - hu {
                if !(self.touches_horizontal(i, j) && self.touches_vertical(i, j)) {
                    continue;
                }
                let lines = |along_u: bool, half: usize| -> Result<Vec<usize>, TSplineRefusal> {
                    let mut before = self.ray(i, j, along_u, false, half);
                    let after = self.ray(i, j, along_u, true, half);
                    if before.len() < half || after.len() < half {
                        return Err(TSplineRefusal::Ray { i, j });
                    }
                    before.reverse();
                    before.push(if along_u { i } else { j });
                    before.extend(after);
                    Ok(before)
                };
                anchors.push(TAnchor { i, j, lines_u: lines(true, hu)?, lines_v: lines(false, hv)? });
            }
        }
        Ok(anchors)
    }
}

/// One B-spline on the knots `knots` (degree `knots.len() − 2`) at `t`,
/// right-closed at `end` (the domain's upper bound).
fn single_basis(knots: &[f64], t: f64, end: f64) -> f64 {
    let degree = knots.len() - 2;
    let mut values: Vec<f64> = knots
        .windows(2)
        .map(|w| {
            let inside = (w[0] <= t && t < w[1]) || (t == end && w[1] == end && w[0] < w[1]);
            if inside { 1.0 } else { 0.0 }
        })
        .collect();
    for p in 1..=degree {
        for r in 0..values.len() - 1 {
            let mut value = 0.0;
            let left = knots[r + p] - knots[r];
            if left > 0.0 {
                value += (t - knots[r]) / left * values[r];
            }
            let right = knots[r + p + 1] - knots[r + 1];
            if right > 0.0 {
                value += (knots[r + p + 1] - t) / right * values[r + 1];
            }
            values[r] = value;
        }
        values.pop();
    }
    values[0]
}

/// The coefficients of the local B-spline on index lines `lines` in the
/// global B-spline basis of `knots` (Boehm insertion of every missing index
/// line); returns (first global function index, coefficients).
fn refine_onto_global(knots: &[f64], degree: usize, lines: &[usize]) -> (usize, Vec<f64>) {
    let first = lines[0];
    let last = *lines.last().unwrap();
    let mut present: Vec<usize> = lines.to_vec();
    let mut coefficients = vec![1.0];
    for g in first + 1..last {
        if present.contains(&g) {
            continue;
        }
        let t = knots[g];
        let position = present.iter().position(|&index| index > g).unwrap();
        let values: Vec<f64> = present.iter().map(|&index| knots[index]).collect();
        let k = position - 1;
        let n = coefficients.len();
        let mut next = Vec::with_capacity(n + 1);
        for j in 0..=n {
            let alpha = if j + degree <= k {
                1.0
            } else if j >= k + 1 {
                0.0
            } else {
                let span = values[j + degree] - values[j];
                if span > 0.0 { (t - values[j]) / span } else { 0.0 }
            };
            let current = if j < n { coefficients[j] } else { 0.0 };
            let previous = if j > 0 { coefficients[j - 1] } else { 0.0 };
            next.push(alpha * current + (1.0 - alpha) * previous);
        }
        coefficients = next;
        present.insert(position, g);
    }
    (first, coefficients)
}

/// A rational T-spline surface on an analysis-suitable T-mesh.
#[derive(Clone, Debug)]
pub struct TSpline {
    mesh: TMesh,
    /// Homogeneous points `(w·p, w)` in anchor order, scaled so the largest weight is one.
    points: Vec<Vec4>,
}

impl TSpline {
    /// A T-spline from homogeneous points `(w·p, w)` in [`TMesh::anchors`] order.
    pub fn new(mesh: TMesh, points: Vec<Vec4>) -> Result<Self, TSplineRefusal> {
        let expected = mesh.anchors.len();
        if points.len() != expected {
            return Err(TSplineRefusal::PointCount { expected, found: points.len() });
        }
        for (index, point) in points.iter().enumerate() {
            if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite() && point.w.is_finite()) {
                return Err(TSplineRefusal::NonFinite { index });
            }
            if !(point.w > 0.0) {
                return Err(TSplineRefusal::Weight { index, weight: point.w });
            }
            if [point.x, point.y, point.z].iter().any(|value| !((value / point.w).abs() <= MAX_T_SPLINE_VALUE)) {
                return Err(TSplineRefusal::Magnitude { index });
            }
        }
        let max = points.iter().map(|point| point.w).fold(0.0, f64::max);
        let min = points.iter().map(|point| point.w).fold(f64::INFINITY, f64::min);
        if min < MIN_T_SPLINE_WEIGHT_RATIO * max {
            return Err(TSplineRefusal::WeightRatio { min, max });
        }
        // Divide (not multiply by 1/max, which overflows for a subnormal max).
        let points: Vec<Vec4> = points
            .into_iter()
            .map(|point| Vec4 { x: point.x / max, y: point.y / max, z: point.z / max, w: point.w / max })
            .collect();
        if let Some(index) = points.iter().position(|point| !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite())) {
            return Err(TSplineRefusal::NonFinite { index });
        }
        Ok(Self { mesh, points })
    }

    /// A polynomial T-spline (all weights one).
    pub fn from_points(mesh: TMesh, points: &[Vec3]) -> Result<Self, TSplineRefusal> {
        Self::new(mesh, points.iter().map(|point| Vec4::from_point(*point, 1.0)).collect())
    }

    pub fn mesh(&self) -> &TMesh {
        &self.mesh
    }

    pub fn points(&self) -> &[Vec4] {
        &self.points
    }

    fn check_parameter(&self, u: f64, v: f64) -> Result<(), TSplineRefusal> {
        let ([u0, u1], [v0, v1]) = self.mesh.domain();
        if !(u0 <= u && u <= u1 && v0 <= v && v <= v1) {
            return Err(TSplineRefusal::Parameter { u, v });
        }
        Ok(())
    }

    /// The (non-rational) blending function `N_i(u) M_i(v)` of anchor `index`.
    pub fn blending(&self, index: usize, u: f64, v: f64) -> Result<f64, TSplineRefusal> {
        self.check_parameter(u, v)?;
        Ok(self.blending_unchecked(index, u, v))
    }

    fn blending_unchecked(&self, index: usize, u: f64, v: f64) -> f64 {
        let anchor = &self.mesh.anchors[index];
        let ([_, u1], [_, v1]) = self.mesh.domain();
        let ku: Vec<f64> = anchor.lines_u.iter().map(|&line| self.mesh.knots_u[line]).collect();
        let kv: Vec<f64> = anchor.lines_v.iter().map(|&line| self.mesh.knots_v[line]).collect();
        single_basis(&ku, u, u1) * single_basis(&kv, v, v1)
    }

    /// Homogeneous point `Σ (w_i p_i, w_i) N_i M_i`.
    pub fn evaluate_homogeneous(&self, u: f64, v: f64) -> Result<Vec4, TSplineRefusal> {
        self.check_parameter(u, v)?;
        let mut sum = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
        for (index, point) in self.points.iter().enumerate() {
            let value = self.blending_unchecked(index, u, v);
            if value != 0.0 {
                sum = sum.add(point.scale(value));
            }
        }
        Ok(sum)
    }

    /// Point at `(u, v)` (3.9.1).
    pub fn evaluate(&self, u: f64, v: f64) -> Result<Vec3, TSplineRefusal> {
        let h = self.evaluate_homogeneous(u, v)?;
        if !(h.w > 0.0) {
            // Analysis-suitable blending functions sum to one and the weights
            // are positive, so this is unreachable on a valid mesh.
            return Err(TSplineRefusal::Parameter { u, v });
        }
        Ok(Vec3::new(h.x / h.w, h.y / h.w, h.z / h.w))
    }

    /// The same rational surface as one clamped NURBS on the global index
    /// knots: each blending function refined onto the global B-splines.
    pub fn to_nurbs(&self) -> Result<NurbsSurface, TSplineRefusal> {
        let mesh = &self.mesh;
        let (du, dv) = (mesh.degree_u, mesh.degree_v);
        let count_u = mesh.knots_u.len() - du - 1;
        let count_v = mesh.knots_v.len() - dv - 1;
        let zero = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
        let mut net = vec![vec![zero; count_v]; count_u];
        for (anchor, point) in mesh.anchors.iter().zip(&self.points) {
            let (first_u, alpha) = refine_onto_global(&mesh.knots_u, du, &anchor.lines_u);
            let (first_v, beta) = refine_onto_global(&mesh.knots_v, dv, &anchor.lines_v);
            for (a, alpha) in alpha.iter().enumerate() {
                if *alpha == 0.0 {
                    continue;
                }
                for (b, beta) in beta.iter().enumerate() {
                    let cell = &mut net[first_u + a][first_v + b];
                    *cell = cell.add(point.scale(alpha * beta));
                }
            }
        }
        NurbsSurface::new(du, dv, mesh.knots_u.clone(), mesh.knots_v.clone(), net)
            .map_err(|reason| TSplineRefusal::Conversion { reason })
    }
}

