//! Lane J row 9: the sphere or ring torus of a closed mesh that the
//! segmentation holds as one freeform region.
//!
//! Under vertex jitter no segmenter carrier meets its bar, so the whole mesh
//! is one freeform region and row 6's whole-primitive path, which needs a
//! sphere or torus region, never fires. The primitive of the mesh's genus is
//! fitted to the vertices by least squares (deterministic: an algebraic or
//! principal-axis start, then a fixed number of Gauss-Newton steps) and is
//! accepted only when the residual reads as noise rather than shape.
//!
//! The residual test. Jitter moves each vertex independently, so on the
//! right primitive the vertex residuals are uncorrelated with position. A
//! near-primitive of another shape (an ellipsoid, a torus of elliptical
//! section, a bumped ball) leaves a residual that is a smooth, low-order
//! function on the surface: an ellipsoid of axis ratio 1 + e fitted by a
//! sphere leaves e*R times a degree-2 harmonic. The residual is regressed on
//! the 25 low-order functions of the surface's own parameters (spherical
//! harmonics of degree <= 4 for the sphere, the 5 x 5 products of
//! {1, cos, sin, cos 2, sin 2} in both torus angles) and the explained sum of
//! squares is compared with the unexplained variance. Under independent
//! noise `ESS / (RSS / (N - K))` follows a chi-square law of at most K
//! degrees of freedom (the fit's own parameters already zero some of those
//! directions, so K is the lenient count), and the fit is accepted at most at
//! its p = 1e-6 quantile.

/// Number of residual-structure functions on either primitive.
const STRUCTURE_FUNCTIONS: usize = 25;
/// Standard normal deviate of p = 1e-6, one-sided.
const STRUCTURE_Z: f64 = 4.753;
/// Lane J row 11: the local residual scan reads every vertex's 1- to
/// `LOCAL_RINGS`-ring neighbourhood (along mesh edges), the scale between one
/// vertex and the global harmonics' reach.
const LOCAL_RINGS: usize = 4;
/// Family-wise false-alarm rate of the local scan: row 9's p = 1e-6, split
/// over every patch it reads (Bonferroni, valid however the patches overlap).
const LOCAL_FALSE_ALARM: f64 = 1e-6;
/// Gauss-Newton steps: fixed, so the fit is a function of the mesh alone.
const SPHERE_STEPS: usize = 10;
const TORUS_STEPS: usize = 20;

/// A fitted whole primitive.
#[derive(Clone, Copy, Debug)]
pub(crate) enum WholeFit {
    Sphere { center: [f64; 3], radius: f64 },
    Torus { center: [f64; 3], axis: [f64; 3], major: f64, minor: f64 },
}

/// The fit with its residual readings.
#[derive(Clone, Copy, Debug)]
pub(crate) struct WholeFitReport {
    pub(crate) fit: WholeFit,
    /// Largest vertex distance from the fitted surface.
    pub(crate) max_deviation: f64,
    /// `ESS / (RSS / (N - K))` of the residual on the structure functions.
    pub(crate) statistic: f64,
    /// The chi-square quantile it must not exceed.
    pub(crate) quantile: f64,
    /// Lane J row 11: the largest |sum of residuals| / (sigma * sqrt(n)) over
    /// the vertex neighbourhoods, sigma the residual's spread left after the
    /// structure functions.
    pub(crate) local_statistic: f64,
    /// The normal deviate it must not exceed: the two-sided Mills bound at
    /// the local false-alarm rate over every patch read.
    pub(crate) local_threshold: f64,
    pub(crate) vertices: usize,
}

impl WholeFitReport {
    pub(crate) fn residual_is_noise(&self) -> bool {
        self.statistic.is_finite()
            && self.statistic <= self.quantile
            && !(whole_local_enabled() && !(self.local_statistic <= self.local_threshold))
    }
}

/// Lane J row 11, promoted ON 2026-10-16: BREP_HYBRID_WHOLE_LOCAL=0 restores
/// row 9's global test alone.
pub(crate) fn whole_local_enabled() -> bool {
    std::env::var("BREP_HYBRID_WHOLE_LOCAL").map_or(true, |v| v != "0")
}

/// Lane J row 9, promoted ON 2026-10-16: BREP_HYBRID_WHOLE_FREEFORM=0 restores
/// row 6 alone.
pub(crate) fn whole_freeform_enabled() -> bool {
    std::env::var("BREP_HYBRID_WHOLE_FREEFORM").map_or(true, |v| v != "0")
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn length(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}
fn normalized(a: [f64; 3]) -> Option<[f64; 3]> {
    let l = length(a);
    (l > 0.0 && l.is_finite()).then(|| scale(a, 1.0 / l))
}

/// Two unit vectors completing `axis` to an orthonormal frame.
fn frame(axis: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let helper = if axis[0].abs() <= axis[1].abs() && axis[0].abs() <= axis[2].abs() {
        [1.0, 0.0, 0.0]
    } else if axis[1].abs() <= axis[2].abs() {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let e1 = normalized(cross(axis, helper)).unwrap_or([1.0, 0.0, 0.0]);
    (e1, cross(axis, e1))
}

/// Solve the dense system `a x = b` (row-major, n x n) by Gaussian
/// elimination with partial pivoting; `None` when it is singular relative to
/// its largest pivot.
fn solve(mut a: Vec<f64>, mut b: Vec<f64>, n: usize) -> Option<Vec<f64>> {
    let largest = a.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    if !(largest > 0.0) || !largest.is_finite() {
        return None;
    }
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i * n + col].abs().total_cmp(&a[j * n + col].abs()))?;
        if a[pivot * n + col].abs() <= largest * 1e-14 {
            return None;
        }
        if pivot != col {
            for k in 0..n {
                a.swap(pivot * n + k, col * n + k);
            }
            b.swap(pivot, col);
        }
        for row in col + 1..n {
            let f = a[row * n + col] / a[col * n + col];
            if f != 0.0 {
                for k in col..n {
                    a[row * n + k] -= f * a[col * n + k];
                }
                b[row] -= f * b[col];
            }
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut s = b[row];
        for k in row + 1..n {
            s -= a[row * n + k] * x[k];
        }
        x[row] = s / a[row * n + row];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// Least squares `min |J x + r|`: accumulates `JᵀJ` and `Jᵀr` in row order.
fn least_squares(rows: impl Iterator<Item = (Vec<f64>, f64)>, n: usize) -> Option<Vec<f64>> {
    let mut ata = vec![0.0; n * n];
    let mut atb = vec![0.0; n];
    for (j, r) in rows {
        for p in 0..n {
            atb[p] -= j[p] * r;
            for q in 0..n {
                ata[p * n + q] += j[p] * j[q];
            }
        }
    }
    solve(ata, atb, n)
}

/// Eigenvector of the smallest eigenvalue of a symmetric 3 x 3 matrix, by a
/// fixed number of cyclic Jacobi sweeps.
fn smallest_eigenvector(mut m: [[f64; 3]; 3]) -> [f64; 3] {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..32 {
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if m[p][q] == 0.0 {
                continue;
            }
            let theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (mkp, mkq) = (m[k][p], m[k][q]);
                m[k][p] = c * mkp - s * mkq;
                m[k][q] = s * mkp + c * mkq;
            }
            for k in 0..3 {
                let (mpk, mqk) = (m[p][k], m[q][k]);
                m[p][k] = c * mpk - s * mqk;
                m[q][k] = s * mpk + c * mqk;
            }
            for row in v.iter_mut() {
                let (a, b) = (row[p], row[q]);
                row[p] = c * a - s * b;
                row[q] = s * a + c * b;
            }
        }
    }
    let k = (0..3).min_by(|&a, &b| m[a][a].total_cmp(&m[b][b])).unwrap_or(0);
    [v[0][k], v[1][k], v[2][k]]
}

fn fit_sphere(points: &[[f64; 3]], diagonal: f64) -> Option<WholeFit> {
    // Algebraic start: |p|^2 = 2 c.p + k.
    let start = least_squares(points.iter().map(|&p| (vec![2.0 * p[0], 2.0 * p[1], 2.0 * p[2], 1.0], -dot(p, p))), 4)?;
    let mut center = [start[0], start[1], start[2]];
    let mut radius = (start[3] + dot(center, center)).sqrt();
    for _ in 0..SPHERE_STEPS {
        let step = least_squares(
            points.iter().filter_map(|&p| {
                let q = sub(p, center);
                let l = length(q);
                (l > diagonal * 1e-12).then(|| (vec![-q[0] / l, -q[1] / l, -q[2] / l, -1.0], l - radius))
            }),
            4,
        )?;
        center = add(center, [step[0], step[1], step[2]]);
        radius += step[3];
    }
    (radius.is_finite() && radius > diagonal * 1e-9).then_some(WholeFit::Sphere { center, radius })
}

fn torus_angles(p: [f64; 3], center: [f64; 3], axis: [f64; 3], major: f64) -> (f64, f64, f64, f64, [f64; 3]) {
    let q = sub(p, center);
    let z = dot(q, axis);
    let w = sub(q, scale(axis, z));
    let rho = length(w);
    (rho, z, ((rho - major) * (rho - major) + z * z).sqrt(), z, q)
}

fn fit_torus(points: &[[f64; 3]], diagonal: f64) -> Option<WholeFit> {
    let n = points.len() as f64;
    let mut center = scale(points.iter().fold([0.0; 3], |s, &p| add(s, p)), 1.0 / n);
    let mut moment = [[0.0; 3]; 3];
    for &p in points {
        let q = sub(p, center);
        for a in 0..3 {
            for b in 0..3 {
                moment[a][b] += q[a] * q[b];
            }
        }
    }
    let mut axis = normalized(smallest_eigenvector(moment))?;
    let (e1, e2) = frame(axis);
    // Section circle in (rho, z): rho^2 + z^2 = 2 R rho + 2 b z + k.
    let start = least_squares(
        points.iter().map(|&p| {
            let q = sub(p, center);
            let (x, y, z) = (dot(q, e1), dot(q, e2), dot(q, axis));
            let rho = (x * x + y * y).sqrt();
            (vec![2.0 * rho, 2.0 * z, 1.0], -(rho * rho + z * z))
        }),
        3,
    )?;
    let mut major = start[0];
    center = add(center, scale(axis, start[1]));
    let mut minor = (start[2] + major * major + start[1] * start[1]).sqrt();
    let floor = diagonal * 1e-9;
    for _ in 0..TORUS_STEPS {
        let (e1, e2) = frame(axis);
        let step = least_squares(
            points.iter().filter_map(|&p| {
                let (rho, z, e, _, q) = torus_angles(p, center, axis, major);
                if rho <= floor || e <= floor {
                    return None;
                }
                let w = sub(q, scale(axis, z));
                // d = e - r; de/dq = ((rho - R) w / rho + z a) / e.
                let grad = scale(add(scale(w, (rho - major) / rho), scale(axis, z)), 1.0 / e);
                let tilt = z * major / (rho * e);
                Some((
                    vec![-grad[0], -grad[1], -grad[2], tilt * dot(q, e1), tilt * dot(q, e2), -(rho - major) / e, -1.0],
                    e - minor,
                ))
            }),
            7,
        )?;
        center = add(center, [step[0], step[1], step[2]]);
        axis = normalized(add(axis, add(scale(e1, step[3]), scale(e2, step[4]))))?;
        major += step[5];
        minor += step[6];
    }
    (major.is_finite() && minor.is_finite() && minor > floor && major > minor)
        .then_some(WholeFit::Torus { center, axis, major, minor })
}

/// Signed distance and the 25 structure functions at one vertex.
fn residual_and_basis(fit: &WholeFit, p: [f64; 3]) -> Option<(f64, [f64; STRUCTURE_FUNCTIONS])> {
    let mut basis = [0.0; STRUCTURE_FUNCTIONS];
    match *fit {
        WholeFit::Sphere { center, radius } => {
            let q = sub(p, center);
            let l = length(q);
            let u = normalized(q)?;
            let mut k = 0;
            for degree in [3, 4] {
                for a in 0..=degree {
                    for b in 0..=degree - a {
                        let c = degree - a - b;
                        basis[k] = u[0].powi(a as i32) * u[1].powi(b as i32) * u[2].powi(c as i32);
                        k += 1;
                    }
                }
            }
            Some((l - radius, basis))
        }
        WholeFit::Torus { center, axis, major, minor } => {
            let (e1, e2) = frame(axis);
            let q = sub(p, center);
            let (x, y, z) = (dot(q, e1), dot(q, e2), dot(q, axis));
            let rho = (x * x + y * y).sqrt();
            let u = y.atan2(x);
            let v = z.atan2(rho - major);
            let harmonics = |t: f64| [1.0, t.cos(), t.sin(), (2.0 * t).cos(), (2.0 * t).sin()];
            let (hu, hv) = (harmonics(u), harmonics(v));
            for i in 0..5 {
                for j in 0..5 {
                    basis[5 * i + j] = hu[i] * hv[j];
                }
            }
            Some((((rho - major).powi(2) + z * z).sqrt() - minor, basis))
        }
    }
}

/// Wilson-Hilferty chi-square quantile of `k` degrees of freedom at the
/// one-sided normal deviate `z`.
/// The z with 2 phi(z) / z = `tail`, by bisection. The Mills ratio bounds
/// the two-sided normal tail, P(|Z| > z) <= 2 phi(z) / z, so the deviate is
/// conservative.
fn normal_deviate(tail: f64) -> f64 {
    let bound = |z: f64| 2.0 * (-0.5 * z * z).exp() / ((2.0 * std::f64::consts::PI).sqrt() * z);
    let (mut low, mut high) = (1.0, 40.0);
    for _ in 0..100 {
        let mid = 0.5 * (low + high);
        if bound(mid) > tail {
            low = mid;
        } else {
            high = mid;
        }
    }
    high
}

/// Lane J row 13: `BREP_WHOLE_CARRIER_SCAN=0` restores the direct analytic
/// builders and row 6's whole primitive without the local residual scan.
pub(crate) fn whole_carrier_scan_enabled() -> bool {
    std::env::var("BREP_WHOLE_CARRIER_SCAN").map_or(true, |v| v != "0")
}

/// Lane J row 12: the row 11 local scan read on one carrier's own vertex
/// residuals, `residual(v)` for every vertex of the triangles `indices`
/// (flattened), against the same Bonferroni-split threshold. `sigma` is the
/// residual spread left by the carrier's `parameters`, sqrt(RSS / (N - p)).
/// Returns (statistic, threshold); the scan passes when statistic <=
/// threshold, and fails when there are no more vertices than parameters.
pub(crate) fn carrier_local_scan(indices: &[u32], residual: impl Fn(u32) -> f64, parameters: usize) -> (f64, f64) {
    let mut used = indices.to_vec();
    used.sort_unstable();
    used.dedup();
    let residuals: Vec<f64> = used.iter().map(|&v| residual(v)).collect();
    let rss: f64 = residuals.iter().map(|r| r * r).sum();
    let sigma = if used.len() > parameters { (rss / (used.len() - parameters) as f64).sqrt() } else { f64::NAN };
    let (statistic, patches) = local_residual_scan(indices, &used, &residuals, sigma);
    (statistic, normal_deviate(LOCAL_FALSE_ALARM / patches.max(1) as f64))
}

/// Lane J row 13: what the local scan says about one carrier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ScanVerdict {
    /// The scan reads at or under its threshold.
    Accepted,
    /// The scan reads above its threshold, and the residuals outside every
    /// flagged patch pass the independence test the threshold assumes.
    Declined,
    /// The scan cannot decide: it reads above its threshold, but the
    /// residuals are not independent noise (or too few to tell), so the
    /// threshold does not apply. NOT a pass; the caller keeps the status quo.
    Abstained,
}

/// Lane J row 13: the pre-test refuses to call residuals independent on
/// fewer outside edges than this. Below it a 1e-6 lower tail of the edge
/// mean is not carried by the normal approximation (a few dozen correlated
/// terms), so the scan abstains.
pub(crate) const INDEPENDENCE_MIN_EDGES: usize = 100;

/// One reading of [`carrier_scan_verdict`].
#[derive(Clone, Copy, Debug)]
pub(crate) struct CarrierScanReading {
    pub(crate) statistic: f64,
    pub(crate) threshold: f64,
    /// D / (2 s^2) outside the flagged patches (NaN when not measured).
    pub(crate) independence: f64,
    pub(crate) bound: f64,
    pub(crate) outside_edges: usize,
    pub(crate) verdict: ScanVerdict,
}

/// Lane J row 13: row 11's local scan on one carrier's vertex residuals, with
/// the independence pre-test its threshold assumes. The threshold is derived
/// for independent residuals; a mesh whose residuals are coherent along whole
/// rings (an OpenSCAD sphere's per-latitude coordinate encoding) reads as
/// flats everywhere. Every vertex of every patch above the threshold is
/// excluded, and over the mesh edges with neither end excluded, D = mean (r_a
/// - r_b)^2 is read against 2 s^2. For independent zero-mean residuals E[D] =
/// 2 sigma^2 whatever their distribution. With kurtosis k = m4 / s^4 of the
/// outside residuals, one edge term has variance (2k + 2) sigma^4 and two
/// terms sharing a vertex covary by (k - 1) sigma^4, so over E edges with P =
/// sum_v d_v (d_v - 1) ordered sharing pairs sd(D / 2 sigma^2) = sqrt(E (2k +
/// 2) + P (k - 1)) / (2E). The bound is 1 - z sd with z row 9's one-sided
/// 1e-6 deviate. Under it, or on fewer than `INDEPENDENCE_MIN_EDGES` outside
/// edges, the scan abstains.
pub(crate) fn carrier_scan_verdict(indices: &[u32], residual: impl Fn(u32) -> f64, parameters: usize) -> CarrierScanReading {
    let mut used = indices.to_vec();
    used.sort_unstable();
    used.dedup();
    let residuals: Vec<f64> = used.iter().map(|&v| residual(v)).collect();
    let rss: f64 = residuals.iter().map(|r| r * r).sum();
    let sigma = if used.len() > parameters { (rss / (used.len() - parameters) as f64).sqrt() } else { f64::NAN };
    let (statistic, patches) = local_residual_scan(indices, &used, &residuals, sigma);
    let threshold = normal_deviate(LOCAL_FALSE_ALARM / patches.max(1) as f64);
    let mut reading = CarrierScanReading {
        statistic,
        threshold,
        independence: f64::NAN,
        bound: f64::NAN,
        outside_edges: 0,
        verdict: ScanVerdict::Accepted,
    };
    if statistic <= threshold {
        return reading;
    }
    reading.verdict = ScanVerdict::Abstained;
    if !(sigma > 0.0) {
        return reading;
    }
    let mut flagged = vec![false; used.len()];
    walk_patches(indices, &used, &residuals, sigma, |patch, z| {
        if z > threshold {
            for &v in patch {
                flagged[v] = true;
            }
        }
    });
    let neighbours = vertex_neighbours(indices, &used);
    let mut degree = vec![0_usize; used.len()];
    let (mut edges, mut d_sum) = (0_usize, 0.0_f64);
    for (a, list) in neighbours.iter().enumerate() {
        for &b in list.iter().filter(|&&b| b > a) {
            if !flagged[a] && !flagged[b] {
                edges += 1;
                degree[a] += 1;
                degree[b] += 1;
                d_sum += (residuals[a] - residuals[b]).powi(2);
            }
        }
    }
    let outside: Vec<f64> = (0..used.len()).filter(|&v| !flagged[v]).map(|v| residuals[v]).collect();
    reading.outside_edges = edges;
    if edges < INDEPENDENCE_MIN_EDGES || outside.is_empty() {
        return reading;
    }
    let mean = outside.iter().sum::<f64>() / outside.len() as f64;
    let s2 = outside.iter().map(|r| (r - mean).powi(2)).sum::<f64>() / outside.len() as f64;
    let m4 = outside.iter().map(|r| (r - mean).powi(4)).sum::<f64>() / outside.len() as f64;
    if !(s2 > 0.0) {
        return reading;
    }
    let kurtosis = m4 / (s2 * s2);
    let pairs: f64 = degree.iter().map(|&d| (d * d.saturating_sub(1)) as f64).sum();
    let e = edges as f64;
    let sd = (e * (2.0 * kurtosis + 2.0) + pairs * (kurtosis - 1.0).max(0.0)).sqrt() / (2.0 * e);
    reading.independence = d_sum / e / (2.0 * s2);
    reading.bound = 1.0 - STRUCTURE_Z * sd;
    if reading.independence >= reading.bound {
        reading.verdict = ScanVerdict::Declined;
    }
    reading
}

/// Lane J row 11: a flat or dimple too small for the degree <= 4 harmonics
/// (a 26-triangle cap on a 988-triangle ball) moves a few neighbouring
/// vertices the same way. Under independent jitter the residual summed over
/// n neighbouring vertices is about N(0, n sigma^2); every 1- to
/// `LOCAL_RINGS`-ring neighbourhood is read and the largest normalised sum
/// is returned with the patch count. `sigma` is the residual's spread left
/// after the global structure functions, the noise the sum is read against.
fn local_residual_scan(indices: &[u32], used: &[u32], residuals: &[f64], sigma: f64) -> (f64, usize) {
    if !(sigma > 0.0) {
        return (f64::INFINITY, used.len() * LOCAL_RINGS);
    }
    let mut worst = 0.0_f64;
    walk_patches(indices, used, residuals, sigma, |_, z| worst = worst.max(z));
    (worst, used.len() * LOCAL_RINGS)
}

/// Every vertex's 1- to `LOCAL_RINGS`-ring patch (indices into `used`) with
/// its normalised residual sum, in the order `local_residual_scan` reads them.
fn walk_patches(indices: &[u32], used: &[u32], residuals: &[f64], sigma: f64, mut each: impl FnMut(&[usize], f64)) {
    let neighbours = vertex_neighbours(indices, used);
    let mut ring_of = vec![usize::MAX; used.len()];
    for centre in 0..used.len() {
        let mut patch = vec![centre];
        ring_of[centre] = 0;
        let mut sum = residuals[centre];
        let mut start = 0;
        for ring in 1..=LOCAL_RINGS {
            let end = patch.len();
            for i in start..end {
                for &n in &neighbours[patch[i]] {
                    if ring_of[n] == usize::MAX {
                        ring_of[n] = ring;
                        patch.push(n);
                        sum += residuals[n];
                    }
                }
            }
            start = end;
            each(&patch, sum.abs() / (sigma * (patch.len() as f64).sqrt()));
        }
        for &v in &patch {
            ring_of[v] = usize::MAX;
        }
    }
}

/// Sorted, deduplicated edge neighbours of every vertex of `used`.
fn vertex_neighbours(indices: &[u32], used: &[u32]) -> Vec<Vec<usize>> {
    let index = |v: u32| used.binary_search(&v).ok();
    let mut neighbours = vec![Vec::<usize>::new(); used.len()];
    for t in indices.chunks_exact(3) {
        for k in 0..3 {
            if let (Some(a), Some(b)) = (index(t[k]), index(t[(k + 1) % 3])) {
                neighbours[a].push(b);
                neighbours[b].push(a);
            }
        }
    }
    for list in &mut neighbours {
        list.sort_unstable();
        list.dedup();
    }
    neighbours
}

fn chi_square_quantile(k: f64, z: f64) -> f64 {
    let h = 2.0 / (9.0 * k);
    k * (1.0 - h + z * h.sqrt()).powi(3)
}

/// Fit the primitive of `genus` (0 sphere, 1 ring torus) to the vertices the
/// triangles use, and read its residual. `None` when the fit fails.
pub(crate) fn fit_whole_primitive(positions: &[f64], indices: &[u32], genus: i64) -> Option<WholeFitReport> {
    let used: std::collections::BTreeSet<u32> = indices.iter().copied().collect();
    let points: Vec<[f64; 3]> = used
        .iter()
        .map(|&v| {
            let i = 3 * v as usize;
            [positions[i], positions[i + 1], positions[i + 2]]
        })
        .collect();
    if points.len() < 4 * STRUCTURE_FUNCTIONS {
        return None;
    }
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for p in &points {
        for a in 0..3 {
            low[a] = low[a].min(p[a]);
            high[a] = high[a].max(p[a]);
        }
    }
    let diagonal = length(sub(high, low));
    let fit = match genus {
        0 => fit_sphere(&points, diagonal)?,
        1 => fit_torus(&points, diagonal)?,
        _ => return None,
    };
    let k = STRUCTURE_FUNCTIONS;
    let mut gram = vec![0.0; k * k];
    let mut moment = vec![0.0; k];
    let mut total = 0.0;
    let mut max_deviation = 0.0_f64;
    let mut residuals = Vec::with_capacity(points.len());
    for &p in &points {
        let (d, basis) = residual_and_basis(&fit, p)?;
        residuals.push(d);
        total += d * d;
        max_deviation = max_deviation.max(d.abs());
        for a in 0..k {
            moment[a] += basis[a] * d;
            for b in 0..k {
                gram[a * k + b] += basis[a] * basis[b];
            }
        }
    }
    let beta = solve(gram, moment.clone(), k)?;
    let explained = beta.iter().zip(&moment).map(|(b, m)| b * m).sum::<f64>().max(0.0);
    let unexplained = (total - explained).max(0.0);
    let dof = (points.len() - k) as f64;
    let statistic = if unexplained > 0.0 { explained / (unexplained / dof) } else { f64::INFINITY };
    let used: Vec<u32> = used.into_iter().collect();
    let (local_statistic, patches) = local_residual_scan(indices, &used, &residuals, (unexplained / dof).sqrt());
    Some(WholeFitReport {
        fit,
        max_deviation,
        statistic,
        quantile: chi_square_quantile(k as f64, STRUCTURE_Z),
        local_statistic,
        local_threshold: normal_deviate(LOCAL_FALSE_ALARM / patches as f64),
        vertices: points.len(),
    })
}

