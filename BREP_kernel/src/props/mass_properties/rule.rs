//! Which Gauss–Legendre rule reads ONE span of the integrand, and where that
//! span is cut.
//!
//! The integrator's fixed 8-point rule is exact for a polynomial integrand of
//! degree 15, and every integrand it meets is polynomial — *until a weight is
//! not constant*. A rational curve or surface carries its weight function in
//! the denominator: the Green integrand along a pcurve `C = P/w` is `(P ×
//! P′)/w²`, and a rational patch's area and volume integrands carry `W(u, v)`
//! to the third power. A fixed-order rule does not integrate those exactly,
//! and what it misses depends on the WEIGHTS, not on the geometry: a Möbius
//! re-weighting moves no point of a curve by more than a few 1e-16 and moved
//! the groove document's face area by 6.0e-7 of itself, and a sphere's 90°
//! rational arcs put its area 1.7e-10 low while every other reading of the
//! same body is exact.
//!
//! # The rule, and its comparand
//!
//! A Gauss–Legendre rule of `n` points on a function analytic inside the
//! Bernstein ellipse `E_ρ` of the panel converges as `ρ^(−2n)` (Trefethen,
//! *Approximation Theory and Approximation Practice*, Thm 19.3) — `2n` is the
//! rule's own order, and `ρ` is fixed by the integrand's nearest singularity,
//! which for a rational integrand is the nearest complex ROOT OF THE WEIGHT.
//! So the panel is read at the smallest tabulated order whose convergence
//! factor `ρ^(−2n)` is at or below **`f64::EPSILON`** — the panel's own
//! rounding is the comparand, not a geometric tolerance — and where no
//! tabulated order reaches it the panel is halved and each half asked again.
//!
//! Measured against that bound on the probe set (the record's §1): the error
//! of the accepted rule is between 1 and 25 times `ρ^(−2n)`, so a panel taken
//! at the bound reads its integrand to a few 1e-15 of itself — round-off, and
//! a hundred thousand times tighter than the volume drift band.
//!
//! A span whose weights are all equal has no root, takes the 8-point rule on
//! one panel, and so reads BIT FOR BIT what it read before this rule existed;
//! that is most of every model. `BREP_MASS_RATIONAL_RULE=0` forces that path
//! everywhere, which is the A/B switch for the cost.
use super::*;
use crate::curve::knot_find_span;

/// The tabulated orders, cheapest first. 24 points reach the bound at
/// ρ = 2.13; below that a panel is halved instead (a pole that close to a
/// span is a Möbius re-weighting of λ ≳ 10, or a vendor weight of the same
/// shape).
pub(super) const ORDERS: [usize; 4] = [8, 12, 16, 24];

/// How many times a span may be halved before its panels are read at the
/// highest order whatever their ρ. A pole sits `2^−MAX_SPLITS` of the span
/// from it only when the weight all but vanishes inside the span, which is a
/// curve or patch that is already outside what the evaluator can read.
const MAX_SPLITS: usize = 20;

const GAUSS_X12: [f64; 12] = [
    -0.9815606342467192, -0.9041172563704749, -0.7699026741943047, -0.5873179542866175,
    -0.3678314989981802, -0.1252334085114689, 0.1252334085114689, 0.3678314989981802,
    0.5873179542866175, 0.7699026741943047, 0.9041172563704749, 0.9815606342467192,
];
const GAUSS_W12: [f64; 12] = [
    0.04717533638651183, 0.10693932599531843, 0.16007832854334622, 0.20316742672306592,
    0.2334925365383548, 0.24914704581340277, 0.24914704581340277, 0.2334925365383548,
    0.20316742672306592, 0.16007832854334622, 0.10693932599531843, 0.04717533638651183,
];
const GAUSS_X16: [f64; 16] = [
    -0.9894009349916499, -0.9445750230732326, -0.8656312023878318, -0.755404408355003,
    -0.6178762444026438, -0.45801677765722737, -0.2816035507792589, -0.09501250983763744,
    0.09501250983763744, 0.2816035507792589, 0.45801677765722737, 0.6178762444026438,
    0.755404408355003, 0.8656312023878318, 0.9445750230732326, 0.9894009349916499,
];
const GAUSS_W16: [f64; 16] = [
    0.027152459411754096, 0.062253523938647894, 0.09515851168249279, 0.12462897125553388,
    0.14959598881657674, 0.16915651939500254, 0.18260341504492358, 0.1894506104550685,
    0.1894506104550685, 0.18260341504492358, 0.16915651939500254, 0.14959598881657674,
    0.12462897125553388, 0.09515851168249279, 0.062253523938647894, 0.027152459411754096,
];
const GAUSS_X24: [f64; 24] = [
    -0.9951872199970213, -0.9747285559713095, -0.9382745520027328, -0.8864155270044011,
    -0.820001985973903, -0.7401241915785544, -0.6480936519369755, -0.5454214713888396,
    -0.4337935076260451, -0.3150426796961634, -0.1911188674736163, -0.06405689286260563,
    0.06405689286260563, 0.1911188674736163, 0.3150426796961634, 0.4337935076260451,
    0.5454214713888396, 0.6480936519369755, 0.7401241915785544, 0.820001985973903,
    0.8864155270044011, 0.9382745520027328, 0.9747285559713095, 0.9951872199970213,
];
const GAUSS_W24: [f64; 24] = [
    0.0123412297999872, 0.028531388628933663, 0.04427743881741981, 0.05929858491543678,
    0.0733464814110803, 0.08619016153195327, 0.09761865210411388, 0.10744427011596563,
    0.1155056680537256, 0.12167047292780339, 0.1258374563468283, 0.12793819534675216,
    0.12793819534675216, 0.1258374563468283, 0.12167047292780339, 0.1155056680537256,
    0.10744427011596563, 0.09761865210411388, 0.08619016153195327, 0.0733464814110803,
    0.05929858491543678, 0.04427743881741981, 0.028531388628933663, 0.0123412297999872,
];

/// The most stations one direction of a tensor block can ask for. DEFINED as
/// the tensor evaluator's own limit rather than repeating its value: the
/// station arrays in `integration.rs` are sized by this, `deriv1_tensor_each`
/// refuses a block wider than its own, and its callers PROPAGATE that refusal
/// rather than falling back — so a drift between the two is a hard
/// mass-properties failure, not a slower path.
pub(super) const MAX_ORDER: usize = crate::surface::TENSOR_MAX_BLOCK;

/// The ladder must fit in the block. Checked at compile time because deriving
/// `MAX_ORDER` only covers one direction: were the evaluator's block ever to
/// SHRINK, a tabulated order above it would index past those station arrays at
/// run time, on whichever model first asked for that order.
const _: () = {
    let mut index = 0;
    while index < ORDERS.len() {
        assert!(
            ORDERS[index] <= MAX_ORDER,
            "a tabulated Gauss order exceeds the tensor evaluator's block limit"
        );
        index += 1;
    }
};

/// One panel of a span and the rule it is read at.
#[derive(Clone, Copy, Debug)]
pub(super) struct Panel {
    pub(super) lo: f64,
    pub(super) hi: f64,
    pub(super) order: usize,
}

impl Panel {
    pub(super) fn nodes(&self) -> (&'static [f64], &'static [f64]) {
        match self.order {
            12 => (&GAUSS_X12, &GAUSS_W12),
            16 => (&GAUSS_X16, &GAUSS_W16),
            24 => (&GAUSS_X24, &GAUSS_W24),
            _ => (&GAUSS_X, &GAUSS_W),
        }
    }

    /// The `(parameter, weight)` of each of this panel's stations, with the
    /// panel's half-width folded into the weight.
    pub(super) fn stations(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        let half = (self.hi - self.lo) * 0.5;
        let middle = (self.hi + self.lo) * 0.5;
        let (x, w) = self.nodes();
        (0..x.len()).map(move |index| (middle + half * x[index], w[index] * half))
    }
}

/// The whole span at the base rule: what every polynomial integrand gets, and
/// bit for bit the rule the integrator had before this module.
fn whole(lo: f64, hi: f64) -> Vec<Panel> {
    vec![Panel { lo, hi, order: GAUSS_COUNT }]
}

/// `BREP_MASS_RATIONAL_RULE=0` keeps one base-order panel per span
/// everywhere — the A/B switch for this rule's cost, and the way a reading is
/// compared against what the integrator read before it.
pub(super) fn rational_rule_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("BREP_MASS_RATIONAL_RULE").as_deref() != Ok("0"))
}

// --- the weight polynomial and its roots ------------------------------------

#[derive(Clone, Copy, Debug)]
struct Complex {
    re: f64,
    im: f64,
}

impl Complex {
    fn add(self, other: Complex) -> Complex {
        Complex { re: self.re + other.re, im: self.im + other.im }
    }
    fn sub(self, other: Complex) -> Complex {
        Complex { re: self.re - other.re, im: self.im - other.im }
    }
    fn mul(self, other: Complex) -> Complex {
        Complex {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }
    fn div(self, other: Complex) -> Complex {
        let denominator = other.re * other.re + other.im * other.im;
        Complex {
            re: (self.re * other.re + self.im * other.im) / denominator,
            im: (self.im * other.re - self.re * other.im) / denominator,
        }
    }
    fn abs(self) -> f64 {
        self.re.hypot(self.im)
    }
}

/// Power-basis coefficients of the polynomial through `values` at the
/// equispaced points `j / degree` of the unit span (Newton divided
/// differences, expanded). The weight of a degree-`p` span is a degree-`p`
/// polynomial, so `p + 1` values determine it exactly.
fn power_basis(values: &[f64]) -> Vec<f64> {
    let degree = values.len() - 1;
    let sample: Vec<f64> = (0..=degree)
        .map(|j| if degree == 0 { 0.0 } else { j as f64 / degree as f64 })
        .collect();
    let mut table = values.to_vec();
    for level in 1..=degree {
        for j in (level..=degree).rev() {
            table[j] = (table[j] - table[j - 1]) / (sample[j] - sample[j - level]);
        }
    }
    let mut coefficients = vec![0.0f64; degree + 1];
    for k in (0..=degree).rev() {
        let mut next = vec![0.0f64; degree + 1];
        for i in 0..degree {
            next[i + 1] += coefficients[i];
            next[i] -= sample[k] * coefficients[i];
        }
        next[0] += table[k];
        coefficients = next;
    }
    coefficients
}

/// `|s| + |1 − s|` over the base order's own ellipse — the largest the
/// Bernstein basis can sum to inside it.
///
/// `Σ_i |B_i^p(z)| = (|z| + |1 − z|)^p`, and the ellipse the base rule needs
/// (`ρ_base = f64::EPSILON^(−1/16)` ≈ 9.51) reaches `|x| ≤ (ρ + 1/ρ)/2` in the
/// rule's own [−1, 1], which is `|s|, |1 − s| ≤ 2.906` in the span's [0, 1].
const BERNSTEIN_REACH: f64 = 5.812;

/// True when no root of the span's weight can lie inside the base order's
/// ellipse, from the CONTROL weights alone — no sampling, no root finding.
///
/// A span's weight in Bernstein form has coefficients in the convex hull of
/// the B-spline weights it reads (Bézier extraction is a chain of convex
/// combinations), so `|w(z) − w̄| ≤ spread · Σ|B_i(z)| ≤ spread ·
/// BERNSTEIN_REACH^degree` there. Below `w̄` that cannot vanish.
///
/// This is what keeps a vendor patch whose weights differ in their last bits
/// — an imported body is full of them — on the base rule at the cost of a
/// dozen comparisons, instead of sampling and solving for roots that are
/// nowhere near it.
fn weights_are_far_from_a_root(low: f64, high: f64, degree: usize) -> bool {
    let middle = 0.5 * (low + high);
    middle > 0.0 && (high - low) * 0.5 * BERNSTEIN_REACH.powi(degree as i32) < middle
}

/// Every complex root of `coefficients` (ascending powers), in the unit
/// parameter of the span: closed form to degree 2, Durand–Kerner above it.
/// Coefficients within 1e-14 of the largest are dropped from the top first, so
/// a weight that is constant to round-off has no roots and takes the base
/// rule.
fn roots(mut coefficients: Vec<f64>) -> Vec<Complex> {
    let scale = coefficients.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if !(scale > 0.0) {
        return Vec::new();
    }
    while coefficients.len() > 1 && coefficients.last().unwrap().abs() <= 1e-14 * scale {
        coefficients.pop();
    }
    let degree = coefficients.len() - 1;
    if degree == 0 {
        return Vec::new();
    }
    if degree == 1 {
        return vec![Complex { re: -coefficients[0] / coefficients[1], im: 0.0 }];
    }
    if degree == 2 {
        let (c, b, a) = (coefficients[0], coefficients[1], coefficients[2]);
        let discriminant = b * b - 4.0 * a * c;
        return if discriminant >= 0.0 {
            let root = discriminant.sqrt();
            vec![
                Complex { re: (-b + root) / (2.0 * a), im: 0.0 },
                Complex { re: (-b - root) / (2.0 * a), im: 0.0 },
            ]
        } else {
            let imaginary = (-discriminant).sqrt() / (2.0 * a);
            let real = -b / (2.0 * a);
            vec![Complex { re: real, im: imaginary }, Complex { re: real, im: -imaginary }]
        };
    }
    let lead = coefficients[degree];
    let monic: Vec<f64> = coefficients.iter().map(|v| v / lead).collect();
    let evaluate = |z: Complex| {
        monic
            .iter()
            .rev()
            .fold(Complex { re: 0.0, im: 0.0 }, |acc, &k| acc.mul(z).add(Complex { re: k, im: 0.0 }))
    };
    let mut estimates: Vec<Complex> = (0..degree)
        .map(|k| {
            // Off-axis seeds, so a real root never starts on top of another.
            let angle = 0.4 + std::f64::consts::TAU * k as f64 / degree as f64;
            Complex { re: 0.5 + 0.9 * angle.cos(), im: 0.9 * angle.sin() }
        })
        .collect();
    for _ in 0..100 {
        let mut moved = 0.0f64;
        for i in 0..degree {
            let mut denominator = Complex { re: 1.0, im: 0.0 };
            for j in 0..degree {
                if i != j {
                    denominator = denominator.mul(estimates[i].sub(estimates[j]));
                }
            }
            if denominator.abs() <= f64::MIN_POSITIVE {
                continue;
            }
            let step = evaluate(estimates[i]).div(denominator);
            estimates[i] = estimates[i].sub(step);
            moved = moved.max(step.abs());
        }
        if moved <= 1e-14 {
            break;
        }
    }
    estimates
}

/// The Bernstein-ellipse parameter of `root` (in the SPAN's unit parameter)
/// seen from the panel `[lo, hi]` of that same unit parameter: the rule's
/// error on the panel decays as ρ^(−2n).
fn ellipse_parameter(root: Complex, lo: f64, hi: f64) -> f64 {
    let width = hi - lo;
    if !(width > 0.0) {
        return f64::INFINITY;
    }
    let z = Complex { re: (2.0 * (root.re - lo) / width) - 1.0, im: 2.0 * root.im / width };
    let sum = 0.5 * (z.sub(Complex { re: 1.0, im: 0.0 }).abs() + z.add(Complex { re: 1.0, im: 0.0 }).abs());
    sum + (sum * sum - 1.0).max(0.0).sqrt()
}

/// The cheapest tabulated order whose ρ^(−2n) is at or below `f64::EPSILON`,
/// or None when even the highest does not reach it on this panel.
fn order_for(rho: f64) -> Option<usize> {
    if !(rho > 1.0) {
        return None;
    }
    let needed = -f64::EPSILON.ln();
    ORDERS
        .iter()
        .copied()
        .find(|&order| 2.0 * order as f64 * rho.ln() >= needed)
}

/// Panels of `[0, 1]` (the span's unit parameter) for a weight with these
/// roots, halving toward whichever root no tabulated order can outrun.
fn unit_panels(lo: f64, hi: f64, roots: &[Complex], depth: usize, out: &mut Vec<Panel>) {
    let rho = roots
        .iter()
        .map(|&root| ellipse_parameter(root, lo, hi))
        .fold(f64::INFINITY, f64::min);
    if let Some(order) = order_for(rho) {
        out.push(Panel { lo, hi, order });
        return;
    }
    if depth >= MAX_SPLITS {
        out.push(Panel { lo, hi, order: MAX_ORDER });
        return;
    }
    let middle = 0.5 * (lo + hi);
    unit_panels(lo, middle, roots, depth + 1, out);
    unit_panels(middle, hi, roots, depth + 1, out);
}

/// The panels of `[lo, hi]` given the weight's roots in the SPAN's unit
/// parameter (`[span_lo, span_hi]` is the span those roots were read on).
fn panels_from_roots(roots: &[Complex], span: [f64; 2], lo: f64, hi: f64) -> Vec<Panel> {
    if roots.is_empty() || !(hi > lo) {
        return whole(lo, hi);
    }
    // The panel, in the unit parameter the roots live in.
    let width = span[1] - span[0];
    let (unit_lo, unit_hi) = ((lo - span[0]) / width, (hi - span[0]) / width);
    let mut unit = Vec::new();
    unit_panels(unit_lo, unit_hi, roots, 0, &mut unit);
    if unit.len() == 1 && unit[0].order == GAUSS_COUNT {
        // The base rule over the whole panel: keep the caller's own endpoints
        // so the reading is bit-identical to the one-block rule.
        return whole(lo, hi);
    }
    unit.into_iter()
        .map(|panel| Panel {
            lo: span[0] + width * panel.lo,
            hi: span[0] + width * panel.hi,
            order: panel.order,
        })
        .collect()
}

// --- the two carriers -------------------------------------------------------

/// Whether `[lo, hi]` lies inside the knot vector's domain.
///
/// A span outside it keeps the base rule: the evaluator CLAMPS such a
/// parameter, so the weight read there is the domain edge's, not the caller's.
/// A cell whose CROSS line sits outside (an antiderivative carried past the
/// domain by the periodic extension) is instead keyed to the nearest in-domain
/// span by [`SurfaceRules::span_of`] and reads that span's weights — on every
/// periodic patch the kernel builds, the wrapped span carries the same weights
/// as the one it wraps onto, and where it does not, what is approximated is
/// which ORDER to use, never the geometry.
fn inside_domain(knots: &[f64], degree: usize, lo: f64, hi: f64) -> bool {
    let [start, end] = crate::curve::knot_domain(knots, degree);
    let slack = 1e-12 * (end - start).abs();
    lo >= start - slack && hi <= end + slack
}

/// The control points a span of a knot vector reads, as an index range.
fn window(knots: &[f64], degree: usize, count: usize, parameter: f64) -> std::ops::RangeInclusive<usize> {
    let span = knot_find_span(knots, degree, parameter);
    let first = span.saturating_sub(degree);
    first..=span.min(count.saturating_sub(1))
}

/// The panels of one knot span `[lo, hi]` of `curve`, from its own weights.
pub(super) fn curve_panels(curve: &NurbsCurve, lo: f64, hi: f64) -> Result<Vec<Panel>, String> {
    if !rational_rule_on()
        || !(hi > lo)
        || curve.degree == 0
        || !inside_domain(&curve.knots, curve.degree, lo, hi)
    {
        return Ok(whole(lo, hi));
    }
    let range = window(&curve.knots, curve.degree, curve.control_points.len(), 0.5 * (lo + hi));
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for index in range {
        low = low.min(curve.control_points[index].w);
        high = high.max(curve.control_points[index].w);
    }
    if weights_are_far_from_a_root(low, high, curve.degree) {
        return Ok(whole(lo, hi));
    }
    super::mass_profile(|p| p.rule_sampled += 1);
    let mut values = Vec::with_capacity(curve.degree + 1);
    for j in 0..=curve.degree {
        let t = lo + (hi - lo) * j as f64 / curve.degree as f64;
        values.push(curve.evaluate_homogeneous(t)?.w);
    }
    Ok(panels_from_roots(&roots(power_basis(&values)), [lo, hi], lo, hi))
}

/// The weight roots of one knot span of `surface`'s `u` (or `v`) direction,
/// for a cell whose OTHER parameter runs over `cross`, in that span's unit
/// parameter — empty when nothing can lie inside the base order's ellipse.
///
/// The weight of a tensor patch restricted to a line of constant cross
/// parameter is a polynomial of that direction's degree, and its roots are
/// what the rule has to outrun. Where the weight net is separable
/// (`w[i][j] = a[i]·b[j]` — every revolution the kernel builds, and every
/// Möbius re-weighting of one) those roots do not depend on the cross
/// parameter and the cell's middle line settles it; where it is not, the
/// cross ends are read too and every line's roots are kept, so the nearest of
/// them decides each panel.
fn direction_roots(
    surface: &NurbsSurface,
    in_u: bool,
    span: [f64; 2],
    cross: [f64; 2],
) -> Result<Vec<Complex>, String> {
    let degree = if in_u { surface.degree_u } else { surface.degree_v };
    let (knots, cross_knots, cross_degree) = if in_u {
        (&surface.knots_u, &surface.knots_v, surface.degree_v)
    } else {
        (&surface.knots_v, &surface.knots_u, surface.degree_u)
    };
    let [lo, hi] = span;
    let cross_middle = 0.5 * (cross[0] + cross[1]);
    if !(hi > lo)
        || degree == 0
        || !inside_domain(knots, degree, lo, hi)
        || !inside_domain(cross_knots, cross_degree, cross_middle, cross_middle)
    {
        return Ok(Vec::new());
    }
    let rows = window(
        &surface.knots_u,
        surface.degree_u,
        surface.control_points.len(),
        if in_u { 0.5 * (lo + hi) } else { cross_middle },
    );
    let columns = window(
        &surface.knots_v,
        surface.degree_v,
        surface.control_points.first().map_or(0, Vec::len),
        if in_u { cross_middle } else { 0.5 * (lo + hi) },
    );
    let corner = surface.control_points[*rows.start()][*columns.start()].w;
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut separable = true;
    for row in rows.clone() {
        for column in columns.clone() {
            let weight = surface.control_points[row][column].w;
            low = low.min(weight);
            high = high.max(weight);
            // Rank one: w[i][j]·w[0][0] == w[i][0]·w[0][j].
            let product = weight * corner;
            let cross_product = surface.control_points[row][*columns.start()].w
                * surface.control_points[*rows.start()][column].w;
            separable &= (product - cross_product).abs() <= 1e-12 * product.abs().max(cross_product.abs());
        }
    }
    if weights_are_far_from_a_root(low, high, degree) {
        return Ok(Vec::new());
    }
    let lines: [f64; 3] = [cross[0], cross_middle, cross[1]];
    let lines = if separable { &lines[1..2] } else { &lines[..] };
    let mut found = Vec::new();
    for &line in lines {
        super::mass_profile(|p| p.rule_sampled += 1);
        let mut values = Vec::with_capacity(degree + 1);
        for j in 0..=degree {
            let parameter = lo + (hi - lo) * j as f64 / degree as f64;
            let (u, v) = if in_u { (parameter, line) } else { (line, parameter) };
            values.push(surface.evaluate_homogeneous(u, v)?.w);
        }
        found.extend(roots(power_basis(&values)));
    }
    Ok(found)
}

/// The panels of one knot span `[lo, hi]` of `surface`'s `u` (or `v`)
/// direction, for a cell whose other parameter runs over `cross`. The cached
/// form ([`SurfaceRules`]) is what the integrators use; this reads the weights
/// afresh and is the reference the tests drive.
pub(super) fn surface_panels(
    surface: &NurbsSurface,
    in_u: bool,
    lo: f64,
    hi: f64,
    cross: [f64; 2],
) -> Result<Vec<Panel>, String> {
    if !rational_rule_on() {
        return Ok(whole(lo, hi));
    }
    let started = super::profile::profile_started();
    let roots = direction_roots(surface, in_u, [lo, hi], cross)?;
    let panels = panels_from_roots(&roots, [lo, hi], lo, hi);
    super::mass_profile(|p| {
        p.rule_spans += 1;
        p.rule_ms += super::profile::elapsed_ms(started);
    });
    Ok(panels)
}

/// A patch's weight roots, read ONCE per knot cell and per direction and
/// reused by every cell that lands in it.
///
/// The quadtree asks the same knot cell for panels at four depths, and an
/// untrimmed sweep asks every cell of a span row in turn; reading the weights
/// again each time cost 255 ms of a 400 ms integration on an 87-face import
/// (measured 2026-09-15, `rule_ms` in `BREP_PROFILE=1`). The roots are a
/// property of the knot cell, so they are cached against it — the panels
/// themselves still come from the caller's own sub-range.
pub(super) struct SurfaceRules {
    u_breaks: Vec<f64>,
    v_breaks: Vec<f64>,
    /// `None` until the cell has been read; `(u roots, v roots)` after.
    cells: Vec<Option<(Vec<Complex>, Vec<Complex>)>>,
}

impl SurfaceRules {
    pub(super) fn new(surface: &NurbsSurface) -> Result<Self, String> {
        let (u_breaks, v_breaks) = super::surface_breaks(surface)?;
        let cells = vec![None; u_breaks.len().saturating_sub(1) * v_breaks.len().saturating_sub(1)];
        Ok(Self { u_breaks, v_breaks, cells })
    }

    /// Which knot span of `breaks` holds `parameter` (the last one when it
    /// sits at or past the end).
    fn span_of(breaks: &[f64], parameter: f64) -> usize {
        match breaks.binary_search_by(|value| value.total_cmp(&parameter)) {
            Ok(index) => index.min(breaks.len().saturating_sub(2)),
            Err(index) => index.saturating_sub(1).min(breaks.len().saturating_sub(2)),
        }
    }

    /// The u and v panels of `cell`, from the knot cell's cached roots.
    pub(super) fn panels(
        &mut self,
        surface: &NurbsSurface,
        cell: [f64; 4],
    ) -> Result<(Vec<Panel>, Vec<Panel>), String> {
        let [u_lo, u_hi, v_lo, v_hi] = cell;
        if !rational_rule_on() || self.cells.is_empty() {
            return Ok((whole(u_lo, u_hi), whole(v_lo, v_hi)));
        }
        let started = super::profile::profile_started();
        let u_span = Self::span_of(&self.u_breaks, 0.5 * (u_lo + u_hi));
        let v_span = Self::span_of(&self.v_breaks, 0.5 * (v_lo + v_hi));
        let u_range = [self.u_breaks[u_span], self.u_breaks[u_span + 1]];
        let v_range = [self.v_breaks[v_span], self.v_breaks[v_span + 1]];
        let index = u_span * (self.v_breaks.len() - 1) + v_span;
        if self.cells[index].is_none() {
            let u_roots = direction_roots(surface, true, u_range, v_range)?;
            let v_roots = direction_roots(surface, false, v_range, u_range)?;
            self.cells[index] = Some((u_roots, v_roots));
        }
        let (u_roots, v_roots) = self.cells[index].as_ref().expect("just filled");
        let panels = (
            panels_from_roots(u_roots, u_range, u_lo, u_hi),
            panels_from_roots(v_roots, v_range, v_lo, v_hi),
        );
        super::mass_profile(|p| {
            p.rule_spans += 1;
            p.rule_ms += super::profile::elapsed_ms(started);
        });
        Ok(panels)
    }

    /// One direction's panels at a fixed cross line — the antiderivative's
    /// question, answered from the same cache.
    pub(super) fn direction_panels(
        &mut self,
        surface: &NurbsSurface,
        in_u: bool,
        lo: f64,
        hi: f64,
        cross: f64,
    ) -> Result<Vec<Panel>, String> {
        let cell = if in_u { [lo, hi, cross, cross] } else { [cross, cross, lo, hi] };
        let (u_panels, v_panels) = self.panels(surface, cell)?;
        Ok(if in_u { u_panels } else { v_panels })
    }
}
