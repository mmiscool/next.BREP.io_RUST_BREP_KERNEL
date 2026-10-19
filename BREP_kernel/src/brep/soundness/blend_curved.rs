//! Curved-neighbour blend patches (lane K2, `BREP_SOUNDNESS_BLEND_CURVED`, on
//! by default, `=0` to disable): the blend-band body certificate (`blend_patch.rs`) extended to
//! patches RATIONAL IN BOTH DIRECTIONS, and to a patch that meets a native
//! CYLINDER or a face on a recognised TORUS. That is the shape a fillet leaves
//! on a cylinder rim or a torus, and the torus faces themselves; nothing here
//! reads who built the body. Anything this module cannot prove declines to
//! the full scan.
use super::analytic_body::Cylinder;
use super::blend_patch::{breaks, local, segment, Net};
use super::tube_grid::{bezier_spans, injective, Cell, MAX_CELLS};
use super::*;

pub(super) type Decline = &'static str;

pub(super) fn enabled() -> bool {
    // On by default (promoted 2026-10-16); `BREP_SOUNDNESS_BLEND_CURVED=0`
    // (any value but 1/on) restores the blend row without curved neighbours.
    match std::env::var("BREP_SOUNDNESS_BLEND_CURVED") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    }
}

// What is proved, on top of blend_patch.rs and analytic_body.rs.
//
// - Cells. A patch rational in both directions is cut into Bezier spans both
//   ways in homogeneous coordinates (knot insertion is linear, so the weighted
//   points and the weights take the same coefficients), restricted to its
//   trims' chart box and split `SPLIT` times per span each way. Positive
//   weights: each cell's Euclidean control points hold it.
// - Tangent cones. With S = N / W, S_u = (N_u W - N W_u) / W^2. The numerator
//   is a polynomial of degree (2p_u - 1, 2p_v) whose Bernstein coefficients
//   are computed exactly from the cell's net (products of Bernstein
//   polynomials), and W^2 > 0: S_u lies in the cone of those coefficients.
//   The same for v. The block and separation proof is tube_grid's, unchanged
//   (`injective`); a patch closed in one direction (its box spans the whole
//   knot range and its first and last rows agree to `DELTA`) is glued into a
//   cyclic grid, as tube_grid glues a ring.
// - One side of a curved carrier. A cylinder (axis a through b, radius r) is
//   g = |x - b|^2 - ((x - b).a)^2 - r^2 = (rho - r)(rho + r); a ring torus
//   (centre c, axis a, radii R > r) is g = (|x - c|^2 + R^2 - r^2)^2 -
//   4 R^2 rho^2 = (D - r)(D + r)((rho + R)^2 + z^2 - r^2), D the distance to
//   the tube's core circle. Composed with a cell, G = g W^k (k = 2, 4) is a
//   polynomial whose Bernstein coefficients bound it; so g >= Gmin / Wmin^k
//   when Gmin < 0, and likewise above. The distance d to the surface has the
//   sign of g and |g| >= |d| times a factor bounded below: r (inside) and 2r
//   (outside) for the cylinder; r 4R(R - r) inside the tube (rho >= R - r)
//   and 2r (R^2 - r^2) outside for the torus. Rounding is padded by
//   `ROUNDING` of the same polynomial over absolute values.
//   A patch whose every cell is within `tau` on ONE side of the carrier a face
//   it shares an edge with lies on, is covered by the plane rule's argument
//   (blend_patch.rs): every confirmation the scan has between the two faces
//   needs points of the patch's own trimmed surface past the band on BOTH
//   sides of that carrier. A cell that cannot show it is halved both ways up
//   to `MAX_REFINE` times.
// - Faces that share no edge, when the faces' boxes overlap: every cell is
//   box-separated from the other face (or from each of its cells), or, against
//   a plane, more than `tau` on one side of it.
//
// The torus is recognised as the surface `make_torus_surface` rebuilds from
// the net's own corners, to `DELTA`, as analytic_body recognises a cylinder:
// a full turn both ways, so its chart is a full torus with no edge.

/// Recognition slack, relative to the carrier's size (analytic_body's).
const DELTA: f64 = 1e-12;
/// At most this many pieces per Bezier span each way, tried from one up: a
/// quarter circle in both directions (a torus) needs three for its 2x2
/// blocks' cones to stay apart; a fillet's thin spans prove at one.
const SPLIT: usize = 3;
/// How many times a cell is halved before a one-sided test declines.
const MAX_REFINE: usize = 4;
/// Refined cells a one-sided test may build, per cell of the patch (work,
/// never time).
const REFINE_PER_CELL: usize = 4;
/// `tube_grid::injective`'s cone decline, the one a finer split can cure.
const CONES: Decline = "a block's tangent cones do not prove it injective";
/// Relative rounding pad on a composed polynomial's coefficients.
const ROUNDING: f64 = 1e-13;
/// A ring torus reads as embedded only when its reach min(r, R - r) is this
/// many bands `tau` (a torus thinner than that is the cell proof's).
const TORUS_REACH_BARS: f64 = 1e3;

/// Lane R1 row 4: a face on a recognised ring torus is fold-free in closed
/// form (no cones, no separations). On by default (promoted 2026-10-18 on the
/// 98 + 144 census); `BREP_SOUNDNESS_TORUS_EMBEDDED=0` (or `off`) restores the
/// cell proof.
pub(super) fn torus_embedded_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| !matches!(std::env::var("BREP_SOUNDNESS_TORUS_EMBEDDED").as_deref(), Ok("0") | Ok("off")))
}

/// A recognised ring torus.
#[derive(Clone, Copy, Debug)]
pub(super) struct Torus {
    center: Vec3,
    axis: Vec3,
    major: f64,
    minor: f64,
}

/// A carrier with an implicit equation this module reads one side of.
#[derive(Clone, Copy, Debug)]
pub(super) enum Implicit {
    Cylinder { base: Vec3, axis: Vec3, radius: f64 },
    Torus(Torus),
}

impl Implicit {
    pub(super) fn cylinder(c: &Cylinder) -> Self {
        let (base, axis, radius) = c.frame();
        Implicit::Cylinder { base, axis, radius }
    }
}

/// The torus `make_torus_surface` builds, when `s` is it to `DELTA`.
pub(super) fn torus_of(s: &NurbsSurface) -> Option<Torus> {
    if s.degree_u != 2 || s.degree_v != 2 || s.control_points.len() != 9 || s.control_points.iter().any(|r| r.len() != 9) {
        return None;
    }
    let at = |i: usize, l: usize| s.control_points[i][l].point().ok();
    let (outer, top, inner, opposite) = (at(0, 0)?, at(0, 2)?, at(0, 4)?, at(4, 0)?);
    let tube = outer.add(inner).scale(0.5);
    let center = outer.add(opposite).scale(0.5);
    let minor = outer.sub(tube).length();
    let major = tube.sub(center).length();
    let axis = top.sub(tube).normalized().ok()?;
    if ![minor, major].iter().all(|v| v.is_finite() && *v > 0.) || !(major > minor) {
        return None;
    }
    let rebuilt = crate::make_torus_surface(center, axis, major, minor).ok()?;
    let same_bits = |a: &[f64], b: &[f64]| a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits());
    if !same_bits(&s.knots_u, &rebuilt.knots_u) || !same_bits(&s.knots_v, &rebuilt.knots_v) {
        return None;
    }
    let size = center.x.abs().max(center.y.abs()).max(center.z.abs()) + major + minor;
    let close = s.control_points.len() == rebuilt.control_points.len()
        && s.control_points.iter().zip(&rebuilt.control_points).all(|(a, b)| a.len() == b.len())
        && s.control_points.iter().flatten().zip(rebuilt.control_points.iter().flatten()).all(|(a, b)| {
            (a.x - b.x).abs() <= DELTA * size
                && (a.y - b.y).abs() <= DELTA * size
                && (a.z - b.z).abs() <= DELTA * size
                && (a.w - b.w).abs() <= DELTA
                && a.w > 0.
        });
    close.then_some(Torus { center, axis, major, minor })
}

/// The reach min(r, ρ_c − r) of a surface that is, to `DELTA`, the full-turn
/// revolution of ONE circular arc about an axis the arc's circle stays clear
/// of: a piece of a ring torus (a closed-rim fillet between coaxial
/// revolutions, `make_revolution` of the ball's section). Read from the net
/// alone, never from a tag: the axis from the u circle of the first column,
/// the generatrix from the u = 0 row, and the net rebuilt from them by
/// `make_revolution` must be the face's net; the generatrix must be a
/// rational quadratic circular arc (equal legs, middle weight the cosine of
/// its half angle) in one meridian half-plane. `None` otherwise.
fn revolved_arc_reach(s: &NurbsSurface) -> Option<f64> {
    revolved_arc(s).map(|arc| arc.reach)
}

/// A recognised full-turn revolution of one circular arc
/// ([`revolved_arc_reach`]): its axis, its reach, and the meridian range of
/// its generatrix's control hull — `ρ ∈ rho`, `z ∈ z` (z along `axis` from
/// `origin`) — which holds the arc (positive weights), so every point of the
/// carrier and of any trimmed piece of it.
#[derive(Clone, Copy, Debug)]
pub(super) struct RevolvedArc {
    origin: Vec3,
    axis: Vec3,
    reach: f64,
    rho: [f64; 2],
    z: [f64; 2],
}

fn revolved_arc(s: &NurbsSurface) -> Option<RevolvedArc> {
    if s.degree_u != 2 || s.degree_v != 2 || s.control_points.len() != 9 || s.control_points.iter().any(|row| row.len() != 3) {
        return None;
    }
    let at = |i: usize, l: usize| s.control_points[i][l].point().ok();
    let (first, quarter, opposite) = (at(0, 0)?, at(2, 0)?, at(4, 0)?);
    let origin = first.add(opposite).scale(0.5);
    let axis = first.sub(origin).cross(quarter.sub(origin)).normalized().ok()?;
    let row = &s.control_points[0];
    let generatrix = crate::NurbsCurve::new(2, s.knots_v.clone(), row.clone()).ok()?;
    let rebuilt = crate::make_revolution(origin, axis, &generatrix, std::f64::consts::TAU).ok()?;
    let same_bits = |a: &[f64], b: &[f64]| a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits());
    if !same_bits(&s.knots_u, &rebuilt.knots_u) || !same_bits(&s.knots_v, &rebuilt.knots_v) {
        return None;
    }
    let size = s.control_points.iter().flatten().filter_map(|c| c.point().ok()).fold(0f64, |m, q| m.max(q.x.abs()).max(q.y.abs()).max(q.z.abs()));
    if !(size > 0.) || rebuilt.control_points.len() != 9 || rebuilt.control_points.iter().any(|r| r.len() != 3) {
        return None;
    }
    let close = s.control_points.iter().flatten().zip(rebuilt.control_points.iter().flatten()).all(|(a, b)| {
        a.w > 0.
            && b.w > 0.
            && (a.w - b.w).abs() <= DELTA * a.w.abs().max(b.w.abs())
            && (a.x / a.w - b.x / b.w).abs() <= DELTA * size
            && (a.y / a.w - b.y / b.w).abs() <= DELTA * size
            && (a.z / a.w - b.z / b.w).abs() <= DELTA * size
    });
    if !close || s.knots_v.len() != 6 {
        return None;
    }
    // The generatrix: one rational quadratic span, a circular arc.
    let [p0, p1, p2] = [row[0].point().ok()?, row[1].point().ok()?, row[2].point().ok()?];
    let (w0, w1, w2) = (row[0].w, row[1].w, row[2].w);
    if !(w0 > 0. && w2 > 0. && w1 > 0.) {
        return None;
    }
    // Standard form: the middle weight relative to the ends' geometric mean.
    let cos_half = w1 / (w0 * w2).sqrt();
    let (leg0, leg2) = (p0.sub(p1), p2.sub(p1));
    let (l0, l2) = (leg0.length(), leg2.length());
    if !(cos_half > 0. && cos_half < 1.) || !(l0 > 0.) || (l0 - l2).abs() > DELTA * size {
        return None;
    }
    // The legs meet at angle π − 2θ, so sin(half that) = cos θ.
    let corner = (leg0.dot(leg2) / (l0 * l2)).clamp(-1., 1.).acos();
    if ((0.5 * corner).sin() - cos_half).abs() > 1e-9 {
        return None;
    }
    let chord_mid = p0.add(p2).scale(0.5);
    let sin2 = 1. - cos_half * cos_half;
    let centre = p1.add(chord_mid.sub(p1).scale(1. / sin2));
    let radius = p0.sub(centre).length();
    if (p2.sub(centre).length() - radius).abs() > DELTA * size * 1e3 {
        return None;
    }
    // One meridian half-plane: every point of the section, its centre too,
    // on the plane through the axis and the first contact.
    let radial = |q: Vec3| {
        let d = q.sub(origin);
        d.sub(axis.scale(d.dot(axis)))
    };
    let normal = axis.cross(radial(p0)).normalized().ok()?;
    if [p0, p1, p2, centre].iter().any(|q| q.sub(origin).dot(normal).abs() > DELTA * size * 1e3) {
        return None;
    }
    let reach = radius.min(radial(centre).length() - radius);
    // The hull's meridian range: ρ is linear on the half-plane (every point
    // on the side of `p0`), z along the axis.
    let x_axis = radial(p0).normalized().ok()?;
    let mut rho = [f64::INFINITY, f64::NEG_INFINITY];
    let mut z = [f64::INFINITY, f64::NEG_INFINITY];
    for q in [p0, p1, p2] {
        let d = q.sub(origin);
        let x = d.dot(x_axis);
        if !(x >= 0.) {
            return None;
        }
        rho = [rho[0].min(x), rho[1].max(x)];
        z = [z[0].min(d.dot(axis)), z[1].max(d.dot(axis))];
    }
    (reach.is_finite() && reach > 0.).then_some(RevolvedArc { origin, axis, reach, rho, z })
}

/// Is the face's patch within `tau` on one side of `implicit`? For a face
/// that is a recognised full-turn revolved arc ([`revolved_arc`]) against a
/// cylinder coaxial with it (axes parallel to 1e-12, the cylinder's base off
/// the band's axis by `offset`), decided from the generatrix's exact ρ range:
/// every point's distance to the cylinder is its ρ − R to within `offset`,
/// so `ρmax − R + offset ≤ tau` (inside) or `ρmin − R − offset ≥ −tau`
/// (outside) proves the side. Anything else — a partial band, a tilted or
/// shifted cylinder, a torus — reads the cells ([`one_sided`]) as before
/// (lane R1, `BREP_SOUNDNESS_ONE_SIDED_REVOLUTION`).
pub(super) fn one_sided_patch(record: &FaceRecord, nets: &[Net], implicit: &Implicit, tau: f64) -> bool {
    if one_sided_revolution_on() {
        if let (Some(arc), Implicit::Cylinder { base, axis, radius }) = (revolved_arc(&record.surface), implicit) {
            if axis.cross(arc.axis).length() <= DELTA {
                let d = base.sub(arc.origin);
                let offset = d.sub(arc.axis.scale(d.dot(arc.axis))).length();
                if arc.rho[1] - radius + offset <= tau || arc.rho[0] - radius - offset >= -tau {
                    crate::blend::record_count("count.soundness.one_sided_closed_form", 1);
                    return true;
                }
            }
        }
        crate::blend::record_count("count.soundness.one_sided_cells", 1);
    }
    one_sided(nets, implicit, tau)
}

/// Lane R1 (grant b): [`one_sided_patch`]'s closed form. On by default
/// (promoted 2026-10-18 on the 98 + 144 census);
/// `BREP_SOUNDNESS_ONE_SIDED_REVOLUTION=0` (or `off`) reads the cells.
pub(super) fn one_sided_revolution_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| !matches!(std::env::var("BREP_SOUNDNESS_ONE_SIDED_REVOLUTION").as_deref(), Ok("0") | Ok("off")))
}

/// Cheap shape checks, no cells: a face this module could read as a patch.
pub(super) fn screen(record: &FaceRecord) -> Result<(), Decline> {
    let s = &record.surface;
    let net = &s.control_points;
    if s.degree_u == 0 || s.degree_v == 0 || net.len() < 2 || net[0].len() < 2 || net.iter().any(|row| row.len() != net[0].len()) {
        return Err("a patch net is not a grid");
    }
    if net.iter().flatten().any(|c| !(c.w > 0.) || !c.w.is_finite() || !c.x.is_finite() || !c.y.is_finite() || !c.z.is_finite()) {
        return Err("a patch control point is not finite with a positive weight");
    }
    if 4 * s.degree_u > MAX_DEGREE || 4 * s.degree_v > MAX_DEGREE {
        return Err("a patch degree above the proof's table");
    }
    if s.knots_u.len() != net.len() + s.degree_u + 1 || s.knots_v.len() != net[0].len() + s.degree_v + 1 {
        return Err("a patch's knots do not fit its net");
    }
    if record.loops.is_empty() || record.loops.iter().flat_map(|l| &l.coedges).next().is_none() {
        return Err("a patch has no trim");
    }
    Ok(())
}

/// Clamped homogeneous B-spline -> its Bezier spans, by `bezier_spans` on the
/// weighted points and on the weights (the insertion is linear).
fn spans4(knots: &[f64], points: &[[f64; 4]], degree: usize) -> Option<Vec<Vec<[f64; 4]>>> {
    let weighted: Vec<Vec3> = points.iter().map(|h| Vec3::new(h[0], h[1], h[2])).collect();
    let weights: Vec<Vec3> = points.iter().map(|h| Vec3::new(h[3], 0., 0.)).collect();
    let a = bezier_spans(knots, &weighted, degree)?;
    let b = bezier_spans(knots, &weights, degree)?;
    (a.len() == b.len()).then(|| {
        a.iter().zip(&b).map(|(p, w)| p.iter().zip(w).map(|(p, w)| [p.x, p.y, p.z, w.x]).collect()).collect()
    })
}

/// Binomials up to `MAX_DEGREE`, built once.
const MAX_DEGREE: usize = 48;

fn binomial(n: usize, k: usize) -> f64 {
    static TABLE: std::sync::OnceLock<Vec<Vec<f64>>> = std::sync::OnceLock::new();
    let table = TABLE.get_or_init(|| {
        (0..=MAX_DEGREE).map(|n| (0..=n).map(|k| (1..=k.min(n - k)).fold(1., |v, i| v * (n - k.min(n - k) + i) as f64 / i as f64)).collect()).collect()
    });
    table[n][k]
}

/// A tensor Bernstein polynomial of degree (n, m), coefficients row-major.
#[derive(Clone)]
struct Poly {
    n: usize,
    m: usize,
    c: Vec<f64>,
}

impl Poly {
    fn of(net: &Net, f: impl Fn(&[f64; 4]) -> f64) -> Self {
        Poly { n: net.len() - 1, m: net[0].len() - 1, c: net.iter().flatten().map(f).collect() }
    }

    fn mul(&self, o: &Poly) -> Poly {
        let (n, m) = (self.n + o.n, self.m + o.m);
        assert!(n <= MAX_DEGREE && m <= MAX_DEGREE, "screened: degree within the binomial table");
        let mut c = vec![0.; (n + 1) * (m + 1)];
        for i in 0..=self.n {
            for l in 0..=self.m {
                let a = self.c[i * (self.m + 1) + l];
                if a == 0. {
                    continue;
                }
                for j in 0..=o.n {
                    let wu = binomial(self.n, i) * binomial(o.n, j) / binomial(n, i + j);
                    for k in 0..=o.m {
                        let wv = binomial(self.m, l) * binomial(o.m, k) / binomial(m, l + k);
                        c[(i + j) * (m + 1) + l + k] += wu * wv * a * o.c[j * (o.m + 1) + k];
                    }
                }
            }
        }
        Poly { n, m, c }
    }

    fn combine(&self, a: f64, o: &Poly, b: f64) -> Poly {
        Poly { n: self.n, m: self.m, c: self.c.iter().zip(&o.c).map(|(x, y)| a * x + b * y).collect() }
    }

    fn abs(&self) -> Poly {
        Poly { n: self.n, m: self.m, c: self.c.iter().map(|x| x.abs()).collect() }
    }

    fn range(&self) -> (f64, f64) {
        self.c.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(*x), hi.max(*x)))
    }
}

/// The u (rows) partial's numerator N_u W - N W_u, as direction vectors.
fn hodograph(net: &Net) -> Vec<Vec3> {
    let n = net.len() - 1;
    let diff: Net = (0..n).map(|i| (0..net[0].len()).map(|l| std::array::from_fn(|c| n as f64 * (net[i + 1][l][c] - net[i][l][c]))).collect()).collect();
    let w = Poly::of(net, |h| h[3]);
    let wu = Poly::of(&diff, |h| h[3]);
    let comp = |c: usize| Poly::of(&diff, |h| h[c]).mul(&w).combine(1., &Poly::of(net, |h| h[c]).mul(&wu), -1.);
    let (x, y, z) = (comp(0), comp(1), comp(2));
    x.c.iter().zip(&y.c).zip(&z.c).map(|((x, y), z)| Vec3::new(*x, *y, *z)).filter(|d| d.length_squared() > 0.).collect()
}

fn transpose(net: &Net) -> Net {
    (0..net[0].len()).map(|l| net.iter().map(|row| row[l]).collect()).collect()
}

fn euclid(net: &Net) -> Vec<Vec<Vec3>> {
    net.iter().map(|row| row.iter().map(|h| Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3])).collect()).collect()
}

/// A cell of the proof from its homogeneous net, its cones read from the
/// hodographs; `swap` lays it out transposed (a grid cyclic in v).
fn cell(net: &Net, swap: bool) -> Option<Cell> {
    let (u, v) = (hodograph(net), hodograph(&transpose(net)));
    if swap {
        Cell::with_cones(euclid(&transpose(net)), v, u)
    } else {
        Cell::with_cones(euclid(net), u, v)
    }
}

/// Split a homogeneous net at `t` along its rows (u).
fn halve_u(net: &Net, t: f64) -> (Net, Net) {
    let columns: Vec<(Vec<[f64; 4]>, Vec<[f64; 4]>)> = (0..net[0].len())
        .map(|l| {
            let column: Vec<[f64; 4]> = net.iter().map(|row| row[l]).collect();
            (segment(&column, 0., t), segment(&column, t, 1.))
        })
        .collect();
    let rebuild = |pick: &dyn Fn(&(Vec<[f64; 4]>, Vec<[f64; 4]>)) -> Vec<[f64; 4]>| -> Net {
        let parts: Vec<Vec<[f64; 4]>> = columns.iter().map(pick).collect();
        (0..net.len()).map(|k| parts.iter().map(|p| p[k]).collect()).collect()
    };
    (rebuild(&|c| c.0.clone()), rebuild(&|c| c.1.clone()))
}

fn pieces(net: &Net, parts: usize) -> Vec<Net> {
    let mut out = Vec::with_capacity(parts);
    let mut rest = net.clone();
    for remaining in (2..=parts).rev() {
        let (left, right) = halve_u(&rest, 1. / remaining as f64);
        out.push(left);
        rest = right;
    }
    out.push(rest);
    out
}

/// A curved patch: its restricted cells' homogeneous nets, proved fold-free.
pub(super) fn patch(record: &FaceRecord, tau: f64) -> Result<Vec<Net>, Decline> {
    let _timing = crate::blend::BlendTimer::start("timing.soundness.curved_patch_ms");
    screen(record)?;
    let s = &record.surface;
    let pv = s.degree_v;
    let net: Net = s.control_points.iter().map(|row| row.iter().map(|c| [c.x, c.y, c.z, c.w]).collect()).collect();
    let mut bounds = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
    for c in record.loops.iter().flat_map(|l| &l.coedges).flat_map(|ce| &ce.pcurve.control_points) {
        if !(c.w > 0.) {
            return Err("a patch trim weight is not positive");
        }
        let (u, v) = (c.x / c.w, c.y / c.w);
        if !u.is_finite() || !v.is_finite() {
            return Err("a patch trim is not finite");
        }
        bounds = [bounds[0].min(u), bounds[1].max(u), bounds[2].min(v), bounds[3].max(v)];
    }
    let (bu, bv) = (breaks(&s.knots_u), breaks(&s.knots_v));
    // A trim bound within `DELTA` of the domain of a knot is that knot: the
    // sliver between is under `DELTA` of the patch, far under `tau`.
    let snap = |x: f64, knots: &[f64]| {
        let slack = DELTA * (knots[knots.len() - 1] - knots[0]);
        knots.iter().copied().find(|k| (x - k).abs() <= slack).unwrap_or(x)
    };
    let [u0, u1, v0, v1] = [
        snap(bounds[0].max(bu[0]), &bu),
        snap(bounds[1].min(bu[bu.len() - 1]), &bu),
        snap(bounds[2].max(bv[0]), &bv),
        snap(bounds[3].min(bv[bv.len() - 1]), &bv),
    ];
    if !(u1 > u0 && v1 > v0) {
        return Err("a patch trim box is empty");
    }
    // v spans per row, then u spans per column of each v span.
    let rows: Vec<Vec<Vec<[f64; 4]>>> =
        net.iter().map(|row| spans4(&s.knots_v, row, pv)).collect::<Option<_>>().ok_or("a patch's knots are not a clamped spline")?;
    if rows.iter().any(|r| r.len() != bv.len() - 1) {
        return Err("a patch's spans do not match its knots");
    }
    // Closed directions: the box spans the whole range and the net's first
    // and last rows (columns) agree.
    let size = net.iter().flatten().fold(0f64, |m, h| m.max((h[0] / h[3]).abs()).max((h[1] / h[3]).abs()).max((h[2] / h[3]).abs())).max(1e-300);
    let agree = |a: &[f64; 4], b: &[f64; 4]| {
        (0..3).all(|c| (a[c] / a[3] - b[c] / b[3]).abs() <= DELTA * size) && (a[3] - b[3]).abs() <= DELTA * a[3].abs().max(b[3].abs())
    };
    let closed_u = u0 == bu[0] && u1 == bu[bu.len() - 1] && net[0].iter().zip(&net[net.len() - 1]).all(|(a, b)| agree(a, b));
    let closed_v = v0 == bv[0] && v1 == bv[bv.len() - 1] && net.iter().all(|row| agree(&row[0], &row[row.len() - 1]));
    if closed_u && closed_v {
        return Err("a patch closed in both directions");
    }
    // A face on a recognised RING torus is a piece of an embedded surface
    // whose reach is min(r, R - r): it cannot fold, and two of its points
    // closer than twice the reach are joined by a short path on the surface,
    // so no part of it comes back near another — what the cones and the
    // separations below prove cell by cell is known in closed form (lane R1
    // row 4, `BREP_SOUNDNESS_TORUS_EMBEDDED`). Its cells are the ones the
    // proof would have accepted (`SPLIT` pieces per span), so every check a
    // neighbour takes against them reads the same nets.
    if torus_embedded_on() {
        let reach = torus_of(s).map(|torus| torus.minor.min(torus.major - torus.minor)).or_else(|| revolved_arc_reach(s));
        if reach.is_some_and(|reach| reach > TORUS_REACH_BARS * tau) {
            crate::blend::record_count("count.soundness.torus_embedded", 1);
            return grid_at(&rows, s, &bu, &bv, [u0, u1, v0, v1], SPLIT, closed_u, closed_v, tau, false);
        }
    }
    // The coarsest split whose blocks prove it, up to `SPLIT`.
    let mut last = Err("a patch cell grid outside the proof's size");
    // Only a block's cones get better with a finer split; a separation that
    // fails is the patch's own shape, and declines at once.
    for split in 1..=SPLIT {
        crate::blend::record_count("count.soundness.curved_grid_attempts", 1);
        match grid_at(&rows, s, &bu, &bv, [u0, u1, v0, v1], split, closed_u, closed_v, tau, true) {
            Ok(nets) => return Ok(nets),
            Err(reason @ CONES) => last = Err(reason),
            Err(reason) => return Err(reason),
        }
    }
    last
}

/// The restricted cells at `split` pieces per span each way, proved fold-free
/// (`prove`), or taken as fold-free by the caller's closed form.
#[allow(clippy::too_many_arguments)]
fn grid_at(
    rows: &[Vec<Vec<[f64; 4]>>],
    s: &NurbsSurface,
    bu: &[f64],
    bv: &[f64],
    [u0, u1, v0, v1]: [f64; 4],
    split: usize,
    closed_u: bool,
    closed_v: bool,
    tau: f64,
    prove: bool,
) -> Result<Vec<Net>, Decline> {
    let (pu, pv) = (s.degree_u, s.degree_v);
    // grid[u piece][v piece]
    let mut grid: Vec<Vec<Net>> = Vec::new();
    let mut nv_total = 0;
    for j in 0..bv.len() - 1 {
        let Some((r0, r1)) = local(bv[j], bv[j + 1], v0, v1) else { continue };
        let restricted: Vec<Vec<[f64; 4]>> = rows.iter().map(|r| segment(&r[j], r0, r1)).collect();
        let columns: Vec<Vec<[f64; 4]>> = (0..=pv).map(|l| restricted.iter().map(|r| r[l]).collect()).collect();
        let column_spans: Vec<Vec<Vec<[f64; 4]>>> =
            columns.iter().map(|c| spans4(&s.knots_u, c, pu)).collect::<Option<_>>().ok_or("a patch's knots are not a clamped spline")?;
        if column_spans.iter().any(|c| c.len() != bu.len() - 1) {
            return Err("a patch's spans do not match its knots");
        }
        let mut a = 0;
        for i in 0..bu.len() - 1 {
            let Some((t0, t1)) = local(bu[i], bu[i + 1], u0, u1) else { continue };
            let span_columns: Vec<Vec<[f64; 4]>> = column_spans.iter().map(|c| segment(&c[i], t0, t1)).collect();
            let span: Net = (0..=pu).map(|k| span_columns.iter().map(|c| c[k]).collect()).collect();
            for piece_u in pieces(&span, split) {
                if grid.len() <= a {
                    grid.push(Vec::new());
                }
                for piece in pieces(&transpose(&piece_u), split) {
                    grid[a].push(transpose(&piece));
                }
                a += 1;
            }
        }
        nv_total += split;
    }
    let nu = grid.len();
    if nu == 0 || grid.iter().any(|column| column.len() != nv_total) || nu * nv_total > MAX_CELLS {
        return Err("a patch cell grid outside the proof's size");
    }
    if !prove {
        return Ok(grid.into_iter().flatten().collect());
    }
    let cells: Vec<Vec<Cell>> = if closed_v {
        (0..nv_total).map(|j| (0..nu).map(|a| cell(&grid[a][j], true)).collect::<Option<_>>()).collect::<Option<_>>()
    } else {
        grid.iter().map(|column| column.iter().map(|n| cell(n, false)).collect::<Option<_>>()).collect::<Option<_>>()
    }
    .ok_or("a degenerate patch cell")?;
    let _injective = crate::blend::BlendTimer::start("timing.soundness.curved_injective_ms");
    crate::blend::record_count("count.soundness.curved_cells", (nu * nv_total) as u64);
    injective(&cells, closed_u || closed_v, tau)?;
    Ok(grid.into_iter().flatten().collect())
}

/// Bounds on the signed distance to `implicit` over one cell, or `None` when
/// the composition is not finite.
fn distance_bounds(net: &Net, implicit: &Implicit) -> Option<(f64, f64)> {
    crate::blend::record_count("count.soundness.distance_bounds", 1);
    let w = Poly::of(net, |h| h[3]);
    let wmin = w.range().0;
    if !(wmin > 0.) {
        return None;
    }
    let (origin, axis) = match implicit {
        Implicit::Cylinder { base, axis, .. } => (*base, *axis),
        Implicit::Torus(t) => (t.center, t.axis),
    };
    // Translated weighted coordinates: P = N - origin W.
    let p: [Poly; 3] = std::array::from_fn(|c| {
        let o = [origin.x, origin.y, origin.z][c];
        Poly::of(net, |h| h[c] - o * h[3])
    });
    let pa: [Poly; 3] = std::array::from_fn(|c| p[c].abs());
    let q = p[0].mul(&p[0]).combine(1., &p[1].mul(&p[1]), 1.).combine(1., &p[2].mul(&p[2]), 1.);
    let qa = pa[0].mul(&pa[0]).combine(1., &pa[1].mul(&pa[1]), 1.).combine(1., &pa[2].mul(&pa[2]), 1.);
    let z = p[0].combine(axis.x, &p[1], axis.y).combine(1., &p[2], axis.z);
    let za = pa[0].combine(axis.x.abs(), &pa[1], axis.y.abs()).combine(1., &pa[2], axis.z.abs());
    let w2 = w.mul(&w);
    let rho2 = q.combine(1., &z.mul(&z), -1.);
    let rho2a = qa.combine(1., &za.mul(&za), 1.);
    let (g, ga, inside, outside, power) = match implicit {
        Implicit::Cylinder { radius, .. } => {
            let r2 = radius * radius;
            (rho2.combine(1., &w2, -r2), rho2a.combine(1., &w2, r2), *radius, 2. * radius, 2)
        }
        Implicit::Torus(t) => {
            let (big, small) = (t.major, t.minor);
            let k = big * big - small * small;
            let a = q.combine(1., &w2, k);
            let aa = qa.combine(1., &w2, k.abs());
            let rw = rho2.mul(&w2);
            let rwa = rho2a.mul(&w2);
            let g = a.mul(&a).combine(1., &rw, -4. * big * big);
            let ga = aa.mul(&aa).combine(1., &rwa, 4. * big * big);
            (g, ga, small * 4. * big * (big - small), 2. * small * k, 4)
        }
    };
    let pad = ROUNDING * ga.range().1;
    let (lo, hi) = g.range();
    let (lo, hi) = (lo - pad, hi + pad);
    let wk = wmin.powi(power);
    let below = if lo < 0. { lo / (wk * inside) } else { 0. };
    let above = if hi > 0. { hi / (wk * outside) } else { 0. };
    (below.is_finite() && above.is_finite()).then_some((below, above))
}

/// The exact distance from `q` to the carrier.
fn distance(implicit: &Implicit, q: Vec3) -> f64 {
    match implicit {
        Implicit::Cylinder { base, axis, radius } => {
            let d = q.sub(*base);
            let z = d.dot(*axis);
            (d.length_squared() - z * z).max(0.).sqrt() - radius
        }
        Implicit::Torus(t) => {
            let d = q.sub(t.center);
            let z = d.dot(t.axis);
            let rho = (d.length_squared() - z * z).max(0.).sqrt();
            (rho - t.major).hypot(z) - t.minor
        }
    }
}

/// The four corner points of a cell: points of the patch itself.
fn corners(net: &Net) -> [Vec3; 4] {
    let (n, m) = (net.len() - 1, net[0].len() - 1);
    [net[0][0], net[0][m], net[n][0], net[n][m]].map(|h| Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]))
}

/// Is the patch (its cells' nets) within `tau` on one side of `implicit`?
pub(super) fn one_sided(nets: &[Net], implicit: &Implicit, tau: f64) -> bool {
    let _timing = crate::blend::BlendTimer::start(match implicit {
        Implicit::Cylinder { .. } => "timing.soundness.one_sided_cylinder_ms",
        Implicit::Torus(_) => "timing.soundness.one_sided_torus_ms",
    });
    // A side one exact corner point is more than `tau` past is ruled out
    // before any bound is composed.
    let reach = |above: bool| move |net: &Net| corners(net).iter().all(|q| if above { distance(implicit, *q) <= tau } else { distance(implicit, *q) >= -tau });
    // side: above = at most tau above (d <= tau), else at most tau below.
    fn holds(net: &Net, implicit: &Implicit, tau: f64, above: bool, depth: usize, budget: &mut usize) -> bool {
        let Some((lo, hi)) = distance_bounds(net, implicit) else { return false };
        if if above { hi <= tau } else { lo >= -tau } {
            return true;
        }
        if depth == 0 || *budget < 4 {
            return false;
        }
        *budget -= 4;
        let (a, b) = halve_u(net, 0.5);
        [a, b].iter().all(|half| {
            let (c, d) = halve_u(&transpose(half), 0.5);
            [transpose(&c), transpose(&d)].iter().all(|child| {
                corners(child).iter().all(|q| if above { distance(implicit, *q) <= tau } else { distance(implicit, *q) >= -tau })
                    && holds(child, implicit, tau, above, depth - 1, budget)
            })
        })
    }
    [true, false].iter().any(|&above| {
        if !nets.iter().all(reach(above)) {
            return false;
        }
        let mut budget = REFINE_PER_CELL * nets.len();
        nets.iter().all(|net| holds(net, implicit, tau, above, MAX_REFINE, &mut budget))
    })
}

/// A cell's Euclidean control points' box.
pub(super) fn net_box(net: &Net) -> (Vec3, Vec3) {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for h in net.iter().flatten() {
        let p = Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3]);
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (lo, hi)
}

