//! A slab: a block flat underneath and on its four sides whose top is a
//! height field, the bicubic B-spline `z = f(x / a, y / b)` over the
//! rectangle `0 ≤ x ≤ a`, `0 ≤ y ≤ b`.
//!
//! The top is that spline exactly, as a surface whose control points sit at
//! the knots' Greville abscissae times the size (so its x and y run linearly
//! with its parameters) and at the coefficients' heights. Each side is a
//! plane trimmed by the floor's edge, two uprights and the top's boundary
//! curve on that side (the spline's own side, which lies in the plane); the
//! six faces are sewn along their shared curves.

use crate::{
    face_from_surface, face_from_surface_trimmed, make_line, solid_from_sheets, BrepSolid, KernelRefusal, KernelStage,
    NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4,
};

/// The prefix of every refusal of this module.
pub const SLAB_REFUSAL: &str = "slab";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{SLAB_REFUSAL}: {message}"))
}

/// The Greville abscissae of a cubic spline's knots: the mean of the three
/// knots after each coefficient's first.
fn greville(knots: &[f64]) -> Vec<f64> {
    (0..knots.len() - 4).map(|i| (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3.0).collect()
}

/// Whether `knots` is a clamped cubic knot vector over `[0, 1]` for `count`
/// coefficients: `count + 4` knots, non-decreasing, four 0s first and four 1s
/// last, no inner knot repeated more than three times.
fn clamped_cubic(knots: &[f64], count: usize) -> bool {
    knots.len() == count + 4
        && count >= 4
        && knots.windows(2).all(|w| w[0] <= w[1])
        && knots[..4].iter().all(|&k| k == 0.0)
        && knots[count..].iter().all(|&k| k == 1.0)
        && knots[4..count].iter().all(|&k| k > 0.0 && k < 1.0)
        && knots.windows(4).skip(1).take(count.saturating_sub(4)).all(|w| w[0] < w[3])
}

/// The plane patch through `origin` spanned by `along` and `up` (bilinear).
fn plane_patch(origin: Vec3, along: Vec3, up: Vec3) -> Result<NurbsSurface, String> {
    let p = |a: f64, b: f64| Vec4::from_point(origin.add(along.scale(a)).add(up.scale(b)), 1.0);
    NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], vec![vec![p(0.0, 0.0), p(0.0, 1.0)], vec![p(1.0, 0.0), p(1.0, 1.0)]])
}

/// The slab over the rectangle `0 ≤ x ≤ size[0]`, `0 ≤ y ≤ size[1]`, from
/// `z = 0` up to the bicubic B-spline height field with clamped knots
/// `knots_u`, `knots_v` (along x and along y) and coefficients
/// `heights[i][j]` (`i` along x), every one positive.
///
/// Refuses a size that is not positive, knots that are not a clamped cubic
/// vector over `[0, 1]` for the coefficients, a height that is not positive
/// and finite, and a slab under a millionth as thick or narrow as it is wide.
pub fn make_slab_brep(size: [f64; 2], knots_u: &[f64], knots_v: &[f64], heights: &[Vec<f64>]) -> Result<BrepSolid, KernelRefusal> {
    let [a, b] = size;
    if !(a.is_finite() && b.is_finite() && a > 0.0 && b > 0.0) {
        return Err(input("slab_size", format!("the size {size:?} is not positive")));
    }
    let (nu, nv) = (heights.len(), heights.first().map_or(0, Vec::len));
    if heights.iter().any(|row| row.len() != nv) {
        return Err(input("slab_heights", "the rows of heights differ in length".into()));
    }
    if !clamped_cubic(knots_u, nu) || !clamped_cubic(knots_v, nv) {
        return Err(input(
            "slab_knots",
            format!("the knots ({} and {}) are not clamped cubic knots over [0, 1] for {nu} x {nv} heights", knots_u.len(), knots_v.len()),
        ));
    }
    if heights.iter().flatten().any(|&h| !(h.is_finite() && h > 0.0)) {
        return Err(input("slab_heights", "a height is not positive".into()));
    }
    // A slab far thinner or narrower than it is wide lies inside the faces'
    // own joining tolerance (a billionth of each face's extent), which would
    // weld its corners together.
    let lowest = heights.iter().flatten().fold(f64::INFINITY, |m, &h| m.min(h));
    let widest = a.max(b).max(heights.iter().flatten().fold(0.0f64, |m, &h| m.max(h)));
    if lowest.min(a).min(b) < 1e-6 * widest {
        return Err(input("slab_aspect", format!("the slab is under a millionth as thick or narrow ({}) as it is wide ({widest})", lowest.min(a).min(b))));
    }
    let (gu, gv) = (greville(knots_u), greville(knots_v));
    let poles: Vec<Vec<Vec4>> =
        (0..nu).map(|i| (0..nv).map(|j| Vec4::from_point(Vec3::new(gu[i] * a, gv[j] * b, heights[i][j]), 1.0)).collect()).collect();
    let top = NurbsSurface::new(3, 3, knots_u.to_vec(), knots_v.to_vec(), poles).or_refuse(KernelStage::Collect, "slab.top")?;
    // Each side plane reaches past the top's highest pole, which the top lies under.
    let reach = heights.iter().flatten().fold(0.0f64, |m, &h| m.max(h)) * 1.25 + 1.0;
    let up = Vec3::new(0.0, 0.0, reach);
    let corner = |x: f64, y: f64| Vec3::new(x, y, 0.0);
    let line = |p: Vec3, q: Vec3| make_line(p, q).or_refuse(KernelStage::Collect, "slab.line");
    // One side: the top's side curve from over `p` to over `q` (run as the
    // top's own edge runs, so the two sew with one parametrization), the
    // upright down at `q`, the floor's edge back to `p`, the upright up at `p`.
    let side = |p: Vec3, q: Vec3, curve: NurbsCurve| -> Result<BrepSolid, KernelRefusal> {
        let plane = plane_patch(p, q.sub(p), up).or_refuse(KernelStage::Collect, "slab.side")?;
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Collect, "slab.side")?;
        let start = curve.evaluate(t0).or_refuse(KernelStage::Collect, "slab.side")?;
        let end = curve.evaluate(t1).or_refuse(KernelStage::Collect, "slab.side")?;
        face_from_surface_trimmed(&plane, &[vec![curve, line(end, q)?, line(q, p)?, line(p, start)?]])
    };
    let iso = |along_x: bool, at: f64| if along_x { top.iso_curve_v(at) } else { top.iso_curve_u(at) }.or_refuse(KernelStage::Collect, "slab.top_side");
    let sheets = vec![
        face_from_surface(&top)?,
        face_from_surface(&plane_patch(corner(0.0, 0.0), Vec3::new(a, 0.0, 0.0), Vec3::new(0.0, b, 0.0)).or_refuse(KernelStage::Collect, "slab.floor")?)?,
        // y = 0 and y = b: the top's sides along x (v at 0 and 1).
        side(corner(0.0, 0.0), corner(a, 0.0), iso(true, 0.0)?)?,
        side(corner(0.0, b), corner(a, b), iso(true, 1.0)?)?,
        // x = 0 and x = a: the top's sides along y (u at 0 and 1).
        side(corner(0.0, 0.0), corner(0.0, b), iso(false, 0.0)?)?,
        side(corner(a, 0.0), corner(a, b), iso(false, 1.0)?)?,
    ];
    // The shared curves are the same curves; the tolerance only pairs them, and
    // leaves room for the projections that read a cubic's locus. It stays far
    // under the slab's least thickness (the top never dips below its least
    // coefficient) and its narrower side, so no edge pairs with another than
    // its own.
    let tolerance = (1e-7 * a.max(b).max(reach)).min(1e-3 * lowest.min(a).min(b));
    solid_from_sheets(&sheets, tolerance)
}

