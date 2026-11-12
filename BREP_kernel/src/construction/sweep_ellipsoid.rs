//! The whole sweep of an ellipsoid slid along a line.
//!
//! An ellipsoid is the unit ball under an affine map `A` (a stretch onto its
//! three semi-axes, then a placement), and a slide commutes with `A`:
//! `A(K + t·w) = A(K) + t·A·w`. So the ellipsoid slid by `v` sweeps `A` of what
//! the unit ball sweeps slid by `w = A⁻¹·v`, which the whole sweep of a solid
//! ([`crate::sweep_solid`]) builds from the ball's two caps and the wall its
//! great circle square to `w` sweeps. `A` carries that body's rational poles
//! exactly, so the result's caps are pieces of the ellipsoid where it starts
//! and where it stops, and its wall the elliptic cylinder between them.
use crate::{sweep_solid, transform_brep, AffineTransform, BrepSolid, KernelRefusal, KernelStage, OrRefuse, OutlinePiece, ScrewStep, SolidMotion, SolidTool, Vec3};
use std::f64::consts::FRAC_PI_2;

/// Everything the ellipsoid about `center` passes through as it slides without
/// turning by `slide`. Its semi-axes are `semi_axes[0]` along `x_direction`,
/// `semi_axes[1]` square to it toward `y_hint`, and `semi_axes[2]` along their
/// cross product (as [`crate::make_ellipsoid_brep`]).
pub fn sweep_ellipsoid_along_line(center: Vec3, x_direction: Vec3, y_hint: Vec3, semi_axes: [f64; 3], slide: Vec3) -> Result<BrepSolid, KernelRefusal> {
    let input = |what: &'static str, message: &str| KernelRefusal::input(KernelStage::Collect, what, format!("sweepEllipsoid: {message}"));
    if semi_axes.iter().any(|axis| !(axis.is_finite() && *axis > 0.0)) {
        return Err(input("ellipsoid_axes", "the three semi-axes must be positive"));
    }
    let x = x_direction.normalized().map_err(|_| input("ellipsoid_frame", "the first direction must be non-zero"))?;
    let y = y_hint.sub(x.scale(y_hint.dot(x))).normalized().map_err(|_| input("ellipsoid_frame", "the second direction must not be parallel to the first"))?;
    let z = x.cross(y);
    let [a, b, c] = semi_axes;
    // the slide as the unit ball sees it
    let seen = Vec3::new(slide.dot(x) / a, slide.dot(y) / b, slide.dot(z) / c);
    let along = seen.normalized().map_err(|_| input("slide", "the slide has no length"))?;
    let across = if along.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let radial = across.sub(along.scale(across.dot(along))).normalized().or_refuse(KernelStage::Collect, "sweepEllipsoid.frame")?;
    let origin = Vec3::new(0.0, 0.0, 0.0);
    // the ball as a solid of revolution about the slide's own line
    let ball = SolidTool::Revolved {
        axis_origin: origin,
        axis_direction: along,
        radial,
        meridian: vec![
            OutlinePiece::Arc { centre: [0.0, 0.0], radius: 1.0, start: -FRAC_PI_2, end: FRAC_PI_2 },
            OutlinePiece::Line { from: [0.0, 1.0], to: [0.0, -1.0] },
        ],
    };
    let motion = SolidMotion { steps: vec![ScrewStep { axis_origin: origin, axis_direction: along, turn: 0.0, advance: seen.length() }], from: 0.0, to: 1.0 };
    let swept = sweep_solid(&ball, &motion)?;
    let (ex, ey, ez) = (x.scale(a), y.scale(b), z.scale(c));
    let map = AffineTransform { elements: [ex.x, ey.x, ez.x, center.x, ex.y, ey.y, ez.y, center.y, ex.z, ey.z, ez.z, center.z, 0.0, 0.0, 0.0, 1.0] };
    transform_brep(&swept, map, false).or_refuse(KernelStage::Collect, "sweepEllipsoid.stretch")
}

