//! Tool-axis control for multi-axis finishing (lane C10b, row 1): the axis
//! of a cutter-location point from the surface normal and travel direction
//! (lead about the binormal, tilt about the tangent), toward or away from a
//! point or a line, or at a fixed angle from vertical; smoothing that limits
//! the change per sample; a limit on the angle from vertical. The
//! [`AxisMode`] enum itself lives in the model (persisted operation data);
//! this module adds its geometry.

use super::slerp;
use crate::kinematics::rotate;
pub use crate::model::AxisMode;
use crate::model::Point3;

impl AxisMode {
    /// The unit axis at a contact point `cc` with unit surface normal `n` and
    /// unit travel tangent `t` (already perpendicular to `n`). `None` when the
    /// mode is degenerate there (the contact lies on the point or the line).
    pub fn axis_at(&self, cc: Point3, n: Point3, t: Point3) -> Option<Point3> {
        match self {
            AxisMode::Normal { lead_deg, tilt_deg } => {
                let b = n.cross(t);
                let a = rotate(b, lead_deg.to_radians(), n);
                rotate(t, tilt_deg.to_radians(), a).normalized()
            }
            AxisMode::Point { point, away } => {
                let d = if *away { cc.sub(*point) } else { point.sub(cc) };
                d.normalized()
            }
            AxisMode::Line { origin, direction, away } => {
                let d = direction.normalized()?;
                let rel = cc.sub(*origin);
                let perp = rel.sub(d.scale(rel.dot(d)));
                let v = if *away { perp } else { perp.scale(-1.0) };
                v.normalized()
            }
            AxisMode::FixedFromVertical { angle_deg, azimuth_deg } => {
                let (s, c) = angle_deg.to_radians().sin_cos();
                let (sa, ca) = azimuth_deg.to_radians().sin_cos();
                Some(Point3::new(s * ca, s * sa, c))
            }
        }
    }
}

/// Limit the angle between consecutive axes to `max_rad` (alternating
/// forward and backward passes until stable); returns the number of changes.
pub fn smooth_axes(axes: &mut [Point3], max_rad: f64) -> usize {
    let mut changed = 0;
    let n = axes.len();
    for round in 0..64 {
        let mut round_changes = 0;
        for k in 1..n {
            let (i, j) = if round % 2 == 0 { (k - 1, k) } else { (n - k, n - k - 1) };
            let angle = axes[i].dot(axes[j]).clamp(-1.0, 1.0).acos();
            if angle > max_rad {
                axes[j] = slerp(axes[i], axes[j], (max_rad / angle) * (1.0 - 1e-9));
                round_changes += 1;
            }
        }
        changed += round_changes;
        if round_changes == 0 {
            break;
        }
    }
    changed
}

/// Pull `axis` back to within `max_rad` of +Z when it leans further; returns
/// the clamped axis and whether it changed.
pub fn clamp_from_vertical(axis: Point3, max_rad: f64) -> (Point3, bool) {
    let angle = axis.dot(Point3::Z).clamp(-1.0, 1.0).acos();
    if angle <= max_rad {
        return (axis, false);
    }
    (slerp(Point3::Z, axis, max_rad / angle), true)
}
