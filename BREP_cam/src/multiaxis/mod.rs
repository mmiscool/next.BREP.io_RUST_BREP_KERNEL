//! Multi-axis machining (lane C10): the 3+2 indexed wrapper, contact in the
//! tool frame, and collision avoidance by tilting. Strategies that emit
//! simultaneous multi-axis motion live in `strategy/{rotary, swarf,
//! multiaxis_contour}.rs`.

pub mod contact;
pub mod indexed;
pub mod links;
pub mod orientation;
pub mod sim;
pub mod tilt;

pub use contact::{clearance_along, drop_along, AxisDrop, Clearance, ClearanceHit, ToolFrame};
pub use indexed::{generate_indexed, linearize_arc, IndexedFrame};
pub use orientation::{clamp_from_vertical, smooth_axes, AxisMode};
pub use tilt::{avoid_collisions, ClPose, TiltOptions, TiltedPose};

/// Spherical interpolation of unit vectors.
pub(crate) fn slerp(a: crate::model::Point3, b: crate::model::Point3, t: f64) -> crate::model::Point3 {
    let d = a.dot(b).clamp(-1.0, 1.0);
    let theta = d.acos();
    if theta < 1e-12 {
        return a;
    }
    let s = theta.sin();
    a.scale(((1.0 - t) * theta).sin() / s).add(b.scale((t * theta).sin() / s)).normalized().unwrap_or(b)
}

