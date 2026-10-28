//! 3+2 indexed operations (lane C10, row 2): run any 3-axis strategy in the
//! work-plane frame of a fixed tool orientation and map its stream back.
//!
//! - the frame `F` has its origin at the setup origin, `z = tool_axis`, `x` =
//!   setup X projected onto the plane perpendicular to the axis (setup Y when
//!   the axis is along X);
//! - heights of an indexed operation are measured along the tool axis;
//! - positions in the returned stream are in setup coordinates with
//!   `tool_axis` stamped on every motion that moves the tool;
//! - arcs are linearised at the operation tolerance (no tilted `ArcPlane`);
//! - canned cycles keep their depths along the tool axis;
//! - the stream is wrapped in `WorkPlane` / `WorkPlaneOff`.

use crate::arcfit::{arc_sweep, from_plane_coords, plane_coords};
use crate::model::{
    ApproximationLimit, ArcPlane, Feed, Motion, MotionKind, Operation, Point3, RigidTransform, Toolpath,
};
use crate::{CamError, MeshSnapshot, Tolerance};

/// The work-plane frame of a fixed tool orientation.
#[derive(Clone, Debug, PartialEq)]
pub struct IndexedFrame {
    /// Unit tool axis in setup coordinates (the frame's +Z).
    pub tool_axis: Point3,
    /// Unit frame X in setup coordinates.
    pub x_axis: Point3,
    /// Frame origin in setup coordinates.
    pub origin: Point3,
    /// Setup coordinates to frame coordinates.
    pub to_local: RigidTransform,
    /// Frame coordinates to setup coordinates.
    pub to_setup: RigidTransform,
}

impl IndexedFrame {
    /// Frame for `tool_axis` with the deterministic X rule.
    pub fn new(tool_axis: Point3, origin: Point3, tolerance: &Tolerance) -> Result<IndexedFrame, CamError> {
        let z = tool_axis
            .normalized()
            .ok_or_else(|| CamError::InvalidInput("indexed tool axis must be a non-zero vector".into()))?;
        if !origin.is_finite() {
            return Err(CamError::NonFinite { what: "indexed frame origin".into() });
        }
        let x_ref = if z.cross(Point3::X).length() <= tolerance.angle_eps.max(1e-12).sqrt() { Point3::Y } else { Point3::X };
        let to_local = RigidTransform::from_frame(origin, x_ref, z)?;
        let x_axis = to_local.inverse().apply_vector(Point3::X);
        Ok(IndexedFrame { tool_axis: z, x_axis, origin, to_setup: to_local.inverse(), to_local })
    }

    /// The operation's frame: `None` when it machines along `+Z` (no indexing).
    pub fn of_operation(operation: &Operation, tolerance: &Tolerance) -> Result<Option<IndexedFrame>, CamError> {
        let Some(axis) = operation.tool_axis else { return Ok(None) };
        let z = axis
            .normalized()
            .ok_or_else(|| CamError::InvalidInput("indexed tool axis must be a non-zero vector".into()))?;
        if 1.0 - z.dot(Point3::Z) <= tolerance.angle_eps {
            return Ok(None);
        }
        Self::new(z, Point3::ZERO, tolerance).map(Some)
    }

    /// The `WorkPlane` motion announcing this frame.
    pub fn work_plane(&self) -> Motion {
        Motion::new(
            MotionKind::WorkPlane { origin: self.origin, x_axis: self.x_axis, z_axis: self.tool_axis },
            Feed::RAPID,
        )
    }

    /// Frame coordinates of a setup point (what the post writes under G68.2).
    pub fn local_of(&self, p: Point3) -> Point3 {
        self.to_local.apply_point(p)
    }
}

/// Run `inner` (any 3-axis generator taking the operation and the complete
/// protected mesh in ITS machine frame) for an operation with a fixed tool
/// orientation. A `+Z` operation runs `inner` unchanged. The returned stream
/// is in setup coordinates, wrapped in `WorkPlane` / `WorkPlaneOff`.
pub fn generate_indexed<F>(
    operation: &Operation,
    mesh: &MeshSnapshot,
    tolerance: Tolerance,
    inner: F,
) -> Result<Toolpath, CamError>
where
    F: FnOnce(&Operation, &MeshSnapshot) -> Result<Toolpath, CamError>,
{
    let Some(frame) = IndexedFrame::of_operation(operation, &tolerance)? else {
        return inner(operation, mesh);
    };
    let local_mesh = mesh.transformed(&frame.to_local);
    let mut local_op = operation.clone();
    local_op.tool_axis = None;
    let local = inner(&local_op, &local_mesh)?;
    let mut out = Toolpath::new(local.operation.clone(), local.tool.clone());
    out.report = local.report.clone();
    out.motions.push(frame.work_plane());
    let mut at: Option<Point3> = None;
    let mut arcs = 0u64;
    let chord = operation.tolerance;
    for m in &local.motions {
        match &m.kind {
            MotionKind::ArcCw { to, center, plane, pitch } | MotionKind::ArcCcw { to, center, plane, pitch } => {
                let Some(from) = at else {
                    return Err(CamError::InvalidInput("indexed: an arc has no start position".into()));
                };
                let cw = matches!(m.kind, MotionKind::ArcCw { .. });
                let points = linearize_arc(from, *to, *center, *plane, *pitch, cw, chord);
                arcs += 1;
                for p in points {
                    let mut cut = Motion::new(MotionKind::Cut { to: frame.to_setup.apply_point(p) }, m.feed);
                    cut.tool_axis = Some(frame.tool_axis);
                    cut.path_id = m.path_id;
                    out.motions.push(cut);
                }
                at = Some(*to);
            }
            other => {
                let mut mapped = m.clone();
                mapped.kind = map_kind(other, &frame.to_setup);
                if other.end().is_some() {
                    mapped.tool_axis = Some(frame.tool_axis);
                }
                if let Some(e) = other.end() {
                    at = Some(e);
                }
                out.motions.push(mapped);
            }
        }
    }
    out.motions.push(Motion::new(MotionKind::WorkPlaneOff, Feed::RAPID));
    if arcs > 0 {
        out.report.approximations.push(ApproximationLimit {
            kind: "arc-discretized".into(),
            bound: Some(chord),
            detail: Some(format!("{arcs} arcs linearised for the indexed work plane")),
        });
    }
    out.report.counts.motions = out.motions.len() as u64;
    Ok(out)
}

/// A motion's positions mapped by `t`; cycles keep their axial depths.
fn map_kind(kind: &MotionKind, t: &RigidTransform) -> MotionKind {
    let p = |q: &Point3| t.apply_point(*q);
    match kind {
        MotionKind::Rapid { to } => MotionKind::Rapid { to: p(to) },
        MotionKind::Plunge { to } => MotionKind::Plunge { to: p(to) },
        MotionKind::Cut { to } => MotionKind::Cut { to: p(to) },
        MotionKind::Retract { to } => MotionKind::Retract { to: p(to) },
        MotionKind::LeadIn { to } => MotionKind::LeadIn { to: p(to) },
        MotionKind::LeadOut { to } => MotionKind::LeadOut { to: p(to) },
        MotionKind::Link { to } => MotionKind::Link { to: p(to) },
        MotionKind::Cycle { at, cycle } => MotionKind::Cycle { at: p(at), cycle: cycle.clone() },
        other => other.clone(),
    }
}

/// Points along an arc (excluding the start, including the end) with chord
/// error at most `chord`.
pub fn linearize_arc(
    from: Point3,
    to: Point3,
    center: Point3,
    plane: ArcPlane,
    _pitch: f64,
    clockwise: bool,
    chord: f64,
) -> Vec<Point3> {
    let Some((sweep, radius)) = arc_sweep(from, to, center, plane, clockwise) else { return vec![to] };
    let max_step = if chord >= radius { std::f64::consts::PI } else { 2.0 * (1.0 - chord / radius).acos() };
    let n = ((sweep / max_step.max(1e-6)).ceil() as usize).clamp(1, 100_000);
    let (cu, cv, _) = plane_coords(center, plane);
    let (u0, v0, w0) = plane_coords(from, plane);
    let (_, _, w1) = plane_coords(to, plane);
    let r0 = (u0 - cu).hypot(v0 - cv);
    let a0 = (v0 - cv).atan2(u0 - cu);
    // The axial rise is linear in the sweep, so the end's `w` fixes the pitch.
    let sign = if clockwise { -1.0 } else { 1.0 };
    let mut out = Vec::with_capacity(n);
    for i in 1..=n {
        if i == n {
            out.push(to);
            break;
        }
        let f = i as f64 / n as f64;
        let a = a0 + sign * sweep * f;
        let w = w0 + (w1 - w0) * f;
        out.push(from_plane_coords(cu + r0 * a.cos(), cv + r0 * a.sin(), w, plane));
    }
    out
}
