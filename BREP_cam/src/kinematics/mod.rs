//! Machine kinematics (lane C10, row 1): forward and inverse kinematics of
//! 3-, 4- and 5-axis mills from the shared [`Machine`] definition.
//!
//! Conventions (sections 1-3):
//! - A cutter-location point is the tool tip `p` plus the unit tool axis `a`
//!   (tip toward spindle), both in the SETUP frame S. `None` axis is `+Z`.
//! - A rotary joint `(d, o, θ)` rotates by `θ` (right-hand rule about the unit
//!   direction `d`) about the line through `o`. A TABLE joint rotates the part,
//!   a HEAD joint rotates the tool. The primary joint carries the secondary.
//! - Head pivots are relative to the controlled point (tool tip at zero
//!   angles); table pivots are in S.
//! - Angles are degrees in the API and radians inside. Controller sign
//!   conventions belong to the post.
//!
//! Every chain reduces to one inverse problem, [`two_axis`]: rotate `u` first
//! about `d_a` by `α`, then about `d_b` by `β`, to reach `w`. No orthogonality
//! between the axes is assumed, so nutating heads solve with the same code.
//! Both solution branches are returned; a free angle inside the singular cone
//! is held at the previous value and tagged `singular`.

use crate::model::{Kinematics, Machine, MachineKind, Point3, RigidTransform, RotaryAxis};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

pub mod library;
pub mod post;
pub mod sequence;

/// Which side of the machine a rotary joint moves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Carrier {
    /// Rotates the part.
    Table,
    /// Rotates the tool.
    Head,
}

/// One rotary joint of a chain.
#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    /// Axis letter the post emits.
    pub name: String,
    /// Unit rotation direction.
    pub direction: Point3,
    /// A point on the axis: in S for table joints, relative to the controlled
    /// point for head joints.
    pub origin: Point3,
    /// Angular limits in degrees (`None`: continuous).
    pub limits_deg: Option<[f64; 2]>,
    pub carrier: Carrier,
}

/// A rigid link of the machine that solids attach to: the model's type, re-exported
/// so machine definitions and the chain share one `Link`.
pub use crate::model::Link;

/// The kinematic arrangement, with the joints in output order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChainKind {
    /// No rotary joint; only `+Z` tool axes are reachable.
    Mill3,
    /// One table rotary (`joints[0]`).
    Mill4,
    /// `joints[0]` primary table carries `joints[1]` secondary table.
    TableTable,
    /// `joints[0]` primary head carries `joints[1]` secondary head (spindle).
    HeadHead,
    /// `joints[0]` head tilts the tool, `joints[1]` table turns the part.
    HeadTable,
}

/// A machine's kinematic chain.
#[derive(Clone, Debug, PartialEq)]
pub struct Chain {
    pub kind: ChainKind,
    pub joints: Vec<Joint>,
}

/// Solver settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicsOptions {
    /// Rotary tool-centre-point control (G43.4 / TRAORI / M128): linear words
    /// are the tip in S. Off: linear words are the machine axes with the
    /// pivots pushed through the chain.
    pub tcp: bool,
    /// Half-angle of the cone about a joint's direction inside which the
    /// tool axis leaves that joint's angle free (the near-vertical C problem).
    pub singular_cone_deg: f64,
    /// Reachability tolerance on direction cosines (a 4-axis tool axis that
    /// misses the rotary's plane by more than this is refused).
    pub cos_eps: f64,
}

impl Default for KinematicsOptions {
    fn default() -> Self {
        Self { tcp: true, singular_cone_deg: 0.5, cos_eps: 1e-6 }
    }
}

impl KinematicsOptions {
    pub fn validate(&self) -> Result<(), KinematicsError> {
        if !self.singular_cone_deg.is_finite() || self.singular_cone_deg < 0.0 || self.singular_cone_deg >= 90.0 {
            return Err(KinematicsError::InvalidInput("singular cone must be in [0, 90) degrees".into()));
        }
        if !self.cos_eps.is_finite() || self.cos_eps < 0.0 || self.cos_eps > 1e-3 {
            return Err(KinematicsError::InvalidInput("cosine tolerance must be in [0, 1e-3]".into()));
        }
        Ok(())
    }
}

/// One machine configuration reaching a CL point.
#[derive(Clone, Debug, PartialEq)]
pub struct Pose {
    /// Linear axis words: the tip in S with TCP on, the machine axes with TCP off.
    pub linear: Point3,
    /// Rotary angles in degrees, in `Chain::joints` order, in `(-180, 180]`
    /// unless the sequence solver unwound them.
    pub angles_deg: Vec<f64>,
    /// Solution branch (0 or 1) of the inverse problem.
    pub branch: u8,
    /// A free angle was held at the previous (or zero) value.
    pub singular: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub enum KinematicsError {
    /// The machine definition cannot be turned into a chain.
    InvalidMachine(String),
    InvalidInput(String),
    /// The tool axis is not finite or zero.
    InvalidAxis,
    /// No joint configuration reaches the tool axis (ignoring limits).
    Unreachable { axis: Point3 },
    /// Every solution violates a joint's limits. `index` is the CL point.
    LimitViolation { index: usize, joint: String },
}

impl std::fmt::Display for KinematicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            KinematicsError::InvalidMachine(m) => write!(f, "machine: {m}"),
            KinematicsError::InvalidInput(m) => write!(f, "kinematics: {m}"),
            KinematicsError::InvalidAxis => write!(f, "tool axis must be a finite non-zero vector"),
            KinematicsError::Unreachable { axis } => {
                write!(f, "tool axis [{}, {}, {}] is unreachable on this machine", axis.x, axis.y, axis.z)
            }
            KinematicsError::LimitViolation { index, joint } => {
                write!(f, "CL point {index} needs rotary {joint} outside its limits")
            }
        }
    }
}

impl std::error::Error for KinematicsError {}

impl From<KinematicsError> for crate::CamError {
    fn from(e: KinematicsError) -> Self {
        crate::CamError::InvalidInput(e.to_string())
    }
}

// ---------------------------------------------------------------------------
// Rotation helpers
// ---------------------------------------------------------------------------

/// Rodrigues rotation of `v` by `theta` radians about the unit direction `d`.
pub fn rotate(d: Point3, theta: f64, v: Point3) -> Point3 {
    let (s, c) = theta.sin_cos();
    v.scale(c).add(d.cross(v).scale(s)).add(d.scale(d.dot(v) * (1.0 - c)))
}

/// Rotation of the point `x` about the line through `o` with direction `d`.
pub fn rotate_about(o: Point3, d: Point3, theta: f64, x: Point3) -> Point3 {
    o.add(rotate(d, theta, x.sub(o)))
}

/// Signed angle (radians) about `d` from `x` to `y`, both perpendicular to `d`
/// and non-zero.
fn signed_angle(d: Point3, x: Point3, y: Point3) -> f64 {
    (x.cross(y).dot(d)).atan2(x.dot(y))
}

/// Component of `v` perpendicular to the unit direction `d`.
fn perp(d: Point3, v: Point3) -> Point3 {
    v.sub(d.scale(v.dot(d)))
}

/// Wrap radians into `(-π, π]`.
fn wrap_pi(a: f64) -> f64 {
    let mut r = a % (2.0 * PI);
    if r <= -PI {
        r += 2.0 * PI;
    } else if r > PI {
        r -= 2.0 * PI;
    }
    r
}

/// Wrap degrees into `(-180, 180]`.
pub fn wrap_deg(a: f64) -> f64 {
    let mut r = a % 360.0;
    if r <= -180.0 {
        r += 360.0;
    } else if r > 180.0 {
        r -= 360.0;
    }
    r
}

/// One branch of [`two_axis`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoAxisSolution {
    /// Angle about the first axis (radians).
    pub alpha: f64,
    /// Angle about the second axis (radians).
    pub beta: f64,
    pub singular: bool,
}

/// Solve `R(d_b, β) R(d_a, α) u = w` for unit `u`, `w`, `d_a`, `d_b`.
///
/// `prev` supplies the held values of free angles (singular configurations).
/// `cone` is the singular cone half-angle in radians: when `w` is within
/// it of `±d_b`, `β` is free; when the first rotation cannot change the dot
/// product with `d_b`, `α` is free. Returns 0, 1 or 2 branches, branch 0 first.
pub fn two_axis(
    d_a: Point3,
    d_b: Point3,
    u: Point3,
    w: Point3,
    prev: [f64; 2],
    cone: f64,
    cos_eps: f64,
) -> Vec<TwoAxisSolution> {
    // Rotation about d_b preserves the dot with d_b:
    //   (R(d_a, α) u)·d_b = A cos α + B sin α + C = k.
    let ua = u.dot(d_a);
    let ab = d_a.dot(d_b);
    let a = u.dot(d_b) - ua * ab;
    let b = d_a.cross(u).dot(d_b);
    let c = ua * ab;
    let k = w.dot(d_b);
    // r = sin(u, d_a) · sin(d_a, d_b): the amplitude the first joint can give
    // the dot product. Inside the singular cone the first angle is free.
    let r = a.hypot(b);
    let sin_cone = cone.sin().max(0.0);
    let mut alphas: Vec<(f64, bool)> = Vec::new();
    if r <= sin_cone.max(cos_eps) {
        if (k - c).abs() > r + cos_eps {
            return Vec::new();
        }
        alphas.push((prev[0], true));
    } else {
        let phi = b.atan2(a);
        let ratio = (k - c) / r;
        if ratio > 1.0 + cos_eps || ratio < -1.0 - cos_eps {
            return Vec::new();
        }
        let delta = ratio.clamp(-1.0, 1.0).acos();
        alphas.push((wrap_pi(phi + delta), false));
        if delta > 0.0 {
            alphas.push((wrap_pi(phi - delta), false));
        }
    }
    let w_perp = perp(d_b, w);
    let w_singular = w_perp.length() <= sin_cone.max(cos_eps);
    let mut out = Vec::new();
    for (alpha, alpha_free) in alphas {
        let u1 = rotate(d_a, alpha, u);
        let u_perp = perp(d_b, u1);
        let (beta, beta_free) = if w_singular || u_perp.length() <= 1e-12 {
            (prev[1], true)
        } else {
            (wrap_pi(signed_angle(d_b, u_perp, w_perp)), false)
        };
        out.push(TwoAxisSolution { alpha, beta, singular: alpha_free || beta_free });
    }
    out
}

/// Solve `R(d, θ) u = w` for one joint: reachable only when `u·d = w·d`.
/// `prev` holds the free angle when `u ∥ d`.
pub fn one_axis(d: Point3, u: Point3, w: Point3, prev: f64, cone: f64, cos_eps: f64) -> Option<(f64, bool)> {
    if (u.dot(d) - w.dot(d)).abs() > cos_eps {
        return None;
    }
    let up = perp(d, u);
    let wp = perp(d, w);
    if wp.length() <= cone.sin().max(cos_eps) || up.length() <= cos_eps {
        return Some((prev, true));
    }
    Some((wrap_pi(signed_angle(d, up, wp)), false))
}

// ---------------------------------------------------------------------------
// Chains
// ---------------------------------------------------------------------------

fn joint(axis: &RotaryAxis, carrier: Carrier, wcs_origin: Point3) -> Result<Joint, KinematicsError> {
    axis.validate().map_err(KinematicsError::InvalidMachine)?;
    let direction = axis.direction.normalized().ok_or_else(|| {
        KinematicsError::InvalidMachine(format!("rotary axis {} has a zero direction", axis.name))
    })?;
    let origin = match carrier {
        Carrier::Table => axis.origin.sub(wcs_origin),
        Carrier::Head => axis.origin,
    };
    Ok(Joint { name: axis.name.clone(), direction, origin, limits_deg: axis.limits_deg, carrier })
}

impl Chain {
    /// Build the chain of `machine`. Table pivots given in machine coordinates
    /// are shifted into S by the setup's `wcs_origin`.
    pub fn from_machine(machine: &Machine, wcs_origin: Point3) -> Result<Chain, KinematicsError> {
        machine.validate().map_err(KinematicsError::InvalidMachine)?;
        if !wcs_origin.is_finite() {
            return Err(KinematicsError::InvalidMachine("work offset origin is not finite".into()));
        }
        let chain = match &machine.kind {
            MachineKind::Mill3 => Chain { kind: ChainKind::Mill3, joints: Vec::new() },
            MachineKind::Lathe => {
                return Err(KinematicsError::InvalidMachine("lathe kinematics are not a milling chain".into()))
            }
            MachineKind::Mill4 { rotary } => {
                Chain { kind: ChainKind::Mill4, joints: vec![joint(rotary, Carrier::Table, wcs_origin)?] }
            }
            // A mill-turn's C axis is one table joint, as C11b specified (item 3 of its
            // request); the optional Y is constrained in the owned millturn API.
            MachineKind::MillTurn { c_axis, .. } => {
                Chain { kind: ChainKind::Mill4, joints: vec![joint(c_axis, Carrier::Table, wcs_origin)?] }
            }
            MachineKind::Mill5 { kinematics } => match kinematics {
                Kinematics::TableTable { primary, secondary } => Chain {
                    kind: ChainKind::TableTable,
                    joints: vec![
                        joint(primary, Carrier::Table, wcs_origin)?,
                        joint(secondary, Carrier::Table, wcs_origin)?,
                    ],
                },
                Kinematics::HeadHead { primary, secondary } => Chain {
                    kind: ChainKind::HeadHead,
                    joints: vec![
                        joint(primary, Carrier::Head, wcs_origin)?,
                        joint(secondary, Carrier::Head, wcs_origin)?,
                    ],
                },
                Kinematics::HeadTable { head, table } => Chain {
                    kind: ChainKind::HeadTable,
                    joints: vec![joint(head, Carrier::Head, wcs_origin)?, joint(table, Carrier::Table, wcs_origin)?],
                },
            },
        };
        if chain.joints.len() == 2 {
            let (a, b) = (chain.joints[0].direction, chain.joints[1].direction);
            if a.cross(b).length() <= 1e-9 {
                return Err(KinematicsError::InvalidMachine("the two rotary axes are parallel".into()));
            }
            if chain.joints[0].name == chain.joints[1].name {
                return Err(KinematicsError::InvalidMachine("the two rotary axes share a name".into()));
            }
        }
        Ok(chain)
    }

    /// A chain without rotary joints.
    pub fn mill3() -> Chain {
        Chain { kind: ChainKind::Mill3, joints: Vec::new() }
    }

    pub fn rotary_count(&self) -> usize {
        self.joints.len()
    }

    /// Joint names in output order.
    pub fn joint_names(&self) -> Vec<&str> {
        self.joints.iter().map(|j| j.name.as_str()).collect()
    }

    fn check_angles(&self, angles_deg: &[f64]) -> Result<(), KinematicsError> {
        if angles_deg.len() != self.joints.len() {
            return Err(KinematicsError::InvalidInput(format!(
                "expected {} rotary angles, got {}",
                self.joints.len(),
                angles_deg.len()
            )));
        }
        if angles_deg.iter().any(|a| !a.is_finite()) {
            return Err(KinematicsError::InvalidInput("rotary angle is not finite".into()));
        }
        Ok(())
    }

    /// Rigid transform of a rotary joint at `theta` radians: `x ↦ o + R(x − o)`.
    fn joint_transform(j: &Joint, theta: f64) -> RigidTransform {
        let r = RigidTransform::axis_angle(j.direction, theta, Point3::ZERO).unwrap_or_else(|_| RigidTransform::identity());
        let shift = j.origin.sub(r.apply_vector(j.origin));
        RigidTransform { rotation: r.rotation, translation: shift }
    }

    /// World placement of a link's frame at the given angles and linear
    /// position (`x_world = T(x_link)`). Table links are given in S at zero
    /// angles; head and tool links relative to the controlled point.
    pub fn link_transform(&self, link: Link, angles_deg: &[f64], linear: Point3) -> Result<RigidTransform, KinematicsError> {
        self.check_angles(angles_deg)?;
        let th: Vec<f64> = angles_deg.iter().map(|a| a.to_radians()).collect();
        let j = &self.joints;
        let table_joints: Vec<usize> = (0..j.len()).filter(|&k| j[k].carrier == Carrier::Table).collect();
        let head_joints: Vec<usize> = (0..j.len()).filter(|&k| j[k].carrier == Carrier::Head).collect();
        // Outer-to-inner composition: for joints [a, b] carried in order, x_world = T_a(T_b(x)).
        let compose = |ids: &[usize]| -> RigidTransform {
            let mut t = RigidTransform::identity();
            for &k in ids.iter().rev() {
                // t = T_k ∘ t  (apply t first, then T_k)
                t = t.then(&Self::joint_transform(&j[k], th[k]));
            }
            t
        };
        let shift = RigidTransform::translation(linear);
        Ok(match link {
            Link::Base => RigidTransform::identity(),
            Link::Part => compose(&table_joints),
            Link::Tool => compose(&head_joints).then(&shift),
            Link::Joint { index: k } => {
                if k >= j.len() {
                    return Err(KinematicsError::InvalidInput(format!("link joint {k} does not exist")));
                }
                match j[k].carrier {
                    Carrier::Table => {
                        let upto: Vec<usize> = table_joints.iter().copied().take_while(|&i| i <= k).collect();
                        compose(&upto)
                    }
                    Carrier::Head => {
                        let upto: Vec<usize> = head_joints.iter().copied().take_while(|&i| i <= k).collect();
                        compose(&upto).then(&shift)
                    }
                }
            }
        })
    }

    /// Part point `x` (in S) to the machine frame W at the given angles.
    pub fn part_to_world(&self, angles_deg: &[f64], x: Point3) -> Point3 {
        let j = &self.joints;
        match self.kind {
            ChainKind::Mill3 | ChainKind::HeadHead => x,
            ChainKind::Mill4 => rotate_about(j[0].origin, j[0].direction, angles_deg[0].to_radians(), x),
            ChainKind::TableTable => {
                let inner = rotate_about(j[1].origin, j[1].direction, angles_deg[1].to_radians(), x);
                rotate_about(j[0].origin, j[0].direction, angles_deg[0].to_radians(), inner)
            }
            ChainKind::HeadTable => rotate_about(j[1].origin, j[1].direction, angles_deg[1].to_radians(), x),
        }
    }

    /// Machine-frame point to S at the given angles (inverse of [`Chain::part_to_world`]).
    pub fn world_to_part(&self, angles_deg: &[f64], x: Point3) -> Point3 {
        let j = &self.joints;
        match self.kind {
            ChainKind::Mill3 | ChainKind::HeadHead => x,
            ChainKind::Mill4 => rotate_about(j[0].origin, j[0].direction, -angles_deg[0].to_radians(), x),
            ChainKind::TableTable => {
                let outer = rotate_about(j[0].origin, j[0].direction, -angles_deg[0].to_radians(), x);
                rotate_about(j[1].origin, j[1].direction, -angles_deg[1].to_radians(), outer)
            }
            ChainKind::HeadTable => rotate_about(j[1].origin, j[1].direction, -angles_deg[1].to_radians(), x),
        }
    }

    /// Part-frame direction to W (rotation only).
    pub fn part_dir_to_world(&self, angles_deg: &[f64], v: Point3) -> Point3 {
        let j = &self.joints;
        match self.kind {
            ChainKind::Mill3 | ChainKind::HeadHead => v,
            ChainKind::Mill4 => rotate(j[0].direction, angles_deg[0].to_radians(), v),
            ChainKind::TableTable => {
                let inner = rotate(j[1].direction, angles_deg[1].to_radians(), v);
                rotate(j[0].direction, angles_deg[0].to_radians(), inner)
            }
            ChainKind::HeadTable => rotate(j[1].direction, angles_deg[1].to_radians(), v),
        }
    }

    /// Tool axis in W at the given angles.
    pub fn tool_axis_world(&self, angles_deg: &[f64]) -> Point3 {
        let j = &self.joints;
        match self.kind {
            ChainKind::Mill3 | ChainKind::Mill4 | ChainKind::TableTable => Point3::Z,
            ChainKind::HeadHead => {
                let inner = rotate(j[1].direction, angles_deg[1].to_radians(), Point3::Z);
                rotate(j[0].direction, angles_deg[0].to_radians(), inner)
            }
            ChainKind::HeadTable => rotate(j[0].direction, angles_deg[0].to_radians(), Point3::Z),
        }
    }

    /// Offset from the controlled point (linear axes) to the tool tip in W,
    /// produced by the head joints: zero for table-only chains.
    pub fn head_offset(&self, angles_deg: &[f64]) -> Point3 {
        let j = &self.joints;
        match self.kind {
            ChainKind::Mill3 | ChainKind::Mill4 | ChainKind::TableTable => Point3::ZERO,
            ChainKind::HeadHead => {
                let (t1, t2) = (angles_deg[0].to_radians(), angles_deg[1].to_radians());
                let o1 = j[0].origin;
                let o2 = j[1].origin;
                let inner = o2.sub(rotate(j[1].direction, t2, o2)).sub(o1);
                o1.add(rotate(j[0].direction, t1, inner))
            }
            ChainKind::HeadTable => {
                let o = j[0].origin;
                o.sub(rotate(j[0].direction, angles_deg[0].to_radians(), o))
            }
        }
    }

    /// Forward kinematics: the tip and unit tool axis in S of a pose.
    pub fn forward(&self, pose: &Pose, options: &KinematicsOptions) -> Result<(Point3, Point3), KinematicsError> {
        self.check_angles(&pose.angles_deg)?;
        if !pose.linear.is_finite() {
            return Err(KinematicsError::InvalidInput("linear position is not finite".into()));
        }
        let axis_w = self.tool_axis_world(&pose.angles_deg);
        let axis = self.world_dir_to_part(&pose.angles_deg, axis_w);
        let tip = if options.tcp {
            pose.linear
        } else {
            let tip_w = pose.linear.add(self.head_offset(&pose.angles_deg));
            self.world_to_part(&pose.angles_deg, tip_w)
        };
        Ok((tip, axis))
    }

    /// W direction to the part frame (rotation only).
    pub fn world_dir_to_part(&self, angles_deg: &[f64], v: Point3) -> Point3 {
        let j = &self.joints;
        match self.kind {
            ChainKind::Mill3 | ChainKind::HeadHead => v,
            ChainKind::Mill4 => rotate(j[0].direction, -angles_deg[0].to_radians(), v),
            ChainKind::TableTable => {
                let outer = rotate(j[0].direction, -angles_deg[0].to_radians(), v);
                rotate(j[1].direction, -angles_deg[1].to_radians(), outer)
            }
            ChainKind::HeadTable => rotate(j[1].direction, -angles_deg[1].to_radians(), v),
        }
    }

    /// Linear axis words for a tip in S at the given angles.
    pub fn linear_for(&self, tip: Point3, angles_deg: &[f64], options: &KinematicsOptions) -> Point3 {
        if options.tcp {
            tip
        } else {
            self.part_to_world(angles_deg, tip).sub(self.head_offset(angles_deg))
        }
    }

    /// Inverse kinematics: every joint configuration (ignoring limits, angles
    /// in `(-180, 180]`) that puts the tool along `axis` at `tip`.
    /// `previous` holds free angles in singular configurations (zero when
    /// `None`). Returns an empty vector only for a 3-axis chain whose tool
    /// axis is not `+Z` (reported as `Unreachable`).
    pub fn inverse(
        &self,
        tip: Point3,
        axis: Option<Point3>,
        previous: Option<&[f64]>,
        options: &KinematicsOptions,
    ) -> Result<Vec<Pose>, KinematicsError> {
        options.validate()?;
        if !tip.is_finite() {
            return Err(KinematicsError::InvalidInput("tool tip is not finite".into()));
        }
        let a = match axis {
            None => Point3::Z,
            Some(v) => v.normalized().ok_or(KinematicsError::InvalidAxis)?,
        };
        let prev: Vec<f64> = match previous {
            Some(p) => {
                self.check_angles(p)?;
                p.iter().map(|d| d.to_radians()).collect()
            }
            None => vec![0.0; self.joints.len()],
        };
        let cone = options.singular_cone_deg.to_radians();
        let eps = options.cos_eps;
        let unreachable = || KinematicsError::Unreachable { axis: a };
        let j = &self.joints;
        let mut raw: Vec<(Vec<f64>, bool)> = Vec::new();
        match self.kind {
            ChainKind::Mill3 => {
                if 1.0 - a.dot(Point3::Z) > eps {
                    return Err(unreachable());
                }
                raw.push((Vec::new(), false));
            }
            ChainKind::Mill4 => {
                let (theta, singular) =
                    one_axis(j[0].direction, a, Point3::Z, prev[0], cone, eps).ok_or_else(unreachable)?;
                raw.push((vec![theta], singular));
            }
            ChainKind::TableTable => {
                // R_P(θ1) R_Sec(θ2) a = Z: first the secondary, then the primary.
                for s in two_axis(j[1].direction, j[0].direction, a, Point3::Z, [prev[1], prev[0]], cone, eps) {
                    raw.push((vec![s.beta, s.alpha], s.singular));
                }
            }
            ChainKind::HeadHead => {
                // R_P(θ1) R_Sec(θ2) Z = a.
                for s in two_axis(j[1].direction, j[0].direction, Point3::Z, a, [prev[1], prev[0]], cone, eps) {
                    raw.push((vec![s.beta, s.alpha], s.singular));
                }
            }
            ChainKind::HeadTable => {
                // R_H(θh) Z = R_T(θt) a  ⇔  R_T(−θt) R_H(θh) Z = a.
                for s in two_axis(j[0].direction, j[1].direction, Point3::Z, a, [prev[0], -prev[1]], cone, eps) {
                    raw.push((vec![s.alpha, -s.beta], s.singular));
                }
            }
        }
        if raw.is_empty() {
            return Err(unreachable());
        }
        let mut poses = Vec::with_capacity(raw.len());
        for (branch, (angles, singular)) in raw.into_iter().enumerate() {
            let angles_deg: Vec<f64> = angles.iter().map(|r| wrap_deg(r.to_degrees())).collect();
            let linear = self.linear_for(tip, &angles_deg, options);
            poses.push(Pose { linear, angles_deg, branch: branch as u8, singular });
        }
        Ok(poses)
    }
}

