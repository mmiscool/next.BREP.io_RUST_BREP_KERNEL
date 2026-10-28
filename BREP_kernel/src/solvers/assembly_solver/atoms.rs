use super::*;

// ---------------------------------------------------------------------------
// Quaternion helpers ([w, x, y, z], unit, local → world)
// ---------------------------------------------------------------------------

pub(super) fn quat_normalize(q: [f64; 4]) -> Result<[f64; 4], String> {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if !(n.is_finite() && n > ASM_DIR_EPS) {
        return Err("rotation quaternion must be finite and nonzero".into());
    }
    Ok([q[0] / n, q[1] / n, q[2] / n, q[3] / n])
}

pub(super) fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

pub(super) fn quat_rotate(q: [f64; 4], v: Vec3) -> Vec3 {
    let u = Vec3::new(q[1], q[2], q[3]);
    let uv = u.cross(v);
    let uuv = u.cross(uv);
    v.add(uv.scale(2.0 * q[0])).add(uuv.scale(2.0))
}

/// Exponential map: rotation vector → unit quaternion. Only ever called with
/// increments near zero (the solver re-centres every step), so the small-angle
/// series keeps full accuracy and the ‖θ‖ = π singularity is never approached.
pub(super) fn quat_from_rotvec(w: Vec3) -> [f64; 4] {
    let a = w.length();
    let half = 0.5 * a;
    let (c, k) = if a > 1e-8 {
        (half.cos(), half.sin() / a)
    } else {
        (1.0 - half * half / 2.0, 0.5 - a * a / 48.0)
    };
    [c, k * w.x, k * w.y, k * w.z]
}

// ---------------------------------------------------------------------------
// Residual atoms
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub(super) struct Pose {
    pub(super) q: [f64; 4],
    pub(super) t: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct LocalPoint {
    pub(super) body: usize,
    pub(super) p: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct LocalDir {
    pub(super) body: usize,
    pub(super) d: Vec3, // unit
}

fn world_point(poses: &[Pose], lp: LocalPoint) -> Vec3 {
    quat_rotate(poses[lp.body].q, lp.p).add(poses[lp.body].t)
}

fn world_dir(poses: &[Pose], ld: LocalDir) -> Vec3 {
    quat_rotate(poses[ld.body].q, ld.d)
}

/// One residual atom contributing rows to the stacked residual vector.
/// `rows()` is the number of scalar rows emitted; `eff_rows()` is the atom's
/// theoretical rank at a solution (a 3-row unit-direction block can only ever
/// remove 2 DOF), used for redundancy accounting. Not `Copy`: a curve atom
/// shares its sampled curve.
#[derive(Clone, Debug)]
pub(super) enum Atom {
    /// wa − wb (3 rows, eff 3).
    PointsCoincide(LocalPoint, LocalPoint),
    /// dot(n_w, p_w − o_w) − target (1 row).
    PointOnPlane(LocalPoint, LocalPoint, LocalDir, f64),
    /// da_w − sign·db_w (3 rows, eff 2).
    DirMatch(LocalDir, LocalDir, f64),
    /// da_w × db_w (3 rows, eff 2) — either alignment sense.
    DirCross(LocalDir, LocalDir),
    /// dot(da_w, db_w) − target (1 row).
    DirDot(LocalDir, LocalDir, f64),
    /// Wrapped signed angular difference around a body-local reference axis.
    DirAngle(LocalDir, LocalDir, LocalDir, f64),
    /// ‖wa − wb‖ − target (1 row).
    PointDistance(LocalPoint, LocalPoint, f64),
    /// Component of (p_w − o_w) perpendicular to axis dir (3 rows, eff 2).
    AxisLateral(LocalPoint, LocalPoint, LocalDir),
    /// ‖(p_w − o_w) ⊥ d_w‖ − target (1 row) — point-to-infinite-line
    /// distance.
    PointLineDistance(LocalPoint, LocalPoint, LocalDir, f64),
    /// Blended skew/parallel line-line closest approach − target (1 row):
    /// `(origin_a, dir_a, origin_b, dir_b, target)`. See `line_line_frame`
    /// for the C1 parallel-degeneracy blend.
    LineLineDistance(LocalPoint, LocalDir, LocalPoint, LocalDir, f64),
    /// Zero-distance line-line touch (3 rows, eff 1): C1-blends the signed
    /// mutual-perpendicular offset vector `(u·ĉ)ĉ` (skew: lines intersect)
    /// into the full lateral vector of A's origin about line B (parallel:
    /// lines collinear), avoiding the ‖·‖ − 0 kink a scalar residual would
    /// have at the solution.
    LineLineTouch(LocalPoint, LocalDir, LocalPoint, LocalDir),
    /// A limit row (1 row, eff 0): zero while `measure` is inside
    /// `[lo, hi]`, else the signed distance to the nearer end. A circular
    /// measure (`DirAngle`, radians) runs from `lo` counter-clockwise to `hi`.
    Limit(Measure, f64, f64),
    /// Mirror atoms: the trailing `(LocalPoint, LocalDir)` is the mirror plane
    /// (origin, unit normal) and `R` reflects across it.
    /// R(pa) − pb (3 rows, eff 3).
    SymPoints(LocalPoint, LocalPoint, LocalPoint, LocalDir),
    /// R(da) − db for directions (3 rows, eff 2).
    SymDirMatch(LocalDir, LocalDir, LocalDir),
    /// R(da) × db — mirrored directions parallel in either sense (3 rows,
    /// eff 2).
    SymDirCross(LocalDir, LocalDir, LocalDir),
    /// (pb − R(oa))·R(na): point B on plane A's mirror image (1 row).
    SymPointOnPlane(LocalPoint, LocalPoint, LocalDir, LocalPoint, LocalDir),
    /// The part of R(oa) − ob across line B (`ob`, `db`): A's mirrored axis
    /// point on B's line (3 rows, eff 2).
    SymAxisLateral(LocalPoint, LocalPoint, LocalDir, LocalPoint, LocalDir),
    /// `ka·θa + kb·θb` for two spins (1 row) — the gear coupling.
    GearPhase(Spin, Spin, f64, f64),
    /// Rack travel minus `k·θ` (1 row): `(rack point − world point)·rack
    /// direction − k·θ(pinion)`.
    RackTravel(Spin, LocalPoint, LocalDir, Vec3, f64),
    /// Screw advance (1 row): `((pb − oa)·da − offset0) − k·wrap(θ − angle0)`
    /// with `θ` the angle about `da` from A's reference to B's:
    /// `(oa, da, ref_a, ref_b, pb, angle0, offset0, k)`.
    ScrewAdvance(LocalPoint, LocalDir, LocalDir, LocalDir, LocalPoint, f64, f64, f64),
    /// World `p − C(t*)` for the curve's closest point (3 rows, eff 2).
    PointOnCurve(LocalPoint, CurveRef),
    /// The axis line `(point, direction)` through the curve: the part of
    /// `p − C(t*)` across the axis, `t*` the closest curve point to the
    /// axis's crossing of the curve's plane (3 rows, eff 2).
    AxisOnCurve(LocalPoint, LocalDir, CurveRef),
    /// Where the closest point sits along the curve, minus a target fraction
    /// (1 row): `(point, projection axis, curve, target, reverse)` — with an
    /// axis, the point is first slid along it onto the curve's plane.
    CurveAlong(LocalPoint, Option<LocalDir>, CurveRef, f64, bool),
    /// Cam contact (1 row): `(follower centre, roller axis, profile, profile
    /// normal, radius, side)` — the signed in-plane distance from the
    /// follower to the profile on `side`, minus the radius.
    CamContact(LocalPoint, Option<LocalDir>, CurveRef, LocalDir, f64, f64),
}

/// A body's spin about its own axis, read against a fixed WORLD reference
/// direction: `(local reference ⟂ axis, local axis, world reference)`.
#[derive(Clone, Copy, Debug)]
pub(super) struct Spin {
    pub(super) reference: LocalDir,
    pub(super) axis: LocalDir,
    pub(super) world: Vec3,
}

impl Spin {
    fn angle(&self, poses: &[Pose]) -> f64 {
        signed_angle(self.world, world_dir(poses, self.reference), world_dir(poses, self.axis))
    }
}

/// The signed angle about unit `n` from `from` to `to` (both projected ⟂ n).
pub(super) fn signed_angle(from: Vec3, to: Vec3, n: Vec3) -> f64 {
    let from = from.sub(n.scale(from.dot(n)));
    let to = to.sub(n.scale(to.dot(n)));
    n.dot(from.cross(to)).atan2(from.dot(to))
}

/// Wrap an angle into (−π, π].
fn wrap_angle(angle: f64) -> f64 {
    angle.sin().atan2(angle.cos())
}

/// Reflect a world point across the plane through `o` with unit normal `n`.
fn mirror_point(p: Vec3, o: Vec3, n: Vec3) -> Vec3 {
    p.sub(n.scale(2.0 * p.sub(o).dot(n)))
}

/// Reflect a world direction across a plane with unit normal `n`.
fn mirror_dir(d: Vec3, n: Vec3) -> Vec3 {
    d.sub(n.scale(2.0 * d.dot(n)))
}

/// The quantity a limit row reads: a one-row atom with its target removed.
/// `DirDot` reads the cosine, `DirAngle` the wrapped signed angle (radians).
#[derive(Clone, Copy, Debug)]
pub(super) enum Measure {
    PointOnPlane(LocalPoint, LocalPoint, LocalDir),
    DirDot(LocalDir, LocalDir),
    DirAngle(LocalDir, LocalDir, LocalDir),
    PointDistance(LocalPoint, LocalPoint),
    PointLineDistance(LocalPoint, LocalPoint, LocalDir),
    LineLineDistance(LocalPoint, LocalDir, LocalPoint, LocalDir),
}

impl Measure {
    /// The measure of a target-carrying atom (the zero-target degradations
    /// map back to the distance they stand for); `None` for an atom that
    /// measures nothing.
    pub(super) fn of(atom: &Atom) -> Option<Self> {
        Some(match *atom {
            Atom::PointOnPlane(p, o, n, _) => Measure::PointOnPlane(p, o, n),
            Atom::DirDot(a, b, _) => Measure::DirDot(a, b),
            Atom::DirAngle(a, b, axis, _) => Measure::DirAngle(a, b, axis),
            Atom::PointDistance(a, b, _) | Atom::PointsCoincide(a, b) => Measure::PointDistance(a, b),
            Atom::PointLineDistance(p, o, d, _) | Atom::AxisLateral(p, o, d) => {
                Measure::PointLineDistance(p, o, d)
            }
            Atom::LineLineDistance(oa, da, ob, db, _) | Atom::LineLineTouch(oa, da, ob, db) => {
                Measure::LineLineDistance(oa, da, ob, db)
            }
            _ => return None,
        })
    }

    fn circular(&self) -> bool {
        matches!(self, Measure::DirAngle(..))
    }

    fn value(&self, poses: &[Pose]) -> f64 {
        let atom = match *self {
            Measure::PointOnPlane(p, o, n) => Atom::PointOnPlane(p, o, n, 0.0),
            Measure::DirDot(a, b) => Atom::DirDot(a, b, 0.0),
            Measure::DirAngle(a, b, axis) => Atom::DirAngle(a, b, axis, 0.0),
            Measure::PointDistance(a, b) => Atom::PointDistance(a, b, 0.0),
            Measure::PointLineDistance(p, o, d) => Atom::PointLineDistance(p, o, d, 0.0),
            Measure::LineLineDistance(oa, da, ob, db) => Atom::LineLineDistance(oa, da, ob, db, 0.0),
        };
        let mut out = Vec::with_capacity(1);
        atom.eval(poses, &mut out);
        out[0]
    }
}

/// The limit row for measured value `m` against `[lo, hi]`.
pub(super) fn limit_residual(m: f64, lo: f64, hi: f64, circular: bool) -> f64 {
    if circular {
        let turn = std::f64::consts::TAU;
        let rel = (m - lo).rem_euclid(turn);
        let span = (hi - lo).rem_euclid(turn);
        if rel <= span {
            return 0.0;
        }
        let past_hi = rel - span;
        let short_of_lo = turn - rel;
        if past_hi <= short_of_lo { past_hi } else { -short_of_lo }
    } else if m < lo {
        m - lo
    } else if m > hi {
        m - hi
    } else {
        0.0
    }
}

/// C1 skew weight for the line-line parallel-degeneracy switch: 0 at
/// `sin ≤ ASM_LL_SIN_PARALLEL` (pure parallel measure), 1 at
/// `sin ≥ ASM_LL_SIN_SKEW` (pure closest approach), smoothstep between
/// (zero derivative at both band edges, hence C1 overall).
fn line_line_skew_weight(sin_angle: f64) -> f64 {
    let x = (sin_angle - ASM_LL_SIN_PARALLEL) / (ASM_LL_SIN_SKEW - ASM_LL_SIN_PARALLEL);
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Shared world-frame quantities of the two line-line atoms.
struct LineLineFrame {
    /// `origin_a − origin_b`, world.
    u: Vec3,
    /// `dir_a × dir_b`, world (‖c‖ = sine of the angle: unit directions).
    c: Vec3,
    /// `‖c‖²`.
    s2: f64,
    /// Component of `u` perpendicular to `dir_b` — the parallel-branch
    /// lateral vector (its norm is the point-to-line distance of A's origin
    /// about line B).
    v: Vec3,
    /// C1 skew weight (0 = parallel branch, 1 = skew branch).
    beta: f64,
}

fn line_line_frame(
    poses: &[Pose],
    oa: LocalPoint,
    da: LocalDir,
    ob: LocalPoint,
    db: LocalDir,
) -> LineLineFrame {
    let woa = world_point(poses, oa);
    let wda = world_dir(poses, da); // unit: rotation preserves norms
    let wob = world_point(poses, ob);
    let wdb = world_dir(poses, db);
    let u = woa.sub(wob);
    let c = wda.cross(wdb);
    let s2 = c.dot(c);
    let v = u.sub(wdb.scale(wdb.dot(u)));
    let beta = line_line_skew_weight(s2.sqrt());
    LineLineFrame { u, c, s2, v, beta }
}

impl Atom {
    pub(super) fn rows(&self) -> usize {
        match self {
            Atom::PointsCoincide(..)
            | Atom::DirMatch(..)
            | Atom::DirCross(..)
            | Atom::AxisLateral(..)
            | Atom::LineLineTouch(..)
            | Atom::SymPoints(..)
            | Atom::SymDirMatch(..)
            | Atom::SymDirCross(..)
            | Atom::SymAxisLateral(..)
            | Atom::PointOnCurve(..)
            | Atom::AxisOnCurve(..) => 3,
            Atom::PointOnPlane(..)
            | Atom::DirDot(..)
            | Atom::DirAngle(..)
            | Atom::PointDistance(..)
            | Atom::PointLineDistance(..)
            | Atom::LineLineDistance(..)
            | Atom::Limit(..)
            | Atom::SymPointOnPlane(..)
            | Atom::GearPhase(..)
            | Atom::RackTravel(..)
            | Atom::ScrewAdvance(..)
            | Atom::CurveAlong(..)
            | Atom::CamContact(..) => 1,
        }
    }

    pub(super) fn eff_rows(&self) -> usize {
        match self {
            Atom::PointsCoincide(..) | Atom::SymPoints(..) => 3,
            Atom::DirMatch(..)
            | Atom::DirCross(..)
            | Atom::AxisLateral(..)
            | Atom::SymDirMatch(..)
            | Atom::SymDirCross(..)
            | Atom::SymAxisLateral(..)
            | Atom::PointOnCurve(..)
            | Atom::AxisOnCurve(..) => 2,
            Atom::CurveAlong(..) | Atom::CamContact(..) => 1,
            Atom::SymPointOnPlane(..)
            | Atom::GearPhase(..)
            | Atom::RackTravel(..)
            | Atom::ScrewAdvance(..) => 1,
            // A satisfied touch removes 1 DOF at a skew intersection but 2 at
            // a collinear solution; report the minimum so a satisfied mate
            // never fabricates an "over" (redundancy) diagnosis.
            Atom::PointOnPlane(..)
            | Atom::DirDot(..)
            | Atom::DirAngle(..)
            | Atom::PointDistance(..)
            | Atom::PointLineDistance(..)
            | Atom::LineLineDistance(..)
            | Atom::LineLineTouch(..) => 1,
            // Inside its range a limit binds nothing; at an end it binds what
            // its measure binds, which the rank sees without being counted.
            Atom::Limit(..) => 0,
        }
    }

    pub(super) fn eval(&self, poses: &[Pose], out: &mut Vec<f64>) {
        match *self {
            Atom::PointsCoincide(a, b) => {
                let wa = world_point(poses, a);
                let wb = world_point(poses, b);
                out.extend([wa.x - wb.x, wa.y - wb.y, wa.z - wb.z]);
            }
            Atom::PointOnPlane(p, o, n, target) => {
                let wp = world_point(poses, p);
                let wo = world_point(poses, o);
                let wn = world_dir(poses, n);
                out.push(wn.dot(wp.sub(wo)) - target);
            }
            Atom::DirMatch(a, b, sign) => {
                let da = world_dir(poses, a);
                let db = world_dir(poses, b);
                out.extend([da.x - sign * db.x, da.y - sign * db.y, da.z - sign * db.z]);
            }
            Atom::DirCross(a, b) => {
                let da = world_dir(poses, a);
                let db = world_dir(poses, b);
                let c = da.cross(db);
                out.extend([c.x, c.y, c.z]);
            }
            Atom::DirDot(a, b, target) => {
                let da = world_dir(poses, a);
                let db = world_dir(poses, b);
                out.push(da.dot(db) - target);
            }
            Atom::DirAngle(a, b, axis, target) => {
                let da = world_dir(poses, a);
                let db = world_dir(poses, b);
                let n = world_dir(poses, axis);
                let delta = n.dot(da.cross(db)).atan2(da.dot(db)) - target;
                out.push(delta.sin().atan2(delta.cos()));
            }
            Atom::PointDistance(a, b, target) => {
                let wa = world_point(poses, a);
                let wb = world_point(poses, b);
                out.push(wa.sub(wb).length() - target);
            }
            Atom::AxisLateral(p, o, d) => {
                let wp = world_point(poses, p);
                let wo = world_point(poses, o);
                let wd = world_dir(poses, d); // unit: rotation preserves norms
                let v = wp.sub(wo);
                let lateral = v.sub(wd.scale(wd.dot(v)));
                out.extend([lateral.x, lateral.y, lateral.z]);
            }
            Atom::PointLineDistance(p, o, d, target) => {
                let wp = world_point(poses, p);
                let wo = world_point(poses, o);
                let wd = world_dir(poses, d); // unit: rotation preserves norms
                let v = wp.sub(wo);
                let lateral = v.sub(wd.scale(wd.dot(v)));
                out.push(lateral.length() - target);
            }
            Atom::LineLineDistance(oa, da, ob, db, target) => {
                let f = line_line_frame(poses, oa, da, ob, db);
                let d_par2 = f.v.dot(f.v);
                let d2 = if f.beta > 0.0 {
                    // Skew closest approach squared: (u·c)²/‖c‖² — well
                    // conditioned here because beta > 0 implies sin > LO.
                    let triple = f.u.dot(f.c);
                    f.beta * (triple * triple / f.s2) + (1.0 - f.beta) * d_par2
                } else {
                    // sin ≤ LO: the skew term is 0/0 and its C1 weight is
                    // exactly 0 — branch instead of multiplying NaN by zero.
                    d_par2
                };
                out.push(d2.sqrt() - target);
            }
            Atom::LineLineTouch(oa, da, ob, db) => {
                let f = line_line_frame(poses, oa, da, ob, db);
                let r = if f.beta > 0.0 {
                    // Signed mutual-perpendicular offset (u·ĉ)ĉ blended into
                    // the lateral vector; both live in the plane ⊥ dir_b, so
                    // the blend is a consistent 2D interpolation.
                    let c_hat = f.c.scale(1.0 / f.s2.sqrt());
                    c_hat
                        .scale(f.beta * f.u.dot(c_hat))
                        .add(f.v.scale(1.0 - f.beta))
                } else {
                    f.v
                };
                out.extend([r.x, r.y, r.z]);
            }
            Atom::Limit(measure, lo, hi) => {
                out.push(limit_residual(measure.value(poses), lo, hi, measure.circular()));
            }
            Atom::SymPoints(a, b, o, n) => {
                let r = mirror_point(world_point(poses, a), world_point(poses, o), world_dir(poses, n))
                    .sub(world_point(poses, b));
                out.extend([r.x, r.y, r.z]);
            }
            Atom::SymDirMatch(a, b, n) => {
                let r = mirror_dir(world_dir(poses, a), world_dir(poses, n)).sub(world_dir(poses, b));
                out.extend([r.x, r.y, r.z]);
            }
            Atom::SymDirCross(a, b, n) => {
                let r = mirror_dir(world_dir(poses, a), world_dir(poses, n)).cross(world_dir(poses, b));
                out.extend([r.x, r.y, r.z]);
            }
            Atom::SymPointOnPlane(pb, oa, na, o, n) => {
                let (wo, wn) = (world_point(poses, o), world_dir(poses, n));
                let origin = mirror_point(world_point(poses, oa), wo, wn);
                let normal = mirror_dir(world_dir(poses, na), wn);
                out.push(world_point(poses, pb).sub(origin).dot(normal));
            }
            Atom::GearPhase(a, b, ka, kb) => {
                out.push(ka * a.angle(poses) + kb * b.angle(poses));
            }
            Atom::RackTravel(pinion, p, d, origin, k) => {
                let travel = world_point(poses, p).sub(origin).dot(world_dir(poses, d));
                out.push(travel - k * pinion.angle(poses));
            }
            Atom::ScrewAdvance(oa, da, ref_a, ref_b, pb, angle0, offset0, k) => {
                let n = world_dir(poses, da);
                let height = world_point(poses, pb).sub(world_point(poses, oa)).dot(n);
                let turn = signed_angle(world_dir(poses, ref_a), world_dir(poses, ref_b), n);
                out.push((height - offset0) - k * wrap_angle(turn - angle0));
            }
            Atom::PointOnCurve(p, ref curve) => eval_point_on_curve(poses, p, curve, out),
            Atom::AxisOnCurve(p, d, ref curve) => eval_axis_on_curve(poses, p, d, curve, out),
            Atom::CamContact(p, axis, ref curve, normal, radius, side) => {
                eval_cam_contact(poses, p, axis, curve, normal, radius, side, out)
            }
            Atom::CurveAlong(p, d, ref curve, target, reverse) => {
                eval_curve_along(poses, p, d, curve, target, reverse, out)
            }
            Atom::SymAxisLateral(oa, ob, db, o, n) => {
                let v = mirror_point(world_point(poses, oa), world_point(poses, o), world_dir(poses, n))
                    .sub(world_point(poses, ob));
                let d = world_dir(poses, db);
                let lateral = v.sub(d.scale(d.dot(v)));
                out.extend([lateral.x, lateral.y, lateral.z]);
            }
        }
    }
}
