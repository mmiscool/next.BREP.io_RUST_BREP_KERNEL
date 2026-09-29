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
/// remove 2 DOF), used for redundancy accounting.
#[derive(Clone, Copy, Debug)]
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
            | Atom::LineLineTouch(..) => 3,
            Atom::PointOnPlane(..)
            | Atom::DirDot(..)
            | Atom::PointDistance(..)
            | Atom::PointLineDistance(..)
            | Atom::LineLineDistance(..) => 1,
        }
    }

    pub(super) fn eff_rows(&self) -> usize {
        match self {
            Atom::PointsCoincide(..) => 3,
            Atom::DirMatch(..) | Atom::DirCross(..) | Atom::AxisLateral(..) => 2,
            // A satisfied touch removes 1 DOF at a skew intersection but 2 at
            // a collinear solution; report the minimum so a satisfied mate
            // never fabricates an "over" (redundancy) diagnosis.
            Atom::PointOnPlane(..)
            | Atom::DirDot(..)
            | Atom::PointDistance(..)
            | Atom::PointLineDistance(..)
            | Atom::LineLineDistance(..)
            | Atom::LineLineTouch(..) => 1,
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
        }
    }
}
