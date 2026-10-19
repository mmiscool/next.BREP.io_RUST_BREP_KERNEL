//! Resolving the scan's UNDECIDED candidates on the exact carriers (lane K2
//! row scan-undecided, 2026-10-16). A candidate the mesh confirmation cannot
//! decide (its witness is not a point both trims own, or the pierce lane could
//! not evaluate a projection or a normal) is asked again of the carriers
//! themselves: points of the two carriers' common curve near the mesh
//! crossing, found by Newton on both surfaces at once, are classified by both
//! trims at their OWN parameters (no fresh projection). A point both faces own
//! where the carriers cross transversally, straddled past the band in both
//! directions, confirms a crossing; points that all lie on an edge the two
//! faces share, or outside a trim, clear it. Anything else stays undecided,
//! and the budget is bounded: `SEEDS` points per candidate, `ITERATIONS`
//! Newton steps per point, `MAX_RESOLVED` candidates per scan.
use super::*;
use crate::projection::project_point_to_curve;

/// On by default (2026-10-16); `BREP_SOUNDNESS_RESOLVE=0` (any value but
/// 1/on) restores the scan without resolution, for an on/off census, and
/// keeps the acceptance from refusing on an undecided scan.
pub(crate) fn enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| match std::env::var("BREP_SOUNDNESS_RESOLVE") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    })
}

/// Off by default (2026-10-16): `BREP_SOUNDNESS_REFUSE_UNDECIDED=1|on` makes
/// the acceptance refuse a result whose scan stays undecided after resolution
/// (`accept_refuses_undecided_crossing_20261016`). Default-off until the lib
/// battery reads no correct body refused: a near-tangent contact (a sphere
/// tangent family) still ends undecided there. The report lists them either
/// way.
pub(crate) fn refuse_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| matches!(std::env::var("BREP_SOUNDNESS_REFUSE_UNDECIDED").as_deref(), Ok("1") | Ok("on")))
}

/// Seeds along the mesh crossing segment (both ends included).
pub(super) const SEEDS: usize = 5;
/// Newton steps per seed.
const ITERATIONS: usize = 24;
/// Candidates resolved per scan; the rest stay undecided.
pub(super) const MAX_RESOLVED: usize = 256;
/// Smallest sine between the carriers' normals read as a transversal crossing.
const MIN_SINE: f64 = 1e-3;

/// Why the mesh confirmation left a candidate undecided.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CrossingUndecidedCause {
    /// The crossing's witness is not a point both faces own.
    Witness,
    /// The pierce lane could not evaluate a projection or a normal.
    Pierce,
    /// A triangle's face index is not in the body.
    MissingFace,
}

/// One candidate the scan could not decide, after resolution.
#[derive(Clone, Debug, Serialize)]
pub struct UndecidedCrossing {
    pub face_a: u64,
    pub face_b: u64,
    pub cause: CrossingUndecidedCause,
    /// Midpoint of the mesh crossing segment.
    pub point: Vec3,
}

/// What the carriers say about one candidate.
pub(super) enum Resolution {
    /// A transversal crossing at a point both faces own, with its straddle.
    Crossing { point: Vec3, straddle: f64 },
    /// The carriers meet only on a shared edge or outside a trim here.
    Clear,
    Undecided,
}

/// One common point of the two carriers: `(point, (ua, va), (ub, vb))`.
type Common = (Vec3, (f64, f64), (f64, f64));

/// Minimum-norm Newton on `S_a(ua, va) = S_b(ub, vb)` from both feet of `seed`.
fn common_point(a: &NurbsSurface, b: &NurbsSurface, seed: Vec3, tolerance: f64, reach: f64) -> Option<Common> {
    let (fa, fb) = (project_point_to_surface(a, seed).ok()?, project_point_to_surface(b, seed).ok()?);
    let (da_u, da_v) = (a.domain_u().ok()?, a.domain_v().ok()?);
    let (db_u, db_v) = (b.domain_u().ok()?, b.domain_v().ok()?);
    let (mut ua, mut va, mut ub, mut vb) = (fa.u, fa.v, fb.u, fb.v);
    for _ in 0..ITERATIONS {
        let (sa, sb) = (a.derivatives(ua, va, 1).ok()?, b.derivatives(ub, vb, 1).ok()?);
        let residual = sa[0][0].sub(sb[0][0]);
        if residual.length() <= tolerance {
            let point = sa[0][0].add(sb[0][0]).scale(0.5);
            return (point.sub(seed).length() <= reach).then_some((point, (ua, va), (ub, vb)));
        }
        // J = [Sa_u, Sa_v, -Sb_u, -Sb_v] (3x4); step = -J^T (J J^T)^-1 r.
        let columns = [sa[1][0], sa[0][1], sb[1][0].scale(-1.), sb[0][1].scale(-1.)];
        let mut m = [[0.0f64; 3]; 3];
        for c in &columns {
            let v = [c.x, c.y, c.z];
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] += v[i] * v[j];
                }
            }
        }
        let lambda = solve3(m, [-residual.x, -residual.y, -residual.z])?;
        let lambda = Vec3::new(lambda[0], lambda[1], lambda[2]);
        let step = columns.map(|c| c.dot(lambda));
        ua = (ua + step[0]).clamp(da_u[0], da_u[1]);
        va = (va + step[1]).clamp(da_v[0], da_v[1]);
        ub = (ub + step[2]).clamp(db_u[0], db_u[1]);
        vb = (vb + step[3]).clamp(db_v[0], db_v[1]);
    }
    None
}

/// Cramer's rule on a 3x3 system; `None` when it is singular.
fn solve3(m: [[f64; 3]; 3], r: [f64; 3]) -> Option<[f64; 3]> {
    let det = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(m);
    if !(d.abs() > 1e-300) || !d.is_finite() {
        return None;
    }
    let mut out = [0.; 3];
    for (col, slot) in out.iter_mut().enumerate() {
        let mut k = m;
        for row in 0..3 {
            k[row][col] = r[row];
        }
        *slot = det(k) / d;
    }
    out.iter().all(|x| x.is_finite()).then_some(out)
}

/// Does `face` own its own parameter `(u, v)`? `None` when the trim test fails.
fn owns(face: &FaceRecord, (u, v): (f64, f64)) -> Option<PolygonClass> {
    parameter_point_in_face(face, Vec2 { x: u, y: v }, TRIM_SAMPLE_TOLERANCE).ok()
}

/// Straddle of `face` across `other`'s carrier at its own parameter `at`,
/// stepping `h` (world) each way across the common curve: the smaller of the
/// two one-sided distances when they have opposite signs and both points are
/// the face's own material, else `None`.
fn straddle_across(face: &FaceRecord, at: (f64, f64), across: Vec3, other: &NurbsSurface, h: f64) -> Option<f64> {
    let d = face.surface.derivatives(at.0, at.1, 1).ok()?;
    let (su, sv) = (d[1][0], d[0][1]);
    // Least squares (du, dv) with su du + sv dv = across.
    let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let det = a * c - b * b;
    if !(det > 0.) {
        return None;
    }
    let (p, q) = (su.dot(across), sv.dot(across));
    let (du, dv) = ((c * p - b * q) / det, (a * q - b * p) / det);
    let scale = h / su.scale(du).add(sv.scale(dv)).length();
    if !scale.is_finite() {
        return None;
    }
    let mut sides = [0.; 2];
    for (slot, sign) in sides.iter_mut().zip([1., -1.]) {
        let (u, v) = (at.0 + sign * scale * du, at.1 + sign * scale * dv);
        if !matches!(owns(face, (u, v))?, PolygonClass::Inside) {
            return None;
        }
        *slot = signed_distance(other, face.surface.evaluate(u, v).ok()?)?;
    }
    (sides[0] * sides[1] < 0.).then(|| sides[0].abs().min(sides[1].abs()))
}

/// Resolve one candidate: `segment` is the mesh crossing, `shared` the curves
/// of the edges both faces use, `corners` the vertices both faces use.
pub(super) fn resolve(
    face_a: &FaceRecord,
    face_b: &FaceRecord,
    shared: &[&NurbsCurve],
    corners: &[Vec3],
    segment: (Vec3, Vec3),
    options: SelfIntersectionOptions,
) -> Resolution {
    let band = options.straddle_band;
    let reach = 4. * options.chord_tolerance + segment.1.sub(segment.0).length();
    let mut all_clear = true;
    for i in 0..SEEDS {
        let t = i as f64 / (SEEDS - 1) as f64;
        let seed = segment.0.add(segment.1.sub(segment.0).scale(t));
        // Converged to the body's weld scale, or to what the coordinates'
        // own precision can resolve where they are large (far from the origin
        // a weld-scale residual is below one ulp).
        let magnitude = seed.x.abs().max(seed.y.abs()).max(seed.z.abs());
        let tolerance = (options.weld * 1e-3).max(64. * f64::EPSILON * magnitude);
        let Some((point, pa, pb)) = common_point(&face_a.surface, &face_b.surface, seed, tolerance, reach) else {
            all_clear = false;
            continue;
        };
        // On an edge both faces use: the faces meet there by construction. An
        // edge is the carriers' intersection only to its own accuracy, so the
        // carriers' true common curve can sit up to that deviation (over the
        // angle between them) off the edge, in a sliver both trims own.
        let on_shared = shared.iter().any(|curve| {
            project_point_to_curve(curve, point).is_ok_and(|foot| {
                let off = |surface: &NurbsSurface| project_point_to_surface(surface, foot.point).map_or(f64::INFINITY, |f| f.distance);
                let deviation = off(&face_a.surface) + off(&face_b.surface);
                let sine = match (face_a.surface.normal(pa.0, pa.1), face_b.surface.normal(pb.0, pb.1)) {
                    (Ok(na), Ok(nb)) => match (na.normalized(), nb.normalized()) {
                        (Ok(na), Ok(nb)) => na.cross(nb).length(),
                        _ => 0.,
                    },
                    _ => 0.,
                };
                foot.distance <= band || (deviation.is_finite() && sine > 0. && foot.distance <= band + 4. * deviation / sine)
            })
        });
        // At a vertex both faces use: they touch there by construction.
        let on_corner = corners.iter().any(|corner| corner.sub(point).length() <= band.max(tolerance));
        if on_shared || on_corner {
            continue;
        }
        let (Some(in_a), Some(in_b)) = (owns(face_a, pa), owns(face_b, pb)) else {
            all_clear = false;
            continue;
        };
        if matches!(in_a, PolygonClass::Outside) || matches!(in_b, PolygonClass::Outside) {
            continue;
        }
        all_clear = false;
        if !(matches!(in_a, PolygonClass::Inside) && matches!(in_b, PolygonClass::Inside)) {
            continue;
        }
        let (Ok(na), Ok(nb)) = (face_a.surface.normal(pa.0, pa.1), face_b.surface.normal(pb.0, pb.1)) else { continue };
        let (Ok(na), Ok(nb)) = (na.normalized(), nb.normalized()) else { continue };
        let along = na.cross(nb);
        let sine = along.length();
        if !(sine >= MIN_SINE) {
            continue;
        }
        let along = along.scale(1. / sine);
        // Step far enough that a transversal crossing clears the band twice.
        let h = 4. * band / sine;
        let (Some(forward), Some(backward)) = (
            straddle_across(face_a, pa, nb.sub(along.scale(nb.dot(along))), &face_b.surface, h),
            straddle_across(face_b, pb, na.sub(along.scale(na.dot(along))), &face_a.surface, h),
        ) else {
            continue;
        };
        if forward > band && backward > band {
            return Resolution::Crossing { point, straddle: forward.min(backward) };
        }
    }
    if all_clear { Resolution::Clear } else { Resolution::Undecided }
}

