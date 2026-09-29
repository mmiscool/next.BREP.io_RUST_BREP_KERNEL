use crate::curve::interior_knot_count;
use crate::fit::solve_small;
use crate::{project_point_to_surface, KnotVector, NurbsSurface, Vec3};
use serde::{Deserialize, Serialize};

const EPSILON: f64 = 1e-12;
const LINEAR_TOLERANCE: f64 = 1e-7;

/// Minimum `|n_a × n_b|` at a start point for a trace to begin there — the
/// gate `intersect_surfaces` applies to explicit seeds and grid starts alike,
/// and `intersect_surfaces_supplemental` to its own seeds.
///
/// Below it the carriers are tangent there to working precision, so the point
/// sits in a tangency BAND — the set of points within the coincidence
/// tolerance of both surfaces, about `√(2·R·tol)` wide across a contact of
/// curvature radius `R` — rather than on a transverse curve.  A trace started
/// inside the band walks the band's noise: it neither closes nor reaches a
/// boundary and exhausts its step budget (a corner blend's sphere kissing a
/// mirrored copy's face plane at one point; the same sphere inscribed in its
/// own fillet cylinder, kissing it along a circle).  Shared topology at a
/// tangential contact comes from imprint's boundary-curve exchange, never
/// from marching ("similar faces we do not intersect" — Golovanov §6.2).
const TRANSVERSE_START_CROSS: f64 = 1e-2;

/// [`TRANSVERSE_START_CROSS`] under its crate-wide name: the bound at which a
/// contact stops being a graze and becomes a crossing. `csg::imprint`'s
/// tangential-only gate asks the same question of a marched point, and must ask
/// it with the same number the seeds were accepted by.
pub(crate) const TRANSVERSE_SEED_CROSS: f64 = TRANSVERSE_START_CROSS;

/// How much CURVE a trace must be able to reach, as a multiple of the
/// carriers' shared extent, whatever step ceiling the caller asked for.
///
/// `maximum_steps` bounds corrector CALLS; multiplied by the ceiling in force
/// it bounds the ARC a trace can reach, and that product is the only thing
/// that decides whether an ordinary section is reachable at all.  At the
/// marcher's own ceiling (`diagonal / 15`) the default budget reaches 266
/// extents — no bound in practice.  A caller that asks for a FINER ceiling is
/// asking for RESOLUTION, and its request silently shortened the reach in the
/// same proportion: `offset_shell`'s `0.0005 · scale` leaves 2.0 extents, less
/// than one turn of an ordinary closed section on a part that size, so a
/// perfectly marched circle died at 4000 steps (2026-09-13: the apex-cone
/// carrier against its base plane — 42.714 of curve against a 28.844 reach,
/// every step at the cap, the §4.12 curvature budget permitting 113× more).
///
/// That is the 2026-08-06 rule one level up: what a marcher can reach must be
/// a fixed fraction of the MODEL, never a function of the step the caller
/// asked for.  So the ceiling is honoured as resolution and the step COUNT is
/// derived from it — the trace takes however many steps this much curve needs.
/// Nothing that completes today changes by a single step: the derived count
/// only ever exceeds the caller's own, and only on the fine-ceiling lane.
///
/// It is a FLOOR on the reach, not a bound on how long a section may be — the
/// default lane keeps its 266 extents.  Sized for the fine-ceiling lane alone,
/// where the longest section measured over the kernel's 37 631 traces is 1.76
/// extents (the 2026-09-13 census; 4.43 is the most any capped ceiling reached
/// before this and 0.13 the least).
const MARCH_REACH_EXTENTS: f64 = 4.0;

/// Corrector calls a trace may spend: the caller's own budget, or however many
/// [`MARCH_REACH_EXTENTS`] of curve need at `maximum_step`, whichever is more.
fn step_budget(maximum_steps: usize, maximum_step: f64, diagonal: f64) -> usize {
    if !(maximum_step > 0.0) || !diagonal.is_finite() {
        return maximum_steps;
    }
    let needed = (MARCH_REACH_EXTENTS * diagonal / maximum_step).ceil();
    if !needed.is_finite() || needed <= maximum_steps as f64 {
        return maximum_steps;
    }
    needed as usize
}

#[derive(Clone, Copy)]
struct Bounds {
    minimum: Vec3,
    maximum: Vec3,
}

impl Bounds {
    fn from_surface(surface: &NurbsSurface) -> Result<Self, String> {
        let mut minimum = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut maximum = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for point in surface.control_points.iter().flatten() {
            let point = point.point()?;
            minimum.x = minimum.x.min(point.x);
            minimum.y = minimum.y.min(point.y);
            minimum.z = minimum.z.min(point.z);
            maximum.x = maximum.x.max(point.x);
            maximum.y = maximum.y.max(point.y);
            maximum.z = maximum.z.max(point.z);
        }
        Ok(Self { minimum, maximum })
    }

    fn diagonal(self) -> f64 {
        self.maximum.sub(self.minimum).length()
    }

    fn expanded(self, amount: f64) -> Self {
        let delta = Vec3::new(amount, amount, amount);
        Self {
            minimum: self.minimum.sub(delta),
            maximum: self.maximum.add(delta),
        }
    }

    fn contains(self, point: Vec3) -> bool {
        point.x >= self.minimum.x
            && point.x <= self.maximum.x
            && point.y >= self.minimum.y
            && point.y <= self.maximum.y
            && point.z >= self.minimum.z
            && point.z <= self.maximum.z
    }

    fn intersects(self, other: Self, tolerance: f64) -> bool {
        self.expanded(tolerance).minimum.x <= other.maximum.x
            && self.maximum.x + tolerance >= other.minimum.x
            && self.minimum.y - tolerance <= other.maximum.y
            && self.maximum.y + tolerance >= other.minimum.y
            && self.minimum.z - tolerance <= other.maximum.z
            && self.maximum.z + tolerance >= other.minimum.z
    }
}

#[derive(Clone, Copy)]
struct SurfaceInfo<'a> {
    surface: &'a NurbsSurface,
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
    closed_u: bool,
    closed_v: bool,
}

fn surface_info(surface: &NurbsSurface) -> Result<SurfaceInfo<'_>, String> {
    let ku = KnotVector::new(surface.knots_u.clone(), surface.degree_u)?;
    let kv = KnotVector::new(surface.knots_v.clone(), surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let mut closed_u = true;
    let mut closed_v = true;
    for fraction in [0.17, 0.5, 0.83] {
        let v = v0 + (v1 - v0) * fraction;
        if surface
            .evaluate(u0, v)?
            .sub(surface.evaluate(u1, v)?)
            .length()
            > LINEAR_TOLERANCE * 10.0
        {
            closed_u = false;
        }
        let u = u0 + (u1 - u0) * fraction;
        if surface
            .evaluate(u, v0)?
            .sub(surface.evaluate(u, v1)?)
            .length()
            > LINEAR_TOLERANCE * 10.0
        {
            closed_v = false;
        }
    }
    Ok(SurfaceInfo {
        surface,
        u0,
        u1,
        v0,
        v1,
        closed_u,
        closed_v,
    })
}

fn fit(value: f64, minimum: f64, maximum: f64, closed: bool) -> f64 {
    if closed {
        (value - minimum).rem_euclid(maximum - minimum) + minimum
    } else {
        value.clamp(minimum, maximum)
    }
}


#[derive(Clone, Copy)]
struct IntersectionPoint {
    ua: f64,
    va: f64,
    ub: f64,
    vb: f64,
    point: Vec3,
}

fn refine_point(
    first: SurfaceInfo<'_>,
    second: SurfaceInfo<'_>,
    mut ua: f64,
    mut va: f64,
    mut ub: f64,
    mut vb: f64,
    tolerance: f64,
) -> Result<Option<IntersectionPoint>, String> {
    for _ in 0..40 {
        let da = first.surface.derivatives(ua, va, 1)?;
        let db = second.surface.derivatives(ub, vb, 1)?;
        let residual = da[0][0].sub(db[0][0]);
        if residual.length() <= tolerance * 0.01 + EPSILON {
            return Ok(Some(IntersectionPoint {
                ua,
                va,
                ub,
                vb,
                point: da[0][0].add(db[0][0]).scale(0.5),
            }));
        }
        let columns = [
            da[1][0],
            da[0][1],
            db[1][0].scale(-1.0),
            db[0][1].scale(-1.0),
        ];
        let mut matrix = [[0.0f64; 3]; 3];
        for column in columns {
            let values = [column.x, column.y, column.z];
            for row in 0..3 {
                for col in 0..3 {
                    matrix[row][col] += values[row] * values[col];
                }
            }
        }
        let lambda = match solve_small(matrix, [-residual.x, -residual.y, -residual.z], 3) {
            Ok(value) => value,
            Err(_) => return Ok(None),
        };
        let multiplier = Vec3::new(lambda[0], lambda[1], lambda[2]);
        let mut changes = columns.map(|column| column.dot(multiplier));
        let domains = [
            first.u1 - first.u0,
            first.v1 - first.v0,
            second.u1 - second.u0,
            second.v1 - second.v0,
        ];
        for index in 0..4 {
            changes[index] = changes[index].clamp(-domains[index] / 4.0, domains[index] / 4.0);
        }
        ua = fit(ua + changes[0], first.u0, first.u1, first.closed_u);
        va = fit(va + changes[1], first.v0, first.v1, first.closed_v);
        ub = fit(ub + changes[2], second.u0, second.u1, second.closed_u);
        vb = fit(vb + changes[3], second.v0, second.v1, second.closed_v);
    }
    let a = first.surface.evaluate(ua, va)?;
    let b = second.surface.evaluate(ub, vb)?;
    if a.sub(b).length() <= tolerance {
        Ok(Some(IntersectionPoint {
            ua,
            va,
            ub,
            vb,
            point: a.add(b).scale(0.5),
        }))
    } else {
        Ok(None)
    }
}

fn find_start_points(
    first: SurfaceInfo<'_>,
    second: SurfaceInfo<'_>,
    tolerance: f64,
    density: f64,
) -> Result<Vec<IntersectionPoint>, String> {
    let grid_count = |info: SurfaceInfo<'_>, direction_u: bool| {
        let (knots, degree) = if direction_u {
            (&info.surface.knots_u, info.surface.degree_u)
        } else {
            (&info.surface.knots_v, info.surface.degree_v)
        };
        (8.0_f64)
            .max(((interior_knot_count(knots, degree) + 1) * (degree + 1)) as f64 * density)
            .ceil()
            .min(24.0) as usize
    };
    let nu = grid_count(first, true);
    let nv = grid_count(first, false);
    let first_box = Bounds::from_surface(first.surface)?;
    let second_box = Bounds::from_surface(second.surface)?;
    let gate = first_box.diagonal().min(second_box.diagonal()) * 0.15 + tolerance;
    let expanded_second = second_box.expanded((tolerance * 100.0).max(gate));
    let mut starts = Vec::new();
    for i in 0..=nu {
        for j in 0..=nv {
            let ua = first.u0 + (first.u1 - first.u0) * i as f64 / nu as f64;
            let va = first.v0 + (first.v1 - first.v0) * j as f64 / nv as f64;
            let point = first.surface.evaluate(ua, va)?;
            if !expanded_second.contains(point) {
                continue;
            }
            let projection = project_point_to_surface(second.surface, point)?;
            if projection.distance > gate {
                continue;
            }
            if let Some(refined) =
                refine_point(first, second, ua, va, projection.u, projection.v, tolerance)?
            {
                starts.push(refined);
            }
        }
    }
    Ok(starts)
}

/// Whether the two carriers CROSS at `point` (`|n_a × n_b|` at or above
/// [`TRANSVERSE_START_CROSS`]) rather than touch tangentially; a surface
/// with no normal there counts as not crossing.
fn transverse_at(
    first: SurfaceInfo<'_>,
    second: SurfaceInfo<'_>,
    point: IntersectionPoint,
) -> bool {
    match (
        first.surface.normal(point.ua, point.va),
        second.surface.normal(point.ub, point.vb),
    ) {
        (Ok(a), Ok(b)) => a.cross(b).length() >= TRANSVERSE_START_CROSS,
        _ => false,
    }
}

fn tangent_at(
    first: SurfaceInfo<'_>,
    second: SurfaceInfo<'_>,
    point: IntersectionPoint,
) -> Result<Option<Vec3>, String> {
    let first_normal = first.surface.normal(point.ua, point.va).ok();
    let second_normal = second.surface.normal(point.ub, point.vb).ok();
    match (first_normal, second_normal) {
        (Some(a), Some(b)) => {
            let tangent = a.cross(b);
            if tangent.length() <= 1e-9 {
                Ok(None)
            } else {
                Ok(Some(tangent.normalized()?))
            }
        }
        _ => Ok(None),
    }
}

fn polish_boundary(
    first: SurfaceInfo<'_>,
    second: SurfaceInfo<'_>,
    mut point: IntersectionPoint,
    tolerance: f64,
) -> Result<Option<IntersectionPoint>, String> {
    let free = [
        first.closed_u || (point.ua > first.u0 && point.ua < first.u1),
        first.closed_v || (point.va > first.v0 && point.va < first.v1),
        second.closed_u || (point.ub > second.u0 && point.ub < second.u1),
        second.closed_v || (point.vb > second.v0 && point.vb < second.v1),
    ];
    for _ in 0..40 {
        let da = first.surface.derivatives(point.ua, point.va, 1)?;
        let db = second.surface.derivatives(point.ub, point.vb, 1)?;
        let residual = da[0][0].sub(db[0][0]);
        if residual.length() <= tolerance {
            point.point = da[0][0].add(db[0][0]).scale(0.5);
            return Ok(Some(point));
        }
        let all_columns = [
            da[1][0],
            da[0][1],
            db[1][0].scale(-1.0),
            db[0][1].scale(-1.0),
        ];
        let indices: Vec<usize> = (0..4).filter(|index| free[*index]).collect();
        if indices.is_empty() {
            return Ok(None);
        }
        let mut matrix = [[0.0f64; 4]; 4];
        let mut rhs = [0.0f64; 4];
        for (row, &r) in indices.iter().enumerate() {
            rhs[row] = -all_columns[r].dot(residual);
            for (column, &c) in indices.iter().enumerate() {
                matrix[row][column] = all_columns[r].dot(all_columns[c]);
            }
        }
        let changes = match solve_small(matrix, rhs, indices.len()) {
            Ok(value) => value,
            Err(_) => return Ok(None),
        };
        let mut update = [0.0; 4];
        for (index, &parameter_index) in indices.iter().enumerate() {
            update[parameter_index] = changes[index];
        }
        point.ua = fit(point.ua + update[0], first.u0, first.u1, first.closed_u);
        point.va = fit(point.va + update[1], first.v0, first.v1, first.closed_v);
        point.ub = fit(point.ub + update[2], second.u0, second.u1, second.closed_u);
        point.vb = fit(point.vb + update[3], second.v0, second.v1, second.closed_v);
    }
    Ok(None)
}

fn correct(
    first: SurfaceInfo<'_>,
    second: SurfaceInfo<'_>,
    from: IntersectionPoint,
    predicted: Vec3,
    tangent: Vec3,
    tolerance: f64,
) -> Result<Option<(IntersectionPoint, bool)>, String> {
    let mut point = from;
    let mut boundary = false;
    for _ in 0..30 {
        let da = first.surface.derivatives(point.ua, point.va, 1)?;
        let db = second.surface.derivatives(point.ub, point.vb, 1)?;
        let residual = da[0][0].sub(db[0][0]);
        let plane_residual = da[0][0].sub(predicted).dot(tangent);
        if residual.length() <= tolerance * 0.01 + EPSILON
            && plane_residual.abs() <= tolerance * 0.01 + EPSILON
        {
            point.point = da[0][0].add(db[0][0]).scale(0.5);
            return Ok(Some((point, boundary)));
        }
        let matrix = [
            [da[1][0].x, da[0][1].x, -db[1][0].x, -db[0][1].x],
            [da[1][0].y, da[0][1].y, -db[1][0].y, -db[0][1].y],
            [da[1][0].z, da[0][1].z, -db[1][0].z, -db[0][1].z],
            [da[1][0].dot(tangent), da[0][1].dot(tangent), 0.0, 0.0],
        ];
        let changes = match solve_small(
            matrix,
            [-residual.x, -residual.y, -residual.z, -plane_residual],
            4,
        ) {
            Ok(value) => value,
            Err(_) => return Ok(None),
        };
        let scale = [
            changes[0].abs() / ((first.u1 - first.u0) / 4.0),
            changes[1].abs() / ((first.v1 - first.v0) / 4.0),
            changes[2].abs() / ((second.u1 - second.u0) / 4.0),
            changes[3].abs() / ((second.v1 - second.v0) / 4.0),
            1.0,
        ]
        .into_iter()
        .fold(1.0_f64, f64::max);
        let raw = [
            point.ua + changes[0] / scale,
            point.va + changes[1] / scale,
            point.ub + changes[2] / scale,
            point.vb + changes[3] / scale,
        ];
        let domains = [
            (first.u0, first.u1, first.closed_u),
            (first.v0, first.v1, first.closed_v),
            (second.u0, second.u1, second.closed_u),
            (second.v0, second.v1, second.closed_v),
        ];
        boundary = false;
        let mut next = [0.0; 4];
        for index in 0..4 {
            next[index] = fit(
                raw[index],
                domains[index].0,
                domains[index].1,
                domains[index].2,
            );
            if !domains[index].2
                && (next[index] == domains[index].0 || next[index] == domains[index].1)
                && raw[index] != next[index]
            {
                boundary = true;
            }
        }
        let old = [point.ua, point.va, point.ub, point.vb];
        let spans = [
            first.u1 - first.u0,
            first.v1 - first.v0,
            second.u1 - second.u0,
            second.v1 - second.v0,
        ];
        let stalled = (0..4).all(|index| (next[index] - old[index]).abs() <= 1e-14 * spans[index]);
        point.ua = next[0];
        point.va = next[1];
        point.ub = next[2];
        point.vb = next[3];
        if stalled {
            if boundary {
                if let Some(polished) = polish_boundary(first, second, point, tolerance)? {
                    return Ok(Some((polished, true)));
                }
            }
            return Ok(None);
        }
    }
    Ok(None)
}

struct Trace {
    points: Vec<IntersectionPoint>,
    closed: bool,
}

fn vector_angle(a: Vec3, b: Vec3) -> f64 {
    (a.dot(b) / (a.length() * b.length()))
        .clamp(-1.0, 1.0)
        .acos()
}

/// Predictive step size from the surface-normal turn budget (Golovanov
/// §4.12): for each parameter of each surface the permissible parametric
/// step is Δα·√g/b (g, b = first/second fundamental form coefficients);
/// its 3D advance is Δα·g/b, and the usable step is the smallest advance
/// PROJECTED onto the march direction (the displacement sphere of §4.8),
/// ignoring parameters that barely move along the curve.  Returns None
/// when curvature data is unavailable (degenerate normals) — the caller
/// keeps its reactive step then.
fn normal_turn_step(
    first: SurfaceInfo<'_>,
    second: SurfaceInfo<'_>,
    point: IntersectionPoint,
    tangent: Vec3,
    budget: f64,
) -> Option<f64> {
    let mut best: Option<f64> = None;
    for (surface, u, v) in [
        (first.surface, point.ua, point.va),
        (second.surface, point.ub, point.vb),
    ] {
        let derivatives = surface.derivatives(u, v, 2).ok()?;
        let normal = derivatives[1][0].cross(derivatives[0][1]);
        let normal_length = normal.length();
        if normal_length <= 1e-12 {
            continue;
        }
        let unit_normal = normal.scale(1.0 / normal_length);
        for (first_derivative, second_derivative) in [
            (derivatives[1][0], derivatives[2][0]),
            (derivatives[0][1], derivatives[0][2]),
        ] {
            let metric = first_derivative.dot(first_derivative);
            if metric <= 1e-16 {
                continue;
            }
            let curvature = second_derivative.dot(unit_normal).abs();
            if curvature <= 1e-12 {
                continue; // flat along this parameter: no budget limit
            }
            let advance = budget * metric / curvature;
            let alignment = first_derivative
                .scale(1.0 / metric.sqrt())
                .dot(tangent)
                .abs();
            if alignment <= 0.1 {
                continue; // barely moves along the curve (§4.8)
            }
            let along = advance * alignment;
            if best.map(|value| along < value).unwrap_or(true) {
                best = Some(along);
            }
        }
    }
    best
}

/// The switches a trace reads, taken from the environment once per
/// intersection call rather than once per trace.
#[derive(Clone, Copy)]
struct TraceSwitches {
    /// `BREP_SSI_FIRST_STEP_BUDGET=0` lets the first step skip the §4.12
    /// ceiling, as every trace did before 2026-09-15.
    first_step_ceiling: bool,
    /// `BREP_DEBUG_SSI_TRACE=1`: every start and every accepted station.
    debug: bool,
}

impl TraceSwitches {
    fn from_env() -> Self {
        Self {
            first_step_ceiling: std::env::var("BREP_SSI_FIRST_STEP_BUDGET").as_deref() != Ok("0"),
            debug: std::env::var_os("BREP_DEBUG_SSI_TRACE").is_some(),
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn trace(
    first: SurfaceInfo<'_>,
    second: SurfaceInfo<'_>,
    start: IntersectionPoint,
    direction: f64,
    tolerance: f64,
    initial_step: f64,
    minimum_step: f64,
    maximum_step: f64,
    maximum_steps: usize,
    switches: TraceSwitches,
) -> Result<Trace, String> {
    let mut points = vec![start];
    let mut current = start;
    let Some(mut tangent) = tangent_at(first, second, start)? else {
        return Ok(Trace {
            points,
            closed: false,
        });
    };
    tangent = tangent.scale(direction);
    // The FIRST step is held to the same §4.12 normal-turn ceiling as every
    // later one. The ceiling used to be applied only after a step was accepted,
    // so step zero was `diagonal / 100` whatever the curvature — and a step
    // that lands on a trim-window boundary ends the branch before the turn
    // check below ever reads it. On the rescue pair
    // (`boolean_fuzz_corpus/23_section_pcurve_fold_t222`, cone × a 90° pin-wall
    // window) that one step was the whole 0.69 and 0.77 of section, turning the
    // tangent 0.97 rad where the ceiling allows 1.0e-2 of advance: two
    // stations, a four-point fit, and trims 3.0e-2 and 3.8e-2 apart. Escape
    // hatch: BREP_SSI_FIRST_STEP_BUDGET=0.
    let mut step_size = initial_step.min(maximum_step);
    if switches.first_step_ceiling {
        if let Some(budget) = normal_turn_step(first, second, start, tangent, 0.1) {
            step_size = step_size.min(budget).clamp(minimum_step, maximum_step);
        }
    }
    let mut terminated = false;
    for _ in 0..maximum_steps {
        let mut accepted = None;
        while step_size >= minimum_step {
            let predicted = current.point.add(tangent.scale(step_size));
            if let Some((corrected, boundary)) =
                correct(first, second, current, predicted, tangent, tolerance)?
            {
                if corrected.point.sub(current.point).length() <= 2.5 * step_size {
                    accepted = Some((corrected, boundary));
                    break;
                }
            }
            step_size /= 2.0;
        }
        let Some((candidate, boundary)) = accepted else {
            terminated = true;
            break;
        };
        points.push(candidate);
        if switches.debug {
            // One line per accepted station: the tangent's turn across the
            // step, the normal-turn ceiling where it started, the step asked
            // for and the distance actually covered.
            let turn = tangent_at(first, second, candidate)
                .ok()
                .flatten()
                .map(|next| vector_angle(tangent, if next.dot(tangent) < 0.0 { next.scale(-1.0) } else { next }))
                .unwrap_or(f64::NAN);
            let budget = normal_turn_step(first, second, current, tangent, 0.1).unwrap_or(f64::NAN);
            let at = candidate.point;
            eprintln!(
                "SSI-TRACE dir {direction} step {step_size:.4e} jump {:.4e} turn {turn:.4} ceiling {budget:.4e} boundary {boundary} p ({:.6},{:.6},{:.6})",
                at.sub(current.point).length(),
                at.x,
                at.y,
                at.z
            );
        }
        if boundary {
            terminated = true;
            break;
        }
        let Some(mut next_tangent) = tangent_at(first, second, candidate)? else {
            terminated = true;
            break;
        };
        if next_tangent.dot(tangent) < 0.0 {
            next_tangent = next_tangent.scale(-1.0);
        }
        let angle = vector_angle(tangent, next_tangent);
        if angle > 0.35 {
            points.pop();
            step_size = minimum_step.max(step_size / 2.0);
            if step_size <= minimum_step * 1.01 {
                points.push(candidate);
            } else {
                continue;
            }
        }
        // §4.12 normal-turn budget on BOTH surfaces as a hard CEILING on
        // the reactive controller: growth stays reactive (accelerating
        // into under-sampled flats regressed downstream curve fits), but
        // the budget stops the step from outrunning surface curvature.
        let reactive = if angle < 0.03 {
            step_size * 1.5
        } else if angle > 0.15 {
            step_size * 0.6
        } else {
            step_size
        };
        step_size = match normal_turn_step(first, second, candidate, next_tangent, 0.1) {
            Some(budget) => reactive.min(budget),
            None => reactive,
        }
        .clamp(minimum_step, maximum_step);
        if points.len() > 4 && candidate.point.sub(start.point).length() <= step_size * 1.2 {
            let to_start = start.point.sub(candidate.point);
            if to_start.length() <= EPSILON || to_start.dot(next_tangent) >= 0.0 {
                points.push(start);
                return Ok(Trace {
                    points,
                    closed: true,
                });
            }
        }
        current = candidate;
        tangent = next_tangent;
    }
    if !terminated {
        // The step budget ran out with the curve still going: a silently
        // truncated branch flows into topology as if complete, which is far
        // worse than a loud failure here.
        return Err(format!(
            "surface intersection trace exhausted {maximum_steps} steps \
             ({} points, step {step_size:.3e}) without reaching a boundary or closure",
            points.len()
        ));
    }
    Ok(Trace {
        points,
        closed: false,
    })
}

fn point_segment_distance(point: Vec3, start: Vec3, end: Vec3) -> f64 {
    let segment = end.sub(start);
    let length_squared = segment.length_squared();
    if length_squared <= EPSILON {
        return point.sub(start).length();
    }
    let parameter = point.sub(start).dot(segment) / length_squared;
    let parameter = parameter.clamp(0.0, 1.0);
    point.sub(start.add(segment.scale(parameter))).length()
}

#[derive(Clone, Debug, Serialize)]
pub struct SurfaceIntersectionCurve {
    pub points: Vec<Vec3>,
    pub params_a: Vec<[f64; 2]>,
    pub params_b: Vec<[f64; 2]>,
    pub closed: bool,
}

#[derive(Clone, Debug, Deserialize)]
pub struct SurfaceIntersectionOptions {
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
    #[serde(default = "default_density")]
    pub seed_density: f64,
    #[serde(default = "default_maximum_steps")]
    pub maximum_steps: usize,
    pub maximum_step: Option<f64>,
    #[serde(default)]
    pub seed_points: Vec<Vec3>,
    #[serde(default)]
    pub seed_only: bool,
    pub seed_gate: Option<f64>,
}

fn default_tolerance() -> f64 {
    LINEAR_TOLERANCE
}
fn default_density() -> f64 {
    1.0
}
fn default_maximum_steps() -> usize {
    4000
}

impl Default for SurfaceIntersectionOptions {
    fn default() -> Self {
        Self {
            tolerance: default_tolerance(),
            seed_density: default_density(),
            maximum_steps: default_maximum_steps(),
            maximum_step: None,
            seed_points: Vec::new(),
            seed_only: false,
            seed_gate: None,
        }
    }
}

/// Refine `seed` onto the intersection of `first_surface` and `second_surface`:
/// the closest point on each carrier seeds the same Newton the march uses
/// ([`refine_point`]), so the answer is an INTERSECTION point at the march's own
/// residual (`tolerance * 0.01`), not a projection onto one surface — a point
/// merely projected onto one carrier would carry the very bias a section
/// refiner exists to remove (2026-09-26, the sphere/bore rim of `BadBoolean`).
/// `None` when the Newton does not converge or lands further than `reach` from
/// the seed: a refiner must not pull a station onto another branch.
pub(crate) fn refine_to_intersection(
    first_surface: &NurbsSurface,
    second_surface: &NurbsSurface,
    seed: Vec3,
    tolerance: f64,
    reach: f64,
) -> Result<Option<Vec3>, String> {
    let first = surface_info(first_surface)?;
    let second = surface_info(second_surface)?;
    let on_first = project_point_to_surface(first_surface, seed)?;
    let on_second = project_point_to_surface(second_surface, seed)?;
    let Some(refined) = refine_point(
        first,
        second,
        on_first.u,
        on_first.v,
        on_second.u,
        on_second.v,
        tolerance,
    )?
    else {
        return Ok(None);
    };
    if refined.point.sub(seed).length() > reach {
        return Ok(None);
    }
    Ok(Some(refined.point))
}

pub fn intersect_surfaces(
    first_surface: &NurbsSurface,
    second_surface: &NurbsSurface,
    options: &SurfaceIntersectionOptions,
) -> Result<Vec<SurfaceIntersectionCurve>, String> {
    let first_box = Bounds::from_surface(first_surface)?;
    let second_box = Bounds::from_surface(second_surface)?;
    if !first_box.intersects(second_box, options.tolerance * 10.0) {
        return Ok(Vec::new());
    }
    let first = surface_info(first_surface)?;
    let second = surface_info(second_surface)?;
    let diagonal = first_box.diagonal().min(second_box.diagonal());
    let initial_step = diagonal / 100.0;
    let minimum_step = (options.tolerance * 100.0).max(diagonal * 1e-6);
    let maximum_step = (diagonal / 15.0).min(options.maximum_step.unwrap_or(f64::INFINITY));
    let maximum_steps = step_budget(options.maximum_steps, maximum_step, diagonal);
    let mut starts = if options.seed_only {
        Vec::new()
    } else {
        find_start_points(first, second, options.tolerance, options.seed_density)?
    };
    // A grid start is refined onto both carriers exactly like an explicit
    // seed and is held to the same transversality gate: inside a tangency
    // band the refinement converges (the band IS within tolerance of both
    // surfaces) to a point no transverse curve passes through.
    starts.retain(|start| transverse_at(first, second, *start));
    let seed_gate = options.seed_gate.unwrap_or(options.tolerance * 100.0);
    for seed in &options.seed_points {
        let projection_a = project_point_to_surface(first_surface, *seed)?;
        if projection_a.distance > seed_gate {
            continue;
        }
        let projection_b = project_point_to_surface(second_surface, *seed)?;
        if projection_b.distance > seed_gate {
            continue;
        }
        let Some(refined) = refine_point(
            first,
            second,
            projection_a.u,
            projection_a.v,
            projection_b.u,
            projection_b.v,
            options.tolerance,
        )?
        else {
            continue;
        };
        if !transverse_at(first, second, refined) {
            continue;
        }
        starts.push(refined);
    }

    let switches = TraceSwitches::from_env();
    let mut curves: Vec<SurfaceIntersectionCurve> = Vec::new();
    for start in starts {
        let claimed = curves.iter().any(|curve| {
            curve.points.windows(2).any(|segment| {
                point_segment_distance(start.point, segment[0], segment[1]) <= initial_step * 1.5
            })
        });
        if switches.debug {
            let at = start.point;
            eprintln!(
                "SSI-START claimed {claimed} p ({:.6},{:.6},{:.6}) initial {initial_step:.4e} minimum {minimum_step:.4e} maximum {maximum_step:.4e}",
                at.x, at.y, at.z
            );
        }
        if claimed {
            continue;
        }
        let forward = trace(
            first,
            second,
            start,
            1.0,
            options.tolerance,
            initial_step,
            minimum_step,
            maximum_step,
            maximum_steps,
            switches,
        )?;
        let (all, closed) = if forward.closed {
            (forward.points, true)
        } else {
            let backward = trace(
                first,
                second,
                start,
                -1.0,
                options.tolerance,
                initial_step,
                minimum_step,
                maximum_step,
                maximum_steps,
                switches,
            )?;
            let mut all: Vec<_> = backward.points.into_iter().skip(1).rev().collect();
            all.extend(forward.points);
            (all, false)
        };
        if all.len() < 2 {
            continue;
        }
        curves.push(SurfaceIntersectionCurve {
            points: all.iter().map(|point| point.point).collect(),
            params_a: all.iter().map(|point| [point.ua, point.va]).collect(),
            params_b: all.iter().map(|point| [point.ub, point.vb]).collect(),
            closed,
        });
    }
    Ok(curves)
}

/// Minimum |n_a × n_b| for a seed/branch to count as a *transverse* crossing
/// rather than a tangential graze — the same threshold `intersect_surfaces`
/// applies to its start points.
const SUPPLEMENTAL_TRANSVERSE_CROSS: f64 = TRANSVERSE_START_CROSS;

/// Gated supplemental SSI **detector** for a pair the coarse pair-classifier
/// culled as "tangential-only" (`NearTangent` + `tangential_only`).
///
/// The 5×5 classification grid can land only on an incidental tangential KISS
/// between two curved carriers (e.g. the inner walls / tube tops of two
/// overlapping tori) and never sample the transverse loop where their tubes
/// actually CROSS, so `tangential_only` is a false positive that would drop a
/// real intersection. This routine re-seeds a DENSER grid on BOTH carriers,
/// keeps only *transverse* seeds (near-parallel normals rejected up front, so
/// the tangency band is never walked), SWALLOWS per-branch trace-exhaustion (a
/// genuine tangency band never closes/exits and would otherwise error — here
/// it simply means "no transverse curve from this seed"), and returns only
/// branches that are genuinely transverse along their length.
///
/// A *genuinely* tangential pair yields an empty result (every seed rejected /
/// every trace fruitless), so the caller skips exactly as before — the common
/// intersection path is never touched.
///
/// NOTE — deferred capability: when this DOES return branches, the true
/// intersection is *singular* (the transverse loops meet at tangent nodes,
/// e.g. equal-radius torus-torus at `(1.5, ±3.708, ±1.5)`). Imprinting them
/// requires 4-valent tangent-vertex ("checkerboard") singular assembly, which
/// the kernel does not yet do; until then the caller REFUSES such a pair with
/// a clear error rather than silently dropping the geometry or emitting the
/// cryptic downstream pcurve/open-loop error. See imprint.rs (tangential-only
/// skip) and the project singular-assembly roadmap.
pub fn intersect_surfaces_supplemental(
    first_surface: &NurbsSurface,
    second_surface: &NurbsSurface,
    options: &SurfaceIntersectionOptions,
) -> Result<Vec<SurfaceIntersectionCurve>, String> {
    let first_box = Bounds::from_surface(first_surface)?;
    let second_box = Bounds::from_surface(second_surface)?;
    if !first_box.intersects(second_box, options.tolerance * 10.0) {
        return Ok(Vec::new());
    }
    let first = surface_info(first_surface)?;
    let second = surface_info(second_surface)?;
    let diagonal = first_box.diagonal().min(second_box.diagonal());
    let initial_step = diagonal / 100.0;
    let minimum_step = (options.tolerance * 100.0).max(diagonal * 1e-6);
    let maximum_step = (diagonal / 15.0).min(options.maximum_step.unwrap_or(f64::INFINITY));
    let maximum_steps = step_budget(options.maximum_steps, maximum_step, diagonal);

    // Denser self-seed (≥2× density pushes the ≤24 grid toward its cap so a
    // thin transverse overlap the classifier stepped over still gets a start),
    // gridding BOTH carriers so an interior loop touching neither trim boundary
    // is still hit from whichever surface samples it best.
    let density = options.seed_density.max(2.0);
    let mut starts: Vec<IntersectionPoint> = Vec::new();
    for gridded_second in [false, true] {
        let (a, b) = if gridded_second {
            (second, first)
        } else {
            (first, second)
        };
        for start in find_start_points(a, b, options.tolerance, density)? {
            // find_start_points expresses params as (grid surface, projected
            // surface); normalise every start to (first, second).
            let start = if gridded_second {
                IntersectionPoint {
                    ua: start.ub,
                    va: start.vb,
                    ub: start.ua,
                    vb: start.va,
                    point: start.point,
                }
            } else {
                start
            };
            starts.push(start);
        }
    }
    // Reject tangential seeds outright: a start whose two surface normals are
    // (near-)parallel is on a graze, not a crossing, and marching it walks the
    // tangency band.
    starts.retain(|start| {
        match (
            first.surface.normal(start.ua, start.va),
            second.surface.normal(start.ub, start.vb),
        ) {
            (Ok(na), Ok(nb)) => na.cross(nb).length() > SUPPLEMENTAL_TRANSVERSE_CROSS,
            _ => false,
        }
    });

    let switches = TraceSwitches::from_env();
    let mut curves: Vec<SurfaceIntersectionCurve> = Vec::new();
    for start in starts {
        let claimed = curves.iter().any(|curve| {
            curve.points.windows(2).any(|segment| {
                point_segment_distance(start.point, segment[0], segment[1]) <= initial_step * 1.5
            })
        });
        if claimed {
            continue;
        }
        // Swallow trace-exhaustion here (unlike the common path, which errors
        // loudly): on this gated path a non-closing branch means the seed sat
        // on a tangency band, which we simply drop.
        let Ok(forward) = trace(
            first,
            second,
            start,
            1.0,
            options.tolerance,
            initial_step,
            minimum_step,
            maximum_step,
            maximum_steps,
            switches,
        ) else {
            continue;
        };
        let (all, closed) = if forward.closed {
            (forward.points, true)
        } else {
            let Ok(backward) = trace(
                first,
                second,
                start,
                -1.0,
                options.tolerance,
                initial_step,
                minimum_step,
                maximum_step,
                maximum_steps,
                switches,
            ) else {
                continue;
            };
            let mut all: Vec<_> = backward.points.into_iter().skip(1).rev().collect();
            all.extend(forward.points);
            (all, false)
        };
        if all.len() < 2 {
            continue;
        }
        // Keep only a branch that is genuinely transverse somewhere along its
        // length — a final guard against a tangency-band polyline that happened
        // to close.
        let transverse = all.iter().step_by((all.len() / 5).max(1)).any(|point| {
            match (
                first.surface.normal(point.ua, point.va),
                second.surface.normal(point.ub, point.vb),
            ) {
                (Ok(na), Ok(nb)) => na.cross(nb).length() > SUPPLEMENTAL_TRANSVERSE_CROSS,
                _ => false,
            }
        });
        if !transverse {
            continue;
        }
        curves.push(SurfaceIntersectionCurve {
            points: all.iter().map(|point| point.point).collect(),
            params_a: all.iter().map(|point| [point.ua, point.va]).collect(),
            params_b: all.iter().map(|point| [point.ub, point.vb]).collect(),
            closed,
        });
    }
    Ok(curves)
}

