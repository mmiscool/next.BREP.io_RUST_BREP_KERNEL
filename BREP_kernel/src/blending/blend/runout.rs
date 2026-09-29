//! The RUNOUT: a convex stripe's march continued with a CONCAVE STRIPE's own
//! surface as one of its carriers.
//!
//! # The shape
//!
//! A convex edge that DIES on the concave edges it meets — the rib spine
//! running out into the pad ring on the floor, case
//! `inbox-20260909-fillet-rib-spine-corner` — has no corner ball at that
//! vertex: the face the convex stripe shares with a concave one is approached
//! from both sides.  What the rolling ball actually does there is not a corner
//! at all, it is a CARRIER SWITCH, and the switch happens twice.
//!
//! Take the rib's tip.  Three selected edges meet: the convex spine (roof ∩
//! side), and two concave ones (floor ∩ side, floor ∩ roof).  The floor is the
//! face the two concave stripes SHARE and the convex stripe does not touch.
//!
//!  1. The convex ball rolls on the roof and the side until it becomes tangent
//!     to the FLOOR — the third face at the corner.  That is the TRI-TANGENT
//!     STATION, and it is a ball-tangency condition on an input face, not an
//!     incidence: [`plan_runout`] solves `dist(centre, floor) = r` along the
//!     edge, which is transversal.
//!
//!     At that station the ball is automatically tangent to the floor-side
//!     stripe's own surface as well, and its contact on the SIDE sits exactly
//!     on that stripe's rail there.  Both follow from one fact: a ball of
//!     radius r tangent to both faces of a concave edge from the material side
//!     is the MIRROR, across either face, of the ball that generates that
//!     edge's radius-r fillet, so their centres are 2r apart and the two balls
//!     touch — at the contact point itself.  The tangency is therefore a
//!     GRAZE: the centre's distance to the stripe's SURFACE has a MINIMUM
//!     equal to r there rather than crossing it, a double root, which is why
//!     the station is solved on the floor and only CHECKED against the stripe.
//!
//!  2. Past it the ball rolls on the roof and on the floor-side stripe's
//!     SURFACE.  That is the runout march this module builds: the same §4.9
//!     tangency system with the stripe's fitted surface as the second carrier
//!     and the section plane still riding the convex edge (extended past its
//!     own vertex).
//!
//!  3. It stops where its contact with the kept mate ENDS — where that contact
//!     reaches the OTHER concave stripe's rail on the roof.  By the same
//!     mirror identity that is exactly where the ball grazes the second
//!     stripe, so the stop is found as the MINIMUM of the distance to it
//!     (well conditioned, unlike the double root) and reported with its
//!     residual.  Past the stop the first carrier switches too, and the ball
//!     rolls on the two STRIPES.
//!
//!  4. That last stretch ends in a POLE: the two concave stripes' rails on the
//!     face they share MEET, the ball's two contacts merge there, and the
//!     blend section shrinks to a point.  The pole is a curve-curve
//!     intersection of two rails the march already fitted, not a solve.
//!
//! # What this module is for
//!
//! It measures the class: the station, the carrier pair, the marched stretch
//! with its contact residuals, the stop and the pole. The mixed-convexity
//! refusal (`fillet/edges.rs`) reports those numbers instead of only naming
//! the corner, so the refusal says WHICH construction is missing and where.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::{intersect_curves, project_point_to_surface, NurbsCurve, NurbsSurface, Vec3};

use super::edge::{extrusion_direction, fit_open_rows, march_open_stations, vertex_stations};
use super::network::{stripe_mates, MixedCorner, OVERSHOOTS};
use super::stations::*;

/// Samples used to bracket a station along the convex edge, and bisections
/// spent on it afterwards.
const BRACKET_SAMPLES: usize = 64;
const BISECTIONS: usize = 60;

/// Stations marched over the runout stretch, and over the search window the
/// stop is minimised in.
const RUNOUT_STATIONS: usize = 32;
const STOP_SAMPLES: usize = 96;

/// One carrier switch of a runout march.
pub(crate) struct RunoutStation {
    /// Parameter on the convex edge (EXTENDED past its own vertex where the
    /// station sits beyond it).
    #[allow(dead_code)]
    pub(crate) t: f64,
    /// Arc distance from the mixed-convexity vertex, along the convex edge.
    pub(crate) s: f64,
    /// The ball centre there.
    pub(crate) center: Vec3,
    /// The contact that reached the end of its carrier.
    #[allow(dead_code)]
    pub(crate) contact: Vec3,
    /// `|dist(centre, the stripe's surface) − r|` at the station: zero says
    /// the input-face solve and the stripe agree that this is the switch.
    pub(crate) stripe_residual: f64,
}

/// What a convex stripe's march needs past the tri-tangent station, measured.
pub(crate) struct RunoutPlan {
    /// The mixed-convexity vertex.
    #[allow(dead_code)]
    pub(crate) vertex: u64,
    /// The convex edge that runs out there.
    pub(crate) convex_edge: u64,
    /// The face the two concave stripes share and the convex stripe does not
    /// touch — the one the tri-tangent station is solved against.
    pub(crate) shared_face: u64,
    /// The convex stripe's mate that is KEPT past the station, and the one the
    /// concave stripe replaces.
    pub(crate) kept_mate: u64,
    pub(crate) replaced_mate: u64,
    /// Station A: where the second carrier becomes `second_carrier`'s surface.
    pub(crate) station: RunoutStation,
    /// The concave edge whose stripe is the second carrier past station A.
    pub(crate) second_carrier: u64,
    /// Station B: where the kept mate's contact ends on `first_carrier`'s
    /// rail, so the FIRST carrier switches to that stripe too.
    pub(crate) stop: Option<RunoutStation>,
    /// The concave edge whose stripe is the first carrier past station B.
    pub(crate) first_carrier: Option<u64>,
    /// Where the two concave stripes' rails on the shared face meet: the
    /// degenerate terminus of the last stretch.
    pub(crate) pole: Option<Vec3>,
    /// Stations marched between A and B with the stripe as second carrier.
    pub(crate) marched: usize,
    /// The worst excursion of either contact OUTSIDE its carrier's own
    /// parameter domain over those stations, as a fraction of that domain's
    /// span.  `|centre − contact| − r` would be a tautology here — the centre
    /// IS the offset point the Newton converged on — where this is the
    /// question that matters: `evaluate_extended` continues an open direction
    /// along the boundary tangent plane without bound, so a converged station
    /// can rest on a fictitious surface (`check_supports_on_carriers`).
    ///
    /// It is NOT zero on this class, and that is the shape rather than a
    /// defect: the kept mate's contact walks toward the strip the concave
    /// stripe's pad ADDS to that face, which the original face's own patch
    /// does not cover, so it leaves the patch before the switch station.  The
    /// bar is `check_supports_on_carriers`'s — one full span — and 5.3e-2 of
    /// one is what the rib measures.
    pub(crate) carrier_excursion: f64,
    /// Worst distance between the fitted runout rows' rails and the marched
    /// contacts they were fitted through.  Measured against the STATIONS, not
    /// by projecting onto the carriers: projection clamps at the carrier's
    /// patch boundary, and on this class the kept mate's contact deliberately
    /// walks off that patch (see `carrier_excursion`), so a projected residual
    /// would report the excursion again instead of the fit.
    pub(crate) fit_residual: f64,
}

/// The ball of a stripe at one section, solved and evaluated.
struct Ball {
    uv: [f64; 4],
    p1: Vec3,
    p2: Vec3,
    center: Vec3,
}

#[allow(clippy::too_many_arguments)]
fn solve_ball(
    surface1: &NurbsSurface,
    surface2: &NurbsSurface,
    rho: [f64; 2],
    seed: [f64; 4],
    edge: &EdgeRecord,
    t: f64,
    scale: f64,
) -> Result<Ball, KernelRefusal> {
    let (section_point, section_tangent) = section_frame(edge, t)?;
    let uv = solve_station(
        surface1,
        surface2,
        rho,
        seed,
        section_point,
        section_tangent,
        scale,
    )?;
    let (_, p1, p2, center) = tangency_residual(
        surface1,
        surface2,
        rho,
        uv,
        section_point,
        section_tangent,
    )?;
    Ok(Ball {
        uv,
        p1,
        p2,
        center,
    })
}

/// How far `uv` lies OUTSIDE `surface`'s own domain, as a fraction of the
/// domain span, worst of the two directions.  Closed directions are exempt:
/// every periodic image is the same real surface.
fn domain_excursion(surface: &NurbsSurface, uv: [f64; 2]) -> Result<f64, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let mut worst = 0.0f64;
    for (closed, value, low, high) in [
        (closed_u, uv[0], u0, u1),
        (closed_v, uv[1], v0, v1),
    ] {
        if closed {
            continue;
        }
        let span = (high - low).abs();
        if !(span > 0.0) {
            continue;
        }
        worst = worst.max((low - value).max(value - high).max(0.0) / span);
    }
    Ok(worst)
}

/// Distance from `point` to `surface`, and the parameters it projects to.
fn surface_gap(surface: &NurbsSurface, point: Vec3) -> Result<(f64, f64, f64), KernelRefusal> {
    let projection = project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    Ok((projection.distance, projection.u, projection.v))
}

/// March and fit one stripe the way `blend_star_network` step 4 does, taking
/// the first rung of the overshoot ladder that fits.
///
/// `widest` takes the LAST rung instead, which the pole needs: the pole sits
/// where the two stripes' rails on their shared face cross, and that crossing
/// is PAST the end of at least one of the two edges (on the rib it is half a
/// radius beyond the tip edge's own end, against that edge's 0.08·2.0 = 0.16
/// first-rung overshoot), so the narrow fit's rails do not reach each other.
fn fit_stripe(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius: f64,
    widest: bool,
) -> Result<FittedRows, KernelRefusal> {
    let radius_at = |_: f64| radius;
    let mut last = None;
    let ladder: Vec<f64> = if widest {
        OVERSHOOTS.iter().rev().copied().collect()
    } else {
        OVERSHOOTS.to_vec()
    };
    for &overshoot in &ladder {
        match march_open_stations(edge, first, second, &radius_at, overshoot) {
            Ok((stations, vertex_indices)) => {
                let parameters = station_parameters(&stations);
                let extrusion = extrusion_direction(edge, first, second);
                match fit_open_rows(&stations, &parameters, false, Some(vertex_indices), extrusion) {
                    Ok(mut rows) => {
                        rows.vertex_stations = vertex_stations(&parameters, vertex_indices);
                        return Ok(rows);
                    }
                    Err(error) => last = Some(error),
                }
            }
            Err(error) => last = Some(error),
        }
    }
    Err(last.unwrap_or_else(|| KernelRefusal::internal(KernelStage::Refine, "runout_march", format!("blend runout: edge {} did not march", edge.id))))
}

/// Arc distance from `vertex_t` to `t` along `curve`, by chord sampling (the
/// convex edges this runs on are straight, and a chord sum is exact there and
/// conservative elsewhere).
fn arc_distance(curve: &NurbsCurve, vertex_t: f64, t: f64) -> Result<f64, KernelRefusal> {
    const STEPS: usize = 64;
    let mut total = 0.0;
    let mut previous = curve.evaluate_extended(vertex_t).or_refuse(KernelStage::Refine, "evaluate_extended")?;
    for step in 1..=STEPS {
        let at = vertex_t + (t - vertex_t) * step as f64 / STEPS as f64;
        let point = curve.evaluate_extended(at).or_refuse(KernelStage::Refine, "evaluate_extended")?;
        total += point.sub(previous).length();
        previous = point;
    }
    Ok(total)
}

/// Measure what the convex stripe at `corner` needs past its tri-tangent
/// station.  `Err` says the class could not be measured on this selection —
/// the caller keeps its unmeasured refusal rather than dropping it.
pub(crate) fn plan_runout(
    solid: &BrepSolid,
    edge_ids: &[u64],
    radius: f64,
    corner: &MixedCorner,
) -> Result<RunoutPlan, KernelRefusal> {
    let mates = stripe_mates(solid, edge_ids, radius)?;
    let convex_index = *corner
        .convex
        .first()
        .ok_or(KernelRefusal::internal(KernelStage::Classify, "runout_convex_edge", "blend runout: the corner names no convex edge"))?;
    let (convex_edge, convex_first, convex_second) = &mates[convex_index];
    // The corner vertex is the one the convex edge shares with the concave
    // ones; `mixed_convexity_corner` reports its point, so match on that.
    let vertex = [convex_edge.start_vertex_id, convex_edge.end_vertex_id]
        .into_iter()
        .find(|id| {
            solid
                .vertices
                .iter()
                .find(|vertex| vertex.id == *id)
                .map(|vertex| vertex.point.sub(corner.point).length() <= 1e-9 * (1.0 + radius))
                .unwrap_or(false)
        })
        .ok_or(KernelRefusal::internal(KernelStage::Classify, "runout_convex_edge", "blend runout: the convex edge does not end at the reported corner"))?;
    let concave: Vec<usize> = corner
        .concave
        .iter()
        .copied()
        .filter(|index| {
            let edge = mates[*index].0;
            edge.start_vertex_id == vertex || edge.end_vertex_id == vertex
        })
        .collect();
    if concave.len() < 2 {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_concave_edges", "blend runout: fewer than two concave edges reach the corner"));
    }
    // The face the concave stripes SHARE and the convex stripe does not touch.
    let convex_faces = [convex_first.face.id, convex_second.face.id];
    let mut shared: Option<&FaceRecord> = None;
    for &index in &concave {
        for mate in [&mates[index].1, &mates[index].2] {
            if convex_faces.contains(&mate.face.id) {
                continue;
            }
            let carried_by_all = concave.iter().all(|other| {
                [&mates[*other].1, &mates[*other].2]
                    .iter()
                    .any(|candidate| candidate.face.id == mate.face.id)
            });
            if carried_by_all {
                shared = Some(mate.face);
            }
        }
    }
    let shared = shared.ok_or(
        KernelRefusal::unsupported(KernelStage::Classify, "runout_shared_face", "blend runout: the concave edges at the corner share no face the convex edge misses",)
    )?;

    // ---- Station A: the convex ball tangent to the shared face. ----
    let vertex_t = if convex_edge.start_vertex_id == vertex {
        convex_edge.t0
    } else {
        convex_edge.t1
    };
    let far_t = if convex_edge.start_vertex_id == vertex {
        convex_edge.t1
    } else {
        convex_edge.t0
    };
    let scale = march_model_scale(&convex_edge.curve, convex_edge.t0, convex_edge.t1, radius)?;
    let rho = [convex_first.rho, convex_second.rho];
    let surface1 = &convex_first.face.surface;
    let surface2 = &convex_second.face.surface;
    let seed_at = |t: f64| -> Result<[f64; 4], KernelRefusal> {
        let clamped = t.clamp(convex_edge.t0, convex_edge.t1);
        let uv1 = edge_uv_on_face(convex_first.coedge, convex_edge, clamped)?;
        let uv2 = edge_uv_on_face(convex_second.coedge, convex_edge, clamped)?;
        Ok([uv1[0], uv1[1], uv2[0], uv2[1]])
    };
    // Walk from the FAR end toward the corner so the first sign change found
    // is the first switch the ball meets.
    let mut seed = seed_at(far_t)?;
    let mut previous: Option<(f64, f64)> = None;
    let mut bracket: Option<(f64, f64)> = None;
    for sample in 0..=BRACKET_SAMPLES {
        let t = far_t + (vertex_t - far_t) * sample as f64 / BRACKET_SAMPLES as f64;
        let ball = solve_ball(surface1, surface2, rho, seed, convex_edge, t, scale)?;
        seed = ball.uv;
        let (gap, _, _) = surface_gap(&shared.surface, ball.center)?;
        let value = gap - radius;
        if let Some((previous_t, previous_value)) = previous {
            if previous_value.signum() != value.signum() {
                bracket = Some((previous_t, t));
                break;
            }
        }
        previous = Some((t, value));
    }
    let (mut low, mut high) = bracket.ok_or_else(|| {
        format!(
            "blend runout: the ball on edge {} never becomes tangent to face {} along it",
            convex_edge.id, shared.id
        )
    }).or_refuse(KernelStage::Refine, "ok_or_else")?;
    let station_ball = {
        let mut seed = seed_at(low)?;
        let mut best = solve_ball(surface1, surface2, rho, seed, convex_edge, low, scale)?;
        let mut low_value = surface_gap(&shared.surface, best.center)?.0 - radius;
        for _ in 0..BISECTIONS {
            let middle = 0.5 * (low + high);
            let ball = solve_ball(
                surface1,
                surface2,
                rho,
                seed,
                convex_edge,
                middle,
                scale,
            )?;
            seed = ball.uv;
            let value = surface_gap(&shared.surface, ball.center)?.0 - radius;
            if value.signum() == low_value.signum() {
                low = middle;
                low_value = value;
            } else {
                high = middle;
            }
            best = ball;
        }
        best
    };
    let station_t = 0.5 * (low + high);
    let station_s = arc_distance(&convex_edge.curve, vertex_t, station_t)?;

    // Which concave stripe becomes the second carrier: the one the ball is
    // tangent to at the station.  Its stripe is fitted against the ORIGINAL
    // solid, exactly as the network marches it.
    let mut stripes: Vec<(usize, FittedRows)> = Vec::new();
    for &index in &concave {
        let (edge, first, second) = &mates[index];
        stripes.push((index, fit_stripe(edge, first, second, radius, false)?));
    }
    let mut ranked: Vec<(usize, f64)> = Vec::new();
    for (index, rows) in &stripes {
        let (gap, _, _) = surface_gap(&rows.surface, station_ball.center)?;
        ranked.push((*index, (gap - radius).abs()));
    }
    ranked.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    let (second_index, stripe_residual) = ranked[0];
    let (_, second_rows) = stripes
        .iter()
        .find(|(index, _)| *index == second_index)
        .expect("ranked stripe present");
    let other_index = ranked
        .get(1)
        .map(|(index, _)| *index)
        .ok_or(KernelRefusal::unsupported(KernelStage::Classify, "runout_concave_stripes", "blend runout: only one concave stripe at the corner"))?;
    let (_, other_rows) = stripes
        .iter()
        .find(|(index, _)| *index == other_index)
        .expect("ranked stripe present");

    // Which mate the stripe replaces: the one whose contact is ON the stripe.
    let gap1 = surface_gap(&second_rows.surface, station_ball.p1)?.0;
    let gap2 = surface_gap(&second_rows.surface, station_ball.p2)?.0;
    let (kept, replaced, kept_surface, kept_rho, contact) = if gap2 <= gap1 {
        (
            convex_first.face.id,
            convex_second.face.id,
            surface1,
            convex_first.rho,
            station_ball.p2,
        )
    } else {
        (
            convex_second.face.id,
            convex_first.face.id,
            surface2,
            convex_second.rho,
            station_ball.p1,
        )
    };
    let station = RunoutStation {
        t: station_t,
        s: station_s,
        center: station_ball.center,
        contact,
        stripe_residual,
    };

    // ---- The pole: the two stripes' rails on the shared face meet. ----
    let rail_on = |index: usize, rows: &FittedRows| -> Result<NurbsCurve, KernelRefusal> {
        let (_, first, second) = &mates[index];
        if first.face.id == shared.id {
            Ok(rows.cr.clone())
        } else if second.face.id == shared.id {
            Ok(rows.cs.clone())
        } else {
            Err(KernelRefusal::internal(KernelStage::Refine, "runout_shared_face", format!(
                "blend runout: edge {} does not touch the shared face {}",
                mates[index].0.id, shared.id
            )))
        }
    };
    // The rails are FITS, so their crossing is only as sharp as the fit: the
    // band is the kernel's own committed fit accuracy at this scale, the same
    // one `march_stations` refines toward, not a hand-picked number.
    let band = crate::KernelTolerances::for_scale(scale, 1e-7).intersection_fit;
    let cross = |a: &NurbsCurve, b: &NurbsCurve| -> Option<Vec3> {
        intersect_curves(a, b, band)
            .ok()
            .and_then(|hits| {
                hits.into_iter().min_by(|x, y| {
                    x.gap.partial_cmp(&y.gap).unwrap_or(std::cmp::Ordering::Equal)
                })
            })
            .map(|hit| hit.point)
    };
    let mut pole = match (
        rail_on(second_index, second_rows),
        rail_on(other_index, other_rows),
    ) {
        (Ok(a), Ok(b)) => cross(&a, &b),
        _ => None,
    };
    if pole.is_none() {
        // The crossing is past at least one edge's own end, so refit both
        // stripes on the WIDEST rung of the ladder and look again.  Only the
        // rails are taken from this fit; the surfaces above stay the ones the
        // network itself would march.
        let mut wide: Vec<(usize, FittedRows)> = Vec::new();
        for &index in &[second_index, other_index] {
            let (edge, first, second) = &mates[index];
            if let Ok(rows) = fit_stripe(edge, first, second, radius, true) {
                wide.push((index, rows));
            }
        }
        if wide.len() == 2 {
            if let (Ok(a), Ok(b)) = (rail_on(wide[0].0, &wide[0].1), rail_on(wide[1].0, &wide[1].1))
            {
                pole = cross(&a, &b);
            }
        }
    }

    // ---- The runout march, and station B. ----
    // The stripe's signed radius: the centre sits one radius off its surface
    // along the RAW normal, and which way is read off the station itself.
    let (_, stripe_u, stripe_v) = surface_gap(&second_rows.surface, station_ball.center)?;
    let stripe_normal = raw_normal(&second_rows.surface, stripe_u, stripe_v)?;
    let stripe_point = second_rows.surface.evaluate(stripe_u, stripe_v).or_refuse(KernelStage::Refine, "evaluate")?;
    let stripe_rho = radius * station_ball.center.sub(stripe_point).dot(stripe_normal).signum();
    let runout_rho = [kept_rho, stripe_rho];
    let kept_uv = if kept == convex_first.face.id {
        [station_ball.uv[0], station_ball.uv[1]]
    } else {
        [station_ball.uv[2], station_ball.uv[3]]
    };
    let runout_seed = [kept_uv[0], kept_uv[1], stripe_u, stripe_v];
    // The window the stop is searched in: from the station to the pole's own
    // section plane (or to the corner vertex when there is no pole).
    let window_end = match pole {
        Some(point) => section_parameter(convex_edge, point, station_t, vertex_t)?,
        None => vertex_t,
    };
    let graze = |t: f64, seed: [f64; 4]| -> Result<(f64, Ball), KernelRefusal> {
        let ball = solve_ball(
            kept_surface,
            &second_rows.surface,
            runout_rho,
            seed,
            convex_edge,
            t,
            scale,
        )?;
        let (gap, _, _) = surface_gap(&other_rows.surface, ball.center)?;
        Ok(((gap - radius).abs(), ball))
    };
    let mut seed = runout_seed;
    let mut samples: Vec<(f64, f64, [f64; 4])> = Vec::new();
    for sample in 0..=STOP_SAMPLES {
        let t = station_t + (window_end - station_t) * sample as f64 / STOP_SAMPLES as f64;
        let Ok((value, ball)) = graze(t, seed) else {
            break;
        };
        seed = ball.uv;
        samples.push((t, value, ball.uv));
    }
    // Station B grazes the other stripe, so it is the MINIMUM of that distance,
    // not a crossing of it — and a minimum read off the sample grid is only
    // located to half a step (0.028 of arc here, which would report the mirror
    // identity as holding to 1e-4 when it holds far tighter).  Golden-section
    // the bracketing interval down to the sample seeds' own accuracy.
    let best = samples
        .iter()
        .enumerate()
        .min_by(|a, b| a.1 .1.partial_cmp(&b.1 .1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(index, sample)| (index, *sample))
        .filter(|(index, _)| *index > 0 && *index + 1 < samples.len());
    let (stop_station, stop_t) = match best {
        Some((index, _)) => {
            const GOLDEN: f64 = 0.618_033_988_749_894_9;
            let seed = samples[index].2;
            let mut low = samples[index - 1].0;
            let mut high = samples[index + 1].0;
            let mut x1 = high - GOLDEN * (high - low);
            let mut x2 = low + GOLDEN * (high - low);
            let mut f1 = graze(x1, seed)?.0;
            let mut f2 = graze(x2, seed)?.0;
            for _ in 0..BISECTIONS {
                if f1 < f2 {
                    high = x2;
                    x2 = x1;
                    f2 = f1;
                    x1 = high - GOLDEN * (high - low);
                    f1 = graze(x1, seed)?.0;
                } else {
                    low = x1;
                    x1 = x2;
                    f1 = f2;
                    x2 = low + GOLDEN * (high - low);
                    f2 = graze(x2, seed)?.0;
                }
            }
            let t = 0.5 * (low + high);
            let (residual, ball) = graze(t, seed)?;
            (
                Some(RunoutStation {
                    t,
                    s: arc_distance(&convex_edge.curve, vertex_t, t)?,
                    center: ball.center,
                    contact: ball.p1,
                    stripe_residual: residual,
                }),
                t,
            )
        }
        None => (None, window_end),
    };

    // ---- The stretch itself, marched and fitted. ----
    let mut stations: Vec<Station> = Vec::new();
    let mut carrier_excursion: f64 = 0.0;
    let mut seed = runout_seed;
    for sample in 0..=RUNOUT_STATIONS {
        let t = station_t + (stop_t - station_t) * sample as f64 / RUNOUT_STATIONS as f64;
        let (section_point, section_tangent) = section_frame(convex_edge, t)?;
        let uv = solve_station(
            kept_surface,
            &second_rows.surface,
            runout_rho,
            seed,
            section_point,
            section_tangent,
            scale,
        )?;
        seed = uv;
        let (_, p1, p2, center) = tangency_residual(
            kept_surface,
            &second_rows.surface,
            runout_rho,
            uv,
            section_point,
            section_tangent,
        )?;
        let n1 = raw_normal(kept_surface, uv[0], uv[1])?;
        let n2 = raw_normal(&second_rows.surface, uv[2], uv[3])?;
        let cos_alpha = runout_rho[0].signum() * runout_rho[1].signum() * n1.dot(n2);
        let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
        if weight <= 1e-6 {
            break;
        }
        let apex = apex_point(p1, n1, p2, n2, center)?;
        carrier_excursion = carrier_excursion
            .max(domain_excursion(kept_surface, [uv[0], uv[1]])?)
            .max(domain_excursion(&second_rows.surface, [uv[2], uv[3]])?);
        stations.push(Station {
            uv1: [uv[0], uv[1]],
            uv2: [uv[2], uv[3]],
            p1,
            p2,
            center,
            weight,
            apex,
        });
    }
    let mut fit_residual = f64::NAN;
    if stations.len() >= 4 {
        let parameters = station_parameters(&stations);
        if let Ok(rows) = fit_open_rows(&stations, &parameters, false, None, None) {
            let mut worst: f64 = 0.0;
            for (station, u) in stations.iter().zip(&parameters) {
                worst = worst
                    .max(rows.cr.evaluate(*u).or_refuse(KernelStage::Refine, "evaluate")?.sub(station.p1).length())
                    .max(rows.cs.evaluate(*u).or_refuse(KernelStage::Refine, "evaluate")?.sub(station.p2).length());
            }
            fit_residual = worst;
        }
    }

    Ok(RunoutPlan {
        vertex,
        convex_edge: convex_edge.id,
        shared_face: shared.id,
        kept_mate: kept,
        replaced_mate: replaced,
        station,
        second_carrier: mates[second_index].0.id,
        stop: stop_station,
        first_carrier: Some(mates[other_index].0.id),
        pole,
        marched: stations.len(),
        carrier_excursion,
        fit_residual,
    })
}

/// The section plane of `edge` at `t`: the extended point and the edge's unit
/// tangent, read as the one-sided limit where the parameterization is
/// stationary. The runout rides the convex edge's section planes past its own
/// vertex, so the read is the extended one, and past a stationary open end it
/// advances by the chord of the open march's nominal station step
/// (`march_section`). The runout's own reads (bracketing, bisection, its
/// stretch) have no one step, so every one of them takes the stripe march's
/// `span / STATIONS`, which keeps them on one family of section planes.
fn section_frame(edge: &EdgeRecord, t: f64) -> Result<(Vec3, Vec3), KernelRefusal> {
    let station_step = (edge.t1 - edge.t0).abs() / STATIONS as f64;
    march_section(edge, t, station_step, "blend runout")
}

/// The parameter on `edge` whose normal (section) plane contains `point`,
/// bracketed by `from`..`to`.  The section planes of the convex edge are what
/// the runout march rides, so a point past the edge's own vertex still has
/// one.
fn section_parameter(
    edge: &EdgeRecord,
    point: Vec3,
    from: f64,
    to: f64,
) -> Result<f64, KernelRefusal> {
    let value = |t: f64| -> Result<f64, KernelRefusal> {
        let (section_point, section_tangent) = section_frame(edge, t)?;
        Ok(point.sub(section_point).dot(section_tangent))
    };
    // The section plane sweeps monotonically along a straight or mildly
    // curved edge, so walk the window for a sign change and bisect.
    let span = to - from;
    let mut low = from;
    let mut low_value = value(from)?;
    let mut high = to + span;
    let mut found = false;
    for sample in 1..=BRACKET_SAMPLES {
        let t = from + (to + span - from) * sample as f64 / BRACKET_SAMPLES as f64;
        let current = value(t)?;
        if current.signum() != low_value.signum() {
            high = t;
            found = true;
            break;
        }
        low = t;
        low_value = current;
    }
    if !found {
        return Err(KernelRefusal::internal(KernelStage::Refine, "runout_section_plane", "blend runout: the pole has no section plane on the convex edge"));
    }
    for _ in 0..BISECTIONS {
        let middle = 0.5 * (low + high);
        let current = value(middle)?;
        if current.signum() == low_value.signum() {
            low = middle;
            low_value = current;
        } else {
            high = middle;
        }
    }
    Ok(0.5 * (low + high))
}
