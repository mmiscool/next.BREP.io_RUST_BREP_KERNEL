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
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{intersect_curves, project_point_to_curve, project_point_to_surface, NurbsCurve, NurbsSurface, Vec3, Vec4};

use super::edge::{
    extrusion_direction, fit_open_rows, fit_rows_breaking, fresh_id_source, interpolate_piece,
    march_open_stations, prune_orphan_vertices, trim_edge_at, vertex_stations,
};
use super::network::{stripe_mates, tangent_away_from, MixedCorner, OVERSHOOTS};
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
                        // The fitted wall itself must not double back along
                        // a row (`fold::check_rows_loop`); terminal.
                        super::fold::check_rows_loop(
                            &rows, &stations, &parameters, Some(vertex_indices), radius, vec![first.face.id, second.face.id], stations.iter().map(|station| station.center.length()).fold(1.0f64, f64::max),
                        )?;
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

// ======================================================================
// The composition: one wall per spine, pole to pole, on the ring result.
// ======================================================================

/// Stations per unit of the convex edge's parameter on the middle stretch,
/// read as the open march's `STATIONS` over the stretch between the two
/// tri-tangent stations; the shorter stretches take the same density with a
/// floor of `FIT_DEGREE + 1` stations so every row piece carries the fit.
const RUNOUT_DENSITY_STATIONS: usize = STATIONS;
const MIN_STRETCH_STATIONS: usize = FIT_DEGREE + 1;

/// One stretch of the runout wall: the carrier under each rail and the
/// stations marched on them, in increasing convex-edge parameter.  `first`
/// carries the FIRST rail (`Station::p1`), `second` the second, on every
/// stretch — the carriers change along the wall, the rails' sides do not.
struct Stretch {
    first: u64,
    second: u64,
    stations: Vec<Station>,
}

/// What the ring built at one end of a spine's G1 ridge: the two chain edges
/// past the spine, the pole they end on, the pads under them, the face the
/// pads share, and the two pad rails the switch vertices land on.
struct EndChain {
    /// The spine's own vertex at this end (v118 on the document).
    spine_vertex: u64,
    /// The kept mate ∩ pad_s edge (#120) and its far vertex (v117).
    e1: u64,
    mid_vertex: u64,
    /// The pad_r ∩ pad_s seam (#119) and the pole it ends on (v107).
    e2: u64,
    pole: u64,
    pole_point: Vec3,
    /// The concave stripe whose surface replaces the spine's `replaced` mate
    /// past station A, and the one replacing the kept mate past B.
    pad_s: u64,
    pad_r: u64,
    /// The face both pads run along and the spine misses: the one station A
    /// is solved against.
    floor: u64,
    /// pad_s's rail on the replaced mate, ending at `spine_vertex`, and
    /// pad_r's rail on the kept mate, ending at `mid_vertex`.
    side_rail: u64,
    roof_rail: u64,
}

// ======================================================================
//
// A convex spine whose fillet dies on ONE selected concave edge at a
// three-valent vertex whose third edge is unselected.  On the ring the
// spine's ridge continues smoothly into the pad's END ARC on the kept mate
// (`e1`) and then into the unselected edge itself (`e2`, kept ∩ floor): no
// seam, no pole.  Past the tri-tangent station the ball rolls on the kept
// mate and the pad, and the stripe ends
//
//  * where its kept-mate rail meets the unselected edge's line extended
//    (C5's `RAIL_X_EXTENDED_EDGEUSE`), closed by a planar CLIFF in the final
//    section's own plane, when that edge is CONVEX (the cliff keeps it
//    sharp; the Pappus body of `one_ended_runout_corner.py`);
//  * at the pad's far rail, rolling on the kept mate EXTENDED into the
//    floor and trimmed by the floor, when that edge is CONCAVE (not built
//    in this commit; named).
//
// Behind `BREP_BLEND_ONE_ENDED_RUNOUT` until its census.
// ======================================================================

/// `BREP_BLEND_ONE_ENDED_RUNOUT=1` turns the one-ended runout on, `=0` off;
/// `set_one_ended_runout_for_test` forces it on a test's own thread.  Off,
/// the public entry refuses the class exactly as before.
/// Promoted 2026-10-14: the switch-on census held the
/// battery, the case manifest and hlr_tests; `=0`/`off` is the escape hatch.
const ONE_ENDED_RUNOUT_DEFAULT: bool = true;

thread_local! {
    static ONE_ENDED_RUNOUT_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

pub(crate) fn one_ended_runout_on() -> bool {
    if let Some(forced) = ONE_ENDED_RUNOUT_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_ONE_ENDED_RUNOUT").as_deref() {
        Ok("0") | Ok("off") => false,
        Ok("1") | Ok("on") => true,
        _ => ONE_ENDED_RUNOUT_DEFAULT,
    }
}


/// The refusal class of a one-ended end whose unselected third edge is
/// CONCAVE and whose pad overshoots it (lane L, 2026-10-15).
pub(crate) const ONE_ENDED_CONCAVE_THIRD_WHAT: &str = "one_ended_concave_third";

// ======================================================================
// Lane L row 2 (2026-10-15): the one-ended end with a CONCAVE third edge
// (the wedge nine-pick's spine A at the wall).
//
// The pad (the selected concave edge's stripe) overshoots the unselected
// concave edge: its end arc on the kept mate climbs past that edge, and the
// kept face's boundary turns straight back along it.  The ball:
//
//  * runs on the spine's mates to the PAD'S GRAZE — the section through the
//    spine's ring vertex, where the ball's contact on the replaced mate sits
//    on the pad's rail and the ball touches the pad there (the mirror
//    identity on that mate; the floor tangency the convex-third end solves
//    coincides with it only for a right-angled pad);
//  * then rolls on the kept mate EXTENDED and on the pad, until its pad
//    contact reaches the pad's rail on the FLOOR (the face across the
//    unselected edge) — the terminus;
//  * and the stripe is TRIMMED by the floor's plane: the part past it lies
//    inside the floor's material.  The kept rail ends where it crosses that
//    plane (on the unselected edge), the trim curve runs on the floor from
//    there to the terminus's pad contact (on the pad's floor rail), and the
//    pad's end arc and the corner of the kept face it bounded are consumed.
//
// `BREP_BLEND_CONCAVE_THIRD_RUNOUT=0`/`off` is the escape hatch; promoted
// 2026-10-15 (the wedge nine-pick at its quadrature).
// ======================================================================

const CONCAVE_THIRD_RUNOUT_DEFAULT: bool = true;

thread_local! {
    static CONCAVE_THIRD_RUNOUT_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

/// `BREP_BLEND_CONCAVE_THIRD_RUNOUT=1`/`on` builds the concave-third
/// one-ended end, `=0`/`off` refuses it by name (`ONE_ENDED_CONCAVE_THIRD_WHAT`).
pub(crate) fn concave_third_runout_on() -> bool {
    if let Some(forced) = CONCAVE_THIRD_RUNOUT_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_CONCAVE_THIRD_RUNOUT").as_deref() {
        Ok("0") | Ok("off") => false,
        Ok("1") | Ok("on") => true,
        _ => CONCAVE_THIRD_RUNOUT_DEFAULT,
    }
}


/// What the ring built at a ONE-ENDED end of a spine: the pad's end arc on
/// the kept mate continuing the spine, the unselected edge continuing that
/// arc onto the floor, the pad, the floor, and the pad's rail on the
/// replaced mate that ends at the spine's vertex.
struct OneEndChain {
    spine_vertex: u64,
    /// kept ∩ pad: the pad's end arc, from `spine_vertex` to `mid_vertex`.
    e1: u64,
    mid_vertex: u64,
    /// kept ∩ floor: the UNSELECTED edge, continuing `e1` past `mid_vertex`.
    e2: u64,
    pad: u64,
    floor: u64,
    /// The pad's rail on the replaced mate, ending at `spine_vertex`.
    side_rail: u64,
    /// The unselected edge's convexity: convex takes the C5 cliff.
    third_convex: bool,
    /// A CONCAVE third edge (row 2): the pad's rail on the floor, leaving
    /// `mid_vertex` (the pad ∩ floor edge) — trimmed at the terminus.
    floor_rail: Option<u64>,
}

/// Read a one-ended end on the ring, or say why this end is not one.
fn one_ended_chain(
    ring: &BrepSolid,
    spine: &EdgeRecord,
    spine_vertex: u64,
    mates: [u64; 2],
    radius: f64,
) -> Result<(OneEndChain, u64, u64), KernelRefusal> {
    let refuse = |what: &str, message: String| KernelRefusal::unsupported(KernelStage::Classify, what, format!("blend one-ended runout: {message}"));
    let e1_id = continuation(ring, spine, spine_vertex)?.ok_or_else(|| refuse(
        "one_ended_chain",
        format!("the spine {} does not continue smoothly past vertex {spine_vertex} on the ring result", spine.id),
    ))?;
    let e1 = edge_by_id(ring, e1_id)?;
    let e1_faces = faces_of(ring, e1_id);
    let kept = *mates.iter().find(|id| e1_faces.contains(id)).ok_or_else(|| refuse(
        "one_ended_chain",
        format!("edge {e1_id} past the spine runs along neither of the spine's mates"),
    ))?;
    let replaced = if kept == mates[0] { mates[1] } else { mates[0] };
    let pad = *e1_faces.iter().find(|id| **id != kept).ok_or_else(|| KernelRefusal::internal(
        KernelStage::Classify, "one_ended_chain", format!("blend one-ended runout: edge {e1_id} has one face"),
    ))?;
    let mid_vertex = other_vertex(e1, spine_vertex);
    let e2_id = match continuation(ring, e1, mid_vertex)? {
        Some(id) => id,
        None => {
            // Lane L (2026-10-15): the CONCAVE-third branch. When the unselected
            // third edge is concave the pad overshoots it: its end arc on the
            // kept mate climbs past the edge's end and the kept mate's boundary
            // turns BACK down along the unselected edge's continuation (the wedge
            // nine-pick at its wall: arc to z = 6.339, back down to z = 6).
            let arriving = tangent_away_from(e1, mid_vertex)?;
            let reverses = edges_at(ring, mid_vertex).into_iter().filter(|id| *id != e1_id).any(|id| {
                let Ok(edge) = edge_by_id(ring, id) else { return false };
                faces_of(ring, id).contains(&kept)
                    && tangent_away_from(edge, mid_vertex)
                        .is_ok_and(|leaving| leaving.dot(arriving) >= 1.0 - 1e-6)
            });
            let point = vertex_point(ring, mid_vertex)?;
            if reverses && concave_third_runout_on() {
                return concave_third_chain(ring, spine, spine_vertex, mates, kept, replaced, pad, e1_id, mid_vertex, arriving);
            }
            return Err(if reverses {
                refuse(ONE_ENDED_CONCAVE_THIRD_WHAT, format!(
                    "at ({:.3}, {:.3}, {:.3}) the spine {} runs out with a CONCAVE unselected third edge: the pad's end arc {e1_id} on the kept face climbs past that edge and the kept face's boundary turns back along it, so the stripe must roll on the kept face EXTENDED and be trimmed by the face across the unselected edge — not built here",
                    point.x, point.y, point.z, spine.id
                ))
            } else {
                refuse("one_ended_chain", format!("the pad's end arc {e1_id} does not continue smoothly past vertex {mid_vertex}"))
            });
        }
    };
    let e2_faces = faces_of(ring, e2_id);
    if !e2_faces.contains(&kept) || e2_faces.contains(&pad) {
        return Err(refuse("one_ended_chain", format!(
            "the edge {e2_id} past vertex {mid_vertex} runs along faces {e2_faces:?}, not along the kept mate {kept} and a third face (a two-ended ridge runs along the pad here)"
        )));
    }
    let floor = *e2_faces.iter().find(|id| **id != kept).ok_or_else(|| KernelRefusal::internal(
        KernelStage::Classify, "one_ended_chain", format!("blend one-ended runout: edge {e2_id} has one face"),
    ))?;
    if floor == replaced {
        return Err(refuse("one_ended_chain", format!(
            "the edge {e2_id} past the pad's end arc runs back onto the spine's replaced mate {replaced}"
        )));
    }
    // The pad's rail on the replaced mate ends at the spine's vertex.
    let side_rail = edges_at(ring, spine_vertex)
        .into_iter()
        .filter(|id| *id != spine.id && *id != e1_id)
        .find(|id| {
            let faces = faces_of(ring, *id);
            faces.contains(&replaced) && faces.contains(&pad)
        })
        .ok_or_else(|| refuse("one_ended_chain", format!(
            "no rail of pad {pad} on face {replaced} ends at vertex {spine_vertex}"
        )))?;
    let third_convex = super::network::selection_convexity(ring, &[e2_id], radius)
        .and_then(|convexity| convexity.first().copied())
        .ok_or_else(|| refuse("one_ended_chain", format!("the convexity of the unselected edge {e2_id} could not be read")))?;
    Ok((
        OneEndChain { spine_vertex, e1: e1_id, mid_vertex, e2: e2_id, pad, floor, side_rail, third_convex, floor_rail: None },
        kept,
        replaced,
    ))
}

/// Read the concave-third one-ended end (row 2) at `mid_vertex`, where the
/// pad's end arc `e1` arrives going `arriving` and the kept face's boundary
/// turns straight back along the unselected edge.
#[allow(clippy::too_many_arguments)]
fn concave_third_chain(
    ring: &BrepSolid,
    spine: &EdgeRecord,
    spine_vertex: u64,
    mates: [u64; 2],
    kept: u64,
    replaced: u64,
    pad: u64,
    e1_id: u64,
    mid_vertex: u64,
    arriving: Vec3,
) -> Result<(OneEndChain, u64, u64), KernelRefusal> {
    let refuse = |message: String| KernelRefusal::unsupported(KernelStage::Classify, ONE_ENDED_CONCAVE_THIRD_WHAT, format!("blend one-ended runout (concave third): {message}"));
    let _ = mates;
    // The unselected edge: on the kept face, leaving mid_vertex straight back.
    let e2_id = edges_at(ring, mid_vertex)
        .into_iter()
        .filter(|id| *id != e1_id)
        .find(|id| {
            let Ok(edge) = edge_by_id(ring, *id) else { return false };
            faces_of(ring, *id).contains(&kept)
                && tangent_away_from(edge, mid_vertex).is_ok_and(|leaving| leaving.dot(arriving) >= 1.0 - 1e-6)
        })
        .ok_or_else(|| refuse(format!("no edge of the kept face {kept} turns back from vertex {mid_vertex}")))?;
    let floor = *faces_of(ring, e2_id).iter().find(|id| **id != kept).ok_or_else(|| KernelRefusal::internal(
        KernelStage::Classify, "one_ended_chain", format!("blend one-ended runout: edge {e2_id} has one face"),
    ))?;
    if floor == pad || floor == replaced {
        return Err(refuse(format!("the unselected edge {e2_id} runs onto face {floor}, the pad or the replaced mate")));
    }
    // The pad's rail on the floor, leaving mid_vertex.
    let floor_rail = edges_at(ring, mid_vertex)
        .into_iter()
        .filter(|id| *id != e1_id && *id != e2_id)
        .find(|id| {
            let faces = faces_of(ring, *id);
            faces.contains(&pad) && faces.contains(&floor)
        })
        .ok_or_else(|| refuse(format!("no rail of pad {pad} on the floor {floor} leaves vertex {mid_vertex}")))?;
    let side_rail = edges_at(ring, spine_vertex)
        .into_iter()
        .filter(|id| *id != spine.id && *id != e1_id)
        .find(|id| {
            let faces = faces_of(ring, *id);
            faces.contains(&replaced) && faces.contains(&pad)
        })
        .ok_or_else(|| refuse(format!("no rail of pad {pad} on face {replaced} ends at vertex {spine_vertex}")))?;
    // The floor must be a plane: the trim is by its plane.
    if !matches!(face_by_id(ring, floor)?.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. })) {
        return Err(refuse(format!("the floor {floor} across the unselected edge is not planar; the wall is trimmed by its plane")));
    }
    Ok((
        OneEndChain { spine_vertex, e1: e1_id, mid_vertex, e2: e2_id, pad, floor, side_rail, third_convex: false, floor_rail: Some(floor_rail) },
        kept,
        replaced,
    ))
}

/// A plain OPEN end of a spine on the ring (the L prism's far end): the
/// spine's vertex is three-valent, its two other edges are unselected and
/// run along the spine's two mates onto one END FACE, a plane perpendicular
/// to the spine there, so the wall ends FLUSH: its last section lies in that
/// plane and becomes an edge of the end face.
struct OpenEnd {
    spine_vertex: u64,
    end_face: u64,
    /// The unselected edge on the FIRST mate and the end face, and the one
    /// on the second mate and the end face.
    edge_first: u64,
    edge_second: u64,
}

/// Read a plain open end at `spine_vertex`, or say why this end is not one.
/// `mates` is (first, second) in the spine's own mate order.
fn open_end(ring: &BrepSolid, spine: &EdgeRecord, spine_vertex: u64, mates: [u64; 2]) -> Result<OpenEnd, KernelRefusal> {
    let refuse = |what: &str, message: String| KernelRefusal::unsupported(KernelStage::Classify, what, format!("blend one-ended runout: {message}"));
    if continuation(ring, spine, spine_vertex)?.is_some() {
        return Err(refuse("open_end", format!("the spine {} continues smoothly past vertex {spine_vertex}, which is not an open end", spine.id)));
    }
    let others: Vec<u64> = edges_at(ring, spine_vertex).into_iter().filter(|id| *id != spine.id).collect();
    if others.len() != 2 {
        return Err(refuse("open_end", format!(
            "vertex {spine_vertex} carries {} edges beside the spine (an open end has exactly two)", others.len()
        )));
    }
    let mut on_mate = [None; 2];
    let mut end_face = None;
    for id in &others {
        let faces = faces_of(ring, *id);
        let Some(side) = (0..2).find(|k| faces.contains(&mates[*k])) else {
            return Err(refuse("open_end", format!("edge {id} at vertex {spine_vertex} runs along neither of the spine's mates")));
        };
        let Some(third) = faces.iter().copied().find(|f| *f != mates[side]) else {
            return Err(KernelRefusal::internal(KernelStage::Classify, "open_end", format!("blend one-ended runout: edge {id} has one face")));
        };
        if on_mate[side].is_some() {
            return Err(refuse("open_end", format!("both edges at vertex {spine_vertex} run along mate {}", mates[side])));
        }
        on_mate[side] = Some(*id);
        match end_face {
            None => end_face = Some(third),
            Some(face) if face == third => {}
            Some(face) => {
                return Err(refuse("open_end", format!(
                    "the two edges at vertex {spine_vertex} end on different faces ({face} and {third}), not on one end face"
                )));
            }
        }
    }
    let (Some(edge_first), Some(edge_second), Some(end_face)) = (on_mate[0], on_mate[1], end_face) else {
        return Err(refuse("open_end", format!("the edges at vertex {spine_vertex} do not run along both mates")));
    };
    if mates.contains(&end_face) {
        return Err(refuse("open_end", format!("the end face at vertex {spine_vertex} is one of the spine's own mates")));
    }
    // Flush: the end face is a plane whose normal is the spine's tangent at
    // the vertex, so the spine's last section plane IS the end face.
    let face = face_by_id(ring, end_face)?;
    let Some(crate::AnalyticSurface::Plane { u_dir, v_dir, .. }) = face.surface.analytic() else {
        return Err(refuse("open_end_oblique", format!(
            "the end face {end_face} at vertex {spine_vertex} is not planar; a flush open end needs the spine's section plane to be the end face"
        )));
    };
    let normal = u_dir.cross(*v_dir).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let tangent = tangent_away_from(spine, spine_vertex)?;
    let lean = 1.0 - normal.dot(tangent).abs();
    if lean > 1e-9 {
        return Err(refuse("open_end_oblique", format!(
            "the end face {end_face} at vertex {spine_vertex} leans 1 − |n·t| = {lean:.3e} off the spine's section plane; an oblique open end (the wall trimmed by the end face) is not built here"
        )));
    }
    Ok(OpenEnd { spine_vertex, end_face, edge_first, edge_second })
}

/// One end of a spine's ridge on the ring: a two-ended chain (seam and
/// pole), a one-ended end (the pad's end arc and the unselected edge), or
/// a plain open end (two unselected edges onto a flush end face).
enum RingEnd {
    Chain(EndChain),
    One(OneEndChain),
    Open(OpenEnd),
}

/// Read either end kind at `spine_vertex`.  A two-ended chain is read first
/// (the shipped class); the one-ended end and the plain open end only with
/// the switch on, and the chain's own refusal is kept when none reads.  An
/// open end keeps no mate of its own: its (kept, replaced) are the mates in
/// the spine's order and the caller takes the other end's.
fn read_end(
    ring: &BrepSolid,
    spine: &EdgeRecord,
    spine_vertex: u64,
    mates: [u64; 2],
    radius: f64,
) -> Result<(RingEnd, u64, u64), KernelRefusal> {
    match end_chain(ring, spine, spine_vertex, mates) {
        Ok((chain, kept, replaced)) => Ok((RingEnd::Chain(chain), kept, replaced)),
        Err(chain_error) => {
            if !one_ended_runout_on() {
                return Err(chain_error);
            }
            match one_ended_chain(ring, spine, spine_vertex, mates, radius) {
                Ok((chain, kept, replaced)) => Ok((RingEnd::One(chain), kept, replaced)),
                Err(one_error) => match open_end(ring, spine, spine_vertex, mates) {
                    Ok(end) => Ok((RingEnd::Open(end), mates[0], mates[1])),
                    Err(open_error) => Err(chain_error.with_message(|message| format!(
                        "{message}; nor a one-ended end: {one_error}; nor an open end: {open_error}"
                    ))),
                },
            }
        }
    }
}

/// Every ONE-ENDED mixed corner of a selection: a three-valent vertex where
/// exactly two selected edges meet, one convex and one concave, sharing one
/// face, the third edge unselected.  Reported as a [`MixedCorner`] with one
/// concave index, which `mixed_convexity_corners` (two or more concave) never
/// lists, so the runout composition can count the end.  Empty with the
/// switch off.
pub(crate) fn one_ended_runout_corners(solid: &BrepSolid, edge_ids: &[u64], radius: f64) -> Vec<MixedCorner> {
    let mut found = Vec::new();
    if !one_ended_runout_on() {
        return found;
    }
    let Ok(mates) = stripe_mates(solid, edge_ids, radius) else {
        return found;
    };
    let mut by_vertex: Vec<(u64, Vec<usize>)> = Vec::new();
    for (index, (edge, _, _)) in mates.iter().enumerate() {
        for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
            match by_vertex.iter_mut().find(|(id, _)| *id == vertex) {
                Some((_, indices)) => indices.push(index),
                None => by_vertex.push((vertex, vec![index])),
            }
        }
    }
    for (vertex_id, indices) in by_vertex {
        if indices.len() != 2 || edges_at(solid, vertex_id).len() != 3 {
            continue;
        }
        let (a, b) = (indices[0], indices[1]);
        let (Ok(a_convex), Ok(b_convex)) = (
            super::network::edge_is_convex(mates[a].0, &mates[a].1, &mates[a].2),
            super::network::edge_is_convex(mates[b].0, &mates[b].1, &mates[b].2),
        ) else {
            continue;
        };
        if a_convex == b_convex {
            continue;
        }
        let faces_a = [mates[a].1.face.id, mates[a].2.face.id];
        let faces_b = [mates[b].1.face.id, mates[b].2.face.id];
        if faces_a.iter().filter(|id| faces_b.contains(id)).count() != 1 {
            continue;
        }
        let Some(point) = solid.vertices.iter().find(|vertex| vertex.id == vertex_id).map(|vertex| vertex.point) else {
            continue;
        };
        let (convex, concave) = if a_convex { (a, b) } else { (b, a) };
        found.push(MixedCorner { vertex: vertex_id, point, convex: vec![convex], concave: vec![concave] });
    }
    found
}

fn face_by_id(solid: &BrepSolid, id: u64) -> Result<&FaceRecord, KernelRefusal> {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "runout_face", format!("blend runout: face {id} missing")))
}

fn edge_by_id(solid: &BrepSolid, id: u64) -> Result<&EdgeRecord, KernelRefusal> {
    solid
        .edges
        .iter()
        .find(|edge| edge.id == id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "runout_edge", format!("blend runout: edge {id} missing")))
}

fn vertex_point(solid: &BrepSolid, id: u64) -> Result<Vec3, KernelRefusal> {
    solid
        .vertices
        .iter()
        .find(|vertex| vertex.id == id)
        .map(|vertex| vertex.point)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "runout_vertex", format!("blend runout: vertex {id} missing")))
}

/// The faces whose loops use `edge_id`.
fn faces_of(solid: &BrepSolid, edge_id: u64) -> Vec<u64> {
    let mut out = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if face.loops.iter().any(|lp| lp.coedges.iter().any(|c| c.edge_id == edge_id)) {
            out.push(face.id);
        }
    }
    out
}

fn edges_at(solid: &BrepSolid, vertex: u64) -> Vec<u64> {
    solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate && (edge.start_vertex_id == vertex || edge.end_vertex_id == vertex))
        .map(|edge| edge.id)
        .collect()
}

fn other_vertex(edge: &EdgeRecord, vertex: u64) -> u64 {
    if edge.start_vertex_id == vertex {
        edge.end_vertex_id
    } else {
        edge.start_vertex_id
    }
}

/// The edge continuing `from` smoothly (G1) through `vertex`, if exactly one does.
fn continuation(solid: &BrepSolid, from: &EdgeRecord, vertex: u64) -> Result<Option<u64>, KernelRefusal> {
    let own = tangent_away_from(from, vertex)?;
    let mut found = None;
    for id in edges_at(solid, vertex) {
        if id == from.id {
            continue;
        }
        let edge = edge_by_id(solid, id)?;
        let other = tangent_away_from(edge, vertex)?;
        if own.dot(other) <= -(1.0 - 1e-6) {
            if found.is_some() {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
                    "blend runout: vertex {vertex} continues edge {} smoothly along more than one edge",
                    from.id
                )));
            }
            found = Some(id);
        }
    }
    Ok(found)
}

/// Read the ring's ridge past one end of the spine: kept ∩ pad_s, then
/// pad_r ∩ pad_s, then the pole.  `kept` and `replaced` are the spine's two
/// mates; which is which is decided HERE, by which mate the first chain edge
/// runs along.
fn end_chain(
    ring: &BrepSolid,
    spine: &EdgeRecord,
    spine_vertex: u64,
    mates: [u64; 2],
) -> Result<(EndChain, u64, u64), KernelRefusal> {
    let e1_id = continuation(ring, spine, spine_vertex)?.ok_or_else(|| KernelRefusal::unsupported(
        KernelStage::Classify, "runout_chain",
        format!("blend runout: the spine {} does not continue smoothly past vertex {spine_vertex} on the ring result", spine.id),
    ))?;
    let e1 = edge_by_id(ring, e1_id)?;
    let e1_faces = faces_of(ring, e1_id);
    let kept = *mates.iter().find(|id| e1_faces.contains(id)).ok_or_else(|| KernelRefusal::unsupported(
        KernelStage::Classify, "runout_chain",
        format!("blend runout: edge {e1_id} past the spine runs along neither of the spine's mates"),
    ))?;
    let replaced = if kept == mates[0] { mates[1] } else { mates[0] };
    let pad_s = *e1_faces.iter().find(|id| **id != kept).ok_or_else(|| KernelRefusal::internal(
        KernelStage::Classify, "runout_chain", format!("blend runout: edge {e1_id} has one face"),
    ))?;
    let mid_vertex = other_vertex(e1, spine_vertex);
    let e2_id = continuation(ring, e1, mid_vertex)?.ok_or_else(|| KernelRefusal::unsupported(
        KernelStage::Classify, "runout_chain",
        format!("blend runout: edge {e1_id} does not continue smoothly past vertex {mid_vertex}"),
    ))?;
    let e2 = edge_by_id(ring, e2_id)?;
    let e2_faces = faces_of(ring, e2_id);
    if !e2_faces.contains(&pad_s) {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
            "blend runout: the seam {e2_id} past vertex {mid_vertex} does not run along pad {pad_s}"
        )));
    }
    let pad_r = *e2_faces.iter().find(|id| **id != pad_s).ok_or_else(|| KernelRefusal::internal(
        KernelStage::Classify, "runout_chain", format!("blend runout: edge {e2_id} has one face"),
    ))?;
    let pole = other_vertex(e2, mid_vertex);
    if continuation(ring, e2, pole)?.is_some() {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
            "blend runout: the ridge continues smoothly past the pole vertex {pole}; the class is a ridge of exactly three edges per end"
        )));
    }
    // The floor: the face both pad rails at the pole lie on, other than the pads.
    let mut floor = None;
    let rails: Vec<u64> = edges_at(ring, pole).into_iter().filter(|id| *id != e2_id).collect();
    if rails.len() != 2 {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
            "blend runout: the pole vertex {pole} carries {} edges beside the seam (expected two pad rails)",
            rails.len()
        )));
    }
    let faces_a = faces_of(ring, rails[0]);
    let faces_b = faces_of(ring, rails[1]);
    for id in &faces_a {
        if faces_b.contains(id) && *id != pad_s && *id != pad_r {
            floor = Some(*id);
        }
    }
    let floor = floor.ok_or_else(|| KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
        "blend runout: the two pad rails at the pole {pole} share no face beside the pads"
    )))?;
    // pad_s's rail on the replaced mate ends at the spine's vertex.
    let side_rail = edges_at(ring, spine_vertex)
        .into_iter()
        .filter(|id| *id != spine.id && *id != e1_id)
        .find(|id| {
            let faces = faces_of(ring, *id);
            faces.contains(&replaced) && faces.contains(&pad_s)
        })
        .ok_or_else(|| KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
            "blend runout: no rail of pad {pad_s} on face {replaced} ends at vertex {spine_vertex}"
        )))?;
    // pad_r's rail on the kept mate ends at the mid vertex.
    let roof_rail = edges_at(ring, mid_vertex)
        .into_iter()
        .filter(|id| *id != e1_id && *id != e2_id)
        .find(|id| {
            let faces = faces_of(ring, *id);
            faces.contains(&kept) && faces.contains(&pad_r)
        })
        .ok_or_else(|| KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
            "blend runout: no rail of pad {pad_r} on face {kept} ends at vertex {mid_vertex}"
        )))?;
    Ok((
        EndChain {
            spine_vertex,
            e1: e1_id,
            mid_vertex,
            e2: e2_id,
            pole,
            pole_point: vertex_point(ring, pole)?,
            pad_s,
            pad_r,
            floor,
            side_rail,
            roof_rail,
        },
        kept,
        replaced,
    ))
}

/// A station of the runout wall from a converged `uv` on `(surface1, surface2)`.
fn make_station(
    surface1: &NurbsSurface,
    surface2: &NurbsSurface,
    rho: [f64; 2],
    uv: [f64; 4],
    edge: &EdgeRecord,
    t: f64,
) -> Result<Station, KernelRefusal> {
    let (section_point, section_tangent) = section_frame(edge, t)?;
    let (_, p1, p2, center) = tangency_residual(surface1, surface2, rho, uv, section_point, section_tangent)?;
    let n1 = raw_normal(surface1, uv[0], uv[1])?;
    let n2 = raw_normal(surface2, uv[2], uv[3])?;
    let cos_alpha = rho[0].signum() * rho[1].signum() * n1.dot(n2);
    let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
    if weight <= 1e-6 {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "tangent_station", "blend runout: the carriers are tangent at a station (α = π)"));
    }
    let apex = apex_point(p1, n1, p2, n2, center)?;
    Ok(Station { uv1: [uv[0], uv[1]], uv2: [uv[2], uv[3]], p1, p2, center, weight, apex })
}

/// The signed radius a convex ball marching on `surface` takes there, read
/// off a centre already seated against it: `centre = p + ρ·n_raw(p)`.
fn signed_rho(surface: &NurbsSurface, center: Vec3, radius: f64) -> Result<(f64, [f64; 2]), KernelRefusal> {
    let (_, u, v) = surface_gap(surface, center)?;
    let normal = raw_normal(surface, u, v)?;
    let point = surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
    Ok((radius * center.sub(point).dot(normal).signum(), [u, v]))
}

/// Measured residuals of one wall, for the record and the debug trace.
#[derive(Default, Debug, Clone)]
pub(crate) struct RunoutWallReport {
    pub(crate) spine: u64,
    /// Per end (t0 side then t1 side): the mirror-identity residuals at A and
    /// B against the ring's own pads, the pole station's two-centre miss, and
    /// how many stations the (pad, pad) stretch held before the pole.
    pub(crate) station_a_residual: [f64; 2],
    pub(crate) station_b_residual: [f64; 2],
    pub(crate) pole_center_miss: [f64; 2],
    pub(crate) pole_stretch_stations: [usize; 2],
    /// The switch vertices' distance off the pad rails they are trimmed on.
    pub(crate) switch_off_rail: [f64; 4],
    pub(crate) stations: usize,
    /// Worst `1 − t·t′` of a rail's one-sided unit tangents at a switch, and
    /// worst `1 − |n·n′|` of the wall's normal against a carrier's at a
    /// station.
    pub(crate) switch_turn: f64,
    pub(crate) plane_lean: f64,
    /// Worst `1 − n·n′` of the fitted wall's normals across a switch's break
    /// knot, along the section.
    pub(crate) switch_lean: f64,
}

thread_local! {
    static WALL_REPORTS: std::cell::RefCell<Vec<RunoutWallReport>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// The reports of the walls the last [`build_runout_walls`] built.
pub(crate) fn take_runout_reports() -> Vec<RunoutWallReport> {
    WALL_REPORTS.with(|reports| std::mem::take(&mut *reports.borrow_mut()))
}

/// One thing a spine's ridge claims on the ring: an EDGE (its ridge edge
/// kept∩pad, its pad∩pad seam, or the pad's side rail it trims) or its pole
/// VERTEX.  Edges share one id namespace whatever role they play, poles
/// another, so sharing is read per namespace, never per role: spine 1's ridge
/// edge and spine 2's side rail with the same id are the same edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ChainRole {
    Edge(&'static str),
    Pole,
}

impl ChainRole {
    fn namespace(&self) -> &'static str {
        match self {
            ChainRole::Edge(_) => "edge",
            ChainRole::Pole => "pole vertex",
        }
    }
    fn name(&self) -> &'static str {
        match self {
            ChainRole::Edge(what) => what,
            ChainRole::Pole => "pole",
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ChainClaim {
    pub(crate) spine: u64,
    pub(crate) role: ChainRole,
    pub(crate) id: u64,
}

/// The first pair of claims by DIFFERENT spines on one id of one namespace,
/// whatever their roles.  Sharing a ROOF rail is not a claim at all (it is
/// the reporter's own shape, each spine trimming that rail at its own end),
/// so roof rails are never passed in.
pub(crate) fn shared_chain_claim(claims: &[ChainClaim]) -> Option<(ChainClaim, ChainClaim)> {
    for (index, a) in claims.iter().enumerate() {
        for b in &claims[index + 1..] {
            if a.spine != b.spine && a.id == b.id && a.role.namespace() == b.role.namespace() {
                return Some((*a, *b));
            }
        }
    }
    None
}

/// Build the runout wall of every spine in `spines` on `ring`, the result of
/// blending the concave edges on their own: the composition §5 of the
/// 2026-09-30 runout record designs.  Each spine's wall runs from pole to pole
/// in the spine's own section planes, switching carriers at the tri-tangent
/// station A (solved against the floor, the face the two concave stripes
/// share) and at B (where the kept mate's contact reaches the other pad's rail,
/// a graze minimum), and closes on the ring's own pole vertices across a
/// degenerate edge.  The rails are split at the switches into one edge per
/// carrier; nothing is cut and nothing is intersected.
/// Row 2: the station where the ball on (`surface1`, `surface2`) GRAZES
/// `pad` — the minimum of |dist(centre, pad) − r| along the spine between
/// `t_from` and `t_to` (sampled, then golden-section), with its ball.
#[allow(clippy::too_many_arguments)]
fn graze_station(
    surface1: &NurbsSurface,
    surface2: &NurbsSurface,
    rho: [f64; 2],
    pad: &NurbsSurface,
    spine: &EdgeRecord,
    seed: [f64; 4],
    t_from: f64,
    t_to: f64,
    radius: f64,
    scale: f64,
) -> Result<(f64, Ball), KernelRefusal> {
    let graze = |t: f64, seed: [f64; 4]| -> Result<(f64, Ball), KernelRefusal> {
        let ball = solve_ball(surface1, surface2, rho, seed, spine, t, scale)?;
        Ok(((surface_gap(pad, ball.center)?.0 - radius).abs(), ball))
    };
    let mut seed = seed;
    let mut samples: Vec<(f64, f64, [f64; 4])> = Vec::new();
    for sample in 0..=STOP_SAMPLES {
        let t = t_from + (t_to - t_from) * sample as f64 / STOP_SAMPLES as f64;
        let Ok((value, ball)) = graze(t, seed) else { break };
        seed = ball.uv;
        samples.push((t, value, ball.uv));
    }
    let (index, _) = samples
        .iter()
        .enumerate()
        .min_by(|a, b| a.1 .1.partial_cmp(&b.1 .1).unwrap_or(std::cmp::Ordering::Equal))
        .filter(|(index, _)| *index > 0 && *index + 1 < samples.len())
        .ok_or_else(|| KernelRefusal::unsupported(KernelStage::Refine, ONE_ENDED_CONCAVE_THIRD_WHAT, format!(
            "blend one-ended runout (concave third): the ball never grazes the pad ({} stations marched)", samples.len()
        )))?;
    const GOLDEN: f64 = 0.618_033_988_749_894_9;
    let seed = samples[index].2;
    let (mut low, mut high) = (samples[index - 1].0, samples[index + 1].0);
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
    let (_, ball) = graze(t, seed)?;
    Ok((t, ball))
}

pub(crate) fn build_runout_walls(
    ring: &BrepSolid,
    spines: &[(u64, Option<String>)],
    radius: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let mut result = ring.clone();
    let mut take_id = fresh_id_source(ring);
    WALL_REPORTS.with(|reports| reports.borrow_mut().clear());
    // Every spine's chains are read from the RING while the walls are built
    // one after another in RESULT, so two spines whose ridges share a chain
    // edge, a pole or a side rail would have the second wall trim an edge the
    // first already consumed (its old vertex gone) and fail inside the
    // surgery as an internal error.  Sharing a ROOF rail is the reporter's own
    // shape (both spines run out on the tip pad and on the wall∩roof pad,
    // each trimming that rail at its own end) and is built; anything else
    // refuses by name before any wall is cut.
    {
        let mut claims: Vec<ChainClaim> = Vec::new();
        for (spine_id, _) in spines {
            let mates = stripe_mates(ring, &[*spine_id], radius)?;
            let (spine, first, second) = &mates[0];
            let mate_ids = [first.face.id, second.face.id];
            for vertex in [spine.start_vertex_id, spine.end_vertex_id] {
                let (end, _, _) = read_end(ring, spine, vertex, mate_ids, radius)?;
                match end {
                    RingEnd::Chain(chain) => {
                        for (role, id) in [(ChainRole::Edge("ridge edge"), chain.e1), (ChainRole::Edge("seam"), chain.e2), (ChainRole::Edge("side rail"), chain.side_rail), (ChainRole::Pole, chain.pole)] {
                            claims.push(ChainClaim { spine: *spine_id, role, id });
                        }
                    }
                    // A one-ended end cuts the pad's end arc and the side
                    // rail; the unselected edge is left as it is.
                    RingEnd::One(chain) => {
                        for (role, id) in [(ChainRole::Edge("ridge edge"), chain.e1), (ChainRole::Edge("side rail"), chain.side_rail)] {
                            claims.push(ChainClaim { spine: *spine_id, role, id });
                        }
                        // Row 2: the concave-third end also trims the
                        // unselected edge and the pad's floor rail.
                        if let Some(floor_rail) = chain.floor_rail {
                            for (role, id) in [(ChainRole::Edge("unselected edge"), chain.e2), (ChainRole::Edge("floor rail"), floor_rail)] {
                                claims.push(ChainClaim { spine: *spine_id, role, id });
                            }
                        }
                    }
                    // An open end trims the two unselected edges at its vertex.
                    RingEnd::Open(end) => {
                        for (role, id) in [(ChainRole::Edge("open-end edge"), end.edge_first), (ChainRole::Edge("open-end edge"), end.edge_second)] {
                            claims.push(ChainClaim { spine: *spine_id, role, id });
                        }
                    }
                }
            }
        }
        if let Some((a, b)) = shared_chain_claim(&claims) {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_shared_chain", format!(
                "blend runout: spines {} and {} share {} {} on the ring (as {} and {}); two runouts on one ridge are not built",
                a.spine, b.spine, a.role.namespace(), a.id, a.role.name(), b.role.name()
            )));
        }
    }
    for (spine_id, name) in spines {
        let report = build_runout_wall(ring, &mut result, &mut take_id, *spine_id, radius, name.as_deref())?;
        WALL_REPORTS.with(|reports| reports.borrow_mut().push(report));
    }
    prune_orphan_vertices(&mut result);
    Ok(result)
}

fn build_runout_wall(
    ring: &BrepSolid,
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    spine_id: u64,
    radius: f64,
    name: Option<&str>,
) -> Result<RunoutWallReport, KernelRefusal> {
    let debug = std::env::var("BREP_DEBUG_NETWORK").is_ok();
    let mates = stripe_mates(ring, &[spine_id], radius)?;
    let (spine, first, second) = &mates[0];
    let mate_ids = [first.face.id, second.face.id];
    // The ridge past each end, and which mate is KEPT past A (the one the
    // chain's first edge runs along).  Both ends must agree.
    let (end0_ring, kept0, replaced0) = read_end(ring, spine, spine.start_vertex_id, mate_ids, radius)?;
    let (end1_ring, kept1, replaced1) = read_end(ring, spine, spine.end_vertex_id, mate_ids, radius)?;
    // An open end keeps no mate: the other end decides.  Two open ends are
    // no runout at all.
    let (kept, replaced) = match (&end0_ring, &end1_ring) {
        (RingEnd::Open(_), RingEnd::Open(_)) => {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
                "blend runout: both ends of spine {spine_id} are open ends; the spine runs out at neither"
            )));
        }
        (RingEnd::Open(_), _) => (kept1, replaced1),
        (_, RingEnd::Open(_)) => (kept0, replaced0),
        _ => {
            // Row 2: a concave-third end keeps the mate its pad does NOT
            // replace, which need not be the other end's (the wedge's spine
            // A keeps the roof at its tip, the side at the wall); with the
            // switch on, each end keeps its own.
            let mixed_allowed = concave_third_runout_on()
                && [&end0_ring, &end1_ring].iter().any(|end| matches!(end, RingEnd::One(chain) if chain.floor_rail.is_some()));
            if kept0 != kept1 && !mixed_allowed {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_chain", format!(
                    "blend runout: the two ends of spine {spine_id} keep different mates ({kept0} and {kept1})"
                )));
            }
            (kept0, replaced0)
        }
    };
    let kept_is_first = kept == first.face.id;
    // Each end's kept mate (an open end takes the other end's): equal to
    // `kept` at both ends unless row 2's mixed case.
    let end_kept: [u64; 2] = match (&end0_ring, &end1_ring) {
        (RingEnd::Open(_), _) => [kept1, kept1],
        (_, RingEnd::Open(_)) => [kept0, kept0],
        _ => [kept0, kept1],
    };
    let end_kept_first = [end_kept[0] == first.face.id, end_kept[1] == first.face.id];
    let _ = replaced;
    // Each end runs out against ITS OWN shared face: the floor at the rib's
    // tip, the back wall at the other end.
    let ring_ends = [end0_ring, end1_ring];
    let scale = march_model_scale(&spine.curve, spine.t0, spine.t1, radius)?;
    let rho = [first.rho, second.rho];
    let surface1 = &first.face.surface;
    let surface2 = &second.face.surface;
    let seed_at = |t: f64| -> Result<[f64; 4], KernelRefusal> {
        let clamped = t.clamp(spine.t0.min(spine.t1), spine.t0.max(spine.t1));
        let uv1 = edge_uv_on_face(first.coedge, spine, clamped)?;
        let uv2 = edge_uv_on_face(second.coedge, spine, clamped)?;
        Ok([uv1[0], uv1[1], uv2[0], uv2[1]])
    };
    let t_mid = 0.5 * (spine.t0 + spine.t1);
    let mut report = RunoutWallReport { spine: spine_id, ..Default::default() };

    // ---- Per end: the pole's section, station A and station B, solved
    //      once.  The stretches between them are marched below on a station
    //      ladder.
    /// How a wall end terminates: at the ring's pole after station B (a
    /// two-ended chain), or at C5's stop with a planar cliff (a one-ended
    /// end).
    enum Terminus {
        Pole {
            t_b: f64,
            t_pole: f64,
            /// (pad_r, pad_s) carriers past B.
            last: (u64, u64, [f64; 2], [f64; 4]),
            pole_station: Station,
        },
        Cliff {
            t_stop: f64,
            stop: Station,
        },
        /// A flush open end: the spine's own last section, on the end face.
        Open {
            station: Station,
        },
        /// Row 2: a concave-third one-ended end — the section where the
        /// ball's PAD contact reaches the floor's plane; the wall is trimmed
        /// by that plane.
        WallTrim {
            t_end: f64,
            end: Station,
        },
    }
    struct EndSolve {
        t_a: f64,
        pad_s_first: bool,
        /// (kept, pad_s) carriers past A: surfaces by face id, signed radii
        /// and the seed at A.
        middle: (u64, u64, [f64; 2], [f64; 4]),
        terminus: Terminus,
    }
    let mut ends: Vec<EndSolve> = Vec::with_capacity(2);
    for (slot, ring_end) in ring_ends.iter().enumerate() {
        // This end's own kept mate (row 2; the same at both ends otherwise).
        let (kept, kept_is_first) = (end_kept[slot], end_kept_first[slot]);
        let vertex_t = if slot == 0 { spine.t0 } else { spine.t1 };
        // An open end has no station A: the wall runs on the spine's own
        // mates to the vertex's section, which lies in the end face.
        if let RingEnd::Open(end) = ring_end {
            let ball = solve_ball(surface1, surface2, rho, seed_at(vertex_t)?, spine, vertex_t, scale)?;
            let station = make_station(surface1, surface2, rho, ball.uv, spine, vertex_t)?;
            let end_face = face_by_id(ring, end.end_face)?;
            let mut off_face: f64 = 0.0;
            for point in [station.p1, station.p2, station.center] {
                off_face = off_face.max(surface_gap(&end_face.surface, point)?.0);
            }
            let band = crate::KernelTolerances::for_solid(ring, 1e-7).pcurve_consistency;
            if off_face > band {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "open_end_oblique", format!(
                    "blend one-ended runout: the spine {spine_id}'s last section at vertex {} stands {off_face:.3e} off the end face {} (band {band:.1e}); the open end is not flush",
                    end.spine_vertex, end.end_face
                )));
            }
            if debug {
                eprintln!(
                    "runout spine {spine_id} end {slot} (open): section at t={vertex_t:.6} on end face {} ({off_face:.3e} off it); p1 ({:.6}, {:.6}, {:.6}) p2 ({:.6}, {:.6}, {:.6})",
                    end.end_face, station.p1.x, station.p1.y, station.p1.z, station.p2.x, station.p2.y, station.p2.z
                );
            }
            ends.push(EndSolve {
                t_a: vertex_t,
                pad_s_first: !kept_is_first,
                middle: (first.face.id, second.face.id, rho, ball.uv),
                terminus: Terminus::Open { station },
            });
            continue;
        }
        // Station A is bracketed from the spine's middle to the pole's
        // section on a chain, and to half a spine past the vertex on a
        // one-ended end (the ball meets the floor's plane near the vertex).
        let (t_pole, floor_id, pad_s_id) = match ring_end {
            RingEnd::Chain(chain) => (section_parameter(spine, chain.pole_point, t_mid, vertex_t)?, chain.floor, chain.pad_s),
            RingEnd::One(chain) => (vertex_t + (vertex_t - t_mid), chain.floor, chain.pad),
            RingEnd::Open(_) => unreachable!("an open end was solved above"),
        };
        let floor = face_by_id(ring, floor_id)?;
        // A one-ended end reads the floor's distance on its PLANE (C5's
        // side-face extension): the convex ball's centre lies past the
        // floor's own patch there (under the slot floor on the channel), where
        // the projector clamps and the gap never closes.  A chain end reads
        // the projected distance exactly as before.
        let floor_plane: Option<(Vec3, Vec3)> = match ring_end {
            RingEnd::Chain(_) | RingEnd::Open(_) => None,
            RingEnd::One(chain) => {
                let Some(crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = floor.surface.analytic() else {
                    return Err(KernelRefusal::unsupported(KernelStage::Classify, "one_ended_floor", format!(
                        "blend one-ended runout: the floor face {floor_id} at vertex {} is not planar; C5's side-face extension is only exact on a plane",
                        chain.spine_vertex
                    )));
                };
                Some((*origin, u_dir.cross(*v_dir).normalized().or_refuse(KernelStage::Refine, "normalized")?))
            }
        };
        let floor_distance = |point: Vec3| -> Result<f64, KernelRefusal> {
            match floor_plane {
                Some((origin, normal)) => Ok(point.sub(origin).dot(normal).abs()),
                None => Ok(surface_gap(&floor.surface, point)?.0),
            }
        };
        // Station A: the convex ball on (first, second) tangent to the floor.
        let gap_at = |t: f64, seed: [f64; 4]| -> Result<(f64, Ball), KernelRefusal> {
            let ball = solve_ball(surface1, surface2, rho, seed, spine, t, scale)?;
            let gap = floor_distance(ball.center)? - radius;
            Ok((gap, ball))
        };
        let mut seed = seed_at(t_mid)?;
        let mut previous: Option<(f64, f64)> = None;
        let mut bracket: Option<(f64, f64)> = None;
        // A sample landing ON the root (a one-ended end whose ring vertex is
        // the tangency itself: the pad's rail at 5 + r, a station sample)
        // is the station; bisecting from a re-evaluated end whose rounding
        // flipped its sign walked to the far end of the bracket instead.
        let exact = 1e-12 * (1.0 + scale);
        let mut hit: Option<(f64, Ball)> = None;
        // Row 2: a concave-third end's station A is the PAD'S graze, solved
        // below; the floor tangency is not on its path.
        let concave_third = matches!(ring_end, RingEnd::One(chain) if chain.floor_rail.is_some());
        for sample in 0..=BRACKET_SAMPLES {
            if concave_third {
                break;
            }
            let t = t_mid + (t_pole - t_mid) * sample as f64 / BRACKET_SAMPLES as f64;
            let (value, ball) = gap_at(t, seed)?;
            seed = ball.uv;
            if matches!(ring_end, RingEnd::One(_)) && value.abs() <= exact {
                hit = Some((t, ball));
                break;
            }
            if let Some((previous_t, previous_value)) = previous {
                if previous_value.signum() != value.signum() {
                    bracket = Some((previous_t, t));
                    break;
                }
            }
            previous = Some((t, value));
        }
        let (t_a, ball_a) = if concave_third {
            graze_station(surface1, surface2, rho, &face_by_id(ring, pad_s_id)?.surface, spine, seed_at(t_mid)?, t_mid, t_pole, radius, scale)
                .map_err(|error| error.with_message(|message| format!("{message} (spine {spine_id}, the concave-third end at vertex {})", if slot == 0 { spine.start_vertex_id } else { spine.end_vertex_id })))?
        } else if let Some((t, ball)) = hit {
            (t, ball)
        } else {
        let (mut low, mut high) = bracket.ok_or_else(|| KernelRefusal::unsupported(KernelStage::Refine, "runout_station_a", format!(
            "blend runout: the ball on spine {spine_id} never becomes tangent to face {floor_id} toward vertex {}",
            if slot == 0 { spine.start_vertex_id } else { spine.end_vertex_id }
        )))?;
        let mut seed = seed_at(low)?;
        let (mut low_value, mut ball_a) = gap_at(low, seed)?;
        for _ in 0..BISECTIONS {
            let middle = 0.5 * (low + high);
            let (value, ball) = gap_at(middle, seed)?;
            seed = ball.uv;
            if value.signum() == low_value.signum() {
                low = middle;
                low_value = value;
            } else {
                high = middle;
            }
            ball_a = ball;
        }
        (0.5 * (low + high), ball_a)
        };
        let pad_s = face_by_id(ring, pad_s_id)?;
        report.station_a_residual[slot] = (surface_gap(&pad_s.surface, ball_a.center)?.0 - radius).abs();
        if debug {
            eprintln!(
                "runout spine {spine_id} end {slot}: station A t={t_a:.6} (spine t0 {:.4} t1 {:.4}, vertex t {vertex_t:.4}, window to {t_pole:.4}); centre ({:.6}, {:.6}, {:.6}) p1 ({:.6}, {:.6}, {:.6}) p2 ({:.6}, {:.6}, {:.6}); floor gap {:.3e}",
                spine.t0, spine.t1, ball_a.center.x, ball_a.center.y, ball_a.center.z, ball_a.p1.x, ball_a.p1.y, ball_a.p1.z, ball_a.p2.x, ball_a.p2.y, ball_a.p2.z,
                floor_distance(ball_a.center)? - radius
            );
        }
        // Past A the replaced mate's slot carries pad_s.
        let (pad_s_rho, pad_s_uv) = signed_rho(&pad_s.surface, ball_a.center, radius)?;
        let pad_s_first = !kept_is_first;
        let (middle_s1, middle_s2, middle_rho, middle_seed): (&NurbsSurface, &NurbsSurface, [f64; 2], [f64; 4]) = if pad_s_first {
            (&pad_s.surface, surface2, [pad_s_rho, rho[1]], [pad_s_uv[0], pad_s_uv[1], ball_a.uv[2], ball_a.uv[3]])
        } else {
            (surface1, &pad_s.surface, [rho[0], pad_s_rho], [ball_a.uv[0], ball_a.uv[1], pad_s_uv[0], pad_s_uv[1]])
        };
        let middle_ids = if pad_s_first { (pad_s_id, kept) } else { (kept, pad_s_id) };
        let terminus = match ring_end {
        RingEnd::Chain(chain) => {
        let pad_r = face_by_id(ring, chain.pad_r)?;
        // Station B: the graze minimum of the centre's distance to pad_r.
        let graze = |t: f64, seed: [f64; 4]| -> Result<(f64, Ball), KernelRefusal> {
            let ball = solve_ball(middle_s1, middle_s2, middle_rho, seed, spine, t, scale)?;
            let gap = surface_gap(&pad_r.surface, ball.center)?.0;
            Ok(((gap - radius).abs(), ball))
        };
        let mut seed = middle_seed;
        let mut samples: Vec<(f64, f64, [f64; 4])> = Vec::new();
        for sample in 0..=STOP_SAMPLES {
            let t = t_a + (t_pole - t_a) * sample as f64 / STOP_SAMPLES as f64;
            let Ok((value, ball)) = graze(t, seed) else { break };
            seed = ball.uv;
            samples.push((t, value, ball.uv));
        }
        let best = samples
            .iter()
            .enumerate()
            .min_by(|a, b| a.1 .1.partial_cmp(&b.1 .1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(index, sample)| (index, *sample))
            .filter(|(index, _)| *index > 0 && *index + 1 < samples.len())
            .ok_or_else(|| KernelRefusal::unsupported(KernelStage::Refine, "runout_station_b", format!(
                "blend runout: the kept mate's contact on spine {spine_id} never reaches pad {}'s rail toward vertex {} ({} stations marched)",
                chain.pad_r, chain.spine_vertex, samples.len()
            )))?;
        const GOLDEN: f64 = 0.618_033_988_749_894_9;
        let (index, _) = best;
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
        let t_b = 0.5 * (low + high);
        let (residual_b, ball_b) = graze(t_b, seed)?;
        report.station_b_residual[slot] = residual_b;
        // Past B the kept mate's slot carries pad_r.
        let (pad_r_rho, pad_r_uv) = signed_rho(&pad_r.surface, ball_b.center, radius)?;
        let (last_s1, last_s2, last_rho, last_seed): (&NurbsSurface, &NurbsSurface, [f64; 2], [f64; 4]) = if pad_s_first {
            (&pad_s.surface, &pad_r.surface, [middle_rho[0], pad_r_rho], [ball_b.uv[0], ball_b.uv[1], pad_r_uv[0], pad_r_uv[1]])
        } else {
            (&pad_r.surface, &pad_s.surface, [pad_r_rho, middle_rho[1]], [pad_r_uv[0], pad_r_uv[1], ball_b.uv[2], ball_b.uv[3]])
        };
        let last_ids = if pad_s_first { (chain.pad_s, chain.pad_r) } else { (chain.pad_r, chain.pad_s) };
        // The pole station, analytically: both contacts ARE the pole, the
        // centre sits one radius off it along either pad's normal.
        let pole = chain.pole_point;
        let (_, u1, v1) = surface_gap(last_s1, pole)?;
        let (_, u2, v2) = surface_gap(last_s2, pole)?;
        let n1 = raw_normal(last_s1, u1, v1)?;
        let n2 = raw_normal(last_s2, u2, v2)?;
        let center1 = pole.add(n1.scale(last_rho[0]));
        let center2 = pole.add(n2.scale(last_rho[1]));
        report.pole_center_miss[slot] = center1.sub(center2).length();
        let cos_alpha = last_rho[0].signum() * last_rho[1].signum() * n1.dot(n2);
        let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
        let pole_station = Station { uv1: [u1, v1], uv2: [u2, v2], p1: pole, p2: pole, center: center1, weight, apex: pole };
        // The march rides the spine's section planes, and a station's CENTRE
        // lies in its plane.  The pole ball's centre sits a radius off the
        // pole along the shared normal, which is not in the pole's own section
        // plane (0.46 out of it on the wedge's wall end), so the pole's
        // section is the one through that CENTRE: measured with the plane
        // through P instead, the marched contacts stood 0.7 from the pole at
        // every station count.
        let t_pole_center = section_parameter(spine, center1, t_mid, t_pole)?;
        if debug {
            eprintln!(
                "runout spine {spine_id} end {slot}: A t={t_a:.6} (pad residual {:.3e}), B t={t_b:.6} (residual {:.3e}), pole section t={t_pole:.6} through P, {t_pole_center:.6} through its centre; pole centre miss {:.3e}, weight {weight:.6}",
                report.station_a_residual[slot], residual_b, report.pole_center_miss[slot]
            );
        }
        Terminus::Pole {
            t_b,
            t_pole: t_pole_center,
            last: (last_ids.0, last_ids.1, last_rho, last_seed),
            pole_station,
        }
        }
        RingEnd::One(chain) => {
            // Past A the ball rolls on the pad, so A must BE the pad's graze:
            // true when the concave edge is right-angled (the mirror identity
            // of the module doc), where the ball tangent to the floor is the
            // reflection of the pad's own ball across the shared mate.  A
            // sloped pad (the wedge's wall ∩ roof) grazes elsewhere — the
            // two-ended lane's station B kind — and is named here.
            let graze_bar = 1e-9 * (1.0 + scale);
            if report.station_a_residual[slot] > graze_bar {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "one_ended_pad_graze", format!(
                    "blend one-ended runout: at vertex {} the ball tangent to the floor {floor_id} stands {:.3e} off the pad {}'s graze (bar {graze_bar:.1e}); the concave edge is not right-angled, so the one-ended end is not the floor-tangent station",
                    chain.spine_vertex, report.station_a_residual[slot], chain.pad
                )));
            }
            if chain.floor_rail.is_some() {
                // Row 2: past the graze the ball rolls on the kept mate
                // (extended) and the pad until its PAD contact reaches the
                // pad's rail on the floor.  The pad is TANGENT to the floor
                // along that rail, so neither the contact's height over the
                // floor (a double root) nor the ball's tangency to the floor
                // (identically true on the pad's linear extension past the
                // rail) is transversal there; the contact's own PARAMETER on
                // the pad is: it crosses the rail's iso-line at full speed.
                let (floor_origin, floor_normal) = floor_plane.expect("a one-ended end has a planar floor");
                let pad_contact = |ball: &Ball| if kept_is_first { ball.p2 } else { ball.p1 };
                let floor_rail = chain.floor_rail.expect("checked");
                let rail_iso = {
                    let pad_face = face_by_id(ring, chain.pad)?;
                    let coedge = pad_face.loops.iter().flat_map(|lp| &lp.coedges).find(|coedge| coedge.edge_id == floor_rail)
                        .ok_or_else(|| KernelRefusal::internal(KernelStage::Classify, "one_ended_chain", "blend one-ended runout: the pad does not use its floor rail"))?;
                    let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                    let a = coedge.pcurve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?;
                    let b = coedge.pcurve.evaluate(q1).or_refuse(KernelStage::Refine, "evaluate")?;
                    if (a.x - b.x).abs() <= (a.y - b.y).abs() { (0usize, 0.5 * (a.x + b.x)) } else { (1usize, 0.5 * (a.y + b.y)) }
                };
                let pad_uv = |ball: &Ball| if kept_is_first { [ball.uv[2], ball.uv[3]] } else { [ball.uv[0], ball.uv[1]] };
                let height = |t: f64, seed: [f64; 4]| -> Result<(f64, Ball), KernelRefusal> {
                    let ball = solve_ball(middle_s1, middle_s2, middle_rho, seed, spine, t, scale)?;
                    Ok((pad_uv(&ball)[rail_iso.0] - rail_iso.1, ball))
                };
                let chord = spine.curve.evaluate(spine.t1).or_refuse(KernelStage::Refine, "evaluate")?
                    .sub(spine.curve.evaluate(spine.t0).or_refuse(KernelStage::Refine, "evaluate")?).length();
                let speed = (chord / (spine.t1 - spine.t0).abs()).max(1e-300);
                let direction = (vertex_t - t_mid).signum();
                let t_far = t_a + direction * 4.0 * radius / speed;
                let mut seed = middle_seed;
                let (mut low, mut low_value) = (t_a, height(t_a, seed)?.0);
                let mut bracket: Option<(f64, f64)> = None;
                let mut hit: Option<(f64, Ball)> = None;
                for sample in 1..=STOP_SAMPLES {
                    let t = t_a + (t_far - t_a) * sample as f64 / STOP_SAMPLES as f64;
                    let (value, ball) = height(t, seed)?;
                    seed = ball.uv;
                    if value.abs() <= exact {
                        hit = Some((t, ball));
                        break;
                    }
                    if value.signum() != low_value.signum() {
                        bracket = Some((low, t));
                        break;
                    }
                    low = t;
                    low_value = value;
                }
                let (t_end, ball_end) = if let Some(hit) = hit {
                    hit
                } else {
                    let (mut low, mut high) = bracket.ok_or_else(|| KernelRefusal::unsupported(KernelStage::Refine, ONE_ENDED_CONCAVE_THIRD_WHAT, format!(
                        "blend one-ended runout (concave third): the pad contact on spine {spine_id} never reaches the floor {floor_id}'s plane within 4 r past the graze"
                    )))?;
                    let (mut low_value, mut ball_end) = height(low, seed)?;
                    for _ in 0..BISECTIONS {
                        let middle = 0.5 * (low + high);
                        let (value, ball) = height(middle, seed)?;
                        seed = ball.uv;
                        if value.signum() == low_value.signum() {
                            low = middle;
                            low_value = value;
                        } else {
                            high = middle;
                        }
                        ball_end = ball;
                    }
                    (0.5 * (low + high), ball_end)
                };
                // Reported: the pad contact's distance off the floor's plane
                // (zero at the rail).
                report.station_b_residual[slot] = pad_contact(&ball_end).sub(floor_origin).dot(floor_normal).abs();
                let end = make_station(middle_s1, middle_s2, middle_rho, ball_end.uv, spine, t_end)?;
                if debug {
                    eprintln!(
                        "runout spine {spine_id} end {slot} (concave third): graze A t={t_a:.6} (pad residual {:.3e}), wall terminus t={t_end:.6} (pad contact {:.3e} off the floor's plane) centre ({:.6}, {:.6}, {:.6})",
                        report.station_a_residual[slot], report.station_b_residual[slot], end.center.x, end.center.y, end.center.z
                    );
                }
                Terminus::WallTrim { t_end, end }
            } else {
            if !chain.third_convex {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "one_ended_concave_third", format!(
                    "blend one-ended runout: the unselected edge {} at vertex {} is concave; the roll on the kept mate extended to the pad's far rail, trimmed by the floor, is not built here (the convex case ends at C5's extended edge with a cliff)",
                    chain.e2, chain.spine_vertex
                )));
            }
            // C5's stop: the KEPT contact reaches the floor's plane extended,
            // which is the unselected edge's line extended (that edge is
            // kept ∩ floor).
            let (floor_origin, floor_normal) = floor_plane.expect("a one-ended end has a planar floor");
            let kept_contact = |ball: &Ball| if kept_is_first { ball.p1 } else { ball.p2 };
            let height = |t: f64, seed: [f64; 4]| -> Result<(f64, Ball), KernelRefusal> {
                let ball = solve_ball(middle_s1, middle_s2, middle_rho, seed, spine, t, scale)?;
                Ok((kept_contact(&ball).sub(floor_origin).dot(floor_normal), ball))
            };
            // The stop lies within a few radii of A along the spine's
            // direction; the window is read in the spine's own parameter.
            let chord = spine.curve.evaluate(spine.t1).or_refuse(KernelStage::Refine, "evaluate")?
                .sub(spine.curve.evaluate(spine.t0).or_refuse(KernelStage::Refine, "evaluate")?).length();
            let speed = (chord / (spine.t1 - spine.t0).abs()).max(1e-300);
            let direction = (vertex_t - t_mid).signum();
            let t_far = t_a + direction * 2.5 * radius / speed;
            let mut seed = middle_seed;
            let (mut low, mut low_value) = (t_a, f64::NAN);
            let mut bracket: Option<(f64, f64)> = None;
            let mut hit: Option<(f64, Ball)> = None;
            for sample in 0..=STOP_SAMPLES {
                let t = t_a + (t_far - t_a) * sample as f64 / STOP_SAMPLES as f64;
                let (value, ball) = height(t, seed)?;
                seed = ball.uv;
                if sample > 0 && value.abs() <= exact {
                    hit = Some((t, ball));
                    break;
                }
                if sample > 0 && value.signum() != low_value.signum() {
                    bracket = Some((low, t));
                    break;
                }
                low = t;
                low_value = value;
            }
            let (t_stop, ball_stop) = if let Some(hit) = hit {
                hit
            } else {
            let (mut low, mut high) = bracket.ok_or_else(|| KernelRefusal::unsupported(KernelStage::Refine, "one_ended_stop", format!(
                "blend one-ended runout: the kept mate's contact on spine {spine_id} never reaches the unselected edge {}'s line within 2.5 r past station A",
                chain.e2
            )))?;
            let (mut low_value, mut ball_stop) = height(low, seed)?;
            for _ in 0..BISECTIONS {
                let middle = 0.5 * (low + high);
                let (value, ball) = height(middle, seed)?;
                seed = ball.uv;
                if value.signum() == low_value.signum() {
                    low = middle;
                    low_value = value;
                } else {
                    high = middle;
                }
                ball_stop = ball;
            }
            (0.5 * (low + high), ball_stop)
            };
            report.station_b_residual[slot] = kept_contact(&ball_stop).sub(floor_origin).dot(floor_normal).abs();
            let stop = make_station(middle_s1, middle_s2, middle_rho, ball_stop.uv, spine, t_stop)?;
            if debug {
                eprintln!(
                    "runout spine {spine_id} end {slot} (one-ended): A t={t_a:.6} (pad residual {:.3e}), C5 stop t={t_stop:.6} (kept contact {:.3e} off the unselected edge's plane) at ({:.6}, {:.6}, {:.6})",
                    report.station_a_residual[slot], report.station_b_residual[slot], kept_contact(&ball_stop).x, kept_contact(&ball_stop).y, kept_contact(&ball_stop).z
                );
            }
            Terminus::Cliff { t_stop, stop }
            }
        }
        RingEnd::Open(_) => unreachable!("an open end was solved above"),
        };
        ends.push(EndSolve {
            t_a,
            pad_s_first,
            middle: (middle_ids.0, middle_ids.1, middle_rho, middle_seed),
            terminus,
        });
    }
    let [end0, end1] = match ends.len() {
        2 => {
            let end1 = ends.pop().expect("two ends");
            let end0 = ends.pop().expect("two ends");
            [end0, end1]
        }
        _ => return Err(KernelRefusal::internal(KernelStage::Refine, "runout_ends", "blend runout: two ends expected")),
    };
    if !((end1.t_a - end0.t_a) * (spine.t1 - spine.t0) > 0.0) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "runout_stations_cross", format!(
            "blend runout: the two tri-tangent stations of spine {spine_id} cross each other (t {:.6} and {:.6}); the spine is shorter than its two runouts",
            end0.t_a, end1.t_a
        )));
    }

    // ---- The five stretches, marched on a station ladder: each stretch
    //      doubles its stations until the rails fitted through them stand on
    //      their carriers to the network's own rail bar (`rails_off_carriers`
    //      sampling, half `intersection_fit`), read between the stations. ----
    let rail_bar = 0.5 * crate::KernelTolerances::for_solid(ring, 1e-7).intersection_fit;
    let span = (spine.t1 - spine.t0).abs();
    let count_for = |t_from: f64, t_to: f64| -> usize {
        ((((t_to - t_from).abs() / span) * RUNOUT_DENSITY_STATIONS as f64).ceil() as usize)
            .max(MIN_STRETCH_STATIONS - 1)
    };
    // Stretch k: (first carrier, second carrier, rho, seed at t_from, t_from, t_to, pole at the far end?)
    struct StretchSpec {
        first: u64,
        second: u64,
        rho: [f64; 2],
        seed: [f64; 4],
        t_from: f64,
        t_to: f64,
        /// `Some(pole station)` when `t_to` is the pole and the march stops
        /// short of it on the singular Newton.
        pole: Option<Station>,
        /// Seed at the MIDDLE and march outward (the centre stretch).
        seed_middle: bool,
    }
    // An end's stretches, outermost first: a chain's last stretch (to the
    // pole) and its middle stretch (A to B); a one-ended end's middle
    // stretch alone (A to the C5 stop).
    let end_specs = |end: &EndSolve| -> Vec<StretchSpec> {
        match &end.terminus {
            Terminus::Pole { t_b, t_pole, last, pole_station } => vec![
                StretchSpec { first: last.0, second: last.1, rho: last.2, seed: last.3, t_from: *t_b, t_to: *t_pole, pole: Some(*pole_station), seed_middle: false },
                StretchSpec { first: end.middle.0, second: end.middle.1, rho: end.middle.2, seed: end.middle.3, t_from: end.t_a, t_to: *t_b, pole: None, seed_middle: false },
            ],
            Terminus::Cliff { t_stop: t_to, .. } | Terminus::WallTrim { t_end: t_to, .. } => vec![
                StretchSpec { first: end.middle.0, second: end.middle.1, rho: end.middle.2, seed: end.middle.3, t_from: end.t_a, t_to: *t_to, pole: None, seed_middle: false },
            ],
            // An open end adds no stretch: the centre stretch reaches it.
            Terminus::Open { .. } => Vec::new(),
        }
    };
    let mut specs: Vec<StretchSpec> = end_specs(&end0);
    let n0 = specs.len();
    specs.push(StretchSpec { first: first.face.id, second: second.face.id, rho, seed: seed_at(t_mid)?, t_from: end0.t_a, t_to: end1.t_a, pole: None, seed_middle: true });
    let mut end1_specs = end_specs(&end1);
    end1_specs.reverse();
    specs.extend(end1_specs);
    // The end-0 stretches are marched from the middle outward (toward
    // decreasing t) and reversed into increasing t below.
    let march_stretch = |spec: &StretchSpec, count: usize| -> Result<Vec<Station>, KernelRefusal> {
        let s1 = &face_by_id(ring, spec.first)?.surface;
        let s2 = &face_by_id(ring, spec.second)?.surface;
        // Toward a pole the stations are spaced quadratically in the distance
        // to the pole's section, on the expectation that the two contacts
        // close on the pole like a square root (the ball is tangent to both
        // pads AT the pole).  Added while the pole's section was still taken
        // through P, where the last chord stood 2.6e-3 off the pads at 4, 33
        // and 1024 stations alike; the spacing did not move that stall — the
        // section through the pole ball's CENTRE did (see `t_pole_center`).
        // Kept, not separately A/B'd against uniform spacing.
        let t_of = |k: usize| {
            let fraction = k as f64 / count as f64;
            let fraction = if spec.pole.is_some() { 1.0 - (1.0 - fraction) * (1.0 - fraction) } else { fraction };
            spec.t_from + (spec.t_to - spec.t_from) * fraction
        };
        let mut out: Vec<Station> = Vec::with_capacity(count + 1);
        if spec.seed_middle {
            let mut solved: Vec<Option<[f64; 4]>> = vec![None; count + 1];
            let mid = count / 2;
            let mut seed = spec.seed;
            for k in (0..=mid).rev() {
                let (section_point, section_tangent) = section_frame(spine, t_of(k))?;
                let uv = solve_station(s1, s2, spec.rho, seed, section_point, section_tangent, scale)?;
                seed = uv;
                solved[k] = Some(uv);
            }
            let mut seed = solved[mid].expect("middle solved");
            for k in mid + 1..=count {
                let (section_point, section_tangent) = section_frame(spine, t_of(k))?;
                let uv = solve_station(s1, s2, spec.rho, seed, section_point, section_tangent, scale)?;
                seed = uv;
                solved[k] = Some(uv);
            }
            for (k, uv) in solved.into_iter().enumerate() {
                out.push(make_station(s1, s2, spec.rho, uv.expect("solved"), spine, t_of(k))?);
            }
            return Ok(out);
        }
        let mut seed = spec.seed;
        let last_regular = if spec.pole.is_some() { count - 1 } else { count };
        for k in 0..=last_regular {
            let (section_point, section_tangent) = section_frame(spine, t_of(k))?;
            let uv = match solve_station(s1, s2, spec.rho, seed, section_point, section_tangent, scale) {
                Ok(uv) => uv,
                Err(error) => {
                    if spec.pole.is_none() || k < MIN_STRETCH_STATIONS - 1 {
                        return Err(error.with_message(|error| format!(
                            "blend runout: spine {spine_id}'s stretch from t {:.6} to {:.6} holds only {k} stations: {error}",
                            spec.t_from, spec.t_to
                        )));
                    }
                    break;
                }
            };
            seed = uv;
            out.push(make_station(s1, s2, spec.rho, uv, spine, t_of(k))?);
        }
        if let Some(pole) = &spec.pole {
            out.push(*pole);
        }
        Ok(out)
    };
    let mut counts: Vec<usize> = specs.iter().map(|spec| count_for(spec.t_from, spec.t_to)).collect();
    const MAX_STRETCH_STATIONS: usize = 64;
    let (stations, ranges, parameters, rows, stretches) = loop {
        let mut stretches: Vec<Stretch> = Vec::with_capacity(5);
        for (index, spec) in specs.iter().enumerate() {
            let mut marched = march_stretch(spec, counts[index])?;
            if index < n0 {
                marched.reverse();
            }
            stretches.push(Stretch { first: spec.first, second: spec.second, stations: marched });
        }
        // Global station list (switch stations once) with each stretch's index range.
        let mut stations: Vec<Station> = Vec::new();
        let mut ranges: Vec<[usize; 2]> = Vec::new();
        for (index, stretch) in stretches.iter().enumerate() {
            let skip = usize::from(index > 0);
            let start = stations.len().saturating_sub(skip);
            stations.extend(stretch.stations.iter().skip(skip).cloned());
            ranges.push([start, stations.len() - 1]);
        }
        let breaks: Vec<usize> = ranges.iter().skip(1).map(|range| range[0]).collect();
        let parameters = station_parameters(&stations);
        let rows = fit_rows_breaking(&stations, &parameters, &breaks)?;
        // Each stretch's rails off its own carriers, between its stations.
        let mut over: Vec<usize> = Vec::new();
        let mut worst_all: f64 = 0.0;
        for (index, (stretch, range)) in stretches.iter().zip(&ranges).enumerate() {
            let s1 = &face_by_id(ring, stretch.first)?.surface;
            let s2 = &face_by_id(ring, stretch.second)?.surface;
            let mut worst: f64 = 0.0;
            for pair in parameters[range[0]..=range[1]].windows(2) {
                for fraction in [0.25, 0.5, 0.75] {
                    let u = pair[0] + (pair[1] - pair[0]) * fraction;
                    let p1 = rows.cr.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
                    let p2 = rows.cs.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
                    worst = worst.max(surface_gap(s1, p1)?.0).max(surface_gap(s2, p2)?.0);
                }
            }
            if debug {
                eprintln!(
                    "runout spine {spine_id}: stretch {index} on faces ({}, {}) at {} stations: rails {worst:.3e} off their carriers (bar {rail_bar:.3e})",
                    stretch.first, stretch.second, stretch.stations.len()
                );
                if worst > rail_bar {
                    let range_of = |pick: fn(&Station) -> [f64; 2]| {
                        stretch.stations.iter().fold([f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY], |acc, st| {
                            let uv = pick(st);
                            [acc[0].min(uv[0]), acc[1].max(uv[0]), acc[2].min(uv[1]), acc[3].max(uv[1])]
                        })
                    };
                    for (which, surface, uv_range) in [("first", s1, range_of(|st| st.uv1)), ("second", s2, range_of(|st| st.uv2))] {
                        eprintln!(
                            "    {which} carrier domain u {:?} v {:?}; stations span u [{:.4}, {:.4}] v [{:.4}, {:.4}]",
                            surface.domain_u().unwrap_or([f64::NAN; 2]), surface.domain_v().unwrap_or([f64::NAN; 2]),
                            uv_range[0], uv_range[1], uv_range[2], uv_range[3]
                        );
                    }
                    // Where the worst sample sits: which interval, and the two stations' contacts.
                    let mut worst_at = (0usize, 0.0f64);
                    for (k, pair) in parameters[range[0]..=range[1]].windows(2).enumerate() {
                        for fraction in [0.25, 0.5, 0.75] {
                            let u = pair[0] + (pair[1] - pair[0]) * fraction;
                            let p1 = rows.cr.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
                            let p2 = rows.cs.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
                            let off = surface_gap(s1, p1)?.0.max(surface_gap(s2, p2)?.0);
                            if off > worst_at.1 { worst_at = (k, off); }
                        }
                    }
                    let (k, off) = worst_at;
                    let a = &stretch.stations[k];
                    let b = &stretch.stations[k + 1];
                    eprintln!(
                        "    worst {off:.3e} in interval {k}/{}: p1 ({:.5},{:.5},{:.5})->({:.5},{:.5},{:.5}) p2 ({:.5},{:.5},{:.5})->({:.5},{:.5},{:.5}) weights {:.4}->{:.4}",
                        stretch.stations.len() - 1,
                        a.p1.x, a.p1.y, a.p1.z, b.p1.x, b.p1.y, b.p1.z, a.p2.x, a.p2.y, a.p2.z, b.p2.x, b.p2.y, b.p2.z, a.weight, b.weight
                    );
                }
            }
            worst_all = worst_all.max(worst);
            if worst > rail_bar {
                over.push(index);
            }
        }
        if over.is_empty() {
            break (stations, ranges, parameters, rows, stretches);
        }
        if over.iter().any(|index| counts[*index] >= MAX_STRETCH_STATIONS) {
            return Err(KernelRefusal::non_convergence(KernelStage::Refine, super::miter::MARCHED_FIT_OFF_CARRIERS_WHAT, format!(
                "blend runout: spine {spine_id}'s rails still stand {worst_all:.3e} off their carriers (bar {rail_bar:.3e}) with {MAX_STRETCH_STATIONS} stations on a stretch"
            )));
        }
        for index in over {
            counts[index] *= 2;
        }
    };
    report.stations = stations.len();
    let breaks: Vec<usize> = ranges.iter().skip(1).map(|range| range[0]).collect();
    // ---- G1 at every switch.  The RAIL kinks there and that is the
    //      geometry: a contact is the centre displaced by r along the
    //      carrier's normal, and that normal's rate jumps from zero on a plane
    //      to its value on the pad cylinder (measured: the unit tangents' dot
    //      reads 0.946 at B on both shapes).  What is G1 is the WALL: the
    //      centre path is G1 across carriers that are themselves tangent,
    //      so the envelope's tangent plane is continuous across the switch.
    //      Read off the FITTED surface: its normal just before and just
    //      after each break knot, along the whole section, must agree to the
    //      network's own reading of a smooth continuation (1 − 1e-6 on the
    //      unit normals' dot, `tangent_away_from`'s bar), not a bar set here.
    //      The rails' turns are reported, never judged. ----
    let mut worst_switch_turn: f64 = 0.0;
    let mut worst_switch_lean: f64 = 0.0;
    let [u_low, u_high] = rows.u_domain;
    let u_step = 1e-6 * (u_high - u_low).abs();
    for &index in &breaks {
        let u = parameters[index];
        for row in [&rows.cr, &rows.cs] {
            let (left, right) = row.split(u).or_refuse(KernelStage::Refine, "split")?;
            let [_, left_end] = left.domain().or_refuse(KernelStage::Refine, "domain")?;
            let [right_start, _] = right.domain().or_refuse(KernelStage::Refine, "domain")?;
            let before = left.derivatives(left_end, 1).or_refuse(KernelStage::Refine, "derivatives")?[1];
            let after = right.derivatives(right_start, 1).or_refuse(KernelStage::Refine, "derivatives")?[1];
            if let (Ok(before), Ok(after)) = (before.normalized(), after.normalized()) {
                worst_switch_turn = worst_switch_turn.max((1.0 - before.dot(after)).max(0.0));
            }
        }
        for k in 0..=8 {
            let v = k as f64 / 8.0;
            let before = raw_normal(&rows.surface, u - u_step, v)?;
            let after = raw_normal(&rows.surface, u + u_step, v)?;
            let lean = (1.0 - before.dot(after)).max(0.0);
            worst_switch_lean = worst_switch_lean.max(lean);
            if before.dot(after) < 1.0 - 1e-6 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "runout_switch_not_g1", format!(
                    "blend runout: spine {spine_id}'s wall is not G1 across its carrier switch at station {index} (v = {v:.3}: unit normals' dot {:.9} across the break)",
                    before.dot(after)
                )));
            }
        }
    }
    report.switch_lean = worst_switch_lean;
    report.switch_turn = worst_switch_turn;
    // The wall's tangent plane against each carrier's at the contacts: the
    // section arc is tangent to both carriers by construction, read back off
    // the FITTED surface at every station.
    let mut worst_plane_lean: f64 = 0.0;
    for (stretch, range) in stretches.iter().zip(&ranges) {
        let s1 = &face_by_id(ring, stretch.first)?.surface;
        let s2 = &face_by_id(ring, stretch.second)?.surface;
        for (j, station) in stretch.stations.iter().enumerate() {
            let u = parameters[range[0] + j];
            if station.p1.sub(station.p2).length() < 1e-9 * (1.0 + scale) {
                continue; // the pole: the section is a point
            }
            for (v, surface, uv) in [(0.0, s1, station.uv1), (1.0, s2, station.uv2)] {
                let Ok(wall) = raw_normal(&rows.surface, u, v) else { continue };
                let carrier = raw_normal(surface, uv[0], uv[1])?;
                worst_plane_lean = worst_plane_lean.max(1.0 - wall.dot(carrier).abs());
            }
        }
    }
    report.plane_lean = worst_plane_lean;
    if debug {
        eprintln!(
            "runout spine {spine_id}: rails turn at the switches by at most 1 − t·t′ = {worst_switch_turn:.3e} (the geometry); the wall's normal across a switch moves by at most 1 − n·n′ = {worst_switch_lean:.3e}; the wall's tangent plane leans off its carriers by at most 1 − |n·n′| = {worst_plane_lean:.3e}"
        );
    }
    if debug {
        let mut worst: f64 = 0.0;
        for (station, u) in stations.iter().zip(&parameters) {
            worst = worst
                .max(rows.cr.evaluate(*u).or_refuse(KernelStage::Refine, "evaluate")?.sub(station.p1).length())
                .max(rows.cs.evaluate(*u).or_refuse(KernelStage::Refine, "evaluate")?.sub(station.p2).length());
        }
        eprintln!("runout spine {spine_id}: {} stations, breaks {breaks:?}, rows reproduce their stations to {worst:.3e}", stations.len());
    }

    // ---- The rail pieces: one edge per run of stretches sharing a carrier
    //      on that side. ----
    struct Piece {
        carrier: u64,
        /// Global station index range, inclusive.
        range: [usize; 2],
        /// uv on `carrier` per station of the range.
        uvs: Vec<[f64; 2]>,
    }
    // The uv samples come from each STRETCH's own stations: at a switch the
    // global list keeps the earlier stretch's copy of the shared station,
    // whose uv on the switching side is on the earlier carrier.
    let pieces_on = |side_first: bool| -> Vec<Piece> {
        let mut pieces: Vec<Piece> = Vec::new();
        for (stretch, range) in stretches.iter().zip(&ranges) {
            let carrier = if side_first { stretch.first } else { stretch.second };
            let uvs: Vec<[f64; 2]> = stretch.stations.iter().map(|s| if side_first { s.uv1 } else { s.uv2 }).collect();
            match pieces.last_mut() {
                Some(last) if last.carrier == carrier => {
                    last.range[1] = range[1];
                    last.uvs.extend(uvs.into_iter().skip(1));
                }
                _ => pieces.push(Piece { carrier, range: *range, uvs }),
            }
        }
        pieces
    };
    let pieces_cr = pieces_on(true);
    let pieces_cs = pieces_on(false);
    // A chain end adds a pad piece on both sides, a one-ended end on the
    // replaced side only, an open end on neither: the replaced side has one
    // piece more than its runout ends, the kept side one more than its
    // chain ends.
    // Counted per side and per end (row 2: the ends may keep different
    // mates): a side gains a piece at every non-open end where it is
    // replaced, and at every chain end where it is kept.
    let expected_pieces = |first_side: bool| -> usize {
        1 + ring_ends.iter().enumerate().filter(|(slot, end)| {
            let kept_here = end_kept_first[*slot] == first_side;
            (!matches!(end, RingEnd::Open(_)) && !kept_here) || (matches!(end, RingEnd::Chain(_)) && kept_here)
        }).count()
    };
    let (expected_cr, expected_cs) = (expected_pieces(true), expected_pieces(false));
    if pieces_cr.len() != expected_cr || pieces_cs.len() != expected_cs {
        let (kept_pieces, replaced_pieces) = if kept_is_first { (pieces_cr.len(), pieces_cs.len()) } else { (pieces_cs.len(), pieces_cr.len()) };
        let (kept_expected, replaced_expected) = if kept_is_first { (expected_cr, expected_cs) } else { (expected_cs, expected_cr) };
        return Err(KernelRefusal::internal(KernelStage::Refine, "runout_pieces", format!(
            "blend runout: spine {spine_id}'s rails split into {kept_pieces} kept-side and {replaced_pieces} replaced-side pieces ({kept_expected} and {replaced_expected} expected)"
        )));
    }

    // ---- Topology. ----
    // Each end's two rail vertices (first-mate side, second-mate side): the
    // ring's pole for a chain end, two fresh vertices at the stop station's
    // contacts for a one-ended end.
    let mut end_vertices = [[0u64; 2]; 2];
    // Row 2: at a wall-trim end the kept rail stops where it crosses the
    // floor's plane (u_K on the kept row): (u_K, the trim's u at the pad
    // contact) per slot.
    let mut wall_trim_u: [Option<(f64, f64)>; 2] = [None, None];
    for (slot, (ring_end, end)) in ring_ends.iter().zip([&end0, &end1]).enumerate() {
        let (RingEnd::One(chain), Terminus::WallTrim { .. }) = (ring_end, &end.terminus) else { continue };
        let kept_row = if end_kept_first[slot] { &rows.cr } else { &rows.cs };
        let floor = face_by_id(ring, chain.floor)?;
        let Some(crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = floor.surface.analytic() else {
            return Err(KernelRefusal::internal(KernelStage::Refine, "runout_wall_trim", "blend runout: a wall-trim floor is not planar"));
        };
        let normal = u_dir.cross(*v_dir).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let height = |u: f64| -> Result<f64, KernelRefusal> {
            Ok(kept_row.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?.sub(*origin).dot(normal))
        };
        // The end's own stretch, outermost first in parameter order.
        let range = if slot == 0 { ranges[0] } else { ranges[ranges.len() - 1] };
        let u_end = if slot == 0 { parameters[range[0]] } else { parameters[range[1]] };
        let u_a = if slot == 0 { parameters[range[1]] } else { parameters[range[0]] };
        // Walk from A toward the end until the kept contact's height changes
        // sign, then bisect on the row itself.
        let steps = 4 * (range[1] - range[0]).max(1);
        let mut low = u_a;
        let mut low_value = height(low)?;
        let mut found = None;
        for k in 1..=steps {
            let u = u_a + (u_end - u_a) * k as f64 / steps as f64;
            let value = height(u)?;
            if value.signum() != low_value.signum() {
                found = Some((low, u));
                break;
            }
            low = u;
            low_value = value;
        }
        let (mut low, mut high) = found.ok_or_else(|| KernelRefusal::unsupported(KernelStage::Refine, ONE_ENDED_CONCAVE_THIRD_WHAT, format!(
            "blend one-ended runout (concave third): the kept rail of spine {spine_id} never crosses the floor {}'s plane before the terminus", chain.floor
        )))?;
        for _ in 0..BISECTIONS {
            let middle = 0.5 * (low + high);
            if height(middle)?.signum() == low_value.signum() {
                low = middle;
            } else {
                high = middle;
            }
        }
        wall_trim_u[slot] = Some((0.5 * (low + high), u_end));
    }
    for (slot, (ring_end, end)) in ring_ends.iter().zip([&end0, &end1]).enumerate() {
        end_vertices[slot] = match (ring_end, &end.terminus) {
            (RingEnd::Chain(chain), _) => [chain.pole, chain.pole],
            (RingEnd::One(_), Terminus::WallTrim { end: station, .. }) => {
                let kept_is_first = end_kept_first[slot];
                let kept_row = if kept_is_first { &rows.cr } else { &rows.cs };
                let (u_k, _) = wall_trim_u[slot].expect("solved above");
                let k_point = kept_row.evaluate(u_k).or_refuse(KernelStage::Refine, "evaluate")?;
                let pad_point = if kept_is_first { station.p2 } else { station.p1 };
                let (k_id, p_id) = (take_id(), take_id());
                result.vertices.push(VertexRecord { id: k_id, point: k_point });
                result.vertices.push(VertexRecord { id: p_id, point: pad_point });
                if kept_is_first { [k_id, p_id] } else { [p_id, k_id] }
            }
            (RingEnd::One(_), Terminus::Cliff { stop, .. }) | (RingEnd::Open(_), Terminus::Open { station: stop }) => {
                let mut fresh = [0u64; 2];
                for (side, point) in [stop.p1, stop.p2].into_iter().enumerate() {
                    let id = take_id();
                    result.vertices.push(VertexRecord { id, point });
                    fresh[side] = id;
                }
                fresh
            }
            _ => {
                return Err(KernelRefusal::internal(KernelStage::Refine, "runout_terminus", "blend runout: an end's terminus does not match its ring end"));
            }
        };
    }
    // The switch vertices: the contact at each interior piece boundary.
    let mut switch_off_rail = Vec::new();
    let mut commit_pieces = |pieces: &[Piece], side_first: bool, result: &mut BrepSolid, take_id: &mut dyn FnMut() -> u64| -> Result<Vec<(u64, u64, u64, NurbsCurve, u64)>, KernelRefusal> {
        // (edge id, start vertex, end vertex, mate pcurve in station order, carrier)
        let row = if side_first { &rows.cr } else { &rows.cs };
        let side = usize::from(!side_first);
        let mut out = Vec::new();
        let mut vertices: Vec<u64> = vec![end_vertices[0][side]];
        for pair in pieces.windows(2) {
            let index = pair[0].range[1];
            let point = if side_first { stations[index].p1 } else { stations[index].p2 };
            let id = take_id();
            result.vertices.push(VertexRecord { id, point });
            vertices.push(id);
        }
        vertices.push(end_vertices[1][side]);
        for (k, piece) in pieces.iter().enumerate() {
            let params = &parameters[piece.range[0]..=piece.range[1]];
            let samples: Vec<Vec4> = piece.uvs.iter().map(|uv| Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0)).collect();
            if samples.len() != params.len() {
                return Err(KernelRefusal::internal(KernelStage::Refine, "runout_piece", "blend runout: a rail piece's uv samples do not match its stations"));
            }
            // The row breaks that fall inside this piece, as local indices.
            let local_breaks: Vec<usize> = breaks
                .iter()
                .filter(|b| **b > piece.range[0] && **b < piece.range[1])
                .map(|b| b - piece.range[0])
                .collect();
            let pcurve = interpolate_piece(&samples, params, &local_breaks)?;
            if debug {
                let carrier = face_by_id(ring, piece.carrier)?;
                let (mut fit_worst, mut data_worst) = (0.0f64, 0.0f64);
                for (j, (uv, u)) in piece.uvs.iter().zip(params).enumerate() {
                    let fitted = pcurve.evaluate(*u).or_refuse(KernelStage::Refine, "evaluate")?;
                    fit_worst = fit_worst.max(((fitted.x - uv[0]).powi(2) + (fitted.y - uv[1]).powi(2)).sqrt());
                    let on_face = carrier.surface.evaluate(uv[0], uv[1]).or_refuse(KernelStage::Refine, "evaluate")?;
                    let station = &stations[piece.range[0] + j];
                    let contact = if side_first { station.p1 } else { station.p2 };
                    data_worst = data_worst.max(on_face.sub(contact).length());
                }
                eprintln!(
                    "runout spine {spine_id}: {} piece {k} on face {} ({} samples, u [{:.4}, {:.4}], breaks {local_breaks:?}): pcurve off its samples {fit_worst:.3e}, samples off the contacts {data_worst:.3e}",
                    if side_first { "cr" } else { "cs" }, piece.carrier, piece.uvs.len(), params[0], params[params.len() - 1]
                );
            }
            let id = take_id();
            result.edges.push(EdgeRecord {
                id,
                curve: row.clone(),
                t0: params[0],
                t1: params[params.len() - 1],
                start_vertex_id: vertices[k],
                end_vertex_id: vertices[k + 1],
                degenerate: false,
                name: None,
            });
            out.push((id, vertices[k], vertices[k + 1], pcurve, piece.carrier));
        }
        Ok(out)
    };
    let mut rails_cr = commit_pieces(&pieces_cr, true, result, take_id)?;
    let mut rails_cs = commit_pieces(&pieces_cs, false, result, take_id)?;
    // Row 2: a wall-trim end's kept rail stops at u_K (on the floor's plane):
    // its edge range and its mate pcurve end there.
    for slot in 0..2 {
        let Some((u_k, _)) = wall_trim_u[slot] else { continue };
        let kept_side = if end_kept_first[slot] { &mut rails_cr } else { &mut rails_cs };
        let index = if slot == 0 { 0 } else { kept_side.len() - 1 };
        let rail = &mut kept_side[index];
        let (low, high) = rail.3.split(u_k).or_refuse(KernelStage::Refine, "split")?;
        rail.3 = if slot == 0 { high } else { low };
        let edge = result.edges.iter_mut().find(|edge| edge.id == rail.0).ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "runout_wall_trim", "blend runout: the kept rail is missing"))?;
        if slot == 0 { edge.t0 = u_k; } else { edge.t1 = u_k; }
    }
    // Each end's rail pieces by side: an end's kept side runs on its kept
    // mate (and so on its pad_r), its replaced side on its pad (row 2: the
    // sides are per end).
    let side_rails = |first_side: bool| if first_side { &rails_cr } else { &rails_cs };
    let end_rail = |first_side: bool, slot: usize| {
        let rails = side_rails(first_side);
        if slot == 0 { rails[0].clone() } else { rails[rails.len() - 1].clone() }
    };
    // Trim the pad rails at the switch vertices: a chain's roof rail at B
    // and side rail at A; a one-ended end's side rail at A; an open end's
    // two unselected edges at the section's contacts (first-mate side, then
    // second-mate side).
    for (slot, ring_end) in ring_ends.iter().enumerate() {
        let kept_is_first = end_kept_first[slot];
        let (kept_end, replaced_end) = (end_rail(kept_is_first, slot), end_rail(!kept_is_first, slot));
        let (b_vertex, a_vertex) = if slot == 0 { (kept_end.2, replaced_end.2) } else { (kept_end.1, replaced_end.1) };
        let trims: Vec<(u64, u64, u64, usize)> = match ring_end {
            RingEnd::Chain(chain) => vec![
                (chain.roof_rail, chain.mid_vertex, b_vertex, 0usize),
                (chain.side_rail, chain.spine_vertex, a_vertex, 1usize),
            ],
            RingEnd::One(chain) => {
                let mut trims = vec![(chain.side_rail, chain.spine_vertex, a_vertex, 1usize)];
                // Row 2: the pad's floor rail at the terminus's pad contact
                // and the unselected edge at K.
                if let (Some(floor_rail), Some(_)) = (chain.floor_rail, wall_trim_u[slot]) {
                    let side_of = |kept_side: bool| if kept_side == kept_is_first { 0 } else { 1 };
                    trims.push((floor_rail, chain.mid_vertex, end_vertices[slot][side_of(false)], 2usize));
                    trims.push((chain.e2, chain.mid_vertex, end_vertices[slot][side_of(true)], 2usize));
                }
                trims
            }
            RingEnd::Open(end) => vec![
                (end.edge_first, end.spine_vertex, end_vertices[slot][0], 2usize),
                (end.edge_second, end.spine_vertex, end_vertices[slot][1], 2usize),
            ],
        };
        for (rail_id, old_vertex, new_vertex, which) in trims {
            let rail = edge_by_id(result, rail_id)?;
            let point = vertex_point(result, new_vertex)?;
            let projection = project_point_to_curve(&rail.curve, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
            switch_off_rail.push(projection.distance);
            if debug {
                let fraction = (projection.u - rail.t0) / (rail.t1 - rail.t0);
                eprintln!(
                    "runout spine {spine_id} end {slot}: {} rail {rail_id} trimmed at fraction {fraction:.6} (vertex {new_vertex} {:.3e} off the rail)",
                    ["roof", "side", "open-end"][which], projection.distance
                );
            }
            trim_edge_at(result, rail_id, projection.u, old_vertex, new_vertex)?;
        }
    }
    for (k, value) in switch_off_rail.iter().enumerate().take(4) {
        report.switch_off_rail[k] = *value;
    }

    // ---- The one-ended ends' cliffs: at each, the pad's end arc is cut
    // where the stop's section plane crosses it (q), and three edges close
    // the stripe — the stop's section arc between the two rail ends, the
    // pad's generator from its contact to q, and the segment on the kept mate
    // from q to the kept rail's end — bounding a planar face in that
    // section's plane. (edge ids: arc, generator, segment; q; the face)
    struct CliffEdges {
        arc: u64,
        generator: u64,
        segment: u64,
        q: u64,
        plane: NurbsSurface,
    }
    let mut cliffs: Vec<Option<CliffEdges>> = vec![None, None];
    let mut cliff_segments: Vec<(usize, u64, NurbsCurve)> = Vec::new();
    let mut cliff_planes: Vec<(usize, NurbsCurve, NurbsCurve, NurbsCurve)> = Vec::new();
    let pcurve_band = crate::KernelTolerances::for_solid(ring, 1e-7).pcurve_consistency;
    for (slot, (ring_end, end)) in ring_ends.iter().zip([&end0, &end1]).enumerate() {
        let (RingEnd::One(chain), Terminus::Cliff { stop, .. }) = (ring_end, &end.terminus) else { continue };
        let (kept, kept_is_first) = (end_kept[slot], end_kept_first[slot]);
        let centre = stop.center;
        let x_axis = stop.p1.sub(centre).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let to_p2 = stop.p2.sub(centre);
        let plane_normal = x_axis.cross(to_p2).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let y_axis = plane_normal.cross(x_axis).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let angle = to_p2.dot(y_axis).atan2(to_p2.dot(x_axis));
        // q: the pad's end arc crosses the stop's section plane once.
        let e1 = edge_by_id(result, chain.e1)?;
        let plane_height = |t: f64| -> Result<f64, KernelRefusal> {
            Ok(e1.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?.sub(centre).dot(plane_normal))
        };
        let mut low = e1.t0;
        let mut low_value = plane_height(low)?;
        let mut high = None;
        for sample in 1..=BRACKET_SAMPLES {
            let t = e1.t0 + (e1.t1 - e1.t0) * sample as f64 / BRACKET_SAMPLES as f64;
            let value = plane_height(t)?;
            if value.signum() != low_value.signum() {
                high = Some(t);
                break;
            }
            low = t;
            low_value = value;
        }
        let mut high = high.ok_or_else(|| KernelRefusal::unsupported(KernelStage::Refine, "one_ended_cliff_q", format!(
            "blend one-ended runout: the pad's end arc {} does not cross the stop's section plane on spine {spine_id}",
            chain.e1
        )))?;
        for _ in 0..BISECTIONS {
            let middle = 0.5 * (low + high);
            let value = plane_height(middle)?;
            if value.signum() == low_value.signum() {
                low = middle;
                low_value = value;
            } else {
                high = middle;
            }
        }
        let t_q = 0.5 * (low + high);
        let q_point = e1.curve.evaluate(t_q).or_refuse(KernelStage::Refine, "evaluate")?;
        let (e1_t0, e1_t1) = (e1.t0, e1.t1);
        let q = take_id();
        result.vertices.push(VertexRecord { id: q, point: q_point });
        if debug {
            eprintln!(
                "runout spine {spine_id} end {slot} (one-ended): q on the pad's end arc {} at fraction {:.6}: ({:.6}, {:.6}, {:.6}); section angle {:.6}",
                chain.e1, (t_q - e1_t0) / (e1_t1 - e1_t0), q_point.x, q_point.y, q_point.z, angle
            );
        }
        trim_edge_at(result, chain.e1, t_q, chain.spine_vertex, q)?;
        // The stop's contacts by role.
        let (kept_point, pad_point) = if kept_is_first { (stop.p1, stop.p2) } else { (stop.p2, stop.p1) };
        let (kept_vertex, pad_vertex) = if kept_is_first { (end_vertices[slot][0], end_vertices[slot][1]) } else { (end_vertices[slot][1], end_vertices[slot][0]) };
        let pad = face_by_id(ring, chain.pad)?;
        let kept_face = face_by_id(ring, kept)?;
        // The pad's generator q -> pad contact must lie on the pad (a ruling
        // of a straight concave edge's pad); a curved pad is named.
        let generator_mid = q_point.add(pad_point).scale(0.5);
        let off_pad = surface_gap(&pad.surface, generator_mid)?.0;
        if off_pad > pcurve_band {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "one_ended_pad_generator", format!(
                "blend one-ended runout: the pad {}'s section in the stop's plane is not its straight generator ({off_pad:.3e} off at the middle, band {pcurve_band:.1e}); a curved concave edge's pad is not cut here",
                chain.pad
            )));
        }
        let uv_of = |surface: &NurbsSurface, point: Vec3| -> Result<[f64; 2], KernelRefusal> {
            let projection = project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            Ok([projection.u, projection.v])
        };
        let line_pcurve = |surface: &NurbsSurface, from: Vec3, to: Vec3| -> Result<NurbsCurve, KernelRefusal> {
            let a = uv_of(surface, from)?;
            let b = uv_of(surface, to)?;
            crate::sweep_topology::parameter_line(a[0], a[1], b[0], b[1]).or_refuse(KernelStage::Refine, "parameter_line")
        };
        // The three edges.  The arc runs from the first-mate rail's end to
        // the second's (v = 0 to v = 1 on the wall), exactly on the ball.
        let arc_curve = crate::make_arc(centre, x_axis, y_axis, radius, 0.0, angle).or_refuse(KernelStage::Refine, "make_arc")?;
        let [arc_t0, arc_t1] = arc_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let arc = take_id();
        result.edges.push(EdgeRecord { id: arc, curve: arc_curve.clone(), t0: arc_t0, t1: arc_t1, start_vertex_id: end_vertices[slot][0], end_vertex_id: end_vertices[slot][1], degenerate: false, name: None });
        let generator_curve = crate::make_line(q_point, pad_point).or_refuse(KernelStage::Refine, "make_line")?;
        let generator = take_id();
        result.edges.push(EdgeRecord { id: generator, curve: generator_curve.clone(), t0: 0.0, t1: 1.0, start_vertex_id: q, end_vertex_id: pad_vertex, degenerate: false, name: None });
        let segment_curve = crate::make_line(kept_point, q_point).or_refuse(KernelStage::Refine, "make_line")?;
        let segment = take_id();
        result.edges.push(EdgeRecord { id: segment, curve: segment_curve.clone(), t0: 0.0, t1: 1.0, start_vertex_id: kept_vertex, end_vertex_id: q, degenerate: false, name: None });
        // The cliff's plane, large enough to hold the section with margin so
        // the projector never clamps.
        let extent = 6.0 * radius;
        let plane_origin = centre.sub(x_axis.scale(0.5 * extent)).sub(y_axis.scale(0.5 * extent));
        let plane = crate::make_plane(plane_origin, x_axis, y_axis, extent, extent).or_refuse(KernelStage::Refine, "make_plane")?;
        // Pcurves on the pad and the kept mate (lines in their uv), and on
        // the plane (the arc's control points mapped through the plane's
        // affine inverse, exact for a rational arc).
        let generator_on_pad = line_pcurve(&pad.surface, q_point, pad_point)?;
        let segment_on_kept = line_pcurve(&kept_face.surface, kept_point, q_point)?;
        {
            let mid = generator_on_pad.evaluate(0.5).or_refuse(KernelStage::Refine, "evaluate")?;
            let on_pad = pad.surface.evaluate(mid.x, mid.y).or_refuse(KernelStage::Refine, "evaluate")?;
            let miss = on_pad.sub(generator_mid).length();
            if miss > pcurve_band {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "one_ended_pad_generator", format!(
                    "blend one-ended runout: the pad {}'s uv line misses its generator by {miss:.3e} at the middle (band {pcurve_band:.1e})",
                    chain.pad
                )));
            }
        }
        let arc_on_plane = {
            let mut points = Vec::with_capacity(arc_curve.control_points.len());
            for cp in &arc_curve.control_points {
                let w = cp.w;
                let uv = uv_of(&plane, Vec3::new(cp.x / w, cp.y / w, cp.z / w))?;
                points.push(Vec4 { x: uv[0] * w, y: uv[1] * w, z: 0.0, w });
            }
            NurbsCurve::new(arc_curve.degree, arc_curve.knots.clone(), points).or_refuse(KernelStage::Refine, "new")?
        };
        let generator_on_plane = line_pcurve(&plane, q_point, pad_point)?;
        let segment_on_plane = line_pcurve(&plane, kept_point, q_point)?;
        // The pad gains the generator and its rail piece between its end arc
        // (now ending at q) and its side rail (now starting at A); the kept
        // mate gains the segment with its rail pieces (spliced below with
        // the ridge run).  The cliff face: its arc coedge opposite to the
        // wall's, the other two chained by their vertices.
        let pad_piece = end_rail(!kept_is_first, slot);
        splice_loop(result, chain.pad, &[], Some((chain.e1, chain.side_rail)), &[(generator, generator_on_pad), (pad_piece.0, pad_piece.3.clone())], take_id)?;
        cliffs[slot] = Some(CliffEdges { arc, generator, segment, q, plane });
        // Kept-mate pcurve of the segment, and the plane pcurves, are
        // carried into the splice and the face below.
        cliff_segments.push((slot, segment, segment_on_kept));
        cliff_planes.push((slot, arc_on_plane, generator_on_plane, segment_on_plane));
    }

    // ---- The open ends: the section arc at the spine's vertex, exactly on
    //      the ball, from the first-mate rail's end to the second's (v = 0
    //      to v = 1 on the wall); the end face's loop gains it between the
    //      two trimmed edges (an anchored empty run).  Its pcurve on the end
    //      face maps the rational arc's control points through the plane's
    //      affine inverse, exact for a rational arc. ----
    let mut open_arcs: [Option<u64>; 2] = [None, None];
    for (slot, (ring_end, end)) in ring_ends.iter().zip([&end0, &end1]).enumerate() {
        let (RingEnd::Open(open), Terminus::Open { station }) = (ring_end, &end.terminus) else { continue };
        let centre = station.center;
        let x_axis = station.p1.sub(centre).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let to_p2 = station.p2.sub(centre);
        let plane_normal = x_axis.cross(to_p2).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let y_axis = plane_normal.cross(x_axis).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let angle = to_p2.dot(y_axis).atan2(to_p2.dot(x_axis));
        let arc_curve = crate::make_arc(centre, x_axis, y_axis, radius, 0.0, angle).or_refuse(KernelStage::Refine, "make_arc")?;
        let [arc_t0, arc_t1] = arc_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let arc = take_id();
        result.edges.push(EdgeRecord { id: arc, curve: arc_curve.clone(), t0: arc_t0, t1: arc_t1, start_vertex_id: end_vertices[slot][0], end_vertex_id: end_vertices[slot][1], degenerate: false, name: None });
        let end_face = face_by_id(ring, open.end_face)?;
        let arc_on_face = {
            let mut points = Vec::with_capacity(arc_curve.control_points.len());
            for cp in &arc_curve.control_points {
                let w = cp.w;
                let projection = project_point_to_surface(&end_face.surface, Vec3::new(cp.x / w, cp.y / w, cp.z / w)).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
                points.push(Vec4 { x: projection.u * w, y: projection.v * w, z: 0.0, w });
            }
            NurbsCurve::new(arc_curve.degree, arc_curve.knots.clone(), points).or_refuse(KernelStage::Refine, "new")?
        };
        {
            let mid_t = 0.5 * (arc_t0 + arc_t1);
            let uv = arc_on_face.evaluate(mid_t).or_refuse(KernelStage::Refine, "evaluate")?;
            let on_face = end_face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?;
            let on_arc = arc_curve.evaluate(mid_t).or_refuse(KernelStage::Refine, "evaluate")?;
            let miss = on_face.sub(on_arc).length();
            if miss > pcurve_band {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "open_end_oblique", format!(
                    "blend one-ended runout: the open end's section arc on spine {spine_id} misses the end face {} by {miss:.3e} at its middle (band {pcurve_band:.1e})",
                    open.end_face
                )));
            }
        }
        if debug {
            eprintln!(
                "runout spine {spine_id} end {slot} (open): section arc {arc} of angle {angle:.6} on end face {} between vertices {} and {}; edges {} and {} trimmed to them",
                open.end_face, end_vertices[slot][0], end_vertices[slot][1], open.edge_first, open.edge_second
            );
        }
        splice_loop(result, open.end_face, &[], Some((open.edge_first, open.edge_second)), &[(arc, arc_on_face)], take_id)?;
        open_arcs[slot] = Some(arc);
    }

    // ---- Row 2: the wall-trim ends.  The wall ∩ the floor's plane from K
    //      (the kept rail's end, v = the kept side) to the terminus's pad
    //      contact (u_end, v = the pad side), traced on the FITTED wall: for
    //      each u the v where the wall's section crosses the plane.  One knot
    //      vector for the edge and both pcurves; the floor pcurve is the
    //      plane's affine image of the edge's samples.  It goes into the
    //      floor's loop between the trimmed unselected edge and the trimmed
    //      floor rail, walked first-mate side to second-mate side as the
    //      section arcs are. ----
    let mut wall_trims: [Option<(u64, NurbsCurve)>; 2] = [None, None];
    for (slot, ring_end) in ring_ends.iter().enumerate() {
        let (RingEnd::One(chain), Some((u_k, u_end))) = (ring_end, wall_trim_u[slot]) else { continue };
        let Some(floor_rail) = chain.floor_rail else { continue };
        let kept_is_first = end_kept_first[slot];
        let floor = face_by_id(ring, chain.floor)?;
        let Some(crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = floor.surface.analytic() else {
            return Err(KernelRefusal::internal(KernelStage::Refine, "runout_wall_trim", "blend runout: a wall-trim floor is not planar"));
        };
        let normal = u_dir.cross(*v_dir).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let (v_kept, v_pad) = if kept_is_first { (0.0, 1.0) } else { (1.0, 0.0) };
        let wall_height = |u: f64, v: f64| -> Result<f64, KernelRefusal> {
            Ok(rows.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?.sub(*origin).dot(normal))
        };
        let crossing_v = |u: f64| -> Result<f64, KernelRefusal> {
            let (mut low, mut high) = (v_kept, v_pad);
            let low_value = wall_height(u, low)?;
            let high_value = wall_height(u, high)?;
            if low_value.signum() == high_value.signum() {
                // At the ends the crossing IS an end of the section.
                return Ok(if low_value.abs() <= high_value.abs() { v_kept } else { v_pad });
            }
            for _ in 0..BISECTIONS {
                let middle = 0.5 * (low + high);
                if wall_height(u, middle)?.signum() == low_value.signum() {
                    low = middle;
                } else {
                    high = middle;
                }
            }
            Ok(0.5 * (low + high))
        };
        let floor_uv = |point: Vec3| -> Result<Vec3, KernelRefusal> {
            let projection = project_point_to_surface(&floor.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            Ok(Vec3::new(projection.u, projection.v, 0.0))
        };
        let pcurve_band = crate::KernelTolerances::for_solid(ring, 1e-7).intersection_fit;
        let mut samples = 16usize;
        let (curve, blend_pcurve, floor_pcurve) = loop {
            // From K (u_k, v_kept) to the pad contact (u_end, v_pad), then
            // ordered first-mate side (v = 0) to second (v = 1).
            let mut uv: Vec<Vec3> = Vec::with_capacity(samples + 1);
            for k in 0..=samples {
                let fraction = k as f64 / samples as f64;
                let u = u_k + (u_end - u_k) * fraction;
                let v = if k == 0 { v_kept } else if k == samples { v_pad } else { crossing_v(u)? };
                uv.push(Vec3::new(u, v, 0.0));
            }
            if v_kept > v_pad {
                uv.reverse();
            }
            let points: Vec<Vec3> = uv.iter().map(|p| rows.surface.evaluate(p.x, p.y)).collect::<Result<_, _>>().or_refuse(KernelStage::Refine, "evaluate")?;
            let parameters: Vec<f64> = (0..=samples).map(|k| k as f64 / samples as f64).collect();
            let to4 = |values: &[Vec3]| -> Vec<Vec4> { values.iter().map(|p| Vec4::from_point(*p, 1.0)).collect() };
            let floor_samples: Vec<Vec3> = points.iter().map(|p| floor_uv(*p)).collect::<Result<_, _>>()?;
            let curve = crate::fit::interpolate_homogeneous(&to4(&points), 3, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
            let blend_pcurve = crate::fit::interpolate_homogeneous(&to4(&uv), 3, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
            let floor_pcurve = crate::fit::interpolate_homogeneous(&to4(&floor_samples), 3, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
            // Read between the samples: the edge on the plane and on the
            // wall, through its wall pcurve.
            let mut worst: f64 = 0.0;
            for k in 0..samples {
                let z = (k as f64 + 0.5) / samples as f64;
                let point = curve.evaluate(z).or_refuse(KernelStage::Refine, "evaluate")?;
                let on_wall = blend_pcurve.evaluate(z).or_refuse(KernelStage::Refine, "evaluate")?;
                let wall_point = rows.surface.evaluate(on_wall.x, on_wall.y).or_refuse(KernelStage::Refine, "evaluate")?;
                worst = worst.max(point.sub(*origin).dot(normal).abs()).max(point.sub(wall_point).length());
            }
            if debug {
                eprintln!("runout spine {spine_id} end {slot}: wall trim on floor {} at {samples} samples, u [{u_k:.6}, {u_end:.6}]: {worst:.3e} off the plane / its wall pcurve (bar {pcurve_band:.1e})", chain.floor);
            }
            if worst <= pcurve_band {
                break (curve, blend_pcurve, floor_pcurve);
            }
            if samples >= 256 {
                return Err(KernelRefusal::non_convergence(KernelStage::Refine, ONE_ENDED_CONCAVE_THIRD_WHAT, format!(
                    "blend one-ended runout (concave third): the wall trim on floor {} stands {worst:.3e} off its surfaces at {samples} samples (bar {pcurve_band:.1e})", chain.floor
                )));
            }
            samples *= 2;
        };
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let trim = take_id();
        result.edges.push(EdgeRecord { id: trim, curve, t0, t1, start_vertex_id: end_vertices[slot][0], end_vertex_id: end_vertices[slot][1], degenerate: false, name: None });
        splice_loop(result, chain.floor, &[], Some((chain.e2, floor_rail)), &[(trim, floor_pcurve)], take_id)?;
        wall_trims[slot] = Some((trim, blend_pcurve));
    }

    // The mate loops: each loses its run of ridge coedges and gains the rail
    // piece(s) that replace it.  (face, removed edges, inserted rail pieces)
    // Each mate face gets the centre piece of its own side: the piece after
    // end 0's pad pieces on that side (row 2: per end, by the side's role
    // there).
    let centre_index = |first_side: bool| -> usize {
        let kept_here = end_kept_first[0] == first_side;
        match &ring_ends[0] {
            RingEnd::Chain(_) => 1,
            RingEnd::Open(_) => 0,
            RingEnd::One(_) => usize::from(!kept_here),
        }
    };
    let centre_piece = |first_side: bool| side_rails(first_side)[centre_index(first_side)].clone();
    // The ridge edges each mate loses with the spine: a chain end's ridge
    // edge, and (row 2) a wall-trim end's pad end arc, both on that END's
    // kept mate.
    let ridge_e1 = |slot: usize| -> Option<u64> {
        match &ring_ends[slot] {
            RingEnd::Chain(chain) => Some(chain.e1),
            RingEnd::One(chain) if wall_trim_u[slot].is_some() => Some(chain.e1),
            _ => None,
        }
    };
    let mate_run = |mate: u64| -> Vec<u64> {
        let mut run: Vec<u64> = Vec::new();
        if end_kept[0] == mate {
            run.extend(ridge_e1(0));
        }
        run.push(spine_id);
        if end_kept[1] == mate {
            run.extend(ridge_e1(1));
        }
        run
    };
    // `kept` (end 0's, or the only non-open end's) first, as before.
    let mut splices: Vec<(u64, Vec<u64>, Vec<(u64, NurbsCurve)>)> = Vec::new();
    for mate in [kept, if kept == first.face.id { second.face.id } else { first.face.id }] {
        let first_side = mate == first.face.id;
        let centre = centre_piece(first_side);
        let mut inserted: Vec<(u64, NurbsCurve)> = vec![(centre.0, centre.3.clone())];
        for (slot, segment, segment_on_kept) in &cliff_segments {
            if end_kept[*slot] == mate {
                inserted.push((*segment, segment_on_kept.clone()));
            }
        }
        splices.push((mate, mate_run(mate), inserted));
    }
    for (slot, ring_end) in ring_ends.iter().enumerate() {
        let RingEnd::Chain(chain) = ring_end else { continue };
        let kept_is_first = end_kept_first[slot];
        let (pad_s_piece, pad_r_piece) = (end_rail(!kept_is_first, slot), end_rail(kept_is_first, slot));
        splices.push((chain.pad_s, vec![chain.e1, chain.e2], vec![(pad_s_piece.0, pad_s_piece.3.clone())]));
        splices.push((chain.pad_r, vec![chain.e2], vec![(pad_r_piece.0, pad_r_piece.3.clone())]));
    }
    // Row 2: a wall-trim end's pad loses its end arc for its rail piece.
    for (slot, ring_end) in ring_ends.iter().enumerate() {
        let RingEnd::One(chain) = ring_end else { continue };
        if wall_trim_u[slot].is_none() {
            continue;
        }
        let pad_piece = end_rail(!end_kept_first[slot], slot);
        splices.push((chain.pad, vec![chain.e1], vec![(pad_piece.0, pad_piece.3.clone())]));
    }
    // The wall's use of each rail edge: cr pieces walked forward exactly
    // when the blend loop's walk is station order (`build_open_surgery`).
    let blend_cr_forward = !first.coedge.forward;
    let is_rail = |edge_id: u64| rails_cr.iter().chain(rails_cs.iter()).any(|rail| rail.0 == edge_id);
    let wall_forward_of = |edge_id: u64| -> bool {
        if rails_cr.iter().any(|rail| rail.0 == edge_id) { blend_cr_forward } else { !blend_cr_forward }
    };
    for (face_id, removed, inserted) in splices {
        let senses = replace_run(result, face_id, &removed, &inserted, take_id)?;
        for (edge_id, mate_forward) in senses {
            if is_rail(edge_id) && mate_forward == wall_forward_of(edge_id) {
                return Err(KernelRefusal::internal(KernelStage::Sew, "runout_pairing", format!(
                    "blend runout: face {face_id}'s loop walks rail {edge_id} in the same sense as the wall would (manifold pairing broken)"
                )));
            }
        }
    }
    if debug {
        // Every new rail's pcurve on each mate against the rail itself.
        for rail in rails_cr.iter().chain(rails_cs.iter()) {
            let edge = edge_by_id(result, rail.0)?;
            for face in result.shells.iter().flat_map(|shell| &shell.faces) {
                for lp in &face.loops {
                    for coedge in lp.coedges.iter().filter(|c| c.edge_id == rail.0) {
                        let mut worst: f64 = 0.0;
                        for k in 0..=16 {
                            let t = edge.t0 + (edge.t1 - edge.t0) * k as f64 / 16.0;
                            let on_edge = edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
                            // A reversed pcurve is mirrored over its own domain.
                            let s = if coedge.forward { t } else { edge.t0 + edge.t1 - t };
                            let uv = coedge.pcurve.evaluate(s).or_refuse(KernelStage::Refine, "evaluate")?;
                            let on_face = face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?;
                            worst = worst.max(on_face.sub(on_edge).length());
                        }
                        let [d0, d1] = coedge.pcurve.domain().unwrap_or([f64::NAN; 2]);
                        eprintln!(
                            "runout spine {spine_id}: rail {} on face {} (forward {}) edge t [{:.4}, {:.4}] pcurve domain [{d0:.4}, {d1:.4}] deviation {worst:.3e}",
                            rail.0, face.id, coedge.forward, edge.t0, edge.t1
                        );
                    }
                }
            }
        }
    }
    // Delete the ridge edges: the spine, and a chain end's two chain edges
    // (a one-ended end's arc was trimmed, its unselected edge is untouched).
    let mut ridge: Vec<u64> = vec![spine_id];
    for (slot, ring_end) in ring_ends.iter().enumerate() {
        if let RingEnd::Chain(chain) = ring_end {
            ridge.push(chain.e1);
            ridge.push(chain.e2);
        }
        if let RingEnd::One(chain) = ring_end {
            if wall_trim_u[slot].is_some() {
                ridge.push(chain.e1);
            }
        }
    }
    result.edges.retain(|edge| !ridge.contains(&edge.id));

    // ---- The wall face. ----
    let u_pole = [parameters[0], parameters[parameters.len() - 1]];
    // The wall's closing edge at each end: a chain's degenerate pole edge, a
    // one-ended end's section arc (first-mate side to second-mate side).
    let mut pole_edges = [0u64; 2];
    for slot in 0..2 {
        match &ring_ends[slot] {
            RingEnd::Chain(chain) => {
                let point = chain.pole_point;
                let id = take_id();
                result.edges.push(EdgeRecord {
                    id,
                    curve: crate::make_line(point, point).or_refuse(KernelStage::Refine, "make_line")?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: chain.pole,
                    end_vertex_id: chain.pole,
                    degenerate: true,
                    name: None,
                });
                pole_edges[slot] = id;
            }
            RingEnd::One(_) if wall_trims[slot].is_some() => {
                pole_edges[slot] = wall_trims[slot].as_ref().map(|(id, _)| *id).expect("checked");
            }
            RingEnd::One(_) => {
                pole_edges[slot] = cliffs[slot].as_ref().ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "runout_cliff", "blend runout: a one-ended end has no cliff"))?.arc;
            }
            RingEnd::Open(_) => {
                pole_edges[slot] = open_arcs[slot].ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "runout_open_end", "blend runout: an open end has no section arc"))?;
            }
        }
    }
    let line = |u0: f64, v0: f64, u1: f64, v1: f64| crate::sweep_topology::parameter_line(u0, v0, u1, v1).or_refuse(KernelStage::Refine, "parameter_line");
    let rail_coedges = |rails: &[(u64, u64, u64, NurbsCurve, u64)], v: f64, forward: bool, take_id: &mut dyn FnMut() -> u64| -> Result<Vec<CoedgeRecord>, KernelRefusal> {
        let mut out = Vec::new();
        let ordered: Vec<&(u64, u64, u64, NurbsCurve, u64)> = if forward { rails.iter().collect() } else { rails.iter().rev().collect() };
        for rail in ordered {
            let edge = edge_by_id(result, rail.0)?;
            let (a, b) = (edge.t0, edge.t1);
            out.push(CoedgeRecord {
                id: take_id(),
                edge_id: rail.0,
                forward,
                pcurve: if forward { line(a, v, b, v)? } else { line(b, v, a, v)? },
            });
        }
        Ok(out)
    };
    // A pole edge is walked `forward` whichever way (both ends are the
    // pole); a section arc runs from v = 0 to v = 1, so a walk from v = 1 to
    // v = 0 is its reverse.
    let pole_coedge = |slot: usize, from_v: f64, to_v: f64, take_id: &mut dyn FnMut() -> u64| -> Result<CoedgeRecord, KernelRefusal> {
        let forward = matches!(ring_ends[slot], RingEnd::Chain(_)) || from_v < to_v;
        // Row 2: a wall trim's own pcurve on the wall (it runs v = 0 -> 1).
        if let Some((_, pcurve)) = &wall_trims[slot] {
            let pcurve = if forward { pcurve.clone() } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? };
            return Ok(CoedgeRecord { id: take_id(), edge_id: pole_edges[slot], forward, pcurve });
        }
        Ok(CoedgeRecord { id: take_id(), edge_id: pole_edges[slot], forward, pcurve: line(u_pole[slot], from_v, u_pole[slot], to_v)? })
    };
    let mut coedges = Vec::new();
    if blend_cr_forward {
        coedges.extend(rail_coedges(&rails_cr, 0.0, true, take_id)?);
        coedges.push(pole_coedge(1, 0.0, 1.0, take_id)?);
        coedges.extend(rail_coedges(&rails_cs, 1.0, false, take_id)?);
        coedges.push(pole_coedge(0, 1.0, 0.0, take_id)?);
    } else {
        coedges.extend(rail_coedges(&rails_cr, 0.0, false, take_id)?);
        coedges.push(pole_coedge(0, 0.0, 1.0, take_id)?);
        coedges.extend(rail_coedges(&rails_cs, 1.0, true, take_id)?);
        coedges.push(pole_coedge(1, 1.0, 0.0, take_id)?);
    }
    // Orientation: the wall's normal against the first mate's outward normal
    // at the centre stretch's middle station.
    let centre_range = ranges[n0];
    let mid_index = (centre_range[0] + centre_range[1]) / 2;
    let mid_u = parameters[mid_index];
    let blend_normal = raw_normal(&rows.surface, mid_u, 0.0)?;
    let uv1 = stations[mid_index].uv1;
    let n1 = raw_normal(&first.face.surface, uv1[0], uv1[1])?;
    let out1 = if first.face.same_sense { n1 } else { n1.scale(-1.0) };
    let same_sense = blend_normal.dot(out1) >= 0.0;
    let face_id = take_id();
    let shell_index = result
        .shells
        .iter()
        .position(|shell| shell.faces.iter().any(|face| face.id == kept))
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_shell", "blend runout: the kept mate's shell is missing"))?;
    let wall_arc_forward: Vec<(usize, bool)> = (0..2)
        .filter(|slot| matches!(ring_ends[*slot], RingEnd::One(_) | RingEnd::Open(_)))
        .map(|slot| (slot, coedges.iter().find(|coedge| coedge.edge_id == pole_edges[slot]).map(|coedge| coedge.forward).unwrap_or(true)))
        .collect();
    result.shells[shell_index].faces.push(FaceRecord {
        id: face_id,
        surface: rows.surface.clone(),
        same_sense,
        loops: vec![LoopRecord { id: take_id(), coedges }],
        name: name.map(str::to_string),
    });

    // ---- The cliff faces: planar, three edges, the arc walked
    // opposite to the wall, the normal into the air (back toward
    // station A, where the removed material was). ----
    for (slot, arc_on_plane, generator_on_plane, segment_on_plane) in cliff_planes {
        let cliff = cliffs[slot].take().ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "runout_cliff", "blend runout: a cliff's edges are missing"))?;
        let arc_forward = !wall_arc_forward.iter().find(|(s, _)| *s == slot).map(|(_, forward)| *forward).unwrap_or(true);
        let endpoints = |result: &BrepSolid, edge_id: u64| -> Result<(u64, u64), KernelRefusal> {
            let edge = edge_by_id(result, edge_id)?;
            Ok((edge.start_vertex_id, edge.end_vertex_id))
        };
        let mut loop_coedges = vec![CoedgeRecord {
            id: take_id(),
            edge_id: cliff.arc,
            forward: arc_forward,
            pcurve: if arc_forward { arc_on_plane.clone() } else { arc_on_plane.reversed().or_refuse(KernelStage::Refine, "reversed")? },
        }];
        let (arc_start, arc_end) = endpoints(result, cliff.arc)?;
        let (loop_start, mut cursor) = if arc_forward { (arc_start, arc_end) } else { (arc_end, arc_start) };
        let mut pending: Vec<(u64, NurbsCurve)> = vec![(cliff.generator, generator_on_plane), (cliff.segment, segment_on_plane)];
        while !pending.is_empty() {
            let position = pending
                .iter()
                .position(|(edge_id, _)| endpoints(result, *edge_id).map(|(s, e)| s == cursor || e == cursor).unwrap_or(false))
                .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "runout_cliff_loop", format!("blend runout: the cliff's edges do not continue from vertex {cursor}")))?;
            let (edge_id, pcurve) = pending.remove(position);
            let (s, e) = endpoints(result, edge_id)?;
            let forward = s == cursor;
            cursor = if forward { e } else { s };
            loop_coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id,
                forward,
                pcurve: if forward { pcurve } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
            });
        }
        if cursor != loop_start {
            return Err(KernelRefusal::internal(KernelStage::Sew, "runout_cliff_loop", "blend runout: the cliff's loop does not close"));
        }
        // Normal into the air: toward station A's centre.  The plane's
        // normal is read at its middle.
        let end = if slot == 0 { &end0 } else { &end1 };
        let Terminus::Cliff { stop, .. } = &end.terminus else { unreachable!("a cliff end") };
        let a_ball = {
            let (section_point, section_tangent) = section_frame(spine, end.t_a)?;
            let (_, _, _, centre_a) = tangency_residual(
                &face_by_id(ring, end.middle.0)?.surface,
                &face_by_id(ring, end.middle.1)?.surface,
                end.middle.2,
                end.middle.3,
                section_point,
                section_tangent,
            )?;
            centre_a
        };
        let [u0, u1] = cliff.plane.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = cliff.plane.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let plane_normal = cliff.plane.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)).or_refuse(KernelStage::Refine, "normal")?;
        let same_sense = plane_normal.dot(a_ball.sub(stop.center)) > 0.0;
        let _ = cliff.q;
        result.shells[shell_index].faces.push(FaceRecord {
            id: take_id(),
            surface: cliff.plane,
            same_sense,
            loops: vec![LoopRecord { id: take_id(), coedges: loop_coedges }],
            name: name.map(|value| format!("{value}_cliff")),
        });
    }
    Ok(report)
}

/// In `face_id`'s loop that carries `removed[0]`, replace the contiguous run
/// of coedges on `removed` with coedges on `inserted` (edge id, pcurve in the
/// edge's own parameter direction), each oriented so the loop stays
/// connected.  Returns each inserted edge's sense in that loop.
fn replace_run(
    result: &mut BrepSolid,
    face_id: u64,
    removed: &[u64],
    inserted: &[(u64, NurbsCurve)],
    take_id: &mut dyn FnMut() -> u64,
) -> Result<Vec<(u64, bool)>, KernelRefusal> {
    splice_loop(result, face_id, removed, None, inserted, take_id)
}

/// [`replace_run`] with the run located either by `removed[0]` or, for an
/// EMPTY run, by `anchor`: two edges whose coedges are adjacent in the loop
/// (they met at a vertex a trim has since moved), between which the inserted
/// edges go.  The inserted edges are chained by their vertices from the
/// loop's cursor in whatever order they are given.
fn splice_loop(
    result: &mut BrepSolid,
    face_id: u64,
    removed: &[u64],
    anchor: Option<(u64, u64)>,
    inserted: &[(u64, NurbsCurve)],
    take_id: &mut dyn FnMut() -> u64,
) -> Result<Vec<(u64, bool)>, KernelRefusal> {
    let endpoints = |result: &BrepSolid, coedge: &CoedgeRecord| -> Result<(u64, u64), KernelRefusal> {
        let edge = edge_by_id(result, coedge.edge_id)?;
        Ok(if coedge.forward { (edge.start_vertex_id, edge.end_vertex_id) } else { (edge.end_vertex_id, edge.start_vertex_id) })
    };
    let new_edges: Vec<(u64, u64, u64)> = inserted
        .iter()
        .map(|(id, _)| edge_by_id(result, *id).map(|edge| (*id, edge.start_vertex_id, edge.end_vertex_id)))
        .collect::<Result<_, _>>()?;
    let face_index = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .position(|face| face.id == face_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "runout_face", format!("blend runout: face {face_id} missing")))?;
    let locator = match (removed.first(), anchor) {
        (Some(first), _) => *first,
        (None, Some((before, _))) => before,
        (None, None) => return Err(KernelRefusal::internal(KernelStage::Sew, "runout_run", "blend runout: a splice with nothing removed needs an anchor")),
    };
    let mut loop_index = None;
    {
        let face = result.shells.iter().flat_map(|shell| &shell.faces).nth(face_index).expect("face present");
        for (index, lp) in face.loops.iter().enumerate() {
            if lp.coedges.iter().any(|coedge| coedge.edge_id == locator) {
                loop_index = Some(index);
            }
        }
    }
    let loop_index = loop_index.ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "runout_loop", format!(
        "blend runout: face {face_id} has no loop on edge {locator}"
    )))?;
    // Read the loop, compute the splice, then write it back.
    let old: Vec<CoedgeRecord> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .nth(face_index)
        .expect("face present")
        .loops[loop_index]
        .coedges
        .clone();
    let count = old.len();
    let positions: Vec<usize> = old.iter().enumerate().filter(|(_, c)| removed.contains(&c.edge_id)).map(|(i, _)| i).collect();
    if positions.len() != removed.len() {
        return Err(KernelRefusal::internal(KernelStage::Sew, "runout_run", format!(
            "blend runout: face {face_id}'s loop carries {} of the {} ridge edges {removed:?}",
            positions.len(), removed.len()
        )));
    }
    // The run starts at the removed coedge whose predecessor is kept; an
    // empty run starts at the anchor's later coedge.
    let start = if positions.is_empty() {
        let (a, b) = anchor.expect("an empty run has an anchor");
        let (pa, pb) = (old.iter().position(|c| c.edge_id == a), old.iter().position(|c| c.edge_id == b));
        let (Some(pa), Some(pb)) = (pa, pb) else {
            return Err(KernelRefusal::internal(KernelStage::Sew, "runout_run", format!(
                "blend runout: face {face_id}'s loop does not carry both anchor edges {a} and {b}"
            )));
        };
        if (pa + 1) % count == pb {
            pb
        } else if (pb + 1) % count == pa {
            pa
        } else {
            return Err(KernelRefusal::internal(KernelStage::Sew, "runout_run", format!(
                "blend runout: the anchor edges {a} and {b} are not adjacent in face {face_id}'s loop"
            )));
        }
    } else {
        *positions
            .iter()
            .find(|&&p| !positions.contains(&((p + count - 1) % count)))
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "runout_run", format!("blend runout: face {face_id}'s loop is wholly the ridge")))?
    };
    for k in 0..positions.len() {
        if !positions.contains(&((start + k) % count)) {
            return Err(KernelRefusal::internal(KernelStage::Sew, "runout_run", format!(
                "blend runout: the ridge edges {removed:?} are not contiguous in face {face_id}'s loop"
            )));
        }
    }
    let before = old[(start + count - 1) % count].clone();
    let after = old[(start + positions.len()) % count].clone();
    let (_, mut cursor) = endpoints(result, &before)?;
    let (after_start, _) = endpoints(result, &after)?;
    let mut senses = Vec::new();
    let mut fresh = Vec::new();
    let mut pending: Vec<usize> = (0..inserted.len()).collect();
    while !pending.is_empty() {
        // The next inserted edge is the one touching the cursor (the given
        // order first, so a single rail reads exactly as before).
        let next = pending
            .iter()
            .position(|&index| new_edges[index].1 == cursor || new_edges[index].2 == cursor)
            .ok_or_else(|| {
                let names: Vec<u64> = pending.iter().map(|&index| new_edges[index].0).collect();
                KernelRefusal::internal(KernelStage::Sew, "runout_run", format!(
                    "blend runout: none of the rails {names:?} continues face {face_id}'s loop from vertex {cursor}"
                ))
            })?;
        let index = pending.remove(next);
        let (id, pcurve) = (&inserted[index].0, &inserted[index].1);
        let (_, s, e) = &new_edges[index];
        let forward = *s == cursor;
        cursor = if forward { *e } else { *s };
        fresh.push(CoedgeRecord {
            id: take_id(),
            edge_id: *id,
            forward,
            pcurve: if forward { pcurve.clone() } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? },
        });
        senses.push((*id, forward));
    }
    if cursor != after_start {
        return Err(KernelRefusal::internal(KernelStage::Sew, "runout_run", format!(
            "blend runout: the rails spliced into face {face_id}'s loop end at vertex {cursor}, not at {after_start} where the loop continues"
        )));
    }
    let mut rebuilt: Vec<CoedgeRecord> = Vec::with_capacity(count - positions.len() + fresh.len());
    let mut k = (start + positions.len()) % count;
    for _ in 0..count - positions.len() {
        rebuilt.push(old[k].clone());
        k = (k + 1) % count;
    }
    rebuilt.extend(fresh);
    let face = result
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .nth(face_index)
        .expect("face present");
    face.loops[loop_index].coedges = rebuilt;
    Ok(senses)
}
