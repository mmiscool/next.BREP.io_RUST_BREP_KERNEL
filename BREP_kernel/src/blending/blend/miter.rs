//! The two-edge miter (§6.9.6): two stripes meeting at a vertex whose third
//! edge stays sharp.
//!
//! # Why this is not a surface-surface intersection call
//!
//! The seam between the two blends starts at P, the corner ball's tangency
//! point with the SHARED face — and there the two blend surfaces are tangent
//! to each other (both carry the shared face's normal).  A normal-based SSI
//! refuses P as a seed and is ill-conditioned next to it, which is the
//! blend-blend tangency drift the boolean miter has to weld over.  And two
//! equal-radius canals whose spines cross intersect in TWO branches through
//! P; only the one running toward the sharp edge is the seam.
//!
//! So the seam is marched as a level set on the FIRST stripe's own domain.
//! A constant-radius blend is the radius-r canal of its centre path, so a
//! point of stripe 1 lies on stripe 2 exactly when its distance to stripe 2's
//! centre row is r.  Step the station parameter from P toward the vertex and
//! solve the section coordinate alone: P is exact by construction, the
//! pcurve on stripe 1 is the sample list itself, the branch is chosen by the
//! direction of the march, and the gradient is fine everywhere off P.
//!
//! The seam leaves the strips through one of the two OTHER rails.  When it
//! leaves both at once the exit lies on the sharp edge and the corner is
//! symmetric.  Otherwise the stripe it left first is done, and the other one
//! is closed by a CONNECTOR — its own section by the first stripe's other
//! mate face, from the exit point down to the sharp edge — which is a
//! transversal level set (distance to that face) and marches the same way.

use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use crate::topology::{EdgeRecord, FaceRecord};
use crate::Vec3;

use super::stations::*;

/// One stripe's part in a miter.
pub(super) struct MiterStripe<'a> {
    pub(super) rows: &'a FittedRows,
    /// True when the shared face is this stripe's FIRST mate (the cr row,
    /// z = 0 on the blend surface).
    pub(super) shared_first: bool,
    /// True when the miter vertex is at the `u_domain[0]` end of the rows.
    pub(super) at_start: bool,
    /// Row parameter of the corner ball's tangency point on the shared rail.
    pub(super) u_ball: f64,
    /// The carrier of this stripe's OTHER mate: the face the sibling's
    /// connector runs on when this stripe is the one the seam leaves first.
    pub(super) other_face: &'a FaceRecord,
}

impl MiterStripe<'_> {
    /// Section coordinate from `f` (0 on the shared rail, 1 on the other).
    fn z(&self, fraction: f64) -> f64 {
        if self.shared_first {
            fraction
        } else {
            1.0 - fraction
        }
    }

    fn fraction_of_z(&self, z: f64) -> f64 {
        if self.shared_first {
            z
        } else {
            1.0 - z
        }
    }

    /// The row parameter at the vertex's end of the marched rows.
    fn u_end(&self) -> f64 {
        if self.at_start {
            self.rows.u_domain[0]
        } else {
            self.rows.u_domain[1]
        }
    }

    /// The row parameter of this stripe's own end vertex — where the seam is
    /// headed from the ball's station.
    fn u_vertex(&self) -> f64 {
        match self.rows.vertex_stations {
            Some(stations) => stations[usize::from(!self.at_start)],
            None => self.u_end(),
        }
    }

    /// The end of the marched rows on the side the seam travels toward.
    ///
    /// A CONVEX corner seats its ball inside the material, so P sits on the
    /// shared rail INSIDE the edge's own span and the seam runs outward, past
    /// the vertex, to the overshot end — `u_end`.  A CONCAVE one seats it in
    /// the air PAST the vertex (the two rails of a boss's base cross one
    /// radius beyond the corner), so P is out in the overshoot and the seam
    /// runs the other way, back into the span.  Both are "toward the vertex";
    /// only the row end that lies beyond it differs.
    fn u_limit(&self) -> f64 {
        if self.u_vertex() >= self.u_ball {
            self.rows.u_domain[1]
        } else {
            self.rows.u_domain[0]
        }
    }

    fn point(&self, u: f64, fraction: f64) -> Result<Vec3, KernelRefusal> {
        self.rows.surface.evaluate_extended(u, self.z(fraction)).or_refuse(KernelStage::Refine, "evaluate_extended")
    }

    fn center(&self) -> Result<&crate::NurbsCurve, KernelRefusal> {
        self.rows
            .center
            .as_ref()
            .ok_or_else(|| "miter: stripe rows carry no centre path".to_string()).or_refuse(KernelStage::Refine, "ok_or_else")
    }
}

/// A marched curve with its images on the two surfaces it lies on.
pub(super) struct MarchedCurve {
    pub(super) points: Vec<Vec3>,
    /// (u, z) on the first surface, per sample.
    pub(super) uv_first: Vec<[f64; 2]>,
    /// (u, z) on the second surface, per sample.
    pub(super) uv_second: Vec<[f64; 2]>,
}

/// A marched curve's fit: the 3D curve and its images on the two surfaces,
/// in the order the [`MarchedCurve`] stores them, on one parameterisation.
pub(super) type MarchedFit = (crate::NurbsCurve, crate::NurbsCurve, crate::NurbsCurve);

/// How the seam closes the corner.
pub(super) enum MiterClosure {
    /// The seam leaves both strips at one point, on the sharp edge.
    Symmetric {
        /// From P to the exit; images on stripe 0 then stripe 1.
        seam: MarchedCurve,
        /// The seam's fit, measured against its carriers ([`fit_marched`]).
        seam_fit: MarchedFit,
        /// Row parameter on each stripe where its OTHER rail stops.
        exit: [f64; 2],
        /// Where the exit lands on the sharp edge.
        sharp_parameter: f64,
    },
    /// The seam leaves stripe `leader` first.  The other stripe is closed by
    /// a connector: its section by the leader's other mate face, from the
    /// seam's exit to the sharp edge.
    Asymmetric {
        seam: MarchedCurve,
        seam_fit: MarchedFit,
        leader: usize,
        /// Row parameter on the leader where its other rail stops.
        exit_leader: f64,
        /// The connector's fit, images on the FOLLOWER's blend then on the
        /// leader's other mate face.
        connector_fit: MarchedFit,
        /// Row parameter on the follower where its other rail stops.
        exit_follower: f64,
        sharp_parameter: f64,
    },
}

/// How far `point` stands off `surface` the way the marches here solve "on":
/// along the surface normal at the point's projection.
///
/// The projection clamps to the patch, and a march places points on a patch's
/// continuation — a wall through `evaluate_extended`, a face's plane past its
/// own parameter box — where the clamped foot is away from the point ALONG the
/// patch and not across it.  `march_connector`'s level is this same reading,
/// signed.  Measured on the 2026-09-08 rib-base fixture: a connector whose
/// every sample its march had put on both carriers read 0.303 off them by
/// projection distance, and 1.1e-12 at its samples (9.8e-7 between them) by
/// this.
fn off_surface(surface: &crate::NurbsSurface, point: Vec3) -> Result<f64, KernelRefusal> {
    let foot = crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let normal = raw_normal(surface, foot.u, foot.v)?;
    Ok(point.sub(foot.point).dot(normal).abs())
}

/// Signed distance from `point` to `stripe`'s ruled BEVEL, positive on the
/// side AWAY from the rolling ball's centre.
///
/// This is the chamfer's level, and it is a different surface — not a
/// different section of the same one.  A fillet's wall IS the sibling's
/// radius-`r` canal, so `dist(p, spine) − r` vanishes exactly on it; a
/// chamfer's wall is the CHORD ruling inside that canal, and the canal level
/// would converge on a seam that is nowhere near the bevel, without failing.
/// The sense is carried over unchanged: the ball's centre sits at `−r` on the
/// canal level, and on the negative side of its own chord here, so both
/// levels are positive on the material the sibling has not taken and negative
/// on what it has.  `station`'s two end tests read the same way for either.
///
/// The projection CLAMPS to the bevel's own uv box.  On planar supports —
/// where the bevel is a plane — that costs nothing: `(p − q)·n` is the
/// distance to the whole plane wherever `q` landed on it.  Off a planar
/// support the clamp bounds the level's reach, which the marched rows'
/// overshoot already covers.
fn bevel_level(stripe: &MiterStripe<'_>, point: Vec3, scale: f64) -> Result<f64, KernelRefusal> {
    let projection = crate::project_point_to_surface(&stripe.rows.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let normal = raw_normal(&stripe.rows.surface, projection.u, projection.v)?;
    let center = stripe.center()?.evaluate(projection.u).or_refuse(KernelStage::Refine, "evaluate")?;
    // The ball stands off its own chord by `r·cos(half the dihedral)`, which
    // only collapses on a knife edge.  Take the sign from that stand-off
    // rather than from the blend's own normal orientation, and refuse where
    // there is no stand-off to read it from.
    let seated = center.sub(projection.point).dot(normal);
    if seated.abs() <= 1e-9 * (1.0 + scale) {
        return Err(
            KernelRefusal::unsupported(KernelStage::Refine, "miter_chord_seat", "miter: the chamfer's rolling ball sits on its own chord — the supports meet too \
             sharply to sign the seam's level"),
        );
    }
    Ok(point.sub(projection.point).dot(normal) * -seated.signum())
}

/// Bisection to a relative bar on an interval with a sign change.
fn bisect(
    mut low: f64,
    mut high: f64,
    mut f_low: f64,
    f: &mut dyn FnMut(f64) -> Result<f64, KernelRefusal>,
    bar: f64,
) -> Result<f64, KernelRefusal> {
    for _ in 0..80 {
        let mid = (low + high) * 0.5;
        if (high - low).abs() <= bar {
            return Ok(mid);
        }
        let f_mid = f(mid)?;
        if (f_mid > 0.0) == (f_low > 0.0) {
            low = mid;
            f_low = f_mid;
        } else {
            high = mid;
        }
    }
    Ok((low + high) * 0.5)
}

/// Seam steps between the ball station and the vertex's end of the rows, to
/// find where the seam leaves the strips.
const SEAM_STEPS: usize = 48;
/// Samples over the seam's own length once its exit is known.
const SEAM_SAMPLES: usize = 64;

/// Rungs of the resample ladder a marched curve climbs until its fit is on its
/// carriers ([`fit_marched`]), doubling the seam's samples and the connector's
/// steps on each.  The top rung's refusal is the one that stands.
const MARCH_RUNGS: usize = 4;

/// Where the seam's `step`-th of `samples` resamples sits between P (0) and the
/// exit (1): crowded toward the exit as `1 − (1 − s)²`.
///
/// The seam runs across its strip toward the rail it exits through, so its
/// speed in the row parameter climbs steeply at the exit and the last even span
/// carries the fit's error.  Measured 2026-09-15 on the notched cap,
/// `revolve_test` and the oblique bore mouth: the exit span is 7 to 9 times the
/// median chord and reads 1.8e-5, 7.1e-5 and 1.4e-5 off the carriers, against
/// 9.6e-7, 3.8e-6 and 6.8e-7 on the span before it; doubling the even samples
/// takes only 3.5 to 3.9 off per doubling, where a cubic on smooth data takes
/// 16.  Crowded toward the exit, 64 samples read 1.5e-8, 7.7e-8 and 1.1e-8.
///
/// Even samples on rung zero, crowded above it, sent 104 of the lib suite's
/// 142 seams and all 12 of the case corpus's up a rung, paying a whole
/// resample to throw away; crowded from rung zero, none of the lib suite's
/// 142, the integration suite's 82 or the corpus's 12 climbs.
fn seam_spacing(step: usize, samples: usize) -> f64 {
    let s = step as f64 / samples as f64;
    1.0 - (1.0 - s) * (1.0 - s)
}

/// Climb the resample ladder: march rung by rung and keep the first whose fit
/// stands on its carriers.  Only [`MARCHED_FIT_OFF_CARRIERS`] climbs; any other
/// failure, and the top rung's refusal, is returned as it came.
fn fit_on_ladder(
    march: &dyn Fn(usize) -> Result<MarchedCurve, KernelRefusal>,
    what: &str,
    off_carriers: &dyn Fn(Vec3) -> Result<f64, KernelRefusal>,
    tolerance: f64,
) -> Result<(MarchedCurve, MarchedFit), KernelRefusal> {
    let mut rung = 0;
    loop {
        let curve = march(rung)?;
        match fit_marched(&curve, what, off_carriers, tolerance) {
            Ok(fit) => return Ok((curve, fit)),
            Err(error) if is_marched_fit_off_carriers(&error) && rung + 1 < MARCH_RUNGS => {
                rung += 1;
            }
            Err(error) => return Err(error),
        }
    }
}

/// March the seam and decide how the corner closes.
///
/// `scale` is the model scale for the tolerance bars; `sharp` the vertex's
/// unselected third edge.  `fit_tolerance` is what the seam's and the
/// connector's fits are held to against their carriers (the kernel's
/// `intersection_fit`): each climbs [`fit_on_ladder`] and refuses by name at
/// its top.
pub(super) fn solve_miter(
    stripes: [&MiterStripe<'_>; 2],
    radius: f64,
    chamfer: bool,
    sharp: &EdgeRecord,
    scale: f64,
    fit_tolerance: f64,
) -> Result<MiterClosure, KernelRefusal> {
    let [first, second] = stripes;
    // The bar the EXIT is bisected to, and the exit is an endpoint of the
    // fitted seam, so its error does not stay local — it tilts the whole curve
    // away from every sample between it and P.  Both levels are smooth with an
    // O(1) gradient in the row parameter, so the root is there to be had at the
    // parameter's own resolution, and it is worth having: at the old
    // `1e-11·(1 + scale)` the tilt left a seam that is exactly straight sitting
    // 4.5e-11 off its own chord, which is enough for `fit_marched` to miss that
    // it is a line.  A row parameter is a PARAMETER; the bar does not scale
    // with the model.
    let bar = 1e-14;
    let center_second = second.center()?;
    // Distance-to-the-sibling's-spine level: zero where a point of stripe 0
    // lies on stripe 1.  A CHAMFER's wall is the chord ruling INSIDE that
    // canal, not the canal, so the level asks the same question of the right
    // surface (§6.11) — see `bevel_level`.
    let sibling_level = |point: Vec3| -> Result<f64, KernelRefusal> {
        if chamfer {
            return bevel_level(second, point, scale);
        }
        Ok(crate::project_point_to_curve(center_second, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?.distance - radius)
    };
    let level = |u: f64, fraction: f64| -> Result<f64, KernelRefusal> {
        sibling_level(first.point(u, fraction)?)
    };
    // Where a point lands on the second stripe, as (u, fraction from the
    // shared rail, how far the projection had to clamp to get there).  A seam
    // point is ON stripe 1's canal by construction, so a non-zero distance is
    // the strip's own boundary: the point has run PAST a rail and the
    // projection clamped it back onto one.
    let on_second = |point: Vec3| -> Result<([f64; 2], f64, f64), KernelRefusal> {
        let projection = crate::project_point_to_surface(&second.rows.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        Ok((
            [projection.u, projection.v],
            second.fraction_of_z(projection.v),
            projection.distance,
        ))
    };
    // The seam's samples on the second stripe are its pcurve there, ONE branch
    // of that surface from the ball's own station outward: each continues from
    // the previous sample's foot. `on_second` stays the nearest-point question,
    // because the exit measure below is a level, not a sample.
    let sample_second = |point: Vec3, seed: [f64; 2]| -> Result<[f64; 2], KernelRefusal> {
        let projection =
            crate::projection::project_point_to_surface_from_seed(&second.rows.surface, point, seed).or_refuse(KernelStage::Refine, "project_point_to_surface_from_seed")?;
        Ok([projection.u, projection.v])
    };
    // Signed measure of stripe 1's far rail: negative while the seam is still
    // on the strip, zero on the rail, positive once it has run off.  The
    // fraction alone cannot say — it CLAMPS at 1 — so past the rail the
    // measure is the distance the projection clamped by.
    let past_second = |point: Vec3| -> Result<f64, KernelRefusal> {
        let (_, fraction, distance) = on_second(point)?;
        Ok(if fraction < 1.0 { fraction - 1.0 } else { distance })
    };

    let u_start = first.u_ball;
    let u_end = first.u_limit();
    let span = u_end - u_start;
    if !(span.abs() > 1e-9) {
        return Err(KernelRefusal::internal(KernelStage::Refine, "miter_vertex_station", "miter: the ball station sits at the vertex end of the rows"));
    }
    let du = span / SEAM_STEPS as f64;

    let mut points = vec![first.point(u_start, 0.0)?];
    let mut uv_first = vec![[u_start, first.z(0.0)]];
    let mut uv_second = vec![[second.u_ball, second.z(0.0)]];

    // One station of the seam: the fraction where the level crosses zero, or
    // `None` when the level is non-negative on the whole other-rail end (the
    // seam has already left this strip).
    let station = |u: f64| -> Result<Option<(f64, Vec3)>, KernelRefusal> {
        let at_far = level(u, 1.0)?;
        if at_far >= 0.0 {
            return Ok(None);
        }
        let at_near = level(u, 0.0)?;
        if at_near <= 0.0 {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "miter_rail_reentry", "miter: the shared rail re-enters the sibling blend"));
        }
        let fraction = bisect(0.0, 1.0, at_near, &mut |f| level(u, f), 1e-13)?;
        Ok(Some((fraction, first.point(u, fraction)?)))
    };
    // The seam's own point at `u`, or — once stripe 0 has been left — the far
    // rail point the seam ran out through, so both exit measures stay defined
    // (and continuous) across each other's crossing.
    let seam_point = |u: f64| -> Result<Vec3, KernelRefusal> {
        Ok(match station(u)? {
            Some((_, point)) => point,
            None => first.point(u, 1.0)?,
        })
    };

    let mut previous_u = u_start;
    let mut exit: Option<(usize, f64)> = None; // (stripe that exits, u on stripe 0)
    for step in 1..=SEAM_STEPS + 8 {
        let u = u_start + du * step as f64;
        // Past the marched rows: refuse rather than extrapolate.
        if (u - u_end) * span.signum() > du.abs() * 8.0 {
            break;
        }
        let station_here = station(u)?;
        let left_first = station_here.is_none();
        let left_second = past_second(seam_point(u)?)? >= 0.0;
        if let (false, false, Some((fraction, point))) = (left_first, left_second, station_here) {
            let uv2 = sample_second(point, uv_second[uv_second.len() - 1])?;
            points.push(point);
            uv_first.push([u, first.z(fraction)]);
            uv_second.push(uv2);
            previous_u = u;
            continue;
        }
        // One or both strips were left inside THIS step — on an asymmetric
        // corner the two rails reach the sharp edge a fraction of a step
        // apart, so whichever was left first is the one the corner closes on.
        // Bisecting only the one the step happened to notice picks the wrong
        // leader and hands the connector a section it has already run past.
        let mut candidates: Vec<(usize, f64)> = Vec::new();
        if left_first {
            let mut predicate =
                |candidate: f64| -> Result<f64, KernelRefusal> { level(candidate, 1.0) };
            let f_low = predicate(previous_u)?;
            candidates.push((0, bisect(previous_u, u, f_low, &mut predicate, bar)?));
        }
        if left_second {
            let mut predicate =
                |candidate: f64| -> Result<f64, KernelRefusal> { past_second(seam_point(candidate)?) };
            let f_low = predicate(previous_u)?;
            candidates.push((1, bisect(previous_u, u, f_low, &mut predicate, bar)?));
        }
        exit = candidates
            .into_iter()
            .min_by(|a, b| (a.1 - u_start).abs().total_cmp(&(b.1 - u_start).abs()));
        break;
    }
    let Some((leaves_first, u_exit)) = exit else {
        return Err(
            KernelRefusal::unsupported(KernelStage::Refine, "miter_radius_too_large", "miter: the seam does not leave either blend within the marched rows — the \
             radius is too large for this corner"),
        );
    };
    // The exit point, and whether the other stripe leaves there too.  Its foot
    // on stripe 1 continues from the COARSE march's last sample: the resample
    // below is a closure the ladder may run more than once, so its own last
    // foot is not available here, and both seeds sit on the same branch.
    let (exit_point, exit_uv_first, exit_uv_second, exit_u_second) = if leaves_first == 0 {
        let point = first.point(u_exit, 1.0)?;
        let uv2 = sample_second(point, uv_second[uv_second.len() - 1])?;
        (point, [u_exit, first.z(1.0)], uv2, uv2[0])
    } else {
        let (fraction, point) = station(u_exit)?
            .ok_or(KernelRefusal::internal(KernelStage::Refine, "miter_exit_seam", "miter: the exit station lost the seam"))?;
        let uv2 = sample_second(point, uv_second[uv_second.len() - 1])?;
        (point, [u_exit, first.z(fraction)], [uv2[0], second.z(1.0)], uv2[0])
    };
    // The exit is known: resample the seam densely between P and the exit so
    // the fitted edge rides both blends between samples too.  The coarse
    // march above stepped the whole row span and put only a dozen samples on
    // a seam that leaves within a fifth of it; a cubic through those bows
    // ~1e-4 off the cylinders at r = 1.  Sixty-four samples over the seam's
    // own length, crowded toward the exit (`seam_spacing`), bring that to the
    // fit's noise.
    let resample = |rung: usize| -> Result<MarchedCurve, KernelRefusal> {
        let samples = SEAM_SAMPLES << rung;
        let mut points = vec![first.point(u_start, 0.0)?];
        let mut uv_first = vec![[u_start, first.z(0.0)]];
        let mut uv_second = vec![[second.u_ball, second.z(0.0)]];
        for step in 1..samples {
            let u = u_start + (u_exit - u_start) * seam_spacing(step, samples);
            let Some((fraction, point)) = station(u)? else {
                return Err(KernelRefusal::internal(KernelStage::Refine, "miter_seam_resample", "miter: the seam vanished while being resampled"));
            };
            let uv2 = sample_second(point, *uv_second.last().expect("a seeded sample"))?;
            points.push(point);
            uv_first.push([u, first.z(fraction)]);
            uv_second.push(uv2);
        }
        points.push(exit_point);
        uv_first.push(exit_uv_first);
        uv_second.push(exit_uv_second);
        Ok(MarchedCurve {
            points,
            uv_first,
            uv_second,
        })
    };
    // The seam is evaluated on stripe 0's wall and bisected onto stripe 1's
    // level, so those are what its fit is read against.
    let seam_off_carriers = |point: Vec3| -> Result<f64, KernelRefusal> {
        Ok(off_surface(&first.rows.surface, point)?.max(sibling_level(point)?.abs()))
    };
    let (seam, seam_fit) = fit_on_ladder(&resample, "seam", &seam_off_carriers, fit_tolerance)?;

    let on_sharp = crate::project_point_to_curve(&sharp.curve, exit_point).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
    let symmetric_bar = 1e-6 * (1.0 + radius);
    if on_sharp.distance <= symmetric_bar {
        return Ok(MiterClosure::Symmetric {
            seam,
            seam_fit,
            exit: [u_exit, exit_u_second],
            sharp_parameter: on_sharp.u,
        });
    }

    // Asymmetric: the follower is closed by its section on the leader's
    // other mate face, from the exit point to the sharp edge.
    let (leader, follower) = if leaves_first == 0 {
        (first, second)
    } else {
        (second, first)
    };
    let (follower_uv_at_exit, follower_fraction_at_exit) = if leaves_first == 0 {
        (exit_uv_second, second.fraction_of_z(exit_uv_second[1]))
    } else {
        (exit_uv_first, first.fraction_of_z(exit_uv_first[1]))
    };
    let march = |rung: usize| {
        march_connector(
            follower,
            leader.other_face,
            follower_uv_at_exit[0],
            follower_fraction_at_exit,
            exit_point,
            scale,
            CONNECTOR_STEPS << rung,
        )
    };
    // Evaluated on the follower's wall, bisected onto the leader's other face.
    let connector_off_carriers = |point: Vec3| -> Result<f64, KernelRefusal> {
        Ok(off_surface(&follower.rows.surface, point)?
            .max(off_surface(&leader.other_face.surface, point)?))
    };
    let (connector, connector_fit) =
        fit_on_ladder(&march, "connector", &connector_off_carriers, fit_tolerance)?;
    let connector_end = *connector
        .points
        .last()
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "miter_connector_samples", "miter: the connector has no samples"))?;
    let on_sharp = crate::project_point_to_curve(&sharp.curve, connector_end).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
    if on_sharp.distance > 1e-5 * (1.0 + radius) {
        return Err(KernelRefusal::internal(KernelStage::Refine, "miter_connector_end", format!(
            "miter: the connector ends {:.3e} off the sharp edge",
            on_sharp.distance
        )));
    }
    let exit_follower = connector.uv_first.last().map(|uv| uv[0]).unwrap_or(0.0);
    Ok(MiterClosure::Asymmetric {
        seam,
        seam_fit,
        leader: leaves_first,
        // The seam is marched on stripe 0's domain, so `u_exit` is stripe 0's
        // row parameter.  The caller trims the LEADER's rail at this station,
        // and when the leader is stripe 1 that is the exit's parameter on
        // stripe 1's own rows.
        exit_leader: if leaves_first == 0 { u_exit } else { exit_u_second },
        connector_fit,
        exit_follower,
        sharp_parameter: on_sharp.u,
    })
}

/// Connector steps from the seam's exit to the sharp edge, on the ladder's
/// first rung; each later rung doubles them.
const CONNECTOR_STEPS: usize = 24;

/// The follower's section by `face`, from `(u0, fraction0)` (the seam's exit,
/// already on the face) to the follower's other rail.  Parameterised by the
/// section fraction, solving the station parameter at each — the section by
/// a face through the vertex is close to one station of the follower, so the
/// station parameter is the unknown, not the fraction.
fn march_connector(
    follower: &MiterStripe<'_>,
    face: &FaceRecord,
    u0: f64,
    fraction0: f64,
    start: Vec3,
    scale: f64,
    steps: usize,
) -> Result<MarchedCurve, KernelRefusal> {
    // `bisect` below converges an interval of the follower's ROW PARAMETER `u`,
    // and `station_parameters` normalises that parameter to [0, 1] (chord length
    // over the mean support polyline). So `u` is dimensionless and does not grow
    // with the model, while `scale` is `march_model_scale` — a LENGTH. Comparing
    // one against the other is dimensionally inconsistent, and it was inconsistent
    // in the expensive direction: the position the crossing lands on is off by
    // roughly `bar * |dP/du|`, and `|dP/du|` is itself about the row's arc length,
    // so multiplying by the scale made the positional error grow as the SQUARE of
    // model size — about 1e-9 on a 10 mm model and 1e-5 on a metre.
    //
    // Dividing is what turns a length tolerance into the equivalent parameter
    // tolerance, which makes the positional error scale-invariant at ~1e-11
    // instead. The floor keeps a very large model from spending all 80 bisection
    // steps chasing a bar below what f64 can resolve on a [0, 1] interval.
    //
    // The three sibling bars are NOT this defect and are deliberately untouched:
    // `stations.rs` (both) and `corner/ball.rs` compare a RESIDUAL whose every
    // component is a length — point-to-point mismatches, and a displacement
    // dotted with a NORMALISED section tangent — so scaling those by model size
    // is a relative tolerance and correct.
    let bar = (1e-11 / (1.0 + scale)).max(1e-15);
    let signed_distance = |point: Vec3| -> Result<f64, KernelRefusal> {
        let projection = crate::project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        let normal = raw_normal(&face.surface, projection.u, projection.v)?;
        Ok(point.sub(projection.point).dot(normal))
    };
    // Bracket the station parameter around the exit, on the side of the
    // marched rows that reaches the vertex.
    let u_end = follower.u_limit();
    let mut points = vec![start];
    let mut uv_follower = vec![[u0, follower.z(fraction0)]];
    let first_projection = crate::project_point_to_surface(&face.surface, start).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let mut uv_face = vec![[first_projection.u, first_projection.v]];
    let mut previous_u = u0;
    for step in 1..=steps {
        let fraction = fraction0 + (1.0 - fraction0) * step as f64 / steps as f64;
        // Signed distance to the face along the station parameter at this
        // fraction; bracket from the previous station toward the row end and
        // back, whichever side changes sign.
        let mut g = |u: f64| -> Result<f64, KernelRefusal> {
            signed_distance(follower.point(u, fraction)?)
        };
        let g_prev = g(previous_u)?;
        let reach = (u_end - previous_u).abs().max(1e-9);
        let mut found = None;
        for direction in [(u_end - previous_u).signum(), -(u_end - previous_u).signum()] {
            let mut width = reach * 0.05;
            for _ in 0..8 {
                let candidate = previous_u + direction * width;
                let g_candidate = g(candidate)?;
                if (g_candidate > 0.0) != (g_prev > 0.0) {
                    let (low, high, f_low) = if candidate > previous_u {
                        (previous_u, candidate, g_prev)
                    } else {
                        (candidate, previous_u, g_candidate)
                    };
                    found = Some(bisect(low, high, f_low, &mut g, bar)?);
                    break;
                }
                width *= 2.0;
            }
            if found.is_some() {
                break;
            }
        }
        let u = match found {
            Some(u) => u,
            None if g_prev.abs() <= 1e-9 * (1.0 + scale) => previous_u,
            None => {
                return Err(KernelRefusal::internal(KernelStage::Refine, "miter_connector_face", "miter: the connector left the face without crossing it"))
            }
        };
        let point = follower.point(u, fraction)?;
        // The connector's pcurve on the face is ONE branch: each sample
        // continues from the previous sample's foot.
        let seed = *uv_face.last().ok_or(KernelRefusal::internal(KernelStage::Refine, "miter_connector_samples", "miter: the connector has no face sample"))?;
        let projection =
            crate::projection::project_point_to_surface_from_seed(&face.surface, point, seed).or_refuse(KernelStage::Refine, "project_point_to_surface_from_seed")?;
        points.push(point);
        uv_follower.push([u, follower.z(fraction)]);
        uv_face.push([projection.u, projection.v]);
        previous_u = u;
    }
    Ok(MarchedCurve {
        points,
        uv_first: uv_follower,
        uv_second: uv_face,
    })
}

/// The band a marched curve's samples must sit inside the straight line
/// between its ends — at their own chord-length parameters — to BE that line.
///
/// It is the march's own arithmetic, not a shape allowance: each sample is
/// placed by a bisection that stops at `1e-13` of the section's width, so a
/// sample of a genuinely straight seam sits that far off the line and no
/// further, while a seam with any curvature to it bows orders of magnitude
/// more.  `BREP_DEBUG_NETWORK` prints the measured deviation of every marched
/// curve so the band can be read against the real numbers.
///
/// # What this band now decides, and what it was derived against
///
/// Its derivation above is BISECTION NOISE — "the march's own arithmetic". That
/// was the whole story while `march_connector`'s own bar was
/// `1e-11 * (1.0 + scale)`, which scaled a NORMALISED row parameter by model
/// size and left every connector sitting orders over this band on noise alone:
/// the groove rim's read 3.592e-9 against a 1.791e-11 bar, 200x over, and no
/// connector could come near collapsing. Inverting that bar (see the comment at
/// the head of `march_connector`) drops the same connector to 3.514e-12, 5.1x
/// UNDER. So this band's job changed with it: it used to separate "straight"
/// from "noisy samples", and it now separates "straight" from "genuinely curved
/// by a few times 1e-11" — a finer judgement than the number was chosen for.
///
/// Surveyed over `cargo test --test all` under `BREP_DEBUG_NETWORK` at the
/// commit that inverted that bar: 72 marched curves, 20 distinct shapes.
///
/// ```text
/// no collapse (some deviation over its bar)   59   closest 8.05e+08x OVER
/// collapses (worst deviation < bar/3)         13   closest 0.1962 of its bar
/// all three under, worst within 3x of bar      0
/// ```
///
/// About nine orders of empty parameter space between the two populations, so
/// nothing in that suite is a judgement call. That is evidence the band
/// separates cleanly in practice; it is not the reason the band is safe.
///
/// # Why a near-miss does not matter, and what actually constrains this
///
/// It is tempting to read "a curve that only just fits under the band" as the
/// dangerous case. It is not. `line_deviation` maxes over the SAMPLES at their
/// own chord-length parameters, so all-three-under says the sampled polyline
/// lies within `bar` of the chord — and replacing it with that chord therefore
/// moves nothing by more than `bar` itself, `1e-11·(1 + extent)`, about 1.1e-10
/// on a 10 mm model against the 1e-7 the kernel treats as linear. A curve at
/// 0.9 of the band and one at 0.001 of it are both discarded at a magnitude
/// nothing downstream can see, so "genuinely curved or merely noisy" has no
/// consequence here.
///
/// Which tolerance, though, decides whether that margin is constant — and the
/// 1e-7 above is the kernel's STRICTEST, not the one that adjudicates this
/// property. Against `model` alone the margin does shrink with size: 909x on a
/// 10 mm part, 10x at a metre, and exactly 1x at a ten-metre extent. But what
/// actually tests an edge's curve against its pcurve — the thing this collapse
/// changes — is `KernelTolerances::pcurve_acceptance`, which is
/// `max(pcurve_consistency, 0.025 · model diagonal)` and so scales WITH the
/// model. Both sides then grow together and the margin is AT LEAST about 2.5e9
/// at every size: 1.1e-10 against 0.25 on a 10 mm part, 1.0e-7 against 250 at
/// ten metres. There is no crossover against the comparand that matters.
///
/// At least, not exactly, and the gap is in the reader's favour: this band
/// scales with the CURVE's own chord (`extent` is `last - first` of its
/// samples) while the limit scales with the MODEL's diagonal. Those are equal
/// only for a curve spanning the whole model, which is the worst case and the
/// one these figures assume. A miter seam or connector is a blend-sized feature
/// on a part-sized model, so its extent is orders below the diagonal and the
/// real margin is larger still.
///
/// Quote the figures and the comparand, never an order count: the count hides
/// the model size, and a bare crossover hides which tolerance it crossed. Both
/// read as facts about the collapse when they are facts about a comparison. The band's JOB changed when `march_connector`'s bar was
/// inverted; its SAFETY never depended on that.
///
/// What the safety does depend on is the PAIR (band, sample count), because the
/// bound above holds only at the samples. Re-derive this if the band is ever
/// LOOSENED, or if `SEAM_SAMPLES` (64) or `CONNECTOR_STEPS` (24) is ever cut far
/// enough that a smooth curve could bow between samples by more than the band.
/// Neither is a free parameter once the other is fixed.
const MARCH_STRAIGHT: f64 = 1e-11;

/// How far `points` sit from the straight line between their ends, measured at
/// their own `parameters` — so this answers both questions the collapse below
/// needs: whether the samples are COLLINEAR, and whether the chord-length
/// parameterisation is already the line's own affine one.
fn line_deviation(points: &[Vec3], parameters: &[f64]) -> f64 {
    let (Some(first), Some(last)) = (points.first(), points.last()) else {
        return f64::INFINITY;
    };
    let travel = last.sub(*first);
    points
        .iter()
        .zip(parameters)
        .fold(0.0f64, |worst, (point, parameter)| {
            worst.max(point.sub(first.add(travel.scale(*parameter))).length())
        })
}

/// The refusal [`fit_marched`] answers with when the curve it fitted leaves the
/// surfaces it was marched on.
pub(crate) const MARCHED_FIT_OFF_CARRIERS: &str = "miter: a marched curve's fit leaves its carriers";

/// The slug of that refusal (`NonConvergence`).
pub(crate) const MARCHED_FIT_OFF_CARRIERS_WHAT: &str = "marched_fit_off_carriers";

/// Is this the refusal of a marched fit that leaves its carriers ([`MARCHED_FIT_OFF_CARRIERS`])?
/// Read off the class and its slug, which the mint site and this check share.
pub(crate) fn is_marched_fit_off_carriers(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, RefusalClass::NonConvergence { what } if what == MARCHED_FIT_OFF_CARRIERS_WHAT)
}

/// Fit a marched curve and its two images with ONE chord-length
/// parameterisation, so the pcurves reproduce the 3D curve parameter for
/// parameter.  Returns (curve, first image, second image).
///
/// # The fit is measured, between its samples, against its carriers
///
/// An interpolant passes through its samples, so the samples cannot say
/// whether the curve between them is the marched curve.  `off_carriers` is the
/// march's own witness — how far a point lies from the two surfaces the curve
/// was solved on — and the returned 3D curve is read by it at 0.25, 0.5 and
/// 0.75 of every span between samples, the fit record's between-station
/// sampling.  Against the carriers, not the sample polyline: a seam's chord
/// sags off its own curve by more than any fit tolerance (2.8e-3 over the last
/// span of a 64-sample seam at r = 0.5), and a fit that is right between its
/// samples leaves the polyline by exactly that.  A curve further off its
/// carriers than `tolerance` is refused by name with the number
/// ([`MARCHED_FIT_OFF_CARRIERS`]); `what` names the curve.
///
/// Measured on the 2026-09-15 notched-cap report at `2dd14cdef`: an asymmetric
/// miter's connector had no section left to cross and marched one chord of
/// 1.444e-5 and then 23 of 1.19e-12, so 24 of its 25 chord-length parameters
/// sat within 1.9e-6 of 1.  The interpolant passed through every sample to
/// 4e-15, stood 38 units off them at the first span's middle with poles 102
/// units out, and nothing noticed until the network's acceptance read the
/// edge 44 units off its pcurve.
///
/// A STRAIGHT marched curve is returned as the line it is rather than fitted.
/// A chamfer miter on planar supports has an exactly straight seam — the two
/// bevel planes meet in a line — and interpolating the 64 resamples of it
/// spends 65 control points saying so, less accurately than the two a line
/// needs.  The collapse is all-or-nothing across the 3D curve and both images
/// because they share one parameterisation: a line's is affine in the
/// chord-length parameter, which is exactly what `line_deviation` measures.
pub(super) fn fit_marched(
    curve: &MarchedCurve,
    what: &str,
    off_carriers: &dyn Fn(Vec3) -> Result<f64, KernelRefusal>,
    tolerance: f64,
) -> Result<(crate::NurbsCurve, crate::NurbsCurve, crate::NurbsCurve), KernelRefusal> {
    use crate::Vec4;
    let count = curve.points.len();
    if count < 2 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "marched_samples", "miter: a marched curve needs at least two samples"));
    }
    let mut parameters = Vec::with_capacity(count);
    let mut accumulated = 0.0;
    parameters.push(0.0);
    for pair in curve.points.windows(2) {
        accumulated += pair[1].sub(pair[0]).length();
        parameters.push(accumulated);
    }
    let total = accumulated.max(1e-12);
    for parameter in &mut parameters {
        *parameter /= total;
    }
    let degree = FIT_DEGREE.min(count - 1);
    let flatten = |samples: &[[f64; 2]]| {
        samples
            .iter()
            .map(|uv| Vec3::new(uv[0], uv[1], 0.0))
            .collect::<Vec<_>>()
    };
    let as_points = |samples: &[Vec3]| {
        samples
            .iter()
            .map(|point| Vec4::from_point(*point, 1.0))
            .collect::<Vec<_>>()
    };
    let first_image = flatten(&curve.uv_first);
    let second_image = flatten(&curve.uv_second);
    let extent = |points: &[Vec3]| {
        let (Some(first), Some(last)) = (points.first(), points.last()) else {
            return 0.0;
        };
        last.sub(*first).length()
    };
    let straight: Vec<(f64, f64)> = [&curve.points[..], &first_image[..], &second_image[..]]
        .into_iter()
        .map(|points| {
            (
                line_deviation(points, &parameters),
                MARCH_STRAIGHT * (1.0 + extent(points)),
            )
        })
        .collect();
    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
        eprintln!(
            "marched curve: {count} samples, off its own chord 3d {:.3e} / first {:.3e} / \
             second {:.3e} (bars {:.3e} / {:.3e} / {:.3e})",
            straight[0].0, straight[1].0, straight[2].0,
            straight[0].1, straight[1].1, straight[2].1,
        );
    }
    let fitted = if straight.iter().all(|(deviation, bar)| deviation <= bar) {
        let line = |points: &[Vec3]| -> Result<crate::NurbsCurve, KernelRefusal> {
            crate::make_line(points[0], points[count - 1]).or_refuse(KernelStage::Refine, "make_line")
        };
        (
            line(&curve.points)?,
            line(&first_image)?,
            line(&second_image)?,
        )
    } else {
        (
            crate::fit::interpolate_homogeneous(&as_points(&curve.points), degree, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?,
            crate::fit::interpolate_homogeneous(&as_points(&first_image), degree, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?,
            crate::fit::interpolate_homogeneous(&as_points(&second_image), degree, &parameters).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?,
        )
    };
    // A NaN reading is the worst there is, and stays so.
    let worse = |worst: f64, off: f64| !worst.is_nan() && (off.is_nan() || off > worst);
    let mut worst = 0.0f64;
    let mut worst_span = 0;
    for (span, pair) in parameters.windows(2).enumerate() {
        for fraction in [0.25, 0.5, 0.75] {
            let off = off_carriers(fitted.0.evaluate(pair[0] + (pair[1] - pair[0]) * fraction).or_refuse(KernelStage::Refine, "evaluate")?)?;
            if worse(worst, off) {
                (worst, worst_span) = (off, span);
            }
        }
    }
    // The samples' own reading is for the refusal and the debug line only.
    let debug = std::env::var("BREP_DEBUG_NETWORK").is_ok();
    let mut at_samples = f64::NAN;
    if debug || !(worst <= tolerance) {
        at_samples = 0.0;
        for point in &curve.points {
            let off = off_carriers(*point)?;
            if worse(at_samples, off) {
                at_samples = off;
            }
        }
    }
    if debug {
        eprintln!(
            "marched {what} [{}]: {count} samples, fit off its carriers {worst:.3e} between them \
             (span {worst_span}), {at_samples:.3e} at them, tolerance {tolerance:.3e}",
            std::thread::current().name().unwrap_or("main"),
        );
    }
    if !(worst <= tolerance) {
        return Err(KernelRefusal::non_convergence(KernelStage::Refine, MARCHED_FIT_OFF_CARRIERS_WHAT, format!(
            "{MARCHED_FIT_OFF_CARRIERS}: the {what}'s fit stands {worst:.3e} off them between its \
             {count} samples (span {worst_span}; the samples themselves {at_samples:.3e}), over the \
             fit tolerance {tolerance:.3e}"
        )));
    }
    Ok(fitted)
}
