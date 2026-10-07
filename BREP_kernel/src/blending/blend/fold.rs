//! Does the wall a march would build FOLD THROUGH ITSELF?
//!
//! A constant-radius blend surface is the envelope of one ball rolling along
//! its centre curve, so it is a CANAL surface, and a canal surface of radius ρ
//! folds exactly where the centre curve turns tighter than ρ — the ball's
//! characteristic circle sweeps past the centre of curvature and comes back
//! through itself. The kernel's own section says the same thing in Pappus
//! form: the volume element `(1 − κ ξ) dA ds` changes SIGN inside the section
//! when `ρ κ > 1`, and a sign change there is not a small error, it is two
//! parameters carrying one point.
//!
//! The refusal is TERMINAL, like [`super::BALL_OFF_CARRIER`], and for the same
//! reason: the shape of the answer is wrong, not the lane. Neither the exact
//! cutter nor the maximal-valid-subset search builds the trimmed envelope a
//! folded wall needs; they build the same self-intersecting surface or none.
//!
//! ## What is measured, and why it is not a chord
//!
//! Three consecutive MARCH STATIONS are useless for this: on the 2026-09-02
//! two-collar document the tightest bend is sampled with centre chords of
//! 1.500 on a curve whose radius of curvature is 1.512, and the circumradius
//! through those three stations reads 1.97 — clean, at a place that folds by
//! 32 %. The fold factor here is measured by RE-SOLVING the tangency system a
//! small step either side of the station, so the circumradius is a converged
//! finite difference of the true centre curve rather than a chord of the
//! march's own sampling.
//!
//! ## A tapered ball
//!
//! When the radius varies along the edge the wall is no longer a canal
//! surface, and the picture of the envelope of spheres — its characteristic
//! circle tilting out of the normal plane by `asin(−dρ/ds)` — describes a
//! surface the kernel does not build. What the march builds is the loft of
//! PLANAR circular sections: at every station a circle of the LOCAL radius
//! centred on the ball, in the plane of the ball's centre and its two contacts
//! (`apex_point` takes exactly that plane). For that surface,
//! `x(t, ψ) = c(t) + ρ(t) û(t, ψ)` with `û` in the section plane of unit normal
//! `m(t)`, the out-of-plane rate is
//!
//! ```text
//! x_t · m = c' · m − ρ (û · m')          (û · m ≡ 0, so û' · m = −û · m')
//! ```
//!
//! and the Pappus sign change — two parameters on one point — is where that
//! vanishes on the section: `ρ (û · k_s) ≥ 1` with the SECTION TURN `k_s = m'
//! / (c' · m)`. `ρ'` has no term of its own; it enters only through the
//! sections themselves — as the ball grows its contacts slide, and the plane
//! through the centre and the contacts turns with them, which is a part of
//! `m'` the probes measure directly. On planar mates that part is zero, so a
//! taper along a straight box edge sits in distinct parallel planes and cannot
//! fold however the radius law bends; on the 2026-09-02 collar, whose tube the
//! plane grazes, it is the dominant part (a symmetric taper peaking at 2.2
//! over the bend reads 4.7 where the constant r = 2 reads 1.33). When the
//! section plane is normal to the centre curve, `m = T` and `k_s` is the
//! centre curve's curvature vector, so the constant-radius factor above is
//! this one's special case (the two read 1.3341001 and 1.3340625 at the
//! constant collar's peak station). The tapered verdict reads `m` and `c` off
//! the same re-solved probes, differenced centrally at an interior station and
//! one-sidedly at the two end stations, because the open lane's radius law is
//! CLAMPED past the edge's ends: a probe straddling that kink through a
//! circumcircle read a radius of curvature 0.024230 against ρ = 2 on the
//! tapered box edge (factor 82), which was the clamp's slope discontinuity and
//! not the wall.
//!
//! ## The section, not the ball
//!
//! `ρ κ > 1` alone is not the condition. The wall spans only the arc between
//! the two contacts, and the fold is at the direction the curvature vector
//! points: a valley whose crevice lies on the CONVEX side of its centre curve
//! never folds however tight the turn. So the factor maximises `ρ (k⃗ · û)`
//! over the section arc — `ρ|k⃗|` when the curvature direction is inside the
//! arc, and the larger endpoint otherwise. On a circular boss's concave rim
//! the curvature direction IS inside the arc and the factor is
//! `ρ/(R + ρ) < 1`; on a bore floor's it is outside it altogether.

use crate::{KernelRefusal, KernelStage, RefusalClass, SoundnessDefect, Vec3};

/// `BREP_BLEND_STATION_TRACE=1` also prints what the fold probe could not
/// measure. A skipped station is not a clean station: on a carrier that is a
/// FITTED wall closed in u, a probe stepping across the seam re-seeds on the
/// wrong branch, and a silent `continue` would hide exactly the case this
/// module exists for.
fn fold_trace(message: std::fmt::Arguments<'_>) {
    if std::env::var("BREP_BLEND_STATION_TRACE").ok().as_deref() == Some("1") {
        eprintln!("{message}");
    }
}

/// Is this refusal a WALL FOLD — the march's verdict that the envelope folds
/// through itself, minted only by [`wall_fold_refusal`]?
///
/// Read off the class: `UnsoundResult { Fold }` at `Classify`, the stage that
/// helper mints at. The healing acceptance mints the same defect at `Validate`
/// for a BUILT result, and that conviction is deliberately not a wall fold: the
/// fallbacks that stop here do so because every lane would build the same
/// folding centre curve, which is not true of a multi-edge assembly that failed
/// acceptance (a subset without the offending edge can be sound). The class
/// survives every wrap (`with_message`), so a fold a rung above has composed
/// into a longer message is still one, and a message that merely QUOTES the
/// fold's text under another class is not.
pub(crate) fn is_wall_fold(refusal: &KernelRefusal) -> bool {
    matches!(
        refusal.class,
        RefusalClass::UnsoundResult {
            defect: SoundnessDefect::Fold,
            ..
        }
    ) && refusal.stage == KernelStage::Classify
}

/// The prefix of the refusal a march makes when the wall it would build folds
/// through itself. Callers match it to tell a fold from a march that merely
/// failed to converge: the fallbacks that answer a non-convergence build the
/// same envelope with the same fold, so this one is reported as-is.
pub(crate) const WALL_FOLDS: &str = "blend: no wall of radius";

/// Mint a wall-fold refusal. The envelope this march would build carries one
/// face of its own domain twice onto one 3-D point — the soundness defect
/// `healing/accept.rs` convicts a BUILT face of — read here before the face
/// exists, from the ball's centre curve; `faces` are the mates it was measured
/// against (the one the fold reaches, or both). Every message composed under
/// [`WALL_FOLDS`] is minted here so the class stays one across the carve's
/// band refusals (`carve::band_refusal`) and the two verdicts below.
pub(super) fn wall_fold_refusal(faces: Vec<u64>, message: String) -> KernelRefusal {
    KernelRefusal::unsound(KernelStage::Classify, SoundnessDefect::Fold, faces, message)
}

/// How far either side of a station the tangency system is re-solved, as a
/// fraction of the marched parameter span. The circumradius error is
/// `O((chord/R)²)`: on the 2026-09-02 collar this step puts the chord near
/// 2e-2 against a radius of 1.5, so the curvature is right to about 1e-4
/// relative — three decades below the margin the verdict turns on.
const PROBE_FRACTION: f64 = 5.0e-4;

/// Sub-samples spent refining around the tightest station. The factor is a
/// smooth function of the edge parameter with one dominant peak per bend, and
/// the station nearest the peak is adjacent to it; refining can only RAISE the
/// measured factor, so it adds refusals for real folds and never removes one.
const REFINE_SAMPLES: usize = 8;

/// One station's fold measurement. The radius is the LOCAL one: a tapered
/// blend's ball is a different size at every station, and judging a thin end
/// by the thick end's radius would refuse shapes that build.
#[derive(Clone, Copy)]
pub(super) struct FoldSample {
    /// `ρ · max over the section arc of (curvature vector · direction)`. One
    /// or more is a fold. The vector is the centre curve's curvature for a
    /// constant radius and the section turn `m' / (c' · m)` for a tapered one.
    pub(super) factor: f64,
    /// The centre curve's radius of curvature here (constant radius), or the
    /// sections' radius of turn `1 / |k_s|` (tapered).
    pub(super) curvature_radius: f64,
    pub(super) t: f64,
    pub(super) centre: Vec3,
}

/// What a caller has to be able to do for one edge parameter: solve the
/// tangency system there and hand back the ball centre and both contacts.
pub(super) type SolveCentre<'a> =
    &'a dyn Fn(f64, [f64; 4]) -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal>;

/// The circumcircle's curvature vector at `centre`, through the two probes.
fn curvature_vector(before: Vec3, centre: Vec3, after: Vec3) -> Option<Vec3> {
    let u = before.sub(centre);
    let v = after.sub(centre);
    let cross = u.cross(v);
    let denominator = 2.0 * cross.dot(cross);
    if !(denominator > 0.0) || !denominator.is_finite() {
        return None;
    }
    let to_circumcentre = v
        .cross(cross)
        .scale(u.dot(u))
        .add(cross.cross(u).scale(v.dot(v)))
        .scale(1.0 / denominator);
    let squared = to_circumcentre.dot(to_circumcentre);
    if !(squared > 0.0) || !squared.is_finite() {
        return None;
    }
    Some(to_circumcentre.scale(1.0 / squared))
}

/// The fold factor of one station: the largest `ρ (k⃗ · û)` over the arc the
/// section actually spans, which is the minor arc from the first contact to
/// the second (the rational-quadratic section's own arc — its weight
/// `cos(α/2)` is positive exactly when that arc is under a half turn).
fn factor_of(radius: f64, curvature: Vec3, centre: Vec3, p1: Vec3, p2: Vec3) -> Option<f64> {
    let first = p1.sub(centre).normalized().ok()?;
    let second = p2.sub(centre).normalized().ok()?;
    let endpoint = curvature.dot(first).max(curvature.dot(second)) * radius;
    // An orthonormal basis of the section plane: the arc runs from angle 0
    // (the first contact) to `total` (the second), and the curvature vector is
    // perpendicular to the march tangent, so its own out-of-plane part is the
    // probe's error and nothing else.
    let across = second.sub(first.scale(second.dot(first)));
    let Ok(across) = across.normalized() else {
        return Some(endpoint);
    };
    let total = second.dot(first).clamp(-1.0, 1.0).acos();
    let along_first = curvature.dot(first);
    let along_across = curvature.dot(across);
    let angle = along_across.atan2(along_first);
    if !(0.0..=total).contains(&angle) {
        return Some(endpoint);
    }
    let in_plane = along_first.hypot(along_across);
    Some(endpoint.max(radius * in_plane))
}

/// One edge parameter's tangency system solved at `t` and a small step either
/// side: what both the fold factor and the rail test below read.
struct Probe {
    uv: [f64; 4],
    contacts: [Vec3; 2],
    centre: Vec3,
    before: ([Vec3; 2], Vec3),
    after: ([Vec3; 2], Vec3),
}

/// Solve the three sections of one probe.
fn probe(radius: f64, span: f64, t: f64, seed: [f64; 4], solve: SolveCentre<'_>) -> Option<Probe> {
    let step = span.abs() * PROBE_FRACTION;
    let measure = |at: f64| -> Option<([f64; 4], [Vec3; 2], Vec3)> {
        match solve(at, seed) {
            Ok((uv, p1, p2, centre)) => Some((uv, [p1, p2], centre)),
            Err(error) => {
                fold_trace(format_args!("  fold probe at t {at:.9}: {error}"));
                None
            }
        }
    };
    let (uv, contacts, centre) = measure(t)?;
    let (_, before_contacts, before) = measure(t - step)?;
    let (_, after_contacts, after) = measure(t + step)?;
    // A PROBE STEP THAT IS NOT SMALL is not a probe. On a carrier closed in a
    // fitted parameter the Newton can re-seed onto the other branch of a seam
    // and converge a whole period away; the circumradius through that is
    // meaningless in either direction, so the station is reported as unmeasured
    // rather than silently believed.
    for (label, probed) in [("before", before), ("after", after)] {
        let moved = probed.sub(centre).length();
        if !(moved < radius) {
            fold_trace(format_args!(
                "  fold probe at t {t:.9}: the {label} step moved the ball centre \
                 {moved:.6}, which is not less than the radius {radius} — station unmeasured"
            ));
            return None;
        }
    }
    Some(Probe {
        uv,
        contacts,
        centre,
        before: (before_contacts, before),
        after: (after_contacts, after),
    })
}

/// Measure the fold factor at one edge parameter.
fn sample(radius: f64, t: f64, probe: &Probe) -> Option<FoldSample> {
    let curvature = curvature_vector(probe.before.1, probe.centre, probe.after.1)?;
    let [p1, p2] = probe.contacts;
    let factor = factor_of(radius, curvature, probe.centre, p1, p2)?;
    Some(FoldSample {
        factor,
        curvature_radius: 1.0 / curvature.length(),
        t,
        centre: probe.centre,
    })
}

/// Which finite difference a tapered station's section turn is read from.
/// The end stations of an open march sit ON the radius law's clamp, so a
/// difference straddling them would average the wall's rate with the clamped
/// overshoot's; they are read one-sidedly, from the wall's own side.
#[derive(Clone, Copy, PartialEq)]
enum Difference {
    Central,
    Forward,
    Backward,
}

/// The unit normal of the planar section through `centre` and both contacts —
/// the plane `apex_point` puts the built arc in.
fn section_normal(contacts: [Vec3; 2], centre: Vec3) -> Option<Vec3> {
    contacts[0]
        .sub(centre)
        .cross(contacts[1].sub(centre))
        .normalized()
        .ok()
}

/// Measure the TAPERED fold factor at one edge parameter: the section turn
/// `k_s = m' / (c' · m)` from the probe's section normals and ball centres,
/// then the same maximisation over the arc the constant case does.
fn sample_section(radius: f64, t: f64, probe: &Probe, difference: Difference) -> Option<FoldSample> {
    let m = section_normal(probe.contacts, probe.centre)?;
    let oriented = |contacts: [Vec3; 2], centre: Vec3| -> Option<Vec3> {
        let normal = section_normal(contacts, centre)?;
        Some(if normal.dot(m) < 0.0 { normal.scale(-1.0) } else { normal })
    };
    let (m_low, c_low, m_high, c_high) = match difference {
        Difference::Central => (
            oriented(probe.before.0, probe.before.1)?,
            probe.before.1,
            oriented(probe.after.0, probe.after.1)?,
            probe.after.1,
        ),
        Difference::Forward => (m, probe.centre, oriented(probe.after.0, probe.after.1)?, probe.after.1),
        Difference::Backward => (oriented(probe.before.0, probe.before.1)?, probe.before.1, m, probe.centre),
    };
    // `c' · m`, the rate at which the section plane advances along its own
    // normal; a plane the centre does not cross is no section at all.
    let advance = c_high.sub(c_low).dot(m);
    if !(advance.abs() > 0.0) || !advance.is_finite() {
        return None;
    }
    let turn = m_high.sub(m_low).scale(1.0 / advance);
    let [p1, p2] = probe.contacts;
    let factor = factor_of(radius, turn, probe.centre, p1, p2)?;
    let magnitude = turn.length();
    Some(FoldSample {
        factor,
        curvature_radius: if magnitude > 0.0 { 1.0 / magnitude } else { f64::INFINITY },
        t,
        centre: probe.centre,
    })
}

/// The worst fold factor over a marched edge, refined around its peak.
///
/// `stations` are the parameters the march already solved, each with its
/// converged unknowns as the probe's seed. `visit` sees every station's probe
/// first, in march order, and an error from it ends the scan there. A
/// `tapered` march is measured by [`sample_section`], a constant one by
/// [`sample`]; the station trace prints both readings so a station where they
/// part can be read.
fn worst_fold(
    radius_at: &dyn Fn(f64) -> f64,
    span: f64,
    stations: &[(f64, [f64; 4])],
    solve: SolveCentre<'_>,
    tapered: bool,
    visit: &mut dyn FnMut(f64, &Probe) -> Result<(), KernelRefusal>,
) -> Result<Option<FoldSample>, KernelRefusal> {
    let mut worst: Option<(usize, FoldSample)> = None;
    let mut unmeasured = 0usize;
    let t_low = stations.iter().map(|(t, _)| *t).fold(f64::INFINITY, f64::min);
    let t_high = stations.iter().map(|(t, _)| *t).fold(f64::NEG_INFINITY, f64::max);
    let trace = std::env::var("BREP_BLEND_STATION_TRACE").ok().as_deref() == Some("1");
    let measure = |radius: f64, t: f64, probed: &Probe, difference: Difference| -> Option<FoldSample> {
        let measured = if tapered {
            sample_section(radius, t, probed, difference)
        } else {
            sample(radius, t, probed)
        };
        if trace {
            let circumcircle = sample(radius, t, probed).map(|s| s.factor);
            let section = sample_section(radius, t, probed, difference).map(|s| s.factor);
            fold_trace(format_args!(
                "  fold probe at t {t:.9}: radius {radius:.9} factor circumcircle {circumcircle:?} section {section:?}"
            ));
        }
        measured
    };
    for (index, (t, seed)) in stations.iter().enumerate() {
        let radius = radius_at(*t);
        let Some(probed) = probe(radius, span, *t, *seed, solve) else {
            unmeasured += 1;
            continue;
        };
        visit(radius, &probed)?;
        let difference = if *t <= t_low {
            Difference::Forward
        } else if *t >= t_high {
            Difference::Backward
        } else {
            Difference::Central
        };
        let Some(measured) = measure(radius, *t, &probed, difference) else {
            unmeasured += 1;
            continue;
        };
        if worst.map(|(_, best)| measured.factor > best.factor).unwrap_or(true) {
            worst = Some((index, measured));
        }
    }
    if unmeasured > 0 {
        fold_trace(format_args!(
            "fold probe: {unmeasured} of {} station(s) unmeasured",
            stations.len()
        ));
    }
    let Some((index, mut best)) = worst else {
        return Ok(None);
    };
    // Refine over the two intervals the peak station sits between.
    let low = index.saturating_sub(1);
    let high = (index + 1).min(stations.len() - 1);
    let (t0, t1) = (stations[low].0, stations[high].0);
    if t1 > t0 {
        for step in 1..REFINE_SAMPLES {
            let t = t0 + (t1 - t0) * step as f64 / REFINE_SAMPLES as f64;
            let radius = radius_at(t);
            let measured = probe(radius, span, t, stations[index].1, solve)
                .and_then(|probed| measure(radius, t, &probed, Difference::Central));
            if let Some(measured) = measured {
                if measured.factor > best.factor {
                    best = measured;
                }
            }
        }
    }
    Ok(Some(best))
}

/// The marker a fold that reaches a RAIL carries inside its refusal, beside
/// [`WALL_FOLDS`]: the band the closed chain's carve cuts is a lens strictly
/// between the rails, so a caller that would carve asks this first.
pub(crate) const RAIL_FOLDS: &str = "along the ball's track on it";

/// Does this refusal carry a fold that reaches the wall's rail?
///
/// The ONE fallback decision in the blending stack still read from text: a rail
/// fold and a lens fold are the same class (`UnsoundResult { Fold }`), minted at
/// the same stage by the same helper, so nothing but the marker tells them
/// apart. The marker is [`RAIL_FOLDS`], the constant [`check_wall_fold`]'s rail
/// verdict formats into its message, so an edit to the text moves the mint and
/// this check together.
pub(crate) fn is_rail_fold(refusal: &KernelRefusal) -> bool {
    is_wall_fold(refusal) && refusal.message.contains(RAIL_FOLDS)
}

/// The normal curvature of `surface` at `uv`, toward the ball `centre`, in the
/// direction `travel` — the carrier's own second fundamental form, so it is
/// exact on an analytic carrier and the fitted carrier's own on a fitted one.
///
/// `None` for a contact off the carrier's own extent: past an open boundary
/// `derivatives_extended` evaluates a ruled extension whose second derivatives
/// are the boundary anchor's, which says nothing about the carrier. The contact
/// is read at its parameter wrapped (closed) or clamped (open) onto the
/// domain, and kept when that point is the contact to `1e-9 * (1 + scale)` —
/// the bar `edge/support.rs` holds two solves of one station to, a hundred
/// times the station Newton's own `1e-11 * (1 + scale)` — so a junction
/// station the Newton left a rounding past the boundary is still measured.
fn rail_curvature(
    surface: &crate::NurbsSurface,
    uv: [f64; 2],
    contact: Vec3,
    centre: Vec3,
    travel: Vec3,
    scale: f64,
) -> Option<f64> {
    let (closed_u, closed_v) = surface.closed_directions().ok()?;
    let onto = |value: f64, [low, high]: [f64; 2], closed: bool| {
        if closed {
            low + (value - low).rem_euclid(high - low)
        } else {
            value.clamp(low, high)
        }
    };
    let u = onto(uv[0], surface.domain_u().ok()?, closed_u);
    let v = onto(uv[1], surface.domain_v().ok()?, closed_v);
    let derivatives = surface.derivatives(u, v, 2).ok()?;
    if derivatives[0][0].sub(contact).length() > 1.0e-9 * (1.0 + scale) {
        return None;
    }
    let (su, sv) = (derivatives[1][0], derivatives[0][1]);
    let inward = centre.sub(contact).normalized().ok()?;
    let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let determinant = e * g - f * f;
    if !(determinant > 0.0) {
        return None;
    }
    // `travel` in the carrier's parameters: its tangent-plane part, which is
    // all of it for a contact that stays on the carrier.
    let (along_u, along_v) = (su.dot(travel), sv.dot(travel));
    let du = (g * along_u - f * along_v) / determinant;
    let dv = (e * along_v - f * along_u) / determinant;
    let first = e * du * du + 2.0 * f * du * dv + g * dv * dv;
    if !(first > 0.0) {
        return None;
    }
    let second = derivatives[2][0].dot(inward) * du * du
        + 2.0 * derivatives[1][1].dot(inward) * du * dv
        + derivatives[0][2].dot(inward) * dv * dv;
    Some(second / first)
}

/// Is the ball the same size at every station this march would measure?
///
/// The bar is the kernel's `1e-11 * (1 + scale)` family, with the radius itself
/// as the scale: a law that is constant to that returns the same number at
/// every station, so this separates a genuine taper from arithmetic noise in a
/// constant law.
fn radius_is_constant(radius_at: &dyn Fn(f64) -> f64, stations: &[(f64, [f64; 4])]) -> bool {
    let mut radii = stations.iter().map(|(t, _)| radius_at(*t));
    let Some(first) = radii.next() else {
        return true;
    };
    let bar = 1.0e-11 * (1.0 + first.abs());
    radii.all(|radius| (radius - first).abs() <= bar)
}

/// Refuse a march whose wall would fold through itself.
///
/// # A fold that reaches a rail is proved at the first station it touches
///
/// For a constant-radius wall the contact on a mate is `P = c + ρ û`, so
/// `T · P' = 1 − ρ (k⃗ · û)`: the fold factor at a RAIL is one exactly where that
/// contact stops moving with the ball's centre. On the mate itself the same
/// contact satisfies `c' = (I − ρ W) P'` with `W` its shape operator toward the
/// centre, so `c' · P' = (1 − ρ κ_n) |P'|²`, with `κ_n` the mate's normal
/// curvature along the contact's own track. The two signs are the same sign:
/// the fold reaches the rail on a mate exactly where that mate turns tighter
/// than the ball along the track, `ρ κ_n > 1` — and a ball that touches a face
/// which turns tighter than itself pokes out through that face beside the
/// contact. Read off the carrier's own second fundamental form, that needs no
/// finite difference of the centre curve, so it is asked at every station
/// before the fold factor is, and the first station that proves it refuses by
/// naming the marched `edge` (in a chain, the segment the wall was crossing),
/// the face, its radius of curvature and the ball's.
///
/// The bar is the kernel's `1e-11 * (1 + scale)` family with the radius as the
/// scale — the one [`radius_is_constant`] uses — on the two RADII: a ball the
/// same size as the arc it rolls around (the sphere a corner of two equal
/// rounds is) is not this refusal's.
pub(super) fn check_wall_fold(
    radius_at: &dyn Fn(f64) -> f64,
    span: f64,
    stations: &[(f64, [f64; 4])],
    solve: SolveCentre<'_>,
    edge: &crate::topology::EdgeRecord,
    mates: [&crate::topology::FaceRecord; 2],
    scale: f64,
) -> Result<(), KernelRefusal> {
    // A TAPERED ball is measured by the section turn, not the centre curve's
    // circumcircle (module doc, "A tapered ball"). The circumcircle read 82 at
    // (8, 8, 10) on `fillet_edges_variable_tapers_a_box_edge` — the END station
    // of a straight edge, where the probe straddles the radius law's clamp and
    // the centre curve kinks — and until 2026-09-30 that reading was taken for
    // a property of tapers and the check abstained on every tapered span, so a
    // tapered wall that folded was built and shipped. The section turn on that
    // same edge is exactly zero at every station: its sections are circles in
    // parallel planes.
    let tapered = !radius_is_constant(radius_at, stations);
    if tapered {
        fold_trace(format_args!(
            "fold probe: the radius varies over these {} station(s) — measuring the section turn",
            stations.len()
        ));
    }
    let mut rail = |radius: f64, probed: &Probe| -> Result<(), KernelRefusal> {
        for side in 0..2 {
            let travel = probed.after.0[side].sub(probed.before.0[side]);
            let Ok(travel) = travel.normalized() else {
                continue;
            };
            let Some(curvature) = rail_curvature(
                &mates[side].surface,
                [probed.uv[2 * side], probed.uv[2 * side + 1]],
                probed.contacts[side],
                probed.centre,
                travel,
                scale,
            ) else {
                continue;
            };
            if !(curvature > 0.0) {
                continue;
            }
            let tighter = 1.0 / curvature;
            if radius - tighter <= 1.0e-11 * (1.0 + radius) {
                continue;
            }
            let face = mates[side];
            let contact = probed.contacts[side];
            let named = |name: &Option<String>| {
                name.as_deref()
                    .map(|name| format!(" `{name}`"))
                    .unwrap_or_default()
            };
            return Err(wall_fold_refusal(
                vec![face.id],
                format!(
                "{WALL_FOLDS} {radius} fits this edge: along edge {}{}, face {}{} turns tighter \
                 than the ball {RAIL_FOLDS} at ({:.6}, {:.6}, {:.6}) — radius of curvature \
                 {tighter:.6} against a blend radius of {radius} — so the contact there runs backward \
                 against the ball's centre and the fold reaches the wall's rail: the ball \
                 pokes out through that face, and the wall has no lens between its rails to \
                 carve",
                edge.id,
                named(&edge.name),
                face.id,
                named(&face.name),
                contact.x,
                contact.y,
                contact.z,
            ),
            ));
        }
        Ok(())
    };
    let Some(worst) = worst_fold(radius_at, span, stations, solve, tapered, &mut rail)? else {
        return Ok(());
    };
    if worst.factor <= 1.0 {
        return Ok(());
    }
    let radius = radius_at(worst.t);
    if tapered {
        let (low, high) = stations.iter().map(|(t, _)| radius_at(*t)).fold(
            (f64::INFINITY, f64::NEG_INFINITY),
            |(low, high), r| (low.min(r), high.max(r)),
        );
        return Err(wall_fold_refusal(
            vec![mates[0].id, mates[1].id],
            format!(
            "{WALL_FOLDS} {radius} fits this edge where the taper reaches it: the tapered wall's \
             sections turn tighter than the local ball at ({:.6}, {:.6}, {:.6}) — radius of turn \
             {:.6} against a blend radius of {radius} there, so the section sweeps back through \
             its neighbours (fold factor {:.6}; the radius runs from {low} to {high} along the edge)",
            worst.centre.x,
            worst.centre.y,
            worst.centre.z,
            worst.curvature_radius,
            worst.factor,
        ),
        ));
    }
    Err(wall_fold_refusal(
        vec![mates[0].id, mates[1].id],
        format!(
        "{WALL_FOLDS} {radius} fits this edge: the rolling ball's centre curve turns tighter \
         than the ball at ({:.6}, {:.6}, {:.6}) — radius of curvature {:.6} against a blend \
         radius of {radius}, so the envelope's section sweeps back through itself there \
         (fold factor {:.6})",
        worst.centre.x,
        worst.centre.y,
        worst.centre.z,
        worst.curvature_radius,
        worst.factor,
    ),
    ))
}
