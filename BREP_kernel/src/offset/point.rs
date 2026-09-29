//! The ONE definition of "the offset of a surface at a parameter".
//!
//! Push-face, offset-shell, thicken and the blend march are all surface
//! *offsetting* problems, and each of them had written the same three lines by
//! hand: evaluate the carrier, take a unit normal, step `distance` along it.
//! The audit records four independent copies; a fifth (`blend/edge/keep.rs`)
//! and two more in offset-shell turned up while this module was written. They
//! are collected here.
//!
//! # What this module is NOT
//!
//! It is **not** [`crate::offset_surface`]. That function is a Greville *fit*:
//! it samples the offset at the source basis's Greville parameters and
//! re-interpolates. Two independent reasons a fit is the wrong shared seam are
//! recorded in-tree — handing a fitted surface to the blend march's Newton loop
//! would inject the fit's approximation error into a residual whose bar is
//! `1e-11·(1+scale)` (`blending/blend/stations.rs`), and
//! `edit/direct_edit/face_offset_sphere.rs` records that `offset_surface` was
//! tried for the full-sphere push and *failed*, because the Greville fit
//! degenerates at the poles and the result stops re-recognising as a sphere.
//! The shared foundation is the **pointwise evaluator**; `offset_surface` is one
//! of its consumers (a fit of it), not the other way round.
//!
//! # The sign convention
//!
//! There is exactly ONE here: **`distance` is signed ALONG the returned
//! normal.** `point = source + distance · normal`. The kernel's other
//! convention — `offset_surface`'s "positive distance moves *opposite* the
//! face's outward normal" — is expressed by its adapter negating on the way in,
//! at one labelled place, instead of by four unlabelled hand negations
//! (audit §4.1).
//!
//! # The orientation conventions
//!
//! Orientation is an explicit parameter, never read off a `FaceRecord`, because
//! the callers genuinely disagree (audit §4.2) and both readings are
//! load-bearing. [`OffsetNormal`] has exactly the three that exist in the tree,
//! and each is defined to be *bit-identical* to the formula it replaces — see
//! its variant docs.

use crate::{AnalyticSurface, NurbsSurface, Vec3};

/// Which unit normal the offset rides, and how it is recovered where the
/// parametric normal degenerates.
///
/// Every variant names an existing in-tree formula and is bit-identical to it.
/// They differ ONLY where the parameter leaves the domain or the cross product
/// `S_u × S_v` collapses; a shared evaluator that silently picked one for
/// everybody would move behaviour at exactly the singular points the callers
/// each decided about on purpose.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OffsetNormal {
    /// The bare `S_u × S_v`, normalized, through the C¹ domain extension
    /// (`deriv1_extended`). Orientation is the *caller's* business — the blend
    /// march carries it in the sign of ρ instead (`signed_radii`), which is why
    /// this variant must not apply `same_sense`.
    ///
    /// Not [`OffsetNormal::FaceStable`]'s recovery — no nudge off a point and
    /// no walk along a collapsed row. Where the cross product vanishes this
    /// reads the ONE-SIDED LIMIT of the same cross product from inside the
    /// domain ([`raw_unit_normal`]), the surface twin of
    /// `NurbsCurve::unit_tangent`: an extruded face inherits a stationary
    /// parameter of its profile curve as an isoline where `S_u` vanishes,
    /// though the face is perfectly regular there. Only a carrier with no
    /// regular point in reach refuses, by name. The POINT past such a boundary
    /// moves along the same limit ([`raw_extended_point`]) where the ruled
    /// extension would stand still. Near such an isoline the vanishing partial
    /// is re-read from the control net's differences ([`well_rounded_partials`]),
    /// whose rounding shrinks with it, so the normal keeps its direction up to
    /// the isoline instead of turning with the basis derivatives' rounding.
    ///
    /// Bit-identical to `blending::blend::stations::raw_normal` (and to
    /// `evaluate_extended` for the point) wherever `S_u × S_v` is non-zero.
    Raw,
    /// `NurbsSurface::normal` — the in-domain `derivatives(u, v, 1)` cross —
    /// negated when `same_sense` is false. Clamped, not extended, and no
    /// singular recovery.
    ///
    /// Bit-identical to the `let mut normal = surface.normal(u, v)?; if
    /// !face.same_sense { normal = normal.scale(-1.0) }` block written out in
    /// `face_offset_revolution.rs`, `face_offset_freeform.rs`,
    /// `face_offset_torus.rs` and `offset_shell/smooth_sync.rs::face_normal`.
    Face { same_sense: bool },
    /// [`OffsetNormal::Face`] plus the singular-row recovery that
    /// `offset_surface` depends on: nudge off a point where the normal is
    /// undefined, and at a *collapsed parameter row* (a cone apex) walk deep
    /// inward for the true per-ruling limit rather than accept the axis
    /// direction the at-point cross degenerates to.
    ///
    /// Bit-identical to the former `offset::stable_face_normal`, whose body
    /// this is. The blend march must NOT inherit it — the recovery would
    /// change the residual at poles, so it stays opt-in (audit slice 1,
    /// "keep it as an opt-in flag").
    FaceStable { same_sense: bool },
    /// The exact closed-form normal of the *recognised analytic carrier*,
    /// oriented like [`OffsetNormal::Face`].
    ///
    /// This is the "exact per analytic surface type" lane: a sphere's normal is
    /// `(p − centre)/r` — defined at the poles, where `S_u × S_v` vanishes — a
    /// cylinder's is its radial direction, a cone's is the meridian
    /// perpendicular (defined even AT the apex, from the ruling's own azimuth),
    /// a torus's is `(p − tube centre)/r`. Where there is no exact form (the
    /// general `Revolution`, and every free-form patch) it falls back to
    /// [`OffsetNormal::Face`], so this variant is always at least as defined as
    /// that one.
    ///
    /// `NurbsSurface::analytic` memoizes per instance, so recognition is paid
    /// once per surface, not once per point.
    ///
    /// **No consumer rides this lane yet.** It exists as the exact half of the
    /// evaluator's contract and as the diagnostic's comparator; switching a
    /// caller onto it changes that caller's last digits and is a separate,
    /// measured step. See [`offset_normal_diagnostic`].
    ExactAnalytic { same_sense: bool },
}

impl OffsetNormal {
    /// The `same_sense`-carrying variants, for a caller that has a face.
    fn same_sense(self) -> bool {
        match self {
            OffsetNormal::Raw => true,
            OffsetNormal::Face { same_sense }
            | OffsetNormal::FaceStable { same_sense }
            | OffsetNormal::ExactAnalytic { same_sense } => same_sense,
        }
    }
}

/// One evaluation of the offset: the source point, the unit normal the offset
/// rides, and the offset point itself.
///
/// The normal is also the *offset surface's own* normal wherever the offset is
/// regular — it flips only past the evolute, which is what the region-scoped
/// scan in `offset/regularity.rs` (the kernel's only curvature gate, shared by
/// `thicken`'s sheet refusal and offset-shell's carrier-collapse lane) exists
/// to refuse, over the trim rather than over the whole carrier. This struct
/// deliberately carries no other derivative data: the
/// five consumers were audited and not one of them uses `S_u`/`S_v` for
/// anything but the normal, and the blend march's Jacobian is finite-differenced
/// on the residual rather than assembled from partials.
#[derive(Clone, Copy, Debug)]
pub struct OffsetSample {
    /// `S(u, v)` on the source carrier.
    pub source: Vec3,
    /// The unit normal, in the requested orientation.
    pub normal: Vec3,
    /// `source + distance · normal`.
    pub point: Vec3,
}

/// The pointwise offset evaluator, bound to one carrier and one orientation
/// convention.
///
/// Construction is free — it stores three references and reads nothing — so a
/// caller inside a Newton loop may build one per call without paying anything.
/// `site` is a short stable label used only by [`offset_normal_diagnostic`].
#[derive(Clone, Copy)]
pub struct OffsetEvaluator<'a> {
    site: &'static str,
    surface: &'a NurbsSurface,
    convention: OffsetNormal,
}

impl<'a> OffsetEvaluator<'a> {
    pub fn new(site: &'static str, surface: &'a NurbsSurface, convention: OffsetNormal) -> Self {
        Self {
            site,
            surface,
            convention,
        }
    }

    /// The unit normal alone, for callers that only need the direction (a
    /// residual check, or an affine offset that shifts the whole control net by
    /// one vector). Evaluates the point only on the lanes that need it.
    pub fn normal(&self, u: f64, v: f64) -> Result<Vec3, String> {
        let result = self.normal_with(self.convention, u, v);
        offset_normal_diagnostic(self.site, self.surface, self.convention, u, v, &result);
        result
    }

    /// Whether the carrier evaluated at `(u, v)` is the REAL surface, or its
    /// fiction — the question [`Self::at`] cannot answer for itself.
    ///
    /// `OffsetNormal::Raw` evaluates through `deriv1_extended`, which
    /// continues an OPEN direction along its boundary tangent plane without
    /// bound instead of refusing. A solve with no solution on the real
    /// carrier therefore converges on that continuation and reports a
    /// residual of zero, because the extended point IS a genuine root of the
    /// extended system. Callers must ask here, at convergence.
    ///
    /// Being outside the domain is NOT by itself the fault, and testing for
    /// that alone over-refuses badly. Three cases:
    ///
    /// * a CLOSED direction wraps (`rem_euclid`) back onto the real surface,
    ///   so it is always faithful — which is why a full cylinder cannot
    ///   exhibit the problem and a trimmed arc can;
    /// * an open direction along which the surface is RULED extends exactly.
    ///   A plane is a bounded bilinear patch in this kernel, and its
    ///   continuation is still that plane; a cylinder's axial direction
    ///   continues as the same cylinder. A re-entrant corner touching a
    ///   wall's carrier beyond its patch is real geometry and must keep
    ///   working;
    /// * an open direction with non-zero NORMAL CURVATURE is where the ruled
    ///   continuation leaves the surface, and that is the fiction.
    ///
    /// So the test is geometric, not a domain comparison and not keyed on
    /// [`NurbsSurface::analytic`] — an analytic-keyed guard silently no-ops on
    /// every fitted patch. The ruled extension drops the second-order term, so
    /// over an excursion `d` it departs the true surface by about
    /// `½·d²·|S_ii·n|`, where `S_ii·n` is the second-fundamental-form
    /// coefficient along the exceeded direction (Golovanov §4.12's `b11`/`b22`,
    /// the same quantities the marcher's step budget uses). Faithful when that
    /// departure is within `tolerance`.
    ///
    /// The bound is QUADRATIC in the excursion, so a parameter a rounding bit
    /// past the boundary is faithful by many orders and a real excursion onto
    /// a curved carrier is unfaithful by many orders. The exact threshold
    /// barely matters; passing the caller's own convergence bar keeps the
    /// answer honest to the accuracy it claims.
    pub fn on_real_carrier(&self, u: f64, v: f64, tolerance: f64) -> Result<bool, String> {
        let [u0, u1] = self.surface.domain_u()?;
        let [v0, v1] = self.surface.domain_v()?;
        let (closed_u, closed_v) = self.surface.closed_directions()?;
        let out = |value: f64, lo: f64, hi: f64, closed: bool| -> f64 {
            if closed {
                0.0
            } else if value < lo {
                value - lo
            } else if value > hi {
                value - hi
            } else {
                0.0
            }
        };
        let du_out = out(u, u0, u1, closed_u);
        let dv_out = out(v, v0, v1, closed_v);
        if du_out == 0.0 && dv_out == 0.0 {
            return Ok(true);
        }
        let derivatives = self.surface.derivatives(u - du_out, v - dv_out, 2)?;
        let normal = match derivatives[1][0].cross(derivatives[0][1]).normalized() {
            Ok(normal) => normal,
            // A degenerate anchor: see `degenerate_anchor_on_real_carrier`.
            Err(_) => {
                return self.degenerate_anchor_on_real_carrier(u, v, du_out, dv_out, tolerance)
            }
        };
        let departure = |step: f64, second: Vec3| 0.5 * step * step * second.dot(normal).abs();
        if du_out != 0.0 && departure(du_out, derivatives[2][0]) > tolerance {
            return Ok(false);
        }
        if dv_out != 0.0 && departure(dv_out, derivatives[0][2]) > tolerance {
            return Ok(false);
        }
        Ok(true)
    }

    /// [`Self::on_real_carrier`] where `S_u × S_v` vanishes at the anchor.
    ///
    /// Where the anchor lies on a STATIONARY boundary isoline
    /// ([`stationary_boundary`]) that the point has left across, the carrier is
    /// regular and has a normal (the `Raw` lane's one-sided limit), and the
    /// continuation past it is [`raw_extended_point`]'s: straight along the
    /// isoline's limit direction, `speed·|excursion|` long. The second-order
    /// departure `½·d²·|S_ii·n|` cannot measure it, because `S_ii` is the
    /// parameterization's, not the carrier's. So the departure is measured on
    /// the carrier itself: the point the same distance back INSIDE the patch,
    /// through the [`StationaryChart`], sits off the boundary's tangent plane by
    /// what a regular continuation would depart from the straight one, to
    /// leading order (the mirror image of `½·κ_n·s²`). Faithful when that is
    /// within `tolerance`. An excursion longer than the patch mirrors to its far
    /// boundary and scales that departure by the square of the ratio.
    ///
    /// Every other degenerate anchor — a pole, an apex, a collapsed row — has no
    /// normal to measure against, and the faithfulness of its extension cannot
    /// be established, so it answers `false`, as it always has. A surface that
    /// does not leave its boundary to any order refuses by name
    /// ([`Self::stationary_chart`]).
    fn degenerate_anchor_on_real_carrier(
        &self,
        u: f64,
        v: f64,
        du_out: f64,
        dv_out: f64,
        tolerance: f64,
    ) -> Result<bool, String> {
        let (anchor_u, anchor_v) = (u - du_out, v - dv_out);
        let charts = [
            (du_out != 0.0)
                .then(|| self.stationary_chart(true, anchor_u, anchor_v))
                .transpose()?
                .flatten(),
            (dv_out != 0.0)
                .then(|| self.stationary_chart(false, anchor_u, anchor_v))
                .transpose()?
                .flatten(),
        ];
        if charts.iter().all(Option::is_none) {
            return Ok(false);
        }
        let (anchor_point, su, sv) = self.surface.deriv1(anchor_u, anchor_v)?;
        let Ok(normal) = raw_unit_normal(self.surface, anchor_u, anchor_v, su, sv) else {
            return Ok(false);
        };
        let derivatives = self.surface.derivatives(anchor_u, anchor_v, 2)?;
        let ([u0, u1], [v0, v1]) = domains(self.surface)?;
        for (along_u, out, chart) in [(true, du_out, charts[0]), (false, dv_out, charts[1])] {
            if out == 0.0 {
                continue;
            }
            let departure = match chart {
                Some(chart) => {
                    let excursion = chart.regular(if along_u { u } else { v });
                    let (lo, hi) = if along_u { (u0, u1) } else { (v0, v1) };
                    let far = if chart.outward > 0.0 { lo } else { hi };
                    // The patch mirrors at most back to its far boundary; a longer
                    // excursion scales that departure by the square of the ratio,
                    // the order a regular continuation departs at.
                    let reach = -chart.regular(far);
                    let (inside, growth) = if excursion <= reach {
                        (chart.parameter(-excursion), 1.0)
                    } else {
                        (far, (excursion / reach).powi(2))
                    };
                    let mirrored = if along_u {
                        self.surface.evaluate(inside, anchor_v)?
                    } else {
                        self.surface.evaluate(anchor_u, inside)?
                    };
                    growth * mirrored.sub(anchor_point).dot(normal).abs()
                }
                None => {
                    let second = if along_u { derivatives[2][0] } else { derivatives[0][2] };
                    0.5 * out * out * second.dot(normal).abs()
                }
            };
            if departure > tolerance {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// The offset of this carrier at `(u, v)` by `distance` **along the
    /// normal** (see the module's sign convention).
    pub fn at(&self, u: f64, v: f64, distance: f64) -> Result<OffsetSample, String> {
        let (source, normal) = self.evaluate(u, v)?;
        Ok(OffsetSample {
            source,
            normal,
            point: source.add(normal.scale(distance)),
        })
    }

    fn evaluate(&self, u: f64, v: f64) -> Result<(Vec3, Vec3), String> {
        let result = match self.convention {
            // ONE evaluation for the point and the partials the normal needs —
            // `deriv1_extended`'s point is bit-identical to `evaluate_extended`,
            // which is what licenses `blend/edge/keep.rs` to drop its second
            // evaluation of the same surface.
            OffsetNormal::Raw => self
                .surface
                .deriv1_extended(u, v)
                .and_then(|(point, su, sv)| {
                    let normal = raw_unit_normal(self.surface, u, v, su, sv)?;
                    Ok((raw_extended_point(self.surface, u, v, point, su, sv)?, normal))
                }),
            convention => self
                .surface
                .evaluate(u, v)
                .and_then(|point| Ok((point, self.normal_with(convention, u, v)?))),
        };
        let normal = result
            .as_ref()
            .map(|(_, normal)| *normal)
            .map_err(Clone::clone);
        offset_normal_diagnostic(self.site, self.surface, self.convention, u, v, &normal);
        result
    }

    fn normal_with(&self, convention: OffsetNormal, u: f64, v: f64) -> Result<Vec3, String> {
        match convention {
            OffsetNormal::Raw => {
                let (_, su, sv) = self.surface.deriv1_extended(u, v)?;
                raw_unit_normal(self.surface, u, v, su, sv)
            }
            OffsetNormal::Face { same_sense } => {
                Ok(orient(self.surface.normal(u, v)?, same_sense))
            }
            OffsetNormal::FaceStable { same_sense } => {
                stable_normal(self.surface, same_sense, u, v)
            }
            OffsetNormal::ExactAnalytic { same_sense } => {
                let point = self.surface.evaluate(u, v)?;
                match exact_analytic_normal(self.surface, u, v, point)? {
                    Some(normal) => Ok(orient(normal, same_sense)),
                    None => Ok(orient(self.surface.normal(u, v)?, same_sense)),
                }
            }
        }
    }
}

fn orient(normal: Vec3, same_sense: bool) -> Vec3 {
    if same_sense {
        normal
    } else {
        normal.scale(-1.0)
    }
}

fn domains(surface: &NurbsSurface) -> Result<([f64; 2], [f64; 2]), String> {
    Ok((surface.domain_u()?, surface.domain_v()?))
}

/// The `Raw` lane's unit normal from the partials `deriv1_extended` returned at
/// `(u, v)`: `S_u × S_v` normalized — the same vector, to the bit, wherever it
/// is non-zero — or its one-sided limit where it vanishes.
///
/// Why a limit and not a refusal: a surface can be stationary in its
/// PARAMETERIZATION where its geometry is regular. The standard source is an
/// extrusion (or any ruled face) of a profile curve whose first derivative
/// vanishes at a vertex — the involute flank's Hermite fit, a cubic with two
/// coincident poles — which carries that parameter as a whole isoline where
/// `S_u = 0`. The blend march evaluates its support's normal on exactly that
/// isoline when it rolls along the rim edge, and the face's normal is still the
/// limit from inside, as a curve's direction of travel is at its cusp
/// (`NurbsCurve::unit_tangent`, whose ladder this mirrors). The limit is also
/// the side the march's orientation came from: `signed_radii` reads the sign of
/// ρ at a regular interior point, and a one-sided probe from inside the domain
/// keeps that sense, where a probe past a stationary boundary would not exist.
///
/// The probe origin is the in-domain parameter `deriv1_extended` anchors on —
/// wrapped in a closed direction, clamped past an open boundary — and the
/// ladder walks first along the parameter whose partial collapsed, then along
/// the other (a pole row collapses `S_u` along the whole row, so only a step in
/// `v` leaves it). Only a carrier with no regular point within a tenth of its
/// domain refuses, and the refusal names where.
///
/// `BREP_CUSP_TANGENT_RESCUE=0` restores the bare `normalized()` refusal, the
/// same hatch the curve tangent carries.
fn raw_unit_normal(
    surface: &NurbsSurface,
    u: f64,
    v: f64,
    su: Vec3,
    sv: Vec3,
) -> Result<Vec3, String> {
    let (su, sv) = well_rounded_partials(surface, u, v, su, sv)?;
    let error = match su.cross(sv).normalized() {
        Ok(normal) => return Ok(normal),
        Err(error) => error,
    };
    if std::env::var("BREP_CUSP_TANGENT_RESCUE").as_deref() == Ok("0") {
        return Err(error);
    }
    let ([u0, u1], [v0, v1]) = domains(surface)?;
    let (closed_u, closed_v) = surface.closed_directions()?;
    let anchor = |value: f64, lo: f64, hi: f64, closed: bool| -> f64 {
        if closed && (value < lo || value > hi) {
            lo + (value - lo).rem_euclid(hi - lo)
        } else {
            value.clamp(lo, hi)
        }
    };
    let (anchor_u, anchor_v) = (anchor(u, u0, u1, closed_u), anchor(v, v0, v1, closed_v));
    let along_u_first = su.length() <= sv.length();
    let directions = if along_u_first {
        [(1.0, 0.0), (0.0, 1.0)]
    } else {
        [(0.0, 1.0), (1.0, 0.0)]
    };
    for scale in [1e-6, 1e-4, 1e-2, 1e-1] {
        for (along_u, along_v) in directions {
            let step_u = (u1 - u0) * scale * along_u;
            let step_v = (v1 - v0) * scale * along_v;
            for sign in [1.0, -1.0] {
                let probe_u = anchor_u + sign * step_u;
                let probe_v = anchor_v + sign * step_v;
                if probe_u < u0 || probe_u > u1 || probe_v < v0 || probe_v > v1 {
                    continue;
                }
                if let Some(normal) = surface
                    .deriv1_extended(probe_u, probe_v)
                    .and_then(|(_, su, sv)| {
                        well_rounded_partials(surface, probe_u, probe_v, su, sv)
                    })
                    .ok()
                    .and_then(|(su, sv)| su.cross(sv).normalized().ok())
                {
                    return Ok(normal);
                }
            }
        }
    }
    Err(format!(
        "offset normal: S_u x S_v vanishes at (u, v) = ({u}, {v}) and at every probe the \
         one-sided limit can reach in [{u0}, {u1}] x [{v0}, {v1}], so this carrier has no \
         normal to report there"
    ))
}

/// The `Raw` lane's POINT past an open boundary: `deriv1_extended`'s point —
/// the same point, to the bit — unless the partial ACROSS that boundary
/// vanishes at the anchor, where the ruled extension `S(anchor) + d·S_u` does
/// not move at all.
///
/// That is the stationary isoline [`raw_unit_normal`] reads its limit on: an
/// extrusion of a profile whose cubic has coincident end poles is regular up
/// to and across the boundary, and only its parameterization stops there. A
/// solve that has to leave the domain across such a boundary — the blend
/// march's overshoot stations, which seat the rolling ball past a vertex —
/// then meets a Jacobian column that is exactly zero ("station Newton is
/// singular"), though the continuation of the face exists and is the one its
/// cusp-free twin extends onto. So the extension moves along the one-sided
/// limit of the vanished partial's DIRECTION, probed from inside the domain on
/// the ladder `raw_unit_normal` uses, at the isoline's mean speed across its
/// parameter range, `|S(hi, ·) − S(lo, ·)| / (hi − lo)`.
///
/// The speed is a choice the geometry does not make — the continuation's
/// locus is fixed by the direction alone — and this one is intrinsic to the
/// carrier: on a ruled face over a straight profile it is the speed the
/// cusp-free twin's own partial has, so the solved parameter past the boundary
/// is the twin's.
///
/// Refuses by name only where the surface itself gives no direction: the
/// vanished partial is zero at every probe the ladder reaches, a patch
/// collapsed across a tenth of its domain, where `raw_unit_normal` refuses
/// too. `BREP_CUSP_TANGENT_RESCUE=0` restores the stationary extension.
fn raw_extended_point(
    surface: &NurbsSurface,
    u: f64,
    v: f64,
    point: Vec3,
    su: Vec3,
    sv: Vec3,
) -> Result<Vec3, String> {
    let ([u0, u1], [v0, v1]) = domains(surface)?;
    if u >= u0 && u <= u1 && v >= v0 && v <= v1 {
        return Ok(point);
    }
    if su.normalized().is_ok() && sv.normalized().is_ok() {
        return Ok(point);
    }
    if std::env::var("BREP_CUSP_TANGENT_RESCUE").as_deref() == Ok("0") {
        return Ok(point);
    }
    let (closed_u, closed_v) = surface.closed_directions()?;
    let excursion = |value: f64, lo: f64, hi: f64, closed: bool| -> f64 {
        if closed {
            0.0
        } else if value < lo {
            value - lo
        } else if value > hi {
            value - hi
        } else {
            0.0
        }
    };
    let du_out = excursion(u, u0, u1, closed_u);
    let dv_out = excursion(v, v0, v1, closed_v);
    let wrap = |value: f64, lo: f64, hi: f64, closed: bool| -> f64 {
        if closed && (value < lo || value > hi) {
            lo + (value - lo).rem_euclid(hi - lo)
        } else {
            value.clamp(lo, hi)
        }
    };
    let (anchor_u, anchor_v) = (wrap(u, u0, u1, closed_u), wrap(v, v0, v1, closed_v));
    let mut point = point;
    for (along_u, out, partial) in [(true, du_out, su), (false, dv_out, sv)] {
        if out == 0.0 || partial.normalized().is_ok() {
            continue;
        }
        let (lo, hi) = if along_u { (u0, u1) } else { (v0, v1) };
        let inward = -out.signum();
        let direction = [1e-6, 1e-4, 1e-2, 1e-1].iter().find_map(|scale| {
            let step = inward * (hi - lo) * scale;
            let (probe_u, probe_v) = if along_u {
                (anchor_u + step, anchor_v)
            } else {
                (anchor_u, anchor_v + step)
            };
            if probe_u < u0 || probe_u > u1 || probe_v < v0 || probe_v > v1 {
                return None;
            }
            let (_, probe_su, probe_sv) = surface.deriv1(probe_u, probe_v).ok()?;
            let (probe_su, probe_sv) =
                well_rounded_partials(surface, probe_u, probe_v, probe_su, probe_sv).ok()?;
            (if along_u { probe_su } else { probe_sv }).normalized().ok()
        });
        let Some(direction) = direction else {
            return Err(format!(
                "offset point: S_{} vanishes on the boundary of [{u0}, {u1}] x [{v0}, {v1}] at \
                 (u, v) = ({anchor_u}, {anchor_v}) and at every probe the one-sided limit can \
                 reach, so the surface has no direction to extend along toward ({u}, {v})",
                if along_u { "u" } else { "v" }
            ));
        };
        let speed = isoline_mean_speed(surface, along_u, anchor_u, anchor_v)?;
        point = point.add(direction.scale(out * speed));
    }
    Ok(point)
}

/// The mean speed of the isoline through `(anchor_u, anchor_v)` along `along_u`
/// across its whole parameter range, `|S(hi, ·) − S(lo, ·)| / (hi − lo)`: the
/// speed [`raw_extended_point`] moves a point past a stationary boundary at, and
/// the one [`StationaryChart`] joins it with.
fn isoline_mean_speed(
    surface: &NurbsSurface,
    along_u: bool,
    anchor_u: f64,
    anchor_v: f64,
) -> Result<f64, String> {
    let ([u0, u1], [v0, v1]) = domains(surface)?;
    let (lo, hi) = if along_u { (u0, u1) } else { (v0, v1) };
    let (start, end) = if along_u {
        (surface.evaluate(lo, anchor_v)?, surface.evaluate(hi, anchor_v)?)
    } else {
        (surface.evaluate(anchor_u, lo)?, surface.evaluate(anchor_u, hi)?)
    };
    Ok(end.sub(start).length() / (hi - lo))
}

/// The in-domain anchor `deriv1_extended` reads for `(u, v)`: wrapped in a
/// closed direction, clamped past an open boundary.
fn extension_anchor(surface: &NurbsSurface, u: f64, v: f64) -> Result<(f64, f64), String> {
    let ([u0, u1], [v0, v1]) = domains(surface)?;
    let (closed_u, closed_v) = surface.closed_directions()?;
    let anchor = |value: f64, lo: f64, hi: f64, closed: bool| -> f64 {
        if closed && (value < lo || value > hi) {
            lo + (value - lo).rem_euclid(hi - lo)
        } else {
            value.clamp(lo, hi)
        }
    };
    Ok((anchor(u, u0, u1, closed_u), anchor(v, v0, v1, closed_v)))
}

/// The open boundary nearest `(u, v)` in the direction `along_u`, when the
/// partial ACROSS it vanishes there while the partial ALONG it does not: an
/// isoline on which only the parameterization is stationary, the extrusion of a
/// profile cubic with coincident end poles. `None` in a closed direction, past
/// a regular boundary, and on a collapsed row (a pole or an apex, where the
/// partial along the boundary vanishes too, so the boundary is a point and not
/// an isoline).
///
/// The zero test is `Vec3::normalized`'s, the one [`raw_unit_normal`] and
/// [`raw_extended_point`] already apply.
fn stationary_boundary(
    surface: &NurbsSurface,
    along_u: bool,
    u: f64,
    v: f64,
) -> Result<Option<f64>, String> {
    let ([u0, u1], [v0, v1]) = domains(surface)?;
    let (closed_u, closed_v) = surface.closed_directions()?;
    if (along_u && closed_u) || (!along_u && closed_v) {
        return Ok(None);
    }
    let (anchor_u, anchor_v) = extension_anchor(surface, u, v)?;
    let (value, lo, hi) = if along_u { (u, u0, u1) } else { (v, v0, v1) };
    let boundary = if (value - lo).abs() <= (hi - value).abs() { lo } else { hi };
    let (at_u, at_v) = if along_u { (boundary, anchor_v) } else { (anchor_u, boundary) };
    let (_, su, sv) = surface.deriv1(at_u, at_v)?;
    let (across, along) = if along_u { (su, sv) } else { (sv, su) };
    Ok((across.normalized().is_err() && along.normalized().is_ok()).then_some(boundary))
}

/// Below this fraction of the other partial's reach over its domain, a partial
/// is small enough that its rounding may be its direction: only then is its
/// boundary asked whether it is stationary. A gate on cost, not on the answer —
/// the answer is [`stationary_boundary`]'s — and the bare partial's tilt at the
/// gate is about ulp·|S|/|S_u| ≈ 1e-12 of a unit model, so switching there moves
/// a normal by no more than that.
const STATIONARY_PARTIAL_REACH: f64 = 1e-3;

/// The partials `(S_u, S_v)` the `Raw` lane builds its normal from: the ones
/// `deriv1_extended` returned — the same vectors, to the bit — unless one of
/// them is small against the other and belongs to a [`stationary_boundary`],
/// where it is re-read by [`NurbsSurface::partial_by_differences`] at the
/// in-domain anchor.
///
/// Why: near such an isoline the partial vanishes like the distance to it, and
/// the basis-derivative evaluation's rounding does not, so the normal
/// `S_u × S_v` turns by `ulp·|S|/|S_u|`. The tombstone fixtures measured 3.5e-11
/// at `|S_u| = 7.7e-5` and 3.7e-5 at `|S_u| = 1.7e-10`, and the one-sided limit's
/// own probe (a millionth of the span in) 2.2e-10: all above the blend march's
/// residual bar of `1e-11·(1 + scale)`, and inconsistent with each other across
/// the isoline, so the station Newton at a stationary vertex could not settle.
/// The difference form's rounding scales with the partial, so the direction
/// survives to the isoline itself.
///
/// `BREP_CUSP_TANGENT_RESCUE=0` keeps the bare partials.
fn well_rounded_partials(
    surface: &NurbsSurface,
    u: f64,
    v: f64,
    su: Vec3,
    sv: Vec3,
) -> Result<(Vec3, Vec3), String> {
    let ([u0, u1], [v0, v1]) = domains(surface)?;
    let reach_u = su.length_squared() * (u1 - u0) * (u1 - u0);
    let reach_v = sv.length_squared() * (v1 - v0) * (v1 - v0);
    let gate = STATIONARY_PARTIAL_REACH * STATIONARY_PARTIAL_REACH;
    let small_u = reach_u <= gate * reach_v;
    let small_v = reach_v <= gate * reach_u;
    if !small_u && !small_v {
        return Ok((su, sv));
    }
    if std::env::var("BREP_CUSP_TANGENT_RESCUE").as_deref() == Ok("0") {
        return Ok((su, sv));
    }
    let (anchor_u, anchor_v) = extension_anchor(surface, u, v)?;
    let mut partials = (su, sv);
    if small_u && stationary_boundary(surface, true, u, v)?.is_some() {
        if let Ok(partial) = surface.partial_by_differences(true, anchor_u, anchor_v) {
            partials.0 = partial;
        }
    }
    if small_v && stationary_boundary(surface, false, u, v)?.is_some() {
        if let Ok(partial) = surface.partial_by_differences(false, anchor_u, anchor_v) {
            partials.1 = partial;
        }
    }
    Ok(partials)
}

/// A REGULAR parameter for one direction of a carrier near an open boundary on
/// which that direction's partial vanishes ([`stationary_boundary`]).
///
/// There the carrier is regular and only its parameterization stops: the point
/// leaves the boundary like `L·d^m` in the outward parameter distance `d`
/// (`m` the order of the first non-vanishing partial across the boundary, 2 for
/// a cubic with two coincident end poles). A Newton that solves for the
/// parameter itself therefore meets a root of multiplicity `m` wherever its
/// answer lies ON the isoline — the blend march's station at a stationary
/// vertex — and converges linearly on a Jacobian column that vanishes with
/// the step.
///
/// The chart's value `w` is
///
/// * `−L·|d|^m` inside the domain (`d ≤ 0`), which is the distance from the
///   boundary along the carrier to leading order, so the point moves at unit
///   speed in `w` right up to the isoline;
/// * `speed·d` past it, where [`raw_extended_point`] moves the point along the
///   one-sided limit at the isoline's mean speed — so in `w` the continuation
///   also moves at unit speed, and the chart joins it C¹ at `w = 0`.
///
/// On a straight isoline the point is affine in `w` on both sides to the order
/// of the carrier's next term; the Newton's column in `w` no longer vanishes.
#[derive(Clone, Copy, Debug)]
pub(crate) struct StationaryChart {
    boundary: f64,
    /// +1 when the boundary is the high end of the domain, −1 the low end.
    outward: f64,
    order: i32,
    /// `L = |∂^m S| / m!` across the boundary.
    coefficient: f64,
    speed: f64,
}

impl StationaryChart {
    /// The chart value of the carrier parameter `parameter`.
    pub(crate) fn regular(&self, parameter: f64) -> f64 {
        let outward = self.outward * (parameter - self.boundary);
        if outward >= 0.0 {
            self.speed * outward
        } else {
            -self.coefficient * (-outward).powi(self.order)
        }
    }

    /// The carrier parameter of the chart value `regular`.
    pub(crate) fn parameter(&self, regular: f64) -> f64 {
        let outward = if regular >= 0.0 {
            regular / self.speed
        } else if self.order == 2 {
            -(-regular / self.coefficient).sqrt()
        } else {
            -(-regular / self.coefficient).powf(1.0 / f64::from(self.order))
        };
        self.boundary + self.outward * outward
    }
}

impl OffsetEvaluator<'_> {
    /// The [`StationaryChart`] for the direction `along_u` at `(u, v)`, when that
    /// direction's nearest open boundary is a stationary isoline; `None`
    /// otherwise, and on every lane but `Raw`, whose point past the boundary is
    /// what the chart joins.
    ///
    /// Refuses by name only where the surface itself is singular: the partial
    /// across the boundary vanishes to every order the carrier's degree
    /// reaches, so the patch does not leave the boundary at all.
    /// `BREP_CUSP_TANGENT_RESCUE=0` answers `None`.
    pub(crate) fn stationary_chart(
        &self,
        along_u: bool,
        u: f64,
        v: f64,
    ) -> Result<Option<StationaryChart>, String> {
        if self.convention != OffsetNormal::Raw
            || std::env::var("BREP_CUSP_TANGENT_RESCUE").as_deref() == Ok("0")
        {
            return Ok(None);
        }
        let surface = self.surface;
        let Some(boundary) = stationary_boundary(surface, along_u, u, v)? else {
            return Ok(None);
        };
        let ([u0, u1], [v0, v1]) = domains(surface)?;
        let hi = if along_u { u1 } else { v1 };
        let (anchor_u, anchor_v) = extension_anchor(surface, u, v)?;
        let (at_u, at_v) = if along_u { (boundary, anchor_v) } else { (anchor_u, boundary) };
        let degree = if along_u { surface.degree_u } else { surface.degree_v };
        let derivatives = surface.derivatives(at_u, at_v, degree)?;
        let mut factorial = 1.0;
        let mut found = None;
        for order in 2..=degree {
            factorial *= order as f64;
            let partial = if along_u {
                derivatives[order][0]
            } else {
                derivatives[0][order]
            };
            if partial.normalized().is_ok() {
                found = Some((order, partial.length() / factorial));
                break;
            }
        }
        let Some((order, coefficient)) = found else {
            return Err(format!(
                "offset chart: S_{} vanishes to order {degree} across the boundary of \
                 [{u0}, {u1}] x [{v0}, {v1}] at (u, v) = ({at_u}, {at_v}), so the surface does \
                 not leave that boundary and has no regular parameter there",
                if along_u { "u" } else { "v" }
            ));
        };
        let speed = isoline_mean_speed(surface, along_u, anchor_u, anchor_v)?;
        if !(speed > 0.0) {
            return Ok(None);
        }
        Ok(Some(StationaryChart {
            boundary,
            outward: if boundary == hi { 1.0 } else { -1.0 },
            order: order as i32,
            coefficient,
            speed,
        }))
    }
}

/// The body of the former `offset::stable_face_normal`, verbatim, with the
/// `FaceRecord` replaced by the `same_sense` flag it read (audit §4.5: the
/// `FaceRecord` requirement is a type mismatch, not a geometric one — it is why
/// `thicken` fabricates a synthetic face with empty loops just to offset a bare
/// surface).
fn stable_normal(
    surface: &NurbsSurface,
    same_sense: bool,
    u: f64,
    v: f64,
) -> Result<Vec3, String> {
    let normal_at = |u, v| surface.normal(u, v).ok();
    let mut normal = normal_at(u, v);
    let ([u0, u1], [v0, v1]) = domains(surface)?;
    if normal.is_none() {
        let du = (u1 - u0) * 1e-5;
        let dv = (v1 - v0) * 1e-5;
        for (candidate_u, candidate_v) in [
            ((u + du).clamp(u0, u1), v),
            ((u - du).clamp(u0, u1), v),
            (u, (v + dv).clamp(v0, v1)),
            (u, (v - dv).clamp(v0, v1)),
        ] {
            normal = normal_at(candidate_u, candidate_v);
            if normal.is_some() {
                break;
            }
        }
    }
    // SINGULAR-ROW override: at a surface singularity where a whole
    // parameter row collapses to one point (a cone apex), the at-point /
    // nudged normal is the cross of a vanishing partial with noise — the
    // AXIS direction instead of the ruling normal, which offsets the apex
    // row straight down the axis and bends the fitted surface by exactly
    // d·cos(half-angle). Detect the collapse by local point spread, walk
    // DEEP inward for the true per-ruling limit, and replace the at-point
    // value only when the two genuinely DISAGREE. A sphere/dome pole also
    // reads as collapsed, but there the at-point normal (the axis) IS the
    // limit — agreement keeps the exact baseline value.
    //
    // (`OffsetNormal::ExactAnalytic` answers this case in closed form for a
    // recognised cone — the ruling's own azimuth, not a deep-inward probe —
    // but no consumer rides that lane yet, so this stays the `offset_surface`
    // behaviour it always was.)
    let singular_here = {
        let du = (u1 - u0) * 1e-4;
        let dv = (v1 - v0) * 1e-4;
        let here = surface.evaluate(u, v)?;
        let along_u = surface
            .evaluate((u + du).clamp(u0, u1), v)?
            .sub(here)
            .length()
            .max(
                surface
                    .evaluate((u - du).clamp(u0, u1), v)?
                    .sub(here)
                    .length(),
            );
        let along_v = surface
            .evaluate(u, (v + dv).clamp(v0, v1))?
            .sub(here)
            .length()
            .max(
                surface
                    .evaluate(u, (v - dv).clamp(v0, v1))?
                    .sub(here)
                    .length(),
            );
        let scale = along_u.max(along_v);
        scale > 0.0 && along_u.min(along_v) < scale * 1e-6
    };
    if singular_here {
        let v_mid = (v0 + v1) * 0.5;
        let u_mid = (u0 + u1) * 0.5;
        let mut interior = None;
        for fraction in [1e-3, 1e-2, 5e-2, 0.25] {
            let candidate_v = v + (v_mid - v) * fraction;
            let candidate_u = u + (u_mid - u) * fraction;
            for (cu, cv) in [(u, candidate_v), (candidate_u, v), (candidate_u, candidate_v)] {
                if let Some(candidate) = normal_at(cu, cv) {
                    interior = Some(candidate);
                    break;
                }
            }
            if interior.is_some() {
                break;
            }
        }
        normal = match (normal, interior) {
            (Some(at_point), Some(interior)) if at_point.dot(interior) > 1.0 - 1e-6 => {
                Some(at_point)
            }
            (_, Some(interior)) => Some(interior),
            (at_point, None) => at_point,
        };
    }
    let normal =
        normal.ok_or_else(|| "offset_surface: cannot determine surface normal".to_string())?;
    Ok(orient(normal, same_sense))
}

// ---------------------------------------------------------------------------
// The exact analytic lane
// ---------------------------------------------------------------------------

/// The exact unit normal of a recognised analytic carrier at `point`, in the
/// carrier's own canonical orientation *calibrated to agree with `S_u × S_v`*.
///
/// Returns `Ok(None)` when there is no exact closed form for this carrier (the
/// general `Revolution`, and every unrecognised free-form patch), so the caller
/// can fall back.
///
/// **Sign.** The natural closed forms (a sphere's outward radial, a cone's
/// meridian perpendicular) have no fixed sign relation to the patch's
/// parametric normal — a `make_revolution` product's `S_u × S_v` points inward
/// or outward depending on how the generatrix was oriented. So the exact
/// direction is calibrated against the parametric normal once, at `(u, v)` if
/// it is regular there and at the domain midpoint otherwise. If no regular
/// probe exists anywhere tried, there is nothing to calibrate against and the
/// lane declines.
fn exact_analytic_normal(
    surface: &NurbsSurface,
    u: f64,
    v: f64,
    point: Vec3,
) -> Result<Option<Vec3>, String> {
    let Some(analytic) = surface.analytic() else {
        return Ok(None);
    };
    let Some(direction) = exact_direction(surface, analytic, u, v, point)? else {
        return Ok(None);
    };
    let ([u0, u1], [v0, v1]) = domains(surface)?;
    // Calibration probes: here first (free agreement when the patch is regular
    // at the query), then the domain midpoint, then two off-centre staggers
    // that miss a seam or a pole row.
    let probes = [
        (u, v),
        ((u0 + u1) * 0.5, (v0 + v1) * 0.5),
        (u0 + (u1 - u0) * 0.37, v0 + (v1 - v0) * 0.41),
        (u0 + (u1 - u0) * 0.63, v0 + (v1 - v0) * 0.59),
    ];
    for (pu, pv) in probes {
        let Ok(parametric) = surface.normal(pu, pv) else {
            continue;
        };
        let probe_point = if (pu, pv) == (u, v) {
            point
        } else {
            surface.evaluate(pu, pv)?
        };
        let Some(probe_direction) = exact_direction(surface, analytic, pu, pv, probe_point)? else {
            continue;
        };
        let alignment = probe_direction.dot(parametric);
        // A probe whose two readings are near-orthogonal is not a calibration —
        // it is a degenerate row read as noise. Demand a decisive sign.
        if alignment.abs() < 0.5 {
            continue;
        }
        return Ok(Some(if alignment > 0.0 {
            direction
        } else {
            direction.scale(-1.0)
        }));
    }
    Ok(None)
}

/// The closed-form normal direction (unit, canonical orientation, uncalibrated)
/// of one analytic carrier at a point known to lie on it.
fn exact_direction(
    surface: &NurbsSurface,
    analytic: &AnalyticSurface,
    u: f64,
    v: f64,
    point: Vec3,
) -> Result<Option<Vec3>, String> {
    Ok(match analytic {
        AnalyticSurface::Plane { u_dir, v_dir, .. } => u_dir.cross(*v_dir).normalized().ok(),
        AnalyticSurface::Sphere { frame, .. } => {
            // Exact AT the poles, which is precisely where `S_u × S_v`
            // vanishes and the Greville fit that `face_offset_sphere.rs`
            // rejected went wrong.
            point.sub(frame.origin).normalized().ok()
        }
        AnalyticSurface::RuledRevolution {
            frame,
            rho0,
            rho1,
            height,
        } => {
            let radial = match radial_direction(surface, frame.origin, frame.axis, u, v, point)? {
                Some(radial) => radial,
                None => return Ok(None),
            };
            // Meridian generatrix runs (rho0, 0) → (rho1, height) in
            // (radial, axial); the in-meridian perpendicular is
            // (height, −(rho1 − rho0)) over its length.
            let dr = rho1 - rho0;
            let length = (dr * dr + height * height).sqrt();
            if length <= 0.0 {
                return Ok(None);
            }
            radial
                .scale(*height / length)
                .sub(frame.axis.scale(dr / length))
                .normalized()
                .ok()
        }
        AnalyticSurface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => {
            let radial = match radial_direction(surface, frame.origin, frame.axis, u, v, point)? {
                Some(radial) => radial,
                None => return Ok(None),
            };
            if *minor_radius <= 0.0 {
                return Ok(None);
            }
            let tube_centre = frame.origin.add(radial.scale(*major_radius));
            point.sub(tube_centre).normalized().ok()
        }
        // A general revolution's exact normal needs the generatrix tangent at
        // the meridian station, which is a 1D projection — not closed form, and
        // no cheaper than `S_u × S_v`. Decline rather than pretend.
        AnalyticSurface::Revolution { .. } => None,
    })
}

/// Unit radial direction of `point` about the axis.
///
/// On the axis itself (a cone apex, or a sphere/torus degeneracy) the point
/// carries no azimuth — but the *parameter* still does, because `u` names the
/// ruling. Recover it from the opposite end of the same `u` iso-line, which is
/// the exact per-ruling limit the `FaceStable` recovery only approximates by
/// walking inward.
fn radial_direction(
    surface: &NurbsSurface,
    origin: Vec3,
    axis: Vec3,
    u: f64,
    v: f64,
    point: Vec3,
) -> Result<Option<Vec3>, String> {
    let radial_of = |point: Vec3| {
        let relative = point.sub(origin);
        relative.sub(axis.scale(relative.dot(axis)))
    };
    let here = radial_of(point);
    let [v0, v1] = surface.domain_v()?;
    let scale = point.sub(origin).length().max(1.0);
    if here.length() > 1e-12 * scale {
        return Ok(here.normalized().ok());
    }
    let far = if (v - v0).abs() >= (v1 - v).abs() {
        v0
    } else {
        v1
    };
    let candidate = radial_of(surface.evaluate(u, far)?);
    if candidate.length() <= 1e-12 * scale {
        return Ok(None);
    }
    Ok(candidate.normalized().ok())
}

// ---------------------------------------------------------------------------
// Migration diagnostic
// ---------------------------------------------------------------------------

/// Per-site agreement between the normal lanes, aggregated over one thread.
///
/// Enabled by `BREP_OFFSET_DIAG`; the value is a sampling stride (`1` = every
/// call). Printing per call is useless here — one blend march makes hundreds of
/// thousands of evaluations — so the comparison is accumulated and dumped when
/// the thread ends, which for `cargo test` is once per test.
///
/// A site with `calls` but zero disagreement is evidence the corpus does not
/// *exercise* the difference, not proof the lanes are equivalent; a site that
/// never appears was never reached at all. Both readings matter, so the counts
/// are reported alongside the magnitudes.
#[derive(Default, Clone, Copy)]
struct LaneStats {
    compared: u64,
    /// Calls where this lane errored but the selected lane did not.
    lane_errors: u64,
    /// Calls where this lane succeeded and the selected lane did not.
    lane_rescues: u64,
    /// Worst CHORD distance between this lane's unit normal and the selected
    /// lane's, taken as an unsigned DIRECTION: `min(|a − b|, |a + b|)`.
    ///
    /// Chord, not `acos`: two bitwise identical unit vectors have a dot product
    /// one ULP below 1.0 as often as not (three rounded products summed), and
    /// `acos` turns that ULP into a flat 1.49e-8 rad floor that hides every
    /// real signal beneath it.  The chord is `2·sin(θ/2)`, so for a small angle
    /// it IS the angle in radians, and it is exactly 0 for identical vectors.
    ///
    /// Unsigned, because `Raw` carries no orientation while the `Face` lanes
    /// apply `same_sense`: on a reversed face they are exact opposites, and
    /// that is the CONVENTION difference (§4.2), not a geometry difference.
    /// The flips are counted separately so neither reading is lost.
    worst_chord: f64,
    /// Calls where this lane's normal pointed OPPOSITE the selected lane's.
    flipped: u64,
    worst_u: f64,
    worst_v: f64,
}

#[derive(Default, Clone, Copy)]
struct SiteStats {
    calls: u64,
    /// Calls where the lane the caller SELECTED could not produce a normal.
    errors: u64,
    convention: Option<OffsetNormal>,
    lanes: [LaneStats; 4],
}

const LANE_NAMES: [&str; 4] = ["raw", "face", "stable", "exact"];

fn lane_of(index: usize, same_sense: bool) -> OffsetNormal {
    match index {
        0 => OffsetNormal::Raw,
        1 => OffsetNormal::Face { same_sense },
        2 => OffsetNormal::FaceStable { same_sense },
        _ => OffsetNormal::ExactAnalytic { same_sense },
    }
}

struct DiagnosticSink {
    sites: std::collections::BTreeMap<&'static str, SiteStats>,
}

impl Drop for DiagnosticSink {
    fn drop(&mut self) {
        use std::fmt::Write as _;
        for (site, stats) in &self.sites {
            // Build the WHOLE line first and emit it with ONE `eprintln!`.
            // Threads exit concurrently under `cargo test`, and a report
            // assembled from a run of `eprint!`s interleaves mid-line with
            // another thread's — the first version of this lost ~60% of its
            // reports to exactly that, which looked like missing sites rather
            // than shredded ones.
            let mut line = format!(
                "offset-diag site={site} convention={} calls={} errors={}",
                stats
                    .convention
                    .map(|convention| format!("{convention:?}"))
                    .unwrap_or_else(|| "?".to_string()),
                stats.calls,
                stats.errors
            );
            for (index, lane) in stats.lanes.iter().enumerate() {
                if lane.compared == 0 && lane.lane_errors == 0 && lane.lane_rescues == 0 {
                    continue;
                }
                let _ = write!(
                    line,
                    " | {}: n={} chord={:.3e} at=({:.6},{:.6}) flip={} lane_err={} rescue={}",
                    LANE_NAMES[index],
                    lane.compared,
                    lane.worst_chord,
                    lane.worst_u,
                    lane.worst_v,
                    lane.flipped,
                    lane.lane_errors,
                    lane.lane_rescues
                );
            }
            eprintln!("{line}");
        }
    }
}

fn diagnostic_stride() -> u64 {
    static STRIDE: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    *STRIDE.get_or_init(|| match std::env::var("BREP_OFFSET_DIAG") {
        Ok(value) => value.trim().parse::<u64>().unwrap_or(1).max(1),
        Err(_) => 0,
    })
}

thread_local! {
    static DIAGNOSTIC: std::cell::RefCell<DiagnosticSink> = std::cell::RefCell::new(DiagnosticSink {
        sites: std::collections::BTreeMap::new(),
    });
}

/// Compare every lane against the one the caller selected, and accumulate.
///
/// Off unless `BREP_OFFSET_DIAG` is set, and then the only cost on the hot path
/// is one `OnceLock` read and a modulo.
fn offset_normal_diagnostic(
    site: &'static str,
    surface: &NurbsSurface,
    convention: OffsetNormal,
    u: f64,
    v: f64,
    selected: &Result<Vec3, String>,
) {
    let stride = diagnostic_stride();
    if stride == 0 {
        return;
    }
    let sampled = DIAGNOSTIC.with(|sink| {
        let mut sink = sink.borrow_mut();
        let stats = sink.sites.entry(site).or_default();
        stats.convention = Some(convention);
        stats.calls += 1;
        if selected.is_err() {
            stats.errors += 1;
        }
        stats.calls % stride == 0
    });
    if !sampled {
        return;
    }
    let evaluator = OffsetEvaluator::new(site, surface, convention);
    let same_sense = convention.same_sense();
    let mut readings = [(0usize, f64::NAN, false, false); 4];
    for (index, reading) in readings.iter_mut().enumerate() {
        let lane = lane_of(index, same_sense);
        let value = evaluator.normal_with(lane, u, v);
        *reading = match (selected, &value) {
            (Ok(chosen), Ok(other)) => {
                let aligned = chosen.sub(*other).length();
                let opposed = chosen.add(*other).length();
                (index, aligned.min(opposed), true, opposed < aligned)
            }
            (Ok(_), Err(_)) => (index, f64::NAN, false, false),
            (Err(_), Ok(_)) => (index, f64::NAN, true, false),
            (Err(_), Err(_)) => (index, f64::NAN, false, false),
        };
    }
    DIAGNOSTIC.with(|sink| {
        let mut sink = sink.borrow_mut();
        let stats = sink.sites.entry(site).or_default();
        for (index, chord, lane_ok, flipped) in readings {
            let lane = &mut stats.lanes[index];
            if chord.is_finite() {
                lane.compared += 1;
                if flipped {
                    lane.flipped += 1;
                }
                if chord > lane.worst_chord {
                    lane.worst_chord = chord;
                    lane.worst_u = u;
                    lane.worst_v = v;
                }
            } else if selected.is_err() && lane_ok {
                lane.lane_rescues += 1;
            } else if selected.is_ok() && !lane_ok {
                lane.lane_errors += 1;
            }
        }
    });
}

