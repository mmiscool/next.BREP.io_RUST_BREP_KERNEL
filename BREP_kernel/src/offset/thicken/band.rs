//! A fold BAND on a FULL revolution: the half-plane union, revolved.
//!
//! A thicken is the UNION of the sheet's normal segments. When the sheet is a
//! surface of REVOLUTION and the offset reaches the axis, the segments in the
//! folded band cross it and come out the other side — at the azimuth half a turn
//! away. Dropping that band bounds LESS than the sheet sweeps, which is what the
//! fold-band slice refused; carving it is not the answer, and neither is a
//! per-piece volume fitted to the construction.
//!
//! ## The construction
//!
//! For a FULL revolution the far side of every segment lands at an azimuth the
//! sheet itself covers, so the union is again a solid of REVOLUTION and the
//! whole question is planar. In the meridian half-plane, with the tube centre
//! `T` at distance `R` from the axis and the segments running `s ∈ [s_lo, s_hi]`
//! from it over the meridian range `[φ0, φ1]`:
//!
//! ```text
//!     S  = the annular sector  {T + s(cos φ, sin φ) : s ∈ [s_lo, s_hi], φ ∈ [φ0, φ1]}
//!     S' = S ∩ {ρ ≥ 0}                       the part that never left the half-plane
//!     L  = {ρ ≥ 0 : (ρ + R)² + z² ≤ s_hi²}   the REFLECTED lemon — what crossed, folded back
//!     U  = S' ∪ L,  revolved a full turn
//! ```
//!
//! `S ∩ {ρ < 0}` is exactly the disc of radius `s_hi` about `T` cut by the axis
//! (a point with `ρ < 0` has `s > R > s_lo`, and its meridian angle is inside the
//! fold band, which is inside the trim), so `L` is that segment reflected — the
//! LEMON of the spindle `(R, s_hi)`, the same region the swept-envelope lane
//! builds as its far lobe.
//!
//! `L` is not disjoint from `S'`: it reaches into the TUBE HOLE (`s < s_lo`),
//! which `S` never covered. `U = S' ⊔ lens` with `lens = L ∩ {s < s_lo}` — two
//! DISJOINT regions — and that is what makes the closed form a sum rather than
//! an inclusion–exclusion:
//!
//! ```text
//!     V = 2π·[ I(S') + I(lens) ],    I(X) = ∫∫_X ρ dA
//!     I(S)            = Δφ·R(s_hi² − s_lo²)/2 + (s_hi³ − s_lo³)/3·(sin φ1 − sin φ0)
//!     I(S ∩ {ρ<0})    = R·A_seg − (2/3)(s_hi² − R²)^{3/2},  A_seg = s_hi²·acos(R/s_hi) − R√(s_hi²−R²)
//!     I(lens)         = the two circular segments the RADICAL line ρ* = (s_hi² − s_lo²)/(4R) cuts
//! ```
//!
//! The lens is the intersection of two discs whose centres both lie ON the
//! ρ-axis, so their radical line is VERTICAL and the lens splits into one
//! segment of each disc at `ρ*` — no quadrature anywhere in the form.
//!
//! ## What it refuses, by name
//!
//! A PARTIAL revolution above all: there the reflected band lands at azimuths
//! the sheet does not cover, the union is a second BODY rather than a revolve,
//! and there is no planar form to take Pappus on. Also a generatrix that is not
//! a circle, a trim that is not the whole face, a fold that is not a BAND
//! strictly inside the meridian range, and a reflected lemon that reaches
//! material the trim never swept — each with the measurement that decided it.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// How far the generatrix may sit off the circle fitted to it, relative to that
/// circle's radius, and still be called a circular meridian.
///
/// The comparand is the tube radius. A revolve's generatrix is the sketch's own
/// rational arc, which reproduces its circle to machine precision, so this is
/// orders of slack over what an exact arc reads.
pub(crate) const MERIDIAN_RESIDUAL_BAR: f64 = 1e-9;

/// The slack the reflected-band scan reads its own boundary with: relative to
/// the offset radius for a distance, absolute in radians for a meridian angle.
///
/// Every sample of the scan's grid edge lies exactly on the region it is being
/// tested against, so the comparison is of two roundings of one number.
const COVERAGE_SLACK: f64 = 1e-9;

/// How far the trim's own (u, v) rectangle may sit off the surface's full domain
/// — relative to that domain's extent — and still be "the whole face".
const WHOLE_FACE_BAR: f64 = 1e-9;

/// The recognized configuration, in the meridian half-plane's own coordinates.
#[derive(Debug, Clone, Copy)]
pub(crate) struct RevolvedBand {
    /// A point on the revolution axis.
    origin: Vec3,
    /// Unit axis direction; the revolve turns `+θ` about it.
    axis: Vec3,
    /// Unit radial direction of the meridian half-plane the generatrix sits in.
    radial: Vec3,
    /// `R` — the tube centre's distance from the axis.
    major: f64,
    /// The tube centre's axial coordinate, relative to `origin`.
    axial: f64,
    /// The segments' near and far ends, measured from the tube centre.
    low: f64,
    high: f64,
    /// The meridian range the trim covers, `φ` measured from the OUTWARD radial
    /// about the tube centre. `start < end`, `end − start < 2π`.
    start: f64,
    end: f64,
    /// The worst relative residual the generatrix fit read.
    #[cfg_attr(not(test), allow(dead_code))]
    residual: f64,
}

#[cfg_attr(not(test), allow(dead_code))]
impl RevolvedBand {
    /// The tube centre, in 3D.
    fn tube_centre(&self) -> Vec3 {
        self.origin
            .add(self.radial.scale(self.major))
            .add(self.axis.scale(self.axial))
    }

    /// The tube centre REFLECTED across the axis — the reflected lemon's centre.
    fn mirror_centre(&self) -> Vec3 {
        self.origin
            .sub(self.radial.scale(self.major))
            .add(self.axis.scale(self.axial))
    }

    /// The meridian angle at which the far end of the segments meets the axis:
    /// `R + s_hi·cos φ = 0`.
    fn fold_angle(&self) -> f64 {
        (-self.major / self.high).acos()
    }

    /// Half the axis chord the folded band spans, `√(s_hi² − R²)`.
    fn chord_half_length(&self) -> f64 {
        (self.high * self.high - self.major * self.major).max(0.0).sqrt()
    }

    /// The RADICAL line of the tube hole and the reflected lemon: both circles
    /// are centred on the ρ-axis, so they meet on one vertical line.
    fn radical_rho(&self) -> f64 {
        (self.high * self.high - self.low * self.low) / (4.0 * self.major)
    }

    /// `∫∫ ρ dA` over the half-plane union — the whole closed form but for the
    /// `2π` Pappus factor.
    pub(crate) fn moment(&self) -> f64 {
        self.sector_moment() + self.lens_moment()
    }

    /// The closed-form volume of the revolved union.
    pub(crate) fn volume(&self) -> f64 {
        std::f64::consts::TAU * self.moment()
    }

    /// `I(S')` — the annular sector's own moment, less the part the axis cut off.
    fn sector_moment(&self) -> f64 {
        let (low, high, major) = (self.low, self.high, self.major);
        let span = self.end - self.start;
        let sector = span * major * (high * high - low * low) / 2.0
            + (high * high * high - low * low * low) / 3.0 * (self.end.sin() - self.start.sin());
        sector - cut_moment(major, high)
    }

    /// `I(lens)` — the two circular segments the radical line cuts, or zero when
    /// the reflected lemon never reaches the tube hole.
    fn lens_moment(&self) -> f64 {
        let Some((rho, _)) = self.lens_crossing() else {
            return 0.0;
        };
        // The HOLE's share: the disc of radius `low` about the tube centre, the
        // part with ρ ≤ ρ*.
        let inner = rho - self.major;
        let hole_area = std::f64::consts::PI * self.low * self.low - cap_area(inner, self.low);
        let hole = self.major * hole_area
            - (2.0 / 3.0) * (self.low * self.low - inner * inner).max(0.0).powf(1.5);
        // The LEMON's share: the disc of radius `high` about the MIRROR centre,
        // the part with ρ ≥ ρ*.
        let outer = rho + self.major;
        let lemon = -self.major * cap_area(outer, self.high)
            + (2.0 / 3.0) * (self.high * self.high - outer * outer).max(0.0).powf(1.5);
        hole + lemon
    }

    /// Where the reflected lemon crosses INTO the tube hole: `(ρ*, z*)` with
    /// `z* > 0`, or `None` when it never reaches the hole.
    fn lens_crossing(&self) -> Option<(f64, f64)> {
        let rho = self.radical_rho();
        let height = self.low * self.low - (rho - self.major) * (rho - self.major);
        (height > 0.0).then(|| (rho, height.sqrt()))
    }

    /// The half-plane point at tube-centre distance `s` and meridian angle `phi`.
    fn meridian_point(&self, s: f64, phi: f64) -> Vec3 {
        self.tube_centre()
            .add(self.radial.scale(s * phi.cos()))
            .add(self.axis.scale(s * phi.sin()))
    }

    /// The radial distance from the axis of that point — `ρ = R + s·cos φ`.
    fn rho_at(&self, s: f64, phi: f64) -> f64 {
        self.major + s * phi.cos()
    }
}

#[cfg_attr(not(test), allow(dead_code))]
/// `∫∫_{ρ<0} ρ dA` over the disc of radius `radius` centred `major` from the
/// axis — negative, and the term both the sector and the lemon are built from.
fn cut_moment(major: f64, radius: f64) -> f64 {
    let gap = (radius * radius - major * major).max(0.0);
    let segment = radius * radius * (major / radius).acos() - major * gap.sqrt();
    major * segment - (2.0 / 3.0) * gap.powf(1.5)
}

#[cfg_attr(not(test), allow(dead_code))]
/// The area of the circular CAP `{u ≥ cut}` of a disc of radius `radius`
/// centred at `u = 0`, for `|cut| ≤ radius`.
fn cap_area(cut: f64, radius: f64) -> f64 {
    let clamped = (cut / radius).clamp(-1.0, 1.0);
    radius * radius * clamped.acos() - cut * (radius * radius - cut * cut).max(0.0).sqrt()
}

/// Recognize a FULL-revolution sheet whose offset folds in a BAND, or say why
/// this sheet is not one.
///
/// `Ok(None)` is "not this lane" — no fold band, or not a revolution at all — and
/// leaves every other sheet exactly the builder it had. `Err` is a refusal BY
/// NAME: the configuration IS a folded revolution and this construction does not
/// cover it.
pub(crate) fn recognize(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    distance_bottom: f64,
    distance_top: f64,
) -> Result<Option<RevolvedBand>, KernelRefusal> {
    let Some(crate::AnalyticSurface::Revolution {
        frame,
        sweep,
        generatrix,
        ..
    }) = surface.analytic()
    else {
        return Ok(None);
    };
    // The generatrix has to be a CIRCLE for the swept region to be an annular
    // sector with a closed form. Fitted by reconstruction and judged by the
    // residual over every sample, not by the three points the centre came from.
    let samples = generatrix_samples(generatrix)?;
    let Some((tube_centre, tube_radius, residual)) = fit_meridian_circle(&samples) else {
        return Ok(None);
    };
    if residual > MERIDIAN_RESIDUAL_BAR {
        return Ok(None);
    }
    // The meridian half-plane's own frame, read off the tube centre rather than
    // the surface's stored `x_axis`: the generatrix may sit at any azimuth.
    let delta = tube_centre.sub(frame.origin);
    let axial = delta.dot(frame.axis);
    let planar = delta.sub(frame.axis.scale(axial));
    let major = planar.length();
    if major <= tube_radius {
        // The SHEET itself crosses the axis. That is a spindle, not a ring, and
        // its own trim is not a band between two parallels.
        return Ok(None);
    }
    let radial = planar.scale(1.0 / major);
    // Which way the surface normal points decides which end of the segment is
    // which: `thicken` measures along `Su × Sv` and the revolve's own winding
    // sets that.
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    // Which way the surface normal points is measured against the outward
    // direction from the TUBE CENTRE — the direction the offset actually runs —
    // and not from the axis: over the meridian's far half those two disagree,
    // which is the whole of what makes this lane's band fold.
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let middle = surface.evaluate(um, vm).or_refuse(KernelStage::Refine, "evaluate")?;
    let Some(outward) = outward_at(middle, frame.origin, frame.axis, major, axial) else {
        return Ok(None);
    };
    let outward_sign = if surface.normal(um, vm).or_refuse(KernelStage::Refine, "normal")?.dot(outward) > 0.0 {
        1.0
    } else {
        -1.0
    };
    let ends = [
        tube_radius + outward_sign * distance_bottom,
        tube_radius + outward_sign * distance_top,
    ];
    let low = ends[0].min(ends[1]);
    let high = ends[0].max(ends[1]);
    if !(low > 0.0) || low >= major || high <= major {
        // No fold at all (`high ≤ R`), or an offset that eats through the tube
        // centre or past the axis on the near side — neither is this lane's.
        return Ok(None);
    }
    // The meridian range, read off the generatrix's own ends in the half-plane
    // frame, and UNWRAPPED so the run is monotone.
    let (start, end) = meridian_range(&samples, tube_centre, radial, frame.axis)?;
    let band = RevolvedBand {
        origin: frame.origin,
        axis: frame.axis,
        radial,
        major,
        axial,
        low,
        high,
        start,
        end,
        residual,
    };
    // Is the fold a BAND, strictly inside the meridian range? A fold that only
    // touches one end is the single-parallel carve's, which is a different
    // construction and already has one.
    let fold = band.fold_angle();
    let Some(band_low) = wrap_into(fold, start, end) else {
        return Ok(None);
    };
    let band_high = band_low + 2.0 * (std::f64::consts::PI - fold);
    if band_high >= end {
        return Ok(None);
    }
    // The TRIM decides whether this lane is even asked: its half-plane union is
    // the sector the face's own generatrix sweeps, so anything less than the
    // whole face is left to the carve — which is what answers a fold that only
    // covers part of a trim, and what the zig-zag and sub-window fixtures
    // measure. Ok(None) here, never a refusal: those shapes have their own.
    if !whole_face(loops, [u0, u1], [v0, v1])? {
        return Ok(None);
    }
    // From here the configuration IS a folded revolution over a whole face, and
    // every exit is a refusal by name.
    let full_turn = (*sweep - std::f64::consts::TAU).abs() <= 1e-9;
    if !full_turn {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "thicken_band_partial_revolution", format!(
            "thickenSheet: this sheet's offset folds in a BAND and the sheet is revolved only \
             {:.6} rad. The segments in the band cross the axis and come out at the azimuth half \
             a turn away, so for a FULL revolution the union is again a solid of revolution — the \
             half-plane union of the normal segments, which this lane builds — but a PARTIAL one \
             sweeps that reflected band over azimuths the sheet never covers. The answer there is \
             a SECOND body, not a revolve, and it has no planar form to take Pappus on: refused \
             rather than approximated. Revolve the sheet a full turn, or thicken it thin enough \
             that the offset (now {:.6} from the tube centre, against a major radius of {:.6}) \
             does not reach the axis",
            sweep, band.high, band.major
        )));
    }
    reflected_band_is_covered(&band)?;
    Ok(Some(band))
}

/// The direction the sheet's own normal has to be compared against: OUTWARD
/// from the tube centre **at the sample's own azimuth**, not from the one the
/// generatrix sits at.
///
/// The distinction is the whole of this lane's geometry. A mid-domain sample of
/// a FULL revolution is half a turn from the generatrix, so the vector to the
/// generatrix's own tube centre runs across the whole torus and its sign says
/// nothing about which way the sheet faces. Rotating the centre to the sample's
/// meridian first is what makes the reading local.
pub(crate) fn outward_at(point: Vec3, origin: Vec3, axis: Vec3, major: f64, axial: f64) -> Option<Vec3> {
    let delta = point.sub(origin);
    let radial = delta.sub(axis.scale(delta.dot(axis)));
    let length = radial.length();
    if length <= 0.0 {
        return None;
    }
    let centre = origin
        .add(radial.scale(major / length))
        .add(axis.scale(axial));
    Some(point.sub(centre))
}

#[cfg_attr(not(test), allow(dead_code))]
/// Every step of [`recognize`], reported — the instrument behind a fixture that
/// needs to say WHICH clause declined rather than only that the lane did.
pub(crate) fn diagnose(
    surface: &NurbsSurface,
    loops: &[Vec<NurbsCurve>],
    distance_bottom: f64,
    distance_top: f64,
) -> String {
    let Some(crate::AnalyticSurface::Revolution {
        frame,
        sweep,
        generatrix,
        ..
    }) = surface.analytic()
    else {
        return "not a Revolution".into();
    };
    let samples = match generatrix_samples(generatrix) {
        Ok(samples) => samples,
        Err(why) => return format!("generatrix samples failed: {why}"),
    };
    let Some((tube_centre, tube_radius, residual)) = fit_meridian_circle(&samples) else {
        return "generatrix is not a circle (collinear)".into();
    };
    let delta = tube_centre.sub(frame.origin);
    let axial = delta.dot(frame.axis);
    let planar = delta.sub(frame.axis.scale(axial));
    let major = planar.length();
    let mut report = format!(
        "sweep {sweep:.6}, tube centre {tube_centre:?} r {tube_radius:.6} residual {residual:.3e},          R {major:.6}, axial {axial:.6}"
    );
    if major <= tube_radius {
        return report + " -> the sheet itself crosses the axis";
    }
    let radial = planar.scale(1.0 / major);
    let [u0, u1] = surface.domain_u().unwrap_or([0.0, 1.0]);
    let [v0, v1] = surface.domain_v().unwrap_or([0.0, 1.0]);
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let middle = match surface.evaluate(um, vm) {
        Ok(point) => point,
        Err(why) => return report + &format!(" -> evaluate failed: {why}"),
    };
    let normal = match surface.normal(um, vm) {
        Ok(normal) => normal,
        Err(why) => return report + &format!(" -> normal failed: {why}"),
    };
    let Some(outward) = outward_at(middle, frame.origin, frame.axis, major, axial) else {
        return report + " -> the mid-domain sample sits on the axis";
    };
    let outward_sign = if normal.dot(outward) > 0.0 { 1.0 } else { -1.0 };
    let ends = [
        tube_radius + outward_sign * distance_bottom,
        tube_radius + outward_sign * distance_top,
    ];
    let (low, high) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
    report += &format!(", outward {outward_sign}, s [{low:.6}, {high:.6}]");
    if !(low > 0.0) || low >= major || high <= major {
        return report + " -> no fold band in range";
    }
    let (start, end) = match meridian_range(&samples, tube_centre, radial, frame.axis) {
        Ok(range) => range,
        Err(why) => return report + &format!(" -> meridian range failed: {why}"),
    };
    report += &format!(", meridian [{start:.6}, {end:.6}]");
    let fold = (-major / high).acos();
    report += &format!(", fold {fold:.6}");
    let Some(band_low) = wrap_into(fold, start, end) else {
        return report + " -> the fold parallel is outside the meridian range";
    };
    let band_high = band_low + 2.0 * (std::f64::consts::PI - fold);
    report += &format!(", band [{band_low:.6}, {band_high:.6}]");
    if band_high >= end {
        return report + " -> the band is not strictly inside the meridian range";
    }
    match whole_face(loops, [u0, u1], [v0, v1]) {
        Ok(true) => report + ", whole face -> recognized",
        Ok(false) => report + " -> the trim is not the whole face",
        Err(why) => report + &format!(" -> whole-face check failed: {why}"),
    }
}

/// Sample the generatrix end to end — 65 points, the population every fit and
/// every range below reads.
fn generatrix_samples(generatrix: &NurbsCurve) -> Result<Vec<Vec3>, KernelRefusal> {
    let [t0, t1] = generatrix.domain().or_refuse(KernelStage::Refine, "domain")?;
    (0..=64)
        .map(|index| {
            generatrix
                .evaluate(t0 + (t1 - t0) * index as f64 / 64.0)
                .or_refuse(KernelStage::Refine, "evaluate")
        })
        .collect()
}

/// Fit a circle to the sampled generatrix: `(centre, radius, worst relative
/// residual)`, or `None` when the samples are collinear.
pub(crate) fn fit_meridian_circle(samples: &[Vec3]) -> Option<(Vec3, f64, f64)> {
    let (first, middle, last) = (
        samples[0],
        samples[samples.len() / 3],
        samples[2 * samples.len() / 3],
    );
    let ab = middle.sub(first);
    let ac = last.sub(first);
    let normal = ab.cross(ac);
    let denominator = 2.0 * normal.dot(normal);
    if denominator <= 0.0 || !denominator.is_finite() {
        return None;
    }
    let centre = first.add(
        ac.cross(normal)
            .scale(ab.dot(ab))
            .add(normal.cross(ab).scale(ac.dot(ac)))
            .scale(1.0 / denominator),
    );
    let radius = first.sub(centre).length();
    if !(radius > 0.0) {
        return None;
    }
    let residual = samples
        .iter()
        .map(|sample| (sample.sub(centre).length() - radius).abs() / radius)
        .fold(0.0_f64, f64::max);
    Some((centre, radius, residual))
}

/// The meridian range the generatrix covers, unwrapped so the run is monotone
/// increasing and `end − start < 2π`.
fn meridian_range(
    samples: &[Vec3],
    tube_centre: Vec3,
    radial: Vec3,
    axis: Vec3,
) -> Result<(f64, f64), KernelRefusal> {
    let angle = |point: Vec3| {
        let delta = point.sub(tube_centre);
        delta.dot(axis).atan2(delta.dot(radial))
    };
    let mut previous = angle(samples[0]);
    let start = previous;
    let mut total = 0.0;
    for sample in &samples[1..] {
        let current = angle(*sample);
        let mut step = current - previous;
        while step > std::f64::consts::PI {
            step -= std::f64::consts::TAU;
        }
        while step < -std::f64::consts::PI {
            step += std::f64::consts::TAU;
        }
        total += step;
        previous = current;
    }
    if total < 0.0 {
        // The generatrix runs the other way round the tube: read it reversed, so
        // the range is always increasing.
        return Ok((start + total, start));
    }
    Ok((start, start + total))
}

/// `value + 2πk` inside `(low, high)`, or `None` when no shift lands there.
pub(crate) fn wrap_into(value: f64, low: f64, high: f64) -> Option<f64> {
    let mut shifted = value + std::f64::consts::TAU * ((low - value) / std::f64::consts::TAU).floor();
    for _ in 0..3 {
        if shifted > low && shifted < high {
            return Some(shifted);
        }
        shifted += std::f64::consts::TAU;
    }
    None
}

/// Is the trim the WHOLE face — one loop whose (u, v) rectangle is the surface's
/// own domain?
///
/// Measured twice: the bounding box, and the loop's signed AREA against that
/// rectangle's. A loop that wanders inside its own bounding box has the same box
/// and a smaller area, so the pair is what says "the whole rectangle" rather
/// than "as wide as it".
pub(crate) fn whole_face(
    loops: &[Vec<NurbsCurve>],
    [u0, u1]: [f64; 2],
    [v0, v1]: [f64; 2],
) -> Result<bool, KernelRefusal> {
    if loops.len() != 1 {
        return Ok(false);
    }
    let extent = (u1 - u0).max(v1 - v0);
    let mut area = 0.0;
    let mut box_u = (f64::INFINITY, f64::NEG_INFINITY);
    let mut box_v = (f64::INFINITY, f64::NEG_INFINITY);
    for curve in &loops[0] {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut previous = curve.evaluate(t0).or_refuse(KernelStage::Refine, "evaluate")?;
        for index in 1..=32 {
            let point = curve.evaluate(t0 + (t1 - t0) * index as f64 / 32.0).or_refuse(KernelStage::Refine, "evaluate")?;
            area += 0.5 * (previous.x * point.y - point.x * previous.y);
            box_u = (box_u.0.min(point.x), box_u.1.max(point.x));
            box_v = (box_v.0.min(point.y), box_v.1.max(point.y));
            previous = point;
        }
    }
    let rectangle = (u1 - u0) * (v1 - v0);
    Ok((box_u.0 - u0).abs() <= WHOLE_FACE_BAR * extent
        && (box_u.1 - u1).abs() <= WHOLE_FACE_BAR * extent
        && (box_v.0 - v0).abs() <= WHOLE_FACE_BAR * extent
        && (box_v.1 - v1).abs() <= WHOLE_FACE_BAR * extent
        && (area.abs() - rectangle).abs() <= WHOLE_FACE_BAR * rectangle)
}

/// Every point of the reflected lemon must be material this sheet really
/// sweeps: inside the annular sector, or inside the TUBE HOLE the sector never
/// covered (which is where the fold band's own far ends land).
///
/// A BOUND and not a proof, like every other scan in this family: a polar grid
/// over the lemon, and the first sample that is neither refuses by name.
fn reflected_band_is_covered(band: &RevolvedBand) -> Result<(), KernelRefusal> {
    let mirror = band.mirror_centre();
    let limit = (band.major / band.high).acos();
    // The grid's own corners sit ON the region's boundary — `s = s_hi` at
    // `α = ±limit` is the axis crossing itself — so the containment test is
    // taken with the same slack the recognition reads a circle to. Without it
    // the scan refuses its own boundary on the last bit of a square root.
    let slack = COVERAGE_SLACK * band.high;
    for ring in 0..=16 {
        let s = band.high * ring as f64 / 16.0;
        for step in 0..=32 {
            let alpha = -limit + 2.0 * limit * step as f64 / 32.0;
            let point = mirror
                .add(band.radial.scale(s * alpha.cos()))
                .add(band.axis.scale(s * alpha.sin()));
            // The mirror centre sits at ρ = −R, so a point `s` from it at angle
            // `α` has ρ = s·cos α − R; the ones below zero are outside the
            // half-plane and are not part of the reflected region at all.
            if s * alpha.cos() - band.major < 0.0 {
                continue;
            }
            let delta = point.sub(band.tube_centre());
            let distance = delta.length();
            if distance < band.low + slack {
                continue; // the tube hole — swept by the reflection, by nothing else
            }
            let phi = delta.dot(band.axis).atan2(delta.dot(band.radial));
            if distance <= band.high + slack
                && wrap_into(phi, band.start - COVERAGE_SLACK, band.end + COVERAGE_SLACK).is_some()
            {
                continue;
            }
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "thicken_band_unswept_reflection", format!(
                "thickenSheet: this sheet's offset folds in a BAND, and the band's reflection \
                 reaches material the trim never swept — the point at meridian distance {:.6} and \
                 angle {:.6} rad from the tube centre is in neither the swept sector \
                 ({:.6} rad to {:.6} rad, {:.6} to {:.6} from the centre) nor the tube hole. The \
                 half-plane union this lane revolves is the sector plus the reflected lemon, and \
                 that identity is what it just failed",
                distance,
                phi,
                band.start,
                band.end,
                band.low,
                band.high
            )));
        }
    }
    Ok(())
}

/// Build the half-plane union's profile and revolve it a full turn.
pub(crate) fn build(band: &RevolvedBand) -> Result<BrepSolid, KernelRefusal> {
    let centre = band.tube_centre();
    let (radial, axis) = (band.radial, band.axis);
    let fold = band.fold_angle();
    let band_low = wrap_into(fold, band.start, band.end)
        .ok_or_else(|| "thickenSheet: the fold band left the meridian range".to_string()).or_refuse(KernelStage::Refine, "thicken_band_meridian_range")?;
    let band_high = band_low + 2.0 * (std::f64::consts::PI - fold);
    let mut profile: Vec<NurbsCurve> = Vec::with_capacity(8);

    // A. the offset arc from the trim's start to where it meets the axis.
    profile.push(make_arc(
        centre,
        radial,
        axis,
        band.high,
        band.start,
        band_low,
    ).or_refuse(KernelStage::Refine, "make_arc")?);
    // B. the AXIS chord — the fold band's own far end, which revolves to nothing.
    let half = band.chord_half_length();
    let top = band.origin.add(axis.scale(band.axial + half));
    let bottom = band.origin.add(axis.scale(band.axial - half));
    profile.push(make_line(top, bottom).or_refuse(KernelStage::Refine, "make_line")?);
    // C. the offset arc from the axis back out to the trim's end.
    profile.push(make_arc(
        centre,
        radial,
        axis,
        band.high,
        band_high,
        band.end,
    ).or_refuse(KernelStage::Refine, "make_arc")?);
    // D. the extreme normal segment at the trim's END, offset back to sheet.
    profile.push(make_line(
        band.meridian_point(band.high, band.end),
        band.meridian_point(band.low, band.end),
    ).or_refuse(KernelStage::Refine, "make_line")?);
    // E–G. the sheet's own arc back to the start — interrupted, where the
    // reflected lemon reaches into the tube hole, by the lemon's own arc: that
    // hole is material the reflection sweeps and the sheet's arc is no longer
    // the boundary there.
    match band.lens_crossing() {
        Some((rho, height)) => {
            let upper = ((rho - band.major) / band.low)
                .clamp(-1.0, 1.0)
                .acos();
            let lower = std::f64::consts::TAU - upper;
            let upper = wrap_into(upper, band.start, band_low).ok_or_else(|| {
                format!(
                    "thickenSheet: the reflected band meets the sheet at meridian angle {upper:.6} \
                     rad, which is outside the regular part of the trim ({:.6} to {:.6})",
                    band.start, band_low
                )
            }).or_refuse(KernelStage::Refine, "thicken_band_reflection_range")?;
            let lower = wrap_into(lower, band_high, band.end).ok_or_else(|| {
                format!(
                    "thickenSheet: the reflected band meets the sheet at meridian angle {lower:.6} \
                     rad, which is outside the regular part of the trim ({:.6} to {:.6})",
                    band_high, band.end
                )
            }).or_refuse(KernelStage::Refine, "thicken_band_reflection_range")?;
            // E. the sheet, from the trim's end down to the lower crossing.
            profile.push(make_arc(centre, radial, axis, band.low, lower, band.end).or_refuse(KernelStage::Refine, "make_arc")?.reversed().or_refuse(KernelStage::Refine, "reversed")?);
            // F. the reflected lemon, from the lower crossing to the upper one.
            let mirror = band.mirror_centre();
            let alpha = height.atan2(rho + band.major);
            profile.push(make_arc(mirror, radial, axis, band.high, -alpha, alpha).or_refuse(KernelStage::Refine, "make_arc")?);
            // G. the sheet, from the upper crossing back to the trim's start.
            profile.push(make_arc(centre, radial, axis, band.low, band.start, upper).or_refuse(KernelStage::Refine, "make_arc")?.reversed().or_refuse(KernelStage::Refine, "reversed")?);
        }
        None => {
            profile.push(
                make_arc(centre, radial, axis, band.low, band.start, band.end).or_refuse(KernelStage::Refine, "make_arc")?.reversed().or_refuse(KernelStage::Refine, "reversed")?,
            );
        }
    }
    // H. the extreme normal segment at the trim's START, sheet back out to the
    //    offset — closing the loop.
    profile.push(make_line(
        band.meridian_point(band.low, band.start),
        band.meridian_point(band.high, band.start),
    ).or_refuse(KernelStage::Refine, "make_line")?);

    let solid = crate::revolve_profile_brep(
        &profile,
        band.origin,
        axis,
        std::f64::consts::TAU,
    )
    .map_err(|error| {
        KernelRefusal::internal(
            KernelStage::Refine,
            "revolve_profile_brep",
            format!("thickenSheet: the fold band's half-plane union could not be revolved: {error}"),
        )
    })?;
    crate::accept_sound(solid, "thickenSheet")
}

#[cfg_attr(not(test), allow(dead_code))]
/// The measurements a probe or a fixture reads off a recognized band, so the
/// closed form and the geometry it came from are both quotable.
impl RevolvedBand {
    pub(crate) fn report(&self) -> String {
        format!(
            "R = {:.6}, s ∈ [{:.6}, {:.6}], meridian [{:.6}, {:.6}] rad, fold band \
             [{:.6}, {:.6}] rad, axis chord ±{:.6}, radical ρ* = {:.6}, generatrix residual \
             {:.3e}, V = {:.12}",
            self.major,
            self.low,
            self.high,
            self.start,
            self.end,
            self.fold_angle(),
            std::f64::consts::TAU - self.fold_angle(),
            self.chord_half_length(),
            self.radical_rho(),
            self.residual,
            self.volume(),
        )
    }

    /// `ρ` at the trim's two ends, for a fixture that wants to see the sector
    /// really does stay in the half-plane there.
    pub(crate) fn end_radii(&self) -> (f64, f64) {
        (
            self.rho_at(self.high, self.start),
            self.rho_at(self.high, self.end),
        )
    }
}
