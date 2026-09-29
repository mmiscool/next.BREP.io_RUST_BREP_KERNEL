//! The swept envelope TRIMMED at its own self-intersection.
//!
//! Past `ρκ = 1` the sections a sweep skins through CROSS, and the wall lofted
//! between them passes through itself. The correct solid there is not "no
//! solid": it is the union of the swept SECTIONS — the envelope trimmed where
//! it turns back through itself — which for the one configuration with a closed
//! form is a REVOLVE, and is built here as one.
//!
//! ## The configuration, and why it is the only one
//!
//! A circle of radius `r` carried square to a planar CIRCULAR path of radius
//! `R` sweeps a solid of REVOLUTION: every placed section is the start section
//! rotated about the path's own axis, so the union of the placed DISCS is the
//! revolution of one disc. In the meridian half-plane that disc is centred at
//! `(R, 0)` with radius `r`, and for `r > R` it CROSSES the axis — which is the
//! same statement as `ρκ ≥ 1`, since `ρ = r` and `κ = 1/R` at every station.
//!
//! The union in the half-plane is therefore the disc TRUNCATED at the axis
//! together with the REFLECTION of the part that crossed, and each of those is
//! revolved separately:
//!
//! ```text
//!     near arc (x ≥ 0) + the axis chord, revolved Θ  →  the APPLE
//!     far  arc (x ≤ 0) + the axis chord, revolved Θ  →  the LEMON
//! ```
//!
//! The far arc already SITS in the half-plane at azimuth π (its points have
//! `x < 0`), so the same `+Θ` revolve carries it over the azimuths `[π, π+Θ]` —
//! which is where the section's far side really does sweep material. The two
//! lobes are two BODIES: they meet only along the axis chord, and a shell is
//! edge-connected by definition.
//!
//! A FULL turn is the exception and the one-body case: the lemon then lies
//! INSIDE the apple (every point of the reflected part is within the truncated
//! disc), so the apple alone is the swept solid — the outer sheet of the
//! SPINDLE TORUS, with a dimple at each pole.
//!
//! ## The closed forms
//!
//! Pappus on each half-plane region, with `A_seg` the area of the circular
//! segment the axis cuts off and `M = R·A_seg − (2/3)(r² − R²)^{3/2}` its first
//! moment `∫∫_{x<0} x dA` (negative):
//!
//! ```text
//!     A_seg   = r²·acos(R/r) − R·√(r² − R²)
//!     V_apple = Θ·(π r² R − M)
//!     V_lemon = Θ·(−M)
//!     V_apple(2π) = 2π²R r² + V_lemon(2π)          the ring formula plus the lemon
//! ```
//!
//! `R = 0` reads `(4π/3)r³` (a sphere) and `R → r` reads `Θ π r² R` with the
//! lemon vanishing (the horn torus), which is what pins the algebra at both
//! ends.
//!
//! ## What it does NOT cover, and says so
//!
//! A non-circular section, a section not centred on the path under `Rigid`, a
//! path that is not one circle, and a turn in `[π, 2π)` — where the far lobe
//! overlaps the near one and the union stops being two disjoint revolves. Each
//! is refused BY NAME with the measurement that decided it, and the tight-bend
//! refusal carries the reason so a caller learns which half of the
//! configuration it missed.

use super::*;

/// How far a path or section sample may sit off the circle fitted to it,
/// relative to that circle's own radius, and still be called a circle.
///
/// The comparand is the RADIUS, not the model: both curves here come from a
/// sketch as exact rational arcs, which reproduce their circle to machine
/// precision, so this is four orders of slack over what an exact arc reads and
/// still refuses a spline that merely looks round. The measured residual is
/// carried on the plan and quoted by every refusal, so the bar is never the
/// only thing reported.
const CIRCLE_RESIDUAL_BAR: f64 = 1e-7;

/// Below this — again relative to the section radius — the section circle is
/// TANGENT to the axis rather than crossing it: the lemon is empty, the axis
/// chord has no length, and the apple is the whole disc revolved.
///
/// `√(r² − R²)` is the chord's half-length, so a section `1e-14·r` past the bar
/// leaves a chord of `1.4e-7·r` — the profile would be a line no revolve should
/// be asked to carry. The volume this rounds away is the lemon's, which at that
/// separation is `Θ·(2/3)·(1.4e-7·r)³` — below 1e-20·r³.
const TANGENT_CHORD_BAR: f64 = 1e-7;

/// A circle fitted to a sampled curve run, with the residual that says whether
/// the run really is one.
struct FittedCircle {
    centre: Vec3,
    /// Unit normal of the circle's plane, oriented so the run TRAVELS `+θ`.
    axis: Vec3,
    radius: f64,
    /// Worst `| |p − centre| − radius |` and out-of-plane distance over every
    /// sample, relative to the radius.
    residual: f64,
    /// Total turn the run makes about `axis`, summed over the samples.
    turn: f64,
}

/// The recognized configuration: a circular section on a circular planar path,
/// tight enough that the envelope crosses itself.
#[derive(Debug, Clone, Copy)]
pub struct SweptEnvelope {
    /// The path circle's centre — the revolve's origin.
    pub centre: Vec3,
    /// The path plane's unit normal, oriented so the sweep runs `+turn`.
    pub axis: Vec3,
    /// Unit vector from `centre` to the section's centre.
    pub radial: Vec3,
    /// `R`, the path's radius of curvature.
    pub major: f64,
    /// `r`, the section's radius.
    pub minor: f64,
    /// `Θ`, the turn the sweep makes, in `(0, 2π]`.
    pub turn: f64,
    /// The worst relative residual either circle read on recognition.
    pub residual: f64,
}

impl SweptEnvelope {
    /// `ρκ` — `r/R`. The envelope self-intersects at and past 1.
    pub fn ratio(&self) -> f64 {
        self.minor / self.major
    }

    /// Half the axis chord, `√(r² − R²)`: how far the section reaches past the
    /// axis on each side. Zero for a section TANGENT to the axis.
    pub fn chord_half_length(&self) -> f64 {
        (self.minor * self.minor - self.major * self.major)
            .max(0.0)
            .sqrt()
    }

    /// True when the section only TOUCHES the axis: no lemon, no chord, and the
    /// apple is the whole disc revolved.
    pub fn tangent(&self) -> bool {
        self.chord_half_length() <= TANGENT_CHORD_BAR * self.minor
    }

    /// `∫∫_{x<0} x dA` over the section disc — the moment of the part the axis
    /// cuts off, negative, and the one term both closed forms are built from.
    fn cut_moment(&self) -> f64 {
        // The ratio is CLAMPED: the lane engages a hair below `ρκ = 1` (the fit's
        // own slack), where `R/r` is a shade over 1 and `acos` would be NaN. At
        // the bar the whole term vanishes — `A_seg = 0` — and the apple's form
        // reduces to `Θ π r² R`, the plain Pappus reading the sub-bar lane has.
        let gap = (self.minor * self.minor - self.major * self.major).max(0.0);
        let segment = self.minor * self.minor * (self.major / self.minor).clamp(-1.0, 1.0).acos()
            - self.major * gap.sqrt();
        self.major * segment - (2.0 / 3.0) * gap.powf(1.5)
    }

    /// The APPLE's closed-form volume: `Θ·∫∫_{x≥0} x dA`.
    pub fn apple_volume(&self) -> f64 {
        self.turn * (std::f64::consts::PI * self.minor * self.minor * self.major - self.cut_moment())
    }

    /// The LEMON's closed-form volume: `Θ·∫∫_{x<0} |x| dA`. Zero for a tangent
    /// section, and never counted twice — a FULL turn puts the lemon inside the
    /// apple and [`swept_envelope_volume`] is the one to read there.
    pub fn lemon_volume(&self) -> f64 {
        if self.tangent() {
            return 0.0;
        }
        self.turn * -self.cut_moment()
    }
}

/// The whole swept solid's closed-form volume: both lobes for a partial turn,
/// the apple alone for a full one (where the lemon is inside it).
pub fn swept_envelope_volume(plan: &SweptEnvelope) -> f64 {
    if plan.turn >= std::f64::consts::TAU - 1e-9 {
        plan.apple_volume()
    } else {
        plan.apple_volume() + plan.lemon_volume()
    }
}

/// Sample every curve of a run end to end, `per_curve` points each plus the
/// run's final point.
fn run_samples(curves: &[NurbsCurve], per_curve: usize) -> Result<Vec<Vec3>, KernelRefusal> {
    let mut samples = Vec::with_capacity(curves.len() * per_curve + 1);
    for curve in curves {
        let [start, end] = curve.domain().or_refuse(KernelStage::Collect, "domain")?;
        for index in 0..per_curve {
            samples.push(curve.evaluate(start + (end - start) * index as f64 / per_curve as f64).or_refuse(KernelStage::Collect, "evaluate")?);
        }
    }
    let last = curves.last().ok_or_else(|| {
        KernelRefusal::input(KernelStage::Collect, "empty_run", "envelope: empty curve run")
    })?;
    samples.push(last.evaluate(last.domain().or_refuse(KernelStage::Collect, "domain")?[1]).or_refuse(KernelStage::Collect, "evaluate")?);
    Ok(samples)
}

/// Fit a circle to a sampled run BY RECONSTRUCTION: take the centre from three
/// spread samples, then measure every sample against it.
///
/// The fit is three points and the verdict is all of them, which is what makes
/// this a measurement rather than an assumption — a spline through three points
/// of a circle fits the same centre and fails the residual.
fn fit_circle(samples: &[Vec3]) -> Result<Option<FittedCircle>, KernelRefusal> {
    if samples.len() < 5 {
        return Ok(None);
    }
    let first = samples[0];
    let middle = samples[samples.len() / 3];
    let last = samples[2 * samples.len() / 3];
    let Some(centre) = circumcentre(first, middle, last) else {
        return Ok(None);
    };
    let radial = first.sub(centre);
    let radius = radial.length();
    if radius <= 0.0 || !radius.is_finite() {
        return Ok(None);
    }
    let Ok(axis) = radial.cross(middle.sub(centre)).normalized() else {
        return Ok(None);
    };
    let mut residual: f64 = 0.0;
    for sample in samples {
        let delta = sample.sub(centre);
        residual = residual.max((delta.length() - radius).abs() / radius);
        residual = residual.max(delta.dot(axis).abs() / radius);
    }
    // The TURN, summed over consecutive samples so a run past half a turn is
    // measured rather than wrapped. Each step is well under π at 16 samples per
    // curve, so its own arccos is unambiguous; the sign says the run travels the
    // way `axis` was oriented from the first three points.
    let mut turn = 0.0;
    for pair in samples.windows(2) {
        let from = pair[0].sub(centre);
        let into = pair[1].sub(centre);
        let cross = from.cross(into);
        let step = cross.length().atan2(from.dot(into));
        if cross.dot(axis) < 0.0 && step > CIRCLE_RESIDUAL_BAR {
            // The run doubled back: not a single arc travelled one way.
            return Ok(None);
        }
        turn += step;
    }
    Ok(Some(FittedCircle {
        centre,
        axis,
        radius,
        residual,
        turn,
    }))
}

/// The centre of the circle through three points, or `None` when they are
/// collinear.
fn circumcentre(a: Vec3, b: Vec3, c: Vec3) -> Option<Vec3> {
    let ab = b.sub(a);
    let ac = c.sub(a);
    let normal = ab.cross(ac);
    let denominator = 2.0 * normal.dot(normal);
    if denominator <= 0.0 || !denominator.is_finite() {
        return None;
    }
    let term = ac
        .cross(normal)
        .scale(ab.dot(ab))
        .add(normal.cross(ab).scale(ac.dot(ac)));
    Some(a.add(term.scale(1.0 / denominator)))
}

/// Recognize the one configuration the envelope lane builds, or say — in words
/// a refusal can quote — which half of it this sweep is not.
///
/// `Ok(Ok(plan))` is the configuration; `Ok(Err(reason))` is a measured reason
/// it is not. A path that is not tight at all reads `Err` too: this lane only
/// ever speaks past `ρκ = 1`, and below it the skinning lanes own the shape.
pub fn recognize_swept_envelope(
    profile: &[NurbsCurve],
    path: &SweepPath,
    placement_mode: SectionPlacement,
) -> Result<Result<SweptEnvelope, String>, KernelRefusal> {
    let path_samples = run_samples(&path.curves, 16)?;
    let Some(path_circle) = fit_circle(&path_samples)? else {
        return Ok(Err("its path is not a circular arc".into()));
    };
    if path_circle.residual > CIRCLE_RESIDUAL_BAR {
        return Ok(Err(format!(
            "its path is not a circular arc (its samples sit up to {:.3e} of the fitted radius \
             {:.6} off that circle, against a bar of {CIRCLE_RESIDUAL_BAR:.0e})",
            path_circle.residual, path_circle.radius
        )));
    }
    let Some(section_circle) = fit_circle(&run_samples(profile, 16)?)? else {
        return Ok(Err("its section is not a circle".into()));
    };
    if section_circle.residual > CIRCLE_RESIDUAL_BAR {
        return Ok(Err(format!(
            "its section is not a circle (its samples sit up to {:.3e} of the fitted radius \
             {:.6} off that circle, against a bar of {CIRCLE_RESIDUAL_BAR:.0e})",
            section_circle.residual, section_circle.radius
        )));
    }
    // A CLOSED loop every one of whose points lies on one circle IS that whole
    // circle — a partial arc cannot close — so the full turn is checked rather
    // than assumed, and a section that is a circular arc plus something else has
    // already failed the residual above.
    let last = profile.last().ok_or_else(|| {
        KernelRefusal::input(KernelStage::Collect, "empty_section", "envelope: empty section")
    })?;
    let closure = last
        .evaluate(last.domain().or_refuse(KernelStage::Classify, "domain")?[1]).or_refuse(KernelStage::Classify, "evaluate")?
        .sub(profile[0].evaluate(profile[0].domain().or_refuse(KernelStage::Classify, "domain")?[0]).or_refuse(KernelStage::Classify, "evaluate")?)
        .length();
    if closure > CIRCLE_RESIDUAL_BAR * section_circle.radius {
        return Ok(Err(format!(
            "its section is not a closed circle (it ends {closure:.3e} from where it starts)"
        )));
    }
    let major = path_circle.radius;
    let minor = section_circle.radius;
    // The bar is `ρκ = 1`, and it is read off the two FITTED radii rather than
    // off the sampled reach the tight-bend guard uses — which under-reads a
    // circle by 3.0e-4·r, so a section exactly as wide as its bend would fall on
    // whichever side of the sampling. The `1 − 1e-7` is the fit's own slack: two
    // radii fitted from exact rational arcs agree to 1e-15, and a section that
    // close to the bar sweeps a lemon of volume below 1e-20·r³ — nothing this
    // lane could resolve either way. Below it the skinning lanes own the shape,
    // and every sweep that builds today keeps the builder it has.
    if minor < major * (1.0 - CIRCLE_RESIDUAL_BAR) {
        return Ok(Err(format!(
            "its bend is not tighter than its section (ρ·κ = {:.6} < 1)",
            minor / major
        )));
    }
    let start = path_samples[0];
    let radial = start.sub(path_circle.centre).normalized().or_refuse(KernelStage::Classify, "normalized")?;
    // `Rigid` carries the section from where it was DRAWN, so it has to be drawn
    // centred on the path and square to it for the sweep to be a revolve;
    // `Transplant` MOVES it onto the path and squares it, so a circle drawn
    // anywhere becomes the same placed circle and only its radius matters.
    if placement_mode == SectionPlacement::Rigid {
        let offset = section_circle.centre.sub(start).length();
        if offset > CIRCLE_RESIDUAL_BAR * minor.max(major) {
            return Ok(Err(format!(
                "its section is not centred on the path (its centre sits {offset:.6} off the \
                 path's start, and `pathAlign` carries a section from where it was drawn)"
            )));
        }
        let out_of_plane = section_circle
            .axis
            .dot(path_circle.axis)
            .abs()
            .max(section_circle.axis.dot(radial).abs());
        if out_of_plane > CIRCLE_RESIDUAL_BAR {
            return Ok(Err(format!(
                "its section is not square to the path (its plane's normal is {:.3e} off the \
                 path's start tangent)",
                out_of_plane
            )));
        }
    }
    let turn = if path.closed {
        std::f64::consts::TAU
    } else {
        path_circle.turn
    };
    if path.closed && (path_circle.turn - std::f64::consts::TAU).abs() > CIRCLE_RESIDUAL_BAR {
        return Ok(Err(format!(
            "its path closes without going once round its own circle (it turns {:.6} rad)",
            path_circle.turn
        )));
    }
    Ok(Ok(SweptEnvelope {
        centre: path_circle.centre,
        axis: path_circle.axis,
        radial,
        major,
        minor,
        turn,
        residual: path_circle.residual.max(section_circle.residual),
    }))
}

/// The prefix every refusal from this lane starts with, so a caller can
/// CLASSIFY one without matching free text.
pub const SWEEP_ENVELOPE_REFUSAL: &str = "sweepSolid: the swept envelope";

/// Build the swept union: the APPLE first, then the LEMON where the turn leaves
/// one outside it.
///
/// One body for a FULL turn (the lemon is inside the apple) and for a section
/// merely TANGENT to the axis (there is no lemon); two for a turn under half a
/// revolution. A turn in `[π, 2π)` is refused by name — the far lobe then
/// OVERLAPS the near one and the union is no longer two disjoint revolves.
pub fn build_swept_envelope(plan: &SweptEnvelope) -> Result<Vec<BrepSolid>, KernelRefusal> {
    use std::f64::consts::{PI, TAU};
    if !(plan.turn > 0.0 && plan.turn <= TAU + 1e-9) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "turn_range",
            format!(
                "{SWEEP_ENVELOPE_REFUSAL} needs a turn in (0, 2π]; this path turns {:.6} rad",
                plan.turn
            ),
        ));
    }
    let full = plan.turn >= TAU - 1e-9;
    if full && plan.tangent() {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "horn_torus",
            format!(
            "{SWEEP_ENVELOPE_REFUSAL} of a section exactly as wide as its bend (ρ·κ = {:.6}, the \
             section reaching {:.3e} of its own radius past the axis) carried ALL the way round \
             is a HORN torus: the tube touches the axis at one point and the solid is PINCHED \
             there, which is not a boundary representation this kernel can hold — its Euler count \
             reads one short of a sphere's and no face pair meets along an edge at that point. \
             Sweep less than a full turn, where the same section builds, or widen the bend",
            plan.ratio(),
            plan.chord_half_length() / plan.minor
            ),
        ));
    }
    if !full && plan.turn >= PI && !plan.tangent() {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "lobe_overlap",
            format!(
            "{SWEEP_ENVELOPE_REFUSAL} of a section that crosses the axis (ρ·κ = {:.6}) sweeps a \
             SECOND lobe at the azimuths half a turn from the path, and this path turns {:.6} rad \
             — past π that lobe runs back into the first and the union is no longer two disjoint \
             revolves. Sweep the run in pieces of less than half a turn, or all the way round",
            plan.ratio(),
            plan.turn
            ),
        ));
    }
    let turn = if full { TAU } else { plan.turn };
    let centre = plan.centre;
    let up = plan.axis;
    let section_centre = centre.add(plan.radial.scale(plan.major));
    let mut bodies = Vec::with_capacity(2);
    bodies.push(revolve_lobe(
        &near_profile(plan, section_centre)?,
        centre,
        up,
        turn,
    )?);
    if !full && !plan.tangent() {
        bodies.push(revolve_lobe(
            &far_profile(plan, section_centre)?,
            centre,
            up,
            turn,
        )?);
    }
    Ok(bodies)
}

/// The section arc on the AXIS side of the path (`x ≥ 0`), closed by the axis
/// chord — or, for a section merely tangent to the axis, the whole section
/// circle split at the tangency and at the outer point (there is no chord to
/// close with, and a revolve may touch its axis at a boundary VERTEX).
fn near_profile(plan: &SweptEnvelope, section_centre: Vec3) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    use std::f64::consts::PI;
    let (u, w) = (plan.radial, plan.axis);
    if plan.tangent() {
        return Ok(vec![
            make_arc(section_centre, u, w, plan.minor, -PI, 0.0).or_refuse(KernelStage::Fragment, "make_arc")?,
            make_arc(section_centre, u, w, plan.minor, 0.0, PI).or_refuse(KernelStage::Fragment, "make_arc")?,
        ]);
    }
    let phi = (-plan.major / plan.minor).acos();
    let half = plan.chord_half_length();
    Ok(vec![
        make_arc(section_centre, u, w, plan.minor, -phi, phi).or_refuse(KernelStage::Fragment, "make_arc")?,
        make_line(
            plan.centre.add(w.scale(half)),
            plan.centre.sub(w.scale(half)),
        ).or_refuse(KernelStage::Fragment, "make_line")?,
    ])
}

/// The section arc on the FAR side of the axis (`x ≤ 0`), closed by the same
/// axis chord. It already lies in the half-plane at azimuth π, so revolving it
/// through the same `+Θ` carries it over `[π, π+Θ]` — where the section's far
/// side really does sweep.
fn far_profile(plan: &SweptEnvelope, section_centre: Vec3) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    use std::f64::consts::TAU;
    let (u, w) = (plan.radial, plan.axis);
    let phi = (-plan.major / plan.minor).acos();
    let half = plan.chord_half_length();
    Ok(vec![
        make_arc(section_centre, u, w, plan.minor, phi, TAU - phi).or_refuse(KernelStage::Fragment, "make_arc")?,
        make_line(
            plan.centre.sub(w.scale(half)),
            plan.centre.add(w.scale(half)),
        ).or_refuse(KernelStage::Fragment, "make_line")?,
    ])
}

/// Revolve one lobe's profile and put it through the soundness acceptance — the
/// same gate the loft's exit applies, because the whole point of this lane is
/// that the loft's answer here passes through itself.
fn revolve_lobe(
    profile: &[NurbsCurve],
    centre: Vec3,
    axis: Vec3,
    turn: f64,
) -> Result<BrepSolid, KernelRefusal> {
    // The revolve is still a stringly builder: its text is kept and classed
    // Internal here, at the consuming site.
    let solid = crate::revolve_profile_brep(profile, centre, axis, turn).map_err(|error| {
        KernelRefusal::internal(
            KernelStage::Fragment,
            "revolve",
            format!("{SWEEP_ENVELOPE_REFUSAL} could not be revolved: {error}"),
        )
    })?;
    crate::accept_sound(solid, "sweepSolid")
}

/// The swept union as BODIES — the entry for a caller that can take more than
/// one, which is what a partial turn past `ρκ = 1` really sweeps.
///
/// Refuses by name, quoting the measurement, when the configuration is not the
/// one this lane builds.
pub fn sweep_envelope_bodies(
    profile: &[NurbsCurve],
    path: &SweepPath,
    placement_mode: SectionPlacement,
) -> Result<Vec<BrepSolid>, KernelRefusal> {
    match recognize_swept_envelope(profile, path, placement_mode)? {
        Ok(plan) => build_swept_envelope(&plan),
        Err(reason) => Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "envelope_configuration",
            format!(
            "{SWEEP_ENVELOPE_REFUSAL} trimmed at its own self-intersection is built for a CIRCLE \
             carried along one circular planar arc, and this sweep is not that: {reason}. Nothing \
             else has a closed form to gate the trim with, so it is refused rather than \
             approximated"
            ),
        )),
    }
}
