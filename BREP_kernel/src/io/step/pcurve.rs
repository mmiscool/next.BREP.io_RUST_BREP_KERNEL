//! Curve-on-surface geometry for the AP242 writer: the 2D half of every
//! `SURFACE_CURVE`/`SEAM_CURVE` bundle `io/step.rs` emits.
//!
//! The kernel already carries an exact pcurve on every coedge
//! (`CoedgeRecord.pcurve`), but it is expressed in the STORED NURBS surface's
//! parameters, which are NOT the parameters of the analytic STEP entity we
//! write for that carrier: our periodic direction is a rational-arc parameter
//! and our sphere/torus meridians are 90°-span rational arcs
//! (`geometry/analytic_surface.rs`), while `CYLINDRICAL_SURFACE` and friends
//! are angle/length parameterized by ISO 10303-42.  A stored pcurve therefore
//! cannot be moved onto an emitted analytic entity by any affine remap — an
//! affine map puts interior points at the wrong angle, which is exactly the
//! "trusted but wrong" geometry this kernel refuses to ship.
//!
//! So this module derives the 2D curve in the EMITTED entity's own
//! parameterization, by one uniform recipe:
//!
//! 1. sample the emitted 3D curve at its OWN STEP parameters `sᵢ`,
//! 2. obtain `(u,v)ᵢ` for each sample — by closed-form inversion through the
//!    emitted frame for an analytic carrier, or by reading the stored pcurve
//!    at the matching fraction for a B-spline carrier (where emitted UV IS
//!    stored UV),
//! 3. propose the most exact 2D entity that can carry those points with a
//!    parameterization SHARED with the 3D entity (2D `LINE`, 2D `CIRCLE`, the
//!    stored net remapped, or — for an edge that is neither an iso-line nor a
//!    rim, which is what a boolean or a fillet produces — a cubic fitted
//!    THROUGH the inverted samples and parameterized by the emitted 3D
//!    parameter itself),
//! 4. VERIFY the proposal on an independent dense sample against ISO 10303-42
//!    evaluators for both emitted entities, and OMIT it if it misses the
//!    export band.
//!
//! Step 4 is the fail-safe hinge: an omitted pcurve costs a reader one
//! reprojection — exactly what every reader must do for our files today — and
//! is counted in the export report.  A wrong pcurve costs it a wrong solid, so
//! it is never written.
//!
//! Why a 2D `LINE` is EXACT on an angle-parameterized carrier, which is the
//! whole reason analytic exports stay B-spline-free: a rim circle emitted as a
//! 3D `CIRCLE` is parameterized by its own angle, and on a cylinder/cone that
//! angle IS the surface's u up to a constant and a sign, so the 2D image is
//! `(u₀ ± s, v₀)` — a straight line in parameter space whose parameter is the
//! 3D one.  A ruling or meridian is the same statement with the roles swapped.

use crate::analytic_surface::circle_angle_to_parameter;
use crate::{interpolate_curve, NurbsCurve, NurbsSurface, Vec3};

const TAU: f64 = std::f64::consts::TAU;

/// Samples of the emitted 3D curve the candidate builders see.  33 matches the
/// seed count of `adaptive_coedge_error`, the validator that already gates
/// these pcurves on the way into the exporter.
const FIT_SAMPLES: usize = 33;

/// Sample counts a fitted pcurve is retried at, coarsest first.  A fit is only
/// as good as the sample spacing lets it be, and refusing a pcurve that a
/// denser fit would have carried exactly is a real cost — so a fit that lands
/// inside the band but not comfortably inside it is refined rather than
/// shipped at the edge.
const REFINEMENT_SAMPLES: [usize; 4] = [FIT_SAMPLES, 65, 129, 257];

/// Refinement stops once a fit verifies to this fraction of the export band.
/// A pcurve at 1% of the band is exact for every practical purpose and the
/// next doubling would only buy control points.
const REFINEMENT_TARGET: f64 = 0.01;

/// Samples the verify-or-omit gate re-checks a proposal on.  Deliberately not
/// a multiple of `FIT_SAMPLES` and offset off the endpoints, so a candidate is
/// never graded only on points it was built from.
const VERIFY_SAMPLES: usize = 97;

/// The frame of an emitted revolution entity: the `AXIS2_PLACEMENT_3D` that
/// was actually written, plus which way its azimuth runs relative to the
/// STORED carrier's own frame.
#[derive(Clone, Debug)]
pub(super) struct EmittedFrame {
    pub origin: Vec3,
    /// Placement `ref_direction`; STEP azimuth u = 0 points along it.
    pub x_axis: Vec3,
    /// axis × x_axis — the second axis a reader derives, so u increases
    /// counterclockwise about the placement axis.
    pub y_axis: Vec3,
    /// Placement `axis`.
    pub axis: Vec3,
    /// `+1` when this placement axis agrees with the stored analytic frame's
    /// axis, `-1` when the cone's "radius must grow along +axis" flip in
    /// `write_analytic_surface` reversed it.  A reversed axis makes the emitted
    /// azimuth run the OTHER way from the stored u parameter, which the
    /// periodic-branch anchor below has to know.
    pub azimuth_sign: f64,
}

impl EmittedFrame {
    /// Azimuth of a frame-relative offset about the placement axis, or `None`
    /// on the axis itself (a sphere pole), where every meridian ties.
    fn azimuth(&self, offset: Vec3) -> Option<f64> {
        let x = offset.dot(self.x_axis);
        let y = offset.dot(self.y_axis);
        let scale = 1.0 + offset.length();
        (x.hypot(y) > 1e-12 * scale).then(|| y.atan2(x))
    }
}

/// The surface entity actually written for a face, with enough geometry to
/// evaluate it in ISO 10303-42's own parameterization — the only
/// parameterization a pcurve may be expressed in.
#[derive(Clone, Debug)]
pub(super) enum EmittedSurface {
    /// `PLANE`: S(x,y) = origin + x·x̂ + y·ŷ, with ŷ = normal × x̂ derived by
    /// the reader from the `AXIS2_PLACEMENT_3D` we wrote.
    Plane {
        origin: Vec3,
        x_axis: Vec3,
        y_axis: Vec3,
    },
    /// `CYLINDRICAL_SURFACE`: S(u,v) = o + r(cos u·x̂ + sin u·ŷ) + v·ẑ.
    Cylinder { frame: EmittedFrame, radius: f64 },
    /// `CONICAL_SURFACE`: S(u,v) = o + (r + v·tan α)(cos u·x̂ + sin u·ŷ) + v·ẑ.
    ///
    /// v is the AXIAL coordinate, per ISO 10303-42.  (OpenCASCADE's own
    /// `Geom_ConicalSurface` measures v along the GENERATRIX and converts on
    /// write — its rim pcurves land at the axial value, confirming the STEP
    /// reading, while its ruling pcurves keep the unit-speed generatrix
    /// direction and so disagree with their 3D line by a factor cos α.  We
    /// carry the true 2D `VECTOR` magnitude instead and stay exact.)
    Cone {
        frame: EmittedFrame,
        radius: f64,
        semi_angle: f64,
    },
    /// `SPHERICAL_SURFACE`: S(u,v) = o + r·cos v(cos u·x̂ + sin u·ŷ) + r·sin v·ẑ,
    /// v the latitude in [−π/2, π/2] — NOT the kernel's south-pole polar angle.
    Sphere { frame: EmittedFrame, radius: f64 },
    /// `TOROIDAL_SURFACE`: S(u,v) = o + (R + r·cos v)(cos u·x̂ + sin u·ŷ) + r·sin v·ẑ,
    /// v measured from the OUTER equator.
    Torus {
        frame: EmittedFrame,
        major_radius: f64,
        minor_radius: f64,
    },
    /// `B_SPLINE_SURFACE_WITH_KNOTS` (or its rational complex form): the
    /// emitted control net and knots ARE the stored ones, so emitted UV is
    /// stored UV and the stored pcurve transfers directly.
    Spline,
}

impl EmittedSurface {
    /// ISO 10303-42 evaluation of the emitted entity.  `stored` is consulted
    /// only for the B-spline carrier, whose emitted parameterization is the
    /// stored one; `evaluate_extended` there so a seam-riding pcurve that
    /// carries parameters just past the domain still lands on the surface.
    fn evaluate(&self, stored: &NurbsSurface, u: f64, v: f64) -> Result<Vec3, String> {
        let radial = |frame: &EmittedFrame, rho: f64, along: f64| {
            frame
                .origin
                .add(frame.x_axis.scale(rho * u.cos()))
                .add(frame.y_axis.scale(rho * u.sin()))
                .add(frame.axis.scale(along))
        };
        Ok(match self {
            EmittedSurface::Plane {
                origin,
                x_axis,
                y_axis,
            } => origin.add(x_axis.scale(u)).add(y_axis.scale(v)),
            EmittedSurface::Cylinder { frame, radius } => radial(frame, *radius, v),
            EmittedSurface::Cone {
                frame,
                radius,
                semi_angle,
            } => radial(frame, radius + v * semi_angle.tan(), v),
            EmittedSurface::Sphere { frame, radius } => {
                radial(frame, radius * v.cos(), radius * v.sin())
            }
            EmittedSurface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => radial(
                frame,
                major_radius + minor_radius * v.cos(),
                minor_radius * v.sin(),
            ),
            EmittedSurface::Spline => return stored.evaluate_extended(u, v),
        })
    }

    fn frame(&self) -> Option<&EmittedFrame> {
        match self {
            EmittedSurface::Plane { .. } | EmittedSurface::Spline => None,
            EmittedSurface::Cylinder { frame, .. }
            | EmittedSurface::Cone { frame, .. }
            | EmittedSurface::Sphere { frame, .. }
            | EmittedSurface::Torus { frame, .. } => Some(frame),
        }
    }

    /// Closed-form inverse of [`EmittedSurface::evaluate`] for a point known to
    /// lie on the carrier.  The azimuth is `None` for an on-axis point (a
    /// sphere pole), where the surface collapses and every meridian ties; the
    /// caller fills those from a neighbouring sample.  `None` overall for the
    /// B-spline carrier, whose pcurve comes from the stored net instead.
    fn invert(&self, point: Vec3) -> Option<(Option<f64>, f64)> {
        match self {
            EmittedSurface::Plane {
                origin,
                x_axis,
                y_axis,
            } => {
                let offset = point.sub(*origin);
                Some((Some(offset.dot(*x_axis)), offset.dot(*y_axis)))
            }
            EmittedSurface::Cylinder { frame, .. } | EmittedSurface::Cone { frame, .. } => {
                let offset = point.sub(frame.origin);
                Some((frame.azimuth(offset), offset.dot(frame.axis)))
            }
            EmittedSurface::Sphere { frame, .. } => {
                let offset = point.sub(frame.origin);
                let rho = offset.dot(frame.x_axis).hypot(offset.dot(frame.y_axis));
                // atan2 rather than asin: exact at the poles and insensitive to
                // a radius the recognizer measured a few ulps away.
                Some((frame.azimuth(offset), offset.dot(frame.axis).atan2(rho)))
            }
            EmittedSurface::Torus {
                frame,
                major_radius,
                ..
            } => {
                let offset = point.sub(frame.origin);
                let rho = offset.dot(frame.x_axis).hypot(offset.dot(frame.y_axis));
                Some((
                    frame.azimuth(offset),
                    offset.dot(frame.axis).atan2(rho - major_radius),
                ))
            }
            EmittedSurface::Spline => None,
        }
    }

    /// Period of each emitted parameter direction, or `None` where the
    /// parameter is single-valued (a plane's coordinates, a cylinder/cone
    /// height, a sphere's latitude).
    fn periods(&self) -> (Option<f64>, Option<f64>) {
        match self {
            EmittedSurface::Plane { .. } | EmittedSurface::Spline => (None, None),
            EmittedSurface::Cylinder { .. }
            | EmittedSurface::Cone { .. }
            | EmittedSurface::Sphere { .. } => (Some(TAU), None),
            EmittedSurface::Torus { .. } => (Some(TAU), Some(TAU)),
        }
    }
}

/// The 3D curve entity actually written for an edge, together with the
/// parameterization ISO 10303-42 gives it — the parameter a pcurve on this
/// edge has to share.
#[derive(Clone, Debug)]
pub(super) enum EmittedCurve {
    /// `LINE(pnt, VECTOR(dir, |chord|))`: C(s) = start + s·(end − start), s ∈ [0,1].
    /// The magnitude carries the chord length, so the STEP parameter is the
    /// fraction along the chord (see `write_analytic_curve`).
    Line { start: Vec3, end: Vec3 },
    /// `CIRCLE(placement, r)`: C(a) = c + r(cos a·x̂ + sin a·ŷ), a ∈ [0, sweep] —
    /// the parameter IS the angle, not the kernel's rational-arc parameter.
    Circle {
        center: Vec3,
        x_axis: Vec3,
        y_axis: Vec3,
        radius: f64,
        sweep: f64,
        /// Span count of the kernel's rational-quadratic arc, which converts
        /// this angle back to the kernel's own parameter exactly.
        spans: usize,
    },
    /// `B_SPLINE_CURVE_WITH_KNOTS` (or its rational complex form) over the edge
    /// subcurve's own knot domain, so the STEP parameter is the kernel
    /// parameter.
    Spline { curve: NurbsCurve },
}

impl EmittedCurve {
    pub(super) fn domain(&self) -> Result<[f64; 2], String> {
        Ok(match self {
            EmittedCurve::Line { .. } => [0.0, 1.0],
            EmittedCurve::Circle { sweep, .. } => [0.0, *sweep],
            EmittedCurve::Spline { curve } => curve.domain()?,
        })
    }

    fn evaluate(&self, parameter: f64) -> Result<Vec3, String> {
        Ok(match self {
            EmittedCurve::Line { start, end } => start.add(end.sub(*start).scale(parameter)),
            EmittedCurve::Circle {
                center,
                x_axis,
                y_axis,
                radius,
                ..
            } => center
                .add(x_axis.scale(radius * parameter.cos()))
                .add(y_axis.scale(radius * parameter.sin())),
            EmittedCurve::Spline { curve } => curve.evaluate(parameter)?,
        })
    }

    /// True when the emitted STEP parameter is an affine image of the kernel's
    /// fraction-of-domain coordinate — the synchronisation the validator
    /// enforces between a coedge pcurve and its edge.
    ///
    /// `Line` and `Spline` are, by construction: a two-point degree-1 net is
    /// uniform, and a spline goes out on its own knots.  A `Circle` is NOT —
    /// the kernel's carrier is a rational quadratic arc whose parameter is a
    /// tangent-half-angle image of the angle — so a stored pcurve may never be
    /// REMAPPED onto one (though it can still be RESAMPLED through
    /// [`EmittedCurve::fraction_at`], which is exact).
    fn parameter_is_affine_in_fraction(&self) -> bool {
        matches!(
            self,
            EmittedCurve::Line { .. } | EmittedCurve::Spline { .. }
        )
    }

    /// The kernel's fraction-of-domain coordinate at an emitted STEP parameter.
    ///
    /// For a recognized `CIRCLE` this is the EXACT inverse of the `make_arc`
    /// construction the recognizer matched: `circle_angle_to_parameter` maps an
    /// angle to the tangent-half-angle parameter of the same rational quadratic
    /// arc, and recognition already proved the kernel curve IS that pristine
    /// arc over [0, 1] (a split subrange fails to match and honestly stays a
    /// spline). So a stored pcurve can be read at the right place for an
    /// emitted angle even though it cannot be remapped onto one.
    fn fraction_at(&self, parameter: f64) -> Result<f64, String> {
        Ok(match self {
            EmittedCurve::Line { .. } => parameter,
            EmittedCurve::Circle { sweep, spans, .. } => {
                circle_angle_to_parameter(*spans, *sweep, parameter)
            }
            EmittedCurve::Spline { curve } => {
                let [t0, t1] = curve.domain()?;
                if (t1 - t0).abs() <= f64::EPSILON {
                    return Err("export_step: emitted spline has an empty domain".into());
                }
                (parameter - t0) / (t1 - t0)
            }
        })
    }
}

/// A 2D curve in an emitted surface's parameter space, ready to be written
/// into a `DEFINITIONAL_REPRESENTATION`.
#[derive(Clone, Debug)]
pub(super) enum Pcurve2d {
    /// `LINE(pnt, VECTOR(dir, |vector|))`: P(s) = point + s·vector.
    Line { point: [f64; 2], vector: [f64; 2] },
    /// `CIRCLE(AXIS2_PLACEMENT_2D(centre, ref), r)`: P(a) = c + r(cos a·ref +
    /// sin a·ref⊥), counterclockwise by definition — ISO 10303-42 derives the
    /// second 2D axis as the +90° rotation of `ref_direction`, so a 2D
    /// placement can never be left-handed.
    Circle {
        center: [f64; 2],
        ref_direction: [f64; 2],
        radius: f64,
    },
    /// A 2D B-spline: the stored coedge net remapped onto the emitted
    /// parameter.  `z` is always 0; only `x`/`y` are written.
    Spline(NurbsCurve),
}

impl Pcurve2d {
    fn evaluate(&self, parameter: f64) -> Result<(f64, f64), String> {
        Ok(match self {
            Pcurve2d::Line { point, vector } => (
                point[0] + parameter * vector[0],
                point[1] + parameter * vector[1],
            ),
            Pcurve2d::Circle {
                center,
                ref_direction,
                radius,
            } => {
                let (sin, cos) = parameter.sin_cos();
                (
                    center[0] + radius * (cos * ref_direction[0] - sin * ref_direction[1]),
                    center[1] + radius * (cos * ref_direction[1] + sin * ref_direction[0]),
                )
            }
            Pcurve2d::Spline(curve) => {
                let point = curve.evaluate(parameter)?;
                (point.x, point.y)
            }
        })
    }
}

/// What [`build_pcurve`] decided, including the measured evidence for it.
pub(super) struct PcurveOutcome {
    /// `None` when every candidate missed the export band: the reader
    /// reprojects, exactly as it must for our pcurve-free files today.
    pub curve: Option<Pcurve2d>,
    /// Max ‖S(c₂d(s)) − c₃d(s)‖ of the accepted candidate, or of the best
    /// rejected one when nothing was accepted (infinite when no candidate was
    /// even proposed).
    pub deviation: f64,
}

/// Build the 2D curve for one coedge use of one edge.
///
/// `pcurve` must ALREADY be reversed into edge direction when the coedge runs
/// against the edge; fraction-synchronisation makes that reversal exact.
pub(super) fn build_pcurve(
    stored_surface: &NurbsSurface,
    emitted_surface: &EmittedSurface,
    emitted_curve: &EmittedCurve,
    pcurve: &NurbsCurve,
    tolerance: f64,
) -> Result<PcurveOutcome, String> {
    let [s0, s1] = emitted_curve.domain()?;
    if s1 <= s0 {
        return Ok(PcurveOutcome {
            curve: None,
            deviation: f64::INFINITY,
        });
    }
    let parameters: Vec<f64> = (0..FIT_SAMPLES)
        .map(|index| s0 + (s1 - s0) * index as f64 / (FIT_SAMPLES - 1) as f64)
        .collect();
    let samples = sample_uv(
        stored_surface,
        emitted_surface,
        emitted_curve,
        pcurve,
        &parameters,
    )?;

    let mut best_rejected = f64::INFINITY;
    let mut accept = |candidate: Option<Pcurve2d>| -> Option<PcurveOutcome> {
        let candidate = candidate?;
        let deviation = verify(
            stored_surface,
            emitted_surface,
            emitted_curve,
            &candidate,
            s0,
            s1,
        )
        .ok()?;
        if deviation <= tolerance {
            return Some(PcurveOutcome {
                curve: Some(candidate),
                deviation,
            });
        }
        best_rejected = best_rejected.min(deviation);
        None
    };

    // A spline on a plane has an exact 2D representation: project its
    // homogeneous controls into the emitted orthonormal frame. Preserve
    // knots and weights instead of replacing shallow curvature by a line
    // merely because it fits inside the export sewing band.
    if let (EmittedSurface::Plane { origin, x_axis, y_axis },
            EmittedCurve::Spline { curve }) = (emitted_surface, emitted_curve) {
        let controls = curve.control_points.iter().map(|p| {
            let offset = Vec3::new(p.x - origin.x * p.w,
                                   p.y - origin.y * p.w,
                                   p.z - origin.z * p.w);
            crate::Vec4 { x: offset.dot(*x_axis), y: offset.dot(*y_axis), z: 0.0, w: p.w }
        }).collect();
        let projected = NurbsCurve::new(curve.degree, curve.knots.clone(), controls)?;
        if let Some(outcome) = accept(Some(Pcurve2d::Spline(projected))) {
            return Ok(outcome);
        }
    }

    // Most exact first: a 2D line shares the emitted 3D entity's parameter
    // EXACTLY and keeps analytic exports free of B_SPLINE entities.  It covers
    // every straight edge on a plane and every iso-line on a revolution —
    // rulings, meridians, rims, seams — which is nearly all of an analytic
    // solid's edges.
    if let Some(points) = samples.as_ref() {
        if let Some(outcome) = accept(line_candidate(points, s0, s1)) {
            return Ok(outcome);
        }
    }
    // A rim circle on a plane stays a circle, with the same angular parameter.
    if let Some(outcome) = accept(circle_candidate(emitted_surface, emitted_curve)) {
        return Ok(outcome);
    }
    // A B-spline carrier's emitted UV IS its stored UV, so the stored net
    // transfers with nothing but an affine knot remap — exact wherever the
    // kernel built the pcurve at curve speed. It is NOT automatically inside
    // the band everywhere else: the export gate accepts a stored pcurve at
    // `pcurve_acceptance(diagonal)` = max(pcurve_consistency, 2.5% of the model
    // diagonal), while this emits at the raw `export_knit`, which is tighter on
    // any model bigger than a fraction of a millimetre. That is the right way
    // round — a sloppy stored pcurve is never promoted into the file — and the
    // gate below is what enforces it.
    if let Some(outcome) = accept(stored_net_candidate(
        emitted_surface,
        emitted_curve,
        pcurve,
    )?) {
        return Ok(outcome);
    }
    // Everything left is an edge that is neither an iso-line nor a rim: a
    // boolean intersection curve, a fillet spine, a rim on a B-spline carrier
    // whose emitted 3D entity is angle-parameterized. Its exact 2D image has no
    // closed form, so it is fitted through the inverted samples, refined until
    // it is comfortably inside the band rather than merely inside it, and then
    // held to the same gate as everything else.
    let mut best: Option<(Pcurve2d, f64)> = None;
    for &count in &REFINEMENT_SAMPLES {
        let dense: Vec<f64> = (0..count)
            .map(|index| s0 + (s1 - s0) * index as f64 / (count - 1) as f64)
            .collect();
        let Some(points) = sample_uv(
            stored_surface,
            emitted_surface,
            emitted_curve,
            pcurve,
            &dense,
        )?
        else {
            break;
        };
        let Some(candidate) = fitted_candidate(&points, &dense)? else {
            break;
        };
        let deviation = verify(
            stored_surface,
            emitted_surface,
            emitted_curve,
            &candidate,
            s0,
            s1,
        )?;
        let improved = best.as_ref().is_none_or(|(_, best)| deviation < *best);
        if improved {
            best = Some((candidate, deviation));
        }
        if deviation <= tolerance * REFINEMENT_TARGET {
            break;
        }
    }
    if let Some((candidate, deviation)) = best {
        if deviation <= tolerance {
            return Ok(PcurveOutcome {
                curve: Some(candidate),
                deviation,
            });
        }
        best_rejected = best_rejected.min(deviation);
    }

    Ok(PcurveOutcome {
        curve: None,
        deviation: best_rejected,
    })
}

/// The emitted (u,v) of every sample of the emitted 3D curve, periodic
/// coordinates unwrapped along the curve and anchored to the branch the STORED
/// pcurve sits on — which is what puts the two halves of a seam on opposite
/// domain boundaries instead of on top of each other.
///
/// `None` when the carrier has no closed-form inverse (the pcurve then has to
/// come from the stored net instead).
fn sample_uv(
    stored_surface: &NurbsSurface,
    emitted_surface: &EmittedSurface,
    emitted_curve: &EmittedCurve,
    pcurve: &NurbsCurve,
    parameters: &[f64],
) -> Result<Option<Vec<(f64, f64)>>, String> {
    if matches!(emitted_surface, EmittedSurface::Spline) {
        // Emitted UV IS stored UV here, so there is nothing to invert: read the
        // stored pcurve at the fraction that matches each emitted parameter.
        // For an emitted `CIRCLE` that fraction is the tangent-half-angle
        // inverse, which is what lets a rim on a B-spline carrier be fitted
        // exactly instead of refused.
        let [q0, q1] = pcurve.domain()?;
        let mut points = Vec::with_capacity(parameters.len());
        for &parameter in parameters {
            let fraction = emitted_curve.fraction_at(parameter)?;
            let uv = pcurve.evaluate(q0 + (q1 - q0) * fraction)?;
            points.push((uv.x, uv.y));
        }
        return Ok(Some(points));
    }
    let mut azimuths: Vec<Option<f64>> = Vec::with_capacity(parameters.len());
    let mut others: Vec<f64> = Vec::with_capacity(parameters.len());
    for &parameter in parameters {
        let point = emitted_curve.evaluate(parameter)?;
        let Some((azimuth, other)) = emitted_surface.invert(point) else {
            return Ok(None);
        };
        azimuths.push(azimuth);
        others.push(other);
    }
    // A curve entirely on the axis is a pole point, not a pcurve.
    if azimuths.iter().all(Option::is_none) {
        return Ok(None);
    }
    // Fill the azimuth of on-axis samples (a sphere pole) from the nearest
    // sample that has one: the surface collapses there, so every value maps to
    // the same point and the verify gate still proves the result.
    let mut us: Vec<f64> = Vec::with_capacity(azimuths.len());
    for index in 0..azimuths.len() {
        let filled = azimuths[index].or_else(|| {
            (1..azimuths.len())
                .flat_map(|offset| [index.checked_sub(offset), index.checked_add(offset)])
                .flatten()
                .filter(|probe| *probe < azimuths.len())
                .find_map(|probe| azimuths[probe])
        });
        us.push(filled.ok_or("export_step: pcurve sample has no resolvable azimuth")?);
    }
    let (u_period, v_period) = emitted_surface.periods();
    unwrap_in_place(&mut us, u_period);
    unwrap_in_place(&mut others, v_period);
    anchor_to_stored(
        stored_surface,
        emitted_surface,
        emitted_curve,
        pcurve,
        parameters,
        &mut us,
        &mut others,
    )?;
    Ok(Some(us.into_iter().zip(others).collect()))
}

/// Make a sampled periodic coordinate continuous by moving each value into the
/// period nearest its predecessor, so a rim circle reads as `θ₀ → θ₀ + 2π`
/// rather than as a jump back through the branch cut.
fn unwrap_in_place(values: &mut [f64], period: Option<f64>) {
    let Some(period) = period else {
        return;
    };
    for index in 1..values.len() {
        let step = values[index] - values[index - 1];
        values[index] -= period * (step / period).round();
    }
}

/// Slide an unwrapped periodic coordinate by whole periods onto the branch the
/// STORED pcurve occupies.  Without this the two halves of a seam — which are
/// geometrically the same curve — would both land on the same domain boundary
/// and the seam would collapse.
///
/// The stored parameter is a rational-arc image of the angle rather than the
/// angle itself, so `period · (stored − domain start) / domain span` is only an
/// approximation of the emitted angle.  It does not need to be better: the
/// tangent-half-angle parameterization of a 90° span deviates from linear by at
/// most ~0.9°, and all this has to do is pick the right multiple of 2π.
fn anchor_to_stored(
    stored_surface: &NurbsSurface,
    emitted_surface: &EmittedSurface,
    emitted_curve: &EmittedCurve,
    pcurve: &NurbsCurve,
    parameters: &[f64],
    us: &mut [f64],
    vs: &mut [f64],
) -> Result<(), String> {
    let (u_period, v_period) = emitted_surface.periods();
    if u_period.is_none() && v_period.is_none() {
        return Ok(());
    }
    let sign = emitted_surface
        .frame()
        .map(|frame| frame.azimuth_sign)
        .unwrap_or(1.0);
    let [stored_u0, stored_u1] = stored_surface.domain_u()?;
    let [stored_v0, stored_v1] = stored_surface.domain_v()?;
    let [q0, q1] = pcurve.domain()?;
    let mut u_offset = 0.0;
    let mut v_offset = 0.0;
    for (index, &parameter) in parameters.iter().enumerate() {
        // Fraction-of-domain is the kernel's own edge/pcurve synchronisation.
        let fraction = emitted_curve.fraction_at(parameter)?;
        let stored = pcurve.evaluate(q0 + (q1 - q0) * fraction)?;
        if let Some(period) = u_period {
            u_offset += sign * period * (stored.x - stored_u0) / (stored_u1 - stored_u0) - us[index];
        }
        if let Some(period) = v_period {
            v_offset += period * (stored.y - stored_v0) / (stored_v1 - stored_v0) - vs[index];
        }
    }
    let count = parameters.len().max(1) as f64;
    if let Some(period) = u_period {
        let shift = period * ((u_offset / count) / period).round();
        for value in us.iter_mut() {
            *value += shift;
        }
    }
    if let Some(period) = v_period {
        let shift = period * ((v_offset / count) / period).round();
        for value in vs.iter_mut() {
            *value += shift;
        }
    }
    Ok(())
}

/// The straight 2D segment through the first and last inverted sample,
/// parameterized so `P(s)` tracks the emitted 3D parameter exactly.  Exact for
/// every iso-line (cylinder/cone ruling, sphere meridian, torus seam, rim
/// circle on a revolution) and for every straight edge on a plane; the verify
/// gate rejects it for everything else.
fn line_candidate(points: &[(f64, f64)], s0: f64, s1: f64) -> Option<Pcurve2d> {
    let (u0, v0) = *points.first()?;
    let (u1, v1) = *points.last()?;
    let span = s1 - s0;
    if span.abs() <= f64::EPSILON {
        return None;
    }
    let vector = [(u1 - u0) / span, (v1 - v0) / span];
    let magnitude = vector[0].hypot(vector[1]);
    let scale = 1.0 + u0.abs().max(v0.abs());
    if magnitude <= 1e-12 * scale {
        return None;
    }
    Some(Pcurve2d::Line {
        point: [u0 - s0 * vector[0], v0 - s0 * vector[1]],
        vector,
    })
}

/// A rim circle on a PLANE is a circle in that plane's parameters too, with the
/// same angular parameter: the 2D placement's `ref_direction` is the UV image
/// of the 3D circle's own `ref_direction`, so parameter = angle on both sides.
///
/// A 2D `AXIS2_PLACEMENT_2D` is right-handed by definition, so a circle running
/// CLOCKWISE in the plane's UV (its axis opposing the plane normal) cannot be
/// expressed this way at all.  Nothing here tests for that: the proposal is
/// simply built and the verify gate finds it half a plane away and refuses it,
/// which is the same fail-safe path every other rejected candidate takes.
fn circle_candidate(
    emitted_surface: &EmittedSurface,
    emitted_curve: &EmittedCurve,
) -> Option<Pcurve2d> {
    let EmittedSurface::Plane {
        origin,
        x_axis,
        y_axis,
    } = emitted_surface
    else {
        return None;
    };
    let EmittedCurve::Circle {
        center,
        x_axis: arc_x_axis,
        radius,
        ..
    } = emitted_curve
    else {
        return None;
    };
    let offset = center.sub(*origin);
    Some(Pcurve2d::Circle {
        center: [offset.dot(*x_axis), offset.dot(*y_axis)],
        ref_direction: [arc_x_axis.dot(*x_axis), arc_x_axis.dot(*y_axis)],
        radius: *radius,
    })
}

/// The stored coedge net, knots remapped affinely onto the emitted 3D curve's
/// own parameter range.  Only offered for a B-spline carrier (emitted UV IS
/// stored UV) and an emitted curve whose STEP parameter is affine in the
/// kernel's fraction — for a recognized `CIRCLE` the two parameterizations
/// differ by the tangent-half-angle map and a remap would be positionally
/// wrong, so that pair is refused here and omitted.
fn stored_net_candidate(
    emitted_surface: &EmittedSurface,
    emitted_curve: &EmittedCurve,
    pcurve: &NurbsCurve,
) -> Result<Option<Pcurve2d>, String> {
    if !matches!(emitted_surface, EmittedSurface::Spline)
        || !emitted_curve.parameter_is_affine_in_fraction()
    {
        return Ok(None);
    }
    let [s0, s1] = emitted_curve.domain()?;
    let [q0, q1] = pcurve.domain()?;
    if (q1 - q0).abs() <= f64::EPSILON || (s1 - s0).abs() <= f64::EPSILON {
        return Ok(None);
    }
    let knots = pcurve
        .knots
        .iter()
        .map(|knot| s0 + (s1 - s0) * (knot - q0) / (q1 - q0))
        .collect();
    Ok(Some(Pcurve2d::Spline(NurbsCurve::new(
        pcurve.degree,
        knots,
        pcurve.control_points.clone(),
    )?)))
}

/// Cubic interpolation through the inverted samples, parameterized by the
/// emitted 3D parameter itself so the two representations stay in step by
/// construction rather than by luck.  33 samples is the seed count of
/// `adaptive_coedge_error`, the validator that already graded the stored pcurve
/// this one is derived from; the verify gate then re-checks the result on 97
/// DIFFERENT parameters, so a fit that only agrees where it interpolates is
/// caught and omitted.
fn fitted_candidate(
    points: &[(f64, f64)],
    parameters: &[f64],
) -> Result<Option<Pcurve2d>, String> {
    if points.len() != parameters.len() || points.len() < 2 {
        return Ok(None);
    }
    let coordinates: Vec<Vec3> = points
        .iter()
        .map(|(u, v)| Vec3::new(*u, *v, 0.0))
        .collect();
    Ok(interpolate_curve(&coordinates, 3, parameters)
        .ok()
        .map(Pcurve2d::Spline))
}

/// The verify-or-omit gate: max ‖S(c₂d(s)) − c₃d(s)‖ over a dense sample,
/// measured with the ISO 10303-42 evaluators of the entities we are actually
/// about to write — not with the kernel's own carriers, whose parameterization
/// is precisely what differs.
fn verify(
    stored_surface: &NurbsSurface,
    emitted_surface: &EmittedSurface,
    emitted_curve: &EmittedCurve,
    candidate: &Pcurve2d,
    s0: f64,
    s1: f64,
) -> Result<f64, String> {
    let mut worst: f64 = 0.0;
    for index in 0..VERIFY_SAMPLES {
        // Offset off the endpoints and off the builder's own sample grid, so a
        // candidate is graded where it was NOT pinned.
        let fraction = (index as f64 + 0.37) / VERIFY_SAMPLES as f64;
        let parameter = s0 + (s1 - s0) * fraction;
        let (u, v) = candidate.evaluate(parameter)?;
        let on_surface = emitted_surface.evaluate(stored_surface, u, v)?;
        let on_curve = emitted_curve.evaluate(parameter)?;
        worst = worst.max(on_surface.sub(on_curve).length());
    }
    Ok(worst)
}


