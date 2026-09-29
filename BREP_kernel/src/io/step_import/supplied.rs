//! Supplied pcurves: the vendor's own statement of where an edge lies in a
//! face's parameter space, read from `SURFACE_CURVE` / `SEAM_CURVE` bundles.
//!
//! The importer's default is to DERIVE a pcurve by projecting the edge's 3D
//! curve onto the carrier. That answer is ours, not the file's, and the two
//! differ wherever the vendor's 3D curve is an approximation of its own trim —
//! which is the normal case, because ISO 10303-42 lets a `SURFACE_CURVE`
//! declare `.PCURVE_S1.`/`.PCURVE_S2.` as the MASTER representation and treat
//! the written 3D curve as the approximation.
//!
//! ## Why the supplied 2D net cannot simply be seated
//!
//! A supplied pcurve is expressed in the STEP ENTITY's parameterization: a
//! cylinder's u is an angle, a plane's (u,v) are frame coordinates in the
//! file's length unit. The kernel's carrier for that same face is a NURBS whose
//! periodic direction is a RATIONAL-ARC parameter and whose meridians are
//! 90°-span rational arcs. No control-net transform relates the two — the same
//! obstruction the WRITER documents in `io/step/pcurve.rs`, which is why the
//! writer derives its 2D curve in the emitted entity's parameterization rather
//! than remapping the stored one.
//!
//! So this module runs the writer's recipe backwards:
//!
//! 1. rebuild the STEP surface entity in ISO 10303-42's own parameterization
//!    ([`StepSurface`]) — the only parameterization the supplied 2D curve is
//!    expressed in;
//! 2. for each station along the edge, invert the edge point through that
//!    entity to `(u, v)`, then SNAP it onto the supplied 2D curve;
//! 3. map the snapped `(u, v)` back to 3D through the entity, giving a station
//!    that lies exactly on the vendor's stated trim;
//! 4. fit the kernel pcurve through those stations on the kernel's carrier, and
//!    VERIFY the result against the kernel's own coedge-image measurement —
//!    declining to the projection fit when it does not hold.
//!
//! Step 2 is where the supplied data does its work, and it is worth being
//! exact about how little that is. The station handed to the fitter moves from
//! the vendor's 3D curve onto the vendor's 2D one; everything after that —
//! inversion onto the kernel carrier, branch repair, refinement — is the
//! derived lane's own code, unchanged. So this does NOT resolve a seam branch
//! or a pole: `build_pcurve_on_surface_stations` inverts whatever 3D point it
//! is given and repairs jumps exactly as before, and the snapped point sits at
//! the same place on the surface whichever branch it was snapped on.
//!
//! What it does resolve is a point that is not ON the surface. A vendor's 3D
//! curve is a fitted approximation wherever the true intersection is not an
//! analytic curve, so projecting it answers "which point of the carrier is
//! nearest this nearby point" — and that answer inherits the fit's error. The
//! supplied 2D curve lies on the carrier by construction, so the snapped
//! station carries none of it.
//!
//! Measured, so the claim is not larger than the evidence. On the two vendor
//! files in the corpus that supply pcurves at all, the snap moves a station by
//! at most 9.5e-8 mm and 7.2e-5 mm — those producers' 3D curves already agree
//! with their own pcurves. On `supplied-pcurve-master-3d-approximate.step`,
//! constructed by pushing one cylinder/cylinder intersection's 3D spline 0.05
//! mm off the surfaces it bounds, the snap moves stations 1.6e-3 mm and
//! recovers all but 6% of the 0.149 mm³ volume error the derived lane inherits.
//!
//! What this preserves is the supplied trim's IMAGE — the locus in parameter
//! space — to a measured, bounded residual. It does not preserve the supplied
//! net or its parameterization, and this module never claims to: the kernel's
//! carrier cannot carry either.
//!
//! ## What this slice cannot do
//!
//! The fitted pcurve is verified against the edge's own 3D curve inside
//! `pcurve_consistency` (4e-3 mm at model scale), because the 3D curve is the
//! geometry of record and this slice does not move it. That band is also the
//! ceiling on what the lane can change: a file whose pcurve genuinely IS the
//! master, with a 3D curve coarse enough to disagree by more than the band, is
//! DECLINED rather than corrected — the constructed fixture above declines on
//! exactly the face whose 3D curve left the carrier. Honouring such a file
//! means replacing the edge's 3D curve with the supplied trim's image, which
//! is a separate change with its own consequences for every consumer of
//! `EdgeRecord::curve`.
//!
//! `master_representation` is read by nobody here, and deliberately: every
//! `SURFACE_CURVE` in both vendor files declares `.PCURVE_S1.`, including the
//! ones whose 3D curve is an exact `LINE` and whose pcurves are exact 2D
//! `LINE`s. For these producers the field is boilerplate, so gating on it
//! would add a branch no fixture exercises while changing nothing.

use super::*;

/// Radians per unit of the file's `PLANE_ANGLE_UNIT`, or `None` when the file
/// declares no resolvable plane-angle unit.
///
/// A supplied pcurve on a cylinder, cone, sphere or torus carries ANGLES in its
/// u (and, on a torus, v) coordinate. Unlike a cone's `semi_angle` — which
/// `analytic_surface` can disambiguate by range, since a semi-angle cannot
/// reach pi/2 — a pcurve angle admits no range heuristic: 1.5 is a legal value
/// in radians and in degrees alike. An unreadable unit therefore DECLINES every
/// angular supplied pcurve rather than guessing.
pub(super) fn derive_angle_scale(entities: &HashMap<usize, Entity>) -> Option<f64> {
    let resolve = |id: usize, depth: usize| -> Option<f64> { unit_angle_scale(entities, id, depth) };
    for entity in entities.values() {
        if !entity.has("GLOBAL_UNIT_ASSIGNED_CONTEXT") {
            continue;
        }
        let Some(args) = entity.find("GLOBAL_UNIT_ASSIGNED_CONTEXT") else {
            continue;
        };
        let Some(units) = args.first().and_then(|value| value.as_list().ok()) else {
            continue;
        };
        if let Some(scale) = units
            .iter()
            .filter_map(|unit| unit.as_ref_id().ok())
            .find_map(|unit_id| resolve(unit_id, 0))
        {
            return Some(scale);
        }
    }
    for (&id, entity) in entities {
        if entity.has("PLANE_ANGLE_UNIT") {
            if let Some(scale) = resolve(id, 0) {
                return Some(scale);
            }
        }
    }
    None
}

/// Radians per one of the given unit entity (an entity carrying
/// `PLANE_ANGLE_UNIT`): `SI_UNIT(prefix, .RADIAN.)` → the prefix factor;
/// `CONVERSION_BASED_UNIT(name, #measure)` → measure × its base unit's scale
/// (which is how every producer writes degrees).
fn unit_angle_scale(
    entities: &HashMap<usize, Entity>,
    id: usize,
    depth: usize,
) -> Option<f64> {
    if depth > 4 {
        return None;
    }
    let entity = entities.get(&id)?;
    if !entity.has("PLANE_ANGLE_UNIT") {
        return None;
    }
    if let Some(args) = entity.find("SI_UNIT") {
        if !args.get(1).is_some_and(|name| name.enum_is("RADIAN")) {
            return None;
        }
        let prefix = match args.first() {
            Some(Value::Enum(prefix)) => match prefix.as_str() {
                "MILLI" => 1e-3,
                "CENTI" => 1e-2,
                "DECI" => 1e-1,
                "DECA" => 1e1,
                "HECTO" => 1e2,
                "KILO" => 1e3,
                "MICRO" => 1e-6,
                "NANO" => 1e-9,
                _ => return None,
            },
            _ => 1.0,
        };
        return Some(prefix);
    }
    if let Some(args) = entity.find("CONVERSION_BASED_UNIT") {
        let measure_entity = entities.get(&args.get(1)?.as_ref_id().ok()?)?;
        let measure_args = measure_entity
            .find("PLANE_ANGLE_MEASURE_WITH_UNIT")
            .or_else(|| measure_entity.find("MEASURE_WITH_UNIT"))?;
        let value = match measure_args.first()? {
            Value::Typed(_, inner) => inner.first()?.as_real().ok()?,
            other => other.as_real().ok()?,
        };
        let base = unit_angle_scale(entities, measure_args.get(1)?.as_ref_id().ok()?, depth + 1)?;
        return Some(value * base);
    }
    None
}

// ---------------------------------------------------------------------------
// The STEP surface entity in ISO 10303-42's own parameterization
// ---------------------------------------------------------------------------

/// A STEP elementary surface evaluated in the parameterization ISO 10303-42
/// gives it — the only parameterization a supplied pcurve is expressed in.
///
/// This is deliberately NOT the kernel carrier built for the same entity by
/// `analytic_surface`: that carrier is reseamed, axially sized to the face and
/// rational-arc parameterized. Both describe the same point set; only this one
/// shares coordinates with the file's 2D curves.
#[derive(Clone, Debug)]
pub(super) enum StepSurface {
    /// `PLANE`: S(u,v) = o + u·x̂ + v·ŷ. Both coordinates are LENGTHS.
    Plane(Frame),
    /// `CYLINDRICAL_SURFACE`: S(u,v) = o + r(cos u·x̂ + sin u·ŷ) + v·ẑ.
    Cylinder { frame: Frame, radius: f64 },
    /// `CONICAL_SURFACE`: S(u,v) = o + (r + v·tan α)(cos u·x̂ + sin u·ŷ) + v·ẑ,
    /// v the AXIAL coordinate (see the writer's note on OCC's generatrix form).
    Cone {
        frame: Frame,
        radius: f64,
        semi_angle: f64,
    },
    /// `SPHERICAL_SURFACE`: S(u,v) = o + r·cos v(cos u·x̂ + sin u·ŷ) + r·sin v·ẑ,
    /// v the LATITUDE in [−π/2, π/2].
    Sphere { frame: Frame, radius: f64 },
    /// `TOROIDAL_SURFACE`: S(u,v) = o + (R + r·cos v)(cos u·x̂ + sin u·ŷ) + r·sin v·ẑ,
    /// v measured from the outer equator.
    Torus {
        frame: Frame,
        major_radius: f64,
        minor_radius: f64,
    },
}

impl StepSurface {
    /// ISO 10303-42 evaluation, in millimetres (the frame and radii were scaled
    /// on the way in, exactly as every other imported length is).
    pub(super) fn evaluate(&self, u: f64, v: f64) -> Vec3 {
        let radial = |frame: &Frame, rho: f64, along: f64| {
            frame
                .origin
                .add(frame.x.scale(rho * u.cos()))
                .add(frame.y.scale(rho * u.sin()))
                .add(frame.z.scale(along))
        };
        match self {
            StepSurface::Plane(frame) => frame.origin.add(frame.x.scale(u)).add(frame.y.scale(v)),
            StepSurface::Cylinder { frame, radius } => radial(frame, *radius, v),
            StepSurface::Cone {
                frame,
                radius,
                semi_angle,
            } => radial(frame, radius + v * semi_angle.tan(), v),
            StepSurface::Sphere { frame, radius } => {
                radial(frame, radius * v.cos(), radius * v.sin())
            }
            StepSurface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => radial(
                frame,
                major_radius + minor_radius * v.cos(),
                minor_radius * v.sin(),
            ),
        }
    }

    /// Closed-form inverse of [`StepSurface::evaluate`] for a point on (or very
    /// near) the carrier. The azimuth is `None` on the axis — a sphere pole,
    /// where every meridian ties — and the caller supplies a branch there.
    ///
    /// The returned azimuth is the PRINCIPAL value in (−π, π]; periodic
    /// unwrapping is the caller's job, because only the caller knows which
    /// branch the supplied 2D curve lives on.
    pub(super) fn invert(&self, point: Vec3) -> (Option<f64>, f64) {
        let azimuth = |frame: &Frame, offset: Vec3| {
            let x = offset.dot(frame.x);
            let y = offset.dot(frame.y);
            let scale = 1.0 + offset.length();
            (x.hypot(y) > 1e-12 * scale).then(|| y.atan2(x))
        };
        match self {
            StepSurface::Plane(frame) => {
                let offset = point.sub(frame.origin);
                (Some(offset.dot(frame.x)), offset.dot(frame.y))
            }
            StepSurface::Cylinder { frame, .. } | StepSurface::Cone { frame, .. } => {
                let offset = point.sub(frame.origin);
                (azimuth(frame, offset), offset.dot(frame.z))
            }
            StepSurface::Sphere { frame, .. } => {
                let offset = point.sub(frame.origin);
                let rho = offset.dot(frame.x).hypot(offset.dot(frame.y));
                // atan2, not asin: exact at the poles and insensitive to a
                // radius read a few ulps away from the point's true distance.
                (azimuth(frame, offset), offset.dot(frame.z).atan2(rho))
            }
            StepSurface::Torus {
                frame,
                major_radius,
                ..
            } => {
                let offset = point.sub(frame.origin);
                let rho = offset.dot(frame.x).hypot(offset.dot(frame.y));
                (
                    azimuth(frame, offset),
                    offset.dot(frame.z).atan2(rho - major_radius),
                )
            }
        }
    }

    /// Period of each parameter direction, `None` where the parameter is
    /// single-valued (a plane's coordinates, a cylinder/cone height, a sphere's
    /// latitude).
    fn periods(&self) -> (Option<f64>, Option<f64>) {
        match self {
            StepSurface::Plane(_) => (None, None),
            StepSurface::Cylinder { .. } | StepSurface::Cone { .. } | StepSurface::Sphere { .. } => {
                (Some(TAU), None)
            }
            StepSurface::Torus { .. } => (Some(TAU), Some(TAU)),
        }
    }

    /// Multipliers taking the file's raw 2D coordinates into this surface's
    /// evaluation units: millimetres where the parameter is a length, radians
    /// where it is an angle.
    fn parameter_scales(&self, length_scale: f64, angle_scale: f64) -> (f64, f64) {
        match self {
            StepSurface::Plane(_) => (length_scale, length_scale),
            StepSurface::Cylinder { .. } | StepSurface::Cone { .. } => (angle_scale, length_scale),
            StepSurface::Sphere { .. } | StepSurface::Torus { .. } => (angle_scale, angle_scale),
        }
    }

    /// Whether either parameter direction is angular, i.e. whether reading this
    /// surface's supplied pcurves needs the file's plane-angle unit at all.
    fn needs_angle_unit(&self) -> bool {
        !matches!(self, StepSurface::Plane(_))
    }

    /// Millimetres of surface travelled per unit of each parameter, near the
    /// trim represented by `(u, v)`.
    ///
    /// Without this the nearest-point search that snaps an inverted station
    /// onto the supplied curve would minimise `√(Δu² + Δv²)` across a RADIAN
    /// and a MILLIMETRE — a metric that weights a 200 mm cylinder's angular
    /// direction two hundred times too lightly, so the snap slides along the
    /// trim and the fit it feeds is declined on a residual that is the metric's
    /// fault, not the file's. Multiplying both axes by this makes the search
    /// isotropic in millimetres, which is the space the verification band is
    /// stated in.
    fn parameter_metric(&self, v: f64) -> [f64; 2] {
        match self {
            StepSurface::Plane(_) => [1.0, 1.0],
            StepSurface::Cylinder { radius, .. } => [radius.abs().max(f64::MIN_POSITIVE), 1.0],
            StepSurface::Cone {
                radius, semi_angle, ..
            } => [
                (radius + v * semi_angle.tan())
                    .abs()
                    .max(f64::MIN_POSITIVE),
                // v is the AXIAL coordinate, so a unit of v travels
                // `1/cos α` along the generatrix.
                (1.0 / semi_angle.cos()).abs().max(f64::MIN_POSITIVE),
            ],
            StepSurface::Sphere { radius, .. } => [
                (radius * v.cos()).abs().max(f64::MIN_POSITIVE),
                radius.abs().max(f64::MIN_POSITIVE),
            ],
            StepSurface::Torus {
                major_radius,
                minor_radius,
                ..
            } => [
                (major_radius + minor_radius * v.cos())
                    .abs()
                    .max(f64::MIN_POSITIVE),
                minor_radius.abs().max(f64::MIN_POSITIVE),
            ],
        }
    }
}

// ---------------------------------------------------------------------------
// The supplied 2D curve
// ---------------------------------------------------------------------------

/// A supplied pcurve's 2D geometry, in the basis surface's own parameters.
///
/// `Line` is kept symbolic because ISO 10303-42 lines are UNBOUNDED: a
/// `NurbsCurve` would need a domain invented for it, and the nearest-point
/// answer a fabricated domain gives is a clamp, not a projection.
#[derive(Clone, Debug)]
enum Curve2d {
    Line { point: [f64; 2], direction: [f64; 2] },
    Bounded(NurbsCurve),
}

impl Curve2d {
    fn evaluate(&self, s: f64) -> Result<[f64; 2], String> {
        Ok(match self {
            Curve2d::Line { point, direction } => {
                [point[0] + s * direction[0], point[1] + s * direction[1]]
            }
            Curve2d::Bounded(curve) => {
                let value = curve.evaluate(s)?;
                [value.x, value.y]
            }
        })
    }

    /// Parameter of the point on this curve nearest `target`.
    ///
    /// A closed 2D curve — a vendor's plain circle, which is the commonest
    /// supplied pcurve there is — used to need a wrapped nearest-point search
    /// of this module's own, because `project_point_to_curve` clamped its
    /// Newton walk to `[start, end]` and returned the seam, 4.82 mm out on a
    /// 20 mm circle. That is fixed at source now: the shared projector wraps
    /// when the curve is closed, and this asks it directly again.
    fn nearest(&self, target: [f64; 2]) -> Result<f64, String> {
        Ok(match self {
            Curve2d::Line { point, direction } => {
                let length_squared = direction[0] * direction[0] + direction[1] * direction[1];
                if length_squared <= 0.0 {
                    return Err("step_import: degenerate supplied 2D line".into());
                }
                ((target[0] - point[0]) * direction[0] + (target[1] - point[1]) * direction[1])
                    / length_squared
            }
            Curve2d::Bounded(curve) => {
                crate::project_point_to_curve(curve, Vec3::new(target[0], target[1], 0.0))?.u
            }
        })
    }

    /// Apply the per-axis unit scale. Both forms are exact under a diagonal
    /// affine map: a NURBS control net transforms with the map, and a rational
    /// conic stays a rational conic (an ellipse is the affine image of one).
    fn scaled(self, u_scale: f64, v_scale: f64) -> Result<Self, String> {
        if u_scale == 1.0 && v_scale == 1.0 {
            return Ok(self);
        }
        Ok(match self {
            Curve2d::Line { point, direction } => Curve2d::Line {
                point: [point[0] * u_scale, point[1] * v_scale],
                direction: [direction[0] * u_scale, direction[1] * v_scale],
            },
            Curve2d::Bounded(curve) => {
                let control_points = curve
                    .control_points
                    .iter()
                    .map(|control| {
                        let weight = control.w;
                        let point = control.point()?;
                        Ok(Vec4::from_point(
                            Vec3::new(point.x * u_scale, point.y * v_scale, 0.0),
                            weight,
                        ))
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                Curve2d::Bounded(NurbsCurve::new(
                    curve.degree,
                    curve.knots.clone(),
                    control_points,
                )?)
            }
        })
    }
}

/// One `PCURVE` of a `SURFACE_CURVE`/`SEAM_CURVE` bundle: the basis surface in
/// its own parameterization, plus the 2D curve expressed in those parameters.
#[derive(Clone, Debug)]
pub(super) struct SuppliedPcurve {
    /// The `basis_surface` entity id, which is how a face claims its own branch
    /// out of the bundle (a `SEAM_CURVE` names the SAME surface twice).
    pub(super) basis_ref: usize,
    pub(super) surface: StepSurface,
    curve: Curve2d,
    /// The same curve with both axes scaled to millimetres, used ONLY for the
    /// nearest-point search. An affine scale leaves the knot vector alone, so
    /// the parameter it reports indexes `curve` exactly.
    metric_curve: Curve2d,
    metric: [f64; 2],
}

impl SuppliedPcurve {
    /// The point of the vendor's stated trim nearest `point`, and the `(u, v)`
    /// it sits at, branch-continued from `previous` where the surface is
    /// periodic.
    ///
    /// `previous` carries the last accepted `(u, v)`; the azimuth (and, on a
    /// torus, the minor angle) is lifted to the branch nearest it before the
    /// snap, which is how a supplied curve that runs OUTSIDE the canonical
    /// period — OCC writes cone rims at u ∈ [−2.09, 8.38] — is read on the
    /// branch the vendor actually meant.
    pub(super) fn snap(
        &self,
        point: Vec3,
        previous: Option<(f64, f64)>,
    ) -> Result<((f64, f64), Vec3), String> {
        let (azimuth, v_raw) = self.surface.invert(point);
        let (u_period, v_period) = self.surface.periods();
        let mut u = match azimuth {
            Some(value) => value,
            // On the axis every meridian ties: carry the previous branch
            // rather than inventing one.
            None => previous.map(|(u, _)| u).unwrap_or(0.0),
        };
        let mut v = v_raw;
        if let (Some(period), Some((previous_u, _))) = (u_period, previous) {
            u = lift_to_branch(u, previous_u, period);
        }
        if let (Some(period), Some((_, previous_v))) = (v_period, previous) {
            v = lift_to_branch(v, previous_v, period);
        }
        // With no cursor yet, seat the first station on the branch the supplied
        // curve itself occupies: its own start is the vendor's declaration of
        // which period this trim lives in.
        if previous.is_none() {
            let seed = self.seed_uv()?;
            if let Some(period) = u_period {
                u = lift_to_branch(u, seed[0], period);
            }
            if let Some(period) = v_period {
                v = lift_to_branch(v, seed[1], period);
            }
        }
        let s = self
            .metric_curve
            .nearest([u * self.metric[0], v * self.metric[1]])?;
        let snapped = self.curve.evaluate(s)?;
        Ok((
            (snapped[0], snapped[1]),
            self.surface.evaluate(snapped[0], snapped[1]),
        ))
    }

    /// How much of the surface's own parameter domain this trim spans, per
    /// direction: the sampled `(u, v)` extent divided by the surface's period,
    /// or `None` where that direction has no period (a plane's coordinates, a
    /// cylinder height, a sphere's latitude) or the curve has no domain to
    /// sample (the unbounded `Line` form).
    ///
    /// Read as a measurement only. A vendor that writes a cone rim at
    /// u in [-2.09, 8.38] states a span of 1.67 periods for a curve that closes
    /// in one, which is a visible fact about its parameterization and a
    /// candidate for telling one producer's habits from another's.
    pub(super) fn span_in_periods(&self) -> (Option<f64>, Option<f64>) {
        let Curve2d::Bounded(curve) = &self.curve else {
            return (None, None);
        };
        let Ok([start, end]) = curve.domain() else {
            return (None, None);
        };
        let samples = 64usize;
        let (mut min_u, mut max_u) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut min_v, mut max_v) = (f64::INFINITY, f64::NEG_INFINITY);
        for index in 0..=samples {
            let s = start + (end - start) * index as f64 / samples as f64;
            let Ok([u, v]) = self.curve.evaluate(s) else {
                return (None, None);
            };
            min_u = min_u.min(u);
            max_u = max_u.max(u);
            min_v = min_v.min(v);
            max_v = max_v.max(v);
        }
        let (u_period, v_period) = self.surface.periods();
        (
            u_period.map(|period| (max_u - min_u) / period),
            v_period.map(|period| (max_v - min_v) / period),
        )
    }

    /// A representative `(u, v)` on the supplied curve, used to choose the
    /// periodic branch for the first station.
    fn seed_uv(&self) -> Result<[f64; 2], String> {
        match &self.curve {
            Curve2d::Line { point, .. } => Ok(*point),
            Curve2d::Bounded(curve) => {
                let [start, end] = curve.domain()?;
                self.curve.evaluate(0.5 * (start + end))
            }
        }
    }
}

/// The mean v of a 2D curve's control net — a cheap stand-in for "where on the
/// surface this trim sits", which is all the metric needs.
///
/// For the unbounded `Line` form this is the v of its base point rather than a
/// mean, because the line has no domain to average over. That is only ever
/// approximate on a cone or torus, where the metric genuinely varies with v;
/// it costs nothing, because the metric only weights the two axes against each
/// other in the nearest-point search and never moves the foot it finds.
fn mean_v(curve: &Curve2d) -> Option<f64> {
    match curve {
        Curve2d::Line { point, .. } => Some(point[1]),
        Curve2d::Bounded(curve) => {
            let mut total = 0.0;
            for control in &curve.control_points {
                total += control.point().ok()?.y;
            }
            Some(total / curve.control_points.len().max(1) as f64)
        }
    }
}

/// `value` shifted by whole periods onto the branch nearest `reference`.
fn lift_to_branch(value: f64, reference: f64, period: f64) -> f64 {
    if !(period > 0.0) {
        return value;
    }
    value + period * ((reference - value) / period).round()
}

// ---------------------------------------------------------------------------
// Reading the bundle
// ---------------------------------------------------------------------------

impl<'a> Resolver<'a> {
    /// Every `PCURVE` a curve-on-surface bundle supplies for one edge, in the
    /// file's order (which is the order a `SEAM_CURVE`'s two branches are
    /// declared in).
    ///
    /// Returns an EMPTY vector rather than an error whenever anything cannot be
    /// read: a supplied pcurve is an optimisation over the projection fit that
    /// the importer performs anyway, so an unreadable one must never turn a file
    /// that imports today into a refusal.
    pub(super) fn supplied_pcurves(&self, curve_ref: usize) -> Vec<SuppliedPcurve> {
        let Some(angle_scale) = self.angle_scale() else {
            // Only angular surfaces actually need it, but the decision has to be
            // made per basis surface — see `read_pcurve`.
            return self.read_bundle(curve_ref, None);
        };
        self.read_bundle(curve_ref, Some(angle_scale))
    }

    fn read_bundle(&self, curve_ref: usize, angle_scale: Option<f64>) -> Vec<SuppliedPcurve> {
        let Ok(entity) = self.get(curve_ref) else {
            return Vec::new();
        };
        let mut supplied = Vec::new();
        for wrapper in ["SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"] {
            let Some(args) = entity.find(wrapper) else {
                continue;
            };
            let Some(associated) = args.get(2).and_then(|value| value.as_list().ok()) else {
                continue;
            };
            for value in associated {
                let Ok(pcurve_ref) = value.as_ref_id() else {
                    continue;
                };
                if let Some(pcurve) = self.read_pcurve(pcurve_ref, angle_scale) {
                    supplied.push(pcurve);
                }
            }
            break;
        }
        supplied
    }

    /// One `PCURVE(name, basis_surface, DEFINITIONAL_REPRESENTATION)`.
    fn read_pcurve(&self, id: usize, angle_scale: Option<f64>) -> Option<SuppliedPcurve> {
        let entity = self.get(id).ok()?;
        let args = entity.find("PCURVE")?;
        let basis_ref = args.get(1)?.as_ref_id().ok()?;
        let representation_ref = args.get(2)?.as_ref_id().ok()?;
        let surface = self.step_surface(basis_ref)?;
        // An angular parameterization with no readable plane-angle unit has no
        // range heuristic that could recover it. Decline rather than guess.
        let angle_scale = match (surface.needs_angle_unit(), angle_scale) {
            (true, None) => return None,
            (_, scale) => scale.unwrap_or(1.0),
        };
        let representation = self.get(representation_ref).ok()?;
        let items = representation
            .find("DEFINITIONAL_REPRESENTATION")
            .or_else(|| representation.find("REPRESENTATION"))?
            .get(1)?
            .as_list()
            .ok()?;
        let curve_ref = items.first()?.as_ref_id().ok()?;
        let raw = self.read_curve2d(curve_ref)?;
        let (u_scale, v_scale) = surface.parameter_scales(self.length_scale, angle_scale);
        let curve = raw.scaled(u_scale, v_scale).ok()?;
        // One metric for the whole trim, taken at its own mean v: a supplied
        // pcurve is one edge of one face, so the surface's scale factors barely
        // move along it, and a per-station metric would make the search's own
        // geometry depend on where the search started.
        let metric = surface.parameter_metric(mean_v(&curve).unwrap_or(0.0));
        let metric_curve = curve.clone().scaled(metric[0], metric[1]).ok()?;
        Some(SuppliedPcurve {
            basis_ref,
            surface,
            curve,
            metric_curve,
            metric,
        })
    }

    /// A STEP elementary surface in ISO 10303-42's own parameterization, with
    /// every length converted to millimetres. `None` for any surface whose
    /// parameterization this module does not claim to know — a B-spline
    /// (whose supplied pcurve is in the stored net's parameters, which our
    /// carrier may have reparameterized), an offset, a swept or revolved
    /// carrier (whose u or v is the PROFILE's parameter, not a geometric one).
    fn step_surface(&self, id: usize) -> Option<StepSurface> {
        let entity = self.get(id).ok()?;
        if let Some(args) = entity.find("PLANE") {
            return Some(StepSurface::Plane(
                self.placement(args.get(1)?.as_ref_id().ok()?).ok()?,
            ));
        }
        if let Some(args) = entity.find("CYLINDRICAL_SURFACE") {
            return Some(StepSurface::Cylinder {
                frame: self.placement(args.get(1)?.as_ref_id().ok()?).ok()?,
                radius: self.length(args.get(2)?.as_real().ok()?),
            });
        }
        if let Some(args) = entity.find("CONICAL_SURFACE") {
            let mut semi_angle = args.get(3)?.as_real().ok()?;
            // Same disambiguation `analytic_surface` applies, so the two
            // constructions agree on the same entity: a semi-angle cannot reach
            // pi/2, so a larger value was written in the file's degree unit.
            if semi_angle.abs() >= std::f64::consts::FRAC_PI_2 {
                semi_angle = semi_angle.to_radians();
            }
            return Some(StepSurface::Cone {
                frame: self.placement(args.get(1)?.as_ref_id().ok()?).ok()?,
                radius: self.length(args.get(2)?.as_real().ok()?),
                semi_angle,
            });
        }
        if let Some(args) = entity.find("SPHERICAL_SURFACE") {
            return Some(StepSurface::Sphere {
                frame: self.placement(args.get(1)?.as_ref_id().ok()?).ok()?,
                radius: self.length(args.get(2)?.as_real().ok()?),
            });
        }
        if let Some(args) = entity.find("TOROIDAL_SURFACE") {
            return Some(StepSurface::Torus {
                frame: self.placement(args.get(1)?.as_ref_id().ok()?).ok()?,
                major_radius: self.length(args.get(2)?.as_real().ok()?),
                minor_radius: self.length(args.get(3)?.as_real().ok()?),
            });
        }
        None
    }

    /// A 2D curve from a `DEFINITIONAL_REPRESENTATION`, in the file's RAW
    /// coordinates (the per-axis unit scale is applied by the caller, which is
    /// the only place that knows whether an axis is an angle or a length).
    fn read_curve2d(&self, id: usize) -> Option<Curve2d> {
        let entity = self.get(id).ok()?;
        if let Some(args) = entity.find("LINE") {
            let point = self.raw_point2d(args.get(1)?.as_ref_id().ok()?)?;
            let vector_entity = self.get(args.get(2)?.as_ref_id().ok()?).ok()?;
            // VECTOR(name, orientation, magnitude) — `find` hands back the
            // name at index 0, exactly as `Resolver::surface` reads the
            // extrusion axis. Reading from index 0 here made every 2D `LINE`
            // unreadable, which is the commonest supplied pcurve there is.
            let vector_args = vector_entity.find("VECTOR")?;
            let direction = self.raw_direction2d(vector_args.get(1)?.as_ref_id().ok()?)?;
            let magnitude = vector_args.get(2)?.as_real().ok()?;
            return Some(Curve2d::Line {
                point,
                direction: [direction[0] * magnitude, direction[1] * magnitude],
            });
        }
        if let Some(args) = entity.find("CIRCLE") {
            let frame = self.raw_frame2d(args.get(1)?.as_ref_id().ok()?)?;
            let radius = args.get(2)?.as_real().ok()?;
            return build_ellipse_arc(&frame, radius, radius, 0.0, TAU)
                .ok()
                .map(Curve2d::Bounded);
        }
        if let Some(args) = entity.find("ELLIPSE") {
            let frame = self.raw_frame2d(args.get(1)?.as_ref_id().ok()?)?;
            let semi_major = args.get(2)?.as_real().ok()?;
            let semi_minor = args.get(3)?.as_real().ok()?;
            return build_ellipse_arc(&frame, semi_major, semi_minor, 0.0, TAU)
                .ok()
                .map(Curve2d::Bounded);
        }
        // Rational complex form, then the plain one — the same two shapes
        // `Resolver::curve` reads for 3D splines.
        if let (Some(spline), Some(with_knots)) = (
            entity.find("B_SPLINE_CURVE"),
            entity.find("B_SPLINE_CURVE_WITH_KNOTS"),
        ) {
            let degree = spline.first()?.as_real().ok()? as usize;
            let point_refs = spline.get(1)?.as_list().ok()?;
            let multiplicities = with_knots.first()?.as_list().ok()?;
            let knot_values = with_knots.get(1)?.as_list().ok()?;
            let weights = match entity.find("RATIONAL_B_SPLINE_CURVE") {
                Some(rational) => rational
                    .first()?
                    .as_list()
                    .ok()?
                    .iter()
                    .map(Value::as_real)
                    .collect::<Result<Vec<_>, _>>()
                    .ok()?,
                None => vec![1.0; point_refs.len()],
            };
            return self.build_curve2d(degree, point_refs, multiplicities, knot_values, &weights);
        }
        if let Some(args) = entity.find("B_SPLINE_CURVE_WITH_KNOTS") {
            let degree = args.get(1)?.as_real().ok()? as usize;
            let point_refs = args.get(2)?.as_list().ok()?;
            let multiplicities = args.get(6)?.as_list().ok()?;
            let knot_values = args.get(7)?.as_list().ok()?;
            let weights = vec![1.0; point_refs.len()];
            return self.build_curve2d(degree, point_refs, multiplicities, knot_values, &weights);
        }
        None
    }

    fn build_curve2d(
        &self,
        degree: usize,
        point_refs: &[Value],
        multiplicities: &[Value],
        knot_values: &[Value],
        weights: &[f64],
    ) -> Option<Curve2d> {
        let mut knots = expand_knots(multiplicities, knot_values).ok()?;
        let mut control_points = point_refs
            .iter()
            .enumerate()
            .map(|(index, value)| {
                let raw = self.raw_point2d(value.as_ref_id().ok()?)?;
                Some(Vec4::from_point(
                    Vec3::new(raw[0], raw[1], 0.0),
                    weights.get(index).copied().unwrap_or(1.0),
                ))
            })
            .collect::<Option<Vec<_>>>()?;
        clamp_spline(degree, &mut knots, &mut control_points).ok()?;
        NurbsCurve::new(degree, knots, control_points)
            .ok()
            .map(Curve2d::Bounded)
    }

    /// A `CARTESIAN_POINT` read WITHOUT the length conversion `Resolver::point`
    /// applies. A 2D pcurve coordinate is a length on a plane and an ANGLE on
    /// every other carrier, so a blanket length scale is wrong on four of the
    /// five surface kinds; the caller applies the per-axis scale instead.
    fn raw_point2d(&self, id: usize) -> Option<[f64; 2]> {
        let coords = self.get(id).ok()?.find("CARTESIAN_POINT")?.get(1)?.as_list().ok()?;
        Some([
            coords.first().map(Value::as_real)?.ok()?,
            coords.get(1).and_then(|v| v.as_real().ok()).unwrap_or(0.0),
        ])
    }

    fn raw_direction2d(&self, id: usize) -> Option<[f64; 2]> {
        let coords = self.get(id).ok()?.find("DIRECTION")?.get(1)?.as_list().ok()?;
        let x = coords.first().map(Value::as_real)?.ok()?;
        let y = coords.get(1).and_then(|v| v.as_real().ok()).unwrap_or(0.0);
        let length = x.hypot(y);
        (length > 0.0).then(|| [x / length, y / length])
    }

    /// An `AXIS2_PLACEMENT_2D(name, location, ref_direction)` in raw file
    /// coordinates, presented as a `Frame` in the z = 0 plane so the existing
    /// conic builders can carry a 2D circle or ellipse exactly.
    fn raw_frame2d(&self, id: usize) -> Option<Frame> {
        let entity = self.get(id).ok()?;
        let args = entity
            .find("AXIS2_PLACEMENT_2D")
            .or_else(|| entity.find("AXIS2_PLACEMENT_3D"))?;
        let origin = self.raw_point2d(args.get(1)?.as_ref_id().ok()?)?;
        let reference = match args.get(2) {
            Some(Value::Ref(dir_ref)) => self.raw_direction2d(*dir_ref)?,
            _ => [1.0, 0.0],
        };
        Some(Frame {
            origin: Vec3::new(origin[0], origin[1], 0.0),
            x: Vec3::new(reference[0], reference[1], 0.0),
            y: Vec3::new(-reference[1], reference[0], 0.0),
            z: Vec3::new(0.0, 0.0, 1.0),
        })
    }
}
