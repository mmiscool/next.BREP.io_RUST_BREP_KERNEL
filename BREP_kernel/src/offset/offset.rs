use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::fit::solve_dense;
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{
    interpolate_curve, measure_edge_against_pcurve_image,
    measure_surface_fit_against_pointwise_offset, offset_construction_band, solid_model_scale,
    vertex_endpoint_gap, vertex_tolerance_from_edges, KnotVector, MeasuredTolerance, NurbsCurve,
    NurbsSurface, OffsetEvaluator, OffsetNormal, Vec2, Vec3, Vec4,
};
use rustc_hash::FxHashMap as HashMap;
use serde::Serialize;

fn domains(surface: &NurbsSurface) -> Result<([f64; 2], [f64; 2]), KernelRefusal> {
    Ok((
        KnotVector::new(surface.knots_u.clone(), surface.degree_u).or_refuse(KernelStage::Refine, "new")?.domain(),
        KnotVector::new(surface.knots_v.clone(), surface.degree_v).or_refuse(KernelStage::Refine, "new")?.domain(),
    ))
}

/// The normal `offset_surface` offsets along: the face's oriented normal with
/// the singular-row recovery, i.e. the shared evaluator's
/// [`OffsetNormal::FaceStable`] lane, whose body this function used to be.
fn stable_face_normal(face: &FaceRecord, u: f64, v: f64) -> Result<Vec3, KernelRefusal> {
    OffsetEvaluator::new(
        "offset_surface",
        &face.surface,
        OffsetNormal::FaceStable {
            same_sense: face.same_sense,
        },
    )
    .normal(u, v)
    .or_refuse(KernelStage::Refine, "normal")
}

fn greville_parameters(knots: &KnotVector) -> Vec<f64> {
    let mut parameters = (0..knots.control_point_count())
        .map(|index| {
            knots.knots[index + 1..=index + knots.degree]
                .iter()
                .sum::<f64>()
                / knots.degree as f64
        })
        .collect::<Vec<_>>();
    let domain = knots.domain();
    parameters[0] = domain[0];
    *parameters.last_mut().unwrap() = domain[1];
    parameters
}

/// Rational collocation matrix: rows are the rational basis functions
/// R_i(t) = N_i(t)·w_i / Σ_k N_k(t)·w_k evaluated at each parameter. With
/// uniform weights this reduces to the ordinary B-spline collocation matrix.
fn collocation_matrix(knots: &KnotVector, parameters: &[f64], weights: &[f64]) -> Vec<Vec<f64>> {
    parameters
        .iter()
        .map(|parameter| {
            let mut row = vec![0.0; knots.control_point_count()];
            let span = knots.find_span(*parameter);
            for (offset, value) in knots
                .basis_functions(span, *parameter)
                .into_iter()
                .enumerate()
            {
                let index = span - knots.degree + offset;
                row[index] = value * weights[index];
            }
            let denominator: f64 = row.iter().sum();
            if denominator.abs() > 0.0 {
                for value in &mut row {
                    *value /= denominator;
                }
            }
            row
        })
        .collect()
}

/// Split the weight grid into per-direction factors when it is separable
/// (w_ij = a_i·b_j), which covers every tensor surface built from rational
/// profile/rail curves (cylinders, cones, spheres, tori, revolves).
fn separable_weights(weights: &[Vec<f64>]) -> Option<(Vec<f64>, Vec<f64>)> {
    let first_row = weights.first()?;
    let anchor = *first_row.first()?;
    if anchor.abs() <= 1e-12 {
        return None;
    }
    let a: Vec<f64> = weights.iter().map(|row| row[0]).collect();
    let b: Vec<f64> = first_row.iter().map(|w| w / anchor).collect();
    for (i, row) in weights.iter().enumerate() {
        for (j, &w) in row.iter().enumerate() {
            if (w - a[i] * b[j]).abs() > 1e-10 * (1.0 + w.abs()) {
                return None;
            }
        }
    }
    Some((a, b))
}

/// Interpolate the sample grid in the SOURCE surface's rational basis (same
/// knots and weights). When the true offset is representable in that basis —
/// planes, cylinders, cones, spheres, tori — collocation at the Greville grid
/// recovers it EXACTLY, so offset carriers stay real analytic surfaces
/// instead of non-rational approximations with span-scale wobble.
fn interpolate_tensor(
    knot_u: &KnotVector,
    knot_v: &KnotVector,
    parameters_u: &[f64],
    parameters_v: &[f64],
    samples: &[Vec<Vec3>],
    weights: &[Vec<f64>],
) -> Result<Vec<Vec<Vec4>>, KernelRefusal> {
    let count_u = parameters_u.len();
    let count_v = parameters_v.len();
    if let Some((weights_u, weights_v)) = separable_weights(weights) {
        let matrix_u = collocation_matrix(knot_u, parameters_u, &weights_u);
        let matrix_v = collocation_matrix(knot_v, parameters_v, &weights_v);
        let mut intermediate = vec![vec![Vec3::default(); count_v]; count_u];
        for column in 0..count_v {
            let solve_axis = |axis: fn(Vec3) -> f64| {
                solve_dense(
                    matrix_u.clone(),
                    samples.iter().map(|row| axis(row[column])).collect(),
                )
            };
            let x = solve_axis(|point| point.x).or_refuse(KernelStage::Refine, "solve_axis")?;
            let y = solve_axis(|point| point.y).or_refuse(KernelStage::Refine, "solve_axis")?;
            let z = solve_axis(|point| point.z).or_refuse(KernelStage::Refine, "solve_axis")?;
            for row in 0..count_u {
                intermediate[row][column] = Vec3::new(x[row], y[row], z[row]);
            }
        }
        let mut controls = vec![vec![Vec4::from_point(Vec3::default(), 1.0); count_v]; count_u];
        for row in 0..count_u {
            let solve_axis = |axis: fn(Vec3) -> f64| {
                solve_dense(
                    matrix_v.clone(),
                    intermediate[row].iter().copied().map(axis).collect(),
                )
            };
            let x = solve_axis(|point| point.x).or_refuse(KernelStage::Refine, "solve_axis")?;
            let y = solve_axis(|point| point.y).or_refuse(KernelStage::Refine, "solve_axis")?;
            let z = solve_axis(|point| point.z).or_refuse(KernelStage::Refine, "solve_axis")?;
            for column in 0..count_v {
                controls[row][column] = Vec4::from_point(
                    Vec3::new(x[column], y[column], z[column]),
                    weights[row][column],
                );
            }
        }
        return Ok(controls);
    }

    // Non-separable weights: solve the full tensor collocation system with
    // the exact 2D rational basis. Nets are small in practice.
    let unknowns = count_u * count_v;
    let mut matrix = vec![vec![0.0; unknowns]; unknowns];
    for (k, &u) in parameters_u.iter().enumerate() {
        let span_u = knot_u.find_span(u);
        let basis_u = knot_u.basis_functions(span_u, u);
        for (l, &v) in parameters_v.iter().enumerate() {
            let span_v = knot_v.find_span(v);
            let basis_v = knot_v.basis_functions(span_v, v);
            let row = &mut matrix[k * count_v + l];
            let mut denominator = 0.0;
            for (du, value_u) in basis_u.iter().enumerate() {
                let i = span_u - knot_u.degree + du;
                for (dv, value_v) in basis_v.iter().enumerate() {
                    let j = span_v - knot_v.degree + dv;
                    let entry = value_u * value_v * weights[i][j];
                    row[i * count_v + j] = entry;
                    denominator += entry;
                }
            }
            if denominator.abs() > 0.0 {
                for value in row.iter_mut() {
                    *value /= denominator;
                }
            }
        }
    }
    let solve_axis = |axis: fn(Vec3) -> f64| {
        solve_dense(
            matrix.clone(),
            samples
                .iter()
                .flat_map(|row| row.iter().copied().map(axis))
                .collect(),
        )
    };
    let x = solve_axis(|point| point.x).or_refuse(KernelStage::Refine, "solve_axis")?;
    let y = solve_axis(|point| point.y).or_refuse(KernelStage::Refine, "solve_axis")?;
    let z = solve_axis(|point| point.z).or_refuse(KernelStage::Refine, "solve_axis")?;
    let mut controls = vec![vec![Vec4::from_point(Vec3::default(), 1.0); count_v]; count_u];
    for row in 0..count_u {
        for column in 0..count_v {
            let index = row * count_v + column;
            controls[row][column] = Vec4::from_point(
                Vec3::new(x[index], y[index], z[index]),
                weights[row][column],
            );
        }
    }
    Ok(controls)
}

/// Which branch of [`offset_surface`] built a carrier — and, with it, whether
/// comparing that carrier against the pointwise offset at the same `(u, v)` is
/// even the right question.
///
/// Recorded rather than inferred, in the pattern `offset/reintersect.rs`
/// established for its own two lanes: two of this function's three branches
/// deliberately move the result off the pointwise offset, and a measurement that
/// did not know which branch ran would report a designed divergence as a fit
/// error.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OffsetSurfaceLane {
    /// A rigid control-net shift of an affine (plane-like) carrier. The normal
    /// is constant, so the shifted net IS the pointwise offset — exact by
    /// construction, with no fit to measure.
    Affine,
    /// A CLOSED-FORM offset built from the source's recognized analytic shape
    /// rather than sampled: a sphere's concentric mate, whose net is the
    /// source's scaled about the centre. Exact like `Affine`, but derived from
    /// geometry the recognizer proved rather than from a constant normal, so
    /// `offset_surface_measured_sided` measures it instead of declaring it.
    Analytic,
    /// Greville collocation of the pointwise offset. `S_fit(u, v)` is meant to
    /// BE `offset(u, v)`, and the distance between them is the fit error this
    /// slice measures.
    #[default]
    Fit,
    /// The result was deliberately moved off the pointwise offset: the planar /
    /// ruled EXTENSION (which grows the carrier past its source rim, so the same
    /// `(u, v)` names a different point) or the apex-cone PINCH RETRIM (which
    /// pulls the crossed sample row back to the offset cone's own apex). Both
    /// are correct and intended; neither is comparable pointwise.
    Reparameterised,
}

/// A fitted offset carrier together with what its construction measured about
/// itself.
#[derive(Clone, Debug)]
pub struct MeasuredOffsetSurface {
    pub surface: NurbsSurface,
    pub lane: OffsetSurfaceLane,
    /// `max ‖S_fit(u, v) − offset(u, v)‖` over
    /// [`crate::measure_surface_fit_against_pointwise_offset`]'s grid, against
    /// the band it was judged with. When the RULED EXTENSION fired, the offset
    /// compared against is the offset of the source EXTENDED ALONG ITS RULINGS —
    /// the surface the carrier is the pointwise offset of — not of `face.surface`.
    ///
    /// `None` for [`OffsetSurfaceLane::Reparameterised`] — see that variant.
    pub fit: Option<MeasuredTolerance>,
}

/// How far an offset carrier's TRIM grows past the source face at each of
/// its four parametric sides, in world units.
///
/// A carrier is trimmed by the parametric IMAGE of the source loops, so the
/// growth is realised by reshaping the carrier surface's net while the cloned
/// pcurves stay put: an affine plane's 2x2 net slides along the plane, a ruled
/// (linear-v) net stretches each sampled ruling. Both are solved so that the
/// trim's own uv bounding box moves by exactly these amounts — a trim that
/// occupies a sub-range of its surface's domain (every booleaned or
/// blend-trimmed face) grows by the same distance as one that spans it. The
/// legacy `planar_extension: f64` entry points are the uniform case.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarrierExtension {
    pub u_min: f64,
    pub u_max: f64,
    pub v_min: f64,
    pub v_max: f64,
}

impl CarrierExtension {
    pub const NONE: CarrierExtension = CarrierExtension {
        u_min: 0.0,
        u_max: 0.0,
        v_min: 0.0,
        v_max: 0.0,
    };

    pub fn uniform(amount: f64) -> CarrierExtension {
        CarrierExtension {
            u_min: amount,
            u_max: amount,
            v_min: amount,
            v_max: amount,
        }
    }

    pub fn is_active(&self) -> bool {
        self.u_min > 0.0 || self.u_max > 0.0 || self.v_min > 0.0 || self.v_max > 0.0
    }

    fn extends_v(&self) -> bool {
        self.v_min > 0.0 || self.v_max > 0.0
    }
}

/// The trim's uv bounding box as FRACTIONS of the surface domain:
/// `([ta_u, tb_u], [ta_v, tb_v])`, each in `[0, 1]`. Sampled over every
/// coedge pcurve of every loop.
fn trim_domain_fractions(
    face: &FaceRecord,
    [u0, u1]: [f64; 2],
    [v0, v1]: [f64; 2],
) -> Result<([f64; 2], [f64; 2]), KernelRefusal> {
    let mut low = Vec2 {
        x: f64::INFINITY,
        y: f64::INFINITY,
    };
    let mut high = Vec2 {
        x: f64::NEG_INFINITY,
        y: f64::NEG_INFINITY,
    };
    for coedge in face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
    {
        let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        for sample in 0..=24 {
            let uv = coedge
                .pcurve
                .evaluate(p0 + (p1 - p0) * sample as f64 / 24.0).or_refuse(KernelStage::Refine, "evaluate")?;
            low.x = low.x.min(uv.x);
            low.y = low.y.min(uv.y);
            high.x = high.x.max(uv.x);
            high.y = high.y.max(uv.y);
        }
    }
    let fraction = |value: f64, start: f64, end: f64| {
        let span = end - start;
        if span.abs() <= f64::EPSILON || !value.is_finite() {
            return None;
        }
        Some(((value - start) / span).clamp(0.0, 1.0))
    };
    let u = match (fraction(low.x, u0, u1), fraction(high.x, u0, u1)) {
        (Some(a), Some(b)) if b - a > 1e-6 => [a, b],
        _ => [0.0, 1.0],
    };
    let v = match (fraction(low.y, v0, v1), fraction(high.y, v0, v1)) {
        (Some(a), Some(b)) if b - a > 1e-6 => [a, b],
        _ => [0.0, 1.0],
    };
    Ok((u, v))
}

/// How far the surface's domain ENDS must move (`(back, forward)`: the start
/// end backwards, the far end forwards, world units) so that a trim occupying
/// the domain fractions `[ta, tb]` grows by exactly `grow_min` at its start and
/// `grow_max` at its end when its pcurves are left untouched. A point at
/// fraction `t` of a linearly re-mapped domain moves by
/// `-(1 - t)·back + t·forward`; solving that at `ta` and `tb` gives the pair
/// below. It reduces to `(grow_min, grow_max)` for a full-domain trim, and
/// never shrinks either end (both results are non-negative for non-negative
/// inputs).
fn domain_end_moves(grow_min: f64, grow_max: f64, [ta, tb]: [f64; 2]) -> (f64, f64) {
    let span = tb - ta;
    if span <= 1e-6 {
        return (grow_min, grow_max);
    }
    let rate = (grow_min + grow_max) / span;
    let back = grow_min + ta * rate;
    let forward = (1.0 - ta) * rate - grow_min;
    (back.max(0.0), forward.max(0.0))
}

/// Construct the same fitted offset carrier surface as the reference shell
/// implementation. Positive distance follows its convention and moves
/// opposite the face's outward normal.
pub fn offset_surface(
    face: &FaceRecord,
    distance: f64,
    planar_extension: f64,
) -> Result<NurbsSurface, KernelRefusal> {
    offset_surface_with_lane(face, distance, &CarrierExtension::uniform(planar_extension))
        .map(|(surface, _)| surface)
}

/// [`offset_surface`], plus the deviation MEASURED between the carrier it built
/// and the pointwise offset that carrier approximates.
///
/// The surface is bit-identical to [`offset_surface`]'s — this calls the same
/// body and adds a read-only pass afterwards. Nothing here can change the
/// carrier: measuring the deviation AFTER the carrier is built records what
/// happened; the size-derived band stays the floor and the bar it is judged
/// against. A measurement never widens a band.
pub fn offset_surface_measured(
    face: &FaceRecord,
    distance: f64,
    planar_extension: f64,
    band: f64,
) -> Result<MeasuredOffsetSurface, KernelRefusal> {
    offset_surface_measured_sided(
        face,
        distance,
        &CarrierExtension::uniform(planar_extension),
        band,
    )
}

/// [`offset_surface_measured`] with a per-side [`CarrierExtension`].
pub fn offset_surface_measured_sided(
    face: &FaceRecord,
    distance: f64,
    extension: &CarrierExtension,
    band: f64,
) -> Result<MeasuredOffsetSurface, KernelRefusal> {
    let (surface, lane, extended) = offset_surface_constructed(face, distance, extension)?;
    let fit = match lane {
        OffsetSurfaceLane::Affine => Some(MeasuredTolerance::exact(band)),
        // `Analytic` is exact by construction and already verified against the
        // pointwise offset before it was returned — but it is measured here
        // like a fit anyway, so the record carries the number instead of the
        // claim. It reads ~1e-13 where `Fit` reads ~1e-6.
        OffsetSurfaceLane::Analytic | OffsetSurfaceLane::Fit => {
            Some(measure_surface_fit_against_pointwise_offset(
                extended.as_ref().unwrap_or(&face.surface),
                face.same_sense,
                &surface,
                distance,
                band,
            )?)
        }
        OffsetSurfaceLane::Reparameterised => None,
    };
    Ok(MeasuredOffsetSurface { surface, lane, fit })
}

/// The EXACT concentric offset of a sphere, or `None` for every other carrier.
///
/// A sphere's normal is radial, so its pointwise offset at distance `d` is the
/// concentric sphere of radius `R' = R ± d`. A NURBS map is affine in its
/// control points — `S = Σ N_i w_i P_i / Σ N_i w_i` — so scaling every control
/// point about the centre by `R'/R` scales the whole image by that factor about
/// that point, and the result IS that sphere to rounding. Knots, degrees and
/// weights are untouched, so the same `(u, v)` still names the corresponding
/// point: the trims `offset_face_carrier_impl` clones onto this carrier
/// verbatim land exactly where they did on the source.
///
/// The Greville fit below reaches the same surface only to its collocation
/// error. On `offset-shell-box-bore-dished-face` that error put the dished
/// face's volume contribution 4.69e-3 away from the closed form — the larger
/// share of the case's 3.9435e-3 deficit.
///
/// Everything here is COMPUTED, never claimed. The offset radius comes from the
/// same [`OffsetEvaluator`] the fit lane samples, so this lane cannot invent a
/// sign convention; the guards below reject a centre the recognizer got wrong
/// or a sphere turned inside out by an inward offset deeper than its radius;
/// and the finished net is checked against that evaluator on a grid before it
/// is returned. Every refusal falls through to the fit, which is what built
/// these carriers before.
///
/// A sphere the recognizer does not recognize — a FITTED patch that merely
/// looks spherical — stays on the fit lane, unchanged. This lane widens nothing
/// on its own: cylinders, cones and tori want a scaling about an axis or a
/// spine rather than a point, which is a different construction.
fn sphere_offset_surface(face: &FaceRecord, distance: f64) -> Result<Option<NurbsSurface>, KernelRefusal> {
    // Escape hatch, the same shape as `BREP_NO_SPHERE_CHARTS`: it makes the
    // closed form and the collocation fit two cells of one binary, so a
    // measurement can separate this lane's effect from everything else.
    if std::env::var_os("BREP_NO_ANALYTIC_OFFSET").is_some() {
        return Ok(None);
    }
    let source = &face.surface;
    let Some((centre, radius)) = source.analytic().and_then(|shape| shape.sphere_geometry()) else {
        return Ok(None);
    };
    let ([u0, u1], [v0, v1]) = domains(source)?;
    // `offset_surface`'s positive distance moves OPPOSITE the face normal while
    // the evaluator's moves ALONG it; the negation is the fit's own, reused
    // here rather than re-derived (audit §4.1's hand negations get no new one).
    // A normal this lane cannot read is a DECLINE, not an error: the fit below
    // samples the same evaluator across the whole Greville grid and has its own
    // singular-row recovery, so anything that offset before must still offset.
    let Ok(sample) = OffsetEvaluator::new(
        "offset_surface",
        source,
        OffsetNormal::FaceStable {
            same_sense: face.same_sense,
        },
    )
    .at((u0 + u1) / 2.0, (v0 + v1) / 2.0, -distance)
    else {
        return Ok(None);
    };
    let source_radial = sample.source.sub(centre);
    let offset_radial = sample.point.sub(centre);
    let offset_radius = offset_radial.length();
    let scale = radius.max(1.0);
    if radius <= 1e-12
        || offset_radius <= 1e-9 * scale
        // Offset through the centre and out the far side: the concentric mate
        // is a sphere turned inside out, not this construction's answer.
        || offset_radial.dot(source_radial) <= 0.0
        // The move must be radial and exactly |d| long. A recognized centre
        // that is not the real one fails here before it can scale anything.
        || ((offset_radius - radius).abs() - distance.abs()).abs() > 1e-9 * scale
    {
        return Ok(None);
    }
    let factor = offset_radius / radius;
    // A zero-weight control has no Euclidean point to scale. The fit lane never
    // asks one for a position, so declining keeps this lane strictly additive.
    let Ok(controls) = source
        .control_points
        .iter()
        .map(|row| {
            row.iter()
                .map(|control| {
                    Ok(Vec4::from_point(
                        centre.add(control.point().or_refuse(KernelStage::Refine, "point")?.sub(centre).scale(factor)),
                        control.w,
                    ))
                })
                .collect::<Result<Vec<_>, KernelRefusal>>()
        })
        .collect::<Result<Vec<Vec<_>>, KernelRefusal>>()
    else {
        return Ok(None);
    };
    let Ok(surface) = NurbsSurface::new(
        source.degree_u,
        source.degree_v,
        source.knots_u.clone(),
        source.knots_v.clone(),
        controls,
    ) else {
        return Ok(None);
    };
    // The claim, tested. A net that does not land on the pointwise offset
    // everywhere is not this lane's surface, whatever it was recognized as —
    // and a measurement that could not be taken is not a pass.
    match measure_surface_fit_against_pointwise_offset(
        source,
        face.same_sense,
        &surface,
        distance,
        1e-9 * scale,
    ) {
        Ok(measured) if measured.within_band() => Ok(Some(surface)),
        _ => Ok(None),
    }
}

/// The EXACT offset of a surface of REVOLUTION whose generatrix is ONE CIRCLE
/// ARC — a torus of any kind (ring, horn, spindle), over a full or a partial
/// sweep — or `None` for every other carrier.
///
/// Why it is exact. A revolution's normal lies in the meridian plane and equals
/// the generatrix's own in-plane normal there, so the offset surface IS the
/// revolution of the offset generatrix, and the offset of a circle arc of
/// radius `r` about `c` is the concentric arc of radius `r ± d`. On the net:
/// row `i` of a revolution's control net is an AFFINE image `A_i` of the
/// generatrix's controls — a rotation for the on-circle rows, but a rotation
/// STRETCHED radially by `1/cos` of the half-span for the mid rows, which is
/// not a similarity. Affine maps commute with scaling about a point, so scaling
/// row `i` about `C_i = A_i(c)` by `(r ± d)/r` is exactly `A_i` applied to the
/// offset circle's net; that commutation is the only reason the stretched rows
/// come out right. `C_i` is found without reading `A_i` at all: `c` is an affine
/// combination of three generatrix controls, and the same combination of row
/// `i`'s controls is `A_i(c)`. Knots, degrees and weights are untouched, so the
/// same `(u, v)` still names the corresponding point and cloned trims land
/// where they did, as on [`sphere_offset_surface`].
///
/// Everything is computed and then tested, never claimed. The generatrix is
/// row 0 of the net, and it must lie on one circle to rounding. The sign and
/// size of the move come from the same [`OffsetEvaluator`] the fit samples,
/// read at a generatrix point against the circle's centre: the move must be
/// radial and exactly `|d|` long, and an offset through the centre (`r − d ≤
/// 0`, where the offset circle turns inside out) declines. The finished net
/// must land on the pointwise offset everywhere on the measurement grid, to
/// `1e-9` of the model's scale. Every decline falls through to the fit.
///
/// Measured on the partial lemon (a 106° arc of radius 5 centred 3 off the
/// axis) shelled inward by 1: the fitted carrier sat 1.4e-5 off the pointwise
/// offset and its shell 3.6e-7 short of the Pappus form in volume.
fn revolved_arc_offset_surface(
    face: &FaceRecord,
    distance: f64,
) -> Result<Option<NurbsSurface>, KernelRefusal> {
    if std::env::var_os("BREP_NO_ANALYTIC_OFFSET").is_some() {
        return Ok(None);
    }
    let source = &face.surface;
    match source.analytic() {
        Some(crate::AnalyticSurface::Revolution { .. } | crate::AnalyticSurface::Torus { .. }) => {}
        _ => return Ok(None),
    }
    let rows = &source.control_points;
    let Some(first_row) = rows.first() else {
        return Ok(None);
    };
    if rows.iter().flatten().any(|control| !(control.w > 0.0)) {
        return Ok(None);
    }
    let Ok(generatrix) = NurbsCurve::new(source.degree_v, source.knots_v.clone(), first_row.clone())
    else {
        return Ok(None);
    };
    let [t0, t1] = generatrix.domain().or_refuse(KernelStage::Refine, "domain")?;
    let at = |fraction: f64| generatrix.evaluate(t0 + (t1 - t0) * fraction);
    // The circle through three generatrix points, then every sample held to it.
    let (a, b, e) = (at(0.0).or_refuse(KernelStage::Refine, "at")?, at(1.0 / 3.0).or_refuse(KernelStage::Refine, "at")?, at(2.0 / 3.0).or_refuse(KernelStage::Refine, "at")?);
    let (ab, ae) = (b.sub(a), e.sub(a));
    let normal = ab.cross(ae);
    let normal_length_squared = normal.dot(normal);
    if normal_length_squared <= 1e-24 {
        return Ok(None);
    }
    let centre = a.add(
        ae.cross(normal)
            .scale(ab.dot(ab))
            .add(normal.cross(ab).scale(ae.dot(ae)))
            .scale(0.5 / normal_length_squared),
    );
    let radius = a.sub(centre).length();
    let scale = (centre.length() + radius).max(1.0);
    let band = 1e-9 * scale;
    let unit_normal = normal.scale(1.0 / normal_length_squared.sqrt());
    let samples = 16 * generatrix.control_points.len();
    for index in 0..=samples {
        let point = at(index as f64 / samples as f64).or_refuse(KernelStage::Refine, "at")?;
        let radial = point.sub(centre);
        if (radial.length() - radius).abs() > band || radial.dot(unit_normal).abs() > band {
            return Ok(None);
        }
    }
    // `centre` as an affine combination of three generatrix controls, the best
    // conditioned pair against the first.
    let controls = first_row
        .iter()
        .map(|control| control.point().or_refuse(KernelStage::Refine, "point"))
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let origin = controls[0];
    let mut best = (0usize, 0usize, 0.0f64);
    for first in 1..controls.len() {
        for second in first + 1..controls.len() {
            let area = controls[first]
                .sub(origin)
                .cross(controls[second].sub(origin))
                .length();
            if area > best.2 {
                best = (first, second, area);
            }
        }
    }
    let (first, second, area) = best;
    if area <= 1e-9 * scale * scale {
        return Ok(None);
    }
    let (along_first, along_second) = (controls[first].sub(origin), controls[second].sub(origin));
    let offset_from_origin = centre.sub(origin);
    let (g11, g12, g22) = (
        along_first.dot(along_first),
        along_first.dot(along_second),
        along_second.dot(along_second),
    );
    let determinant = g11 * g22 - g12 * g12;
    let (r1, r2) = (along_first.dot(offset_from_origin), along_second.dot(offset_from_origin));
    let s = (r1 * g22 - r2 * g12) / determinant;
    let t = (r2 * g11 - r1 * g12) / determinant;
    if origin
        .add(along_first.scale(s))
        .add(along_second.scale(t))
        .sub(centre)
        .length()
        > band
    {
        return Ok(None);
    }
    // The move, read off the shared evaluator on the generatrix row. The
    // negation is the fit's own, as in `sphere_offset_surface`.
    let ([u0, _], [v0, v1]) = domains(source)?;
    let Ok(sample) = OffsetEvaluator::new(
        "offset_surface",
        source,
        OffsetNormal::FaceStable {
            same_sense: face.same_sense,
        },
    )
    .at(u0, (v0 + v1) / 2.0, -distance)
    else {
        return Ok(None);
    };
    let source_radial = sample.source.sub(centre);
    let offset_radial = sample.point.sub(centre);
    let offset_radius = offset_radial.length();
    if (source_radial.length() - radius).abs() > band
        || offset_radius <= band
        || offset_radial.dot(source_radial) <= 0.0
        || ((offset_radius - radius).abs() - distance.abs()).abs() > band
    {
        return Ok(None);
    }
    let factor = offset_radius / radius;
    let Ok(offset_rows) = rows
        .iter()
        .map(|row| {
            let points = row.iter().map(|control| control.point().or_refuse(KernelStage::Refine, "point")).collect::<Result<Vec<_>, KernelRefusal>>()?;
            let row_origin = points[0];
            let row_centre = row_origin
                .add(points[first].sub(row_origin).scale(s))
                .add(points[second].sub(row_origin).scale(t));
            Ok(points
                .iter()
                .zip(row)
                .map(|(point, control)| {
                    Vec4::from_point(row_centre.add(point.sub(row_centre).scale(factor)), control.w)
                })
                .collect::<Vec<_>>())
        })
        .collect::<Result<Vec<Vec<_>>, KernelRefusal>>()
    else {
        return Ok(None);
    };
    let Ok(surface) = NurbsSurface::new(
        source.degree_u,
        source.degree_v,
        source.knots_u.clone(),
        source.knots_v.clone(),
        offset_rows,
    ) else {
        return Ok(None);
    };
    match measure_surface_fit_against_pointwise_offset(
        source,
        face.same_sense,
        &surface,
        distance,
        band,
    ) {
        Ok(measured) if measured.within_band() => Ok(Some(surface)),
        _ => Ok(None),
    }
}

pub(crate) fn offset_surface_with_lane(
    face: &FaceRecord,
    distance: f64,
    extension: &CarrierExtension,
) -> Result<(NurbsSurface, OffsetSurfaceLane), KernelRefusal> {
    offset_surface_constructed(face, distance, extension).map(|(surface, lane, _)| (surface, lane))
}

/// [`offset_surface_with_lane`], plus the surface the carrier is the pointwise
/// offset OF when that is not the face's own: the source extended along its
/// rulings, for a ruled extension (see the RULED EXTENSION block).
fn offset_surface_constructed(
    face: &FaceRecord,
    distance: f64,
    extension: &CarrierExtension,
) -> Result<(NurbsSurface, OffsetSurfaceLane, Option<NurbsSurface>), KernelRefusal> {
    let source = &face.surface;
    let extension_active = extension.is_active();
    if source.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        let ([u0, u1], [v0, v1]) = domains(source)?;
        let normal = stable_face_normal(face, (u0 + u1) / 2.0, (v0 + v1) / 2.0)?;
        let shift = normal.scale(-distance);
        let mut points = source
            .control_points
            .iter()
            .map(|row| {
                row.iter()
                    .map(|control| Ok(control.point().or_refuse(KernelStage::Refine, "point")?.add(shift)))
                    .collect::<Result<Vec<_>, KernelRefusal>>()
            })
            .collect::<Result<Vec<_>, KernelRefusal>>()?;
        if extension_active {
            let p00 = points[0][0];
            let p01 = points[0][1];
            let p10 = points[1][0];
            let direction_u = p10.sub(p00).normalized().or_refuse(KernelStage::Refine, "normalized")?;
            let direction_v = p01.sub(p00).normalized().or_refuse(KernelStage::Refine, "normalized")?;
            // The net slides while the cloned pcurves stay put, so a trim
            // that spans only part of the domain would grow by less than
            // asked (a 20-wide face trimmed to [0,16] by a blend moved its
            // x=16 edge 0.6 for a 1.0 pad). Solve the slide for the TRIM's
            // own bounding box instead.
            let (trim_u, trim_v) = trim_domain_fractions(face, [u0, u1], [v0, v1])?;
            let (back_u, forward_u) = domain_end_moves(extension.u_min, extension.u_max, trim_u);
            let (back_v, forward_v) = domain_end_moves(extension.v_min, extension.v_max, trim_v);
            points[0][0] = p00
                .sub(direction_u.scale(back_u))
                .sub(direction_v.scale(back_v));
            points[0][1] = p01
                .sub(direction_u.scale(back_u))
                .add(direction_v.scale(forward_v));
            points[1][0] = p10
                .add(direction_u.scale(forward_u))
                .sub(direction_v.scale(back_v));
            points[1][1] = points[1][1]
                .add(direction_u.scale(forward_u))
                .add(direction_v.scale(forward_v));
        }
        let controls = points
            .into_iter()
            .enumerate()
            .map(|(row, points)| {
                points
                    .into_iter()
                    .enumerate()
                    .map(|(column, point)| {
                        Vec4::from_point(point, source.control_points[row][column].w)
                    })
                    .collect()
            })
            .collect();
        // The extension slides the control net along the plane, so the same
        // `(u, v)` no longer names the pointwise offset of the same source
        // point; without it the shift is rigid and exact.
        let lane = if extension_active {
            OffsetSurfaceLane::Reparameterised
        } else {
            OffsetSurfaceLane::Affine
        };
        return Ok((
            NurbsSurface::new(
                source.degree_u,
                source.degree_v,
                source.knots_u.clone(),
                source.knots_v.clone(),
                controls,
            ).or_refuse(KernelStage::Refine, "new")?,
            lane,
            None,
        ));
    }

    // A recognized SPHERE has a closed-form concentric offset that needs no
    // collocation at all. Taken before the fit so the exact answer wins
    // wherever it exists; `sphere_offset_surface` declines on everything else,
    // and on a sphere whose construction it cannot verify.
    //
    // The extension cannot reach a sphere either way: both blocks below sit
    // under `parameters_v.len() == 2`, and a sphere's meridian is a degree-2
    // net of five controls. Taking this lane under an active extension keeps
    // exactly the behaviour the fit had — no growth — and only sharpens it.
    if let Some(surface) = sphere_offset_surface(face, distance)? {
        return Ok((surface, OffsetSurfaceLane::Analytic, None));
    }
    // The same holds for a revolved ARC: its generatrix has three or more
    // controls, so no extension below could reach it.
    if let Some(surface) = revolved_arc_offset_surface(face, distance)? {
        return Ok((surface, OffsetSurfaceLane::Analytic, None));
    }

    let knot_u = KnotVector::new(source.knots_u.clone(), source.degree_u).or_refuse(KernelStage::Refine, "new")?;
    let knot_v = KnotVector::new(source.knots_v.clone(), source.degree_v).or_refuse(KernelStage::Refine, "new")?;
    let parameters_u = greville_parameters(&knot_u);
    let parameters_v = greville_parameters(&knot_v);
    // The Greville sample grid IS a pointwise offset evaluation — this fit is
    // the shared evaluator's consumer, not its peer. `offset_surface`'s
    // positive distance moves OPPOSITE the face normal while the evaluator's
    // moves ALONG it, so the negation happens once, here, with a name on it
    // (audit §4.1's four hand negations get no fifth).
    let evaluator = OffsetEvaluator::new(
        "offset_surface",
        source,
        OffsetNormal::FaceStable {
            same_sense: face.same_sense,
        },
    );
    let mut samples = Vec::new();
    for &u in &parameters_u {
        let mut row = Vec::new();
        for &v in &parameters_v {
            row.push(evaluator.at(u, v, -distance).or_refuse(KernelStage::Refine, "at")?.point);
        }
        samples.push(row);
    }
    // APEX-CONE PINCH RETRIM: offsetting an apex cone INWARD moves each
    // ruling past the axis — the sampled far row becomes a ring on the far
    // side (radius d·cos half-angle, mirrored through the axis) and the
    // offset surface self-pinches inside the v-domain. The genuine cavity
    // ends AT the pinch (the offset cone's own apex). For a linear-v net
    // (two sample rows — every made/booleaned cone) the pinch lies on each
    // ruling at the fraction where the radial vector vanishes: detect the
    // inversion (far-row radials anti-parallel to near-row radials about the
    // row centroids) and pull the far row back to the pinch point, so the
    // fitted surface ends in a proper degenerate apex row instead of a
    // parasitic inverted tip ending in an unweldable ring.
    // Both blocks below move the sample grid OFF the pointwise offset on
    // purpose. Recording that is what lets `offset_surface_measured` decline to
    // report a designed divergence as a fit error.
    let mut reparameterised = false;
    let mut extended_source: Option<NurbsSurface> = None;
    // Two u samples are enough for both: a 2 × 2 bilinear net (a planar
    // trapezoid wall of a lofted square frustum, which the affine test above
    // rejects on its parallelogram defect) has straight rulings like any
    // other linear-v net, and without the extension its inward carrier
    // stopped 0.059 short of the opening the reach lane had measured
    // (2026-09-26, the SHARP rim corner fixture).
    if parameters_v.len() == 2 && parameters_u.len() >= 2 {
        let centroid = |column: usize| {
            let mut sum = Vec3::default();
            for row in &samples {
                sum = sum.add(row[column]);
            }
            sum.scale(1.0 / samples.len() as f64)
        };
        let near_centroid = centroid(0);
        let far_centroid = centroid(1);
        let mut inverted = true;
        let mut pinch_fraction = 0.0f64;
        let mut near_mean = 0.0f64;
        let mut far_mean = 0.0f64;
        for row in &samples {
            let near_radial = row[0].sub(near_centroid);
            let far_radial = row[1].sub(far_centroid);
            let near_len = near_radial.length();
            let far_len = far_radial.length();
            if near_len <= 1e-9 || far_len <= 1e-9 {
                inverted = false;
                break;
            }
            if near_radial.dot(far_radial) >= 0.0 {
                inverted = false;
                break;
            }
            pinch_fraction += near_len / (near_len + far_len) / samples.len() as f64;
            near_mean += near_len / samples.len() as f64;
            far_mean += far_len / samples.len() as f64;
        }
        if inverted {
            let retrim_far = far_mean <= near_mean;
            // WHOSE fold is it?  The retrim exists so a cavity that ends at the
            // offset cone's own apex ends there in the TOPOLOGY too, instead of
            // running past the pinch into a mirrored ring no weld can use.  It
            // pays for that by moving the sample grid off the pointwise offset,
            // and the price is the shared-basis identity every consumer rests
            // on: after it, the SAME `(u, v)` no longer names the offset of the
            // same source point.
            //
            // That price is only worth paying when the face's TRIM actually
            // reaches past the pinch.  A trim on a cone's wide half offsets
            // perfectly well; its carrier folds somewhere the face never goes,
            // and shearing its parameterization to tidy that away sheared the
            // trim's own pcurves with it — a 45° cone sheet trimmed to its wide
            // half shipped a slab 0.86 % off its Pappus volume, validating and
            // wrong (`thicken.rs`,
            // `a_re_parameterized_offset_carrier_keeps_the_pointwise_offset`).
            //
            // So read the trim, the way the extension below already does.  With
            // a linear v basis the pinch sits at the v FRACTION
            // `pinch_fraction`, and the folded side is whichever end the retrim
            // would have pulled.  A face with no loops still reports the whole
            // domain, which keeps every full-domain carrier on its exact
            // previous path.
            let (_, trim_v) = trim_domain_fractions(face, knot_u.domain(), knot_v.domain())?;
            let trim_reaches_the_pinch = if retrim_far {
                trim_v[1] > pinch_fraction + 1e-9
            } else {
                trim_v[0] < pinch_fraction - 1e-9
            };
            if trim_reaches_the_pinch {
                // Pull the crossed (smaller-ring, past-the-pinch) end back to
                // the pinch point on each ruling.
                reparameterised = true;
                for row in &mut samples {
                    let near = row[0];
                    let far = row[1];
                    let pinch = near.add(far.sub(near).scale(pinch_fraction));
                    if retrim_far {
                        row[1] = pinch;
                    } else {
                        row[0] = pinch;
                    }
                }
            }
        }
        // RULED EXTENSION: `planar_extension` is a no-op for curved carriers
        // above, but a cone/cylinder lateral joined at a reflex edge needs
        // its offset skin to GROW past the source rim exactly like a plane
        // (a cylinder piercing a cone: the two offsets only meet past both
        // cloned rims). The carrier's parameterization stays (knots/pcurves
        // untouched) while its world image — and with it the cloned trim's
        // image — inflates: the carrier is the pointwise offset of the SOURCE
        // EXTENDED ALONG ITS RULINGS, which a linear-v net does exactly.
        //
        // Stretching the sampled OFFSET rulings instead reaches the same
        // surface only where the source is DEVELOPABLE — its normal constant
        // along each ruling, as on a cylinder, a cone or an extruded wall —
        // because only there is the offset ruled too. A twisted loft is ruled
        // and not developable: its stretched offset rulings left the offset,
        // and an outward shell of one validated 0.235 short (1.0 %). So the
        // stretch is MEASURED against the extended source's pointwise offset,
        // at both ends and the middle of every sampled ruling, and kept only
        // within the fit contract; otherwise the extended source is sampled
        // pointwise, and either way the refinement reads the carrier against it.
        if extension.extends_v() && !inverted {
            let mut min_ruling = f64::MAX;
            let mut back_allowance = f64::MAX;
            let mut forward_allowance = f64::MAX;
            let mut extendable = true;
            for row in &samples {
                let ruling = row[1].sub(row[0]);
                let length = ruling.length();
                min_ruling = min_ruling.min(length);
                // Radii about the row centroids expose a converging (conic)
                // ruling sheaf; the extension must stop short of its apex or
                // the sheet folds through it.
                let near_radial = row[0].sub(near_centroid).length();
                let far_radial = row[1].sub(far_centroid).length();
                if (far_radial - near_radial).abs() > 1e-9 {
                    let apex_at = near_radial / (near_radial - far_radial);
                    if (-1e-9..=1.0 + 1e-9).contains(&apex_at) {
                        // Apex inside the span: degenerate sheet, do not touch.
                        extendable = false;
                        break;
                    }
                    if apex_at < 0.0 {
                        back_allowance = back_allowance.min(0.9 * -apex_at);
                    } else {
                        forward_allowance = forward_allowance.min(0.9 * (apex_at - 1.0));
                    }
                }
            }
            // The allowances above are read off the OFFSET rows. The source's
            // own ruling sheaf converges at a different rate — an outward
            // offset moves a cone's apex FARTHER along the ruling — so a
            // growth the offset rows allow can still carry the EXTENDED SOURCE
            // through its own apex, and its pointwise offset would then be
            // taken on a surface folded back on itself. Read the same radial
            // ratio on the source rows and clamp by the tighter of the two.
            let mut source_back = f64::MAX;
            let mut source_forward = f64::MAX;
            if extendable {
                let source_centroid = |column: usize| {
                    let mut sum = Vec3::default();
                    for row in &source.control_points {
                        if let Ok(point) = row[column].point() {
                            sum = sum.add(point);
                        }
                    }
                    sum.scale(1.0 / source.control_points.len() as f64)
                };
                let (near_source, far_source) = (source_centroid(0), source_centroid(1));
                for row in &source.control_points {
                    let (near, far) = (row[0].point().or_refuse(KernelStage::Refine, "point")?, row[1].point().or_refuse(KernelStage::Refine, "point")?);
                    let near_radial = near.sub(near_source).length();
                    let far_radial = far.sub(far_source).length();
                    if (far_radial - near_radial).abs() > 1e-9 {
                        let apex_at = near_radial / (near_radial - far_radial);
                        if (-1e-9..=1.0 + 1e-9).contains(&apex_at) {
                            extendable = false;
                            break;
                        }
                        if apex_at < 0.0 {
                            source_back = source_back.min(0.9 * -apex_at);
                        } else {
                            source_forward = source_forward.min(0.9 * (apex_at - 1.0));
                        }
                    }
                }
            }
            let (back_allowance, forward_allowance) =
                (back_allowance.min(source_back), forward_allowance.min(source_forward));
            if extendable && min_ruling > 1e-9 {
                // Solved for the TRIM's v-extent, not the domain's: a bore
                // face keeps its drill's full-height surface and is trimmed
                // to the part it pierces, so moving the surface's own ends
                // by |d| moved the trim's rims by only a fraction of that
                // (a 30-long ruling trimmed to 20 gave 0.667 for 1.0) — and
                // an outward shell's bore never reached the grown planes.
                let (_, trim_v) = trim_domain_fractions(face, knot_u.domain(), knot_v.domain())?;
                let (back_world, forward_world) =
                    domain_end_moves(extension.v_min, extension.v_max, trim_v);
                let back = (back_world / min_ruling).min(back_allowance);
                let forward = (forward_world / min_ruling).min(forward_allowance);
                let stretched = samples
                    .iter()
                    .map(|row| {
                        let ruling = row[1].sub(row[0]);
                        vec![row[0].sub(ruling.scale(back)), row[1].add(ruling.scale(forward))]
                    })
                    .collect::<Vec<_>>();
                let extended = extend_along_rulings(source, back, forward)?;
                let extended_offsets = OffsetEvaluator::new(
                    "offset_surface",
                    &extended,
                    OffsetNormal::FaceStable {
                        same_sense: face.same_sense,
                    },
                );
                let [v0, v1] = knot_v.domain();
                let mut developability = 0.0f64;
                for (row, &u) in stretched.iter().zip(&parameters_u) {
                    for fraction in [0.0, 0.5, 1.0] {
                        let exact = extended_offsets.at(u, v0 + (v1 - v0) * fraction, -distance).or_refuse(KernelStage::Refine, "at")?.point;
                        let along = row[0].add(row[1].sub(row[0]).scale(fraction));
                        developability = developability.max(exact.sub(along).length());
                    }
                }
                let contract = offset_fit_contract(source);
                let stretch_kept = developability <= contract;
                samples = if stretch_kept {
                    stretched
                } else {
                    parameters_u
                        .iter()
                        .map(|&u| {
                            parameters_v
                                .iter()
                                .map(|&v| Ok(extended_offsets.at(u, v, -distance).or_refuse(KernelStage::Refine, "at")?.point))
                                .collect::<Result<Vec<_>, KernelRefusal>>()
                        })
                        .collect::<Result<Vec<_>, KernelRefusal>>()?
                };
                if let Ok(path) = std::env::var("BREP_OFFSET_FIT_TRACE") {
                    use std::io::Write;
                    let line = format!(
                        "extension deg=({},{}) net=({},{}) d={distance:.6} back={back:.6} forward={forward:.6} \
                         developability={developability:.3e} contract={contract:.3e} lane={} thread={} args={}\n",
                        source.degree_u,
                        source.degree_v,
                        source.control_points.len(),
                        source.control_points[0].len(),
                        if stretch_kept { "stretch" } else { "pointwise" },
                        std::thread::current().name().unwrap_or("-"),
                        std::env::args().skip(1).collect::<Vec<_>>().join(" "),
                    );
                    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                        let _ = file.write_all(line.as_bytes());
                    }
                }
                extended_source = Some(extended);
            }
        }
    }
    let reference = extended_source.as_ref().unwrap_or(source);
    let weights = reference
        .control_points
        .iter()
        .map(|row| row.iter().map(|point| point.w).collect::<Vec<_>>())
        .collect::<Vec<_>>();
    let fitted = NurbsSurface::new(
        source.degree_u,
        source.degree_v,
        source.knots_u.clone(),
        source.knots_v.clone(),
        interpolate_tensor(
            &knot_u,
            &knot_v,
            &parameters_u,
            &parameters_v,
            &samples,
            &weights,
        )?,
    ).or_refuse(KernelStage::Refine, "new")?;
    if reparameterised {
        return Ok((fitted, OffsetSurfaceLane::Reparameterised, None));
    }
    let reference_offsets = OffsetEvaluator::new(
        "offset_surface",
        reference,
        OffsetNormal::FaceStable {
            same_sense: face.same_sense,
        },
    );
    Ok((
        refine_offset_fit(reference, &reference_offsets, distance, fitted)?,
        OffsetSurfaceLane::Fit,
        extended_source,
    ))
}

/// A linear-v surface continued along its own rulings by `back` and `forward`
/// ruling lengths — the same surface, re-parameterized onto the longer span.
/// Exact: each ruling is linear in homogeneous coordinates, so extrapolating the
/// homogeneous control rows traces the ruling past both ends. Refused where a
/// continued weight stops being positive, which is where the ruling leaves the
/// rational patch.
fn extend_along_rulings(surface: &NurbsSurface, back: f64, forward: f64) -> Result<NurbsSurface, KernelRefusal> {
    let rows = surface
        .control_points
        .iter()
        .map(|row| {
            let step = row[1].add(row[0].scale(-1.0));
            vec![row[0].add(step.scale(-back)), row[1].add(step.scale(forward))]
        })
        .collect::<Vec<_>>();
    if rows.iter().flatten().any(|point| point.w <= 1e-12) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_ruled_extension_weight", format!(
            "offset_surface: extending a ruled face by ({back:.6}, {forward:.6}) of its rulings \
             carries a weight through zero; the extended face is not this surface"
        )));
    }
    NurbsSurface::new(surface.degree_u, 1, surface.knots_u.clone(), surface.knots_v.clone(), rows)
        .or_refuse(KernelStage::Refine, "new")
}

/// The fit contract a carrier of `source` is refined toward: the kernel's
/// intersection-fit tolerance for the source net's extent.
fn offset_fit_contract(source: &NurbsSurface) -> f64 {
    let scale = crate::model_scale(
        source
            .control_points
            .iter()
            .flatten()
            .filter_map(|point| point.point().ok()),
    );
    crate::KernelTolerances::for_scale(scale, 1e-7).intersection_fit
}

/// Halvings of every knot span [`refine_offset_fit`] may take, per direction.
const OFFSET_FIT_REFINEMENTS: usize = 6;

/// The largest net one refined collocation may solve: separable weights solve
/// one direction at a time, non-separable ones solve the whole tensor densely.
const OFFSET_FIT_MAX_CONTROLS_SEPARABLE: usize = 16_384;
const OFFSET_FIT_MAX_CONTROLS_TENSOR: usize = 900;

/// The out-of-sample deviation of a collocation fit from the pointwise offset,
/// read at the MIDPOINTS between consecutive Greville parameters, where the fit
/// interpolates nothing: along u (u midpoints on the v Greville rows), along v,
/// and between both.
#[derive(Clone, Copy, Debug)]
struct OffsetFitDeviation {
    along_u: f64,
    along_v: f64,
    between: f64,
}

impl OffsetFitDeviation {
    fn worst(&self) -> f64 {
        self.along_u.max(self.along_v).max(self.between)
    }
}

fn greville_midpoints(parameters: &[f64]) -> Vec<f64> {
    parameters
        .windows(2)
        .filter(|pair| pair[1] - pair[0] > 1e-12)
        .map(|pair| 0.5 * (pair[0] + pair[1]))
        .collect()
}

fn offset_fit_deviation(
    fitted: &NurbsSurface,
    evaluator: &OffsetEvaluator,
    distance: f64,
) -> Result<OffsetFitDeviation, KernelRefusal> {
    let parameters_u = greville_parameters(&KnotVector::new(fitted.knots_u.clone(), fitted.degree_u).or_refuse(KernelStage::Refine, "new")?);
    let parameters_v = greville_parameters(&KnotVector::new(fitted.knots_v.clone(), fitted.degree_v).or_refuse(KernelStage::Refine, "new")?);
    let (middle_u, middle_v) = (greville_midpoints(&parameters_u), greville_midpoints(&parameters_v));
    let gap = |u: f64, v: f64| -> Result<f64, KernelRefusal> {
        Ok(fitted.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?.sub(evaluator.at(u, v, -distance).or_refuse(KernelStage::Refine, "at")?.point).length())
    };
    let mut deviation = OffsetFitDeviation {
        along_u: 0.0,
        along_v: 0.0,
        between: 0.0,
    };
    for &u in &middle_u {
        for &v in &parameters_v {
            deviation.along_u = deviation.along_u.max(gap(u, v)?);
        }
        for &v in &middle_v {
            deviation.between = deviation.between.max(gap(u, v)?);
        }
    }
    for &u in &parameters_u {
        for &v in &middle_v {
            deviation.along_v = deviation.along_v.max(gap(u, v)?);
        }
    }
    Ok(deviation)
}

/// `surface` with each requested LINEAR direction raised to cubic, the same
/// geometry: every span re-expressed as a cubic Bezier on the homogeneous net
/// (the points at thirds), joined C0 by triple knots. `None` when a requested
/// direction is not linear or carries a repeated interior knot.
///
/// A linear direction is raised rather than halved because the offset of a
/// ruled surface is generally not ruled, and a piecewise-linear fit converges
/// only as h^2: the twisted loft's v direction still read 3.3e-6 after six
/// halvings, where the cubic basis converges as h^4.
fn raise_linear_directions(
    surface: &NurbsSurface,
    in_u: bool,
    in_v: bool,
) -> Result<Option<NurbsSurface>, KernelRefusal> {
    fn raise(knots: &[f64], row: &[Vec4]) -> Option<(Vec<f64>, Vec<Vec4>)> {
        // Degree 1: knots are [k0, k0, k1, ..., kn, kn] for n + 1 controls.
        let count = row.len();
        if knots.len() != count + 2 {
            return None;
        }
        let breaks = &knots[1..=count];
        if breaks.windows(2).any(|pair| pair[1] - pair[0] <= 1e-12) {
            return None;
        }
        let mut raised_knots = vec![breaks[0]; 4];
        for &interior in &breaks[1..count - 1] {
            raised_knots.extend([interior; 3]);
        }
        raised_knots.extend([breaks[count - 1]; 4]);
        let mut points = Vec::with_capacity(3 * (count - 1) + 1);
        for pair in row.windows(2) {
            let step = pair[1].add(pair[0].scale(-1.0));
            points.push(pair[0]);
            points.push(pair[0].add(step.scale(1.0 / 3.0)));
            points.push(pair[0].add(step.scale(2.0 / 3.0)));
        }
        points.push(row[count - 1]);
        Some((raised_knots, points))
    }
    let mut knots_u = surface.knots_u.clone();
    let mut knots_v = surface.knots_v.clone();
    let mut degree_u = surface.degree_u;
    let mut degree_v = surface.degree_v;
    let mut net = surface.control_points.clone();
    if in_v {
        if degree_v != 1 {
            return Ok(None);
        }
        let mut rows = Vec::with_capacity(net.len());
        for row in &net {
            let Some((knots, points)) = raise(&surface.knots_v, row) else {
                return Ok(None);
            };
            knots_v = knots;
            rows.push(points);
        }
        net = rows;
        degree_v = 3;
    }
    if in_u {
        if degree_u != 1 {
            return Ok(None);
        }
        let count_v = net[0].len();
        let mut columns = Vec::with_capacity(count_v);
        for column in 0..count_v {
            let row = net.iter().map(|row| row[column]).collect::<Vec<_>>();
            let Some((knots, points)) = raise(&surface.knots_u, &row) else {
                return Ok(None);
            };
            knots_u = knots;
            columns.push(points);
        }
        let count_u = columns[0].len();
        net = (0..count_u)
            .map(|row| columns.iter().map(|column| column[row]).collect())
            .collect();
        degree_u = 3;
    }
    Ok(Some(NurbsSurface::new(degree_u, degree_v, knots_u, knots_v, net).or_refuse(KernelStage::Refine, "new")?))
}

/// `surface`, the same geometry with a knot inserted at the midpoint of every
/// non-empty span of each requested direction (exact Boehm insertion on the
/// homogeneous net, so the weights refine with it).
fn halve_knot_spans(surface: &NurbsSurface, in_u: bool, in_v: bool) -> Result<NurbsSurface, KernelRefusal> {
    fn midpoints(knots: &[f64], degree: usize) -> Vec<f64> {
        let last = knots.len() - 1 - degree;
        knots[degree..=last]
            .windows(2)
            .filter(|pair| pair[1] - pair[0] > 1e-12)
            .map(|pair| 0.5 * (pair[0] + pair[1]))
            .collect()
    }
    let refine = |degree: usize, knots: &[f64], net: Vec<Vec<Vec4>>| -> Result<(Vec<f64>, Vec<Vec<Vec4>>), KernelRefusal> {
        let inserted = midpoints(knots, degree);
        let mut refined_knots = knots.to_vec();
        let mut rows = Vec::with_capacity(net.len());
        for row in net {
            let mut curve = NurbsCurve::new(degree, knots.to_vec(), row).or_refuse(KernelStage::Refine, "new")?;
            for &parameter in &inserted {
                curve = curve.insert_knot(parameter, 1).or_refuse(KernelStage::Refine, "insert_knot")?;
            }
            refined_knots = curve.knots.clone();
            rows.push(curve.control_points);
        }
        Ok((refined_knots, rows))
    };
    let mut knots_u = surface.knots_u.clone();
    let mut knots_v = surface.knots_v.clone();
    let mut net = surface.control_points.clone();
    if in_v {
        let (knots, rows) = refine(surface.degree_v, &knots_v, net)?;
        knots_v = knots;
        net = rows;
    }
    if in_u {
        // Transpose so each u column is a curve, refine, transpose back.
        let count_v = net[0].len();
        let columns = (0..count_v)
            .map(|column| net.iter().map(|row| row[column]).collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let (knots, refined) = refine(surface.degree_u, &knots_u, columns)?;
        knots_u = knots;
        let count_u = refined[0].len();
        net = (0..count_u)
            .map(|row| refined.iter().map(|column| column[row]).collect())
            .collect();
    }
    NurbsSurface::new(surface.degree_u, surface.degree_v, knots_u, knots_v, net).or_refuse(KernelStage::Refine, "new")
}

/// A collocation fit of the pointwise offset, refined until its OUT-OF-SAMPLE
/// deviation meets the kernel's intersection-fit contract.
///
/// The Greville fit interpolates the pointwise offset in the SOURCE's basis.
/// That is exact wherever the offset lives in that basis — planes, cylinders,
/// cones, spheres, tori, which is why it was built that way — and, on a
/// free-form face, only as good as the source's span count: a single-span cubic
/// Bezier wall offset by 0.3 was measured 1.2e-2 to 1.7e-2 off its pointwise
/// offset, with the fit exact at its four Greville rows and nowhere else.
///
/// Refinement halves the knot spans of the direction the deviation is read in
/// (exact knot insertion, so the parameterization does not move: the same
/// `(u, v)` still names the offset of the same source point, and every trim
/// pcurve cloned onto the carrier stays valid) and re-fits. A fit already under
/// the bar is returned untouched, bit for bit. The bar is the one a carrier's
/// EDGES are already held to (`offset_face_carrier_impl`'s `fit_tolerance`),
/// derived from the source net's extent.
///
/// The contract is a TARGET; the offset construction band is the GATE. A fit
/// that is still over the contract when the rounds run out
/// ([`OFFSET_FIT_REFINEMENTS`]), when a round stops improving, or when the next
/// net would pass the solver ceilings, is ACCEPTED as its best measured round if
/// that round is inside `offset_construction_band` — the allowance every
/// offset construction in this family is already judged against — and refused
/// by name if it is not. Accepted over the contract is a decision, not a
/// silence: `offset_surface_measured` reports the deviation, and
/// `a_fit_the_rounds_leave_over_the_contract_is_accepted_inside_the_band` pins
/// it. (Measured on the corpus: two imported `OFFSET_SURFACE`s end their rounds
/// at 1.3e-5 and 3.4e-5 against a contract of 2.15e-6 and a band of 0.107.)
///
/// Cost is bounded by the rounds: a direction's spans grow at most 64-fold, a
/// round costs one pointwise offset per Greville node plus about four per node
/// to measure, and one dense collocation solve per row and per column
/// (`n_v·n_u³ + n_u·n_v³` for separable weights), and the rounds' costs are
/// geometric, so the last round is about half of the whole.
fn refine_offset_fit(
    source: &NurbsSurface,
    evaluator: &OffsetEvaluator,
    distance: f64,
    fitted: NurbsSurface,
) -> Result<NurbsSurface, KernelRefusal> {
    let scale = crate::model_scale(
        source
            .control_points
            .iter()
            .flatten()
            .filter_map(|point| point.point().ok()),
    );
    let bar = crate::KernelTolerances::for_scale(scale, 1e-7).intersection_fit;
    let trace = std::env::var("BREP_OFFSET_FIT_TRACE").ok();
    let started = web_time::Instant::now();
    let enabled = std::env::var("BREP_OFFSET_FIT_REFINE").as_deref() != Ok("0");
    let Ok(initial) = offset_fit_deviation(&fitted, evaluator, distance) else {
        return Ok(fitted);
    };
    let mut best = fitted;
    let mut best_deviation = initial;
    let mut refined_source = source.clone();
    let mut rounds = 0usize;
    let mut stop = "under bar";
    if enabled {
        while best_deviation.worst() > bar {
            if rounds == OFFSET_FIT_REFINEMENTS {
                stop = "refinement limit";
                break;
            }
            let (along_u, along_v) = (best_deviation.along_u > bar, best_deviation.along_v > bar);
            let (in_u, in_v) = if along_u || along_v {
                (along_u, along_v)
            } else {
                (true, true)
            };
            let (raise_u, raise_v) = (in_u && refined_source.degree_u == 1, in_v && refined_source.degree_v == 1);
            let raised = if raise_u || raise_v {
                raise_linear_directions(&refined_source, raise_u, raise_v)?
            } else {
                None
            };
            let (halve_u, halve_v) = match raised {
                Some(_) => (in_u && !raise_u, in_v && !raise_v),
                None => (in_u, in_v),
            };
            let raised = raised.unwrap_or_else(|| refined_source.clone());
            let candidate_source = if halve_u || halve_v {
                halve_knot_spans(&raised, halve_u, halve_v)?
            } else {
                raised
            };
            let weights = candidate_source
                .control_points
                .iter()
                .map(|row| row.iter().map(|point| point.w).collect::<Vec<_>>())
                .collect::<Vec<_>>();
            let controls = weights.len() * weights[0].len();
            let ceiling = if separable_weights(&weights).is_some() {
                OFFSET_FIT_MAX_CONTROLS_SEPARABLE
            } else {
                OFFSET_FIT_MAX_CONTROLS_TENSOR
            };
            if controls > ceiling {
                stop = "net ceiling";
                break;
            }
            let knot_u = KnotVector::new(candidate_source.knots_u.clone(), candidate_source.degree_u).or_refuse(KernelStage::Refine, "new")?;
            let knot_v = KnotVector::new(candidate_source.knots_v.clone(), candidate_source.degree_v).or_refuse(KernelStage::Refine, "new")?;
            let parameters_u = greville_parameters(&knot_u);
            let parameters_v = greville_parameters(&knot_v);
            let samples = parameters_u
                .iter()
                .map(|&u| {
                    parameters_v
                        .iter()
                        .map(|&v| Ok(evaluator.at(u, v, -distance).or_refuse(KernelStage::Refine, "at")?.point))
                        .collect::<Result<Vec<_>, KernelRefusal>>()
                })
                .collect::<Result<Vec<_>, KernelRefusal>>();
            let Ok(samples) = samples else {
                stop = "unevaluable sample";
                break;
            };
            let candidate = NurbsSurface::new(
                candidate_source.degree_u,
                candidate_source.degree_v,
                candidate_source.knots_u.clone(),
                candidate_source.knots_v.clone(),
                interpolate_tensor(&knot_u, &knot_v, &parameters_u, &parameters_v, &samples, &weights)?,
            ).or_refuse(KernelStage::Refine, "new")?;
            let Ok(deviation) = offset_fit_deviation(&candidate, evaluator, distance) else {
                stop = "unmeasurable round";
                break;
            };
            rounds += 1;
            if deviation.worst() >= best_deviation.worst() {
                stop = "stopped improving";
                break;
            }
            best = candidate;
            best_deviation = deviation;
            refined_source = candidate_source;
        }
    } else if best_deviation.worst() > bar {
        stop = "refinement disabled";
    }
    // The gate. Refinement aims at the fit contract; a carrier it could not
    // bring inside the offset family's own acceptance band is refused by name
    // rather than handed on — a fit measured six orders past its band used to be
    // recorded and used anyway.
    let band = offset_construction_band(scale);
    let refused = enabled && best_deviation.worst() > band;
    if let Some(path) = trace {
        use std::io::Write;
        let caller = std::env::args().skip(1).collect::<Vec<_>>().join(" ");
        let thread = std::thread::current().name().unwrap_or("-").to_string();
        let line = format!(
            "fit deg=({},{}) net=({},{})->({},{}) d={distance:.6} scale={scale:.4} bar={bar:.3e} \
             initial=(u {:.3e}, v {:.3e}, uv {:.3e}) final={:.3e} rounds={rounds} stop={stop} \
             refused={refused} seconds={:.4} thread={thread} args={caller}\n",
            source.degree_u,
            source.degree_v,
            source.control_points.len(),
            source.control_points[0].len(),
            best.control_points.len(),
            best.control_points[0].len(),
            initial.along_u,
            initial.along_v,
            initial.between,
            best_deviation.worst(),
            started.elapsed().as_secs_f64(),
        );
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = file.write_all(line.as_bytes());
        }
    }
    if refused {
        return Err(KernelRefusal::non_convergence(KernelStage::Refine, "offset_carrier_fit", format!(
            "offset_surface: the fitted offset carrier stays {:.3e} off its pointwise offset at \
             distance {distance} after {rounds} refinement round(s) ({stop}), past the offset \
             construction band {band:.3e}",
            best_deviation.worst()
        )));
    }
    Ok(best)
}

/// Two offset sheets of one source, re-expressed on ONE basis without moving
/// either: a linear direction facing a cubic one is raised, then each sheet
/// takes the knots the other has and it lacks (exact insertion).
///
/// [`refine_offset_fit`] refines each carrier by its own measured deviation, so
/// two sheets of the same source at different distances can leave on different
/// nets; a consumer that pairs their iso curves control point for control point
/// (`thicken`'s ruled walls) needs them on the same one. Weights follow the
/// insertions, and both sheets carry the source's weights refined the same way.
pub(crate) fn unify_offset_sheet_bases(
    first: &NurbsSurface,
    second: &NurbsSurface,
) -> Result<(NurbsSurface, NurbsSurface), KernelRefusal> {
    fn raise_to(surface: &NurbsSurface, degree_u: usize, degree_v: usize) -> Result<NurbsSurface, KernelRefusal> {
        let raise_u = surface.degree_u == 1 && degree_u == 3;
        let raise_v = surface.degree_v == 1 && degree_v == 3;
        if !raise_u && !raise_v {
            return Ok(surface.clone());
        }
        raise_linear_directions(surface, raise_u, raise_v)?
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Refine,
                    "offset_raise_linear",
                    "offset sheets: a linear direction could not be raised",
                )
            })
    }
    let degree_u = first.degree_u.max(second.degree_u);
    let degree_v = first.degree_v.max(second.degree_v);
    let mut first = raise_to(first, degree_u, degree_v)?;
    let mut second = raise_to(second, degree_u, degree_v)?;
    if first.degree_u != second.degree_u || first.degree_v != second.degree_v {
        return Err(KernelRefusal::internal(KernelStage::Refine, "offset_common_basis", format!(
            "offset sheets: degrees ({},{}) and ({},{}) have no common basis",
            first.degree_u, first.degree_v, second.degree_u, second.degree_v
        )));
    }
    // Knots one vector has beyond the other, counted with multiplicity.
    fn missing(from: &[f64], into: &[f64]) -> Vec<f64> {
        let mut extra = Vec::new();
        let mut index = 0usize;
        for &knot in from {
            while index < into.len() && into[index] < knot - 1e-12 {
                index += 1;
            }
            if index < into.len() && (into[index] - knot).abs() <= 1e-12 {
                index += 1;
            } else {
                extra.push(knot);
            }
        }
        extra
    }
    fn insert(surface: &NurbsSurface, knots: &[f64], along_u: bool) -> Result<NurbsSurface, KernelRefusal> {
        if knots.is_empty() {
            return Ok(surface.clone());
        }
        let (degree, vector) = if along_u {
            (surface.degree_u, &surface.knots_u)
        } else {
            (surface.degree_v, &surface.knots_v)
        };
        let lines = if along_u {
            (0..surface.control_points[0].len())
                .map(|column| surface.control_points.iter().map(|row| row[column]).collect::<Vec<_>>())
                .collect::<Vec<_>>()
        } else {
            surface.control_points.clone()
        };
        let mut refined_knots = vector.clone();
        let mut refined = Vec::with_capacity(lines.len());
        for line in lines {
            let mut curve = NurbsCurve::new(degree, vector.clone(), line).or_refuse(KernelStage::Refine, "new")?;
            for &knot in knots {
                curve = curve.insert_knot(knot, 1).or_refuse(KernelStage::Refine, "insert_knot")?;
            }
            refined_knots = curve.knots.clone();
            refined.push(curve.control_points);
        }
        let net = if along_u {
            (0..refined[0].len())
                .map(|row| refined.iter().map(|column| column[row]).collect())
                .collect()
        } else {
            refined
        };
        let (knots_u, knots_v) = if along_u {
            (refined_knots, surface.knots_v.clone())
        } else {
            (surface.knots_u.clone(), refined_knots)
        };
        NurbsSurface::new(surface.degree_u, surface.degree_v, knots_u, knots_v, net).or_refuse(KernelStage::Refine, "new")
    }
    let (to_first_u, to_second_u) = (
        missing(&second.knots_u, &first.knots_u),
        missing(&first.knots_u, &second.knots_u),
    );
    first = insert(&first, &to_first_u, true)?;
    second = insert(&second, &to_second_u, true)?;
    let (to_first_v, to_second_v) = (
        missing(&second.knots_v, &first.knots_v),
        missing(&first.knots_v, &second.knots_v),
    );
    first = insert(&first, &to_first_v, false)?;
    second = insert(&second, &to_second_v, false)?;
    Ok((first, second))
}

/// The 3D curve of one carrier edge: the image of the source coedge's pcurve
/// on the carrier surface, with the parameter range `(t0, t1)` that matches the
/// pcurve's domain fraction for fraction (the contract every coedge of this
/// carrier relies on, since the source pcurves are reused verbatim).
///
/// Built through the [`crate::image_curve`] ladder — exact on an affine sheet
/// (every planar offset), the exact iso-curve where the pcurve holds one
/// coordinate constant (a blend rail, a cylinder's cap rim, every
/// extrude/revolve boundary), and a fitted curve verified to `fit_tolerance`
/// off its nodes otherwise. Only where the ladder refuses does the edge fall
/// back to the degree-1 interpolant through `points` (the historical
/// construction), so nothing that offset before is refused now.
///
/// Why the polyline is no longer the first choice: its adaptive subdivision
/// stops at a 5e-4 chord sag, and a 128-chord rim on an r = 8.5 cylinder sits
/// up to 1.6e-4 inside the true arc. That is 16x the boolean imprint's
/// coincidence band, so when a later operation's section curve runs along that
/// rim (a cutter's side wall coplanar with the offset shell's opening wall,
/// 2026-09-06 report) the imprint cannot recognise the rim as the section it
/// already is, and reports every chord vertex as a crossing — one edge split
/// into forty. An exact arc is recognised as coincident and left alone.
fn carrier_edge_curve(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    points: &[Vec3],
    parameters: &[f64],
    fit_tolerance: f64,
) -> Result<(NurbsCurve, f64, f64), KernelRefusal> {
    // `BREP_CARRIER_POLYLINE=1` restores the polyline for an A/B comparison.
    let image = if std::env::var("BREP_CARRIER_POLYLINE").as_deref() == Ok("1") {
        Err("BREP_CARRIER_POLYLINE set".to_string())
    } else {
        crate::image_curve::image_curve(surface, pcurve, fit_tolerance, "offset_face_carrier")
    };
    match image {
        Ok(image) if image.t0 <= image.t1 => Ok((image.curve, image.t0, image.t1)),
        Ok(image) => {
            // The image runs against the pcurve: reverse it so the edge range
            // is increasing, reflecting the range through the curve's domain.
            let [start, end] = image.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let curve = image.curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
            Ok((curve, start + end - image.t0, start + end - image.t1))
        }
        Err(_) => {
            let curve = interpolate_curve(points, 1, parameters).or_refuse(KernelStage::Refine, "interpolate_curve")?;
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            Ok((curve, t0, t1))
        }
    }
}

fn mapped_pcurve_polyline(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    degenerate: bool,
) -> Result<(Vec<Vec3>, Vec<f64>), KernelRefusal> {
    let [start, end] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let evaluate = |fraction: f64| -> Result<Vec3, KernelRefusal> {
        let uv = pcurve.evaluate(start + (end - start) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
        surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")
    };
    let first = evaluate(0.0)?;
    let last = evaluate(1.0)?;
    if degenerate {
        return Ok((vec![first, last], vec![0.0, 1.0]));
    }
    fn append(
        evaluate: &impl Fn(f64) -> Result<Vec3, KernelRefusal>,
        a_fraction: f64,
        a: Vec3,
        b_fraction: f64,
        b: Vec3,
        depth: usize,
        parameters: &mut Vec<f64>,
        points: &mut Vec<Vec3>,
    ) -> Result<(), KernelRefusal> {
        let fractions =
            [0.25, 0.5, 0.75].map(|local| a_fraction + (b_fraction - a_fraction) * local);
        let samples = fractions
            .map(evaluate)
            .into_iter()
            .collect::<Result<Vec<_>, KernelRefusal>>()?;
        let deviation = samples
            .iter()
            .enumerate()
            .map(|(index, point)| {
                point
                    .sub(a.add(b.sub(a).scale((index + 1) as f64 * 0.25)))
                    .length()
            })
            .fold(0.0, f64::max);
        if deviation <= 5e-4 || depth >= 10 {
            parameters.push(b_fraction);
            points.push(b);
            return Ok(());
        }
        append(
            evaluate,
            a_fraction,
            a,
            fractions[1],
            samples[1],
            depth + 1,
            parameters,
            points,
        )?;
        append(
            evaluate,
            fractions[1],
            samples[1],
            b_fraction,
            b,
            depth + 1,
            parameters,
            points,
        )
    }
    let mut parameters = vec![0.0];
    let mut points = vec![first];
    append(
        &evaluate,
        0.0,
        first,
        1.0,
        last,
        0,
        &mut parameters,
        &mut points,
    )?;
    Ok((points, parameters))
}

/// Everything an offset carrier's construction MEASURED about itself.
///
/// The half this kernel lacked: a tolerance MEASURED from the geometry that was
/// actually built, rather than a band chosen from `scale` before building. It
/// lives in the one place that needs no durable-format change: alongside the
/// transient construction result, never as a field on a [`crate::BrepSolid`]
/// record. `io/snapshot.rs` is a documented durable format and
/// `SOLID_CODEC_VERSION` a versioned wire layout; persisting per-entity
/// tolerances is real, planned, and separately designed. This lands the
/// measurement with no format churn at all.
///
/// Every number here is a RECORD. None of it widens a band — see
/// [`MeasuredTolerance`]'s direction rule.
#[derive(Clone, Debug)]
pub struct CarrierDeviation {
    /// The derived band every measurement below was judged against:
    /// [`crate::offset_construction_band`] of the source solid's extent.
    pub band: f64,
    /// Which branch built the carrier surface.
    pub lane: OffsetSurfaceLane,
    /// The carrier surface's own fit error, when the lane has one.
    pub surface: Option<MeasuredTolerance>,
    /// `max_t ‖C_3d(t) − S_off(p(t))‖` per carrier edge id, folded over every
    /// coedge that references the edge — OCCT's `FillEdgeData` rule
    /// (`BRepOffset_SimpleOffset.cxx:296-310`), which takes the maximum over
    /// **every** adjacent face rather than the first one.
    pub edges: Vec<(u64, MeasuredTolerance)>,
    /// Per carrier vertex id, propagated from the incident edge ends by
    /// [`crate::vertex_tolerance_from_edges`] — which is also where the verdict
    /// on OCCT's 1.001 inflation factor is recorded.
    pub vertices: Vec<(u64, f64)>,
}

impl CarrierDeviation {
    /// The worst thing the construction did, against the tightest band it
    /// faced. `None` only when there was nothing at all to measure.
    pub fn worst(&self) -> Option<MeasuredTolerance> {
        MeasuredTolerance::worst(
            self.surface
                .into_iter()
                .chain(self.edges.iter().map(|(_, measured)| *measured))
                .chain(
                    self.vertices
                        .iter()
                        .map(|(_, gap)| MeasuredTolerance::new(*gap, self.band)),
                ),
        )
    }

    /// The entities whose measured deviation exceeded the derived band — the
    /// interesting case, and the only one any gate acts on.
    pub fn exceedances(&self) -> Vec<String> {
        let mut out = Vec::new();
        if let Some(surface) = self.surface {
            if surface.exceeds_band() {
                out.push(format!("carrier surface fit {}", surface.describe()));
            }
        }
        for (id, measured) in &self.edges {
            if measured.exceeds_band() {
                out.push(format!("edge {id} {}", measured.describe()));
            }
        }
        for (id, gap) in &self.vertices {
            let measured = MeasuredTolerance::new(*gap, self.band);
            if measured.exceeds_band() {
                out.push(format!("vertex {id} {}", measured.describe()));
            }
        }
        out
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct OffsetFaceCarrier {
    pub vertices: Vec<VertexRecord>,
    pub edges: Vec<EdgeRecord>,
    pub face: FaceRecord,
    /// What the construction measured about itself, or `None` when it was built
    /// through the unmeasured [`offset_face_carrier`] entry point.
    ///
    /// `#[serde(skip)]` on purpose: this struct crosses the wasm ABI as JSON
    /// (`abi/modeling_b.rs:500`), and a measurement is a diagnostic about a
    /// build, not part of the carrier the caller asked for. Skipping it keeps
    /// that payload byte-identical.
    #[serde(skip)]
    pub deviation: Option<CarrierDeviation>,
}

fn claim_vertex_image(
    source_id: u64,
    point: Vec3,
    vertex_images: &mut HashMap<u64, u64>,
    vertices: &mut Vec<VertexRecord>,
    next_id: &mut u64,
) -> u64 {
    if let Some(id) = vertex_images.get(&source_id) {
        return *id;
    }
    let id = *next_id;
    *next_id += 1;
    vertices.push(VertexRecord { id, point });
    vertex_images.insert(source_id, id);
    id
}

pub fn offset_face_carrier(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
    planar_extension: f64,
) -> Result<OffsetFaceCarrier, KernelRefusal> {
    offset_face_carrier_impl(
        solid,
        face_id,
        distance,
        &CarrierExtension::uniform(planar_extension),
        false,
    )
}

/// [`offset_face_carrier`] with a per-side [`CarrierExtension`].
pub fn offset_face_carrier_sided(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
    extension: &CarrierExtension,
) -> Result<OffsetFaceCarrier, KernelRefusal> {
    offset_face_carrier_impl(solid, face_id, distance, extension, false)
}

/// [`offset_face_carrier`], with every entity it builds measured against the
/// deviation it was meant to reproduce.
///
/// The carrier is bit-identical to [`offset_face_carrier`]'s — same body, same
/// arithmetic, in the same order — with a read-only measurement pass appended.
/// Three quantities land in [`CarrierDeviation`]:
///
/// * the carrier SURFACE's fit against the pointwise offset it interpolates
///   ([`offset_surface_measured`]);
/// * each carrier EDGE's 3D curve against the locus its pcurve traces on that
///   surface — the trim boundary is built by sampling exactly that composition
///   and interpolating the images, so this is the construction's own claim,
///   measured rather than assumed;
/// * each carrier VERTEX, propagated from the incident edge ends.
///
/// The interesting output is [`CarrierDeviation::exceedances`]: an entity whose
/// measured deviation is worse than the size-derived band assumed. That is a
/// construction that went wrong in a way the derived band alone cannot see, and
/// it is the case a caller should refuse on rather than ship.
pub fn offset_face_carrier_measured(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
    planar_extension: f64,
) -> Result<OffsetFaceCarrier, KernelRefusal> {
    offset_face_carrier_impl(
        solid,
        face_id,
        distance,
        &CarrierExtension::uniform(planar_extension),
        true,
    )
}

/// The measured deviations of one built carrier: surface fit, per edge, per
/// vertex.
///
/// Read-only over what the construction produced. The edge measurement is
/// [`crate::measure_edge_against_pcurve_image`] — validate's own
/// `adaptive_coedge_error` floored by a span-midpoint pass, because the sampler
/// alone is aliased against exactly the curves this construction builds (see
/// that module's doc) — taken against the far tighter
/// [`crate::offset_construction_band`] instead of the vendor-forgiving
/// `pcurve_acceptance` validate will use later.
fn measure_carrier(
    surface: &NurbsSurface,
    vertices: &[VertexRecord],
    edges: &[EdgeRecord],
    loops: &[LoopRecord],
    lane: OffsetSurfaceLane,
    surface_fit: Option<MeasuredTolerance>,
    band: f64,
) -> Result<CarrierDeviation, KernelRefusal> {
    let edge_by_id: HashMap<u64, &EdgeRecord> = edges.iter().map(|edge| (edge.id, edge)).collect();
    // Fold over coedges, not edges: a seam edge is referenced twice with two
    // different pcurves, and OCCT's `FillEdgeData` takes the maximum over every
    // adjacent face for exactly that reason.
    let mut per_edge: HashMap<u64, MeasuredTolerance> = HashMap::default();
    for loop_record in loops {
        for coedge in &loop_record.coedges {
            let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
                continue;
            };
            let measured = measure_edge_against_pcurve_image(
                surface,
                &coedge.pcurve,
                edge,
                coedge.forward,
                band,
            )?;
            per_edge
                .entry(edge.id)
                .and_modify(|existing| *existing = existing.worse_of(measured))
                .or_insert(measured);
        }
    }

    let mut measured_edges: Vec<(u64, MeasuredTolerance)> = per_edge.into_iter().collect();
    measured_edges.sort_by_key(|(id, _)| *id);
    let deviation_of: HashMap<u64, f64> = measured_edges
        .iter()
        .map(|(id, measured)| (*id, measured.deviation()))
        .collect();

    // Edge ENDS by the vertex they claim, built once. A degenerate edge claims
    // the same vertex at both ends and contributes both, which is right: the
    // question is how far every representation meeting there actually lands.
    let mut ends_at: HashMap<u64, Vec<(&EdgeRecord, f64)>> = HashMap::default();
    for edge in edges {
        ends_at
            .entry(edge.start_vertex_id)
            .or_default()
            .push((edge, edge.t0));
        ends_at
            .entry(edge.end_vertex_id)
            .or_default()
            .push((edge, edge.t1));
    }

    let mut measured_vertices = Vec::with_capacity(vertices.len());
    for vertex in vertices {
        let mut gaps = Vec::new();
        let mut incident = Vec::new();
        for (edge, parameter) in ends_at.get(&vertex.id).into_iter().flatten() {
            gaps.push(vertex_endpoint_gap(
                vertex.point,
                edge.curve.evaluate(*parameter).or_refuse(KernelStage::Refine, "evaluate")?,
            ));
            incident.push(deviation_of.get(&edge.id).copied().unwrap_or(0.0));
        }
        measured_vertices.push((vertex.id, vertex_tolerance_from_edges(gaps, incident)));
    }

    Ok(CarrierDeviation {
        band,
        lane,
        surface: surface_fit,
        edges: measured_edges,
        vertices: measured_vertices,
    })
}

fn offset_face_carrier_impl(
    solid: &BrepSolid,
    face_id: u64,
    distance: f64,
    extension: &CarrierExtension,
    measure: bool,
) -> Result<OffsetFaceCarrier, KernelRefusal> {
    let source = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .ok_or_else(|| format!("offset_face_carrier: missing face {face_id}")).or_refuse(KernelStage::Refine, "offset_missing_face")?;
    // The band is derived from the SOURCE SOLID's extent, matching every other
    // direct-edit/offset site (`face_offset.rs:103` and its siblings all take
    // `solid_model_scale`). The alternative basis is the FACE's own extent — the
    // reference shell keys its size-relative offset quantities on the 3D arc
    // length of the face's own boundary iso, never on the assembly. Which basis
    // is better is unsettled: it is a measurement to make over the refusal
    // corpus before switching, not a change to smuggle in here.
    let band = offset_construction_band(solid_model_scale(solid));
    // The accuracy bar a carrier EDGE's 3D curve must meet against the locus its
    // pcurve traces on the carrier surface — the kernel's own SSI fit contract,
    // the same bar `thicken` holds its wall boundaries to. NOT the construction
    // band: that is the surface-fit allowance (5e-4 of the part), 50x looser
    // than the boolean's coincidence band, and an edge fitted only that well is
    // what a later imprint mistakes for a crossing (see `carrier_edge_curve`).
    let fit_tolerance =
        crate::KernelTolerances::for_scale(solid_model_scale(solid), 1e-7).intersection_fit;
    let (surface, lane, surface_fit) = if measure {
        let measured = offset_surface_measured_sided(source, distance, extension, band)?;
        (measured.surface, measured.lane, measured.fit)
    } else {
        let (surface, lane) = offset_surface_with_lane(source, distance, extension)?;
        (surface, lane, None)
    };
    let source_edges = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let source_vertices = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex))
        .collect::<HashMap<_, _>>();
    let mut vertices = Vec::new();
    let mut vertex_images = HashMap::default();
    let mut edges = Vec::new();
    let mut edge_images = HashMap::default();
    let mut loops = Vec::new();
    let mut next_id = 1u64;

    for source_loop in &source.loops {
        let mut coedges = Vec::new();
        for source_coedge in &source_loop.coedges {
            let source_edge = source_edges
                .get(&source_coedge.edge_id)
                .ok_or_else(|| "offset_face_carrier: missing source edge".to_string()).or_refuse(KernelStage::Refine, "offset_missing_edge")?;
            let (source_start, source_end) = if source_coedge.forward {
                (source_edge.start_vertex_id, source_edge.end_vertex_id)
            } else {
                (source_edge.end_vertex_id, source_edge.start_vertex_id)
            };
            if !source_vertices.contains_key(&source_start)
                || !source_vertices.contains_key(&source_end)
            {
                return Err(KernelRefusal::internal(KernelStage::Refine, "offset_missing_vertex", "offset_face_carrier: missing source vertex"));
            }
            // Map even DEGENERATE source edges through the full polyline: a
            // cone apex's image on the offset surface is a genuine CIRCLE
            // (radius d·cos half-angle), not a point — shortcutting to the
            // two endpoints would collapse the ring and leave the carrier's
            // topology inconsistent with its surface. Edges whose image truly
            // collapses (sphere poles, planar corners) still interpolate to a
            // point-sized curve and keep their degenerate flag below.
            // Map even DEGENERATE source edges through the full polyline: an
            // EXTERIOR cone offset turns the apex point into a genuine RING
            // (radius d·cos half-angle) — shortcutting to the endpoints would
            // collapse it and leave the carrier topology inconsistent with
            // its surface (and the ring imprint would be dropped as
            // boundary-coincident with a "degenerate" edge). Images that
            // truly collapse (sphere poles; interior apexes after the pinch
            // retrim) stay degenerate below. The threshold scales with the
            // offset distance: a real ring measures ~d·cos α, while fitted
            // pole rows wobble ~1e-4 absolute.
            let (points, parameters) =
                mapped_pcurve_polyline(&surface, &source_coedge.pcurve, false)?;
            let collapse_tolerance = 1e-6f64.max(distance.abs() * 1e-2);
            let image_collapsed = points
                .iter()
                .all(|point| point.sub(points[0]).length() <= collapse_tolerance);
            let (edge_id, forward) =
                if let Some((edge_id, edge_start_vertex_id, creator_forward)) =
                    edge_images.get(&source_edge.id)
                {
                    if source_start == source_end {
                        // A CLOSED source edge (a seam of a periodic face: both
                        // ends are the same vertex) is referenced twice by the
                        // same loop, once per seam side, and the two references
                        // traverse it in OPPOSITE senses — that is what closes
                        // the loop. The endpoint test below cannot see that:
                        // both ends map to the same vertex image, so it answers
                        // `true` for both references and the returning coedge
                        // comes back mis-oriented.
                        //
                        // Measured on a full torus (one vertex, two seam edges):
                        // the source records `forward: false` on the two
                        // returning coedges and validates clean, while the
                        // carrier recorded `forward: true` on both and its rim
                        // edge measured 12.5 — a whole part diameter — against
                        // its own composed pcurve image. The measured tolerance
                        // this slice adds is what surfaced it; no existing gate
                        // reaches this shape.
                        //
                        // The source coedge's own sense is the answer: the image
                        // edge runs along the CREATING coedge's pcurve, so a
                        // later reference runs with it exactly when the two
                        // source coedges traverse the source edge the same way.
                        (*edge_id, source_coedge.forward == *creator_forward)
                    } else {
                        // UNCHANGED for every open edge: the image edge's start
                        // vertex identifies which way this coedge runs.
                        (
                            *edge_id,
                            vertex_images.get(&source_start) == Some(edge_start_vertex_id),
                        )
                    }
                } else {
                    let (curve, t0, t1) = if image_collapsed {
                        let curve = NurbsCurve::new(
                            1,
                            vec![0.0, 0.0, 1.0, 1.0],
                            vec![
                                Vec4::from_point(points[0], 1.0),
                                Vec4::from_point(points[0], 1.0),
                            ],
                        ).or_refuse(KernelStage::Refine, "new")?;
                        (curve, 0.0, 1.0)
                    } else {
                        carrier_edge_curve(
                            &surface,
                            &source_coedge.pcurve,
                            &points,
                            &parameters,
                            fit_tolerance,
                        )?
                    };
                    let start_vertex_id = claim_vertex_image(
                        source_start,
                        points[0],
                        &mut vertex_images,
                        &mut vertices,
                        &mut next_id,
                    );
                    let end_vertex_id = claim_vertex_image(
                        source_end,
                        points[points.len() - 1],
                        &mut vertex_images,
                        &mut vertices,
                        &mut next_id,
                    );
                    let id = next_id;
                    next_id += 1;
                    edges.push(EdgeRecord {
                        id,
                        curve,
                        t0,
                        t1,
                        start_vertex_id,
                        end_vertex_id,
                        // Degenerate exactly when the IMAGE collapsed. A cone
                        // apex maps to a real RING on an exterior offset and
                        // must carry a real closed edge; a real source edge
                        // whose image collapses to a point is a degenerate edge
                        // ON THE CARRIER whatever it was on the source.
                        //
                        // That second half is what the CARVE needs. The new
                        // boundary a carve introduces is a piece of the fold
                        // locus, which on the source is an ordinary curve — a
                        // parallel circle on a cone — and whose whole defining
                        // property is that the offset collapses there. Reading
                        // the source's flag as well left it a zero-length
                        // NON-degenerate closed edge, which no weld can use.
                        degenerate: image_collapsed,
                        // Image of a named source edge on the offset carrier;
                        // suffixed so it cannot collide with the source edge
                        // when both faces survive into one solid.
                        name: source_edge
                            .name
                            .as_ref()
                            .map(|name| format!("{name}_Offset")),
                    });
                    edge_images.insert(
                        source_edge.id,
                        (id, start_vertex_id, source_coedge.forward),
                    );
                    (id, true)
                };
            let id = next_id;
            next_id += 1;
            coedges.push(CoedgeRecord {
                id,
                edge_id,
                forward,
                pcurve: source_coedge.pcurve.clone(),
            });
        }
        let id = next_id;
        next_id += 1;
        loops.push(LoopRecord { id, coedges });
    }
    let deviation = if measure {
        Some(measure_carrier(
            &surface,
            &vertices,
            &edges,
            &loops,
            lane,
            surface_fit,
            band,
        )?)
    } else {
        None
    };
    Ok(OffsetFaceCarrier {
        vertices,
        edges,
        face: FaceRecord {
            id: next_id,
            surface,
            same_sense: source.same_sense,
            loops,
            name: source.name.as_ref().map(|name| format!("{name}_Offset")),
        },
        deviation,
    })
}

