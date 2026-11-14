use crate::topology::BrepSolid;
use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct AffineTransform {
    pub elements: [f64; 16],
}

impl AffineTransform {
    /// Whether this affine map is a finite, proper rigid placement.
    pub fn is_rigid(&self, tolerance: f64) -> bool {
        let m = &self.elements;
        if !tolerance.is_finite() || tolerance < 0.0 || m.iter().any(|x| !x.is_finite()) {
            return false;
        }
        if m[12].abs() > tolerance || m[13].abs() > tolerance || m[14].abs() > tolerance
            || (m[15] - 1.0).abs() > tolerance {
            return false;
        }
        let rows = [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]];
        for i in 0..3 {
            for j in i..3 {
                let dot: f64 = (0..3).map(|k| rows[i][k] * rows[j][k]).sum();
                let expected = if i == j { 1.0 } else { 0.0 };
                if (dot - expected).abs() > tolerance { return false; }
            }
        }
        (self.determinant3() - 1.0).abs() <= tolerance
    }

    /// Compose row-major transforms, applying `b` first.
    pub fn compose(&self, b: &AffineTransform) -> Result<AffineTransform, String> {
        // Iterator Sum starts at -0.0; the raw STEP loop starts at +0.0.
        AffineTransform::new(multiply_elements_seeded(&self.elements, &b.elements, -0.0))
    }

    pub fn new(elements: [f64; 16]) -> Result<Self, String> {
        if elements.iter().any(|value| !value.is_finite()) {
            return Err("AffineTransform: matrix must be finite".into());
        }
        if elements[12].abs() > 1e-12
            || elements[13].abs() > 1e-12
            || elements[14].abs() > 1e-12
            || (elements[15] - 1.0).abs() > 1e-12
        {
            return Err("AffineTransform: projective matrices are unsupported".into());
        }
        if (Self { elements }).determinant3().abs() <= 1e-14 {
            return Err("AffineTransform: singular matrix".into());
        }
        Ok(Self { elements })
    }

    /// Invert a rigid map as `[Rᵀ | −Rᵀ·t]`. The caller must verify rigidity.
    pub fn rigid_inverse(&self) -> Result<Self, String> {
        let m = &self.elements;
        let r = [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]];
        let t = [m[3], m[7], m[11]];
        let mut out = [0.0f64; 16];
        for row in 0..3 {
            for col in 0..3 {
                out[row * 4 + col] = r[col][row]; // Rᵀ
            }
            out[row * 4 + 3] = -(0..3).map(|k| r[k][row] * t[k]).sum::<f64>();
        }
        out[15] = 1.0;
        AffineTransform::new(out)
    }

    pub fn determinant3(self) -> f64 {
        let m = self.elements;
        m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8])
            + m[2] * (m[4] * m[9] - m[5] * m[8])
    }

    pub fn point(self, point: Vec3) -> Vec3 {
        let m = self.elements;
        Vec3::new(
            m[0] * point.x + m[1] * point.y + m[2] * point.z + m[3],
            m[4] * point.x + m[5] * point.y + m[6] * point.z + m[7],
            m[8] * point.x + m[9] * point.y + m[10] * point.z + m[11],
        )
    }

    fn homogeneous(self, point: Vec4) -> Result<Vec4, String> {
        Ok(Vec4::from_point(self.point(point.point()?), point.w))
    }
}

pub(crate) fn transform_curve(
    curve: &NurbsCurve,
    transform: AffineTransform,
) -> Result<NurbsCurve, String> {
    NurbsCurve::new(
        curve.degree,
        curve.knots.clone(),
        curve
            .control_points
            .iter()
            .map(|point| transform.homogeneous(*point))
            .collect::<Result<_, _>>()?,
    )
}

pub(crate) fn transform_surface(
    surface: &NurbsSurface,
    transform: AffineTransform,
) -> Result<NurbsSurface, String> {
    NurbsSurface::new(
        surface.degree_u,
        surface.degree_v,
        surface.knots_u.clone(),
        surface.knots_v.clone(),
        surface
            .control_points
            .iter()
            .map(|row| {
                row.iter()
                    .map(|point| transform.homogeneous(*point))
                    .collect::<Result<Vec<_>, _>>()
            })
            .collect::<Result<_, _>>()?,
    )
}

/// Transform exact BREP geometry without changing entity IDs or parameter
/// ranges. `reverse_orientation` is required for negative-determinant maps.
pub fn transform_brep(
    solid: &BrepSolid,
    transform: AffineTransform,
    reverse_orientation: bool,
) -> Result<BrepSolid, String> {
    let determinant = transform.determinant3();
    if determinant < 0.0 && !reverse_orientation {
        return Err("transformSolid: reflection requires orientation reversal".into());
    }
    if determinant > 0.0 && reverse_orientation {
        return Err("transformSolid: orientation reversal requires a reflection".into());
    }
    let mut result = solid.clone();
    for vertex in &mut result.vertices {
        vertex.point = transform.point(vertex.point);
    }
    for edge in &mut result.edges {
        edge.curve = transform_curve(&edge.curve, transform)?;
    }
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            face.surface = transform_surface(&face.surface, transform)?;
            if reverse_orientation {
                face.same_sense = !face.same_sense;
                for loop_record in &mut face.loops {
                    loop_record.coedges.reverse();
                    for coedge in &mut loop_record.coedges {
                        coedge.forward = !coedge.forward;
                        coedge.pcurve = coedge.pcurve.reversed()?;
                    }
                }
            }
        }
    }
    // Where the copy's coordinates are spaced finely, a placement moves the
    // part faithfully. Farther out, rounding to the coordinates there can
    // reshape or collapse it while every topological check still passes, so
    // the move is measured against the part and the copy is validated itself.
    let precise = arithmetic_spacing(solid, &result, transform) <= PRECISE_SPACING;
    if !precise {
        placement_resolves_the_part(solid, &result, transform)?;
    }
    let mode = carry_verdict_mode();
    let carried = precise
        && mode != crate::env_read::Mode::Off
        && carries_clean_verdict(solid, transform, reverse_orientation);
    if carried && mode == crate::env_read::Mode::On {
        CARRIED_VERDICTS.with(|count| count.set(count.get() + 1));
        if crate::profile::profile_enabled() {
            profile_line!("transform.verdict carried=1");
        }
        return Ok(result);
    }
    let issues = result.validate();
    if mode == crate::env_read::Mode::Check && crate::profile::profile_enabled() {
        // What carrying would have answered, against the copy's own verdict.
        profile_line!(
            "transform.verdict check carried={} issues={} disagree={}",
            u8::from(carried),
            issues.len(),
            u8::from(carried && !issues.is_empty())
        );
    }
    if issues.is_empty() {
        Ok(result)
    } else {
        Err(format!("transformed BREP is invalid: {issues:?}"))
    }
}

/// Spacing of the coordinates a placement computes with, below which a moved
/// copy is faithful to the model tolerance with room to spare (a tenth of
/// it): within about 4.5e7 of the origin. Such copies keep the fast path
/// unchanged.
const PRECISE_SPACING: f64 = 0.1 * crate::tolerance::DEFAULT_MODEL;

/// Every point a solid's geometry is stated by, in a fixed order: its
/// vertices, then each edge curve's control points, then each face surface's.
/// A source and its transformed copy list them in parallel. A pole that does
/// not read as a point comes out non-finite.
fn stated_points(solid: &BrepSolid) -> impl Iterator<Item = Vec3> + '_ {
    let cartesian =
        |point: &Vec4| point.point().unwrap_or(Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY));
    solid
        .vertices
        .iter()
        .map(|vertex| vertex.point)
        .chain(solid.edges.iter().flat_map(move |edge| edge.curve.control_points.iter().map(cartesian)))
        .chain(
            solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .flat_map(move |face| face.surface.control_points.iter().flatten().map(cartesian)),
        )
}

/// The farthest coordinate of `points` from the origin; infinite when one is
/// not finite.
fn reach(points: impl Iterator<Item = Vec3>) -> f64 {
    points
        .flat_map(|point| [point.x, point.y, point.z])
        .fold(0.0f64, |reach, value| if value.is_finite() { reach.max(value.abs()) } else { f64::INFINITY })
}

/// The spacing of representable coordinates at the largest magnitude the
/// placement computes with: the copy's points, the source's as the linear
/// part scales them, and the translation. A far part brought back near the
/// origin loses its precision on the way, so the copy alone cannot tell; a
/// far pole loses it though every vertex is near.
fn arithmetic_spacing(source: &BrepSolid, copy: &BrepSolid, transform: AffineTransform) -> f64 {
    let m = transform.elements;
    let gain = (0..3).map(|row| m[row * 4].abs() + m[row * 4 + 1].abs() + m[row * 4 + 2].abs()).fold(0.0f64, f64::max);
    let translation = m[3].abs().max(m[7].abs()).max(m[11].abs());
    reach(stated_points(copy)).max(gain * reach(stated_points(source))).max(translation) * f64::EPSILON
}

/// The sum of `terms`, exact until its final rounding: a Shewchuk expansion
/// keeps every bit any partial sum would lose, however the terms cancel.
fn exact_sum(terms: impl IntoIterator<Item = f64>) -> f64 {
    let mut partials: Vec<f64> = Vec::new();
    for mut term in terms {
        let mut kept = 0;
        for index in 0..partials.len() {
            let mut other = partials[index];
            if term.abs() < other.abs() {
                std::mem::swap(&mut term, &mut other);
            }
            let high = term + other;
            let low = other - (high - term);
            if low != 0.0 {
                partials[kept] = low;
                kept += 1;
            }
            term = high;
        }
        partials.truncate(kept);
        partials.push(term);
    }
    // Smallest first: the large partials do not overlap, so this rounds once
    // to within an ulp of the result.
    partials.iter().sum()
}

/// How far a computed coordinate lies from the exact `A p + t` for one row:
/// the three products split exactly (fused multiply-add), the translation and
/// the computed value summed exactly, so the rounding is measured to its own
/// precision whatever the magnitudes.
fn coordinate_rounding(row: [f64; 4], point: Vec3, computed: f64) -> f64 {
    let mut terms = [0.0f64; 8];
    for (slot, (factor, value)) in [(row[0], point.x), (row[1], point.y), (row[2], point.z)].into_iter().enumerate() {
        let product = factor * value;
        terms[2 * slot] = product;
        terms[2 * slot + 1] = factor.mul_add(value, -product);
    }
    terms[6] = row[3];
    terms[7] = -computed;
    -exact_sum(terms)
}

/// Whether a measured rounding is fine enough for roundings to be compared
/// against the model tolerance: its own ulp must sit well below it. One past
/// that (tens of kilometres of rounding, from coordinates near 1e23) has lost
/// the part in any case.
fn measurable(rounding: f64) -> bool {
    rounding.is_finite() && rounding.abs() * f64::EPSILON <= 0.01 * crate::tolerance::DEFAULT_MODEL
}

/// Refuse a copy the placement could not carry at the precision the kernel
/// models to. Each stated point of the copy is set against its exact image
/// under the placement: the difference is that coordinate's rounding. Points
/// rounded alike have only moved together, which changes nothing; the spread
/// of the roundings across the part is how far it was moved out of shape. A
/// spread past the model tolerance, the band vertex identity is judged in,
/// has reshaped the part (a thin side or wall lost, a corner merged, a cavity
/// shifted into its solid), however small that is next to the part, and
/// validation cannot see it. No reference point is subtracted, so nothing the
/// placement rounds away can be rounded away from the measure too.
fn placement_resolves_the_part(
    source: &BrepSolid,
    copy: &BrepSolid,
    transform: AffineTransform,
) -> Result<(), String> {
    let m = transform.elements;
    let rows = [[m[0], m[1], m[2], m[3]], [m[4], m[5], m[6], m[7]], [m[8], m[9], m[10], m[11]]];
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    let mut lost = false;
    for (from, to) in stated_points(source).zip(stated_points(copy)) {
        for (axis, value) in [to.x, to.y, to.z].into_iter().enumerate() {
            let rounding = coordinate_rounding(rows[axis], from, value);
            if !measurable(rounding) {
                lost = true;
                continue;
            }
            low[axis] = low[axis].min(rounding);
            high[axis] = high[axis].max(rounding);
        }
    }
    let spread = (0..3).map(|axis| (high[axis] - low[axis]).max(0.0).powi(2)).sum::<f64>().sqrt();
    let worst = if lost { f64::INFINITY } else { spread };
    if worst > crate::tolerance::DEFAULT_MODEL {
        return Err(format!(
            "transformSolid: the placement computes with coordinates spaced {:.1e}, too coarse to hold the part: it moves its points against one another by up to {worst:.1e}, past the {:.0e} modelling tolerance",
            arithmetic_spacing(source, copy, transform),
            crate::tolerance::DEFAULT_MODEL
        ));
    }
    Ok(())
}

/// Whether a rigid placement may keep its source's clean validation verdict
/// instead of validating the moved copy. A proper rigid map (the shared
/// `RIGID_TOL` band components are placed under) changes no distance, angle,
/// parameter or topology. It does change one input of `validate`: the
/// identity and pcurve bands are sized from the AXIS-ALIGNED bounding box of
/// the vertices, whose diagonal moves under a rotation, so a copy read with
/// its own (narrower) bands could report an issue its source does not. The
/// carried verdict is the source's, read in the source's frame. The source's
/// verdict comes from the content-keyed validate memo: the first placement of
/// a part validates the part once, each further placement of the same
/// content reads the memo. A source with issues is never carried; its copy is
/// validated as before. `BREP_TRANSFORM_CARRY_VERDICT`: on by default (`0`
/// validates every copy, as before), `check` validates every copy anyway and
/// prints (`BREP_PROFILE`) whether carrying would have disagreed.
fn carries_clean_verdict(
    solid: &BrepSolid,
    transform: AffineTransform,
    reverse_orientation: bool,
) -> bool {
    !reverse_orientation
        && transform.is_rigid(crate::RIGID_TOL)
        && solid.validate_memoised().is_empty()
}

thread_local! {
    /// Placements on this thread that kept their source's verdict.
    static CARRIED_VERDICTS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}


fn carry_verdict_mode() -> crate::env_read::Mode {
    crate::mode_switch!("BREP_TRANSFORM_CARRY_VERDICT", true)
}

/// Reflect an exact BREP across the plane through `plane_point` with unit
/// normal `n = plane_normal.normalized()`. The reflection is the affine map
/// `p' = R p + t` with linear part `R = I - 2 n nᵀ` and translation
/// `t = 2 (plane_point·n) n`, so points on the plane map to themselves. A
/// reflection has negative determinant (it flips handedness), so faces are
/// re-oriented via the existing transform with `reverse_orientation = true`,
/// keeping outward normals.
pub fn mirror_brep(
    solid: &BrepSolid,
    plane_point: Vec3,
    plane_normal: Vec3,
) -> Result<BrepSolid, String> {
    let n = plane_normal.normalized()?;
    let (nx, ny, nz) = (n.x, n.y, n.z);
    let d = plane_point.dot(n);
    let (tx, ty, tz) = (2.0 * d * nx, 2.0 * d * ny, 2.0 * d * nz);
    let matrix = [
        1.0 - 2.0 * nx * nx,
        -2.0 * nx * ny,
        -2.0 * nx * nz,
        tx,
        -2.0 * ny * nx,
        1.0 - 2.0 * ny * ny,
        -2.0 * ny * nz,
        ty,
        -2.0 * nz * nx,
        -2.0 * nz * ny,
        1.0 - 2.0 * nz * nz,
        tz,
        0.0,
        0.0,
        0.0,
        1.0,
    ];
    let transform = AffineTransform::new(matrix)?;
    transform_brep(solid, transform, true)
}


pub(crate) fn multiply_elements(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    multiply_elements_seeded(a, b, 0.0)
}

fn multiply_elements_seeded(a: &[f64; 16], b: &[f64; 16], seed: f64) -> [f64; 16] {
    let mut out = [0.0f64; 16];
    for row in 0..4 {
        for col in 0..4 {
            let mut sum = seed;
            for k in 0..4 {
                sum += a[row * 4 + k] * b[k * 4 + col];
            }
            out[row * 4 + col] = sum;
        }
    }
    out
}

