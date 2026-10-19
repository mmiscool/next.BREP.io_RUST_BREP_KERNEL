//! Canonical carriers for imported faces (`BREP_CANONICAL_CARRIERS`).
//!
//! The recogniser (`recognition.rs`) takes an analytic carrier only in the
//! parametrisation the kernel's own constructors produce: the circle along
//! the FIRST control index, every knot vector on [0, 1], a plane as an
//! affine patch. More than four hundred `analytic()` readers rely on that
//! parametrisation, so an imported face that is exactly analytic in another
//! parametrisation is rewritten here, once, at import, rather than recognised
//! in place:
//!
//! - **knots** (C1): an affine knot map onto [0, 1];
//! - **transpose** (C2): the circle along the second control index; the net
//!   is transposed, which flips `Su × Sv`, so the face's `same_sense` flips
//!   with it and every outward normal is unchanged;
//! - **plane** (C3): a planar face on a non-affine patch (a collapsed-apex
//!   bilinear triangle, a planar bicubic) gets an affine plane spanned on its
//!   own plane; each pcurve becomes the affine image of its planar 3D edge.
//!
//! C1 and C2 keep the control net bit-identical and map each pcurve by the
//! same affine map / coordinate swap, so the face's point set and every
//! coedge's 3D trace are unchanged; the new carrier must pass today's exact
//! recogniser. C3 is taken only when the old carrier lies on the plane to
//! 1e-9·scale over dense samples and every trimming edge's control points do
//! too. Every rewrite is checked by sampling each coedge on the new carrier
//! against the old one, and a face that fails any check stays as it came.
//! Names and ids are never touched.

use crate::topology::{BrepSolid, EdgeRecord, FaceRecord, LoopRecord};
use crate::{make_plane, NurbsCurve, NurbsSurface, Vec3, Vec4};

/// One face's rewrite, for the import report and the tests.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CanonicalRewrite {
    Knots,
    Transpose,
    TransposeKnots,
    Plane,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct CanonicalReport {
    /// `(face id, rewrite)` for every face rewritten.
    pub rewritten: Vec<(u64, CanonicalRewrite)>,
}

/// Whether the import lanes canonicalise carriers: on unless
/// `BREP_CANONICAL_CARRIERS=0` restores the verbatim import.
pub(crate) fn canonical_carriers_enabled() -> bool {
    std::env::var("BREP_CANONICAL_CARRIERS").as_deref() != Ok("0")
}

const PLANE_TOLERANCE: f64 = 1e-9;
const TRACE_TOLERANCE: f64 = 1e-12;
const TRACE_SAMPLES: usize = 32;
const SURFACE_SAMPLES: usize = 16;

/// Rewrites every non-canonical, exactly analytic face of `solid` in place.
pub(crate) fn canonicalise_carriers(solid: &mut BrepSolid) -> CanonicalReport {
    let mut report = CanonicalReport::default();
    let edges: Vec<EdgeRecord> = solid.edges.clone();
    for face in solid.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        if face.surface.analytic().is_some() {
            continue;
        }
        if let Some(rewrite) = reparametrise(face).or_else(|| replane(face, &edges)) {
            report.rewritten.push((face.id, rewrite));
        }
    }
    if !report.rewritten.is_empty() {
        solid.mass_properties_cache = Default::default();
    }
    report
}

fn surface_scale(surface: &NurbsSurface) -> f64 {
    let mut lo = Vec3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut hi = Vec3::new(f64::MIN, f64::MIN, f64::MIN);
    for control in surface.control_points.iter().flatten() {
        let p = Vec3::new(control.x / control.w, control.y / control.w, control.z / control.w);
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    hi.sub(lo).length().max(1.0)
}

/// The affine map of one knot vector onto [0, 1] as `(offset, extent)`.
fn knot_span(knots: &[f64]) -> Option<(f64, f64)> {
    let (first, last) = (*knots.first()?, *knots.last()?);
    let extent = last - first;
    (extent > 0.0).then_some((first, extent))
}

fn normalised_knots(knots: &[f64], (offset, extent): (f64, f64)) -> Vec<f64> {
    let last = offset + extent;
    // The end knots map exactly; the interior keeps the map's rounding.
    knots
        .iter()
        .map(|&knot| if knot == offset { 0.0 } else if knot == last { 1.0 } else { (knot - offset) / extent })
        .collect()
}

/// C1 / C2: the same control net under an affine knot map and/or a transpose.
fn reparametrise(face: &mut FaceRecord) -> Option<CanonicalRewrite> {
    let surface = &face.surface;
    let span_u = knot_span(&surface.knots_u)?;
    let span_v = knot_span(&surface.knots_v)?;
    let unit = |(offset, extent): (f64, f64)| offset == 0.0 && extent == 1.0;
    let needs_knots = !unit(span_u) || !unit(span_v);
    let candidates: &[(bool, bool, CanonicalRewrite)] = &[
        (false, true, CanonicalRewrite::Knots),
        (true, false, CanonicalRewrite::Transpose),
        (true, true, CanonicalRewrite::TransposeKnots),
    ];
    for &(transpose, knots, rewrite) in candidates {
        if knots && !needs_knots {
            continue;
        }
        let (knots_u, knots_v) = if knots {
            (normalised_knots(&surface.knots_u, span_u), normalised_knots(&surface.knots_v, span_v))
        } else {
            (surface.knots_u.clone(), surface.knots_v.clone())
        };
        let candidate = if transpose {
            let (rows, columns) = (surface.control_points.len(), surface.control_points[0].len());
            let net = (0..columns)
                .map(|j| (0..rows).map(|i| surface.control_points[i][j]).collect())
                .collect();
            NurbsSurface::new(surface.degree_v, surface.degree_u, knots_v, knots_u, net)
        } else {
            NurbsSurface::new(surface.degree_u, surface.degree_v, knots_u, knots_v, surface.control_points.clone())
        };
        let Ok(candidate) = candidate else { continue };
        match candidate.analytic() {
            None => continue,
            // A sphere stays as it came until the offset shell reads a
            // transposed sphere the way it reads the NURBS one (lane S,
            // smOffsetTest_ShellBrep_814).
            Some(crate::AnalyticSurface::Sphere { .. }) => continue,
            Some(_) => {}
        }
        let (offset_u, extent_u) = if knots { span_u } else { (0.0, 1.0) };
        let (offset_v, extent_v) = if knots { span_v } else { (0.0, 1.0) };
        let to_candidate = |u: f64, v: f64| {
            let (u, v) = ((u - offset_u) / extent_u, (v - offset_v) / extent_v);
            if transpose { (v, u) } else { (u, v) }
        };
        // The same net under a parameter bijection: the point set must agree
        // over the whole patch, not only along the trims.
        let tolerance = TRACE_TOLERANCE * surface_scale(surface);
        let same_points = (0..=SURFACE_SAMPLES).all(|i| {
            (0..=SURFACE_SAMPLES).all(|j| {
                let u = span_u.0 + span_u.1 * i as f64 / SURFACE_SAMPLES as f64;
                let v = span_v.0 + span_v.1 * j as f64 / SURFACE_SAMPLES as f64;
                let (cu, cv) = to_candidate(u, v);
                match (surface.evaluate(u, v), candidate.evaluate(cu, cv)) {
                    (Ok(a), Ok(b)) => a.sub(b).length() <= tolerance,
                    _ => false,
                }
            })
        });
        if !same_points {
            continue;
        }
        let map = |control: Vec4| {
            let u = (control.x - offset_u * control.w) / extent_u;
            let v = (control.y - offset_v * control.w) / extent_v;
            let (u, v) = if transpose { (v, u) } else { (u, v) };
            Vec4 { x: u, y: v, z: 0.0, w: control.w }
        };
        let Some(loops) = mapped_loops(face, &candidate, |pcurve| {
            NurbsCurve::new(pcurve.degree, pcurve.knots.clone(), pcurve.control_points.iter().copied().map(map).collect()).ok()
        }) else {
            continue;
        };
        face.surface = candidate;
        face.loops = loops;
        if transpose {
            face.same_sense = !face.same_sense;
        }
        return Some(rewrite);
    }
    None
}

/// The face's loops with each pcurve replaced by `remap(pcurve)`, accepted
/// only when every coedge traces the same 3D points on `carrier` as it did on
/// the face's own surface (≤ 1e-12·scale at 33 fractions).
fn mapped_loops(
    face: &FaceRecord,
    carrier: &NurbsSurface,
    remap: impl Fn(&NurbsCurve) -> Option<NurbsCurve>,
) -> Option<Vec<LoopRecord>> {
    let tolerance = TRACE_TOLERANCE * surface_scale(&face.surface);
    let mut loops = face.loops.clone();
    for record in &mut loops {
        for coedge in &mut record.coedges {
            let mapped = remap(&coedge.pcurve)?;
            let [a0, a1] = coedge.pcurve.domain().ok()?;
            let [b0, b1] = mapped.domain().ok()?;
            for index in 0..=TRACE_SAMPLES {
                let fraction = index as f64 / TRACE_SAMPLES as f64;
                let old = coedge.pcurve.evaluate(a0 + (a1 - a0) * fraction).ok()?;
                let new = mapped.evaluate(b0 + (b1 - b0) * fraction).ok()?;
                let before = face.surface.evaluate_extended(old.x, old.y).ok()?;
                let after = carrier.evaluate_extended(new.x, new.y).ok()?;
                if before.sub(after).length() > tolerance {
                    return None;
                }
            }
            coedge.pcurve = mapped;
        }
    }
    Some(loops)
}

/// C3: a planar face on a non-affine patch gets an affine plane carrier.
fn replane(face: &mut FaceRecord, edges: &[EdgeRecord]) -> Option<CanonicalRewrite> {
    let surface = &face.surface;
    let scale = surface_scale(surface);
    let tolerance = PLANE_TOLERANCE * scale;
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let mut points = Vec::new();
    let mut normal_sum = Vec3::new(0.0, 0.0, 0.0);
    for i in 0..=SURFACE_SAMPLES {
        for j in 0..=SURFACE_SAMPLES {
            let u = u0 + (u1 - u0) * i as f64 / SURFACE_SAMPLES as f64;
            let v = v0 + (v1 - v0) * j as f64 / SURFACE_SAMPLES as f64;
            points.push(surface.evaluate(u, v).ok()?);
            if let Ok(derivatives) = surface.derivatives(u, v, 1) {
                normal_sum = normal_sum.add(derivatives[1][0].cross(derivatives[0][1]));
            }
        }
    }
    // The parametrisation's own normal, summed over the face: it fixes the
    // plane's sense, so `same_sense` keeps its meaning.
    let normal = normal_sum.normalized().ok()?;
    let origin = points.iter().fold(Vec3::new(0.0, 0.0, 0.0), |sum, p| sum.add(*p)).scale(1.0 / points.len() as f64);
    if points.iter().any(|p| p.sub(origin).dot(normal).abs() > tolerance) {
        return None;
    }
    // Every trimming edge must lie on the plane exactly, or its pcurve could
    // not be the affine image of its 3D curve.
    let face_edges: Vec<&EdgeRecord> = face
        .loops
        .iter()
        .flat_map(|record| &record.coedges)
        .map(|coedge| edges.iter().find(|edge| edge.id == coedge.edge_id))
        .collect::<Option<_>>()?;
    for edge in &face_edges {
        for control in &edge.curve.control_points {
            let p = Vec3::new(control.x / control.w, control.y / control.w, control.z / control.w);
            if p.sub(origin).dot(normal).abs() > tolerance {
                return None;
            }
        }
    }
    let seed = [Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)]
        .into_iter()
        .min_by(|a, b| a.dot(normal).abs().total_cmp(&b.dot(normal).abs()))?;
    let u_dir = seed.sub(normal.scale(seed.dot(normal))).normalized().ok()?;
    let v_dir = normal.cross(u_dir);
    let (mut lo_u, mut hi_u, mut lo_v, mut hi_v) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
    for p in &points {
        let d = p.sub(origin);
        lo_u = lo_u.min(d.dot(u_dir));
        hi_u = hi_u.max(d.dot(u_dir));
        lo_v = lo_v.min(d.dot(v_dir));
        hi_v = hi_v.max(d.dot(v_dir));
    }
    let pad = 0.01 * (hi_u - lo_u).max(hi_v - lo_v);
    let corner = origin.add(u_dir.scale(lo_u - pad)).add(v_dir.scale(lo_v - pad));
    let plane = make_plane(corner, u_dir, v_dir, hi_u - lo_u + 2.0 * pad, hi_v - lo_v + 2.0 * pad).ok()?;
    if !matches!(plane.analytic(), Some(crate::AnalyticSurface::Plane { .. })) {
        return None;
    }
    let to_plane = |control: Vec4| {
        let d = Vec3::new(control.x, control.y, control.z).sub(corner.scale(control.w));
        Vec4 { x: d.dot(u_dir), y: d.dot(v_dir), z: 0.0, w: control.w }
    };
    let mut loops = face.loops.clone();
    for record in &mut loops {
        for coedge in &mut record.coedges {
            let edge = edges.iter().find(|edge| edge.id == coedge.edge_id)?;
            let mut curve = crate::step::edge_subcurve(edge).ok()?;
            if !coedge.forward {
                curve = curve.reversed().ok()?;
            }
            let pcurve = NurbsCurve::new(curve.degree, curve.knots.clone(), curve.control_points.iter().copied().map(to_plane).collect()).ok()?;
            // The new pcurve must trace its edge on the plane, fraction for
            // fraction, the way validation reads a coedge.
            let [b0, b1] = pcurve.domain().ok()?;
            for index in 0..=TRACE_SAMPLES {
                let fraction = index as f64 / TRACE_SAMPLES as f64;
                let t = if coedge.forward {
                    edge.t0 + (edge.t1 - edge.t0) * fraction
                } else {
                    edge.t1 - (edge.t1 - edge.t0) * fraction
                };
                let on_edge = edge.curve.evaluate(t).ok()?;
                let uv = pcurve.evaluate(b0 + (b1 - b0) * fraction).ok()?;
                if plane.evaluate(uv.x, uv.y).ok()?.sub(on_edge).length() > TRACE_TOLERANCE * scale {
                    return None;
                }
            }
            coedge.pcurve = pcurve;
        }
    }
    face.surface = plane;
    face.loops = loops;
    Some(CanonicalRewrite::Plane)
}
