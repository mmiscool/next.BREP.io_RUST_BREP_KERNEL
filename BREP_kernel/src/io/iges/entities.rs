//! Conversions between kernel NURBS geometry and IGES entities.
//!
//! Supported entity types:
//! * **126** Rational B-Spline Curve  ← [`NurbsCurve`] (export)
//! * **128** Rational B-Spline Surface ↔ [`NurbsSurface`]
//!
//! Import reads every other entity in [`super::to_step`].
//!
//! The kernel stores control points in weighted-homogeneous form (`Vec4`, with
//! `x,y,z` premultiplied by `w`); IGES stores plain Cartesian coordinates and a
//! separate weight array, so every control point is de-homogenized on export
//! and re-homogenized on import. Kernel knot vectors are full and clamped,
//! exactly the IGES form.

use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};

use super::reader::ParsedEntity;
use super::writer::{real_field, IgesEntity};

const WEIGHT_TOL: f64 = 1e-12;

// ===========================================================================
// Entity 126 — Rational B-Spline Curve
// ===========================================================================

/// Serialize a NURBS curve as an IGES type-126 entity. Coordinates are written
/// verbatim (model units); the caller decides units in the Global section.
pub(crate) fn curve_to_126(curve: &NurbsCurve) -> Result<IgesEntity, String> {
    let num = curve.control_points.len();
    if num < 2 {
        return Err("iges: curve has fewer than 2 control points".into());
    }
    let degree = curve.degree;
    let k = (num - 1) as i64;
    let expected_knots = num + degree + 1;
    if curve.knots.len() != expected_knots {
        return Err(format!(
            "iges: curve knot count {} != control points {} + degree {} + 1",
            curve.knots.len(),
            num,
            degree
        ));
    }
    let rational = curve
        .control_points
        .iter()
        .any(|c| (c.w - 1.0).abs() > WEIGHT_TOL);
    let [t0, t1] = curve.domain()?;

    let mut params: Vec<String> = Vec::new();
    params.push(k.to_string()); // K = upper index
    params.push((degree as i64).to_string()); // M = degree
    params.push("0".into()); // PROP1 planar (0 = nonplanar)
    params.push("0".into()); // PROP2 closed
    params.push(if rational { "0" } else { "1" }.into()); // PROP3 rational/poly
    params.push("0".into()); // PROP4 periodic
    for knot in &curve.knots {
        params.push(real_field(*knot));
    }
    for c in &curve.control_points {
        params.push(real_field(c.w));
    }
    for c in &curve.control_points {
        let p = c.point()?;
        params.push(real_field(p.x));
        params.push(real_field(p.y));
        params.push(real_field(p.z));
    }
    params.push(real_field(t0)); // V(0) start parameter
    params.push(real_field(t1)); // V(1) end parameter
    params.push(real_field(0.0)); // unit normal X (nonplanar → 0,0,0)
    params.push(real_field(0.0));
    params.push(real_field(0.0));

    Ok(IgesEntity::new(126, params))
}

// ===========================================================================
// Entity 128 — Rational B-Spline Surface
// ===========================================================================

fn flag(value: bool) -> String {
    if value { "1" } else { "0" }.into()
}

/// Whether the first and last control rows (u) and columns (v) coincide, to
/// within a relative 1e-9 of the net's size, weights included.
fn closed_directions(surface: &NurbsSurface) -> Result<(bool, bool), String> {
    let net = &surface.control_points;
    let mut points = Vec::with_capacity(net.len() * net[0].len());
    for row in net {
        for c in row {
            points.push(c.point()?);
        }
    }
    let (mut lo, mut hi) = (points[0], points[0]);
    for p in &points {
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let tolerance = 1e-9 * (1.0 + hi.sub(lo).length());
    let same = |a: &Vec4, b: &Vec4| -> Result<bool, String> {
        Ok(a.point()?.sub(b.point()?).length() <= tolerance && (a.w - b.w).abs() <= WEIGHT_TOL)
    };
    let (last_u, last_v) = (net.len() - 1, net[0].len() - 1);
    let mut closed_u = true;
    for j in 0..=last_v {
        closed_u &= same(&net[0][j], &net[last_u][j])?;
    }
    let mut closed_v = true;
    for row in net {
        closed_v &= same(&row[0], &row[last_v])?;
    }
    Ok((closed_u, closed_v))
}

/// Serialize a NURBS surface as an IGES type-128 entity.
pub(crate) fn surface_to_128(surface: &NurbsSurface) -> Result<IgesEntity, String> {
    let num_u = surface.control_points.len();
    if num_u < 2 {
        return Err("iges: surface has fewer than 2 rows of control points".into());
    }
    let num_v = surface.control_points[0].len();
    if num_v < 2 {
        return Err("iges: surface has fewer than 2 columns of control points".into());
    }
    let (du, dv) = (surface.degree_u, surface.degree_v);
    let k1 = (num_u - 1) as i64;
    let k2 = (num_v - 1) as i64;
    if surface.knots_u.len() != num_u + du + 1 {
        return Err("iges: surface u-knot count inconsistent with control net".into());
    }
    if surface.knots_v.len() != num_v + dv + 1 {
        return Err("iges: surface v-knot count inconsistent with control net".into());
    }
    let rational = surface
        .control_points
        .iter()
        .flatten()
        .any(|c| (c.w - 1.0).abs() > WEIGHT_TOL);
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;

    let mut params: Vec<String> = Vec::new();
    params.push(k1.to_string());
    params.push(k2.to_string());
    params.push((du as i64).to_string());
    params.push((dv as i64).to_string());
    // A net whose first and last rows coincide is closed in that direction.
    // It is also flagged periodic: the kernel's pcurves on such a carrier may
    // run past the domain and wrap, and OpenCASCADE only wraps a parameter on a
    // surface marked periodic (otherwise it extrapolates the net, which put
    // m0015's seam-crossing edges 2.8 mm off their face).
    let (closed_u, closed_v) = closed_directions(surface)?;
    params.push(flag(closed_u)); // PROP1 closed in u
    params.push(flag(closed_v)); // PROP2 closed in v
    params.push(if rational { "0" } else { "1" }.into()); // PROP3 rational/poly
    params.push(flag(closed_u)); // PROP4 periodic in u
    params.push(flag(closed_v)); // PROP5 periodic in v
    for knot in &surface.knots_u {
        params.push(real_field(*knot));
    }
    for knot in &surface.knots_v {
        params.push(real_field(*knot));
    }
    // Weights and control points: second index (v/j) outer, first (u/i) fastest.
    for j in 0..num_v {
        for i in 0..num_u {
            params.push(real_field(surface.control_points[i][j].w));
        }
    }
    for j in 0..num_v {
        for i in 0..num_u {
            let p = surface.control_points[i][j].point()?;
            params.push(real_field(p.x));
            params.push(real_field(p.y));
            params.push(real_field(p.z));
        }
    }
    params.push(real_field(u0));
    params.push(real_field(u1));
    params.push(real_field(v0));
    params.push(real_field(v1));

    Ok(IgesEntity::new(128, params))
}

/// Reconstruct a NURBS surface from an IGES type-128 entity, scaling 3D
/// coordinates by `coord_scale` (units → mm).
pub(crate) fn surface_from_128(entity: &ParsedEntity, coord_scale: f64) -> Result<NurbsSurface, String> {
    let k1 = entity.int(0)?;
    let k2 = entity.int(1)?;
    let du = entity.int(2)? as usize;
    let dv = entity.int(3)? as usize;
    if k1 < 1 || k2 < 1 {
        return Err(format!("iges 128: invalid upper indices K1={k1} K2={k2}"));
    }
    let num_u = (k1 + 1) as usize;
    let num_v = (k2 + 1) as usize;
    let u_knot_count = num_u + du + 1;
    let v_knot_count = num_v + dv + 1;
    let mut idx = 9; // after K1,K2,M1,M2,PROP1..PROP5
    let mut knots_u = Vec::with_capacity(u_knot_count);
    for _ in 0..u_knot_count {
        knots_u.push(entity.real(idx)?);
        idx += 1;
    }
    let mut knots_v = Vec::with_capacity(v_knot_count);
    for _ in 0..v_knot_count {
        knots_v.push(entity.real(idx)?);
        idx += 1;
    }
    let count = num_u * num_v;
    let mut weights = Vec::with_capacity(count);
    for _ in 0..count {
        weights.push(entity.real(idx)?);
        idx += 1;
    }
    // Read the point stream (v outer, u fastest) into the kernel grid [i][j].
    let mut control_points = vec![vec![Vec4::from_point(Vec3::new(0.0, 0.0, 0.0), 1.0); num_v]; num_u];
    let mut w_index = 0;
    for j in 0..num_v {
        for i in 0..num_u {
            let x = entity.real(idx)? * coord_scale;
            let y = entity.real(idx + 1)? * coord_scale;
            let z = entity.real(idx + 2)? * coord_scale;
            idx += 3;
            control_points[i][j] = Vec4::from_point(Vec3::new(x, y, z), weights[w_index]);
            w_index += 1;
        }
    }
    NurbsSurface::new(du, dv, knots_u, knots_v, control_points)
}
