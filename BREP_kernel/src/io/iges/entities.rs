//! Conversions between kernel NURBS geometry and IGES entities.
//!
//! Supported entity types:
//! * **126** Rational B-Spline Curve  ↔ [`NurbsCurve`]
//! * **128** Rational B-Spline Surface ↔ [`NurbsSurface`]
//! * **102** Composite Curve (a loop of 126 curves)
//! * **142** Curve on a Parametric Surface (trimming curve: model + parameter)
//! * **144** Trimmed (Parametric) Surface (one kernel face)
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

/// Reconstruct a NURBS curve from an IGES type-126 entity. When `coord_scale`
/// is `Some(s)`, the 3D coordinates are multiplied by `s` (units → mm); pass
/// `None` for parameter-space (pcurve) curves, whose `(u, v)` values must not
/// be scaled.
pub(crate) fn curve_from_126(entity: &ParsedEntity, coord_scale: Option<f64>) -> Result<NurbsCurve, String> {
    let k = entity.int(0)?;
    let degree = entity.int(1)? as usize;
    if k < 1 {
        return Err(format!("iges 126: invalid upper index K={k}"));
    }
    let num = (k + 1) as usize;
    let knot_count = num + degree + 1;
    let mut idx = 6; // after K, M, PROP1..PROP4
    let mut knots = Vec::with_capacity(knot_count);
    for _ in 0..knot_count {
        knots.push(entity.real(idx)?);
        idx += 1;
    }
    let mut weights = Vec::with_capacity(num);
    for _ in 0..num {
        weights.push(entity.real(idx)?);
        idx += 1;
    }
    let scale = coord_scale.unwrap_or(1.0);
    let mut control_points = Vec::with_capacity(num);
    for i in 0..num {
        let x = entity.real(idx)? * scale;
        let y = entity.real(idx + 1)? * scale;
        let z = entity.real(idx + 2)? * scale;
        idx += 3;
        control_points.push(Vec4::from_point(Vec3::new(x, y, z), weights[i]));
    }
    NurbsCurve::new(degree, knots, control_points)
}

// ===========================================================================
// Entity 128 — Rational B-Spline Surface
// ===========================================================================

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
    params.push("0".into()); // PROP1 closed in u
    params.push("0".into()); // PROP2 closed in v
    params.push(if rational { "0" } else { "1" }.into()); // PROP3 rational/poly
    params.push("0".into()); // PROP4 periodic in u
    params.push("0".into()); // PROP5 periodic in v
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

// ===========================================================================
// Entity 102 — Composite Curve
// ===========================================================================

/// A composite curve groups constituent curve entities (their DE pointers) into
/// a single ordered boundary.
pub(crate) fn composite_102(members: &[i64]) -> IgesEntity {
    let mut params: Vec<String> = Vec::with_capacity(members.len() + 1);
    params.push(members.len().to_string());
    for m in members {
        params.push(m.to_string());
    }
    IgesEntity::new(102, params)
}

/// Read the ordered member DE pointers of a composite curve.
pub(crate) fn composite_members(entity: &ParsedEntity) -> Result<Vec<i64>, String> {
    let n = entity.int(0)? as usize;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        out.push(entity.int(1 + i)?);
    }
    Ok(out)
}

// ===========================================================================
// Entity 142 — Curve on a Parametric Surface
// ===========================================================================

/// Build a curve-on-surface entity: `sptr` = surface DE pointer, `bptr` = the
/// boundary curve in parameter space (a 102/126 in `(u, v, 0)`), `cptr` = the
/// same boundary in model space (a 102/126 in 3D). PREF=1 (parameter space is
/// authoritative).
pub(crate) fn curve_on_surface_142(sptr: i64, bptr: i64, cptr: i64) -> IgesEntity {
    IgesEntity::new(
        142,
        vec![
            "0".into(),         // CRTN: how the curve was created (unspecified)
            sptr.to_string(),   // SPTR: surface
            bptr.to_string(),   // BPTR: parameter-space curve
            cptr.to_string(),   // CPTR: model-space curve
            "1".into(),         // PREF: prefer parameter space
        ],
    )
}

/// Parsed 142: (surface ptr, parameter-space curve ptr, model-space curve ptr).
pub(crate) struct CurveOnSurface {
    pub surface: i64,
    pub param_curve: i64,
    pub model_curve: i64,
}

pub(crate) fn curve_on_surface_from_142(entity: &ParsedEntity) -> Result<CurveOnSurface, String> {
    Ok(CurveOnSurface {
        surface: entity.int(1)?,
        param_curve: entity.int(2)?,
        model_curve: entity.int(3)?,
    })
}

// ===========================================================================
// Entity 144 — Trimmed (Parametric) Surface
// ===========================================================================

/// Build a trimmed surface: `pts` = surface DE pointer, `outer` = outer
/// boundary (142) DE pointer, `holes` = inner boundary (142) DE pointers.
pub(crate) fn trimmed_surface_144(pts: i64, outer: i64, holes: &[i64]) -> IgesEntity {
    let mut params: Vec<String> = Vec::new();
    params.push(pts.to_string()); // PTS surface
    params.push("1".into()); // N1: outer boundary is given by PTO
    params.push(holes.len().to_string()); // N2: number of inner boundaries
    params.push(outer.to_string()); // PTO outer boundary
    for h in holes {
        params.push(h.to_string());
    }
    IgesEntity::new(144, params)
}

/// Parsed 144: surface pointer, outer boundary pointer, inner boundary pointers.
pub(crate) struct TrimmedSurface {
    pub surface: i64,
    pub outer: Option<i64>,
    pub inner: Vec<i64>,
}

pub(crate) fn trimmed_surface_from_144(entity: &ParsedEntity) -> Result<TrimmedSurface, String> {
    let surface = entity.int(0)?;
    let n1 = entity.int(1)?;
    let n2 = entity.int(2)? as usize;
    let outer = entity.int(3)?;
    let outer = if n1 != 0 && outer != 0 { Some(outer) } else { None };
    let mut inner = Vec::with_capacity(n2);
    for i in 0..n2 {
        inner.push(entity.int(4 + i)?);
    }
    Ok(TrimmedSurface {
        surface,
        outer,
        inner,
    })
}
