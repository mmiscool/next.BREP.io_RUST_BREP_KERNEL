use super::*;

/// A full pointed cone is the union of straight generators from its rim to
/// its apex. Triangulate those generators directly: positive UV area does not
/// imply positive 3D area near the collapsed pole, and metric-UV flips/refines
/// can fold skinny triangles there even when their chord sag is small.
///
/// Only accept the complete seam rectangle with one collapsed cross boundary
/// and no intermediate seam stations. A trimmed wall, hole, frustum, or shared
/// subdivided seam keeps the general path. The rim positions are the original
/// shared-edge samples. Fan edges follow the straight generators; deviation
/// inside a wedge is bounded by its rim sag and certified pole residual.
pub(super) fn tessellate_pointed_cone_fan(
    face: &FaceRecord,
    polygon: &[FaceVertex],
    domain: [f64; 4],
    chord_tolerance: f64,
    face_id: u32,
    mesh: &mut Mesh,
) -> Result<bool, String> {
    if !matches!(
        face.surface.analytic(),
        Some(crate::AnalyticSurface::RuledRevolution { .. })
    ) {
        return Ok(false);
    }
    // Analytic recognition allows reconstruction residuals; it is only a
    // dispatch hint, not proof that the authored carrier collapses. Likewise,
    // periodic trim reconstruction copies one pole sample into both corners.
    // Certify the ENTIRE authored net instead. Equal positive weights along
    // each degree-one generator make S(u,v) an affine blend of its rim curve
    // and pole. Check the pole in homogeneous coordinates to preserve exact
    // native construction identities even on translated/rotated models.
    let surface = &face.surface;
    if surface.degree_v != 1
        || surface.knots_v.as_slice() != &[0.0, 0.0, 1.0, 1.0]
        || surface.control_points.iter().any(|row| {
            row.len() != 2 || !row[0].w.is_finite() || row[0].w <= 0.0 || row[0].w != row[1].w
        })
    {
        return Ok(false);
    }
    let collapsed = |end: usize| {
        let Some(pole) = polygon.iter().find(|p| p.uv[1] == end as f64) else {
            return false;
        };
        surface.control_points.iter().all(|row| {
            let p = row[end];
            p.x == pole.position.x * p.w
                && p.y == pole.position.y * p.w
                && p.z == pole.position.z * p.w
                // Multiplication followed by dehomogenization can round by an
                // ulp on translated models. Bound even that represented pole
                // residual by the requested chord, never a coordinate-scaled
                // recognition allowance. Positive weights bound the whole
                // pole curve by its controls. Since v is affine, the wedge
                // error is <= max(rim sag, pole residual), not their sum.
                && Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w)
                    .sub(pole.position).length() <= chord_tolerance
        })
    };
    let (rim_v, pole_v) = if collapsed(1) && !collapsed(0) {
        (0.0, 1.0)
    } else if collapsed(0) && !collapsed(1) {
        (1.0, 0.0)
    } else {
        return Ok(false);
    };
    // The analytic ruled-revolution contract uses this full unit domain.
    if domain != [0.0, 1.0, 0.0, 1.0] || !polygon_is_simple(polygon) {
        return Ok(false);
    }
    // Exact chart boundaries are intentional: nearby trims must not be filled
    // or snapped to the rim/pole. Reject interior seam samples rather than
    // orphaning their neighbours when replacing the seam by one generator.
    for (a, b) in polygon.iter().zip(polygon.iter().cycle().skip(1)) {
        if ![rim_v, pole_v].contains(&a.uv[1]) || !(0.0..=1.0).contains(&a.uv[0]) {
            return Ok(false);
        }
        if a.uv[1] != b.uv[1] && !(a.uv[0] == b.uv[0] && [0.0, 1.0].contains(&a.uv[0])) {
            return Ok(false);
        }
    }
    if (signed_area(polygon).abs() - 1.0).abs() > 1e-12 {
        return Ok(false);
    }
    let mut rim: Vec<_> = polygon.iter().filter(|p| p.uv[1] == rim_v).collect();
    rim.sort_by(|a, b| a.uv[0].total_cmp(&b.uv[0]));
    if rim.len() < 5 || rim[0].uv[0] != 0.0 || rim.last().unwrap().uv[0] != 1.0 {
        return Ok(false);
    }
    // No duplicate stations or major-arc wedges. Shared-edge sampling normally
    // provides many more than four spans, even at coarse tolerances.
    if rim
        .windows(2)
        .any(|p| p[1].uv[0] <= p[0].uv[0] || p[1].uv[0] - p[0].uv[0] > 0.25)
    {
        return Ok(false);
    }
    let poles: Vec<_> = polygon.iter().filter(|p| p.uv[1] == pole_v).collect();
    let Some(pole) = poles.first() else {
        return Ok(false);
    };
    if poles
        .iter()
        .any(|p| p.position.sub(pole.position).length() != 0.0)
    {
        return Ok(false);
    }
    let base = (mesh.positions.len() / 3) as u32;
    for vertex in &rim {
        let normal = face_normal_at(face, vertex.uv, domain)?;
        mesh.positions
            .extend([vertex.position.x, vertex.position.y, vertex.position.z]);
        mesh.normals.extend([normal.x, normal.y, normal.z]);
    }
    for (i, pair) in rim.windows(2).enumerate() {
        // The apex has no unique normal; each wedge uses its own limiting
        // meridian normal while all copies retain the exact same position.
        let normal = face_normal_at(
            face,
            [(pair[0].uv[0] + pair[1].uv[0]) * 0.5, pole_v],
            domain,
        )?;
        let apex = (mesh.positions.len() / 3) as u32;
        mesh.positions
            .extend([pole.position.x, pole.position.y, pole.position.z]);
        mesh.normals.extend([normal.x, normal.y, normal.z]);
        let a = base + i as u32;
        let b = a + 1;
        if face.same_sense == (pole_v > rim_v) {
            mesh.indices.extend([a, b, apex]);
        } else {
            mesh.indices.extend([b, a, apex]);
        }
        mesh.face_ids.push(face_id);
    }
    Ok(true)
}

