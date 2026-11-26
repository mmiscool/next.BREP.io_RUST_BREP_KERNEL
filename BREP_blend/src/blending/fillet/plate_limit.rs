//! The full-round limit of a convex polygonal prism: erode its outline by
//! the ball radius, then offset that planar region by the ball in space.
//! The boundary has two planar caps, half cylinders on its edges and
//! spherical wedges at its corners. No zero-length blend strip is minted.
use crate::{BrepSolid, KernelRefusal, KernelStage, OrRefuse, Vec3};
use std::f64::consts::FRAC_PI_2;

pub(super) fn mode() -> crate::env_read::Mode {
    crate::mode_switch!("BREP_FILLET_PLATE_LIMIT", true)
}

fn unsupported(what: &'static str, message: &str) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, message)
}

/// Decline anything except a complete selection on a straight-sided prism
/// whose thickness equals the ball diameter to arithmetic precision.
pub(super) fn build(solid: &BrepSolid, picks: &[Vec3], radius: f64) -> Option<Result<BrepSolid, KernelRefusal>> {
    if solid.shells.len() != 1 || solid.vertices.is_empty() || !radius.is_finite() || radius <= 0.0 {
        return None;
    }
    let faces = &solid.shells[0].faces;
    if faces.len() < 5 || picks.len() != solid.edges.len() { return None; }
    let mut selected = picks.iter().map(|p| super::edges::resolve_edge_by_point(solid, *p).ok()).collect::<Option<Vec<_>>>()?;
    selected.sort_unstable(); selected.dedup();
    if selected.len() != solid.edges.len() || solid.edges.iter().any(|e| e.degenerate || e.curve.degree != 1 || e.curve.control_points.len() != 2) {
        return None;
    }
    if faces.iter().any(|f| f.loops.len() != 1) { return None; }
    let planes = faces.iter().map(super::full_round::plane_of).collect::<Option<Vec<_>>>()?;
    let origin = solid.vertices[0].point;
    let scale = solid.vertices.iter().map(|v| v.point.sub(origin).length()).fold(0.0, f64::max);
    // Subtracting translated coordinates loses absolute precision even
    // though the dimensions are local. Account for that rounding, then
    // decline if it would make the directional reading poorly resolved.
    let coordinate = solid.vertices.iter().map(|v| v.point.x.abs().max(v.point.y.abs()).max(v.point.z.abs())).fold(0.0, f64::max);
    let band = 128.0 * f64::EPSILON * (scale + coordinate);
    let angular = (band / radius).max(1e-12);
    if angular > 1e-8 { return None; }
    if !(radius > 100.0 * band && scale.is_finite()) { return None; }
    for i in 0..faces.len() {
        let (normal, on) = planes[i];
        for j in i+1..faces.len() {
            let (opposite, end) = planes[j];
            if normal.add(opposite).length() > angular || (normal.dot(on.sub(end)) - 2.0 * radius).abs() > band { continue; }
            if planes.iter().enumerate().any(|(k,(n,_))| k != i && k != j && n.dot(normal).abs() > angular) { continue; }
            let coedges = &faces[i].loops[0].coedges;
            let count = coedges.len();
            if count < 3 || faces.len() != count + 2 || solid.vertices.len() != 2 * count || solid.edges.len() != 3 * count { continue; }
            let mut points = Vec::new();
            for c in coedges {
                let edge = solid.edges.iter().find(|e| e.id == c.edge_id)?;
                points.push(edge.curve.evaluate(if c.forward {edge.t0} else {edge.t1}).ok()?);
            }
            // Every vertex belongs to one of the matching end polygons;
            // every control point lies on its edge's vertex segment.
            let matches = solid.vertices.iter().all(|v| points.iter().any(|p| v.point.sub(*p).length() <= band || v.point.sub(p.sub(normal.scale(2.0*radius))).length() <= band));
            if !matches || points.iter().any(|p| normal.dot(p.sub(on)).abs() > band) { continue; }
            if solid.edges.iter().any(|e| {
                let a = solid.vertices.iter().find(|v| v.id == e.start_vertex_id).map(|v|v.point);
                let b = solid.vertices.iter().find(|v| v.id == e.end_vertex_id).map(|v|v.point);
                match (a,b,e.curve.evaluate(e.t0),e.curve.evaluate(e.t1)) {
                    (Some(a),Some(b),Ok(c),Ok(d)) => a.sub(c).length() > band || b.sub(d).length() > band,
                    _ => true,
                }
            }) { continue; }
            let anchor = points[0];
            let area = (0..count).map(|k| points[k].sub(anchor).cross(points[(k+1)%count].sub(anchor)).dot(normal)).sum::<f64>();
            if area < 0.0 { points.reverse(); }
            for p in &mut points { *p = p.sub(normal.scale(radius)); }
            return match inset_and_sew(&points, normal, radius, band) {
                Ok(Some(mut body)) => {
                    // The end planes survive with smaller trims; retain their names.
                    for face in body.shells.iter_mut().flat_map(|s| &mut s.faces) {
                        if let Some((n, _)) = super::full_round::plane_of(face) {
                            face.name = if n.dot(normal) > 0.0 { faces[i].name.clone() } else { faces[j].name.clone() };
                        }
                    }
                    Some(Ok(body))
                },
                Ok(None) => None,
                Err(error) => Some(Err(error)),
            };
        }
    }
    None
}

pub(super) fn inset_and_sew(points: &[Vec3], normal: Vec3, radius: f64, band: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let count = points.len();
    let tangents = (0..count).map(|i| points[(i+1)%count].sub(points[i]).normalized()).collect::<Result<Vec<_>,_>>().or_refuse(KernelStage::Classify,"plate_limit_outline")?;
    if (0..count).any(|i| tangents[i].cross(tangents[(i+1)%count]).dot(normal) <= 1e-12) {
        return Err(unsupported("plate_limit_concave", "full-round plate: concave or straight-through corners are not supported"));
    }
    let inward: Vec<_> = tangents.iter().map(|t| normal.cross(*t)).collect();
    // All vertices must be on the inner side of every edge; local turns
    // alone do not rule out a self-crossing polygon.
    if (0..count).any(|i| points.iter().any(|p| p.sub(points[i]).dot(inward[i]) < -band)) {
        return Err(unsupported("plate_limit_outline", "full-round plate: the outline is not convex"));
    }
    let inset: Vec<_> = (0..count).map(|i| {
        let a = inward[(i+count-1)%count]; let b = inward[i];
        points[i].add(a.add(b).scale(radius / (1.0 + a.dot(b))))
    }).collect();
    if (0..count).any(|i| inset[(i+1)%count].sub(inset[i]).dot(tangents[i]) <= band || inset.iter().any(|p| p.sub(points[i]).dot(inward[i]) < radius-band)) {
        return Ok(None); // An eroded edge needs a different limit topology.
    }
    sew(&inset, normal, radius).map(Some)
}

fn sew(points: &[Vec3], normal: Vec3, radius: f64) -> Result<BrepSolid, KernelRefusal> {
    let count = points.len();
    let mut sheets = Vec::new();
    for i in 0..count {
        let a = points[i]; let b = points[(i+1)%count];
        let out = b.sub(a).normalized().or_refuse(KernelStage::Sew,"plate_limit_edge")?.cross(normal);
        let before = a.sub(points[(i+count-1)%count]).normalized().or_refuse(KernelStage::Sew,"plate_limit_edge")?.cross(normal);
        let arc = crate::make_arc(a,out,normal,radius,-FRAC_PI_2,FRAC_PI_2).or_refuse(KernelStage::Sew,"plate_limit_arc")?;
        let cylinder = crate::make_extrusion(&arc,b.sub(a)).or_refuse(KernelStage::Sew,"plate_limit_cylinder")?;
        sheets.push(crate::face_from_surface(&cylinder)?);
        let arc = crate::make_arc(a,before,normal,radius,-FRAC_PI_2,FRAC_PI_2).or_refuse(KernelStage::Sew,"plate_limit_arc")?;
        let angle = before.cross(out).dot(normal).atan2(before.dot(out));
        let corner = crate::make_revolution(a,normal,&arc,angle).or_refuse(KernelStage::Sew,"plate_limit_corner")?;
        sheets.push(crate::face_from_surface(&corner)?);
    }
    let origin = points[0];
    let x = points[1].sub(origin).normalized().or_refuse(KernelStage::Sew,"plate_limit_frame")?;
    let y = normal.cross(x);
    let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for p in points {
        let d = p.sub(origin); let uv = [d.dot(x),d.dot(y)];
        for k in 0..2 {lo[k]=lo[k].min(uv[k]);hi[k]=hi[k].max(uv[k]);}
    }
    for sign in [-1.0,1.0] {
        let shift = normal.scale(sign*radius);
        let rim = (0..count).map(|i| crate::make_line(points[i].add(shift),points[(i+1)%count].add(shift))).collect::<Result<Vec<_>,_>>().or_refuse(KernelStage::Sew,"plate_limit_rim")?;
        let corner = origin.add(shift).add(x.scale(lo[0]-radius)).add(y.scale(lo[1]-radius));
        let plane = crate::make_plane(corner,x,y,hi[0]-lo[0]+2.0*radius,hi[1]-lo[1]+2.0*radius).or_refuse(KernelStage::Sew,"plate_limit_cap")?;
        sheets.push(crate::face_from_surface_trimmed(&plane,&[rim])?);
    }
    let size = points.iter().map(|p|p.sub(origin).length()).fold(radius,f64::max);
    let tolerance = (256.0*f64::EPSILON*size).max(1e-7);
    if radius <= 100.0*tolerance {
        return Err(unsupported("plate_limit_scale", "full-round plate: the radius is inside the sewing resolution"));
    }
    crate::solid_from_sheets(&sheets,tolerance)
}

