//! Solids bounded by plane polygons: any closed set of them, and the pyramid
//! on a polygon.
use crate::{face_from_surface_trimmed, make_line, make_plane, solid_from_sheets, BrepSolid, KernelRefusal, KernelStage, OrRefuse, Vec3};

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, message)
}

/// The sheet of one flat face inside the polygon `corners` (in order, either
/// way round), which must lie in one plane.
fn polygon_face(corners: &[Vec3], index: usize) -> Result<BrepSolid, KernelRefusal> {
    if corners.len() < 3 {
        return Err(input("polygon_corners", format!("solidFromPolygons: polygon {index} has fewer than three corners")));
    }
    let origin = corners[0];
    let arms: Vec<Vec3> = corners.iter().map(|corner| corner.sub(origin)).collect();
    let size = arms.iter().map(|arm| arm.length()).fold(0.0, f64::max);
    let flat = || input("polygon_plane", format!("solidFromPolygons: polygon {index} does not span a plane"));
    // the plane's frame: along the longest arm, and square to it toward the arm farthest off it
    let longest = arms.iter().copied().fold(Vec3::new(0.0, 0.0, 0.0), |best, arm| if arm.length() > best.length() { arm } else { best });
    let e1 = longest.normalized().map_err(|_| flat())?;
    let off = |arm: Vec3| arm.sub(e1.scale(arm.dot(e1)));
    let widest = arms.iter().map(|arm| off(*arm)).fold(Vec3::new(0.0, 0.0, 0.0), |best, arm| if arm.length() > best.length() { arm } else { best });
    if widest.length() <= 1e-9 * size {
        return Err(flat());
    }
    let e2 = widest.normalized().map_err(|_| flat())?;
    let normal = e1.cross(e2);
    if arms.iter().any(|arm| arm.dot(normal).abs() > 1e-9 * size) {
        return Err(input("polygon_plane", format!("solidFromPolygons: polygon {index}'s corners do not lie in one plane")));
    }
    let reach = |axis: Vec3| arms.iter().map(|arm| arm.dot(axis)).fold((0.0f64, 0.0f64), |(low, high), at| (low.min(at), high.max(at)));
    let ((low1, high1), (low2, high2)) = (reach(e1), reach(e2));
    let corner = origin.add(e1.scale(low1 - 0.5 * size)).add(e2.scale(low2 - 0.5 * size));
    let plane = make_plane(corner, e1, e2, high1 - low1 + size, high2 - low2 + size).or_refuse(KernelStage::Collect, "solidFromPolygons.plane")?;
    let sides = (0..corners.len())
        .map(|k| make_line(corners[k], corners[(k + 1) % corners.len()]))
        .collect::<Result<Vec<_>, _>>()
        .or_refuse(KernelStage::Collect, "solidFromPolygons.side")?;
    face_from_surface_trimmed(&plane, &[sides])
}

/// The solid bounded by the plane polygons `faces`, each its corners in order
/// (either way round): every polygon a flat face, joined along their common
/// sides ([`crate::solid_from_sheets`]) and turned outward. The polygons need
/// not be convex, nor the solid; each side of each polygon must be a whole
/// side of exactly one other.
pub fn solid_from_polygons(faces: &[Vec<Vec3>]) -> Result<BrepSolid, KernelRefusal> {
    if faces.len() < 4 {
        return Err(input("polygon_count", "solidFromPolygons: a solid has at least four faces".into()));
    }
    let sheets = faces.iter().enumerate().map(|(index, corners)| polygon_face(corners, index)).collect::<Result<Vec<_>, _>>()?;
    let size = faces.iter().flatten().map(|corner| corner.sub(faces[0][0]).length()).fold(0.0, f64::max);
    solid_from_sheets(&sheets, (1e-8 * size).max(1e-7))
}

/// The pyramid on the plane polygon `base` (its corners in order, either way
/// round) with its point at `apex`: the base and one triangle on each of its
/// sides. The base need not be regular, nor the apex over its middle.
pub fn make_pyramid_on_polygon(base: &[Vec3], apex: Vec3) -> Result<BrepSolid, KernelRefusal> {
    if base.len() < 3 {
        return Err(input("pyramid_base", "makePyramidOnPolygon: the base has fewer than three corners".into()));
    }
    let mut faces = vec![base.to_vec()];
    faces.extend((0..base.len()).map(|k| vec![base[k], base[(k + 1) % base.len()], apex]));
    solid_from_polygons(&faces)
}

