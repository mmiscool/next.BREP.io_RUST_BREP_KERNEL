use crate::Vec3;

/// Newell's unnormalized normal, including the closing edge from last to first.
/// For planar polygons its magnitude is twice the area and its sign follows winding.
pub(crate) fn newell_normal(points: &[Vec3]) -> Vec3 {
    let mut normal = Vec3::default();
    for index in 0..points.len() {
        let point = points[index];
        let next = points[(index + 1) % points.len()];
        normal.x += (point.y - next.y) * (point.z + next.z);
        normal.y += (point.z - next.z) * (point.x + next.x);
        normal.z += (point.x - next.x) * (point.y + next.y);
    }
    normal
}

