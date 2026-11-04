use crate::{Vec2, Vec3};

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


/// Form-K horizontal-ray crossing; operand order is shared with the trim indexes.
#[inline]
pub fn crosses_ray(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> bool {
    if (a[1] > p[1]) != (b[1] > p[1]) {
        let t = (p[1] - a[1]) / (b[1] - a[1]);
        let x = a[0] + t * (b[0] - a[0]);
        x > p[0]
    } else {
        false
    }
}

/// Even-odd parity over a closed polygon, including its closing edge.
#[inline]
pub fn parity_inside(p: [f64; 2], count: usize, uv: impl Fn(usize) -> [f64; 2]) -> bool {
    let mut inside = false;
    for offset in 0..count {
        if crosses_ray(p, uv(offset), uv((offset + 1) % count)) {
            inside = !inside;
        }
    }
    inside
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PolygonClass {
    Inside,
    Outside,
    Boundary,
}


/// Boundary band followed by form-K parity. Keep the caller's segment guard.
#[inline]
pub fn classify(point: Vec2, polygon: &[Vec2], tolerance: f64, segment_guard: f64) -> PolygonClass {
    for index in 0..polygon.len() {
        if point.distance_to_segment(polygon[index], polygon[(index + 1) % polygon.len()], segment_guard) <= tolerance {
            return PolygonClass::Boundary;
        }
    }
    if parity_inside([point.x, point.y], polygon.len(), |i| [polygon[i].x, polygon[i].y]) {
        PolygonClass::Inside
    } else {
        PolygonClass::Outside
    }
}

/// One oriented shoelace term (twice the edge's area contribution).
#[inline]
pub fn area_term(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - b[0] * a[1]
}

#[inline]
pub fn twice_signed_area_2d(count: usize, uv: impl Fn(usize) -> [f64; 2]) -> f64 {
    let mut area = 0.0;
    for index in 0..count {
        area += area_term(uv(index), uv((index + 1) % count));
    }
    area
}

#[inline]
pub fn signed_area_2d(count: usize, uv: impl Fn(usize) -> [f64; 2]) -> f64 {
    twice_signed_area_2d(count, uv) * 0.5
}

