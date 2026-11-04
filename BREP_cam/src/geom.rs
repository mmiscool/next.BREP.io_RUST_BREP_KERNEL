//! Geometry shared by CAM algorithms, independent of the optional kernel adapter.
use crate::model::Point3;

/// Unit perpendicular using the machine/tool frame's stable Z-then-X seed.
/// Callers supply a unit axis; zero input falls back to X.
pub fn perpendicular(axis: Point3) -> Point3 {
    axis.cross(if axis.z.abs() < 0.9 { Point3::Z } else { Point3::X }).normalized().unwrap_or(Point3::X)
}

/// Distance to a closed segment, including a zero-length segment.
pub fn point_segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let d = b.sub(a);
    let q = d.length_squared();
    if q == 0.0 { return p.distance(a); }
    p.distance(a.lerp(b, (p.sub(a).dot(d) / q).clamp(0.0, 1.0)))
}

/// The closest point on triangle `abc` (Ericson Voronoi regions).
pub fn closest_point_on_triangle(p: Point3, a: Point3, b: Point3, c: Point3) -> Point3 {
    let ab = b.sub(a);
    let ac = c.sub(a);
    let ap = p.sub(a);
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p.sub(b);
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = if (d1 - d3).abs() > 0.0 { d1 / (d1 - d3) } else { 0.0 };
        return a.add(ab.scale(v));
    }
    let cp = p.sub(c);
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = if (d2 - d6).abs() > 0.0 { d2 / (d2 - d6) } else { 0.0 };
        return a.add(ac.scale(w));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        let w = if denom > 0.0 { (d4 - d3) / denom } else { 0.0 };
        return b.add(c.sub(b).scale(w));
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a.add(ab.scale(v)).add(ac.scale(w))
}

/// Squared distance to triangle `abc`.
pub fn point_triangle_distance_squared(p: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    p.sub(closest_point_on_triangle(p, a, b, c)).length_squared()
}

/// Distance to a segment in XY.
pub fn point_segment_distance_2d(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    ((p.0 - a.0 - t * dx).powi(2) + (p.1 - a.1 - t * dy).powi(2)).sqrt()
}

/// Exact coordinate key: IEEE signed zeros denote the same geometric point.
pub(crate) fn canonical_bits(value: f64) -> u64 {
    if value == 0.0 { 0 } else { value.to_bits() }
}
