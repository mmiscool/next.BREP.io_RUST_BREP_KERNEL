use crate::Vec3;

/// Ericson Voronoi-region closest point, including degenerate triangle edges.
pub fn closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b.sub(a);
    let ac = c.sub(a);
    if ab.cross(ac).length_squared() == 0.0 {
        let mut best = a;
        let mut best_distance = f64::INFINITY;
        for (start, end) in [(a, b), (b, c), (c, a)] {
            let edge = end.sub(start);
            let length = edge.length_squared();
            let t = if length > 0.0 { (p.sub(start).dot(edge) / length).clamp(0.0, 1.0) } else { 0.0 };
            let point = start.add(edge.scale(t));
            let distance = p.sub(point).length_squared();
            if distance < best_distance {
                best = point;
                best_distance = distance;
            }
        }
        return best;
    }
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

