//! A BALL and a CYLINDER TANGENT INSIDE IT meet in Viviani's curve, exactly:
//! `BREP_SPHERE_CYLINDER_TANGENT_SECTION` (default on; `=0` leaves the pair to
//! the marcher).
//!
//! With the ball's centre `O` and radius `R`, the cylinder's axis direction
//! `d`, its radius `r` and its axis at distance `a` from `O` with `a + r = R`
//! (the cylinder touches the ball from inside along one ruling's point), `u`
//! the unit vector from `O` towards the axis and `w = d × u`, a cylinder point
//! `O + (a + r cos θ) u + (r sin θ) w + h d` is on the ball exactly when
//! `h² = R² − a² − r² − 2ar cos θ = 4ar sin²(θ/2)`. With `θ = 2s`:
//!
//! `P(s) = O + (a + r cos 2s) u + (r sin 2s) w + (2√(ar) sin s) d`,
//!
//! one closed curve through the point of contact `O + R u` at `s = 0` and at
//! `s = π` (its node), its two lobes on either side of the plane through `O`
//! square to `d`. On each quarter `s = s₀ + 2 atan t`, `t ∈ [0, 1]`, every
//! term is a polynomial of degree four in `t` over `(1 + t²)²`, so each lobe is
//! an exact rational NURBS of degree four in two spans, read here by
//! interpolating each span's homogeneous polynomial at five stations.
//!
//! The section answers only where the ball is a whole analytic sphere, the
//! cylinder a whole analytic one whose height covers the curve, the contact is
//! internal to `tolerance`, and the two are not coaxial; anything else is left
//! to the paths it took before (`None`).
use super::*;
use crate::Vec4;

/// `BREP_SPHERE_CYLINDER_TANGENT_SECTION`: on by default; `=0` marches the pair.
fn sphere_cylinder_tangent_section_enabled() -> bool {
    crate::switch!("BREP_SPHERE_CYLINDER_TANGENT_SECTION", true)
}

/// The two lobes of Viviani's curve where `first` and `second` are a ball and
/// a cylinder tangent inside it, or `None`.
pub(super) fn intersect_sphere_cylinder_tangent(first: &NurbsSurface, second: &NurbsSurface, tolerance: f64) -> Option<Vec<NurbsCurve>> {
    if !sphere_cylinder_tangent_section_enabled() {
        return None;
    }
    let debug = std::env::var_os("BREP_DEBUG_VIVIANI").is_some();
    if debug {
        let kind = |s: &NurbsSurface| s.analytic().map(|a| format!("{a:?}").chars().take(14).collect::<String>()).unwrap_or_else(|| "none".into());
        eprintln!("viviani: pair {} x {}", kind(first), kind(second));
    }
    let (ball, wall) = match (first.analytic()?, second.analytic()?) {
        (AnalyticSurface::Sphere { .. }, AnalyticSurface::RuledRevolution { .. }) => (first, second),
        (AnalyticSurface::RuledRevolution { .. }, AnalyticSurface::Sphere { .. }) => (second, first),
        _ => return None,
    };
    let Some(AnalyticSurface::Sphere { frame: ball_frame, radius: big }) = ball.analytic() else { return None };
    let Some(AnalyticSurface::RuledRevolution { frame: wall_frame, rho0, rho1, height }) = wall.analytic() else { return None };
    let (big, r, height) = (*big, *rho0, *height);
    if !(big > 0.0 && r > 0.0 && height > 0.0) || (rho1 - rho0).abs() > 1e-12 * r.max(1.0) {
        return None;
    }
    let centre = ball_frame.origin;
    let d = wall_frame.axis;
    // The axis line's foot from the centre, and the distance to it.
    let offset = wall_frame.origin.sub(centre);
    let foot = centre.add(offset.sub(d.scale(offset.dot(d))));
    let towards = foot.sub(centre);
    let a = towards.length();
    let scale = big.max(1.0);
    let band = tolerance.max(1e-9 * scale);
    // Internal tangency, and not coaxial (a coaxial cylinder meets the ball in
    // circles, another lane's).
    if !(a > band) || (a + r - big).abs() > band || r >= big {
        if debug {
            eprintln!("viviani: not tangent inside: a {a} r {r} R {big}");
        }
        return None;
    }
    let u = towards.scale(1.0 / a);
    let w = d.cross(u);
    let reach = 2.0 * (a * r).sqrt();
    // The cylinder's height covers the curve: the foot's height on its axis,
    // ± the curve's reach, inside [0, height].
    let at = foot.sub(wall_frame.origin).dot(d);
    if at - reach < band || at + reach > height - band {
        if debug {
            eprintln!("viviani: the cylinder does not cover the curve: at {at} reach {reach} height {height}");
        }
        return None;
    }
    let point = |s: f64| -> Vec3 {
        centre.add(u.scale(a + r * (2.0 * s).cos())).add(w.scale(r * (2.0 * s).sin())).add(d.scale(reach * s.sin()))
    };
    let lobe = |from: f64| -> Option<NurbsCurve> {
        let mut controls: Vec<Vec4> = Vec::new();
        for quarter in 0..2 {
            let start = from + quarter as f64 * std::f64::consts::FRAC_PI_2;
            // The span's homogeneous polynomial at t = k/4, k = 0..4, and its
            // Bernstein coefficients by the exact 5 x 5 solve.
            let stations: Vec<[f64; 4]> = (0..=4)
                .map(|k| {
                    let t = k as f64 / 4.0;
                    let weight = (1.0 + t * t) * (1.0 + t * t);
                    let p = point(start + 2.0 * t.atan());
                    [p.x * weight, p.y * weight, p.z * weight, weight]
                })
                .collect();
            let bernstein = |i: usize, t: f64| -> f64 {
                let binomial = [1.0, 4.0, 6.0, 4.0, 1.0][i];
                binomial * t.powi(i as i32) * (1.0 - t).powi(4 - i as i32)
            };
            let mut matrix = [[0.0f64; 5]; 5];
            for (k, row) in matrix.iter_mut().enumerate() {
                for (i, value) in row.iter_mut().enumerate() {
                    *value = bernstein(i, k as f64 / 4.0);
                }
            }
            let mut coefficients = solve5(matrix, &stations)?;
            // The two spans share their joint's control point: the second
            // span's homogeneous net is scaled so its first weight is the
            // first span's last (a span's net scaled is the same curve).
            if let Some(joint) = controls.last() {
                let factor = joint.w / coefficients[0][3];
                for c in &mut coefficients {
                    for value in c.iter_mut() {
                        *value *= factor;
                    }
                }
            }
            let skip = if quarter == 0 { 0 } else { 1 };
            for c in coefficients.iter().skip(skip) {
                if !(c[3] > 0.0) {
                    return None;
                }
                controls.push(Vec4 { x: c[0], y: c[1], z: c[2], w: c[3] });
            }
        }
        let knots = vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0];
        NurbsCurve::new(4, knots, controls).ok()
    };
    let curves = vec![lobe(0.0)?, lobe(std::f64::consts::PI)?];
    if debug {
        eprintln!("viviani: built two lobes");
    }
    // Every lobe lies on both carriers: a reading at its span middles.
    for curve in &curves {
        for t in [0.25, 0.5, 0.75, 1.25, 1.5, 1.75] {
            let p = curve.evaluate(t).ok()?;
            let q = p.sub(wall_frame.origin);
            let (on_ball, on_wall) = ((p.sub(centre).length() - big).abs(), (q.sub(d.scale(q.dot(d))).length() - r).abs());
            if on_ball > band || on_wall > band {
                if debug {
                    eprintln!("viviani: lobe off its carriers at t {t}: ball {on_ball:e} wall {on_wall:e} band {band:e}");
                }
                return None;
            }
        }
    }
    Some(curves)
}

/// The 5 x 5 system `matrix · x = rhs` for four right-hand sides, by Gaussian
/// elimination with partial pivoting.
fn solve5(mut matrix: [[f64; 5]; 5], rhs: &[[f64; 4]]) -> Option<Vec<[f64; 4]>> {
    let mut b: Vec<[f64; 4]> = rhs.to_vec();
    for column in 0..5 {
        let pivot = (column..5).max_by(|&i, &j| matrix[i][column].abs().total_cmp(&matrix[j][column].abs()))?;
        if matrix[pivot][column].abs() < 1e-14 {
            return None;
        }
        matrix.swap(column, pivot);
        b.swap(column, pivot);
        for row in column + 1..5 {
            let factor = matrix[row][column] / matrix[column][column];
            for k in column..5 {
                matrix[row][k] -= factor * matrix[column][k];
            }
            for k in 0..4 {
                b[row][k] -= factor * b[column][k];
            }
        }
    }
    let mut x = vec![[0.0f64; 4]; 5];
    for row in (0..5).rev() {
        for k in 0..4 {
            let mut value = b[row][k];
            for column in row + 1..5 {
                value -= matrix[row][column] * x[column][k];
            }
            x[row][k] = value / matrix[row][row];
        }
    }
    Some(x)
}

