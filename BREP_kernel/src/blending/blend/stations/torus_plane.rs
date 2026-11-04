//! A scalar proposal for a ring-torus/affine-plane station. The original
//! stored-carrier tangency residual certifies every returned proposal.
use super::*;

pub(super) struct Solver {
    torus: usize,
    frame: crate::analytic_surface::RevolutionFrame,
    major: f64,
    minor: f64,
    sense: f64,
    origin: Vec3,
    u: Vec3,
    v: Vec3,
    low: [f64; 2],
    normal: Vec3,
    gram: [f64; 3],
}

impl Solver {
    pub(super) fn new(surfaces: [&NurbsSurface; 2]) -> Option<Self> {
        if !switch!("BREP_TORUS_PLANE_STATION", true) { return None; }
        for torus in 0..2 {
            let crate::AnalyticSurface::Torus { frame, major_radius, minor_radius } = surfaces[torus].analytic()? else { continue; };
            let crate::AnalyticSurface::Plane { origin, u_dir, v_dir, u_domain, v_domain } = surfaces[1-torus].analytic()? else { continue; };
            if !(major_radius > minor_radius && *minor_radius > 0.0)
                || surfaces[torus].closed_directions().ok()? != (true, true) { continue; }
            let (point, su, sv) = surfaces[torus].deriv1_extended(0.123, 0.234).ok()?;
            let raw = su.cross(sv).normalized().ok()?;
            let delta = point.sub(frame.origin);
            let radial = delta.sub(frame.axis.scale(delta.dot(frame.axis))).normalized().ok()?;
            let ideal = delta.sub(radial.scale(*major_radius)).normalized().ok()?;
            let dot = raw.dot(ideal);
            if !dot.is_finite() || dot.abs() < 0.999999 { continue; }
            let normal = u_dir.cross(*v_dir).normalized().ok()?;
            // This lane's covered geometry is an axis-parallel plane cut.
            if normal.dot(frame.axis).abs()>1e-10 { continue; }
            return Some(Self { torus, frame: frame.clone(), major: *major_radius, minor: *minor_radius,
                sense: dot.signum(), origin: *origin, u: *u_dir, v: *v_dir,
                low: [u_domain[0],v_domain[0]], normal,
                gram: [u_dir.dot(*u_dir), u_dir.dot(*v_dir), v_dir.dot(*v_dir)] });
        }
        None
    }

    pub(super) fn solve(&self, surfaces: [&NurbsSurface; 2], rho: [f64; 2], seed: [f64; 4], section: Vec3, tangent: Vec3, scale: f64) -> Option<([f64; 4], TangencyState)> {
        let minor = self.minor + self.sense * rho[self.torus];
        if !(minor > 0.0 && self.major > minor) { return None; }
        let i = 2*self.torus;
        let source = crate::blend::track_fit::carrier_point(surfaces[self.torus], seed[i], seed[i+1]).ok()?;
        let delta = source.sub(self.frame.origin);
        let radial = delta.sub(self.frame.axis.scale(delta.dot(self.frame.axis))).normalized().ok()?;
        let normal = delta.sub(radial.scale(self.major)).normalized().ok()?;
        let guessed = source.add(normal.scale(self.sense*rho[self.torus]));
        // Intersect the two planes, with the nearest point to the seed as
        // the scalar origin. No axis-specific placement is assumed.
        let (a,b,c) = (self.normal.dot(self.normal), self.normal.dot(tangent), tangent.dot(tangent));
        let det = a*c-b*b;
        if !det.is_finite() || det <= 1e-12*a*c { return None; }
        let plane_origin = self.origin.add(self.normal.scale(rho[1-self.torus]));
        let (f,g) = (guessed.sub(plane_origin).dot(self.normal), guessed.sub(section).dot(tangent));
        let base = guessed.sub(self.normal.scale((c*f-b*g)/det)).sub(tangent.scale((a*g-b*f)/det));
        let direction = self.normal.cross(tangent).normalized().ok()?;
        let mut along = 0.0;
        let mut centre = base;
        let bar = 2.0*minor*1e-13*(1.0+scale);
        let mut converged = false;
        for _ in 0..16 {
            centre = base.add(direction.scale(along));
            let d = centre.sub(self.frame.origin);
            let z = d.dot(self.frame.axis);
            let radial = d.sub(self.frame.axis.scale(z));
            let radius = radial.length();
            if !(radius > 0.0) { return None; }
            let residual = (radius-self.major)*(radius-self.major)+z*z-minor*minor;
            if !residual.is_finite() { return None; }
            if residual.abs() <= bar { converged = true; break; }
            let derivative = 2.0*(radius-self.major)*radial.dot(direction)/radius+2.0*z*direction.dot(self.frame.axis);
            if !derivative.is_finite() || derivative.abs() < 1e-14 { return None; }
            along -= residual/derivative;
            if !along.is_finite() { return None; }
        }
        if !converged { return None; }
        let d = centre.sub(self.frame.origin);
        let z = d.dot(self.frame.axis);
        let radial = d.sub(self.frame.axis.scale(z));
        let theta = radial.dot(self.frame.y_axis).atan2(radial.dot(self.frame.x_axis)).rem_euclid(std::f64::consts::TAU);
        let phi = z.atan2(radial.length()-self.major).rem_euclid(std::f64::consts::TAU);
        let circle = |angle| crate::analytic_surface::circle_angle_to_parameter(4, std::f64::consts::TAU, angle);
        let mut uv = seed;
        uv[i] = circle(theta); uv[i+1] = circle(phi);
        uv[i] += (seed[i]-uv[i]).round(); uv[i+1] += (seed[i+1]-uv[i+1]).round();
        // A scalar proposal only accelerates a local continuation. Distant
        // roots go back to the original solver from its untouched seed.
        if (uv[i]-seed[i]).abs()>0.125 || (uv[i+1]-seed[i+1]).abs()>0.125 { return None; }
        let contact = centre.sub(self.normal.scale(rho[1-self.torus]));
        let d = contact.sub(self.origin);
        let (fu,fv) = (d.dot(self.u),d.dot(self.v));
        let [a,b,c] = self.gram;
        let determinant = a*c-b*b;
        if !determinant.is_finite() || determinant.abs() <= 1e-16*(a*c).max(1.0) { return None; }
        let p = 2*(1-self.torus);
        uv[p] = (fu*c-fv*b)/determinant+self.low[0];
        uv[p+1] = (fv*a-fu*b)/determinant+self.low[1];
        if !uv.iter().all(|v| v.is_finite()) { return None; }
        // The model is only a proposal: use the original stored surface
        // points, Raw normals and original convergence bar for acceptance.
        let state = tangency_state(surfaces[0],surfaces[1],rho,uv,section,tangent).ok()?;
        if !state.residual.iter().all(|v| v.is_finite() && v.abs() <= 1e-11*(1.0+scale)) { return None; }
        crate::blend::record_count("count.closed.torus_plane_station", 1);
        Some((uv,state))
    }
}

