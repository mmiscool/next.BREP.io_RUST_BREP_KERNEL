use super::*;

/// An extrusion's profile reduced into the plane perpendicular to its direction, held on its
/// surface (`NurbsSurface::held_extrusion_profile`) so the closed form does not rebuild the reduced
/// curve, and the projector does not re-hash it, on every projection (`BREP_EXTRUSION_HELD_PROFILE`).
#[derive(Clone, Debug)]
pub struct ReducedProfile {
    axis: Vec3,
    origin: Vec3,
    held: crate::projection::HeldCurve,
}

/// `BREP_EXTRUSION_HELD_PROFILE` (default on since its promotion; `=0` rebuilds it every call): the extrusion closed form projects onto a reduced
/// profile held on the surface, for the same answer as the one it rebuilds every call.
fn held_profile_enabled() -> bool {
    switch!("BREP_EXTRUSION_HELD_PROFILE", true)
}

/// `BREP_EXTRUSION_HELD_PROFILE_CHECK=1`: project both ways and stop on any difference.
fn held_profile_check() -> bool {
    switch!("BREP_EXTRUSION_HELD_PROFILE_CHECK", false)
}

impl ExtrusionStructure {
    /// The profile reduced into the plane perpendicular to the sweep direction, through the
    /// profile's first control point; with `hold`, with its seed samples for repeated projection.
    pub(crate) fn reduce(&self, hold: bool) -> Option<ReducedProfile> {
        let axis = self.direction.normalized().ok()?;
        let origin = self.profile.control_points.first()?.point().ok()?;
        let perpendicular = |p: Vec3| {
            let relative = p.sub(origin);
            relative.sub(axis.scale(relative.dot(axis)))
        };
        let controls = self.profile.control_points.iter().map(|p| {
            let q = perpendicular(p.point().ok()?);
            Some(crate::Vec4 { x:q.x*p.w, y:q.y*p.w, z:q.z*p.w, w:p.w })
        }).collect::<Option<Vec<_>>>()?;
        let reduced = NurbsCurve::new(self.profile.degree, self.profile.knots.clone(), controls).ok()?;
        let held = if hold { crate::projection::HeldCurve::new(reduced) } else { crate::projection::HeldCurve::bare(reduced) };
        Some(ReducedProfile { axis, origin, held })
    }

    /// Eliminate the sweep coordinate in the metric perpendicular to its
    /// direction. An interior minimizer of that curve problem is also a
    /// minimizer on the finite patch. When the free foot is outside the
    /// sweep band, decline: clamping it could miss another profile branch
    /// whose free foot is inside. The general surface solver handles that
    /// constrained problem, including its boundary curves.
    pub fn project(&self, surface: &NurbsSurface, point: Vec3) -> Option<SurfaceProjection> {
        let (on, check) = (held_profile_enabled(), held_profile_check());
        if on || check {
            if let Some(held) = surface.held_extrusion_profile(self) {
                let answer = held.and_then(|reduced| self.project_reduced(surface, point, reduced));
                if check {
                    let fresh = self.reduce(false).and_then(|reduced| self.project_reduced(surface, point, &reduced));
                    let bits = |a: &Option<SurfaceProjection>| a.as_ref().map(|p| [p.u, p.v, p.point.x, p.point.y, p.point.z, p.distance].map(f64::to_bits));
                    if bits(&answer) != bits(&fresh) {
                        panic!("extrusion held profile check: held {answer:?} differs from rebuilt {fresh:?} at {point:?}");
                    }
                }
                return answer;
            }
        }
        let reduced = self.reduce(false)?;
        self.project_reduced(surface, point, &reduced)
    }

    fn project_reduced(
        &self,
        surface: &NurbsSurface,
        point: Vec3,
        reduced: &ReducedProfile,
    ) -> Option<SurfaceProjection> {
        let relative = point.sub(reduced.origin);
        let foot = reduced.held.project(relative.sub(reduced.axis.scale(relative.dot(reduced.axis)))).ok()?;
        let profile_point = self.profile.evaluate(foot.u).ok()?;
        let sweep = self.sweep_domain[0] + point.sub(profile_point).dot(reduced.axis) / self.direction.length();
        if !sweep.is_finite() || sweep < self.sweep_domain[0] || sweep > self.sweep_domain[1] { return None; }
        let (u,v) = if self.profile_is_u { (foot.u,sweep) } else { (sweep,foot.u) };
        let projected = surface.evaluate(u,v).ok()?;
        Some(SurfaceProjection { u,v,point:projected,distance:projected.sub(point).length() })
    }
}
