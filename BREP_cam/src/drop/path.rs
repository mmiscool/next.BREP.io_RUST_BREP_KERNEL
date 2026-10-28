//! Line/arc sampling and bounded adaptive projection. Adaptive output carries
//! conservative whole-segment clearance, not just endpoint collision checks.
use super::*;
use crate::cutter::{CutterKind, CutterError, ProfileSegment};
use crate::model::{Diagnostic, ApproximationLimit};
use serde::{Serialize, Deserialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum Span {
    Line { start: Point3, end: Point3 },
    Arc { start: Point3, end: Point3, center: Point3, clockwise: bool },
}
impl Span {
    pub fn validate(&self, tol: &Tolerance) -> Result<(), CamError> {
        let (a,b) = self.ends();
        if !a.is_finite() || !b.is_finite() { return Err(CamError::InvalidInput("path endpoints must be finite".into())); }
        if let Self::Arc { center, .. } = self {
            if !center.is_finite() || a.distance_xy(*center) <= tol.sample_eps
                || (a.distance_xy(*center)-b.distance_xy(*center)).abs() > tol.distance_eps
                || a.distance_xy(b) <= tol.sample_eps {
                return Err(CamError::InvalidInput("arc needs finite center, equal nonzero radii and distinct endpoints; split full circles".into()));
            }
        }
        if !self.length2d().is_finite() { return Err(CamError::InvalidInput("path length is not finite".into())); }
        Ok(())
    }
    pub fn ends(&self) -> (Point3,Point3) {
        match self { Self::Line {start,end} | Self::Arc {start,end,..} => (*start,*end) }
    }
    fn sweep(&self) -> f64 {
        if let Self::Arc {start,end,center,clockwise} = self {
            let a = (start.y-center.y).atan2(start.x-center.x);
            let b = (end.y-center.y).atan2(end.x-center.x);
            if *clockwise { -((a-b).rem_euclid(std::f64::consts::TAU)) } else { (b-a).rem_euclid(std::f64::consts::TAU) }
        } else { 0. }
    }
    pub fn length2d(&self) -> f64 {
        let (a,b) = self.ends();
        match self { Self::Line {..} => a.distance_xy(b), Self::Arc {center,..} => a.distance_xy(*center)*self.sweep().abs() }
    }
    pub fn point_at(&self, t: f64) -> Point3 {
        let t = t.clamp(0.,1.);
        let (a,b) = self.ends();
        if t == 0. { return a; } if t == 1. { return b; }
        match self {
            Self::Line {..} => a.lerp(b,t),
            Self::Arc {center,..} => {
                let angle = (a.y-center.y).atan2(a.x-center.x) + self.sweep()*t;
                let r = a.distance_xy(*center);
                Point3::new(center.x+r*angle.cos(),center.y+r*angle.sin(),a.z+(b.z-a.z)*t)
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct Sampling {
    pub sample_spacing: f64,
    pub min_sample_spacing: f64,
    pub chord_error: f64,
    pub flatness_cos_limit: f64,
    pub max_depth: u32,
    pub max_samples: u64,
    pub floor_z: f64,
}
impl Default for Sampling {
    fn default() -> Self {
        Self { sample_spacing: 0.5, min_sample_spacing: 1e-6, chord_error: 0.01, flatness_cos_limit: 0.999, max_depth: 32, max_samples: 1_000_000, floor_z: 0. }
    }
}
impl Sampling {
    pub fn validate(&self) -> Result<(), CamError> {
        if ![self.sample_spacing,self.min_sample_spacing,self.chord_error,self.floor_z,self.flatness_cos_limit].iter().all(|v|v.is_finite())
            || self.sample_spacing <= 0. || self.min_sample_spacing <= 0. || self.min_sample_spacing > self.sample_spacing
            || self.chord_error <= 0. || !(-1.0..=1.0).contains(&self.flatness_cos_limit) || self.max_depth > 60 || self.max_samples < 2 {
            return Err(CamError::InvalidInput("invalid path sampling controls".into()));
        }
        Ok(())
    }
}

/// A radial dilation of the bottom graph: h(max(0,r-pad)). Its vertical
/// clearance at a segment midpoint bounds every tool position in that segment.
pub(crate) struct Dilated<'a> { pub(crate) base: &'a dyn CutterProfile, pub(crate) pad: f64 }
impl CutterProfile for Dilated<'_> {
    fn kind(&self) -> CutterKind { self.base.kind() }
    fn radius(&self) -> f64 { self.base.radius()+self.pad }
    fn cutting_length(&self) -> f64 { self.base.cutting_length() }
    fn shaft_length(&self) -> f64 { self.base.shaft_length() }
    fn shaft_radius(&self) -> f64 { self.base.shaft_radius()+self.pad }
    fn profile_height(&self) -> f64 { self.base.profile_height() }
    fn height_at_radius(&self,r:f64) -> Option<f64> { if r > self.radius() || r < 0. { None } else { self.base.height_at_radius((r-self.pad).clamp(0.,self.base.radius())) } }
    fn radius_at_height(&self,h:f64) -> Option<f64> { self.base.radius_at_height(h).map(|r|r+self.pad) }
    fn validate(&self) -> Result<(),CutterError> { self.base.validate() }
    fn segments(&self) -> Vec<ProfileSegment> {
        let mut s = self.base.segments();
        for v in &mut s {
            v.radius_range[0] += self.pad; v.radius_range[1] += self.pad;
            if matches!(v.kind,SegmentKind::Sphere | SegmentKind::Torus | SegmentKind::SphereUpper) { v.ring_radius += self.pad; }
        }
        s.push(ProfileSegment { kind: SegmentKind::Flat, radius_range: [0.,self.pad], height_range: [0.,0.], profile_radius: 0., ring_radius: 0., half_angle_rad: 0., z_offset: 0. });
        s
    }
}

impl DropCutter<'_> {
    fn counted(&self, p: Point3, settings: &Sampling, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<ClPoint,CamError> {
        if report.counts.samples >= settings.max_samples { return Err(CamError::InvalidInput("drop path sample work budget exhausted".into())); }
        let r = self.point(p.x,p.y,settings.floor_z,runner)?;
        report.counts.samples += 1; report.counts.candidates += r.candidate_count; report.counts.contacts += r.contact_count;
        Ok(r.point)
    }

    /// Whether the profile's height grows convexly with the radius (ball,
    /// flat, bull, V, tapered ball): then a triangle's contact height minus a
    /// straight segment's height is concave along the segment and
    /// [`Self::segment_excess`] is a certificate. Checked on 65 radii.
    fn profile_is_convex(&self) -> bool {
        let r = self.cutter.radius();
        let h: Vec<Option<f64>> = (0..=64).map(|k| self.cutter.height_at_radius(r * k as f64 / 64.0)).collect();
        if h.iter().any(|v| v.map_or(true, |v| !v.is_finite())) { return false; }
        let h: Vec<f64> = h.into_iter().map(|v| v.unwrap()).collect();
        h.windows(3).all(|w| w[0] - 2.0 * w[1] + w[2] >= -1e-12 * (1.0 + r)) && h.windows(2).all(|w| w[1] >= w[0] - 1e-12 * (1.0 + r))
    }

    /// An upper bound of how far the mesh rises above the cutter moving in a
    /// straight line from `a` to `b` (tip heights interpolated linearly):
    /// the maximum over triangles and over the segment of the triangle's
    /// contact height minus the segment's height. Non-positive means the
    /// segment is clear; lifting both ends by a positive value clears it.
    /// Against one triangle that difference is concave along the segment for
    /// a convex profile (the contact is the maximum over the triangle of
    /// its height minus the profile at a distance that is convex in the
    /// segment parameter), so a few samples and the chord extensions of a
    /// concave function bound it from above (`concave_max_bound`). Triangles
    /// whose top cannot rise above the best found are skipped. Returns as
    /// soon as the bound exceeds `stop` (the caller refines anyway).
    pub(crate) fn segment_excess(&self, a: Point3, b: Point3, stop: f64, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<f64, CamError> {
        let excess = self.flute_excess(a, b, stop, report, runner)?;
        // A wider shank: its excess on the segment raised by the cutting
        // length (a flat profile, convex), the larger of the two bounds.
        match self.shank() {
            Some((shank, lift)) if excess <= stop => {
                let up = Point3::new(0.0, 0.0, lift);
                Ok(excess.max(shank.flute_excess(a.add(up), b.add(up), stop, report, runner)?))
            }
            _ => Ok(excess),
        }
    }

    fn flute_excess(&self, a: Point3, b: Point3, stop: f64, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<f64, CamError> {
        let radius = self.cutter.radius();
        let eps = self.tolerance.distance_eps;
        let reach = radius + eps;
        let lo = Point3::new(a.x.min(b.x) - reach, a.y.min(b.y) - reach, f64::NEG_INFINITY);
        let hi = Point3::new(a.x.max(b.x) + reach, a.y.max(b.y) + reach, f64::INFINITY);
        let mut ids = Vec::new();
        match self.index {
            Some(index) => index.query_aabb(&crate::model::Bounds3::new(lo, hi), crate::ProjectionMode::Xy, &mut ids),
            None => ids.extend(0..self.mesh.triangle_count() as u32),
        }
        let z = |t: f64| a.z + (b.z - a.z) * t;
        // Each candidate: the parameter range over which its box is within
        // reach of the moving axis, and an upper key (its top minus the
        // segment's lowest height over that range).
        let mut keyed: Vec<(f64, f64, f64, u32)> = Vec::with_capacity(ids.len());
        for id in ids {
            runner.work(1, "segment-candidates", keyed.len() as u64, self.mesh.triangle_count() as u64)?;
            let tri = self.mesh.triangle(id);
            let Some((t0, t1)) = clip_param(a, b, &tri.bounds, reach) else { continue };
            // The key bounds every contact on the triangle from above: its top,
            // minus the profile at the closest the axis comes to its box (the
            // profile never decreases outward), minus the segment's lowest
            // height over the range.
            let (pa, pb) = (a.lerp(b, t0), a.lerp(b, t1));
            let corners = [(tri.bounds.min.x, tri.bounds.min.y), (tri.bounds.max.x, tri.bounds.min.y), (tri.bounds.max.x, tri.bounds.max.y), (tri.bounds.min.x, tri.bounds.max.y)];
            let dmin = segment_polygon_distance(pa, pb, &corners).0.min(radius);
            let h = self.cutter.height_at_radius(dmin).unwrap_or(0.0);
            let key = tri.max_z() - h - z(t0).min(z(t1));
            keyed.push((key, t0, t1, id));
        }
        keyed.sort_by(|x, y| y.0.total_cmp(&x.0).then(x.3.cmp(&y.3)));
        let segments = self.cutter.segments();
        let mut excess = f64::NEG_INFINITY;
        for (key, t0, t1, id) in keyed {
            if key <= excess.max(0.0) {
                // Every remaining triangle's key is at most this one's: the
                // result stays an upper bound of the whole excess.
                excess = excess.max(key);
                break;
            }
            let tri = self.mesh.triangle(id);
            report.counts.candidates += 1;
            // The XY distance from the moving axis to the triangle is convex
            // in the parameter (zero where the path crosses it); its least
            // value decides whether the triangle is in reach at all, and its
            // place is a sample inside the contact range.
            let dist = |t: f64| { let p = a.lerp(b, t); xy_distance_to_triangle(tri, p.x, p.y) };
            let (dmin_tri, u_star) = segment_polygon_distance(a.lerp(b, t0), a.lerp(b, t1), &[(tri.a.x, tri.a.y), (tri.b.x, tri.b.y), (tri.c.x, tri.c.y)]);
            if dmin_tri > reach {
                continue;
            }
            let t_star = t0 + (t1 - t0) * u_star;
            let mut evals = 0u64;
            let mut bracketed = false;
            let mut f = |t: f64| -> Result<Option<f64>, CamError> {
                evals += 1;
                let p = a.lerp(b, t);
                Ok(self.triangle_contact(tri, p.x, p.y, &segments)?.map(|(c, edge)| { bracketed |= edge; c - z(t) }))
            };
            let target = excess.max(0.0);
            // Resolution: a twentieth of the decision height, so the bound
            // can tell a clear segment from one to refine; finite even when
            // the caller wants the value itself (an unbounded `stop`).
            let resolution = if stop.is_finite() { 0.05 * stop.max(eps) } else { eps };
            // A gap's key: the profile is non-decreasing, the contact radius
            // is at least the XY distance to the triangle (convex in t, least
            // at `t_star`), and the segment is lowest at a gap end.
            let top = tri.max_z();
            let cutter = self.cutter;
            let gap_cap = |ta: f64, tb: f64| -> f64 {
                let dmin = if ta <= t_star && t_star <= tb { dist(t_star) } else { dist(ta).min(dist(tb)) };
                let h = cutter.height_at_radius(dmin.min(radius)).unwrap_or(0.0);
                top - h - z(ta).min(z(tb))
            };
            let bound = concave_max_bound(&mut f, &gap_cap, t0, t1, t_star, target, resolution, 96)?;
            report.counts.contacts += evals;
            // Vertex and facet contacts are exact; an edge contact is the
            // converged maximiser of a concave function, exact to rounding: a
            // rounding margin covers it when one entered the samples.
            let margin = if bracketed { 1e-10 * (1.0 + radius) } else { 0.0 };
            let bound = match bound {
                Some(v) if v.is_finite() => v + margin,
                // A contact range too narrow to bound from samples (a graze
                // at the reach): the triangle's top over the range.
                _ => key,
            };
            if bound > excess {
                excess = bound;
            }
            if excess > stop {
                return Ok(excess);
            }
        }
        Ok(excess)
    }

    /// Conservative constant-height clearance for the full straight segment.
    pub fn segment_bound(&self, a: Point3, b: Point3, settings: &Sampling, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<f64,CamError> {
        let dilated = Dilated { base: self.cutter, pad: a.distance_xy(b)*0.5 };
        let wide = DropCutter::from_profile(self.mesh,&dilated,self.index,self.tolerance)?;
        Ok(wide.counted(a.lerp(b,0.5),settings,report,runner)?.z)
    }

    pub fn uniform(&self, spans: &[Span], settings: &Sampling, runner: &mut Runner<'_>) -> Result<(Vec<Vec<ClPoint>>,OperationReport),CamError> {
        settings.validate()?;
        let mut report = OperationReport::default(); report.counts.triangles = self.mesh.triangle_count() as u64;
        let mut paths = Vec::new();
        runner.stage("uniform-path-sample","Sample source spans",0,spans.len() as u64)?;
        for span in spans {
            span.validate(&self.tolerance)?;
            let length = span.length2d();
            if length <= self.tolerance.sample_eps && spans.len() > 1 {
                report.warn(Diagnostic::new("degenerate-span","Skipped a degenerate source span")); continue;
            }
            let steps = (length/settings.sample_spacing).ceil().max(1.);
            if !steps.is_finite() || steps > settings.max_samples as f64 { return Err(CamError::InvalidInput("uniform span exceeds sample work budget".into())); }
            let mut points = Vec::new();
            for i in 0..=steps as u64 { points.push(self.counted(span.point_at(i as f64/steps),settings,&mut report,runner)?); }
            paths.push(points);
        }
        report.finish();
        runner.stage("uniform-path-complete","Spans projected",paths.len() as u64,spans.len() as u64)?;
        Ok((paths,report))
    }

    /// Independent spans remain independent paths. Each accepted straight output
    /// segment has both endpoints lifted to at least its swept clearance bound,
    /// by at most chord_error. Unresolved intervals are errors, never usable paths.
    pub fn adaptive(&self, spans: &[Span], settings: &Sampling, runner: &mut Runner<'_>) -> Result<(Vec<Vec<ClPoint>>,OperationReport),CamError> {
        settings.validate()?;
        let mut report = OperationReport::default(); report.counts.triangles = self.mesh.triangle_count() as u64;
        let mut paths = Vec::new();
        runner.stage("adaptive-path-start","Project and certify spans",0,spans.len() as u64)?;
        #[cfg(not(feature = "parallel"))]
        for span in spans {
            if let Some(output) = self.adaptive_span(span, spans.len(), settings, &mut report, runner)? {
                paths.push(output);
            }
        }
        // Spans are independent: solved in parallel, each with its own report,
        // then merged in span order exactly as the serial loop accumulates
        // (counts summed, diagnostics and approximations appended); the sample
        // budget is checked on the running total, as the serial loop does.
        #[cfg(feature = "parallel")]
        {
            let solved = crate::par::map_ordered(spans, runner, "adaptive-path-spans", |span, worker| {
                let mut own = OperationReport::default();
                let result = self.adaptive_span(span, spans.len(), settings, &mut own, worker);
                (result, own)
            })?;
            for (result, own) in solved {
                if report.counts.samples + own.counts.samples > settings.max_samples {
                    return Err(CamError::InvalidInput("drop path sample work budget exhausted".into()));
                }
                let output = result?;
                report.counts.samples += own.counts.samples;
                report.counts.candidates += own.counts.candidates;
                report.counts.contacts += own.counts.contacts;
                report.warnings.extend(own.warnings);
                report.errors.extend(own.errors);
                report.approximations.extend(own.approximations);
                if let Some(output) = output {
                    paths.push(output);
                }
            }
        }
        report.approximations.push(if self.profile_is_convex() {
            ApproximationLimit { kind: "conservative-segment-lift".into(), bound: Some(settings.chord_error), detail: Some("Each cut chord cleared by its sloped-segment excess over every triangle in reach (concave per triangle; sampled with chord-extension upper bounds); both ends raised by it, within chord tolerance".into()) }
        } else {
            ApproximationLimit { kind: "conservative-segment-lift".into(), bound: Some(settings.chord_error), detail: Some("Each cut chord cleared by a midpoint radial-dilation bound; endpoints may be raised within chord tolerance".into()) }
        });
        report.finish();
        runner.stage("adaptive-path-complete","Certified spans projected",paths.len() as u64,spans.len() as u64)?;
        Ok((paths,report))
    }

    /// One span of [`Self::adaptive`]: its certified points, or `None` for a
    /// degenerate span skipped with a warning.
    fn adaptive_span(&self, span: &Span, span_count: usize, settings: &Sampling, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<Option<Vec<ClPoint>>,CamError> {
        span.validate(&self.tolerance)?;
        if span.length2d() <= self.tolerance.sample_eps && span_count > 1 {
            report.warn(Diagnostic::new("degenerate-span","Skipped a degenerate source span")); return Ok(None);
        }
        let a = self.counted(span.point_at(0.),settings,report,runner)?;
        let b = self.counted(span.point_at(1.),settings,report,runner)?;
        let mut output = vec![a.clone()];
        let mut stack = vec![(0.,1.,a,b,0u32)];
        let convex = self.profile_is_convex();
        let (mut steps, mut widest_step) = (0usize, 0.0f64);
        while let Some((t0,t1,a,b,depth)) = stack.pop() {
            let tm = (t0+t1)*0.5;
            let m = self.counted(span.point_at(tm),settings,report,runner)?;
            let u = m.position().sub(a.position()); let v = b.position().sub(m.position());
            let flat = match (u.normalized(),v.normalized()) { (Some(u),Some(v)) => u.dot(v) >= settings.flatness_cos_limit, _ => true };
            let chord_error = m.position().distance(a.position().lerp(b.position(),0.5));
            let source_length = span.length2d()*(t1-t0);
            let contact = |p:&ClPoint| p.contact.as_ref().and_then(|c|c.triangle_id).map(|id| self.mesh.triangle(id).face);
            let topology = contact(&a) != contact(&b) || contact(&a) != contact(&m);
            // An interval longer than the sample spacing, or whose chord
            // misses the midpoint, is refined whatever its swept bound;
            // the bound (a drop with the cutter dilated by half the
            // interval, the costly query) is only taken for an interval
            // that could be accepted, where it decides `safe` and the lift.
            let refine_cheap = source_length > settings.sample_spacing || chord_error > settings.chord_error
                || ((!flat || topology) && source_length > settings.min_sample_spacing.max(settings.chord_error));
            // The swept certificate: for a convex profile the exact
            // sloped-segment excess (`segment_excess`), lifting both ends
            // by it; otherwise the constant-height dilated bound.
            let (lift_a, lift_b, safe) = if refine_cheap {
                (a.z, b.z, true)
            } else if convex {
                let excess = self.segment_excess(a.position(),b.position(),settings.chord_error,report,runner)?;
                (a.z + excess.max(0.0), b.z + excess.max(0.0), excess <= settings.chord_error)
            } else {
                let bound = self.segment_bound(a.position(),b.position(),settings,report,runner)?;
                (a.z.max(bound), b.z.max(bound), bound <= a.z.min(b.z)+settings.chord_error)
            };
            let refine = refine_cheap || !safe;
            if refine {
                if depth >= settings.max_depth || source_length <= settings.min_sample_spacing {
                    // The depth budget is a budget: only an interval at
                    // the minimum spacing (refinement exhausted, not cut
                    // short) is stepped over.
                    if !convex || depth >= settings.max_depth || source_length > settings.min_sample_spacing {
                        return Err(CamError::InvalidInput(format!("adaptive path cannot certify tolerance at depth {depth}; decrease minimum spacing or increase depth")));
                    }
                    // A jump in the drop height (the cutter reaching the
                    // top edge of a wall taller than its corner) cannot be
                    // followed by a straight cut. Step over it: rise
                    // vertically, cross at a height the certificate clears,
                    // descend vertically. Vertical moves above a dropped
                    // point never gouge; material is left only inside this
                    // interval, no wider than the minimum spacing.
                    let top = a.z.max(b.z);
                    let (pa, pb) = (Point3::new(a.x, a.y, top), Point3::new(b.x, b.y, top));
                    let e = self.segment_excess(pa, pb, f64::INFINITY, report, runner)?.max(0.0);
                    let h = top + e;
                    let last = output.last().unwrap().clone();
                    if h > last.z {
                        let mut up = last; up.z = h; output.push(up);
                    }
                    let mut across = b.clone(); across.z = h; output.push(across);
                    if b.z < h {
                        output.push(b);
                    }
                    steps += 1;
                    widest_step = widest_step.max(source_length);
                    continue;
                }
                stack.push((tm,t1,m.clone(),b,depth+1));
                stack.push((t0,tm,a,m,depth+1));
            } else {
                let last = output.last_mut().unwrap(); last.z = last.z.max(lift_a);
                let mut end = b; end.z = end.z.max(lift_b); output.push(end);
            }
        }
        if steps > 0 {
            report.approximations.push(ApproximationLimit { kind: "drop-height-step".into(), bound: Some(widest_step), detail: Some(format!("{steps} jumps in the drop height (wall tops) stepped over vertically; material is left only within {widest_step:.3e} of each, along the path")) });
        }
        Ok(Some(output))
    }
}
/// The XY distance between the segment `p`-`q` and the convex polygon
/// `poly` (vertices in order), and the segment parameter where it is
/// attained: zero where they meet; otherwise the least of the polygon's
/// vertices to the segment and the segment's ends to the polygon (for two
/// disjoint convex sets the least distance is at a vertex of one of them).
fn segment_polygon_distance(p: Point3, q: Point3, poly: &[(f64, f64)]) -> (f64, f64) {
    let (dx, dy) = (q.x - p.x, q.y - p.y);
    let l2 = dx * dx + dy * dy;
    let n = poly.len();
    let inside = |x: f64, y: f64| {
        let mut sign = 0.0f64;
        for i in 0..n {
            let (ax, ay) = poly[i];
            let (bx, by) = poly[(i + 1) % n];
            let c = (bx - ax) * (y - ay) - (by - ay) * (x - ax);
            if c != 0.0 {
                if sign == 0.0 { sign = c.signum(); } else if c.signum() != sign { return false; }
            }
        }
        true
    };
    if inside(p.x, p.y) { return (0.0, 0.0); }
    if inside(q.x, q.y) { return (0.0, 1.0); }
    // Crossing a polygon edge.
    let mut best = (f64::INFINITY, 0.0);
    for i in 0..n {
        let (ax, ay) = poly[i];
        let (bx, by) = poly[(i + 1) % n];
        let (ex, ey) = (bx - ax, by - ay);
        let den = dx * ey - dy * ex;
        if den != 0.0 {
            let u = ((ax - p.x) * ey - (ay - p.y) * ex) / den;
            let v = ((ax - p.x) * dy - (ay - p.y) * dx) / den;
            if (0.0..=1.0).contains(&u) && (0.0..=1.0).contains(&v) {
                return (0.0, u);
            }
        }
        // Polygon vertex to the segment.
        let u = if l2 > 0.0 { (((ax - p.x) * dx + (ay - p.y) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
        let d = (p.x + u * dx - ax).hypot(p.y + u * dy - ay);
        if d < best.0 { best = (d, u); }
        // Segment ends to the polygon edge.
        let el2 = ex * ex + ey * ey;
        for (t, (x, y)) in [(0.0, (p.x, p.y)), (1.0, (q.x, q.y))] {
            let w = if el2 > 0.0 { (((x - ax) * ex + (y - ay) * ey) / el2).clamp(0.0, 1.0) } else { 0.0 };
            let d = (ax + w * ex - x).hypot(ay + w * ey - y);
            if d < best.0 { best = (d, t); }
        }
    }
    best
}

/// The XY distance from `(x, y)` to triangle `t` (zero inside).
fn xy_distance_to_triangle(t: &crate::mesh::Triangle, x: f64, y: f64) -> f64 {
    if super::inside_xy(t, x, y) {
        return 0.0;
    }
    let seg = |a: Point3, b: Point3| {
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let l2 = dx * dx + dy * dy;
        let u = if l2 > 0.0 { (((x - a.x) * dx + (y - a.y) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
        (a.x + u * dx - x).hypot(a.y + u * dy - y)
    };
    seg(t.a, t.b).min(seg(t.b, t.c)).min(seg(t.c, t.a))
}

/// The parameter range `[t0, t1]` of the segment `a`-`b` over which its XY
/// point lies within `reach` of the box (Liang-Barsky against the box grown
/// by `reach`), or `None`.
fn clip_param(a: Point3, b: Point3, bounds: &crate::model::Bounds3, reach: f64) -> Option<(f64, f64)> {
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (p, d, lo, hi) in [(a.x, b.x - a.x, bounds.min.x - reach, bounds.max.x + reach), (a.y, b.y - a.y, bounds.min.y - reach, bounds.max.y + reach)] {
        if d == 0.0 {
            if p < lo || p > hi { return None; }
        } else {
            let (mut u0, mut u1) = ((lo - p) / d, (hi - p) / d);
            if u0 > u1 { std::mem::swap(&mut u0, &mut u1); }
            t0 = t0.max(u0);
            t1 = t1.min(u1);
            if t0 > t1 { return None; }
        }
    }
    Some((t0, t1))
}

/// An upper bound of the maximum over `[t0, t1]` of a concave function `f`
/// defined on a sub-interval (`None` outside it), from samples: on each gap
/// between samples a concave function lies below the extensions of the
/// chords on both sides. Samples are added where the bound is highest until
/// the bound is within `resolution` of the best sample, or at most `target`
/// (certified below what the caller already has), or `max_samples` are
/// taken; `None` when no sample meets the domain.
fn concave_max_bound(f: &mut dyn FnMut(f64) -> Result<Option<f64>, CamError>, gap_cap: &dyn Fn(f64, f64) -> f64, t0: f64, t1: f64, inside: f64, target: f64, resolution: f64, max_samples: usize) -> Result<Option<f64>, CamError> {
    let mut s: Vec<(f64, Option<f64>)> = Vec::with_capacity(max_samples);
    for k in 0..=4 {
        let t = t0 + (t1 - t0) * k as f64 / 4.0;
        s.push((t, f(t)?));
    }
    // A sample known to be inside the domain (the closest approach).
    if let Err(i) = s.binary_search_by(|x| x.0.total_cmp(&inside)) {
        let v = f(inside)?;
        s.insert(i, (inside, v));
    }
    loop {
        let best = s.iter().filter_map(|x| x.1).fold(f64::NEG_INFINITY, f64::max);
        // Gap bounds.
        let mut worst = (f64::NEG_INFINITY, usize::MAX);
        let mut worst_at = 0.5f64;
        let line = |i: usize, j: usize, t: f64| -> Option<f64> {
            let (ti, vi) = (s[i].0, s[i].1?);
            let (tj, vj) = (s[j].0, s[j].1?);
            if tj == ti { return None; }
            Some(vi + (vj - vi) * (t - ti) / (tj - ti))
        };
        let any_finite = best.is_finite();
        for g in 0..s.len() - 1 {
            let (ta, tb) = (s[g].0, s[g + 1].0);
            let (fa, fb) = (s[g].1.is_some(), s[g + 1].1.is_some());
            if !fa && !fb {
                // The domain is an interval: with a finite sample elsewhere
                // it cannot lie in this gap; with none it may.
                if any_finite { continue; }
                let cap = gap_cap(ta, tb);
                if cap > worst.0 { worst = (cap, g); worst_at = 0.5; }
                continue;
            }
            let left = if g >= 1 { line(g - 1, g, ta).and(line(g - 1, g, tb)).map(|_| (g - 1, g)) } else { None };
            let right = if g + 2 < s.len() { line(g + 1, g + 2, ta).and(line(g + 1, g + 2, tb)).map(|_| (g + 1, g + 2)) } else { None };
            // The bound and where in the gap (fraction) it peaks.
            let (ub, at) = match (left, right) {
                (Some((i, j)), Some((k, l))) => {
                    // Max over [ta, tb] of the lower of two lines.
                    let (la, lb) = (line(i, j, ta).unwrap(), line(i, j, tb).unwrap());
                    let (ra, rb) = (line(k, l, ta).unwrap(), line(k, l, tb).unwrap());
                    let m0 = la.min(ra);
                    let m1 = lb.min(rb);
                    let (dl, dr) = (lb - la, rb - ra);
                    let cross = if dl != dr { let u = (ra - la) / (dl - dr); if (0.0..=1.0).contains(&u) { Some((la + dl * u, u)) } else { None } } else { None };
                    let mut best = if m0 >= m1 { (m0, 0.0) } else { (m1, 1.0) };
                    if let Some((v, u)) = cross { if v > best.0 { best = (v, u); } }
                    best
                }
                (Some((i, j)), None) => { let (va, vb) = (line(i, j, ta).unwrap(), line(i, j, tb).unwrap()); if va >= vb { (va, 0.0) } else { (vb, 1.0) } }
                (None, Some((k, l))) => { let (va, vb) = (line(k, l, ta).unwrap(), line(k, l, tb).unwrap()); if va >= vb { (va, 0.0) } else { (vb, 1.0) } }
                (None, None) => (f64::INFINITY, 0.5),
            };
            // Every gap is also bounded by its own key (the triangle's top
            // over the gap): finite, and tight for a small gap at the reach.
            let (ub, at) = { let cap = gap_cap(ta, tb); if cap < ub { (cap, 0.5) } else { (ub, at) } };
            // The samples themselves bound the gap from below only; the
            // bound is never below them.
            let ub = ub.max(s[g].1.unwrap_or(f64::NEG_INFINITY)).max(s[g + 1].1.unwrap_or(f64::NEG_INFINITY));
            if ub > worst.0 { worst = (ub, g); worst_at = at; }
        }
        let bound = worst.0.max(best);
        if !any_finite && (s.len() >= max_samples || bound <= target) {
            return Ok(if bound.is_finite() { Some(bound) } else { None });
        }
        // Stop when the bound cannot raise what the caller has (`target`),
        // or when a positive excess is pinned within the resolution. While
        // no sample is above the target the bound is driven down to it
        // (to rounding): a touching contact (the segment's own end on the
        // surface, excess exactly zero) must not lift the segment by the
        // resolution, which would make the output depend on rounding.
        let touch = 1e-12 * (1.0 + t1.abs().max(t0.abs()) + target.abs());
        let pinned = if best > target + touch { bound - best <= resolution } else { bound <= target + touch };
        if any_finite && (bound <= target || pinned || s.len() >= max_samples) {
            return Ok(Some(bound));
        }
        if worst.1 == usize::MAX {
            return Ok(if any_finite { Some(bound) } else { None });
        }
        // Sample where the bound peaks (the cutting-plane rule), kept off the
        // gap's ends so every gap shrinks geometrically.
        let g = worst.1;
        let u = worst_at.clamp(0.1, 0.9);
        let t = s[g].0 + (s[g + 1].0 - s[g].0) * u;
        if t <= s[g].0 || t >= s[g + 1].0 {
            return Ok(if any_finite { Some(bound) } else { None });
        }
        let v = f(t)?;
        s.insert(g + 1, (t, v));
    }
}

