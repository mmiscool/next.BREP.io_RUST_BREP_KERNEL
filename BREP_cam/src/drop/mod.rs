//! Deterministic vertical cutter projection in machine coordinates.
//!
//! Each contact maximizes `mesh_z - bottom_profile(radius)`. The five shared
//! cutter bottoms are convex radial graphs, making each finite-edge objective
//! concave. Facet support is analytic; edge maximization uses a bounded bracket.

use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, TriangleIndex, Tolerance, Runner};
use crate::cutter::SegmentKind;
use crate::mesh::{FaceSource, Triangle};
use crate::model::{ClPoint, Contact, ContactType, Point3, OperationReport};

pub mod path;
pub mod finish;
pub mod containment;
pub(crate) mod geodesic;

/// One point and the stable source identity of its winning contact.
#[derive(Clone, Debug, PartialEq)]
pub struct DropResult {
    pub point: ClPoint,
    pub source: Option<FaceSource>,
    pub candidate_count: u64,
    pub contact_count: u64,
}

/// Reusable validated projection context. `mesh` must contain ALL protected
/// geometry, including unselected target faces and fixtures. The optional index
/// must have been built from this same immutable snapshot.
pub struct DropCutter<'a> {
    pub(crate) mesh: &'a MeshSnapshot,
    pub(crate) cutter: &'a dyn CutterProfile,
    pub(crate) index: Option<&'a dyn TriangleIndex>,
    pub(crate) tolerance: Tolerance,
    /// A shank wider than the flutes, as a flat cutter of the shank's radius
    /// whose tip sits at the flutes' top: for a vertical tool the shank's
    /// lowest surface is that annulus, so the tool's bound is the larger of
    /// the flutes' and the shank's bound lowered by the cutting length.
    pub(crate) shank: Option<Cutter>,
}

impl<'a> DropCutter<'a> {
    pub fn new(mesh: &'a MeshSnapshot, cutter: &'a Cutter, index: Option<&'a dyn TriangleIndex>, tolerance: Tolerance) -> Result<Self, CamError> {
        Self::from_profile(mesh, cutter, index, tolerance)
    }

    pub(crate) fn from_profile(mesh: &'a MeshSnapshot, cutter: &'a dyn CutterProfile, index: Option<&'a dyn TriangleIndex>, tolerance: Tolerance) -> Result<Self, CamError> {
        cutter.validate()?;
        tolerance.validate()?;
        if mesh.is_empty() { return Err(CamError::EmptyTarget { operation_id: "drop-cutter".into() }); }
        let shank = (cutter.shaft_radius() > cutter.radius() && cutter.shaft_length() > 0.0)
            .then(|| Cutter::flat(2.0 * cutter.shaft_radius(), cutter.cutting_length(), 0.0));
        if let Some(i) = index {
            if i.stats().triangle_count != mesh.triangle_count() {
                return Err(CamError::InvalidInput("drop index does not match the protected mesh".into()));
            }
        }
        Ok(Self { mesh, cutter, index, tolerance, shank })
    }

    /// The wider shank's own drop context and the height its tip sits above
    /// the tool's (the cutting length), or `None` for a shank no wider than
    /// the flutes.
    pub(crate) fn shank(&self) -> Option<(DropCutter<'_>, f64)> {
        self.shank.as_ref().map(|c| (DropCutter { mesh: self.mesh, cutter: c, index: self.index, tolerance: self.tolerance, shank: None }, self.cutter.cutting_length()))
    }

    pub fn mesh(&self) -> &MeshSnapshot { self.mesh }
    pub fn cutter(&self) -> &dyn CutterProfile { self.cutter }

    /// Whether the floor bound at `(x, y)` is certainly at or below `limit`,
    /// decided from the candidate triangles' top heights alone: every contact
    /// lies on a triangle and the cutter's profile height is never negative,
    /// so the bound never exceeds the highest candidate. `Ok((true, n))` with
    /// the candidate count means [`Self::point`] would return a bound at most
    /// `limit`; `Ok((false, n))` means nothing (call `point`). A certificate
    /// over a floor asks this first and drops the cutter only near walls.
    pub fn bound_at_most(&self, x: f64, y: f64, limit: f64, runner: &mut Runner<'_>) -> Result<(bool, u64), CamError> {
        runner.token().check()?;
        if ![x, y, limit].iter().all(|n| n.is_finite()) {
            return Err(CamError::NonFinite { what: "drop point or limit".into() });
        }
        let radius = self.cutter.radius();
        let eps = self.tolerance.distance_eps;
        let mut ids = Vec::new();
        if let Some(index) = self.index {
            index.query_cutter_xy(x, y, radius + eps, &mut ids);
        } else {
            ids.extend(0..self.mesh.triangle_count() as u32);
        }
        let mut candidates = 0u64;
        let mut clear = true;
        for id in ids {
            let tri = self.mesh.triangles.get(id as usize).ok_or_else(|| CamError::InvalidInput("drop index contains an invalid triangle id".into()))?;
            if tri.bounds.min.x > x + radius + eps || tri.bounds.max.x < x - radius - eps
                || tri.bounds.min.y > y + radius + eps || tri.bounds.max.y < y - radius - eps { continue; }
            candidates += 1;
            if tri.max_z() > limit {
                clear = false;
            }
        }
        runner.work(candidates.max(1), "drop-candidates", 0, self.mesh.triangle_count() as u64)?;
        if let Some((shank, lift)) = self.shank() {
            let (shank_clear, shank_candidates) = shank.bound_at_most(x, y, limit + lift, runner)?;
            return Ok((clear && shank_clear, candidates + shank_candidates));
        }
        Ok((clear, candidates))
    }

    pub fn point(&self, x: f64, y: f64, floor_z: f64, runner: &mut Runner<'_>) -> Result<DropResult, CamError> {
        runner.token().check()?;
        if ![x, y, floor_z].iter().all(|n| n.is_finite()) {
            return Err(CamError::NonFinite { what: "drop point or floor".into() });
        }
        let mut result = DropResult { point: ClPoint::new(x, y, floor_z), source: None, candidate_count: 0, contact_count: 0 };
        let mut ids = Vec::new();
        if let Some(index) = self.index {
            index.query_cutter_xy(x, y, self.cutter.radius() + self.tolerance.distance_eps, &mut ids);
            ids.sort_unstable();
            ids.dedup();
        } else {
            ids.extend(0..self.mesh.triangle_count() as u32);
        }
        let radius = self.cutter.radius();
        let eps = self.tolerance.distance_eps;
        // Candidates in order of the highest contact each could give: a
        // contact on a triangle lies at or below the triangle's top and at a
        // radius at least the triangle's box distance from the axis, where
        // the profile is at least as high as there (the profile rises with
        // the radius), so `top - h(r_min)` bounds the triangle's contacts
        // from above. Triangles below the running best cannot change it;
        // equal ties across triangles go to the lower id, which is what the
        // id-ordered scan recorded, so the result is the same to the bit
        // (`drop::tests` and the benchmark digests compare it).
        let mut keyed: Vec<(f64, u32)> = Vec::with_capacity(ids.len());
        for id in ids {
            let tri = self.mesh.triangles.get(id as usize).ok_or_else(|| CamError::InvalidInput("drop index contains an invalid triangle id".into()))?;
            // Every candidate visited is a work unit, so cancellation is
            // observed inside the candidate loop whether or not it is evaluated.
            runner.work(1, "drop-candidates", keyed.len() as u64, self.mesh.triangle_count() as u64)?;
            // Use identical bounds filtering in both modes so counts and ties agree.
            if tri.bounds.min.x > x + radius + eps || tri.bounds.max.x < x - radius - eps
                || tri.bounds.min.y > y + radius + eps || tri.bounds.max.y < y - radius - eps { continue; }
            let dx = (tri.bounds.min.x - x).max(x - tri.bounds.max.x).max(0.0);
            let dy = (tri.bounds.min.y - y).max(y - tri.bounds.max.y).max(0.0);
            let r_min = dx.hypot(dy).min(radius);
            let h = self.cutter.height_at_radius(r_min).ok_or_else(|| CamError::InvalidInput("invalid cutter profile evaluation".into()))?;
            keyed.push((tri.max_z() - h, id));
        }
        keyed.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        let mut best_id = u32::MAX;
        for (key, id) in keyed {
            // Below the best: nothing here or after can raise it. Equal to
            // it: a contact here can only tie, and a tie wins only with a
            // lower id, while equal keys come in ascending id order and
            // every later key is lower.
            if key < result.point.z || (key == result.point.z && result.point.contact.is_some() && id > best_id) {
                break;
            }
            let tri = &self.mesh.triangles[id as usize];
            result.candidate_count += 1;
            if !tri.vertices().iter().all(Point3::is_finite) {
                return Err(CamError::NonFinite { what: format!("triangle {id}") });
            }
            let mut accept = |p: Point3, kind: ContactType, upper: Option<f64>| -> Result<(), CamError> {
                let r = p.distance_xy(Point3::new(x, y, 0.));
                if r > radius + eps { return Ok(()); }
                let h = self.cutter.height_at_radius(r.min(radius)).ok_or_else(|| CamError::InvalidInput("invalid cutter profile evaluation".into()))?;
                let z = upper.unwrap_or(p.z - h);
                if !z.is_finite() { return Err(CamError::NonFinite { what: "drop contact".into() }); }
                result.contact_count += 1;
                // Within one triangle the first contact of a height wins
                // (vertices, edges, facet); across triangles the lower id.
                let better = z > result.point.z || (z == result.point.z && (result.point.contact.is_none() || id < best_id));
                if better {
                    result.point.z = z;
                    result.point.contact = Some(Contact { contact_type: kind, point: Some(p), triangle_id: Some(id), distance: Some(r), detail: None });
                    result.source = Some(self.mesh.face_of(id).clone());
                    best_id = id;
                }
                Ok(())
            };
            // Stable tie order: vertices, edges, then facet; triangle ids ascend.
            for p in tri.vertices() { accept(p, ContactType::Vertex, None)?; }
            for (a, b) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
                if let Some((p, z)) = self.edge(x, y, a, b)? { accept(p, ContactType::Edge, Some(z))?; }
            }
            let n = tri.area_vector();
            if n.z != 0.0 {
                let sx = -n.x / n.z;
                let sy = -n.y / n.z;
                let slope = sx.hypot(sy);
                if !slope.is_finite() { return Err(CamError::NonFinite { what: "facet slope".into() }); }
                for seg in self.cutter.segments() {
                    let [lo, hi] = seg.radius_range;
                    let r = match seg.kind {
                        SegmentKind::Flat => if slope == 0. { lo } else { hi },
                        SegmentKind::Sphere | SegmentKind::Torus => (seg.ring_radius + seg.profile_radius * (slope / slope.hypot(1.))).clamp(lo, hi),
                        SegmentKind::Cone => if slope > 1. / seg.half_angle_rad.tan() { hi } else { lo },
                        SegmentKind::Cylinder => continue,
                        // Upper surfaces (lollipop ball above its equator, dovetail flank) never touch from above.
                        SegmentKind::SphereUpper | SegmentKind::ConeInverted => continue,
                    };
                    let (px, py) = if slope == 0. { (x, y) } else { (x + r * sx / slope, y + r * sy / slope) };
                    if inside_xy(tri, px, py) {
                        let pz = tri.a.z + sx * (px - tri.a.x) + sy * (py - tri.a.y);
                        accept(Point3::new(px, py, pz), ContactType::Facet, None)?;
                    }
                }
            }
        }
        if let Some((shank, lift)) = self.shank() {
            let lifted = shank.point(x, y, floor_z + lift, runner)?;
            result.candidate_count += lifted.candidate_count;
            result.contact_count += lifted.contact_count;
            // The flutes keep a tie: the shank wins only by standing higher.
            if lifted.point.z - lift > result.point.z {
                result.point.z = lifted.point.z - lift;
                result.point.contact = lifted.point.contact.map(|c| Contact { detail: Some("shank".into()), ..c });
                result.source = lifted.source;
            }
        }
        Ok(result)
    }

    /// The highest cutter-tip height at which the cutter centred at `(x, y)`
    /// touches triangle `tri` alone (vertices, edges, facet: the same contacts
    /// as [`Self::point`]), or `None` when the triangle is out of reach. Edge
    /// contacts are the converged edge point's exact value (for a convex
    /// profile the edge contact is concave and the search converges to it).
    /// The flag is set when the maximum came from an edge (a bracket).
    pub(crate) fn triangle_contact(&self, tri: &Triangle, x: f64, y: f64, segments: &[crate::cutter::ProfileSegment]) -> Result<Option<(f64, bool)>, CamError> {
        let radius = self.cutter.radius();
        let eps = self.tolerance.distance_eps;
        let mut best: Option<(f64, bool)> = None;
        let mut take_kind = |z: f64, edge: bool| { if best.map_or(true, |b| z > b.0) { best = Some((z, edge)); } };
        for p in tri.vertices() {
            let r = p.distance_xy(Point3::new(x, y, 0.));
            if r <= radius + eps {
                let h = self.cutter.height_at_radius(r.min(radius)).ok_or_else(|| CamError::InvalidInput("invalid cutter profile evaluation".into()))?;
                take_kind(p.z - h, false);
            }
        }
        for (a, b) in [(tri.a, tri.b), (tri.b, tri.c), (tri.c, tri.a)] {
            // The edge's converged contact point, evaluated exactly (its
            // height minus the profile at its distance): for a convex profile
            // the contact along an edge is concave and the ternary search
            // has converged to it; `edge`'s bracket value is looser.
            if let Some(z) = self.edge_contact_concave(x, y, a, b)? {
                take_kind(z, true);
            }
        }
        let n = tri.area_vector();
        if n.z != 0.0 {
            let sx = -n.x / n.z;
            let sy = -n.y / n.z;
            let slope = sx.hypot(sy);
            if !slope.is_finite() { return Err(CamError::NonFinite { what: "facet slope".into() }); }
            for seg in segments {
                let [lo, hi] = seg.radius_range;
                let r = match seg.kind {
                    SegmentKind::Flat => if slope == 0. { lo } else { hi },
                    SegmentKind::Sphere | SegmentKind::Torus => (seg.ring_radius + seg.profile_radius * (slope / slope.hypot(1.))).clamp(lo, hi),
                    SegmentKind::Cone => if slope > 1. / seg.half_angle_rad.tan() { hi } else { lo },
                    SegmentKind::Cylinder | SegmentKind::SphereUpper | SegmentKind::ConeInverted => continue,
                };
                let (px, py) = if slope == 0. { (x, y) } else { (x + r * sx / slope, y + r * sy / slope) };
                if inside_xy(tri, px, py) {
                    let pz = tri.a.z + sx * (px - tri.a.x) + sy * (py - tri.a.y);
                    let rr = Point3::new(px, py, 0.).distance_xy(Point3::new(x, y, 0.));
                    if rr <= radius + eps {
                        let h = self.cutter.height_at_radius(rr.min(radius)).ok_or_else(|| CamError::InvalidInput("invalid cutter profile evaluation".into()))?;
                        take_kind(pz - h, false);
                    }
                }
            }
        }
        Ok(best)
    }

    /// The contact height of edge `a`-`b` for a convex profile: the edge
    /// point's height minus the profile at its distance is concave along the
    /// edge, so a ternary search stopped at a 1e-7 bracket of the edge has
    /// its value within rounding at an interior maximum (the function is
    /// flat to second order there), and the clipped ends are evaluated too
    /// (exact at a boundary maximum).
    fn edge_contact_concave(&self, x: f64, y: f64, a: Point3, b: Point3) -> Result<Option<f64>, CamError> {
        let d = b.sub(a);
        let q = d.x * d.x + d.y * d.y;
        let radius = self.cutter.radius();
        let axis = Point3::new(x, y, 0.);
        let value = |t: f64| -> Option<f64> {
            let p = a.lerp(b, t);
            let r = p.distance_xy(axis);
            (r <= radius).then(|| p.z - self.cutter.height_at_radius(r).unwrap_or(f64::INFINITY))
        };
        if !d.is_finite() || !q.is_finite() { return Err(CamError::NonFinite { what: "edge contact coefficients".into() }); }
        if q == 0. {
            let r = a.distance_xy(axis);
            if r > radius { return Ok(None); }
            return Ok(Some(a.z.max(b.z) - self.cutter.height_at_radius(r).unwrap_or(f64::INFINITY)));
        }
        let near = -((a.x - x) * d.x + (a.y - y) * d.y) / q;
        let closest = a.lerp(b, near);
        let perpendicular = closest.distance_xy(axis);
        if !(perpendicular <= radius) { return Ok(None); }
        let half = ((radius - perpendicular) * (radius + perpendicular) / q).max(0.).sqrt();
        let lo0 = (near - half).max(0.);
        let hi0 = (near + half).min(1.);
        if lo0 > hi0 { return Ok(None); }
        // Golden-section search: one evaluation per step.
        let v = |t: f64| value(t).unwrap_or(f64::NEG_INFINITY);
        let g = 0.5 * (5f64.sqrt() - 1.0);
        let (mut lo, mut hi) = (lo0, hi0);
        let (mut l, mut r) = (hi - g * (hi - lo), lo + g * (hi - lo));
        let (mut vl, mut vr) = (v(l), v(r));
        while hi - lo > 1e-7 {
            if vl < vr {
                lo = l; l = r; vl = vr; r = lo + g * (hi - lo); vr = v(r);
            } else {
                hi = r; r = l; vr = vl; l = hi - g * (hi - lo); vl = v(l);
            }
        }
        let best = [value(0.5 * (lo + hi)), value(lo0), value(hi0)].into_iter().flatten().fold(f64::NEG_INFINITY, f64::max);
        Ok(best.is_finite().then_some(best))
    }

    /// Conservative maximum on an edge clipped to the cutter disc. A bracket
    /// upper bound is returned, so termination cannot under-estimate a contact.
    fn edge(&self, x: f64, y: f64, a: Point3, b: Point3) -> Result<Option<(Point3, f64)>, CamError> {
        let d = b.sub(a);
        let dx = a.x - x;
        let dy = a.y - y;
        let q = d.x*d.x + d.y*d.y;
        let radius = self.cutter.radius();
        if !d.is_finite() || !q.is_finite() { return Err(CamError::NonFinite { what: "edge contact coefficients".into() }); }
        if q == 0. {
            if dx.hypot(dy) > radius { return Ok(None); }
            let p = if a.z >= b.z { a } else { b };
            return Ok(Some((p, p.z - self.cutter.height_at_radius(dx.hypot(dy)).unwrap())));
        }
        let near = -(dx*d.x + dy*d.y) / q;
        if !near.is_finite() { return Err(CamError::NonFinite { what: "edge closest approach".into() }); }
        let closest = a.lerp(b, near);
        let perpendicular = closest.distance_xy(Point3::new(x, y, 0.));
        if perpendicular > radius { return Ok(None); }
        let half = ((radius - perpendicular) * (radius + perpendicular) / q).max(0.).sqrt();
        let mut lo = (near - half).max(0.);
        let mut hi = (near + half).min(1.);
        if lo > hi { return Ok(None); }
        let value = |t: f64| {
            let p = a.lerp(b, t);
            let r = p.distance_xy(Point3::new(x, y, 0.)).min(radius);
            p.z - self.cutter.height_at_radius(r).unwrap()
        };
        for _ in 0..96 {
            let l = lo + (hi-lo)/3.;
            let r = hi - (hi-lo)/3.;
            if l == lo || r == hi { break; }
            if value(l) < value(r) { lo = l; } else { hi = r; }
        }
        let mid = (lo+hi)*0.5;
        let p = a.lerp(b, mid);
        let rmin = a.lerp(b, near.clamp(lo, hi)).distance_xy(Point3::new(x, y, 0.)).min(radius);
        let upper = a.lerp(b, lo).z.max(a.lerp(b, hi).z) - self.cutter.height_at_radius(rmin).unwrap();
        if !upper.is_finite() || upper - value(mid) > self.tolerance.distance_eps {
            return Err(CamError::InvalidInput("edge contact failed to converge within distance tolerance".into()));
        }
        Ok(Some((p, upper)))
    }

    /// Transactional: no partial batch is returned on cancellation or error.
    pub fn batch(&self, xy: &[[f64; 2]], floor_z: f64, runner: &mut Runner<'_>) -> Result<(Vec<ClPoint>, OperationReport), CamError> {
        runner.stage("drop-points", "Project cutter locations", 0, xy.len() as u64)?;
        let mut report = OperationReport::default();
        report.counts.triangles = self.mesh.triangle_count() as u64;
        let mut points = Vec::with_capacity(xy.len());
        for (i, &[x,y]) in xy.iter().enumerate() {
            let result = self.point(x, y, floor_z, runner)?;
            report.counts.samples += 1;
            report.counts.candidates += result.candidate_count;
            report.counts.contacts += result.contact_count;
            points.push(result.point);
            runner.work(1, "drop-points", i as u64 + 1, xy.len() as u64)?;
        }
        report.finish();
        runner.stage("drop-complete", "Cutter locations projected", xy.len() as u64, xy.len() as u64)?;
        Ok((points, report))
    }
}

fn inside_xy(t: &Triangle, x: f64, y: f64) -> bool {
    let cross = |a: Point3, b: Point3| (b.x-a.x)*(y-a.y) - (b.y-a.y)*(x-a.x);
    let a = cross(t.a,t.b); let b = cross(t.b,t.c); let c = cross(t.c,t.a);
    (a >= 0. && b >= 0. && c >= 0.) || (a <= 0. && b <= 0. && c <= 0.)
}


