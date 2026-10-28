//! What the 2.5D operations share: the resolved planar input, depth levels,
//! pass assembly from exact loops/chains, the no-gouge certificate of every
//! constant-Z chord against the full protected mesh, and the hand-over to
//! the linker (lane C4's `link::connect`), filter and arc fitter.
//!
//! A pass is certified in two parts. Above the cut the push cutter's
//! horizontal sweep (`push::PushContext::segment`) at `z + tolerance` must
//! report no collision interval: every wall, fixture and step higher than
//! the machining tolerance above the cut is caught with the full cutter
//! profile. At the cut the drop cutter's point bound at the chord ends must
//! not exceed `z`: the floor under the pass is at or below the cut. An
//! operation whose pass fails either check is an error, never a usable path.

use super::{Chain, Loop, PlanarError, Region, Vec2};
use serde::{Serialize, Deserialize};
use crate::arcfit::ArcFitOptions;
use crate::drop::DropCutter;
use crate::link::{connect, LinkContext, LinkOptions, Pass};
use crate::model::{
    CutDirection, Depths, Diagnostic, FeedsSpeeds, Heights, Motion, Operation, Point2, Point3, Setup, Tab, Tool,
    Toolpath,
};
use crate::push::PushContext;
use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};

/// A planar region at a machine height (a planar face, a sketch profile, a
/// stock outline).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlanarRegion {
    pub region: Region,
    /// Machine Z of the plane.
    pub z: f64,
    /// `true` when the plane's normal points up (+Z) in machine space.
    pub upward: bool,
}

/// A chain of edges at one machine height.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlanarChain {
    pub chain: Chain,
    pub z: f64,
}

/// A circular feature for bore / circular / thread milling.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Circle2d {
    pub center: Point2,
    /// Radius of the hole (internal) or boss (external).
    pub radius: f64,
    /// Machine Z of the feature top.
    pub top: f64,
    /// Machine Z of the feature bottom.
    pub bottom: f64,
    /// A hole (`true`) or a boss (`false`).
    pub internal: bool,
    /// For a boss: the radius out to which material is cleared (the stock).
    pub outer_radius: Option<f64>,
}

impl Circle2d {
    pub fn hole(center: Point2, radius: f64, top: f64, bottom: f64) -> Self {
        Self { center, radius, top, bottom, internal: true, outer_radius: None }
    }

    pub fn boss(center: Point2, radius: f64, top: f64, bottom: f64, outer_radius: f64) -> Self {
        Self { center, radius, top, bottom, internal: false, outer_radius: Some(outer_radius) }
    }

    pub fn validate(&self, strategy: &str) -> Result<(), CamError> {
        if !self.center.is_finite() || !self.radius.is_finite() || !self.top.is_finite() || !self.bottom.is_finite() {
            return Err(invalid(strategy, "a circle has a non-finite parameter"));
        }
        if self.radius <= 0.0 {
            return Err(invalid(strategy, "a circle has a non-positive radius"));
        }
        if self.bottom > self.top {
            return Err(invalid(strategy, "a circle's bottom is above its top"));
        }
        if let Some(o) = self.outer_radius {
            if !o.is_finite() || o <= self.radius {
                return Err(invalid(strategy, "a boss's outer radius must exceed its radius"));
            }
        }
        Ok(())
    }
}

/// The resolved geometry a 2.5D operation works on, in machine space. The
/// caller resolves the operation's targets (through
/// `kernel_adapter::planar` in the app, directly in tests) and hands it over;
/// the operation never sees the kernel.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PlanarInput {
    pub regions: Vec<PlanarRegion>,
    pub chains: Vec<PlanarChain>,
    pub circles: Vec<Circle2d>,
    /// Machine Z of the stock top when known (the default top of cut).
    pub stock_top: Option<f64>,
}

/// Everything an operation validates once: tool, heights, feeds, mesh.
pub struct OpContext<'a> {
    pub operation: &'a Operation,
    pub setup: &'a Setup,
    pub tool: &'a Tool,
    pub mesh: &'a MeshSnapshot,
    pub index: Option<&'a dyn TriangleIndex>,
    pub tolerance: Tolerance,
    pub cutter: Cutter,
    pub heights: Heights,
    pub feeds: FeedsSpeeds,
    pub strategy: &'static str,
    /// Top of the material the operation removes, recorded by `top_of_cut`
    /// (the highest of the operation's, the setup's and the stock's tops):
    /// the linker keeps rapids and partial retracts above the stock this
    /// operation has not cut yet (lane C4's `LinkOptions::material_top`).
    pub material_top: std::cell::Cell<Option<f64>>,
}

pub fn invalid(strategy: &str, message: impl std::fmt::Display) -> CamError {
    CamError::InvalidInput(format!("{strategy}: {message}"))
}

impl<'a> OpContext<'a> {
    /// Validate the operation's shared inputs for `strategy`.
    pub fn new(
        strategy: &'static str,
        operation: &'a Operation,
        setup: &'a Setup,
        tool: &'a Tool,
        mesh: &'a MeshSnapshot,
        index: Option<&'a dyn TriangleIndex>,
        tolerance: Tolerance,
    ) -> Result<Self, CamError> {
        tolerance.validate()?;
        operation.validate().map_err(|m| invalid(strategy, m))?;
        if !operation.enabled {
            return Err(invalid(strategy, "operation is disabled"));
        }
        if operation.setup != setup.id || operation.tool != tool.id {
            return Err(invalid(strategy, "setup or tool identity mismatch"));
        }
        if let Some(axis) = operation.tool_axis {
            if axis.normalized().map_or(true, |a| a.distance(Point3::Z) > tolerance.angle_eps) {
                return Err(invalid(strategy, "requires the machine +Z tool axis; transform indexed setups first"));
            }
        }
        if operation.stock_to_leave.radial < 0.0 || operation.stock_to_leave.axial < 0.0 {
            return Err(invalid(strategy, "stock to leave must not be negative"));
        }
        let cutter = Cutter::from_tool(tool)?;
        let heights = operation.heights.clone().unwrap_or_else(|| setup.heights.clone());
        heights.validate().map_err(|m| invalid(strategy, m))?;
        let feeds = operation.feeds.clone().unwrap_or_else(|| tool.feeds.clone());
        feeds.validate().map_err(|m| invalid(strategy, m))?;
        if mesh.is_empty() {
            return Err(CamError::EmptyTarget { operation_id: operation.id.to_string() });
        }
        for solid in operation.targets.protected_bodies() {
            if !mesh.faces.iter().any(|f| f.solid == solid) {
                return Err(CamError::UnresolvedTarget { solid, face: None });
            }
        }
        for face in operation.targets.drive_faces.iter().chain(&operation.targets.avoid_faces) {
            if mesh.resolve_faces(face).is_empty() {
                return Err(CamError::UnresolvedTarget { solid: face.solid.clone(), face: face.face.clone() });
            }
        }
        if heights.clearance <= mesh.bounds.max.z + tolerance.distance_eps {
            return Err(invalid(strategy, "clearance height must be above the full protected mesh"));
        }
        // The drop cutter validates the cutter against the mesh and index once.
        DropCutter::new(mesh, &cutter, index, tolerance)?;
        Ok(Self { operation, setup, tool, mesh, index, tolerance, cutter, heights, feeds, strategy, material_top: std::cell::Cell::new(None) })
    }

    pub fn radius(&self) -> f64 {
        self.cutter.radius()
    }

    /// The default top of cut: the operation's heights, else the stock top
    /// from the input, else the mesh top.
    pub fn top_of_cut(&self, depths: &Depths, input: &PlanarInput) -> f64 {
        let top = depths.top.or(self.heights.top).or(input.stock_top).unwrap_or(self.mesh.bounds.max.z);
        let material = [depths.top, self.heights.top, input.stock_top].into_iter().flatten().fold(top, f64::max);
        self.note_material_top(material);
        top
    }

    /// Raise the recorded material top to `z` (a feature's own top above the
    /// operation's default, e.g. a circle's).
    pub fn note_material_top(&self, z: f64) {
        self.material_top.set(Some(self.material_top.get().map_or(z, |m: f64| m.max(z))));
    }

    /// The highest material over a feature whose own top is `local_top`, for
    /// strategies that write their own approaches (bore, thread mill): their
    /// rapid to the feed height must stop above the stock, not the feature.
    pub fn material_over(&self, depths: &Depths, input: &PlanarInput, local_top: f64) -> f64 {
        self.top_of_cut(depths, input);
        self.note_material_top(local_top);
        self.material_top.get().unwrap_or(local_top)
    }

    /// Certify one constant-Z polyline pass against the full protected mesh.
    /// `floor_exempt` skips the floor check (chamfers and engraving cut into
    /// the model by design and certify walls and fixtures only).
    pub fn certify(&self, points: &[Point3], report: &mut crate::model::OperationReport, runner: &mut Runner<'_>) -> Result<(), CamError> {
        self.certify_with(points, false, report, runner)
    }

    pub fn certify_with(&self, points: &[Point3], fixtures_only: bool, report: &mut crate::model::OperationReport, runner: &mut Runner<'_>) -> Result<(), CamError> {
        self.certify_with_cutter(points, &self.cutter, fixtures_only, report, runner)
    }

    /// As [`Self::certify_with`] with an explicit cutter (a strategy that
    /// cuts a slot of exactly its own width certifies with the cutter shrunk
    /// by the tolerance band, since the walls are the cut itself).
    pub fn certify_with_cutter(&self, points: &[Point3], cutter: &Cutter, fixtures_only: bool, report: &mut crate::model::OperationReport, runner: &mut Runner<'_>) -> Result<(), CamError> {
        if points.len() < 2 {
            return Ok(());
        }
        let tol = self.tolerance;
        let z = points[0].z;
        if points.iter().any(|p| (p.z - z).abs() > tol.distance_eps) {
            return Err(invalid(self.strategy, "a pass is not at constant Z"));
        }
        if fixtures_only {
            // Only protected-only bodies and avoid faces are checked.
            let keep: Vec<bool> = self
                .mesh
                .faces
                .iter()
                .map(|f| f.protected_only || self.operation.targets.protected_solids.contains(&f.solid) || self.operation.targets.avoid_faces.iter().any(|a| a.solid == f.solid && a.face.as_ref().map_or(true, |n| *n == f.face)))
                .collect();
            if !keep.iter().any(|k| *k) {
                return Ok(());
            }
            let mut builder = crate::MeshBuilder::new(tol);
            let mut map = vec![u32::MAX; self.mesh.faces.len()];
            for (i, f) in self.mesh.faces.iter().enumerate() {
                if keep[i] {
                    map[i] = builder.add_face(f.clone());
                }
            }
            for t in &self.mesh.triangles {
                if keep[t.face as usize] {
                    builder.add_triangle(map[t.face as usize], t.a, t.b, t.c).map_err(|e| invalid(self.strategy, e))?;
                }
            }
            let sub = builder.finish();
            if sub.is_empty() {
                return Ok(());
            }
            return certify_against(&sub, cutter, None, tol, self.operation.tolerance, points, self.strategy, report, runner);
        }
        certify_against(self.mesh, cutter, self.index, tol, self.operation.tolerance, points, self.strategy, report, runner)
    }

    /// Certify a 3D polyline (a helix): every chord's walls are checked at
    /// the chord's lower Z and the floor at both ends.
    pub fn certify_3d(&self, points: &[Point3], fixtures_only: bool, report: &mut crate::model::OperationReport, runner: &mut Runner<'_>) -> Result<(), CamError> {
        for w in points.windows(2) {
            let z = w[0].z.min(w[1].z);
            let flat = [Point3::new(w[0].x, w[0].y, z), Point3::new(w[1].x, w[1].y, z)];
            self.certify_with(&flat, fixtures_only, report, runner)?;
        }
        Ok(())
    }

    /// Wrap directly emitted motions (helical strategies) into the toolpath
    /// with the report finished and counted; no linker, filter or arc fit.
    pub fn direct(&self, motions: Vec<Motion>, passes: u64, mut report: crate::model::OperationReport) -> Result<Toolpath, CamError> {
        if motions.is_empty() {
            return Err(invalid(self.strategy, "no motions produced"));
        }
        let mut path = Toolpath::new(self.operation.id.clone(), self.tool.id.clone());
        path.motions = motions;
        if let Some(i) = path.first_non_finite() {
            return Err(CamError::NonFinite { what: format!("motion {i}") });
        }
        report.counts.triangles = self.mesh.triangle_count() as u64;
        report.counts.passes = passes;
        report.counts.motions = path.motions.len() as u64;
        report.finish();
        path.report = report;
        Ok(path)
    }

    /// Link certified passes into the operation's motion stream: proven
    /// approaches, links, leads and ramps from the linker, then the chord
    /// filter and the arc fitter.
    pub fn finish(&self, passes: &[Pass], reorder: bool, mut report: crate::model::OperationReport, runner: &mut Runner<'_>) -> Result<Toolpath, CamError> {
        if passes.is_empty() {
            return Err(invalid(self.strategy, "no passes to link (empty or unreachable region)"));
        }
        let options = LinkOptions {
            chord_tolerance: self.operation.tolerance,
            reorder,
            allow_reverse: false,
            material_top: self.material_top.get(),
            ..LinkOptions::default()
        };
        let linked = crate::link::with_current_pre_op_stock(|stock| {
        let ctx = LinkContext {
            mesh: self.mesh,
            cutter: &self.cutter,
            index: self.index,
            tolerance: self.tolerance,
            heights: &self.heights,
            feeds: &self.feeds,
            params: &self.operation.linking,
            options,
            stock,
        };
        connect(passes, &ctx, runner)
        })?;
        report.warnings.extend(linked.report.warnings);
        report.counts.samples += linked.report.counts.samples;
        report.counts.candidates += linked.report.counts.candidates;
        report.counts.contacts += linked.report.counts.contacts;
        let mut path = Toolpath::new(self.operation.id.clone(), self.tool.id.clone());
        path.motions = linked.motions;
        path.report = report;
        let filtered = crate::filter::filter_toolpath(&path, self.tolerance.distance_eps)?;
        let mut out = if self.operation.linking.fit_arcs {
            crate::arcfit::fit_toolpath(&filtered, &ArcFitOptions::with_tolerance(self.operation.tolerance))?
        } else {
            filtered
        };
        out.report.counts.triangles = self.mesh.triangle_count() as u64;
        out.report.counts.passes = passes.len() as u64;
        out.report.counts.motions = out.motions.len() as u64;
        out.report.finish();
        Ok(out)
    }
}

/// Certify the chords of `points` against `mesh` (see the module doc).
#[allow(clippy::too_many_arguments)]
fn certify_against(
    mesh: &MeshSnapshot,
    cutter: &Cutter,
    index: Option<&dyn TriangleIndex>,
    tol: Tolerance,
    machining_tolerance: f64,
    points: &[Point3],
    strategy: &str,
    report: &mut crate::model::OperationReport,
    runner: &mut Runner<'_>,
) -> Result<(), CamError> {
    let z = points[0].z;
    let push = PushContext { mesh, cutter, index, tolerance: tol, allowance: 0.0 };
    push.validate()?;
    let lift = machining_tolerance.max(2.0 * tol.distance_eps);
    let drop = DropCutter::new(mesh, cutter, index, tol)?;
    let floor = mesh.bounds.min.z - cutter.total_height() - 1.0;
    for (i, p) in points.iter().enumerate() {
        report.counts.samples += 1;
        // Over a floor every candidate's top is at the pass height and the
        // bound cannot exceed it: the cutter is dropped only near walls.
        let (clear, candidates) = drop.bound_at_most(p.x, p.y, z + lift, runner)?;
        if clear {
            report.counts.candidates += candidates;
            continue;
        }
        let bound = drop.point(p.x, p.y, floor, runner)?;
        report.counts.candidates += bound.candidate_count;
        report.counts.contacts += bound.contact_count;
        if bound.point.z > z + lift {
            let what = bound.point.contact.as_ref().map(|c| format!("{:?} contact with triangle {:?} at {:?}", c.contact_type, c.triangle_id, c.point)).unwrap_or_default();
            let d = Diagnostic::new("gouge-detected", format!("{strategy}: the cutter at point {i} of a pass would cut {:.4} below the model surface ({what})", bound.point.z - z)).at(*p);
            let text = format!("a pass gouges the model at ({:.4}, {:.4}, {:.4}): {}", p.x, p.y, p.z, d.message);
            report.fail(d);
            return Err(invalid(strategy, text));
        }
    }
    for w in points.windows(2) {
        let (a, b) = (Point3::new(w[0].x, w[0].y, z + lift), Point3::new(w[1].x, w[1].y, z + lift));
        runner.work(1, "certify-chords", 0, 0)?;
        report.counts.samples += 1;
        if let Some((at, triangle, dist, radius)) = chord_collision(mesh, cutter, index, a, b, tol) {
            let d = Diagnostic::new(
                "gouge-detected",
                format!("{strategy}: a pass chord collides with protected geometry above the cut: triangle {triangle} comes within {dist:.4} of the chord where the cutter is {radius:.4} wide"),
            )
            .at(at);
            let text = format!("a pass collides with the model or a fixture at ({:.4}, {:.4}, {:.4}): {}", at.x, at.y, at.z, d.message);
            report.fail(d);
            return Err(invalid(strategy, text));
        }
    }
    let _ = &push;
    Ok(())
}

/// Exact swept-envelope collision of a horizontal chord `a`-`b` (the tool
/// reference point at `a.z`) with the mesh: every triangle is clipped to
/// the cutter's height slabs and its XY distance to the chord compared with
/// the cutter's radius in that slab (exact for cylindrical slabs, within
/// the slab's radius growth otherwise, so never optimistic). Returns the
/// offending point, triangle id, its distance and the radius there.
pub fn chord_collision(mesh: &MeshSnapshot, cutter: &Cutter, index: Option<&dyn TriangleIndex>, a: Point3, b: Point3, tol: Tolerance) -> Option<(Point3, u32, f64, f64)> {
    let z0 = a.z;
    let height = cutter.total_height();
    let radius = cutter.radius().max(cutter.shaft_radius());
    let lo = Point3::new(a.x.min(b.x) - radius, a.y.min(b.y) - radius, z0);
    let hi = Point3::new(a.x.max(b.x) + radius, a.y.max(b.y) + radius, z0 + height);
    let bounds = crate::model::Bounds3::new(lo, hi);
    let ids: Vec<u32> = match index {
        Some(i) => i.query(&bounds, crate::ProjectionMode::Xyz),
        None => (0..mesh.triangle_count() as u32).collect(),
    };
    // Height slabs: profile segment ranges, the shank, subdivided where the
    // radius changes so the slab's top radius is a tight bound.
    let mut breaks: Vec<f64> = vec![0.0, cutter.cutting_length(), height];
    for s in cutter.segments() {
        breaks.extend(s.height_range);
    }
    breaks.retain(|h| h.is_finite() && *h >= 0.0 && *h <= height);
    breaks.sort_by(f64::total_cmp);
    breaks.dedup_by(|x, y| (*x - *y).abs() <= 1e-12);
    let mut slabs: Vec<(f64, f64, f64)> = Vec::new(); // (h0, h1, radius bound)
    for w in breaks.windows(2) {
        let (h0, h1) = (w[0], w[1]);
        if h1 - h0 <= 0.0 {
            continue;
        }
        let r0 = cutter.max_radius_at_height(h0.max(1e-12)).unwrap_or(0.0);
        let r1 = cutter.max_radius_at_height(h1).unwrap_or(radius);
        let parts = if (r1 - r0).abs() <= tol.distance_eps { 1 } else { 16 };
        for k in 0..parts {
            let s0 = h0 + (h1 - h0) * k as f64 / parts as f64;
            let s1 = h0 + (h1 - h0) * (k + 1) as f64 / parts as f64;
            let rb = cutter.max_radius_at_height(s1).unwrap_or(radius).max(cutter.max_radius_at_height(s0.max(1e-12)).unwrap_or(0.0));
            slabs.push((s0, s1, rb));
        }
    }
    let chord = (Point2::new(a.x, a.y), Point2::new(b.x, b.y));
    let mut worst: Option<(Point3, u32, f64, f64)> = None;
    for id in ids {
        let t = mesh.triangle(id);
        if t.max_z() <= z0 + tol.distance_eps || t.min_z() >= z0 + height {
            continue;
        }
        if t.bounds.min.x > hi.x || t.bounds.max.x < lo.x || t.bounds.min.y > hi.y || t.bounds.max.y < lo.y {
            continue;
        }
        for &(h0, h1, rb) in &slabs {
            let (zlo, zhi) = (z0 + h0, z0 + h1);
            if t.max_z() <= zlo + tol.distance_eps || t.min_z() >= zhi - tol.distance_eps {
                continue;
            }
            let poly = clip_to_slab(&t.vertices(), zlo + tol.distance_eps, zhi);
            if poly.len() < 2 {
                continue;
            }
            let (d, at) = polygon_chord_distance(&poly, chord);
            if d < rb - tol.distance_eps && worst.as_ref().map_or(true, |w| d < w.2) {
                worst = Some((at, id, d, rb));
            }
        }
    }
    worst
}

/// A triangle clipped to `zlo <= z <= zhi` (convex polygon, possibly empty).
fn clip_to_slab(tri: &[Point3; 3], zlo: f64, zhi: f64) -> Vec<Point3> {
    let mut poly: Vec<Point3> = tri.to_vec();
    for (level, keep_above) in [(zlo, true), (zhi, false)] {
        let mut out = Vec::with_capacity(poly.len() + 2);
        let n = poly.len();
        for i in 0..n {
            let p = poly[i];
            let q = poly[(i + 1) % n];
            let inside = |v: Point3| if keep_above { v.z >= level } else { v.z <= level };
            if inside(p) {
                out.push(p);
            }
            if inside(p) != inside(q) && (q.z - p.z).abs() > 0.0 {
                let f = (level - p.z) / (q.z - p.z);
                out.push(p.lerp(q, f.clamp(0.0, 1.0)));
            }
        }
        poly = out;
        if poly.is_empty() {
            break;
        }
    }
    poly
}

/// Minimal XY distance between a convex polygon and a segment, with the
/// polygon point realising it (zero when the chord crosses the polygon).
fn polygon_chord_distance(poly: &[Point3], chord: (Point2, Point2)) -> (f64, Point3) {
    let n = poly.len();
    let mut best = (f64::INFINITY, poly[0]);
    let xy = |p: Point3| Point2::new(p.x, p.y);
    // Chord endpoints inside the polygon (n >= 3) mean overlap.
    if n >= 3 {
        for c in [chord.0, chord.1] {
            let mut sign = 0.0f64;
            let mut inside = true;
            for i in 0..n {
                let e = xy(poly[(i + 1) % n]).sub(xy(poly[i]));
                let s = e.cross(c.sub(xy(poly[i])));
                if sign == 0.0 {
                    sign = s;
                } else if s * sign < 0.0 {
                    inside = false;
                    break;
                }
            }
            if inside && sign != 0.0 {
                return (0.0, Point3::new(c.x, c.y, poly[0].z));
            }
        }
    }
    let edges = if n >= 3 { n } else { 1 };
    for i in 0..edges {
        let p = poly[i];
        let q = poly[(i + 1) % n];
        let (d, at) = segment_segment_distance(xy(p), xy(q), chord.0, chord.1);
        if d < best.0 {
            let f = if xy(q).distance(xy(p)) > 0.0 { at.distance(xy(p)) / xy(q).distance(xy(p)) } else { 0.0 };
            best = (d, p.lerp(q, f.clamp(0.0, 1.0)));
        }
    }
    best
}

/// Distance between segments `p0p1` and `q0q1` in 2D and the point on the
/// first realising it.
fn segment_segment_distance(p0: Point2, p1: Point2, q0: Point2, q1: Point2) -> (f64, Point2) {
    let sp = super::Segment::line(p0, p1);
    let sq = super::Segment::line(q0, q1);
    let cross = |a: Point2, b: Point2| a.x * b.y - a.y * b.x;
    // Proper crossing: distance zero.
    let d1 = p1.sub(p0);
    let d2 = q1.sub(q0);
    let denom = cross(d1, d2);
    if denom.abs() > 1e-18 {
        let w = q0.sub(p0);
        let t = cross(w, d2) / denom;
        let u = cross(w, d1) / denom;
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            return (0.0, sp.point_at(t));
        }
    }
    let mut best = (f64::INFINITY, p0);
    for (seg, pts, on_p) in [(&sq, [p0, p1], true), (&sp, [q0, q1], false)] {
        for e in pts {
            let (near, _) = seg.nearest(e);
            let d = near.distance(e);
            if d < best.0 {
                best = (d, if on_p { e } else { near });
            }
        }
    }
    best
}

/// Depth levels from `top` down to `bottom`: equal roughing steps no larger
/// than `step_down` (one level when it is zero), then `finishing_passes`
/// levels of `finishing_step_down` ending at `bottom`. Descending Z.
pub fn depth_levels(top: f64, bottom: f64, depths: &Depths, strategy: &str) -> Result<Vec<f64>, CamError> {
    if !top.is_finite() || !bottom.is_finite() {
        return Err(invalid(strategy, "top or bottom of cut is not finite"));
    }
    if bottom > top + 1e-9 {
        return Err(invalid(strategy, format!("bottom of cut {bottom} is above the top of cut {top}")));
    }
    let finishing = if depths.finishing_passes > 0 && depths.finishing_step_down > 0.0 {
        (depths.finishing_passes as f64 * depths.finishing_step_down).min(top - bottom)
    } else {
        0.0
    };
    let rough_bottom = bottom + finishing;
    let mut levels = Vec::new();
    let span = top - rough_bottom;
    if span > 1e-9 {
        let n = if depths.step_down > 0.0 { (span / depths.step_down - 1e-9).ceil().max(1.0) } else { 1.0 };
        if n > 100_000.0 {
            return Err(invalid(strategy, "depth level budget exceeded"));
        }
        let n = n as usize;
        for i in 1..=n {
            levels.push(top - span * i as f64 / n as f64);
        }
    } else {
        levels.push(rough_bottom);
    }
    if finishing > 0.0 {
        let n = depths.finishing_passes as usize;
        for i in 1..=n {
            levels.push(rough_bottom - finishing * i as f64 / n as f64);
        }
    }
    // Exactly the bottom last.
    if let Some(last) = levels.last_mut() {
        *last = bottom;
    }
    levels.dedup_by(|a, b| (*a - *b).abs() <= 1e-9);
    Ok(levels)
}

/// Orient a loop for the cut direction. `material_right` says whether the
/// uncut material lies on the right of the loop as given (true for the
/// inward offsets of pockets and inside contours, false for outward
/// offsets). Climb milling keeps the material on the right of travel.
/// Returns the oriented loop and whether the material is on its right.
pub fn orient_for(loop_: &Loop, material_right: bool, direction: CutDirection, alternate: bool) -> (Loop, bool) {
    let climb = match direction {
        CutDirection::Climb => true,
        CutDirection::Conventional => false,
        CutDirection::Either => !alternate,
    };
    if climb == material_right {
        (loop_.clone(), material_right)
    } else {
        (loop_.reversed(), !material_right)
    }
}

/// Chord points of a segment for a pass: arcs whose centre side holds the
/// material are discretised outward (the chords never dip into it), the
/// others inward.
fn pass_points(seg: &super::Segment, chord_tol: f64, material_right: Option<bool>) -> Vec<Point2> {
    match (seg.arc, material_right) {
        (Some(a), Some(mr)) if (!a.ccw) == mr => seg.discretize_outward(chord_tol),
        _ => seg.discretize(chord_tol),
    }
}

/// A closed loop as a constant-Z polyline pass starting at its first
/// vertex, closed back onto it, with an optional overlap along the loop.
pub fn loop_pass(loop_: &Loop, z: f64, chord_tol: f64, overlap: f64, path_id: u32, material_right: Option<bool>) -> Pass {
    let mut pts: Vec<Point2> = Vec::new();
    for s in &loop_.segments {
        let p = pass_points(s, chord_tol, material_right);
        pts.extend_from_slice(&p[..p.len() - 1]);
    }
    pts.push(pts[0]);
    if overlap > 0.0 {
        let mut acc = 0.0;
        let mut extra = Vec::new();
        for w in pts.clone().windows(2) {
            let d = w[0].distance(w[1]);
            if acc + d >= overlap {
                let f = ((overlap - acc) / d).clamp(0.0, 1.0);
                extra.push(w[0].add(w[1].sub(w[0]).scale(f)));
                break;
            }
            acc += d;
            extra.push(w[1]);
        }
        pts.extend(extra);
    }
    Pass::new(pts.iter().map(|p| Point3::new(p.x, p.y, z)).collect(), Some(path_id))
}

/// An open chain as a constant-Z polyline pass.
pub fn chain_pass(chain: &Chain, z: f64, chord_tol: f64, path_id: u32, material_right: Option<bool>) -> Pass {
    let mut pts: Vec<Point2> = Vec::new();
    for (i, s) in chain.segments.iter().enumerate() {
        let p = pass_points(s, chord_tol, material_right);
        if i == 0 {
            pts.push(p[0]);
        }
        pts.extend_from_slice(&p[1..]);
    }
    Pass::new(pts.iter().map(|p| Point3::new(p.x, p.y, z)).collect(), Some(path_id))
}

/// A full circle of tool-centre radius `rho` at `z` as chord points, outward
/// when the material is inside the circle (a boss), inward for a hole.
pub fn circle_points(center: Point2, rho: f64, ccw: bool, z: f64, chord_tol: f64, material_inside: bool) -> Vec<Point3> {
    let l = Loop::circle(center, rho, ccw);
    // Inside the circle is the centre side: material there means outward chords.
    let mr = if material_inside { Some(!ccw) } else { Some(ccw) };
    let mut pts: Vec<Point2> = Vec::new();
    for s in &l.segments {
        let p = pass_points(s, chord_tol, mr);
        pts.extend_from_slice(&p[..p.len() - 1]);
    }
    pts.push(pts[0]);
    pts.iter().map(|p| Point3::new(p.x, p.y, z)).collect()
}

/// Raise the pass over its tabs: within each tab's window along the pass
/// (centre at `at` times the pass length, `width` wide) the cut rises to
/// `bottom + height` when the pass is below that, with vertical steps at
/// the window edges.
pub fn apply_tabs(pass: &mut Pass, tabs: &[Tab], bottom: f64) {
    if tabs.is_empty() {
        return;
    }
    let total: f64 = pass.points.windows(2).map(|w| w[0].distance_xy(w[1])).sum();
    if total <= 0.0 {
        return;
    }
    let z = pass.points[0].z;
    let mut out: Vec<Point3> = Vec::new();
    for tab in tabs {
        let ztab = bottom + tab.height;
        if ztab <= z || tab.width <= 0.0 {
            continue;
        }
        let s0 = (tab.at.rem_euclid(1.0) * total - 0.5 * tab.width).max(0.0);
        let s1 = (s0 + tab.width).min(total);
        out.clear();
        let mut acc = 0.0;
        let pts = pass.points.clone();
        for (i, w) in pts.windows(2).enumerate() {
            let a = w[0];
            let b = w[1];
            let d = a.distance_xy(b);
            if i == 0 {
                out.push(a);
            }
            let mut push_at = |s: f64, zz: f64| {
                let f = if d > 0.0 { ((s - acc) / d).clamp(0.0, 1.0) } else { 0.0 };
                let p = a.lerp(b, f);
                out.push(Point3::new(p.x, p.y, zz));
            };
            if s0 > acc && s0 < acc + d {
                push_at(s0, z);
                push_at(s0, ztab);
            }
            if s1 > acc && s1 < acc + d {
                push_at(s1, ztab);
                push_at(s1, z);
            }
            let end_z = if acc + d > s0 && acc + d < s1 { ztab } else { b.z };
            out.push(Point3::new(b.x, b.y, end_z));
            acc += d;
        }
        pass.points = out.clone();
    }
}

/// The exact motion count check every operation runs before linking.
pub fn non_empty(passes: &[Pass], strategy: &str) -> Result<(), CamError> {
    if passes.is_empty() {
        Err(invalid(strategy, "the operation produced no passes: empty or unreachable region"))
    } else {
        Ok(())
    }
}

impl From<PlanarError> for crate::model::Diagnostic {
    fn from(e: PlanarError) -> Self {
        Diagnostic::new("planar", e.to_string())
    }
}

/// Keep a motion list's path ids compact after an operation (helper for
/// strategies that assemble several passes).
pub fn renumber_paths(motions: &mut [Motion]) {
    let mut next = 0u32;
    let mut last: Option<u32> = None;
    for m in motions.iter_mut() {
        if let Some(id) = m.path_id {
            if last != Some(id) {
                last = Some(id);
                next += 1;
            }
            m.path_id = Some(next - 1);
        }
    }
}
