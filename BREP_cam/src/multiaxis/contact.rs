//! Contact in the tool frame (lane C10, spec section 5): the primitive rows
//! 3, 4 and 5 share.
//!
//! [`drop_along`] drops the cutter along an arbitrary tool axis: the index
//! candidates around the tool's capsule are transformed into the tool frame
//! (tip at the origin, axis `+Z`) and the exact 3-axis drop cutter runs at
//! `(0, 0)`. The result is the signed offset along the axis at which the
//! cutting part (and its shank) first touches: positive means the requested
//! tip gouges by that much, negative means it clears by that much.
//!
//! [`clearance_along`] tests the full revolved [`ToolProfile`] (cutting part,
//! shank, holder frustums) against the same candidates with exact
//! segment-triangle distances per cylindrical slab; tapered bands are
//! over-approximated by slabs whose radius grows at most `slab` (stated as
//! the approximation bound).

use crate::cutter::{CutterProfile, ToolPart, ToolProfile};
use crate::drop::DropCutter;
use crate::model::{Bounds3, Contact, Point3, RigidTransform};
use crate::{CamError, MeshSnapshot, ProjectionMode, Runner, Tolerance, Triangle, TriangleIndex};

/// A tool position: tip and unit axis in setup coordinates with the rigid
/// transforms into the tool frame.
#[derive(Clone, Debug, PartialEq)]
pub struct ToolFrame {
    pub tip: Point3,
    pub axis: Point3,
    pub to_local: RigidTransform,
    pub to_setup: RigidTransform,
}

impl ToolFrame {
    pub fn new(tip: Point3, axis: Point3, tolerance: &Tolerance) -> Result<ToolFrame, CamError> {
        let z = axis.normalized().ok_or_else(|| CamError::InvalidInput("tool axis must be a non-zero vector".into()))?;
        if !tip.is_finite() {
            return Err(CamError::NonFinite { what: "tool tip".into() });
        }
        let x_ref = if z.cross(Point3::X).length() <= tolerance.angle_eps.max(1e-12).sqrt() { Point3::Y } else { Point3::X };
        let to_local = RigidTransform::from_frame(tip, x_ref, z)?;
        Ok(ToolFrame { tip, axis: z, to_setup: to_local.inverse(), to_local })
    }

    /// Setup point of a tool-frame point.
    pub fn setup_of(&self, local: Point3) -> Point3 {
        self.to_setup.apply_point(local)
    }

    /// Tool-frame point of a setup point.
    pub fn local_of(&self, p: Point3) -> Point3 {
        self.to_local.apply_point(p)
    }
}

/// Triangle ids whose bounds meet the tool's capsule: radius `radius`
/// around the axis segment from local height `z_lo` to `z_hi`.
pub fn candidates(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    frame: &ToolFrame,
    radius: f64,
    z_lo: f64,
    z_hi: f64,
) -> Vec<u32> {
    let a = frame.tip.add(frame.axis.scale(z_lo));
    let b = frame.tip.add(frame.axis.scale(z_hi));
    let query = Bounds3::from_points([a, b]).expanded(radius);
    match index {
        Some(i) => {
            let mut ids = i.query(&query, ProjectionMode::Xyz);
            ids.sort_unstable();
            ids.dedup();
            ids
        }
        None => mesh
            .triangles
            .iter()
            .filter(|t| ProjectionMode::Xyz.overlaps(&t.bounds, &query))
            .map(|t| t.id)
            .collect(),
    }
}

/// The candidate triangles in the tool frame as a snapshot; the returned
/// vector maps the snapshot's triangle ids back to the source mesh.
fn sub_mesh(mesh: &MeshSnapshot, ids: &[u32], frame: &ToolFrame) -> (MeshSnapshot, Vec<u32>) {
    let mut faces = Vec::new();
    let mut face_map: Vec<Option<u32>> = vec![None; mesh.faces.len()];
    let mut triangles = Vec::with_capacity(ids.len());
    let mut bounds = Bounds3::empty();
    for (i, &id) in ids.iter().enumerate() {
        let t = &mesh.triangles[id as usize];
        let face = match face_map[t.face as usize] {
            Some(f) => f,
            None => {
                faces.push(mesh.faces[t.face as usize].clone());
                let f = (faces.len() - 1) as u32;
                face_map[t.face as usize] = Some(f);
                f
            }
        };
        let a = frame.to_local.apply_point(t.a);
        let b = frame.to_local.apply_point(t.b);
        let c = frame.to_local.apply_point(t.c);
        let tb = Bounds3::from_points([a, b, c]);
        bounds = bounds.union(&tb);
        triangles.push(Triangle { id: i as u32, a, b, c, bounds: tb, normal: frame.to_local.apply_vector(t.normal), face });
    }
    (MeshSnapshot { faces, triangles, bounds, report: mesh.report }, ids.to_vec())
}

/// Result of [`drop_along`].
#[derive(Clone, Debug, PartialEq)]
pub struct AxisDrop {
    /// Signed offset along the axis of the first contact relative to the
    /// requested tip: `> 0` gouge, `< 0` clearance, `-reach` when nothing is
    /// within reach.
    pub offset: f64,
    /// The winning contact (triangle ids in the source mesh), in setup coordinates.
    pub contact: Option<Contact>,
    pub candidates: u64,
}

/// Drop `cutter` along `axis` toward `-axis`, starting with the tip at `tip`
/// and looking `reach` beyond it.
pub fn drop_along(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    cutter: &dyn CutterProfile,
    tip: Point3,
    axis: Point3,
    reach: f64,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<AxisDrop, CamError> {
    if !reach.is_finite() || reach < 0.0 {
        return Err(CamError::InvalidInput("drop reach must be finite and non-negative".into()));
    }
    let frame = ToolFrame::new(tip, axis, &tolerance)?;
    let radius = cutter.radius().max(cutter.shaft_radius()) + tolerance.distance_eps;
    let ids = candidates(mesh, index, &frame, radius, -reach, cutter.total_height());
    if ids.is_empty() {
        return Ok(AxisDrop { offset: -reach, contact: None, candidates: 0 });
    }
    let (local, map) = sub_mesh(mesh, &ids, &frame);
    let drop = DropCutter::from_profile(&local, cutter, None, tolerance)?;
    let result = drop.point(0.0, 0.0, -reach, runner)?;
    let contact = result.point.contact.map(|mut c| {
        if let Some(p) = c.point {
            c.point = Some(frame.setup_of(p));
        }
        if let Some(t) = c.triangle_id {
            c.triangle_id = map.get(t as usize).copied();
        }
        c
    });
    Ok(AxisDrop { offset: result.point.z, contact, candidates: ids.len() as u64 })
}

/// A collision (or the nearest approach) of the tool solid.
#[derive(Clone, Debug, PartialEq)]
pub struct ClearanceHit {
    /// Triangle id in the source mesh.
    pub triangle: u32,
    pub part: ToolPart,
    /// Local height band `[lo, hi]` of the slab that hit.
    pub band: [f64; 2],
}

/// Result of [`clearance_along`].
#[derive(Clone, Debug, PartialEq)]
pub struct Clearance {
    /// Smallest (surface distance − slab radius) over the checked bands:
    /// negative is a collision. `+inf` without candidates.
    pub min_gap: f64,
    /// The first slab (lowest band, lowest triangle id) that collides.
    pub hit: Option<ClearanceHit>,
    /// Largest radius over-approximation of the slabs (tapered bands).
    pub bound: f64,
    pub candidates: u64,
}

/// Clearance of the revolved `profile` (only bands of the given `parts`)
/// placed with its tip at `tip` along `axis`, against the mesh. Bands are
/// cut into slabs whose radius varies at most `slab`; each slab is tested as
/// a cylinder of its larger radius.
pub fn clearance_along(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    profile: &ToolProfile,
    parts: &[ToolPart],
    tip: Point3,
    axis: Point3,
    slab: f64,
    tolerance: Tolerance,
) -> Result<Clearance, CamError> {
    if !slab.is_finite() || slab <= 0.0 {
        return Err(CamError::InvalidInput("clearance slab must be finite and positive".into()));
    }
    let frame = ToolFrame::new(tip, axis, &tolerance)?;
    let radius = profile.max_radius() + tolerance.distance_eps;
    let ids = candidates(mesh, index, &frame, radius, 0.0, profile.top_height());
    let mut out = Clearance { min_gap: f64::INFINITY, hit: None, bound: 0.0, candidates: ids.len() as u64 };
    if ids.is_empty() {
        return Ok(out);
    }
    let (local, map) = sub_mesh(mesh, &ids, &frame);
    for w in profile.points.windows(2) {
        let (a, b) = (w[0], w[1]);
        if !parts.contains(&a.part) || b.height <= a.height {
            continue;
        }
        let steps = (((b.radius - a.radius).abs() / slab).ceil() as usize).max(1);
        let r_lo = a.radius.min(b.radius);
        let r_hi = a.radius.max(b.radius);
        for s in 0..steps {
            let f0 = s as f64 / steps as f64;
            let f1 = (s + 1) as f64 / steps as f64;
            let h0 = a.height + (b.height - a.height) * f0;
            let h1 = a.height + (b.height - a.height) * f1;
            let ra = a.radius + (b.radius - a.radius) * f0;
            let rb = a.radius + (b.radius - a.radius) * f1;
            let r = ra.max(rb).clamp(r_lo, r_hi);
            out.bound = out.bound.max((rb - ra).abs());
            for t in &local.triangles {
                // Exact lateral gap of the triangle inside the slab's height
                // range: the slab is a cylinder, so the part of the triangle
                // between its caps collides iff its XY projection comes
                // within `r` of the axis.
                let Some(poly) = clip_to_slab([t.a, t.b, t.c], h0, h1) else { continue };
                let gap = polygon_distance_xy(&poly) - r;
                if gap < out.min_gap {
                    out.min_gap = gap;
                }
                if gap < -tolerance.distance_eps && out.hit.is_none() {
                    out.hit = Some(ClearanceHit { triangle: map[t.id as usize], part: a.part, band: [h0, h1] });
                }
            }
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Slab geometry
// ---------------------------------------------------------------------------

/// The part of a triangle with `h0 <= z <= h1` as a convex polygon
/// (Sutherland-Hodgman against the two caps); `None` when nothing is left.
pub fn clip_to_slab(tri: [Point3; 3], h0: f64, h1: f64) -> Option<Vec<Point3>> {
    let mut poly: Vec<Point3> = tri.to_vec();
    for (level, keep_above) in [(h0, true), (h1, false)] {
        let inside = |p: &Point3| if keep_above { p.z >= level } else { p.z <= level };
        let mut next = Vec::with_capacity(poly.len() + 1);
        for i in 0..poly.len() {
            let a = poly[i];
            let b = poly[(i + 1) % poly.len()];
            let (ia, ib) = (inside(&a), inside(&b));
            if ia {
                next.push(a);
            }
            if ia != ib {
                let t = (level - a.z) / (b.z - a.z);
                next.push(a.lerp(b, t.clamp(0.0, 1.0)));
            }
        }
        poly = next;
        if poly.is_empty() {
            return None;
        }
    }
    Some(poly)
}

/// Distance in the XY plane from the origin to a convex polygon (zero inside).
pub fn polygon_distance_xy(poly: &[Point3]) -> f64 {
    let n = poly.len();
    if n == 0 {
        return f64::INFINITY;
    }
    if n == 1 {
        return poly[0].distance_xy(Point3::ZERO);
    }
    let mut best = f64::INFINITY;
    let mut pos = 0;
    let mut neg = 0;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        let d = point_segment_distance_xy(a, b);
        best = best.min(d);
        let cross = a.x * b.y - a.y * b.x;
        if cross > 0.0 {
            pos += 1;
        } else if cross < 0.0 {
            neg += 1;
        }
    }
    // Inside a (non-degenerate) convex polygon: every edge turns the same way.
    if n >= 3 && (pos == 0 || neg == 0) && pos + neg > 0 {
        let area: f64 = (0..n).map(|i| {
            let a = poly[i];
            let b = poly[(i + 1) % n];
            a.x * b.y - a.y * b.x
        }).sum();
        if area.abs() > 0.0 {
            return 0.0;
        }
    }
    best
}

/// XY distance from the origin to the segment `ab`.
fn point_segment_distance_xy(a: Point3, b: Point3) -> f64 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (-(a.x * dx + a.y * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    (a.x + t * dx).hypot(a.y + t * dy)
}
