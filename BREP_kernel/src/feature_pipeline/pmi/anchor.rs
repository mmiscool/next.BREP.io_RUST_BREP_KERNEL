//! Balloon leader anchoring: where a Find-number balloon's arrow head lands.
//!
//! The head is DERIVED, never stored, and it lies on a face or an edge of
//! the target occurrence AS DISPLAYED in the owning view. By default it is
//! the point of the occurrence's displayed surface nearest the bubble in
//! screen space ([`nearest_screen_point`]); an optional `anchor` reference pins it to
//! an edge (the point of that edge nearest the bubble) or to a vertex
//! (exactly there). Every candidate is exact B-rep geometry: face feet from
//! the surface projector kept only where they land inside the face's trim,
//! edge feet on the trimmed span, and the topology vertices — never a
//! tessellation — so the head lies on the surface to the kernel's spatial
//! tolerance.
//!
//! "As displayed" is what the user sees, and it is decided per solid by the
//! view ([`HeadScope`]):
//!
//! - **Explode poses.** Every enabled explode row naming a solid poses its
//!   display in turn, `p' = R((p − c) ∘ s) + c + t` per row, so a solid's
//!   display is `S_n(…S_1(p))`. The bubble is mapped back through the
//!   solid's own stages ([`pose_unapply`]), projected in the modeling frame,
//!   and the foot mapped forward ([`pose_apply`]). Two solids of one
//!   occurrence may carry different stages (an explode of one member), so
//!   candidates are compared in the DISPLAYED frame, never the modeling one.
//! - **Section.** A sectioned view clips every display to a half-space
//!   ([`HalfSpace`], kept where `(p − point)·normal ≥ 0`, the clip's own
//!   test). A candidate on the removed side is not displayed geometry and
//!   is never a head; an edge's crossings of the section plane are offered
//!   instead, so a cut part still takes the arrow on its kept rim. When
//!   nothing of a solid is kept there is no head on it.
//! - **Hidden solids** are not in the recipe at all: the resolver lists only
//!   the occurrence's visible members.
//!
//! The recipe ([`BalloonAnchor`]) rides the report's leader geometry so the
//! viewport can re-derive the head from a dragged bubble on the exact-solid
//! clone it holds, with the very same functions, and never run the history.
//!
//! Legacy 3D projection limits: under a non-uniform explode scale the mapped foot is still on the
//! displayed surface but is not guaranteed to be the nearest point of it;
//! under a section the head is the nearest KEPT candidate (a kept foot, a
//! kept vertex, or an edge's crossing of the plane), not the nearest point of
//! the cut surface itself, which may lie on a face's section curve. The screen
//! projector additionally searches planar face/section lines; curved face
//! section curves are not constructed here.

use serde::{Deserialize, Serialize};

use crate::arrangement::Vec2;
use crate::classification::{parameter_point_in_face, PolygonClass};
use crate::feature_pipeline::features::datum::rotate_euler_xyz;
use crate::projection::{project_point_to_curve, project_point_to_surface};
use crate::topology::{EdgeRecord, FaceRecord};
use crate::{BrepSolid, KernelTolerances, NurbsCurve, Vec3};

/// The display pose of an exploded occurrence: `p' = R((p − c) ∘ s) + c + t`
/// with `R` the intrinsic-XYZ Euler rotation the explode annotation carries.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PmiPose {
    pub center: [f64; 3],
    pub translate: [f64; 3],
    #[serde(rename = "rotateDeg")]
    pub rotate_deg: [f64; 3],
    pub scale: [f64; 3],
}

impl PmiPose {
    fn radians(&self) -> [f64; 3] {
        self.rotate_deg.map(f64::to_radians)
    }

    /// Modeling frame → displayed frame.
    pub fn apply(&self, point: [f64; 3]) -> [f64; 3] {
        let c = self.center;
        let local = Vec3::new(
            (point[0] - c[0]) * self.scale[0],
            (point[1] - c[1]) * self.scale[1],
            (point[2] - c[2]) * self.scale[2],
        );
        let rotated = rotate_euler_xyz(local, self.radians());
        [
            rotated.x + c[0] + self.translate[0],
            rotated.y + c[1] + self.translate[1],
            rotated.z + c[2] + self.translate[2],
        ]
    }

    /// Displayed frame → modeling frame (the exact inverse of [`Self::apply`]:
    /// the rotation is orthonormal, so its inverse is its transpose).
    pub fn unapply(&self, point: [f64; 3]) -> [f64; 3] {
        let c = self.center;
        let moved = Vec3::new(
            point[0] - c[0] - self.translate[0],
            point[1] - c[1] - self.translate[1],
            point[2] - c[2] - self.translate[2],
        );
        let e = self.radians();
        // The rows of R are the images of the unit axes under Rᵀ.
        let rx = rotate_euler_xyz(Vec3::new(1.0, 0.0, 0.0), e);
        let ry = rotate_euler_xyz(Vec3::new(0.0, 1.0, 0.0), e);
        let rz = rotate_euler_xyz(Vec3::new(0.0, 0.0, 1.0), e);
        let local = Vec3::new(rx.dot(moved), ry.dot(moved), rz.dot(moved));
        let unscale = |v: f64, s: f64| if s.abs() > 1e-300 { v / s } else { v };
        [
            unscale(local.x, self.scale[0]) + c[0],
            unscale(local.y, self.scale[1]) + c[1],
            unscale(local.z, self.scale[2]) + c[2],
        ]
    }

    /// Whether the pose keeps lengths (unit scale): then the nearest point in
    /// the modeling frame is the nearest point in the displayed frame.
    pub fn is_rigid(&self) -> bool {
        self.scale.iter().all(|s| (s - 1.0).abs() < 1e-12)
    }
}

/// Modeling frame → displayed frame through `stages`, in the order the
/// display applies them.
pub fn pose_apply(stages: &[PmiPose], point: [f64; 3]) -> [f64; 3] {
    stages.iter().fold(point, |p, stage| stage.apply(p))
}

/// Displayed frame → modeling frame: the stages undone last-first.
pub fn pose_unapply(stages: &[PmiPose], point: [f64; 3]) -> [f64; 3] {
    stages.iter().rev().fold(point, |p, stage| stage.unapply(p))
}

/// One solid's display poses, in application order.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidPoses {
    pub solid: String,
    pub stages: Vec<PmiPose>,
}

/// A section half-space in the DISPLAYED frame: geometry is kept where
/// `(p − point)·normal ≥ 0` (the display clip's own test). A section of
/// several planes lists the others in `also`: it keeps where ANY plane keeps
/// (material is removed only where every plane removes it).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HalfSpace {
    pub point: [f64; 3],
    pub normal: [f64; 3],
    /// The section's further planes, `(point, kept-side normal)`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub also: Vec<([f64; 3], [f64; 3])>,
}

impl HalfSpace {
    /// The section of `planes` (`(point, kept-side normal)`, the first being
    /// the main one); `None` without any.
    pub fn of(planes: &[([f64; 3], [f64; 3])]) -> Option<Self> {
        let (&(point, normal), rest) = planes.split_first()?;
        Some(Self { point, normal, also: rest.to_vec() })
    }

    /// The signed distance of `p` from the kept region's boundary, positive
    /// inside: from the plane, or for several planes the largest of their
    /// distances (≥ 0 exactly where one of them keeps `p`).
    pub fn signed(&self, p: [f64; 3]) -> f64 {
        let from = |point: [f64; 3], normal: [f64; 3]| (0..3).map(|i| (p[i] - point[i]) * normal[i]).sum::<f64>();
        self.also.iter().fold(from(self.point, self.normal), |best, &(point, normal)| best.max(from(point, normal)))
    }

    /// Whether `p` is displayed (kept), to `tolerance` on the plane itself.
    pub fn keeps(&self, p: [f64; 3], tolerance: f64) -> bool {
        self.signed(p) >= -tolerance
    }
}

/// How a balloon's head was derived — what the viewport re-derives it with.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "camelCase")]
pub enum BalloonAnchorMode {
    /// The nearest point on the occurrence's surface (the default).
    Surface,
    /// The nearest point on one edge, named by its owning solid and edge id.
    Edge {
        solid: String,
        #[serde(rename = "edgeId")]
        edge_id: u64,
    },
    /// A picked vertex: the head is fixed there.
    Point,
}

/// The head derivation recipe carried on a balloon's leader geometry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BalloonAnchor {
    /// Camera used to minimize automatic leaders in screen space.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camera: Option<super::PmiCamera>,
    /// The occurrence's VISIBLE solid names (scene names) the default head
    /// projects onto.
    pub solids: Vec<String>,
    /// Flattened: `"mode": "surface" | "edge" (+ `solid`, `edgeId`) | "point"`.
    #[serde(flatten)]
    pub mode: BalloonAnchorMode,
    /// The display poses per solid (an un-posed solid is absent).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub poses: Vec<SolidPoses>,
    /// The view's section half-space, when the view is sectioned.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub section: Option<HalfSpace>,
}

impl BalloonAnchor {
    /// `solid`'s display stages (empty when it is not posed).
    pub fn stages(&self, solid: &str) -> &[PmiPose] {
        self.poses
            .iter()
            .find(|poses| poses.solid == solid)
            .map_or(&[], |poses| poses.stages.as_slice())
    }

    /// The scope one solid's head is searched in.
    pub fn scope(&self, solid: &str) -> HeadScope<'_> {
        HeadScope {
            stages: self.stages(solid),
            section: self.section.as_ref(),
        }
    }
}

/// Where one solid's head may land: its display stages and the view's
/// section. [`HeadScope::PLAIN`] is the un-posed, unsectioned solid.
#[derive(Debug, Clone, Copy)]
pub struct HeadScope<'a> {
    pub stages: &'a [PmiPose],
    pub section: Option<&'a HalfSpace>,
}

impl HeadScope<'_> {
    pub const PLAIN: HeadScope<'static> = HeadScope {
        stages: &[],
        section: None,
    };

    fn display(&self, modeling: Vec3) -> [f64; 3] {
        pose_apply(self.stages, [modeling.x, modeling.y, modeling.z])
    }

    fn modeling(&self, display: [f64; 3]) -> Vec3 {
        let p = pose_unapply(self.stages, display);
        Vec3::new(p[0], p[1], p[2])
    }

    fn keeps(&self, display: [f64; 3], tolerance: f64) -> bool {
        self.section.is_none_or(|section| section.keeps(display, tolerance))
    }

    fn is_rigid(&self) -> bool {
        self.stages.iter().all(PmiPose::is_rigid)
    }
}

/// The best candidate so far: its displayed position and squared distance
/// to the bubble, in the displayed frame.
struct Best {
    display: [f64; 3],
    d2: f64,
}

fn offer(best: &mut Option<Best>, scope: &HeadScope<'_>, bubble: [f64; 3], tolerance: f64, modeling: Vec3) {
    let display = scope.display(modeling);
    if !scope.keeps(display, tolerance) {
        return;
    }
    let d2 = (0..3).map(|i| (display[i] - bubble[i]).powi(2)).sum::<f64>();
    if best.as_ref().is_none_or(|b| d2 < b.d2) {
        *best = Some(Best { display, d2 });
    }
}

/// The point of `solid`'s DISPLAYED surface nearest `bubble` (both in the
/// displayed frame): the nearest of its face feet that land inside their
/// trims, its edge spans and its vertices, as posed by `scope` and kept by
/// its section; under a section an edge's crossings of the plane are offered
/// too. `Ok(None)` when no candidate is displayed (the solid is sectioned
/// away). A face whose projector refuses is skipped — the edges and vertices
/// are always on the surface — so one bad face never loses the head.
pub fn nearest_displayed_point(solid: &BrepSolid, scope: HeadScope<'_>, bubble: [f64; 3]) -> Result<Option<[f64; 3]>, String> {
    let spatial = KernelTolerances::for_solid(solid, 1e-7).spatial();
    let query = scope.modeling(bubble);
    let mut best: Option<Best> = None;
    for vertex in &solid.vertices {
        offer(&mut best, &scope, bubble, spatial, vertex.point);
    }
    for edge in &solid.edges {
        if edge.degenerate {
            continue;
        }
        offer(&mut best, &scope, bubble, spatial, nearest_point_on_edge(edge, query)?);
        if let Some(section) = scope.section {
            for crossing in edge_plane_crossings(edge, &scope, section)? {
                offer(&mut best, &scope, bubble, spatial, crossing);
            }
        }
    }
    let rigid = scope.is_rigid();
    for shell in &solid.shells {
        for face in &shell.faces {
            // A face whose control hull cannot come closer than the running
            // best is skipped without projecting (the hull bounds the
            // surface; a rigid pose keeps distances, so the modeling-frame
            // bound applies to the displayed one).
            if rigid {
                if let (Some(best), Some(hull_d2)) = (best.as_ref(), hull_distance_squared(face, query)?) {
                    if hull_d2 > best.d2 {
                        continue;
                    }
                }
            }
            let Ok(projection) = project_point_to_surface(&face.surface, query) else {
                continue;
            };
            if rigid && best.as_ref().is_some_and(|b| projection.distance * projection.distance >= b.d2) {
                continue;
            }
            let uv_band = match face.surface.derivatives(projection.u, projection.v, 1) {
                Ok(derivatives) => crate::tolerance::surface_uv_tolerance(
                    spatial,
                    derivatives[1][0].length(),
                    derivatives[0][1].length(),
                ),
                Err(_) => spatial,
            };
            let Ok(class) = parameter_point_in_face(
                face,
                Vec2 {
                    x: projection.u,
                    y: projection.v,
                },
                uv_band,
            ) else {
                continue;
            };
            if matches!(class, PolygonClass::Inside | PolygonClass::Boundary) {
                offer(&mut best, &scope, bubble, spatial, projection.point);
            }
        }
    }
    Ok(best.map(|b| b.display))
}

/// Camera-plane coordinates. Perspective projection is a homogeneous transform,
/// so projecting NURBS controls preserves the original curve/surface parameters.
fn screen_control(camera: &super::PmiCamera, scope: HeadScope<'_>, p: crate::Vec4) -> crate::Vec4 {
    let world = scope.display(Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w));
    let d = camera.view_direction();
    let forward = Vec3::new(d[0], d[1], d[2]);
    let up = Vec3::new(camera.up[0], camera.up[1], camera.up[2]);
    let right = forward.cross(up);
    let right = right.scale(1.0 / right.length().max(1e-15));
    let up = right.cross(forward);
    let v = Vec3::new(world[0] - camera.eye[0], world[1] - camera.eye[1], world[2] - camera.eye[2]);
    let weight = match camera.projection {
        super::PmiProjection::Orthographic { .. } => p.w,
        super::PmiProjection::Perspective { .. } => p.w * v.dot(forward),
    };
    crate::Vec4 { x: v.dot(right) * p.w, y: v.dot(up) * p.w, z: 0.0, w: weight }
}

pub(super) fn head_distance(camera: Option<&super::PmiCamera>, a: [f64; 3], b: [f64; 3]) -> f64 {
    let project = |p: [f64; 3]| {
        if let Some(camera) = camera {
            let q = screen_control(camera, HeadScope::PLAIN, crate::Vec4::from_point(Vec3::new(p[0], p[1], p[2]), 1.0));
            [q.x / q.w, q.y / q.w, 0.0]
        } else { p }
    };
    let a = project(a);
    let b = project(b);
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum()
}

/// A planar face's section line may contain the nearest kept point between
/// topology edges. Intersect the section with the affine parameter rectangle,
/// project that segment, and qualify its foot against the original face trim.
/// Trim-hole endpoints are already covered by the topology edge crossings.
fn planar_section_feet(face: &FaceRecord, scope: HeadScope<'_>, camera: &super::PmiCamera, query: Vec3, spatial: f64) -> Result<Vec<Vec3>, String> {
    let Some(section) = scope.section else { return Ok(Vec::new()); };
    let Some(crate::AnalyticSurface::Plane { .. }) = face.surface.analytic() else { return Ok(Vec::new()); };
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    let corners = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
    let mut feet = Vec::new();
    for (point, normal) in std::iter::once((section.point, section.normal)).chain(section.also.iter().copied()) {
        let mut crossings: Vec<[f64; 2]> = Vec::new();
        for k in 0..4 {
            let a = corners[k];
            let b = corners[(k + 1) % 4];
            let signed = |uv: [f64; 2]| -> Result<f64, String> {
                let p = scope.display(face.surface.evaluate(uv[0], uv[1])?);
                Ok((0..3).map(|i| (p[i] - point[i]) * normal[i]).sum())
            };
            let da = signed(a)?;
            let db = signed(b)?;
            if da.abs() <= spatial { crossings.push(a); }
            if da * db < 0.0 {
                let t = da / (da - db);
                crossings.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
            }
        }
        // Pick the farthest pair (a corner may be encountered twice).
        let pair = crossings.iter().flat_map(|a| crossings.iter().map(move |b| (*a, *b)))
            .max_by(|(a, b), (c, d)| {
                let length = |a: &[f64; 2], b: &[f64; 2]| (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2);
                length(a, b).total_cmp(&length(c, d))
            });
        let Some((a, b)) = pair else { continue; };
        if a == b { continue; }
        let controls = [a, b].into_iter().map(|uv| {
            face.surface.evaluate(uv[0], uv[1]).map(|p| screen_control(camera, scope, crate::Vec4::from_point(p, 1.0)))
        }).collect::<Result<Vec<_>, _>>()?;
        let Ok(line) = NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], controls) else { continue; };
        let Ok(foot) = project_point_to_curve(&line, query) else { continue; };
        let uv = Vec2 { x: a[0] + foot.u * (b[0] - a[0]), y: a[1] + foot.u * (b[1] - a[1]) };
        let derivatives = face.surface.derivatives(uv.x, uv.y, 1)?;
        let band = crate::tolerance::surface_uv_tolerance(spatial, derivatives[1][0].length(), derivatives[0][1].length());
        if matches!(parameter_point_in_face(face, uv, band), Ok(PolygonClass::Inside | PolygonClass::Boundary)) {
            feet.push(face.surface.evaluate(uv.x, uv.y)?);
        }
    }
    Ok(feet)
}

/// Minimize the projected distance, then evaluate the original B-rep at the
/// winning parameters. Depth never biases the screen-space minimization.
/// Explicit edge/vertex anchors retain their existing semantics.
pub fn nearest_screen_point(
    solid: &BrepSolid, scope: HeadScope<'_>, bubble: [f64; 3], camera: Option<&super::PmiCamera>,
) -> Result<Option<[f64; 3]>, String> {
    let Some(camera) = camera else { return nearest_displayed_point(solid, scope, bubble); };
    let q = screen_control(camera, HeadScope::PLAIN, crate::Vec4::from_point(Vec3::new(bubble[0], bubble[1], bubble[2]), 1.0));
    let query = Vec3::new(q.x / q.w, q.y / q.w, 0.0);
    let spatial = KernelTolerances::for_solid(solid, 1e-7).spatial();
    let mut best: Option<Best> = None;
    let direction = camera.view_direction();
    let depth = |p: [f64; 3]| (0..3).map(|i| (p[i] - camera.eye[i]) * direction[i]).sum::<f64>();
    let mut offer = |point: Vec3| {
        let display = scope.display(point);
        if !scope.keeps(display, spatial) { return; }
        if matches!(camera.projection, super::PmiProjection::Perspective { .. }) && depth(display) <= 0.0 { return; }
        let d2 = head_distance(Some(camera), display, bubble);
        if d2.is_finite() && best.as_ref().is_none_or(|b| d2 < b.d2 - 1e-12 || ((d2 - b.d2).abs() <= 1e-12 && depth(display) < depth(b.display))) {
            best = Some(Best { display, d2 });
        }
    };
    for vertex in &solid.vertices { offer(vertex.point); }
    for edge in &solid.edges {
        for t in [edge.t0, edge.t1] { offer(edge.curve.evaluate(t)?); }
        if edge.degenerate { continue; }
        let span = span_curve(&edge.curve, edge.t0.min(edge.t1), edge.t0.max(edge.t1))?;
        let projected = NurbsCurve::new(span.degree, span.knots.clone(), span.control_points.iter().map(|p| screen_control(camera, scope, *p)).collect());
        if let Ok(curve) = projected {
            if let Ok(foot) = project_point_to_curve(&curve, query) { offer(edge.curve.evaluate(foot.u)?); }
        }
        if let Some(section) = scope.section {
            for p in edge_plane_crossings(edge, &scope, section)? { offer(p); }
        }
    }
    for face in solid.shells.iter().flat_map(|s| &s.faces) {
        for foot in planar_section_feet(face, scope, camera, query, spatial)? { offer(foot); }
        let s = &face.surface;
        // Construct anew: cached analytic carriers and projection grids belong
        // to the unprojected surface and must not survive this transform.
        let projected = crate::NurbsSurface::new(s.degree_u, s.degree_v, s.knots_u.clone(), s.knots_v.clone(),
            s.control_points.iter().map(|row| row.iter().map(|p| screen_control(camera, scope, *p)).collect()).collect());
        let Ok(surface) = projected else { continue; };
        let Ok(foot) = project_point_to_surface(&surface, query) else { continue; };
        let derivatives = s.derivatives(foot.u, foot.v, 1)?;
        let band = crate::tolerance::surface_uv_tolerance(spatial, derivatives[1][0].length(), derivatives[0][1].length());
        if matches!(parameter_point_in_face(face, Vec2 { x: foot.u, y: foot.v }, band), Ok(PolygonClass::Inside | PolygonClass::Boundary)) {
            offer(s.evaluate(foot.u, foot.v)?);
        }
    }
    Ok(best.map(|b| b.display))
}

/// The point of `solid`'s surface nearest `point`, un-posed and unsectioned
/// (the modeling-frame form of [`nearest_displayed_point`]).
pub fn nearest_point_on_solid(solid: &BrepSolid, point: Vec3) -> Result<Vec3, String> {
    nearest_displayed_point(solid, HeadScope::PLAIN, [point.x, point.y, point.z])?
        .map(|p| Vec3::new(p[0], p[1], p[2]))
        .ok_or_else(|| "solid has no geometry".to_string())
}

/// The squared distance from `point` to the AABB of `face`'s control hull
/// (`None` for a face without control points).
fn hull_distance_squared(face: &FaceRecord, point: Vec3) -> Result<Option<f64>, String> {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut any = false;
    for row in &face.surface.control_points {
        for control in row {
            let p = control.point()?;
            low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
            high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
            any = true;
        }
    }
    if !any {
        return Ok(None);
    }
    let axis = |v: f64, lo: f64, hi: f64| if v < lo { lo - v } else if v > hi { v - hi } else { 0.0 };
    let dx = axis(point.x, low.x, high.x);
    let dy = axis(point.y, low.y, high.y);
    let dz = axis(point.z, low.z, high.z);
    Ok(Some(dx * dx + dy * dy + dz * dz))
}

/// The point of `edge`'s trimmed span `[t0, t1]` nearest `point`. The carrier
/// is cut down to the span (a partial arc on a full circle must not answer
/// with the far side), then projected with the curve projector.
pub fn nearest_point_on_edge(edge: &EdgeRecord, point: Vec3) -> Result<Vec3, String> {
    if edge.degenerate {
        return edge.curve.evaluate(edge.t0);
    }
    let span = span_curve(&edge.curve, edge.t0.min(edge.t1), edge.t0.max(edge.t1))?;
    let projection = project_point_to_curve(&span, point)?;
    let mut best = (projection.distance, projection.point);
    // The projector's answer is a local minimum refined from the nearest
    // sample; the span's ends are exact candidates it may not have seeded.
    for parameter in [edge.t0, edge.t1] {
        let end = edge.curve.evaluate(parameter)?;
        let distance = end.sub(point).length();
        if distance < best.0 {
            best = (distance, end);
        }
    }
    Ok(best.1)
}

/// The point of `edge`'s DISPLAYED span nearest `bubble` within `scope`:
/// the span's foot when it is kept, else the kept crossing of the section
/// plane nearest the bubble; `Ok(None)` when the whole span is sectioned
/// away.
pub fn nearest_displayed_point_on_edge(edge: &EdgeRecord, scope: HeadScope<'_>, bubble: [f64; 3]) -> Result<Option<[f64; 3]>, String> {
    let tolerance = 1e-7;
    let mut best: Option<Best> = None;
    offer(&mut best, &scope, bubble, tolerance, nearest_point_on_edge(edge, scope.modeling(bubble))?);
    if let Some(section) = scope.section {
        if best.is_none() && !edge.degenerate {
            for crossing in edge_plane_crossings(edge, &scope, section)? {
                offer(&mut best, &scope, bubble, tolerance, crossing);
            }
        }
    }
    Ok(best.map(|b| b.display))
}

/// Where `edge`'s span crosses the section's boundary (modeling points): the
/// signed distance of the DISPLAYED curve is sampled along the span and each
/// sign change bisected to the parameter.
fn edge_plane_crossings(edge: &EdgeRecord, scope: &HeadScope<'_>, section: &HalfSpace) -> Result<Vec<Vec3>, String> {
    const SAMPLES: usize = 64;
    let (t0, t1) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    if !(t1 > t0) {
        return Ok(Vec::new());
    }
    let signed_at = |t: f64| -> Result<f64, String> { Ok(section.signed(scope.display(edge.curve.evaluate(t)?))) };
    let mut out = Vec::new();
    let mut previous = (t0, signed_at(t0)?);
    for k in 1..=SAMPLES {
        let t = t0 + (t1 - t0) * k as f64 / SAMPLES as f64;
        let s = signed_at(t)?;
        if previous.1 == 0.0 {
            out.push(edge.curve.evaluate(previous.0)?);
        } else if (previous.1 < 0.0) != (s < 0.0) && s != 0.0 {
            let (mut a, mut sa, mut b) = (previous.0, previous.1, t);
            for _ in 0..60 {
                let m = (a + b) * 0.5;
                let sm = signed_at(m)?;
                if (sm < 0.0) == (sa < 0.0) {
                    a = m;
                    sa = sm;
                } else {
                    b = m;
                }
            }
            out.push(edge.curve.evaluate((a + b) * 0.5)?);
        }
        previous = (t, s);
    }
    if previous.1 == 0.0 {
        out.push(edge.curve.evaluate(previous.0)?);
    }
    Ok(out)
}

/// `curve` restricted to `[t0, t1]` (the curve itself when the span is its
/// whole domain). `NurbsCurve::split` refuses a cut at a domain end, so an end
/// that coincides with the span is simply not cut.
fn span_curve(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<NurbsCurve, String> {
    let [start, end] = curve.domain()?;
    let eps = (end - start).abs() * 1e-9;
    let mut span = curve.clone();
    if t0 > start + eps && t0 < end - eps {
        span = span.split(t0)?.1;
    }
    let [_, span_end] = span.domain()?;
    if t1 > start + eps && t1 < span_end - eps {
        span = span.split(t1)?.0;
    }
    Ok(span)
}

/// The head for `bubble` (displayed frame) through `anchor`'s recipe, from
/// the solids `lookup` serves by scene name. `Ok(None)` for a fixed point
/// (the head never moves) — and for a solid `lookup` cannot serve, in which
/// case `missing` names it, so the caller can fetch it and try again. An
/// error when the recipe has no displayed geometry left for the head (the
/// anchor edge, or every solid, sectioned away).
pub fn balloon_head<'a>(
    anchor: &BalloonAnchor,
    bubble: [f64; 3],
    mut lookup: impl FnMut(&str) -> Option<&'a BrepSolid>,
    missing: &mut Vec<String>,
) -> Result<Option<[f64; 3]>, String> {
    match &anchor.mode {
        BalloonAnchorMode::Point => Ok(None),
        BalloonAnchorMode::Edge { solid, edge_id } => {
            let Some(brep) = lookup(solid) else {
                missing.push(solid.clone());
                return Ok(None);
            };
            let edge = brep
                .edges
                .iter()
                .find(|edge| edge.id == *edge_id)
                .ok_or_else(|| format!("anchor edge {edge_id} is not on '{solid}'"))?;
            nearest_displayed_point_on_edge(edge, anchor.scope(solid), bubble)?
                .map(Some)
                .ok_or_else(|| format!("anchor edge {edge_id} of '{solid}' is sectioned away in this view"))
        }
        BalloonAnchorMode::Surface => {
            let mut best: Option<Best> = None;
            for name in &anchor.solids {
                let Some(brep) = lookup(name) else {
                    missing.push(name.clone());
                    continue;
                };
                if let Some(display) = nearest_screen_point(brep, anchor.scope(name), bubble, anchor.camera.as_ref())? {
                    let d2 = head_distance(anchor.camera.as_ref(), display, bubble);
                    if best.as_ref().is_none_or(|b| d2 < b.d2) {
                        best = Some(Best { display, d2 });
                    }
                }
            }
            if !missing.is_empty() {
                return Ok(None);
            }
            best.map(|b| Some(b.display))
                .ok_or_else(|| "the balloon's occurrence has no displayed geometry in this view".to_string())
        }
    }
}
