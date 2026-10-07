//! Assembly selection resolution — a selection reference resolved to an
//! ANALYTIC frame read from the exact BREP surfaces/curves, never from
//! tessellation (the retired app's polyline-PCA approximation is the wart
//! this module kills):
//!
//! - planar face → plane (origin + OUTWARD unit normal, plane-z convention)
//! - cylindrical/conical face → axis (origin + direction), radius when the
//!   radius is constant (a genuine cylinder)
//! - spherical face → center + radius
//! - circular/arc edge → center + axis + radius
//! - straight edge → the INFINITE carrier line
//! - vertex → point
//! - whole component → representative point (aggregate bbox center)
//!
//! Anything without an analytic frame is a typed [`ResolveError`] mapping onto
//! the constraint status vocabulary (`unsupported-selection` /
//! `invalid-selection`), never a panic.
//!
//! # Coordinate space — read this before marshaling mates
//!
//! Every resolver reads the geometry EXACTLY as stored on the given
//! [`BrepSolid`], so the resolved frame is in the SOLID'S OWN space. A
//! component instance whose resident solids are world-posed (the ACOMP feature
//! bakes its placement into the geometry) must pass the component's
//! world→local INVERSE transform to [`SelectionGeometry::transformed`] to
//! obtain the COMPONENT-LOCAL frame the assembly solver's mate inputs require
//! ([`MateKind`](crate::MateKind): `*_a`/`*_b` geometry is local to the owning
//! body, which the solver poses).
//!
//! # Lane seam — component lookup
//!
//! [`split_component_namespace`] only PARSES the `ACOMP…:` prefix chain off a
//! namespaced topology name. Mapping that chain to the owning component's
//! resident solids and its world→local inverse transform is the scene
//! component registry's job (built in parallel); the resolvers here
//! deliberately take `(solid, entity)` plus a transform argument instead of a
//! registry. Vertices carry no kernel names (the renderer's `VertexRef`
//! convention is owning solid + position), so vertex selections arrive as a
//! position and snap to the nearest topology vertex.

use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::{AffineTransform, AnalyticSurface, MateAxis, MatePlane, NurbsCurve, Vec3};
use serde::Serialize;

/// Scale-relative acceptance for the straight/circular edge verification —
/// the analytic-surface recognition tolerance, for the same reason: exact
/// kernel-built geometry verifies at machine precision, anything else fails
/// by orders of magnitude.
const RESOLVE_TOLERANCE: f64 = 1e-9;
/// Scale-relative snap distance for vertex-by-position resolution (positions
/// round-trip through the app as f64 but may be re-serialized).
const VERTEX_SNAP_TOLERANCE: f64 = 1e-6;
/// Circle acceptance samples across the edge's trimmed span.
const CIRCLE_VERIFY_SAMPLES: usize = 16;

// ---------------------------------------------------------------------------
// Result / error types
// ---------------------------------------------------------------------------

/// A resolved analytic selection frame, in the coordinate space of the solid
/// it was resolved from (module docs: pass the component's world→local inverse
/// to [`Self::transformed`] for the solver's component-local mate inputs).
///
/// Cross-lane invariant: [`SelectionGeometry::Axis`]`::radius` is `Some` IFF
/// the face is a genuine constant-radius cylinder — the gate for the
/// `tangent_cylinder_plane` mate. Cones, tori, and general revolution faces
/// resolve with `radius: None`.
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SelectionGeometry {
    /// Planar face: origin on the plane (the centre of the face boundary's
    /// bounding box in the face's own in-plane basis — a fixed point of the
    /// body, see `planar_face`) + OUTWARD unit normal (face sense respected,
    /// per the one plane-z direction convention).
    Plane { origin: Vec3, normal: Vec3 },
    /// Axis-bearing face (cylinder / cone / torus / general revolution):
    /// a point on the axis + unit direction.
    Axis {
        origin: Vec3,
        direction: Vec3,
        radius: Option<f64>,
    },
    /// Spherical face.
    Sphere { center: Vec3, radius: f64 },
    /// Circular or arc edge; `axis` follows the right-hand rule along the
    /// edge's increasing-parameter direction.
    Circle { center: Vec3, axis: Vec3, radius: f64 },
    /// Straight edge: the INFINITE carrier line (origin = chord midpoint,
    /// unit direction start→end). Distance mates measure against the carrier,
    /// never the clamped finite segment.
    Line { origin: Vec3, direction: Vec3 },
    /// Vertex (or whole-component representative point).
    Point { position: Vec3 },
}

/// Typed resolution failure. [`Self::status`] maps each variant onto the
/// constraint status vocabulary so the constraint lifecycle reports it
/// without string-matching messages.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub enum ResolveError {
    /// No such entity on the solid (or no vertex within snap distance).
    NotFound { name: String },
    /// The entity exists but carries no analytic frame (freeform surface,
    /// spline edge, degenerate edge).
    Unsupported { name: String, detail: String },
    /// The entity's geometry failed to evaluate.
    Geometry { name: String, detail: String },
}

impl ResolveError {
    /// The constraint-status word this failure maps to.
    pub fn status(&self) -> &'static str {
        match self {
            ResolveError::NotFound { .. } | ResolveError::Geometry { .. } => "invalid-selection",
            ResolveError::Unsupported { .. } => "unsupported-selection",
        }
    }
}

impl std::fmt::Display for ResolveError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ResolveError::NotFound { name } => write!(formatter, "selection '{name}' not found"),
            ResolveError::Unsupported { name, detail } => {
                write!(formatter, "selection '{name}' has no analytic frame: {detail}")
            }
            ResolveError::Geometry { name, detail } => {
                write!(formatter, "selection '{name}' failed to resolve: {detail}")
            }
        }
    }
}

impl SelectionGeometry {
    /// The solver's plane input, for planar-face selections.
    pub fn mate_plane(&self) -> Option<MatePlane> {
        match *self {
            SelectionGeometry::Plane { origin, normal } => Some(MatePlane {
                origin: triple(origin),
                normal: triple(normal),
            }),
            _ => None,
        }
    }

    /// The solver's axis input, for every axis-bearing selection: an axis
    /// face, a circular edge (center + circle axis), or a straight edge's
    /// carrier line.
    pub fn mate_axis(&self) -> Option<MateAxis> {
        let (origin, direction) = match *self {
            SelectionGeometry::Axis {
                origin, direction, ..
            }
            | SelectionGeometry::Line { origin, direction } => (origin, direction),
            SelectionGeometry::Circle { center, axis, .. } => (center, axis),
            _ => return None,
        };
        Some(MateAxis {
            origin: triple(origin),
            direction: triple(direction),
        })
    }

    /// The per-kind anchor point: plane / axis / line origin, sphere / circle
    /// center, the point itself.
    pub fn representative_point(&self) -> Vec3 {
        match *self {
            SelectionGeometry::Plane { origin, .. }
            | SelectionGeometry::Axis { origin, .. }
            | SelectionGeometry::Line { origin, .. } => origin,
            SelectionGeometry::Sphere { center, .. }
            | SelectionGeometry::Circle { center, .. } => center,
            SelectionGeometry::Point { position } => position,
        }
    }

    /// Map the frame through `transform` — pass the owning component's
    /// world→local inverse to express a world-resolved frame in the
    /// component-local coordinates the solver's mate inputs require. Rigid /
    /// uniform-similarity matrices only (component poses are rigid by spec):
    /// radii scale by the uniform factor; a shearing or non-uniformly scaling
    /// matrix is refused (it has no analytic image for circles/cylinders).
    pub fn transformed(&self, transform: &AffineTransform) -> Result<Self, ResolveError> {
        let scale = uniform_scale(transform).ok_or_else(|| ResolveError::Unsupported {
            name: "transform".into(),
            detail: "selection frames transform only by rigid/uniform-scale matrices".into(),
        })?;
        let direction = |vector: Vec3| {
            linear(transform, vector)
                .normalized()
                .map_err(|error| ResolveError::Geometry {
                    name: "transform".into(),
                    detail: error,
                })
        };
        Ok(match *self {
            SelectionGeometry::Plane { origin, normal } => SelectionGeometry::Plane {
                origin: transform.point(origin),
                normal: direction(normal)?,
            },
            SelectionGeometry::Axis {
                origin,
                direction: axis,
                radius,
            } => SelectionGeometry::Axis {
                origin: transform.point(origin),
                direction: direction(axis)?,
                radius: radius.map(|radius| radius * scale),
            },
            SelectionGeometry::Sphere { center, radius } => SelectionGeometry::Sphere {
                center: transform.point(center),
                radius: radius * scale,
            },
            SelectionGeometry::Circle {
                center,
                axis,
                radius,
            } => SelectionGeometry::Circle {
                center: transform.point(center),
                axis: direction(axis)?,
                radius: radius * scale,
            },
            SelectionGeometry::Line {
                origin,
                direction: axis,
            } => SelectionGeometry::Line {
                origin: transform.point(origin),
                direction: direction(axis)?,
            },
            SelectionGeometry::Point { position } => SelectionGeometry::Point {
                position: transform.point(position),
            },
        })
    }
}

fn triple(vector: Vec3) -> [f64; 3] {
    [vector.x, vector.y, vector.z]
}

/// The linear (rotation/scale) part of the transform applied to a vector.
pub(crate) fn linear(transform: &AffineTransform, vector: Vec3) -> Vec3 {
    let m = transform.elements;
    Vec3::new(
        m[0] * vector.x + m[1] * vector.y + m[2] * vector.z,
        m[4] * vector.x + m[5] * vector.y + m[6] * vector.z,
        m[8] * vector.x + m[9] * vector.y + m[10] * vector.z,
    )
}

/// The uniform scale factor of a rigid/similarity matrix: columns of equal
/// length AND pairwise orthogonal (equal lengths alone admit shear).
fn uniform_scale(transform: &AffineTransform) -> Option<f64> {
    let m = transform.elements;
    let columns = [
        Vec3::new(m[0], m[4], m[8]),
        Vec3::new(m[1], m[5], m[9]),
        Vec3::new(m[2], m[6], m[10]),
    ];
    let lengths = [
        columns[0].length(),
        columns[1].length(),
        columns[2].length(),
    ];
    let scale = (lengths[0] + lengths[1] + lengths[2]) / 3.0;
    if !(scale > 0.0) {
        return None;
    }
    if lengths
        .iter()
        .any(|length| (length - scale).abs() > 1e-9 * scale)
    {
        return None;
    }
    let orthogonality = 1e-9 * scale * scale;
    if columns[0].dot(columns[1]).abs() > orthogonality
        || columns[1].dot(columns[2]).abs() > orthogonality
        || columns[0].dot(columns[2]).abs() > orthogonality
    {
        return None;
    }
    Some(scale)
}

// ---------------------------------------------------------------------------
// Namespace parsing
// ---------------------------------------------------------------------------

/// Split a possibly-namespaced topology name into its `ACOMP…:` component-id
/// chain (outermost first) and the remaining component-LOCAL name. A segment
/// is peeled only when it matches `ACOMP<digits>` AND is followed by `:`, so a
/// bare whole-component reference stays in the LOCAL position — callers detect
/// it with [`is_component_reference`]:
///
/// - `"ACOMP2:Extrude1|Extrude1_top[0]"` → `(["ACOMP2"], "Extrude1|…")`
/// - `"ACOMP3:ACOMP1:S1:G20"` → `(["ACOMP3", "ACOMP1"], "S1:G20")`
/// - `"ACOMP2"` → `([], "ACOMP2")` (whole-component selection)
/// - `"ACOMP3:ACOMP1"` → `(["ACOMP3"], "ACOMP1")` (nested whole-component)
/// - `"S1:G20"` → `([], "S1:G20")` (sketch-child names never namespace)
pub fn split_component_namespace(name: &str) -> (Vec<&str>, &str) {
    let mut chain = Vec::new();
    let mut local = name;
    while let Some((head, tail)) = local.split_once(':') {
        if !is_component_reference(head) {
            break;
        }
        chain.push(head);
        local = tail;
    }
    (chain, local)
}

/// `ACOMP<digits>` — an ACOMP feature id, i.e. a whole-component selection or
/// one namespace segment of a nested chain.
pub fn is_component_reference(name: &str) -> bool {
    name.strip_prefix("ACOMP")
        .map(|digits| !digits.is_empty() && digits.bytes().all(|byte| byte.is_ascii_digit()))
        .unwrap_or(false)
}

// ---------------------------------------------------------------------------
// Entity resolution
// ---------------------------------------------------------------------------

/// Resolve a component-LOCAL topology name (already namespace-stripped)
/// against one solid: faces first, then edges. Names never collide across the
/// two kinds under the deterministic-naming scheme; the order only settles
/// pathological ties.
pub fn resolve_named_selection(
    solid: &BrepSolid,
    name: &str,
) -> Result<SelectionGeometry, ResolveError> {
    if let Some(record) = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.name.as_deref() == Some(name))
    {
        return resolve_face_record(solid, record);
    }
    if let Some(record) = solid
        .edges
        .iter()
        .find(|edge| edge.name.as_deref() == Some(name))
    {
        return resolve_edge_record(record);
    }
    Err(ResolveError::NotFound { name: name.into() })
}

/// Resolve a face by topology id (the scene-map `FaceRef` lane: the caller
/// already resolved the selection name to `(handle, face_id)`).
pub fn resolve_face_selection(
    solid: &BrepSolid,
    face_id: u64,
) -> Result<SelectionGeometry, ResolveError> {
    let record = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .ok_or_else(|| ResolveError::NotFound {
            name: format!("face {face_id}"),
        })?;
    resolve_face_record(solid, record)
}

/// Resolve an edge by topology id (the `EdgeRef` lane).
pub fn resolve_edge_selection(
    solid: &BrepSolid,
    edge_id: u64,
) -> Result<SelectionGeometry, ResolveError> {
    let record = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| ResolveError::NotFound {
            name: format!("edge {edge_id}"),
        })?;
    resolve_edge_record(record)
}

/// Resolve a vertex selection by position (vertices have no kernel names; the
/// renderer references them by owning solid + position). Nearest topology
/// vertex wins — deterministic when two vertices are close — then the
/// scale-relative snap tolerance gates acceptance; the returned point is the
/// EXACT vertex position, not the query.
pub fn resolve_vertex_selection(
    solid: &BrepSolid,
    position: Vec3,
) -> Result<SelectionGeometry, ResolveError> {
    let mut best: Option<(f64, Vec3)> = None;
    for vertex in &solid.vertices {
        let distance = vertex.point.sub(position).length();
        if best
            .map(|(best_distance, _)| distance < best_distance)
            .unwrap_or(true)
        {
            best = Some((distance, vertex.point));
        }
    }
    match best {
        Some((distance, point))
            if distance <= VERTEX_SNAP_TOLERANCE * crate::solid_scale(solid) =>
        {
            Ok(SelectionGeometry::Point { position: point })
        }
        _ => Err(ResolveError::NotFound {
            name: format!(
                "vertex near ({}, {}, {})",
                position.x, position.y, position.z
            ),
        }),
    }
}

/// Whole-component representative point: the center of the aggregate AABB of
/// every face surface's control-point hull (plus topology vertices) across the
/// component's solids. The hull BOUNDS the exact surfaces (convex-hull
/// property) without touching tessellation; it overshoots curved faces, so
/// this is a representative anchor for coincident/parallel-with-component
/// semantics, not an exact bounding box.
pub fn resolve_component_point(solids: &[&BrepSolid]) -> Result<SelectionGeometry, ResolveError> {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut any = false;
    let mut include = |point: Vec3| {
        low = Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
        high = Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
        any = true;
    };
    for solid in solids {
        for vertex in &solid.vertices {
            include(vertex.point);
        }
        for shell in &solid.shells {
            for face in &shell.faces {
                for row in &face.surface.control_points {
                    for control in row {
                        let point = control.point().map_err(|error| ResolveError::Geometry {
                            name: "component".into(),
                            detail: error,
                        })?;
                        include(point);
                    }
                }
            }
        }
    }
    if !any {
        return Err(ResolveError::Geometry {
            name: "component".into(),
            detail: "component has no geometry to anchor".into(),
        });
    }
    Ok(SelectionGeometry::Point {
        position: low.add(high).scale(0.5),
    })
}

// ---------------------------------------------------------------------------
// Face resolution
// ---------------------------------------------------------------------------

fn face_label(record: &FaceRecord) -> String {
    record
        .name
        .clone()
        .unwrap_or_else(|| format!("face {}", record.id))
}

fn resolve_face_record(
    solid: &BrepSolid,
    record: &FaceRecord,
) -> Result<SelectionGeometry, ResolveError> {
    match record.surface.analytic() {
        Some(AnalyticSurface::RuledRevolution {
            frame, rho0, rho1, ..
        }) => {
            // Meridional generatrix (reconstruction-verified), so equal end
            // radii already mean a constant radius: a genuine cylinder.
            let span = rho0.abs().max(rho1.abs());
            let radius = ((rho0 - rho1).abs() <= RESOLVE_TOLERANCE * (1.0 + span))
                .then_some(0.5 * (rho0 + rho1));
            Ok(SelectionGeometry::Axis {
                origin: frame.origin,
                direction: frame.axis,
                radius,
            })
        }
        Some(AnalyticSurface::Sphere { frame, radius }) => Ok(SelectionGeometry::Sphere {
            center: frame.origin,
            radius: *radius,
        }),
        Some(AnalyticSurface::Torus { frame, .. }) => Ok(SelectionGeometry::Axis {
            origin: frame.origin,
            direction: frame.axis,
            radius: None,
        }),
        Some(AnalyticSurface::Revolution {
            frame, generatrix, ..
        }) => Ok(SelectionGeometry::Axis {
            origin: frame.origin,
            direction: frame.axis,
            radius: revolution_cylinder_radius(frame, generatrix),
        }),
        // Exact plane, or unrecognized (imported) geometry that may still be
        // planar — one shared sampled-planarity lane, `face_frame` parity.
        Some(AnalyticSurface::Plane { .. }) | None => planar_face(solid, record),
    }
}

/// `Some(radius)` when a general-revolution generatrix is a straight line at
/// CONSTANT radial distance from the axis — a partial cylinder (e.g. a 180°
/// revolve wall). Equal END radii alone are not enough: a line skew to the
/// axis revolves into a hyperboloid whose radius dips between equal ends. But
/// squared radial distance along a straight line is a QUADRATIC function of
/// arc length, so three equal samples (ends + midpoint) force it constant.
fn revolution_cylinder_radius(
    frame: &crate::RevolutionFrame,
    generatrix: &NurbsCurve,
) -> Option<f64> {
    if !control_net_is_colinear(generatrix) {
        return None;
    }
    let [t0, t1] = generatrix.domain().ok()?;
    let radial = |t: f64| -> Option<f64> {
        let point = generatrix.evaluate(t).ok()?;
        let delta = point.sub(frame.origin);
        Some(delta.sub(frame.axis.scale(delta.dot(frame.axis))).length())
    };
    let rho0 = radial(t0)?;
    let rho_mid = radial(0.5 * (t0 + t1))?;
    let rho1 = radial(t1)?;
    let span = rho0.abs().max(rho1.abs());
    let tolerance = RESOLVE_TOLERANCE * (1.0 + span);
    ((rho0 - rho1).abs() <= tolerance && (rho_mid - rho0).abs() <= tolerance)
        .then_some((rho0 + rho_mid + rho1) / 3.0)
}

/// Planar-face resolution: sampled-constant-normal acceptance (the established
/// `face_frame` planarity test, densified to a 3×3 grid), OUTWARD normal via
/// the face sense, origin = the centre of the face boundary's bounding box
/// taken in the face's OWN in-plane basis, projected onto the plane.
///
/// The basis is the surface's u-tangent at the patch midpoint (its normal
/// component removed) and the normal's cross with it. A component pose moves
/// the control net rigidly and that basis with it, so the box of the same
/// boundary samples is the same box and the anchor is a FIXED point of the
/// body. A world-axis box is not: on a face with no centre of symmetry (a
/// triangle, an L) its centre slides over the face as the body turns — and the
/// assembly angle gizmo, hinged on the two face anchors, slid with it on
/// every edit (`planar_face_anchor_is_a_fixed_body_point_on_an_asymmetric_face`).
/// For a centrally symmetric face (rectangle, disc) both rules name the same
/// point. The tangent test is scale-free (a stretched knot domain shrinks the
/// partials without moving the plane); the v-partial is the body-fixed
/// fallback, and a face whose partials both vanish at the midpoint cannot
/// reach this code — its normal would have failed above.
fn planar_face(solid: &BrepSolid, record: &FaceRecord) -> Result<SelectionGeometry, ResolveError> {
    let geometry = (|| -> Result<Option<SelectionGeometry>, String> {
        let [u0, u1] = record.surface.domain_u()?;
        let [v0, v1] = record.surface.domain_v()?;
        let (um, vm) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);
        let center_normal = record.surface.normal(um, vm)?;
        for u in [u0, um, u1] {
            for v in [v0, vm, v1] {
                if record.surface.normal(u, v)?.dot(center_normal) < 1.0 - 1e-6 {
                    return Ok(None);
                }
            }
        }
        let normal = if record.same_sense {
            center_normal
        } else {
            center_normal.scale(-1.0)
        }
        .normalized()?;

        // The face's own in-plane basis (body-fixed, see the doc above). The
        // test is SCALE-FREE — the in-plane part must carry the derivative's
        // own magnitude, whatever that magnitude is — because a knot domain
        // can be stretched arbitrarily without moving a point of the plane,
        // and an absolute floor would send such a face to a pose-dependent
        // rule (`planar_anchor_is_body_fixed_whatever_the_parameter_scale`).
        // Both partials are tried; both are body-fixed. Neither can fail for
        // a face that got here: `normal(um, vm)` above succeeded, so
        // |∂u × ∂v| > 1e-12 at the midpoint — both partials are non-zero —
        // and the accepted constant normal is their cross, so each lies in
        // the plane. The error is the unreachable branch, not a world axis.
        let plane_point = record.surface.evaluate(um, vm)?;
        let partials = record.surface.derivatives(um, vm, 1)?;
        let in_plane = |partial: Vec3| -> Option<Vec3> {
            let tangent = partial.sub(normal.scale(partial.dot(normal)));
            let length = tangent.length();
            (length.is_finite() && length > 1e-6 * partial.length() && length > 0.0)
                .then(|| tangent.scale(1.0 / length))
        };
        let e1 = in_plane(partials[1][0])
            .or_else(|| in_plane(partials[0][1]))
            .ok_or_else(|| {
                "planar face: both surface partials degenerate at the patch midpoint".to_string()
            })?;
        let e2 = normal.cross(e1).normalized()?;
        // Boundary bounding box in that basis, from the face loops'
        // non-degenerate edges; the patch midpoint for loop-less faces.
        let mut low = [f64::INFINITY; 2];
        let mut high = [f64::NEG_INFINITY; 2];
        let mut any = false;
        for loop_record in &record.loops {
            for coedge in &loop_record.coedges {
                let Some(edge) = solid.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
                    continue;
                };
                if edge.degenerate {
                    continue;
                }
                for step in 0..=4 {
                    let t = edge.t0 + (edge.t1 - edge.t0) * (step as f64 / 4.0);
                    let offset = edge.curve.evaluate(t)?.sub(plane_point);
                    let (a, b) = (offset.dot(e1), offset.dot(e2));
                    low = [low[0].min(a), low[1].min(b)];
                    high = [high[0].max(a), high[1].max(b)];
                    any = true;
                }
            }
        }
        let center = if any {
            plane_point
                .add(e1.scale((low[0] + high[0]) * 0.5))
                .add(e2.scale((low[1] + high[1]) * 0.5))
        } else {
            plane_point
        };
        // Project onto the plane so the origin lies exactly on it (the basis
        // is in-plane; this clears rounding).
        let signed = center.sub(plane_point).dot(normal);
        let origin = center.sub(normal.scale(signed));
        Ok(Some(SelectionGeometry::Plane { origin, normal }))
    })();
    match geometry {
        Ok(Some(frame)) => Ok(frame),
        Ok(None) => Err(ResolveError::Unsupported {
            name: face_label(record),
            detail: "face carries no analytic frame (freeform surface)".into(),
        }),
        Err(error) => Err(ResolveError::Geometry {
            name: face_label(record),
            detail: error,
        }),
    }
}

// ---------------------------------------------------------------------------
// Edge resolution
// ---------------------------------------------------------------------------

fn edge_label(record: &EdgeRecord) -> String {
    record
        .name
        .clone()
        .unwrap_or_else(|| format!("edge {}", record.id))
}

fn resolve_edge_record(record: &EdgeRecord) -> Result<SelectionGeometry, ResolveError> {
    if record.degenerate {
        return Err(ResolveError::Unsupported {
            name: edge_label(record),
            detail: "degenerate edge".into(),
        });
    }
    match edge_geometry(record) {
        Ok(Some(frame)) => Ok(frame),
        Ok(None) => Err(ResolveError::Unsupported {
            name: edge_label(record),
            detail: "edge is neither straight nor circular".into(),
        }),
        Err(error) => Err(ResolveError::Geometry {
            name: edge_label(record),
            detail: error,
        }),
    }
}

fn edge_geometry(record: &EdgeRecord) -> Result<Option<SelectionGeometry>, String> {
    if let Some(line) = straight_edge_line(record)? {
        return Ok(Some(line));
    }
    circular_edge_circle(record)
}

/// The whole curve's extent, for scale-relative acceptance.
fn control_net_extent(curve: &NurbsCurve) -> f64 {
    let mut extent = 0.0_f64;
    for control in &curve.control_points {
        if let Ok(point) = control.point() {
            extent = extent
                .max(point.x.abs())
                .max(point.y.abs())
                .max(point.z.abs());
        }
    }
    extent
}

/// A colinear control net is EXACT proof of a straight curve (convex-hull
/// property), and every straight curve this kernel builds — `make_line`
/// products through any split/degree change — keeps its net colinear.
fn control_net_is_colinear(curve: &NurbsCurve) -> bool {
    let points: Vec<Vec3> = curve
        .control_points
        .iter()
        .filter_map(|control| control.point().ok())
        .collect();
    let Some((&first, rest)) = points.split_first() else {
        return false;
    };
    let tolerance = RESOLVE_TOLERANCE * (1.0 + control_net_extent(curve));
    let Some(direction) = rest
        .iter()
        .map(|point| point.sub(first))
        .max_by(|a, b| a.length().total_cmp(&b.length()))
        .and_then(|chord| chord.normalized().ok())
    else {
        return false;
    };
    points.iter().all(|point| {
        let offset = point.sub(first);
        offset.sub(direction.scale(offset.dot(direction))).length() <= tolerance
    })
}

/// Straight edge → its infinite carrier line, oriented start→end over the
/// trimmed span, origin at the chord midpoint.
fn straight_edge_line(record: &EdgeRecord) -> Result<Option<SelectionGeometry>, String> {
    if !control_net_is_colinear(&record.curve) {
        return Ok(None);
    }
    let start = record.curve.evaluate(record.t0)?;
    let end = record.curve.evaluate(record.t1)?;
    let chord = end.sub(start);
    if chord.length() <= RESOLVE_TOLERANCE * (1.0 + control_net_extent(&record.curve)) {
        // Zero-length trim of a straight curve — nothing to orient by.
        return Ok(None);
    }
    Ok(Some(SelectionGeometry::Line {
        origin: start.add(end).scale(0.5),
        direction: chord.normalized()?,
    }))
}

/// Circular/arc edge → center + axis + radius: a candidate circle from three
/// interior samples (distinct even on a closed full circle), then VERIFIED at
/// [`CIRCLE_VERIFY_SAMPLES`] points — equidistant from the center and coplanar
/// — so ellipses, splines, and helices fail cleanly while exact kernel arcs
/// pass at machine precision.
fn circular_edge_circle(record: &EdgeRecord) -> Result<Option<SelectionGeometry>, String> {
    let span = record.t1 - record.t0;
    if !(span > 0.0) {
        return Ok(None);
    }
    let at = |fraction: f64| record.curve.evaluate(record.t0 + span * fraction);
    let (a, b, c) = (at(1.0 / 6.0)?, at(0.5)?, at(5.0 / 6.0)?);
    let Some(center) = crate::analytic_surface::circumcenter(a, b, c) else {
        return Ok(None);
    };
    // Right-hand rule along the increasing-parameter direction.
    let Ok(axis) = b.sub(a).cross(c.sub(b)).normalized() else {
        return Ok(None);
    };
    let radius = a.sub(center).length();
    let tolerance = RESOLVE_TOLERANCE * (1.0 + control_net_extent(&record.curve));
    for step in 0..=CIRCLE_VERIFY_SAMPLES {
        let point = at(step as f64 / CIRCLE_VERIFY_SAMPLES as f64)?;
        let delta = point.sub(center);
        if (delta.length() - radius).abs() > tolerance || delta.dot(axis).abs() > tolerance {
            return Ok(None);
        }
    }
    Ok(Some(SelectionGeometry::Circle {
        center,
        axis,
        radius,
    }))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

