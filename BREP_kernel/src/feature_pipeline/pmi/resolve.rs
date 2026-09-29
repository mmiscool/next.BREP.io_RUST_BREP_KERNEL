//! PMI reference resolution and the measurement helpers every annotation
//! resolver shares.
//!
//! A reference is a kernel entity NAME; both plain and component-owned
//! geometry resolve (the assembly constraints' component fence does not apply
//! — PMI annotates whatever is on screen). Conventions:
//!
//! - a face / edge name → its analytic frame through
//!   [`crate::assembly_resolve`] (plane, axis, sphere, circle, line);
//! - `{solid}@x,y,z` → the vertex of `solid` nearest that WORLD position
//!   (PMI resolves against the world-posed resident solids, the same solids
//!   the STEP exporter writes — unlike an assembly constraint's
//!   component-local vertex ref);
//! - a D / P frame name → the plane (origin + z axis);
//! - a solid name → its representative point (the aggregate bbox center).

use crate::feature_pipeline::SceneMap;
use crate::{
    resolve_edge_selection, resolve_face_selection, resolve_vertex_selection, SelectionGeometry,
    Vec3,
};


pub fn v3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

pub fn a3(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// Parse `x,y,z`.
fn parse_triple(text: &str) -> Option<Vec3> {
    let mut parts = text.split(',').map(|part| part.trim().parse::<f64>());
    let x = parts.next()?.ok()?;
    let y = parts.next()?.ok()?;
    let z = parts.next()?.ok()?;
    if parts.next().is_some() {
        return None;
    }
    Some(Vec3::new(x, y, z))
}

/// Resolve one reference name to its analytic frame (module docs).
pub fn resolve_reference(scene: &SceneMap, name: &str) -> Result<SelectionGeometry, String> {
    let name = name.trim();
    if name.is_empty() {
        return Err("empty reference".into());
    }
    // Vertex ref: `{solid}@x,y,z` in world coordinates.
    if let Some((solid_name, coords)) = name.split_once('@') {
        let handle = scene
            .resolve_solid(solid_name)
            .ok_or_else(|| format!("vertex ref '{name}': unknown solid '{solid_name}'"))?;
        let position = parse_triple(coords)
            .ok_or_else(|| format!("vertex ref '{name}': position must be 'x,y,z' numbers"))?;
        return crate::with_registered_solid_str(handle, |solid| {
            Ok(resolve_vertex_selection(solid, position))
        })?
        .map_err(|error| format!("vertex ref '{name}': {error}"));
    }
    if let Some(face) = scene.resolve_face(name) {
        return crate::with_registered_solid_str(face.handle, |solid| {
            Ok(resolve_face_selection(solid, face.face_id))
        })?
        .map_err(|error| format!("face '{name}': {error}"));
    }
    if let Some(edge) = scene.resolve_edge(name) {
        return crate::with_registered_solid_str(edge.handle, |solid| {
            Ok(resolve_edge_selection(solid, edge.edge_id))
        })?
        .map_err(|error| format!("edge '{name}': {error}"));
    }
    if let Some(frame) = scene.resolve_frame(name) {
        return Ok(SelectionGeometry::Plane {
            origin: frame.origin,
            normal: frame.z_axis,
        });
    }
    if let Some(handle) = scene.resolve_solid(name) {
        let center = solid_bbox_center(handle)?;
        return Ok(SelectionGeometry::Point { position: center });
    }
    Err(format!("'{name}' is not in the model"))
}

/// The world endpoints of a named edge (its curve evaluated at the trimmed
/// span ends), for a single-edge length dimension.
pub fn edge_endpoints(scene: &SceneMap, name: &str) -> Result<(Vec3, Vec3), String> {
    let edge = scene
        .resolve_edge(name)
        .ok_or_else(|| format!("'{name}' is not an edge"))?;
    crate::with_registered_solid_str(edge.handle, |solid| {
        let record = solid
            .edges
            .iter()
            .find(|record| record.id == edge.edge_id)
            .ok_or_else(|| format!("edge '{name}' has no record"))?;
        let start = record.curve.evaluate(record.t0)?;
        let end = record.curve.evaluate(record.t1)?;
        Ok((start, end))
    })
}

/// Whether `name` resolves to a straight edge.
pub fn is_straight_edge(geometry: &SelectionGeometry) -> bool {
    matches!(geometry, SelectionGeometry::Line { .. })
}

/// The center of the aggregate AABB of a resident solid's topology vertices
/// and surface control hulls (bounds the exact surfaces without tessellation).
pub fn solid_bbox_center(handle: u32) -> Result<Vec3, String> {
    let (low, high) = solid_bbox(handle)?;
    Ok(Vec3::new(
        (low.x + high.x) * 0.5,
        (low.y + high.y) * 0.5,
        (low.z + high.z) * 0.5,
    ))
}

/// The aggregate AABB (topology vertices ∪ control-point hulls) of a resident solid.
pub fn solid_bbox(handle: u32) -> Result<(Vec3, Vec3), String> {
    crate::with_registered_solid_str(handle, |solid| {
        let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        let mut any = false;
        let mut include = |point: Vec3| {
            low = Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
            high = Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
            any = true;
        };
        for vertex in &solid.vertices {
            include(vertex.point);
        }
        for shell in &solid.shells {
            for face in &shell.faces {
                for row in &face.surface.control_points {
                    for control in row {
                        include(control.point()?);
                    }
                }
            }
        }
        if !any {
            return Err("solid has no geometry".into());
        }
        Ok((low, high))
    })
}

/// The direction an element carries: a plane's normal, a line's / axis's
/// direction, a circle's axis. Points and spheres carry none.
pub fn direction_of(geometry: &SelectionGeometry) -> Option<Vec3> {
    match geometry {
        SelectionGeometry::Plane { normal, .. } => Some(*normal),
        SelectionGeometry::Line { direction, .. } | SelectionGeometry::Axis { direction, .. } => {
            Some(*direction)
        }
        SelectionGeometry::Circle { axis, .. } => Some(*axis),
        SelectionGeometry::Sphere { .. } | SelectionGeometry::Point { .. } => None,
    }
}

/// A lowercase geometry class word for messages.
pub fn class_of(geometry: &SelectionGeometry) -> &'static str {
    match geometry {
        SelectionGeometry::Plane { .. } => "plane",
        SelectionGeometry::Axis { .. } => "axis",
        SelectionGeometry::Sphere { .. } => "sphere",
        SelectionGeometry::Circle { .. } => "circle",
        SelectionGeometry::Line { .. } => "line",
        SelectionGeometry::Point { .. } => "point",
    }
}

/// The measurement classification of an element for a linear dimension:
/// planes stay planes; straight edges and axis-bearing faces measure on their
/// INFINITE carrier lines; circles and spheres from their centers; vertices
/// from their points.
pub enum LinearSide {
    Plane { origin: Vec3, normal: Vec3 },
    Line { origin: Vec3, direction: Vec3 },
    Point(Vec3),
}

pub fn classify_linear(geometry: &SelectionGeometry) -> LinearSide {
    match *geometry {
        SelectionGeometry::Plane { origin, normal } => LinearSide::Plane { origin, normal },
        SelectionGeometry::Line { origin, direction }
        | SelectionGeometry::Axis {
            origin, direction, ..
        } => LinearSide::Line { origin, direction },
        SelectionGeometry::Circle { center, .. } | SelectionGeometry::Sphere { center, .. } => {
            LinearSide::Point(center)
        }
        SelectionGeometry::Point { position } => LinearSide::Point(position),
    }
}

fn unit(v: Vec3) -> Result<Vec3, String> {
    v.normalized().map_err(|_| "degenerate direction".to_string())
}

/// Foot of the perpendicular from `point` onto the line `(origin, direction)`.
fn foot_on_line(point: Vec3, origin: Vec3, direction: Vec3) -> Vec3 {
    let d = direction;
    let t = point.sub(origin).dot(d) / d.dot(d).max(1e-300);
    origin.add(d.scale(t))
}

/// Foot of the perpendicular from `point` onto the plane `(origin, normal)`.
fn foot_on_plane(point: Vec3, origin: Vec3, normal: Vec3) -> Vec3 {
    let n = normal;
    let h = point.sub(origin).dot(n) / n.dot(n).max(1e-300);
    point.sub(n.scale(h))
}

/// The two points a linear dimension measures between (`a` on the first
/// element, `b` on the second), per the module's classification. Errors name
/// the unsupported pairing (a non-parallel plane pair, a plane against a
/// crossing line).
pub fn linear_between(first: &SelectionGeometry, second: &SelectionGeometry) -> Result<(Vec3, Vec3), String> {
    use LinearSide::*;
    match (classify_linear(first), classify_linear(second)) {
        (Point(p), Point(q)) => Ok((p, q)),
        (Point(p), Line { origin, direction }) => Ok((p, foot_on_line(p, origin, direction))),
        (Line { origin, direction }, Point(q)) => Ok((foot_on_line(q, origin, direction), q)),
        (Point(p), Plane { origin, normal }) => Ok((p, foot_on_plane(p, origin, normal))),
        (Plane { origin, normal }, Point(q)) => Ok((foot_on_plane(q, origin, normal), q)),
        (
            Line {
                origin: oa,
                direction: da,
            },
            Line {
                origin: ob,
                direction: db,
            },
        ) => {
            let da = unit(da)?;
            let db = unit(db)?;
            let cross = da.cross(db);
            if cross.length() < 1e-7 {
                // Parallel carriers: the spacing, measured from the first origin.
                return Ok((oa, foot_on_line(oa, ob, db)));
            }
            // Skew / crossing: the closest points on the two infinite lines.
            let w = oa.sub(ob);
            let a = da.dot(da);
            let b = da.dot(db);
            let c = db.dot(db);
            let d = da.dot(w);
            let e = db.dot(w);
            let denom = a * c - b * b;
            let s = (b * e - c * d) / denom;
            let t = (a * e - b * d) / denom;
            Ok((oa.add(da.scale(s)), ob.add(db.scale(t))))
        }
        (
            Plane {
                origin: oa,
                normal: na,
            },
            Plane {
                origin: ob,
                normal: nb,
            },
        ) => {
            let na = unit(na)?;
            let nb = unit(nb)?;
            if na.cross(nb).length() > 1e-6 {
                return Err("the two planes are not parallel — a linear dimension needs parallel planes (use an angle dimension)".into());
            }
            Ok((oa, foot_on_plane(oa, ob, nb)))
        }
        (
            Plane {
                origin: po,
                normal,
            },
            Line { origin, direction },
        ) => {
            if unit(normal)?.dot(unit(direction)?).abs() > 1e-6 {
                return Err("the line is not parallel to the plane".into());
            }
            Ok((foot_on_plane(origin, po, normal), origin))
        }
        (
            Line { origin, direction },
            Plane {
                origin: po,
                normal,
            },
        ) => {
            if unit(normal)?.dot(unit(direction)?).abs() > 1e-6 {
                return Err("the line is not parallel to the plane".into());
            }
            Ok((origin, foot_on_plane(origin, po, normal)))
        }
    }
}

/// A unit vector perpendicular to `axis` — `preferred` projected into the
/// plane when it has a component there, else an arbitrary perpendicular.
pub fn perpendicular_in_plane(axis: Vec3, preferred: Vec3) -> Vec3 {
    let axis = axis.normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    let planar = preferred.sub(axis.scale(preferred.dot(axis)));
    match planar.normalized() {
        Ok(v) if planar.length() > 1e-9 => v,
        _ => axis
            .perpendicular()
            .unwrap_or(Vec3::new(1.0, 0.0, 0.0)),
    }
}
