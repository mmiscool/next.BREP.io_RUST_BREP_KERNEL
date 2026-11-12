//! Center mark / centerline (`CL`): the drafting marks that locate a round
//! feature's centre or axis. One type, two styles:
//!
//! - **mark** — a centre cross on every picked circular edge, cylindrical or
//!   conical face of constant radius, or sphere: a short cross at the centre,
//!   a gap, then long dashes reaching `extension` past the circle;
//! - **line** — a chain line (long dash, gap, short dash, gap …) along one
//!   cylinder's or cone's axis over the face's length, through the centres of
//!   two or more round features (a bolt pattern's row), or midway between two
//!   parallel straight edges (a symmetry line); it runs `extension` past its
//!   ends (past the circles when it ends on round features).
//!
//! A mark's cross is fixed in the model: it lies in the circle's plane with
//! one arm parallel to the optional `orientation` reference (a straight
//! edge, a planar face, or a cylinder / cone / circular edge's axis) and the
//! other square to it; without a reference the first arm runs along the
//! model X axis (Y where X is the circle's axis). Moving the camera never
//! turns it.
//!
//! Presentation only: there is no value and no text, so nothing is written
//! as semantic PMI and the viewport draws no label chip for it (its name is
//! in the PMI tree); the STEP file carries the drawn polylines.

use super::{id_field, number_field, options_field, params, reference_field, schema_entry, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, class_of, edge_endpoints, face_axial_span, perpendicular_in_plane, resolve_reference};
use crate::feature_pipeline::pmi::{CenterCircle, PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use crate::feature_pipeline::SelectionProbe;
use crate::{SelectionGeometry, Vec3};

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "centerline",
    short_name: "CL",
    icon: "\u{2104}",
    long_name: "\u{2104} Center mark & line",
    label: "Center mark & line",
    applicable,
    schema,
    resolve,
};

/// The most targets one annotation takes (a hole pattern in one go).
const MAX_TARGETS: usize = 64;

/// One or more faces / edges and nothing else.
fn applicable(probe: &SelectionProbe) -> bool {
    probe.faces + probe.edges >= 1
        && probe.vertices == 0
        && probe.planes == 0
        && probe.solids == 0
        && probe.sketches == 0
}

fn schema() -> serde_json::Value {
    schema_entry(
        &DEF,
        params(vec![
            ("id", id_field()),
            (
                "style",
                options_field(
                    "Style",
                    &["mark", "line"],
                    "mark",
                    "mark: a centre cross on each round feature · line: a chain line along an axis, through centres, or between two parallel edges",
                ),
            ),
            (
                "targets",
                reference_field(
                    "Targets",
                    &["FACE", "EDGE"],
                    true,
                    1,
                    MAX_TARGETS,
                    "Circular edges, cylindrical / conical / spherical faces — or, for a line, two parallel straight edges",
                ),
            ),
            (
                "extension",
                number_field("Extension", 2.0, 0.5, "How far the lines reach past the circle (mark) or the ends (line)"),
            ),
            (
                "orientation",
                reference_field(
                    "Orientation",
                    &["FACE", "EDGE"],
                    false,
                    0,
                    1,
                    "Optional (mark): a straight edge, planar face or cylinder one arm of the cross runs parallel to; empty = the model X axis",
                ),
            ),
        ]),
    )
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("targets");
    if targets.is_empty() {
        return Err("select the round features (circular edges, holes, cylinders) to mark".into());
    }
    if targets.len() > MAX_TARGETS {
        return Err(format!("at most {MAX_TARGETS} targets"));
    }
    let extension = annotation.number("extension", context.env, 2.0)?;
    if !extension.is_finite() || extension < 0.0 {
        return Err(format!("extension must be zero or more, not {extension}"));
    }
    let mut frames = Vec::with_capacity(targets.len());
    for target in &targets {
        frames.push((target.as_str(), resolve_reference(context.scene, target)?));
    }
    if annotation.text("style").eq_ignore_ascii_case("line") {
        line(&frames, extension, context, targets.clone())
    } else {
        let orientation = match annotation.references("orientation").first() {
            Some(name) => Some((name.clone(), resolve_reference(context.scene, name)?)),
            None => None,
        };
        let mut references = targets.clone();
        if let Some((name, _)) = &orientation {
            references.push(name.clone());
        }
        mark(&frames, orientation.as_ref().map(|(name, geometry)| (name.as_str(), geometry)), extension, context, references)
    }
}

/// Below this length a direction projected into a circle's plane is taken
/// as square to the plane (no usable in-plane direction).
const DEGENERATE: f64 = 1e-6;

/// `d` with its component along the unit `axis` removed, normalised; `None`
/// when `d` is (within [`DEGENERATE`]) along the axis.
fn in_plane(d: Vec3, axis: Vec3) -> Option<Vec3> {
    let planar = d.sub(axis.scale(d.dot(axis)));
    if planar.length() < DEGENERATE {
        return None;
    }
    planar.normalized().ok()
}

/// The model-fixed in-plane direction of one mark's first arm. `axis` is the
/// circle's unit normal, or `None` for a sphere (its direction is projected
/// into the mark's plane by the layout). Without a reference: model X, or Y
/// where X lies along the axis. With one:
/// - a straight edge: its direction, projected into the circle's plane;
/// - a planar face: the face's trace on the circle's plane (`normal × axis`),
///   so the arm is parallel to the face;
/// - a cylinder / cone / circular edge: its axis projected into the plane,
///   or, when that axis is parallel to the circle's (a neighbouring hole),
///   the direction from the mark's centre toward it.
fn arm_direction(center: Vec3, axis: Option<Vec3>, reference: Option<(&str, &SelectionGeometry)>) -> Result<Vec3, String> {
    let x = Vec3::new(1.0, 0.0, 0.0);
    let Some((name, geometry)) = reference else {
        return Ok(match axis {
            Some(axis) => in_plane(x, axis).or_else(|| in_plane(Vec3::new(0.0, 1.0, 0.0), axis)).unwrap_or(x),
            None => x,
        });
    };
    let square = || format!("orientation '{name}' is square to the marked circle's plane — pick an edge or face that has a direction in that plane");
    let direction = |d: Vec3| -> Result<Vec3, String> {
        match axis {
            Some(axis) => in_plane(d, axis).ok_or_else(square),
            None => d.normalized().map_err(|_| square()),
        }
    };
    match *geometry {
        SelectionGeometry::Line { direction: d, .. } => direction(d),
        SelectionGeometry::Plane { normal, .. } => match axis {
            Some(axis) => {
                let trace = normal.cross(axis);
                if trace.length() < DEGENERATE {
                    return Err(format!(
                        "orientation face '{name}' is parallel to the marked circle's plane — pick a face or edge that crosses it"
                    ));
                }
                trace.normalized().map_err(|_| square())
            }
            // A sphere's mark: an arm along the face, in the XY plane the
            // layout lays the mark in.
            None => direction(normal.cross(Vec3::new(0.0, 0.0, 1.0))),
        },
        SelectionGeometry::Axis { origin, direction: d, .. } | SelectionGeometry::Circle { center: origin, axis: d, .. } => {
            let d = d.normalized().map_err(|_| format!("orientation '{name}' has no axis direction"))?;
            match axis {
                Some(axis) => match in_plane(d, axis) {
                    Some(v) => Ok(v),
                    None => {
                        // Parallel axes: toward the reference's axis.
                        let toward = origin.sub(center);
                        in_plane(toward.sub(d.scale(toward.dot(d))), axis).ok_or_else(|| {
                            format!("orientation '{name}' shares the marked circle's axis — pick an edge or face beside it")
                        })
                    }
                },
                None => direction(d),
            }
        }
        ref other => Err(format!(
            "orientation '{name}' is a {} — orient a center mark by a straight edge, a planar face or a cylinder",
            class_of(other)
        )),
    }
}

/// A round feature's centre circle, for a mark. A cylinder or cone face is
/// marked at the end of its length its axis points to, with the length as
/// its `depth`, so the layout can draw it on the end facing the viewer.
fn round(name: &str, geometry: &SelectionGeometry, context: &PmiContext<'_>) -> Result<CenterCircle, String> {
    match *geometry {
        SelectionGeometry::Circle { center, axis, radius } => {
            let axis = axis.normalized().map_err(|_| format!("'{name}' has no axis direction"))?;
            Ok(CenterCircle { center: a3(center), axis: a3(axis), radius, depth: 0.0, x_dir: [0.0; 3] })
        }
        SelectionGeometry::Axis { origin, direction, radius: Some(radius) } => {
            let direction = direction.normalized().map_err(|_| format!("'{name}' has no axis direction"))?;
            let (low, high) = face_axial_span(context.scene, name, origin, direction)?;
            Ok(CenterCircle {
                center: a3(origin.add(direction.scale(high))),
                axis: a3(direction),
                radius,
                depth: high - low,
                x_dir: [0.0; 3],
            })
        }
        SelectionGeometry::Axis { radius: None, .. } => Err(format!(
            "'{name}' has no constant radius (a cone or a general revolution) — pick a circular edge on it"
        )),
        // A sphere has no plane of its own: the layout lays its mark in the
        // model's XY plane (or a picked annotation plane).
        SelectionGeometry::Sphere { center, radius } => Ok(CenterCircle { center: a3(center), axis: [0.0; 3], radius, depth: 0.0, x_dir: [0.0; 3] }),
        ref other => Err(format!(
            "'{name}' is a {} — a center mark needs a circular edge, a cylinder, a cone or a sphere",
            class_of(other)
        )),
    }
}

fn mark(
    frames: &[(&str, SelectionGeometry)],
    orientation: Option<(&str, &SelectionGeometry)>,
    extension: f64,
    context: &PmiContext<'_>,
    references: Vec<String>,
) -> Result<Resolved, String> {
    let mut marks = Vec::with_capacity(frames.len());
    for (name, geometry) in frames {
        let mut mark = round(name, geometry, context)?;
        let axis = Vec3::new(mark.axis[0], mark.axis[1], mark.axis[2]);
        let center = Vec3::new(mark.center[0], mark.center[1], mark.center[2]);
        let axis = (axis.length() > 1e-12).then_some(axis);
        mark.x_dir = a3(arm_direction(center, axis, orientation)?);
        marks.push(mark);
    }
    let first = &marks[0];
    let axis = Vec3::new(first.axis[0], first.axis[1], first.axis[2]);
    let center = Vec3::new(first.center[0], first.center[1], first.center[2]);
    // The chip stands off the cross, beyond the dashes' ends.
    let out = perpendicular_in_plane(if axis.length() > 1e-12 { axis } else { Vec3::new(0.0, 0.0, 1.0) }, Vec3::new(1.0, 1.0, 0.0));
    let default_label = center.add(out.scale((first.radius + extension) * 1.25 + 1.0));
    Ok(Resolved {
        text: String::new(),
        value: None,
        unit: "",
        references,
        geometry: PmiGeometry::CenterMark { marks, extension },
        default_label: a3(default_label),
    })
}

/// A centre a line passes through: the point, and how far past it the line
/// must reach to clear the feature (its radius).
struct Station {
    point: Vec3,
    clearance: f64,
}

fn line(
    frames: &[(&str, SelectionGeometry)],
    extension: f64,
    context: &PmiContext<'_>,
    references: Vec<String>,
) -> Result<Resolved, String> {
    let (a, b) = match frames {
        // Along one cylinder's or cone's axis, over the face's length.
        [(name, SelectionGeometry::Axis { origin, direction, .. })] => {
            let direction = direction.normalized().map_err(|_| format!("'{name}' has no axis direction"))?;
            let (low, high) = face_axial_span(context.scene, name, *origin, direction)?;
            (
                origin.add(direction.scale(low - extension)),
                origin.add(direction.scale(high + extension)),
            )
        }
        [(_, SelectionGeometry::Line { .. }), (_, SelectionGeometry::Line { .. })] => midline(frames[0].0, frames[1].0, extension, context)?,
        [(name, other)] => {
            return Err(format!(
                "'{name}' is a {} — a centerline on one target needs a cylindrical or conical face (or pick two round features, or two parallel edges)",
                class_of(other)
            ))
        }
        _ => through_centres(frames, extension)?,
    };
    let span = b.sub(a);
    let mid = a.add(span.scale(0.5));
    let side = perpendicular_in_plane(span, Vec3::new(1.0, 1.0, 1.0));
    let default_label = mid.add(side.scale(span.length() * 0.1 + 2.0));
    Ok(Resolved {
        text: String::new(),
        value: None,
        unit: "",
        references,
        geometry: PmiGeometry::CenterLine { a: a3(a), b: a3(b) },
        default_label: a3(default_label),
    })
}

/// A line through the centres of round features, first to last, reaching
/// `extension` past the outer circles. An axis face contributes the point of
/// its axis level with the first centre (two parallel holes give the line
/// between their axes).
fn through_centres(frames: &[(&str, SelectionGeometry)], extension: f64) -> Result<(Vec3, Vec3), String> {
    let mut stations: Vec<Station> = Vec::with_capacity(frames.len());
    let mut reference: Option<Vec3> = None;
    for (name, geometry) in frames {
        let station = match *geometry {
            SelectionGeometry::Circle { center, radius, .. } | SelectionGeometry::Sphere { center, radius } => Station { point: center, clearance: radius },
            SelectionGeometry::Axis { origin, direction, radius } => {
                let level = reference.unwrap_or(origin);
                let d = direction.normalized().map_err(|_| format!("'{name}' has no axis direction"))?;
                Station {
                    point: origin.add(d.scale(level.sub(origin).dot(d))),
                    clearance: radius.unwrap_or(0.0),
                }
            }
            SelectionGeometry::Point { position } => Station { point: position, clearance: 0.0 },
            ref other => {
                return Err(format!(
                    "'{name}' is a {} — a centerline through centres needs circular edges, cylinders, cones or spheres",
                    class_of(other)
                ))
            }
        };
        reference.get_or_insert(station.point);
        stations.push(station);
    }
    // The two stations farthest apart are the ends; every other centre must
    // lie on the line between them.
    let (mut i, mut j, mut best) = (0, 0, 0.0f64);
    for (p, s) in stations.iter().enumerate() {
        for (q, t) in stations.iter().enumerate().skip(p + 1) {
            let d = s.point.sub(t.point).length();
            if d > best {
                (i, j, best) = (p, q, d);
            }
        }
    }
    if best < 1e-9 {
        return Err("the picked features share one centre — pick features whose centres are apart, or use the mark style".into());
    }
    let d = stations[j].point.sub(stations[i].point).scale(1.0 / best);
    let tolerance = 1e-6 * best.max(1.0);
    for (k, station) in stations.iter().enumerate() {
        let rel = station.point.sub(stations[i].point);
        if rel.sub(d.scale(rel.dot(d))).length() > tolerance {
            return Err(format!("the centre of '{}' is not on the line through the others", frames[k].0));
        }
    }
    Ok((
        stations[i].point.sub(d.scale(stations[i].clearance + extension)),
        stations[j].point.add(d.scale(stations[j].clearance + extension)),
    ))
}

/// The symmetry line midway between two parallel straight edges, over the
/// span either covers, reaching `extension` past it.
fn midline(first: &str, second: &str, extension: f64, context: &PmiContext<'_>) -> Result<(Vec3, Vec3), String> {
    let (a0, a1) = edge_endpoints(context.scene, first)?;
    let (b0, b1) = edge_endpoints(context.scene, second)?;
    let da = a1.sub(a0);
    let db = b1.sub(b0);
    let (length_a, length_b) = (da.length(), db.length());
    if length_a < 1e-9 || length_b < 1e-9 {
        return Err("a straight edge has no length".into());
    }
    let d = da.scale(1.0 / length_a);
    if d.cross(db.scale(1.0 / length_b)).length() > 1e-6 {
        return Err(format!("'{first}' and '{second}' are not parallel — a centerline between edges needs parallel ones"));
    }
    // The second edge's foot on the first's carrier: the offset between them.
    let offset = b0.sub(a0).sub(d.scale(b0.sub(a0).dot(d)));
    if offset.length() < 1e-9 {
        return Err(format!("'{first}' and '{second}' lie on one line"));
    }
    let middle = a0.add(offset.scale(0.5));
    let stations = [0.0, length_a, b0.sub(a0).dot(d), b1.sub(a0).dot(d)];
    let low = stations.iter().cloned().fold(f64::INFINITY, f64::min);
    let high = stations.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    Ok((middle.add(d.scale(low - extension)), middle.add(d.scale(high + extension))))
}
