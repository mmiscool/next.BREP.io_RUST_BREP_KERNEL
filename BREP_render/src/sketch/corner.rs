//! Sketch corner fillet and chamfer: two lines meeting at one shared point are
//! trimmed back and joined by a tangent arc (fillet) or a straight cut (chamfer).
//!
//! The corner point itself stays, as a construction point held on both lines
//! (point-on-line `⏛`): the virtual sharp. Every constraint that named the corner
//! (a rectangle's perpendiculars, a side's length) keeps its meaning, and the new
//! profile edges are tied to it by constraints, so the corner stays parametric
//! when the sketch is dragged or re-solved:
//! - fillet: the arc is tangent to both lines, held as the radius square to
//!   each line at its tangent point (`⟂`), and carries a radius dimension
//!   (`⟺` with `displayStyle: "radius"`);
//! - chamfer: a distance dimension from the virtual sharp to the first cut point,
//!   and either an equal distance to the second (`⇌`) or an angle (`∠`) between
//!   the first line and the cut.
//!
//! Values are left for the solver to seed from the exact geometry built here
//! (`value: null`, `valueNeedsSetup: true`), so the dimensions read what was asked.

use super::doc::{id_key, SketchDoc, SketchGeometry, SketchPoint};
use serde_json::{Map, Value};

/// How a chamfer is sized.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ChamferSize {
    /// Both cut points `distance` from the corner.
    Equal { distance: f64 },
    /// The first cut point `distance` from the corner along the first line, the
    /// cut at `angle_degrees` to that line.
    DistanceAngle { distance: f64, angle_degrees: f64 },
}

/// The fillet and chamfer tools' options (the sketch tool-options panel).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CornerOptions {
    /// The fillet radius.
    pub radius: f64,
    /// Whether a chamfer is sized by an angle as well (`Distance + angle`) or is
    /// equal both ways (`Equal distance`).
    pub chamfer_by_angle: bool,
    /// The chamfer distance (from the corner, along the first line).
    pub chamfer_distance: f64,
    /// The chamfer angle to the first line, in degrees (`Distance + angle`).
    pub chamfer_angle_degrees: f64,
}

impl Default for CornerOptions {
    fn default() -> Self {
        Self { radius: 1.0, chamfer_by_angle: false, chamfer_distance: 1.0, chamfer_angle_degrees: 45.0 }
    }
}

impl CornerOptions {
    /// The chamfer size these options describe.
    pub fn chamfer_size(&self) -> ChamferSize {
        if self.chamfer_by_angle {
            ChamferSize::DistanceAngle { distance: self.chamfer_distance, angle_degrees: self.chamfer_angle_degrees }
        } else {
            ChamferSize::Equal { distance: self.chamfer_distance }
        }
    }
}

/// What a fillet or chamfer added: the new profile edge and the constraints.
#[derive(Clone, Debug, PartialEq)]
pub struct CornerEdit {
    /// The arc (fillet) or line (chamfer) inserted at the corner.
    pub edge: Value,
    /// The constraints added (their ids).
    pub constraints: Vec<Value>,
}

/// The two lines meeting at `corner`, each with its far endpoint.
struct Corner {
    corner: (f64, f64),
    lines: [(Value, Value, (f64, f64)); 2],
}

fn point_xy(doc: &SketchDoc, id: &Value) -> Option<(f64, f64)> {
    doc.point(id).map(|p| (p.x, p.y))
}

/// The two lines that share `corner` as an endpoint. Refuses anything else, by
/// name, so the sketch is never left half-edited.
fn corner_lines(doc: &SketchDoc, corner: &Value) -> Result<Corner, String> {
    let key = id_key(corner);
    let at = point_xy(doc, corner).ok_or_else(|| "the corner point is not in the sketch".to_string())?;
    let touching: Vec<&SketchGeometry> = doc.geometries.iter().filter(|g| g.points.iter().any(|p| id_key(p) == key)).collect();
    if touching.len() != 2 {
        return Err(format!("no shared corner: {} curves meet at this point (a corner is two)", touching.len()));
    }
    let mut lines = Vec::new();
    for geometry in touching {
        if geometry.geom_type != "line" {
            return Err(format!("only corners of two lines are supported yet (this one has a {})", geometry.geom_type));
        }
        if geometry.points.len() != 2 {
            return Err("a line of the corner is malformed".to_string());
        }
        let far = if id_key(&geometry.points[0]) == key { &geometry.points[1] } else { &geometry.points[0] };
        if id_key(far) == key {
            return Err("a line of the corner has no length".to_string());
        }
        let far_xy = point_xy(doc, far).ok_or_else(|| "a corner line's endpoint is missing".to_string())?;
        if (far_xy.0 - at.0).hypot(far_xy.1 - at.1) <= 1e-9 {
            return Err("a line of the corner has no length".to_string());
        }
        lines.push((geometry.id.clone(), far.clone(), far_xy));
    }
    let [first, second]: [(Value, Value, (f64, f64)); 2] = lines.try_into().map_err(|_| "no shared corner".to_string())?;
    Ok(Corner { corner: at, lines: [first, second] })
}

/// The corner nearest `(u, v)` within `radius`: a point two lines share. `None`
/// when no such point is in reach.
pub fn corner_near(doc: &SketchDoc, u: f64, v: f64, radius: f64) -> Option<Value> {
    doc.points
        .iter()
        .filter(|p| (p.x - u).hypot(p.y - v) <= radius)
        .filter(|p| corner_lines(doc, &p.id).is_ok())
        .min_by(|a, b| (a.x - u).hypot(a.y - v).total_cmp(&(b.x - u).hypot(b.y - v)))
        .map(|p| p.id.clone())
}

fn add_point(doc: &mut SketchDoc, (x, y): (f64, f64)) -> Value {
    let id = doc.next_point_id();
    doc.points.push(SketchPoint { id: id.clone(), x, y, fixed: false, construction: false, external_reference: false });
    id
}

fn add_geometry(doc: &mut SketchDoc, geom_type: &str, points: Vec<Value>) -> Value {
    let id = doc.next_geometry_id();
    let mut extra = Map::new();
    extra.insert("construction".to_string(), Value::Bool(false));
    doc.geometries.push(SketchGeometry { id: id.clone(), geom_type: geom_type.to_string(), points, extra });
    id
}

/// Append a constraint (always: the corner's new points are fresh, so none can
/// already exist), its value left for the solver to seed.
fn add_constraint(doc: &mut SketchDoc, ctype: &str, points: Vec<Value>, display_style: &str) -> Value {
    let id = doc.next_constraint_id();
    let mut raw = Map::new();
    raw.insert("id".to_string(), id.clone());
    raw.insert("type".to_string(), Value::String(ctype.to_string()));
    raw.insert("points".to_string(), Value::Array(points));
    raw.insert("labelX".to_string(), Value::from(0));
    raw.insert("labelY".to_string(), Value::from(0));
    raw.insert("displayStyle".to_string(), Value::String(display_style.to_string()));
    raw.insert("value".to_string(), Value::Null);
    raw.insert("valueNeedsSetup".to_string(), Value::Bool(true));
    doc.constraints.push(super::doc::SketchConstraint { raw });
    id
}

/// Swap the corner endpoint of line `line` for `with`.
fn retarget_line(doc: &mut SketchDoc, line: &Value, corner: &Value, with: &Value) {
    let key = id_key(corner);
    if let Some(geometry) = doc.geometry_mut(line) {
        for point in geometry.points.iter_mut() {
            if id_key(point) == key {
                *point = with.clone();
            }
        }
    }
}

/// Turn the corner into the virtual sharp: a construction point held on both
/// (now shortened) lines.
fn keep_virtual_sharp(doc: &mut SketchDoc, corner: &Value, on: [(&Value, &Value); 2], constraints: &mut Vec<Value>) {
    if let Some(point) = doc.point_mut(corner) {
        point.construction = true;
    }
    for (far, cut) in on {
        constraints.push(add_constraint(doc, "⏛", vec![far.clone(), cut.clone(), corner.clone()], ""));
    }
}

fn unit((x, y): (f64, f64)) -> (f64, f64) {
    let length = x.hypot(y);
    (x / length, y / length)
}

/// The corner's two unit directions (towards each line's far end), the angle
/// between them, and each line's length.
fn directions(corner: &Corner) -> Result<([(f64, f64); 2], f64, [f64; 2]), String> {
    let (px, py) = corner.corner;
    let vectors = corner.lines.each_ref().map(|(_, _, (x, y))| (x - px, y - py));
    let lengths = vectors.map(|(x, y)| x.hypot(y));
    let units = vectors.map(unit);
    let cross = units[0].0 * units[1].1 - units[0].1 * units[1].0;
    let dot = units[0].0 * units[1].0 + units[0].1 * units[1].1;
    let angle = cross.abs().atan2(dot);
    if angle <= 1e-6 || angle >= std::f64::consts::PI - 1e-6 {
        return Err("the two lines are parallel: there is no corner to round or cut".to_string());
    }
    Ok((units, angle, lengths))
}

/// Round the corner at point `corner` with a tangent arc of `radius`.
pub fn fillet_corner(doc: &mut SketchDoc, corner: &Value, radius: f64) -> Result<CornerEdit, String> {
    if !(radius.is_finite() && radius > 1e-9) {
        return Err("the fillet radius must be positive".to_string());
    }
    let shape = corner_lines(doc, corner)?;
    let (units, angle, lengths) = directions(&shape)?;
    let back = radius / (angle / 2.0).tan();
    let shorter = lengths[0].min(lengths[1]);
    if back >= shorter {
        return Err(format!(
            "the radius is too large for this corner: it needs {back:.6} along each line, the shorter is {shorter:.6} (largest radius {:.6})",
            shorter * (angle / 2.0).tan()
        ));
    }
    let (px, py) = shape.corner;
    let tangent = units.map(|(x, y)| (px + x * back, py + y * back));
    let bisector = unit((units[0].0 + units[1].0, units[0].1 + units[1].1));
    let reach = radius / (angle / 2.0).sin();
    let centre = (px + bisector.0 * reach, py + bisector.1 * reach);
    let [(first, first_far, _), (second, second_far, _)] = shape.lines;
    let first_cut = add_point(doc, tangent[0]);
    let second_cut = add_point(doc, tangent[1]);
    let centre_id = add_point(doc, centre);
    retarget_line(doc, &first, corner, &first_cut);
    retarget_line(doc, &second, corner, &second_cut);
    // The arc runs counter-clockwise from start to end: the short way round.
    let turn = (tangent[0].0 - centre.0) * (tangent[1].1 - centre.1) - (tangent[0].1 - centre.1) * (tangent[1].0 - centre.0);
    let (start, end) = if turn > 0.0 { (first_cut.clone(), second_cut.clone()) } else { (second_cut.clone(), first_cut.clone()) };
    let edge = add_geometry(doc, "arc", vec![centre_id.clone(), start.clone(), end]);
    let mut constraints = Vec::new();
    // Tangency at each joint is the radius standing square to the line there
    // (`⟂ [centre, cut, cut, far]`). The solver's tangent row (the centre's
    // distance to the line against a radius) leaves the tangent point free to
    // first order along its line, since every distance from the centre is
    // stationary at the foot: two such rows add no rank and the fillet would
    // read as two degrees free and two constraints redundant. The square
    // radius pins each tangent point; with the radius dimension and the arc's
    // implied equal-radius row it makes the arc tangent to both lines.
    constraints.push(add_constraint(doc, "⟂", vec![centre_id.clone(), first_cut.clone(), first_cut.clone(), first_far.clone()], ""));
    constraints.push(add_constraint(doc, "⟂", vec![centre_id.clone(), second_cut.clone(), second_cut.clone(), second_far.clone()], ""));
    constraints.push(add_constraint(doc, "⟺", vec![centre_id, start], "radius"));
    keep_virtual_sharp(doc, corner, [(&first_far, &first_cut), (&second_far, &second_cut)], &mut constraints);
    Ok(CornerEdit { edge, constraints })
}

/// Round several corners at once with one radius (the selection's Fillet). The
/// corners are taken in the order given, which is the order the user selected
/// them: the first one rounded keeps its radius dimension, and every later arc
/// drops its own and is tied to the first by an equal-radius constraint
/// (`⊜ [first centre, first start, centre, start]`), so editing the one
/// dimension resizes them all. A point that is not a corner of two lines is
/// skipped and returned with the reason.
pub fn fillet_corners(doc: &mut SketchDoc, corners: &[Value], radius: f64) -> (Vec<(Value, CornerEdit)>, Vec<(Value, String)>) {
    let mut done: Vec<(Value, CornerEdit)> = Vec::new();
    let mut refused = Vec::new();
    let mut first: Option<(Value, Value)> = None;
    for corner in corners {
        match fillet_corner(doc, corner, radius) {
            Ok(mut edit) => {
                let arc = doc.geometries.iter().find(|g| id_key(&g.id) == id_key(&edit.edge)).map(|g| g.points.clone());
                if let Some([centre, start, _end]) = arc.as_deref() {
                    match &first {
                        None => first = Some((centre.clone(), start.clone())),
                        Some((first_centre, first_start)) => {
                            // Trade this arc's own radius dimension for an equal radius to the first arc.
                            if let Some(radius_id) = edit.constraints.iter().find(|id| {
                                doc.constraints.iter().any(|c| {
                                    c.raw.get("id").map(id_key) == Some(id_key(id))
                                        && c.ctype() == Some("⟺")
                                        && c.raw.get("displayStyle").and_then(Value::as_str) == Some("radius")
                                })
                            }) {
                                let radius_id = radius_id.clone();
                                doc.constraints.retain(|c| c.raw.get("id").map(id_key) != Some(id_key(&radius_id)));
                                edit.constraints.retain(|id| id_key(id) != id_key(&radius_id));
                            }
                            let equal = add_constraint(
                                doc,
                                "⊜",
                                vec![first_centre.clone(), first_start.clone(), centre.clone(), start.clone()],
                                "",
                            );
                            edit.constraints.push(equal);
                        }
                    }
                }
                done.push((corner.clone(), edit));
            }
            Err(message) => refused.push((corner.clone(), message)),
        }
    }
    (done, refused)
}

/// Cut the corner at point `corner` with a straight line sized by `size`.
pub fn chamfer_corner(doc: &mut SketchDoc, corner: &Value, size: ChamferSize) -> Result<CornerEdit, String> {
    let shape = corner_lines(doc, corner)?;
    let (units, angle, lengths) = directions(&shape)?;
    let (first_back, second_back) = match size {
        ChamferSize::Equal { distance } => {
            if !(distance.is_finite() && distance > 1e-9) {
                return Err("the chamfer distance must be positive".to_string());
            }
            (distance, distance)
        }
        ChamferSize::DistanceAngle { distance, angle_degrees } => {
            if !(distance.is_finite() && distance > 1e-9) {
                return Err("the chamfer distance must be positive".to_string());
            }
            let cut = angle_degrees.to_radians();
            // The triangle corner, first cut point, second cut point: angle
            // `angle` at the corner, `cut` at the first cut point.
            let far = std::f64::consts::PI - angle - cut;
            if !(cut > 1e-6 && far > 1e-6) {
                return Err(format!(
                    "the chamfer angle must lie between 0 and {:.4} degrees at this corner",
                    (std::f64::consts::PI - angle).to_degrees()
                ));
            }
            (distance, distance * cut.sin() / far.sin())
        }
    };
    for (back, length) in [(first_back, lengths[0]), (second_back, lengths[1])] {
        if back >= length {
            return Err(format!("the chamfer is too large for this corner: it needs {back:.6} along a line {length:.6} long"));
        }
    }
    let (px, py) = shape.corner;
    let cuts = [(px + units[0].0 * first_back, py + units[0].1 * first_back), (px + units[1].0 * second_back, py + units[1].1 * second_back)];
    let [(first, first_far, _), (second, second_far, _)] = shape.lines;
    let first_cut = add_point(doc, cuts[0]);
    let second_cut = add_point(doc, cuts[1]);
    retarget_line(doc, &first, corner, &first_cut);
    retarget_line(doc, &second, corner, &second_cut);
    let edge = add_geometry(doc, "line", vec![first_cut.clone(), second_cut.clone()]);
    let mut constraints = Vec::new();
    constraints.push(add_constraint(doc, "⟺", vec![corner.clone(), first_cut.clone()], ""));
    match size {
        ChamferSize::Equal { .. } => {
            constraints.push(add_constraint(doc, "⇌", vec![corner.clone(), first_cut.clone(), corner.clone(), second_cut.clone()], ""));
        }
        ChamferSize::DistanceAngle { .. } => {
            // Between the cut and the first line towards the corner (the
            // virtual sharp): the angle asked for.
            constraints.push(add_constraint(doc, "∠", vec![first_cut.clone(), corner.clone(), first_cut.clone(), second_cut.clone()], ""));
        }
    }
    keep_virtual_sharp(doc, corner, [(&first_far, &first_cut), (&second_far, &second_cut)], &mut constraints);
    Ok(CornerEdit { edge, constraints })
}

