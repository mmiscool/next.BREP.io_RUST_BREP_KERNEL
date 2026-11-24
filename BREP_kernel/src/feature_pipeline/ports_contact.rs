//! A connection point's CONTACT BODY: a pin that is a whole solid of the part
//! (an XLR pin, a barrel jack's centre pin or sleeve, a JST crimp, a square
//! header post, a spade blade, a pin bent along a curve), which a face pick
//! cannot express ("A contact body").
//!
//! The body is one more SEAT source, below the explicit references: it
//! supplies the origin when `pointRef` does not, and the direction when
//! neither `directionRef` nor `pointRef` does. What it supplies is its two
//! ENDS, each with the direction a wire leaves along there, found by the
//! first rule that applies:
//!
//! * the AXIS rule: the body's axis-bearing analytic faces (cylinder, cone,
//!   torus, general revolution) grouped by collinear axis, and exactly one
//!   group. The ends are the axis points at the extremes of the body's extent
//!   along it (its edges, and a sphere face centred on the axis, so a rounded
//!   tip ends at its pole); the wire leaves along the axis, out of the body;
//! * the END-CAP rule, for a body with no single axis (a square post, a
//!   blade, a pin swept along a curve, a pin with a cross hole): its TERMINAL
//!   faces are the planar faces the whole body lies behind, and the tip of a
//!   rounded or pointed end (a sphere's pole, or the axis point at the end of
//!   a fillet's torus or a cone). A flat end opposite a tip is read on the
//!   tip's axis line, so a scooped solder cup's off-centre half-ring still
//!   attaches on the pin's axis. The
//!   two ends are the pair of terminal faces farthest apart, when no other
//!   pair comes close (a cube has three such pairs and no distinct ends). A
//!   body whose only planar faces are two caps (a pin swept round a bend of
//!   more than a right angle) takes those two, unless both lie at one end (a
//!   cup's floor behind its rim). A wire attaches at a cap's
//!   centroid and leaves along its outward normal, which for a swept pin is
//!   the path's end tangent; on a straight round pin the two rules agree;
//! * the DIRECTION rule, for a body with no distinct ends (a cube, a block
//!   terminal) when the point names a Direction (`directionRef`: a face's
//!   normal, an edge, an axis, a datum): the end is the body's extreme along
//!   that direction (the centroid of a face normal to it, else the middle of
//!   the extreme points) and the wire leaves along it. With no Direction such
//!   a body is refused by name, and the message says to pick one.
//!
//! The WIRE END is the end farther from the centre of the rest of the part,
//! or the `+axis` end (the chord between the ends in its canonical sense:
//! largest component positive) when the body is the only solid or the two
//! are equally far; under the direction rule it is the end the Direction
//! points to. `reverseDirection` takes the other one. The wire leaves OUTWARD
//! from the chosen end.
//!
//! [`contact_body_solid`] and [`contact_body_end`] are public so the board's
//! pad check finds the same solid and the same end the router does.

use serde::{Deserialize, Serialize};

use super::{SceneMap, SelectionGeometry, Vec3};

/// Samples per edge for the axial extent. The extremes of a cylinder, cone or
/// planar face along the axis lie on its boundary, so the edges carry them; a
/// circle's axial coordinate is constant, so any sample is exact for it.
const EDGE_SAMPLES: usize = 16;
/// Grid samples per direction over a sphere face's surface: a rounded tip's
/// pole is at a domain boundary, which the grid includes exactly.
const SPHERE_SAMPLES: usize = 16;
/// Directions closer than this (|d1 x d2|) are parallel.
const PARALLEL: f64 = 1e-6;
/// A terminal face lies within this of the body's extreme along its normal,
/// scaled by the body's size (edge samples are chords of curved edges).
const TERMINAL_TOLERANCE: f64 = 1e-4;
/// Two pairs of terminal faces whose distances are within this ratio are a
/// tie: the body has no distinct ends.
const DISTINCT_ENDS: f64 = 0.85;
/// Lengths below this, scaled by the body's size, are the same point.
const RELATIVE_TOLERANCE: f64 = 1e-6;

/// Which rule found a contact body's ends (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactRule {
    /// The body's one axis.
    Axis,
    /// Its two terminal faces.
    Caps,
    /// The point's Direction, on a body with no distinct ends.
    Direction,
}

impl ContactRule {
    /// The word the report shows.
    pub fn as_str(self) -> &'static str {
        match self {
            ContactRule::Axis => "axis",
            ContactRule::Caps => "caps",
            ContactRule::Direction => "direction",
        }
    }
}

/// What a contact body derives: its two ends, the direction a wire leaves
/// along at each, and the end a wire leaves from.
#[derive(Debug, Clone, Copy)]
pub struct ContactAxis {
    /// The axis direction in its canonical sense (largest component
    /// positive): the body's axis, or under the cap rule the chord from the
    /// `-axis` end to the `+axis` end, or the Direction.
    pub axis: Vec3,
    /// The `+axis` end.
    pub plus_end: Vec3,
    /// The `-axis` end.
    pub minus_end: Vec3,
    /// The direction a wire leaves along at the `+axis` end (out of the body).
    pub plus_outward: Vec3,
    /// The direction a wire leaves along at the `-axis` end.
    pub minus_outward: Vec3,
    /// Whether the wire leaves from the `+axis` end.
    pub wire_at_plus: bool,
    /// The rule that found the ends.
    pub rule: ContactRule,
}

impl ContactAxis {
    /// The end a wire attaches at.
    pub fn wire_end(&self) -> Vec3 {
        if self.wire_at_plus {
            self.plus_end
        } else {
            self.minus_end
        }
    }

    /// The other end (the mating end, on a pin whose wire side is right).
    pub fn other_end(&self) -> Vec3 {
        if self.wire_at_plus {
            self.minus_end
        } else {
            self.plus_end
        }
    }

    /// The direction a wire leaves along: OUT of the body at the wire end.
    pub fn outward(&self) -> Vec3 {
        if self.wire_at_plus {
            self.plus_outward
        } else {
            self.minus_outward
        }
    }
}

/// The contact body as the tail's report shows it, so a wrong default end is
/// visible next to the one-click flip (`reverseDirection`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PortContactRow {
    /// The solid named by `contactBody`.
    pub body: String,
    /// `"+axis"` or `"-axis"`: which end the wire took, in the axis's
    /// canonical sense.
    pub end: String,
    /// The canonical axis direction.
    pub axis: [f64; 3],
    /// Where the wire attaches.
    pub wire_end: [f64; 3],
    /// The other end.
    pub other_end: [f64; 3],
    /// Whether the body SEATED the origin (no `pointRef`). With a `pointRef`
    /// the body is reported, but the origin is the reference's.
    pub seats_origin: bool,
    /// `"axis"`, `"caps"` or `"direction"`: the rule that found the ends.
    #[serde(default)]
    pub rule: String,
}

/// The resident solid a contact body names, by the same lookup every seat
/// reference uses ([`SceneMap::resolve_solid`]): its handle, or `None` when
/// the scene holds no solid of that name.
pub fn contact_body_solid(scene: &SceneMap, name: &str) -> Option<u32> {
    let name = name.trim();
    if name.is_empty() {
        return None;
    }
    scene.resolve_solid(name)
}

/// The ends and the wire end of the contact body `name`, by the axis or the
/// end-cap rule. `reverse` (a point's `reverseDirection`) takes the other end.
/// A body with no distinct ends is refused; [`contact_body_end`] takes a
/// Direction for it.
pub fn contact_body_axis(scene: &SceneMap, name: &str, reverse: bool) -> Result<ContactAxis, String> {
    contact_body_end(scene, name, reverse, None)
}

/// The ends and the wire end of the contact body `name`. `reverse` (a point's
/// `reverseDirection`) takes the other end; `direction` (the point's resolved
/// `directionRef`) picks the end of a body that has no distinct ends.
///
/// `Err` names why nothing is derived: no such solid, a body with no length,
/// or one with no distinct ends and no Direction to pick one by.
pub fn contact_body_end(
    scene: &SceneMap,
    name: &str,
    reverse: bool,
    direction: Option<Vec3>,
) -> Result<ContactAxis, String> {
    let name = name.trim();
    let handle = contact_body_solid(scene, name)
        .ok_or_else(|| format!("contact body '{name}' is not a solid of this part"))?;
    let (ends, scale) = crate::with_registered_solid_str(handle, |solid| body_ends(solid, name, direction))?;
    let mut wire_at_plus = match ends.toward {
        // The direction rule: the end the Direction points to.
        Some(toward) => toward,
        // Otherwise the end farther from the rest of the part.
        None => {
            let mut plus_side = true;
            if let Some(centre) = rest_of_part_centre(scene, handle) {
                let plus = ends.plus_end.sub(centre).length();
                let minus = ends.minus_end.sub(centre).length();
                if (plus - minus).abs() > RELATIVE_TOLERANCE * scale {
                    plus_side = plus > minus;
                }
            }
            plus_side
        }
    };
    if reverse {
        wire_at_plus = !wire_at_plus;
    }
    Ok(ContactAxis {
        axis: ends.axis,
        plus_end: ends.plus_end,
        minus_end: ends.minus_end,
        plus_outward: ends.plus_outward,
        minus_outward: ends.minus_outward,
        wire_at_plus,
        rule: ends.rule,
    })
}

/// A body's two ends before the wire side is chosen.
struct Ends {
    axis: Vec3,
    plus_end: Vec3,
    minus_end: Vec3,
    plus_outward: Vec3,
    minus_outward: Vec3,
    rule: ContactRule,
    /// Under the direction rule, whether the Direction points to the `+axis`
    /// end; `None` leaves the side to the rest of the part.
    toward: Option<bool>,
}

/// The body's ends by the first rule that applies, and its size scale.
fn body_ends(solid: &crate::BrepSolid, name: &str, direction: Option<Vec3>) -> Result<(Ends, f64), String> {
    let scale = solid_scale(solid);
    let points = body_points(solid);
    if points.is_empty() {
        return Err(format!("contact body '{name}' has no geometry to measure"));
    }
    let tolerance = RELATIVE_TOLERANCE * scale;

    // Every axis-bearing face's axis, grouped by collinearity; planar faces
    // and sphere faces for the end caps.
    let mut groups: Vec<(Vec3, Vec3)> = Vec::new();
    let mut spheres: Vec<(&crate::FaceRecord, Vec3, f64)> = Vec::new();
    let mut planes: Vec<(&crate::FaceRecord, Vec3)> = Vec::new();
    // Round faces that are not cylinders (a fillet's torus, a cone, a general
    // revolution): a rounded or pointed tip, with its axis.
    let mut round_tips: Vec<(&crate::FaceRecord, Vec3, Vec3)> = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            match crate::resolve_face_selection(solid, face.id) {
                Ok(SelectionGeometry::Axis { origin, direction, radius }) => {
                    let Ok(direction) = direction.normalized() else { continue };
                    let direction = canonical(direction);
                    if radius.is_none() {
                        round_tips.push((face, origin, direction));
                    }
                    let known = groups
                        .iter()
                        .any(|(o, d)| collinear(*o, *d, origin, direction, tolerance));
                    if !known {
                        groups.push((origin, direction));
                    }
                }
                Ok(SelectionGeometry::Sphere { center, radius }) => spheres.push((face, center, radius)),
                Ok(SelectionGeometry::Plane { normal, .. }) => {
                    if let Ok(normal) = normal.normalized() {
                        planes.push((face, normal));
                    }
                }
                _ => {}
            }
        }
    }

    if let [(origin, axis)] = groups.as_slice() {
        let (origin, axis) = (*origin, *axis);
        let (low, high) = axial_extent(&points, &spheres, origin, axis, tolerance);
        if high - low <= RELATIVE_TOLERANCE * scale {
            return Err(format!("contact body '{name}' has no length along its axis"));
        }
        let ends = Ends {
            axis,
            plus_end: origin.add(axis.scale(high)),
            minus_end: origin.add(axis.scale(low)),
            plus_outward: axis,
            minus_outward: axis.scale(-1.0),
            rule: ContactRule::Axis,
            toward: None,
        };
        return Ok((ends, scale));
    }

    // The end caps: planar faces the whole body lies behind, and the tips of
    // rounded or pointed ends. A tip carries its axis line, so the end at the
    // other side can be read on the same line.
    let slack = TERMINAL_TOLERANCE * scale;
    let behind = |centre: Vec3, outward: Vec3| points.iter().all(|p| p.sub(centre).dot(outward) <= slack);
    let body_centre = bbox_centre(&points);
    let mut caps: Vec<Cap> = Vec::new();
    for (face, normal) in &planes {
        if let Some(centre) = boundary_centroid(solid, face) {
            if behind(centre, *normal) {
                caps.push(Cap { point: centre, outward: *normal, line: None });
            }
        }
    }
    for (face, center, _) in &spheres {
        if let Some(pole) = sphere_tip(face, body_centre) {
            if let Ok(outward) = pole.sub(*center).normalized() {
                if behind(pole, outward) {
                    caps.push(Cap { point: pole, outward, line: Some((*center, outward)) });
                }
            }
        }
    }
    for (face, origin, axis) in &round_tips {
        if let Some(tip) = axial_tip(face, *origin, *axis, body_centre) {
            if behind(tip.point, tip.outward) {
                caps.push(tip);
            }
        }
    }
    // One end found twice (a partly filleted tip: its torus's top and the
    // flat disc left inside it are the same point) is one end.
    let mut ends: Vec<Cap> = Vec::new();
    for cap in caps {
        match ends.iter_mut().find(|end| end.point.sub(cap.point).length() <= slack) {
            Some(end) => {
                if end.line.is_none() {
                    end.line = cap.line;
                }
            }
            None => ends.push(cap),
        }
    }
    if ends.len() < 2 && planes.len() == 2 {
        // A pin swept round a sharp bend: its two caps are its only planes,
        // unless both lie at ONE end (a cup's floor behind its rim), which
        // says nothing about where the other end is.
        let two: Vec<Cap> = planes
            .iter()
            .filter_map(|(face, normal)| {
                boundary_centroid(solid, face).map(|centre| Cap { point: centre, outward: *normal, line: None })
            })
            .collect();
        if two.len() == 2 && !same_end(&two[0], &two[1]) {
            ends = two;
        }
    }
    if let Some((a, b)) = distinct_ends(&ends, tolerance) {
        let (mut a, mut b) = (ends[a], ends[b]);
        // A flat end opposite a tip is read on the tip's line: a scooped
        // solder cup's half-ring has its centroid off the pin's axis.
        a.point = on_line_of(a, &b);
        b.point = on_line_of(b, &a);
        if let Ok(chord) = a.point.sub(b.point).normalized() {
            let axis = canonical(chord);
            let (plus, minus) = if a.point.dot(axis) >= b.point.dot(axis) { (a, b) } else { (b, a) };
            let ends = Ends {
                axis,
                plus_end: plus.point,
                minus_end: minus.point,
                plus_outward: plus.outward,
                minus_outward: minus.outward,
                rule: ContactRule::Caps,
                toward: None,
            };
            return Ok((ends, scale));
        }
    }

    // No distinct ends: the point's Direction picks one.
    let Some(direction) = direction.and_then(|d| d.normalized().ok()) else {
        return Err(format!(
            "contact body '{name}' has no distinct ends to take a wire end from; pick a Direction (a face, an edge or an axis) to say which way the wire leaves"
        ));
    };
    let axis = canonical(direction);
    let extreme = |toward: Vec3| -> Vec3 {
        // A terminal face normal to the direction: its centroid.
        for (face, normal) in &planes {
            if normal.dot(toward) >= 1.0 - PARALLEL {
                if let Some(centre) = boundary_centroid(solid, face) {
                    if behind(centre, *normal) {
                        return centre;
                    }
                }
            }
        }
        // Else the middle of the points at the extreme.
        let top = points.iter().map(|p| p.dot(toward)).fold(f64::NEG_INFINITY, f64::max);
        let at_top: Vec<Vec3> = points.iter().copied().filter(|p| top - p.dot(toward) <= slack).collect();
        bbox_centre(&at_top)
    };
    let plus_end = extreme(axis);
    let minus_end = extreme(axis.scale(-1.0));
    if plus_end.sub(minus_end).dot(axis) <= RELATIVE_TOLERANCE * scale {
        return Err(format!("contact body '{name}' has no length along its Direction"));
    }
    let ends = Ends {
        axis,
        plus_end,
        minus_end,
        plus_outward: axis,
        minus_outward: axis.scale(-1.0),
        rule: ContactRule::Direction,
        toward: Some(direction.dot(axis) > 0.0),
    };
    Ok((ends, scale))
}

/// The body's extent `[low, high]` along an axis through `origin`, measured
/// from it: its vertices, its edges, and any sphere face centred on the axis
/// (a rounded tip, whose pole is the end).
fn axial_extent(
    points: &[Vec3],
    spheres: &[(&crate::FaceRecord, Vec3, f64)],
    origin: Vec3,
    axis: Vec3,
    tolerance: f64,
) -> (f64, f64) {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    let mut include = |point: Vec3| {
        let t = point.sub(origin).dot(axis);
        low = low.min(t);
        high = high.max(t);
    };
    for point in points {
        include(*point);
    }
    for (face, center, _) in spheres {
        if off_axis(origin, axis, *center) > tolerance {
            continue;
        }
        for point in sphere_samples(face) {
            include(point);
        }
    }
    (low, high)
}

/// The body's vertices and samples along its edges: the extremes of a
/// cylinder, cone or planar face along any direction lie on its boundary.
fn body_points(solid: &crate::BrepSolid) -> Vec<Vec3> {
    let mut points: Vec<Vec3> = solid.vertices.iter().map(|vertex| vertex.point).collect();
    for edge in &solid.edges {
        if edge.degenerate {
            continue;
        }
        for i in 0..=EDGE_SAMPLES {
            let t = edge.t0 + (edge.t1 - edge.t0) * (i as f64 / EDGE_SAMPLES as f64);
            if let Ok(point) = edge.curve.evaluate(t) {
                points.push(point);
            }
        }
    }
    points
}

/// A grid of points over a sphere face (its pole lies on a domain boundary,
/// which the grid includes exactly).
fn sphere_samples(face: &crate::FaceRecord) -> Vec<Vec3> {
    let (Ok(u), Ok(v)) = (face.surface.domain_u(), face.surface.domain_v()) else { return Vec::new() };
    let mut out = Vec::new();
    for i in 0..=SPHERE_SAMPLES {
        for j in 0..=SPHERE_SAMPLES {
            let s = u[0] + (u[1] - u[0]) * (i as f64 / SPHERE_SAMPLES as f64);
            let t = v[0] + (v[1] - v[0]) * (j as f64 / SPHERE_SAMPLES as f64);
            if let Ok(point) = face.surface.evaluate(s, t) {
                out.push(point);
            }
        }
    }
    out
}

/// A rounded tip's pole: the sphere face's point farthest from the body's
/// centre.
fn sphere_tip(face: &crate::FaceRecord, body_centre: Vec3) -> Option<Vec3> {
    sphere_samples(face)
        .into_iter()
        .max_by(|a, b| a.sub(body_centre).length().total_cmp(&b.sub(body_centre).length()))
}

/// The centroid of a face's boundary, each edge sample weighted by the length
/// it stands for: the centre of a cap's rectangle, disc or ring.
fn boundary_centroid(solid: &crate::BrepSolid, face: &crate::FaceRecord) -> Option<Vec3> {
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    let mut length = 0.0;
    for edge_id in face.loops.iter().flat_map(|l| l.coedges.iter().map(|c| c.edge_id)) {
        let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id) else { continue };
        if edge.degenerate {
            continue;
        }
        let samples: Vec<Vec3> = (0..=EDGE_SAMPLES)
            .filter_map(|i| edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * (i as f64 / EDGE_SAMPLES as f64)).ok())
            .collect();
        for pair in samples.windows(2) {
            let piece = pair[1].sub(pair[0]).length();
            sum = sum.add(pair[0].add(pair[1]).scale(0.5 * piece));
            length += piece;
        }
    }
    (length > 0.0).then(|| sum.scale(1.0 / length))
}

/// The pair of caps farthest apart, when no other pair comes within
/// [`DISTINCT_ENDS`] of it (a cube's three opposite pairs tie).
fn distinct_ends(caps: &[Cap], tolerance: f64) -> Option<(usize, usize)> {
    let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
    for i in 0..caps.len() {
        for j in i + 1..caps.len() {
            pairs.push((caps[i].point.sub(caps[j].point).length(), i, j));
        }
    }
    pairs.sort_by(|a, b| b.0.total_cmp(&a.0));
    let &(best, i, j) = pairs.first()?;
    if best <= tolerance {
        return None;
    }
    let rival = pairs.iter().skip(1).any(|&(d, k, l)| (k, l) != (i, j) && d >= DISTINCT_ENDS * best);
    (!rival).then_some((i, j))
}

/// One candidate end: where a wire attaches, the direction it leaves along,
/// and for a tip its axis line (a point on it and its direction).
#[derive(Debug, Clone, Copy)]
pub(super) struct Cap {
    pub(super) point: Vec3,
    pub(super) outward: Vec3,
    pub(super) line: Option<(Vec3, Vec3)>,
}

/// A round non-cylindrical face's tip (a fillet's torus, a cone), when it is
/// at an end: the axis point at the face's extreme along its axis on the side
/// away from the body's centre, leaving along the axis.
fn axial_tip(face: &crate::FaceRecord, origin: Vec3, axis: Vec3, body_centre: Vec3) -> Option<Cap> {
    let samples = sphere_samples(face);
    if samples.is_empty() {
        return None;
    }
    let along: Vec<f64> = samples.iter().map(|p| p.sub(origin).dot(axis)).collect();
    let mean = along.iter().sum::<f64>() / along.len() as f64;
    let centre = body_centre.sub(origin).dot(axis);
    let (t, outward) = if mean >= centre {
        (along.iter().copied().fold(f64::NEG_INFINITY, f64::max), axis)
    } else {
        (along.iter().copied().fold(f64::INFINITY, f64::min), axis.scale(-1.0))
    };
    Some(Cap { point: origin.add(axis.scale(t)), outward, line: Some((origin, axis)) })
}

/// `end`'s point, moved onto `other`'s tip line when `end` faces along that
/// line (a flat end opposite a tip); unchanged otherwise.
fn on_line_of(end: Cap, other: &Cap) -> Vec3 {
    let (Some((origin, direction)), None) = (other.line, end.line) else { return end.point };
    let Ok(direction) = direction.normalized() else { return end.point };
    if end.outward.dot(direction).abs() < 1.0 - PARALLEL {
        return end.point;
    }
    origin.add(direction.scale(end.point.sub(origin).dot(direction)))
}

/// Whether two flat caps lie at ONE end of a body: they face the same way and
/// one stands behind the other along that way (a cup's floor behind its rim),
/// rather than side by side (the two legs of a hairpin).
pub(super) fn same_end(a: &Cap, b: &Cap) -> bool {
    let apart = a.point.sub(b.point);
    let length = apart.length();
    a.outward.dot(b.outward) > 0.9 && (length == 0.0 || apart.dot(a.outward).abs() >= 0.9 * length)
}

/// The centre of the points' bounding box.
fn bbox_centre(points: &[Vec3]) -> Vec3 {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in points {
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    low.add(high).scale(0.5)
}

/// The centre of the bounding box of every OTHER resident solid of the scene:
/// what "the rest of the part" a pin sticks out of is. `None` when the body is
/// the only solid.
fn rest_of_part_centre(scene: &SceneMap, body: u32) -> Option<Vec3> {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut any = false;
    let mut handles: Vec<u32> = scene.solids.values().copied().filter(|h| *h != body).collect();
    handles.sort_unstable();
    handles.dedup();
    for handle in handles {
        let Ok((a, b)) = crate::feature_pipeline::pmi::resolve::solid_bbox(handle) else { continue };
        low = Vec3::new(low.x.min(a.x), low.y.min(a.y), low.z.min(a.z));
        high = Vec3::new(high.x.max(b.x), high.y.max(b.y), high.z.max(b.z));
        any = true;
    }
    any.then(|| low.add(high).scale(0.5))
}

/// The direction with its largest component positive: one sense for an axis,
/// whichever way a face happens to report it.
fn canonical(direction: Vec3) -> Vec3 {
    let [x, y, z] = [direction.x, direction.y, direction.z];
    let largest = if x.abs() >= y.abs() && x.abs() >= z.abs() {
        x
    } else if y.abs() >= z.abs() {
        y
    } else {
        z
    };
    if largest < 0.0 {
        direction.scale(-1.0)
    } else {
        direction
    }
}

/// Whether two axes are one line.
fn collinear(o1: Vec3, d1: Vec3, o2: Vec3, d2: Vec3, tolerance: f64) -> bool {
    d1.cross(d2).length() <= PARALLEL && off_axis(o1, d1, o2) <= tolerance.max(1e-9)
}

/// How far `point` is from the line through `origin` along unit `axis`.
fn off_axis(origin: Vec3, axis: Vec3, point: Vec3) -> f64 {
    let offset = point.sub(origin);
    offset.sub(axis.scale(offset.dot(axis))).length()
}

/// A size for the body: its vertex bounding-box diagonal, at least 1.
fn solid_scale(solid: &crate::BrepSolid) -> f64 {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in &solid.vertices {
        let p = vertex.point;
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    if solid.vertices.is_empty() {
        return 1.0;
    }
    high.sub(low).length().max(1.0)
}
