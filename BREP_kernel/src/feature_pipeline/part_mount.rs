//! A part's MOUNT FRAME: how the part sits on its footprint, kept beside its
//! features rather than in them.
//!
//! The part document's top-level `mount` block is a rigid pose in the shape an
//! ACOMP `transform` takes, `{translate (mm), rotateEulerDeg (intrinsic XYZ)}`.
//! It maps the part's model coordinates to its footprint's:
//! `p_footprint = M · p_model`. The footprint frame is the board's: top copper at
//! z = 0, millimetres with y up.
//!
//! The mount always moves the WHOLE part (every solid, every connection point
//! and its seat). Nothing selects what it applies to. It is applied in ONE
//! place: the ACOMP feature poses an instance's members, ports and nested
//! poses by M before the instance pose. So a part has one frame wherever it is
//! placed, and the instance's own pose is still exactly the one it was given.
//! The solver writes that pose back, so it must never carry M.
//!
//! No block means identity. An identity mount is never written
//! (`mount_block_of` says `None` for it), so a part with no mount and every
//! KiCad import stay byte-identical.

use serde_json::Value;

use crate::AffineTransform;

/// The part document's top-level key for its mount frame.
pub const MOUNT_BLOCK: &str = "mount";

/// How far from identity a mount must be to be written at all: a millionth of
/// a millimetre in translation, a millionth in each rotation entry.
const IDENTITY_TOLERANCE: f64 = 1e-6;

/// The mount frame `document` declares, `None` when it has none.
///
/// A block that is not a rigid pose is an error that names what is wrong:
/// an unknown key, a value that is not three numbers, or a non-finite number.
/// A mount that silently read as identity would put the part back in the wrong
/// frame with no word said.
pub fn part_mount(document: &Value) -> Result<Option<AffineTransform>, String> {
    let Some(block) = document.get(MOUNT_BLOCK).filter(|block| !block.is_null()) else {
        return Ok(None);
    };
    mount_from_block(block).map(Some)
}

/// Read one `mount` block as the rigid pose it is.
pub fn mount_from_block(block: &Value) -> Result<AffineTransform, String> {
    let object = block
        .as_object()
        .ok_or("mount: must be an object {translate, rotateEulerDeg}")?;
    if let Some(key) = object.keys().find(|key| !matches!(key.as_str(), "translate" | "rotateEulerDeg")) {
        return Err(format!("mount: unknown key '{key}' (a mount has translate and rotateEulerDeg)"));
    }
    for key in ["translate", "rotateEulerDeg"] {
        let Some(value) = object.get(key) else { continue };
        let numbers = value.as_array().filter(|values| values.len() == 3);
        let finite = numbers.is_some_and(|values| values.iter().all(|v| v.as_f64().is_some_and(f64::is_finite)));
        if !finite {
            return Err(format!("mount: {key} must be three finite numbers, found {value}"));
        }
    }
    super::assembly::pose_params_to_transform(block)
}

/// The `mount` block to store for `transform`: `None` for identity (the key is
/// then removed, never written as a default), otherwise the pose in the
/// shape [`part_mount`] reads back.
pub fn mount_block_of(transform: &AffineTransform) -> Option<Value> {
    if is_identity(transform) {
        return None;
    }
    Some(super::assembly::transform_to_pose_params(transform))
}

/// Whether `transform` is the identity, to `IDENTITY_TOLERANCE` (1e-6).
pub fn is_identity(transform: &AffineTransform) -> bool {
    transform
        .elements
        .iter()
        .enumerate()
        .all(|(index, value)| (value - if index % 5 == 0 { 1.0 } else { 0.0 }).abs() <= IDENTITY_TOLERANCE)
}

fn translation(x: f64, y: f64, z: f64) -> AffineTransform {
    AffineTransform { elements: [1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0] }
}

fn identity() -> AffineTransform {
    translation(0.0, 0.0, 0.0)
}

/// `v` moved by `transform`'s linear part (a direction, not a point).
fn linear(transform: &AffineTransform, v: [f64; 3]) -> [f64; 3] {
    let e = &transform.elements;
    [0, 1, 2].map(|row| e[row * 4] * v[0] + e[row * 4 + 1] * v[1] + e[row * 4 + 2] * v[2])
}

/// The SHORTEST rotation taking direction `n` to −z, about the origin; a
/// half turn about x when `n` is straight up. `None` for no direction.
fn turn_down(n: [f64; 3]) -> Option<AffineTransform> {
    let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if !(length.is_finite() && length > 1e-12) {
        return None;
    }
    let n = n.map(|c| c / length);
    // Rotation taking n to d = (0, 0, -1): axis n × d, cos = n · d.
    let (axis, cos) = ([-n[1], n[0], 0.0], -n[2]);
    let sin = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
    Some(if sin <= 1e-12 && cos > 0.0 {
        identity()
    } else if sin <= 1e-12 {
        AffineTransform { elements: [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0] }
    } else {
        let k = axis.map(|c| c / sin);
        let v = 1.0 - cos;
        let r = |i: usize, j: usize| {
            let cross = match (i, j) {
                (0, 1) => -k[2],
                (1, 0) => k[2],
                (0, 2) => k[1],
                (2, 0) => -k[1],
                (1, 2) => -k[0],
                (2, 1) => k[0],
                _ => 0.0,
            };
            (if i == j { cos } else { 0.0 }) + sin * cross + v * k[i] * k[j]
        };
        AffineTransform {
            elements: [r(0, 0), r(0, 1), r(0, 2), 0.0, r(1, 0), r(1, 1), r(1, 2), 0.0, r(2, 0), r(2, 1), r(2, 2), 0.0, 0.0, 0.0, 0.0, 1.0],
        }
    })
}

/// The mount `current` (identity when `None`) moved IN THE FOOTPRINT'S PLANE
/// so that `point` (model coordinates) lands on the footprint origin. Its
/// height is left alone: that is the seating plane's to say.
pub fn mount_origin_to(current: Option<&AffineTransform>, point: [f64; 3]) -> Result<AffineTransform, String> {
    let current = current.copied().unwrap_or_else(identity);
    let at = current.point(crate::Vec3::new(point[0], point[1], point[2]));
    translation(-at.x, -at.y, 0.0).compose(&current)
}

/// The mount `current` (identity when `None`) turned and lifted so the plane
/// through `origin` with OUTWARD normal `normal` (both model coordinates) is
/// the seating plane: the normal points down (−z) and the plane is at z = 0.
///
/// The turn is the SHORTEST one taking the mounted normal to −z, about the
/// footprint origin, so a face that is already nearly down keeps the part's
/// in-plane angle. A face facing straight up turns a half turn about x.
pub fn mount_seated_on(
    current: Option<&AffineTransform>,
    origin: [f64; 3],
    normal: [f64; 3],
) -> Result<AffineTransform, String> {
    let current = current.copied().unwrap_or_else(identity);
    let rotation = turn_down(linear(&current, normal)).ok_or("the picked face has no normal to seat on")?;
    let turned = rotation.compose(&current)?;
    let at = turned.point(crate::Vec3::new(origin[0], origin[1], origin[2]));
    translation(0.0, 0.0, -at.z).compose(&turned)
}

/// The plane a picked name stands for, against the scene `prefix` builds:
/// `(origin, outward unit normal)`. A PLANAR face gives its own plane, its
/// normal pointing out of the material; a datum plane gives its frame's z.
/// Anything else is refused by name: a seating plane must be a plane.
pub fn reference_plane(prefix: &[crate::feature_pipeline::FeatureResult], name: &str) -> Result<([f64; 3], [f64; 3]), String> {
    use crate::feature_pipeline::pmi::resolve::resolve_reference;
    use crate::{resolve_face_selection, SelectionGeometry};
    let name = name.trim();
    if name.is_empty() {
        return Err("nothing was picked".into());
    }
    let mut scene = crate::feature_pipeline::SceneMap::default();
    for result in prefix {
        scene.apply(result);
    }
    let geometry = match scene.resolve_face(name) {
        Some(face) => crate::with_registered_solid_str(face.handle, |solid| {
            resolve_face_selection(solid, face.face_id).map_err(|error| format!("face '{name}': {error:?}"))
        })?,
        None => resolve_reference(&scene, name)?,
    };
    match geometry {
        SelectionGeometry::Plane { origin, normal } => {
            let out = ([origin.x, origin.y, origin.z], [normal.x, normal.y, normal.z]);
            match out.0.iter().chain(out.1.iter()).all(|c| c.is_finite()) {
                true => Ok(out),
                false => Err(format!("'{name}' did not evaluate to a finite plane")),
            }
        }
        _ => Err(format!("'{name}' is not flat: pick a planar face (the part's seating face) or a datum plane")),
    }
}

/// Wire directions further than this from the pins' mean do not agree on a
/// seating plane.
const ALIGN_TILT_AGREEMENT_DEG: f64 = 5.0;
/// A proper fit leaving less than this (RMS, mm) is never second-guessed by
/// the mirrored one.
const ALIGN_MIRROR_FLOOR: f64 = 0.05;

/// One seated pcb point for [`mount_aligned_to_footprint`], in MODEL
/// coordinates as the part builds it.
#[derive(Debug, Clone, PartialEq)]
pub struct AlignSeat {
    /// The pin number (the point's name), which picks its pads.
    pub pin: String,
    /// Where the point resolved (the ports report's position).
    pub position: [f64; 3],
    /// The way its wire leaves the part, which is into the board.
    pub direction: [f64; 3],
    /// Its contact body's points (a display mesh's vertices), whose lowest
    /// is the pin's bottom; empty for a point seated by reference, whose
    /// bottom is its position.
    pub support: Vec<[f64; 3]>,
}

/// One pad for [`mount_aligned_to_footprint`]: its number and centre in the
/// footprint frame (mm, y up), and whether it is surface mount (no drill).
#[derive(Debug, Clone, PartialEq)]
pub struct AlignPad {
    pub number: String,
    pub at: [f64; 2],
    pub smd: bool,
}

/// What Align to footprint did: the new mount, the leftover, and what it
/// wants said (a rotation it kept, a height it could not set, a mirror).
#[derive(Debug, Clone)]
pub struct AlignReport {
    pub mount: AffineTransform,
    /// The largest and the RMS pin-to-pad distance in the board's plane, mm.
    pub largest: f64,
    pub rms: f64,
    /// The pin with the largest distance.
    pub worst: String,
    /// How many seats were measured for the leftover.
    pub pins: usize,
    pub notes: Vec<String>,
}

/// Whether `transform` turns the model's z out of the board's z: a tilt
/// someone asked for (a face pick, Y up → Z up), not a turn in the plane.
fn has_tilt(transform: &AffineTransform) -> bool {
    let z = linear(transform, [0.0, 0.0, 1.0]);
    (z[0] * z[0] + z[1] * z[1]).sqrt() > IDENTITY_TOLERANCE || z[2] < 0.0
}

/// The rigid 2D fit (rotation forced proper) of `from` onto `to`, with its
/// summed square leftover: `(angle, translation, squares)`. With one pair it
/// is a translation only.
fn fit_2d(from: &[[f64; 2]], to: &[[f64; 2]]) -> (f64, [f64; 2], f64) {
    let n = from.len() as f64;
    let centre = |points: &[[f64; 2]]| {
        let sum = points.iter().fold([0.0, 0.0], |s, p| [s[0] + p[0], s[1] + p[1]]);
        [sum[0] / n, sum[1] / n]
    };
    let (p, q) = (centre(from), centre(to));
    let (mut dot, mut cross) = (0.0, 0.0);
    for (a, b) in from.iter().zip(to) {
        let (a, b) = ([a[0] - p[0], a[1] - p[1]], [b[0] - q[0], b[1] - q[1]]);
        dot += a[0] * b[0] + a[1] * b[1];
        cross += a[0] * b[1] - a[1] * b[0];
    }
    let angle = if from.len() < 2 { 0.0 } else { cross.atan2(dot) };
    let (s, c) = angle.sin_cos();
    let t = [q[0] - (c * p[0] - s * p[1]), q[1] - (s * p[0] + c * p[1])];
    let squares = from
        .iter()
        .zip(to)
        .map(|(a, b)| {
            let (x, y) = (c * a[0] - s * a[1] + t[0] - b[0], s * a[0] + c * a[1] + t[1] - b[1]);
            x * x + y * y
        })
        .sum();
    (angle, t, squares)
}

/// Align to footprint: the mount `current` (identity when `None`) refitted so
/// the part's seats land on their pads, as one rigid move, with the leftover.
///
/// 1. The seating plane. A mount that already tilts the part (a face pick,
///    Y up → Z up) keeps its plane. Otherwise the pins' wire directions give
///    the tilt, turned to −z; they must agree within 5° or Align refuses and
///    names the pins that do not. The height is then the lowest point of the
///    surface-mount pins (their contact bodies, or their seats), at z = 0; a
///    part with only through-hole pins keeps its height and is told to pick
///    its seating face.
/// 2. The fit in the board's plane, pins matched to pads by number. A pin with
///    one pad drives the fit; a pin with several pads of its number is matched
///    afterwards to the nearest. One driving pin moves the part and keeps its
///    rotation; none is refused. A mirrored fit that would leave a clearly
///    smaller leftover is said, and never applied.
pub fn mount_aligned_to_footprint(
    current: Option<&AffineTransform>,
    seats: &[AlignSeat],
    pads: &[AlignPad],
) -> Result<AlignReport, String> {
    let current = current.copied().unwrap_or_else(identity);
    let pads_of = |pin: &str| pads.iter().filter(|pad| pad.number.trim() == pin.trim()).collect::<Vec<_>>();
    let seats: Vec<(&AlignSeat, Vec<&AlignPad>)> = seats
        .iter()
        .map(|seat| (seat, pads_of(&seat.pin)))
        .filter(|(_, pads)| !pads.is_empty())
        .collect();
    if seats.is_empty() {
        return Err("no seated pcb point has a pad of its number: seat the pins first".into());
    }
    let mut notes = Vec::new();

    // 1. The seating plane.
    let planed = if has_tilt(&current) {
        current
    } else {
        let directions: Vec<[f64; 3]> = seats
            .iter()
            .map(|(seat, _)| {
                let d = linear(&current, seat.direction);
                let length = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                d.map(|c| c / length)
            })
            .collect();
        let sum = directions.iter().fold([0.0; 3], |s, d| [s[0] + d[0], s[1] + d[1], s[2] + d[2]]);
        let length = (sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]).sqrt();
        if !(length.is_finite() && length > 1e-9) {
            return Err("the pins' wire directions give no seating plane: pick its seating face".into());
        }
        let mean = sum.map(|c| c / length);
        let cos_limit = ALIGN_TILT_AGREEMENT_DEG.to_radians().cos();
        let mut apart: Vec<&str> = seats
            .iter()
            .zip(&directions)
            .filter(|(_, d)| !(d[0] * mean[0] + d[1] * mean[1] + d[2] * mean[2] >= cos_limit))
            .map(|((seat, _), _)| seat.pin.trim())
            .collect();
        apart.dedup();
        if !apart.is_empty() {
            return Err(format!(
                "pins {} leave the part more than {ALIGN_TILT_AGREEMENT_DEG}° away from the others, so they give no seating plane: pick its seating face",
                apart.join(", ")
            ));
        }
        let turn = turn_down(mean).ok_or("the pins' wire directions give no seating plane")?;
        let turned = turn.compose(&current)?;
        let lowest = seats
            .iter()
            .filter(|(_, pads)| pads.iter().all(|pad| pad.smd))
            .flat_map(|(seat, _)| if seat.support.is_empty() { vec![seat.position] } else { seat.support.clone() })
            .map(|p| turned.point(crate::Vec3::new(p[0], p[1], p[2])).z)
            .fold(f64::INFINITY, f64::min);
        if lowest.is_finite() {
            translation(0.0, 0.0, -lowest).compose(&turned)?
        } else {
            notes.push("its pins are all through-hole, so its height was not set: pick its seating face".to_string());
            turned
        }
    };

    // 2. The fit in the board's plane.
    let flat = |transform: &AffineTransform, seat: &AlignSeat| {
        let p = transform.point(crate::Vec3::new(seat.position[0], seat.position[1], seat.position[2]));
        [p.x, p.y]
    };
    let driving: Vec<(&AlignSeat, [f64; 2])> =
        seats.iter().filter(|(_, pads)| pads.len() == 1).map(|(seat, pads)| (*seat, pads[0].at)).collect();
    if driving.is_empty() {
        return Err("every seated pin has several pads of its number, so none fixes where the part goes".into());
    }
    let from: Vec<[f64; 2]> = driving.iter().map(|(seat, _)| flat(&planed, seat)).collect();
    let to: Vec<[f64; 2]> = driving.iter().map(|(_, at)| *at).collect();
    let (angle, t, squares) = fit_2d(&from, &to);
    if driving.len() == 1 {
        notes.push(format!("only pin {} has a single pad, so the part was moved and its rotation kept", driving[0].0.pin.trim()));
    } else {
        let mirrored: Vec<[f64; 2]> = from.iter().map(|p| [-p[0], p[1]]).collect();
        let (_, _, mirror_squares) = fit_2d(&mirrored, &to);
        let n = driving.len() as f64;
        let (proper, mirror) = ((squares / n).sqrt(), (mirror_squares / n).sqrt());
        if proper > ALIGN_MIRROR_FLOOR && mirror < 0.5 * proper {
            notes.push(format!(
                "a mirrored fit would leave {mirror:.3} mm RMS instead of {proper:.3} mm: the part may be modelled mirrored (Align never mirrors it)"
            ));
        }
    }
    let (s, c) = angle.sin_cos();
    let in_plane = AffineTransform { elements: [c, -s, 0.0, t[0], s, c, 0.0, t[1], 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0] };
    let mount = in_plane.compose(&planed)?;

    // 3. The leftover, every seat against its nearest pad of its number.
    let mut squares = 0.0;
    let (mut largest, mut worst) = (0.0_f64, String::new());
    for (seat, pads) in &seats {
        let at = flat(&mount, seat);
        let distance = pads
            .iter()
            .map(|pad| ((at[0] - pad.at[0]).powi(2) + (at[1] - pad.at[1]).powi(2)).sqrt())
            .fold(f64::INFINITY, f64::min);
        squares += distance * distance;
        if distance > largest || worst.is_empty() {
            largest = distance.max(largest);
            worst = seat.pin.trim().to_string();
        }
    }
    Ok(AlignReport { mount, largest, rms: (squares / seats.len() as f64).sqrt(), worst, pins: seats.len(), notes })
}

