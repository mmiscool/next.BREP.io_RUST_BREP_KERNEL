use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::face_move::{
    cached_plane, carrier_is_planar, carrier_kind_name, carrier_ruled,
    corner_on_plane_and_ruled, plane_and_ruled_carriers, resolve_corner_on_fixed_edge,
    retrim_ruled_face, solve_corner, verify_section_between,
};

// ---------------------------------------------------------------------------
// §6.12 sibling operation: ROTATE a face group about a picked axis.
//
// This file is what implements it, and nothing here reaches past what that
// section claims.
//
// Rotation is NOT a mode of the translation. `move_faces` moves a carrier's
// POSITION and leaves its normal alone, which is why a plane the translation is
// parallel to is invariant and why a moved rim can ride an exact affine of the
// fixed carrier. A rotation moves the normal too: the invariants are different
// (a cone about its OWN axis is invariant here and not there), the sections are
// different (plane × oblique cylinder is an ellipse, not a translated circle),
// and the corners must be solved from the ROTATED carriers rather than mapped.
// Sharing the machinery would mean an `if rotation` in every one of those
// places; the shared parts are the pure helpers this file imports.
// ---------------------------------------------------------------------------

/// How the rotated group uses an edge — the same three-way split
/// [`super::face_move`] makes, because the question ("is this edge inside the
/// group, on its boundary, or untouched?") is about the SELECTION, not the
/// transform.
enum EdgeClass {
    Fixed,
    Interior,
    Boundary { moved_face: u64, fixed_face: u64 },
}

enum EdgeAction {
    /// Map the curve by the rotation. Exact for any curve type: an affine maps
    /// a rational NURBS control-point-wise, so the parametrisation survives.
    Rotate,
    /// Replace the curve by the straight line between two re-solved corners.
    Rebuild { start: Vec3, end: Vec3 },
    /// Replace the curve outright by the EXACT conic section of the ROTATED
    /// planar carrier with a fixed OBLIQUE cylinder or cone — an ellipse on a
    /// cylinder, a conic on a cone. Unlike a translation, a rotation has no
    /// affine that carries the old rim to the new one (it moves the carrier's
    /// NORMAL, and neither the cylinder's axial shift nor the cone's homothety
    /// about the apex is a symmetry that does that), so the rim is
    /// re-intersected rather than mapped. The curve spans its own whole domain,
    /// so the edge's trim becomes that domain.
    Replace { curve: NurbsCurve },
}

enum FaceAction {
    /// Rotate the whole control net. Exact for any carrier type; the analytic
    /// tag is re-derived from the moved net, so a rotated plane is still a
    /// plane and a rotated cylinder still a cylinder of the same radius.
    RotateSurface,
    /// Rebuild the (planar) carrier around the new boundary and recompute every
    /// pcurve. `plane` is the carrier the face ends up on: the ROTATED plane for
    /// a moved face, its own unchanged plane for a re-trimmed fixed neighbour.
    Retrim(Plane),
}

/// The refusal a face whose carrier this operation cannot use deserves: one
/// that names the FACE, its CARRIER KIND and the ROLE it plays in the rotation.
///
/// [`super::face_move`] has the same helper with its own wording; the sentence
/// differs because the reach differs in its details. Both re-intersect a PLANE
/// and a CYLINDER or CONE neighbour, but a translation carries a ruled rim by
/// an exact affine while a rotation re-intersects it, and a rotation carries
/// any other curved carrier only when the axis leaves it invariant. Either side
/// of such a pair may be the one that turns — a tilted cap against a fixed bore,
/// or a tilted bore against a fixed flat — but ONE of the two must be the plane.
/// What is refused through here — a sphere, a torus, a general revolution, a
/// free-form patch, or a quadric meeting a quadric — is what the sentence names.
fn unsupported_carrier(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    role: &str,
) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, "carrier_kind", format!(
        "rotate_faces: {role} is a {}; a rotation re-intersects a PLANE with a plane, a \
         cylinder or a cone — whichever of the two it turns — and carries any other curved \
         carrier only when the axis leaves it invariant — refusing",
        face_label(solid, face_lookup, face_id)
    ))
}

/// `name (face id) … kind` in one phrase, so every refusal names the thing the
/// user picked and not just an integer they have no way to see.
fn face_label(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
) -> String {
    match face_lookup.get(&face_id) {
        Some(&(shell, face)) => {
            let record = &solid.shells[shell].faces[face];
            let kind = carrier_kind_name(&record.surface);
            match record.name.as_deref() {
                Some(name) => format!("`{name}` (face {face_id}) {kind}"),
                None => format!("face {face_id} {kind}"),
            }
        }
        None => format!("face {face_id} (missing)"),
    }
}

/// A short `name (id)` for a face, for messages that name a PAIR and would be
/// unreadable with two carrier kinds in them.
fn face_short(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
) -> String {
    match face_lookup.get(&face_id) {
        Some(&(shell, face)) => match solid.shells[shell].faces[face].name.as_deref() {
            Some(name) => format!("`{name}` (face {face_id})"),
            None => format!("face {face_id}"),
        },
        None => format!("face {face_id}"),
    }
}

/// The rigid motion `p ↦ a + R·(p − a)` as one affine: Rodrigues' rotation of
/// `angle` radians about the unit `direction`, with the translation that pins
/// the axis point `a`.
pub(super) fn rotation_about(point: Vec3, direction: Vec3, angle: f64) -> Result<AffineTransform, String> {
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    let (x, y, z) = (direction.x, direction.y, direction.z);
    // Row-major 3x3, the layout `AffineTransform` indexes.
    let r = [
        t * x * x + c,
        t * x * y - s * z,
        t * x * z + s * y,
        t * x * y + s * z,
        t * y * y + c,
        t * y * z - s * x,
        t * x * z - s * y,
        t * y * z + s * x,
        t * z * z + c,
    ];
    let a = [point.x, point.y, point.z];
    let mut elements = [0.0f64; 16];
    for row in 0..3 {
        for col in 0..3 {
            elements[row * 4 + col] = r[row * 3 + col];
        }
        // a − R·a, so the axis point is a fixed point of the map.
        elements[row * 4 + 3] = a[row] - (0..3).map(|k| r[row * 3 + k] * a[k]).sum::<f64>();
    }
    elements[15] = 1.0;
    AffineTransform::new(elements)
}

/// The rotation applied to a DIRECTION: the linear part alone, without the
/// translation that pins the axis point. A plane's normal, and the `u_dir` /
/// `v_dir` that carry its `same_sense`, all move this way.
pub(super) fn rotate_direction(rotation: &AffineTransform, vector: Vec3) -> Vec3 {
    let m = rotation.elements;
    Vec3::new(
        m[0] * vector.x + m[1] * vector.y + m[2] * vector.z,
        m[4] * vector.x + m[5] * vector.y + m[6] * vector.z,
        m[8] * vector.x + m[9] * vector.y + m[10] * vector.z,
    )
}

/// A whole plane carried by the rotation. `retrim_planar_face` rebuilds the
/// carrier from `origin`/`u_dir`/`v_dir` and relies on `normal = u × v` matching
/// the surface's own normal, so all four move together — rotating the origin and
/// leaving the frame behind would silently flip a face's `same_sense`.
fn rotate_plane(rotation: &AffineTransform, plane: &Plane) -> Plane {
    Plane {
        origin: rotation.point(plane.origin),
        u_dir: rotate_direction(rotation, plane.u_dir),
        v_dir: rotate_direction(rotation, plane.v_dir),
        normal: rotate_direction(rotation, plane.normal),
    }
}

/// The SIGNED AREA VECTOR of a trim loop, walked in its own coedge order in 3D:
/// `½ Σ pᵢ × pᵢ₊₁` over samples of every coedge, taken in the direction the
/// coedge runs. On a PLANAR loop this is Newell's normal — it points along the
/// side the loop winds counter-clockwise about, and its length is the loop's
/// area, so a degenerate loop reports a short vector rather than a wrong
/// direction.
///
/// This is the quantity that decides a re-trimmed planar carrier's FRAME. The
/// loop itself is the authority because the rotation does not move it: this
/// operation RE-SOLVES every corner from the carriers meeting there, so a
/// re-trimmed face's boundary is wherever those carriers put it and its 3D
/// traversal order is the topology's, which the edit never touches.
fn loop_area_vector(
    loop_record: &LoopRecord,
    edges: &HashMap<u64, EdgeRecord>,
    op: &str,
) -> Result<Vec3, KernelRefusal> {
    let mut points: Vec<Vec3> = Vec::new();
    for coedge in &loop_record.coedges {
        let edge = edges.get(&coedge.edge_id).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "loop_edge_lookup",
                format!("{op}: missing edge {}", coedge.edge_id),
            )
        })?;
        if edge.degenerate {
            continue;
        }
        for step in 0..8 {
            let fraction = step as f64 / 8.0;
            // ...in the direction the COEDGE runs, which is the direction the
            // loop is traversed and therefore the one the winding is about.
            let fraction = if coedge.forward {
                fraction
            } else {
                1.0 - fraction
            };
            points.push(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?);
        }
    }
    if points.len() < 3 {
        return Ok(Vec3::default());
    }
    let mut total = Vec3::default();
    for index in 0..points.len() {
        let a = points[index];
        let b = points[(index + 1) % points.len()];
        total = total.add(a.cross(b));
    }
    Ok(total.scale(0.5))
}

/// Point a re-trimmed planar carrier's frame the way the face's OWN outer loop
/// winds, flipping `v_dir` (and with it the carrier normal `u × v`) when the
/// candidate frame would hand the face a LEFT-HANDED chart.
///
/// # Why a rotated frame is not always the legal one
///
/// [`rotate_plane`] carries a moved face's frame by `R`, which is right for as
/// long as the face's boundary rides with it. It is not right once the rotation
/// has carried the carrier far enough that the face's re-solved boundary comes
/// back around the OTHER side of it: a half-turn about an axis IN the face maps
/// the carrier onto itself with its normal reversed, every corner re-solves to
/// exactly where it was, and the transported frame then describes the same
/// patch INSIDE OUT. `validate()` cannot see that — it tests incidence, and the
/// incidences are all still perfect — but the face's outer wire now winds
/// against its own sense, which is the chart convention
/// `validate_uv_wire` asserts and that `face_profile`, `thicken` and the
/// periodic trimmer all read as kernel-wide law.
///
/// The loop decides because the loop is the fixed point of the rebuild: its 3D
/// traversal is the topology's, and the neighbours on the other side of every
/// one of its edges keep the winding they had. Choosing the frame that agrees
/// with it is what keeps the shell COHERENT — and it is a real geometric
/// decision, not bookkeeping, because it is the face's outward normal that
/// changes with it. Whether the shell as a whole ends up inside out is a
/// separate question, and the signed-volume guard at the end is what answers it.
fn orient_plane_to_loop(
    face: &FaceRecord,
    plane: &Plane,
    edges: &HashMap<u64, EdgeRecord>,
) -> Result<Plane, KernelRefusal> {
    let Some(outer) = face.loops.first() else {
        return Ok(*plane);
    };
    let area = loop_area_vector(outer, edges, "rotate_faces")?;
    let projection = area.dot(plane.normal);
    // A loop with no area in this carrier decides nothing; leave the carried
    // frame alone rather than flip on rounding noise.
    if projection.abs() <= 1e-12 * area.length().max(1.0) {
        return Ok(*plane);
    }
    // The chart is right-handed (`normal = u × v`), so the outer loop winds
    // POSITIVELY in (u, v) exactly when it winds counter-clockwise about the
    // carrier normal — which is what `same_sense` asks of it.
    if (projection > 0.0) == face.same_sense {
        return Ok(*plane);
    }
    Ok(Plane {
        origin: plane.origin,
        u_dir: plane.u_dir,
        v_dir: plane.v_dir.scale(-1.0),
        normal: plane.normal.scale(-1.0),
    })
}

/// A ruled revolution's frame carried by the rotation. `R` is rigid, so the
/// turned frame describes the turned carrier EXACTLY and its radii and height
/// are untouched — which is what lets the bore wall's own section against a
/// fixed flat be re-intersected with the same closed form the mirror case uses.
fn rotate_ruled_frame(
    rotation: &AffineTransform,
    frame: &crate::RevolutionFrame,
) -> crate::RevolutionFrame {
    crate::RevolutionFrame {
        origin: rotation.point(frame.origin),
        axis: rotate_direction(rotation, frame.axis),
        x_axis: rotate_direction(rotation, frame.x_axis),
        y_axis: rotate_direction(rotation, frame.y_axis),
    }
}

/// True iff a FIXED carrier is INVARIANT under the rotation, so a corner on it
/// rides by `R` and provably stays on it. Two carriers are:
///   • a PLANE whose normal is parallel to the axis direction — the rotation
///     spins it within itself, at any offset along the axis; and
///   • a RULED REVOLUTION (cylinder, cone, or a partial-sweep fillet band) whose
///     own axis LINE *is* the rotation axis line.
///
/// The cone is the interesting difference from the translation's rule, which
/// excludes it: an axial TRANSLATION moves a cone's point to a station with a
/// different radius and so leaves the carrier, while a rotation about the cone's
/// own axis maps it exactly onto itself.
///
/// A sphere centred on the axis and a torus about it are invariant too, and are
/// deliberately NOT admitted here: this operation has no re-trim route for
/// either carrier, so a rim that rode onto one would arrive at the face pass
/// with a stale pcurve and no way to rebuild it. They refuse by name instead.
fn carrier_invariant_under(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    axis_point: Vec3,
    axis_direction: Vec3,
    angular_tolerance: f64,
    distance_tolerance: f64,
) -> bool {
    let Some(&(shell, face)) = face_lookup.get(&face_id) else {
        return false;
    };
    let surface = &solid.shells[shell].faces[face].surface;
    if let Some(AnalyticSurface::Plane { u_dir, v_dir, .. }) = surface.analytic() {
        let unit = match u_dir.cross(*v_dir).normalized() {
            Ok(unit) => unit,
            Err(_) => return false,
        };
        return unit.cross(axis_direction).length() <= angular_tolerance;
    }
    let Some((frame, _, _, _)) = carrier_ruled(solid, face_lookup, face_id) else {
        return false;
    };
    if frame.axis.cross(axis_direction).length() > angular_tolerance {
        return false;
    }
    // Parallel is not enough: the carrier's axis must be the SAME LINE, or the
    // rotation carries the whole surface sideways.
    let offset = frame.origin.sub(axis_point);
    offset
        .sub(axis_direction.scale(offset.dot(axis_direction)))
        .length()
        <= distance_tolerance
}

/// The ROTATED ruled carrier of the one moved face at a corner, when that is
/// what the corner has to be solved against: `Some((face id, the frame `R`
/// leaves, rho0, rho1, height))` for exactly one moved face at the vertex whose
/// carrier is a cylinder or cone the axis does NOT leave invariant.
///
/// `None` — and so the ordinary three-plane solve — for a planar moved face, for
/// a moved carrier the rotation maps onto itself (the identity/rigid regimes
/// above have already claimed those), and for a group that brings TWO faces to
/// the corner, where "the seam generatrix places it" is no longer a statement
/// about one carrier. A moved sphere, torus, revolution or free-form patch is
/// not ruled, so it falls through to the three-plane solve and refuses there by
/// name, as it did before this route existed.
fn rotated_moved_ruled_carrier(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    rotation: &AffineTransform,
    moved: &HashSet<u64>,
    adjacent: &HashSet<u64>,
    invariant: &dyn Fn(u64) -> bool,
) -> Option<(u64, crate::RevolutionFrame, f64, f64, f64)> {
    let mut here = adjacent.iter().copied().filter(|id| moved.contains(id));
    let face_id = here.next()?;
    if here.next().is_some() {
        return None;
    }
    if invariant(face_id) {
        return None;
    }
    let (frame, rho0, rho1, height) = carrier_ruled(solid, face_lookup, face_id)?;
    Some((
        face_id,
        rotate_ruled_frame(rotation, &frame),
        rho0,
        rho1,
        height,
    ))
}

/// How far `point` sits off a ruled revolution's carrier: the difference between
/// its radial distance from the axis and the carrier's own radius at that axial
/// station (constant for a cylinder, linear for a cone).
fn ruled_offset(frame: &crate::RevolutionFrame, rho0: f64, rho1: f64, height: f64, point: Vec3) -> f64 {
    let delta = point.sub(frame.origin);
    let axial = delta.dot(frame.axis);
    let radial = delta.sub(frame.axis.scale(axial)).length();
    let target = if height == 0.0 {
        rho0
    } else {
        rho0 + (rho1 - rho0) * (axial / height)
    };
    (radial - target).abs()
}

/// A STRAIGHT rebuilt edge bordering a curved carrier must still lie ON that
/// carrier — endpoints and midpoint — or the rebuild has torn the edge off it.
///
/// This is the guard that answers the coaxial-cap question the contract asks.
/// Spinning a cylinder's cap about the cylinder's OWN axis leaves both carriers
/// invariant, so the rim rides by `R` — but the WALL's seam edge has one
/// endpoint on that rim and one on the untouched bottom rim, and the straight
/// chord between them is not a generatrix: it cuts the chord of a 25° arc
/// through the inside of the cylinder. Measured before this guard existed, the
/// result reached `validate()` and failed there on a stale pcurve with the
/// arc's chord length as the deviation. The refusal names the wall.
///
/// `carrier_rotation` is `Some(R)` when the face is one the rotation MOVES —
/// the tilted bore wall carrying its own seam generatrix — in which case the
/// chord is measured against the carrier `R` leaves, not the one on record.
/// Measuring against the stored frame there would convict every tilt of a bore
/// of tearing its own seam off itself.
fn verify_chord_on_curved(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    edge_id: u64,
    start: Vec3,
    end: Vec3,
    carrier_rotation: Option<&AffineTransform>,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let Some((frame, rho0, rho1, height)) = carrier_ruled(solid, face_lookup, face_id) else {
        return Ok(());
    };
    let frame = match carrier_rotation {
        Some(rotation) => rotate_ruled_frame(rotation, &frame),
        None => frame,
    };
    let midpoint = start.add(end).scale(0.5);
    for point in [start, midpoint, end] {
        let off = ruled_offset(&frame, rho0, rho1, height, point);
        if off > 10.0 * tolerance {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, "chord_off_carrier", format!(
                "rotate_faces: rebuilding edge {edge_id} straight between its re-solved corners                  would leave its curved neighbour {} (off {off:.3e}); the rotation moves one end                  of that edge along the carrier and the other not at all, and no straight chord                  spans that — refusing",
                face_short(solid, face_lookup, face_id)
            )));
        }
    }
    Ok(())
}

/// A point that rode rigidly must genuinely still be ON every fixed carrier it
/// touches. The invariance predicate proves it in exact arithmetic; this is the
/// measurement that says so on the actual numbers, and it is what turns "rigid"
/// into a checked claim rather than an asserted one.
fn verify_point_on_fixed(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    planes: &mut HashMap<u64, Plane>,
    face_id: u64,
    point: Vec3,
    vertex_id: u64,
    plane_tolerance: f64,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    if let Some((frame, rho0, rho1, height)) = carrier_ruled(solid, face_lookup, face_id) {
        let off = ruled_offset(&frame, rho0, rho1, height, point);
        if off > 10.0 * tolerance {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, "corner_off_curved_neighbour", format!(
                "rotate_faces: the rotated corner at vertex {vertex_id} left its curved \
                 neighbour {} (off {off:.3e}) — refusing",
                face_short(solid, face_lookup, face_id)
            )));
        }
        return Ok(());
    }
    let plane = cached_plane(planes, solid, face_lookup, face_id, plane_tolerance)
        .map_err(|_| {
            unsupported_carrier(
                solid,
                face_lookup,
                face_id,
                &format!("a FIXED neighbour at vertex {vertex_id}"),
            )
        })?;
    let off = point.sub(plane.origin).dot(plane.normal).abs();
    if off > 10.0 * tolerance {
        return Err(KernelRefusal::unsupported(KernelStage::Validate, "corner_off_fixed_neighbour", format!(
            "rotate_faces: the rotated corner at vertex {vertex_id} left its fixed neighbour \
             {} (off {off:.3e}) — refusing",
            face_short(solid, face_lookup, face_id)
        )));
    }
    Ok(())
}

/// Rebuild a straight edge between two re-solved corners, refusing the two ways
/// an angle can turn a neighbour's trim inside out.
///
/// Both refusals name the EDGE and the two faces that carry it, which is the
/// half of the message a user can act on: "edge 9" alone does not say which wall
/// stopped the rotation.
fn plan_straight_rebuild(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    faces_of_edge: &HashMap<u64, Vec<u64>>,
    edge: &EdgeRecord,
    start_old: Vec3,
    end_old: Vec3,
    start_new: Vec3,
    end_new: Vec3,
    tolerance: f64,
) -> Result<EdgeAction, KernelRefusal> {
    let between = || -> String {
        let carriers = faces_of_edge
            .get(&edge.id)
            .map(Vec::as_slice)
            .unwrap_or(&[])
            .iter()
            .map(|&face_id| face_short(solid, face_lookup, face_id))
            .collect::<Vec<_>>();
        match carriers.len() {
            2 => format!("{} and {}", carriers[0], carriers[1]),
            _ => carriers.join(", "),
        }
    };
    if edge.degenerate {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "degenerate_edge", format!(
            "rotate_faces: degenerate edge {} between {} would need re-stretching (deferred) \
             — refusing",
            edge.id,
            between()
        )));
    }
    if edge.curve.straight_segment(tolerance).is_none() {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "curved_edge_rebuild", format!(
            "rotate_faces: edge {} between {} must be re-stretched but is not a straight line; \
             a rotation rebuilds a curved edge only where it is the rim of the rotated plane \
             against a cylinder or cone, not between two faces that stay put — refusing",
            edge.id,
            between()
        )));
    }
    let new_chord = end_new.sub(start_new);
    if new_chord.length() <= tolerance {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "edge_collapse", format!(
            "rotate_faces: the rotation collapses edge {} between {} to zero length — refusing",
            edge.id,
            between()
        )));
    }
    if end_old.sub(start_old).dot(new_chord) <= 0.0 {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, EDGE_INVERSION, format!(
            "rotate_faces: the rotation inverts edge {} between {} (the trim is turned inside \
             out) — refusing",
            edge.id,
            between()
        )));
    }
    Ok(EdgeAction::Rebuild {
        start: start_new,
        end: end_new,
    })
}

/// A corner solve shared with [`super::face_move`] words its refusals for a
/// PUSH ("the pushed cap plane … the push drives the corner off the carrier").
/// Swapping only the prefix left a rotation telling the user it had pushed
/// something, so the vocabulary is translated and the vertex named.
fn translate_corner_refusal(error: &str, vertex_id: u64) -> String {
    let body = error
        .trim_start_matches("move_faces: ")
        .replace("pushed cap plane", "rotated plane")
        .replace("the push drives", "the rotation drives")
        .replace("the push tears", "the rotation tears")
        .replace("pushed cap", "rotated face");
    format!("rotate_faces: at vertex {vertex_id}, {body}")
}

/// The SIGNED AREA VECTOR of a CLOSED curve over `[t0, t1]`: `½ Σ pᵢ × pᵢ₊₁`
/// around it. Dotted with a direction it gives the sense the curve winds about
/// that direction — positive for counter-clockwise looking down it — and the
/// magnitude is the projected area, so a rim seen edge-on reports ~0 rather
/// than a sign read off noise.
fn closed_curve_area_vector(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<Vec3, String> {
    const STEPS: usize = 64;
    let mut points = Vec::with_capacity(STEPS);
    for step in 0..STEPS {
        let t = t0 + (t1 - t0) * (step as f64 / STEPS as f64);
        points.push(curve.evaluate(t)?);
    }
    let mut total = Vec3::default();
    for index in 0..points.len() {
        total = total.add(points[index].cross(points[(index + 1) % points.len()]));
    }
    Ok(total.scale(0.5))
}

/// The sense a rim runs in about the carrier's axis: `true` when its azimuth
/// INCREASES along its own parameterisation.
///
/// [`super::face_move::arc_sweeps_forward`] answers this from the midpoint's
/// azimuth against the endpoints', which is undecidable for a CLOSED rim (the
/// start and end azimuths are the same one). This sums the wrapped azimuth
/// steps instead, so it reads a closed rim and an open arc the same way.
fn rim_runs_forward(
    frame: &crate::RevolutionFrame,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
) -> Result<bool, String> {
    let azimuth = |point: Vec3| {
        let delta = point.sub(frame.origin);
        delta.dot(frame.y_axis).atan2(delta.dot(frame.x_axis))
    };
    const STEPS: usize = 32;
    let mut total = 0.0;
    let mut previous = azimuth(curve.evaluate(t0)?);
    for step in 1..=STEPS {
        let t = t0 + (t1 - t0) * (step as f64 / STEPS as f64);
        let current = azimuth(curve.evaluate(t)?);
        let mut delta = current - previous;
        while delta > std::f64::consts::PI {
            delta -= std::f64::consts::TAU;
        }
        while delta < -std::f64::consts::PI {
            delta += std::f64::consts::TAU;
        }
        total += delta;
        previous = current;
    }
    Ok(total >= 0.0)
}

/// The new rim where a ROTATED planar carrier meets a fixed OBLIQUE cylinder or
/// cone: the EXACT conic section, anchored on the re-solved corners.
///
/// Two shapes of rim, one rule:
///
/// * a **CLOSED** rim — the hole mouth an oblique bore opens through the face —
///   is the WHOLE section, and the one thing that is free about it is where its
///   single vertex sits. That vertex is the face's own SEAM vertex, and the
///   section must start there, not at the carrier's `u = 0`: a face's seam EDGE
///   need not sit at the generatrix (the push-face seam fix, `becc64a98`), and a
///   rim anchored anywhere else would relocate the seam vertex and strand the
///   wall's seam edge. So the arc is built from the re-solved seam vertex's own
///   azimuth through a full turn.
/// * an **OPEN** rim is the sub-arc between the two re-solved corners' azimuths.
///
/// Both run in the OLD rim's own sense, so every coedge's `forward` flag — and
/// therefore every trim loop's orientation — survives the rebuild untouched.
///
/// The section is `plane ∩ frame`, and which of the two the rotation moved is
/// not this function's business: a rotated plane against a fixed cylinder and a
/// fixed plane against a ROTATED cylinder are the same closed form with the
/// roles swapped. `sense_frame` is the frame the OLD rim curve was measured in —
/// the carrier as it stands in `solid` — because that is the only frame
/// `edge.curve` is consistent with; `frame` is where the carrier ENDS UP, and it
/// is what the anchor azimuth and the section itself are built in.
#[allow(clippy::too_many_arguments)]
fn rotated_rim_section(
    edge: &EdgeRecord,
    frame: &crate::RevolutionFrame,
    sense_frame: &crate::RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    plane: &Plane,
    fixed_reference: Vec3,
    start_new: Vec3,
    end_new: Vec3,
    tolerance: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let azimuth = |point: Vec3| {
        let delta = point.sub(frame.origin);
        delta.dot(frame.y_axis).atan2(delta.dot(frame.x_axis))
    };
    let tau = std::f64::consts::TAU;
    let wrap = |angle: f64| {
        let value = angle % tau;
        if value < 0.0 {
            value + tau
        } else {
            value
        }
    };
    let forward = rim_runs_forward(sense_frame, &edge.curve, edge.t0, edge.t1).or_refuse(KernelStage::Refine, "rim_runs_forward")?;
    let closed = edge.start_vertex_id == edge.end_vertex_id;
    let (anchor, sweep) = if closed {
        (azimuth(start_new), tau)
    } else if forward {
        (azimuth(start_new), wrap(azimuth(end_new) - azimuth(start_new)))
    } else {
        (azimuth(end_new), wrap(azimuth(start_new) - azimuth(end_new)))
    };
    if !closed && (sweep <= 1e-9 || sweep >= tau - 1e-9) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "rim_degenerate", format!(
            "rotate_faces: the re-intersected rim of edge {} degenerates to a point or a \
             full turn — refusing",
            edge.id
        )));
    }
    let mut curve = plane_ruled_section_arc(
        plane.origin,
        plane.normal,
        frame,
        rho0,
        rho1,
        height,
        anchor,
        sweep,
        tolerance,
    )
    .map_err(|error| {
        KernelRefusal::unsupported(KernelStage::Refine, "ruled_section", format!(
            "rotate_faces: rebuilding the rim of edge {} as the section of the rotated \
             carrier with its fixed neighbour — {}",
            edge.id,
            error.trim_start_matches("plane_ruled_section_arc: ")
        ))
    })?;
    if !forward {
        curve = curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
    }
    if closed {
        // THE TURNOVER. A closed rim's direction is genuinely FREE — reversing a
        // closed edge's curve leaves its single vertex where it is, so both
        // faces it bounds simply traverse it the other way and no topology
        // moves — and the sense above transports it in the carrier that TURNED.
        // Once that carrier's axis has swung past the fixed neighbour's own
        // direction, a rim that keeps its sense about the turning axis reverses
        // its sense about the FIXED one, and the fixed face — whose chart the
        // rotation never touched — is handed a hole that winds backwards.
        //
        // So the sense is kept in the carrier that did NOT move: the cylinder's
        // axis when the plane is the side that turned, the plane's normal when
        // it is the bore. That face has no freedom left — its carrier, its
        // `same_sense` and its other loops all stand — so it is the one the rim
        // must agree with, and agreeing with it puts the TURNED face right too,
        // because the two traverse the one rim in opposite directions.
        //
        // Below the turnover this changes nothing: the transported sense and the
        // fixed carrier's already agree, and the branch is a measured no-op.
        let before = closed_curve_area_vector(&edge.curve, edge.t0, edge.t1).or_refuse(KernelStage::Refine, "closed_curve_area_vector")?.dot(fixed_reference);
        let [d0, d1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let after = closed_curve_area_vector(&curve, d0, d1).or_refuse(KernelStage::Refine, "closed_curve_area_vector")?.dot(fixed_reference);
        if before * after < 0.0 {
            curve = curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
    }
    // The same "reapply the trimming" contract every rebuilt section is held to:
    // it runs between the re-solved corners and every sample sits on BOTH
    // carriers. For a closed rim both corners are the one seam vertex.
    verify_section_between(
        &curve,
        frame,
        rho0,
        rho1,
        height,
        plane.normal,
        plane.normal.dot(plane.origin),
        start_new,
        end_new,
        tolerance,
    )
    .map_err(|error| error.with_message(|error| error.replace("move_faces:", "rotate_faces:")))?;
    Ok(curve)
}

/// ROTATE the faces `face_ids` of `solid` by `angle_radians` about the axis
/// through `axis_point` along `axis_direction` (right-handed), re-intersecting
/// the neighbours the rotation moves the selection relative to.
///
/// `solid` is never touched: every refusal below happens before a clone is even
/// made, or on the clone before it is returned, so a refused rotation leaves the
/// caller's solid exactly as it was.
///
/// # Regimes
///
/// **Rigid** — every fixed carrier a boundary vertex touches is invariant under
/// the rotation ([`carrier_invariant_under`]). The corner rides by `R`, the
/// edges map by `R` exactly, and the fixed faces re-trim around them. This is
/// what makes a cylindrical wall spun about its own axis a no-op that BUILDS.
///
/// **Re-intersection** — otherwise the corner is the common point of the
/// carriers meeting there, with the moved ones ROTATED, and every affected edge
/// is rebuilt between the new corners — straight between planes, and as the
/// EXACT conic wherever a plane meets an oblique cylinder or cone, whichever of
/// the two the rotation turns. The TURNED side of such a pair is the tilted
/// bore: its mouth is re-intersected in the frame `R` leaves, and its seam
/// corner rides its own generatrix down to the fixed flat. A quadric meeting a
/// quadric, or a sphere, torus, general revolution or free-form neighbour,
/// refuses by name.
pub fn rotate_faces(
    solid: &BrepSolid,
    face_ids: &[u64],
    axis_point: Vec3,
    axis_direction: Vec3,
    angle_radians: f64,
) -> Result<BrepSolid, KernelRefusal> {
    if !(axis_point.x.is_finite() && axis_point.y.is_finite() && axis_point.z.is_finite()) {
        return Err(KernelRefusal::input(KernelStage::Collect, "axis_point", "rotate_faces: the axis point must be finite"));
    }
    if !(axis_direction.x.is_finite()
        && axis_direction.y.is_finite()
        && axis_direction.z.is_finite())
    {
        return Err(KernelRefusal::input(KernelStage::Collect, "axis_direction", "rotate_faces: the axis direction must be finite"));
    }
    if !angle_radians.is_finite() {
        return Err(KernelRefusal::input(KernelStage::Collect, "angle", "rotate_faces: the angle must be finite"));
    }
    let axis = axis_direction.normalized().map_err(|_| {
        KernelRefusal::input(
            KernelStage::Collect,
            "axis_length",
            "rotate_faces: the axis direction has zero length",
        )
    })?;
    if face_ids.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "selection", "rotate_faces: no faces selected"));
    }
    let moved: HashSet<u64> = face_ids.iter().copied().collect();
    let mut face_lookup: HashMap<u64, (usize, usize)> = HashMap::default();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            face_lookup
                .entry(face.id)
                .or_insert((shell_index, face_index));
        }
    }
    for &face_id in face_ids {
        if !face_lookup.contains_key(&face_id) {
            return Err(KernelRefusal::input(KernelStage::Collect, "face_id", format!("rotate_faces: no face with id {face_id}")));
        }
    }

    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    // Direction comparisons are between UNIT vectors, so the band is
    // dimensionless — the sine of the angle two carriers' directions differ by.
    let angular_tolerance = 1e-9;
    // "The corner landed exactly where R put it": the rigid endpoints are
    // assigned `R(point)` verbatim, so this only absorbs rounding noise.
    let rigid_tolerance = (scale * 1e-9).max(1e-12);
    let rotation = rotation_about(axis_point, axis, angle_radians).or_refuse(KernelStage::Collect, "rotation")?;

    // --- Classify every edge by how the group uses it ----------------------
    let mut faces_of_edge: HashMap<u64, Vec<u64>> = HashMap::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    faces_of_edge
                        .entry(coedge.edge_id)
                        .or_default()
                        .push(face.id);
                }
            }
        }
    }
    let invariant = |face_id: u64| -> bool {
        carrier_invariant_under(
            solid,
            &face_lookup,
            face_id,
            axis_point,
            axis,
            angular_tolerance,
            plane_tolerance,
        )
    };

    let mut classes: HashMap<u64, EdgeClass> = HashMap::default();
    let mut planes: HashMap<u64, Plane> = HashMap::default();
    for edge in &solid.edges {
        let uses = faces_of_edge
            .get(&edge.id)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        let expected = if edge.degenerate { 1 } else { 2 };
        if uses.len() != expected {
            return Err(KernelRefusal::input(KernelStage::Collect, "non_manifold", format!(
                "rotate_faces: edge {} is used {} times (non-manifold input)",
                edge.id,
                uses.len()
            )));
        }
        let moved_uses = uses.iter().filter(|face_id| moved.contains(*face_id)).count();
        let class = if moved_uses == 0 {
            EdgeClass::Fixed
        } else if moved_uses == uses.len() {
            EdgeClass::Interior
        } else {
            let moved_face = *uses.iter().find(|face_id| moved.contains(*face_id)).unwrap();
            let fixed_face = *uses.iter().find(|face_id| !moved.contains(*face_id)).unwrap();
            // The face left behind across a boundary edge is what the rotated
            // carrier is re-intersected with. Planar → cache its plane. An
            // INVARIANT curved carrier → allowed: the rim rides it by `R`.
            // Anything else refuses HERE, before a single number is computed,
            // which is what makes "the refusal names the pair" true of the
            // curved cases as well as the planar ones.
            if carrier_is_planar(solid, &face_lookup, fixed_face) {
                cached_plane(&mut planes, solid, &face_lookup, fixed_face, plane_tolerance)?;
            } else if !invariant(fixed_face)
                && carrier_ruled(solid, &face_lookup, fixed_face).is_some()
                && !edge.degenerate
                && edge.curve.straight_segment(tolerance).is_some()
            {
                // A STRAIGHT boundary edge against a cylinder or cone is one of
                // the carrier's own rulings: the face was TANGENT to it (a fillet
                // band beside the face) or runs along its axis. Rotating such a
                // plane either separates it from the carrier or cuts it in a
                // different pair of rulings — a topology change, not a conic rim
                // — so there is nothing for the oblique route to re-intersect.
                // Refused here, before any corner is solved, so the message says
                // that instead of whichever corner solve would have failed first.
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "ruling_edge", format!(
                    "rotate_faces: boundary edge {} of {} is a straight ruling of its neighbour {} \
                     — the face is tangent to that carrier or runs along its axis, as beside a \
                     fillet band — and a rotation re-intersects a cylinder or cone only where the \
                     face cuts it in a conic — refusing",
                    edge.id,
                    face_short(solid, &face_lookup, moved_face),
                    face_label(solid, &face_lookup, fixed_face)
                )));
            } else if !invariant(fixed_face)
                && carrier_ruled(solid, &face_lookup, fixed_face).is_none()
            {
                // An OBLIQUE cylinder or cone is admitted here: the rotated
                // planar carrier meets it in an exact ELLIPSE (a conic on a
                // cone), which `rotated_rim_section` builds and which the wall
                // is then re-trimmed around. A sphere, a torus, a general
                // revolution and a free-form patch have no such route and still
                // refuse, by name, before a single number is computed.
                return Err(unsupported_carrier(
                    solid,
                    &face_lookup,
                    fixed_face,
                    &format!(
                        "the fixed neighbour across boundary edge {} of {}",
                        edge.id,
                        face_short(solid, &face_lookup, moved_face)
                    ),
                ));
            }
            EdgeClass::Boundary {
                moved_face,
                fixed_face,
            }
        };
        classes.insert(edge.id, class);
    }

    // --- THE PARALLEL LIMIT, before anything is solved ---------------------
    // A boundary pair that must be re-intersected needs the rotated moved plane
    // and the fixed plane to MEET. `solve_corner` would report the same failure
    // later as "parallel or under-constrained planes" at a vertex, which names
    // neither face and cannot distinguish this from a tearing corner. The angle
    // is in the message because the angle is the input that caused it.
    for edge in &solid.edges {
        let EdgeClass::Boundary {
            moved_face,
            fixed_face,
        } = classes[&edge.id]
        else {
            continue;
        };
        if invariant(fixed_face) {
            continue; // rides rigidly; the two carriers never have to meet anew
        }
        if carrier_ruled(solid, &face_lookup, fixed_face).is_some() {
            // A plane and a quadric are never PARALLEL — they meet in a conic
            // unless the plane misses the carrier entirely, which the section
            // construction itself reports (and names) at the rim rebuild.
            continue;
        }
        if let Some((frame, rho0, rho1, _)) = carrier_ruled(solid, &face_lookup, moved_face)
            .filter(|_| !invariant(moved_face))
        {
            // THE MIRROR LIMIT. A turned CYLINDER meets the flat in an ellipse
            // until its axis runs PARALLEL to that flat: there the section is a
            // pair of rulings, and there is no rim to build.
            // `plane_ruled_section_arc` reports it too, but only once a corner
            // has been solved on a carrier the rotation has already ruined, so
            // the angle that caused it is named here instead.
            //
            // A CONE is deliberately not tested here: an axis parallel to the
            // flat cuts it in a HYPERBOLA, not a ruling pair, and the limits
            // that actually stop a cone — the apex plane, and a section whose
            // rational weights change sign — are the ones the section
            // construction names for itself, on the fixed-cone row's wording.
            let radius_scale = rho0.abs().max(rho1.abs()).max(1.0);
            if (rho1 - rho0).abs() <= 1e-9 * radius_scale {
                let turned_axis = rotate_direction(&rotation, frame.axis);
                let fixed_plane = planes[&fixed_face];
                if turned_axis.dot(fixed_plane.normal).abs() <= PARALLEL_EPS {
                    return Err(KernelRefusal::unsupported(KernelStage::Classify, AXIS_PARALLEL_LIMIT, format!(
                        "rotate_faces: {:.6}° lays the axis of {} parallel to its neighbour {} \
                         across edge {}; the two carriers then meet in a pair of rulings rather \
                         than a conic, so there is no rim to rebuild — refusing",
                        angle_radians.to_degrees(),
                        face_short(solid, &face_lookup, moved_face),
                        face_short(solid, &face_lookup, fixed_face),
                        edge.id
                    )));
                }
            }
            continue;
        }
        if !carrier_is_planar(solid, &face_lookup, moved_face) {
            continue; // refused below, by the carrier gate, with a better message
        }
        let moved_plane =
            cached_plane(&mut planes, solid, &face_lookup, moved_face, plane_tolerance)?;
        let fixed_plane = planes[&fixed_face];
        let rotated_normal = rotate_direction(&rotation, moved_plane.normal);
        if rotated_normal.cross(fixed_plane.normal).length() <= PARALLEL_EPS {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, PARALLEL_LIMIT, format!(
                "rotate_faces: {:.6}° turns {} parallel to its neighbour {} across edge {}; \
                 the two carriers no longer meet, so there is no edge to rebuild — refusing",
                angle_radians.to_degrees(),
                face_short(solid, &face_lookup, moved_face),
                face_short(solid, &face_lookup, fixed_face),
                edge.id
            )));
        }
    }

    // --- Relocate every vertex the group touches ---------------------------
    let mut vertex_faces: HashMap<u64, HashSet<u64>> = HashMap::default();
    for edge in &solid.edges {
        if let Some(uses) = faces_of_edge.get(&edge.id) {
            for vertex_id in [edge.start_vertex_id, edge.end_vertex_id] {
                vertex_faces
                    .entry(vertex_id)
                    .or_default()
                    .extend(uses.iter().copied());
            }
        }
    }
    let mut new_vertex: HashMap<u64, Vec3> = HashMap::default();
    for vertex in &solid.vertices {
        let Some(adjacent) = vertex_faces.get(&vertex.id) else {
            continue;
        };
        if !adjacent.iter().any(|face_id| moved.contains(face_id)) {
            continue;
        }
        // THE IDENTITY. If EVERY carrier at this vertex — the rotated ones as
        // well as the fixed ones — is invariant under `R`, the rotation
        // constrains the vertex to nothing: its old position already satisfies
        // every (unchanged) carrier equation. Leaving it is not an optimisation,
        // it is the right answer, and it is what makes a cap spun about its own
        // cylinder's axis return the input rather than dragging the rim's seam
        // vertex around a circle the wall's own seam edge cannot follow.
        if adjacent.iter().all(|&face_id| invariant(face_id)) {
            continue;
        }
        let fixed_at: Vec<u64> = adjacent
            .iter()
            .copied()
            .filter(|face_id| !moved.contains(face_id))
            .collect();
        if fixed_at.is_empty() {
            // Interior vertex: carried rigidly with the group.
            new_vertex.insert(vertex.id, rotation.point(vertex.point));
            continue;
        }
        // RIGID: every fixed carrier here is invariant, so `R(vertex)` is on all
        // of them. Verified rather than asserted.
        if fixed_at.iter().all(|&face_id| invariant(face_id)) {
            let point = rotation.point(vertex.point);
            for &face_id in &fixed_at {
                verify_point_on_fixed(
                    solid,
                    &face_lookup,
                    &mut planes,
                    face_id,
                    point,
                    vertex.id,
                    plane_tolerance,
                    tolerance,
                )?;
            }
            new_vertex.insert(vertex.id, point);
            continue;
        }
        // A corner against a NON-INVARIANT ruled carrier — the oblique bore's
        // seam vertex. Its carrier is not a plane, so the three-plane solve
        // below cannot place it; it rides the ONE fixed edge it shares with the
        // body (the wall's seam generatrix, or a cone's straight meridian) to
        // where the ROTATED cap plane crosses it. Move Face solves the same
        // corner the same way under a translation; only the cap plane differs.
        if fixed_at
            .iter()
            .any(|&face_id| carrier_ruled(solid, &face_lookup, face_id).is_some() && !invariant(face_id))
        {
            let moved_here: Vec<u64> = adjacent
                .iter()
                .copied()
                .filter(|face_id| moved.contains(face_id))
                .collect();
            if moved_here.len() != 1 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "single_cap_only", format!(
                    "rotate_faces: the corner at vertex {} touches {} rotated faces against a \
                     ruled neighbour (a single rotated cap only) — refusing",
                    vertex.id,
                    moved_here.len()
                )));
            }
            let cap_plane =
                cached_plane(&mut planes, solid, &face_lookup, moved_here[0], plane_tolerance)
                    .map_err(|_| {
                        unsupported_carrier(
                            solid,
                            &face_lookup,
                            moved_here[0],
                            &format!("the ROTATED cap at vertex {}", vertex.id),
                        )
                    })?;
            let rotated_cap = rotate_plane(&rotation, &cap_plane);
            let plane_c = rotated_cap.normal.dot(rotated_cap.origin);
            let fixed_edges: Vec<&EdgeRecord> = solid
                .edges
                .iter()
                .filter(|edge| {
                    (edge.start_vertex_id == vertex.id || edge.end_vertex_id == vertex.id)
                        && matches!(classes.get(&edge.id), Some(EdgeClass::Fixed))
                })
                .collect();
            if fixed_edges.len() != 1 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "single_fixed_edge_only", format!(
                    "rotate_faces: the corner at vertex {} rides {} fixed edges against a ruled \
                     neighbour (exactly one required) — refusing",
                    vertex.id,
                    fixed_edges.len()
                )));
            }
            let fixed_edge = fixed_edges[0];
            let seed = if fixed_edge.start_vertex_id == vertex.id {
                fixed_edge.t0
            } else {
                fixed_edge.t1
            };
            // A STRAIGHT fixed edge rides its own line exactly, and a crossing
            // OUTSIDE its current span is still exact (a line is its own
            // extension) — which is what lets the rotation lengthen the wall.
            // A CURVED one is solved on the CARRIERS instead: the boolean split
            // it exactly at the corner, so riding it could only move the corner
            // inward. Straightness is geometry, not pole count.
            let carriers = if fixed_edge.curve.straight_segment(tolerance).is_some() {
                None
            } else {
                plane_and_ruled_carriers(
                    solid,
                    &face_lookup,
                    &mut planes,
                    &fixed_at,
                    plane_tolerance,
                )
            };
            let corner = match carriers {
                Some((fixed_plane, (frame, rho0, rho1, height))) => corner_on_plane_and_ruled(
                    &fixed_plane,
                    rotated_cap.normal,
                    plane_c,
                    &frame,
                    rho0,
                    rho1,
                    height,
                    vertex.point,
                    tolerance,
                )
                .map_err(|error| error.with_message(|error| translate_corner_refusal(error, vertex.id)))?,
                None => resolve_corner_on_fixed_edge(
                    fixed_edge,
                    seed,
                    rotated_cap.normal,
                    plane_c,
                    tolerance,
                )
                .map_err(|error| error.with_message(|error| translate_corner_refusal(error, vertex.id)))?,
            };
            // Checked, not asserted: the corner must sit on EVERY fixed carrier
            // at the vertex and on the rotated cap, or the group tore off.
            for &face_id in &fixed_at {
                verify_point_on_fixed(
                    solid,
                    &face_lookup,
                    &mut planes,
                    face_id,
                    corner,
                    vertex.id,
                    plane_tolerance,
                    tolerance,
                )?;
            }
            let off_cap = corner.sub(rotated_cap.origin).dot(rotated_cap.normal).abs();
            if off_cap > 10.0 * tolerance {
                return Err(KernelRefusal::internal(KernelStage::Validate, "corner_on_cap", format!(
                    "rotate_faces: the re-solved corner at vertex {} left the ROTATED cap \
                     (off {off_cap:.3e}) — refusing",
                    vertex.id
                )));
            }
            new_vertex.insert(vertex.id, corner);
            continue;
        }
        // THE MIRROR CORNER — a ROTATED ruled carrier against a fixed FLAT.
        // This is the bore wall the user tilts: the moved face is the cylinder
        // (or cone), the fixed neighbour is the flat it opens through, and the
        // rim they meet in is an ELLIPSE on that flat. The three-plane solve
        // below cannot place the corner (the moved carrier is not a plane), and
        // the branch above cannot either (it is the FIXED side that is curved
        // there). One point on the rim is free — the ellipse is a whole
        // one-parameter family — and the thing that fixes it is the face's own
        // SEAM: the corner is where the seam generatrix, carried by `R` with the
        // rest of the wall, crosses the flat. Riding the rotated edge is exact
        // for a straight generatrix, and it keeps the seam vertex on the seam,
        // which is what the rim section is then anchored at.
        if let Some(moved_ruled) = rotated_moved_ruled_carrier(
            solid,
            &face_lookup,
            &rotation,
            &moved,
            &adjacent,
            &invariant,
        ) {
            let (moved_face, turned_frame, rho0, rho1, height) = moved_ruled;
            if fixed_at.len() != 1 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "mirror_corner_flats", format!(
                    "rotate_faces: the corner at vertex {} where the rotated {} meets its \
                     neighbours touches {} fixed faces (a rotated cylinder or cone is \
                     re-intersected against exactly one flat there) — refusing",
                    vertex.id,
                    face_short(solid, &face_lookup, moved_face),
                    fixed_at.len()
                )));
            }
            let fixed_face = fixed_at[0];
            let flat = cached_plane(&mut planes, solid, &face_lookup, fixed_face, plane_tolerance)
                .map_err(|_| {
                    unsupported_carrier(
                        solid,
                        &face_lookup,
                        fixed_face,
                        &format!(
                            "the fixed neighbour of the rotated {} at vertex {}",
                            face_short(solid, &face_lookup, moved_face),
                            vertex.id
                        ),
                    )
                })?;
            let seam: Vec<&EdgeRecord> = solid
                .edges
                .iter()
                .filter(|edge| {
                    (edge.start_vertex_id == vertex.id || edge.end_vertex_id == vertex.id)
                        && matches!(classes.get(&edge.id), Some(EdgeClass::Interior))
                        && !edge.degenerate
                })
                .collect();
            // A rim vertex that is NOT the seam vertex — the bookkeeping split an
            // import leaves at a carrier's parameter origin — has no edge of the
            // group's own at it, and nothing then says where on the new ellipse
            // it belongs. It refuses here rather than being placed by guesswork.
            if seam.len() != 1 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "seam_edge_count", format!(
                    "rotate_faces: the corner at vertex {} on the rotated {} rides {} of the \
                     group's own edges (the seam generatrix, exactly one, is what places it) \
                     — refusing",
                    vertex.id,
                    face_short(solid, &face_lookup, moved_face),
                    seam.len()
                )));
            }
            let seam = seam[0];
            let seed = if seam.start_vertex_id == vertex.id {
                seam.t0
            } else {
                seam.t1
            };
            let turned_seam = EdgeRecord {
                curve: transform_curve(&seam.curve, rotation).or_refuse(KernelStage::Refine, "transform_curve")?,
                ..seam.clone()
            };
            let corner = resolve_corner_on_fixed_edge(
                &turned_seam,
                seed,
                flat.normal,
                flat.normal.dot(flat.origin),
                tolerance,
            )
            .map_err(|error| error.with_message(|error| translate_corner_refusal(error, vertex.id)))?;
            // Checked, not asserted, exactly as every other corner here is: on
            // the flat it must stay on, and on the carrier the rotation left.
            verify_point_on_fixed(
                solid,
                &face_lookup,
                &mut planes,
                fixed_face,
                corner,
                vertex.id,
                plane_tolerance,
                tolerance,
            )?;
            let off = ruled_offset(&turned_frame, rho0, rho1, height, corner);
            if off > 10.0 * tolerance {
                return Err(KernelRefusal::internal(KernelStage::Validate, "mirror_corner_on_carrier", format!(
                    "rotate_faces: the re-solved corner at vertex {} left the ROTATED {} \
                     (off {off:.3e}) — refusing",
                    vertex.id,
                    face_short(solid, &face_lookup, moved_face)
                )));
            }
            new_vertex.insert(vertex.id, corner);
            continue;
        }
        // RE-INTERSECTION: the corner is the common point of the carriers
        // meeting here, with the moved ones ROTATED. Every one of them must be
        // planar to solve it in closed form.
        let mut corner_planes = Vec::with_capacity(adjacent.len());
        for &face_id in &fixed_at {
            corner_planes.push(
                cached_plane(&mut planes, solid, &face_lookup, face_id, plane_tolerance).map_err(
                    |_| {
                        unsupported_carrier(
                            solid,
                            &face_lookup,
                            face_id,
                            &format!(
                                "a FIXED carrier meeting the rotated group at vertex {}",
                                vertex.id
                            ),
                        )
                    },
                )?,
            );
        }
        for face_id in adjacent.iter().copied().filter(|id| moved.contains(id)) {
            let plane = cached_plane(&mut planes, solid, &face_lookup, face_id, plane_tolerance)
                .map_err(|_| {
                    unsupported_carrier(
                        solid,
                        &face_lookup,
                        face_id,
                        &format!(
                            "a ROTATED carrier meeting a fixed neighbour at vertex {}",
                            vertex.id
                        ),
                    )
                })?;
            corner_planes.push(rotate_plane(&rotation, &plane));
        }
        let corner = solve_corner(&corner_planes).ok_or_else(|| {
            KernelRefusal::ill_posed(KernelStage::Refine, "corner_underconstrained", format!(
                "rotate_faces: cannot re-intersect the carriers meeting at vertex {} \
                 (parallel or under-constrained planes) — refusing",
                vertex.id
            ))
        })?;
        // The corner must genuinely sit on EVERY carrier; otherwise the group
        // tears away from its fixed neighbours and no manifold heal exists.
        for plane in &corner_planes {
            if corner.sub(plane.origin).dot(plane.normal).abs() > tolerance {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "corner_tear", format!(
                    "rotate_faces: the rotated group tears away from its neighbours at \
                     vertex {} — refusing rather than emitting an invalid solid",
                    vertex.id
                )));
            }
        }
        new_vertex.insert(vertex.id, corner);
    }

    // --- Plan every edge update -------------------------------------------
    let vertex_position: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let mut actions: HashMap<u64, EdgeAction> = HashMap::default();
    for edge in &solid.edges {
        let position = |vertex_id: u64| -> Result<Vec3, KernelRefusal> {
            vertex_position.get(&vertex_id).copied().ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "vertex_lookup",
                    format!("rotate_faces: missing vertex {vertex_id}"),
                )
            })
        };
        let start_old = position(edge.start_vertex_id)?;
        let end_old = position(edge.end_vertex_id)?;
        let start_new = new_vertex
            .get(&edge.start_vertex_id)
            .copied()
            .unwrap_or(start_old);
        let end_new = new_vertex
            .get(&edge.end_vertex_id)
            .copied()
            .unwrap_or(end_old);
        // Neither endpoint relocated, and every ROTATED carrier the edge lies on
        // maps onto itself: the edge is where it belongs already. (For a Fixed
        // edge the second clause is vacuous — no face using it is moved.)
        if start_new.sub(start_old).length() == 0.0
            && end_new.sub(end_old).length() == 0.0
            && faces_of_edge[&edge.id]
                .iter()
                .all(|face_id| !moved.contains(face_id) || invariant(*face_id))
        {
            continue;
        }
        let rigid = start_new.sub(rotation.point(start_old)).length() <= rigid_tolerance
            && end_new.sub(rotation.point(end_old)).length() <= rigid_tolerance;
        let rebuild = |planes: &mut HashMap<u64, Plane>| -> Result<EdgeAction, KernelRefusal> {
            // Every carrier of an edge that is REBUILT straight has to be
            // re-trimmed around it afterwards, and the only re-trims here are the
            // planar one and the invariant-ruled one.
            for &face_id in &faces_of_edge[&edge.id] {
                if invariant(face_id) {
                    continue;
                }
                // A NON-invariant RULED carrier re-trims too, on its own grown
                // carrier (`retrim_ruled_face`) — this is the arm the oblique
                // bore's seam generatrix takes, and `verify_chord_on_curved`
                // below is what proves the straight chord stayed on it.
                if carrier_ruled(solid, &face_lookup, face_id).is_some() {
                    continue;
                }
                cached_plane(planes, solid, &face_lookup, face_id, plane_tolerance).map_err(
                    |_| {
                        unsupported_carrier(
                            solid,
                            &face_lookup,
                            face_id,
                            &format!("a carrier of edge {}, whose trim the rotation moves", edge.id),
                        )
                    },
                )?;
            }
            let action = plan_straight_rebuild(
                solid,
                &face_lookup,
                &faces_of_edge,
                edge,
                start_old,
                end_old,
                start_new,
                end_new,
                tolerance,
            )?;
            if let EdgeAction::Rebuild { start, end } = &action {
                for &face_id in &faces_of_edge[&edge.id] {
                    verify_chord_on_curved(
                        solid,
                        &face_lookup,
                        face_id,
                        edge.id,
                        *start,
                        *end,
                        moved.contains(&face_id).then_some(&rotation),
                        tolerance,
                    )?;
                }
            }
            Ok(action)
        };
        match classes[&edge.id] {
            EdgeClass::Fixed => {
                // A fixed side edge follows its re-solved endpoint. Both of its
                // own carriers stayed put, so it cannot ride `R`: it is rebuilt
                // between the endpoints, straight.
                actions.insert(edge.id, rebuild(&mut planes)?);
            }
            EdgeClass::Interior => {
                if rigid {
                    actions.insert(edge.id, EdgeAction::Rotate);
                } else {
                    actions.insert(edge.id, rebuild(&mut planes)?);
                }
            }
            EdgeClass::Boundary {
                moved_face,
                fixed_face,
            } => {
                if rigid && invariant(fixed_face) {
                    // The edge maps by `R` and lands on the (unchanged) fixed
                    // carrier as well as on the rotated moved one — exact for any
                    // curve type, no re-intersection needed. This is the arm a
                    // cap spun about its own cylinder's axis takes.
                    actions.insert(edge.id, EdgeAction::Rotate);
                } else if let Some((frame, rho0, rho1, height)) =
                    carrier_ruled(solid, &face_lookup, fixed_face).filter(|_| !invariant(fixed_face))
                {
                    // THE OBLIQUE QUADRIC RIM. The rotated planar carrier meets
                    // the fixed cylinder in an ELLIPSE and the fixed cone in a
                    // conic, and no affine carries the old rim to it — a
                    // rotation moves the carrier's NORMAL, which neither the
                    // cylinder's axial shift nor the cone's homothety about its
                    // apex can follow. So the rim is RE-INTERSECTED, exactly,
                    // and anchored on the corners already re-solved above.
                    let moved_plane =
                        cached_plane(&mut planes, solid, &face_lookup, moved_face, plane_tolerance)
                            .map_err(|_| {
                                unsupported_carrier(
                                    solid,
                                    &face_lookup,
                                    moved_face,
                                    &format!(
                                        "the ROTATED side of boundary edge {} against {}",
                                        edge.id,
                                        face_short(solid, &face_lookup, fixed_face)
                                    ),
                                )
                            })?;
                    let curve = rotated_rim_section(
                        edge,
                        &frame,
                        &frame,
                        rho0,
                        rho1,
                        height,
                        &rotate_plane(&rotation, &moved_plane),
                        frame.axis,
                        start_new,
                        end_new,
                        tolerance,
                    )?;
                    actions.insert(edge.id, EdgeAction::Replace { curve });
                } else if let Some((frame, rho0, rho1, height)) =
                    carrier_ruled(solid, &face_lookup, moved_face)
                        .filter(|_| !invariant(moved_face))
                {
                    // THE MIRROR RIM — the tilted bore's mouth. The same
                    // `plane ∩ quadric` section as the arm above, with the roles
                    // swapped: here it is the CYLINDER (or cone) that turned and
                    // the FLAT that stayed, so the section is built in the frame
                    // `R` leaves and against the fixed plane. The sense is still
                    // read in the carrier's OLD frame, the only one the rim
                    // curve on record is consistent with, and the anchor is the
                    // seam corner the vertex pass just rode down the generatrix.
                    let fixed_plane =
                        cached_plane(&mut planes, solid, &face_lookup, fixed_face, plane_tolerance)
                            .map_err(|_| {
                                unsupported_carrier(
                                    solid,
                                    &face_lookup,
                                    fixed_face,
                                    &format!(
                                        "the FIXED side of boundary edge {} against the rotated {}",
                                        edge.id,
                                        face_short(solid, &face_lookup, moved_face)
                                    ),
                                )
                            })?;
                    let curve = rotated_rim_section(
                        edge,
                        &rotate_ruled_frame(&rotation, &frame),
                        &frame,
                        rho0,
                        rho1,
                        height,
                        &fixed_plane,
                        fixed_plane.normal,
                        start_new,
                        end_new,
                        tolerance,
                    )?;
                    actions.insert(edge.id, EdgeAction::Replace { curve });
                } else {
                    // Real re-intersection: the line where the ROTATED moved
                    // plane meets the fixed plane, delimited by the re-solved
                    // corners. The moved side must be planar for the chord to
                    // stay on it.
                    cached_plane(&mut planes, solid, &face_lookup, moved_face, plane_tolerance)
                        .map_err(|_| {
                            unsupported_carrier(
                                solid,
                                &face_lookup,
                                moved_face,
                                &format!(
                                    "the ROTATED side of boundary edge {} against {}",
                                    edge.id,
                                    face_short(solid, &face_lookup, fixed_face)
                                ),
                            )
                        })?;
                    actions.insert(edge.id, rebuild(&mut planes)?);
                }
            }
        }
    }

    // --- Plan face updates -------------------------------------------------
    // Edges whose SHAPE changed. A face bounding one of these must be RE-TRIMMED
    // (its rim moved to a new curve), not merely surface-transformed, else its
    // pcurve goes stale. A rigidly ROTATED edge on a rigidly rotated face keeps
    // its pcurve exactly, which is what the rigid regime is worth.
    let reshaped: HashSet<u64> = actions
        .iter()
        .filter(|(_, action)| {
            matches!(
                action,
                EdgeAction::Rebuild { .. } | EdgeAction::Replace { .. }
            )
        })
        .map(|(edge_id, _)| *edge_id)
        .collect();
    let dirty: HashSet<u64> = actions.keys().copied().collect();
    let mut face_actions: Vec<(usize, usize, FaceAction)> = Vec::new();
    // Ruled faces re-trim in a post-pass (they need `&mut solid`): the fixed
    // neighbours a rim rode onto, and the ROTATED wall whose own rims moved.
    let mut ruled_retrim_faces: Vec<u64> = Vec::new();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            let edge_ids = || {
                face.loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .map(|coedge| coedge.edge_id)
            };
            if moved.contains(&face.id) {
                if edge_ids().any(|edge_id| reshaped.contains(&edge_id))
                    && carrier_ruled(solid, &face_lookup, face.id).is_some()
                    && !invariant(face.id)
                {
                    // THE TILTED WALL ITSELF. Its rims were re-intersected, so
                    // the pcurves on record are stale — a bore that met its
                    // flats in circles meets them in ellipses once it leans, and
                    // in the wall's own parameters that is a sine, not a line.
                    // Turning the control net is exact (a rotated cylinder is a
                    // cylinder of the same radius), and the re-trim afterwards
                    // rebuilds the loops on THAT carrier, grown along its axis
                    // to cover the new rims — the tilt lengthens the wall.
                    face_actions.push((shell_index, face_index, FaceAction::RotateSurface));
                    ruled_retrim_faces.push(face.id);
                } else if edge_ids().any(|edge_id| reshaped.contains(&edge_id)) {
                    // A boundary edge changed shape, so the patch is re-trimmed
                    // around it — on the ROTATED plane, frame and all.
                    let plane =
                        cached_plane(&mut planes, solid, &face_lookup, face.id, plane_tolerance)
                            .map_err(|_| {
                                unsupported_carrier(
                                    solid,
                                    &face_lookup,
                                    face.id,
                                    "the ROTATED face whose rim the neighbours reshaped",
                                )
                            })?;
                    face_actions.push((
                        shell_index,
                        face_index,
                        FaceAction::Retrim(rotate_plane(&rotation, &plane)),
                    ));
                } else if edge_ids().any(|edge_id| dirty.contains(&edge_id)) {
                    // Every edge of the face rode along rigidly: rotating the
                    // control net keeps surface, curves and pcurves in exact
                    // agreement for ANY carrier type.
                    face_actions.push((shell_index, face_index, FaceAction::RotateSurface));
                } else if !invariant(face.id) {
                    // Nothing moved and the carrier does not map onto itself —
                    // which the vertex rule above makes unreachable, since a face
                    // whose boundary stayed put had every carrier at every one of
                    // its corners invariant, this one included. Rotate the net
                    // rather than silently emit a face off its own boundary.
                    face_actions.push((shell_index, face_index, FaceAction::RotateSurface));
                }
                // ...and an invariant carrier nothing touched is left ALONE: the
                // rotation maps it onto itself, so turning the control net would
                // only re-parameterise the surface under pcurves that were never
                // rebuilt. This is the arm the identity rotations exit through.
            } else if edge_ids().any(|edge_id| dirty.contains(&edge_id)) {
                if carrier_is_planar(solid, &face_lookup, face.id) {
                    // A FIXED plane keeps its carrier; only its trim moved.
                    let plane =
                        cached_plane(&mut planes, solid, &face_lookup, face.id, plane_tolerance)?;
                    face_actions.push((shell_index, face_index, FaceAction::Retrim(plane)));
                } else if carrier_ruled(solid, &face_lookup, face.id).is_some() {
                    // Both the INVARIANT coaxial band (whose rim rode by `R`)
                    // and the OBLIQUE cylinder or cone (whose rim was
                    // re-intersected) re-trim on their own carrier, grown along
                    // its axis to cover the new rim.
                    ruled_retrim_faces.push(face.id);
                } else {
                    return Err(unsupported_carrier(
                        solid,
                        &face_lookup,
                        face.id,
                        "a FIXED face the rotation must re-trim",
                    ));
                }
            }
        }
    }

    // --- Apply to a fresh clone (the input is never touched) ---------------
    let mut result = solid.clone();
    for edge in &mut result.edges {
        match actions.get(&edge.id) {
            Some(EdgeAction::Rotate) => {
                edge.curve = transform_curve(&edge.curve, rotation).or_refuse(KernelStage::Sew, "transform_curve")?;
            }
            Some(EdgeAction::Rebuild { start, end }) => {
                edge.curve = make_line(*start, *end).or_refuse(KernelStage::Sew, "make_line")?;
                edge.t0 = 0.0;
                edge.t1 = 1.0;
            }
            Some(EdgeAction::Replace { curve }) => {
                // An exactly re-intersected section spans its whole domain by
                // construction, so the trim IS the domain.
                let [d0, d1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
                edge.curve = curve.clone();
                edge.t0 = d0;
                edge.t1 = d1;
            }
            None => {}
        }
    }
    for vertex in &mut result.vertices {
        if let Some(point) = new_vertex.get(&vertex.id) {
            vertex.point = *point;
        }
    }
    let final_edges: HashMap<u64, EdgeRecord> = result
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    for (shell_index, face_index, action) in face_actions {
        let face = &mut result.shells[shell_index].faces[face_index];
        match action {
            FaceAction::RotateSurface => {
                face.surface = transform_surface(&face.surface, rotation).or_refuse(KernelStage::Sew, "transform_surface")?;
            }
            FaceAction::Retrim(plane) => {
                // A MOVED face's carrier frame is this operation's to choose,
                // and `R` is not always the choice that leaves the chart legal;
                // a FIXED neighbour's frame is not ours to touch, and its
                // outward normal must survive the re-trim exactly.
                let plane = if moved.contains(&face.id) {
                    orient_plane_to_loop(face, &plane, &final_edges)?
                } else {
                    plane
                };
                retrim_planar_face(face, &plane, &final_edges, scale, "rotate_faces")?;
                // `retrim_planar_face` maps each edge's WHOLE curve onto the
                // rebuilt plane, so an edge that represents a strict SUBRANGE of
                // its curve needs the range fitter to span exactly [t0, t1];
                // full-domain edges keep the exact affine pcurve just built.
                let face_edges: HashSet<u64> = face
                    .loops
                    .iter()
                    .flat_map(|loop_record| loop_record.coedges.iter().map(|c| c.edge_id))
                    .collect();
                refit_touched_pcurves(
                    face,
                    &final_edges,
                    &face_edges,
                    true,
                    tolerance,
                    "rotate_faces",
                )?;
            }
        }
    }
    for face_id in ruled_retrim_faces {
        retrim_ruled_face(&mut result, face_id, &final_edges, tolerance, "rotate_faces")?;
    }

    // Topology (and therefore genus) is untouched — only geometry moved — so
    // validate() re-checks Euler, loop closure and pcurve agreement.
    //
    // Taken through `validate_detailed` in ONE pass, because the floor below
    // needs the `wire_warnings` half of the same report and a second validation
    // of the same solid would buy nothing.
    let report = result.validate_detailed(&crate::KernelTolerances::for_solid(&result, 1e-7));
    if !report.issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "validate", format!(
            "rotate_faces: rotated solid failed validation: {:?}",
            report.issues
        )));
    }
    // THE ACCEPTANCE FLOOR. `validate()` tests INCIDENCE: it would pass a face
    // whose own trim loop crosses itself in its own parameter domain, because no
    // incidence check can see a bowtie — and that is exactly what a neighbour's
    // re-trim produces when the rotation folds it. Read `unreadable` as well as
    // the verdict: an empty crossing list beside a loop the scan could not read
    // is no answer, not a clean one.
    let crossings = crate::loop_self_crossings(&result);
    if !crossings.unreadable.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "crossing_scan_unreadable", format!(
            "rotate_faces: the loop self-crossing scan could not read {:?}; refusing rather \
             than shipping an unchecked result",
            crossings.unreadable
        )));
    }
    if crossings.loops == 0 {
        return Err(KernelRefusal::internal(KernelStage::Validate, "crossing_scan_empty",
            "rotate_faces: the loop self-crossing scan read no loops at all; refusing rather              than shipping an unchecked result"));
    }
    if crossings.is_flagged() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "loop_self_crossing", format!(
            "rotate_faces: the rotation makes a re-trimmed neighbour's own loop cross itself \
             ({}) — refusing",
            crossings.summary().unwrap_or_default()
        )));
    }
    // Belt and braces on top of the per-edge inversion guard: a global inversion
    // flips the signed volume even if every edge kept its direction.
    if let (Ok(before), Ok(after)) = (solid_signed_volume(solid), solid_signed_volume(&result)) {
        if before * after <= 0.0 {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, SOLID_INVERSION,
                "rotate_faces: the rotation inverts the solid (signed volume changed sign) \
                 — refusing"));
        }
    }
    // ...and a guard on a quantity the signed volume CANNOT carry.
    //
    // The volume is `⅓ ∮ x·n dA`, and on a trimmed face that integral reads the
    // carrier normal and the trim loop's winding TOGETHER — Green's theorem
    // turns the loop into the sign of the area element. Flip both at once, as a
    // carrier frame carried past its own turnover does, and the two negatives
    // cancel: the number comes back unchanged. That is not a weak signal, it is
    // a BLIND one, and no threshold on it can ever be made to see this. The
    // measured proof is the half-turn, where the body agreed with its input to
    // 3.6e-15 and the volume to 3.0e-16 with a face inside out.
    //
    // So the floor also reads the winding DIRECTLY, against the same
    // `validate_uv_wire` convention the rest of the kernel cites as law.
    // `validate()` drops that verdict — it returns `issues` and this lives under
    // `wire_warnings` — which is why a mis-wound result walked out past a
    // validation the operation already ran.
    //
    // The comparison is with what the INPUT carried, not with zero: a body that
    // arrives already mis-wound (an import, or an older edit) is not this
    // rotation's doing, and refusing it here would strand a user on a file they
    // cannot edit. Count, not identity — a rotation that repaired one face and
    // broke another would read as even here, and that is a narrower lie than
    // the blindness this replaces.
    //
    // The input is only validated when the result actually carries a warning,
    // so a rotation that leaves a clean body pays nothing for this at all.
    if !report.wire_warnings.is_empty() {
        let inherited = solid
            .validate_detailed(&crate::KernelTolerances::for_solid(solid, 1e-7))
            .wire_warnings;
        if report.wire_warnings.len() > inherited.len() {
            return Err(KernelRefusal::internal(KernelStage::Validate, "chart_winding", format!(
                "rotate_faces: the rotation leaves a face whose (u,v) chart is left-handed \
                 against its own normal ({}) — refusing",
                report
                    .wire_warnings
                    .iter()
                    .map(|warning| warning.message.as_str())
                    .collect::<Vec<_>>()
                    .join("; ")
            )));
        }
    }
    Ok(result)
}
