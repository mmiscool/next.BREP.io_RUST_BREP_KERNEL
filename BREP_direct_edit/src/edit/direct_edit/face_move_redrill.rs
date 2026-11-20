//! The RE-DRILL road: a through bore's wall carried rigidly, answered as the
//! body with the bore filled in less the moved carrier run the body's whole
//! way through.
//!
//! A through bore's extent was never its own cylinder's: the bore runs until
//! the body ends, and the faces it opens through are the body's. So when the
//! wall is carried, "where does the moved bore end" has one answer — wherever
//! its carrier meets the boundary — and that is a boolean of two primitives,
//! not a repair of the body the primitives built. The direct roads re-trim the
//! mouths on the faces they opened through, and the acceptance's crossing
//! repair follows a mouth across ONE neighbouring edge at a time; a mouth that
//! straddles a corner — crossing two neighbours at once — leaves two crossing
//! pairs the repair does not take, although the body exists. This road reaches
//! it from the other end.
//!
//! It is offered only after the direct road's body was refused, and it reads
//! nothing of that body. Each step is checked, and declines by name:
//!
//! * the selection is ONE cylinder carrier (every selected face on the same
//!   axis line and radius) bounding a HOLE — the face's normal points at its
//!   axis (`not_a_bore`);
//! * deleting the selection heals the hole shut (Delete Face healing, or,
//!   where that has no route, the intersection of every other face's own
//!   region; `bore_does_not_heal` — a blind hole's wall alone does neither);
//! * the IDENTITY: the healed body less the UNMOVED carrier, run the whole way
//!   through, is the input body to the volume tolerance. That is what makes the
//!   selection a through bore — a blind hole, or a carrier that would also cut
//!   some other part of the body, fails it (`not_a_through_bore`);
//! * the moved carrier removes material (`carrier_clear_of_body` — a bore
//!   carried clean off the body has no cut to perform, and handing back the
//!   healed body would make the hole silently vanish);
//! * what it removes is ONE connected piece (`carrier_cuts_twice` — on a
//!   non-convex part the whole carrier can drill a far leg the bore never
//!   reached), and what is left is one body (`bore_severs_body`);
//! * the moved wall is on the result.
//!
//! The tool's end caps lie beyond the healed body's bounds by construction,
//! so no face of the result lies on them.
//!
//! A BLIND bore (`BREP_REDRILL_BLIND_BORES`, default on; `=0` restores): a hole whose wall
//! ends on one flat FLOOR — an unselected plane square to the axis, bounded
//! only by the wall — has its extent fixed at one end by that floor and at the
//! other by the body. So the wall and the floor are healed shut together, and
//! the tool is the carrier run from the floor's plane out through the mouth
//! (`blind_tool`); the identity, the removal and the one-piece checks are the
//! through bore's. The floor is not selected, so it stays on its plane: only a
//! translation is taken (`turned_blind_bore` otherwise), and the floor comes
//! back as the tool's base under its own name.
//!
//! Names: the result's faces take the input's names back by carrier — a plane
//! by its plane, the wall by its moved carrier — and a carrier that comes
//! back as two faces gives the second `name_1`, the spelling the crossing
//! repair's own split uses. Edges take an authored name back where they run
//! where they ran, and every topology edge its derived `{faceA}|{faceB}[n]`.

use super::*;
use crate::offset_retrim::plane_of_surface;
use crate::{boolean_operation, make_cylinder_brep, BooleanOperation, BooleanOptions};
use crate::{KernelRefusal, KernelStage, OrRefuse};

const OP: &str = "transformFace(re-drill)";

/// Relative volume agreement the identity check and the "removes material"
/// check are held to. Booleans of a box-like body and a cylinder agree with
/// their closed forms far inside this.
const VOLUME_TOLERANCE: f64 = 1e-9;

/// A cylinder carrier: a point on its axis, its unit axis, its radius.
#[derive(Clone, Copy, Debug)]
struct Carrier {
    origin: Vec3,
    axis: Vec3,
    radius: f64,
}

impl Carrier {
    fn same_line_and_radius(&self, other: &Carrier, tolerance: f64) -> bool {
        let parallel = self.axis.cross(other.axis).length() <= 1e-9;
        let offset = other.origin.sub(self.origin);
        let off_axis = offset.sub(self.axis.scale(offset.dot(self.axis))).length();
        parallel && off_axis <= tolerance && (self.radius - other.radius).abs() <= tolerance
    }
}

fn declined(what: &str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{OP}: {message}"))
}

fn volume(solid: &BrepSolid, label: &str) -> Result<f64, KernelRefusal> {
    solid_signed_volume(solid)
        .map(f64::abs)
        .map_err(|error| declined("volume", format!("the {label} has no volume: {error}")))
}

fn cylinder_carrier(face: &FaceRecord, tolerance: f64) -> Option<Carrier> {
    match face.surface.analytic()? {
        // A cylinder, not a cone: the taper is held to the radius's own
        // rounding, so a slightly tapered bore is never re-drilled straight.
        AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. }
            if (rho1 - rho0).abs() <= 1e-12 * rho0.abs().max(1.0) && *rho0 > tolerance =>
        {
            Some(Carrier { origin: frame.origin, axis: frame.axis, radius: *rho0 })
        }
        _ => None,
    }
}

/// Whether a cylinder face bounds a HOLE: its outward normal points at the
/// axis.
fn bounds_a_hole(face: &FaceRecord, carrier: &Carrier) -> Option<bool> {
    let (u, v) = {
        let [u0, u1] = face.surface.domain_u().ok()?;
        let [v0, v1] = face.surface.domain_v().ok()?;
        (0.5 * (u0 + u1), 0.5 * (v0 + v1))
    };
    let point = face.surface.evaluate(u, v).ok()?;
    let normal = face.surface.normal(u, v).ok()?;
    let normal = if face.same_sense { normal } else { normal.scale(-1.0) };
    let offset = point.sub(carrier.origin);
    let radial = offset.sub(carrier.axis.scale(offset.dot(carrier.axis)));
    Some(normal.dot(radial) < 0.0)
}

/// The carrier run through the whole of `body`: a cylinder on the carrier
/// whose end caps lie beyond the body's bounds along the axis.
fn through_tool(carrier: &Carrier, body: &BrepSolid) -> Result<BrepSolid, KernelRefusal> {
    let bounds = crate::recentre::solid_bounds(body);
    let centre = bounds.minimum.add(bounds.maximum).scale(0.5);
    let half_diagonal = bounds.maximum.sub(bounds.minimum).length() * 0.5;
    if !half_diagonal.is_finite() {
        return Err(declined("unbounded_body", "the healed body's bounds cannot be read".into()));
    }
    let along = centre.sub(carrier.origin).dot(carrier.axis);
    let foot = carrier.origin.add(carrier.axis.scale(along));
    // Every point of the body is within `half_diagonal` of `centre`, so its
    // axial station lies within that of `foot`'s; the caps sit a margin past.
    let reach = half_diagonal + carrier.radius + 0.1 * half_diagonal.max(1.0);
    make_cylinder_brep(foot.sub(carrier.axis.scale(reach)), carrier.axis, carrier.radius, 2.0 * reach)
        .map_err(|error| declined("tool", format!("the through cylinder does not build: {error}")))
}

fn subtract(body: &BrepSolid, tool: &BrepSolid, label: &str) -> Result<BrepSolid, KernelRefusal> {
    boolean_operation(body, tool, BooleanOperation::Subtract, &BooleanOptions::default())
        .map_err(|error| error.with_message(|message| format!("{OP}: the {label} refused — {message}")))
}

/// `BREP_REDRILL_BLIND_BORES` (default on; `=0` restores): the re-drill also takes a blind
/// bore, its floor's plane fixed (see the module comment).
fn blind_bores_enabled() -> bool {
    switch!(live "BREP_REDRILL_BLIND_BORES", true)
}

/// A blind bore's floor: the one unselected face that is a plane square to the
/// carrier's axis and shares every one of its edges with the selected wall.
/// Returns the face, its outward normal (pointing up the hole, toward the
/// mouth) and its offset. `Ok(None)` when no face is one.
fn blind_floor<'a>(
    solid: &'a BrepSolid,
    face_ids: &[u64],
    carrier: &Carrier,
    tolerance: f64,
) -> Result<Option<(&'a FaceRecord, Vec3, f64)>, KernelRefusal> {
    let wall_edges: HashSet<u64> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face_ids.contains(&face.id))
        .flat_map(|face| face.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id)))
        .collect();
    let mut floors = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if face_ids.contains(&face.id) {
            continue;
        }
        let edges: Vec<u64> = face.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id)).collect();
        if edges.is_empty() || !edges.iter().all(|edge| wall_edges.contains(edge)) {
            continue;
        }
        let Ok(plane) = plane_of_surface(&face.surface, tolerance.max(1e-9) * 1e3, OP) else { continue };
        let normal = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
        if normal.cross(carrier.axis).length() > 1e-9 {
            continue;
        }
        floors.push((face, normal, normal.dot(plane.origin)));
    }
    match floors.len() {
        0 => Ok(None),
        1 => Ok(floors.pop()),
        n => Err(declined(
            "not_a_bore",
            format!("the selected wall closes on {n} flat faces, a closed pocket rather than a hole with a mouth"),
        )),
    }
}

/// A blind bore's DRILL POINT: the one unselected face that is a cone on the
/// carrier's axis, its wide end the carrier's radius and its other end an apex,
/// whose every non-degenerate edge is the wall's. Returns the face, the unit
/// direction up the hole (apex → rim), the rim plane's offset along it and the
/// point's depth (rim to apex). `Ok(None)` when no face is one.
fn point_floor<'a>(
    solid: &'a BrepSolid,
    face_ids: &[u64],
    carrier: &Carrier,
    tolerance: f64,
) -> Result<Option<(&'a FaceRecord, Vec3, f64, f64)>, KernelRefusal> {
    let band = tolerance * 1e3;
    let wall_edges: HashSet<u64> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face_ids.contains(&face.id))
        .flat_map(|face| face.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id)))
        .collect();
    let mut points = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if face_ids.contains(&face.id) {
            continue;
        }
        // Its own seam (an edge the face uses twice) and a degenerate apex
        // edge are not boundaries.
        let uses: Vec<u64> = face.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id)).collect();
        let edges: Vec<&EdgeRecord> = uses
            .iter()
            .filter(|id| uses.iter().filter(|other| other == id).count() == 1)
            .filter_map(|id| solid.edges.iter().find(|edge| edge.id == *id))
            .filter(|edge| !edge.degenerate)
            .collect();
        if edges.is_empty() || !edges.iter().all(|edge| wall_edges.contains(&edge.id)) {
            continue;
        }
        let Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) = face.surface.analytic() else { continue };
        // A cone, coaxial with the carrier, from the carrier's radius to a point.
        let (rim_at, apex_at) = if rho1.abs() <= band && (rho0 - carrier.radius).abs() <= band {
            (0.0, *height)
        } else if rho0.abs() <= band && (rho1 - carrier.radius).abs() <= band {
            (*height, 0.0)
        } else {
            continue;
        };
        if frame.axis.cross(carrier.axis).length() > 1e-9 {
            continue;
        }
        let offset = frame.origin.sub(carrier.origin);
        if offset.sub(carrier.axis.scale(offset.dot(carrier.axis))).length() > band {
            continue;
        }
        let rim = frame.origin.add(frame.axis.scale(rim_at));
        let apex = frame.origin.add(frame.axis.scale(apex_at));
        let Ok(up) = rim.sub(apex).normalized() else { continue };
        points.push((face, up, up.dot(rim), rim.sub(apex).length()));
    }
    match points.len() {
        0 => Ok(None),
        1 => Ok(points.pop()),
        n => Err(declined("not_a_bore", format!("the selected wall closes on {n} drill points"))),
    }
}

/// The blind tool for a drill-point bore: the carrier from the rim plane up
/// through the mouth (`blind_tool`), united with the point, the cone from the
/// rim down to its apex.
fn point_tool(carrier: &Carrier, up: Vec3, rim_offset: f64, depth: f64, body: &BrepSolid) -> Result<BrepSolid, KernelRefusal> {
    let shank = blind_tool(carrier, up, rim_offset, body)?;
    let axis = if carrier.axis.dot(up) >= 0.0 { carrier.axis } else { carrier.axis.scale(-1.0) };
    let rim = carrier.origin.add(axis.scale((rim_offset - up.dot(carrier.origin)) / up.dot(axis)));
    let point = crate::make_cone_brep(rim, axis.scale(-1.0), carrier.radius, 0.0, depth)
        .map_err(|error| declined("tool", format!("the drill point does not build: {error}")))?;
    boolean_operation(&shank, &point, BooleanOperation::Union, &BooleanOptions::default())
        .map_err(|error| error.with_message(|message| format!("{OP}: the drill-point tool refused — {message}")))
}

/// The carrier run from the floor's plane up through the mouth and past the
/// body: a cylinder whose base lies on the floor's plane and whose far cap is
/// beyond the body's bounds.
fn blind_tool(carrier: &Carrier, up: Vec3, floor_offset: f64, body: &BrepSolid) -> Result<BrepSolid, KernelRefusal> {
    let bounds = crate::recentre::solid_bounds(body);
    let centre = bounds.minimum.add(bounds.maximum).scale(0.5);
    let half_diagonal = bounds.maximum.sub(bounds.minimum).length() * 0.5;
    if !half_diagonal.is_finite() {
        return Err(declined("unbounded_body", "the healed body's bounds cannot be read".into()));
    }
    let axis = if carrier.axis.dot(up) >= 0.0 { carrier.axis } else { carrier.axis.scale(-1.0) };
    // Where the axis line meets the floor's plane.
    let rate = up.dot(axis);
    if rate.abs() <= 1e-12 {
        return Err(declined("not_a_bore", "the floor is not square to the bore's axis".into()));
    }
    let base = carrier.origin.add(axis.scale((floor_offset - up.dot(carrier.origin)) / rate));
    let length = centre.sub(base).dot(axis).max(0.0) + half_diagonal + carrier.radius + 0.1 * half_diagonal.max(1.0);
    make_cylinder_brep(base, axis, carrier.radius, length)
        .map_err(|error| declined("tool", format!("the blind cylinder does not build: {error}")))
}

/// `BREP_REDRILL_DRILL_POINTS` (default on; `=0` restores): a blind bore may end on a drill
/// point — a coaxial cone from its radius to an apex — instead of a flat floor
/// (`point_floor`, `point_tool`); needs `BREP_REDRILL_BLIND_BORES` too.
fn drill_points_enabled() -> bool {
    switch!(live "BREP_REDRILL_DRILL_POINTS", true)
}

/// `BREP_REDRILL_STEPPED_HOLES` (default on; `=0` restores): a hole whose walls are several
/// coaxial cylinders and cones with flat steps between them (a counterbore, a
/// countersink), every face of it selected, is re-drilled as a stack of pieces
/// (`redrill_stepped_hole`).
fn stepped_holes_enabled() -> bool {
    switch!(live "BREP_REDRILL_STEPPED_HOLES", true)
}

/// `BREP_REDRILL_TURNED_STEPPED` (default on; `=0` restores): a THROUGH stepped hole may also
/// be turned (the whole stack carried rigidly, its run-ons along the turned
/// axis); a blind one still declines a turn.
fn turned_stepped_enabled() -> bool {
    switch!(live "BREP_REDRILL_TURNED_STEPPED", true)
}

/// `BREP_REDRILL_CARRIED_FLOORS` (default on; `=0` restores): a hole whose floor (a flat cap
/// or a drill point) is SELECTED with its walls is carried whole — floor
/// included — by any rigid motion, along its axis and turned alike.
fn carried_floors_enabled() -> bool {
    switch!(live "BREP_REDRILL_CARRIED_FLOORS", true)
}

/// One piece of a stepped hole's stack: axial stations `lo < hi` along the
/// hole's axis and the radius at each.
#[derive(Clone, Copy, Debug)]
struct Piece {
    lo: f64,
    hi: f64,
    r_lo: f64,
    r_hi: f64,
    /// A cylinder by its own surface (rho0 == rho1 to the radius's rounding),
    /// never by a model-scale band: a shallow taper stays a cone.
    straight: bool,
}

impl Piece {
    /// The cone narrows to an APEX at its low end — zero relative to its own
    /// wide radius (an apex's analytic roundoff), never a model-scale band: a
    /// small but real radius stays a frustum.
    fn is_point_low(&self) -> bool {
        !self.straight && self.r_lo.abs() <= 1e-9 * self.r_hi.abs().max(1e-300)
    }

    /// The same at the high end.
    fn is_point_high(&self) -> bool {
        !self.straight && self.r_hi.abs() <= 1e-9 * self.r_lo.abs().max(1e-300)
    }
}

/// A STEPPED hole carried by a translation: every selected face is a cylinder
/// or cone on one axis bounding the hole, or a flat step square to it; the walls
/// read as a contiguous stack of pieces along the axis. The hole (and a flat
/// floor or drill point closing one end) heals shut, and the result is the
/// healed body less every piece carried, the pieces at an open end run on past
/// the body. Checks as the bore's: the IDENTITY (healed less the unmoved stack
/// is the input, volume and area), material removed, one removed piece, the
/// same bodies. Names come back by matching each carried selected surface.
fn redrill_stepped_hole(
    solid: &BrepSolid,
    face_ids: &[u64],
    selected: &[&FaceRecord],
    turn: Option<(Vec3, Vec3, f64)>,
    translation: Vec3,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    const OP_STEP: &str = "transformFace(stepped hole)";
    let decline = |what: &str, message: String| KernelRefusal::unsupported(KernelStage::Classify, what, format!("{OP_STEP}: {message}"));
    if turn.is_some() && !turned_stepped_enabled() {
        return Err(decline("turned_stepped_hole", "only a translation re-drills a stepped hole".into()));
    }
    // The rigid motion: turn first, then translate (identity turn for a pure
    // translation).
    let rotation = match turn {
        None => None,
        Some((point, direction, angle)) => Some(
            super::face_rotate::rotation_about(point, direction.normalized().or_refuse(KernelStage::Collect, "rotation")?, angle)
                .or_refuse(KernelStage::Collect, "rotation")?,
        ),
    };
    let map_point = |point: Vec3| match &rotation {
        None => point.add(translation),
        Some(rotation) => rotation.point(point).add(translation),
    };
    let map_dir = |direction: Vec3| match &rotation {
        None => direction,
        Some(rotation) => super::face_rotate::rotate_direction(rotation, direction).normalized().unwrap_or(direction),
    };
    let band = tolerance * 1e3;
    // The axis: the first revolution face's.
    let revolutions: Vec<(&FaceRecord, crate::RevolutionFrame, f64, f64, f64)> = selected
        .iter()
        .filter_map(|face| match face.surface.analytic() {
            Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) => Some((*face, frame.clone(), *rho0, *rho1, *height)),
            _ => None,
        })
        .collect();
    let Some((_, first, ..)) = revolutions.first() else {
        return Err(decline("not_a_bore", "no selected face is a cylinder or cone".into()));
    };
    let (origin, axis) = (first.origin, first.axis);
    let station = |point: Vec3| axis.dot(point.sub(origin));
    let mut pieces: Vec<Piece> = Vec::new();
    for (face, frame, rho0, rho1, height) in &revolutions {
        let offset = frame.origin.sub(origin);
        if frame.axis.cross(axis).length() > 1e-9 || offset.sub(axis.scale(offset.dot(axis))).length() > band {
            return Err(decline("not_a_bore", format!("face {} is not on the hole's axis", face.id)));
        }
        let carrier = Carrier { origin, axis, radius: rho0.max(*rho1) };
        if bounds_a_hole(face, &carrier) != Some(true) {
            return Err(decline("not_a_bore", format!("face {} bounds material, not a hole", face.id)));
        }
        // The face's axial extent from its vertices; the radius there from the
        // generatrix (rho0 at the frame's origin, rho1 `height` along its axis).
        let stations: Vec<f64> = face
            .loops
            .iter()
            .flat_map(|lp| &lp.coedges)
            .filter_map(|coedge| solid.edges.iter().find(|edge| edge.id == coedge.edge_id))
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .filter_map(|id| solid.vertices.iter().find(|vertex| vertex.id == id))
            .map(|vertex| station(vertex.point))
            .collect();
        let (lo, hi) = stations.iter().fold((f64::MAX, f64::MIN), |(lo, hi), s| (lo.min(*s), hi.max(*s)));
        if !(hi - lo > band) {
            return Err(decline("not_a_bore", format!("face {} has no axial extent", face.id)));
        }
        let base = station(frame.origin);
        let along = frame.axis.dot(axis).signum();
        let radius_at = |s: f64| rho0 + (rho1 - rho0) * ((s - base) * along / height);
        let straight = (rho1 - rho0).abs() <= 1e-12 * rho0.abs().max(1.0);
        let piece = Piece { lo, hi, r_lo: radius_at(lo), r_hi: radius_at(hi), straight };
        if !pieces.iter().any(|other| (other.lo - piece.lo).abs() <= band && (other.hi - piece.hi).abs() <= band) {
            pieces.push(piece);
        }
    }
    for face in selected {
        if matches!(face.surface.analytic(), Some(AnalyticSurface::RuledRevolution { .. })) {
            continue;
        }
        let plane = plane_of_surface(&face.surface, band, OP_STEP)
            .map_err(|_| decline("not_a_bore", format!("face {} is neither a wall on the axis nor a flat step", face.id)))?;
        if plane.normal.cross(axis).length() > 1e-9 {
            return Err(decline("not_a_bore", format!("step face {} is not square to the hole's axis", face.id)));
        }
    }
    pieces.sort_by(|a, b| a.lo.total_cmp(&b.lo));
    for pair in pieces.windows(2) {
        if (pair[0].hi - pair[1].lo).abs() > band {
            return Err(decline("not_a_bore", "the hole's walls do not stack end to end along its axis".into()));
        }
    }
    let (bottom, top) = (pieces[0], pieces[pieces.len() - 1]);

    // A flat floor or a drill point may close one end.
    // Each closing form only under its own switch, so `=0` restores the old
    // refusal on this road too.
    let floor = if blind_bores_enabled() {
        blind_floor(solid, face_ids, &Carrier { origin, axis, radius: bottom.r_lo }, tolerance)?
    } else {
        None
    };
    let point = if floor.is_none() && blind_bores_enabled() && drill_points_enabled() {
        let low = point_floor(solid, face_ids, &Carrier { origin, axis, radius: bottom.r_lo }, tolerance)?;
        let high = point_floor(solid, face_ids, &Carrier { origin, axis, radius: top.r_hi }, tolerance)?;
        match (low, high) {
            // The same face found from both ends is one point.
            (Some(a), Some(b)) if a.0.id != b.0.id => {
                return Err(decline("not_a_bore", "the hole is closed at both ends".into()));
            }
            (low, high) => low.or(high),
        }
    } else {
        None
    };
    // Which end is closed: the one nearer the closing face's plane.
    let closed_at = |offset: f64, up: Vec3| -> f64 {
        let along = up.dot(axis);
        (offset - up.dot(origin)) * along
    };
    let (bottom_open, top_open) = match (floor, point) {
        (Some((_, up, offset)), _) | (None, Some((_, up, offset, _))) => {
            let s = closed_at(offset, up);
            if (s - bottom.lo).abs() <= band {
                (false, true)
            } else if (s - top.hi).abs() <= band {
                (true, false)
            } else {
                return Err(decline("not_a_bore", "the hole's floor is not at an end of its walls".into()));
            }
        }
        // No unselected floor or point: an end may still be closed by a
        // SELECTED one (`BREP_REDRILL_CARRIED_FLOORS`), which the motion
        // carries with the walls — a flat cap square to the axis at that end
        // bounded by the walls, or the stack narrowing to a point there.
        (None, None) => {
            let (bottom_closed, top_closed) = if carried_floors_enabled() {
                let wall_edges: HashSet<u64> = revolutions
                    .iter()
                    .flat_map(|(face, ..)| face.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id)))
                    .collect();
                let cap_at = |s_end: f64| {
                    selected.iter().any(|face| {
                        !matches!(face.surface.analytic(), Some(AnalyticSurface::RuledRevolution { .. }))
                            && plane_of_surface(&face.surface, band, OP_STEP).is_ok_and(|plane| (station(plane.origin) - s_end).abs() <= band)
                            && face.loops.iter().flat_map(|lp| &lp.coedges).all(|coedge| wall_edges.contains(&coedge.edge_id))
                    })
                };
                (cap_at(bottom.lo) || bottom.is_point_low(), cap_at(top.hi) || top.is_point_high())
            } else {
                (false, false)
            };
            (!bottom_closed, !top_closed)
        }
    };
    // A floor or point closing one end is not selected and stays where it is,
    // so a translation along the axis has no one answer: refused.
    if (floor.is_some() || point.is_some()) && translation.dot(axis).abs() > band {
        return Err(decline(
            "blind_stepped_along_axis",
            "the hole is closed by a floor that is not selected; a motion along its axis would carry the floor \
             or leave it, so only a motion square to the axis re-drills a blind stepped hole"
                .into(),
        ));
    }
    // A TURN takes a through hole only: a floor or point is not selected and
    // stays where it is, which the turned axis leaves.
    if turn.is_some() && (floor.is_some() || point.is_some()) {
        return Err(decline("turned_blind_stepped_hole", "a turn carries the hole's axis off its floor, which is not selected".into()));
    }
    let mut heal_ids: Vec<u64> = face_ids.to_vec();
    heal_ids.extend(floor.map(|(face, ..)| face.id));
    heal_ids.extend(point.map(|(face, ..)| face.id));
    let healed = match crate::delete_faces_and_heal(solid, &heal_ids) {
        Ok(healed) => healed,
        Err(error) => super::face_move_convex::filled_without(solid, &heal_ids).map_err(|fill| {
            decline("bore_does_not_heal", format!("the hole does not heal shut — {error}; nor is the body the other faces' regions — {fill}"))
        })?,
    };
    let bounds = crate::recentre::solid_bounds(&healed);
    // Past the body wherever the stack is carried: the body's size plus the
    // motion.
    // A pure translation keeps the run-on length it always had (the body's
    // size plus the motion), so the default road is unchanged bit for bit; a
    // turn reads it per end below.
    let translation_reach = bounds.maximum.sub(bounds.minimum).length() + translation.length() + 1.0;
    let (moved_origin, moved_axis) = (map_point(origin), map_dir(axis));
    // The stack where it is, or carried by the motion, as solids (the pieces,
    // the run-on past each open end, the drill point).
    let stack = |carried: bool| -> Result<Vec<BrepSolid>, KernelRefusal> {
        let (origin, axis) = if carried { (moved_origin, moved_axis) } else { (origin, axis) };
        let at = |s: f64| origin.add(axis.scale(s));
        let mut solids = Vec::new();
        for piece in &pieces {
            let solid = if piece.straight {
                make_cylinder_brep(at(piece.lo), axis, piece.r_lo, piece.hi - piece.lo)
            } else if carried_floors_enabled() && piece.is_point_low() {
                // A point at the low end: built from its wide end.
                crate::make_cone_brep(at(piece.hi), axis.scale(-1.0), piece.r_hi, 0.0, piece.hi - piece.lo)
            } else {
                // An apex at the high end is exactly zero (its roundoff can
                // read a hair below).
                let high = if carried_floors_enabled() && piece.is_point_high() { 0.0 } else { piece.r_hi };
                crate::make_cone_brep(at(piece.lo), axis, piece.r_lo, high, piece.hi - piece.lo)
            };
            solids.push(solid.map_err(|error| decline("tool", format!("a piece does not build: {error}")))?);
        }
        // The run-on past an open end continues the end piece's OWN surface (a
        // cone stays that cone, to its apex if it narrows to one), so a carried
        // countersink's wall is the carried cone wherever the body is.
        let run_on = |start: f64, radius: f64, rate: f64, outward: Vec3| -> Result<BrepSolid, KernelRefusal> {
            // Exactly past the body from where the end is carried: the
            // farthest corner of its bounds along the outward axis, and a
            // margin (a turn about a remote pivot moves the end far).
            let from = at(start);
            let corners = [bounds.minimum, bounds.maximum];
            let mut beyond: f64 = 0.0;
            for x in corners {
                for y in corners {
                    for z in corners {
                        beyond = beyond.max(outward.dot(Vec3::new(x.x, y.y, z.z).sub(from)));
                    }
                }
            }
            let reach = if rotation.is_none() {
                translation_reach
            } else {
                beyond + radius.abs() + 0.1 * bounds.maximum.sub(bounds.minimum).length() + 1.0
            };
            let (length, far) = if rate.abs() <= 1e-15 {
                (reach, radius)
            } else if radius + rate * reach > 0.0 {
                (reach, radius + rate * reach)
            } else {
                (radius / -rate, 0.0)
            };
            let built = if (far - radius).abs() <= 1e-12 * radius.max(1.0) {
                make_cylinder_brep(at(start), outward, radius, length)
            } else {
                crate::make_cone_brep(at(start), outward, radius, far, length)
            };
            built.map_err(|error| decline("tool", format!("a run-on does not build: {error}")))
        };
        let slope = |piece: &Piece| if piece.straight { 0.0 } else { (piece.r_hi - piece.r_lo) / (piece.hi - piece.lo) };
        if bottom_open {
            solids.push(run_on(bottom.lo, bottom.r_lo, -slope(&bottom), axis.scale(-1.0))?);
        }
        if top_open {
            solids.push(run_on(top.hi, top.r_hi, slope(&top), axis)?);
        }
        if let Some((_, up, offset, depth)) = point {
            let s = closed_at(offset, up);
            let down = if (s - bottom.lo).abs() <= band { axis.scale(-1.0) } else { axis };
            solids.push(crate::make_cone_brep(at(s), down, if down.dot(axis) < 0.0 { bottom.r_lo } else { top.r_hi }, 0.0, depth).map_err(|error| decline("tool", format!("{error}")))?);
        }
        Ok(solids)
    };
    let cut = |body: &BrepSolid, tools: &[BrepSolid], label: &str| -> Result<BrepSolid, KernelRefusal> {
        let mut body = body.clone();
        for tool in tools {
            body = boolean_operation(&body, tool, BooleanOperation::Subtract, &BooleanOptions::default())
                .map_err(|error| error.with_message(|message| format!("{OP_STEP}: the {label} refused — {message}")))?;
        }
        Ok(body)
    };
    let area = |body: &BrepSolid, label: &str| {
        crate::solid_mass_properties(body)
            .map(|properties| properties.surface_area)
            .map_err(|error| decline("volume", format!("the {label} has no area: {error}")))
    };
    let input_volume = volume(solid, "input body")?;
    let identity = cut(&healed, &stack(false)?, "identity re-drill")?;
    if (volume(&identity, "identity")? - input_volume).abs() > VOLUME_TOLERANCE * input_volume
        || (area(&identity, "identity")? - area(solid, "input")?).abs() > VOLUME_TOLERANCE * area(solid, "input")?
        || outer_shells(&identity)? != outer_shells(solid)?
    {
        return Err(decline(
            "not_a_through_bore",
            "the healed body less the unmoved stack is not the input — the selection is not that hole".into(),
        ));
    }
    let drilled = cut(&healed, &stack(true)?, "re-drill")?;
    let healed_volume = volume(&healed, "healed body")?;
    let largest = pieces.iter().map(|piece| piece.r_lo.max(piece.r_hi)).fold(0.0, f64::max);
    let nothing = (1e-10 * healed_volume).max(1e-6 * std::f64::consts::PI * largest.powi(3));
    if healed_volume - volume(&drilled, "re-drilled body")? <= nothing {
        return Err(decline("carrier_clear_of_body", "the moved hole removes nothing".into()));
    }
    if outer_shells(&drilled)? != outer_shells(&healed)? {
        return Err(decline("bore_severs_body", "the moved hole divides the body".into()));
    }
    let removed = boolean_operation(&healed, &drilled, BooleanOperation::Subtract, &BooleanOptions::default())
        .map_err(|error| error.with_message(|message| format!("{OP_STEP}: the removed-material check refused — {message}")))?;
    if outer_shells(&removed)? != 1 {
        return Err(decline("carrier_cuts_twice", "the moved hole crosses the body in separate pieces".into()));
    }

    // Names: the healed body's planes (and a flat floor, which stays on its
    // plane) by plane, then each carried selected surface (a wall, a step, the
    // point) by its own carried form.
    let floor_plane = floor.and_then(|(face, up, offset)| face.name.clone().map(|name| (name, up, offset)));
    // `restamp` finds a flat floor inside the carried carrier at the floor's
    // radius (no wall names: those come by surface below).
    let carried = Carrier { origin: moved_origin, axis: moved_axis, radius: bottom.r_lo.max(top.r_hi) };
    let mut drilled = restamp(drilled, &healed, Vec::new(), &carried, floor_plane, tolerance);
    let mut taken: HashSet<String> = drilled.shells.iter().flat_map(|shell| &shell.faces).filter_map(|face| face.name.clone()).collect();
    let mut sources: Vec<&FaceRecord> = selected.to_vec();
    sources.extend(point.map(|(face, ..)| face));
    let same_surface = |source: &FaceRecord, other: &FaceRecord| -> bool {
        match (source.surface.analytic(), other.surface.analytic()) {
            (Some(AnalyticSurface::RuledRevolution { frame: a, rho0: a0, rho1: a1, .. }), Some(AnalyticSurface::RuledRevolution { frame: b, rho0: b0, rho1: b1, .. })) => {
                let (carried_origin, carried_axis) = (map_point(a.origin), map_dir(a.axis));
                let offset = b.origin.sub(carried_origin);
                carried_axis.cross(b.axis).length() <= 1e-9
                    && offset.sub(carried_axis.scale(offset.dot(carried_axis))).length() <= band
                    // Same generatrix slope (cylinder or the same cone).
                    && {
                        let slope = |r0: f64, r1: f64, along: Vec3, reference: Vec3, h: f64| (r1 - r0) / h * along.dot(reference).signum();
                        let (ha, hb) = (source.surface.analytic(), other.surface.analytic());
                        let (Some(AnalyticSurface::RuledRevolution { height: ha, .. }), Some(AnalyticSurface::RuledRevolution { height: hb, .. })) = (ha, hb) else { return false };
                        (slope(*a0, *a1, a.axis, axis, *ha) - slope(*b0, *b1, b.axis, moved_axis, *hb)).abs() <= 1e-9
                            && {
                                // The same radius at the same carried station
                                // (stations along the axis survive a rigid motion).
                                let s = moved_axis.dot(b.origin.sub(moved_origin));
                                let base = axis.dot(a.origin.sub(origin));
                                let r_a = a0 + (a1 - a0) * ((s - base) * a.axis.dot(axis).signum() / ha);
                                (r_a - b0).abs() <= band
                            }
                    }
            }
            _ => match (plane_of_surface(&source.surface, band, OP_STEP), plane_of_surface(&other.surface, band, OP_STEP)) {
                (Ok(pa), Ok(pb)) => {
                    let na = if source.same_sense { pa.normal } else { pa.normal.scale(-1.0) };
                    let nb = if other.same_sense { pb.normal } else { pb.normal.scale(-1.0) };
                    let carried_normal = map_dir(na);
                    carried_normal.sub(nb).length() <= 1e-9 && (carried_normal.dot(map_point(pa.origin)) - nb.dot(pb.origin)).abs() <= band
                }
                _ => false,
            },
        }
    };
    // Where along the axis a face lies, from its vertices in its own body: a
    // source along the hole's axis where it was, a result face along the
    // carried axis from the carried origin (the same stations, rigidly).
    let span = |body: &BrepSolid, face: &FaceRecord, carried: bool| -> (f64, f64) {
        let station = |point: Vec3| if carried { moved_axis.dot(point.sub(moved_origin)) } else { axis.dot(point.sub(origin)) };
        face.loops
            .iter()
            .flat_map(|lp| &lp.coedges)
            .filter_map(|coedge| body.edges.iter().find(|edge| edge.id == coedge.edge_id))
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .filter_map(|id| body.vertices.iter().find(|vertex| vertex.id == id))
            .map(|vertex| station(vertex.point))
            .fold((f64::MAX, f64::MIN), |(lo, hi), s| (lo.min(s), hi.max(s)))
    };
    // A wall at an open end owns its run-on too: its stretch runs on without
    // limit that way, so it still names the face its run-on cuts when the
    // carried hole's own stretch has left the body.
    let source_spans: Vec<(f64, f64)> = sources
        .iter()
        .map(|source| {
            let (lo, hi) = span(solid, source, false);
            let lo = if bottom_open && (lo - bottom.lo).abs() <= band { f64::NEG_INFINITY } else { lo };
            let hi = if top_open && (hi - top.hi).abs() <= band { f64::INFINITY } else { hi };
            (lo, hi)
        })
        .collect();
    let drilled_snapshot = drilled.clone();
    for shell in &mut drilled.shells {
        for other in &mut shell.faces {
            if other.name.is_some() {
                continue;
            }
            // The same carried surface AND an overlapping stretch of the axis:
            // two walls of one radius at different depths are different walls.
            let (lo, hi) = span(&drilled_snapshot, other, true);
            let Some(name) = sources
                .iter()
                .zip(&source_spans)
                .find(|(source, (s_lo, s_hi))| same_surface(source, other) && {
                    // A flat step spans one station: touching is enough there.
                    let overlap = hi.min(*s_hi) - lo.max(*s_lo);
                    overlap > band || ((hi - lo <= band || s_hi - s_lo <= band) && overlap >= -band)
                })
                .and_then(|(source, _)| source.name.clone())
            else {
                continue;
            };
            let fresh = std::iter::once(name.clone())
                .chain((1..).map(|index| format!("{name}_{index}")))
                .find(|candidate| !taken.contains(candidate))
                .expect("an unused suffix exists");
            taken.insert(fresh.clone());
            other.name = Some(fresh);
        }
    }
    restore_edge_names(&mut drilled, solid, tolerance * 1e3);
    Ok(drilled)
}

/// `BREP_REDRILL_PRISMATIC_HOLES` (default on; `=0` restores): a hole whose walls are flat
/// faces and cylinders all parallel to one direction — an obround slot, a
/// keyhole — every wall selected, is re-drilled as its own volume stretched
/// along that direction past the body (`redrill_prismatic_hole`).
fn prismatic_holes_enabled() -> bool {
    switch!(live "BREP_REDRILL_PRISMATIC_HOLES", true)
}

/// The row-major affine transform of the rigid motion (turn, then translate).
fn rigid_transform(turn: Option<(Vec3, Vec3, f64)>, translation: Vec3) -> Result<AffineTransform, KernelRefusal> {
    let (columns, origin) = match turn {
        None => ([Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)], Vec3::new(0.0, 0.0, 0.0)),
        Some((point, direction, angle)) => {
            let rotation = super::face_rotate::rotation_about(point, direction.normalized().or_refuse(KernelStage::Collect, "rotation")?, angle)
                .or_refuse(KernelStage::Collect, "rotation")?;
            let image = |axis: Vec3| super::face_rotate::rotate_direction(&rotation, axis);
            ([image(Vec3::new(1.0, 0.0, 0.0)), image(Vec3::new(0.0, 1.0, 0.0)), image(Vec3::new(0.0, 0.0, 1.0))], rotation.point(Vec3::new(0.0, 0.0, 0.0)))
        }
    };
    let o = origin.add(translation);
    AffineTransform::new([
        columns[0].x, columns[1].x, columns[2].x, o.x,
        columns[0].y, columns[1].y, columns[2].y, o.y,
        columns[0].z, columns[1].z, columns[2].z, o.z,
        0.0, 0.0, 0.0, 1.0,
    ])
    .map_err(|error| declined("rotation", format!("the motion is not a transform: {error}")))
}

/// A PRISMATIC hole carried by a rigid motion. Its own volume — the healed
/// body less the input — must be one prism along `direction` (walls parallel
/// to it, flat caps square to it); stretched exactly along `direction` past
/// the body (both ways through, toward the mouth only when an unselected flat
/// floor closes it), it is the tool: the identity (healed less the unmoved
/// tool is the input, volume and area), the material removed, one removed
/// piece and the same bodies are checked as on the other roads, and names come
/// back by each carried selected surface.
#[allow(clippy::too_many_arguments)]
fn redrill_prismatic_hole(
    solid: &BrepSolid,
    face_ids: &[u64],
    selected: &[&FaceRecord],
    direction: Vec3,
    first: Carrier,
    turn: Option<(Vec3, Vec3, f64)>,
    translation: Vec3,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    const OP_PRISM: &str = "transformFace(prismatic hole)";
    let decline = |what: &str, message: String| KernelRefusal::unsupported(KernelStage::Classify, what, format!("{OP_PRISM}: {message}"));
    let band = tolerance * 1e3;
    for face in selected {
        if let Some(carrier) = cylinder_carrier(face, band) {
            if bounds_a_hole(face, &carrier) != Some(true) {
                return Err(decline("not_a_bore", format!("face {} bounds material, not a hole", face.id)));
            }
        }
    }
    // An unselected flat floor square to the direction may close one end; it
    // stays, so only a motion square to the direction is taken then.
    let floor = if blind_bores_enabled() {
        blind_floor(solid, face_ids, &Carrier { origin: first.origin, axis: direction, radius: first.radius }, tolerance)?
    } else {
        None
    };
    if floor.is_some() && (turn.is_some() || translation.dot(direction).abs() > band) {
        return Err(decline(
            "blind_prism_along_axis",
            "the hole's floor is not selected and stays; only a motion square to its walls re-drills it".into(),
        ));
    }
    let mut heal_ids: Vec<u64> = face_ids.to_vec();
    heal_ids.extend(floor.map(|(face, ..)| face.id));
    let healed = match crate::delete_faces_and_heal(solid, &heal_ids) {
        Ok(healed) => healed,
        Err(error) => super::face_move_convex::filled_without(solid, &heal_ids).map_err(|fill| {
            decline("bore_does_not_heal", format!("the hole does not heal shut — {error}; nor is the body the other faces' regions — {fill}"))
        })?,
    };
    let subtract_with = |body: &BrepSolid, tool: &BrepSolid, label: &str| {
        boolean_operation(body, tool, BooleanOperation::Subtract, &BooleanOptions::default())
            .map_err(|error| error.with_message(|message| format!("{OP_PRISM}: the {label} refused — {message}")))
    };
    // The hole's own volume, which must be one prism along the direction.
    let mut hole = subtract_with(&healed, solid, "hole volume")?;
    if outer_shells(&hole)? != 1 {
        return Err(decline("not_prismatic", "the hole's volume is not one piece".into()));
    }
    // The hole's faces inherit the input's names by the boolean's own
    // bookkeeping; the carried tool must bring none of them (a cap took a
    // wall's name), so every name below is given by geometry.
    for face in hole.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        face.name = None;
    }
    for edge in &mut hole.edges {
        edge.name = None;
    }
    // Every WALL of the hole's volume must lie on a selected face (cross
    // review): a notch open to a side has a wall on the side's plane that the
    // healing made, and carried with the rest it would close the notch.
    let on_a_selected_cylinder = |carrier: &Carrier| selected.iter().any(|source| cylinder_carrier(source, band).is_some_and(|own| own.same_line_and_radius(carrier, band)));
    let on_a_selected_plane = |normal: Vec3, origin: Vec3| {
        selected.iter().any(|source| {
            cylinder_carrier(source, band).is_none()
                && plane_of_surface(&source.surface, band, OP_PRISM)
                    .is_ok_and(|own| own.normal.cross(normal).length() <= 1e-9 && (own.normal.dot(origin) - own.normal.dot(own.origin)).abs() <= band)
        })
    };
    let mut caps: Vec<f64> = Vec::new();
    for face in hole.shells.iter().flat_map(|shell| &shell.faces) {
        if let Some(carrier) = cylinder_carrier(face, band) {
            if carrier.axis.cross(direction).length() > 1e-9 {
                return Err(decline("not_prismatic", "a curved wall of the hole is not along the direction".into()));
            }
            if !on_a_selected_cylinder(&carrier) {
                return Err(decline("not_prismatic", "a curved wall of the hole is not a selected face (the hole is open to a side)".into()));
            }
            continue;
        }
        let plane = plane_of_surface(&face.surface, band, OP_PRISM)
            .map_err(|_| decline("not_prismatic", format!("hole face {} is neither a wall along the direction nor a flat cap", face.id)))?;
        if plane.normal.dot(direction).abs() <= 1e-9 {
            if !on_a_selected_plane(plane.normal, plane.origin) {
                return Err(decline("not_prismatic", "a flat wall of the hole is not a selected face (the hole is open to a side)".into()));
            }
            continue;
        }
        if plane.normal.cross(direction).length() > 1e-9 {
            return Err(decline("not_prismatic", "a cap of the hole is not square to its walls".into()));
        }
        let s = direction.dot(plane.origin);
        if !caps.iter().any(|known| (known - s).abs() <= band) {
            caps.push(s);
        }
    }
    if caps.len() != 2 {
        return Err(decline("not_prismatic", format!("the hole has {} cap level(s), not two", caps.len())));
    }
    let (lo, hi) = (caps[0].min(caps[1]), caps[0].max(caps[1]));
    let bounds = crate::recentre::solid_bounds(&healed);
    let motion = rigid_transform(turn, translation)?;
    let m = motion.elements;
    let map_dir = |v: Vec3| Vec3::new(m[0] * v.x + m[1] * v.y + m[2] * v.z, m[4] * v.x + m[5] * v.y + m[6] * v.z, m[8] * v.x + m[9] * v.y + m[10] * v.z);
    let map_point = |p: Vec3| map_dir(p).add(Vec3::new(m[3], m[7], m[11]));
    // Each open end runs on exactly past the body from where the motion
    // carries it: the farthest corner of the body's bounds along the carried
    // outward direction, plus the hole's own width and a margin (a turn about a
    // remote pivot moves an end far, so a length fixed before the motion can
    // stop inside the body and leave a pocket).
    let hole_bounds = crate::recentre::solid_bounds(&hole);
    let centre = hole_bounds.minimum.add(hole_bounds.maximum).scale(0.5);
    let width = hole_bounds.maximum.sub(hole_bounds.minimum).length();
    let margin = width + 0.1 * bounds.maximum.sub(bounds.minimum).length() + 1.0;
    let carried_d = map_dir(direction).normalized().unwrap_or(direction);
    // The same length serves the unmoved identity check, so it is also at
    // least the reach past the body from the UNMOVED end (cross re-review: a
    // motion carrying an end outside the body shortened it, and a hole through
    // only part of the body passed the identity). Any length past the carried
    // reach lies outside the body, so the larger one cuts the same.
    let past = |end: Vec3, out: Vec3| -> f64 {
        let corners = [bounds.minimum, bounds.maximum];
        let mut beyond: f64 = 0.0;
        for x in corners {
            for y in corners {
                for z in corners {
                    beyond = beyond.max(out.dot(Vec3::new(x.x, y.y, z.z).sub(end)));
                }
            }
        }
        beyond
    };
    let end_reach = |level: f64, outward: f64| -> f64 {
        let home = centre.add(direction.scale(level - direction.dot(centre)));
        past(map_point(home), carried_d.scale(outward)).max(past(home, direction.scale(outward))) + margin
    };
    // Which ends run on: both through; with a floor, only the end away from it.
    let (low_open, high_open) = match floor {
        None => (true, true),
        Some((_, up, offset)) => {
            let s = offset * up.dot(direction).signum();
            if (s - lo).abs() <= band {
                (false, true)
            } else if (s - hi).abs() <= band {
                (true, false)
            } else {
                return Err(decline("not_prismatic", "the hole's floor is not at an end of its walls".into()));
            }
        }
    };
    let (new_lo, new_hi) = (if low_open { lo - end_reach(lo, -1.0) } else { lo }, if high_open { hi + end_reach(hi, 1.0) } else { hi });
    let k = (new_hi - new_lo) / (hi - lo);
    let d = direction;
    let shift = d.scale(new_lo - k * lo);
    let stretch = AffineTransform::new([
        1.0 + (k - 1.0) * d.x * d.x, (k - 1.0) * d.x * d.y, (k - 1.0) * d.x * d.z, shift.x,
        (k - 1.0) * d.y * d.x, 1.0 + (k - 1.0) * d.y * d.y, (k - 1.0) * d.y * d.z, shift.y,
        (k - 1.0) * d.z * d.x, (k - 1.0) * d.z * d.y, 1.0 + (k - 1.0) * d.z * d.z, shift.z,
        0.0, 0.0, 0.0, 1.0,
    ])
    .map_err(|error| decline("tool", format!("the stretch is not a transform: {error}")))?;
    let tool_home = crate::transform_brep(&hole, stretch, false).map_err(|error| decline("tool", format!("the stretched hole does not build: {error}")))?;
    // The identity: the healed body less the unmoved tool is the input.
    let area = |body: &BrepSolid, label: &str| {
        crate::solid_mass_properties(body)
            .map(|properties| properties.surface_area)
            .map_err(|error| decline("volume", format!("the {label} has no area: {error}")))
    };
    let input_volume = volume(solid, "input body")?;
    let identity = subtract_with(&healed, &tool_home, "identity re-drill")?;
    if (volume(&identity, "identity")? - input_volume).abs() > VOLUME_TOLERANCE * input_volume
        || (area(&identity, "identity")? - area(solid, "input")?).abs() > VOLUME_TOLERANCE * area(solid, "input")?
        || outer_shells(&identity)? != outer_shells(solid)?
    {
        return Err(decline("not_a_through_bore", "the healed body less the unmoved stretched hole is not the input".into()));
    }
    let tool = crate::transform_brep(&tool_home, motion.clone(), false).map_err(|error| decline("tool", format!("the carried hole does not build: {error}")))?;
    let drilled = subtract_with(&healed, &tool, "re-drill")?;
    let healed_volume = volume(&healed, "healed body")?;
    let hole_volume = volume(&hole, "hole volume")?;
    if healed_volume - volume(&drilled, "re-drilled body")? <= (1e-10 * healed_volume).max(1e-6 * hole_volume) {
        return Err(decline("carrier_clear_of_body", "the moved hole removes nothing".into()));
    }
    if outer_shells(&drilled)? != outer_shells(&healed)? {
        return Err(decline("bore_severs_body", "the moved hole divides the body".into()));
    }
    let removed = subtract_with(&healed, &drilled, "removed-material check")?;
    if outer_shells(&removed)? != 1 {
        return Err(decline("carrier_cuts_twice", "the moved hole crosses the body in separate pieces".into()));
    }
    // Names: each carried selected wall by its own carried surface FIRST (so a
    // flat wall carried onto another named face's plane keeps its own name),
    // then the floor (it stays: only a motion square to the walls is taken
    // with one) as the tool's face on its plane, then the healed body's planes.
    let carried = Carrier { origin: map_point(first.origin), axis: map_dir(first.axis).normalized().unwrap_or(first.axis), radius: first.radius };
    let mut drilled = drilled;
    let mut taken: HashSet<String> = drilled.shells.iter().flat_map(|shell| &shell.faces).filter_map(|face| face.name.clone()).collect();
    for shell in &mut drilled.shells {
        for other in &mut shell.faces {
            if other.name.is_some() {
                continue;
            }
            let matches = |source: &&&FaceRecord| match (cylinder_carrier(source, band), cylinder_carrier(other, band)) {
                (Some(a), Some(b)) => {
                    let moved = Carrier { origin: map_point(a.origin), axis: map_dir(a.axis).normalized().unwrap_or(a.axis), radius: a.radius };
                    moved.same_line_and_radius(&b, band)
                }
                (None, None) => match (plane_of_surface(&source.surface, band, OP_PRISM), plane_of_surface(&other.surface, band, OP_PRISM)) {
                    (Ok(pa), Ok(pb)) => {
                        let na = if source.same_sense { pa.normal } else { pa.normal.scale(-1.0) };
                        let nb = if other.same_sense { pb.normal } else { pb.normal.scale(-1.0) };
                        let carried_normal = map_dir(na).normalized().unwrap_or(na);
                        carried_normal.sub(nb).length() <= 1e-9 && (carried_normal.dot(map_point(pa.origin)) - nb.dot(pb.origin)).abs() <= band
                    }
                    _ => false,
                },
                _ => false,
            };
            // A wall split into several selected faces (a cylinder in halves)
            // gives each result face a name not yet used before suffixing one
            // (cross review: the first piece's name twice lost the second's).
            let found = selected
                .iter()
                .filter(matches)
                .find(|source| source.name.as_ref().is_some_and(|name| !taken.contains(name)))
                .or_else(|| selected.iter().find(matches));
            let Some(name) = found.and_then(|source| source.name.clone()) else { continue };
            let fresh = std::iter::once(name.clone())
                .chain((1..).map(|index| format!("{name}_{index}")))
                .find(|candidate| !taken.contains(candidate))
                .expect("an unused suffix exists");
            taken.insert(fresh.clone());
            other.name = Some(fresh);
        }
    }
    if let Some((name, up, offset)) = floor.and_then(|(face, up, offset)| face.name.clone().map(|name| (name, up, offset))) {
        // Every piece of the carried floor (a boolean may split it): the
        // first takes the name, the others `name_1`, ….
        for shell in &mut drilled.shells {
            for other in &mut shell.faces {
                if other.name.is_some() {
                    continue;
                }
                let Ok(plane) = plane_of_surface(&other.surface, band, OP_PRISM) else { continue };
                let normal = if other.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
                if normal.sub(up).length() <= 1e-9 && (normal.dot(plane.origin) - offset).abs() <= band {
                    let fresh = std::iter::once(name.clone())
                        .chain((1..).map(|index| format!("{name}_{index}")))
                        .find(|candidate| !taken.contains(candidate))
                        .expect("an unused suffix exists");
                    taken.insert(fresh.clone());
                    other.name = Some(fresh);
                }
            }
        }
    }
    let mut drilled = restamp(drilled, &healed, Vec::new(), &carried, None, tolerance);
    restore_edge_names(&mut drilled, solid, tolerance * 1e3);
    Ok(drilled)
}

/// `BREP_REDRILL_CARRIED_BOSS_TOPS` (default on; `=0` restores): a boss whose top is selected
/// with its wall is carried by any rigid motion (turns included), its top with
/// it, the base plane it stands on fixed (`remake_moved_boss`).
fn carried_boss_tops_enabled() -> bool {
    switch!(live "BREP_REDRILL_CARRIED_BOSS_TOPS", true)
}

/// `BREP_REDRILL_BOSSES` (default on; `=0` restores): a cylindrical BOSS's wall — a wall
/// bounding material, with a flat top and standing on one plane, both square
/// to its axis — carried by a translation is re-made as the body with the boss
/// healed away, united with the moved cylinder between the same two planes
/// (`remake_moved_boss`); with its top selected too, any rigid motion
/// (`BREP_REDRILL_CARRIED_BOSS_TOPS`).
fn bosses_enabled() -> bool {
    switch!(live "BREP_REDRILL_BOSSES", true)
}

/// The cylinder on `carrier` between the planes `n̂·x = low` and `n̂·x = high`
/// (`n̂` along the carrier's axis, `low < high`).
fn slab_tool(carrier: &Carrier, up: Vec3, low: f64, high: f64) -> Result<BrepSolid, KernelRefusal> {
    let axis = if carrier.axis.dot(up) >= 0.0 { carrier.axis } else { carrier.axis.scale(-1.0) };
    let rate = up.dot(axis);
    if rate.abs() <= 1e-12 || high <= low {
        return Err(declined("not_a_boss", "the boss's planes are not square to its axis".into()));
    }
    let base = carrier.origin.add(axis.scale((low - up.dot(carrier.origin)) / rate));
    make_cylinder_brep(base, axis, carrier.radius, (high - low) / rate)
        .map_err(|error| declined("tool", format!("the boss cylinder does not build: {error}")))
}

/// A BOSS's wall carried by a translation: the body with the wall and its top
/// healed away, united with the moved carrier between the boss's base plane
/// (the one plane every other edge of the wall lies on, square to the axis —
/// the face it stands on) and its top's plane, neither of which the
/// translation moves. With its TOP selected too (`selected_top`,
/// `BREP_REDRILL_CARRIED_BOSS_TOPS`, default on) the top is carried with the
/// wall by any rigid motion, turns included, and only the base plane stays:
/// the tool is the carried cylinder from the carried top down, clipped to the
/// material side of the base plane (`boss_turned_flat` when the turn lays the
/// axis along or past that plane). Checks, each declining by name: one flat top
/// (`not_a_boss`), one base plane square to the axis (`not_a_boss`), the
/// IDENTITY (the healed body united with the UNMOVED cylinder is the input;
/// `not_a_boss`), no turn unless the top is selected (`turned_boss`), and one
/// body after the move (`boss_leaves_body`).
fn remake_moved_boss(
    solid: &BrepSolid,
    face_ids: &[u64],
    selected: &[&FaceRecord],
    carrier: Carrier,
    turn: Option<(Vec3, Vec3, f64)>,
    translation: Vec3,
    tolerance: f64,
    selected_top: Option<&FaceRecord>,
) -> Result<BrepSolid, KernelRefusal> {
    const OP_BOSS: &str = "transformFace(boss)";
    let decline = |what: &str, message: String| KernelRefusal::unsupported(KernelStage::Classify, what, format!("{OP_BOSS}: {message}"));
    for face in selected {
        if bounds_a_hole(face, &carrier) != Some(false) {
            return Err(decline("not_a_boss", format!("face {} does not bound the boss's material", face.id)));
        }
    }
    if turn.is_some() && selected_top.is_none() {
        return Err(decline(
            "turned_boss",
            "a turn carries the boss's axis off its base and top planes, which are not selected and stay \
             where they are; only a translation re-makes a boss"
                .into(),
        ));
    }
    // The top: unselected (it stays on its plane), or SELECTED with the wall
    // (`BREP_REDRILL_CARRIED_BOSS_TOPS`: it is carried by the motion, turns
    // included, while the base plane the boss stands on stays).
    let (top, up, top_offset) = match selected_top {
        Some(face) => {
            let plane = plane_of_surface(&face.surface, tolerance.max(1e-9) * 1e3, OP_BOSS)
                .map_err(|_| decline("not_a_boss", "the selected top is not a plane".into()))?;
            let normal = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
            if normal.cross(carrier.axis).length() > 1e-9 {
                return Err(decline("not_a_boss", "the selected top is not square to the boss's axis".into()));
            }
            let wall_edge_ids: HashSet<u64> = selected
                .iter()
                .flat_map(|wall| wall.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id)))
                .collect();
            if !face.loops.iter().flat_map(|lp| &lp.coedges).all(|coedge| wall_edge_ids.contains(&coedge.edge_id)) {
                return Err(decline("not_a_boss", "the selected top is not bounded by the boss's wall alone".into()));
            }
            (face, normal, normal.dot(plane.origin))
        }
        None => blind_floor(solid, face_ids, &carrier, tolerance)?
            .ok_or_else(|| decline("not_a_boss", "the boss's wall does not close on one flat top square to its axis".into()))?,
    };
    // The base: every face across a wall edge that is not the top's, one plane.
    let top_edges: HashSet<u64> = top.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id)).collect();
    let wall_edges: HashSet<u64> = selected
        .iter()
        .flat_map(|face| face.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id)))
        .filter(|edge| !top_edges.contains(edge))
        .collect();
    let mut base: Option<f64> = None;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if face_ids.contains(&face.id) || face.id == top.id {
            continue;
        }
        let touches = face.loops.iter().flat_map(|lp| &lp.coedges).any(|coedge| wall_edges.contains(&coedge.edge_id));
        if !touches {
            continue;
        }
        let plane = plane_of_surface(&face.surface, tolerance.max(1e-9) * 1e3, OP_BOSS)
            .map_err(|_| decline("not_a_boss", format!("the boss stands on face {}, which is not a plane", face.id)))?;
        let normal = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
        if normal.sub(up).length() > 1e-9 {
            return Err(decline("not_a_boss", format!("face {} the boss stands on is not square to its axis, facing the top's way", face.id)));
        }
        let offset = normal.dot(plane.origin);
        if base.is_some_and(|known| (known - offset).abs() > tolerance * 1e3) {
            return Err(decline("not_a_boss", "the boss stands on faces at more than one height".into()));
        }
        base = Some(offset);
    }
    let Some(base_offset) = base else {
        return Err(decline("not_a_boss", "the boss's wall stands on no face".into()));
    };
    if top_offset <= base_offset {
        return Err(decline("not_a_boss", "the boss's top is not above the face it stands on".into()));
    }

    let mut heal_ids: Vec<u64> = face_ids.to_vec();
    heal_ids.push(top.id);
    let healed = crate::delete_faces_and_heal(solid, &heal_ids)
        .map_err(|error| decline("not_a_boss", format!("the boss does not heal away — {error}")))?;
    let unite = |body: &BrepSolid, tool: &BrepSolid, label: &str| {
        boolean_operation(body, tool, BooleanOperation::Union, &BooleanOptions::default())
            .map_err(|error| error.with_message(|message| format!("{OP_BOSS}: the {label} refused — {message}")))
    };
    let input_volume = volume(solid, "input body")?;
    let identity = unite(&healed, &slab_tool(&carrier, up, base_offset, top_offset)?, "identity union")?;
    let identity_volume = volume(&identity, "identity union")?;
    // Volume AND area: a different shape of the same volume is not the input.
    let area = |body: &BrepSolid, label: &str| {
        crate::solid_mass_properties(body)
            .map(|properties| properties.surface_area)
            .map_err(|error| decline("volume", format!("the {label} has no area: {error}")))
    };
    let (input_area, identity_area) = (area(solid, "input body")?, area(&identity, "identity union")?);
    if (identity_volume - input_volume).abs() > VOLUME_TOLERANCE * input_volume
        || (identity_area - input_area).abs() > VOLUME_TOLERANCE * input_area
        || identity.shells.len() != solid.shells.len()
    {
        return Err(decline(
            "not_a_boss",
            format!(
                "the healed body united with the unmoved boss cylinder measures {identity_volume} in {} shell(s), \
                 not the input's {input_volume} in {} — the selection is not that boss",
                identity.shells.len(),
                solid.shells.len()
            ),
        ));
    }
    // The rigid motion: turn, then translate.
    let rotation = match turn {
        None => None,
        Some((point, direction, angle)) => Some(
            super::face_rotate::rotation_about(point, direction.normalized().or_refuse(KernelStage::Collect, "rotation")?, angle)
                .or_refuse(KernelStage::Collect, "rotation")?,
        ),
    };
    let map_point = |point: Vec3| match &rotation {
        None => point.add(translation),
        Some(rotation) => rotation.point(point).add(translation),
    };
    let map_dir = |direction: Vec3| match &rotation {
        None => direction,
        Some(rotation) => super::face_rotate::rotate_direction(rotation, direction).normalized().unwrap_or(direction),
    };
    let moved = Carrier { origin: map_point(carrier.origin), axis: map_dir(carrier.axis), radius: carrier.radius };
    // The carried top's plane (the same as the top's when the top stays).
    let (top_up, top_level) = if selected_top.is_some() {
        let on_top = carrier.origin.add(carrier.axis.scale((top_offset - up.dot(carrier.origin)) / up.dot(carrier.axis)));
        let carried_up = map_dir(up);
        (carried_up, carried_up.dot(map_point(on_top)))
    } else {
        (up, top_offset)
    };
    let tool = if selected_top.is_some() {
        // The carried cylinder from the carried top down past the body,
        // clipped to the material side of the base plane by a slab standing
        // on it.
        let axis = if moved.axis.dot(top_up) >= 0.0 { moved.axis } else { moved.axis.scale(-1.0) };
        let cap = moved.origin.add(axis.scale((top_level - top_up.dot(moved.origin)) / top_up.dot(axis)));
        // Exactly past the base plane across the whole disc: the axial run
        // from the carried top to the base plane, and r·tan of the tilt so the
        // rim's lowest point clears it too.
        let cosine = up.dot(axis);
        if cosine <= 1e-3 {
            return Err(decline(
                "boss_turned_flat",
                "the turn lays the boss's axis (nearly) along the plane it stands on, or past it into the body, so \
                 its foot runs off along that plane without end or the boss stands inside the material"
                    .into(),
            ));
        }
        let to_base = (up.dot(cap) - base_offset) / cosine;
        let tilt = carrier.radius * (1.0 - cosine * cosine).max(0.0).sqrt() / cosine;
        let reach = to_base.max(0.0) + tilt + 0.1 * (top_offset - base_offset) + carrier.radius + 1.0;
        let column = make_cylinder_brep(cap.sub(axis.scale(reach)), axis, carrier.radius, reach)
            .map_err(|error| decline("tool", format!("the carried boss column does not build: {error}")))?;
        // The slab stands on the base plane under the carried top, wide and
        // tall enough to hold the whole column: every point of the column lies
        // within `reach + r` of the top's centre.
        let foot = cap.sub(up.scale(up.dot(cap) - base_offset));
        let span = reach + carrier.radius + 1.0;
        let slab = make_cylinder_brep(foot, up, span, (up.dot(cap) - base_offset).abs() + span)
            .map_err(|error| decline("tool", format!("the base slab does not build: {error}")))?;
        boolean_operation(&column, &slab, BooleanOperation::Intersect, &BooleanOptions::default())
            .map_err(|error| error.with_message(|message| format!("{OP_BOSS}: the carried boss clip refused — {message}")))?
    } else {
        slab_tool(&moved, up, base_offset, top_offset)?
    };
    let remade = unite(&healed, &tool, "boss union")?;
    // The moved boss must add material and its wall must be on the result: a
    // boss carried wholly into the body would vanish into it.
    let has_wall = remade
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .any(|face| cylinder_carrier(face, tolerance * 1e3).is_some_and(|own| own.same_line_and_radius(&moved, tolerance * 1e3)));
    // Relative to the boss's own volume, so a small boss on a large body is
    // still read as adding material.
    let boss_volume = std::f64::consts::PI * carrier.radius * carrier.radius * (top_offset - base_offset);
    if volume(&remade, "re-made body")? - volume(&healed, "healed body")? <= 1e-6 * boss_volume || !has_wall {
        return Err(decline(
            "boss_absorbed",
            "the moved boss lies inside the body: it adds no material and its wall is not on the result".into(),
        ));
    }
    // Bodies, not shells: a boss moved over a pocket's mouth seals it into a
    // void, a shell of its own inside the same body.
    let (bodies, before) = (outer_shells(&remade)?, outer_shells(&healed)?);
    if bodies != before {
        return Err(decline(
            "boss_leaves_body",
            format!("the moved boss no longer meets the body: the union is {bodies} bodies, not {before}"),
        ));
    }
    let top_plane = top.name.clone().map(|name| (name, top_up, top_level));
    let mut remade = restamp(remade, &healed, selected.iter().filter_map(|face| face.name.clone()).collect(), &moved, top_plane, tolerance);
    // Where the moved boss overhangs the face it stood on, its foot is a new
    // face on the base plane facing away from the top: `{wall}_foot`.
    if let Some(wall) = selected.iter().find_map(|face| face.name.clone()) {
        let band = tolerance.max(1e-9) * 1e3;
        let inside = |solid: &BrepSolid, face: &FaceRecord| {
            let mut corners = face
                .loops
                .iter()
                .flat_map(|lp| &lp.coedges)
                .filter_map(|coedge| solid.edges.iter().find(|edge| edge.id == coedge.edge_id))
                .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
                .filter_map(|id| solid.vertices.iter().find(|vertex| vertex.id == id))
                .peekable();
            corners.peek().is_some()
                && corners.all(|vertex| {
                    let offset = vertex.point.sub(moved.origin);
                    offset.sub(moved.axis.scale(offset.dot(moved.axis))).length() <= moved.radius + band
                })
        };
        let feet: Vec<u64> = remade
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| face.name.is_none())
            .filter(|face| {
                plane_of_surface(&face.surface, band, OP_BOSS).is_ok_and(|plane| {
                    let normal = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
                    normal.add(up).length() <= 1e-9 && (normal.dot(plane.origin) + base_offset).abs() <= band
                })
            })
            .filter(|face| inside(&remade, face))
            .map(|face| face.id)
            .collect();
        for (index, id) in feet.iter().enumerate() {
            if let Some(face) = remade.shells.iter_mut().flat_map(|shell| &mut shell.faces).find(|face| face.id == *id) {
                face.name = Some(if index == 0 { format!("{wall}_foot") } else { format!("{wall}_foot_{index}") });
            }
        }
    }
    restore_edge_names(&mut remade, solid, tolerance * 1e3);
    Ok(remade)
}

/// `BREP_REDRILL_PRISMATIC_BOSSES` (default off): a boss whose walls are flat
/// faces and cylinders all parallel to one direction — an obround or keyhole
/// boss — every wall and its flat top selected, is carried by any rigid motion
/// while the plane it stands on stays (`remake_prismatic_boss`).
fn prismatic_bosses_enabled() -> bool {
    switch!(live "BREP_REDRILL_PRISMATIC_BOSSES", false)
}

/// A PRISMATIC boss carried by a rigid motion with its top. Its walls and top
/// heal away; the boss's own volume (the input less the healed body) must be
/// one prism along the walls, standing on the base plane (the one plane every
/// other face across a wall edge lies on, facing the top's way) and capped by
/// the top, with every wall on a selected face. Stretched down from its top
/// past where the motion carries it, carried, and clipped to the material side
/// of the base plane, it is united back: the identity (the healed body united
/// with the unmoved tool is the input, volume, area and bodies), material
/// added and one body are checked as on the round boss's road, and names come
/// back by each carried selected surface, the top by its carried plane and a
/// foot over open space as `{wall}_foot`.
#[allow(clippy::too_many_arguments)]
fn remake_prismatic_boss(
    solid: &BrepSolid,
    face_ids: &[u64],
    selected: &[&FaceRecord],
    direction: Vec3,
    turn: Option<(Vec3, Vec3, f64)>,
    translation: Vec3,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    const OP_PBOSS: &str = "transformFace(prismatic boss)";
    let decline = |what: &str, message: String| KernelRefusal::unsupported(KernelStage::Classify, what, format!("{OP_PBOSS}: {message}"));
    let band = tolerance * 1e3;
    let oriented = |face: &FaceRecord| -> Option<(Vec3, f64)> {
        let plane = plane_of_surface(&face.surface, band, OP_PBOSS).ok()?;
        let normal = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
        Some((normal, normal.dot(plane.origin)))
    };
    // The walls (along the direction, bounding material) and the one top.
    let mut tops: Vec<&FaceRecord> = Vec::new();
    for face in selected {
        if let Some(carrier) = cylinder_carrier(face, band) {
            if carrier.axis.cross(direction).length() > 1e-9 {
                return Err(decline("not_prismatic", format!("curved wall {} is not along the boss's direction", face.id)));
            }
            if bounds_a_hole(face, &carrier) != Some(false) {
                return Err(decline("not_a_boss", format!("face {} does not bound the boss's material", face.id)));
            }
            continue;
        }
        let (normal, _) = oriented(face).ok_or_else(|| decline("not_prismatic", format!("face {} is neither a plane nor a cylinder", face.id)))?;
        if normal.dot(direction).abs() <= 1e-9 {
            continue;
        }
        if normal.cross(direction).length() > 1e-9 {
            return Err(decline("not_prismatic", format!("face {} is neither along the walls nor square to them", face.id)));
        }
        tops.push(face);
    }
    let [top] = tops.as_slice() else {
        return Err(decline("not_a_boss", format!("{} faces square to the walls are selected, not the one top", tops.len())));
    };
    let (up, top_offset) = oriented(top).expect("read above");
    let walls: Vec<&FaceRecord> = selected.iter().copied().filter(|face| face.id != top.id).collect();
    let wall_edges: HashSet<u64> = walls.iter().flat_map(|wall| wall.loops.iter().flat_map(|lp| lp.coedges.iter().map(|coedge| coedge.edge_id))).collect();
    if !top.loops.iter().flat_map(|lp| &lp.coedges).all(|coedge| wall_edges.contains(&coedge.edge_id)) {
        return Err(decline("not_a_boss", "the selected top is not bounded by the boss's walls alone".into()));
    }
    // The base: every unselected face across a wall edge, one plane facing up.
    let mut base: Option<f64> = None;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if face_ids.contains(&face.id) || !face.loops.iter().flat_map(|lp| &lp.coedges).any(|coedge| wall_edges.contains(&coedge.edge_id)) {
            continue;
        }
        let (normal, offset) = oriented(face).ok_or_else(|| decline("not_a_boss", format!("the boss stands on face {}, which is not a plane", face.id)))?;
        if normal.sub(up).length() > 1e-9 {
            return Err(decline("not_a_boss", format!("face {} the boss stands on is not square to its walls, facing the top's way", face.id)));
        }
        if base.is_some_and(|known| (known - offset).abs() > band) {
            return Err(decline("not_a_boss", "the boss stands on faces at more than one height".into()));
        }
        base = Some(offset);
    }
    let Some(base_offset) = base else {
        return Err(decline("not_a_boss", "the boss's walls stand on no face".into()));
    };
    if top_offset <= base_offset + band {
        return Err(decline("not_a_boss", "the boss's top is not above the face it stands on".into()));
    }
    let healed = crate::delete_faces_and_heal(solid, face_ids).map_err(|error| decline("not_a_boss", format!("the boss does not heal away — {error}")))?;
    let operate = |a: &BrepSolid, b: &BrepSolid, operation: BooleanOperation, label: &str| {
        boolean_operation(a, b, operation, &BooleanOptions::default())
            .map_err(|error| error.with_message(|message| format!("{OP_PBOSS}: the {label} refused — {message}")))
    };
    // The boss's own volume: one prism between the base and the top, every
    // wall on a selected face (a wall the healing made would be carried too).
    let mut boss = operate(solid, &healed, BooleanOperation::Subtract, "boss volume")?;
    if outer_shells(&boss)? != 1 {
        return Err(decline("not_prismatic", "the boss's volume is not one piece".into()));
    }
    for face in boss.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        face.name = None;
    }
    for edge in &mut boss.edges {
        edge.name = None;
    }
    for face in boss.shells.iter().flat_map(|shell| &shell.faces) {
        if let Some(carrier) = cylinder_carrier(face, band) {
            if carrier.axis.cross(direction).length() > 1e-9
                || !walls.iter().any(|wall| cylinder_carrier(wall, band).is_some_and(|own| own.same_line_and_radius(&carrier, band)))
            {
                return Err(decline("not_prismatic", "a curved wall of the boss's volume is not a selected wall".into()));
            }
            continue;
        }
        let (normal, offset) = oriented(face).ok_or_else(|| decline("not_prismatic", format!("boss face {} is neither a wall nor a cap", face.id)))?;
        if normal.dot(up).abs() <= 1e-9 {
            let on_a_wall = walls.iter().any(|wall| {
                cylinder_carrier(wall, band).is_none() && oriented(wall).is_some_and(|(own, own_offset)| own.sub(normal).length() <= 1e-9 && (own_offset - offset).abs() <= band)
            });
            if !on_a_wall {
                return Err(decline("not_prismatic", "a flat wall of the boss's volume is not a selected wall".into()));
            }
            continue;
        }
        let level = up.dot(normal) * offset;
        if normal.cross(up).length() > 1e-9 || ((level - base_offset).abs() > band && (level - top_offset).abs() > band) {
            return Err(decline("not_prismatic", "a cap of the boss's volume is neither its base nor its top".into()));
        }
    }
    let motion = rigid_transform(turn, translation)?;
    let m = motion.elements;
    let map_dir = |v: Vec3| Vec3::new(m[0] * v.x + m[1] * v.y + m[2] * v.z, m[4] * v.x + m[5] * v.y + m[6] * v.z, m[8] * v.x + m[9] * v.y + m[10] * v.z);
    let map_point = |p: Vec3| map_dir(p).add(Vec3::new(m[3], m[7], m[11]));
    let carried_up = map_dir(up).normalized().unwrap_or(up);
    let cosine = carried_up.dot(up);
    if cosine <= 1e-3 {
        return Err(decline(
            "boss_turned_flat",
            "the turn lays the boss's walls (nearly) along the plane it stands on, or past it into the body".into(),
        ));
    }
    // Stretched down from the top so that, carried, every point of its foot is
    // past the base plane: the carried top's height over the plane and the
    // boss's width, along the carried walls' slope, and a margin.
    let boss_bounds = crate::recentre::solid_bounds(&boss);
    let centre = boss_bounds.minimum.add(boss_bounds.maximum).scale(0.5);
    let width = boss_bounds.maximum.sub(boss_bounds.minimum).length();
    let body_bounds = crate::recentre::solid_bounds(&healed);
    let margin = 0.1 * body_bounds.maximum.sub(body_bounds.minimum).length() + 1.0;
    let top_centre = centre.add(up.scale(top_offset - up.dot(centre)));
    let carried_height = up.dot(map_point(top_centre)) - base_offset;
    let reach = (carried_height.max(0.0) + width) / cosine + width + margin;
    let (lo, hi) = (base_offset, top_offset);
    let k = (hi + reach - lo) / (hi - lo);
    let shift = up.scale(hi * (1.0 - k));
    let d = up;
    let stretch = AffineTransform::new([
        1.0 + (k - 1.0) * d.x * d.x, (k - 1.0) * d.x * d.y, (k - 1.0) * d.x * d.z, shift.x,
        (k - 1.0) * d.y * d.x, 1.0 + (k - 1.0) * d.y * d.y, (k - 1.0) * d.y * d.z, shift.y,
        (k - 1.0) * d.z * d.x, (k - 1.0) * d.z * d.y, 1.0 + (k - 1.0) * d.z * d.z, shift.z,
        0.0, 0.0, 0.0, 1.0,
    ])
    .map_err(|error| decline("tool", format!("the stretch is not a transform: {error}")))?;
    let column = crate::transform_brep(&boss, stretch, false).map_err(|error| decline("tool", format!("the stretched boss does not build: {error}")))?;
    // The column clipped to the material side of the base plane by a slab
    // standing on it under the column's top, wide and tall enough to hold it.
    let clipped = |column: &BrepSolid, cap: Vec3, label: &str| -> Result<BrepSolid, KernelRefusal> {
        let height = up.dot(cap) - base_offset;
        let span = reach + 2.0 * width + 1.0;
        let foot = cap.sub(up.scale(height));
        let slab = make_cylinder_brep(foot, up, span, height.max(0.0) + span)
            .map_err(|error| decline("tool", format!("the base slab does not build: {error}")))?;
        operate(column, &slab, BooleanOperation::Intersect, label)
    };
    let tool_home = clipped(&column, top_centre, "unmoved clip")?;
    let input_volume = volume(solid, "input body")?;
    let area = |body: &BrepSolid, label: &str| {
        crate::solid_mass_properties(body)
            .map(|properties| properties.surface_area)
            .map_err(|error| decline("volume", format!("the {label} has no area: {error}")))
    };
    // The identity: the healed body united with the unmoved clipped column is
    // the input. It is weak by construction (the boss is `solid - healed`, and
    // the column was built from that boss's own walls), so it catches a column
    // or clip that does not rebuild the boss, not a wrong boss reading; the
    // face check above (every boss face a selected wall or a cap) is what
    // proves the reading.
    let identity = operate(&healed, &tool_home, BooleanOperation::Union, "identity union")?;
    if (volume(&identity, "identity union")? - input_volume).abs() > VOLUME_TOLERANCE * input_volume
        || (area(&identity, "identity union")? - area(solid, "input body")?).abs() > VOLUME_TOLERANCE * area(solid, "input body")?
        || outer_shells(&identity)? != outer_shells(solid)?
    {
        return Err(decline("not_a_boss", "the healed body united with the unmoved boss is not the input".into()));
    }
    let carried = crate::transform_brep(&column, motion.clone(), false).map_err(|error| decline("tool", format!("the carried boss does not build: {error}")))?;
    let tool = clipped(&carried, map_point(top_centre), "carried clip")?;
    let remade = operate(&healed, &tool, BooleanOperation::Union, "boss union")?;
    let boss_volume = volume(&boss, "boss volume")?;
    if volume(&remade, "re-made body")? - volume(&healed, "healed body")? <= 1e-6 * boss_volume {
        return Err(decline("boss_absorbed", "the moved boss lies inside the body: it adds no material".into()));
    }
    let (bodies, before) = (outer_shells(&remade)?, outer_shells(&healed)?);
    if bodies != before {
        return Err(decline("boss_leaves_body", format!("the moved boss no longer meets the body: the union is {bodies} bodies, not {before}")));
    }
    // Names: each carried selected surface first (the walls and the top), a
    // piece of a split wall taking a name not yet used before a suffix; then
    // the healed body's planes; then a foot over open space.
    let mut remade = remade;
    let mut taken: HashSet<String> = remade.shells.iter().flat_map(|shell| &shell.faces).filter_map(|face| face.name.clone()).collect();
    for shell in &mut remade.shells {
        for other in &mut shell.faces {
            if other.name.is_some() {
                continue;
            }
            let matches = |source: &&&FaceRecord| match (cylinder_carrier(source, band), cylinder_carrier(other, band)) {
                (Some(a), Some(b)) => {
                    let moved = Carrier { origin: map_point(a.origin), axis: map_dir(a.axis).normalized().unwrap_or(a.axis), radius: a.radius };
                    moved.same_line_and_radius(&b, band)
                }
                (None, None) => match (oriented(source), oriented(other)) {
                    (Some((na, oa)), Some((nb, ob))) => {
                        let carried_normal = map_dir(na).normalized().unwrap_or(na);
                        let on_source = na.scale(oa);
                        carried_normal.sub(nb).length() <= 1e-9 && (carried_normal.dot(map_point(on_source)) - ob).abs() <= band
                    }
                    _ => false,
                },
                _ => false,
            };
            let found = selected
                .iter()
                .filter(matches)
                .find(|source| source.name.as_ref().is_some_and(|name| !taken.contains(name)))
                .or_else(|| selected.iter().find(matches));
            let Some(name) = found.and_then(|source| source.name.clone()) else { continue };
            let fresh = std::iter::once(name.clone())
                .chain((1..).map(|index| format!("{name}_{index}")))
                .find(|candidate| !taken.contains(candidate))
                .expect("an unused suffix exists");
            taken.insert(fresh.clone());
            other.name = Some(fresh);
        }
    }
    let first = walls
        .iter()
        .find_map(|wall| cylinder_carrier(wall, band))
        .map(|carrier| Carrier { origin: map_point(carrier.origin), axis: map_dir(carrier.axis).normalized().unwrap_or(carrier.axis), radius: carrier.radius })
        .unwrap_or(Carrier { origin: map_point(top_centre), axis: carried_up, radius: 0.0 });
    let mut remade = restamp(remade, &healed, Vec::new(), &first, None, tolerance);
    // A foot is only a face under the carried boss: every corner inside the
    // carried tool's box, so an unnamed face of the body elsewhere on the base
    // plane keeps its (absent) name.
    let (low, high) = tool.vertices.iter().fold(
        ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
        |(low, high), vertex| {
            let p = [vertex.point.x, vertex.point.y, vertex.point.z];
            (std::array::from_fn(|i| low[i].min(p[i])), std::array::from_fn(|i| high[i].max(p[i])))
        },
    );
    let under_the_tool = |body: &BrepSolid, face: &FaceRecord| {
        let mut corners = face
            .loops
            .iter()
            .flat_map(|lp| &lp.coedges)
            .filter_map(|coedge| body.edges.iter().find(|edge| edge.id == coedge.edge_id))
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .filter_map(|id| body.vertices.iter().find(|vertex| vertex.id == id))
            .peekable();
        corners.peek().is_some()
            && corners.all(|vertex| {
                let p = [vertex.point.x, vertex.point.y, vertex.point.z];
                (0..3).all(|i| p[i] >= low[i] - band && p[i] <= high[i] + band)
            })
    };
    if let Some(wall) = walls.iter().find_map(|face| face.name.clone()) {
        let feet: Vec<u64> = remade
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| face.name.is_none())
            .filter(|face| oriented(face).is_some_and(|(normal, offset)| normal.add(up).length() <= 1e-9 && (offset + base_offset).abs() <= band))
            .filter(|face| under_the_tool(&remade, face))
            .map(|face| face.id)
            .collect();
        for (index, id) in feet.iter().enumerate() {
            if let Some(face) = remade.shells.iter_mut().flat_map(|shell| &mut shell.faces).find(|face| face.id == *id) {
                face.name = Some(if index == 0 { format!("{wall}_foot") } else { format!("{wall}_foot_{index}") });
            }
        }
    }
    restore_edge_names(&mut remade, solid, band);
    Ok(remade)
}

/// How many of `body`'s shells bound material from outside (a positive signed
/// volume on their own); a void's shell encloses a negative one.
fn outer_shells(body: &BrepSolid) -> Result<usize, KernelRefusal> {
    let mut count = 0;
    for shell in &body.shells {
        let mut alone = body.clone();
        alone.shells = vec![shell.clone()];
        alone.cells.clear();
        alone.mass_properties_cache = Default::default();
        let signed = solid_signed_volume(&alone).map_err(|error| declined("volume", format!("a shell has no volume: {error}")))?;
        if signed > 0.0 {
            count += 1;
        }
    }
    Ok(count)
}

/// Re-drill the selected through-bore wall at its carrier carried by the rigid
/// motion `turn` (axis point, unit axis, angle in radians; applied first) then
/// `translation`. See the module comment for what it checks.
pub fn redrill_moved_bore(
    solid: &BrepSolid,
    face_ids: &[u64],
    turn: Option<(Vec3, Vec3, f64)>,
    translation: Vec3,
) -> Result<BrepSolid, KernelRefusal> {
    let tolerance = solid_model_scale(solid).max(1.0) * 1e-9;
    let selected: Vec<&FaceRecord> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face_ids.contains(&face.id))
        .collect();
    if selected.is_empty() || selected.len() != face_ids.len() {
        return Err(declined("not_a_bore", "the selection is not on the body".into()));
    }
    // A STEPPED hole (`BREP_REDRILL_STEPPED_HOLES`): several coaxial walls —
    // a counterbore, a countersink — selected together.
    let one_cylinder = cylinder_carrier(selected[0], tolerance * 1e3).is_some_and(|first| {
        selected.iter().all(|face| cylinder_carrier(face, tolerance * 1e3).is_some_and(|own| first.same_line_and_radius(&own, tolerance * 1e3)))
    });
    // A PRISMATIC boss (`BREP_REDRILL_PRISMATIC_BOSSES`): walls of planes and
    // cylinders along one direction bounding material, not all on one
    // cylinder, and a flat top.
    if !one_cylinder && prismatic_bosses_enabled() {
        let band = tolerance * 1e3;
        let cylinders: Vec<Carrier> = selected.iter().filter_map(|face| cylinder_carrier(face, band)).collect();
        if let Some(first) = cylinders.first() {
            let direction = first.axis;
            let material = selected.iter().all(|face| {
                cylinder_carrier(face, band).is_none_or(|carrier| carrier.axis.cross(direction).length() <= 1e-9 && bounds_a_hole(face, &carrier) == Some(false))
            });
            let flat_wall = selected.iter().any(|face| {
                cylinder_carrier(face, band).is_none() && plane_of_surface(&face.surface, band, OP).is_ok_and(|plane| plane.normal.dot(direction).abs() <= 1e-9)
            });
            let several = cylinders.iter().any(|carrier| !carrier.same_line_and_radius(first, band));
            if material && (flat_wall || several) {
                return remake_prismatic_boss(solid, face_ids, &selected, direction, turn, translation, tolerance);
            }
        }
    }
    // A PRISMATIC hole (`BREP_REDRILL_PRISMATIC_HOLES`): flat walls and
    // cylinder walls all parallel to one direction (an obround slot, a
    // keyhole), at least one of each.
    if !one_cylinder && prismatic_holes_enabled() {
        let cylinders: Vec<Carrier> = selected.iter().filter_map(|face| cylinder_carrier(face, tolerance * 1e3)).collect();
        if let Some(first) = cylinders.first() {
            let direction = first.axis;
            let parallel = cylinders.iter().all(|carrier| carrier.axis.cross(direction).length() <= 1e-9);
            let planes_along = selected.iter().filter(|face| cylinder_carrier(face, tolerance * 1e3).is_none()).all(|face| {
                plane_of_surface(&face.surface, tolerance * 1e3, OP).is_ok_and(|plane| plane.normal.dot(direction).abs() <= 1e-9)
            });
            let has_plane = selected.iter().any(|face| cylinder_carrier(face, tolerance * 1e3).is_none());
            if parallel && planes_along && has_plane {
                return redrill_prismatic_hole(solid, face_ids, &selected, direction, *first, turn, translation, tolerance);
            }
        }
    }
    // A BOSS with its TOP selected too (`BREP_REDRILL_CARRIED_BOSS_TOPS`): the
    // walls on one cylinder bounding material, and one plane, the top.
    if !one_cylinder && bosses_enabled() && carried_boss_tops_enabled() {
        let (walls, others): (Vec<&FaceRecord>, Vec<&FaceRecord>) =
            selected.iter().copied().partition(|face| cylinder_carrier(face, tolerance * 1e3).is_some());
        if let ([first, ..], [top]) = (walls.as_slice(), others.as_slice()) {
            let carrier = cylinder_carrier(first, tolerance * 1e3).expect("partitioned on it");
            let one_carrier = walls.iter().all(|face| cylinder_carrier(face, tolerance * 1e3).is_some_and(|own| carrier.same_line_and_radius(&own, tolerance * 1e3)));
            if one_carrier && walls.iter().all(|face| bounds_a_hole(face, &carrier) == Some(false)) {
                let wall_ids: Vec<u64> = walls.iter().map(|face| face.id).collect();
                return remake_moved_boss(solid, &wall_ids, &walls, carrier, turn, translation, tolerance, Some(top));
            }
        }
    }
    if !one_cylinder && stepped_holes_enabled() {
        return redrill_stepped_hole(solid, face_ids, &selected, turn, translation, tolerance);
    }
    let carrier = cylinder_carrier(selected[0], tolerance * 1e3)
        .ok_or_else(|| declined("not_a_bore", "the selection is not a cylinder wall".into()))?;
    for face in &selected {
        let own = cylinder_carrier(face, tolerance * 1e3)
            .ok_or_else(|| declined("not_a_bore", format!("face {} is not a cylinder", face.id)))?;
        if !carrier.same_line_and_radius(&own, tolerance * 1e3) {
            return Err(declined("not_a_bore", "the selected faces are not on one cylinder".into()));
        }
    }
    // A BOSS: every selected face on the one carrier (checked above) bounds
    // material.
    if bosses_enabled() && selected.iter().all(|face| bounds_a_hole(face, &carrier) == Some(false)) {
        return remake_moved_boss(solid, face_ids, &selected, carrier, turn, translation, tolerance, None);
    }
    for face in &selected {
        if bounds_a_hole(face, &carrier) != Some(true) {
            return Err(declined("not_a_bore", format!("face {} bounds material, not a hole", face.id)));
        }
    }

    // A BLIND bore: the wall and its floor heal together, and the tool runs
    // from the floor's (fixed) plane out through the mouth.
    let floor = if blind_bores_enabled() { blind_floor(solid, face_ids, &carrier, tolerance)? } else { None };
    // A DRILL POINT instead of a flat floor (`BREP_REDRILL_DRILL_POINTS`).
    let point = if floor.is_none() && blind_bores_enabled() && drill_points_enabled() {
        point_floor(solid, face_ids, &carrier, tolerance)?
    } else {
        None
    };
    if (floor.is_some() || point.is_some()) && turn.is_some() {
        return Err(declined(
            "turned_blind_bore",
            "a turn carries a blind bore's axis off its floor, which is not selected and stays where it is; \
             only a translation re-drills a blind bore"
                .into(),
        ));
    }
    let mut heal_ids: Vec<u64> = face_ids.to_vec();
    if let Some((face, ..)) = floor {
        heal_ids.push(face.id);
    }
    if let Some((face, ..)) = point {
        heal_ids.push(face.id);
    }
    let tool_for = |carrier: &Carrier, body: &BrepSolid| match (floor, point) {
        (Some((_, up, offset)), _) => blind_tool(carrier, up, offset, body),
        (None, Some((_, up, rim, depth))) => point_tool(carrier, up, rim, depth, body),
        (None, None) => through_tool(carrier, body),
    };

    // Healed by Delete Face; where that has no route (a bore through a round
    // bar, whose mouths lie on the bar's wall), filled in as the intersection
    // of every other face's own region. Either way the identity below is what
    // proves the fill.
    let healed = match crate::delete_faces_and_heal(solid, &heal_ids) {
        Ok(healed) => healed,
        Err(error) => super::face_move_convex::filled_without(solid, &heal_ids).map_err(|fill| {
            declined(
                "bore_does_not_heal",
                format!("the bore does not heal shut — {error}; nor is the body the other faces' regions — {fill}"),
            )
        })?,
    };
    let input_volume = volume(solid, "input body")?;
    let healed_volume = volume(&healed, "healed body")?;

    // The identity: the bore IS the unmoved carrier run through the body.
    let unmoved = subtract(&healed, &tool_for(&carrier, &healed)?, "identity re-drill")?;
    let identity_volume = volume(&unmoved, "identity re-drill")?;
    if (identity_volume - input_volume).abs() > VOLUME_TOLERANCE * input_volume
        || unmoved.shells.len() != solid.shells.len()
    {
        return Err(declined(
            "not_a_through_bore",
            format!(
                "the healed body less the unmoved carrier run through it measures {identity_volume} \
                 in {} shell(s), not the input's {input_volume} in {} — the selection is not a \
                 bore running the body's whole way through",
                unmoved.shells.len(),
                solid.shells.len()
            ),
        ));
    }

    // The carrier, carried.
    let (origin, axis) = match turn {
        None => (carrier.origin.add(translation), carrier.axis),
        Some((point, direction, angle)) => {
            let rotation = super::face_rotate::rotation_about(point, direction, angle)
                .or_refuse(KernelStage::Collect, "rotation")?;
            (
                rotation.point(carrier.origin).add(translation),
                super::face_rotate::rotate_direction(&rotation, carrier.axis).normalized().or_refuse(KernelStage::Collect, "rotation")?,
            )
        }
    };
    let moved = Carrier { origin, axis, radius: carrier.radius };
    let tool = tool_for(&moved, &healed)?;
    let drilled = subtract(&healed, &tool, "re-drill")?;
    let drilled_volume = volume(&drilled, "re-drilled body")?;
    // "Removes nothing" read at the bore's own scale (a small bore in a large
    // body still cuts), above the booleans' roundoff on the body.
    let nothing = (1e-10 * healed_volume).max(1e-6 * std::f64::consts::PI * carrier.radius.powi(3));
    if healed_volume - drilled_volume <= nothing {
        return Err(declined(
            "carrier_clear_of_body",
            "the moved carrier, run the body's whole way through, removes nothing: the bore \
             would leave the body and there is no hole left to cut"
                .into(),
        ));
    }
    // Bodies, not shells: a bore that opens an internal void joins a void's
    // shell to the outside without dividing any material.
    let (bodies, before) = (outer_shells(&drilled)?, outer_shells(&healed)?);
    if bodies != before {
        return Err(declined(
            "bore_severs_body",
            format!("the moved bore divides the body into {bodies} bodies (it was {before})"),
        ));
    }
    let removed = boolean_operation(&healed, &tool, BooleanOperation::Intersect, &BooleanOptions::default())
        .map_err(|error| error.with_message(|message| format!("{OP}: the removed-material check refused — {message}")))?;
    let pieces = outer_shells(&removed)?;
    if pieces != 1 {
        return Err(declined(
            "carrier_cuts_twice",
            format!(
                "the moved carrier crosses the body in {pieces} separate pieces: it would drill \
                 material the bore never reached"
            ),
        ));
    }

    let floor_plane = floor.and_then(|(face, up, offset)| face.name.clone().map(|name| (name, up, offset)));
    let mut drilled = restamp(drilled, &healed, selected.iter().filter_map(|face| face.name.clone()).collect(), &moved, floor_plane, tolerance);
    // A drill point comes back as the moved tool's cone: every unnamed cone
    // face on the moved axis takes the point's name (a point split into
    // several patches: `name`, `name_1`, …, skipping any name already on the
    // body).
    if let Some((face, ..)) = point {
        if let Some(name) = face.name.clone() {
            let band = tolerance * 1e3;
            // Never a name another face already carries.
            let mut taken: HashSet<String> = drilled
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .filter_map(|face| face.name.clone())
                .collect();
            for shell in &mut drilled.shells {
                for other in &mut shell.faces {
                    let on_moved = other.name.is_none()
                        && matches!(other.surface.analytic(), Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. })
                            if (rho0 - rho1).abs() > band
                                && frame.axis.cross(moved.axis).length() <= 1e-9
                                && {
                                    let offset = frame.origin.sub(moved.origin);
                                    offset.sub(moved.axis.scale(offset.dot(moved.axis))).length() <= band
                                });
                    if on_moved {
                        let fresh = std::iter::once(name.clone())
                            .chain((1..).map(|index| format!("{name}_{index}")))
                            .find(|candidate| !taken.contains(candidate))
                            .expect("an unused suffix exists");
                        taken.insert(fresh.clone());
                        other.name = Some(fresh);
                    }
                }
            }
        }
    }
    restore_edge_names(&mut drilled, solid, tolerance * 1e3);
    let has_wall = drilled
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .any(|face| cylinder_carrier(face, tolerance * 1e3).is_some_and(|own| own.same_line_and_radius(&moved, tolerance * 1e3)));
    if !has_wall {
        return Err(declined("carrier_clear_of_body", "the moved wall is not on the re-drilled body".into()));
    }
    Ok(drilled)
}

/// Give the re-drilled body its names back: a face on a named face's plane in
/// the healed body takes that name, a face on the moved carrier takes the
/// selection's names in order, and a second face on one carrier takes `name_1`.
fn restamp(
    mut cut: BrepSolid,
    healed: &BrepSolid,
    wall_names: Vec<String>,
    moved: &Carrier,
    floor: Option<(String, Vec3, f64)>,
    tolerance: f64,
) -> BrepSolid {
    let band = tolerance.max(1e-9) * 1e3;
    let oriented_plane = |face: &FaceRecord| -> Option<(Vec3, f64)> {
        let plane = plane_of_surface(&face.surface, band, OP).ok()?;
        let normal = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
        Some((normal, normal.dot(plane.origin)))
    };
    let mut planes: Vec<(String, Vec3, f64)> = Vec::new();
    for face in healed.shells.iter().flat_map(|shell| &shell.faces) {
        let Some(name) = face.name.clone() else { continue };
        if let Some((normal, offset)) = oriented_plane(face) {
            planes.push((name, normal, offset));
        }
    }
    // A blind bore's floor, healed away, comes back as the tool's base: the
    // face on the floor's plane whose every vertex lies inside the moved
    // carrier. Found by that geometry, not by plane alone, so another hole's
    // floor on the same plane never lends it its name.
    let floor_faces: HashMap<u64, String> = match &floor {
        None => HashMap::default(),
        Some((name, up, floor_offset)) => cut
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| face.name.is_none())
            .filter(|face| {
                oriented_plane(face).is_some_and(|(normal, offset)| {
                    normal.sub(*up).length() <= 1e-9 && (offset - floor_offset).abs() <= band
                })
            })
            .filter(|face| {
                let mut corners = face
                    .loops
                    .iter()
                    .flat_map(|lp| &lp.coedges)
                    .filter_map(|coedge| cut.edges.iter().find(|edge| edge.id == coedge.edge_id))
                    .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
                    .filter_map(|id| cut.vertices.iter().find(|vertex| vertex.id == id))
                    .peekable();
                corners.peek().is_some()
                    && corners.all(|vertex| {
                        let offset = vertex.point.sub(moved.origin);
                        offset.sub(moved.axis.scale(offset.dot(moved.axis))).length() <= moved.radius + band
                    })
            })
            .map(|face| (face.id, name.clone()))
            .collect(),
    };
    let mut used: HashMap<String, usize> = HashMap::default();
    for face in cut.shells.iter().flat_map(|shell| &shell.faces) {
        if let Some(name) = &face.name {
            *used.entry(name.clone()).or_insert(0) += 1;
        }
    }
    let mut next_wall = 0usize;
    for shell in &mut cut.shells {
        for face in &mut shell.faces {
            if face.name.is_some() {
                continue;
            }
            let name = if let Some(name) = floor_faces.get(&face.id) {
                name.clone()
            } else if cylinder_carrier(face, band).is_some_and(|own| own.same_line_and_radius(moved, band)) {
                let Some(base) = wall_names.get(next_wall.min(wall_names.len().saturating_sub(1))) else { continue };
                next_wall += 1;
                base.clone()
            } else if let Some((normal, offset)) = oriented_plane(face) {
                let Some((name, ..)) = planes.iter().find(|(_, other, other_offset)| {
                    other.sub(normal).length() <= 1e-9 && (other_offset - offset).abs() <= band
                }) else {
                    continue;
                };
                name.clone()
            } else {
                continue;
            };
            let count = used.entry(name.clone()).or_insert(0);
            *count += 1;
            face.name = Some(if *count == 1 { name } else { format!("{name}_{}", *count - 1) });
        }
    }
    cut
}

/// Edge names for a body a boolean re-made: an authored name (one without
/// `|`) goes back on the edge that runs between the same two points through
/// the same midpoint, and every topology edge then takes its derived
/// `{faceA}|{faceB}[n]` name from the faces it now bounds, as a freshly built
/// body's do.
pub(super) fn restore_edge_names(result: &mut BrepSolid, input: &BrepSolid, band: f64) {
    let ends = |solid: &BrepSolid, edge: &EdgeRecord| -> Option<(Vec3, Vec3, Vec3)> {
        let point = |id: u64| solid.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point);
        let middle = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)).ok()?;
        Some((point(edge.start_vertex_id)?, point(edge.end_vertex_id)?, middle))
    };
    let authored: Vec<(String, (Vec3, Vec3, Vec3))> = input
        .edges
        .iter()
        .filter_map(|edge| {
            let name = edge.name.clone().filter(|name| !name.contains('|'))?;
            Some((name, ends(input, edge)?))
        })
        .collect();
    let near = |a: Vec3, b: Vec3| a.sub(b).length() <= band;
    let found: Vec<(u64, String)> = result
        .edges
        .iter()
        .filter(|edge| edge.name.as_deref().is_none_or(|name| name.contains('|')))
        .filter_map(|edge| {
            let (a, b, m) = ends(result, edge)?;
            authored
                .iter()
                .find(|(_, (p, q, n))| near(m, *n) && ((near(a, *p) && near(b, *q)) || (near(a, *q) && near(b, *p))))
                .map(|(name, _)| (edge.id, name.clone()))
        })
        .collect();
    for (id, name) in found {
        if let Some(edge) = result.edges.iter_mut().find(|edge| edge.id == id) {
            edge.name = Some(name);
        }
    }
    crate::face_names::stamp_derived_edge_names(result);
}
