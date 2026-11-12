//! `BREP_OFFSET_SPHERE_CUT_SHELL` (default on; `=0` restores): the inward shell
//! of a ball cut by planes, built as the source less its cavity.
//!
//! The source is a convex body `B(c, R) ∩ H₁ ∩ … ∩ Hₙ`: a ball and the half-spaces of
//! its planar faces. The inward offset of a solid is its erosion by the ball of
//! radius `d`, and erosion distributes over intersection: a point's `d`-ball lies in
//! every `Hᵢ` and in `B(c, R)` exactly when it lies in their intersection. So the
//! eroded body is `B(c, R − d)` cut by each retained plane moved in by `d`. An
//! opening removes the wall in front of it, so the cavity runs on through an
//! opening's plane, which stays where it is. The shell is the source less that
//! cavity:
//! - the source's faces, kept (`Source`; an opening's face keeps its rim and is the
//!   opening's `Wall`);
//! - the cavity's sphere, on `B(c, R − d)` (`Offset` of the sphere face);
//! - the cavity's faces on moved planes (`Offset` of their plane's face).
//!
//! brep-bench shell/sphere-open-at-a-cap and shell/sphere-opening-sealed: the
//! pipeline refused both with "unwelded rim ... curved-solid opening is not yet
//! supported".
//!
//! The lane builds the cavity with the kernel's sphere and plane split and takes
//! the difference with its boolean. It then checks that difference instead of
//! trusting it: the shell count the geometry asks for, a hole in every opening
//! face the cavity reaches, and the volume and area identities of a difference.
//! A boolean that misses a contact circle answers a body that validates but
//! leaves the cavity sealed (shell/sphere-pinhole-opening, a rim of radius 0.028
//! on a ball of radius 5), and those checks send it back to the pipeline.
//! Anything that is not this configuration declines to the pipeline untouched,
//! with the reason under `BREP_OS_DEBUG`: an outward or tiny distance, a face on
//! another surface or on a second sphere, a body that is not the ball and
//! half-spaces it names, a plane tangent to the cavity, or a cavity the planes cut
//! away entirely. The lane never answers with a refusal the pipeline would not
//! give.
use super::*;
use crate::{boolean_operation, face_area, make_sphere_brep, solid_signed_volume, split_solid_by_plane};
use crate::{AnalyticSurface, BooleanOperation, BooleanOptions};

/// Relative bar for coincidence and membership, against the model scale.
const LINEAR_BAR: f64 = 1e-9;
/// Relative bar for the volume and area identities of the difference.
const MEASURE_BAR: f64 = 1e-9;

/// The switch, default on (`=0` restores the pipeline): the lane takes an inward
/// shell of a ball cut by planes.
fn sphere_cut_shell_on() -> bool {
    switch!("BREP_OFFSET_SPHERE_CUT_SHELL", true)
}


/// The lane's answer, or `None` when it declines (or the switch is off) and the
/// pipeline takes the shell.
pub(super) fn sphere_cut_shell(
    source: &BrepSolid,
    opening_face_ids: &[u64],
    distance: f64,
) -> Option<OffsetShellResultRecord> {
    if !sphere_cut_shell_on() {
        return None;
    }
    match build(source, opening_face_ids, distance) {
        Ok(result) => {
            os_debug!("sphere cut shell lane built {} faces", result.face_images.len());
            Some(result)
        }
        Err(reason) => {
            os_debug!("sphere cut shell lane declined: {reason}");
            None
        }
    }
}

/// A planar face of the source: its outward plane `normal · x = offset`.
struct Plane {
    id: u64,
    normal: Vec3,
    offset: f64,
    opening: bool,
}

impl Plane {
    /// The plane the cavity is cut by: moved in by `distance` unless it is an opening.
    fn cavity_offset(&self, distance: f64) -> f64 {
        if self.opening {
            self.offset
        } else {
            self.offset - distance
        }
    }
}

fn grazing_cavity_on() -> bool {
    switch!("BREP_OFFSET_SHELL_GRAZING_CAVITY", true)
}

fn plane_of(face: &FaceRecord) -> Option<(Vec3, f64)> {
    let Some(AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = face.surface.analytic() else {
        return None;
    };
    let mut normal = u_dir.cross(*v_dir).normalized().ok()?;
    if !face.same_sense {
        normal = normal.scale(-1.0);
    }
    Some((normal, normal.dot(*origin)))
}

/// The ball `B(centre, radius)` cut by `planes` (each keeping its `normal · x ≤
/// offset` side), or `None` when the planes leave nothing. A plane that misses the
/// ball on its kept side does not cut it; one that grazes it (within `linear`)
/// is refused, since the split would leave a sliver or nothing.
fn cut_ball(centre: Vec3, radius: f64, axis: Vec3, planes: &[(Vec3, f64)], linear: f64) -> Result<Option<BrepSolid>, String> {
    let mut body = make_sphere_brep(centre, radius, axis)?;
    for &(normal, offset) in planes {
        // The centre's signed height over the plane, positive on the side cut away.
        let height = normal.dot(centre) - offset;
        if height <= -radius - linear {
            continue;
        }
        if height >= radius - linear {
            return if height >= radius + linear { Ok(None) } else { Err("a plane grazes the ball".into()) };
        }
        if (height + radius).abs() <= linear {
            // GRAZING FROM THE KEPT SIDE (`BREP_OFFSET_SHELL_GRAZING_CAVITY`, default on; `=0` restores): the
            // plane touches the ball at one point with the whole ball on its kept side, so it cuts
            // nothing away. Cause (brep-bench probe, the ball r 5 below z 3 shelled open at its
            // flat with wall 2): the cavity, the ball r 3, touches the opening plane at its top;
            // the lane refused and the pipeline left the offset sphere's trim image as an open rim.
            if grazing_cavity_on() {
                continue;
            }
            return Err("a plane grazes the ball".into());
        }
        let foot = centre.sub(normal.scale(height));
        body = split_solid_by_plane(&body, foot, normal)?.0;
    }
    Ok(Some(body))
}

fn shell_area(solid: &BrepSolid) -> Result<f64, String> {
    let mut total = 0.0;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        total += face_area(face)?;
    }
    Ok(total)
}

fn build(source: &BrepSolid, opening_face_ids: &[u64], distance: f64) -> Result<OffsetShellResultRecord, String> {
    if !(distance.is_finite() && distance > 0.0) {
        return Err("not a finite inward distance".into());
    }
    if source.kind != crate::BodyKind::Solid {
        return Err("not a solid".into());
    }
    let [shell] = &source.shells[..] else {
        return Err(format!("{} shells, not one", source.shells.len()));
    };
    if opening_face_ids.is_empty() {
        return Err("no opening".into());
    }
    if opening_face_ids.iter().any(|id| !shell.faces.iter().any(|face| face.id == *id)) {
        return Err("an opening face is not in the source".into());
    }
    let scale = crate::solid_scale(source).max(1.0);
    let linear = LINEAR_BAR * scale;

    // The faces: every one on one sphere or on a plane.
    let mut sphere = None::<(Vec3, f64, u64)>;
    let mut planes = Vec::<Plane>::new();
    for face in &shell.faces {
        let opening = opening_face_ids.contains(&face.id);
        if let Some((normal, offset)) = plane_of(face) {
            planes.push(Plane { id: face.id, normal, offset, opening });
            continue;
        }
        let Some((centre, radius)) = face.surface.analytic().and_then(|shape| shape.sphere_geometry()) else {
            return Err(format!("face {} is neither a plane nor a sphere", face.id));
        };
        if opening {
            return Err(format!("opening face {} is not planar", face.id));
        }
        match sphere {
            None => sphere = Some((centre, radius, face.id)),
            Some((first, first_radius, _)) => {
                if first.sub(centre).length() > linear || (first_radius - radius).abs() > linear {
                    return Err(format!("face {} is on a second sphere", face.id));
                }
            }
        }
        // The face's own inward move: radial, exactly `distance` long, toward the centre.
        let [u0, u1] = face.surface.domain_u()?;
        let [v0, v1] = face.surface.domain_v()?;
        let sample = crate::OffsetEvaluator::new("offset_shell", &face.surface, crate::OffsetNormal::FaceStable { same_sense: face.same_sense })
            .at((u0 + u1) * 0.5, (v0 + v1) * 0.5, -distance)
            .map_err(|error| format!("face {}: {error}", face.id))?;
        let (from, to) = (sample.source.sub(centre), sample.point.sub(centre));
        if (from.length() - radius).abs() > 1e2 * linear || ((radius - to.length()) - distance).abs() > 1e2 * linear {
            return Err(format!("face {}: the inward offset is not the concentric sphere", face.id));
        }
    }
    let Some((centre, radius, sphere_face)) = sphere else {
        return Err("no sphere face".into());
    };
    let inner = radius - distance;
    if !(inner > 1e-6 * radius) {
        return Err("the offset passes the centre".into());
    }
    if !planes.iter().any(|plane| plane.opening) {
        return Err("no planar opening".into());
    }

    // The source must BE the ball and the half-spaces it names: every vertex and edge
    // sample inside each, and the same volume as the ball cut by its planes.
    for vertex in &source.vertices {
        if vertex.point.sub(centre).length() > radius + 1e2 * linear {
            return Err("a vertex outside the ball".into());
        }
    }
    for edge in &source.edges {
        for step in 0..=8 {
            let point = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * step as f64 / 8.0)?;
            if point.sub(centre).length() > radius + 1e2 * linear
                || planes.iter().any(|plane| plane.normal.dot(point) - plane.offset > 1e2 * linear)
            {
                return Err(format!("edge {} leaves the ball or a half-space", edge.id));
            }
        }
    }
    let axis = planes.iter().find(|plane| plane.opening).map(|plane| plane.normal.scale(-1.0)).ok_or("no opening plane")?;
    let source_planes = planes.iter().map(|plane| (plane.normal, plane.offset)).collect::<Vec<_>>();
    let hull = cut_ball(centre, radius, axis, &source_planes, linear)?.ok_or("the planes leave nothing of the ball")?;
    let source_volume = solid_signed_volume(source)?;
    if (solid_signed_volume(&hull)? - source_volume).abs() > MEASURE_BAR * source_volume.abs().max(1.0) {
        return Err("the source is not the ball cut by its planes".into());
    }

    // The cavity: the ball moved in, cut by each retained plane moved in and each
    // opening plane in place. Its pole points away from the first opening, so the
    // opening's cut is a latitude and the pole is cut away or far from it.
    let cavity_planes = planes.iter().map(|plane| (plane.normal, plane.cavity_offset(distance))).collect::<Vec<_>>();
    let cavity = cut_ball(centre, inner, axis, &cavity_planes, linear)?.ok_or("the moved planes leave no cavity")?;
    let cavity_volume = solid_signed_volume(&cavity)?;
    // A WHOLE ball sealed below every opening (the grazing case: it touches an opening plane at
    // one point) is the source with the ball as a void: the boolean reads a single point of
    // contact wrong (the ball r 5 below z 3 at wall 2 read 222.006 for 356.047).
    let whole_ball = cavity.shells.iter().flat_map(|shell| &shell.faces).all(|face| plane_of(face).is_none());
    let sealed_below = !planes.iter().any(|plane| {
        let height = plane.normal.dot(centre) - plane.offset;
        plane.opening && height.abs() < inner - linear
    });
    let result = if grazing_cavity_on() && whole_ball && sealed_below {
        with_void(source, &cavity)?
    } else {
        boolean_operation(source, &cavity, BooleanOperation::Subtract, &BooleanOptions::default())
            .map_err(|refusal| format!("the difference refused: {}", refusal.message))?
    };
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(format!("the difference does not validate: {issues:?}"));
    }

    // The checks a missed contact would fail. Which opening planes the cavity reaches:
    let reaches = |plane: &Plane| -> bool {
        let height = plane.normal.dot(centre) - plane.offset;
        plane.opening && height.abs() < inner - linear
    };
    let open = planes.iter().any(reaches);
    let shells_owed = if open { 1 } else { 2 };
    if result.shells.len() != shells_owed {
        return Err(format!("the difference has {} shells, the geometry asks for {shells_owed}", result.shells.len()));
    }
    let on_plane = |face: &FaceRecord, normal: Vec3, offset: f64| -> bool {
        plane_of(face).is_some_and(|(n, o)| n.sub(normal).length() <= 1e-9 && (o - offset).abs() <= 1e2 * linear)
    };
    let result_faces = result.shells.iter().flat_map(|shell| &shell.faces).collect::<Vec<_>>();
    for plane in planes.iter().filter(|plane| reaches(plane)) {
        let faces = result_faces.iter().filter(|face| on_plane(face, plane.normal, plane.offset)).collect::<Vec<_>>();
        let loops = faces.iter().map(|face| face.loops.len()).sum::<usize>();
        if faces.is_empty() || loops <= faces.len() {
            return Err(format!("opening face {} keeps no hole where the cavity reaches it", plane.id));
        }
    }
    let result_volume = solid_signed_volume(&result)?;
    let owed_volume = source_volume - cavity_volume;
    if (result_volume - owed_volume).abs() > MEASURE_BAR * source_volume.abs().max(1.0) {
        return Err(format!("the difference reads {result_volume} against {owed_volume}"));
    }
    let mut owed_area = shell_area(source)?;
    for face in cavity.shells.iter().flat_map(|shell| &shell.faces) {
        let area = face_area(face)?;
        if planes.iter().any(|plane| plane.opening && on_plane(face, plane.normal, plane.offset)) {
            owed_area -= area;
        } else {
            owed_area += area;
        }
    }
    let result_area = shell_area(&result)?;
    if (result_area - owed_area).abs() > MEASURE_BAR * owed_area.abs().max(1.0) {
        return Err(format!("the difference's area reads {result_area} against {owed_area}"));
    }

    // Each face's provenance, read off its carrier.
    let mut face_images = Vec::with_capacity(result_faces.len());
    for face in &result_faces {
        let image = if let Some((normal, offset)) = plane_of(face) {
            let same = |plane: &&Plane| plane.normal.sub(normal).length() <= 1e-9 && (plane.offset - offset).abs() <= 1e2 * linear;
            // A moved plane's face bounds the cavity, so it faces the other way.
            let moved = |plane: &&Plane| {
                !plane.opening
                    && plane.normal.add(normal).length() <= 1e-9
                    && (plane.offset - distance + offset).abs() <= 1e2 * linear
            };
            if let Some(plane) = planes.iter().find(same) {
                let role = if plane.opening { OffsetFaceRole::Wall } else { OffsetFaceRole::Source };
                OffsetShellFaceImageRecord { role, source_face_id: plane.id }
            } else if let Some(plane) = planes.iter().find(moved) {
                OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: plane.id }
            } else {
                return Err(format!("result face {} is on no source or moved plane", face.id));
            }
        } else {
            match face.surface.analytic().and_then(|shape| shape.sphere_geometry()) {
                Some((at, r)) if at.sub(centre).length() <= 1e2 * linear && (r - radius).abs() <= 1e2 * linear => {
                    OffsetShellFaceImageRecord { role: OffsetFaceRole::Source, source_face_id: sphere_face }
                }
                Some((at, r)) if at.sub(centre).length() <= 1e2 * linear && (r - inner).abs() <= 1e2 * linear => {
                    OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: sphere_face }
                }
                _ => return Err(format!("result face {} is on neither sphere", face.id)),
            }
        };
        face_images.push(image);
    }
    Ok(OffsetShellResultRecord { solid: result, face_images })
}

/// `solid` with `void` added as an inner shell, its faces reversed and every id of it moved past
/// `solid`'s own.
fn with_void(solid: &BrepSolid, void: &BrepSolid) -> Result<BrepSolid, String> {
    let mut hollow = void.clone();
    crate::offset_shell::flip_all_faces(&mut hollow).map_err(|refusal| refusal.message)?;
    let next = |ids: &mut dyn Iterator<Item = u64>| ids.max().unwrap_or(0) + 1;
    let vertex_base = next(&mut solid.vertices.iter().map(|vertex| vertex.id));
    let edge_base = next(&mut solid.edges.iter().map(|edge| edge.id));
    let faces = solid.shells.iter().flat_map(|shell| &shell.faces);
    let face_base = next(&mut faces.clone().map(|face| face.id));
    let loop_base = next(&mut faces.clone().flat_map(|face| face.loops.iter().map(|loop_record| loop_record.id)));
    let coedge_base = next(&mut faces.flat_map(|face| face.loops.iter().flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.id))));
    let shell_base = next(&mut solid.shells.iter().map(|shell| shell.id));
    let mut result = solid.clone();
    for mut vertex in hollow.vertices {
        vertex.id += vertex_base;
        result.vertices.push(vertex);
    }
    for mut edge in hollow.edges {
        edge.id += edge_base;
        edge.start_vertex_id += vertex_base;
        edge.end_vertex_id += vertex_base;
        result.edges.push(edge);
    }
    for mut shell in hollow.shells {
        shell.id += shell_base;
        for face in &mut shell.faces {
            face.id += face_base;
            for loop_record in &mut face.loops {
                loop_record.id += loop_base;
                for coedge in &mut loop_record.coedges {
                    coedge.id += coedge_base;
                    coedge.edge_id += edge_base;
                }
            }
        }
        result.shells.push(shell);
    }
    result.mass_properties_cache = Default::default();
    Ok(result)
}
