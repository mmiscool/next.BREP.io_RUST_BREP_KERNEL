//! Public one-call operations on a solid's faces named by points.

use crate::{thicken_sheet_faces, BrepSolid, KernelRefusal, KernelStage, OffsetCorners, Vec3};
use std::collections::BTreeSet;

/// The solid the faces of `solid` that `face_points` lie on sweep moving from 0
/// to `thickness` along their outward normals (inward where it is negative);
/// `solid` itself is not part of it. The faces are taken as one sheet, with the
/// edges they share, and thickened by [`thicken_sheet_faces`] with sharp joins.
pub fn thicken_faces(solid: &BrepSolid, face_points: &[Vec3], thickness: f64) -> Result<BrepSolid, KernelRefusal> {
    if face_points.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "thicken_faces_points", "thicken_faces: no face named".to_string()));
    }
    let mut faces = BTreeSet::new();
    for &point in face_points {
        faces.insert(crate::resolve_face_by_point(solid, point)?);
    }
    let sheet = crate::feature_pipeline::features::sew_to_solid::extract(solid, &faces);
    thicken_sheet_faces(&[sheet], thickness, OffsetCorners::Sharp)
}

/// How two solids meet, as they stand: the volume they share and the gap
/// between them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Interference {
    /// The volume of their intersection: above 0 where they overlap.
    pub volume: f64,
    /// The least distance from a point of one to a point of the other: 0
    /// where they overlap or touch.
    pub gap: f64,
}

/// How `a` and `b` meet: the volume of their intersection (the exact boolean's,
/// 0 when their least boxes lie apart) and, when they share none, the least
/// distance between their boundaries on their exact geometry
/// ([`crate::solid_gap`]; 0 where they touch). `tolerance` (mm) is the band a
/// point may stand past a face's trim and still count as on it.
pub fn solid_interference(a: &BrepSolid, b: &BrepSolid, tolerance: f64) -> Result<Interference, KernelRefusal> {
    let [la, ha] = crate::solid_least_box(a, tolerance)?;
    let [lb, hb] = crate::solid_least_box(b, tolerance)?;
    let apart = la.x > hb.x || lb.x > ha.x || la.y > hb.y || lb.y > ha.y || la.z > hb.z || lb.z > ha.z;
    if apart {
        return Ok(Interference { volume: 0.0, gap: crate::solid_gap(a, b, tolerance)? });
    }
    let scale = crate::solid_model_scale(a).max(crate::solid_model_scale(b)).max(1.0);
    // Boundaries that stand apart leave each shell of one wholly inside or
    // wholly outside the other, which one point of the shell tells; solids
    // apart need no boolean (whose faces may lie too near one another for it
    // to cut).
    let gap = crate::solid_gap(a, b, tolerance)?;
    let meet = gap <= tolerance.max(1e-9 * scale);
    if !meet && !any_shell_inside(a, b, tolerance)? && !any_shell_inside(b, a, tolerance)? {
        return Ok(Interference { volume: 0.0, gap });
    }
    let common = crate::boolean_operation(a, b, crate::BooleanOperation::Intersect, &crate::BooleanOptions::default())?;
    let volume = if common.shells.iter().all(|shell| shell.faces.is_empty()) {
        0.0
    } else {
        crate::solid_signed_volume(&common)
            .map_err(|e| KernelRefusal::internal(KernelStage::Validate, "interference_volume", format!("interference: {e}")))?
            .max(0.0)
    };
    // Where the boundaries meet, a shared volume no more than the smaller
    // part's broadest side times the round-off of its coordinates is faces in
    // contact, not an overlap; a part inside the other shares what it shares.
    let side = |l: Vec3, h: Vec3| {
        let mut e = [h.x - l.x, h.y - l.y, h.z - l.z];
        e.sort_by(f64::total_cmp);
        e[1] * e[2]
    };
    if volume > 0.0 && (!meet || volume > 1e-10 * scale * side(la, ha).min(side(lb, hb))) {
        return Ok(Interference { volume, gap: 0.0 });
    }
    Ok(Interference { volume: 0.0, gap })
}

/// Whether a point of any shell of `inner` (where its first face's first trim
/// begins, or its surface's middle for a face with no trim) lies inside `outer`.
fn any_shell_inside(inner: &BrepSolid, outer: &BrepSolid, tolerance: f64) -> Result<bool, KernelRefusal> {
    for shell in &inner.shells {
        let Some(face) = shell.faces.first() else { continue };
        let uv = match face.loops.iter().flat_map(|l| &l.coedges).next() {
            Some(coedge) => coedge.pcurve.domain().and_then(|[t, _]| coedge.pcurve.evaluate(t)).map(|p| [p.x, p.y]),
            None => face.surface.domain_u().and_then(|[u0, u1]| face.surface.domain_v().map(|[v0, v1]| [0.5 * (u0 + u1), 0.5 * (v0 + v1)])),
        };
        let point = uv.and_then(|[u, v]| face.surface.evaluate(u, v));
        let point = point.map_err(|e| KernelRefusal::internal(KernelStage::Classify, "interference_inside", format!("interference: {e}")))?;
        let class = crate::classify_point(point, outer, tolerance)
            .map_err(|e| KernelRefusal::internal(KernelStage::Classify, "interference_inside", format!("interference: {e}")))?;
        if class.class != crate::PointClass::Out {
            return Ok(true);
        }
    }
    Ok(false)
}

/// The faces of `solid` the points name, deduplicated in order: each point
/// must lie inside one face's trim (within 1e-6 of the model's scale), not on
/// an edge between faces, nor off them (a point in a hole's mouth names no
/// face, though the cap's carrier runs through it).
fn faces_named(solid: &BrepSolid, face_points: &[Vec3], what: &str) -> Result<Vec<u64>, KernelRefusal> {
    if face_points.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, format!("{what}_points"), format!("{what}: no face named")));
    }
    let tolerance = (crate::solid_model_scale(solid) * 1e-6).max(1e-9);
    let mut seen = BTreeSet::new();
    let mut faces = Vec::new();
    for &point in face_points {
        let foot = crate::nearest_boundary_point(solid, point, tolerance)?;
        let face = match foot {
            // Off the body first, whatever its nearest boundary point is.
            Some(crate::BoundaryFoot { distance, .. }) if !(distance <= tolerance) => {
                return Err(KernelRefusal::input(KernelStage::Collect, "point_off_faces", format!("{what}: the point {point:?} is {distance:e} from the boundary, off every face")));
            }
            Some(crate::BoundaryFoot { place: crate::BoundaryPlace::Face { face, .. }, .. }) => face,
            // On an edge or vertex: it names a face only where every face
            // round it is that one (a seam).
            Some(foot) => {
                let edges: Vec<u64> = match foot.place {
                    crate::BoundaryPlace::Edge { edge, .. } => vec![edge],
                    crate::BoundaryPlace::Vertex { vertex } => solid.edges.iter().filter(|e| e.start_vertex_id == vertex || e.end_vertex_id == vertex).map(|e| e.id).collect(),
                    crate::BoundaryPlace::Face { face, .. } => vec![face],
                };
                let round: BTreeSet<u64> = solid
                    .shells
                    .iter()
                    .flat_map(|shell| &shell.faces)
                    .filter(|face| face.loops.iter().flat_map(|lp| &lp.coedges).any(|coedge| edges.contains(&coedge.edge_id)))
                    .map(|face| face.id)
                    .collect();
                match (round.len(), round.iter().next()) {
                    (1, Some(&face)) => face,
                    _ => {
                        return Err(KernelRefusal::input(KernelStage::Collect, "point_on_edge", format!("{what}: the point {point:?} lies on an edge or vertex between faces, naming none")));
                    }
                }
            }
            None => return Err(KernelRefusal::input(KernelStage::Collect, "no_faces", format!("{what}: the solid has no faces"))),
        };
        if seen.insert(face) {
            faces.push(face);
        }
    }
    Ok(faces)
}

/// `solid` with the faces `face_points` lie on each moved `distance` along
/// its outward normal onto its offset surface (inward where it is negative),
/// the faces round them extended or trimmed to meet them: Push Face's push
/// (a plane translated, a cylinder, cone, sphere or torus offset exactly, a
/// general revolution or free-form carrier within a bounded residual), then
/// its acceptance. A plane landed exactly on a neighbour (two tops merged, a
/// slot closed) is the boolean the push defines; a plane carried past one (an
/// edge between them turned over) is refused as ill-posed. Exact to the push's tolerance, max(1e-7 of the model's
/// scale, 1e-9); a nonzero distance below it is refused by name.
pub fn offset_faces(solid: &BrepSolid, face_points: &[Vec3], distance: f64) -> Result<BrepSolid, KernelRefusal> {
    if !distance.is_finite() {
        return Err(KernelRefusal::input(KernelStage::Collect, "offset_faces_distance", "offset_faces: the distance must be finite"));
    }
    let faces = faces_named(solid, face_points, "offset_faces")?;
    if distance == 0.0 {
        return Ok(solid.clone());
    }
    // A distance the push's tolerance cannot tell from rest is refused by
    // name, never returned as the body unmoved.
    let resolution = (crate::solid_model_scale(solid) * 1e-7).max(1e-9);
    if distance.abs() < resolution {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "motion_below_tolerance",
            format!("offset_faces: the distance {distance:e} is below the {resolution:e} the push reads points to"),
        ));
    }
    let mut notes = Vec::new();
    crate::feature_pipeline::features::push_face::push_faces(solid.clone(), &faces, distance, &mut notes, false).map_err(|refusal| match refusal {
        crate::feature_pipeline::FeatureRefusal::Typed(refusal) => refusal,
        crate::feature_pipeline::FeatureRefusal::Text(message) => KernelRefusal::unsupported(KernelStage::Collect, "offset_faces", message),
    })
}

/// The turn a rotation matrix makes: its unit axis and angle in (0, π], or
/// None for the identity. Read from the skew part where the angle is clear of
/// π, and from the symmetric part (its largest diagonal) near π, where the
/// skew part vanishes.
fn rotation_turn(r: &[[f64; 3]; 3]) -> Option<(Vec3, f64)> {
    let trace = r[0][0] + r[1][1] + r[2][2];
    let skew = Vec3::new(r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]);
    // Its length scaled first, so a tiny turn's skew part does not square to zero.
    let big = skew.x.abs().max(skew.y.abs()).max(skew.z.abs());
    let skew_length = if big > 0.0 { big * Vec3::new(skew.x / big, skew.y / big, skew.z / big).length() } else { 0.0 };
    let sine = 0.5 * skew_length;
    let cosine = (0.5 * (trace - 1.0)).clamp(-1.0, 1.0);
    let angle = sine.atan2(cosine);
    if !(angle > 0.0) {
        return None;
    }
    if cosine > -0.5 {
        return Some((Vec3::new(skew.x / big, skew.y / big, skew.z / big).normalized().ok()?, angle));
    }
    // Near π: R + Rᵀ = 2 cos θ I + 2 (1 − cos θ) a aᵀ.
    let k = 1.0 - cosine;
    let column = (0..3).max_by(|&i, &j| r[i][i].total_cmp(&r[j][j])).unwrap_or(0);
    let mut a = [0.0; 3];
    a[column] = ((r[column][column] - cosine) / k).max(0.0).sqrt();
    for i in 0..3 {
        if i != column {
            a[i] = (r[i][column] + r[column][i]) / (2.0 * k * a[column]);
        }
    }
    let mut axis = Vec3::new(a[0], a[1], a[2]).normalized().ok()?;
    // The skew part fixes the sign where it reads at all.
    if axis.dot(skew) < 0.0 {
        axis = axis.scale(-1.0);
    }
    Some((axis, angle))
}

/// `solid` with the faces `face_points` lie on moved rigidly by the rows
/// `[R | t]` (a point x to R·x + t), the faces round them extended or trimmed
/// to meet them: Transform Face's motion road (the turn and the slide solved
/// in one re-intersection, its acceptance, and the re-cut and re-drill roads
/// past a refusal). R must be a rotation (orthonormal, determinant +1).
///
/// The result is exact to the re-solve's tolerance, max(1e-7 of the model's
/// scale, 1e-9): a part of the motion below it (a lift of 5e-10 riding on a
/// slide along the face) may be read as rest. Within its envelope a whole
/// motion below that tolerance, or a slide whose 1e-9 passes it, is refused
/// by name; only R exactly the identity with no slide returns the body as it
/// is.
pub fn transform_faces(solid: &BrepSolid, face_points: &[Vec3], matrix: [[f64; 4]; 3]) -> Result<BrepSolid, KernelRefusal> {
    if !matrix.iter().flatten().all(|x| x.is_finite()) {
        return Err(KernelRefusal::input(KernelStage::Collect, "transform_faces_matrix", "transform_faces: the matrix must be finite"));
    }
    let r = [[matrix[0][0], matrix[0][1], matrix[0][2]], [matrix[1][0], matrix[1][1], matrix[1][2]], [matrix[2][0], matrix[2][1], matrix[2][2]]];
    let t = Vec3::new(matrix[0][3], matrix[1][3], matrix[2][3]);
    let mut off = 0.0f64;
    for i in 0..3 {
        for j in 0..3 {
            let dot: f64 = (0..3).map(|k| r[k][i] * r[k][j]).sum();
            off = off.max((dot - if i == j { 1.0 } else { 0.0 }).abs());
        }
    }
    let det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    if !(off <= 1e-9 && det > 0.0) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "transform_faces_rigid",
            format!("transform_faces: the matrix's R is not a rotation (off orthonormal by {off:e}, determinant {det}): a face transform is a rigid motion"),
        ));
    }
    let faces = faces_named(solid, face_points, "transform_faces")?;
    // The turn about the selection's centre p (conditioning only: the motion
    // is the same), the slide what is left: R(x − p) + p + s = R·x + t, so
    // s = t + R·p − p carries all of a turn's displacement at p however small
    // its angle. The turn itself is dropped only where it moves no corner of
    // the body by more than 1e-12 of the model's scale.
    let pivot = crate::feature_pipeline::features::transform_face::selection_centre(solid, &faces).unwrap_or(Vec3::new(0.0, 0.0, 0.0));
    let turned = Vec3::new(
        r[0][0] * pivot.x + r[0][1] * pivot.y + r[0][2] * pivot.z,
        r[1][0] * pivot.x + r[1][1] * pivot.y + r[1][2] * pivot.z,
        r[2][0] * pivot.x + r[2][1] * pivot.y + r[2][2] * pivot.z,
    );
    let slide = t.add(turned.sub(pivot));
    // How far the body reaches from the pivot: its vertices and its faces'
    // control points (a body without vertices, a whole torus, still turns).
    let reach = solid
        .vertices
        .iter()
        .map(|v| v.point)
        .chain(solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| face.surface.control_points.iter().flatten()).filter_map(|p| p.point().ok()))
        .map(|p| p.sub(pivot).length())
        .fold(0.0f64, f64::max);
    let scale = crate::solid_model_scale(solid).max(1e-300);
    let read = rotation_turn(&r);
    let swing = read.map_or(0.0, |(_, angle)| angle * reach);
    let turn = read.filter(|_| swing > 1e-12 * scale).map(|(axis, angle)| (pivot, axis, angle));
    // Exactly, by components (a length would square a tiny slide to zero).
    let translates = slide.x != 0.0 || slide.y != 0.0 || slide.z != 0.0;
    // Only the motion that moves nothing (R exactly the identity, element by
    // element, and no slide) returns the body as it is; any other the turn's
    // read cannot resolve falls to the refusal below.
    let rest = (0..3).all(|i| (0..3).all(|j| r[i][j] == if i == j { 1.0 } else { 0.0 })) && t.x == 0.0 && t.y == 0.0 && t.z == 0.0;
    if rest {
        return Ok(solid.clone());
    }
    // A motion the re-solve's tolerance cannot tell from rest (it reads
    // points to max(1e-7 of the scale, 1e-9)) is refused by name, never
    // returned as the body unmoved.
    let resolution = (scale * 1e-7).max(1e-9);
    // The envelope: the re-solve reads a motion to 1e-9 of its length, so a
    // slide whose 1e-9 passes the model's resolution could have a resolvable
    // part read as rest; it is refused by name.
    if slide.length() * 1e-9 > resolution {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "motion_out_of_envelope",
            format!("transform_faces: the motion slides the faces {:e}, past what the re-solve reads to the model's resolution {resolution:e}", slide.length()),
        ));
    }
    if turn.is_none() && slide.length() + swing < resolution {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "motion_below_tolerance",
            format!("transform_faces: the motion moves the faces {:e}, below the {resolution:e} the re-solve reads points to", slide.length() + swing),
        ));
    }
    // A motion every road refused for carrying a face past its neighbour (an
    // edge between them turned over: the face pushed through the part) is the
    // input's fault, not a shape the kernel lacks: refused as ill-posed, its
    // message kept. A face landed exactly on a neighbour is answered above.
    crate::feature_pipeline::features::transform_face::move_faces_road(solid, &faces, turn, translates, slide)
        .map(|(body, _notes)| body)
        .map_err(|(refusal, _step)| past_a_neighbour_is_ill_posed(refusal))
}

/// A refusal that a face was carried past its neighbour (a straight or curved
/// edge turned over), in the ill-posed class; any other refusal as it is.
fn past_a_neighbour_is_ill_posed(mut refusal: KernelRefusal) -> KernelRefusal {
    use crate::face_refusal_slugs::{CURVED_EDGE_INVERSION, EDGE_INVERSION};
    if matches!(&refusal.class, crate::RefusalClass::UnsupportedGeometry { what } if what == EDGE_INVERSION || what == CURVED_EDGE_INVERSION) {
        refusal.class = crate::RefusalClass::IllPosed { what: "moved_past_a_neighbour".into() };
    }
    refusal
}

