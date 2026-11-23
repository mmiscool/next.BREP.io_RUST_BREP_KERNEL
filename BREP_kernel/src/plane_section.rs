//! The cut of a solid by a plane: where the plane meets the boundary, as
//! polylines, and the area of the cut.
//!
//! The solid is cut by the plane ([`solid_piece_by_plane`]); the piece below
//! it (or above it, when that cut declines) carries one cap face on the plane for every region of the cut, bounded by
//! the section curves themselves. The area is those faces' areas, and the
//! curves are their edges, each sampled from end to end with every chord
//! within the caller's tolerance of the edge curve. Every sample is a point of
//! an edge curve, so it lies on the plane and on the boundary to the accuracy
//! the boolean built those edges to.

use crate::{face_area, solid_piece_by_plane, BrepSolid, FaceRecord, KernelRefusal, KernelStage, NurbsCurve, Vec3};

/// The covers [`caps`] cuts with, tightest first: a box just over the solid
/// keeps the boolean's tolerances to the solid's own size.
const COVERS: [f64; 4] = [1.2, 1.5, 2.0, 3.0];

/// The faces of a piece of `solid` cut off by the plane through `origin`
/// square to `normal` that lie in that plane (the caps), with the piece that
/// holds them; `None` when the plane does not cut the solid.
///
/// The plane misses when every control point of the solid lies strictly on
/// one side (each face and edge lies in the hull of its control points), and
/// no boolean is run. Otherwise the piece below the plane is cut, then the
/// piece above (both carry the same caps), with boxes from [`COVERS`] in turn
/// until one cut goes through. A piece with no face on the plane is a miss when
/// the other side is empty, or when the plane passes between the solid's lumps
/// ([`lumps_part`]); a cut that declines at every box, or a solid on both
/// sides with no face on the plane otherwise, is refused, never read as a miss.
fn caps(solid: &BrepSolid, origin: Vec3, normal: Vec3) -> Result<Option<(BrepSolid, Vec<usize>)>, KernelRefusal> {
    let unit = normal
        .normalized()
        .map_err(|_| KernelRefusal::input(KernelStage::Collect, "section_normal", "section: the plane normal has no direction"))?;
    let refuse = |message: String| KernelRefusal::internal(KernelStage::Fragment, "section_split", format!("section: {message}"));
    let surfaces = solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| face.surface.control_points.iter().flatten());
    let curves = solid.edges.iter().flat_map(|edge| edge.curve.control_points.iter());
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for control in surfaces.chain(curves) {
        let height = control.point().map_err(|message| refuse(message))?.sub(origin).dot(unit);
        low = low.min(height);
        high = high.max(height);
    }
    if !(low < 0.0 && high > 0.0) {
        return Ok(None);
    }
    let mut declined = Vec::new();
    let mut found = None;
    'covers: for cover in COVERS {
        for above in [false, true] {
            match solid_piece_by_plane(solid, origin, unit, above, cover) {
                Ok(piece) => {
                    found = Some(piece);
                    break 'covers;
                }
                Err(message) => declined.push(message),
            }
        }
    }
    let Some(piece) = found else {
        return Err(refuse(declined.join("; ")));
    };
    // The control points straddle the plane, but the solid itself may not.
    let Some(piece) = piece else {
        return Ok(None);
    };
    let extent = piece.vertices.iter().fold(0.0f64, |extent, vertex| {
        extent.max(vertex.point.x.abs()).max(vertex.point.y.abs()).max(vertex.point.z.abs())
    });
    let scale = 1.0 + extent;
    let tolerance = 1e-9 * scale;
    let on_plane = |face: &FaceRecord| -> bool {
        if !face.surface.is_affine().unwrap_or(false) {
            return false;
        }
        face.surface.control_points.iter().flatten().all(|control| {
            control.point().is_ok_and(|point| point.sub(origin).dot(unit).abs() <= tolerance)
        })
    };
    let faces: Vec<usize> = piece.shells.iter().flat_map(|shell| &shell.faces).enumerate().filter(|(_, face)| on_plane(face)).map(|(index, _)| index).collect();
    if faces.is_empty() {
        // No cap: the plane misses the solid when nothing of it lies on the
        // other side (the piece is then the whole solid), or when the solid's
        // lumps fall wholly to one side or the other: the two pieces then
        // share out the solid's faces without cutting any, their volumes sum
        // to the solid's, and every vertex of each lies clear of the plane on
        // its own side.
        let other = declined.len() % 2 == 0;
        let mut other_declined = Vec::new();
        for cover in COVERS {
            match solid_piece_by_plane(solid, origin, unit, other, cover) {
                Ok(None) => return Ok(None),
                Ok(Some(other_piece)) => {
                    if lumps_part(solid, &piece, &other_piece, origin, unit, other, tolerance) {
                        return Ok(None);
                    }
                    return Err(refuse("the solid lies on both sides of the plane but the cut left no face on it".into()));
                }
                Err(message) => other_declined.push(message),
            }
        }
        return Err(refuse(format!(
            "the cut left no face on the plane, and the other side declined: {}",
            other_declined.join("; ")
        )));
    }
    Ok(Some((piece, faces)))
}

/// Whether `first` and `second`, the pieces of `solid` on the two sides of the
/// plane (`second` above it when `second_above`), are its lumps parted whole:
/// together they hold exactly the solid's faces, their volumes sum to the
/// solid's, and every vertex of each lies more than `tolerance` from the plane
/// on its own side. A cut that split a face adds faces, so a solid the plane
/// truly crosses never passes.
fn lumps_part(solid: &BrepSolid, first: &BrepSolid, second: &BrepSolid, origin: Vec3, unit: Vec3, second_above: bool, tolerance: f64) -> bool {
    let faces = |body: &BrepSolid| body.shells.iter().map(|shell| shell.faces.len()).sum::<usize>();
    if faces(first) + faces(second) != faces(solid) {
        return false;
    }
    let side = |body: &BrepSolid, above: bool| {
        body.vertices.iter().all(|vertex| {
            let height = vertex.point.sub(origin).dot(unit);
            if above { height > tolerance } else { height < -tolerance }
        })
    };
    if !(side(first, !second_above) && side(second, second_above)) {
        return false;
    }
    match (crate::solid_mass_properties(solid), crate::solid_mass_properties(first), crate::solid_mass_properties(second)) {
        // A lost or turned lump moves the sum by a whole lump; the mass
        // integrals of swept faces carry a few parts in 1e9.
        (Ok(whole), Ok(a), Ok(b)) => (a.volume + b.volume - whole.volume).abs() <= 1e-6 * whole.volume.abs().max(f64::MIN_POSITIVE),
        _ => false,
    }
}

/// `curve` over `[t0, t1]` as points from end to end, every chord within
/// `tolerance` of the curve at its quarter points: each knot span is halved
/// until it is.
fn sample_edge(curve: &NurbsCurve, t0: f64, t1: f64, tolerance: f64) -> Result<Vec<Vec3>, String> {
    let (low, high) = (t0.min(t1), t0.max(t1));
    let mut breaks: Vec<f64> = vec![low];
    breaks.extend(curve.knots.iter().copied().filter(|&knot| knot > low && knot < high));
    breaks.push(high);
    breaks.dedup();
    let chord_error = |a: f64, b: f64| -> Result<f64, String> {
        let (p, q) = (curve.evaluate(a)?, curve.evaluate(b)?);
        let chord = q.sub(p);
        let length = chord.length();
        let mut worst = 0.0f64;
        for fraction in [0.25, 0.5, 0.75] {
            let point = curve.evaluate(a + (b - a) * fraction)?;
            let offset = point.sub(p);
            let along = if length > 0.0 { offset.dot(chord) / (length * length) } else { 0.0 };
            worst = worst.max(offset.sub(chord.scale(along.clamp(0.0, 1.0))).length());
        }
        Ok(worst)
    };
    let mut parameters = vec![low];
    for pair in breaks.windows(2) {
        // Depth-first halving of the span, at most 24 levels deep.
        let mut start = pair[0];
        let mut pending = vec![(pair[1], 0u32)];
        while let Some((end, depth)) = pending.pop() {
            if depth < 24 && chord_error(start, end)? > tolerance {
                let middle = 0.5 * (start + end);
                pending.push((end, depth + 1));
                pending.push((middle, depth + 1));
                continue;
            }
            parameters.push(end);
            start = end;
        }
    }
    let mut points = parameters.iter().map(|&t| curve.evaluate(t)).collect::<Result<Vec<_>, _>>()?;
    if t1 < t0 {
        points.reverse();
    }
    Ok(points)
}

/// Where the plane through `origin` square to `normal` cuts the boundary of
/// `solid`, in pieces in any order: each piece is one edge of the cut, as
/// points along it from end to end, every chord within `chord_tolerance` of
/// the edge curve. The pieces join end to end into the closed curves of the
/// cut; a piece that is a whole closed curve ends on its first point. Empty
/// when the plane misses the solid.
pub fn solid_section(
    solid: &BrepSolid,
    origin: Vec3,
    normal: Vec3,
    chord_tolerance: f64,
) -> Result<Vec<Vec<Vec3>>, KernelRefusal> {
    if !(chord_tolerance > 0.0 && chord_tolerance.is_finite()) {
        return Err(KernelRefusal::input(KernelStage::Collect, "section_tolerance", "section: the chord tolerance must be positive"));
    }
    let Some((below, faces)) = caps(solid, origin, normal)? else {
        return Ok(Vec::new());
    };
    let all: Vec<&FaceRecord> = below.shells.iter().flat_map(|shell| &shell.faces).collect();
    let mut pieces = Vec::new();
    for index in faces {
        for coedge in all[index].loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let edge = below.edges.iter().find(|edge| edge.id == coedge.edge_id).ok_or_else(|| {
                KernelRefusal::internal(KernelStage::Collect, "section_edge", format!("section: edge {} is missing", coedge.edge_id))
            })?;
            if edge.degenerate {
                continue;
            }
            let points = sample_edge(&edge.curve, edge.t0, edge.t1, chord_tolerance)
                .map_err(|message| KernelRefusal::internal(KernelStage::Collect, "section_sample", format!("section: {message}")))?;
            pieces.push(points);
        }
    }
    Ok(pieces)
}

/// The area of the cut of `solid` by the plane through `origin` square to
/// `normal`: the area of the faces the split leaves on that plane. Zero when
/// the plane misses the solid.
pub fn solid_section_area(solid: &BrepSolid, origin: Vec3, normal: Vec3) -> Result<f64, KernelRefusal> {
    let Some((below, faces)) = caps(solid, origin, normal)? else {
        return Ok(0.0);
    };
    let all: Vec<&FaceRecord> = below.shells.iter().flat_map(|shell| &shell.faces).collect();
    let mut area = 0.0;
    for index in faces {
        area += face_area(all[index])
            .map_err(|message| KernelRefusal::internal(KernelStage::Collect, "section_area", format!("section area: {message}")))?;
    }
    Ok(area)
}

