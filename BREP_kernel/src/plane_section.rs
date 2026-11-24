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

use crate::{face_area, shell_vector_areas, solid_piece_by_plane, BrepSolid, FaceRecord, KernelRefusal, KernelStage, NurbsCurve, Vec3};

/// `BREP_SECTION_SUB_BAND_AREA` (default on, `=0` restores the refusal): a
/// cut whose regions are thinner than the boolean's weld band is measured by
/// [`sub_band_area`] instead of refused.
fn sub_band_area_enabled() -> bool {
    crate::switch!("BREP_SECTION_SUB_BAND_AREA", true)
}

/// What [`caps`] found.
enum Cut {
    /// The plane does not cut the solid.
    Miss,
    /// The piece on one side, and the indices of its faces on the plane.
    Caps(BrepSolid, Vec<usize>),
    /// The solid lies on both sides, but every region of the cut is thinner
    /// than the boolean's weld band, so neither piece carries a face on the
    /// plane; the cut's area, read by [`sub_band_area`].
    SubBand(f64),
}

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
fn caps(solid: &BrepSolid, origin: Vec3, normal: Vec3) -> Result<Cut, KernelRefusal> {
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
        return Ok(Cut::Miss);
    }
    let mut declined = Vec::new();
    let mut found = None;
    'covers: for cover in COVERS {
        for above in [false, true] {
            match solid_piece_by_plane(solid, origin, unit, above, cover) {
                Ok(piece) => {
                    found = Some((piece, above));
                    break 'covers;
                }
                Err(message) => declined.push(message),
            }
        }
    }
    let Some((piece, piece_above)) = found else {
        return Err(refuse(declined.join("; ")));
    };
    // The control points straddle the plane, but the solid itself may not.
    let Some(piece) = piece else {
        return Ok(Cut::Miss);
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
        let other = !piece_above;
        let mut other_declined = Vec::new();
        for cover in COVERS {
            match solid_piece_by_plane(solid, origin, unit, other, cover) {
                Ok(None) => return Ok(Cut::Miss),
                Ok(Some(other_piece)) => {
                    if lumps_part(solid, &piece, &other_piece, origin, unit, other, tolerance) {
                        return Ok(Cut::Miss);
                    }
                    if sub_band_area_enabled() {
                        if let Some(area) = sub_band_area(solid, &piece, &other_piece, unit, piece_above) {
                            return Ok(Cut::SubBand(area));
                        }
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
    Ok(Cut::Caps(piece, faces))
}

/// The area of a cut that left no face on the plane although the solid lies
/// on both sides of it: every region of the cut is thinner than the weld band
/// (a ring a few micrometres thick around a rod), so the boolean sewed the
/// faces either side straight across the plane. Each piece is then a closed
/// shell short of its caps, and `∫ n dA = 0` over the whole boundary makes
/// the caps' area the piece's own trim residual ([`shell_vector_areas`]):
/// `−A n` for the piece below, `+A n` for the one above. It is read only when
/// it is a measurement: every shell of both pieces closed and read to the
/// end, each residual along the normal with its side's sign, the two areas
/// agreeing, and the area clear of the noise (the solid's own residual, the
/// residuals' components off the normal, the two pieces' disagreement, and
/// every quadrature error) by a factor of ten. `None` otherwise, and the cut
/// is refused as before.
fn sub_band_area(solid: &BrepSolid, piece: &BrepSolid, other: &BrepSolid, unit: Vec3, piece_above: bool) -> Option<f64> {
    // The summed residual of every shell, its quadrature error, or `None`
    // when a shell is open, unread or abandoned.
    let residual = |body: &BrepSolid| -> Option<(Vec3, f64)> {
        let report = shell_vector_areas(body);
        if !report.unreadable.is_empty() || report.shells.is_empty() {
            return None;
        }
        let mut sum = Vec3::new(0.0, 0.0, 0.0);
        let mut error = 0.0;
        for shell in &report.shells {
            if !shell.closed || shell.gave_up || !shell.trim_residual.length().is_finite() {
                return None;
            }
            sum = sum.add(shell.trim_residual);
            error += shell.quadrature_error;
        }
        Some((sum, error))
    };
    let (whole, whole_error) = residual(solid)?;
    let (first, first_error) = residual(piece)?;
    let (second, second_error) = residual(other)?;
    // The piece above lacks caps facing −n, so its faces sum to +A n.
    let (above, below) = if piece_above { (first, second) } else { (second, first) };
    let (area_above, area_below) = (above.dot(unit), -below.dot(unit));
    if !(area_above > 0.0 && area_below > 0.0) {
        return None;
    }
    let off = |vector: Vec3, along: f64| vector.sub(unit.scale(along)).length();
    let noise = whole.length()
        + off(above, area_above)
        + off(below, -area_below)
        + (area_above - area_below).abs()
        + 10.0 * (whole_error + first_error + second_error);
    let area = 0.5 * (area_above + area_below);
    (area > 10.0 * noise).then_some(area)
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
    let (below, faces) = match caps(solid, origin, normal)? {
        Cut::Miss => return Ok(Vec::new()),
        Cut::Caps(below, faces) => (below, faces),
        Cut::SubBand(area) => {
            return Err(KernelRefusal::internal(
                KernelStage::Fragment,
                "section_split",
                format!(
                    "section: every region of the cut is thinner than the boolean's weld band (area {area:.3e}), \
                     so no face on the plane holds its curves"
                ),
            ))
        }
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
    let (below, faces) = match caps(solid, origin, normal)? {
        Cut::Miss => return Ok(0.0),
        Cut::Caps(below, faces) => (below, faces),
        Cut::SubBand(area) => return Ok(area),
    };
    let all: Vec<&FaceRecord> = below.shells.iter().flat_map(|shell| &shell.faces).collect();
    let mut area = 0.0;
    for index in faces {
        area += face_area(all[index])
            .map_err(|message| KernelRefusal::internal(KernelStage::Collect, "section_area", format!("section area: {message}")))?;
    }
    Ok(area)
}

