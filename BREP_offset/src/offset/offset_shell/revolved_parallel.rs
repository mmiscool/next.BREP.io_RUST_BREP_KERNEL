//! `BREP_OFFSET_REVOLVED_PARALLEL` (default on; `=0` restores the refusal): the
//! rounded GROWTH of a solid of revolution, built from its half-section.
//!
//! A solid of revolution's distance to a point is read in that point's
//! meridian plane, so its parallel body `K ⊕ B_d` is the revolution of its full
//! meridian section grown by the disk of radius `d`. Where every edge of the
//! section is convex, that grown section is bounded by each boundary piece
//! moved out by `d` (a line to the parallel line, an arc to the concentric arc
//! of radius `R ± d`) and, at each corner, the arc of radius `d` about the
//! corner from the one piece's normal to the next's: the rolling ball's torus
//! about a circular rim. The lane reads the half-section off the body (every
//! edge a circle about one axis, every face a plane square to it, a cylinder or
//! cone about it, a sphere centred on it, or a torus about it), offsets it, and
//! revolves it.
//!
//! Cause: a hemisphere, a ball cut by a plane
//! and the lens two balls share, each grown by 1, refused ("completion produced
//! non-integral genus", "unwelded rim leaves a non-watertight shell"). The
//! sharp body the pipeline rounds needs each face's carrier to run past its
//! own rim to meet its neighbour's offset; a plane's net slides out under its
//! trims, but a sphere's trim stays on the source rim, so the moved plane and
//! the moved sphere never meet and their imprint is empty.
//!
//! It runs only where the pipeline's own growth refused, and declines (the
//! pipeline's refusal stands) on anything else: a concave corner, a face of
//! another kind or about another axis, an edge that is not a circle about the
//! axis, a piece whose offset would pass through the axis, or a revolved body
//! that does not validate.
use super::*;
use crate::{make_arc, make_line, parameter_point_in_face, project_point_to_surface, revolve_profile_brep};
use crate::{AnalyticSurface, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, PolygonClass, Vec2};

/// The switch, default on (`=0` restores the pipeline's refusal).
fn revolved_parallel_on() -> bool {
    switch!("BREP_OFFSET_REVOLVED_PARALLEL", true)
}


/// A point of the meridian half-plane: distance from the axis, height along it.
#[derive(Clone, Copy, Debug)]
struct P2 {
    rho: f64,
    z: f64,
}

impl P2 {
    fn sub(self, other: P2) -> P2 {
        P2 { rho: self.rho - other.rho, z: self.z - other.z }
    }
    fn add(self, other: P2) -> P2 {
        P2 { rho: self.rho + other.rho, z: self.z + other.z }
    }
    fn scale(self, k: f64) -> P2 {
        P2 { rho: self.rho * k, z: self.z * k }
    }
    fn length(self) -> f64 {
        self.rho.hypot(self.z)
    }
    fn cross(self, other: P2) -> f64 {
        self.rho * other.z - self.z * other.rho
    }
}

/// One piece of the half-section's boundary, from `start` to `end`.
#[derive(Clone, Copy, Debug)]
enum Piece {
    Line { start: P2, end: P2 },
    /// About `center`, anticlockwise (in (ρ, z)) when `ccw`.
    Arc { center: P2, radius: f64, start: P2, end: P2, ccw: bool },
}

impl Piece {
    fn start(&self) -> P2 {
        match *self {
            Piece::Line { start, .. } | Piece::Arc { start, .. } => start,
        }
    }
    fn end(&self) -> P2 {
        match *self {
            Piece::Line { end, .. } | Piece::Arc { end, .. } => end,
        }
    }
    fn reversed(&self) -> Piece {
        match *self {
            Piece::Line { start, end } => Piece::Line { start: end, end: start },
            Piece::Arc { center, radius, start, end, ccw } => Piece::Arc { center, radius, start: end, end: start, ccw: !ccw },
        }
    }
    /// Signed sweep (anticlockwise positive) of an arc from start to end.
    fn sweep(center: P2, start: P2, end: P2, ccw: bool) -> f64 {
        let a0 = (start.z - center.z).atan2(start.rho - center.rho);
        let a1 = (end.z - center.z).atan2(end.rho - center.rho);
        let mut sweep = a1 - a0;
        if ccw {
            while sweep <= 0.0 {
                sweep += std::f64::consts::TAU;
            }
        } else {
            while sweep >= 0.0 {
                sweep -= std::f64::consts::TAU;
            }
        }
        sweep
    }
    /// The point at `fraction` of the way along.
    fn at(&self, fraction: f64) -> P2 {
        match *self {
            Piece::Line { start, end } => start.add(end.sub(start).scale(fraction)),
            Piece::Arc { center, radius, start, end, ccw } => {
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let a = a0 + fraction * Piece::sweep(center, start, end, ccw);
                P2 { rho: center.rho + radius * a.cos(), z: center.z + radius * a.sin() }
            }
        }
    }
    /// The distance from `p` to this piece.
    fn distance(&self, p: P2) -> f64 {
        let ends = p.sub(self.start()).length().min(p.sub(self.end()).length());
        match *self {
            Piece::Line { start, end } => {
                let along = end.sub(start);
                let length2 = along.rho * along.rho + along.z * along.z;
                if !(length2 > 0.0) {
                    return ends;
                }
                let w = p.sub(start);
                let t = (w.rho * along.rho + w.z * along.z) / length2;
                if (0.0..=1.0).contains(&t) { p.sub(start.add(along.scale(t))).length() } else { ends }
            }
            Piece::Arc { center, radius, start, end, ccw } => {
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let sweep = Piece::sweep(center, start, end, ccw);
                let mut delta = (p.z - center.z).atan2(p.rho - center.rho) - a0;
                if ccw {
                    delta = delta.rem_euclid(std::f64::consts::TAU);
                } else {
                    delta = -(-delta).rem_euclid(std::f64::consts::TAU);
                }
                if delta.abs() <= sweep.abs() { (p.sub(center).length() - radius).abs() } else { ends }
            }
        }
    }
    /// The point halfway along.
    fn middle(&self) -> P2 {
        match *self {
            Piece::Line { start, end } => start.add(end).scale(0.5),
            Piece::Arc { center, radius, start, end, ccw } => {
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let a = a0 + 0.5 * Piece::sweep(center, start, end, ccw);
                P2 { rho: center.rho + radius * a.cos(), z: center.z + radius * a.sin() }
            }
        }
    }
    /// The unit tangent at the start (`at_end` false) or the end.
    fn tangent(&self, at_end: bool) -> Option<P2> {
        let t = match *self {
            Piece::Line { start, end } => end.sub(start),
            Piece::Arc { center, start, end, ccw, .. } => {
                let p = if at_end { end } else { start };
                let r = p.sub(center);
                if ccw { P2 { rho: -r.z, z: r.rho } } else { P2 { rho: r.z, z: -r.rho } }
            }
        };
        let length = t.length();
        (length > 0.0).then(|| t.scale(1.0 / length))
    }
    /// Twice the signed area this piece sweeps with the origin of (ρ, z)
    /// (exact for an arc: the chord's triangle plus the segment).
    fn area2(&self) -> f64 {
        let (start, end) = (self.start(), self.end());
        let chord = start.cross(end);
        match *self {
            Piece::Line { .. } => chord,
            Piece::Arc { center, radius, start, end, ccw } => {
                let sweep = Piece::sweep(center, start, end, ccw);
                chord + radius * radius * (sweep - sweep.sin())
            }
        }
    }
}

/// The lane's answer, or `None` when it declines or the switch is off.
pub(super) fn revolved_parallel(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !revolved_parallel_on() || !(distance > 0.0) {
        return None;
    }
    match revolved_parallel_impl(source, distance) {
        Ok(Some(solid)) => {
            let face_images = solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: 0 })
                .collect();
            Some(OffsetShellResultRecord { solid, face_images })
        }
        Ok(None) => None,
        Err(error) => {
            os_debug!("revolved parallel lane declined: {error}");
            None
        }
    }
}

/// The axis (a point and a unit direction) and the body's frame across it.
struct Axis {
    origin: Vec3,
    axis: Vec3,
    x: Vec3,
}

impl Axis {
    fn to_2d(&self, p: Vec3) -> P2 {
        let w = p.sub(self.origin);
        let z = w.dot(self.axis);
        P2 { rho: w.sub(self.axis.scale(z)).length(), z }
    }
    /// The point of the meridian half-plane at azimuth `phi` from `x`.
    fn to_3d(&self, p: P2, phi: f64) -> Vec3 {
        let y = self.axis.cross(self.x);
        let radial = self.x.scale(phi.cos()).add(y.scale(phi.sin()));
        self.origin.add(radial.scale(p.rho)).add(self.axis.scale(p.z))
    }
    fn on_axis(&self, p: Vec3, tolerance: f64) -> bool {
        let w = p.sub(self.origin);
        w.sub(self.axis.scale(w.dot(self.axis))).length() <= tolerance
    }
    fn parallel(&self, direction: Vec3) -> bool {
        direction.normalized().is_ok_and(|d| d.cross(self.axis).length() <= 1e-9)
    }
}

/// Whether the half-section point `p` (at an azimuth off any seam) lies on `face`.
fn on_face(face: &FaceRecord, axis: &Axis, p: P2, tolerance: f64) -> bool {
    let point = axis.to_3d(p, 1.0);
    let Ok(hit) = project_point_to_surface(&face.surface, point) else { return false };
    if hit.distance > tolerance {
        return false;
    }
    parameter_point_in_face(face, Vec2 { x: hit.u, y: hit.v }, 1e-9).is_ok_and(|class| class == PolygonClass::Inside)
}

fn revolved_parallel_impl(source: &BrepSolid, distance: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let [shell] = source.shells.as_slice() else { return Ok(None) };
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;

    // Each edge: its faces, by how many coedges of each face walk it.
    let mut uses: std::collections::HashMap<u64, Vec<usize>> = std::collections::HashMap::new();
    for (index, face) in shell.faces.iter().enumerate() {
        for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
            uses.entry(coedge.edge_id).or_default().push(index);
        }
    }
    // The rims: edges between two different faces. Each must be a circle,
    // all about one axis.
    let mut axis: Option<Axis> = None;
    let mut rims: Vec<(u64, P2, [usize; 2])> = Vec::new();
    for edge in &source.edges {
        let Some(faces) = uses.get(&edge.id) else { continue };
        if edge.degenerate {
            continue;
        }
        if faces.len() == 2 && faces[0] == faces[1] {
            continue; // a seam
        }
        let [a, b] = faces.as_slice() else { return Ok(None) };
        let Some(circle) = super::rim_welds::fit_rim_circle(edge, tolerance)? else { return Ok(None) };
        if axis.is_none() {
            let x = circle.axis.perpendicular().or_refuse(KernelStage::Classify, "perpendicular")?;
            axis = Some(Axis { origin: circle.center, axis: circle.axis, x });
        }
        let frame = axis.as_ref().expect("set above");
        if !frame.parallel(circle.axis) || !frame.on_axis(circle.center, tolerance) {
            return Ok(None);
        }
        rims.push((edge.id, frame.to_2d(edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Classify, "evaluate")?), [*a, *b]));
    }
    let Some(axis) = axis else { return Ok(None) };

    // Each face's piece of the half-section.
    let mut pieces: Vec<(usize, Piece, [Option<usize>; 2])> = Vec::new();
    for (index, face) in shell.faces.iter().enumerate() {
        let ends: Vec<usize> = rims.iter().enumerate().filter(|(_, rim)| rim.2.contains(&index)).map(|(k, _)| k).collect();
        let generatrix = match face.surface.analytic() {
            Some(AnalyticSurface::Plane { u_dir, v_dir, .. }) => {
                if !axis.parallel(u_dir.cross(*v_dir)) {
                    return Ok(None);
                }
                None
            }
            Some(AnalyticSurface::RuledRevolution { frame, .. }) => {
                if !axis.parallel(frame.axis) || !axis.on_axis(frame.origin, tolerance) {
                    return Ok(None);
                }
                None
            }
            Some(AnalyticSurface::Sphere { frame, radius }) => {
                if !axis.on_axis(frame.origin, tolerance) {
                    return Ok(None);
                }
                Some((axis.to_2d(frame.origin), *radius))
            }
            Some(AnalyticSurface::Torus { frame, major_radius, minor_radius }) => {
                if !axis.parallel(frame.axis) || !axis.on_axis(frame.origin, tolerance) {
                    return Ok(None);
                }
                let h = frame.origin.sub(axis.origin).dot(axis.axis);
                Some((P2 { rho: *major_radius, z: h }, *minor_radius))
            }
            _ => return Ok(None),
        };
        let piece = match (ends.as_slice(), generatrix) {
            // Between two rims: a segment, or the arc of the generatrix circle
            // through the face.
            ([a, b], None) => Piece::Line { start: rims[*a].1, end: rims[*b].1 },
            ([a, b], Some((center, radius))) => {
                let (start, end) = (rims[*a].1, rims[*b].1);
                let ccw = Piece::Arc { center, radius, start, end, ccw: true };
                let cw = Piece::Arc { center, radius, start, end, ccw: false };
                // The arc that stays in the half-plane and runs through the face.
                let holds = |piece: &Piece| piece.middle().rho > tolerance && on_face(face, &axis, piece.middle(), 1e-7 * scale);
                match (holds(&ccw), holds(&cw)) {
                    (true, false) => ccw,
                    (false, true) => cw,
                    _ => return Ok(None),
                }
            }
            // From one rim to the axis: a disk, a cone to its apex, a sphere to
            // the pole on the face.
            ([a], None) => {
                let start = rims[*a].1;
                let axis_end = match face.surface.analytic() {
                    Some(AnalyticSurface::Plane { origin, .. }) => P2 { rho: 0.0, z: axis.to_2d(*origin).z },
                    _ => return Ok(None),
                };
                Piece::Line { start, end: axis_end }
            }
            ([a], Some((center, radius))) => {
                if center.rho.abs() > tolerance {
                    return Ok(None);
                }
                let start = rims[*a].1;
                let mut found = None;
                for pole in [P2 { rho: 0.0, z: center.z + radius }, P2 { rho: 0.0, z: center.z - radius }] {
                    let ccw = Piece::sweep(center, start, pole, true).abs() <= std::f64::consts::PI + 1e-12;
                    let piece = Piece::Arc { center, radius, start, end: pole, ccw };
                    if on_face(face, &axis, piece.middle(), 1e-7 * scale) {
                        if found.is_some() {
                            return Ok(None);
                        }
                        found = Some(piece);
                    }
                }
                match found {
                    Some(piece) => piece,
                    None => return Ok(None),
                }
            }
            _ => return Ok(None),
        };
        let ends: Vec<Option<usize>> = ends.iter().map(|k| Some(*k)).collect();
        pieces.push((index, piece, [ends.first().copied().flatten(), ends.get(1).copied().flatten()]));
    }

    // Order the pieces into one chain from the axis back to the axis, or one
    // loop clear of it, joined at the rims.
    let chain = order_pieces(&pieces, &rims, tolerance)?;
    let Some(mut chain) = chain else { return Ok(None) };
    let touches_axis = chain.first().is_some_and(|p| p.start().rho.abs() <= tolerance);
    // The closing run along the axis (ρ = 0) sweeps no area about the origin
    // of (ρ, z), so the pieces alone give the orientation.
    let area2: f64 = chain.iter().map(Piece::area2).sum();
    if area2 < 0.0 {
        chain = chain.iter().rev().map(Piece::reversed).collect();
    }

    // Offset every piece outward; round every corner; decline a concave one.
    let outward = |piece: &Piece, at_end: bool| -> Option<P2> {
        piece.tangent(at_end).map(|t| P2 { rho: t.z, z: -t.rho })
    };
    let mut grown: Vec<Piece> = Vec::new();
    for (k, piece) in chain.iter().enumerate() {
        let moved = match *piece {
            Piece::Line { start, end } => {
                let Some(n) = outward(piece, false) else { return Ok(None) };
                Piece::Line { start: start.add(n.scale(distance)), end: end.add(n.scale(distance)) }
            }
            Piece::Arc { center, radius, start, end, ccw } => {
                let r = if ccw { radius + distance } else { radius - distance };
                if !(r > tolerance) {
                    return Ok(None);
                }
                let k = r / radius;
                Piece::Arc { center, radius: r, start: center.add(start.sub(center).scale(k)), end: center.add(end.sub(center).scale(k)), ccw }
            }
        };
        if k > 0 {
            let previous = &chain[k - 1];
            let corner = piece.start();
            let (Some(n1), Some(n2)) = (outward(previous, true), outward(piece, false)) else { return Ok(None) };
            let turn = n1.cross(n2);
            if turn < -1e-12 {
                return Ok(None); // concave
            }
            if turn > 1e-12 || n1.rho * n2.rho + n1.z * n2.z < 0.0 {
                grown.push(Piece::Arc {
                    center: corner,
                    radius: distance,
                    start: corner.add(n1.scale(distance)),
                    end: corner.add(n2.scale(distance)),
                    ccw: true,
                });
            }
        }
        grown.push(moved);
    }
    if touches_axis {
        let (first, last) = (grown[0].start(), grown[grown.len() - 1].end());
        if first.rho.abs() > tolerance || last.rho.abs() > tolerance {
            return Ok(None); // an end leaves the axis: an apex, not built here
        }
        grown.push(Piece::Line { start: last, end: first });
    } else {
        let (last, first) = (&chain[chain.len() - 1], &chain[0]);
        let corner = first.start();
        let (Some(n1), Some(n2)) = (outward(last, true), outward(first, false)) else { return Ok(None) };
        let turn = n1.cross(n2);
        if turn < -1e-12 {
            return Ok(None);
        }
        if turn > 1e-12 || n1.rho * n2.rho + n1.z * n2.z < 0.0 {
            grown.push(Piece::Arc {
                center: corner,
                radius: distance,
                start: corner.add(n1.scale(distance)),
                end: corner.add(n2.scale(distance)),
                ccw: true,
            });
        }
    }
    // A loop clear of the axis whose growth crosses it (a bore grown shut):
    // the grown half-section is the loop's own growth cut at ρ = 0, since a
    // point with ρ ≥ 0 is never nearer the section's mirror half than its own
    // ((ρ + a)² ≥ (ρ − a)² for a ≥ 0). The part kept must be one run, closed
    // up the axis.
    if !touches_axis && grown.iter().any(|piece| piece.start().rho < -tolerance || piece.middle().rho < -tolerance) {
        let split: Vec<Piece> = grown.iter().flat_map(|piece| split_at_axis(piece, tolerance)).collect();
        let kept: Vec<bool> = split.iter().map(|piece| piece.middle().rho >= -tolerance).collect();
        let count = split.len();
        let starts: Vec<usize> = (0..count).filter(|&k| kept[k] && !kept[(k + count - 1) % count]).collect();
        let [first] = starts.as_slice() else { return Ok(None) };
        let mut run: Vec<Piece> = Vec::new();
        for offset in 0..count {
            let k = (first + offset) % count;
            if !kept[k] {
                break;
            }
            run.push(split[k]);
        }
        let (head, tail) = (run[0].start(), run[run.len() - 1].end());
        if head.rho.abs() > 1e-7 * scale || tail.rho.abs() > 1e-7 * scale {
            return Ok(None);
        }
        let snap = |p: P2| P2 { rho: 0.0, z: p.z };
        let last = run.len() - 1;
        run[0] = with_start(&run[0], snap(head));
        run[last] = with_end(&run[last], snap(tail));
        run.push(Piece::Line { start: snap(tail), end: snap(head) });
        grown = run;
    }
    // Every grown piece stays in the half-plane.
    for piece in &grown {
        for p in [piece.start(), piece.end(), piece.middle()] {
            if p.rho < -tolerance {
                return Ok(None);
            }
        }
    }

    // The corners were read one at a time, so growths that meet across a gap
    // (the two rims of a narrow groove) are not seen there. Every point of the
    // true boundary lies exactly the distance from the section: a grown piece
    // with a point nearer than that to any source piece is not the boundary,
    // and the lane declines.
    for piece in &grown {
        if matches!(piece, Piece::Line { start, end } if start.rho.abs() <= tolerance && end.rho.abs() <= tolerance) {
            continue; // the closing run along the axis
        }
        for step in 1..16 {
            let p = piece.at(step as f64 / 16.0);
            let nearest = chain.iter().map(|source| source.distance(p)).fold(f64::INFINITY, f64::min);
            if nearest < distance - 1e-7 * scale {
                os_debug!("revolved parallel lane declined: a grown piece passes {nearest:.6} from the section, under {distance:.6}");
                return Ok(None);
            }
        }
    }

    // The grown half-section as curves in the plane of `x` and the axis, revolved.
    let mut curves: Vec<NurbsCurve> = Vec::with_capacity(grown.len());
    for piece in &grown {
        let curve = match *piece {
            Piece::Line { start, end } => {
                if start.sub(end).length() <= tolerance {
                    continue;
                }
                make_line(axis.to_3d(start, 0.0), axis.to_3d(end, 0.0)).or_refuse(KernelStage::Sew, "make_line")?
            }
            Piece::Arc { center, radius, start, end, ccw } => {
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let sweep = Piece::sweep(center, start, end, ccw);
                let c = axis.to_3d(center, 0.0);
                if ccw {
                    make_arc(c, axis.x, axis.axis, radius, a0, a0 + sweep).or_refuse(KernelStage::Sew, "make_arc")?
                } else {
                    make_arc(c, axis.x, axis.axis, radius, a0 + sweep, a0)
                        .and_then(|arc| arc.reversed())
                        .or_refuse(KernelStage::Sew, "make_arc")?
                }
            }
        };
        curves.push(curve);
    }
    let solid = revolve_profile_brep(&curves, axis.origin, axis.axis, std::f64::consts::TAU).or_refuse(KernelStage::Sew, "revolve")?;
    if !solid.validate().is_empty() {
        return Ok(None);
    }
    os_debug!("revolved parallel lane built {} faces from {} section pieces", solid.shells.iter().map(|s| s.faces.len()).sum::<usize>(), grown.len());
    Ok(Some(solid))
}

/// The pieces in boundary order: from an axis end round to the other, or
/// round a loop, consecutive pieces sharing a rim. `None` where they do not
/// form one such run.
fn order_pieces(
    pieces: &[(usize, Piece, [Option<usize>; 2])],
    rims: &[(u64, P2, [usize; 2])],
    tolerance: f64,
) -> Result<Option<Vec<Piece>>, KernelRefusal> {
    let _ = rims;
    if pieces.is_empty() {
        return Ok(None);
    }
    let close = |a: P2, b: P2| a.sub(b).length() <= 1e-7 * (1.0 + a.length()).max(tolerance);
    let mut used = vec![false; pieces.len()];
    // Start at a piece with an axis end, if any, oriented to leave the axis.
    let start_index = pieces.iter().position(|(_, piece, _)| piece.start().rho.abs() <= tolerance || piece.end().rho.abs() <= tolerance);
    let (first_index, first_piece) = match start_index {
        Some(index) => {
            let piece = pieces[index].1;
            (index, if piece.start().rho.abs() <= tolerance { piece } else { piece.reversed() })
        }
        None => (0, pieces[0].1),
    };
    used[first_index] = true;
    let mut chain = vec![first_piece];
    loop {
        let tail = chain[chain.len() - 1].end();
        if start_index.is_some() && tail.rho.abs() <= tolerance {
            break;
        }
        let next = pieces.iter().enumerate().find_map(|(index, (_, piece, _))| {
            if used[index] {
                return None;
            }
            if close(piece.start(), tail) {
                Some((index, *piece))
            } else if close(piece.end(), tail) {
                Some((index, piece.reversed()))
            } else {
                None
            }
        });
        match next {
            Some((index, piece)) => {
                used[index] = true;
                chain.push(piece);
            }
            None => break,
        }
    }
    if used.iter().any(|u| !u) {
        return Ok(None);
    }
    if start_index.is_none() && !close(chain[chain.len() - 1].end(), chain[0].start()) {
        return Ok(None);
    }
    Ok(Some(chain))
}

/// `piece` with its start moved to `start` (a point already on it).
fn with_start(piece: &Piece, start: P2) -> Piece {
    match *piece {
        Piece::Line { end, .. } => Piece::Line { start, end },
        Piece::Arc { center, radius, end, ccw, .. } => Piece::Arc { center, radius, start, end, ccw },
    }
}

/// `piece` with its end moved to `end` (a point already on it).
fn with_end(piece: &Piece, end: P2) -> Piece {
    match *piece {
        Piece::Line { start, .. } => Piece::Line { start, end },
        Piece::Arc { center, radius, start, ccw, .. } => Piece::Arc { center, radius, start, end, ccw },
    }
}

/// `piece` split where it crosses the axis ρ = 0, in order along it.
fn split_at_axis(piece: &Piece, tolerance: f64) -> Vec<Piece> {
    match *piece {
        Piece::Line { start, end } => {
            if (start.rho < -tolerance && end.rho > tolerance) || (start.rho > tolerance && end.rho < -tolerance) {
                let t = start.rho / (start.rho - end.rho);
                let cut = start.add(end.sub(start).scale(t));
                vec![Piece::Line { start, end: cut }, Piece::Line { start: cut, end }]
            } else {
                vec![*piece]
            }
        }
        Piece::Arc { center, radius, start, end, ccw } => {
            let cosine = -center.rho / radius;
            if !(cosine.abs() < 1.0) {
                return vec![*piece];
            }
            let a0 = (start.z - center.z).atan2(start.rho - center.rho);
            let sweep = Piece::sweep(center, start, end, ccw);
            let mut fractions: Vec<f64> = Vec::new();
            for crossing in [cosine.acos(), -cosine.acos()] {
                let mut delta = crossing - a0;
                if ccw {
                    delta = delta.rem_euclid(std::f64::consts::TAU);
                } else {
                    delta = -(-delta).rem_euclid(std::f64::consts::TAU);
                }
                let fraction = delta / sweep;
                if fraction > 1e-12 && fraction < 1.0 - 1e-12 {
                    fractions.push(fraction);
                }
            }
            fractions.sort_by(f64::total_cmp);
            let at = |fraction: f64| {
                let a = a0 + sweep * fraction;
                P2 { rho: center.rho + radius * a.cos(), z: center.z + radius * a.sin() }
            };
            let mut pieces = Vec::new();
            let mut from = start;
            for fraction in fractions {
                let cut = at(fraction);
                pieces.push(Piece::Arc { center, radius, start: from, end: cut, ccw });
                from = cut;
            }
            pieces.push(Piece::Arc { center, radius, start: from, end, ccw });
            pieces
        }
    }
}
