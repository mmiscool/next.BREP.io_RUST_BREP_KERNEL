//! The SHRUNK rounded parallel body of a right prism over a polygon
//! (`offset_solid` by −d with `OffsetCorners::Rounded`): the polygon's erosion
//! by d, extruded between the caps moved in by d.
//!
//! Exact by a product argument. A ball of radius d about (x, z) lies inside
//! profile × [0, h] exactly when its disk about x lies in the profile and its
//! span [z − d, z + d] lies in [0, h]: the ball's projections are that disk and
//! that span, and every point of the ball projects into both. So the prism's
//! erosion is (the profile's erosion) × [d, h − d], and only a planar erosion
//! is computed. That erosion is bounded by each edge's inward offset and by an
//! arc of radius d about each REFLEX vertex, from one edge's normal to the
//! next; every such piece lies exactly d from its own edge or vertex, and it
//! bounds the erosion where it lies at least d from the whole outline. The
//! pieces are split where they cross, each kept by its midpoint's distance to
//! the outline, and chained into loops with the erosion on their left. The
//! erosion of a polygon without holes has no holes: every point within d of
//! the outline reaches it along its shortest segment without leaving that
//! band.
//!
//! Cause (brep-bench offset/dumbbell-split and shell/dumbbell-neck-filled):
//! two 10 × 10 squares joined by a neck 2 wide, extruded 6 and shrunk by 1.5.
//! The neck vanishes and two bodies are left, each a 7 × 7 square bulging
//! towards the old neck in arcs about its corners. The pipeline refuses the
//! walls that offset across each other, and the by-pieces lane (the
//! complement in a frame box, cut, grown piece by piece, united and
//! subtracted) took 1.5 s.
//!
//! Scope: one shell; two planar caps square to the prism's direction, each one
//! outer loop of straight edges; every other face a planar rectangle across
//! the full height. Holes, curved or slanted sides decline. A knife edge
//! declines too: two offsets that run on top of each other (a clearance of
//! exactly 2d), a tangency inside two pieces, a piece too short to read.
//!
//! The lane runs before the pipeline only where the erosion changes the
//! outline's topology (an edge's offset vanishes, or more than one body is
//! left), so a prism the pipeline already builds keeps the pipeline's faces;
//! after a pipeline refusal it is offered again for any prism.
use super::*;
use crate::{extrude_profile_brep, make_arc, make_line};
use std::f64::consts::TAU;

/// `BREP_OFFSET_PRISM_EROSION` (default on; `=0` restores): [`prism_erosion`].
pub(super) fn prism_erosion_on() -> bool {
    switch!("BREP_OFFSET_PRISM_EROSION", true)
}


/// The shrunk rounded parallel body of `source` by `distance` (< 0), or `None`
/// when the lane does not apply. With `only_new_topology` it answers only
/// where the erosion drops an edge or leaves more than one body.
pub(super) fn prism_erosion(source: &BrepSolid, distance: f64, only_new_topology: bool) -> Option<OffsetShellResultRecord> {
    if !prism_erosion_on() || !(distance < 0.0) || !distance.is_finite() {
        return None;
    }
    match prism_erosion_impl(source, -distance, only_new_topology) {
        Ok(result) => result,
        Err(reason) => {
            os_debug!("prism erosion declined: {reason}");
            None
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct P2 {
    pub x: f64,
    pub y: f64,
}

impl P2 {
    pub(super) fn sub(self, o: P2) -> P2 {
        P2 { x: self.x - o.x, y: self.y - o.y }
    }
    pub(super) fn add(self, o: P2) -> P2 {
        P2 { x: self.x + o.x, y: self.y + o.y }
    }
    pub(super) fn scale(self, s: f64) -> P2 {
        P2 { x: self.x * s, y: self.y * s }
    }
    pub(super) fn dot(self, o: P2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    pub(super) fn cross(self, o: P2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    pub(super) fn len(self) -> f64 {
        self.x.hypot(self.y)
    }
}

/// A boundary piece of the erosion: a segment, or an arc about `center` of the
/// erosion radius from angle `start` by `sweep` (negative: clockwise).
#[derive(Clone, Copy, Debug)]
pub(super) enum Piece {
    Line { a: P2, b: P2, edge: usize },
    Arc { center: P2, start: f64, sweep: f64 },
}

impl Piece {
    pub(super) fn at(&self, radius: f64, t: f64) -> P2 {
        match *self {
            Piece::Line { a, b, .. } => a.add(b.sub(a).scale(t)),
            Piece::Arc { center, start, sweep } => {
                let angle = start + sweep * t;
                center.add(P2 { x: radius * angle.cos(), y: radius * angle.sin() })
            }
        }
    }
    fn length(&self, radius: f64) -> f64 {
        match *self {
            Piece::Line { a, b, .. } => b.sub(a).len(),
            Piece::Arc { sweep, .. } => radius * sweep.abs(),
        }
    }
    /// The parameter of `point` (on the piece's line or circle) along the piece.
    fn parameter(&self, point: P2, radius: f64, tolerance: f64) -> f64 {
        match *self {
            Piece::Line { a, b, .. } => {
                let run = b.sub(a);
                point.sub(a).dot(run) / run.dot(run)
            }
            Piece::Arc { center, start, sweep } => {
                let angle = (point.y - center.y).atan2(point.x - center.x);
                let along = if sweep > 0.0 { (angle - start).rem_euclid(TAU) } else { (start - angle).rem_euclid(TAU) };
                // A point a hair before the start wraps to a full turn.
                let along = if (TAU - along) * radius <= tolerance { 0.0 } else { along };
                along / sweep.abs()
            }
        }
    }
    /// The piece between parameters `t0` and `t1`.
    fn between(&self, radius: f64, t0: f64, t1: f64) -> Piece {
        match *self {
            Piece::Line { edge, .. } => Piece::Line { a: self.at(radius, t0), b: self.at(radius, t1), edge },
            Piece::Arc { center, start, sweep } => Piece::Arc { center, start: start + sweep * t0, sweep: sweep * (t1 - t0) },
        }
    }
}

/// Where two pieces' carriers cross, as point lists; `Err` at a knife edge (a
/// tangency or an overlap inside both pieces).
fn crossings(p: &Piece, q: &Piece, radius: f64, tolerance: f64) -> Result<Vec<P2>, String> {
    let inside = |piece: &Piece, point: P2| {
        let t = piece.parameter(point, radius, tolerance);
        let slack = tolerance / piece.length(radius).max(tolerance);
        (-slack..=1.0 + slack).contains(&t)
    };
    let strictly_inside = |piece: &Piece, point: P2| {
        let t = piece.parameter(point, radius, tolerance);
        let slack = 10.0 * tolerance / piece.length(radius).max(tolerance);
        t > slack && t < 1.0 - slack
    };
    let line_circle = |a: P2, b: P2, center: P2| -> Result<Vec<P2>, ()> {
        let run = b.sub(a);
        let unit = run.scale(1.0 / run.len());
        let foot = a.add(unit.scale(center.sub(a).dot(unit)));
        let off = foot.sub(center).len();
        if (off - radius).abs() <= tolerance {
            return Err(());
        }
        if off > radius {
            return Ok(Vec::new());
        }
        let half = (radius * radius - off * off).sqrt();
        Ok(vec![foot.sub(unit.scale(half)), foot.add(unit.scale(half))])
    };
    let points = match (*p, *q) {
        (Piece::Line { a, b, .. }, Piece::Line { a: c, b: d, .. }) => {
            let u = b.sub(a);
            let w = d.sub(c);
            let den = u.cross(w);
            if den.abs() <= 1e-12 * u.len() * w.len() {
                // Parallel: on one line and overlapping is a knife edge.
                let gap = c.sub(a).cross(u).abs() / u.len();
                if gap <= tolerance {
                    let (t0, t1) = (c.sub(a).dot(u) / u.dot(u), d.sub(a).dot(u) / u.dot(u));
                    let (lo, hi) = (t0.min(t1), t0.max(t1));
                    let slack = tolerance / u.len();
                    if hi > slack && lo < 1.0 - slack && hi - lo > slack {
                        return Err("two offsets run on top of each other".into());
                    }
                }
                Vec::new()
            } else {
                vec![a.add(u.scale(c.sub(a).cross(w) / den))]
            }
        }
        (Piece::Line { a, b, .. }, Piece::Arc { center, .. }) | (Piece::Arc { center, .. }, Piece::Line { a, b, .. }) => {
            match line_circle(a, b, center) {
                Ok(points) => points,
                Err(()) => {
                    let run = b.sub(a);
                    let unit = run.scale(1.0 / run.len());
                    let touch = a.add(unit.scale(center.sub(a).dot(unit)));
                    if strictly_inside(p, touch) && strictly_inside(q, touch) {
                        return Err("an offset is tangent to an arc inside both".into());
                    }
                    vec![touch]
                }
            }
        }
        (Piece::Arc { center: c1, .. }, Piece::Arc { center: c2, .. }) => {
            let gap = c2.sub(c1).len();
            if gap <= tolerance {
                return Err("two arcs about one point".into());
            }
            if (gap - 2.0 * radius).abs() <= tolerance {
                let touch = c1.add(c2.sub(c1).scale(0.5));
                if strictly_inside(p, touch) && strictly_inside(q, touch) {
                    return Err("two arcs touch inside both".into());
                }
                vec![touch]
            } else if gap > 2.0 * radius {
                Vec::new()
            } else {
                let middle = c1.add(c2.sub(c1).scale(0.5));
                let half = (radius * radius - 0.25 * gap * gap).sqrt();
                let across = P2 { x: -(c2.y - c1.y) / gap, y: (c2.x - c1.x) / gap };
                vec![middle.add(across.scale(half)), middle.sub(across.scale(half))]
            }
        }
    };
    Ok(points.into_iter().filter(|&point| inside(p, point) && inside(q, point)).collect())
}

/// The distance from `point` to the real edges of the closed polygon `outline`.
fn outline_distance(outline: &[P2], real: &[bool], point: P2) -> f64 {
    let mut best = f64::INFINITY;
    for k in (0..outline.len()).filter(|&k| real[k]) {
        let (a, b) = (outline[k], outline[(k + 1) % outline.len()]);
        let run = b.sub(a);
        let t = (point.sub(a).dot(run) / run.dot(run)).clamp(0.0, 1.0);
        best = best.min(point.sub(a.add(run.scale(t))).len());
    }
    best
}

struct RightPrism<'a> {
    /// The bottom cap's outline, counter-clockwise about `axis` in (`ex`, `ey`).
    outline: Vec<P2>,
    origin: Vec3,
    ex: Vec3,
    ey: Vec3,
    axis: Vec3,
    height: f64,
    faces: &'a [FaceRecord],
}

fn outward(face: &FaceRecord) -> Result<(Vec3, Vec3), String> {
    let [a0, a1] = face.surface.domain_u()?;
    let [b0, b1] = face.surface.domain_v()?;
    let (u, v) = (0.5 * (a0 + a1), 0.5 * (b0 + b1));
    let normal = face.surface.normal(u, v)?;
    let normal = if face.same_sense { normal } else { normal.scale(-1.0) };
    Ok((face.surface.evaluate(u, v)?, normal))
}

/// The ordered corner points of a face's only loop.
fn loop_corners(source: &BrepSolid, face: &FaceRecord, tolerance: f64) -> Result<Vec<Vec3>, String> {
    let [outer] = face.loops.as_slice() else {
        return Err("a cap or side has more than one loop".into());
    };
    let mut corners = Vec::with_capacity(outer.coedges.len());
    for coedge in &outer.coedges {
        let edge = source.edges.iter().find(|edge| edge.id == coedge.edge_id).ok_or("a coedge names no edge")?;
        let vertex = |id: u64| source.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point);
        let (start, end) = if coedge.forward {
            (vertex(edge.start_vertex_id), vertex(edge.end_vertex_id))
        } else {
            (vertex(edge.end_vertex_id), vertex(edge.start_vertex_id))
        };
        let (Some(start), Some(end)) = (start, end) else {
            return Err("an edge names no vertex".into());
        };
        let middle = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1))?;
        let run = end.sub(start);
        let length = run.length();
        if length <= tolerance || middle.sub(start).cross(run).length() / length > tolerance {
            return Err("an edge is not a straight segment".into());
        }
        corners.push(start);
    }
    Ok(corners)
}

fn recognize(source: &BrepSolid, tolerance: f64) -> Result<RightPrism<'_>, String> {
    let [shell] = source.shells.as_slice() else {
        return Err("not one shell".into());
    };
    let faces = shell.faces.as_slice();
    if faces.len() < 5 {
        return Err("too few faces for a prism".into());
    }
    let mut frames = Vec::with_capacity(faces.len());
    for face in faces {
        if !face.surface.is_affine()? {
            return Err("a face is not planar".into());
        }
        frames.push(outward(face)?);
    }
    // The caps: the one antiparallel pair every other face is square to.
    let mut caps = None;
    'pairs: for i in 0..faces.len() {
        for j in i + 1..faces.len() {
            if frames[i].1.dot(frames[j].1) > -1.0 + 1e-12 {
                continue;
            }
            let axis = frames[i].1;
            if (0..faces.len()).filter(|&k| k != i && k != j).all(|k| frames[k].1.dot(axis).abs() <= 1e-12) {
                caps = Some((i, j));
                break 'pairs;
            }
        }
    }
    let (bottom, top) = caps.ok_or("no pair of caps square to every side")?;
    // The bottom faces −axis.
    let axis = frames[bottom].1.scale(-1.0);
    let height = frames[top].0.sub(frames[bottom].0).dot(axis);
    if height <= tolerance {
        return Err("the caps are not apart along the axis".into());
    }
    let corners = loop_corners(source, &faces[bottom], tolerance)?;
    let top_corners = loop_corners(source, &faces[top], tolerance)?;
    if corners.len() < 3 || top_corners.len() != corners.len() {
        return Err("the caps' outlines differ in size".into());
    }
    let lift = axis.scale(height);
    for corner in &corners {
        if !top_corners.iter().any(|other| other.sub(corner.add(lift)).length() <= tolerance) {
            return Err("the top cap is not the bottom one moved along the axis".into());
        }
    }
    // Every side a rectangle across the height, one per outline edge.
    if faces.len() != corners.len() + 2 {
        return Err("the sides are not one face per outline edge".into());
    }
    let origin = corners[0];
    for (k, face) in faces.iter().enumerate() {
        if k == bottom || k == top {
            continue;
        }
        let side = loop_corners(source, face, tolerance)?;
        if side.len() != 4 {
            return Err("a side is not a rectangle".into());
        }
        for point in &side {
            let level = point.sub(origin).dot(axis);
            if level.abs() > tolerance && (level - height).abs() > tolerance {
                return Err("a side's corner is not on a cap".into());
            }
        }
    }
    let ex = corners[1].sub(corners[0]);
    let ex = ex.sub(axis.scale(ex.dot(axis))).normalized()?;
    let ey = axis.cross(ex);
    let mut outline: Vec<P2> = corners
        .iter()
        .map(|point| {
            let run = point.sub(origin);
            P2 { x: run.dot(ex), y: run.dot(ey) }
        })
        .collect();
    for (point, corner) in outline.iter().zip(&corners) {
        if corner.sub(origin).dot(axis).abs() > tolerance || !point.x.is_finite() {
            return Err("the bottom cap is not flat".into());
        }
    }
    // Collinear corners carry no reflex arc and no edge of their own.
    let mut k = 0;
    while outline.len() > 3 && k < outline.len() {
        let n = outline.len();
        let (a, b, c) = (outline[(k + n - 1) % n], outline[k], outline[(k + 1) % n]);
        if b.sub(a).cross(c.sub(b)).abs() <= tolerance * c.sub(a).len() && b.sub(a).dot(c.sub(b)) > 0.0 {
            outline.remove(k);
        } else {
            k += 1;
        }
    }
    let area: f64 = (0..outline.len()).map(|k| outline[k].cross(outline[(k + 1) % outline.len()])).sum::<f64>() * 0.5;
    if area.abs() <= tolerance * tolerance {
        return Err("the outline has no area".into());
    }
    if area < 0.0 {
        outline.reverse();
    }
    Ok(RightPrism { outline, origin, ex, ey, axis, height, faces })
}

/// The erosion's boundary loops, each counter-clockwise, and whether the
/// erosion changed the outline's topology. An edge not `real` bounds no
/// material (a solid of revolution's section closed along its axis): it is
/// kept as it stands where the erosion reaches it, is no distance's source,
/// and rounds no corner.
pub(super) fn eroded_loops(outline: &[P2], real: &[bool], radius: f64, tolerance: f64) -> Result<(Vec<Vec<Piece>>, bool), String> {
    let n = outline.len();
    let normal = |k: usize| {
        let run = outline[(k + 1) % n].sub(outline[k]);
        P2 { x: -run.y, y: run.x }.scale(1.0 / run.len())
    };
    let mut raw = Vec::new();
    for k in 0..n {
        let inward = if real[k] { normal(k).scale(radius) } else { P2 { x: 0.0, y: 0.0 } };
        raw.push(Piece::Line { a: outline[k].add(inward), b: outline[(k + 1) % n].add(inward), edge: k });
    }
    for k in 0..n {
        if !real[k] || !real[(k + n - 1) % n] {
            continue;
        }
        let (before, after) = (normal((k + n - 1) % n), normal(k));
        let turn = before.cross(after);
        // A reflex corner of a counter-clockwise outline turns clockwise.
        if turn < 0.0 && before.dot(after) < 1.0 - 1e-15 {
            let start = before.y.atan2(before.x);
            let sweep = -(before.cross(after).atan2(before.dot(after))).abs();
            raw.push(Piece::Arc { center: outline[k], start, sweep });
        }
    }
    let mut cuts: Vec<Vec<f64>> = vec![Vec::new(); raw.len()];
    for i in 0..raw.len() {
        for j in i + 1..raw.len() {
            for point in crossings(&raw[i], &raw[j], radius, tolerance)? {
                cuts[i].push(raw[i].parameter(point, radius, tolerance));
                cuts[j].push(raw[j].parameter(point, radius, tolerance));
            }
        }
    }
    let mut kept = Vec::new();
    let mut edge_kept = vec![false; n];
    for (piece, cuts) in raw.iter().zip(&mut cuts) {
        let length = piece.length(radius);
        let slack = tolerance / length.max(tolerance);
        let mut stops: Vec<f64> = vec![0.0];
        cuts.sort_by(f64::total_cmp);
        for &t in cuts.iter() {
            if t > slack && t < 1.0 - slack && t - stops.last().unwrap() > slack {
                stops.push(t);
            }
        }
        stops.push(1.0);
        for pair in stops.windows(2) {
            let part = piece.between(radius, pair[0], pair[1]);
            let middle = part.at(radius, 0.5);
            // Past an edge that is not real lies the mirror's side, never this
            // erosion's.
            let beyond = (0..n).filter(|&k| !real[k]).any(|k| {
                let (a, b) = (outline[k], outline[(k + 1) % n]);
                b.sub(a).cross(middle.sub(a)) < -tolerance * b.sub(a).len()
            });
            if beyond {
                continue;
            }
            let reach = outline_distance(outline, real, middle);
            if reach < radius - tolerance {
                continue;
            }
            if part.length(radius) <= 1e3 * tolerance {
                return Err("a boundary piece too short to read".into());
            }
            if let Piece::Line { edge, .. } = part {
                edge_kept[edge] = true;
            }
            kept.push(part);
        }
    }
    if kept.is_empty() {
        return Err("nothing is left".into());
    }
    // Chain: every end meets exactly one start.
    let ends: Vec<(P2, P2)> = kept.iter().map(|piece| (piece.at(radius, 0.0), piece.at(radius, 1.0))).collect();
    let snap = 100.0 * tolerance;
    let mut next = vec![usize::MAX; kept.len()];
    let mut taken = vec![false; kept.len()];
    for i in 0..kept.len() {
        let followers: Vec<usize> = (0..kept.len()).filter(|&j| ends[j].0.sub(ends[i].1).len() <= snap).collect();
        let [follower] = followers.as_slice() else {
            return Err(format!("a boundary end meets {} starts", followers.len()));
        };
        if taken[*follower] {
            return Err("two boundary ends meet one start".into());
        }
        taken[*follower] = true;
        next[i] = *follower;
    }
    let mut seen = vec![false; kept.len()];
    let mut loops = Vec::new();
    for first in 0..kept.len() {
        if seen[first] {
            continue;
        }
        let mut chain = Vec::new();
        let mut at = first;
        while !seen[at] {
            seen[at] = true;
            chain.push(kept[at]);
            at = next[at];
        }
        if at != first {
            return Err("a boundary chain does not close".into());
        }
        let mut area = 0.0;
        for piece in &chain {
            for k in 0..16 {
                let (a, b) = (piece.at(radius, k as f64 / 16.0), piece.at(radius, (k + 1) as f64 / 16.0));
                area += 0.5 * a.cross(b);
            }
        }
        if area <= tolerance * tolerance {
            return Err("a boundary loop is not counter-clockwise".into());
        }
        loops.push(chain);
    }
    let new_topology = loops.len() > 1 || edge_kept.iter().zip(real).any(|(kept, real)| *real && !kept);
    Ok((loops, new_topology))
}

/// `other`'s ids moved past every id `solid` holds, and its records appended.
pub(super) fn append_disjoint(solid: &mut BrepSolid, mut other: BrepSolid) {
    let mut top = 0;
    for vertex in &solid.vertices {
        top = top.max(vertex.id);
    }
    for edge in &solid.edges {
        top = top.max(edge.id);
    }
    for shell in &solid.shells {
        top = top.max(shell.id);
        for face in &shell.faces {
            top = top.max(face.id);
            for loop_record in &face.loops {
                top = top.max(loop_record.id);
                for coedge in &loop_record.coedges {
                    top = top.max(coedge.id);
                }
            }
        }
    }
    let shift = top + 1;
    for vertex in &mut other.vertices {
        vertex.id += shift;
    }
    for edge in &mut other.edges {
        edge.id += shift;
        edge.start_vertex_id += shift;
        edge.end_vertex_id += shift;
    }
    for shell in &mut other.shells {
        shell.id += shift;
        for face in &mut shell.faces {
            face.id += shift;
            for loop_record in &mut face.loops {
                loop_record.id += shift;
                for coedge in &mut loop_record.coedges {
                    coedge.id += shift;
                    coedge.edge_id += shift;
                }
            }
        }
    }
    solid.vertices.extend(other.vertices);
    solid.edges.extend(other.edges);
    solid.shells.extend(other.shells);
    solid.genus += other.genus;
}

fn prism_erosion_impl(source: &BrepSolid, depth: f64, only_new_topology: bool) -> Result<Option<OffsetShellResultRecord>, String> {
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;
    let prism = recognize(source, 1e-7 * scale)?;
    if prism.height <= 2.0 * depth + 1e-7 * scale {
        return Err("the moved caps meet or cross".into());
    }
    let real = vec![true; prism.outline.len()];
    let (loops, new_topology) = eroded_loops(&prism.outline, &real, depth, tolerance)?;
    if only_new_topology && !new_topology {
        return Ok(None);
    }
    let place = |point: P2| prism.origin.add(prism.ex.scale(point.x)).add(prism.ey.scale(point.y)).add(prism.axis.scale(depth));
    let mut body: Option<BrepSolid> = None;
    for chain in &loops {
        let mut curves = Vec::with_capacity(chain.len());
        // Each piece starts where the previous one ended, exactly.
        let joints: Vec<P2> = chain.iter().map(|piece| piece.at(depth, 0.0)).collect();
        for (k, piece) in chain.iter().enumerate() {
            let end = joints[(k + 1) % chain.len()];
            let curve = match *piece {
                Piece::Line { .. } => make_line(place(joints[k]), place(end))?,
                Piece::Arc { center, start, sweep } if sweep > 0.0 => make_arc(place(center), prism.ex, prism.ey, depth, start, start + sweep)?,
                Piece::Arc { center, start, sweep } => {
                    make_arc(place(center), prism.ex, prism.ey.scale(-1.0), depth, -start, -start - sweep)?
                }
            };
            curves.push(curve);
        }
        let part = extrude_profile_brep(&curves, prism.axis, prism.height - 2.0 * depth).map_err(|error| error.message)?;
        match &mut body {
            None => body = Some(part),
            Some(solid) => append_disjoint(solid, part),
        }
    }
    let solid = body.ok_or("nothing is left")?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!("the eroded prism does not validate: {:?}", issues.first()));
    }
    // Provenance: a cap or a side on a moved source plane is that face's image;
    // an arc's cylinder rounds a corner and has no single source face.
    let source_planes: Vec<(u64, Vec3, Vec3)> =
        prism.faces.iter().filter_map(|face| outward(face).ok().map(|(point, normal)| (face.id, point, normal))).collect();
    let mut face_images = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let mut source_face_id = 0;
        if face.surface.is_affine().unwrap_or(false) {
            if let Ok((point, normal)) = outward(face) {
                for (id, at, along) in &source_planes {
                    if normal.dot(*along) >= 1.0 - 1e-9 && (point.sub(*at).dot(*along) + depth).abs() <= 1e-7 * scale {
                        source_face_id = *id;
                        break;
                    }
                }
            }
        }
        face_images.push(OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id });
    }
    Ok(Some(OffsetShellResultRecord { solid, face_images }))
}
