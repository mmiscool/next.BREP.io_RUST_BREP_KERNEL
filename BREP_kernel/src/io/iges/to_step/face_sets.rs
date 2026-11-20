//! Trimmed-surface sets (144, 143) to Part 21.
//!
//! A 144 carries its own copy of every boundary curve and says nothing about
//! neighbours or orientation, so the topology a 186 states has to be recovered:
//! coincident vertices weld at the file's stated resolution, coincident curves
//! become one shared edge, and agreement along shared edges orients the faces of
//! each connected body. Which way the whole body faces is not knowable from
//! this data, so each body is offered in both senses and the importer's signed
//! volume picks (see `import::face_set_body`).

use super::*;
use rustc_hash::FxHashSet as HashSet;

/// How far apart two statements of one curve may be at the probed points, as a
/// fraction of the distance between its ends. Foreign writers approximate each
/// face's copy independently; two genuinely different edges between the same
/// vertices (the halves of a lens) are separated by far more.
const SAME_CURVE_FRACTION: f64 = 1e-3;
/// The same, as a multiple of the file's stated resolution.
const SAME_CURVE_RESOLUTIONS: f64 = 10.0;

/// A boundary curve as the file states it, before anything is emitted.
#[derive(Clone)]
struct CurveRef {
    pointer: i64,
    parent: Transform,
    start: Vec3,
    end: Vec3,
    /// Points a quarter and half way along; `None` for a conic the probes
    /// cannot place.
    probes: Option<[Vec3; 2]>,
    /// The part of the curve this boundary piece is, as fractions along it
    /// (0 at its start, 1 at its end): the whole curve unless a T-junction
    /// cut it.
    range: (f64, f64),
}

struct SoupEdge {
    curve: CurveRef,
    start: usize,
    end: usize,
    uses: Vec<(usize, bool)>,
    /// Whether the curve runs from `start` to `end`, for a piece of a curve
    /// (whose ends are not the curve's); `None` reads it from the curve's ends.
    forward: Option<bool>,
}

struct SoupFace {
    pointer: i64,
    surface: i64,
    parent: Transform,
    /// Loops of (edge index, runs along the edge's own direction).
    loops: Vec<Vec<(usize, bool)>>,
    /// Whether the boundary as stated winds counter-clockwise about the STEP
    /// carrier's normal; `None` when that could not be measured.
    counter_clockwise: Option<bool>,
}

/// One body's faces and how each must flip to agree with its neighbours.
pub(in super::super) struct FaceSets {
    faces: Vec<SoupFace>,
    edges: Vec<SoupEdge>,
    vertices: Vec<Vec3>,
    flips: Vec<bool>,
    /// Face indices of each connected body.
    pub(in super::super) bodies: Vec<Vec<usize>>,
    resolution: f64,
}

/// Point welding on a hash grid.
struct Welder {
    tolerance: f64,
    cells: HashMap<[i64; 3], Vec<usize>>,
    points: Vec<Vec3>,
}

impl Welder {
    fn weld(&mut self, p: Vec3) -> usize {
        let cell = |v: f64| (v / (2.0 * self.tolerance)).floor() as i64;
        let key = [cell(p.x), cell(p.y), cell(p.z)];
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let near = [key[0] + dx, key[1] + dy, key[2] + dz];
                    if let Some(found) = self.cells.get(&near).and_then(|list| {
                        list.iter().copied().find(|&i| self.points[i].sub(p).length() <= self.tolerance)
                    }) {
                        return found;
                    }
                }
            }
        }
        let index = self.points.len();
        self.points.push(p);
        self.cells.entry(key).or_default().push(index);
        index
    }
}

/// Every trimmed surface (144, 143) of the file, in directory order.
pub(in super::super) fn trimmed_surfaces(file: &IgesFile) -> Vec<i64> {
    let mut pointers: Vec<i64> = file.pointers_of_type(144).iter().map(|(p, _)| *p).collect();
    pointers.extend(file.pointers_of_type(143).iter().map(|(p, _)| *p));
    pointers.sort_unstable();
    pointers
}

/// Read the trimmed surfaces at `pointers` (in directory order) into welded,
/// oriented bodies. Only these surfaces weld to one another: a subfigure's
/// surfaces are read apart from the file's own and from other subfigures',
/// which are defined in frames of their own and may overlap them.
pub(in super::super) fn analyse_face_sets(file: &IgesFile, scale: f64, pointers: &[i64]) -> Result<FaceSets, String> {
    let reader = Translator::new(file, scale, false);
    // Surfaces are translated once here only to learn their charts.
    let mut charts = Translator::new(file, scale, false);
    let mut stated = Vec::new();
    for &pointer in pointers {
        let entity = file.entity(pointer)?;
        let parent = reader.placed(entity, &Transform::IDENTITY)?;
        let (surface, boundaries) = reader
            .face_boundaries(entity, &parent)
            .map_err(|error| format!("iges_import: trimmed surface DE {pointer}: {error}"))?;
        stated.push((pointer, entity, parent, surface, boundaries));
    }
    let resolution = reader.resolution;
    let mut welder = Welder { tolerance: resolution, cells: HashMap::default(), points: Vec::new() };
    // Consecutive curves of one boundary meet: that is what a boundary is.
    // A writer's two statements of that meeting point can stand further apart
    // than the resolution it states (a full-turn sphere's seam ending 1.7e-4
    // from where its rim circle starts, under a stated 7e-5), and welded by
    // position alone they are two vertices and the loop is open. Such a pair
    // is one vertex, where it is no further apart than two statements of one
    // curve may be and joining it folds neither curve onto a point.
    // `BREP_IGES_LOOP_JOIN` (default on; `=0` restores welding by position alone).
    let join = crate::switch!("BREP_IGES_LOOP_JOIN", true);
    let mut same: Vec<usize> = Vec::new();
    fn root(same: &[usize], mut index: usize) -> usize {
        while same[index] != index {
            index = same[index];
        }
        index
    }
    for (_, _, _, _, boundaries) in &stated {
        for boundary in boundaries {
            let ends: Vec<(usize, usize)> = boundary
                .iter()
                .map(|(curve, reversed)| {
                    let (a, b) = (welder.weld(curve.start), welder.weld(curve.end));
                    if *reversed { (b, a) } else { (a, b) }
                })
                .collect();
            same.extend(same.len()..welder.points.len());
            for index in 0..ends.len() {
                let next = (index + 1) % ends.len();
                let (leaves, enters) = (root(&same, ends[index].1), root(&same, ends[next].0));
                if leaves == enters {
                    continue;
                }
                let gap = welder.points[leaves].sub(welder.points[enters]).length();
                let folds = |pair: (usize, usize)| {
                    let (a, b) = (root(&same, pair.0), root(&same, pair.1));
                    a != b && [a, b].contains(&leaves) && [a, b].contains(&enters)
                };
                if join && gap <= SAME_CURVE_RESOLUTIONS * resolution && !folds(ends[index]) && !folds(ends[next]) {
                    let (keep, drop) = (leaves.min(enters), leaves.max(enters));
                    same[drop] = keep;
                }
            }
        }
    }
    let mut edges: Vec<SoupEdge> = Vec::new();
    let mut by_ends: HashMap<(usize, usize), Vec<usize>> = HashMap::default();
    let mut faces: Vec<SoupFace> = Vec::new();

    for (pointer, entity, parent, surface, boundaries) in stated {
        let carrier = charts
            .surface(surface, &parent)
            .map_err(|error| format!("iges_import: trimmed surface DE {pointer}: {error}"))?;
        charts.text.clear();
        // A boundary splits a sphere into two bounded pieces, so neither the chart
        // nor the standard's convention says which is the face: only reading
        // around a point the surface covers does.
        let counter_clockwise = if let Some(chart @ Chart::Sphere { .. }) = carrier.chart {
            // The file's parameter curves say it outright; without them, read
            // around a point the 120 covers.
            let read = reader
                .sphere_parameter_winding(entity, &parent, carrier.reversed)
                .or_else(|| carrier.interior.and_then(|interior| reader.sphere_winding(interior, chart, &boundaries)));
            if read.is_none() {
                return Err(format!(
                    "iges_import: trimmed surface DE {pointer} lies on a sphere, and the file does not say which side of its boundary it covers (no parameter-space curves, and the boundary does not enclose the surface's middle)"
                ));
            }
            read
        } else {
            reader.winding(entity, &parent, carrier.chart, carrier.domain_v, carrier.spline.as_deref(), &boundaries)
        };
        if std::env::var_os("BREP_DEBUG_IGES_WINDING").is_some() {
            // Audit: the projected reading beside the file's parameter curves,
            // and every face whose side is still taken as written.
            let kind = file.entity(surface).map(|e| e.de.entity_type).unwrap_or(0);
            let chart = match carrier.chart {
                None => "none",
                Some(Chart::Parameter) => "parameter",
                Some(Chart::Plane { .. }) => "plane",
                Some(Chart::Axial { .. }) => "axial",
                Some(Chart::Sphere { .. }) => "sphere",
                Some(Chart::Torus { .. }) => "torus",
            };
            let parameter = matches!(carrier.chart, Some(Chart::Parameter)) && reader.parameter_curves(entity).is_some();
            let projected = carrier
                .spline
                .as_deref()
                .and_then(|spline| reader.spline_loops(spline, &boundaries))
                .and_then(|loops| signed_chart_area(&loops, (false, false), None))
                .map(|area| area > 0.0);
            let (sphere_parameter, sphere_point) = match carrier.chart {
                Some(sphere @ Chart::Sphere { .. }) => (
                    reader.sphere_parameter_winding(entity, &parent, carrier.reversed),
                    carrier.interior.and_then(|interior| reader.sphere_winding(interior, sphere, &boundaries)),
                ),
                _ => (None, None),
            };
            eprintln!(
                "IGES_WINDING de={pointer} surface={kind} chart={chart} parameter_curves={parameter} read={counter_clockwise:?} projected={projected:?} sphere_parameter={sphere_parameter:?} sphere_point={sphere_point:?}"
            );
        }
        let face_index = faces.len();
        let mut loops = Vec::with_capacity(boundaries.len());
        for boundary in boundaries {
            let mut coedges = Vec::with_capacity(boundary.len());
            for (curve, reversed) in boundary {
                let a = root(&same, welder.weld(curve.start));
                let b = root(&same, welder.weld(curve.end));
                let edge = match_edge(&reader, &mut edges, &mut by_ends, curve, a, b, (face_index, reversed), resolution);
                let along = (a == edges[edge.0].start && b == edges[edge.0].end && edge.1) != reversed;
                edges[edge.0].uses.push((face_index, along));
                coedges.push((edge.0, along));
            }
            loops.push(coedges);
        }
        faces.push(SoupFace { pointer, surface, parent, loops, counter_clockwise });
    }
    if crate::switch!("BREP_IGES_T_JUNCTIONS", true) {
        split_at_t_junctions(&reader, &mut edges, &mut faces, &welder.points, resolution);
    }
    let (flips, bodies) = orient(&faces, &edges)?;
    Ok(FaceSets { faces, edges, vertices: welder.points, flips, bodies, resolution })
}

/// Find the edge this statement of a curve belongs to, or start a new one.
/// The second value says the curve runs the way the stored edge does.
fn match_edge(
    reader: &Translator,
    edges: &mut Vec<SoupEdge>,
    by_ends: &mut HashMap<(usize, usize), Vec<usize>>,
    curve: CurveRef,
    a: usize,
    b: usize,
    (face, reversed): (usize, bool),
    resolution: f64,
) -> (usize, bool) {
    let key = (a.min(b), a.max(b));
    let bar = (SAME_CURVE_RESOLUTIONS * resolution).max(SAME_CURVE_FRACTION * curve.start.sub(curve.end).length());
    for &index in by_ends.get(&key).map_or(&[][..], Vec::as_slice) {
        // A face walks one edge twice only as a seam, once each way: a second
        // statement it walks the same way between the same ends (a curve
        // shorter than the bar, stated twice) is another edge.
        if a != b && edges[index].uses.contains(&(face, (a == edges[index].start) != reversed)) {
            continue;
        }
        let other = &edges[index].curve;
        let (Some(mine), Some(theirs)) = (curve.probes, other.probes) else {
            // A conic with no probes: its ends are all there is to go on.
            return (index, a == edges[index].start);
        };
        if a != b {
            if mine[1].sub(theirs[1]).length() <= bar {
                return (index, a == edges[index].start);
            }
            // Two statements of one curve need not share a parametrization (a
            // rational spline's weights, its knots), so the points half way
            // along each by parameter can stand apart on the one locus: each
            // statement's probes lying on the other is the same curve.
            // `BREP_IGES_LOCUS_MATCH` (default on; `=0` compares by parameter alone).
            if crate::switch!("BREP_IGES_LOCUS_MATCH", true) && reader.same_locus(&curve, other, bar) {
                return (index, a == edges[index].start);
            }
        } else {
            let closed_bar = bar.max(SAME_CURVE_FRACTION * mine[1].sub(curve.start).length());
            // As for an open edge, a face walks a closed one twice only once
            // each way (a seam round a torus).
            let walked_twice = |same: bool| edges[index].uses.contains(&(face, same != reversed));
            if mine[0].sub(theirs[0]).length() <= closed_bar && mine[1].sub(theirs[1]).length() <= closed_bar {
                if walked_twice(true) {
                    continue;
                }
                return (index, true);
            }
            // The other way round a closed curve: a quarter of the way along
            // one is three quarters along the other, and the half is shared.
            if mine[1].sub(theirs[1]).length() <= closed_bar && !walked_twice(false) {
                return (index, false);
            }
        }
    }
    let index = edges.len();
    edges.push(SoupEdge { curve, start: a, end: b, uses: Vec::new(), forward: None });
    by_ends.entry(key).or_default().push(index);
    (index, true)
}

/// Split each edge only one face bounds where the end of another such edge
/// lies on it, and join the pieces that then meet. Writers often cut a
/// boundary differently on its two sides: one face's long edge runs past the
/// corners of two shorter edges of its neighbour (a T-junction), and matched
/// whole, the long edge and the short ones never pair. A point within `bar`
/// (two statements of one point) of the edge's curve, away from its ends,
/// splits it there; the pieces are edges on the same curve between those
/// vertices, and two pieces (or a piece and an edge) between the same
/// vertices on one locus become one edge. Every face's loop walks the pieces
/// in its edge's place. `BREP_IGES_T_JUNCTIONS` (default on; `=0` matches
/// whole edges only).
fn split_at_t_junctions(
    reader: &Translator,
    edges: &mut Vec<SoupEdge>,
    faces: &mut [SoupFace],
    vertices: &[Vec3],
    resolution: f64,
) {
    // Two statements of one point first; what that leaves open, again at ten
    // times the distance (writers whose sides stand further apart than they
    // say: 0.012 mm under a stated 0.001).
    for factor in [1.0, 10.0] {
        sew_free_edges(reader, edges, faces, vertices, factor * SAME_CURVE_RESOLUTIONS * resolution);
    }
}

/// One pass of [`split_at_t_junctions`] at `bar`.
fn sew_free_edges(reader: &Translator, edges: &mut Vec<SoupEdge>, faces: &mut [SoupFace], vertices: &[Vec3], bar: f64) {
    let free: Vec<usize> = (0..edges.len()).filter(|&e| edges[e].uses.len() == 1).collect();
    if free.is_empty() {
        return;
    }
    weld_free_ends(edges, faces, &free, vertices, bar);
    let free: Vec<usize> = free.into_iter().filter(|&e| edges[e].uses.len() == 1 && edges[e].start != edges[e].end).collect();
    if free.is_empty() {
        return;
    }
    // The ends of the free edges, on a grid.
    let mut ends: Vec<usize> = free.iter().flat_map(|&e| [edges[e].start, edges[e].end]).collect();
    ends.sort_unstable();
    ends.dedup();
    let extent = |e: usize| {
        let curve = &edges[e].curve;
        let mut points = vec![curve.start, curve.end];
        points.extend(curve.probes.into_iter().flatten());
        let chord = curve.start.sub(curve.end).length();
        (points, chord)
    };
    let cell = {
        let mut chords: Vec<f64> = free.iter().map(|&e| extent(e).1).collect();
        chords.sort_by(f64::total_cmp);
        chords[chords.len() / 2].max(1000.0 * bar)
    };
    let key = |p: Vec3| [(p.x / cell).floor() as i64, (p.y / cell).floor() as i64, (p.z / cell).floor() as i64];
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::default();
    for &v in &ends {
        grid.entry(key(vertices[v])).or_default().push(v);
    }
    // Each free edge's split points: (fraction along its curve, vertex).
    let mut pieces_of: Vec<(usize, Vec<(f64, usize)>)> = Vec::new();
    for &e in &free {
        // A polyline through the curve, and how far the curve strays from it,
        // bound where a point on it can be and rule out most candidates
        // without a search along the curve.
        let Some((polyline, sag)) = reader.polyline(&edges[e].curve) else { continue };
        let margin = bar + sag;
        let corner = |p: Vec3, by: f64| key(Vec3::new(p.x + by, p.y + by, p.z + by));
        let (mut lo, mut hi) = (corner(polyline[0], -margin), corner(polyline[0], margin));
        for &p in &polyline {
            let (low, high) = (corner(p, -margin), corner(p, margin));
            for axis in 0..3 {
                lo[axis] = lo[axis].min(low[axis]);
                hi[axis] = hi[axis].max(high[axis]);
            }
        }
        let mut candidates = Vec::new();
        for x in lo[0]..=hi[0] {
            for y in lo[1]..=hi[1] {
                for z in lo[2]..=hi[2] {
                    if let Some(list) = grid.get(&[x, y, z]) {
                        candidates.extend(list.iter().copied());
                    }
                }
            }
        }
        let (start, end) = (edges[e].start, edges[e].end);
        let mut splits = Vec::new();
        for v in candidates {
            let point = vertices[v];
            if v == start || v == end || point.sub(vertices[start]).length() <= bar || point.sub(vertices[end]).length() <= bar {
                continue;
            }
            if distance_to_polyline(&polyline, point) > bar + sag {
                continue;
            }
            if let Some((distance, t)) = reader.nearest_on_curve(&edges[e].curve, point) {
                if distance <= bar && t > 0.0 && t < 1.0 {
                    splits.push((t, v));
                }
            }
        }
        if splits.is_empty() {
            continue;
        }
        let at = |v: usize| reader.nearest_on_curve(&edges[e].curve, vertices[v]).map_or(0.0, |(_, t)| t);
        splits.push((at(start), start));
        splits.push((at(end), end));
        splits.sort_by(|a, b| a.0.total_cmp(&b.0));
        splits.dedup_by_key(|(_, v)| *v);
        if splits.len() > 2 {
            pieces_of.push((e, splits));
        }
    }
    // Cut each edge into its pieces and walk them in its place.
    for (e, splits) in pieces_of {
        let Ok(entity) = reader.file.entity(edges[e].curve.pointer) else { continue };
        let Ok(transform) = reader.placed(entity, &edges[e].curve.parent) else { continue };
        let (low, high) = edges[e].curve.range;
        let point = |t: f64| reader.curve_point(entity, &transform, low + (high - low) * t);
        let mut pieces = Vec::with_capacity(splits.len() - 1);
        for pair in splits.windows(2) {
            let ((t0, a), (t1, b)) = (pair[0], pair[1]);
            let (Some(from), Some(to)) = (point(t0), point(t1)) else { break };
            let probes = point(t0 + 0.25 * (t1 - t0)).zip(point(t0 + 0.5 * (t1 - t0))).map(|(q, h)| [q, h]);
            let (low, high) = edges[e].curve.range;
            let range = (low + (high - low) * t0, low + (high - low) * t1);
            let curve = CurveRef { start: from, end: to, probes, range, ..edges[e].curve.clone() };
            pieces.push(SoupEdge { curve, start: a, end: b, uses: Vec::new(), forward: Some(true) });
        }
        if pieces.len() != splits.len() - 1 {
            continue;
        }
        let (face, along) = edges[e].uses[0];
        // The walk runs along the curve's parameter when it leaves from the
        // end that is first along it.
        let increasing = along == (splits[0].1 == edges[e].start);
        let first = edges.len();
        let count = pieces.len();
        for (offset, mut piece) in pieces.into_iter().enumerate() {
            let _ = offset;
            piece.uses.push((face, increasing));
            edges.push(piece);
        }
        let mut walk: Vec<(usize, bool)> = (first..first + count).map(|index| (index, increasing)).collect();
        if !increasing {
            walk.reverse();
        }
        for coedges in &mut faces[face].loops {
            if let Some(position) = coedges.iter().position(|&(edge, _)| edge == e) {
                coedges.splice(position..=position, walk.iter().copied());
                break;
            }
        }
        edges[e].uses.clear();
    }
    // Join what now meets: two one-use edges between the same vertices on one
    // locus.
    let mut by_ends: HashMap<(usize, usize), Vec<usize>> = HashMap::default();
    for index in 0..edges.len() {
        let edge = &edges[index];
        if edge.uses.len() == 1 && edge.start != edge.end {
            by_ends.entry((edge.start.min(edge.end), edge.start.max(edge.end))).or_default().push(index);
        }
    }
    let mut groups: Vec<Vec<usize>> = by_ends.into_values().filter(|group| group.len() > 1).collect();
    groups.sort_unstable();
    for group in groups {
        for i in 0..group.len() {
            let keep = group[i];
            if edges[keep].uses.len() != 1 {
                continue;
            }
            for &other in &group[i + 1..] {
                if edges[other].uses.len() != 1 {
                    continue;
                }
                let (face, along) = edges[other].uses[0];
                let same_way = edges[other].start == edges[keep].start;
                let walked = along == same_way;
                if edges[keep].uses[0] == (face, walked) {
                    continue;
                }
                let bar = bar.max(SAME_CURVE_FRACTION * edges[keep].curve.start.sub(edges[keep].curve.end).length());
                if !reader.same_locus(&edges[keep].curve, &edges[other].curve, bar) {
                    continue;
                }
                edges[keep].uses.push((face, walked));
                edges[other].uses.clear();
                for coedges in &mut faces[face].loops {
                    for coedge in coedges.iter_mut() {
                        if *coedge == (other, along) {
                            *coedge = (keep, walked);
                        }
                    }
                }
                break;
            }
        }
    }
}

/// The least distance from `point` to a polyline.
fn distance_to_polyline(polyline: &[Vec3], point: Vec3) -> f64 {
    polyline
        .windows(2)
        .map(|pair| {
            let (a, b) = (pair[0], pair[1]);
            let along = b.sub(a);
            let length = along.dot(along);
            let t = if length > 0.0 { (point.sub(a).dot(along) / length).clamp(0.0, 1.0) } else { 0.0 };
            point.sub(a.add(along.scale(t))).length()
        })
        .fold(f64::INFINITY, f64::min)
}

/// Weld the ends of edges only one face bounds where they stand within `bar`
/// of each other: one corner the two sides state a little apart, further
/// than the file's resolution, which welding by position alone left as two
/// vertices. First, such an edge that lies wholly within `bar` (a speck one
/// side states at a corner and the other does not) leaves its loop, and its
/// ends are one vertex. Otherwise two ends of one edge are never welded (that
/// would fold it onto a point).
fn weld_free_ends(edges: &mut [SoupEdge], faces: &mut [SoupFace], free: &[usize], vertices: &[Vec3], bar: f64) {
    let mut root: HashMap<usize, usize> = HashMap::default();
    fn find(root: &HashMap<usize, usize>, mut v: usize) -> usize {
        while let Some(&up) = root.get(&v) {
            v = up;
        }
        v
    }
    let shared: Vec<(usize, usize)> = edges.iter().filter(|e| e.uses.len() > 1).map(|e| (e.start, e.end)).collect();
    for &e in free {
        let curve = &edges[e].curve;
        let mut points = vec![curve.start, curve.end];
        points.extend(curve.probes.into_iter().flatten());
        let speck = points.iter().all(|p| points.iter().all(|q| p.sub(*q).length() <= bar))
            && vertices[edges[e].start].sub(vertices[edges[e].end]).length() <= bar;
        let (a, b) = (find(&root, edges[e].start), find(&root, edges[e].end));
        // An edge two faces share between these two vertices (as they now
        // weld) must keep its length.
        let between = |(start, end): &(usize, usize)| {
            let (c, d) = (find(&root, *start), find(&root, *end));
            (c == a && d == b) || (c == b && d == a)
        };
        if !speck || (a != b && shared.iter().any(between)) {
            continue;
        }
        let (face, along) = edges[e].uses[0];
        let Some(coedges) = faces[face].loops.iter_mut().find(|coedges| coedges.contains(&(e, along))) else { continue };
        if coedges.len() < 2 {
            continue;
        }
        coedges.retain(|&coedge| coedge != (e, along));
        edges[e].uses.clear();
        if a != b {
            root.insert(a.max(b), a.min(b));
        }
    }
    let ends: Vec<usize> = {
        let mut ends: Vec<usize> =
            free.iter().filter(|&&e| !edges[e].uses.is_empty()).flat_map(|&e| [edges[e].start, edges[e].end]).collect();
        ends.sort_unstable();
        ends.dedup();
        ends
    };
    let key = |p: Vec3| [(p.x / bar).floor() as i64, (p.y / bar).floor() as i64, (p.z / bar).floor() as i64];
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::default();
    for &v in &ends {
        grid.entry(key(vertices[v])).or_default().push(v);
    }
    // Vertices joined by an edge, which must stay apart, kept by the vertex
    // each now welds to as welds are made.
    let mut adjacent: HashMap<usize, HashSet<usize>> = HashMap::default();
    for edge in edges.iter().filter(|e| !e.uses.is_empty()) {
        let (a, b) = (find(&root, edge.start), find(&root, edge.end));
        if a != b {
            adjacent.entry(a).or_default().insert(b);
            adjacent.entry(b).or_default().insert(a);
        }
    }
    for &v in &ends {
        let k = key(vertices[v]);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(list) = grid.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) else { continue };
                    for &w in list {
                        if w <= v || vertices[v].sub(vertices[w]).length() > bar {
                            continue;
                        }
                        let (a, b) = (find(&root, v), find(&root, w));
                        if a == b || adjacent.get(&a).is_some_and(|near| near.contains(&b)) {
                            continue;
                        }
                        let (keep, drop) = (a.min(b), a.max(b));
                        root.insert(drop, keep);
                        for neighbour in adjacent.remove(&drop).unwrap_or_default() {
                            if let Some(near) = adjacent.get_mut(&neighbour) {
                                near.remove(&drop);
                                near.insert(keep);
                            }
                            adjacent.entry(keep).or_default().insert(neighbour);
                        }
                    }
                }
            }
        }
    }
    if root.is_empty() {
        return;
    }
    for edge in edges.iter_mut() {
        edge.start = find(&root, edge.start);
        edge.end = find(&root, edge.end);
    }
}

/// Propagate agreement across shared edges. Returns per-face flips and the
/// connected bodies, in first-seen order.
fn orient(faces: &[SoupFace], edges: &[SoupEdge]) -> Result<(Vec<bool>, Vec<Vec<usize>>), String> {
    let mut flips = vec![None::<bool>; faces.len()];
    let mut bodies = Vec::new();
    for seed in 0..faces.len() {
        if flips[seed].is_some() {
            continue;
        }
        flips[seed] = Some(false);
        let mut body = vec![seed];
        let mut stack = vec![seed];
        while let Some(face) = stack.pop() {
            let flip = flips[face].unwrap_or(false);
            let touched: HashSet<usize> =
                faces[face].loops.iter().flatten().map(|(edge, _)| *edge).collect();
            for edge in touched {
                let uses = &edges[edge].uses;
                if uses.len() != 2 {
                    continue;
                }
                let [(a, along_a), (b, along_b)] = [uses[0], uses[1]];
                if a == b {
                    continue;
                }
                let (other, along_here, along_there) =
                    if a == face { (b, along_a, along_b) } else { (a, along_b, along_a) };
                // Neighbours agree when they walk the shared edge opposite ways.
                let want = flip != (along_here == along_there);
                match flips[other] {
                    None => {
                        flips[other] = Some(want);
                        body.push(other);
                        stack.push(other);
                    }
                    Some(have) if have != want => {
                        return Err(format!(
                            "iges_import: trimmed surfaces DE {} and DE {} cannot be oriented consistently across their shared edge",
                            faces[face].pointer, faces[other].pointer
                        ));
                    }
                    Some(_) => {}
                }
            }
        }
        bodies.push(body);
    }
    Ok((flips.into_iter().map(|f| f.unwrap_or(false)).collect(), bodies))
}

impl FaceSets {
    /// Why this body cannot be a solid, or `None` when every edge is matched.
    pub(in super::super) fn open_reason(&self, body: &[usize]) -> Option<String> {
        let members: HashSet<usize> = body.iter().copied().collect();
        let mut single = 0usize;
        let mut many = 0usize;
        let mut worst = None;
        for edge in &self.edges {
            if !edge.uses.iter().any(|(face, _)| members.contains(face)) {
                continue;
            }
            if edge.start == edge.end && edge.curve.probes.is_some_and(|p| {
                p.iter().all(|q| q.sub(edge.curve.start).length() <= self.resolution)
            }) {
                continue; // A pole: it bounds one face by nature.
            }
            match edge.uses.len() {
                1 => {
                    single += 1;
                    worst.get_or_insert(edge.curve.start);
                }
                2 => {}
                _ => many += 1,
            }
        }
        if single + many == 0 {
            return None;
        }
        let at = worst.map_or(String::new(), |p| format!(", first near ({:.3}, {:.3}, {:.3})", p.x, p.y, p.z));
        Some(format!(
            "iges_import: a set of {} trimmed surfaces does not close into a solid: {single} edge(s) bound one face and {many} bound three or more{at}",
            body.len()
        ))
    }

    /// A Part 21 document holding one solid made of `outer`, with `voids` as
    /// its cavities. `flip` turns the outer shell over; a void always faces the
    /// other way from its solid.
    pub(in super::super) fn to_step(
        &self,
        file: &IgesFile,
        scale: f64,
        outer: &[usize],
        voids: &[(&[usize], bool)],
        flip: bool,
        split_bridges: bool,
    ) -> Result<String, String> {
        let mut out = Translator::new(file, scale, split_bridges);
        let mut vertex_ids: HashMap<usize, usize> = HashMap::default();
        let mut edge_ids: HashMap<usize, usize> = HashMap::default();
        let shell = self.shell(&mut out, outer, flip, &mut vertex_ids, &mut edge_ids, "CLOSED_SHELL")?;
        if voids.is_empty() {
            out.add(format!("MANIFOLD_SOLID_BREP('',#{shell})"));
        } else {
            let mut cavities = Vec::with_capacity(voids.len());
            for (faces, void_flip) in voids {
                let void_shell = self.shell(&mut out, faces, *void_flip, &mut vertex_ids, &mut edge_ids, "CLOSED_SHELL")?;
                cavities.push(format!("#{}", out.add(format!("ORIENTED_CLOSED_SHELL('',*,#{void_shell},.T.)"))));
            }
            out.add(format!("BREP_WITH_VOIDS('',#{shell},({}))", cavities.join(",")));
        }
        Ok(out.finish(&format!("sets{}", outer.first().copied().unwrap_or(0))))
    }

    /// A Part 21 document holding `body` as one open shell of a surface model:
    /// a set that does not close, as a sheet. Its faces keep the senses their
    /// agreement along shared edges gives them.
    pub(in super::super) fn to_step_sheet(
        &self,
        file: &IgesFile,
        scale: f64,
        body: &[usize],
        split_bridges: bool,
    ) -> Result<String, String> {
        let mut out = Translator::new(file, scale, split_bridges);
        let mut vertex_ids: HashMap<usize, usize> = HashMap::default();
        let mut edge_ids: HashMap<usize, usize> = HashMap::default();
        let shell = self.shell(&mut out, body, false, &mut vertex_ids, &mut edge_ids, "OPEN_SHELL")?;
        out.add(format!("SHELL_BASED_SURFACE_MODEL('',(#{shell}))"));
        Ok(out.finish(&format!("sheet{}", body.first().copied().unwrap_or(0))))
    }

    fn shell(
        &self,
        out: &mut Translator,
        body: &[usize],
        flip: bool,
        vertex_ids: &mut HashMap<usize, usize>,
        edge_ids: &mut HashMap<usize, usize>,
        kind: &str,
    ) -> Result<usize, String> {
        let mut face_ids = Vec::with_capacity(body.len());
        for &index in body {
            let face = &self.faces[index];
            let reversed = self.flips[index] != flip;
            let surface = out
                .surface(face.surface, &face.parent)
                .map_err(|error| format!("iges_import: trimmed surface DE {}: {error}", face.pointer))?;
            let mut bounds = Vec::with_capacity(face.loops.len());
            for (loop_index, coedges) in face.loops.iter().enumerate() {
                let mut stated = Vec::with_capacity(coedges.len());
                for &(edge, along) in coedges {
                    stated.push((self.edge_id(out, edge, vertex_ids, edge_ids)?, along));
                }
                let loops = out
                    .emit_loops(stated, &surface, face.pointer)
                    .map_err(|error| format!("iges_import: {error}"))?;
                let outer = loop_index == 0 && loops.len() == 1;
                for loop_id in loops {
                    let kind = if outer { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                    let sense = if reversed { ".F." } else { ".T." };
                    let bound = out.add(format!("{kind}('',#{loop_id},{sense})"));
                    bounds.push(format!("#{bound}"));
                }
            }
            // A face's normal is the one its boundary winds counter-clockwise
            // about, so the carrier's agreement with it is measured. Without
            // a measurement, fall back on the standard's stated convention.
            let same_sense = match face.counter_clockwise {
                Some(counter_clockwise) => counter_clockwise != reversed,
                None => !reversed != surface.reversed,
            };
            face_ids.push(format!(
                "#{}",
                out.add(format!(
                    "ADVANCED_FACE('',({}),#{},{})",
                    bounds.join(","),
                    surface.id,
                    if same_sense { ".T." } else { ".F." }
                ))
            ));
        }
        Ok(out.add(format!("{kind}('',({}))", face_ids.join(","))))
    }

    fn vertex_id(
        &self,
        out: &mut Translator,
        vertex: usize,
        vertex_ids: &mut HashMap<usize, usize>,
    ) -> usize {
        if let Some(&id) = vertex_ids.get(&vertex) {
            return id;
        }
        let location = self.vertices[vertex];
        let point = out.point_id(location);
        let id = out.add(format!("VERTEX_POINT('',#{point})"));
        out.vertex_points.insert(id, location);
        vertex_ids.insert(vertex, id);
        id
    }

    fn edge_id(
        &self,
        out: &mut Translator,
        edge: usize,
        vertex_ids: &mut HashMap<usize, usize>,
        edge_ids: &mut HashMap<usize, usize>,
    ) -> Result<usize, String> {
        if let Some(&id) = edge_ids.get(&edge) {
            return Ok(id);
        }
        let record = &self.edges[edge];
        let curve = out
            .curve(record.curve.pointer, &record.curve.parent)
            .map_err(|error| format!("iges_import: {error}"))?;
        let start = self.vertex_id(out, record.start, vertex_ids);
        let end = self.vertex_id(out, record.end, vertex_ids);
        let (from, to) = (self.vertices[record.start], self.vertices[record.end]);
        let same_sense = record.forward.unwrap_or_else(|| {
            curve.start.sub(from).length() + curve.end.sub(to).length()
                <= curve.start.sub(to).length() + curve.end.sub(from).length()
        });
        let id = out.add(format!(
            "EDGE_CURVE('',#{start},#{end},#{},{})",
            curve.id,
            if same_sense { ".T." } else { ".F." }
        ));
        out.edge_ends.insert(id, (start, end));
        if curve.collapsed {
            out.collapsed_edges.insert(id);
        }
        edge_ids.insert(edge, id);
        Ok(id)
    }
}

/// Where a boundary piece starts and ends, walked as the boundary walks it.
fn walked_ends((curve, reversed): &(CurveRef, bool)) -> (Vec3, Vec3) {
    if *reversed {
        (curve.end, curve.start)
    } else {
        (curve.start, curve.end)
    }
}

/// The widest step from one piece of a closed boundary to the next.
fn widest_joint(boundary: &[(CurveRef, bool)]) -> f64 {
    (0..boundary.len())
        .map(|index| walked_ends(&boundary[index]).1.sub(walked_ends(&boundary[(index + 1) % boundary.len()]).0).length())
        .fold(0.0, f64::max)
}

/// Turn round the pieces of a closed boundary that the file states against
/// the walk. A composite curve (102) lists members each starting where the
/// last ends, but some writers state a member end to start: walked as
/// written, the boundary steps across that member's length at both of its
/// ends. Each piece is turned to the sense whose ends meet its neighbours'
/// (whichever way those run), and the turned boundary is kept only when every
/// joint then closes within `bar` (two statements of one point); a boundary
/// that already closes, or that no turning closes, stays as the file states it.
/// `BREP_IGES_MEMBER_TURN` (default on; `=0` walks every member as stated).
fn orient_members(boundary: &mut [(CurveRef, bool)], bar: f64) {
    let count = boundary.len();
    if count < 2 || widest_joint(boundary) <= bar || !crate::switch!("BREP_IGES_MEMBER_TURN", true) {
        return;
    }
    let nearest = |point: Vec3, piece: &(CurveRef, bool)| {
        point.sub(piece.0.start).length().min(point.sub(piece.0.end).length())
    };
    let mut turned = boundary.to_vec();
    if count == 2 {
        // Two pieces are each other's neighbour at both ends: the first sets
        // the walk and the second follows it.
        let leaves = walked_ends(&turned[0]).1;
        let (start, end) = walked_ends(&turned[1]);
        if leaves.sub(end).length() < leaves.sub(start).length() {
            turned[1].1 = !turned[1].1;
        }
    } else {
        for index in 0..count {
            let (before, after) = (&boundary[(index + count - 1) % count], &boundary[(index + 1) % count]);
            let (start, end) = walked_ends(&boundary[index]);
            let as_walked = nearest(start, before) + nearest(end, after);
            let against = nearest(end, before) + nearest(start, after);
            if against < as_walked {
                turned[index].1 = !turned[index].1;
            }
        }
    }
    if widest_joint(&turned) <= bar {
        boundary.clone_from_slice(&turned);
    }
}

impl Translator<'_> {
    /// Remove the specks of spikes from a closed boundary: a piece no longer
    /// than `bar` followed by one that walks straight back along it bounds
    /// nothing (a sliver of no width and no length) and would be one edge
    /// walked twice the same way round the face. A longer out-and-back pair is
    /// a seam's two sides on a periodic surface and stays. Pieces are removed
    /// in such pairs while more than two remain. `BREP_IGES_DROP_SPIKES`
    /// (default on; `=0` keeps them).
    fn drop_spikes(&self, boundary: &mut Vec<(CurveRef, bool)>, bar: f64) {
        if !crate::switch!("BREP_IGES_DROP_SPIKES", true) {
            return;
        }
        let mut index = 0;
        while boundary.len() > 2 && index < boundary.len() {
            let next = (index + 1) % boundary.len();
            let (out, back) = (&boundary[index], &boundary[next]);
            let returns = walked_ends(out).0.sub(walked_ends(back).1).length() <= bar
                && walked_ends(out).1.sub(walked_ends(back).0).length() <= bar;
            let speck = |piece: &CurveRef| {
                let mut points = vec![piece.start, piece.end];
                points.extend(piece.probes.into_iter().flatten());
                points.iter().all(|p| points.iter().all(|q| p.sub(*q).length() <= bar))
            };
            if returns && speck(&out.0) && speck(&back.0) && self.same_locus(&out.0, &back.0, bar) {
                let (first, second) = (index.max(next), index.min(next));
                boundary.remove(first);
                boundary.remove(second);
                index = index.saturating_sub(1);
            } else {
                index += 1;
            }
        }
    }

    /// Whether two statements of a curve between the same ends are one locus:
    /// points at every 32nd of each (by its own parameter) lie within `bar`
    /// of the other, so a bulge between a few probes is not missed. (One
    /// narrower than a 32nd of a curve's parameter range can still pass.)
    fn same_locus(&self, first: &CurveRef, second: &CurveRef, bar: f64) -> bool {
        const POINTS: usize = 32;
        let on = |piece: &CurveRef, locus: &CurveRef| {
            let Ok(entity) = self.file.entity(piece.pointer) else { return false };
            let Ok(transform) = self.placed(entity, &piece.parent) else { return false };
            // The locus's polyline (its samples) says which stretch of it a
            // point is nearest; the search along the curve there answers. No
            // point is settled on the polyline alone (its sag is sampled, not
            // a bound).
            let Some((polyline, _)) = self.polyline(locus) else { return false };
            let (low, high) = piece.range;
            (1..POINTS).all(|i| {
                let Some(point) = self.curve_point(entity, &transform, low + (high - low) * i as f64 / POINTS as f64) else {
                    return false;
                };
                self.within_near_polyline(locus, &polyline, point, bar)
            })
        };
        on(first, second) && on(second, first)
    }

    /// Whether a point of the curve stands within `bar` of `point`: the search
    /// runs by golden section over the two segments either side of the
    /// curve's polyline vertex (from [`Self::polyline`]; its vertices are
    /// curve points) nearest `point`, and stops at the first curve point
    /// within `bar`, which settles it.
    fn within_near_polyline(&self, curve: &CurveRef, polyline: &[Vec3], point: Vec3, bar: f64) -> bool {
        let Some(segments) = polyline.len().checked_sub(1).filter(|&n| n > 0) else { return false };
        let (seed, nearest) = polyline
            .iter()
            .enumerate()
            .map(|(index, vertex)| (index, vertex.sub(point).length()))
            .fold((0, f64::INFINITY), |best, candidate| if candidate.1 < best.1 { candidate } else { best });
        if nearest <= bar {
            return true;
        }
        let Ok(entity) = self.file.entity(curve.pointer) else { return false };
        let Ok(transform) = self.placed(entity, &curve.parent) else { return false };
        let (low, high) = curve.range;
        let distance = |t: f64| self.curve_point(entity, &transform, low + (high - low) * t).map(|at| at.sub(point).length());
        let step = 1.0 / segments as f64;
        let (mut lo, mut hi) = (((seed as f64 - 1.0) * step).max(0.0), ((seed as f64 + 1.0) * step).min(1.0));
        let ratio = (5f64.sqrt() - 1.0) / 2.0;
        for _ in 0..40 {
            let (left, right) = (hi - ratio * (hi - lo), lo + ratio * (hi - lo));
            let (Some(dl), Some(dr)) = (distance(left), distance(right)) else { return false };
            if dl <= bar || dr <= bar {
                return true;
            }
            if dl < dr {
                hi = right;
            } else {
                lo = left;
            }
        }
        false
    }

    /// A boundary curve as a polyline through evenly spaced parameters, with
    /// a bound on how far the curve strays from it: twice the widest gap
    /// between a chord's middle and the curve's point half way across it.
    fn polyline(&self, curve: &CurveRef) -> Option<(Vec<Vec3>, f64)> {
        const SEGMENTS: usize = 32;
        let entity = self.file.entity(curve.pointer).ok()?;
        let transform = self.placed(entity, &curve.parent).ok()?;
        let (low, high) = curve.range;
        let at = |t: f64| self.curve_point(entity, &transform, low + (high - low) * t);
        let points: Vec<Vec3> = (0..=SEGMENTS).map(|i| at(i as f64 / SEGMENTS as f64)).collect::<Option<_>>()?;
        let mut sag = 0.0f64;
        for i in 0..SEGMENTS {
            let middle = at((i as f64 + 0.5) / SEGMENTS as f64)?;
            sag = sag.max(middle.sub(points[i].add(points[i + 1]).scale(0.5)).length());
        }
        Some((points, 2.0 * sag))
    }

    /// How far `point` stands from a boundary curve as the file states it,
    /// and the fraction along the piece (0 at its start and 1 at its end, by
    /// the curve's parameter) where it is nearest: the nearest of an even
    /// sampling, refined by golden section between the samples either side of
    /// it. `None` when the curve cannot be evaluated.
    fn nearest_on_curve(&self, curve: &CurveRef, point: Vec3) -> Option<(f64, f64)> {
        const SAMPLES: usize = 32;
        let entity = self.file.entity(curve.pointer).ok()?;
        let transform = self.placed(entity, &curve.parent).ok()?;
        let (low, high) = curve.range;
        let distance = |t: f64| self.curve_point(entity, &transform, low + (high - low) * t).map(|at| at.sub(point).length());
        let mut best = (0usize, f64::INFINITY);
        for index in 0..=SAMPLES {
            let d = distance(index as f64 / SAMPLES as f64)?;
            if d < best.1 {
                best = (index, d);
            }
        }
        let step = 1.0 / SAMPLES as f64;
        let (mut lo, mut hi) = (((best.0 as f64 - 1.0) * step).max(0.0), ((best.0 as f64 + 1.0) * step).min(1.0));
        let ratio = (5f64.sqrt() - 1.0) / 2.0;
        let mut nearest = (best.1, best.0 as f64 * step);
        for _ in 0..40 {
            let (left, right) = (hi - ratio * (hi - lo), lo + ratio * (hi - lo));
            let (dl, dr) = (distance(left)?, distance(right)?);
            if dl < nearest.0 {
                nearest = (dl, left);
            }
            if dr < nearest.0 {
                nearest = (dr, right);
            }
            if dl < dr {
                hi = right;
            } else {
                lo = left;
            }
        }
        Some(nearest)
    }

    /// A 144 or 143 as (surface pointer, its boundaries). Each boundary is its
    /// curves in order, each with whether it is walked against its own direction.
    #[allow(clippy::type_complexity)]
    fn face_boundaries(
        &self,
        entity: &ParsedEntity,
        parent: &Transform,
    ) -> Result<(i64, Vec<Vec<(CurveRef, bool)>>), String> {
        let mut boundaries = Vec::new();
        if entity.de.entity_type == 144 {
            let surface = entity.int(0)?;
            let has_outer = entity.int(1)? != 0;
            let inner = entity.int(2)?.max(0) as usize;
            let outer = entity.int(3)?;
            if !has_outer || outer == 0 {
                return Err("the outer boundary is the surface's whole domain (144 N1=0), which this import does not build".into());
            }
            boundaries.push(self.curve_on_surface(outer, parent)?);
            for index in 0..inner {
                boundaries.push(self.curve_on_surface(entity.int(4 + index)?, parent)?);
            }
            for boundary in &mut boundaries {
                orient_members(boundary, SAME_CURVE_RESOLUTIONS * self.resolution);
                self.drop_spikes(boundary, SAME_CURVE_RESOLUTIONS * self.resolution);
            }
            return Ok((surface, boundaries));
        }
        // 143: TYPE, SPTR, N, then N pointers to 141 boundaries.
        let surface = entity.int(1)?;
        let count = entity.int(2)?.max(0) as usize;
        for index in 0..count {
            let pointer = entity.int(3 + index)?;
            let boundary = self.file.entity(pointer)?;
            if boundary.de.entity_type != 141 {
                return Err(format!("DE {pointer} should be a boundary (141), is {}", boundary.de.entity_type));
            }
            let transform = self.placed(boundary, parent)?;
            let curves = boundary.int(3)?.max(0) as usize;
            let mut at = 4;
            let mut pieces = Vec::new();
            for _ in 0..curves {
                let curve = boundary.int(at)?;
                let reversed = boundary.int(at + 1)? == 2;
                let parameter_curves = boundary.int(at + 2)?.max(0) as usize;
                at += 3 + parameter_curves;
                let mut run = self.curve_chain(curve, &transform)?;
                if reversed {
                    run.reverse();
                    run.iter_mut().for_each(|(_, r)| *r = !*r);
                }
                pieces.extend(run);
            }
            orient_members(&mut pieces, SAME_CURVE_RESOLUTIONS * self.resolution);
            self.drop_spikes(&mut pieces, SAME_CURVE_RESOLUTIONS * self.resolution);
            boundaries.push(pieces);
        }
        Ok((surface, boundaries))
    }

    fn curve_on_surface(&self, pointer: i64, parent: &Transform) -> Result<Vec<(CurveRef, bool)>, String> {
        let entity = self.file.entity(pointer)?;
        if entity.de.entity_type != 142 {
            return Err(format!("DE {pointer} should be a curve on a surface (142), is {}", entity.de.entity_type));
        }
        let transform = self.placed(entity, parent)?;
        let model = entity.int(3)?;
        if model == 0 {
            return Err(format!("curve on surface DE {pointer} has no model-space curve"));
        }
        self.curve_chain(model, &transform)
    }

    /// A curve pointer as its ordered simple curves, with 102s flattened.
    fn curve_chain(&self, pointer: i64, parent: &Transform) -> Result<Vec<(CurveRef, bool)>, String> {
        let entity = self.file.entity(pointer)?;
        if entity.de.entity_type == 102 {
            let transform = self.placed(entity, parent)?;
            let count = entity.int(0)?.max(0) as usize;
            let mut chain = Vec::with_capacity(count);
            for index in 0..count {
                let member = entity.int(1 + index)?;
                if self.file.entity(member)?.de.entity_type == 116 {
                    continue; // A point member of a composite adds no boundary.
                }
                chain.extend(self.curve_chain(member, &transform)?);
            }
            return Ok(chain);
        }
        let transform = self.placed(entity, parent)?;
        Ok(vec![(self.curve_ref(entity, pointer, parent, &transform)?, false)])
    }

    fn curve_ref(
        &self,
        entity: &ParsedEntity,
        pointer: i64,
        parent: &Transform,
        transform: &Transform,
    ) -> Result<CurveRef, String> {
        let at = |t: f64| self.curve_point(entity, transform, t);
        if entity.de.entity_type == 130 {
            // An offset this reader refuses says why.
            self.offset_geometry(entity, transform).map_err(|error| format!("curve DE {pointer}: {error}"))?;
        }
        let start = at(0.0).ok_or_else(|| format!("curve DE {pointer} (entity {}) cannot be read as a boundary", entity.de.entity_type))?;
        let end = at(1.0).ok_or_else(|| format!("curve DE {pointer} cannot be read as a boundary"))?;
        let probes = at(0.25).zip(at(0.5)).map(|(a, b)| [a, b]);
        Ok(CurveRef { pointer, parent: *parent, start, end, probes, range: (0.0, 1.0) })
    }

    /// The point a fraction `t` along a curve, in mm.
    pub(super) fn curve_point(&self, entity: &ParsedEntity, transform: &Transform, t: f64) -> Option<Vec3> {
        let r = |i: usize| entity.real(i).ok();
        match entity.de.entity_type {
            110 => {
                let a = [r(0)?, r(1)?, r(2)?];
                let b = [r(3)?, r(4)?, r(5)?];
                Some(self.location(transform, [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]))
            }
            100 => {
                let (z, cx, cy, sx, sy, ex, ey) = (r(0)?, r(1)?, r(2)?, r(3)?, r(4)?, r(5)?, r(6)?);
                let radius = (sx - cx).hypot(sy - cy);
                let from = (sy - cy).atan2(sx - cx);
                let mut to = (ey - cy).atan2(ex - cx);
                if to <= from + 1e-12 {
                    to += std::f64::consts::TAU;
                }
                let angle = from + (to - from) * t;
                Some(self.location(transform, [cx + radius * angle.cos(), cy + radius * angle.sin(), z]))
            }
            104 => {
                if t == 0.0 {
                    return Some(self.location(transform, [r(7)?, r(8)?, r(6)?]));
                }
                if t == 1.0 {
                    return Some(self.location(transform, [r(9)?, r(10)?, r(6)?]));
                }
                self.ellipse_point(entity, transform, t)
            }
            126 => {
                let count = entity.int(0).ok()? as usize + 1;
                let degree = entity.int(1).ok()? as usize;
                let knots: Vec<f64> = (0..count + degree + 1).map(|i| r(6 + i)).collect::<Option<_>>()?;
                let base = 6 + knots.len();
                let tail = base + count + 3 * count;
                let (v0, v1) = (r(tail)?, r(tail + 1)?);
                let mut homogeneous = Vec::with_capacity(count);
                for i in 0..count {
                    let w = r(base + i)?;
                    let at = base + count + i * 3;
                    homogeneous.push([r(at)? * w, r(at + 1)? * w, r(at + 2)? * w, w]);
                }
                let h = de_boor(degree, &knots, &homogeneous, v0 + (v1 - v0) * t)?;
                (h[3] != 0.0).then(|| self.location(transform, [h[0] / h[3], h[1] / h[3], h[2] / h[3]]))
            }
            106 => self.polyline_point(entity, transform, t),
            112 => self.spline_point(entity, transform, t),
            130 => self.offset_point(entity, transform, t),
            _ => None,
        }
    }

    /// Which way a face's boundaries wind about its carrier, from the signed
    /// area they enclose in the carrier's own chart.
    fn winding(
        &self,
        entity: &ParsedEntity,
        parent: &Transform,
        chart: Option<Chart>,
        domain_v: Option<f64>,
        spline: Option<&crate::NurbsSurface>,
        boundaries: &[Vec<(CurveRef, bool)>],
    ) -> Option<bool> {
        let chart = chart?;
        let mut loops: Vec<Vec<(f64, f64)>> = Vec::with_capacity(boundaries.len());
        let parameter_curves = match chart {
            Chart::Parameter => self.parameter_curves(entity),
            _ => None,
        };
        if matches!(chart, Chart::Parameter) && parameter_curves.is_none() {
            // Model-space boundaries only, as many foreign files write them:
            // read them in the spline's own (u, v) by projection.
            loops = self.spline_loops(spline?, boundaries)?;
        } else if let Some(parameter_curves) = parameter_curves {
            loops = self.parameter_loops(&parameter_curves, parent)?;
        } else {
            for boundary in boundaries {
                let mut uv = Vec::new();
                for (curve, reversed) in boundary {
                    let member = self.file.entity(curve.pointer).ok()?;
                    let place = self.placed(member, &curve.parent).ok()?;
                    let mut run = Vec::new();
                    for i in 0..=WINDING_SAMPLES {
                        let p = self.curve_point(member, &place, i as f64 / WINDING_SAMPLES as f64)?;
                        run.extend(chart.read(p));
                    }
                    if *reversed {
                        run.reverse();
                    }
                    uv.extend(run);
                }
                loops.push(uv);
            }
        }
        signed_chart_area(&loops, chart.periodic(), domain_v).map(|area| area > 0.0)
    }

    /// The file's parameter-space boundary curves (the 142s' BPTR), sampled in
    /// the IGES surface's own (u, v), one loop per boundary.
    fn parameter_loops(&self, parameter_curves: &[i64], parent: &Transform) -> Option<Vec<Vec<(f64, f64)>>> {
        let mut loops = Vec::with_capacity(parameter_curves.len());
        for &pointer in parameter_curves {
            let mut uv = Vec::new();
            for (curve, reversed) in self.curve_chain(pointer, parent).ok()? {
                let member = self.file.entity(curve.pointer).ok()?;
                let place = self.placed(member, &Transform::IDENTITY).ok()?;
                let mut run = Vec::new();
                for i in 0..=WINDING_SAMPLES {
                    let p = self.curve_point(member, &place, i as f64 / WINDING_SAMPLES as f64)?;
                    run.push((p.x / self.scale, p.y / self.scale));
                }
                if reversed {
                    run.reverse();
                }
                uv.extend(run);
            }
            loops.push(uv);
        }
        Some(loops)
    }

    /// Which way a sphere face winds about the STEP carrier's normal, from the
    /// file's parameter-space curves: a 120's (u, v) is a rectangle (along the
    /// generatrix, then the turn), the poles and the seam are its edges, so the
    /// boundary as written closes in it and the standard's counter-clockwise
    /// outer boundary names the face, a whole sphere included. The 120's
    /// normal is its du x dv; `reversed` says the STEP carrier's is opposite.
    fn sphere_parameter_winding(&self, entity: &ParsedEntity, parent: &Transform, reversed: bool) -> Option<bool> {
        let loops = self.parameter_loops(&self.parameter_curves(entity)?, parent)?;
        let area = signed_chart_area(&loops, (false, false), None)?;
        Some((area > 0.0) != reversed)
    }

    /// Model-space boundaries read in a spline carrier's (u, v), each sample
    /// projected from its neighbour's foot so the walk stays on one branch.
    /// `None` when a sample is off the surface or a step jumps across half the
    /// domain (a closed spline's seam), where the area would not be the face's.
    fn spline_loops(
        &self,
        surface: &crate::NurbsSurface,
        boundaries: &[Vec<(CurveRef, bool)>],
    ) -> Option<Vec<Vec<(f64, f64)>>> {
        let [u0, u1] = surface.domain_u().ok()?;
        let [v0, v1] = surface.domain_v().ok()?;
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for h in surface.control_points.iter().flatten() {
            let p = h.point().ok()?;
            for (axis, value) in [p.x, p.y, p.z].into_iter().enumerate() {
                low[axis] = low[axis].min(value);
                high[axis] = high[axis].max(value);
            }
        }
        let size = Vec3::new(high[0] - low[0], high[1] - low[1], high[2] - low[2]).length();
        let off_surface = (1e-3 * size).max(10.0 * self.resolution);
        let mut loops = Vec::with_capacity(boundaries.len());
        for boundary in boundaries {
            let mut points = Vec::new();
            for (curve, reversed) in boundary {
                let member = self.file.entity(curve.pointer).ok()?;
                let place = self.placed(member, &curve.parent).ok()?;
                let mut run = Vec::with_capacity(WINDING_SAMPLES + 1);
                for i in 0..=WINDING_SAMPLES {
                    run.push(self.curve_point(member, &place, i as f64 / WINDING_SAMPLES as f64)?);
                }
                if *reversed {
                    run.reverse();
                }
                points.extend(run);
            }
            let mut uv: Vec<(f64, f64)> = Vec::with_capacity(points.len());
            for point in points {
                let foot = match uv.last() {
                    Some(&(u, v)) => crate::project_point_to_surface_seeded(surface, point, u, v).ok()?,
                    None => crate::project_point_to_surface(surface, point).ok()?,
                };
                if foot.distance > off_surface {
                    return None;
                }
                if let Some(&(u, v)) = uv.last() {
                    if (foot.u - u).abs() > 0.5 * (u1 - u0) || (foot.v - v).abs() > 0.5 * (v1 - v0) {
                        return None;
                    }
                }
                uv.push((foot.u, foot.v));
            }
            loops.push(uv);
        }
        Some(loops)
    }

    /// Which way a sphere face's boundaries wind about the sphere's outward
    /// normal, read around a point the face covers: projected stereographically
    /// from that point's antipode, the point is the origin and orientation seen
    /// from outside is kept, so the winding number around the origin answers it
    /// with no poles or seam involved. `None` if the point is outside them all.
    fn sphere_winding(
        &self,
        interior: Vec3,
        chart: Chart,
        boundaries: &[Vec<(CurveRef, bool)>],
    ) -> Option<bool> {
        let Chart::Sphere { origin, .. } = chart else {
            return None;
        };
        let q = interior.sub(origin);
        let radius = q.length();
        let n = q.normalized().ok()?;
        let x = perpendicular(n).ok()?;
        let y = n.cross(x);
        let mut winding = 0.0;
        for boundary in boundaries {
            let mut plane = Vec::new();
            for (curve, reversed) in boundary {
                let member = self.file.entity(curve.pointer).ok()?;
                let place = self.placed(member, &curve.parent).ok()?;
                let mut run = Vec::new();
                for i in 0..=WINDING_SAMPLES {
                    let p = self.curve_point(member, &place, i as f64 / WINDING_SAMPLES as f64)?.sub(origin);
                    let denominator = radius + p.dot(n);
                    if denominator.abs() > 1e-9 * radius {
                        run.push((p.dot(x) / denominator, p.dot(y) / denominator));
                    }
                }
                if *reversed {
                    run.reverse();
                }
                plane.extend(run);
            }
            for i in 0..plane.len() {
                let (ax, ay) = plane[i];
                let (bx, by) = plane[(i + 1) % plane.len()];
                winding += (ax * by - ay * bx).atan2(ax * bx + ay * by);
            }
        }
        let turns = (winding / std::f64::consts::TAU).round();
        (turns != 0.0).then_some(turns > 0.0)
    }

    /// The parameter-space curve of each boundary of a 144 or 143, when the
    /// file gives one for all of them.
    fn parameter_curves(&self, entity: &ParsedEntity) -> Option<Vec<i64>> {
        if entity.de.entity_type != 144 {
            return None;
        }
        let inner = entity.int(2).ok()?.max(0) as usize;
        let mut owners = vec![entity.int(3).ok()?];
        for index in 0..inner {
            owners.push(entity.int(4 + index).ok()?);
        }
        owners
            .into_iter()
            .map(|owner| self.file.entity(owner).ok()?.int(2).ok().filter(|p| *p != 0))
            .collect()
    }

    fn polyline_point(&self, entity: &ParsedEntity, transform: &Transform, t: f64) -> Option<Vec3> {
        let count = entity.int(1).ok()? as usize;
        if count < 2 {
            return None;
        }
        let point = |i: usize| -> Option<[f64; 3]> {
            match entity.de.form {
                1 | 11 => Some([entity.real(3 + i * 2).ok()?, entity.real(4 + i * 2).ok()?, entity.real(2).ok()?]),
                2 | 12 => Some([entity.real(2 + i * 3).ok()?, entity.real(3 + i * 3).ok()?, entity.real(4 + i * 3).ok()?]),
                _ => None,
            }
        };
        let along = t * (count - 1) as f64;
        let i = (along.floor() as usize).min(count - 2);
        let (a, b, f) = (point(i)?, point(i + 1)?, along - i as f64);
        Some(self.location(transform, [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f]))
    }

    fn spline_point(&self, entity: &ParsedEntity, transform: &Transform, t: f64) -> Option<Vec3> {
        let segments = entity.int(3).ok()? as usize;
        if segments == 0 {
            return None;
        }
        let knot = |i: usize| entity.real(4 + i).ok();
        let (first, last) = (knot(0)?, knot(segments)?);
        let value = first + (last - first) * t;
        let mut segment = 0;
        while segment + 1 < segments && value >= knot(segment + 1)? {
            segment += 1;
        }
        let s = value - knot(segment)?;
        let base = 4 + segments + 1 + segment * 12;
        let axis = |a: usize| -> Option<f64> {
            let c = |p: usize| entity.real(base + a * 4 + p).ok();
            Some(c(0)? + s * (c(1)? + s * (c(2)? + s * c(3)?)))
        };
        Some(self.location(transform, [axis(0)?, axis(1)?, axis(2)?]))
    }

    /// A point along a 104 ellipse arc, by the angle about its centre.
    fn ellipse_point(&self, entity: &ParsedEntity, transform: &Transform, t: f64) -> Option<Vec3> {
        if entity.de.form != 1 {
            return None;
        }
        let r = |i: usize| entity.real(i).ok();
        let (a, b, c, d, e, f) = (r(0)?, r(1)?, r(2)?, r(3)?, r(4)?, r(5)?);
        let z = r(6)?;
        let det = 4.0 * a * c - b * b;
        if det == 0.0 {
            return None;
        }
        let cx = (b * e - 2.0 * c * d) / det;
        let cy = (b * d - 2.0 * a * e) / det;
        let k = -(f + 0.5 * (d * cx + e * cy));
        let from = (r(8)? - cy).atan2(r(7)? - cx);
        let mut to = (r(10)? - cy).atan2(r(9)? - cx);
        if to <= from + 1e-12 {
            to += std::f64::consts::TAU;
        }
        let angle = from + (to - from) * t;
        let (u, v) = (angle.cos(), angle.sin());
        let radius = (k / (a * u * u + b * u * v + c * v * v)).sqrt();
        radius.is_finite().then(|| self.location(transform, [cx + radius * u, cy + radius * v, z]))
    }
}

/// Samples per boundary curve when measuring winding.
const WINDING_SAMPLES: usize = 16;

/// The signed area loops enclose in a chart. Angles are unwrapped along each
/// loop, so a loop that goes around a periodic direction still counts. Of the
/// two Green's integrals the larger is used: a band around one period has area
/// only in the integral along that period. A boundary with net winding around
/// a period does not say which side it encloses (the cap above a circle on a
/// sphere, or the rest below it). `domain_v`, a v the surface actually covers,
/// settles it: the enclosed side is the side that holds it. Without one there
/// is no answer.
fn signed_chart_area(
    loops: &[Vec<(f64, f64)>],
    (wrap_u, wrap_v): (bool, bool),
    domain_v: Option<f64>,
) -> Option<f64> {
    let step = |from: f64, to: f64, wrap: bool| {
        let mut delta = to - from;
        if wrap {
            delta -= std::f64::consts::TAU * (delta / std::f64::consts::TAU).round();
        }
        delta
    };
    let (mut u_dv, mut v_du) = (0.0, 0.0);
    let (mut net_u, mut net_v) = (0.0, 0.0);
    for points in loops {
        if points.len() < 2 {
            continue;
        }
        let (mut u, mut v) = points[0];
        for i in 1..=points.len() {
            let (next_u, next_v) = points[i % points.len()];
            let (du, dv) = (step(u, next_u, wrap_u), step(v, next_v, wrap_v));
            u_dv += (u + 0.5 * du) * dv;
            v_du -= (v + 0.5 * dv) * du;
            u += du;
            v += dv;
            net_u += du;
            net_v += dv;
        }
    }
    if net_v.abs() > std::f64::consts::PI {
        return None;
    }
    if net_u.abs() > std::f64::consts::PI {
        let inside = domain_v?;
        let count = loops.iter().map(Vec::len).sum::<usize>().max(1) as f64;
        let boundary = loops.iter().flatten().map(|(_, v)| v).sum::<f64>() / count;
        // Going up in u, the enclosed side (the left) is the side of greater v.
        return Some(if (net_u > 0.0) == (inside > boundary) { 1.0 } else { -1.0 });
    }
    let area = if u_dv.abs() >= v_du.abs() { u_dv } else { v_du };
    (area != 0.0 && area.is_finite()).then_some(area)
}

/// A homogeneous B-spline point by de Boor's algorithm, on the file's own knot
/// vector, so unclamped and periodic curves need no conversion to be sampled.
fn de_boor(degree: usize, knots: &[f64], points: &[[f64; 4]], t: f64) -> Option<[f64; 4]> {
    if points.is_empty() || knots.len() != points.len() + degree + 1 {
        return None;
    }
    let last = points.len() - 1;
    let t = t.clamp(knots[degree], knots[last + 1]);
    let mut span = degree;
    while span < last && t >= knots[span + 1] {
        span += 1;
    }
    let mut work: Vec<[f64; 4]> = (0..=degree).map(|j| points[j + span - degree]).collect();
    for r in 1..=degree {
        for j in (r..=degree).rev() {
            let i = j + span - degree;
            let denominator = knots[i + degree - r + 1] - knots[i];
            let alpha = if denominator == 0.0 { 0.0 } else { (t - knots[i]) / denominator };
            for axis in 0..4 {
                work[j][axis] = (1.0 - alpha) * work[j - 1][axis] + alpha * work[j][axis];
            }
        }
    }
    Some(work[degree])
}
