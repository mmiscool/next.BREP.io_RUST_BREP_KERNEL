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
}

struct SoupEdge {
    curve: CurveRef,
    start: usize,
    end: usize,
    uses: Vec<(usize, bool)>,
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
                let edge = match_edge(&mut edges, &mut by_ends, curve, a, b, resolution);
                let along = (a == edges[edge.0].start && b == edges[edge.0].end && edge.1) != reversed;
                edges[edge.0].uses.push((face_index, along));
                coedges.push((edge.0, along));
            }
            loops.push(coedges);
        }
        faces.push(SoupFace { pointer, surface, parent, loops, counter_clockwise });
    }
    let (flips, bodies) = orient(&faces, &edges)?;
    Ok(FaceSets { faces, edges, vertices: welder.points, flips, bodies, resolution })
}

/// Find the edge this statement of a curve belongs to, or start a new one.
/// The second value says the curve runs the way the stored edge does.
fn match_edge(
    edges: &mut Vec<SoupEdge>,
    by_ends: &mut HashMap<(usize, usize), Vec<usize>>,
    curve: CurveRef,
    a: usize,
    b: usize,
    resolution: f64,
) -> (usize, bool) {
    let key = (a.min(b), a.max(b));
    let bar = (SAME_CURVE_RESOLUTIONS * resolution).max(SAME_CURVE_FRACTION * curve.start.sub(curve.end).length());
    for &index in by_ends.get(&key).map_or(&[][..], Vec::as_slice) {
        let other = &edges[index].curve;
        let (Some(mine), Some(theirs)) = (curve.probes, other.probes) else {
            // A conic with no probes: its ends are all there is to go on.
            return (index, a == edges[index].start);
        };
        if a != b {
            if mine[1].sub(theirs[1]).length() <= bar {
                return (index, a == edges[index].start);
            }
        } else {
            let closed_bar = bar.max(SAME_CURVE_FRACTION * mine[1].sub(curve.start).length());
            if mine[0].sub(theirs[0]).length() <= closed_bar && mine[1].sub(theirs[1]).length() <= closed_bar {
                return (index, true);
            }
            // The other way round a closed curve: a quarter of the way along
            // one is three quarters along the other, and the half is shared.
            if mine[1].sub(theirs[1]).length() <= closed_bar {
                return (index, false);
            }
        }
    }
    let index = edges.len();
    edges.push(SoupEdge { curve, start: a, end: b, uses: Vec::new() });
    by_ends.entry(key).or_default().push(index);
    (index, true)
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
        let shell = self.shell(&mut out, outer, flip, &mut vertex_ids, &mut edge_ids)?;
        if voids.is_empty() {
            out.add(format!("MANIFOLD_SOLID_BREP('',#{shell})"));
        } else {
            let mut cavities = Vec::with_capacity(voids.len());
            for (faces, void_flip) in voids {
                let void_shell = self.shell(&mut out, faces, *void_flip, &mut vertex_ids, &mut edge_ids)?;
                cavities.push(format!("#{}", out.add(format!("ORIENTED_CLOSED_SHELL('',*,#{void_shell},.T.)"))));
            }
            out.add(format!("BREP_WITH_VOIDS('',#{shell},({}))", cavities.join(",")));
        }
        Ok(out.finish(&format!("sets{}", outer.first().copied().unwrap_or(0))))
    }

    fn shell(
        &self,
        out: &mut Translator,
        body: &[usize],
        flip: bool,
        vertex_ids: &mut HashMap<usize, usize>,
        edge_ids: &mut HashMap<usize, usize>,
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
        Ok(out.add(format!("CLOSED_SHELL('',({}))", face_ids.join(","))))
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
        let same_sense = curve.start.sub(from).length() + curve.end.sub(to).length()
            <= curve.start.sub(to).length() + curve.end.sub(from).length();
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

impl Translator<'_> {
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
        Ok(CurveRef { pointer, parent: *parent, start, end, probes })
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
