//! Lane O row 32 (`BREP_OFFSET_PLANAR_SHELL`, default on since batch 14; `0`/`off`
//! restores the pipeline): the outward shell
//! of a convex polyhedron, built directly.
//!
//! The pipeline grows every retained face, imprints the grown carriers on the
//! source and assembles the fragments. For a convex source whose faces are all
//! planar, that result has a closed form: the grown body is the intersection
//! of the face half-spaces, a retained face's plane moved out by |distance|
//! and an opening's plane left where it is, and the shell is the grown body
//! less the source. Its faces are
//! - each retained face's skin, the source face turned inside out (`Source`);
//! - each retained face's grown image, on its moved plane (`Offset`);
//! - each opening's wall, the grown body's face on the opening's plane less
//!   the source face (`Wall`).
//!
//! With every source vertex on exactly three faces, the three moved planes
//! meet in one point, the vertex's grown image g(v), and the grown body has
//! the source's face lattice. The lane checks that lattice instead of assuming
//! it: every image must lie inside every moved half-space, and every grown
//! face must stay a convex polygon with the source face's turning. A wall
//! touches the source only along an edge it shares with another opening; that
//! edge's run is replaced by the connectors [g(v), v] at its ends, so walls
//! come out as annuli (no opening neighbour) or as single loops.
//!
//! Anything outside that (a curved or holed face, a reflex or flat edge, a
//! vertex on four faces, an inward or tiny distance, a lattice that moves)
//! declines to the pipeline, and so does any failure after recognition: the
//! lane never answers with a refusal the pipeline would not give.
use super::*;
use crate::{make_line, AnalyticSurface, NurbsSurface, Vec4};

/// Relative linear bar: coincidence, plane membership and straightness at
/// `LINEAR_BAR · scale`, the same bar as the revolved-wedge lane.
const LINEAR_BAR: f64 = 1e-9;

/// The switch, default on: `0`/`off` sends every shell to the pipeline.
fn planar_shell_on() -> bool {
    switch!("BREP_OFFSET_PLANAR_SHELL", true)
}


/// The lane's answer, or `None` when it declines (or the switch is off) and
/// the pipeline takes the shell.
pub(super) fn planar_shell(
    source: &BrepSolid,
    opening_face_ids: &[u64],
    distance: f64,
) -> Option<OffsetShellResultRecord> {
    if !planar_shell_on() {
        return None;
    }
    let polyhedron = match Polyhedron::read(source, opening_face_ids, distance) {
        Ok(polyhedron) => polyhedron,
        Err(reason) => {
            os_debug!("planar shell lane declined: {reason}");
            return None;
        }
    };
    match polyhedron.build() {
        Ok(result) => {
            // The thread's name is the test's under libtest, so a debug run of
            // the suites says which of them reach the lane.
            os_debug!(
                "planar shell lane built {} faces (thread {})",
                result.face_images.len(),
                std::thread::current().name().unwrap_or("main")
            );
            Some(result)
        }
        Err(reason) => {
            os_debug!("planar shell lane declined after recognition: {reason}");
            None
        }
    }
}

/// A source face as the lane reads it: its outward plane and its vertex cycle,
/// counter-clockwise about the outward normal.
struct PlanarFace {
    id: u64,
    name: Option<String>,
    normal: Vec3,
    /// `normal · x = offset` on the face's plane.
    offset: f64,
    /// How far the plane moves: |distance| for a retained face, 0 for an opening.
    shift: f64,
    opening: bool,
    cycle: Vec<usize>,
}

/// A vertex of the shell: a source vertex or the grown image of one.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum Key {
    Source(usize),
    Grown(usize),
}

struct Polyhedron {
    points: Vec<Vec3>,
    grown: Vec<Vec3>,
    faces: Vec<PlanarFace>,
    /// The face across each directed cycle edge (a, b).
    across: HashMap<(usize, usize), usize>,
    linear: f64,
}

/// One face of the result: outward normal, loops (outer first), provenance.
struct ShellFace {
    normal: Vec3,
    loops: Vec<Vec<Key>>,
    role: OffsetFaceRole,
    source_face: usize,
}

impl Polyhedron {
    fn read(source: &BrepSolid, opening_face_ids: &[u64], distance: f64) -> Result<Self, String> {
        if !(distance < -MINIMUM_DISTANCE) || !distance.is_finite() {
            return Err("not a finite outward distance".into());
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
        if shell.faces.iter().all(|face| opening_face_ids.contains(&face.id)) {
            return Err("every face is an opening".into());
        }
        let linear = LINEAR_BAR * crate::solid_scale(source).max(1.0);
        let vertex_index = source
            .vertices
            .iter()
            .enumerate()
            .map(|(index, vertex)| (vertex.id, index))
            .collect::<HashMap<_, _>>();
        let edges = source.edges.iter().map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
        let points = source.vertices.iter().map(|vertex| vertex.point).collect::<Vec<_>>();
        let mut faces = Vec::with_capacity(shell.faces.len());
        for face in &shell.faces {
            let Some(AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) = face.surface.analytic() else {
                return Err(format!("face {} is not a plane", face.id));
            };
            let mut normal = u_dir.cross(*v_dir).normalized()?;
            if !face.same_sense {
                normal = normal.scale(-1.0);
            }
            let [face_loop] = &face.loops[..] else {
                return Err(format!("face {} has {} loops, not one", face.id, face.loops.len()));
            };
            let mut cycle = Vec::with_capacity(face_loop.coedges.len());
            let mut previous_end = None::<usize>;
            for coedge in &face_loop.coedges {
                let edge = edges.get(&coedge.edge_id).ok_or("a coedge names a missing edge")?;
                if edge.degenerate {
                    return Err(format!("face {} has a degenerate edge", face.id));
                }
                let (start, end) = if coedge.forward {
                    (edge.start_vertex_id, edge.end_vertex_id)
                } else {
                    (edge.end_vertex_id, edge.start_vertex_id)
                };
                let start = *vertex_index.get(&start).ok_or("an edge names a missing vertex")?;
                let end = *vertex_index.get(&end).ok_or("an edge names a missing vertex")?;
                // The edge must be the segment between its vertices.
                let (from, to) = edge
                    .curve
                    .straight_segment(linear)
                    .ok_or_else(|| format!("edge {} is not straight", edge.id))?;
                let chord = to.sub(from);
                for point in [points[start], points[end]] {
                    let along = point.sub(from);
                    let axial = along.dot(chord) / chord.length_squared();
                    if along.sub(chord.scale(axial)).length() > linear || !(-1e-9..=1.0 + 1e-9).contains(&axial) {
                        return Err(format!("edge {} does not run between its vertices", edge.id));
                    }
                }
                if previous_end.is_some_and(|previous| previous != start) {
                    return Err(format!("face {}'s loop does not close up", face.id));
                }
                previous_end = Some(end);
                cycle.push(start);
            }
            if previous_end != cycle.first().copied() {
                return Err(format!("face {}'s loop does not close up", face.id));
            }
            if cycle.len() < 3 {
                return Err(format!("face {} has fewer than three vertices", face.id));
            }
            // Counter-clockwise about the outward normal, whatever the loop's own sense.
            if newell(cycle.iter().map(|&index| points[index])).dot(normal) < 0.0 {
                cycle.reverse();
            }
            let offset = normal.dot(*origin);
            let opening = opening_face_ids.contains(&face.id);
            faces.push(PlanarFace {
                id: face.id,
                name: face.name.clone(),
                normal,
                offset,
                shift: if opening { 0.0 } else { -distance },
                opening,
                cycle,
            });
        }
        // Each directed edge on exactly one face, its reverse on exactly one other.
        let mut across_of = HashMap::<(usize, usize), usize>::default();
        for (index, face) in faces.iter().enumerate() {
            for (a, b) in cycle_edges(&face.cycle) {
                if across_of.insert((a, b), index).is_some() {
                    return Err("a directed edge is used twice".into());
                }
            }
        }
        let mut across = HashMap::<(usize, usize), usize>::default();
        for (&(a, b), _) in &across_of {
            let other = *across_of.get(&(b, a)).ok_or("an edge has one face")?;
            across.insert((a, b), other);
        }
        // Every vertex on exactly three faces.
        let mut incident = vec![Vec::<usize>::new(); points.len()];
        for (index, face) in faces.iter().enumerate() {
            for &vertex in &face.cycle {
                incident[vertex].push(index);
            }
        }
        let used = incident.iter().enumerate().filter(|(_, list)| !list.is_empty()).collect::<Vec<_>>();
        if used.iter().any(|(_, list)| list.len() != 3) {
            return Err("a vertex is not on exactly three faces".into());
        }
        // Convex, strictly: every vertex on a face's plane, every other vertex
        // well behind it (a flat or reflex edge puts one on or in front of it).
        for (index, face) in faces.iter().enumerate() {
            for &(vertex, ref list) in &used {
                let height = face.normal.dot(points[vertex]) - face.offset;
                if list.contains(&index) {
                    if height.abs() > linear {
                        return Err(format!("a vertex of face {} is off its plane", face.id));
                    }
                } else if height > -1e3 * linear {
                    return Err(format!("the source is not strictly convex at face {}", face.id));
                }
            }
        }
        // Grown images: the three moved planes' meeting point, as the source
        // vertex plus the displacement that moves each plane by its shift.
        let mut grown = points.clone();
        for &(vertex, ref list) in &used {
            let [f1, f2, f3] = [&faces[list[0]], &faces[list[1]], &faces[list[2]]];
            let (n1, n2, n3) = (f1.normal, f2.normal, f3.normal);
            let determinant = n1.dot(n2.cross(n3));
            if determinant.abs() < 1e-6 {
                return Err("three planes at a vertex are nearly dependent".into());
            }
            let displacement = n2
                .cross(n3)
                .scale(f1.shift)
                .add(n3.cross(n1).scale(f2.shift))
                .add(n1.cross(n2).scale(f3.shift))
                .scale(1.0 / determinant);
            grown[vertex] = points[vertex].add(displacement);
        }
        let polyhedron = Self { points, grown, faces, across, linear };
        polyhedron.check_lattice(&used.iter().map(|(vertex, _)| *vertex).collect::<Vec<_>>())?;
        Ok(polyhedron)
    }

    /// The grown body keeps the source's face lattice: every image inside
    /// every moved half-space, every grown face a convex polygon turning the
    /// same way as its source face with no edge collapsed.
    fn check_lattice(&self, used: &[usize]) -> Result<(), String> {
        for face in &self.faces {
            for &vertex in used {
                if face.normal.dot(self.grown[vertex]) - face.offset - face.shift > self.linear {
                    return Err(format!("a grown vertex lies outside face {}'s moved plane", face.id));
                }
            }
            let cycle = face.cycle.iter().map(|&vertex| self.grown[vertex]).collect::<Vec<_>>();
            for (index, point) in cycle.iter().enumerate() {
                let next = cycle[(index + 1) % cycle.len()];
                let after = cycle[(index + 2) % cycle.len()];
                if next.sub(*point).length() <= 1e3 * self.linear {
                    return Err(format!("a grown edge of face {} collapses", face.id));
                }
                if next.sub(*point).cross(after.sub(next)).dot(face.normal) <= 0.0 {
                    return Err(format!("grown face {} is not convex the source's way", face.id));
                }
            }
        }
        Ok(())
    }

    fn point(&self, key: Key) -> Vec3 {
        match key {
            Key::Source(index) => self.points[index],
            Key::Grown(index) => self.grown[index],
        }
    }

    /// Skins, grown faces and walls, in that order, each in source face order.
    fn shell_faces(&self) -> Result<Vec<ShellFace>, String> {
        let mut result = Vec::new();
        for (index, face) in self.faces.iter().enumerate().filter(|(_, face)| !face.opening) {
            result.push(ShellFace {
                normal: face.normal.scale(-1.0),
                loops: vec![face.cycle.iter().rev().map(|&vertex| Key::Source(vertex)).collect()],
                role: OffsetFaceRole::Source,
                source_face: index,
            });
        }
        for (index, face) in self.faces.iter().enumerate().filter(|(_, face)| !face.opening) {
            result.push(ShellFace {
                normal: face.normal,
                loops: vec![face.cycle.iter().map(|&vertex| Key::Grown(vertex)).collect()],
                role: OffsetFaceRole::Offset,
                source_face: index,
            });
        }
        for (index, face) in self.faces.iter().enumerate().filter(|(_, face)| face.opening) {
            for loops in self.walls(face)? {
                result.push(ShellFace { normal: face.normal, loops, role: OffsetFaceRole::Wall, source_face: index });
            }
        }
        Ok(result)
    }

    /// The opening's wall faces: the grown polygon less the source polygon.
    /// Along a retained neighbour the boundary is the grown edge forward and
    /// the source edge backward; along an opening neighbour both coincide and
    /// drop out, and connectors [g(v), v] join the ends of each such run.
    fn walls(&self, face: &PlanarFace) -> Result<Vec<Vec<Vec<Key>>>, String> {
        let edges = cycle_edges(&face.cycle).collect::<Vec<_>>();
        let retained = |a: usize, b: usize| !self.faces[self.across[&(a, b)]].opening;
        if !edges.iter().any(|&(a, b)| retained(a, b)) {
            return Ok(Vec::new());
        }
        if edges.iter().all(|&(a, b)| retained(a, b)) {
            let outer = face.cycle.iter().map(|&vertex| Key::Grown(vertex)).collect();
            let hole = face.cycle.iter().rev().map(|&vertex| Key::Source(vertex)).collect();
            return Ok(vec![vec![outer, hole]]);
        }
        let mut next = HashMap::<Key, Key>::default();
        let mut link = |from: Key, to: Key| -> Result<(), String> {
            if next.insert(from, to).is_some() {
                return Err("a wall boundary branches".into());
            }
            Ok(())
        };
        let count = edges.len();
        for (position, &(a, b)) in edges.iter().enumerate() {
            if retained(a, b) {
                link(Key::Grown(a), Key::Grown(b))?;
                link(Key::Source(b), Key::Source(a))?;
            }
            // At b, where the run along the cycle changes kind, a connector.
            let (c, d) = edges[(position + 1) % count];
            debug_assert_eq!(b, c);
            match (retained(a, b), retained(c, d)) {
                (true, false) => link(Key::Grown(b), Key::Source(b))?,
                (false, true) => link(Key::Source(b), Key::Grown(b))?,
                _ => {}
            }
        }
        let mut walls = Vec::new();
        let mut seen = HashSet::<Key>::default();
        let mut starts = next.keys().copied().collect::<Vec<_>>();
        starts.sort_by_key(|key| match key {
            Key::Grown(index) => (0, *index),
            Key::Source(index) => (1, *index),
        });
        for start in starts {
            if seen.contains(&start) {
                continue;
            }
            let mut wall_loop = Vec::new();
            let mut key = start;
            loop {
                if !seen.insert(key) {
                    return Err("a wall boundary revisits a vertex".into());
                }
                wall_loop.push(key);
                key = *next.get(&key).ok_or("a wall boundary stops short")?;
                if key == start {
                    break;
                }
            }
            if newell(wall_loop.iter().map(|&key| self.point(key))).dot(face.normal) <= 0.0 {
                return Err("a wall loop turns the wrong way".into());
            }
            walls.push(vec![wall_loop]);
        }
        Ok(walls)
    }

    fn build(&self) -> Result<OffsetShellResultRecord, String> {
        let shell_faces = self.shell_faces()?;
        let mut vertex_ids = HashMap::<Key, u64>::default();
        let mut vertices = Vec::new();
        let mut edge_index = HashMap::<(Key, Key), usize>::default();
        let mut edges = Vec::<EdgeRecord>::new();
        let mut faces = Vec::with_capacity(shell_faces.len());
        let mut face_images = Vec::with_capacity(shell_faces.len());
        let mut next_id = 1_u64;
        let mut used_names = HashSet::<String>::default();
        for shell_face in &shell_faces {
            for key in shell_face.loops.iter().flatten() {
                vertex_ids.entry(*key).or_insert_with(|| {
                    vertices.push(VertexRecord { id: next_id, point: self.point(*key) });
                    next_id += 1;
                    next_id - 1
                });
            }
        }
        for shell_face in &shell_faces {
            let normal = shell_face.normal;
            let outer = &shell_face.loops[0];
            let first = self.point(outer[1]).sub(self.point(outer[0]));
            let u_dir = first.sub(normal.scale(normal.dot(first))).normalized()?;
            let v_dir = normal.cross(u_dir);
            let anchor = self.point(outer[0]);
            let local = |point: Vec3| {
                let offset = point.sub(anchor);
                (offset.dot(u_dir), offset.dot(v_dir))
            };
            let (mut u_min, mut u_max, mut v_min, mut v_max) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for key in shell_face.loops.iter().flatten() {
                let (u, v) = local(self.point(*key));
                u_min = u_min.min(u);
                u_max = u_max.max(u);
                v_min = v_min.min(v);
                v_max = v_max.max(v);
            }
            let (u_length, v_length) = (u_max - u_min, v_max - v_min);
            let corner = anchor.add(u_dir.scale(u_min)).add(v_dir.scale(v_min));
            let surface = NurbsSurface::new(
                1,
                1,
                vec![0.0, 0.0, u_length, u_length],
                vec![0.0, 0.0, v_length, v_length],
                vec![
                    vec![Vec4::from_point(corner, 1.0), Vec4::from_point(corner.add(v_dir.scale(v_length)), 1.0)],
                    vec![
                        Vec4::from_point(corner.add(u_dir.scale(u_length)), 1.0),
                        Vec4::from_point(corner.add(u_dir.scale(u_length)).add(v_dir.scale(v_length)), 1.0),
                    ],
                ],
            )?;
            let uv = |key: Key| {
                let (u, v) = local(self.point(key));
                Vec3::new(u - u_min, v - v_min, 0.0)
            };
            let mut loops = Vec::with_capacity(shell_face.loops.len());
            for keys in &shell_face.loops {
                let mut coedges = Vec::with_capacity(keys.len());
                for (index, &from) in keys.iter().enumerate() {
                    let to = keys[(index + 1) % keys.len()];
                    let (first, second) = if vertex_ids[&from] < vertex_ids[&to] { (from, to) } else { (to, from) };
                    let position = match edge_index.get(&(first, second)) {
                        Some(&position) => position,
                        None => {
                            edges.push(EdgeRecord {
                                id: next_id,
                                curve: make_line(self.point(first), self.point(second))?,
                                t0: 0.0,
                                t1: 1.0,
                                start_vertex_id: vertex_ids[&first],
                                end_vertex_id: vertex_ids[&second],
                                degenerate: false,
                                name: None,
                            });
                            next_id += 1;
                            edge_index.insert((first, second), edges.len() - 1);
                            edges.len() - 1
                        }
                    };
                    coedges.push(CoedgeRecord {
                        id: next_id,
                        edge_id: edges[position].id,
                        forward: first == from,
                        pcurve: make_line(uv(from), uv(to))?,
                    });
                    next_id += 1;
                }
                loops.push(LoopRecord { id: next_id, coedges });
                next_id += 1;
            }
            let source_face = &self.faces[shell_face.source_face];
            faces.push(FaceRecord {
                id: next_id,
                surface,
                same_sense: true,
                loops,
                name: pipeline_name(source_face.name.as_deref(), shell_face.role, &mut used_names),
            });
            next_id += 1;
            face_images.push(OffsetShellFaceImageRecord { role: shell_face.role, source_face_id: source_face.id });
        }
        // One shell: V − E + F − H = 2 − 2g, H the inner loops (an opposed
        // pair of openings makes a tube, genus 1).
        let holes = faces.iter().map(|face| face.loops.len() - 1).sum::<usize>();
        let euler = vertices.len() as i64 - edges.len() as i64 + faces.len() as i64 - holes as i64;
        if euler > 2 || (2 - euler) % 2 != 0 {
            return Err(format!("the shell's Euler characteristic is {euler}"));
        }
        let genus = (2 - euler) / 2;
        let solid = BrepSolid {
            mass_properties_cache: Default::default(),
            cells: Default::default(),
            id: next_id + 1,
            vertices,
            edges,
            shells: vec![ShellRecord { id: next_id, faces }],
            genus: genus as _,
            kind: crate::BodyKind::Solid,
        };
        let solid = crate::accept_sound(solid, "offsetShell").map_err(|refusal| refusal.to_string())?;
        Ok(OffsetShellResultRecord { solid, face_images })
    }
}

/// The name the pipeline's assembly gives a face: a skin keeps its source
/// face's name, and a grown face and a wall append `_Offset` (the pipeline's
/// wall on a planar opening is that face's carrier at distance 0, which
/// `offset.rs` names `{name}_Offset`); a repeat takes the first free `_1`, `_2`, ….
fn pipeline_name(base: Option<&str>, role: OffsetFaceRole, used: &mut HashSet<String>) -> Option<String> {
    let base = base?;
    let base = if role == OffsetFaceRole::Source { base.to_string() } else { format!("{base}_Offset") };
    if used.insert(base.clone()) {
        return Some(base);
    }
    (1usize..)
        .map(|suffix| format!("{base}_{suffix}"))
        .find(|candidate| used.insert(candidate.clone()))
}

fn cycle_edges(cycle: &[usize]) -> impl Iterator<Item = (usize, usize)> + '_ {
    (0..cycle.len()).map(move |index| (cycle[index], cycle[(index + 1) % cycle.len()]))
}

/// Newell's area vector of a closed polygon.
fn newell(points: impl Iterator<Item = Vec3>) -> Vec3 {
    let points = points.collect::<Vec<_>>();
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    for (index, point) in points.iter().enumerate() {
        sum = sum.add(point.cross(points[(index + 1) % points.len()]));
    }
    sum.scale(0.5)
}
