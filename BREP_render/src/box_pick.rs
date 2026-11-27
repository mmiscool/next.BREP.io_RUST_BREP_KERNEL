//! BOX selection (multi-select rework, owner 2026-11-26): the entities a
//! screen rectangle takes, by the common CAD convention —
//!
//! * WINDOW (dragged left to right): only entities lying FULLY inside the box;
//! * CROSSING (dragged right to left): anything the box touches.
//!
//! Visible entities only, by default: an entity is taken only when some part
//! of it inside the box is what the depth buffer shows (the front face under
//! that pixel, probed on the CPU exactly as a click picks). Hidden solids,
//! scene-tree-hidden faces/edges/vertices and display-only (aux) edges are
//! never taken. The engine applies the selection filter on top
//! ([`crate::engine_state::EngineState::box_select`]).

use std::collections::{HashMap, HashSet};

use crate::pick::{front_face_at, point_view_depth, PickCandidate, PickKind, PickOptions};
use crate::scene::RenderScene;
use crate::view::{Projection, ViewCamera};

/// Grid samples for the touched-face probe (whole box).
const GRID_SAMPLES: f64 = 1600.0;
/// Most visibility samples tried per face / per edge.
const SAMPLES_PER_ENTITY: usize = 24;

/// A screen rectangle in CSS pixels, any corner order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScreenBox {
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
}

impl ScreenBox {
    /// The box from a drag's press to its current point. Dragged right to
    /// left it is a CROSSING box (see [`Self::crossing`]).
    pub fn from_drag(start: (f64, f64), end: (f64, f64)) -> Self {
        Self { x0: start.0, y0: start.1, x1: end.0, y1: end.1 }
    }

    /// Dragged right to left: crossing (touching) rather than window.
    pub fn crossing(&self) -> bool {
        self.x1 < self.x0
    }

    fn min_max(&self) -> (f64, f64, f64, f64) {
        (self.x0.min(self.x1), self.y0.min(self.y1), self.x0.max(self.x1), self.y0.max(self.y1))
    }

    fn contains(&self, x: f64, y: f64) -> bool {
        let (ax, ay, bx, by) = self.min_max();
        x >= ax && x <= bx && y >= ay && y <= by
    }
}

struct Probe<'a> {
    scene: &'a RenderScene,
    camera: &'a ViewCamera,
    options: &'a PickOptions,
    cache: HashMap<(i64, i64), Option<PickCandidate>>,
}

impl Probe<'_> {
    /// The front face at a pixel (cached at whole pixels).
    fn front(&mut self, x: f64, y: f64) -> Option<PickCandidate> {
        let key = (x.round() as i64, y.round() as i64);
        if let Some(hit) = self.cache.get(&key) {
            return hit.clone();
        }
        let hit = front_face_at(self.scene, self.camera, key.0 as f64, key.1 as f64, self.options);
        self.cache.insert(key, hit.clone());
        hit
    }

    /// The depth slack within which a point on a surface counts as ON the
    /// front surface rather than behind it.
    fn slack(&self, depth: f64) -> f64 {
        1e-4 * depth.abs().max(1.0) + self.camera.world_per_pixel() * 1.5
    }

    /// Whether a world point (projected at `(x, y)`, view depth `depth`) is
    /// visible: nothing in front of it, or the front face is at its depth.
    fn point_visible(&mut self, x: f64, y: f64, depth: f64) -> bool {
        match self.front(x, y) {
            None => true,
            Some(front) => depth <= front.depth + self.slack(depth),
        }
    }
}

/// Project, or `None` behind a perspective eye.
fn project(camera: &ViewCamera, p: [f64; 3]) -> Option<(f64, f64, f64)> {
    let (x, y, depth) = camera.project(p);
    if matches!(camera.projection, Projection::Perspective { .. }) && depth <= 1e-6 {
        return None;
    }
    Some((x, y, depth))
}

fn f64s(p: [f32; 3]) -> [f64; 3] {
    [p[0] as f64, p[1] as f64, p[2] as f64]
}

/// Every visible entity `rect` takes, as pick candidates (kind, kernel name,
/// owning solid, nearest depth): faces, edges, vertices and solids, with no
/// filter applied. WINDOW or CROSSING by the rectangle's drag direction.
pub fn box_pick(scene: &RenderScene, camera: &ViewCamera, rect: ScreenBox, options: &PickOptions) -> Vec<PickCandidate> {
    let crossing = rect.crossing();
    let (ax, ay, bx, by) = rect.min_max();
    let mut probe = Probe { scene, camera, options, cache: HashMap::new() };
    let mut out: Vec<PickCandidate> = Vec::new();
    if bx - ax < 1.0 && by - ay < 1.0 {
        return out;
    }

    // The faces (and section-capped solids) the box SHOWS: a grid of front
    // probes over the box.
    let mut touched_faces: HashSet<(String, String)> = HashSet::new();
    let mut touched_solids: HashSet<String> = HashSet::new();
    let step = (((bx - ax) * (by - ay)) / GRID_SAMPLES).sqrt().max(3.0);
    let mut y = ay;
    while y <= by {
        let mut x = ax;
        while x <= bx {
            if let Some(hit) = probe.front(x, y) {
                match hit.kind {
                    PickKind::Face => {
                        touched_solids.insert(hit.solid.clone());
                        touched_faces.insert((hit.solid, hit.name));
                    }
                    _ => {
                        touched_solids.insert(hit.solid);
                    }
                }
            }
            x += step;
        }
        y += step;
    }

    for solid in scene.solids() {
        if !solid.visible {
            continue;
        }
        let positions = &solid.mesh.positions;
        let indices = &solid.mesh.indices;
        let mut solid_all_inside = true;
        let mut solid_seen = touched_solids.contains(&solid.name);
        let mut solid_depth = f64::INFINITY;

        // --- faces -------------------------------------------------------
        for (index, face) in solid.faces.iter().enumerate() {
            if face.tri_count == 0 || !solid.visibility.is_face_visible(index) {
                continue;
            }
            let start = face.tri_start as usize;
            let end = (start + face.tri_count as usize).min(indices.len() / 3);
            let mut all_inside = true;
            let mut depth = f64::INFINITY;
            let mut position = [0.0; 3];
            for tri in start..end {
                for k in 0..3 {
                    let p = f64s(positions[indices[tri * 3 + k] as usize]);
                    match project(camera, p) {
                        Some((sx, sy, d)) => {
                            if !rect.contains(sx, sy) {
                                all_inside = false;
                            }
                            if d < depth {
                                depth = d;
                                position = p;
                            }
                        }
                        None => all_inside = false,
                    }
                }
            }
            if !all_inside {
                solid_all_inside = false;
            }
            if !crossing && !all_inside {
                continue;
            }
            let key = (solid.name.clone(), face.name.clone());
            let mut visible = touched_faces.contains(&key);
            if !visible {
                // Probe triangle centroids inside the box until one shows
                // this face (a small face the grid stepped over).
                let count = end.saturating_sub(start);
                let stride = (count / SAMPLES_PER_ENTITY).max(1);
                let mut tri = start;
                while tri < end && !visible {
                    let c = (0..3).fold([0.0; 3], |acc, k| {
                        let p = f64s(positions[indices[tri * 3 + k] as usize]);
                        [acc[0] + p[0] / 3.0, acc[1] + p[1] / 3.0, acc[2] + p[2] / 3.0]
                    });
                    if let Some((sx, sy, _)) = project(camera, c) {
                        if rect.contains(sx, sy) {
                            visible = probe
                                .front(sx, sy)
                                .is_some_and(|hit| hit.kind == PickKind::Face && hit.solid == solid.name && hit.name == face.name);
                        }
                    }
                    tri += stride;
                }
            }
            if visible {
                solid_seen = true;
                solid_depth = solid_depth.min(depth);
                out.push(PickCandidate {
                    kind: PickKind::Face,
                    name: face.name.clone(),
                    solid: solid.name.clone(),
                    depth,
                    screen_dist: 0.0,
                    position,
                });
            }
        }

        // --- edges -------------------------------------------------------
        for (index, edge) in solid.edges.iter().enumerate() {
            if edge.aux || !solid.visibility.is_edge_visible(index) || edge.polyline.is_empty() {
                continue;
            }
            let projected: Vec<Option<(f64, f64, f64)>> =
                edge.polyline.iter().map(|p| project(camera, f64s(*p))).collect();
            let all_inside = projected.iter().all(|p| p.is_some_and(|(x, y, _)| rect.contains(x, y)));
            if !all_inside {
                solid_all_inside = false;
            }
            if !crossing && !all_inside {
                continue;
            }
            // Samples along the edge on screen (every ~4 px, capped).
            // (x, y, depth, world, at an END of the edge)
            let mut samples: Vec<(f64, f64, f64, [f64; 3], bool)> = Vec::new();
            let last_segment = edge.polyline.len().saturating_sub(2);
            for (i, pair) in edge.polyline.windows(2).enumerate() {
                let (Some(a), Some(b)) = (projected[i], projected[i + 1]) else { continue };
                let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
                let n = ((len / 4.0).ceil() as usize).clamp(1, 32);
                let (pa, pb) = (f64s(pair[0]), f64s(pair[1]));
                for j in 0..=n {
                    let t = j as f64 / n as f64;
                    let w = [pa[0] + (pb[0] - pa[0]) * t, pa[1] + (pb[1] - pa[1]) * t, pa[2] + (pb[2] - pa[2]) * t];
                    let (sx, sy) = (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
                    if rect.contains(sx, sy) {
                        let end = (i == 0 && j == 0) || (i == last_segment && j == n);
                        samples.push((sx, sy, point_view_depth(camera, w), w, end));
                    }
                }
            }
            if edge.polyline.len() == 1 {
                if let Some((sx, sy, d)) = projected[0] {
                    if rect.contains(sx, sy) {
                        samples.push((sx, sy, d, f64s(edge.polyline[0]), true));
                    }
                }
            }
            // An edge is SHOWN where a point between its ends is: an end can
            // sit on the visible silhouette while the edge runs behind the
            // model (a cube's three back edges all end on its outline). Only
            // an edge too short to have an inner sample is judged by its ends.
            let inner: Vec<_> = samples.iter().filter(|s| !s.4).copied().collect();
            let samples = if inner.is_empty() { samples } else { inner };
            let stride = (samples.len() / SAMPLES_PER_ENTITY).max(1);
            let hit = samples.iter().step_by(stride).find(|s| probe.point_visible(s.0, s.1, s.2)).copied();
            if let Some((_, _, depth, position, _)) = hit {
                solid_seen = true;
                solid_depth = solid_depth.min(depth);
                out.push(PickCandidate {
                    kind: PickKind::Edge,
                    name: edge.name.clone(),
                    solid: solid.name.clone(),
                    depth,
                    screen_dist: 0.0,
                    position,
                });
            }
        }

        // --- vertices ----------------------------------------------------
        for (index, vertex) in solid.vertices.iter().enumerate() {
            if !solid.visibility.is_vertex_visible(index) {
                continue;
            }
            let Some((sx, sy, depth)) = project(camera, vertex.position) else {
                solid_all_inside = false;
                continue;
            };
            if !rect.contains(sx, sy) {
                solid_all_inside = false;
                continue;
            }
            if probe.point_visible(sx, sy, depth) {
                out.push(PickCandidate {
                    kind: PickKind::Vertex,
                    name: String::new(),
                    solid: solid.name.clone(),
                    depth,
                    screen_dist: 0.0,
                    position: vertex.position,
                });
            }
        }

        // --- the solid ---------------------------------------------------
        // Window: all of its drawn geometry inside; crossing: any of it shown.
        if solid_seen && (crossing || solid_all_inside) {
            out.push(PickCandidate {
                kind: PickKind::Solid,
                name: solid.name.clone(),
                solid: solid.name.clone(),
                depth: if solid_depth.is_finite() { solid_depth } else { 0.0 },
                screen_dist: 0.0,
                position: [0.0; 3],
            });
        }
    }
    out
}
