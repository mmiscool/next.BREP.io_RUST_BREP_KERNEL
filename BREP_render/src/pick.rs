//! Picking in Rust (R23/R24), replacing the retired raycaster +
//! `selectionMethods._pickAtEvent`: returns KERNEL NAMES with the candidate
//! priority order VERTEX > EDGE > FACE > … > SOLID, CSS-pixel thresholds for
//! lines/points (the earlier picker's `worldPerPixel * 6` is exactly 6 CSS px), a
//! double-sided face toggle, and a ranked candidate list feeding the host's
//! multi-candidate popup. Selection-filter *filtering* stays in the UI layer where the
//! filter state lives — the engine reports everything under the cursor.
//!
//! CPU ray/screen-space testing over the scene's display buffers: exact,
//! deterministic, identical on native and wasm, and cheap at CAD face counts
//! (dense meshes use cached kernel BVHs before exact triangle tests).

use crate::geometry2d::point_segment_distance;

use crate::scene::{RenderScene, SolidDisplay};
use crate::view::{add3, cross3, dot3, scale3, sub3, Projection, Ray, ViewCamera};

/// Pick thresholds in CSS pixels (ported from `selectionMethods`).
pub const EDGE_PICK_PX: f64 = 6.0;
pub const VERTEX_PICK_PX: f64 = 6.0;
const MAX_CANDIDATES: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum PickKind {
    Vertex = 0,
    Edge = 1,
    Face = 2,
    /// A construction PLANE / DATUM base plane (the pick-list category right
    /// AFTER faces). Never produced by the ray tests here — the plane cards live
    /// in the widget registry, so the engine appends one candidate per drawn card
    /// the pointer ray crosses (`EngineState::pick_candidates_at`); `name` is the
    /// datum FRAME name (`Pl`, `Datum:XY`), `solid` is empty.
    ///
    /// Ranking below FACE is deliberate: a plane competes for a pick by KIND, not
    /// by depth, so a face under the cursor always out-priorities the (large,
    /// unshaded) plane card — the plane is still listed, one row down, instead of
    /// swallowing the click. See `EngineState::pick_candidates_at`.
    Plane = 3,
    Solid = 4,
    /// An assembly COMPONENT entry (the pick-list category after solids). Never
    /// produced by the ray tests here — the engine appends one per owning
    /// component when the selection filter's COMPONENT lane is on
    /// (`candidates_filtered_at`); `name` is the component id, `solid` is empty.
    Component = 5,
}

impl PickKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            PickKind::Vertex => "VERTEX",
            PickKind::Edge => "EDGE",
            PickKind::Face => "FACE",
            PickKind::Plane => "PLANE",
            PickKind::Solid => "SOLID",
            PickKind::Component => "COMPONENT",
        }
    }
}

#[derive(Debug, Clone)]
pub struct PickCandidate {
    pub kind: PickKind,
    /// Kernel name (faces/edges; empty for unnamed). For vertices this is
    /// empty — vertices resolve by `position` within `solid`.
    pub name: String,
    /// Owning solid's scene name.
    pub solid: String,
    /// View-space depth of the hit (for ranking within a kind).
    pub depth: f64,
    /// Cursor→hit distance in CSS px (0 for face hits).
    pub screen_dist: f64,
    /// World-space hit position (face hit point / edge closest point / vertex).
    pub position: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
pub struct PickOptions {
    pub double_sided: bool,
    pub edge_px: f64,
    pub vertex_px: f64,
}

impl Default for PickOptions {
    fn default() -> Self {
        Self {
            // The retired picker force-flipped FrontSide materials to DoubleSide
            // for every pick, so double-sided is the parity default.
            double_sided: true,
            edge_px: EDGE_PICK_PX,
            vertex_px: VERTEX_PICK_PX,
        }
    }
}

/// Rank candidates under CSS-pixel `(x, y)`. Ordered by the priority chain
/// then depth; a SOLID entry per hit solid is appended at the end (the earlier
/// "extras" behavior).
pub fn pick(
    scene: &RenderScene,
    camera: &ViewCamera,
    x: f64,
    y: f64,
    options: &PickOptions,
) -> Vec<PickCandidate> {
    let ray = camera.pick_ray(x, y);
    let (_, _, forward) = camera.basis();
    let persp = matches!(camera.projection, Projection::Perspective { .. });

    let mut hits: Vec<PickCandidate> = Vec::new();
    let mut triangles = Vec::new();
    for solid in scene.solids() {
        if !solid.visible {
            continue;
        }
        if ray_hits_aabb(&ray, &solid.bbox) {
            // Small component meshes cost less to scan than to index. Dense
            // copper/silkscreen displays dominate UNO hover time without this
            // broad phase, even though they contain only a handful of faces.
            let candidates = if solid.mesh.indices.len() / 3 > 128 {
                triangles.clear();
                scene.pick_triangles(solid, &ray, &mut triangles);
                Some(triangles.as_slice())
            } else {
                None
            };
            pick_faces(solid, camera, &ray, forward, options, candidates, &mut hits);
        }
        pick_edges(solid, camera, x, y, forward, persp, options, &mut hits);
        pick_vertices(solid, camera, x, y, forward, persp, options, &mut hits);
    }

    hits.sort_by(|a, b| {
        (a.kind as u8)
            .cmp(&(b.kind as u8))
            .then(a.depth.total_cmp(&b.depth))
            .then(a.screen_dist.total_cmp(&b.screen_dist))
    });
    hits.truncate(MAX_CANDIDATES);

    // Append one SOLID candidate per hit solid, ordered by first (best) hit.
    let mut solids_seen: Vec<String> = Vec::new();
    let mut solid_entries: Vec<PickCandidate> = Vec::new();
    for hit in &hits {
        if solids_seen.iter().any(|name| name == &hit.solid) {
            continue;
        }
        solids_seen.push(hit.solid.clone());
        solid_entries.push(PickCandidate {
            kind: PickKind::Solid,
            name: hit.solid.clone(),
            solid: hit.solid.clone(),
            depth: hit.depth,
            screen_dist: hit.screen_dist,
            position: hit.position,
        });
    }
    hits.extend(solid_entries);
    hits
}

/// The nearest hit of an ALLOWED KIND under CSS-pixel `(x, y)`, using the same
/// ranking as [`pick`]. `filter` is a set of kind strings (`"SOLID"`, `"FACE"`,
/// `"EDGE"`, `"VERTEX"`, case-insensitive); an empty filter means any kind. This
/// is the type-constrained pick the reference-selection widget uses so a
/// `targetSolid` field resolves the SOLID under the cursor (candidates rank
/// faces first, then the trailing SOLID entry — `find` walks that order and
/// returns the first candidate whose kind is allowed). Returns the full
/// candidate (name + owning solid + position) or `None` on a miss.
pub fn pick_filtered(
    scene: &RenderScene,
    camera: &ViewCamera,
    x: f64,
    y: f64,
    options: &PickOptions,
    filter: &[String],
) -> Option<PickCandidate> {
    pick(scene, camera, x, y, options).into_iter().find(|c| {
        filter.is_empty() || filter.iter().any(|f| f.eq_ignore_ascii_case(c.kind.as_str()))
    })
}

fn view_depth(camera: &ViewCamera, forward: [f64; 3], point: [f64; 3]) -> f64 {
    dot3(sub3(point, camera.eye), forward)
}

#[allow(clippy::too_many_arguments)]
fn pick_faces(
    solid: &SolidDisplay,
    camera: &ViewCamera,
    ray: &Ray,
    forward: [f64; 3],
    options: &PickOptions,
    candidates: Option<&[usize]>,
    out: &mut Vec<PickCandidate>,
) {
    if solid.mesh.indices.is_empty() {
        return;
    }
    if !ray_hits_aabb(ray, &solid.bbox) {
        return;
    }
    let positions = &solid.mesh.positions;
    let indices = &solid.mesh.indices;
    for (index, face) in solid.faces.iter().enumerate() {
        // Per-entity visibility (the scene tree's face checkboxes): a hidden
        // face isn't rendered, so it must not be hoverable/pickable either.
        if !solid.visibility.is_face_visible(index) {
            continue;
        }
        if face.tri_count == 0 {
            continue;
        }
        let mut best: Option<(f64, [f64; 3])> = None;
        let start = face.tri_start as usize;
        let end = start + face.tri_count as usize;
        let end = end.min(indices.len() / 3);
        let mut test_triangle = |tri: usize| {
            let i0 = indices[tri * 3] as usize;
            let i1 = indices[tri * 3 + 1] as usize;
            let i2 = indices[tri * 3 + 2] as usize;
            let a = to_f64(positions[i0]);
            let b = to_f64(positions[i1]);
            let c = to_f64(positions[i2]);
            if let Some(t) = ray_triangle(ray, a, b, c, options.double_sided) {
                let point = add3(ray.origin, scale3(ray.dir, t));
                if best.map(|(bt, _)| t < bt).unwrap_or(true) {
                    best = Some((t, point));
                }
            }
        };
        match candidates {
            Some(triangles) => {
                let first = triangles.partition_point(|&tri| tri < start);
                let last = triangles.partition_point(|&tri| tri < end);
                for &tri in &triangles[first..last] {
                    test_triangle(tri);
                }
            }
            None => {
                for tri in start..end {
                    test_triangle(tri);
                }
            }
        }
        if let Some((_, point)) = best {
            out.push(PickCandidate {
                kind: PickKind::Face,
                name: face.name.clone(),
                solid: solid.name.clone(),
                depth: view_depth(camera, forward, point),
                screen_dist: 0.0,
                position: point,
            });
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn pick_edges(
    solid: &SolidDisplay,
    camera: &ViewCamera,
    x: f64,
    y: f64,
    forward: [f64; 3],
    persp: bool,
    options: &PickOptions,
    out: &mut Vec<PickCandidate>,
) {
    for (index, edge) in solid.edges.iter().enumerate() {
        // Hidden edges (scene-tree checkboxes) are not rendered → not pickable.
        // An AUX edge is display-only — a board's copper outline — and has no
        // entity to hover or select.
        if edge.aux || !solid.visibility.is_edge_visible(index) {
            continue;
        }
        let mut best: Option<(f64, f64, [f64; 3])> = None; // (screen_dist, depth, world)
        for pair in edge.polyline.windows(2) {
            let mut a = to_f64(pair[0]);
            let mut b = to_f64(pair[1]);
            if persp {
                let da = view_depth(camera, forward, a);
                let db = view_depth(camera, forward, b);
                const EPS: f64 = 1e-6;
                if da <= EPS && db <= EPS {
                    continue;
                }
                if da <= EPS || db <= EPS {
                    // Clip the behind-eye endpoint to just in front.
                    let t = (EPS - da) / (db - da);
                    let clip = add3(a, scale3(sub3(b, a), t));
                    if da <= EPS {
                        a = clip;
                    } else {
                        b = clip;
                    }
                }
            }
            let (ax, ay, _) = camera.project(a);
            let (bx, by, _) = camera.project(b);
            let (dist, t) = point_segment_distance((x, y), (ax, ay), (bx, by));
            if dist <= options.edge_px {
                let world = add3(a, scale3(sub3(b, a), t));
                let depth = view_depth(camera, forward, world);
                if best
                    .map(|(bd, bdepth, _)| dist < bd || (dist == bd && depth < bdepth))
                    .unwrap_or(true)
                {
                    best = Some((dist, depth, world));
                }
            }
        }
        if let Some((screen_dist, depth, position)) = best {
            out.push(PickCandidate {
                kind: PickKind::Edge,
                name: edge.name.clone(),
                solid: solid.name.clone(),
                depth,
                screen_dist,
                position,
            });
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn pick_vertices(
    solid: &SolidDisplay,
    camera: &ViewCamera,
    x: f64,
    y: f64,
    forward: [f64; 3],
    persp: bool,
    options: &PickOptions,
    out: &mut Vec<PickCandidate>,
) {
    for (index, vertex) in solid.vertices.iter().enumerate() {
        // Hidden vertices (scene-tree checkboxes) are not rendered → not pickable.
        if !solid.visibility.is_vertex_visible(index) {
            continue;
        }
        let depth = view_depth(camera, forward, vertex.position);
        if persp && depth <= 1e-6 {
            continue;
        }
        let (sx, sy, _) = camera.project(vertex.position);
        let dist = ((sx - x).powi(2) + (sy - y).powi(2)).sqrt();
        if dist <= options.vertex_px {
            out.push(PickCandidate {
                kind: PickKind::Vertex,
                name: String::new(),
                solid: solid.name.clone(),
                depth,
                screen_dist: dist,
                position: vertex.position,
            });
        }
    }
}

fn to_f64(p: [f32; 3]) -> [f64; 3] {
    [p[0] as f64, p[1] as f64, p[2] as f64]
}

/// Möller–Trumbore; returns the ray parameter t of the hit.
fn ray_triangle(ray: &Ray, a: [f64; 3], b: [f64; 3], c: [f64; 3], double_sided: bool) -> Option<f64> {
    let e1 = sub3(b, a);
    let e2 = sub3(c, a);
    let pvec = cross3(ray.dir, e2);
    let det = dot3(e1, pvec);
    const EPS: f64 = 1e-14;
    if double_sided {
        if det.abs() < EPS {
            return None;
        }
    } else if det < EPS {
        return None;
    }
    let inv_det = 1.0 / det;
    let tvec = sub3(ray.origin, a);
    let u = dot3(tvec, pvec) * inv_det;
    if !(-1e-9..=1.0 + 1e-9).contains(&u) {
        return None;
    }
    let qvec = cross3(tvec, e1);
    let v = dot3(ray.dir, qvec) * inv_det;
    if v < -1e-9 || u + v > 1.0 + 1e-9 {
        return None;
    }
    let t = dot3(e2, qvec) * inv_det;
    if t <= 0.0 {
        return None;
    }
    Some(t)
}

fn ray_hits_aabb(ray: &Ray, bbox: &crate::camera::Aabb) -> bool {
    if bbox.is_empty() {
        return false;
    }
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    for axis in 0..3 {
        let dir = ray.dir[axis];
        let origin = ray.origin[axis];
        if dir.abs() < 1e-15 {
            if origin < bbox.min[axis] - 1e-9 || origin > bbox.max[axis] + 1e-9 {
                return false;
            }
            continue;
        }
        let inv = 1.0 / dir;
        let t0 = (bbox.min[axis] - origin) * inv;
        let t1 = (bbox.max[axis] - origin) * inv;
        let (lo, hi) = if t0 <= t1 { (t0, t1) } else { (t1, t0) };
        t_min = t_min.max(lo);
        t_max = t_max.min(hi);
        if t_min > t_max {
            return false;
        }
    }
    t_max > 0.0
}

/// Serialize candidates for the R3 JSON boundary.
pub fn candidates_to_json(candidates: &[PickCandidate]) -> String {
    let list: Vec<serde_json::Value> = candidates
        .iter()
        .map(|c| {
            serde_json::json!({
                "kind": c.kind.as_str(),
                "name": c.name,
                "solid": c.solid,
                "depth": c.depth,
                "screenDist": c.screen_dist,
                "position": c.position,
            })
        })
        .collect();
    serde_json::Value::Array(list).to_string()
}

