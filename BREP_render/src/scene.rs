//! Renderer-independent display objects keyed by kernel feature names.
//! Rendering, picking, and feature-reference display share this scene map.

use crate::camera::Aabb;
use std::cell::RefCell;
use std::collections::HashMap;

/// The kind of a display face (surface classification rides along when known —
/// typed replacement for the previous app's untyped `faceKind` tag).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum FaceKind {
    Unknown,
}

/// One face of a solid: a contiguous triangle range of the solid mesh plus the
/// kernel face identity.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FaceDisplay {
    /// Kernel face name (byte-exact pipeline name); empty when unnamed.
    pub name: String,
    /// Kernel topology face id.
    pub topo_id: u64,
    /// First triangle (not index) of this face in the mesh.
    pub tri_start: u32,
    /// Triangle count.
    pub tri_count: u32,
    pub kind: FaceKind,
    /// Per-FACE base colour, resolved from this face's `color` metadata
    /// attribute by [`RenderScene::apply_metadata_colors`]. Takes precedence
    /// over the owning solid's colour (and over `faceColorMode`), but selection
    /// / hover emphasis still wins over it. `None` = inherit the solid.
    #[serde(default)]
    pub color_override: Option<[f32; 3]>,
}

/// One display edge: a world-space polyline plus the kernel edge identity and
/// the typed flags the previous display layer kept per object.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EdgeDisplay {
    /// Kernel edge name (`faceA|faceB[n]` convention); empty when unnamed.
    pub name: String,
    /// Kernel topology edge id.
    pub topo_id: u64,
    /// World-space polyline (chord-tolerance sampled, ≥ 2 points).
    pub polyline: Vec<[f32; 3]>,
    /// Auxiliary display edge (not a real BREP boundary): drawn, but never
    /// picked, listed or named — a board's copper outline is one.
    pub aux: bool,
    /// Centerline flag (hole/revolve axis display).
    pub centerline: bool,
}

/// One display vertex (kernel topology vertex).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct VertexDisplay {
    pub topo_id: u64,
    pub position: [f64; 3],
}

/// The triangle mesh of one solid, ready for GPU upload (f32; the kernel's f64
/// buffers are narrowed exactly once, here).
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct DisplayMesh {
    /// Interleaved-ready parallel arrays: xyz per vertex.
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    /// Triangle indices (3 per triangle).
    pub indices: Vec<u32>,
    /// Per-TRIANGLE face index into `SolidDisplay::faces`.
    pub face_ids: Vec<u32>,
}

/// A displayed solid: mesh + named faces/edges/vertices + visibility.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SolidDisplay {
    /// Kernel solid name — the scene key.
    pub name: String,
    /// The resident kernel handle this display was tessellated from (0 = none,
    /// e.g. a synthesized sheet with no kernel solid). Handles are monotonic and
    /// never recycled, so this is a stable identity for the geometry: the R10
    /// display-reuse fast path keeps an existing display across a history rerun
    /// ONLY when the name's resident handle still equals this — a name re-bound to
    /// a DIFFERENT handle (a SUBTRACT result inherits its target's name; a
    /// roll-back replays that name's ORIGINAL producer) must re-tessellate.
    pub source_handle: u32,
    pub visible: bool,
    /// Optional per-solid base color override (R14 — user-set solid color),
    /// else the name-hashed stable color is used.
    pub color_override: Option<[f32; 3]>,
    /// Monotonic content revision (R10). Every freshly built display gets a
    /// unique value, so the renderer's GPU-buffer cache re-uploads on any
    /// rebuild — correct-by-default (equivalent to teardown-rebuild). The
    /// reused-buffer fast path (keep a revision stable across a `reused`
    /// pipeline result) is a follow-up optimization.
    pub revision: u64,
    pub mesh: DisplayMesh,
    pub faces: Vec<FaceDisplay>,
    pub edges: Vec<EdgeDisplay>,
    pub vertices: Vec<VertexDisplay>,
    /// Per-entity + group hide state (individual faces/edges/vertices, or a
    /// whole group). Default = everything visible; see [`crate::visibility`].
    /// Reused solids keep it across history reruns (this whole struct is cloned
    /// forward); a re-tessellated solid resets to all-visible.
    pub visibility: crate::visibility::EntityVisibility,
    /// World bbox over mesh positions (edges lie on the mesh by construction).
    pub bbox: Aabb,
    /// This display is a SYNTHESIZED committed-sketch SHEET (planar face + named
    /// boundary edges + corner vertices), not a kernel solid — it carries no
    /// resident handle (`source_handle == 0`). The marker lets the UI treat a
    /// sketch as a sketch: it is listed under "Sketches" (not among solids) yet is
    /// pickable / selectable / measurable like any scene solid.
    pub is_sketch: bool,
    /// This body is SHEET METAL — its resident handle carried a `SheetTree` when
    /// the display was built. Stamped by the pipeline from
    /// [`brep_kernel::is_sheet_metal_handle`] on the RUNNER thread (where the
    /// tree's thread-local is warm), so the UI thread can answer "is this a
    /// sheet-metal body?" straight off the scene — the gate for the sheet-metal
    /// edit features (SM Flange / Fillet / Chamfer). Immutable-correct: a tree is
    /// attached at solid creation and dropped exactly when the handle is freed,
    /// handles never recycle, and the display-reuse fast path clones this struct
    /// forward only while the handle is unchanged.
    pub is_sheet_metal: bool,
}

/// Source of monotonic [`SolidDisplay::revision`] values.
pub(crate) fn next_revision() -> u64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(1);
    COUNTER.fetch_add(1, Ordering::Relaxed)
}

/// The scene: insertion-ordered solids + an exact name index (R8 — the
/// `getObjectByName` heuristic-scoring lookup is replaced by this map).
#[derive(Debug, Default)]
pub struct RenderScene {
    solids: Vec<SolidDisplay>,
    index: HashMap<String, usize>,
    /// Triangle BVHs for dense displays, built once on first pick. Scene
    /// mutations invalidate these independently of camera and visibility.
    pick_trees: RefCell<HashMap<String, brep_kernel::Bvh>>,
    /// The B-rep of the kernel displays a drawing sheet has ASKED for, keyed by
    /// solid name with the resident handle it came from: `name -> (handle,
    /// fingerprint, solid)`. Only a sheet reads topology, and the runner sends
    /// it only when asked (`EngineState::request_exact_solids`), so a document
    /// with no sheet holds none. Beside the displays rather than inside them,
    /// because a display is cloned (the PMI explode keeps originals) and
    /// rebuilt on the UI thread (committed sketches), and neither wants a copy
    /// of the topology. Read through [`Self::exact_solid`], which answers only
    /// while the handle still matches the display's. The `u64` is the solid's
    /// [`brep_fingerprint`], taken once here rather than per projection.
    exact: HashMap<String, (u32, u64, brep_kernel::BrepSolid)>,
    /// Bumped whenever [`Self::exact`] gains or loses a solid, so a projection
    /// cached without a topology is redrawn when it arrives.
    exact_revision: u64,
    /// Bumped whenever [`Self::set_visible`] actually CHANGES a solid's
    /// visibility — see [`Self::visibility_revision`].
    visibility_revision: u64,
}

impl RenderScene {
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert or replace a solid by name (replacement keeps insertion order —
    /// a boolean result reusing its target's name stays in place).
    pub fn insert_solid(&mut self, solid: SolidDisplay) {
        self.pick_trees.get_mut().remove(&solid.name);
        match self.index.get(&solid.name) {
            Some(&slot) => self.solids[slot] = solid,
            None => {
                self.index.insert(solid.name.clone(), self.solids.len());
                self.solids.push(solid);
            }
        }
    }

    /// Drop every solid (the scene rebuild path clears then repopulates).
    pub fn clear(&mut self) {
        self.pick_trees.get_mut().clear();
        self.solids.clear();
        self.index.clear();
    }

    /// Empty the scene, RETURNING every solid by value (the name index is cleared
    /// and the vec is `mem::take`-n out). The history-apply seam MOVES existing
    /// displays out this way to reinsert the reused ones without cloning their
    /// meshes — a scene-free [`crate::pipeline::SceneRunner`] delta is applied by
    /// draining then reinserting in snapshot order.
    pub fn drain(&mut self) -> Vec<SolidDisplay> {
        self.pick_trees.get_mut().clear();
        self.index.clear();
        std::mem::take(&mut self.solids)
    }

    /// Set (or clear) a solid's metadata color override, bumping its revision
    /// so the renderer re-derives the base style. Returns false if unknown.
    pub fn set_color_override(&mut self, name: &str, color: Option<[f32; 3]>) -> bool {
        let Some(slot) = self.index.get(name).copied() else {
            return false;
        };
        let solid = &mut self.solids[slot];
        if solid.color_override != color {
            solid.color_override = color;
            solid.revision = next_revision();
        }
        true
    }

    /// Re-derive every solid's and face's base colour from the name-keyed
    /// metadata store — the ONE seam through which the durable `color`
    /// attribute reaches the display.
    ///
    /// `lookup` maps an object NAME (a solid's or a face's) to its resolved
    /// colour. It returns `None` both for "no colour recorded" and for "the
    /// display setting is overriding model colours", so this method needs to
    /// know about neither.
    ///
    /// SKETCH sheets are skipped: their `color_override` is the synthesized
    /// [`crate::engine_state::SKETCH_SHEET_COLOR`], not a metadata colour, and
    /// re-deriving it from a store that has no record for the sheet would blank
    /// it back to the global face colour.
    ///
    /// A changed solid's `revision` is bumped so the renderer re-uploads it —
    /// and ONLY when something actually changed. That no-op guarantee is
    /// load-bearing, not a nicety: this runs after EVERY history apply, and the
    /// R10 GPU-buffer reuse fast path keys off a stable revision.
    pub fn apply_metadata_colors(&mut self, lookup: impl Fn(&str) -> Option<[f32; 3]>) -> bool {
        let mut any = false;
        for solid in &mut self.solids {
            if solid.is_sketch {
                continue;
            }
            let mut changed = false;
            let want = lookup(&solid.name);
            if solid.color_override != want {
                solid.color_override = want;
                changed = true;
            }
            for face in &mut solid.faces {
                // An unnamed face can carry no metadata record, so it always
                // inherits the solid rather than costing a store lookup.
                let want = if face.name.is_empty() {
                    None
                } else {
                    lookup(&face.name)
                };
                if face.color_override != want {
                    face.color_override = want;
                    changed = true;
                }
            }
            if changed {
                solid.revision = next_revision();
                any = true;
            }
        }
        any
    }

    /// Set a solid's visibility (R11). Returns false if unknown.
    pub fn set_visible(&mut self, name: &str, visible: bool) -> bool {
        let Some(solid) = self.solid_mut(name) else {
            return false;
        };
        let changed = solid.visible != visible;
        solid.visible = visible;
        if changed {
            self.visibility_revision = self.visibility_revision.wrapping_add(1);
        }
        true
    }

    /// Monotonic counter of VISIBILITY changes — the cache key for anything
    /// derived from which bodies are shown.
    ///
    /// Hiding a body is SESSION state: it writes no history, runs nothing, and
    /// lands no exact solid, so it moves none of the counters a derived
    /// drawing would otherwise key on. A drawing sheet draws "the solids
    /// visible in the scene" (`sheets::project::sheet_solid_names`), so its
    /// cached projection depends on this and nothing else could tell it.
    ///
    /// Only a REAL change counts. `EngineState::apply_view_display` sets every
    /// solid's visibility from the active PMI view's hidden set on every
    /// activation, and almost all of those calls are no-ops; counting them
    /// would re-project the open sheet for nothing.
    pub fn visibility_revision(&self) -> u64 {
        self.visibility_revision
    }

    /// Scene enumeration for the host scene-tree panel (R11): names, kind,
    /// visibility, child face/edge/vertex counts — as JSON.
    pub fn listing_json(&self) -> String {
        let solids: Vec<serde_json::Value> = self
            .solids
            .iter()
            .map(|solid| {
                serde_json::json!({
                    "name": solid.name,
                    "kind": "SOLID",
                    "visible": solid.visible,
                    "faces": solid.faces.len(),
                    "edges": solid.edges.iter().filter(|e| !e.aux).count(),
                    "vertices": solid.vertices.len(),
                })
            })
            .collect();
        serde_json::Value::Array(solids).to_string()
    }

    /// Remove a solid by exact name.
    pub fn remove_solid(&mut self, name: &str) -> bool {
        let Some(slot) = self.index.remove(name) else {
            return false;
        };
        self.solids.remove(slot);
        self.pick_trees.get_mut().remove(name);
        for value in self.index.values_mut() {
            if *value > slot {
                *value -= 1;
            }
        }
        true
    }

    pub fn solid(&self, name: &str) -> Option<&SolidDisplay> {
        self.index.get(name).map(|&slot| &self.solids[slot])
    }

    /// Drop every kept B-rep whose display is gone or now comes from a
    /// different handle. Called after a run's displays are reconciled, so "the
    /// display's handle" is that run's.
    pub fn prune_exact_solids(&mut self) {
        let (solids, index) = (&self.solids, &self.index);
        let before = self.exact.len();
        self.exact.retain(|name, (handle, _, _)| {
            index.get(name).is_some_and(|&slot| solids[slot].source_handle == *handle)
        });
        if self.exact.len() != before {
            self.exact_revision += 1;
        }
    }

    /// Record the B-rep the runner sent for `name`, when `handle` is still the
    /// one its display was tessellated from; `false` (and nothing kept) when
    /// the display has moved on since the request went out.
    pub fn set_exact_solid(&mut self, name: &str, handle: u32, solid: brep_kernel::BrepSolid) -> bool {
        if handle == 0 || self.solid(name).map(|display| display.source_handle) != Some(handle) {
            return false;
        }
        self.exact.insert(name.to_string(), (handle, brep_fingerprint(&solid), solid));
        self.exact_revision += 1;
        true
    }

    /// Forget the B-rep of `name`: its geometry changed under the SAME handle
    /// (an assembly solve re-poses a resident solid in place), so the next
    /// projection must ask again.
    pub fn drop_exact_solid(&mut self, name: &str) {
        if self.exact.remove(name).is_some() {
            self.exact_revision += 1;
        }
    }

    /// Forget every B-rep: the handles they were keyed by belong to a runner
    /// that is gone (a cancel restarts its registry, which counts from one
    /// again) or to a document that was replaced.
    pub fn clear_exact_solids(&mut self) {
        if !self.exact.is_empty() {
            self.exact.clear();
            self.exact_revision += 1;
        }
    }

    /// See [`Self::exact`]'s revision counter.
    pub fn exact_revision(&self) -> u64 {
        self.exact_revision
    }

    /// The B-rep the displayed solid `name` was tessellated from, or `None`
    /// when the scene holds none for its CURRENT handle — a synthesized sketch
    /// sheet, or a solid whose topology did not reach this thread.
    pub fn exact_solid(&self, name: &str) -> Option<&brep_kernel::BrepSolid> {
        self.exact_solid_fingerprinted(name).map(|(solid, _)| solid)
    }

    /// [`Self::exact_solid`] with its content fingerprint.
    pub fn exact_solid_fingerprinted(&self, name: &str) -> Option<(&brep_kernel::BrepSolid, u64)> {
        let display = self.solid(name)?;
        self.exact
            .get(name)
            .filter(|(handle, _, _)| *handle == display.source_handle && *handle != 0)
            .map(|(_, fingerprint, solid)| (solid, *fingerprint))
    }

    pub fn solid_mut(&mut self, name: &str) -> Option<&mut SolidDisplay> {
        let slot = *self.index.get(name)?;
        // A caller can edit positions, indices or face ranges through this
        // reference without changing the display revision.
        self.pick_trees.get_mut().remove(name);
        Some(&mut self.solids[slot])
    }

    pub(crate) fn pick_triangles(&self, solid: &SolidDisplay, ray: &crate::view::Ray, out: &mut Vec<usize>) {
        let mut trees = self.pick_trees.borrow_mut();
        let tree = trees.entry(solid.name.clone()).or_insert_with(|| {
            let boxes: Vec<_> = solid.mesh.indices.chunks_exact(3).map(|indices| {
                let mut bounds = brep_kernel::Aabb::empty();
                for &index in indices {
                    let p = solid.mesh.positions[index as usize];
                    bounds.include_point(brep_kernel::Vec3::new(p[0] as f64, p[1] as f64, p[2] as f64));
                }
                bounds
            }).collect();
            brep_kernel::Bvh::build(&boxes)
        });
        let v = |p: [f64; 3]| brep_kernel::Vec3::new(p[0], p[1], p[2]);
        tree.intersecting_ray(v(ray.origin), v(ray.dir), 1e-9, out);
        // Preserve face/triangle traversal order, including tied hits.
        out.sort_unstable();
    }

    /// Insertion-ordered iteration (deterministic — drives draw order).
    pub fn solids(&self) -> &[SolidDisplay] {
        &self.solids
    }

    /// The world-space polyline of the first display edge named `name` across all
    /// solids (widened to `f64`), or `None` when no edge carries that exact name.
    /// The engine-native sketch pickEdges tool (S6b-2) uses this to fetch a picked
    /// scene edge's geometry for projection into the sketch plane. There is no name
    /// index for edges (only solids), so this is a linear scan — fine for the
    /// interactive per-click use.
    pub fn edge_polyline_world(&self, name: &str) -> Option<Vec<[f64; 3]>> {
        for solid in &self.solids {
            for edge in &solid.edges {
                if edge.name == name {
                    return Some(
                        edge.polyline
                            .iter()
                            .map(|p| [p[0] as f64, p[1] as f64, p[2] as f64])
                            .collect(),
                    );
                }
            }
        }
        None
    }

    /// The name of the solid owning the first display edge named `name`, or `None`
    /// (the companion of [`edge_polyline_world`](Self::edge_polyline_world) — the
    /// pickEdges tool stores it as external-ref metadata).
    pub fn edge_solid_name(&self, name: &str) -> Option<&str> {
        for solid in &self.solids {
            if solid.edges.iter().any(|edge| edge.name == name) {
                return Some(&solid.name);
            }
        }
        None
    }

    /// The world plane of the first display face named `name` across all solids,
    /// as `(centroid, unit outward normal)`: the area-weighted centroid of the
    /// face's mesh triangles and the normalized sum of their cross products. The
    /// watertight tessellation winds every face's triangles by `same_sense`, so
    /// the cross-product sum IS the outward normal (the stored per-vertex normals
    /// are shading normals and can be blended at shared boundary vertices). This
    /// is what lets an extrude/revolve whose `profile` is a resident solid FACE
    /// (not a sketch) anchor its dimension gizmo — the engine's dimension refs
    /// fall back to it (`EngineState::lookup_profile_plane`). Hidden solids count
    /// too: a hidden source solid still anchors the gizmo. `None` when no face
    /// carries that exact name, it has no triangles, or the triangles are
    /// degenerate (zero area). Linear scan like [`edge_polyline_world`](Self::edge_polyline_world).
    pub fn face_plane_world(&self, name: &str) -> Option<([f64; 3], [f64; 3])> {
        for solid in &self.solids {
            let Some(face) = solid.faces.iter().find(|face| face.name == name) else {
                continue;
            };
            let positions = &solid.mesh.positions;
            let indices = &solid.mesh.indices;
            let start = face.tri_start as usize;
            let end = (start + face.tri_count as usize).min(indices.len() / 3);
            let mut weighted = [0.0f64; 3];
            let mut normal = [0.0f64; 3];
            let mut total_area = 0.0f64;
            for tri in start..end {
                let a = f64_point(positions[indices[tri * 3] as usize]);
                let b = f64_point(positions[indices[tri * 3 + 1] as usize]);
                let c = f64_point(positions[indices[tri * 3 + 2] as usize]);
                let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                // Twice the signed-area vector; its length is 2·area.
                let cross = [
                    ab[1] * ac[2] - ab[2] * ac[1],
                    ab[2] * ac[0] - ab[0] * ac[2],
                    ab[0] * ac[1] - ab[1] * ac[0],
                ];
                let area =
                    0.5 * (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt();
                for k in 0..3 {
                    weighted[k] += area * (a[k] + b[k] + c[k]) / 3.0;
                    normal[k] += cross[k];
                }
                total_area += area;
            }
            let normal_len =
                (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
            if total_area <= 1e-18 || normal_len <= 1e-18 {
                return None;
            }
            return Some((
                [
                    weighted[0] / total_area,
                    weighted[1] / total_area,
                    weighted[2] / total_area,
                ],
                [normal[0] / normal_len, normal[1] / normal_len, normal[2] / normal_len],
            ));
        }
        None
    }

    pub fn is_empty(&self) -> bool {
        self.solids.is_empty()
    }

    /// World bbox over every VISIBLE solid.
    pub fn bbox(&self) -> Aabb {
        let mut bbox = Aabb::empty();
        for solid in &self.solids {
            if solid.visible {
                bbox.union(&solid.bbox);
            }
        }
        bbox
    }
}

/// A hash of everything in a B-rep the drawing sheet's hidden-line pass reads: every vertex,
/// curve, carrier and pcurve number to the bit, and the topology that ties
/// them together.
pub fn brep_fingerprint(solid: &brep_kernel::BrepSolid) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    let mut numbers = |values: &mut dyn Iterator<Item = f64>| {
        for value in values {
            value.to_bits().hash(&mut hasher);
        }
    };
    let curve = |curve: &brep_kernel::NurbsCurve| -> Vec<f64> {
        let mut out = vec![curve.degree as f64];
        out.extend(curve.knots.iter().copied());
        out.extend(curve.control_points.iter().flat_map(|cp| [cp.x, cp.y, cp.z, cp.w]));
        out
    };
    for vertex in &solid.vertices {
        numbers(&mut [vertex.id as f64, vertex.point.x, vertex.point.y, vertex.point.z].into_iter());
    }
    for edge in &solid.edges {
        numbers(&mut [edge.id as f64, edge.t0, edge.t1, edge.start_vertex_id as f64, edge.end_vertex_id as f64].into_iter());
        numbers(&mut [edge.degenerate as u8 as f64].into_iter());
        numbers(&mut curve(&edge.curve).into_iter());
    }
    for shell in &solid.shells {
        for face in &shell.faces {
            let surface = &face.surface;
            numbers(&mut [face.same_sense as u8 as f64, surface.degree_u as f64, surface.degree_v as f64].into_iter());
            numbers(&mut surface.knots_u.iter().chain(&surface.knots_v).copied());
            numbers(&mut surface.control_points.iter().flatten().flat_map(|cp| [cp.x, cp.y, cp.z, cp.w]));
            for lp in &face.loops {
                numbers(&mut [f64::NAN].into_iter());
                for coedge in &lp.coedges {
                    numbers(&mut [coedge.edge_id as f64, coedge.forward as u8 as f64].into_iter());
                    numbers(&mut curve(&coedge.pcurve).into_iter());
                }
            }
        }
    }
    hasher.finish()
}

/// Widen a display-mesh vertex to `f64` (the mesh stores `f32`).
fn f64_point(p: [f32; 3]) -> [f64; 3] {
    [p[0] as f64, p[1] as f64, p[2] as f64]
}

/// Build a [`SolidDisplay`] from the kernel's native display payload.
pub fn solid_display_from_payload(
    name: &str,
    payload: brep_kernel::DisplaySolidPayload,
) -> SolidDisplay {
    let mesh_in = payload.mesh;
    let vertex_count = mesh_in.positions.len() / 3;
    let mut positions = Vec::with_capacity(vertex_count);
    let mut normals = Vec::with_capacity(vertex_count);
    let mut bbox = Aabb::empty();
    for i in 0..vertex_count {
        let p = [
            mesh_in.positions[i * 3],
            mesh_in.positions[i * 3 + 1],
            mesh_in.positions[i * 3 + 2],
        ];
        bbox.expand(p);
        positions.push([p[0] as f32, p[1] as f32, p[2] as f32]);
        normals.push([
            mesh_in.normals[i * 3] as f32,
            mesh_in.normals[i * 3 + 1] as f32,
            mesh_in.normals[i * 3 + 2] as f32,
        ]);
    }

    // Face ranges: the watertight mesh emits each face's triangles as one
    // contiguous run of `face_ids`. Group runs; a face with no triangles gets
    // an empty range.
    let mut faces: Vec<FaceDisplay> = payload
        .faces
        .iter()
        .map(|(topo_id, name)| FaceDisplay {
            name: name.clone().unwrap_or_default(),
            topo_id: *topo_id,
            tri_start: 0,
            tri_count: 0,
            kind: FaceKind::Unknown,
            color_override: None,
        })
        .collect();
    let mut run_start = 0u32;
    let mut run_face: Option<u32> = None;
    for (tri, &face_id) in mesh_in.face_ids.iter().enumerate() {
        if run_face != Some(face_id) {
            run_face = Some(face_id);
            run_start = tri as u32;
        }
        if let Some(face) = faces.get_mut(face_id as usize) {
            if face.tri_count == 0 {
                face.tri_start = run_start;
            }
            face.tri_count += 1;
        }
    }

    // Edges and vertices expand the bbox too. For a real solid they lie ON the
    // meshed boundary, so this changes nothing; for a synthesized OPEN-sketch
    // display (edges + endpoints, no face) they are the ONLY extent there is, and
    // an empty bbox would leave the sketch out of zoom-to-fit and out of every
    // bbox-gated traversal.
    for (_, _, points) in &payload.edges {
        for point in points {
            bbox.expand([point.x, point.y, point.z]);
        }
    }
    for (_, point) in &payload.vertices {
        bbox.expand([point.x, point.y, point.z]);
    }

    let edges = payload
        .edges
        .into_iter()
        .map(|(topo_id, name, points)| EdgeDisplay {
            name: name.unwrap_or_default(),
            topo_id,
            polyline: points
                .iter()
                .map(|p| [p.x as f32, p.y as f32, p.z as f32])
                .collect(),
            aux: false,
            centerline: false,
        })
        .collect();

    let vertices = payload
        .vertices
        .into_iter()
        .map(|(topo_id, p)| VertexDisplay {
            topo_id,
            position: [p.x, p.y, p.z],
        })
        .collect();

    SolidDisplay {
        name: name.to_string(),
        source_handle: 0, // set by the caller that knows the resident handle
        visible: true,
        color_override: None,
        revision: next_revision(),
        mesh: DisplayMesh {
            positions,
            normals,
            indices: mesh_in.indices,
            face_ids: mesh_in.face_ids,
        },
        faces,
        edges,
        vertices,
        visibility: crate::visibility::EntityVisibility::default(),
        bbox,
        is_sketch: false, // set by the sketch-sheet synthesizer for a committed sketch
        is_sheet_metal: false, // stamped by the pipeline from the resident handle's SheetTree
    }
}
