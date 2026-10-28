//! The interactive HTML player: a trusted template and its data format, kept
//! apart.
//!
//! * [`TEMPLATE`] (`player/player.html`) is code: one script block, one style
//!   block, no inline event handlers, and it never puts data into the page as
//!   markup (text goes in as text, pictures only as `data:image/…` or same-site
//!   `/api/tools/…` URLs). A server serves it under its own content security
//!   policy with a per-response nonce ([`served_html`]).
//! * [`PlayerData`] is data: the steps flat in reading order, each with its
//!   parent's id and depth, the meshes deduplicated per part, a node per
//!   occurrence with its matrix, and per step what is placed, what is being
//!   added, the insertion offsets and the camera. A server stores it per
//!   instructions revision and the template fetches it. Data written before
//!   the step tree (schema 1: chapters of steps) reads as a flat list.
//! * The offline copy is the same template with the data inlined
//!   ([`offline_html`]): one self-contained `.html`.

use crate::edit;
use crate::model::{Instructions, Target, ToolUse};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};

/// The player template.
pub const TEMPLATE: &str = include_str!("../player/player.html");

/// The player data's schema version.
pub const DATA_SCHEMA: u32 = 2;

/// The two player modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum Mode {
    /// Touch-first, with a step checklist and per-step sign-off.
    #[default]
    Shop,
    /// Pictures first, minimal text.
    Customer,
}

impl Mode {
    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "shop" => Some(Self::Shop),
            "customer" => Some(Self::Customer),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Shop => "shop",
            Self::Customer => "customer",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlayerData {
    pub schema: u32,
    pub title: String,
    #[serde(default)]
    pub subtitle: String,
    pub target: Target,
    /// Every step in reading order (a step's children before it).
    pub steps: Vec<PlayerStep>,
    /// Distinct tools, in first-use order.
    #[serde(default)]
    pub tools: Vec<ToolUse>,
    /// A picture per tool of the library (by `toolId`): the server's own
    /// `/api/tools/<id>/picture` as built, which the page the PLM serves
    /// loads same-site; the offline page carries only `data:image` pictures
    /// ([`PlayerData::inline_tool_pictures`]), the rest dropped.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub tool_pictures: BTreeMap<String, String>,
    pub meshes: Vec<PlayerMesh>,
    pub nodes: Vec<PlayerNode>,
    /// A motion study's baked frames: with no chapters the page is the study's
    /// player (a timeline over the frames); a step may carry one too (its
    /// motion). Absent from instructions made before motion existed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion: Option<Motion>,
}

/// A motion study baked for the player: every frame's poses, in the order
/// they play. Keyed by OCCURRENCE id (a node's `occurrence` is its solid's
/// scene name, `ACOMP2:Part`, under the occurrence), so the same JSON is also
/// what a document keeps as a study's published `motion`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Motion {
    pub fps: u32,
    /// Seconds.
    pub duration: f64,
    pub frames: Vec<MotionFrame>,
}

/// One frame: the occurrences whose pose differs from the document, each as
/// the delta over its document pose (column-major 4×4, applied over the
/// node's own matrix: `model = pose × node.matrix`); the occurrences a
/// visibility clip hides; opacities (0..1) a transparency clip sets; the
/// camera when a camera clip drives it (absolute: eye, target, up, and the
/// projection); whether parts interfere, and which.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct MotionFrame {
    pub time: f64,
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub poses: std::collections::BTreeMap<String, [f64; 16]>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub hidden: Vec<String>,
    #[serde(default, skip_serializing_if = "std::collections::BTreeMap::is_empty")]
    pub opacity: std::collections::BTreeMap<String, f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camera: Option<Camera>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub clashing: Vec<String>,
}

impl MotionFrame {
    /// Whether parts interfere at this frame.
    pub fn clash(&self) -> bool {
        !self.clashing.is_empty()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlayerStep {
    pub id: String,
    /// 1-based, in reading order.
    pub number: usize,
    /// The parent step's id; none at the top level.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub parent: Option<String>,
    /// 0 at the top level.
    #[serde(default)]
    pub depth: usize,
    pub title: String,
    #[serde(default)]
    pub notes: String,
    pub adds: Vec<PlayerAdd>,
    #[serde(default)]
    pub tools: Vec<ToolUse>,
    /// `12 N·m`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub torque: Option<String>,
    #[serde(default)]
    pub callouts: Vec<PlayerCallout>,
    pub signoff: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camera: Option<Camera>,
    /// Node indices drawn ghosted.
    pub placed: Vec<usize>,
    /// Node indices highlighted.
    pub adding: Vec<usize>,
    /// Where an adding node starts its insertion motion.
    #[serde(default)]
    pub insertion: Vec<Insertion>,
    /// The step view's explode poses: a matrix per posed node, applied after
    /// the node's own matrix (world space), as the step's pictures pose it.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub poses: Vec<Pose>,
    /// A rendered picture of the step (PNG data URL), for customer mode and
    /// for viewers without WebGL.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub image: Option<String>,
    /// A motion study's published motion attached to the step: played in
    /// place of the insertion animation, keyed by the scene's occurrences
    /// (the study's keys under the wrapper).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion: Option<Motion>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlayerAdd {
    /// The PLM part id, or the name ([`crate::model::Add::part`]).
    pub part: String,
    /// The display name (what the page shows; `part` when absent).
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub find_number: String,
    pub quantity: usize,
    /// The occurrences, so shop mode can ask for one serial per unit.
    pub occurrences: Vec<String>,
    #[serde(default)]
    pub serial_tracked: bool,
    #[serde(default)]
    pub subassembly: bool,
    /// The part's thumbnail (PNG data URL).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thumbnail: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlayerCallout {
    pub kind: crate::model::CalloutKind,
    pub text: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Camera {
    pub eye: [f64; 3],
    pub target: [f64; 3],
    pub up: [f64; 3],
    /// Perspective field of view in degrees; `None` = orthographic.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fov_deg: Option<f64>,
    /// Orthographic half height, in model units.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub half_height: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Insertion {
    pub node: usize,
    pub offset: [f64; 3],
}

/// A posed node of a step: its view's explode pose as a column-major 4×4.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Pose {
    pub node: usize,
    pub matrix: [f64; 16],
}

/// One triangle mesh, shared by every node of the same part. Arrays are
/// little-endian and base64-encoded.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlayerMesh {
    pub id: String,
    /// f32 xyz.
    pub positions: String,
    /// f32 xyz, one per position.
    pub normals: String,
    /// u32 triangle indices.
    pub indices: String,
    /// f32 xyz pairs: the solid's display edges as line segments, drawn over
    /// the faces as the viewport draws them. Absent from data written before
    /// edges were carried.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub edges: String,
}

impl PlayerMesh {
    pub fn new(id: &str, positions: &[f32], normals: &[f32], indices: &[u32]) -> Self {
        Self { id: id.into(), positions: encode_f32(positions), normals: encode_f32(normals), indices: encode_u32(indices), edges: String::new() }
    }

    /// The same mesh with its edges, given as polylines (each at least two
    /// points): every polyline becomes its consecutive segments.
    /// Take the mesh back through `placement` (a rigid column-major 4×4): the
    /// vertices through its inverse, the normals and edges through the
    /// rotation's transpose, so the mesh reads in the part's own frame.
    fn localise(&mut self, placement: &[f64; 16]) {
        let m = placement;
        let r = |p: [f32; 3]| -> [f32; 3] {
            // Rᵀ · p: rows of R are the matrix's columns' first three entries.
            let p = [f64::from(p[0]), f64::from(p[1]), f64::from(p[2])];
            [
                (m[0] * p[0] + m[1] * p[1] + m[2] * p[2]) as f32,
                (m[4] * p[0] + m[5] * p[1] + m[6] * p[2]) as f32,
                (m[8] * p[0] + m[9] * p[1] + m[10] * p[2]) as f32,
            ]
        };
        let back = |p: [f32; 3]| -> [f32; 3] { r([p[0] - m[12] as f32, p[1] - m[13] as f32, p[2] - m[14] as f32]) };
        let map = |text: &str, f: &dyn Fn([f32; 3]) -> [f32; 3]| -> String {
            let values = decode_f32(text);
            let mapped: Vec<f32> = values.chunks_exact(3).flat_map(|p| f([p[0], p[1], p[2]])).collect();
            encode_f32(&mapped)
        };
        self.positions = map(&self.positions, &back);
        self.normals = map(&self.normals, &r);
        if !self.edges.is_empty() {
            self.edges = map(&self.edges, &back);
        }
    }

    pub fn with_edges(mut self, polylines: &[Vec<[f32; 3]>]) -> Self {
        let mut segments: Vec<f32> = Vec::new();
        for line in polylines {
            for pair in line.windows(2) {
                segments.extend_from_slice(&pair[0]);
                segments.extend_from_slice(&pair[1]);
            }
        }
        self.edges = if segments.is_empty() { String::new() } else { encode_f32(&segments) };
        self
    }
}

/// One drawn solid of one occurrence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PlayerNode {
    /// The occurrence path the solid belongs to.
    pub occurrence: String,
    /// Index into `meshes`.
    pub mesh: usize,
    /// Column-major 4 × 4 placement.
    pub matrix: [f64; 16],
    /// sRGB colour, 0..1.
    pub color: [f32; 3],
}

/// What the caller supplies beside the block: the meshes and nodes of the
/// assembly as placed, each step's camera and picture, and part thumbnails.
#[derive(Debug, Clone, Default)]
pub struct Scene {
    pub meshes: Vec<PlayerMesh>,
    pub nodes: Vec<PlayerNode>,
    pub cameras: HashMap<String, Camera>,
    pub step_images: HashMap<String, String>,
    pub thumbnails: HashMap<String, String>,
    /// Each step's explode poses, by step id.
    pub poses: HashMap<String, Vec<Pose>>,
    /// The wrapper occurrence the instructions place the assembly under
    /// (`ACOMP1`): a step's motion, keyed by the assembly's own occurrences,
    /// is re-keyed under it. Empty when the nodes are the assembly's own.
    pub wrapper: String,
}

/// The tolerance of a rigid fit, as a share of the mesh's extent.
const RIGID_FIT_TOLERANCE: f64 = 1e-4;

impl Scene {
    /// One mesh per part. `keys` is one key per node (its part and member)
    /// and `placements` one optional placement per node (column-major, the
    /// engine's pose of the occurrence in the world). Among the nodes of one
    /// key, the first is the reference:
    ///
    /// * with placements, its mesh is made part-local (its vertices taken back
    ///   through its placement, normals through the rotation) and every placed
    ///   node of the key shares it, each carrying its own placement as its
    ///   matrix, whatever that copy's own tessellation was;
    /// * without, a node whose mesh is the same tessellation as the reference's
    ///   moved rigidly shares it and carries the rigid move, decided by the
    ///   vertices themselves (the same index order and count, every vertex
    ///   within `RIGID_FIT_TOLERANCE` of the fit), so a mirrored or
    ///   re-tessellated copy keeps its own.
    ///
    /// Meshes left unreferenced are dropped and the rest renumbered.
    pub fn share_meshes(&mut self, keys: &[String], placements: &[Option<[f64; 16]>]) {
        let decoded: Vec<Vec<[f64; 3]>> = self.meshes.iter().map(|mesh| decode_f32(&mesh.positions).chunks(3).map(|p| [f64::from(p[0]), f64::from(p[1]), f64::from(p[2])]).collect()).collect();
        let placement = |i: usize| placements.get(i).copied().flatten();
        let mut first_by_key: HashMap<&str, usize> = HashMap::new();
        let mut shared: Vec<Option<(usize, [f64; 16])>> = vec![None; self.nodes.len()];
        let mut localised: Vec<Option<[f64; 16]>> = vec![None; self.nodes.len()];
        for (i, node) in self.nodes.iter().enumerate() {
            let Some(key) = keys.get(i) else { continue };
            match first_by_key.get(key.as_str()) {
                None => {
                    first_by_key.insert(key, i);
                    if let Some(at) = placement(i) {
                        localised[i] = Some(at);
                    }
                }
                Some(&reference) => {
                    let rm = self.nodes[reference].mesh;
                    match (placement(reference), placement(i)) {
                        (Some(_), Some(at)) => shared[i] = Some((rm, at)),
                        _ => {
                            let cm = node.mesh;
                            if rm != cm && self.meshes[rm].indices == self.meshes[cm].indices {
                                if let Some(matrix) = rigid_fit(&decoded[rm], &decoded[cm]) {
                                    shared[i] = Some((rm, matrix));
                                }
                            }
                        }
                    }
                }
            }
        }
        for (i, at) in localised.iter().enumerate() {
            if let Some(at) = at {
                let mesh = self.nodes[i].mesh;
                self.meshes[mesh].localise(at);
                self.nodes[i].matrix = *at;
            }
        }
        for (i, found) in shared.iter().enumerate() {
            if let Some((mesh, matrix)) = found {
                self.nodes[i].mesh = *mesh;
                self.nodes[i].matrix = *matrix;
            }
        }
        let used: Vec<bool> = (0..self.meshes.len()).map(|m| self.nodes.iter().any(|n| n.mesh == m)).collect();
        let mut renumbered = vec![0usize; self.meshes.len()];
        let mut next = 0;
        for (m, keep) in used.iter().enumerate() {
            if *keep {
                renumbered[m] = next;
                next += 1;
            }
        }
        let meshes = std::mem::take(&mut self.meshes);
        self.meshes = meshes.into_iter().enumerate().filter(|(m, _)| used[*m]).map(|(_, mesh)| mesh).collect();
        for node in &mut self.nodes {
            node.mesh = renumbered[node.mesh];
        }
    }
}

/// The rigid move (a proper rotation and a translation, column-major 4×4)
/// taking every point of `a` onto the point of `b` with the same index, or
/// `None` when there is none within tolerance (a different shape, a mirror,
/// or too few points to fix a frame).
fn rigid_fit(a: &[[f64; 3]], b: &[[f64; 3]]) -> Option<[f64; 16]> {
    if a.len() != b.len() || a.len() < 3 {
        return None;
    }
    let sub = |p: [f64; 3], q: [f64; 3]| [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
    let dot = |p: [f64; 3], q: [f64; 3]| p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
    let cross = |p: [f64; 3], q: [f64; 3]| [p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]];
    let norm = |p: [f64; 3]| dot(p, p).sqrt();
    let scale = |p: [f64; 3], k: f64| [p[0] * k, p[1] * k, p[2] * k];
    // Three points that fix a frame: the farthest from the first, then the
    // farthest from that line.
    let i0 = 0;
    let i1 = (0..a.len()).max_by(|&i, &j| norm(sub(a[i], a[i0])).partial_cmp(&norm(sub(a[j], a[i0]))).unwrap_or(std::cmp::Ordering::Equal))?;
    let axis = sub(a[i1], a[i0]);
    let extent = norm(axis);
    if extent <= 0.0 {
        return None;
    }
    let off_line = |i: usize| norm(cross(axis, sub(a[i], a[i0]))) / extent;
    let i2 = (0..a.len()).max_by(|&i, &j| off_line(i).partial_cmp(&off_line(j)).unwrap_or(std::cmp::Ordering::Equal))?;
    let tolerance = RIGID_FIT_TOLERANCE * extent.max(1e-9);
    if off_line(i2) <= tolerance {
        return None;
    }
    let frame = |p: &[[f64; 3]]| -> Option<[[f64; 3]; 3]> {
        let u = sub(p[i1], p[i0]);
        let u = scale(u, 1.0 / norm(u));
        let v0 = sub(p[i2], p[i0]);
        let v = sub(v0, scale(u, dot(v0, u)));
        let lv = norm(v);
        if lv <= 0.0 {
            return None;
        }
        let v = scale(v, 1.0 / lv);
        Some([u, v, cross(u, v)])
    };
    let (fa, fb) = (frame(a)?, frame(b)?);
    // R = Fb · Faᵀ: column k of R = Σ_j fb[j] * fa[j][k].
    let mut r = [[0.0f64; 3]; 3];
    for col in 0..3 {
        for row in 0..3 {
            r[col][row] = (0..3).map(|j| fb[j][row] * fa[j][col]).sum();
        }
    }
    let apply = |p: [f64; 3]| [
        r[0][0] * p[0] + r[1][0] * p[1] + r[2][0] * p[2],
        r[0][1] * p[0] + r[1][1] * p[1] + r[2][1] * p[2],
        r[0][2] * p[0] + r[1][2] * p[1] + r[2][2] * p[2],
    ];
    let t = sub(b[i0], apply(a[i0]));
    for (p, q) in a.iter().zip(b) {
        let moved = apply(*p);
        if norm(sub([moved[0] + t[0], moved[1] + t[1], moved[2] + t[2]], *q)) > tolerance {
            return None;
        }
    }
    Some([
        r[0][0], r[0][1], r[0][2], 0.0,
        r[1][0], r[1][1], r[1][2], 0.0,
        r[2][0], r[2][1], r[2][2], 0.0,
        t[0], t[1], t[2], 1.0,
    ])
}

impl Motion {
    /// The same motion with every occurrence key put under `wrapper`
    /// (`ACOMP2` → `ACOMP1:ACOMP2`); unchanged when `wrapper` is empty.
    pub fn under(&self, wrapper: &str) -> Motion {
        if wrapper.is_empty() {
            return self.clone();
        }
        let key = |k: &str| format!("{wrapper}:{k}");
        let frames = self
            .frames
            .iter()
            .map(|frame| MotionFrame {
                time: frame.time,
                poses: frame.poses.iter().map(|(k, v)| (key(k), *v)).collect(),
                hidden: frame.hidden.iter().map(|k| key(k)).collect(),
                opacity: frame.opacity.iter().map(|(k, v)| (key(k), *v)).collect(),
                camera: frame.camera.clone(),
                clashing: frame.clashing.iter().map(|k| key(k)).collect(),
            })
            .collect();
        Motion { fps: self.fps, duration: self.duration, frames }
    }
}

/// A step's attached motion (the study's published JSON, `{study,
/// assemblySignature}` beside the frames) re-keyed under the wrapper; a
/// value that is not a motion is refused like a node naming a missing mesh.
fn step_motion(step: &crate::model::Step, wrapper: &str) -> Result<Option<Motion>, String> {
    let Some(value) = &step.motion else { return Ok(None) };
    let motion: Motion = serde_json::from_value(value.clone()).map_err(|e| format!("step '{}': its motion is not a motion study's: {e}", step.id))?;
    if motion.frames.is_empty() {
        return Err(format!("step '{}': its motion has no frames", step.id));
    }
    Ok(Some(motion.under(wrapper)))
}

impl PlayerData {
    /// The player data of `block` over `scene`.
    pub fn build(block: &Instructions, scene: Scene) -> Result<Self, String> {
        for (i, node) in scene.nodes.iter().enumerate() {
            if node.mesh >= scene.meshes.len() {
                return Err(format!("node {i} names mesh {} of {}", node.mesh, scene.meshes.len()));
            }
        }
        let occurrences: Vec<String> = scene.nodes.iter().map(|n| n.occurrence.clone()).collect();
        let mut steps = Vec::new();
        for (number, (depth, step)) in block.steps().enumerate() {
            let number = number + 1;
            let parent = block.parent_id(&step.id).flatten().map(str::to_string);
            {
                let presentation = edit::presentation(block, &step.id, &occurrences)?;
                let index_of = |paths: &[String]| -> Vec<usize> {
                    let set: std::collections::HashSet<&str> = paths.iter().map(String::as_str).collect();
                    occurrences.iter().enumerate().filter(|(_, o)| set.contains(o.as_str())).map(|(i, _)| i).collect()
                };
                let adding = index_of(&presentation.adding);
                let insertion = adding
                    .iter()
                    .filter_map(|&node| {
                        let path = &occurrences[node];
                        step.arrows
                            .iter()
                            .find(|a| path == &a.occurrence || (path.starts_with(&a.occurrence) && path[a.occurrence.len()..].starts_with(':')))
                            .map(|a| Insertion { node, offset: a.start_offset() })
                    })
                    .collect();
                steps.push(PlayerStep {
                    id: step.id.clone(),
                    number,
                    parent,
                    depth,
                    title: step.title.clone(),
                    notes: step.notes.clone(),
                    adds: step
                        .adds
                        .iter()
                        .map(|a| PlayerAdd {
                            part: a.part.clone(),
                            name: a.display_name().to_string(),
                            find_number: a.find_number.clone(),
                            quantity: a.quantity(),
                            occurrences: a.occurrences.clone(),
                            serial_tracked: a.serial_tracked,
                            subassembly: a.subassembly,
                            thumbnail: scene.thumbnails.get(&a.part).cloned(),
                        })
                        .collect(),
                    tools: step.tools.clone(),
                    torque: step.torque.as_ref().map(|t| t.text()),
                    callouts: step.callouts.iter().map(|c| PlayerCallout { kind: c.kind, text: c.text.clone() }).collect(),
                    signoff: step.signoff.required,
                    camera: scene.cameras.get(&step.id).cloned(),
                    placed: index_of(&presentation.placed),
                    adding,
                    insertion,
                    poses: scene.poses.get(&step.id).cloned().unwrap_or_default(),
                    image: scene.step_images.get(&step.id).cloned(),
                    motion: step_motion(step, &scene.wrapper)?,
                });
            }
        }
        let title = if block.front.title.is_empty() { block.target.name.clone() } else { block.front.title.clone() };
        Ok(Self {
            schema: DATA_SCHEMA,
            title,
            subtitle: block.front.subtitle.clone(),
            target: block.target.clone(),
            steps,
            tool_pictures: block
                .tools_list()
                .iter()
                .filter_map(|(tool, _)| tool.tool_id.as_deref())
                .filter(|id| !id.is_empty() && id.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-'))
                .map(|id| (id.to_string(), format!("/api/tools/{id}/picture")))
                .collect(),
            tools: block.tools_list().into_iter().map(|(t, _)| t).collect(),
            meshes: scene.meshes,
            nodes: scene.nodes,
            motion: None,
        })
    }

    /// Put the fetched tool pictures in: for each tool with a picture entry,
    /// `fetched[id]` as `Some((media type, bytes))` becomes a `data:image`
    /// URL when the type is one the page shows (PNG, JPEG, WebP) and the
    /// bytes are within [`TOOL_PICTURE_MAX_BYTES`]; any other entry (not
    /// fetched, no picture, another type, too large) is dropped, so an
    /// offline page never carries a picture it cannot load. Returns how many
    /// were put in.
    pub fn inline_tool_pictures(&mut self, fetched: &BTreeMap<String, Option<(String, Vec<u8>)>>) -> usize {
        let mut inlined = 0;
        let pictures = std::mem::take(&mut self.tool_pictures);
        for (id, url) in pictures {
            if url.starts_with("data:image/") {
                self.tool_pictures.insert(id, url);
                inlined += 1;
                continue;
            }
            let Some(Some((media, bytes))) = fetched.get(&id) else { continue };
            let media = media.split(';').next().unwrap_or("").trim().to_ascii_lowercase();
            if !matches!(media.as_str(), "image/png" | "image/jpeg" | "image/webp") || bytes.len() > TOOL_PICTURE_MAX_BYTES {
                continue;
            }
            self.tool_pictures.insert(id, format!("data:{media};base64,{}", base64(bytes)));
            inlined += 1;
        }
        inlined
    }

    /// A motion study's player: no steps, the scene's meshes and nodes, and
    /// the study's frames as `motion`. `title` is the study's name.
    pub fn motion_study(title: &str, subtitle: &str, target: Target, scene: Scene, motion: Motion) -> Result<Self, String> {
        for (i, node) in scene.nodes.iter().enumerate() {
            if node.mesh >= scene.meshes.len() {
                return Err(format!("node {i} names mesh {} of {}", node.mesh, scene.meshes.len()));
            }
        }
        if motion.frames.is_empty() {
            return Err("a motion study's player needs at least one frame".into());
        }
        Ok(Self {
            schema: DATA_SCHEMA,
            title: title.to_string(),
            subtitle: subtitle.to_string(),
            target,
            steps: Vec::new(),
            tools: Vec::new(),
            tool_pictures: BTreeMap::new(),
            meshes: scene.meshes,
            nodes: scene.nodes,
            motion: Some(motion),
        })
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("player data serializes")
    }

    /// Read the data, schema 1 (chapters of steps) included: its steps become
    /// one flat list, every step at the top.
    pub fn from_json(text: &str) -> Result<Self, String> {
        let mut value: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("player data: {e}"))?;
        let schema = value.get("schema").and_then(serde_json::Value::as_u64).unwrap_or(0);
        if schema == 0 || schema > u64::from(DATA_SCHEMA) {
            return Err(format!("player data schema {schema} is not supported"));
        }
        if schema == 1 {
            if let Some(object) = value.as_object_mut() {
                let chapters = object.remove("chapters").and_then(|c| c.as_array().cloned()).unwrap_or_default();
                if !object.contains_key("steps") {
                    let steps: Vec<serde_json::Value> =
                        chapters.into_iter().flat_map(|c| c.get("steps").and_then(|s| s.as_array().cloned()).unwrap_or_default()).collect();
                    object.insert("steps".into(), serde_json::Value::Array(steps));
                }
                object.insert("schema".into(), serde_json::Value::from(DATA_SCHEMA));
            }
        }
        serde_json::from_value(value).map_err(|e| format!("player data: {e}"))
    }
}

/// How the template finds its data and what it may do.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
struct Config<'a> {
    mode: &'a str,
    /// Where to fetch the data; `None` = inlined.
    #[serde(skip_serializing_if = "Option::is_none")]
    data_url: Option<&'a str>,
    /// The job whose steps shop mode signs off; `None` = a local checklist.
    #[serde(skip_serializing_if = "Option::is_none")]
    job: Option<&'a str>,
}

/// The self-contained offline copy: the template with `data` inlined. Shop
/// mode keeps a local checklist (no server to sign off to).
pub fn offline_html(data: &PlayerData, mode: Mode) -> String {
    let config = Config { mode: mode.as_str(), data_url: None, job: None };
    // An offline page loads no `/api/tools/…` picture: only the inlined ones go.
    let json = if data.tool_pictures.values().all(|url| url.starts_with("data:image/")) {
        data.to_json()
    } else {
        let mut data = data.clone();
        data.tool_pictures.retain(|_, url| url.starts_with("data:image/"));
        data.to_json()
    };
    fill(None, &serde_json::to_string(&config).unwrap(), &json)
}

/// The largest tool picture an offline page inlines, in bytes.
pub const TOOL_PICTURE_MAX_BYTES: usize = 1024 * 1024;

/// The template as a server serves it: `nonce` goes on the script and style
/// blocks, the page fetches its data from `data_url` (same origin), and in shop
/// mode with a `job` its sign-offs go to that job.
pub fn served_html(nonce: &str, data_url: &str, mode: Mode, job: Option<&str>) -> Result<String, String> {
    if nonce.is_empty() || !nonce.bytes().all(|b| b.is_ascii_alphanumeric() || b"+/=_-".contains(&b)) {
        return Err("the nonce must be base64 or base64url text".into());
    }
    if !data_url.starts_with('/') || data_url.starts_with("//") {
        return Err("the data URL must be a same-origin path".into());
    }
    let job = job.filter(|_| mode == Mode::Shop);
    let config = Config { mode: mode.as_str(), data_url: Some(data_url), job };
    Ok(fill(Some(nonce), &serde_json::to_string(&config).unwrap(), "null"))
}

fn fill(nonce: Option<&str>, config: &str, data: &str) -> String {
    let nonce_attribute = nonce.map(|n| format!(" nonce=\"{n}\"")).unwrap_or_default();
    TEMPLATE
        .replace("{{NONCE}}", &nonce_attribute)
        .replace("{{CONFIG}}", &script_json(config))
        .replace("{{DATA}}", &script_json(data))
}

/// JSON made safe inside a `<script type="application/json">` block: no `<`,
/// `>` or `&` (so no `</script>` and no comment openers), and no U+2028/9.
pub fn script_json(json: &str) -> String {
    let mut out = String::with_capacity(json.len());
    for ch in json.chars() {
        match ch {
            '<' => out.push_str("\\u003c"),
            '>' => out.push_str("\\u003e"),
            '&' => out.push_str("\\u0026"),
            '\u{2028}' => out.push_str("\\u2028"),
            '\u{2029}' => out.push_str("\\u2029"),
            _ => out.push(ch),
        }
    }
    out
}

const BASE64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Standard base64 with padding.
pub fn base64(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [chunk[0], *chunk.get(1).unwrap_or(&0), *chunk.get(2).unwrap_or(&0)];
        let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
        out.push(BASE64[(n >> 18) as usize & 63] as char);
        out.push(BASE64[(n >> 12) as usize & 63] as char);
        out.push(if chunk.len() > 1 { BASE64[(n >> 6) as usize & 63] as char } else { '=' });
        out.push(if chunk.len() > 2 { BASE64[n as usize & 63] as char } else { '=' });
    }
    out
}

/// Standard base64 back to bytes (an unknown character ends the input).
pub fn unbase64(text: &str) -> Vec<u8> {
    let value = |c: u8| BASE64.iter().position(|&b| b == c);
    let mut out = Vec::with_capacity(text.len() / 4 * 3);
    let mut acc: u32 = 0;
    let mut bits = 0;
    for c in text.bytes() {
        let Some(v) = value(c) else { break };
        acc = (acc << 6) | v as u32;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push((acc >> bits) as u8);
            acc &= (1 << bits) - 1;
        }
    }
    out
}

/// The f32 values an [`encode_f32`] string carries.
pub fn decode_f32(text: &str) -> Vec<f32> {
    unbase64(text).chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect()
}

pub fn encode_f32(values: &[f32]) -> String {
    base64(&values.iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>())
}

pub fn encode_u32(values: &[u32]) -> String {
    base64(&values.iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>())
}

/// A PNG as a data URL, the only picture form the player shows from data.
pub fn png_data_url(png: &[u8]) -> String {
    format!("data:image/png;base64,{}", base64(png))
}

