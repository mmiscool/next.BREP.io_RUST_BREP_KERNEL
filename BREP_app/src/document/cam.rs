//! The document's `cam` block on the shared CAM model (`brep_cam::model`):
//! the user's setups, tools, operations and NC programs, read from and
//! written to the raw JSON block the history keeps
//! ([`brep_render::history::History::cam_block`]).
//!
//! This is the ONE place the block's shape is spelled. The model types are
//! lane C0's and serialize as the persistence contract says (camelCase,
//! `kind`-tagged enums, `serde(default)` on every newer field); this module
//! owns only the container around them and the document-level identities —
//! the id counter and the NC program records, which are user input the post
//! lane (C4) assembles into a `brep_cam::program::ProgramInput` at post time.
//!
//! Reading is version-tolerant two ways: fields the model gained later
//! default, and a block written before the model was imported (lane C1 row 1:
//! an operation's `strategy` as a bare kind name, a tool with no geometry) is
//! upgraded on read. A block that still does not parse is reported, never
//! replaced: [`load`] fails, the pane shows why, and every mutation is
//! refused until the document is repaired, so no save can silently drop what
//! a user wrote.

use brep_cam::model::{ChainRef, ChainSource, EdgeRef, Machine, Operation, OperationId, Point3, RigidTransform, Setup, Strategy, TargetRef, Tool, ToolGeometry, ToolKind};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// An NC program: an ordered group of operations posted through one
/// post-processor to one file. User input; the motion stream it posts is
/// derived at post time (lane C4).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct NcProgram {
    pub id: String,
    pub name: String,
    /// The program number the post writes (`O1234`), when the post has one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub number: Option<u32>,
    /// The post-processor's id (a built-in name or a plugin's script id).
    pub post: String,
    /// The operations, in posting order.
    pub operations: Vec<OperationId>,
    /// The post's property values by name (`programNumber`, a mill-turn
    /// post's builder codes, ...); absent ones take the post's defaults.
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub properties: BTreeMap<String, Value>,
}

/// How a setup's machine frame was PICKED: the model entities whose
/// geometry gives the frame's Z axis, X axis and origin. Kept beside the
/// setup (keyed by its id) rather than in it, because the shared model
/// carries the resolved `modelToMachine` transform only; the references are
/// what let the frame follow the model when it is rebuilt, and what the
/// dialog shows in its picker fields. Entity names are the persistent names
/// the selection uses (`Box_PZ`, a `{solid}@x,y,z` vertex).
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct SetupOrientation {
    /// Machine +Z: a planar FACE (its outward normal), a cylindrical FACE
    /// (its axis) or a straight EDGE (its direction, first point to last).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub z_ref: Option<String>,
    /// Machine +Z as a model direction instead of a pick: an axis name
    /// (`+X`, `-Y`, ...) or three numbers `x,y,z` (normalized). A tilted
    /// (3+2) setup that no face or edge of the model points along takes
    /// this. The form sets one and clears the other; a pick committed
    /// directly may leave both, and then `z_ref` wins.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub z_dir: Option<String>,
    /// Reverse the Z the reference gives (an axis or an edge has no
    /// preferred sense; a face's normal points out of the material).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub flip_z: bool,
    /// A straight EDGE whose direction is machine +X (re-orthogonalized
    /// against Z); absent, the model's X projected onto the Z plane.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub x_ref: Option<String>,
    /// A VERTEX (or a FACE, its plane point) that becomes the machine origin;
    /// absent, the Z face's plane point, else the model origin.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub origin_ref: Option<String>,
}

impl SetupOrientation {
    pub fn is_empty(&self) -> bool {
        self.z_ref.is_none() && self.z_dir.is_none() && self.x_ref.is_none() && self.origin_ref.is_none() && !self.flip_z
    }
}

/// What a picked face is, for orientation: a plane (a point on it and its
/// outward normal) or a cylinder (a point on its axis and the axis).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FaceShape {
    Planar { point: Point3, normal: Point3 },
    Cylindrical { point: Point3, axis: Point3 },
}

/// The symmetric 3x3 `m`'s eigenvalues (descending) with unit eigenvectors,
/// by Jacobi rotations (exact enough for a 3x3 normal covariance).
fn eigen3(mut m: [[f64; 3]; 3]) -> [(f64, [f64; 3]); 3] {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let (mut p, mut q, mut largest) = (0, 1, 0.0f64);
        for (i, j) in [(0, 1), (0, 2), (1, 2)] {
            if m[i][j].abs() > largest {
                largest = m[i][j].abs();
                p = i;
                q = j;
            }
        }
        if largest < 1e-15 {
            break;
        }
        let theta = 0.5 * (2.0 * m[p][q]).atan2(m[q][q] - m[p][p]);
        let (c, s) = (theta.cos(), theta.sin());
        let mut r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        r[p][p] = c;
        r[q][q] = c;
        r[p][q] = s;
        r[q][p] = -s;
        // m = rᵀ m r; v = v r
        let mul = |a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]| {
            let mut o = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    o[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
                }
            }
            o
        };
        let rt = [[r[0][0], r[1][0], r[2][0]], [r[0][1], r[1][1], r[2][1]], [r[0][2], r[1][2], r[2][2]]];
        m = mul(&mul(&rt, &m), &r);
        v = mul(&v, &r);
    }
    let mut out = [0, 1, 2].map(|k| (m[k][k], [v[0][k], v[1][k], v[2][k]]));
    out.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    out
}

/// Classify a face from its triangles: PLANAR when every triangle normal
/// agrees (the normal covariance has one eigenvalue), CYLINDRICAL when every
/// normal lies perpendicular to one direction (the covariance's smallest
/// eigenvalue vanishes while the other two do not) — that direction is the
/// axis. Anything else (a cone, a sphere, a freeform face) is `None`. The
/// point is the area-weighted centroid, which for a whole cylinder lies on
/// its axis.
pub fn classify_face(triangles: &[[[f64; 3]; 3]]) -> Option<FaceShape> {
    let (mut area, mut centroid, mut mean, mut cov) = (0.0f64, [0.0f64; 3], [0.0f64; 3], [[0.0f64; 3]; 3]);
    for [a, b, c] in triangles {
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len <= 1e-18 {
            continue;
        }
        let w = 0.5 * len;
        let u = [n[0] / len, n[1] / len, n[2] / len];
        for i in 0..3 {
            centroid[i] += w * (a[i] + b[i] + c[i]) / 3.0;
            mean[i] += w * u[i];
            for j in 0..3 {
                cov[i][j] += w * u[i] * u[j];
            }
        }
        area += w;
    }
    if area <= 1e-18 {
        return None;
    }
    for row in cov.iter_mut() {
        for x in row.iter_mut() {
            *x /= area;
        }
    }
    let point = Point3::new(centroid[0] / area, centroid[1] / area, centroid[2] / area);
    let [(l1, e1), (l2, _), (l3, e3)] = eigen3(cov);
    const FLAT: f64 = 1e-6;
    if l2 + l3 < FLAT && l1 > 1.0 - 1e-3 {
        // The mean normal gives the sense (out of the material).
        let s = if e1[0] * mean[0] + e1[1] * mean[1] + e1[2] * mean[2] < 0.0 { -1.0 } else { 1.0 };
        return Some(FaceShape::Planar { point, normal: Point3::new(s * e1[0], s * e1[1], s * e1[2]) });
    }
    if l3 < FLAT && l2 > FLAT {
        // An axis has no sense of its own: the one with a non-negative
        // largest component, so a vertical cylinder gives +Z (Flip Z reverses it).
        let k = (0..3).max_by(|&i, &j| e3[i].abs().partial_cmp(&e3[j].abs()).unwrap_or(std::cmp::Ordering::Equal)).unwrap_or(2);
        let s = if e3[k] < 0.0 { -1.0 } else { 1.0 };
        return Some(FaceShape::Cylindrical { point, axis: Point3::new(s * e3[0], s * e3[1], s * e3[2]) });
    }
    None
}

/// The machine frame a setup's orientation resolves to, in model
/// coordinates: what the viewport draws as the WCS triad and what
/// `Setup::model_to_machine` is built from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SetupFrame {
    pub origin: Point3,
    pub x_axis: Point3,
    pub z_axis: Point3,
}

impl Default for SetupFrame {
    fn default() -> Self {
        Self { origin: Point3::ZERO, x_axis: Point3::new(1.0, 0.0, 0.0), z_axis: Point3::new(0.0, 0.0, 1.0) }
    }
}

impl SetupFrame {
    /// The frame's transform: the one place a frame becomes the model's
    /// `modelToMachine`. An unusable frame (X along Z) is an error.
    pub fn transform(&self) -> Result<RigidTransform, String> {
        RigidTransform::from_frame(self.origin, self.x_axis, self.z_axis)
            .map_err(|e| format!("the setup's frame is not usable: {e:?}"))
    }
}

/// Resolve an orientation's references against the model: `face_plane`
/// answers a face name with (a point on it, its outward normal) and
/// `edge_polyline` an edge name with its world polyline (a feature's axis
/// as a straight two-point line along it). A reference that
/// does not resolve is an error naming it, so a frame never silently falls
/// back to the model's axes once a pick was made.
pub fn resolve_frame(
    orientation: &SetupOrientation,
    face_shape: &dyn Fn(&str) -> Option<FaceShape>,
    edge_polyline: &dyn Fn(&str) -> Option<Vec<[f64; 3]>>,
) -> Result<SetupFrame, String> {
    let mut frame = SetupFrame::default();
    let mut z_plane_point = None;
    if let Some(name) = &orientation.z_ref {
        let (z, point) = match face_shape(name) {
            Some(FaceShape::Planar { point, normal }) => (normal, point),
            Some(FaceShape::Cylindrical { point, axis }) => (axis, point),
            None => match edge_polyline(name) {
                Some(line) => {
                    let (Some(&a), Some(&b)) = (line.first(), line.last()) else {
                        return Err(format!("Z edge `{name}` has no length"));
                    };
                    let chord = Point3::new(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
                    let length = chord.length();
                    let straight = length > 1e-12
                        && line.iter().all(|p| {
                            let d = Point3::new(p[0] - a[0], p[1] - a[1], p[2] - a[2]);
                            let t = d.dot(chord) / (length * length);
                            let off = Point3::new(d.x - chord.x * t, d.y - chord.y * t, d.z - chord.z * t);
                            off.length() <= 1e-6 * length.max(1.0)
                        });
                    if !straight {
                        return Err(format!("Z edge `{name}` is not straight: pick a straight edge, a planar face or a cylindrical face"));
                    }
                    (chord, Point3::new(a[0], a[1], a[2]))
                }
                None => return Err(format!("Z reference `{name}` is not a planar face, a cylindrical face, an edge or a feature's axis of the model")),
            },
        };
        frame.z_axis = if orientation.flip_z { Point3::new(-z.x, -z.y, -z.z) } else { z };
        z_plane_point = Some(point);
    } else if let Some(dir) = &orientation.z_dir {
        let z = parse_direction(dir)?;
        frame.z_axis = if orientation.flip_z { Point3::new(-z.x, -z.y, -z.z) } else { z };
    } else if orientation.flip_z {
        frame.z_axis = Point3::new(0.0, 0.0, -1.0);
    }
    if let Some(edge) = &orientation.x_ref {
        let line = edge_polyline(edge).ok_or_else(|| format!("X edge `{edge}` is not an edge of the model"))?;
        let (Some(first), Some(last)) = (line.first(), line.last()) else {
            return Err(format!("X edge `{edge}` has no length"));
        };
        frame.x_axis = Point3::new(last[0] - first[0], last[1] - first[1], last[2] - first[2]);
    } else {
        // The model's X, unless Z already lies along it: then the model's Y.
        let z = frame.z_axis.normalized().unwrap_or(Point3::new(0.0, 0.0, 1.0));
        frame.x_axis = if z.x.abs() > 0.9 { Point3::new(0.0, 1.0, 0.0) } else { Point3::new(1.0, 0.0, 0.0) };
    }
    frame.origin = match &orientation.origin_ref {
        Some(name) => match vertex_position(name) {
            Some(position) => position,
            None => match face_shape(name) {
                Some(FaceShape::Planar { point, .. } | FaceShape::Cylindrical { point, .. }) => point,
                None => return Err(format!("origin `{name}` is neither a vertex nor a planar or cylindrical face of the model")),
            },
        },
        None => z_plane_point.unwrap_or(Point3::ZERO),
    };
    frame.transform()?;
    Ok(frame)
}

/// A setup's Z direction: `+X`/`-X`/`+Y`/`-Y`/`+Z`/`-Z` (the sign may be
/// omitted for +, case ignored) or three numbers `x,y,z`, normalized. A zero
/// or malformed direction is an error naming it.
pub fn parse_direction(text: &str) -> Result<Point3, String> {
    let t = text.trim();
    let axis = |sign: f64, c: &str| -> Option<Point3> {
        match c {
            "x" => Some(Point3::new(sign, 0.0, 0.0)),
            "y" => Some(Point3::new(0.0, sign, 0.0)),
            "z" => Some(Point3::new(0.0, 0.0, sign)),
            _ => None,
        }
    };
    let lower = t.to_ascii_lowercase();
    let named = match lower.as_str() {
        s if s.len() == 2 && s.starts_with('-') => axis(-1.0, &s[1..]),
        s if s.len() == 2 && s.starts_with('+') => axis(1.0, &s[1..]),
        s if s.len() == 1 => axis(1.0, s),
        _ => None,
    };
    if let Some(d) = named {
        return Ok(d);
    }
    let parts: Vec<Option<f64>> = t.split(',').map(|p| p.trim().parse::<f64>().ok().filter(|v| v.is_finite())).collect();
    if let [Some(x), Some(y), Some(z)] = parts[..] {
        if let Some(d) = Point3::new(x, y, z).normalized() {
            return Ok(d);
        }
    }
    Err(format!("Z direction `{t}` is neither an axis (+X, -Y, ...) nor three numbers x,y,z of a non-zero vector"))
}

/// A vertex reference's world position: `{solid}@x,y,z`.
pub fn vertex_position(name: &str) -> Option<Point3> {
    let (_, coords) = name.rsplit_once('@')?;
    let mut parts = coords.split(',').map(|p| p.trim().parse::<f64>().ok());
    let position = Point3::new(parts.next()??, parts.next()??, parts.next()??);
    if parts.next().is_some() {
        return None;
    }
    Some(position)
}

/// The `cam` block.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct CamDocument {
    pub setups: Vec<Setup>,
    pub tools: Vec<Tool>,
    pub operations: Vec<Operation>,
    /// Exact automatically programmed feature inputs, regenerated associatively on model edits.
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub autoprogram_inputs: BTreeMap<String, brep_cam::autoprogram::binding::Binding>,
    pub nc_programs: Vec<NcProgram>,
    /// How each setup's frame was picked, by setup id (see [`SetupOrientation`]).
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub orientations: BTreeMap<String, SetupOrientation>,
    /// The document's own (custom) machines; a setup's `machine` id names
    /// one of these or a built-in of `brep_cam::kinematics::library`.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub machines: Vec<Machine>,
    /// Each multi-axis finish's three-axis source operation, by the finish's
    /// id (see `panels::cam_multiaxis`): the generator tilts that path and
    /// never guesses one.
    #[serde(skip_serializing_if = "BTreeMap::is_empty")]
    pub multi_axis_sources: BTreeMap<String, String>,
    /// One counter mints every id (`SETUP1`, `T2`, `OP3`, `NC4`): monotonic,
    /// never reused — deleting a record does not free its number — and
    /// rewound by undo with the block it lives in.
    pub id_counter: u64,
}

/// The strategy kinds, in the model's order: the serde tag of each
/// [`Strategy`] variant, READ FROM THE MODEL rather than pinned here — serde
/// names every variant when it refuses an unknown tag, and that list is the
/// enum's declaration order — so a kind lane C0 adds reaches the pane, the
/// automation and the operation dialog without an edit in the app. Computed
/// once; `strategy_kinds_are_the_models` checks that each one builds.
pub fn strategy_kinds() -> &'static [&'static str] {
    Strategy::KINDS
}

/// The strategy of `kind` with the model's defaults, or why there is none.
pub fn strategy_of_kind(kind: &str) -> Result<Strategy, String> {
    let kind = kind.trim();
    if kind.is_empty() {
        return Err("an operation needs a strategy".to_string());
    }
    Strategy::default_of(kind).ok_or_else(|| format!("no strategy `{kind}` (one of: {})", strategy_kinds().join(", ")))
}

impl CamDocument {
    /// Whether the block holds nothing — then it is omitted from the file.
    pub fn is_empty(&self) -> bool {
        self.setups.is_empty() && self.tools.is_empty() && self.operations.is_empty() && self.nc_programs.is_empty() && self.machines.is_empty()
    }

    /// A setup's orientation picks (empty when none were made).
    pub fn orientation(&self, setup_id: &str) -> SetupOrientation {
        self.orientations.get(setup_id).cloned().unwrap_or_default()
    }

    /// Replace a setup's orientation picks; an empty one is dropped.
    pub fn set_orientation(&mut self, setup_id: &str, orientation: SetupOrientation) {
        if orientation.is_empty() {
            self.orientations.remove(setup_id);
        } else {
            self.orientations.insert(setup_id.to_string(), orientation);
        }
    }

    /// Mint the next id with `prefix`.
    pub fn next_id(&mut self, prefix: &str) -> String {
        self.id_counter += 1;
        format!("{prefix}{}", self.id_counter)
    }

    pub fn setup(&self, id: &str) -> Option<&Setup> {
        self.setups.iter().find(|s| s.id.as_str() == id)
    }

    pub fn setup_mut(&mut self, id: &str) -> Option<&mut Setup> {
        self.setups.iter_mut().find(|s| s.id.as_str() == id)
    }

    pub fn tool(&self, id: &str) -> Option<&Tool> {
        self.tools.iter().find(|t| t.id.as_str() == id)
    }

    pub fn tool_mut(&mut self, id: &str) -> Option<&mut Tool> {
        self.tools.iter_mut().find(|t| t.id.as_str() == id)
    }

    pub fn operation(&self, id: &str) -> Option<&Operation> {
        self.operations.iter().find(|op| op.id.as_str() == id)
    }

    pub fn operation_mut(&mut self, id: &str) -> Option<&mut Operation> {
        self.operations.iter_mut().find(|op| op.id.as_str() == id)
    }

    pub fn program(&self, id: &str) -> Option<&NcProgram> {
        self.nc_programs.iter().find(|p| p.id == id)
    }

    /// The operations of setup `setup_id`, in block order.
    pub fn operations_of<'a>(&'a self, setup_id: &'a str) -> impl Iterator<Item = &'a Operation> + 'a {
        self.operations.iter().filter(move |op| op.setup.as_str() == setup_id)
    }

    /// Whether any record carries `id`.
    pub fn has(&self, id: &str) -> bool {
        self.setup(id).is_some()
            || self.tool(id).is_some()
            || self.operation(id).is_some()
            || self.program(id).is_some()
            || self.machines.iter().any(|m| m.id.as_str() == id)
    }

    /// Add a setup named `name` (`Setup N` when absent). Returns the id.
    pub fn add_setup(&mut self, name: Option<&str>) -> String {
        let id = self.next_id("SETUP");
        let name = name.map(str::to_string).unwrap_or_else(|| format!("Setup {}", self.setups.len() + 1));
        self.setups.push(Setup::new(id.as_str(), name));
        id
    }

    /// Add a flat end mill of the default geometry, named `name` (`Tool N`
    /// when absent), with the next free tool number. Returns the id.
    /// The machine `id` names: the document's own first, then the built-in library.
    pub fn machine(&self, id: &str) -> Option<Machine> {
        self.machines.iter().find(|m| m.id.as_str() == id).cloned().or_else(|| brep_cam::kinematics::library::builtin(id))
    }

    /// The resolved machine of setup `id`, when it names one.
    pub fn setup_machine(&self, id: &str) -> Option<Machine> {
        self.setup(id)?.machine.as_ref().and_then(|m| self.machine(m.as_str()))
    }

    /// Add a custom machine copied from `base` under a fresh id (`M5`). Returns the id.
    pub fn add_machine(&mut self, mut base: Machine, name: Option<&str>) -> String {
        let id = self.next_id("M");
        base.id = id.as_str().into();
        base.name = name.map(str::to_string).unwrap_or_else(|| format!("{} (custom)", base.name));
        self.machines.push(base);
        id
    }

    pub fn machine_mut(&mut self, id: &str) -> Option<&mut Machine> {
        self.machines.iter_mut().find(|m| m.id.as_str() == id)
    }

    pub fn add_tool(&mut self, name: Option<&str>) -> String {
        let id = self.next_id("T");
        let number = self.tools.iter().map(|t| t.number).max().unwrap_or(0) + 1;
        let mut tool = Tool::new(id.as_str(), number, ToolKind::Flat, ToolGeometry::default());
        tool.name = name.map(str::to_string).unwrap_or_else(|| format!("Tool {number}"));
        self.tools.push(tool);
        id
    }

    /// Add an operation of strategy kind `kind` to `setup` with `tool`, every
    /// parameter at its default; refused, naming what is missing, when the
    /// setup, the tool or the kind does not exist. Returns the id.
    pub fn add_operation(&mut self, setup: &str, tool: &str, kind: &str, name: Option<&str>) -> Result<String, String> {
        if self.setup(setup).is_none() {
            return Err(if self.setups.is_empty() {
                "no setup yet: press New setup first".to_string()
            } else {
                format!("no setup `{setup}`")
            });
        }
        if self.tool(tool).is_none() {
            return Err(if self.tools.is_empty() {
                "no tool yet: press New tool first".to_string()
            } else {
                format!("no tool `{tool}`")
            });
        }
        let strategy = strategy_of_kind(kind)?;
        let id = self.next_id("OP");
        let name = name.map(str::to_string).unwrap_or_else(|| format!("{} {}", strategy.kind_name(), self.operations.len() + 1));
        self.operations.push(Operation::new(id.as_str(), name, setup, tool, strategy));
        Ok(id)
    }

    /// Add an NC program named `name` (`Program N` when absent) over
    /// `operations`, each of which must exist. Returns the id.
    pub fn add_program(&mut self, name: Option<&str>, post: &str, operations: &[String]) -> Result<String, String> {
        for op in operations {
            if self.operation(op).is_none() {
                return Err(format!("no operation `{op}`"));
            }
        }
        // An empty post takes the default of the first operation's setup machine.
        let post = if post.trim().is_empty() {
            operations
                .first()
                .and_then(|op| self.operation(op))
                .and_then(|op| self.setup_machine(op.setup.as_str()))
                .and_then(|m| m.post)
                .unwrap_or_default()
        } else {
            post.to_string()
        };
        let post = post.as_str();
        let id = self.next_id("NC");
        let name = name.map(str::to_string).unwrap_or_else(|| format!("Program {}", self.nc_programs.len() + 1));
        self.nc_programs.push(NcProgram {
            id: id.clone(),
            name,
            number: None,
            post: post.to_string(),
            operations: operations.iter().map(|op| OperationId::new(op.as_str())).collect(),
            properties: BTreeMap::new(),
        });
        Ok(id)
    }

    /// Set (or, with a `null` value, clear) post properties of NC program
    /// `id`. A built-in post's names and enum values are checked against its
    /// own property list, so a misspelt or invalid one is refused by name
    /// here rather than at post time; a plugin post's are checked when it runs.
    pub fn set_program_properties(&mut self, id: &str, patch: &BTreeMap<String, Value>) -> Result<(), String> {
        let program = self.nc_programs.iter_mut().find(|p| p.id == id).ok_or_else(|| format!("no NC program `{id}`"))?;
        if let Some(post) = brep_cam::post::builtin(&program.post) {
            let definition = brep_cam::post::describe(post.source, post.id).map_err(|e| format!("post `{}`: {}", post.id, e.message))?;
            for (name, value) in patch {
                let Some(spec) = definition.properties.get(name) else {
                    return Err(format!("post `{}` has no property `{name}` (it has: {})", post.id, definition.properties.keys().cloned().collect::<Vec<_>>().join(", ")));
                };
                if let (Some(values), Some(v)) = (&spec.values, value.as_str()) {
                    if !values.iter().any(|x| x.as_str() == Some(v)) {
                        return Err(format!("post `{}` property `{name}` must be one of {}, not `{v}`", post.id, values.iter().map(|x| x.as_str().map(str::to_string).unwrap_or_else(|| x.to_string())).collect::<Vec<_>>().join(", ")));
                    }
                }
            }
        }
        for (name, value) in patch {
            if value.is_null() {
                program.properties.remove(name);
            } else {
                program.properties.insert(name.clone(), value.clone());
            }
        }
        Ok(())
    }

    /// The designated three-axis source of multi-axis finish `op_id`.
    pub fn multi_axis_source(&self, op_id: &str) -> Option<&str> {
        self.multi_axis_sources.get(op_id).map(String::as_str)
    }

    /// Designate (or with `None` clear) the source of `op_id`; the caller
    /// checks eligibility (`panels::cam_multiaxis::apply_source`).
    pub fn set_multi_axis_source(&mut self, op_id: &str, source: Option<String>) {
        match source {
            Some(s) => {
                self.multi_axis_sources.insert(op_id.to_string(), s);
            }
            None => {
                self.multi_axis_sources.remove(op_id);
            }
        }
    }

    /// Remove the record with `id`: a setup takes its operations with it; a
    /// tool in use by an operation is refused; an operation leaves every NC
    /// program that listed it.
    pub fn remove(&mut self, id: &str) -> Result<(), String> {
        if self.tool(id).is_some() && self.operations.iter().any(|op| op.tool.as_str() == id) {
            return Err(format!("tool `{id}` is used by an operation; remove the operation first"));
        }
        if self.machines.iter().any(|m| m.id.as_str() == id) {
            if let Some(setup) = self.setups.iter().find(|s| s.machine.as_ref().is_some_and(|m| m.as_str() == id)) {
                return Err(format!("machine `{id}` is used by setup `{}`; choose another machine there first", setup.id));
            }
        }
        if !self.has(id) {
            return Err(format!("no CAM record `{id}`"));
        }
        let gone: Vec<String> = if self.setup(id).is_some() {
            self.operations_of(id).map(|op| op.id.as_str().to_string()).chain(std::iter::once(id.to_string())).collect()
        } else {
            vec![id.to_string()]
        };
        self.setups.retain(|s| s.id.as_str() != id);
        self.orientations.remove(id);
        self.multi_axis_sources.retain(|finish, source| !gone.contains(finish) && !gone.contains(source));
        self.tools.retain(|t| t.id.as_str() != id);
        self.machines.retain(|m| m.id.as_str() != id);
        self.operations.retain(|op| !gone.contains(&op.id.as_str().to_string()));
        self.autoprogram_inputs.retain(|id, _| !gone.contains(id));
        self.nc_programs.retain(|p| p.id != id);
        for program in &mut self.nc_programs {
            program.operations.retain(|op| !gone.contains(&op.as_str().to_string()));
        }
        Ok(())
    }
}

/// Upgrade a block written by lane C1 row 1, before the model was imported:
/// an operation's `strategy` was its kind NAME and a tool carried only `id`,
/// `number` and `name`. The model's defaults cover the tool; the strategy
/// becomes the tagged object the model reads.
fn upgrade_row1(value: &mut Value) {
    if let Some(ops) = value.get_mut("operations").and_then(Value::as_array_mut) {
        for op in ops {
            if let Some(kind) = op.get("strategy").and_then(Value::as_str).map(str::to_string) {
                op["strategy"] = json!({ "kind": kind });
            }
        }
    }
}

/// A reference pick commits NAMES (`["Box"]`) at a setup's `targets` /
/// `protected` and an operation's `targets.driveFaces` / `avoidFaces`,
/// which the model holds as `TargetRef` objects; the names become
/// references on read (a body name a whole-body reference, a face name a
/// face reference whose owning solid the pane fills from the scene), so the
/// block stays the model's.
/// The key a chain pick commits its names under, inside an operation's
/// `targets`; the load folds it into `chains` (see [`chain_from_names`]).
pub const CHAIN_PICK_KEY: &str = "chainPick";

/// The key a machining-boundary pick commits its names under, inside an
/// operation's `targets`; the pane samples the picked sketch profile into the
/// strategy's containment boundary ([`crate::panels::cam::sync_operation_faces`])
/// and the rewritten block no longer carries it.
pub const BOUNDARY_PICK_KEY: &str = "boundaryPick";

/// The names a pick committed: a list, or one name for a single-valued field.
pub fn pick_names(value: &Value) -> Vec<String> {
    match value {
        Value::String(s) => vec![s.clone()],
        Value::Array(list) => list.iter().filter_map(Value::as_str).map(str::to_string).collect(),
        _ => Vec::new(),
    }
}

/// Whether the stored block still holds a committed chain pick the model has
/// not absorbed (the pane rewrites the block once, without an undo step).
pub fn has_pending_chain_pick(block: &Value) -> bool {
    block
        .get("operations")
        .and_then(Value::as_array)
        .is_some_and(|ops| ops.iter().any(|op| op.get("targets").and_then(|t| t.get(CHAIN_PICK_KEY)).is_some()))
}

/// The names a machining-boundary pick committed for operation `id` that the
/// model has not absorbed yet.
pub fn pending_boundary_pick(block: &Value, id: &str) -> Option<Vec<String>> {
    block
        .get("operations")?
        .as_array()?
        .iter()
        .find(|op| op.get("id").and_then(Value::as_str) == Some(id))?
        .get("targets")?
        .get(BOUNDARY_PICK_KEY)
        .map(pick_names)
}

/// The chain the picked entity `names` describe, in pick order: one sketch
/// profile face (`{sketch}:PROFILE` or `{sketch}:FACE`) is that sketch's
/// profile, every region; names that are all segments of ONE sketch
/// (`{sketch}:G{id}`) are its curves by bare id; anything else is a BREP
/// edge chain whose owning bodies the pane fills from the scene. `None` for
/// no names. Duplicates are dropped, keeping the first.
pub fn chain_from_names(names: &[String]) -> Option<ChainRef> {
    let names: Vec<&str> = names.iter().map(|s| s.trim()).filter(|s| !s.is_empty()).collect();
    if names.is_empty() {
        return None;
    }
    let profile = |n: &str| n.strip_suffix(":PROFILE").or_else(|| n.strip_suffix(":FACE")).filter(|s| !s.is_empty()).map(str::to_string);
    if names.len() == 1 {
        if let Some(sketch) = profile(names[0]) {
            return Some(ChainRef { source: ChainSource::SketchProfile { sketch, regions: Vec::new() }, closed: true, ..Default::default() });
        }
    }
    let curve = |n: &str| -> Option<(String, String)> {
        let (sketch, id) = n.rsplit_once(":G")?;
        (!sketch.is_empty() && !id.is_empty() && id.chars().all(|c| c.is_ascii_digit())).then(|| (sketch.to_string(), id.to_string()))
    };
    if let Some(parsed) = names.iter().map(|n| curve(n)).collect::<Option<Vec<_>>>() {
        let sketch = parsed[0].0.clone();
        if parsed.iter().all(|(s, _)| *s == sketch) {
            let mut curves: Vec<String> = Vec::new();
            for (_, id) in parsed {
                if !curves.contains(&id) {
                    curves.push(id);
                }
            }
            return Some(ChainRef { source: ChainSource::SketchCurves { sketch, curves }, ..Default::default() });
        }
    }
    let mut edges: Vec<EdgeRef> = Vec::new();
    for name in names {
        if !edges.iter().any(|e| e.edge == name) {
            edges.push(EdgeRef::new("", name));
        }
    }
    Some(ChainRef { source: ChainSource::Edges { edges }, ..Default::default() })
}

fn normalise_target_refs(value: &mut Value) {
    if let Some(setups) = value.get_mut("setups").and_then(Value::as_array_mut) {
        for setup in setups {
            for key in ["targets", "protected"] {
                if let Some(list) = setup.get_mut(key).and_then(Value::as_array_mut) {
                    for entry in list.iter_mut() {
                        if let Some(name) = entry.as_str().map(str::to_string) {
                            *entry = serde_json::to_value(TargetRef::solid(name)).unwrap_or(Value::Null);
                        }
                    }
                }
            }
        }
    }
    if let Some(operations) = value.get_mut("operations").and_then(Value::as_array_mut) {
        for operation in operations {
            // A committed chain pick (the picker writes the names it collected
            // under `targets.chainPick`) becomes one more chain.
            if let Some(targets) = operation.get_mut("targets").and_then(Value::as_object_mut) {
                if let Some(pick) = targets.remove(CHAIN_PICK_KEY) {
                    if let Some(chain) = chain_from_names(&pick_names(&pick)) {
                        let entry = serde_json::to_value(chain).unwrap_or(Value::Null);
                        match targets.get_mut("chains").and_then(Value::as_array_mut) {
                            Some(list) => list.push(entry),
                            None => {
                                targets.insert("chains".to_string(), json!([entry]));
                            }
                        }
                    }
                }
            }
            for key in ["driveFaces", "avoidFaces"] {
                if let Some(list) = operation.get_mut("targets").and_then(|t| t.get_mut(key)).and_then(Value::as_array_mut) {
                    for entry in list.iter_mut() {
                        if let Some(name) = entry.as_str().map(str::to_string) {
                            *entry = json!({ "solid": "", "face": name });
                        }
                    }
                }
            }
        }
    }
}

/// A single-valued pick with nothing picked commits an EMPTY name: an
/// orientation reference that is empty reads as no reference, so clearing a
/// field in the picker is the same as never having picked it.
fn normalise_orientations(value: &mut Value) {
    let Some(orientations) = value.get_mut("orientations").and_then(Value::as_object_mut) else {
        return;
    };
    for orientation in orientations.values_mut() {
        if let Some(fields) = orientation.as_object_mut() {
            fields.retain(|_, v| !matches!(v, Value::String(s) if s.trim().is_empty()));
        }
    }
}

/// Read the block (`None` = no block = empty). A block that does not parse
/// is an error naming why; the caller leaves it as it is.
pub fn load(block: Option<&Value>) -> Result<CamDocument, String> {
    let Some(block) = block else {
        return Ok(CamDocument::default());
    };
    let mut value = block.clone();
    upgrade_row1(&mut value);
    normalise_target_refs(&mut value);
    normalise_orientations(&mut value);
    serde_json::from_value(value).map_err(|e| format!("the document's `cam` block cannot be read: {e}"))
}

/// The block as the document stores it, or `None` when empty.
pub fn store(doc: &CamDocument) -> Option<Value> {
    if doc.is_empty() {
        return None;
    }
    serde_json::to_value(doc).ok()
}

