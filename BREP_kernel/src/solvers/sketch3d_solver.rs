//! 3D sketch constraint solver — the free-POINT sibling of `assembly_solver`.
//!
//! A 3D sketch is a set of world-space points, the geometry that references
//! them (points, lines, arcs, bezier chains, reference planes) and constraints
//! between those points. Unlike the assembly solver, whose unknowns are the
//! rigid-body POSES of parts, the unknowns here are the free points' `(x, y, z)`
//! coordinates: a sketch line must stretch, so it is not a rigid body, and a
//! point carries no rotation. What this solver shares with `assembly_solver`
//! is everything else — the residual vocabulary (coincident, distance,
//! point-on-plane, point-on-line, parallel, perpendicular, angle, tangent),
//! the Levenberg-Marquardt schedule (`assembly_solver::lm_core`: the same
//! central-difference Jacobian, the same Cholesky solve, the same λ up/down
//! schedule and non-worsening best-max guard — copied rather than shared so
//! the pinned assembly numerics never move under a sketch change), and the
//! diagnostics contract (`dof` / `rank` / `redundant` / `status` /
//! `conflicting`, plus per-point mobility from the Jacobian null space, the
//! shape `sketch_solver::compute_diagnostics` publishes for 2D sketches).
//!
//! # Document
//!
//! `{points, geometries, constraints}`, typed as [`Sketch3dDoc`]:
//!
//! - a point is `{id, x, y, z, fixed?, construction?, externalReference?}`;
//!   a `fixed` point is excluded from the unknowns (grounded);
//! - a geometry is `{id, type, points: [ids], construction?}` with
//!   `point = [p]`, `line = [a, b]`, `arc = [center, start, end]` (the minor
//!   arc from start to end about the center, in the plane of the three points),
//!   `bezier = [p0, c1, c2, p1, …]` (every 3 ids a new cubic span) and
//!   `plane = [origin, normalTip]` (a reference plane: its normal is
//!   `normalTip − origin`; always construction);
//! - a constraint is `{id, type, points: [ids], value?}`, see
//!   [`SKETCH3D_CONSTRAINT_TYPES`] for the vocabulary and the point layouts.
//!
//! An `arc` carries an IMPLIED equal-radius row (`‖start − center‖ =
//! ‖end − center‖`), and a multi-span `bezier` an implied G1 row at every
//! interior anchor (the two handles and the anchor collinear), exactly as the
//! 2D solver's temporary `⇌` / `⏛` constraints do. Implied rows count towards
//! rank and redundancy but are never named as conflicting.
//!
//! # Solve
//!
//! Levenberg-Marquardt on the damped normal equations over the free
//! coordinates; the step is in the row space of the Jacobian, so an
//! under-constrained sketch keeps its free degrees of freedom where the user
//! left them. The solved coordinates are written back into the document and
//! the diagnostics are returned alongside (see [`Sketch3dDiagnostics`]). A
//! constraint set with no solution near the current configuration comes back
//! as the least-squares best with `conflicting = true` and the violated
//! constraints named — the editor paints them, it does not lose them.

use crate::solver_linear_algebra::{row_reduce, solve_spd};
use crate::Vec3;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Tuning constants (mirroring assembly_solver's)
// ---------------------------------------------------------------------------

/// Maximum damping increases per outer iteration before abandoning the step.
const MAX_DAMPING: usize = 16;
/// Damping growth on a rejected step / Cholesky failure.
const LAMBDA_UP: f64 = 8.0;
/// Damping shrink after an accepted step.
const LAMBDA_DOWN: f64 = 0.25;
/// An accepted step below this (relative to scale) counts as converged.
const STEP_REL_TOL: f64 = 1e-13;
/// Central-difference step for the coordinate columns, times the model scale.
const FD_STEP: f64 = 1e-6;
/// Relative pivot tolerance for the numerical Jacobian rank.
const RANK_REL_TOL: f64 = 1e-7;
/// A null-space component below this (relative to the vector's max) does not
/// count as mobility.
const MOBILITY_TOL: f64 = 1e-6;
/// A residual row still this far (in length units, relative to scale) from
/// zero once the solve has come to rest is a conflict.
const CONFLICT_REL_TOL: f64 = 1e-6;
/// Directions shorter than this (relative to scale) carry no orientation.
const DIR_EPS_REL: f64 = 1e-12;

fn default_tolerance() -> f64 {
    1e-10
}

fn default_max_iterations() -> usize {
    200
}

// ---------------------------------------------------------------------------
// Document types
// ---------------------------------------------------------------------------

/// A 3D sketch point: world coordinates plus the role flags the solver tracks.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Sketch3dPoint {
    /// Stable point id (a number in practice; strings are legal — keyed by
    /// [`crate::sketch_id_key`]).
    pub id: Value,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Grounded: excluded from the unknowns, never moves.
    #[serde(default)]
    pub fixed: bool,
    /// Construction point: constrains but never models an edge or a vertex.
    #[serde(default)]
    pub construction: bool,
    /// Adopted from a model reference (a picked vertex, an edge end, a plane
    /// origin); re-resolved on every feature run.
    #[serde(default, rename = "externalReference")]
    pub external_reference: bool,
}

impl Sketch3dPoint {
    pub fn position(&self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }
}

/// A 3D sketch geometry (see the module doc for the point layouts).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Sketch3dGeometry {
    pub id: Value,
    #[serde(rename = "type")]
    pub geom_type: String,
    #[serde(default)]
    pub points: Vec<Value>,
    /// Every field not modelled explicitly (`construction`, authoring
    /// metadata) rides along so a load/save is lossless.
    #[serde(flatten)]
    pub extra: Map<String, Value>,
}

impl Sketch3dGeometry {
    /// Whether this geometry is construction-only (never a model edge). A
    /// reference `plane` is construction by nature.
    pub fn construction(&self) -> bool {
        self.geom_type == "plane"
            || self
                .extra
                .get("construction")
                .and_then(Value::as_bool)
                .unwrap_or(false)
    }
}

/// A 3D sketch constraint, kept as a transparent map so authoring metadata
/// (expression text, label offsets) persists exactly.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(transparent)]
pub struct Sketch3dConstraint {
    pub raw: Map<String, Value>,
}

impl Sketch3dConstraint {
    pub fn ctype(&self) -> Option<&str> {
        self.raw.get("type").and_then(Value::as_str)
    }

    pub fn id(&self) -> Option<&Value> {
        self.raw.get("id")
    }

    pub fn points(&self) -> &[Value] {
        self.raw
            .get("points")
            .and_then(Value::as_array)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    /// The dimensional value (a number; expression strings are evaluated by
    /// the feature before the solver sees the doc).
    pub fn value(&self) -> Option<f64> {
        self.raw.get("value").and_then(Value::as_f64)
    }
}

/// The editable 3D sketch document.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Sketch3dDoc {
    #[serde(default)]
    pub points: Vec<Sketch3dPoint>,
    #[serde(default)]
    pub geometries: Vec<Sketch3dGeometry>,
    #[serde(default)]
    pub constraints: Vec<Sketch3dConstraint>,
}

impl Sketch3dDoc {
    pub fn point(&self, id: &Value) -> Option<&Sketch3dPoint> {
        let key = id_key(id);
        self.points.iter().find(|p| id_key(&p.id) == key)
    }

    pub fn point_mut(&mut self, id: &Value) -> Option<&mut Sketch3dPoint> {
        let key = id_key(id);
        self.points.iter_mut().find(|p| id_key(&p.id) == key)
    }

    pub fn geometry(&self, id: &Value) -> Option<&Sketch3dGeometry> {
        let key = id_key(id);
        self.geometries.iter().find(|g| id_key(&g.id) == key)
    }

    pub fn geometry_mut(&mut self, id: &Value) -> Option<&mut Sketch3dGeometry> {
        let key = id_key(id);
        self.geometries.iter_mut().find(|g| id_key(&g.id) == key)
    }

    pub fn constraint(&self, id: &Value) -> Option<&Sketch3dConstraint> {
        let key = id_key(id);
        self.constraints
            .iter()
            .find(|c| c.id().map(id_key) == Some(key.clone()))
    }

    /// Mint the next unused POINT id (`max numeric id + 1`).
    pub fn next_point_id(&self) -> Value {
        mint_next_id(self.points.iter().map(|p| &p.id))
    }

    /// Mint the next unused GEOMETRY id.
    pub fn next_geometry_id(&self) -> Value {
        mint_next_id(self.geometries.iter().map(|g| &g.id))
    }

    /// Mint the next unused CONSTRAINT id.
    pub fn next_constraint_id(&self) -> Value {
        mint_next_id(self.constraints.iter().filter_map(|c| c.id()))
    }

    /// Add a free point at `p` and return its id.
    pub fn add_point(&mut self, p: Vec3) -> Value {
        let id = self.next_point_id();
        self.points.push(Sketch3dPoint {
            id: id.clone(),
            x: p.x,
            y: p.y,
            z: p.z,
            fixed: false,
            construction: false,
            external_reference: false,
        });
        id
    }

    /// Return the id of the existing point within `radius` of `p` (the nearest
    /// one), else mint a new free point there.
    pub fn snap_or_add_point(&mut self, p: Vec3, radius: f64) -> Value {
        let mut best: Option<(f64, Value)> = None;
        for point in &self.points {
            let d = point.position().sub(p).length();
            if d <= radius && best.as_ref().map_or(true, |(bd, _)| d < *bd) {
                best = Some((d, point.id.clone()));
            }
        }
        match best {
            Some((_, id)) => id,
            None => self.add_point(p),
        }
    }

    /// Add a geometry of `geom_type` over `points` and return its id.
    pub fn add_geometry(&mut self, geom_type: &str, points: Vec<Value>, construction: bool) -> Value {
        let id = self.next_geometry_id();
        let mut extra = Map::new();
        if construction {
            extra.insert("construction".into(), Value::Bool(true));
        }
        self.geometries.push(Sketch3dGeometry {
            id: id.clone(),
            geom_type: geom_type.to_string(),
            points,
            extra,
        });
        id
    }

    /// Add a constraint and return its id. `value` is the dimension for the
    /// dimensional types (`None` for the geometric ones).
    pub fn add_constraint(&mut self, ctype: &str, points: Vec<Value>, value: Option<f64>) -> Value {
        let id = self.next_constraint_id();
        let mut raw = Map::new();
        raw.insert("id".into(), id.clone());
        raw.insert("type".into(), Value::String(ctype.to_string()));
        raw.insert("points".into(), Value::Array(points));
        if let Some(value) = value {
            raw.insert("value".into(), Value::from(value));
        }
        self.constraints.push(Sketch3dConstraint { raw });
        id
    }

    /// The point ids a geometry carries, resolved to positions (`None` when any
    /// id is missing).
    pub fn geometry_positions(&self, geometry: &Sketch3dGeometry) -> Option<Vec<Vec3>> {
        geometry
            .points
            .iter()
            .map(|id| self.point(id).map(Sketch3dPoint::position))
            .collect()
    }
}

/// Shared solver key for sketch entity and mobility-map lookups.
pub use crate::sketch_solver::sketch_id_key as id_key;

fn id_num(value: &Value) -> Option<f64> {
    match value {
        Value::Number(number) => number.as_f64(),
        Value::String(text) => text.trim().parse::<f64>().ok(),
        _ => None,
    }
}

fn mint_next_id<'a>(ids: impl Iterator<Item = &'a Value>) -> Value {
    let ids: Vec<&Value> = ids.collect();
    let mut max: Option<i64> = None;
    for &id in &ids {
        if let Some(n) = id_num(id) {
            if n.is_finite() {
                let i = n.floor() as i64;
                max = Some(max.map_or(i, |m| m.max(i)));
            }
        }
    }
    let mut candidate = max.map(|m| m + 1).unwrap_or(ids.len() as i64);
    let existing: std::collections::HashSet<String> = ids.iter().map(|&id| id_key(id)).collect();
    while existing.contains(&id_key(&Value::from(candidate))) {
        candidate += 1;
    }
    Value::from(candidate)
}

// ---------------------------------------------------------------------------
// Constraint vocabulary
// ---------------------------------------------------------------------------

/// One constraint type: its `type` string, how many point ids it takes, whether
/// it carries a `value`, and a one-line meaning (the editor's palette reads
/// these; the solver dispatches on the string).
#[derive(Clone, Copy, Debug)]
pub struct Sketch3dConstraintType {
    pub name: &'static str,
    /// The point-id layout: how many ids `points` carries, in which roles.
    pub arity: usize,
    /// Whether the constraint carries a dimensional `value`.
    pub dimensional: bool,
    pub meaning: &'static str,
}

/// The constraint vocabulary. Point layouts: a LINE is its two endpoints
/// `[a0, a1]` (direction `a1 − a0`); a PLANE is `[origin, normalTip]`
/// (normal `normalTip − origin`); an ARC is `[center, start, end]`.
pub const SKETCH3D_CONSTRAINT_TYPES: &[Sketch3dConstraintType] = &[
    Sketch3dConstraintType { name: "fixed", arity: 1, dimensional: false, meaning: "ground a point where it is" },
    Sketch3dConstraintType { name: "coincident", arity: 2, dimensional: false, meaning: "two points touch" },
    Sketch3dConstraintType { name: "distance", arity: 2, dimensional: true, meaning: "distance between two points (a line's length)" },
    Sketch3dConstraintType { name: "onLine", arity: 3, dimensional: false, meaning: "[p, a0, a1]: point on the infinite line a0→a1" },
    Sketch3dConstraintType { name: "lineDistance", arity: 3, dimensional: true, meaning: "[p, a0, a1]: distance from the point to the infinite line" },
    Sketch3dConstraintType { name: "onPlane", arity: 3, dimensional: false, meaning: "[p, origin, normalTip]: point on the plane" },
    Sketch3dConstraintType { name: "planeDistance", arity: 3, dimensional: true, meaning: "[p, origin, normalTip]: signed height of the point above the plane" },
    Sketch3dConstraintType { name: "parallel", arity: 4, dimensional: false, meaning: "[a0, a1, b0, b1]: the two directions are parallel (either sense)" },
    Sketch3dConstraintType { name: "perpendicular", arity: 4, dimensional: false, meaning: "[a0, a1, b0, b1]: the two directions are perpendicular" },
    Sketch3dConstraintType { name: "angle", arity: 4, dimensional: true, meaning: "[a0, a1, b0, b1]: angle in degrees between the two directions" },
    Sketch3dConstraintType { name: "alongX", arity: 2, dimensional: false, meaning: "the line runs along world X" },
    Sketch3dConstraintType { name: "alongY", arity: 2, dimensional: false, meaning: "the line runs along world Y" },
    Sketch3dConstraintType { name: "alongZ", arity: 2, dimensional: false, meaning: "the line runs along world Z" },
    Sketch3dConstraintType { name: "equal", arity: 4, dimensional: false, meaning: "[a0, a1, b0, b1]: equal lengths (an arc's radius is center→start)" },
    Sketch3dConstraintType { name: "tangent", arity: 5, dimensional: false, meaning: "[a0, a1, center, start, end]: the line is tangent to the arc at the arc end it shares" },
];

/// Look up a constraint type by its `type` string.
pub fn sketch3d_constraint_type(name: &str) -> Option<&'static Sketch3dConstraintType> {
    SKETCH3D_CONSTRAINT_TYPES.iter().find(|t| t.name == name)
}

// ---------------------------------------------------------------------------
// Request / options / diagnostics
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Sketch3dSolveOptions {
    /// Convergence target for the maximum residual, relative to the sketch's
    /// length scale.
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
    /// Maximum accepted Levenberg-Marquardt steps.
    #[serde(default = "default_max_iterations")]
    pub max_iterations: usize,
}

impl Default for Sketch3dSolveOptions {
    fn default() -> Self {
        Self {
            tolerance: default_tolerance(),
            max_iterations: default_max_iterations(),
        }
    }
}

/// The JSON request: the document plus optional solver options.
#[derive(Debug, Deserialize)]
pub struct SolveSketch3dRequest {
    pub sketch: Value,
    #[serde(default)]
    pub tolerance: Option<f64>,
    #[serde(default)]
    pub max_iterations: Option<usize>,
}

/// The solver's read-only diagnostics: the same shape the 2D sketch solver
/// publishes (`dof`, `rank`, `unknowns`, `equations`, `redundant`, `status`,
/// `conflicting`, `conflictingConstraints`, `pointMobility`,
/// `geometryMobility`), so the editor's readouts are shared.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Sketch3dDiagnostics {
    #[serde(default)]
    pub dof: i64,
    #[serde(default)]
    pub rank: i64,
    #[serde(default)]
    pub unknowns: i64,
    #[serde(default)]
    pub equations: i64,
    #[serde(default)]
    pub redundant: i64,
    /// `"well"`, `"under"` or `"over"`.
    #[serde(default)]
    pub status: String,
    #[serde(default)]
    pub conflicting: bool,
    /// The constraints (as [`id_key`] strings) still violated at the
    /// least-squares rest point. Empty exactly when `conflicting` is clear.
    #[serde(default, rename = "conflictingConstraints")]
    pub conflicting_constraints: Vec<String>,
    /// `id_key -> "movable" | "locked"` per point.
    #[serde(default, rename = "pointMobility")]
    pub point_mobility: std::collections::BTreeMap<String, String>,
    #[serde(default, rename = "geometryMobility")]
    pub geometry_mobility: std::collections::BTreeMap<String, String>,
    /// Accepted Levenberg-Marquardt steps.
    #[serde(default)]
    pub iterations: i64,
    /// The worst residual row at the solution, in length units.
    #[serde(default, rename = "maxResidual")]
    pub max_residual: f64,
}

impl Sketch3dDiagnostics {
    pub fn point_movable(&self, id: &Value) -> Option<bool> {
        self.point_mobility.get(&id_key(id)).map(|v| v == "movable")
    }

    pub fn geometry_movable(&self, id: &Value) -> Option<bool> {
        self.geometry_mobility.get(&id_key(id)).map(|v| v == "movable")
    }

    pub fn constraint_conflicting(&self, id: &Value) -> bool {
        if self.conflicting_constraints.is_empty() {
            return false;
        }
        let key = id_key(id);
        self.conflicting_constraints.iter().any(|c| *c == key)
    }
}

// ---------------------------------------------------------------------------
// Residual atoms over world points
// ---------------------------------------------------------------------------

/// One residual atom over point INDICES into the coordinate array. `rows()`
/// is the number of scalar rows; `eff_rows()` the atom's rank at a solution
/// (a 3-row cross-product block only ever removes 2 DOF).
#[derive(Clone, Copy, Debug)]
enum Atom {
    /// `a − b` (3 rows, eff 3).
    Coincide(usize, usize),
    /// `‖a − b‖ − target` (1 row).
    Distance(usize, usize, f64),
    /// Component of `p − o` perpendicular to `q − o` (3 rows, eff 2): point on
    /// the infinite line `o → q`.
    Lateral(usize, usize, usize),
    /// `‖(p − o) ⊥ (q − o)‖ − target` (1 row).
    LineDistance(usize, usize, usize, f64),
    /// `n̂·(p − o) − target` with `n̂ = unit(t − o)` (1 row).
    PlaneOffset(usize, usize, usize, f64),
    /// `unit(a1 − a0) × unit(b1 − b0)` (3 rows, eff 2).
    DirCross(usize, usize, usize, usize),
    /// `unit(a1 − a0) · unit(b1 − b0) − target` (1 row).
    DirDot(usize, usize, usize, usize, f64),
    /// `unit(a1 − a0) × axis` (3 rows, eff 2).
    AxisCross(usize, usize, Vec3),
    /// `‖a1 − a0‖ − ‖b1 − b0‖` (1 row).
    EqualLength(usize, usize, usize, usize),
    /// `unit(t − c) · unit(a1 − a0)` (1 row): the line is perpendicular to
    /// the arc radius at the touch point `t`.
    TangentTouch(usize, usize, usize, usize),
    /// `unit(a1 − a0) · unit((s − c) × (e − c))` (1 row): the line lies in
    /// the arc's plane.
    InArcPlane(usize, usize, usize, usize, usize),
}

impl Atom {
    fn rows(&self) -> usize {
        match self {
            Atom::Coincide(..) | Atom::Lateral(..) | Atom::DirCross(..) | Atom::AxisCross(..) => 3,
            Atom::Distance(..)
            | Atom::LineDistance(..)
            | Atom::PlaneOffset(..)
            | Atom::DirDot(..)
            | Atom::EqualLength(..)
            | Atom::TangentTouch(..)
            | Atom::InArcPlane(..) => 1,
        }
    }

    fn eff_rows(&self) -> usize {
        match self {
            Atom::Coincide(..) => 3,
            Atom::Lateral(..) | Atom::DirCross(..) | Atom::AxisCross(..) => 2,
            _ => 1,
        }
    }

    fn eval(&self, p: &[Vec3], dir_eps: f64, out: &mut Vec<f64>) {
        let unit = |v: Vec3| {
            let len = v.length();
            if len > dir_eps {
                v.scale(1.0 / len)
            } else {
                v.scale(1.0 / dir_eps)
            }
        };
        match *self {
            Atom::Coincide(a, b) => {
                let d = p[a].sub(p[b]);
                out.extend([d.x, d.y, d.z]);
            }
            Atom::Distance(a, b, target) => out.push(p[a].sub(p[b]).length() - target),
            Atom::Lateral(pt, o, q) => {
                let d = unit(p[q].sub(p[o]));
                let v = p[pt].sub(p[o]);
                let lateral = v.sub(d.scale(d.dot(v)));
                out.extend([lateral.x, lateral.y, lateral.z]);
            }
            Atom::LineDistance(pt, o, q, target) => {
                let d = unit(p[q].sub(p[o]));
                let v = p[pt].sub(p[o]);
                let lateral = v.sub(d.scale(d.dot(v)));
                out.push(lateral.length() - target);
            }
            Atom::PlaneOffset(pt, o, t, target) => {
                let n = unit(p[t].sub(p[o]));
                out.push(n.dot(p[pt].sub(p[o])) - target);
            }
            Atom::DirCross(a0, a1, b0, b1) => {
                let c = unit(p[a1].sub(p[a0])).cross(unit(p[b1].sub(p[b0])));
                out.extend([c.x, c.y, c.z]);
            }
            Atom::DirDot(a0, a1, b0, b1, target) => {
                out.push(unit(p[a1].sub(p[a0])).dot(unit(p[b1].sub(p[b0]))) - target);
            }
            Atom::AxisCross(a0, a1, axis) => {
                let c = unit(p[a1].sub(p[a0])).cross(axis);
                out.extend([c.x, c.y, c.z]);
            }
            Atom::EqualLength(a0, a1, b0, b1) => {
                out.push(p[a1].sub(p[a0]).length() - p[b1].sub(p[b0]).length());
            }
            Atom::TangentTouch(a0, a1, c, t) => {
                out.push(unit(p[t].sub(p[c])).dot(unit(p[a1].sub(p[a0]))));
            }
            Atom::InArcPlane(a0, a1, c, s, e) => {
                let n = unit(p[s].sub(p[c]).cross(p[e].sub(p[c])));
                out.push(unit(p[a1].sub(p[a0])).dot(n));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Problem preparation
// ---------------------------------------------------------------------------

struct Prepared {
    /// Every point's coordinates (fixed ones never change).
    coords: Vec<Vec3>,
    /// Indices of the free points, ascending.
    free: Vec<usize>,
    atoms: Vec<Atom>,
    /// Atom → owning constraint index, or `None` for an implied row.
    atom_owner: Vec<Option<usize>>,
    m: usize,
    effective_rows: usize,
    scale: f64,
    tight: f64,
}

fn prepare(doc: &Sketch3dDoc, options: &Sketch3dSolveOptions) -> Result<Prepared, String> {
    let mut index: HashMap<String, usize> = HashMap::with_capacity(doc.points.len());
    let mut coords: Vec<Vec3> = Vec::with_capacity(doc.points.len());
    let mut fixed: Vec<bool> = Vec::with_capacity(doc.points.len());
    for (i, point) in doc.points.iter().enumerate() {
        if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite()) {
            return Err(format!("sketch3d: point {} has a non-finite coordinate", id_key(&point.id)));
        }
        // A duplicate id: the later one wins (matches the 2D solver's map).
        index.insert(id_key(&point.id), i);
        coords.push(point.position());
        fixed.push(point.fixed);
    }
    let lookup = |what: &str, id: &Value| -> Result<usize, String> {
        index
            .get(&id_key(id))
            .copied()
            .ok_or_else(|| format!("sketch3d: {what} references unknown point {}", id_key(id)))
    };

    let mut atoms: Vec<Atom> = Vec::new();
    let mut atom_owner: Vec<Option<usize>> = Vec::new();

    // User constraints.
    for (ci, constraint) in doc.constraints.iter().enumerate() {
        let Some(ctype) = constraint.ctype() else {
            return Err(format!("sketch3d: constraint {ci} has no type"));
        };
        let spec = sketch3d_constraint_type(ctype)
            .ok_or_else(|| format!("sketch3d: unknown constraint type '{ctype}'"))?;
        let ids = constraint.points();
        if ids.len() < spec.arity {
            return Err(format!(
                "sketch3d: constraint '{ctype}' needs {} points, got {}",
                spec.arity,
                ids.len()
            ));
        }
        let what = format!("constraint '{ctype}'");
        let mut idx = Vec::with_capacity(spec.arity);
        for id in &ids[..spec.arity] {
            idx.push(lookup(&what, id)?);
        }
        let value = || -> Result<f64, String> {
            constraint
                .value()
                .filter(|v| v.is_finite())
                .ok_or_else(|| format!("sketch3d: constraint '{ctype}' needs a finite value"))
        };
        let mut push = |atom: Atom| {
            atoms.push(atom);
            atom_owner.push(Some(ci));
        };
        match ctype {
            "fixed" => fixed[idx[0]] = true,
            "coincident" => push(Atom::Coincide(idx[0], idx[1])),
            "distance" => {
                let target = value()?;
                if target.abs() <= 0.0 {
                    push(Atom::Coincide(idx[0], idx[1]));
                } else {
                    push(Atom::Distance(idx[0], idx[1], target.abs()));
                }
            }
            "onLine" => push(Atom::Lateral(idx[0], idx[1], idx[2])),
            "lineDistance" => {
                let target = value()?;
                if target.abs() <= 0.0 {
                    push(Atom::Lateral(idx[0], idx[1], idx[2]));
                } else {
                    push(Atom::LineDistance(idx[0], idx[1], idx[2], target.abs()));
                }
            }
            "onPlane" => push(Atom::PlaneOffset(idx[0], idx[1], idx[2], 0.0)),
            "planeDistance" => push(Atom::PlaneOffset(idx[0], idx[1], idx[2], value()?)),
            "parallel" => push(Atom::DirCross(idx[0], idx[1], idx[2], idx[3])),
            "perpendicular" => push(Atom::DirDot(idx[0], idx[1], idx[2], idx[3], 0.0)),
            "angle" => {
                let target = value()?.to_radians().cos();
                push(Atom::DirDot(idx[0], idx[1], idx[2], idx[3], target));
            }
            "alongX" => push(Atom::AxisCross(idx[0], idx[1], Vec3::new(1.0, 0.0, 0.0))),
            "alongY" => push(Atom::AxisCross(idx[0], idx[1], Vec3::new(0.0, 1.0, 0.0))),
            "alongZ" => push(Atom::AxisCross(idx[0], idx[1], Vec3::new(0.0, 0.0, 1.0))),
            "equal" => push(Atom::EqualLength(idx[0], idx[1], idx[2], idx[3])),
            "tangent" => {
                // The touch point is the arc end the line shares (by id, else
                // the nearer one): [a0, a1, center, start, end].
                let (a0, a1, c, s, e) = (idx[0], idx[1], idx[2], idx[3], idx[4]);
                let touch = if s == a0 || s == a1 {
                    s
                } else if e == a0 || e == a1 {
                    e
                } else {
                    let near = |t: usize| {
                        coords[t]
                            .sub(coords[a0])
                            .length()
                            .min(coords[t].sub(coords[a1]).length())
                    };
                    if near(s) <= near(e) {
                        s
                    } else {
                        e
                    }
                };
                push(Atom::TangentTouch(a0, a1, c, touch));
                push(Atom::InArcPlane(a0, a1, c, s, e));
            }
            other => return Err(format!("sketch3d: unknown constraint type '{other}'")),
        }
    }

    // Implied rows: arc radius equality, bezier G1 at interior anchors.
    for geometry in &doc.geometries {
        match geometry.geom_type.as_str() {
            "arc" if geometry.points.len() >= 3 => {
                let what = "arc geometry";
                let c = lookup(what, &geometry.points[0])?;
                let s = lookup(what, &geometry.points[1])?;
                let e = lookup(what, &geometry.points[2])?;
                atoms.push(Atom::EqualLength(c, s, c, e));
                atom_owner.push(None);
            }
            "bezier" if geometry.points.len() >= 7 => {
                let what = "bezier geometry";
                let ids = &geometry.points;
                let span_count = (ids.len() - 1) / 3;
                let last_anchor = span_count * 3;
                let mut i = 3usize;
                while i < last_anchor {
                    let prev = lookup(what, &ids[i - 1])?;
                    let anchor = lookup(what, &ids[i])?;
                    let next = lookup(what, &ids[i + 1])?;
                    if prev != anchor && next != anchor && prev != next {
                        atoms.push(Atom::Lateral(anchor, prev, next));
                        atom_owner.push(None);
                    }
                    i += 3;
                }
            }
            _ => {}
        }
    }

    let free: Vec<usize> = (0..coords.len()).filter(|&i| !fixed[i]).collect();
    let m: usize = atoms.iter().map(Atom::rows).sum();
    let effective_rows: usize = atoms.iter().map(Atom::eff_rows).sum();

    let mut scale = 1.0f64;
    for c in &coords {
        scale = scale.max(c.x.abs()).max(c.y.abs()).max(c.z.abs());
    }
    let tight = options.tolerance * scale;
    Ok(Prepared {
        coords,
        free,
        atoms,
        atom_owner,
        m,
        effective_rows,
        scale,
        tight,
    })
}

// ---------------------------------------------------------------------------
// Numerics (mirroring assembly_solver::numerics)
// ---------------------------------------------------------------------------

fn apply_delta(base: &[Vec3], free: &[usize], delta: &[f64], out: &mut Vec<Vec3>) {
    out.clear();
    out.extend_from_slice(base);
    for (k, &i) in free.iter().enumerate() {
        out[i] = base[i].add(Vec3::new(delta[3 * k], delta[3 * k + 1], delta[3 * k + 2]));
    }
}

fn eval_residuals(
    atoms: &[Atom],
    base: &[Vec3],
    free: &[usize],
    delta: &[f64],
    dir_eps: f64,
    scratch: &mut Vec<Vec3>,
    out: &mut Vec<f64>,
) {
    apply_delta(base, free, delta, scratch);
    out.clear();
    for atom in atoms {
        atom.eval(scratch, dir_eps, out);
    }
}

fn finite_diff_jacobian(
    atoms: &[Atom],
    base: &[Vec3],
    free: &[usize],
    h: f64,
    dir_eps: f64,
    m: usize,
) -> Vec<Vec<f64>> {
    let n = 3 * free.len();
    let mut jac = vec![vec![0.0f64; n]; m];
    if n == 0 || m == 0 {
        return jac;
    }
    let mut delta = vec![0.0f64; n];
    let mut scratch: Vec<Vec3> = Vec::with_capacity(base.len());
    let mut rp: Vec<f64> = Vec::with_capacity(m);
    let mut rm: Vec<f64> = Vec::with_capacity(m);
    let inv = 1.0 / (2.0 * h);
    for col in 0..n {
        delta[col] = h;
        eval_residuals(atoms, base, free, &delta, dir_eps, &mut scratch, &mut rp);
        delta[col] = -h;
        eval_residuals(atoms, base, free, &delta, dir_eps, &mut scratch, &mut rm);
        delta[col] = 0.0;
        for i in 0..m {
            jac[i][col] = (rp[i] - rm[i]) * inv;
        }
    }
    jac
}

/// Levenberg-Marquardt over the free coordinates (`assembly_solver::lm_core`'s
/// loop with point increments in place of pose increments). On return `coords`
/// holds the best (max-residual) iterate found. Returns the accepted steps.
fn lm_core(prep: &Prepared, coords: &mut Vec<Vec3>, max_iterations: usize) -> Result<usize, String> {
    let atoms = &prep.atoms;
    let free = &prep.free;
    let n = 3 * free.len();
    let m = prep.m;
    let scale = prep.scale;
    let dir_eps = DIR_EPS_REL * scale;
    let h = FD_STEP * scale;
    let step_tol = STEP_REL_TOL * scale;
    let tight = prep.tight;

    let max_abs = |v: &[f64]| v.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
    let sum_sq = |v: &[f64]| v.iter().fold(0.0f64, |acc, &x| acc + x * x);

    let zeros = vec![0.0f64; n];
    let mut scratch: Vec<Vec3> = Vec::with_capacity(coords.len());
    let mut r: Vec<f64> = Vec::with_capacity(m);
    eval_residuals(atoms, coords, free, &zeros, dir_eps, &mut scratch, &mut r);
    if !max_abs(&r).is_finite() {
        return Err("sketch3d: non-finite residual at the initial coordinates".into());
    }

    let mut iterations = 0usize;
    let mut s = sum_sq(&r);
    let mut best_coords = coords.clone();
    let mut best_max = max_abs(&r);
    let mut lambda = -1.0f64;

    if n > 0 && m > 0 {
        'outer: for _iter in 0..max_iterations {
            if best_max <= tight {
                break;
            }
            let jac = finite_diff_jacobian(atoms, coords, free, h, dir_eps, m);
            let mut hm = vec![vec![0.0f64; n]; n];
            let mut g = vec![0.0f64; n];
            for i in 0..m {
                let row = &jac[i];
                let ri = r[i];
                for a in 0..n {
                    let ja = row[a];
                    if ja == 0.0 {
                        continue;
                    }
                    g[a] += ja * ri;
                    for b in a..n {
                        hm[a][b] += ja * row[b];
                    }
                }
            }
            for a in 0..n {
                for b in (a + 1)..n {
                    hm[b][a] = hm[a][b];
                }
            }
            let mut max_diag = 0.0f64;
            for a in 0..n {
                max_diag = max_diag.max(hm[a][a]);
            }
            if !(max_diag > 0.0) {
                break;
            }
            if lambda < 0.0 {
                lambda = 1e-9 * max_diag;
            }
            let lambda_ceiling = 1e12 * max_diag;

            let mut stepped = false;
            let mut step_small = false;
            for _try in 0..MAX_DAMPING {
                let mut a_mat = hm.clone();
                for d in 0..n {
                    a_mat[d][d] += lambda;
                }
                let mut delta: Vec<f64> = g.iter().map(|v| -v).collect();
                if !solve_spd(&mut a_mat, &mut delta) {
                    lambda *= LAMBDA_UP;
                    if lambda > lambda_ceiling {
                        break;
                    }
                    continue;
                }
                let mut r_trial: Vec<f64> = Vec::with_capacity(m);
                eval_residuals(atoms, coords, free, &delta, dir_eps, &mut scratch, &mut r_trial);
                let s_trial = sum_sq(&r_trial);
                if s_trial.is_finite() && s_trial < s {
                    let step_norm = sum_sq(&delta).sqrt();
                    apply_delta(coords, free, &delta, &mut scratch);
                    coords.copy_from_slice(&scratch);
                    eval_residuals(atoms, coords, free, &zeros, dir_eps, &mut scratch, &mut r);
                    s = sum_sq(&r);
                    let m_now = max_abs(&r);
                    if m_now < best_max {
                        best_max = m_now;
                        best_coords.copy_from_slice(coords);
                    }
                    lambda = (lambda * LAMBDA_DOWN).max(1e-12 * max_diag);
                    iterations += 1;
                    stepped = true;
                    step_small = step_norm <= step_tol;
                    break;
                }
                lambda *= LAMBDA_UP;
                if lambda > lambda_ceiling {
                    break;
                }
            }
            if !stepped || step_small {
                break 'outer;
            }
        }
    }
    coords.copy_from_slice(&best_coords);
    Ok(iterations)
}

/// Numerical rank + a null-space basis of a dense `m × n` matrix (the 2D
/// solver's `rank_and_nullspace`).
fn rank_and_nullspace(mut a: Vec<Vec<f64>>, m: usize, n: usize) -> (usize, Vec<Vec<f64>>) {
    if n == 0 {
        return (0, Vec::new());
    }
    let mut maxabs = 0.0f64;
    for row in &a {
        for &v in row {
            maxabs = maxabs.max(v.abs());
        }
    }
    let tol = if maxabs > 0.0 {
        RANK_REL_TOL * maxabs
    } else {
        RANK_REL_TOL
    };
    let mut pivot_cols: Vec<usize> = Vec::new();
    row_reduce(&mut a, m, n, tol, |col| pivot_cols.push(col));
    let rank = pivot_cols.len();
    let mut is_pivot = vec![false; n];
    for &pc in &pivot_cols {
        is_pivot[pc] = true;
    }
    let mut basis: Vec<Vec<f64>> = Vec::new();
    for free_col in 0..n {
        if is_pivot[free_col] {
            continue;
        }
        let mut v = vec![0.0f64; n];
        v[free_col] = 1.0;
        for (i, &pc) in pivot_cols.iter().enumerate() {
            v[pc] = -a[i][free_col];
        }
        basis.push(v);
    }
    (rank, basis)
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

/// Solve `doc` in place: the free points take their solved coordinates and the
/// diagnostics describe the constraint system at that configuration.
pub fn solve_sketch3d_doc(
    doc: &mut Sketch3dDoc,
    options: &Sketch3dSolveOptions,
) -> Result<Sketch3dDiagnostics, String> {
    let prep = prepare(doc, options)?;
    let mut coords = prep.coords.clone();
    let iterations = lm_core(&prep, &mut coords, options.max_iterations)?;

    let n = 3 * prep.free.len();
    let m = prep.m;
    let dir_eps = DIR_EPS_REL * prep.scale;
    let zeros = vec![0.0f64; n];
    let mut scratch: Vec<Vec3> = Vec::with_capacity(coords.len());
    let mut r: Vec<f64> = Vec::with_capacity(m);
    eval_residuals(&prep.atoms, &coords, &prep.free, &zeros, dir_eps, &mut scratch, &mut r);

    let jac = finite_diff_jacobian(&prep.atoms, &coords, &prep.free, FD_STEP * prep.scale, dir_eps, m);
    // Per-row residual in LENGTH units: divide by the row's Jacobian norm so
    // a direction row (dimensionless) reads as the coordinate distance that
    // would zero it, like the 2D solver's `scaled_res`.
    let row_norms: Vec<f64> = jac
        .iter()
        .map(|row| row.iter().fold(0.0f64, |acc, &v| acc + v * v).sqrt())
        .collect();
    let max_row_norm = row_norms.iter().fold(0.0f64, |a, &v| a.max(v));
    let norm_floor = if max_row_norm > 0.0 { 1e-9 * max_row_norm } else { 1.0 };
    let scaled: Vec<f64> = r
        .iter()
        .zip(row_norms.iter())
        .map(|(&v, &norm)| v / norm.max(norm_floor))
        .collect();
    let max_residual = scaled.iter().fold(0.0f64, |acc, &v| acc.max(v.abs()));

    let (rank, basis) = rank_and_nullspace(jac, m, n);
    let dof = n - rank;
    let redundant = prep.effective_rows.saturating_sub(rank);

    // Conflict: a row still violated at rest names its constraint. Implied
    // rows (arc radius, bezier G1) are analysed but never named.
    let conflict_tol = CONFLICT_REL_TOL * prep.scale;
    let mut flagged = vec![false; doc.constraints.len()];
    let mut row = 0usize;
    for (ai, atom) in prep.atoms.iter().enumerate() {
        let rows = atom.rows();
        let violated = scaled[row..row + rows].iter().any(|v| v.abs() > conflict_tol);
        if violated {
            if let Some(owner) = prep.atom_owner[ai] {
                flagged[owner] = true;
            }
        }
        row += rows;
    }
    let mut conflicting_constraints: Vec<String> = Vec::new();
    for (ci, &hit) in flagged.iter().enumerate() {
        if hit {
            if let Some(id) = doc.constraints[ci].id() {
                conflicting_constraints.push(id_key(id));
            }
        }
    }
    // An implied row alone can be violated (an arc that cannot keep equal
    // radii under the user's dimensions): then the flag stands with the
    // geometry's constraints unnamed, so name every constraint touching a
    // violated implied row's points instead.
    if max_residual > conflict_tol && conflicting_constraints.is_empty() {
        let mut row = 0usize;
        for (ai, atom) in prep.atoms.iter().enumerate() {
            let rows = atom.rows();
            let violated = scaled[row..row + rows].iter().any(|v| v.abs() > conflict_tol);
            row += rows;
            if !violated || prep.atom_owner[ai].is_some() {
                continue;
            }
            for (ci, constraint) in doc.constraints.iter().enumerate() {
                if flagged[ci] {
                    continue;
                }
                if let Some(id) = constraint.id() {
                    flagged[ci] = true;
                    conflicting_constraints.push(id_key(id));
                }
            }
        }
    }
    let conflicting = !conflicting_constraints.is_empty();

    let status = if redundant > 0 {
        "over"
    } else if dof > 0 {
        "under"
    } else {
        "well"
    };

    // Mobility: a free point moves iff some null-space vector has a
    // component at one of its three columns.
    let basis_max: Vec<f64> = basis
        .iter()
        .map(|v| v.iter().fold(0.0f64, |a, &x| a.max(x.abs())).max(1e-300))
        .collect();
    let mut col_of: HashMap<usize, usize> = HashMap::with_capacity(prep.free.len());
    for (k, &i) in prep.free.iter().enumerate() {
        col_of.insert(i, k);
    }
    let mut point_mobility = std::collections::BTreeMap::new();
    for (i, point) in doc.points.iter().enumerate() {
        let movable = match col_of.get(&i) {
            None => false,
            Some(&k) => basis.iter().enumerate().any(|(b, v)| {
                let c = 3 * k;
                let comp = ((v[c] / basis_max[b]).powi(2)
                    + (v[c + 1] / basis_max[b]).powi(2)
                    + (v[c + 2] / basis_max[b]).powi(2))
                .sqrt();
                comp > MOBILITY_TOL
            }),
        };
        point_mobility.insert(
            id_key(&point.id),
            (if movable { "movable" } else { "locked" }).to_string(),
        );
    }
    let mut geometry_mobility = std::collections::BTreeMap::new();
    for geometry in &doc.geometries {
        let movable = geometry
            .points
            .iter()
            .any(|pid| point_mobility.get(&id_key(pid)).map(String::as_str) == Some("movable"));
        geometry_mobility.insert(
            id_key(&geometry.id),
            (if movable { "movable" } else { "locked" }).to_string(),
        );
    }

    // Write the solved coordinates back.
    for (i, point) in doc.points.iter_mut().enumerate() {
        point.x = coords[i].x;
        point.y = coords[i].y;
        point.z = coords[i].z;
    }

    Ok(Sketch3dDiagnostics {
        dof: dof as i64,
        rank: rank as i64,
        unknowns: n as i64,
        equations: m as i64,
        redundant: redundant as i64,
        status: status.into(),
        conflicting,
        conflicting_constraints,
        point_mobility,
        geometry_mobility,
        iterations: iterations as i64,
        max_residual,
    })
}

/// Solve a JSON request: returns `{"sketch": {points, geometries, constraints,
/// diagnostics}}`, the 2D solver's response shape.
pub fn solve_sketch3d(request: &SolveSketch3dRequest) -> Result<Value, String> {
    let mut doc: Sketch3dDoc = serde_json::from_value(request.sketch.clone())
        .map_err(|e| format!("sketch3d: invalid sketch document: {e}"))?;
    let options = Sketch3dSolveOptions {
        tolerance: request.tolerance.unwrap_or_else(default_tolerance),
        max_iterations: request.max_iterations.unwrap_or_else(default_max_iterations),
    };
    let diagnostics = solve_sketch3d_doc(&mut doc, &options)?;
    let mut sketch = serde_json::to_value(&doc).map_err(|e| format!("sketch3d: {e}"))?;
    if let Some(map) = sketch.as_object_mut() {
        map.insert(
            "diagnostics".into(),
            serde_json::to_value(&diagnostics).map_err(|e| format!("sketch3d: {e}"))?,
        );
    }
    Ok(serde_json::json!({ "sketch": sketch }))
}

/// Solve a JSON request given as text; the response is JSON text.
pub fn solve_sketch3d_from_json(request: &str) -> Result<String, String> {
    let request: SolveSketch3dRequest =
        serde_json::from_str(request).map_err(|e| format!("sketch3d: invalid request: {e}"))?;
    let response = solve_sketch3d(&request)?;
    serde_json::to_string(&response).map_err(|e| format!("sketch3d: {e}"))
}

