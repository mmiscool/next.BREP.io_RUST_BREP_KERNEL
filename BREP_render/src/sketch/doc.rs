//! Typed sketch JSON shared with the 2D constraint solver.
//!
//! Documents contain points, geometries, constraints, and optional diagnostics.
//! Flattened serde fields preserve unmodeled geometry and constraint data across
//! load/save, including solver bookkeeping.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

/// Index points using the solver's identity keys; later duplicate ids take precedence.
pub(super) fn point_index(doc: &SketchDoc) -> HashMap<String, &SketchPoint> {
    let mut by_id: HashMap<String, &SketchPoint> = HashMap::with_capacity(doc.points.len());
    for p in &doc.points {
        by_id.insert(id_key(&p.id), p);
    }
    by_id
}

/// A sketch point: a solved 2D coordinate in the plane's `(u, v)` frame, plus the
/// three role flags the solver tracks. Emitted by the solver as exactly these six
/// fields (`solvers/sketch_solver.rs` solve output).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SketchPoint {
    /// Stable point id (a number in practice, but the solver keys by `Value`, so
    /// strings are legal too — kept as a `Value` for a lossless round-trip).
    pub id: Value,
    pub x: f64,
    pub y: f64,
    /// Ground/pinned: the solver removes this point's coordinates from the free
    /// set entirely (always `locked`).
    #[serde(default)]
    pub fixed: bool,
    /// Construction point: constrains but never models a profile edge; drawn in
    /// the construction color.
    #[serde(default)]
    pub construction: bool,
    /// A point adopted from an external reference (a picked edge endpoint).
    #[serde(default, rename = "externalReference")]
    pub external_reference: bool,
}

/// A sketch geometry: `line = [p0,p1]`, `circle = [center,radiusPoint]`,
/// `arc = [center,start,end]` (CCW start→end), `ellipse = [center,majEnd,minEnd]`,
/// `bezier = [p0,p1,p2,p3,…]` (every 3 ids a new cubic span). Point semantics are
/// the ground truth from the sketch feature's edge builder.
///
/// `points` are point ids (matched against [`SketchPoint::id`] via [`id_key`]).
/// The `construction` flag (dashed, non-modeling) and any other authored fields
/// ride along in `extra`, so the geometry round-trips losslessly.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SketchGeometry {
    pub id: Value,
    #[serde(rename = "type")]
    pub geom_type: String,
    #[serde(default)]
    pub points: Vec<Value>,
    /// Every field this slice does not model explicitly (`construction`, authoring
    /// metadata, …) — preserved verbatim so a load/save is lossless.
    #[serde(flatten)]
    pub extra: Map<String, Value>,
}

impl SketchGeometry {
    /// Whether this geometry is construction-only (dashed, excluded from profiles).
    pub fn construction(&self) -> bool {
        self.extra
            .get("construction")
            .and_then(Value::as_bool)
            .unwrap_or(false)
    }
}

/// A sketch constraint. The solver mutates constraints with a large amount of
/// bookkeeping (`status`, `error`, `previousPointValues`, `_previousSolveValue`,
/// the `_distance*` slide state, …) that must persist between solves EXACTLY, so
/// the whole object is kept as a transparent map rather than a lossy typed struct.
/// Typed accessors ([`ctype`](Self::ctype), [`points`](Self::points)) read the
/// fields this slice needs.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(transparent)]
pub struct SketchConstraint {
    pub raw: Map<String, Value>,
}

impl SketchConstraint {
    /// The constraint glyph/type (`"⏚"` ground, `"━"` horizontal, `"⟺"` distance, …).
    pub fn ctype(&self) -> Option<&str> {
        self.raw.get("type").and_then(Value::as_str)
    }

    /// The point ids this constraint references (empty if absent).
    pub fn points(&self) -> &[Value] {
        self.raw
            .get("points")
            .and_then(Value::as_array)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    /// Whether this is a solver-internal helper constraint (excluded from the DOF
    /// diagnostics and from the "constrained points" fallback coloring).
    pub fn temporary(&self) -> bool {
        self.raw
            .get("temporary")
            .and_then(Value::as_bool)
            .unwrap_or(false)
    }
}

/// The editable sketch document — a typed mirror of the solver's
/// `{points, geometries, constraints}`. Unknown top-level keys (e.g. the solved
/// output's `diagnostics`) are ignored on load; [`crate::sketch::solve`] pulls
/// diagnostics out into [`SketchDiagnostics`] separately.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct SketchDoc {
    #[serde(default)]
    pub points: Vec<SketchPoint>,
    #[serde(default)]
    pub geometries: Vec<SketchGeometry>,
    #[serde(default)]
    pub constraints: Vec<SketchConstraint>,
}

impl SketchDoc {
    /// Look up a point by id (matched via [`id_key`], mirroring the solver's
    /// `point_key` identity).
    pub fn point(&self, id: &Value) -> Option<&SketchPoint> {
        let key = id_key(id);
        self.points.iter().find(|p| id_key(&p.id) == key)
    }

    /// Mutable lookup of a point by id (used by the interactive point drag to write
    /// a solved coordinate / toggle the transient `fixed` anchor flag).
    pub fn point_mut(&mut self, id: &Value) -> Option<&mut SketchPoint> {
        let key = id_key(id);
        self.points.iter_mut().find(|p| id_key(&p.id) == key)
    }

    /// The set of non-temporary constrained point ids (as [`id_key`] strings) —
    /// the fallback used to color under-constrained points when the solver did not
    /// supply per-point mobility (it always does, so this is a safety net).
    pub fn constrained_point_keys(&self) -> std::collections::HashSet<String> {
        let mut set = std::collections::HashSet::new();
        for c in &self.constraints {
            if c.temporary() {
                continue;
            }
            for pid in c.points() {
                set.insert(id_key(pid));
            }
        }
        set
    }
}

/// The solver's read-only constraint diagnostics: degrees of freedom, over/under
/// status, and per-point / per-geometry mobility (movable vs locked) derived from
/// the constraint-Jacobian null space. Keys in the mobility maps are [`id_key`]
/// strings.
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct SketchDiagnostics {
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
    #[serde(default)]
    pub status: String,
    #[serde(default)]
    pub conflicting: bool,
    /// The constraints the solver named as mutually unsatisfiable, as [`id_key`]
    /// strings. When the constraint rows are linearly dependent this is the group
    /// a left null-space certificate implicates, NOT merely whichever constraint
    /// relaxation happened to leave violated last; a contradiction whose rows stay
    /// independent admits no such certificate, and the group is then every
    /// constraint the solve came to rest still violating. Empty exactly when
    /// [`conflicting`](Self::conflicting) is clear — the solver drops the flag
    /// rather than raise it with nothing to point at.
    #[serde(default, rename = "conflictingConstraints")]
    pub conflicting_constraints: Vec<String>,
    /// `id_key -> "movable" | "locked"` per point (`BTreeMap` for a deterministic,
    /// solver-matching key order).
    #[serde(default, rename = "pointMobility")]
    pub point_mobility: std::collections::BTreeMap<String, String>,
    #[serde(default, rename = "geometryMobility")]
    pub geometry_mobility: std::collections::BTreeMap<String, String>,
}

impl SketchDiagnostics {
    /// Whether `id` names a constraint in the conflicting group (matched via
    /// [`id_key`], so a numeric `4` and a string `"4"` agree with the solver).
    pub fn constraint_conflicting(&self, id: &Value) -> bool {
        if self.conflicting_constraints.is_empty() {
            return false;
        }
        let key = id_key(id);
        self.conflicting_constraints.iter().any(|c| *c == key)
    }
}

impl SketchDiagnostics {
    /// `Some(true)` movable, `Some(false)` locked, `None` if the solver gave no
    /// mobility for this point id.
    pub fn point_movable(&self, id: &Value) -> Option<bool> {
        self.point_mobility
            .get(&id_key(id))
            .map(|v| v == "movable")
    }

    /// `Some(true)` movable, `Some(false)` locked, `None` if unknown.
    pub fn geometry_movable(&self, id: &Value) -> Option<bool> {
        self.geometry_mobility
            .get(&id_key(id))
            .map(|v| v == "movable")
    }
}

/// Shared solver key for sketch entity and mobility-map lookups.
pub use brep_kernel::sketch_id_key as id_key;

// ---------------------------------------------------------------------------
// S3a: id minting + snap-or-add (the draw tools' point/geometry factory).
//
// Points and geometries have SEPARATE id spaces; a fresh id is `max(numeric ids)
// + 1`, or a count-based id when nothing parses as a number, and is guaranteed
// not to collide (under [`id_key`]) with any existing id in the collection.
// [`SketchDoc::snap_or_add_point`] reuses an existing point within a grab radius
// so shared vertices coincide (chained lines, closed loops, arc endpoints).
// ---------------------------------------------------------------------------

impl SketchDoc {
    /// Mint the next unused POINT id (`max numeric id + 1`, else a count-based id),
    /// non-colliding within `self.points`.
    pub fn next_point_id(&self) -> Value {
        let ids: Vec<&Value> = self.points.iter().map(|p| &p.id).collect();
        mint_next_id(&ids)
    }

    /// Mint the next unused GEOMETRY id (`max numeric id + 1`, else a count-based
    /// id), non-colliding within `self.geometries`.
    pub fn next_geometry_id(&self) -> Value {
        let ids: Vec<&Value> = self.geometries.iter().map(|g| &g.id).collect();
        mint_next_id(&ids)
    }

    /// Return the id of the existing point within `radius` of `(u, v)` (the nearest
    /// one, so drawn vertices coincide), else mint a new free point at `(u, v)` and
    /// return its id. The `radius` is the same ~8px→world grab tolerance S2 uses.
    pub fn snap_or_add_point(&mut self, u: f64, v: f64, radius: f64) -> Value {
        let mut best: Option<(f64, Value)> = None;
        for p in &self.points {
            let d = ((p.x - u).powi(2) + (p.y - v).powi(2)).sqrt();
            if d <= radius && best.as_ref().map_or(true, |(bd, _)| d < *bd) {
                best = Some((d, p.id.clone()));
            }
        }
        if let Some((_, id)) = best {
            return id;
        }
        let id = self.next_point_id();
        self.points.push(SketchPoint {
            id: id.clone(),
            x: u,
            y: v,
            fixed: false,
            construction: false,
            external_reference: false,
        });
        id
    }
}

// ---------------------------------------------------------------------------
// S4: constraint id minting + geometry lookup (the constraint palette's factory).
//
// Constraints have their OWN id space (like points/geometries): a fresh id is
// `max(numeric constraint ids) + 1`. Geometry lookup mirrors [`SketchDoc::point`]
// so the palette can resolve a selected geometry ref back to its `SketchGeometry`
// (its point roles + type) to assemble a constraint's ordered point list.
// ---------------------------------------------------------------------------

impl SketchDoc {
    /// Mint the next unused CONSTRAINT id (`max numeric id + 1`, else a count-based
    /// id), non-colliding within `self.constraints`. Constraints carry their id in
    /// the raw map's `"id"` field.
    pub fn next_constraint_id(&self) -> Value {
        let ids: Vec<&Value> = self
            .constraints
            .iter()
            .filter_map(|c| c.raw.get("id"))
            .collect();
        mint_next_id(&ids)
    }

    /// Look up a geometry by id (matched via [`id_key`], mirroring [`Self::point`]).
    pub fn geometry(&self, id: &Value) -> Option<&SketchGeometry> {
        let key = id_key(id);
        self.geometries.iter().find(|g| id_key(&g.id) == key)
    }

    /// Mutable lookup of a geometry by id (used by the construction toggle to flip a
    /// selected geometry's `construction` flag in its `extra` bag).
    pub fn geometry_mut(&mut self, id: &Value) -> Option<&mut SketchGeometry> {
        let key = id_key(id);
        self.geometries.iter_mut().find(|g| id_key(&g.id) == key)
    }
}

/// Parse an id `Value` as a number (numeric ids, or numeric strings like `"12"`),
/// or `None` for a non-numeric id.
fn id_num(value: &Value) -> Option<f64> {
    match value {
        Value::Number(number) => number.as_f64(),
        Value::String(text) => text.trim().parse::<f64>().ok(),
        _ => None,
    }
}

/// Mint the next id for a collection: `max(numeric ids) + 1`, falling back to the
/// element count when nothing parses as a number, then bumped until it does not
/// collide (under [`id_key`]) with any existing id.
fn mint_next_id(ids: &[&Value]) -> Value {
    // The reserved part-origin point (and its ground) never takes a slot in the
    // numbering, so the first user point / constraint is still 0.
    let ids: Vec<&Value> = ids
        .iter()
        .copied()
        .filter(|id| !brep_kernel::is_sketch_origin_point_id(id))
        .collect();
    let ids = ids.as_slice();
    let mut max: Option<i64> = None;
    for &id in ids {
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

