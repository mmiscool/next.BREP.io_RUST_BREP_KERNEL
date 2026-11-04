//! The sheet-metal parametric model tree.
//!
//! A sheet-metal part is a tree of **flats** (planar wall segments) joined by
//! **bends** (fold lines). One global `thickness`. This is the same idea the retired engine
//! engine uses (`sheetMetalModel.tree`), but here it drives EXACT-BREP geometry,
//! not a triangle mesh, and a single fold-parametric evaluator produces BOTH the
//! folded solid (`fold = 1`) and the flat pattern (`fold = 0`, the "unfold").
//!
//! Geometry is built entirely from existing kernel ops: a flat is an
//! `extrude_profile_brep` plate; a bend is a `revolve_profile_brep_named` annular
//! wedge; the parts fold into place via `transform_brep` and merge via
//! `boolean_operation` (union). No mesh, ever.

use serde::{Deserialize, Serialize};

/// A whole sheet-metal part: one thickness, one root flat, and the bends hanging
/// off it recursively.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SheetTree {
    /// The single material thickness for the entire part.
    pub thickness: f64,
    /// The base wall. Every other wall descends from it through a chain of bends.
    pub root: Flat,
    /// Default inside bend radius captured with the base feature (a flange that
    /// omits its own radius inherits this).
    #[serde(default)]
    pub default_inside_radius: f64,
    /// Default neutral-fiber factor (0..1) for bend-allowance / flat-pattern.
    #[serde(default = "half")]
    pub default_k_factor: f64,
    /// World placement of the base flat's local frame, as `[origin, xAxis, yAxis,
    /// zAxis]`. The base feature (SM.TAB / SM.CF) sets it from the chosen sketch
    /// plane + placement mode; it is preserved as bends are stacked so the folded
    /// solid and the flat pattern share one world anchor. Defaults to identity.
    #[serde(default = "identity_placement")]
    pub root_transform: [[f64; 3]; 4],
    /// Mirror namespaces, oldest first. The evaluator appends `::{suffix}` to
    /// every name it emits, so a mirrored copy keeps the names Mirror gave it
    /// through later sheet-metal edits (see [`SheetTree::mirrored`]).
    #[serde(default)]
    pub name_suffixes: Vec<String>,
}

/// A planar wall segment. Its `outline` is a closed 2D polygon in the flat's own
/// local plane (local +Z is the thickness direction); `holes` are cut regions
/// subtracted from it. `edges` carry the fold information: an edge WITH a `bend`
/// is a fold line to a child flat.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Flat {
    /// Stable id (feature-scoped, e.g. `SM.TAB1:flat_root`). Used for face names.
    pub id: String,
    /// Closed CCW polygon `[[x, y], ...]` in the flat's local plane.
    pub outline: Vec<[f64; 2]>,
    /// One entry per outline segment `outline[i] -> outline[(i+1) % n]`, in order.
    /// A segment with no fold has `bend: None`.
    #[serde(default)]
    pub edges: Vec<Edge>,
    /// Cut regions (holes / cutouts / pockets) baked into the flat — see [`Hole`].
    /// Baked into the TREE (not post-subtracted from one output solid) so every
    /// re-evaluation — flange, cutout, unfold — keeps them, and the flat pattern
    /// shows them too.
    #[serde(default)]
    pub holes: Vec<Hole>,
    /// Bends hanging off HOLE-rim segments (outline bends live on `edges`).
    /// A straight window segment folds a flap into the opening exactly like an
    /// outline flange; a circular loop grows a collar (torus-sector bend + a
    /// revolved wall). Identity is positional: `(hole, segment)` index into
    /// `holes[i].outer`, matching the evaluator's `{flat}:CUTOUT:{hole}:{segment}`
    /// bore face names. Through-holes only (a blind pocket rim cannot fold).
    #[serde(default)]
    pub hole_bends: Vec<HoleBend>,
    /// EXACT-curve overrides for CURVED outline segments, keyed by the segment's
    /// [`Edge::id`]. The polygon `outline` stays the load-bearing skeleton (fold
    /// lines, setbacks, jogs, wedge strips all assume straight segments); a
    /// segment whose id appears here is emitted from this exact flat-local curve
    /// (control points at local `z = 0`, endpoints ON the segment's two outline
    /// vertices) instead of the chord — an arc outline edge stays a true
    /// cylindrical wall in the folded solid AND the flat pattern. Keyed by id
    /// (not index) so flange setback surgery — which splits/renames STRAIGHT
    /// segments but keeps the fold segment's original id — can never silently
    /// re-target an override; the evaluator refuses loudly if a keyed segment
    /// ever carries a bend (straight fold edges only).
    #[serde(default)]
    pub outline_curves: std::collections::BTreeMap<String, crate::NurbsCurve>,
    /// Planes that trim this flat in its local 3D frame (x, y in the plane,
    /// z through the thickness from the midplane): material is kept where
    /// `(p - point) · normal <= 0`. A closed corner trims a wall's extended end
    /// against the other wall's face, or on the mitre plane, at any bend angle,
    /// so the end face may slant through the thickness. Each cut face is named
    /// `{flat}:SIDE:cut{i}`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub end_cuts: Vec<PlaneCut>,
}

/// One trimming plane of [`Flat::end_cuts`], in the flat's local 3D frame.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PlaneCut {
    pub point: [f64; 3],
    /// Unit normal pointing into the removed side.
    pub normal: [f64; 3],
}

/// A fold anchored on a HOLE-rim segment (the hole analogue of [`Edge::bend`]).
/// Self-contained (does not reuse [`Bend`]) because a collar has no child
/// `Flat` — its wall is part of one revolved solid.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HoleBend {
    /// Stable id for the emitted faces (e.g. `SM.F3:bend`).
    pub id: String,
    /// Index into [`Flat::holes`] of the hole this bend lives on.
    pub hole: usize,
    /// Index of the segment (curve) within that hole's `outer` loop.
    pub segment: usize,
    /// Straight segments only: fold with the segment direction REVERSED so the
    /// evaluator's `out = ê × n̂` convention points INTO the opening (set when
    /// the stored loop winds counter-clockwise around the opening).
    #[serde(default)]
    pub reversed: bool,
    /// Fold angle in degrees. Signed exactly like [`Bend::angle_deg`]:
    /// `+` folds toward the flat's −normal side, `-` toward +normal.
    pub angle_deg: f64,
    /// INSIDE bend radius (the tight face).
    pub inside_radius: f64,
    /// Neutral-fiber factor (0..1) for the flat-pattern allowance.
    #[serde(default = "half")]
    pub k_factor: f64,
    /// Straight flap (child wall) vs circular collar.
    pub kind: HoleBendKind,
}

/// What a [`HoleBend`] folds into.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HoleBendKind {
    /// A straight window-rim segment folds a wall into the opening — the exact
    /// analogue of an outline flange (the child may carry further bends).
    Straight { child: Box<Flat> },
    /// A full-circle loop grows a collar: a torus-sector bend swept about the
    /// hole axis plus a cylindrical/conical wall of this leg length, all one
    /// exact revolve.
    Collar { leg: f64 },
}

/// One cut region on a flat: an EXACT closed outer curve loop in the flat's
/// local plane (control points at local `z = 0`), optional island loops that
/// KEEP material inside the cut, and a through-thickness span. Exact curves —
/// not polygons — so a circular hole stays a true cylindrical bore in both the
/// folded solid and the flat pattern.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Hole {
    /// The cut boundary: a closed head-to-tail exact curve chain, local `z = 0`.
    pub outer: Vec<crate::NurbsCurve>,
    /// Island loops nested inside `outer` whose interiors stay material (the
    /// even-odd sketch-region semantics: a region's inner loops are its islands).
    /// A THROUGH cut with an island leaves that island as a disconnected slug —
    /// exactly what even-odd semantics say; a blind cut keeps it attached.
    #[serde(default)]
    pub islands: Vec<Vec<crate::NurbsCurve>>,
    /// Cut floor along flat-local +Z, relative to the flat MIDPLANE (`z = 0`,
    /// faces at ±t/2). `None` = the cut is open below (reaches through face B).
    #[serde(default)]
    pub z_min: Option<f64>,
    /// Cut ceiling, same convention. `None` = open above (through face A).
    #[serde(default)]
    pub z_max: Option<f64>,
}

impl Hole {
    /// A classic through-thickness cut with no islands.
    pub fn through(outer: Vec<crate::NurbsCurve>) -> Self {
        Hole {
            outer,
            islands: Vec::new(),
            z_min: None,
            z_max: None,
        }
    }
}

/// One outline segment of a flat. Carries a `bend` iff it is a fold line.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Edge {
    /// Stable id for face naming (e.g. `SM.TAB1:edge_2`).
    #[serde(default)]
    pub id: String,
    /// The fold at this edge, if any.
    #[serde(default)]
    pub bend: Option<Bend>,
}

/// A fold line joining a parent flat's edge to a child flat.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Bend {
    /// Stable id for the bend faces (e.g. `SM.F3:bend`).
    pub id: String,
    /// Fold angle in degrees. Signed: `+` folds one way, `-` the other. `0` = flat.
    pub angle_deg: f64,
    /// INSIDE bend radius (the tight face). Mid-surface radius = inside + t/2.
    pub inside_radius: f64,
    /// Neutral-fiber factor (0..1); flat-pattern allowance uses it.
    #[serde(default = "half")]
    pub k_factor: f64,
    /// The wall reached across this bend.
    pub child: Box<Flat>,
    /// Through cuts across the bend zone (closed-corner reliefs), as exact
    /// loops in DEVELOPED band coordinates: x across the band from the parent
    /// fold line (the allowance strip spans `0..allowance`), y along the hinge
    /// from its start. The flat pattern cuts them from the allowance strip
    /// exactly; the folded bend cuts their image under the fold map, refit to
    /// tolerance (see `evaluate::warp`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub reliefs: Vec<Hole>,
}

fn half() -> f64 {
    0.5
}

/// Identity root placement: origin at 0, local axes = world axes.
pub fn identity_placement() -> [[f64; 3]; 4] {
    [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
}

impl Bend {
    /// Mid-surface radius: the inside radius plus half the thickness.
    pub fn mid_radius(&self, thickness: f64) -> f64 {
        (self.inside_radius + thickness * 0.5).max(1e-6)
    }

    /// Neutral-fiber radius used for the flat-pattern bend allowance:
    /// `midRadius + (k - 0.5) * thickness`.
    pub fn neutral_radius(&self, thickness: f64) -> f64 {
        self.mid_radius(thickness) + (self.k_factor - 0.5) * thickness
    }

    /// Flat-pattern bend allowance (unfolded arc length of the neutral fiber):
    /// `|angle| * neutralRadius`.
    pub fn allowance(&self, thickness: f64) -> f64 {
        self.angle_deg.to_radians().abs() * self.neutral_radius(thickness)
    }
}

impl SheetTree {
    /// This part reflected through the plane at `point` with unit `normal`,
    /// named `::{suffix}` as Mirror names its copy. Each flat is reflected in
    /// its own frame (the root across local x, every wall along its hinge)
    /// with its outline reversed to stay counter-clockwise, so every placement
    /// frame stays a proper rotation and fold angles keep their sign. A part
    /// with hole-rim bends (indexed by loop segment) is not mirrored: `None`.
    pub(crate) fn mirrored(
        &self,
        point: crate::Vec3,
        normal: crate::Vec3,
        suffix: &str,
    ) -> Result<Option<SheetTree>, String> {
        let Some(root) = self.root.mirrored(&|p| [-p[0], p[1]])? else {
            return Ok(None);
        };
        let v = |a: [f64; 3]| crate::Vec3::new(a[0], a[1], a[2]);
        let reflect = |a: crate::Vec3| a.sub(normal.scale(2.0 * a.dot(normal)));
        let [o, x, y, z] = self.root_transform;
        let o = v(o).sub(point);
        let o = point.add(reflect(o));
        let x = reflect(v(x)).scale(-1.0);
        let (y, z) = (reflect(v(y)), reflect(v(z)));
        let mut name_suffixes = self.name_suffixes.clone();
        name_suffixes.push(suffix.to_string());
        Ok(Some(SheetTree {
            root,
            root_transform: [[o.x, o.y, o.z], [x.x, x.y, x.z], [y.x, y.y, y.z], [z.x, z.y, z.z]],
            name_suffixes,
            ..self.clone()
        }))
    }
}

impl SheetTree {
    /// This part joined with its reflection through the plane at `point` with
    /// unit `normal`, when that plane holds one of the base plate's plain side
    /// walls: the reflected plate merges into the base flat across that edge
    /// (one body, one flat pattern) and every reflected wall hangs off it. The
    /// reflected half's ids take `@{tag}`. A wall that meets its own reflection
    /// in line, with the same bend, merges into one wall twice as long.
    ///
    /// `Ok(None)`: the plane holds no plain base-plate side wall. `Err`: it does,
    /// but the part cannot join (it crosses the plane, or folds from hole rims).
    pub(crate) fn mirror_joined(
        &self,
        point: crate::Vec3,
        normal: crate::Vec3,
        tag: &str,
    ) -> Result<Option<SheetTree>, String> {
        let v = |a: [f64; 3]| crate::Vec3::new(a[0], a[1], a[2]);
        let [o, x, y, z] = self.root_transform;
        let (o, x, y, z) = (v(o), v(x), v(y), v(z));
        if normal.dot(z).abs() > 1e-9 {
            return Ok(None);
        }
        let root = &self.root;
        let n = root.outline.len();
        let world = |p: [f64; 2]| o.add(x.scale(p[0])).add(y.scale(p[1]));
        let on_plane = |p: [f64; 2]| world(p).sub(point).dot(normal).abs() <= 1e-7;
        let Some(k) = (0..n).find(|&k| {
            root.edges.get(k).is_some_and(|edge| edge.bend.is_none() && !root.outline_curves.contains_key(&edge.id))
                && on_plane(root.outline[k])
                && on_plane(root.outline[(k + 1) % n])
        }) else {
            return Ok(None);
        };
        let (a, b) = (root.outline[k], root.outline[(k + 1) % n]);
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        let t = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
        // CCW: the plate lies left of a→b. A vertex right of the line crosses it.
        if root.outline.iter().any(|p| t[0] * (p[1] - a[1]) - t[1] * (p[0] - a[0]) < -1e-7) {
            return Err("the mirror plane passes through the part; pick a plate side wall at its edge".into());
        }
        let reflect = |p: [f64; 2]| {
            let d = [p[0] - a[0], p[1] - a[1]];
            let along = d[0] * t[0] + d[1] * t[1];
            [a[0] + 2.0 * along * t[0] - d[0], a[1] + 2.0 * along * t[1] - d[1]]
        };
        let Some(mut twin) = root.mirrored(&reflect)? else {
            return Err("a part with flanges on hole rims cannot be mirrored yet".into());
        };
        for edge in &mut twin.edges {
            if let Some(curve) = twin.outline_curves.remove(&edge.id) {
                twin.outline_curves.insert(format!("{}@{tag}", edge.id), curve);
            }
            edge.id = format!("{}@{tag}", edge.id);
            if let Some(bend) = &mut edge.bend {
                bend.retag(tag);
            }
        }
        // `mirrored` maps old edge k to twin edge n−2−k, running b→a; walk the
        // plate from b round to a, then the twin from a round to b.
        let jk = (2 * n - 2 - k) % n;
        let mut outline = Vec::with_capacity(2 * n - 2);
        let mut edges = Vec::with_capacity(2 * n - 2);
        for i in 0..n {
            outline.push(root.outline[(k + 1 + i) % n]);
            if i + 1 < n {
                edges.push(root.edges[(k + 1 + i) % n].clone());
            }
        }
        edges.push(twin.edges[(jk + 1) % n].clone());
        for i in 2..n {
            outline.push(twin.outline[(jk + i) % n]);
            edges.push(twin.edges[(jk + i) % n].clone());
        }
        let mut merged = Flat {
            outline,
            edges,
            holes: root.holes.iter().chain(&twin.holes).cloned().collect(),
            outline_curves: root.outline_curves.clone().into_iter().chain(twin.outline_curves).collect(),
            ..root.clone()
        };
        // The seam vertices: a (index n−1, plate edge in, twin edge out) and
        // b (index 0, twin edge in, plate edge out).
        merge_in_line(&mut merged, n - 1, false);
        merge_in_line(&mut merged, 0, true);
        Ok(Some(SheetTree { root: merged, ..self.clone() }))
    }
}

/// Merge the two edges meeting at `vertex` of `flat` when they run in line and
/// can be one edge: both plain, or both carrying the same bend to a plain
/// rectangular wall. The merged edge keeps the plate side's id, bend and wall
/// (`twin_in` says the incoming edge is the reflected one).
fn merge_in_line(flat: &mut Flat, vertex: usize, twin_in: bool) {
    let n = flat.outline.len();
    let (prev, next) = ((vertex + n - 1) % n, (vertex + 1) % n);
    let (p, v, q) = (flat.outline[prev], flat.outline[vertex], flat.outline[next]);
    let (d1, d2) = ([v[0] - p[0], v[1] - p[1]], [q[0] - v[0], q[1] - v[1]]);
    let (l1, l2) = (d1[0].hypot(d1[1]), d2[0].hypot(d2[1]));
    if (d1[0] * d2[1] - d1[1] * d2[0]).abs() > 1e-9 * l1 * l2 || d1[0] * d2[0] + d1[1] * d2[1] <= 0.0 {
        return;
    }
    let (incoming, outgoing) = (&flat.edges[prev], &flat.edges[vertex]);
    if flat.outline_curves.contains_key(&incoming.id) || flat.outline_curves.contains_key(&outgoing.id) {
        return;
    }
    let plain_wall = |bend: &Bend, span: f64| {
        let c = &bend.child;
        let leg = c.outline.get(1).map_or(0.0, |p| p[0]);
        c.outline.len() == 4
            && c.holes.is_empty()
            && c.hole_bends.is_empty()
            && c.outline_curves.is_empty()
            && c.edges.iter().all(|e| e.bend.is_none())
            && [[0.0, 0.0], [leg, 0.0], [leg, span], [0.0, span]]
                .iter()
                .zip(&c.outline)
                .all(|(e, a)| (e[0] - a[0]).abs() <= 1e-9 && (e[1] - a[1]).abs() <= 1e-9)
    };
    let keep = if twin_in { outgoing } else { incoming };
    let edge = match (&incoming.bend, &outgoing.bend) {
        (None, None) => keep.clone(),
        (Some(b1), Some(b2))
            if b1.angle_deg == b2.angle_deg
                && b1.inside_radius == b2.inside_radius
                && b1.k_factor == b2.k_factor
                && plain_wall(b1, l1)
                && plain_wall(b2, l2)
                && (b1.child.outline[1][0] - b2.child.outline[1][0]).abs() <= 1e-9 =>
        {
            let mut edge = keep.clone();
            let child = &mut edge.bend.as_mut().expect("bent edge").child;
            let span = l1 + l2;
            child.outline[2][1] = span;
            child.outline[3][1] = span;
            edge
        }
        _ => return,
    };
    flat.outline.remove(vertex);
    flat.edges[prev] = edge;
    flat.edges.remove(vertex);
}

impl Bend {
    /// Give this bend, its wall and everything beyond it the id suffix `@{tag}`.
    fn retag(&mut self, tag: &str) {
        self.id = format!("{}@{tag}", self.id);
        self.child.retag(tag);
    }
}

impl Flat {
    fn retag(&mut self, tag: &str) {
        self.id = format!("{}@{tag}", self.id);
        self.outline_curves = std::mem::take(&mut self.outline_curves)
            .into_iter()
            .map(|(id, curve)| (format!("{id}@{tag}"), curve))
            .collect();
        for edge in &mut self.edges {
            edge.id = format!("{}@{tag}", edge.id);
            if let Some(bend) = &mut edge.bend {
                bend.retag(tag);
            }
        }
    }
}

impl Flat {
    /// This flat with its local plane reflected by `map` (an in-plane mirror),
    /// outline reversed so it stays CCW and each edge keeps its id and bend:
    /// new vertex `j` is old vertex `n−1−j`, new edge `j` old edge `n−2−j`.
    /// A child wall's frame runs along its hinge from the edge start, which the
    /// reversal swaps, so each child reflects along its hinge length.
    fn mirrored(&self, map: &dyn Fn([f64; 2]) -> [f64; 2]) -> Result<Option<Flat>, String> {
        let n = self.outline.len();
        if !self.hole_bends.is_empty() || self.edges.len() != n {
            return Ok(None);
        }
        let curve = |c: &crate::NurbsCurve| -> Result<crate::NurbsCurve, String> {
            let points = c
                .control_points
                .iter()
                .map(|p| {
                    let q = p.point()?;
                    let [x, y] = map([q.x, q.y]);
                    Ok(crate::Vec4::from_point(crate::Vec3::new(x, y, q.z), p.w))
                })
                .collect::<Result<Vec<_>, String>>()?;
            crate::NurbsCurve::new(c.degree, c.knots.clone(), points)?.reversed()
        };
        let chain = |curves: &[crate::NurbsCurve]| -> Result<Vec<crate::NurbsCurve>, String> {
            curves.iter().rev().map(curve).collect()
        };
        let mut edges = Vec::with_capacity(n);
        // Reversed from the last vertex, so a wall built as `[0,0] [leg,0]
        // [leg,span] [0,span]` (hinge last) keeps exactly that layout.
        for j in 0..n {
            let i = (2 * n - 2 - j) % n;
            let edge = &self.edges[i];
            let bend = match &edge.bend {
                None => None,
                Some(bend) => {
                    let (a, b) = (self.outline[i], self.outline[(i + 1) % n]);
                    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
                    let Some(child) = bend.child.mirrored(&|p| [p[0], length - p[1]])? else {
                        return Ok(None);
                    };
                    Some(Bend { child: Box::new(child), ..bend.clone() })
                }
            };
            edges.push(Edge { id: edge.id.clone(), bend });
        }
        let mut holes = Vec::with_capacity(self.holes.len());
        for hole in &self.holes {
            holes.push(Hole {
                outer: chain(&hole.outer)?,
                islands: hole.islands.iter().map(|island| chain(island)).collect::<Result<_, _>>()?,
                ..hole.clone()
            });
        }
        let mut outline_curves = std::collections::BTreeMap::new();
        for (id, c) in &self.outline_curves {
            outline_curves.insert(id.clone(), curve(c)?);
        }
        Ok(Some(Flat {
            id: self.id.clone(),
            outline: (0..n).map(|j| map(self.outline[(2 * n - 1 - j) % n])).collect(),
            edges,
            holes,
            hole_bends: Vec::new(),
            outline_curves,
            // A cut plane reflects with its flat: the point through `map`, the
            // normal through `map`'s linear part (z is unchanged).
            end_cuts: self
                .end_cuts
                .iter()
                .map(|cut| {
                    let [x, y, z] = cut.point;
                    let [nx, ny, nz] = cut.normal;
                    let (o, p, q) = (map([0.0, 0.0]), map([x, y]), map([nx, ny]));
                    PlaneCut { point: [p[0], p[1], z], normal: [q[0] - o[0], q[1] - o[1], nz] }
                })
                .collect(),
        }))
    }

    /// Search this flat, outline bends, then straight hole-flap bends by id.
    pub(crate) fn find_by_id(&self, flat_id: &str) -> Option<&Self> {
        if self.id == flat_id {
            return Some(self);
        }
        for edge in &self.edges {
            if let Some(bend) = &edge.bend {
                if let Some(found) = bend.child.find_by_id(flat_id) {
                    return Some(found);
                }
            }
        }
        for hole_bend in &self.hole_bends {
            if let HoleBendKind::Straight { child } = &hole_bend.kind {
                if let Some(found) = child.find_by_id(flat_id) {
                    return Some(found);
                }
            }
        }
        None
    }

    /// Search this flat, outline bends, then straight hole-flap bends by id.
    pub(crate) fn find_by_id_mut(&mut self, flat_id: &str) -> Option<&mut Self> {
        if self.id == flat_id {
            return Some(self);
        }
        for edge in &mut self.edges {
            if let Some(bend) = &mut edge.bend {
                if let Some(found) = bend.child.find_by_id_mut(flat_id) {
                    return Some(found);
                }
            }
        }
        for hole_bend in &mut self.hole_bends {
            if let HoleBendKind::Straight { child } = &mut hole_bend.kind {
                if let Some(found) = child.find_by_id_mut(flat_id) {
                    return Some(found);
                }
            }
        }
        None
    }

    /// Outline in flat-local coordinates, with 16 samples per curved segment.
    /// Each segment includes its start and excludes its end so adjacent segments
    /// share one endpoint. Straight segments retain their original vertex.
    /// Used for footprint tests and flat-pattern export; raw outline indexing
    /// remains available to the fold-zone construction code.
    pub(super) fn densified_outline(&self) -> Result<Vec<[f64; 2]>, String> {
        if self.outline_curves.is_empty() {
            return Ok(self.outline.clone());
        }
        let mut poly = Vec::with_capacity(self.outline.len());
        for (i, vertex) in self.outline.iter().enumerate() {
            match self
                .edges
                .get(i)
                .and_then(|edge| self.outline_curves.get(&edge.id))
            {
                Some(curve) => {
                    let [t0, t1] = curve.domain()?;
                    for step in 0..16 {
                        let t = t0 + (t1 - t0) * (step as f64) / 16.0;
                        let p = curve.evaluate(t)?;
                        poly.push([p.x, p.y]);
                    }
                }
                None => poly.push(*vertex),
            }
        }
        Ok(poly)
    }
}

