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

impl Flat {
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

