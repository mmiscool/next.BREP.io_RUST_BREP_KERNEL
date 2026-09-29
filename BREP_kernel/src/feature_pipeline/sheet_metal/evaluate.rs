//! The fold-parametric evaluator: a [`SheetTree`] → one exact-BREP solid.
//!
//! `evaluate(tree, fold)` walks the tree and unions an exact-BREP plate for every
//! flat with an exact-BREP annular wedge for every bend. The `fold` fraction
//! scales the bend angles:
//!   * `fold = 1.0` → the folded 3D part (authored angles),
//!   * `fold = 0.0` → the flat pattern (all bends opened) — the "unfold".
//!
//! Geometry comes only from existing kernel ops: [`extrude_profile_brep`] for
//! plates, [`revolve_profile_brep_named`] for bends, [`transform_brep`] for
//! placement, and [`boolean_operation`] union to merge. No triangle mesh.
//!
//! ## Bend construction (why the union is watertight)
//!
//! A bend is a `revolve` of the parent's through-thickness edge cross-section
//! (`[Rin, Rout] × [0, L]`, where `Rout - Rin = thickness`) about an axis parallel
//! to the fold edge and offset `-Rm·n̂` from it. Its angle-0 cap is that exact
//! `thickness × L` rectangle, coincident with the parent plate's fold-edge side
//! face; its angle-θ cap is coincident with the child plate's fold-edge face. So
//! every bend↔plate union interface is planar-coplanar (the easy boolean case),
//! never a tangent cylinder-plane graze.
//!
//! ## Hole-rim bends
//!
//! [`Flat::hole_bends`] fold walls off HOLE rims: a straight window segment
//! reuses [`build_bend`] verbatim (the flap folds INTO the opening), and a
//! circular loop grows a [`build_collar`] — one full-turn revolve whose
//! cross-section is the same annular bend wedge (torus-sector faces) glued to
//! the straight wall leg, meeting the plate through a thin buried overlap
//! ring (the transversal stand-in for the exactly-coincident bore cylinders,
//! which the boolean cannot classify).

use super::tree::{Bend, Edge, Flat, HoleBend, HoleBendKind, SheetTree};
use crate::{
    boolean_operation, extrude_profile_brep, make_arc, make_line, revolve_profile_brep_named,
    transform_brep, AffineTransform, BooleanOperation, BooleanOptions, BrepSolid, Vec3,
};

mod frame;
mod plate;
mod bend;
mod collar;
mod walk;

pub use walk::{evaluate, folded_flat_placements, FlatPlacement, WedgeStrip};
pub(crate) use collar::circle_of_loop;
