//! Direct editing — Golovanov §6.12: "delete face and heal" and "move a face
//! group" (see [`move_faces`] for the sibling operation's algorithm and
//! honest v1 scope).
//!
//! Removes a transition face (a chamfer, a constant-radius fillet, or any
//! simple 4-sided face) that sits between two larger neighbour faces, and
//! heals the resulting hole by EXTENDING the neighbours' carrier surfaces and
//! RE-INTERSECTING them so the neighbours meet each other directly where the
//! deleted face used to be. The classic use is "undo a chamfer/fillet": the
//! two large planes that flanked the bevel extend and re-intersect at the
//! original sharp edge, the end-cap faces lose the bevel corner, and the
//! solid returns to its pre-blend shape.
//!
//! ## Algorithm (extend → re-intersect → re-trim → reconnect)
//!
//! For a 4-sided transition face `F` with neighbours `N0..N3` (one per
//! boundary edge, in loop order):
//!
//! 1. **Classify.** The two neighbours on OPPOSITE boundary edges whose
//!    (planar) carriers re-intersect in a line passing through `F`'s region
//!    are the *primary* pair (the faces to extend and rejoin). The other two
//!    opposite neighbours are the *lateral* faces (the end caps). Exactly one
//!    opposite pairing must qualify, otherwise the heal is refused.
//! 2. **Re-intersect (extend).** Because analytic planes are unbounded
//!    carriers, the primary pair's intersection line `S` is computed in closed
//!    form (no marching within a bounded patch). `S` clipped by each lateral
//!    plane gives the two recovered corner points — the new sharp edge's
//!    endpoints (`triple points`).
//! 3. **Re-trim.** Every side edge that met `F` at a transition corner is
//!    relocated onto the recovered corner; the deleted face's two support
//!    edges become the single new edge `S` in both primary faces; the two
//!    lateral (cap) edges collapse to points. Each affected neighbour face's
//!    carrier plane is rebuilt large enough to cover its new boundary and all
//!    its pcurves are recomputed against it (analytic planes extend for free).
//! 4. **Reconnect.** `F` and its boundary edges/vertices are pruned, `S` and
//!    its two vertices are inserted, the shell is reassembled, the genus is
//!    preserved, and the result must `validate()` or the whole operation is
//!    refused.
//!
//! ## Covered vs deferred (honest scope of this first slice)
//!
//! - COVERED: a 4-sided chamfer/fillet (or any simple 4-edge transition face)
//!   between two PLANAR primary neighbours with two PLANAR lateral caps, convex
//!   or concave. Chamfer (planar transition) and fillet (cylindrical
//!   transition) both reduce to the same planar-neighbour heal. CURVED
//!   ANALYTIC neighbours route to [`heal_open_transition_mixed`], which
//!   handles both curved primaries (plane × cylinder/ruled revolution
//!   re-intersection) and curved lateral caps (the re-intersection branch is
//!   clipped by curve×surface intersection against the cap's carrier).
//! - COVERED: the WALL OF A THROUGH FEATURE — a drilled bore, or any closed
//!   wall whose two rims are each the whole hole loop of the face it was
//!   punched through. Those neighbours were never meant to meet, so there is
//!   no re-intersection to make: the wall and both hole loops are dropped and
//!   the neighbours close back over the opening, un-drilling the hole. Nothing
//!   is refit (the surviving carriers and outer loops are untouched), and the
//!   genus drops by the handle that goes with it. A mouth that is TANGENT to
//!   the boundary of the face it opens through is covered by the same cap: the
//!   tangency pinches that face and the arrangement splits it, so the rim
//!   arrives as runs of several faces' loops instead of as a hole loop, and
//!   dropping it rejoins the pieces into the face they were (see
//!   `direct_edit/delete_faces.rs`).
//! - COVERED, as a SET: a BAND across a mirrored union — a fillet groove that
//!   runs over a part and down its side, flanked on both sides by cosurface
//!   TWINS of one plane or one fillet cylinder. Its free boundary stops on
//!   setback and tangent lines the band interrupted rather than on a gap
//!   between two carriers, so the rejoin BRIDGES each interrupted line across
//!   the band and the twins close back into one face
//!   (`direct_edit/delete_faces.rs`).
//! - COVERED, as a SET: a whole CORNER BLEND — a three-sided vertex blend and
//!   the three four-sided strips that meet on it. One face at a time it is
//!   ill-posed (a strip's walls re-intersect in the sharp edge, which passes
//!   the corner sphere at `r·√2` and never meets it, so the strip has no cap to
//!   clip against); taken together the walls close a cycle of three carriers
//!   and each strip's corner end is where its own recovered edge meets the
//!   third wall (`direct_edit/corner_heal.rs`).
//! - COVERED, as a SET: a CLOSED BAND — any ring of faces whose free boundary
//!   is one closed run on each of exactly two survivors, such as a stepped nut
//!   pocket sunk over a through bore. It is the closed transition strip with
//!   many faces: the two survivors are extended, re-intersected in closed
//!   form, and each run collapses onto the rim, so the bore runs on up to the
//!   face the pocket opened through (`direct_edit/closed_band.rs`).
//! - COVERED, as a SET: a whole BLEND NETWORK over planar faces — the strips,
//!   vertex blends and mitres a fillet left around a gusset's foot, with more
//!   than one corner and strips that end on other strips. Each cluster of end
//!   edges shrinks to the vertex its planes meet in, and each strip becomes the
//!   sharp edge between its two clusters (`direct_edit/blend_network_heal.rs`).
//!   A cylindrical strip the selection leaves standing is KEPT: where it mitred
//!   into a deleted strip it now ends on the cap plane square to its axis, on
//!   that plane's circular section; any other curved face left beside a network
//!   is refused by name.
//! - COVERED, as a SET (see [`delete_faces_and_heal`] and
//!   `direct_edit/delete_faces.rs`): a BLIND pocket, a blind bore, or a boss —
//!   every face of the feature selected together. One face at a time these are
//!   a multi-face gap with nothing to re-intersect; taken together they are a
//!   PATCH whose free boundary is the whole hole loop of the face the feature
//!   was sunk through, and the same cap that un-drills a bore un-cuts them.
//! - DEFERRED (returns a clear `Err`, never a bad solid): free-form
//!   neighbours (need NURBS carrier extension + marched re-intersect),
//!   transition faces that are not 4-sided (multi-face gaps) — the cap lane
//!   reads any number of boundary edges, but a face that has to be healed by
//!   RE-INTERSECTION still has to be four-sided — a neighbour that
//!   borders the transition more than once (periodic/closed transition), a
//!   BLIND pocket's wall on its own (the wall and its floor disc are a
//!   two-face gap — select the floor as well and the cap lane takes it), a
//!   wall whose removal would SEVER the body into two parts (a post joining
//!   two otherwise separate plates wears a through wall's exact signature, so
//!   the cap checks the shell is still connected before it hands anything
//!   back), and configurations where the neighbours cannot re-intersect
//!   cleanly (non-adjacent healing, or parallel planes that are not a through
//!   wall).

use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::transform_topology::{transform_curve, transform_surface};
use crate::offset_retrim::{boundary_samples, retrim_face_in_solid};
use crate::offset_reintersect::{
    arc_of_section, edge_seeds, match_marched_rim_direction, reintersect_carriers, section_corner,
    MarchPolicy, ReintersectRefusal, RimLane,
};
use crate::{
    build_pcurve_on_surface, build_pcurve_on_surface_range, intersect_analytic_pair,
    intersect_curve_surface, make_line, make_revolution, parameter_space_area,
    plane_ruled_section_arc,
    project_point_to_curve, solid_model_scale, solid_signed_volume, AffineTransform,
    AnalyticSurface, BrepSolid,
    NurbsCurve, NurbsSurface, OffsetEvaluator, OffsetNormal, SurfaceSide, Vec3,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

#[path = "direct_edit/geom.rs"]
mod geom;
#[path = "direct_edit/delete_face.rs"]
mod delete_face;
#[path = "direct_edit/delete_faces.rs"]
mod delete_faces;
#[path = "direct_edit/refit_faces.rs"]
mod refit_faces;
#[path = "direct_edit/face_move.rs"]
mod face_move;
#[path = "direct_edit/face_move_carrier.rs"]
mod face_move_carrier;
#[path = "direct_edit/face_move_recut.rs"]
mod face_move_recut;
#[path = "direct_edit/face_rotate.rs"]
mod face_rotate;
#[path = "direct_edit/face_offset.rs"]
mod face_offset;
#[path = "direct_edit/face_offset_sphere.rs"]
mod face_offset_sphere;
#[path = "direct_edit/face_offset_torus.rs"]
mod face_offset_torus;
#[path = "direct_edit/face_offset_revolution.rs"]
mod face_offset_revolution;
#[path = "direct_edit/face_offset_freeform.rs"]
mod face_offset_freeform;
#[path = "direct_edit/closed_heal.rs"]
mod closed_heal;
#[path = "direct_edit/open_carriers.rs"]
mod open_carriers;
#[path = "direct_edit/open_heal.rs"]
mod open_heal;
#[path = "direct_edit/open_heal_census.rs"]
mod open_heal_census;
#[path = "direct_edit/closed_heal_census.rs"]
mod closed_heal_census;
#[path = "direct_edit/triple_point.rs"]
mod triple_point;
#[path = "direct_edit/corner_heal.rs"]
mod corner_heal;
#[path = "direct_edit/closed_band.rs"]
mod closed_band;
#[path = "direct_edit/blend_network_heal.rs"]
mod blend_network_heal;


use blend_network_heal::*;
use closed_band::*;
use closed_heal::*;
use closed_heal_census::*;
use corner_heal::*;
use delete_face::*;
use face_move_carrier::*;
use geom::*;
use open_carriers::*;
use open_heal::*;
use open_heal_census::*;
use triple_point::*;

/// The slugs of the direct-edit refusals the app explains to a user (its
/// face-transform help, `BREP_render::face_transform_help`), shared by each
/// mint and by [`face_transform_reason`].
pub mod face_refusal_slugs {
    /// The motion turns the whole body inside out (`UnsupportedGeometry`).
    pub const SOLID_INVERSION: &str = "solid_inversion";
    /// The same, measured on the sphere lane (`UnsupportedGeometry`).
    pub const SPHERE_SOLID_INVERSION: &str = "sphere_solid_inversion";
    /// A push that inverts the solid (`Internal` from the offset lanes,
    /// `InvalidInput` from the shared acceptance in `geom.rs`).
    pub const INVERTED: &str = "inverted";
    /// The motion carries a face through a neighbour it must meet
    /// (`UnsupportedGeometry`).
    pub const EDGE_INVERSION: &str = "edge_inversion";
    /// The same, across a curved edge (`UnsupportedGeometry`).
    pub const CURVED_EDGE_INVERSION: &str = "curved_edge_inversion";
    /// A turn that lays the face parallel to a neighbour (`UnsupportedGeometry`).
    pub const PARALLEL_LIMIT: &str = "parallel_limit";
    /// A turn that lays a curved face's axis parallel to a neighbour
    /// (`UnsupportedGeometry`).
    pub const AXIS_PARALLEL_LIMIT: &str = "axis_parallel_limit";
    /// A Transform Face asked to scale (`InvalidInput`).
    pub const RIGID_SCALE: &str = "scale";
    /// A Transform Face whose selection has no geometry for the default pivot
    /// (`InvalidInput`).
    pub const PIVOT_GEOMETRY: &str = "pivot_geometry";
}
use face_refusal_slugs::*;

/// Why a Transform Face refused, as far as the app explains it — read off the
/// refusal's CLASS and slug, never its text, so a reworded refusal keeps its
/// explanation and a refusal that merely mentions "parallel" does not borrow
/// one. `None` is every other refusal: the app shows the kernel's own words.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceTransformReason {
    /// The body turns inside out.
    InsideOut,
    /// A face is carried past a neighbour it has to meet.
    PastNeighbour,
    /// A TURN lays a face (or a curved face's axis) parallel to a neighbour.
    TurnParallel,
    /// A face transform asked to scale.
    RigidScale,
    /// No geometry to centre the default pivot on.
    NoPivot,
}

/// [`FaceTransformReason`] of a refusal, by class and slug.
pub fn face_transform_reason(refusal: &crate::KernelRefusal) -> Option<FaceTransformReason> {
    use crate::RefusalClass::{Internal, InvalidInput, UnsupportedGeometry};
    use FaceTransformReason::*;
    match &refusal.class {
        UnsupportedGeometry { what } if what == SOLID_INVERSION || what == SPHERE_SOLID_INVERSION => {
            Some(InsideOut)
        }
        Internal { what } | InvalidInput { what } if what == INVERTED => Some(InsideOut),
        UnsupportedGeometry { what } if what == EDGE_INVERSION || what == CURVED_EDGE_INVERSION => {
            Some(PastNeighbour)
        }
        UnsupportedGeometry { what } if what == PARALLEL_LIMIT || what == AXIS_PARALLEL_LIMIT => {
            Some(TurnParallel)
        }
        InvalidInput { what } if what == RIGID_SCALE => Some(RigidScale),
        InvalidInput { what } if what == PIVOT_GEOMETRY => Some(NoPivot),
        _ => None,
    }
}

pub use delete_face::{delete_face_and_heal, resolve_face_by_point};
pub use delete_faces::delete_faces_and_heal;
pub use refit_faces::{
    refit_faces_as_one, refit_refusal_phrases, RefitCarrier, RefitReport, RefitSurfaceKind,
};
pub use face_move::move_faces;
pub use face_rotate::rotate_faces;
// The ROUTING reading: which faces a moved selection would have to open a new
// hole in, measured before anything is built. Public so the matrix probe can
// print the partition over every cell without the road being wired to it.
pub use face_move_carrier::{route_reading, RouteReading};
// The RE-CUT road: the same edit performed from the original body and the
// motion, for a translation whose answer needs the topology to change.
pub use face_move_recut::{recut_moved_planes, recut_moved_planes_rigid};
pub use face_offset::offset_ruled_face;
pub use face_offset_sphere::offset_sphere_face;
pub use face_offset_torus::offset_torus_face;
pub use face_offset_revolution::offset_revolution_face;
pub use face_offset_freeform::offset_freeform_face;
