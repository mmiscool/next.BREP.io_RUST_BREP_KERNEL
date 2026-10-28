//! Adaptive (constant-engagement) clearing, 3D pocket clearing and rest
//! roughing (lane C6). Everything here works on the planar engine's exact
//! regions (`crate::planar`): the material left at a Z level is a region, the
//! tool-centre feasible set is a region, and the radial engagement of the
//! tool at a position is the exact angular measure of the tool circle inside
//! the remaining material ([`engagement`](mod@engagement)).
//!
//! - [`engagement`](mod@engagement): the engagement model (row 1): a uniform grid over the
//!   material boundary, exact circle/boundary crossings classified by the
//!   boundary's orientation, angular interval arithmetic, and the current
//!   pass's own swept stadiums subtracted so the tool never "sees" material
//!   it has already removed.
//! - [`clear`]: the level clearing solver (row 2): fronts grown from the
//!   cleared tool-centre set by the optimal load, trochoidal loops where the
//!   front's engagement would exceed the limit, exact material bookkeeping
//!   per pass and a coverage certificate.
//! - [`dilate`]: dilation and erosion of regions by a disc and swept regions
//!   of chains, from unions of single-segment capsules with area checks.
//! - [`levels`]: Z levels with flat-area detection, part shadows, stock
//!   slices (box, cylinder, mesh, simulated dexel stock) and rest regions
//!   (rows 3 to 5).
//!
//! Written from the engagement-angle literature (trochoidal milling and
//! constant-engagement pocketing) without reference to other CAM code.

pub mod clear;
pub mod dilate;
pub mod engagement;
pub mod levels;
pub mod roughing;
pub mod strategy_support;

pub use clear::{clear_level, deepest_point, ClearInput, ClearParams, Entry, LevelPass, LevelResult};
pub use dilate::{capsule_loop, circle_loop, disc, grow, merge_pieces, polygonal, shrink, swept_region};
pub use engagement::{circle_inside, engagement, Intervals, SegmentGrid, Stadium, TAU};

