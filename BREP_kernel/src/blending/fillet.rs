//! Edge fillets and chamfers (Golovanov §4.9, §6.9–§6.11).
//!
//! The construction is the rolling-ball march with direct topology surgery
//! (`crate::blend`): the ball's tangency system is solved station by station,
//! the blend surface and its two contact rails are fitted through the
//! stations — exactly, as a cylinder patch or a revolution, where the
//! stations are rigid translates or rotations of one section — and the
//! mates are re-trimmed to the rails.  A whole selection is one job for the
//! stripe network (`blend/network.rs`): every stripe is marched against the
//! original solid and every shared vertex is solved before anything is cut —
//! a star is closed by a patch of the corner ball, a two-edge corner by the
//! seam between the two blends, a re-entrant corner by a horn torus, a
//! tangent pair by a flush join, and an unselected tangent continuation by a
//! cap.  No tool solid, no boolean.
//!
//! The older construction survives as the FALLBACK: build the §6.9
//! cross-section as a tool solid, extrude or revolve it along the edge, and
//! let the boolean perform the surgery (`fillet/tool.rs`). It is reached
//! where the march refuses by name, and by the group's sequential
//! composition (`Lane::CutterFirst`), whose remaining corner closures — the
//! mixed-convexity torus sectors and the N≥4 no-common-ball Coons fill —
//! reconstruct the corner from the cutter's output. Both are on their way
//! out.
//!
//! Diagnostic environment switches (all off by default, none change a
//! result that reaches the user through a different lane than the one
//! named):
//!   * `BREP_DEBUG_NETWORK=1` prints every network refusal and the seam
//!     residuals;
//!   * `BREP_NO_NETWORK=1` skips the network attempt so a selection can be
//!     compared against the composition;
//!   * `BREP_CUTTER_FIRST=1` restores the cutter's first turn on single
//!     edges for the same comparison;
//!   * `BREP_NO_THIRD_FACE_TRIM=1` skips the network's own restriction against
//!     a third face (`blend/restrict.rs`), so a selection can be compared
//!     against the cutter fallback it used to take, and so the restriction's
//!     own cost can be measured against a run without it.

use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::{
    boolean_operation, extrude_profile_brep, make_arc, make_line, revolve_profile_brep,
    BooleanOperation, BooleanOptions, NurbsCurve, Vec3,
};
use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};

#[path = "fillet/analyze.rs"]
mod analyze;
#[path = "fillet/edges.rs"]
mod edges;
#[path = "fillet/tool.rs"]
mod tool;

pub use edges::{
    chamfer_edge, chamfer_edge_angle, chamfer_edge_asymmetric, chamfer_edges_angle,
    chamfer_edges_asymmetric, fillet_edge, fillet_edges, fillet_edges_reported,
    fillet_edges_variable, fillet_edges_variable_law, fillet_edges_variable_vertex_radii,
    BlendSelection,
};
pub(crate) use analyze::into_face_direction;

use analyze::{
    analyze_edge, check_blend_interference, check_loop_self_crossings, check_mixed_concavity,
    check_support_extent, EdgeCross,
    EdgePath,
};
use edges::heal_edge_vertex_gaps;
use tool::{
    apply_tool, chamfer_angle_second_distance, chamfer_cross_section_offsets, fillet_or_chamfer,
    scaffold_tag, settle_cutter_scaffold, tool_ends_to_original_extent, Lane, ToolEnds,
    CUTTER_SCAFFOLD_NAME,
};
#[allow(unused_imports)] // consumed by the corner-closure lanes' cfg(test) siblings
pub(crate) use edges::fillet_edge_cutter;
