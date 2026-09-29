//! The offset shell's own refusals, minted as TYPED `KernelRefusal`s at their
//! origin with stable slugs (census item 4, 2026-09-27), and the predicates
//! `features/offset_shell.rs` dispatches on — the class and slug, never the
//! text:
//!
//! | feature branch | predicate | class |
//! |---|---|---|
//! | every face selected: warn and no-op | `is_offset_shell_no_retained_face` | `InvalidInput { what: "offset_shell_no_retained_face" }` |
//! | an open curved rim: the reorder advice | `is_offset_shell_open_curved_rim` | `NonIntegralGenus` (typed at `finalize_assembled_solid`, carried through every cause put in front of it) or `UnsupportedGeometry { what: "offset_shell_unwelded_curved_rim" }` |
//!
//! plus the outward guard's `Internal { what: "offset_shell_empty_shell" }`
//! and the sharp-side lane's `UnsupportedGeometry { what:
//! "offset_shell_sharp_side_continuation" }`, which no feature branch matches.

use crate::{KernelRefusal, KernelStage};

pub(super) const NO_RETAINED_FACE: &str = "offset_shell_no_retained_face";
pub(super) const UNWELDED_CURVED_RIM: &str = "offset_shell_unwelded_curved_rim";
pub(super) const EMPTY_SHELL: &str = "offset_shell_empty_shell";
/// The sharp-side lane could not continue a curved carrier past its sharp
/// outward side (`sharp_side_continued_source`): a named deferral.
pub(super) const SHARP_SIDE_CONTINUATION: &str = "offset_shell_sharp_side_continuation";

/// Every face was selected as an opening, so no support is left to offset.
pub(super) fn no_retained_face() -> KernelRefusal {
    KernelRefusal::input(
        KernelStage::Collect,
        NO_RETAINED_FACE,
        "offset_shell: removing every face cannot produce a shell",
    )
}

/// A one-use closed rim the welds could not close on a curved-solid opening.
pub(super) fn unwelded_curved_rim(edge_id: u64, anchor: crate::Vec3) -> KernelRefusal {
    KernelRefusal::unsupported(
        KernelStage::Sew,
        UNWELDED_CURVED_RIM,
        format!(
            "offset_shell: unwelded rim leaves a non-watertight shell \
             (one-use closed edge {} near ({:.3},{:.3},{:.3})); this \
             curved-solid opening is not yet supported",
            edge_id, anchor.x, anchor.y, anchor.z
        ),
    )
}

/// An outward result with shells that enclose no volume
/// (`outward_shell_is_connected`).
pub(super) fn empty_shell(shells: usize, empty: usize, floor: f64) -> KernelRefusal {
    // Internal, not `InvalidResultTopology`: that class is backed as an
    // arrangement degeneracy and so is perturbation-eligible (the boolean's
    // retry contract), which a builder's own verdict must not inherit, the
    // same ruling the loft's "failed validation" sites took.
    KernelRefusal::internal(
        KernelStage::Validate,
        EMPTY_SHELL,
        format!(
            "offset_shell: the outward shell assembled {shells} shells, {empty} of which enclose no volume \
             (under {floor:.1e}); those are fragments of an assembly that cannot be built as it stands"
        ),
    )
}

/// Whether `refusal` is the shell's "every face was selected" refusal: the
/// class and slug, never the text.
pub(crate) fn is_offset_shell_no_retained_face(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, crate::RefusalClass::InvalidInput { what } if what == NO_RETAINED_FACE)
}

/// Whether `refusal` is an open curved rim the shell could not close: the
/// assembly's `NonIntegralGenus` (typed at `finalize_assembled_solid`, carried
/// through every cause put in front of it) or the unwelded curved rim. The
/// class, never the text; a genus sentence quoted under another class is not
/// this refusal.
pub(crate) fn is_offset_shell_open_curved_rim(refusal: &KernelRefusal) -> bool {
    match &refusal.class {
        crate::RefusalClass::NonIntegralGenus { .. } => true,
        crate::RefusalClass::UnsupportedGeometry { what } => what == UNWELDED_CURVED_RIM,
        _ => false,
    }
}
