//! The guided loft along a guide that is a chain of curves.
//!
//! The guided loft ([`crate::loft_profile_brep_guided`] and
//! [`crate::loft_profile_brep_guided_frame`]) bends its sections along ONE
//! spine curve. A guide drawn as several curves that meet smoothly (a line
//! into a fillet arc into a line) is such a spine: the chain is joined into
//! one curve ([`crate::join_curve_chain`], exactly, each piece on a stretch of
//! the parameter as long as the piece) and lofted along. A chain with a corner
//! is not a spine: the loft's skin passes smoothly through its stations and
//! would round the corner off, so it is refused by name with the angle
//! measured there.
use crate::{join_curve_chain, loft_profile_brep_guided, loft_profile_brep_guided_frame, BrepSolid, JointContinuity, KernelRefusal, KernelStage, NurbsCurve, SweepPath};

/// The loft of `sections` along the guide `guide`, a chain of curves that
/// follow one another head to tail and meet smoothly; `rotate_to_frame` turns
/// the sections with the guide's moving frame
/// ([`crate::loft_profile_brep_guided_frame`]) instead of keeping their own
/// orientation ([`crate::loft_profile_brep_guided`]). A chain of one curve is
/// that curve.
///
/// Refuses, by name: an empty chain (the path's own refusal), curves that do
/// not meet (`guide_chain`), a chain that closes on itself (`guide_closed`), and a joint where
/// the tangent turns (`guide_corner`, with the angle).
pub fn loft_profile_brep_guided_chain(sections: &[Vec<NurbsCurve>], guide: &[NurbsCurve], name: Option<&str>, rotate_to_frame: bool) -> Result<BrepSolid, KernelRefusal> {
    let loft = |spine: &NurbsCurve| if rotate_to_frame { loft_profile_brep_guided_frame(sections, spine, name) } else { loft_profile_brep_guided(sections, spine, name) };
    if let [spine] = guide {
        return loft(spine);
    }
    let path = SweepPath::new(guide.to_vec(), Vec::new()).map_err(|error| error.with_message(|message| format!("guidedLoft: guide chain: {message}")))?;
    if path.closed {
        return Err(KernelRefusal::input(KernelStage::Classify, "guide_closed", "guidedLoft: the guide chain closes on itself; a guide runs from the first section to the last"));
    }
    for joint in &path.joints {
        if joint.continuity == JointContinuity::G0 {
            return Err(KernelRefusal::input(
                KernelStage::Classify,
                "guide_corner",
                format!(
                    "guidedLoft: the guide turns {:.3}° where its curve {} follows curve {}; the curves of a guide chain must meet smoothly (a loft along a corner would round it off)",
                    joint.tangent_break.to_degrees(),
                    joint.after,
                    joint.before
                ),
            ));
        }
    }
    let spine = join_curve_chain(&path.curves).map_err(|message| KernelRefusal::input(KernelStage::Collect, "guide_chain", format!("guidedLoft: guide chain: {message}")))?;
    loft(&spine)
}

