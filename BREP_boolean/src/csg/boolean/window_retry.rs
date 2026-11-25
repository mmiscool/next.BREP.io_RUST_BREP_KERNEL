//! A refused Boolean retried with its operands' curved faces on their own
//! windows.
//!
//! A face trimmed on a freeform patch that runs on past it (folding back over
//! the face, or to a second layer close beside it) gives the Boolean a carrier
//! whose every nearest-point read near the face has two answers: a pierce, a
//! section pcurve or a refit can land on the other layer, outside every trim,
//! and the centre of the carrier's domain can be a fold where it has no
//! normal. The face's window ([`crate::imprint::restricted_carrier`]: the
//! carrier cut down by exact knot insertion to its trims' parameter box, in the
//! carrier's own parameters) is the same surface over the face, so the face's
//! pcurves stand on it unchanged, and the other layer is not on it.

use crate::{BooleanOperation, BooleanOptions, BrepSolid, KernelRefusal};

/// `BREP_BOOLEAN_WINDOW_RETRY` (default on; `=0` restores the first refusal): [`retry`] answers a refused
/// Boolean whose operands have a face to re-carry. The result's re-carried
/// faces keep their windows as their surfaces (the same geometry over each
/// face, in the same parameters).
fn window_retry_on() -> bool {
    crate::switch!("BREP_BOOLEAN_WINDOW_RETRY", true)
}

/// `BREP_BOOLEAN_WINDOW_RETRY_CHECK`: say, for every refused Boolean with a
/// face to re-carry, whether the retry builds (stderr), without changing the
/// outcome.
fn window_retry_check() -> bool {
    crate::debug_flag!("BREP_BOOLEAN_WINDOW_RETRY_CHECK")
}

/// `solid` with each curved freeform face whose window is smaller than its
/// carrier re-carried on the window, or `None` when it has no such face. A
/// plane (an affine patch), an analytic carrier (its closed forms are read
/// from the carrier) and a lifted window (its parameters are not the
/// carrier's) are left as they are.
fn windowed(solid: &BrepSolid) -> Option<BrepSolid> {
    let mut out = solid.clone();
    let mut changed = 0usize;
    for face in out.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
        if face.surface.analytic().is_some() || face.surface.is_affine().unwrap_or(true) {
            continue;
        }
        let Some(window) = crate::imprint::restricted_carrier(face) else { continue };
        if window.lifted {
            continue;
        }
        face.surface = window.surface;
        changed += 1;
    }
    (changed > 0).then(|| {
        out.mass_properties_cache = Default::default();
        out
    })
}

/// `refused`, the Boolean's refusal of `first` `operation` `second`, or under
/// [`window_retry_on`] the answer `run` gives on the operands with their faces
/// re-carried on their windows. Where neither operand has a face to re-carry,
/// or the retry is refused too, `refused` stands word for word.
pub(super) fn retry(
    first: &BrepSolid,
    second: &BrepSolid,
    operation: BooleanOperation,
    options: &BooleanOptions,
    refused: KernelRefusal,
    run: impl FnOnce(&BrepSolid, &BrepSolid, BooleanOperation, &BooleanOptions) -> Result<BrepSolid, KernelRefusal>,
) -> Result<BrepSolid, KernelRefusal> {
    let (on, check) = (window_retry_on(), window_retry_check());
    if !(on || check) {
        return Err(refused);
    }
    let (left, right) = (windowed(first), windowed(second));
    if left.is_none() && right.is_none() {
        return Err(refused);
    }
    let answer = run(left.as_ref().unwrap_or(first), right.as_ref().unwrap_or(second), operation, options);
    if check {
        match &answer {
            Ok(_) => eprintln!("BOOLEAN_WINDOW_RETRY check: refused ({refused}); on the windows it builds"),
            Err(other) => eprintln!("BOOLEAN_WINDOW_RETRY check: refused ({refused}); on the windows refused too ({other})"),
        }
    }
    match answer {
        Ok(solid) if on => Ok(solid),
        _ => Err(refused),
    }
}
