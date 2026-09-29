//! SM.UNFOLD — flatten a sheet-metal body to its flat pattern.
//!
//! Evaluates the sheet tree at `fold = 0`, producing an exact flat BREP with
//! bends developed into neutral-fiber allowance strips. The folded body remains.
//!
//! Contract: `sheet` — the target sheet body name (reference_selection [SOLID]).
//!
//! ## Bend-line annotations (the flat pattern's naming contract)
//!
//! Every opened bend leaves its allowance strip in the flat pattern as NAMED
//! band faces — plain geometry the app renders and selects today, no metadata
//! side-channel:
//!
//! * `{bendId}:BAND:{DIR}:{angle}:A` / `…:B` — the band's top / bottom face
//!   (`A` = sheet +normal side, matching plate face naming). `DIR` ∈
//!   `{UP, DOWN}`: `DOWN` = the folded part bends toward the sheet's B side
//!   (positive authored angle), `UP` = toward A (negative angle). `{angle}` =
//!   the absolute fold angle in degrees (`f64` Display — shortest exact
//!   round-trip, so an expression-driven angle prints all its digits).
//! * `{bendId}:BAND:{DIR}:{angle}:SIDE:s{0,2}` — the band's surviving end
//!   walls on the sheet rim.
//! * The bend's two FOLD LINES are the band faces' boundary edges shared with
//!   the parent/child plate faces. The flat-pattern union keeps coplanar faces
//!   un-merged so these edges exist, and this op stamps the derived
//!   `{faceA}|{faceB}[n]` edge names (sorted face names), e.g.
//!   `SM.F1:bend:BAND:DOWN:90:A|SM.TAB1:flat_root:A[0]` — selectable like any
//!   solid edge.
//!
//! Special cases: a 0° bend has no allowance (the flats abut
//! — no band, no annotation); a circular-hole COLLAR is not developable (a
//! drawn collar stretches material), so it contributes NOTHING at fold = 0 —
//! the flat pattern shows only the inset-enlarged opening.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::sheet_metal;
use crate::feature_pipeline::{FeatureContext, FeatureResult};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let Some(body_name) = ctx
        .param("sheet")
        .and_then(|v| v.as_array())
        .and_then(|a| a.iter().find_map(|e| e.as_str().map(str::to_string)))
    else {
        return Ok(FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone()));
    };
    let Some(handle) = ctx.scene.resolve_solid(&body_name) else {
        let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
        result.unresolved.push(body_name);
        return Ok(result);
    };
    let Some(tree) = sheet_metal::get_tree(handle) else {
        return Err("sheet-metal unfold: target is not a sheet-metal body".into());
    };

    // fold = 0 → the flat pattern (same evaluator, bends opened flat).
    // `register_added` dedupes face names and stamps the derived
    // `{faceA}|{faceB}[n]` edge names — that pass is what makes the bend-band
    // FOLD LINES (band-face ↔ plate-face edges) selectable by name.
    let mut flat = sheet_metal::evaluate(&tree, 0.0)?;
    // The flat pattern is a COPY of the folded body's geometry and both stay
    // resident (`removed` is empty — see the test). Suffix the flat body's
    // face/edge names with its own id so they don't collide with the folded
    // body's names (the naming-contract guard forbids two resident solids
    // sharing a name). Export reads the body by handle, not by these names.
    common::namespace_copy_names(&mut flat, &ctx.id);
    let added = common::register_added(flat, &ctx.id);
    sheet_metal::put_tree(added.handle, tree); // the flat pattern can be re-folded

    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.added.push(added);
    Ok(result)
}

