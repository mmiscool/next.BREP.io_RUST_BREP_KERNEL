//! Hide body (`HID`): a display-only step that keeps solids / components out
//! of the owning view while it is active — the visibility counterpart of the
//! explode step, recorded, listed, edited and undone the same way. Nothing is
//! drawn for it: no chip in the 3D view, no text on a sheet. The viewport,
//! the sheets, the 3D PDF and the instructions pictures all read the same
//! rule, [`PmiView::hides`](crate::feature_pipeline::pmi::PmiView::hides).

use super::{id_field, params, reference_field, schema_entry, target_solids, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, solid_bbox};
use crate::feature_pipeline::pmi::{PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use crate::feature_pipeline::SelectionProbe;
use crate::Vec3;

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "hide",
    short_name: "HID",
    icon: "\u{E0D4}",
    long_name: "\u{E0D4} Hide body",
    label: "Hide body",
    applicable,
    schema,
    resolve,
};

/// One or more solids / components.
fn applicable(probe: &SelectionProbe) -> bool {
    probe.solids + probe.components >= 1
}

fn schema() -> serde_json::Value {
    schema_entry(
        &DEF,
        params(vec![
            ("id", id_field()),
            (
                "targets",
                reference_field("Targets", &["SOLID", "COMPONENT"], true, 1, 64, "The solids or components hidden while the view is active"),
            ),
        ]),
    )
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    // A view's baseline step (the solids hidden at capture) skips a name
    // that no longer resolves, as the captured list always did; an author's
    // step names what it hides, so a stale target is its error.
    let solids = if annotation.is_baseline_hide() {
        let mut solids: Vec<(String, u32)> = Vec::new();
        for target in annotation.references("targets") {
            for solid in target_solids(std::slice::from_ref(&target), context).unwrap_or_default() {
                if !solids.iter().any(|(name, _)| *name == solid.0) {
                    solids.push(solid);
                }
            }
        }
        solids
    } else {
        target_solids(&annotation.references("targets"), context)?
    };
    if solids.is_empty() {
        return Ok(Resolved {
            text: "Hide (nothing present)".into(),
            value: None,
            unit: "",
            references: Vec::new(),
            geometry: PmiGeometry::Hide { solids: Vec::new() },
            default_label: [0.0; 3],
        });
    }
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for (_, handle) in &solids {
        let (lo, hi) = solid_bbox(*handle)?;
        low = Vec3::new(low.x.min(lo.x), low.y.min(lo.y), low.z.min(lo.z));
        high = Vec3::new(high.x.max(hi.x), high.y.max(hi.y), high.z.max(hi.z));
    }
    let center = low.add(high).scale(0.5);
    let names: Vec<String> = solids.into_iter().map(|(name, _)| name).collect();
    Ok(Resolved {
        text: format!("Hide {}", names.join(", ")),
        value: None,
        unit: "",
        references: names.clone(),
        geometry: PmiGeometry::Hide { solids: names },
        default_label: a3(center),
    })
}
