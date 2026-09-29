//! Explode body (`EXP`): a display-only pose delta on solids / components
//! while the owning view is active. Never touches ACOMP transforms or the
//! assembly solver; the base pose is the modeling pose at apply time.

use super::{boolean_field, id_field, params, reference_field, schema_entry, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, solid_bbox};
use crate::feature_pipeline::pmi::{PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use crate::feature_pipeline::SelectionProbe;
use crate::Vec3;

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "explode",
    short_name: "EXP",
    icon: "\u{29C9}",
    long_name: "\u{29C9} Explode body",
    label: "Explode body",
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
                reference_field("Targets", &["SOLID", "COMPONENT"], true, 1, 64, "The solids or components to move"),
            ),
            (
                "transform",
                serde_json::json!({
                    "type": "transform",
                    "default_value": { "position": [0, 0, 0], "rotationEuler": [0, 0, 0], "scale": [1, 1, 1] },
                    "hint": "The pose delta applied while the view is active"
                }),
            ),
            ("showTraceLine", boolean_field("Trace line", true, "Draw a dashed line from the original to the exploded position")),
        ]),
    )
}

fn triple(value: Option<&serde_json::Value>, default: [f64; 3]) -> [f64; 3] {
    let Some(items) = value.and_then(serde_json::Value::as_array) else {
        return default;
    };
    let mut out = default;
    for (slot, item) in out.iter_mut().zip(items) {
        if let Some(number) = item.as_f64() {
            *slot = number;
        } else if let Some(text) = item.as_str() {
            if let Ok(number) = text.trim().parse::<f64>() {
                *slot = number;
            }
        }
    }
    out
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("targets");
    if targets.is_empty() {
        return Err("select at least one solid or component".into());
    }
    let mut solids: Vec<(String, u32)> = Vec::new();
    for target in &targets {
        if let Some(handle) = context.scene.resolve_solid(target) {
            solids.push((target.clone(), handle));
        } else if crate::is_component_reference(crate::split_component_namespace(target).1) {
            let members = context.scene.component_solids(target);
            if members.is_empty() {
                return Err(format!("component '{target}' has no solids"));
            }
            solids.extend(members);
        } else {
            return Err(format!("'{target}' is not a solid or component"));
        }
    }
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for (_, handle) in &solids {
        let (lo, hi) = solid_bbox(*handle)?;
        low = Vec3::new(low.x.min(lo.x), low.y.min(lo.y), low.z.min(lo.z));
        high = Vec3::new(high.x.max(hi.x), high.y.max(hi.y), high.z.max(hi.z));
    }
    let center = low.add(high).scale(0.5);
    let transform = annotation.params.get("transform");
    let translate = triple(transform.and_then(|t| t.get("position")), [0.0; 3]);
    let rotate_deg = triple(transform.and_then(|t| t.get("rotationEuler")), [0.0; 3]);
    let scale = triple(transform.and_then(|t| t.get("scale")), [1.0; 3]);
    let exploded = center.add(Vec3::new(translate[0], translate[1], translate[2]));
    let names: Vec<String> = solids.into_iter().map(|(name, _)| name).collect();
    Ok(Resolved {
        text: format!("Explode {}", names.join(", ")),
        value: None,
        unit: "",
        references: names.clone(),
        geometry: PmiGeometry::Explode {
            solids: names,
            translate,
            rotate_deg,
            scale,
            center: a3(center),
            trace: annotation
                .params
                .get("showTraceLine")
                .and_then(serde_json::Value::as_bool)
                .unwrap_or(true),
        },
        default_label: a3(exploded),
    })
}
