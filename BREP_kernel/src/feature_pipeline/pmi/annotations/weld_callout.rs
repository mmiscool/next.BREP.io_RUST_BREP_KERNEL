//! A weld callout reads its size and process from the history feature.
//! Its anchor is independent of the optional bead solid.
use super::{id_field, params, schema_entry, string_field, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::a3;
use crate::feature_pipeline::pmi::{
    format_number, PmiAnnotation, PmiContext, PmiGeometry, Resolved,
};
use crate::Vec3;

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "weldCallout",
    short_name: "WELDNOTE",
    icon: "\u{2197}",
    long_name: "\u{2197} Weld callout",
    label: "Weld callout",
    applicable: |_| false,
    schema,
    resolve,
};

fn schema() -> serde_json::Value {
    schema_entry(
        &DEF,
        params(vec![
            ("id", id_field()),
            (
                "weld",
                string_field(
                    "Weld feature",
                    "",
                    "ID of the weld feature in this document's history.",
                ),
            ),
            (
                "afterText",
                string_field("Additional text", "", "Optional fabrication note."),
            ),
        ]),
    )
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let id = annotation.text("weld").trim();
    let feature = context
        .request
        .features
        .iter()
        .find(|f| {
            f.feature_type == "WELD"
                && f.input_params.get("id").and_then(serde_json::Value::as_str) == Some(id)
        })
        .ok_or_else(|| format!("weld feature '{id}' not found"))?;
    let anchor_name = format!("{id}:Callout");
    let anchor = context
        .scene
        .resolve_point(&anchor_name)
        .ok_or_else(|| format!("weld '{id}' is not evaluated"))?;
    let size = match feature.input_params.get("legSize") {
        None => 3.0,
        Some(serde_json::Value::Number(n)) => n.as_f64().ok_or("invalid weld size")?,
        Some(serde_json::Value::String(s)) => context.env.eval(s)?,
        _ => return Err("invalid weld size".into()),
    };
    let mut text = format!("FILLET {}", format_number(size, 2));
    if let Some(process) = feature
        .input_params
        .get("process")
        .and_then(serde_json::Value::as_str)
        .filter(|s| !s.trim().is_empty())
    {
        text.push_str(&format!("\n{}", process.trim()));
    }
    let extra = annotation.text("afterText").trim();
    if !extra.is_empty() {
        text.push_str(&format!("\n{extra}"));
    }
    Ok(Resolved {
        text,
        value: Some(size),
        unit: "",
        references: vec![anchor_name],
        geometry: PmiGeometry::Leader {
            targets: vec![a3(anchor)],
            dot: false,
            balloon: false,
            anchor: None,
        },
        default_label: a3(anchor.add(Vec3::new(5.0, 5.0, 0.0))),
    })
}
