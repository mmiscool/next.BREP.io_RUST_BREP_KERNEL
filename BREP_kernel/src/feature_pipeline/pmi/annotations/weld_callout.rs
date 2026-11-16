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
        ].into_iter().chain(crate::feature_pipeline::pmi::weld_symbol::symbol_fields()).collect()),
    )
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let id = annotation.text("weld").trim();
    let feature = context
        .request
        .features
        .iter()
        .find(|f| {
            crate::feature_pipeline::features::weld::is_weld_type(&f.feature_type)
                && f.input_params.get("id").and_then(serde_json::Value::as_str) == Some(id)
        })
        .ok_or_else(|| format!("weld feature '{id}' not found"))?;
    // A weld symbol number that does not evaluate is the row's error, never
    // a silently empty field.
    symbol(annotation, context)?;
    let anchor_name = format!("{id}:Callout");
    let anchor = context
        .scene
        .resolve_point(&anchor_name)
        .ok_or_else(|| format!("weld '{id}' is not evaluated"))?;
    let number = |key: &str, default: f64| -> Result<f64, String> {
        match feature.input_params.get(key) {
            None => Ok(default),
            Some(serde_json::Value::Number(n)) => n.as_f64().ok_or_else(|| format!("invalid weld {key}")),
            Some(serde_json::Value::String(s)) => context.env.eval(s),
            _ => Err(format!("invalid weld {key}")),
        }
    };
    // Each weld kind's callout names its kind and its size.
    let (mut text, size) = match feature.feature_type.as_str() {
        "WELD.GROOVE" => {
            let shape = feature.input_params.get("shape").and_then(serde_json::Value::as_str).unwrap_or("V");
            let double = feature.input_params.get("double").and_then(serde_json::Value::as_bool).unwrap_or(false);
            let angle = number("grooveAngle", 60.0)?;
            let kind = format!("{}{shape} GROOVE", if double { "DOUBLE " } else { "" });
            if shape == "SQUARE" {
                (kind, None)
            } else {
                (format!("{kind} {}°", format_number(angle, 0)), Some(angle))
            }
        }
        "WELD.SPOT" => {
            let diameter = number("diameter", 5.0)?;
            (format!("SPOT \u{2300}{}", format_number(diameter, 2)), Some(diameter))
        }
        "WELD.RSEAM" => {
            let width = number("width", 5.0)?;
            (format!("SEAM {}", format_number(width, 2)), Some(width))
        }
        "WELD.PLUG" => ("PLUG".to_string(), None),
        "WELD.EDGE" => {
            let height = number("reinforcement", 1.0)?;
            (format!("EDGE {}", format_number(height, 2)), Some(height))
        }
        "WELD.CORNER" => {
            let size = number("size", 3.0)?;
            (format!("CORNER {}", format_number(size, 2)), Some(size))
        }
        "WELD.SURF" => {
            let thickness = number("thickness", 2.0)?;
            (format!("SURFACING {}", format_number(thickness, 2)), Some(thickness))
        }
        "WELD.TACK" => {
            let size = number("legSize", 3.0)?;
            (format!("TACK {}", format_number(size, 2)), Some(size))
        }
        _ => {
            let size = number("legSize", 3.0)?;
            (format!("FILLET {}", format_number(size, 2)), Some(size))
        }
    };
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
        value: size,
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

/// The weld symbol a callout draws, or `None` (the text style, a weld kind
/// with no symbol). Built from the callout's fields and the weld feature's
/// kind and parameters; `n` (how many segments, tacks or spots one side
/// has) is counted from the weld's outputs in the scene.
pub fn symbol(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Option<crate::feature_pipeline::pmi::weld_symbol::WeldSymbol>, String> {
    let id = annotation.text("weld").trim();
    let Some(feature) = context.request.features.iter().find(|f| {
        crate::feature_pipeline::features::weld::is_weld_type(&f.feature_type)
            && f.input_params.get("id").and_then(serde_json::Value::as_str) == Some(id)
    }) else {
        return Ok(None);
    };
    let evaluate = |source: &str| context.env.eval(source).ok();
    // One side's count, as the weld lays it from its seam's length.
    let seam = context.scene.paths.get(&format!("{id}:Seam")).map_or(0.0, |curves| crate::feature_pipeline::pmi::weld_symbol::path_length(curves));
    let segments = crate::feature_pipeline::pmi::weld_symbol::segments(&feature.feature_type, &feature.input_params, seam, &evaluate);
    crate::feature_pipeline::pmi::weld_symbol::WeldSymbol::from_params(&annotation.params, &feature.feature_type, &feature.input_params, &evaluate, segments)
}
