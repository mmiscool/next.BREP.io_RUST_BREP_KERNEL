//! Hole callout (`HOLE`): `⌀d ↧depth` / `THRU ALL` / countersink ⌵ /
//! counterbore ⌴ read from the owning Hole feature, with an optional
//! quantity (0 = count the feature's holes).
//!
//! A hole wall is named `{sketchId}:P{pid}_Hole_S` (the Hole feature's
//! placement sketch + point); the feature is the `H` whose `face` names that
//! sketch, and its params are read from the request at tail time. A
//! cylindrical face that is not a Hole feature's wall (a component hole, an
//! extruded circle) falls back to a bare `⌀d` from the face metadata.

use super::{boolean_field, id_field, number_field, params, reference_field, schema_entry, string_field, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, resolve_reference};
use crate::feature_pipeline::pmi::{format_number, PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use crate::feature_pipeline::{FeatureDescriptor, SelectionProbe};
use crate::{SelectionGeometry, Vec3};

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "holeCallout",
    short_name: "HOLE",
    icon: "\u{2334}",
    long_name: "\u{2334} Hole callout",
    label: "Hole callout",
    applicable,
    schema,
    resolve,
};

/// Exactly one face or edge.
fn applicable(probe: &SelectionProbe) -> bool {
    probe.faces + probe.edges == 1
        && probe.vertices == 0
        && probe.planes == 0
        && probe.solids == 0
        && probe.sketches == 0
}

fn schema() -> serde_json::Value {
    schema_entry(
        &DEF,
        params(vec![
            ("id", id_field()),
            (
                "target",
                reference_field("Target", &["FACE", "EDGE"], false, 1, 1, "The hole's wall face, or a circular edge on it"),
            ),
            ("quantity", number_field("Quantity", 0.0, 1.0, "0 = count the holes of the same feature automatically")),
            ("showQuantity", boolean_field("Show quantity", true, "Prefix the callout with `N×`")),
            ("beforeText", string_field("Before", "", "Text before the callout")),
            ("afterText", string_field("After", "", "Text after the callout")),
        ]),
    )
}

/// The hole wall face name behind a target: the face itself, or the wall
/// among a derived edge name's two faces.
fn wall_face_name(target: &str) -> Option<String> {
    if target.contains("_Hole_S") && !target.contains('|') {
        return Some(target.to_string());
    }
    let (body, _) = target.rsplit_once('[').unwrap_or((target, ""));
    body.split('|')
        .find(|part| part.contains("_Hole_S"))
        .map(String::from)
}

/// `{prefix}{sketch}:P{pid}_Hole_S` → (component prefix chain, sketch id, point id).
fn parse_wall(name: &str) -> Option<(String, String, String)> {
    let (chain, local) = crate::split_component_namespace(name);
    let stem = local.strip_suffix("_Hole_S")?;
    let (sketch, point) = stem.rsplit_once(':')?;
    let prefix = if chain.is_empty() {
        String::new()
    } else {
        format!("{}:", chain.join(":"))
    };
    Some((prefix, sketch.to_string(), point.to_string()))
}

/// The `H` feature placed by `sketch_id` (top-level features only).
fn hole_feature<'a>(request: &'a crate::feature_pipeline::HistoryRequest, sketch_id: &str) -> Option<&'a FeatureDescriptor> {
    request.features.iter().find(|feature| {
        if feature.feature_type != "H" {
            return false;
        }
        // The placement sketch: a string or a one-element list.
        let face = feature.input_params.get("face");
        let named = match face {
            Some(serde_json::Value::String(name)) => Some(name.trim()),
            Some(serde_json::Value::Array(items)) => items.first().and_then(serde_json::Value::as_str).map(str::trim),
            _ => None,
        };
        named == Some(sketch_id)
    })
}

/// A numeric hole param (expressions allowed).
fn hole_number(feature: &FeatureDescriptor, key: &str, context: &PmiContext<'_>, default: f64) -> f64 {
    match feature.input_params.get(key) {
        Some(serde_json::Value::Number(n)) => n.as_f64().unwrap_or(default),
        Some(serde_json::Value::String(s)) => context.env.eval(s.trim()).unwrap_or(default),
        _ => default,
    }
}

/// The hole descriptor text lines for a Hole feature.
fn describe_hole(feature: &FeatureDescriptor, context: &PmiContext<'_>, decimals: usize) -> String {
    let hole_type = feature
        .input_params
        .get("holeType")
        .and_then(serde_json::Value::as_str)
        .unwrap_or("SIMPLE")
        .to_ascii_uppercase();
    let diameter = hole_number(feature, "diameter", context, 0.0);
    let depth = hole_number(feature, "depth", context, 0.0);
    let through = feature
        .input_params
        .get("throughAll")
        .and_then(serde_json::Value::as_bool)
        .unwrap_or(false);
    let depth_text = if through {
        " THRU ALL".to_string()
    } else {
        format!(" \u{21A7}{}", format_number(depth, decimals))
    };
    let mut text = format!("\u{2300}{}{depth_text}", format_number(diameter, decimals));
    match hole_type.as_str() {
        "THREADED" => text.push_str(" THD"),
        "COUNTERSINK" => {
            let sink_dia = hole_number(feature, "countersinkDiameter", context, 0.0);
            let sink_angle = hole_number(feature, "countersinkAngle", context, 90.0);
            text.push_str(&format!(
                "\n\u{2335} \u{2300}{} \u{00D7} {}\u{00B0}",
                format_number(sink_dia, decimals),
                format_number(sink_angle, 0)
            ));
        }
        "COUNTERBORE" => {
            let bore_dia = hole_number(feature, "counterboreDiameter", context, 0.0);
            let bore_depth = hole_number(feature, "counterboreDepth", context, 0.0);
            text.push_str(&format!(
                "\n\u{2334} \u{2300}{} \u{21A7}{}",
                format_number(bore_dia, decimals),
                format_number(bore_depth, decimals)
            ));
        }
        _ => {}
    }
    text
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("target");
    let Some(target) = targets.first() else {
        return Err("select a hole wall face or a circular edge on it".into());
    };
    let geometry = resolve_reference(context.scene, target)?;
    let (anchor, normal, radius) = match geometry {
        SelectionGeometry::Axis {
            origin,
            direction,
            radius,
        } => (origin, direction, radius),
        SelectionGeometry::Circle { center, axis, radius } => (center, axis, Some(radius)),
        other => {
            return Err(format!(
                "'{target}' is not a hole: pick the cylindrical wall or a circular edge ({} found)",
                crate::feature_pipeline::pmi::resolve::class_of(&other)
            ))
        }
    };
    let decimals = 2;
    let wall = wall_face_name(target);
    let parsed = wall.as_deref().and_then(parse_wall);
    let feature = parsed
        .as_ref()
        .filter(|(prefix, _, _)| prefix.is_empty())
        .and_then(|(_, sketch, _)| hole_feature(context.request, sketch));
    let mut body = match (feature, parsed.as_ref()) {
        (Some(feature), _) => describe_hole(feature, context, decimals),
        _ => match radius {
            Some(radius) => format!("\u{2300}{}", format_number(radius * 2.0, decimals)),
            None => return Err(format!("'{target}' has no constant diameter")),
        },
    };
    // Quantity: the explicit count, or every wall of the same feature in the scene.
    let explicit = annotation.number("quantity", context.env, 0.0)?.round().max(0.0) as usize;
    let count = if explicit > 0 {
        explicit
    } else if let Some((prefix, sketch, _)) = &parsed {
        let stem = format!("{prefix}{sketch}:P");
        let mut points: Vec<&str> = context
            .scene
            .faces
            .keys()
            .filter_map(|name| {
                let rest = name.strip_prefix(&stem)?;
                let point = rest.strip_suffix("_Hole_S")?;
                (!point.contains(':')).then_some(point)
            })
            .collect();
        points.sort_unstable();
        points.dedup();
        points.len().max(1)
    } else {
        1
    };
    if annotation.flag("showQuantity") && count > 1 {
        body = format!("{count}\u{00D7} {body}");
    }
    let before = annotation.text("beforeText");
    let after = annotation.text("afterText");
    let text = format!("{}{}{}", if before.is_empty() { String::new() } else { format!("{before} ") }, body, if after.is_empty() { String::new() } else { format!(" {after}") });
    let default_label = anchor.add(normal.scale(radius.unwrap_or(1.0) * 2.0 + 2.0)).add(Vec3::new(1.0, 1.0, 0.0).scale(radius.unwrap_or(1.0)));
    Ok(Resolved {
        text,
        value: radius.map(|r| r * 2.0),
        unit: "mm",
        references: targets,
        geometry: PmiGeometry::Hole {
            anchor: a3(anchor),
            normal: a3(normal),
        },
        default_label: a3(default_label),
    })
}
