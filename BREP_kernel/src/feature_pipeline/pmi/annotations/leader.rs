//! Leader (`LEAD`): free text pointing at one or more elements.

use super::{id_field, options_field, params, reference_field, schema_entry, string_field, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, resolve_reference};
use crate::feature_pipeline::pmi::{PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use crate::feature_pipeline::SelectionProbe;
use crate::Vec3;

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "leader",
    short_name: "LEAD",
    icon: "\u{2197}",
    long_name: "\u{2197} Leader",
    label: "Leader",
    applicable,
    schema,
    resolve,
};

/// One or more faces / edges / vertices.
fn applicable(probe: &SelectionProbe) -> bool {
    probe.faces + probe.edges + probe.vertices >= 1 && probe.solids == 0 && probe.sketches == 0
}

fn schema() -> serde_json::Value {
    schema_entry(
        &DEF,
        params(vec![
            ("id", id_field()),
            (
                "targets",
                reference_field("Targets", &["VERTEX", "EDGE", "FACE"], true, 1, 8, "The elements the leader points at"),
            ),
            ("text", string_field("Text", "", "The leader text")),
            (
                "endStyle",
                options_field("End", &["arrow", "dot"], "arrow", "Arrowhead or dot at each target"),
            ),
        ]),
    )
}

/// A label offset scale from the targets' spread (or a unit step).
pub(super) fn offset_scale(points: &[Vec3]) -> f64 {
    let mut best = 0.0f64;
    for a in points {
        for b in points {
            best = best.max(a.sub(*b).length());
        }
    }
    (best * 0.5).max(2.0)
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("targets");
    if targets.is_empty() {
        return Err("select at least one element to point at".into());
    }
    let mut points = Vec::with_capacity(targets.len());
    for target in &targets {
        points.push(resolve_reference(context.scene, target)?.representative_point());
    }
    let text = annotation.text("text").to_string();
    let first = points[0];
    let default_label = first.add(Vec3::new(1.0, 1.0, 0.0).scale(offset_scale(&points) * 0.7));
    Ok(Resolved {
        text,
        value: None,
        unit: "",
        references: targets,
        geometry: PmiGeometry::Leader {
            targets: points.iter().map(|p| a3(*p)).collect(),
            dot: annotation.text("endStyle").eq_ignore_ascii_case("dot"),
        },
        default_label: a3(default_label),
    })
}
