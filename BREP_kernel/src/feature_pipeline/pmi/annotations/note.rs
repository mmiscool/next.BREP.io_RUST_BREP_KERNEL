//! Note (`NOTE`): free text at a world point (the label IS the note: dragging
//! it moves the note). An optional anchor seeds the default position.

use super::{id_field, params, reference_field, schema_entry, string_field, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, resolve_reference};
use crate::feature_pipeline::pmi::{PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use crate::feature_pipeline::SelectionProbe;

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "note",
    short_name: "NOTE",
    icon: "\u{1F5CE}",
    long_name: "\u{1F5CE} Note",
    label: "Note",
    applicable,
    schema,
    resolve,
};

/// Any selection seats a note (its representative point is the seed).
fn applicable(probe: &SelectionProbe) -> bool {
    probe.faces + probe.edges + probe.vertices + probe.planes + probe.solids >= 1
}

fn schema() -> serde_json::Value {
    schema_entry(
        &DEF,
        params(vec![
            ("id", id_field()),
            ("text", string_field("Text", "NOTE", "The note text")),
            (
                "anchor",
                reference_field("Anchor", &["VERTEX", "EDGE", "FACE"], false, 0, 1, "Optional: the element whose point seeds the note position"),
            ),
        ]),
    )
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let anchors = annotation.references("anchor");
    let seed = match anchors.first() {
        Some(anchor) => resolve_reference(context.scene, anchor)?.representative_point(),
        None => crate::Vec3::new(0.0, 0.0, 0.0),
    };
    let position = annotation.label_world.unwrap_or(a3(seed));
    Ok(Resolved {
        text: annotation.text("text").to_string(),
        value: None,
        unit: "",
        references: anchors,
        geometry: PmiGeometry::Note { position },
        default_label: a3(seed),
    })
}
