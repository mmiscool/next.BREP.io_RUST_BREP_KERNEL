//! Datum (`DTM`): a datum feature letter on a face or edge. Letters are
//! unique per part (the kernel registry); a duplicate is an error on the
//! later annotation.

use super::{id_field, params, reference_field, schema_entry, string_field, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, direction_of, resolve_reference};
use crate::feature_pipeline::pmi::{PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use crate::feature_pipeline::SelectionProbe;
use crate::Vec3;

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "datum",
    short_name: "DTM",
    icon: "\u{25B3}",
    long_name: "\u{25B3} Datum",
    label: "Datum",
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
            ("target", reference_field("Target", &["FACE", "EDGE"], false, 1, 1, "The datum feature")),
            ("letter", string_field("Letter", "", "The datum identifier (A–Z, skipping I, O, Q); assigned automatically when left empty at creation")),
        ]),
    )
}

/// Validate a datum letter: 1–2 letters A–Z, never I / O / Q.
pub fn valid_letter(letter: &str) -> bool {
    !letter.is_empty()
        && letter.len() <= 2
        && letter
            .chars()
            .all(|c| c.is_ascii_uppercase() && !matches!(c, 'I' | 'O' | 'Q'))
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("target");
    let Some(target) = targets.first() else {
        return Err("select the datum feature (a face or an edge)".into());
    };
    let letter = annotation.text("letter").to_ascii_uppercase();
    if !valid_letter(&letter) {
        return Err(format!("'{letter}' is not a datum letter (A–Z, skipping I, O and Q)"));
    }
    if let Some(owner) = context.datums.get(&letter) {
        if owner != annotation.id() {
            return Err(format!("datum letter {letter} is already used by {owner}"));
        }
    }
    let geometry = resolve_reference(context.scene, target)?;
    let anchor = geometry.representative_point();
    let normal = direction_of(&geometry).unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    let default_label = anchor.add(normal.scale(4.0)).add(Vec3::new(2.0, 2.0, 0.0));
    Ok(Resolved {
        text: letter.clone(),
        value: None,
        unit: "",
        references: targets,
        geometry: PmiGeometry::Datum {
            anchor: a3(anchor),
            normal: a3(normal),
            letter,
        },
        default_label: a3(default_label),
    })
}
