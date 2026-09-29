//! Feature control frame (`FCF`): a geometric tolerance on a face or edge —
//! characteristic, tolerance zone (⌀, material condition) and up to three
//! datum references, validated against the part's datum registry.
//!
//! All fourteen ASME Y14.5-2009 characteristics are offered: AP242 defines an
//! entity for each (concentricity and symmetry included, which Y14.5-2018
//! retired) and the export target is the schema, not one drawing standard.

use super::{boolean_field, id_field, number_field, options_field, params, reference_field, schema_entry, string_field, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, direction_of, resolve_reference};
use crate::feature_pipeline::pmi::{format_number, FcfFrame, PmiAnnotation, PmiContext, PmiGeometry, Resolved};
use crate::feature_pipeline::SelectionProbe;
use crate::Vec3;

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "fcf",
    short_name: "FCF",
    icon: "\u{2316}",
    long_name: "\u{2316} Feature control frame",
    label: "Feature control frame",
    applicable,
    schema,
    resolve,
};

/// How many datum references a characteristic takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DatumRule {
    /// A form tolerance: none allowed.
    None,
    /// Position / profile: none or up to three.
    Optional,
    /// Orientation / location / runout: one to three.
    Required,
}

/// One geometric characteristic: id, symbol, label, its AP242 entity and
/// its datum rule.
pub struct Characteristic {
    pub id: &'static str,
    pub symbol: &'static str,
    pub label: &'static str,
    pub step_entity: &'static str,
    pub datums: DatumRule,
}

pub const CHARACTERISTICS: [Characteristic; 14] = [
    Characteristic { id: "straightness", symbol: "\u{23E4}", label: "Straightness", step_entity: "STRAIGHTNESS_TOLERANCE", datums: DatumRule::None },
    Characteristic { id: "flatness", symbol: "\u{23E5}", label: "Flatness", step_entity: "FLATNESS_TOLERANCE", datums: DatumRule::None },
    Characteristic { id: "circularity", symbol: "\u{25CB}", label: "Circularity", step_entity: "ROUNDNESS_TOLERANCE", datums: DatumRule::None },
    Characteristic { id: "cylindricity", symbol: "\u{232D}", label: "Cylindricity", step_entity: "CYLINDRICITY_TOLERANCE", datums: DatumRule::None },
    Characteristic { id: "profileLine", symbol: "\u{2312}", label: "Profile of a line", step_entity: "LINE_PROFILE_TOLERANCE", datums: DatumRule::Optional },
    Characteristic { id: "profileSurface", symbol: "\u{2313}", label: "Profile of a surface", step_entity: "SURFACE_PROFILE_TOLERANCE", datums: DatumRule::Optional },
    Characteristic { id: "angularity", symbol: "\u{2220}", label: "Angularity", step_entity: "ANGULARITY_TOLERANCE", datums: DatumRule::Required },
    Characteristic { id: "perpendicularity", symbol: "\u{27C2}", label: "Perpendicularity", step_entity: "PERPENDICULARITY_TOLERANCE", datums: DatumRule::Required },
    Characteristic { id: "parallelism", symbol: "\u{2225}", label: "Parallelism", step_entity: "PARALLELISM_TOLERANCE", datums: DatumRule::Required },
    Characteristic { id: "position", symbol: "\u{2316}", label: "Position", step_entity: "POSITION_TOLERANCE", datums: DatumRule::Optional },
    Characteristic { id: "concentricity", symbol: "\u{25CE}", label: "Concentricity", step_entity: "CONCENTRICITY_TOLERANCE", datums: DatumRule::Required },
    Characteristic { id: "symmetry", symbol: "\u{232F}", label: "Symmetry", step_entity: "SYMMETRY_TOLERANCE", datums: DatumRule::Required },
    Characteristic { id: "circularRunout", symbol: "\u{2197}", label: "Circular runout", step_entity: "CIRCULAR_RUNOUT_TOLERANCE", datums: DatumRule::Required },
    Characteristic { id: "totalRunout", symbol: "\u{2330}", label: "Total runout", step_entity: "TOTAL_RUNOUT_TOLERANCE", datums: DatumRule::Required },
];

pub fn characteristic(id: &str) -> Option<&'static Characteristic> {
    CHARACTERISTICS.iter().find(|c| c.id == id)
}

/// The characteristic behind an AP242 entity keyword.
pub fn characteristic_for_entity(keyword: &str) -> Option<&'static Characteristic> {
    CHARACTERISTICS.iter().find(|c| c.step_entity == keyword)
}

/// The material-condition modifier glyph (`Ⓜ` / `Ⓛ`), empty for none.
pub fn modifier_symbol(modifier: &str) -> &'static str {
    match modifier.trim().to_ascii_uppercase().as_str() {
        "MMC" => "\u{24C2}",
        "LMC" => "\u{24C1}",
        _ => "",
    }
}

/// Exactly one face or edge.
fn applicable(probe: &SelectionProbe) -> bool {
    probe.faces + probe.edges == 1
        && probe.vertices == 0
        && probe.planes == 0
        && probe.solids == 0
        && probe.sketches == 0
}

fn schema() -> serde_json::Value {
    let ids: Vec<&str> = CHARACTERISTICS.iter().map(|c| c.id).collect();
    let modifiers = ["none", "MMC", "LMC"];
    schema_entry(
        &DEF,
        params(vec![
            ("id", id_field()),
            ("target", reference_field("Target", &["FACE", "EDGE"], false, 1, 1, "The toleranced feature")),
            ("characteristic", options_field("Characteristic", &ids, "flatness", "The geometric characteristic")),
            ("zoneValue", number_field("Tolerance", 0.1, 0.01, "The tolerance zone size")),
            ("zoneDiameter", boolean_field("⌀ zone", false, "A cylindrical (⌀) tolerance zone")),
            ("materialCondition", options_field("Material condition", &modifiers, "none", "Ⓜ MMC / Ⓛ LMC on the tolerance")),
            ("datumA", string_field("Datum 1", "", "The primary datum letter")),
            ("datumAModifier", options_field("Datum 1 modifier", &modifiers, "none", "")),
            ("datumB", string_field("Datum 2", "", "The secondary datum letter")),
            ("datumBModifier", options_field("Datum 2 modifier", &modifiers, "none", "")),
            ("datumC", string_field("Datum 3", "", "The tertiary datum letter")),
            ("datumCModifier", options_field("Datum 3 modifier", &modifiers, "none", "")),
        ]),
    )
}

/// The datum reference cells `(letter, modifier)` an annotation declares, in
/// order, empty letters skipped.
pub fn datum_references(annotation: &PmiAnnotation) -> Vec<(String, String)> {
    ["A", "B", "C"]
        .iter()
        .filter_map(|slot| {
            let letter = annotation.text(&format!("datum{slot}")).to_ascii_uppercase();
            if letter.is_empty() {
                None
            } else {
                Some((letter, annotation.text(&format!("datum{slot}Modifier")).to_ascii_uppercase()))
            }
        })
        .collect()
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("target");
    let Some(target) = targets.first() else {
        return Err("select the toleranced feature (a face or an edge)".into());
    };
    let id = annotation.text("characteristic");
    let Some(characteristic) = characteristic(id) else {
        return Err(format!("unknown characteristic '{id}'"));
    };
    let zone = annotation.number("zoneValue", context.env, 0.0)?;
    if !(zone.is_finite() && zone > 0.0) {
        return Err("the tolerance zone must be positive".into());
    }
    let datums = datum_references(annotation);
    match characteristic.datums {
        DatumRule::None if !datums.is_empty() => {
            return Err(format!("{} is a form tolerance and takes no datum reference", characteristic.label))
        }
        DatumRule::Required if datums.is_empty() => {
            return Err(format!("{} needs at least one datum reference", characteristic.label))
        }
        _ => {}
    }
    for (letter, _) in &datums {
        if !super::datum::valid_letter(letter) {
            return Err(format!("'{letter}' is not a datum letter"));
        }
        if !context.datums.contains_key(letter) {
            return Err(format!("datum {letter} is not defined in the part"));
        }
    }
    let geometry = resolve_reference(context.scene, target)?;
    let anchor = geometry.representative_point();
    let normal = direction_of(&geometry).unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    let mut zone_text = String::new();
    if annotation.flag("zoneDiameter") {
        zone_text.push('\u{2300}');
    }
    zone_text.push_str(&format_number(zone, 3).trim_end_matches('0').trim_end_matches('.').to_string());
    let condition = modifier_symbol(annotation.text("materialCondition"));
    if !condition.is_empty() {
        zone_text.push(' ');
        zone_text.push_str(condition);
    }
    let datum_cells: Vec<String> = datums
        .iter()
        .map(|(letter, modifier)| {
            let symbol = modifier_symbol(modifier);
            if symbol.is_empty() {
                letter.clone()
            } else {
                format!("{letter} {symbol}")
            }
        })
        .collect();
    let mut text = format!("{} | {zone_text}", characteristic.symbol);
    for cell in &datum_cells {
        text.push_str(" | ");
        text.push_str(cell);
    }
    let default_label = anchor.add(normal.scale(5.0)).add(Vec3::new(3.0, 2.0, 0.0));
    Ok(Resolved {
        text,
        value: Some(zone),
        unit: "mm",
        references: targets,
        geometry: PmiGeometry::Fcf {
            anchor: a3(anchor),
            normal: a3(normal),
            frame: FcfFrame {
                characteristic: characteristic.id.to_string(),
                symbol: characteristic.symbol.to_string(),
                zone: zone_text,
                datums: datum_cells,
            },
        },
        default_label: a3(default_label),
    })
}
