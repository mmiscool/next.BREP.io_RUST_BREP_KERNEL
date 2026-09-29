//! Linear dimension (`DIM`): the distance between two elements, or the
//! length of one straight edge, with an optional X / Y / Z component.

use super::{decimals_of, dimension_fields, id_field, options_field, params, reference_field, schema_entry, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, edge_endpoints, is_straight_edge, linear_between, perpendicular_in_plane, resolve_reference};
use crate::feature_pipeline::pmi::{format_dimension, PmiAnnotation, PmiContext, PmiGeometry, Resolved, ToleranceBlock};
use crate::feature_pipeline::SelectionProbe;
use crate::Vec3;

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "linear",
    short_name: "DIM",
    icon: "\u{2194}",
    long_name: "\u{2194} Linear dimension",
    label: "Linear dimension",
    applicable,
    schema,
    resolve,
};

/// One or two of faces / edges / vertices / planes, nothing else.
fn applicable(probe: &SelectionProbe) -> bool {
    let named = probe.faces + probe.edges + probe.vertices + probe.planes;
    (1..=2).contains(&named) && probe.solids == 0 && probe.sketches == 0
}

fn schema() -> serde_json::Value {
    let mut fields = params(vec![
        ("id", id_field()),
        (
            "targets",
            reference_field(
                "Targets",
                &["VERTEX", "EDGE", "FACE", "PLANE"],
                true,
                1,
                2,
                "Two elements to measure between, or one straight edge for its length",
            ),
        ),
        (
            "alignment",
            options_field(
                "Alignment",
                &["free", "X", "Y", "Z"],
                "free",
                "free = the true distance; X / Y / Z = the distance component along that world axis",
            ),
        ),
    ]);
    dimension_fields(&mut fields, 3);
    schema_entry(&DEF, fields)
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("targets");
    let (a, b) = match targets.as_slice() {
        [] => return Err("select one straight edge or two elements".into()),
        [single] => {
            let geometry = resolve_reference(context.scene, single)?;
            if !is_straight_edge(&geometry) {
                return Err(format!(
                    "'{single}' is not a straight edge — a single target must be one (pick two elements to measure between)"
                ));
            }
            edge_endpoints(context.scene, single)?
        }
        [first, second] => {
            let ga = resolve_reference(context.scene, first)?;
            let gb = resolve_reference(context.scene, second)?;
            linear_between(&ga, &gb)?
        }
        _ => return Err("a linear dimension takes at most two targets".into()),
    };
    let alignment = annotation.text("alignment").to_ascii_uppercase();
    let (b, component) = match alignment.as_str() {
        "X" => (Vec3::new(b.x, a.y, a.z), Some('X')),
        "Y" => (Vec3::new(a.x, b.y, a.z), Some('Y')),
        "Z" => (Vec3::new(a.x, a.y, b.z), Some('Z')),
        _ => (b, None),
    };
    let value = b.sub(a).length();
    let decimals = decimals_of(annotation, context, 3.0);
    let tolerance = ToleranceBlock::read(annotation, context.env)?;
    let text = format_dimension(value, decimals, &tolerance, annotation.flag("isReference"), "", "");
    // Default label: beside the measured span, offset a fraction of its length.
    let span = b.sub(a);
    let offset_dir = if span.length() > 1e-9 {
        perpendicular_in_plane(span, Vec3::new(0.0, 1.0, 0.0))
    } else {
        Vec3::new(0.0, 1.0, 0.0)
    };
    let midpoint = a.add(b).scale(0.5);
    let default_label = midpoint.add(offset_dir.scale(0.25 * value.max(1.0)));
    Ok(Resolved {
        text,
        value: Some(value),
        unit: "mm",
        references: targets,
        geometry: PmiGeometry::Linear {
            a: a3(a),
            b: a3(b),
            component,
        },
        default_label: a3(default_label),
    })
}
