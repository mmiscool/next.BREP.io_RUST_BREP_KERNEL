//! Radial dimension (`RAD`): the radius / diameter of a cylindrical or
//! spherical face or a circular edge, read from the face metadata.

use super::{decimals_of, dimension_fields, id_field, options_field, params, reference_field, schema_entry, PmiTypeDef};
use crate::feature_pipeline::pmi::resolve::{a3, class_of, perpendicular_in_plane, resolve_reference};
use crate::feature_pipeline::pmi::{format_dimension, PmiAnnotation, PmiContext, PmiGeometry, Resolved, ToleranceBlock};
use crate::feature_pipeline::SelectionProbe;
use crate::{SelectionGeometry, Vec3};

pub const DEF: PmiTypeDef = PmiTypeDef {
    type_id: "radial",
    short_name: "RAD",
    icon: "\u{2300}",
    long_name: "\u{2300} Radial dimension",
    label: "Radial dimension",
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
    let mut fields = params(vec![
        ("id", id_field()),
        (
            "target",
            reference_field(
                "Target",
                &["FACE", "EDGE"],
                false,
                1,
                1,
                "A cylindrical or spherical face, or a circular edge",
            ),
        ),
        (
            "displayStyle",
            options_field("Style", &["diameter", "radius"], "diameter", "Show ⌀ (diameter) or R (radius)"),
        ),
    ]);
    dimension_fields(&mut fields, 3);
    schema_entry(&DEF, fields)
}

fn resolve(annotation: &PmiAnnotation, context: &PmiContext<'_>) -> Result<Resolved, String> {
    let targets = annotation.references("target");
    let Some(target) = targets.first() else {
        return Err("select a cylindrical face, a spherical face or a circular edge".into());
    };
    let geometry = resolve_reference(context.scene, target)?;
    let (center, axis, radius, sphere) = match geometry {
        SelectionGeometry::Axis {
            origin,
            direction,
            radius: Some(radius),
        } => (origin, direction, radius, false),
        SelectionGeometry::Axis { radius: None, .. } => {
            return Err(format!("'{target}' has no constant radius (a cone or a general revolution) — pick a circular edge on it"))
        }
        SelectionGeometry::Circle { center, axis, radius } => (center, axis, radius, false),
        SelectionGeometry::Sphere { center, radius } => (center, Vec3::new(0.0, 0.0, 1.0), radius, true),
        other => {
            return Err(format!(
                "'{target}' is a {} — a radial dimension needs a cylinder, a sphere or a circular edge",
                class_of(&other)
            ))
        }
    };
    let diameter = !annotation.text("displayStyle").eq_ignore_ascii_case("radius");
    let value = if diameter { radius * 2.0 } else { radius };
    let decimals = decimals_of(annotation, context, 3.0);
    let tolerance = ToleranceBlock::read(annotation, context.env)?;
    let prefix = if diameter { "\u{2300}" } else { "R" };
    let text = format_dimension(value, decimals, &tolerance, annotation.flag("isReference"), prefix, "");
    let out = perpendicular_in_plane(axis, Vec3::new(1.0, 1.0, 0.0));
    let default_label = center.add(out.scale(radius * 1.6 + 0.5));
    Ok(Resolved {
        text,
        value: Some(value),
        unit: "mm",
        references: targets,
        geometry: PmiGeometry::Radial {
            center: a3(center),
            axis: a3(axis),
            radius,
            diameter,
            sphere,
        },
        default_label: a3(default_label),
    })
}
