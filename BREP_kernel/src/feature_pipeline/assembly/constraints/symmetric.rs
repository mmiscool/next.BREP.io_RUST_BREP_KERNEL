//! Symmetric (`SYMM`) — hold two elements as mirror images about a planar
//! face (the "symmetric" mate of other CAD systems).
//!
//! `elements` is THREE refs in pick order: the first element, the second
//! element, then the MIRROR — a planar face, usually on a third (often
//! grounded) component, but it may belong to either of the first two. The
//! first two must sit on two different components and be of one kind:
//!
//! | elements | mirrored as | DOF removed |
//! |---|---|---|
//! | two planar faces | plane onto plane, facing the mirrored way | 3 |
//! | two lines (cylindrical/conical faces, straight edges) | axis onto axis, either sense | 4 |
//! | two points (vertices, circular edges, spheres, components) | point onto point | 3 |
//!
//! The solver reads the mirror in its own component's frame (a three-body
//! mate), so a mirror on a moving component moves the symmetry with it.

use super::super::mapping::{
    local_point, require_axis, require_plane, ConstraintFailure, MappedConstraint, MappedMate,
    ResolvedElement,
};
use super::{elements_field, id_field, schema_entry, ConstraintTypeDef};
use crate::feature_pipeline::SelectionProbe;
use crate::{MateKind, SelectionGeometry, SymmetricElement};

pub(super) const DEF: ConstraintTypeDef = ConstraintTypeDef {
    type_id: "symmetric",
    label: "Symmetric",
    short_name: "SYMM",
    icon: "\u{22C8}",
    long_name: "\u{22C8} Symmetric",
    min_elements: 3,
    max_elements: 3,
    duplicate_family: false,
    applicable,
};

/// Two like elements and a planar mirror face across two or three
/// components (vertex refs come from the modal picker only).
fn applicable(probe: &SelectionProbe) -> bool {
    probe.all_component
        && (2..=3).contains(&probe.components)
        && probe.faces >= 1
        && probe.faces + probe.edges == 3
}

pub(super) fn schema() -> serde_json::Value {
    schema_entry(&DEF, serde_json::json!({
        "id": id_field(),
        "elements": elements_field(&["FACE", "EDGE", "VERTEX"], 3, 3,
            "Two like elements on two components, then the planar face they are symmetric about"),
    }))
}

/// What one side of the pair is mirrored as.
fn element(resolved: &ResolvedElement) -> Result<SymmetricElement, ConstraintFailure> {
    Ok(match resolved.world {
        SelectionGeometry::Plane { .. } => SymmetricElement::Plane { plane: require_plane(resolved)? },
        SelectionGeometry::Line { .. } | SelectionGeometry::Axis { .. } => {
            SymmetricElement::Axis { axis: require_axis(resolved)? }
        }
        SelectionGeometry::Circle { .. }
        | SelectionGeometry::Sphere { .. }
        | SelectionGeometry::Point { .. } => SymmetricElement::Point { point: local_point(resolved) },
    })
}

pub(in crate::feature_pipeline::assembly) fn map(
    resolved: &[ResolvedElement],
) -> Result<MappedConstraint, ConstraintFailure> {
    let [a, b, mirror] = resolved else {
        return Err(ConstraintFailure::new("incomplete", "Select two elements and a mirror face."));
    };
    if a.component == b.component {
        return Err(ConstraintFailure::new(
            "invalid-selection",
            format!(
                "Both symmetric elements belong to component '{}' — pick one on each of two components.",
                a.component
            ),
        ));
    }
    let plane_c = require_plane(mirror).map_err(|_| {
        ConstraintFailure::unsupported(format!(
            "the mirror '{}' (the third pick) must be a planar face",
            mirror.name
        ))
    })?;
    let (element_a, element_b) = (element(a)?, element(b)?);
    if std::mem::discriminant(&element_a) != std::mem::discriminant(&element_b) {
        return Err(ConstraintFailure::unsupported(
            "symmetric elements must be alike: two planar faces, two lines or two points",
        ));
    }
    Ok(MappedConstraint {
        mates: vec![MappedMate {
            body_a: a.component.clone(),
            body_b: b.component.clone(),
            body_c: Some(mirror.component.clone()),
            kind: MateKind::Symmetric { element_a, element_b, plane_c },
        }],
        ..Default::default()
    })
}
