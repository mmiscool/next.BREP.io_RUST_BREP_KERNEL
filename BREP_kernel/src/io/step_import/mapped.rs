//! MAPPED_ITEM / REPRESENTATION_MAP: placing a SHARED representation.
//!
//! A `MAPPED_ITEM` is the other way a STEP file places geometry it wants to use
//! more than once. Where a `NEXT_ASSEMBLY_USAGE_OCCURRENCE` relates two PRODUCT
//! DEFINITIONS and hangs the placement off a
//! `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`, a mapped item sits directly in a
//! representation's item list and names a `REPRESENTATION_MAP`:
//!
//! ```text
//!   SHAPE_REPRESENTATION( ..., #item, #target, ... )   <- the USING rep
//!     #item   = MAPPED_ITEM(name, #map, #target)
//!     #map    = REPRESENTATION_MAP(#origin, #mapped)
//!     #origin = an item of #mapped   (the frame the mapped rep is authored in)
//!     #target = an item of the using rep (where that frame lands)
//! ```
//!
//! Per ISO 10303-43 the item instantiates `#mapped` under
//!
//! ```text
//!   T = M(mapping_target) . M(mapping_origin)^-1
//! ```
//!
//! so the origin's frame is undone first and the target's applied second. The
//! origin is an item of the MAPPED representation and the target an item of the
//! USING one, which is why each is read at ITS OWN representation's unit scale
//! ([`mapped_item_transform`] takes two): a mapping from an inch part into a
//! millimetre assembly is a length in inches composed with a length in
//! millimetres, and one file-global scale gets it wrong in both directions.
//!
//! # What is NOT expanded, and why it matters
//!
//! Mapped items are found by walking a structure node's own SHAPE
//! representations ([`bodies::node_representations`]), never by sweeping the
//! file for `MAPPED_ITEM` records. That is load-bearing rather than tidy: AP242
//! saved views place the whole shape into a `DRAUGHTING_MODEL` through a mapped
//! item with an identity transform — our own PMI writer does it
//! (`io/step/pmi.rs`), and the `io1-ac-214.stp` fixture carries exactly that
//! shape at `#1026` — and expanding one would instantiate the entire part a
//! second time as a phantom occurrence of an annotation view. A
//! `DRAUGHTING_MODEL` is not one of the representations
//! [`bodies::representation_args`] recognises, so the walk never reaches it.

use super::*;

/// A mapping the importer cannot instantiate at all, named by the entity that
/// carries it.
///
/// A MIRRORED or SCALED mapping is NOT one of these. `MAPPED_ITEM` is the one
/// placement the importer reads whose transform is not built from
/// `AXIS2_PLACEMENT_3D`s — a `CARTESIAN_TRANSFORMATION_OPERATOR_3D` may carry a
/// left-handed axis triad (a reflection) or a `scale` — and such an occurrence is
/// carried with [`StepOccurrence::rigid`] `false`, exactly as a non-rigid NAUO
/// occurrence would be, for the caller to split into a rigid pose and a baked
/// factor. `BREP_render`'s assembly intake does precisely that (`model_io.rs`
/// `split_rigid` / `build_library_entry`): the mirror becomes a distinct part
/// whose geometry is mirrored through `transform_brep`'s orientation reversal,
/// counted in the import notice's "N mirrored instances baked". The two edge
/// kinds of one graph therefore agree, and the user's file imports.
///
/// What is left is the mapping no consumer could place: a SINGULAR composed
/// transform, which has no inverse, no rigid part and no factor. Its edge is
/// dropped — so nothing is instantiated for it, and nothing under it — and the
/// reason is reported through the import's `first_error`, which the app appends
/// to its import notice (`BREP_app/src/panels/file.rs`), so a missing instance is
/// stated rather than silent. It takes a malformed file to reach:
/// `cartesian_transformation_operator`'s own WR1 requires `scale > 0`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StepMappedItemError {
    /// The composed transform collapses space: `|det| <= 1e-14`.
    Degenerate {
        /// The MAPPED_ITEM entity id.
        item: usize,
        /// The REPRESENTATION_MAP entity id.
        map: usize,
        determinant: f64,
    },
}

impl std::fmt::Display for StepMappedItemError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Degenerate {
                item,
                map,
                determinant,
            } => write!(
                formatter,
                "step_import: MAPPED_ITEM #{item} (REPRESENTATION_MAP #{map}) has a singular \
                 mapping transform (determinant {determinant}): it collapses the mapped \
                 representation, so nothing was instantiated for it"
            ),
        }
    }
}

/// One `MAPPED_ITEM` of a representation, with its map already resolved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct MappedItem {
    /// The MAPPED_ITEM entity id — the mapped edge's identity, the way a NAUO
    /// id identifies an occurrence edge.
    pub(crate) item: usize,
    /// The REPRESENTATION_MAP entity id.
    pub(crate) map: usize,
    /// `mapping_origin`: an item of [`Self::mapped_rep`].
    pub(crate) origin: usize,
    /// `mapping_target`: an item of the representation holding the mapped item.
    pub(crate) target: usize,
    /// `mapped_representation` — the representation this item instantiates.
    pub(crate) mapped_rep: usize,
}

/// The instance label of a mapped item: its own `name`, else `mapped#<id>`.
///
/// The sibling of `bodies::nauo_designator`, and read for the same reason — the
/// app turns a designator into a component id, so an instance keeps whatever
/// identity the file gave it.
pub(super) fn mapped_item_designator(resolver: &Resolver, item: usize) -> String {
    (|| {
        match resolver.get(item).ok()?.find("MAPPED_ITEM")?.first()? {
            Value::Str(text) if !text.trim().is_empty() => Some(text.trim().to_string()),
            _ => None,
        }
    })()
    .unwrap_or_else(|| format!("mapped#{item}"))
}

/// Every `MAPPED_ITEM` in one representation's item list, ascending by entity
/// id, with each item's `REPRESENTATION_MAP` resolved.
///
/// An item whose map, origin, target or mapped representation cannot be read is
/// SKIPPED rather than reported: it places nothing, the same graceful answer
/// `edge_transform` gives a partial NAUO transform chain.
pub(super) fn representation_mapped_items(resolver: &Resolver, rep: usize) -> Vec<MappedItem> {
    let mut items = Vec::new();
    let Ok(entity) = resolver.get(rep) else {
        return items;
    };
    let Some(list) = representation_items(entity) else {
        return items;
    };
    for value in list {
        let Ok(item_ref) = value.as_ref_id() else {
            continue;
        };
        if let Some(item) = mapped_item(resolver, item_ref) {
            items.push(item);
        }
    }
    items.sort_unstable_by_key(|item| item.item);
    items.dedup();
    items
}

/// Read one entity as a `MAPPED_ITEM(name, mapping_source, mapping_target)`
/// whose `mapping_source` is a `REPRESENTATION_MAP(mapping_origin,
/// mapped_representation)`. `None` for anything else, including a styled
/// wrapper around a mapped item — a `STYLED_ITEM` names the item it styles and
/// is not itself an occurrence.
fn mapped_item(resolver: &Resolver, item_ref: usize) -> Option<MappedItem> {
    let args = resolver.get(item_ref).ok()?.find("MAPPED_ITEM")?;
    let map = args.get(1)?.as_ref_id().ok()?;
    let target = args.get(2)?.as_ref_id().ok()?;
    let map_args = resolver.get(map).ok()?.find("REPRESENTATION_MAP")?;
    let origin = map_args.first()?.as_ref_id().ok()?;
    let mapped_rep = map_args.get(1)?.as_ref_id().ok()?;
    Some(MappedItem {
        item: item_ref,
        map,
        origin,
        target,
        mapped_rep,
    })
}

/// The mapped representation's frame -> the using representation's frame:
/// `M(mapping_target) . M(mapping_origin)^-1`.
///
/// `origin_scale` is millimetres per unit of the MAPPED representation's
/// context and `target_scale` the same for the USING one, because that is the
/// representation each of the two frames is an item of. They are equal for a
/// single-context file, where this reduces to one scale.
///
/// A MIRRORED or SCALED mapping is returned like any other: it is a placement,
/// just not a rigid one, and the occurrence carries `rigid: false` for the caller
/// to bake (see [`StepMappedItemError`], which is now only the singular case).
/// A frame that cannot be read at all is taken as IDENTITY instead (a mapped item
/// with an unreadable target still instantiates its part, in the part's own
/// frame), which is the same graceful fallback `nauo_edge_transform` applies to an
/// incomplete NAUO transform chain.
pub(super) fn mapped_item_transform(
    entities: &HashMap<usize, Entity>,
    item: &MappedItem,
    origin_scale: f64,
    target_scale: f64,
) -> Result<Mat4, StepMappedItemError> {
    let origin = Resolver::new(entities, origin_scale);
    let target = Resolver::new(entities, target_scale);
    let origin_matrix = placement_matrix(&origin, item.origin).unwrap_or_else(mat4_identity);
    let target_matrix = placement_matrix(&target, item.target).unwrap_or_else(mat4_identity);
    let composed = mat4_affine_inverse(&origin_matrix)
        .map(|inverse| mat4_mul(&target_matrix, &inverse))
        .unwrap_or_else(|_| target_matrix);
    let determinant = mat4_determinant3(&composed);
    if determinant.abs() <= 1e-14 || !composed.iter().all(|value| value.is_finite()) {
        return Err(StepMappedItemError::Degenerate {
            item: item.item,
            map: item.map,
            determinant,
        });
    }
    Ok(composed)
}

/// The local->world matrix of a `representation_item` used as a placement: an
/// `AXIS2_PLACEMENT_3D`, or a `CARTESIAN_TRANSFORMATION_OPERATOR_3D`.
///
/// The operator form is read ONLY here, on the mapped-item path. A NAUO
/// placement still comes from a pair of `AXIS2_PLACEMENT_3D`s
/// (`bodies::resolve_rep_rel_transform`); reading an operator THERE is separate,
/// unimplemented work with its own fixtures to author.
fn placement_matrix(resolver: &Resolver, id: usize) -> Option<Mat4> {
    if let Ok(frame) = resolver.placement(id) {
        return Some(frame_matrix(&frame));
    }
    transformation_operator_matrix(resolver, id)
}

/// A `CARTESIAN_TRANSFORMATION_OPERATOR_3D` as an affine matrix: the derived
/// axis triad `u` scaled by `scl`, translated to `local_origin`.
///
/// # Reading the record without counting its labels
///
/// `cartesian_transformation_operator` is a SUBTYPE OF
/// (`geometric_representation_item`, `functionally_defined_transformation`), so
/// its Part 21 record begins with `representation_item.name` AND the
/// transformation's own `name` and `description` before
/// `axis1, axis2, local_origin, scale`, with `axis3` last. Producers disagree
/// about how many of those labels they write, and a miscount would silently
/// shift every axis by one argument. So this anchors on `local_origin` — the
/// one MANDATORY attribute, and the only `CARTESIAN_POINT` among them — and
/// reads `axis1`/`axis2` immediately before it and `scale`/`axis3` immediately
/// after. Absent optional axes (`$`) are read as such because the anchor makes
/// every position relative to it, not to the start of the record.
///
/// The triad follows ISO 10303-42's `base_axis(3, axis1, axis2, axis3)`:
/// `u3` from `axis3`, `u1` = `axis1` projected orthogonal to `u3`, and `u2` =
/// `axis2` orthogonalised against both — `second_proj_axis` keeps the SIGN of
/// the given `axis2`, which is what lets a conforming file express a
/// left-handed triad (a reflection) at all. Only when `axis2` is absent is `u2`
/// derived as `u3 x u1`, which forces right-handedness.
fn transformation_operator_matrix(resolver: &Resolver, id: usize) -> Option<Mat4> {
    let entity = resolver.get(id).ok()?;
    let args = entity
        .find("CARTESIAN_TRANSFORMATION_OPERATOR_3D")
        .or_else(|| entity.find("CARTESIAN_TRANSFORMATION_OPERATOR"))?;
    let is_point = |value: &Value| {
        value
            .as_ref_id()
            .ok()
            .and_then(|point_ref| resolver.get(point_ref).ok())
            .is_some_and(|point| point.has("CARTESIAN_POINT"))
    };
    let anchor = args.iter().position(is_point)?;
    let local_origin = resolver.point(args[anchor].as_ref_id().ok()?).ok()?;
    let direction = |offset: usize, before: bool| -> Option<Vec3> {
        let index = if before {
            anchor.checked_sub(offset)?
        } else {
            anchor + offset
        };
        match args.get(index) {
            Some(Value::Ref(reference)) => resolver.direction(*reference).ok(),
            _ => None,
        }
    };
    let axis1 = direction(2, true);
    let axis2 = direction(1, true);
    let axis3 = direction(2, false);
    let scale = match args.get(anchor + 1) {
        Some(value) => value.as_real().unwrap_or(1.0),
        None => 1.0,
    };
    let u3 = axis3.unwrap_or_else(|| Vec3::new(0.0, 0.0, 1.0));
    let u1 = axis1
        .and_then(|axis| axis.sub(u3.scale(axis.dot(u3))).normalized().ok())
        .or_else(|| u3.perpendicular().ok())?;
    let u2 = match axis2 {
        Some(axis) => axis
            .sub(u3.scale(axis.dot(u3)))
            .sub(u1.scale(axis.dot(u1)))
            .normalized()
            .ok()?,
        None => u3.cross(u1).normalized().ok()?,
    };
    let (u1, u2, u3) = (u1.scale(scale), u2.scale(scale), u3.scale(scale));
    Some([
        u1.x,
        u2.x,
        u3.x,
        local_origin.x,
        u1.y,
        u2.y,
        u3.y,
        local_origin.y,
        u1.z,
        u2.z,
        u3.z,
        local_origin.z,
        0.0,
        0.0,
        0.0,
        1.0,
    ])
}
