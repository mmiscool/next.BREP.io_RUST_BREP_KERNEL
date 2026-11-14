//! STEP presentation entities — the colour half of ISO 10303-46, read into
//! [`crate::ImportedColor`].
//!
//! # The chain
//!
//! ```text
//! STYLED_ITEM(name, styles, item)                    item = what is coloured
//!   └ PRESENTATION_STYLE_ASSIGNMENT(styles)          (or PRESENTATION_STYLE_BY_CONTEXT)
//!       └ SURFACE_STYLE_USAGE(side, style)           .BOTH. / .POSITIVE. / .NEGATIVE.
//!           └ SURFACE_SIDE_STYLE(name, styles)
//!               ├ SURFACE_STYLE_FILL_AREA(fill_area) the SHADED colour
//!               │   └ FILL_AREA_STYLE(name, styles)
//!               │       └ FILL_AREA_STYLE_COLOUR(name, colour)
//!               │           └ COLOUR_RGB(name, r, g, b) | DRAUGHTING_PRE_DEFINED_COLOUR(name)
//!               └ SURFACE_STYLE_RENDERING[_WITH_PROPERTIES](method, colour, …)
//! ```
//!
//! `item` is either a WHOLE BODY (MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS / the
//! shell) or a SINGLE `ADVANCED_FACE`. Both matter and both are read: per-body
//! colour is the common vendor case, per-face colour is what makes a
//! deliberately multi-coloured model look right.
//!
//! # What is deliberately NOT a colour here
//!
//! `SURFACE_STYLE_BOUNDARY` → `CURVE_STYLE` carries the EDGE colour, not the
//! surface's. Reading it would be actively wrong: `io1-ug-214.stp` styles its
//! solid with a boundary-only style (yellow) while the actual face colour is red
//! on an over-riding item, and `BIMExample-Site.step` pairs every fill style
//! with a black curve style. Only fill-area and rendering colours count.
//!
//! # Precedence and determinism
//!
//! * `OVER_RIDING_STYLED_ITEM(name, styles, item, over_ridden_style)` is the
//!   per-instance override; its own styles WIN over a plain `STYLED_ITEM` on the
//!   same item.
//! * Within one rank, the LOWEST entity id wins, so a file that styles one item
//!   several times (once per representation context) imports the same colour
//!   every run.
//! * An OCCURRENCE's own colour is a
//!   `CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM(name, styles, item,
//!   over_ridden_style, style_context)` whose context is that occurrence: the
//!   `REPRESENTATION_RELATIONSHIP` its `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`
//!   places the part through (or, read leniently, the
//!   `NEXT_ASSEMBLY_USAGE_OCCURRENCE` itself). It colours that item of that
//!   instance only, never the part, so it is kept apart, keyed by occurrence
//!   and item ([`StyleTable::occurrence_item_color`]). A
//!   context naming a path of several occurrences (an instance deep inside a
//!   sub-assembly) is not read; such styles are counted. A plain style on a
//!   `MAPPED_ITEM` is likewise the colour of the occurrence that item places.
//! * `MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION` is the container
//!   that gathers styled items. It needs no special handling — every styled item
//!   in the file is scanned directly, which also picks up the styled items of
//!   files that use a plain `PRESENTATION_REPRESENTATION` or hang them off
//!   `DRAUGHTING_MODEL`. The container is thus read THROUGH, not read.
//! * `INVISIBILITY` (a set of styled items marking hidden geometry) is NOT
//!   honoured — an invisible item still imports as a body, and hiding it is a
//!   display concern the kernel has no channel for yet.

use super::*;
use crate::appearance::ImportedColor;

/// Every colour the file's presentation entities assign, keyed by the styled
/// ITEM's entity id (a solid, a shell, or a face).
///
/// Built ONCE per parse and then only read, so the per-body appearance lookups
/// are hash hits rather than a re-walk of the style graph per face.
pub(super) struct StyleTable {
    /// item entity id → its resolved surface colour.
    by_item: HashMap<usize, ImportedColor>,
    /// (NEXT_ASSEMBLY_USAGE_OCCURRENCE id, styled item id) → the colour that
    /// item takes in that one occurrence.
    by_occurrence: HashMap<(usize, usize), ImportedColor>,
    /// Occurrence styles whose context is a path of several occurrences, which
    /// this reader does not attach.
    pub(super) nested_occurrence_styles: usize,
    /// Occurrence id (a NEXT_ASSEMBLY_USAGE_OCCURRENCE or a MAPPED_ITEM) → the
    /// representations its placement relates (either side of its relationship,
    /// which files write both ways round, or a mapped item's mapped
    /// representation): a style on the placed one in the occurrence's context
    /// colours the whole instance.
    placed_representation: HashMap<usize, Vec<usize>>,
}

impl StyleTable {
    /// Read every `STYLED_ITEM` / `OVER_RIDING_STYLED_ITEM` in the file.
    ///
    /// Nothing here can fail the import: an entity whose style chain is absent,
    /// malformed, or of a shape this reader does not know simply contributes no
    /// colour. A file with no presentation entities produces an empty table at
    /// the cost of one pass over the entity map.
    pub(super) fn read(entities: &HashMap<usize, Entity>) -> Self {
        // (item, colour, over_riding) triples first, so precedence is applied
        // once over a sorted list rather than by fighting HashMap order.
        let mut found: Vec<(usize, usize, ImportedColor, bool)> = Vec::new();
        let mut occurrences: Vec<(usize, usize, usize, ImportedColor)> = Vec::new();
        let mut nested_occurrence_styles = 0;
        let mut placed_through: Option<(HashMap<usize, usize>, HashMap<usize, Vec<usize>>)> = None;
        for (id, entity) in entities.iter() {
            if let Some(args) = entity.find("CONTEXT_DEPENDENT_OVER_RIDING_STYLED_ITEM") {
                let Some(color) = args.get(1).and_then(|styles| style_list_color(entities, styles)) else {
                    continue;
                };
                let Some(Ok(item_ref)) = args.get(2).map(Value::as_ref_id) else {
                    continue;
                };
                let Some(context) = args.get(4).and_then(|context| context.as_list().ok()) else {
                    continue;
                };
                if context.len() != 1 {
                    nested_occurrence_styles += 1;
                    continue;
                }
                let Ok(context_ref) = context[0].as_ref_id() else {
                    continue;
                };
                let (placed_through, _) = placed_through.get_or_insert_with(|| relationship_occurrences(entities));
                // The occurrence the context names: the NAUO itself, the
                // MAPPED_ITEM that is the occurrence, or the relationship or
                // CONTEXT_DEPENDENT_SHAPE_REPRESENTATION that places it.
                let context = entities.get(&context_ref);
                let occurrence = if context
                    .is_some_and(|e| e.has("NEXT_ASSEMBLY_USAGE_OCCURRENCE") || e.has("MAPPED_ITEM"))
                {
                    Some(context_ref)
                } else {
                    placed_through.get(&context_ref).copied()
                };
                if let Some(occurrence) = occurrence {
                    occurrences.push((occurrence, item_ref, *id, color));
                }
                continue;
            }
            let over_riding = entity.has("OVER_RIDING_STYLED_ITEM");
            let Some(args) = entity
                .find("OVER_RIDING_STYLED_ITEM")
                .or_else(|| entity.find("STYLED_ITEM"))
            else {
                continue;
            };
            // STYLED_ITEM(name, styles, item); OVER_RIDING_STYLED_ITEM adds a
            // fourth `over_ridden_style` argument after the identical three.
            let (Some(styles), Some(item)) = (args.get(1), args.get(2)) else {
                continue;
            };
            let Ok(item_ref) = item.as_ref_id() else {
                continue;
            };
            let Some(color) = style_list_color(entities, styles) else {
                continue;
            };
            found.push((item_ref, *id, color, over_riding));
        }
        // Over-riding items win; within a rank the lowest entity id wins.
        found.sort_unstable_by_key(|(item, id, _, over_riding)| {
            (*item, !*over_riding, *id)
        });
        let mut by_item: HashMap<usize, ImportedColor> = HashMap::default();
        for (item_ref, _, color, _) in found {
            by_item.entry(item_ref).or_insert(color);
        }
        // The lowest entity id wins, as for items.
        occurrences.sort_unstable_by_key(|(occurrence, item, id, _)| (*occurrence, *item, *id));
        let mut by_occurrence: HashMap<(usize, usize), ImportedColor> = HashMap::default();
        for (occurrence, item, _, color) in occurrences {
            by_occurrence.entry((occurrence, item)).or_insert(color);
        }
        let placed_representation = if by_occurrence.is_empty() {
            HashMap::default()
        } else {
            placed_through.map(|(_, representations)| representations).unwrap_or_default()
        };
        Self { by_item, by_occurrence, nested_occurrence_styles, placed_representation }
    }

    /// The colour one body takes in one occurrence (a
    /// NEXT_ASSEMBLY_USAGE_OCCURRENCE, or the MAPPED_ITEM that places a shared
    /// representation), where the file styles it there on its own: an
    /// override on any of the body's `items` in that occurrence's context, else
    /// a style on the mapped item, which colours everything it places.
    pub(super) fn occurrence_item_color(&self, occurrence_ref: usize, items: &[usize]) -> Option<ImportedColor> {
        items
            .iter()
            .find_map(|item| self.by_occurrence.get(&(occurrence_ref, *item)))
            .copied()
            .or_else(|| self.whole_occurrence_color(occurrence_ref))
    }

    /// A colour the occurrence gives everything it places: a style on its
    /// MAPPED_ITEM, or an override on the representation it places.
    fn whole_occurrence_color(&self, occurrence_ref: usize) -> Option<ImportedColor> {
        self.by_item.get(&occurrence_ref).copied().or_else(|| {
            let representations = self.placed_representation.get(&occurrence_ref)?;
            representations
                .iter()
                .find_map(|representation| self.by_occurrence.get(&(occurrence_ref, *representation)))
                .copied()
        })
    }

    /// The one colour an occurrence gives everything it places: a style on
    /// its MAPPED_ITEM, whatever that places; else, for an occurrence of a
    /// LEAF part (`leaf_bodies`, each body as its styled items), the colour its
    /// overrides give every one of those bodies alike. `None` when it leaves
    /// any uncoloured, colours them differently, or places a sub-assembly
    /// whose own overrides cannot speak for the parts beneath it.
    pub(super) fn occurrence_color(
        &self,
        occurrence_ref: usize,
        leaf_bodies: Option<&[Vec<usize>]>,
    ) -> Option<ImportedColor> {
        if let Some(color) = self.whole_occurrence_color(occurrence_ref) {
            return Some(color);
        }
        let bodies = leaf_bodies?;
        let mut found: Option<ImportedColor> = None;
        for items in bodies {
            let color = self.occurrence_item_color(occurrence_ref, items)?;
            if *found.get_or_insert(color) != color {
                return None;
            }
        }
        found
    }

    /// The colour assigned to one entity (a face, a shell, or a solid), if any.
    pub(super) fn color_for(&self, item_ref: usize) -> Option<ImportedColor> {
        self.by_item.get(&item_ref).copied()
    }

    /// Whether the file carried any surface colour at all — lets the callers
    /// skip the per-body face-ref walk entirely on the (common) uncoloured file.
    pub(super) fn is_empty(&self) -> bool {
        self.by_item.is_empty()
    }
}

/// Each `REPRESENTATION_RELATIONSHIP` a `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`
/// places a part through, and the CDSR itself, mapped to the occurrence that
/// placement is: the NEXT_ASSEMBLY_USAGE_OCCURRENCE its PRODUCT_DEFINITION_SHAPE
/// defines.
fn relationship_occurrences(
    entities: &HashMap<usize, Entity>,
) -> (HashMap<usize, usize>, HashMap<usize, Vec<usize>>) {
    let mut map = HashMap::default();
    let mut representations: HashMap<usize, Vec<usize>> = HashMap::default();
    for (&id, entity) in entities.iter() {
        // MAPPED_ITEM(name, mapping_source, mapping_target): the source is a
        // REPRESENTATION_MAP(mapping_origin, mapped_representation).
        if let Some(args) = entity.find("MAPPED_ITEM") {
            if let Some(Ok(mapped)) = args
                .get(1)
                .and_then(|source| source.as_ref_id().ok())
                .and_then(|source| entities.get(&source))
                .and_then(|source| source.find("REPRESENTATION_MAP"))
                .and_then(|map| map.get(1))
                .map(Value::as_ref_id)
            {
                representations.entry(id).or_default().push(mapped);
            }
            continue;
        }
        let Some(args) = entity.find("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") else {
            continue;
        };
        let (Some(Ok(relationship)), Some(Ok(shape))) =
            (args.first().map(Value::as_ref_id), args.get(1).map(Value::as_ref_id))
        else {
            continue;
        };
        let Some(occurrence) = entities
            .get(&shape)
            .and_then(|shape| shape.find("PRODUCT_DEFINITION_SHAPE"))
            .and_then(|args| args.get(2))
            .and_then(|definition| definition.as_ref_id().ok())
        else {
            continue;
        };
        if entities.get(&occurrence).is_some_and(|e| e.has("NEXT_ASSEMBLY_USAGE_OCCURRENCE")) {
            map.insert(relationship, occurrence);
            // The CONTEXT_DEPENDENT_SHAPE_REPRESENTATION names the same
            // occurrence.
            map.insert(id, occurrence);
            // REPRESENTATION_RELATIONSHIP(name, description, rep_1, rep_2):
            // the placed (child) representation is usually rep_1, but files
            // write it both ways round, so both are kept.
            if let Some(args) = entities.get(&relationship).and_then(|entity| entity.find("REPRESENTATION_RELATIONSHIP")) {
                let sides = [2, 3].into_iter().filter_map(|index| args.get(index)?.as_ref_id().ok());
                representations.entry(occurrence).or_default().extend(sides);
            }
        }
    }
    (map, representations)
}

/// The first surface colour reachable from a `STYLED_ITEM`'s `styles` list.
fn style_list_color(entities: &HashMap<usize, Entity>, styles: &Value) -> Option<ImportedColor> {
    for assignment in styles.as_list().ok()? {
        let Ok(assignment_ref) = assignment.as_ref_id() else {
            continue;
        };
        if let Some(color) = presentation_style_color(entities, assignment_ref) {
            return Some(color);
        }
    }
    None
}

/// PRESENTATION_STYLE_ASSIGNMENT(styles) — and its context-bound subtype
/// PRESENTATION_STYLE_BY_CONTEXT(styles, context), whose `styles` is the same
/// first argument (the BIM fixtures use it exclusively).
fn presentation_style_color(
    entities: &HashMap<usize, Entity>,
    assignment_ref: usize,
) -> Option<ImportedColor> {
    let entity = entities.get(&assignment_ref)?;
    let args = entity
        .find("PRESENTATION_STYLE_ASSIGNMENT")
        .or_else(|| entity.find("PRESENTATION_STYLE_BY_CONTEXT"))?;
    for style in args.first()?.as_list().ok()? {
        let Ok(style_ref) = style.as_ref_id() else {
            continue;
        };
        if let Some(color) = surface_style_color(entities, style_ref) {
            return Some(color);
        }
    }
    None
}

/// SURFACE_STYLE_USAGE(side, style) → SURFACE_SIDE_STYLE. Every `side`
/// (.BOTH./.POSITIVE./.NEGATIVE.) is accepted as the face's display colour — the
/// kernel shades one face with one colour, so a side-specific style is still the
/// only colour that face has.
fn surface_style_color(
    entities: &HashMap<usize, Entity>,
    style_ref: usize,
) -> Option<ImportedColor> {
    let usage = entities.get(&style_ref)?.find("SURFACE_STYLE_USAGE")?;
    let side_style_ref = usage.get(1)?.as_ref_id().ok()?;
    let side_style = entities.get(&side_style_ref)?.find("SURFACE_SIDE_STYLE")?;
    // SURFACE_SIDE_STYLE(name, styles) — a SET of style elements; the fill area
    // is the shaded colour, the rendering colour the shader's own. A boundary
    // (curve) style in the same set is edge colour and is skipped.
    for element in side_style.get(1)?.as_list().ok()? {
        let Ok(element_ref) = element.as_ref_id() else {
            continue;
        };
        if let Some(color) = style_element_color(entities, element_ref) {
            return Some(color);
        }
    }
    None
}

/// One SURFACE_SIDE_STYLE element: the fill-area chain, or a rendering colour.
fn style_element_color(
    entities: &HashMap<usize, Entity>,
    element_ref: usize,
) -> Option<ImportedColor> {
    let entity = entities.get(&element_ref)?;
    if let Some(args) = entity.find("SURFACE_STYLE_FILL_AREA") {
        let fill_ref = args.first()?.as_ref_id().ok()?;
        return fill_area_style_color(entities, fill_ref);
    }
    // SURFACE_STYLE_RENDERING(method, colour) and its
    // …_WITH_PROPERTIES(method, colour, properties) superset both carry the
    // colour second. Some writers give ONLY this, with no fill area.
    if let Some(args) = entity
        .find("SURFACE_STYLE_RENDERING_WITH_PROPERTIES")
        .or_else(|| entity.find("SURFACE_STYLE_RENDERING"))
    {
        let colour_ref = args.get(1)?.as_ref_id().ok()?;
        return resolve_colour(entities, colour_ref);
    }
    None
}

/// FILL_AREA_STYLE(name, styles) → FILL_AREA_STYLE_COLOUR(name, colour).
fn fill_area_style_color(
    entities: &HashMap<usize, Entity>,
    fill_ref: usize,
) -> Option<ImportedColor> {
    let args = entities.get(&fill_ref)?.find("FILL_AREA_STYLE")?;
    for style in args.get(1)?.as_list().ok()? {
        let Ok(style_ref) = style.as_ref_id() else {
            continue;
        };
        let Some(colour_args) = entities
            .get(&style_ref)
            .and_then(|entity| entity.find("FILL_AREA_STYLE_COLOUR"))
        else {
            continue;
        };
        let Some(colour_ref) = colour_args.get(1).and_then(|v| v.as_ref_id().ok()) else {
            continue;
        };
        if let Some(color) = resolve_colour(entities, colour_ref) {
            return Some(color);
        }
    }
    None
}

/// A COLOUR select: an explicit `COLOUR_RGB(name, r, g, b)` (0..1 doubles), or a
/// `DRAUGHTING_PRE_DEFINED_COLOUR` naming one of ISO 10303-46's eight.
fn resolve_colour(
    entities: &HashMap<usize, Entity>,
    colour_ref: usize,
) -> Option<ImportedColor> {
    let entity = entities.get(&colour_ref)?;
    if let Some(args) = entity.find("COLOUR_RGB") {
        let r = args.get(1)?.as_real().ok()?;
        let g = args.get(2)?.as_real().ok()?;
        let b = args.get(3)?.as_real().ok()?;
        return Some(ImportedColor::new(r, g, b));
    }
    if let Some(args) = entity.find("DRAUGHTING_PRE_DEFINED_COLOUR") {
        // The name rides on arg 0 of the simple form; on a complex instance it
        // rides on the PRE_DEFINED_ITEM half instead, so try both.
        let name = args
            .first()
            .and_then(string_value)
            .or_else(|| entity.find("PRE_DEFINED_ITEM")?.first().and_then(string_value))?;
        return pre_defined_colour(&name);
    }
    None
}

fn string_value(value: &Value) -> Option<String> {
    match value {
        Value::Str(text) => Some(text.clone()),
        _ => None,
    }
}

/// ISO 10303-46's eight pre-defined colour names, at full saturation.
fn pre_defined_colour(name: &str) -> Option<ImportedColor> {
    let (r, g, b) = match name.trim().to_ascii_lowercase().as_str() {
        "red" => (1.0, 0.0, 0.0),
        "green" => (0.0, 1.0, 0.0),
        "blue" => (0.0, 0.0, 1.0),
        "yellow" => (1.0, 1.0, 0.0),
        "magenta" => (1.0, 0.0, 1.0),
        "cyan" => (0.0, 1.0, 1.0),
        "black" => (0.0, 0.0, 0.0),
        "white" => (1.0, 1.0, 1.0),
        _ => return None,
    };
    Some(ImportedColor::new(r, g, b))
}
