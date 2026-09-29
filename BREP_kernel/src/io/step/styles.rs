//! Geometry COLOUR on export — the ISO 10303-46 presentation entities that give
//! an exported body or face the colour the document persisted for it.
//!
//! This is the WRITE half of `io/step_import/styles.rs`, and it writes exactly
//! the chain that module reads:
//!
//! ```text
//! MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(styled …),ctx)
//!   └ STYLED_ITEM('color',(#style),#item)            item = solid or face
//!       └ PRESENTATION_STYLE_ASSIGNMENT((#usage))
//!           └ SURFACE_STYLE_USAGE(.BOTH.,#side)
//!               └ SURFACE_SIDE_STYLE('',(#fill))
//!                   └ SURFACE_STYLE_FILL_AREA(#area)
//!                       └ FILL_AREA_STYLE('',(#colour))
//!                           └ FILL_AREA_STYLE_COLOUR('',#rgb)
//!                               └ COLOUR_RGB('',r,g,b)
//! ```
//!
//! `SURFACE_STYLE_FILL_AREA` (the SHADED colour) is written rather than
//! `SURFACE_STYLE_RENDERING`: it is the form OpenCASCADE's XCAF reader takes
//! first, the form the corpus fixtures overwhelmingly carry, and the only one
//! the importer's fill-area lane needs. `.BOTH.` because the kernel shades one
//! face with one colour and has no side-specific notion to express.
//!
//! **The style ENTITIES are shared with the PMI writer** — `write_colour_rgb`,
//! [`write_style_assignment`] and [`write_styled_item`] here are the single
//! producers, and `step/pmi.rs` writes its null style, its annotation colour and
//! its mapped-shape styled item through them. Colour and annotation
//! presentation stay independent otherwise: a file may carry either, both or
//! neither, and nothing here reads the PMI block.
//!
//! # Where the colour comes from
//!
//! [`StepColors`] is the caller's map from SCENE NAME to an 8-bit sRGB triple —
//! a body name (`Extrude1`, `ACOMP3:Extrude1`) or a face name
//! (`Extrude1_PX`). The caller owns the policy: the app fills it from the
//! document's persisted `color` metadata attribute, which is the same attribute
//! a STEP import stamps and the Info window edits, and adds a name-derived
//! colour only for a body the viewport is itself colouring by name hash. This
//! module never invents a colour: a name with no entry gets no style, and an
//! export with an empty map is byte-for-byte the export that ran before colours
//! existed.
//!
//! The triple is 8-bit because `#RRGGBB` IS the persisted form
//! (`io/appearance.rs`), so the round trip is exact: `n/255` prints through
//! [`real`] and reads back through `round(c * 255) = n`.

use std::collections::BTreeMap;

use super::{id_list, real, ProductGeometry, StepWriter};

/// Display colours to write as presentation styles, keyed by SCENE NAME.
///
/// Deterministic iteration (a `BTreeMap`) so a file's styled items come out in
/// the same order every run; lookups are by exact name, with the occurrence
/// prefixes a structured export resolves through supplied per product.
#[derive(Clone, Debug, Default)]
pub struct StepColors {
    by_name: BTreeMap<String, [u8; 3]>,
}

impl StepColors {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record one entity's colour. An empty name is ignored (no phantom entry).
    pub fn set(&mut self, name: &str, rgb: [u8; 3]) {
        if name.is_empty() {
            return;
        }
        self.by_name.insert(name.to_string(), rgb);
    }

    /// Nothing to style — the whole writer short-circuits on it.
    pub fn is_empty(&self) -> bool {
        self.by_name.is_empty()
    }

    /// How many names carry a colour.
    pub fn len(&self) -> usize {
        self.by_name.len()
    }

    /// The colour for `name` under the first of `prefixes` that has one.
    ///
    /// `prefixes` is the occurrence namespaces a product's geometry is known by
    /// in the document (`""` for the root, `"ACOMP3:"`, `"ACOMP5:ACOMP1:"` …).
    /// A part placed several times has ONE written geometry, so it can carry
    /// only one colour per entity: the first prefix in the caller's order wins,
    /// the same "a shared part resolves to the one face" rule the PMI writer
    /// documents for annotations.
    fn resolve(&self, prefixes: &[String], name: &str) -> Option<[u8; 3]> {
        prefixes.iter().find_map(|prefix| {
            if prefix.is_empty() {
                self.by_name.get(name).copied()
            } else {
                self.by_name.get(&format!("{prefix}{name}")).copied()
            }
        })
    }
}

/// `COLOUR_RGB('',r,g,b)` — the ONE producer, shared with the PMI writer.
/// Components are 0..=1 sRGB, which is what every consumer of an exchange
/// colour treats them as (`io/appearance.rs` "Units").
pub(crate) fn write_colour_rgb(
    writer: &mut StepWriter,
    rgb: [f64; 3],
) -> Result<usize, String> {
    Ok(writer.add(format!(
        "COLOUR_RGB('',{},{},{})",
        real(rgb[0])?,
        real(rgb[1])?,
        real(rgb[2])?
    )))
}

/// `PRESENTATION_STYLE_ASSIGNMENT((…))` over `elements`, already rendered as a
/// Part 21 list body (`#12` references, or an inline `NULL_STYLE(.NULL.)`).
pub(crate) fn write_style_assignment(writer: &mut StepWriter, elements: &str) -> usize {
    writer.add(format!("PRESENTATION_STYLE_ASSIGNMENT(({elements}))"))
}

/// `STYLED_ITEM(name,(#style),#item)` — one style assignment on one item.
pub(crate) fn write_styled_item(
    writer: &mut StepWriter,
    name: &str,
    style: usize,
    item: usize,
) -> usize {
    writer.add(format!("STYLED_ITEM('{name}',(#{style}),#{item})"))
}

/// The styled items one product's geometry contributes: its coloured bodies
/// first (in body order), then its coloured faces (in face order).
///
/// Bodies before faces is deliberate — it is the precedence a reader that keeps
/// only one style per shell sees, and it matches the importer's own reading
/// (a body colour is the body's, a face colour overrides it for that face).
pub(crate) fn styled_items(
    geometry: &ProductGeometry,
    colors: &StepColors,
    prefixes: &[String],
) -> Vec<(usize, [u8; 3])> {
    let mut items = Vec::new();
    for (name, id) in &geometry.solids {
        if let Some(rgb) = colors.resolve(prefixes, name) {
            items.push((*id, rgb));
        }
    }
    for (name, id) in &geometry.faces {
        if let Some(rgb) = colors.resolve(prefixes, name) {
            items.push((*id, rgb));
        }
    }
    items
}

/// Write the file's geometry styles: one style chain per DISTINCT colour, one
/// `STYLED_ITEM` per entity, and one
/// `MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION` gathering them.
///
/// Returns how many styled items were written — `0` for an empty `items`, in
/// which case NOTHING is emitted (not an empty presentation representation,
/// which several readers reject and which would change every uncoloured file).
pub(crate) fn write_geometry_styles(
    writer: &mut StepWriter,
    geometry_context: usize,
    items: &[(usize, [u8; 3])],
) -> Result<usize, String> {
    if items.is_empty() {
        return Ok(0);
    }
    // One chain per distinct colour, minted on first use so entity order
    // follows `items` order. Keyed on the 8-bit triple, which IS the colour's
    // persisted identity.
    let mut assignments: BTreeMap<[u8; 3], usize> = BTreeMap::new();
    let mut styled = Vec::with_capacity(items.len());
    for (item, rgb) in items {
        let assignment = match assignments.get(rgb) {
            Some(id) => *id,
            None => {
                let id = write_surface_style(writer, *rgb)?;
                assignments.insert(*rgb, id);
                id
            }
        };
        styled.push(write_styled_item(writer, "color", assignment, *item));
    }
    writer.add(format!(
        "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',{},#{geometry_context})",
        id_list(&styled)
    ));
    Ok(styled.len())
}

/// One colour's whole chain, `COLOUR_RGB` up to the
/// `PRESENTATION_STYLE_ASSIGNMENT` a `STYLED_ITEM` names.
fn write_surface_style(writer: &mut StepWriter, rgb: [u8; 3]) -> Result<usize, String> {
    let colour = write_colour_rgb(
        writer,
        [
            rgb[0] as f64 / 255.0,
            rgb[1] as f64 / 255.0,
            rgb[2] as f64 / 255.0,
        ],
    )?;
    let fill_colour = writer.add(format!("FILL_AREA_STYLE_COLOUR('',#{colour})"));
    let fill_area = writer.add(format!("FILL_AREA_STYLE('',(#{fill_colour}))"));
    let fill_style = writer.add(format!("SURFACE_STYLE_FILL_AREA(#{fill_area})"));
    let side_style = writer.add(format!("SURFACE_SIDE_STYLE('',(#{fill_style}))"));
    let usage = writer.add(format!("SURFACE_STYLE_USAGE(.BOTH.,#{side_style})"));
    Ok(write_style_assignment(writer, &format!("#{usage}")))
}

