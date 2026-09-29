//! The ten assembly-constraint definitions — ONE MODULE PER CONSTRAINT, the
//! feature-module pattern: each `constraints/<type>.rs` owns everything about its
//! type — its [`ConstraintTypeDef`] row (names, element count range, duplicate
//! family, the `applicable` selection-context predicate), its schema entry, and (for
//! the mate-mapped types) its `map` function. `fixed` has no `map` — grounding is
//! handled by [`super::lifecycle`] before body building.

use crate::feature_pipeline::SelectionProbe;

pub(crate) mod angle;
pub(crate) mod center;
pub(crate) mod coincident;
pub(crate) mod concentric;
pub(crate) mod distance;
pub(crate) mod fixed;
pub(crate) mod parallel;
pub(crate) mod perpendicular;
pub(crate) mod tangent;
pub(crate) mod touch_align;

/// One constraint type's static definition.
pub struct ConstraintTypeDef {
    /// Canonical `type` string (lowercase snake — the persisted key).
    pub type_id: &'static str,
    /// Id-mint prefix (`CONC` → `CONC3`).
    pub short_name: &'static str,
    /// The type's ONE icon — a single catalogued glyph character (the app's
    /// `assets/glyphs` catalog, where Coincident shares `≡` with the sketch
    /// solver's coincident). Every surface that stands for the type by picture
    /// alone — the workbench toolbar button, the viewport chip, the tree row's
    /// glyph column — reads THIS field; nothing derives it from `long_name`.
    pub icon: &'static str,
    /// Display label: `"{icon} {label}"` (the context bar draws the leading
    /// glyph as artwork). Pinned to the other two by a test below.
    pub long_name: &'static str,
    /// Plain label for messages ("Duplicate … conflicts with Distance
    /// constraint DIST4.").
    pub label: &'static str,
    /// The accepted `elements` count range, inclusive: fixed takes exactly 1,
    /// the pairing types exactly 2, center 3 or 4 (two width faces plus a one-
    /// or two-element tab). The lifecycle marks a count outside the range
    /// `incomplete`; the schema's `minSelections`/`maxSelections` mirror it.
    pub min_elements: usize,
    pub max_elements: usize,
    /// Member of the duplicate-detection family (overlapping selection pairs
    /// conflict across these types).
    pub duplicate_family: bool,
    /// Selection-context applicability (the feature `context_applicable`
    /// pattern, [`crate::feature_pipeline::context_offer`]): does the current
    /// selection make creating this constraint meaningful? Every element must
    /// be component geometry ([`super::mapping`]'s `resolve_element` rejects
    /// everything else), and the counted kinds are the ones the app's element
    /// seeder can actually produce (faces/edges, solids as COMPONENT refs —
    /// never vertices, whose `@`-refs only the modal picker builds).
    pub applicable: fn(&SelectionProbe) -> bool,
}

/// Spec §4 table order — also the panel's `+` dropdown order.
pub const CONSTRAINT_TYPES: [ConstraintTypeDef; 10] = [
    fixed::DEF,
    coincident::DEF,
    touch_align::DEF,
    parallel::DEF,
    distance::DEF,
    angle::DEF,
    concentric::DEF,
    perpendicular::DEF,
    tangent::DEF,
    center::DEF,
];

/// Look up a type definition by its canonical `type` string.
pub fn constraint_type(type_id: &str) -> Option<&'static ConstraintTypeDef> {
    CONSTRAINT_TYPES.iter().find(|def| def.type_id == type_id)
}

/// `number` params are expression-capable against the part's expression scope
/// (the dialog engine's established convention).
pub fn constraint_schema_catalogue() -> serde_json::Value {
    serde_json::Value::Array(vec![
        fixed::schema(),
        coincident::schema(),
        touch_align::schema(),
        parallel::schema(),
        distance::schema(),
        angle::schema(),
        concentric::schema(),
        perpendicular::schema(),
        tangent::schema(),
        center::schema(),
    ])
}

/// A two-element pairing across TWO distinct components (a pair on one rigid
/// body cannot move anything), counting `seedable` selected elements — the
/// shared shape behind every pairing type's `applicable`.
pub(super) fn pair_applicable(probe: &SelectionProbe, seedable: usize) -> bool {
    probe.all_component && probe.components == 2 && seedable == 2
}

// --- shared schema-field builders (each module's `schema()` composes these) --

pub(super) fn id_field() -> serde_json::Value {
    serde_json::json!({
        "type": "string",
        "default_value": null,
        "hint": "Unique identifier for the constraint"
    })
}

/// The `elements` reference field: `min..=max` picks from `filter` (the type
/// def's `min_elements..=max_elements` — a test below pins the two together).
pub(super) fn elements_field(
    filter: &[&str],
    min: usize,
    max: usize,
    hint: &str,
) -> serde_json::Value {
    serde_json::json!({
        "type": "reference_selection",
        "selectionFilter": filter,
        "multiple": max > 1,
        "minSelections": min,
        "maxSelections": max,
        "default_value": null,
        "hint": hint
    })
}

pub(super) fn schema_entry(
    def: &ConstraintTypeDef,
    params: serde_json::Value,
) -> serde_json::Value {
    serde_json::json!({
        "type": def.type_id,
        "shortName": def.short_name,
        "longName": def.long_name,
        "label": def.label,
        "icon": def.icon,
        "inputParamsSchema": params,
    })
}

