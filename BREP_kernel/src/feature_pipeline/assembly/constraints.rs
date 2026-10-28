//! The fourteen assembly-constraint definitions — ONE MODULE PER CONSTRAINT, the
//! feature-module pattern: each `constraints/<type>.rs` owns everything about its
//! type — its [`ConstraintTypeDef`] row (names, element count range, duplicate
//! family, the `applicable` selection-context predicate), its schema entry, and (for
//! the mate-mapped types) its `map` function. `fixed` has no `map` — grounding is
//! handled by [`super::lifecycle`] before body building.

use crate::feature_pipeline::SelectionProbe;

pub(crate) mod angle;
pub(crate) mod cam;
pub(crate) mod center;
pub(crate) mod coincident;
pub(crate) mod concentric;
pub(crate) mod distance;
pub(crate) mod fixed;
pub(crate) mod parallel;
pub(crate) mod path;
pub(crate) mod gear;
pub(super) mod mechanical;
pub(crate) mod perpendicular;
pub(crate) mod rack_pinion;
pub(crate) mod screw;
pub(crate) mod slot;
pub(crate) mod symmetric;
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
    /// be component geometry (`super::mapping`'s `resolve_element` rejects
    /// everything else), and the counted kinds are the ones the app's element
    /// seeder can actually produce (faces/edges, solids as COMPONENT refs —
    /// never vertices, whose `@`-refs only the modal picker builds).
    pub applicable: fn(&SelectionProbe) -> bool,
}

/// Spec §4 table order — also the panel's `+` dropdown order.
pub const CONSTRAINT_TYPES: [ConstraintTypeDef; 17] = [
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
    symmetric::DEF,
    gear::DEF,
    rack_pinion::DEF,
    screw::DEF,
    path::DEF,
    slot::DEF,
    cam::DEF,
];

/// The limit-mate fields Distance and Angle share: a `limit` toggle and the
/// `minimum`/`maximum` ends, in the constraint's own value convention.
pub(super) fn limit_fields(unit: &str, step: f64, default_max: f64) -> [(&'static str, serde_json::Value); 3] {
    [
        ("limit", serde_json::json!({
            "type": "boolean",
            "default_value": false,
            "hint": "Limit mate: hold the value anywhere between Minimum and Maximum instead of at the target; a drag stops at the ends"
        })),
        ("minimum", serde_json::json!({
            "type": "number",
            "step": step,
            "default_value": 0,
            "hint": format!("Limit mate: the smallest allowed value ({unit}), in the same convention as the target")
        })),
        ("maximum", serde_json::json!({
            "type": "number",
            "step": step,
            "default_value": default_max,
            "hint": format!("Limit mate: the largest allowed value ({unit}), in the same convention as the target")
        })),
    ]
}

/// Turn a mapped Distance/Angle into its limit mate when `limit` is on: the
/// measuring mate is wrapped in [`crate::MateKind::Limit`] with the entry's
/// `minimum`/`maximum`, mapped through `to_solver` (the display → solver
/// convention: the exterior-angle remap, or the identity).
pub(super) fn apply_limit(
    entry: &super::ConstraintEntry,
    mapped: &mut super::mapping::MappedConstraint,
    env: &crate::feature_pipeline::Env,
    to_solver: impl Fn(f64) -> f64,
) -> Result<(), super::mapping::ConstraintFailure> {
    if !entry.flag("limit") {
        return Ok(());
    }
    let read = |key: &str, default: f64| {
        entry
            .number(key, env, default)
            .map_err(|error| super::mapping::ConstraintFailure::new("error", error))
    };
    let (minimum, maximum) = (read("minimum", 0.0)?, read("maximum", 0.0)?);
    if minimum > maximum {
        return Err(super::mapping::ConstraintFailure::new(
            "error",
            format!("Limit minimum {minimum} is above the maximum {maximum}."),
        ));
    }
    let (a, b) = (to_solver(minimum), to_solver(maximum));
    let (min, max) = if a <= b { (a, b) } else { (b, a) };
    for mapped_mate in &mut mapped.mates {
        let inner = std::mem::replace(&mut mapped_mate.kind, crate::MateKind::Parallel {
            direction_a: [0.0, 0.0, 1.0],
            direction_b: [0.0, 0.0, 1.0],
            align: crate::MateAlign::Any,
        });
        mapped_mate.kind = crate::MateKind::Limit { mate: Box::new(inner), min, max };
    }
    mapped.target = None;
    mapped.note = Some(format!("Limit {minimum} to {maximum}"));
    Ok(())
}

/// The constraint-form params to HIDE for the current `values` (the feature
/// dialogs' `feature_hidden_params` hook): a Distance or Angle shows its
/// `minimum`/`maximum` only with `limit` on, and its single target only with
/// `limit` off (the target is ignored while the range rules).
pub fn constraint_hidden_params(type_id: &str, values: &serde_json::Value) -> Vec<&'static str> {
    let target = match type_id {
        "distance" => "distance",
        "angle" => "angle",
        _ => return Vec::new(),
    };
    if values.get("limit").and_then(|limit| limit.as_bool()).unwrap_or(false) {
        vec![target]
    } else {
        vec!["minimum", "maximum"]
    }
}

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
        symmetric::schema(),
        gear::schema(),
        rack_pinion::schema(),
        screw::schema(),
        path::schema(),
        slot::schema(),
        cam::schema(),
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


/// What **Flip** does to one constraint: the parameter patch that selects the
/// other solution, and how the moving component is seeded so the solve reaches
/// it (`seed`: `turn` = 180° about an axis through the moving element's anchor,
/// perpendicular to its direction; `turn-about-reference` = 180° about the
/// reference element's direction; `mirror` = the moving anchor reflected across
/// the reference plane; `rotate` = −2θ about the angle axis; `none` = the
/// solve alone reaches it, or nothing may move). `Err` names why a type has no
/// other solution. `geoms` are the elements' geometry classes in pick order.
pub fn flip_rule(constraint_type: &str, params: &serde_json::Value, geoms: &[&str]) -> Result<(serde_json::Value, &'static str), String> {
    let toggle = |key: &str| {
        let on = params.get(key).and_then(serde_json::Value::as_bool).unwrap_or(false);
        serde_json::json!({ key: !on })
    };
    let negate = |key: &str| params.get(key).and_then(serde_json::Value::as_f64).map(|value| -value);
    let limited = params.get("limit").and_then(serde_json::Value::as_bool).unwrap_or(false);
    let mirrored_range = || {
        let min = params.get("minimum").and_then(serde_json::Value::as_f64).unwrap_or(0.0);
        let max = params.get("maximum").and_then(serde_json::Value::as_f64).unwrap_or(0.0);
        serde_json::json!({ "minimum": -max, "maximum": -min })
    };
    match constraint_type {
        "touch_align" | "parallel" | "concentric" | "center" => Ok((toggle("reverse"), "turn")),
        "gear" | "rack_pinion" | "screw" => Ok((toggle("reverse"), "none")),
        "path" | "slot" => Ok((toggle("reverse"), "none")),
        "cam" => Ok((toggle("flip"), "turn")),
        "distance" => {
            if geoms.first() != Some(&"plane") {
                return Err("a distance to a line or point has no side to flip to".into());
            }
            if limited {
                return Ok((mirrored_range(), "mirror"));
            }
            let flipped = negate("distance").ok_or("the distance has no value to flip")?;
            Ok((serde_json::json!({ "distance": flipped }), "mirror"))
        }
        "angle" => {
            if limited {
                return Ok((mirrored_range(), "rotate"));
            }
            let flipped = negate("angle").ok_or("the angle has no value to flip")?;
            Ok((serde_json::json!({ "angle": flipped }), "rotate"))
        }
        "tangent" => Ok((serde_json::json!({}), "mirror")),
        "perpendicular" => Ok((serde_json::json!({}), "turn-about-reference")),
        "fixed" => Err("a fixed component has nothing to flip".into()),
        "symmetric" => Err("a symmetric pair has one solution".into()),
        "coincident" => Err("coincident elements have one solution".into()),
        other => Err(format!("'{other}' has no flip")),
    }
}
