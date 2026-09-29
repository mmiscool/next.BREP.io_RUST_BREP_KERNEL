//! The catalog rules: how a category's schema is inherited, what shape the
//! tree may take, and whether a value fits an attribute.
//!
//! Pure functions over the category list, so the store can call them INSIDE
//! its locked write — the one place a rule cannot be raced past.
//!
//! # The rules (round 1 and the catalog slice)
//!
//! * A category's **effective schema** is its ancestors' attributes, root
//!   first, then its own.
//! * A child may ADD attributes but never redefine one an ancestor defines.
//!   [`check_tree`] enforces that in both directions: a parent cannot gain a
//!   key a descendant already has, either.
//! * The tree has no cycles, and every parent exists.
//! * A value is checked against its attribute's type when it is WRITTEN.
//!   Whether the set is COMPLETE — every required key present — is checked
//!   only at release ([`release_problems`]), because a draft may be unfinished.

use std::collections::BTreeSet;

use serde::Serialize;
use serde_json::Value;

use crate::model::{AttributeDef, AttributeKind, Category, Part};
use crate::Error;

/// The deepest a chain of categories may go. Deep enough for any real
/// catalog; small enough that a walk over a corrupted file stops.
pub const MAX_DEPTH: usize = 32;

/// The category `id`, compared without regard to ASCII case.
pub fn find<'a>(categories: &'a [Category], id: &str) -> Option<&'a Category> {
    let id = id.trim();
    if id.is_empty() {
        return None;
    }
    categories.iter().find(|c| c.id.eq_ignore_ascii_case(id))
}

/// `id` and its ancestors, ROOT FIRST. Refuses a missing parent and a cycle.
pub fn chain<'a>(categories: &'a [Category], id: &str) -> Result<Vec<&'a Category>, Error> {
    let mut out: Vec<&Category> = Vec::new();
    let mut current = id.to_string();
    loop {
        let category = find(categories, &current).ok_or_else(|| {
            if out.is_empty() {
                Error::not_found(format!("category '{current}'"))
            } else {
                Error::conflict(format!(
                    "category '{}' names a parent '{current}' that does not exist",
                    out.last().expect("not empty").id
                ))
            }
        })?;
        if out.iter().any(|seen| seen.id == category.id) {
            return Err(Error::conflict(format!(
                "category '{}' would be its own ancestor",
                category.id
            )));
        }
        if out.len() >= MAX_DEPTH {
            return Err(Error::conflict(format!(
                "categories nest at most {MAX_DEPTH} deep"
            )));
        }
        out.push(category);
        if category.parent.trim().is_empty() {
            break;
        }
        current = category.parent.clone();
    }
    out.reverse();
    Ok(out)
}

/// One attribute of an effective schema, and which category defines it.
#[derive(Debug, Clone, Serialize)]
pub struct Effective {
    #[serde(flatten)]
    pub def: AttributeDef,
    /// The id of the category that defines it — the category itself or an
    /// ancestor.
    pub from: String,
}

/// Every attribute a part in `id` can carry, ancestors' first.
pub fn schema(categories: &[Category], id: &str) -> Result<Vec<Effective>, Error> {
    Ok(chain(categories, id)?
        .into_iter()
        .flat_map(|category| {
            category.attributes.iter().map(move |def| Effective {
                def: def.clone(),
                from: category.id.clone(),
            })
        })
        .collect())
}

/// `id` and every category beneath it, as lower-case ids.
pub fn descendants(categories: &[Category], id: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let Some(root) = find(categories, id) else {
        return out;
    };
    let mut frontier = vec![root.id.to_ascii_lowercase()];
    while let Some(next) = frontier.pop() {
        if !out.insert(next.clone()) {
            continue; // a cycle in a hand-edited file; stop, don't loop
        }
        for child in categories.iter().filter(|c| c.parent.eq_ignore_ascii_case(&next)) {
            frontier.push(child.id.to_ascii_lowercase());
        }
    }
    out
}

/// "Fasteners / Screws" — the names from the root down to `id`.
pub fn path_name(categories: &[Category], id: &str) -> String {
    match chain(categories, id) {
        Ok(chain) => chain.iter().map(|c| c.name.as_str()).collect::<Vec<_>>().join(" / "),
        Err(_) => String::new(),
    }
}

/// Whether a key is one a category may define: lower-case letters, digits,
/// `_` and `-`, starting with a letter.
fn usable_key(key: &str) -> bool {
    key.chars().next().is_some_and(|c| c.is_ascii_lowercase())
        && key
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_' || c == '-')
}

/// A category id follows the part-type id rule: letters, digits, `-`, `_`,
/// stored lower-case.
pub fn normalize_id(id: &str) -> Result<String, Error> {
    let id = id.trim().to_ascii_lowercase();
    if id.is_empty() || !id.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_') {
        return Err(Error::bad_request(
            "a category id is letters, digits, '-' and '_'",
        ));
    }
    Ok(id)
}

/// Tidy and check one category's OWN attribute list: keys lower-case and
/// unique, every attribute named, every enum with at least one distinct value.
pub fn check_definitions(defs: Vec<AttributeDef>) -> Result<Vec<AttributeDef>, Error> {
    let mut out: Vec<AttributeDef> = Vec::with_capacity(defs.len());
    for mut def in defs {
        def.key = def.key.trim().to_ascii_lowercase();
        if !usable_key(&def.key) {
            return Err(Error::bad_request(format!(
                "'{}' is not a usable attribute key — start with a letter; then letters, digits, '_' and '-'",
                def.key
            )));
        }
        def.name = def.name.trim().to_string();
        if def.name.is_empty() {
            def.name = def.key.clone();
        }
        if out.iter().any(|d| d.key == def.key) {
            return Err(Error::bad_request(format!("attribute '{}' is defined twice", def.key)));
        }
        match &mut def.kind {
            AttributeKind::Number { unit } => *unit = unit.trim().to_string(),
            AttributeKind::Enum { values } => {
                let mut tidy: Vec<String> = Vec::new();
                for value in values.iter() {
                    let value = value.trim();
                    if value.is_empty() {
                        continue;
                    }
                    if tidy.iter().any(|v| v.eq_ignore_ascii_case(value)) {
                        return Err(Error::bad_request(format!(
                            "attribute '{}' lists '{value}' twice",
                            def.key
                        )));
                    }
                    tidy.push(value.to_string());
                }
                if tidy.is_empty() {
                    return Err(Error::bad_request(format!(
                        "attribute '{}' is a list of values and needs at least one",
                        def.key
                    )));
                }
                *values = tidy;
            }
            AttributeKind::Text | AttributeKind::Bool => {}
        }
        out.push(def);
    }
    Ok(out)
}

/// Check the WHOLE tree after a change: every parent exists, no cycle, no
/// category deeper than [`MAX_DEPTH`], and no attribute key defined twice
/// along any root-to-leaf chain. Checking every chain is what makes the
/// no-override rule hold in both directions — a child cannot redefine its
/// parent's key, and a parent cannot gain a key a child already defines.
pub fn check_tree(categories: &[Category]) -> Result<(), Error> {
    for category in categories {
        let chain = chain(categories, &category.id)?;
        let mut owner: Vec<(&str, &str)> = Vec::new();
        for link in &chain {
            for def in &link.attributes {
                if let Some((_, first)) = owner.iter().find(|(key, _)| *key == def.key) {
                    return Err(Error::conflict(format!(
                        "attribute '{}' is defined by '{first}' and again by '{}' — a category inherits its parent's attributes and cannot redefine one",
                        def.key, link.id
                    )));
                }
                owner.push((&def.key, &link.id));
            }
        }
    }
    Ok(())
}

/// Check `value` against `def` and return what to store: `None` to clear the
/// attribute (a JSON `null` or an empty string), else the value in its
/// canonical form. The browser sends form text, so a number may arrive as
/// `"12.5"` and a flag as `"true"`; an enum value matches without regard to
/// case and is stored in the spelling the schema lists.
pub fn check_value(def: &AttributeDef, value: &Value) -> Result<Option<Value>, String> {
    if value.is_null() || value.as_str().is_some_and(|s| s.trim().is_empty()) {
        return Ok(None);
    }
    match &def.kind {
        AttributeKind::Text => match value {
            Value::String(text) => Ok(Some(Value::String(text.trim().to_string()))),
            other => Err(format!("{} takes text, not {other}", def.name)),
        },
        AttributeKind::Number { .. } => {
            let number = match value {
                Value::Number(n) => n.as_f64(),
                Value::String(text) => text.trim().parse::<f64>().ok(),
                _ => None,
            };
            match number.filter(|n| n.is_finite()) {
                // Keep integers integral: 12 is stored as 12, not 12.0.
                Some(n) if n.fract() == 0.0 && n.abs() < 9.0e15 => Ok(Some(Value::from(n as i64))),
                Some(n) => Ok(serde_json::Number::from_f64(n).map(Value::Number)),
                None => Err(format!("{} takes a number, not {value}", def.name)),
            }
        }
        AttributeKind::Bool => match value {
            Value::Bool(b) => Ok(Some(Value::Bool(*b))),
            Value::String(text) => match text.trim().to_ascii_lowercase().as_str() {
                "true" | "yes" => Ok(Some(Value::Bool(true))),
                "false" | "no" => Ok(Some(Value::Bool(false))),
                _ => Err(format!("{} takes yes or no, not {value}", def.name)),
            },
            other => Err(format!("{} takes yes or no, not {other}", def.name)),
        },
        AttributeKind::Enum { values } => {
            let text = value.as_str().map(str::trim).unwrap_or_default();
            match values.iter().find(|v| v.eq_ignore_ascii_case(text)) {
                Some(canonical) => Ok(Some(Value::String(canonical.clone()))),
                None => Err(format!("{} is one of {}, not {value}", def.name, values.join(", "))),
            }
        }
    }
}

/// Why `part` cannot release, as far as its catalog values go: every required
/// attribute of its category has a value, and every value its category
/// defines still fits (an administrator may have changed an attribute's type
/// since it was written). An uncategorized part — or one whose category text
/// names no category — has no schema and nothing to check.
pub fn release_problems(categories: &[Category], part: &Part) -> Vec<String> {
    if find(categories, &part.category).is_none() {
        return Vec::new();
    }
    let Ok(schema) = schema(categories, &part.category) else {
        return Vec::new();
    };
    let mut problems = Vec::new();
    for Effective { def, .. } in &schema {
        match part.attributes.get(&def.key) {
            None => {
                if def.required {
                    problems.push(format!("{} is required", def.name));
                }
            }
            Some(value) => match check_value(def, value) {
                Ok(Some(_)) => {}
                Ok(None) => {
                    if def.required {
                        problems.push(format!("{} is required", def.name));
                    }
                }
                Err(message) => problems.push(message),
            },
        }
    }
    problems
}

/// Merge `changes` into a part's `values`, checking each written value against
/// the effective schema of `category`. A `null` or empty value CLEARS its key
/// and is always allowed — including a key the schema no longer defines,
/// which is how an inert value left behind by a category move is removed.
/// Setting any other key needs the part to be in a category that defines it.
pub fn apply_values(
    categories: &[Category],
    category: &str,
    values: &mut std::collections::BTreeMap<String, Value>,
    changes: &serde_json::Map<String, Value>,
) -> Result<(), Error> {
    let schema = match find(categories, category) {
        Some(found) => schema(categories, &found.id)?,
        None => Vec::new(),
    };
    for (key, value) in changes {
        let key = key.trim().to_ascii_lowercase();
        let clearing = value.is_null() || value.as_str().is_some_and(|s| s.trim().is_empty());
        if clearing {
            values.remove(&key);
            continue;
        }
        let Some(Effective { def, .. }) = schema.iter().find(|e| e.def.key == key) else {
            return Err(Error::bad_request(if schema.is_empty() && find(categories, category).is_none() {
                format!("'{key}' cannot be set — choose a category for the part first")
            } else {
                format!(
                    "'{key}' is not an attribute of category '{}'",
                    path_name(categories, category)
                )
            }));
        };
        match check_value(def, value).map_err(Error::bad_request)? {
            Some(stored) => {
                values.insert(key, stored);
            }
            None => {
                values.remove(&key);
            }
        }
    }
    Ok(())
}

/// Tags, trimmed, without empties, and without a repeat that differs only in
/// case. The first spelling wins.
pub fn tidy_tags(tags: impl IntoIterator<Item = String>) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for tag in tags {
        let tag = tag.trim().to_string();
        if !tag.is_empty() && !out.iter().any(|t| t.eq_ignore_ascii_case(&tag)) {
            out.push(tag);
        }
    }
    out
}

