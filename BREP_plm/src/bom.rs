//! Assembly structure: the "uses" lists, and the BOM and where-used views
//! computed from them.
//!
//! The CAD app publishes a revision's uses list on save and release (round
//! 1); the server never reads geometry. Everything in this file is a pure
//! function of [`State`]: checking a list before it is stored, walking it
//! down into a bill of materials, walking it up into a where-used answer,
//! and naming the children a release gate cares about.
//!
//! # What a reference means
//!
//! A [`Use`] names its child revision by id — PINNED — or leaves it empty —
//! FLOATING, which reads as the child's current release, else its newest
//! revision, at the moment the BOM is read. A floating reference stays
//! floating after its parent releases: the released BOM follows the child's
//! releases (BOM defaults).
//!
//! # Cycles
//!
//! Checked at the PART level across every revision: a list may not name a
//! part from which the parent part can be reached again through any
//! revision's uses. That is stricter than a revision-level check, and it is
//! what keeps a floating reference — which may resolve to any revision — from
//! ever closing a loop.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use serde::Serialize;
use serde_json::Value;

use crate::catalog;
use crate::db::State;
use crate::model::{Lifecycle, Part, Revision, Use};
use crate::Error;

/// How deep a BOM or where-used walk goes before it stops and says so. Cycles
/// are refused on write, so only a store written before that check could
/// reach this.
pub const MAX_DEPTH: usize = 32;

// ===========================================================================
// Resolving a reference
// ===========================================================================

/// The revision a use points at: the pinned one, or for a floating use the
/// child's current release, else its newest revision. `None` when a pinned
/// revision no longer exists (a deleted draft from before that was refused).
pub fn resolve<'a>(part: &'a Part, line: &Use) -> Option<&'a Revision> {
    if line.revision.is_empty() {
        part.current_release().or(part.latest())
    } else {
        part.revision(&line.revision)
    }
}

/// A revision of `part` named by id or by label.
pub fn find_revision<'a>(part: &'a Part, key: &str) -> Option<&'a Revision> {
    let key = key.trim();
    part.revision(key).or_else(|| part.revision_by_label(key))
}

// ===========================================================================
// Checking a list before it is stored
// ===========================================================================

/// Turn what a client sent into the list to store, or refuse it.
///
/// Accepts `{"uses": [...]}` or a bare list. Each line names its part by id or
/// number and its revision by id or label (empty for floating); both are
/// stored as ids. Refused: an unknown part or revision, the parent itself, a
/// line that would close a cycle, a quantity that is not a positive number,
/// and the same part, revision and find number listed twice.
pub fn check_uses(state: &State, parent_id: &str, body: &Value) -> Result<Vec<Use>, Error> {
    let list = match body {
        Value::Array(list) => list,
        Value::Object(map) => map
            .get("uses")
            .and_then(Value::as_array)
            .ok_or_else(|| Error::bad_request("send {\"uses\": [ ... ]} or a list"))?,
        _ => return Err(Error::bad_request("send {\"uses\": [ ... ]} or a list")),
    };
    let parent = state.part(parent_id).ok_or_else(|| Error::not_found("part"))?;
    let mut out: Vec<Use> = Vec::with_capacity(list.len());
    for (index, raw) in list.iter().enumerate() {
        let line = index + 1;
        let fields = raw
            .as_object()
            .ok_or_else(|| Error::bad_request(format!("line {line} is not an object")))?;
        for key in fields.keys() {
            if !matches!(
                key.as_str(),
                "part" | "revision" | "quantity" | "unit" | "find_number" | "reference" | "notes"
            ) {
                return Err(Error::bad_request(format!(
                    "line {line}: '{key}' is not a field of a use — part, revision, quantity, unit, find_number, reference, notes"
                )));
            }
        }
        let text = |key: &str| -> Result<String, Error> {
            match fields.get(key) {
                None | Some(Value::Null) => Ok(String::new()),
                Some(Value::String(s)) => Ok(s.trim().to_string()),
                Some(Value::Number(n)) => Ok(n.to_string()),
                Some(_) => Err(Error::bad_request(format!("line {line}: {key} must be text"))),
            }
        };
        let part_key = text("part")?;
        if part_key.is_empty() {
            return Err(Error::bad_request(format!("line {line} names no part")));
        }
        let child = state
            .part_by_id_or_number(&part_key)
            .ok_or_else(|| Error::bad_request(format!("line {line}: there is no part '{part_key}'")))?;
        if child.id == parent.id {
            return Err(Error::bad_request(format!(
                "line {line}: {} cannot use itself",
                parent.number
            )));
        }
        let revision_key = text("revision")?;
        let revision = if revision_key.is_empty() {
            String::new()
        } else {
            find_revision(child, &revision_key)
                .map(|r| r.id.clone())
                .ok_or_else(|| {
                    Error::bad_request(format!(
                        "line {line}: {} has no revision '{revision_key}'",
                        child.number
                    ))
                })?
        };
        let quantity = match fields.get("quantity") {
            None | Some(Value::Null) => 1.0,
            Some(Value::Number(n)) => n.as_f64().unwrap_or(f64::NAN),
            Some(Value::String(s)) if s.trim().is_empty() => 1.0,
            Some(Value::String(s)) => s.trim().parse::<f64>().unwrap_or(f64::NAN),
            Some(_) => f64::NAN,
        };
        if !quantity.is_finite() || quantity <= 0.0 {
            return Err(Error::bad_request(format!(
                "line {line}: the quantity of {} must be a number above zero",
                child.number
            )));
        }
        let unit = text("unit")?;
        let entry = Use {
            part: child.id.clone(),
            revision,
            quantity,
            unit: if unit.is_empty() { "each".to_string() } else { unit },
            find_number: text("find_number")?,
            reference: text("reference")?,
            notes: text("notes")?,
        };
        if let Some(same) = out.iter().position(|u| {
            u.part == entry.part && u.revision == entry.revision && u.find_number == entry.find_number
        }) {
            return Err(Error::bad_request(format!(
                "line {line} repeats line {} ({}) — give one line its count as the quantity, or a different find number",
                same + 1,
                child.number
            )));
        }
        out.push(entry);
    }
    for entry in &out {
        if let Some(path) = path_to(state, &entry.part, &parent.id) {
            let names: Vec<String> = path
                .iter()
                .map(|id| state.part(id).map(|p| p.number.clone()).unwrap_or_else(|| id.clone()))
                .collect();
            return Err(Error::conflict(format!(
                "{} cannot use {}: that would make a cycle ({} → {})",
                parent.number,
                names[0],
                parent.number,
                names.join(" → ")
            )));
        }
    }
    Ok(out)
}

/// A chain of part ids from `from` to `to` through any revision's uses, if
/// one exists. Breadth-first, so the chain named in a refusal is the
/// shortest.
pub fn path_to(state: &State, from: &str, to: &str) -> Option<Vec<String>> {
    let mut previous: BTreeMap<String, String> = BTreeMap::new();
    let mut seen: BTreeSet<String> = BTreeSet::from([from.to_string()]);
    let mut queue = std::collections::VecDeque::from([from.to_string()]);
    while let Some(id) = queue.pop_front() {
        if id == to {
            let mut path = vec![id.clone()];
            let mut at = id;
            while let Some(back) = previous.get(&at) {
                path.push(back.clone());
                at = back.clone();
            }
            path.reverse();
            return Some(path);
        }
        let Some(part) = state.part(&id) else { continue };
        for child in part.revisions.iter().flat_map(|r| &r.uses).map(|u| &u.part) {
            if seen.insert(child.clone()) {
                previous.insert(child.clone(), id.clone());
                queue.push_back(child.clone());
            }
        }
    }
    None
}

// ===========================================================================
// The release gate
// ===========================================================================

/// One sentence per child of `revision` that is not Released — what
/// [`crate::model::Settings::require_released_children`] refuses on, and what
/// a release warns about when that setting is off. Empty when every child is
/// released (or the revision uses nothing).
pub fn unreleased_children(state: &State, revision: &Revision) -> Vec<String> {
    let mut out = Vec::new();
    for line in &revision.uses {
        let Some(child) = state.part(&line.part) else {
            out.push(format!("a used part ({}) no longer exists", line.part));
            continue;
        };
        match resolve(child, line) {
            None => out.push(format!("{}: the revision it names no longer exists", child.number)),
            Some(_) if line.revision.is_empty() && child.current_release().is_none() => {
                out.push(format!("{} has no released revision", child.number))
            }
            Some(rev) if rev.lifecycle != Lifecycle::Released => {
                out.push(format!("{} rev {} is {}", child.number, rev.label, rev.lifecycle.as_str()))
            }
            Some(_) => {}
        }
    }
    out
}

/// The uses list with each child resolved to what a person reads — the
/// `uses` a release hook receives.
pub fn resolved_uses(state: &State, revision: &Revision) -> Value {
    Value::Array(
        revision
            .uses
            .iter()
            .map(|line| {
                let child = state.part(&line.part);
                let rev = child.and_then(|c| resolve(c, line));
                serde_json::json!({
                    "part": line.part,
                    "number": child.map(|c| c.number.clone()).unwrap_or_default(),
                    "name": child.map(|c| c.name.clone()).unwrap_or_default(),
                    "revision": rev.map(|r| r.id.clone()).unwrap_or_default(),
                    "revision_label": rev.map(|r| r.label.clone()).unwrap_or_default(),
                    "state": rev.map(|r| r.lifecycle.as_str()).unwrap_or(""),
                    "floating": line.revision.is_empty(),
                    "quantity": line.quantity,
                    "unit": line.unit,
                    "find_number": line.find_number,
                    "reference": line.reference,
                    "notes": line.notes,
                })
            })
            .collect(),
    )
}

// ===========================================================================
// The bill of materials
// ===========================================================================

/// One line of a BOM. In the indented view it is one use at one place in the
/// tree; in the flat view it is every occurrence of one part revision (and
/// unit) added up.
#[derive(Debug, Clone, Serialize)]
pub struct BomLine {
    /// Depth below the top: 1 for what the top uses directly. 0 in the flat
    /// view.
    pub level: usize,
    /// `1.2.3` — the line's place in the tree. Empty in the flat view.
    pub position: String,
    pub find_number: String,
    pub reference: String,
    pub notes: String,
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub revision_label: String,
    /// The resolved revision's state; empty when it no longer exists.
    pub state: String,
    /// The use names no revision; this one is what it resolves to now.
    pub floating: bool,
    pub category_path: String,
    /// The part's catalog values as "Name: value unit", schema order.
    pub attributes: Vec<String>,
    /// How many per ONE parent. In the flat view, the same as `total`.
    pub quantity: f64,
    pub unit: String,
    /// How many for ONE of the top part: the quantities along the path
    /// multiplied, and in the flat view summed over every path.
    pub total: f64,
    /// This part revision uses something itself.
    pub assembly: bool,
    /// The preferred manufacturer part number, else the first; empty with no
    /// sourcing.
    pub mpn: String,
    pub manufacturer: String,
    /// The offer priced here: the cheapest of the MPN's offers at `total`.
    pub supplier: String,
    pub spn: String,
    pub currency: String,
    /// Each, at `total` — the price break `total` reaches, or the first break
    /// when `total` is below it (the least a supplier sells).
    pub unit_price: Option<f64>,
    /// `unit_price × total`.
    pub extended: Option<f64>,
    /// Counted in the totals: a priced line with nothing beneath it. An
    /// assembly's own price is shown but its children are what is bought.
    pub costed: bool,
    pub warnings: Vec<String>,
    /// One short word per warning, in the same order, for a chip in a table
    /// row: `unreleased`, `superseded`, `obsolete`, `no release`, `missing`,
    /// `bake`, `cycle`, `too deep`.
    pub flags: Vec<String>,    /// The files on the part and on the resolved revision, by name (part
    /// files first) — a drawing to hand the shop with the list.
    pub attachments: Vec<String>,
}

/// A currency and what the costed lines in it add up to. Currencies are
/// never added to each other.
#[derive(Debug, Clone, Serialize)]
pub struct CurrencyTotal {
    pub currency: String,
    pub total: f64,
    pub lines: usize,
}

#[derive(Debug, Clone, Serialize)]
pub struct Bom {
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub revision_label: String,
    pub state: String,
    pub flat: bool,
    /// The depth shown in the indented view; 0 is every level. Totals are
    /// always over the whole tree.
    pub levels: usize,
    pub lines: Vec<BomLine>,
    pub totals: Vec<CurrencyTotal>,
    /// Lines with nothing beneath them and no price — what the totals miss.
    pub unpriced: usize,
    /// Every line's warnings, prefixed with the line's part and revision,
    /// once each.
    pub warnings: Vec<String>,
}

/// How a walk reached a line.
struct Walked<'a> {
    level: usize,
    position: String,
    line: &'a Use,
    child: Option<&'a Part>,
    revision: Option<&'a Revision>,
    total: f64,
    /// Where the walk stopped instead of descending.
    stop: Option<String>,
}

/// Walk the whole tree beneath `revision`, depth-first in list order.
fn walk<'a>(state: &'a State, top: &'a Part, revision: &'a Revision) -> Vec<Walked<'a>> {
    let mut out = Vec::new();
    let mut path = vec![top.id.clone()];
    descend(state, revision, 1, "", 1.0, &mut path, &mut out);
    out
}

fn descend<'a>(
    state: &'a State,
    revision: &'a Revision,
    level: usize,
    prefix: &str,
    multiplier: f64,
    path: &mut Vec<String>,
    out: &mut Vec<Walked<'a>>,
) {
    for (index, line) in revision.uses.iter().enumerate() {
        let position = if prefix.is_empty() {
            format!("{}", index + 1)
        } else {
            format!("{prefix}.{}", index + 1)
        };
        let child = state.part(&line.part);
        let resolved = child.and_then(|c| resolve(c, line));
        let total = multiplier * line.quantity;
        let mut stop = None;
        if child.is_some_and(|c| path.contains(&c.id)) {
            stop = Some("cycle: this part is already above it — not expanded".to_string());
        } else if level >= MAX_DEPTH && resolved.is_some_and(|r| !r.uses.is_empty()) {
            stop = Some(format!("deeper than {MAX_DEPTH} levels — not expanded"));
        }
        let expand = stop.is_none();
        out.push(Walked { level, position: position.clone(), line, child, revision: resolved, total, stop });
        if expand {
            if let (Some(child), Some(rev)) = (child, resolved) {
                if !rev.uses.is_empty() {
                    path.push(child.id.clone());
                    descend(state, rev, level + 1, &position, total, path, out);
                    path.pop();
                }
            }
        }
    }
}

/// The price of `part` at `quantity`: the preferred MPN (else the first), and
/// among its offers the cheapest at that quantity. Offers in another currency
/// than the first priced offer's are not compared with it.
fn price(state: &State, part: &Part, quantity: f64) -> Option<(String, String, String, String, String, f64)> {
    let mp = part.sourcing.iter().find(|m| m.preferred).or(part.sourcing.first())?;
    let manufacturer = crate::sourcing::find_company(&state.manufacturers, &mp.manufacturer)
        .map(|c| c.name.clone())
        .unwrap_or_default();
    let mut best: Option<(&crate::model::SupplierOffer, f64)> = None;
    for offer in &mp.offers {
        let mut breaks: Vec<_> = offer.price_breaks.iter().collect();
        breaks.sort_by_key(|b| b.qty);
        let Some(first) = breaks.first() else { continue };
        let unit = breaks
            .iter()
            .rev()
            .find(|b| (b.qty as f64) <= quantity)
            .map(|b| b.unit_price)
            .unwrap_or(first.unit_price);
        match best {
            Some((held, _)) if held.currency != offer.currency => {}
            Some((_, cheapest)) if cheapest <= unit => {}
            _ => best = Some((offer, unit)),
        }
    }
    let (mpn, manufacturer) = (mp.mpn.clone(), manufacturer);
    match best {
        Some((offer, unit)) => {
            let supplier = crate::sourcing::find_company(&state.suppliers, &offer.supplier)
                .map(|c| c.name.clone())
                .unwrap_or_default();
            Some((mpn, manufacturer, supplier, offer.spn.clone(), offer.currency.clone(), unit))
        }
        None => Some((mpn, manufacturer, String::new(), String::new(), String::new(), f64::NAN)),
    }
}

/// The part's catalog values as "Name: value unit", in schema order.
fn attribute_texts(state: &State, part: &Part) -> Vec<String> {
    let Some(category) = catalog::find(&state.categories, &part.category) else {
        return Vec::new();
    };
    let Ok(schema) = catalog::schema(&state.categories, &category.id) else {
        return Vec::new();
    };
    schema
        .iter()
        .filter_map(|e| {
            let value = part.attributes.get(&e.def.key)?;
            let text = match value {
                Value::String(s) => s.clone(),
                Value::Bool(true) => "yes".into(),
                Value::Bool(false) => "no".into(),
                other => other.to_string(),
            };
            let unit = match &e.def.kind {
                crate::model::AttributeKind::Number { unit } if !unit.is_empty() => format!(" {unit}"),
                _ => String::new(),
            };
            Some(format!("{}: {text}{unit}", e.def.name))
        })
        .collect()
}

/// What is wrong with the revision a line resolved to: a short flag and the
/// sentence a person reads.
fn line_warnings(child: Option<&Part>, line: &Use, resolved: Option<&Revision>) -> Vec<(&'static str, String)> {
    let mut out = Vec::new();
    let Some(child) = child else {
        out.push(("missing", "the part no longer exists".to_string()));
        return out;
    };
    let Some(rev) = resolved else {
        out.push(("missing", "the revision it names no longer exists".to_string()));
        return out;
    };
    let current = child.current_release();
    let current_text = match current {
        Some(c) => format!("{} is current", c.label),
        None => "nothing is released now".to_string(),
    };
    match rev.lifecycle {
        Lifecycle::Released => {}
        Lifecycle::Superseded => out.push(("superseded", format!("uses superseded rev {}; {current_text}", rev.label))),
        Lifecycle::Obsolete => out.push(("obsolete", format!("uses obsolete rev {}; {current_text}", rev.label))),
        Lifecycle::Draft | Lifecycle::InReview if line.revision.is_empty() => out.push((
            "no release",
            format!(
                "floating, and {} has no released revision — showing its newest, {} ({})",
                child.number,
                rev.label,
                rev.lifecycle.as_str()
            ),
        )),
        Lifecycle::Draft | Lifecycle::InReview => {
            out.push(("unreleased", format!("unreleased: rev {} is {}", rev.label, rev.lifecycle.as_str())))
        }
    }
    if rev.needs_bake() {
        out.push(("bake", format!("rev {} is waiting for a bake", rev.label)));
    }
    out
}

/// The BOM of `revision` of `top`. `levels` limits the indented view's depth
/// (0 = all); `flat` adds every occurrence of each part revision together.
pub fn bom(state: &State, top: &Part, revision: &Revision, levels: usize, flat: bool) -> Bom {
    let walked = walk(state, top, revision);
    let mut lines: Vec<BomLine> = Vec::new();
    if flat {
        // Key: part, revision, unit — a part bought in two units stays two
        // lines rather than adding millimetres to pieces.
        let mut index: BTreeMap<(String, String, String), usize> = BTreeMap::new();
        for w in &walked {
            let key = (
                w.line.part.clone(),
                w.revision.map(|r| r.id.clone()).unwrap_or_default(),
                w.line.unit.clone(),
            );
            match index.get(&key) {
                Some(&at) => {
                    let line = &mut lines[at];
                    line.total += w.total;
                    line.quantity = line.total;
                    if !w.line.find_number.is_empty() && !line.find_number.split(", ").any(|f| f == w.line.find_number) {
                        if !line.find_number.is_empty() {
                            line.find_number.push_str(", ");
                        }
                        line.find_number.push_str(&w.line.find_number);
                    }
                    if !w.line.reference.is_empty() {
                        if !line.reference.is_empty() {
                            line.reference.push_str(", ");
                        }
                        line.reference.push_str(&w.line.reference);
                    }
                    if let Some(stop) = &w.stop {
                        if !line.warnings.contains(stop) {
                            line.warnings.push(stop.clone());
                            line.flags.push(if stop.starts_with("cycle") { "cycle" } else { "too deep" }.to_string());
                        }
                    }
                }
                None => {
                    index.insert(key, lines.len());
                    let mut line = make_line(state, w);
                    line.level = 0;
                    line.position.clear();
                    line.quantity = line.total;
                    lines.push(line);
                }
            }
        }
        // Priced at the total bought.
        for line in &mut lines {
            reprice(state, line);
        }
        lines.sort_by(|a, b| a.number.to_ascii_lowercase().cmp(&b.number.to_ascii_lowercase()).then(a.revision_label.cmp(&b.revision_label)));
    } else {
        for w in &walked {
            let mut line = make_line(state, w);
            reprice(state, &mut line);
            lines.push(line);
        }
    }

    let mut totals: BTreeMap<String, (f64, usize)> = BTreeMap::new();
    let mut unpriced = 0;
    for line in &lines {
        if line.assembly {
            continue;
        }
        match line.extended {
            Some(extended) if line.costed => {
                let slot = totals.entry(line.currency.clone()).or_insert((0.0, 0));
                slot.0 += extended;
                slot.1 += 1;
            }
            _ => unpriced += 1,
        }
    }
    let mut warnings: Vec<String> = Vec::new();
    for line in &lines {
        for warning in &line.warnings {
            let text = format!("{} rev {}: {warning}", line.number, if line.revision_label.is_empty() { "?" } else { &line.revision_label });
            if !warnings.contains(&text) {
                warnings.push(text);
            }
        }
    }
    if !flat && levels > 0 {
        lines.retain(|line| line.level <= levels);
    }
    Bom {
        part_id: top.id.clone(),
        number: top.number.clone(),
        name: top.name.clone(),
        revision_id: revision.id.clone(),
        revision_label: revision.label.clone(),
        state: revision.lifecycle.as_str().to_string(),
        flat,
        levels,
        lines,
        totals: totals
            .into_iter()
            .map(|(currency, (total, lines))| CurrencyTotal { currency, total, lines })
            .collect(),
        unpriced,
        warnings,
    }
}

fn make_line(state: &State, w: &Walked) -> BomLine {
    let child = w.child;
    let mut found = line_warnings(child, w.line, w.revision);
    if let Some(stop) = &w.stop {
        found.push((if stop.starts_with("cycle") { "cycle" } else { "too deep" }, stop.clone()));
    }
    let flags = found.iter().map(|(flag, _)| flag.to_string()).collect();
    let warnings = found.into_iter().map(|(_, text)| text).collect();
    BomLine {
        level: w.level,
        position: w.position.clone(),
        find_number: w.line.find_number.clone(),
        reference: w.line.reference.clone(),
        notes: w.line.notes.clone(),
        part_id: w.line.part.clone(),
        number: child.map(|c| c.number.clone()).unwrap_or_else(|| "(removed part)".into()),
        name: child.map(|c| c.name.clone()).unwrap_or_default(),
        revision_id: w.revision.map(|r| r.id.clone()).unwrap_or_default(),
        revision_label: w.revision.map(|r| r.label.clone()).unwrap_or_default(),
        state: w.revision.map(|r| r.lifecycle.as_str().to_string()).unwrap_or_default(),
        floating: w.line.revision.is_empty(),
        category_path: child.map(|c| catalog::path_name(&state.categories, &c.category)).unwrap_or_default(),
        attributes: child.map(|c| attribute_texts(state, c)).unwrap_or_default(),
        quantity: w.line.quantity,
        unit: w.line.unit.clone(),
        total: w.total,
        assembly: w.revision.is_some_and(|r| !r.uses.is_empty()),
        mpn: String::new(),
        manufacturer: String::new(),
        supplier: String::new(),
        spn: String::new(),
        currency: String::new(),
        unit_price: None,
        extended: None,
        costed: false,
        warnings,
        flags,
        attachments: child
            .map(|c| c.attachments.iter().map(|a| a.name.clone()).collect::<Vec<_>>())
            .unwrap_or_default()
            .into_iter()
            .chain(w.revision.map(|r| r.attachments.iter().map(|a| a.name.clone()).collect::<Vec<_>>()).unwrap_or_default())
            .collect(),
    }
}

/// Fill a line's sourcing and price at its `total`.
fn reprice(state: &State, line: &mut BomLine) {
    let Some(part) = state.part(&line.part_id) else { return };
    let Some((mpn, manufacturer, supplier, spn, currency, unit)) = price(state, part, line.total) else {
        return;
    };
    line.mpn = mpn;
    line.manufacturer = manufacturer;
    line.supplier = supplier;
    line.spn = spn;
    line.currency = currency;
    if unit.is_finite() {
        line.unit_price = Some(unit);
        line.extended = Some(unit * line.total);
        line.costed = !line.assembly;
    }
}

// ===========================================================================
// Comparing two revisions' lists
// ===========================================================================

/// One side of a diff line: what one revision's list says about one part.
#[derive(Debug, Clone, Serialize, PartialEq)]
pub struct DiffSide {
    pub quantity: f64,
    pub unit: String,
    /// The revision labels the lines name, `floating` for a floating line.
    pub revisions: Vec<String>,
    pub find_numbers: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct DiffLine {
    pub part_id: String,
    pub number: String,
    pub name: String,
    /// `added`, `removed` or `changed`.
    pub change: &'static str,
    pub from: Option<DiffSide>,
    pub to: Option<DiffSide>,
    /// "quantity 2 → 4", "revision B → C", "find number 3 → 5".
    pub details: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct BomDiff {
    pub part_id: String,
    pub number: String,
    pub from_revision: String,
    pub to_revision: String,
    pub lines: Vec<DiffLine>,
    /// Parts whose lines are the same in both.
    pub unchanged: usize,
}

fn sides(state: &State, revision: &Revision) -> BTreeMap<String, DiffSide> {
    let mut out: BTreeMap<String, DiffSide> = BTreeMap::new();
    for line in &revision.uses {
        let label = if line.revision.is_empty() {
            "floating".to_string()
        } else {
            state
                .part(&line.part)
                .and_then(|p| p.revision(&line.revision))
                .map(|r| r.label.clone())
                .unwrap_or_else(|| "(deleted)".into())
        };
        let side = out.entry(line.part.clone()).or_insert_with(|| DiffSide {
            quantity: 0.0,
            unit: line.unit.clone(),
            revisions: Vec::new(),
            find_numbers: Vec::new(),
        });
        side.quantity += line.quantity;
        if side.unit != line.unit && !side.unit.contains(&line.unit) {
            side.unit = format!("{}, {}", side.unit, line.unit);
        }
        if !side.revisions.contains(&label) {
            side.revisions.push(label);
        }
        if !line.find_number.is_empty() && !side.find_numbers.contains(&line.find_number) {
            side.find_numbers.push(line.find_number.clone());
        }
    }
    for side in out.values_mut() {
        side.revisions.sort();
        side.find_numbers.sort();
    }
    out
}

/// What changed in the uses list from `from` to `to`, one level, part by
/// part.
pub fn diff(state: &State, part: &Part, from: &Revision, to: &Revision) -> BomDiff {
    let a = sides(state, from);
    let b = sides(state, to);
    let keys: BTreeSet<&String> = a.keys().chain(b.keys()).collect();
    let mut lines = Vec::new();
    let mut unchanged = 0;
    for key in keys {
        let child = state.part(key);
        let (number, name) = child
            .map(|c| (c.number.clone(), c.name.clone()))
            .unwrap_or_else(|| ("(removed part)".into(), String::new()));
        let (before, after) = (a.get(key), b.get(key));
        let (change, details) = match (before, after) {
            (None, Some(s)) => ("added", vec![format!("{} {}", qty(s.quantity), s.unit)]),
            (Some(s), None) => ("removed", vec![format!("{} {}", qty(s.quantity), s.unit)]),
            (Some(x), Some(y)) if x == y => {
                unchanged += 1;
                continue;
            }
            (Some(x), Some(y)) => {
                let mut details = Vec::new();
                if x.quantity != y.quantity || x.unit != y.unit {
                    details.push(format!("quantity {} {} → {} {}", qty(x.quantity), x.unit, qty(y.quantity), y.unit));
                }
                if x.revisions != y.revisions {
                    details.push(format!("revision {} → {}", x.revisions.join(", "), y.revisions.join(", ")));
                }
                if x.find_numbers != y.find_numbers {
                    let show = |f: &Vec<String>| if f.is_empty() { "none".to_string() } else { f.join(", ") };
                    details.push(format!("find number {} → {}", show(&x.find_numbers), show(&y.find_numbers)));
                }
                ("changed", details)
            }
            (None, None) => continue,
        };
        lines.push(DiffLine {
            part_id: key.clone(),
            number,
            name,
            change,
            from: before.cloned(),
            to: after.cloned(),
            details,
        });
    }
    lines.sort_by(|x, y| x.number.to_ascii_lowercase().cmp(&y.number.to_ascii_lowercase()));
    BomDiff {
        part_id: part.id.clone(),
        number: part.number.clone(),
        from_revision: from.label.clone(),
        to_revision: to.label.clone(),
        lines,
        unchanged,
    }
}

/// A quantity as a person writes it: `4`, not `4.0`.
pub fn qty(value: f64) -> String {
    if value.fract() == 0.0 && value.abs() < 1e15 {
        format!("{}", value as i64)
    } else {
        format!("{value}")
    }
}

// ===========================================================================
// Where-used
// ===========================================================================

#[derive(Debug, Clone, Serialize)]
pub struct WhereUsedLine {
    /// 1 for a direct parent, 2 for its parents, and so on.
    pub level: usize,
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub revision_label: String,
    pub state: String,
    /// The parent revision is the part's current release, or in work — the
    /// ones a replacement would still have to touch. A superseded or obsolete
    /// parent is history.
    pub current: bool,
    /// The child revision this use points at: its label, or
    /// `floating → <label>` for a floating use.
    pub uses_revision: String,
    pub floating: bool,
    pub quantity: f64,
    pub unit: String,
    pub find_number: String,
    /// Nothing uses this parent revision in turn: a top-level assembly.
    pub top: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct WhereUsed {
    pub part_id: String,
    pub number: String,
    /// The revision asked about, or empty for any revision.
    pub revision_label: String,
    /// 0 is every level up.
    pub levels: usize,
    pub lines: Vec<WhereUsedLine>,
}

/// Every uses-list line in the store, by the part it names: built once per
/// where-used question, in one pass, so climbing a level looks up its
/// parents instead of scanning every part again. In store order, so the
/// answer's order is the one a scan gives.
type UsedBy<'a> = HashMap<&'a str, Vec<(&'a Part, &'a Revision, &'a Use)>>;

fn used_by(state: &State) -> UsedBy<'_> {
    let mut index: UsedBy<'_> = HashMap::new();
    for parent in &state.parts {
        for rev in &parent.revisions {
            for line in &rev.uses {
                index.entry(line.part.as_str()).or_default().push((parent, rev, line));
            }
        }
    }
    index
}

/// Every revision that uses `part` — `revision` of it when one is given, a
/// floating use counting when it resolves to that revision now.
fn parents<'a>(
    index: &UsedBy<'a>,
    part: &'a Part,
    revision: Option<&'a Revision>,
) -> Vec<(&'a Part, &'a Revision, &'a Use, Option<&'a Revision>)> {
    let Some(lines) = index.get(part.id.as_str()) else { return Vec::new() };
    lines
        .iter()
        .filter_map(|&(parent, rev, line)| {
            let resolved = resolve(part, line);
            let hit = match revision {
                None => true,
                Some(wanted) => resolved.is_some_and(|r| r.id == wanted.id),
            };
            hit.then_some((parent, rev, line, resolved))
        })
        .collect()
}

/// Where `part` (or one revision of it) is used, up to `levels` levels (0 =
/// all the way to the top).
pub fn where_used(state: &State, part: &Part, revision: Option<&Revision>, levels: usize) -> WhereUsed {
    let index = used_by(state);
    let mut lines = Vec::new();
    let mut path = vec![part.id.clone()];
    climb(&index, part, revision, 1, levels, &mut path, &mut lines);
    WhereUsed {
        part_id: part.id.clone(),
        number: part.number.clone(),
        revision_label: revision.map(|r| r.label.clone()).unwrap_or_default(),
        levels,
        lines,
    }
}

fn climb<'a>(
    index: &UsedBy<'a>,
    part: &'a Part,
    revision: Option<&'a Revision>,
    level: usize,
    levels: usize,
    path: &mut Vec<String>,
    out: &mut Vec<WhereUsedLine>,
) {
    for (parent, rev, line, resolved) in parents(index, part, revision) {
        let top = parents(index, parent, Some(rev)).is_empty();
        let label = resolved.map(|r| r.label.clone()).unwrap_or_else(|| "(deleted)".into());
        out.push(WhereUsedLine {
            level,
            part_id: parent.id.clone(),
            number: parent.number.clone(),
            name: parent.name.clone(),
            revision_id: rev.id.clone(),
            revision_label: rev.label.clone(),
            state: rev.lifecycle.as_str().to_string(),
            current: rev.lifecycle.is_editable() || rev.lifecycle == Lifecycle::Released,
            uses_revision: if line.revision.is_empty() { format!("floating → {label}") } else { label },
            floating: line.revision.is_empty(),
            quantity: line.quantity,
            unit: line.unit.clone(),
            find_number: line.find_number.clone(),
            top,
        });
        let deeper = levels == 0 || level < levels;
        if deeper && !top && level < MAX_DEPTH && !path.contains(&parent.id) {
            path.push(parent.id.clone());
            climb(index, parent, Some(rev), level + 1, levels, path, out);
            path.pop();
        }
    }
}

// ===========================================================================
// CSV
// ===========================================================================

fn cell(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_string()
    }
}

fn money(value: Option<f64>) -> String {
    value.map(|v| format!("{v:.4}").trim_end_matches('0').trim_end_matches('.').to_string()).unwrap_or_default()
}

/// The BOM as CSV, one row per line, the indented view with its level and
/// position columns, the flat view without them. Totals follow as rows of
/// their own.
pub fn bom_csv(bom: &Bom) -> String {
    bom_csv_with(bom, false)
}

/// [`bom_csv`], with an `Attachments` column when `attachments` is set.
pub fn bom_csv_with(bom: &Bom, attachments: bool) -> String {
    let mut header = Vec::new();
    if !bom.flat {
        header.extend(["Level", "Position"]);
    }
    header.extend([
        "Find", "Number", "Name", "Rev", "State", "Qty", "Unit", "Total qty", "Category", "Attributes", "MPN",
        "Manufacturer", "Supplier", "SPN", "Currency", "Unit price", "Extended", "Reference", "Warnings",
    ]);
    if attachments {
        header.push("Attachments");
    }
    let mut out = header.join(",");
    out.push('\n');
    for line in &bom.lines {
        let mut row: Vec<String> = Vec::new();
        if !bom.flat {
            row.push(line.level.to_string());
            row.push(line.position.clone());
        }
        let rev = if line.floating { format!("{} (floating)", line.revision_label) } else { line.revision_label.clone() };
        row.extend([
            line.find_number.clone(),
            line.number.clone(),
            line.name.clone(),
            rev,
            line.state.clone(),
            qty(line.quantity),
            line.unit.clone(),
            qty(line.total),
            line.category_path.clone(),
            line.attributes.join("; "),
            line.mpn.clone(),
            line.manufacturer.clone(),
            line.supplier.clone(),
            line.spn.clone(),
            line.currency.clone(),
            money(line.unit_price),
            if line.costed { money(line.extended) } else { String::new() },
            line.reference.clone(),
            line.warnings.join("; "),
        ]);
        if attachments {
            row.push(line.attachments.join("; "));
        }
        out.push_str(&row.iter().map(|c| cell(c)).collect::<Vec<_>>().join(","));
        out.push('\n');
    }
    for total in &bom.totals {
        out.push_str(&format!("Total {},{}\n", cell(&total.currency), money(Some(total.total))));
    }
    if bom.unpriced > 0 {
        out.push_str(&format!("Unpriced lines,{}\n", bom.unpriced));
    }
    out
}

