//! REPLACE COMPONENT: swap the part an ACOMP occurrence (or every occurrence
//! of a part) shows, keeping the occurrence — its id, pose, fixed flag and
//! attributes — and every mate that still has something to hold on to.
//!
//! # Why the mates can survive at all
//!
//! A mate references `ACOMP<n>:<part entity name>` (see the kernel's
//! `assembly::mapping` module doc). The replace keeps the ACOMP id and swaps
//! only `inputParams.partName`, so every reference re-resolves by NAME
//! against the new part on the next run. The names come from the part's own
//! feature ids (`Part_PZ`, `Hole1_S`, split pieces `_1`, `_2`), so a family
//! member (the family's document with other expression values) and a part
//! saved as a variation and then edited carry the same names for the same
//! faces.
//!
//! # Lineage: when a name match is trusted
//!
//! Two unrelated parts can both have an `Extrude1_top`, so a name match alone
//! proves nothing. The match is trusted automatically only when the two
//! parts share LINEAGE ([`lineage_keys`](crate::engine_state::replace_component::lineage_keys)): the same source, members of one
//! family (`familySource.family`), or one saved from the other
//! ([`DERIVED_FROM_KEY`](crate::engine_state::replace_component::DERIVED_FROM_KEY), the chain Save As writes). Even then, an entity
//! whose geometry class changed (a plane that became a cylinder) is not kept.
//!
//! Without shared lineage, every healthy mate on the replaced occurrences is
//! written DISABLED before the run, with a `replace` marker. Nothing resolves
//! against the new part and nothing moves until the user confirms. The
//! summary offers the same-name entity (pre-ticked when its geometry agrees
//! with the old frame) or the nearest entity of the same class. A suggestion
//! is never applied silently.
//!
//! # The marker and the report
//!
//! An unresolved mate is KEPT, disabled, with
//! `persistentData.replace = {from, to, wasEnabled, refs: [{ref, reason,
//! suggestion}]}` (it rides save/reopen). [`EngineState::replace_repair`]
//! points a reference at a picked entity and, once nothing is left
//! unresolved, re-enables the mate (if it was enabled before) and drops the
//! marker. Deleting goes through the ordinary constraint delete.
//!
//! # Part edits
//!
//! A PART EDIT (Update components, a Work part push) is the same check with
//! the part replaced by its own next version: [`EngineState::commit_part_edit`]
//! adopts the edited document and classifies the mates of every occurrence of
//! the edited parts once its run lands, with lineage trusted. One rule differs:
//! a reference whose name is gone (or now names another class of geometry) is
//! RE-ATTACHED automatically when exactly one entity of the edited part lies on
//! the same carrier ([`geometry_agrees`](crate::engine_state::replace_component::geometry_agrees)); the report lists each one, old ref →
//! new ref. Anything else is held with the same marker (`origin: "edit"`), so
//! the Repair pick and Accept work unchanged. A replace never auto-attaches.
//!
//! The replace itself is ONE undo step: one document edit (library block +
//! partName + disabled mates), adopted with one checkpoint. The
//! classification runs once that edit's run has landed
//! (`EngineState::finish_component_replace`, from `pump`), and its marker
//! write is silent (no second undo entry).

use super::*;
use serde_json::{json, Value};
use std::borrow::Borrow;
use std::collections::{BTreeMap, BTreeSet};

/// The top-level document key Save As appends the source's identity to.
pub const DERIVED_FROM_KEY: &str = "derivedFrom";

/// The persistentData key an unresolved mate carries.
pub const REPLACE_MARKER: &str = "replace";

/// Statuses that mean a mate was already broken before the replace (it stays
/// out of the summary and is left as it was).
const BROKEN: [&str; 3] = ["invalid-selection", "unsupported-selection", "error"];

/// `BROKEN`, for callers outside this module.
pub const BROKEN_STATUSES: [&str; 3] = BROKEN;

/// The lineage keys of a part: its own source identity, every identity it
/// was saved from (`derivedFrom`) and its family (`family:<file>`), each
/// passed through `canonicalize` (stores spell one file several ways). Two
/// parts share lineage when their key sets intersect.
pub fn lineage_keys(
    document: &Value,
    identity: &str,
    canonicalize: &dyn Fn(&str) -> String,
) -> BTreeSet<String> {
    let mut keys = BTreeSet::new();
    if !identity.trim().is_empty() {
        keys.insert(canonicalize(identity.trim()));
    }
    for entry in document[DERIVED_FROM_KEY].as_array().into_iter().flatten() {
        if let Some(entry) = entry.as_str().map(str::trim).filter(|e| !e.is_empty()) {
            // A family entry is stored already keyed (`family:<file>`).
            if entry.starts_with("family:") {
                keys.insert(entry.to_string());
            } else {
                keys.insert(canonicalize(entry));
            }
        }
    }
    if let Some(family) = family_key(document) {
        keys.insert(family);
    }
    keys
}

/// `family:<file>` for a family member (its `familySource.family`).
pub fn family_key(document: &Value) -> Option<String> {
    document["familySource"]["family"]
        .as_str()
        .map(str::trim)
        .filter(|family| !family.is_empty())
        .map(|family| format!("family:{family}"))
}

/// How two parts are related, or `None` when they share nothing.
pub fn shared_lineage(old: &BTreeSet<String>, new: &BTreeSet<String>) -> Option<&'static str> {
    let common: Vec<&String> = old.intersection(new).collect();
    if common.is_empty() {
        None
    } else if common.iter().any(|key| !key.starts_with("family:")) {
        // One saved from the other (or the same source): the closer relation.
        Some("derived")
    } else {
        Some("family")
    }
}

/// Append `source_identity` (and, for a family member, its family) to the
/// document's [`DERIVED_FROM_KEY`] chain — what Save As writes so a copy
/// edited later is still recognised as a variation of its original. Keeps
/// the existing chain, no duplicates.
pub fn stamp_derived_from(document: &mut Value, source_identity: &str) {
    let Some(object) = document.as_object_mut() else { return };
    let family = object
        .get("familySource")
        .and_then(|source| source.get("family"))
        .and_then(Value::as_str)
        .map(str::trim)
        .filter(|family| !family.is_empty())
        .map(|family| format!("family:{family}"));
    let chain = object
        .entry(DERIVED_FROM_KEY)
        .or_insert_with(|| Value::Array(Vec::new()));
    if !chain.is_array() {
        *chain = Value::Array(Vec::new());
    }
    let list = chain.as_array_mut().expect("array");
    for entry in [Some(source_identity.trim().to_string()), family].into_iter().flatten() {
        if !entry.is_empty() && !list.iter().any(|e| e.as_str() == Some(&entry)) {
            list.push(Value::String(entry));
        }
    }
}

// ---------------------------------------------------------------------------
// Geometry comparison (rows from the kernel's assembly_resolve_refs_json /
// assembly_component_entities_json: {geom, point, direction, radius}, all in
// the component's local frame)
// ---------------------------------------------------------------------------

fn triple(value: &Value) -> Option<[f64; 3]> {
    let list = value.as_array()?;
    if list.len() != 3 {
        return None;
    }
    Some([list[0].as_f64()?, list[1].as_f64()?, list[2].as_f64()?])
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// Distance from `p` to the line through `origin` along unit `direction`.
fn line_distance(p: [f64; 3], origin: [f64; 3], direction: [f64; 3]) -> f64 {
    let d = sub(p, origin);
    let along = dot(d, direction);
    (dot(d, d) - along * along).max(0.0).sqrt()
}

/// Relative length tolerance for "the same carrier / the same point": the
/// kernel's own vertex-resolution tolerance (`VERTEX_SNAP_TOLERANCE`, 1e-6 of
/// the solid's scale), so a corner rebuilt through another boolean path,
/// a few ulps off, still counts as the same point.
const POSITION_TOLERANCE: f64 = 1e-6;
const ANGLE_TOLERANCE: f64 = 1e-9;

/// Whether `new` is the same analytic carrier as `old`: the same class, and
/// the same plane / axis / circle / line / point / sphere within tolerance
/// (a plane must keep its sense: a flipped face is a different face). This is
/// what a mate actually resolves against, so a split face's piece agrees.
pub fn geometry_agrees(old: &Value, new: &Value) -> bool {
    if old["geom"] != new["geom"] {
        return false;
    }
    let (Some(p_old), Some(p_new)) = (triple(&old["point"]), triple(&new["point"])) else {
        return false;
    };
    let directions = (triple(&old["direction"]), triple(&new["direction"]));
    let radius_agrees = match (old["radius"].as_f64(), new["radius"].as_f64()) {
        (Some(a), Some(b)) => (a - b).abs() <= POSITION_TOLERANCE * (1.0 + a.abs()),
        (None, None) => true,
        _ => false,
    };
    let scale = POSITION_TOLERANCE * (1.0 + norm(p_old));
    match old["geom"].as_str().unwrap_or("") {
        "plane" => match directions {
            (Some(n_old), Some(n_new)) => {
                dot(n_old, n_new) >= 1.0 - ANGLE_TOLERANCE
                    && dot(sub(p_old, p_new), n_new).abs() <= scale
            }
            _ => false,
        },
        "axis" | "line" => match directions {
            (Some(d_old), Some(d_new)) => {
                dot(d_old, d_new).abs() >= 1.0 - ANGLE_TOLERANCE
                    && line_distance(p_old, p_new, d_new) <= scale
                    && radius_agrees
            }
            _ => false,
        },
        "circle" => match directions {
            (Some(d_old), Some(d_new)) => {
                dot(d_old, d_new).abs() >= 1.0 - ANGLE_TOLERANCE
                    && norm(sub(p_old, p_new)) <= scale
                    && radius_agrees
            }
            _ => false,
        },
        "sphere" | "point" => norm(sub(p_old, p_new)) <= scale && radius_agrees,
        _ => false,
    }
}

/// How far `new` sits from `old` for ranking suggestions of one class: the
/// anchor distance plus a direction penalty (a unit of misalignment weighs
/// like 1000 length units, so a parallel face always beats a tilted one).
fn suggestion_score(old: &Value, new: &Value) -> f64 {
    let distance = match (triple(&old["point"]), triple(&new["point"])) {
        (Some(a), Some(b)) => norm(sub(a, b)),
        _ => f64::INFINITY,
    };
    let misalignment = match (triple(&old["direction"]), triple(&new["direction"])) {
        (Some(a), Some(b)) => {
            if old["geom"] == "plane" {
                1.0 - dot(a, b)
            } else {
                1.0 - dot(a, b).abs()
            }
        }
        _ => 0.0,
    };
    distance + 1000.0 * misalignment
}

/// The part-local name of a namespaced ref (`ACOMP2:Part_PZ` → `Part_PZ`).
fn local_name<'a>(component: &str, reference: &'a str) -> &'a str {
    reference
        .strip_prefix(component)
        .and_then(|rest| rest.strip_prefix(':'))
        .unwrap_or(reference)
}

/// What the replace concluded about one reference.
#[derive(Debug, Clone, PartialEq)]
pub enum RefOutcome {
    /// Shared lineage, the name exists, the class is unchanged.
    Kept { split: bool },
    /// Needs the user: `reason` is `missing` (no entity of that name),
    /// `type-changed` (the name now names another class of geometry) or
    /// `unconfirmed` (no shared lineage). `suggestion` is a namespaced ref.
    Unresolved {
        reason: &'static str,
        suggestion: Option<Suggestion>,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Suggestion {
    pub reference: String,
    /// `same-name` or `nearest`.
    pub basis: &'static str,
    /// The geometry agrees with the old frame: shown ticked, still applied
    /// only on the user's confirmation.
    pub pre_ticked: bool,
}

impl Suggestion {
    fn to_json(&self) -> Value {
        json!({ "ref": self.reference, "basis": self.basis, "preTicked": self.pre_ticked })
    }
}

/// Classify one reference of a replaced component. `pre` is the reference's
/// resolution BEFORE the replace (the kernel row, `ok: true`); `candidates`
/// the new part's entities; `old_names` the old part's local entity names
/// (for the split note); `trusted` = shared lineage.
pub fn classify_ref<V: std::borrow::Borrow<Value>>(
    component: &str,
    reference: &str,
    pre: &Value,
    candidates: &[V],
    old_names: &BTreeSet<String>,
    trusted: bool,
) -> RefOutcome {
    let name = local_name(component, reference);
    let by_name = candidates
        .iter()
        .map(Borrow::borrow)
        .find(|candidate| candidate["ref"].as_str().map(|r| local_name(component, r)) == Some(name));
    if trusted {
        match by_name {
            Some(candidate) if candidate["geom"] == pre["geom"] => {
                let split_piece = format!("{name}_1");
                let split = !old_names.contains(&split_piece)
                    && candidates.iter().map(Borrow::borrow).any(|c: &Value| {
                        c["ref"].as_str().map(|r| local_name(component, r)) == Some(split_piece.as_str())
                    });
                return RefOutcome::Kept { split };
            }
            Some(_) => {
                return RefOutcome::Unresolved {
                    reason: "type-changed",
                    suggestion: nearest(pre, candidates),
                }
            }
            None => {
                return RefOutcome::Unresolved { reason: "missing", suggestion: nearest(pre, candidates) }
            }
        }
    }
    let suggestion = match by_name {
        Some(candidate) if candidate["geom"] == pre["geom"] => Some(Suggestion {
            reference: candidate["ref"].as_str().unwrap_or_default().to_string(),
            basis: "same-name",
            pre_ticked: geometry_agrees(pre, candidate),
        }),
        _ => nearest(pre, candidates),
    };
    RefOutcome::Unresolved { reason: "unconfirmed", suggestion }
}

/// Classify a VERTEX reference (`{solid}@x,y,z`, a position in the
/// component's frame): kept only when, with shared lineage, the new part has
/// a vertex at exactly the old point (the resolver snaps within a tolerance,
/// so the point is compared, never trusted). Otherwise unresolved; the same
/// ref is offered when it resolves, pre-ticked when it is the same point.
pub fn classify_vertex_ref(reference: &str, pre: &Value, post: Option<&Value>, trusted: bool) -> RefOutcome {
    let found = post.filter(|row| row["ok"] == true);
    let same_point = found.is_some_and(|row| geometry_agrees(pre, row));
    if trusted && same_point {
        return RefOutcome::Kept { split: false };
    }
    let suggestion = found.map(|_| Suggestion {
        reference: reference.to_string(),
        basis: "same-name",
        pre_ticked: same_point,
    });
    let reason = match (trusted, found) {
        (false, _) => "unconfirmed",
        (true, Some(_)) => "moved",
        (true, None) => "missing",
    };
    RefOutcome::Unresolved { reason, suggestion }
}

/// The nearest entity of the same geometry class, pre-ticked only when it
/// IS the old carrier (same plane / axis / …).
fn nearest<V: std::borrow::Borrow<Value>>(pre: &Value, candidates: &[V]) -> Option<Suggestion> {
    candidates
        .iter()
        .map(Borrow::borrow)
        .filter(|candidate| candidate["geom"] == pre["geom"])
        .map(|candidate| (suggestion_score(pre, candidate), candidate))
        .filter(|(score, _)| score.is_finite())
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, candidate)| Suggestion {
            reference: candidate["ref"].as_str().unwrap_or_default().to_string(),
            basis: "nearest",
            pre_ticked: geometry_agrees(pre, candidate),
        })
}

/// The marker an unresolved mate carries (`persistentData.replace`). `origin`
/// is `replace` or `edit`; a marker saved before origins existed has none and
/// reads as `replace`.
pub fn marker_json(origin: &str, from: &str, to: &str, was_enabled: bool, refs: Vec<Value>) -> Value {
    json!({
        "origin": origin,
        "from": from,
        "to": to,
        "wasEnabled": was_enabled,
        "refs": refs,
    })
}

/// The one entity of `candidates` on the same carrier as `pre`
/// ([`geometry_agrees`]), or `None` when there is none or more than one: a
/// part edit re-attaches only what is unambiguous.
fn unique_carrier(pre: &Value, candidates: &[&Value]) -> Option<String> {
    let mut agreeing = candidates.iter().filter(|candidate| geometry_agrees(pre, candidate));
    let only = agreeing.next()?;
    if agreeing.next().is_some() {
        return None;
    }
    only["ref"].as_str().map(str::to_string)
}

// ---------------------------------------------------------------------------
// The engine flow
// ---------------------------------------------------------------------------

/// One mate touching a replaced component, as it stood before the swap.
#[derive(Debug, Clone)]
pub(crate) struct AffectedMate {
    id: String,
    constraint_type: String,
    was_enabled: bool,
    /// (component id, ref, pre-replace kernel row).
    refs: Vec<(String, String, Value)>,
    /// For a sub-assembly's OWN mate (a part edit inside it): the occurrence
    /// path of that sub-assembly and the mate's id in its document. `id` and
    /// the refs are then namespaced under that path, as the session shows them.
    home: Option<(String, String)>,
}

/// A replace whose run has not landed yet.
#[derive(Debug, Clone)]
pub(crate) struct PendingReplace {
    generation: u64,
    components: Vec<String>,
    from: BTreeMap<String, String>,
    to: String,
    /// Component id → lineage word (`family` / `derived`), absent = none.
    lineage: BTreeMap<String, &'static str>,
    affected: Vec<AffectedMate>,
    /// Component id → the OLD part's local entity names.
    old_names: BTreeMap<String, BTreeSet<String>>,
    /// Lines the caller adds to the report (a nested replace's held mates).
    notes: Vec<String>,
    /// `replace`, or `edit` for a part edit ([`EngineState::commit_part_edit`]).
    origin: &'static str,
    /// A part edit's parts → the `sourceSignature` each must still carry when
    /// the run lands (an undo before it lands restores the old one).
    signatures: BTreeMap<String, String>,
}


/// What [`EngineState::capture_replace`] read before the swap.
#[derive(Debug, Clone)]
pub struct ReplaceCapture {
    components: Vec<String>,
    affected: Vec<AffectedMate>,
    old_names: BTreeMap<String, BTreeSet<String>>,
}

/// The lineage word of two parts-library entries (`{sourceKey, document}`),
/// or `None` when they share nothing.
pub fn entries_lineage(old: &Value, new: &Value, canonicalize: &dyn Fn(&str) -> String) -> Option<&'static str> {
    let old_keys = lineage_keys(&old["document"], old["sourceKey"].as_str().unwrap_or(""), canonicalize);
    let new_keys = lineage_keys(&new["document"], new["sourceKey"].as_str().unwrap_or(""), canonicalize);
    shared_lineage(&old_keys, &new_keys)
}



/// [`entities_of`] for many paths under the same top components: each top
/// component's listing is asked for once.
#[derive(Default)]
struct EntityListings {
    by_top: BTreeMap<String, Vec<Value>>,
    /// Each top component's local names by occurrence path under it (every
    /// prefix of a row's ref), indexed once: a capture asks for the names of
    /// every nested owner, and filtering the whole listing per owner cost
    /// ~7 ms per module on l500.
    names_by_owner: BTreeMap<String, BTreeMap<String, BTreeSet<String>>>,
    /// Each top listing's rows by occurrence path under it, as indices in
    /// listing order: the top holds every row, each deeper component prefix
    /// the rows whose ref starts with it. A part edit's check asks for every
    /// nested owner (555 on l500), and filtering and copying the whole
    /// listing per owner was ~65 ms of the frame its run landed in.
    rows_by_owner: BTreeMap<String, BTreeMap<String, Vec<usize>>>,
}

/// Every occurrence path a listed ref sits under, below `top`: `TOP:A`,
/// `TOP:A:B`, … (the segments before its name).
fn owners_below<'a>(top: &'a str, reference: &'a str) -> impl Iterator<Item = String> + 'a {
    let rest = reference.strip_prefix(top).and_then(|r| r.strip_prefix(':')).unwrap_or("");
    rest.match_indices(':').map(move |(at, _)| format!("{top}:{}", &rest[..at]))
}

impl EntityListings {
    /// The top component's listing, asked for once.
    fn listed(&mut self, top: &str) -> &[Value] {
        self.by_top.entry(top.to_string()).or_insert_with(|| {
            entities_of(top)
        })
    }

    /// Index the listing `path` sits in by owner, once per top.
    fn index(&mut self, path: &str) {
        let top = path.split(':').next().unwrap_or(path).to_string();
        if self.rows_by_owner.contains_key(&top) {
            return;
        }
        let rows = self.listed(&top);
        let mut index: BTreeMap<String, Vec<usize>> = BTreeMap::new();
        index.insert(top.clone(), (0..rows.len()).collect());
        for (at, row) in rows.iter().enumerate() {
            let Some(reference) = row["ref"].as_str() else { continue };
            for owner in owners_below(&top, reference) {
                index.entry(owner).or_default().push(at);
            }
        }
        self.rows_by_owner.insert(top, index);
    }

    /// The entities under an occurrence path, in listing order: the top
    /// component's whole listing, or the rows whose ref starts with `path:`.
    /// [`Self::index`] it first.
    fn under(&self, path: &str) -> Vec<&Value> {
        let top = path.split(':').next().unwrap_or(path);
        let (Some(rows), Some(index)) = (self.by_top.get(top), self.rows_by_owner.get(top)) else {
            return Vec::new();
        };
        index.get(path).into_iter().flatten().map(|&at| &rows[at]).collect()
    }

    /// The local names (`local_name(path, ref)`) of every entity under an
    /// occurrence path: what `of(path)` lists, as names, from the index.
    fn names(&mut self, path: &str) -> BTreeSet<String> {
        let top = path.split(':').next().unwrap_or(path).to_string();
        if !self.names_by_owner.contains_key(&top) {
            self.listed(&top);
            let mut index: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
            for row in self.by_top.get(&top).into_iter().flatten() {
                let Some(reference) = row["ref"].as_str() else { continue };
                let Some(rest) = reference.strip_prefix(&top).and_then(|r| r.strip_prefix(':')) else {
                    index.entry(top.clone()).or_default().insert(reference.to_string());
                    continue;
                };
                // Every owner the ref sits under: the top, then each deeper
                // component prefix (`TOP:A`, `TOP:A:B`, … before the name).
                let mut owner = top.clone();
                index.entry(owner.clone()).or_default().insert(rest.to_string());
                let mut tail = rest;
                while let Some((segment, after)) = tail.split_once(':') {
                    owner = format!("{owner}:{segment}");
                    index.entry(owner.clone()).or_default().insert(after.to_string());
                    tail = after;
                }
            }
            self.names_by_owner.insert(top.clone(), index);
        }
        self.names_by_owner[&top].get(path).cloned().unwrap_or_default()
    }
}

/// The named entities of an occurrence path: the top component's listing,
/// narrowed to the nested occurrence for an `ACOMP3:ACOMP1` path.
fn entities_of(path: &str) -> Vec<Value> {
    let top = path.split(':').next().unwrap_or(path);
    let prefix = format!("{path}:");
    parse(&brep_kernel::assembly_component_entities_json(top))
        .as_array()
        .cloned()
        .unwrap_or_default()
        .into_iter()
        .filter(|row| top == path || row["ref"].as_str().is_some_and(|r| r.starts_with(&prefix)))
        .collect()
}

/// The document at the end of a parts-library chain (outermost part first)
/// inside `document`, for writing.
fn document_in_chain_mut<'a>(document: &'a mut Value, chain: &[String]) -> Option<&'a mut Value> {
    let mut current = document;
    for part in chain {
        current = current.get_mut("partsLibrary")?.get_mut(part.as_str())?.get_mut("document")?;
    }
    Some(current)
}

/// The component a ref belongs to among `ids` (bare id or `id:` prefix).
fn owner_among<'a>(ids: &'a [String], reference: &str) -> Option<&'a String> {
    ids.iter().find(|id| {
        reference == id.as_str()
            || reference
                .strip_prefix(id.as_str())
                .is_some_and(|rest| rest.starts_with(':'))
    })
}

/// The report's notes for the Pattern Components seeded by a replaced
/// component in `document`: one per pattern, how many of its instances
/// follow. Nothing about them is edited: the instances re-emit from the
/// seed's new part in the same run, and a seed given as a face of the
/// component (`ACOMP2:Pin_S`, a pick) resolves by its component prefix, so it
/// survives a part without that face. `instances` is every Pattern Component
/// instance the last run placed.
pub fn follow_pattern_seeds(
    document: &Value,
    from: &BTreeMap<String, String>,
    to: &str,
    instances: &[String],
) -> Vec<String> {
    let mut notes = Vec::new();
    for feature in document["features"].as_array().into_iter().flatten() {
        if feature["type"] != "APAT" {
            continue;
        }
        let pattern = feature["inputParams"]["id"].as_str().unwrap_or_default().to_string();
        let mut seeded = BTreeSet::new();
        for name in feature["inputParams"]["seeds"].as_array().into_iter().flatten().filter_map(Value::as_str) {
            let owner = name.split(':').next().unwrap_or(name).to_string();
            if from.contains_key(&owner) {
                seeded.insert(owner);
            }
        }
        for owner in seeded {
            let count = instances
                .iter()
                .filter_map(|id| brep_kernel::pattern_component_instance(id))
                .filter(|(p, s, _)| *p == pattern && *s == owner)
                .count();
            notes.push(format!("{pattern}: its {count} instance(s) of {owner} follow the seed and now show '{to}'"));
        }
    }
    notes
}

fn parse(json: &str) -> Value {
    serde_json::from_str(json).unwrap_or(Value::Null)
}

impl EngineState {
    /// Replace the part shown by the ACOMP occurrences `component_ids` with
    /// `insert` (an existing parts-library entry or a new part payload).
    /// `canonicalize` spells a store identity one way (lineage comparison).
    /// One undo step. Returns the new part's library entry name; the
    /// classification of the mates lands with the run
    /// ([`Self::replace_report`]).
    pub fn replace_components(
        &mut self,
        component_ids: &[String],
        insert: ComponentInsert<'_>,
        canonicalize: &dyn Fn(&str) -> String,
    ) -> Result<String, String> {
        if component_ids.is_empty() {
            return Err("replace: no component given".into());
        }
        // A Pattern Component instance has no feature of its own: it shows
        // whatever its seed shows, so replacing it replaces the seed (and
        // with it every instance), and the report says so.
        let mut ids: Vec<String> = Vec::new();
        let mut instance_notes = Vec::new();
        for id in component_ids {
            let id = match brep_kernel::pattern_component_instance(id) {
                Some((pattern, seed, n)) => {
                    instance_notes.push(format!(
                        "{id} is instance {n} of {pattern} and follows its seed {seed}: the seed was replaced"
                    ));
                    seed
                }
                None => id.clone(),
            };
            if !ids.contains(&id) {
                ids.push(id);
            }
        }
        let component_ids = ids.as_slice();
        let mut from = BTreeMap::new();
        for id in component_ids {
            let info = self
                .component_info(id)
                .ok_or_else(|| format!("replace: '{id}' is not an assembly component"))?;
            from.insert(id.clone(), info.part_name);
        }
        let capture = self.capture_replace(component_ids);

        // The new part's library entry (added or reused), and lineage.
        let old_library = self.history.parts_library().clone();
        let to = self.resolve_component_insert(insert)?;
        let library = parse(&brep_kernel::parts_library_json());
        let new_entry = &library[&to];
        let mut lineage = BTreeMap::new();
        for (id, part) in &from {
            if part == &to {
                continue;
            }
            let old_entry = if old_library[part].is_object() { &old_library[part] } else { &library[part] };
            if let Some(word) = entries_lineage(old_entry, new_entry, canonicalize) {
                lineage.insert(id.clone(), word);
            }
        }
        if from.values().all(|part| part == &to) {
            return Err(format!("replace: the component already shows '{to}'"));
        }

        // ONE document edit: the library and the partNames (the commit adds
        // the held mates). A Pattern Component seeded by a replaced
        // component follows by itself (see [`follow_pattern_seeds`]).
        let mut document = parse(&self.history_request_json());
        document["partsLibrary"] = library.clone();
        for feature in document["features"].as_array_mut().into_iter().flatten() {
            let id = feature["inputParams"]["id"].as_str().unwrap_or_default().to_string();
            if from.contains_key(&id) {
                feature["inputParams"]["partName"] = Value::String(to.clone());
            }
        }
        let mut notes = instance_notes;
        notes.extend(follow_pattern_seeds(&document, &from, &to, &self.pattern_instance_ids()));
        self.commit_replace(capture, document, from, to.clone(), lineage, notes)?;
        Ok(to)
    }

    /// The live state a replace of `paths` must compare against, read BEFORE
    /// anything moves: every healthy mate touching them with each
    /// reference's local frame, and the old parts' entity names. A path is an
    /// ACOMP id, or a NESTED occurrence path (`ACOMP3:ACOMP1`) for a part
    /// inside a sub-assembly (frames are then in the top occurrence's frame,
    /// which the replace does not move).
    pub fn capture_replace(&mut self, paths: &[String]) -> ReplaceCapture {
        self.capture_replace_listed(paths, &mut EntityListings::default())
    }

    /// [`Self::capture_replace`] reading the components' names through
    /// `listings`, which the caller can go on sharing (a part edit's nested
    /// owners sit under these same top components).
    fn capture_replace_listed(&mut self, paths: &[String], listings: &mut EntityListings) -> ReplaceCapture {
        self.ensure_assembly_session();
        let state = parse(&brep_kernel::assembly_state_json());
        let mut affected = Vec::new();
        for entry in state["constraints"].as_array().into_iter().flatten() {
            let refs: Vec<(String, String)> = entry["inputParams"]["elements"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(Value::as_str)
                .filter_map(|r| owner_among(paths, r).map(|owner| (owner.clone(), r.to_string())))
                .collect();
            if refs.is_empty() {
                continue;
            }
            let enabled = entry["enabled"].as_bool().unwrap_or(true);
            let status = entry["persistentData"]["status"].as_str().unwrap_or("");
            if enabled && BROKEN.contains(&status) {
                continue; // already broken: left exactly as it was
            }
            let names: Vec<&String> = refs.iter().map(|(_, r)| r).collect();
            let rows = parse(&brep_kernel::assembly_resolve_refs_json(
                &serde_json::to_string(&names).unwrap_or_default(),
            ));
            let rows = rows.as_array().cloned().unwrap_or_default();
            affected.push(AffectedMate {
                id: entry["inputParams"]["id"].as_str().unwrap_or_default().to_string(),
                constraint_type: entry["type"].as_str().unwrap_or_default().to_string(),
                was_enabled: enabled,
                refs: refs
                    .into_iter()
                    .zip(rows.into_iter().chain(std::iter::repeat(Value::Null)))
                    .map(|((owner, r), row)| (owner, r, row))
                    .collect(),
                home: None,
            });
        }
        let mut old_names = BTreeMap::new();
        for path in paths {
            old_names.insert(path.clone(), listings.names(path));
        }
        ReplaceCapture { components: paths.to_vec(), affected, old_names }
    }

    /// [`Self::capture_replace`] for a PART EDIT: also the sub-assemblies' OWN
    /// mates below `paths`, at every depth. Those live in the sub-assembly's
    /// document, not this one, and a renamed face breaks them just the same;
    /// [`Self::commit_part_edit`] classifies them with the rest and writes the
    /// outcome back into that document. (A replace takes the old sub-assembly's
    /// mates away with it, so it captures none.)
    pub fn capture_part_edit(&mut self, paths: &[String]) -> ReplaceCapture {
        // One listing per top component, shared by the components' own names
        // and every nested owner under them (a listing resolves every name
        // the component has).
        let mut listings = EntityListings::default();
        let mut capture = self.capture_replace_listed(paths, &mut listings);
        // One sub-assembly definition placed twice holds one set of mates,
        // and the same definitions below it: its subtree is walked once.
        let mut seen: BTreeSet<Vec<String>> = BTreeSet::new();
        let mut stack: Vec<String> = paths.to_vec();
        while let Some(path) = stack.pop() {
            let Some(chain) = self.part_chain(&path) else { continue };
            if !seen.insert(chain.clone()) {
                continue;
            }
            let Some(document) = self.document_in_chain(&chain) else { continue };
            for feature in document["features"].as_array().into_iter().flatten() {
                if super::components::is_acomp_feature_type(feature["type"].as_str().unwrap_or("")) {
                    if let Some(id) = feature["inputParams"]["id"].as_str() {
                        stack.push(format!("{path}:{id}"));
                    }
                }
            }
            for entry in document["assembly"]["constraints"].as_array().into_iter().flatten() {
                let inner_id = entry["inputParams"]["id"].as_str().unwrap_or_default();
                let refs: Vec<(String, String)> = entry["inputParams"]["elements"]
                    .as_array()
                    .into_iter()
                    .flatten()
                    .filter_map(Value::as_str)
                    .map(|r| {
                        let occurrence = r.split(':').next().unwrap_or(r);
                        (format!("{path}:{occurrence}"), format!("{path}:{r}"))
                    })
                    .collect();
                if inner_id.is_empty() || refs.is_empty() {
                    continue;
                }
                let enabled = entry["enabled"].as_bool().unwrap_or(true);
                let status = entry["persistentData"]["status"].as_str().unwrap_or("");
                if enabled && BROKEN.contains(&status) {
                    continue; // already broken: left exactly as it was
                }
                let names: Vec<&String> = refs.iter().map(|(_, r)| r).collect();
                let rows = parse(&brep_kernel::assembly_resolve_refs_json(
                    &serde_json::to_string(&names).unwrap_or_default(),
                ));
                let rows = rows.as_array().cloned().unwrap_or_default();
                // A ref this session cannot resolve now cannot be judged after
                // the edit either: the mate is left exactly as it is, never
                // held on a guess.
                if rows.len() != refs.len() || rows.iter().any(|row| row["ok"] != true) {
                    continue;
                }
                for (owner, _) in &refs {
                    if !capture.old_names.contains_key(owner) {
                        let names = listings.names(owner);
                        capture.old_names.insert(owner.clone(), names);
                    }
                }
                capture.affected.push(AffectedMate {
                    id: format!("{path}:{inner_id}"),
                    constraint_type: entry["type"].as_str().unwrap_or_default().to_string(),
                    was_enabled: enabled,
                    refs: refs
                        .into_iter()
                        .zip(rows.into_iter().chain(std::iter::repeat(Value::Null)))
                        .map(|((owner, r), row)| (owner, r, row))
                        .collect(),
                    home: Some((path.clone(), inner_id.to_string())),
                });
            }
        }
        capture
    }

    /// Write a part edit's outcome for the sub-assemblies' OWN mates into
    /// their documents, in this document's parts library: a re-attached ref is
    /// rewritten, a held mate is disabled and marked, with every ref in the
    /// sub-assembly's own frame (Repair runs there). Each rewritten entry, and
    /// every entry above it, is re-signed and loses its snapshot, as the edit
    /// that caused it did. No checkpoint: it rides the edit's undo step.
    fn write_back_inner_mates(
        &mut self,
        affected: &[AffectedMate],
        reattached: &[(String, String, String)],
        markers: &BTreeMap<String, Value>,
    ) {
        let touched: Vec<&AffectedMate> = affected
            .iter()
            .filter(|mate| mate.home.is_some())
            .filter(|mate| markers.contains_key(&mate.id) || reattached.iter().any(|(id, _, _)| *id == mate.id))
            .collect();
        if touched.is_empty() {
            return;
        }
        let Ok(mut document) = serde_json::from_str::<Value>(&self.history.request_json()) else { return };
        let mut chains: BTreeSet<Vec<String>> = BTreeSet::new();
        for mate in touched {
            let (home, inner_id) = mate.home.as_ref().expect("filtered");
            let Some(chain) = self.part_chain(home) else { continue };
            let Some(inner) = document_in_chain_mut(&mut document, &chain) else { continue };
            let prefix = format!("{home}:");
            let local = |reference: &str| reference.strip_prefix(&prefix).unwrap_or(reference).to_string();
            for entry in inner["assembly"]["constraints"].as_array_mut().into_iter().flatten() {
                if entry["inputParams"]["id"].as_str() != Some(inner_id.as_str()) {
                    continue;
                }
                for (_, old_ref, new_ref) in reattached.iter().filter(|(id, _, _)| *id == mate.id) {
                    for element in entry["inputParams"]["elements"].as_array_mut().into_iter().flatten() {
                        if element == local(old_ref).as_str() {
                            *element = Value::String(local(new_ref));
                        }
                    }
                }
                if let Some(marker) = markers.get(&mate.id) {
                    let mut marker = marker.clone();
                    for row in marker["refs"].as_array_mut().into_iter().flatten() {
                        if let Some(reference) = row["ref"].as_str().map(|r| local(r)) {
                            row["ref"] = Value::String(reference);
                        }
                        if let Some(reference) = row["suggestion"]["ref"].as_str().map(|r| local(r)) {
                            row["suggestion"]["ref"] = Value::String(reference);
                        }
                    }
                    entry["enabled"] = Value::Bool(false);
                    if !entry["persistentData"].is_object() {
                        entry["persistentData"] = json!({});
                    }
                    entry["persistentData"][REPLACE_MARKER] = marker;
                }
            }
            chains.insert(chain);
        }
        // Deepest first, so each signature covers the rewrites below it.
        let mut prefixes: BTreeSet<Vec<String>> = BTreeSet::new();
        for chain in &chains {
            for depth in 1..=chain.len() {
                prefixes.insert(chain[..depth].to_vec());
            }
        }
        let mut prefixes: Vec<Vec<String>> = prefixes.into_iter().collect();
        prefixes.sort_by_key(|chain| std::cmp::Reverse(chain.len()));
        for chain in prefixes {
            let (last, parent) = chain.split_last().expect("non-empty");
            let library = if parent.is_empty() {
                Some(&mut document["partsLibrary"])
            } else {
                document_in_chain_mut(&mut document, parent).map(|doc| &mut doc["partsLibrary"])
            };
            let Some(entry) = library.and_then(|library| library.get_mut(last.as_str())) else { continue };
            let signature = super::assembly_ops::document_signature(&entry["document"].to_string());
            entry["sourceSignature"] = Value::String(signature);
            entry["snapshot"] = Value::String(String::new());
            if let Some(object) = entry.as_object_mut() {
                object.remove("nestedRows");
            }
        }
        // As `edit_document_json`: the run reads the kernel's resident store,
        // not the document's block.
        let written = serde_json::from_value::<brep_kernel::PartsLibraryMap>(document["partsLibrary"].clone())
            .map_err(|error| error.to_string())
            .and_then(|library| self.history.adopt_document(&document.to_string()).map(|()| library));
        match written {
            Ok(library) => {
                brep_kernel::install_parts_library(&library);
            }
            Err(error) => {
                self.push_notice(format!("part edit: the sub-assembly mates could not be written: {error}"));
            }
        }
    }

    /// The document of the part an occurrence path places (see
    /// [`Self::part_at_path`]).
    fn document_in_chain(&self, chain: &[String]) -> Option<&Value> {
        let mut library = self.history.parts_library();
        let mut document = None;
        for part in chain {
            let entry = library.get(part.as_str())?;
            document = Some(&entry["document"]);
            library = &entry["document"]["partsLibrary"];
        }
        document
    }

    /// The parts-library names along an occurrence path, outermost first: the
    /// top ACOMP's part, then each placement's inside the one before. (Walked
    /// by reference: the library can be megabytes.)
    fn part_chain(&self, path: &str) -> Option<Vec<String>> {
        let mut segments = path.split(':');
        let top = segments.next()?;
        let index = self.history.index_of(top)?;
        let mut part = self.history.feature_params(index)?["partName"].as_str()?.to_string();
        let mut chain = vec![part.clone()];
        let mut library = self.history.parts_library();
        for id in segments {
            let document = &library.get(part.as_str())?["document"];
            part = document["features"]
                .as_array()?
                .iter()
                .find(|f| f["inputParams"]["id"].as_str() == Some(id))?["inputParams"]["partName"]
                .as_str()?
                .to_string();
            chain.push(part.clone());
            library = &document["partsLibrary"];
        }
        Some(chain)
    }

    /// Adopt `document` (the caller's replace edit) as ONE undo step, holding
    /// every captured mate on an occurrence without shared lineage
    /// (disabled, marked once the run lands), and classify the rest when the
    /// run lands. `from` / `to` / `lineage` are keyed by the captured paths;
    /// `notes` join the report.
    pub fn commit_replace(
        &mut self,
        capture: ReplaceCapture,
        mut document: Value,
        from: BTreeMap<String, String>,
        to: String,
        lineage: BTreeMap<String, &'static str>,
        notes: Vec<String>,
    ) -> Result<(), String> {
        let untrusted: BTreeSet<&String> =
            capture.components.iter().filter(|id| !lineage.contains_key(*id)).collect();
        if let Some(constraints) = document["assembly"]["constraints"].as_array_mut() {
            for entry in constraints.iter_mut() {
                let id = entry["inputParams"]["id"].as_str().unwrap_or_default();
                let Some(mate) = capture.affected.iter().find(|m| m.id == id) else { continue };
                if mate.refs.iter().any(|(owner, _, _)| untrusted.contains(owner)) {
                    entry["enabled"] = Value::Bool(false);
                }
            }
        }
        self.edit_document(document)?;
        self.replace_pending = Some(PendingReplace {
            generation: self.run_generation,
            components: capture.components,
            from,
            to,
            lineage,
            affected: capture.affected,
            old_names: capture.old_names,
            notes,
            origin: "replace",
            signatures: BTreeMap::new(),
        });
        self.last_replace_report = None;
        if !self.run_pending() {
            self.finish_component_replace();
        }
        Ok(())
    }

    /// Adopt `document` (the caller's part edit: `parts` rewritten in its
    /// parts library) as ONE undo step, and classify the mates of the
    /// captured occurrences once its run lands: a reference that still
    /// resolves is kept, one whose name is gone is re-attached when exactly
    /// one entity of the edited part lies on its carrier, and the rest are
    /// held for Repair (module doc, *Part edits*).
    pub fn commit_part_edit(&mut self, capture: ReplaceCapture, document: Value, parts: &[String]) -> Result<(), String> {
        // One pending check at a time: a replace (or an earlier edit) whose
        // run has not landed would otherwise lose its classification here.
        self.finish_component_replace();
        if self.replace_pending.is_some() {
            return Err("a component replace is still landing; try again in a moment".into());
        }
        let signatures: BTreeMap<String, String> = parts
            .iter()
            .map(|part| {
                let signature = document["partsLibrary"][part]["sourceSignature"].as_str().unwrap_or_default();
                (part.clone(), signature.to_string())
            })
            .collect();
        let from: BTreeMap<String, String> = capture
            .components
            .iter()
            .filter_map(|path| Some((path.clone(), self.part_at_path(path)?)))
            .collect();
        self.edit_document(document)?;
        if capture.affected.is_empty() {
            return Ok(());
        }
        self.replace_pending = Some(PendingReplace {
            generation: self.run_generation,
            lineage: capture.components.iter().map(|path| (path.clone(), "edit")).collect(),
            components: capture.components,
            to: parts.join(", "),
            from,
            affected: capture.affected,
            old_names: capture.old_names,
            notes: Vec::new(),
            origin: "edit",
            signatures,
        });
        self.last_replace_report = None;
        if !self.run_pending() {
            self.finish_component_replace();
        }
        Ok(())
    }

    /// The ACOMP ids (top level) that place any of `parts`.
    pub fn components_of_parts(&self, parts: &[String]) -> Vec<String> {
        self.history
            .features()
            .iter()
            .filter(|feature| super::components::is_acomp_feature_type(feature["type"].as_str().unwrap_or("")))
            .filter(|feature| {
                feature["inputParams"]["partName"].as_str().is_some_and(|part| parts.iter().any(|p| p == part))
            })
            .filter_map(|feature| feature["inputParams"]["id"].as_str().map(str::to_string))
            .collect()
    }

    /// Classify the mates of the replace whose run just landed, mark the
    /// unresolved ones (silently: the replace is already one undo step) and
    /// publish the report. Called from `pump` once the run is applied.
    pub(crate) fn finish_component_replace(&mut self) {
        let Some(pending) = self.replace_pending.as_ref() else { return };
        if self.run_pending() || self.applied_generation < pending.generation {
            return;
        }
        let pending = self.replace_pending.take().expect("checked above");
        // The run that landed must be the replace's own document: an undo (or
        // any document change) before it landed means there is nothing of
        // this replace to classify, and marking would hold the OLD part's
        // mates.
        let landed = if pending.origin == "edit" {
            pending.signatures.iter().all(|(part, signature)| {
                self.history.parts_library()[part.as_str()]["sourceSignature"].as_str() == Some(signature.as_str())
            })
        } else {
            pending.components.iter().all(|path| self.part_at_path(path).as_deref() == Some(pending.to.as_str()))
        };
        if !landed {
            self.last_replace_report = None;
            return;
        }
        self.ensure_assembly_session();
        let statuses = parse(&brep_kernel::assembly_statuses_json());
        // A sub-assembly's own mates report on the instance, namespaced.
        let nested = parse(&brep_kernel::assembly_dof_json())["nestedConstraints"].clone();
        let status_of = |id: &str| -> String {
            statuses
                .as_array()
                .into_iter()
                .flatten()
                .chain(nested.as_array().into_iter().flatten())
                .find(|row| row["id"] == id)
                .and_then(|row| row["status"].as_str())
                .unwrap_or("")
                .to_string()
        };
        // Vertex refs (`{solid}@x,y,z`) carry a POSITION, not a name, and no
        // vertex is listed as an entity: they are resolved again here.
        let vertex_refs: Vec<&String> = pending
            .affected
            .iter()
            .flat_map(|mate| mate.refs.iter().map(|(_, r, _)| r))
            .filter(|r| r.contains('@'))
            .collect();
        let post_vertex: BTreeMap<String, Value> = if vertex_refs.is_empty() {
            BTreeMap::new()
        } else {
            parse(&brep_kernel::assembly_resolve_refs_json(&serde_json::to_string(&vertex_refs).unwrap_or_default()))
                .as_array()
                .cloned()
                .unwrap_or_default()
                .into_iter()
                .filter_map(|row| Some((row["ref"].as_str()?.to_string(), row)))
                .collect()
        };
        // Only an affected mate's owner is ever looked up: list those, each
        // top component once. An owner among the replaced components is
        // listed; a sub-assembly's own mate is owned by the nested
        // occurrence, listed too; any other owner has nothing to compare.
        let mut listings = EntityListings::default();
        let owners: BTreeSet<&String> = pending
            .affected
            .iter()
            .flat_map(|mate| mate.refs.iter().map(move |(owner, _, _)| (mate, owner)))
            .filter(|(mate, owner)| mate.home.is_some() || pending.components.contains(owner))
            .map(|(_, owner)| owner)
            .collect();
        for owner in &owners {
            listings.index(owner);
        }
        let entities: BTreeMap<&String, Vec<&Value>> = owners.iter().map(|owner| (*owner, listings.under(owner))).collect();

        let mut kept = Vec::new();
        let mut unresolved = Vec::new();
        let mut notes = pending.notes.clone();
        let mut markers: BTreeMap<String, Value> = BTreeMap::new();
        // A part edit's automatic re-attachments: (mate id, old ref, new ref).
        let mut reattached: Vec<(String, String, String)> = Vec::new();
        for mate in &pending.affected {
            let mut ref_rows = Vec::new();
            for (owner, reference, pre) in &mate.refs {
                // Below an edited occurrence every part is the one it was,
                // or the edited part's own next version.
                let trusted = pending.lineage.contains_key(owner)
                    || (mate.home.is_some() && owner_among(&pending.components, owner).is_some_and(|top| pending.lineage.contains_key(top)));
                let outcome = if pre["ok"] != true {
                    // The reference did not resolve before either (a disabled
                    // mate's dangling ref): nothing to compare, so the user
                    // decides.
                    RefOutcome::Unresolved { reason: "missing", suggestion: None }
                } else if reference.contains('@') {
                    classify_vertex_ref(reference, pre, post_vertex.get(reference.as_str()), trusted)
                } else {
                    classify_ref(
                        owner,
                        reference,
                        pre,
                        entities.get(owner).map(Vec::as_slice).unwrap_or(&[]),
                        pending.old_names.get(owner).unwrap_or(&BTreeSet::new()),
                        trusted,
                    )
                };
                match outcome {
                    RefOutcome::Kept { split } => {
                        if split {
                            notes.push(format!(
                                "{}: '{}' was split by the new part; the mate keeps the piece that kept the name (same carrier)",
                                mate.id, reference
                            ));
                        }
                    }
                    RefOutcome::Unresolved { reason, suggestion } => {
                        let only = (pending.origin == "edit" && !reference.contains('@'))
                            .then(|| unique_carrier(pre, entities.get(owner).map(Vec::as_slice).unwrap_or(&[])))
                            .flatten();
                        match only {
                            Some(new_ref) => reattached.push((mate.id.clone(), reference.clone(), new_ref)),
                            None => ref_rows.push(json!({
                                "ref": reference,
                                "reason": reason,
                                "suggestion": suggestion.as_ref().map(Suggestion::to_json),
                            })),
                        }
                    }
                }
            }
            // Shared lineage and every name kept, but the mate no longer maps
            // (e.g. a plane–plane distance whose partner face is now a cone):
            // unresolved too, never silently half-working.
            let status = status_of(&mate.id);
            // A mate re-attached here was broken in the run that just landed
            // (its old name was gone): the rerun below judges it again.
            let mate_reattached = reattached.iter().any(|(id, _, _)| *id == mate.id);
            if ref_rows.is_empty() && !mate_reattached && mate.was_enabled && BROKEN.contains(&status.as_str()) {
                for (_, reference, _) in &mate.refs {
                    ref_rows.push(json!({ "ref": reference, "reason": "no-longer-maps", "suggestion": null }));
                }
            }
            if ref_rows.is_empty() {
                kept.push(json!({ "id": mate.id, "type": mate.constraint_type }));
                continue;
            }
            let owner = &mate.refs[0].0;
            let marker = marker_json(
                pending.origin,
                &pending.from.get(owner).cloned().or_else(|| self.part_at_path(owner)).unwrap_or_default(),
                &pending.to,
                mate.was_enabled,
                ref_rows,
            );
            unresolved.push(json!({
                "id": mate.id,
                "type": mate.constraint_type,
                "refs": marker["refs"],
            }));
            markers.insert(mate.id.clone(), marker);
        }

        if !markers.is_empty() || !reattached.is_empty() {
            let mut block = self.history.assembly_block().cloned().unwrap_or(Value::Null);
            let mut disabled_now = false;
            for entry in block["constraints"].as_array_mut().into_iter().flatten() {
                let id = entry["inputParams"]["id"].as_str().unwrap_or_default().to_string();
                for (mate, old_ref, new_ref) in &reattached {
                    if *mate != id {
                        continue;
                    }
                    for element in entry["inputParams"]["elements"].as_array_mut().into_iter().flatten() {
                        if element == old_ref.as_str() {
                            *element = Value::String(new_ref.clone());
                        }
                    }
                }
                let Some(marker) = markers.get(&id) else { continue };
                if entry["enabled"] != false {
                    entry["enabled"] = Value::Bool(false);
                    disabled_now = true;
                }
                if !entry["persistentData"].is_object() {
                    entry["persistentData"] = json!({});
                }
                entry["persistentData"][REPLACE_MARKER] = marker.clone();
            }
            self.history.set_assembly_block(block, false);
            self.write_back_inner_mates(&pending.affected, &reattached, &markers);
            // The session must see the markers (a later constraint edit
            // folds the SESSION back into the document); a mate disabled
            // only now also changes the solve.
            let _ = disabled_now;
            self.rerun_history();
        }

        // A part edit is not a dialog the user is looking at: say what it did
        // to the mates, never silently.
        if pending.origin == "edit" && (!reattached.is_empty() || !unresolved.is_empty()) {
            let mut lines = Vec::new();
            if !reattached.is_empty() {
                let moves: Vec<String> = reattached.iter().map(|(id, from, to)| format!("{id} {from} → {to}")).collect();
                lines.push(format!("re-attached {}", moves.join(", ")));
            }
            if !unresolved.is_empty() {
                let ids: Vec<&str> = unresolved.iter().filter_map(|row| row["id"].as_str()).collect();
                lines.push(format!(
                    "held for Repair (Constraints panel): {} — the face or edge it used is gone and no single one replaces it",
                    ids.join(", ")
                ));
            }
            let severity = if unresolved.is_empty() { NoticeSeverity::Info } else { NoticeSeverity::Warning };
            self.push_notice_as(severity, format!("Mates after editing {}: {}.", pending.to, lines.join("; ")));
        }
        let report = json!({
            "ok": true,
            "origin": pending.origin,
            "reattached": reattached.iter().map(|(id, from, to)| json!({ "id": id, "from": from, "to": to })).collect::<Vec<_>>(),
            "components": pending.components,
            "from": pending.from,
            "to": pending.to,
            "lineage": pending.components.iter().map(|id| {
                (id.clone(), Value::String(pending.lineage.get(id).copied().unwrap_or("none").to_string()))
            }).collect::<serde_json::Map<String, Value>>(),
            "kept": kept,
            "unresolved": unresolved,
            "notes": notes,
        });
        self.last_replace_report = Some(report);
    }

    /// The named faces and edges of an occurrence path (`ACOMP3` or a nested
    /// `ACOMP3:ACOMP1`), each with its frame in the TOP occurrence's local
    /// space: what a held mate's suggestion is chosen from.
    pub fn occurrence_entities(&mut self, path: &str) -> Vec<Value> {
        self.ensure_assembly_session();
        entities_of(path)
    }

    /// Resolve namespaced refs against the live assembly (frames in the
    /// owning top occurrence's local space; `{ref, ok, geom, point, …}` rows).
    pub fn resolve_refs(&mut self, refs: &[String]) -> Vec<Value> {
        self.ensure_assembly_session();
        parse(&brep_kernel::assembly_resolve_refs_json(&serde_json::to_string(refs).unwrap_or_default()))
            .as_array()
            .cloned()
            .unwrap_or_default()
    }

    /// The last replace's report (`null` while its run is in flight or when
    /// nothing was replaced this session): `{components, from, to, lineage,
    /// kept, unresolved: [{id, type, refs: [{ref, reason, suggestion}]}],
    /// notes}`.
    pub fn replace_report(&mut self) -> Value {
        self.finish_component_replace();
        // A report speaks for its replace only while the document shows it:
        // after an undo it is gone, after a redo it is back.
        match &self.last_replace_report {
            Some(report) if self.report_in_effect(report) => report.clone(),
            _ => Value::Null,
        }
    }

    fn report_in_effect(&self, report: &Value) -> bool {
        let to = report["to"].as_str().unwrap_or_default();
        report["components"]
            .as_array()
            .is_some_and(|paths| paths.iter().filter_map(Value::as_str).all(|path| self.part_at_path(path).as_deref() == Some(to)))
    }

    /// The parts-library name an occurrence path shows in the document: a
    /// top ACOMP's `partName`, or (`ACOMP3:ACOMP1`) the placement's inside
    /// the sub-assembly definitions. `None` when the path does not exist.
    pub fn part_at_path(&self, path: &str) -> Option<String> {
        let mut segments = path.split(':');
        let top = segments.next()?;
        let index = self.history.index_of(top)?;
        let mut part = self.history.feature_params(index)?["partName"].as_str()?.to_string();
        let mut library = self.history.parts_library();
        for id in segments {
            let document = &library[&part]["document"];
            part = document["features"]
                .as_array()?
                .iter()
                .find(|f| f["inputParams"]["id"].as_str() == Some(id))?["inputParams"]["partName"]
                .as_str()?
                .to_string();
            library = &document["partsLibrary"];
        }
        Some(part)
    }

    /// Every mate still carrying a replace marker (survives save/reopen):
    /// `[{id, type, from, to, refs}]`.
    pub fn replace_unresolved(&self) -> Value {
        let rows: Vec<Value> = self
            .history
            .assembly_block()
            .and_then(|block| block["constraints"].as_array().cloned())
            .unwrap_or_default()
            .into_iter()
            .filter(|entry| entry["persistentData"][REPLACE_MARKER].is_object())
            .map(|entry| {
                let marker = &entry["persistentData"][REPLACE_MARKER];
                json!({
                    "id": entry["inputParams"]["id"],
                    "type": entry["type"],
                    "origin": marker["origin"].as_str().unwrap_or("replace"),
                    "from": marker["from"],
                    "to": marker["to"],
                    "refs": marker["refs"],
                })
            })
            .collect();
        Value::Array(rows)
    }

    /// Repair unresolved references, as ONE undo step: each `(constraint id,
    /// old ref, new ref)` rewrites that element and clears the reference from
    /// the marker; a mate with nothing left unresolved drops its marker and
    /// is re-enabled when it was enabled before the replace. `new ref` equal
    /// to `old ref` confirms the same-name match.
    pub fn replace_repair(&mut self, repairs: &[(String, String, String)]) -> Result<(), String> {
        let mut block = self
            .history
            .assembly_block()
            .cloned()
            .ok_or("replace repair: the document has no assembly")?;
        for (id, old_ref, new_ref) in repairs {
            let entry = block["constraints"]
                .as_array_mut()
                .into_iter()
                .flatten()
                .find(|entry| entry["inputParams"]["id"] == id.as_str())
                .ok_or_else(|| format!("replace repair: no constraint '{id}'"))?;
            let marker = &mut entry["persistentData"][REPLACE_MARKER];
            let refs = marker["refs"]
                .as_array_mut()
                .ok_or_else(|| format!("replace repair: '{id}' has nothing unresolved"))?;
            let before = refs.len();
            refs.retain(|row| row["ref"] != old_ref.as_str());
            if refs.len() == before {
                return Err(format!("replace repair: '{id}' has no unresolved ref '{old_ref}'"));
            }
            let done = refs.is_empty();
            let was_enabled = marker["wasEnabled"].as_bool().unwrap_or(true);
            for element in entry["inputParams"]["elements"].as_array_mut().into_iter().flatten() {
                if element == old_ref.as_str() {
                    *element = Value::String(new_ref.clone());
                }
            }
            if done {
                if let Some(data) = entry["persistentData"].as_object_mut() {
                    data.remove(REPLACE_MARKER);
                }
                entry["enabled"] = Value::Bool(was_enabled);
            }
        }
        self.history.set_assembly_block(block, true);
        self.rerun_history();
        Ok(())
    }

    /// Accept every PRE-TICKED suggestion of the given mates (all marked
    /// mates when `ids` is empty) as one undo step. Returns how many
    /// references were repaired.
    pub fn replace_accept_suggestions(&mut self, ids: &[String]) -> Result<usize, String> {
        let unresolved = self.replace_unresolved();
        let mut repairs = Vec::new();
        for mate in unresolved.as_array().into_iter().flatten() {
            let id = mate["id"].as_str().unwrap_or_default().to_string();
            if !ids.is_empty() && !ids.contains(&id) {
                continue;
            }
            for row in mate["refs"].as_array().into_iter().flatten() {
                if row["suggestion"]["preTicked"] == true {
                    repairs.push((
                        id.clone(),
                        row["ref"].as_str().unwrap_or_default().to_string(),
                        row["suggestion"]["ref"].as_str().unwrap_or_default().to_string(),
                    ));
                }
            }
        }
        if repairs.is_empty() {
            return Ok(0);
        }
        self.replace_repair(&repairs)?;
        Ok(repairs.len())
    }
}

