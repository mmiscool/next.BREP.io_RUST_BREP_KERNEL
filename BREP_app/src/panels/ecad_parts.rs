//! Diagram and PCB offer native occurrences in this assembly, at ANY DEPTH.
//! The library key identifies the embedded part; the BOM owns each occurrence's label.
//!
//! # The walk
//!
//! A wiring diagram must reach every wireable device that is a child of this
//! assembly AND of its nested children, at arbitrary depth — EXCEPT that a
//! BOUNDARY assembly (a PCB) stops the walk. A board's own components are not
//! directly wireable; the only way through it is an interface the board itself
//! declares. That is the PART's rule, not the diagram's: the kernel applies the
//! same filter to the port set a part exports (`ports::export_ports`), and
//! [`devices`] is the consumer that honours it on the app side.
//!
//! A nested assembly's document is embedded whole — its own `features` and its
//! own `partsLibrary` — so the walk is a plain recursion into
//! `partsLibrary[partName]["document"]`, and a cycle is impossible because a
//! document literally contains its children.
//!
//! # What identifies a device
//!
//! The OCCURRENCE CHAIN (`ACOMP7:ACOMP1`) — [`PartSource::instance`], and the
//! prefix the device's connection points address under. The parts-library KEY
//! is a different namespace at every level of the tree and is NOT stable across
//! a part's version history (the kernel's `add_part_to_library` disambiguates a
//! repeat to `name-2`), so it says which document to copy a symbol out of and
//! never which device this is.
use crate::document::Document;
use brep_ecad_egui::Part;
use brep_render::engine_state::NoticeSeverity;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

/// One placed device the open assembly reaches.
pub(crate) struct Device {
    /// The occurrence chain, outermost first (`ACOMP7:ACOMP1`).
    pub(crate) occurrence: String,
    /// The parts-library key AS THE OWNING DOCUMENT SPELLS IT.
    pub(crate) key: String,
    /// The OWNING library entry's `sourceSignature` — the version of the part
    /// this occurrence carries. Read where the entry is, because a nested
    /// device's key is not a key of the open document's library. Empty for a
    /// device with nothing a sheet could place.
    pub(crate) signature: String,
    /// The reference-designator path, `/`-joined (`A1/J1`), which is what the
    /// sheet labels the device.
    pub(crate) reference: String,
    /// The blocks of the embedded part document a CONSUMER reads: `symbol`,
    /// `pads` and `ports`. Empty when the owning document's library has no
    /// entry under `key`.
    ///
    /// Not the whole document. The rest of a nested assembly's is its own
    /// features, its own parts library and its embedded solid snapshots —
    /// geometry-sized, read by the walk alone, and this runs on every revision
    /// and every sheet edit.
    pub(crate) blocks: Value,
    /// Whether the OWNING library carries an entry under `key` at all. A
    /// device whose part is missing has nothing to draw and cannot be offered,
    /// and the assembly is broken rather than ordinary — the difference between
    /// that and a plain mechanical part, which has no symbol and no pads and is
    /// not offered either.
    pub(crate) known: bool,
    /// This device's index in the `features` slice the walk was GIVEN, for the
    /// depth-0 devices alone. A nested device has none: its placement lives in
    /// a part document shared by every instance of its parent, so writing a
    /// board pose back through it would move the device in all of them.
    pub(crate) local_index: Option<usize>,
}

fn is_component(feature: &Value) -> bool {
    matches!(feature["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT"))
}

fn features_of(document: &Value) -> &[Value] {
    document["features"].as_array().map(Vec::as_slice).unwrap_or(&[])
}

/// The three blocks a consumer of [`Device`] reads, lifted out of a part
/// document. See [`Device::blocks`] for why the rest is left where it is.
fn blocks_of(document: &Value) -> Value {
    let mut blocks = serde_json::Map::new();
    for key in ["symbol", "pads", "ports"] {
        if let Some(value) = document.get(key).filter(|value| !value.is_null()) {
            blocks.insert(key.to_owned(), value.clone());
        }
    }
    Value::Object(blocks)
}

/// Every device this assembly reaches, a parent before its children.
pub(crate) fn devices(features: &[Value], library: &Value) -> Vec<Device> {
    let mut found = Vec::new();
    walk(features, library, "", "", &mut found);
    found
}

fn walk(features: &[Value], library: &Value, occurrence: &str, label: &str, found: &mut Vec<Device>) {
    for (index, feature) in features.iter().enumerate() {
        if !is_component(feature) { continue; }
        let params = &feature["inputParams"];
        let (Some(id), Some(key)) = (params["id"].as_str(), params["partName"].as_str()) else { continue; };
        let chain = if occurrence.is_empty() { id.to_owned() } else { format!("{occurrence}:{id}") };
        let designator = params["bom"]["Reference_Designator"].as_str().unwrap_or("").trim();
        let own = if designator.is_empty() { id } else { designator };
        let reference = if label.is_empty() { own.to_owned() } else { format!("{label}/{own}") };
        let entry = &library[key];
        let document = &entry["document"];
        let blocks = blocks_of(document);
        // The fallback HASHES the whole document, so it is taken only for an
        // entry that could be offered at all — an intermediate assembly with
        // neither symbol nor pads is never placed and has no version to carry.
        let signature = entry["sourceSignature"].as_str().filter(|s| !s.is_empty())
            .map(str::to_owned)
            .unwrap_or_else(|| match blocks.get("symbol").or(blocks.get("pads")) {
                Some(_) => super::parts_library::document_signature(&document.to_string()),
                None => String::new(),
            });
        found.push(Device {
            occurrence: chain.clone(), key: key.to_owned(), signature, reference: reference.clone(),
            blocks, known: !entry.is_null(),
            local_index: occurrence.is_empty().then_some(index),
        });
        // A boundary assembly's children are reached through the ports it
        // declares itself, never directly.
        if document.is_null() || brep_render::brep_kernel::document_is_boundary(document) { continue; }
        walk(features_of(document), &document["partsLibrary"], &chain, &reference, found);
    }
}

/// A device offered to a sheet, and whether the part it came from carries a
/// SYMBOL — which is what makes it a Diagram part rather than a PCB-only one.
/// Read here rather than from the open document's library, because a nested
/// device's key is not a key of THAT library.
struct Offer { part: Part, symbol: bool }


/// A part's pads as a sheet copies them: named after the part when the Pads
/// workbench left the name empty. The name is a label on the board, the BOM and a
/// fabrication file; an empty one used to refuse the whole sheet on every store
/// (the eCAD workflow audit, issue 3), and the board now tolerates it
/// (`Footprint::validate_placeable`), but a board of blank names reads worse than
/// one that says which part each footprint came from.
fn named_pads(mut pads: brep_ecad_core::board::Footprint, key: &str) -> brep_ecad_core::board::Footprint {
    if pads.name.trim().is_empty() { pads.name = key.to_owned(); }
    pads
}

/// Why a device cannot go on a sheet at all, in words: its symbol or its pads
/// break a rule a stored sheet keeps (a zero-sized pad, a repeated pin number). Such
/// a part is not offered, and `sync` names it, rather than letting the board seat
/// it and every later store of the sheet be refused for it.
fn unplaceable(part: &Part) -> Option<String> {
    let mut probe = brep_ecad_core::Document::default();
    probe.place_part(part.source.clone(), part.symbol.clone(), part.pads.clone(), None,
        brep_ecad_core::Point::default(), 0).err()
        .or_else(|| probe.validate().err())
}

fn offers(reached: Vec<Device>) -> Vec<Offer> {
    offers_and_refusals(reached).0
}

/// [`offers`], and a line for each device that could not be offered because its
/// part cannot go on a sheet ([`unplaceable`]).
fn offers_and_refusals(reached: Vec<Device>) -> (Vec<Offer>, Vec<String>) {
    let mut used = BTreeSet::new();
    let mut refused = Vec::new();
    let offered = reached.into_iter().filter_map(|device| {
        let mut reference = device.reference;
        if used.contains(&reference) { reference = format!("{reference} ({})", device.occurrence); }
        while !used.insert(reference.clone()) { reference.push('_'); }
        let key = device.key;
        let mut symbol: Option<brep_ecad_core::Symbol> = device.blocks.get("symbol").filter(|v| !v.is_null())
            .map(|v| serde_json::from_value(v.clone())).transpose().ok()?;
        // A part saved before gates existed is offered, drawn on its card and
        // placed as gates; the library entry itself is left as it was saved.
        if let Some(symbol) = &mut symbol { symbol.migrate_legacy_units(); }
        let pads = match device.blocks.get("pads").filter(|v| !v.is_null()) {
            Some(value) => Some(named_pads(serde_json::from_value(value.clone()).ok()?, &key)),
            None => None,
        };
        if symbol.is_none() && pads.is_none() { return None; }
        let signature = device.signature;
        let offer = Offer {
            symbol: symbol.is_some(),
            part: Part {
                source: brep_ecad_core::PartSource {
                    key: key.clone(), signature, instance: Some(device.occurrence),
                },
                name: key.clone(), symbol: symbol.unwrap_or_else(|| empty_symbol(&key)), pads,
                reference: Some(reference.clone()),
            },
        };
        if let Some(reason) = unplaceable(&offer.part) {
            refused.push(format!(
                "'{reference}' cannot go on a sheet: part '{key}' {reason}. Fix it in the part's \
                 Symbol or Pads workbench, then Update components."));
            return None;
        }
        Some(offer)
    }).collect();
    (offered, refused)
}

// The sheet schema stores a symbol on every component. Pads-only instances use
// an empty storage value; no symbol or pins are added to the native part.
fn empty_symbol(name: &str) -> brep_ecad_core::Symbol {
    brep_ecad_core::Symbol { unit_count: 0, graphic_gates: vec![], hide_pin_names: false, hide_pin_numbers: false, properties: BTreeMap::new(), power_net: None,
        library_id: name.into(), reference_prefix: "P".into(), description: String::new(),
        graphics: Vec::new(), pins: Vec::new() }
}

/// What [`follow_occurrences`] last saw of a document's assembly: the revision it
/// looked at, the undo step on top then, and the top-level occurrences it held.
#[derive(Debug, Default)]
pub(crate) struct OccurrenceMark {
    revision: u64,
    step: Option<u64>,
    /// Each top-level occurrence by feature id, with its part key and its label
    /// ([`Occurrence`]).
    occurrences: BTreeMap<String, Occurrence>,
    /// Occurrences a forward step added with no designator, whose part the library
    /// did not carry yet — so their symbol's prefix was not known. Kept while that
    /// step is still on top, and designated as soon as the part arrives.
    pending: BTreeSet<String>,
}

/// A top-level occurrence as a sheet component can be bound to it: by its feature
/// id (the chain a component carries), or — for a LEGACY component, drawn before
/// components carried a chain — by its part key and its label, which is what
/// [`follow_sheet`] binds one by.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Occurrence { key: String, reference: String }

/// The top-level assembly occurrences, by feature id.
fn top_occurrences(history: &brep_render::history::History) -> BTreeMap<String, Occurrence> {
    (0..history.len()).filter(|&index| history.feature_type(index)
        .is_some_and(|kind| matches!(kind.as_str(), "ACOMP" | "ASSEMBLY COMPONENT")))
        .filter_map(|index| {
            let id = history.feature_id(index)?;
            let params = history.feature_params(index)?;
            let designator = params["bom"]["Reference_Designator"].as_str().unwrap_or("").trim();
            let reference = if designator.is_empty() { id.clone() } else { designator.to_owned() };
            Some((id, Occurrence { key: params["partName"].as_str().unwrap_or("").to_owned(), reference }))
        })
        .collect()
}

/// Follow the assembly's occurrences to the eCAD sheets, INSIDE the undo step
/// that moved them. Called every frame, in every workbench, and costs one integer
/// comparison on a frame in which the document did not move.
///
/// - An occurrence a step ADDED with no `Reference_Designator` takes the next free
///   one for its symbol's prefix (`U1`, `R3`), written into its BOM attributes. The
///   BOM owns the designator, and every sheet labels the component by it, so a part
///   is `U1` on the schematic, the board, the diagram and the BOM alike rather than
///   its assembly feature id (`ACOMP3`; the eCAD workflow audit, issue 6).
/// - An occurrence a step REMOVED takes its components off both sheets and the board,
///   and its harness connections with them, and says what went (issue 4). The wires
///   and tracks that ran to it stay, as unconnected copper and unconnected wire ends,
///   exactly as a Delete on the schematic leaves them.
///
/// Both write with no undo checkpoint of their own, so they join the step that made
/// them (`History::adopt_document`, `History::fold_param_no_undo`): one undo of the
/// delete brings the component, its placement, its tracks' connections and its
/// harness connections back together, and one undo of the insert takes the
/// designator with the occurrence.
///
/// Only a FORWARD step is followed: a new undo step on top and nothing to redo. An
/// undo is never followed, since it restores a snapshot that already holds both sides
/// as they stood together. A redo that empties the redo stack does read as forward,
/// and finds its snapshot already consistent, so it retires and designates nothing. And a step that both removes and adds occurrences is a restructure
/// (a part moved into a sub-assembly changes its chain), which is the UNLINKED
/// picker's to resolve — deleting the component there would lose it.
pub(crate) fn follow_occurrences(doc: &mut Document) {
    let history = &doc.engine.history;
    let revision = history.revision();
    if doc.ecad_occurrences.as_ref().is_some_and(|mark| mark.revision == revision) { return; }
    let now = top_occurrences(history);
    let step = history.undo_top();
    let forward = !history.can_redo();
    let Some(mark) = doc.ecad_occurrences.take() else {
        // The first sight of a document records it and acts on nothing: a loaded
        // file's occurrences are not new, and its labels are the user's.
        doc.ecad_occurrences = Some(OccurrenceMark { revision, step, occurrences: now, pending: BTreeSet::new() });
        return;
    };
    let moved = forward && step.is_some() && step > mark.step;
    let gone: Vec<(String, Occurrence)> = mark.occurrences.iter()
        .filter(|(id, _)| !now.contains_key(*id)).map(|(id, o)| (id.clone(), o.clone())).collect();
    let added: BTreeSet<String> = now.keys().filter(|id| !mark.occurrences.contains_key(*id)).cloned().collect();
    let mut pending: BTreeSet<String> = if step == mark.step && forward {
        mark.pending.iter().filter(|id| now.contains_key(*id)).cloned().collect()
    } else {
        BTreeSet::new()
    };
    if moved && gone.is_empty() { pending.extend(added.iter().cloned()); }
    if !pending.is_empty() { pending = designate(&mut doc.engine, pending); }
    if moved && !gone.is_empty() && added.is_empty() { retire(&mut doc.engine, &gone); }
    doc.ecad_occurrences = Some(OccurrenceMark {
        revision: doc.engine.history.revision(), step: doc.engine.history.undo_top(), occurrences: now, pending,
    });
}

/// The designators already in use: every device's label the walk reaches and every
/// component label on either sheet (a free symbol's too, since a sheet refuses a
/// label twice).
fn designators_in_use(history: &brep_render::history::History, features: &[Value]) -> BTreeSet<String> {
    let mut used: BTreeSet<String> = devices(features, history.parts_library()).into_iter()
        .flat_map(|device| {
            let own = device.reference.rsplit('/').next().unwrap_or_default().to_owned();
            [device.reference, own]
        })
        .collect();
    for block in [history.pcb_block(), history.diagram_block()].into_iter().flatten() {
        for component in block["components"].as_array().into_iter().flatten() {
            if let Some(reference) = component["reference"].as_str() { used.insert(reference.to_owned()); }
        }
    }
    used
}

/// Give each of `occurrences` that has no designator the lowest free one for its
/// symbol's prefix, in assembly order. Returns the ones whose part the library does
/// not carry yet, to try again when it does. A part with no symbol, or a symbol with
/// no prefix, is left as it is: a bolt has no designator to take.
fn designate(engine: &mut brep_render::engine_state::EngineState, occurrences: BTreeSet<String>) -> BTreeSet<String> {
    let history = &engine.history;
    let features: Vec<Value> = (0..history.len()).filter_map(|index| {
        let kind = history.feature_type(index)?;
        if !matches!(kind.as_str(), "ACOMP" | "ASSEMBLY COMPONENT") { return None; }
        Some(serde_json::json!({"type": kind, "inputParams": history.feature_params(index)?}))
    }).collect();
    let mut used = designators_in_use(history, &features);
    let mut waiting = BTreeSet::new();
    let mut writes: Vec<(String, Value)> = Vec::new();
    for device in devices(&features, history.parts_library()) {
        if !occurrences.contains(&device.occurrence) { continue; }
        let Some(index) = device.local_index else { continue; };
        let params = &features[index]["inputParams"];
        if params["bom"]["Reference_Designator"].as_str().is_some_and(|d| !d.trim().is_empty()) { continue; }
        if !device.known { waiting.insert(device.occurrence); continue; }
        let prefix = device.blocks["symbol"]["reference_prefix"].as_str().unwrap_or_default().trim().to_owned();
        if prefix.is_empty() || prefix.contains(['/', ':']) { continue; }
        let designator = (1u32..).map(|n| format!("{prefix}{n}")).find(|d| !used.contains(d)).unwrap_or_default();
        used.insert(designator.clone());
        let mut bom = params["bom"].as_object().cloned().unwrap_or_default();
        bom.insert("Reference_Designator".into(), Value::String(designator));
        writes.push((device.occurrence, Value::Object(bom)));
    }
    for (id, bom) in writes { engine.history.fold_param_no_undo(&id, "bom", bom); }
    waiting
}

/// Whether an occurrence chain is `occurrence` or lies inside it.
fn under(chain: &str, occurrence: &str) -> bool {
    chain == occurrence || chain.strip_prefix(occurrence).is_some_and(|rest| rest.starts_with(':'))
}

/// Take the components of the removed top-level `gone` occurrences off both sheets
/// and the board, and the harness connections that end on them, in the step that
/// removed them, and say what went. See [`follow_occurrences`].
fn retire(engine: &mut brep_render::engine_state::EngineState, gone: &[(String, Occurrence)]) {
    let Ok(mut draft) = serde_json::from_str::<Value>(&engine.history.request_json()) else { return; };
    let lost = |instance: Option<&str>| instance.is_some_and(|chain| gone.iter().any(|(g, _)| under(chain, g)));
    // A legacy component carries no chain, and is the gone device's when its key and
    // its label are — the rule `follow_sheet` binds it by.
    let lost_component = |c: &brep_ecad_core::Component| match &c.part {
        Some(source) if source.instance.is_some() => lost(source.instance.as_deref()),
        Some(source) => gone.iter().any(|(_, o)| o.key == source.key && o.reference == c.reference),
        None => false,
    };
    let mut lines: Vec<String> = Vec::new();
    let mut retired: Vec<String> = Vec::new();
    let mut changed = false;
    for (block, sheet_name) in [("pcb", "the schematic and the board"), ("diagram", "the wiring diagram")] {
        let Some(value) = draft.get(block).filter(|v| !v.is_null()).cloned() else { continue; };
        // An unreadable block is its editor's to report, and is not written over.
        let Ok(mut sheet) = brep_ecad_core::Document::from_value(value) else { continue; };
        let doomed: Vec<(brep_ecad_core::Uuid, String)> = sheet.components.iter()
            .filter(|c| lost_component(c))
            .map(|c| (c.id, c.reference.clone())).collect();
        if doomed.is_empty() { continue; }
        let wired = sheet.wires.iter().filter(|w| [&w.start, &w.end].into_iter().flatten()
            .any(|t| doomed.iter().any(|(id, _)| *id == t.component))).count();
        let placed = doomed.iter().filter(|(id, _)| sheet.board.placement(*id).is_some()).count();
        for (id, _) in &doomed { sheet.delete_component(*id); }
        let Ok(stored) = sheet.to_value() else { continue; };
        draft[block] = stored;
        changed = true;
        for (_, name) in &doomed { if !retired.contains(name) { retired.push(name.clone()); } }
        let plural = |n: usize, one: &str| format!("{n} {one}{}", if n == 1 { "" } else { "s" });
        let mut line = format!("off {sheet_name}");
        if block == "pcb" {
            let mut kept = Vec::new();
            if wired > 0 { kept.push(format!("{} to it left unconnected", plural(wired, "wire"))); }
            if placed > 0 { kept.push("any track to its pads kept as unconnected copper".to_owned()); }
            if !kept.is_empty() { line.push_str(&format!(" ({})", kept.join("; "))); }
        } else if wired > 0 {
            line.push_str(&format!(" with its {}", plural(wired, "connection")));
        }
        lines.push(line);
    }
    if let Some(connections) = draft["wireHarness"]["connections"].as_array_mut() {
        let before = connections.len();
        connections.retain(|c| !["from", "to"].iter().any(|end| lost(c[end].as_str())));
        let dropped = before - connections.len();
        if dropped > 0 {
            changed = true;
            lines.push(format!("{dropped} harness connection{} dropped", if dropped == 1 { "" } else { "s" }));
        }
    }
    if !changed { return; }
    if let Err(error) = engine.history.adopt_document(&draft.to_string()) {
        engine.push_notice(format!("The deleted part stays on the eCAD sheets: {error}"));
        return;
    }
    engine.rebuild_current_history();
    let what = if retired.is_empty() {
        gone.iter().map(|(_, o)| o.reference.as_str()).collect::<Vec<_>>().join(", ")
    } else {
        retired.join(", ")
    };
    engine.push_notice_as(NoticeSeverity::Warning, format!("{what} left the assembly, so it left eCAD too: {}. Undo brings it all back.",
        lines.join(", ")));
}

pub(crate) fn sync(doc: &mut Document) {
    follow_occurrences(doc);
    if !matches!(doc.engine.settings.workbench.as_str(), "diagram" | "pcb") { return; }
    let history = &doc.engine.history;
    let revision = history.revision();
    if doc.ecad_parts_revision == Some(revision) { return; }
    // Read the library by reference: serializing whole embedded STEP parts just
    // to draw their symbol cards would put geometry-sized work on every edit.
    let features: Vec<_> = (0..history.len()).filter_map(|index| {
        let kind = history.feature_type(index)?;
        if !matches!(kind.as_str(), "ACOMP" | "ASSEMBLY COMPONENT") { return None; }
        Some(serde_json::json!({"type": kind, "inputParams": history.feature_params(index)?}))
    }).collect();
    let reached = devices(&features, history.parts_library());
    // A device whose part the library does not carry has nothing to draw, so it
    // cannot be offered — but the assembly is BROKEN, not ordinary, and used to
    // go from the palette without a word.
    let missing: Vec<String> = reached.iter().filter(|device| !device.known)
        .map(|device| format!(
            "'{}' is placed from part '{}', which this assembly's library does not \
             carry — it cannot be offered to Diagram or PCB.", device.reference, device.key))
        .collect();
    let (offered, refused) = offers_and_refusals(reached);
    let missing: Vec<String> = missing.into_iter().chain(refused).collect();
    // A nested device's key is a key of ITS OWN parent's library, not of this
    // document's, so "does it have a symbol" is carried out of the walk rather
    // than looked up here.
    doc.ecad.diagram.parts = offered.iter().filter(|offer| offer.symbol).map(|offer| offer.part.clone()).collect();
    doc.ecad.pcb.parts = offered.into_iter().map(|offer| offer.part).collect();
    // The pane's button runs Assembly's own Add Component (`take_add_part_request`),
    // so the hint under it says only what the button does not: which parts it lists.
    let hints = [
        (&mut doc.ecad.diagram, "A part appears here once the assembly holds it and it has a symbol. Draw one in the Symbol workbench, or import one from KiCad into the part."),
        (&mut doc.ecad.pcb, "A part appears here once the assembly holds it and it has a symbol or pads. Parts with pads go onto the board as well."),
    ];
    for (editor, hint) in hints {
        editor.parts_hint = hint.into();
        editor.add_part_label = Some(ADD_PART_LABEL.into());
        // The BOM owns a placed part's designator (a new part takes the next free
        // one for its prefix, `follow_occurrences`), and a rename there reaches both
        // sheets and the board in one step, so the Inspector shows it read-only and
        // says where it is edited.
        editor.labels_from_host = Some("Named by the BOM: rename it in the BOM pane's Reference_Designator column.".into());
    }
    // Once, not once per revision: the revision bumps on every edit, and a
    // device whose part is missing is a STANDING state, not an event — the same
    // rule `BlockSync::unreported` keeps for an UNLINKED component.
    let fresh: Vec<String> = missing.iter().filter(|line| !doc.ecad_parts_reported.contains(line)).cloned().collect();
    doc.ecad_parts_reported = missing;
    for line in fresh { doc.engine.push_notice_as(NoticeSeverity::Warning, line); }
    doc.ecad_parts_revision = Some(revision);
}

/// The parts pane's button. It says what happens rather than naming the toolbar
/// button it presses: the part goes into the ASSEMBLY, which is where this pane
/// reads its list from.
const ADD_PART_LABEL: &str = "Add part to assembly\u{2026}";

/// The parts pane's widgets, published as `__brepEcadPartsHit`.
pub static HIT_KEYS: &[crate::automation::hit_keys::HitKeyDoc] = &[
    crate::automation::hit_keys::HitKeyDoc { panel: "ecadparts", prefix: "parts:add", meaning: "add a part to the assembly: Assembly's Add Component (the insert-component modal), the same arm as the toolbar's assembly.add_component.", command: Some("workbench_button") },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecadparts", prefix: "parts:search", meaning: "the search field over the part cards (shown while the pane lists any)", command: None },
    crate::automation::hit_keys::HitKeyDoc { panel: "ecadparts", prefix: "parts:card:", meaning: "a part card (parts:card:<reference>, or <name> for a part with no reference), visible rows only: a click starts placing it, a drag drops it on the sheet, a right-click opens its menu", command: None },
];

/// Whether the parts pane asked for a part to be added since the last frame, and
/// the pane's rects as it drew them, which are published here so that a pane that
/// was not drawn publishes none. Called every frame, not only on a new revision.
pub(crate) fn take_add_part_request(doc: &mut Document) -> bool {
    let active = match doc.engine.settings.workbench.as_str() {
        "diagram" => Some(&doc.ecad.diagram),
        "pcb" => Some(&doc.ecad.pcb),
        _ => None,
    };
    if crate::automation::registry::enabled() {
        let hits: Vec<(String, eframe::egui::Rect)> = active.map(|e| e.library_hits.clone()).unwrap_or_default();
        crate::automation::registry::publish(
            "__brepEcadPartsHit",
            "the eCAD parts pane's widget rects (parts:add, parts:search, parts:card:<reference>)",
            &crate::automation::hit_rects::hits_json(hits.iter().map(|(k, r)| (k, r))),
        );
    }
    let mut asked = false;
    for editor in [&mut doc.ecad.diagram, &mut doc.ecad.pcb] {
        editor.library_hits.clear();
        asked |= std::mem::take(&mut editor.add_part_requested);
    }
    asked
}

pub(crate) fn take_open_requests(doc: &mut Document) -> Vec<String> {
    let requests: BTreeSet<_> = doc.ecad.diagram.open_part_requests.drain(..)
        .chain(doc.ecad.pcb.open_part_requests.drain(..)).collect();
    if requests.is_empty() { return Vec::new(); }
    let document: Value = serde_json::from_str(&doc.engine.history.request_json()).unwrap_or_default();
    // A nested device's key is a key of ITS OWN parent's library, so the entry
    // is found by walking rather than by one lookup. The `sourceKey` it carries
    // names a store document whatever depth it was found at.
    let mut entries: BTreeMap<String, String> = BTreeMap::new();
    let mut libraries = vec![document["partsLibrary"].clone()];
    while let Some(library) = libraries.pop() {
        for (key, entry) in library.as_object().into_iter().flatten() {
            let source = entry["sourceKey"].as_str().unwrap_or("").to_owned();
            entries.entry(key.clone()).or_insert(source);
            let nested = entry["document"]["partsLibrary"].clone();
            if nested.is_object() { libraries.push(nested); }
        }
    }
    requests.into_iter().filter_map(|part| {
        match entries.get(&part).filter(|key| !key.is_empty()) {
            Some(key) => Some(key.clone()),
            None => { doc.engine.push_notice(format!("'{part}' is embedded only; use Edit Part in Assembly to edit it.")); None }
        }
    }).collect()
}

/// Refresh both sheet snapshots to the versions the assembly NOW carries.
/// `assembly` is the caller's DRAFT, with the new library entry already in it;
/// `before` is the assembly as it stood, and is what a point rename is read
/// against. Build the whole change in a draft so an unreadable block cannot
/// leave half an update behind (`parts_library::refresh_library_entry`).
///
/// # Keyed on the OCCURRENCE CHAIN, not on the library key
///
/// A component refreshes to the version ITS OWN device carries. The key cannot
/// say which version that is, in either direction: refreshing the entry `sub`
/// changes the part a component placed from `ACOMP9:ACOMP1` holds, whose key is
/// `connector` in `sub`'s own library and never `sub`; and refreshing a
/// TOP-LEVEL `connector` used to stamp that nested component too, because its
/// key reads the same in a namespace one level down. Both are the same mistake
/// — the key is not an identity — and the chain is, which is the rule
/// [`follow_sheet`] and `Document::relink` already bind by.
///
/// `key` is still read for the one component the chain cannot name: a LEGACY
/// one, drawn before a component carried an occurrence at all.
///
/// # Per component, and soft
///
/// Each component is refreshed on its own (`Document::refresh_component`),
/// because two components can legitimately want different versions of the same
/// key. A component the walk cannot resolve — a dangling chain — is left
/// alone and says nothing HERE: [`follow_sheet`] names it UNLINKED in the same
/// transaction IF A WIRE LANDS ON IT, and saying that twice is noise. An
/// unwired dangling symbol is silent in both, which is `follow_sheet`'s reach
/// and not this call's to widen. A component whose new part is
/// unreadable, or whose sheet refuses the new symbol, is named and SKIPPED:
/// one bad part inside a nested assembly costing the user the whole refresh is
/// the outcome the depth slice removed for a dangling instance, and it is the
/// same outcome here.
///
/// Returns the renames it acted on WITHOUT evidence, one line each, and the
/// components it could not refresh. A connection point's NAME is its identity,
/// so there is no id for two saved versions to join on: `part_pins::point_renames`
/// pairs what both versions SEAT the same way and reads the rest positionally,
/// and a positional read cannot tell a rename from a delete-and-add. It is
/// applied anyway — leaving it unapplied deletes the user's connection, which
/// is the data loss the 2026-09-18 record set out to stop — but it is not
/// applied silently.
pub(crate) fn refresh_snapshots(
    assembly: &mut Value, before: &Value, key: &str,
) -> Result<Vec<String>, String> {
    let now = devices(features_of(assembly), &assembly["partsLibrary"]);
    let then = devices(features_of(before), &before["partsLibrary"]);
    let nothing = Value::Null;
    // The refreshed entry itself, standing in for a legacy component's device.
    // It has no occurrence, which is exactly what makes it legacy.
    let entry = &assembly["partsLibrary"][key];
    let legacy = Device {
        occurrence: String::new(), key: key.to_owned(),
        signature: entry["sourceSignature"].as_str().unwrap_or_default().to_owned(),
        reference: String::new(), blocks: blocks_of(&entry["document"]), known: true,
        local_index: None,
    };
    let legacy_before = blocks_of(&before["partsLibrary"][key]["document"]);
    let resolve = |source: &brep_ecad_core::PartSource| -> Option<(&Device, &Value)> {
        match &source.instance {
            Some(chain) => Some((
                now.iter().find(|device| device.occurrence == *chain)?,
                then.iter().find(|device| device.occurrence == *chain)
                    .map_or(&nothing, |device| &device.blocks),
            )),
            None if source.key == key => Some((&legacy, &legacy_before)),
            None => None,
        }
    };
    let mut notes: Vec<String> = Vec::new();
    for block in ["diagram", "pcb"] {
        let Some(value) = assembly.get(block).filter(|v| !v.is_null()) else { continue; };
        let mut sheet = brep_ecad_core::Document::from_value(value.clone())
            .map_err(|e| format!("{block}: {e}"))?;
        let mut moved = false;
        for component in sheet.components.clone() {
            let Some(source) = &component.part else { continue; };
            let Some((device, was)) = resolve(source) else { continue; };
            // An intermediate assembly with neither symbol nor pads carries no
            // version, and a component holding the version it is offered has
            // nothing to take.
            if device.signature.is_empty() || device.signature == source.signature { continue; }
            let mut refused = |error: String| {
                let line = format!("'{}' kept the version it had: part '{}' {error}",
                    component.reference, device.key);
                if !notes.contains(&line) { notes.push(line) }
            };
            let symbol = match device.blocks.get("symbol").filter(|v| !v.is_null())
                .map(|v| serde_json::from_value(v.clone())).transpose() {
                Ok(symbol) => symbol.unwrap_or_else(|| empty_symbol(&device.key)),
                Err(error) => { refused(format!("symbol: {error}")); continue; }
            };
            let pads: Option<brep_ecad_core::board::Footprint> =
                match device.blocks.get("pads").filter(|v| !v.is_null())
                    .map(|v| serde_json::from_value(v.clone())).transpose() {
                Ok(pads) => pads.map(|pads| named_pads(pads, &device.key)),
                Err(error) => { refused(format!("pads: {error}")); continue; }
            };
            let renames = brep_render::brep_kernel::point_renames(was, &device.blocks);
            if let Err(error) = sheet.refresh_component(
                component.id, &symbol, pads.as_ref(), &device.signature, &renames.all()) {
                refused(error);
                continue;
            }
            moved = true;
            for (from, to) in &renames.positional {
                let line = format!(
                    "'{}': point '{from}' was read as a rename to '{to}' by POSITION — nothing in \
                     either version seats it, so a deleted point and a new one would read the \
                     same. Check the connections on '{to}'.", device.key);
                if !notes.contains(&line) { notes.push(line) }
            }
        }
        if moved { assembly[block] = sheet.to_value()?; }
    }
    Ok(notes)
}

/// Endpoint identities from the embedded part declarations, for a refresh before
/// the new parts have run. Resolution still goes through the kernel's one resolver.
///
/// Every device the walk reaches contributes the points ITS OWN document
/// declares, addressed under its occurrence chain — which is the same set the
/// kernel's run exports, boundary filter included: a board contributes its
/// declared interface and its components contribute nothing, because the walk
/// never reached them.
pub(crate) fn declared_endpoints(document: &Value) -> Vec<brep_render::brep_kernel::WireHarnessEndpoint> {
    devices(features_of(document), &document["partsLibrary"]).iter()
        .flat_map(|device| brep_render::brep_kernel::declared_point_endpoints(
            &device.occurrence, &device.blocks))
        .collect()
}

/// How an UNLINKED component is put to the user: what it was bound to, why that
/// no longer resolves, and that the answer is to pick the device — never a
/// guess this code makes on their behalf.
fn unlinked_note(component: &brep_ecad_core::Component, reason: &str) -> String {
    format!("'{}' is UNLINKED: {reason}. Pick the device it belongs to.", component.reference)
}

/// Fold a sheet edit into the same assembly document. Legacy, unbound symbols
/// remain editable; only components placed from this assembly drive its 3D data.
///
/// Returns the per-component problems, one line each. A component whose
/// occurrence the assembly no longer has is UNLINKED: it is skipped, it is
/// named, and everything else in the sheet still folds. One dangling instance
/// used to fail the whole call, and with it the whole part-refresh transaction
/// (`parts_library::refresh_library_entry`). `Err` is now reserved for a sheet
/// this document cannot read at all.
pub(crate) fn follow_sheet(
    document: &mut Value, block: &str, sheet: &brep_ecad_core::Document,
    endpoints: &[brep_render::brep_kernel::WireHarnessEndpoint],
) -> Result<Vec<String>, String> {
    let features = document["features"].as_array().cloned().unwrap_or_default();
    let reached = devices(&features, &document["partsLibrary"]);
    let mut problems = Vec::new();
    let mut note = |line: String| if !problems.contains(&line) { problems.push(line) };
    // The occurrence CHAIN is the identity, so a part re-versioned under a new
    // library key (`add_part_to_library` disambiguates a repeat to `name-2`)
    // still binds. A component with no chain at all is legacy, matched by its
    // label and its key as it always was; one drawn without a PART is a free
    // symbol, not a broken link, and says nothing.
    let device = |component: &brep_ecad_core::Component| -> Result<Option<&Device>, String> {
        let Some(source) = &component.part else { return Ok(None) };
        if let Some(chain) = &source.instance {
            return reached.iter().find(|device| device.occurrence == *chain).map(Some).ok_or_else(||
                unlinked_note(component, &format!("this assembly no longer has the occurrence '{chain}'")));
        }
        let matches: Vec<&Device> = reached.iter().filter(|device|
            device.key == source.key && device.reference == component.reference).collect();
        match matches.as_slice() {
            [device] => Ok(Some(device)),
            [] => Err(unlinked_note(component, "no device of this assembly carries its label")),
            several => Err(unlinked_note(component,
                &format!("{} devices carry its label", several.len()))),
        }
    };
    if block == "pcb" {
        for placement in &sheet.board.placements {
            let Some(component) = sheet.components.iter().find(|c| c.id == placement.component) else { continue; };
            let device = match device(component) {
                Ok(Some(device)) => device,
                Ok(None) => continue,
                Err(line) => { note(line); continue; }
            };
            // A NESTED device's placement lives in a part document that every
            // instance of its parent shares, so writing a board pose through it
            // would move the device in all of them. Only this document's own
            // components take one.
            let Some(index) = device.local_index else {
                note(format!("'{}' sits inside '{}' and its board placement is not written back to the assembly.",
                    component.reference, device.occurrence));
                continue;
            };
            // Pad coordinates are µm with Y down; BREP is mm with Y up.
            // The bottom mirror in X is a rigid 180° turn about Y in 3D.
            // A part is referenced to the COPPER PLANE it is soldered to: the
            // top one is z = 0, and a part on the underside sits on the bottom
            // one, a substrate's thickness below it. Until the board had a
            // thickness both sides were written at 0, which put every
            // bottom-side part inside the board the moment there was a board to
            // be inside (`board_geometry`).
            let angle = f64::from(placement.rotation % 4) * 90.;
            let z = if placement.bottom {
                -f64::from(brep_ecad_core::board::SUBSTRATE_THICKNESS) / 1000.
            } else {
                0.
            };
            let params = &mut document["features"][index]["inputParams"];
            params["transform"] = serde_json::json!({
                "translate": [f64::from(placement.at.x) / 1000., -f64::from(placement.at.y) / 1000., z],
                "rotateEulerDeg": [0., if placement.bottom {180.} else {0.}, if placement.bottom {angle} else {-angle}],
            });
        }
    } else if block == "diagram" {
        let mut connections = document["wireHarness"]["connections"].as_array().cloned().unwrap_or_default();
        // Only our UUID namespace is replaced. Manual harness wires keep their ids,
        // diameters and destinations; existing diagram wires keep their diameter.
        let previous = connections.clone();
        connections.retain(|c| !c["id"].as_str().is_some_and(|id| id.starts_with("diagram:")));
        for wire in &sheet.wires {
            let (Some(start), Some(end)) = (&wire.start, &wire.end) else { continue; };
            let resolve = |terminal: &brep_ecad_core::Terminal| -> Result<Option<String>, String> {
                let component = sheet.components.iter().find(|c| c.id == terminal.component)
                    .ok_or_else(|| "A connection names a component the sheet no longer has.".to_string())?;
                let Some(device) = device(component)? else {
                    // A free symbol drives no 3D data, so a wire to one is not
                    // a harness connection and is not a problem either.
                    return Ok(None);
                };
                // The port GROUP, so a name two identical connectors of one
                // part share resolves instead of being refused. A `Terminal`
                // carries no unit, but the component's own symbol does, and one
                // unit maps to one group. A pin the held symbol does not carry,
                // or a unit the part maps to nothing, falls back to "any" and
                // is refused by name if it is genuinely repeated.
                let group = component.symbol.pins.iter().find(|pin| pin.number == terminal.pin)
                    .and_then(|pin| brep_render::brep_kernel::port_group_for_unit(&device.blocks, pin.unit));
                brep_render::brep_kernel::resolve_component_pin(
                    endpoints, &device.occurrence, group.as_deref(), &terminal.pin)
                    .map(Some)
                    .map_err(|reason| format!("'{}' pin '{}': {reason}", component.reference, terminal.pin))
            };
            let (from, to) = match (resolve(start), resolve(end)) {
                (Ok(Some(from)), Ok(Some(to))) => (from, to),
                (from, to) => {
                    for line in [from.err(), to.err()].into_iter().flatten() { note(line); }
                    continue;
                }
            };
            let id = format!("diagram:{}", wire.id);
            let diameter = previous.iter().find(|c| c["id"] == id)
                .and_then(|c| c["diameter"].as_f64()).unwrap_or(1.0);
            connections.push(serde_json::json!({"id": id, "name": wire.connection_id,
                "from": from, "to": to, "diameter": diameter}));
        }
        if !connections.is_empty() || document.get("wireHarness").is_some() {
            if !document["wireHarness"].is_object() { document["wireHarness"] = serde_json::json!({}); }
            document["wireHarness"]["connections"] = Value::Array(connections);
        }
    }
    Ok(problems)
}

