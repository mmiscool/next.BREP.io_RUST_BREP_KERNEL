//! The eCAD editors in the central tile — the Diagram, PCB, Symbol and Pads
//! workbenches draw `brep_ecad_egui`'s editors where the 3D view goes, the way an
//! open drawing sheet draws paper there (`sheet.rs`).
//!
//! # The shape, and what is not copied from the sheet
//!
//! [`Viewport::show_ecad`] is the sheet's shape: the whole tile, no 3D input,
//! render or blit, and its widgets published in screen points under their own
//! panel (`ecad/…`). It is a SIBLING of [`Viewport::show`] rather than a branch
//! in it, because `show` is handed the engine alone and the editors live beside
//! it on [`Document`]. That is also the difference not to copy: the sheet's pan
//! and zoom are the viewport's and are forgotten on every document switch,
//! while each editor keeps its own view inside itself, on its document, so a
//! tab switch cannot touch it.
//!
//! # One frame, in order
//!
//! BREP's undo (`handle_shortcuts`, before anything draws) → **pull**: the
//! block moved under the editor, so hand it the host's copy
//! (`set_document`, `set_symbol`, `set_footprint`, which keep the view and
//! clear eCAD's own history) → **show** → **take_change** → **store**: the
//! edit becomes one undo step of the document ([`BlockSync`]). Pull comes
//! before show, or an undo in the same frame would race the editor's commit.
//! There is ONE undo stack, BREP's: eCAD's history keys never reach an editor,
//! because `handle_shortcuts` takes them first.
//!
//! # eCAD's text and the app's fonts
//!
//! eCAD draws its captions, tooltips and canvas labels as text, and the app
//! ships no font fallback chain (`crate::fonts`): on native Linux every
//! character comes from the ONE face `fc-match monospace` names, and everywhere
//! else — wasm, and native without fontconfig, i.e. Windows and macOS without
//! it installed — from egui's four bundled faces, whose Proportional chain
//! (Ubuntu-Light, NotoEmoji, emoji-icon-font) is the narrow one. A character
//! neither carries is drawn as a tofu box. Measured when eCAD moved in, by the
//! fonts' own charmaps: the eight non-ASCII characters eCAD's strings used
//! (`− ° … · — → “ ”`) were all in DejaVu Sans Mono, Liberation Mono and
//! FreeMono, and the bundled Proportional chain lacked one, `→` U+2192, in a
//! single hover text — which now reads `->`. The test below keeps it that way
//! for every string eCAD adds.

use super::*;
use crate::document::ecad::{Block, BlockSync};
use crate::document::Document;
use crate::workbench::ecad::{Editors, Target};
use brep_render::engine_state::NoticeSeverity;
use brep_render::history::History;
use std::collections::HashMap;

/// The block of the document an editor keeps its document in.
pub(crate) fn block_of(target: Target) -> Block {
    match target {
        Target::Diagram => Block::Diagram,
        Target::Pcb => Block::Pcb,
        Target::Symbol => Block::Symbol,
        Target::Pads => Block::Pads,
    }
}

/// The eCAD host's transient state on the viewport: this frame's rects,
/// whether the Label tool was armed last frame (its name field takes focus on
/// the frame the tool arms), and which widget had the keyboard last frame.
#[derive(Default)]
pub struct EcadViewport {
    hits: HashMap<String, egui::Rect>,
    label_armed: bool,
    focus: Option<egui::Id>,
    /// The history revision the pads editor was last told its symbol's pins at.
    pins_at: Option<u64>,
    /// The history revision the symbol editor was last told its pins' points
    /// at. Its twin: the same join read from the other side.
    points_at: Option<u64>,
    /// What the status strip under the tile says about the editor, as of the
    /// last frame it was drawn ([`status_of`]).
    status: Status,
    /// The components on the sheet drawn last frame whose part changed since
    /// they were placed, as `(reference, part)` ([`outdated_components`]).
    outdated: Vec<(String, String)>,
    /// The out-of-date list as it stood when the user pressed Later: the card
    /// stays folded to its one-line chip while the list is still that one.
    outdated_folded: Option<Vec<(String, String)>>,
    /// What the block the editor refused holds that the user can act on, read
    /// at a history revision for one editor ([`diagnose`]); kept while the
    /// block stays unreadable, so the card does not re-read it every frame.
    diagnosis: Option<(Target, u64, Diagnosis)>,
}

/// The status strip's reading of the editor under the pointer: the line it
/// shows, where the pointer is in the editor's own micrometres, and the key of
/// the widget under it (a `hit_rects` key without its `ecad/` panel), `None`
/// while the pointer is off the canvas.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Status {
    pub text: String,
    pub at: Option<[i32; 2]>,
    pub under: Option<String>,
}

/// The pin numbers of the part's own symbol block — what its pads should
/// match, since a part keeps its symbol and its pads together.
pub(crate) fn symbol_pins(history: &History) -> Vec<String> {
    Block::Symbol
        .read(history)
        .and_then(|symbol| symbol.get("pins"))
        .and_then(serde_json::Value::as_array)
        .map(|pins| {
            pins.iter()
                .filter_map(|pin| pin.get("number").and_then(serde_json::Value::as_str))
                .map(String::from)
                .collect()
        })
        .unwrap_or_default()
}

/// The part's own symbol's pins as `(number, name)`, in the symbol's order, so the
/// pads editor's Symbol pins list can say `7 GND` rather than `7`.
fn symbol_pin_names(history: &History) -> Vec<(String, String)> {
    let text = |pin: &serde_json::Value, field: &str| pin.get(field).and_then(serde_json::Value::as_str).map(String::from);
    Block::Symbol
        .read(history)
        .and_then(|symbol| symbol.get("pins"))
        .and_then(serde_json::Value::as_array)
        .map(|pins| pins.iter().filter_map(|pin| Some((text(pin, "number")?, text(pin, "name").unwrap_or_default()))).collect())
        .unwrap_or_default()
}

/// What the part's own pins bind to, as the symbol editor shows it: the
/// kernel's `pin_point_report` and the live hold, carried across in the
/// kernel's own words.
///
/// The editor is handed the ANSWER rather than the rule, because the rule is
/// the kernel's (`brep_kernel::part_pins`) and a second reading of it in the
/// UI is a second reading to keep in step. `None` — a document with no symbol
/// block at all — leaves the editor saying nothing about binding, which is
/// right for a part whose symbol has not been written yet.
pub(crate) fn pin_points(history: &History) -> Option<brep_ecad_egui::PinPoints> {
    use brep_render::brep_kernel::PinPointProblem;
    let report = history.pin_point_report()?;
    Some(brep_ecad_egui::PinPoints {
        bound: report
            .pairs
            .iter()
            .map(|pair| (pair.pin.clone(), pair.point.clone()))
            .collect(),
        problems: report
            .problems
            .iter()
            .map(|problem| {
                // Only the two that name ONE pin are shown beside a pin; the
                // rest concern the symbol, a group or a pad, and go to the
                // panel's summary untagged.
                let pin = match problem {
                    PinPointProblem::RepeatedPin { pin, .. }
                    | PinPointProblem::PinWithoutPoint { pin, .. } => Some(pin.clone()),
                    _ => None,
                };
                (pin, problem.message())
            })
            .collect(),
        hold: history.pin_port_hold().map(str::to_owned),
    })
}

/// Hand the editor the host's copy of its block, if the block moved under it.
/// A block the editor cannot read (a newer eCAD's, or the wrong kind of
/// document) is refused: the editor shows an empty stand-in and
/// [`BlockSync::store`] will not write over the block. Returns the refusal.
fn pull(target: Target, editors: &mut Editors, sync: &mut BlockSync, history: &brep_render::history::History) -> Option<String> {
    let block = sync.pull(history)?;
    let loaded: Result<(), String> = match target {
        Target::Diagram | Target::Pcb => {
            let (editor, kind) = match target {
                Target::Diagram => (&mut editors.diagram, brep_ecad_core::DocumentKind::Wiring),
                _ => (&mut editors.pcb, brep_ecad_core::DocumentKind::Schematic),
            };
            let document = match block {
                None => Ok(brep_ecad_core::Document::new(kind)),
                Some(value) => brep_ecad_core::Document::from_value(value).and_then(|document| {
                    if document.kind == kind {
                        Ok(document)
                    } else {
                        Err(format!(
                            "the {} block holds a {:?} document, not a {kind:?} one",
                            target.workbench_id(),
                            document.kind
                        ))
                    }
                }),
            };
            match document {
                Ok(document) => {
                    editor.set_document(document);
                    Ok(())
                }
                Err(reason) => {
                    editor.set_document(brep_ecad_core::Document::new(kind));
                    Err(reason)
                }
            }
        }
        Target::Symbol => {
            let symbol = match block {
                None => Ok(brep_ecad_egui::SymbolEditor::blank().symbol),
                Some(value) => serde_json::from_value(value).map_err(|e| e.to_string()),
            };
            match symbol {
                Ok(symbol) => {
                    editors.symbol.set_symbol(symbol);
                    Ok(())
                }
                Err(reason) => {
                    editors.symbol.set_symbol(brep_ecad_egui::SymbolEditor::blank().symbol);
                    Err(reason)
                }
            }
        }
        Target::Pads => {
            let footprint = match block {
                None => Ok(Default::default()),
                Some(value) => serde_json::from_value(value).map_err(|e| e.to_string()),
            };
            match footprint {
                Ok(footprint) => {
                    editors.pads.set_footprint(footprint);
                    Ok(())
                }
                Err(reason) => {
                    editors.pads.set_footprint(Default::default());
                    Err(reason)
                }
            }
        }
    };
    let reason = loaded.err()?;
    sync.refuse(reason.clone());
    Some(reason)
}

/// A block the editor refused, read for what the user can do about it: the
/// parts at fault by name, and the rename that frees them, if a rename can.
///
/// eCAD's `validate` says only "Component references must be nonempty and
/// unique", which names neither the part nor a way out, and a sheet that fails
/// it used to open as an empty "Your circuit starts here" (the third eCAD
/// audit, B4). A hand-merged file or an older one can hold two R5s. So the
/// host reads the block's raw JSON — the one thing it has that the refusal
/// does not — and finds them itself.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Diagnosis {
    /// What is wrong, naming the parts: `Two parts are called R5.`
    pub offender: Option<String>,
    /// The references to rewrite, as `(index in components, old, new)`: every
    /// repeat after the first of its name, and every blank one, each to the
    /// next free reference of its prefix — the rule placing a part uses.
    pub renames: Vec<(usize, String, String)>,
}

impl Diagnosis {
    /// The repair button's words: `Rename the second R5 to R6`.
    pub fn repair_label(&self) -> Option<String> {
        match self.renames.as_slice() {
            [] => None,
            [(_, old, new)] if old.trim().is_empty() => Some(format!("Name the unnamed part {new}")),
            [(_, old, new)] => Some(format!("Rename the second {old} to {new}")),
            renames => Some(format!(
                "Rename {}",
                renames
                    .iter()
                    .map(|(_, old, new)| match old.trim().is_empty() {
                        true => format!("the unnamed part to {new}"),
                        false => format!("a repeated {old} to {new}"),
                    })
                    .collect::<Vec<_>>()
                    .join(", ")
            )),
        }
    }
}

/// Read `block` (a sheet's raw JSON, `components[].reference`) for repeated
/// and blank references. Tolerant of anything else being wrong with it: a
/// block too broken to have a component list names no one.
pub(crate) fn diagnose(block: &serde_json::Value) -> Diagnosis {
    let Some(components) = block.get("components").and_then(serde_json::Value::as_array) else {
        return Diagnosis::default();
    };
    let reference = |c: &serde_json::Value| c.get("reference").and_then(serde_json::Value::as_str).unwrap_or("").to_owned();
    let mut used: std::collections::HashSet<String> =
        components.iter().map(reference).filter(|r| !r.trim().is_empty()).collect();
    let mut seen: Vec<String> = Vec::new();
    let mut counts: Vec<(String, usize)> = Vec::new();
    let mut blank = 0;
    let mut renames = Vec::new();
    for (index, component) in components.iter().enumerate() {
        let old = reference(component);
        let repeat = seen.contains(&old);
        if old.trim().is_empty() {
            blank += 1;
        } else if repeat {
            match counts.iter_mut().find(|(name, _)| *name == old) {
                Some((_, count)) => *count += 1,
                None => counts.push((old.clone(), 2)),
            }
        } else {
            seen.push(old.clone());
            continue;
        }
        // The letters of the reference itself, so a second R5 becomes an R;
        // a blank one takes its symbol's prefix, as placing it would have.
        let mut prefix = old.trim().trim_end_matches(|c: char| c.is_ascii_digit()).to_owned();
        if prefix.is_empty() {
            prefix = component["symbol"]["reference_prefix"].as_str().unwrap_or("").trim().to_owned();
        }
        if prefix.is_empty() {
            prefix = "U".into();
        }
        let mut n = 1;
        while used.contains(&format!("{prefix}{n}")) {
            n += 1;
        }
        let new = format!("{prefix}{n}");
        used.insert(new.clone());
        renames.push((index, old, new));
    }
    let number = |n: usize| match n {
        2 => "Two".to_owned(),
        3 => "Three".to_owned(),
        4 => "Four".to_owned(),
        n => n.to_string(),
    };
    let mut lines: Vec<String> = counts.iter().map(|(name, n)| format!("{} parts are called {name}.", number(*n))).collect();
    match blank {
        0 => {}
        1 => lines.push("A part has no reference.".into()),
        n => lines.push(format!("{} parts have no reference.", number(n))),
    }
    Diagnosis { offender: (!lines.is_empty()).then(|| lines.join(" ")), renames }
}

/// `block` with `renames` applied, and nothing else touched: the references
/// are rewritten in the raw JSON, so every other byte of the sheet is as the
/// file held it. Wires, flags and board placements name a component by id,
/// never by reference, so none of them moves.
pub(crate) fn repaired(block: &serde_json::Value, renames: &[(usize, String, String)]) -> serde_json::Value {
    let mut value = block.clone();
    for (index, _, new) in renames {
        if let Some(component) = value.get_mut("components").and_then(|c| c.get_mut(*index)) {
            component["reference"] = serde_json::Value::String(new.clone());
        }
    }
    value
}

/// What the unreadable card and its toast call the block: `The PCB sheet`.
fn block_words(target: Target) -> &'static str {
    match target {
        Target::Diagram => "The wiring diagram",
        Target::Pcb => "The PCB sheet",
        Target::Symbol => "The symbol",
        Target::Pads => "The pads",
    }
}

/// The toast for a block the editor refused: what, who, and the way out.
fn unreadable_notice(target: Target, reason: &str, diagnosis: &Diagnosis) -> String {
    let what = block_words(target);
    match (&diagnosis.offender, diagnosis.repair_label()) {
        (Some(offender), Some(repair)) => format!(
            "{what} could not be read: {offender} It is left as saved. \"{repair}\" on the canvas fixes it."
        ),
        (Some(offender), None) => format!("{what} could not be read: {offender} It is left as saved."),
        _ => format!("{what} could not be read, and is left as saved: {reason}"),
    }
}

/// Store the edit the editor reports, if any, as one undo step of the
/// document, typing in one field coalesced. Returns why it was not stored.
///
/// `opening` is the frame the editor first loaded its block. What it reports
/// THEN is no one's edit: nothing can have been clicked on a canvas not yet
/// drawn, so it is the editor bringing the stored sheet up to the current
/// shape — a board made before the board followed the sheet gets the
/// placements its components have pads for (`board_view`'s `follow_parts`).
/// That is written as part of the document with no undo step: an undo step
/// there was a step no one made, and Ctrl+Z took the parts off the board and
/// left it stale (round seven's lane U, measured on `ecad-sheet.nbrep`).
/// Returns the saved document before and after such a write, for the host to
/// move the clean baseline with it (`Document::absorb_open_write`).
fn store(
    target: Target,
    editors: &mut Editors,
    sync: &mut BlockSync,
    engine: &mut brep_render::engine_state::EngineState,
    opening: bool,
) -> Result<Option<(serde_json::Value, serde_json::Value)>, String> {
    // Nothing to write unless the editor reports a change; only then is its
    // document serialized.
    let taken = (|| Some(match target {
        Target::Diagram => (editors.diagram.take_change()?, editors.diagram.document.to_value()),
        // A part the board seats lands at a free spot of its own
        // (`board_view`'s `follow`, `Document::open_spot`), so the host no
        // longer spreads parts stacked at the origin after the fact.
        Target::Pcb => (editors.pcb.take_change()?, editors.pcb.document.to_value()),
        Target::Symbol => (
            editors.symbol.take_change()?,
            serde_json::to_value(&editors.symbol.symbol).map_err(|e| e.to_string()),
        ),
        Target::Pads => (
            editors.pads.take_change()?,
            serde_json::to_value(&editors.pads.footprint).map_err(|e| e.to_string()),
        ),
    }))();
    let Some((change, value)) = taken else { return Ok(None) };
    let value = match value {
        Ok(value) => value,
        Err(error) => {
            // The sheet the editor holds breaks a rule a stored sheet keeps, so it
            // cannot be written. Keeping it on screen would show an edit the
            // document does not have — and Save would then write the old block
            // with nothing marked dirty (the eCAD workflow audit, issue 3). Put the
            // stored sheet back, so what the canvas shows is what Save writes.
            if let Some(editor) = match target {
                Target::Diagram => Some(&mut editors.diagram),
                Target::Pcb => Some(&mut editors.pcb),
                _ => None,
            } {
                let previous = block_of(target).read(&engine.history).cloned()
                    .and_then(|v| brep_ecad_core::Document::from_value(v).ok())
                    .unwrap_or_else(|| brep_ecad_core::Document::new(editor.document.kind));
                editor.set_document(previous);
                return Err(format!("{error} The edit was taken back."));
            }
            return Err(error);
        }
    };
    // A read-only document (a PLM revision not checked out, or released)
    // writes nothing. What the editor did on opening is kept on screen, as an
    // unlocked open shows it; a user's edit is taken back, and said once.
    if let Some(reason) = engine.history.locked().map(str::to_string) {
        if opening {
            sync.hold_unwritten(&engine.history, value);
            return Ok(None);
        }
        let again = sync.refused_again(&value);
        sync.reload();
        let _ = pull(target, editors, sync, &engine.history);
        return match again {
            true => Ok(None),
            false => Err(format!("the document is read-only: {reason}. The edit was taken back.")),
        };
    }
    let Ok(mut draft) = serde_json::from_str::<serde_json::Value>(&engine.history.request_json()) else { return Ok(None) };
    let before = draft.clone();
    let mut notes: Vec<String> = Vec::new();
    if matches!(target, Target::Diagram | Target::Pcb) {
        let editor = if target == Target::Diagram { &mut editors.diagram } else { &mut editors.pcb };
        let endpoints = engine.wire_harness_report().map(|report| report.endpoints.as_slice()).unwrap_or(&[]);
        match crate::panels::ecad_parts::follow_sheet(&mut draft, block_of(target).key(), &editor.document, endpoints) {
            // A component the assembly cannot place is UNLINKED, not a refusal:
            // the edit stands and the user is told which one and why.
            Ok(unlinked) => notes = sync.unreported(unlinked),
            Err(error) => {
                // A refused edit restores the stored sheet immediately, so the editor
                // never advertises a placement the assembly could not accept.
                let previous = block_of(target).read(&engine.history).cloned()
                    .and_then(|v| brep_ecad_core::Document::from_value(v).ok())
                    .unwrap_or_else(|| brep_ecad_core::Document::new(editor.document.kind));
                editor.set_document(previous);
                return Err(error);
            }
        }
    }
    if opening {
        // The sheet and what it moved in the assembly, adopted together as the
        // document, with no checkpoint.
        let moved = draft != before;
        draft[block_of(target).key()] = value.clone();
        sync.adopt(&mut engine.history, &draft, value)?;
        engine.refresh_board_geometry();
        if moved { engine.rebuild_current_history(); }
        for note in notes { engine.push_notice_as(NoticeSeverity::Warning, note); }
        return Ok(Some((before, draft)));
    }
    sync.store(&mut engine.history, value.clone(), change.coalesce.as_deref())?;
    // The board's 3D bodies follow the block that was just written. A run would
    // rebuild them anyway, and one happens below whenever a PLACEMENT moved —
    // but routing a track moves no placement and runs nothing, and that is
    // exactly the edit a user watches the 3D board for.
    engine.refresh_board_geometry();
    if draft != before {
        draft[block_of(target).key()] = value;
        engine.history.adopt_document(&draft.to_string())?;
        engine.rebuild_current_history();
    }
    for note in notes { engine.push_notice_as(NoticeSeverity::Warning, note); }
    Ok(None)
}

/// The workbench's name as the strip starts its line; PCB's own sheet is its
/// schematic.
fn strip_name(target: Target, editors: &Editors) -> &'static str {
    match target {
        Target::Diagram => "Diagram",
        Target::Pcb if editors.pcb.view == brep_ecad_egui::View::Board => "PCB",
        Target::Pcb => "Schematic",
        Target::Symbol => "Symbol",
        Target::Pads => "Pads",
    }
}

/// What the status strip says about `target`'s editor this frame.
///
/// **On the canvas** it gives the pointer's place in millimetres and names
/// what is under it. Diagram and PCB compose that line themselves
/// (`Editor::status`, with the tool, grid, layer, unrouted count and the net
/// of the pad under the pointer) and the strip shows it as they wrote it,
/// after the workbench's name. Symbol and Pads write none, so the host
/// composes theirs from what they do publish: the view's pan and zoom, which
/// every editor maps the pointer through the same way
/// (`(p − canvas centre − pan) / zoom`, `on_canvas` inverted), and the
/// widgets' own rects, the smallest one under the pointer being the thing it
/// is over (a pin's grab handle over its pin, a silk line's handle over a pad).
///
/// **Off the canvas** the editors' own lines still hold the last coordinates
/// they printed, so the strip does not show them: it says which editor is up
/// and, on a sheet or the board, its tool and zoom as the editor's own line
/// prints them (against 0.012 points per micrometre, `lib.rs` and
/// `board_view.rs`). Symbol and Pads print no zoom anywhere, and open fitted
/// to their drawing rather than at a fixed scale, so the strip gives them none.
fn status_of(
    target: Target,
    editors: &Editors,
    canvas: Option<egui::Rect>,
    pointer: Option<egui::Pos2>,
    drawn: &[(String, egui::Rect)],
    history: &History,
) -> Status {
    let (pan, zoom) = pan_zoom(target, editors);
    let name = strip_name(target, editors);
    let percent = zoom / 0.012 * 100.0;
    let Some((canvas, p)) = canvas.zip(pointer).filter(|(canvas, p)| canvas.contains(*p)) else {
        let text = match name {
            "Diagram" | "Schematic" => {
                let editor = if target == Target::Diagram { &editors.diagram } else { &editors.pcb };
                format!("{name} · {:?} · {percent:.0}%", editor.tool)
            }
            "PCB" => format!("{name} · {percent:.0}%"),
            _ => name.to_string(),
        };
        return Status { text, at: None, under: None };
    };
    let v = (p - canvas.center() - pan) / zoom;
    let at = [v.x.round() as i32, v.y.round() as i32];
    let under = drawn
        .iter()
        .filter(|(_, rect)| rect.contains(p))
        .min_by(|a, b| a.1.area().total_cmp(&b.1.area()))
        .map(|(key, _)| key.clone());
    let text = match target {
        Target::Diagram | Target::Pcb => {
            let editor = if target == Target::Diagram { &editors.diagram } else { &editors.pcb };
            // The sheet spaces its separators wider than the board does; the
            // strip gives both one spacing.
            let words: Vec<&str> = editor.status.split('·').map(str::trim).collect();
            let words = if words.first() == Some(&"PCB") { &words[1..] } else { &words[..] };
            // The board's line names the pad or silk under the pointer itself;
            // the sheet's does not, so the strip adds it.
            let item = match editor.view {
                brep_ecad_egui::View::Board => None,
                brep_ecad_egui::View::Schematic => under.as_deref().map(|key| sheet_words(editor, key)),
            };
            match item {
                Some(item) => format!("{name} · {} · {item}", words.join(" · ")),
                None => format!("{name} · {}", words.join(" · ")),
            }
        }
        Target::Symbol | Target::Pads => {
            let item = under.as_deref().map(|key| item_words(target, editors, key, history)).unwrap_or_default();
            format!(
                "{name} · {:.2}, {:.2} mm{}",
                mm(at[0]),
                mm(at[1]),
                if item.is_empty() { String::new() } else { format!(" · {item}") }
            )
        }
    };
    Status { text, at: Some(at), under }
}

/// Micrometres as the strip prints millimetres: to the hundredth, and never
/// `-0.00` for a pointer a few micrometres left of the origin.
fn mm(micrometres: i32) -> f64 {
    (f64::from(micrometres) / 10.0).round() / 100.0 + 0.0
}

/// The sheet widget `key` in words: a part by its reference and value, a pin
/// as `J1.2`, a label by its name.
fn sheet_words(editor: &brep_ecad_egui::Editor, key: &str) -> String {
    let (kind, id) = key.split_once(':').unwrap_or((key, ""));
    match kind {
        "component" => match editor.document.components.iter().find(|c| c.reference == id) {
            Some(c) if !c.value.trim().is_empty() => format!("{id}, {}", c.value.trim()),
            _ => id.to_string(),
        },
        "pin" => format!("pin {id}"),
        "wire" => format!("wire {id}"),
        "label" => match id.parse::<usize>().ok().and_then(|i| editor.document.labels.get(i)) {
            Some(label) => format!("label {}", label.name),
            None => format!("label {id}"),
        },
        "junction" => "a junction".to_string(),
        _ => key.to_string(),
    }
}

/// The Symbol or Pads widget `key` in words, for the status strip: a pin by
/// number and name, a pad by number and whether the symbol has a pin of that
/// number, a silkscreen line by its index.
fn item_words(target: Target, editors: &Editors, key: &str, history: &History) -> String {
    if let Some(rest) = key.strip_prefix("pin:").or_else(|| key.strip_prefix("unnumbered-pin:")) {
        let (pin, handle) = match rest.rsplit_once(':') {
            Some((pin, handle @ ("tip" | "root"))) => (pin, Some(handle)),
            _ => (rest, None),
        };
        let named = editors
            .symbol
            .symbol
            .pins
            .iter()
            .find(|p| p.number == pin)
            .map(|p| p.name.trim())
            .filter(|name| !name.is_empty());
        let mut words = match (key.starts_with("unnumbered-pin:"), named) {
            (true, _) => "a pin with no number".to_string(),
            (false, Some(name)) => format!("pin {pin} ({name})"),
            (false, None) => format!("pin {pin}"),
        };
        if let Some(handle) = handle {
            words.push_str(&format!(", its {handle} handle"));
        }
        return words;
    }
    if let Some(number) = key.strip_prefix("pad:") {
        return match target == Target::Pads && !symbol_pins(history).iter().any(|pin| pin == number) {
            true => format!("pad {number}, no pin {number} on the symbol"),
            false => format!("pad {number}"),
        };
    }
    if key.starts_with("unnumbered-pad:") {
        return "a mechanical pad, with no number".to_string();
    }
    if let Some(index) = key.strip_prefix("silk:") {
        return format!("silkscreen line {index}");
    }
    key.to_string()
}

/// Where `target`'s editor is looking, as `(pan, zoom)`.
fn pan_zoom(target: Target, editors: &Editors) -> (egui::Vec2, f32) {
    match target {
        Target::Diagram => editors.diagram.pan_zoom(),
        Target::Pcb => editors.pcb.pan_zoom(),
        Target::Symbol => editors.symbol.pan_zoom(),
        Target::Pads => editors.pads.pan_zoom(),
    }
}

impl Viewport {
    /// Draw `target`'s editor into the whole tile and keep it in step with its
    /// block of `doc` — the frame described in the module notes.
    pub fn show_ecad(&mut self, ui: &mut egui::Ui, doc: &mut Document, target: Target) {
        host_frame(&mut self.ecad, &mut self.last_rect, ui, doc, target);
    }

    /// The status strip's line for the eCAD editor in the tile, as of the last
    /// frame it was drawn. The strip is laid out before the tile, so this is
    /// one frame behind the pointer, as every bottom-bar reading of the tile is.
    pub fn show_ecad_status(&self, ui: &mut egui::Ui) {
        let status = &self.ecad.status;
        ui.horizontal(|ui| {
            ui.add_space(6.0);
            let (name, rest) = status.text.split_once(" · ").unwrap_or((status.text.as_str(), ""));
            ui.label(egui::RichText::new(name).strong());
            if !rest.is_empty() {
                ui.label(egui::RichText::new(format!("·  {rest}")).monospace());
            }
        });
    }

    /// The status strip's reading of the eCAD editor, as of the last frame
    /// one was drawn — the `status` of `__brepEcad` and `ecad_state`.
    pub(crate) fn ecad_status(&self) -> &Status {
        &self.ecad.status
    }

    /// The components the out-of-date card listed, as of the last frame an
    /// eCAD editor was drawn — the `outdated` of `__brepEcad` and `ecad_state`.
    pub(crate) fn ecad_outdated(&self) -> &[(String, String)] {
        &self.ecad.outdated
    }

    /// Publish that no eCAD editor is drawn this frame.
    pub(super) fn publish_no_ecad(&mut self) {
        self.ecad.hits.clear();
        self.ecad.status = Status::default();
        self.ecad.outdated.clear();
        self.ecad.label_armed = false;
        self.ecad.publish(None);
    }
}

/// One frame of `target`'s editor in the tile `ui` offers — pull, show, take,
/// store — with the tool card over it. The whole host, apart from the GPU
/// viewport it sits in, so a headless test drives exactly this.
/// Run `draw` with this frame's keyboard input (keys, typed text, copy, cut
/// and paste) out of sight when `hide`, and put the input back exactly as it
/// was afterwards, so what draws later — a modal's Escape, its text field —
/// still has it. Only key events are hidden and none of them can be spent
/// while hidden, so restoring the whole list loses nothing `draw` consumed.
fn keys_hidden<R>(ctx: &egui::Context, hide: bool, draw: impl FnOnce() -> R) -> R {
    if !hide {
        return draw();
    }
    let keyboard = |event: &egui::Event| {
        matches!(
            event,
            egui::Event::Key { .. } | egui::Event::Text(_) | egui::Event::Copy | egui::Event::Cut | egui::Event::Paste(_)
        )
    };
    let all = ctx.input_mut(|i| {
        let all = std::mem::take(&mut i.events);
        i.events = all.iter().filter(|event| !keyboard(event)).cloned().collect();
        all
    });
    let drawn = draw();
    ctx.input_mut(|i| i.events = all);
    drawn
}

pub(crate) fn host_frame(
    host: &mut EcadViewport,
    last_rect: &mut Option<egui::Rect>,
    ui: &mut egui::Ui,
    doc: &mut Document,
    target: Target,
) {
    let block = block_of(target);
    // The frame this editor first loads its block is the document's open (see
    // `store`); the edit key before it is what the tab's dot was read at.
    let opening = !doc.ecad_blocks.iter().any(|sync| sync.block() == block && sync.pulled());
    let key = doc.edit_key();
    let Document { engine, ecad, ecad_blocks, ecad_outdated, ecad_update_requested, .. } = &mut *doc;
    let sync = ecad_blocks
        .iter_mut()
        .find(|sync| sync.block() == block)
        .expect("a document keeps one BlockSync per block");
    // eCAD reports typing in a field on every keystroke, under one key per
    // field and object, and says nothing when the field loses the keyboard.
    // So a run ends HERE, when the focus moves: typing in R1's reference,
    // clicking away and typing there again later is two undo steps, not one
    // (the checkpoint coalesces a repeated key until something breaks it, as a
    // PMI or sheet drag's end does).
    let focus = ui.ctx().memory(|memory| memory.focused());
    if focus != host.focus {
        engine.history.break_coalescing();
        host.focus = focus;
    }
    let was_unreadable = sync.unreadable().is_some();
    let refused = pull(target, ecad, sync, &engine.history);
    // The frame a block first READS is its open: the document's first frame,
    // or the frame a repair (the unreadable card's rename) made a refused
    // block readable. What the editor brings up to date then — a board seating
    // parts that have pads — is part of that open, not a second undo step
    // after the repair's own.
    let opening = opening || (was_unreadable && sync.unreadable().is_none());
    if refused.is_some() || sync.unreadable().is_some() {
        let revision = engine.history.revision();
        if host.diagnosis.as_ref().map(|(t, r, _)| (*t, *r)) != Some((target, revision)) {
            let diagnosis = block_of(target).read(&engine.history).map(diagnose).unwrap_or_default();
            host.diagnosis = Some((target, revision, diagnosis));
        }
    }
    if let Some(reason) = refused {
        let diagnosis = host.diagnosis.as_ref().map(|(_, _, d)| d.clone()).unwrap_or_default();
        engine.push_notice(unreadable_notice(target, &reason, &diagnosis));
    } else if opening {
        // A multi-unit part saved before gates existed is brought to the gate
        // shape as part of opening, with no undo step (`store`), and silently:
        // nothing the user drew moves, and no action would follow a toast.
        match target {
            Target::Diagram => drop(ecad.diagram.adopt_legacy_units()),
            Target::Pcb => drop(ecad.pcb.adopt_legacy_units()),
            Target::Symbol => drop(ecad.symbol.adopt_legacy_units()),
            Target::Pads => {}
        }
    }
    // The pads belong to the part's own symbol, so the pads editor is told its
    // pins — and lists the ones with no pad yet — whenever the document moves.
    if target == Target::Pads && host.pins_at != Some(engine.history.revision()) {
        ecad.pads.set_pin_names(symbol_pin_names(&engine.history));
        host.pins_at = Some(engine.history.revision());
    }
    // And the symbol editor is told what its pins BIND to, for the same
    // reason from the other side: a pin and the part's connection point of
    // that name are one thing, and the editor says which one each pin is.
    if target == Target::Symbol && host.points_at != Some(engine.history.revision()) {
        ecad.symbol.set_pin_points(pin_points(&engine.history));
        host.points_at = Some(engine.history.revision());
    }
    // A selection a host asked for lands HERE, after the pull that would have
    // cleared it — the Qualify panel's jump from a connection point to its pin
    // or its pad, which crosses a workbench switch to get here.
    ecad.apply_focus(target);
    host.hits.clear();
    // A block the editor could not read is not drawn as the editor's empty
    // stand-in, whose first-run card said "Your circuit starts here" over a
    // file that holds a circuit: the tile says it could not be read instead.
    let unreadable = sync.unreadable().map(str::to_owned);
    let mut repair = false;
    egui::containers::panel::CentralPanel::default()
        .frame(egui::Frame::NONE)
        .show(ui, |ui| {
            let rect = ui.available_rect_before_wrap();
            *last_rect = Some(rect);
            host.hits.insert("canvas".into(), rect);
            if let Some(reason) = unreadable.as_ref() {
                let diagnosis = host.diagnosis.as_ref().map(|(_, _, d)| d.clone()).unwrap_or_default();
                repair = host.unreadable_card(ui, rect, target, reason, &diagnosis);
                return;
            }
            // A host modal owns the keys (`modal_open_last_pass`). The
            // editors' own rule stops their shortcuts under one
            // (`keys_reach_editor`'s modal clause), and egui takes the focus
            // from any field under it. This hides the keyboard from whatever
            // else the editor reads a key with: the Net flag window's Enter,
            // which also counts on `lost_focus` — true on the very frame
            // egui takes that field's focus away.
            let ctx = ui.ctx().clone();
            keys_hidden(&ctx, super::modal_open_last_pass(&ctx), || match target {
                Target::Diagram => ecad.diagram.show(ui),
                Target::Pcb => ecad.pcb.show(ui),
                Target::Symbol | Target::Pads => {
                    // These two leave their keys to their host, and the host
                    // gives them every key while their workbench is up — the
                    // pointer need not be over the canvas, since the keys that
                    // matter (R, [ ], F) act on the selection a click in the
                    // Inspector's lists made as often as one on the canvas.
                    // The one exception is a TEXT FIELD with the keyboard, in
                    // the Inspector, a side pane or a window: `run_shortcuts`
                    // itself returns on `text_edit_focused`, the test
                    // `handle_shortcuts` applies to Delete and Escape here. Not
                    // `egui_wants_keyboard_input`: that is ANY focused widget,
                    // and a button tabbed to would take R from the editor while
                    // doing nothing with it.
                    if target == Target::Symbol {
                        ecad.symbol.run_shortcuts(ui.ctx());
                        ecad.symbol.show(ui);
                    } else {
                        ecad.pads.run_shortcuts(ui.ctx());
                        ecad.pads.show(ui);
                    }
                }
            });
        });
    let tile = *last_rect;
    // Each clickable thing, by key, from the editor's own mapping and bounds:
    // what the status strip names under the pointer, and what automation
    // clicks.
    let drawn = match (target, tile) {
        (_, None) => vec![],
        _ if unreadable.is_some() => vec![],
        (Target::Diagram, Some(canvas)) => ecad.diagram.hits(canvas),
        (Target::Pcb, Some(canvas)) => ecad.pcb.hits(canvas),
        (Target::Symbol, Some(canvas)) => ecad.symbol.hits(canvas),
        (Target::Pads, Some(canvas)) => ecad.pads.hits(canvas),
    };
    let pointer = ui.input(|i| i.pointer.hover_pos());
    host.status = match unreadable {
        Some(_) => Status { text: format!("{} · could not be read", strip_name(target, ecad)), at: None, under: None },
        None => status_of(target, ecad, tile, pointer, &drawn, &engine.history),
    };
    host.outdated = match target {
        _ if unreadable.is_some() => Vec::new(),
        Target::Diagram => outdated_components(&ecad.diagram, &engine.history, ecad_outdated),
        Target::Pcb => outdated_components(&ecad.pcb, &engine.history, ecad_outdated),
        Target::Symbol | Target::Pads => Vec::new(),
    };
    if let Some(tile) = tile {
        let board = target == Target::Pcb && ecad.pcb.view == brep_ecad_egui::View::Board;
        let sheet = match target {
            Target::Diagram => Some(&ecad.diagram.document),
            Target::Pcb => Some(&ecad.pcb.document),
            Target::Symbol | Target::Pads => None,
        };
        let rings: Vec<String> = host
            .outdated
            .iter()
            .flat_map(|(reference, _)| ring_keys(sheet, board, reference))
            .collect();
        host.outdated_card(ui, tile, &drawn, &rings, ecad_update_requested);
    }
    if crate::automation::registry::enabled() {
        host.hits.extend(drawn);
    }
    match target {
        _ if unreadable.is_some() => host.label_armed = false,
        Target::Diagram => host.card(ui.ctx(), tile, &mut ecad.diagram),
        Target::Pcb => host.card(ui.ctx(), tile, &mut ecad.pcb),
        Target::Symbol | Target::Pads => host.label_armed = false,
    }
    let opened = match store(target, ecad, sync, engine, opening) {
        Ok(opened) => opened,
        Err(reason) => {
            engine.push_notice(format!("{}: edit not saved: {reason}", target.workbench_id()));
            None
        }
    };
    // The card's repair: the block as the file holds it with the repeated
    // references renamed, written as ONE undo step past `BlockSync` — whose
    // store refuses while the block is unreadable, which is the point of it —
    // so the next frame's pull reads the sheet, and Ctrl+Z brings the file's
    // own block, and this card, back.
    if repair {
        let renames = host.diagnosis.as_ref().map(|(_, _, d)| d.renames.clone()).unwrap_or_default();
        if let Some(block) = block_of(target).read(&engine.history).filter(|_| !renames.is_empty()) {
            let value = repaired(block, &renames);
            block_of(target).write(&mut engine.history, value, None);
        }
    }
    host.publish(Some((target, &*ecad, &*sync, &engine.history, ui.ctx())));
    // What the open wrote is what the file becomes on its next save; one not
    // saved again is brought up the same way the next time it is opened. So
    // it is no unsaved change of the user's.
    if let Some((before, after)) = opened {
        doc.absorb_open_write(&before, &after, key);
    }
}

/// The components on `editor`'s sheet placed from a part in `outdated` — the
/// shell's list of parts-library entries whose saved part moved on since this
/// document took it — as `(reference, part)`.
///
/// A component names its assembly OCCURRENCE, and the occurrence names its
/// part, so the part is read through the outermost occurrence of the chain:
/// a device nested in a sub-assembly is out of date with the sub-assembly this
/// document holds, and updating that is what brings the device along. A legacy
/// component, drawn before components carried an occurrence, is matched by
/// its library key.
fn outdated_components(editor: &brep_ecad_egui::Editor, history: &History, outdated: &[String]) -> Vec<(String, String)> {
    if outdated.is_empty() {
        return Vec::new();
    }
    let part_of: HashMap<String, String> = (0..history.len())
        .filter(|&index| matches!(history.feature_type(index).as_deref(), Some("ACOMP" | "ASSEMBLY COMPONENT")))
        .filter_map(|index| {
            let params = history.feature_params(index)?;
            Some((params["id"].as_str()?.to_owned(), params["partName"].as_str()?.to_owned()))
        })
        .collect();
    editor
        .document
        .components
        .iter()
        .filter_map(|component| {
            let source = component.part.as_ref()?;
            let part = match &source.instance {
                Some(chain) => part_of.get(chain.split(':').next().unwrap_or(chain))?.clone(),
                None => source.key.clone(),
            };
            outdated.contains(&part).then(|| (component.reference.clone(), part))
        })
        .collect()
}

/// The drawn rects that ring out-of-date component `reference`: its courtyard
/// on the board (`part:`), and on a sheet every rect it is drawn as. That is
/// one `component:` rect for a part drawn whole, and one per gate for a part
/// whose gates are placed apart (`component:U1A`, `U1B`, …), since each gate is
/// the part as much as another and a ring on the first alone left the others
/// looking current. The bare `component:U1` of a split part is its first
/// gate's rect, and is not ringed twice.
fn ring_keys(sheet: Option<&brep_ecad_core::Document>, board: bool, reference: &str) -> Vec<String> {
    if board {
        return vec![format!("part:{reference}")];
    }
    match sheet.and_then(|d| d.components.iter().find(|c| c.reference == reference)) {
        Some(c) if !c.gates.is_empty() => {
            c.gates.iter().map(|g| format!("component:{}", c.gate_reference(g.gate))).collect()
        }
        _ => vec![format!("component:{reference}")],
    }
}

impl EcadViewport {
    /// The OUT-OF-DATE mark: a part edited and saved since it was placed (a pin
    /// renumbered, a footprint renamed) reached neither sheet until the user
    /// found Assembly > Constraints > Update components, and nothing in the
    /// Diagram or PCB workbench said so (the eCAD workflow audit, issue 11).
    /// Each component placed from such a part is ringed where it is drawn — its
    /// symbol on a sheet, its courtyard on the board — and a card over the top
    /// middle of the tile names the parts and offers the update in one click.
    /// The click is `UpdateComponents::run`, the Constraints button's own lane,
    /// so it is one undo step and says what it did. Later folds the card to a
    /// one-line chip and changes nothing.
    ///
    /// It is OFFERED, never done on entering the workbench. Opening a workbench
    /// is navigation, and navigation must not change the document: an update
    /// entered that way would put an undo step on the stack the user never
    /// took. A renamed point is matched by POSITION when neither version seats
    /// it (`refresh_snapshots` says so rather than guessing silently), so
    /// applying it unasked would move connections on a guess. And a user who
    /// keeps a board on the version it was laid out with — one already sent to
    /// be made — would have it changed under them each time they looked at it.
    /// The mark costs the user nothing to ignore; an unwanted update costs them
    /// an undo they have to know to make.
    fn outdated_card(
        &mut self,
        ui: &egui::Ui,
        tile: egui::Rect,
        drawn: &[(String, egui::Rect)],
        rings: &[String],
        requested: &mut bool,
    ) {
        if self.outdated.is_empty() {
            return;
        }
        let warn = ui.visuals().warn_fg_color;
        let painter = ui.painter().with_clip_rect(tile);
        for key in rings {
            let Some((_, rect)) = drawn.iter().find(|(drawn, _)| drawn == key) else { continue };
            painter.rect_stroke(rect.expand(3.0), 3.0, egui::Stroke::new(1.5, warn), egui::StrokeKind::Outside);
            let badge = rect.expand(3.0).right_top();
            painter.circle_filled(badge, 7.0, warn);
            painter.text(badge, egui::Align2::CENTER_CENTER, "!", egui::FontId::proportional(11.0), egui::Color32::BLACK);
        }
        let mut parts: Vec<(&str, Vec<&str>)> = Vec::new();
        for (reference, part) in &self.outdated {
            match parts.iter_mut().find(|(name, _)| name == part) {
                Some((_, references)) => references.push(reference),
                None => parts.push((part, vec![reference])),
            }
        }
        let count = self.outdated.len();
        let heading = if count == 1 {
            "1 placed part is out of date".to_owned()
        } else {
            format!("{count} placed parts are out of date")
        };
        // Folded by Later while the list is the one it was folded on; a part
        // that changes again, or another one, opens the card again.
        let folded = self.outdated_folded.as_ref() == Some(&self.outdated);
        let mut fold = None;
        let hits = &mut self.hits;
        // Over the top middle of the tile: the editor writes the sheet's title
        // and its notice line in the top left, and the tool card sits top right.
        egui::Area::new(egui::Id::new("brep-ecad-outdated"))
            .order(egui::Order::Foreground)
            .pivot(egui::Align2::CENTER_TOP)
            .fixed_pos(tile.center_top() + egui::vec2(0.0, 8.0))
            .show(ui.ctx(), |ui| {
                let line = |text: egui::RichText| egui::Label::new(text).wrap_mode(egui::TextWrapMode::Extend);
                let card = egui::Frame::popup(ui.style()).show(ui, |ui| {
                    if folded {
                        let chip = ui
                            .add(egui::Button::new(egui::RichText::new(&heading).color(warn)))
                            .on_hover_text("Show which parts, and the update");
                        if chip.clicked() {
                            fold = Some(None);
                        }
                        return;
                    }
                    ui.add(line(egui::RichText::new(&heading).color(warn).strong()));
                    for (part, references) in &parts {
                        ui.add(line(egui::RichText::new(format!("{part} \u{2014} {}", references.join(", ")))));
                    }
                    let why = if count == 1 {
                        "Its saved part changed after it was placed."
                    } else {
                        "Their saved parts changed after they were placed."
                    };
                    ui.add(line(egui::RichText::new(why).small().weak()));
                    ui.horizontal(|ui| {
                        let update = ui.button("Update parts").on_hover_text(
                            "Take the saved version of each part into this document, as Assembly > Constraints > \
                             Update components does. One undo step.",
                        );
                        if update.clicked() {
                            *requested = true;
                        }
                        hits.insert("outdated:update".into(), update.rect);
                        let later = ui
                            .button("Later")
                            .on_hover_text("Keep the placed version. The parts stay marked on the sheet.");
                        if later.clicked() {
                            fold = Some(Some(()));
                        }
                        hits.insert("outdated:later".into(), later.rect);
                    });
                });
                hits.insert("outdated:card".into(), card.response.rect);
            });
        match fold {
            Some(Some(())) => self.outdated_folded = Some(self.outdated.clone()),
            Some(None) => self.outdated_folded = None,
            None => {}
        }
    }

    /// The UNREADABLE card: the tile of an editor whose block could not be read.
    /// It says so, names the parts at fault and offers the rename that frees
    /// them, where one does, in one click; returns whether it was clicked. It
    /// takes the whole tile, so nothing of the editor's empty stand-in — its
    /// "Your circuit starts here" above all — is drawn over a file that holds
    /// a circuit, and nothing on the canvas can be edited.
    fn unreadable_card(&mut self, ui: &egui::Ui, tile: egui::Rect, target: Target, reason: &str, diagnosis: &Diagnosis) -> bool {
        let visuals = ui.visuals().clone();
        ui.painter().rect_filled(tile, 0.0, visuals.extreme_bg_color);
        let mut clicked = false;
        let hits = &mut self.hits;
        egui::Area::new(egui::Id::new("brep-ecad-unreadable"))
            .order(egui::Order::Middle)
            .pivot(egui::Align2::CENTER_CENTER)
            .fixed_pos(tile.center())
            .constrain_to(tile)
            .show(ui.ctx(), |ui| {
                let card = egui::Frame::popup(ui.style()).inner_margin(16.0).show(ui, |ui| {
                    ui.set_max_width((tile.width() - 48.0).clamp(240.0, 520.0));
                    ui.label(
                        egui::RichText::new(format!("{} could not be read", block_words(target)))
                            .color(visuals.warn_fg_color)
                            .strong()
                            .size(16.0),
                    );
                    ui.add_space(4.0);
                    if let Some(offender) = &diagnosis.offender {
                        ui.label(egui::RichText::new(offender).strong());
                    }
                    ui.label(egui::RichText::new(format!("The file says: {reason}")).weak());
                    ui.add_space(6.0);
                    ui.label(
                        "It is left exactly as saved: nothing here is written over it until it reads. \
                         Save keeps the file as it is.",
                    );
                    if let Some(label) = diagnosis.repair_label() {
                        ui.add_space(8.0);
                        let button = ui
                            .add(
                                egui::Button::new(egui::RichText::new(&label).strong().color(visuals.selection.stroke.color))
                                    .fill(visuals.selection.bg_fill)
                                    .min_size(egui::vec2(0.0, 28.0)),
                            )
                            .on_hover_text("Give each repeated part the next free reference, as placing a part would. One undo step.");
                        if button.clicked() {
                            clicked = true;
                        }
                        hits.insert("unreadable:fix".into(), button.rect);
                    }
                });
                hits.insert("unreadable:card".into(), card.response.rect);
            });
        clicked
    }

    /// The TOOL CARD: the things eCAD's own toolbar held that are not
    /// actions — the Label tool's name, the Power tool's net and a wiring
    /// diagram's stock part for new connections — as a card over the top right of the tile, the way the
    /// sketch's Finish card sits over the 3D view. Not a pane, so nothing the
    /// user closed can hide it; drawn only while it applies. The name is typed
    /// at the moment of placement, so its field takes the keyboard on the frame
    /// the Label tool arms.
    fn card(&mut self, ctx: &egui::Context, tile: Option<egui::Rect>, editor: &mut brep_ecad_egui::Editor) {
        let on_sheet = editor.view == brep_ecad_egui::View::Schematic;
        let label = on_sheet && editor.tool == brep_ecad_egui::Tool::Label;
        let power = on_sheet && editor.tool == brep_ecad_egui::Tool::Power;
        let stock = on_sheet
            && editor.tool == brep_ecad_egui::Tool::Select
            && editor.document.kind == brep_ecad_core::DocumentKind::Wiring;
        let armed = label && !self.label_armed;
        self.label_armed = label;
        let Some(tile) = tile.filter(|_| label || stock || power) else {
            return;
        };
        let hits = &mut self.hits;
        egui::Area::new(egui::Id::new("brep-ecad-card"))
            .order(egui::Order::Foreground)
            .pivot(egui::Align2::RIGHT_TOP)
            .fixed_pos(tile.right_top() + egui::vec2(-8.0, 8.0))
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    if label {
                        ui.label("Label name");
                        let field = ui.add(egui::TextEdit::singleline(&mut editor.label_text).desired_width(160.0));
                        // As the tool arms AND after each placement (whose click
                        // took the keyboard), with the name selected: typing
                        // replaces it, another click repeats it.
                        if editor.take_label_placed() || armed {
                            field.request_focus();
                            let mut state = egui::TextEdit::load_state(ctx, field.id).unwrap_or_default();
                            let end = egui::text::CCursor::new(editor.label_text.chars().count());
                            state.cursor.set_char_range(Some(egui::text::CCursorRange::two(egui::text::CCursor::new(0), end)));
                            egui::TextEdit::store_state(ctx, field.id, state);
                        }
                        hits.insert("label".into(), field.rect);
                    }
                    if power {
                        // The Power tool's net: the four KiCad's power library
                        // is mostly used for, or any name typed.
                        ui.label("Power net");
                        ui.horizontal(|ui| {
                            for name in brep_ecad_egui::POWER_NETS {
                                let pick = ui.selectable_label(editor.power_net_text == name, name);
                                if pick.clicked() {
                                    editor.power_net_text = name.into();
                                }
                                hits.insert(format!("power_net:{name}"), pick.rect);
                            }
                        });
                        let field = ui.add(egui::TextEdit::singleline(&mut editor.power_net_text).desired_width(160.0));
                        hits.insert("power_net".into(), field.rect);
                    }
                    if stock {
                        ui.label("Stock part for new connections");
                        let field = ui.add(
                            egui::TextEdit::singleline(&mut editor.stock_part_number).desired_width(160.0),
                        );
                        hits.insert("stock".into(), field.rect);
                    }
                });
            });
    }

    /// The `__brepEcad` state and the `ecad/…` rects — `target: null` while no
    /// eCAD editor is drawn, so a script never reads a stale editor.
    fn publish(&self, shown: Option<(Target, &Editors, &BlockSync, &History, &egui::Context)>) {
        if !crate::automation::registry::enabled() {
            return;
        }
        let state = match shown {
            None => serde_json::json!({ "target": null }),
            Some((target, editors, sync, history, ctx)) => {
                let mut state = state_json(target, editors, sync, history);
                state["status"] = status_json(&self.status);
                state["outdated"] = outdated_json(&self.outdated);
                keys_json(&mut state, ctx);
                state
            }
        };
        crate::automation::registry::publish("__brepEcad", STATE_DOC, &state.to_string());
        crate::automation::registry::publish(
            "__brepEcadHit",
            "the eCAD editor's widget rects in screen points (canvas; label, stock, and power_net with power_net:<name> per offered power net, while the tool card shows them; outdated:card, outdated:update and outdated:later while a placed part is out of date, outdated:card alone once Later folded it; unreadable:card, and unreadable:fix where a rename frees the parts it names, in place of the editor while its block could not be read; the Symbol and Pads Inspector's inspector: keys while it is drawn, and the sheet's inspector:erc: keys, the Connectivity panel's electrical rule check)",
            &crate::automation::hit_rects::hits_json(&self.hits),
        );
    }
}

/// The out-of-date components as `__brepEcad` and `ecad_state` publish them:
/// each one's reference and the part it was placed from.
pub(crate) fn outdated_json(outdated: &[(String, String)]) -> serde_json::Value {
    outdated
        .iter()
        .map(|(reference, part)| serde_json::json!({ "reference": reference, "part": part }))
        .collect::<Vec<_>>()
        .into()
}

/// The status strip as `__brepEcad` and `ecad_state` publish it: its line,
/// and the pointer's place in micrometres and the key under it (`null` off
/// the canvas).
pub(crate) fn status_json(status: &Status) -> serde_json::Value {
    serde_json::json!({ "text": status.text, "at": status.at, "under": status.under })
}

/// Who has the keys, as the editor's frame ends: a menu or combo box open
/// (`popupOpen`, the popup clause of `keys_reach_editor`), a modal up
/// (`modalOpen`), and the rule's own answer. The canvas's right-click menu has
/// drawn by now, so a script sees it open the frame it opens.
pub(crate) fn keys_json(state: &mut serde_json::Value, ctx: &egui::Context) {
    state["popupOpen"] = egui::Popup::is_any_open(ctx).into();
    state["modalOpen"] = super::modal_open_last_pass(ctx).into();
    state["keysReachEditor"] = brep_ecad_egui::keys_reach_editor(ctx).into();
}

/// What `__brepEcad` and the `ecad_state` command say about an editor.
pub(crate) const STATE_DOC: &str = "the eCAD editor drawn in the central tile: target (diagram|pcb|symbol|pads, null when none); inStep, true when the editor's document is exactly its block of the document (or the empty document when there is no block) — false after ANY edit that did not reach the document or any document change that did not reach the editor; the history revision; pan [x,y] in points and zoom in points per micrometre; why its block is not being edited (unreadable), and while it is not, the parts at fault by name (offender, null when the host cannot name them) and the unreadable card's one-click repair (repair, its button's words, null when there is none); for the sheet editors the view, tool, selection, components, wires (connections, by terminal name), labels, nets (the Connectivity panel's, each net's name and pins), erc (the Connectivity panel's electrical rule check: run, whether its button has been pressed; findings, each with severity, kind, message, at in micrometres, pins and the fix it offers; noConnects and powerFlags, the pins its fixes marked) and board (counts; each placement's pads with their centres on the board; each track's net, layer, width and points in micrometres as trackPaths; each copper zone as zones — its net, layer, outline, and its fill's pieces, holes, vertices, area in mm² and, once filled in this session, the spokes each relieved pad got — with zonesStale, true when copper has changed since the fill on show was made, and selectedZone, the selected zone's index or null; the board's selected pad by reference and number, its selected silkscreen line by reference and index, and its message, the Inspector's line from the last sync, autoroute or refused board edit, \"\" once spent); for Symbol its tool (select|line|rectangle|circle|pin|text) and pins; for Pads its tool (select|smd|through_hole|line|rectangle|circle), pads, its silkscreen lines and the selection (a pad by number, a silk line by index); popupOpen, a menu or combo box open as the frame ends (the popup clause of the editors' key rule); modalOpen, a modal up last pass; keysReachEditor, the key rule's answer (no text field focused, no popup, no modal); status, the status strip under the editor: its text, the pointer's place in the editor's micrometres and the hit key under it (both null off the canvas); and outdated, for the sheet editors the components placed from a part whose saved version changed after they were placed, each as its reference and part (what the Update parts card lists and the canvas rings; empty when none)";

/// Whether the editor's document is exactly its block of `history` — the empty
/// document when there is no block. One comparison that fails whatever the
/// route to a desync: an edit not stored, a pull missed after an undo, a store
/// refused, an action that changed the document without reporting it, a store
/// into the wrong block, a second undo history.
pub(crate) fn in_step(target: Target, editors: &Editors, history: &History) -> bool {
    let block = block_of(target).read(history);
    let (held, empty) = match target {
        Target::Diagram => (
            editors.diagram.document.to_value().ok(),
            brep_ecad_core::Document::new(brep_ecad_core::DocumentKind::Wiring).to_value().ok(),
        ),
        Target::Pcb => (
            editors.pcb.document.to_value().ok(),
            brep_ecad_core::Document::new(brep_ecad_core::DocumentKind::Schematic).to_value().ok(),
        ),
        Target::Symbol => (
            serde_json::to_value(&editors.symbol.symbol).ok(),
            serde_json::to_value(brep_ecad_egui::SymbolEditor::blank().symbol).ok(),
        ),
        Target::Pads => (
            serde_json::to_value(&editors.pads.footprint).ok(),
            serde_json::to_value(brep_ecad_core::board::Footprint::default()).ok(),
        ),
    };
    match (held, block) {
        (Some(held), Some(block)) => &held == block,
        (Some(held), None) => Some(held) == empty,
        (None, _) => false,
    }
}

/// The state a script reads back: a summary in the sheet and PMI blobs'
/// shape, never the whole document.
pub(crate) fn state_json(target: Target, editors: &Editors, sync: &BlockSync, history: &History) -> serde_json::Value {
    use serde_json::json;
    let (pan, zoom) = pan_zoom(target, editors);
    let mut state = json!({
        "target": target.workbench_id(),
        "inStep": in_step(target, editors, history),
        "revision": history.revision(),
        "pan": [pan.x, pan.y],
        "zoom": zoom,
        "unreadable": sync.unreadable(),
    });
    // Read afresh from the block rather than from the card's copy, so the
    // `ecad_state` command, which has no viewport, says the same.
    let diagnosis = sync.unreadable().and_then(|_| block_of(target).read(history)).map(diagnose).unwrap_or_default();
    state["offender"] = diagnosis.offender.clone().into();
    state["repair"] = diagnosis.repair_label().into();
    let point = |p: brep_ecad_core::Point| json!([p.x, p.y]);
    match target {
        Target::Diagram | Target::Pcb => {
            let editor = if target == Target::Diagram { &editors.diagram } else { &editors.pcb };
            let document = &editor.document;
            let reference = |id: &brep_ecad_core::Uuid| {
                document.components.iter().find(|c| &c.id == id).map(|c| c.reference.clone())
            };
            let terminal = |t: &Option<brep_ecad_core::Terminal>| t.as_ref().map(|t| document.terminal_name(t));
            state["view"] = format!("{:?}", editor.view).to_lowercase().into();
            state["tool"] = format!("{:?}", editor.tool).to_lowercase().into();
            state["kind"] = format!("{:?}", document.kind).to_lowercase().into();
            state["labelText"] = editor.label_text.clone().into();
            state["powerNetText"] = editor.power_net_text.clone().into();
            state["stockPartNumber"] = editor.stock_part_number.clone().into();
            state["selection"] = match &editor.selected {
                None => serde_json::Value::Null,
                Some(brep_ecad_egui::Selection::Component(id)) => {
                    json!({ "kind": "component", "id": id.to_string(), "reference": reference(id) })
                }
                Some(brep_ecad_egui::Selection::Wire(id)) => json!({ "kind": "wire", "id": id.to_string() }),
                Some(brep_ecad_egui::Selection::Label(id)) => json!({ "kind": "label", "id": id.to_string() }),
                Some(brep_ecad_egui::Selection::Junction(at)) => json!({ "kind": "junction", "at": point(*at) }),
            };
            state["components"] = document
                .components
                .iter()
                .map(|c| {
                    json!({
                        "id": c.id.to_string(),
                        "reference": c.reference,
                        "value": c.value,
                        "symbol": c.symbol.library_id,
                        "at": point(c.at),
                        "rotation": c.rotation,
                        "pins": c.symbol.pins.len(),
                        // A part split into gates: each gate by the name the
                        // sheet gives it, where it sits and how it is turned.
                        // Empty for a part drawn whole.
                        "gates": c.gates.iter().map(|g| json!({
                            "gate": g.gate,
                            "name": c.gate_reference(g.gate),
                            "at": point(g.at),
                            "rotation": g.rotation,
                        })).collect::<Vec<_>>(),
                    })
                })
                .collect::<Vec<_>>()
                .into();
            state["wires"] = document
                .wires
                .iter()
                .map(|w| {
                    json!({
                        "id": w.id.to_string(),
                        "from": terminal(&w.start),
                        "to": terminal(&w.end),
                        "connectionId": w.connection_id,
                        "stockPartNumber": w.stock_part_number,
                    })
                })
                .collect::<Vec<_>>()
                .into();
            state["labels"] = document
                .labels
                .iter()
                .map(|l| json!({ "id": l.id.to_string(), "name": l.name, "at": point(l.at), "rotation": l.rotation }))
                .collect::<Vec<_>>()
                .into();
            state["junctions"] = document.junctions.len().into();
            // The Connectivity panel's nets, by the names it lists them under,
            // and its electrical rule check: whether it has been run (its rows
            // and the sheet's rings show only then), its findings (the same
            // either way) and the pins its fixes marked.
            state["nets"] = document
                .netlist()
                .nets
                .iter()
                .map(|n| json!({ "name": n.name, "pins": n.pins.iter().map(|p| format!("{}.{}", p.reference, p.number)).collect::<Vec<_>>() }))
                .collect::<Vec<_>>()
                .into();
            let (run, findings) = editor.erc_state();
            let names = |ts: &[brep_ecad_core::Terminal]| ts.iter().map(|t| document.terminal_name(t)).collect::<Vec<_>>();
            state["erc"] = json!({
                "run": run,
                "findings": findings
                    .iter()
                    .map(|f| json!({
                        "severity": format!("{:?}", f.severity).to_lowercase(),
                        "kind": f.kind.key(),
                        "message": f.message,
                        "at": point(f.at),
                        "pins": f.pins,
                        "fix": f.fix.as_ref().map(|fix| fix.label()),
                    }))
                    .collect::<Vec<_>>(),
                "noConnects": names(&document.no_connects),
                "powerFlags": names(&document.power_flags),
            });
            // Every track as drawn: the net its copper island carries (`null`
            // for a track joined to nothing, or bridging two nets), its copper
            // layer (0 is the top), its width and its points, in micrometres.
            // `tracks` stays the count.
            let nets = document.track_nets();
            let net_names: Vec<String> = document.netlist().nets.into_iter().map(|n| n.name).collect();
            let track_paths: Vec<serde_json::Value> = document
                .board
                .tracks
                .iter()
                .map(|t| {
                    json!({
                        "net": nets.get(&t.id),
                        "layer": t.layer,
                        "width": t.width,
                        "points": t.points.iter().map(|p| point(*p)).collect::<Vec<_>>(),
                    })
                })
                .collect();
            state["board"] = json!({
                "layers": document.board.layer_count,
                "placements": document
                    .board
                    .placements
                    .iter()
                    .map(|p| {
                        json!({
                            "reference": reference(&p.component),
                            "at": point(p.at),
                            "bottom": p.bottom,
                            // Each pad's centre ON THE BOARD, through the
                            // placement's own transform (rotation, flip), so a
                            // script can say a track end is exactly on it.
                            "pads": p
                                .footprint
                                .pads
                                .iter()
                                .map(|pad| json!({ "number": pad.number, "at": point(p.transform(pad.at)) }))
                                .collect::<Vec<_>>(),
                        })
                    })
                    .collect::<Vec<_>>(),
                "tracks": document.board.tracks.len(),
                "trackPaths": track_paths,
                "vias": document.board.vias.len(),
                // Each copper zone: its net, layer, outline, and what its fill
                // is — pieces, holes, vertices and area in mm² as stored, and the
                // spokes each relieved pad got in its last fill here (`null`
                // until it has been filled in this session). `zonesStale` says
                // copper has changed since the fill on show was made.
                "zones": document
                    .board
                    .zones
                    .iter()
                    .map(|z| {
                        json!({
                            "net": z.net,
                            "layer": z.layer,
                            "outline": z.outline.iter().map(|p| point(*p)).collect::<Vec<_>>(),
                            "pieces": z.fill.len(),
                            "holes": z.fill.iter().map(|p| p.holes.len()).sum::<usize>(),
                            "vertices": z.fill.iter().map(|p| p.outer.len() + p.holes.iter().map(Vec::len).sum::<usize>()).sum::<usize>(),
                            "area": (z.fill_area() / 1000.).round() / 1000.,
                            "spokes": editor.zone_spokes(z.id).map(|spokes| {
                                spokes.into_iter().map(|(pad, n)| json!({ "pad": pad, "spokes": n })).collect::<Vec<_>>()
                            }),
                        })
                    })
                    .collect::<Vec<_>>(),
                "zonesStale": editor.zones_stale(),
                // The net classes, Default first (the board's own rules), each
                // with its sizes in µm, its name patterns and the nets it holds;
                // `netClassOf` the class every net on the sheet resolves to,
                // `netClassAssigned` only the explicit assignments, and
                // `netClassNet` the net the Inspector's Net classes section is
                // about (the selection's, else the one picked there).
                "netClasses": document
                    .board
                    .rules
                    .classes()
                    .iter()
                    .map(|c| {
                        json!({
                            "name": c.name,
                            "clearance": c.clearance,
                            "trackWidth": c.track_width,
                            "viaDiameter": c.via_diameter,
                            "viaDrill": c.via_drill,
                            "patterns": c.patterns,
                            "nets": net_names
                                .iter()
                                .filter(|n| document.board.rules.class_of(Some(n.as_str())).name == c.name)
                                .collect::<Vec<_>>(),
                        })
                    })
                    .collect::<Vec<_>>(),
                "netClassOf": net_names
                    .iter()
                    .map(|n| (n.clone(), json!(document.board.rules.class_of(Some(n.as_str())).name)))
                    .collect::<serde_json::Map<_, _>>(),
                "netClassAssigned": document.board.rules.net_class_of,
                "netClassNet": editor.net_class_subject(),
                // The design rule check's findings as the Inspector lists them
                // ({kind, message}), `null` until the check has run.
                "findings": editor.board_findings().map(|findings| {
                    findings.into_iter().map(|(kind, message)| json!({ "kind": kind, "message": message })).collect::<Vec<_>>()
                }),
                "selectedZone": editor.selected_board_zone(),
                // The board's selected PAD, by part and number — what a click on
                // `pad:<reference>.<number>` selects. `null` for any other board
                // selection, and on the sheet.
                "selectedPad": editor
                    .selected_board_pad()
                    .map(|(id, number)| json!({ "reference": reference(&id), "number": number })),
                // The board's selected SILKSCREEN line, by part and index — what
                // a click on `silk:<reference>.<index>` selects. `null` for any
                // other board selection, and on the sheet.
                "selectedSilk": editor
                    .selected_board_silk()
                    .map(|(id, index)| json!({ "reference": reference(&id), "index": index })),
                // The line the board's Inspector shows under its tools — what the
                // last sync, autoroute or refused edit said — and "" once the
                // next press on the board has spent it.
                "message": editor.board_message(),
            });
        }
        Target::Symbol => {
            state["tool"] = editors.symbol.tool_name().into();
            state["unitCount"] = editors.symbol.symbol.unit_count.into();
            state["gateCount"] = editors.symbol.symbol.gate_count().into();
            state["pins"] = editors
                .symbol
                .symbol
                .pins
                .iter()
                .map(|pin| json!({ "number": pin.number, "name": pin.name, "at": point(pin.at), "end": point(pin.end), "unit": pin.unit, "gate": pin.gate }))
                .collect::<Vec<_>>()
                .into();
        }
        Target::Pads => {
            state["tool"] = editors.pads.tool_name().into();
            let pads = &editors.pads.footprint.pads;
            state["pads"] = pads
                .iter()
                .map(|pad| json!({ "number": pad.number, "at": point(pad.at) }))
                .collect::<Vec<_>>()
                .into();
            state["pinsWithoutPads"] = symbol_pins(history)
                .into_iter()
                .filter(|pin| !pads.iter().any(|pad| &pad.number == pin))
                .collect::<Vec<_>>()
                .into();
            state["silk"] = editors
                .pads
                .footprint
                .silk
                .iter()
                .map(|line| line.iter().map(|p| point(*p)).collect::<Vec<_>>())
                .collect::<Vec<_>>()
                .into();
            // What a click on `pad:` or `silk:` selected: a pad by its number, a
            // silkscreen line by its index, the identities those keys carry.
            state["selection"] = match (editors.pads.selected_pad(), editors.pads.selected_silk()) {
                (Some(number), _) => json!({ "kind": "pad", "number": number }),
                (None, Some(index)) => json!({ "kind": "silk", "index": index }),
                (None, None) => serde_json::Value::Null,
            };
        }
    }
    state
}


