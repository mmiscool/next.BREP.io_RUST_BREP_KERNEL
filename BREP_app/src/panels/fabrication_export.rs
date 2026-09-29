//! The PCB's manufacturing outputs, for the Export dialog and `document_export`:
//! the fabrication bundle (Gerber X2 layers, Excellon drills, pick-and-place,
//! the electronics BOM and a README, in one zip), the two CSVs on their own, and
//! the schematic's KiCad netlist.
//!
//! Everything is written by `brep_ecad_core::fabrication` from the document's
//! `pcb` block, the same block the PCB editor stores every edit into. This module
//! only finds that block and names the files.
use brep_ecad_core::fabrication::Fabrication;
use brep_ecad_core::Document;
use brep_render::engine_state::EngineState;

/// The four outputs, by the id the Export dialog and `document_export` share.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Output {
    /// Every file in one zip.
    Bundle,
    /// The pick-and-place (centroid) CSV.
    PickPlace,
    /// The electronics BOM CSV: references grouped by value and footprint.
    Bom,
    /// The schematic's netlist as KiCad writes one (`.net`), for another tool.
    /// It needs parts on the schematic, not on the board.
    Netlist,
}

impl Output {
    /// The file this output writes for a document named `name`. The CSVs carry
    /// the names they have inside the zip.
    pub fn file_name(self, name: &str) -> String {
        let base = base(name);
        match self {
            Output::Bundle => format!("{base}-fabrication.zip"),
            Output::PickPlace => format!("{base}-pos.csv"),
            Output::Bom => format!("{base}-bom.csv"),
            Output::Netlist => format!("{base}.net"),
        }
    }

    /// What the export's toasts call it: each output names itself, so the
    /// netlist's "exported board.net" is not filed under "Fabrication".
    pub fn label(self) -> &'static str {
        match self {
            Output::Bundle => "Fabrication",
            Output::PickPlace => "Pick & place",
            Output::Bom => "BOM",
            Output::Netlist => "Netlist",
        }
    }
}

/// A document name without the `.nbrep` it can carry when it was opened from
/// a file, so the Gerbers are not called `board.nbrep-F_Cu.gtl`.
fn base(name: &str) -> &str {
    let name = name.trim();
    name.strip_suffix(".nbrep").unwrap_or(name)
}

/// Whether the document has a board with parts on it: what the Export
/// dialog's fabrication row waits for.
pub fn has_board(engine: &EngineState) -> bool {
    engine
        .history
        .pcb_block()
        .and_then(|block| block.get("board"))
        .and_then(|board| board.get("placements"))
        .and_then(|placements| placements.as_array())
        .is_some_and(|placements| !placements.is_empty())
}

/// Whether the document's PCB schematic has parts: what the Export dialog's
/// netlist row waits for.
pub fn has_schematic(engine: &EngineState) -> bool {
    engine
        .history
        .pcb_block()
        .and_then(|block| block.get("components"))
        .and_then(|components| components.as_array())
        .is_some_and(|components| !components.is_empty())
}

/// The document's schematic and board, as the PCB editor stored them.
pub fn board_document(engine: &EngineState) -> Result<Document, String> {
    let block = engine
        .history
        .pcb_block()
        .ok_or("this document has no PCB; draw a schematic and place its parts on the board first")?;
    Document::from_value(block.clone())
}

/// The whole fabrication bundle for a document named `name`.
pub fn bundle(engine: &EngineState, name: &str) -> Result<Fabrication, String> {
    board_document(engine)?.fabrication(base(name))
}

/// One text output: a CSV, or the netlist of a document named `name`.
pub fn text(engine: &EngineState, output: Output, name: &str) -> Result<String, String> {
    match output {
        Output::Netlist => {
            let document = board_document(engine)?;
            document.validate()?;
            document.kicad_netlist(base(name))
        }
        _ => csv(engine, output),
    }
}

/// One CSV on its own.
pub fn csv(engine: &EngineState, output: Output) -> Result<String, String> {
    let document = board_document(engine)?;
    document.validate()?;
    Ok(match output {
        Output::PickPlace => document.pick_and_place_csv(),
        _ => document.electronics_bom_csv(),
    })
}

