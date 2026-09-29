//! The eCAD workbenches' documents, kept inside the BREP document.
//!
//! Each eCAD editor edits a copy of one top-level block of its tab's document:
//! Diagram and PCB edit the assembly's `diagram` and `pcb`, Symbol and Pads the
//! part's `symbol` and `pads` (`brep_render::history`). The editor commits
//! every edit at once and reports it through `take_change`; [`BlockSync::store`]
//! writes it into the block as one undo checkpoint, a typing run coalesced.
//! That write is also what lights the tab's unsaved dot and re-arms the
//! autosave, since both key on the history revision it moves
//! ([`super::Document::edit_key`]).
//!
//! When the block moves under the editor — BREP's undo or redo, an import that
//! writes it, a document replaced by a load — [`BlockSync::pull`] hands back
//! the host's copy for the editor's `set_document` (or `set_symbol`,
//! `set_footprint`), which keeps the view and clears the editor's own history.
//! So there is one undo stack, BREP's, and Ctrl+Z reverts an eCAD edit and a
//! feature edit newest first.

use brep_render::history::{History, DIAGRAM, PADS, PCB, SYMBOL};
use serde_json::Value;

/// Which block an editor reads and writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Block {
    /// The assembly's wiring diagram (an eCAD `Document` of kind `Wiring`).
    Diagram,
    /// The assembly's schematic and board (an eCAD `Document`).
    Pcb,
    /// The part's schematic symbol (an eCAD `Symbol`).
    Symbol,
    /// The part's footprint pads (an eCAD `Footprint`).
    Pads,
}

impl Block {
    /// The top-level key in the saved document.
    pub fn key(self) -> &'static str {
        match self {
            Block::Diagram => DIAGRAM,
            Block::Pcb => PCB,
            Block::Symbol => SYMBOL,
            Block::Pads => PADS,
        }
    }

    /// The block as the document holds it; `None` when no editor wrote it.
    pub fn read(self, history: &History) -> Option<&Value> {
        match self {
            Block::Diagram => history.diagram_block(),
            Block::Pcb => history.pcb_block(),
            Block::Symbol => history.symbol_block(),
            Block::Pads => history.pads_block(),
        }
    }

    /// Write the block as one user edit; `coalesce` is eCAD's
    /// `Change::coalesce`, which the setter namespaces by block.
    pub fn write(self, history: &mut History, value: Value, coalesce: Option<&str>) {
        match self {
            Block::Diagram => history.set_diagram_block(Some(value), coalesce),
            Block::Pcb => history.set_pcb_block(Some(value), coalesce),
            Block::Symbol => history.set_symbol_block(Some(value), coalesce),
            Block::Pads => history.set_pads_block(Some(value), coalesce),
        }
    }
}

/// One editor's standing with its block: the value it holds and the history
/// revision it was last in step at, so a frame in which nothing changed costs
/// one integer comparison.
#[derive(Debug)]
pub struct BlockSync {
    block: Block,
    /// The revision the editor was last in step at; `None` before the first
    /// [`Self::pull`], which therefore always loads.
    revision: Option<u64>,
    /// The block value the editor holds, as far as the host knows.
    held: Option<Value>,
    /// Why the editor could not read the block last pulled (a block from a
    /// newer eCAD, say). While set, [`Self::store`] refuses, so the editor's
    /// stand-in document can never be written over the block it failed to read.
    unreadable: Option<String>,
    /// The per-component problems this block last reported ([`Self::unreported`]).
    reported: Vec<String>,
    /// The last edit a read-only document refused ([`Self::refused_again`]).
    refused: Option<Value>,
    /// How many times a read-only document took an edit back ([`Self::reload`]).
    reloads: u64,
}

impl BlockSync {
    pub fn new(block: Block) -> Self {
        Self { block, revision: None, held: None, unreadable: None, reported: Vec::new(), refused: None, reloads: 0 }
    }

    pub fn block(&self) -> Block {
        self.block
    }

    /// Which of `lines` this block has not already reported, remembering the
    /// set for next time.
    ///
    /// An UNLINKED component is a STANDING state, not an event: the fold
    /// reports it on every sheet edit, and one toast per keystroke is noise.
    /// A problem that goes and comes back is reported again, which is right —
    /// it is news both times.
    pub fn unreported(&mut self, lines: Vec<String>) -> Vec<String> {
        let fresh = lines.iter().filter(|line| !self.reported.contains(line)).cloned().collect();
        self.reported = lines;
        fresh
    }

    /// Write an edit the editor reported: `value` is the editor's document
    /// (`to_value`), `coalesce` its `Change::coalesce`. Refused while the
    /// block is [`Self::unreadable`].
    pub fn store(&mut self, history: &mut History, value: Value, coalesce: Option<&str>) -> Result<(), String> {
        if let Some(reason) = &self.unreadable {
            return Err(format!("the {} block was not written: {reason}", self.block.key()));
        }
        self.block.write(history, value.clone(), coalesce);
        self.held = Some(value);
        self.revision = Some(history.revision());
        Ok(())
    }

    /// Whether [`Self::pull`] has run at all: `false` until the editor first
    /// loads this block, which is the document's OPEN as far as the editor can
    /// tell (a document is built with its syncs, and a load builds a document).
    pub fn pulled(&self) -> bool {
        self.revision.is_some()
    }

    /// Write `value` as the block with NO undo step: `document` is the whole
    /// saved document with `value` already in it, adopted as it stands. For
    /// what the editor did to a block the frame it first loaded it — an older
    /// file brought up to the current shape, which is part of opening it and
    /// not an edit anyone made (see `viewport::ecad::store`). Refused while
    /// the block is [`Self::unreadable`], as [`Self::store`] is.
    pub fn adopt(&mut self, history: &mut History, document: &Value, value: Value) -> Result<(), String> {
        if let Some(reason) = &self.unreadable {
            return Err(format!("the {} block was not written: {reason}", self.block.key()));
        }
        history.adopt_document(&document.to_string())?;
        self.held = Some(value);
        self.revision = Some(history.revision());
        Ok(())
    }

    /// The host's copy when the editor does not hold it: `Some(Some(block))`,
    /// or `Some(None)` when the block is absent (never written, or undone past
    /// its first edit), for the editor to load its empty document. `None` when
    /// the editor is in step. The first call always loads.
    ///
    /// Every `Some` clears [`Self::unreadable`], so the caller owes this after
    /// EACH `Some(Some(block))`: try to read it (`from_value`) and call
    /// [`Self::refuse`] if that fails. Refusing once and not again on a later
    /// `Some` lets the stand-in the editor shows be stored over the block.
    pub fn pull(&mut self, history: &History) -> Option<Option<Value>> {
        let revision = history.revision();
        let first = self.revision.is_none();
        if self.revision == Some(revision) {
            return None;
        }
        self.revision = Some(revision);
        let current = self.block.read(history);
        if !first && current == self.held.as_ref() {
            return None;
        }
        self.held = current.cloned();
        self.unreadable = None;
        Some(self.held.clone())
    }

    /// On a read-only document, the editor's first frame brought the block up
    /// to the current shape: keep that on screen as held, unwritten. The saved
    /// bytes do not move (`History::lock`), and the next pull does not undo
    /// what the editor showed on opening.
    pub fn hold_unwritten(&mut self, history: &History, value: Value) {
        self.held = Some(value);
        self.revision = Some(history.revision());
    }

    /// On a read-only document an edit is taken back: the next [`Self::pull`]
    /// loads the stored block again whatever the editor holds.
    pub fn reload(&mut self) {
        self.revision = None;
        self.reloads += 1;
    }

    /// How many edits a read-only document has taken back from this editor.
    pub fn reloads(&self) -> u64 {
        self.reloads
    }

    /// Whether `value` is the edit a read-only document refused last time, so
    /// an editor that reports the same change every frame is told once.
    pub fn refused_again(&mut self, value: &Value) -> bool {
        let again = self.refused.as_ref() == Some(value);
        self.refused = Some(value.clone());
        again
    }

    /// The editor could not read the block [`Self::pull`] just handed it.
    pub fn refuse(&mut self, reason: String) {
        self.unreadable = Some(reason);
    }

    /// Why the block is not being edited, while the editor cannot read it.
    pub fn unreadable(&self) -> Option<&str> {
        self.unreadable.as_deref()
    }
}


