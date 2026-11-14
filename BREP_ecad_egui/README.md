# BREP_ecad_egui — embeddable electronics editors for egui

`BREP_ecad_egui` draws the BREP CAD application's electronics editors into any
[`egui`](https://crates.io/crates/egui) `Ui`: schematic and wiring-diagram
sheets, the board, a symbol editor and a pads (footprint) editor, over the
documents of [`BREP_ecad_core`](https://crates.io/crates/BREP_ecad_core).

It is for an application that wants a working schematic or board editor
inside its own window. **The host owns file I/O and the application chrome**:
an editor interprets pointer and key input, edits the `Document` it holds, and
reports that something changed. It opens no files, owns no menus and has no
GPU or CAD-kernel dependency; its only dependencies are `BREP_ecad_core` and
`egui`.

## Example

```rust,no_run
use brep_ecad_core::{Document, DocumentKind};
use brep_ecad_egui::Editor;

/// Held by the host application between frames.
struct SchematicTab {
    editor: Editor,
    dirty: bool,
}

impl SchematicTab {
    fn new() -> Self {
        let mut editor = Editor::default();
        editor.set_document(Document::new(DocumentKind::Schematic));
        Self { editor, dirty: false }
    }

    /// Call once per frame from any egui host (eframe, a game engine, ...).
    fn ui(&mut self, ui: &mut egui::Ui) {
        self.editor.toolbar(ui);
        self.editor.show(ui);
        // The editor never touches the disk: it reports edits, the host saves.
        if self.editor.take_change().is_some() {
            self.dirty = true;
        }
    }

    fn save(&mut self) -> Result<String, String> {
        self.dirty = false;
        self.editor.document.to_json()
    }
}
```

`Editor::library_panel` and `Editor::inspector` draw the parts library and the
selection's properties wherever the host puts them; `Editor::parts` is the
list of parts the host offers to place.

## The contract

- `Editor` — the schematic / wiring-diagram editor. `take_change` returns a
  `Change` once per edit (with a coalescing key for edits typed into one
  field), so the host can fold it into its own undo stack and dirty tracking;
  `undo` / `redo` use the editor's own `History`.
- `Tool`, `Selection`, `Part` — the active tool, what is selected, and a
  placeable part as the editor sees it.
- `Action`, `ActionGroup`, `actions()` — every command the editor can perform,
  named and grouped, so a host builds its toolbar, menus and automation from
  one list rather than hard-coding button ids.
  `Editor::run_action(id)` runs one by id, as a click on its button does.
- `View`, `BoardTool`, `BoardSelection` — the board editor (`Editor::set_view`
  switches between the sheet and the board).
- `Editor::erc_state` — whether the electrical rule check in the editor's
  connectivity panel has run, and its findings.
- `SymbolEditor` and `symbol_actions()` — the symbol editor; `PinPoints`
  hands it the connection point the host binds to each pin, to show beside it.
- `FootprintEditor` and `pad_actions()` — the pads (footprint) editor.
- `power_symbol`, `POWER_NETS`, `is_ground` — the power symbols the Power tool
  places.
- `keys_reach_editor` — whether keyboard input should go to an editor this
  frame (no text field, popup or modal has it).
- `widgets::enum_dropdown` — the dropdown the editors' panels use for an
  enumeration, with its hit rectangles keyed for automation.

The sheet, board, symbol and footprint canvases paint their own dark
background; panels and controls use the host's egui visuals. The editors load
no texture or font of their own.

`BREP_ecad_core`'s autorouter and design checks are compute-heavy; a host
building in a debug profile will want:

```toml
[profile.dev.package.BREP_ecad_core]
opt-level = 2
```

## Feature flags

None.

## Licence

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file`
in the manifest). It is not an SPDX licence: read it before you modify or
redistribute the crate.
