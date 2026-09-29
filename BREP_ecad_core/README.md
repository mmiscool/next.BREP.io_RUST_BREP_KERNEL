# BREP_ecad_core — electronics documents, with no UI

`BREP_ecad_core` is the document model and the algorithms behind the BREP CAD
application's electronics workbenches: schematic, wiring diagram and board
documents, KiCad 9 symbol and footprint import, connectivity and netlisting, a
board autorouter, design-rule checks, and fabrication output.

It has no UI, no GPU and no CAD-kernel dependency — only `serde`, `serde_json`
and `uuid` — so it suits a command-line tool, a server, or a host with its own
editor. The egui editors over these documents are
[`BREP_ecad_egui`](https://crates.io/crates/BREP_ecad_egui).

## Example

Import a KiCad library, place two resistors, wire them, read the nets back
and save the sheet:

```rust,no_run
use brep_ecad_core::{kicad, Document, DocumentKind, Point};

fn main() -> Result<(), String> {
    // Read a KiCad 9 symbol library and report what could not be imported.
    let text = std::fs::read_to_string("Device.kicad_sym").map_err(|e| e.to_string())?;
    let library = kicad::import_library(&text, "Device")?;
    for warning in &library.warnings {
        eprintln!("import: {warning}");
    }
    let resistor = library
        .symbols
        .iter()
        .find(|s| s.library_id == "Device:R")
        .cloned()
        .ok_or("no Device:R in the library")?;

    // Place two resistors on a schematic sheet (coordinates in micrometres)
    // and join a pin of each with a wire.
    let mut sheet = Document::new(DocumentKind::Schematic);
    let r1 = sheet.place(resistor.clone(), Point { x: 0, y: 0 }, 0);
    let r2 = sheet.place(resistor, Point { x: 10_160, y: 0 }, 0);
    let pin_of = |sheet: &Document, id| {
        let c = sheet.components.iter().find(|c| c.id == id).unwrap();
        c.pin_at(&c.symbol.pins[1])
    };
    let (a, b) = (pin_of(&sheet, r1), pin_of(&sheet, r2));
    sheet.add_wire(a, b);

    // Read the connectivity back out, and save the sheet.
    for net in sheet.netlist().nets {
        let pins: Vec<_> = net.pins.iter().map(|p| format!("{}.{}", p.reference, p.number)).collect();
        println!("{}: {}", net.name, pins.join(", "));
    }
    std::fs::write("sheet.json", sheet.to_json()?).map_err(|e| e.to_string())?;
    Ok(())
}
```

With KiCad's `Device.kicad_sym` this prints `N$1: R1.1`, `N$2: R1.2, R2.2` and
`N$3: R2.1`.

## What is in it

- `Document` / `DocumentKind` — one schematic or wiring-diagram sheet, and its
  editing surface: `place`, `place_part`, `add_wire`, `connect`,
  `set_reference`, `set_net_flag`, `refresh_part`. `History` records edits as
  undoable steps. `to_json` / `from_json` save and load it.
- `Point` — integer micrometres, +Y **down** on a sheet, with quarter-turn
  `rotate`, `offset` and grid `snapped`.
- `Symbol`, `Pin`, `Component`, `Wire`, `Net`, `Netlist` — symbols and their
  pins, placed instances, wires with the edits a router needs, and the
  connectivity read out of a sheet. A wiring diagram also answers a
  point-to-point connection list (`connections`, `connections_csv`).
- `erc` — an electrical rule check over a schematic's nets: KiCad's default
  pin map plus unconnected pins, undriven power inputs, duplicate references
  and dangling wire ends.
- `netlist` — the KiCad `.net` netlist (export version "E") that Pcbnew and
  most KiCad-reading tools take, written byte-stable, and read back.
- `board` — printed circuit board layout: `Board`, pads, footprints,
  placements, tracks, vias, zones, net classes and design rules. Copper
  connectivity comes from geometry, net identity from the schematic.
- `kicad` — import from KiCad 9 symbol and footprint libraries, reporting
  everything it could not read in `ImportReport`.
- `autoroute` — the board autorouter: a `RouteJob` yields a `RouteOutcome` and
  reports progress as it goes.
- `fabrication` — Gerber X2 layers, Excellon drill files and a zip bundle.

Object IDs are random version 4 UUIDs. On `wasm32-unknown-unknown` the
randomness comes from the host: enable the `uuid` crate's `js` feature, or
configure another `getrandom` backend, or creating an object fails.

The autorouter and design checks are compute-heavy. A host that builds this
crate in a debug profile will want `opt-level = 2` for it:

```toml
[profile.dev.package.BREP_ecad_core]
opt-level = 2
```

## Licence

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file`
in the manifest). It is not an SPDX licence: read it before you modify or
redistribute the crate.
