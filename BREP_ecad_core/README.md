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
use brep_ecad_core::{kicad, Document, DocumentKind, PartSource, Point, Uuid};

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

    // Every valued component on a sheet stands for an assembly occurrence: the
    // host names the occurrence and the part (its key and version signature).
    // Coordinates are integer micrometres.
    let part = PartSource { instance: None, key: "Device:R".into(), signature: "1".into() };
    let mut sheet = Document::new(DocumentKind::Schematic);
    let r1 = sheet.place_assembly_component(
        "occurrence-1", part.clone(), resistor.clone(), None, None, Point::new(0, 0), 0,
    )?;
    let r2 = sheet.place_assembly_component(
        "occurrence-2", part, resistor, None, None, Point::new(10_160, 0), 0,
    )?;

    // Join pin 2 of each with a straight wire (`add_wire` takes a horizontal
    // or vertical segment; `connect` routes between two pin terminals).
    let pin_of = |sheet: &Document, id: Uuid| {
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

The components take the next free references (`R1`, `R2`), and the program
prints one line per net with its pins; a net with no label is named `N$1`,
`N$2`, and so on.

## What is in it

- `Document` / `DocumentKind` — one schematic or wiring-diagram sheet (with
  its `board`), and its editing surface: `place_assembly_component` (the one
  door for a valued part: it takes the assembly occurrence the component is),
  `place_power_symbol`, `place_unbound` (a symbol with no part, for previews
  and power symbols), `add_wire`, `connect`, `set_reference`, `set_value`,
  `set_net_flag`, `refresh_part`, `transform_component`, `delete_component`.
  `no_connects` and `power_flags` mark pins meant to be left open and nets
  driven from off the sheet. `to_json` / `from_json` (and `to_value` /
  `from_value` for a document kept inside a host's own JSON) save and load it.
- `History` — snapshot undo/redo, one transaction per gesture; it defaults to
  a `Document` and also serves symbol and footprint editing.
- `Point` — integer micrometres, +Y **down** on a sheet, with quarter-turn
  `rotate`, `offset` and grid `snapped`.
- `Symbol`, `Pin`, `Component`, `Wire`, `Net`, `Netlist` — symbols and their
  pins, placed instances, wires with the edits a router needs, and the
  connectivity read out of a sheet (`Document::netlist`). A wiring diagram
  also answers a point-to-point connection list (`connections`,
  `connections_csv`).
- Gates — a symbol split into separately placed sections (KiCad's units,
  `U1A`, `U1B`): `Pin::gate`, `GatePlacement`, `Document::transform_gate`.
- `erc` — an electrical rule check over a schematic's nets
  (`Document::erc`): KiCad's default pin map plus unconnected pins, undriven
  power inputs, duplicate references, dangling wire ends and unplaced gates;
  findings carry a sheet position and, where one edit answers them, a fix.
- `netlist` — the KiCad `.net` netlist (export version "E") that Pcbnew and
  most KiCad-reading tools take (`Document::kicad_netlist`), written
  byte-stable, and read back (`read_kicad_netlist`).
- `board` — printed circuit board layout: `Board`, pads, footprints,
  placements, tracks, vias, zones, net classes and design rules. Copper
  connectivity comes from geometry, net identity from the schematic.
- `kicad` — import from KiCad 9 symbol libraries (`import_library`, reporting
  what it could not read in `ImportReport::warnings`) and footprints
  (`import_footprint`), and footprint assignment helpers.
- `kicad_git` — shallow clones of KiCad's official symbol, footprint and 3D
  model libraries at a release tag the caller names (`kicad_git::clone`). It
  runs the `git` executable, which must be on the `PATH`.
- `footprint` — natural ordering of pad and pin labels (`natural_cmp`,
  `pin_numbers`).
- `autoroute` — the board autorouter: a `RouteJob` yields a `RouteOutcome` and
  reports progress as it goes.
- `fabrication` — Gerber X2 layers, Excellon drill files and a zip bundle
  (`Document::fabrication`), plus a pick-and-place CSV
  (`pick_and_place_csv`) and an electronics bill of materials
  (`electronics_bom_csv`).

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
