# BREP_app

The BREP CAD application: one Rust egui/wgpu codebase for native desktop and
WebAssembly. Package `BREP_app`, library `brep_app`, version 0.7.0.
The 3D engine is `BREP_render`; geometry comes from `BREP_kernel`.

## Build and run

```sh
cargo install BREP_app
brep-app
brep-app --mcp
```

From a source checkout, `cargo build --release` in this crate builds the same
binary, and `cargo build --release --target x86_64-pc-windows-gnu` cross-builds
it for Windows (the GNU target and the mingw-w64 linker are needed). The
crate's floor is `rust-version = "1.92"`, which is what eframe/egui 0.35
declare. The WebAssembly build is made with `wasm-pack` from a source checkout.

`--mcp` serves the visible native app over loopback HTTP as a
[Model Context Protocol](https://modelcontextprotocol.io) server, through
[`BREP_mcp_core`](https://crates.io/crates/BREP_mcp_core), so an agent can drive
the same window a person is using.

## Touch navigation in the 3D view

Drag with one finger to orbit; taps select. Existing single-finger interactions
with sketch tools and gizmos still apply. Drag with two fingers to pan, and
pinch or spread to zoom around their midpoint; pan and zoom work together.
Both fingers must start in the viewport, clear of floating UI. Lift all fingers
before returning to single-finger interaction. Two-finger navigation also works
in a locked sketch view without changing its orientation.

## JavaScript editor

Choose **JavaScript…** from the toolbar's workbench dropdown (beside **Plugins…**)
to edit and run code in the active document.
The editor starts with a box example. **Run script** executes the statements;
console output, notifications, returned JSON, and errors appear below the editor.
**Save draft** keeps the source locally, including in PLM mode. **Open .js…** and
**Export .js…** exchange source files on desktop and in the browser.

```js
for (let i = 0; i < 3; i++) {
  cad.document.addFeature("P.CU", { sizeX: 10 + i * 5, sizeY: 20, sizeZ: 10 });
}
console.log("Added three boxes");
return { featuresBefore: cad.document.features.length };
```

`cad.document` provides the history snapshot and `addFeature(type, params)`,
`updateFeature(id, params)`, and `deleteFeature(id)`. `cad.selection` provides
the selection snapshot. `ui.notify(text)` displays a message in the output.
Scripts can use JavaScript for calculations, loops, and inspecting the snapshot
without changing a model. Feature types and parameters use the same schemas as
the modeling UI; `P.CU`, `P.S`, and `P.CY` create boxes, spheres, and cylinders.

Execution uses the history worker on desktop and in the browser. Successful
changes create one undo step; failed scripts or failed model recomputes discard
all staged edits. Cancel discards pending results. Run executes again each time
it is pressed. Source is kept outside the model; generated built-in features
save and replay normally. Scripts run synchronously with interpreter loop and
recursion limits; DOM, Node, network, filesystem access, and Promises are
unavailable. The JavaScript plugin API provides replayable custom geometry
features for more advanced generators.

## Feature flags

- `mcp` (default, native only) — the embedded MCP server behind `--mcp`.
- `scripting` (default) — scripts in the application, on the Boa interpreter
  from [`BREP_script_core`](https://crates.io/crates/BREP_script_core), native
  and wasm.
- `automation` — the command registry, queue and virtual pointer an automation
  host drives the application through; `mcp` turns it on.

## Architecture

| Module | Responsibility |
|---|---|
| `src/app.rs` | eframe shell, document lifecycle and action dispatch |
| `src/document.rs` | Open documents, each with one `EngineState` |
| `src/viewport.rs`, `src/viewport/` | Offscreen 3D render, egui composition, input and labels |
| `src/panels/` | History, scene, sketches, files, assemblies, PMI, harness and other panels |
| `src/form.rs`, `src/form_view.rs` | Shared schema fields and complete forms |
| `src/workbench/` | Six definitions, feature predicates, contributed actions and panel claims |
| `src/store.rs` | Native filesystem and browser persistence |
| `src/recovery.rs` | Dirty-document autosave and startup recovery |
| `src/automation/` | Command/state/hit-key registries, queue and virtual input |
| `src/mcp.rs` | Native attached-window server host |

`EngineState` owns the editable history, undo, scene, camera, selections and
editing modes. Panels borrow it and apply intents; they do not keep another
feature list. Native and browser runners evaluate history off the UI thread.

History has a tree and a separate form. The same form view serves features,
assembly constraints, PMI annotations and dialog screenshots. Workbenches filter
creation and panel/action visibility; existing histories remain editable.
PMI additionally activates captured views and restores modeling display state.

Scrollable areas show solid scrollbars whenever their content overflows, without
requiring hover or scrolling first. This applies in both themes on desktop and
in the browser; bars reserve space beside the content and disappear when it fits.

The viewport renders into an app-owned texture on eframe's wgpu device. A paint
callback composites it into the egui frame. Scene-row hover and viewport hover
use the same engine highlighting. Glyphs are SVG artwork from the asset catalogue.

## Files and persistence

Open, Save As, Import and Export use the shared in-app file explorer. Desktop
storage uses real filesystem paths; browser storage uses an IndexedDB mirror
with upload/download bridging. Saved document state and user settings are separate.
Startup offers dirty-document recovery instead of silently reopening an old session.

## Help and distribution

The in-app help is a static site generated from the user documentation by
[`BREP_docs`](https://crates.io/crates/BREP_docs); it closes with a Licences
section (the project licence, the third-party notices, and an inventory of
every crate the application ships, with the font notices that must accompany
the embedded fonts).

The native default `mcp` feature embeds `BREP_mcp_core`; the wasm build uses
its automation layer without the native server dependencies. The crate ships
`LICENSE.md` (the Autodrop3d licence, `license-file` in the manifest) and
`THIRD-PARTY-NOTICES.md`, which covers the four bundled font faces; a native
binary distributed on its own needs both files beside it.
