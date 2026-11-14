# BREP_app

The BREP CAD application: one Rust egui/wgpu codebase for native desktop and
WebAssembly. Package `BREP_app`, library `brep_app`, version 0.20.0.
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

For independent automation with no window or focus dependency, install
`BREP_mcp` and launch `brep-mcp serve` (stdio) or `brep-mcp serve --http PORT`.
HTTP requires the bearer token in the app's configuration directory, normally
`~/.config/brep-app/mcp-token`, or `BREP_MCP_TOKEN_FILE`. It does not use OAuth
or Dynamic Client Registration. Configure the client through an environment
credential setting without printing or committing the token.

On Linux, `cargo install BREP_app` installs an executable on PATH. Then run
`brep-app --install-file-associations` to register the desktop entry; launch it
with `gtk-launch brep-app` in the same desktop environment. The PLM download
installer (`install/install.sh --prefix DIR URL`) performs both steps and sets
mode 0755 after verifying the download. Copy a manually built binary with
`install -m 755 SOURCE DESTINATION` so the executable mode is explicit.

Desktop records live in `$XDG_DATA_HOME/applications` (default
`~/.local/share/applications`). Installation and launch must use the same XDG
location; an existing file elsewhere does not establish launcher discovery.
The generated Exec argument quotes the actual executable path, including spaces.
Avoid `%` in the installation path: some GLib desktop launchers check executable
existence before expanding escaped percent characters and cannot discover that entry.
Desktop launch starts the ordinary GUI, like `brep-app`, without implicitly
starting another MCP listener. To attach to that window, use the **Start MCP
server** button in its Info window. Use `brep-app --mcp --session-root DIR` for a CLI-launched window
host and configure artifacts before opening the JavaScript editor. Do not
launch another MCP host on the same port.

## Command line

`brep-app --help` prints the full list. The main switches:

- `brep-app FILE` opens a part (`.nbrep`), family seed (`.fbrep`) or template
  (`.tbrep`) in a tab; a STEP, IGES, STL, 3MF, OBJ or KiCad file is imported
  into a new untitled tab.
- `--mcp [--mcp-port PORT] [--session-root DIR]` embeds the MCP server (default
  port 8765); `--mcp-instance NAME`, `--mcp-state-dir DIR` or `--mcp-ephemeral`
  choose where its execution journal lives.
- `--provision-typescript` and `--typescript-status` install or report the
  pinned TypeScript compiler used for TypeScript scripts.
- `--kicad-library` opens the KiCad library importer instead of the CAD window.
- `--install-file-associations` / `--uninstall-file-associations` register or
  remove this executable as the opener for BREP documents.
- `--plm-url URL`, `--plm-token-file FILE`, `--plm-configure URL`,
  `--plm-forget`, `--plm-show` and `--plm-check` connect to and configure a
  [`BREP_plm`](https://crates.io/crates/BREP_plm) server.
- `--bake-worker [--once | --poll SECS]` runs as the PLM server's headless
  bake worker, with no window.
- `--version` prints the version.

## Touch navigation in the 3D view

Drag with one finger to orbit; taps select. Existing single-finger interactions
with sketch tools and gizmos still apply. Drag with two fingers to pan, and
pinch or spread to zoom around their midpoint; pan and zoom work together.
Both fingers must start in the viewport, clear of floating UI. Lift all fingers
before returning to single-finger interaction. Two-finger navigation also works
in a locked sketch view without changing its orientation.

## JavaScript editor

Choose **JavaScript** from the File menu (under **Settings**, beside
**Plugins**) to edit and run code in the active document.
The editor starts with the saved draft, or a box example when there is none.
**Run script** executes the code; console output, notifications, returned JSON,
and errors appear below the editor. **Cancel** stops a running script.
**Save draft** keeps the source locally, including in PLM mode. **Export .js…**
saves the source as a file on desktop and in the browser; **Open .js…** loads a
file in the browser.

```js
for (let i = 0; i < 3; i++) {
  cad.document.addFeature("P.CU", { sizeX: 10 + i * 5, sizeY: 20, sizeZ: 10 });
}
console.log("Added three boxes");
return { featuresBefore: cad.document.features.length };
```

`cad.document` provides the history snapshot and `addFeature(type, params)`,
`updateFeature(id, params)`, and `deleteFeature(id)`; these document commands
apply in order after the script returns. `cad.selection` provides the selection
snapshot. `ui.notify(text)` displays a message in the output.
Scripts can use JavaScript for calculations, loops, and inspecting the snapshot
without changing a model. Feature types and parameters use the same schemas as
the modeling UI; `P.CU`, `P.S`, and `P.CY` create boxes, spheres, and cylinders.

With the default features, Run sends the script to the application's own
script runner: on desktop the embedded MCP server's code runner, in the browser
the page's Web Worker runner. The script body may `await`, and
`await cad.brep.<group>.<tool>(args)` calls any application tool, the same
functions an agent's `run_script` gets. A script that starts with
`import { brep } from "@brep/cad"` and `export default async function main()`
runs as written. A run's edits form one undo step; a failed or cancelled run is
rolled back. Run executes again each time it is pressed. Source is kept outside
the model; generated built-in features save and replay normally. The JavaScript
plugin API provides replayable custom geometry features for more advanced
generators.

## Feature flags

- `mcp` (default) — natively, the embedded MCP server behind `--mcp`; in the
  WebAssembly build, the WebMCP tool host (see below).
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
| `src/workbench/` | Workbench definitions (All, Modeling, Sheet Metal, Wire harness, Assembly, Drawing, Diagram, PCB, Symbol, Pads, CAM, Instructions, Simulation), the PMI command set, feature predicates, contributed actions and panel claims |
| `src/store.rs` | Native filesystem and browser persistence |
| `src/recovery.rs` | Dirty-document autosave and startup recovery |
| `src/automation/` | Command/state/hit-key registries, queue and virtual input |
| `src/mcp.rs` | Native attached-window server host |
| `src/webmcp.rs` | Browser WebMCP tool host |
| `src/javascript.rs` | In-app JavaScript editor |
| `src/plm/` | PLM client: connection, sign-in, change feed and bake worker |

`EngineState` owns the editable history, undo, scene, camera, selections and
editing modes. Panels borrow it and apply intents; they do not keep another
feature list. Native and browser runners evaluate history off the UI thread.

The workbench is chosen from the toolbar's workbench dropdown. PMI is not a
workbench: Modeling, Sheet Metal and Assembly (and All) each show a **PMI**
ribbon tab and the shared PMI panel.

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

The default `mcp` feature embeds `BREP_mcp_core`: natively the whole server
(`brep-app --mcp`); on the web the core without its `server` feature — the
tool set alone, hosted by `src/webmcp.rs` over the page's own platform (the
document store as the file system, blob URLs as the kept screenshots,
downloads as exports) and registered as
[WebMCP](https://webmachinelearning.github.io/webmcp/) tools on
`document.modelContext` (installed by the page from `web/webmcp.js` after
`start`), so a browser's own agent gets the same tools a desktop one does. The crate ships
`LICENSE.md` (the Autodrop3d licence, `license-file` in the manifest) and
`THIRD-PARTY-NOTICES.md`, which covers the four bundled font faces; a native
binary distributed on its own needs both files beside it.
