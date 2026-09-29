# BREP_mcp — drive the BREP CAD application from an agent, with no display

`brep-mcp` runs the real BREP CAD application headlessly (under
[`egui_kittest`](https://crates.io/crates/egui_kittest)) and serves it as a
[Model Context Protocol](https://modelcontextprotocol.io) server, over stdio or
streamable HTTP. An agent — or a script, or a CI job — opens documents, adds
and edits features, moves the pointer, reads state and takes screenshots with
no window and no person at the machine.

The server is [`BREP_mcp_core`](https://crates.io/crates/BREP_mcp_core), the
same one [`BREP_app`](https://crates.io/crates/BREP_app) embeds behind
`brep-app --mcp` to expose a visible window. Use `brep-app --mcp` to drive a
window you are watching, `brep-mcp` for everything else.

## Install

```sh
cargo install BREP_mcp
```

Rendering goes through `wgpu`, so the machine needs a GPU or a software
Vulkan or OpenGL driver; it does not need a display server.

## Quick start

Register it with an MCP client over stdio; for example, in a `.mcp.json`:

```json
{
  "mcpServers": {
    "brep": { "command": "brep-mcp", "args": ["serve"] }
  }
}
```

Or serve streamable HTTP on a loopback port (`0` picks a free one; the URL is
printed on stderr):

```sh
brep-mcp serve --http 8765
```

Without an MCP client:

```sh
brep-mcp shot model.nbrep model.png --view ISO --size 1280x800   # render a document
brep-mcp schema --all                                           # feature catalogue + script format, as JSON Schema
brep-mcp test script.json                                       # play automation scripts
brep-mcp render-mesh mesh.json mesh.png --normalize             # CPU mesh rasteriser, no GPU
```

Sessions keep their store, screenshots and logs under the OS temporary
directory unless `--session-root` or `BREP_MCP_SESSION_ROOT` says otherwise.

## The tools

The tool list is generated from the application's own registries, so it is
exactly what the application can do: about 190 tools, each with a JSON Schema
for its input in `tools/list`. The groups:

- **session** — `session_start`, `session_stop`, `session_restart`,
  `session_info`, and recording a session's calls back out as a script. Each
  session is its own headless application.
- **document** — new, open, save, activate and close documents;
  `document_import` (STEP, IGES, STL, OBJ, 3MF) and `document_export` (native,
  STEP, STL, OBJ, IGES, glTF, flat-pattern DXF/SVG, drawing SVG/PDF); part
  attributes, families and templates.
- **features** and **history** — the feature catalogue and schemas,
  `feature_add`, `feature_set_params`, delete, reorder, roll back, undo and
  redo, expressions.
- **scene** and **camera** — entities, picking, hover, selection, visibility,
  mass properties; `camera_get`, `camera_set`, `zoom_to_fit`,
  `standard_view`.
- **pointer**, **keyboard** and **widgets** — real input through the
  application's own input path: move, press, drag, wheel, keys and text, and
  clicking a named widget.
- **assembly**, **pmi**, **sheets**, **harness**, **ecad** and **metadata** —
  assembly constraints and components, PMI views and annotations, drawing
  sheets, wire harnesses, the electronics workbenches, and custom metadata.
- **state**, **settings**, **shell** and **frame** — the application's
  published state, settings, windows and panes, and frame timing.
- **capture** — `wait_idle`, `wait_frames`, `wait_state` and `screenshot` (the
  whole window, the viewport, or a region, returned inline and saved as PNG).

`brep-mcp schema --write <dir>` writes the full tool and feature reference
(Markdown and JSON) into a directory.

## Feature flags

None.

## Licence

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file`
in the manifest). It is not an SPDX licence: read it before you modify or
redistribute the crate.
