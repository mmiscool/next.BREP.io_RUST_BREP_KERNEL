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
printed on stderr). Every request needs `Authorization: Bearer <token>`, the
token in `~/.config/brep-app/mcp-token` (minted on first start, mode 0600;
`BREP_MCP_TOKEN_FILE` names another file):

```sh
brep-mcp serve --http 8765
```

Without an MCP client:

```sh
brep-mcp shot model.nbrep model.png --view ISO --size 1280x800   # render a document
brep-mcp schema --all                                           # feature catalogue + script format, as JSON Schema
brep-mcp test script.js                                         # play test scripts (.js programs or .json step lists)
brep-mcp render-mesh mesh.json mesh.png --normalize             # CPU mesh rasteriser, no GPU
```

Sessions keep their store, screenshots and logs under the OS temporary
directory unless `--session-root` or `BREP_MCP_SESSION_ROOT` says otherwise.

`serve` keeps durable execution state under a named profile (`headless` by
default). `--instance NAME` selects another profile for a second concurrent
server, `--state-dir DIR` names the journal directory directly, and
`--ephemeral` keeps execution state in memory for this process only.
`--event-journal-dir DIR` opts into a private diagnostic metadata archive.
`brep-mcp --help` and `brep-mcp <command> --help` list every option.

## Execution and compiler setup

`run_script` awaits the final result in the same call, including for clients
that advertise Tasks. Only an explicit `wait_ms` opts into a bounded wait;
`wait_ms: 0` opts into immediate execution status (a Task for capable clients).
Use `script_result` to recover after a disconnect, and reuse the same `key`
with identical input to recover without submitting duplicate work. Conflicting
input under the same key is refused. Execution limits still apply while waiting. Task-based export-option forms also
require explicit `wait_ms: 0`; ordinary awaited scripts should supply the export
format and path directly.

JavaScript is available without an external compiler. TypeScript is unavailable
until Node and the pinned TypeScript 7.0.2 package are installed. Neither is
bundled with BREP, and CAD/JavaScript execution needs neither.
`tools/list` reports availability and a diagnostic in the `run_script` language
schema (`x-brep-typescript`). Available means the compiler execution would use
resolved AND its launcher reported the pinned version when run the way
compilation runs it (probed once per installation), so a wrong Node or a broken
install is reported as unusable rather than advertised.

Provision it once, outside any tool call (it downloads with npm, from `BREP_NPM`
or PATH, into the user-local prefix discovery checks first after PATH):

```sh
brep-mcp typescript provision        # or, from a desktop install: brep-app --provision-typescript
brep-mcp typescript status           # what run_script will use; exits 1 when unavailable
```

Provisioning is the equivalent of
`npm install --prefix "${XDG_DATA_HOME:-$HOME/.local/share}/brep-typescript" --no-audit --no-fund --save-exact --ignore-scripts typescript@7.0.2`
followed by the status check. A running server picks the compiler up on its next
`tools/list`; no restart is needed. Node must be discoverable locally, or set
`BREP_NODE` to its executable path.

`BREP_TSC` and `BREP_NODE` are authoritative overrides, including when invalid;
set them in the server's launch environment when using another location. Without
an override, compiler discovery checks absolute PATH directories, then
`${XDG_DATA_HOME:-$HOME/.local/share}/brep-typescript/node_modules/typescript/bin/tsc`,
the default `~/.local/share/brep-typescript` prefix, npm prefixes from
`npm_config_prefix`/`NPM_CONFIG_PREFIX`, `~/.local`,
`~/.npm-global`, and up to 32 installations under
`${NVM_DIR:-$HOME/.nvm}/versions/node`. Windows also checks `%APPDATA%/npm`.
Incompatible automatic candidates are skipped in favor of a compatible one;
the pinned version policy is unchanged. Node is resolved from PATH or these
user-local prefixes, preferring the discovered npm-global compiler's Node
installation when PATH has none. Discovery never searches the current project
or CAD document directory, sources shell profiles, or runs a package manager.
Paths containing spaces are supported. No tool call downloads a compiler. TypeScript is strictly checked
against live generated declarations before any user code runs; missing,
incompatible or unusable compilers refuse execution. There is no unchecked
JavaScript fallback.

HTTP uses a local bearer credential, **not OAuth or Dynamic Client Registration**.
Configure the client's bearer-token environment setting from the credential file;
do not paste tokens into committed configuration, logs or screenshots. Stdio
uses the local process boundary and needs no HTTP Authorization header. Start
only one server per port: `brep-mcp serve --http 8765` drives a private headless
host; `brep-app --mcp` deliberately drives the visible app instead.

Read live discovery and generated declarations before modeling. `cad.*` binds
operations to document identities and revisions; broader `brep.*` functions
usually act on the active tab. Check the selected document before each active-tab
mutation. A waited feature/history response now places the completed report and
step at the top level as well as in `after`; `wait: false` retains submission
semantics. Transport success is not geometric success: inspect `report.error`, `featureErrors`,
validity, bounds and volume.
Mesh reconstruction keeps its existing `report.meshImport: "submitted"` marker;
that marker is not geometric success. Inspect `after` and application notices
for its completed result or reconstruction failure.

`--session-root` (or `BREP_MCP_SESSION_ROOT`) controls session artifacts for both
hosts. Configure it before the desktop's JavaScript editor initializes its
embedded session; a later attempt to relocate that live session is refused.
Document stores and persistent execution journals have distinct configuration.
`session_info.host.width/height` are current logical points; `ppp` is physical
pixels per point. The current frame uses the same units. Screenshot width/height
are physical pixels from that capture; use its scale and crop origin when
converting coordinates, especially after resizing or changing display scale.

## Reviewed skills on disk

The eight embedded workflows can also be exported and installed as Markdown.
Use a trusted `brep-mcp` binary to export its reviewed bundle and obtain its
SHA-256, then pass that digest explicitly when installing:

```sh
brep-mcp skills bundle --output ./brep-skills.json
brep-mcp skills install --bundle ./brep-skills.json --sha256 <printed-sha256> --directory ./managed-skills
brep-mcp skills inspect --directory ./managed-skills
```

`fetch` aliases `bundle`; `update` aliases `install`. All commands work offline.
Bundle export refuses to overwrite an existing file. Install/update accepts only
this binary's exact embedded bundle, with the matching supplied digest. A digest
or publisher label from an untrusted download is not a provenance guarantee.

The destination is a dedicated managed directory whose parent already exists.
The installer uses an exclusive lock, verifies the eight skill files and selects
a digest-named version. It rejects modified files, symlinks, unexpected assets
and executable data files. It never runs skill content or changes agent settings.
Point your agent's skill loader at the returned `directory` path. After an update
or rollback, use the newly returned path; an agent pinned to an older version
will continue reading that version until its configuration changes.

Updating with a newer trusted binary retains the previous version. To return to
it, use the `previous` digest reported by inspection:

```sh
brep-mcp skills rollback --directory ./managed-skills --sha256 <previous-sha256>
```

Rollback verifies both versions before replacing the selection. Reinstalling the
current version preserves rollback history. Do not edit installed version trees;
corruption is reported instead of silently overwritten. A failure before selection
replacement leaves the prior selection intact. If durability syncing fails after
replacement, the error says the selection changed; inspect before retrying.
The filesystem behavior is qualified on Linux. Content integrity does not
establish comparative model effectiveness or create an OS sandbox.

## The tools

An MCP client lists six tools: `run_script`, `script_result`,
`script_halt`, `screenshot`, `input` and `files`. Everything else the application can
do is a function a program calls, `await brep.<group>.<tool>(args)`. Those
functions are generated from the application's own registries, so they are
exactly what it can do: about 300 tools, typed in the server's
`brep://cad/sdk.d.ts` resource (`brep-mcp schema --write <dir>` writes the same
`brep-cad.d.ts`).
The `session` group is the server's own (a program runs inside a session).
File management uses one `files` tool with `op`: `list`, `stat`, `read`,
`write`, `mkdir`, `copy`, `move`, or `delete`. Scripts use the same operations
as `await brep.files.read({root: "documents", path: "note.txt"})` (without `op`).
`documents` and `workspace` name the app's document store. Native `artifacts`
is scoped to the running execution; direct tool calls address the parent area
and can use an execution ID as a path prefix. Native hosts can configure extra
roots with `BREP_MCP_FILE_ROOTS='{"assets":"/absolute/asset/directory"}'`.
The browser supports `documents` and `workspace` in its persistent store;
`artifacts` and host-configured roots are unsupported there.

All paths are relative to a named root, with single separators and no trailing
slash. Absolute paths, dot components,
escaping symlinks, devices and sockets are refused. Reads/writes transfer at
most 256 KiB per call; use `offset`/`length` for bounded reads and `base64`
encoding for binary data. Lists page at most 256 entries. Copies are limited
to 32 MiB per file, 128 MiB per tree, 1024 entries and depth 64.
Replacement requires `overwrite: true`; directory copy and nonempty directory
deletion require `recursive: true`. The aggregate tool is conservatively
marked destructive; individual read methods are marked read-only. Browser
writes report success after persistence settles. File paths are separate from
opaque `cad.artifacts` handles.

The groups:

- **session** — `session_info`, `session_record` and `session_script`.
  Lifecycle changes are host-controlled and are not callable inside a program.
- **document** — new, open, save, activate and close documents;
  `document_import` (STEP, IGES, STL, OBJ, 3MF) and `document_export` (native,
  STEP, STL, OBJ, IGES, binary glTF, 3MF, flat-pattern DXF/SVG, drawing
  SVG/PDF and PDF with an embedded 3D model, PCB fabrication files, BOM
  CSV/JSON); part attributes, families and templates.
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
- **cam** — CAM setups, tools, operations, toolpath generation, simulation
  and post-processing.
- **instructions** and **simulation** — manufacturing instructions (steps,
  fields, playback, export) and motion studies (motors, clips, playback,
  export).
- **plugins** — the JavaScript editor window, installed plugins, plugin
  features, panels, annotations and actions.
- **plm** — PLM follow and bake actions for an app connected to a PLM server.
- **files** — the `files` operations above, one function per operation.

`brep-mcp schema --write <dir>` writes the full tool and feature reference
(Markdown and JSON) into a directory.

## Feature flags

None.

## Licence

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file`
in the manifest). It is not an SPDX licence: read it before you modify or
redistribute the crate.
