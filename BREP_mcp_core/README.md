# BREP_mcp_core — the BREP CAD application's MCP server

`BREP_mcp_core` is the [Model Context Protocol](https://modelcontextprotocol.io)
server of the BREP CAD application, as a library that does not depend on the
application. Its tools and schemas are generated from the application's own
command registry, so an agent drives exactly the commands a person can. It is
embedded in [`BREP_app`](https://crates.io/crates/BREP_app) behind
`brep-app --mcp`, which serves the visible window over loopback HTTP.

Hosts implement the protocol in `host`: an *attached* host is a running
application the server lives inside (every session shares it), a *spawning*
host builds an application per session. Either way the same tool, session and
transport code runs. A host also supplies the `host::Platform` the tools touch
the world through — the clock, a pause, files, the pictures a session keeps:
the native platform is the filesystem and tokio; the browser build of the
application is one too (the page's document store, `setTimeout`, blob URLs),
which is how a page serves the same tools over WebMCP.

## What is in it

- `server`, `session`, `tools`, `schema` — the MCP surface: registry-derived
  tools and JSON schemas, sessions, feature validation.
- `http` — the streamable-HTTP transport; stdio is supported too.
- `script`, `runner` — the JSON step-list script format (tool calls with
  `expect` assertions and image capture or comparison) and the runner that
  plays a script against a session and writes per-step artefacts.
- `program` — JavaScript test programs played through the code runner the
  way an agent's `run_script` is, with the same per-step artefacts.
- `script_sdk` — the `brep.<group>.<tool>(args)` functions every host's code
  runner exposes.
- `files` — bounded file operations under named roots, shared by MCP, the
  code runner and the browser store.
- `launch_state` — the native launch policy for execution journals (instance,
  state directory or ephemeral).
- `image` — screenshot processing and comparison.
- `render_mesh` — a CPU mesh rasteriser and silhouette comparison that need no
  GPU, window or browser; the `render_mesh` example is its command line.
- `generate` — registry input/output schemas and TypeScript declarations.
- `cad` — bounded JavaScript/TypeScript workers, semantic CAD operations,
  scoped artifacts, selective function discovery and process-scoped task handles.
- `prompts` — the same eight workflows via `prompts/list` and `prompts/get`.
  Each takes a required `request` string and optional workflow context listed in
  its descriptor. Retrieval returns user-role messages and performs no CAD work.
  The catalogue is static for the server lifetime; it does not advertise change
  notifications. Unknown names, cursors, arguments and non-string values fail.
- `skills` — eight embedded workflows (`inspect-model`,
  `safe-parametric-edit`, `import-step`, `recover-execution`, `edit-assembly`,
  `repair-feature`, `export-model`, `design-part`) exposed through the official Skills
  extension and Resources, with SHA-256 file manifests.
- `skill_distribution` — offline bundle export, explicitly pinned installation,
  verified version selection and rollback for those reviewed Markdown workflows.

The public tool list stays at six entries (`run_script`, `script_result`, `script_halt`,
`screenshot`, `input`, `files`). Scripts discover other functions as
needed. Resources expose function contracts, execution/document snapshots and
bounded artifact chunks. Resource notifications invalidate snapshots and clients
reread them after changes or reconnects. The server supports MCP 2026-07-28
cache metadata, the Skills extension and polling through the Tasks extension;
legacy clients continue using `script_result` and `script_halt`.

Clients can opt into request progress with a `progressToken` when `run_script`
or `script_result` waits for execution. Updates report elapsed request-wait
seconds, once per second, with at most 600 updates and no completion percentage.
They stop when the request finishes or is cancelled; cancelling the wait does
not cancel accepted execution. Use `script_halt` to stop execution. Legacy HTTP
reconnection can replay an identical cached SSE progress event; clients should
deduplicate by event ID. Task polling remains available independently.

The local credential grants the user's application authority. Trusted scripts
can call functions that read and write files. Restricted executions omit that
bridge and check document permissions per capability. Workers are separate
processes, with Linux watchdog measurements and bounded host bridges; they are
not an OS sandbox. Task records, diagnostic events and artifacts use bounded retention. Library hosts select durable execution
admission and result records with `BrepServer::with_state_dir`. The native launch
paths resolve stable profiles: `headless` for `brep-mcp serve` and `desktop` for
the application. Profiles live under the per-user `brep-app/mcp-state` directory.
Use `serve --instance NAME` for another concurrent server, `--state-dir PATH` to
reopen an existing journal in place, or `--ephemeral` for process-only execution
state. An identity already owned by another process produces a startup error.

Launch flags override `BREP_MCP_INSTANCE`, `BREP_MCP_STATE_DIR`, or
`BREP_MCP_EPHEMERAL=true`; environment settings override the corresponding
`headless` or `desktop` policy in the config directory's `mcp-state.json`.
Each layer selects one whole mode. Desktop flags use the `--mcp-` prefix and
machine-local preferences take effect on restart, including editor-first use.
The default config directory follows `XDG_CONFIG_HOME`, Windows `APPDATA`, then
`HOME/.config`, with `brep-app` appended.

An initial durable launch starts a new journal; prior ephemeral executions cannot
be recovered. Interrupted executions require reconciliation and are never
replayed automatically. Artifact bytes, app state and the diagnostic event ring
are not persisted by the execution journal.

It depends on [`BREP_render`](https://crates.io/crates/BREP_render), on
[`BREP_script_core`](https://crates.io/crates/BREP_script_core) for the code
runner (with `server`), and on `rmcp` and `tokio`.

## Native TypeScript toolchain

User JavaScript runs in Boa. TypeScript additionally needs Node and exactly
TypeScript 7.0.2. Set `BREP_NODE` to the Node executable and `BREP_TSC` to the
TypeScript package's JavaScript `bin/tsc` launcher, or expose `node` and that
`tsc` on PATH. If an installation exposes a shell shim, point `BREP_TSC`
directly at the package launcher. Nothing is installed automatically:
`cad::code::provision_typescript` (the `brep-mcp typescript provision` and
`brep-app --provision-typescript` commands) installs the pinned package with
npm into `cad::code::typescript_prefix`, and `cad::code::typescript_status` is the
discovery-plus-launcher-probe check the server advertises.

Node starts the compiler launcher (which may start its native compiler), with
one strict check-and-emit invocation using `--noEmitOnError`. Compiler time,
memory and output limits still apply. Only successfully checked JavaScript
is passed to Boa; Node never runs user programs. Bun is not required.

## Feature flags

- `server` (default): the server — the protocol over `rmcp`, the two
  transports, the runner, the generated docs, the PLM fixture and the native
  `Platform`. Without it the crate is the tool set alone over the host
  protocol (`tokio` with `sync` only), for a host that supplies its own
  platform; that build compiles for `wasm32-unknown-unknown`.

## Licence

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file`
in the manifest). It is not an SPDX licence: read it before you modify or
redistribute the crate.
