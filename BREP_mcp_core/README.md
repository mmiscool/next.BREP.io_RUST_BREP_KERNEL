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
- `script`, `runner` — JSON automation scripts: steps, waits, assertions and
  screenshots, run against a session.
- `image` — screenshot processing and comparison.
- `render_mesh` — a CPU mesh rasteriser and silhouette comparison that need no
  GPU, window or browser; the `render_mesh` example is its command line.
- `generate` — the generated tool reference.

It depends on [`BREP_render`](https://crates.io/crates/BREP_render), and on
`rmcp` and `tokio`.

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
