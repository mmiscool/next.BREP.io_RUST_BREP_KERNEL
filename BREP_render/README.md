# BREP_render — the wgpu render/pick engine

The windowing-agnostic rendering, picking, and viewer-state engine for the
BREP kernel (publishes on crates.io as **`BREP_render`**, lib `brep_render`).
It runs a kernel feature history in-process, populates a renderer-agnostic
scene store from the resident solids, and draws it with a wgpu 29 core — the
same code backing a headless artifact binary, a native winit window, and a
browser canvas.

## Layering

From `src/lib.rs`, bottom up:

- **`scene`** — the renderer-agnostic scene store (`RenderScene`): display
  objects keyed by the same kernel names the feature pipeline mints (per-face
  triangle ranges + names, edge polylines + names, vertices) with typed
  metadata. No GPU, no window.
- **`pipeline`** — history-request JSON → the kernel's native
  `execute_history` (same process, same thread) → scene population from the
  resident solid handles via the kernel's display accessors. No serialization
  boundary between kernel and renderer. A failed feature still displays what
  the history did build, with diagnostics.
- **`runner`** — the submit/poll boundary around that pipeline. `EngineState`
  defaults to the synchronous `InlineRunner`; native hosts can install the
  persistent `ThreadRunner`, and the browser app supplies its worker-backed
  implementation. The same channel also runs STL/OBJ RANSAC reconstruction
  through `BREP_reconstruction`.
- **`engine_state`** — [`EngineState`], the viewer brain every shell wraps:
  scene + camera + arcball controls + render settings, selection and
  hover emphasis with a selection filter, picking, the editable feature
  **history** (rollback, undo/redo — the single source of truth; UIs keep no
  copy), **sketch mode** (in-viewer 2D sketch editing: input, solving,
  dimensions, inference), widget overlays (transform gizmo, ViewCube, datums,
  feature dimensions via `BREP_gizmos`), expressions, and model I/O. It also owns
  assembly solving/BOM/interference, PMI views and overlays, wire-harness routing
  and the spline anchor editor. No GPU,
  no canvas — it is fully unit-testable on native, and everything crosses its
  boundary as plain JSON and scalars.
- **`render`** — the wgpu core. `RenderCore::render_to_view` draws a scene
  into **any** `wgpu::TextureView` (4x MSAA): shaded faces with per-face
  selection/hover emphasis, screen-constant-width edges with occluded portions
  dimmed rather than dropped, selected-face boundary outlines, vertex point
  sprites, overlays. `render_to_png` adds render-to-texture → readback for
  headless use. Per-solid GPU buffers are retained while a solid's scene
  revision is unchanged, so history reruns don't re-upload unchanged solids;
  the same scene + camera + size + styles produce identical PNG bytes on the
  same device. The WGSL lives in `src/shaders.wgsl` (shipped in the package).
- **`pick`** — CPU ray/screen-space picking over the scene's display buffers,
  returning kernel names with VERTEX > EDGE > FACE priority, CSS-pixel
  thresholds, and a ranked candidate list. Exact, deterministic, and identical
  on native and wasm.
- **`camera` / `controls` / `view`** — pure-math cameras (artifact framing,
  orbit/arcball, ortho/perspective).

The crate re-exports its kernel dependency as `brep_render::brep_kernel`,
so downstream shells can name the run-boundary types (`HistoryRequest`, …)
without taking their own kernel dependency and re-mirroring this crate's
kernel feature split.

## Presentation shells

The render core never owns a window or canvas; the shells are thin:

- **Headless**: `brep-render-artifact` (always built) —
  `brep-render-artifact <history-request.json> <out.png> [width height]` runs
  the whole history natively and writes one framed PNG. History errors are
  reported on stderr but still produce a frame.
- **Desktop** (feature `desktop`): `brep-render-desktop
  <history-request.json>` wraps a winit window + wgpu surface around the same
  `EngineState` + `RenderCore` — orbit/pan/zoom, standard views, the
  ViewCube, and picking all flow through the shared state. Frames are drawn
  on demand (only when the state is dirty). Off by default so the headless
  binary builds without windowing dependencies.
- **Web** (`wasm32` only): the `engine` module is the wasm-bindgen canvas
  shell — attach/resize/event/camera/pick/scene-feed/settings API over a
  WebGPU or WebGL2 device (wgpu's `webgl` feature is enabled). Every
  dependency on the shared path is wasm32-clean.

The interactive application host is the `BREP_app` crate, which embeds this
engine in an egui frame; dependency chain: `BREP_app` → `BREP_render` →
{ `BREP_kernel`, `BREP_gizmos`, `BREP_reconstruction` }.

## Build and test

```sh
cargo build                                # engine + headless artifact binary
cargo run --bin brep-render-artifact -- history.json out.png
cargo build --features desktop             # + the winit desktop shell
```

The crate is standalone (not a workspace member) on purpose: the kernel's own
`Cargo.toml` carries wasm-specific profile settings that a workspace root
would override. Native builds link the kernel with its `parallel`
tessellation feature; the wasm build deliberately does not.

## License and links

- License: the Autodrop3d licence in `LICENSE.md`, shipped in the crate
  (`license-file` in the manifest).
- Version: 0.6.0.
