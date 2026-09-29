# BREP_gizmos — in-scene gizmos and overlay widgets

Pure-geometry gizmos and overlay widgets for the BREP CAD stack (publishes on
crates.io as **`BREP_gizmos`**, lib `brep_gizmos`): the transform gizmo,
ViewCube, datum/axis visuals, dimension leaders, and curve display used by the
`BREP_render` engine's overlay pass.

A gizmo here is **geometry plus hit-testing, nothing else**. It consumes a
[`GizmoCamera`] (the render engine's view state, mirrored with no engine
dependency) and emits an [`Overlay`] of colored line segments and triangles in
world space; interactive gizmos also answer `Gizmo::hit` (which handle is
under a screen point) and compute drag deltas from pick rays. The crate has
**no GPU, kernel, or windowing dependency** — its only dependency is `png`,
used by the CPU rasterizer for headless demos — so it compiles fast and can be
developed and verified in isolation. The render engine converts the emitted
`Overlay` into its own overlay vertex buffers and draws it over the shaded
solids.

## The contract

Defined in `src/lib.rs`:

- `trait Gizmo` — `geometry(&camera, hovered, active) -> Overlay` and
  `hit(&camera, screen) -> Option<HandleId>`.
- `GizmoCamera` — view-projection matrix, eye, forward, true up, viewport
  size, and projection kind, plus the helpers gizmos build on:
  `world_to_screen`, `world_per_pixel` (screen-constant handle sizing), and
  `ray_from_screen` (perspective and orthographic pick rays).
- `Overlay` — accumulated world-space `lines` (segment pairs) and `tris`
  (flat-shaded triples), both with linear-space RGBA colors.
- `HandleId` — an opaque `u32` token returned by `hit` and echoed back by the
  host's interaction layer to start a drag (`0` = body / no specific handle).

Conventions (shared with the engine): right-handed world space; column-major
4x4 world-to-clip matrices with z in 0..1, byte-identical to the engine's
`Camera::view_proj`; screen coordinates top-left origin, y down, in CSS
pixels.

## Modules

- `transform` — the move + rotate gizmo: 3 axis translate arrows, 3 planar
  translate quads, 3 rotation rings, and an optional center free-move handle.
  It holds no interaction state; the host feeds the feature frame
  (`TransformGizmo::set_frame`) and start/current pick rays, and reads back
  frame-space drag deltas (`axis_translate`, `plane_translate`, `ring_rotate`,
  `center_translate`).
- `view_cube` — the orientation cube that mirrors the main camera (via the
  camera's true up, not a forward-derived heuristic). Its 26 cube hit regions
  (6 faces + 12 edges + 8 corners) decode directly to the standard view
  direction and up hint the camera should snap to (`target_view` /
  `target_up`). Six additional screen-fixed handles apply relative pan/orbit
  or roll rotations; face letters are stroked as overlay line glyphs, no text
  rendering involved.
- `datum` — builders for datum display geometry: `datum_plane` /
  `datum_plane_screen` (world-sized or screen-constant plane cards),
  `datum_axis` (axis line with arrowhead), `datum_frame` (the screen-constant
  X/Y/Z triad), and `world_axes`, each returning an `Overlay` plus hit-testing
  where the visual is pickable.
- `dimension` — CAD dimension leader geometry: `linear_dimension`,
  `angular_dimension`, and `radial_dimension` build the witness/extension
  lines, dimension line, arc, and arrowheads, and return a `label_anchor`
  world point where the host renders the numeric text (text itself is drawn by
  the host, never here).
- `curve_display` — turns already-sampled curve polylines into overlay
  segments (`polyline_display`), provides a parametric helix fallback
  (`helix_display` / `helix_points`), and optional spline control-point
  handles with hit-testing (`control_point_handles` / `hit_control_point`).
  No curve math beyond the helix — the engine samples kernel curves.
- `raster` — a small CPU rasterizer (z-buffered flat-lit triangles,
  screen-space-width lines, PNG output) so a gizmo can be eyeballed headless
  during development. Not a production render path.
- `math` — minimal `Vec3`/`Ray` math, no external math dependency.

## Headless demos

Each demo renders a sample scene through the CPU rasterizer and writes a PNG:

```sh
cargo run --bin gizmo-demo       # transform gizmo
cargo run --bin viewcube-demo    # orientation cube (with a hovered face)
cargo run --bin datum-demo       # datum plane + triad + axis
cargo run --bin dimension-demo   # linear/angular/radial leaders + spline
```

Each takes an optional output path argument.

## Place in the crate family

`BREP_app` → `BREP_render` → { `BREP_kernel`, **`BREP_gizmos`**,
`BREP_reconstruction` }.
`BREP_render` consumes this crate's `Overlay` output for its overlay pass; the
gizmos themselves never touch the kernel or the GPU.

- License: the Autodrop3d licence in `LICENSE.md`, shipped in the crate
  (`license-file` in the manifest).
- Version: 0.2.4.
