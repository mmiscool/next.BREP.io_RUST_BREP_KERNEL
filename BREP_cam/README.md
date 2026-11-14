# BREP_cam

`BREP_cam` is the CAM engine of the BREP CAD kernel. Its Rust library target is
`brep_cam`. It is pure Rust and builds for native targets and for
`wasm32-unknown-unknown`, so the same code generates toolpaths in the desktop
application and in the browser worker.

All algorithms work in machine coordinates with the cutter axis along `+Z`;
the setup transform is the only place model coordinates become machine
coordinates. Every strategy emits the same typed motion stream
(`model::Toolpath`), and the same inputs produce a byte-identical stream.

## The shared contract

- `model`: setups (rigid model-to-machine transform, work offset, stock,
  clearance heights), machines, tools and holders, operations with one typed
  parameter set per strategy (`Strategy`), target references by stable
  body/face identity, and the toolpath as an ordered stream of typed motions
  with per-operation diagnostics.
- `numeric`: the single tolerance object all algorithms read (`Tolerance`).
- `mesh`: the immutable triangle snapshot that keeps each triangle's source
  body and face (`MeshSnapshot`, `MeshBuilder`).
- `index`: the triangle spatial-index trait and its implementations.
- `cutter`: cutter profiles (flat, ball, bull, cone/V, ball-cone, frustum,
  lollipop, dovetail) with validated dimensions.
- `cancel`: cancellation and progress reporting at bounded work chunks
  (`CancelToken`, `Runner`).
- `error`: the crate error type, `CamError`.

## Generating toolpaths

- `dispatch`: one entry point over every strategy. `dispatch::generate`
  (and `generate_with_context`, `generate_expanded` for operation patterns)
  builds each strategy's inputs from the operation, setup, tools and mesh
  snapshot and returns the operation's `Toolpath`.
- Strategy families of `model::Strategy`:
  - 2D / 2.5D: face, pocket, contour, adaptive, slot, trace, chamfer,
    engrave, bore, circular, thread mill.
  - Drilling (`drill`, with hole recognition and grouping in `holes`).
  - 3D roughing: adaptive and pocket clearing, with rest roughing of the
    material a previous operation or a larger reference tool left.
  - 3D finishing: parallel, contour/waterline, scallop, pencil,
    steep/shallow, radial, spiral, morphed spiral, horizontal, ramp, project,
    flow, morph, rest finishing.
  - Multi-axis: swarf, multi-axis contour, rotary wrap, rotary finishing and
    multi-axis finishing over a 3-axis operation, with machine kinematics in
    `kinematics`.
  - Turning: face, profile, groove, thread and part-off (`turning`).
  - Probing (work offset and geometry), operation patterns and manual NC
    entries (`probe`, `pattern`).
- `features` and `autoprogram`: 2.5D feature recognition and automatic
  programming through a technology table (the built-in table is
  `data/techdb.json`).
- `library`, `feeds`, `template`: tool libraries, feeds-and-speeds starting
  values, and saved operation templates.

## Program, simulation and output

- `program`: assembles ordered operations across setups into one NC
  `Program` (`program::assemble`), with tool changes, spindle and coolant
  switches; `filter`, `arcfit`, `link` and `polish` filter, arc-fit, link
  and optionally smooth the motions.
- `post`: post-processors are JavaScript files that call `post.define({...})`.
  The crate ships 14 built-in posts in `posts/` (`post::BUILTIN_POSTS`:
  Fanuc, Haas, GRBL, LinuxCNC, Mach, Heidenhain, Siemens 840D, Marlin, Okuma,
  Mazak, and Fanuc/Haas/Okuma/Mazak lathe posts). Running a post
  (`post::run`, `post::run_builtin`) needs the `post-runtime` feature.
- `stock`, `stock_sim`: the setup's stock resolved into machine coordinates,
  and Z-dexel material-removal simulation with stock carried between setups.
- `collision`: shank, holder and rapid collision checks during simulation
  (`simulate_checked`).
- `verify`: the stock-versus-part deviation map (`deviation_map`).
- `cycle_time`: machining time estimates from the motion stream
  (`estimate_cycle_time`).

## Features

| Feature | Default | Effect |
|---|---|---|
| `schema` | off | Derives JSON Schemas (`schemars`) for persisted CAM model types, including strategy parameters. |
| `kernel` | off | Enables `kernel_adapter`: tessellates `BREP_kernel` bodies into mesh snapshots and reports the analytic faces hole and feature recognition read. |
| `post-runtime` | off | Runs post-processor scripts on `BREP_script_core` (Boa). The post types and built-in sources are available without it. |
| `parallel` | off | Runs contact solves on rayon's thread pool. Output is byte-identical with and without it. |

The default runtime dependencies are `serde` and `serde_json`.

Build it from the repository root:

```bash
cargo build --manifest-path BREP_cam/Cargo.toml
```

The public package contains production sources, the built-in posts
(`posts/`), the technology table (`data/`), the manifest, README, and
license.
