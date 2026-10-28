# BREP_cam

`BREP_cam` is the CAM engine of the BREP CAD kernel. Its Rust library target is
`brep_cam`. It is pure Rust and builds for native targets and for
`wasm32-unknown-unknown`, so the same code generates toolpaths in the desktop
application and in the browser worker.

The crate holds the shared CAM contract every strategy builds on:

- `model`: setups (rigid model-to-machine transform, work offset, stock,
  clearance heights), machines, tools and holders, operations with one typed
  parameter set per strategy, target references by stable body/face identity,
  and the toolpath as an ordered stream of typed motions with per-operation
  diagnostics.
- `numeric`: the single tolerance object all algorithms read.
- `mesh`: the immutable triangle snapshot that keeps each triangle's source
  body and face.
- `index`: the triangle spatial-index trait and its implementations.
- `cutter`: cutter profiles (flat, ball, bull, cone/V, ball-cone) with
  validated dimensions.
- `cancel`: cancellation and progress reporting at bounded work chunks.

All algorithms work in machine coordinates with the cutter axis along `+Z`;
the setup transform is the only place model coordinates become machine
coordinates. Output is deterministic: the same inputs produce a byte-identical
motion stream.

Build it from the repository root:

```bash
cargo build --manifest-path BREP_cam/Cargo.toml
```

The public package contains production sources, the manifest, README, and
license. The runtime dependency is `serde`.
