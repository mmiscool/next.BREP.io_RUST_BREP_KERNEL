# BREP — a boundary-representation geometry kernel

A native/WASM B-rep geometry kernel for building CAD applications, written in
Rust. Published on crates.io as **`BREP_kernel`** with library name
`brep_kernel`; this is version **0.8.0**.
It is the authoritative geometry engine behind the BREP CAD application
(`BREP_app` + `BREP_render` in this repository, re-exported as `brep::kernel` by
the umbrella `BREP` crate), and is usable standalone as a
library: exact NURBS geometry, manifold B-rep topology, booleans, offsets,
fillets, sheet metal, tessellation, STEP import/export, and a feature-history
pipeline driven by JSON schemas.

Current port coverage is intentionally capability-gated:

### Curve and surface geometry

- vector math;
- clamped knot vectors and nonvanishing B-spline basis derivatives;
- exact rational NURBS curve evaluation and derivatives;
- exact line, rational quadratic circular-arc, and arbitrary-plane circle
  constructors, plus clamped-knot generation and curve interpolation helpers;
- banded linear solves with a stable public math API;
- persistent curve handles using typed-array WASM transfers;
- tensor-product rational NURBS surface evaluation, partial derivatives, and
  normals with persistent typed-array handles;
- robust point projection onto rational curves and surfaces, including
  closed parameters and pole/apex rescue;
- lazily recognized analytic carriers (plane, cylinder/cone, sphere, torus)
  cached on each exact rational surface, with closed-form point projection
  in the surface's own rational parameterization replacing grid-plus-Newton
  search, and exact analytic intersection curves (circles, generatrix line
  pairs, elliptic plane/quadric sections via homogeneous control-net maps,
  sphere/sphere circles, torus circles) that bypass surface/surface
  marching and polyline fitting entirely, with proven-empty results
  skipping the marcher;

### Topology and construction

- exact BREP topology records and structural/geometric validation;
- exact manifold box and regular-pyramid construction, including shared edges,
  coedge senses, p-curves, loops, faces, and shell ownership;
- exact cylinder, cone/frustum, sphere, and torus topology, including closed
  rational surfaces, seam edges, cap sharing, pole/apex degeneracies, and
  genus;
- exact closed-profile extrusion with orientation normalization, rational
  side surfaces, shared longitudinal edges, and planar caps;
- exact full and partial profile revolution, including axis-edge degeneracies,
  shared circle/arc edges, per-face seams, radial end caps, and genus;
- exact compatible-section lofts using shared averaged chord parameters,
  global B-spline interpolation, shared skin boundaries, and planar caps;
- exact affine BREP transforms and reflection-aware orientation reversal;

### Intersection, classification, and arrangement

- exact curve/curve and curve/surface intersection search with convex-hull
  broad phases, damped Newton correction, seam wrapping, and tangency flags;
- predictor/corrector surface/surface intersection marching with adaptive
  step control, seam wrapping, loop closure, boundary polishing, and seeds;
- trimmed-face-aware point-in-solid classification with deterministic
  boundary detection and clean-ray retry logic;
- planar segment arrangement with crossing splits, dangling-chain pruning,
  cycle extraction, and hole assignment;
- public segment-intersection and boundary-aware point-in-polygon helpers;
- exact affine and adaptive curved-surface p-curve construction with periodic
  seam unwrapping;

### Booleans, offsets, and healing

- imprint construction with cosurface boundaries, fitted SSI branches,
  global trim-edge splitting, missed-pierce recovery, and shared p-curves;
- traversal-aligned boundary-edge splitting and per-face fragmentation;
- regularized union, intersection, and subtraction with fragment
  classification, coincident-edge sewing, shell grouping, genus recovery,
  validation, and exact-volume verification;
- offset-face carriers and an intersection-built offset-shell API, including
  smooth-boundary synchronization, partial-chain reconstruction, opening-wall
  provenance, same-carrier wall coalescing, and exact manifold validation;
- exact concatenation of collinear and same-NURBS continuation edges, plus
  overlapping one-use edge conformance when adjacent faces split the same
  line or arc differently;
- exact same-carrier face grouping and affine coplanar face merging with
  boundary p-curves rebuilt on the merged planar carrier;
- persistent face/edge names carried on topology records and propagated
  exactly through booleans: split fragments get deterministic `_1`/`_2`
  suffixes, merges keep the surviving face's name, welded shared edges keep
  the first name seen, and new intersection edges are named `FACEA|FACEB`
  from their two supporting faces (the application's derived-edge-name
  convention), so the application no longer re-derives boolean face names by
  geometric support matching;

### Analysis

- surface area and volume integration for planar, untrimmed curved, and
  trimmed curved faces, plus full mass properties (volume centroid and the
  unit-density inertia tensor about the centroid) via divergence-theorem
  moment integrals with an exact Green-boundary path for affine faces;

### Tessellation and meshing

- general trimmed-face BREP tessellation with per-triangle face ownership,
  plus a watertight chord-tolerance tessellator that samples every edge once
  (both adjacent faces reuse identical sample positions, so shared edges
  coincide exactly with no cracks or T-junctions) and refines face interiors
  by conforming edge splits until chords meet the tolerance;
- public parameter-space area, sampled trim polygon, face area/volume
  contribution, and individual-face tessellation APIs;
- indexed mesh representation, validation, and signed volume;
- box and cylinder mesh generation;

### Interop and infrastructure

- exact AP242 STEP output for rational and non-rational BREP topology;
- binary STL read/write and OBJ output;
- persistent solid handles for validation, transforms, mass properties,
  tessellation, booleans, offset-shell benchmarks, and STEP
  output without repeatedly decoding the input topology;
- native Criterion benchmarks and a versioned WASM JSON ABI.

Beyond that list, the crate root also re-exports edge blending
(`fillet_edge`/`fillet_edges`, `chamfer_edge` and variants, `blend_*`), STEP
import (`import_step`), the 2D sketch and assembly constraint solvers
(`solve_sketch`, `solve_assembly`), direct editing (`move_faces`,
`delete_face_and_heal`, `delete_faces_and_heal`), sewing and repair (`sew_solid`,
`mesh_regions_to_brep`), and the JSON feature-history execution pipeline
(`execute_history_json` plus the `feature_schema_catalogue` the application's
dialogs are generated from). See `src/lib.rs` for the full public surface; the
`#[wasm_bindgen]` JSON/typed-array endpoints live under `src/abi.rs`.

This kernel is the sole exact-geometry backend for the application. No exact
operation silently falls back to a mesh.

## Using the crate

The package name and the library name differ on purpose — the crates.io package
is `BREP_kernel`, the Rust library is the idiomatic lowercase `brep_kernel`:

```toml
[dependencies]
brep_kernel = { package = "BREP_kernel", version = "0.8" }
```

```rust
use brep_kernel::{
    boolean_operation, make_box_brep, make_cylinder_brep, solid_mass_properties,
    tessellate_brep_watertight, BooleanOperation, BooleanOptions, Vec3,
};

fn main() -> Result<(), String> {
    // A 40 x 30 x 20 block with a vertical radius-8 hole through it.
    let block = make_box_brep(Vec3::new(0.0, 0.0, 0.0), 40.0, 30.0, 20.0)?;
    let hole = make_cylinder_brep(
        Vec3::new(20.0, 15.0, -1.0), // base point
        Vec3::new(0.0, 0.0, 1.0),    // axis direction
        8.0,                         // radius
        22.0,                        // height (through the block)
    )?;
    let part = boolean_operation(
        &block,
        &hole,
        BooleanOperation::Subtract,
        &BooleanOptions::default(),
    ).map_err(|error| error.to_string())?;

    // Mass properties integrated from the B-rep (no display mesh involved).
    let props = solid_mass_properties(&part)?;
    println!("volume {:.3}, area {:.3}", props.volume, props.surface_area);

    // Watertight display mesh at a 0.05 chord tolerance.
    let mesh = tessellate_brep_watertight(&part, 0.05)?;
    println!("{} triangles", mesh.indices.len() / 3);
    Ok(())
}
```

Boolean operations return `Result<_, KernelRefusal>` with a typed refusal class;
most other modeling APIs still return `Result<_, String>`. Analytic and fitted
geometry coexist, so successful construction is not a promise of zero approximation.
The feature pipeline currently converts boolean refusals to an error string. STEP text goes in and out
through `import_step(&str) -> Result<Vec<BrepSolid>, String>` and
`export_step`.

## Building

The crate builds natively and for `wasm32-unknown-unknown`. The BREP CAD
application links it as an rlib inside its own wasm bundle; `wasm-pack build
--target web` on this crate produces a standalone kernel package with the
`wasm-bindgen` exports.

The packaged crate ships the library sources, this README, the licence and the
manifest; it carries no test suite.

## License and links

- License: the Autodrop3d licence in `LICENSE.md`, shipped in the crate
  (`license-file` in the manifest).
- The family: [`BREP_gizmos`](https://crates.io/crates/BREP_gizmos) (overlay
  widgets), [`BREP_render`](https://crates.io/crates/BREP_render) (the wgpu
  render/pick engine), [`BREP_app`](https://crates.io/crates/BREP_app) (the
  application), and the umbrella [`BREP`](https://crates.io/crates/BREP).
