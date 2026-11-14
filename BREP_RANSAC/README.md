# BREP_RANSAC

`BREP_RANSAC` is the kernel-free analytic surface recognizer of the BREP crate
family. Its Rust library target is `brep_ransac`.

It recognizes plane, cylinder, cone, sphere, and torus carriers in neutral
indexed triangle meshes (`Mesh`: vertices, triangles, optional vertex normals
and source metadata). It supports generic recognition and, through
`SurfaceHint`, known-type fitting, initial guesses, constrained fitting, and
exact-candidate validation. Results contain neutral carrier parameters
(`AnalyticSurface`), supporting triangle indices, residual metrics, a
confidence score, and fit diagnostics (the fit path taken, refinement and
rejected competitors); they contain no BREP topology.

Build it from the repository root:

```sh
cargo build --manifest-path BREP_RANSAC/Cargo.toml
```

The main entry points are:

- `recognize_surfaces` and `recognize_surfaces_with_unresolved` for mesh-wide
  recognition (the latter returns a `RecognitionResult` that also lists the unresolved
  triangles and diagnostics for metadata regions that stayed unresolved);
- `recognize_surfaces_with_attempt_budget`, which caps the number of
  hypothesis attempts and restricts the surface types tried;
- `recognize_surfaces_with_candidates`, which validates carriers another stage
  already fitted to whole connected components (`ComponentCandidate`);
- `reconstruct_surface` / `reconstruct_surface_from_vertices` for a selected
  support (triangle or vertex indices) with a `SurfaceHint`;
- the `constrain_*` design-intent helpers (shared axis, coaxial cylinders,
  parallel/perpendicular/coplanar planes, concentric centers, equal radii,
  tangency), which turn neighbouring analytic geometry into
  `SurfaceHint::Constrained` values;
- `export_debug_obj`, which returns in-memory Wavefront OBJ and MTL text
  (`DebugObjExport`) with one coloured group per recognized region plus the
  unresolved triangles.

`RecognitionOptions` (with a `Default`) carries the distance, relative and
normal tolerances, minimum support, hypothesis and refinement limits, feature
angle, sampling mode, thread count and an optional deterministic seed.

The public package contains production sources, the manifest, README, and license.
Runtime dependencies are `serde` and `web-time`.
