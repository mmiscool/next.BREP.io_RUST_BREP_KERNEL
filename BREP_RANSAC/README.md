# BREP_RANSAC

`BREP_RANSAC` is the kernel-free analytic recognition leaf prepared for the
BREP repository. Its Rust library target is `brep_ransac`.

It recognizes plane, cylinder, cone, sphere, and torus carriers in neutral
indexed triangle meshes. It supports generic recognition, known-type fitting,
initial guesses, constrained fitting, and exact-candidate validation. Results
contain neutral carrier parameters, support IDs, residuals, observability,
confidence evidence, and diagnostics; they contain no BREP topology.

Build it from the repository root:

```bash
cargo build --manifest-path BREP_RANSAC/Cargo.toml
```

The main entry points are `recognize_surfaces` and
`recognize_surfaces_with_unresolved` for mesh-wide recognition, plus
`reconstruct_surface` / `reconstruct_surface_from_vertices` for a selected support.
`export_debug_obj` writes recognition debug geometry.

The public package contains production sources, the manifest, README, and license.
Runtime dependencies are `serde` and `web-time`.
