# BREP

Umbrella crate for the **BREP** CAD kernel family. One dependency that
re-exports the geometry kernel and, behind feature flags, the higher layers of
the stack — so downstream code says `use brep::kernel::…` / `use brep::render::…`
instead of depending on each sibling crate directly.

| `brep::` module | Feature    | Backing crate  | Role |
|-----------------|------------|----------------|------|
| `kernel`        | *(always)* | [`BREP_kernel`](https://crates.io/crates/BREP_kernel) | The BREP geometry kernel (NURBS B-rep, booleans, tessellation). |
| `gizmos`        | `gizmos`   | [`BREP_gizmos`](https://crates.io/crates/BREP_gizmos) | In-scene gizmos / overlay widgets (pure geometry + hit-testing). |
| `render`        | `render`   | [`BREP_render`](https://crates.io/crates/BREP_render) | Windowing-agnostic wgpu render/pick engine. |
| `app`           | `app`      | [`BREP_app`](https://crates.io/crates/BREP_app)       | The eframe (egui + wgpu) CAD application. |
| `reconstruction` | `reconstruction` | [`BREP_reconstruction`](https://crates.io/crates/BREP_reconstruction) | RANSAC recognition-to-kernel reconstruction. |

The kernel is always re-exported; everything above it is opt-in so a headless
geometry consumer never compiles `wgpu` / `eframe`.

```toml
[dependencies]
BREP = "0.8"                                      # kernel only (headless)
BREP = { version = "0.8", features = ["render"] } # + the wgpu render engine
BREP = { version = "0.8", features = ["full"] }   # app + render + gizmos
```

Additional features: `reconstruction` (opt-in recognition integration), `full`
(an alias for `app`, which also enables `render` and `gizmos`), and `parallel`
(forwards to the kernel's rayon-parallel tessellation). `full` does not enable
`reconstruction` or `parallel`; request those separately when needed.

> **Package vs. lib name:** the crates.io package is `BREP`; the Rust library is
> `brep` (lowercase, lint-clean). This mirrors the whole family — package
> `BREP_render` / lib `brep_render`, etc.

This is version 0.8.0. The optional reconstruction feature re-exports the
reconstruction library directly; rendering also consumes it transitively.

Licence: the Autodrop3d licence in `LICENSE.md`, shipped in the crate
(`license-file` in the manifest).
