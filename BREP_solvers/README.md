# BREP_solvers

`BREP_solvers` holds the constraint solvers of the BREP kernel. Its Rust library
target is `brep_solvers`; this is version **0.25.0**.

It is one of the implementation crates the kernel is built from. Applications
should depend on [`BREP_kernel`](https://crates.io/crates/BREP_kernel), which
re-exports the public API of every kernel crate at its root (`brep_kernel::…`)
and adds the feature pipeline and the STEP, IGES, Parasolid (import), 3MF,
GLB, STL and OBJ exchange.
Many items here are public only so the other kernel crates can reach them;
those are hidden from the documentation and carry no stability promise beyond
the kernel's own. The kernel crates are released together, at one version.

It contains:

- the 2D sketch solver;
- the 3D sketch solver;
- the assembly mate solver, its resolution of mates against parts, and assembly ids.

Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology). It is required by [`BREP_kernel`](https://crates.io/crates/BREP_kernel).

## Building

The crate builds natively and for `wasm32-unknown-unknown`. The `parallel`
feature runs some per-face loops on rayon; `profile-wall` turns on a diagnostic
wall-time partition. Both are off by default, and `BREP_kernel` forwards them.

The packaged crate ships the library sources, the README, the licence and the
manifest; it carries no test suite.

## License

The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file` in
the manifest). Source: <https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL>
