# BREP_surface_offset

Part of the [next.BREP.io](https://next.brep.io) Rust B-rep kernel: `cargo add BREP_surface_offset` · [licence summary](#licence)

`BREP_surface_offset` holds the surface offsets of the BREP kernel. Its Rust library
target is `brep_surface_offset`; this is version **0.32.2**.

It is one of the implementation crates the kernel is built from. Applications
should depend on [`BREP_kernel`](https://crates.io/crates/BREP_kernel), which
re-exports the public API of every kernel crate at its root (`brep_kernel::…`)
and adds the feature pipeline and the STEP, IGES, Parasolid (import), 3MF,
GLB, STL and OBJ exchange.
Many items here are public only so the other kernel crates can reach them;
those are hidden from the documentation and carry no stability promise beyond
the kernel's own. The kernel crates are released together, at one version.

It contains:

- offset carriers of analytic and NURBS faces;
- re-trimming and re-intersecting offset faces;
- regularity checks, fold loci and carving of the parts an offset folds over.

Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology). It is required by [`BREP_blend`](https://crates.io/crates/BREP_blend), [`BREP_offset`](https://crates.io/crates/BREP_offset), [`BREP_direct_edit`](https://crates.io/crates/BREP_direct_edit), [`BREP_kernel`](https://crates.io/crates/BREP_kernel).

## Building

The crate builds natively and for `wasm32-unknown-unknown`. The `parallel`
feature runs some per-face loops on rayon; `profile-wall` turns on a diagnostic
wall-time partition. Both are off by default, and `BREP_kernel` forwards them.

The packaged crate ships the library sources, the README, the licence and the
manifest; it carries no test suite.

## Licence

In plain words: the crates are free to use as they are, including in a commercial project. If you
fix or change something in the kernel, send the change back as a pull request to
[github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL](https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL). If you do not want to share your
changes, buy a commercial licence from [Autodrop3d LLC](https://autodrop3d.com); with it you never
have to share your code back.

The exact terms are in `LICENSE.md`, shipped in the crate (`license-file` in the manifest) and at
[LICENSE.md](https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL/blob/master/LICENSE.md); if this summary and the file ever differ, the file wins.
