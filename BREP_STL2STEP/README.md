# BREP_STL2STEP

Part of the [next.BREP.io](https://next.brep.io) Rust B-rep kernel: `cargo install BREP_STL2STEP` · [licence summary](#licence)

Command-line STL-to-STEP converter of the BREP crate family. It depends on
`BREP_reconstruction`; its binary name is `stl2step`.

Install and run it with:

```sh
cargo install BREP_STL2STEP
stl2step input.stl
```

The command writes `input.step` beside `input.stl`, refuses to replace an existing STEP or
JSON report unless `--force` is supplied, and writes through a temporary file before the
destination is replaced. STL coordinates default to millimetres; `--unit` accepts `mm`,
`cm`, `m`, `micron`, `inch`, or `foot`, plus the spelled-out and short aliases
`millimeter`/`millimetre`, `centimeter`/`centimetre`, `meter`/`metre`,
`um`/`micrometer`/`micrometre`, `in`, and `ft` (case-insensitive).

The output is a STEP (AP242) document that is round-trip validated before it is written. A
file holding several disjoint closed shells converts to one STEP document with one solid per
shell; a closed inward-facing shell inside another component becomes a cavity of that body.
Recognition preserves safe analytic plane, cylinder, cone, and sphere regions in a hybrid
model; full spheres, ring tori, capped cylinders, and capped cones also have direct analytic
paths. Other repairable closed input can fall back to a faceted BREP unless
`--strict-analytic` is set. Use `--report-json PATH` for the schema-version `2` JSON report;
a successful run always prints a human-readable terminal report.

Exit statuses are `0` for success, `2` for usage, `3` for STL input, `4` for conversion,
and `5` for output I/O. Run `stl2step --help` for all recognition and tolerance options.

The package is version 0.2.28 and contains only this binary. Licence: the Autodrop3d
licence in `LICENSE.md`, shipped in the crate.

## Licence

In plain words: the crates are free to use as they are, including in a commercial project. If you
fix or change something in the kernel, send the change back as a pull request to
[github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL](https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL). If you do not want to share your
changes, buy a commercial licence from [Autodrop3d LLC](https://autodrop3d.com); with it you never
have to share your code back.

The exact terms are in `LICENSE.md`, shipped in the crate (`license-file` in the manifest) and at
[LICENSE.md](https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL/blob/master/LICENSE.md); if this summary and the file ever differ, the file wins.
