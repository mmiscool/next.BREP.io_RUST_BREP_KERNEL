# BREP_STL2STEP

First-class BREP-family STL-to-STEP CLI crate. It depends on
`BREP_reconstruction`; its binary name is `stl2step`.

Install and run it with:

```bash
cargo install BREP_STL2STEP
stl2step input.stl
```

The command writes `input.step` beside `input.stl`, refuses to replace an existing STEP or
JSON report unless `--force` is supplied, and writes through a temporary file before the
destination is replaced. STL coordinates default to millimetres; `--unit` accepts `mm`,
`cm`, `m`, `micron`, `inch`, or `foot` (and aliases accepted by `--help`).

A file holding several disjoint closed shells converts to one STEP document with one solid
per shell. Recognition preserves safe analytic plane, cylinder, cone, and sphere regions in
a hybrid model;
full spheres, ring tori, capped cylinders, and capped cones also have direct analytic
paths. Other repairable closed input can fall back to a faceted BREP unless
`--strict-analytic` is set. Use `--report-json PATH` for the schema-version `2` JSON report;
the command always prints a human-readable terminal report.

Exit statuses are `0` for success, `2` for usage, `3` for STL input, `4` for conversion,
and `5` for output I/O. Run `stl2step --help` for all recognition and tolerance options.

The package is version 0.2.3 and contains only this binary. Licence: the Autodrop3d
licence in `LICENSE.md`, shipped in the crate.
