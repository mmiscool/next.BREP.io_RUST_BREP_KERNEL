# BREP_reconstruction

`BREP_reconstruction` owns code that needs both the kernel-free
`BREP_RANSAC` recognizer and `BREP_kernel`: mesh/metadata conversion,
carrier-to-kernel conversion, hybrid analytic/faceted reconstruction, STEP
validation support, and the STL-to-STEP conversion pipeline.

The crate re-exports the `brep_ransac` public API. Integration-specific APIs are grouped
under:

- `brep` for kernel mesh/metadata conversion, analytic carrier conversion, and attaching
  face metadata;
- `step_validation` for STEP validation reports, progress, and CSV output;
- `stl` for ASCII/binary STL parsing and welding;
- `stl_conversion` for recognition, hybrid/faceted fallback, and validated AP214 STEP
  serialization.

It is a library-only package, version 0.5.0. The kernel dependency is pinned
EXACTLY (`BREP_kernel = "=0.8.0"`), so the two move in lockstep; it also depends on
`BREP_RANSAC`, `serde` and `serde_json`. Licence: the Autodrop3d licence in
`LICENSE.md`, shipped in the crate.
