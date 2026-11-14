# BREP_reconstruction

`BREP_reconstruction` owns code that needs both the kernel-free
`BREP_RANSAC` recognizer and `BREP_kernel`: mesh/metadata conversion,
carrier-to-kernel conversion, hybrid analytic/faceted reconstruction, STEP
validation support, and the STL-to-STEP conversion pipeline. Its Rust library
target is `brep_reconstruction`.

The crate re-exports the `brep_ransac` public API. Integration-specific APIs are grouped
under:

- `brep` for kernel mesh/metadata conversion, analytic carrier conversion (including
  `nurbs_surface_from_analytic`), and attaching face metadata;
- `step_validation` for validating STEP faces against their exact analytic carriers
  (`validate_step_file`, `validate_step_file_faces_with_progress`), reports, progress,
  and CSV output (`reports_to_csv`);
- `stl` for ASCII/binary STL parsing and welding (`read_stl`, `read_stl_with_options`,
  `parse_stl_bytes`);
- `stl_conversion` for recognition, hybrid/faceted fallback, and validated AP242 STEP
  serialization (`convert_stl_mesh_to_step`, and `convert_stl_mesh_to_step_with_colors`,
  which carries per-triangle colours onto the STEP presentation styles);
- `import_session` for interactive mesh import: `analyze_regions` reports the regions the
  hybrid builder will try to fit, `fit_region` refits one region as a chosen surface kind
  against a user tolerance, and `fit_nurbs_region` fits a least-squares B-spline surface
  where no analytic surface works.

`segment_for_hybrid` (crate root) exposes the mesh partition the hybrid builder starts
from.

It is a library-only package, version 0.16.0. The kernel dependency is pinned
EXACTLY (`BREP_kernel = "=0.19.0"`), so the two move in lockstep; it also depends on
`BREP_RANSAC`, `serde` and `web-time`. Licence: the Autodrop3d licence in
`LICENSE.md`, shipped in the crate.
