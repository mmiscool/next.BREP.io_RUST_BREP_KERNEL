# BREP_simulation

Part of the [next.BREP.io](https://next.brep.io) Rust B-rep kernel: `cargo add BREP_simulation` · [licence summary](#licence)

Motion studies for BREP assemblies, as plain data with no kernel or GPU dependency:

- **model**: the `simulation` block an assembly document keeps its studies in. A study
  has a duration, a frame rate, motors and clips. A motor is one of the assembly's
  distance or angle mates, driven at a constant speed or along a motion profile;
  everything else follows through the assembly solver. Clips are the timeline's other
  rows: a mate switched off, the camera moving through saved views, components hidden
  or made transparent, a section plane and an exploded view's steps.
- **sample**: what a study asks for at one time: the value or suppression each mate
  takes, and the camera, visibility, transparency, section and explode state. The
  caller solves the assembly with those values (`Study::sample` returns a
  `FrameSample`; `Study::frame_count`, `frame_time` and `frame_at` map between
  frames and seconds).
- **forms**: a motor's and a clip's fields as a schema-driven form reads them
  (`motor_schema` / `clip_schema`), the flat parameters the form edits
  (`motor_params` / `clip_params`), which fields hide in the current mode, and the
  patch an edit applies (`motor_patch` / `clip_patch`).

`Simulation::from_document` reads and validates the block from a whole document's JSON
(`from_value` for the block alone, `to_value` to write it back). A study also carries an
interference flag (check every frame for colliding parts) and, once published, its baked
motion.

Features: `schema` (off by default) derives `schemars::JsonSchema` for the model types.
Licence: the Autodrop3d licence in `LICENSE.md`, shipped in the crate.

## Licence

In plain words: the crates are free to use as they are, including in a commercial project. If you
fix or change something in the kernel, send the change back as a pull request to
[github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL](https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL). If you do not want to share your
changes, buy a commercial licence from [Autodrop3d LLC](https://autodrop3d.com); with it you never
have to share your code back.

The exact terms are in `LICENSE.md`, shipped in the crate (`license-file` in the manifest) and at
[LICENSE.md](https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL/blob/master/LICENSE.md); if this summary and the file ever differ, the file wins.
