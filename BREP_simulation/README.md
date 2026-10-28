# BREP_simulation

Motion studies for BREP assemblies, as plain data with no kernel or GPU dependency:

- **model**: the `simulation` block an assembly document keeps its studies in. A study
  has a duration, a frame rate, motors and clips. A motor is one of the assembly's
  distance or angle mates, driven at a constant speed or along a motion profile;
  everything else follows through the assembly solver. Clips are the timeline's other
  rows: a mate switched off, the camera moving through saved views, components hidden
  or made transparent, a section plane and an exploded view's steps.
- **sample**: what a study asks for at one time: the value or suppression each mate
  takes, and the camera, visibility, transparency, section and explode state. The
  caller solves the assembly with those values.
