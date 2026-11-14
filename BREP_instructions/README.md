# BREP_instructions

Manufacturing instructions for BREP assemblies, as plain data with no kernel or GPU
dependency:

- **model**: the `instructions` block of an instructions document (`.ibrep`): the target
  assembly revision, the page template, front matter, and the step tree: a step holds
  the parts it adds, a view, tools, torque, callouts, insertion arrows, notes and its
  child steps (the group it installs, built first).
- **source**: reads the wizard's input (a `wizard::Level`) straight from an
  assembly document's JSON: its placements, embedded parts and their attributes
  (`Fastener`, `Torque`, `Serial_Tracked`), mates and exploded views, with
  sub-assemblies read the same way, recursively up to `source::MAX_DEPTH`
  levels (`source::level_from_document`).
- **wizard**: proposes a step order from an assembly's occurrence tree and mate graph:
  a sub-assembly as one step whose children build it, then mate order, with fasteners last.
- **edit**: merges, splits, reorders and updates steps, and derives each step's
  presentation (placed, being added, not yet placed).
- **retarget**: compares two revisions of the target assembly and folds the changes
  into the steps, held for the author's approval.
- **fields**: what the person doing the work records at a step: boxes to tick, choices,
  text, measurements with limits and serial numbers, any number per step, and the one
  rule every player and server applies to their values (required, in limits).
- **layout**: printed pages from a template (1, 2 or 4 steps per page, portrait or
  landscape, line-art or shaded pictures), with a cover, a parts list and a tools list.
- **record**: a step's fields as a paper form for the printed pages (a choice prints
  every option as its own box), paged by the layout and continued on Record pages.
- **player**: the interactive HTML player. `player/player.html` is a trusted template,
  and `PlayerData` is its data format. A server serves the template and the data
  separately (`served_html`), and an offline copy inlines the data into one
  self-contained file (`offline_html`). Beside the steps the data carries each step's
  view PMI and the assembly's tree with a thumbnail per part, for the page's Model and
  Parts tabs. The player runs in shop mode (step checklist and per-step sign-off)
  or customer mode (pictures first), and `PlayerData::motion_study` builds a
  player for a motion study instead of an instructions document.

## Features

- `schema` (off by default): derives JSON Schemas (`schemars`) for the
  instructions block's types, the step fields and the editing report types.

The crate depends on `serde` and `serde_json` only (plus `schemars` with
`schema`).
