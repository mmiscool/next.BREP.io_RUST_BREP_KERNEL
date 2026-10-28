# BREP_instructions

Manufacturing instructions for BREP assemblies, as plain data with no kernel or GPU
dependency:

- **model**: the `instructions` block of an instructions document (`.ibrep`): the target
  assembly revision, the page template, front matter, and the step tree: a step holds
  the parts it adds, a view, tools, torque, callouts, insertion arrows, notes and its
  child steps (the group it installs, built first).
- **wizard**: proposes a step order from an assembly's occurrence tree and mate graph:
  a sub-assembly as one step whose children build it, then mate order, with fasteners last.
- **edit**: merges, splits, reorders and updates steps, and derives each step's
  presentation (placed, being added, not yet placed).
- **retarget**: compares two revisions of the target assembly and folds the changes
  into the steps, held for the author's approval.
- **layout**: printed pages from a template (1, 2 or 4 steps per page, portrait or
  landscape), with a cover, a parts list and a tools list.
- **player**: the interactive HTML player. `player/player.html` is a trusted template,
  and `PlayerData` is its data format. A server serves the template and the data
  separately (`served_html`), and an offline copy inlines the data into one
  self-contained file (`offline_html`).
