---
name: safe-parametric-edit
description: Edit a BREP feature using document identity, revision checks and post-edit verification.
---

Read the live function and feature schemas before constructing parameters. Inspect open tabs and the intended feature first. Supply context with documentId and expectedRevision where the command supports it; obtain the current revision from docs_list. Await each write and read its resulting context before the next write. A revision conflict requires inspecting the intervening change, not blindly retrying.

After the edit settles, inspect solid count, validity, dimensions and warnings. Save only to the user-authorized destination. Scripts can partially commit: failure does not roll back earlier calls. Reuse the same run_script key to recover a lost response; use a new key only for a new intended execution.

With wait: true, top-level report and after.report describe the same settled state. Inspect featureErrors before declaring geometric success; wait: false reports submission only. Cylinder radii must be positive, including evaluated expressions; use signed height or transforms for direction.
