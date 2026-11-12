---
name: safe-parametric-edit
description: Edit a BREP feature using document identity, revision checks and post-edit verification.
---

Read the live function and feature schemas before constructing parameters. Inspect open tabs and the intended feature first. Pass `context: {documentId, expectedRevision}` at the root of a `brep.*` call's arguments (every app command and the feature tools take it; a tool that cannot check one refuses it): take `id` and `revision` of the visible tab from `brep.document.docs_list({})`. The write is refused with `revision_conflict` or `document_not_active` unless it names the visible document at that revision, and its reply carries `context` as it stands afterwards: pass that to the next write. Await each write before the next. A revision conflict requires inspecting the intervening change, not blindly retrying.

After the edit settles, inspect solid count, validity, dimensions and warnings. Save only to the user-authorized destination. Scripts can partially commit: failure does not roll back earlier calls. Reuse the same run_script key to recover a lost response; use a new key only for a new intended execution.

With wait: true, top-level report and after.report describe the same settled state. Inspect featureErrors before declaring geometric success; wait: false reports submission only. Cylinder radii must be positive, including evaluated expressions; use signed height or transforms for direction.
