---
name: repair-feature
description: Diagnose and repair a refused BREP feature using its actual schema, geometry references and refusal details.
---

Inspect the failed history entry, refusal text and feature schema. Determine whether the cause is invalid parameters, missing references, stale document context or geometric refusal. Reacquire geometry references from the live scene after upstream edits.

Make a targeted change supported by the diagnosis, await evaluation, then inspect validity and downstream history. Preserve the engine refusal text in the report. Stop when another change would alter design intent without enough information; repeated blind radius or tolerance changes are not a repair strategy.
