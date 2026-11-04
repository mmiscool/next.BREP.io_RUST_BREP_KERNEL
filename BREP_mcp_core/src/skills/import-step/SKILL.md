---
name: import-step
description: Import a STEP artifact into BREP and qualify the resulting solids and operation outcome.
---

Use cad.files.import for the semantic STEP workflow; inspect its descriptor at brep://cad/method/cad.files.import. For large local files, service.artifacts.create accepts a host path under the trusted local profile. Other clients can use begin, append and commit with the declared byte length and SHA-256.

Create or bind the intended document and await its import. Normally await run_script itself through completion. Only request wait_ms explicitly when a bounded wait is needed; use script_result by executionId to recover an unfinished or disconnected execution. Verify terminal status, resulting solids, validity, units and warnings. Report partial effects or unknown effects explicitly. Do not repeat an import merely because the connection was lost.
