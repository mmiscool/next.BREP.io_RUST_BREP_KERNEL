---
name: recover-execution
description: Recover a lost BREP script response or reconcile failed and cancelled CAD work without duplicating changes.
---

If submission returned no response, resubmit exactly the same source, language and key. If an executionId is known, use script_result. Inspect state, error, operations and pendingOperationIds. script_halt kills the script worker but accepted CAD operations continue; cancel those explicitly through service.operations.cancel when the requested outcome requires it.

An epoch mismatch or expired/unknown handle is not proof that nothing happened. Inspect the document and saved artifacts before deciding whether another execution is appropriate. Do not infer rollback from failure, and do not automatically retry an external write with a new key.
