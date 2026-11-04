---
name: edit-assembly
description: Edit BREP assembly components and constraints while preserving work-part context and component identity.
---

Inspect the current document, work-part context and assembly component identities. Read brep://api/commands and load the exact assembly functions needed. Distinguish an occurrence from its source part: editing a source may affect multiple occurrences. Use live component and constraint identifiers.

Await edits sequentially. Reinspect constraints, component transforms and interference relevant to the user request. Check that the intended work part remains active before a UI-driven step. Save only the documents authorized by the requested change; explain any unresolved constraints.
