---
name: export-model
description: Validate a BREP model and export it to an authorized format and destination.
---

Inspect the intended document and requested output format. Verify relevant solid validity, units and assembly scope before export. Read the exact export schema; cad.files.export provides a STEP artifact, while brep.document.document_export covers the application's broader formats.

Return artifact identity, media type, size and digest where available. An artifact handle is temporary and is not a saved user file. Avoid returning base64 geometry to the model; use chunked artifact retrieval or the requested file destination. A failed export or lost reply requires checking the destination before retrying an overwrite.

For basic file management use the `files` front-door tool with `op`, or the equivalent `brep.files.*` methods in a script. Paths are relative to named roots (`documents`/`workspace`, native execution `artifacts`, or configured native roots). Use `stat` before retrying a write; replacement requires `overwrite: true`. Browser file management supports its persistent documents/workspace store only. File paths and opaque export artifact handles are distinct.
