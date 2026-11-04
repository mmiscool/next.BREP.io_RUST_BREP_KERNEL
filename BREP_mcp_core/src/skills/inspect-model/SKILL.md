---
name: inspect-model
description: Inspect BREP documents, units, solid validity, bounds and volume before choosing an edit.
---

Read brep://cad/sdk.d.ts when signatures are needed. Use run_script with this module:

```js
import { brep } from "@brep/cad";
export default async function main() {
  const tabs = await brep.document.docs_list({});
  const model = await brep.scene.model_inspect({});
  return { tabs, model };
}
```

Confirm the intended document and units before interpreting dimensions. Names and geometry references come from the live model; do not invent them. Return relevant measurements and warnings, not the entire registry. Use the screenshot tool when visual confirmation helps.

Connection: HTTP requires the bearer token from the configured token file
(default `~/.config/brep-app/mcp-token`, overridden by `BREP_MCP_TOKEN_FILE`).
Use the client's environment-backed credential configuration; never print or
save the token in instructions or committed settings. This server does not use
OAuth/Dynamic Client Registration. For independent modeling without a window,
connect to `brep-mcp serve`; `brep-app --mcp` controls the visible app.
`run_script` normally awaits its result in the same call; use `script_result`
for recovery or when explicitly requesting a bounded `wait_ms`.
