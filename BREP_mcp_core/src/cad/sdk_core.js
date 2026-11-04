// @brep/cad — the code-mode SDK's hand-written CORE (contract §8.1): the
// bridge, Response unwrapping, ApiFailure, the low-level `service` and the
// one-operation lifecycle. The per-command convenience bindings that follow
// it are GENERATED from the catalogue (`cad/sdk.rs`). Served as a virtual module
// inside the restricted realm; its only door out is the host bridge
// `__brep_host(method, json) -> Promise<json>`, whose replies are the
// service's own `Response<T>` records. Host validation, defaulting and
// authorization stay authoritative: nothing here is trusted by the service.
const host = globalThis.__brep_host;
/** The manifest {apiVersion, sha256} this SDK was generated against. */
export const manifest = JSON.parse(globalThis.__brep_manifest);
delete globalThis.__brep_host;
delete globalThis.__brep_manifest;

let failureSequence = 0;
export class ApiFailure extends Error {
  constructor(error) {
    super(error && error.message ? error.message : "API failure");
    this.name = "ApiFailure";
    // Preserve server diagnostics; SDK-originated failures still satisfy ApiError.
    this.error = Object.assign({
      code: "internal_error", message: this.message, phase: "running",
      correlationId: "sdk-" + Date.now().toString(36) + "-" + (++failureSequence),
      operationId: null, executionId: null, context: null,
      effects: { disposition: "unknown", documents: [], external: [] },
      retry: "inspect_then_new_submission"
    }, error || {});
    this.operationId = error ? error.operationId : null;
    this.executionId = error ? error.executionId : null;
  }
}

async function call(method, request) {
  const text = await host(method, JSON.stringify(request === undefined ? {} : request));
  return JSON.parse(text);
}

function unwrap(response) {
  if (response && response.ok === true) return response.value;
  throw new ApiFailure(response ? response.error : null);
}

export const service = {
  inputs: {
    exportOptions: (req) => call("cad.inputs.exportOptions", req),
  },
  discovery: {
    search: (req) => call("cad.discovery.search", req),
    describe: (req) => call("cad.discovery.describe", req),
  },
  documents: {
    list: (req) => call("cad.documents.list", req),
    get: (req) => call("cad.documents.get", req),
  },
  operations: {
    issueKey: (req) => call("cad.operations.issueKey", req),
    submit: (req) => call("cad.operations.submit", req),
    status: (req) => call("cad.operations.status", req),
    result: (req) => call("cad.operations.result", req),
    cancel: (req) => call("cad.operations.cancel", req),
  },
  commands: {
    query: (req) => call("cad.commands.query", req),
  },
  code: {
    submit: (req) => call("cad.code.submit", req),
    status: (req) => call("cad.code.status", req),
    result: (req) => call("cad.code.result", req),
    cancel: (req) => call("cad.code.cancel", req),
  },
  artifacts: {
    read: (req) => call("cad.artifacts.read", req),
    create: (req) => call("cad.artifacts.create", req),
    begin: (req) => call("cad.artifacts.begin", req),
    append: (req) => call("cad.artifacts.append", req),
    commit: (req) => call("cad.artifacts.commit", req),
  },
};

// One operation, start to finish: a key issued once, one submission, then
// the terminal result. A lost reply is recovered by resubmitting the SAME
// key and envelope — never a fresh key.
async function operate(command, args, context) {
  const key = unwrap(await service.operations.issueKey({ purpose: "command" })).key;
  const request = { manifest, command, args, context, idempotencyKey: key };
  let admission;
  for (let attempt = 0; ; attempt++) {
    const r = await service.operations.submit(request);
    if (r.ok) { admission = r.value; break; }
    if (r.error && r.error.code === "overloaded" && attempt < 20) {
      await call("cad.internal.pause", { ms: Math.min(1000, 100 * 2 ** attempt) });
      continue;
    }
    throw new ApiFailure(r.error);
  }
  const id = admission.operationId;
  const result = unwrap(await call("cad.internal.awaitOperation", { operationId: id }));
  if (result.state !== "succeeded") {
    throw new ApiFailure(result.error || { code: "job_failed", message: "operation " + result.state, operationId: id });
  }
  return result.output;
}
