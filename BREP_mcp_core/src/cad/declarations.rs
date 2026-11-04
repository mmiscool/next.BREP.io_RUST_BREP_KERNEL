//! The `@brep/cad` TypeScript declarations (contract §4, §8.1), generated
//! from the catalogue's own schemas: every command's argument and output
//! type is read off its descriptor, so a TypeScript consumer compiles
//! against exactly what the service validates. Written beside the other
//! generated references (`brep-mcp schema --write`, pinned by
//! `generated_docs_fresh`) and served as `brep://cad/sdk.d.ts`.
//!
//! Schema → TypeScript: string → `string`; number/integer → `number`;
//! boolean, null → themselves; `const` / `enum` → literal unions; an object's
//! declared properties → an object type with required/optional fidelity
//! (additional properties are not admitted); arrays → `T[]`; `oneOf`,
//! `anyOf` and type lists → unions; local JSON Pointer references → their
//! definitions. Unsupported or recursive shapes fall back to `unknown`
//! (never `any`), with bounded expansion.
use super::catalogue::Catalogue;
use serde_json::Value;

fn literal(v: &Value) -> String {
    match v {
        Value::String(s) => serde_json::to_string(s).unwrap_or_default(),
        other => other.to_string(),
    }
}

/// One schema as a TypeScript type expression. Local references always resolve
/// against this root, including while rendering nested properties and unions.
pub fn ts_type(schema: &Value) -> String {
    let mut renderer = SchemaTypes { root: schema, references: Vec::new(), remaining: 4096, names: Default::default() };
    renderer.render(schema, 0)
}

/// Module-scoped aliases preserve recursive object/array references. Pure
/// reference cycles still use the anonymous renderer's bounded fallback.
fn named_type(schema: &Value, prefix: &str) -> (String, String) {
    fn collect(node: &Value, root: &Value, depth: usize, remaining: &mut usize, pointers: &mut std::collections::BTreeSet<String>) {
        if depth >= 64 || *remaining == 0 { return; }
        *remaining -= 1;
        if let Some(reference) = node.get("$ref").and_then(Value::as_str) {
            if let Some(pointer) = local_pointer(reference) {
                if let Some(target) = root.pointer(&pointer) {
                    if matches!(target.get("type").and_then(Value::as_str), Some("object" | "array"))
                        && ["$ref", "oneOf", "anyOf"].iter().all(|key| target.get(key).is_none()) {
                        pointers.insert(pointer);
                    }
                }
            }
        }
        match node {
            Value::Object(map) => for value in map.values() { collect(value, root, depth + 1, remaining, pointers); },
            Value::Array(values) => for value in values { collect(value, root, depth + 1, remaining, pointers); },
            _ => {}
        }
    }
    let mut pointers = std::collections::BTreeSet::new();
    collect(schema, schema, 0, &mut 4096, &mut pointers);
    let names: std::collections::BTreeMap<_,_> = pointers.into_iter().enumerate()
        .map(|(i,pointer)| (pointer,format!("{prefix}Ref{i}"))).collect();
    let mut renderer = SchemaTypes { root: schema, references: Vec::new(), remaining: 4096, names };
    let expression = renderer.render(schema, 0);
    let mut declarations = String::new();
    for (pointer,name) in renderer.names.clone() {
        // One shared work budget bounds the complete declaration, not merely
        // each definition. References retain their names if a body truncates.
        let body = renderer.render(schema.pointer(&pointer).unwrap(), 0);
        declarations.push_str(&format!("  type {name} = {body};\n"));
    }
    (expression, declarations)
}

struct SchemaTypes<'a> {
    root: &'a Value,
    references: Vec<String>,
    remaining: usize,
    names: std::collections::BTreeMap<String, String>,
}

impl SchemaTypes<'_> {
    fn render(&mut self, schema: &Value, depth: usize) -> String {
        // Recursive JSON Schemas cannot be expanded into finite anonymous TS
        // expressions. Bound both depth and total work, including acyclic
        // references that repeatedly expand the same large definition.
        if depth >= 64 || self.remaining == 0 {
            return "unknown".into();
        }
        self.remaining -= 1;
        if schema == &Value::Bool(false) {
            return "never".into();
        }
        if let Some(reference) = schema.get("$ref") {
            let Some(reference) = reference.as_str() else { return "unknown".into() };
            let Some(pointer) = local_pointer(reference) else { return "unknown".into() };
            let referenced = if let Some(name) = self.names.get(&pointer) {
                name.clone()
            } else {
                if self.references.contains(&pointer) { return "unknown".into(); }
                let root = self.root;
                let Some(target) = root.pointer(&pointer) else { return "unknown".into() };
                self.references.push(pointer);
                let rendered = self.render(target, depth + 1);
                self.references.pop();
                rendered
            };
            // Modern JSON Schema permits constraints beside $ref. Preserve
            // representable siblings instead of silently discarding them.
            if ["type", "const", "enum", "oneOf", "anyOf"].iter().any(|key| schema.get(key).is_some()) {
                let mut sibling = schema.clone();
                sibling.as_object_mut().unwrap().remove("$ref");
                return format!("({referenced}) & ({})", self.render(&sibling, depth + 1));
            }
            return referenced;
        }
        if let Some(c) = schema.get("const") {
            return literal(c);
        }
        if let Some(e) = schema.get("enum").and_then(Value::as_array) {
            return if e.is_empty() { "never".into() } else { e.iter().map(literal).collect::<Vec<_>>().join(" | ") };
        }
        if let Some(alts) = schema.get("oneOf").or_else(|| schema.get("anyOf")).and_then(Value::as_array) {
            if alts.is_empty() { return "never".into(); }
            let union = alts.iter().map(|alternative| self.render(alternative, depth + 1)).collect::<Vec<_>>().join(" | ");
            // Serde-flattened tagged enums place common object fields beside
            // the alternatives. Those fields constrain every union member.
            if schema.get("type").and_then(Value::as_str) == Some("object")
                && schema.get("properties").and_then(Value::as_object).is_some_and(|p| !p.is_empty()) {
                return format!("({}) & ({union})", self.object(schema, depth + 1));
            }
            return union;
        }
        match schema.get("type") {
            Some(Value::Array(types)) => {
                if types.is_empty() { return "never".into(); }
                types.iter().map(|kind| {
                    // Retain items/properties/required/additionalProperties:
                    // a nullable array of strings is not an array of unknowns.
                    let mut alternative = schema.clone();
                    alternative["type"] = kind.clone();
                    self.render(&alternative, depth + 1)
                }).collect::<Vec<_>>().join(" | ")
            }
            Some(Value::String(t)) => match t.as_str() {
                "string" => "string".into(),
                "number" | "integer" => "number".into(),
                "boolean" => "boolean".into(),
                "null" => "null".into(),
                "array" => {
                    let item = schema.get("items").map_or_else(|| "unknown".to_string(), |items| self.render(items, depth + 1));
                    if item.contains(' ') { format!("Array<{item}>") } else { format!("{item}[]") }
                }
                "object" => self.object(schema, depth + 1),
                _ => "unknown".into(),
            },
            _ => "unknown".into(),
        }
    }

    fn object(&mut self, schema: &Value, depth: usize) -> String {
        // No declared properties: a dictionary, unless the schema closes it.
        let closed = schema.get("additionalProperties") == Some(&Value::Bool(false));
        let Some(props) = schema.get("properties").and_then(Value::as_object).filter(|props| !props.is_empty()) else {
            return if closed { "Record<string, never>".into() }
                else if let Some(value) = schema.get("additionalProperties").filter(|v| v.is_object()) {
                    format!("Record<string, {}>", self.render(value, depth))
                } else { "Record<string, unknown>".into() };
        };
        let required: Vec<&str> = schema.get("required").and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str).collect();
        let fields: Vec<String> = props.iter().map(|(key, value)| {
            format!("{}{}: {}", property_name(key), if required.contains(&key.as_str()) { "" } else { "?" }, self.render(value, depth))
        }).collect();
        let declared = format!("{{ {} }}", fields.join("; "));
        if schema.get("additionalProperties") == Some(&Value::Bool(true)) {
            return format!("{declared} & Record<string, unknown>");
        }
        // JSON Schema excludes explicit properties from additionalProperties.
        // TypeScript index signatures do not: include declared field types in
        // the index union so envelope fields do not make dictionaries unusable.
        if let Some(additional) = schema.get("additionalProperties").filter(|value| value.is_object()) {
            let mut types = vec![self.render(additional, depth)];
            for (name, value) in props {
                types.push(self.render(value, depth));
                if !required.contains(&name.as_str()) { types.push("undefined".into()); }
            }
            types.sort();
            types.dedup();
            return format!("{declared} & Record<string, {}>", types.join(" | "));
        }
        declared
    }
}

/// A URI-fragment JSON Pointer, decoded before serde_json applies ~0/~1.
/// External references and named anchors are deliberately not fetched/resolved.
fn local_pointer(reference: &str) -> Option<String> {
    let fragment = reference.strip_prefix('#')?;
    let mut bytes = Vec::with_capacity(fragment.len());
    let mut input = fragment.as_bytes().iter().copied();
    while let Some(byte) = input.next() {
        if byte == b'%' {
            let high = (input.next()? as char).to_digit(16)?;
            let low = (input.next()? as char).to_digit(16)?;
            bytes.push((high * 16 + low) as u8);
        } else {
            bytes.push(byte);
        }
    }
    let pointer = String::from_utf8(bytes).ok()?;
    if !pointer.is_empty() && !pointer.starts_with('/') { return None; }
    let mut characters = pointer.chars();
    while let Some(character) = characters.next() {
        if character == '~' && !matches!(characters.next(), Some('0' | '1')) { return None; }
    }
    Some(pointer)
}

fn property_name(key: &str) -> String {
    let mut chars = key.chars();
    if chars.next().is_some_and(|c| c.is_ascii_alphabetic() || c == '_' || c == '$')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '$') {
        key.into()
    } else {
        serde_json::to_string(key).unwrap_or_default()
    }
}

/// `BoundDocument` and `CadClient`, by the same rule the executable
/// bindings are generated by (`cad::sdk`): a document-scoped command is a
/// method of a bound document, a document-creating one a client method
/// answering a bound document; the answer is the output's single
/// non-`context` property when it has one.
fn convenience(catalogue: &Catalogue) -> String {
    use super::sdk::{picked, split};
    let mut bound: std::collections::BTreeMap<&str, Vec<String>> = Default::default();
    let mut free: std::collections::BTreeMap<&str, Vec<String>> = Default::default();
    for d in &catalogue.descriptors {
        let Some((group, verb)) = split(d.path) else { continue };
        let takes_args = d.input_schema.get("properties").and_then(Value::as_object).is_some_and(|p| !p.is_empty());
        let param = if takes_args { format!("args: CommandArgs[{:?}]", d.path) } else { String::new() };
        let answer = match picked(d) {
            Some(p) => format!("CommandOutputs[{:?}][{:?}]", d.path, p),
            None => format!("CommandOutputs[{:?}]", d.path),
        };
        match d.document_scope {
            "one" => bound.entry(group).or_default().push(format!("{verb}({param}): Promise<{answer}>")),
            "creates_document" => free.entry(group).or_default().push(format!("{verb}({param}): Promise<BoundDocument>")),
            _ if d.execution == "operation" => free.entry(group).or_default().push(format!("{verb}({param}): Promise<{answer}>")),
            _ if d.execution == "input" => free.entry(group).or_default().push(format!("{verb}({param}): Promise<{answer}>")),
            _ => {}
        }
    }
    free.entry("documents").or_default().push("bind(context: DocumentContext): Promise<BoundDocument>".into());
    let block = |m: &std::collections::BTreeMap<&str, Vec<String>>| {
        m.iter().map(|(g, ms)| format!("    {g}: {{ {} }};", ms.join("; "))).collect::<Vec<_>>().join("\n")
    };
    format!(
        "  export interface BoundDocument {{\n    readonly context: DocumentContext;\n{}\n  }}\n  export interface CadClient {{\n    discovery: {{ search(args: SearchRequest): Promise<SearchPage>; describe(args: DescribeRequest): Promise<DescribeResult> }};\n{}\n  }}",
        block(&bound),
        block(&free)
    )
}

/// `brep`: every script-callable registry tool, typed from its input schema
/// and output schema, including scoped aliases for recursive DTOs.
fn tool_declarations(tools: &[super::ScriptTool]) -> String {
    use super::sdk::{ident, summary};
    let mut groups: std::collections::BTreeMap<String, Vec<String>> = Default::default();
    let mut aliases = String::new();
    for (index, t) in tools.iter().enumerate() {
        let (input, input_aliases) = named_type(&t.input_schema, &format!("BrepTool{index}Input"));
        let (output, output_aliases) = named_type(&t.output_schema, &format!("BrepTool{index}Output"));
        aliases.push_str(&input_aliases); aliases.push_str(&output_aliases);
        let takes_args = t.input_schema.get("properties").and_then(Value::as_object).is_some_and(|p| !p.is_empty());
        let required = t.input_schema.get("required").and_then(Value::as_array).is_some_and(|r| !r.is_empty());
        let param = if !takes_args { format!("args?: {}", input) } else if required { format!("args: {}", input) } else { format!("args?: {}", input) };
        groups.entry(ident(&t.group)).or_default().push(format!("      /** {} */\n      {}({param}): Promise<{}>;", summary(&t.doc), crate::script_sdk::method_name(&t.group, &t.name), output));
    }
    let body = groups.iter().map(|(g, ms)| format!("    {g}: {{\n{}\n    }};", ms.join("\n"))).collect::<Vec<_>>().join("\n");
    format!("{aliases}  /** The app's tool registry as script functions: each runs the tool's own handler. */\n  export const brep: {{\n{body}\n  }};")
}

/// The whole module declaration.
pub fn module(catalogue: &Catalogue, tools: &[super::ScriptTool]) -> String {
    let mut args = String::new();
    let mut outputs = String::new();
    let mut aliases = String::new();
    for (index, d) in catalogue.descriptors.iter().enumerate() {
        let (input, input_aliases) = named_type(&d.input_schema, &format!("BrepCommand{index}Input"));
        let (output, output_aliases) = named_type(&d.output_schema, &format!("BrepCommand{index}Output"));
        aliases.push_str(&input_aliases); aliases.push_str(&output_aliases);
        args.push_str(&format!("    {:?}: {};\n", d.path, input));
        outputs.push_str(&format!("    {:?}: {};\n", d.path, output));
    }
    format!(
        r#"// GENERATED by `brep-mcp schema --write <output-directory>` from
// the cad.* catalogue (manifest {hash}); do not edit.
declare module "@brep/cad" {{
{aliases}  export type DocumentId = string;
  export type Revision = string;
  export type OperationId = string;
  export type ExecutionId = string;
  export type ArtifactId = string;
  export type Sequence = string;
  export type Timestamp = string;
  export type CommandPath = {paths};
  export type Json = null | boolean | number | string | Json[] | {{ [key: string]: Json }};

  export interface DocumentContext {{ documentId: DocumentId; expectedRevision: Revision; }}
  export interface DocumentInfo {{ context: DocumentContext; title: string; lifecycle: "open" | "closing"; dirty: boolean; }}
  export interface Vec3 {{ x: number; y: number; z: number; }}
  export interface EntityRef {{ documentId: DocumentId; entityId: string; kind: "sketch" | "solid"; }}
  export interface ArtifactRef {{ artifactId: ArtifactId; mediaType: string; byteLength: number; sha256: string; expiresAt: Timestamp; uri: string; chunkUriTemplate: string; }}
  export interface ModelSummary {{
    context: DocumentContext; solidCount: number; valid: boolean;
    boundsMm: {{ min: Vec3; max: Vec3 }} | null; volumeMm3: number | null; warnings: string[];
  }}
  export interface ManifestIdentity {{ apiVersion: "1"; sha256: string; }}

  export type Phase = "validation" | "authorization" | "admission" | "queue" | "running" | "evaluation" | "commit"
    | "external_io" | "compilation" | "execution" | "delivery" | "lookup" | "input" | "completion" | "recovery";
  export type RetryAdvice = "do_not_retry" | "query_operation" | "retry_same_key_within_retention" | "inspect_then_new_submission";
  export type CommitDisposition = "none" | "committed" | "partial" | "unknown";
  export interface Effects {{
    disposition: CommitDisposition;
    documents: Array<{{ documentId: DocumentId; beforeRevision: Revision | null; afterRevision: Revision | null; changed: boolean }}>;
    external: Array<{{ kind: "artifact_created" | "export_written" | "import_read"; state: "completed" | "partial" | "unknown"; artifactId: ArtifactId | null }}>;
  }}
  export type ErrorCode = "invalid_arguments" | "unsupported_version" | "manifest_mismatch" | "method_unknown" | "capability_denied"
    | "document_unknown" | "document_closed" | "document_not_active" | "document_busy" | "document_mismatch" | "revision_conflict"
    | "overloaded" | "idempotency_conflict" | "idempotency_expired" | "operation_unknown" | "execution_unknown" | "handle_epoch_mismatch"
    | "not_ready" | "cancelled" | "geometry_refused" | "evaluation_failed" | "compile_failed" | "script_failed" | "job_failed"
    | "deadline_exceeded" | "resource_limit" | "external_io_failed" | "artifact_unknown" | "artifact_expired" | "host_stopping"
    | "host_stopped" | "internal_error" | "input_required" | "input_declined" | "input_expired"
    | "persistence_failed" | "reconciliation_required";
  export interface ApiError {{
    code: ErrorCode; message: string; phase: Phase; correlationId: string;
    operationId: OperationId | null; executionId: ExecutionId | null; context: DocumentContext | null;
    effects: Effects; retry: RetryAdvice;
  }}
  export type Response<T> = {{ ok: true; value: T }} | {{ ok: false; error: ApiError }};

  /** Each command's arguments, from its descriptor's inputSchema. */
  export interface CommandArgs {{
{args}  }}
  /** Each command's typed output, from its descriptor's outputSchema. */
  export interface CommandOutputs {{
{outputs}  }}

  export type OperationState = "queued" | "running" | "evaluating" | "committing" | "succeeded" | "failed" | "cancelled";
  export interface Retention {{ scope: "process_epoch"; epoch: string; retainUntil: Timestamp | null; terminalRetentionMs: number; artifactRetentionMs: number; }}
  export interface Admission {{ operationId: OperationId; acceptedAt: Timestamp; context: DocumentContext | null; state: OperationState; sequence: Sequence; retention: Retention; }}
  export interface OperationStatus extends Admission {{
    command: CommandPath;
    cancellation: {{ requested: boolean; effectiveNow: boolean; nonPreemptible: boolean }};
    progress: {{ stage: string; completed: number | null; total: number | null; summary: string }};
  }}
  export interface OperationResult<T = unknown> {{
    operationId: OperationId; state: "succeeded" | "failed" | "cancelled"; sequence: Sequence; accepted: true; finishedAt: Timestamp;
    effects: Effects; geometry: {{ status: "not_applicable" | "succeeded" | "refused" | "partial" | "unknown"; report: ArtifactRef | null; summary: string }};
    output: T | null; error: ApiError | null; retention: Retention;
  }}
  export interface CancelAck {{ operationId: OperationId; disposition: "requested" | "cancelled_before_start" | "too_late" | "already_terminal"; status: OperationStatus; }}
  export interface SearchRequest {{ query: string; limit?: number; cursor?: string; }}
  export type DiscoveryPath = CommandPath | `brep.${{string}}.${{string}}`;
  export interface SearchPage {{ manifest: ManifestIdentity; methods: Array<{{ path: DiscoveryPath; summary: string }}>; nextCursor: string | null; }}
  export interface DescribeRequest {{ paths: DiscoveryPath[]; }}
  export interface DescribeResult {{ manifest: ManifestIdentity; methods: unknown[]; }}
  export interface SubmitRequest<P extends CommandPath = CommandPath> {{
    manifest: ManifestIdentity; command: P; args: P extends keyof CommandArgs ? CommandArgs[P] : Json; context: DocumentContext | null; idempotencyKey: string;
  }}
  export interface DiscoveryService {{
    search(req: SearchRequest): Promise<Response<SearchPage>>;
    describe(req: DescribeRequest): Promise<Response<DescribeResult>>;
  }}
  export interface DocumentsService {{
    list(req?: Record<string, never>): Promise<Response<{{ documents: DocumentInfo[] }}>>;
    get(req: {{ documentId: DocumentId }}): Promise<Response<DocumentInfo>>;
  }}
  export interface OperationsService {{
    issueKey(req: {{ purpose: "command" | "execution" }}): Promise<Response<{{ key: string; firstAdmissionExpiresAt: Timestamp }}>>;
    submit<P extends CommandPath>(req: SubmitRequest<P>): Promise<Response<Admission>>;
    status(req: {{ operationId: OperationId; afterSequence?: Sequence }}): Promise<Response<{{ changed: boolean; status: OperationStatus }}>>;
    result<P extends keyof CommandOutputs>(req: {{ operationId: OperationId }}): Promise<Response<OperationResult<CommandOutputs[P]>>>;
    cancel(req: {{ operationId: OperationId }}): Promise<Response<CancelAck>>;
  }}
  export interface CommandsService {{
    query(req: {{ manifest: ManifestIdentity; command: "cad.model.inspect"; args: Record<string, never>; context: DocumentContext }}): Promise<Response<{{ context: DocumentContext; output: ModelSummary }}>>;
  }}
  export interface ExecutionService {{
    submit(req: unknown): Promise<Response<unknown>>;
    status(req: {{ executionId: ExecutionId }}): Promise<Response<unknown>>;
    result(req: {{ executionId: ExecutionId }}): Promise<Response<unknown>>;
    cancel(req: {{ executionId: ExecutionId }}): Promise<Response<unknown>>;
  }}
  export interface ArtifactsService {{
    read(req: {{ artifactId: ArtifactId; offset?: number; limit?: number }}): Promise<Response<{{ artifact: ArtifactRef; offset: number; bytesBase64: string; eof: boolean }}>>;
    /** The bytes, or (the local user's run_script: `host.files.read`) a file on the host. */
    create(req: {{ mediaType: "model/step"; bytesBase64: string }} | {{ mediaType: "model/step"; path: string }}): Promise<Response<ArtifactRef>>;
    begin(req: {{ mediaType: "model/step"; byteLength: number; sha256: string }}): Promise<Response<{{ uploadId: ArtifactId; maxChunkBytes: number; expiresAt: Timestamp }}>>;
    append(req: {{ uploadId: ArtifactId; offset: number; bytesBase64: string }}): Promise<Response<{{ uploadId: ArtifactId; received: number; byteLength: number }}>>;
    commit(req: {{ uploadId: ArtifactId }}): Promise<Response<ArtifactRef>>;
  }}
  export interface CadService {{
    inputs: {{ exportOptions(req: CommandArgs["cad.inputs.exportOptions"]): Promise<Response<CommandOutputs["cad.inputs.exportOptions"]>> }};
    discovery: DiscoveryService; documents: DocumentsService; operations: OperationsService;
    commands: CommandsService; code: ExecutionService; artifacts: ArtifactsService;
  }}

{bound_and_client}
  export class ApiFailure extends Error {{
    readonly name: "ApiFailure";
    readonly error: ApiError;
    readonly operationId: OperationId | null;
    readonly executionId: ExecutionId | null;
    constructor(error: ApiError);
  }}
{tools_decl}
  export const manifest: ManifestIdentity;
  export const cad: CadClient;
  export const service: CadService;
  export default cad;
}}
"#,
        hash = catalogue.sha256,
        bound_and_client = convenience(catalogue),
        tools_decl = tool_declarations(tools),
        paths = catalogue.descriptors.iter().map(|d| format!("{:?}", d.path)).collect::<Vec<_>>().join(" | "),
    )
}

