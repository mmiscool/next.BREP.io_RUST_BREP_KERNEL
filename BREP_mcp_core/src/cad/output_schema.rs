//! Public execution-tool reply contracts. Arbitrary program output and error
//! context stay JSON; the server-owned envelope and lifecycle fields are typed.
use serde_json::{json, Value};

fn object(properties: Value, required: &[&str]) -> Value {
    json!({"type":"object","properties":properties,"required":required,"additionalProperties":false})
}
fn reference(name: &str) -> Value { json!({"$ref":format!("#/$defs/{name}")}) }
fn array(items: Value) -> Value { json!({"type":"array","items":items}) }
fn nullable(schema: Value) -> Value { json!({"anyOf":[schema,{"type":"null"}]}) }
fn text() -> Value { json!({"type":"string"}) }
fn sequence() -> Value { json!({"type":"string","pattern":"^(0|[1-9][0-9]*)$","description":"Decimal sequence; retain as a string without floating-point conversion."}) }

pub fn execution(halt: bool) -> Value {
    let string_list = array(text());
    let effects = object(json!({
        "disposition":{"enum":["none","committed","partial","unknown"]},
        "documents":array(object(json!({"documentId":text(),"beforeRevision":nullable(text()),"afterRevision":nullable(text()),"changed":{"type":"boolean"}}), &["documentId","beforeRevision","afterRevision","changed"])),
        "external":array(object(json!({"kind":text(),"state":text(),"artifactId":text()}), &["kind","state","artifactId"]))
    }), &["disposition","documents","external"]);
    let error = object(json!({
        "code":text(),"message":text(),"phase":text(),"correlationId":text(),
        "operationId":nullable(text()),"executionId":nullable(text()),
        "context":{"description":"Error-specific JSON context; may contain diagnostics or reconciliation details."},
        "effects":reference("effects"),
        "retry":{"enum":["retry_same_key_within_retention","inspect_then_new_submission","do_not_retry"]}
    }), &["code","message","phase","correlationId","operationId","executionId","context","effects","retry"]);
    let mut limit_properties = serde_json::Map::new();
    let limit_names = ["sourceBytes","inputBytes","compileMs","compileHeapBytes","bridgeBytes","jsCpuMs","jsHeapBytes","wallMs","outstandingHostCalls","logBytes","resultBytes","artifactCount","artifactBytes"];
    for name in limit_names { limit_properties.insert(name.into(),json!({"type":"integer","minimum":1})); }
    let persistence = object(json!({"durable":{"const":true},"automaticReplay":{"const":false},"referencedArtifactsAndHandlesMayBeUnavailable":{"type":"boolean"}}), &["durable","automaticReplay"]);
    let status = object(json!({
        "executionId":text(),"manifest":object(json!({"apiVersion":text(),"sha256":text()}), &["apiVersion","sha256"]),
        "acceptedAt":text(),"updatedAt":text(),"state":{"enum":["queued","compiling","running","waiting","input_required","succeeded","failed","cancelled"]},
        "sequence":sequence(),"typeChecking":reference("typeChecking"),
        "effectiveCapabilities":array(object(json!({"name":text(),"documentIds":string_list,"allowCreatedDocuments":{"type":"boolean"},"allowExecutionArtifacts":{"type":"boolean"},"formats":array(text()),"maxBytes":{"type":"integer"},"controlScope":text()}), &["name"])),
        "effectiveLimits":object(Value::Object(limit_properties), &limit_names),
        "cancellationRequested":{"type":"boolean"},"acceptedOperations":string_list,"pendingOperations":string_list,
        "retainUntil":nullable(text()),"persistence":reference("persistence")
    }), &["executionId","manifest","acceptedAt","updatedAt","state","sequence","typeChecking","effectiveCapabilities","effectiveLimits","cancellationRequested","acceptedOperations","pendingOperations","retainUntil"]);
    let artifact = object(json!({"artifactId":text(),"uri":text(),"chunkUriTemplate":text(),"mediaType":text(),"byteLength":{"type":"integer","minimum":0},"sha256":text(),"expiresAt":text()}), &["artifactId","uri","chunkUriTemplate","mediaType","byteLength","sha256","expiresAt"]);
    let operation = object(json!({"operationId":text(),"state":{"enum":["queued","running","evaluating","committing","succeeded","failed","cancelled"]},"effects":reference("effects"),"retainUntil":nullable(text())}), &["operationId","state","effects","retainUntil"]);
    let observation = object(json!({"operation_id":text(),"state":nullable(text()),"effects":nullable(text())}), &["operation_id","state","effects"]);
    let completion = object(json!({"reply_received":{"type":"boolean"},"ok":{"type":["boolean","null"]},"operation":nullable(observation)}), &["reply_received","ok","operation"]);
    let call = object(json!({"call_id":{"type":"integer","minimum":0},"method":text(),"completion":nullable(completion)}), &["call_id","method","completion"]);
    let evidence = object(json!({"hostCallRecordingEnabled":{"type":"boolean"},"operationsEvidenceComplete":{"const":false},"automaticReplay":{"const":false},"calls":array(call),"unacknowledgedCallIds":array(json!({"type":"integer","minimum":0})),"observationsMayBeStale":{"const":true},"interrupted":{"type":"boolean"}}), &["hostCallRecordingEnabled","operationsEvidenceComplete","automaticReplay","calls","unacknowledgedCallIds","observationsMayBeStale"]);
    let terminal = object(json!({
        "executionId":text(),"state":{"enum":["succeeded","failed","cancelled"]},"sequence":sequence(),"finishedAt":text(),"typeChecking":reference("typeChecking"),
        "output":{"description":"The program's arbitrary JSON return value, or null when unavailable."},"error":nullable(reference("error")),
        "operations":array(operation),"pendingOperationIds":string_list,"artifacts":array(reference("artifact")),"logArtifact":nullable(reference("artifact")),"retainUntil":nullable(text()),
        "persistence":reference("persistence"),"hostCallEvidence":reference("hostCallEvidence"),"recovery":reference("hostCallEvidence")
    }), &["executionId","state","sequence","finishedAt","typeChecking","output","error","operations","pendingOperationIds","artifacts","logArtifact","retainUntil"]);
    let running = object(json!({"running":{"const":true},"status":reference("status")}), &["running","status"]);
    let value = if halt { reference("status") } else { json!({"oneOf":[reference("terminal"),reference("running")]}) };
    json!({"$schema":"https://json-schema.org/draft/2020-12/schema","type":"object",
        "description":"Structured execution reply. ok describes the query/admission envelope; a terminal execution may itself have failed. Transport errors without structured content are MCP errors, not this envelope.",
        "oneOf":[object(json!({"ok":{"const":true},"value":value}), &["ok","value"]),object(json!({"ok":{"const":false},"error":reference("error")}), &["ok","error"])],
        "$defs":{"effects":effects,"error":error,"status":status,"terminal":terminal,"running":running,"artifact":artifact,"persistence":persistence,"hostCallEvidence":evidence,
            "typeChecking":{"enum":["not_applicable","pending","passed","failed"]}}
    })
}
