//! The authoritative `cad.*` command catalogue (contract §4, §8): one
//! descriptor per command, the canonical manifest and its SHA-256, and
//! lexical search / exact describe. The SDK, the MCP tools and the code-mode
//! bridge all read these same descriptors; nothing else lists commands.
use super::canon;
use super::wire::ApiError;
use serde_json::{json, Value};

pub const API_VERSION: &str = "1";

#[derive(Debug, Clone)]
pub struct Descriptor {
    pub path: &'static str,
    pub summary: &'static str,
    pub documentation: &'static str,
    pub input_schema: Value,
    pub output_schema: Value,
    pub effect_classes: &'static [&'static str],
    pub document_scope: &'static str,
    pub execution: &'static str,
    pub revision_policy: &'static str,
    pub background_policy: &'static str,
    pub required_capabilities: &'static [&'static str],
}

impl Descriptor {
    pub fn to_json(&self) -> Value {
        json!({
            "path": self.path,
            "summary": self.summary,
            "documentation": self.documentation,
            "inputSchema": self.input_schema,
            "outputSchema": self.output_schema,
            "outputKnown": true,
            "effectClasses": self.effect_classes,
            "documentScope": self.document_scope,
            "execution": self.execution,
            "revisionPolicy": self.revision_policy,
            "backgroundPolicy": self.background_policy,
            "requiredCapabilities": self.required_capabilities,
            "compatibilityNames": [],
        })
    }
}

fn obj(props: Value, required: &[&str]) -> Value {
    json!({ "type": "object", "properties": props, "required": required, "additionalProperties": false })
}

fn context_schema() -> Value {
    obj(
        json!({
            "documentId": { "type": "string", "pattern": "^doc_[A-Za-z0-9_-]{22}_[A-Za-z0-9_-]{22}$" },
            "expectedRevision": { "type": "string", "pattern": "^([1-9][0-9]*|0)$" },
        }),
        &["documentId", "expectedRevision"],
    )
}

fn entity_ref(kind: Option<&str>) -> Value {
    let kinds = kind.map_or(json!(["sketch", "solid"]), |k| json!([k]));
    obj(
        json!({
            "documentId": { "type": "string" },
            "entityId": { "type": "string", "minLength": 1, "maxLength": 256 },
            "kind": { "enum": kinds },
        }),
        &["documentId", "entityId", "kind"],
    )
}

fn artifact_ref() -> Value {
    obj(
        json!({
            "artifactId": { "type": "string" },
            "mediaType": { "type": "string" },
            "byteLength": { "type": "integer", "minimum": 0 },
            "sha256": { "type": "string", "pattern": "^[0-9a-f]{64}$" },
            "expiresAt": { "type": "string" },
            "uri": { "type": "string" },
            "chunkUriTemplate": { "type": "string" },
        }),
        &["artifactId", "mediaType", "byteLength", "sha256", "expiresAt", "uri", "chunkUriTemplate"],
    )
}

fn document_info() -> Value {
    obj(
        json!({
            "context": context_schema(),
            "title": { "type": "string" },
            "lifecycle": { "enum": ["open", "closing"] },
            "dirty": { "type": "boolean" },
        }),
        &["context", "title", "lifecycle", "dirty"],
    )
}

fn vec3() -> Value {
    obj(json!({ "x": { "type": "number" }, "y": { "type": "number" }, "z": { "type": "number" } }), &["x", "y", "z"])
}

fn model_summary() -> Value {
    obj(
        json!({
            "context": context_schema(),
            "solidCount": { "type": "integer", "minimum": 0 },
            "valid": { "type": "boolean" },
            "boundsMm": { "oneOf": [{ "type": "null" }, obj(json!({ "min": vec3(), "max": vec3() }), &["min", "max"])] },
            "volumeMm3": { "type": ["number", "null"] },
            "warnings": { "type": "array", "items": { "type": "string" } },
        }),
        &["context", "solidCount", "valid", "boundsMm", "volumeMm3", "warnings"],
    )
}

const MM: f64 = 1_000_000.0;

pub fn descriptors() -> Vec<Descriptor> {
    let dim = || json!({ "type": "number", "exclusiveMinimum": 0, "maximum": MM });
    vec![
        Descriptor {
            path: "cad.inputs.exportOptions",
            summary: "Validate export choices and ask a supporting client for missing format/path fields; writes no file.",
            documentation: "Returns {format,path}. Supplied choices are preserved. Missing choices require a modern Tasks client with form elicitation; otherwise supply both fields explicitly. Only formats offered by the live document_export function are accepted. Decline, cancellation and expiry refuse without exporting. Inspect the active document before separately calling document_export.",
            input_schema: obj(json!({"format":{"type":"string"},"path":{"type":"string","minLength":1,"maxLength":4096}}), &[]),
            output_schema: obj(json!({"format":{"type":"string"},"path":{"type":"string"}}), &["format","path"]),
            effect_classes: &[], document_scope: "none", execution: "input",
            revision_policy: "none", background_policy: "requires_client_when_missing", required_capabilities: &[],
        },
        Descriptor {
            path: "cad.documents.create",
            summary: "Create a new, empty millimetre part document.",
            documentation: "Opens a new part document (a datum and its XY/XZ/YZ planes) titled `title`, behind the visible document (it does not take focus). Returns its DocumentInfo; its `context` is what the next write names. Needs document.create.",
            input_schema: obj(json!({ "title": { "type": "string", "minLength": 1, "maxLength": 128 }, "units": { "const": "mm" } }), &["title", "units"]),
            output_schema: document_info(),
            effect_classes: &["document_lifecycle"],
            document_scope: "creates_document",
            execution: "operation",
            revision_policy: "none",
            background_policy: "engine_only",
            required_capabilities: &["document.create"],
        },
        Descriptor {
            path: "cad.documents.close",
            summary: "Close a document by identity.",
            documentation: "Closes the named document at its expected revision. A dirty document refuses unless `discardUnsaved`; a document with another operation in flight answers document_busy and stays open. Needs document.close.",
            input_schema: obj(json!({ "discardUnsaved": { "type": "boolean", "default": false } }), &[]),
            output_schema: obj(json!({ "documentId": { "type": "string" }, "closed": { "const": true } }), &["documentId", "closed"]),
            effect_classes: &["document_lifecycle"],
            document_scope: "one",
            execution: "operation",
            revision_policy: "require_expected",
            background_policy: "engine_only",
            required_capabilities: &["document.close"],
        },
        Descriptor {
            path: "cad.sketch.create",
            summary: "Create a centred rectangle sketch on a base plane.",
            documentation: "Adds a sketch on XY, XZ or YZ (right-handed: XY normal +Z, XZ normal -Y, YZ normal +X) holding one rectangle of widthMm × heightMm centred on the origin. Commits only if the sketch evaluates. Needs document.write.",
            input_schema: obj(
                json!({
                    "plane": { "enum": ["XY", "XZ", "YZ"] },
                    "profile": obj(json!({ "kind": { "const": "rectangle" }, "widthMm": dim(), "heightMm": dim() }), &["kind", "widthMm", "heightMm"]),
                }),
                &["plane", "profile"],
            ),
            output_schema: obj(json!({ "context": context_schema(), "sketch": entity_ref(Some("sketch")) }), &["context", "sketch"]),
            effect_classes: &["document_write"],
            document_scope: "one",
            execution: "operation",
            revision_policy: "require_expected",
            background_policy: "engine_only",
            required_capabilities: &["document.write"],
        },
        Descriptor {
            path: "cad.solids.extrude",
            summary: "Extrude a sketch along its plane normal into a new solid.",
            documentation: "Extrudes the named sketch of the same document distanceMm along its plane's positive normal into a new solid. Commits only if a solid results. Needs document.write.",
            input_schema: obj(json!({ "sketch": entity_ref(Some("sketch")), "distanceMm": dim() }), &["sketch", "distanceMm"]),
            output_schema: obj(json!({ "context": context_schema(), "solid": entity_ref(Some("solid")) }), &["context", "solid"]),
            effect_classes: &["document_write"],
            document_scope: "one",
            execution: "operation",
            revision_policy: "require_expected",
            background_policy: "engine_only",
            required_capabilities: &["document.write"],
        },
        Descriptor {
            path: "cad.model.inspect",
            summary: "Summarise a document's solids: count, validity, bounds, volume.",
            documentation: "An ordered read of the named document, tagged with its current revision (an older expectedRevision is allowed and is NOT upgraded). `valid` requires every solid to pass the kernel's topology validation; numbers the kernel cannot give are null with a warning. Needs document.read.",
            input_schema: obj(json!({}), &[]),
            output_schema: model_summary(),
            effect_classes: &["document_read"],
            document_scope: "one",
            execution: "query",
            revision_policy: "tag_current",
            background_policy: "engine_only",
            required_capabilities: &["document.read"],
        },
        Descriptor {
            path: "cad.files.export",
            summary: "Export solids of a document as a STEP artifact.",
            documentation: "Writes the named solids (1–256; all of them or any subset) of a part document at its expected revision as STEP into a new scoped artifact; never a host path. Needs document.read, cad.export and artifact.write.",
            input_schema: obj(
                json!({ "format": { "const": "step" }, "entities": { "type": "array", "minItems": 1, "maxItems": 256, "items": entity_ref(Some("solid")) } }),
                &["format", "entities"],
            ),
            output_schema: obj(json!({ "context": context_schema(), "artifact": artifact_ref() }), &["context", "artifact"]),
            effect_classes: &["document_read", "external_write"],
            document_scope: "one",
            execution: "operation",
            revision_policy: "require_expected",
            background_policy: "engine_only",
            required_capabilities: &["document.read", "cad.export", "artifact.write"],
        },
        Descriptor {
            path: "cad.files.import",
            summary: "Import a STEP artifact into a document as solids.",
            documentation: "Reads an authorised STEP artifact, appends it to the document as an import feature, and commits it only if it evaluates to at least one solid. Returns the imported solids. Needs document.write, cad.import and artifact.read.",
            input_schema: obj(json!({ "artifactId": { "type": "string" }, "format": { "const": "step" } }), &["artifactId", "format"]),
            output_schema: obj(json!({ "context": context_schema(), "solids": { "type": "array", "items": entity_ref(Some("solid")) } }), &["context", "solids"]),
            effect_classes: &["external_read", "document_write"],
            document_scope: "one",
            execution: "operation",
            revision_policy: "require_expected",
            background_policy: "engine_only",
            required_capabilities: &["document.write", "cad.import", "artifact.read"],
        },
    ]
}

/// The catalogue as served: descriptors sorted by exact path, the canonical
/// manifest bytes and their hash.
pub struct Catalogue {
    pub descriptors: Vec<Descriptor>,
    pub manifest_json: Value,
    pub sha256: String,
}

impl Catalogue {
    pub fn new() -> Self {
        let mut descriptors = descriptors();
        descriptors.sort_by(|a, b| a.path.cmp(b.path));
        let manifest_json = json!({
            "apiVersion": API_VERSION,
            "methods": descriptors.iter().map(Descriptor::to_json).collect::<Vec<_>>(),
        });
        let bytes = canon::canonical(&manifest_json).expect("the catalogue is canonical JSON");
        let sha256 = canon::sha256_hex(bytes.as_bytes());
        Catalogue { descriptors, manifest_json, sha256 }
    }

    pub fn identity(&self) -> Value {
        json!({ "apiVersion": API_VERSION, "sha256": self.sha256 })
    }

    pub fn get(&self, path: &str) -> Option<&Descriptor> {
        self.descriptors.iter().find(|d| d.path == path)
    }

    /// The submitted manifest must be this one (§4): checked at first
    /// admission only, never for a retained retry.
    pub fn check_manifest(&self, manifest: &Value) -> Result<(), ApiError> {
        let version = manifest.get("apiVersion").and_then(Value::as_str);
        let hash = manifest.get("sha256").and_then(Value::as_str);
        if version != Some(API_VERSION) {
            return Err(ApiError::new("unsupported_version", "validation", "apiVersion must be \"1\""));
        }
        if hash != Some(self.sha256.as_str()) {
            return Err(ApiError::new("manifest_mismatch", "validation", "the manifest hash is not this server's catalogue; rediscover with cad_describe"));
        }
        Ok(())
    }

    /// Lexical, case-insensitive search over path, summary and documentation
    /// (§4): ranked path hits, then summary, then documentation, ties by path.
    pub fn search(&self, query: &str, limit: Option<u64>, cursor: Option<&str>) -> Result<Value, ApiError> {
        let chars = query.chars().count();
        if !(1..=256).contains(&chars) {
            return Err(ApiError::new("invalid_arguments", "validation", "query must be 1–256 characters"));
        }
        let limit = limit.unwrap_or(10);
        if !(1..=20).contains(&limit) {
            return Err(ApiError::new("invalid_arguments", "validation", "limit must be 1–20"));
        }
        let q = query.to_lowercase();
        let terms: Vec<&str> = q.split_whitespace().collect();
        let mut hits: Vec<(u32, &Descriptor)> = self
            .descriptors
            .iter()
            .filter_map(|d| {
                let (p, s, doc) = (d.path.to_lowercase(), d.summary.to_lowercase(), d.documentation.to_lowercase());
                let mut score = 0;
                for t in &terms {
                    score += if p.contains(t) { 100 } else if s.contains(t) { 10 } else if doc.contains(t) { 1 } else { 0 };
                }
                (score > 0).then_some((score, d))
            })
            .collect();
        hits.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.path.cmp(b.1.path)));
        // The cursor binds query and manifest: `<offset>:<hash of query+manifest>`.
        let binding = &canon::sha256_hex(format!("{q}\u{0}{}", self.sha256).as_bytes())[..16];
        let start = match cursor {
            None => 0,
            Some(c) => {
                let (offset, bound) = c.split_once(':').ok_or_else(|| ApiError::new("invalid_arguments", "validation", "cursor is not from this search"))?;
                if bound != binding {
                    return Err(ApiError::new("invalid_arguments", "validation", "cursor belongs to another query or manifest"));
                }
                offset.parse::<usize>().map_err(|_| ApiError::new("invalid_arguments", "validation", "cursor is not from this search"))?
            }
        };
        let page: Vec<Value> = hits.iter().skip(start).take(limit as usize).map(|(_, d)| json!({ "path": d.path, "summary": d.summary })).collect();
        let next = start + page.len();
        Ok(json!({
            "manifest": self.identity(),
            "methods": page,
            "nextCursor": if next < hits.len() { Value::String(format!("{next}:{binding}")) } else { Value::Null },
        }))
    }

    pub fn describe(&self, paths: &[String]) -> Result<Value, ApiError> {
        if !(1..=8).contains(&paths.len()) {
            return Err(ApiError::new("invalid_arguments", "validation", "describe takes 1–8 exact paths"));
        }
        let mut methods = Vec::new();
        for p in paths {
            let d = self.get(p).ok_or_else(|| ApiError::new("method_unknown", "lookup", format!("no method `{p}`; search for its exact path")))?;
            methods.push(d.to_json());
        }
        let out = json!({ "manifest": self.identity(), "methods": methods });
        if out.to_string().len() > 64 << 10 {
            return Err(ApiError::new("resource_limit", "validation", "describe fewer paths at once"));
        }
        Ok(out)
    }
}

impl Default for Catalogue {
    fn default() -> Self {
        Self::new()
    }
}

