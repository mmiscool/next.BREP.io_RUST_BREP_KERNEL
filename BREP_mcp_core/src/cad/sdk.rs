//! The `@brep/cad` module, generated from the catalogue (contract §4: the
//! SDK's executable bindings and its declarations come from the same
//! manifest). A hand-written core (`sdk_core.js`: the bridge, `Response`
//! unwrapping, `ApiFailure`, the low-level `service`, one operation's
//! lifecycle) is followed by bindings derived from each descriptor:
//!
//! - path `cad.<group>.<verb>`;
//! - `documentScope: "one"` → `BoundDocument.<group>.<verb>(args)`, sent with
//!   the bound context; a write advances the handle's expectation from its
//!   own committed output, a read never does;
//! - `documentScope: "creates_document"` → `cad.<group>.<verb>(args)`,
//!   answering a `BoundDocument`;
//! - `execution: "query"` → `service.commands.query`, else one operation;
//! - the answer is the output's single non-`context` property when it has
//!   exactly one (`sketch`, `solid`, `artifact`, `solids`), else the output.
use super::catalogue::{Catalogue, Descriptor};
use serde_json::Value;

pub const CORE: &str = include_str!("sdk_core.js");

/// `(group, verb)` of `cad.<group>.<verb>`.
pub fn split(path: &str) -> Option<(&str, &str)> {
    let rest = path.strip_prefix("cad.")?;
    rest.split_once('.')
}

/// The property a convenience call answers with, if the output has exactly
/// one besides `context`.
pub fn picked(d: &Descriptor) -> Option<String> {
    let props = d.output_schema.get("properties").and_then(Value::as_object)?;
    let rest: Vec<&String> = props.keys().filter(|k| *k != "context").collect();
    (rest.len() == 1).then(|| rest[0].clone())
}

fn is_write(d: &Descriptor) -> bool {
    d.execution == "operation"
}

pub use crate::script_sdk::{ident, summary};
use crate::script_sdk::tool_bindings;

/// The whole module source.
pub fn module(catalogue: &Catalogue, tools: &[super::ScriptTool]) -> String {
    let mut bound: std::collections::BTreeMap<&str, Vec<String>> = Default::default();
    let mut free: std::collections::BTreeMap<&str, Vec<String>> = Default::default();
    for d in &catalogue.descriptors {
        let Some((group, verb)) = split(d.path) else { continue };
        let pick = picked(d).map_or("(o) => o".to_string(), |p| format!("(o) => o.{p}"));
        let takes_args = d.input_schema.get("properties").and_then(Value::as_object).is_some_and(|p| !p.is_empty());
        let param = if takes_args { "args" } else { "" };
        let arg = if takes_args { "args" } else { "{}" };
        match d.document_scope {
            "one" if d.execution == "query" => bound.entry(group).or_default().push(format!(
                "{verb}: ({param}) => self.#query({:?}, {arg}, {pick})",
                d.path
            )),
            "one" => bound.entry(group).or_default().push(format!(
                "{verb}: ({param}) => self.#write({:?}, {arg}, {pick})",
                d.path
            )),
            "creates_document" => free.entry(group).or_default().push(format!(
                "async {verb}({param}) {{ const info = await operate({:?}, {arg}, null); return new BoundDocument(info.context); }}",
                d.path
            )),
            _ if is_write(d) => free.entry(group).or_default().push(format!(
                "{verb}: ({param}) => operate({:?}, {arg}, null).then({pick})",
                d.path
            )),
            _ if d.execution == "input" => free.entry(group).or_default().push(format!(
                "{verb}: async ({param}) => unwrap(await service.{group}.{verb}({arg}))"
            )),
            _ => {}
        }
    }
    let groups = |m: &std::collections::BTreeMap<&str, Vec<String>>, indent: &str| -> String {
        m.iter()
            .map(|(g, methods)| format!("{indent}{g}: {{\n{}\n{indent}}},", methods.iter().map(|x| format!("{indent}  {x},")).collect::<Vec<_>>().join("\n")))
            .collect::<Vec<_>>()
            .join("\n")
    };
    let mut free_groups = groups(&free, "  ");
    // `bind` is the client's own (no command): it checks a context, it does not mutate.
    let bind = "    async bind(context) {\n      const info = unwrap(await service.documents.get({ documentId: context.documentId }));\n      if (info.context.expectedRevision !== context.expectedRevision) {\n        throw new ApiFailure({ code: \"revision_conflict\", message: \"the document is at revision \" + info.context.expectedRevision + \", not \" + context.expectedRevision, phase: \"admission\", operationId: null, executionId: null, context: info.context, effects: { disposition: \"none\", documents: [], external: [] } });\n      }\n      return new BoundDocument(info.context);\n    },";
    if free_groups.contains("  documents: {") {
        free_groups = free_groups.replacen("  documents: {\n", &format!("  documents: {{\n{bind}\n"), 1);
    } else {
        free_groups.push_str(&format!("\n  documents: {{\n{bind}\n  }},"));
    }
    format!(
        r#"{CORE}
// ---- GENERATED from the cad.* catalogue (manifest {hash}) ----

class BoundDocument {{
  #context;
  #busy = false;
  constructor(context) {{
    this.#context = {{ documentId: context.documentId, expectedRevision: context.expectedRevision }};
    const self = this;
{bound}
  }}
  get context() {{
    return {{ documentId: this.#context.documentId, expectedRevision: this.#context.expectedRevision }};
  }}
  async #write(command, args, pick) {{
    // Same-document mutations are sequential: `Promise.all` over two writes
    // to one document is refused, never reordered.
    if (this.#busy) {{
      throw new ApiFailure({{ code: "document_busy", message: "another write to this document is still outstanding; await it first", phase: "admission", operationId: null, executionId: null, effects: {{ disposition: "none", documents: [], external: [] }} }});
    }}
    this.#busy = true;
    try {{
      const output = await operate(command, args, this.context);
      // Only this handle's own committed change advances its expectation.
      if (output && output.context) this.#context.expectedRevision = output.context.expectedRevision;
      return pick(output);
    }} finally {{
      this.#busy = false;
    }}
  }}
  async #query(command, args, pick) {{
    const r = unwrap(await service.commands.query({{ manifest, command, args, context: this.context }}));
    return pick(r.output);
  }}
}}

{tools}
export const cad = guardTree({{
  discovery: {{
    search: async (args) => unwrap(await service.discovery.search(args)),
    describe: async (args) => unwrap(await service.discovery.describe(args)),
  }},
  session: {{
    /** One call for "what is open": documents, the active one, its features, solids and errors. */
    async summary() {{
      const docs = await brep.document.docs_list({{}});
      const h = await brep.history.history_listing({{}});
      const active = docs.tabs[docs.active] || null;
      const errors = (h.report && h.report.featureErrors) || [];
      return {{
        documents: docs.tabs.map((t) => ({{ id: t.id, title: t.title, dirty: t.dirty, active: t.index === docs.active }})),
        active: active && {{ id: active.id, title: active.title, revision: active.revision }},
        units: "mm",
        featureCount: h.listing.features.length,
        features: h.listing.features.map((f) => f.id + " (" + f.type + ")"),
        rollbackStep: h.listing.step,
        solidCount: h.model.solidCount,
        solids: h.model.solids.map((x) => x.name),
        featureErrors: errors,
        idle: h.idle,
      }};
    }},
  }},
  docs: {{
    list: async () => unwrap(await call("cad.docs.list", {{}})).guides,
    read: async (name) => unwrap(await call("cad.docs.read", typeof name === "string" ? {{ name }} : name)).text,
    sdk: async (group) => {{ const r = unwrap(await call("cad.docs.sdk", group === undefined ? {{}} : typeof group === "string" ? {{ group }} : group)); return r.declarations !== undefined ? r.declarations : r; }},
  }},
{free_groups}
}}, "cad");

export default cad;
"#,
        tools = tool_bindings(tools),
        hash = catalogue.sha256,
        bound = bound
            .iter()
            .map(|(g, methods)| format!("    this.{g} = {{\n{}\n    }};", methods.iter().map(|x| format!("      {x},")).collect::<Vec<_>>().join("\n")))
            .collect::<Vec<_>>()
            .join("\n"),
    )
}

