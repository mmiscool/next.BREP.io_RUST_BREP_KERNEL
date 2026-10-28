//! Manufacturing instructions in the app: making an instructions document
//! from an assembly, the open one or a stored one.
//!
//! The new document places the assembly as one grounded component (its
//! store key and content signature recorded, so Update components sees a new
//! revision) and records the same key as the instructions' target: in a PLM
//! session that is `part/<id>/rev/<rev>`, from which the server derives what
//! the instructions describe.
use crate::document::{Document, Documents, EMPTY_DOCUMENT};
use crate::document_class::{self, DocumentClass};
use crate::store::ModelStore;
use brep_instructions::model::Target;
use brep_render::engine_state::{document_signature, ComponentInsert};
use serde_json::Value;

/// Where the assembly comes from.
pub enum Source<'a> {
    /// The active tab's document, which must have been saved.
    Active,
    /// A document in the store, by its key.
    Stored(&'a str),
}

/// Open a new tab holding instructions for the assembly, with the wizard's
/// proposal when `wizard` (made once the assembly's run lands). Returns the new tab's index; nothing opens on a
/// refusal.
pub fn new_instructions(docs: &mut Documents, store: &dyn ModelStore, source: Source<'_>, wizard: bool) -> Result<usize, String> {
    let (key, json, name) = match source {
        Source::Active => {
            let key = docs
                .active()
                .name()
                .map(str::to_string)
                .ok_or("save the assembly first: instructions describe a saved revision")?;
            let name = docs.active().title();
            (key, docs.tab_engine().history.request_json(), name)
        }
        Source::Stored(key) => {
            let json = store.read(key).ok_or_else(|| format!("no document '{key}'"))?;
            let name = document_class::strip_class_extension(&crate::store::file_name_of(key)).to_string();
            (key.to_string(), json, name)
        }
    };
    let document: Value = serde_json::from_str(&json).map_err(|e| format!("'{key}' is not a document: {e}"))?;
    if DocumentClass::of_document(&document) == DocumentClass::Instructions {
        return Err("that document is already instructions; open an assembly".into());
    }
    let signature = document_signature(&json);
    let target = Target {
        source_key: key.clone(),
        part_number: String::new(),
        revision: String::new(),
        signature: signature.clone(),
        name: name.clone(),
    };
    let mut engine = docs.spawn_engine();
    engine.set_history_json(EMPTY_DOCUMENT)?;
    // Instructions are edited in their own workbench, and saved with it.
    engine.settings.workbench = crate::workbench::instructions::INSTRUCTIONS.id.into();
    engine.instructions_start(
        ComponentInsert::New { name: &name, source_key: &key, source_signature: &signature, document_json: &json },
        target,
    )?;
    if wizard {
        engine.instructions_run_wizard_when_built()?;
    }
    Ok(docs.open_document(Document::new(engine)))
}
