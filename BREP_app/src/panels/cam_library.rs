//! The USER tool library of the CAM workbench: the tools a user keeps across
//! documents, beside the document's own library (the `cam` block's `tools`).
//!
//! It is lane C12's `LibraryDocument` (the interchange format the import and
//! export use), persisted whole under one LOCAL key of the model store
//! ([`CAM_TOOL_LIBRARY_KEY`], a `@` key the explorer never lists and that
//! stays on this machine — see `store::LOCAL_KEYS` for why it is not a
//! PLM-served preference yet), so "export the user library" writes the very
//! bytes the store holds. Reading tolerates no file (an empty library) and
//! reports a file it cannot read instead of replacing it, like the block.

use brep_cam::library::interchange::{import_csv, import_fusion, ImportReport, LibraryDocument};
use brep_cam::library::{LibraryUnits, ToolLibrary};
use brep_cam::model::Tool;
use crate::store::ModelStore;

/// The store key the user library lives under.
pub const CAM_TOOL_LIBRARY_KEY: &str = "@cam_tools";

/// Read the user library (`Ok(empty)` with no file; a file that cannot be
/// read is an error naming why, and the file is left as it is).
pub fn load(store: &dyn ModelStore) -> Result<LibraryDocument, String> {
    match store.read(CAM_TOOL_LIBRARY_KEY) {
        None => Ok(LibraryDocument::new(ToolLibrary::new("User library"))),
        Some(text) if text.trim().is_empty() => Ok(LibraryDocument::new(ToolLibrary::new("User library"))),
        Some(text) => LibraryDocument::from_json(&text).map_err(|e| format!("the user tool library cannot be read: {e}")),
    }
}

/// Write the user library back (validated by its own `to_json`).
pub fn save(store: &dyn ModelStore, library: &LibraryDocument) -> Result<(), String> {
    let text = library.to_json()?;
    store.write(CAM_TOOL_LIBRARY_KEY, &text).map_err(|e| format!("the user tool library could not be saved: {e}"))
}

/// The library as text, for export (the stored bytes, or a fresh document).
pub fn export_text(store: &dyn ModelStore) -> Result<String, String> {
    load(store)?.to_json()
}

/// Put `tool` into the user library under its own id and number, replacing
/// a tool with the same id (a tool saved twice updates its entry) and
/// refusing a different tool that already holds the number.
pub fn save_tool(store: &dyn ModelStore, tool: &Tool) -> Result<(), String> {
    let mut library = load(store)?;
    library.library.tools.retain(|t| t.id != tool.id);
    library.library.insert(tool.clone()).map_err(|e| format!("user library: {e}"))?;
    save(store, &library)
}

/// Remove tool `id` from the user library.
pub fn remove_tool(store: &dyn ModelStore, id: &str) -> Result<(), String> {
    let mut library = load(store)?;
    let before = library.library.tools.len();
    library.library.tools.retain(|t| t.id.as_str() != id);
    if library.library.tools.len() == before {
        return Err(format!("no tool `{id}` in the user library"));
    }
    save(store, &library)
}

/// The user library's tool `id`, cloned.
pub fn tool(store: &dyn ModelStore, id: &str) -> Result<Tool, String> {
    load(store)?.library.tools.iter().find(|t| t.id.as_str() == id).cloned().ok_or_else(|| format!("no tool `{id}` in the user library"))
}

/// Import a library file's text by its name's extension: a Fusion 360
/// `.json` / `.tools` export, a `.csv` sheet, or this app's own `.json`
/// library document. The report carries the tools read and every row the
/// importer had to skip, with why.
pub fn import_text(name: &str, text: &str) -> Result<ImportReport, String> {
    let lower = name.to_ascii_lowercase();
    if lower.ends_with(".csv") {
        return import_csv(text, LibraryUnits::Millimeters);
    }
    if let Ok(document) = LibraryDocument::from_json(text) {
        return Ok(ImportReport { document, issues: Vec::new() });
    }
    import_fusion(text)
}

