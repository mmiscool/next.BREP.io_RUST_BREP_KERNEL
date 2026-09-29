//! The three document classes: a normal part (`.nbrep`), a family seed
//! (`.fbrep`) and a template (`.tbrep`).
//!
//! On the file system the EXTENSION carries the class, matched
//! case-insensitively and written lowercase. The document carries it too, as
//! the top-level `documentClass` field (absent on a normal part, so every
//! existing file stays byte-identical). The two agree by construction: a load
//! writes the field from the file's extension, and Save As writes the field
//! the dialog's class choice names before the file gets that extension. The
//! field is what answers for a document that has no file yet (a new family),
//! and what the PLM, which keys documents without extensions, will read.

use brep_render::engine_state::EngineState;

/// The document key the class is stored under.
pub const DOCUMENT_CLASS_KEY: &str = "documentClass";

/// What a document is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum DocumentClass {
    /// A normal part: everything before this slice, imported parts, family
    /// members and parts spun out of templates.
    #[default]
    Normal,
    /// A family seed: the model plus the table of its members.
    Family,
    /// A template: a seed whose marked inputs specialise a new part on insert.
    Template,
}

impl DocumentClass {
    pub const ALL: [DocumentClass; 3] = [Self::Normal, Self::Family, Self::Template];

    /// The file extension, dot included, lowercase.
    pub fn extension(self) -> &'static str {
        match self {
            Self::Normal => ".nbrep",
            Self::Family => ".fbrep",
            Self::Template => ".tbrep",
        }
    }

    /// The `documentClass` field's value; `None` for a normal part, which
    /// writes no field.
    pub fn field(self) -> Option<&'static str> {
        match self {
            Self::Normal => None,
            Self::Family => Some("family"),
            Self::Template => Some("template"),
        }
    }

    /// The word the UI shows.
    pub fn label(self) -> &'static str {
        match self {
            Self::Normal => "Part",
            Self::Family => "Family",
            Self::Template => "Template",
        }
    }

    /// A stable slug for automation keys and blobs.
    pub fn slug(self) -> &'static str {
        match self {
            Self::Normal => "normal",
            Self::Family => "family",
            Self::Template => "template",
        }
    }

    /// The class a file name's extension names, if it names one.
    pub fn of_name(name: &str) -> Option<Self> {
        let lower = name.to_ascii_lowercase();
        Self::ALL
            .into_iter()
            .find(|class| lower.ends_with(class.extension()))
    }

    /// The class a `documentClass` field value names. Anything else (absent,
    /// unknown) is a normal part.
    pub fn of_field(value: Option<&serde_json::Value>) -> Self {
        match value.and_then(serde_json::Value::as_str) {
            Some("family") => Self::Family,
            Some("template") => Self::Template,
            _ => Self::Normal,
        }
    }

    /// The class a document's JSON says it is.
    pub fn of_document(document: &serde_json::Value) -> Self {
        Self::of_field(document.get(DOCUMENT_CLASS_KEY))
    }

    /// `name` with any class extension replaced by this class's: `bolt` and
    /// `bolt.nbrep` both become `bolt.fbrep` for a family. A directory part is
    /// kept as it is.
    pub fn file_name(self, name: &str) -> String {
        format!("{}{}", strip_class_extension(name), self.extension())
    }
}

/// `name` without a trailing class extension (any case).
pub fn strip_class_extension(name: &str) -> &str {
    match DocumentClass::of_name(name) {
        Some(class) => &name[..name.len() - class.extension().len()],
        None => name,
    }
}

/// The class of the document the engine holds, from its `documentClass`
/// field. Cheap enough for every frame: one map lookup.
pub fn document_class(engine: &EngineState) -> DocumentClass {
    DocumentClass::of_field(engine.history.document_block(DOCUMENT_CLASS_KEY))
}

/// Whether the engine holds a family seed.
pub fn is_family(engine: &EngineState) -> bool {
    document_class(engine) == DocumentClass::Family
}

/// Whether the engine holds a template.
pub fn is_template(engine: &EngineState) -> bool {
    document_class(engine) == DocumentClass::Template
}

/// Write the class into the engine's document without an undo step: this is
/// bookkeeping (what the file is), not an edit the user could undo.
pub fn set_document_class(engine: &mut EngineState, class: DocumentClass) {
    if document_class(engine) == class
        && (class != DocumentClass::Normal
            || engine.history.document_block(DOCUMENT_CLASS_KEY).is_none())
    {
        return;
    }
    engine.history.set_document_block_no_undo(
        DOCUMENT_CLASS_KEY,
        class.field().map(|field| serde_json::Value::String(field.into())),
    );
}

/// Set the class field on a document's JSON (`None` removes it).
pub fn set_class_in_json(document: &mut serde_json::Value, class: DocumentClass) {
    if let Some(object) = document.as_object_mut() {
        match class.field() {
            Some(field) => {
                object.insert(DOCUMENT_CLASS_KEY.into(), serde_json::Value::String(field.into()));
            }
            None => {
                object.remove(DOCUMENT_CLASS_KEY);
            }
        }
    }
}

