//! Weld symbols for the `weldCallout` PMI type. The kernel builds each
//! callout's symbol on its report row and lays it out
//! (`brep_kernel::feature_pipeline::pmi::weld_symbol`), so the 3D view, the 3D
//! PDF, the drawing sheets and the STEP presentation draw the same symbol. This
//! module is the render crate's name for it, plus the callout's new-annotation
//! defaults.
pub use brep_kernel::feature_pipeline::pmi::weld_symbol::{default_elbow, present, Frame, Glyph, Side, Standard, WeldSymbol};
use serde_json::Value;

/// The PMI type this module draws.
pub const WELD_CALLOUT: &str = "weldCallout";

/// The PMI schema catalogue: the kernel's, whose `weldCallout` entry carries
/// the symbol fields.
pub fn pmi_schema_catalogue() -> Value {
    brep_kernel::pmi_schema_catalogue()
}

/// Defaults of the symbol fields for a new `weldCallout`, for keys absent from `params`.
pub fn seed_defaults(type_id: &str, params: &mut serde_json::Map<String, Value>) {
    if type_id != WELD_CALLOUT {
        return;
    }
    for (key, spec) in brep_kernel::feature_pipeline::pmi::weld_symbol::symbol_fields() {
        if let Some(default) = spec.get("default_value") {
            params.entry(key.to_string()).or_insert_with(|| default.clone());
        }
    }
}

