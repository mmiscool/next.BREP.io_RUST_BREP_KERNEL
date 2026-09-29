//! JSON and WebAssembly entry points, re-exported through the crate root.
use super::*;

mod curves;
mod display;
mod geometry;
mod handles;
mod io_abi;
mod modeling_a;
mod modeling_b;
mod objects;
mod primitives;
mod requests;

pub use curves::*;
pub use display::*;
pub use geometry::*;
pub use handles::*;
pub use io_abi::*;
pub use modeling_a::*;
pub use modeling_b::*;
pub use objects::*;
pub use primitives::*;
use requests::*;

fn decode_solid_buffer(data: &[f64], names_json: &str) -> Result<BrepSolid, JsValue> {
    let names: SolidNames = if names_json.trim().is_empty() {
        SolidNames::default()
    } else {
        serde_json::from_str(names_json).map_err(|error| javascript_error(error.to_string()))?
    };
    decode_solid(data, &names).map_err(javascript_error)
}

/// Reject invalid topology at external ingestion boundaries before registration.
fn validate_imported_solid(solid: BrepSolid, endpoint: &str) -> Result<BrepSolid, JsValue> {
    let solid = TopologyArena::from_brep(&solid)
        .and_then(|arena| arena.to_brep())
        .map_err(javascript_error)?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(javascript_error(format!("{endpoint}: invalid topology: {issues:?}")));
    }
    Ok(solid)
}
