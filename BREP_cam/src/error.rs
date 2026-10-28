//! The crate-wide error type. Generation-stopping failures are errors; a
//! recoverable degradation is a warning on the operation report instead.

use crate::cancel::Cancelled;
use crate::cutter::CutterError;
use crate::model::TransformError;
use crate::numeric::NumericError;

#[derive(Clone, Debug, PartialEq)]
pub enum CamError {
    /// The caller cancelled; the previous accepted result stands.
    Cancelled,
    /// A cutter's dimensions were refused.
    Cutter(CutterError),
    /// The tolerance object was refused.
    Numeric(NumericError),
    /// A setup transform was refused.
    Transform(TransformError),
    /// A coordinate that must be finite was not. `what` names it.
    NonFinite { what: String },
    /// An enabled operation has no target triangles.
    EmptyTarget { operation_id: String },
    /// A referenced body or face does not exist in the snapshot.
    UnresolvedTarget { solid: String, face: Option<String> },
    /// A strategy does not support the selected cutter kind.
    UnsupportedCutter { strategy: String, cutter: String },
    /// Any other invalid input, with a user-facing message.
    InvalidInput(String),
}

impl std::fmt::Display for CamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CamError::Cancelled => write!(f, "cancelled"),
            CamError::Cutter(e) => write!(f, "cutter: {e}"),
            CamError::Numeric(e) => write!(f, "tolerance: {e}"),
            CamError::Transform(e) => write!(f, "setup transform: {e}"),
            CamError::NonFinite { what } => write!(f, "{what} is not finite"),
            CamError::EmptyTarget { operation_id } => {
                write!(f, "operation {operation_id} has no target triangles")
            }
            CamError::UnresolvedTarget { solid, face: None } => {
                write!(f, "target body {solid:?} is not in the mesh snapshot")
            }
            CamError::UnresolvedTarget { solid, face: Some(face) } => {
                write!(f, "target face {face:?} of body {solid:?} is not in the mesh snapshot")
            }
            CamError::UnsupportedCutter { strategy, cutter } => {
                write!(f, "strategy {strategy} does not support a {cutter} cutter")
            }
            CamError::InvalidInput(msg) => write!(f, "{msg}"),
        }
    }
}

impl std::error::Error for CamError {}

impl From<Cancelled> for CamError {
    fn from(_: Cancelled) -> Self {
        CamError::Cancelled
    }
}

impl From<CutterError> for CamError {
    fn from(e: CutterError) -> Self {
        CamError::Cutter(e)
    }
}

impl From<NumericError> for CamError {
    fn from(e: NumericError) -> Self {
        CamError::Numeric(e)
    }
}

impl From<TransformError> for CamError {
    fn from(e: TransformError) -> Self {
        CamError::Transform(e)
    }
}
