use std::fmt::{Display, Formatter};

#[derive(Clone, Debug, PartialEq)]
/// An error produced while validating geometry or recognizing a surface.
pub enum RecognitionError {
    /// The input mesh is malformed or contains invalid values.
    InvalidMesh(String),
    /// Recognition or mesh-analysis options are outside their valid range.
    InvalidOptions(String),
    /// A requested triangle or vertex selection is empty or invalid.
    InvalidSelection(String),
    /// The supplied geometry does not contain enough usable information.
    DegenerateData(String),
    /// A surface could not be fitted to otherwise valid geometry.
    FitFailed {
        /// The primitive type being fitted, when known.
        surface: Option<&'static str>,
        /// A human-readable explanation of the failure.
        reason: String,
    },
}

impl Display for RecognitionError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidMesh(message) => write!(f, "invalid mesh: {message}"),
            Self::InvalidOptions(message) => write!(f, "invalid recognition options: {message}"),
            Self::InvalidSelection(message) => write!(f, "invalid selection: {message}"),
            Self::DegenerateData(message) => write!(f, "degenerate geometry: {message}"),
            Self::FitFailed { surface, reason } => match surface {
                Some(kind) => write!(f, "{kind} fit failed: {reason}"),
                None => write!(f, "surface fit failed: {reason}"),
            },
        }
    }
}

impl std::error::Error for RecognitionError {}
