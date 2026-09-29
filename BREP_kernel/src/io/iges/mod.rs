//! IGES (Initial Graphics Exchange Specification, IGES 5.3) B-rep exchange.
//!
//! Round-trips NURBS B-rep geometry through IGES trimmed parametric surfaces:
//!
//! * [`export_iges`] writes each kernel face as a **144** Trimmed Surface over a
//!   **128** rational B-spline surface, with **142** Curve-on-Surface loops
//!   built from **126** rational B-spline curves grouped by **102** composite
//!   curves.
//! * [`import_iges`] reads those entities back, reconstructs one face per 144,
//!   and sews the face set into a coherently oriented solid.
//!
//! Supported: entities 128, 144, 142, 126, 102 (the minimum viable trimmed-
//! NURBS B-rep). Deferred: Manifold Solid B-Rep Object (186) with its
//! 502/504/508/510/514 subordinates, and analytic wireframe entities
//! (100/104/110); these produce a clear error on import.
//!
//! The write and read framework (fixed 80-column S/G/D/P/T sections, the DE↔PD
//! pointer indirection, Hollerith strings, and free-format wrapping) lives in
//! [`writer`] and [`reader`]; the entity ⇄ NURBS conversions in [`entities`].

mod entities;
mod export;
mod import;
mod reader;
mod writer;


pub use export::export_iges;
pub use import::import_iges;
