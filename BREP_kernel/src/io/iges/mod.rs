//! IGES (Initial Graphics Exchange Specification, IGES 5.3) B-rep exchange.
//!
//! * [`export_iges`] and [`export_iges_bodies`] write each solid as a **186**
//!   Manifold Solid B-Rep: shells, faces and loops over shared edge and vertex
//!   lists, analytic carriers as **190**–**198** and the rest as **128**, with
//!   a parameter curve on each edge use a reader would otherwise have to
//!   project, and a body's name (**406**) and colour (**314**).
//! * [`import_iges`] reads foreign and native files: **186** Manifold Solid
//!   B-Reps, and **144**/**143** trimmed-surface sets sewn into solids, by
//!   translating them for the STEP importer (`to_step`).
//!
//! The write and read framework (fixed 80-column S/G/D/P/T sections, the DE↔PD
//! pointer indirection, Hollerith strings, and free-format wrapping) lives in
//! [`writer`] and [`reader`]; the entity ⇄ NURBS conversions in [`entities`].

mod entities;
mod export;
mod import;
mod reader;
mod structure;
mod to_step;
mod writer;


pub use export::{export_iges, export_iges_assembly, export_iges_bodies, IgesBody};
pub use import::{import_iges, import_iges_report, read_iges_assembly, IgesImportReport};
