//! Feature execution and schemas, grouped by modeling operation.
//! Shared parameter readers and solid finalization live in [`common`].

pub mod common;

// Basic primitives.
pub mod cube;
pub mod sphere;

// Assemblies: one placed parts-library instance (ACOMP).
pub mod assembly_component;

// Solid-producing primitives.
pub mod cone;
pub mod cylinder;
pub mod pyramid;
pub mod torus;
pub mod transform_bake;

// Construction geometry and editable curves.
pub mod datum;
pub mod helix;
pub mod spline;
pub mod waypoint;
pub mod face_profile;
pub mod plane;
pub mod sketch;

// Add-material.
pub mod extrude;
pub mod loft;
pub mod path_sweep;
pub mod revolve;
pub mod rib;
pub mod sweep;
pub mod tube;

// Edit / boolean / transform.
pub mod boolean;
pub mod mirror;
pub mod pattern;
pub mod split;
pub mod transform;

// Dressups.
pub mod chamfer;
pub mod delete_face;
pub mod refit_faces;
pub mod fillet;
pub mod hole;
pub mod offset_face;
pub mod offset_shell;
pub mod push_face;
pub mod thicken;
pub mod transform_face;

// Imported geometry and sheet metal.
pub mod import3d;
pub mod sheet_metal_contour_flange;
pub mod sheet_metal_corner;
pub mod sheet_metal_cutout;
pub mod sheet_metal_flange;
pub mod sheet_metal_hem;
pub mod sheet_metal_tab;
pub mod sheet_metal_unfold;
