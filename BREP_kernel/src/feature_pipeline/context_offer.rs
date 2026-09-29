//! Selection-context applicability — kernel-owned show/no-show for the app's
//! selection context bar.
//!
//! Each feature module exposes `context_applicable(&SelectionProbe) -> bool`
//! next to its `schema()`: the ONE per-feature answer to "does the current
//! selection make creating me meaningful?". This module only AGGREGATES those
//! functions (the `schema.rs` pattern) into [`FEATURE_APPLICABILITY`], keyed by
//! the catalogue `type` string, and the assembly-constraint equivalents live as
//! `applicable` on each [`super::assembly::constraints::ConstraintTypeDef`].
//!
//! The probe is a KIND-LEVEL summary — per-kind counts plus a few app-derived
//! roll-up booleans (`all_component`, `all_sheet_metal`) that answer ownership
//! questions the raw names would; never the names themselves. Predicates decide
//! eligibility; the app still derives WHICH reference fields to pre-fill from
//! the schema `selectionFilter`s. A feature whose references a selection can
//! never satisfy (primitives, datum-driven splines) simply returns `false`.

/// Kind-level summary of the user's current selection, handed to each
/// feature's / constraint's `context_applicable` test.
///
/// `solids` counts REAL solids only — committed sketches present in the scene
/// as solids, but their reference kind is `SKETCH`, so the app partitions them
/// into `sketches` before probing (the context bar's established split).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SelectionProbe {
    pub solids: usize,
    pub sketches: usize,
    pub faces: usize,
    pub edges: usize,
    /// Selected construction PLANES / DATUM planes (their scene FRAME names, e.g.
    /// `Datum:XY` or a `P` plane feature's id). A datum/plane can seat a sketch's
    /// `sketchPlane` (the kernel resolves a frame name there), so Sketch offers on
    /// a plane-only selection just like it does on a planar face.
    pub planes: usize,
    /// Selected vertices (position-keyed, no names — no feature offer keys on
    /// them, but they disqualify component-only shapes via `all_component`).
    pub vertices: usize,
    /// Distinct owning ACOMP components across the selected named entities.
    pub components: usize,
    /// Every selected named entity is component-owned (and ≥1 is selected).
    /// Feature creation is fenced on component geometry, so feature predicates
    /// never see it; constraint predicates REQUIRE it (`mapping::resolve_element`
    /// rejects non-component references).
    pub all_component: bool,
    /// Every selected named entity sits on a SHEET-METAL body (and ≥1 is
    /// selected) — the app resolves each pick's owning solid and checks its
    /// `SheetTree` marker. The sheet-metal EDIT features (SM Flange / Fillet /
    /// Chamfer) require it: they mutate an existing sheet-metal body, so they
    /// are meaningless on plain geometry.
    pub all_sheet_metal: bool,
}

impl SelectionProbe {
    /// A profile source is selected (the extrude/revolve/sweep family's
    /// `["SKETCH","FACE"]` fields).
    pub fn has_profile(&self) -> bool {
        self.faces > 0 || self.sketches > 0
    }
}

/// Every catalogue feature's applicability predicate, catalogue order, keyed by
/// the schema `type` string ([`super::schema::feature_schema_catalogue`] —
/// `sheet_metal_corner` contributes two entries from one module). The paired
/// test keeps this 1:1 with the catalogue.
pub static FEATURE_APPLICABILITY: &[(&str, fn(&SelectionProbe) -> bool)] = &[
    ("D", super::features::datum::context_applicable),
    ("P", super::features::plane::context_applicable),
    ("P.CU", super::features::cube::context_applicable),
    ("P.CY", super::features::cylinder::context_applicable),
    ("P.CO", super::features::cone::context_applicable),
    ("P.S", super::features::sphere::context_applicable),
    ("P.T", super::features::torus::context_applicable),
    ("P.PY", super::features::pyramid::context_applicable),
    ("IMPORT3D", super::features::import3d::context_applicable),
    ("S", super::features::sketch::context_applicable),
    ("SP", super::features::spline::context_applicable),
    ("WP", super::features::waypoint::context_applicable),
    ("HX", super::features::helix::context_applicable),
    ("E", super::features::extrude::context_applicable),
    ("B", super::features::boolean::context_applicable),
    ("F", super::features::fillet::context_applicable),
    ("CH", super::features::chamfer::context_applicable),
    ("O.S", super::features::offset_shell::context_applicable),
    ("O.F", super::features::offset_face::context_applicable),
    ("PF", super::features::push_face::context_applicable),
    ("TF", super::features::transform_face::context_applicable),
    ("DF", super::features::delete_face::context_applicable),
    ("RFS", super::features::refit_faces::context_applicable),
    ("THK", super::features::thicken::context_applicable),
    ("SM.TAB", super::features::sheet_metal_tab::context_applicable),
    ("SM.CF", super::features::sheet_metal_contour_flange::context_applicable),
    ("SM.F", super::features::sheet_metal_flange::context_applicable),
    ("SM.HEM", super::features::sheet_metal_hem::context_applicable),
    ("SM.FILLET", super::features::sheet_metal_corner::context_applicable),
    ("SM.CHAMFER", super::features::sheet_metal_corner::context_applicable),
    ("SM.CUTOUT", super::features::sheet_metal_cutout::context_applicable),
    ("LOFT", super::features::loft::context_applicable),
    ("M", super::features::mirror::context_applicable),
    ("SPL", super::features::split::context_applicable),
    ("R", super::features::revolve::context_applicable),
    ("RIB", super::features::rib::context_applicable),
    ("SW", super::features::sweep::context_applicable),
    ("SWP", super::features::path_sweep::context_applicable),
    ("H", super::features::hole::context_applicable),
    ("TU", super::features::tube::context_applicable),
    ("XFORM", super::features::transform::context_applicable),
    ("PATTERN", super::features::pattern::context_applicable),
    ("ACOMP", super::features::assembly_component::context_applicable),
];

/// Run feature `type_id`'s applicability predicate. Unknown type → `false`
/// (nothing is offered for a feature the registry does not know).
pub fn feature_context_applicable(type_id: &str, probe: &SelectionProbe) -> bool {
    FEATURE_APPLICABILITY
        .iter()
        .find(|(id, _)| *id == type_id)
        .is_some_and(|(_, applicable)| applicable(probe))
}

