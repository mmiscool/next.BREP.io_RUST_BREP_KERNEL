//! STRUCTURED STEP export: write a product structure, not just bodies.
//!
//! The flat writer ([`super::export_step_report_named`]) puts every body of the
//! document into ONE `PRODUCT`. That is the right answer for a part, and the
//! wrong one for an assembly: a nested assembly came out as a bag of bodies at
//! their world positions, with the parts library, the instance count and the
//! hierarchy all discarded.
//!
//! This module is the other answer. It takes a [`StepAssemblyExport`] — one
//! [`StepExportProduct`] per distinct part holding that part's bodies IN ITS OWN
//! LOCAL FRAME, plus one [`StepExportOccurrence`] per placement — and writes the
//! AP242 graph the importer (`io/step_import/`) reads back:
//!
//! ```text
//!   PRODUCT -> PRODUCT_DEFINITION_FORMATION -> PRODUCT_DEFINITION
//!       -> PRODUCT_DEFINITION_SHAPE -> SHAPE_DEFINITION_REPRESENTATION
//!       -> ADVANCED_BREP_SHAPE_REPRESENTATION (the part's own bodies)
//!
//!   NEXT_ASSEMBLY_USAGE_OCCURRENCE(parent_pd, child_pd)
//!       <- PRODUCT_DEFINITION_SHAPE('NAUO PRDDFN')
//!       <- CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
//!       -> ( REPRESENTATION_RELATIONSHIP(child_rep, parent_rep)
//!            REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(
//!              ITEM_DEFINED_TRANSFORMATION(identity_axis, placement_axis) )
//!            SHAPE_REPRESENTATION_RELATIONSHIP() )
//! ```
//!
//! # The placement convention, and why it is copied rather than chosen
//!
//! A reader composes `T = M(item_2) · M(item_1)⁻¹` to map `rep_1` into `rep_2`.
//! With `rep_1` the CHILD and `item_1` the identity axis, `T` is simply
//! `M(item_2)` — the child's frame expressed in the parent. That ordering is
//! taken from the `as1-ug-214` vendor fixture
//! (`REPRESENTATION_RELATIONSHIP(' ',' ',#213 /* rod */,#219 /* rod_assem */)`
//! with `ITEM_DEFINED_TRANSFORMATION(' ',' ',#1041 /* identity */,#1109)`), NOT
//! from our own importer: `resolve_rep_rel_transform` reorients using the
//! child's known root representation, so a self round-trip would have passed
//! with the ordering reversed and told us nothing.
//!
//! For the same reason `item_1`/`item_2` are ITEMS of the reps they belong to —
//! the identity axis is the first item of every representation this writer
//! emits, and each occurrence's placement axis is pushed into its PARENT's item
//! list. A conformance checker requires exactly that.
//!
//! # Naming, and what PMI resolves against
//!
//! A part's bodies, faces and edges keep the part's OWN names — `Extrude1_top`,
//! not `ACOMP3:Extrude1_top` — because they are written into the part's product,
//! where the component namespace does not apply. The document still refers to
//! them by the namespaced name, so [`StepAssemblyExport::occurrence_paths`]
//! walks the graph and registers every entity under EVERY chained occurrence
//! path that reaches it (`ACOMP3:`, `ACOMP5:ACOMP1:`), with the vertex points
//! moved into root space so a `{body}@x,y,z` reference still matches. Two
//! instances of one part share the part's entities, so `ACOMP3:top` and
//! `ACOMP4:top` resolve to the SAME `ADVANCED_FACE`.
//!
//! They are not the same ANNOTATION, though, and PMI is written accordingly:
//!
//! - a PART's own annotations ([`StepExportProduct::pmi`], from the part
//!   document's `pmi` block) are written into that part's product, with the
//!   part's own names and its own draughting model — once, however many
//!   instances place it;
//! - the ROOT document's annotations are written through the occurrence paths,
//!   and a reference that reaches a part through an occurrence hangs its
//!   `SHAPE_ASPECT` off THAT occurrence's `PRODUCT_DEFINITION_SHAPE` (the
//!   `'NAUO PRDDFN'` one), not the part's. The `GEOMETRIC_ITEM_SPECIFIC_USAGE`
//!   still points at the part's face; only the owner says which instance is
//!   meant, and it is the only place that can say it.
//!
//! `io/step_import/assembly_pmi.rs` reads exactly that distinction back, which
//! is what makes an assembly's PMI survive export → import unchanged.

use super::{
    finish_step_file, id_list, mat4_mul, pmi, step_string, styles, write_file_contexts,
    write_product, write_product_geometry, Mat4, StepColors, StepExportReport, StepItemOwner,
    StepNameMaps, StepPmi, StepWriter, MAT4_IDENTITY,
};
use crate::feature_pipeline::pmi::{PmiReport, PmiState};
use crate::{BrepSolid, Vec3};

/// One product of an exported structure: a part, a sub-assembly, or the root
/// document. `bodies` are in the product's OWN local frame — never posed.
#[derive(Clone, Debug, Default)]
pub struct StepExportProduct {
    /// `PRODUCT.name` — the parts-library entry name, or the document name.
    pub name: String,
    /// `PRODUCT.id` — the vendor part number: a library part's BOM
    /// `Part_Number` attribute, else its `sourceKey` (`export_tree.rs`). The
    /// name is used when empty.
    pub id: String,
    /// This product's own solids and their part-local scene names. Empty for a
    /// pure assembly node, which is then written as a bare `SHAPE_REPRESENTATION`.
    pub bodies: Vec<(String, BrepSolid)>,
    /// This product's OWN PMI — the part document's `pmi` block, resolved
    /// against the part's own geometry (`export_tree.rs` runs it). Written
    /// into THIS product, with the part's own names and its own draughting
    /// model, which is where a structured import reads it back from.
    pub pmi: Option<(PmiState, PmiReport)>,
}

/// One placement of `child` inside `parent` — a `NEXT_ASSEMBLY_USAGE_OCCURRENCE`.
#[derive(Clone, Debug)]
pub struct StepExportOccurrence {
    /// The instance label: the placing component's id (`ACOMP3`). Written into
    /// the NAUO's `reference_designator`, which is what the importer reads back
    /// first, so an instance keeps its identity through the round trip.
    pub designator: String,
    /// Index into [`StepAssemblyExport::products`].
    pub parent: usize,
    /// Index into [`StepAssemblyExport::products`].
    pub child: usize,
    /// child-local -> parent-local, row-major affine. Must be RIGID: a mirrored
    /// or scaled instance has no `AXIS2_PLACEMENT_3D` and is rejected rather
    /// than silently written as its rotation part.
    pub placement: Mat4,
}

/// A document's product structure, ready to write. Exactly one product must be
/// a root (never an occurrence child) — the document itself.
#[derive(Clone, Debug, Default)]
pub struct StepAssemblyExport {
    pub products: Vec<StepExportProduct>,
    pub occurrences: Vec<StepExportOccurrence>,
}

/// How many occurrence paths the PMI alias walk will register before it stops.
/// One path per instance is the honest count, and a real assembly has
/// thousands at most; the cap only bounds a pathological diamond graph, whose
/// path count is exponential in its depth.
const MAX_OCCURRENCE_PATHS: usize = 100_000;

impl StepAssemblyExport {
    /// The single root product's index, with the graph checked on the way: every
    /// occurrence in range, no self-placement, exactly one root, no cycle, every
    /// product reachable, every placement rigid.
    pub fn root(&self) -> Result<usize, String> {
        if self.products.is_empty() {
            return Err("export_step: an assembly needs at least one product".into());
        }
        let count = self.products.len();
        let mut is_child = vec![false; count];
        for occurrence in &self.occurrences {
            if occurrence.parent >= count || occurrence.child >= count {
                return Err(format!(
                    "export_step: occurrence '{}' names a product outside the structure",
                    occurrence.designator
                ));
            }
            if occurrence.parent == occurrence.child {
                return Err(format!(
                    "export_step: occurrence '{}' places a product inside itself",
                    occurrence.designator
                ));
            }
            rigid(&occurrence.placement).map_err(|error| {
                format!(
                    "export_step: occurrence '{}' placement: {error}",
                    occurrence.designator
                )
            })?;
            is_child[occurrence.child] = true;
        }
        let roots: Vec<usize> = (0..count).filter(|index| !is_child[*index]).collect();
        let [root] = roots.as_slice() else {
            return Err(format!(
                "export_step: an assembly needs exactly one root product, found {}",
                roots.len()
            ));
        };
        // Reachability doubles as the cycle check: a cycle makes every product
        // on it a child, so an unreached product is either orphaned or looping.
        let mut reached = vec![false; count];
        let mut stack = vec![*root];
        reached[*root] = true;
        while let Some(product) = stack.pop() {
            for occurrence in &self.occurrences {
                if occurrence.parent == product && !reached[occurrence.child] {
                    reached[occurrence.child] = true;
                    stack.push(occurrence.child);
                }
            }
        }
        if let Some(lost) = reached.iter().position(|hit| !hit) {
            return Err(format!(
                "export_step: product '{}' is not reachable from the root",
                self.products[lost].name
            ));
        }
        Ok(*root)
    }

    /// Every chained occurrence path that reaches each product: the component
    /// namespace prefix the DOCUMENT knows its entities by, and the transform
    /// into root space. The root itself gets one empty path at identity.
    ///
    /// Depth-first from the root, guarding the path's own ancestors, so a
    /// structure that somehow still carries a cycle terminates.
    fn occurrence_paths(&self, root: usize) -> Vec<Vec<OccurrencePath>> {
        let mut paths: Vec<Vec<OccurrencePath>> = vec![Vec::new(); self.products.len()];
        let mut total = 0usize;
        let mut stack: Vec<(usize, OccurrencePath, Vec<usize>)> = vec![(
            root,
            OccurrencePath {
                prefix: String::new(),
                world: MAT4_IDENTITY,
                last: None,
            },
            vec![root],
        )];
        while let Some((product, path, ancestors)) = stack.pop() {
            if total >= MAX_OCCURRENCE_PATHS {
                break;
            }
            total += 1;
            for (slot, occurrence) in self.occurrences.iter().enumerate() {
                if occurrence.parent != product || ancestors.contains(&occurrence.child) {
                    continue;
                }
                let mut ancestors = ancestors.clone();
                ancestors.push(occurrence.child);
                stack.push((
                    occurrence.child,
                    OccurrencePath {
                        prefix: format!("{}{}:", path.prefix, occurrence.designator),
                        world: mat4_mul(&path.world, &occurrence.placement),
                        last: Some(slot),
                    },
                    ancestors,
                ));
            }
            paths[product].push(path);
        }
        paths
    }
}

/// One way the document's geometry reaches a product: the component namespace
/// that names it (`ACOMP5:ACOMP1:`, `""` at the root), the transform into root
/// space, and the LAST occurrence on the way — the instance an annotation
/// written against this namespace is an annotation on.
#[derive(Clone)]
struct OccurrencePath {
    prefix: String,
    world: Mat4,
    /// Index into [`StepAssemblyExport::occurrences`]; `None` at the root.
    last: Option<usize>,
}

/// A placement is rigid when its linear block is orthonormal with a positive
/// determinant — the only kind an `AXIS2_PLACEMENT_3D` can express.
fn rigid(matrix: &Mat4) -> Result<(), String> {
    if matrix.iter().any(|value| !value.is_finite()) {
        return Err("matrix is not finite".into());
    }
    let column = |index: usize| {
        [
            matrix[index],
            matrix[4 + index],
            matrix[8 + index],
        ]
    };
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let (x, y, z) = (column(0), column(1), column(2));
    const TOLERANCE: f64 = 1e-9;
    for (a, b, expected) in [
        (x, x, 1.0),
        (y, y, 1.0),
        (z, z, 1.0),
        (x, y, 0.0),
        (x, z, 0.0),
        (y, z, 0.0),
    ] {
        if (dot(a, b) - expected).abs() > TOLERANCE {
            return Err("matrix is not a rotation (mirrored or scaled)".into());
        }
    }
    let cross = [
        x[1] * y[2] - x[2] * y[1],
        x[2] * y[0] - x[0] * y[2],
        x[0] * y[1] - x[1] * y[0],
    ];
    if (dot(cross, z) - 1.0).abs() > TOLERANCE {
        return Err("matrix is left-handed (a mirrored instance)".into());
    }
    Ok(())
}

/// The `AXIS2_PLACEMENT_3D` expressing a rigid placement's frame: the origin is
/// its translation, the axis its third linear COLUMN and the reference direction
/// its first — the exact inverse of the importer's `frame_matrix`.
fn write_placement_axis(writer: &mut StepWriter, matrix: &Mat4) -> Result<usize, String> {
    super::write_placement(
        writer,
        Vec3::new(matrix[3], matrix[7], matrix[11]),
        Vec3::new(matrix[2], matrix[6], matrix[10]),
        Vec3::new(matrix[0], matrix[4], matrix[8]),
    )
}

/// [`export_step_assembly_report`] returning just the Part 21 text.
pub fn export_step_assembly(
    assembly: &StepAssemblyExport,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    export_step_assembly_report(assembly, unit, timestamp, None, None).map(|report| report.text)
}

/// Write a product structure as an AP242 Part 21 document: one product per
/// distinct part, one `NEXT_ASSEMBLY_USAGE_OCCURRENCE` per placement, exact
/// B-rep geometry in each part's own frame.
///
/// The file is named after the ROOT product, and the optional PMI block rides
/// on the root exactly as it does in the flat lane — with each shape aspect
/// attached to the product that actually OWNS the geometry it names.
pub fn export_step_assembly_report(
    assembly: &StepAssemblyExport,
    unit: &str,
    timestamp: &str,
    pmi: Option<&StepPmi<'_>>,
    colors: Option<&StepColors>,
) -> Result<StepExportReport, String> {
    let root = assembly.root()?;
    if assembly
        .products
        .iter()
        .all(|product| product.bodies.is_empty())
    {
        return Err("export_step: at least one solid is required".into());
    }
    let mut report = StepExportReport {
        products: assembly.products.len(),
        occurrences: assembly.occurrences.len(),
        ..StepExportReport::default()
    };
    let mut writer = StepWriter::default();
    let contexts = write_file_contexts(&mut writer, unit)?;

    // 1 — every product's geometry, each in its own local frame. Written first
    // because a representation's item list needs the solids it carries.
    let mut geometries = Vec::with_capacity(assembly.products.len());
    for product in &assembly.products {
        let bodies: Vec<(String, &BrepSolid)> = product
            .bodies
            .iter()
            .map(|(name, solid)| (name.clone(), solid))
            .collect();
        geometries.push(write_product_geometry(
            &mut writer,
            &contexts,
            &bodies,
            &mut report,
        )?);
    }

    // 2 — each occurrence's placement frame. It is an ITEM of the parent
    // representation, so it has to exist before that representation is written.
    let mut placement_axes = Vec::with_capacity(assembly.occurrences.len());
    for occurrence in &assembly.occurrences {
        placement_axes.push(write_placement_axis(&mut writer, &occurrence.placement)?);
    }

    // 3 — the representations: identity axis, own solids, child placements.
    let geometry_context = contexts.geometry_context;
    let mut representations = Vec::with_capacity(assembly.products.len());
    for (index, product) in assembly.products.iter().enumerate() {
        let mut items = vec![contexts.axis];
        items.extend(geometries[index].solids.iter().map(|(_, id)| *id));
        for (slot, occurrence) in assembly.occurrences.iter().enumerate() {
            if occurrence.parent == index {
                items.push(placement_axes[slot]);
            }
        }
        // A pure assembly node carries no bodies, and an ADVANCED_BREP shape
        // representation with no solid in it is not one — say SHAPE_REPRESENTATION,
        // which is what a vendor writes and what our importer reads for the
        // structure-only levels of a tree.
        let keyword = if geometries[index].solids.is_empty() {
            "SHAPE_REPRESENTATION"
        } else {
            "ADVANCED_BREP_SHAPE_REPRESENTATION"
        };
        representations.push(writer.add(format!(
            "{keyword}('{}',{},#{geometry_context})",
            step_string(&product.name),
            id_list(&items)
        )));
    }

    // 4 — the product definition chains, bound to those representations.
    let mut definitions = Vec::with_capacity(assembly.products.len());
    for (index, product) in assembly.products.iter().enumerate() {
        // 'assembly' vs 'part' is the distinction a receiving system builds its
        // own structure tree from, so it follows the graph, not the geometry: a
        // product that places others is an assembly even when it also carries
        // bodies of its own.
        let places_others = assembly
            .occurrences
            .iter()
            .any(|occurrence| occurrence.parent == index);
        let category = if places_others { "assembly" } else { "part" };
        let ids = write_product(&mut writer, &contexts, &product.name, &product.id, category);
        let (product_shape, representation) = (ids.product_shape, representations[index]);
        writer.add(format!(
            "SHAPE_DEFINITION_REPRESENTATION(#{product_shape},#{representation})"
        ));
        definitions.push(ids);
    }

    // 5 — the occurrences themselves. Each one's `PRODUCT_DEFINITION_SHAPE` is
    // kept: it is what an OCCURRENCE-scoped annotation's shape aspect hangs
    // off (step 7), and the entity a reader tells "this instance" from "this
    // part" by.
    let mut nauo_shapes: Vec<usize> = Vec::with_capacity(assembly.occurrences.len());
    for (slot, occurrence) in assembly.occurrences.iter().enumerate() {
        let parent = definitions[occurrence.parent].definition;
        let child = definitions[occurrence.child].definition;
        let label = step_string(&occurrence.designator);
        let nauo = writer.add(format!(
            "NEXT_ASSEMBLY_USAGE_OCCURRENCE('{label}','{label}','',#{parent},#{child},'{label}')"
        ));
        let nauo_shape =
            writer.add(format!("PRODUCT_DEFINITION_SHAPE('','NAUO PRDDFN',#{nauo})"));
        nauo_shapes.push(nauo_shape);
        let identity = contexts.axis;
        let placement = placement_axes[slot];
        let transformation = writer.add(format!(
            "ITEM_DEFINED_TRANSFORMATION('','',#{identity},#{placement})"
        ));
        let (child_rep, parent_rep) = (
            representations[occurrence.child],
            representations[occurrence.parent],
        );
        let relationship = writer.add(format!(
            "(REPRESENTATION_RELATIONSHIP('','',#{child_rep},#{parent_rep})\
             REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{transformation})\
             SHAPE_REPRESENTATION_RELATIONSHIP())"
        ));
        writer.add(format!(
            "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{relationship},#{nauo_shape})"
        ));
    }

    // 6 — geometry colour. A product's entities are known to the DOCUMENT under
    // every occurrence namespace that reaches it, so the colour lookup resolves
    // through the same paths the PMI references do — sorted, so which of a
    // repeated part's namespaces supplies the colour does not depend on the
    // walk's stack order.
    // ONE walk for both consumers below — each product's occurrence namespaces —
    // and only when one of them needs it: the walk is capped at
    // `MAX_OCCURRENCE_PATHS`, so an export that carries neither colour nor PMI
    // must not pay for it.
    let colors = colors.filter(|colors| !colors.is_empty());
    let paths = (pmi.is_some() || colors.is_some())
        .then(|| assembly.occurrence_paths(root))
        .unwrap_or_default();
    if let Some(colors) = colors {
        let mut items = Vec::new();
        for (index, geometry) in geometries.iter().enumerate() {
            let mut prefixes: Vec<String> =
                paths[index].iter().map(|path| path.prefix.clone()).collect();
            prefixes.sort();
            items.extend(styles::styled_items(geometry, colors, &prefixes));
        }
        report.styled_items = styles::write_geometry_styles(&mut writer, geometry_context, &items)?;
    }

    // 7a — each PART's own PMI, written into that part's product with the
    // part's own names. A part shared by six instances carries its annotations
    // ONCE, which is the same rule the importer reads them back under.
    for (index, geometry) in geometries.iter().enumerate() {
        let Some((state, part_report)) = &assembly.products[index].pmi else {
            continue;
        };
        let mut names = StepNameMaps::default();
        names.register(
            geometry,
            StepItemOwner {
                product_shape: definitions[index].product_shape,
                representation: representations[index],
            },
            "",
            &MAT4_IDENTITY,
        );
        let context = pmi::StepContext {
            product_shape: definitions[index].product_shape,
            representation: representations[index],
            geometry_context,
            length_unit: contexts.length_unit,
            angle_unit: contexts.angle_unit,
            faces: &names.faces,
            edges: &names.edges,
            vertices: &names.vertices,
        };
        report.pmi_unresolved_references += pmi::write_pmi(
            &mut writer,
            &context,
            &StepPmi {
                state,
                report: part_report,
            },
        )?;
    }

    // 7b — PMI on the ROOT document, resolving through every occurrence path.
    //
    // A reference that reaches a part THROUGH an occurrence hangs its shape
    // aspect off that occurrence's `PRODUCT_DEFINITION_SHAPE`, not the part's:
    // `ACOMP3:top` and `ACOMP4:top` name the SAME `ADVANCED_FACE` (two
    // instances share the part's geometry, and the `GEOMETRIC_ITEM_SPECIFIC_-
    // USAGE` still points at it), but they are annotations on two different
    // INSTANCES, and the owner is the only place that distinction can live.
    // It is also what the importer reads the distinction back out of
    // (`io/step_import/assembly_pmi.rs`), which is what closes the round trip.
    if let Some(pmi) = pmi {
        let mut names = StepNameMaps::default();
        for (index, geometry) in geometries.iter().enumerate() {
            for path in &paths[index] {
                let owner = StepItemOwner {
                    product_shape: match path.last {
                        Some(slot) => nauo_shapes[slot],
                        None => definitions[index].product_shape,
                    },
                    representation: representations[index],
                };
                names.register(geometry, owner, &path.prefix, &path.world);
            }
        }
        let context = pmi::StepContext {
            product_shape: definitions[root].product_shape,
            representation: representations[root],
            geometry_context,
            length_unit: contexts.length_unit,
            angle_unit: contexts.angle_unit,
            faces: &names.faces,
            edges: &names.edges,
            vertices: &names.vertices,
        };
        report.pmi_unresolved_references += pmi::write_pmi(&mut writer, &context, pmi)?;
    }

    finish_step_file(
        writer,
        &assembly.products[root].name,
        timestamp,
        &mut report,
    )?;
    Ok(report)
}

