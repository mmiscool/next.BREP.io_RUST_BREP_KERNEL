//! STRUCTURED STEP import: read the product structure and keep it.
//!
//! [`collect_step_solids`] resolves a STEP assembly graph and then throws the
//! structure away — it composes each occurrence's world transform, bakes it
//! into a copy of the part, and returns a flat `Vec<BrepSolid>`. That is the
//! right answer for "import these bodies", and it stays exactly as it is.
//!
//! This module is the other answer. It reads the SAME graph in the SAME single
//! parse and returns it: one [`StepProduct`] per structure node holding that
//! node's own bodies IN ITS OWN LOCAL FRAME, and one [`StepOccurrence`] per
//! structure EDGE — a NEXT_ASSEMBLY_USAGE_OCCURRENCE or a MAPPED_ITEM
//! (`mapped.rs`) — carrying the child→parent placement.
//! **Nothing here is transformed.** Composing the tree — and so choosing what a
//! "component" is — belongs to the caller, which maps products onto parts-library
//! entries and occurrences onto assembly-component instances.
//!
//! Two invariants make that safe:
//!
//! - **ONE parse, each body built ONCE.** The parse is shared with nothing (the
//!   caller passes the text once), and inside it every distinct [`StepBody`] is
//!   reconstructed a single time and cloned into its product, exactly as
//!   `resolve_assembly` caches per body.
//! - **The same walk.** Tree shape, visit order, transform composition and the
//!   cycle guard all come from [`walk_occurrences`], which the flat lane also
//!   uses, so the two lanes cannot drift apart. The kernel test suite asserts
//!   the strong form of this: replaying this module's output through the flat
//!   lane's own `transform_brep` call reproduces `resolve_assembly`'s solids
//!   BIT-for-bit on every single-context assembly fixture.
//!
//! **Units are resolved PER PRODUCT here**, not once per file.
//! `derive_length_scale_mm` picks one global scale out of `HashMap` iteration
//! order, which a mixed-unit assembly — each product representation carrying
//! its own `GLOBAL_UNIT_ASSIGNED_CONTEXT` — makes both wrong and
//! nondeterministic. See [`ProductScales`].

use super::*;
use crate::feature_pipeline::pmi::PmiState;

/// One node of the file's product structure: a PRODUCT_DEFINITION, or a
/// representation that a MAPPED_ITEM instantiates and no product definition owns
/// (`bodies::representation_owner_index`).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct StepProduct {
    /// The node's entity id — a PRODUCT_DEFINITION, or the representation that
    /// stands in for one. The stable identity and dedup key either way, because
    /// Part 21 ids are unique within a file.
    pub pd_ref: usize,
    /// `PRODUCT.name`, the representation's name for a node that is one, or
    /// `pd#<id>` when the file carries neither.
    pub name: String,
    /// `PRODUCT.id` — the vendor part number, for the BOM. May be empty.
    ///
    /// The CONSUMER is what makes this durable: the app's assembly intake
    /// records it as `partAttributes.Part_Number` on the part document it mints
    /// (`BREP_render`'s `model_io.rs::part_attributes`), which is the record
    /// `io/step/export_tree.rs` reads back out as `PRODUCT.id`. An intake that
    /// dropped it would silently replace a vendor's part numbers with our own
    /// file names on the first export -> import -> export.
    pub id: String,
    /// This product's OWN solids, in its OWN local frame and at its OWN unit
    /// scale. Empty ⇒ a pure assembly node (structure, no geometry).
    pub bodies: Vec<BrepSolid>,
    /// The styled colour of each entry of `bodies`, PARALLEL to it (index `i`
    /// is body `i`). Default-filled when the file carries no presentation
    /// entities, so it is always the same length as `bodies` and a caller can
    /// `zip` without a length check.
    pub appearances: Vec<BodyAppearance>,
    /// Bodies of this product that did not reconstruct. Graceful degradation:
    /// counted, never fatal, and never a reason to reject the file.
    pub failed_bodies: usize,
    /// The file's PMI that annotates THIS product's own geometry, with
    /// references naming the faces / edges / vertices the part document's
    /// `IMPORT3D1` feature will stamp (`assembly_pmi.rs`).
    ///
    /// ONE lift per PART, however many occurrences place it — an annotation on
    /// a part's face IS an annotation on every instance of that part, which is
    /// the exporter's rule read backwards (`io/step/assembly.rs`: two
    /// instances of one part share the part's `ADVANCED_FACE`s). PMI the file
    /// pinned to a particular INSTANCE is not here; it rides
    /// [`StepAssembly::pmi`].
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pmi: Option<PmiState>,
}

/// One occurrence: a placement of `child` inside `parent`, from a NAUO edge or a
/// mapped item.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct StepOccurrence {
    /// The entity that IS this occurrence: a NEXT_ASSEMBLY_USAGE_OCCURRENCE, or
    /// the MAPPED_ITEM that placed a shared representation (`mapped.rs`). Part 21
    /// ids are unique per file, so this identifies the occurrence either way, and
    /// consumers order sibling occurrences by it for determinism.
    pub edge_ref: usize,
    /// Index into [`StepAssembly::products`].
    pub parent: usize,
    /// Index into [`StepAssembly::products`].
    pub child: usize,
    /// The instance label ("bolt-3") — see [`edge_designator`] for which of a
    /// NAUO's several label attributes wins, and what a mapped item contributes.
    pub designator: String,
    /// child-local → parent-local, row-major affine, translations in MILLIMETRES
    /// (scaled by the PARENT representation's unit, which is the frame the
    /// placement is expressed in). NOT a world transform: the caller composes
    /// these down from [`StepAssembly::roots`].
    pub placement: [f64; 16],
    /// RᵀR = I and det R = +1 within 1e-9 — i.e. the placement is a pure
    /// rotation + translation, representable as a translate/rotate component
    /// pose. A mirrored or scaled occurrence has no such representation and its
    /// non-rigid factor must be baked into a distinct part by the caller, which
    /// is what `BREP_render`'s assembly intake does. A MAPPED_ITEM through a
    /// CARTESIAN_TRANSFORMATION_OPERATOR_3D is the placement that makes this
    /// `false` today (`mapped.rs`).
    pub rigid: bool,
}

/// A STEP file's product structure, with geometry, and without a single
/// transform applied to it.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct StepAssembly {
    /// Every product the structure mentions, ordered by ascending `pd_ref`.
    pub products: Vec<StepProduct>,
    /// One entry per structure edge, ordered by ascending `edge_ref`.
    pub occurrences: Vec<StepOccurrence>,
    /// Indices of the products that are never an occurrence child — where a
    /// consumer starts composing world transforms.
    pub roots: Vec<usize>,
    /// The first thing that did not come in: a body that failed to reconstruct,
    /// or a mapping that was refused ([`StepMappedItemError`]). A refused mapping
    /// is NOT counted in any product's `failed_bodies`, which counts bodies that
    /// failed to BUILD — a placement we cannot express is a different thing, and
    /// the geometry under it built perfectly well.
    pub first_error: Option<String>,
    /// The file's ASSEMBLY-level PMI: annotations the file pinned to an
    /// OCCURRENCE rather than a part, annotations spanning two parts, and
    /// annotations with no geometry of their own (a free note, a saved view).
    ///
    /// References to a component's geometry carry the STEP occurrence chain
    /// they resolve through (`OCCURRENCE_REF_PREFIX`): the kernel cannot know
    /// what the consumer will call its components, so the consumer rewrites
    /// them with `rewrite_occurrence_refs`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pmi: Option<PmiState>,
    /// What the PMI lift could not do cleanly — a reference naming geometry
    /// this import did not build, an occurrence chain nothing placed. Every
    /// one of these describes a KEPT annotation: PMI is never dropped for want
    /// of a reference, it arrives as a row that reports itself.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pmi_diagnostics: Vec<String>,
}

/// Read a STEP file's product structure AND build each product's own bodies, in
/// ONE parse, with each distinct body built ONCE.
///
/// `Ok(None)` when the file carries no assembly structure worth using — no
/// structure edges at all (neither NAUO nor mapped item), or none that the walk
/// reaches a built body through. That is the same condition on which
/// [`collect_step_solids`] abandons the assembly branch, so a caller can treat
/// `None` as "fall back to the flat lane" and get exactly today's behaviour.
///
/// `Err` only for a file that is not Part 21 or does not parse — a broken
/// *assembly* degrades, it does not fail.
pub fn read_step_assembly(text: &str) -> Result<Option<StepAssembly>, String> {
    if !text.contains("ISO-10303-21") {
        return Err("step_import: not an ISO-10303-21 Part 21 file".into());
    }
    let entities = parse_data_section(text)?;
    let global_scale = derive_length_scale_mm(&entities);
    let edges = structure_edges(&entities, &Resolver::new(&entities, global_scale));
    if edges.is_empty() {
        return Ok(None); // no product structure: the flat lane is the answer
    }

    let mut scales = ProductScales::new(&entities, global_scale);

    // Each edge's placement, read at the scale of the representation each of its
    // frames is an item of: an ITEM_DEFINED_TRANSFORMATION's AXIS2_PLACEMENT_3D
    // origins are lengths in the PARENT representation's context (§3.6.2), so
    // the parent's unit — not the file-global one — converts them to
    // millimetres, and a mapped item's `mapping_origin` is an item of the CHILD
    // representation and takes the child's (`mapped.rs`). The rotation blocks
    // are dimensionless and unaffected.
    //
    // (Strictly, a NAUO's `item_1` lives in the CHILD representation too, so a
    // file that BOTH mixes units AND gives item_1 a non-identity origin would
    // want its two frames scaled separately. Every producer we have seen writes
    // item_1 as the child rep's origin placement, where the two agree; splitting
    // them would mean reimplementing `nauo_edge_transform` rather than handing
    // it a Resolver, which is a worse trade for a case no fixture exhibits. A
    // MAPPED_ITEM is not in that position: its two frames are REQUIRED to be
    // items of the two different representations, so the split is the only
    // correct reading and `mapped-item-nested-nauo.step` exhibits it.)
    //
    // A mapping we cannot express as a rigid placement is REFUSED here: its edge
    // is dropped, so nothing under it is instantiated, and the refusal is
    // reported through `first_error`.
    let ResolvedEdges {
        edges,
        transforms,
        refusals,
    } = resolve_structure_edges(&entities, &edges, |node| scales.get(node));
    let mut first_error: Option<String> = refusals.first().map(ToString::to_string);
    if edges.is_empty() {
        return Ok(None); // every edge refused: nothing to build a structure from
    }

    // Every structure node the surviving edges mention, by ascending entity id.
    let mut pd_refs: Vec<usize> = edges
        .iter()
        .flat_map(|edge| [edge.parent_pd, edge.child_pd])
        .collect();
    pd_refs.sort_unstable();
    pd_refs.dedup();
    let product_index: HashMap<usize, usize> = pd_refs
        .iter()
        .enumerate()
        .map(|(index, &pd)| (pd, index))
        .collect();

    // Each product's own bodies, built at that product's own unit scale. The
    // cache is keyed by (body, scale) rather than by body alone: two products
    // can only share a body entity by sharing a representation, in which case
    // they share a context and the scale key is redundant — but if they ever do
    // not, the second product gets its own build instead of the first's.
    // The file's presentation entities, read ONCE for the whole document.
    let styles = StyleTable::read(&entities);
    let mut solid_cache: HashMap<(StepBody, u64), Result<(BrepSolid, BodyAppearance), String>> =
        HashMap::default();
    let mut products: Vec<StepProduct> = Vec::with_capacity(pd_refs.len());
    // The face refs of every body that built, in `bodies` order — the pairing
    // an annotation's `ADVANCED_FACE` is mapped to `IMPORT3D1_Face_i` through
    // (`assembly_pmi.rs`). Collected only when the file mentions PMI at all,
    // so a plain assembly pays nothing for it.
    let wants_pmi = super::pmi::file_mentions_pmi(text);
    let mut face_refs: super::assembly_pmi::ProductFaceRefs = Vec::with_capacity(pd_refs.len());
    for &pd in &pd_refs {
        let scale = scales.get(pd);
        let resolver = Resolver::new(&entities, scale);
        let scale_key = scale.to_bits();
        let mut bodies = Vec::new();
        let mut appearances = Vec::new();
        let mut product_face_refs = Vec::new();
        let mut failed_bodies = 0usize;
        for body in pd_step_bodies(&entities, &resolver, pd) {
            let built = solid_cache.entry((body, scale_key)).or_insert_with(|| {
                build_step_body(&resolver, body)
                    .map(|(solid, _readings)| {
                        let appearance = step_body_appearance(&resolver, &styles, body, &solid);
                        (solid, appearance)
                    })
                    .map_err(|error| {
                        format!("step_import: part body {body:?} failed to build: {error}")
                    })
            });
            match built {
                // Pushed together, so `bodies` and `appearances` stay parallel
                // through every skip arm below.
                Ok((solid, appearance)) => {
                    if wants_pmi {
                        product_face_refs.push(super::pmi::body_face_refs(&resolver, body, solid));
                    }
                    bodies.push(solid.clone());
                    appearances.push(appearance.clone());
                }
                // An OPEN SHELL_BASED_SURFACE_MODEL sheet is not a solid body:
                // skipped, not counted, exactly as both existing lanes do.
                Err(_) if matches!(body, StepBody::SurfaceModelShell { .. }) => {}
                Err(error) => {
                    failed_bodies += 1;
                    if first_error.is_none() {
                        first_error = Some(error.clone());
                    }
                }
            }
        }
        face_refs.push(product_face_refs);
        products.push(StepProduct {
            pd_ref: pd,
            name: product_name(&resolver, pd),
            id: product_id(&resolver, pd),
            bodies,
            appearances,
            failed_bodies,
            pmi: None,
        });
    }

    // Which products the structure actually REACHES — through the flat lane's
    // own walk, so the cycle guard and root set are identical. A graph that
    // reaches no geometry is not a structure we can import, so the caller falls
    // back to the flat lane, exactly as `collect_step_solids` does when
    // `resolve_assembly` yields no solid.
    //
    // (One corner differs harmlessly: a body that builds but fails
    // `transform_brep` yields no flat solid, yet counts as reached here.)
    let mut reached: HashSet<usize> = HashSet::default();
    {
        let resolver = Resolver::new(&entities, global_scale);
        walk_occurrences(&resolver, &edges, &transforms, |node| {
            reached.insert(node.pd);
        });
    }
    let reaches_geometry = products
        .iter()
        .any(|product| !product.bodies.is_empty() && reached.contains(&product.pd_ref));
    if !reaches_geometry {
        return Ok(None);
    }

    let label_resolver = Resolver::new(&entities, global_scale);
    let occurrences: Vec<StepOccurrence> = edges
        .iter()
        .map(|edge| {
            let placement = transforms
                .get(&edge.edge_id)
                .copied()
                .unwrap_or_else(mat4_identity);
            StepOccurrence {
                edge_ref: edge.edge_id,
                parent: product_index[&edge.parent_pd],
                child: product_index[&edge.child_pd],
                designator: edge_designator(&label_resolver, edge),
                placement,
                rigid: transform_is_rigid(&placement),
            }
        })
        .collect();

    let roots: Vec<usize> = assembly_roots(&edges)
        .into_iter()
        .map(|pd| product_index[&pd])
        .collect();

    // The file's PMI, split between the part documents this import will mint
    // and the assembly document that places them (`assembly_pmi.rs`). Read
    // here rather than by the consumer because this is where the file text,
    // the parse and each product's own face pairing are all in hand at once —
    // the same reason the geometry is built here.
    let mut pmi = super::assembly_pmi::AssemblyPmi::default();
    if wants_pmi {
        pmi = super::assembly_pmi::lift(
            text,
            &entities,
            &products,
            &face_refs,
            &occurrences,
            &roots,
            global_scale,
            |pd| scales.get(pd),
        );
        for (product, state) in products.iter_mut().zip(pmi.per_product.drain(..)) {
            product.pmi = state;
        }
    }

    Ok(Some(StepAssembly {
        products,
        occurrences,
        roots,
        first_error,
        pmi: pmi.assembly,
        pmi_diagnostics: pmi.diagnostics,
    }))
}

/// Per-product length units, memoised.
///
/// A product's geometry is expressed in the REPRESENTATION_CONTEXT of the shape
/// representation that defines it, and that context carries its own unit
/// assignment. `Resolver` is only `{entities, length_scale}`, so building a
/// product at its own scale costs one struct rebuild — which is the whole
/// implementation of "per-product units".
///
/// Falls back to the file-global scale for a product with no resolvable context
/// (a pure assembly node with no shape representation, typically), so a
/// single-context file resolves to the identical scale everywhere and the
/// structured lane's geometry stays byte-identical to the flat lane's.
pub(super) struct ProductScales<'a> {
    entities: &'a HashMap<usize, Entity>,
    global: f64,
    cache: HashMap<usize, f64>,
}

impl<'a> ProductScales<'a> {
    pub(super) fn new(entities: &'a HashMap<usize, Entity>, global: f64) -> Self {
        Self {
            entities,
            global,
            cache: HashMap::default(),
        }
    }

    /// Millimetres per native length unit for `pd`'s own representation. The
    /// candidate SRs come from `pd_root_srs`, which sorts and dedups, so a
    /// product with several representations resolves deterministically (unlike
    /// `derive_length_scale_mm`, which reads whatever the `HashMap` yields).
    pub(super) fn get(&mut self, pd: usize) -> f64 {
        if let Some(&scale) = self.cache.get(&pd) {
            return scale;
        }
        let scale = pd_root_srs(self.entities, pd)
            .into_iter()
            .filter_map(|sr| self.entities.get(&sr).and_then(representation_context))
            .find_map(|context| context_length_scale_mm(self.entities, context))
            .unwrap_or(self.global);
        self.cache.insert(pd, scale);
        scale
    }
}

/// Is this affine's linear block a pure rotation — RᵀR = I and det R = +1 to
/// 1e-9? A mirrored (det = −1) or scaled occurrence is NOT representable as a
/// component pose, and its caller must bake the non-rigid factor into a
/// distinct part rather than silently reuse the unmirrored twin.
///
/// A NAUO placement cannot fail this: it is built from two AXIS2_PLACEMENT_3Ds,
/// and `Resolver::placement` always returns an orthonormal right-handed frame. A
/// MAPPED_ITEM's can, and does — its target may be a
/// CARTESIAN_TRANSFORMATION_OPERATOR_3D with a left-handed triad or a `scale`
/// (`mapped.rs`) — so this flag is now LIVE rather than carried for a future
/// lane, and `mapped-item-mirrored.step` is the fixture that makes it false.
///
/// `false` is not a refusal. It is the statement that this placement is not a
/// component pose, which the caller answers by splitting `world = rigid · factor`
/// and baking `factor` into a distinct part — `BREP_render`'s `split_rigid` /
/// `build_library_entry`, whose mirrored part is named `… (mirrored)` and counted
/// in the import notice's "N mirrored instances baked". The flat import lane bakes
/// the same reflection directly into the posed geometry it returns
/// (`resolve_assembly`, through `transform_brep`'s orientation reversal), so the
/// two lanes agree about what the file means.
pub(super) fn transform_is_rigid(matrix: &Mat4) -> bool {
    const TOLERANCE: f64 = 1e-9;
    let column = |index: usize| [matrix[index], matrix[4 + index], matrix[8 + index]];
    let (x, y, z) = (column(0), column(1), column(2));
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    for (a, b, expected) in [
        (x, x, 1.0),
        (y, y, 1.0),
        (z, z, 1.0),
        (x, y, 0.0),
        (x, z, 0.0),
        (y, z, 0.0),
    ] {
        if (dot(a, b) - expected).abs() > TOLERANCE {
            return false;
        }
    }
    // det R, as the scalar triple product of the columns.
    let cross = [
        y[1] * z[2] - y[2] * z[1],
        y[2] * z[0] - y[0] * z[2],
        y[0] * z[1] - y[1] * z[0],
    ];
    (dot(x, cross) - 1.0).abs() <= TOLERANCE
}

/// The instance label of one structure edge: a mapped item's own name
/// (`mapped::mapped_item_designator`), or a NAUO's designator below.
fn edge_designator(resolver: &Resolver, edge: &AssemblyEdge) -> String {
    match edge.kind {
        StructureEdgeKind::Nauo => nauo_designator(resolver, edge.edge_id),
        StructureEdgeKind::Mapped => mapped_item_designator(resolver, edge.edge_id),
    }
}

/// The instance label of a NAUO —
/// `NEXT_ASSEMBLY_USAGE_OCCURRENCE(id, name, description, relating, related,
/// reference_designator)`.
///
/// Producers disagree about which attribute carries it, so take the first
/// NON-BLANK of `reference_designator` → `name` → `description` → `id`. The
/// three assembly fixtures each land on a different one: AssemblyExample and
/// io1-ac put it in `name`; as1-ug writes `name` as a single space and the real
/// designator ('ROD', 'NUT1', …) in `description`; a producer that fills
/// `reference_designator` is the standard-conforming case and wins outright.
fn nauo_designator(resolver: &Resolver, nauo: usize) -> String {
    let field = |args: &[Value], index: usize| match args.get(index) {
        Some(Value::Str(text)) if !text.trim().is_empty() => Some(text.trim().to_string()),
        _ => None,
    };
    (|| {
        let args = resolver
            .get(nauo)
            .ok()?
            .find("NEXT_ASSEMBLY_USAGE_OCCURRENCE")?;
        field(args, 5)
            .or_else(|| field(args, 1))
            .or_else(|| field(args, 2))
            .or_else(|| field(args, 0))
    })()
    .unwrap_or_else(|| format!("nauo#{nauo}"))
}
