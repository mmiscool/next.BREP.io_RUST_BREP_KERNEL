//! PMI on a STRUCTURED import: which PART, or which OCCURRENCE, each
//! annotation belongs to.
//!
//! The flat lane reads a file's PMI once, against one name map covering every
//! solid in the file ([`super::pmi::read_step_pmi`]). A structured import has
//! no such thing: each PRODUCT becomes its own part document whose bodies are
//! named by that document's own `IMPORT3D1` feature, so two parts both call
//! their first face `IMPORT3D1_Face_0`, and an annotation has to be told which
//! of them it means.
//!
//! # One read, product-qualified names
//!
//! So the lift is still ONE read of the file. Every product's faces / edges /
//! vertices go into one [`NameMap`](super::pmi::NameMap), each entry tagged
//! with the product it belongs to:
//!
//! ```text
//!   3#IMPORT3D1_Face_7          face 7 of product 3's first body
//!   3@1042#IMPORT3D1_Face_7     …as seen through occurrence #1042
//! ```
//!
//! The `@` form is the file PINNING an annotation to an instance: a
//! `SHAPE_ASPECT` whose `of_shape` is an occurrence's
//! `PRODUCT_DEFINITION_SHAPE` (`PRODUCT_DEFINITION_SHAPE('','NAUO PRDDFN',
//! #nauo)`) says "this tolerance is on THIS bolt", where an aspect on the
//! part's own product shape says "this tolerance is on the bolt". That
//! distinction is the whole reason this module exists, and it is the one the
//! exporter writes back out (`io/step/assembly.rs`).
//!
//! # Where each annotation lands
//!
//! | the annotation's references | lands on | named |
//! |---|---|---|
//! | one product, unpinned | that PART's document | part-local (`IMPORT3D1_Face_7`) |
//! | one product, pinned to an occurrence | the ASSEMBLY document | [`OCCURRENCE_REF_PREFIX`] + the occurrence chain |
//! | two or more products | the ASSEMBLY document | ditto, per reference |
//! | none that resolved | the ASSEMBLY document | the file's own text, unchanged |
//!
//! "One lift per PART, not per occurrence" matches the exporter's rule
//! exactly: two instances of one part share the part's `ADVANCED_FACE`s, so an
//! annotation on the part IS an annotation on every instance, and writing it
//! once per occurrence would multiply it on the next round trip.
//!
//! # Why the occurrence chain, and not a component id
//!
//! The kernel does not know what the consumer will call its components — ACOMP
//! ids are minted by the import that consumes this ([`BREP_render`]'s
//! `model_io.rs`). So an occurrence-scoped reference carries the STEP
//! occurrence chain it resolves through, root first, and the consumer rewrites
//! it into its own namespace with [`rewrite_occurrence_refs`]. A chain that
//! the consumer cannot place (a sub-assembly node it flattened away) leaves
//! the reference unresolved and reports it — never a silent drop.

use super::assembly::{StepOccurrence, StepProduct};
use super::parse::Entity;
use super::pmi::{file_mentions_pmi, lift_pmi, NameMap};
use super::{HashMap, HashSet, Resolver};
use crate::feature_pipeline::pmi::{PmiAnnotation, PmiState, PmiView};

/// The reference-name prefix an OCCURRENCE-scoped annotation carries out of a
/// structured import, until its consumer rewrites it:
/// `OCC#<edge>#<edge>…:<part-local name>`, the occurrence chain root first.
/// An empty chain (`OCC:`) is the root product's own geometry.
///
/// `#` cannot occur in a name this importer mints, so a consumer finds every
/// one of them by prefix alone.
pub const OCCURRENCE_REF_PREFIX: &str = "OCC";

/// The feature id a structured import's part document gives its geometry —
/// `BREP_render`'s `native_part_document` writes exactly this, and a lifted
/// annotation's references have to name what that feature will stamp.
pub(super) const PART_IMPORT_FEATURE: &str = "IMPORT3D1";

/// Tag an already product-qualified name with the occurrence that pins it:
/// `3#Face` → `3@1042#Face`. Leaves a name with no product qualifier alone
/// (the flat lane's names, which have no occurrences to pin to).
pub(super) fn pin_occurrence(name: &str, occurrence: usize) -> String {
    match name.split_once('#') {
        Some((product, local)) if !product.contains('@') => {
            format!("{product}@{occurrence}#{local}")
        }
        _ => name.to_string(),
    }
}

/// Read a product-qualified name back: `(product index, pinned occurrence,
/// part-local name)`. `None` for a name this module did not qualify — an
/// unresolved reference, or a free note's absent one.
fn split_qualified(name: &str) -> Option<(usize, Option<usize>, &str)> {
    let (qualifier, local) = name.split_once('#')?;
    let (product, occurrence) = match qualifier.split_once('@') {
        Some((product, occurrence)) => (product, Some(occurrence.parse().ok()?)),
        None => (qualifier, None),
    };
    Some((product.parse().ok()?, occurrence, local))
}

/// What one structured import's PMI came to.
#[derive(Default)]
pub(super) struct AssemblyPmi {
    /// Parallel to the assembly's products: that part document's own PMI.
    pub per_product: Vec<Option<PmiState>>,
    /// The PMI that belongs to the ASSEMBLY document — occurrence-scoped
    /// annotations, cross-part ones, and everything with no geometry of its
    /// own (a free note, a saved view).
    pub assembly: Option<PmiState>,
    /// What did not come in cleanly. Every one of these is a KEPT annotation,
    /// never a dropped one.
    pub diagnostics: Vec<String>,
}

/// Each product's ordered face refs, one entry per body of
/// [`StepProduct::bodies`] — the pairing `read_step_assembly` collects while
/// it builds them, and the only thing that ties an `ADVANCED_FACE` in the file
/// to `IMPORT3D1_Face_i` in the part document.
pub(super) type ProductFaceRefs = Vec<Vec<Vec<usize>>>;

/// Lift a structured import's PMI (module docs). `scale_of` gives a product's
/// own millimetres-per-unit, because a mixed-unit assembly's vertex
/// coordinates are read per product exactly as its geometry is.
pub(super) fn lift(
    text: &str,
    entities: &HashMap<usize, Entity>,
    products: &[StepProduct],
    face_refs: &ProductFaceRefs,
    occurrences: &[StepOccurrence],
    roots: &[usize],
    global_scale: f64,
    mut scale_of: impl FnMut(usize) -> f64,
) -> AssemblyPmi {
    let mut out = AssemblyPmi {
        per_product: vec![None; products.len()],
        ..AssemblyPmi::default()
    };
    if !file_mentions_pmi(text) {
        return out;
    }

    // --- one name map, every product tagged --------------------------------
    let mut names = NameMap::empty(entities, global_scale);
    for (index, product) in products.iter().enumerate() {
        let Some(refs) = face_refs.get(index) else {
            continue;
        };
        let resolver = Resolver::new(entities, scale_of(product.pd_ref));
        let body_names =
            crate::feature_pipeline::imported_solid_names(product.bodies.len(), PART_IMPORT_FEATURE);
        names.add_bodies(entities, &resolver, &body_names, refs, &format!("{index}#"));
    }

    // --- which aspects the file pinned to an occurrence ---------------------
    let edges: HashSet<usize> = occurrences.iter().map(|edge| edge.edge_ref).collect();
    let pins = occurrence_pins(entities, &edges);

    let mut diagnostics = Vec::new();
    let Some(state) = lift_pmi(text, entities, &names, &pins, &mut diagnostics) else {
        out.diagnostics = diagnostics;
        return out;
    };
    out.diagnostics = diagnostics;

    // --- partition -----------------------------------------------------------
    let paths = OccurrencePaths::build(products.len(), occurrences, roots);
    let mut assembly = PmiState::default();
    let mut per_product: Vec<PmiState> = vec![PmiState::default(); products.len()];
    for view in &state.views {
        // One view of the file can hold annotations of several parts: each
        // document gets the view it needs, minted in its own id space, and a
        // document with nothing in a view never gets an empty one.
        let mut assembly_rows: Vec<PmiAnnotation> = Vec::new();
        let mut product_rows: Vec<Vec<PmiAnnotation>> = vec![Vec::new(); products.len()];
        for annotation in &view.annotations {
            let refs = subject_references(&annotation.params);
            let every = all_references(&annotation.params);
            let owners: Vec<usize> = {
                let mut owners: Vec<usize> = refs.iter().map(|(product, _, _)| *product).collect();
                owners.sort_unstable();
                owners.dedup();
                owners
            };
            let pinned = refs.iter().find_map(|(_, occurrence, _)| *occurrence);
            // The plane gets no vote on OWNERSHIP but it still has to resolve:
            // an annotation laid out in another part's face can only live in
            // the assembly, where both names exist.
            let outside = |product: usize| {
                every
                    .iter()
                    .any(|(other, occurrence, _)| *other != product || occurrence.is_some())
            };
            match (owners.as_slice(), pinned) {
                // A part's own annotation: one product, and the file never
                // said which instance. One lift for the part, whatever the
                // instance count.
                ([product], None) if !outside(*product) => {
                    let mut annotation = annotation.clone();
                    rewrite_references(&mut annotation.params, |_, _, local, _| local.to_string());
                    product_rows[*product].push(annotation);
                }
                // Everything else is the assembly's business: pinned to an
                // instance, spanning two parts, or naming no geometry at all.
                _ => {
                    let mut annotation = annotation.clone();
                    rewrite_references(
                        &mut annotation.params,
                        |product, occurrence, local, original| {
                            // A pin naming an occurrence no chain ends in
                            // (a malformed file) still tells us the PRODUCT,
                            // so fall back to that product's own chain rather
                            // than losing the reference entirely.
                            match paths
                                .chain(product, occurrence)
                                .or_else(|| paths.chain(product, None))
                            {
                                Some(chain) => occurrence_ref(&chain, local),
                                // Reachable from no root at all: keep the
                                // qualified name, which resolves to nothing and
                                // says which product it meant, rather than a
                                // bare part-local name that might resolve to
                                // something else.
                                None => original.to_string(),
                            }
                        },
                    );
                    for (product, occurrence, local) in &every {
                        if paths.chain(*product, *occurrence).is_some() {
                            continue;
                        }
                        out.diagnostics.push(if paths.chain(*product, None).is_some() {
                            format!(
                                "PMI: '{local}' names occurrence #{} of product {product}, which \
                                 no chain from a root ends in; the reference was resolved through \
                                 that product's first occurrence instead",
                                occurrence.unwrap_or_default()
                            )
                        } else {
                            format!(
                                "PMI: '{local}' annotates product {product}, which this import \
                                 placed through no occurrence at all; the reference is kept \
                                 unresolved"
                            )
                        });
                    }
                    assembly_rows.push(annotation);
                }
            }
        }
        if !assembly_rows.is_empty() {
            push_view(&mut assembly, view, assembly_rows);
        }
        for (product, rows) in product_rows.into_iter().enumerate() {
            if !rows.is_empty() {
                push_view(&mut per_product[product], view, rows);
            }
        }
    }
    for (index, state) in per_product.into_iter().enumerate() {
        if !state.views.is_empty() {
            out.per_product[index] = Some(state);
        }
    }
    if !assembly.views.is_empty() {
        out.assembly = Some(assembly);
    }
    out
}

/// Copy one lifted view into a document's state with the rows that belong to
/// it, minting the view's id in THAT document's id space.
fn push_view(state: &mut PmiState, source: &PmiView, annotations: Vec<PmiAnnotation>) {
    let id = state.next_id("VIEW");
    let annotations = annotations
        .into_iter()
        .map(|mut annotation| {
            let prefix = crate::feature_pipeline::pmi::pmi_type(&annotation.kind)
                .map(|def| def.short_name)
                .unwrap_or("PMI");
            let fresh = state.next_id(prefix);
            if let Some(object) = annotation.params.as_object_mut() {
                object.insert("id".into(), serde_json::Value::String(fresh));
            }
            annotation
        })
        .collect();
    state.views.push(PmiView {
        id,
        name: source.name.clone(),
        camera: source.camera.clone(),
        display: source.display.clone(),
        annotations,
    });
}

/// The field naming an annotation's ANNOTATION PLANE — the plane it is laid
/// out in, not what it is about.
const PLANE_FIELD: &str = "plane";

/// Every product-qualified reference an annotation's params carry, minus its
/// annotation plane.
///
/// The plane is excluded because it decides nothing about OWNERSHIP: it is
/// where the callout is drawn, and our own exporter writes one for every
/// annotation including a view-aligned note, which a re-import then binds to
/// whatever planar face it coincides with. Letting that decide would move a
/// free note with no subject at all onto whichever part happened to have a
/// parallel face — which is exactly what the round-trip test caught. The plane
/// is still REWRITTEN with everything else; it just does not get a vote.
fn all_references(params: &serde_json::Value) -> Vec<(usize, Option<usize>, String)> {
    let mut out = Vec::new();
    visit_strings(params, &mut |text| {
        if let Some((product, occurrence, local)) = split_qualified(text) {
            out.push((product, occurrence, local.to_string()));
        }
    });
    out
}

fn subject_references(params: &serde_json::Value) -> Vec<(usize, Option<usize>, String)> {
    let mut out = Vec::new();
    let mut push = |text: &str| {
        if let Some((product, occurrence, local)) = split_qualified(text) {
            out.push((product, occurrence, local.to_string()));
        }
    };
    match params {
        serde_json::Value::Object(map) => {
            for (key, value) in map {
                if key == PLANE_FIELD {
                    continue;
                }
                visit_strings(value, &mut push);
            }
        }
        other => visit_strings(other, &mut push),
    }
    out
}

/// Rewrite every product-qualified reference in place. The closure is handed
/// the product, the pinned occurrence, the part-local name, and the ORIGINAL
/// qualified text — which it can hand back when nothing better resolves.
fn rewrite_references(
    params: &mut serde_json::Value,
    mut name: impl FnMut(usize, Option<usize>, &str, &str) -> String,
) {
    map_strings(params, &mut |text| {
        split_qualified(text)
            .map(|(product, occurrence, local)| name(product, occurrence, local, text))
    });
}

fn visit_strings(value: &serde_json::Value, visit: &mut impl FnMut(&str)) {
    match value {
        serde_json::Value::String(text) => visit(text),
        serde_json::Value::Array(items) => items.iter().for_each(|item| visit_strings(item, visit)),
        serde_json::Value::Object(map) => map.values().for_each(|item| visit_strings(item, visit)),
        _ => {}
    }
}

fn map_strings(value: &mut serde_json::Value, map: &mut impl FnMut(&str) -> Option<String>) {
    match value {
        serde_json::Value::String(text) => {
            if let Some(replacement) = map(text) {
                *text = replacement;
            }
        }
        serde_json::Value::Array(items) => items.iter_mut().for_each(|item| map_strings(item, map)),
        serde_json::Value::Object(object) => {
            object.values_mut().for_each(|item| map_strings(item, map))
        }
        _ => {}
    }
}

/// `OCC#12#47:IMPORT3D1_Face_3` — an occurrence chain and a part-local name.
fn occurrence_ref(chain: &[usize], local: &str) -> String {
    let mut out = String::from(OCCURRENCE_REF_PREFIX);
    for edge in chain {
        out.push('#');
        out.push_str(&edge.to_string());
    }
    out.push(':');
    out.push_str(local);
    out
}

/// Read an occurrence-scoped reference back: `(chain, part-local name)`.
/// `None` for any other name.
pub fn occurrence_ref_parts(name: &str) -> Option<(Vec<usize>, &str)> {
    let (head, local) = name.split_once(':')?;
    let mut parts = head.split('#');
    if parts.next()? != OCCURRENCE_REF_PREFIX {
        return None;
    }
    let chain: Option<Vec<usize>> = parts.map(|edge| edge.parse().ok()).collect();
    Some((chain?, local))
}

/// Rewrite every occurrence-scoped reference in a lifted annotation's params
/// into the consumer's own namespace. `namespace` turns an occurrence chain
/// into the prefix a reference to that instance's geometry opens with
/// (`"ACOMP3:"`, or `""` for a component that IS the document); `None` from it
/// means the consumer placed no component for that chain, and the reference is
/// left naming the chain so the row reports itself rather than resolving to
/// the wrong instance.
///
/// The one function a consumer of [`StepAssembly::pmi`](super::StepAssembly)
/// needs: the chain encoding is this module's business, not theirs.
pub fn rewrite_occurrence_refs(
    params: &mut serde_json::Value,
    mut namespace: impl FnMut(&[usize]) -> Option<String>,
) -> usize {
    let mut unplaced = 0usize;
    map_strings(params, &mut |text| {
        let (chain, local) = occurrence_ref_parts(text)?;
        match namespace(&chain) {
            Some(prefix) => Some(format!("{prefix}{local}")),
            None => {
                unplaced += 1;
                None
            }
        }
    });
    unplaced
}

/// Shape aspects the file pinned to an occurrence: `SHAPE_ASPECT.of_shape` is
/// a `PRODUCT_DEFINITION_SHAPE` whose definition is one of this assembly's
/// `NEXT_ASSEMBLY_USAGE_OCCURRENCE`s.
///
/// An aspect on a PART's own product shape is absent here, and that is the
/// common case: part PMI travels with the part.
fn occurrence_pins(
    entities: &HashMap<usize, Entity>,
    edges: &HashSet<usize>,
) -> HashMap<usize, usize> {
    // The occurrence each PRODUCT_DEFINITION_SHAPE defines, for the shapes
    // that define one at all.
    let mut occurrence_of_shape: HashMap<usize, usize> = HashMap::default();
    for (id, entity) in entities {
        let Some(args) = entity.find("PRODUCT_DEFINITION_SHAPE") else {
            continue;
        };
        let Some(definition) = args.get(2).and_then(|value| value.as_ref_id().ok()) else {
            continue;
        };
        if edges.contains(&definition) {
            occurrence_of_shape.insert(*id, definition);
        }
    }
    let mut pins = HashMap::default();
    if occurrence_of_shape.is_empty() {
        return pins;
    }
    for (id, entity) in entities {
        let Some(args) = entity.find("SHAPE_ASPECT") else {
            continue;
        };
        let Some(of_shape) = args.get(2).and_then(|value| value.as_ref_id().ok()) else {
            continue;
        };
        if let Some(edge) = occurrence_of_shape.get(&of_shape) {
            pins.insert(*id, *edge);
        }
    }
    pins
}

/// How many root→node occurrence chains the walk records per product before it
/// stops. A real assembly has one per instance; the cap only bounds a
/// pathological diamond graph, whose chain count is exponential in its depth.
const MAX_CHAINS: usize = 4096;

/// Every occurrence chain that reaches each product, root first.
struct OccurrencePaths {
    chains: Vec<Vec<Vec<usize>>>,
}

impl OccurrencePaths {
    fn build(products: usize, occurrences: &[StepOccurrence], roots: &[usize]) -> Self {
        let mut chains: Vec<Vec<Vec<usize>>> = vec![Vec::new(); products];
        // Children by ascending `edge_ref`, so a product reached several ways
        // resolves to the same chain on every run.
        let mut edges: Vec<&StepOccurrence> = occurrences.iter().collect();
        edges.sort_by_key(|edge| edge.edge_ref);
        let mut total = 0usize;
        let mut stack: Vec<(usize, Vec<usize>)> =
            roots.iter().rev().map(|root| (*root, Vec::new())).collect();
        while let Some((product, chain)) = stack.pop() {
            if total >= MAX_CHAINS || product >= products {
                break;
            }
            total += 1;
            chains[product].push(chain.clone());
            for edge in edges.iter().rev() {
                if edge.parent != product || chain.contains(&edge.edge_ref) {
                    continue;
                }
                let mut next = chain.clone();
                next.push(edge.edge_ref);
                stack.push((edge.child, next));
            }
        }
        Self { chains }
    }

    /// The chain for one reference: the one ending in the pinned occurrence
    /// when the file pinned it, else the product's first chain (ascending
    /// `edge_ref` at every level, so it is the same chain on every run).
    fn chain(&self, product: usize, occurrence: Option<usize>) -> Option<Vec<usize>> {
        let chains = self.chains.get(product)?;
        match occurrence {
            Some(edge) => chains
                .iter()
                .find(|chain| chain.last() == Some(&edge))
                .or_else(|| chains.iter().find(|chain| chain.contains(&edge)))
                .cloned(),
            None => chains.first().cloned(),
        }
    }
}

