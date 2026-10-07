use super::*;
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(super) enum StepBody {
    SolidBrep(usize),
    BrepWithVoids(usize),
    /// One CLOSED_SHELL of a SHELL_BASED_SURFACE_MODEL. That entity is a surface-
    /// model *container* wrapping a LIST of shells; when those shells are closed
    /// they are full solid bodies authored via the surface-model form rather than
    /// MANIFOLD_SOLID_BREP, so each expands to its own body. `model_ref` is the
    /// SHELL_BASED_SURFACE_MODEL entity (identity), `shell_ref` the shell to build.
    SurfaceModelShell { model_ref: usize, shell_ref: usize },
}

impl StepBody {
    /// Total order for a stable, reproducible body sequence. Bodies are gathered
    /// from a HashMap (nondeterministic iteration), so callers sort by this before
    /// building; sibling shells of one surface model share `model_ref`, hence the
    /// `shell_ref` tiebreaker keeps their order fixed run-to-run.
    fn order_key(self) -> (usize, usize) {
        match self {
            Self::SolidBrep(entity_ref) | Self::BrepWithVoids(entity_ref) => (entity_ref, 0),
            Self::SurfaceModelShell {
                model_ref,
                shell_ref,
            } => (model_ref, shell_ref),
        }
    }
}

pub(super) fn step_body_for_entity(entity_ref: usize, entity: &Entity) -> Option<StepBody> {
    if entity.has("BREP_WITH_VOIDS") {
        Some(StepBody::BrepWithVoids(entity_ref))
    } else if entity.has("MANIFOLD_SOLID_BREP") || entity.has("FACETED_BREP") {
        Some(StepBody::SolidBrep(entity_ref))
    } else {
        None
    }
}

/// The CLOSED_SHELL references wrapped by a SHELL_BASED_SURFACE_MODEL
/// (`SHELL_BASED_SURFACE_MODEL(name, (#shell, ...))`), in author order.
fn surface_model_shell_refs(resolver: &Resolver, model_ref: usize) -> Option<Vec<usize>> {
    let args = resolver
        .get(model_ref)
        .ok()?
        .find("SHELL_BASED_SURFACE_MODEL")?;
    let list = args.get(1)?.as_list().ok()?;
    let mut refs = Vec::with_capacity(list.len());
    for value in list {
        refs.push(value.as_ref_id().ok()?);
    }
    Some(refs)
}

/// Append every solid body an entity contributes. Most bodies map 1→1 via
/// [`step_body_for_entity`]; a SHELL_BASED_SURFACE_MODEL fans out 1→many, one
/// [`StepBody::SurfaceModelShell`] per wrapped shell (a closed shell reconstructs
/// as a solid; a genuinely open shell simply fails the shared validation gate).
fn push_step_bodies(
    resolver: &Resolver,
    entity_ref: usize,
    entity: &Entity,
    out: &mut Vec<StepBody>,
) {
    if let Some(body) = step_body_for_entity(entity_ref, entity) {
        out.push(body);
        return;
    }
    if entity.has("SHELL_BASED_SURFACE_MODEL") {
        if let Some(shell_refs) = surface_model_shell_refs(resolver, entity_ref) {
            for shell_ref in shell_refs {
                out.push(StepBody::SurfaceModelShell {
                    model_ref: entity_ref,
                    shell_ref,
                });
            }
        }
    }
}

/// Every body the FLAT import builds, in the order it builds them.
pub(super) fn flat_step_bodies(
    resolver: &Resolver,
    entities: &HashMap<usize, Entity>,
) -> Vec<StepBody> {
    let mut bodies: Vec<StepBody> = Vec::new();
    for (id, entity) in entities.iter() {
        push_step_bodies(resolver, *id, entity, &mut bodies);
    }
    dedupe_step_bodies(resolver, &mut bodies);
    bodies
}

/// Build one body and the trims it accepted at a residual over the fit's bar
/// because the file's curve is not on their carriers
/// ([`builder::readings::BoundedTrim`]); the report lane names them.
pub(super) fn build_step_body(
    resolver: &Resolver,
    body: StepBody,
) -> Result<(BrepSolid, builder::readings::BodyReadings), StepBodyError> {
    build_step_body_captured_or_not(resolver, body, false).map(|(solid, _, readings)| (solid, readings))
}

fn build_step_body_captured_or_not(
    resolver: &Resolver,
    body: StepBody,
    capture: bool,
) -> Result<(BrepSolid, Option<builder::readings::TrimCapture>, builder::readings::BodyReadings), StepBodyError> {
    match body {
        StepBody::SolidBrep(entity_ref) => build_solid_inner(resolver, entity_ref, capture),
        StepBody::BrepWithVoids(entity_ref) => build_brep_with_voids_captured(resolver, entity_ref, capture),
        StepBody::SurfaceModelShell { shell_ref, .. } => {
            build_solid_from_shell_captured(resolver, shell_ref, capture)
        }
    }
}

/// [`build_step_body`] carrying what the importer did to every edge, for
/// [`crate::import_step_trim_readings`].
pub(super) fn build_step_body_captured(
    resolver: &Resolver,
    body: StepBody,
) -> Result<(BrepSolid, Option<builder::readings::TrimCapture>, builder::readings::BodyReadings), StepBodyError> {
    build_step_body_captured_or_not(resolver, body, true)
}

/// The colours the file's presentation entities assign to ONE built body:
/// the body's own, plus one per face in the body's shell/face order.
///
/// **How the per-face pairing is trusted.** Every lane of [`build_step_body`]
/// builds a shell's faces by `zip`-ing straight down the STEP shell's face-ref
/// list ([`build_shell_faces_in_namespace`] / [`build_solid_from_shell`]), and
/// every pass that runs afterwards — the pinched-vertex split, the orientation
/// flips, and the occurrence lane's `transform_brep` — mutates faces IN PLACE.
/// So face `i` of built shell `s` IS `#face_refs[s][i]`. This function re-walks
/// the same shell/face refs and CHECKS that pairing per shell (shell count, then
/// each shell's face count): a mismatch means the two walks have drifted, and
/// the honest answer is no face colours at all rather than colours on the wrong
/// faces. A body colour is unaffected by the check — it is keyed on the body's
/// own entity, not on a position.
pub(super) fn step_body_appearance(
    resolver: &Resolver,
    styles: &StyleTable,
    body: StepBody,
    solid: &BrepSolid,
) -> BodyAppearance {
    let mut appearance = BodyAppearance::default();
    if styles.is_empty() {
        return appearance;
    }
    let shell_refs = step_body_claimed_shells(resolver, body);

    // The body's own colour: the file may style the solid entity, or (for a
    // surface model) the model container, or the OUTER shell that stands for the
    // body. Only the outer shell — a BREP_WITH_VOIDS void shell is a cavity, not
    // the body, and its style is not the body's.
    let mut body_items: Vec<usize> = match body {
        StepBody::SolidBrep(entity_ref) | StepBody::BrepWithVoids(entity_ref) => vec![entity_ref],
        StepBody::SurfaceModelShell { model_ref, .. } => vec![model_ref],
    };
    body_items.extend(shell_refs.as_ref().and_then(|shells| shells.first()).copied());
    appearance.body = body_items
        .into_iter()
        .find_map(|item_ref| styles.color_for(item_ref));

    // Per-face colours, positionally paired and length-checked per shell.
    let Some(shell_refs) = shell_refs else {
        return appearance;
    };
    if shell_refs.len() != solid.shells.len() {
        return appearance;
    }
    let mut faces: Vec<Option<ImportedColor>> = Vec::new();
    let mut any = false;
    for (shell_ref, shell) in shell_refs.into_iter().zip(&solid.shells) {
        let Some(face_refs) = shell_face_refs(resolver, shell_ref) else {
            return appearance;
        };
        if face_refs.len() != shell.faces.len() {
            return appearance;
        }
        for face_ref in face_refs {
            let color = styles.color_for(face_ref);
            any |= color.is_some();
            faces.push(color);
        }
    }
    if any {
        appearance.faces = faces;
    }
    appearance
}

/// A shell's ordered ADVANCED_FACE references. Accepts OPEN_SHELL as well as
/// CLOSED_SHELL (the SHELL_BASED_SURFACE_MODEL lane can hold either), and
/// answers `None` — never an error — for anything it cannot read, because a
/// missing colour must not fail an import.
fn shell_face_refs(resolver: &Resolver, shell_ref: usize) -> Option<Vec<usize>> {
    let entity = resolver.get(shell_ref).ok()?;
    let list = entity
        .find("CLOSED_SHELL")
        .or_else(|| entity.find("OPEN_SHELL"))?
        .get(1)?
        .as_list()
        .ok()?;
    list.iter().map(|value| value.as_ref_id().ok()).collect()
}

pub(super) fn step_body_claimed_shells(resolver: &Resolver, body: StepBody) -> Option<Vec<usize>> {
    match body {
        StepBody::SolidBrep(entity_ref) => resolver
            .get(entity_ref)
            .ok()?
            .find("MANIFOLD_SOLID_BREP")
            .or_else(|| resolver.get(entity_ref).ok()?.find("FACETED_BREP"))
            .and_then(|args| args.get(1))
            .and_then(|value| value.as_ref_id().ok())
            .map(|shell_ref| vec![shell_ref]),
        StepBody::BrepWithVoids(entity_ref) => {
            let args = resolver.get(entity_ref).ok()?.find("BREP_WITH_VOIDS")?;
            let mut claims = vec![args.get(1)?.as_ref_id().ok()?];
            for oriented_value in args.get(2)?.as_list().ok()? {
                let oriented_ref = oriented_value.as_ref_id().ok()?;
                let oriented = resolver
                    .get(oriented_ref)
                    .ok()?
                    .find("ORIENTED_CLOSED_SHELL")?;
                claims.push(oriented.get(2)?.as_ref_id().ok()?);
            }
            Some(claims)
        }
        StepBody::SurfaceModelShell { shell_ref, .. } => Some(vec![shell_ref]),
    }
}

fn dedupe_step_bodies(resolver: &Resolver, bodies: &mut Vec<StepBody>) {
    bodies.sort_unstable_by_key(|body| body.order_key());
    let ordinary_claims: HashSet<usize> = bodies
        .iter()
        .filter(|body| matches!(body, StepBody::SolidBrep(_)))
        .filter_map(|body| step_body_claimed_shells(resolver, *body))
        .flatten()
        .collect();
    let mut seen = HashSet::default();
    bodies.retain(|body| {
        let Some(claims) = step_body_claimed_shells(resolver, *body) else {
            return true;
        };
        if matches!(body, StepBody::BrepWithVoids(_))
            && claims.iter().any(|shell| ordinary_claims.contains(shell))
        {
            return false;
        }
        if claims.iter().any(|shell| seen.contains(shell)) {
            return false;
        }
        seen.extend(claims);
        true
    });
}

/// What one STEP document imports as: the bodies, their appearance, and how
/// gracefully it went.
///
/// `appearances` is PARALLEL to `solids` — index `i` is body `i`'s colour — so
/// every arm that pushes a solid pushes its appearance in the same breath. A
/// file with no presentation entities fills it with defaults rather than leaving
/// it short, which keeps the "parallel" contract a `zip` can rely on.
pub(super) struct ImportedBodies {
    pub(super) solids: Vec<BrepSolid>,
    pub(super) appearances: Vec<BodyAppearance>,
    /// Parallel to `solids`: the STEP face entity of each built face, shell
    /// by shell (face `i` of body `b` IS `face_refs[b][i]`); empty for a body
    /// whose pairing could not be verified. The PMI reader's anchor map.
    pub(super) face_refs: Vec<Vec<usize>>,
    pub(super) failed: usize,
    pub(super) first_error: Option<String>,
    /// Every precision the file states, in mm, ascending
    /// (`parse::stated_precisions_mm`). Read by the import report's
    /// stated-precision consistency check; parsed here because the entity
    /// table does not outlive this function.
    pub(super) stated_precisions_mm: Vec<f64>,
    /// Parallel to `solids` on the flat lane: the trims each body accepted at
    /// a residual over the fit's bar because the file's curve is not on their
    /// carriers. EMPTY on the assembly-occurrence lane, which does not carry
    /// them yet (named in the 2026-09-26 record).
    pub(super) bounded_trims: Vec<Vec<builder::readings::BoundedTrim>>,
    /// Parallel to `solids` on the flat lane: what the shell-closure gate read
    /// on each ACCEPTED body (`builder::ClosureNote`). EMPTY on the
    /// assembly-occurrence lane, like `bounded_trims`.
    pub(super) closure_notes: Vec<builder::ClosureNote>,
    /// The bodies the closure gate REFUSED, typed, in file order — each is
    /// also counted in `failed` and the first is `first_error`'s text. The
    /// feature lane refuses the whole import on any of these; the report lane
    /// returns them beside the bodies that did import.
    pub(super) refused_bodies: Vec<KernelRefusal>,
    /// The bodies that failed to build with a typed cause that is NOT the
    /// closure gate's (`StepBodyError::closure_refusal`), in file order:
    /// counted in `failed`, never a whole-file refusal, skipped on an
    /// opportunistic sheet exactly as their text was.
    pub(super) failed_refusals: Vec<KernelRefusal>,
}

/// [`shell_face_refs`] for the PMI reader (a sibling module).
pub(super) fn shell_face_refs_pub(resolver: &Resolver, shell_ref: usize) -> Option<Vec<usize>> {
    shell_face_refs(resolver, shell_ref)
}

/// Shared core. Either RESOLVES THE ASSEMBLY — positioning each solid-bearing
/// part occurrence by the world transform composed down the assembly tree, and
/// producing one distinct copy per instance of a re-used part — or, for a file
/// with no assembly structure, imports every solid untransformed (identity)
/// exactly as before. In both paths a body that fails to reconstruct is skipped
/// (counted in `failed`) rather than aborting the whole file.
pub(super) fn collect_step_solids(text: &str) -> Result<ImportedBodies, String> {
    if !text.contains("ISO-10303-21") {
        return Err("step_import: not an ISO-10303-21 Part 21 file".into());
    }
    let debug_timing = std::env::var("BREP_DEBUG_STEP_LOOP").is_ok();
    let parse_started = debug_timing.then(web_time::Instant::now);
    let entities = parse_data_section(text)?;
    if let Some(started) = parse_started {
        eprintln!("PHASE parse: {:?} ({} entities)", started.elapsed(), entities.len());
    }
    let resolver = Resolver::new(&entities, derive_length_scale_mm(&entities));
    // The file's presentation entities, read ONCE. An uncoloured file leaves the
    // table empty and every appearance lookup short-circuits.
    let styles = StyleTable::read(&entities);
    let stated_precisions_mm = stated_precisions_mm(&entities);

    // A file that carries a product structure (NEXT_ASSEMBLY_USAGE_OCCURRENCE
    // edges, MAPPED_ITEM edges, or both) is resolved into positioned leaf
    // occurrences. If the graph resolves to no positioned body (partial/unknown
    // transform encoding) we fall through to the flat import, so a malformed
    // assembly still degrades to bodies-at-origin.
    //
    // A REFUSED mapping (a singular mapping transform) survives that
    // fallthrough. Otherwise a file whose ONLY structure is such a mapping would
    // import its bodies flat, at the origin, with nothing said: the geometry is
    // still the best answer available, but the user has to be told the placement
    // was dropped.
    let mut refused: Vec<StepMappedItemError> = Vec::new();
    let edges = structure_edges(&entities, &resolver);
    if !edges.is_empty() {
        let assembly_started = debug_timing.then(web_time::Instant::now);
        let occurrences = resolve_assembly(&entities, &resolver, &styles, &edges);
        if let Some(started) = assembly_started {
            eprintln!("PHASE assembly: {:?}", started.elapsed());
        }
        if !occurrences.solids.is_empty() {
            if std::env::var("BREP_DEBUG_STEP_ASM").is_ok() {
                eprintln!(
                    "[step-asm] {} positioned occurrence(s) from {} structure edge(s):",
                    occurrences.solids.len(),
                    edges.len()
                );
                for line in &occurrences.debug {
                    eprintln!("[step-asm]   {line}");
                }
            }
            return Ok(ImportedBodies {
                bounded_trims: Vec::new(),
                closure_notes: Vec::new(),
                refused_bodies: occurrences.refused_bodies,
                failed_refusals: occurrences.failed_refusals,
                solids: occurrences.solids,
                appearances: occurrences.appearances,
                face_refs: occurrences.face_refs,
                failed: occurrences.failed,
                first_error: occurrences.first_error,
                stated_precisions_mm,
            });
        }
        refused = occurrences.refusals;
    }

    // Flat import: a single part, or several solids already sharing one frame. A
    // body is a MANIFOLD_SOLID_BREP (analytic/NURBS shell) or a FACETED_BREP
    // (planar polygon closed-shell); both wrap a CLOSED_SHELL as their shell arg.
    let bodies = flat_step_bodies(&resolver, &entities);
    // Each body reconstructs an independent `BrepSolid` from the shared read-only
    // `Resolver` (its own `SolidBuilder`/id space, no cross-body state), so the
    // bodies fan out across a rayon pool under the `parallel` feature. `collect`
    // preserves body order, so the resulting solids, failure count, and
    // first_error are byte-identical to the sequential path (only faster).
    // Carry whether each body is an OPPORTUNISTIC surface-model shell: a
    // SHELL_BASED_SURFACE_MODEL closed shell that reconstructs becomes a solid,
    // but one that is an OPEN sheet is simply not a solid body — its failure must
    // not count toward `failed`/`first_error`, which callers read to judge whether
    // the genuine solid bodies imported cleanly.
    let loop_started = debug_timing.then(web_time::Instant::now);
    // The appearance is resolved HERE, beside the build, because it needs the
    // built solid (to length-check the per-face pairing) and the body's STEP
    // refs (to look the colours up) in the same place.
    let build = |body: StepBody| {
        (
            matches!(body, StepBody::SurfaceModelShell { .. }),
            build_step_body(&resolver, body).map(|(solid, readings)| {
                let appearance = step_body_appearance(&resolver, &styles, body, &solid);
                let face_refs = super::pmi::body_face_refs(&resolver, body, &solid);
                (solid, appearance, face_refs, readings)
            }),
        )
    };
    type BuiltBody = (
        bool,
        Result<(BrepSolid, BodyAppearance, Vec<usize>, builder::readings::BodyReadings), StepBodyError>,
    );
    #[cfg(feature = "parallel")]
    let results: Vec<BuiltBody> = {
        use rayon::prelude::*;
        bodies.into_par_iter().map(build).collect()
    };
    #[cfg(not(feature = "parallel"))]
    let results: Vec<BuiltBody> = bodies.into_iter().map(build).collect();
    if let Some(started) = loop_started {
        eprintln!("PHASE reconstruct: {:?} ({} bodies)", started.elapsed(), results.len());
    }
    let mut solids = Vec::new();
    let mut appearances = Vec::new();
    let mut face_refs = Vec::new();
    let mut bounded_trims = Vec::new();
    let mut closure_notes = Vec::new();
    let mut refused_bodies = Vec::new();
    let mut failed_refusals = Vec::new();
    // A mapping the structure lane refused, on a file whose structure yielded no
    // positioned solid at all: the bodies below come in UNPLACED, and the refusal
    // is what says why one of them is not where the file put it.
    let mut failed = refused.len();
    let mut first_error = refused.first().map(ToString::to_string);
    for (opportunistic, result) in results {
        match result {
            // An OPPORTUNISTIC (SHELL_BASED_SURFACE_MODEL) body that closes into a
            // SINGLE face is a full untrimmed primitive the author declared as a
            // surface model — reference/construction geometry, not a part (OCC
            // imports it as a non-solid sheet). Skip it so it isn't solidified into
            // a phantom body (e.g. 00012039's three full-torus sheets -> phantom
            // toroidal solids). Genuine multi-face sewn surface-model solids and all
            // MANIFOLD_SOLID_BREP parts (>= 2 faces) are unaffected.
            Ok((solid, _, _, _))
                if opportunistic
                    && solid.shells.iter().map(|s| s.faces.len()).sum::<usize>() <= 1 => {}
            // Solid, appearance, face refs and bounded trims are pushed
            // together — the vectors are parallel by construction, through
            // every skip arm below.
            Ok((solid, appearance, refs, readings)) => {
                solids.push(solid);
                appearances.push(appearance);
                face_refs.push(refs);
                bounded_trims.push(readings.bounded);
                closure_notes.push(readings.closure);
            }
            // An open surface-model sheet is not a solid: skip it, whether its
            // builder said so in text or, since 2026-10-03, in a typed class.
            Err(error) if opportunistic && !error.closure_refusal() => {}
            Err(error) => {
                // A closure refusal counts on an opportunistic shell too: it
                // reached the gate, so it closed topologically and built as a
                // solid; what it is not is sound.
                failed += 1;
                if first_error.is_none() {
                    first_error = Some(error.to_string());
                }
                let closure = error.closure_refusal();
                if let StepBodyError::Refused(refusal) = error {
                    if closure {
                        refused_bodies.push(refusal);
                    } else {
                        failed_refusals.push(refusal);
                    }
                }
            }
        }
    }
    Ok(ImportedBodies {
        solids,
        appearances,
        face_refs,
        failed,
        first_error,
        stated_precisions_mm,
        bounded_trims,
        closure_notes,
        refused_bodies,
        failed_refusals,
    })
}

// ---------------------------------------------------------------------------
// Assembly resolution: place & instance multi-body parts (ISO 10303-214 §5)
// ---------------------------------------------------------------------------
//
// A STEP assembly wires PRODUCT_DEFINITIONs (parts / sub-assemblies) together
// with NEXT_ASSEMBLY_USAGE_OCCURRENCE (NAUO) edges. Each NAUO carries a rigid
// placement of the child's local frame into the parent's frame, encoded as:
//
//   NAUO ─ (PRODUCT_DEFINITION_SHAPE "NAUO PRDDFN") ← CONTEXT_DEPENDENT_SHAPE_
//   REPRESENTATION → ( REPRESENTATION_RELATIONSHIP(rep_child, rep_parent) +
//   REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION( ITEM_DEFINED_TRANSFORMATION
//   (item_1, item_2) ) ) → a pair of AXIS2_PLACEMENT_3D.
//
// The transform mapping child-rep coordinates into the parent rep is
// T = M(item_2) · M(item_1)⁻¹, where M(placement) is that frame's local→world
// matrix. Composing T along the path from the ROOT assembly down to a leaf part
// gives that occurrence's WORLD transform; a part reached along several paths
// (e.g. AS1's six bolts) yields several distinct positioned copies.

pub(super) use crate::step_matrix::{mat4_mul, Mat4};

pub(super) fn mat4_identity() -> Mat4 {
    crate::step_matrix::MAT4_IDENTITY
}

/// The determinant of an affine's 3x3 linear block: the signed volume scale,
/// negative exactly for a reflection.
pub(super) fn mat4_determinant3(m: &Mat4) -> f64 {
    m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8])
        + m[2] * (m[4] * m[9] - m[5] * m[8])
}

/// The local→world matrix of an orthonormal placement frame: rotation columns
/// are the frame axes, the translation column is the origin.
pub(super) fn frame_matrix(frame: &Frame) -> Mat4 {
    [
        frame.x.x,
        frame.y.x,
        frame.z.x,
        frame.origin.x,
        frame.x.y,
        frame.y.y,
        frame.z.y,
        frame.origin.y,
        frame.x.z,
        frame.y.z,
        frame.z.z,
        frame.origin.z,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// Inverse of an affine 4×4 (invert the 3×3 linear block, then the translation).
pub(super) fn mat4_affine_inverse(m: &Mat4) -> Result<Mat4, String> {
    let (a, b, c) = (m[0], m[1], m[2]);
    let (d, e, f) = (m[4], m[5], m[6]);
    let (g, h, i) = (m[8], m[9], m[10]);
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if det.abs() <= 1e-14 {
        return Err("step_import: singular assembly transform".into());
    }
    let inv_det = 1.0 / det;
    let r = [
        (e * i - f * h) * inv_det,
        (c * h - b * i) * inv_det,
        (b * f - c * e) * inv_det,
        (f * g - d * i) * inv_det,
        (a * i - c * g) * inv_det,
        (c * d - a * f) * inv_det,
        (d * h - e * g) * inv_det,
        (b * g - a * h) * inv_det,
        (a * e - b * d) * inv_det,
    ];
    let (tx, ty, tz) = (m[3], m[7], m[11]);
    let it = [
        -(r[0] * tx + r[1] * ty + r[2] * tz),
        -(r[3] * tx + r[4] * ty + r[5] * tz),
        -(r[6] * tx + r[7] * ty + r[8] * tz),
    ];
    Ok([
        r[0], r[1], r[2], it[0], r[3], r[4], r[5], it[1], r[6], r[7], r[8], it[2], 0.0, 0.0, 0.0,
        1.0,
    ])
}

/// WHICH of the two STEP mechanisms an edge came from. The graph, the walk and
/// the transform composition are shared; only the entity the placement hangs
/// off differs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum StructureEdgeKind {
    /// A NEXT_ASSEMBLY_USAGE_OCCURRENCE between two PRODUCT_DEFINITIONs, placed
    /// by a CONTEXT_DEPENDENT_SHAPE_REPRESENTATION.
    Nauo,
    /// A MAPPED_ITEM in the parent's own representation, placed by its
    /// REPRESENTATION_MAP (`mapped.rs`).
    Mapped,
}

/// One parent→child edge of the structure graph.
#[derive(Clone, Copy)]
pub(crate) struct AssemblyEdge {
    /// The entity that IS this edge: a NEXT_ASSEMBLY_USAGE_OCCURRENCE, or a
    /// MAPPED_ITEM. Unique per file either way, which is what makes it the
    /// edge's identity and the deterministic sort key.
    pub(crate) edge_id: usize,
    /// Structure NODE ids, not necessarily PRODUCT_DEFINITIONs — see
    /// [`representation_owner_index`].
    pub(crate) parent_pd: usize,
    pub(crate) child_pd: usize,
    pub(crate) kind: StructureEdgeKind,
    /// The mapped item this edge IS, read once when the edge was built. `None`
    /// for a NAUO edge.
    pub(crate) mapped: Option<MappedItem>,
}

/// The NEXT_ASSEMBLY_USAGE_OCCURRENCE edges, sorted by id for determinism.
fn nauo_edges(entities: &HashMap<usize, Entity>) -> Vec<AssemblyEdge> {
    let mut edges: Vec<AssemblyEdge> = entities
        .iter()
        .filter_map(|(id, entity)| {
            let args = entity.find("NEXT_ASSEMBLY_USAGE_OCCURRENCE")?;
            let parent_pd = args.get(3)?.as_ref_id().ok()?;
            let child_pd = args.get(4)?.as_ref_id().ok()?;
            Some(AssemblyEdge {
                edge_id: *id,
                parent_pd,
                child_pd,
                kind: StructureEdgeKind::Nauo,
                mapped: None,
            })
        })
        .collect();
    edges.sort_by_key(|edge| edge.edge_id);
    edges
}

/// EVERY edge of the file's product structure — NAUO occurrences AND mapped
/// items — sorted by entity id. Empty ⇒ the file has no structure, and both
/// import lanes fall back to reading its bodies flat.
///
/// The two mechanisms compose because they are the same graph: a mapped item's
/// parent is the node whose representation carries it, so a mapped item inside a
/// NAUO child is one more edge below that child, and [`walk_occurrences`]
/// composes the transforms down the path without knowing which kind it walked.
///
/// # Discovery runs from the mapped items, not from the nodes
///
/// The scoping rule is unchanged and is the load-bearing part: a `MAPPED_ITEM`
/// is an occurrence only when a SHAPE representation's own item list names it.
/// An AP242 saved view maps the whole shape into a `DRAUGHTING_MODEL`, which
/// [`representation_args`] does not recognise, so its item is not a
/// representation item by this definition and never mints a phantom occurrence
/// (`mapped.rs`'s module comment, pinned on `io1-ac-214.stp`).
///
/// What changed is the DIRECTION the rule is applied in. Asking each node for
/// its representations means [`node_representations`] — a couple of full entity
/// scans per node — 118 times over on a 105k-entity building model, seconds of
/// it, to find the handful of mapped items a file has. Enumerating the item
/// lists instead answers the same question in ONE pass, and the two indexes
/// below turn each "which node owns this representation" into a lookup. The
/// cost is now proportional to the mapped items, which is what the feature
/// actually has.
///
/// One attribution differs, and only in a shape no fixture exhibits: a mapped
/// item carried by a product's ALTERNATIVE representation (identity-related to
/// its root, but not itself named by a `SHAPE_DEFINITION_REPRESENTATION`) used
/// to yield TWO edges with the same id, one parented at the product and one at
/// the bare representation, because both were seeds whose closures reached it.
/// It now yields one, parented at the product — which is the answer the graph
/// wants, and the reason [`node_of_representation`] follows identity relations.
pub(crate) fn structure_edges(
    entities: &HashMap<usize, Entity>,
    resolver: &Resolver,
) -> Vec<AssemblyEdge> {
    let mut edges = nauo_edges(entities);
    // No MAPPED_ITEM anywhere in the file ⇒ no mapped edge can exist, and not
    // even the item-list sweep below is worth paying to prove it.
    if !entities.values().any(|entity| entity.has("MAPPED_ITEM")) {
        return edges;
    }
    let owners = representation_owner_index(entities);
    let identity = identity_relation_index(entities);
    // Every SHAPE representation's own item list, once. `representation_items`
    // is the DRAUGHTING_MODEL exclusion: a saved view's item list is not one.
    let mut carriers: Vec<(usize, Vec<MappedItem>)> = entities
        .iter()
        .filter_map(|(id, entity)| {
            representation_items(entity)?;
            let items = representation_mapped_items(resolver, *id);
            (!items.is_empty()).then_some((*id, items))
        })
        .collect();
    carriers.sort_unstable_by_key(|(rep, _)| *rep);
    for (rep, items) in carriers {
        let parent = node_of_representation(rep, &owners, &identity);
        for item in items {
            // The CHILD is the mapped representation's own node, read exactly
            // as it always was: the product that owns that representation, else
            // the representation itself (a bare geometric template no product
            // owns). Identity relations are not followed here — a mapped
            // representation is named directly by its `REPRESENTATION_MAP`.
            let child = owners.get(&item.mapped_rep).copied().unwrap_or(item.mapped_rep);
            edges.push(AssemblyEdge {
                edge_id: item.item,
                parent_pd: parent,
                child_pd: child,
                kind: StructureEdgeKind::Mapped,
                mapped: Some(item),
            });
        }
    }
    edges.sort_by_key(|edge| edge.edge_id);
    edges
}

/// The structure NODE that owns a representation, following IDENTITY
/// relationships outward when the representation is not itself a product's
/// root: a product's alternative representation of the same geometry belongs to
/// that product, which is what [`node_representations`] expressed by walking
/// the relationship in the other direction.
///
/// The representation's OWN owner wins when it has one. Otherwise the lowest
/// owner id across its identity-connected component, so the answer does not
/// depend on `HashMap` order; and the representation itself when nothing in the
/// component is owned at all.
fn node_of_representation(
    rep: usize,
    owners: &HashMap<usize, usize>,
    identity: &HashMap<usize, Vec<usize>>,
) -> usize {
    if let Some(&owner) = owners.get(&rep) {
        return owner;
    }
    let mut visited: HashSet<usize> = HashSet::default();
    let mut queue: VecDeque<usize> = VecDeque::new();
    queue.push_back(rep);
    let mut best: Option<usize> = None;
    while let Some(current) = queue.pop_front() {
        if !visited.insert(current) {
            continue;
        }
        if let Some(&owner) = owners.get(&current) {
            best = Some(best.map_or(owner, |previous: usize| previous.min(owner)));
        }
        for &other in identity.get(&current).into_iter().flatten() {
            if !visited.contains(&other) {
                queue.push_back(other);
            }
        }
    }
    best.unwrap_or(rep)
}

/// representation → the PRODUCT_DEFINITION whose shape it IS:
/// SHAPE_DEFINITION_REPRESENTATION(used_representation = rep) →
/// PRODUCT_DEFINITION_SHAPE → PRODUCT_DEFINITION, built in ONE pass, with the
/// lowest id when a file names several so the answer is deterministic.
///
/// What a structure NODE is, since `node_of_representation` and this index are
/// the two places that answer it: a node id is a PRODUCT_DEFINITION or a
/// representation, and never ambiguous, because Part 21 entity ids are unique
/// within a file and the two entities are distinguishable by their own records
/// wherever that matters ([`pd_root_srs`], [`product_name`]). The representation
/// fallback is what lets a mapped representation with no product of its own — an
/// AP203 geometric template — still be a part in the structure, with the mapping
/// as its placement.
fn representation_owner_index(entities: &HashMap<usize, Entity>) -> HashMap<usize, usize> {
    let mut owners: HashMap<usize, usize> = HashMap::default();
    for entity in entities.values() {
        let Some(args) = entity.find("SHAPE_DEFINITION_REPRESENTATION") else {
            continue;
        };
        let (Some(pds_id), Some(rep)) = (
            args.first().and_then(|value| value.as_ref_id().ok()),
            args.get(1).and_then(|value| value.as_ref_id().ok()),
        ) else {
            continue;
        };
        let owner = entities
            .get(&pds_id)
            .and_then(|pds| pds.find("PRODUCT_DEFINITION_SHAPE"))
            .and_then(|pds_args| pds_args.get(2))
            .and_then(|value| value.as_ref_id().ok());
        // A PRODUCT_DEFINITION_SHAPE can also define a NAUO ('NAUO PRDDFN'),
        // which is an occurrence and not a product.
        let Some(owner) = owner.filter(|owner| {
            entities
                .get(owner)
                .is_some_and(|entity| entity.has("PRODUCT_DEFINITION"))
        }) else {
            continue;
        };
        owners
            .entry(rep)
            .and_modify(|existing| *existing = (*existing).min(owner))
            .or_insert(owner);
    }
    owners
}

/// representation → representations joined to it by an IDENTITY
/// SHAPE_REPRESENTATION_RELATIONSHIP, built in one pass. The whole-file form of
/// [`identity_related_srs`]; with-transform relationships (assembly placements
/// between different products) are excluded there and here.
fn identity_relation_index(entities: &HashMap<usize, Entity>) -> HashMap<usize, Vec<usize>> {
    let mut adjacency: HashMap<usize, Vec<usize>> = HashMap::default();
    for entity in entities.values() {
        if entity.has("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
            continue;
        }
        let Some(args) = entity.find("SHAPE_REPRESENTATION_RELATIONSHIP") else {
            continue;
        };
        if args.len() < 4 {
            continue;
        }
        let (Ok(rep1), Ok(rep2)) = (args[2].as_ref_id(), args[3].as_ref_id()) else {
            continue;
        };
        adjacency.entry(rep1).or_default().push(rep2);
        adjacency.entry(rep2).or_default().push(rep1);
    }
    adjacency
}

/// Every representation that carries a structure node's OWN geometry: its root
/// shape representation(s) and everything reachable from them by an IDENTITY
/// shape-representation relationship (a product's alternative representations of
/// the same geometry), sorted and deduped.
///
/// THE one enumeration of "which representations are this node's own", shared by
/// [`pd_step_bodies`] (which reads their solids) and [`structure_edges`] (which
/// reads their mapped items), so a representation cannot contribute geometry to
/// one and mappings to neither.
pub(super) fn node_representations(
    entities: &HashMap<usize, Entity>,
    resolver: &Resolver,
    node: usize,
) -> Vec<usize> {
    let mut visited: HashSet<usize> = HashSet::default();
    let mut queue: VecDeque<usize> = pd_root_srs(entities, node).into_iter().collect();
    let mut reps = Vec::new();
    while let Some(sr) = queue.pop_front() {
        if !visited.insert(sr) {
            continue;
        }
        if resolver.get(sr).is_ok() {
            reps.push(sr);
        }
        for other in identity_related_srs(entities, sr) {
            if !visited.contains(&other) {
                queue.push_back(other);
            }
        }
    }
    reps.sort_unstable();
    reps.dedup();
    reps
}

/// The arguments of a SHAPE_REPRESENTATION / ADVANCED_BREP_SHAPE_REPRESENTATION
/// (and the faceted/surface variants): `(name, items, context)`.
pub(super) fn representation_args(entity: &Entity) -> Option<&[Value]> {
    entity
        .find("ADVANCED_BREP_SHAPE_REPRESENTATION")
        .or_else(|| entity.find("FACETED_BREP_SHAPE_REPRESENTATION"))
        .or_else(|| entity.find("MANIFOLD_SURFACE_SHAPE_REPRESENTATION"))
        .or_else(|| entity.find("GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION"))
        .or_else(|| entity.find("SHAPE_REPRESENTATION"))
}

/// The item list of a shape representation — the geometry it carries.
pub(super) fn representation_items(entity: &Entity) -> Option<&[Value]> {
    representation_args(entity)?.get(1)?.as_list().ok()
}

/// The REPRESENTATION_CONTEXT a shape representation is expressed in — where
/// its unit assignment lives, hence the scale of every length it carries.
pub(super) fn representation_context(entity: &Entity) -> Option<usize> {
    representation_args(entity)?.get(2)?.as_ref_id().ok()
}

/// The root SHAPE_REPRESENTATION(s) that directly define a PRODUCT_DEFINITION's
/// shape (via PRODUCT_DEFINITION_SHAPE → SHAPE_DEFINITION_REPRESENTATION).
pub(super) fn pd_root_srs(entities: &HashMap<usize, Entity>, pd: usize) -> Vec<usize> {
    // A structure node that IS a representation (a mapped representation with no
    // PRODUCT_DEFINITION of its own — see `representation_owner_index`) is its own root
    // shape representation. A PRODUCT_DEFINITION never carries representation
    // arguments, so the two cases cannot be confused.
    if entities
        .get(&pd)
        .is_some_and(|entity| representation_args(entity).is_some())
    {
        return vec![pd];
    }
    let pds_ids: Vec<usize> = entities
        .iter()
        .filter_map(|(id, entity)| {
            let args = entity.find("PRODUCT_DEFINITION_SHAPE")?;
            (args.get(2)?.as_ref_id().ok()? == pd).then_some(*id)
        })
        .collect();
    let mut srs: Vec<usize> = entities
        .values()
        .filter_map(|entity| {
            let args = entity.find("SHAPE_DEFINITION_REPRESENTATION")?;
            let definition = args.first()?.as_ref_id().ok()?;
            let used = args.get(1)?.as_ref_id().ok()?;
            pds_ids.contains(&definition).then_some(used)
        })
        .collect();
    srs.sort_unstable();
    srs.dedup();
    srs
}

/// SRs joined to `sr` by an IDENTITY SHAPE_REPRESENTATION_RELATIONSHIP (a
/// product's alternative representations of the same geometry). With-transform
/// relationships (assembly placements between different products) are excluded.
fn identity_related_srs(entities: &HashMap<usize, Entity>, sr: usize) -> Vec<usize> {
    let mut out = Vec::new();
    for entity in entities.values() {
        if entity.has("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
            continue;
        }
        let Some(args) = entity.find("SHAPE_REPRESENTATION_RELATIONSHIP") else {
            continue;
        };
        if args.len() < 4 {
            continue;
        }
        let (Ok(rep1), Ok(rep2)) = (args[2].as_ref_id(), args[3].as_ref_id()) else {
            continue;
        };
        if rep1 == sr {
            out.push(rep2);
        } else if rep2 == sr {
            out.push(rep1);
        }
    }
    out
}

/// Solid-brep entity ids belonging to a PRODUCT_DEFINITION (in the part's own
/// local frame), gathered from its shape representation and every identity-
/// related representation.
pub(super) fn pd_step_bodies(
    entities: &HashMap<usize, Entity>,
    resolver: &Resolver,
    pd: usize,
) -> Vec<StepBody> {
    let mut bodies = Vec::new();
    for sr in node_representations(entities, resolver, pd) {
        let Ok(entity) = resolver.get(sr) else {
            continue;
        };
        let Some(items) = representation_items(entity) else {
            continue;
        };
        for item in items {
            if let Ok(item_id) = item.as_ref_id() {
                if let Ok(item_entity) = resolver.get(item_id) {
                    push_step_bodies(resolver, item_id, item_entity, &mut bodies);
                }
            }
        }
    }
    bodies.sort_unstable_by_key(|body| body.order_key());
    bodies.dedup();
    dedupe_step_bodies(resolver, &mut bodies);
    bodies
}

/// Every structure edge's placement, with the edges that cannot be instantiated
/// DROPPED and named.
///
/// One implementation for both import lanes — the flat one
/// ([`resolve_assembly`]) and the structured one
/// (`assembly::read_step_assembly`) — so they cannot disagree about a placement,
/// about which edges survive, or about what was refused. `scale_for` gives
/// millimetres per unit for a structure node's own representation: the flat lane
/// passes the one file-global scale it builds all its geometry at, and the
/// structured lane passes `ProductScales`, which resolves each node's own
/// context (§3.6).
///
/// A refused edge takes its whole subtree with it — nothing under a placement we
/// cannot express is instantiated — and the walk simply never reaches it,
/// because it is not in `edges`.
pub(super) struct ResolvedEdges {
    pub(super) edges: Vec<AssemblyEdge>,
    pub(super) transforms: HashMap<usize, Mat4>,
    pub(super) refusals: Vec<StepMappedItemError>,
}

pub(super) fn resolve_structure_edges(
    entities: &HashMap<usize, Entity>,
    edges: &[AssemblyEdge],
    mut scale_for: impl FnMut(usize) -> f64,
) -> ResolvedEdges {
    let mut resolved = ResolvedEdges {
        edges: Vec::with_capacity(edges.len()),
        transforms: HashMap::default(),
        refusals: Vec::new(),
    };
    for edge in edges {
        // The placement is expressed in the PARENT frame, so it is read at the
        // parent's scale; a mapped item's `mapping_origin` is the one exception,
        // being an item of the CHILD representation (`mapped.rs`).
        let parent_scale = scale_for(edge.parent_pd);
        let transform = match edge.kind {
            StructureEdgeKind::Nauo => {
                let parent = Resolver::new(entities, parent_scale);
                Ok(nauo_edge_transform(entities, &parent, edge))
            }
            StructureEdgeKind::Mapped => {
                let child_scale = scale_for(edge.child_pd);
                match &edge.mapped {
                    Some(item) => mapped_item_transform(entities, item, child_scale, parent_scale),
                    // Unreachable: a Mapped edge is only built from a MappedItem.
                    None => Ok(mat4_identity()),
                }
            }
        };
        match transform {
            Ok(transform) => {
                resolved.transforms.insert(edge.edge_id, transform);
                resolved.edges.push(*edge);
            }
            Err(refusal) => resolved.refusals.push(refusal),
        }
    }
    resolved
}

/// Resolve a NAUO's rigid placement (child-frame → parent-frame) from its
/// CONTEXT_DEPENDENT_SHAPE_REPRESENTATION. Returns identity when any link of the
/// transform chain is absent — a robust graceful fallback per the spec's option
/// that the transform entities may be partial.
pub(crate) fn nauo_edge_transform(
    entities: &HashMap<usize, Entity>,
    resolver: &Resolver,
    edge: &AssemblyEdge,
) -> Mat4 {
    let child_srs = pd_root_srs(entities, edge.child_pd);
    for entity in entities.values() {
        let Some(args) = entity.find("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") else {
            continue;
        };
        let Some(pds_id) = args.get(1).and_then(|value| value.as_ref_id().ok()) else {
            continue;
        };
        let Ok(pds) = resolver.get(pds_id) else {
            continue;
        };
        let owns_nauo = pds
            .find("PRODUCT_DEFINITION_SHAPE")
            .and_then(|pds_args| pds_args.get(2))
            .and_then(|value| value.as_ref_id().ok())
            == Some(edge.edge_id);
        if !owns_nauo {
            continue;
        }
        let Some(rep_rel_id) = args.first().and_then(|value| value.as_ref_id().ok()) else {
            continue;
        };
        if let Some(transform) = resolve_rep_rel_transform(resolver, rep_rel_id, &child_srs) {
            return transform;
        }
    }
    mat4_identity()
}

/// Build the child→parent transform from a (possibly complex) representation
/// relationship carrying a REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION. The
/// stored transform maps rep_1 → rep_2; we orient the result to child → parent
/// using the child's known root SR, so the graph's rep ordering does not matter.
fn resolve_rep_rel_transform(
    resolver: &Resolver,
    rep_rel_id: usize,
    child_srs: &[usize],
) -> Option<Mat4> {
    let entity = resolver.get(rep_rel_id).ok()?;
    let idt_id = entity
        .find("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")?
        .first()?
        .as_ref_id()
        .ok()?;
    let rel = entity.find("REPRESENTATION_RELATIONSHIP")?;
    let rep1 = rel.get(2)?.as_ref_id().ok()?;
    let rep2 = rel.get(3)?.as_ref_id().ok()?;
    let idt = resolver.get(idt_id).ok()?;
    let idt_args = idt.find("ITEM_DEFINED_TRANSFORMATION")?;
    let item1 = idt_args.get(2)?.as_ref_id().ok()?;
    let item2 = idt_args.get(3)?.as_ref_id().ok()?;
    let m1 = frame_matrix(&resolver.placement(item1).ok()?);
    let m2 = frame_matrix(&resolver.placement(item2).ok()?);
    // T maps rep_1 coordinates into rep_2 coordinates.
    let forward = mat4_mul(&m2, &mat4_affine_inverse(&m1).ok()?);
    if child_srs.contains(&rep2) && !child_srs.contains(&rep1) {
        // rep_2 is the child ⇒ invert so the result maps child → parent.
        mat4_affine_inverse(&forward).ok()
    } else {
        Some(forward)
    }
}

pub(super) struct AssemblyResult {
    pub(super) solids: Vec<BrepSolid>,
    /// Parallel to `solids`: each occurrence's STEP face refs (the part's).
    pub(super) face_refs: Vec<Vec<usize>>,
    /// Parallel to `solids`: the styled colour of the PART each occurrence
    /// instantiates. An occurrence is a placed copy of one part body, so every
    /// instance of a re-used part carries that part's colour.
    pub(super) appearances: Vec<BodyAppearance>,
    failed: usize,
    first_error: Option<String>,
    /// Part bodies the shell-closure gate refused, typed, once per part; each
    /// occurrence of such a part is also counted in `failed`.
    refused_bodies: Vec<KernelRefusal>,
    /// Part bodies that failed to build with a typed cause that is not the
    /// closure gate's, once per part; each occurrence is counted in `failed`.
    failed_refusals: Vec<KernelRefusal>,
    /// Mappings this lane would not instantiate, in edge order. Counted in
    /// `failed` and reported through `first_error` when this result is USED; kept
    /// separately so `collect_step_solids` can still report them when it abandons
    /// the assembly branch for want of a positioned solid.
    refusals: Vec<StepMappedItemError>,
    debug: Vec<String>,
}

/// The assembly ROOTS: products that appear as an edge parent but never as an
/// edge child. Sorted and deduped, so the occurrence walk is deterministic.
pub(super) fn assembly_roots(edges: &[AssemblyEdge]) -> Vec<usize> {
    let children: HashSet<usize> = edges.iter().map(|edge| edge.child_pd).collect();
    let mut roots: Vec<usize> = edges
        .iter()
        .map(|edge| edge.parent_pd)
        .filter(|pd| !children.contains(pd))
        .collect();
    roots.sort_unstable();
    roots.dedup();
    roots
}

/// One visited node of the occurrence walk: a PRODUCT_DEFINITION reached along
/// ONE path down from a root, carrying the world transform composed along that
/// path. A product used N times is visited N times, once per path.
pub(super) struct OccurrenceNode {
    pub(super) pd: usize,
    pub(super) world: Mat4,
    /// `root/child/grandchild` product-name path — the occurrence's label.
    pub(super) name: String,
    /// Every product from the root to this node inclusive: the cycle guard.
    ancestors: Vec<usize>,
}

/// Depth-first walk of the occurrence tree. THE one implementation of tree
/// shape, visit order, transform composition and cycle guarding — shared by the
/// flat lane ([`resolve_assembly`], which emits one positioned solid per visit)
/// and the structured lane (`assembly::read_step_assembly`, which uses it to
/// decide which products the structure actually reaches), so the two can never
/// disagree about any of them.
///
/// `transforms` maps a NAUO id to that edge's child-frame -> parent-frame
/// placement; a missing entry composes as identity. Roots are pushed reversed,
/// and each node's child edges are pushed reversed, so the id-sorted edges pop
/// in ascending NAUO order.
pub(super) fn walk_occurrences(
    resolver: &Resolver,
    edges: &[AssemblyEdge],
    transforms: &HashMap<usize, Mat4>,
    mut visit: impl FnMut(&OccurrenceNode),
) {
    let mut stack: Vec<OccurrenceNode> = assembly_roots(edges)
        .iter()
        .rev()
        .map(|&pd| OccurrenceNode {
            pd,
            world: mat4_identity(),
            name: product_name(resolver, pd),
            ancestors: vec![pd],
        })
        .collect();

    while let Some(node) = stack.pop() {
        visit(&node);

        // Descend into children.
        let mut child_edges: Vec<&AssemblyEdge> = edges
            .iter()
            .filter(|edge| edge.parent_pd == node.pd)
            .collect();
        child_edges.sort_by_key(|edge| edge.edge_id);
        for edge in child_edges.into_iter().rev() {
            if node.ancestors.contains(&edge.child_pd) {
                continue; // cycle guard
            }
            let transform = transforms
                .get(&edge.edge_id)
                .copied()
                .unwrap_or_else(mat4_identity);
            let world = mat4_mul(&node.world, &transform);
            let mut ancestors = node.ancestors.clone();
            ancestors.push(edge.child_pd);
            stack.push(OccurrenceNode {
                pd: edge.child_pd,
                world,
                name: format!("{}/{}", node.name, product_name(resolver, edge.child_pd)),
                ancestors,
            });
        }
    }
}

/// Enumerate every leaf occurrence in the assembly and emit a positioned solid
/// (the part's BrepSolid with its composed world transform applied). A part used
/// N times produces N distinct positioned copies. Bodies that fail to build or
/// transform are skipped (graceful degradation), never aborting the file.
///
/// A pure CONSUMER of [`walk_occurrences`]: this owns only the "what to emit at
/// each visited node" half, so the structured lane shares its tree walk exactly.
pub(super) fn resolve_assembly(
    entities: &HashMap<usize, Entity>,
    resolver: &Resolver,
    styles: &StyleTable,
    edges: &[AssemblyEdge],
) -> AssemblyResult {
    // Each edge's placement (child-frame -> parent-frame), through the shared
    // resolution the structured lane uses. This lane builds ALL its geometry at
    // one file-global scale, so it reads every placement at that same scale --
    // the pre-existing single-scale limitation `mixed-unit-assembly.step` pins,
    // and the reason a mixed-unit file is only correct through the structured
    // lane.
    let scale = resolver.length_scale;
    let ResolvedEdges {
        edges,
        transforms,
        refusals,
    } = resolve_structure_edges(entities, edges, |_| scale);
    let edges = edges.as_slice();

    let mut result = AssemblyResult {
        solids: Vec::new(),
        face_refs: Vec::new(),
        appearances: Vec::new(),
        failed: 0,
        first_error: None,
        refusals: refusals.clone(),
        refused_bodies: Vec::new(),
        failed_refusals: Vec::new(),
        debug: Vec::new(),
    };
    // A refused mapping — a singular mapping transform, the only kind
    // `mapped.rs` will not instantiate — is one placement that did not come in,
    // which is what `failed` counts here (it already counts an occurrence whose
    // `transform_brep` failed, not just a body that failed to build).
    for refusal in &refusals {
        result.failed += 1;
        if result.first_error.is_none() {
            result.first_error = Some(refusal.to_string());
        }
    }
    // Build each distinct part solid once — with its appearance, which is a
    // property of the PART, not of the placement — then re-use both for every
    // occurrence.
    let mut solid_cache: HashMap<StepBody, Result<(BrepSolid, BodyAppearance, Vec<usize>), StepBodyError>> =
        HashMap::default();
    let debug_on = std::env::var("BREP_DEBUG_STEP_ASM").is_ok();

    walk_occurrences(resolver, edges, &transforms, |node| {
        // Emit this product's own solids (leaf parts carry geometry; a pure
        // assembly node carries none, so this is a no-op there).
        for body in pd_step_bodies(entities, resolver, node.pd) {
            let base = solid_cache.entry(body).or_insert_with(|| {
                build_step_body(resolver, body)
                    .map(|(solid, _readings)| {
                        let appearance = step_body_appearance(resolver, styles, body, &solid);
                        let face_refs = super::pmi::body_face_refs(resolver, body, &solid);
                        (solid, appearance, face_refs)
                    })
                    .map_err(|error| {
                        error.with_context(|text| {
                            format!("step_import: part body {body:?} failed to build: {text}")
                        })
                    })
            });
            let (base_solid, base_appearance, base_refs) = match base {
                Ok((solid, appearance, refs)) => (solid.clone(), appearance.clone(), refs.clone()),
                // An OPEN SHELL_BASED_SURFACE_MODEL sheet is not a solid body, so
                // (as in the flat path) its build failure is skipped rather than
                // counted against the genuine solid bodies — in text or typed,
                // the closure gate's refusal excepted.
                Err(error)
                    if matches!(body, StepBody::SurfaceModelShell { .. }) && !error.closure_refusal() =>
                {
                    continue
                }
                Err(error) => {
                    result.failed += 1;
                    if result.first_error.is_none() {
                        result.first_error = Some(error.to_string());
                    }
                    let closure = error.closure_refusal();
                    if let StepBodyError::Refused(refusal) = error {
                        // Once per PART, not per occurrence: the cache hands
                        // the same refusal back for every placement.
                        let list = if closure { &mut result.refused_bodies } else { &mut result.failed_refusals };
                        if !list.iter().any(|known| known == refusal) {
                            list.push(refusal.clone());
                        }
                    }
                    continue;
                }
            };
            // A MIRRORED placement (a mapped item through a left-handed
            // CARTESIAN_TRANSFORMATION_OPERATOR_3D) must reverse orientation or
            // `transform_brep` refuses it, an unreversed reflection having turned
            // the solid inside out. This lane hands back POSED geometry, so it
            // bakes the reflection here; the structured lane hands back the
            // placement with `rigid: false` and its caller bakes a distinct part
            // (`BREP_render`'s `split_rigid`). Same answer, two shapes of it.
            match AffineTransform::new(node.world).and_then(|transform| {
                transform_brep(&base_solid, transform, transform.determinant3() < 0.0)
            }) {
                Ok(positioned) => {
                    if debug_on {
                        result.debug.push(format!(
                            "{} body{body:?} @ ({:.1}, {:.1}, {:.1})",
                            node.name, node.world[3], node.world[7], node.world[11]
                        ));
                    }
                    result.solids.push(positioned);
                    // `transform_brep` clones and mutates faces IN PLACE, so the
                    // positioned copy keeps the part's shell/face order and the
                    // part's per-face colour list still pairs index-for-index.
                    result.appearances.push(base_appearance);
                    result.face_refs.push(base_refs);
                }
                Err(error) => {
                    result.failed += 1;
                    if result.first_error.is_none() {
                        result.first_error = Some(error);
                    }
                }
            }
        }
    });

    result
}

/// The PRODUCT entity behind a PRODUCT_DEFINITION:
/// PRODUCT_DEFINITION → PRODUCT_DEFINITION_FORMATION[_WITH_SPECIFIED_SOURCE] →
/// PRODUCT. `None` when any link of the chain is absent.
fn product_entity<'a>(resolver: &'a Resolver, pd: usize) -> Option<&'a Entity> {
    let pd_entity = resolver.get(pd).ok()?;
    let formation_id = pd_entity
        .find("PRODUCT_DEFINITION")?
        .get(2)?
        .as_ref_id()
        .ok()?;
    let formation = resolver.get(formation_id).ok()?;
    let product_ref = formation
        .find("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE")
        .or_else(|| formation.find("PRODUCT_DEFINITION_FORMATION"))?
        .get(2)?
        .as_ref_id()
        .ok()?;
    resolver.get(product_ref).ok()
}

/// One string argument of the PRODUCT record, or `None` when it is absent or
/// not a string. `PRODUCT(id, name, description, frame_of_reference)`.
fn product_field(resolver: &Resolver, pd: usize, index: usize) -> Option<String> {
    match product_entity(resolver, pd)?.find("PRODUCT")?.get(index)? {
        Value::Str(text) => Some(text.clone()),
        _ => None,
    }
}

/// A PRODUCT_DEFINITION's product NAME — instrumentation, occurrence labels,
/// and (through the structured lane) the imported part's library name. Falls
/// back to the entity id when the PRODUCT chain is absent.
///
/// A structure node that is a REPRESENTATION rather than a product (a mapped
/// representation no PRODUCT_DEFINITION owns — see [`representation_owner_index`]) is named
/// by that representation, so it reaches the parts library as `SR1` rather than
/// as a bare entity id.
pub(super) fn product_name(resolver: &Resolver, pd: usize) -> String {
    if let Some(name) = representation_name(resolver, pd) {
        return name;
    }
    product_field(resolver, pd, 1).unwrap_or_else(|| format!("pd#{pd}"))
}

/// The `name` of a representation, when `id` IS one and the name is not blank.
fn representation_name(resolver: &Resolver, id: usize) -> Option<String> {
    let entity = resolver.get(id).ok()?;
    match representation_args(entity)?.first()? {
        Value::Str(text) if !text.trim().is_empty() => Some(text.trim().to_string()),
        _ => None,
    }
}

/// A PRODUCT_DEFINITION's product ID — the vendor part number, which the BOM
/// reports alongside the name. EMPTY when the file carries none: unlike the
/// name there is no sensible synthetic substitute, and a caller that wants one
/// falls back to [`product_name`].
pub(super) fn product_id(resolver: &Resolver, pd: usize) -> String {
    product_field(resolver, pd, 0).unwrap_or_default()
}

