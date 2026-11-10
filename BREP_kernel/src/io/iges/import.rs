//! IGES to kernel `BrepSolid`, by way of the STEP importer.
//!
//! Solids (186) and trimmed-surface sets (144, 143) are translated to Part 21
//! ([`super::to_step`]) so they get the STEP builder, closure gate and refusals.
//! Anything that cannot be read correctly is refused by name; a file never
//! imports short.

use crate::topology::BrepSolid;
use crate::{BodyAppearance, StepAssembly, StepOccurrence, StepProduct};
use rustc_hash::FxHashMap as HashMap;

use super::reader::{parse_iges, IgesFile};
use super::structure::{self, Matrix, Placed};
use super::to_step::{analyse_face_sets, free_shell_to_step, manifold_solid_to_step, FaceSets};

/// Parse an IGES document and reconstruct one or more `BrepSolid`s.
pub fn import_iges(text: &str) -> Result<Vec<BrepSolid>, String> {
    Ok(import_iges_report(text)?.solids)
}

/// What [`import_iges_report`] returns: the solids, and the diagnostics the
/// STEP lane measured while building them, merged over every body.
#[derive(Clone, Debug)]
pub struct IgesImportReport {
    /// The solids, in file order: solids standing on their own, then each
    /// subfigure instance's (408), placed.
    pub solids: Vec<BrepSolid>,
    /// Each solid's name (its 406 name property, else its directory label, else
    /// the name of the instance that placed it), parallel to `solids`; empty
    /// when the file gives none.
    pub names: Vec<String>,
    /// Each solid's colour (314, or the directory colour number), parallel to
    /// `solids`, in the shape the STEP lane returns colours in.
    pub appearances: Vec<BodyAppearance>,
    /// Each solid's level (layer) number from its directory entry, parallel to
    /// `solids`; 0 when unset, negative for a 406 list of levels.
    pub levels: Vec<i64>,
    /// The file's stated minimum resolution in mm (Global field 19), when it
    /// states one.
    pub stated_resolution_mm: Option<f64>,
    /// Measurements and warnings, as `import_step_report` gives for STEP:
    /// stated-precision consistency, bounded trims, closure readings.
    pub diagnostics: crate::KernelDiagnostics,
}

/// [`import_iges`] with the import's diagnostics, the IGES counterpart of
/// [`crate::import_step_report`].
pub fn import_iges_report(text: &str) -> Result<IgesImportReport, String> {
    let file = parse_iges(text)?;
    let scale = file.global.length_scale_mm()?;
    let stated = file.global.min_resolution * scale;
    let mut diagnostics = crate::KernelDiagnostics::default();
    let bodies = solids_with_diagnostics(&file, scale, &mut diagnostics)?;
    let mut report = IgesImportReport {
        solids: Vec::with_capacity(bodies.len()),
        names: Vec::with_capacity(bodies.len()),
        appearances: Vec::with_capacity(bodies.len()),
        levels: Vec::with_capacity(bodies.len()),
        stated_resolution_mm: (stated > 0.0 && stated.is_finite()).then_some(stated),
        diagnostics,
    };
    for body in bodies {
        report.solids.push(body.solid);
        report.names.push(body.name.unwrap_or_default());
        report.appearances.push(BodyAppearance { body: body.color, faces: Vec::new() });
        report.levels.push(body.level);
    }
    Ok(report)
}

/// One imported solid and what the file said about it.
struct Body {
    solid: BrepSolid,
    name: Option<String>,
    color: Option<crate::ImportedColor>,
    level: i64,
}

impl Body {
    fn bare(solid: BrepSolid) -> Self {
        Self { solid, name: None, color: None, level: 0 }
    }
}

/// Each 186 built once, through the STEP lane, in its definition frame.
type SolidCache = HashMap<i64, Vec<BrepSolid>>;

fn built_solid<'c>(
    cache: &'c mut SolidCache,
    file: &IgesFile,
    pointer: i64,
    scale: f64,
    diagnostics: &mut crate::KernelDiagnostics,
) -> Result<&'c [BrepSolid], String> {
    if !cache.contains_key(&pointer) {
        let (built, measured) = manifold_solid(file, pointer, scale)?;
        merge_diagnostics(diagnostics, measured);
        cache.insert(pointer, built);
    }
    Ok(&cache[&pointer])
}

/// A built solid moved by a placement; a mirroring placement turns it inside
/// out unless the orientation is reversed with it.
fn place(solid: &BrepSolid, placement: &Matrix) -> Result<BrepSolid, String> {
    if structure::is_identity(placement) {
        return Ok(solid.clone());
    }
    let transform = crate::AffineTransform::new(*placement)?;
    let mirrored = transform.determinant3() < 0.0;
    crate::transform_brep(solid, transform, mirrored)
}

fn solids_with_diagnostics(
    file: &IgesFile,
    scale: f64,
    diagnostics: &mut crate::KernelDiagnostics,
) -> Result<Vec<Body>, String> {
    // A file with explicit solids or shells goes down the STEP lane whole: same
    // builder, same closure gate, same refusals. A solid a subfigure defines
    // comes in once per instance, where the instance puts it; a shell no solid
    // uses comes in as the STEP lane reads a surface model.
    if file.entities_of_type(186).next().is_some() || file.entities_of_type(514).next().is_some() {
        refuse_uncovered_geometry(file)?;
        let mut cache = SolidCache::default();
        let mut bodies = Vec::new();
        for Placed { solid, placement, name, color, level } in structure::placed_solids(file, scale)? {
            for built in built_solid(&mut cache, file, solid, scale, diagnostics)? {
                bodies.push(Body { solid: place(built, &placement)?, name: name.clone(), color, level });
            }
        }
        bodies.extend(free_shell_bodies(file, scale, diagnostics)?);
        if bodies.is_empty() {
            let sheets = diagnostics.counters.get(crate::step_import::SHEETS_SKIPPED).copied().unwrap_or(0);
            return Err(format!(
                "iges_import: the file has no solids: its {sheets} shell(s) are sheets (open, or a single untrimmed face), not solids"
            ));
        }
        return Ok(bodies);
    }
    if file.entities_of_type(408).next().is_some() {
        return Err(
            "iges_import: the file places subfigures (408) of trimmed surfaces, which this import cannot build yet"
                .into(),
        );
    }

    if file.entities_of_type(144).next().is_none() && file.entities_of_type(143).next().is_none() {
        if file.entities_of_type(128).next().is_some() {
            return Err(
                "iges_import: file has untrimmed B-spline surfaces (entity 128) but no trimmed surfaces (entity 144) to form faces"
                    .into(),
            );
        }
        return Err("iges_import: the file has no solids (186) or trimmed surfaces (144, 143) to import".into());
    }
    let (solids, measured) = face_set_solids(file, scale)?;
    for each in measured {
        merge_diagnostics(diagnostics, each);
    }
    Ok(solids.into_iter().map(Body::bare).collect())
}

/// The file's subfigure structure, the IGES counterpart of
/// [`crate::read_step_assembly`]: one product for the file itself holding the
/// solids that stand on their own, one per instanced subfigure (308) holding
/// its own solids in its own frame, and one occurrence per instance (408) with
/// its placement in millimetres. `Ok(None)` when the file places no
/// subfigures, so the caller takes the flat import as for STEP.
///
/// Every solid is built once, as in the flat import, and a file the flat
/// import refuses is refused here with the same words: an assembly never
/// comes in short.
pub fn read_iges_assembly(text: &str) -> Result<Option<StepAssembly>, String> {
    let file = parse_iges(text)?;
    let members = structure::subfigure_member_set(&file)?;
    let top_instances: Vec<i64> = file
        .pointers_of_type(408)
        .into_iter()
        .map(|(pointer, _)| pointer)
        .filter(|pointer| !members.contains(pointer))
        .collect();
    if top_instances.is_empty() {
        return Ok(None);
    }
    // The flat import's checks and refusals, once, before any structure.
    let scale = file.global.length_scale_mm()?;
    if file.entities_of_type(186).next().is_none() {
        return Err(
            "iges_import: the file places subfigures (408) of trimmed surfaces, which this import cannot build yet"
                .into(),
        );
    }
    refuse_uncovered_geometry(&file)?;
    structure::placed_solids(&file, scale)?;

    let mut cache = SolidCache::default();
    let mut diagnostics = crate::KernelDiagnostics::default();
    let root_name = {
        let product = file.global.product_id.trim();
        if product.is_empty() { "IGES".to_string() } else { product.to_string() }
    };
    let mut assembly = StepAssembly {
        products: vec![StepProduct {
            pd_ref: 0,
            name: root_name,
            id: String::new(),
            bodies: Vec::new(),
            appearances: Vec::new(),
            failed_bodies: 0,
            pmi: None,
        }],
        occurrences: Vec::new(),
        roots: vec![0],
        first_error: None,
        pmi: None,
        pmi_diagnostics: Vec::new(),
        sheets_skipped: 0,
    };
    for (pointer, solid) in file.pointers_of_type(186) {
        if !members.contains(&pointer) {
            let color = structure::entity_color(&file, solid);
            for built in built_solid(&mut cache, &file, pointer, scale, &mut diagnostics)? {
                assembly.products[0].bodies.push(built.clone());
                assembly.products[0].appearances.push(BodyAppearance { body: color, faces: Vec::new() });
            }
        }
    }
    for body in free_shell_bodies(&file, scale, &mut diagnostics)? {
        assembly.products[0].bodies.push(body.solid);
        assembly.products[0].appearances.push(BodyAppearance { body: body.color, faces: Vec::new() });
    }
    assembly.sheets_skipped = diagnostics.counters.get(crate::step_import::SHEETS_SKIPPED).copied().unwrap_or(0) as usize;
    let mut products: HashMap<i64, usize> = HashMap::default();
    for pointer in top_instances {
        add_instance(&file, scale, pointer, 0, &mut assembly, &mut products, &mut cache, &mut diagnostics, 0)?;
    }
    Ok(Some(assembly))
}

/// One 408 as an occurrence under `parent`, its subfigure as a product (added
/// once, however many instances place it), and the subfigure's own instances
/// beneath it.
#[allow(clippy::too_many_arguments)]
fn add_instance(
    file: &IgesFile,
    scale: f64,
    pointer: i64,
    parent: usize,
    assembly: &mut StepAssembly,
    products: &mut HashMap<i64, usize>,
    cache: &mut SolidCache,
    diagnostics: &mut crate::KernelDiagnostics,
    depth: usize,
) -> Result<(), String> {
    if depth > 64 {
        return Err(format!("iges_import: instance DE {pointer} nests deeper than 64 levels"));
    }
    let instance = file.entity(pointer)?;
    let (definition_pointer, definition) = structure::instanced_subfigure(file, pointer)?;
    let child = match products.get(&definition_pointer) {
        Some(&index) => index,
        None => {
            let index = assembly.products.len();
            products.insert(definition_pointer, index);
            assembly.products.push(StepProduct {
                pd_ref: definition_pointer as usize,
                name: structure::entity_name(file, definition).unwrap_or_else(|| format!("subfigure#{definition_pointer}")),
                id: String::new(),
                bodies: Vec::new(),
                appearances: Vec::new(),
                failed_bodies: 0,
                pmi: None,
            });
            for member in structure::subfigure_members(definition)? {
                let entity = file.entity(member)?;
                match entity.de.entity_type {
                    186 => {
                        let color = structure::entity_color(file, entity);
                        let built = built_solid(cache, file, member, scale, diagnostics)?.to_vec();
                        let product = &mut assembly.products[index];
                        for solid in built {
                            product.bodies.push(solid);
                            product.appearances.push(BodyAppearance { body: color, faces: Vec::new() });
                        }
                    }
                    408 => add_instance(file, scale, member, index, assembly, products, cache, diagnostics, depth + 1)?,
                    _ => {}
                }
            }
            index
        }
    };
    let placement = structure::instance_matrix(file, instance, scale)?;
    let rigid = crate::AffineTransform::new(placement).is_ok_and(|transform| transform.is_rigid(1e-9));
    assembly.occurrences.push(StepOccurrence {
        edge_ref: pointer as usize,
        parent,
        child,
        designator: structure::entity_name(file, instance).unwrap_or_else(|| format!("instance#{pointer}")),
        placement,
        rigid,
    });
    Ok(())
}

/// Fold one body's diagnostics into the file's: events and counts add up, and
/// each measurement keeps its worst reading, as `measure_max` does.
fn merge_diagnostics(into: &mut crate::KernelDiagnostics, from: crate::KernelDiagnostics) {
    into.events.extend(from.events);
    for (key, count) in from.counters {
        *into.counters.entry(key).or_insert(0) += count;
    }
    for (key, value) in from.measurements {
        let slot = into.measurements.entry(key).or_insert(value);
        if value > *slot {
            *slot = value;
        }
    }
}

/// Build a file of trimmed surfaces: one solid per connected, closed set of
/// faces, and a set lying inside another becomes its cavity.
fn face_set_solids(
    file: &IgesFile,
    scale: f64,
) -> Result<(Vec<BrepSolid>, Vec<crate::KernelDiagnostics>), String> {
    let sets = analyse_face_sets(file, scale)?;
    for body in &sets.bodies {
        if let Some(reason) = sets.open_reason(body) {
            return Err(reason);
        }
    }
    let mut built = Vec::with_capacity(sets.bodies.len());
    for body in &sets.bodies {
        built.push(face_set_body(&sets, file, scale, body, &[])?);
    }
    // Largest first, so a cavity is always tested against what can hold it.
    let mut order: Vec<usize> = (0..built.len()).collect();
    order.sort_by(|&a, &b| built[b].volume.total_cmp(&built[a].volume));
    let mut cavities: Vec<Vec<usize>> = vec![Vec::new(); built.len()];
    let mut is_cavity = vec![false; built.len()];
    for (rank, &inner) in order.iter().enumerate() {
        for &outer in &order[..rank] {
            if !is_cavity[outer] && inside(&built[inner].solid, &built[outer].solid)? {
                cavities[outer].push(inner);
                is_cavity[inner] = true;
                break;
            }
        }
    }
    let mut solids = Vec::with_capacity(built.len());
    let mut measured = Vec::with_capacity(built.len());
    for index in 0..built.len() {
        if is_cavity[index] {
            continue;
        }
        if cavities[index].is_empty() {
            solids.push(built[index].solid.clone());
            measured.push(built[index].diagnostics.clone());
            continue;
        }
        let voids: Vec<(&[usize], bool)> = cavities[index]
            .iter()
            .map(|&cavity| (sets.bodies[cavity].as_slice(), !built[cavity].flip))
            .collect();
        let with_cavities = face_set_body(&sets, file, scale, &sets.bodies[index], &voids)?;
        solids.push(with_cavities.solid);
        measured.push(with_cavities.diagnostics);
    }
    Ok((solids, measured))
}

struct BuiltBody {
    solid: BrepSolid,
    volume: f64,
    flip: bool,
    diagnostics: crate::KernelDiagnostics,
}

/// One closed face set through the STEP lane, in whichever sense gives it a
/// positive volume; a twice-walked edge is read as written first.
fn face_set_body(
    sets: &FaceSets,
    file: &IgesFile,
    scale: f64,
    body: &[usize],
    voids: &[(&[usize], bool)],
) -> Result<BuiltBody, String> {
    let mut first_failure = None;
    for split in [false, true] {
        for flip in [false, true] {
            let text = sets.to_step(file, scale, body, voids, flip, split)?;
            match solids_or_refusal(&text) {
                Ok((mut solids, diagnostics)) if solids.len() == 1 => {
                    let solid = solids.remove(0);
                    let volume = crate::solid_signed_volume(&solid)?;
                    if volume > 0.0 {
                        return Ok(BuiltBody { solid, volume, flip, diagnostics });
                    }
                }
                Ok(_) => {}
                Err(error) => {
                    first_failure.get_or_insert(error);
                }
            }
        }
    }
    Err(first_failure.unwrap_or_else(|| {
        format!("iges_import: a set of {} trimmed surfaces built no solid with positive volume", body.len())
    }))
}

/// Whether `inner` lies inside `outer`, judged at one vertex.
fn inside(inner: &BrepSolid, outer: &BrepSolid) -> Result<bool, String> {
    let Some(vertex) = inner.vertices.first() else {
        return Ok(false);
    };
    let reading = crate::classify_point(vertex.point, outer, 1e-6)?;
    Ok(reading.class == crate::PointClass::In)
}

/// The 514 shells no 186 uses, in file order. An OCC-written file carries a
/// STEP SHELL_BASED_SURFACE_MODEL this way.
fn free_shells(file: &IgesFile) -> Result<Vec<i64>, String> {
    let mut used_shells: rustc_hash::FxHashSet<i64> = Default::default();
    for (_, solid) in file.pointers_of_type(186) {
        used_shells.insert(solid.int(0)?);
        for index in 0..solid.int(2)?.max(0) as usize {
            used_shells.insert(solid.int(3 + index * 2)?);
        }
    }
    Ok(file
        .pointers_of_type(514)
        .into_iter()
        .map(|(pointer, _)| pointer)
        .filter(|pointer| !used_shells.contains(pointer))
        .collect())
}

/// Refuse a file holding trimmed surfaces beside its solids. Importing only
/// the solids would drop them and look like a complete import.
fn refuse_uncovered_geometry(file: &IgesFile) -> Result<(), String> {
    let free_faces = [144, 143]
        .iter()
        .flat_map(|&kind| file.pointers_of_type(kind))
        .filter(|(_, entity)| entity.de.subordinate == 0)
        .count();
    if free_faces > 0 {
        return Err(format!(
            "iges_import: the file holds {free_faces} trimmed surface(s) outside its solids, which this import cannot build yet; importing the solids alone would leave them out"
        ));
    }
    Ok(())
}

/// Each free shell through the STEP lane: a closed shell of several faces as
/// the solid a 186 would make of it, refused by name if it fails as one; a
/// sheet (open, or a single untrimmed face) is not a solid and is counted in
/// `SHEETS_SKIPPED`, never dropped silently.
fn free_shell_bodies(
    file: &IgesFile,
    scale: f64,
    diagnostics: &mut crate::KernelDiagnostics,
) -> Result<Vec<Body>, String> {
    let mut bodies = Vec::new();
    for pointer in free_shells(file)? {
        let entity = file.entity(pointer)?;
        let whole_text = free_shell_to_step(file, pointer, scale, false)?;
        let report = match surface_model_report(&whole_text) {
            Ok(report) => report,
            Err(first_failure) => {
                let split_text = free_shell_to_step(file, pointer, scale, true)?;
                if split_text == whole_text {
                    return Err(format!("shell DE {pointer}: {first_failure}"));
                }
                surface_model_report(&split_text).map_err(|_| format!("shell DE {pointer}: {first_failure}"))?
            }
        };
        if report.solids.is_empty() && report.sheets_skipped == 0 {
            // The STEP lane skips a single-face surface model as reference
            // geometry without counting it; here it is counted with the sheets.
            diagnostics.count_n(crate::step_import::SHEETS_SKIPPED, 1);
        }
        merge_diagnostics(diagnostics, report.diagnostics);
        for solid in report.solids {
            bodies.push(Body {
                solid,
                name: structure::entity_name(file, entity),
                color: structure::entity_color(file, entity),
                level: entity.de.level,
            });
        }
    }
    Ok(bodies)
}

/// A surface model through the STEP lane: what it built, refusing a shell that
/// reached the solid builder and failed there.
fn surface_model_report(step: &str) -> Result<crate::StepImportReport, String> {
    let report = crate::import_step_report(step).map_err(|error| format!("iges_import: {error}"))?;
    if report.failed > 0 {
        return Err(format!(
            "iges_import: {}",
            report.first_error.as_deref().unwrap_or("the shell did not import")
        ));
    }
    for solid in &report.solids {
        collapsed_face_refusal(solid)?;
    }
    Ok(report)
}

/// One 186 through the STEP lane. A twice-walked edge on a quadric is told apart
/// by its position; elsewhere it is read as written first, because it is usually
/// a seam the builder handles, and as a bridge (`split_at_repeated_edges`) only
/// when that reading is refused. The first refusal is the one reported.
fn manifold_solid(file: &IgesFile, pointer: i64, scale: f64) -> Result<Built, String> {
    let whole_text = manifold_solid_to_step(file, pointer, scale, false)?;
    let first_failure = match solids_or_refusal(&whole_text) {
        Ok(built) => return Ok(built),
        Err(error) => error,
    };
    let split_text = manifold_solid_to_step(file, pointer, scale, true)?;
    if split_text == whole_text {
        return Err(format!("solid DE {pointer}: {first_failure}"));
    }
    solids_or_refusal(&split_text).map_err(|_| format!("solid DE {pointer}: {first_failure}"))
}

/// Import translated STEP, refusing when the body did not build. The STEP lane
/// skips a body that fails with text and returns the rest, but a file that
/// comes in short looks like a complete import.
/// Solids that built, with the diagnostics the STEP lane measured on them.
type Built = (Vec<BrepSolid>, crate::KernelDiagnostics);

fn solids_or_refusal(step: &str) -> Result<Built, String> {
    let report = crate::import_step_report(step).map_err(|error| format!("iges_import: {error}"))?;
    if report.failed > 0 {
        return Err(format!(
            "iges_import: {}",
            report.first_error.as_deref().unwrap_or("the body did not import")
        ));
    }
    if report.solids.is_empty() {
        return Err("iges_import: the solid produced no body".into());
    }
    for solid in &report.solids {
        collapsed_face_refusal(solid)?;
    }
    Ok((report.solids, report.diagnostics))
}

/// Area over boundary length squared below which a face has collapsed. Measured
/// on foreign files: collapsed faces read a few times 1e-6. A 1 mm by 100 mm
/// sliver reads 2.4e-3 and a 0.01 by 100 one 2.5e-5, so real faces clear it.
const COLLAPSED_FACE_RATIO: f64 = 1e-5;

/// A face with a real boundary that encloses almost nothing is a loop the
/// builder read the wrong way round the carrier. It validates, and the solid
/// then quietly lacks that face, so it is refused here.
fn collapsed_face_refusal(solid: &BrepSolid) -> Result<(), String> {
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let (Ok(area), Ok(length)) = (crate::face_area(face), crate::face_boundary_length(solid, face)) else {
            continue;
        };
        if length > 0.0 && area < COLLAPSED_FACE_RATIO * length * length {
            return Err(format!(
                "iges_import: a face collapsed on import, enclosing {area:.3e} mm2 inside a boundary {length:.3e} mm long"
            ));
        }
    }
    Ok(())
}
