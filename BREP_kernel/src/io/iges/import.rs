//! IGES to kernel `BrepSolid`, by way of the STEP importer.
//!
//! Solids (186) and trimmed-surface sets (144, 143) are translated to Part 21
//! ([`super::to_step`]) so they get the STEP builder, closure gate and refusals.
//! Anything that cannot be read correctly is refused by name; a file never
//! imports short.

use crate::topology::BrepSolid;
use crate::Vec3;
use crate::{BodyAppearance, StepAssembly, StepOccurrence, StepProduct};
use rustc_hash::FxHashMap as HashMap;

use super::reader::{parse_iges, IgesFile};
use super::structure::{self, Matrix, Placed};
use super::to_step::{analyse_face_sets, free_shell_to_step, manifold_solid_to_step, trimmed_surfaces, FaceSets};

/// Parse an IGES document and reconstruct one or more `BrepSolid`s.
pub fn import_iges(text: &str) -> Result<Vec<BrepSolid>, String> {
    Ok(import_iges_report(text)?.solids)
}

/// [`import_iges_report`] for a caller that drops the report's diagnostics
/// (the IMPORT3D feature): under `BREP_IMPORT_LEAN` the STEP builds skip their
/// trim readings, which only read the trims already chosen (on a 4917-face
/// surface model they were most of the import). The bodies are the same.
pub fn import_iges_bodies(text: &str) -> Result<IgesImportReport, String> {
    let _lean = Lean::enter();
    import_iges_report(text)
}

std::thread_local! {
    /// Whether this thread's IGES import builds the trim readings: off inside
    /// [`import_iges_bodies`] and [`read_iges_assembly`] under `BREP_IMPORT_LEAN`.
    static TRIM_READINGS: std::cell::Cell<bool> = const { std::cell::Cell::new(true) };
}

/// Turns the trim readings off on this thread for its lifetime (when
/// `BREP_IMPORT_LEAN` is on), restoring what was there.
struct Lean(bool);

impl Lean {
    fn enter() -> Self {
        let before = TRIM_READINGS.with(|readings| readings.get());
        if crate::step_import::import_lean() {
            TRIM_READINGS.with(|readings| readings.set(false));
        }
        Lean(before)
    }
}

impl Drop for Lean {
    fn drop(&mut self) {
        TRIM_READINGS.with(|readings| readings.set(self.0));
    }
}

/// One translated Part 21 document through the STEP lane, with sheets built
/// as sheet bodies when `sheets`, and the trim readings when this thread's
/// import wants them.
fn step_report(step: &str, sheets: bool) -> Result<crate::StepImportReport, String> {
    let readings = TRIM_READINGS.with(|readings| readings.get());
    if readings {
        if sheets {
            crate::step_import::import_step_report_with_sheets(step)
        } else {
            crate::import_step_report(step)
        }
    } else {
        crate::step_import::import_step_report_dropping_readings(step, sheets)
    }
    .map_err(|error| format!("iges_import: {error}"))
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
        report.appearances.push(BodyAppearance { body: body.color, faces: body.faces });
        report.levels.push(body.level);
    }
    Ok(report)
}

/// One imported solid and what the file said about it.
struct Body {
    solid: BrepSolid,
    name: Option<String>,
    color: Option<crate::ImportedColor>,
    /// Each face's own colour in shell/face order, or empty when no face of
    /// the body has one.
    faces: FaceColors,
    level: i64,
}

impl Body {
    fn bare(solid: BrepSolid) -> Self {
        Self { solid, name: None, color: None, faces: Vec::new(), level: 0 }
    }
}

/// A body's per-face colours, as [`BodyAppearance::faces`] holds them.
type FaceColors = Vec<Option<crate::ImportedColor>>;

/// Each 186 built once, through the STEP lane, in its definition frame, with
/// the colours its faces carry.
type SolidCache = HashMap<i64, Vec<(BrepSolid, FaceColors)>>;

/// Each subfigure's (308) trimmed-surface sets, sewn once in its own frame,
/// keyed by the 308.
type SetCache = HashMap<i64, Vec<BrepSolid>>;

fn built_solid<'c>(
    cache: &'c mut SolidCache,
    file: &IgesFile,
    pointer: i64,
    scale: f64,
    diagnostics: &mut crate::KernelDiagnostics,
) -> Result<&'c [(BrepSolid, FaceColors)], String> {
    if !cache.contains_key(&pointer) {
        let (built, measured) = manifold_solid(file, pointer, scale)?;
        merge_diagnostics(diagnostics, measured);
        cache.insert(pointer, built);
    }
    Ok(&cache[&pointer])
}

/// A built solid moved by a placement; a mirroring placement turns it inside
/// out unless the orientation is reversed with it.
pub(super) fn place(solid: &BrepSolid, placement: &Matrix) -> Result<BrepSolid, String> {
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
            for (built, faces) in built_solid(&mut cache, file, solid, scale, diagnostics)? {
                bodies.push(Body { solid: place(built, &placement)?, name: name.clone(), color, faces: faces.clone(), level });
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
    // A file of trimmed surfaces whose subfigures are placed: the file's own
    // sets, then each instance's sets where it puts them.
    if file.entities_of_type(408).next().is_some() {
        let bodies = placed_face_set_bodies(file, scale, diagnostics)?;
        if bodies.is_empty() {
            return Err(nothing_to_import(file));
        }
        return Ok(bodies);
    }

    if file.entities_of_type(144).next().is_none() && file.entities_of_type(143).next().is_none() {
        return Err(nothing_to_import(file));
    }
    let (solids, measured) = face_set_solids(file, scale, &trimmed_surfaces(file))?;
    for each in measured {
        merge_diagnostics(diagnostics, each);
    }
    Ok(solids.into_iter().map(Body::bare).collect())
}

/// Why a file without solids or sewable surfaces imports nothing.
fn nothing_to_import(file: &IgesFile) -> String {
    if file.entities_of_type(144).next().is_none()
        && file.entities_of_type(143).next().is_none()
        && file.entities_of_type(128).next().is_some()
    {
        return "iges_import: file has untrimmed B-spline surfaces (entity 128) but no trimmed surfaces (entity 144) to form faces"
            .into();
    }
    "iges_import: the file has no solids (186) or trimmed surfaces (144, 143) to import".into()
}

/// The trimmed surfaces no subfigure (308) lists: the file's own, in
/// directory order.
fn loose_surfaces(file: &IgesFile) -> Result<Vec<i64>, String> {
    let members = structure::subfigure_member_set(file)?;
    Ok(trimmed_surfaces(file).into_iter().filter(|pointer| !members.contains(pointer)).collect())
}

/// The file's own surface sets sewn into solids, refused as the flat import
/// of a file without subfigures refuses them.
fn loose_set_solids(
    file: &IgesFile,
    scale: f64,
    diagnostics: &mut crate::KernelDiagnostics,
) -> Result<Vec<BrepSolid>, String> {
    let loose = loose_surfaces(file)?;
    if loose.is_empty() {
        return Ok(Vec::new());
    }
    let (solids, measured) = face_set_solids(file, scale, &loose)?;
    for each in measured {
        merge_diagnostics(diagnostics, each);
    }
    Ok(solids)
}

/// The solids a subfigure's (308) own trimmed surfaces sew into, in its own
/// frame, built once however many instances place it. A set that does not
/// sew refuses the file, naming the subfigure.
fn subfigure_set_solids<'c>(
    cache: &'c mut SetCache,
    file: &IgesFile,
    pointer: i64,
    scale: f64,
    diagnostics: &mut crate::KernelDiagnostics,
) -> Result<&'c [BrepSolid], String> {
    if !cache.contains_key(&pointer) {
        let mut surfaces = Vec::new();
        for member in structure::subfigure_members(file.entity(pointer)?)? {
            if matches!(file.entity(member)?.de.entity_type, 144 | 143) {
                surfaces.push(member);
            }
        }
        surfaces.sort_unstable();
        surfaces.dedup();
        let mut solids = Vec::new();
        if !surfaces.is_empty() {
            let (built, measured) =
                face_set_solids(file, scale, &surfaces).map_err(|error| format!("subfigure DE {pointer}: {error}"))?;
            for each in measured {
                merge_diagnostics(diagnostics, each);
            }
            solids = built;
        }
        cache.insert(pointer, solids);
    }
    Ok(&cache[&pointer])
}

/// A file of trimmed surfaces with subfigure instances, flattened: the
/// file's own sets, then each instance's sets placed, in file order.
fn placed_face_set_bodies(
    file: &IgesFile,
    scale: f64,
    diagnostics: &mut crate::KernelDiagnostics,
) -> Result<Vec<Body>, String> {
    // The structure's refusals come before any sewing.
    let placed = structure::placed_face_sets(file, scale)?;
    let mut bodies: Vec<Body> = loose_set_solids(file, scale, diagnostics)?.into_iter().map(Body::bare).collect();
    let mut cache = SetCache::default();
    for Placed { solid: definition, placement, name, color, level } in placed {
        for built in subfigure_set_solids(&mut cache, file, definition, scale, diagnostics)? {
            bodies.push(Body { solid: place(built, &placement)?, name: name.clone(), color, faces: Vec::new(), level });
        }
    }
    Ok(bodies)
}

/// The file's subfigure structure, the IGES counterpart of
/// [`crate::read_step_assembly`]: one product for the file itself holding the
/// solids that stand on their own, one per instanced subfigure (308) holding
/// its own solids in its own frame, and one occurrence per instance (408) with
/// its placement in millimetres. `Ok(None)` when the file places no
/// subfigures, so the caller takes the flat import as for STEP.
///
/// A file of trimmed surfaces (144, 143) without solids reads the same way:
/// each instanced subfigure's product holds the solids its own surfaces sew
/// into, and the root product the file's own sets.
///
/// Every solid is built once, as in the flat import, and a file the flat
/// import refuses is refused here with the same words: an assembly never
/// comes in short.
pub fn read_iges_assembly(text: &str) -> Result<Option<StepAssembly>, String> {
    // The structure reads no trim readings: the build skips them as for
    // [`import_iges_bodies`].
    let _lean = Lean::enter();
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
    if file.entities_of_type(186).next().is_none() && file.entities_of_type(514).next().is_none() {
        return face_set_assembly(&file, scale, top_instances).map(Some);
    }
    if file.entities_of_type(186).next().is_none() {
        return Err(
            "iges_import: the file places subfigures (408) of trimmed surfaces, which this import cannot build yet"
                .into(),
        );
    }
    refuse_uncovered_geometry(&file)?;
    structure::placed_solids(&file, scale)?;

    let mut cache = SolidCache::default();
    let mut sets = SetCache::default();
    let mut diagnostics = crate::KernelDiagnostics::default();
    let mut assembly = root_only_assembly(&file);
    for (pointer, solid) in file.pointers_of_type(186) {
        if !members.contains(&pointer) {
            let color = structure::entity_color(&file, solid);
            for (built, faces) in built_solid(&mut cache, &file, pointer, scale, &mut diagnostics)? {
                assembly.products[0].bodies.push(built.clone());
                assembly.products[0].appearances.push(BodyAppearance { body: color, faces: faces.clone() });
            }
        }
    }
    for body in free_shell_bodies(&file, scale, &mut diagnostics)? {
        assembly.products[0].bodies.push(body.solid);
        assembly.products[0].appearances.push(BodyAppearance { body: body.color, faces: body.faces });
    }
    assembly.sheets_skipped = diagnostics.counters.get(crate::step_import::SHEETS_SKIPPED).copied().unwrap_or(0) as usize;
    let mut products: HashMap<i64, usize> = HashMap::default();
    for pointer in top_instances {
        add_instance(&file, scale, pointer, 0, &mut assembly, &mut products, (&mut cache, &mut sets), &mut diagnostics, 0)?;
    }
    Ok(Some(assembly))
}

/// The assembly with only its root product, the file itself, and nothing in it.
fn root_only_assembly(file: &IgesFile) -> StepAssembly {
    let root_name = {
        let product = file.global.product_id.trim();
        if product.is_empty() { "IGES".to_string() } else { product.to_string() }
    };
    StepAssembly {
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
    }
}

/// [`read_iges_assembly`] for a file of trimmed surfaces: every set the flat
/// import would sew is sewn first, in the flat import's order, so a file it
/// refuses is refused here with the same words before any structure.
fn face_set_assembly(file: &IgesFile, scale: f64, top_instances: Vec<i64>) -> Result<StepAssembly, String> {
    let mut diagnostics = crate::KernelDiagnostics::default();
    let placed = structure::placed_face_sets(file, scale)?;
    let loose = loose_set_solids(file, scale, &mut diagnostics)?;
    let mut sets = SetCache::default();
    let mut any = !loose.is_empty();
    for each in &placed {
        // Each instance placed as the flat import places it, so a placement it
        // cannot apply (singular, or one taking the geometry out of range) is
        // refused here with its words, not carried as an occurrence that
        // cannot be built.
        for solid in subfigure_set_solids(&mut sets, file, each.solid, scale, &mut diagnostics)? {
            place(solid, &each.placement)?;
            any = true;
        }
    }
    if !any {
        return Err(nothing_to_import(file));
    }
    let mut assembly = root_only_assembly(file);
    for solid in loose {
        assembly.products[0].bodies.push(solid);
        assembly.products[0].appearances.push(BodyAppearance { body: None, faces: Vec::new() });
    }
    let mut cache = SolidCache::default();
    let mut products: HashMap<i64, usize> = HashMap::default();
    for pointer in top_instances {
        add_instance(file, scale, pointer, 0, &mut assembly, &mut products, (&mut cache, &mut sets), &mut diagnostics, 0)?;
    }
    Ok(assembly)
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
    (cache, sets): (&mut SolidCache, &mut SetCache),
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
            // The subfigure's own trimmed surfaces, sewn in its frame.
            let color = structure::entity_color(file, definition);
            for solid in subfigure_set_solids(sets, file, definition_pointer, scale, diagnostics)?.to_vec() {
                assembly.products[index].bodies.push(solid);
                assembly.products[index].appearances.push(BodyAppearance { body: color, faces: Vec::new() });
            }
            for member in structure::subfigure_members(definition)? {
                let entity = file.entity(member)?;
                match entity.de.entity_type {
                    186 => {
                        let color = structure::entity_color(file, entity);
                        let built = built_solid(cache, file, member, scale, diagnostics)?.to_vec();
                        let product = &mut assembly.products[index];
                        for (solid, faces) in built {
                            product.bodies.push(solid);
                            product.appearances.push(BodyAppearance { body: color, faces });
                        }
                    }
                    408 => add_instance(
                        file,
                        scale,
                        member,
                        index,
                        assembly,
                        products,
                        (&mut *cache, &mut *sets),
                        diagnostics,
                        depth + 1,
                    )?,
                    _ => {}
                }
            }
            index
        }
    };
    let placement = structure::instance_matrix(file, instance, scale)?;
    let rigid = crate::AffineTransform::new(placement).is_ok_and(|transform| transform.is_rigid(1e-9));
    // An instance's colour in IGES is INHERITED: a body's or an inner
    // instance's own colour wins over it, as the flat import reads it. As a
    // whole-instance colour (which colours everything the instance places) it
    // is given only where nothing beneath is coloured on its own; otherwise it
    // goes to the child's own uncoloured bodies alone.
    let (color, body_colors) = match structure::entity_color(file, instance) {
        None => (None, Vec::new()),
        Some(own) if !coloured_below(file, definition_pointer, 0)? => (Some(own), Vec::new()),
        Some(own) => {
            let bodies: Vec<Option<crate::ImportedColor>> = assembly.products[child]
                .appearances
                .iter()
                .map(|appearance| appearance.body.is_none().then_some(own))
                .collect();
            let any = bodies.iter().any(Option::is_some);
            (None, if any { bodies } else { Vec::new() })
        }
    };
    assembly.occurrences.push(StepOccurrence {
        edge_ref: pointer as usize,
        parent,
        child,
        designator: structure::entity_name(file, instance).unwrap_or_else(|| format!("instance#{pointer}")),
        placement,
        rigid,
        color,
        body_colors,
    });
    Ok(())
}

/// Whether anything a subfigure places carries a colour of its own: the
/// subfigure itself (its surface sets take it), a member solid, or a member
/// instance, at any depth.
fn coloured_below(file: &IgesFile, definition_pointer: i64, depth: usize) -> Result<bool, String> {
    if depth > 64 {
        return Err(format!("iges_import: subfigure DE {definition_pointer} nests deeper than 64 levels"));
    }
    let definition = file.entity(definition_pointer)?;
    if structure::entity_color(file, definition).is_some() {
        return Ok(true);
    }
    for member in structure::subfigure_members(definition)? {
        let entity = file.entity(member)?;
        match entity.de.entity_type {
            186 if structure::entity_color(file, entity).is_some() => return Ok(true),
            408 => {
                if structure::entity_color(file, entity).is_some() {
                    return Ok(true);
                }
                let (inner, _) = structure::instanced_subfigure(file, member)?;
                if coloured_below(file, inner, depth + 1)? {
                    return Ok(true);
                }
            }
            _ => {}
        }
    }
    Ok(false)
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

/// Build trimmed surfaces (`surfaces`, in directory order) into solids: one
/// solid per connected, closed set of faces, and a set lying inside another
/// becomes its cavity.
fn face_set_solids(
    file: &IgesFile,
    scale: f64,
    surfaces: &[i64],
) -> Result<(Vec<BrepSolid>, Vec<crate::KernelDiagnostics>), String> {
    let sets = analyse_face_sets(file, scale, surfaces)?;
    // A set that does not close bounds no volume: it comes in as the sheet it
    // is, every face of it, not refused and not dropped. `BREP_IGES_OPEN_SHEETS`
    // (default on; `=0` refuses the file, naming the open set).
    let as_sheets = crate::switch!("BREP_IGES_OPEN_SHEETS", true);
    let mut sheets = Vec::new();
    let mut sheet_readings = Vec::new();
    let mut closed = Vec::with_capacity(sets.bodies.len());
    for body in &sets.bodies {
        match sets.open_reason(body) {
            None => closed.push(body.as_slice()),
            Some(reason) if !as_sheets => return Err(reason),
            Some(reason) => {
                let (sheet, readings) = face_set_sheet(&sets, file, scale, body).map_err(|error| format!("{reason}; as a sheet: {error}"))?;
                sheets.push(sheet);
                sheet_readings.push(readings);
            }
        }
    }
    let mut built = Vec::with_capacity(closed.len());
    for body in &closed {
        built.push(face_set_body(&sets, file, scale, body, &[])?);
    }
    // Every body that lies inside another, whatever their sizes: no order of
    // boxes or volumes is sure to put a container before what it holds. A
    // body inside an odd number of others is a cavity of the one it is
    // directly in (inside one fewer); inside an even number (none, or an
    // island in a cavity) it is a solid of its own.
    let reach: Vec<_> = built.iter().map(|body| brep_topology::recentre::solid_bounds(&body.solid)).collect();
    let corners: Vec<_> = built.iter().map(|body| vertex_box(&body.solid)).collect();
    let mut containers: Vec<Vec<usize>> = vec![Vec::new(); built.len()];
    for inner in 0..built.len() {
        for outer in 0..built.len() {
            let fits = |axis: fn(Vec3) -> f64| {
                axis(corners[inner].0) >= axis(reach[outer].minimum) && axis(corners[inner].1) <= axis(reach[outer].maximum)
            };
            if inner != outer
                && fits(|p| p.x)
                && fits(|p| p.y)
                && fits(|p| p.z)
                && inside(&built[inner].solid, &built[outer].solid)?
            {
                containers[inner].push(outer);
            }
        }
    }
    let mut cavities: Vec<Vec<usize>> = vec![Vec::new(); built.len()];
    let mut is_cavity = vec![false; built.len()];
    for inner in 0..built.len() {
        let depth = containers[inner].len();
        if depth % 2 == 1 {
            if let Some(&parent) = containers[inner].iter().find(|&&outer| containers[outer].len() == depth - 1) {
                cavities[parent].push(inner);
                is_cavity[inner] = true;
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
        let voids: Vec<(&[usize], bool)> = cavities[index].iter().map(|&cavity| (closed[cavity], !built[cavity].flip)).collect();
        let with_cavities = face_set_body(&sets, file, scale, closed[index], &voids)?;
        solids.push(with_cavities.solid);
        measured.push(with_cavities.diagnostics);
    }
    solids.extend(sheets);
    measured.extend(sheet_readings);
    Ok((solids, measured))
}

/// One face set that does not close, through the STEP lane as a sheet body
/// (an open shell of a surface model); a twice-walked edge is read as written
/// first, then as a bridge.
fn face_set_sheet(
    sets: &FaceSets,
    file: &IgesFile,
    scale: f64,
    body: &[usize],
) -> Result<(BrepSolid, crate::KernelDiagnostics), String> {
    let mut first_failure = None;
    for split in [false, true] {
        let text = sets.to_step_sheet(file, scale, body, split)?;
        let built = step_report(&text, true).and_then(|report| {
                if report.failed > 0 || report.solids.len() != 1 {
                    return Err(format!(
                        "iges_import: {}",
                        report.first_error.as_deref().unwrap_or("the sheet did not build as one body")
                    ));
                }
                Ok(report)
            });
        match built {
            Ok(mut report) => {
                let sheet = report.solids.remove(0);
                collapsed_face_refusal(&sheet)?;
                return Ok((sheet, report.diagnostics));
            }
            Err(error) => {
                first_failure.get_or_insert(error);
            }
        }
    }
    Err(first_failure.unwrap_or_default())
}

struct BuiltBody {
    solid: BrepSolid,
    flip: bool,
    diagnostics: crate::KernelDiagnostics,
}

/// One closed face set through the STEP lane as one solid (the builder turns
/// it outward); a twice-walked edge is read as written first, then as a
/// bridge.
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
                    // A set of trimmed surfaces carries no face colours here.
                    // The STEP builder has already turned the solid outward by
                    // its volume's sign; only that sign is read again here (the
                    // exact integral was a fifth of a 4236-face import), so a
                    // shell enclosing nothing is still refused.
                    let (solid, _) = solids.remove(0);
                    let sign = crate::solid_signed_volume_for_sign(&solid, 0.0).or_else(|_| crate::solid_signed_volume(&solid))?;
                    if sign > 0.0 {
                        return Ok(BuiltBody { solid, flip, diagnostics });
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

/// The corners of the box round a solid's vertices. A body inside another has
/// its vertices inside the other's surfaces' control nets, which bound it.
fn vertex_box(solid: &BrepSolid) -> (Vec3, Vec3) {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in &solid.vertices {
        let p = vertex.point;
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    (low, high)
}

/// Whether `inner` lies inside `outer` (a cavity of it), judged at up to 16
/// of its vertices spread through its list: every one must be in. A body one
/// vertex of which is in and another out crosses or touches the outer one's
/// boundary: two bodies, not a solid and its void.
fn inside(inner: &BrepSolid, outer: &BrepSolid) -> Result<bool, String> {
    const READINGS: usize = 16;
    let count = inner.vertices.len();
    if count == 0 {
        return Ok(false);
    }
    let step = count.div_ceil(READINGS).max(1);
    for vertex in inner.vertices.iter().step_by(step) {
        if crate::classify_point(vertex.point, outer, 1e-6)?.class != crate::PointClass::In {
            return Ok(false);
        }
    }
    Ok(true)
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
        let faces = face_colors(&report);
        merge_diagnostics(diagnostics, report.diagnostics);
        for (solid, faces) in report.solids.into_iter().zip(faces) {
            bodies.push(Body {
                solid,
                name: structure::entity_name(file, entity),
                color: structure::entity_color(file, entity),
                faces,
                level: entity.de.level,
            });
        }
    }
    Ok(bodies)
}

/// A surface model through the STEP lane: what it built, refusing a shell that
/// reached the solid builder and failed there.
fn surface_model_report(step: &str) -> Result<crate::StepImportReport, String> {
    let report = step_report(step, false)?;
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
/// Solids that built, each with its faces' colours, and the diagnostics the
/// STEP lane measured on them.
type Built = (Vec<(BrepSolid, FaceColors)>, crate::KernelDiagnostics);

/// Each built solid's per-face colours as the STEP lane paired them: one list
/// per solid, as long as its faces where any face has a colour, else empty.
/// A list of another length is not a pairing and is dropped.
fn face_colors(report: &crate::StepImportReport) -> Vec<FaceColors> {
    report
        .solids
        .iter()
        .enumerate()
        .map(|(index, solid)| {
            let faces: usize = solid.shells.iter().map(|shell| shell.faces.len()).sum();
            match report.appearances.get(index) {
                Some(appearance) if appearance.faces.len() == faces && appearance.faces.iter().any(Option::is_some) => {
                    appearance.faces.clone()
                }
                _ => Vec::new(),
            }
        })
        .collect()
}

fn solids_or_refusal(step: &str) -> Result<Built, String> {
    let report = step_report(step, false)?;
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
    let faces = face_colors(&report);
    Ok((report.solids.into_iter().zip(faces).collect(), report.diagnostics))
}

/// Area over boundary length squared below which a face has collapsed. Measured
/// on foreign files: collapsed faces read a few times 1e-6. A 1 mm by 100 mm
/// sliver reads 2.4e-3 and a 0.01 by 100 one 2.5e-5, so real faces clear it.
const COLLAPSED_FACE_RATIO: f64 = 1e-5;

/// A face with a real boundary that encloses almost nothing is a loop the
/// builder read the wrong way round the carrier. It validates, and the solid
/// then quietly lacks that face, so it is refused here.
///
/// `BREP_IGES_AREA_SCREEN` (default on; `=0` integrates every face): a face's
/// area is at least the length of its vector area, which its boundary alone
/// gives (`boundary_vector_area`). A face whose boundary clears the ratio by a
/// wide margin is not integrated; the exact area is read only for the rest.
fn collapsed_face_refusal(solid: &BrepSolid) -> Result<(), String> {
    let screen = crate::switch!("BREP_IGES_AREA_SCREEN", true);
    let mut readings = [0usize; 3]; // screened, unread by the screen, short of its margin
    let outcome = collapsed_face_scan(solid, screen, &mut readings);
    if crate::profile::profile_enabled() {
        profile_line!("iges.area_screen screened={} unread={} short={}", readings[0], readings[1], readings[2]);
    }
    outcome
}

fn collapsed_face_scan(solid: &BrepSolid, screen: bool, readings: &mut [usize; 3]) -> Result<(), String> {
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let Ok(length) = crate::face_boundary_length(solid, face) else {
            continue;
        };
        if screen {
            match boundary_vector_area(face) {
                Some((vector_area, error)) if vector_area - error > AREA_SCREEN_MARGIN * COLLAPSED_FACE_RATIO * length * length => {
                    readings[0] += 1;
                    continue;
                }
                Some(_) => readings[2] += 1,
                None => readings[1] += 1,
            }
        }
        let Ok(area) = crate::face_area(face) else {
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

/// How far a face's vector area must clear the collapse ratio before the
/// screen skips the exact area: a face that encloses little reads well under
/// it, and the chord error is held separately.
const AREA_SCREEN_MARGIN: f64 = 10.0;

/// Chords per coedge of the boundary polygon, shared out over its knot spans
/// (`pcurve_stations`).
const AREA_SCREEN_CHORDS: usize = 16;

/// The parameters a pcurve is read at: its start, then for each chord its
/// middle and its end. Chords never straddle a knot, and every knot span has
/// at least two, so a span holding all of a curve's travel is read as closely
/// as a curve's span that is its whole domain; otherwise the
/// `AREA_SCREEN_CHORDS` chords are shared out by span length.
fn pcurve_stations(pcurve: &crate::NurbsCurve) -> Option<Vec<f64>> {
    let [start, end] = pcurve.domain().ok()?;
    if !(start.is_finite() && end.is_finite() && end > start) {
        return None;
    }
    let mut breaks = vec![start];
    for &knot in &pcurve.knots {
        if knot > breaks[breaks.len() - 1] && knot < end {
            breaks.push(knot);
        }
    }
    breaks.push(end);
    let mut stations = vec![start];
    for span in breaks.windows(2) {
        let share = (AREA_SCREEN_CHORDS as f64 * (span[1] - span[0]) / (end - start)).ceil() as usize;
        let chords = share.max(2);
        for chord in 0..chords {
            let at = |fraction: f64| span[0] + (span[1] - span[0]) * fraction;
            stations.push(at((chord as f64 + 0.5) / chords as f64));
            stations.push(if chord + 1 == chords { span[1] } else { at((chord + 1) as f64 / chords as f64) });
        }
    }
    Some(stations)
}

/// How closely the stations' parameter area must agree with the pcurves'
/// quadrature: a chord polygon of a smooth loop reads within a fraction of a
/// percent of it, and one that misses a pcurve's travel reads far off.
const AREA_SCREEN_AGREEMENT: f64 = 1e-2;

/// Fraction of a loop's parameter extent within which two pcurve ends are one
/// joint, closed by a chord: fitted pcurves miss their neighbours by up to a
/// few times 1e-6 of it, and a step across a period reads about one.
const AREA_SCREEN_JOINT: f64 = 1e-3;

/// The length of a face's vector area, `|½ ∮ r × dr|` round the images of its
/// loops' pcurves on its carrier, and an estimate of what the chords miss: the
/// sliver between each chord and its arc, read as the chord times twice the
/// sag at its middle (about three times a parabola's sliver; a span that turns
/// within one chord reads less, which the caller's tenfold margin carries).
///
/// By Stokes the vector area of any surface spanning those loops is the same,
/// and no surface's area is less than the length of its vector area. That
/// holds for the region the area integral reads only when the loops bound it
/// in the parameter plane, so `None` (the caller integrates) unless every
/// loop closes there at every joint (none steps across a period; a miss within
/// `AREA_SCREEN_JOINT` is closed by a chord, in the plane and on the carrier)
/// and the loops together wind positively there: a loop laid the
/// wrong way round reads a negative area, which is the collapse the caller
/// refuses. And `None` unless the stations read every pcurve whole: each is
/// polynomial (a rational one can bunch its travel between any stations by its
/// weights alone), and the stations' parameter area agrees with the pcurves'
/// quadrature. `None` too when a pcurve or the carrier cannot be read.
pub(super) fn boundary_vector_area(face: &crate::topology::FaceRecord) -> Option<(f64, f64)> {
    let image = |uv: Vec3| -> Option<Vec3> {
        let point = face.surface.evaluate(uv.x, uv.y).ok()?;
        (point.x.is_finite() && point.y.is_finite() && point.z.is_finite()).then_some(point)
    };
    // A rational pcurve can bunch its travel between any stations by its
    // weights alone; only polynomial ones are read.
    let polynomial = face.loops.iter().flat_map(|loop_record| &loop_record.coedges).all(|coedge| {
        let first = coedge.pcurve.control_points.first().map_or(1.0, |point| point.w);
        first.is_finite() && first != 0.0 && coedge.pcurve.control_points.iter().all(|point| (point.w - first).abs() <= 1e-12 * first.abs())
    });
    if !polynomial {
        return None;
    }
    let mut origin = None;
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    let mut error = 0.0;
    let mut winding = 0.0;
    // The stations' own parameter area, joints left open, against the
    // quadrature of the same pcurves (exact for polynomial spans): stations
    // that miss part of a pcurve's travel read a different area.
    let mut sampled = 0.0;
    for loop_record in &face.loops {
        // The loop's parameter polygon, coedge by coedge.
        let mut polygon: Vec<Vec<Vec3>> = Vec::with_capacity(loop_record.coedges.len());
        for coedge in &loop_record.coedges {
            let stations = pcurve_stations(&coedge.pcurve)?
                .into_iter()
                .map(|t| coedge.pcurve.evaluate(t).ok())
                .collect::<Option<Vec<_>>>()?;
            polygon.push(stations);
        }
        let (mut low, mut high) = (Vec3::new(f64::INFINITY, f64::INFINITY, 0.0), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, 0.0));
        for uv in polygon.iter().flatten() {
            low = Vec3::new(low.x.min(uv.x), low.y.min(uv.y), 0.0);
            high = Vec3::new(high.x.max(uv.x), high.y.max(uv.y), 0.0);
        }
        let joint = AREA_SCREEN_JOINT * high.sub(low).length();
        for (index, stations) in polygon.iter().enumerate() {
            let next = &polygon[(index + 1) % polygon.len()];
            let end = stations[stations.len() - 1];
            if !(end.sub(next[0]).length() <= joint) {
                return None;
            }
            for pair in stations.windows(2) {
                sampled += 0.5 * (pair[0].x * pair[1].y - pair[0].y * pair[1].x);
            }
            winding += 0.5 * (end.x * next[0].y - end.y * next[0].x);
        }
        // The image polygon, with a chord across every joint and one closing
        // the loop, so its vector area does not depend on the origin.
        let mut last: Option<Vec3> = None;
        let mut first = None;
        for stations in &polygon {
            let mut previous = image(stations[0])?;
            let origin = *origin.get_or_insert(previous);
            first.get_or_insert(previous);
            if let Some(end) = last {
                sum = sum.add(end.sub(origin).cross(previous.sub(origin)));
            }
            for chord in stations[1..].chunks(2) {
                let (arc, next) = (image(chord[0])?, image(chord[1])?);
                let sag = arc.sub(previous.add(next).scale(0.5)).length();
                error += 2.0 * next.sub(previous).length() * sag;
                sum = sum.add(previous.sub(origin).cross(next.sub(origin)));
                previous = next;
            }
            last = Some(previous);
        }
        if let (Some(start), Some(end), Some(origin)) = (first, last, origin) {
            sum = sum.add(end.sub(origin).cross(start.sub(origin)));
        }
    }
    let exact = crate::parameter_space_area(face).ok()?;
    if !((sampled - exact).abs() <= AREA_SCREEN_AGREEMENT * exact.abs()) {
        return None;
    }
    winding += sampled;
    let length = 0.5 * sum.length();
    (winding > 0.0 && length.is_finite() && error.is_finite()).then_some((length, error))
}
