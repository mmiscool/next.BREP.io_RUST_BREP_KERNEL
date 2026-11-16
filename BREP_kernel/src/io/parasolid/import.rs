//! Parasolid part to kernel `BrepSolid`s, by way of the STEP importer.
//!
//! Each body is translated to Part 21 ([`super::to_step`]) and imported
//! there, so it gets the STEP builder, closure gate and refusals. A body that
//! cannot be read correctly is refused by name; a file never imports short.

use crate::topology::BrepSolid;
use crate::{BodyAppearance, KernelDiagnostics};

use super::reader::{parse_xt, parse_xt_text, Node, XtFile};
use super::geometry::MM_PER_METRE;
use super::to_step::body_to_step;

/// What [`import_parasolid_report`] returns.
#[derive(Clone, Debug)]
pub struct ParasolidImportReport {
    /// The solids, in file order.
    pub solids: Vec<BrepSolid>,
    /// Each solid's name (the body's name attribute), parallel to `solids`;
    /// empty when the file gives none.
    pub names: Vec<String>,
    /// Each solid's colours, parallel to `solids`.
    pub appearances: Vec<BodyAppearance>,
    /// The writer (`APPL` in the header) and kernel version.
    pub application: Option<String>,
    pub modeller_version: String,
    /// The file's linear resolution in mm (the body's `res_linear`).
    pub stated_resolution_mm: Option<f64>,
    /// Edges whose 3D curve was fitted (intersection curves and SP-curves,
    /// which STEP has no entity for).
    pub fitted_curves: usize,
    /// Of those, the intersection curves fitted through their chart alone
    /// where no point between chart points could be solved (a surface with
    /// no distance the reader computes, such as a blend boundary, or a
    /// tangency): their fit is not checked against the true curve.
    pub unverified_curves: usize,
    /// Measurements and warnings from the STEP lane, merged over every body.
    pub diagnostics: KernelDiagnostics,
}

/// Read a Parasolid text part (`.x_t`, `.xmt_txt`) into solids.
pub fn import_parasolid(text: &str) -> Result<Vec<BrepSolid>, String> {
    Ok(import_parasolid_report(text)?.solids)
}

/// [`import_parasolid`] with names, colours and the import's diagnostics.
pub fn import_parasolid_report(text: &str) -> Result<ParasolidImportReport, String> {
    let file = parse_xt_text(text).map_err(|e| format!("parasolid_import: {e}"))?;
    import_file(&file)
}

/// [`import_parasolid_report`] from a file's bytes: text or binary (`.x_b`),
/// a part or a partition (the form SolidWorks embeds its bodies in).
pub fn import_parasolid_bytes_report(bytes: &[u8]) -> Result<ParasolidImportReport, String> {
    let file = parse_xt(bytes).map_err(|e| format!("parasolid_import: {e}"))?;
    import_file(&file)
}

fn import_file(file: &XtFile) -> Result<ParasolidImportReport, String> {
    let bodies = bodies(file).map_err(|e| format!("parasolid_import: {e}"))?;
    let mut report = ParasolidImportReport {
        solids: Vec::new(),
        names: Vec::new(),
        appearances: Vec::new(),
        application: file.header_value("APPL").map(str::to_string),
        modeller_version: file.modeller_version.trim_start_matches(':').trim().to_string(),
        stated_resolution_mm: bodies
            .first()
            .and_then(|b| b.body.real("res_linear"))
            .map(|r| r * MM_PER_METRE)
            .filter(|r| *r > 0.0),
        fitted_curves: 0,
        unverified_curves: 0,
        diagnostics: KernelDiagnostics::default(),
    };
    // Each distinct body is translated and imported once; every placement
    // of it, in file order, is a copy moved to its place.
    struct Imported {
        solids: Vec<BrepSolid>,
        appearances: Vec<Vec<Option<crate::ImportedColor>>>,
        name: Option<String>,
        color: Option<crate::ImportedColor>,
    }
    let mut imported: rustc_hash::FxHashMap<u32, Imported> = rustc_hash::FxHashMap::default();
    let distinct = {
        let mut seen = rustc_hash::FxHashSet::default();
        bodies.iter().filter(|p| seen.insert(p.body.index)).count()
    };
    let mut occurrences: rustc_hash::FxHashMap<u32, usize> = rustc_hash::FxHashMap::default();
    for p in &bodies {
        *occurrences.entry(p.body.index).or_default() += 1;
    }
    let mut ordinal = 0;
    let mut seen_count: rustc_hash::FxHashMap<u32, usize> = rustc_hash::FxHashMap::default();
    for Placed { body, at } in &bodies {
        let body = *body;
        if !imported.contains_key(&body.index) {
            ordinal += 1;
            let what = || match distinct {
                1 => "the body".to_string(),
                _ => format!("body {ordinal}"),
            };
            let translated = body_to_step(file, body, ordinal - 1).map_err(|e| format!("parasolid_import: {}: {e}", what()))?;
            // A sheet body (body type 3) is written as open shells, which come
            // in as sheet bodies; a solid's must close.
            let sheet = body.int("body_type") == Some(3);
            let step = if sheet {
                crate::step_import::import_step_report_with_sheets(&translated.step)
            } else {
                crate::import_step_report(&translated.step)
            }
            .map_err(|e| format!("parasolid_import: {}: {e}", what()))?;
            if step.failed > 0 {
                return Err(format!(
                    "parasolid_import: {}: {}",
                    what(),
                    step.first_error.as_deref().unwrap_or("the body did not import")
                ));
            }
            if step.sheets_skipped > 0 && !step.solids.is_empty() {
                // Part of the body did not come in: refuse it whole.
                return Err(format!(
                    "parasolid_import: {}: {} of its shells are open sheets, which do not bound a solid",
                    what(),
                    step.sheets_skipped
                ));
            }
            if step.solids.is_empty() {
                return Err(format!("parasolid_import: {}: the body produced no solid", what()));
            }
            report.fitted_curves += translated.fitted_curves;
            report.unverified_curves += translated.unverified_curves;
            merge_diagnostics(&mut report.diagnostics, step.diagnostics);
            let appearances = (0..step.solids.len()).map(|j| step.appearances.get(j).map(|a| a.faces.clone()).unwrap_or_default()).collect();
            imported.insert(body.index, Imported { solids: step.solids, appearances, name: translated.name, color: translated.color });
        }
        let entry = &imported[&body.index];
        let count = entry.solids.len() * occurrences[&body.index];
        let k = seen_count.entry(body.index).or_default();
        for (j, solid) in entry.solids.iter().enumerate() {
            let solid = match at {
                None => solid.clone(),
                Some(matrix) => {
                    let transform = crate::AffineTransform::new(*matrix)
                        .map_err(|e| format!("parasolid_import: placing an instance: {e}"))?;
                    crate::transform_brep(solid, transform, transform.determinant3() < 0.0)
                        .map_err(|e| format!("parasolid_import: placing an instance: {e}"))?
                }
            };
            report.appearances.push(BodyAppearance { body: entry.color, faces: entry.appearances[j].clone() });
            *k += 1;
            let name = match (&entry.name, count) {
                (Some(name), 1) => name.clone(),
                (Some(name), _) => format!("{name} ({k})"),
                (None, _) => String::new(),
            };
            report.names.push(name);
            report.solids.push(solid);
        }
    }
    Ok(report)
}

/// A body to import and where it goes: untransformed (`None`), or where an
/// assembly's instance places it (row-major 4×4, mm).
struct Placed<'f> {
    body: &'f Node,
    at: Option<crate::Mat4>,
}

/// The bodies a part file holds: its root BODY, or the bodies of a
/// PART_XMT_BLOCK / POINTER_LIS_BLOCK array of parts (§2.1); a partition's,
/// its WORLD's chain of bodies; an assembly's, every body its instances
/// place, through sub-assemblies (§4.3.1, §4.3.2).
fn bodies(file: &XtFile) -> Result<Vec<Placed<'_>>, String> {
    let root = file.root().ok_or("the file holds no nodes")?;
    let parts: Vec<&Node> = match root.type_name() {
        "BODY" | "ASSEMBLY" => vec![root],
        "WORLD" => {
            // Bodies a partition places through an assembly would come in
            // once, unplaced: refused until a partition with an assembly is
            // seen.
            if root.ptr("assembly") != 0 {
                return Err("the partition holds an assembly, which is not read yet".into());
            }
            let mut chain = Vec::new();
            let mut next = file.follow(root, "body");
            while let Some(body) = next {
                if chain.len() >= file.nodes().len() || chain.iter().any(|b: &&Node| b.index == body.index) {
                    return Err("the partition's chain of bodies loops".into());
                }
                chain.push(body);
                next = file.follow(body, "next");
            }
            chain
        }
        "PART_XMT_BLOCK" | "POINTER_LIS_BLOCK" => root
            .ints("entries")
            .into_iter()
            .filter_map(|i| u32::try_from(i).ok())
            .filter_map(|i| file.node(i))
            .collect(),
        other => return Err(format!("the file's root is a {other}, not a body")),
    };
    let mut out: Vec<Placed> = Vec::new();
    for part in parts {
        match part.type_name() {
            "BODY" => out.push(Placed { body: part, at: None }),
            "ASSEMBLY" => {
                let mut placed = Vec::new();
                place_instances(file, part, crate::step_matrix::MAT4_IDENTITY, 0, &mut placed)?;
                out.extend(placed.into_iter().map(|(body, matrix)| Placed { body, at: Some(matrix) }));
            }
            other => return Err(format!("the file holds a {other} where a part should be")),
        }
    }
    // A body listed on its own and also placed by an assembly in the file
    // would come in twice, once unplaced: no file seen does this.
    let unplaced: rustc_hash::FxHashSet<u32> = out.iter().filter(|p| p.at.is_none()).map(|p| p.body.index).collect();
    if out.iter().any(|p| p.at.is_some() && unplaced.contains(&p.body.index)) {
        return Err("the file lists a body both on its own and in an assembly, which is not read yet".into());
    }
    if out.is_empty() {
        return Err("the file holds no bodies".into());
    }
    Ok(out)
}

/// Every body `assembly`'s instances place, with its placement composed
/// down from `parent`. An instance's transform acts as x' = (R x + t) s
/// (§5.2.6), R row-major as written: a SolidWorks machining assembly's four
/// rotated blocks sit on its stock so, and off it transposed.
fn place_instances<'f>(
    file: &'f XtFile,
    assembly: &'f Node,
    parent: crate::Mat4,
    depth: usize,
    out: &mut Vec<(&'f Node, crate::Mat4)>,
) -> Result<(), String> {
    if depth > 64 {
        return Err("the assembly nests more than 64 deep".into());
    }
    let mut seen = 0usize;
    let mut next = file.follow(assembly, "sub_instance");
    while let Some(instance) = next {
        seen += 1;
        if seen > file.nodes().len() {
            return Err("an assembly's chain of instances loops".into());
        }
        if instance.int("type") == Some(2) {
            return Err("the assembly holds a negative instance, which is not read yet".into());
        }
        let local = match file.follow(instance, "transform") {
            None => crate::step_matrix::MAT4_IDENTITY,
            Some(transform) => transform_matrix(file, transform)?,
        };
        let matrix = crate::step_matrix::mat4_mul(&parent, &local);
        let part = file.follow(instance, "part").ok_or("an instance has no part")?;
        match part.type_name() {
            "BODY" => out.push((part, matrix)),
            "ASSEMBLY" => place_instances(file, part, matrix, depth + 1, out)?,
            other => return Err(format!("an instance places a {other}, not a body or an assembly")),
        }
        next = file.follow(instance, "next_in_part");
    }
    Ok(())
}

/// A TRANSFORM node as a row-major 4×4 in mm: x' = (R x + t) s, with the
/// optional precision node's corrections added (§5.2.6).
fn transform_matrix(file: &XtFile, node: &Node) -> Result<crate::Mat4, String> {
    let mut r = node.reals("rotation_matrix");
    let mut t = node.vector("translation_vector").unwrap_or([0.0; 3]);
    if r.len() != 9 || r.iter().any(|x| !x.is_finite()) {
        return Err("a transform has no rotation matrix".into());
    }
    if let Some(precision) = file.follow(node, "precision") {
        let dr = precision.reals("rotation_matrix");
        if dr.len() == 9 && dr.iter().all(|x| x.is_finite()) {
            for (v, d) in r.iter_mut().zip(&dr) {
                *v += d;
            }
        }
        if let Some(dt) = precision.vector("translation_vector") {
            for (v, d) in t.iter_mut().zip(dt) {
                *v += d;
            }
        }
    }
    if node.vector("perspective_vector").is_some_and(|p| p.iter().any(|x| *x != 0.0)) {
        return Err("an instance's transform has a perspective part, which is not read".into());
    }
    let s = node.real("scale").unwrap_or(1.0);
    if !(s.is_finite() && s != 0.0) {
        return Err(format!("a transform has scale {s}"));
    }
    // The kernel places a solid by a matrix whose determinant is above 1e-14
    // (its singularity bound, in mm³ per mm³).
    if s.abs().powi(3) <= 1e-14 {
        return Err(format!("an instance scales its part by {s}, below what the kernel can place (1e-14 in volume)"));
    }
    for v in &mut r {
        *v *= s;
    }
    for v in &mut t {
        *v *= s * MM_PER_METRE;
    }
    Ok([r[0], r[1], r[2], t[0], r[3], r[4], r[5], t[1], r[6], r[7], r[8], t[2], 0.0, 0.0, 0.0, 1.0])
}

fn merge_diagnostics(into: &mut KernelDiagnostics, from: KernelDiagnostics) {
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
