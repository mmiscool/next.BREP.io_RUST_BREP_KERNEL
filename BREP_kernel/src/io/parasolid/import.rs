//! Parasolid part to kernel `BrepSolid`s, by way of the STEP importer.
//!
//! Each body is translated to Part 21 ([`super::to_step`]) and imported
//! there, so it gets the STEP builder, closure gate and refusals. A body that
//! cannot be read correctly is refused by name; a file never imports short.

use crate::topology::BrepSolid;
use crate::{BodyAppearance, KernelDiagnostics};

use super::reader::{parse_xt_text, Node, XtFile};
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
    let bodies = bodies(&file).map_err(|e| format!("parasolid_import: {e}"))?;
    let mut report = ParasolidImportReport {
        solids: Vec::new(),
        names: Vec::new(),
        appearances: Vec::new(),
        application: file.header_value("APPL").map(str::to_string),
        modeller_version: file.modeller_version.trim_start_matches(':').trim().to_string(),
        stated_resolution_mm: bodies
            .first()
            .and_then(|b| b.real("res_linear"))
            .map(|r| r * MM_PER_METRE)
            .filter(|r| *r > 0.0),
        fitted_curves: 0,
        unverified_curves: 0,
        diagnostics: KernelDiagnostics::default(),
    };
    for (index, body) in bodies.iter().enumerate() {
        let what = || match bodies.len() {
            1 => "the body".to_string(),
            _ => format!("body {}", index + 1),
        };
        let translated = body_to_step(&file, body, index).map_err(|e| format!("parasolid_import: {}: {e}", what()))?;
        let step = crate::import_step_report(&translated.step).map_err(|e| format!("parasolid_import: {}: {e}", what()))?;
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
            return Err(format!(
                "parasolid_import: {}: {}",
                what(),
                if body.int("body_type") == Some(3) {
                    "the body is an open sheet, which does not bound a solid"
                } else {
                    "the body produced no solid"
                }
            ));
        }
        report.fitted_curves += translated.fitted_curves;
        report.unverified_curves += translated.unverified_curves;
        merge_diagnostics(&mut report.diagnostics, step.diagnostics);
        let count = step.solids.len();
        for (k, solid) in step.solids.into_iter().enumerate() {
            let faces = step.appearances.get(k).map(|a| a.faces.clone()).unwrap_or_default();
            report.appearances.push(BodyAppearance { body: translated.color, faces });
            let name = match (&translated.name, count) {
                (Some(name), 1) => name.clone(),
                (Some(name), _) => format!("{name} ({})", k + 1),
                (None, _) => String::new(),
            };
            report.names.push(name);
            report.solids.push(solid);
        }
    }
    Ok(report)
}

/// The bodies a part file holds: its root BODY, or the bodies of a
/// PART_XMT_BLOCK / POINTER_LIS_BLOCK array of parts (§2.1).
fn bodies(file: &XtFile) -> Result<Vec<&Node>, String> {
    let root = file.root().ok_or("the file holds no nodes")?;
    let parts: Vec<&Node> = match root.type_name() {
        "BODY" => vec![root],
        "ASSEMBLY" => return Err("the file is an assembly, which is not read yet".into()),
        "PART_XMT_BLOCK" | "POINTER_LIS_BLOCK" => root
            .ints("entries")
            .into_iter()
            .filter_map(|i| u32::try_from(i).ok())
            .filter_map(|i| file.node(i))
            .collect(),
        other => return Err(format!("the file's root is a {other}, not a body")),
    };
    let mut out = Vec::new();
    for part in parts {
        match part.type_name() {
            "BODY" => out.push(part),
            "ASSEMBLY" => return Err("the file holds an assembly, which is not read yet".into()),
            other => return Err(format!("the file holds a {other} where a part should be")),
        }
    }
    if out.is_empty() {
        return Err("the file holds no bodies".into());
    }
    Ok(out)
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
