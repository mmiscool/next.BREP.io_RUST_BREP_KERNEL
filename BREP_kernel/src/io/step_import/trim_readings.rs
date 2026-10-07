//! Three readings of every imported edge: the file's 3D curve as written, the
//! file's own pcurve mapped through the file's surface, and the edge the
//! importer built — the instrument that says whether a defect on an imported
//! solid is the FILE's geometry or OUR reconstruction of it.
//!
//! A diagnostic lane, like [`crate::import_step_report`]: it builds the same
//! bodies through the same stages, and carries what each stage did to each edge
//! beside the solid instead of discarding it. Nothing in the import path reads
//! it.

use super::*;

/// Every flat body of one STEP document, with the per-edge provenance.
#[derive(Clone, Debug)]
pub struct StepTrimReadings {
    /// Every precision the file states, in millimetres, ascending (the same
    /// reading [`StepImportReport::stated_precisions_mm`] carries).
    pub stated_precisions_mm: Vec<f64>,
    /// The file carries a product structure. Its bodies are listed here in the
    /// FLAT order, once per representation — [`crate::import_step_report`]
    /// places one copy per occurrence instead, so the two indices only agree on
    /// a single-part file.
    pub has_structure: bool,
    pub bodies: Vec<StepBodyTrimReadings>,
}

#[derive(Clone, Debug)]
pub struct StepBodyTrimReadings {
    /// The STEP body entity (`MANIFOLD_SOLID_BREP`, `BREP_WITH_VOIDS`, or the
    /// `SHELL_BASED_SURFACE_MODEL`'s shell).
    pub body_ref: usize,
    /// The body as the importer built it, or why it refused.
    pub solid: Result<BrepSolid, String>,
    /// The shell-closure refusal the ordinary import lane draws on this body
    /// (`builder/closure.rs`), which THIS lane holds so the body can be read:
    /// a `Some` here is a body `import_step` refuses.
    pub closure_refusal: Option<String>,
    pub edges: Vec<StepEdgeReading>,
    pub faces: Vec<StepFaceReading>,
}

#[derive(Clone, Debug)]
pub struct StepEdgeReading {
    /// The built edge's id in [`StepBodyTrimReadings::solid`].
    pub edge_id: u64,
    /// The `EDGE_CURVE.edge_geometry` entity, when the file wrote one.
    pub curve_ref: Option<usize>,
    /// The file's own curve over this edge's span, `(curve, [t0, t1])`, before
    /// any importer stage — including the endpoint heal. `None` for an edge
    /// the importer minted with no file statement behind it.
    pub written: Option<(NurbsCurve, [f64; 2])>,
    /// Every stage that changed the edge's geometry, in order. Empty means the
    /// built edge is the written one (up to the endpoint heal, which is listed
    /// when it moved anything).
    pub stages: Vec<String>,
}

#[derive(Clone, Debug)]
pub struct StepFaceReading {
    /// The built face's id in [`StepBodyTrimReadings::solid`].
    pub face_id: u64,
    pub face_ref: usize,
    /// `usize::MAX` for a face the planar split minted.
    pub surface_ref: usize,
    /// The file's own trims on this face, one per coedge whose edge's
    /// `SURFACE_CURVE`/`SEAM_CURVE` bundle names this face's surface.
    pub supplied: Vec<StepSuppliedTrim>,
}

/// A vendor pcurve evaluated through the vendor's own surface.
#[derive(Clone, Debug)]
pub struct StepSuppliedTrim {
    pub edge_id: u64,
    pub forward: bool,
    /// `STATIONS + 1` points of the stated trim, each the trim point nearest
    /// the written curve's station at the same coedge fraction.
    pub stations: Vec<Vec3>,
}

/// Coedge fractions the supplied trims are sampled at.
const STATIONS: usize = 64;

/// Build every flat body of `text` with the per-edge provenance captured.
pub fn import_step_trim_readings(text: &str) -> Result<StepTrimReadings, String> {
    if !text.contains("ISO-10303-21") {
        return Err("step_import: not an ISO-10303-21 Part 21 file".into());
    }
    let entities = parse_data_section(text)?;
    let resolver = Resolver::new(&entities, derive_length_scale_mm(&entities));
    let stated_precisions_mm = stated_precisions_mm(&entities);
    let has_structure = !structure_edges(&entities, &resolver).is_empty();
    let mut bodies = Vec::new();
    for body in flat_step_bodies(&resolver, &entities) {
        let body_ref = match body {
            StepBody::SolidBrep(entity_ref) | StepBody::BrepWithVoids(entity_ref) => entity_ref,
            StepBody::SurfaceModelShell { shell_ref, .. } => shell_ref,
        };
        // The capture lane HOLDS a closure refusal (`builder/closure.rs`): the
        // body is built and read so the residual can be attributed edge by
        // edge, and the refusal it would have drawn rides beside it.
        let (solid, capture, closure_refusal) = match build_step_body_captured(&resolver, body) {
            Ok((solid, capture, readings)) => (
                Ok(solid),
                capture.unwrap_or_default(),
                readings.closure.overridden().map(|refusal| refusal.message.clone()),
            ),
            Err(error) => (Err(error.to_string()), Default::default(), None),
        };
        let mut edges: Vec<StepEdgeReading> = Vec::new();
        let mut faces: Vec<StepFaceReading> = Vec::new();
        if let Ok(solid) = &solid {
            for edge in &solid.edges {
                let written = capture.written.get(&edge.id);
                edges.push(StepEdgeReading {
                    edge_id: edge.id,
                    curve_ref: written.and_then(|written| written.curve_ref),
                    written: written.map(|written| (written.curve.clone(), [written.t0, written.t1])),
                    stages: capture.stages.get(&edge.id).cloned().unwrap_or_default(),
                });
            }
            for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
                let Some(&(_, face_ref, surface_ref)) =
                    capture.faces.iter().find(|(id, _, _)| *id == face.id)
                else {
                    continue;
                };
                let mut supplied = Vec::new();
                for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                    let Some(written) = capture.written.get(&coedge.edge_id) else {
                        continue;
                    };
                    let Some(curve_ref) = written.curve_ref else {
                        continue;
                    };
                    let bundle = resolver.supplied_pcurves(curve_ref);
                    let Some(pcurve) = bundle.iter().find(|pcurve| pcurve.basis_ref == surface_ref)
                    else {
                        continue;
                    };
                    let mut stations = Vec::with_capacity(STATIONS + 1);
                    let mut cursor = None;
                    let mut readable = true;
                    for index in 0..=STATIONS {
                        let fraction = index as f64 / STATIONS as f64;
                        let along = if coedge.forward { fraction } else { 1.0 - fraction };
                        let Ok(point) =
                            written.curve.evaluate(written.t0 + (written.t1 - written.t0) * along)
                        else {
                            readable = false;
                            break;
                        };
                        let Ok((uv, on_trim)) = pcurve.snap(point, cursor) else {
                            readable = false;
                            break;
                        };
                        cursor = Some(uv);
                        stations.push(on_trim);
                    }
                    if readable {
                        supplied.push(StepSuppliedTrim {
                            edge_id: coedge.edge_id,
                            forward: coedge.forward,
                            stations,
                        });
                    }
                }
                faces.push(StepFaceReading {
                    face_id: face.id,
                    face_ref,
                    surface_ref,
                    supplied,
                });
            }
        }
        bodies.push(StepBodyTrimReadings {
            body_ref,
            solid,
            closure_refusal,
            edges,
            faces,
        });
    }
    Ok(StepTrimReadings {
        stated_precisions_mm,
        has_structure,
        bodies,
    })
}
