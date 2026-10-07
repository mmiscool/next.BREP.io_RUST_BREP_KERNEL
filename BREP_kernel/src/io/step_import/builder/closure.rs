//! The import's shell-closure gate: a body whose CLOSED shell's trims do not
//! enclose the zero vector area they owe is either refused, typed, or imported
//! carrying a typed [`Approximation`] — never returned with a note nobody
//! downstream reads.
//!
//! `Σ_faces ∫ n dA = 0` is exact on a closed shell — every edge is walked
//! twice, once each way — so the residual [`crate::shell_vector_areas`] reads
//! is the sum of the trims' departures from the edges they claim. On an import
//! that residual has one dominant cause: the file's curve does not lie on one
//! of the two carriers it bounds, each face is trimmed at the curve's
//! projection onto ITS carrier, and the two trims leave a sliver open between
//! them. A shell open by a vector area `A` has no single volume: the
//! divergence integral about a reference `R` reads `V(0) − R·A/3`, so two
//! readers with two reference points disagree by `|A| × lever / 3` and neither
//! is wrong (`importTestWorking` SOLID_03, 0.1016 mm² of residual, moved 0.17
//! mm³ between this kernel and OpenCASCADE).
//!
//! Until 2026-09-30 the importer accepted such a body silently: the per-trim
//! bar (`loops.rs`) allows every trim its stations' own standoff from the
//! carrier, which is exactly the gap that opens the shell, and the case gate
//! recorded the residual as a `note:` beside a volume it went on to compare.
//! This gate reads the shell-level consequence the per-trim bar cannot see,
//! and answers it with two comparands, both derived rather than chosen:
//!
//! * **The bar** — `PCURVE_REFINEMENT_TOLERANCE × edge length`, the residual a
//!   shell carries while every trim lies within the fit floor of its edge. A
//!   body over it is IMPORTED and carries an [`Approximation`]
//!   (`import.shell_closure`): the residual, the bar, the bound
//!   `|A| × diagonal / 3` it puts on the volume, and the edges that carry it
//!   with their STEP entities, their standoff in mm and whose curve it is (the
//!   file's, or one the reconcile pass moved). The 2026-09-30 corpus census
//!   read 25 vendor bodies over this bar in 19 of the 52 fixtures, at ratios
//!   continuous from 1.27x to 2110x, beside the kernel's own boolean and
//!   fillet fixtures at up to 37x — so this bar is a fit-quality bar, not a
//!   soundness threshold, and refusing over it would lose vendor files for
//!   inherited error the volume barely feels (integrator ruling, 2026-09-30;
//!   the 2026-09-26 ruling in `loops.rs` that a fitter's miss is not the
//!   body's to lose stands).
//! * **The cap** — the per-entity plan's I3 cap on a committed band,
//!   `KernelTolerances::pcurve_acceptance(diagonal)` (2.5% of the body's
//!   diagonal, floor 4e-3 mm) times the edge length: the residual a shell
//!   would carry with every trim off its edge by the most any band may grow
//!   to. Over it the body is REFUSED as `RefusalClass::UnsoundResult { defect:
//!   SoundnessDefect::VectorArea }`, the message naming the faces, their STEP
//!   entities and the off-carrier distances. No body in the census reaches it;
//!   the arm has a constructed witness below.
//!
//! What the scan can come back with, and what each does here:
//!
//! * [`crate::ClosureReading::Measured`] — the verdict, against the two
//!   comparands above.
//! * [`crate::ClosureReading::Unmeasured`] — the scan reached its span budget
//!   with faces unread. A scan that ran out of budget did not measure
//!   anything, so the body is ACCEPTED and the note says so (one Degraded
//!   event on the report lane); refusing it would lose a body on the cost of
//!   an instrument, not on evidence about the body.
//! * [`crate::ClosureReading::Open`] — an edge used once. `validate()` runs
//!   first and rejects an open sheet, so this arm is unreachable on a body
//!   that reaches the gate; it is written rather than assumed away.
//!
//! `BREP_IMPORT_CLOSURE_GUARD=0` holds the refusal (the reading is still
//! taken), so the guard can be shown to bite and a body it refuses can be read
//! through the probes.

use super::*;
use crate::{
    Approximation, ClosureReading, EdgeOrigin, KernelRefusal, KernelStage, KernelTolerances,
    OffCarrierEdge, ShellVectorArea, SoundnessDefect,
};

/// The approximation's stable code.
pub const SHELL_CLOSURE: &str = "import.shell_closure";

/// Stations per edge when the gate measures how far a worst face's edges
/// stand off that face's carrier. A reading for the report, not the verdict:
/// the verdict is the residual.
const OFF_CARRIER_STATIONS: usize = 32;

/// How many faces, and how many edges per face, a report names.
const NAMED_FACES: usize = 3;
const NAMED_EDGES: usize = 2;

/// What the closure scan came back with for one body the import ACCEPTED.
#[derive(Clone, Debug)]
pub(in crate::step_import) enum ClosureNote {
    /// Every closed shell measured inside its bar: the worst shell's residual
    /// and its bar.
    Closed { residual: f64, bar: f64 },
    /// A shell over its bar and under the cap: imported, carrying this.
    Approximate(Approximation),
    /// The scan gave up on at least one shell (its span budget) — the body is
    /// accepted UNJUDGED, and the report lane says so.
    Unmeasured { reason: String, spans: usize },
    /// The scan could not read some face at all (`VectorAreaReport::unreadable`);
    /// accepted unjudged like `Unmeasured`.
    Unreadable { first: String, count: usize },
    /// No closed shell to judge (unreachable after `validate()`; kept as an arm).
    Unjudged,
    /// The gate would have refused this body and `BREP_IMPORT_CLOSURE_GUARD=0`
    /// held it back, or the capture lane asked for the body regardless.
    Overridden(KernelRefusal),
}

impl ClosureNote {
    /// The refusal this body carries when the guard is off — the capture lane
    /// reads it, the ordinary lane never sees this arm.
    pub(in crate::step_import) fn overridden(&self) -> Option<&KernelRefusal> {
        match self {
            Self::Overridden(refusal) => Some(refusal),
            _ => None,
        }
    }

    /// The approximation an accepted body carries, if any.
    pub(in crate::step_import) fn approximation(&self) -> Option<&Approximation> {
        match self {
            Self::Approximate(approximation) => Some(approximation),
            _ => None,
        }
    }
}

fn guard_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var("BREP_IMPORT_CLOSURE_GUARD").map_or(true, |v| v != "0"))
}

/// What the gate needs from the builder once the solid has been assembled
/// out of it: the file entities behind each built face and edge, and which
/// edges an importer pass moved.
pub(super) struct ClosureContext {
    /// Built face id -> `(ADVANCED_FACE ref, surface ref)`.
    pub(super) face_refs: HashMap<u64, (usize, usize)>,
    /// Built edge id -> `EDGE_CURVE.edge_geometry` ref, where the edge has one.
    pub(super) curve_refs: HashMap<u64, usize>,
    /// Edges `reconcile_edges_onto_surfaces` replaced.
    pub(super) reconciled: HashSet<u64>,
}

impl<'a> SolidBuilder<'a> {
    /// Take the gate's context OUT of the builder, before its records are
    /// moved into the solid.
    pub(super) fn closure_context(&mut self) -> ClosureContext {
        ClosureContext {
            face_refs: std::mem::take(&mut self.face_refs),
            curve_refs: std::mem::take(&mut self.curve_ref_of_edge),
            reconciled: std::mem::take(&mut self.reconciled_edges),
        }
    }
}

/// The refusal comparand for one shell: the I3 cap on a committed band,
/// `pcurve_acceptance(diagonal)`, times the shell's edge length — the
/// residual the shell would carry with every trim off its edge by the cap.
pub(in crate::step_import) fn closure_cap(solid: &BrepSolid, shell: &ShellVectorArea) -> f64 {
    let diagonal = solid_diagonal(solid);
    KernelTolerances::for_solid(solid, crate::PCURVE_REFINEMENT_TOLERANCE)
        .pcurve_acceptance(diagonal)
        * shell.edge_length
}

/// The bounding-box diagonal of the solid's vertices — the longest lever a
/// reference point inside the body can have.
fn solid_diagonal(solid: &BrepSolid) -> f64 {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in &solid.vertices {
        let p = vertex.point;
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    if solid.vertices.is_empty() {
        return 0.0;
    }
    high.sub(low).length()
}

impl ClosureContext {
    /// Judge a built, validated body's shell closure. `Err` is the typed
    /// refusal (over the cap); `Ok` is the note the accepted body carries,
    /// an [`Approximation`] when a shell is over its bar. With `hold` a
    /// refusal is returned inside [`ClosureNote::Overridden`] instead — the
    /// trim-readings lane needs the built body to attribute the residual.
    pub(super) fn certify_shell_closure(
        &self,
        solid: &BrepSolid,
        shell_ref: usize,
        hold: bool,
    ) -> Result<ClosureNote, KernelRefusal> {
        let report = crate::shell_vector_areas(solid);
        if !report.unreadable.is_empty() {
            return Ok(ClosureNote::Unreadable {
                first: report.unreadable[0].clone(),
                count: report.unreadable.len(),
            });
        }
        let mut worst: Option<(f64, f64)> = None;
        let mut over_bar: Option<&ShellVectorArea> = None;
        let mut over_cap: Option<&ShellVectorArea> = None;
        for shell in &report.shells {
            match shell.reading() {
                ClosureReading::Unmeasured { reason, spans, .. } => {
                    return Ok(ClosureNote::Unmeasured {
                        reason: reason.to_string(),
                        spans,
                    });
                }
                ClosureReading::Open { .. } => {}
                ClosureReading::Measured { residual, bar, .. } => {
                    let worse = |current: Option<&ShellVectorArea>| {
                        current.map_or(true, |current| residual > current.residual())
                    };
                    if residual > closure_cap(solid, shell) && worse(over_cap) {
                        over_cap = Some(shell);
                    }
                    if residual > bar && worse(over_bar) {
                        over_bar = Some(shell);
                    }
                    if worst.map_or(true, |(current, _)| residual > current) {
                        worst = Some((residual, bar));
                    }
                }
            }
        }
        if let Some(shell) = over_cap {
            let refusal = self.closure_refusal(solid, shell_ref, shell);
            if hold || !guard_enabled() {
                return Ok(ClosureNote::Overridden(refusal));
            }
            return Err(refusal);
        }
        if let Some(shell) = over_bar {
            return Ok(ClosureNote::Approximate(self.closure_approximation(solid, shell_ref, shell)));
        }
        Ok(match worst {
            Some((residual, bar)) => ClosureNote::Closed { residual, bar },
            None => ClosureNote::Unjudged,
        })
    }

    /// The faces that carry one shell's residual, worst first, each with the
    /// edges standing off its carrier.
    fn named_faces(&self, solid: &BrepSolid, shell: &ShellVectorArea) -> (Vec<String>, Vec<OffCarrierEdge>, Vec<u64>) {
        let worst_faces = shell.worst_faces(NAMED_FACES);
        let mut named = Vec::with_capacity(worst_faces.len());
        let mut edges = Vec::new();
        for face in &worst_faces {
            let (face_ref, surface_ref) = self.face_refs.get(&face.face).copied().unwrap_or((0, 0));
            let mut lines = Vec::new();
            // Only edges standing off the carrier by more than the fit
            // floor are named: the rest are on it, and a list of 1e-14s
            // beside the one that matters would hide it.
            for edge in self
                .edges_off_carrier(solid, face.face, face_ref, surface_ref)
                .into_iter()
                .filter(|edge| edge.off_carrier_mm > crate::PCURVE_REFINEMENT_TOLERANCE)
                .take(NAMED_EDGES)
            {
                lines.push(format!(
                    "edge {} ({}, {}) {:.3e} mm off it",
                    edge.edge_id,
                    edge.curve_ref.map_or_else(
                        || "a piece with no EDGE_CURVE of its own".to_string(),
                        |r| format!("EDGE_CURVE geometry #{r}")
                    ),
                    match edge.origin {
                        EdgeOrigin::File => "the file's curve",
                        EdgeOrigin::Reconciled => "moved by the reconcile pass",
                    },
                    edge.off_carrier_mm
                ));
                edges.push(edge);
            }
            named.push(format!(
                "ADVANCED_FACE #{face_ref} (surface #{surface_ref}) {:.3e} mm²{}",
                face.residual(),
                if lines.is_empty() { String::new() } else { format!(": {}", lines.join(", ")) }
            ));
        }
        (named, edges, worst_faces.iter().map(|face| face.face).collect())
    }

    /// The approximation for a shell over its bar and under the cap.
    fn closure_approximation(&self, solid: &BrepSolid, shell_ref: usize, shell: &ShellVectorArea) -> Approximation {
        let residual = shell.residual();
        let diagonal = solid_diagonal(solid);
        let volume_bound = residual * diagonal / 3.0;
        let (named, edges, _) = self.named_faces(solid, shell);
        let message = format!(
            "CLOSED_SHELL #{shell_ref} closes to {residual:.3e} mm² of vector area against a bar of {:.3e} ({:.1}x; cap {:.3e}). Its volume is determined only to ±{volume_bound:.3e} mm³ (|A| x diagonal {diagonal:.1} / 3): two faces sharing an edge are trimmed at different curves. Faces carrying the residual: {}.",
            shell.bar,
            residual / shell.bar,
            closure_cap(solid, shell),
            named.join("; ")
        );
        Approximation {
            code: SHELL_CLOSURE.to_string(),
            body: format!("CLOSED_SHELL #{shell_ref}"),
            measured: residual,
            bar: shell.bar,
            volume_bound: Some(volume_bound),
            edges,
            budget: None,
            message,
        }
    }

    /// The refusal for one shell over the cap.
    fn closure_refusal(&self, solid: &BrepSolid, shell_ref: usize, shell: &ShellVectorArea) -> KernelRefusal {
        let residual = shell.residual();
        let (named, _, faces) = self.named_faces(solid, shell);
        let message = format!(
            "step_import: CLOSED_SHELL #{shell_ref} does not close: the trims of its {} faces enclose {residual:.3e} mm² of vector area against a bar of {:.3e} ({:.1}x) and over the cap of {:.3e} ({:.1} mm of edge, quadrature error {:.1e}). Each face is trimmed at its edges' projection onto its own carrier, and where a carrier misses the curve the two projections leave a sliver open wider than any tolerance band may grow to, so the body has no volume the kernel can certify. Faces carrying the residual: {}.",
            shell.faces,
            shell.bar,
            residual / shell.bar,
            closure_cap(solid, shell),
            shell.edge_length,
            shell.quadrature_error,
            named.join("; ")
        );
        KernelRefusal::unsound(KernelStage::Validate, SoundnessDefect::VectorArea, faces, message)
    }

    /// Every non-degenerate edge of face `face_id`, with the file's curve
    /// entity when the edge has one, how far the BUILT edge curve stands off
    /// the face's carrier over [`OFF_CARRIER_STATIONS`] stations, and whose
    /// curve it is; worst first. The projector's distance is an upper bound
    /// (the `seams.rs` doc on the reconcile pass says why), so a reading here
    /// can over-read on a carrier nearly closed in one direction; it is
    /// context for the report, not its evidence.
    fn edges_off_carrier(
        &self,
        solid: &BrepSolid,
        face_id: u64,
        face_ref: usize,
        surface_ref: usize,
    ) -> Vec<OffCarrierEdge> {
        let Some(face) = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == face_id)
        else {
            return Vec::new();
        };
        let mut seen = HashSet::default();
        let mut out = Vec::new();
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if !seen.insert(coedge.edge_id) {
                continue;
            }
            let Some(edge) = solid.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
                continue;
            };
            if edge.degenerate {
                continue;
            }
            let mut worst = 0.0_f64;
            for station in 0..=OFF_CARRIER_STATIONS {
                let t = edge.t0 + (edge.t1 - edge.t0) * (station as f64 / OFF_CARRIER_STATIONS as f64);
                let Ok(point) = edge.curve.evaluate(t) else { continue };
                let Ok(projection) = crate::project_point_to_surface(&face.surface, point) else {
                    continue;
                };
                worst = worst.max(projection.distance);
            }
            out.push(OffCarrierEdge {
                edge_id: edge.id,
                curve_ref: self.curve_refs.get(&edge.id).copied(),
                face_ref,
                surface_ref,
                off_carrier_mm: worst,
                origin: if self.reconciled.contains(&edge.id) {
                    EdgeOrigin::Reconciled
                } else {
                    EdgeOrigin::File
                },
            });
        }
        out.sort_by(|a, b| {
            b.off_carrier_mm
                .partial_cmp(&a.off_carrier_mm)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        out
    }
}

