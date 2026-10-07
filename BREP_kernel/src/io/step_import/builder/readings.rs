//! What the importer did to each edge between the file's statement of it and
//! the edge it built — captured only when [`crate::import_step_trim_readings`]
//! asks, so an ordinary import pays for one `Option` test per stage.
//!
//! The capture keys every record by the kernel id the builder minted, which is
//! the id the finished solid carries: nothing after `finish_face` renumbers an
//! edge or a face (the pinched-vertex split adds vertices, the orientation
//! flip reverses loops in place).

use super::*;

/// The file's own curve for one built edge, over that edge's span, before any
/// importer stage touched it.
#[derive(Clone, Debug)]
pub(in crate::step_import) struct WrittenEdge {
    /// The `EDGE_CURVE.edge_geometry` entity, when there is one.
    pub(in crate::step_import) curve_ref: Option<usize>,
    pub(in crate::step_import) curve: NurbsCurve,
    pub(in crate::step_import) t0: f64,
    pub(in crate::step_import) t1: f64,
}

#[derive(Clone, Debug, Default)]
pub(in crate::step_import) struct TrimCapture {
    /// Edge id -> the file's curve. Absent for an edge the importer minted
    /// with no file statement behind it: a synthesized pole, a seam ruling, a
    /// planar-split diagonal, a piece of an edge re-parameterised before it
    /// was split.
    pub(in crate::step_import) written: HashMap<u64, WrittenEdge>,
    /// Edge id -> every importer stage that changed the edge's geometry, in
    /// the order they ran.
    pub(in crate::step_import) stages: HashMap<u64, Vec<String>>,
    /// `(face id, STEP face ref, STEP surface ref)` per built face.
    pub(in crate::step_import) faces: Vec<(u64, usize, usize)>,
}

/// A derived trim still off the fit bar (the 1e-7 floor plus its stations'
/// standoff) after the refit at the floor: accepted into the solid at the
/// residual it reached, as the importer before the bar accepted every fit,
/// which `EntityTolerances` then measures as that edge's band, and named on
/// the import report by class — the file's residual or the fitter's miss.
#[derive(Clone, Debug)]
pub(in crate::step_import) struct BoundedTrim {
    /// The `ADVANCED_FACE` entity the trim belongs to.
    pub(in crate::step_import) face_ref: usize,
    pub(in crate::step_import) surface_ref: usize,
    /// The built edge's id in the solid.
    pub(in crate::step_import) edge_id: u64,
    /// The trim's residual, mm — the band the edge will carry: the larger of
    /// the fit's own out-of-sample sweep and the band-free read between its
    /// stations (`geometry::trim_floor_reading`, 2026-10-03).
    pub(in crate::step_import) residual: f64,
    /// How far the file's curve stands off the carrier: the larger of the
    /// fit's station standoff and the standoff read between the stations.
    pub(in crate::step_import) standoff: f64,
    /// The fit's own two numbers, as reported, for the record.
    pub(in crate::step_import) fit_residual: f64,
    pub(in crate::step_import) fit_standoff: f64,
    /// How far the image sits from the stations' FEET, mm.
    pub(in crate::step_import) image_to_foot: f64,
    /// The same read against the global projector's foot alone, the reading
    /// before 2026-10-04. The foot `image_to_foot` uses is the one nearer the
    /// EDGE point, so neither value bounds the other; NaN means unreadable.
    pub(in crate::step_import) image_to_global_foot: f64,
    pub(in crate::step_import) samples: usize,
    pub(in crate::step_import) exit: String,
    /// The residual exceeds the floor plus the file curve's standoff read
    /// between the stations: the miss is the FITTER's, not the file's. Until
    /// 2026-10-03 this read `image_to_foot > floor`, which tagged a trim
    /// whose image slips tangentially along the carrier by less than the
    /// file's own standoff — 89 of the 189 rows so tagged on the corpus.
    pub(in crate::step_import) fitter: bool,
    /// How far the fit's stations were clamped into the carrier's chart, in
    /// parameter ([`crate::PcurveFitReport::clamped_excursion`]); zero when
    /// the trim stayed inside it.
    pub(in crate::step_import) clamped_excursion: f64,
    /// The 3D distance that clamp corresponds to, mm.
    pub(in crate::step_import) clamped_distance: f64,
}

/// What one ACCEPTED body carries beside its solid: the trims accepted off the
/// fit bar, and what the shell-closure gate read.
#[derive(Clone, Debug)]
pub(in crate::step_import) struct BodyReadings {
    pub(in crate::step_import) bounded: Vec<BoundedTrim>,
    pub(in crate::step_import) closure: super::ClosureNote,
}

impl<'a> SolidBuilder<'a> {
    pub(super) fn capture_written(
        &mut self,
        edge_id: u64,
        curve_ref: Option<usize>,
        curve: &NurbsCurve,
    ) -> Result<(), String> {
        if let Some(capture) = self.readings.as_mut() {
            let [t0, t1] = curve.domain()?;
            capture.written.insert(
                edge_id,
                WrittenEdge {
                    curve_ref,
                    curve: curve.clone(),
                    t0,
                    t1,
                },
            );
        }
        Ok(())
    }

    pub(super) fn capture_stage(&mut self, edge_id: u64, stage: impl FnOnce() -> String) {
        if let Some(capture) = self.readings.as_mut() {
            capture.stages.entry(edge_id).or_default().push(stage());
        }
    }

    /// A seam split: each piece inherits the parent's file curve over its own
    /// span, which is only the same curve when no earlier stage re-parameterised
    /// the parent. Otherwise the pieces have no file statement to compare with.
    pub(super) fn capture_split(&mut self, parent: u64, t: f64, left: u64, right: u64) {
        let Some(capture) = self.readings.as_mut() else {
            return;
        };
        let history = capture.stages.get(&parent).cloned().unwrap_or_default();
        let reparameterised = history
            .iter()
            .any(|stage| stage.starts_with("rim re-seated") || stage.starts_with("rotated"));
        if let Some(written) = capture.written.get(&parent).cloned() {
            if !reparameterised {
                capture.written.insert(
                    left,
                    WrittenEdge {
                        t1: t,
                        ..written.clone()
                    },
                );
                capture.written.insert(right, WrittenEdge { t0: t, ..written });
            }
        }
        for piece in [left, right] {
            let mut stages = history.clone();
            stages.push(format!("split at the seam from edge {parent} (t = {t:.9})"));
            capture.stages.insert(piece, stages);
        }
    }

    pub(super) fn capture_face(&mut self, face_id: u64, face_ref: usize, surface_ref: usize) {
        if let Some(capture) = self.readings.as_mut() {
            capture.faces.push((face_id, face_ref, surface_ref));
        }
    }
}
