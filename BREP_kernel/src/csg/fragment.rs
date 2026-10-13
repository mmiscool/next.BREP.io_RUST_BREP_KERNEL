use crate::classification::{parameter_point_in_face, PolygonClass};
use crate::imprint::{FaceKey, ImprintPieceRecord, ImprintResultRecord};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord};
use crate::{
    arrange_segments, interpolate_curve, project_point_to_curve, KnotVector, NurbsCurve,
    NurbsSurface, Segment2, Vec2, Vec3,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::Serialize;
use serde_json::json;

#[path = "fragment/types.rs"]
mod types;
#[path = "fragment/sampling.rs"]
mod sampling;
#[path = "fragment/loops.rs"]
mod loops;
#[path = "fragment/face_split.rs"]
mod face_split;
#[path = "fragment/loop_split.rs"]
mod loop_split;

pub use face_split::{fragment_face, fragment_solid};
pub use types::{FaceFragmentRecord, FragmentCoedge, FragmentEdgeSource, FragmentLoop};
use std::time::Duration;
use web_time::Instant;

/// Per-stage wall-clock accumulators for one operand's `fragment_solid`,
/// printed to stderr when `BREP_PROFILE` is set (the `imprint.profile`
/// pattern, "measure before changing"). Stages are the face splitter's in
/// order: the untouched fast return, the domain and strip-resolution setup
/// with the band re-base, the cut hints and keys, the boundary chains, the
/// cut chains, the duplicate drop and bridges, the 2-D arrangement, the
/// regions (interior points, containment, loops) and the uncut rescue. The
/// three slowest faces are named with their time.
#[derive(Default)]
pub(crate) struct FragmentProfile {
    pub(crate) enabled: bool,
    faces: u64,
    cut_faces: u64,
    untouched: Duration,
    setup: Duration,
    hints: Duration,
    boundary: Duration,
    cuts: Duration,
    dedup_bridge: Duration,
    arrange: Duration,
    regions: Duration,
    rescue: Duration,
    loop_split_faces: u64,
    loop_split_declined: u64,
    loop_split: Duration,
    slowest: Vec<(u64, f64)>,
}

impl FragmentProfile {
    pub(crate) fn new() -> Self {
        Self {
            enabled: std::env::var("BREP_PROFILE").is_ok(),
            ..Self::default()
        }
    }

    pub(crate) fn lap(&self, started: &mut Option<Instant>) -> Duration {
        if !self.enabled {
            return Duration::ZERO;
        }
        let now = Instant::now();
        let elapsed = started.map(|s| now - s).unwrap_or_default();
        *started = Some(now);
        elapsed
    }

    fn face_done(&mut self, face_id: u64, started: Option<Instant>) {
        if !self.enabled {
            return;
        }
        self.faces += 1;
        let ms = started.map_or(0.0, |s| s.elapsed().as_secs_f64() * 1_000.0);
        self.slowest.push((face_id, ms));
        self.slowest.sort_by(|a, b| b.1.total_cmp(&a.1));
        self.slowest.truncate(3);
    }

    pub(crate) fn report(&self, operand: u8) {
        if !self.enabled {
            return;
        }
        let ms = |d: Duration| d.as_secs_f64() * 1_000.0;
        eprintln!(
            "fragment.profile operand={operand} faces={} cut_faces={} untouched={:.2} setup={:.2} hints={:.2} boundary={:.2} cuts={:.2} dedup_bridge={:.2} arrange={:.2} regions={:.2} rescue={:.2} loop_split_faces={} loop_split_declined={} loop_split={:.2} slowest={:?}",
            self.faces,
            self.cut_faces,
            ms(self.untouched),
            ms(self.setup),
            ms(self.hints),
            ms(self.boundary),
            ms(self.cuts),
            ms(self.dedup_bridge),
            ms(self.arrange),
            ms(self.regions),
            ms(self.rescue),
            self.loop_split_faces,
            self.loop_split_declined,
            ms(self.loop_split),
            self.slowest.iter().map(|(id, ms)| format!("{id}:{ms:.2}")).collect::<Vec<_>>(),
        );
    }
}
