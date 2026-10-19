//! Diagnostic operation/triangle census; never a soundness certificate.

use crate::{BrepSolid, FaceRecord};
use rustc_hash::FxHashSet;
use std::collections::{BTreeMap, BTreeSet};

pub(crate) fn enabled() -> bool {
    super::scan_profile_enabled()
        && matches!(std::env::var("BREP_SOUNDNESS_CENSUS").as_deref(), Ok("1" | "on"))
}

// This is a diagnostic RAW-input comparison, not a mesh or verdict identity.
// Ignore lookup labels while retaining each face's complete carriers, trims,
// ordered boundary curves and physical endpoints. A malformed lookup is changed.
fn raw_face(solid: &BrepSolid, face: &FaceRecord) -> Option<Vec<u8>> {
    let mut face = face.clone();
    face.id = 0;
    face.name = None;
    let mut boundary = Vec::new();
    for l in &mut face.loops {
        l.id = 0;
        for c in &mut l.coedges {
            let mut edge = solid.edges.iter().find(|e| e.id == c.edge_id)?.clone();
            let start = solid.vertices.iter().find(|v| v.id == edge.start_vertex_id)?.point;
            let end = solid.vertices.iter().find(|v| v.id == edge.end_vertex_id)?.point;
            edge.id = 0;
            edge.name = None;
            edge.start_vertex_id = 0;
            edge.end_vertex_id = 0;
            c.id = 0;
            c.edge_id = boundary.len() as u64;
            boundary.push((edge, start, end));
        }
    }
    serde_json::to_vec(&(face, boundary)).ok()
}

fn changed_faces(solid: &BrepSolid, inputs: &[&BrepSolid]) -> FxHashSet<u64> {
    let before: FxHashSet<Vec<u8>> = inputs.iter().flat_map(|s|
        s.shells.iter().flat_map(|shell| &shell.faces).filter_map(|face| raw_face(s, face))
    ).collect();
    solid.shells.iter().flat_map(|s| &s.faces).filter(|face|
        raw_face(solid, face).is_none_or(|key| !before.contains(&key))
    ).map(|face| face.id).collect()
}

/// Emit explicit operation provenance next to the subsequent scan; no hidden
/// context is installed and no certificate or public report is changed.
pub(crate) fn profile_operation(
    solid: &BrepSolid,
    inputs: &[&BrepSolid],
    touched: Option<&FxHashSet<u64>>,
    entry: &str,
) {
    if !enabled() { return; }
    let changed;
    let (touched, source) = if let Some(touched) = touched {
        (touched, "assembly")
    } else {
        changed = changed_faces(solid, inputs);
        (&changed, "raw_inputs")
    };
    let faces: Vec<_> = solid.shells.iter().flat_map(|s| &s.faces).collect();
    let touched_edges: FxHashSet<_> = faces.iter().filter(|f| touched.contains(&f.id))
        .flat_map(|f| &f.loops).flat_map(|l| &l.coedges).map(|c| c.edge_id).collect();
    let expanded: BTreeSet<_> = faces.iter().filter(|f| touched.contains(&f.id)
        || f.loops.iter().flat_map(|l| &l.coedges).any(|c| touched_edges.contains(&c.edge_id)))
        .map(|f| f.id).collect();
    let touched: BTreeSet<_> = touched.iter().copied().collect();
    eprintln!("soundness.operation {}", serde_json::json!({
        "entry": entry, "source": source, "faces": faces.len(),
        "touched": touched, "expanded": expanded,
        "raw_changed": if inputs.is_empty() { None } else { Some(changed_faces(solid, inputs).into_iter().collect::<BTreeSet<_>>()) },
    }));
}

pub(super) struct ScanCensus {
    face_ids: Vec<u64>,
    triangles: BTreeMap<u64, usize>,
    overlaps: BTreeMap<(u64, u64), usize>,
    candidates: BTreeMap<(u64, u64), usize>,
}

impl ScanCensus {
    pub(super) fn new(face_ids: &[u64]) -> Self {
        let mut triangles = BTreeMap::new();
        for &face in face_ids { *triangles.entry(face).or_insert(0) += 1; }
        Self { face_ids: face_ids.to_vec(), triangles, overlaps: BTreeMap::new(), candidates: BTreeMap::new() }
    }

    pub(super) fn pair(&mut self, first: usize, second: usize, candidate: bool) {
        let (a, b) = (self.face_ids[first], self.face_ids[second]);
        let key = (a.min(b), a.max(b));
        let counts = if candidate { &mut self.candidates } else { &mut self.overlaps };
        *counts.entry(key).or_insert(0) += 1;
    }


    pub(super) fn emit(&self, bounds: &[crate::Aabb]) {
        let mut face_bounds = BTreeMap::new();
        for (&face, &box_of) in self.face_ids.iter().zip(bounds) {
            face_bounds.entry(face).or_insert_with(crate::Aabb::empty).include(box_of);
        }
        let bounds: Vec<_> = face_bounds.into_iter().map(|(id, b)|
            (id, [b.minimum.x, b.minimum.y, b.minimum.z], [b.maximum.x, b.maximum.y, b.maximum.z])
        ).collect();
        let pairs = |counts: &BTreeMap<(u64, u64), usize>| counts.iter()
            .map(|(&(a, b), &n)| [a, b, n as u64]).collect::<Vec<_>>();
        eprintln!("soundness.census {}", serde_json::json!({
            "triangles": self.triangles, "bounds": bounds,
            "overlaps": pairs(&self.overlaps), "candidates": pairs(&self.candidates),
        }));
    }
}

