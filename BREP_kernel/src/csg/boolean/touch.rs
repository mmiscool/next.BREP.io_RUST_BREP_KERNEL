//! Which operand faces the boolean TOUCHED.
//!
//! A face is touched when the imprint minted a piece on it, split one of its
//! edges, or read it as one surface with a face of the other operand (a
//! cosurface pair). Every other face carries no section, keeps every edge
//! whole and has no coincident partner, so the operation can only hand it on
//! as the operand held it. The census is read by the pipeline's counters
//! (`touch.*`, `fragment.candidates_*`) and, when
//! `BooleanOptions::untouched_passthrough` is on, decides which faces the
//! acceptance passes examine.
use super::*;

/// The per-face touch census of one binary boolean.
pub(super) struct TouchCensus {
    touched: HashSet<FaceKey>,
    /// Faces of both operands, touched or not.
    pub(super) faces: u64,
    pub(super) with_pieces: u64,
    pub(super) with_splits: u64,
    pub(super) cosurface: u64,
}

impl TouchCensus {
    /// `solids[operand]` are the operands as the imprint read them (prepared,
    /// not yet split): the census keys on their face and edge ids.
    pub(super) fn build(solids: [&BrepSolid; 2], imprint: &ImprintResultRecord) -> Self {
        let mut with_pieces: HashSet<FaceKey> = HashSet::default();
        for entry in &imprint.by_face {
            if !entry.piece_ids.is_empty() {
                with_pieces.insert(FaceKey { operand: entry.operand, face_id: entry.face_id });
            }
        }
        // Edge → the faces of its operand that use it: a split touches every
        // one of them, whichever face the section that split it belongs to.
        let mut faces_by_edge: HashMap<(u8, u64), Vec<u64>> = HashMap::default();
        let mut faces = 0u64;
        for (operand, solid) in solids.iter().enumerate() {
            for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
                faces += 1;
                for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                    let users = faces_by_edge.entry((operand as u8, coedge.edge_id)).or_default();
                    if !users.contains(&face.id) {
                        users.push(face.id);
                    }
                }
            }
        }
        let mut with_splits: HashSet<FaceKey> = HashSet::default();
        for split in &imprint.edge_splits {
            if split.parameters.is_empty() {
                continue;
            }
            for &face_id in faces_by_edge.get(&(split.operand, split.edge_id)).map(Vec::as_slice).unwrap_or(&[]) {
                with_splits.insert(FaceKey { operand: split.operand, face_id });
            }
        }
        let mut cosurface: HashSet<FaceKey> = HashSet::default();
        for (first, second) in &imprint.cosurface_pairs {
            cosurface.insert(*first);
            cosurface.insert(*second);
        }
        let mut touched = with_pieces.clone();
        touched.extend(with_splits.iter().copied());
        touched.extend(cosurface.iter().copied());
        Self {
            touched,
            faces,
            with_pieces: with_pieces.len() as u64,
            with_splits: with_splits.len() as u64,
            cosurface: cosurface.len() as u64,
        }
    }

    pub(super) fn is_touched(&self, operand: u8, face_id: u64) -> bool {
        self.touched.contains(&FaceKey { operand, face_id })
    }

    pub(super) fn touched(&self) -> u64 {
        self.touched.len() as u64
    }

    pub(super) fn untouched(&self) -> u64 {
        self.faces - self.touched()
    }

    /// The census as counters: `touch.faces_touched`, `touch.faces_untouched`
    /// and the three reasons a face is touched (a face may count under more
    /// than one).
    pub(super) fn report(&self, diagnostics: &mut KernelDiagnostics) {
        diagnostics.count_n("touch.faces_touched", self.touched());
        diagnostics.count_n("touch.faces_untouched", self.untouched());
        diagnostics.count_n("touch.faces_with_pieces", self.with_pieces);
        diagnostics.count_n("touch.faces_with_splits", self.with_splits);
        diagnostics.count_n("touch.faces_cosurface", self.cosurface);
    }

    /// Fragment candidates by the touch of their source face:
    /// `fragment.candidates_touched` / `fragment.candidates_untouched` (an
    /// untouched face yields exactly one candidate, itself) and the largest
    /// candidate count any one face produced, `fragment.candidates_per_face_max`.
    pub(super) fn report_fragments(
        &self,
        fragments: &[&[FaceFragmentRecord]],
        diagnostics: &mut KernelDiagnostics,
    ) {
        let mut per_face: HashMap<FaceKey, u64> = HashMap::default();
        let (mut on_touched, mut on_untouched) = (0u64, 0u64);
        for fragment in fragments.iter().flat_map(|list| list.iter()) {
            let key = FaceKey { operand: fragment.operand, face_id: fragment.source_face_id };
            *per_face.entry(key).or_default() += 1;
            if self.touched.contains(&key) {
                on_touched += 1;
            } else {
                on_untouched += 1;
            }
        }
        diagnostics.count_n("fragment.candidates_touched", on_touched);
        diagnostics.count_n("fragment.candidates_untouched", on_untouched);
        diagnostics.measure_max(
            "fragment.candidates_per_face_max",
            per_face.values().copied().max().unwrap_or(0) as f64,
        );
    }
}
