use super::*;

#[derive(Clone, Debug, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum FragmentEdgeSource {
    Boundary {
        operand: u8,
        edge_id: u64,
    },
    /// A CROSS-OPERAND boundary reference: this fragment's coedge rides an
    /// existing edge of the OTHER operand (a closed shared-section ring —
    /// the inscribed sphere's equator riding the cylinder's cap edge). The
    /// post-construction operand fix-up must NOT rewrite it.
    SharedBoundary {
        operand: u8,
        edge_id: u64,
    },
    Imprint {
        piece_id: u64,
    },
    Derived {
        curve: NurbsCurve,
        t0: f64,
        t1: f64,
        start: Vec3,
        end: Vec3,
    },
}

#[derive(Clone, Debug, Serialize)]
pub struct FragmentCoedge {
    pub source: FragmentEdgeSource,
    pub forward: bool,
    pub pcurve: NurbsCurve,
}

#[derive(Clone, Debug, Serialize)]
pub struct FragmentLoop {
    pub coedges: Vec<FragmentCoedge>,
}

#[derive(Clone, Debug, Serialize)]
pub struct FaceFragmentRecord {
    pub operand: u8,
    pub source_face_id: u64,
    pub surface: NurbsSurface,
    pub same_sense: bool,
    pub loops: Vec<FragmentLoop>,
    pub test_point: Vec3,
    pub test_uv: Vec2,
    /// Additional well-separated interior points. Fragment selection
    /// classifies these alongside `test_point` and overturns the primary
    /// only when the extras unanimously contradict it — a single test
    /// point landing in a pathological spot (near a seam or an edge of
    /// the other solid) must not decide the whole fragment.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub extra_test_points: Vec<Vec3>,
}

#[derive(Clone)]
pub(super) enum ChainSource {
    Boundary {
        coedge: CoedgeRecord,
        curve: NurbsCurve,
    },
    Cut {
        piece_id: u64,
        pcurve: NurbsCurve,
        /// The whole-period shift the arrangement placed this chain at, off
        /// the pcurve's own period (zero unless the cut-chain period lift in
        /// `fragment_face` moved it into the face's unwrapped band).
        lift: Vec2,
        curve: NurbsCurve,
        /// (operand, edge_id, aligned) when the piece rides an existing
        /// closed boundary ring (shared_edge on a closed piece) — the
        /// fragment coedge then references that edge directly, since the
        /// assembler's endpoint weld cannot unify closed rings.
        shared_ring: Option<(u8, u64, bool)>,
    },
}

#[derive(Clone)]
pub(super) struct Chain {
    pub(super) source: ChainSource,
    pub(super) count: usize,
    pub(super) start: Vec2,
    pub(super) end: Vec2,
    pub(super) segments: Vec<(Vec2, Vec2)>,
}
