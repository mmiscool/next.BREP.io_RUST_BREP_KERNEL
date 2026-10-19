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
    /// The fragment's place among its source face's fragments, from the
    /// fragmentation that cut it ([`canonical_fragment_order`]); `None` for a
    /// fragment no arrangement produced (an untouched or rescued face).
    #[serde(skip)]
    pub order: Option<FragmentOrder>,
}

/// What orders a face's fragments (lane X, fillet-bore-notch): the outer
/// area of the fragment's region in the face's parameter plane, the key the
/// arrangement returns regions by, and the region's topological key, the
/// sorted (chain, segment) list of its outer loop, then the sorted lists of
/// its holes. The topological key reads no coordinate.
#[derive(Clone, Debug, Default)]
pub struct FragmentOrder {
    pub area: f64,
    pub key: (Vec<(u64, u64)>, Vec<Vec<(u64, u64)>>),
}

/// Two outer areas within this relative distance are a tie
/// ([`canonical_fragment_order`]).
pub(crate) const FRAGMENT_AREA_TIE: f64 = 1e-9;

/// The order the assembler builds `fragments` in (lane X, fillet-bore-notch).
/// A source face's later fragments are named `_1`, `_2`, ... and every id is
/// minted in this order, and saved documents reference those names, so it must
/// not follow the last bits of an area. Each (operand, source face) group keeps
/// the positions it holds in `fragments` and refills them in ascending outer
/// area, as the arrangement emits them, except that a run of areas whose
/// neighbours lie within [`FRAGMENT_AREA_TIE`] of each other (relative to the
/// larger) is one tie, ordered by the topological key. The runs read only the
/// set of areas, so any emission order of the same fragments gives the same
/// result. A group with a fragment that carries no [`FragmentOrder`] keeps its
/// order.
pub(crate) fn canonical_fragment_order(fragments: Vec<FaceFragmentRecord>) -> Vec<FaceFragmentRecord> {
    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut group_of: HashMap<(u8, u64), usize> = HashMap::default();
    for (position, fragment) in fragments.iter().enumerate() {
        let group = *group_of.entry((fragment.operand, fragment.source_face_id)).or_insert_with(|| {
            groups.push(Vec::new());
            groups.len() - 1
        });
        groups[group].push(position);
    }
    let mut slots: Vec<Option<FaceFragmentRecord>> = fragments.into_iter().map(Some).collect();
    let mut order: Vec<usize> = (0..slots.len()).collect();
    for positions in &groups {
        let Some(keys) = positions
            .iter()
            .map(|position| slots[*position].as_ref().and_then(|fragment| fragment.order.as_ref()))
            .collect::<Option<Vec<&FragmentOrder>>>()
        else {
            continue;
        };
        if positions.len() < 2 {
            continue;
        }
        let mut members: Vec<usize> = (0..positions.len()).collect();
        members.sort_by(|a, b| keys[*a].area.total_cmp(&keys[*b].area).then_with(|| keys[*a].key.cmp(&keys[*b].key)));
        let mut start = 0;
        for end in 1..=members.len() {
            let tied = end < members.len() && {
                let (low, high) = (keys[members[end - 1]].area, keys[members[end]].area);
                high - low <= FRAGMENT_AREA_TIE * low.abs().max(high.abs())
            };
            if !tied {
                members[start..end].sort_by(|a, b| keys[*a].key.cmp(&keys[*b].key));
                start = end;
            }
        }
        for (slot, member) in positions.iter().zip(&members) {
            order[*slot] = positions[*member];
        }
    }
    order.into_iter().map(|position| slots[position].take().expect("each fragment is placed once")).collect()
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
