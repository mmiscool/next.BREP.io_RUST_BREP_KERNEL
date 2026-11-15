//! A REPORT, not a rule: nothing here changes what `validate()` accepts. The
//! kind a body DECLARES is [`BodyKind`] below; the report never sets it and
//! a consumer that branches by kind reads the declaration. Pure counting
//! over ids, no geometry and no tolerance, so like `solid_euler` it cannot
//! be noisy.
use super::BrepSolid;
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::{Deserialize, Serialize};

/// The kind a body DECLARES, as distinct from the kind [`topology_kind`]
/// READS from its edge uses. Nothing infers it: a producer sets it, the wire
/// carries it, and consumers that branch by kind (the boolean's operand gate
/// first) read the declaration. Every body built today is `Solid`, the
/// default, and a Solid serialises byte for byte as before: the JSON skips
/// the key, the flat codec keeps its version-1.0 header (`solid_codec.rs`).
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyKind {
    /// A closed 2-manifold (every non-degenerate edge used exactly twice):
    /// today's only body, every rule unchanged.
    #[default]
    Solid,
    /// An open shell or a lone face: edges may bound one face (lamina).
    Sheet,
    /// Cells sharing walls: edges may bound three or more faces (spine).
    Cellular,
    /// Edges and vertices, no faces.
    Wire,
}

impl BodyKind {
    /// `true` for the default kind; the JSON `skip_serializing_if` predicate.
    pub fn is_solid(&self) -> bool {
        matches!(self, BodyKind::Solid)
    }

    /// The codec slot: `Solid` 0, `Sheet` 1, `Cellular` 2, `Wire` 3.
    pub fn codec_code(self) -> f64 {
        match self {
            BodyKind::Solid => 0.0,
            BodyKind::Sheet => 1.0,
            BodyKind::Cellular => 2.0,
            BodyKind::Wire => 3.0,
        }
    }

    /// The inverse of [`BodyKind::codec_code`]; `None` for any other value.
    pub fn from_codec_code(code: f64) -> Option<Self> {
        match code {
            code if code == 0.0 => Some(BodyKind::Solid),
            code if code == 1.0 => Some(BodyKind::Sheet),
            code if code == 2.0 => Some(BodyKind::Cellular),
            code if code == 3.0 => Some(BodyKind::Wire),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TopologyKind {
    /// No faces and no edges (`IsEmptyModel`). A body of vertex records alone
    /// reads here too.
    Empty,
    /// Edges and no faces (`IsWireFrame`).
    Wire,
    /// Faces, and every edge used by exactly two coedges (`IsManifoldSolid`),
    /// with no shell left open. Any number of shells: a box with a void is
    /// a manifold solid of two shells.
    ManifoldSolid,
    /// Faces in ONE shell, every edge lamina or manifold, at least one
    /// lamina edge (`IsSheet`): an open shell or a lone face.
    Sheet,
    /// At least one spine edge, no wire edge (`IsNonManifoldSolid`): two
    /// cells sharing a wall, a fin standing on a solid.
    NonManifoldSolid,
    /// Anything else with faces (`IsMixedModel`): a wire edge beside faces,
    /// lamina edges spread over more than one shell, two shells that share
    /// an edge once each.
    Mixed,
}

/// Outcome of [`topology_kind`]: the kind and the census it was read from.
///
/// The four edge counts partition the NON-degenerate edge records by coedge
/// uses over every loop of every face: 0 wire, 1 lamina, 2 manifold, 3 or
/// more spine. Degenerate (collapsed pole) edges are left out of every
/// count, exactly as `solid_euler` leaves them out: `validate()` accepts them
/// at one use and they say nothing about the body's class.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct KindReport {
    pub kind: TopologyKind,
    /// Non-degenerate edges used by exactly one coedge: a boundary.
    pub lamina_edges: usize,
    /// Non-degenerate edges used by exactly two coedges.
    pub manifold_edges: usize,
    /// Non-degenerate edges used by three or more coedges.
    pub spine_edges: usize,
    /// Non-degenerate edge records no coedge references.
    pub wire_edges: usize,
    /// `solid.shells.len()`.
    pub shells: usize,
    /// Shells with a boundary: some non-degenerate edge the shell's faces
    /// reference is used by exactly one coedge WITHIN that shell. Counted per
    /// shell, so an edge two shells use once each opens both, and a spine
    /// edge used three times inside one shell does not open it.
    pub open_shells: usize,
}

/// Classify a body record by its edge uses (module docs for the predicates).
///
/// The decision is total and ordered: no faces and no edges → `Empty`; no
/// faces → `Wire`; faces beside a wire edge → `Mixed`; a spine edge →
/// `NonManifoldSolid`; no lamina edge and no open shell → `ManifoldSolid`;
/// lamina edges in one shell → `Sheet`; otherwise `Mixed`. A faced body with
/// no non-degenerate edge at all reads `ManifoldSolid` vacuously.
pub fn topology_kind(solid: &BrepSolid) -> KindReport {
    let degenerate: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| edge.degenerate)
        .map(|edge| edge.id)
        .collect();
    let mut uses: HashMap<u64, usize> = HashMap::default();
    let mut faces = 0usize;
    let mut open_shells = 0usize;
    for shell in &solid.shells {
        faces += shell.faces.len();
        let mut shell_uses: HashMap<u64, usize> = HashMap::default();
        for coedge in shell
            .faces
            .iter()
            .flat_map(|face| &face.loops)
            .flat_map(|record| &record.coedges)
        {
            if degenerate.contains(&coedge.edge_id) {
                continue;
            }
            *uses.entry(coedge.edge_id).or_insert(0) += 1;
            *shell_uses.entry(coedge.edge_id).or_insert(0) += 1;
        }
        if shell_uses.values().any(|&count| count == 1) {
            open_shells += 1;
        }
    }
    let mut report = KindReport {
        kind: TopologyKind::Empty,
        lamina_edges: 0,
        manifold_edges: 0,
        spine_edges: 0,
        wire_edges: 0,
        shells: solid.shells.len(),
        open_shells,
    };
    for edge in solid.edges.iter().filter(|edge| !edge.degenerate) {
        match uses.get(&edge.id).copied().unwrap_or(0) {
            0 => report.wire_edges += 1,
            1 => report.lamina_edges += 1,
            2 => report.manifold_edges += 1,
            _ => report.spine_edges += 1,
        }
    }
    let edges = report.wire_edges + report.lamina_edges + report.manifold_edges + report.spine_edges;
    report.kind = if faces == 0 && edges == 0 {
        TopologyKind::Empty
    } else if faces == 0 {
        TopologyKind::Wire
    } else if report.wire_edges > 0 {
        TopologyKind::Mixed
    } else if report.spine_edges > 0 {
        TopologyKind::NonManifoldSolid
    } else if report.lamina_edges == 0 && report.open_shells == 0 {
        TopologyKind::ManifoldSolid
    } else if report.shells == 1 {
        TopologyKind::Sheet
    } else {
        TopologyKind::Mixed
    };
    report
}
