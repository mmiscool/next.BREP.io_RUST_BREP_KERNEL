//! # Structural soundness — the checks `validate()` deliberately does NOT make
//!
//! [`BrepSolid::validate`](crate::BrepSolid::validate) is an
//! **incidence** test: ids resolve, loops close, an edge is used twice with
//! opposite senses, a pcurve tracks its edge, Euler closes. A great deal of
//! code reads "validate passed" as exactly that claim, so this module does not
//! widen it. What `validate()` never asks:
//!
//! * **Is the thing connected?** Nothing in the incidence test notices that a
//!   shell's faces fall into two mutually unreachable pieces. Every local
//!   incidence can be perfect while the "solid" is two solids in one record.
//! * **Does it pass through itself?** `validate()`'s one string carrying the
//!   words "self-intersects" is a *uv-wire* warning — one loop's sampled
//!   pcurve polygon crossing itself in parameter space. It is not, and never
//!   was, a face-versus-face test.
//! * **Does a single face's own BOUNDARY pass through itself?** That is what
//!   the uv-wire warning above is about — and `validate()` does not return it.
//!   It goes to [`ValidationReport::wire_warnings`](crate::ValidationReport),
//!   and `validate()` hands back `issues`, so every caller in the kernel that
//!   reads `validate().is_empty()` has never seen that verdict. It was also
//!   measured blind to the symmetric case (see [`loop_self_crossings`]).
//!
//! Both live here as SEPARATE detectors, following the precedent set by
//! `csg/oracle.rs`: an independent wrongness detector that a caller asks for,
//! never a silent widening of an existing verdict. Nothing in the kernel's own
//! build path calls either of them; what a CALLER does with the verdict — gate
//! or record — is the table below, and it is the case gate that acts on it.
//!
//! ## Gate or diagnostic — the standing decision
//!
//! | check | cost | verdict |
//! |---|---|---|
//! | [`solid_connectivity`] — per-shell edge-connected components, cross-shell edges | pure topology, one pass over the coedges, no geometry | **hard gate.** A split shell is never acceptable and the test cannot be noisy: it reads ids, not numbers. [`ConnectivityReport::is_disconnected`] is the gate predicate. |
//! | `ConnectivityReport::pinch_vertices` — vertices whose incident faces form more than one fan | same pass | **diagnostic.** A pinch is a real non-manifold defect, but pole/seam vertices are false-positive bait, so it is reported beside the gate rather than inside it. Vertices with an incident degenerate edge are skipped outright and counted in `pinch_skipped`. |
//! | [`loop_self_crossings`] — one face loop crossing ITSELF | samples each trim at the density the containment scan uses and sweeps the chords; no tessellation, no surface evaluation until a hit | **hard gate.** [`LoopCrossingReport::is_flagged`] is the predicate. A bowtie is not a tolerance question: a face's boundary either passes through itself in its own domain or it does not, and no valid face has one. The whole case population reads zero, the blend acceptance refuses a result that does not, and the case gate diffs the count per solid. |
//! | [`face_self_intersections`] — ONE face folded through itself in 3D | the SAME tessellation and BVH sweep as the row below — it is that scan's same-face half, confirmed rather than counted | **hard gate.** A face whose surface carries two distinct trimmed parameter feet to one 3D point is wrong however its volume reads, and no valid face has one. Added 2026-09-13; the case gate diffs the count per solid, and the offset-shell, thicken, face-offset, sweep, loft and boolean acceptances resolve or refuse a result that has one. |
//! | [`shell_vector_areas`] — a CLOSED shell's TRIMS enclosing the zero vector area they owe | one pass over the trims and the edge curves, at the quadrature the mass integrals use; no tessellation, no projection, no surface-surface work | **gate for a NEW crossing of the bar,** and a measurement otherwise. `Σ_faces ∫ n dA = 0` is exact on a closed shell — every edge is walked twice, once each way — so the residual is the sum of the trims' departures from the edges they claim, which is the accuracy floor under every mass property and which nothing in the kernel read before 2026-09-16. The bar is derived, not chosen: `PCURVE_REFINEMENT_TOLERANCE` times the shell's edge length. The case gate records the residual per solid and fails a solid that goes over its bar when its baseline was inside it. |
//! | [`solid_self_intersections`] — face-versus-face crossing | tessellates the solid and walks a triangle BVH | **gate for NEW occurrences.** [`SelfIntersectionReport::is_flagged`] is the predicate. It shipped opt-in because it costs a full tessellation plus a BVH sweep; the decision was reversed once the first corpus run found a case labelled `correct` whose faces cross four orders of magnitude past the band, because a gate that fires only when someone remembers a flag is a diagnostic with extra steps. The case gate now runs it by default, reports a crossing its baseline already holds as a note, and fails a crossing on any other face pair. |
//!
//! ## How the self-intersection test avoids being noisy
//!
//! A mesh-only test flags every tangent neighbour: two faces meeting G1 along
//! a shared edge chord-cross each other one triangle row in, and a fillet
//! corpus is nothing but tangent neighbours. Two rules keep this one quiet:
//!
//! 1. **Welded-vertex skip.** The watertight tessellator pins shared-edge
//!    samples to bit-identical positions, so triangles that meet along a
//!    shared edge share a welded vertex and are never compared.
//! 2. **Exact-surface confirmation.** A mesh crossing is only a *candidate*.
//!    It is confirmed by projecting each triangle's corners onto the OTHER
//!    face's carrier surface and requiring the signed distances to straddle
//!    zero by more than [`SelfIntersectionOptions::straddle_band`]. Every
//!    tessellation vertex lies exactly on its own surface, so chord error
//!    cannot manufacture a straddle: a tangent neighbour's corners all sit on
//!    one side of the other carrier, while a genuine piercing has corners on
//!    both. The confirmation runs only on BVH+tri-tri survivors, so it is
//!    paid a handful of times per solid, not per triangle.
//!
//! ## The same-face case, and why it is a different question
//!
//! Only DISTINCT face pairs can be confirmed that way. When both triangles lie
//! on ONE face the straddle test is degenerate by construction — every corner
//! of both triangles is on that carrier at distance zero — so for a long time
//! a single face folding through itself was only COUNTED, in
//! [`SelfIntersectionReport::same_face_candidates`].
//!
//! [`face_self_intersections`] closes that, by asking the question the fold
//! actually is, in parameter space: the surface carries two DISTINCT trimmed
//! feet to ONE point in space. Each triangle is seeded from its own three
//! corners, and then every branch of the surface that comes near that point is
//! ENUMERATED from a sample grid over the face's own domain and refined
//! through the branch-preserving Newton `project_point_to_surface_seeded` —
//! the same instrument the pcurve tracer uses to stay on one branch of a
//! surface that folds back. A global projection cannot do this job and is not
//! asked to: measured on the ribbon fixture, `project_point_to_surface` put a
//! point lying EXACTLY on the surface 2.766 away from its own foot. A crossing
//! is confirmed only when two DISTINCT feet reach the point, both inside the
//! trim, with non-parallel normals. The last two rules are what keep it quiet:
//! chord error on a tight curve leaves ONE foot however many seeds are
//! refined, and a degenerate parameterization (a sphere's pole, a seam)
//! carries distinct uv to one point with one tangent plane. Two triangles of
//! one face that merely TOUCH — a
//! carved offset image pinching to the apex of a cone, a torus sheet meeting
//! its own axis point — never reach the confirmation at all: they either share
//! a welded vertex or their crossing segment has zero length.
//!
//! What it does NOT ask: a face that lies ON itself over an area (a coplanar
//! double cover) rather than crossing, since a coplanar triangle pair returns
//! no crossing segment; and a fold whose whole crossing region is finer than
//! the tessellation's chord tolerance, which this — like every check keyed to
//! a mesh — resolves nothing below.
//!
//! ## The one that is not a shape question at all
//!
//! Every check above asks about a PLACE — a face, a pair of faces, a loop.
//! [`shell_vector_areas`] asks about a sum: a closed shell's faces enclose
//! zero vector area, whatever shape they are, so any residual is error and
//! the only question is whose. It is the check that notices a trim lying off
//! the edge it claims while every other verdict here is silent — the
//! incidence is perfect, the topology is untouched, no face crosses anything,
//! and the volume is an integral over exactly those trims, so it moves
//! without anything to compare it against. Three such defects on one user
//! solid (the notched cap, 2026-09-16) sat between 5e-7 and 1.9e-5 while
//! `validate()`'s own pcurve-versus-edge bar was 4e-3, and the solid's volume
//! matched its closed form because two of them cancelled.
//!
//! [`loop_self_crossings`] is that blind spot covered, in parameter space
//! where the question is exact. It was added on 2026-09-13 after the
//! rib-spine partition composed a solid whose two rib side faces each had a
//! loop crossing itself twice and EVERY check above passed it: `validate()`
//! silent, one connected component, no pinch, χ = 2, and the crossing scan 0
//! confirmed / 0 rejected / 0 undecided / **0 same-face candidates** over 3900
//! triangles — because both inverted lobes lie in the face's own plane, so
//! there is no surface crossing to find at all. The face's AREA was the only
//! other tell.
//!
//! What that detector in turn does not ask: whether two DIFFERENT loops of one
//! face cross each other (a hole loop escaping its outer loop), and whether a
//! loop runs back along itself without crossing (collinear overlap — which is
//! what a periodic face's seam looks like, traversed once each way, so it
//! cannot be convicted here).

use crate::arrangement::Vec2;
use crate::classification::{parameter_point_in_face, PolygonClass};
use crate::projection::{project_point_to_surface, project_point_to_surface_seeded};
use crate::spatial::{Aabb, Bvh};
use crate::mass_properties::{split_at_surface_breaks, surface_breaks, GAUSS_W, GAUSS_X};
use crate::topology::{BrepSolid, FaceRecord, LoopRecord};
use crate::{NurbsCurve, NurbsSurface, Vec3};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::Serialize;

// ---------------------------------------------------------------------------
// Disjoint set — shared by the shell and the vertex-fan scans
// ---------------------------------------------------------------------------

struct DisjointSet {
    parent: Vec<usize>,
}

impl DisjointSet {
    fn new(count: usize) -> Self {
        Self {
            parent: (0..count).collect(),
        }
    }

    fn find(&mut self, mut index: usize) -> usize {
        while self.parent[index] != index {
            self.parent[index] = self.parent[self.parent[index]];
            index = self.parent[index];
        }
        index
    }

    fn union(&mut self, first: usize, second: usize) {
        let (a, b) = (self.find(first), self.find(second));
        if a != b {
            self.parent[b] = a;
        }
    }
}

// ---------------------------------------------------------------------------
// Connectivity
// ---------------------------------------------------------------------------

/// One shell's connectivity: how many edge-connected pieces its faces form.
#[derive(Clone, Debug, Serialize)]
pub struct ShellConnectivity {
    pub shell_id: u64,
    pub faces: usize,
    /// Edge-connected components of this shell's faces. Anything but 1 means
    /// the record holds several surfaces under one shell.
    pub components: usize,
    /// Face ids of every component EXCEPT the largest, smallest first — the
    /// pieces that would each have had to be their own shell. Empty when
    /// `components == 1`.
    pub detached: Vec<Vec<u64>>,
}

/// Outcome of [`solid_connectivity`].
#[derive(Clone, Debug, Serialize)]
pub struct ConnectivityReport {
    pub shells: Vec<ShellConnectivity>,
    /// Edges used by faces belonging to two DIFFERENT shells. A shell is a
    /// closed surface in its own right; sharing an edge across shells means
    /// the shell split is fiction.
    pub cross_shell_edges: Vec<u64>,
    /// Vertices where the incident face corners form more than one fan — two
    /// pieces of surface touching at a point and nowhere else. Edge
    /// connectivity cannot see this: both pieces are edge-connected, just not
    /// through this vertex. Diagnostic, not part of [`Self::is_disconnected`].
    pub pinch_vertices: Vec<u64>,
    /// Vertices the fan scan skipped because a degenerate (pole) edge meets
    /// there — a collapsed parameter boundary is not an ordinary fan and the
    /// scan would read it as a pinch.
    pub pinch_skipped: usize,
}

impl ConnectivityReport {
    /// The HARD-GATE predicate: the record claims to be one closed surface per
    /// shell and it is not. Pinch vertices are deliberately excluded — see the
    /// module table.
    pub fn is_disconnected(&self) -> bool {
        self.shells.iter().any(|shell| shell.components > 1)
            || !self.cross_shell_edges.is_empty()
    }

    /// One line naming what is wrong, or `None` when the gate predicate is
    /// clear. Pinch vertices are appended when present so a caller printing
    /// the summary still sees them.
    pub fn summary(&self) -> Option<String> {
        let mut parts = Vec::new();
        for shell in &self.shells {
            if shell.components > 1 {
                let sizes: Vec<usize> = shell.detached.iter().map(Vec::len).collect();
                parts.push(format!(
                    "shell {} splits into {} edge-connected components ({} face(s), detached sizes {sizes:?}, first detached faces {:?})",
                    shell.shell_id,
                    shell.components,
                    shell.faces,
                    shell.detached.first().map(Vec::as_slice).unwrap_or(&[]),
                ));
            }
        }
        if !self.cross_shell_edges.is_empty() {
            parts.push(format!(
                "{} edge(s) shared across shells ({:?})",
                self.cross_shell_edges.len(),
                &self.cross_shell_edges[..self.cross_shell_edges.len().min(8)]
            ));
        }
        if !self.pinch_vertices.is_empty() {
            parts.push(format!(
                "{} pinch vertex/vertices ({:?})",
                self.pinch_vertices.len(),
                &self.pinch_vertices[..self.pinch_vertices.len().min(8)]
            ));
        }
        if parts.is_empty() {
            None
        } else {
            Some(parts.join("; "))
        }
    }
}

/// Connectivity of a solid's topology: are each shell's faces reachable from
/// one another across shared edges, do any two shells share an edge, and does
/// any vertex pinch two otherwise-separate fans together.
///
/// Pure topology — ids and use counts, never a coordinate — so it is
/// tolerance-free and cannot be noisy. Degenerate (pole) edges DO count as
/// connections: a collapsed boundary shared by two faces genuinely joins them.
pub fn solid_connectivity(solid: &BrepSolid) -> ConnectivityReport {
    // Global face indexing over (shell, face) in record order.
    let mut face_shell: Vec<usize> = Vec::new();
    let mut face_ids: Vec<u64> = Vec::new();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for face in &shell.faces {
            face_shell.push(shell_index);
            face_ids.push(face.id);
        }
    }

    // edge id -> the global face indices that use it.
    let mut edge_faces: HashMap<u64, Vec<usize>> = HashMap::default();
    let mut global_index = 0usize;
    for shell in &solid.shells {
        for face in &shell.faces {
            let _ = face;
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    let users = edge_faces.entry(coedge.edge_id).or_default();
                    if users.last() != Some(&global_index) {
                        users.push(global_index);
                    }
                }
            }
            global_index += 1;
        }
    }

    let mut sets = DisjointSet::new(face_ids.len());
    let mut cross_shell_edges = Vec::new();
    for (edge_id, users) in &edge_faces {
        for pair in users.windows(2) {
            sets.union(pair[0], pair[1]);
        }
        let shells_touched: HashSet<usize> =
            users.iter().map(|index| face_shell[*index]).collect();
        if shells_touched.len() > 1 {
            cross_shell_edges.push(*edge_id);
        }
    }
    cross_shell_edges.sort_unstable();

    let mut shells = Vec::with_capacity(solid.shells.len());
    let mut base = 0usize;
    for shell in &solid.shells {
        let count = shell.faces.len();
        let mut buckets: HashMap<usize, Vec<u64>> = HashMap::default();
        for offset in 0..count {
            let root = sets.find(base + offset);
            buckets.entry(root).or_default().push(face_ids[base + offset]);
        }
        let mut groups: Vec<Vec<u64>> = buckets.into_values().collect();
        // Largest first so `detached` is "everything but the main piece", and
        // deterministic: ties break on the smallest face id.
        groups.sort_by(|a, b| b.len().cmp(&a.len()).then(a.first().cmp(&b.first())));
        let components = groups.len();
        let mut detached: Vec<Vec<u64>> = groups.into_iter().skip(1).collect();
        detached.sort_by_key(Vec::len);
        for group in &mut detached {
            group.sort_unstable();
        }
        shells.push(ShellConnectivity {
            shell_id: shell.id,
            faces: count,
            components,
            detached,
        });
        base += count;
    }

    let (pinch_vertices, pinch_skipped) = pinch_vertices(solid);

    ConnectivityReport {
        shells,
        cross_shell_edges,
        pinch_vertices,
        pinch_skipped,
    }
}

/// Vertices whose incident face corners form more than one fan.
///
/// A *corner* is one consecutive coedge pair of a loop meeting at the vertex.
/// Each non-degenerate edge at the vertex is used by exactly two coedges, each
/// belonging to exactly one corner there, so the edge links those two corners.
/// One fan = one component. Two cones touching at their apex are edge-
/// connected everywhere else and split into two fans exactly here.
///
/// Any vertex with an incident DEGENERATE edge is skipped: a collapsed pole
/// boundary is not an ordinary fan and would read as a pinch.
fn pinch_vertices(solid: &BrepSolid) -> (Vec<u64>, usize) {
    let mut degenerate_at: HashSet<u64> = HashSet::default();
    for edge in &solid.edges {
        if edge.degenerate {
            degenerate_at.insert(edge.start_vertex_id);
            degenerate_at.insert(edge.end_vertex_id);
        }
    }
    let edges: HashMap<u64, &crate::topology::EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();

    // vertex id -> corners; a corner is the pair of edge ids meeting there.
    let mut corners: HashMap<u64, Vec<[u64; 2]>> = HashMap::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            for loop_record in &face.loops {
                let count = loop_record.coedges.len();
                if count == 0 {
                    continue;
                }
                for index in 0..count {
                    let current = &loop_record.coedges[index];
                    let next = &loop_record.coedges[(index + 1) % count];
                    let Some(current_edge) = edges.get(&current.edge_id) else {
                        continue;
                    };
                    let vertex = if current.forward {
                        current_edge.end_vertex_id
                    } else {
                        current_edge.start_vertex_id
                    };
                    corners
                        .entry(vertex)
                        .or_default()
                        .push([current.edge_id, next.edge_id]);
                }
            }
        }
    }

    let mut pinched = Vec::new();
    let mut skipped = 0usize;
    for (vertex, list) in &corners {
        if degenerate_at.contains(vertex) {
            skipped += 1;
            continue;
        }
        if list.len() < 2 {
            continue;
        }
        let mut sets = DisjointSet::new(list.len());
        let mut by_edge: HashMap<u64, usize> = HashMap::default();
        for (index, corner) in list.iter().enumerate() {
            for edge_id in corner {
                match by_edge.entry(*edge_id) {
                    std::collections::hash_map::Entry::Occupied(slot) => {
                        sets.union(*slot.get(), index);
                    }
                    std::collections::hash_map::Entry::Vacant(slot) => {
                        slot.insert(index);
                    }
                }
            }
        }
        let fans: HashSet<usize> = (0..list.len()).map(|index| sets.find(index)).collect();
        if fans.len() > 1 {
            pinched.push(*vertex);
        }
    }
    pinched.sort_unstable();
    (pinched, skipped)
}

// ---------------------------------------------------------------------------
// Face-versus-face self-intersection
// ---------------------------------------------------------------------------

/// Fraction of the solid's bounding diagonal a confirmed crossing must
/// straddle the other carrier by. Tessellation vertices are exact on their own
/// surface, so this band only has to clear projection/Newton residue, not
/// chord error; it is deliberately far below any real penetration.
const STRADDLE_FRACTION: f64 = 1e-6;

/// Fraction of the diagonal a mesh crossing segment must be longer than. A
/// crossing shorter than this is a corner graze, not a penetration.
const CROSSING_FRACTION: f64 = 1e-7;

/// Fraction of the diagonal used to weld coincident tessellation vertices.
/// Shared-edge samples are pinned to identical values by the watertight
/// tessellator, so this only has to survive being read back out of the mesh
/// buffers.
const WELD_FRACTION: f64 = 1e-9;

/// How far from PARALLEL two surface normals must be for a same-face crossing
/// to be a fold. A degenerate parameterization — a sphere's pole, a seam the
/// period reduction did not fold away — carries distinct uv to one point with
/// one tangent plane, and this is what separates that from a surface genuinely
/// passing through itself. The two mesh triangles already crossed
/// TRANSVERSALLY (a coplanar pair returns no crossing), so a fold clears this
/// by orders of magnitude and the constant is a floor, not a threshold.
const FOLD_TRANSVERSAL_SINE: f64 = 1e-6;

/// `BREP_FOLD_TRACE=1` prints why each same-face candidate was confirmed,
/// rejected or left undecided, with the numbers behind the verdict. The
/// attribution a population sweep needs — "which rule turned this hit away,
/// and by how much" — cannot be reconstructed from the counts.
fn fold_trace_enabled() -> bool {
    std::env::var("BREP_FOLD_TRACE").is_ok_and(|value| !value.is_empty() && value != "0")
}

macro_rules! fold_trace {
    ($($arg:tt)*) => {
        if fold_trace_enabled() {
            eprintln!($($arg)*);
        }
    };
}

/// `BREP_PIERCE_TRACE=1` prints every face-PAIR candidate the corner lane
/// declined and each step of the pierce lane's answer for it: which edge
/// pierced which triangle, the two bracket distances, and where the two
/// samples across the crossing landed and what they read. The fold trace's
/// counterpart for the other half of the scan. It is asked several times for
/// every pair the corner lane declines, so it is one `OnceLock` read rather
/// than an environment lookup each time.
fn pierce_trace_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| {
        std::env::var("BREP_PIERCE_TRACE").is_ok_and(|value| !value.is_empty() && value != "0")
    })
}

macro_rules! pierce_trace {
    ($($arg:tt)*) => {
        if pierce_trace_enabled() {
            eprintln!($($arg)*);
        }
    };
}

/// Knobs for [`solid_self_intersections`]. [`SelfIntersectionOptions::for_solid`]
/// derives every one from the solid's own size.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct SelfIntersectionOptions {
    /// Tessellation density. The detector resolves nothing finer than this.
    pub chord_tolerance: f64,
    /// Signed-distance margin a confirmed crossing must straddle the other
    /// carrier surface by, on BOTH sides.
    pub straddle_band: f64,
    /// Shortest mesh crossing segment that counts as a candidate.
    pub min_crossing_length: f64,
    /// Coincidence radius for welding tessellation vertices.
    pub weld: f64,
    /// Refuse to scan a mesh larger than this (the report comes back
    /// `truncated`), so a pathological solid cannot hang a gate.
    pub max_triangles: usize,
}

impl SelfIntersectionOptions {
    /// Every band derived from the solid's own extent: the display chord
    /// tolerance the app meshes at, and diagonal-relative bands.
    pub fn for_solid(solid: &BrepSolid) -> Self {
        let diagonal = solid_diagonal(solid).max(1e-9);
        Self {
            chord_tolerance: crate::display_chord_tolerance(solid, 1.0),
            straddle_band: diagonal * STRADDLE_FRACTION,
            min_crossing_length: diagonal * CROSSING_FRACTION,
            weld: diagonal * WELD_FRACTION,
            max_triangles: 400_000,
        }
    }
}

/// One confirmed face-versus-face crossing.
#[derive(Clone, Debug, Serialize)]
pub struct FaceCrossing {
    pub face_a: u64,
    pub face_b: u64,
    pub name_a: Option<String>,
    pub name_b: Option<String>,
    /// Midpoint of the mesh crossing segment.
    pub point: Vec3,
    /// Length of the mesh crossing segment.
    pub crossing_length: f64,
    /// How far the confirmation straddled the other carrier: the smaller of
    /// the two one-sided margins, over both directions of the pair.
    pub straddle: f64,
    /// WHICH confirmation convicted this pair. Recorded so a sweep can
    /// attribute every hit to the question that caught it — the two ask the
    /// same thing of different sample points, and a hit only the pierce lane
    /// sees is a hit the corner lane structurally could not.
    pub confirmation: CrossingConfirmation,
    /// How deep one face's own SURFACE reaches past the other's carrier, at
    /// the crossing — the pierce lane's second margin, and the one that says
    /// whether the crossing is the geometry's or the mesh's.
    ///
    /// The lane's first margin brackets the pierce along a mesh EDGE, so on a
    /// regularly tessellated carrier it reads the same number for every pair
    /// in a ring and says more about the tessellation's stride than about the
    /// shapes. This one is measured between exact surface points and is the
    /// discriminator. `straddle` is the smaller of the two; for a `Corners`
    /// confirmation this is 0.0 and means nothing.
    pub penetration: f64,
}

/// One confirmed place a face FOLDS THROUGH ITSELF in 3D: its own surface
/// carries two DISTINCT trimmed parameter feet to ONE point in space.
///
/// This is the defect a [`FaceCrossing`] structurally cannot hold. A crossing
/// is between two faces and is confirmed by a signed straddle of the OTHER
/// carrier; with both triangles on ONE carrier every corner sits at distance
/// zero from it, so that test is degenerate by construction and the pair was
/// only ever counted. A fold is confirmed by a different question, asked in
/// parameter space where it is exact: are the two feet DISTINCT, are both
/// inside the face's own trim, and do they land on the same 3D point?
#[derive(Clone, Debug, Serialize)]
pub struct FaceFold {
    pub face: u64,
    pub face_name: Option<String>,
    /// The two parameter feet, each inside the face's own trim.
    pub uv_a: [f64; 2],
    pub uv_b: [f64; 2],
    /// Where they meet — the midpoint of the two surface images.
    pub point: Vec3,
    /// Parameter distance between the feet, measured the SHORT way round every
    /// closed direction, so a seam is never mistaken for a separation.
    pub uv_separation: f64,
    /// The parameter distance at which two feet count as ONE: half a cell of
    /// the face's own sample grid, which is built at the tessellation's own
    /// resolution. Two seeds closer than this refine to the same foot — which
    /// is what a chord-error crossing on a sharply curved face produces — and
    /// a face that ends with one foot is rejected.
    pub uv_band: f64,
    /// `|S(uv_a) - S(uv_b)|`: how coincident in 3D the two feet actually are.
    /// Bounded by the mesh's own chord tolerance, and recorded so a reader can
    /// see the margin rather than trust the verdict.
    pub residual: f64,
    /// Angle between the two surface normals, in radians. A genuine fold
    /// crosses TRANSVERSALLY. A degenerate parameterization — a pole, a seam
    /// the period reduction did not fold away — also carries distinct uv to
    /// one point, but with ONE tangent plane; that is rejected here rather
    /// than reported as a fold.
    pub normal_angle: f64,
}

/// Outcome of [`solid_self_intersections`].
#[derive(Clone, Debug, Serialize)]
pub struct SelfIntersectionReport {
    pub triangles: usize,
    /// Triangle pairs whose boxes overlapped and that survived the welded-
    /// vertex skip — the work the exact confirmation was offered.
    pub candidate_pairs: usize,
    /// Mesh crossings between DISTINCT faces that the exact-surface straddle
    /// test confirmed, one entry per face pair (deepest kept).
    pub confirmed: Vec<FaceCrossing>,
    /// Mesh crossings between distinct faces the straddle test REJECTED —
    /// tangent neighbours and chord error. A large number here beside zero
    /// confirmations is the detector working, not failing.
    pub rejected: usize,
    /// Mesh crossings whose confirmation could not be evaluated (projection or
    /// normal failed). Never counted as either verdict.
    pub undecided: usize,
    /// Mesh crossings between two triangles of ONE face: the candidates the
    /// fold confirmation was offered. A candidate is not a defect — a sharply
    /// curved face's chords cross each other without the face folding.
    pub same_face_candidates: usize,
    /// Faces confirmed to FOLD THROUGH THEMSELVES in 3D, one entry per FACE
    /// (the widest-separated hit kept). The face is the stable unit: a fold
    /// region produces as many triangle pairs as the mesh happens to have
    /// there, and a denser mesh would rewrite that number without the face
    /// being any more or less folded.
    pub folds: Vec<FaceFold>,
    /// Same-face candidates the fold confirmation REJECTED — the two feet came
    /// back within one tessellation cell of each other (chord error on a tight
    /// curve), or their normals were parallel (a pole). A large number here
    /// beside zero folds is the confirmation working, not failing.
    pub fold_rejected: usize,
    /// Same-face candidates whose confirmation could not be evaluated: a
    /// projection, a normal or a containment query failed, or a corner did not
    /// come back onto its own carrier. Never counted as either verdict, and a
    /// caller must not read an empty `folds` beside a large one of these as
    /// "no folds".
    pub fold_undecided: usize,
    /// The mesh exceeded `max_triangles` and was not scanned.
    pub truncated: bool,
    pub options: SelfIntersectionOptions,
}

impl SelfIntersectionReport {
    /// The predicate for a caller that has opted into this check: the solid
    /// passes through itself — two faces crossing, or ONE face folded through
    /// itself. Both are the same defect to a caller deciding whether to accept
    /// a result, so both are in the predicate.
    pub fn is_flagged(&self) -> bool {
        !self.confirmed.is_empty() || !self.folds.is_empty()
    }

    /// One line naming the worst crossing or fold, or `None` when nothing was
    /// confirmed.
    pub fn summary(&self) -> Option<String> {
        if self.confirmed.is_empty() {
            return self.fold_summary();
        }
        let worst = self
            .confirmed
            .iter()
            .max_by(|a, b| a.straddle.total_cmp(&b.straddle))?;
        Some(format!(
            "{} face pair(s) self-intersect; worst faces {} and {} near ({:.4}, {:.4}, {:.4}) (crossing {:.3e}, straddle {:.3e})",
            self.confirmed.len(),
            worst.face_a,
            worst.face_b,
            worst.point.x,
            worst.point.y,
            worst.point.z,
            worst.crossing_length,
            worst.straddle
        ))
    }

    /// One line naming the worst FOLD, or `None` when no face folded.
    pub fn fold_summary(&self) -> Option<String> {
        let worst = self
            .folds
            .iter()
            .max_by(|a, b| a.uv_separation.total_cmp(&b.uv_separation))?;
        Some(format!(
            "{} face(s) fold through themselves; worst face {}{} carries uv ({:.6}, {:.6}) and \
             ({:.6}, {:.6}) to ({:.4}, {:.4}, {:.4}) (separation {:.3e} vs band {:.3e}, \
             residual {:.3e}, normals {:.1}°)",
            self.folds.len(),
            worst.face,
            worst
                .face_name
                .as_ref()
                .map(|name| format!(" '{name}'"))
                .unwrap_or_default(),
            worst.uv_a[0],
            worst.uv_a[1],
            worst.uv_b[0],
            worst.uv_b[1],
            worst.point.x,
            worst.point.y,
            worst.point.z,
            worst.uv_separation,
            worst.uv_band,
            worst.residual,
            worst.normal_angle.to_degrees()
        ))
    }
}

fn solid_diagonal(solid: &BrepSolid) -> f64 {
    let mut bounds = Aabb::empty();
    for vertex in &solid.vertices {
        bounds.include_point(vertex.point);
    }
    let diagonal = bounds.diagonal();
    if diagonal.is_finite() && diagonal > 0.0 {
        return diagonal;
    }
    // Vertex-free solids (a full sphere, a full torus) fall back to the
    // control hull, exactly as the display tolerance does.
    let mut hull = Aabb::empty();
    for shell in &solid.shells {
        for face in &shell.faces {
            if let Ok(box_of) = Aabb::from_surface_controls(&face.surface) {
                hull.include(box_of);
            }
        }
    }
    let diagonal = hull.diagonal();
    if diagonal.is_finite() {
        diagonal
    } else {
        0.0
    }
}

/// Does this solid pass through itself? Tessellates every face once, walks a
/// triangle BVH for mesh crossings, and confirms each candidate against the
/// two carrier SURFACES so tangent neighbours and chord error cannot
/// manufacture a finding. See the module docs for the design and its one
/// documented blind spot.
///
/// Nothing in the kernel's build path calls this — a caller asks for it and
/// reads [`SelfIntersectionReport::is_flagged`]. The case gate is that caller
/// and treats a flag as a failure unless its baseline already holds the same
/// face pair; see the module table.
pub fn solid_self_intersections(
    solid: &BrepSolid,
    options: SelfIntersectionOptions,
) -> Result<SelfIntersectionReport, String> {
    let mut report = SelfIntersectionReport {
        triangles: 0,
        candidate_pairs: 0,
        confirmed: Vec::new(),
        rejected: 0,
        undecided: 0,
        same_face_candidates: 0,
        folds: Vec::new(),
        fold_rejected: 0,
        fold_undecided: 0,
        truncated: false,
        options,
    };
    if !(options.chord_tolerance > 0.0) || !options.chord_tolerance.is_finite() {
        return Err("solid_self_intersections: chord tolerance must be positive".into());
    }

    // The face-stride entry point rather than `tessellate_brep_watertight`:
    // the latter runs a coherent-orientation pass and a mesh validation that
    // can fail on exactly the broken solids this detector exists to inspect,
    // and winding is irrelevant to a crossing test.
    let mesh = crate::watertight_tessellation::tessellate_brep_watertight_face_stride(
        solid,
        options.chord_tolerance,
        1,
        0,
    )?;
    let triangle_count = mesh.indices.len() / 3;
    report.triangles = triangle_count;
    if triangle_count == 0 {
        return Ok(report);
    }
    if triangle_count > options.max_triangles {
        report.truncated = true;
        return Ok(report);
    }

    // Faces in the tessellator's sequential order, so a triangle's `face_ids`
    // entry indexes straight into this.
    let faces: Vec<&crate::topology::FaceRecord> = solid
        .shells
        .iter()
        .flat_map(|shell| shell.faces.iter())
        .collect();

    let point_of = |index: u32| -> Vec3 {
        let base = index as usize * 3;
        Vec3::new(
            mesh.positions[base],
            mesh.positions[base + 1],
            mesh.positions[base + 2],
        )
    };

    let welded = weld_positions(&mesh.positions, options.weld);

    let mut boxes = Vec::with_capacity(triangle_count);
    let mut corners: Vec<[Vec3; 3]> = Vec::with_capacity(triangle_count);
    for triangle in 0..triangle_count {
        let indices = [
            mesh.indices[triangle * 3],
            mesh.indices[triangle * 3 + 1],
            mesh.indices[triangle * 3 + 2],
        ];
        let points = [
            point_of(indices[0]),
            point_of(indices[1]),
            point_of(indices[2]),
        ];
        boxes.push(Aabb::from_points(points));
        corners.push(points);
    }
    let bvh = Bvh::build(&boxes);

    // Deepest confirmed crossing per unordered face pair, and the
    // widest-separated confirmed fold per face.
    let mut best: HashMap<(u64, u64), FaceCrossing> = HashMap::default();
    let mut folded: HashMap<u64, FaceFold> = HashMap::default();
    // Same-face mesh crossings, per face index, as `(crossing length, point)`.
    let mut same_face: HashMap<usize, Vec<(f64, Vec3)>> = HashMap::default();
    let mut hits = Vec::new();
    for first in 0..triangle_count {
        hits.clear();
        bvh.overlapping(boxes[first], 0.0, &mut hits);
        for second in hits.iter().copied() {
            if second <= first {
                continue;
            }
            let shares_vertex = (0..3).any(|a| {
                let wa = welded[mesh.indices[first * 3 + a] as usize];
                (0..3).any(|b| wa == welded[mesh.indices[second * 3 + b] as usize])
            });
            if shares_vertex {
                continue;
            }
            report.candidate_pairs += 1;
            let Some((start, end)) = triangle_crossing(&corners[first], &corners[second]) else {
                continue;
            };
            let length = end.sub(start).length();
            if length <= options.min_crossing_length {
                continue;
            }
            let face_first = mesh.face_ids[first] as usize;
            let face_second = mesh.face_ids[second] as usize;
            if face_first == face_second {
                report.same_face_candidates += 1;
                // Confirmed in a SECOND pass, per face: the fold test needs a
                // sample grid over the face's own domain, and building that
                // once per face is the difference between a cheap check and a
                // tessellation-sized one.
                same_face
                    .entry(face_first)
                    .or_insert_with(Vec::new)
                    .push((length, start.add(end).scale(0.5)));
                continue;
            }
            let (Some(face_a), Some(face_b)) = (faces.get(face_first), faces.get(face_second))
            else {
                report.undecided += 1;
                continue;
            };
            // TWO confirmations, asked in order, and the first one is
            // unchanged: a triangle's OWN CORNERS must straddle the other
            // carrier, in both directions.
            let forward = straddle(&face_b.surface, &corners[first], options.straddle_band);
            let backward = straddle(&face_a.surface, &corners[second], options.straddle_band);
            let corner_margin = match (forward, backward) {
                (Some(forward), Some(backward))
                    if forward > options.straddle_band && backward > options.straddle_band =>
                {
                    Some(forward.min(backward))
                }
                _ => None,
            };
            // The SECOND runs only when the first did not convict, and it is
            // the one a PLANAR face passing through a curved one needs: the
            // corner test reads a margin of zero there, because every corner
            // of the plane's few triangles is outside the cylinder its
            // INTERIOR passes through. See `pierce_straddle`.
            if corner_margin.is_none() {
                pierce_trace!(
                    "pair faces {}|{} triangles {first}|{second}: corners forward {forward:?} backward {backward:?}; crossing ({:.6}, {:.6}, {:.6})-({:.6}, {:.6}, {:.6}); triangle A {:?} triangle B {:?}",
                    face_a.id,
                    face_b.id,
                    start.x,
                    start.y,
                    start.z,
                    end.x,
                    end.y,
                    end.z,
                    corners[first].map(|corner| (corner.x, corner.y, corner.z)),
                    corners[second].map(|corner| (corner.x, corner.y, corner.z)),
                );
            }
            let (margin, confirmation, penetration) = match corner_margin {
                Some(margin) => (margin, CrossingConfirmation::Corners, 0.0),
                None => match pierce_straddle(
                    &corners[first],
                    &corners[second],
                    face_a,
                    face_b,
                    (start, end),
                    options.straddle_band,
                ) {
                    PierceVerdict::Confirmed(hit) => {
                        pierce_trace!(
                            " verdict: confirmed, bracket {:.6e} penetration {:.6e}",
                            hit.bracket,
                            hit.penetration
                        );
                        (hit.margin(), CrossingConfirmation::Pierce, hit.penetration)
                    }
                    verdict => {
                        // A projection or a normal that could not be evaluated
                        // is undecided in EITHER lane, never a rejection.
                        if forward.is_none()
                            || backward.is_none()
                            || matches!(verdict, PierceVerdict::Undecided)
                        {
                            pierce_trace!(" verdict: undecided");
                            report.undecided += 1;
                        } else {
                            pierce_trace!(" verdict: rejected");
                            report.rejected += 1;
                        }
                        continue;
                    }
                },
            };
            let key = if face_a.id <= face_b.id {
                (face_a.id, face_b.id)
            } else {
                (face_b.id, face_a.id)
            };
            let crossing = FaceCrossing {
                face_a: key.0,
                face_b: key.1,
                name_a: if face_a.id <= face_b.id {
                    face_a.name.clone()
                } else {
                    face_b.name.clone()
                },
                name_b: if face_a.id <= face_b.id {
                    face_b.name.clone()
                } else {
                    face_a.name.clone()
                },
                point: start.add(end).scale(0.5),
                crossing_length: length,
                straddle: margin,
                confirmation,
                penetration,
            };
            let slot = best.entry(key).or_insert_with(|| crossing.clone());
            if crossing.straddle > slot.straddle {
                *slot = crossing;
            }
        }
    }

    // ---- the fold pass ---------------------------------------------------
    // One grid per face that produced a same-face crossing at all, and at most
    // MAX_FOLD_PROBES points per face — the longest crossing segments, which
    // are the clearest evidence. A fold region produces as many candidates as
    // the mesh happens to have there, and confirming every one of them buys
    // nothing beyond the first.
    let mut triangles_on_face: HashMap<usize, usize> = HashMap::default();
    for face in mesh.face_ids.iter() {
        *triangles_on_face.entry(*face as usize).or_insert(0) += 1;
    }
    let mut face_order: Vec<usize> = same_face.keys().copied().collect();
    face_order.sort_unstable();
    for face_index in face_order {
        let Some(face) = faces.get(face_index) else {
            report.fold_undecided += same_face[&face_index].len();
            continue;
        };
        let mut points = same_face.remove(&face_index).unwrap_or_default();
        points.sort_by(|a, b| b.0.total_cmp(&a.0));
        points.truncate(MAX_FOLD_PROBES);
        let Some(grid) = fold_grid(
            &face.surface,
            triangles_on_face.get(&face_index).copied().unwrap_or(0),
        ) else {
            fold_trace!("fold face {}: the sample grid could not be built", face.id);
            report.fold_undecided += points.len();
            continue;
        };
        for (_, point) in points {
            match confirm_fold(face, &grid, point, options) {
                FoldVerdict::Confirmed(fold) => {
                    let slot = folded.entry(face.id).or_insert_with(|| fold.clone());
                    if fold.uv_separation > slot.uv_separation {
                        *slot = fold;
                    }
                }
                FoldVerdict::Rejected => report.fold_rejected += 1,
                FoldVerdict::Undecided => report.fold_undecided += 1,
            }
        }
    }

    report.confirmed = best.into_values().collect();
    report
        .confirmed
        .sort_by(|a, b| b.straddle.total_cmp(&a.straddle).then(a.face_a.cmp(&b.face_a)));
    report.folds = folded.into_values().collect();
    report
        .folds
        .sort_by(|a, b| b.uv_separation.total_cmp(&a.uv_separation).then(a.face.cmp(&b.face)));
    Ok(report)
}

/// Which of this solid's faces FOLD THROUGH THEMSELVES in 3D — a face whose
/// surface carries two distinct trimmed parameter feet to one point in space:
/// an offset taken past the source's curvature radius, a section swept through
/// a bend tighter than its own half-width, a carrier fitted through a
/// swallowtail.
///
/// The brief's named entry point, and one pass: it is
/// [`solid_self_intersections`] read for its fold half, because both verdicts
/// come off the SAME tessellation and paying for that twice would buy nothing.
/// A caller that wants both should call `solid_self_intersections` directly and
/// read `confirmed` and `folds` off the one report.
pub fn face_self_intersections(solid: &BrepSolid) -> Result<Vec<FaceFold>, String> {
    let options = SelfIntersectionOptions::for_solid(solid);
    Ok(solid_self_intersections(solid, options)?.folds)
}

/// Verdict of the fold confirmation for ONE same-face triangle pair.
enum FoldVerdict {
    Confirmed(FaceFold),
    Rejected,
    Undecided,
}


/// A coarse `(u, v) -> 3D` sample of ONE face's surface over its own domain,
/// at the resolution the tessellation already resolved the face at.
///
/// This is the fold test's instrument, and it exists because
/// `project_point_to_surface` cannot be one. A global projection onto a
/// FOLDED general surface is exactly the case its Newton is weakest at:
/// measured on the ribbon fixture, a point lying EXACTLY on the surface came
/// back with a foot 2.766 away from itself. The grid replaces that search with
/// an enumeration — every branch that comes near the point has a node near it
/// — and `project_point_to_surface_seeded` then refines each one WITHOUT
/// re-seeding globally, which is the same instrument the pcurve tracer uses to
/// stay on one branch of a surface that folds back.
struct FoldGrid {
    samples: Vec<(f64, f64, Vec3)>,
    /// Parameter distance at which two feet are ONE foot: half a cell. Two
    /// seeds inside it refine to the same place, which is exactly what chord
    /// error on a tightly curved face produces.
    merge: f64,
    /// The largest 3D step between neighbouring nodes, measured rather than
    /// assumed. A branch's nearest node can sit this far from the point, so it
    /// is how wide the seed window has to be to miss nothing.
    reach: f64,
}

/// At most this many crossing points are confirmed per face. A fold region
/// produces as many mesh candidates as the tessellation happens to have
/// there; the verdict is a property of the FACE, so the longest crossing
/// segments — the clearest evidence — are enough.
const MAX_FOLD_PROBES: usize = 64;

/// At most this many seeds are refined for one crossing point. A fold needs
/// TWO distinct feet; more than a handful of branches through one point is a
/// different shape than this detector claims.
const MAX_FOLD_SEEDS: usize = 16;

/// Build the sample grid for one face, at twice the tessellation's own linear
/// density (`triangles` is how many triangles the mesh gave this face), so the
/// grid resolves anything the mesh that produced the candidate could.
fn fold_grid(surface: &NurbsSurface, triangles: usize) -> Option<FoldGrid> {
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let steps = (((triangles.max(1) as f64).sqrt().ceil() as usize) * 2).clamp(24, 192);
    let node = |index: usize| -> (f64, f64) {
        let iu = index / (steps + 1);
        let iv = index % (steps + 1);
        (
            u0 + (u1 - u0) * iu as f64 / steps as f64,
            v0 + (v1 - v0) * iv as f64 / steps as f64,
        )
    };
    let mut samples = Vec::with_capacity((steps + 1) * (steps + 1));
    for index in 0..(steps + 1) * (steps + 1) {
        let (u, v) = node(index);
        samples.push((u, v, surface.evaluate(u, v).ok()?));
    }
    let mut reach = 0.0f64;
    for iu in 0..=steps {
        for iv in 0..=steps {
            let here = samples[iu * (steps + 1) + iv].2;
            if iu < steps {
                reach = reach.max(samples[(iu + 1) * (steps + 1) + iv].2.sub(here).length());
            }
            if iv < steps {
                reach = reach.max(samples[iu * (steps + 1) + iv + 1].2.sub(here).length());
            }
        }
    }
    let cell_u = (u1 - u0) / steps as f64;
    let cell_v = (v1 - v0) / steps as f64;
    Some(FoldGrid {
        samples,
        merge: 0.5 * (cell_u * cell_u + cell_v * cell_v).sqrt(),
        reach,
    })
}

/// The parameter distance between two feet, measured the SHORT way round every
/// closed direction.
fn parameter_separation(a: Vec2, b: Vec2, period: [Option<f64>; 2]) -> f64 {
    b.add(period_shift(b, a, period)).sub(a).length()
}

/// Are these two feet ONE locus reached from opposite ends of the domain — a
/// seam the surface carries but does not DECLARE?
///
/// `closed_directions()` is an exact test and an imported carrier can fail it
/// while still closing: measured on `abc_00000026.step`, a face whose two
/// u-boundary isocurves sit 2.5e-2 apart (on a model whose pcurves are known to
/// be ~3.5e-2 off their own edges) put one foot at u = 0 and the other at
/// u = 1, a full span apart, on what is one curve. The period reduction cannot
/// fold those together because the surface is not closed to the precision that
/// test demands.
///
/// So it is MEASURED: when the two feet sit within `window` of opposite ends of
/// a direction, the two boundary isocurves are sampled and compared, and they
/// are one locus when every sample agrees within `band`. `window` is the grid's
/// own resolution — a foot is "at the boundary" when the grid cannot place it
/// anywhere else — and `band` is the same coincidence band the feet themselves
/// had to clear.
fn boundary_identified(
    surface: &NurbsSurface,
    a: Vec2,
    b: Vec2,
    domain_u: [f64; 2],
    domain_v: [f64; 2],
    window: f64,
    band: f64,
) -> bool {
    const SAMPLES: usize = 8;
    let straddles = |first: f64, second: f64, low: f64, high: f64| {
        (first - low).abs() <= window && (second - high).abs() <= window
            || (first - high).abs() <= window && (second - low).abs() <= window
    };
    let isocurves_agree = |along_u: bool| {
        for index in 0..=SAMPLES {
            let fraction = index as f64 / SAMPLES as f64;
            let (first, second) = if along_u {
                let v = domain_v[0] + (domain_v[1] - domain_v[0]) * fraction;
                (
                    surface.evaluate(domain_u[0], v),
                    surface.evaluate(domain_u[1], v),
                )
            } else {
                let u = domain_u[0] + (domain_u[1] - domain_u[0]) * fraction;
                (
                    surface.evaluate(u, domain_v[0]),
                    surface.evaluate(u, domain_v[1]),
                )
            };
            let (Ok(first), Ok(second)) = (first, second) else {
                return false;
            };
            if first.sub(second).length() > band {
                return false;
            }
        }
        true
    };
    if straddles(a.x, b.x, domain_u[0], domain_u[1]) && isocurves_agree(true) {
        return true;
    }
    straddles(a.y, b.y, domain_v[0], domain_v[1]) && isocurves_agree(false)
}

/// Is this same-face mesh crossing a genuine 3D FOLD?
///
/// The face-versus-face straddle test cannot be asked here: both triangles lie
/// on one carrier, so every corner of both is at distance zero from the
/// surface it would be measured against. The question asked instead is the
/// parameter one, and it is the definition of a fold — the surface carries two
/// DISTINCT trimmed parameters to ONE point in space:
///
/// 1. **Enumerate the branches.** Every grid node within one cell's reach of
///    the crossing point is a seed ([`FoldGrid`] says why this is an
///    enumeration and not a search).
/// 2. **Refine each**, through the Newton that does NOT re-seed globally, and
///    keep the feet that actually reach the point within the mesh's own chord
///    band. Two seeds on ONE branch refine to one foot and are merged, which
///    is what chord error on a tightly curved face produces — so a face that
///    does not fold ends this step with a single foot and is rejected.
/// 3. **Both feet inside the trim.** An untrimmed carrier may fold wherever it
///    likes; only a fold the FACE carries is its defect.
/// 4. **The feet are TRANSVERSAL.** Distinct parameters carried to one point
///    with ONE tangent plane is a degenerate parameterization — a sphere's
///    pole, a seam — and not a fold.
///
/// The widest-separated qualifying pair is reported, with every margin it
/// cleared.
fn confirm_fold(
    face: &crate::topology::FaceRecord,
    grid: &FoldGrid,
    point: Vec3,
    options: SelfIntersectionOptions,
) -> FoldVerdict {
    let surface = &face.surface;
    let (Ok((closed_u, closed_v)), Ok(domain_u), Ok(domain_v)) = (
        surface.closed_directions(),
        surface.domain_u(),
        surface.domain_v(),
    ) else {
        fold_trace!("fold face {}: domain/closedness unreadable", face.id);
        return FoldVerdict::Undecided;
    };
    let span_u = domain_u[1] - domain_u[0];
    let span_v = domain_v[1] - domain_v[0];
    let period = [closed_u.then_some(span_u), closed_v.then_some(span_v)];
    // A tessellation vertex lies exactly on its surface and the crossing point
    // is within the chord tolerance of one, so this is the band every 3D
    // residual here is held to: the mesh's own accuracy.
    let coincidence = 2.0 * options.chord_tolerance;

    let window = grid.reach + coincidence;
    let mut seeds: Vec<(f64, usize)> = grid
        .samples
        .iter()
        .enumerate()
        .filter_map(|(index, (_, _, sample))| {
            let distance = sample.sub(point).length();
            (distance <= window).then_some((distance, index))
        })
        .collect();
    if seeds.is_empty() {
        fold_trace!(
            "fold face {}: no grid node within {window:.3e} of the crossing point",
            face.id
        );
        return FoldVerdict::Undecided;
    }
    seeds.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut feet: Vec<crate::projection::SurfaceProjection> = Vec::new();
    let mut used: Vec<Vec2> = Vec::new();
    for (_, index) in seeds {
        if feet.len() >= MAX_FOLD_SEEDS {
            break;
        }
        let (u, v, _) = grid.samples[index];
        let seed = Vec2 { x: u, y: v };
        if used
            .iter()
            .any(|taken| parameter_separation(*taken, seed, period) <= grid.merge)
        {
            continue;
        }
        used.push(seed);
        let Ok(foot) = project_point_to_surface_seeded(surface, point, u, v) else {
            continue;
        };
        if !(foot.distance <= coincidence) {
            continue;
        }
        let landed = Vec2 {
            x: foot.u,
            y: foot.v,
        };
        if feet.iter().any(|other| {
            parameter_separation(Vec2 { x: other.u, y: other.v }, landed, period) <= grid.merge
        }) {
            continue;
        }
        feet.push(foot);
    }
    if feet.len() < 2 {
        fold_trace!(
            "fold face {}: {} distinct foot/feet at the crossing point (merge {:.3e}) — one foot is not a fold",
            face.id,
            feet.len(),
            grid.merge
        );
        return FoldVerdict::Rejected;
    }

    // The uv coincidence band the trim is asked with, keyed to the face's own
    // parameter extent exactly as `loop_self_crossings` keys its weld: uv is
    // not a length, and a domain is as likely to be [0, 1] as [0, 2*PI]. A foot
    // ON the trim boundary counts as inside — a fold that reaches the boundary
    // is still a fold.
    let trim_band = UV_WELD_FRACTION * (span_u.abs() + span_v.abs()).max(1.0);
    let mut inside: Vec<(&crate::projection::SurfaceProjection, Vec3)> = Vec::new();
    for foot in &feet {
        let uv = Vec2 {
            x: foot.u,
            y: foot.v,
        };
        match parameter_point_in_face(face, uv, trim_band) {
            Ok(PolygonClass::Outside) => continue,
            Ok(_) => {}
            Err(message) => {
                fold_trace!("fold face {}: containment: {message}", face.id);
                return FoldVerdict::Undecided;
            }
        }
        let Ok(normal) = surface.normal(foot.u, foot.v).and_then(|normal| normal.normalized())
        else {
            fold_trace!("fold face {}: a normal could not be evaluated", face.id);
            return FoldVerdict::Undecided;
        };
        inside.push((foot, normal));
    }
    if inside.len() < 2 {
        fold_trace!(
            "fold face {}: {} of {} feet are inside the trim",
            face.id,
            inside.len(),
            feet.len()
        );
        return FoldVerdict::Rejected;
    }

    let mut chosen: Option<(f64, usize, usize, f64)> = None;
    let mut worst_sine = 0.0f64;
    let mut identified = 0usize;
    for first in 0..inside.len() {
        for second in (first + 1)..inside.len() {
            let sine = inside[first].1.cross(inside[second].1).length();
            worst_sine = worst_sine.max(sine);
            if sine <= FOLD_TRANSVERSAL_SINE {
                continue;
            }
            // A SEAM the surface does not declare. `closed_directions()` is an
            // exact test, and an imported carrier whose two boundary isocurves
            // are 2.5e-2 apart fails it while still being one locus reached
            // from both ends of the domain — so the period reduction above did
            // not fold these two feet together and they read a full span
            // apart. Measured rather than assumed: the two isocurves are
            // sampled and compared.
            if boundary_identified(
                surface,
                Vec2 { x: inside[first].0.u, y: inside[first].0.v },
                Vec2 { x: inside[second].0.u, y: inside[second].0.v },
                domain_u,
                domain_v,
                6.0 * grid.merge,
                coincidence,
            ) {
                identified += 1;
                continue;
            }
            let separation = parameter_separation(
                Vec2 { x: inside[first].0.u, y: inside[first].0.v },
                Vec2 { x: inside[second].0.u, y: inside[second].0.v },
                period,
            );
            if chosen.is_none_or(|(best, _, _, _)| separation > best) {
                chosen = Some((separation, first, second, sine));
            }
        }
    }
    let Some((separation, first, second, sine)) = chosen else {
        fold_trace!(
            "fold face {}: no pair of feet is a fold (widest sine {worst_sine:.3e}, {identified} pair(s) are one locus across an undeclared seam) — a degenerate parameterization, not a fold",
            face.id
        );
        return FoldVerdict::Rejected;
    };
    let (foot_a, normal_a) = inside[first];
    let (foot_b, normal_b) = inside[second];
    let angle = sine.atan2(normal_a.dot(normal_b));
    fold_trace!(
        "fold face {}: CONFIRMED feet ({:.6}, {:.6}) and ({:.6}, {:.6}) separation {separation:.3e} vs merge {:.3e} residual {:.3e} (feet {:.3e}/{:.3e} from the point, band {coincidence:.3e}) normals {:.2}deg",
        face.id,
        foot_a.u,
        foot_a.v,
        foot_b.u,
        foot_b.v,
        grid.merge,
        foot_a.point.sub(foot_b.point).length(),
        foot_a.distance,
        foot_b.distance,
        angle.to_degrees()
    );

    FoldVerdict::Confirmed(FaceFold {
        face: face.id,
        face_name: face.name.clone(),
        uv_a: [foot_a.u, foot_a.v],
        uv_b: [foot_b.u, foot_b.v],
        point: foot_a.point.add(foot_b.point).scale(0.5),
        uv_separation: separation,
        uv_band: grid.merge,
        residual: foot_a.point.sub(foot_b.point).length(),
        normal_angle: angle,
    })
}

/// Map every mesh vertex to a representative index, welding positions that
/// coincide within `weld`. Bucketed on a `weld`-sized lattice with the 27
/// neighbouring cells consulted, so a pair straddling a cell boundary still
/// welds.
fn weld_positions(positions: &[f64], weld: f64) -> Vec<u32> {
    let count = positions.len() / 3;
    let mut representative = vec![0u32; count];
    let cell = weld.max(f64::MIN_POSITIVE);
    let mut buckets: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::default();
    for index in 0..count {
        let base = index * 3;
        let point = Vec3::new(positions[base], positions[base + 1], positions[base + 2]);
        let key = (
            (point.x / cell).floor() as i64,
            (point.y / cell).floor() as i64,
            (point.z / cell).floor() as i64,
        );
        let mut found = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(list) = buckets.get(&(key.0 + dx, key.1 + dy, key.2 + dz)) else {
                        continue;
                    };
                    for candidate in list {
                        let base = *candidate as usize * 3;
                        let other = Vec3::new(
                            positions[base],
                            positions[base + 1],
                            positions[base + 2],
                        );
                        if other.sub(point).length() <= weld {
                            found = Some(representative[*candidate as usize]);
                            break 'search;
                        }
                    }
                }
            }
        }
        let value = found.unwrap_or(index as u32);
        representative[index] = value;
        buckets.entry(key).or_default().push(index as u32);
    }
    representative
}

/// Signed distances of a triangle's corners to `surface`, reduced to the
/// smaller of the two one-sided margins: positive means the corners genuinely
/// sit on both sides of the carrier by at least that much, zero means they do
/// not straddle it at all. `None` when a projection or a normal could not be
/// evaluated — never a verdict.
///
/// Three projections, and only for a triangle pair that already survived the
/// BVH, the welded-vertex skip and the mesh crossing test.
fn straddle(surface: &NurbsSurface, triangle: &[Vec3; 3], band: f64) -> Option<f64> {
    let mut above = 0.0f64;
    let mut below = 0.0f64;
    for corner in triangle {
        let projection = project_point_to_surface(surface, *corner).ok()?;
        if projection.distance <= band {
            continue;
        }
        let normal = surface.normal(projection.u, projection.v).ok()?;
        let normal = normal.normalized().ok()?;
        let signed = corner.sub(projection.point).dot(normal);
        if signed > 0.0 {
            above = above.max(signed);
        } else {
            below = below.max(-signed);
        }
    }
    Some(above.min(below))
}

/// Which confirmation convicted a [`FaceCrossing`].
///
/// Recorded so a sweep can attribute every hit to the question that caught it.
/// The two ask the same thing — does one face's surface sit on BOTH sides of
/// the other's carrier — of different sample points, and the sample points are
/// the whole difference between them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum CrossingConfirmation {
    /// [`straddle`]: the triangles' OWN CORNERS, in both directions.
    Corners,
    /// [`pierce_straddle`]: an edge of one triangle through the other's
    /// interior, with the samples taken at the pierce.
    Pierce,
}

/// Signed distance from `point` to `surface`, positive on the side the
/// surface's normal points to. `None` when the projection or the normal could
/// not be evaluated — never a verdict.
fn signed_distance(surface: &NurbsSurface, point: Vec3) -> Option<f64> {
    let projection = project_point_to_surface(surface, point).ok()?;
    let normal = surface.normal(projection.u, projection.v).ok()?;
    let normal = normal.normalized().ok()?;
    Some(point.sub(projection.point).dot(normal))
}

/// The SECOND confirmation, and the one a PLANAR face passing through a curved
/// one needs.
///
/// [`straddle`] reads a triangle's OWN CORNERS, and a planar face is
/// tessellated as the two triangles its rectangle needs. When such a face
/// passes through a cylinder every one of those corners is OUTSIDE the
/// cylinder, so the corner test reads a one-sided margin of zero and REJECTS a
/// body that demonstrably passes through itself: a 20-block whose `-X` wall is
/// pushed to `x = 6` through its own `r = 6` bore encloses 1669.026644711,
/// which is exactly the closed form that removes the bore's segment TWICE.
///
/// The same question is asked where the two triangles actually meet instead:
///
/// 1. An EDGE of one triangle passes through the INTERIOR of the other
///    (segment-triangle intersection, with a barycentric margin, so two
///    triangles meeting along a shared boundary are never a pierce).
/// 2. That edge's two ENDPOINTS straddle the pierced face's carrier — the same
///    signed-distance question [`straddle`] asks of a triangle's corners,
///    restricted to the two points that bracket the pierce.
/// 3. Two points ON the pierced triangle, either side of the two triangles'
///    crossing line and inside that triangle, straddle the PIERCING face's
///    carrier. This is the half the corner test cannot reach, and it is what
///    separates a face passing THROUGH another from one merely touching it: at
///    a tangency both samples sit on the SAME side, however deep the chords
///    interpenetrate.
///
/// Returns the smallest margin the confirmation cleared, or `None` when
/// neither direction confirmed.
fn pierce_straddle(
    first: &[Vec3; 3],
    second: &[Vec3; 3],
    face_first: &FaceRecord,
    face_second: &FaceRecord,
    crossing: (Vec3, Vec3),
    band: f64,
) -> PierceVerdict {
    pierce_trace!(" forward: an edge of the first triangle through the second");
    let forward = pierce_direction(first, second, face_first, face_second, crossing, band);
    pierce_trace!(" backward: an edge of the second triangle through the first");
    let backward = pierce_direction(second, first, face_second, face_first, crossing, band);
    match (forward, backward) {
        (PierceVerdict::Confirmed(a), PierceVerdict::Confirmed(b)) => {
            PierceVerdict::Confirmed(if a.margin() >= b.margin() { a } else { b })
        }
        (PierceVerdict::Confirmed(hit), _) | (_, PierceVerdict::Confirmed(hit)) => {
            PierceVerdict::Confirmed(hit)
        }
        // A projection or a normal that could not be evaluated is never a
        // verdict — the module's rule, and the same one the corner lane keeps.
        (PierceVerdict::Undecided, _) | (_, PierceVerdict::Undecided) => PierceVerdict::Undecided,
        _ => PierceVerdict::Rejected,
    }
}

/// The pierce lane's two margins, both measured between EXACT surface points.
#[derive(Clone, Copy, Debug)]
struct PierceHit {
    /// How far the piercing mesh edge's two endpoints sit either side of the
    /// pierced face's carrier.
    bracket: f64,
    /// How far the pierced face's own SURFACE reaches either side of the
    /// piercing face's carrier, across the crossing line.
    penetration: f64,
}

impl PierceHit {
    fn margin(&self) -> f64 {
        self.bracket.min(self.penetration)
    }
}

enum PierceVerdict {
    Confirmed(PierceHit),
    Rejected,
    Undecided,
}

/// One direction of [`pierce_straddle`]: an edge of `piercing` through
/// `pierced`'s interior. The widest margin over the three edges.
fn pierce_direction(
    piercing: &[Vec3; 3],
    pierced: &[Vec3; 3],
    face_piercing: &FaceRecord,
    face_pierced: &FaceRecord,
    crossing: (Vec3, Vec3),
    band: f64,
) -> PierceVerdict {
    let (surface_piercing, surface_pierced) = (&face_piercing.surface, &face_pierced.surface);
    let mut best: Option<PierceHit> = None;
    let mut undecided = false;
    for index in 0..3 {
        let p0 = piercing[index];
        let p1 = piercing[(index + 1) % 3];
        let Some(fraction) = segment_pierces_triangle(p0, p1, pierced) else {
            pierce_trace!("  edge {index}: does not pierce the other triangle's interior");
            continue;
        };
        let point = p0.add(p1.sub(p0).scale(fraction));
        pierce_trace!(
            "  edge {index}: pierces at ({:.6}, {:.6}, {:.6}); bracket distances {:?} {:?}",
            point.x,
            point.y,
            point.z,
            signed_distance(surface_pierced, p0),
            signed_distance(surface_pierced, p1)
        );
        let bracket = match bracket_straddle(surface_pierced, p0, p1, band) {
            Margin::Cleared(value) => value,
            Margin::Failed => continue,
            Margin::Unevaluated => {
                undecided = true;
                continue;
            }
        };
        let penetration = match across_crossing(
            surface_piercing,
            face_pierced,
            pierced,
            point,
            crossing,
            band,
        ) {
            Margin::Cleared(value) => value,
            Margin::Failed => continue,
            Margin::Unevaluated => {
                undecided = true;
                continue;
            }
        };
        let hit = PierceHit {
            bracket,
            penetration,
        };
        if best.is_none_or(|value| hit.margin() > value.margin()) {
            best = Some(hit);
        }
    }
    match best {
        Some(hit) => PierceVerdict::Confirmed(hit),
        None if undecided => PierceVerdict::Undecided,
        None => PierceVerdict::Rejected,
    }
}

/// One margin the pierce lane asked for: cleared with a value, genuinely not
/// cleared, or not evaluable at all. The third is never a verdict.
enum Margin {
    Cleared(f64),
    Failed,
    Unevaluated,
}

/// Where the segment `p0 -> p1` passes through `triangle`'s INTERIOR, as the
/// fraction along the segment. `None` when it misses, when it is parallel to
/// the triangle's plane, or when it crosses within `INTERIOR_MARGIN` of the
/// triangle's boundary — a hit on the boundary is two triangles MEETING, which
/// every watertight tessellation does along every edge.
fn segment_pierces_triangle(p0: Vec3, p1: Vec3, triangle: &[Vec3; 3]) -> Option<f64> {
    /// Barycentric margin, as a fraction of the triangle.
    const INTERIOR_MARGIN: f64 = 1e-6;
    let edge_one = triangle[1].sub(triangle[0]);
    let edge_two = triangle[2].sub(triangle[0]);
    let direction = p1.sub(p0);
    let cross = direction.cross(edge_two);
    let determinant = edge_one.dot(cross);
    if determinant.abs() <= 1e-18 {
        return None; // parallel to the triangle's plane
    }
    let inverse = 1.0 / determinant;
    let to_corner = p0.sub(triangle[0]);
    let u = to_corner.dot(cross) * inverse;
    let second_cross = to_corner.cross(edge_one);
    let v = direction.dot(second_cross) * inverse;
    if u <= INTERIOR_MARGIN || v <= INTERIOR_MARGIN || u + v >= 1.0 - INTERIOR_MARGIN {
        return None;
    }
    let fraction = edge_two.dot(second_cross) * inverse;
    if fraction <= 0.0 || fraction >= 1.0 {
        return None;
    }
    Some(fraction)
}

/// Do `p0` and `p1` sit on OPPOSITE sides of `surface`, each by more than
/// `band`? The smaller of the two margins, or `None`.
fn bracket_straddle(surface: &NurbsSurface, p0: Vec3, p1: Vec3, band: f64) -> Margin {
    let (Some(first), Some(second)) = (
        signed_distance(surface, p0),
        signed_distance(surface, p1),
    ) else {
        return Margin::Unevaluated;
    };
    if first.abs() <= band || second.abs() <= band || (first > 0.0) == (second > 0.0) {
        return Margin::Failed;
    }
    Margin::Cleared(first.abs().min(second.abs()))
}

/// The boundary band [`parameter_point_in_face`] is asked with here, so that a
/// sample sitting ON the trim reads as the face's material rather than outside
/// it. The comparand is the same containment band the imprint's own window
/// guard uses (`restricted_carrier`'s wrap sample, `1e-9`) — the tightest in
/// the kernel, since the question is "is this spot the face's at all", not "how
/// far may it stray". The overshoot this test exists to catch is four orders
/// wider (2e-5 on the helmet's turning point).
const TRIM_SAMPLE_TOLERANCE: f64 = 1e-9;

/// `BREP_PIERCE_TRIM_SAMPLES=0` restores the pre-2026-09-16 verdict, where an
/// across-sample was taken from the triangle without asking the face whether
/// it owns that spot.
fn trim_samples_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("BREP_PIERCE_TRIM_SAMPLES").as_deref() != Ok("0"))
}

/// Does `triangle` sit on BOTH sides of `surface`, either side of the crossing
/// line through `point`?
///
/// The two samples are taken on the triangle, stepping perpendicular to the
/// crossing line, half way to the triangle's boundary in each direction, and
/// then put back onto the face's OWN carrier so the margin is a property of
/// the two surfaces rather than of the mesh. A face passing THROUGH the
/// carrier puts them on opposite sides; a face TOUCHING it puts both on the
/// same one, which is the rejection this test exists to make.
///
/// A TRIANGLE IS NOT THE FACE'S MATERIAL. Its corners lie on the trim, but the
/// boundary is approximated by chords, and where the boundary turns sharply
/// the chord bows to the outside: the triangle then covers a sliver the face
/// does not own. Measured on `20_helmet_merge_stage` b − a, at the turning
/// point of the helmet `Face_15` × cutter-plane section, a boundary chord of
/// 6.4e-4 stood 2e-5 outside its own trim, and a sample there read 1.72e-6
/// through the other carrier against a band of 2.2e-7: a confirmed crossing on
/// geometry no face carries. So each sample is asked of the pierced face's own
/// trim, and a sample outside it leaves the pair UNEVALUATED — never rejected,
/// because a genuine crossing whose sample happened to land outside would then
/// read clean. Escape hatch `BREP_PIERCE_TRIM_SAMPLES=0`.
fn across_crossing(
    surface: &NurbsSurface,
    own_face: &FaceRecord,
    triangle: &[Vec3; 3],
    point: Vec3,
    crossing: (Vec3, Vec3),
    band: f64,
) -> Margin {
    let own = &own_face.surface;
    let along = crossing.1.sub(crossing.0);
    let (Ok(along), Some((normal, _))) = (along.normalized(), triangle_plane(triangle)) else {
        return Margin::Unevaluated;
    };
    let Ok(across) = normal.cross(along).normalized() else {
        return Margin::Unevaluated;
    };
    // How far the samples may step before leaving the triangle: the three
    // inward half-planes of its own edges, clipped against the ray.
    let mut low = f64::NEG_INFINITY;
    let mut high = f64::INFINITY;
    for index in 0..3 {
        let a = triangle[index];
        let b = triangle[(index + 1) % 3];
        let opposite = triangle[(index + 2) % 3];
        let mut inward = normal.cross(b.sub(a));
        if inward.dot(opposite.sub(a)) < 0.0 {
            inward = inward.scale(-1.0);
        }
        let slope = inward.dot(across);
        let offset = inward.dot(point.sub(a));
        if slope.abs() <= 1e-18 {
            if offset < 0.0 {
                return Margin::Failed;
            }
            continue;
        }
        let bound = -offset / slope;
        if slope > 0.0 {
            low = low.max(bound);
        } else {
            high = high.min(bound);
        }
    }
    if !(low < 0.0) || !(high > 0.0) || !low.is_finite() || !high.is_finite() {
        pierce_trace!("    across: no room either side of the crossing (reach {low:.6}..{high:.6})");
        return Margin::Failed;
    }
    // Both samples sit on the pierced TRIANGLE, which is a chord of its own
    // face and up to the tessellation's chord tolerance off the surface it
    // approximates — on an imported model that tolerance is 1.5e-3 of the
    // model's extent, orders above the band this margin has to clear. Put each
    // sample back ON its own carrier before asking the other surface about it,
    // so what is measured is a property of the two SURFACES rather than of how
    // coarsely one of them was meshed. On a planar face this is the identity.
    let hold_to_trim = trim_samples_enabled();
    let samples = [low * 0.5, high * 0.5].map(|step| {
        let projection = project_point_to_surface(own, point.add(across.scale(step))).ok();
        let sample = projection.as_ref().map(|projection| projection.point);
        let material = match (&projection, hold_to_trim) {
            (Some(projection), true) => !matches!(
                parameter_point_in_face(
                    own_face,
                    Vec2 { x: projection.u, y: projection.v },
                    TRIM_SAMPLE_TOLERANCE,
                ),
                Ok(PolygonClass::Outside)
            ),
            _ => true,
        };
        let reading = sample.and_then(|sample| signed_distance(surface, sample));
        if let Some(sample) = sample {
            pierce_trace!(
                "    across step {step:.6} (reach {low:.6}..{high:.6}): sample ({:.6}, {:.6}, {:.6}) reads {reading:?}{}",
                sample.x,
                sample.y,
                sample.z,
                if material { "" } else { " OUTSIDE its own trim" }
            );
        }
        material.then_some(reading).flatten()
    });
    let [Some(below), Some(above)] = samples else {
        return Margin::Unevaluated;
    };
    if below.abs() <= band || above.abs() <= band || (below > 0.0) == (above > 0.0) {
        return Margin::Failed;
    }
    Margin::Cleared(below.abs().min(above.abs()))
}

/// The segment along which two triangles cross, or `None` when they do not
/// cross transversally. Coplanar pairs return `None` — a coplanar overlap is a
/// different defect and this detector does not claim it.
fn triangle_crossing(first: &[Vec3; 3], second: &[Vec3; 3]) -> Option<(Vec3, Vec3)> {
    let plane_first = triangle_plane(first)?;
    let plane_second = triangle_plane(second)?;
    let direction = plane_first.0.cross(plane_second.0);
    let length = direction.length();
    if length <= 1e-12 {
        return None; // parallel or coplanar
    }
    let direction = direction.scale(1.0 / length);

    let first_span = plane_span(first, plane_second, direction)?;
    let second_span = plane_span(second, plane_first, direction)?;
    let low = first_span.0.max(second_span.0);
    let high = first_span.1.min(second_span.1);
    if high < low {
        return None;
    }
    // Rebuild 3D points by interpolating the first triangle's own crossing
    // chord, so the returned segment is on the mesh rather than reconstructed
    // from a line equation.
    let span = first_span.1 - first_span.0;
    let lerp = |value: f64| {
        if span <= 0.0 {
            first_span.2
        } else {
            let fraction = ((value - first_span.0) / span).clamp(0.0, 1.0);
            first_span
                .2
                .add(first_span.3.sub(first_span.2).scale(fraction))
        }
    };
    Some((lerp(low), lerp(high)))
}

/// Unit normal and plane offset of a triangle, `None` for a degenerate one.
fn triangle_plane(triangle: &[Vec3; 3]) -> Option<(Vec3, f64)> {
    let normal = triangle[1]
        .sub(triangle[0])
        .cross(triangle[2].sub(triangle[0]));
    let length = normal.length();
    if length <= 1e-18 {
        return None;
    }
    let normal = normal.scale(1.0 / length);
    Some((normal, -normal.dot(triangle[0])))
}

/// Where `triangle` crosses `plane`, as `(low, high, low_point, high_point)`
/// parametrized by `point · direction`. `None` when the triangle does not
/// reach the plane.
fn plane_span(
    triangle: &[Vec3; 3],
    plane: (Vec3, f64),
    direction: Vec3,
) -> Option<(f64, f64, Vec3, Vec3)> {
    let distance = |point: Vec3| plane.0.dot(point) + plane.1;
    let distances = [
        distance(triangle[0]),
        distance(triangle[1]),
        distance(triangle[2]),
    ];
    if distances.iter().all(|value| *value > 0.0) || distances.iter().all(|value| *value < 0.0) {
        return None;
    }
    let mut points: Vec<Vec3> = Vec::with_capacity(3);
    for index in 0..3 {
        let next = (index + 1) % 3;
        if distances[index] == 0.0 {
            points.push(triangle[index]);
        }
        if (distances[index] < 0.0) != (distances[next] < 0.0)
            && distances[index] != 0.0
            && distances[next] != 0.0
        {
            let fraction = distances[index] / (distances[index] - distances[next]);
            points.push(
                triangle[index].add(triangle[next].sub(triangle[index]).scale(fraction)),
            );
        }
    }
    if points.len() < 2 {
        return None;
    }
    let mut low = (f64::INFINITY, Vec3::default());
    let mut high = (f64::NEG_INFINITY, Vec3::default());
    for point in points {
        let value = point.dot(direction);
        if value < low.0 {
            low = (value, point);
        }
        if value > high.0 {
            high = (value, point);
        }
    }
    Some((low.0, high.0, low.1, high.1))
}


// ---------------------------------------------------------------------------
// Euler parity
// ---------------------------------------------------------------------------

/// Outcome of [`solid_euler`].
#[derive(Clone, Debug, Serialize)]
pub struct EulerReport {
    /// `V − E + F − H` over the reduced complex — the same count
    /// [`BrepSolid::validate`] makes, with degenerate (collapsed) cells left
    /// out of all four terms.
    pub characteristic: i64,
    pub shells: usize,
    /// `S − χ/2`, and `None` when `characteristic` is odd — there is no such
    /// surface, so there is no genus to report rather than a halved one.
    pub genus: Option<i64>,
    /// What the solid carries in its `genus` field.
    pub recorded_genus: i64,
    /// Non-degenerate edges used by FEWER than two coedges — a boundary edge,
    /// or an edge record no loop references at all. A closed surface has
    /// neither; a sheet or an in-progress shell legitimately has the first, and
    /// its characteristic is then unconstrained. Both are counted here because
    /// both mean the same thing for parity: these counts are not those of a
    /// closed surface, so decline to judge rather than convict.
    pub boundary_edges: usize,
    /// Non-degenerate edges used by three or more coedges — not a 2-manifold,
    /// so the characteristic means nothing either.
    pub non_manifold_edges: usize,
}

impl EulerReport {
    /// Whether the counts describe a closed 2-manifold at all. Parity is only
    /// an invariant for one of those.
    pub fn is_closed_manifold(&self) -> bool {
        self.boundary_edges == 0 && self.non_manifold_edges == 0
    }

    /// The HARD predicate: a CLOSED manifold whose Euler characteristic is odd.
    ///
    /// No closed orientable surface has one — χ = 2(S − G) is even by
    /// construction — so this is a statement about the record, not about a
    /// tolerance, and it cannot be tuned away. It is the one thing the Euler
    /// count can establish on its own: the genus itself has no second
    /// derivation from the cell complex to check against.
    pub fn is_unsound(&self) -> bool {
        self.is_closed_manifold() && self.characteristic % 2 != 0
    }

    /// One line naming what is wrong, or `None`.
    pub fn summary(&self) -> Option<String> {
        if self.is_unsound() {
            return Some(format!(
                "Euler characteristic {} is ODD over {} shell(s); no closed orientable \
                 surface has one, so these counts do not describe a solid \
                 (recorded genus {})",
                self.characteristic, self.shells, self.recorded_genus
            ));
        }
        None
    }
}

/// Closedness and Euler parity: `V − E + F − H` over the reduced complex, and
/// whether the counts can belong to a closed orientable surface at all.
///
/// Pure topology — ids and use counts, never a coordinate — so like
/// [`solid_connectivity`] it is tolerance-free and cannot be noisy.
///
/// This does NOT verify the genus. A genus has no second derivation from the
/// same cell complex: `genus = S − χ/2` IS the definition, so any recomputation
/// re-runs the same arithmetic. (The only independent route is geometric —
/// Gauss–Bonnet, `∮K dA` plus the edge and vertex terms — which is a different
/// project.) What parity establishes without a second source is that a body
/// whose characteristic is odd is not a closed orientable surface at all.
///
/// The characteristic is a TOTAL over shells, which is necessary but not
/// sufficient: two shells each odd sum to even and pass. Per-shell parity needs
/// edges attributed to shells, which `cross_shell_edges` makes ambiguous
/// exactly when it would matter.
pub fn solid_euler(solid: &BrepSolid) -> EulerReport {
    let degenerate: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| edge.degenerate)
        .map(|edge| edge.id)
        .collect();
    let mut uses: HashMap<u64, usize> = HashMap::default();
    for coedge in solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|record| &record.coedges)
    {
        if !degenerate.contains(&coedge.edge_id) {
            *uses.entry(coedge.edge_id).or_insert(0) += 1;
        }
    }
    let mut boundary_edges = 0usize;
    let mut non_manifold_edges = 0usize;
    for edge in solid.edges.iter().filter(|edge| !edge.degenerate) {
        match uses.get(&edge.id).copied().unwrap_or(0) {
            2 => {}
            0 | 1 => boundary_edges += 1,
            _ => non_manifold_edges += 1,
        }
    }

    let live: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate)
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let vertices = solid
        .vertices
        .iter()
        .filter(|vertex| live.contains(&vertex.id))
        .count() as i64;
    let edges = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let faces: i64 = solid.shells.iter().map(|shell| shell.faces.len() as i64).sum();
    let characteristic = vertices - edges + faces - solid.bounding_hole_count();
    let shells = solid.shells.len();
    EulerReport {
        characteristic,
        shells,
        genus: (characteristic % 2 == 0).then(|| shells as i64 - characteristic / 2),
        recorded_genus: solid.genus,
        boundary_edges,
        non_manifold_edges,
    }
}

// ---------------------------------------------------------------------------
// Face-loop self-crossing
// ---------------------------------------------------------------------------

/// Parametric exclusion at each chord's ends. A crossing this close to a
/// chord end is the shared vertex of two consecutive chords, not a crossing.
const CHORD_END_EXCLUSION: f64 = 1e-9;

/// Sine of the angle below which two chords are treated as PARALLEL and no
/// crossing is computed. Scale-free: it is compared against the cross product
/// divided by both lengths, so a long chord pair and a short one are held to
/// the same angle. Collinear overlap — the two seam coedges of a periodic
/// face, traversed once each way — lands here and is not a crossing.
const CHORD_PARALLEL_SINE: f64 = 1e-12;

/// Coincidence radius for chord ENDPOINTS, as a fraction of the face's own
/// parameter extent. Two chords whose ends meet share a vertex of the loop;
/// where that is a loop touching itself at a point it is a PINCH, which is
/// `ConnectivityReport::pinch_vertices`' question, not this one.
const UV_WELD_FRACTION: f64 = 1e-9;

/// One place a face's trim loop crosses ITSELF in the face's parameter domain.
///
/// The two coedges are the loop's own — a bowtie between two different trims,
/// or one pcurve crossing itself, in which case they are the same coedge.
#[derive(Clone, Debug, Serialize)]
pub struct LoopCrossing {
    pub face: u64,
    pub face_name: Option<String>,
    pub loop_id: u64,
    /// The two crossing coedges, in loop order.
    pub coedge_a: u64,
    pub coedge_b: u64,
    /// The edges those coedges use — what a caller NAMES the defect by, since
    /// an edge survives the surgery that renumbers coedges.
    pub edge_a: u64,
    pub edge_b: u64,
    /// Where they cross, in the face's own domain (wrapped into it on a
    /// periodic direction, so it is a parameter the surface can be evaluated
    /// at rather than a covering-plane lift).
    pub uv: [f64; 2],
    /// The surface point there — the 3D place the face passes through itself.
    pub point: Vec3,
}

/// Outcome of [`loop_self_crossings`].
#[derive(Clone, Debug, Serialize)]
pub struct LoopCrossingReport {
    /// Loops scanned.
    pub loops: usize,
    /// Chords tested — the sampled trim polygon's segments, summed over loops.
    pub chords: usize,
    /// One entry per crossing COEDGE PAIR per loop, first one found kept. The
    /// pair is the stable half: a denser sampling moves the crossing's `uv`
    /// without changing which two trims cross.
    ///
    /// Two kinds of crossing are deliberately NOT here, both because the two
    /// strands are the same curve twice rather than two parts of a boundary:
    ///
    /// * a crossing within `assembler_weld` of one of the loop's own VERTICES.
    ///   Two trims arriving at one vertex are fitted independently, so each
    ///   ends a fit-accuracy away from it and near the vertex they overlap.
    ///   Measured over the case population on 2026-09-13, every such hit sat
    ///   within 2.4e-7 of a vertex and the one true crossing sat 1.8e-1 from
    ///   one — four orders of margin either side of the band, not a threshold
    ///   decision. The cost is stated: a genuine bowtie whose crossing is
    ///   inside that band of a vertex is not reported.
    /// * a crossing between two DIFFERENT coedges that use the SAME EDGE — a
    ///   seam edge, which a periodic face's loop traverses once each way. The
    ///   two are one 3D curve under two fits, so they cannot cross on the
    ///   surface; where the fits wobble past each other they only look like
    ///   it. A coedge against ITSELF is a different question and is asked.
    pub crossings: Vec<LoopCrossing>,
    /// Loops whose pcurves could not be evaluated, `face/loop: reason`. A scan
    /// that could not read a loop did not clear it, so a caller must not read
    /// an empty `crossings` beside a non-empty this as "no crossings".
    pub unreadable: Vec<String>,
}

impl LoopCrossingReport {
    /// The predicate: some face's trim loop crosses itself.
    pub fn is_flagged(&self) -> bool {
        !self.crossings.is_empty()
    }

    /// One line naming the first crossing, or `None` when nothing crossed.
    pub fn summary(&self) -> Option<String> {
        let first = self.crossings.first()?;
        Some(format!(
            "{} loop self-crossing(s); face {}{} loop {} crosses itself where edges {} and {} \
             meet at uv ({:.6}, {:.6}) = ({:.4}, {:.4}, {:.4})",
            self.crossings.len(),
            first.face,
            first
                .face_name
                .as_ref()
                .map(|name| format!(" '{name}'"))
                .unwrap_or_default(),
            first.loop_id,
            first.edge_a,
            first.edge_b,
            first.uv[0],
            first.uv[1],
            first.point.x,
            first.point.y,
            first.point.z
        ))
    }
}

/// One sampled chord of a lifted trim loop.
#[derive(Clone, Copy)]
struct Chord {
    a: Vec2,
    b: Vec2,
}

impl Chord {
    fn shifted(self, shift: Vec2) -> Self {
        Self {
            a: self.a.add(shift),
            b: self.b.add(shift),
        }
    }

    fn direction(&self) -> Vec2 {
        self.b.sub(self.a)
    }

    fn min_u(&self) -> f64 {
        self.a.x.min(self.b.x)
    }

    fn max_u(&self) -> f64 {
        self.a.x.max(self.b.x)
    }

    fn min_v(&self) -> f64 {
        self.a.y.min(self.b.y)
    }

    fn max_v(&self) -> f64 {
        self.a.y.max(self.b.y)
    }
}

/// One loop's sampled trim polygon, lifted into the covering plane.
struct Chain {
    /// `chords + 1` sample points. The last is the loop's closing vertex: the
    /// first again on a loop that closes in the domain, the first plus
    /// [`Chain::wind`] on one that winds the seam.
    points: Vec<Vec2>,
    /// The coedge each chord was sampled from.
    owner: Vec<usize>,
    /// End minus start — zero, or a whole number of periods.
    wind: Vec2,
}

impl Chain {
    fn chords(&self) -> usize {
        self.owner.len()
    }

    fn chord(&self, index: usize) -> Chord {
        Chord {
            a: self.points[index],
            b: self.points[index + 1],
        }
    }

    /// Chain vertex `index`, continued past either end by one LAP of the
    /// loop's own wind — so the strand at the closing vertex still has the
    /// neighbour it has on the surface.
    fn vertex(&self, index: i64) -> Vec2 {
        let last = self.chords() as i64;
        if index < 0 {
            self.points[(index + last) as usize].sub(self.wind)
        } else if index > last {
            self.points[(index - last) as usize].add(self.wind)
        } else {
            self.points[index as usize]
        }
    }
}

/// Which side of the directed chord `point` falls on: `1`, `-1`, or `0`
/// within `weld` of its line.
fn side_of(chord: &Chord, point: Vec2, weld: f64) -> i32 {
    let direction = chord.direction();
    let length = direction.length();
    if !(length > 0.0) {
        return 0;
    }
    let signed = (direction.x * (point.y - chord.a.y) - direction.y * (point.x - chord.a.x)) / length;
    if signed > weld {
        1
    } else if signed < -weld {
        -1
    } else {
        0
    }
}

/// Where along `chord` the nearest point to `point` is, when `point` is within
/// `weld` of the chord at all.
fn along_chord(chord: &Chord, point: Vec2, weld: f64) -> Option<f64> {
    let direction = chord.direction();
    let length_squared = direction.dot(direction);
    if !(length_squared > 0.0) {
        return None;
    }
    let parameter = point.sub(chord.a).dot(direction) / length_squared;
    let foot = chord.a.add(direction.scale(parameter.clamp(0.0, 1.0)));
    (point.sub(foot).length() <= weld).then_some(parameter)
}

/// Do two strands of the chain CROSS where they meet at one point, or does
/// the second merely touch the first and leave the way it came?
///
/// The four directions out of the shared vertex are compared by angle: the
/// strands cross exactly when they ALTERNATE around it — one of the second
/// strand's directions inside the sector the first sweeps and one outside.
///
/// A second-strand direction COLLINEAR with either of the first's — the same
/// way or the opposite way — declines instead of answering. That is not an
/// edge case: it is the standard periodic face. A cylinder's lateral loop
/// spans exactly one period, so the seam it runs UP at `u1` is the seam its
/// own period-shifted image runs DOWN, and the two strands lie along each
/// other at every seam vertex. Two curves that run along each other do not
/// cross there, and a rule that had to pick a side would pick one at every
/// seam in the corpus.
fn strands_cross(
    at: Vec2,
    first_before: Vec2,
    first_after: Vec2,
    second_before: Vec2,
    second_after: Vec2,
    weld: f64,
) -> bool {
    let angle = |point: Vec2| -> Option<f64> {
        let direction = point.sub(at);
        (direction.length() > weld).then(|| direction.y.atan2(direction.x))
    };
    let (Some(from), Some(to), Some(before), Some(after)) = (
        angle(first_before),
        angle(first_after),
        angle(second_before),
        angle(second_after),
    ) else {
        return false;
    };
    for second in [before, after] {
        for first in [from, to] {
            if (second - first).sin().abs() <= CHORD_PARALLEL_SINE {
                return false;
            }
        }
    }
    let sweep = |from: f64, to: f64| (to - from).rem_euclid(std::f64::consts::TAU);
    let span = sweep(from, to);
    let inside = |value: f64| {
        let offset = sweep(from, value);
        offset > 0.0 && offset < span
    };
    inside(before) != inside(after)
}

/// Where chords `first` and `second` of one chain cross, with `second`
/// displaced by `shift` (zero for the loop against itself).
///
/// Three ways a chain crosses itself, and the last two are not exotic: a
/// symmetric bowtie puts the crossing exactly ON a sample of both strands,
/// because both pcurves are sampled at the same fractions of their own
/// domains. A detector that only tested for a crossing strictly inside both
/// chords would miss the very shape it is for.
fn chain_crossing(chain: &Chain, first: usize, second: usize, shift: Vec2, weld: f64) -> Option<Vec2> {
    let one = chain.chord(first);
    let other = chain.chord(second).shifted(shift);

    // 1. Transversally, inside both.
    let a = one.direction();
    let b = other.direction();
    let scale = a.length() * b.length();
    let denominator = a.x * b.y - a.y * b.x;
    if scale > 0.0 && denominator.abs() > CHORD_PARALLEL_SINE * scale {
        let offset = other.a.sub(one.a);
        let t = (offset.x * b.y - offset.y * b.x) / denominator;
        let u = (offset.x * a.y - offset.y * a.x) / denominator;
        let interior = |value: f64| value > CHORD_END_EXCLUSION && value < 1.0 - CHORD_END_EXCLUSION;
        if interior(t) && interior(u) {
            return Some(one.a.add(a.scale(t)));
        }
    }

    // 2. A chain VERTEX sitting inside the other chord: the strand crosses
    //    there when the samples either side of it are on opposite sides.
    let through = |chord: &Chord, vertex: i64, displace: Vec2| -> Option<Vec2> {
        let point = chain.vertex(vertex).add(displace);
        let parameter = along_chord(chord, point, weld)?;
        if !(parameter > CHORD_END_EXCLUSION && parameter < 1.0 - CHORD_END_EXCLUSION) {
            return None;
        }
        let before = side_of(chord, chain.vertex(vertex - 1).add(displace), weld);
        let after = side_of(chord, chain.vertex(vertex + 1).add(displace), weld);
        (before != 0 && after != 0 && before != after).then_some(point)
    };
    if let Some(point) = through(&other, first as i64 + 1, Vec2 { x: 0.0, y: 0.0 }) {
        return Some(point);
    }
    if let Some(point) = through(&one, second as i64 + 1, shift) {
        return Some(point);
    }

    // 3. The two strands meeting AT one shared vertex. Each chain vertex is
    //    the END of exactly one chord, so testing end against end reports a
    //    shared vertex once; end against START is the same point one chord
    //    earlier and is reached as that pair's end-against-end.
    if one.b.sub(other.b).length() <= weld {
        let first_vertex = first as i64 + 1;
        let second_vertex = second as i64 + 1;
        if strands_cross(
            one.b,
            chain.vertex(first_vertex - 1),
            chain.vertex(first_vertex + 1),
            chain.vertex(second_vertex - 1).add(shift),
            chain.vertex(second_vertex + 1).add(shift),
            weld,
        ) {
            return Some(one.b);
        }
    }
    None
}

/// Offer every chord pair whose boxes overlap — `first` against `second`, or
/// against itself when `second` is `None` — by a sweep over sorted
/// u-intervals, so a thousand-chord loop is not a million chord tests.
fn sweep_chords(
    first: &[Chord],
    second: Option<&[Chord]>,
    mut offer: impl FnMut(usize, usize),
) {
    let mut order: Vec<(f64, bool, usize)> = first
        .iter()
        .enumerate()
        .map(|(index, chord)| (chord.min_u(), false, index))
        .collect();
    if let Some(second) = second {
        order.extend(
            second
                .iter()
                .enumerate()
                .map(|(index, chord)| (chord.min_u(), true, index)),
        );
    }
    order.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut active_first: Vec<usize> = Vec::new();
    let mut active_second: Vec<usize> = Vec::new();
    for (sweep_u, is_second, index) in order {
        let chord = if is_second {
            second.expect("second set indexed")[index]
        } else {
            first[index]
        };
        // Lazy deletion: a chord whose u-interval ended before this one starts
        // can never meet anything from here on.
        active_first.retain(|&other| first[other].max_u() >= sweep_u);
        if let Some(second) = second {
            active_second.retain(|&other| second[other].max_u() >= sweep_u);
        }
        // A chord is tested against the ACTIVE chords of the OTHER set, or
        // against its own when there is no other. Either way a pair is
        // offered exactly once.
        let (candidates, pool): (&[usize], &[Chord]) = match (second, is_second) {
            (None, _) => (&active_first, first),
            (Some(second), false) => (&active_second, second),
            (Some(_), true) => (&active_first, first),
        };
        for &other in candidates {
            let against = &pool[other];
            if chord.max_v() < against.min_v() || against.max_v() < chord.min_v() {
                continue;
            }
            if is_second {
                offer(other, index);
            } else {
                offer(index, other);
            }
        }
        if is_second {
            active_second.push(index);
        } else {
            active_first.push(index);
        }
    }
}

/// The shift that carries `point` onto the period of `target` — zero in a
/// direction the surface is not closed in.
fn period_shift(point: Vec2, target: Vec2, period: [Option<f64>; 2]) -> Vec2 {
    let along = |value: f64, want: f64, period: Option<f64>| match period {
        Some(period) if period > 0.0 => ((want - value) / period).round() * period,
        _ => 0.0,
    };
    Vec2 {
        x: along(point.x, target.x, period[0]),
        y: along(point.y, target.y, period[1]),
    }
}

/// One loop's trim polygon, sampled and lifted into the covering plane: each
/// coedge's whole sample run is shifted by the period that joins its start to
/// the previous coedge's end.
///
/// The shift is decided ONCE PER COEDGE, at the boundary where a jump can
/// actually occur — a pcurve is one continuous NURBS and does not jump inside
/// itself. Deciding it per SAMPLE (which is what `validate()`'s uv-wire
/// warning does) takes any genuine chord longer than half a period the short
/// way round instead.
fn lift_loop(loop_record: &LoopRecord, period: [Option<f64>; 2]) -> Result<Chain, String> {
    let mut points: Vec<Vec2> = Vec::new();
    let mut owner: Vec<usize> = Vec::new();
    let mut previous_end: Option<Vec2> = None;
    for (index, coedge) in loop_record.coedges.iter().enumerate() {
        let curve = &coedge.pcurve;
        let [start, end] = curve.domain()?;
        let at = |parameter: f64| -> Result<Vec2, String> {
            let value = curve.evaluate(parameter)?;
            Ok(Vec2 {
                x: value.x,
                y: value.y,
            })
        };
        let shift = match previous_end {
            Some(target) => period_shift(at(start)?, target, period),
            None => Vec2 { x: 0.0, y: 0.0 },
        };
        let samples = crate::classification::trim_sample_count(curve);
        for sample in 0..samples {
            let parameter = start + (end - start) * sample as f64 / samples as f64;
            points.push(at(parameter)?.add(shift));
            owner.push(index);
        }
        previous_end = Some(at(end)?.add(shift));
    }
    let wind = match (points.first(), previous_end) {
        (Some(first), Some(last)) => last.sub(*first),
        _ => Vec2 { x: 0.0, y: 0.0 },
    };
    if let Some(last) = previous_end {
        points.push(last);
    }
    Ok(Chain {
        points,
        owner,
        wind,
    })
}

/// Does any face's trim loop cross ITSELF in its own parameter domain — a
/// bowtie?
///
/// This is the defect every other detector in this module is blind to by
/// construction (see the blind-spot section of the module docs): the crossing
/// is INSIDE one face, so there is no face pair to confirm; the incidence in
/// the record is perfect, so `validate()` is silent; the counts are unchanged,
/// so parity and connectivity are unchanged. The face's own AREA is the only
/// other tell, and nothing compares it to anything.
///
/// The loop is sampled at exactly the density
/// [`crate::parameter_point_in_face`]'s generic lane classifies against
/// (`trim_sample_count`), lifted into the covering plane a coedge at a time,
/// and swept for chord pairs that cross. Chords the CHAIN joins are skipped by
/// index, never by proximity: a loop that runs back over its own last vertex
/// is a finding, not an adjacency.
///
/// Reported once per crossing COEDGE PAIR per loop — the stable half, since a
/// denser sampling moves a crossing's `uv` without changing which two trims
/// cross. Nothing in the kernel's build path calls this; the blend acceptance
/// and the case gate are its callers.
pub fn loop_self_crossings(solid: &BrepSolid) -> LoopCrossingReport {
    let mut report = LoopCrossingReport {
        loops: 0,
        chords: 0,
        crossings: Vec::new(),
        unreadable: Vec::new(),
    };
    // "Are these two independently fitted points the SAME vertex?" — the
    // kernel's own named answer, and the band this detector holds a crossing
    // AT a loop vertex to. See the `crossings` field for why it is needed and
    // what it costs.
    let vertex_band = crate::tolerance::assembler_weld(
        crate::KernelTolerances::for_solid(solid, 1e-7).model,
    );
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let (closed_u, closed_v) = match face.surface.closed_directions() {
            Ok(closed) => closed,
            Err(message) => {
                report.unreadable.push(format!("face {}: {message}", face.id));
                continue;
            }
        };
        let (domain_u, domain_v) = match (face.surface.domain_u(), face.surface.domain_v()) {
            (Ok(u), Ok(v)) => (u, v),
            (Err(message), _) | (_, Err(message)) => {
                report.unreadable.push(format!("face {}: {message}", face.id));
                continue;
            }
        };
        let span_u = domain_u[1] - domain_u[0];
        let span_v = domain_v[1] - domain_v[0];
        let period = [closed_u.then_some(span_u), closed_v.then_some(span_v)];
        // The coincidence band, and the band the side tests hold a sample to.
        // Keyed to the face's own parameter extent: uv is not a length, and a
        // domain is as likely to be [0, 1] as [0, 2π].
        let weld = UV_WELD_FRACTION * (span_u.abs() + span_v.abs()).max(1.0);
        for loop_record in &face.loops {
            let chain = match lift_loop(loop_record, period) {
                Ok(chain) => chain,
                Err(message) => {
                    report.unreadable.push(format!(
                        "face {} loop {}: {message}",
                        face.id, loop_record.id
                    ));
                    continue;
                }
            };
            let count = chain.chords();
            if count < 4 {
                continue;
            }
            report.loops += 1;
            report.chords += count;
            let chords: Vec<Chord> = (0..count).map(|index| chain.chord(index)).collect();

            let mut seen: HashSet<(usize, usize)> = HashSet::default();
            let mut hits: Vec<(usize, usize, Vec2)> = Vec::new();
            let zero = Vec2 { x: 0.0, y: 0.0 };
            // Two different coedges of one loop that use the SAME EDGE are two
            // parameter-space images of ONE 3D curve — a seam edge, which a
            // periodic face's loop traverses once each way. They are the same
            // locus on the surface and cannot cross it; where their two
            // independent fits wobble past each other at the fit tolerance,
            // that is the wobble and not a bowtie. (A coedge against ITSELF is
            // a different question and is still asked.)
            let same_edge = |first: usize, second: usize| {
                let (a, b) = (chain.owner[first], chain.owner[second]);
                a != b
                    && loop_record.coedges[a].edge_id == loop_record.coedges[b].edge_id
            };
            let mut record = |first: usize, second: usize, point: Vec2| {
                let (a, b) = (chain.owner[first], chain.owner[second]);
                let pair = if a <= b { (a, b) } else { (b, a) };
                if seen.insert(pair) {
                    hits.push((pair.0, pair.1, point));
                }
            };

            // How many whole PERIODS the loop's end is from its start. This is
            // the structural question — does the loop close in the domain, or
            // wind the seam — and it must not be asked with a tolerance: the
            // two ends are one vertex reached by two independent pcurve fits,
            // and their uv gap is whatever those fits left (measured up to
            // 2.4e-5 on an offset shell, which is above `validate()`'s own
            // uv-wire closure band). Rounding to laps answers it exactly.
            let laps_between = |value: f64, period: Option<f64>| -> i64 {
                match period {
                    Some(period) if period > 0.0 => (value / period).round() as i64,
                    _ => 0,
                }
            };
            let wind_laps = (
                laps_between(chain.wind.x, period[0]),
                laps_between(chain.wind.y, period[1]),
            );
            let joined = wind_laps == (0, 0);

            // The loop against itself. Chords the chain JOINS are skipped: the
            // consecutive pairs, and the closing pair when the loop closes in
            // the domain rather than winding the seam.
            sweep_chords(&chords, None, |first, second| {
                let (low, high) = if first <= second {
                    (first, second)
                } else {
                    (second, first)
                };
                if high == low + 1 || (joined && low == 0 && high == count - 1) {
                    return;
                }
                if same_edge(low, high) {
                    return;
                }
                if let Some(point) = chain_crossing(&chain, low, high, zero, weld) {
                    record(low, high, point);
                }
            });

            // A PERIODIC face's loop and its own period-shifted image are one
            // curve on the surface, so a loop that WINDS can cross itself
            // across the seam. The shift is taken in one direction only:
            // `(+P, i, j)` and `(-P, j, i)` are the same two points on the
            // surface, so testing both would find every seam crossing twice.
            let extent = |axis: fn(&Vec2) -> f64| -> f64 {
                let values = chain.points.iter().map(axis);
                let (lo, hi) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), value| {
                    (lo.min(value), hi.max(value))
                });
                hi - lo
            };
            let laps = |period: Option<f64>, extent: f64| -> i64 {
                match period {
                    Some(period) if period > 0.0 => (extent / period).ceil() as i64,
                    _ => 0,
                }
            };
            let laps_u = laps(period[0], extent(|point| point.x));
            let laps_v = laps(period[1], extent(|point| point.y));
            for step_u in 0..=laps_u {
                for step_v in -laps_v..=laps_v {
                    // (0, 0) is the scan above; the half-lattice `step_u > 0`,
                    // or `step_u == 0` and `step_v > 0`, holds one of every
                    // (shift, −shift) pair.
                    if step_u == 0 && step_v <= 0 {
                        continue;
                    }
                    let shift = Vec2 {
                        x: step_u as f64 * period[0].unwrap_or(0.0),
                        y: step_v as f64 * period[1].unwrap_or(0.0),
                    };
                    let shifted: Vec<Chord> = chords
                        .iter()
                        .map(|chord| chord.shifted(shift))
                        .collect();
                    // One lap on, the chain's own last chord is joined to its
                    // first: that is the seam, not a crossing.
                    let laps_forward = (step_u, step_v) == wind_laps;
                    let laps_back = (step_u, step_v) == (-wind_laps.0, -wind_laps.1);
                    sweep_chords(&chords, Some(&shifted), |first, second| {
                        if laps_forward && first == count - 1 && second == 0 {
                            return;
                        }
                        if laps_back && first == 0 && second == count - 1 {
                            return;
                        }
                        if same_edge(first, second) {
                            return;
                        }
                        if let Some(point) = chain_crossing(&chain, first, second, shift, weld) {
                            record(first, second, point);
                        }
                    });
                }
            }

            // Where this loop's own VERTICES are, in 3D: one per coedge, at
            // the start of its pcurve.
            let mut corners: Vec<Vec3> = Vec::with_capacity(loop_record.coedges.len());
            if !hits.is_empty() {
                for coedge in &loop_record.coedges {
                    let Ok([start, _]) = coedge.pcurve.domain() else {
                        continue;
                    };
                    let Ok(value) = coedge.pcurve.evaluate(start) else {
                        continue;
                    };
                    if let Ok(point) = face.surface.evaluate(value.x, value.y) {
                        corners.push(point);
                    }
                }
            }
            for (first, second, point) in hits {
                let wrap = |value: f64, domain: [f64; 2], period: Option<f64>| match period {
                    Some(period) if period > 0.0 => {
                        domain[0] + (value - domain[0]).rem_euclid(period)
                    }
                    _ => value.clamp(domain[0], domain[1]),
                };
                let u = wrap(point.x, domain_u, period[0]);
                let v = wrap(point.y, domain_v, period[1]);
                let Ok(position) = face.surface.evaluate(u, v) else {
                    report.unreadable.push(format!(
                        "face {} loop {}: crossing at uv ({u:.6}, {v:.6}) is off the surface",
                        face.id, loop_record.id
                    ));
                    continue;
                };
                // A crossing AT one of the loop's own vertices is the gap
                // between the two pcurves that meet there, not a bowtie. Two
                // trims arriving at one vertex are fitted independently, so
                // each ends a fit-accuracy away from it and near the vertex
                // they overlap; `assembler_weld` is the kernel's own band for
                // "the same vertex reached by two independent fits".
                if corners
                    .iter()
                    .any(|corner| corner.sub(position).length() <= vertex_band)
                {
                    continue;
                }
                let coedge = |index: usize| &loop_record.coedges[index];
                report.crossings.push(LoopCrossing {
                    face: face.id,
                    face_name: face.name.clone(),
                    loop_id: loop_record.id,
                    coedge_a: coedge(first).id,
                    coedge_b: coedge(second).id,
                    edge_a: coedge(first).edge_id,
                    edge_b: coedge(second).edge_id,
                    uv: [u, v],
                    point: position,
                });
            }
        }
    }
    report
}

// ---------------------------------------------------------------------------
// Shell vector area — the closure a trim owes its own edge
// ---------------------------------------------------------------------------

/// Gauss–Legendre accuracy target per span, RELATIVE to that span's own
/// magnitude `∫ |r − R| |dr|`. A span is bisected until the two halves agree
/// with the whole to this fraction of what the span can contribute at all.
///
/// Scale-free by construction, so a 3000 mm part and a 3 mm one are held to
/// the same number of digits. 1e-13 is about a hundred times the f64 noise of
/// an eight-station rule (each station is one rounding of a product of two
/// evaluated vectors), so the recursion terminates on geometry rather than on
/// arithmetic, and two orders below it is what the check needs: the whole
/// point is a residual read at 1e-9 out of face terms worth 1e+2.
const VECTOR_AREA_QUADRATURE_RELATIVE: f64 = 1e-13;

/// `BREP_VECTOR_AREA_QUADRATURE` overrides that target, so the convergence of
/// a residual can be MEASURED rather than argued: read the same shell at
/// 1e-13 and at 1e-15 and the difference is what the quadrature is worth.
/// Read once per process.
fn vector_area_quadrature_relative() -> f64 {
    static TARGET: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *TARGET.get_or_init(|| {
        std::env::var("BREP_VECTOR_AREA_QUADRATURE")
            .ok()
            .and_then(|value| value.parse::<f64>().ok())
            .filter(|value| *value > 0.0 && value.is_finite())
            .unwrap_or(VECTOR_AREA_QUADRATURE_RELATIVE)
    })
}

/// Bisection depth per span. A span that has not converged by here is
/// accepted with its last difference counted into
/// [`ShellVectorArea::quadrature_error`] and into
/// [`ShellVectorArea::spans_at_depth_limit`], never silently.
///
/// The limit is a COST bound, and it is sound because the error it leaves is
/// measured rather than assumed. A span whose integrand has a kink in it — a
/// trim read through the C¹ boundary extension past its surface's domain —
/// converges only linearly, so each further level halves its error for twice
/// the work, and the recursion is buying precision the bar never asked for.
/// `BREP_VECTOR_AREA_DEPTH` overrides it so that trade can be measured.
fn vector_area_quadrature_depth() -> usize {
    static DEPTH: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
    *DEPTH.get_or_init(|| {
        std::env::var("BREP_VECTOR_AREA_DEPTH")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .filter(|value| *value > 0 && *value <= 24)
            .unwrap_or(VECTOR_AREA_QUADRATURE_DEPTH)
    })
}

/// Chosen by measurement, not by taste — and the cheapest setting that keeps
/// the reported error bar is NOT the right one.
///
/// Read on the notched cap's `F7` (curved trims) against the depth-12 value,
/// and on `20_helmet_merge_stage/a` (253 faces, half its spans hitting the
/// limit):
///
/// ```text
/// depth  cap residual   its bar    off depth 12   cap ms   helmet ms
///   12   5.106706e-6    4.204e-10        --        334.9    17347
///    8   5.107557e-6    7.496e-9      8.5e-10       56.4     7356
///    6   5.072736e-6    2.206e-9      3.4e-8        42.9     6385
///    4   5.206351e-6    3.116e-7      1.0e-7        39.2       --
/// ```
///
/// Depth 6 is the trap. It reports a TIGHTER bar than depth 8 while sitting
/// 15x further from the converged value: the half-vs-whole difference
/// underestimates what is left when the recursion stops early on a
/// slowly-converging integrand, so the estimate stops being conservative
/// before the answer stops being right. On the helmet, whose residual is
/// invariant at every depth, that failure is invisible — cost alone would
/// have picked 6.
///
/// 8 is the shallowest depth whose reported bar still COVERS its own
/// deviation from converged, and it is 5.9x cheaper than 12 on the cap and
/// 2.4x on the helmet. The residual stays 681x its own error bar there, so it
/// is still a measurement by a wide margin.
const VECTOR_AREA_QUADRATURE_DEPTH: usize = 8;

/// The most spans one SHELL is integrated over before the scan gives up on it.
///
/// This is a COST bound and it is the only kind that keeps the residual
/// honest: on reaching it the shell is reported in
/// [`VectorAreaReport::unreadable`] and NOT given a residual, so it falls into
/// the same "unjudged" the report uses for a solid with no closed shell. The
/// alternative — integrate the remaining spans at whatever accuracy is to
/// hand and publish the sum — would turn a measurement into a number with an
/// unknown error bar, which is exactly what [`ShellVectorArea::quadrature_error`]
/// exists to prevent. A scan that ran out of budget did not measure anything,
/// and it must not be able to look like a scan that read zero.
///
/// The value is read off the case corpus rather than chosen: see the record's
/// §6.3. `BREP_VECTOR_AREA_SPANS` overrides it.
const VECTOR_AREA_SPAN_BUDGET: usize = 400_000;
//                                        ^ read off the case corpus, not chosen.
// Over its 317 shells the span count runs median 168, p90 8,278, p99 301,826,
// max 1,842,688 — and the distribution has a GAP exactly where a budget wants
// to sit. The four shells above 400,000 are the two `gear-hex-bore-push` rows
// (1.84M spans each, the only rows whose scan broke the case gate's time
// budget); the next largest shell in the whole corpus is 301,826. So this
// number separates the bodies this scan cannot afford from every body it can,
// with a 33% margin below it and a 4.6x jump above it, and it is a property of
// the corpus rather than of anyone's preference.

fn vector_area_span_budget() -> usize {
    static BUDGET: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
    *BUDGET.get_or_init(|| {
        std::env::var("BREP_VECTOR_AREA_SPANS")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .filter(|value| *value > 0)
            .unwrap_or(VECTOR_AREA_SPAN_BUDGET)
    })
}

/// The ABSOLUTE half of the acceptance, per unit of arc: a span is also
/// accepted once the halves agree to this times its own length.
///
/// It is what makes the cost bounded. The relative target alone is a target
/// on a SMOOTH integrand: a trim that runs outside its surface's domain is
/// read through the C¹ boundary extension, so its integrand has a kink, a
/// bisection gains only two orders per four levels, and the recursion bottoms
/// out at the depth limit on every span. Measured on the committed STEP
/// corpus that was not a slow scan but a stopped one — one imported solid
/// held the sweep for eight minutes.
///
/// 1e-11 is a ten-thousandth of [`PCURVE_REFINEMENT_TOLERANCE`], the same
/// per-unit-of-length currency the bar is quoted in, so the whole shell's
/// quadrature error is bounded by `bar / 5000` (a shell's boundary is twice
/// its edge length) whatever its geometry does — and the bound is on the
/// REPORTED estimate too, since every accepted difference is summed into it.
const VECTOR_AREA_QUADRATURE_FLOOR: f64 = 1e-11;

/// `BREP_VECTOR_AREA_FLOOR` overrides [`VECTOR_AREA_QUADRATURE_FLOOR`], so the
/// cost the floor buys can be measured against the error it leaves rather
/// than argued.
fn vector_area_quadrature_floor() -> f64 {
    static FLOOR: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *FLOOR.get_or_init(|| {
        std::env::var("BREP_VECTOR_AREA_FLOOR")
            .ok()
            .and_then(|value| value.parse::<f64>().ok())
            .filter(|value| *value > 0.0 && value.is_finite())
            .unwrap_or(VECTOR_AREA_QUADRATURE_FLOOR)
    })
}

/// One face's vector area `∫ n dA`, as its TRIMS claim it and as its EDGES
/// claim it, and the difference between the two.
///
/// Both are the same boundary integral `(1/2) ∮ (r − R) × dr` about the
/// shell's own reference `R` (see [`ShellVectorArea::reference`]), over the
/// same loops in the same order; only the curve differs. The `trim` walks the
/// pcurve mapped through this face's surface — the boundary the mass
/// integrals actually integrate over — and the `edge` walks the 3D edge curve
/// the coedge names, which is the SAME curve the face across that edge walks.
#[derive(Clone, Debug, Serialize)]
pub struct FaceVectorArea {
    pub shell: u64,
    pub face: u64,
    pub face_name: Option<String>,
    /// `(1/2) ∮ (r − R) × dr` over the face's TRIMS, each coedge sampled in
    /// its own parameter direction (a pcurve is parameterized in its coedge's
    /// traversal direction, so the loop is walked by sampling each pcurve
    /// forwards, never by reversing a reverse coedge's sweep).
    pub trim: Vec3,
    /// The same integral over the 3D EDGE curves, each edge traversed in its
    /// coedge's direction — forwards over `[t0, t1]` for a forward coedge,
    /// backwards for a reverse one.
    pub edge: Vec3,
    /// `trim − edge`: what THIS face's trims add to the shell's residual.
    /// Every edge term cancels across the shell (each edge is walked once
    /// each way), so the shell's residual is exactly the sum of these.
    pub departure: Vec3,
    /// The face's own boundary arc length, taken on the edge curves.
    pub boundary_length: f64,
    /// `PCURVE_REFINEMENT_TOLERANCE * boundary_length` — what this face's
    /// departure can be while every one of its trims lies within the fit
    /// floor of the edge it claims. See [`ShellVectorArea::bar`] for the
    /// derivation; this is the same bound applied to one loop.
    ///
    /// **It does not cover a face whose `closure_gap` is over the floor.**
    /// The derivation takes one integration by parts over a CLOSED loop; a
    /// loop that does not close carries a boundary term the bound never
    /// modelled, and the sliver its chords span is real area that this face's
    /// trims genuinely enclose. Read `closure_gap` first.
    pub bar: f64,
    /// How far this face's own trims fail to MEET each other: the summed 3D
    /// distance from where one coedge's trim ends to where the next one
    /// starts, over this face's loops, including the chord home.
    ///
    /// A different defect from the departure, and the one that is easy to
    /// misread as it. Two trims arriving at one vertex are fitted
    /// independently, each refined to `PCURVE_REFINEMENT_TOLERANCE`, so they
    /// can miss each other by twice it — about 2e-7 — and anything at that
    /// scale is the fit floor rather than a defect. Past it the loop is not
    /// closed in the record, and then this face's `departure` is partly the
    /// area of the sliver its closing chords span rather than any trim lying
    /// off an edge.
    ///
    /// Reported, not verdicted: a bar on it would be a new tolerance, and the
    /// slice that owns a loop's closure is the one that opens it. On the
    /// notched cap (2026-09-16) the top cap `E5:S4:PROFILE:L2_END` carries a
    /// 4.095e-7 net step in uv while every other loop on that shell closes to
    /// 1e-12 or better, and it is the fillet that opens it — the same
    /// document's pre-fillet body closes exactly.
    pub closure_gap: f64,
}

impl FaceVectorArea {
    /// `|departure|`.
    pub fn residual(&self) -> f64 {
        self.departure.length()
    }

    /// Does this face's trim lie further from its own edges than a fit at the
    /// refinement floor can account for?
    pub fn is_over_bar(&self) -> bool {
        self.residual() > self.bar
    }
}

/// One shell's vector-area closure: [`shell_vector_areas`]' per-shell verdict.
#[derive(Clone, Debug, Serialize)]
pub struct ShellVectorArea {
    pub shell: u64,
    pub faces: usize,
    /// Every non-degenerate edge this shell's coedges use is used EXACTLY
    /// TWICE by them. `∫ n dA = 0` is a statement about a closed shell only,
    /// so an open one is measured and reported but never flagged: its
    /// residual is its open boundary's own vector area, which is a fact about
    /// the sheet rather than a defect.
    pub closed: bool,
    /// The point every arm is taken from — the centre of the bounding box of
    /// the vertices this shell's edges use. The residual is independent of it
    /// to `|R| * closure_gap` (below), and a local reference is what keeps a
    /// far-translated part's face terms from dwarfing the number being read,
    /// the same reason `mass_properties::shell_volume_reference` exists.
    pub reference: Vec3,
    /// `Σ_faces (1/2) ∮ (r − R) × dr` over the TRIMS. **Exactly zero on a
    /// sound closed shell**, and otherwise the sum of every face's departure
    /// from the edges it shares.
    pub trim_residual: Vec3,
    /// The same sum over the EDGE curves. Every edge's integral is computed
    /// ONCE and added with each coedge's sign, so on a shell where every edge
    /// is used twice with opposite senses the curve terms cancel to rounding
    /// and what is left is the loops' closing chords alone. It is therefore a
    /// BOOKKEEPING check rather than an accuracy one: it moves off zero when
    /// the shell is open, when an edge is used twice the SAME way, or when
    /// the edge curves do not meet at the vertices they share.
    pub edge_residual: Vec3,
    /// Total arc length of the shell's non-degenerate edges, each counted
    /// ONCE.
    pub edge_length: f64,
    /// Total arc length walked by the faces' loops — `2 * edge_length` on a
    /// closed shell.
    pub boundary_length: f64,
    /// The comparand: `PCURVE_REFINEMENT_TOLERANCE * edge_length`. See the
    /// section docs for the derivation and for why the strict bound is twice
    /// this.
    pub bar: f64,
    /// The summed |half-vs-whole| difference at every accepted span — an
    /// OVER-estimate of this shell's quadrature error, and the error bar the
    /// residual is read against. A residual is a measurement only while it is
    /// far above this.
    pub quadrature_error: f64,
    /// Total length of the chords that close the TRIM loops — how far the
    /// sampled trims fail to meet each other at the vertices they share, over
    /// the whole shell. Two trims arriving at one vertex are fitted
    /// independently, so a few 1e-9 per vertex is ordinary; the chords are
    /// integrated (they are part of the region the mass integrator's own trim
    /// polygon bounds), so this is reported as a fact about the record rather
    /// than as an error term.
    pub trim_closure_gap: f64,
    /// The same for the EDGE loops: how far consecutive edge curves fail to
    /// meet at their shared vertex. `validate()` checks each curve against
    /// its vertex (`IssueKind::CurveVertexGap`); this is what the gaps are
    /// worth to the shell's closure.
    pub edge_closure_gap: f64,
    /// Spans integrated: the pcurve's own knot spans, cut at the surface's
    /// knot levels, bisected to convergence, plus the edge curves' own. The
    /// cost of the whole check is this times one eight-station rule.
    pub spans: usize,
    /// How many of those spans were accepted at the DEPTH LIMIT rather than
    /// because they converged. Their last half-vs-whole difference is in
    /// `quadrature_error` like any other, so the error bar stays honest, but
    /// a shell with many of these is paying the full recursion on an
    /// integrand the bisection cannot resolve — a kink rather than a curve —
    /// and the cost is the symptom.
    pub spans_at_depth_limit: usize,
    /// The scan ABANDONED this shell on [`VECTOR_AREA_SPAN_BUDGET`] with
    /// whole faces still unread.
    ///
    /// Its `trim_residual` is then a PARTIAL SUM over the faces that were
    /// reached, and a partial sum of a boundary integral is not a smaller
    /// version of the answer — it is the vector area of an arbitrary subset
    /// of the shell, which on a 1004-face gear reads 6.08e+3 where the whole
    /// shell reads 1.16e-5. [`Self::residual`] returns `NaN` for such a shell
    /// so that number cannot be read as either a residual or an open
    /// boundary's area.
    pub gave_up: bool,
    /// Per face, in record order.
    pub face_areas: Vec<FaceVectorArea>,
}

/// What a shell's closure scan came back with — a measurement, or nothing.
///
/// The reason this is a type rather than a float: `NaN` is a value, and a value
/// gets folded into a comparison. [`ShellVectorArea::is_over_bar`] is
/// `residual() > bar`, and `NaN > bar` is FALSE, so a shell the scan gave up on
/// reads exactly like a shell that closed — the worst possible answer, since the
/// scan gave up on the bodies whose geometry is hardest. A caller that matches on
/// this cannot make that mistake without writing the arm.
#[derive(Clone, Copy, Debug, Serialize)]
pub enum ClosureReading {
    /// The trims were integrated to the end. `at_depth_limit` spans were
    /// accepted at the bisection's depth limit rather than because they
    /// converged; their last difference is inside `quadrature_error` either way.
    Measured {
        residual: f64,
        bar: f64,
        spans: usize,
        at_depth_limit: usize,
    },
    /// No residual exists for this shell: the scan stopped before it had read
    /// every face, and a partial sum of a boundary integral is not a smaller
    /// version of the answer.
    Unmeasured {
        reason: &'static str,
        spans: usize,
        faces_read: usize,
        faces: usize,
    },
    /// An OPEN shell is not judged by this check at all: `Σ ∫ n dA = 0` is a
    /// statement about a closed one, and an open shell's residual is its own
    /// boundary's vector area, which is a fact about the sheet.
    Open { residual: f64, faces: usize },
}

impl ShellVectorArea {
    /// The typed outcome — what this shell's scan actually came back with.
    pub fn reading(&self) -> ClosureReading {
        if self.gave_up {
            return ClosureReading::Unmeasured {
                reason: "the scan reached its span budget with faces unread",
                spans: self.spans,
                faces_read: self.face_areas.len(),
                faces: self.faces,
            };
        }
        if !self.closed {
            return ClosureReading::Open {
                residual: self.trim_residual.length(),
                faces: self.faces,
            };
        }
        ClosureReading::Measured {
            residual: self.trim_residual.length(),
            bar: self.bar,
            spans: self.spans,
            at_depth_limit: self.spans_at_depth_limit,
        }
    }

    /// `|trim_residual|`, and **`NaN` when the scan gave up on this shell**:
    /// see [`Self::gave_up`]. A partial sum is not a residual. Prefer
    /// [`Self::reading`], which cannot be compared by accident.
    pub fn residual(&self) -> f64 {
        if self.gave_up {
            return f64::NAN;
        }
        self.trim_residual.length()
    }

    /// The predicate: a CLOSED shell whose trims enclose a vector area no fit
    /// at the refinement floor can account for. **False for a shell the scan
    /// did not measure** — ask [`Self::reading`] to tell those apart.
    pub fn is_over_bar(&self) -> bool {
        matches!(self.reading(), ClosureReading::Measured { residual, bar, .. } if residual > bar)
    }

    /// The faces that own the residual, worst departure first.
    pub fn worst_faces(&self, count: usize) -> Vec<&FaceVectorArea> {
        let mut ordered: Vec<&FaceVectorArea> = self.face_areas.iter().collect();
        ordered.sort_by(|a, b| {
            b.residual()
                .partial_cmp(&a.residual())
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| a.face.cmp(&b.face))
        });
        ordered.truncate(count);
        ordered
    }

    /// The worst face's name, or its id when it has none.
    pub fn worst_face_name(&self) -> Option<String> {
        self.worst_faces(1).first().map(|face| {
            face.face_name
                .clone()
                .unwrap_or_else(|| format!("face {}", face.face))
        })
    }
}

/// Outcome of [`shell_vector_areas`].
#[derive(Clone, Debug, Serialize)]
pub struct VectorAreaReport {
    pub shells: Vec<ShellVectorArea>,
    /// Shells or faces whose trims could not be read, `shell/face: reason`. A
    /// scan that could not read a face did not clear it, so an empty
    /// `shells` beside a non-empty this is not "every shell closes".
    pub unreadable: Vec<String>,
}

impl VectorAreaReport {
    /// The predicate: some CLOSED shell is over its bar.
    pub fn is_flagged(&self) -> bool {
        self.shells.iter().any(ShellVectorArea::is_over_bar)
    }

    /// The shells this scan did NOT measure, with what each reached. A report
    /// with entries here has not cleared those shells, whatever the rest read.
    pub fn unmeasured(&self) -> Vec<(u64, ClosureReading)> {
        self.shells
            .iter()
            .map(|shell| (shell.shell, shell.reading()))
            .filter(|(_, reading)| matches!(reading, ClosureReading::Unmeasured { .. }))
            .collect()
    }

    /// The largest closed-shell residual, and the shell that carries it.
    pub fn worst_shell(&self) -> Option<&ShellVectorArea> {
        self.shells
            .iter()
            .filter(|shell| shell.closed)
            .max_by(|a, b| {
                a.residual()
                    .partial_cmp(&b.residual())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    /// `|trim_residual|` of the worst CLOSED shell, and **`NaN` when this
    /// solid has no closed shell at all**.
    ///
    /// Not 0.0. A residual of zero is what a perfectly sound shell reads, so
    /// a solid whose shells are all open — one that has LOST faces, say —
    /// must not be able to read like one through this accessor. The whole
    /// point of the check is that a number nothing compares against is how
    /// the defects it hunts stayed hidden; handing back a sound-looking zero
    /// for "there was nothing here to judge" would rebuild that trap inside
    /// the detector. Use [`Self::has_closed_shell`] to tell the two apart
    /// before reading this, or read the per-shell reports, where an open
    /// shell's residual is its boundary's own vector area and is reported.
    pub fn residual(&self) -> f64 {
        self.worst_shell()
            .map(ShellVectorArea::residual)
            .unwrap_or(f64::NAN)
    }

    /// That shell's bar, `NaN` when there is no closed shell — see
    /// [`Self::residual`] for why this is not 0.0.
    pub fn bar(&self) -> f64 {
        self.worst_shell().map(|shell| shell.bar).unwrap_or(f64::NAN)
    }

    /// Is there a closed shell here for [`Self::residual`] to be about?
    ///
    /// `Σ ∫ n dA = 0` is a statement about a CLOSED shell, so a solid with
    /// none is not "sound" and not "unsound" — it is unjudged, and every
    /// reader of this report has to decide what to do about that rather than
    /// inherit a zero.
    pub fn has_closed_shell(&self) -> bool {
        self.shells.iter().any(|shell| shell.closed)
    }

    /// The shells that are NOT closed, with the vector area their open
    /// boundary encloses — the reading that names a shell which has lost
    /// faces.
    ///
    /// A closed shell's every edge is used exactly twice. A shell that has
    /// dropped a face has that face's edges used once, so it is open here and
    /// its residual is what the missing boundary encloses. That is a
    /// different question from Euler parity, and a strictly stronger one for
    /// this purpose: parity notices an ODD number of faces going and is
    /// silent when an even number does, while this counts the uses of every
    /// edge and does not care how many faces left.
    pub fn open_shells(&self) -> impl Iterator<Item = &ShellVectorArea> {
        self.shells.iter().filter(|shell| !shell.closed)
    }

    /// One line naming the worst shell and the faces that own its residual,
    /// or `None` when every closed shell is inside its bar.
    pub fn summary(&self) -> Option<String> {
        let shell = self
            .shells
            .iter()
            .filter(|shell| shell.is_over_bar())
            .max_by(|a, b| {
                a.residual()
                    .partial_cmp(&b.residual())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })?;
        let named: Vec<String> = shell
            .worst_faces(3)
            .iter()
            .map(|face| {
                format!(
                    "{} {:.3e}",
                    face.face_name
                        .clone()
                        .unwrap_or_else(|| format!("face {}", face.face)),
                    face.residual()
                )
            })
            .collect();
        Some(format!(
            "shell {} closes to {:.3e} of vector area against a bar of {:.3e} \
             ({} faces, {:.4} of edge, quadrature {:.1e}); worst: {}",
            shell.shell,
            shell.residual(),
            shell.bar,
            shell.faces,
            shell.edge_length,
            shell.quadrature_error,
            named.join(", ")
        ))
    }
}

/// `(1/2) ∮ (r − reference) × dr` along the straight chord `from → to` — the
/// term that closes a loop across the gap between two independently fitted
/// curves that both claim to reach one vertex.
fn chord_area(from: Vec3, to: Vec3, reference: Vec3) -> Vec3 {
    from.sub(reference).cross(to.sub(from)).scale(0.5)
}

/// One Gauss–Legendre pass over `[t0, t1]`: that span's half of
/// `(1/2) ∮ (r − reference) × dr`, the magnitude `∫ |r − R| |dr|` its relative
/// accuracy is judged against, and the arc `∫ |dr|` its absolute floor is.
fn vector_area_span<F>(
    at: &F,
    t0: f64,
    t1: f64,
    reference: Vec3,
) -> Result<(Vec3, f64, f64), String>
where
    F: Fn(f64) -> Result<(Vec3, Vec3), String>,
{
    let half = (t1 - t0) * 0.5;
    let middle = (t1 + t0) * 0.5;
    let mut total = Vec3::new(0.0, 0.0, 0.0);
    let mut magnitude = 0.0;
    let mut arc = 0.0;
    for index in 0..GAUSS_X.len() {
        let (point, tangent) = at(middle + half * GAUSS_X[index])?;
        let arm = point.sub(reference);
        let weight = GAUSS_W[index] * half;
        total = total.add(arm.cross(tangent).scale(0.5 * weight));
        magnitude += weight.abs() * arm.length() * tangent.length();
        arc += weight.abs() * tangent.length();
    }
    Ok((total, magnitude, arc))
}

/// [`vector_area_span`] bisected until the halves agree with the whole, or until
/// the disagreement is below `share` — the error this span is ALLOWED, which is
/// the trim's own budget spread over the leaves the depth limit permits.
///
/// Without it a span whose 3D length has collapsed cannot ever be accepted. Its
/// acceptance target is proportional to its own size (relative to `magnitude`,
/// absolute per unit of `arc`), while the disagreement between its halves and
/// its whole is set by the round-off in a tangent the parameterization no longer
/// resolves, which does not shrink with the span. Measured on
/// `fixture_25_cube_pierce_pole_gap_bridge`, a FOUR-face body: spans of arc
/// 2.0e-16 whose halves and whole differ by 1.4e-14 against a target of 2.8e-27,
/// bisecting to the depth limit every time — 313,900 of its 355,610 spans, 5.0 s
/// for a residual whose bar is 1.4e-4. A span contributing 1e-14 cannot move
/// that verdict, and `share` says so.
///
///
/// The composition `S(pcurve(t))` is not a polynomial of any degree — a
/// rational patch read along a rational curve — so no fixed rule integrates
/// it exactly and a fixed rule's error is unknown rather than small. Halving
/// is also what finds a surface KNOT crossing for free: the pcurve knows
/// nothing of the surface's knot lines, and a span that straddles one has a
/// kink in its integrand that the difference sees and the recursion resolves.
fn vector_area_adaptive<F>(
    at: &F,
    t0: f64,
    t1: f64,
    reference: Vec3,
    depth: usize,
    share: f64,
    error: &mut f64,
    spans: &mut usize,
    exhausted: &mut usize,
) -> Result<Vec3, String>
where
    F: Fn(f64) -> Result<(Vec3, Vec3), String>,
{
    let (whole, magnitude, arc) = vector_area_span(at, t0, t1, reference)?;
    let middle = 0.5 * (t0 + t1);
    let (left, _, _) = vector_area_span(at, t0, middle, reference)?;
    let (right, _, _) = vector_area_span(at, middle, t1, reference)?;
    let halves = left.add(right);
    let difference = halves.sub(whole).length();
    let target = (vector_area_quadrature_relative() * magnitude)
        .max(vector_area_quadrature_floor() * arc)
        .max(share);
    if depth == 0 || difference <= target {
        *error += difference;
        *spans += 2;
        if depth == 0 && difference > target {
            *exhausted += 2;
        }
        return Ok(halves);
    }
    let first =
        vector_area_adaptive(at, t0, middle, reference, depth - 1, share, error, spans, exhausted)?;
    let second =
        vector_area_adaptive(at, middle, t1, reference, depth - 1, share, error, spans, exhausted)?;
    Ok(first.add(second))
}

/// The spans one curve is integrated over: its own knot breaks, clipped to
/// `[t0, t1]`.
fn vector_area_breaks(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<Vec<f64>, String> {
    let mut breaks = vec![t0];
    for knot in crate::curve::interior_knots(&curve.knots, curve.degree) {
        if knot > t0 && knot < t1 {
            breaks.push(knot);
        }
    }
    breaks.push(t1);
    Ok(breaks)
}

/// `(1/2) ∮ (r − reference) × dr` over one curve span, plus the arc length
/// and the end-to-end displacement of what was walked.
fn vector_area_of<F>(
    at: &F,
    breaks: &[f64],
    reference: Vec3,
    share: f64,
    error: &mut f64,
    spans: &mut usize,
    exhausted: &mut usize,
) -> Result<Vec3, String>
where
    F: Fn(f64) -> Result<(Vec3, Vec3), String>,
{
    let mut total = Vec3::new(0.0, 0.0, 0.0);
    let depth = vector_area_quadrature_depth();
    for pair in breaks.windows(2) {
        if pair[1] <= pair[0] {
            continue;
        }
        total = total.add(vector_area_adaptive(
            at,
            pair[0],
            pair[1],
            reference,
            depth,
            share,
            error,
            spans,
            exhausted,
        )?);
    }
    Ok(total)
}

/// Arc length over the same spans, by the same rule — used for the bar, so it
/// needs the rule's accuracy and not the residual's.
fn vector_area_length<F>(at: &F, breaks: &[f64]) -> Result<f64, String>
where
    F: Fn(f64) -> Result<(Vec3, Vec3), String>,
{
    let mut length = 0.0;
    for pair in breaks.windows(2) {
        if pair[1] <= pair[0] {
            continue;
        }
        let half = (pair[1] - pair[0]) * 0.5;
        let middle = (pair[1] + pair[0]) * 0.5;
        for index in 0..GAUSS_X.len() {
            let (_, tangent) = at(middle + half * GAUSS_X[index])?;
            length += GAUSS_W[index] * half * tangent.length();
        }
    }
    Ok(length)
}

/// Does a closed shell's boundary enclose the vector area it should — zero?
///
/// A patch's vector area is a BOUNDARY integral,
/// `∫_F n dA = (1/2) ∮_{∂F} r × dr`, so the whole shell's is a sum over the
/// faces' loops. Every edge of a closed shell is walked exactly twice, once
/// each way, so **`Σ_faces ∫ n dA` is exactly zero on a closed shell** — a
/// statement about the RECORD, with no tolerance in it, as long as the two
/// faces meeting at an edge walk the same curve. They do not: each walks its
/// OWN TRIM, the pcurve mapped through its own surface, and what is left over
/// is the sum of every trim's departure from the edge it claims.
///
/// That residual is the one whole-shell number that reads the trims — which
/// is what every mass property is actually an integral over
/// (`geometry/pcurve.rs`, `PCURVE_REFINEMENT_TOLERANCE`) — and nothing in the
/// kernel read it before 2026-09-16. `validate()` compares a pcurve to its
/// edge at `pcurve_consistency` (4e-3 on a small model, 2.5% of the diagonal
/// on a large one): three defects on the notched cap sat between 5e-7 and
/// 1.9e-5, four orders inside that bar, and the solid's volume matched its
/// closed form because two of them cancelled.
///
/// ## The bar, and where it comes from
///
/// Write a trim as its edge plus a departure, `r = e + d`. Over a CLOSED loop,
///
/// ```text
/// (1/2)∮ r×dr − (1/2)∮ e×de = (1/2)∮ (e×dd + d×de) + (1/2)∮ d×dd
///                           = ∮ d×de + (1/2)∮ d×dd
/// ```
///
/// (the middle step is one integration by parts: `∮ e×dd = −∮ de×d` because
/// the loop closes). So a face whose every trim lies within `δ` of its edge
/// departs by at most `δ · L`, its own boundary length, and the second-order
/// term is `δ²`-small. Summing over a closed shell, whose faces walk every
/// edge twice, the strict bound is `2δ · Σ|e|`.
///
/// `δ` is [`PCURVE_REFINEMENT_TOLERANCE`], 1e-7 — the 3D distance
/// `fit_pcurve_on_surface` refines a trim to, and the floor under every mass
/// property. The bar this module applies is the TIGHTER
/// `δ · Σ|e|` — half the strict bound — because the worst case it drops is a
/// shell where every pair of trims sits at the floor on OPPOSITE sides of
/// every edge all the way round. A shell between the two is named, and the
/// per-face `bar` says which faces put it there.
///
/// Two things the bar does not cover, both reported rather than absorbed:
///
/// * a fit that exited at [`crate::PcurveFitExit::SampleCeiling`] never
///   reached the floor, so a shell carrying one can be over the bar and still
///   be the best this kernel builds. That is a finding about the fit, not a
///   reason to widen the bar.
/// * the quadrature's own error, reported per shell as
///   [`ShellVectorArea::quadrature_error`]. A residual inside its own error
///   bar is not a measurement.
///
/// ## What it does not ask
///
/// It does not ask whether a face's trim is in the right PLACE — a trim
/// displaced along its own edge, or two faces' trims displaced the same way,
/// cancel in this sum exactly as they cancel in the volume. It is a closure
/// test, not a fit test: `d` enters only through `∮ d×de`. And it says
/// nothing about an open shell, where the residual is the boundary's own
/// vector area and not a defect.
pub fn shell_vector_areas(solid: &BrepSolid) -> VectorAreaReport {
    let mut report = VectorAreaReport {
        shells: Vec::new(),
        unreadable: Vec::new(),
    };
    let edges: HashMap<u64, &crate::topology::EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let vertices: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    for shell in &solid.shells {
        // Which edges this shell uses, and how often — the closedness test,
        // and the edge set the bar's length is taken over.
        let mut uses: HashMap<u64, usize> = HashMap::default();
        for coedge in shell
            .faces
            .iter()
            .flat_map(|face| &face.loops)
            .flat_map(|record| &record.coedges)
        {
            *uses.entry(coedge.edge_id).or_insert(0) += 1;
        }
        let closed = uses.iter().all(|(id, count)| {
            *count == 2
                || edges
                    .get(id)
                    .map(|edge| edge.degenerate)
                    .unwrap_or(false)
        });

        // The reference: the centre of the bounding box of the vertices this
        // shell's edges use. Far-from-origin arms cost digits in a sum read
        // at 1e-9, and the kernel's volume integrator anchors locally for the
        // same reason.
        let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        let mut seen = false;
        for point in uses
            .keys()
            .filter_map(|id| edges.get(id))
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .filter_map(|id| vertices.get(&id))
        {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                continue;
            }
            low = Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
            high = Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
            seen = true;
        }
        let reference = if seen {
            low.add(high).scale(0.5)
        } else {
            Vec3::new(0.0, 0.0, 0.0)
        };

        // Each edge's own integral and length, computed ONCE and added with
        // the coedge's sign: the two uses of an edge then cancel to rounding
        // rather than to the quadrature, so `edge_residual` reads the
        // BOOKKEEPING (is every edge used twice, and once each way) and the
        // trims carry the geometry.
        // Per edge: its integral over its own ordered span, its arc length,
        // and the two end points — computed once and read by both coedges, so
        // the two uses cancel to rounding rather than to the quadrature.
        // What this shell's reading is ALLOWED to be wrong by, spread over every
        // leaf the depth limit permits — the same bound the per-arc floor above
        // publishes (`VECTOR_AREA_QUADRATURE_FLOOR` per unit of boundary), stated
        // once for the shell instead of per span.
        //
        // A span's own size cannot carry that bound. Where a trim's
        // parameterization has collapsed, the span's magnitude and arc vanish
        // while the disagreement between its halves and its whole does not: it is
        // round-off in a tangent that is no longer resolved. Then the target is
        // unreachable and the span bisects to the depth limit, every time.
        // Measured on `fixture_25_cube_pierce_pole_gap_bridge`, a FOUR-face body:
        // 313,900 of its 355,610 spans, 5.0 s, for a contribution of 1e-14 against
        // a bar of 1.4e-4. A reading may stop once it is inside what the verdict
        // allows, and this says where that is.
        let mut shell_edge_length = 0.0f64;
        let mut top_spans = 0usize;
        {
            let mut measured: HashSet<u64> = HashSet::default();
            for face in &shell.faces {
                for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                    if let Ok(domain) = coedge.pcurve.domain() {
                        if let Ok(breaks) = vector_area_breaks(&coedge.pcurve, domain[0], domain[1]) {
                            top_spans += breaks.len().saturating_sub(1);
                        }
                    }
                    let Some(record) = edges.get(&coedge.edge_id) else { continue };
                    if !measured.insert(record.id) {
                        continue;
                    }
                    let (lo, hi) = if record.t0 <= record.t1 { (record.t0, record.t1) } else { (record.t1, record.t0) };
                    let on_edge = |parameter: f64| -> Result<(Vec3, Vec3), String> { record.curve.deriv1(parameter) };
                    if let Ok(breaks) = vector_area_breaks(&record.curve, lo, hi) {
                        top_spans += breaks.len().saturating_sub(1);
                        if let Ok(length) = vector_area_length(&on_edge, &breaks) {
                            shell_edge_length += length;
                        }
                    }
                }
            }
        }
        let share = if top_spans == 0 {
            0.0
        } else {
            vector_area_quadrature_floor() * 2.0 * shell_edge_length
                / (top_spans as f64 * (1u64 << vector_area_quadrature_depth()) as f64)
        };
        let mut edge_terms: HashMap<u64, (Vec3, f64, Vec3, Vec3)> = HashMap::default();
        let mut quadrature_error = 0.0;
        let mut spans = 0usize;
        let mut exhausted = 0usize;
        let mut unreadable_here = 0usize;
        let mut over_budget = false;
        let mut face_areas: Vec<FaceVectorArea> = Vec::with_capacity(shell.faces.len());
        let mut trim_residual = Vec3::new(0.0, 0.0, 0.0);
        let mut edge_residual = Vec3::new(0.0, 0.0, 0.0);
        let mut trim_gap = 0.0;
        let mut edge_gap = 0.0;
        let mut boundary_length = 0.0;
        for face in &shell.faces {
            // The surface's own knot levels, and its parameter periods: a
            // trim span that crosses one is cut there. The pcurve knows
            // nothing of the surface's knots, and a Gauss rule read straight
            // across one is the same 0.3% error `winding_loops_integral`
            // records — five orders above what this check reads. Bisection
            // alone cannot fix it either: a kink converges polynomially, so
            // the recursion bottoms out at its depth limit instead.
            let levels = surface_breaks(&face.surface)
                .ok()
                .zip(face.surface.domain_u().ok())
                .zip(face.surface.domain_v().ok())
                .map(|(((u_breaks, v_breaks), [u0, u1]), [v0, v1])| {
                    (u_breaks, v_breaks, u1 - u0, v1 - v0)
                });
            let mut trim = Vec3::new(0.0, 0.0, 0.0);
            let mut edge = Vec3::new(0.0, 0.0, 0.0);
            let mut face_length = 0.0;
            let mut face_gap = 0.0;
            let mut failed = false;
            for loop_record in &face.loops {
                // A loop is walked as a CLOSED curve: each coedge's own
                // integral, plus the straight chord from where the previous
                // one ended to where this one starts, and the chord home at
                // the end. Those chords are not bookkeeping — two trims
                // arriving at one vertex are fitted independently and land a
                // fit-accuracy apart, so the sampled loop has small jumps in
                // it, and `(1/2) ∮ r × dr` over an OPEN curve is not a vector
                // area at all: it moves with the reference by
                // `(1/2) R × Σ jumps`. Measured on the notched cap the jumps
                // total 4.1e-7 and that reference dependence is 2e-6, half
                // the residual being read. Closed with its chords the loop
                // has `∮ dr = 0` exactly and the number is a property of the
                // face.
                let mut trim_previous: Option<Vec3> = None;
                let mut trim_first: Option<Vec3> = None;
                let mut edge_previous: Option<Vec3> = None;
                let mut edge_first: Option<Vec3> = None;
                for coedge in &loop_record.coedges {
                    // The trim: the pcurve through this face's own surface.
                    // `deriv1_extended` and not `deriv1`, because a sample a
                    // rounding past a periodic domain must WRAP; the plain
                    // evaluator clamps, which would flatten the curve onto
                    // the domain boundary and manufacture a residual.
                    let on_trim = |parameter: f64| -> Result<(Vec3, Vec3), String> {
                        let (uv, duv) = coedge.pcurve.deriv1(parameter)?;
                        let (point, su, sv) = face.surface.deriv1_extended(uv.x, uv.y)?;
                        Ok((point, su.scale(duv.x).add(sv.scale(duv.y))))
                    };
                    let mut note = |message: String| {
                        report
                            .unreadable
                            .push(format!("shell {} face {}: {message}", shell.id, face.id));
                    };
                    let trim_span = coedge.pcurve.domain().and_then(|domain| {
                        let mut breaks =
                            vector_area_breaks(&coedge.pcurve, domain[0], domain[1])?;
                        if let Some((u_breaks, v_breaks, u_period, v_period)) = levels.as_ref() {
                            breaks = split_at_surface_breaks(
                                &coedge.pcurve,
                                breaks,
                                u_breaks,
                                *u_period,
                                true,
                                false,
                            )?;
                            breaks = split_at_surface_breaks(
                                &coedge.pcurve,
                                breaks,
                                v_breaks,
                                *v_period,
                                false,
                                false,
                            )?;
                        }
                        let value = vector_area_of(
                            &on_trim,
                            &breaks,
                            reference,
                            share,
                            &mut quadrature_error,
                            &mut spans,
                            &mut exhausted,
                        )?;
                        Ok((value, on_trim(domain[0])?.0, on_trim(domain[1])?.0))
                    });
                    match trim_span {
                        Ok((value, start, end)) => {
                            trim = trim.add(value);
                            if let Some(previous) = trim_previous {
                                trim = trim.add(chord_area(previous, start, reference));
                                trim_gap += start.sub(previous).length();
                                face_gap += start.sub(previous).length();
                            } else {
                                trim_first = Some(start);
                            }
                            trim_previous = Some(end);
                        }
                        Err(message) => {
                            note(format!("coedge {}: {message}", coedge.id));
                            unreadable_here += 1;
                            failed = true;
                        }
                    }

                    // The edge: the 3D curve, in the coedge's direction.
                    let Some(record) = edges.get(&coedge.edge_id) else {
                        note(format!(
                            "coedge {} names edge {}, which is not in the record",
                            coedge.id, coedge.edge_id
                        ));
                        unreadable_here += 1;
                        failed = true;
                        continue;
                    };
                    if !edge_terms.contains_key(&record.id) {
                        let on_edge = |parameter: f64| -> Result<(Vec3, Vec3), String> {
                            record.curve.deriv1(parameter)
                        };
                        let (lo, hi) = if record.t0 <= record.t1 {
                            (record.t0, record.t1)
                        } else {
                            (record.t1, record.t0)
                        };
                        let term = vector_area_breaks(&record.curve, lo, hi).and_then(|breaks| {
                            let value = vector_area_of(
                                &on_edge,
                                &breaks,
                                reference,
                                share,
                                &mut quadrature_error,
                                &mut spans,
                                &mut exhausted,
                            )?;
                            let length = vector_area_length(&on_edge, &breaks)?;
                            Ok((value, length, on_edge(lo)?.0, on_edge(hi)?.0))
                        });
                        match term {
                            Ok(term) => {
                                edge_terms.insert(record.id, term);
                            }
                            Err(message) => {
                                note(format!("edge {}: {message}", record.id));
                                unreadable_here += 1;
                                failed = true;
                                continue;
                            }
                        }
                    }
                    let Some((value, length, low, high)) = edge_terms.get(&record.id).copied()
                    else {
                        continue;
                    };
                    // `t0 > t1` on the record is the same span walked the
                    // other way; the integral above was taken over the
                    // ordered span, so the record's own order is one sign and
                    // the coedge's `forward` the other.
                    let ordered = if record.t0 <= record.t1 { 1.0 } else { -1.0 };
                    let sense = if coedge.forward { 1.0 } else { -1.0 };
                    let signed = ordered * sense;
                    edge = edge.add(value.scale(signed));
                    let (start, end) = if signed > 0.0 { (low, high) } else { (high, low) };
                    if let Some(previous) = edge_previous {
                        edge = edge.add(chord_area(previous, start, reference));
                        edge_gap += start.sub(previous).length();
                    } else {
                        edge_first = Some(start);
                    }
                    edge_previous = Some(end);
                    face_length += length;
                }
                if let (Some(previous), Some(first)) = (trim_previous, trim_first) {
                    trim = trim.add(chord_area(previous, first, reference));
                    trim_gap += first.sub(previous).length();
                    face_gap += first.sub(previous).length();
                }
                if let (Some(previous), Some(first)) = (edge_previous, edge_first) {
                    edge = edge.add(chord_area(previous, first, reference));
                    edge_gap += first.sub(previous).length();
                }
            }
            if failed {
                continue;
            }
            // The budget, checked per face so a shell gives up on a face
            // boundary rather than part-way through a loop. `over_budget`
            // makes the shell unreadable below; it is never a residual.
            if spans > vector_area_span_budget() {
                over_budget = true;
                report.unreadable.push(format!(
                    "shell {}: gave up after {} spans on {} of {} faces (budget {}); UNJUDGED rather than clean — raise BREP_VECTOR_AREA_SPANS to read it",
                    shell.id,
                    spans,
                    face_areas.len() + 1,
                    shell.faces.len(),
                    vector_area_span_budget()
                ));
                break;
            }
            boundary_length += face_length;
            trim_residual = trim_residual.add(trim);
            edge_residual = edge_residual.add(edge);
            face_areas.push(FaceVectorArea {
                shell: shell.id,
                face: face.id,
                face_name: face.name.clone(),
                trim,
                edge,
                departure: trim.sub(edge),
                boundary_length: face_length,
                bar: crate::pcurve::PCURVE_REFINEMENT_TOLERANCE * face_length,
                closure_gap: face_gap,
            });
        }
        let edge_length: f64 = edge_terms.values().map(|(_, length, _, _)| *length).sum();
        report.shells.push(ShellVectorArea {
            shell: shell.id,
            faces: shell.faces.len(),
            // A face that could not be read leaves its share of the residual
            // out of the sum, so the shell is not claimed to close — and a
            // shell abandoned on the span budget is in exactly that position,
            // with whole faces missing from the sum.
            closed: closed && unreadable_here == 0 && !over_budget,
            reference,
            trim_residual,
            edge_residual,
            edge_length,
            boundary_length,
            bar: crate::pcurve::PCURVE_REFINEMENT_TOLERANCE * edge_length,
            quadrature_error,
            spans,
            spans_at_depth_limit: exhausted,
            trim_closure_gap: trim_gap,
            edge_closure_gap: edge_gap,
            gave_up: over_budget,
            face_areas,
        });
    }
    report
}
