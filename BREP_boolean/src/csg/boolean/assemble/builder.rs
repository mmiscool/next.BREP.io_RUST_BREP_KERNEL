use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

pub struct SourceEdge {
    pub curve: crate::NurbsCurve,
    pub t0: f64,
    pub t1: f64,
    pub start: Vec3,
    pub end: Vec3,
    pub degenerate: bool,
    pub name: Option<String>,
    /// A CONSTRUCTED section: the source is an imprint piece the Boolean
    /// marched on two carriers, not a boundary edge of an operand (vendor
    /// geometry) nor a derived sub-span of one. Only such an edge's trims may
    /// be refreshed from the edge after vertex settling (`vertices.rs`).
    pub constructed: bool,
    // Identity of the ORIGINAL boundary edge this source came from
    // `(operand, edge_id)`, for uncut faces passed through the arrangement.
    // `None` for imprint / derived (section) edges, which have no pre-boolean
    // identity. Two coedges of one operand that reference DIFFERENT original
    // edge ids are, by construction, distinct 1-cells of a valid input and
    // must never weld into one assembler edge — even when their geometry is
    // coincident within the weld radius (thin seam-cut bands whose rims sit a
    // single weld-tolerance apart). This keeps the weld from confusing a hole
    // wall's two rims, or a rim with its neighbouring plate rim.
    pub boundary_key: Option<(u8, u64)>,
    /// The imprint piece a section source is (`None` for boundary and
    /// derived sources): two different pieces running apart by more than ten
    /// identity tolerances at their middles are not welded into one edge
    /// under `BREP_FEATURE_SCALE_LENS`, whatever the weld band reads (a
    /// plane across a lens 1.8e-6 thick cuts its two walls in two arcs that
    /// share both ends).
    pub piece: Option<u64>,
    /// MATERIAL SIDE of a TANGENT section's coedge (`ImprintOptions::
    /// tangent_curve_imprint`; `TangentSectionRecord::piece_ids`): the
    /// direction into the face from the section's midpoint, `n_out × t`
    /// (the face's outward normal crossed with the coedge's travel
    /// direction). Where two operands' faces touch along a tangent section
    /// and material survives on both of its sides, four coedges arrive on
    /// one locus; the two bounding one sliver of material point INTO it the
    /// same way (the wall half and the bore quarter on the same side of the
    /// contact), the two bounding the other point the opposite way. The weld
    /// pairs coedges whose sides agree and refuses the rest, so the locus is
    /// carried as two manifold edge records instead of one four-coedge
    /// incidence — or, when only two coedges arrive (a union keeps the wall
    /// halves and drops the bore), the identical-edge weld joins them
    /// afterwards and the coplanar merge dissolves the flat edge. `None` on
    /// every other source: the scan pairs by arrival as before. Appended at
    /// the end of the struct (2026-10-11).
    pub tangent_side: Option<Vec3>,
    /// `BREP_ASSEMBLY_FEATURE_WELD`: the weld radius at this source's
    /// `[start, end]`, a quarter of the shortest edge its fragment loop
    /// meets there (see [`FeatureWeld`]). `None` where no loop was read:
    /// the band alone decides, as before.
    pub feature_radius: Option<[f64; 2]>,
    /// Whether the topology upstream already holds this source's two ends
    /// distinct: an operand edge or an imprint piece whose start and end
    /// vertex ids differ. Only such an edge bounds the feature radius of its
    /// ends, so a piece the imprint merged shut (fitting noise a few 1e-6
    /// long) never shrinks a weld that today's band makes.
    pub distinct_ends: bool,
}

/// id-index over the immutable operands + imprint, built once before the
/// assemble loop. Every lookup returns the exact record the previous linear
/// `.iter().find(id==)` scans returned; only the O(N)-per-coedge cost changes
/// (assembly ran these scans once per coedge = O(coedges * (E + V + F))).
pub(super) struct AssembleIndex<'a> {
    operand_edges: HashMap<u8, HashMap<u64, &'a EdgeRecord>>,
    operand_vertices: HashMap<u8, HashMap<u64, Vec3>>,
    operand_face_names: HashMap<u8, HashMap<u64, Option<String>>>,
    imprint_pieces: HashMap<u64, &'a ImprintPieceRecord>,
    imprint_vertices: HashMap<u64, Vec3>,
}

impl<'a> AssembleIndex<'a> {
    pub(super) fn build(solids: &HashMap<u8, &'a BrepSolid>, imprint: &'a ImprintResultRecord) -> Self {
        let mut operand_edges: HashMap<u8, HashMap<u64, &'a EdgeRecord>> = HashMap::default();
        let mut operand_vertices: HashMap<u8, HashMap<u64, Vec3>> = HashMap::default();
        let mut operand_face_names: HashMap<u8, HashMap<u64, Option<String>>> = HashMap::default();
        for (&operand, &solid) in solids {
            operand_edges.insert(
                operand,
                solid.edges.iter().map(|edge| (edge.id, edge)).collect(),
            );
            operand_vertices.insert(
                operand,
                solid
                    .vertices
                    .iter()
                    .map(|vertex| (vertex.id, vertex.point))
                    .collect(),
            );
            operand_face_names.insert(
                operand,
                solid
                    .shells
                    .iter()
                    .flat_map(|shell| &shell.faces)
                    .map(|face| (face.id, face.name.clone()))
                    .collect(),
            );
        }
        AssembleIndex {
            operand_edges,
            operand_vertices,
            operand_face_names,
            imprint_pieces: imprint
                .pieces
                .iter()
                .map(|piece| (piece.id, piece))
                .collect(),
            imprint_vertices: imprint
                .vertices
                .iter()
                .map(|vertex| (vertex.id, vertex.point))
                .collect(),
        }
    }
}

fn vertex_in_solid(index: &AssembleIndex, operand: u8, id: u64) -> Result<Vec3, KernelRefusal> {
    index
        .operand_vertices
        .get(&operand)
        .and_then(|vertices| vertices.get(&id))
        .copied()
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("boolean assembly: missing vertex {id}")))
}

pub(super) fn face_name_in_solids(index: &AssembleIndex, operand: u8, face_id: u64) -> Option<String> {
    index
        .operand_face_names
        .get(&operand)?
        .get(&face_id)
        .cloned()
        .flatten()
}

/// New intersection edges are named after their two supporting faces using
/// the application's `FACEA|FACEB` convention (alphabetical order), so a
/// rebuilt model derives the same edge identity without geometric matching.
fn imprint_edge_name(
    index: &AssembleIndex,
    supports: &[crate::imprint::FaceKey; 2],
) -> Option<String> {
    let mut names = [
        face_name_in_solids(index, supports[0].operand, supports[0].face_id)?,
        face_name_in_solids(index, supports[1].operand, supports[1].face_id)?,
    ];
    names.sort();
    Some(format!("{}|{}", names[0], names[1]))
}

fn boundary_source_edge(
    index: &AssembleIndex,
    operand: u8,
    edge_id: u64,
) -> Result<SourceEdge, KernelRefusal> {
    let edge = *index
        .operand_edges
        .get(&operand)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("boolean assembly: missing operand {operand}")))?
        .get(&edge_id)
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("boolean assembly: missing boundary edge {edge_id}")))?;
    Ok(SourceEdge {
        curve: edge.curve.clone(),
        t0: edge.t0,
        t1: edge.t1,
        start: vertex_in_solid(index, operand, edge.start_vertex_id)?,
        end: vertex_in_solid(index, operand, edge.end_vertex_id)?,
        degenerate: edge.degenerate,
        name: edge.name.clone(),
        constructed: false,
        boundary_key: Some((operand, edge_id)),
        piece: None,
        tangent_side: None,
        feature_radius: None,
        distinct_ends: edge.start_vertex_id != edge.end_vertex_id && !edge.degenerate,
    })
}

pub(super) fn source_edge(source: &FragmentEdgeSource, index: &AssembleIndex) -> Result<SourceEdge, KernelRefusal> {
    match source {
        FragmentEdgeSource::Boundary { operand, edge_id }
        | FragmentEdgeSource::SharedBoundary { operand, edge_id } => {
            boundary_source_edge(index, *operand, *edge_id)
        }
        FragmentEdgeSource::Imprint { piece_id } => {
            let piece = *index
                .imprint_pieces
                .get(piece_id)
                .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("boolean assembly: missing imprint piece {piece_id}")))?;
            let point = |vertex_id| {
                index
                    .imprint_vertices
                    .get(&vertex_id)
                    .copied()
                    .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", "boolean assembly: missing imprint vertex"))
            };
            Ok(SourceEdge {
                curve: piece.curve.clone(),
                t0: piece.t0,
                t1: piece.t1,
                start: point(piece.start_vertex_id)?,
                end: point(piece.end_vertex_id)?,
                degenerate: false,
                name: imprint_edge_name(index, &piece.support_faces),
                constructed: true,
                boundary_key: None,
                piece: Some(*piece_id),
                tangent_side: None,
                feature_radius: None,
                distinct_ends: piece.start_vertex_id != piece.end_vertex_id,
            })
        }
        FragmentEdgeSource::Derived {
            curve,
            t0,
            t1,
            start,
            end,
        } => Ok(SourceEdge {
            curve: curve.clone(),
            t0: *t0,
            t1: *t1,
            start: *start,
            end: *end,
            degenerate: start.sub(*end).length() <= 1e-8,
            name: None,
            constructed: false,
            boundary_key: None,
            piece: None,
            tangent_side: None,
            feature_radius: None,
            // A sub-span of an operand edge cut at imprint vertices: the
            // fragmenter splits only at a vertex the imprint kept apart from
            // the edge's ends, so its ends are distinct unless it collapsed.
            distinct_ends: start.sub(*end).length() > 1e-8,
        }),
    }
}

/// Same-binary attribution for the identity index; never an acceptance switch.
fn weld_index_on() -> bool {
    switch!("BREP_ASSEMBLY_MEMO", true)
}

/// Index the exact identity exclusion already enforced by the weld. Keep
/// every possible candidate (including duplicate keys) in insertion order;
/// geometry, incidence and tangent-side acceptance still run unchanged.
#[derive(Default)]
pub struct WeldCandidates {
    pub(super) vertex_candidates: Option<VertexCandidates>,
    indexed: usize,
    by_operand: HashMap<Option<u8>, Vec<usize>>,
    by_key: HashMap<(u8, u64), Vec<usize>>,
    pub(super) offered: usize,
    pub(super) exhaustive: usize,
}

impl WeldCandidates {
    pub(super) fn for_assembly() -> Self {
        Self { vertex_candidates: switch!("BREP_ASSEMBLY_VERTEX_INDEX", true).then(Default::default), ..Self::default() }
    }

    fn push(&mut self, key: Option<(u8, u64)>) {
        self.by_operand.entry(key.map(|k| k.0)).or_default().push(self.indexed);
        if let Some(key) = key { self.by_key.entry(key).or_default().push(self.indexed); }
        self.indexed += 1;
    }

    fn candidates(&mut self, source: Option<(u8, u64)>) -> Vec<usize> {
        let mut result = if let Some(key) = source {
            let mut indices = self.by_key.get(&key).cloned().unwrap_or_default();
            for (operand, bucket) in &self.by_operand {
                if *operand != Some(key.0) { indices.extend_from_slice(bucket); }
            }
            indices.sort_unstable();
            indices
        } else { (0..self.indexed).collect() };
        self.exhaustive += self.indexed;
        self.offered += result.len();
        // Diagnostic attribution only; both paths retain identical ordering.
        if !weld_index_on() { result = (0..self.indexed).collect(); }
        result
    }
}

/// Broad phase over the widest coordinate of the append-only vertex records.
/// Reconsider the axis as the population doubles, rebuilding only when it
/// changes; this keeps axis-aligned faces from collapsing into one bucket.
/// Buckets only shortlist candidates; the original distance test and first
/// insertion order still decide identity. Disabled before conformance can
/// replace or move the records.
#[derive(Default)]
pub(super) struct VertexCandidates {
    indexed: usize,
    axis: usize,
    reconsider_at: usize,
    buckets: HashMap<i64, Vec<usize>>,
    unbucketed: Vec<usize>,
}

impl VertexCandidates {
    fn coordinate(point: Vec3, axis: usize) -> f64 {
        [point.x, point.y, point.z][axis]
    }

    fn reconsider_axis(&mut self, vertices: &[VertexRecord]) {
        if vertices.len() < self.reconsider_at { return; }
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for vertex in vertices {
            for axis in 0..3 {
                let value = Self::coordinate(vertex.point, axis);
                if value.is_finite() { low[axis] = low[axis].min(value); high[axis] = high[axis].max(value); }
            }
        }
        let mut axis = self.axis;
        for candidate in 0..3 {
            if high[candidate] - low[candidate] > high[axis] - low[axis] { axis = candidate; }
        }
        if axis != self.axis {
            self.axis = axis;
            self.indexed = 0;
            self.buckets.clear();
            self.unbucketed.clear();
        }
        self.reconsider_at = vertices.len().max(8).saturating_mul(2);
    }

    fn key(x: f64, tolerance: f64) -> Option<i64> {
        let cell = 2.0 * tolerance;
        if !(cell.is_finite() && cell > 0.0) { return None; }
        let scaled = (x / cell).floor();
        // Half-cell slack covers division rounding well inside the exact
        // integer range. Extreme coordinates keep the exhaustive search.
        (scaled.is_finite() && scaled.abs() <= (1u64 << 48) as f64).then_some(scaled as i64)
    }

    fn first(&mut self, vertices: &[VertexRecord], point: Vec3, tolerance: f64) -> Option<u64> {
        self.first_admitted(vertices, point, tolerance, |_, _| true)
    }

    /// [`Self::first`] restricted to the vertices `admit(id, distance)`
    /// accepts among those within `tolerance`.
    fn first_admitted(
        &mut self,
        vertices: &[VertexRecord],
        point: Vec3,
        tolerance: f64,
        admit: impl Fn(u64, f64) -> bool,
    ) -> Option<u64> {
        self.reconsider_axis(vertices);
        for (index, vertex) in vertices.iter().enumerate().skip(self.indexed) {
            match Self::key(Self::coordinate(vertex.point, self.axis), tolerance) {
                Some(key) => self.buckets.entry(key).or_default().push(index),
                None => self.unbucketed.push(index),
            }
        }
        self.indexed = vertices.len();
        let Some(key) = Self::key(Self::coordinate(point, self.axis), tolerance) else {
            return vertices
                .iter()
                .find(|v| {
                    let distance = v.point.sub(point).length();
                    distance <= tolerance && admit(v.id, distance)
                })
                .map(|v| v.id);
        };
        let mut first = None;
        for index in (key - 1..=key + 1)
            .flat_map(|key| self.buckets.get(&key).into_iter().flatten())
            .chain(&self.unbucketed)
        {
            if first.is_some_and(|earlier| earlier <= *index) { continue; }
            let distance = vertices[*index].point.sub(point).length();
            if distance <= tolerance && admit(vertices[*index].id, distance) { first = Some(*index); }
        }
        first.map(|index| vertices[index].id)
    }
}

pub struct Assembler {
    pub weld_candidates: WeldCandidates,
    pub tolerance: f64,
    pub vertices: Vec<VertexRecord>,
    // Source positions retain their lookup records after a witnessed edge
    // correspondence joins them. Future coedges resolve those records to the
    // same canonical vertex instead of recreating the old endpoint identity.
    pub vertex_aliases: HashMap<u64, u64>,
    pub edges: Vec<EdgeRecord>,
    pub edge_use_counts: HashMap<u64, usize>,
    // Result `forward` of the FIRST coedge attached to each edge, keyed by
    // edge id. Consulted only while an edge is still one-use (a weld candidate)
    // so the weld can refuse a mate that would give the edge two same-sense
    // coedges — an unconditionally non-manifold incidence (topology.rs). The
    // entry is written once at edge creation and never resynced: a stale or
    // missing entry only makes the guard a no-op (today's behaviour), and by
    // the time later passes flip a coedge's `forward` the edge is already
    // two-use, so the map is never read for it again.
    pub edge_first_forward: HashMap<u64, bool>,
    // Original `(operand, edge_id)` of the boundary source that CREATED each
    // assembler edge (absent for imprint/derived-sourced edges). A weld
    // candidate whose incoming source shares the operand but not the edge id
    // is a distinct 1-cell of the input and is refused — see `SourceEdge`.
    pub edge_boundary_key: HashMap<u64, (u8, u64)>,
    // Edges created from an imprint piece (a constructed section). A weld of a
    // later source onto one of these keeps the edge; a weld of an imprint
    // source onto a boundary edge leaves the boundary edge unmarked.
    pub imprint_edges: HashSet<u64>,
    /// The imprint piece each edge was created from (`SourceEdge::piece`).
    pub edge_piece: HashMap<u64, u64>,
    pub next_vertex_id: u64,
    pub next_edge_id: u64,
    pub next_coedge_id: u64,
    pub next_loop_id: u64,
    pub next_face_id: u64,
    // The FIRST assembler edge minted from each operand boundary edge, by
    // `(operand, edge_id)`: the mate an untouched face's coedge welds to
    // without the geometric scan (`edge_untouched`).
    pub edge_by_boundary_key: HashMap<(u8, u64), u64>,
    // The material side recorded for each assembler edge minted from a
    // sided source (`SourceEdge::tangent_side`): a weld candidate whose side
    // points the other way bounds the other sliver and is refused. Appended
    // at the end of the struct (2026-10-11).
    pub edge_tangent_side: HashMap<u64, Vec3>,
    /// Set from the producer's declared kind before assembly. Each source
    /// boundary key is a region's edge pair; radial validation certifies
    /// the resulting multi-region edge before anything is returned.
    pub cellular: bool,
    /// `BREP_ASSEMBLY_FEATURE_WELD` state: the mode, each vertex's weld
    /// radius and the fragment being assembled (for check-mode reports).
    pub feature: FeatureWeld,
}

/// Differing check-mode decisions printed per process before only the
/// per-assembly count is (see [`FeatureWeld::report`]).
const FEATURE_WELD_REPORT_LINES: usize = 20;

/// Below this a feature is under the model tolerance and the feature weld
/// stops shrinking: two points closer than it are one point.
pub(super) const FEATURE_WELD_FLOOR: f64 = 1e-7;

/// `BREP_ASSEMBLY_FEATURE_WELD` (default on, `0` restores; `check` keeps the
/// band weld and reports every decision the feature weld would make
/// differently). The band weld merges any two endpoints within the caller's
/// tolerance (1e-5 for booleans, 2e-3 for offset) and reads any edge whose
/// chord falls within it as degenerate, even where the imprint and the
/// fragmenter have already proved the two points distinct: two ends of one
/// edge of a fragment loop. A shell whose inner corner arcs are 8.5e-4 long
/// lost them under offset's 2e-3 band (the arcs' ends welded, the corner
/// faces fell away), and two boxes whose faces stand 1e-5 apart lost the
/// corners of the sliver faces between them. The feature weld bounds each
/// vertex's radius by a quarter of the shortest edge its loop meets there,
/// floored at [`FEATURE_WELD_FLOOR`], and reads an edge as degenerate only
/// when flagged or shorter than the floor.
#[derive(Debug, Default)]
pub struct FeatureWeld {
    pub mode: FeatureWeldMode,
    /// Weld radius per assembler vertex id (absent: the band).
    pub radius: HashMap<u64, f64>,
    /// `(operand, source face id)` of the fragment being assembled.
    pub fragment: Option<(u8, u64)>,
    /// Decisions that differed in check mode.
    pub differences: usize,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FeatureWeldMode {
    #[default]
    Off,
    Check,
    On,
}

impl FeatureWeld {
    pub(super) fn from_switch() -> Self {
        let mode = match crate::mode_switch!(live "BREP_ASSEMBLY_FEATURE_WELD", true) {
            crate::env_read::Mode::Off => FeatureWeldMode::Off,
            crate::env_read::Mode::Check => FeatureWeldMode::Check,
            crate::env_read::Mode::On => FeatureWeldMode::On,
        };
        FeatureWeld { mode, ..Default::default() }
    }

    /// Counts a differing decision; prints the first
    /// [`FEATURE_WELD_REPORT_LINES`] per process (every one under
    /// `BREP_ASSEMBLY_FEATURE_WELD_LOG=all`), so a check-mode census does
    /// not flood its log. The count is printed per assembly at its end.
    fn report(&mut self, what: std::fmt::Arguments<'_>) {
        use std::sync::atomic::{AtomicUsize, Ordering};
        static PRINTED: AtomicUsize = AtomicUsize::new(0);
        self.differences += 1;
        let all = env_read!("BREP_ASSEMBLY_FEATURE_WELD_LOG").as_deref() == Some("all");
        if all || PRINTED.fetch_add(1, Ordering::Relaxed) < FEATURE_WELD_REPORT_LINES {
            eprintln!("FEATURE_WELD differs fragment={:?}: {what}", self.fragment);
        }
    }
}

/// Loops up to this many coedges also bound each vertex by its distance to
/// the loop's non-adjacent corners; longer loops read adjacent edges only.
const FEATURE_WELD_PAIRWISE_LOOP: usize = 64;

/// The feature radii of a fragment loop's sources, in each source's own
/// `[start, end]` order. Vertex `j` (the start of coedge `j` along the loop)
/// takes `max(floor, min(band, f/4))`, where `f` is the shorter chord of
/// the two loop edges meeting there and, on loops of at most
/// [`FEATURE_WELD_PAIRWISE_LOOP`] coedges, its distance to every corner of
/// the loop that is not its neighbour. Only an edge whose ends the topology
/// upstream already holds distinct (`SourceEdge::distinct_ends`, chord over
/// the floor) is a feature: the loop's corners joined by any other edge
/// count as one corner, so fitting noise the imprint merged shut never
/// shrinks a radius.
/// The short sources (chord within `band`, over the model floor) of the
/// fragments' loops that may keep their two ends apart, keyed
/// `(fragment, (loop, coedge))` in the order the assembler reads them.
///
/// A short source qualifies when another fragment carries a short source on
/// the same two ends ([`mate_short_sources`]), so both faces sharing it read
/// it alike whatever kind each sees, AND no pair of longer sources needs its
/// two ends merged: two fragments' copies of one longer edge (far ends within
/// a quarter of the short chord, midpoints within the band) whose near ends land on the two different ends
/// of the short source pair only through the band weld, so keeping the short
/// edge would leave both copies one-use. Such a short source and its mates
/// keep the band reading. [`Assembler::edge`] keeps a qualifying source's
/// ends apart under `BREP_ASSEMBLY_FEATURE_WELD`.
pub(super) fn short_source_mates(
    fragments: &[crate::fragment::FaceFragmentRecord],
    index: &AssembleIndex,
    band: f64,
) -> Result<HashSet<(usize, (usize, usize))>, KernelRefusal> {
    let mut short = Vec::new();
    // Every source: (fragment, start, end, midpoint).
    let mut all: Vec<(usize, Vec3, Vec3, Vec3)> = Vec::new();
    for (fragment_index, fragment) in fragments.iter().enumerate() {
        for (loop_index, fragment_loop) in fragment.loops.iter().enumerate() {
            for (coedge_index, usage) in fragment_loop.coedges.iter().enumerate() {
                let source = source_edge(&usage.source, index)?;
                let chord = source.start.sub(source.end).length();
                if chord > FEATURE_WELD_FLOOR && chord <= band && !source.degenerate {
                    short.push(((fragment_index, (loop_index, coedge_index)), source.start, source.end, chord));
                } else if chord > band {
                    let mid = source.curve.evaluate(0.5 * (source.t0 + source.t1)).unwrap_or(source.start.add(source.end).scale(0.5));
                    all.push((fragment_index, source.start, source.end, mid));
                }
            }
        }
    }
    let mut mated = mate_short_sources(&short);
    if mated.is_empty() {
        return Ok(mated);
    }
    // Longer sources by the band cell of each end.
    let cell = |point: Vec3| ((point.x / band).floor() as i64, (point.y / band).floor() as i64, (point.z / band).floor() as i64);
    let mut by_cell: HashMap<(i64, i64, i64), Vec<(usize, bool)>> = HashMap::default();
    for (source, &(_, start, end, _)) in all.iter().enumerate() {
        by_cell.entry(cell(start)).or_default().push((source, true));
        by_cell.entry(cell(end)).or_default().push((source, false));
    }
    let near = |point: Vec3| {
        let (x, y, z) = cell(point);
        let mut found = Vec::new();
        for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
            if let Some(list) = by_cell.get(&(x + dx, y + dy, z + dz)) {
                found.extend(list.iter().copied().filter(|&(source, at_start)| {
                    let (_, start, end, _) = all[source];
                    (if at_start { start } else { end }).sub(point).length() <= band
                }));
            }
        }}}
        found
    };
    let mut demoted = HashSet::default();
    for &(key, start, end, chord) in &short {
        if !mated.contains(&key) {
            continue;
        }
        let reach = 0.25 * chord;
        let side = |point: Vec3| {
            if point.sub(start).length() <= reach { Some(false) } else if point.sub(end).length() <= reach { Some(true) } else { None }
        };
        let ends_here = near(start);
        let needs_merge = ends_here.iter().any(|&(source, at_start)| {
            let (fragment, s0, s1, mid) = all[source];
            let (here, far) = if at_start { (s0, s1) } else { (s1, s0) };
            let Some(here_side) = side(here) else { return false };
            near(far).iter().any(|&(other, other_at_start)| {
                let (other_fragment, o0, o1, other_mid) = all[other];
                if other_fragment == fragment || other == source {
                    return false;
                }
                let (other_far, other_here) = if other_at_start { (o0, o1) } else { (o1, o0) };
                // A copy of the same edge: the far ends coincide as closely
                // as the short source's own ends must stay apart. Two edges
                // ending a short chord apart at both ends (the walls beside
                // a small arc) are two edges, not one edge shifted.
                other_far.sub(far).length() <= reach
                    && other_mid.sub(mid).length() <= band
                    && side(other_here).is_some_and(|other_side| other_side != here_side)
            })
        });
        if needs_merge {
            demoted.insert(key);
        }
    }
    if !demoted.is_empty() {
        // A demoted short source takes its mates with it: they are one edge.
        for &(key, start, end, chord) in &short {
            if !demoted.contains(&key) {
                continue;
            }
            for &(other, other_start, other_end, other_chord) in &short {
                let reach = 0.25 * chord.min(other_chord);
                let direct = start.sub(other_start).length().max(end.sub(other_end).length());
                let reversed = start.sub(other_end).length().max(end.sub(other_start).length());
                if direct.min(reversed) <= reach {
                    mated.remove(&other);
                }
            }
        }
    }
    Ok(mated)
}

/// [`short_source_mates`] over `(key, start, end, chord)` rows, the key's
/// first field naming the fragment: the keys of the rows another fragment's
/// row matches, ends in either order within a quarter of the shorter chord.
pub(super) fn mate_short_sources<K: Copy + Eq + std::hash::Hash>(
    short: &[((usize, K), Vec3, Vec3, f64)],
) -> HashSet<(usize, K)> {
    let mut mated = HashSet::default();
    for (a, &(key, start, end, chord)) in short.iter().enumerate() {
        for &(other, other_start, other_end, other_chord) in &short[a + 1..] {
            if other.0 == key.0 {
                continue;
            }
            let reach = 0.25 * chord.min(other_chord);
            let direct = start.sub(other_start).length().max(end.sub(other_end).length());
            let reversed = start.sub(other_end).length().max(end.sub(other_start).length());
            if direct.min(reversed) <= reach {
                mated.insert(key);
                mated.insert(other);
            }
        }
    }
    mated
}

pub(super) fn loop_feature_radii(sources: &[(SourceEdge, bool)], band: f64) -> Vec<[f64; 2]> {
    let count = sources.len();
    if count == 0 {
        return Vec::new();
    }
    let feature = |(source, _): &(SourceEdge, bool)| {
        let chord = source.start.sub(source.end).length();
        (source.distinct_ends && !source.degenerate && chord > FEATURE_WELD_FLOOR).then_some(chord)
    };
    let chords: Vec<Option<f64>> = sources.iter().map(feature).collect();
    let corner = |j: usize| {
        let (source, forward) = &sources[j];
        if *forward { source.start } else { source.end }
    };
    // Corner clusters: vertex j joins vertex j-1's cluster unless edge j-1
    // is a feature. A loop with no feature at all is one cluster.
    let mut cluster = vec![0usize; count];
    for j in 1..count {
        cluster[j] = cluster[j - 1] + usize::from(chords[j - 1].is_some());
    }
    let clusters = cluster[count - 1] + usize::from(chords[count - 1].is_some());
    if chords[count - 1].is_none() && clusters > 1 {
        // The wrap edge is not a feature: the tail run is the head cluster.
        let tail = cluster[count - 1];
        for value in cluster.iter_mut().filter(|value| **value == tail) {
            *value = 0;
        }
    }
    let clusters = clusters.max(1);
    let adjacent = |a: usize, b: usize| {
        a == b || (a + 1) % clusters == b || (b + 1) % clusters == a
    };
    let at_vertex = |j: usize| {
        let before = chords[(j + count - 1) % count].unwrap_or(f64::INFINITY);
        let mut shortest = before.min(chords[j].unwrap_or(f64::INFINITY));
        if count <= FEATURE_WELD_PAIRWISE_LOOP && clusters > 3 {
            let here = corner(j);
            for k in 0..count {
                if !adjacent(cluster[j], cluster[k]) {
                    shortest = shortest.min(corner(k).sub(here).length());
                }
            }
        }
        (0.25 * shortest).min(band).max(FEATURE_WELD_FLOOR)
    };
    let radii: Vec<f64> = (0..count).map(at_vertex).collect();
    (0..count)
        .map(|j| {
            let (head, tail) = (radii[j], radii[(j + 1) % count]);
            if sources[j].1 { [head, tail] } else { [tail, head] }
        })
        .collect()
}

pub(super) fn snap_edge_curve_endpoints(
    mut curve: crate::NurbsCurve,
    t0: f64,
    t1: f64,
    start: Vec3,
    end: Vec3,
) -> Result<crate::NurbsCurve, KernelRefusal> {
    let [domain_start, domain_end] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    // NurbsCurve::split refuses parameters within its ABSOLUTE knot
    // tolerance (1e-9) of the domain ends; the skip epsilon must cover that
    // or a near-edge trim parameter slips past the guard and errors.
    let epsilon = ((domain_end - domain_start).abs().max(1.0) * 1e-10).max(2e-9);
    if t0 > domain_start + epsilon && t0 < domain_end - epsilon {
        curve = curve.split(t0).or_refuse(KernelStage::Sew, "split")?.1;
    }
    let domain = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    if t1 < domain[1] - epsilon && t1 > domain[0] + epsilon {
        curve = curve.split(t1).or_refuse(KernelStage::Sew, "split")?.0;
    } else if env_read!("BREP_DEBUG_SUBRANGE").is_some() && t1 < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in snap_edge_curve_endpoints");
    }
    let mut controls = curve.control_points.clone();
    let first_weight = controls[0].w;
    controls[0] = crate::Vec4::from_point(start, first_weight);
    let last = controls.len() - 1;
    let last_weight = controls[last].w;
    controls[last] = crate::Vec4::from_point(end, last_weight);
    let snapped = crate::NurbsCurve::new(curve.degree, curve.knots.clone(), controls).or_refuse(KernelStage::Sew, "NurbsCurve::new")?;
    Ok(snapped)
}

impl Assembler {
    fn weld_candidate_indices(&mut self, key: Option<(u8, u64)>) -> Vec<usize> {
        // Edges are append-only throughout building. Identity is assigned on
        // creation and never changes when another use joins an edge.
        for edge in &self.edges[self.weld_candidates.indexed..] {
            self.weld_candidates.push(self.edge_boundary_key.get(&edge.id).copied());
        }
        self.weld_candidates.candidates(key)
    }

    fn cellular_boundary_matches(&self, source: Option<(u8, u64)>, candidate: u64) -> bool {
        if !self.cellular { return true; }
        let Some(key) = source else { return false; };
        if let Some(&mapped) = self.edge_by_boundary_key.get(&key) {
            return mapped == candidate;
        }
        // Never join two DIFFERENT edges of one region merely because
        // their geometry coincides. The other use of a region's edge must
        // follow its established source correspondence.
        !self.edge_by_boundary_key.iter()
            .any(|(&(operand, _), &edge)| operand == key.0 && edge == candidate)
    }

    /// Vertex ids are dense (1..=len) because only `vertex()` appends with a
    /// sequential id. This lookup checks that invariant instead of assuming
    /// it, so a future change that breaks density fails loudly rather than
    /// silently welding edges to the wrong vertex position.
    fn vertex_point(&self, id: u64) -> Result<Vec3, KernelRefusal> {
        let id = self.canonical_vertex(id);
        let record = id
            .checked_sub(1)
            .and_then(|offset| usize::try_from(offset).ok())
            .and_then(|index| self.vertices.get(index))
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!("assembler: vertex id {id} out of range")))?;
        if record.id != id {
            return Err(KernelRefusal::internal(KernelStage::Sew, "assemble.builder", format!(
                "assembler: vertex ids are not dense (id {id} resolved to record {})",
                record.id
            )));
        }
        Ok(record.point)
    }

    fn canonical_vertex(&self, mut id: u64) -> u64 {
        while let Some(&next) = self.vertex_aliases.get(&id) {
            id = next;
        }
        id
    }

    fn join_weld_endpoints(
        &mut self, source_start: Vec3, source_end: Vec3,
        target_start: u64, target_end: u64, reversed: bool,
    ) -> Result<bool, KernelRefusal> {
        self.join_weld_endpoints_within(source_start, source_end, target_start, target_end, reversed, None)
    }

    /// [`Self::join_weld_endpoints`] with the source's feature radii: under
    /// `BREP_ASSEMBLY_FEATURE_WELD=on` no vertex moves farther than its own
    /// radius.
    fn join_weld_endpoints_within(
        &mut self, source_start: Vec3, source_end: Vec3,
        target_start: u64, target_end: u64, reversed: bool,
        feature_radius: Option<[f64; 2]>,
    ) -> Result<bool, KernelRefusal> {
        let source_ids = [
            self.vertex_within(source_start, feature_radius.map(|radius| radius[0])),
            self.vertex_within(source_end, feature_radius.map(|radius| radius[1])),
        ];
        let target_ids = if reversed { [target_end, target_start] }
                         else { [target_start, target_end] };
        let mut aliases = self.vertex_aliases.clone();
        fn root(aliases: &HashMap<u64, u64>, mut id: u64) -> u64 {
            while let Some(&next) = aliases.get(&id) { id = next; }
            id
        }
        for (from, to) in source_ids.into_iter().zip(target_ids) {
            let (from, to) = (root(&aliases, from), root(&aliases, to));
            if from != to { aliases.insert(from, to); }
        }
        if aliases == self.vertex_aliases { return Ok(true); }
        let band = self.tolerance.max(1e-7);
        let feature_on = self.feature.mode == FeatureWeldMode::On;
        for vertex in &self.vertices {
            let target = root(&aliases, vertex.id);
            // A vertex no loop gave a radius keeps the band on its own
            // side; the move is held to the smaller reach of the two ends.
            let reach = if feature_on {
                self.vertex_radius(vertex.id, band).min(self.vertex_radius(target, band))
            } else {
                band
            };
            if vertex.point.sub(self.vertex_point(target)?).length() > reach {
                return Ok(false);
            }
        }
        if self.edges.iter().any(|edge| !edge.degenerate
            && edge.start_vertex_id != edge.end_vertex_id
            && root(&aliases, edge.start_vertex_id) == root(&aliases, edge.end_vertex_id))
        {
            return Ok(false);
        }
        // Validate every geometric edit before committing the correspondence.
        let mut replacements = Vec::new();
        for (index, edge) in self.edges.iter().enumerate() {
            let start = root(&aliases, edge.start_vertex_id);
            let end = root(&aliases, edge.end_vertex_id);
            if (start, end) == (edge.start_vertex_id, edge.end_vertex_id) { continue; }
            let curve = snap_edge_curve_endpoints(edge.curve.clone(), edge.t0, edge.t1,
                self.vertex_point(start)?, self.vertex_point(end)?)?;
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            replacements.push((index, start, end, curve, t0, t1));
        }
        for (index, start, end, curve, t0, t1) in replacements {
            let edge = &mut self.edges[index];
            edge.start_vertex_id = start;
            edge.end_vertex_id = end;
            edge.curve = curve;
            edge.t0 = t0;
            edge.t1 = t1;
        }
        self.vertex_aliases = aliases;
        Ok(true)
    }

    fn vertex(&mut self, point: Vec3) -> u64 {
        self.vertex_within(point, None)
    }

    /// The vertex at `point`: the first within the weld band, or, under
    /// `BREP_ASSEMBLY_FEATURE_WELD`, the first within both this point's
    /// feature `radius` and the found vertex's own (see [`FeatureWeld`]).
    fn vertex_within(&mut self, point: Vec3, radius: Option<f64>) -> u64 {
        // Offset/intersection endpoints are independently fitted and can
        // differ by the sewing tolerance. Welding those endpoint vertices is
        // what makes consecutive coedges form a topological loop; the edge
        // geometry itself remains unchanged.
        let tolerance = self.tolerance.max(1e-7);
        let band = self.vertex_lookup(point, tolerance, None);
        let mode = self.feature.mode;
        let found = if mode == FeatureWeldMode::Off {
            band
        } else {
            let own = radius.unwrap_or(tolerance).min(tolerance);
            let feature = self.vertex_lookup(point, tolerance, Some(own));
            if mode == FeatureWeldMode::Check && feature != band {
                let (band_id, feature_id) = (band.map(|id| self.canonical_vertex(id)), feature.map(|id| self.canonical_vertex(id)));
                self.feature.report(format_args!(
                    "vertex at ({:.9},{:.9},{:.9}) band={tolerance:.3e} radius={own:.3e}: band welds to {band_id:?}, feature to {feature_id:?}",
                    point.x, point.y, point.z,
                ));
            }
            if mode == FeatureWeldMode::On { feature } else { band }
        };
        if let Some(record) = found {
            // The radius rides the vertex RECORD the lookup admits by.
            if mode == FeatureWeldMode::On {
                if let Some(radius) = radius {
                    let held = self.feature.radius.entry(record).or_insert(radius);
                    *held = held.min(radius);
                }
            }
            return self.canonical_vertex(record);
        }
        let id = self.next_vertex_id;
        self.next_vertex_id += 1;
        self.vertices.push(VertexRecord { id, point });
        if mode != FeatureWeldMode::Off {
            if let Some(radius) = radius {
                self.feature.radius.insert(id, radius.min(tolerance));
            }
        }
        id
    }

    /// The first vertex within `tolerance` of `point`; with `feature`, only
    /// one also within that radius and within its own recorded radius.
    fn vertex_lookup(&mut self, point: Vec3, tolerance: f64, feature: Option<f64>) -> Option<u64> {
        let radii = &self.feature.radius;
        let admit = |id: u64, distance: f64| match feature {
            None => true,
            Some(own) => distance <= own.min(radii.get(&id).copied().unwrap_or(tolerance)),
        };
        match &mut self.weld_candidates.vertex_candidates {
            Some(index) => index.first_admitted(&self.vertices, point, tolerance, admit),
            None => self
                .vertices
                .iter()
                .find(|vertex| {
                    let distance = vertex.point.sub(point).length();
                    distance <= tolerance && admit(vertex.id, distance)
                })
                .map(|vertex| vertex.id),
        }
    }

    /// The feature radius recorded at a vertex (the band where none is).
    fn vertex_radius(&self, id: u64, band: f64) -> f64 {
        let held = |id: u64| self.feature.radius.get(&id).copied();
        held(id).or_else(|| held(self.canonical_vertex(id))).unwrap_or(band).min(band)
    }

    pub fn edge(
        &mut self,
        source: SourceEdge,
        incoming_forward: bool,
    ) -> Result<(u64, bool), KernelRefusal> {
        let weld_tolerance = assembler_weld(self.tolerance);
        let identity_tolerance = self.tolerance.max(1e-7);
        let chord = source.start.sub(source.end).length();
        let band_degenerate = source.degenerate || chord <= identity_tolerance;
        // `BREP_ASSEMBLY_FEATURE_WELD`: a source that is a feature
        // (`SourceEdge::distinct_ends`; a short one only when another face
        // carries its mate, `short_source_mates`) is degenerate only when
        // flagged or under the model floor; any other keeps the band reading.
        let feature_mode = self.feature.mode;
        let feature_radius = source.feature_radius.filter(|_| feature_mode != FeatureWeldMode::Off);
        let feature_degenerate = match feature_radius {
            Some(_) if source.distinct_ends => source.degenerate || chord <= FEATURE_WELD_FLOOR,
            Some(_) => band_degenerate,
            None => band_degenerate,
        };
        let kind = match (source.boundary_key, source.piece) {
            (Some(_), _) => "boundary",
            (None, Some(_)) => "imprint",
            (None, None) => "derived",
        };
        if feature_mode == FeatureWeldMode::Check && feature_degenerate != band_degenerate {
            self.feature.report(format_args!(
                "{kind} edge ({:.9},{:.9},{:.9})->({:.9},{:.9},{:.9}) chord={chord:.3e} band={identity_tolerance:.3e}: band reads degenerate={band_degenerate}, feature {feature_degenerate}",
                source.start.x, source.start.y, source.start.z, source.end.x, source.end.y, source.end.z,
            ));
        }
        let source_degenerate =
            if feature_mode == FeatureWeldMode::On { feature_degenerate } else { band_degenerate };
        let source_kind = kind;
        // The pairing bands: today's band, and the feature weld's, bounded by
        // the source's own end radii and a quarter of its chord.
        let source_pair_radius = feature_radius.map(|[start, end]| start.min(end));
        let feature_curve_band = source_pair_radius
            .map(|_| weld_tolerance.min(0.25 * chord).max(FEATURE_WELD_FLOOR))
            .unwrap_or(weld_tolerance);
        // A degenerate pole edge is a face-local topological placeholder,
        // not a shared boundary. Geometrically coincident pole edges must
        // remain distinct so each is referenced by exactly one coedge.
        if !source_degenerate {
            let mut welded: Option<(usize, bool)> = None;
            for index in self.weld_candidate_indices(source.boundary_key) {
                let edge = &self.edges[index];
                if (!self.cellular && self.edge_use_counts.get(&edge.id).copied().unwrap_or(0) >= 2)
                    || !self.cellular_boundary_matches(source.boundary_key, edge.id) {
                    continue;
                }
                if weld_refused_by_identity(
                    source.boundary_key,
                    self.edge_boundary_key.get(&edge.id).copied(),
                ) {
                    continue;
                }
                if pieces_apart(source.piece, self.edge_piece.get(&edge.id).copied(), &source, edge, identity_tolerance) {
                    continue;
                }
                // A tangent section's coedges pair by material side (see
                // `SourceEdge::tangent_side`): a candidate recorded with the
                // opposite side bounds the other sliver of the pinch.
                if let (Some(side), Some(existing)) =
                    (source.tangent_side, self.edge_tangent_side.get(&edge.id))
                {
                    if side.dot(*existing) < 0.0 {
                        continue;
                    }
                }
                let start = self.vertex_point(edge.start_vertex_id)?;
                let end = self.vertex_point(edge.end_vertex_id)?;
                // Feature weld: identity at the smaller of the source's and
                // the candidate's end radii.
                let feature_identity = source_pair_radius.map(|own| {
                    own.min(self.vertex_radius(edge.start_vertex_id, identity_tolerance))
                        .min(self.vertex_radius(edge.end_vertex_id, identity_tolerance))
                        .min(identity_tolerance)
                });
                let candidate_chord = start.sub(end).length();
                let direct_gap = start.sub(source.start).length()
                    .max(end.sub(source.end).length());
                let reversed_gap = start.sub(source.end).length()
                    .max(end.sub(source.start).length());
                // A broad curve-comparison band cannot establish endpoint
                // identity. A later conformance piece may connect these two
                // nearby vertices, even if that short edge is not built yet.
                let ends_match = |identity: f64, floor: f64| -> Option<bool> {
                    if edge.degenerate || candidate_chord <= floor {
                        return None;
                    }
                    let direct = direct_gap <= identity;
                    let reversed = reversed_gap <= identity && (!direct || reversed_gap < direct_gap);
                    (direct || reversed).then_some(reversed)
                };
                let band_ends = ends_match(identity_tolerance, identity_tolerance);
                let feature_ends = feature_identity.map(|identity| ends_match(identity, FEATURE_WELD_FLOOR));
                if feature_mode == FeatureWeldMode::Check {
                    if let Some(feature_ends) = feature_ends.filter(|ends| *ends != band_ends) {
                        self.feature.report(format_args!(
                            "{source_kind} pair ({:.9},{:.9},{:.9})->({:.9},{:.9},{:.9}) with edge {}: band ends {band_ends:?}, feature {feature_ends:?} (gaps {direct_gap:.3e}/{reversed_gap:.3e})",
                            source.start.x, source.start.y, source.start.z, source.end.x, source.end.y, source.end.z, edge.id,
                        ));
                    }
                }
                let (ends, curve_band) = match (feature_mode, feature_ends) {
                    (FeatureWeldMode::On, Some(ends)) => (ends, feature_curve_band),
                    _ => (band_ends, weld_tolerance),
                };
                let Some(reversed) = ends else {
                    continue;
                };
                let edge = &self.edges[index];
                let weld_tolerance = curve_band;
                let same_curve = [0.2, 0.5, 0.8].into_iter().all(|fraction| {
                    let source_parameter = source.t0 + (source.t1 - source.t0) * fraction;
                    let edge_fraction = if reversed { 1.0 - fraction } else { fraction };
                    let edge_parameter = edge.t0 + (edge.t1 - edge.t0) * edge_fraction;
                    source
                        .curve
                        .evaluate(source_parameter)
                        .and_then(|source_point| {
                            edge.curve
                                .evaluate(edge_parameter)
                                .map(|edge_point| source_point.sub(edge_point).length())
                        })
                        .is_ok_and(|distance| distance <= weld_tolerance)
                });
                // Rational arcs parameterize non-uniformly, so the same
                // circular span represented as a fresh arc vs a subrange of
                // a longer arc evaluates DIFFERENT points at equal
                // fractions. When fraction sampling disagrees, accept the
                // weld if the source samples project onto the candidate's
                // trimmed curve within tolerance.
                let same_curve = same_curve
                    || [0.2, 0.5, 0.8].into_iter().all(|fraction| {
                        let source_parameter = source.t0 + (source.t1 - source.t0) * fraction;
                        source
                            .curve
                            .evaluate(source_parameter)
                            .ok()
                            .and_then(|point| project_point_to_curve(&edge.curve, point).ok())
                            .is_some_and(|projection| {
                                let low = edge.t0.min(edge.t1);
                                let high = edge.t0.max(edge.t1);
                                let margin = (high - low).abs() * 1e-6 + 1e-12;
                                projection.distance <= weld_tolerance
                                    && projection.u >= low - margin
                                    && projection.u <= high + margin
                            })
                    }) && (!crate::switch!("BREP_NEAR_SEAM_RIM_SPLIT", true)
                        || reaches_into(&edge.curve, edge.t0, edge.t1, &source.curve, source.t0, source.t1, weld_tolerance));
                if same_curve {
                    // Two coedges on one edge must have OPPOSITE sense; a
                    // same-sense pair is non-manifold (topology.rs). When a
                    // coincident twin edge sits within the weld radius (thin
                    // rims one weld-tolerance apart on imperfect imported
                    // geometry), the first geometric match is not necessarily
                    // the true mate. Refuse a weld that would collide senses
                    // and keep scanning for a candidate that pairs opposite —
                    // falling through to a fresh edge record if none does, so
                    // the real mate can still claim it later.
                    let would_be_forward = if reversed {
                        !incoming_forward
                    } else {
                        incoming_forward
                    };
                    if !self.cellular && self.edge_first_forward.get(&edge.id) == Some(&would_be_forward) {
                        continue;
                    }
                    let endpoints = (edge.start_vertex_id, edge.end_vertex_id);
                    if !self.join_weld_endpoints_within(source.start, source.end,
                        endpoints.0, endpoints.1, reversed, feature_radius)? { continue; }
                    welded = Some((index, reversed));
                    break;
                }
            }
            if let Some((index, reversed)) = welded {
                let edge = &mut self.edges[index];
                // The mate arriving from the other operand may carry the
                // only name for this shared edge; keep the first name seen.
                if edge.name.is_none() {
                    edge.name = source.name;
                }
                let id = edge.id;
                *self.edge_use_counts.entry(id).or_default() += 1;
                if self.cellular {
                    if let Some(key) = source.boundary_key { self.edge_by_boundary_key.entry(key).or_insert(id); }
                }
                return Ok((id, reversed));
            }
        }
        // A CLOSED edge (full circle/seam: start == end but real interior
        // geometry) is shared boundary topology exactly like an open edge —
        // a cap of a closed surface is one closed edge referenced by both
        // adjacent faces (Golovanov §5.6). Without this weld the two
        // operands' coincident copies each stay one-use and the shell splits
        // at the seam (the SweepFace cone-seam failure). Only true point
        // placeholders (poles) stay face-local.
        let interior_sweeps_away =
            |curve: &crate::NurbsCurve, t0: f64, t1: f64, anchor: Vec3| -> bool {
                [0.25, 0.5, 0.75].into_iter().any(|fraction| {
                    curve
                        .evaluate(t0 + (t1 - t0) * fraction)
                        .is_ok_and(|point| point.sub(anchor).length() > 10.0 * weld_tolerance)
                })
            };
        // Admit ANY genuinely closed source — not only degenerate-flagged
        // ones. The scan was built for the cone-seam class whose closed
        // edges arrive flagged degenerate, but a REAL closed ring (the
        // inscribed sphere's equator presented as the cylinder's own cap
        // edge via SharedBoundary) is equally shared boundary topology, and
        // without admission the identical-key copies stay one-use forever.
        // Every downstream gate (identity refusal, seam match, 7-point
        // same-curve, orientation, same-sense refusal) still applies.
        let admit_any_closed =
            std::env::var("BREP_CLOSED_WELD_ANY").as_deref() != Ok("0");
        let source_closed = (source_degenerate || admit_any_closed)
            && source.start.sub(source.end).length() <= identity_tolerance
            && interior_sweeps_away(&source.curve, source.t0, source.t1, source.start);
        if source_closed && std::env::var("BREP_DEBUG_EDGE").as_deref() == Ok("weld") {
            eprintln!(
                "closed-weld scan: source start=({:.3},{:.3},{:.3}) key={:?} candidates={}",
                source.start.x, source.start.y, source.start.z,
                source.boundary_key,
                self.edges.len()
            );
        }
        if source_closed {
            let mut welded: Option<(usize, bool)> = None;
            for index in self.weld_candidate_indices(source.boundary_key) {
                let edge = &self.edges[index];
                if (!self.cellular && self.edge_use_counts.get(&edge.id).copied().unwrap_or(0) >= 2)
                    || !self.cellular_boundary_matches(source.boundary_key, edge.id) {
                    continue;
                }
                if weld_refused_by_identity(
                    source.boundary_key,
                    self.edge_boundary_key.get(&edge.id).copied(),
                ) {
                    continue;
                }
                // A tangent section's coedges pair by material side (see
                // `SourceEdge::tangent_side`): a candidate recorded with the
                // opposite side bounds the other sliver of the pinch.
                if let (Some(side), Some(existing)) =
                    (source.tangent_side, self.edge_tangent_side.get(&edge.id))
                {
                    if side.dot(*existing) < 0.0 {
                        continue;
                    }
                }
                let start = self.vertex_point(edge.start_vertex_id)?;
                let end = self.vertex_point(edge.end_vertex_id)?;
                // The candidate must be a closed edge with real interior
                // geometry sharing the seam vertex; a different seam
                // position means the mate's loop starts elsewhere and the
                // copies cannot share topology.
                if start.sub(end).length() > identity_tolerance
                    || start.sub(source.start).length() > identity_tolerance
                    || !interior_sweeps_away(&edge.curve, edge.t0, edge.t1, start)
                {
                    continue;
                }
                let same_curve = [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875]
                    .into_iter()
                    .all(|fraction| {
                        source
                            .curve
                            .evaluate(source.t0 + (source.t1 - source.t0) * fraction)
                            .ok()
                            .and_then(|point| project_point_to_curve(&edge.curve, point).ok())
                            .is_some_and(|projection| projection.distance <= weld_tolerance)
                    });
                if !same_curve {
                    continue;
                }
                // Quarter-point orientation disambiguation: a closed edge's
                // endpoints coincide, so direction must come from the
                // interior, not the endpoint pairing.
                let span = edge.t1 - edge.t0;
                let quarter_param = source.t0 + (source.t1 - source.t0) * 0.25;
                let quarter = source.curve.evaluate(quarter_param).or_refuse(KernelStage::Sew, "evaluate")?;
                let direct_gap = edge
                    .curve
                    .evaluate(edge.t0 + span * 0.25).or_refuse(KernelStage::Sew, "evaluate")?
                    .sub(quarter)
                    .length();
                let reversed_gap = edge
                    .curve
                    .evaluate(edge.t0 + span * 0.75).or_refuse(KernelStage::Sew, "evaluate")?
                    .sub(quarter)
                    .length();
                let reversed = if direct_gap <= weld_tolerance || reversed_gap <= weld_tolerance {
                    reversed_gap < direct_gap
                } else {
                    // Phase-shifted parameterizations of the same closed
                    // curve: align tangents at the geometric match instead.
                    let projection = project_point_to_curve(&edge.curve, quarter).or_refuse(KernelStage::Sew, "project_point_to_curve")?;
                    let source_tangent = source.curve.derivatives(quarter_param, 1).or_refuse(KernelStage::Sew, "derivatives")?[1];
                    let edge_tangent = edge.curve.derivatives(projection.u, 1).or_refuse(KernelStage::Sew, "derivatives")?[1];
                    source_tangent.dot(edge_tangent) < 0.0
                };
                // Same opposite-sense guard as the open-edge path: a closed
                // rim shared by more than two faces splits into coincident
                // twin edges, and the true mate is the one that pairs opposite
                // sense — not merely the first geometric match within the weld
                // radius. Refuse a same-sense weld and keep scanning.
                let would_be_forward = if reversed {
                    !incoming_forward
                } else {
                    incoming_forward
                };
                if !self.cellular && self.edge_first_forward.get(&edge.id) == Some(&would_be_forward) {
                    if std::env::var("BREP_DEBUG_EDGE").as_deref() == Ok("weld") {
                        eprintln!(
                            "closed-weld REFUSED same-sense: edge {} first_forward={:?} incoming would_be={}",
                            edge.id,
                            self.edge_first_forward.get(&edge.id),
                            would_be_forward
                        );
                    }
                    continue;
                }
                let endpoints = (edge.start_vertex_id, edge.end_vertex_id);
                if !self.join_weld_endpoints(source.start, source.end,
                    endpoints.0, endpoints.1, reversed)? { continue; }
                welded = Some((index, reversed));
                break;
            }
            if let Some((index, reversed)) = welded {
                let edge = &mut self.edges[index];
                if edge.name.is_none() {
                    edge.name = source.name;
                }
                // A successfully welded closed edge IS shared real topology
                // (one cap circle referenced by both adjacent faces —
                // Golovanov §5.6): clear the historical degenerate marking
                // so the Euler count and the degenerate-use validation see
                // the real 1-cell.  Unwelded closed edges keep the marking
                // — the offset pipeline still builds rim pairs with
                // mismatched representations (a closed circle on one
                // side, an open fitted chain on the other), which
                // cannot weld.
                edge.degenerate = false;
                let id = edge.id;
                *self.edge_use_counts.entry(id).or_default() += 1;
                if self.cellular {
                    if let Some(key) = source.boundary_key { self.edge_by_boundary_key.entry(key).or_insert(id); }
                }
                return Ok((id, reversed));
            }
        }
        let start_vertex_id = self.vertex_within(source.start, feature_radius.map(|radius| radius[0]));
        let end_vertex_id = self.vertex_within(source.end, feature_radius.map(|radius| radius[1]));
        let canonical_start = self.vertex_point(start_vertex_id)?;
        let canonical_end = self.vertex_point(end_vertex_id)?;
        // A degenerate source can be a true point placeholder (pole), a
        // CLOSED curve whose endpoints coincide (full circle/seam — real
        // geometry that must be kept), or a sub-tolerance sliver subrange of
        // a real curve (an intersection landing on a seam) whose interior
        // still sweeps measurably away from the vertex. Only the sliver is
        // rewritten: sampling confirms the whole trimmed span stays at the
        // vertex scale, then the canonical point-collapsed form (as analytic
        // pole edges use) replaces it so the curve-end-matches-vertex and
        // pcurve-consistency validations both hold.
        let collapses_to_vertex = source_degenerate
            && start_vertex_id == end_vertex_id
            && [0.25, 0.5, 0.75, 1.0].into_iter().all(|fraction| {
                let parameter = source.t0 + (source.t1 - source.t0) * fraction;
                source
                    .curve
                    .evaluate(parameter)
                    .is_ok_and(|point| point.sub(canonical_start).length() <= 10.0 * assembler_weld(self.tolerance))
            });
        // Compare the CURVE'S evaluated endpoints against the canonical
        // vertices, not the source's claimed endpoints: imprint stamps the
        // exact split positions into start/end, but the stored curve (a
        // fitted polyline, say) can evaluate measurably away from them at
        // t0/t1 — trusting the claim then skips the snap and validation
        // rejects the edge (offset-target subtract, gap 1e-4).
        let evaluated_start = source.curve.evaluate(source.t0).or_refuse(KernelStage::Sew, "evaluate")?;
        let evaluated_end = source.curve.evaluate(source.t1).or_refuse(KernelStage::Sew, "evaluate")?;
        let (curve, edge_t0, edge_t1) = if collapses_to_vertex {
            (
                crate::make_line(canonical_start, canonical_start).or_refuse(KernelStage::Sew, "make_line")?,
                0.0,
                1.0,
            )
        } else if canonical_start.sub(evaluated_start).length() > 1e-8
            || canonical_end.sub(evaluated_end).length() > 1e-8
        {
            let curve = snap_edge_curve_endpoints(
                source.curve,
                source.t0,
                source.t1,
                canonical_start,
                canonical_end,
            )?;
            // Splitting changes the represented curve to the requested
            // subrange. Keep the edge interval synchronized with that actual
            // curve domain; retaining the pre-split interval can evaluate an
            // interior point instead of the newly constrained endpoint.
            let [t0, t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            (curve, t0, t1)
        } else {
            (source.curve, source.t0, source.t1)
        };
        let record_floor = match (feature_mode, feature_radius) {
            (FeatureWeldMode::On, Some(_)) if source.distinct_ends => FEATURE_WELD_FLOOR,
            _ => identity_tolerance,
        };
        let id = self.next_edge_id;
        if std::env::var("BREP_DEBUG_EDGE").is_ok_and(|v| v == id.to_string()) {
            let eval_start = curve.evaluate(edge_t0);
            let eval_end = curve.evaluate(edge_t1);
            eprintln!(
                "EDGE {id}: src start={:?} end={:?} sep={:.9} degen={} t=[{:.9},{:.9}]\n\
                 canon start={canonical_start:?} end={canonical_end:?}\n\
                 curve degree={} knots[0..4]={:?} knots[-4..]={:?} nctrl={}\n\
                 eval(t0)={eval_start:?}\n eval(t1)={eval_end:?}",
                source.start,
                source.end,
                source.start.sub(source.end).length(),
                source_degenerate,
                edge_t0,
                edge_t1,
                curve.degree,
                &curve.knots[..4.min(curve.knots.len())],
                &curve.knots[curve.knots.len().saturating_sub(4)..],
                curve.control_points.len(),
            );
        }
        self.next_edge_id += 1;
        self.edges.push(EdgeRecord {
            id,
            curve,
            t0: edge_t0,
            t1: edge_t1,
            start_vertex_id,
            end_vertex_id,
            // Historical marking: closed and sub-tolerance edges stay
            // face-local unless a closed weld later claims them (the weld
            // clears this flag on success — see above).
            // Under the feature weld a chord within the band is still a
            // real edge, as it was for `source_degenerate`.
            degenerate: source_degenerate
                || start_vertex_id == end_vertex_id
                || canonical_start.sub(canonical_end).length() <= record_floor,
            name: source.name,
        });
        self.edge_use_counts.insert(id, 1);
        // A fresh edge is created reversed=false, so its first coedge's result
        // `forward` equals the incoming `forward`. Record it so a later weld
        // can enforce the opposite-sense (manifold) pairing.
        self.edge_first_forward.insert(id, incoming_forward);
        if let Some(key) = source.boundary_key {
            self.edge_boundary_key.insert(id, key);
            self.edge_by_boundary_key.entry(key).or_insert(id);
        }
        if source.constructed {
            self.imprint_edges.insert(id);
        }
        if let Some(piece) = source.piece {
            self.edge_piece.insert(id, piece);
        }
        if let Some(side) = source.tangent_side {
            self.edge_tangent_side.insert(id, side);
        }
        Ok((id, false))
    }

    /// [`edge`](Self::edge) for a coedge of a face the operation did not
    /// touch. Such a face's operand edge is whole and coincides with no
    /// section, so an assembler edge already minted from the SAME operand
    /// edge is its mate: the curve, span and endpoints are the operand's on
    /// both sides, and the geometric scan would read the same record. The
    /// scan still answers everything else — a closed or degenerate source,
    /// a first use, a mate already two-use or facing the same way (a
    /// non-manifold input), or a key the scan may have welded onto another
    /// operand's coincident edge first.
    pub(in crate::boolean) fn edge_untouched(
        &mut self,
        source: SourceEdge,
        incoming_forward: bool,
    ) -> Result<(u64, bool), KernelRefusal> {
        let identity_tolerance = self.tolerance.max(1e-7);
        let open = !source.degenerate && source.start.sub(source.end).length() > identity_tolerance;
        if let (true, Some(key)) = (open, source.boundary_key) {
            if let Some(&id) = self.edge_by_boundary_key.get(&key) {
                let uses = self.edge_use_counts.get(&id).copied().unwrap_or(0);
                let same_sense = self.edge_first_forward.get(&id) == Some(&incoming_forward);
                if let Ok(index) = self.edges.binary_search_by_key(&id, |edge| edge.id) {
                    let edge = &mut self.edges[index];
                    if uses < 2 && !same_sense && !edge.degenerate {
                        if edge.name.is_none() {
                            edge.name = source.name;
                        }
                        *self.edge_use_counts.entry(id).or_default() += 1;
                        return Ok((id, false));
                    }
                }
            }
        }
        self.edge(source, incoming_forward)
    }
}

/// A weld candidate is refused when the incoming source and the existing edge
/// both descend from the SAME operand's boundary but from DIFFERENT original
/// edge ids: two distinct 1-cells of a valid input solid are never one shared
/// edge, however close their geometry runs. Anything else (cross-operand, or
/// an imprint/derived source on either side) is left to the geometric +
/// opposite-sense checks that follow.
/// Whether two different imprint pieces stand apart (`SourceEdge::piece`,
/// `BREP_FEATURE_SCALE_LENS`): their curves' middles farther than ten
/// identity tolerances from each other's curve.
fn pieces_apart(incoming: Option<u64>, existing: Option<u64>, source: &SourceEdge, edge: &EdgeRecord, identity_tolerance: f64) -> bool {
    let (Some(a), Some(b)) = (incoming, existing) else { return false };
    if a == b || !crate::switch!("BREP_FEATURE_SCALE_LENS", true) {
        return false;
    }
    let middle = |curve: &crate::NurbsCurve, t0: f64, t1: f64| curve.evaluate(0.5 * (t0 + t1)).ok();
    let gap = |point: Option<Vec3>, curve: &crate::NurbsCurve| point.and_then(|p| project_point_to_curve(curve, p).ok()).map(|foot| foot.distance);
    let band = 10.0 * identity_tolerance;
    gap(middle(&source.curve, source.t0, source.t1), &edge.curve).is_some_and(|d| d > band)
        || gap(middle(&edge.curve, edge.t0, edge.t1), &source.curve).is_some_and(|d| d > band)
}

pub(super) fn weld_refused_by_identity(
    incoming: Option<(u8, u64)>,
    existing: Option<(u8, u64)>,
) -> bool {
    match (incoming, existing) {
        (Some((op_a, id_a)), Some((op_b, id_b))) => op_a == op_b && id_a != id_b,
        _ => false,
    }
}


/// Whether the middle of `curve` over [`t0`, `t1`] lies on `other` over
/// [`other_t0`, `other_t1`] within `tolerance`. The projection fallback of the
/// weld tests one direction only (the source's samples on the candidate), and
/// its parameter margin is relative: on a closed rim split a few 1e-6 from
/// its seam, the short piece's samples all land inside the margin of the
/// long piece, so the two pieces welded into one edge used twice by the same
/// face. A real mate also holds the candidate's middle in the source's span.
fn reaches_into(curve: &crate::NurbsCurve, t0: f64, t1: f64, other: &crate::NurbsCurve, other_t0: f64, other_t1: f64, tolerance: f64) -> bool {
    let Ok(middle) = curve.evaluate(0.5 * (t0 + t1)) else { return false };
    let Ok(projection) = project_point_to_curve(other, middle) else { return false };
    // Measured in model space at the span's nearest point: a parameter
    // margin would again be relative to the span.
    let (low, high) = (other_t0.min(other_t1), other_t0.max(other_t1));
    projection.distance <= tolerance
        && other.evaluate(projection.u.clamp(low, high)).is_ok_and(|nearest| nearest.sub(middle).length() <= tolerance)
}

