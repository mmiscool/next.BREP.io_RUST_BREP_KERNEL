//! # Why an entity needs its own band
//!
//! Everything in [`crate::tolerance`] is a *policy*: one spatial base `model`,
//! size-coupled at the site through [`KernelTolerances::heal_band`], with named
//! weld floors. That policy answers "how accurate is geometry in this kernel?"
//! with ONE number, and so it cannot say the thing a dirty import needs said:
//! *this* edge is known to be 4.7e-4 sloppy while *that* one is exact. A vendor
//! STEP file commits an edge's 3D curve and its face surfaces as INDEPENDENT
//! approximations; where they disagree, two intersections computed against the
//! two surfaces sharing that edge cannot agree better than the disagreement,
//! no matter how good the marcher is. The residual is a property OF THE EDGE.
//!
//! So: `tol(E) = min(cap_E, max(policy floor, d_meas(E)))`, where `d_meas(E)`
//! is the largest disagreement the edge's own representations show — exactly
//! the quantity [`crate::measure_edge_against_pcurve_image`] measures, taken
//! over every coedge that references the edge.
//!
//! # The direction rule, and why this type may hand out a band at all
//!
//! [`crate::MeasuredTolerance`] deliberately exposes no "band to use" accessor:
//! a *construction* that measured its own error must never widen the band that
//! judges it, or the sloppier the fit the more forgiving its grade. That is
//! invariant **I1**, and it is not weakened here.
//!
//! This type is the other case. Its input is not a fit this kernel just
//! produced but the disagreement already present in geometry the kernel was
//! HANDED, and the band it returns is spent only on *search / pairing*
//! questions — "which of these candidates might be the same point?" — never on
//! an acceptance gate. The acceptance gates ([`crate::BrepSolid::validate`],
//! the boolean's own assembly gate) still run afterwards on the global policy
//! band and still refuse, so a wider search band can only change WHICH
//! candidates are considered, never whether a wrong answer is accepted. Two
//! structural properties keep that honest:
//!
//! * **Capped** (`cap_E`, `cap_V` below). A measurement above the cap does not
//!   buy a wider band: the entity is *defective*, its band pins at the cap, and
//!   the acceptance gate still fails it. We refuse; we do not widen.
//! * **Local.** The band belongs to one edge or one vertex. A poor fit on one
//!   imported edge cannot authorize an unrelated pair of vertices, or a thin
//!   feature elsewhere, to merge — which is the failure mode the plan names.
//!
//! And **I2**: an entity with no measurable band degrades to the policy floor —
//! *narrower*, so more refusals, never a wrong weld. Every consumer therefore
//! reads `max(its own global band, tol(entity))` and behaves exactly as it did
//! before wherever the measurement is absent or zero. Constructed geometry
//! measures ~0, so nothing this kernel builds itself changes behaviour.
//!
//! # Lazy, not stored — which is also the cache-invalidation answer
//!
//! The view borrows a solid and memoizes per id. It is constructed on the
//! operation's ENTRY snapshot and dropped when the operation ends, so a
//! measurement can never outlive the geometry it was taken from: there is no
//! stale residual to carry, because there is no residual carried. A rigid
//! placement changes neither `d_meas` (distances are invariant) nor the answer,
//! and an edit simply means the next operation measures the edited geometry.
//! Persisting a *committed* band on the records — a different question, with
//! its own grow-only commit rule and a codec obligation — stays future work.

use crate::topology::{BrepSolid, EdgeRecord, FaceRecord};
use crate::tolerance::PCURVE_ACCEPTANCE_REL;
use crate::{
    measure_edge_against_pcurve_image, solid_scale, KernelTolerances, NurbsCurve, NurbsSurface,
    Vec3,
};
use rustc_hash::FxHashMap as HashMap;

/// Fraction of an edge's own length that bounds its measured band.
///
/// The band is spent deciding whether two points ON the edge are the same
/// point, so it must stay far below the edge's extent or a genuine short
/// feature could be collapsed. A tenth leaves an order of magnitude between
/// "the vendor's representations disagree here" and "this edge is one point".
pub const EDGE_CAP_FRACTION: f64 = 0.10;

/// Fraction of the SHORTEST incident edge that bounds a vertex's band.
///
/// A quarter is what makes endpoint welding structurally unable to collapse an
/// edge: both endpoint radii of an edge sum to at most half its length, so the
/// two ends can never reach each other.
pub const VERTEX_CAP_FRACTION: f64 = 0.25;

/// Samples used for an edge's polyline length. The length only feeds a cap, so
/// a coarse chord underestimates it and errs toward a NARROWER band.
const LENGTH_SAMPLES: usize = 8;

/// Lazily-measured, capped per-entity tolerances for one solid.
///
/// Construct once per operation on the operand you are about to consume, ask
/// for the entities you actually touch, and drop it with the operation. Each
/// entity is measured at most once.
///
/// See the module documentation for the invariants; in particular a caller
/// takes `max(its own global band, this)`, never this alone.
pub struct EntityTolerances<'s> {
    solid: &'s BrepSolid,
    /// The policy floor `x` every band starts from.
    floor: f64,
    /// `PCURVE_ACCEPTANCE_REL * solid_scale` — the size-relative half of the
    /// edge cap, computed once.
    size_cap: f64,
    /// edge id -> the coedges that reference it, as (surface, pcurve, forward).
    coedges: Option<HashMap<u64, Vec<CoedgeUse<'s>>>>,
    /// edge id -> the record, for endpoint and length queries.
    edge_index: Option<HashMap<u64, &'s EdgeRecord>>,
    edges: HashMap<u64, f64>,
    vertices: HashMap<u64, f64>,
}

/// One coedge's contribution to an edge's measurement.
struct CoedgeUse<'s> {
    surface: &'s NurbsSurface,
    pcurve: &'s NurbsCurve,
    forward: bool,
}

impl<'s> EntityTolerances<'s> {
    /// A view over `solid` with `policy`'s spatial tolerance as the floor.
    pub fn for_solid(solid: &'s BrepSolid, policy: &KernelTolerances) -> Self {
        Self::with_floor(solid, policy.spatial())
    }

    /// A view whose floor is given directly — for the several callers that
    /// carry a bare `tolerance: f64` rather than a whole policy.
    pub fn with_floor(solid: &'s BrepSolid, floor: f64) -> Self {
        let floor = if floor.is_finite() && floor > 0.0 {
            floor
        } else {
            0.0
        };
        Self {
            solid,
            floor,
            size_cap: PCURVE_ACCEPTANCE_REL * solid_scale(solid),
            coedges: None,
            edge_index: None,
            edges: HashMap::default(),
            vertices: HashMap::default(),
        }
    }

    /// The identity band for edge `id`: `min(cap_E, max(floor, d_meas(E)))`.
    ///
    /// Answers the floor for an unknown edge, an edge with no coedge, or a
    /// measurement that could not be taken — I2's fail-safe direction. Never
    /// returns a non-finite or negative value.
    pub fn edge(&mut self, id: u64) -> f64 {
        if let Some(&memo) = self.edges.get(&id) {
            return memo;
        }
        let band = self.measure_edge(id);
        self.edges.insert(id, band);
        band
    }

    /// The identity band for vertex `id`: `min(cap_V, max(floor, g_meas(V)))`,
    /// where `g_meas` is the largest gap between the vertex's point and the
    /// endpoints of the edges that claim it.
    pub fn vertex(&mut self, id: u64) -> f64 {
        if let Some(&memo) = self.vertices.get(&id) {
            return memo;
        }
        let band = self.measure_vertex(id);
        self.vertices.insert(id, band);
        band
    }

    /// The largest edge band over `ids` — the band a decision that involves
    /// several edges at once (a junction sitting where two boundary edges meet)
    /// must respect, since it can be no more certain than its worst input.
    pub fn worst_edge(&mut self, ids: impl IntoIterator<Item = u64>) -> f64 {
        let mut worst = self.floor;
        for id in ids {
            worst = worst.max(self.edge(id));
        }
        worst
    }

    /// The policy floor this view was built with — what every band degrades to.
    pub fn floor(&self) -> f64 {
        self.floor
    }

    /// The raw `d_meas(E)` for edge `id`: the largest 3D disagreement between
    /// the edge's curve and its pcurve images, over every coedge that
    /// references it — UNCLAMPED, neither floored nor capped, so a defective
    /// entity (measured above `cap_E`) reads as what it is rather than as the
    /// cap. `None` for an unknown or degenerate edge, one no coedge
    /// references, or one whose every measurement failed.
    ///
    /// For REPORTING (the import's stated-precision consistency check, a
    /// census). It is never a band: the band is [`Self::edge`], and nothing a
    /// caller reads here may widen anything (I1/I2 in the module doc).
    pub fn measured_edge_deviation(&mut self, id: u64) -> Option<f64> {
        self.raw_edge(id)
    }

    /// The raw `g_meas(V)` for vertex `id`: the largest gap between the
    /// vertex's point and the ends of the edge curves that claim it —
    /// unclamped, as [`Self::measured_edge_deviation`] is for edges. `None`
    /// for an unknown vertex or one no non-degenerate edge ends at.
    ///
    /// On an imported solid this is the gap AFTER endpoint healing
    /// (`heal_imported_edge_endpoints`), not the vendor's raw miss.
    pub fn measured_vertex_gap(&mut self, id: u64) -> Option<f64> {
        self.raw_vertex(id).map(|(gap, _)| gap)
    }

    fn measure_edge(&mut self, id: u64) -> f64 {
        self.ensure_index();
        let Some(edge) = self.edge_index.as_ref().and_then(|index| index.get(&id)) else {
            return self.floor;
        };
        let edge = *edge;
        if edge.degenerate {
            return self.floor;
        }
        let cap = self.edge_cap(edge);
        // A measurement that failed says nothing, so it must not widen
        // anything: the floor stands (I2).
        let worst = self.raw_edge(id).unwrap_or(0.0);
        clamp_band(self.floor, worst, cap)
    }

    /// The unclamped edge measurement behind [`Self::measure_edge`]; see
    /// [`Self::measured_edge_deviation`] for the contract.
    fn raw_edge(&mut self, id: u64) -> Option<f64> {
        self.ensure_index();
        let edge = *self.edge_index.as_ref()?.get(&id)?;
        if edge.degenerate {
            return None;
        }
        // The band handed to the sampler DRIVES ITS REFINEMENT (it subdivides
        // while the deviation is non-linear relative to this number), it is not
        // a verdict the measurement is judged against. So it must be the TIGHT
        // policy floor: passing the cap here would stop subdivision at depth 0
        // on a smooth curve and alias a real 4e-4 residual down to nothing —
        // which is the same aliasing `offset/measure.rs` documents, arriving
        // through the band instead of through the sample grid.
        let probe_band = self.floor;
        let uses = self.coedges.as_ref()?.get(&id)?;
        let mut worst: Option<f64> = None;
        for use_record in uses {
            match measure_edge_against_pcurve_image(
                use_record.surface,
                use_record.pcurve,
                edge,
                use_record.forward,
                probe_band,
            ) {
                Err(_) => continue,
                Ok(measured) => {
                    let deviation = measured.deviation();
                    if deviation.is_finite() {
                        worst = Some(worst.map_or(deviation, |w| w.max(deviation)));
                    }
                }
            }
        }
        worst
    }

    /// O(edges) per query and deliberately index-free: the vertex band needs
    /// only curve endpoints, not the coedge index the edge band builds, and a
    /// caller that asks for one vertex should not pay for the other structure.
    fn measure_vertex(&mut self, id: u64) -> f64 {
        match self.raw_vertex(id) {
            None => self.floor,
            Some((gap, shortest)) => clamp_band(self.floor, gap, VERTEX_CAP_FRACTION * shortest),
        }
    }

    /// `(g_meas(V), shortest incident edge length)`, unclamped; `None` when
    /// the vertex is unknown or no non-degenerate edge ends at it.
    fn raw_vertex(&mut self, id: u64) -> Option<(f64, f64)> {
        let point = self
            .solid
            .vertices
            .iter()
            .find(|vertex| vertex.id == id)
            .map(|vertex| vertex.point)?;
        let mut gap = 0.0f64;
        let mut shortest = f64::INFINITY;
        for edge in &self.solid.edges {
            if edge.degenerate {
                continue;
            }
            for (vertex_id, parameter) in [
                (edge.start_vertex_id, edge.t0),
                (edge.end_vertex_id, edge.t1),
            ] {
                if vertex_id != id {
                    continue;
                }
                shortest = shortest.min(edge_length(edge));
                if let Ok(end) = edge.curve.evaluate(parameter) {
                    let distance = point.sub(end).length();
                    if distance.is_finite() {
                        gap = gap.max(distance);
                    }
                }
            }
        }
        if !shortest.is_finite() {
            return None;
        }
        Some((gap, shortest))
    }

    /// `cap_E = min(EDGE_CAP_FRACTION * len(E), PCURVE_ACCEPTANCE_REL * D)`.
    fn edge_cap(&self, edge: &EdgeRecord) -> f64 {
        (EDGE_CAP_FRACTION * edge_length(edge)).min(self.size_cap)
    }

    fn ensure_index(&mut self) {
        if self.coedges.is_some() {
            return;
        }
        let mut coedges: HashMap<u64, Vec<CoedgeUse<'s>>> = HashMap::default();
        for face in self.solid.shells.iter().flat_map(|shell| &shell.faces) {
            let face: &'s FaceRecord = face;
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    coedges
                        .entry(coedge.edge_id)
                        .or_default()
                        .push(CoedgeUse {
                            surface: &face.surface,
                            pcurve: &coedge.pcurve,
                            forward: coedge.forward,
                        });
                }
            }
        }
        self.coedges = Some(coedges);
        self.edge_index = Some(
            self.solid
                .edges
                .iter()
                .map(|edge| (edge.id, edge))
                .collect(),
        );
    }
}

/// `min(cap, max(floor, measured))`, guarded so no non-finite input escapes and
/// so the result is never below the floor: the fail-safe direction is NARROW.
fn clamp_band(floor: f64, measured: f64, cap: f64) -> f64 {
    let measured = if measured.is_finite() && measured > 0.0 {
        measured
    } else {
        0.0
    };
    let cap = if cap.is_finite() && cap > 0.0 {
        cap
    } else {
        floor
    };
    floor.max(measured.min(cap.max(floor)))
}

/// Chord length of `LENGTH_SAMPLES` uniform samples across the edge's live
/// span. Underestimates a curved edge, which narrows its cap — the safe way to
/// be wrong.
fn edge_length(edge: &EdgeRecord) -> f64 {
    let mut total = 0.0f64;
    let mut previous: Option<Vec3> = None;
    for step in 0..=LENGTH_SAMPLES {
        let fraction = step as f64 / LENGTH_SAMPLES as f64;
        let Ok(point) = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction) else {
            return 0.0;
        };
        if let Some(last) = previous {
            total += point.sub(last).length();
        }
        previous = Some(point);
    }
    if total.is_finite() {
        total
    } else {
        0.0
    }
}

