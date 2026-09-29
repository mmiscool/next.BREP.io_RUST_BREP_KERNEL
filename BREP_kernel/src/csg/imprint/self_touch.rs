//! SELF-TOUCH SPLIT — a face whose loops touch at an edge INTERIOR.
//!
//! A face's outer loop and one of its holes (or one loop and itself) can
//! touch at a point that is not a vertex of either edge: the 2026-09-07
//! report's r = 6 through-hole is tangent to both r = 4 fillet setbacks on
//! the top face, so the hole circle kisses the setback edges at (16, 10) and
//! (10, 16) with no vertex there.  Such a face is a pinched region; the
//! fragment stage's 2D arrangement resolves the pinch by carving the touched-
//! off corner as its own region — which only assembles when the touch point is
//! a vertex of BOTH touching edges, so every run through a chain is a
//! complete run and every fragment references shared sub-edges.  Without the
//! vertex the closed hole chain is walked in two partial runs and
//! `build_loop` refuses ("chain N fragmented into an incomplete run").  The
//! centre-plane document of the same day only executed because the imprint
//! happened to split the hole edge at those very points.
//!
//! Mint that vertex here, on the operand: split BOTH touching edges at their
//! closest approach when it lies within the contact band and interior to
//! each edge.  `apply_edge_splits` claims one vertex for the two coincident
//! split points and rewrites every coedge on the edge, so the hole's cylinder
//! wall — the face on the other side of the hole edge — carries the same
//! sub-edges by construction.  A face whose loops do not come within the band
//! of each other is untouched.

use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// Samples of an edge (interval count) for the coarse closest-approach scan.
const SCAN_SAMPLES: usize = 16;
/// Golden-section iterations refining the closest approach inside the
/// bracket around the best scan sample (0.618^48 ≈ 1e-10 of the bracket).
const REFINE_ITERATIONS: usize = 48;
/// Only a pair whose best scan sample comes within this fraction of the
/// pair's combined length is refined. A curve departs from its scan samples
/// by at most ~2% of its length between them (a full circle's chord sag at
/// 16 samples is 1.9% of the radius, 0.3% of the length), so a touch can
/// never hide behind the gate; a pair that is merely near in the bounding
/// boxes (most non-adjacent edge pairs of a curved face) costs the scan alone.
const REFINE_GATE: f64 = 0.05;

/// Contact band of the self-touch test: the imprint's contact band
/// (`tolerance × 100`) floored by the weld radius and by `2e-6 × scale`, which
/// stands in for the fragment arrangement's own touch radius (`1e-6 × the
/// face's uv-domain diagonal`) on the kernel's length-parameterised carriers,
/// where a face's domain diagonal stays within about √2 of the operand's
/// extent. Two loops the arrangement will fuse must be fused here as well, or
/// the arrangement still walks the partial runs.
pub(super) fn self_touch_band(tolerance: f64, scale: f64) -> f64 {
    (tolerance * 100.0).max(WELD_FLOOR).max(2e-6 * scale)
}

struct EdgeScan<'a> {
    edge: &'a EdgeRecord,
    curve: NurbsCurve,
    minimum: Vec3,
    maximum: Vec3,
}

fn edge_scan<'a>(edge: &'a EdgeRecord, curve: NurbsCurve) -> EdgeScan<'a> {
    let mut minimum = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut maximum = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for control in &curve.control_points {
        if control.w.abs() <= 1e-300 {
            continue;
        }
        let point = Vec3::new(control.x / control.w, control.y / control.w, control.z / control.w);
        minimum = Vec3::new(minimum.x.min(point.x), minimum.y.min(point.y), minimum.z.min(point.z));
        maximum = Vec3::new(maximum.x.max(point.x), maximum.y.max(point.y), maximum.z.max(point.z));
    }
    EdgeScan {
        edge,
        curve,
        minimum,
        maximum,
    }
}

fn boxes_overlap(a: &EdgeScan<'_>, b: &EdgeScan<'_>, pad: f64) -> bool {
    a.minimum.x <= b.maximum.x + pad
        && b.minimum.x <= a.maximum.x + pad
        && a.minimum.y <= b.maximum.y + pad
        && b.minimum.y <= a.maximum.y + pad
        && a.minimum.z <= b.maximum.z + pad
        && b.minimum.z <= a.maximum.z + pad
}

fn shares_a_vertex(a: &EdgeRecord, b: &EdgeRecord) -> bool {
    a.start_vertex_id == b.start_vertex_id
        || a.start_vertex_id == b.end_vertex_id
        || a.end_vertex_id == b.start_vertex_id
        || a.end_vertex_id == b.end_vertex_id
}

/// Closest approach of `probe` to `target`: `(t on probe, u on target,
/// distance)`.  A coarse uniform scan of `probe` projected onto `target`,
/// then a golden-section refinement of the distance over the bracket around
/// the best sample.  Exact for the tangential kiss this exists for (the
/// distance is smooth and unimodal inside one scan bracket).
fn closest_approach(
    probe: &NurbsCurve,
    target: &NurbsCurve,
    refine_gate: f64,
) -> Result<(f64, f64, f64), KernelRefusal> {
    let [t0, t1] = probe.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let distance_at = |t: f64| -> Result<(f64, f64), KernelRefusal> {
        let point = probe.evaluate(t).or_refuse(KernelStage::Intersect, "evaluate")?;
        let projection =
            project_point_to_curve(target, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
        Ok((projection.u, projection.distance))
    };
    let mut best_index = 0usize;
    let mut best: Option<(f64, f64, f64)> = None;
    for index in 0..=SCAN_SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / SCAN_SAMPLES as f64;
        let (u, distance) = distance_at(t)?;
        if best.is_none_or(|(_, _, current)| distance < current) {
            best = Some((t, u, distance));
            best_index = index;
        }
    }
    let Some(mut best) = best else {
        return Err(KernelRefusal::internal(KernelStage::Intersect, "imprint.self_touch", "closest approach of an empty scan"));
    };
    if best.2 > refine_gate {
        return Ok(best);
    }
    let step = (t1 - t0) / SCAN_SAMPLES as f64;
    let mut low = (t0 + step * best_index.saturating_sub(1) as f64).max(t0);
    let mut high = (t0 + step * (best_index + 1) as f64).min(t1);
    // Golden-section search on a unimodal bracket: keep the two interior
    // probes, drop the outer quarter on the worse side.
    let ratio = (5f64.sqrt() - 1.0) / 2.0;
    let mut inner_low = high - ratio * (high - low);
    let mut inner_high = low + ratio * (high - low);
    let mut at_low = distance_at(inner_low)?;
    let mut at_high = distance_at(inner_high)?;
    for _ in 0..REFINE_ITERATIONS {
        if at_low.1 < at_high.1 {
            high = inner_high;
            inner_high = inner_low;
            at_high = at_low;
            inner_low = high - ratio * (high - low);
            at_low = distance_at(inner_low)?;
        } else {
            low = inner_low;
            inner_low = inner_high;
            at_low = at_high;
            inner_high = low + ratio * (high - low);
            at_high = distance_at(inner_high)?;
        }
        if high - low <= 1e-14 * (t1 - t0).abs().max(1.0) {
            break;
        }
    }
    for (t, (u, distance)) in [(inner_low, at_low), (inner_high, at_high)] {
        if distance < best.2 {
            best = (t, u, distance);
        }
    }
    Ok(best)
}

/// Every self-touch of ONE face's loops, as the `(edge, parameter)` splits
/// that mint each touch as a vertex on BOTH touching edges.  Shared by the
/// imprint builder's per-operand pass and the solid-level entry point below
/// so the two lanes can never scan on different terms.
fn face_self_touches<'a>(
    operand: u8,
    face_id: u64,
    edges: &[&'a EdgeRecord],
    band: f64,
    subcurve: &dyn Fn(&EdgeRecord) -> Result<NurbsCurve, KernelRefusal>,
) -> Result<Vec<(&'a EdgeRecord, f64)>, KernelRefusal> {
    if edges.len() < 2 {
        return Ok(Vec::new());
    }
    let debug = std::env::var("BREP_DEBUG_SELF_TOUCH").is_ok();
    let scans = edges
        .iter()
        .filter(|edge| !edge.degenerate)
        .map(|edge| Ok(edge_scan(edge, subcurve(edge)?)))
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let lengths = scans
        .iter()
        .map(|scan| curve_length_rough(&scan.curve))
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let mut splits = Vec::new();
    for (index, first) in scans.iter().enumerate() {
        for (other, second) in scans.iter().enumerate().skip(index + 1) {
            if shares_a_vertex(first.edge, second.edge) || !boxes_overlap(first, second, band) {
                continue;
            }
            let refine_gate = (REFINE_GATE * (lengths[index] + lengths[other])).max(band);
            let (t, u, distance) = closest_approach(&first.curve, &second.curve, refine_gate)?;
            if distance > band {
                continue;
            }
            if debug {
                eprintln!(
                    "self-touch: operand {} face {} edges {} and {} touch at t={:.9} u={:.9} (gap {:.3e}, band {:.3e})",
                    operand, face_id, first.edge.id, second.edge.id, t, u, distance, band
                );
            }
            splits.push((first.edge, t));
            splits.push((second.edge, u));
        }
    }
    Ok(splits)
}

/// The self-touch splits of a WHOLE SOLID, ready for `apply_edge_splits`.
///
/// `build_imprints` runs the same scan on the two operands it is already
/// pairing, which covers every boolean.  The offset shell fragments solids
/// that no imprint ever describes: its SOURCE goes through `fragment_solid`
/// with an EMPTY imprint, and its offset carriers — whose trims are copied
/// straight from the source faces — deliberately keep their boundaries
/// un-split.  A pinched face therefore reached the arrangement exactly as
/// modelled and `build_loop` refused ("chain N fragmented into an incomplete
/// run"); this is how that lane mints the same vertex.
///
/// Returns one record per split edge, parameters ascending.  An empty result
/// means no face self-touches, and the caller should leave the solid alone.
pub(crate) fn self_touch_edge_splits(
    solid: &BrepSolid,
    operand: u8,
    tolerance: f64,
    scale: f64,
) -> Result<Vec<EdgeSplitRecord>, KernelRefusal> {
    let band = self_touch_band(tolerance, scale);
    let edges_by_id = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let mut splits: HashMap<u64, Vec<f64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let mut seen = HashSet::default();
        let mut edges = Vec::new();
        for coedge in face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            if seen.insert(coedge.edge_id) {
                if let Some(edge) = edges_by_id.get(&coedge.edge_id) {
                    edges.push(*edge);
                }
            }
        }
        for (edge, parameter) in
            face_self_touches(operand, face.id, &edges, band, &edge_subcurve)?
        {
            push_edge_split(
                splits.entry(edge.id).or_default(),
                edge,
                parameter,
                tolerance,
            )?;
        }
    }
    let mut records = splits
        .into_iter()
        .map(|(edge_id, mut parameters)| {
            parameters.sort_by(f64::total_cmp);
            EdgeSplitRecord {
                operand,
                edge_id,
                parameters,
            }
        })
        .collect::<Vec<_>>();
    records.sort_by_key(|record| record.edge_id);
    Ok(records)
}

impl ImprintBuilder<'_> {
    /// Split every pair of edges of one face that touch at an interior point
    /// of either (see the module doc).  Both edges receive the split, at the
    /// parameter of their closest approach; `add_edge_split`'s near-endpoint
    /// guard drops the half that already ends at a vertex there.
    pub(super) fn split_self_touching_loops(
        &mut self,
        faces: &[TaggedFace<'_>],
        face_edge_lists: &HashMap<FaceKey, Vec<&EdgeRecord>>,
        subcurve: &dyn Fn(u8, &EdgeRecord) -> Result<NurbsCurve, KernelRefusal>,
    ) -> Result<(), KernelRefusal> {
        let band = self_touch_band(self.tolerance, self.scale);
        for face in faces {
            let operand = face.operand;
            let touches = face_self_touches(
                operand,
                face.face.id,
                &face_edge_lists[&face.key()],
                band,
                &|edge| subcurve(operand, edge),
            )?;
            for (edge, parameter) in touches {
                self.add_edge_split(operand, edge, parameter)?;
            }
        }
        Ok(())
    }
}
