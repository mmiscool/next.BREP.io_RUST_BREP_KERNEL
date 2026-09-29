use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// Record one split parameter on `edge`, honouring the near-endpoint guard
/// and the dedupe band.  Two split points closer than the assembler's weld
/// tolerance can never survive as distinct vertices — they weld at assembly
/// and leave zero-length edges behind — so they are folded here instead.
///
/// Free-standing because the SOLID-level self-touch scan
/// ([`super::self_touch::self_touch_edge_splits`]) has no builder to record
/// into and must not diverge from the guard the builder applies.
pub(super) fn push_edge_split(
    parameters: &mut Vec<f64>,
    edge: &EdgeRecord,
    parameter: f64,
    model_tolerance: f64,
) -> Result<(), KernelRefusal> {
    let span = edge.t1 - edge.t0;
    let weld_distance = assembler_weld(model_tolerance);
    let tolerance = (KNOT_IDENTITY_TOL * span).max(parameter_tolerance(
        &edge.curve,
        parameter,
        weld_distance,
    )?);
    if parameter <= edge.t0 + tolerance || parameter >= edge.t1 - tolerance {
        return Ok(());
    }
    if !parameters
        .iter()
        .any(|existing| (*existing - parameter).abs() <= tolerance)
    {
        parameters.push(parameter);
    }
    Ok(())
}

pub(super) struct ImprintBuilder<'a> {
    pub(super) open_carriers: bool,
    pub(super) edges: HashMap<(u8, u64), &'a EdgeRecord>,
    pub(super) tolerance: f64,
    /// Combined bbox extent of the two operands ([`solid_scale`]); the merge
    /// bands scale with part SIZE via [`merge_scale`], not distance from origin.
    pub(super) scale: f64,
    pub(super) vertices: Vec<ImprintVertex>,
    pub(super) vertex_radii: HashMap<u64, f64>,
    pub(super) pieces: Vec<ImprintPieceRecord>,
    pub(super) by_face: HashMap<FaceKey, Vec<u64>>,
    pub(super) edge_splits: HashMap<(u8, u64), Vec<f64>>,
    pub(super) barrier_edges: HashSet<(u8, u64)>,
    /// Operand edges some section curve geometrically RODE (the overlap
    /// branch of `process_curve`): candidates for the PIECE-ENDPOINT
    /// post-pass (`exchange_piece_endpoint_junctions`). Deliberately a
    /// separate set from `barrier_edges` (same writer today) so the two
    /// mechanisms cannot silently couple if either gains another writer.
    pub(super) overlap_ridden_edges: HashSet<(u8, u64)>,
    /// Ids of the pieces minted from a MARCHED run (fitted from the marcher's
    /// polyline), as opposed to an analytic, planar-iso or boundary-copy
    /// curve. Only these are candidates for `dissolve_section_origin_vertices`:
    /// an exact analytic ring's origin split is the coalescer's exact-band
    /// closure, and dissolving it moved an offset shell's corners (see there).
    pub(super) marched_pieces: HashSet<u64>,
    pub(super) next_id: u64,
}

impl<'a> ImprintBuilder<'a> {
    fn vertex(&mut self, point: Vec3, merge_radius: f64) -> (u64, Vec3) {
        let base = self.tolerance * 10.0 * merge_scale(self.scale);
        for vertex in &self.vertices {
            let existing_radius = self.vertex_radii.get(&vertex.id).copied().unwrap_or(0.0);
            let distance = vertex.point.sub(point).length();
            if distance <= base.max(existing_radius).max(merge_radius) {
                self.vertex_radii
                    .insert(vertex.id, existing_radius.max(merge_radius));
                // The registry point is canonical.  Returning the caller's
                // near-duplicate here would give one topological vertex ID
                // several geometric positions on its incident curves.
                return (
                    vertex.id,
                    if distance <= base {
                        vertex.point
                    } else {
                        point
                    },
                );
            }
            // A broad fuzzy radius nominates a possible joint, but must not
            // collapse visibly distinct curve endpoints into one vertex.  The
            // caller's fuzzy-junction pass writes the same refined point into
            // both candidates when the joint is genuine.
        }
        let id = self.next_id;
        self.next_id += 1;
        self.vertices.push(ImprintVertex { id, point });
        self.vertex_radii.insert(id, merge_radius);
        (id, point)
    }

    pub(super) fn add_edge_split(
        &mut self,
        operand: u8,
        edge: &EdgeRecord,
        parameter: f64,
    ) -> Result<(), KernelRefusal> {
        let tolerance = self.tolerance;
        push_edge_split(
            self.edge_splits.entry((operand, edge.id)).or_default(),
            edge,
            parameter,
            tolerance,
        )
    }

    /// OVERLAP-JUNCTION EXCHANGE, PIECE-ENDPOINT POST-PASS (rotated
    /// equator-tangency, cardinal-sphere sub-configuration — problemInbox
    /// rot2 2026-08-14): the per-call exchange in `process_curve` can only
    /// imprint junctions IT sees onto edges IT sees — but the seam-crossing
    /// junction of a section riding an operand ring can be minted by a
    /// boundary-copy call whose `split_faces` exclude the ring, while the
    /// analytic call that DOES ride the ring carries that same junction at
    /// its closed curve's parameterization origin (param 0/1), where the
    /// endpoint guards suppress it (`perpendicular(cap_normal)` derives the
    /// analytic circle's origin, which lands exactly on the sphere seam for
    /// cardinal sphere axes). No single call ever holds both halves, so the
    /// ring stays one CLOSED record against two OPEN arc pieces — the
    /// one-use S=2 genus=1 signature. This post-pass runs once ALL calls
    /// have minted their pieces: every endpoint of an OPEN piece that RIDES
    /// an overlap-ridden edge, and that is a real junction (referenced by
    /// ≥ 2 piece-endpoint slots — a marched curve's deliberate slack end is
    /// degree 1 and must NOT imprint; see the `!refine` guard in the
    /// overlap branch), is imprinted onto that edge via `add_edge_split`
    /// (which keeps the near-endpoint and dedup guards — a junction already
    /// at the ring's own vertex is correctly rejected there).
    /// Escape hatches: `BREP_OVERLAP_PIECE_ENDPOINT_SPLIT=0` (this pass) or
    /// the family hatch `BREP_OVERLAP_JUNCTION_SPLIT=0` (whole exchange).
    pub(super) fn exchange_piece_endpoint_junctions(&mut self) -> Result<(), KernelRefusal> {
        if std::env::var("BREP_OVERLAP_JUNCTION_SPLIT").as_deref() == Ok("0")
            || std::env::var("BREP_OVERLAP_PIECE_ENDPOINT_SPLIT").as_deref() == Ok("0")
        {
            return Ok(());
        }
        if self.overlap_ridden_edges.is_empty() || self.pieces.is_empty() {
            return Ok(());
        }
        let overlap_limit = self.tolerance.max(1e-5);
        let mut endpoint_degree: HashMap<u64, usize> = HashMap::default();
        for piece in &self.pieces {
            *endpoint_degree.entry(piece.start_vertex_id).or_insert(0) += 1;
            *endpoint_degree.entry(piece.end_vertex_id).or_insert(0) += 1;
        }
        let mut ridden: Vec<(u8, u64)> = self.overlap_ridden_edges.iter().copied().collect();
        ridden.sort_unstable();
        let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();
        let stats = std::env::var("BREP_ROT2_STATS").is_ok();
        // web_time::Instant (imported above), NOT std::time::Instant — std's clock
        // is unimplemented on wasm32-unknown-unknown and Instant::now() ABORTS there
        // ("RuntimeError: unreachable"). This pass runs on knife-edge tangency booleans
        // (overlap_ridden_edges non-empty), so a raw std::time call here traps the whole
        // cyl×sphere union in the browser while passing natively. See lib.rs boolean path.
        let t_start = Instant::now();
        let mut ride_tests = 0usize;
        let mut planned: Vec<(u8, &EdgeRecord, f64)> = Vec::new();
        for &(operand, edge_id) in &ridden {
            let Some(&edge) = self.edges.get(&(operand, edge_id)) else {
                continue;
            };
            if edge.degenerate {
                continue;
            }
            let low = edge.t0.min(edge.t1);
            let high = edge.t0.max(edge.t1);
            for piece in &self.pieces {
                // Open pieces only: a CLOSED piece riding a closed ring is
                // the shared-section RING reuse's territory; splitting the
                // ring at its origin would recreate the closed-vs-open
                // mismatch this pass exists to remove.
                if piece.start_vertex_id == piece.end_vertex_id || !(piece.t1 > piece.t0) {
                    continue;
                }
                // Ride test over the piece's LIVE window (records may carry
                // a window narrower than the curve's domain).
                ride_tests += 1;
                let mut on = 0usize;
                for fraction in [0.1, 0.3, 0.5, 0.7, 0.9] {
                    let t = piece.t0 + (piece.t1 - piece.t0) * fraction;
                    let point = piece.curve.evaluate(t).or_refuse(KernelStage::Intersect, "evaluate")?;
                    let projection = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                    let nearest = edge.curve.evaluate(projection.u.clamp(low, high)).or_refuse(KernelStage::Intersect, "evaluate")?;
                    if nearest.sub(point).length() <= overlap_limit {
                        on += 1;
                    }
                }
                if on < 3 {
                    continue;
                }
                for (t, vertex_id) in [
                    (piece.t0, piece.start_vertex_id),
                    (piece.t1, piece.end_vertex_id),
                ] {
                    if endpoint_degree.get(&vertex_id).copied().unwrap_or(0) < 2 {
                        continue;
                    }
                    let point = piece.curve.evaluate(t).or_refuse(KernelStage::Intersect, "evaluate")?;
                    let projection = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                    let nearest = edge.curve.evaluate(projection.u.clamp(low, high)).or_refuse(KernelStage::Intersect, "evaluate")?;
                    if nearest.sub(point).length() <= overlap_limit {
                        if debug {
                            eprintln!(
                                "piece-endpoint exchange: piece {} vertex {} \
                                 -> op={operand} edge={edge_id} u={:.6}",
                                piece.id, vertex_id, projection.u
                            );
                        }
                        planned.push((operand, edge, projection.u));
                    }
                }
            }
        }
        if stats {
            eprintln!(
                "ROT2_STATS: ridden_edges={} pieces={} ride_tests={} planned={} post_pass_ms={:.1}",
                ridden.len(),
                self.pieces.len(),
                ride_tests,
                planned.len(),
                t_start.elapsed().as_secs_f64() * 1000.0
            );
        }
        for (operand, edge, parameter) in planned {
            self.add_edge_split(operand, edge, parameter)?;
        }
        Ok(())
    }

    fn face_has_seam_at(&self, face: TaggedFace<'_>, point: Vec3) -> Result<bool, KernelRefusal> {
        let mut counts = HashMap::default();
        for coedge in face
            .face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            *counts.entry(coedge.edge_id).or_insert(0usize) += 1;
        }
        let tolerance = WELD_FLOOR * merge_scale(self.scale);
        for (edge_id, count) in counts {
            let edge = self.edges[&(face.operand, edge_id)];
            if count >= 2
                && !edge.degenerate
                && project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?.distance <= tolerance
            {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn polish_endpoint(
        &self,
        point: Vec3,
        first: &NurbsSurface,
        second: &NurbsSurface,
    ) -> Result<Vec3, KernelRefusal> {
        let threshold = (self.tolerance * 100.0).max(1e-6);
        let original = project_point_to_surface(first, point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?
            .distance
            .max(project_point_to_surface(second, point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.distance);
        if original <= threshold {
            return Ok(point);
        }
        let mut current = point;
        let mut best = point;
        let mut best_residual = original;
        for _ in 0..30 {
            let on_first = project_point_to_surface(first, current).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.point;
            let on_second = project_point_to_surface(second, on_first).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.point;
            let back = project_point_to_surface(first, on_second).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.point;
            let candidate = back.add(on_second).scale(0.5);
            let residual = project_point_to_surface(first, candidate).or_refuse(KernelStage::Intersect, "project_point_to_surface")?
                .distance
                .max(project_point_to_surface(second, candidate).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.distance);
            if residual < best_residual {
                best = candidate;
                best_residual = residual;
            }
            current = candidate;
            if residual <= (self.tolerance * 10.0).max(1e-7) {
                break;
            }
        }
        Ok(if best_residual <= threshold {
            best
        } else {
            point
        })
    }

    pub(super) fn process_curve(
        &mut self,
        curve: NurbsCurve,
        first: TaggedFace<'_>,
        second: TaggedFace<'_>,
        test_faces: &[TaggedFace<'_>],
        target_faces: &[TaggedFace<'_>],
        refine: bool,
    ) -> Result<(), KernelRefusal> {
        let mut split_parameters = Vec::new();
        let mut fuzzy_junctions: Vec<(Vec3, f64)> = Vec::new();
        // Operand edges the section curve geometrically RIDES (the overlap
        // branch below): junctions minted on the section must be exchanged
        // onto them once the full junction set is known — see the
        // OVERLAP-JUNCTION EXCHANGE after the split_parameters dedup.
        let mut overlapped_edges: Vec<(u8, &EdgeRecord)> = Vec::new();
        let [whole_start, whole_end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
        // Anything finer than the assembler's weld tolerance cannot survive
        // as distinct topology, so that is the meaningful noise floor for
        // overlap detection (marched curves wiggle a few 1e-6 around the
        // exact curve and every wiggle would otherwise count as a crossing).
        let overlap_limit = self.tolerance.max(1e-5);
        let distinct_pierces_on_section =
            std::env::var("BREP_PIERCE_DISTINCT_ON_SECTION").as_deref() != Ok("0");
        // Crossings this close to the curve's own endpoints are the
        // endpoint junction itself seen through the finders' spatial
        // tolerance: splitting there leaves a sliver piece whose surviving
        // half evaluates measurably off the true corner (§4.13 — the
        // parametric guard below is far tighter than the finders' 1e-5).
        let whole_start_point = curve.evaluate(whole_start).or_refuse(KernelStage::Intersect, "evaluate")?;
        let whole_end_point = curve.evaluate(whole_end).or_refuse(KernelStage::Intersect, "evaluate")?;
        let near_curve_end = |point: Vec3| {
            point.sub(whole_start_point).length() <= overlap_limit
                || point.sub(whole_end_point).length() <= overlap_limit
        };
        let split_faces = if refine {
            vec![first, second]
        } else {
            target_faces.to_vec()
        };
        for face in split_faces {
            let other = if face.key() == first.key() {
                second
            } else {
                first
            };
            for edge in face_edges(face, &self.edges)? {
                if edge.degenerate {
                    continue;
                }
                // Propagate failures: a swallowed error here silently drops
                // junction splits and corrupts topology far downstream.
                let pierces = if refine {
                    intersect_curve_surface(
                        &edge.curve,
                        &other.face.surface,
                        self.tolerance.max(1e-9),
                    )
                    .map_err(|error| {
                        format!("imprint pierce recovery on edge {}: {error}", edge.id)
                    }).or_refuse(KernelStage::Intersect, "csg.imprint.builder")?
                } else {
                    Vec::new()
                };
                // A (near-)coincident overlap must exchange ENDPOINTS only:
                // intersect_curves between overlapping curves reports
                // spurious subdivision hits along the whole shared span,
                // spraying micro edge-splits (glue unions copy each face's
                // edges onto the other and then hit-test them against their
                // own coincident twins).
                if curve_overlaps_edge(&curve, edge, overlap_limit)? {
                    self.barrier_edges.insert((face.operand, edge.id));
                    self.overlap_ridden_edges.insert((face.operand, edge.id));
                    overlapped_edges.push((face.operand, edge));
                    // Marched (refine) curves stop with deliberate slack past
                    // the trim boundary, so their endpoints are NOT junctions;
                    // exchanging them here plants off-by-a-band vertices on
                    // the overlapped edge. Exact-endpoint sources (cosurface
                    // boundary copies) still exchange.
                    if !refine {
                        for parameter in [whole_start, whole_end] {
                            let point = curve.evaluate(parameter).or_refuse(KernelStage::Intersect, "evaluate")?;
                            let projection = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                            let nearest = edge.curve.evaluate(
                                projection
                                    .u
                                    .clamp(edge.t0.min(edge.t1), edge.t0.max(edge.t1)),
                            ).or_refuse(KernelStage::Intersect, "csg.imprint.builder")?;
                            if nearest.sub(point).length() <= overlap_limit {
                                self.add_edge_split(face.operand, edge, projection.u)?;
                            }
                        }
                    }
                    for parameter in [edge.t0, edge.t1] {
                        let point = edge.curve.evaluate(parameter).or_refuse(KernelStage::Intersect, "evaluate")?;
                        let projection = project_point_to_curve(&curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                        if projection.distance <= overlap_limit
                            && projection.u > whole_start + 1e-7
                            && projection.u < whole_end - 1e-7
                            && !split_parameters
                                .iter()
                                .any(|value: &f64| (*value - projection.u).abs() <= 1e-6)
                        {
                            split_parameters.push(projection.u);
                        }
                    }
                    continue;
                }
                let curve_hits = intersect_curves(&curve, &edge.curve, 1e-5).map_err(|error| {
                    format!("imprint curve-edge crossing on edge {}: {error}", edge.id)
                }).or_refuse(KernelStage::Intersect, "csg.imprint.builder")?;
                for hit in curve_hits {
                    if hit.t < edge.t0 - 1e-9 || hit.t > edge.t1 + 1e-9 {
                        continue;
                    }
                    let mut curve_parameter = hit.s;
                    let mut edge_parameter = hit.t;
                    let mut fuzzy_junction = None;
                    if refine {
                        let hit_point = curve.evaluate(curve_parameter).or_refuse(KernelStage::Intersect, "evaluate")?;
                        let search_radius = 4e-3 * (1.0 + hit_point.length());
                        let mut candidates = pierces
                            .iter()
                            .filter(|pierce| {
                                pierce.t >= edge.t0 - 1e-9 && pierce.t <= edge.t1 + 1e-9
                            })
                            .map(|pierce| (pierce, pierce.point.sub(hit_point).length()))
                            .collect::<Vec<_>>();
                        candidates.sort_by(|a, b| a.1.total_cmp(&b.1));
                        if let Some((best, best_distance)) = candidates.first() {
                            let second_distance = candidates
                                .get(1)
                                .map(|candidate| candidate.1)
                                .unwrap_or(f64::INFINITY);
                            if *best_distance <= search_radius
                                && *best_distance < 0.5 * second_distance
                            {
                                let projection = project_point_to_curve(&curve, best.point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                                if projection.distance <= 2e-3 * (1.0 + best.point.length()) {
                                    curve_parameter = projection.u;
                                    edge_parameter = best.t;
                                    fuzzy_junction = Some((
                                        best.point,
                                        (2.0 * projection.distance).max(2e-5).min(1.5e-3),
                                    ));
                                }
                            }
                        }
                    }
                    let [start, end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
                    let interior = curve_parameter > start + 1e-7
                        && curve_parameter < end - 1e-7
                        && !near_curve_end(curve.evaluate(curve_parameter).or_refuse(KernelStage::Intersect, "evaluate")?);
                    // A marched section that GRAZES an edge where it passes
                    // through is not a junction: the tangential hit is skipped. A
                    // section that ENDS on an edge ends at a junction whatever
                    // the angle, so an end hit splits the edge. The hit is an end
                    // hit when its junction (the pierce it snapped to, or the hit
                    // itself) lies at the section's endpoint, not merely when its
                    // parameter clamps there. A faithful fit reproduces a real
                    // tangency at such an end: the self-crossing repair's section
                    // on `FilletFailureWith2RasiusValues` runs along the bore's
                    // mouth ring at both ends (the carriers read sin 5.7e-10 and
                    // 2.0e-11), and skipping those hits left the ring whole.
                    let junction_point = match fuzzy_junction {
                        Some((point, _)) => point,
                        None => curve.evaluate(curve_parameter).or_refuse(KernelStage::Intersect, "evaluate")?,
                    };
                    let ends_on = !interior && near_curve_end(junction_point);
                    if hit.tangential && refine && !ends_on {
                        continue;
                    }
                    if let Some(junction) = fuzzy_junction {
                        fuzzy_junctions.push(junction);
                    }
                    self.add_edge_split(face.operand, edge, edge_parameter)?;
                    if interior
                        && !split_parameters
                            .iter()
                            .any(|value: &f64| (*value - curve_parameter).abs() <= 1e-6)
                    {
                        split_parameters.push(curve_parameter);
                    }
                }

                // A fitted SSI curve can miss a boundary curve by chord sag.
                // Exact edge/surface pierces recover those topological cuts.
                if refine {
                    for pierce in &pierces {
                        if pierce.t < edge.t0 - 1e-9 || pierce.t > edge.t1 + 1e-9 {
                            continue;
                        }
                        let scale = 1.0 + pierce.point.length();
                        let mut projection = project_point_to_curve(&curve, pierce.point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                        if projection.distance > 2e-3 * scale {
                            // `project_point_to_curve`'s per-knot-span seed grid
                            // (4..deg+2 samples/span) + Newton can settle in the
                            // WRONG basin on a near-closed section curve, mapping
                            // an on-curve pierce onto the far endpoint: 00000170's
                            // box-plane × revolution-band corner grazes the band's
                            // v=1 rim in a shallow <0.2mm arc and re-enters at the
                            // exact edge×plane pierce P0 (on the section at
                            // t≈0.978), yet the projector returns t=0 (the section
                            // start, 0.86mm away) so this gate drops the split, the
                            // tiny on-band corner arc is never minted, the mating
                            // box face's loop cannot close and the band's own corner
                            // fragment strands the shared rim/seam edges one-use
                            // (pool t288). Locating the parameter of a point KNOWN
                            // to lie on the curve (both pierce finders put it there)
                            // is deterministic INVERSION, not a containment verdict,
                            // so a projector-free dense-sample-then-bracket rescue is
                            // sound here — and it runs ONLY on pierces the projector
                            // already rejected, re-verified against the SAME gate, so
                            // no currently-accepted split changes. Escape hatch
                            // BREP_PIERCE_PARAM_RESCUE=0.
                            if std::env::var("BREP_PIERCE_PARAM_RESCUE").as_deref() == Ok("0") {
                                continue;
                            }
                            let [c_start, c_end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
                            const RESCUE_STATIONS: usize = 512;
                            let mut best_u = c_start;
                            let mut best_d2 = f64::INFINITY;
                            for index in 0..=RESCUE_STATIONS {
                                let u = c_start
                                    + (c_end - c_start) * (index as f64) / (RESCUE_STATIONS as f64);
                                let d2 =
                                    curve.evaluate(u).or_refuse(KernelStage::Intersect, "evaluate")?.sub(pierce.point).length_squared();
                                if d2 < best_d2 {
                                    best_d2 = d2;
                                    best_u = u;
                                }
                            }
                            let step = (c_end - c_start) / (RESCUE_STATIONS as f64);
                            let mut lo = (best_u - step).max(c_start);
                            let mut hi = (best_u + step).min(c_end);
                            for _ in 0..64 {
                                let m1 = lo + (hi - lo) / 3.0;
                                let m2 = hi - (hi - lo) / 3.0;
                                let d1 =
                                    curve.evaluate(m1).or_refuse(KernelStage::Intersect, "evaluate")?.sub(pierce.point).length_squared();
                                let d2 =
                                    curve.evaluate(m2).or_refuse(KernelStage::Intersect, "evaluate")?.sub(pierce.point).length_squared();
                                if d1 < d2 {
                                    hi = m2;
                                } else {
                                    lo = m1;
                                }
                            }
                            let rescued_u = 0.5 * (lo + hi);
                            let rescued_point = curve.evaluate(rescued_u).or_refuse(KernelStage::Intersect, "evaluate")?;
                            let rescued_distance = rescued_point.sub(pierce.point).length();
                            if rescued_distance > 2e-3 * scale {
                                continue;
                            }
                            projection = crate::CurveProjection {
                                u: rescued_u,
                                point: rescued_point,
                                distance: rescued_distance,
                            };
                        }
                        // Two pierces near each other are AMBIGUOUS when the
                        // section cannot tell them apart — one of them lies off
                        // it and projects to the other's place. Two pierces that
                        // BOTH lie on the section, at places along it as far
                        // apart as the pierces themselves, are two junctions: a
                        // graze crossing an edge twice 0.107 apart, each pierce
                        // on the section to 2e-9 (`22_sag_boundary_identity_t164`,
                        // 2026-09-26), lost both to this rule and left the
                        // sphere face bounded by a rim arc 4.6e-4 off it.
                        // Read coincidence at the assembler's identity floor,
                        // not the section fit's requested residual: a fitted
                        // section can miss a real crossing by a few microns
                        // while the crossings remain well-separated vertices.
                        // Escape hatch BREP_PIERCE_DISTINCT_ON_SECTION=0.
                        let on_section = assembler_weld(self.tolerance);
                        let mut ambiguous = false;
                        for other_pierce in &pierces {
                            let apart = other_pierce.point.sub(pierce.point).length();
                            if std::ptr::eq(pierce, other_pierce) || apart > 4e-3 * scale {
                                continue;
                            }
                            let other_projection =
                                project_point_to_curve(&curve, other_pierce.point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                            if distinct_pierces_on_section
                                && projection.distance <= on_section
                                && other_projection.distance <= on_section
                                && apart > overlap_limit
                                && projection.point.sub(other_projection.point).length() >= 0.5 * apart
                            {
                                continue;
                            }
                            if projection.distance >= 0.5 * other_projection.distance {
                                ambiguous = true;
                                break;
                            }
                        }
                        if ambiguous {
                            continue;
                        }
                        let curve_tangent = curve.derivatives(projection.u, 1).or_refuse(KernelStage::Intersect, "derivatives")?[1];
                        let edge_tangent = edge.curve.derivatives(pierce.t, 1).or_refuse(KernelStage::Intersect, "derivatives")?[1];
                        let parallel_scale = curve_tangent.length() * edge_tangent.length();
                        let [start, end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
                        let interior = projection.u > start + 1e-7
                            && projection.u < end - 1e-7
                            && !near_curve_end(curve.evaluate(projection.u).or_refuse(KernelStage::Intersect, "evaluate")?);
                        // The parallel skip is for a pierce the section passes
                        // THROUGH while running along the edge. A pierce the
                        // section ENDS on is a junction whatever the angle, the
                        // same rule as the tangential hit above: the edge is
                        // split and the piece boundary is a vertex. The pierce
                        // itself must lie at the endpoint: one past the end
                        // projects onto it too, and the section does not end
                        // there.
                        let ends_on = !interior && near_curve_end(pierce.point);
                        if !ends_on
                            && parallel_scale > 1e-12
                            && curve_tangent.cross(edge_tangent).length() <= 1e-3 * parallel_scale
                        {
                            continue;
                        }
                        self.add_edge_split(face.operand, edge, pierce.t)?;
                        if interior
                            && !split_parameters
                                .iter()
                                .any(|value: &f64| (*value - projection.u).abs() <= 1e-6)
                        {
                            split_parameters.push(projection.u);
                        }
                        fuzzy_junctions.push((
                            pierce.point,
                            if interior {
                                (2.0 * projection.distance).max(2e-5).min(1.5e-3)
                            } else {
                                (1.2 * projection.distance).max(2e-5)
                            },
                        ));
                    }
                }
            }
        }
        // SECTION DOMAIN-EXIT SPLITS. A refine section on an ANALYTIC carrier
        // (plane×cylinder ellipse; the deg-(2,1) revolution "band" faces of
        // 00000170) can leave the carrier's PARAMETRIC domain in a non-closed
        // direction: the ellipse rises past the band's v=1 top, so a piece
        // straddling the boundary has an on-carrier midpoint but an endpoint
        // 12.5 mm off the surface — `build_pcurve_on_surface` then hard-errors
        // (pool trial 645). The section is on the carrier only within [0,1];
        // add a split exactly where it crosses the domain edge so the
        // on-carrier arc mints cleanly and the off-carrier tail is skipped by
        // the existing midpoint Outside gate. Detection is closed-form via
        // `AnalyticSurface::project` (whose clamped-projection distance is the
        // 3D gap once a coordinate leaves [0,1]) — NEVER the poisoned NURBS
        // projector — and every candidate is verified evaluate-vs-evaluate
        // before it is accepted. Planes self-clip through their real coplanar
        // trim edges, so they are excluded. A shallow graze that never leaves
        // the on-carrier band (max excursion below `out_tol`) is left
        // untouched: only a genuine deep exit, the class that errors today,
        // gets a split. Escape hatch `BREP_SECTION_DOMAIN_SPLIT=0`.
        if refine && std::env::var("BREP_SECTION_DOMAIN_SPLIT").as_deref() != Ok("0") {
            for face in [first, second] {
                let Some(analytic) = face.face.surface.analytic() else {
                    continue;
                };
                if matches!(analytic, crate::AnalyticSurface::Plane { .. }) {
                    continue;
                }
                // Local per-point tolerances: `in_tol` is the on-carrier band
                // (kept << build_pcurve's 1e-3·scale endpoint floor so a
                // clipped endpoint always passes); `out_tol` confirms a real
                // deep exit before any split is registered.
                let off_gap = |f: f64| -> Result<f64, KernelRefusal> {
                    let point = curve.evaluate(f).or_refuse(KernelStage::Intersect, "evaluate")?;
                    Ok(analytic
                        .project(&face.face.surface, point)
                        .map(|projection| projection.point.sub(point).length())
                        .unwrap_or(0.0))
                };
                let local_scale = |f: f64| -> Result<f64, KernelRefusal> {
                    Ok(1.0 + curve.evaluate(f).or_refuse(KernelStage::Intersect, "evaluate")?.length())
                };
                let is_inside = |f: f64| -> Result<bool, KernelRefusal> {
                    Ok(off_gap(f)? <= 1e-4 * local_scale(f)?)
                };
                const STATIONS: usize = 96;
                let mut previous_f = whole_start;
                let mut previous_inside = is_inside(previous_f)?;
                let mut previous_far = off_gap(previous_f)?;
                for index in 1..=STATIONS {
                    let f = whole_start + (whole_end - whole_start) * (index as f64) / (STATIONS as f64);
                    let inside = is_inside(f)?;
                    let far = off_gap(f)?;
                    // A genuine crossing: the two flanking stations disagree on
                    // membership AND at least one is DEEP off the carrier
                    // (>out_tol), so marched-section noise around 0 can never
                    // register a spurious split.
                    let deep = previous_far.max(far) > 1e-2 * local_scale(f)?;
                    if inside != previous_inside && deep {
                        // Bisect to the boundary, keeping the accepted parameter
                        // on the INSIDE flank so the retained arc ends on-carrier.
                        let (mut lo, mut hi) = (previous_f, f);
                        let lo_inside = previous_inside;
                        for _ in 0..48 {
                            let mid = 0.5 * (lo + hi);
                            if is_inside(mid)? == lo_inside {
                                lo = mid;
                            } else {
                                hi = mid;
                            }
                        }
                        let crossing = if lo_inside { lo } else { hi };
                        if crossing > whole_start + 1e-7
                            && crossing < whole_end - 1e-7
                            && !near_curve_end(curve.evaluate(crossing).or_refuse(KernelStage::Intersect, "evaluate")?)
                            && off_gap(crossing)? <= 1e-4 * local_scale(crossing)?
                            && !split_parameters
                                .iter()
                                .any(|value: &f64| (*value - crossing).abs() <= 1e-6)
                        {
                            split_parameters.push(crossing);
                        }
                    }
                    previous_f = f;
                    previous_inside = inside;
                    previous_far = far;
                }
            }
        }
        // SECTION CARRIER-SEAM SPLITS. A section crossing the parameter seam of
        // a closed revolution carrier must end there: its pcurve is built on
        // the carrier's own period, and across the branch cut the fit clamps
        // half of it onto the seam line (the 2026-09-14 gear's fillet torus
        // against a box keeper: the x = 8.5 section ran azimuth -45 deg ->
        // +45 deg over the torus's u = 0, its trim read u = 0 for the first
        // half and its ends v = 1 and v = 0 for the rim's 0.25, and the face's
        // arrangement tore on it). A native face carries its seam EDGE on that
        // meridian, so the edge pierce above already splits there and this
        // adds nothing; an imported face can carry its seam edge elsewhere and
        // only a vertex on each rim at the carrier's seam, and then nothing
        // did. The crossing is where the section passes through the seam's
        // plane on the seam's half of it (`CarrierSeam`), bisected on the sign
        // of the signed distance between two stations that stand clear of the
        // plane by more than `overlap_limit` — a section riding IN the seam's
        // plane (a plane through a torus's equator, whose outer circle is the
        // v seam) reads round-off there, and a sign flip of round-off is no
        // crossing. A crossing an existing split already stands on (within
        // `overlap_limit`) or that is the section's own end is not added again.
        // The stations are the domain-exit pass's. Escape hatch
        // `BREP_SECTION_CARRIER_SEAM_SPLIT=0`.
        if refine && std::env::var("BREP_SECTION_CARRIER_SEAM_SPLIT").as_deref() != Ok("0") {
            const STATIONS: usize = 96;
            for face in [first, second] {
                for seam in CarrierSeam::of(&face.face.surface)? {
                    let offset_at = |f: f64| -> Result<f64, KernelRefusal> {
                        Ok(seam.offset(curve.evaluate(f).or_refuse(KernelStage::Intersect, "evaluate")?))
                    };
                    // The last station clear of the plane, and its offset.
                    let mut clear: Option<(f64, f64)> = None;
                    for index in 0..=STATIONS {
                        let f = whole_start + (whole_end - whole_start) * (index as f64) / (STATIONS as f64);
                        let current = offset_at(f)?;
                        if current.abs() <= overlap_limit {
                            continue;
                        }
                        let Some((previous_f, previous)) = clear.replace((f, current)) else {
                            continue;
                        };
                        if (previous < 0.0) != (current < 0.0) {
                            let (mut lo, mut hi) = (previous_f, f);
                            let lo_negative = previous < 0.0;
                            for _ in 0..48 {
                                let mid = 0.5 * (lo + hi);
                                if (offset_at(mid)? < 0.0) == lo_negative {
                                    lo = mid;
                                } else {
                                    hi = mid;
                                }
                            }
                            let crossing = 0.5 * (lo + hi);
                            let point = curve.evaluate(crossing).or_refuse(KernelStage::Intersect, "evaluate")?;
                            let mut standing = near_curve_end(point) || !seam.on_seam_half(point);
                            for value in &split_parameters {
                                if standing {
                                    break;
                                }
                                let existing = curve.evaluate(*value).or_refuse(KernelStage::Intersect, "evaluate")?;
                                standing = existing.sub(point).length() <= overlap_limit;
                            }
                            if !standing && crossing > whole_start && crossing < whole_end {
                                split_parameters.push(crossing);
                            }
                        }
                    }
                }
            }
        }
        let [curve_start, curve_end] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
        let split_tolerance = 2e-9f64.max((curve_end - curve_start) * KNOT_IDENTITY_TOL);
        split_parameters.sort_by(f64::total_cmp);
        split_parameters.dedup_by(|a, b| (*a - *b).abs() <= split_tolerance);
        // OVERLAP-JUNCTION EXCHANGE (rotated equator-tangency): when the
        // section curve RIDES an operand edge (the overlap branch above —
        // the tangency circle IS the cylinder's cap ring), every junction
        // minted ON the section (its split_parameters; e.g. the sphere-seam
        // crossing) is a junction OF that edge too. Without the matching
        // edge split the ring stays one CLOSED record while the section
        // arrives as open ARCS, and closed-ring vs open-arc records can
        // never endpoint-weld — both sides strand one-use (S=2 genus=1).
        // The overlap branch already exchanges the curve's ENDPOINTS; this
        // adds its interior junctions. Escape hatch:
        // BREP_OVERLAP_JUNCTION_SPLIT=0.
        if !overlapped_edges.is_empty()
            && std::env::var("BREP_OVERLAP_JUNCTION_SPLIT").as_deref() != Ok("0")
        {
            for parameter in &split_parameters {
                let point = curve.evaluate(*parameter).or_refuse(KernelStage::Intersect, "evaluate")?;
                for (operand, edge) in &overlapped_edges {
                    let projection = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
                    let nearest = edge.curve.evaluate(
                        projection
                            .u
                            .clamp(edge.t0.min(edge.t1), edge.t0.max(edge.t1)),
                    ).or_refuse(KernelStage::Intersect, "csg.imprint.builder")?;
                    if nearest.sub(point).length() <= overlap_limit {
                        self.add_edge_split(*operand, edge, projection.u)?;
                    }
                }
            }
        }
        if std::env::var("BREP_DEBUG_PROCESS_CURVE").is_ok() {
            eprintln!(
                "process_curve[{:?}/{:?}]: domain=[{curve_start:.6},{curve_end:.6}] splits={} at {:?}",
                first.key(),
                second.key(),
                split_parameters.len(),
                split_parameters
                    .iter()
                    .map(|value| (value * 1e4).round() / 1e4)
                    .collect::<Vec<_>>()
            );
        }
        let mut pieces = Vec::new();
        let mut rest = curve;
        for parameter in split_parameters {
            if parameter <= curve_start + split_tolerance
                || parameter >= curve_end - split_tolerance
            {
                continue;
            }
            let (left, right) = rest.split(parameter).or_refuse(KernelStage::Intersect, "split")?;
            pieces.push(left);
            rest = right;
        }
        pieces.push(rest);

        let debug_process = std::env::var("BREP_DEBUG_PROCESS_CURVE").is_ok();
        if debug_process {
            let spans: Vec<String> = pieces
                .iter()
                .map(|piece| {
                    piece
                        .domain()
                        .map(|[a, b]| format!("[{a:.4},{b:.4}]"))
                        .unwrap_or_else(|_| "[?]".into())
                })
                .collect();
            eprintln!(
                "process_curve[{:?}/{:?}]: {} piece(s): {}",
                first.key(),
                second.key(),
                pieces.len(),
                spans.join(" ")
            );
        }
        for piece in pieces {
            let [t0, t1] = piece.domain().or_refuse(KernelStage::Intersect, "domain")?;
            if debug_process {
                eprintln!(
                    "process_curve[{:?}/{:?}]: ENTER piece [{t0:.4},{t1:.4}] end_dist={:.3e} rough_len={:.3e}",
                    first.key(),
                    second.key(),
                    piece.evaluate(t0).or_refuse(KernelStage::Intersect, "evaluate")?.sub(piece.evaluate(t1).or_refuse(KernelStage::Intersect, "evaluate")?).length(),
                    curve_length_rough(&piece)?
                );
            }
            if piece.evaluate(t0).or_refuse(KernelStage::Intersect, "evaluate")?.sub(piece.evaluate(t1).or_refuse(KernelStage::Intersect, "evaluate")?).length() <= self.tolerance * 10.0
                && curve_length_rough(&piece)? <= self.tolerance * 100.0
            {
                if debug_process {
                    let p = piece.evaluate((t0 + t1) / 2.0).or_refuse(KernelStage::Intersect, "evaluate")?;
                    eprintln!(
                        "process_curve[{:?}/{:?}]: sub-seg SKIP short len={:.3e} mid=({:.5},{:.5},{:.5})",
                        first.key(),
                        second.key(),
                        curve_length_rough(&piece)?,
                        p.x, p.y, p.z
                    );
                }
                continue;
            }
            let midpoint = piece.evaluate((t0 + t1) / 2.0).or_refuse(KernelStage::Intersect, "evaluate")?;
            let mut statuses = HashMap::default();
            for face in test_faces {
                let classify = |point| -> Result<PolygonClass, KernelRefusal> {
                    let projection = project_point_to_surface(&face.face.surface, point)
                        .or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
                    parameter_point_in_face(face.face, Vec2 { x: projection.u, y: projection.v }, 1e-7)
                        .or_refuse(KernelStage::Intersect, "csg.imprint.builder")
                };
                let mut status = classify(midpoint)?;
                if status == PolygonClass::Boundary && !self.face_has_seam_at(*face, midpoint)? {
                    // A tangential trim contact is not an interval boundary:
                    // a section may touch the rim at its midpoint and otherwise
                    // lie inside the face. Check both nearby and farther points
                    // on each side before treating the entire piece as a rim.
                    // Riding boundaries and mixed/ambiguous probes retain the
                    // original boundary handling, including seam ownership.
                    let mut flank = None;
                    for fraction in [0.25, 0.49, 0.51, 0.75] {
                        let point = piece.evaluate(t0 + fraction * (t1 - t0))
                            .or_refuse(KernelStage::Intersect, "evaluate")?;
                        let class = classify(point)?;
                        if class == PolygonClass::Boundary || flank.is_some_and(|old| old != class) {
                            flank = None;
                            break;
                        }
                        flank = Some(class);
                    }
                    if let Some(class) = flank { status = class; }
                }
                statuses.insert(face.key(), status);
            }
            if statuses
                .values()
                .any(|status| *status == PolygonClass::Outside)
            {
                if debug_process {
                    eprintln!(
                        "process_curve[{:?}/{:?}]: sub-seg SKIP outside len={:.3e} mid=({:.5},{:.5},{:.5}) statuses={:?}",
                        first.key(),
                        second.key(),
                        curve_length_rough(&piece)?,
                        midpoint.x, midpoint.y, midpoint.z,
                        statuses
                    );
                }
                continue;
            }
            let mut attach = Vec::new();
            let mut invalid_boundary = false;
            for face in target_faces {
                if statuses.get(&face.key()) == Some(&PolygonClass::Boundary) {
                    if !self.face_has_seam_at(*face, midpoint)? {
                        // Open offset sheets have no neighbouring material face
                        // to own this cut. Keep the accurate marched section on
                        // the other sheet, but do not duplicate a boundary copy.
                        let recover = self.open_carriers
                            && refine
                            && target_faces.len() == 2
                            && !self.pieces.iter().any(|piece| {
                                piece.support_faces == [first.key(), second.key()]
                            });
                        if !recover {
                            invalid_boundary = true;
                            break;
                        }
                    }
                } else {
                    attach.push(*face);
                }
            }
            if invalid_boundary || attach.is_empty() {
                if debug_process {
                    eprintln!(
                        "process_curve[{:?}/{:?}]: sub-seg SKIP {} len={:.3e} mid=({:.5},{:.5},{:.5})",
                        first.key(),
                        second.key(),
                        if invalid_boundary { "invalid-boundary" } else { "attach-empty" },
                        curve_length_rough(&piece)?,
                        midpoint.x, midpoint.y, midpoint.z
                    );
                }
                continue;
            }
            let mut start = self.polish_endpoint(
                piece.evaluate(t0).or_refuse(KernelStage::Intersect, "evaluate")?,
                &first.face.surface,
                &second.face.surface,
            )?;
            let mut end = self.polish_endpoint(
                piece.evaluate(t1).or_refuse(KernelStage::Intersect, "evaluate")?,
                &first.face.surface,
                &second.face.surface,
            )?;
            let fuzzy_at = |point: Vec3| {
                fuzzy_junctions
                    .iter()
                    .filter_map(|(junction, radius)| {
                        let distance = junction.sub(point).length();
                        (distance <= radius.max(1e-5)).then_some((*junction, *radius, distance))
                    })
                    .min_by(|a, b| a.2.total_cmp(&b.2))
            };
            let start_fuzzy = fuzzy_at(start);
            let end_fuzzy = fuzzy_at(end);
            let start_radius = start_fuzzy.map(|value| value.1).unwrap_or(0.0);
            let end_radius = end_fuzzy.map(|value| value.1).unwrap_or(0.0);
            if let Some((junction, _, _)) = start_fuzzy {
                start = junction;
            }
            if let Some((junction, _, _)) = end_fuzzy {
                end = junction;
            }
            let (start_vertex_id, canonical_start) = self.vertex(start, start_radius);
            let (end_vertex_id, canonical_end) = self.vertex(end, end_radius);
            start = canonical_start;
            end = canonical_end;
            if start_vertex_id == end_vertex_id
                && curve_length_rough(&piece)?
                    < (4.0 * piece.evaluate(t0).or_refuse(KernelStage::Intersect, "evaluate")?.sub(piece.evaluate(t1).or_refuse(KernelStage::Intersect, "evaluate")?).length()).max(1e-3)
            {
                if debug_process {
                    eprintln!(
                        "process_curve[{:?}/{:?}]: sub-seg SKIP loopback v{start_vertex_id} len={:.3e} start=({:.5},{:.5},{:.5}) end=({:.5},{:.5},{:.5}) radii=({start_radius:.3e},{end_radius:.3e})",
                        first.key(),
                        second.key(),
                        curve_length_rough(&piece)?,
                        start.x, start.y, start.z,
                        end.x, end.y, end.z
                    );
                }
                continue;
            }
            let snapped = snap_curve_ends(&piece, start, end)?;
            let pcurves = attach
                .iter()
                .map(|face| {
                    Ok(FacePcurve {
                        operand: face.operand,
                        face_id: face.face.id,
                        // Seeded march (see build_pcurve_on_surface_marched): a
                        // section riding a compressed/self-overlapping carrier row
                        // must not alias interior samples onto a distant preimage
                        // sheet, which folds the pcurve across the trim and strands
                        // the shared section one-use. Fail-soft to the global build.
                        // On the face's CHART: a section crossing a lifted
                        // carrier's seam is ONE curve in the band the face's own
                        // loops are drawn in, where the carrier would break it at
                        // the seam and place half of it a period away.
                        pcurve: build_pcurve_on_surface_marched(face.chart(), &snapped).or_refuse(KernelStage::Intersect, "build_pcurve_on_surface_marched")?,
                    })
                })
                .collect::<Result<Vec<_>, KernelRefusal>>()?;
            let id = self.next_id;
            self.next_id += 1;
            if debug_process {
                eprintln!(
                    "process_curve[{:?}/{:?}]: MINT piece {id} [{t0:.4},{t1:.4}] attach={:?}",
                    first.key(),
                    second.key(),
                    attach.iter().map(|face| face.key()).collect::<Vec<_>>()
                );
            }
            for face in &attach {
                self.by_face.entry(face.key()).or_default().push(id);
            }
            self.pieces.push(ImprintPieceRecord {
                id,
                curve: snapped,
                t0,
                t1,
                start_vertex_id,
                end_vertex_id,
                pcurves,
                support_faces: [first.key(), second.key()],
                shared_edge: None,
            });
        }
        Ok(())
    }
}
