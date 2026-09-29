use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use super::types::{Chain, ChainSource};
use super::sampling::{enforce_boundary_crossing_parity, refine_near_hints, sag_refine_chain, sample_chain, trimmed_curve};
use super::loops::{build_loop, edge_for, interior_points, pcurve_for, point_in_polygon, vertex_point};
/// Imprint id-index shared across every face of one operand. Built ONCE in
/// `fragment_solid`; without it each face rebuilt `pieces_by_id`/`vertex_by_id`
/// over ALL pieces (O(F*(P + P*V))) and linear-scanned `by_face`. Lookup-only,
/// so every face gets the identical pieces/points the scans returned.
struct FragmentIndex<'a> {
    by_face: HashMap<(u8, u64), &'a [u64]>,
    pieces_by_id: HashMap<u64, &'a ImprintPieceRecord>,
    vertex_points: HashMap<u64, Vec3>,
    cosurface_pairs: HashSet<(FaceKey, FaceKey)>,
}

impl<'a> FragmentIndex<'a> {
    fn build(imprint: &'a ImprintResultRecord) -> Self {
        let mut by_face: HashMap<(u8, u64), &'a [u64]> = HashMap::default();
        for entry in &imprint.by_face {
            // `.entry().or_insert` keeps the FIRST entry, matching the previous
            // `by_face.iter().find(...)` (keys are unique in practice anyway).
            by_face
                .entry((entry.operand, entry.face_id))
                .or_insert_with(|| entry.piece_ids.as_slice());
        }
        FragmentIndex {
            by_face,
            cosurface_pairs: imprint.cosurface_pairs.iter()
                .flat_map(|&(a,b)| [(a,b),(b,a)]).collect(),
            pieces_by_id: imprint
                .pieces
                .iter()
                .map(|piece| (piece.id, piece))
                .collect(),
            vertex_points: imprint
                .vertices
                .iter()
                .map(|vertex| (vertex.id, vertex.point))
                .collect(),
        }
    }
}

/// Back-compat entry point: builds the imprint index for this single face.
/// `fragment_solid` builds it once and calls `fragment_face_indexed` directly.
pub fn fragment_face(
    solid: &BrepSolid,
    operand: u8,
    face: &FaceRecord,
    imprint: &ImprintResultRecord,
) -> Result<Vec<FaceFragmentRecord>, KernelRefusal> {
    let index = FragmentIndex::build(imprint);
    fragment_face_indexed(solid, operand, face, &index)
}

/// The winding the SOURCE face's own outer loop uses in its (u,v) chart:
/// `Some(true)` when it is negative (clockwise), `Some(false)` when positive,
/// `None` when the sign cannot be trusted.
///
/// `build_loop` orients a rebuilt fragment loop against the arrangement, whose
/// outer cycles are ALWAYS positive-area (`arrangement.rs`: a cycle goes to
/// `positive` only when `area > tol²`, and negative cycles become holes), and
/// reversed it only for `!same_sense` — *material runs counter-clockwise in
/// (u,v) exactly when `same_sense`*.
///
/// That rule is CORRECT and is this kernel's stated invariant: it is what
/// `validate_uv_wire` checks (`brep/topology/validate.rs`, "wire winding
/// disagrees with face sense"), what `watertight_tessellation::periodic::trim`
/// cites as the authority for the same question, and what nine sites in `src/`
/// compare a winding against. It is also forced by the definitions — with
/// `n_s = S_u × S_v` and `n_f = same_sense ? n_s : −n_s`, material-on-the-left
/// projected into the chart with `n_s` up leaves no freedom.
///
/// So a face carrying a CW outer loop with `same_sense = true` is MALFORMED,
/// not merely unusual. This function exists because one arrives anyway: nothing
/// refuses it on the way in, and `validate_uv_wire`'s verdict is filed under
/// `ValidationReport::wire_warnings`, which `validate()` does not return — so
/// every `validate().is_empty()` in the kernel reads clean on it.
///
/// Reading the source winding here is DEFENCE IN DEPTH, not a blessing of the
/// chart: it stops the boolean refusing on such a face and builds the correct
/// body instead. It does not make the face correct, and the real defect is
/// upstream in whatever produced it.
///
/// Measured on the 2026-09-21 through-tunnel body: five of the stretched
/// wedge's six faces wind positive, `Box_PZ` winds −1.2334e3 with `same_sense =
/// true` (the plain box's `Box_PZ` winds +1.2143e3; `TF3` is what flips it).
/// Rebuilding `Box_PZ` against the arrangement's CCW convention flips every
/// coedge on it, which is the six same-sense edges the boolean then refuses on.
///
/// Restricted to OPEN charts on purpose. On a closed/periodic chart a raw
/// shoelace is meaningless without the seam unwrap the arrangement scan does,
/// and that path already has its own material-orientation authority in
/// `full_span_loop_area`; leaving periodic faces on the old rule keeps this
/// change to the case it was measured on. `None` also covers a loop with too
/// few samples or an area too small to carry a reliable sign — the caller then
/// keeps today's `!same_sense`.
fn source_loop_winding_negative(face: &FaceRecord) -> Result<Option<bool>, KernelRefusal> {
    let closed = face
        .surface
        .closed_directions()
        .or_refuse(KernelStage::Fragment, "closed_directions")?;
    if closed.0 || closed.1 {
        return Ok(None);
    }
    // The OUTER loop is the one bounding the most parameter area; holes wind
    // against it. Reading `loops[0]` would trust a convention this function
    // exists to stop trusting.
    let mut best: Option<f64> = None;
    for loop_record in &face.loops {
        let mut points: Vec<Vec2> = Vec::new();
        for coedge in &loop_record.coedges {
            let [d0, d1] = coedge
                .pcurve
                .domain()
                .or_refuse(KernelStage::Fragment, "domain")?;
            const SAMPLES: usize = 24;
            for step in 0..SAMPLES {
                let fraction = step as f64 / SAMPLES as f64;
                // `d0 -> d1` ALWAYS. A pcurve is stored in its coedge's own
                // traversal direction -- `reverse_fragment` flips `forward` and
                // calls `pcurve.reversed()` together, and the loop-polygon
                // builder below walks every pcurve `d0 -> d1` regardless of
                // `forward`. Consulting `forward` here would reverse those
                // segments in place and trace a zig-zag: on the plain box's
                // `Box_NZ` (60.715 x 20 in parameter space) that yields 607.15,
                // exactly half the 1214.3 the loop actually bounds, and the
                // sign of a self-intersecting polygon is not the loop's
                // winding.
                let t = d0 + (d1 - d0) * fraction;
                let point = coedge
                    .pcurve
                    .evaluate(t)
                    .or_refuse(KernelStage::Fragment, "evaluate")?;
                points.push(Vec2 { x: point.x, y: point.y });
            }
        }
        if points.len() < 3 {
            continue;
        }
        let mut area = 0.0;
        for (index, point) in points.iter().enumerate() {
            let next = points[(index + 1) % points.len()];
            area += 0.5 * (point.x * next.y - next.x * point.y);
        }
        if best.map(|current: f64| area.abs() > current.abs()).unwrap_or(true) {
            best = Some(area);
        }
    }
    // Scale the "is this sign real" floor to the chart, so a millimetre-wide
    // face is not judged by a millimetre-squared absolute.
    let u_domain = KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)
        .or_refuse(KernelStage::Fragment, "new")?
        .domain();
    let v_domain = KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)
        .or_refuse(KernelStage::Fragment, "new")?
        .domain();
    let chart = (u_domain[1] - u_domain[0]) * (v_domain[1] - v_domain[0]);
    let floor = (chart.abs() * 1e-9).max(f64::MIN_POSITIVE);
    Ok(match best {
        Some(area) if area.abs() > floor => Some(area < 0.0),
        _ => None,
    })
}

/// A cut against the other operand's coincident copy of an incident face
/// inherits that already-established face identity. This permits bounded rim
/// repair without treating an unrelated nearby curve as the same boundary.
fn boundary_has_cosurface_partner(
    solid: &BrepSolid,
    face: FaceKey,
    boundary_edge: u64,
    piece: &ImprintPieceRecord,
    index: &FragmentIndex,
) -> bool {
    piece.support_faces.iter().filter(|other| other.operand != face.operand).any(|other| {
        solid.shells.iter().flat_map(|shell| &shell.faces).any(|neighbour| {
            neighbour.id != face.face_id
                && index.cosurface_pairs.contains(&(FaceKey { operand:face.operand, face_id:neighbour.id }, *other))
                && neighbour.loops.iter().flat_map(|lp| &lp.coedges)
                    .any(|coedge| coedge.edge_id == boundary_edge)
        })
    })
}

/// Detect a distinct narrow strip before replacing its curves by chords.
/// Search tolerance only nominates a nearby boundary; positive separation in
/// model space and parallel interior tangents establish a resolvable strip.
fn thin_boundary_resolution(
    solid: &BrepSolid,
    face: &FaceRecord,
    face_key: FaceKey,
    piece_ids: &[u64],
    index: &FragmentIndex,
    search: f64,
) -> Option<(f64, f64)> {
    let mut resolution: Option<(f64, f64)> = None;
    for piece_id in piece_ids {
        let Some(piece) = index.pieces_by_id.get(piece_id) else { continue };
        let Ok(cut) = pcurve_for(piece, face_key) else { continue };
        for boundary in face.loops.iter().flat_map(|lp| &lp.coedges) {
            if boundary_has_cosurface_partner(solid, face_key, boundary.edge_id, piece, index) {
                continue;
            }
            let measure = || -> Option<(f64, f64)> {
                let [lo, hi] = cut.domain().ok()?;
                let middle = cut.evaluate((lo + hi) * 0.5).ok()?;
                let nearest = project_point_to_curve(&boundary.pcurve, middle).ok()?;
                if nearest.distance > search * 8.0 { return None; }
                let mut low_gap = f64::INFINITY;
                let mut high_gap = 0.0f64;
                let mut identity = 0.0f64;
                for fraction in [0.125, 0.25, 0.5, 0.75, 0.875] {
                    let parameter = lo + (hi - lo) * fraction;
                    let cut_sample = cut.derivatives(parameter, 1).ok()?;
                    let point = cut_sample[0];
                    let projected = project_point_to_curve(&boundary.pcurve, point).ok()?;
                    if projected.distance > search * 8.0 { return None; }
                    let boundary_sample = boundary.pcurve.derivatives(projected.u, 1).ok()?;
                    let a = cut_sample[1];
                    let b = boundary_sample[1];
                    let lengths = a.length() * b.length();
                    if lengths <= 1e-30 || a.dot(b).abs() < 0.995 * lengths { return None; }
                    let actual = face.surface.evaluate(point.x, point.y).ok()?;
                    let rim = face.surface.evaluate(projected.point.x, projected.point.y).ok()?;
                    if actual.sub(rim).length() <= 8e-7 { return None; }
                    let floor = crate::classification::surface_uv_band(
                        &face.surface, point.x, point.y, 1e-7,
                    );
                    if projected.distance <= floor * 8.0 { return None; }
                    low_gap = low_gap.min(projected.distance);
                    high_gap = high_gap.max(projected.distance);
                    identity = identity.max(floor);
                }
                // A crossing or isolated tangency does not describe a strip.
                if high_gap > low_gap * 4.0 { return None; }
                Some((search.min((low_gap / 128.0).max(identity)), search.min(low_gap / 8.0)))
            };
            if let Some((arrangement, sampling)) = measure() {
                resolution = Some(match resolution {
                    Some((a, s)) => (a.min(arrangement), s.min(sampling)),
                    None => (arrangement, sampling),
                });
            }
        }
    }
    resolution
}

fn fragment_face_indexed(
    solid: &BrepSolid,
    operand: u8,
    face: &FaceRecord,
    index: &FragmentIndex,
) -> Result<Vec<FaceFragmentRecord>, KernelRefusal> {
    let face_key = FaceKey {
        operand,
        face_id: face.id,
    };
    let piece_ids = index
        .by_face
        .get(&(operand, face.id))
        .copied()
        .unwrap_or(&[]);
    if piece_ids.is_empty() {
        let u = KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u).or_refuse(KernelStage::Fragment, "new")?.domain();
        let v = KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v).or_refuse(KernelStage::Fragment, "new")?.domain();
        let uv = Vec2 {
            x: (u[0] + u[1]) / 2.0,
            y: (v[0] + v[1]) / 2.0,
        };
        if parameter_point_in_face(face, uv, 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")? != PolygonClass::Inside {
            // Trimmed faces need the arrangement path even without cuts to
            // find a point in their actual outer loop.
        } else {
            let mut extra_test_points = Vec::new();
            for (fu, fv) in [(0.35, 0.35), (0.65, 0.65)] {
                let candidate = Vec2 {
                    x: u[0] + (u[1] - u[0]) * fu,
                    y: v[0] + (v[1] - v[0]) * fv,
                };
                if parameter_point_in_face(face, candidate, 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")? == PolygonClass::Inside {
                    extra_test_points.push(face.surface.evaluate(candidate.x, candidate.y).or_refuse(KernelStage::Fragment, "evaluate")?);
                }
            }
            return Ok(vec![FaceFragmentRecord {
                operand,
                source_face_id: face.id,
                surface: face.surface.clone(),
                same_sense: face.same_sense,
                loops: face
                    .loops
                    .iter()
                    .map(|loop_record| FragmentLoop {
                        coedges: loop_record
                            .coedges
                            .iter()
                            .map(|coedge| FragmentCoedge {
                                source: FragmentEdgeSource::Boundary {
                                    operand,
                                    edge_id: coedge.edge_id,
                                },
                                forward: coedge.forward,
                                pcurve: coedge.pcurve.clone(),
                            })
                            .collect(),
                    })
                    .collect(),
                test_point: face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Fragment, "evaluate")?,
                test_uv: uv,
                extra_test_points,
            }]);
        }
    }

    let u_domain = KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u).or_refuse(KernelStage::Fragment, "new")?.domain();
    let v_domain = KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v).or_refuse(KernelStage::Fragment, "new")?.domain();
    let diagonal =
        ((u_domain[1] - u_domain[0]).powi(2) + (v_domain[1] - v_domain[0]).powi(2)).sqrt();
    let original_resolution = 1e-7f64.max(diagonal * 1e-6);
    let strip_resolution = thin_boundary_resolution(solid, face, face_key, piece_ids, index, original_resolution);
    let (arrangement_tolerance, sampling_bound) = strip_resolution
        .unwrap_or((original_resolution, original_resolution));
    let snap_threshold = 0.1 * diagonal;
    let junction_radius = (arrangement_tolerance * 50.0).max(diagonal * 1.5e-3);
    // WRAPPED-BAND RE-BASE: a face on a closed direction whose material is the
    // wrapped COMPLEMENT of its trim hull (two full-wrap rims at the hull
    // edges, material crossing the seam between them — trial 35's torus face:
    // rims at v=0.25/1.0, material v∈[0,0.25]) cannot be arranged in the flat
    // domain rectangle: a cut through the band ends at v=0 while its junction
    // partner (the split rim vertex) sits at v=1, so every cut dangles, cycle
    // extraction fails, and the face is dropped wholesale (one-use cascade).
    // Re-base into the material-coherent frame [hull_high, hull_low+period]:
    // shift every chain point on the low side of the hull window (+period), so
    // rims and cuts junction in ONE unwrapped chart, and wrap region test
    // points back into the domain on exit. Detection mirrors the imprint's
    // restricted-carrier wrap guard: the hull-complement strip contains no
    // loop curves, so one sample at its middle decides materiality exactly.
    // Escape hatch: BREP_BAND_REBASE=0.
    let band_rebase: [Option<(f64, f64)>; 2] = {
        let mut rebase = [None, None];
        let closed = face.surface.closed_directions().or_refuse(KernelStage::Fragment, "closed_directions")?;
        if (closed.0 || closed.1) && std::env::var("BREP_BAND_REBASE").as_deref() != Ok("0") {
            // Per-coedge hulls on each axis, plus whether any ISO-RIM
            // (a coedge whose pcurve is near-constant on the axis) sits at a
            // domain edge — the discriminator for the wrapped-band class.
            // A seam-straddling boundary band (helmet Face_15, whose source
            // pcurves are already unwrapped past the domain) has no such rim
            // and its frame is already coherent; it must NOT be re-based.
            let mut coverage: [Vec<[f64; 2]>; 2] = [Vec::new(), Vec::new()];
            let mut edge_rim = [false, false];
            let mut hull = [
                [f64::INFINITY, f64::NEG_INFINITY],
                [f64::INFINITY, f64::NEG_INFINITY],
            ];
            for coedge in face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
            {
                let mut extent = [
                    [f64::INFINITY, f64::NEG_INFINITY],
                    [f64::INFINITY, f64::NEG_INFINITY],
                ];
                for control in &coedge.pcurve.control_points {
                    if control.w.abs() <= 1e-300 {
                        continue;
                    }
                    let point = [control.x / control.w, control.y / control.w];
                    for axis in 0..2 {
                        extent[axis][0] = extent[axis][0].min(point[axis]);
                        extent[axis][1] = extent[axis][1].max(point[axis]);
                        hull[axis][0] = hull[axis][0].min(point[axis]);
                        hull[axis][1] = hull[axis][1].max(point[axis]);
                    }
                }
                for axis in 0..2 {
                    if !extent[axis][0].is_finite() {
                        continue;
                    }
                    let domain = if axis == 0 { u_domain } else { v_domain };
                    let span = domain[1] - domain[0];
                    coverage[axis].push(extent[axis]);
                    let flat = extent[axis][1] - extent[axis][0] <= 1e-3 * span;
                    let at_edge = (extent[axis][0] - domain[0]).abs() <= 1e-6 * span
                        || (extent[axis][1] - domain[1]).abs() <= 1e-6 * span;
                    if flat && at_edge {
                        edge_rim[axis] = true;
                    }
                }
            }
            for axis in 0..2 {
                if !(if axis == 0 { closed.0 } else { closed.1 }) || !edge_rim[axis] {
                    continue;
                }
                let domain = if axis == 0 { u_domain } else { v_domain };
                let span = domain[1] - domain[0];
                // Merge the coverage intervals (clamped into the domain) and
                // find the largest uncovered gap. The gap contains no boundary
                // curve, so it is uniformly material or uniformly void — one
                // sample at its middle decides exactly. A strictly INTERIOR
                // void gap means the material wraps through the domain edge.
                let mut intervals: Vec<[f64; 2]> = coverage[axis]
                    .iter()
                    .map(|window| {
                        [window[0].max(domain[0]), window[1].min(domain[1])]
                    })
                    .filter(|window| window[1] >= window[0])
                    .collect();
                intervals.sort_by(|a, b| a[0].total_cmp(&b[0]));
                let mut gaps: Vec<[f64; 2]> = Vec::new();
                let mut reach = domain[0];
                for window in &intervals {
                    if window[0] > reach {
                        gaps.push([reach, window[0]]);
                    }
                    reach = reach.max(window[1]);
                }
                if reach < domain[1] {
                    gaps.push([reach, domain[1]]);
                }
                let other_mid = (hull[1 - axis][0].max(if axis == 0 {
                    v_domain[0]
                } else {
                    u_domain[0]
                }) + hull[1 - axis][1].min(if axis == 0 {
                    v_domain[1]
                } else {
                    u_domain[1]
                })) / 2.0;
                let mut void: Option<[f64; 2]> = None;
                for gap in gaps {
                    if gap[1] - gap[0] <= 1e-6 * span {
                        continue;
                    }
                    let mid = (gap[0] + gap[1]) / 2.0;
                    let sample = if axis == 0 {
                        Vec2 {
                            x: mid,
                            y: other_mid,
                        }
                    } else {
                        Vec2 {
                            x: other_mid,
                            y: mid,
                        }
                    };
                    if parameter_point_in_face(face, sample, 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")? != PolygonClass::Inside
                        && void
                            .map(|best| gap[1] - gap[0] > best[1] - best[0])
                            .unwrap_or(true)
                    {
                        void = Some(gap);
                    }
                }
                if let Some([g0, g1]) = void {
                    // The material+boundary arc wraps the domain edge exactly
                    // when boundary coverage touches BOTH edge representations
                    // (the on-seam rim at one, the material-side chains at the
                    // other) with the void between them. An ordinary band
                    // (material inside the rims) touches only one edge and
                    // must keep the flat frame.
                    let touches_low = intervals
                        .iter()
                        .any(|window| window[0] <= domain[0] + 1e-6 * span);
                    let touches_high = intervals
                        .iter()
                        .any(|window| window[1] >= domain[1] - 1e-6 * span);
                    if touches_low && touches_high {
                        // Shift everything on the low side of the void up one
                        // period so the band is contiguous in the arrangement
                        // frame.
                        rebase[axis] = Some(((g0 + g1) / 2.0, span));
                    }
                }
            }
        }
        rebase
    };
    let rebase_point = |mut point: Vec2| -> Vec2 {
        if let Some((cutoff, period)) = band_rebase[0] {
            if point.x < cutoff {
                point.x += period;
            }
        }
        if let Some((cutoff, period)) = band_rebase[1] {
            if point.y < cutoff {
                point.y += period;
            }
        }
        point
    };
    // Region test points computed in a re-based or unwrapped frame must be
    // folded back into the domain before trim classification / evaluation.
    // Wrapping is gated on the axis being CLOSED (a coherent flat face never
    // produces out-of-domain interior points, so this is a no-op there).
    let wrap_back = {
        let closed = face.surface.closed_directions().or_refuse(KernelStage::Fragment, "closed_directions")?;
        move |mut point: Vec2| -> Vec2 {
            if closed.0 {
                let span = u_domain[1] - u_domain[0];
                if point.x < u_domain[0] || point.x > u_domain[1] {
                    point.x = u_domain[0] + (point.x - u_domain[0]).rem_euclid(span);
                }
            }
            if closed.1 {
                let span = v_domain[1] - v_domain[0];
                if point.y < v_domain[0] || point.y > v_domain[1] {
                    point.y = v_domain[0] + (point.y - v_domain[0]).rem_euclid(span);
                }
            }
            point
        }
    };
    let pieces_by_id = &index.pieces_by_id;
    let mut cut_hints = Vec::new();
    let mut active_pieces = Vec::new();
    let mut cut_keys = HashMap::default();
    for piece_id in piece_ids {
        let piece = pieces_by_id
            .get(piece_id)
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.face_split", "fragment_face: missing imprint piece"))?;
        let pcurve = pcurve_for(piece, face_key)?;
        let [start, end] = pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
        let a = pcurve.evaluate(start).or_refuse(KernelStage::Fragment, "evaluate")?;
        let b = pcurve.evaluate(end).or_refuse(KernelStage::Fragment, "evaluate")?;
        let middle = pcurve.evaluate((start + end) / 2.0).or_refuse(KernelStage::Fragment, "evaluate")?;
        for point in [a, b] {
            cut_hints.push(Vec2 {
                x: point.x,
                y: point.y,
            });
        }
        let quantize = |point: Vec3| {
            (
                (point.x / arrangement_tolerance).round() as i64,
                (point.y / arrangement_tolerance).round() as i64,
            )
        };
        let mut endpoints = [quantize(a), quantize(b)];
        endpoints.sort();
        let key = (endpoints[0], quantize(middle), endpoints[1]);
        if cut_keys.insert(key, *piece_id).is_none() {
            active_pieces.push(*piece_id);
        }
    }

    let mut segments = Vec::new();
    let mut chains = Vec::new();
    let mut vertex_images: std::collections::HashMap<String, Vec<Vec2>> =
        std::collections::HashMap::new();
    let mut endpoint_images: Vec<(Vec3, Vec2)> = Vec::new();
    // Each boundary image carries its 3D vertex point so the boundary-side
    // canonicalization can refuse to unify two DISTINCT boundary vertices.
    let mut boundary_images: Vec<(Vec2, Vec3)> = Vec::new();
    // BOUNDARY-VERTEX ANCHOR (t427 box-corner graze). The boundary-side snap
    // below (rule 3) merges a chain endpoint onto a same-side boundary image
    // within arr_tol*100 with NO 3D check. For a CUT/section terminus that is
    // the intended near-tangent graze bridge onto a trim edge. For a BOUNDARY
    // endpoint it is a bug: two genuinely-distinct same-operand boundary
    // vertices (t427: box corner v8 at v=68.4851 vs the edge-16 split A at
    // v=68.4783, 6.8e-3 apart on the u=max domain edge, 6.8e-3 apart in 3D ≫
    // the 1e-5 weld floor) get unified, which droops edge 21's pcurve and
    // mints a phantom corner lens bounded by derived edges → a one-use cluster
    // that only strands in UNION (where the unsectioned neighbour keeps the
    // whole edge 21). Anchor a boundary endpoint to a same-side image only when
    // they are the SAME physical vertex (3D-coincident within the weld floor).
    // Hatch BREP_BOUNDARY_VERTEX_ANCHOR=0 restores the pre-fix behaviour.
    let boundary_vertex_anchor_off =
        std::env::var("BREP_BOUNDARY_VERTEX_ANCHOR").as_deref() == Ok("0");
    let boundary_side = |point: Vec2| {
        let distances = [
            (point.x - u_domain[0]).abs(),
            (point.x - u_domain[1]).abs(),
            (point.y - v_domain[0]).abs(),
            (point.y - v_domain[1]).abs(),
        ];
        let index = distances
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map(|entry| entry.0)
            .unwrap();
        (distances[index] <= arrangement_tolerance * 50.0).then_some(index)
    };

    let mut add_chain = |mut points: Vec<Vec2>,
                         source: ChainSource,
                         vertex_start_key: String,
                         vertex_start_point: Vec3,
                         vertex_end_key: String,
                         vertex_end_point: Vec3,
                         is_boundary: bool|
     -> Result<(), KernelRefusal> {
        for (at, key, point3) in [
            (0usize, vertex_start_key, vertex_start_point),
            (points.len() - 1, vertex_end_key, vertex_end_point),
        ] {
            let point = points[at];
            let images = vertex_images.entry(key).or_default();
            let canonical = images
                .iter()
                .copied()
                .find(|image| image.sub(point).length() <= snap_threshold)
                .or_else(|| {
                    endpoint_images
                        .iter()
                        .find(|(candidate3, candidate)| {
                            candidate3.sub(point3).length() <= 1e-5
                                && candidate.sub(point).length() <= arrangement_tolerance * 100.0
                        })
                        .map(|candidate| candidate.1)
                })
                .or_else(|| {
                    boundary_side(point).and_then(|side| {
                        boundary_images
                            .iter()
                            .find(|(candidate, candidate3)| {
                                boundary_side(*candidate) == Some(side)
                                    && candidate.sub(point).length()
                                        <= arrangement_tolerance * 100.0
                                    // Boundary endpoints only merge to a same-side
                                    // image when 3D-coincident (same physical
                                    // vertex); cut endpoints keep the graze bridge.
                                    && (boundary_vertex_anchor_off
                                        || !is_boundary
                                        || candidate3.sub(point3).length() <= 1e-5)
                            })
                            .map(|(candidate, _)| *candidate)
                    })
                });
            if let Some(canonical) = canonical {
                points[at] = canonical;
                images.push(canonical);
            } else {
                images.push(point);
                endpoint_images.push((point3, point));
                if is_boundary && boundary_side(point).is_some() {
                    boundary_images.push((point, point3));
                }
            }
        }
        if points.len() > 2 {
            let start = points[0];
            let end = points[points.len() - 1];
            let mut kept = vec![start];
            kept.extend(points[1..points.len() - 1].iter().copied().filter(|point| {
                point.sub(start).length() > junction_radius
                    && point.sub(end).length() > junction_radius
            }));
            kept.push(end);
            points = kept;
        }
        let chain_index = chains.len();
        let chain_segments = points
            .windows(2)
            .map(|pair| (pair[0], pair[1]))
            .collect::<Vec<_>>();
        for pair in points.windows(2) {
            segments.push(Segment2 {
                a: pair[0],
                b: pair[1],
                tag: json!({ "chain": chain_index }),
            });
        }
        chains.push(Chain {
            source,
            count: points.len() - 1,
            start: points[0],
            end: points[points.len() - 1],
            segments: chain_segments,
        });
        Ok(())
    };

    let _tc = web_time::Instant::now();
    // SEAM-IMAGE LOOPS: per-loop u-control-hulls, computed once for the
    // straddling-hole gate below (does any OTHER loop span the full closed-u
    // domain — the rectangle-with-seam outer that closes the strip walls).
    let loop_hull_u: Vec<[f64; 2]> = face
        .loops
        .iter()
        .map(|loop_record| {
            let mut hull = [f64::INFINITY, f64::NEG_INFINITY];
            for coedge in &loop_record.coedges {
                for control in &coedge.pcurve.control_points {
                    if control.w.abs() <= 1e-300 {
                        continue;
                    }
                    hull[0] = hull[0].min(control.x / control.w);
                    hull[1] = hull[1].max(control.x / control.w);
                }
            }
            hull
        })
        .collect();
    // Signed area of the face's full-u-span loop (the rectangle-with-seam
    // outer), recorded as it is processed: it defines the chart's material
    // orientation, so a HOLE is any loop winding OPPOSITE to it (the absolute
    // sign is chart-dependent — mirrored charts flip both).
    let mut full_span_loop_area: Option<f64> = None;
    // Post-rebase boundary polyline segments (incl. seam-image copies), kept
    // for the sag-refinement boundary-crossing parity guard on cut chains.
    let mut boundary_segments: Vec<(Vec2, Vec2)> = Vec::new();
    // The u and v extent of the boundary chains as the arrangement places
    // them — after the seam-hop unwrap and the band re-base, before any
    // seam-image copy — which is the period window the face's material lives
    // in. See the cut-chain period lift below.
    let mut band_hull = [[f64::INFINITY, f64::NEG_INFINITY]; 2];
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        // SEAM-HOP UNWRAP: a loop whose pcurves are drawn IN-DOMAIN but hop
        // the seam between consecutive coedges (t91's face 675: the boundary
        // jumps u=1→0 at v≈0.967 and back at v≈0.453) is incoherent in the
        // flat chart — its cut chains mint fine and then dangle against the
        // hopping boundary, cycle extraction fails, and the face drops
        // wholesale (the trial-35 zero-fragment signature, third variant).
        // Unwrap onto the covering plane with the SAME per-coedge offsets the
        // mass integrator / tessellator / seam-band classifier use
        // (`loop_seam_offsets`); a continuous loop gets all-zero offsets and
        // is untouched (helmet-class unwrapped sources, native seam-edge
        // loops, plain faces). Composes with the band re-base above: offsets
        // normalize the boundary FIRST, after which the re-base's
        // below-cutoff boundary shifts are no-ops and only its cut-chain
        // shifts still apply. Escape hatch: BREP_LOOP_UNWRAP=0.
        let closed = face.surface.closed_directions().or_refuse(KernelStage::Fragment, "closed_directions")?;
        let offsets = if (closed.0 || closed.1)
            && std::env::var("BREP_LOOP_UNWRAP").as_deref() != Ok("0")
        {
            crate::topology::loop_seam_offsets(
                &loop_record.coedges,
                closed.0,
                closed.1,
                u_domain[1] - u_domain[0],
                v_domain[1] - v_domain[0],
            ).or_refuse(KernelStage::Fragment, "csg.fragment.face_split")?
        } else {
            vec![[0.0, 0.0]; loop_record.coedges.len()]
        };
        let mut sampled: Vec<(usize, Vec<Vec2>, NurbsCurve, u64, u64)> =
            Vec::with_capacity(loop_record.coedges.len());
        for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
            let edge = edge_for(solid, coedge.edge_id)?;
            let (start_vertex, end_vertex) = if coedge.forward {
                (edge.start_vertex_id, edge.end_vertex_id)
            } else {
                (edge.end_vertex_id, edge.start_vertex_id)
            };
            let offset = offsets
                .get(coedge_index)
                .copied()
                .unwrap_or([0.0, 0.0]);
            let initial = sample_chain(&coedge.pcurve)?;
            // Adaptive samples are not uniformly spaced in parameter. Do not
            // feed them to refine_near_hints, which assumes uniform input.
            let boundary_points = if strip_resolution.is_some() {
                sag_refine_chain(&coedge.pcurve, initial, sampling_bound)?.0
            } else { refine_near_hints(&initial, &coedge.pcurve, &cut_hints)? };
            let points: Vec<Vec2> = boundary_points
                    .into_iter()
                    .map(|point| {
                        rebase_point(Vec2 {
                            x: point.x + offset[0],
                            y: point.y + offset[1],
                        })
                    })
                    .collect();
            let curve = trimmed_curve(&edge.curve, edge.t0, edge.t1)?;
            let curve = if coedge.forward {
                curve
            } else {
                curve.reversed().or_refuse(KernelStage::Fragment, "reversed")?
            };
            sampled.push((coedge_index, points, curve, start_vertex, end_vertex));
        }
        // SEAM-IMAGE for a STRADDLING HOLE loop: an inner (hole) loop whose
        // unwrapped chains poke past a closed-u domain wall exists at only ONE
        // period image in the arrangement strip, so the strip's OTHER side
        // never sees it — the seam-adjacent region there keeps a boundary
        // running straight through the hole opening (t181/00000312#p7: face
        // 519's decagon hole unwraps to u∈[−0.039,0.039]; the region left of
        // the seam used the FULL seam edge 475 while the right region split at
        // the hole crossings; the hole's far-half rim edges stranded one-use
        // and an 18mm derived chord bridged the gap). Insert a duplicate of
        // the loop's chains shifted one period toward the strip's other side,
        // so BOTH walls are split by the hole and the half-hole phantom
        // regions (rejected by the cross-frame containment) carve it out.
        // STRUCTURAL GATE (all required): u-closed face; ≥2 loops; the loop's
        // unwrapped span crosses a u-wall by > junction slack; loop width ≤
        // half the period (a compact hole, never a rim/horizon); winding
        // OPPOSITE to the full-span outer loop (hole orientation relative to
        // the chart — an outer loop copy would double-cover material); and
        // some OTHER loop's control hull spans the full u-domain (the
        // seam-carrying rectangle exists). Escape hatch:
        // BREP_SEAM_IMAGE_LOOPS=0.
        let image_shift: Option<f64> = {
            let span = u_domain[1] - u_domain[0];
            let slack = arrangement_tolerance * 50.0;
            if !closed.0
                || face.loops.len() < 2
                || span <= 0.0
                || std::env::var("BREP_SEAM_IMAGE_LOOPS").as_deref() == Ok("0")
            {
                None
            } else {
                let mut min_u = f64::INFINITY;
                let mut max_u = f64::NEG_INFINITY;
                let mut area = 0.0;
                let mut flat: Vec<Vec2> = sampled
                    .iter()
                    .flat_map(|(_, points, ..)| points.iter().copied())
                    .collect();
                if flat.len() < 3 {
                    flat.clear();
                }
                for (index, point) in flat.iter().enumerate() {
                    min_u = min_u.min(point.x);
                    max_u = max_u.max(point.x);
                    let next = flat[(index + 1) % flat.len()];
                    area += point.x * next.y - next.x * point.y;
                }
                // Full-span discrimination must use the UNWRAPPED width: a
                // straddling hole's raw control hull spans the whole domain
                // (its halves are stored at both seam sides), but its
                // unwrapped chains are compact; only the true outer rectangle
                // stays period-wide after unwrapping.
                let spans_full_domain =
                    !flat.is_empty() && max_u - min_u >= span * (1.0 - 1e-6);
                if spans_full_domain && full_span_loop_area.is_none() {
                    full_span_loop_area = Some(area);
                }
                let straddles_low = min_u < u_domain[0] - slack;
                let straddles_high = max_u > u_domain[1] + slack;
                let compact = max_u - min_u <= 0.5 * span;
                let is_hole = full_span_loop_area
                    .map(|outer| !spans_full_domain && area * outer < 0.0)
                    .unwrap_or(false);
                let has_full_span_other = loop_hull_u.iter().enumerate().any(|(other, hull)| {
                    other != loop_index
                        && hull[0] <= u_domain[0] + 1e-6 * span
                        && hull[1] >= u_domain[1] - 1e-6 * span
                });
                if compact && is_hole && has_full_span_other && (straddles_low ^ straddles_high) {
                    if std::env::var("BREP_DEBUG_FRAG")
                        .map(|value| value == face.id.to_string())
                        .unwrap_or(false)
                    {
                        eprintln!(
                            "  seam-image: face {} loop {loop_index} straddles u-wall (u=[{min_u:.6},{max_u:.6}]) — inserting {} period image",
                            face.id,
                            if straddles_low { "+1" } else { "-1" }
                        );
                    }
                    Some(if straddles_low { span } else { -span })
                } else {
                    None
                }
            }
        };
        for (coedge_index, points, curve, start_vertex, end_vertex) in sampled {
            let coedge = &loop_record.coedges[coedge_index];
            for point in &points {
                band_hull[0] = [band_hull[0][0].min(point.x), band_hull[0][1].max(point.x)];
                band_hull[1] = [band_hull[1][0].min(point.y), band_hull[1][1].max(point.y)];
            }
            boundary_segments.extend(points.windows(2).map(|pair| (pair[0], pair[1])));
            add_chain(
                points.clone(),
                ChainSource::Boundary {
                    coedge: coedge.clone(),
                    curve: curve.clone(),
                },
                format!("b:{operand}:{start_vertex}"),
                vertex_point(solid, start_vertex)?,
                format!("b:{operand}:{end_vertex}"),
                vertex_point(solid, end_vertex)?,
                true,
            )?;
            if let Some(shift) = image_shift {
                let shifted: Vec<Vec2> = points
                    .iter()
                    .map(|point| Vec2 {
                        x: point.x + shift,
                        y: point.y,
                    })
                    .collect();
                boundary_segments
                    .extend(shifted.windows(2).map(|pair| (pair[0], pair[1])));
                add_chain(
                    shifted,
                    ChainSource::Boundary {
                        coedge: coedge.clone(),
                        curve,
                    },
                    format!("b:{operand}:{start_vertex}"),
                    vertex_point(solid, start_vertex)?,
                    format!("b:{operand}:{end_vertex}"),
                    vertex_point(solid, end_vertex)?,
                    true,
                )?;
            }
        }
    }
    // CUT-CHAIN CLAMP BOUND: on a CLOSED direction the face's boundary band may
    // legitimately extend past the surface domain — a seam-straddling face is
    // arranged in UNWRAPPED coordinates (helmet Face_15's boundary chains run
    // u∈[-0.178, 0.295] on a [0,1]-domain surface). Clamping a cut pcurve to
    // the surface domain there tears it off the band: the branch-preserved
    // truncation bridge collapsed to a count=1 sliver hugging u=0 and the face
    // never split. Widen the clamp bound to domain ∪ trim-pcurve control hulls
    // (the hulls bound the boundary chains); a cut already inside the domain is
    // untouched, and OPEN directions keep the plain domain clamp. Escape hatch
    // BREP_FRAG_ENV_CLAMP=0.
    let (clamp_u, clamp_v) = {
        let mut clamp_u = u_domain;
        let mut clamp_v = v_domain;
        let (closed_u, closed_v) = face.surface.closed_directions().or_refuse(KernelStage::Fragment, "closed_directions")?;
        // NOTE (2026-09-16): this hatch's ONLY tamper guard,
        // `csg/fragment/tests.rs::seam_band_cut_chain_splits_periodic_face`, went
        // INERT when the march-window lift landed — that test now passes with this
        // hatch off, so nothing tamper-verifies this mechanism any more. Not that it
        // is untested: no other test NAMES this hatch.
        if (closed_u || closed_v) && std::env::var("BREP_FRAG_ENV_CLAMP").as_deref() != Ok("0") {
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    for point in &coedge.pcurve.control_points {
                        if point.w.abs() <= 1e-300 {
                            continue;
                        }
                        if closed_u {
                            clamp_u[0] = clamp_u[0].min(point.x / point.w);
                            clamp_u[1] = clamp_u[1].max(point.x / point.w);
                        }
                        if closed_v {
                            clamp_v[0] = clamp_v[0].min(point.y / point.w);
                            clamp_v[1] = clamp_v[1].max(point.y / point.w);
                        }
                    }
                }
            }
        }
        (clamp_u, clamp_v)
    };
    for piece_id in active_pieces {
        let piece = pieces_by_id[&piece_id];
        let pcurve = pcurve_for(piece, face_key)?.clone();
        // SAG-BOUNDED CUT-CHAIN FIDELITY (t70 split-set asynchrony class).
        // `sample_chain`'s uniform sampling is knot-driven, so the SAME
        // imprint piece gets a 16-segment polyline on one support face and a
        // 92-segment one on the other (the counts follow each face's pcurve
        // knot structure, not the curvature). A coarse chord can then sag
        // MILLIMETRE-scale away from the true pcurve and invade a trim
        // region the true curve clears — t70: piece 55's chord on face
        // 1:405 sagged ~3e-3 into a hole corner the true pcurve misses by
        // 1.3e-3, so the arrangement carved phantom micro-junctions on that
        // face only, the two owners' split sets diverged (whole Imprint
        // edge vs a 4-piece Derived chain of the same locus), and every op
        // failed with one-use edges. Refining every cut chain until its
        // chords hug the true pcurve within the arrangement's own tolerance
        // makes both owners' polylines faithful to the ONE shared curve, so
        // their junction sets agree structurally (no proximity decisions
        // anywhere). Escape hatch: BREP_CHAIN_SAG_REFINE=0.
        let (mut points, original_flags) = sag_refine_chain(
            &pcurve,
            sample_chain(&pcurve)?,
            sampling_bound,
        )?;
        for point in &mut points {
            if point.x < clamp_u[0] {
                point.x = clamp_u[0] + arrangement_tolerance * 10.0;
            } else if point.x > clamp_u[1] {
                point.x = clamp_u[1] - arrangement_tolerance * 10.0;
            }
            if point.y < clamp_v[0] {
                point.y = clamp_v[0] + arrangement_tolerance * 10.0;
            } else if point.y > clamp_v[1] {
                point.y = clamp_v[1] - arrangement_tolerance * 10.0;
            }
        }
        let snap_endpoint_to_domain = |point: Vec2| {
            let candidates = [
                (
                    (point.x - u_domain[0]).abs(),
                    Vec2 {
                        x: u_domain[0],
                        y: point.y,
                    },
                ),
                (
                    (point.x - u_domain[1]).abs(),
                    Vec2 {
                        x: u_domain[1],
                        y: point.y,
                    },
                ),
                (
                    (point.y - v_domain[0]).abs(),
                    Vec2 {
                        x: point.x,
                        y: v_domain[0],
                    },
                ),
                (
                    (point.y - v_domain[1]).abs(),
                    Vec2 {
                        x: point.x,
                        y: v_domain[1],
                    },
                ),
            ];
            let nearest = candidates
                .into_iter()
                .min_by(|first, second| first.0.total_cmp(&second.0))
                .unwrap();
            if nearest.0 <= arrangement_tolerance * 50.0 {
                nearest.1
            } else {
                point
            }
        };
        let last = points.len() - 1;
        points[0] = snap_endpoint_to_domain(points[0]);
        points[last] = snap_endpoint_to_domain(points[last]);
        for point in &mut points {
            *point = rebase_point(*point);
        }
        // CUT-CHAIN PERIOD LIFT. A section's pcurve on a closed carrier is
        // drawn on the carrier's own period, but the boundary chains above can
        // sit a period away from it: an imported face whose seam edge is not
        // at the carrier's parameter origin carries rims split AT that origin,
        // so its loop hops the branch cut between coedges and the seam-hop
        // unwrap lays it out over [seam, seam + period] (the 2026-09-14 gear:
        // cone and fillet torus with seam edges at u = 1/2, boundary chains on
        // u ∈ [0.5, 1.5], section arcs on [0, 0.5] and [0.5, 1]). The arc a
        // period off never meets the band, the section never closes across the
        // face, the face stays one fragment and is kept or dropped whole — one
        // rim and the section ring then strand one-use. Every piece the imprint
        // attaches to a face has its midpoint inside that face's trim, so the
        // chain's image whose middle lies inside the band is the one the
        // arrangement must see; a whole-period shift is exact on a periodic
        // carrier. The chain is lifted only when its middle is OUTSIDE the band
        // and exactly one period image puts it inside, so a chain already
        // coherent with its band — every native face, every re-based band —
        // is untouched. The pcurve keeps the carrier's own period (the lift is
        // an arrangement coordinate, recorded on the chain for the partial-run
        // rescue). A lifted chain that still crosses a wall of the band crosses
        // the face's seam edge unsplit, and the face refuses by name. Escape
        // hatch `BREP_CUT_PERIOD_LIFT=0`.
        let mut lift = Vec2 { x: 0.0, y: 0.0 };
        if std::env::var("BREP_CUT_PERIOD_LIFT").as_deref() != Ok("0") {
            let closed = face.surface.closed_directions().or_refuse(KernelStage::Fragment, "closed_directions")?;
            for axis in 0..2 {
                let (is_closed, domain) = if axis == 0 { (closed.0, u_domain) } else { (closed.1, v_domain) };
                let [low, high] = band_hull[axis];
                let span = domain[1] - domain[0];
                if !is_closed || !(span > 0.0) || !(low < domain[0] || high > domain[1]) {
                    continue;
                }
                let coordinate = |point: &Vec2| if axis == 0 { point.x } else { point.y };
                let middle = coordinate(&points[points.len() / 2]);
                let inside = |value: f64| value > low && value < high;
                if inside(middle) {
                    continue;
                }
                let images: Vec<f64> = [-2.0, -1.0, 1.0, 2.0]
                    .into_iter()
                    .map(|periods| periods * span)
                    .filter(|shift| inside(middle + shift))
                    .collect();
                let [shift] = images[..] else {
                    continue;
                };
                let slack = arrangement_tolerance * 50.0;
                let (reach_low, reach_high) = points.iter().map(coordinate).fold(
                    (f64::INFINITY, f64::NEG_INFINITY),
                    |(lo, hi), value| (lo.min(value), hi.max(value)),
                );
                if reach_low + shift < low - slack || reach_high + shift > high + slack {
                    let [first, second] = piece.support_faces;
                    return Err(KernelRefusal::internal(
                        KernelStage::Fragment,
                        "fragment.face_split",
                        format!(
                            "fragment_face: section piece {piece_id} of faces {}:{} x {}:{} crosses \
                             the seam wall of face {}:{}'s unwrapped band ({} in [{low:.9}, {high:.9}], \
                             the section's period image [{:.9}, {:.9}]) without a vertex there",
                            first.operand,
                            first.face_id,
                            second.operand,
                            second.face_id,
                            operand,
                            face.id,
                            if axis == 0 { "u" } else { "v" },
                            reach_low + shift,
                            reach_high + shift,
                        ),
                    ));
                }
                for point in &mut points {
                    if axis == 0 {
                        point.x += shift;
                    } else {
                        point.y += shift;
                    }
                }
                if axis == 0 {
                    lift.x = shift;
                } else {
                    lift.y = shift;
                }
                if std::env::var("BREP_DEBUG_FRAG")
                    .map(|value| value == face.id.to_string())
                    .unwrap_or(false)
                {
                    eprintln!(
                        "  period lift: face {} cut p{piece_id} {} {shift:+} into the band [{low:.6},{high:.6}]",
                        face.id,
                        if axis == 0 { "u" } else { "v" }
                    );
                }
            }
        }
        // Guard runs on the FINAL (clamped/snapped/rebased) coordinates — the
        // same frame the boundary chains were sampled into — so crossing
        // parity is measured exactly where the arrangement would see it.
        points = enforce_boundary_crossing_parity(
            points,
            &original_flags,
            &boundary_segments,
            arrangement_tolerance,
            std::env::var("BREP_DEBUG_FRAG")
                .map(|value| value == face.id.to_string())
                .unwrap_or(false),
            face.id,
        );
        let start_point = index
            .vertex_points
            .get(&piece.start_vertex_id)
            .copied()
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.face_split", "fragment_face: missing imprint start vertex"))?;
        let end_point = index
            .vertex_points
            .get(&piece.end_vertex_id)
            .copied()
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Fragment, "fragment.face_split", "fragment_face: missing imprint end vertex"))?;
        add_chain(
            points,
            ChainSource::Cut {
                piece_id,
                pcurve,
                lift,
                curve: trimmed_curve(&piece.curve, piece.t0, piece.t1)?,
                shared_ring: piece.shared_edge.filter(|_| {
                    piece
                        .curve
                        .evaluate(piece.t0)
                        .ok()
                        .zip(piece.curve.evaluate(piece.t1).ok())
                        .is_some_and(|(a, b)| a.sub(b).length() <= 1e-6)
                }),
            },
            format!("i:{}", piece.start_vertex_id),
            start_point,
            format!("i:{}", piece.end_vertex_id),
            end_point,
            false,
        )?;
    }
    drop(add_chain);

    // POLE-GAP BRIDGE (single-loop periodic cap; corpus fixture 25 cube-pierce).
    // A face closed in one parameter direction can carry a DEGENERATE pole
    // iso-line — a whole domain-edge row of one parameter collapsing to a
    // single 3D point (the dome apex v2 at v=0 here). The imported trim
    // represents the pole with a partial arc plus a degenerate self-edge that
    // stop SHORT of the opposite seam foot, so the outer boundary is OPEN in
    // the flat UV chart at the pole: the two seam feet sit a FRACTION of a
    // period apart (never a whole period), so no covering-strip period-image
    // can ever join them — this case is structurally outside the seam-image
    // loops machinery above. When a section cut then splits the cap,
    // `arrange_segments` closes only the lens region that avoids the pole and
    // DROPS the seam-and-pole-spanning outside region: the cap fragments to 1
    // not 2, its section + outer-rim edges strand one-use (union/subtract fail
    // `S=1 genus=1`), while intersect keeps the lens and succeeds. Close the
    // boundary in-chart: pair the two dangling (odd-incidence) boundary
    // endpoints that are UV images of the SAME solid vertex (structural
    // identity via `vertex_images`, never proximity clustering), VERIFY the
    // straight UV connector between them is degenerate (every sample collapses
    // to one 3D point — a genuine pole, not a real seam that would cross
    // material), and add a bridge chain that REUSES an existing degenerate
    // pole edge as its source. The reused edge is 3D-zero-length, so the
    // minted coedge is s==e and exempt from the two-use rule — no new edge is
    // introduced (the zero-regression reuse pattern), and the outside fragment
    // reproduces the original cap loop's own pole/seam structure. A
    // cleanly-closing boundary has no odd node and is byte-identical untouched;
    // a genuine (non-degenerate) seam gap fails the degeneracy check and is
    // left alone (the safe direction). Escape hatch: BREP_POLE_GAP_BRIDGE=0.
    let pole_gap_closed = face.surface.closed_directions().or_refuse(KernelStage::Fragment, "closed_directions")?;
    if (pole_gap_closed.0 || pole_gap_closed.1)
        && std::env::var("BREP_POLE_GAP_BRIDGE").as_deref() != Ok("0")
    {
        // A degenerate pole iso-line only exists on a surface closed in the
        // seam direction; a non-periodic face never reaches this block and is
        // byte-identical untouched.
        //
        // `add_chain` canonicalizes shared endpoints to a bit-identical `Vec2`,
        // so a TIGHT epsilon groups exactly the coincident nodes and never
        // over-merges two distinct nearby vertices (which would corrupt the
        // incidence parity on a small face).
        let node_tol = arrangement_tolerance * 10.0;
        // Incidence over BOUNDARY chain endpoints only — the pole gap is a
        // property of the trim boundary, and cut chains anchor to it elsewhere.
        let mut nodes: Vec<(Vec2, usize)> = Vec::new();
        for chain in &chains {
            if !matches!(chain.source, ChainSource::Boundary { .. }) {
                continue;
            }
            for point in [chain.start, chain.end] {
                if let Some(entry) = nodes.iter_mut().find(|(q, _)| q.sub(point).length() <= node_tol)
                {
                    entry.1 += 1;
                } else {
                    nodes.push((point, 1));
                }
            }
        }
        // Odd incidence => the boundary is UV-open at that endpoint.
        let odd: Vec<Vec2> = nodes
            .iter()
            .filter(|(_, count)| count % 2 == 1)
            .map(|(point, _)| *point)
            .collect();
        // Each odd node's owning solid vertex: the canonical image lists in
        // `vertex_images` are keyed "b:{operand}:{vertex_id}".
        let vertex_of = |point: Vec2| -> Option<u64> {
            vertex_images.iter().find_map(|(key, images)| {
                images
                    .iter()
                    .any(|image| image.sub(point).length() <= node_tol)
                    .then(|| key.rsplit(':').next().and_then(|id| id.parse::<u64>().ok()))
                    .flatten()
            })
        };
        let mut by_vertex: HashMap<u64, Vec<Vec2>> = HashMap::default();
        for point in &odd {
            if let Some(vertex_id) = vertex_of(*point) {
                by_vertex.entry(vertex_id).or_default().push(*point);
            }
        }
        // Forward-evaluate only (the projector is a poisoned oracle; evaluate
        // is exact). A degenerate 3D curve collapses to a point over its span.
        let curve_degenerate = |curve: &NurbsCurve| -> bool {
            let Ok(domain) = curve.domain() else {
                return false;
            };
            let Ok(base) = curve.evaluate(domain[0]) else {
                return false;
            };
            [0.5, 1.0].iter().all(|fraction| {
                curve
                    .evaluate(domain[0] + (domain[1] - domain[0]) * fraction)
                    .map(|point| point.sub(base).length() <= 1e-5)
                    .unwrap_or(false)
            })
        };
        for (vertex_id, images) in by_vertex {
            // A pole slit unclosed in exactly one place has EXACTLY two dangling
            // images of its apex vertex; anything else is not this class.
            if images.len() != 2 {
                continue;
            }
            let (a, b) = (images[0], images[1]);
            // The straight UV connector must be a degenerate iso-line: every
            // sample collapses to one 3D point. This is the gate that keeps a
            // real seam (whose connector crosses material) from being bridged.
            let mut base: Option<Vec3> = None;
            let mut degenerate = true;
            for step in 0..=8 {
                let fraction = step as f64 / 8.0;
                let uv = a.add(b.sub(a).scale(fraction));
                match face.surface.evaluate(uv.x, uv.y) {
                    Ok(point) => match base {
                        None => base = Some(point),
                        Some(reference) => {
                            if point.sub(reference).length() > 1e-5 {
                                degenerate = false;
                                break;
                            }
                        }
                    },
                    Err(_) => {
                        degenerate = false;
                        break;
                    }
                }
            }
            if !degenerate {
                continue;
            }
            // Reuse an existing degenerate pole edge's source (its 3D curve is a
            // point, so the minted coedge is s==e and validation-exempt).
            let Some(source) = chains.iter().find_map(|chain| match &chain.source {
                ChainSource::Boundary { coedge, curve }
                    if chain.start.sub(chain.end).length() <= node_tol
                        && chain.count <= 1
                        && (chain.start.sub(a).length() <= node_tol
                            || chain.start.sub(b).length() <= node_tol)
                        && curve_degenerate(curve) =>
                {
                    Some(ChainSource::Boundary {
                        coedge: coedge.clone(),
                        curve: curve.clone(),
                    })
                }
                _ => None,
            }) else {
                continue;
            };
            let chain_index = chains.len();
            segments.push(Segment2 {
                a,
                b,
                tag: json!({ "chain": chain_index }),
            });
            chains.push(Chain {
                source,
                count: 1,
                start: a,
                end: b,
                segments: vec![(a, b)],
            });
            boundary_segments.push((a, b));
            if std::env::var("BREP_DEBUG_FRAG")
                .map(|value| value == face.id.to_string())
                .unwrap_or(false)
            {
                eprintln!(
                    "  pole-gap bridge: face {} vertex {vertex_id} joins \
                     ({:.6},{:.6})->({:.6},{:.6}) along a degenerate iso-line",
                    face.id, a.x, a.y, b.x, b.y
                );
            }
        }
    }

    // A cut chain that coincides with a boundary chain cannot split the
    // interior — the face's own edge already bounds it there. Worse, the
    // duplicate overlapping segments corrupt arrangement cycle extraction
    // and silently swallow every region that touches the overlap (seen when
    // endpoint polishing snaps a face/face intersection line onto a
    // near-coincident boundary edge of the same face). Drop such cuts
    // before arranging.
    let coincidence_tolerance = arrangement_tolerance * 50.0;
    let point_segment_distance = |point: Vec2, a: Vec2, b: Vec2| -> f64 {
        let segment = b.sub(a);
        let length_squared = segment.dot(segment);
        if length_squared <= 1e-30 {
            return point.sub(a).length();
        }
        let parameter = (point.sub(a).dot(segment) / length_squared).clamp(0.0, 1.0);
        point.sub(a.add(segment.scale(parameter))).length()
    };
    let near_chain = |point: Vec2, chain: &Chain, tolerance: f64| {
        chain
            .segments
            .iter()
            .any(|(a, b)| point_segment_distance(point, *a, *b) <= tolerance)
    };
    // A cut coincides with a boundary only if its INTERIOR follows the same
    // path, not merely its endpoints. A single-segment straight cut whose two
    // endpoints happen to land on a CURVED boundary edge's endpoints (a
    // cap∩cap intersection chord spanning a curved rim arc — the missing
    // cone-cap split behind the curved-primitive open-edge booleans) shares
    // only its endpoints and cuts clean across the interior; dropping it as a
    // duplicate loses the region it carves. So also require the segment
    // MIDPOINTS to lie near the boundary. The midpoint test runs at a looser
    // threshold than the endpoint match: a genuinely-coincident curved cut's
    // polyline midpoints carry their own chord sag (order 1× tolerance), which
    // must still count as near, while a chord spanning a curved arc departs by
    // the arc's sagitta (orders of magnitude larger) and is correctly kept. On
    // the box path both edges are straight, so two shared endpoints force the
    // midpoint onto the boundary and the drop is unchanged.
    let midpoint_tolerance = coincidence_tolerance * 8.0;
    // CORNER-CHORD VETO. The endpoint match above admits a cut whose endpoint
    // sits up to `coincidence_tolerance` (50x arr_tol) from the boundary
    // chain's endpoint. When a section threads through a MODEL CORNER where two
    // boundary edges (legs) meet at a shared vertex, the section arm on the
    // sliver face between them is the HYPOTENUSE of a tiny right triangle whose
    // short leg can be well under that slack (t634: the torus crosses a 3-face
    // corner of 00000327; on the sliver face the arm's start is 31x arr_tol
    // from the matched leg's start, and the hypotenuse midpoint is ~15x arr_tol
    // off the leg — both under the 50x/400x tolerances). The old test dropped
    // that arm as a "duplicate" of one leg, so the tiny corner region was never
    // carved on the sliver face while the mate operand kept the arm -> the
    // shared section edge went one-use (invalid genus). A genuine corner chord
    // is distinguishable STRUCTURALLY, not by tolerance: its endpoint coincides
    // (at NODE scale = the arrangement's own find_node snap) with a real,
    // DISTINCT boundary vertex — proof it terminates at a 0-cell of its own,
    // spanning two legs, rather than retracing this one edge. A true retrace
    // (box duplicate, cap-rim chord) has endpoints AT the matched chain's ends
    // (gap ~0), so the veto never triggers for it. Hatch restores the old drop.
    let corner_veto_enabled = std::env::var("BREP_DUP_DROP_CORNER_VETO")
        .map(|value| value != "0")
        .unwrap_or(true);
    let node_tolerance = arrangement_tolerance;
    let terminates_at_distinct_boundary_vertex = |cut_end: Vec2, matched_end: Vec2| -> bool {
        cut_end.sub(matched_end).length() > node_tolerance
            && chains.iter().any(|other| {
                matches!(other.source, ChainSource::Boundary { .. })
                    && (other.start.sub(cut_end).length() <= node_tolerance
                        || other.end.sub(cut_end).length() <= node_tolerance)
            })
    };
    let debug_drop = std::env::var("BREP_DEBUG_FRAG")
        .map(|value| value == face.id.to_string())
        .unwrap_or(false);
    // MICRO-BOUNDARY SKIP (t316). A boundary chain SHORTER than the coincidence
    // tolerance cannot serve as a duplication reference: its two endpoints are
    // indistinguishable at this tolerance, so the orientation-agnostic endpoint
    // match collapses and a cut that merely touches ONE of its ends spuriously
    // "matches" BOTH. That wrongly drops a junction-bridge micro-cut spanning a
    // weld-scale vertex cluster (t316: cut p38, the ~0.004mm bridge from a
    // section terminus C to the plate-rim node C', gets dropped against e99 — a
    // ~0.008mm rim segment shorter than the 0.07mm coincidence tolerance —
    // stranding the whole b-poke bottom-cap sliver one-use). The genuine target
    // (the tangent-wall cap section, coincident with a full-length box edge ≫
    // tolerance) is untouched. Independent of the corner-chord veto below: this
    // guard disqualifies a sub-tolerance BOUNDARY as a reference, that one spares
    // a corner-spanning CUT. Hatch `BREP_DUP_CUT_MICRO_BOUNDARY=0` restores the
    // old, length-agnostic reference test.
    let micro_boundary_skip = std::env::var("BREP_DUP_CUT_MICRO_BOUNDARY").as_deref() != Ok("0");
    let boundary_extent = |boundary: &Chain| -> f64 {
        boundary
            .segments
            .iter()
            .map(|(a, b)| b.sub(*a).length())
            .sum()
    };
    // POLE-COLLAPSED ENDS. A pole is a whole row of the chart at one 3D
    // point, so two chain ends on its degenerate boundary chain are one point
    // however far apart their u reads, and near the pole a chart distance says
    // nothing about the 3D one. The revolve_pole union's G2 end cap (a 93 deg
    // sector with its apex on the axis) is cut by the wedge's two side planes
    // along its own radius edges: the sections are those edges' sub-arcs to
    // 1e-15, ending 3.2e-6 short of the apex. Their pcurves follow u = 1 and
    // u = 0 and then swing to one u = 0.443 at the pole; evaluated on the cap
    // a chain stands 7.4e-4 off its radius at v = 0.99976. The endpoint test
    // read a gap of 0.557 and kept both cuts, and the arrangement carved two
    // slivers, each bounded by a radius edge and its own weld copy. One was
    // kept in the union and the coplanar merge refused its genus; the
    // intersection refused with one-use edges. So when a chain end and a
    // boundary end both lie on one degenerate boundary chain they coincide,
    // and the cut is held to the boundary in 3D, section curve against edge
    // curve, within the coincidence tolerance carried along the boundary by
    // its own stretch (3D length over chart length). Escape hatch
    // `BREP_DUP_DROP_POLE=0`.
    let pole_ends = std::env::var("BREP_DUP_DROP_POLE").as_deref() != Ok("0");
    let degenerate_chains: Vec<usize> = chains
        .iter()
        .enumerate()
        .filter(|(_, chain)| match &chain.source {
            ChainSource::Boundary { coedge, .. } => solid
                .edges
                .iter()
                .any(|edge| edge.id == coedge.edge_id && edge.degenerate),
            ChainSource::Cut { .. } => false,
        })
        .map(|(index, _)| index)
        .collect();
    let on_one_pole = |a: Vec2, b: Vec2| -> bool {
        degenerate_chains.iter().any(|&pole| {
            near_chain(a, &chains[pole], coincidence_tolerance)
                && near_chain(b, &chains[pole], coincidence_tolerance)
        })
    };
    let coincident_in_space = |cut: &Chain, boundary: &Chain, weld_scale: bool| -> bool {
        let ChainSource::Boundary { curve, coedge } = &boundary.source else {
            return false;
        };
        let ChainSource::Cut { curve: section, piece_id, .. } = &cut.source else {
            return false;
        };
        let Ok([start, end]) = section.domain() else {
            return false;
        };
        let mut chart_length = 0.0;
        let mut space_length = 0.0;
        for (a, b) in &boundary.segments {
            let (Ok(pa), Ok(pb)) = (face.surface.evaluate(a.x, a.y), face.surface.evaluate(b.x, b.y)) else {
                return false;
            };
            chart_length += b.sub(*a).length();
            space_length += pb.sub(pa).length();
        }
        if chart_length <= 0.0 || space_length <= 0.0 {
            return false;
        }
        let copy_boundary = index.pieces_by_id.get(piece_id).is_some_and(|piece| {
            boundary_has_cosurface_partner(solid, face_key, coedge.edge_id, piece, index)
        });
        let bound = if weld_scale {
            if copy_boundary { crate::tolerance::WELD_FLOOR } else { 1e-7 }
        } else {
            coincidence_tolerance * space_length / chart_length
        };
        let stations = cut.segments.len().max(16);
        (0..=stations).all(|index| {
            section
                .evaluate(start + (end - start) * index as f64 / stations as f64)
                .ok()
                .and_then(|at| crate::project_point_to_curve(curve, at).ok())
                .is_some_and(|projection| projection.distance <= bound)
        })
    };
    let mut dropped_chains = HashSet::default();
    for (index, chain) in chains.iter().enumerate() {
        if !matches!(chain.source, ChainSource::Cut { .. }) {
            continue;
        }
        let mut matched_boundary: Option<usize> = None;
        for (boundary_index, boundary) in chains.iter().enumerate() {
            if !matches!(boundary.source, ChainSource::Boundary { .. }) {
                continue;
            }
            // t316: a sub-tolerance micro-boundary cannot be a duplication
            // reference — its endpoints collapse at this tolerance, so a cut that
            // merely touches one end spuriously matches both. Skip it as a
            // candidate (never selectable as `matched_boundary`).
            if micro_boundary_skip && boundary_extent(boundary) <= coincidence_tolerance {
                continue;
            }
            let forward = boundary.start.sub(chain.start).length() <= coincidence_tolerance
                && boundary.end.sub(chain.end).length() <= coincidence_tolerance;
            let reversed = boundary.start.sub(chain.end).length() <= coincidence_tolerance
                && boundary.end.sub(chain.start).length() <= coincidence_tolerance;
            let same_end = |a: Vec2, b: Vec2| a.sub(b).length() <= coincidence_tolerance || on_one_pole(a, b);
            let (forward, reversed, at_pole) = if forward || reversed || !pole_ends || degenerate_chains.is_empty() {
                (forward, reversed, false)
            } else {
                let forward = same_end(boundary.start, chain.start) && same_end(boundary.end, chain.end);
                let reversed = same_end(boundary.start, chain.end) && same_end(boundary.end, chain.start);
                (forward, reversed, true)
            };
            if !(forward || reversed) {
                continue;
            }
            let interior_coincident = if at_pole {
                coincident_in_space(chain, boundary, false)
            } else {
                chain.segments.iter().all(|(a, b)| {
                    near_chain(*a, boundary, coincidence_tolerance)
                        && near_chain(*b, boundary, coincidence_tolerance)
                        && near_chain(a.add(*b).scale(0.5), boundary, midpoint_tolerance)
                })
            };
            if !interior_coincident {
                continue;
            }
            // UV proximity only nominates a duplicate. A real cap lens can
            // be narrower than that chart-space search band while its two
            // arcs are distinct in 3D. Discard the cut only when the section
            // follows the boundary within model identity, not the repair band.
            // Endpoint anchoring can turn a very short cut and boundary into
            // the SAME chart segment even when their pre-anchor 3D curves
            // differ. Such identical constraints cannot enclose a region;
            // keeping both would corrupt arrangement cycle extraction.
            let identical_segment = chain.segments.len() == 1
                && boundary.segments.len() == 1
                && ((chain.start.sub(boundary.start).length() <= node_tolerance
                    && chain.end.sub(boundary.end).length() <= node_tolerance)
                    || (chain.start.sub(boundary.end).length() <= node_tolerance
                        && chain.end.sub(boundary.start).length() <= node_tolerance));
            if !identical_segment
                && std::env::var("BREP_DUP_CUT_GEOMETRY").as_deref() != Ok("0")
                && !coincident_in_space(chain, boundary, true)
            {
                continue;
            }
            if corner_veto_enabled {
                // Pair each cut endpoint with the boundary endpoint it matched.
                let (bnd_for_start, bnd_for_end) = if forward {
                    (boundary.start, boundary.end)
                } else {
                    (boundary.end, boundary.start)
                };
                if terminates_at_distinct_boundary_vertex(chain.start, bnd_for_start)
                    || terminates_at_distinct_boundary_vertex(chain.end, bnd_for_end)
                {
                    continue;
                }
            }
            matched_boundary = Some(boundary_index);
            break;
        }
        if let Some(boundary_index) = matched_boundary {
            if debug_drop {
                let boundary = &chains[boundary_index];
                eprintln!(
                    "  drop cut chain {} as duplicate of boundary chain {} (endpoint gaps {:.3e}/{:.3e})",
                    index,
                    boundary_index,
                    boundary.start.sub(chain.start).length(),
                    boundary.end.sub(chain.end).length(),
                );
            }
            dropped_chains.insert(index);
        }
    }
    if !dropped_chains.is_empty() {
        segments.retain(|segment| {
            segment.tag["chain"]
                .as_u64()
                .map(|chain| !dropped_chains.contains(&(chain as usize)))
                .unwrap_or(true)
        });
    }

    let debug = std::env::var("BREP_DEBUG_FRAG")
        .map(|value| value == face.id.to_string())
        .unwrap_or(false);
    if debug {
        eprintln!(
            "fragment_face {}: domain u={:?} v={:?} arr_tol={:.3e} chains={} segments={}",
            face.id,
            u_domain,
            v_domain,
            arrangement_tolerance,
            chains.len(),
            segments.len()
        );
        for (index, chain) in chains.iter().enumerate() {
            let kind = match &chain.source {
                ChainSource::Boundary { coedge, .. } => format!("boundary e{}", coedge.edge_id),
                ChainSource::Cut { piece_id, .. } => format!("cut p{piece_id}"),
            };
            eprintln!(
                "  chain {index} {kind} count={} start=({:.9},{:.9}) end=({:.9},{:.9})",
                chain.count, chain.start.x, chain.start.y, chain.end.x, chain.end.y
            );
        }
    }
    // TRIPLE-POINT CUT-CHAIN BRIDGE (imported-geometry corner rescue).
    // Two SSI cut curves that meet at a triple point (two faces of one operand
    // both crossing a single face of the other) can be marched to endpoints that
    // DISAGREE by more than arr_tol when the operand's shared edge carries an
    // imported edge<->vertex gap — the imprint then mints TWO vertices for the
    // one corner, so the cut chain is BROKEN there. A broken cut cannot separate
    // the domain: the arrangement returns one region covering the whole face, the
    // face is classified by a single test point and wrongly dropped, and its
    // neighbours' shared SSI edges are stranded one-use (non-integral genus).
    // Reconnect a pair of cut-chain endpoints that BOTH dangle in the interior
    // (each neither on a domain boundary nor meeting any other chain endpoint)
    // and whose chains are EACH anchored at their other end, by snapping them to
    // their midpoint. Gated on the dangling-anchored-pair precondition, which
    // only a broken cut produces, so a face whose cuts already connect (every
    // currently-succeeding fragmentation) is never touched.
    if chains.iter().any(|c| matches!(c.source, ChainSource::Cut { .. })) {
        let endpoints: Vec<(usize, bool, Vec2)> = chains
            .iter()
            .enumerate()
            .flat_map(|(i, c)| [(i, false, c.start), (i, true, c.end)])
            .collect();
        let anchored = |chain: usize, point: Vec2| -> bool {
            boundary_side(point).is_some()
                || endpoints.iter().any(|(other_chain, _, other_point)| {
                    *other_chain != chain
                        && other_point.sub(point).length() <= junction_radius
                })
        };
        // Interior cut endpoints with an anchored far end are the loose ends of
        // an almost-through cut. Only these are bridge candidates.
        //
        // OVERSHOOT-STUB EXCLUSION (offset-shell triple junctions). A pairwise
        // section between offset carriers is clipped to the offset IMAGE of the
        // source trims, which can extend PAST the true junction with a third
        // carrier's section instead of stopping at it (O.S14: the plane×cylinder
        // line runs to the source cone∩cylinder circle image at y=100/13 while
        // the plane×cone section crosses it ~0.3 earlier at the real offset
        // junction). Such a chain is already CONNECTED into the cut graph:
        // arrange_segments splits both chains at the crossing node and its
        // degree-1 pruning drops the overshoot stub beyond it, and build_loop
        // emits the surviving partial runs as Derived sub-curves. The stub's
        // dangling endpoint is therefore NOT a broken-junction loose end —
        // bridging it would drag a chain end to a midpoint off the section
        // (a wrong patch) and the 3D gate would refuse a face the arrangement
        // handles exactly. Exclude an endpoint from bridge candidacy when its
        // chain intersects another CUT chain at a point interior to the chain
        // (beyond the junction radius of both its endpoints — an intersection
        // AT the far anchor is the ordinary shared-junction meeting and must
        // keep the endpoint eligible, which preserves the imported-geometry
        // broken-cut rescue: those chains stop SHORT of each other and have no
        // crossing at all).
        let cut_chain_has_interior_crossing = |i: usize| -> bool {
            let chain = &chains[i];
            chains.iter().enumerate().any(|(j, other)| {
                // A boundary-duplicate chain was dropped from the arrangement's
                // segment set; a crossing with it would not materialize a node.
                if j == i
                    || dropped_chains.contains(&j)
                    || !matches!(other.source, ChainSource::Cut { .. })
                {
                    return false;
                }
                chain.segments.iter().any(|&(a0, a1)| {
                    other.segments.iter().any(|&(b0, b1)| {
                        let Some([t, _]) = crate::arrangement::segment_intersection(
                            a0,
                            a1,
                            b0,
                            b1,
                            arrangement_tolerance,
                        ) else {
                            return false;
                        };
                        let crossing = a0.add(a1.sub(a0).scale(t));
                        crossing.sub(chain.start).length() > junction_radius
                            && crossing.sub(chain.end).length() > junction_radius
                    })
                })
            })
        };
        let mut loose: Vec<(usize, bool, Vec2)> = Vec::new();
        for (i, c) in chains.iter().enumerate() {
            if !matches!(c.source, ChainSource::Cut { .. }) {
                continue;
            }
            for (is_end, this, other) in [(false, c.start, c.end), (true, c.end, c.start)] {
                if !anchored(i, this) && anchored(i, other) {
                    if cut_chain_has_interior_crossing(i) {
                        if debug {
                            eprintln!(
                                "  cut chain {i} loose {} excluded from junction bridging — \
                                 the chain crosses another cut chain mid-run; arrangement \
                                 pruning owns the overshoot stub",
                                if is_end { "end" } else { "start" },
                            );
                        }
                        continue;
                    }
                    loose.push((i, is_end, this));
                }
            }
        }
        // A triple-point disagreement on imported geometry runs a few 1e-4 (the
        // edge<->vertex gap), well under a percent of the face diagonal; bound the
        // bridge span so two unrelated loose cuts across a face are never joined.
        let bridge_band = (diagonal * 0.02).max(junction_radius * 4.0);
        let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
        for a in 0..loose.len() {
            for b in (a + 1)..loose.len() {
                if loose[a].0 == loose[b].0 {
                    continue; // never bridge a chain to itself
                }
                let gap = loose[a].2.sub(loose[b].2).length();
                if gap <= bridge_band {
                    pairs.push((gap, a, b));
                }
            }
        }
        pairs.sort_by(|x, y| x.0.total_cmp(&y.0));
        // 3D-DISTANCE CORROBORATION (case 08:10-genus, secondary defect): the
        // 2% UV band is metric-blind — on a big-carrier face (full-sphere
        // domain [0,1]² spanning hundreds of mm) it silently bridged loose
        // ends 8-14 mm apart in 3D, manufacturing UV-closed but 3D-OPEN loops
        // ("loop open between coedges" at assembly). A legitimate bridge heals
        // a triple-point disagreement (imported edge<->vertex gap, a small
        // fraction of the face scale); an impossible one spans a real missing
        // piece. Corroborate every candidate pair in 3D: evaluate the surface
        // at both (wrapped-back) loose ends and REFUSE — loudly, as a
        // missing-piece error — when their 3D gap exceeds 1% of the face's 3D
        // diameter (estimated over a coarse full-domain grid; on a small trim
        // of a big carrier this overstates the diameter, i.e. errs LENIENT —
        // the safe direction). Any probe evaluation failure falls back to the
        // old silent bridging (a diagnostic must not fail the operation).
        // Escape hatch: BREP_BRIDGE_3D_GATE=0 restores unconditional bridging.
        let bridge_3d_gate = std::env::var("BREP_BRIDGE_3D_GATE").as_deref() != Ok("0");
        let face_diameter3 = if bridge_3d_gate && !pairs.is_empty() {
            let mut samples: Vec<Vec3> = Vec::new();
            'grid: for i in 0..3 {
                for j in 0..3 {
                    let u = u_domain[0] + (u_domain[1] - u_domain[0]) * i as f64 / 2.0;
                    let v = v_domain[0] + (v_domain[1] - v_domain[0]) * j as f64 / 2.0;
                    match face.surface.evaluate(u, v) {
                        Ok(point) => samples.push(point),
                        Err(_) => {
                            samples.clear();
                            break 'grid;
                        }
                    }
                }
            }
            let mut diameter = 0.0f64;
            for (index, a) in samples.iter().enumerate() {
                for b in samples.iter().skip(index + 1) {
                    diameter = diameter.max(a.sub(*b).length());
                }
            }
            (diameter > 0.0).then_some(diameter)
        } else {
            None
        };
        let mut consumed = vec![false; loose.len()];
        let mut bridged = 0usize;
        for (gap, a, b) in pairs {
            if consumed[a] || consumed[b] {
                continue;
            }
            consumed[a] = true;
            consumed[b] = true;
            let (chain_a, _, point_a) = loose[a];
            let (chain_b, _, point_b) = loose[b];
            if let Some(diameter) = face_diameter3 {
                let wrapped_a = wrap_back(point_a);
                let wrapped_b = wrap_back(point_b);
                if let (Ok(position_a), Ok(position_b)) = (
                    face.surface.evaluate(wrapped_a.x, wrapped_a.y),
                    face.surface.evaluate(wrapped_b.x, wrapped_b.y),
                ) {
                    let gap3 = position_a.sub(position_b).length();
                    let limit = 0.01 * diameter;
                    if gap3 > limit {
                        return Err(KernelRefusal::internal(KernelStage::Fragment, "fragment.face_split", format!(
                            "fragment_face {}: cut junction bridge refused — 3D gap \
                             {gap3:.4} exceeds {limit:.4} (1% of face diameter \
                             {diameter:.4}) across uv gap {gap:.6}; a section piece \
                             is missing between chains {chain_a} and {chain_b}",
                            face.id
                        )));
                    }
                }
            }
            let midpoint = point_a.add(point_b).scale(0.5);
            for (chain, old) in [(chain_a, point_a), (chain_b, point_b)] {
                if chains[chain].start.sub(old).length() <= 1e-15 {
                    chains[chain].start = midpoint;
                }
                if chains[chain].end.sub(old).length() <= 1e-15 {
                    chains[chain].end = midpoint;
                }
                for seg in &mut chains[chain].segments {
                    if seg.0.sub(old).length() <= 1e-15 {
                        seg.0 = midpoint;
                    }
                    if seg.1.sub(old).length() <= 1e-15 {
                        seg.1 = midpoint;
                    }
                }
                for segment in segments.iter_mut() {
                    if segment.tag["chain"].as_u64() != Some(chain as u64) {
                        continue;
                    }
                    if segment.a.sub(old).length() <= 1e-15 {
                        segment.a = midpoint;
                    }
                    if segment.b.sub(old).length() <= 1e-15 {
                        segment.b = midpoint;
                    }
                }
            }
            bridged += 1;
        }
        if debug && bridged > 0 {
            eprintln!("  bridged {bridged} broken cut junction(s)");
        }
    }
    // GRAZE-BAND COMMON-BLOCK — near-tangent hook absorption (member t548,
    // fixture 28). fragment_face runs on the ALREADY edge-split solid, so
    // every imprint-minted section×boundary crossing is already a chain endpoint
    // by construction. A near-tangent SSI section ending at a shared junction
    // vertex can HOOK in its last span: its sampled pcurve overshoots and crosses
    // the boundary edge a graze before reaching the shared endpoint (the crossing
    // add_edge_split already REFUSED as sub-weld, so the imprint minted only the
    // shared vertex). arrange_segments, seeing the linearised hook, re-invents that
    // crossing as a distinct INTERIOR point Q, splits the boundary there, and
    // spawns a sub-band micro-stub that strands one-use at assembly (t548's
    // A→B→C needle). Absorb it deterministically: when a Cut chain and a Boundary
    // chain SHARE an endpoint node P, and the cut's polyline crosses that boundary
    // at an interior point Q within a MEASURED band of P (3D via surface.evaluate;
    // ceiling = the residual-merge sep_cap family, never grown, never the poisoned
    // projector), CLIP the cut's hooked tail — drop the samples between the
    // crossing and P and run straight into P. The cut's SOURCE curve is untouched,
    // so the complete-run path in build_loop still emits the full section edge to
    // P; only the spurious arrangement self-crossing is removed. This is the
    // arrangement sibling of BREP_SAG_CROSSING_GUARD ("sag refinement may REMOVE a
    // crossing but never ADD one"): the arrangement may not invent a boundary
    // crossing the imprint did not mint. The structural gate (shared endpoint +
    // strictly-interior sub-band crossing) keeps it off legitimate transversals —
    // an interior crossing BEYOND the band bails the chain untouched. Generalising
    // that gate to Cut×Cut pairs over a measured junction blob was tried and
    // falsified as a structural no-op: a crossing in the junction-ADJACENT first
    // segment leaves keep_lo=1 reconnecting an identical polyline, and two arms
    // that DIVERGE from a shared junction cannot be uncrossed by clipping — this
    // absorbs a multi-segment overshoot-and-return hook only. Escape hatch
    // BREP_GRAZE_HOOK_CLIP=0.
    if std::env::var("BREP_GRAZE_HOOK_CLIP").as_deref() != Ok("0") {
        let raw_extent = if solid.vertices.is_empty() {
            0.0
        } else {
            let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for v in &solid.vertices {
                lo.x = lo.x.min(v.point.x);
                lo.y = lo.y.min(v.point.y);
                lo.z = lo.z.min(v.point.z);
                hi.x = hi.x.max(v.point.x);
                hi.y = hi.y.max(v.point.y);
                hi.z = hi.z.max(v.point.z);
            }
            hi.sub(lo).length()
        };
        // Mirror imprint::residual_merge_bands' sep_cap (the measured junction-radius
        // family, 1.5e-3·extent, floored by the achieved weld band) — the largest
        // near-tangent hook a shared-junction section can plausibly span.
        let band = 2.0 * 1e-5 * raw_extent.max(1.0);
        let sep_cap = (1.5e-3 * raw_extent).max(band);
        let graze_debug = debug || std::env::var("BREP_DEBUG_GRAZE").as_deref() == Ok("1");

        let mut plans: Vec<(usize, Vec<(Vec2, Vec2)>, f64)> = Vec::new();
        for ci in 0..chains.len() {
            if !matches!(chains[ci].source, ChainSource::Cut { .. }) {
                continue;
            }
            // A chain already dropped as a boundary duplicate has had its flat
            // `segments` removed; never re-materialise it below.
            if dropped_chains.contains(&ci) {
                continue;
            }
            if chains[ci].segments.is_empty() {
                continue;
            }
            let mut lo_cut: Option<usize> = None; // clip a start hook after this seg
            let mut hi_cut: Option<usize> = None; // clip an end hook from this seg on
            let mut measured = 0.0f64;
            let mut bail = false;
            for &at_end in &[false, true] {
                let p = if at_end {
                    chains[ci].end
                } else {
                    chains[ci].start
                };
                let Ok(p3) = face.surface.evaluate(p.x, p.y) else {
                    continue;
                };
                for bi in 0..chains.len() {
                    if bi == ci || !matches!(chains[bi].source, ChainSource::Boundary { .. }) {
                        continue;
                    }
                    let shares = chains[bi].start.sub(p).length() <= node_tolerance
                        || chains[bi].end.sub(p).length() <= node_tolerance;
                    if !shares {
                        continue;
                    }
                    for si in 0..chains[ci].segments.len() {
                        let (sa, sb) = chains[ci].segments[si];
                        for &(ba, bb) in &chains[bi].segments {
                            let Some([t1, t2]) = crate::arrangement::segment_intersection(
                                sa,
                                sb,
                                ba,
                                bb,
                                arrangement_tolerance,
                            ) else {
                                continue;
                            };
                            // Strictly interior on BOTH segments: not the shared-node
                            // touch (t≈0/1) and not a boundary-endpoint (imprint-minted)
                            // crossing (t2≈0/1) — an arrangement-invented self-crossing.
                            if !(t1 > 1e-3 && t1 < 1.0 - 1e-3 && t2 > 1e-3 && t2 < 1.0 - 1e-3) {
                                continue;
                            }
                            let q = Vec2 {
                                x: sa.x + (sb.x - sa.x) * t1,
                                y: sa.y + (sb.y - sa.y) * t1,
                            };
                            let Ok(q3) = face.surface.evaluate(q.x, q.y) else {
                                continue;
                            };
                            let dist = q3.sub(p3).length();
                            if dist > sep_cap {
                                // A genuine transversal far from the shared vertex —
                                // preserve current behaviour, touch nothing.
                                bail = true;
                                break;
                            }
                            measured = measured.max(dist);
                            if at_end {
                                hi_cut = Some(hi_cut.map_or(si, |m| m.min(si)));
                            } else {
                                lo_cut = Some(lo_cut.map_or(si, |m| m.max(si)));
                            }
                        }
                        if bail {
                            break;
                        }
                    }
                    if bail {
                        break;
                    }
                }
                if bail {
                    break;
                }
            }
            if bail || (lo_cut.is_none() && hi_cut.is_none()) {
                continue;
            }
            let segs = &chains[ci].segments;
            let keep_lo = lo_cut.map(|m| m + 1).unwrap_or(0);
            let keep_hi = hi_cut.unwrap_or(segs.len());
            if keep_lo >= keep_hi {
                continue;
            }
            let mut points: Vec<Vec2> = Vec::with_capacity(keep_hi - keep_lo + 3);
            if lo_cut.is_some() {
                points.push(chains[ci].start);
            }
            points.push(segs[keep_lo].0);
            for si in keep_lo..keep_hi {
                points.push(segs[si].1);
            }
            if hi_cut.is_some() {
                points.push(chains[ci].end);
            }
            let new_segments: Vec<(Vec2, Vec2)> = points
                .windows(2)
                .filter(|w| w[0].sub(w[1]).length() > 0.0)
                .map(|w| (w[0], w[1]))
                .collect();
            if new_segments.is_empty() {
                continue;
            }
            if graze_debug {
                eprintln!(
                    "graze-hook-clip: face {} cut chain {} measured_band={:.3e} sep_cap={:.3e} \
                     segs {} -> {} (lo_cut={:?} hi_cut={:?})",
                    face.id,
                    ci,
                    measured,
                    sep_cap,
                    chains[ci].segments.len(),
                    new_segments.len(),
                    lo_cut,
                    hi_cut
                );
            }
            plans.push((ci, new_segments, measured));
        }
        for (ci, new_segments, _) in plans {
            chains[ci].count = new_segments.len();
            chains[ci].segments = new_segments.clone();
            segments.retain(|s| s.tag["chain"].as_u64() != Some(ci as u64));
            for (a, b) in new_segments {
                segments.push(Segment2 {
                    a,
                    b,
                    tag: json!({ "chain": ci }),
                });
            }
        }
    }
    let regions = arrange_segments(&segments, arrangement_tolerance).or_refuse(KernelStage::Fragment, "arrange_segments")?;
    // Orient every rebuilt loop the way the SOURCE face's own loops are
    // oriented, rather than assuming `same_sense` predicts it. The two agree on
    // every face whose chart is right-handed against its normal, so this is a
    // no-op there; it differs only on a face the old assumption got wrong, and
    // getting it wrong reverses that face's whole fragment. Falls back to the
    // old rule whenever the source winding cannot be read (see the helper).
    // Escape hatch for A/B against the old behaviour.
    let source_winding = if std::env::var("BREP_FRAGMENT_SOURCE_WINDING").as_deref() == Ok("0") {
        None
    } else {
        source_loop_winding_negative(face)?
    };
    let reverse_loops = source_winding.unwrap_or(!face.same_sense);
    if debug && reverse_loops != !face.same_sense {
        eprintln!(
            "  face {}: source loop winds {} against same_sense={} — orienting fragments to the source",
            face.id,
            if reverse_loops { "CW" } else { "CCW" },
            face.same_sense
        );
    }
    let mut fragments = Vec::new();
    for region in regions {
        let arranged: Vec<Vec2> = interior_points(&region.outer, &region.holes, region.area, 3);
        let candidates: Vec<Vec2> = arranged.iter().copied().map(wrap_back).collect();
        let Some(&test_uv) = candidates.first() else {
            if debug {
                eprintln!("  region area={:.6e}: no interior point", region.area);
            }
            continue;
        };
        let mut containment = parameter_point_in_face(face, test_uv, 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")?;
        // TRIM-FRAME CONTAINMENT. A face whose own pcurves run past its
        // carrier's domain (helmet Face_15: its trim spans u in
        // [-0.178, 0.298] on a carrier closed in u, drawn without a hop) is
        // arranged in that frame, and the region's point is on the face there.
        // Its wrapped image (u = 0.970 for -0.030) lies on no trim polygon,
        // and containment has no period image to try for a trim that never
        // hops the seam, so it reads Outside and the region was dropped: the
        // face's y >= 0 piece in `20_helmet_merge_stage` b - a, whose section
        // edges then stood one-use. With no band re-base the arrangement's
        // frame is the trim's own, so the region's point is asked there as
        // well. Escape hatch `BREP_REGION_TRIM_FRAME=0`.
        if containment != PolygonClass::Inside
            && band_rebase == [None, None]
            && (arranged[0].x != test_uv.x || arranged[0].y != test_uv.y)
            && std::env::var("BREP_REGION_TRIM_FRAME").as_deref() != Ok("0")
        {
            containment = parameter_point_in_face(face, arranged[0], 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")?;
            if debug {
                eprintln!(
                    "  region area={:.6e} test=({:.9},{:.9}) asked in the trim's frame at ({:.9},{:.9}): {:?}",
                    region.area, test_uv.x, test_uv.y, arranged[0].x, arranged[0].y,
                    containment == PolygonClass::Inside
                );
            }
        }
        if containment != PolygonClass::Inside {
            if debug {
                eprintln!(
                    "  region area={:.6e} test=({:.9},{:.9}): outside trimmed face",
                    region.area, test_uv.x, test_uv.y
                );
            }
            continue;
        }
        if debug {
            eprintln!(
                "  region area={:.6e} test=({:.9},{:.9}): KEPT",
                region.area, test_uv.x, test_uv.y
            );
        }
        let Some(outer_loop) = build_loop(
            &region.outer,
            reverse_loops,
            &chains,
            arrangement_tolerance,
            &face.surface,
        )?
        else {
            // Spurious zero-area antenna region (e.g. a seam-grazing fold on a
            // periodic band); it bounds no material, so drop it.
            if debug {
                eprintln!("  region area={:.6e}: dropped (degenerate antenna)", region.area);
            }
            continue;
        };
        let mut loops = vec![outer_loop];
        for hole in &region.holes {
            // Holes take the SAME flag as the outer loop: the arrangement
            // hands them back already wound against it, so reversing both
            // together keeps them opposite.
            if let Some(hole_loop) = build_loop(
                hole,
                reverse_loops,
                &chains,
                arrangement_tolerance,
                &face.surface,
            )? {
                loops.push(hole_loop);
            }
        }
        for loop_record in &mut loops {
            for coedge in &mut loop_record.coedges {
                if let FragmentEdgeSource::Boundary {
                    operand: source_operand,
                    ..
                } = &mut coedge.source
                {
                    *source_operand = operand;
                }
            }
        }
        let mut extra_test_points = Vec::new();
        for &candidate in candidates.iter().skip(1) {
            if parameter_point_in_face(face, candidate, 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")? == PolygonClass::Inside {
                extra_test_points.push(face.surface.evaluate(candidate.x, candidate.y).or_refuse(KernelStage::Fragment, "evaluate")?);
            }
        }
        fragments.push(FaceFragmentRecord {
            operand,
            source_face_id: face.id,
            surface: face.surface.clone(),
            same_sense: face.same_sense,
            loops,
            test_point: face.surface.evaluate(test_uv.x, test_uv.y).or_refuse(KernelStage::Fragment, "evaluate")?,
            test_uv,
            extra_test_points,
        });
    }

    // Rescue an UNCUT face the flat 2D arrangement failed to reconstruct.
    // A seam-closed multi-loop face (a band on a surface of revolution, whose
    // param-space loops only close across the u/v seam, or a holed face whose
    // domain centre lands in the hole so the fast path above declined it)
    // collapses to zero-area lines in the [0,1]^2 arrangement and yields NO
    // region -> the face is dropped entirely, stranding its neighbours' shared
    // edges as one-use and knocking the assembly's genus off an integer. Since
    // the face is uncut, its loops already describe the exact trimmed boundary,
    // so pass the unchanged face through — we only need a valid interior test
    // point, found by gridding the ACTUAL knot domain. Gated on
    // `fragments.is_empty()`, this never touches a face the arrangement handled.
    if fragments.is_empty() && piece_ids.is_empty() {
        const GRID: usize = 12;
        let mut test_uv: Option<Vec2> = None;
        let mut extra_test_points: Vec<Vec3> = Vec::new();
        'search: for iu in 1..GRID {
            for iv in 1..GRID {
                let candidate = Vec2 {
                    x: u_domain[0] + (u_domain[1] - u_domain[0]) * iu as f64 / GRID as f64,
                    y: v_domain[0] + (v_domain[1] - v_domain[0]) * iv as f64 / GRID as f64,
                };
                if parameter_point_in_face(face, candidate, 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")? == PolygonClass::Inside {
                    if test_uv.is_none() {
                        test_uv = Some(candidate);
                    } else {
                        extra_test_points.push(face.surface.evaluate(candidate.x, candidate.y).or_refuse(KernelStage::Fragment, "evaluate")?);
                        if extra_test_points.len() >= 2 {
                            break 'search;
                        }
                    }
                }
            }
        }
        // SEAM-STRADDLING BAND fallback (a SINGLY-periodic surface of
        // revolution whose trim loop closes only ACROSS the u/v seam — e.g. a
        // rounded imported cap grazed near-tangent, so it stays uncut). The
        // even-odd `parameter_point_in_face` reads Outside EVERYWHERE on such a
        // loop (the seam hop makes the [0,1]^2 polygon non-simple), so the knot
        // grid above finds nothing and the face would still drop. `seam_band_
        // point_in_face` only covers the DOUBLY-periodic (torus) case, so the
        // singly-periodic band has no seam-aware classifier. Unwrap the loop
        // onto the surface's covering plane (`loop_seam_offsets`, the same fold
        // the mass integrator and tessellator use), where it IS a simple
        // polygon, grid that unwrapped bbox with the plain even-odd test, and
        // map an interior hit back into the domain. Runs ONLY when the grid
        // already failed AND the offsets show the loop truly straddled a seam
        // (a full-wrap rim yields all-zero offsets and is left untouched), so it
        // never changes a face any earlier path handled.
        if test_uv.is_none() {
            let (closed_u, closed_v) = face.surface.closed_directions().or_refuse(KernelStage::Fragment, "closed_directions")?;
            if closed_u || closed_v {
                let u_period = u_domain[1] - u_domain[0];
                let v_period = v_domain[1] - v_domain[0];
                let mut loop_polygons: Vec<Vec<Vec2>> = Vec::new();
                let mut straddled = false;
                let (mut umin, mut umax, mut vmin, mut vmax) = (
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                );
                for loop_record in &face.loops {
                    let offsets = crate::topology::loop_seam_offsets(
                        &loop_record.coedges,
                        closed_u,
                        closed_v,
                        u_period,
                        v_period,
                    ).or_refuse(KernelStage::Fragment, "csg.fragment.face_split")?;
                    if offsets.iter().any(|o| o[0] != 0.0 || o[1] != 0.0) {
                        straddled = true;
                    }
                    let mut polygon = Vec::new();
                    for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                        let [d0, d1] = coedge.pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
                        let samples = 32usize;
                        for k in 0..samples {
                            let t = d0 + (d1 - d0) * k as f64 / samples as f64;
                            let p = coedge.pcurve.evaluate(t).or_refuse(KernelStage::Fragment, "evaluate")?;
                            let uv = Vec2 {
                                x: p.x + offsets[coedge_index][0],
                                y: p.y + offsets[coedge_index][1],
                            };
                            umin = umin.min(uv.x);
                            umax = umax.max(uv.x);
                            vmin = vmin.min(uv.y);
                            vmax = vmax.max(uv.y);
                            polygon.push(uv);
                        }
                    }
                    loop_polygons.push(polygon);
                }
                if straddled && umax > umin && vmax > vmin {
                    let wrap = |value: f64, lo: f64, span: f64| {
                        if span > 0.0 {
                            lo + (value - lo).rem_euclid(span)
                        } else {
                            value
                        }
                    };
                    'seam: for iu in 1..GRID {
                        for iv in 1..GRID {
                            let cu = umin + (umax - umin) * iu as f64 / GRID as f64;
                            let cv = vmin + (vmax - vmin) * iv as f64 / GRID as f64;
                            let candidate = Vec2 { x: cu, y: cv };
                            // Even-odd fill across ALL unwrapped loops: interior
                            // of the trimmed region is where the winding is odd
                            // (outer loop minus any hole loops), independent of
                            // which loop is the outer one.
                            let winding = loop_polygons
                                .iter()
                                .filter(|polygon| point_in_polygon(candidate, polygon))
                                .count();
                            if winding % 2 == 0 {
                                continue;
                            }
                            let mapped = Vec2 {
                                x: if closed_u {
                                    wrap(cu, u_domain[0], u_period)
                                } else {
                                    cu
                                },
                                y: if closed_v {
                                    wrap(cv, v_domain[0], v_period)
                                } else {
                                    cv
                                },
                            };
                            if test_uv.is_none() {
                                test_uv = Some(mapped);
                            } else {
                                extra_test_points
                                    .push(face.surface.evaluate(mapped.x, mapped.y).or_refuse(KernelStage::Fragment, "evaluate")?);
                                if extra_test_points.len() >= 2 {
                                    break 'seam;
                                }
                            }
                        }
                    }
                }
            }
        }
        if let Some(uv) = test_uv {
            fragments.push(FaceFragmentRecord {
                operand,
                source_face_id: face.id,
                surface: face.surface.clone(),
                same_sense: face.same_sense,
                loops: face
                    .loops
                    .iter()
                    .map(|loop_record| FragmentLoop {
                        coedges: loop_record
                            .coedges
                            .iter()
                            .map(|coedge| FragmentCoedge {
                                source: FragmentEdgeSource::Boundary {
                                    operand,
                                    edge_id: coedge.edge_id,
                                },
                                forward: coedge.forward,
                                pcurve: coedge.pcurve.clone(),
                            })
                            .collect(),
                    })
                    .collect(),
                test_point: face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Fragment, "evaluate")?,
                test_uv: uv,
                extra_test_points,
            });
        }
    }
    Ok(fragments)
}

pub fn fragment_solid(
    solid: &BrepSolid,
    operand: u8,
    imprint: &ImprintResultRecord,
) -> Result<Vec<FaceFragmentRecord>, KernelRefusal> {
    let index = FragmentIndex::build(imprint);
    let mut fragments = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| shell.faces.iter()) {
        fragments.extend(fragment_face_indexed(solid, operand, face, &index)?);
    }
    Ok(fragments)
}

