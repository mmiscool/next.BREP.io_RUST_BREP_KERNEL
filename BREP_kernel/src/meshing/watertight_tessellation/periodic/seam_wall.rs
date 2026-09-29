use super::*;

/// A single trim loop on a singly-periodic wall that CROSSES the parametric
/// seam (a window/patch straddling a cylinder's u=0 line). Built in the raw
/// [u0,u1] rectangle its seam jumps make consecutive vertices leap a full
/// period, so ear-clip sprays domain-spanning triangles across the opening
/// (onshape-unclamped-periodic faces 377/391/673/…). UNWRAP it: walk the loop
/// accumulating ±period at each seam jump, giving a seam-free simple polygon
/// in a continuous (unwrapped) domain that ear-clips correctly. The seam is
/// INTERIOR to the face (not a topological edge), so triangles may cross it
/// freely; the caller triangulates boundary-only (no interior seeding, which
/// would evaluate out of domain) and reduces the parameter modulo the period
/// for normals.
///
/// Returns None unless the surface is closed in exactly ONE direction, there
/// is exactly one loop, it actually crosses the seam, and it returns to its
/// starting branch (net winding 0 — not a full rim/spiral).
///
/// The returned bool is `needs_periodic_interior`: false for a SHORT straddle
/// (unwrapped span ≤ ~1.5 periods) that is boundary-only faithful on a
/// developable wall, and true for a HELICAL/threaded ribbon wrapping MANY
/// periods (onshape thread faces 583/651/734, screw Face_11). The tall ribbon
/// is still a single simple polygon once unwrapped — a leaning band of the
/// thread flank — so it ear-clips cleanly; but its cross-period chords bulge
/// off the (developable but u-curved) wall, so the caller must run periodic
/// interior refinement (evaluating the surface with the periodic parameter
/// wrapped back into domain). The multi-period case is admitted only after the
/// unwrapped polygon is verified simple (non-self-intersecting), so a
/// pathological multi-cover region still falls back to the general path.
pub(in crate::watertight_tessellation) fn unwrap_seam_crossing_trim(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<(Vec<FaceVertex>, bool)>, String> {
    if closed_u == closed_v || chains.len() != 1 {
        return Ok(None);
    }
    let [u0, u1, v0, v1] = domains;
    let p_is_u = closed_u;
    let (p0, p1) = if p_is_u { (u0, u1) } else { (v0, v1) };
    let (q0, q1) = if p_is_u { (v0, v1) } else { (u0, u1) };
    let period = p1 - p0;
    if !(period > 0.0) {
        return Ok(None);
    }
    // Boundary-only triangulation (the caller skips interior seeding) is
    // faithful ONLY when the CROSS direction is developable — a cylinder / cone
    // / ruled wall, where seed_interior_grid would add no stations anyway. A
    // doubly-curved wall (sphere, ellipsoid) needs seeded interior points to
    // meet the chord error and stay closed; unwrapping it boundary-only opens
    // the mesh (sphere_watertight / faceted_repair / mesh_segment regressions).
    // Gate exactly on seed_interior_grid's cross-direction test: bail when the
    // cross curvature's chord sag over the cross extent exceeds the tolerance.
    let cross_extent = (q1 - q0).abs();
    let mut cross_second: f64 = 0.0;
    let mut periodic_second: f64 = 0.0;
    for i in 0..=4 {
        for j in 0..=4 {
            let u = u0 + (u1 - u0) * i as f64 / 4.0;
            let v = v0 + (v1 - v0) * j as f64 / 4.0;
            let derivatives = face.surface.derivatives(u, v, 2)?;
            let second = if p_is_u {
                derivatives[0][2]
            } else {
                derivatives[2][0]
            };
            cross_second = cross_second.max(second.length());
            let p_second = if p_is_u {
                derivatives[2][0]
            } else {
                derivatives[0][2]
            };
            periodic_second = periodic_second.max(p_second.length());
        }
    }
    if cross_second * cross_extent * cross_extent > 8.0 * chord_tolerance {
        return Ok(None);
    }
    let p_of = |vertex: &FaceVertex| if p_is_u { vertex.uv[0] } else { vertex.uv[1] };
    let chain = &chains[0];
    let count = chain.len();
    if count < 3 {
        return Ok(None);
    }
    // Unwrap the periodic parameter continuously; a seam jump is a step larger
    // than half a period (real edges are sampled to chord tolerance, so only
    // the coedge-to-coedge seam hops leap that far).
    let mut unwrapped: Vec<FaceVertex> = Vec::with_capacity(count);
    let mut offset = 0.0f64;
    let mut jumps = 0usize;
    let mut previous = p_of(&chain[0]);
    for (index, vertex) in chain.iter().enumerate() {
        let raw = p_of(vertex);
        if index > 0 {
            let delta = raw - previous;
            if delta > 0.5 * period {
                offset -= period;
                jumps += 1;
            } else if delta < -0.5 * period {
                offset += period;
                jumps += 1;
            }
        }
        previous = raw;
        let mut uv = vertex.uv;
        if p_is_u {
            uv[0] = raw + offset;
        } else {
            uv[1] = raw + offset;
        }
        unwrapped.push(FaceVertex {
            uv,
            position: vertex.position,
        });
    }
    if jumps == 0 {
        return Ok(None); // ordinary loop entirely on one branch — general path
    }
    // The implicit closing edge (last -> first) may itself hop the seam; fold
    // it in, then require the loop to return to its starting branch. A net
    // ±period means the loop wraps the wall (a rim/spiral), not a straddle.
    let closing_delta = p_of(&chain[0]) - previous;
    let closing_offset = if closing_delta > 0.5 * period {
        offset - period
    } else if closing_delta < -0.5 * period {
        offset + period
    } else {
        offset
    };
    if closing_offset.abs() > 0.5 * period {
        return Ok(None);
    }
    let (mut pmin, mut pmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for vertex in &unwrapped {
        let p = if p_is_u { vertex.uv[0] } else { vertex.uv[1] };
        pmin = pmin.min(p);
        pmax = pmax.max(p);
    }
    if unwrapped.len() < 3 {
        return Ok(None);
    }
    // Boundary-only triangulation is faithful only when the ear-clipped chords
    // stay on the wall. That holds for a NARROW straddle, but fails two ways
    // that both need periodic interior refinement (evaluating midpoints with
    // the domain-wrapping evaluator and splitting cross-period chords down to
    // the surface):
    //   * a multi-period helical ribbon (span > ~1.5 periods), and
    //   * a WIDE straddle (≤1.5 periods but wrapping most of the circumference)
    //     whose interior triangles span far enough in the curved periodic
    //     direction that their chord sag exceeds the tolerance — the
    //     onshape-unclamped thread-relief face 806 (0.91 period wide) rendered
    //     8.7× the chord deep as a boundary-only strip.
    // The periodic-direction sag test mirrors the cross-direction guard above.
    // The tall/wide ribbon is admitted only if it is a simple polygon — a
    // self-intersecting unwrap is a multi-cover region that ear-clip would
    // spray, so leave it to the general path (no worse than today's fallback).
    let span = pmax - pmin;
    let periodic_sag_exceeds = periodic_second * span * span > 8.0 * chord_tolerance;
    let needs_periodic_interior = span > 1.5 * period || periodic_sag_exceeds;
    if needs_periodic_interior && !polygon_is_simple(&unwrapped) {
        return Ok(None);
    }
    if signed_area(&unwrapped) < 0.0 {
        unwrapped.reverse();
    }
    Ok(Some((unwrapped, needs_periodic_interior)))
}

/// True when the closed polygon (uv) has no pair of non-adjacent edges that
/// properly cross — the ear-clip precondition. Edges are swept by their u
/// bounds so disjoint pairs never reach the exact crossing predicate. The
/// result is identical to the quadratic scan, while dense hole grids avoid
/// repeatedly comparing every boundary segment with every other segment.
pub(in crate::watertight_tessellation) fn polygon_is_simple(polygon: &[FaceVertex]) -> bool {
    let n = polygon.len();
    if n < 4 {
        return true;
    }
    // Edge-length scale for a proper-crossing epsilon consistent with ear_clip.
    let mut total = 0.0f64;
    for i in 0..n {
        let a = polygon[i].uv;
        let b = polygon[(i + 1) % n].uv;
        total += ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
    }
    let epsilon = (total / n as f64).powi(2) * 1e-12;
    let mut edges: Vec<(f64, f64, f64, f64, usize)> = (0..n)
        .map(|index| {
            let a = polygon[index].uv;
            let b = polygon[(index + 1) % n].uv;
            (
                a[0].min(b[0]),
                a[0].max(b[0]),
                a[1].min(b[1]),
                a[1].max(b[1]),
                index,
            )
        })
        .collect();
    edges.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.4.cmp(&b.4)));
    for left in 0..edges.len() {
        let (_, max_u, min_v, max_v, i) = edges[left];
        let a = polygon[i].uv;
        let b = polygon[(i + 1) % n].uv;
        for &(other_min_u, _, other_min_v, other_max_v, j) in &edges[left + 1..] {
            if other_min_u > max_u {
                break;
            }
            if other_min_v > max_v || other_max_v < min_v {
                continue;
            }
            // Skip edges sharing a vertex with edge i (adjacent, incl. wrap).
            if j == i || (j + 1) % n == i || (i + 1) % n == j {
                continue;
            }
            let c = polygon[j].uv;
            let d = polygon[(j + 1) % n].uv;
            if segments_properly_cross(a, b, c, d, epsilon) {
                return false;
            }
        }
    }
    true
}

/// Rebuild the covering polygon for a singly-periodic full-wrap wall (closed in
/// exactly ONE direction) that is a clean seam-cut RECTANGLE — top/bottom rims
/// plus explicit p0/p1 seam edges — but is pierced by one or more small holes
/// that STRADDLE the parametric seam (a ring drilled across its own u=0≡u=1
/// meridian; ABC 00000333 interlocking rings). Such a hole, unwrapped, is a lens
/// whose vertices land ON the outer covering loop's seam edge, so the CDT
/// declines the whole face as a self-touching PSLG and `bridge_holes` sprays a
/// domain-spanning membrane over it (the inside-out fold).
///
/// The seam is CUT so the LEFT (p0) and RIGHT (p1) seam boundaries are separate
/// columns (they weld downstream by identical meridian 3D position), and each
/// straddle hole is spliced into them as a pair of concave NOTCHES — its right
/// arc (bulging in from p1) into the right seam, its left arc (bulging in from
/// p0) into the left seam. The two arcs meet the seam at the hole's two crossing
/// vertices, which are 3D-coincident (p0 ≡ p1) so the notched seams stay
/// watertight exactly as the untouched seam edges already did. The result is a
/// single SIMPLE covering polygon that `surface_cdt` owns; the genuinely
/// interior holes (that never reach the seam) pass straight through as CDT holes.
///
/// Returns None — every other wall keeps its existing path — unless the wall is
/// closed in exactly one direction, its outer loop is a simple rectangle whose
/// vertices all sit on the parameter-domain border, at least one hole actually
/// straddles the seam, and each straddle splits into exactly one left + one right
/// arc. The rims reuse the authored rim edge samples (so they weld with the cap
/// faces); the seam columns are re-sampled meridians (internal to the wall, so
/// they weld only against their own mirror copy).
pub(in crate::watertight_tessellation) fn seam_straddle_wall_polygons(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<(Vec<FaceVertex>, Vec<Vec<FaceVertex>>)>, String> {
    if closed_u == closed_v || chains.is_empty() {
        return Ok(None);
    }
    let [u0, u1, v0, v1] = domains;
    let p_is_u = closed_u;
    let (p0, p1) = if p_is_u { (u0, u1) } else { (v0, v1) };
    let (q0, q1) = if p_is_u { (v0, v1) } else { (u0, u1) };
    let period = p1 - p0;
    let q_extent = q1 - q0;
    if !(period > 0.0) || !(q_extent.abs() > 0.0) {
        return Ok(None);
    }
    let p_of = |vertex: &FaceVertex| if p_is_u { vertex.uv[0] } else { vertex.uv[1] };
    let q_of = |vertex: &FaceVertex| if p_is_u { vertex.uv[1] } else { vertex.uv[0] };
    let mk = |p: f64, q: f64| -> [f64; 2] {
        if p_is_u {
            [p, q]
        } else {
            [q, p]
        }
    };
    // Outer loop = the largest |area| chain.
    let outer_index = (0..chains.len()).max_by(|&a, &b| {
        signed_area(&chains[a])
            .abs()
            .total_cmp(&signed_area(&chains[b]).abs())
    });
    let Some(outer_index) = outer_index else {
        return Ok(None);
    };
    let outer = &chains[outer_index];
    let dbg = std::env::var("BREP_DEBUG_STRADDLE").is_ok();
    if outer.len() < 4 || !polygon_is_simple(outer) {
        if dbg {
            eprintln!("[straddle] bail: outer len={} simple={}", outer.len(), polygon_is_simple(outer));
        }
        return Ok(None);
    }
    // The wall is a full wrap in p (period = the closed direction) but is TRIMMED
    // in q to a sub-band whose two rims sit at interior q-levels (a cylinder ring
    // trimmed to v ∈ [qrim_lo, qrim_hi], not the whole surface). Derive the rim
    // levels from the outer's own q-extent, and require the outer to be a clean
    // seam-cut rectangle: every vertex on either rim or on a p0/p1 seam. A wavy /
    // knurled rim (interior vertices) is a harder class handled elsewhere.
    let (mut qrim_lo, mut qrim_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for vertex in outer {
        let q = q_of(vertex);
        qrim_lo = qrim_lo.min(q);
        qrim_hi = qrim_hi.max(q);
    }
    let rim_extent = qrim_hi - qrim_lo;
    if !(rim_extent > 0.0) {
        return Ok(None);
    }
    let p_tol = 1e-3 * period;
    let q_tol = 1e-3 * rim_extent;
    let mut bottom: Vec<FaceVertex> = Vec::new();
    let mut top: Vec<FaceVertex> = Vec::new();
    for vertex in outer {
        let (p, q) = (p_of(vertex), q_of(vertex));
        let on_q0 = (q - qrim_lo).abs() <= q_tol;
        let on_q1 = (q - qrim_hi).abs() <= q_tol;
        let on_seam = (p - p0).abs() <= p_tol || (p - p1).abs() <= p_tol;
        if on_q0 {
            bottom.push(*vertex);
        } else if on_q1 {
            top.push(*vertex);
        } else if !on_seam {
            if dbg {
                eprintln!("[straddle] bail: interior outer vertex p={p:.5} q={q:.5}");
            }
            return Ok(None);
        }
    }
    if bottom.len() < 2 || top.len() < 2 {
        if dbg {
            eprintln!("[straddle] bail: bottom={} top={}", bottom.len(), top.len());
        }
        return Ok(None);
    }
    bottom.sort_by(|a, b| p_of(a).total_cmp(&p_of(b)));
    top.sort_by(|a, b| p_of(b).total_cmp(&p_of(a)));

    let mut interior: Vec<Vec<FaceVertex>> = Vec::new();
    let mut right_arcs: Vec<Vec<FaceVertex>> = Vec::new();
    let mut left_arcs: Vec<Vec<FaceVertex>> = Vec::new();
    for (index, chain) in chains.iter().enumerate() {
        if index == outer_index {
            continue;
        }
        let (mut pmin, mut pmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for vertex in chain {
            let p = p_of(vertex);
            pmin = pmin.min(p);
            pmax = pmax.max(p);
        }
        if pmax - pmin > 0.5 * period {
            // A genuine seam-straddle hole CROSSES the seam and returns — its net
            // winding in p is ~0. A full-wrap loop that circles the whole period
            // (a thread crest/profile rim, ABC screw Face runouts) nets ±period;
            // splicing it as a notch would corrupt the outer and tear the shared
            // seam. Decline the whole face so those keep their existing path.
            let n = chain.len();
            let mut net = 0.0;
            for i in 0..n {
                let mut delta = p_of(&chain[(i + 1) % n]) - p_of(&chain[i]);
                if delta > 0.5 * period {
                    delta -= period;
                } else if delta < -0.5 * period {
                    delta += period;
                }
                net += delta;
            }
            if net.abs() > 0.25 * period {
                if dbg {
                    eprintln!("[straddle] bail: full-wrap loop {index} (net={net:.4})");
                }
                return Ok(None);
            }
            let Some((right, left)) = super::rim::split_seam_arcs(chain, [p0, p1], p_is_u) else {
                if dbg {
                    eprintln!("[straddle] bail: split_arcs failed on loop {index} (n={})", chain.len());
                }
                return Ok(None);
            };
            right_arcs.push(right);
            left_arcs.push(left);
        } else {
            interior.push(chain.clone());
        }
    }
    if right_arcs.is_empty() {
        if dbg {
            eprintln!("[straddle] bail: no straddle holes (interior={})", interior.len());
        }
        return Ok(None);
    }
    // Build one seam column (p = `p_seam`) ascending in q, EXCLUDING the q0/q1
    // corner samples (they come from the rims) and splicing the given arcs into
    // their q-ranges. Each arc's endpoints already sit on the seam.
    let build_seam = |p_seam: f64,
                      arcs: &[Vec<FaceVertex>]|
     -> Result<Vec<FaceVertex>, String> {
        let base = seam_meridian_samples(
            face,
            p_seam,
            p_is_u,
            qrim_lo,
            qrim_hi,
            chord_tolerance,
            false,
        )?;
        let mut ranges: Vec<(f64, f64, usize)> = arcs
            .iter()
            .enumerate()
            .map(|(index, arc)| {
                let qa = q_of(&arc[0]);
                let qb = q_of(arc.last().unwrap());
                (qa.min(qb), qa.max(qb), index)
            })
            .collect();
        ranges.sort_by(|a, b| a.0.total_cmp(&b.0));
        let q_lo = qrim_lo;
        let q_hi = qrim_hi;
        let border_eps = 1e-9 * rim_extent.max(1.0);
        let mut column: Vec<FaceVertex> = Vec::new();
        let mut cursor = q_lo;
        for &(range_lo, range_hi, arc_index) in &ranges {
            for &(q, position) in &base {
                if q > cursor + border_eps
                    && q > q_lo + border_eps
                    && q < range_lo - border_eps
                {
                    column.push(FaceVertex {
                        uv: mk(p_seam, q),
                        position,
                    });
                }
            }
            for vertex in &arcs[arc_index] {
                column.push(FaceVertex {
                    uv: mk(p_seam, q_of(vertex)),
                    position: vertex.position,
                });
            }
            cursor = range_hi;
        }
        for &(q, position) in &base {
            if q > cursor + border_eps && q < q_hi - border_eps {
                column.push(FaceVertex {
                    uv: mk(p_seam, q),
                    position,
                });
            }
        }
        Ok(column)
    };
    let right_seam = build_seam(p1, &right_arcs)?;
    let mut left_seam = build_seam(p0, &left_arcs)?;
    left_seam.reverse();
    let mut polygon: Vec<FaceVertex> = Vec::new();
    polygon.extend(bottom.iter().copied());
    polygon.extend(right_seam);
    polygon.extend(top.iter().copied());
    polygon.extend(left_seam);
    if polygon.len() < 3 || !polygon_is_simple(&polygon) {
        if dbg {
            eprintln!("[straddle] bail: assembled polygon len={} simple={}", polygon.len(), polygon_is_simple(&polygon));
        }
        return Ok(None);
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }
    let holes = interior
        .into_iter()
        .map(|mut hole| {
            if signed_area(&hole) > 0.0 {
                hole.reverse();
            }
            hole
        })
        .collect::<Vec<_>>();
    Ok(Some((polygon, holes)))
}

/// Rebuild a singly-periodic full-wrap wall (closed in exactly ONE direction)
/// pierced by a loop that STRADDLES the parametric seam, by RE-CUTTING the
/// covering at a hole-free meridian. The authored seam runs THROUGH such a loop
/// (a large lens/window centred on `u = 0 ≡ u = 1`), so
/// [`seam_straddle_wall_polygons`] carves it as two deep boundary bites whose
/// curved rims the CDT/refine double-cover (the folded gold-spike membrane the
/// user reported). Instead, pick a meridian `s` that no loop crosses, ROTATE the
/// covering onto `[s, s+period]`, and every loop — the straddler included —
/// becomes a fully-interior hole in a clean seam-cut rectangle: a simple
/// multi-hole PSLG `surface_cdt` owns, with the re-cut seam an ordinary welded
/// meridian (its two columns share identical 3D so they weld exactly like the
/// authored seam did).
///
/// This is the multi-loop generalisation of [`close_periodic_trim_dir`] (which
/// bails the instant a hole reaches the seam) and the robust superset of
/// [`seam_straddle_wall_polygons`] (which only survives a SHALLOW straddle bite).
///
/// Returns None — every other wall keeps its existing path — unless: closed in
/// exactly one direction; the outer loop is a clean full-wrap rectangle (two
/// constant-cross-level rims + `p0`/`p1` seam edges, no wavy interior rim
/// vertices); every non-outer loop is a genuine positive-area, non-full-wrap
/// hole; and a comfortably hole-free meridian gap exists. The rims reuse the
/// authored rim samples (only relabelled in `p`, so their 3D — and thus their
/// weld with the cap faces — is bit-identical); the re-cut seam columns are
/// fresh meridian samples internal to the wall.
#[allow(clippy::too_many_arguments)]
pub(in crate::watertight_tessellation) fn close_periodic_recut_wall(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<(Vec<FaceVertex>, Vec<Vec<FaceVertex>>)>, String> {
    if closed_u == closed_v || chains.is_empty() {
        return Ok(None);
    }
    let [u0, u1, v0, v1] = domains;
    let p_is_u = closed_u;
    let (p0, p1) = if p_is_u { (u0, u1) } else { (v0, v1) };
    let (q0, q1) = if p_is_u { (v0, v1) } else { (u0, u1) };
    let period = p1 - p0;
    let q_extent = (q1 - q0).abs();
    if !(period > 0.0) || !(q_extent > 0.0) {
        return Ok(None);
    }
    let dbg = std::env::var("BREP_DEBUG_RECUT").is_ok();
    let p_of = |vertex: &FaceVertex| if p_is_u { vertex.uv[0] } else { vertex.uv[1] };
    let q_of = |vertex: &FaceVertex| if p_is_u { vertex.uv[1] } else { vertex.uv[0] };
    let mk = |p: f64, q: f64| -> [f64; 2] { if p_is_u { [p, q] } else { [q, p] } };

    // Outer loop = the largest |area| chain; must be a simple polygon.
    let Some(outer_index) = (0..chains.len()).max_by(|&a, &b| {
        signed_area(&chains[a])
            .abs()
            .total_cmp(&signed_area(&chains[b]).abs())
    }) else {
        return Ok(None);
    };
    let outer = &chains[outer_index];
    if outer.len() < 4 || !polygon_is_simple(outer) {
        return Ok(None);
    }
    // Rim levels from the outer's own q-extent (a sub-band trimmed cylinder).
    let (mut qrim_lo, mut qrim_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for vertex in outer {
        let q = q_of(vertex);
        qrim_lo = qrim_lo.min(q);
        qrim_hi = qrim_hi.max(q);
    }
    let rim_extent = qrim_hi - qrim_lo;
    if !(rim_extent > 0.0) {
        return Ok(None);
    }
    let p_tol = 1e-3 * period;
    let q_tol = 1e-3 * rim_extent;
    // Classify outer vertices into bottom / top rim; DISCARD the old seam-edge
    // interior vertices (p ≈ p0/p1 with q strictly between the rims). A vertex
    // that is neither on a rim nor on the seam means a wavy/irregular outer — a
    // harder class handled elsewhere; decline cleanly.
    let mut bottom: Vec<FaceVertex> = Vec::new();
    let mut top: Vec<FaceVertex> = Vec::new();
    for vertex in outer {
        let (p, q) = (p_of(vertex), q_of(vertex));
        let on_lo = (q - qrim_lo).abs() <= q_tol;
        let on_hi = (q - qrim_hi).abs() <= q_tol;
        let on_seam = (p - p0).abs() <= p_tol || (p - p1).abs() <= p_tol;
        if on_lo {
            bottom.push(*vertex);
        } else if on_hi {
            top.push(*vertex);
        } else if !on_seam {
            if dbg {
                eprintln!("[recut] bail: interior outer vertex p={p:.5} q={q:.5}");
            }
            return Ok(None);
        }
    }
    if bottom.len() < 2 || top.len() < 2 {
        return Ok(None);
    }
    // Non-outer loops must be genuine positive-area, non-full-wrap holes. Collect
    // their p (mod period) so the largest hole-free circular gap gives the re-cut
    // meridian. A full-wrap loop (thread profile / second rim) nets ±period — not
    // our shape, decline. Pole / zero-area loops are dropped (not material holes).
    let domain_area = (period * rim_extent).abs();
    let hole_eps = 1e-9 * domain_area.max(1e-30);
    let mut hole_indices: Vec<usize> = Vec::new();
    let mut angles: Vec<f64> = Vec::new(); // p relative to p0, in [0, period)
    let mut any_straddle = false;
    for (index, chain) in chains.iter().enumerate() {
        if index == outer_index {
            continue;
        }
        if signed_area(chain).abs() <= hole_eps {
            continue;
        }
        let n = chain.len();
        let (mut pmin, mut pmax) = (f64::INFINITY, f64::NEG_INFINITY);
        let mut net = 0.0;
        for vertex in chain {
            let p = p_of(vertex);
            pmin = pmin.min(p);
            pmax = pmax.max(p);
        }
        for i in 0..n {
            let mut delta = p_of(&chain[(i + 1) % n]) - p_of(&chain[i]);
            if delta > 0.5 * period {
                delta -= period;
            } else if delta < -0.5 * period {
                delta += period;
            }
            net += delta;
        }
        if net.abs() > 0.25 * period {
            if dbg {
                eprintln!("[recut] bail: full-wrap loop {index} (net={net:.4})");
            }
            return Ok(None);
        }
        let straddles =
            pmax - pmin > 0.5 * period || pmin <= p0 + p_tol || pmax >= p1 - p_tol;
        any_straddle = any_straddle || straddles;
        hole_indices.push(index);
        for vertex in chain {
            let a = (p_of(vertex) - p0).rem_euclid(period);
            angles.push(a);
        }
    }
    // Only OWN a wall that actually straddles the seam — a fully-interior
    // multi-hole cylinder is `close_periodic_trim_dir`'s and stays there.
    if !any_straddle || hole_indices.is_empty() {
        return Ok(None);
    }
    // Largest circular gap between hole vertices → re-cut meridian at its centre.
    angles.sort_by(|a, b| a.total_cmp(b));
    let (mut gap, mut gap_center) = (0.0f64, 0.0f64);
    for w in angles.windows(2) {
        let g = w[1] - w[0];
        if g > gap {
            gap = g;
            gap_center = 0.5 * (w[0] + w[1]);
        }
    }
    // wrap gap from the last angle back to the first
    let wrap = (angles[0] + period) - angles[angles.len() - 1];
    if wrap > gap {
        gap = wrap;
        gap_center = (0.5 * (angles[angles.len() - 1] + angles[0] + period)).rem_euclid(period);
    }
    // Require the meridian to sit COMFORTABLY clear of every hole so re-sampled
    // seam columns never clip one (a genuine solid wall always has clear
    // meridians between its drilled features; a hole grid with none is not this
    // shape).
    if gap < 0.02 * period {
        if dbg {
            eprintln!("[recut] bail: no hole-free meridian (largest gap={:.4} of period)", gap / period);
        }
        return Ok(None);
    }
    let seam_p = p0 + gap_center;
    // Rotate a p onto the covering [seam_p, seam_p+period): the full-period shift
    // leaves 3D untouched (same meridian), only relabelling the parameter.
    let shift = |p: f64| -> f64 { if p < seam_p { p + period } else { p } };

    // Shift + sort the rims onto the re-cut covering.
    let mut bottom_s: Vec<FaceVertex> = bottom
        .iter()
        .map(|vertex| FaceVertex {
            uv: mk(shift(p_of(vertex)), q_of(vertex)),
            position: vertex.position,
        })
        .collect();
    let mut top_s: Vec<FaceVertex> = top
        .iter()
        .map(|vertex| FaceVertex {
            uv: mk(shift(p_of(vertex)), q_of(vertex)),
            position: vertex.position,
        })
        .collect();
    bottom_s.sort_by(|a, b| p_of(a).total_cmp(&p_of(b)));
    top_s.sort_by(|a, b| p_of(a).total_cmp(&p_of(b)));
    // The authored seam endpoints (p0 and p1) map to the SAME p after the shift
    // (both are the u=0≡u=1 meridian), so each rim carries a coincident duplicate.
    // Drop it — a zero-length rim edge is a self-touching PSLG the CDT declines.
    let dedup = |v: &mut FaceVertex, w: &mut FaceVertex| -> bool {
        (v.uv[0] - w.uv[0]).abs() <= p_tol && (v.uv[1] - w.uv[1]).abs() <= q_tol
    };
    bottom_s.dedup_by(dedup);
    top_s.dedup_by(dedup);
    if bottom_s.len() < 2 || top_s.len() < 2 {
        return Ok(None);
    }

    // The re-cut seam runs between the FIRST authored rim vertex past the cut on
    // each rim (bottom-left `bl`, top-left `tl`). Using authored rim vertices as
    // the seam corners is what keeps the wall WATERTIGHT: a fresh seam-meridian
    // corner would be a rim point the neighbouring cap face never samples (a
    // T-junction). `bl`/`tl` sit in the hole-free gap; require them comfortably
    // inside it so the near-vertical seam never clips a hole (a rim too sparse
    // near the cut declines to the old path).
    let bl = bottom_s[0];
    let tl = top_s[0];
    let (p_bl, q_bl) = (p_of(&bl), q_of(&bl));
    let (p_tl, q_tl) = (p_of(&tl), q_of(&tl));
    if (p_bl - seam_p).abs() > 0.25 * gap || (p_tl - seam_p).abs() > 0.25 * gap {
        if dbg {
            eprintln!("[recut] bail: rim too sparse near cut (p_bl={p_bl:.4} p_tl={p_tl:.4} seam_p={seam_p:.4} gap={:.4})", gap);
        }
        return Ok(None);
    }
    // Sample the near-vertical seam from `bl` (q_bl) to `tl` (q_tl): interpolate p
    // linearly in q and evaluate on the surface (extended evaluator — the covering
    // is rotated, so p may run past the domain end). Both columns share this
    // profile (right = +period), so their identical 3D welds the seam exactly.
    let eval_diag = |q: f64| -> Result<(f64, Vec3), String> {
        let t = if (q_tl - q_bl).abs() > 1e-30 {
            (q - q_bl) / (q_tl - q_bl)
        } else {
            0.0
        };
        let p = p_bl + (p_tl - p_bl) * t;
        let position = if p_is_u {
            face.surface.evaluate_extended(p, q)?
        } else {
            face.surface.evaluate_extended(q, p)?
        };
        Ok((p, position))
    };
    let mut qs = vec![q_bl, q_tl];
    loop {
        let mut refined = Vec::with_capacity(qs.len() * 2);
        let mut split = false;
        for pair in qs.windows(2) {
            refined.push(pair[0]);
            let mid = 0.5 * (pair[0] + pair[1]);
            if deflection_to_chord(eval_diag(mid)?.1, eval_diag(pair[0])?.1, eval_diag(pair[1])?.1)
                > chord_tolerance
            {
                refined.push(mid);
                split = true;
            }
        }
        refined.push(*qs.last().unwrap());
        qs = refined;
        if !split || qs.len() > MAX_EDGE_SPANS {
            break;
        }
    }
    let diagonal: Vec<(f64, f64, Vec3)> = qs
        .into_iter()
        .map(|q| {
            let (p, position) = eval_diag(q)?;
            Ok::<_, String>((p, q, position))
        })
        .collect::<Result<Vec<_>, _>>()?;
    if diagonal.len() < 2 {
        return Ok(None);
    }

    // Assemble the rectangle CCW in (p, q): bottom rim (bl → around → bl+period),
    // right seam (bottom-right → top-right), top rim reversed, left seam
    // (top-left → bottom-left). The corners `bl`/`tl` are authored rim vertices;
    // the seam interiors are internal and weld column-to-column by identical 3D.
    let mut polygon: Vec<FaceVertex> = Vec::new();
    polygon.extend(bottom_s.iter().copied());
    polygon.push(FaceVertex { uv: mk(p_bl + period, q_bl), position: bl.position });
    for &(p, q, position) in &diagonal[1..diagonal.len() - 1] {
        polygon.push(FaceVertex { uv: mk(p + period, q), position });
    }
    polygon.push(FaceVertex { uv: mk(p_tl + period, q_tl), position: tl.position });
    for vertex in top_s.iter().rev() {
        polygon.push(*vertex);
    }
    for &(p, q, position) in diagonal[1..diagonal.len() - 1].iter().rev() {
        polygon.push(FaceVertex { uv: mk(p, q), position });
    }
    // Guard against any residual consecutive-coincident vertex (a zero-length
    // edge the CDT would reject as a self-touch).
    polygon.dedup_by(|a, b| {
        (a.uv[0] - b.uv[0]).abs() <= p_tol && (a.uv[1] - b.uv[1]).abs() <= q_tol
    });
    if polygon.len() < 3 || !polygon_is_simple(&polygon) {
        if dbg {
            eprintln!("[recut] bail: assembled polygon len={} simple={}", polygon.len(), polygon_is_simple(&polygon));
        }
        return Ok(None);
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }
    // Every hole is now fully interior to [seam_p, seam_p2]; shift + orient CW.
    let mut holes: Vec<Vec<FaceVertex>> = Vec::new();
    for &index in &hole_indices {
        let mut hole: Vec<FaceVertex> = chains[index]
            .iter()
            .map(|vertex| FaceVertex {
                uv: mk(shift(p_of(vertex)), q_of(vertex)),
                position: vertex.position,
            })
            .collect();
        if signed_area(&hole) > 0.0 {
            hole.reverse();
        }
        holes.push(hole);
    }
    if dbg {
        eprintln!(
            "[recut] seam_p={seam_p:.4} gap={:.4} p_bl={p_bl:.4} p_tl={p_tl:.4} diag_n={} outer={} holes={} straddle={any_straddle}",
            gap / period,
            diagonal.len(),
            polygon.len(),
            holes.len()
        );
    }
    Ok(Some((polygon, holes)))
}
