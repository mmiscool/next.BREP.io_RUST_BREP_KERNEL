use super::*;

/// One cross-level boundary of a periodic BAND (see [`close_periodic_band`]):
/// either a constant-cross-level rim, or a single-valued "profile" edge whose
/// cross level varies as the periodic parameter sweeps (a thread crest/root
/// zig-zag), but which still crosses each periodic level exactly once.
struct BandBoundary {
    /// Chain vertices ordered by ascending periodic parameter, seam corners
    /// (at p0 and p1) inserted. Positions are authoritative edge samples.
    points: Vec<FaceVertex>,
    /// Cross level AT the seam meridian (p0 ≡ p1), used for the seam runs.
    seam_q: f64,
    /// Cross-level extent of this boundary (constant rim ⇒ qmin ≈ qmax).
    qmin: f64,
    qmax: f64,
}

/// Classify ONE loop of a singly-periodic wall as a band boundary: a FULL-WRAP
/// loop (net winding ±1 across the seam) that is SINGLE-VALUED in the periodic
/// parameter (its unwrapped parameter is monotonic — each periodic level is
/// crossed exactly once). Returns None for a partial straddle (winding 0) or a
/// multi-period helical ribbon (non-monotonic there-and-back), so those keep
/// their existing paths. The seam corner reuses the loop's own seam-crossing
/// vertex position, so the assembled band stays watertight against neighbours.
fn classify_band_boundary(
    chain: &[FaceVertex],
    p_is_u: bool,
    p0: f64,
    p1: f64,
) -> Option<BandBoundary> {
    let period = p1 - p0;
    if !(period > 0.0) || chain.len() < 2 {
        return None;
    }
    let coord = |vertex: &FaceVertex| -> (f64, f64) {
        if p_is_u {
            (vertex.uv[0], vertex.uv[1])
        } else {
            (vertex.uv[1], vertex.uv[0])
        }
    };
    // Unwrap the periodic parameter continuously and require net winding ±1
    // (a full wrap) plus monotonicity (single-valued cross profile).
    let mut offset = 0.0f64;
    let mut previous = coord(&chain[0]).0;
    let mut unwrapped: Vec<f64> = Vec::with_capacity(chain.len());
    for (index, vertex) in chain.iter().enumerate() {
        let raw = coord(vertex).0;
        if index > 0 {
            let delta = raw - previous;
            if delta > 0.5 * period {
                offset -= period;
            } else if delta < -0.5 * period {
                offset += period;
            }
        }
        previous = raw;
        unwrapped.push(raw + offset);
    }
    let closing = coord(&chain[0]).0 - previous;
    let closing_offset = if closing > 0.5 * period {
        offset - period
    } else if closing < -0.5 * period {
        offset + period
    } else {
        offset
    };
    // Full wrap: the loop returns one period away from where it started.
    if (closing_offset.abs() - period).abs() > 0.25 * period {
        return None;
    }
    // Monotonic in the unwrapped parameter (single-valued): no direction
    // reversal larger than sampling jitter. A helical ribbon (there-and-back)
    // has two big reversals and is rejected here.
    let eps = 1e-6 * period;
    let mut sign = 0i32;
    for pair in unwrapped.windows(2) {
        let delta = pair[1] - pair[0];
        let this = if delta > eps {
            1
        } else if delta < -eps {
            -1
        } else {
            0
        };
        if this != 0 {
            if sign != 0 && this != sign {
                return None;
            }
            sign = this;
        }
    }
    if sign == 0 {
        return None;
    }
    // Ascending-parameter chain (raw p ∈ [p0,p1]); single-valued, so sorting by
    // raw parameter reproduces the seam-cut boundary.
    let mut points: Vec<FaceVertex> = chain.to_vec();
    points.sort_by(|a, b| coord(a).0.total_cmp(&coord(b).0));
    let (mut qmin, mut qmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for vertex in &points {
        let q = coord(vertex).1;
        qmin = qmin.min(q);
        qmax = qmax.max(q);
    }
    // The chain endpoint nearest the seam (u=0≡u=1) carries the seam-meridian
    // cross level and the AUTHORITATIVE seam-corner position (a real edge
    // sample, so the corner welds against a non-periodic neighbouring face).
    let front_gap = coord(&points[0]).0 - p0;
    let back_gap = p1 - coord(points.last().unwrap()).0;
    let seam_vertex = if front_gap <= back_gap {
        points[0]
    } else {
        *points.last().unwrap()
    };
    let seam_q = coord(&seam_vertex).1;
    let seam_uv = |p: f64| -> [f64; 2] {
        if p_is_u {
            [p, seam_q]
        } else {
            [seam_q, p]
        }
    };
    // A missing seam endpoint's 3D position is the SURVIVING endpoint at the
    // OTHER seam side (p0 ≡ p1 on the closed wall), matching `close_periodic_trim`.
    let corner_eps = 1e-6 * period;
    let front_position = points[0].position;
    let back_position = points.last().unwrap().position;
    if back_gap > corner_eps {
        points.push(FaceVertex {
            uv: seam_uv(p1),
            position: front_position,
        });
    }
    if front_gap > corner_eps {
        points.insert(
            0,
            FaceVertex {
                uv: seam_uv(p0),
                position: back_position,
            },
        );
    }
    Some(BandBoundary {
        points,
        seam_q,
        qmin,
        qmax,
    })
}

/// Rebuild the trim polygon for a singly-periodic developable wall whose region
/// is a full BAND around the wrap, bounded by TWO full-wrap loops — one a
/// constant-cross-level rim, the other a single-valued profile edge whose cross
/// level varies (a thread runout/transition face: a rim shoulder plus one turn
/// of the thread crest). `close_periodic_trim` rejects it (the profile loop is
/// not a clean constant-level rim) and the single-loop `unwrap_seam_crossing_trim`
/// rejects it (two loops), so it drops to `bridge_holes` on the RAW seamed
/// chains and sprays domain-spanning triangles.
///
/// Both loops cross the surface seam, so cutting there keeps every boundary
/// vertex authoritative; the profile loop, being single-valued, becomes the
/// bottom (or top) edge of the seam-cut rectangle exactly like a rim, only with
/// varying cross level. Returns None unless the wall is closed in exactly ONE
/// (developable) direction, has exactly two full-wrap single-valued loops of
/// which exactly one has a varying cross level, and the two loops are separable
/// (one stays entirely below the other) — anything else keeps its current path.
pub(in crate::watertight_tessellation) fn close_periodic_band(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<Vec<FaceVertex>>, String> {
    if chains.len() != 2 || (!closed_u && !closed_v) {
        return Ok(None);
    }
    // Singly-periodic wall (closed in exactly one direction), OR a TORUS band that
    // wraps u and is confined to a v-sub-interval — the common fillet case (a
    // rounded edge between a hole and a face is a trimmed torus whose two rims
    // both circle u at interior v-levels; ABC 00009867 faces 0/9). For a torus we
    // take p = u; a v-wrapping torus band simply fails `classify_band_boundary`
    // below (not full-wrap in u) and falls through unchanged.
    let torus = closed_u && closed_v;
    let [u0, u1, v0, v1] = domains;
    let p_is_u = closed_u;
    let (p0, p1) = if p_is_u { (u0, u1) } else { (v0, v1) };
    let (q0, q1) = if p_is_u { (v0, v1) } else { (u0, u1) };
    let period = p1 - p0;
    let q_extent = (q1 - q0).abs().max(1e-30);
    if !(period > 0.0) {
        return Ok(None);
    }
    let mut boundaries: Vec<BandBoundary> = Vec::with_capacity(2);
    for chain in chains {
        match classify_band_boundary(chain, p_is_u, p0, p1) {
            Some(boundary) => boundaries.push(boundary),
            None => return Ok(None),
        }
    }
    // The material band lies between the two boundaries in the cross parameter.
    // On a torus the SAME two rims also bound the complement (the strip that
    // crosses the v-seam); this in-domain path can only represent the between
    // strip, so require it to stay clear of the cross seam (both boundaries at
    // interior cross levels). A wall (cross not periodic) has only the one band.
    let band_qlo = boundaries.iter().map(|b| b.qmin).fold(f64::INFINITY, f64::min);
    let band_qhi = boundaries.iter().map(|b| b.qmax).fold(f64::NEG_INFINITY, f64::max);
    if torus {
        let margin = 0.02 * q_extent;
        if band_qlo <= q0 + margin || band_qhi >= q1 - margin {
            return Ok(None);
        }
    }
    // Developable cross direction only for a WALL: it is triangulated with
    // in-domain seed+refine, which relies on the boundary sampling carrying a
    // ruled wall's single-direction curvature (thread walls are cylinders/cones);
    // a doubly-curved wall would open the mesh — bail (matches the unwrap guard).
    // A TORUS fillet band is doubly curved, but its material strip stays inside
    // the cross domain (checked above), so the ordinary seed+refine evaluates
    // real surface points there and follows the curvature to chord tolerance;
    // the flat-cross requirement does not apply, and enforcing it (chord-scaled,
    // so ever tighter at fine render density) would spuriously reject it.
    if !torus {
        let cross_extent = (q1 - q0).abs();
        let mut cross_second: f64 = 0.0;
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
            }
        }
        if cross_second * cross_extent * cross_extent > 8.0 * chord_tolerance {
            return Ok(None);
        }
    }
    // Exactly one boundary must have a VARYING cross level (the profile); the
    // other a (near-)constant rim. Two constant rims are the plain
    // `close_periodic_trim` case (already tried); two profiles are ambiguous.
    let is_flat = |b: &BandBoundary| (b.qmax - b.qmin) <= 0.05 * q_extent;
    let flat_count = boundaries.iter().filter(|b| is_flat(b)).count();
    if flat_count != 1 {
        return Ok(None);
    }
    // Separable: one boundary stays entirely below the other (no interleave, so
    // the band is a simple polygon). Order lower→upper by cross level.
    boundaries.sort_by(|a, b| {
        let ka = 0.5 * (a.qmin + a.qmax);
        let kb = 0.5 * (b.qmin + b.qmax);
        ka.total_cmp(&kb)
    });
    let margin = 1e-6 * q_extent;
    if boundaries[0].qmax > boundaries[1].qmin + margin {
        return Ok(None);
    }
    // Assemble the seam-cut rectangle CCW in (p,q): bottom boundary p0→p1,
    // right seam q_lo→q_hi at p1, top boundary reversed p1→p0, left seam
    // q_hi→q_lo at p0. Seam corners come from the boundaries; the interior seam
    // samples (only present on a cross-curved wall) are private to the wall.
    let mk_uv = |p: f64, q: f64| if p_is_u { [p, q] } else { [q, p] };
    let (lo_seam_q, hi_seam_q) = (boundaries[0].seam_q, boundaries[1].seam_q);
    let seam = seam_meridian_samples(
        face,
        p1,
        p_is_u,
        lo_seam_q,
        hi_seam_q,
        chord_tolerance,
        false,
    )?;
    let mut polygon: Vec<FaceVertex> = Vec::new();
    polygon.extend(boundaries[0].points.iter().copied());
    if seam.len() > 2 {
        for &(q, position) in &seam[1..seam.len() - 1] {
            polygon.push(FaceVertex {
                uv: mk_uv(p1, q),
                position,
            });
        }
    }
    for vertex in boundaries[1].points.iter().rev() {
        polygon.push(*vertex);
    }
    if seam.len() > 2 {
        for &(q, position) in seam[1..seam.len() - 1].iter().rev() {
            polygon.push(FaceVertex {
                uv: mk_uv(p0, q),
                position,
            });
        }
    }
    if polygon.len() < 3 {
        return Ok(None);
    }
    if !polygon_is_simple(&polygon) {
        return Ok(None);
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }
    Ok(Some(polygon))
}

/// Lift one cycle of a loop chain into the periodic COVER: walk it from
/// `start`, accumulating ±`period` at every seam jump, and append a closing copy
/// of the start vertex at the far end. The returned list therefore holds the
/// whole loop plus its wrap-closing duplicate — n+1 vertices whose periodic
/// parameter runs continuously from `p_start` to `p_start + winding·period`.
/// Positions are the loop's own authoritative edge samples (the duplicate shares
/// the start vertex's position exactly, so the cut it sits on welds crack-free).
fn lift_loop_cycle(
    chain: &[FaceVertex],
    start: usize,
    p_is_u: bool,
    period: f64,
    p_start: f64,
) -> Vec<FaceVertex> {
    let coord = |vertex: &FaceVertex| -> (f64, f64) {
        if p_is_u {
            (vertex.uv[0], vertex.uv[1])
        } else {
            (vertex.uv[1], vertex.uv[0])
        }
    };
    let count = chain.len();
    let mut lifted = Vec::with_capacity(count + 1);
    let mut previous = coord(&chain[start]).0;
    let mut current = p_start;
    for step in 0..=count {
        let vertex = &chain[(start + step) % count];
        let (raw, q) = coord(vertex);
        if step > 0 {
            let mut delta = raw - previous;
            if delta > 0.5 * period {
                delta -= period;
            } else if delta < -0.5 * period {
                delta += period;
            }
            current += delta;
            previous = raw;
        }
        let uv = if p_is_u { [current, q] } else { [q, current] };
        lifted.push(FaceVertex {
            uv,
            position: vertex.position,
        });
    }
    lifted
}

/// A thread RUNOUT face whose region is an ANNULUS around the wrap: a
/// constant-level full-wrap RIM plus a MULTI-VALUED helical ribbon loop — the
/// thread flank spiralling many turns down the wall, turning around, and
/// spiralling back, with net winding OPPOSITE the rim's (screw 00035619
/// Face_38: a rim at v≈0.0065 and a 35-coedge ribbon spanning 15.7 periods with
/// two reversals; the region is a full annular band just above the rim that
/// feeds into the spiral arm).
///
/// Every other periodic path rejects it — `close_periodic_trim` because the
/// ribbon is not a clean rim, `close_periodic_band` because the ribbon is not
/// single-valued (`classify_band_boundary`'s monotonicity guard), and
/// `unwrap_seam_crossing_trim` because there are two loops — so it fell through
/// to `bridge_holes` on the RAW seamed chains, where consecutive ribbon samples
/// leap a whole period and the ear clip sprays across the domain.
///
/// A seam cut cannot fix it: one meridian crosses the arm ~16 times, so the
/// seam-cut rectangle is not a simple polygon. But the region is topologically
/// an ANNULUS (two boundary components), and an annulus becomes a DISC when cut
/// along a single arc joining its two boundaries. Cut from the ribbon's extreme
/// vertex (the one nearest the rim) straight to the nearest rim vertex: below
/// that extreme level the band is boundary-free, so the cut cannot meet either
/// loop except at its endpoints, and both endpoints are real edge samples (no
/// T-junction against a neighbour). The disc then lifts into the periodic cover
/// as ONE simple polygon — ribbon traversed once, cut, rim traversed once, cut
/// back — which the shared ear-clip path meshes. The two loops' OWN directions
/// chain up exactly (windings +1 and −1), which is what closes the polygon.
///
/// Returns None unless the wall is closed in exactly ONE direction, has exactly
/// two full-wrap loops of opposite winding, one of them a constant-level rim,
/// the rim is strictly clear of the ribbon's parameter range, and the lifted
/// polygon is verified simple — anything else keeps its current path.
pub(in crate::watertight_tessellation) fn close_periodic_spiral_annulus(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
) -> Result<Option<Vec<FaceVertex>>, String> {
    // Why a candidate was turned away (BREP_DEBUG_TESS_BAND): these guards are
    // the difference between meshing a thread runout and spraying it.
    let reject = |reason: &str| -> Result<Option<Vec<FaceVertex>>, String> {
        if std::env::var("BREP_DEBUG_TESS_BAND").is_ok() {
            eprintln!("face {}: not a spiral annulus ({reason})", face.id);
        }
        Ok(None)
    };
    if closed_u == closed_v || chains.len() != 2 {
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
    let coord = |vertex: &FaceVertex| -> (f64, f64) {
        if p_is_u {
            (vertex.uv[0], vertex.uv[1])
        } else {
            (vertex.uv[1], vertex.uv[0])
        }
    };
    // Net winding and cross-level range of each loop.
    struct LoopShape {
        winding: i32,
        qmin: f64,
        qmax: f64,
    }
    let mut shapes = Vec::with_capacity(2);
    for chain in chains {
        if chain.len() < 2 {
            return Ok(None);
        }
        let mut net = 0.0;
        let (mut qmin, mut qmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for index in 0..chain.len() {
            let (raw, q) = coord(&chain[index]);
            qmin = qmin.min(q);
            qmax = qmax.max(q);
            let next = coord(&chain[(index + 1) % chain.len()]).0;
            let mut delta = next - raw;
            if delta > 0.5 * period {
                delta -= period;
            } else if delta < -0.5 * period {
                delta += period;
            }
            net += delta;
        }
        let turns = net / period;
        if (turns.abs() - 1.0).abs() > 0.25 {
            return reject(&format!(
                "loop winding {turns:.3} is not a single full wrap"
            ));
        }
        shapes.push(LoopShape {
            winding: if turns > 0.0 { 1 } else { -1 },
            qmin,
            qmax,
        });
    }
    if shapes[0].winding == shapes[1].winding {
        return reject("both loops wind the same way (not an annulus)");
    }
    // Exactly one loop is a constant-level rim; the other is the ribbon.
    let flat = |shape: &LoopShape| (shape.qmax - shape.qmin) <= 0.05 * q_extent;
    let rim_index = match (flat(&shapes[0]), flat(&shapes[1])) {
        (true, false) => 0,
        (false, true) => 1,
        _ => return reject("need exactly one constant-level rim loop"),
    };
    let ribbon_index = 1 - rim_index;
    let (rim, ribbon) = (&chains[rim_index], &chains[ribbon_index]);
    // The rim must clear the ribbon's whole cross range, so the band between
    // them carries no boundary and the cut through it is unobstructed.
    let margin = 1e-9 * q_extent;
    let rim_below = shapes[rim_index].qmax + margin < shapes[ribbon_index].qmin;
    let rim_above = shapes[rim_index].qmin - margin > shapes[ribbon_index].qmax;
    if !rim_below && !rim_above {
        return reject(&format!(
            "rim q [{:.5},{:.5}] overlaps ribbon q [{:.5},{:.5}]",
            shapes[rim_index].qmin,
            shapes[rim_index].qmax,
            shapes[ribbon_index].qmin,
            shapes[ribbon_index].qmax
        ));
    }
    // Cut from the ribbon vertex nearest the rim — a STRICT extremum, so the
    // straight cut stays strictly between the two loops.
    let better = |a: f64, b: f64| if rim_below { a < b } else { a > b };
    let mut cut_index = 0;
    for index in 1..ribbon.len() {
        if better(coord(&ribbon[index]).1, coord(&ribbon[cut_index]).1) {
            cut_index = index;
        }
    }
    // A tie is normal — the ribbon's turnaround runs flat at the extreme level
    // (the thread's bottom turn). Any of the tied vertices serves: the cut's
    // interior lies strictly between the two loops' levels, so it cannot meet
    // the flat run either.
    let cut_level = coord(&ribbon[cut_index]).1;
    // Lift the ribbon from the cut, then the rim from where the ribbon lands
    // (their opposite windings make the two traversals close the polygon).
    let cut_p = coord(&ribbon[cut_index]).0;
    let ribbon_lifted = lift_loop_cycle(ribbon, cut_index, p_is_u, period, cut_p);
    let landing = coord(ribbon_lifted.last().unwrap()).0;
    // Rim vertex closest to the landing meridian keeps the cut short.
    let mut rim_index_start = 0;
    let mut best = f64::INFINITY;
    for (index, vertex) in rim.iter().enumerate() {
        let raw = coord(vertex).0;
        let offset = ((landing - raw) / period).round() * period;
        let distance = (raw + offset - landing).abs();
        if distance < best {
            best = distance;
            rim_index_start = index;
        }
    }
    let rim_raw = coord(&rim[rim_index_start]).0;
    let rim_start = rim_raw + ((landing - rim_raw) / period).round() * period;
    let rim_lifted = lift_loop_cycle(rim, rim_index_start, p_is_u, period, rim_start);
    let mut polygon = ribbon_lifted;
    polygon.extend(rim_lifted);
    if polygon.len() < 3 {
        return Ok(None);
    }
    if !polygon_is_simple(&polygon) {
        return reject("lifted polygon self-intersects");
    }
    if std::env::var("BREP_DEBUG_TESS_BAND").is_ok() {
        eprintln!(
            "face {}: spiral annulus, ribbon {} coedge samples over {:.2} periods, rim {} samples \
             {}, cut at ({:.4},{:.4})",
            face.id,
            ribbon.len(),
            {
                let ps: Vec<f64> = polygon.iter().map(|v| coord(v).0).collect();
                (ps.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
                    - ps.iter().cloned().fold(f64::INFINITY, f64::min))
                    / period
            },
            rim.len(),
            if rim_below { "below" } else { "above" },
            cut_p,
            cut_level,
        );
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }
    Ok(Some(polygon))
}
