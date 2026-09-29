use super::*;

/// Build a seam-cut polygon for a singly periodic wall bounded by two full-wrap
/// rims, or one rim and a collapsed pole. At least one rim must vary in the
/// cross direction; constant-rim cases belong to the simpler band paths.
///
/// Rotate wrap boundaries at their seam jump to preserve local reversals, and
/// close both seam columns with the same sampled meridian so their 3D positions
/// weld. Interior holes pass through; seam-straddling holes are deferred.
/// Only return a simple polygon accepted by the CDT, avoiding a self-touching
/// boundary that could trigger the ear-clipping fallback.
/// `BREP_NO_PERIODIC_SEAMCUT` disables this path.
pub(in crate::watertight_tessellation) fn close_periodic_wrap_wall(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<(Vec<FaceVertex>, Vec<Vec<FaceVertex>>)>, String> {
    if closed_u == closed_v || chains.len() < 2 {
        return Ok(None);
    }
    if std::env::var("BREP_NO_PERIODIC_SEAMCUT").is_ok() {
        return Ok(None);
    }
    let [u0, u1, v0, v1] = domains;
    let dbg = std::env::var("BREP_DEBUG_WRAPWALL").is_ok();
    let p_is_u = closed_u;
    let (p0, p1) = if p_is_u { (u0, u1) } else { (v0, v1) };
    let (q0, q1) = if p_is_u { (v0, v1) } else { (u0, u1) };
    let period = p1 - p0;
    let q_extent = (q1 - q0).abs().max(1e-30);
    if !(period > 0.0) {
        return Ok(None);
    }
    let coord = |vertex: &FaceVertex| -> (f64, f64) {
        if p_is_u {
            (vertex.uv[0], vertex.uv[1])
        } else {
            (vertex.uv[1], vertex.uv[0])
        }
    };
    let mk = |p: f64, q: f64| -> [f64; 2] {
        if p_is_u {
            [p, q]
        } else {
            [q, p]
        }
    };
    // A wrap boundary rebuilt ascending in p (weave preserved), or a pole
    // collapsed to a two-vertex edge at its cross extreme.
    struct Boundary {
        pts: Vec<FaceVertex>,
        seam_q: f64,
        weaving: bool,
    }
    let mut boundaries: Vec<Boundary> = Vec::new();
    let mut interior_holes: Vec<Vec<FaceVertex>> = Vec::new();
    for chain in chains {
        // An empty loop is nothing; a single-sample loop is a degenerate
        // VERTEX_LOOP (a pole/tangency point) and flows into the pole branch below
        // (its 3D extent is zero).
        if chain.is_empty() {
            continue;
        }
        let (mut pmin, mut pmax, mut qmin, mut qmax) =
            (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for vertex in chain {
            let (p, q) = coord(vertex);
            pmin = pmin.min(p);
            pmax = pmax.max(p);
            qmin = qmin.min(q);
            qmax = qmax.max(q);
        }
        let n = chain.len();
        let mut net = 0.0;
        for i in 0..n {
            let mut delta = coord(&chain[(i + 1) % n]).0 - coord(&chain[i]).0;
            if delta > 0.5 * period {
                delta -= period;
            } else if delta < -0.5 * period {
                delta += period;
            }
            net += delta;
        }
        let direction = if net > 0.25 * period {
            1
        } else if net < -0.25 * period {
            -1
        } else {
            0
        };
        let extent = chain_extent(chain);
        let q_level = 0.5 * (qmin + qmax);
        let on_q_extreme = (q_level - q0).abs() <= 0.02 * q_extent
            || (q_level - q1).abs() <= 0.02 * q_extent;
        if dbg {
            eprintln!(
                "[wrapwall] face {}: chain n={n} p[{pmin:.4},{pmax:.4}] q[{qmin:.4},{qmax:.4}] net={net:.4} dir={direction} ext={extent:.6} on_extreme={on_q_extreme}",
                face.id
            );
        }
        if extent <= chord_tolerance {
            // A collapsed loop: a pole at a cross extreme is one boundary of the
            // cap; a collapsed puncture in the interior is not a material hole.
            // Anything collapsed but NOT on an extreme is ambiguous → defer.
            if !on_q_extreme {
                if dbg {
                    eprintln!("[wrapwall] face {}: bail collapsed loop not on extreme", face.id);
                }
                return Ok(None);
            }
            // Confirm the whole q-iso line truly collapses to the pole point, so
            // the seam column's endpoint at this level really is the apex.
            let pole = chain[0].position;
            for f in [0.0, 0.5, 1.0] {
                let p = p0 + period * f;
                let at = if p_is_u {
                    face.surface.evaluate(p, q_level)?
                } else {
                    face.surface.evaluate(q_level, p)?
                };
                if at.sub(pole).length() > chord_tolerance.max(1e-9) {
                    if dbg {
                        eprintln!("[wrapwall] face {}: bail pole iso not collapsed (d={:.6})", face.id, at.sub(pole).length());
                    }
                    return Ok(None);
                }
            }
            boundaries.push(Boundary {
                pts: vec![
                    FaceVertex { uv: mk(p0, q_level), position: pole },
                    FaceVertex { uv: mk(p1, q_level), position: pole },
                ],
                seam_q: q_level,
                weaving: false,
            });
        } else if direction != 0 {
            let mut pts = rotate_rim_to_seam(chain, p_is_u);
            let front_q = coord(&pts[0]).1;
            let back_q = coord(pts.last().unwrap()).1;
            complete_rim_seam(&mut pts, [p0, p1], p_is_u);
            let weaving = (qmax - qmin) > 0.05 * q_extent;
            boundaries.push(Boundary {
                pts,
                seam_q: 0.5 * (front_q + back_q),
                weaving,
            });
        } else if pmin > p0 + 1e-6 * period && pmax < p1 - 1e-6 * period {
            interior_holes.push(chain.clone());
        } else {
            // A seam-reaching non-wrap loop (a straddle hole / half-wrap trim) is
            // owned by another path (or, for a straddle hole, needs the arc-splice
            // that this weaving-boundary assembly cannot yet keep touch-free) —
            // defer rather than guess.
            if dbg {
                eprintln!("[wrapwall] face {}: bail seam-reaching non-wrap loop", face.id);
            }
            return Ok(None);
        }
    }
    // Exactly two cross-boundaries bound the cap/band, and at least one must weave
    // (the fold signature; a clean two-rim band is `close_periodic_trim`'s and is
    // tried before this path).
    if boundaries.len() != 2 || !boundaries.iter().any(|b| b.weaving) {
        if dbg {
            eprintln!(
                "[wrapwall] face {}: bail boundaries={} weaving={} holes={}",
                face.id,
                boundaries.len(),
                boundaries.iter().filter(|b| b.weaving).count(),
                interior_holes.len()
            );
        }
        return Ok(None);
    }
    // Bottom = lower seam level, top = higher, so the shared meridian ascends.
    if boundaries[0].seam_q > boundaries[1].seam_q {
        boundaries.swap(0, 1);
    }
    let (bottom, top) = (&boundaries[0], &boundaries[1]);
    let q_lo = bottom.seam_q;
    let q_hi = top.seam_q;
    if (q_hi - q_lo) <= 1e-6 * q_extent {
        return Ok(None);
    }
    // One shared seam meridian (p0 ≡ p1 → identical positions on both columns, so
    // the two sides weld). In-domain: q spans two in-domain cross levels.
    let seam = seam_meridian_samples(face, p1, p_is_u, q_lo, q_hi, chord_tolerance, false)?;
    let mut polygon: Vec<FaceVertex> = Vec::new();
    polygon.extend(bottom.pts.iter().copied());
    if seam.len() > 2 {
        for &(q, position) in &seam[1..seam.len() - 1] {
            polygon.push(FaceVertex {
                uv: mk(p1, q),
                position,
            });
        }
    }
    let mut top_rev = top.pts.clone();
    top_rev.reverse();
    polygon.extend(top_rev);
    if seam.len() > 2 {
        for &(q, position) in seam[1..seam.len() - 1].iter().rev() {
            polygon.push(FaceVertex {
                uv: mk(p0, q),
                position,
            });
        }
    }
    if polygon.len() < 3 || !polygon_is_simple(&polygon) {
        if dbg {
            eprintln!(
                "[wrapwall] face {}: assembled polygon len={} simple={}",
                face.id,
                polygon.len(),
                polygon_is_simple(&polygon)
            );
        }
        return Ok(None);
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }
    let holes = interior_holes
        .into_iter()
        .map(|mut hole| {
            if signed_area(&hole) > 0.0 {
                hole.reverse();
            }
            hole
        })
        .collect::<Vec<_>>();
    // Fire ONLY when the assembled region is a valid PSLG the metric CDT will own
    // (the same check `surface_cdt` runs): `polygon_is_simple` catches proper
    // crossings but not seam self-TOUCHES. This guarantees the wrap-wall path never
    // routes a polygon the CDT declines into the ear-clip fallback (which would
    // fold on a residual self-touch); such a face defers to its existing path,
    // bit-for-bit. Correct cases (a clean weaving rim + rim/pole, ± interior holes)
    // always pass.
    if !valid_planar_pslg(&polygon, &holes) {
        if dbg {
            eprintln!("[wrapwall] face {}: bail not a valid PSLG", face.id);
        }
        return Ok(None);
    }
    if dbg {
        eprintln!(
            "[wrapwall] face {}: OK outer={} holes={} q=[{q_lo:.4},{q_hi:.4}]",
            face.id,
            polygon.len(),
            holes.len()
        );
    }
    Ok(Some((polygon, holes)))
}

/// Rebuild the trim polygon for a periodic face whose material region is a
/// full-wrap BAND between two full-wrap boundaries, where a boundary is
/// CASTELLATED — locally multi-valued in the periodic parameter because it dives
/// toward the opposite rim to open a window, then rises to a spoke (a spoked /
/// gear web, ABC 00010564 sprocket faces 13/16: a torus dish whose outer wire
/// weaves around six spoke windows). Every existing periodic helper rejects it:
/// `close_periodic_trim` needs a near-constant-level rim, `close_biperiodic_
/// seam_band` / `close_periodic_band` need a SINGLE-VALUED wavy cut, and
/// `unwrap_seam_crossing_trim` needs a non-full-wrap straddle. So the face falls
/// to `bridge_holes` on the RAW seamed chains, which teleports a whole period at
/// the seam and sprays a domain-filling membrane over the windows — rendering
/// them closed (inside-out red) instead of open.
///
/// Both boundaries ride the private p-seam (each is a full wrap, net winding
/// ±period), so cutting at the seam (p0 ≡ p1) keeps every boundary vertex
/// authoritative. Each boundary is rebuilt ascending in p by ROTATING at its
/// full-period seam jump — NEVER sorting, which would interleave the
/// castellation's local reversals into a self-crossing polygon — with the
/// missing seam corner filled from the surviving endpoint on the other seam
/// side. The between-boundaries band is the material region; the assembled
/// rectangle-with-castellated-edge is meshed by the shared seed+refine path
/// exactly like `close_periodic_trim`'s band, leaving the window notches OUTSIDE
/// the boundary (open). Runs only after every clean-band helper has declined, and
/// returns None unless the face is exactly two full-wrap loops (≥ one castellated)
/// forming a simple in-domain between-rims band — every other face keeps its path.
pub(in crate::watertight_tessellation) fn close_periodic_wavy_band(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<(Vec<FaceVertex>, bool)>, String> {
    if chains.len() != 2 {
        return Ok(None);
    }
    for p_is_u in [true, false] {
        if p_is_u && !closed_u {
            continue;
        }
        if !p_is_u && !closed_v {
            continue;
        }
        let q_closed = if p_is_u { closed_v } else { closed_u };
        if let Some(result) =
            close_periodic_wavy_band_dir(face, chains, domains, p_is_u, q_closed, chord_tolerance)?
        {
            return Ok(Some(result));
        }
    }
    Ok(None)
}

/// One-direction attempt for [`close_periodic_wavy_band`]: `p_is_u` is the
/// wrapping (periodic) parameter, `q` the cross parameter.
fn close_periodic_wavy_band_dir(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    p_is_u: bool,
    q_closed: bool,
    chord_tolerance: f64,
) -> Result<Option<(Vec<FaceVertex>, bool)>, String> {
    let [u0, u1, v0, v1] = domains;
    let (p0, p1) = if p_is_u { (u0, u1) } else { (v0, v1) };
    let (q0, q1) = if p_is_u { (v0, v1) } else { (u0, u1) };
    let period = p1 - p0;
    let q_extent = (q1 - q0).abs().max(1e-30);
    if !(period > 0.0) {
        return Ok(None);
    }
    let coord = |vertex: &FaceVertex| -> (f64, f64) {
        if p_is_u {
            (vertex.uv[0], vertex.uv[1])
        } else {
            (vertex.uv[1], vertex.uv[0])
        }
    };
    let mk_uv = |p: f64, q: f64| if p_is_u { [p, q] } else { [q, p] };
    // Rebuild a full-wrap loop as an ascending-p boundary by rotating at its seam
    // jump (preserving any castellation), with seam corners inserted at p0/p1.
    // Returns (points, net_direction, qmin, qmax) or None when the loop is not a
    // genuine full wrap (a partial straddle or an interior hole).
    let build = |chain: &[FaceVertex]| -> Option<(Vec<FaceVertex>, i32, f64, f64)> {
        let n = chain.len();
        if n < 2 {
            return None;
        }
        // Net winding along p, seam jumps unwrapped (closing edge included).
        let mut net = 0.0;
        for i in 0..n {
            let mut delta = coord(&chain[(i + 1) % n]).0 - coord(&chain[i]).0;
            if delta > 0.5 * period {
                delta -= period;
            } else if delta < -0.5 * period {
                delta += period;
            }
            net += delta;
        }
        let direction = if net > 0.25 * period {
            1
        } else if net < -0.25 * period {
            -1
        } else {
            return None; // not a full wrap
        };
        // Seam = the consecutive pair (closing edge included) with the largest raw
        // |p jump|; it must be a genuine near-full-period hop.
        let (mut seam, mut worst) = (0usize, -1.0);
        for i in 0..n {
            let jump = (coord(&chain[(i + 1) % n]).0 - coord(&chain[i]).0).abs();
            if jump > worst {
                worst = jump;
                seam = (i + 1) % n;
            }
        }
        if worst < 0.5 * period {
            return None;
        }
        let mut pts: Vec<FaceVertex> = (0..n).map(|k| chain[(seam + k) % n]).collect();
        if coord(&pts[0]).0 > coord(pts.last().unwrap()).0 {
            pts.reverse();
        }
        let eps = 1e-6 * period;
        let front_position = pts[0].position;
        let back_position = pts.last().unwrap().position;
        let front_q = coord(&pts[0]).1;
        let back_q = coord(pts.last().unwrap()).1;
        if coord(pts.last().unwrap()).0 < p1 - eps {
            pts.push(FaceVertex {
                uv: mk_uv(p1, back_q),
                position: front_position,
            });
        }
        if coord(&pts[0]).0 > p0 + eps {
            pts.insert(
                0,
                FaceVertex {
                    uv: mk_uv(p0, front_q),
                    position: back_position,
                },
            );
        }
        let (mut qmin, mut qmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for vertex in &pts {
            let q = coord(vertex).1;
            qmin = qmin.min(q);
            qmax = qmax.max(q);
        }
        Some((pts, direction, qmin, qmax))
    };
    let (Some((b0, dir0, q0min, q0max)), Some((b1, dir1, q1min, q1max))) =
        (build(&chains[0]), build(&chains[1]))
    else {
        return Ok(None);
    };
    // At least one boundary must be castellated (varies in q beyond the flat-rim
    // tolerance); two flat rims are `close_periodic_trim`'s case, handled earlier.
    let flat0 = (q0max - q0min) <= 0.05 * q_extent;
    let flat1 = (q1max - q1min) <= 0.05 * q_extent;
    if flat0 && flat1 {
        return Ok(None);
    }
    // Lower / upper boundary by their cross level at the seam (each boundary's
    // value at p0 — where the band's seam edge sits).
    let seam_q0 = coord(&b0[0]).1;
    let seam_q1 = coord(&b1[0]).1;
    let (lo, hi, lo_seam_q, hi_seam_q, lower_dir, upper_dir) = if seam_q0 <= seam_q1 {
        (&b0, &b1, seam_q0, seam_q1, dir0, dir1)
    } else {
        (&b1, &b0, seam_q1, seam_q0, dir1, dir0)
    };
    // Material side: keep the between-rims band unless the cross direction wraps
    // AND the loop senses conclusively name the complement across the cross seam
    // (mirror `close_periodic_trim_dir`). The spoked-web material is always the
    // in-domain between-rims band, so decline the complement rather than lifting.
    let inconclusive = lower_dir == 0 || upper_dir == 0 || lower_dir == upper_dir;
    let between_rims_is_ccw_uv = if p_is_u { lower_dir > 0 } else { lower_dir < 0 };
    let wraps_cross_seam = q_closed && !inconclusive && between_rims_is_ccw_uv != face.same_sense;
    if wraps_cross_seam {
        return Ok(None);
    }
    // Shared seam meridian (p0 ≡ p1): the band's cross span at the seam, from the
    // lower boundary's seam level to the upper's, with authoritative 3D positions.
    let seam = seam_meridian_samples(
        face,
        p1,
        p_is_u,
        lo_seam_q,
        hi_seam_q,
        chord_tolerance,
        false,
    )?;
    // Assemble CCW in (p,q): bottom lo (p0→p1), right seam (q_lo→q_hi at p1), top
    // hi reversed (p1→p0), left seam (q_hi→q_lo at p0). Seam endpoints dropped —
    // corners come from the boundaries.
    let mut polygon: Vec<FaceVertex> = Vec::new();
    polygon.extend(lo.iter().copied());
    if seam.len() > 2 {
        for &(q, position) in &seam[1..seam.len() - 1] {
            polygon.push(FaceVertex {
                uv: mk_uv(p1, q),
                position,
            });
        }
    }
    let mut top = hi.clone();
    top.reverse();
    polygon.extend(top);
    if seam.len() > 2 {
        for &(q, position) in seam[1..seam.len() - 1].iter().rev() {
            polygon.push(FaceVertex {
                uv: mk_uv(p0, q),
                position,
            });
        }
    }
    if polygon.len() < 3 || !polygon_is_simple(&polygon) {
        return Ok(None);
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }
    Ok(Some((polygon, false)))
}
