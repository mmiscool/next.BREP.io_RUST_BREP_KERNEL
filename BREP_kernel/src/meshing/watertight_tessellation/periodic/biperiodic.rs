use super::*;

/// Build a seam-cut polygon for a doubly periodic rim-and-cut band recognized
/// by [`crate::topology::analyze_doubly_periodic_seam_band`]. Lift the rim to
/// the material-side closure level while retaining its 3D seam positions.
/// Return `None` for other faces or a nonsimple assembled polygon.
pub(in crate::watertight_tessellation) fn close_biperiodic_seam_band(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<Vec<FaceVertex>>, String> {
    if !(closed_u && closed_v) || chains.len() != 2 {
        return Ok(None);
    }
    let loops_uv: Vec<Vec<[f64; 2]>> = chains
        .iter()
        .map(|chain| chain.iter().map(|vertex| vertex.uv).collect())
        .collect();
    let Some(band) =
        crate::topology::analyze_doubly_periodic_seam_band(&loops_uv, domains, face.same_sense)
    else {
        return Ok(None);
    };
    let [u0, u1, v0, v1] = domains;
    let (p0, p1) = if band.p_is_u { (u0, u1) } else { (v0, v1) };
    let coord = |vertex: &FaceVertex| -> (f64, f64) {
        if band.p_is_u {
            (vertex.uv[0], vertex.uv[1])
        } else {
            (vertex.uv[1], vertex.uv[0])
        }
    };
    let mk_uv = |p: f64, q: f64| if band.p_is_u { [p, q] } else { [q, p] };
    let boundary = |chain: &[FaceVertex], q_offset: f64| -> Vec<FaceVertex> {
        let mut points = rotate_rim_to_seam(chain, band.p_is_u);
        for point in &mut points {
            point.uv[usize::from(band.p_is_u)] += q_offset;
        }
        complete_rim_seam(&mut points, [p0, p1], band.p_is_u);
        points
    };
    // Cut boundary keeps its level; the rim is shifted to the flat closure level
    // that bounds the material band (the material-side extreme for the classic
    // rim-at-extreme band; the rim's own level — offset 0 — for the
    // in-domain-rim variant).
    let q_flat = band.flat_level;
    let rim_offset = q_flat - band.rim_level;
    let cut = boundary(&chains[band.cut_index], 0.0);
    let rim = boundary(&chains[band.rim_index], rim_offset);
    // Lower / upper cross boundary of the band.
    let (lo, hi) = if band.material_above_cut {
        (&cut, &rim)
    } else {
        (&rim, &cut)
    };
    // Shared seam meridian (p0 ≡ p1): the band's cross span at the seam. Both
    // boundaries are continuous across the seam, so their q at the seam is their
    // endpoint q; the band is in-domain so no q-wrap.
    let q_lo_seam = coord(&lo[0]).1;
    let q_hi_seam = coord(&hi[0]).1;
    let seam = seam_meridian_samples(
        face,
        p1,
        band.p_is_u,
        q_lo_seam,
        q_hi_seam,
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
    Ok(Some(polygon))
}

/// Rebuild the trim polygon + interior holes for a DOUBLY-periodic (torus) face
/// whose material region is a full-wrap BAND bounded by two full-wrap
/// constant-cross-level RIMS and pierced by a hole that STRADDLES the periodic
/// seam. This is the biperiodic union of [`close_periodic_trim_dir`] (the
/// two-rim seam-cut rectangle + cross-seam COMPLEMENT lift) and
/// [`seam_straddle_wall_polygons`] (a seam-straddling hole split into the two
/// seam columns).
///
/// The flagship case is the ABC signet-ring torus band (00002621/00002624
/// face 15): a ~3 %-of-torus strip that wraps fully in v, straddles the u-seam
/// (its material is the cross-seam complement of the between-rims band), and is
/// pierced by a small bore that straddles BOTH seams. `close_periodic_trim`
/// bails the instant a loop reaches the p-seam (its hole-straddle guard), and
/// the biperiodic-unwrap path lays a self-touching seam cover the CDT declines,
/// so the face fell to `bridge_holes` on the seamed chains and folded inside-out
/// (~7.5k double-covered triangles).
///
/// Cut at the private p-seam (the wall wraps onto itself, so p0 ≡ p1 in 3D);
/// lift the complement band a full cross-period across the cross seam (its cross
/// parameters then run past the domain, evaluated with the domain-wrapping
/// evaluator); q-unwrap and split each straddle hole at the p-seam and weave its
/// two arcs into the p0/p1 seam columns with 3D-coincident far crossings (the
/// weld invariant), so the covering-plane region is a SIMPLE PSLG `surface_cdt`
/// owns — constraint-preserving and CCW-in-uv, hence watertight and not
/// inside-out. Ordinary non-straddling interior holes are returned as holes.
///
/// Returns None (defer to the existing path, bit-for-bit) unless the face is
/// exactly two clean opposite-sense full-wrap rims plus ≥ 1 p-seam-straddling
/// hole that splits cleanly into one right + one left arc, and the assembled
/// polygon is simple.
pub(in crate::watertight_tessellation) fn close_biperiodic_straddle_band(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<(Vec<FaceVertex>, Vec<Vec<FaceVertex>>)>, String> {
    if !(closed_u && closed_v) || chains.len() < 3 {
        return Ok(None);
    }
    let [u0, u1, v0, v1] = domains;
    let dbg = std::env::var("BREP_DEBUG_BISTRADDLE").is_ok();
    for p_is_u in [true, false] {
        let (p0, p1) = if p_is_u { (u0, u1) } else { (v0, v1) };
        let (q0, q1) = if p_is_u { (v0, v1) } else { (u0, u1) };
        let period = p1 - p0;
        let q_period = q1 - q0;
        if !(period > 0.0) || !(q_period > 0.0) {
            continue;
        }
        let q_extent = q_period.abs().max(1e-30);
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
        // Classify loops: full-wrap rims (span the whole period at a near-constant
        // cross level, net winding ±period) vs holes. A hole whose p-span exceeds
        // half a period with net winding ~0 STRADDLES the p-seam (woven into the
        // seam columns); anything strictly interior in both directions is an
        // ordinary hole; anything else leaves the shape ambiguous → defer.
        let mut rim_idx: Vec<usize> = Vec::new();
        let mut rim_dir: Vec<i32> = Vec::new();
        let mut rim_level: Vec<f64> = Vec::new();
        let mut straddle_holes: Vec<Vec<FaceVertex>> = Vec::new();
        let mut interior_holes: Vec<Vec<FaceVertex>> = Vec::new();
        let mut bail = false;
        for (idx, chain) in chains.iter().enumerate() {
            if chain.len() < 2 {
                bail = true;
                break;
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
            if (pmax - pmin) >= 0.6 * period && (qmax - qmin) <= 0.05 * q_extent && direction != 0 {
                rim_idx.push(idx);
                rim_dir.push(direction);
                rim_level.push(0.5 * (qmin + qmax));
            } else if (pmax - pmin) > 0.5 * period && direction == 0 {
                straddle_holes.push(chain.clone());
            } else if pmin > p0 + 1e-6 * period
                && pmax < p1 - 1e-6 * period
                && qmin > q0 + 1e-6 * q_extent
                && qmax < q1 - 1e-6 * q_extent
            {
                interior_holes.push(chain.clone());
            } else {
                bail = true;
                break;
            }
        }
        if bail || rim_idx.len() != 2 || straddle_holes.is_empty() {
            continue;
        }
        // Which of the two regions the rims bound is material, and the cross-seam
        // COMPLEMENT lift — read exactly as `close_periodic_trim_dir`: the material
        // is CCW in (u,v) iff `same_sense`, so the between-rims band is CCW-in-uv
        // when the lower rim runs +p (for p=u) / −p (for p=v). An inconclusive
        // orientation (a zero or equal rim sense) is deferred, never guessed.
        let (mut lo_i, mut hi_i) = (0usize, 1usize);
        if rim_level[lo_i] > rim_level[hi_i] {
            std::mem::swap(&mut lo_i, &mut hi_i);
        }
        let lower_direction = rim_dir[lo_i];
        let upper_direction = rim_dir[hi_i];
        if lower_direction == 0 || upper_direction == 0 || lower_direction == upper_direction {
            continue;
        }
        let between_rims_is_ccw_uv = if p_is_u {
            lower_direction > 0
        } else {
            lower_direction < 0
        };
        let wraps_cross_seam = between_rims_is_ccw_uv != face.same_sense;
        // Band cross range: between-rims keeps the rim levels; the complement lifts
        // the LOWER rim a full cross period so the band runs upward from the upper
        // rim, across the cross seam, to the lifted lower rim.
        let (bottom_i, bottom_off, top_i, top_off, q_lo, q_hi) = if wraps_cross_seam {
            (
                rim_idx[hi_i],
                0.0,
                rim_idx[lo_i],
                q_period,
                rim_level[hi_i],
                rim_level[lo_i] + q_period,
            )
        } else {
            (
                rim_idx[lo_i],
                0.0,
                rim_idx[hi_i],
                0.0,
                rim_level[lo_i],
                rim_level[hi_i],
            )
        };
        if dbg {
            eprintln!(
                "[bistraddle] face {}: p_is_u={p_is_u} rims={} straddle={} interior={} \
                 senses=({lower_direction:+},{upper_direction:+}) same_sense={} \
                 complement={wraps_cross_seam} q=[{q_lo:.4},{q_hi:.4}]",
                face.id,
                rim_idx.len(),
                straddle_holes.len(),
                interior_holes.len(),
                face.same_sense
            );
        }
        let rebuild_rim = |chain: &[FaceVertex], q_offset: f64| -> Vec<FaceVertex> {
            let mut points = rotate_rim_to_seam(chain, p_is_u);
            for point in &mut points {
                point.uv[usize::from(p_is_u)] += q_offset;
            }
            complete_rim_seam(&mut points, [p0, p1], p_is_u);
            points
        };
        let bottom = rebuild_rim(&chains[bottom_i], bottom_off);
        let mut top = rebuild_rim(&chains[top_i], top_off);

        let q_center = 0.5 * (q_lo + q_hi);
        let mut right_arcs: Vec<Vec<FaceVertex>> = Vec::new();
        let mut left_arcs: Vec<Vec<FaceVertex>> = Vec::new();
        for hole in &straddle_holes {
            // q-unwrap each vertex into the band's cross range so a hole that also
            // crosses the CROSS seam (the bore at the u-seam corner) is contiguous
            // before it is split at the p-seam.
            let unwrapped: Vec<FaceVertex> = hole
                .iter()
                .map(|vertex| {
                    let (p, q) = coord(vertex);
                    let shift = ((q_center - q) / q_period).round() * q_period;
                    FaceVertex {
                        uv: mk(p, q + shift),
                        position: vertex.position,
                    }
                })
                .collect();
            let Some((right, left)) = super::rim::split_seam_arcs(&unwrapped, [p0, p1], p_is_u) else {
                if dbg {
                    eprintln!("[bistraddle] face {}: split_arcs failed", face.id);
                }
                return Ok(None);
            };
            right_arcs.push(right);
            left_arcs.push(left);
        }
        // Build one seam column (p = `p_seam`) ascending in q over [q_lo, q_hi],
        // excluding the q_lo/q_hi corners (they come from the rims) and splicing the
        // straddle arcs into their q-ranges. `wrap_q` is true — the complement band
        // sweeps q past the domain end.
        let build_seam = |p_seam: f64, arcs: &[Vec<FaceVertex>]| -> Result<Vec<FaceVertex>, String> {
            let base = seam_meridian_samples(face, p_seam, p_is_u, q_lo, q_hi, chord_tolerance, true)?;
            let mut ranges: Vec<(f64, f64, usize)> = arcs
                .iter()
                .enumerate()
                .map(|(index, arc)| {
                    let qa = coord(&arc[0]).1;
                    let qb = coord(arc.last().unwrap()).1;
                    (qa.min(qb), qa.max(qb), index)
                })
                .collect();
            ranges.sort_by(|a, b| a.0.total_cmp(&b.0));
            let border_eps = 1e-9 * (q_hi - q_lo).abs().max(1.0);
            let mut column: Vec<FaceVertex> = Vec::new();
            let mut cursor = q_lo;
            for &(range_lo, range_hi, arc_index) in &ranges {
                for &(q, position) in &base {
                    if q > cursor + border_eps && q > q_lo + border_eps && q < range_lo - border_eps {
                        column.push(FaceVertex {
                            uv: mk(p_seam, q),
                            position,
                        });
                    }
                }
                for vertex in &arcs[arc_index] {
                    column.push(FaceVertex {
                        uv: mk(p_seam, coord(vertex).1),
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
        top.reverse();
        let mut polygon: Vec<FaceVertex> = Vec::new();
        polygon.extend(bottom);
        polygon.extend(right_seam);
        polygon.extend(top);
        polygon.extend(left_seam);
        if polygon.len() < 3 || !polygon_is_simple(&polygon) {
            if dbg {
                eprintln!(
                    "[bistraddle] face {}: assembled polygon len={} simple={}",
                    face.id,
                    polygon.len(),
                    polygon_is_simple(&polygon)
                );
            }
            continue;
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
        return Ok(Some((polygon, holes)));
    }
    Ok(None)
}
