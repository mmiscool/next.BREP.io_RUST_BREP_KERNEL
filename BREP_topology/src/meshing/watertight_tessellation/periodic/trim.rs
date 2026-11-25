use super::*;

/// Adaptive chord sampling of a seam meridian (`p` fixed, `q` sweeping
/// `q_lo..q_hi`) of a periodic surface. `p_is_u` selects which parameter wraps.
/// Endpoints are included; interior stations are added wherever the surface
/// chord sags past the tolerance, so curved-in-`q` walls (e.g. a sphere)
/// keep a faithful seam while a ruled wall stays two points.
pub(super) fn seam_meridian_samples(
    face: &FaceRecord,
    p_seam: f64,
    p_is_u: bool,
    q_lo: f64,
    q_hi: f64,
    chord_tolerance: f64,
    // A band whose material side crosses the CROSS seam sweeps `q` past the
    // domain end; wrap it back onto the surface. In-domain callers pass false
    // and keep the plain evaluator (bit-identical).
    wrap_q: bool,
) -> Result<Vec<(f64, Vec3)>, String> {
    let eval = |q: f64| -> Result<Vec3, String> {
        match (p_is_u, wrap_q) {
            (true, false) => face.surface.evaluate(p_seam, q),
            (true, true) => face.surface.evaluate_extended(p_seam, q),
            (false, false) => face.surface.evaluate(q, p_seam),
            (false, true) => face.surface.evaluate_extended(q, p_seam),
        }
    };
    let mut qs = vec![q_lo, q_hi];
    loop {
        let mut refined = Vec::with_capacity(qs.len() * 2);
        let mut split = false;
        for pair in qs.windows(2) {
            refined.push(pair[0]);
            let mid = (pair[0] + pair[1]) * 0.5;
            if deflection_to_chord(eval(mid)?, eval(pair[0])?, eval(pair[1])?) > chord_tolerance {
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
    qs.into_iter().map(|q| Ok((q, eval(q)?))).collect()
}

/// Mesh an otherwise untrimmed doubly-periodic face as a periodic tensor grid.
/// There are no duplicated seam vertices: the final cell in each direction
/// wraps to station zero, so the result is closed even though all four
/// parameter corners represent the same physical point.
pub(in crate::watertight_tessellation) fn tessellate_full_biperiodic_face(
    face: &FaceRecord,
    domain: [f64; 4],
    chord_tolerance: f64,
    face_id: u32,
    mesh: &mut Mesh,
) -> Result<(), String> {
    let [u0, u1, v0, v1] = domain;
    let mut u_stations = Vec::new();
    let mut v_stations = Vec::new();
    // Sample several cross sections so a non-uniform doubly-periodic carrier
    // cannot hide interior curvature that is absent at its seam.
    for index in 0..=4 {
        let fraction = index as f64 / 4.0;
        let v = v0 + (v1 - v0) * fraction;
        u_stations.extend(
            seam_meridian_samples(face, v, false, u0, u1, chord_tolerance, false)?
                .into_iter()
                .map(|(u, _)| u),
        );
        let u = u0 + (u1 - u0) * fraction;
        v_stations.extend(
            seam_meridian_samples(face, u, true, v0, v1, chord_tolerance, false)?
                .into_iter()
                .map(|(v, _)| v),
        );
    }
    let normalize = |stations: &mut Vec<f64>, span: f64| {
        stations.sort_by(|a, b| a.total_cmp(b));
        stations.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * span.abs().max(1.0));
        stations.pop(); // periodic endpoint is station zero
    };
    normalize(&mut u_stations, u1 - u0);
    normalize(&mut v_stations, v1 - v0);
    if u_stations.len() < 3 || v_stations.len() < 3 {
        return Err("watertight tessellation: under-sampled full periodic face".into());
    }
    let (nu, nv) = (u_stations.len(), v_stations.len());
    let base = (mesh.positions.len() / 3) as u32;
    for &u in &u_stations {
        for &v in &v_stations {
            let point = face.surface.evaluate(u, v)?;
            let normal = face_normal_at(face, [u, v], domain)?;
            mesh.positions.extend([point.x, point.y, point.z]);
            mesh.normals.extend([normal.x, normal.y, normal.z]);
        }
    }
    let index = |u: usize, v: usize| base + (u * nv + v) as u32;
    for u in 0..nu {
        for v in 0..nv {
            let a = index(u, v);
            let b = index((u + 1) % nu, v);
            let c = index((u + 1) % nu, (v + 1) % nv);
            let d = index(u, (v + 1) % nv);
            if face.same_sense {
                mesh.indices.extend([a, b, c, a, c, d]);
            } else {
                mesh.indices.extend([a, c, b, a, d, c]);
            }
            mesh.face_ids.extend([face_id, face_id]);
        }
    }
    Ok(())
}

/// One cross-level boundary of a periodic wall's trim rectangle.
#[derive(Clone)]
struct PeriodicRing {
    /// Cross-parameter level (`v` for a u-closed wall).
    level: f64,
    /// Boundary samples ordered by ascending periodic parameter. A pole ring
    /// carries a single collapsed sample; a rim ring carries the shared-edge
    /// samples with the wrap-closing corner appended.
    points: Vec<FaceVertex>,
    pole: bool,
    /// The loop's own chain, in traversal order (for a rim the sorted `points` lose it).
    raw: Vec<FaceVertex>,
    /// Traversal sense of the loop along the periodic parameter as the FACE
    /// walks it (+1 with increasing `p`, −1 against, 0 inconclusive). The
    /// material side of a rim is to its left, so this is what tells the two
    /// candidate bands of a bi-periodic surface apart.
    direction: i32,
}

/// Rebuild the outer trim polygon for a periodic (closed) wall whose loops are
/// expressed WITHOUT a seam ruling — a full-circle rim that spans the whole
/// period as an OPEN line in (u,v) plus, for a cone, a degenerate apex point.
///
/// Such loops never close in parameter space, so the plain outer/hole/bridge
/// builder triangulates a degenerate polygon (a line plus a point) and sprays
/// diameter-spanning triangles across the opening — the "filled disc"/cone-base
/// artifact. Reconstruct the trim as the full domain rectangle across the seam,
/// exactly the polygon a seamed cone/cylinder loop already produces, and hand
/// it to the shared ear-clip / refine path.
///
/// Returns `None` (keep the existing path) unless the face is closed in exactly
/// one direction and every loop is a clean full rim or a pole — so seamed
/// periodic faces, planar faces with holes, and tori are untouched. The two
/// seam runs are PRIVATE to the wall (it wraps onto itself): both sides sample
/// the same meridian (`p0 ≡ p1` on a closed surface) at one shared `q`-list, so
/// they weld crack-free, while the rim keeps its authoritative shared-edge
/// samples so the mesh stays watertight against the neighbouring cap.
///
/// The returned bool is `wraps_cross_seam`: true when the material band is the
/// one that crosses the CROSS direction's seam (only possible on a bi-periodic
/// surface — see [`close_periodic_trim_dir`]), whose polygon therefore carries
/// cross parameters past the domain end and must be meshed with the
/// domain-wrapping evaluator.
pub(in crate::watertight_tessellation) fn close_periodic_trim(
    face: &FaceRecord,
    chains: &[Vec<FaceVertex>],
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
    chord_tolerance: f64,
) -> Result<Option<(Vec<FaceVertex>, bool)>, String> {
    // A non-periodic face keeps the existing outer/hole path.
    if !closed_u && !closed_v {
        return Ok(None);
    }
    // Try to close the wall in each direction the SURFACE is periodic. A face
    // on a bi-periodic surface (torus) may still be a simple BAND wrapping in
    // exactly ONE direction — two constant-cross-level rims spanning the whole
    // period (onshape-unclamped-periodic faces 827/832: a fillet torus sliced
    // into v-bands). Those loops are open rim lines the outer/hole/bridge path
    // sprays across the opening, exactly like a seamless cylinder. The band
    // direction is whichever one makes every loop classify as a clean rim/pole;
    // a face wrapping BOTH ways (a full torus patch) fails both and falls back.
    for p_is_u in [true, false] {
        if p_is_u && !closed_u {
            continue;
        }
        if !p_is_u && !closed_v {
            continue;
        }
        let q_closed = if p_is_u { closed_v } else { closed_u };
        if let Some(result) =
            close_periodic_trim_dir(face, chains, domains, p_is_u, q_closed, chord_tolerance)?
        {
            return Ok(Some(result));
        }
    }
    Ok(None)
}

/// `BREP_TESS_UNALIGNED_RIMS` (default on, `=0` restores the seam-cut rectangle for these walls).
fn unaligned_rims_enabled() -> bool {
    crate::switch!("BREP_TESS_UNALIGNED_RIMS", true)
}

/// A wall between two full rims whose pcurves run past the periodic domain (a rim given on
/// `[-0.34, 0.66]`, its partner on `[0.12, 1.12]`): neither rim has a sample on the seam, so the
/// seam-cut rectangle, which sorts each rim into `[p0, p1]` and pads the missing seam end, covers
/// more than one period. Here each rim is unwrapped by continuity around exactly one period from
/// its sample nearest `p0` (`a` on the lower rim, `b` on the upper), and the wall is cut along the
/// straight chart line from `a` to `b`: the cut is private to the wall, sampled once on the surface
/// and used for both of its sides, so they weld, and the rims keep their shared samples. Holes are
/// moved into the same period; one that reaches the cut declines. The polygon carries periodic
/// parameters past the domain, so it is meshed with the domain-wrapping evaluator.
fn unaligned_rim_wall(
    face: &FaceRecord,
    rings: [&PeriodicRing; 2],
    holes: &[Vec<FaceVertex>],
    p_is_u: bool,
    p0: f64,
    period: f64,
    chord_tolerance: f64,
) -> Result<Option<Vec<FaceVertex>>, String> {
    let p_of = |v: &FaceVertex| if p_is_u { v.uv[0] } else { v.uv[1] };
    let set_p = |v: &mut FaceVertex, p: f64| if p_is_u { v.uv[0] = p } else { v.uv[1] = p };
    // One period of a rim, ascending in p, from its sample nearest p0 back to that sample a period on.
    let unwrap = |raw: &[FaceVertex]| -> Option<Vec<FaceVertex>> {
        let n = raw.len();
        if n < 3 {
            return None;
        }
        let circle = |p: f64| {
            let d = (p - p0).rem_euclid(period);
            d.min(period - d)
        };
        let start = (0..n).min_by(|&a, &b| circle(p_of(&raw[a])).total_cmp(&circle(p_of(&raw[b]))))?;
        let mut seq = Vec::with_capacity(n + 1);
        let mut first = raw[start];
        let base = p_of(&first) - ((p_of(&first) - p0) / period).round() * period;
        set_p(&mut first, base);
        seq.push(first);
        let mut p = base;
        for k in 1..=n {
            let mut v = raw[(start + k) % n];
            let mut delta = p_of(&v) - p_of(&raw[(start + k - 1) % n]);
            delta -= (delta / period).round() * period;
            p += delta;
            set_p(&mut v, p);
            seq.push(v);
        }
        let net = p - base;
        if (net.abs() - period).abs() > 1e-6 * period {
            return None;
        }
        if net < 0.0 {
            seq.reverse();
            let shift = base - p_of(&seq[0]);
            for v in &mut seq {
                let q = p_of(v) + shift;
                set_p(v, q);
            }
        }
        Some(seq)
    };
    let (Some(lower), Some(upper)) = (unwrap(&rings[0].raw), unwrap(&rings[1].raw)) else {
        return Ok(None);
    };
    let (a, b) = (lower[0], upper[0]);
    let eval = |uv: [f64; 2]| face.surface.evaluate_extended(uv[0], uv[1]);
    let at = |t: f64| [a.uv[0] + t * (b.uv[0] - a.uv[0]), a.uv[1] + t * (b.uv[1] - a.uv[1])];
    let mut ts = vec![0.0, 1.0];
    for _ in 0..24 {
        let mut refined = Vec::with_capacity(ts.len() * 2);
        let mut split = false;
        for pair in ts.windows(2) {
            refined.push(pair[0]);
            let mid = 0.5 * (pair[0] + pair[1]);
            if deflection_to_chord(eval(at(mid))?, eval(at(pair[0]))?, eval(at(pair[1]))?) > chord_tolerance {
                refined.push(mid);
                split = true;
            }
        }
        refined.push(1.0);
        ts = refined;
        if !split {
            break;
        }
    }
    let cut: Vec<(f64, Vec3)> = ts[1..ts.len() - 1].iter().map(|&t| Ok((t, eval(at(t))?))).collect::<Result<_, String>>()?;
    let lift = |uv: [f64; 2]| if p_is_u { [uv[0] + period, uv[1]] } else { [uv[0], uv[1] + period] };
    let mut polygon: Vec<FaceVertex> = lower.clone();
    for &(t, position) in &cut {
        polygon.push(FaceVertex { uv: lift(at(t)), position });
    }
    polygon.extend(upper.iter().rev().copied());
    for &(t, position) in cut.iter().rev() {
        polygon.push(FaceVertex { uv: at(t), position });
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }
    // Holes into the wall's period, clear of the cut.
    let lo = p_of(&a).max(p_of(&b));
    let hi = p_of(&a).min(p_of(&b)) + period;
    let margin = 1e-6 * period;
    let mut moved = Vec::with_capacity(holes.len());
    for hole in holes {
        let mean = hole.iter().map(p_of).sum::<f64>() / hole.len() as f64;
        let shift = ((0.5 * (lo + hi) - mean) / period).round() * period;
        let mut hole = hole.clone();
        for v in &mut hole {
            let p = p_of(v) + shift;
            set_p(v, p);
        }
        if hole.iter().any(|v| p_of(v) <= lo + margin || p_of(v) >= hi - margin) {
            return Ok(None);
        }
        if signed_area(&hole) > 0.0 {
            hole.reverse();
        }
        moved.push(hole);
    }
    if !moved.is_empty() {
        polygon = bridge_holes(polygon, moved);
    }
    Ok(Some(polygon))
}

/// One-direction attempt for [`close_periodic_trim`]: treat `p_is_u` as the
/// wrapping (periodic) parameter and rebuild the trim as the seam-cut rectangle
/// iff every loop is a clean full rim or a pole at a constant cross level.
///
/// When the CROSS direction is periodic too (`q_closed` — a torus band), the two
/// rims bound TWO valid regions: the band between them and its complement across
/// the cross seam. Picking the between-rims band unconditionally keeps the wrong
/// side of every fillet torus whose material lies across the seam (screw
/// 00035619 Face_5/Face_7: a 59°/90° fillet rendered as the complementary
/// 300°/270° sweep — the surface then bulges past both rims instead of
/// interpolating between them). Resolve it the way the kernel's own
/// `validate_uv_wire` does: the material is to the LEFT of the boundary
/// traversal, i.e. the region's outer wire is CCW in (u,v) iff `same_sense`.
fn close_periodic_trim_dir(
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
    // (periodic, cross) coordinates of a face vertex.
    let coord = |vertex: &FaceVertex| -> (f64, f64) {
        if p_is_u {
            (vertex.uv[0], vertex.uv[1])
        } else {
            (vertex.uv[1], vertex.uv[0])
        }
    };
    let mut rings: Vec<PeriodicRing> = Vec::with_capacity(chains.len());
    // Interior HOLE loops on the periodic wall (a hole drilled through a cylinder):
    // not a cross-boundary rim/pole, but a closed loop sitting inside the domain.
    // They are collected and bridged into the seam-cut rectangle rather than
    // forcing a bail to the domain-spraying generic path.
    let mut holes: Vec<Vec<FaceVertex>> = Vec::new();
    for chain in chains {
        if chain.is_empty() {
            return Ok(None);
        }
        let (mut pmin, mut pmax) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut qmin, mut qmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for vertex in chain {
            let (p, q) = coord(vertex);
            pmin = pmin.min(p);
            pmax = pmax.max(p);
            qmin = qmin.min(q);
            qmax = qmax.max(q);
        }
        // Net traversal along the periodic parameter, seam jumps unwrapped: the
        // loop's sense (the chain follows the coedge order, and each coedge's
        // pcurve is sampled along its own domain, which the loop orients).
        let mut net = 0.0;
        for pair in chain.windows(2) {
            let mut delta = coord(&pair[1]).0 - coord(&pair[0]).0;
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
        let coincident = chain
            .iter()
            .all(|vertex| vertex.position.sub(chain[0].position).length() <= 1e-9);
        if coincident && (pmax - pmin) < 1e-3 * period && (qmax - qmin) < 1e-3 * q_extent {
            // Pole: the whole boundary collapses to one 3D point.
            rings.push(PeriodicRing {
                level: coord(&chain[0]).1,
                points: vec![chain[0]],
                pole: true,
                raw: chain.clone(),
                direction,
            });
        } else if (pmax - pmin) >= 0.6 * period && (qmax - qmin) <= 0.05 * q_extent {
            // Full rim: an open line spanning (nearly) the whole period at a
            // near-constant cross level. Order by ascending periodic parameter,
            // then close the wrap. A CLOSED rim (full circle) traces p0->p1 and
            // back to its seam point, so edge sampling drops exactly ONE seam
            // endpoint — WHICH one depends on the coedge's traversal direction
            // (low->high drops p1, high->low drops p0). Fill the MISSING one at
            // the seam position (u=p0 == u=p1 in 3D on the closed rim, so it is
            // the surviving endpoint's position). The old code always appended
            // p1: for a rim sampled high->low that duplicated p1 and orphaned
            // p0, slanting the closing edge and spraying triangles (onshape
            // torus band face 827's v=0.25 rim: 388 out-of-trim).
            let mut points = chain.clone();
            points.sort_by(|a, b| coord(a).0.total_cmp(&coord(b).0));
            let level = (qmin + qmax) * 0.5;
            let epsilon = 1e-6 * period;
            let uv_at = |p: f64| if p_is_u { [p, level] } else { [level, p] };
            if coord(&points[0]).0 > p0 + epsilon {
                let seam_position = points[points.len() - 1].position;
                points.insert(
                    0,
                    FaceVertex {
                        uv: uv_at(p0),
                        position: seam_position,
                    },
                );
            }
            if coord(&points[points.len() - 1]).0 < p1 - epsilon {
                let seam_position = points[0].position;
                points.push(FaceVertex {
                    uv: uv_at(p1),
                    position: seam_position,
                });
            }
            rings.push(PeriodicRing {
                level,
                points,
                pole: false,
                raw: chain.clone(),
                direction,
            });
        } else {
            // A genuine interior loop that does NOT straddle the seam (a hole
            // drilled through the wall, entirely within the domain). Seam-hopping
            // loops are handled upstream by the seam-unwrap path and never reach
            // here. Collect it as a hole to bridge into the seam-cut rectangle.
            // A hole whose parametric span reaches the seam is rejected (the
            // straddle path owns it) so we never mis-bridge across u0/u1.
            if pmin <= p0 + 1e-6 * period || pmax >= p1 - 1e-6 * period {
                return Ok(None);
            }
            holes.push(chain.clone());
        }
    }
    // RIM-ONLY POLE CAP (`BREP_TESS_RIM_POLE_CAP`, default on, `=0` restores; `check`
    // reports): a cap whose
    // one loop is its rim, the pole written as no edge at all (a STEP drill
    // point: a cone face bounded by the hole's bottom circle alone), is the band
    // from the rim to the cross end where the surface collapses to one point.
    // That pole is given its ring here; with neither or both ends collapsed the
    // face keeps its path. Without it the rim alone is a zero-area polygon and
    // the generic path sprays the cap (2.4x its area, a 0-volume mesh).
    let rim_pole_cap_check = crate::env_read!("BREP_TESS_RIM_POLE_CAP").as_deref() == Some("check");
    if rings.len() == 1 && !rings[0].pole && holes.is_empty() && (rim_pole_cap_enabled() || rim_pole_cap_check) {
        let collapsed_at = |q: f64| -> Option<Vec3> {
            let at = |p: f64| if p_is_u { face.surface.evaluate(p, q) } else { face.surface.evaluate(q, p) };
            let first = at(p0).ok()?;
            let band = chord_tolerance.max(1e-9) * 1e-3;
            (1..=8)
                .all(|k| at(p0 + period * k as f64 / 8.0).ok().is_some_and(|point| point.sub(first).length() <= band))
                .then_some(first)
        };
        let pole = match (collapsed_at(q0), collapsed_at(q1)) {
            (Some(point), None) => Some((q0, point)),
            (None, Some(point)) => Some((q1, point)),
            _ => None,
        };
        if let (Some((level, _)), true) = (pole, rim_pole_cap_check) {
            eprintln!("TESS_RIM_POLE_CAP: a rim-only cap would be closed at its pole (cross level {level:.6}); the switch meshes it so");
        } else if let Some((level, position)) = pole {
            let uv = if p_is_u { [p0, level] } else { [level, p0] };
            let vertex = FaceVertex { uv, position };
            rings.push(PeriodicRing { level, points: vec![vertex], pole: true, raw: vec![vertex], direction: 0 });
        }
    }
    // A wall has exactly two cross-level boundaries; at least one must be a rim
    // (two poles would be an isolated point, not a surface patch).
    if rings.len() != 2 || rings.iter().all(|ring| ring.pole) {
        return Ok(None);
    }
    rings.sort_by(|a, b| a.level.total_cmp(&b.level));
    // Which of the two regions the rims bound is material. With a NON-periodic
    // cross direction only the between-rims band exists. With a periodic one the
    // complement (across the cross seam) is equally valid, so read the loop
    // senses: walking the lower rim along +p keeps the between-rims band on the
    // left, i.e. CCW in (p,q) — and CCW in (u,v) too when p IS u, reversed when
    // the (p,q) frame swaps the axes. `validate_uv_wire` requires the outer wire
    // to be CCW in (u,v) exactly when `same_sense`.
    let (lower_direction, upper_direction) = (rings[0].direction, rings[1].direction);
    let inconclusive = lower_direction == 0
        || upper_direction == 0
        || lower_direction == upper_direction
        || rings.iter().any(|ring| ring.pole);
    let between_rims_is_ccw_uv = if p_is_u {
        lower_direction > 0
    } else {
        lower_direction < 0
    };
    // Keep the between-rims band unless the cross direction wraps AND the senses
    // conclusively name the complement (an unreadable orientation falls back to
    // the historical choice rather than inventing a region).
    let wraps_cross_seam = q_closed && !inconclusive && between_rims_is_ccw_uv != face.same_sense;
    if std::env::var("BREP_DEBUG_TESS_BAND").is_ok() {
        eprintln!(
            "face {}: periodic rims p_is_u={p_is_u} q_closed={q_closed} \
             senses=({lower_direction:+},{upper_direction:+}) same_sense={} levels {:.5}/{:.5} -> {}",
            face.id,
            face.same_sense,
            rings[0].level,
            rings[1].level,
            if wraps_cross_seam { "CROSS-SEAM band" } else { "between-rims band" },
        );
    }
    if !wraps_cross_seam && rings.iter().all(|ring| !ring.pole) && unaligned_rims_enabled() {
        let outside = rings.iter().any(|ring| {
            ring.raw.iter().any(|vertex| {
                let p = coord(vertex).0;
                p < p0 - 1e-9 * period || p > p1 + 1e-9 * period
            })
        });
        if outside {
            return Ok(unaligned_rim_wall(face, [&rings[0], &rings[1]], &holes, p_is_u, p0, period, chord_tolerance)?
                .map(|polygon| (polygon, true)));
        }
    }
    let q_period = q1 - q0;
    // Re-express the complement as an ordinary rectangle by lifting the lower rim
    // a full cross period: the band then runs upward from the upper rim, across
    // the cross seam, to the lifted lower rim. Cross parameters past `q1` are
    // evaluated with the domain-wrapping extended evaluator (Golovanov §3.15).
    let lift_ring = |ring: &PeriodicRing| -> PeriodicRing {
        let mut lifted = ring.clone();
        lifted.level += q_period;
        for vertex in &mut lifted.points {
            if p_is_u {
                vertex.uv[1] += q_period;
            } else {
                vertex.uv[0] += q_period;
            }
        }
        lifted
    };
    let rings: [PeriodicRing; 2] = if wraps_cross_seam {
        [rings[1].clone(), lift_ring(&rings[0])]
    } else {
        [rings[0].clone(), rings[1].clone()]
    };
    let (q_lo, q_hi) = (rings[0].level, rings[1].level);
    let mk_uv = |p: f64, q: f64| if p_is_u { [p, q] } else { [q, p] };
    // Shared seam q-list (identical for both sides: p0 ≡ p1 on a closed wall).
    let seam = seam_meridian_samples(
        face,
        p1,
        p_is_u,
        q_lo,
        q_hi,
        chord_tolerance,
        wraps_cross_seam,
    )?;
    // Cross-level boundary spanning p0..p1 at the given level, ascending in p.
    let boundary_at = |ring: &PeriodicRing| -> Vec<FaceVertex> {
        if ring.pole {
            let position = ring.points[0].position;
            vec![
                FaceVertex {
                    uv: mk_uv(p0, ring.level),
                    position,
                },
                FaceVertex {
                    uv: mk_uv(p1, ring.level),
                    position,
                },
            ]
        } else {
            ring.points.clone()
        }
    };
    // Assemble the rectangle CCW in (p, q): bottom (p0->p1), right seam
    // (q_lo->q_hi), top reversed (p1->p0), left seam (q_hi->q_lo). Seam
    // endpoints are dropped — corners come from the boundaries.
    let mut polygon: Vec<FaceVertex> = Vec::new();
    polygon.extend(boundary_at(&rings[0]));
    if seam.len() > 2 {
        for &(q, position) in &seam[1..seam.len() - 1] {
            polygon.push(FaceVertex {
                uv: mk_uv(p1, q),
                position,
            });
        }
    }
    let mut top = boundary_at(&rings[1]);
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
    if polygon.len() < 3 {
        return Ok(None);
    }
    // Ear clip expects CCW in (u,v); (v,u)-CCW reads reversed when v wraps.
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }
    // Bridge any interior holes into the seam-cut rectangle (CCW outer, CW holes)
    // so a drilled cylinder wall meshes as one clean trimmed region.
    if !holes.is_empty() {
        let holes: Vec<Vec<FaceVertex>> = holes
            .into_iter()
            .map(|mut hole| {
                if signed_area(&hole) > 0.0 {
                    hole.reverse();
                }
                hole
            })
            .collect();
        polygon = bridge_holes(polygon, holes);
    }
    Ok(Some((polygon, wraps_cross_seam)))
}

/// `BREP_TESS_RIM_POLE_CAP` (default on, `=0` restores; `check` reports each cap it would
/// close and meshes it as before): see `close_periodic_trim_dir`.
fn rim_pole_cap_enabled() -> bool {
    crate::switch!("BREP_TESS_RIM_POLE_CAP", true)
}
