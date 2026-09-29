use super::*;
use super::mass_profile;

/// Integrate all `kinds` over the trimmed parameter region in ONE pass over
/// a conforming cell decomposition (Golovanov §8.10): knot-span grid cells,
/// fully covered cells by tensor Gauss–Legendre, boundary cells by the
/// degree-5 triangle cubature over the trim clipped to the cell (signed
/// fans — loop winding subtracts holes).  Replaces the scanline that
/// re-scanned every trim segment at every Gauss station (quadratic in trim
/// complexity), and evaluates the surface once per station for every
/// integrand at once.
pub(super) fn integrate_trimmed_multi(
    face: &FaceRecord,
    kinds: &[Integrand],
) -> Result<Vec<f64>, String> {
    integrate_trimmed_multi_with(face, kinds, true)
}

/// `integrate_trimmed_multi` with the winding boundary integral switchable
/// off, so a test can read the chord-polygon paths on the same face.
pub(super) fn integrate_trimmed_multi_with(
    face: &FaceRecord,
    kinds: &[Integrand],
    allow_winding: bool,
) -> Result<Vec<f64>, String> {
    let ku = crate::KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = crate::KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let domain = [u0, u1, v0, v1];
    let (closed_u, closed_v) = face.surface.closed_directions()?;
    if crate::topology::doubly_periodic_has_only_collapsed_loops(face)? {
        return kinds
            .iter()
            .map(|&kind| integrate_untrimmed(face, kind))
            .collect();
    }
    // A periodic face whose trim loop hops an IMPLICIT seam is integrated on the
    // unwrapped covering plane; every other face keeps the exact (bit-identical)
    // in-domain path. The unwrapped polygon set is the geometrically faithful
    // trim: every seam-hopping loop is reconnected across the seam, so a WIDE
    // band that rides one seam keeps its full extent AND a hole that straddles
    // the seam subtracts correctly. The raw in-domain polygon instead carries a
    // whole-period teleport at the seam, so its winding/area is only a proxy:
    //   - a wide band collapses its raw polygon to the thin uncovered complement
    //     (46× too small — under-counts);
    //   - a seam-straddling HOLE fails to subtract in the raw polygon, leaving
    //     the face LARGER than the real region (over-counts — ABC 00000003
    //     face #87, an 82% area / 70% volume inflation).
    // So the unwrapped set is preferred whenever it is a real region. It is only
    // distrusted when `loop_seam_offsets` could not reconnect the loop and the
    // unwrap DEGENERATES to a sliver, in which case the raw path is the only
    // usable estimate.
    if closed_u || closed_v {
        // Loops that wind around the periodic direction bound no region of the
        // covering plane; the boundary integral across the winding direction
        // reads them exactly, with no proxy region and no chord.
        if allow_winding {
            if let Some(values) = winding::winding_band_integral(face, kinds)? {
                mass_profile(|p| p.route_winding += 1);
                return Ok(values);
            }
        }
        // A doubly-periodic (torus) face whose trim is a full-wrap band bounded
        // by a constant-level seam rim (at a cross extreme) and a single-valued
        // wavy cut collapses to a sliver on the raw winding path. Rebuild the
        // band as one in-domain polygon so the winding integrator fills it.
        if closed_u && closed_v {
            if let Some(polygons) = doubly_periodic_seam_band_polygons(face, domain)? {
                mass_profile(|p| p.route_seam_band += 1);
                return integrate_trimmed_polys(face, kinds, &polygons, false, domain);
            }
        }
        // A singly-periodic wall bounded by two implicit-seam rim circles at the
        // cross-domain extremes collapses to a sliver on the raw path (both rims
        // are zero-area iso lines). Rebuild it as a full-domain band rectangle
        // (minus its hole loops) so the winding integrator fills the wall.
        if closed_u != closed_v {
            if let Some(polygons) =
                singly_periodic_sphere_cap_polygons(face, closed_u, closed_v, domain)?
            {
                let extended = polygons.iter().flat_map(|p| p.points.iter()).any(|point| {
                    point[0] < u0 - 1e-9
                        || point[0] > u1 + 1e-9
                        || point[1] < v0 - 1e-9
                        || point[1] > v1 + 1e-9
                });
                mass_profile(|p| p.route_cap += 1);
                return integrate_trimmed_polys(face, kinds, &polygons, extended, domain);
            }
            if let Some(polygons) = singly_periodic_wall_polygons(face, closed_u, closed_v, domain)?
            {
                let extended = polygons.iter().flat_map(|p| p.points.iter()).any(|point| {
                    point[0] < u0 - 1e-9
                        || point[0] > u1 + 1e-9
                        || point[1] < v0 - 1e-9
                        || point[1] > v1 + 1e-9
                });
                mass_profile(|p| p.route_wall += 1);
                return integrate_trimmed_polys(face, kinds, &polygons, extended, domain);
            }
        }
        let polygons_started = super::profile::profile_started();
        let (unwrapped, hopped) =
            trim_polygons_unwrapped(face, closed_u, closed_v, u1 - u0, v1 - v0)?;
        mass_profile(|p| p.polygons_ms += super::profile::elapsed_ms(polygons_started));
        if hopped {
            let raw = trim_polygons_tagged(face)?;
            let raw_area =
                integrate_trimmed_polys(face, &[Integrand::Area], &raw, false, domain)?[0];
            // The unwrapped pass carries the requested kinds AND the area probe
            // in one sweep: every slot of `integrate_trimmed_polys` accumulates
            // on its own (the cells, the clipping, the correction and the
            // orientation never read a kind), so the area slot here is the
            // number a standalone `[Area]` pass returns, bit for bit, and the
            // kinds are the numbers a second pass over the same polygons would
            // return. Two passes per hopped face were 0.6 s of
            // `anotherBooleanFail` across its five integrator calls.
            let mut probed: Vec<Integrand> = kinds.to_vec();
            probed.push(Integrand::Area);
            let unwrapped_values =
                integrate_trimmed_polys(face, &probed, &unwrapped, true, domain)?;
            let unwrapped_area = unwrapped_values[kinds.len()];
            // Fall back to raw ONLY when the unwrap collapsed to a sliver (a torn
            // covering-plane polygon); a substantial unwrapped region is trusted
            // even when it is SMALLER than raw (the seam-straddling-hole case).
            let unwrap_degenerate = unwrapped_area.abs() < 1e-3 * raw_area.abs();
            return if unwrap_degenerate {
                mass_profile(|p| p.route_hopped_raw += 1);
                integrate_trimmed_polys(face, kinds, &raw, false, domain)
            } else {
                mass_profile(|p| p.route_hopped_unwrapped += 1);
                Ok(unwrapped_values[..kinds.len()].to_vec())
            };
        }
        // No seam hop between coedges, but a single edge that straddles the
        // seam carries a CONTINUOUS pcurve just past the domain boundary (the
        // pcurve fitter keeps closed directions unwrapped rather than clamping
        // straddling geometry onto the seam). Integrate such a run on the tiled
        // covering grid so the wrapped sliver past the boundary is counted.
        let straddles = unwrapped.iter().flat_map(|p| p.points.iter()).any(|point| {
            (closed_u && (point[0] < u0 - 1e-9 || point[0] > u1 + 1e-9))
                || (closed_v && (point[1] < v0 - 1e-9 || point[1] > v1 + 1e-9))
        });
        mass_profile(|p| if straddles { p.route_straddle += 1 } else { p.route_periodic_raw += 1 });
        return integrate_trimmed_polys(face, kinds, &unwrapped, straddles, domain);
    }
    let polygons_started = super::profile::profile_started();
    let raw = trim_polygons_tagged(face)?;
    mass_profile(|p| {
        p.route_raw += 1;
        p.polygons_ms += super::profile::elapsed_ms(polygons_started);
    });
    integrate_trimmed_polys(face, kinds, &raw, false, domain)
}

/// Integrate every `kind` over the trim `polygons` by the conforming
/// cell decomposition. `extended` selects the covering-plane path (grid tiled
/// across the wrapped periods, domain-wrapping evaluator) for an unwrapped
/// seam-hopping loop; `false` is the exact in-domain path.
///
/// The cells see the CHORD polygons; the face is bounded by the pcurve ARCS
/// behind them. The two regions differ by the signed sliver between every
/// chord and its arc, which [`curved_boundary_correction`] integrates
/// exactly, so the result does not depend on how finely the trim was
/// sampled — a fitted sphere rim curved in uv, sampled at 128 chords, put the
/// r = 1 two-sphere fillet 5e-2 above its closed form before this term.
pub(super) fn integrate_trimmed_polys(
    face: &FaceRecord,
    kinds: &[Integrand],
    polygons: &[TrimPolygon],
    extended: bool,
    domain: [f64; 4],
) -> Result<Vec<f64>, String> {
    let [u0, u1, v0, v1] = domain;
    // The scanline used even-odd coverage; winding-signed fans reproduce it
    // for well-formed loops up to the OVERALL orientation, normalized here.
    let orientation = if polygons
        .iter()
        .map(|p| polygon_signed_area(&p.points))
        .sum::<f64>()
        < 0.0
    {
        -1.0
    } else {
        1.0
    };
    let loop_boxes: Vec<[f64; 4]> = polygons
        .iter()
        .map(|polygon| {
            let mut bounds = [
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ];
            for point in &polygon.points {
                bounds[0] = bounds[0].min(point[0]);
                bounds[1] = bounds[1].max(point[0]);
                bounds[2] = bounds[2].min(point[1]);
                bounds[3] = bounds[3].max(point[1]);
            }
            bounds
        })
        .collect();
    let base_u = cell_breaks(&face.surface.knots_u, face.surface.degree_u, u0, u1);
    let base_v = cell_breaks(&face.surface.knots_v, face.surface.degree_v, v0, v1);
    // The unwrapped trim runs past the domain in whichever direction it wrapped;
    // tile the conforming grid across those periods so every covered period
    // keeps its knot-aligned cells. The straddled direction keeps its base grid.
    let (u_breaks, v_breaks) = if extended {
        let bounds = polygons.iter().flat_map(|p| p.points.iter()).fold(
            [
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ],
            |mut acc, point| {
                acc[0] = acc[0].min(point[0]);
                acc[1] = acc[1].max(point[0]);
                acc[2] = acc[2].min(point[1]);
                acc[3] = acc[3].max(point[1]);
                acc
            },
        );
        (
            tiled_breaks(&base_u, bounds[0], bounds[1], u1 - u0),
            tiled_breaks(&base_v, bounds[2], bounds[3], v1 - v0),
        )
    } else {
        (base_u, base_v)
    };

    let sign = if face.same_sense { 1.0 } else { -1.0 };
    // One weight reading per knot cell for the whole walk: the quadtree asks
    // the same cell for panels at four depths.
    let mut rules = rule::SurfaceRules::new(&face.surface)?;
    // A fully covered cell's 8x8 tensor Gauss block asks the surface for the
    // same eight u rows and eight v rows sixty-four times; `deriv1_tensor_each`
    // evaluates each once and hands back bit-identical stations. The
    // covering-plane path keeps the per-station `deriv1_extended` (its wrap is
    // a per-station decision), and so does a patch above the stack-basis
    // degree.
    let tensor_block = !extended && face.surface.deriv1_tensor_supported() && tensor_blocks_on();
    let mut totals = vec![0.0f64; kinds.len()];
    let station = |u: f64, v: f64, weight: f64, totals: &mut [f64]| -> Result<(), String> {
        // On the unwrapped grid a station can sit one or more periods past the
        // domain; the periodic extension wraps it back onto the real surface
        // (identical to `derivatives` in-domain, so the non-extended path is
        // bit-for-bit unchanged).
        let (point, su, sv) = if extended {
            face.surface.deriv1_extended(u, v)?
        } else {
            face.surface.deriv1(u, v)?
        };
        let weighted_normal = su.cross(sv).scale(sign);
        for (slot, kind) in kinds.iter().enumerate() {
            totals[slot] += weight * integrand_value(*kind, point, weighted_normal);
        }
        Ok(())
    };

    // Boundary cells refine as a quadtree so accuracy concentrates where
    // the trim actually runs (tangency cusps, slivers).  Clipping is
    // HIERARCHICAL: each cell carries its loops already clipped to it, and
    // children clip the parent's fragments — re-clipping the full trim
    // polygon at every leaf was the dominant cost, not the quadrature.
    const MAX_DEPTH: usize = 4;
    mass_profile(|p| p.polygon_chords += polygons.iter().map(|poly| poly.points.len() as u64).sum::<u64>());
    let base_clip_started = super::profile::profile_started();
    let mut stack: Vec<([f64; 4], usize, Vec<Vec<[f64; 2]>>)> = Vec::new();
    // Two working vectors for every Sutherland–Hodgman pass in this call; only
    // a surviving fragment is handed to the stack as its own allocation.
    let mut clip_scratch: Vec<[f64; 2]> = Vec::new();
    let mut clip_result: Vec<[f64; 2]> = Vec::new();
    for upair in u_breaks.windows(2) {
        for vpair in v_breaks.windows(2) {
            let cell = [upair[0], upair[1], vpair[0], vpair[1]];
            let mut clipped: Vec<Vec<[f64; 2]>> = Vec::new();
            for (polygon, bounds) in polygons.iter().zip(&loop_boxes) {
                // A loop wholly outside the cell clips to nothing —
                // unless it ENCLOSES the cell, which the bbox test
                // keeps (an enclosing loop's bbox covers the cell).
                if !(bounds[0] <= cell[1]
                    && bounds[1] >= cell[0]
                    && bounds[2] <= cell[3]
                    && bounds[3] >= cell[2])
                {
                    continue;
                }
                clip_polygon_to_cell_into(
                    &polygon.points,
                    cell,
                    &mut clip_scratch,
                    &mut clip_result,
                );
                if clip_result.len() >= 3 {
                    // Copy the fragment out rather than donating the buffer:
                    // the ping-pong vectors carry the WHOLE trim loop's
                    // capacity (thousands of chords on a big face) and handing
                    // that to a four-vertex fragment made every later cell
                    // re-grow it from nothing.
                    clipped.push(clip_result.clone());
                }
            }
            stack.push((cell, 0, clipped));
        }
    }
    mass_profile(|p| {
        p.base_cells += stack.len() as u64;
        p.base_clip_ms += super::profile::elapsed_ms(base_clip_started);
    });
    let cells_started = super::profile::profile_started();
    while let Some((cell, depth, clipped)) = stack.pop() {
        let cell_area = (cell[1] - cell[0]) * (cell[3] - cell[2]);
        if cell_area <= 0.0 {
            continue;
        }
        let mut winding = 0.0;
        let mut partial = false;
        for polygon in &clipped {
            let area = polygon_signed_area(polygon);
            if area.abs() >= cell_area * (1.0 - 1e-9) {
                winding += area.signum();
            } else if area.abs() > cell_area * 1e-12 {
                partial = true;
                break;
            }
        }
        if !partial {
            if winding.abs() <= 0.5 {
                continue;
            }
            mass_profile(|p| p.full_cells += 1);
            let full_started = super::profile::profile_started();
            if tensor_block {
                // The cell's own tensor blocks: the same stations, order and
                // weights as the one-block rule on a polynomial carrier, and
                // the panels and orders a RATIONAL one's weights ask for
                // (`rule`). Either way the u spans/basis rows and v
                // spans/basis rows of a block are evaluated once for it
                // instead of once per station.
                let stations = integrate_cell_scaled(
                    face,
                    cell,
                    winding * orientation,
                    kinds,
                    &mut totals,
                    &mut rules,
                )?;
                mass_profile(|p| p.full_stations += stations as u64);
            } else {
                // The covering-plane path keeps its per-station wrap, so it
                // reads the cell through `station` — at the same panels and
                // orders.
                let half_u = (cell[1] - cell[0]) * 0.5;
                let middle_u = (cell[1] + cell[0]) * 0.5;
                let half_v = (cell[3] - cell[2]) * 0.5;
                let middle_v = (cell[3] + cell[2]) * 0.5;
                let (u_panels, v_panels) = rules.panels(&face.surface, cell)?;
                let one_block = u_panels.len() == 1 && v_panels.len() == 1 && u_panels[0].order == GAUSS_COUNT && v_panels[0].order == GAUSS_COUNT;
                if one_block {
                    for i in 0..GAUSS_X.len() {
                        for j in 0..GAUSS_X.len() {
                            station(
                                middle_u + half_u * GAUSS_X[i],
                                middle_v + half_v * GAUSS_X[j],
                                winding * orientation * GAUSS_W[i] * GAUSS_W[j] * half_u * half_v,
                                &mut totals,
                            )?;
                        }
                    }
                    mass_profile(|p| p.full_stations += (GAUSS_X.len() * GAUSS_X.len()) as u64);
                } else {
                    for u_panel in &u_panels {
                        for v_panel in &v_panels {
                            for (u, u_weight) in u_panel.stations() {
                                for (v, v_weight) in v_panel.stations() {
                                    station(u, v, winding * orientation * u_weight * v_weight, &mut totals)?;
                                }
                            }
                            mass_profile(|p| p.full_stations += (u_panel.order * v_panel.order) as u64);
                        }
                    }
                }
            }
            mass_profile(|p| p.full_ms += super::profile::elapsed_ms(full_started));
            continue;
        }
        if depth < MAX_DEPTH {
            let child_started = super::profile::profile_started();
            let middle_u = (cell[0] + cell[1]) * 0.5;
            let middle_v = (cell[2] + cell[3]) * 0.5;
            for child in [
                [cell[0], middle_u, cell[2], middle_v],
                [middle_u, cell[1], cell[2], middle_v],
                [cell[0], middle_u, middle_v, cell[3]],
                [middle_u, cell[1], middle_v, cell[3]],
            ] {
                let mut child_clipped: Vec<Vec<[f64; 2]>> = Vec::new();
                for polygon in &clipped {
                    clip_polygon_to_cell_into(
                        polygon,
                        child,
                        &mut clip_scratch,
                        &mut clip_result,
                    );
                    if clip_result.len() >= 3 {
                        child_clipped.push(clip_result.clone());
                    }
                }
                stack.push((child, depth + 1, child_clipped));
            }
            mass_profile(|p| p.child_clip_ms += super::profile::elapsed_ms(child_started));
            continue;
        }
        mass_profile(|p| p.leaf_cells += 1);
        let leaf_started = super::profile::profile_started();
        for polygon in &clipped {
            for index in 1..polygon.len() - 1 {
                mass_profile(|p| p.tri_stations += TRIANGLE_CUBATURE.len() as u64);
                let a = polygon[0];
                let b = polygon[index];
                let c = polygon[index + 1];
                let jacobian =
                    0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]));
                if jacobian.abs() <= 1e-30 {
                    continue;
                }
                for (bary, weight) in TRIANGLE_CUBATURE {
                    station(
                        bary[0] * a[0] + bary[1] * b[0] + bary[2] * c[0],
                        bary[0] * a[1] + bary[1] * b[1] + bary[2] * c[1],
                        orientation * weight * jacobian,
                        &mut totals,
                    )?;
                }
            }
        }
        mass_profile(|p| p.leaf_ms += super::profile::elapsed_ms(leaf_started));
    }
    mass_profile(|p| p.cells_ms += super::profile::elapsed_ms(cells_started));
    let corr_started = super::profile::profile_started();
    curved_boundary_correction(face, polygons, extended, domain, orientation, &station, &mut totals)?;
    mass_profile(|p| p.corr_ms += super::profile::elapsed_ms(corr_started));
    Ok(totals)
}

/// Three-point Gauss–Legendre on [0, 1], across the sliver.
const ACROSS_X: [f64; 3] = [0.1127016653792583, 0.5, 0.8872983346207417];
const ACROSS_W: [f64; 3] = [5.0 / 18.0, 8.0 / 18.0, 5.0 / 18.0];

/// Add the signed region between every chord and the pcurve arc behind it.
///
/// Map the unit square onto the sliver: `Q(s, λ) = A(s) + λ·(C(t(s)) − A(s))`
/// with `A` the chord and `C` the arc, both from the chord's start at `s = 0`
/// to its end at `s = 1`. The square's boundary lands on the chord forward and
/// the arc backward, so by the degree formula ∫∫ f(Q)·det(DQ) ds dλ is the
/// integral of `f` weighted by the winding number of (chord − arc) — exactly
/// minus the piece the chord polygon is missing, with the sign coming out of
/// the Jacobian whichever side the arc bulges to and however it crosses the
/// chord. The polygon's own fans carry `orientation`, so the correction does
/// too. The sliver is thin and the integrand smooth, so eight stations along
/// the chord and three across resolve it to quadrature precision.
///
/// On the in-domain path the cells discard everything outside the domain, and
/// so does this: a station past the domain edge is skipped there and wrapped
/// by the periodic evaluator on the covering-plane path.
fn curved_boundary_correction(
    face: &FaceRecord,
    polygons: &[TrimPolygon],
    extended: bool,
    domain: [f64; 4],
    orientation: f64,
    station: &dyn Fn(f64, f64, f64, &mut [f64]) -> Result<(), String>,
    totals: &mut [f64],
) -> Result<(), String> {
    let [u0, u1, v0, v1] = domain;
    let eps_u = 1e-12 * (u1 - u0).abs();
    let eps_v = 1e-12 * (v1 - v0).abs();
    for polygon in polygons {
        let n = polygon.points.len();
        for (k, arc) in polygon.arcs.iter().enumerate() {
            let Some(arc) = arc else {
                continue;
            };
            let p0 = polygon.points[k];
            let p1 = polygon.points[(k + 1) % n];
            let chord = [p1[0] - p0[0], p1[1] - p0[1]];
            let curve = &face.loops[arc.loop_index].coedges[arc.coedge_index].pcurve;
            let span = arc.t1 - arc.t0;
            mass_profile(|p| p.corr_chords += 1);
            // The sliver is walked in the chord's own parameter s ∈ [0, 1];
            // the arc behind it is this pcurve over [t0, t1], so the panels
            // and orders are ITS weights', mapped back to s.
            let panels = rule::curve_panels(curve, arc.t0, arc.t1)?;
            for panel in &panels {
                let (nodes, weights) = panel.nodes();
                let (lo, hi) = ((panel.lo - arc.t0) / span, (panel.hi - arc.t0) / span);
                mass_profile(|p| p.corr_stations += (nodes.len() * ACROSS_X.len()) as u64);
                for i in 0..nodes.len() {
                    let s = lo + (hi - lo) * 0.5 * (nodes[i] + 1.0);
                    let weight_s = 0.5 * weights[i] * (hi - lo);
                    let (point, tangent) = curve.deriv1(arc.t0 + s * span)?;
                    let on_arc = [point.x + arc.offset[0], point.y + arc.offset[1]];
                    let on_chord = [p0[0] + s * chord[0], p0[1] + s * chord[1]];
                    let deviation = [on_arc[0] - on_chord[0], on_arc[1] - on_chord[1]];
                    let deviation_s = [tangent.x * span - chord[0], tangent.y * span - chord[1]];
                    for j in 0..ACROSS_X.len() {
                        let lambda = ACROSS_X[j];
                        let q_s = [chord[0] + lambda * deviation_s[0], chord[1] + lambda * deviation_s[1]];
                        let jacobian = q_s[0] * deviation[1] - q_s[1] * deviation[0];
                        let u = on_chord[0] + lambda * deviation[0];
                        let v = on_chord[1] + lambda * deviation[1];
                        if !extended && (u < u0 - eps_u || u > u1 + eps_u || v < v0 - eps_v || v > v1 + eps_v) {
                            continue;
                        }
                        station(u, v, -orientation * weight_s * ACROSS_W[j] * jacobian, totals)?;
                    }
                }
            }
        }
    }
    Ok(())
}

pub(super) fn integrate_trimmed(face: &FaceRecord, kind: Integrand) -> Result<f64, String> {
    Ok(integrate_trimmed_multi(face, &[kind])?[0])
}

pub(super) fn planar_metric(face: &FaceRecord) -> Result<f64, String> {
    let ku = crate::KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = crate::KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let derivatives = face
        .surface
        .derivatives(ku.domain()[0], kv.domain()[0], 1)?;
    Ok(derivatives[1][0].cross(derivatives[0][1]).length())
}
