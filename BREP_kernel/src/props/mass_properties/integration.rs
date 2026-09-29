pub(super) use crate::curve::interior_knots;
use super::*;

pub(crate) fn curve_breaks(curve: &NurbsCurve) -> Result<Vec<f64>, String> {
    let [start, end] = curve.domain()?;
    let mut result = vec![start];
    result.extend(interior_knots(&curve.knots, curve.degree));
    result.push(end);
    Ok(result)
}

pub fn parameter_space_area(face: &FaceRecord) -> Result<f64, String> {
    let mut area = 0.0;
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            for pair in curve_breaks(&coedge.pcurve)?.windows(2) {
                for panel in rule::curve_panels(&coedge.pcurve, pair[0], pair[1])? {
                    for (parameter, weight) in panel.stations() {
                        let (point, tangent) = coedge.pcurve.deriv1(parameter)?;
                        area += weight * 0.5 * (point.x * tangent.y - point.y * tangent.x);
                    }
                }
            }
        }
    }
    Ok(area)
}

/// The escape hatch for [`closed_parameter_space_area`]: `BREP_OPEN_PLANAR_LOOPS=1`
/// reads every planar face's trim loops open again, as they were read before
/// the closing chords, so the two readings are two cells of one binary.
fn open_planar_loops() -> bool {
    static OPEN: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *OPEN.get_or_init(|| std::env::var_os("BREP_OPEN_PLANAR_LOOPS").is_some())
}

/// Every straight uv chord that closes a trim loop of `face`: from where one
/// coedge's pcurve ends to where the next one's begins, the last back to the
/// first. A loop whose pcurves meet contributes chords of zero length.
pub(super) fn loop_closing_chords(face: &FaceRecord) -> Result<Vec<(Vec3, Vec3)>, String> {
    let mut chords = Vec::new();
    if open_planar_loops() {
        return Ok(chords);
    }
    for loop_record in &face.loops {
        let count = loop_record.coedges.len();
        for index in 0..count {
            let [_, end] = loop_record.coedges[index].pcurve.domain()?;
            let [start, _] = loop_record.coedges[(index + 1) % count].pcurve.domain()?;
            chords.push((
                loop_record.coedges[index].pcurve.evaluate(end)?,
                loop_record.coedges[(index + 1) % count].pcurve.evaluate(start)?,
            ));
        }
    }
    Ok(chords)
}

/// [`parameter_space_area`] with every trim loop CLOSED by its chords
/// ([`loop_closing_chords`]). The mass lane of a planar face reads its area
/// here.
///
/// `½∮(u dv − v du)` is independent of where the uv origin sits only over a
/// closed path. Summed over pcurves whose ends miss one another by `δ`, it
/// reads `½(e × δ)` too much or too little at each joint `e`, so the same
/// region read about a translated parameterization gives a different area.
/// The offset shell's planar faces carry such joints — the pcurve ends of the
/// box-bore-sphere shells' opening cork miss their vertices by up to 5.6e-5
/// while the 3D vertices are exact — and on `offsetShellProblem` that alone
/// put the cork's area 6.680e-4 short and the solid's volume 4.453e-3 short of
/// a kernel-free integral. Closed, the same face reads within 1e-8 of it.
///
/// The chord closes what the face record names. The curved lane's trim
/// polygons close the same way, implicitly, which is why a curved face with
/// the same gaps already read true. [`parameter_space_area`] itself stays
/// OPEN: its other callers read its SIGN as a loop's winding, and on a
/// periodic carrier a loop that wraps once is not closed in uv by design.
pub(crate) fn closed_parameter_space_area(face: &FaceRecord) -> Result<f64, String> {
    let mut area = parameter_space_area(face)?;
    for (end, start) in loop_closing_chords(face)? {
        area += 0.5 * (end.x * start.y - start.x * end.y);
    }
    Ok(area)
}

/// The fraction of a face's own extent within which two parameter points are the SAME
/// point of the carrier. It separates a SEAM — where a loop steps from one edge of the
/// domain to the other and both parameters name one 3D point, so their evaluations agree
/// to floating-point rounding — from a trim that misses its own joint.
///
/// Set from the separation the population shows, not picked: across 92,000 readings of
/// this kernel's corpus and both suites, a seam or pole joint's two images agree to
/// ~1e-14 of the face's extent, while the smallest real miss measured is 1.06e-7 — five
/// orders above this bar, and a hundred above double-precision evaluation noise.
const SEAM_COINCIDENCE_REL: f64 = 1e-12;

/// [`parameter_space_area`] with every joint that MISSES closed by a chord, and every
/// joint that steps across a SEAM left open — the reading a caller on a carrier that may
/// be periodic or have a pole needs.
///
/// The affine lane closes every joint ([`closed_parameter_space_area`]) because a plane has
/// neither. A general carrier has both: a loop that wraps a closed direction steps a whole
/// period at one joint by design, and a chord there cancels the sweep — measured on the lib
/// suite, such a reading collapses to 5e-17 — while a pole's degenerate edge steps a whole
/// domain edge that is one point. Both are recognised the same way, and by GEOMETRY rather
/// than by a flag: the two parameters name one point or they do not. `closed_directions()`
/// is analytic-keyed and answers `false` for a FITTED periodic patch, which is exactly the
/// carrier a flag-keyed test would mistake.
///
/// Nothing is evaluated for a joint whose pcurves meet exactly, which is all but 216 of the
/// 15,443 calls one corpus replay makes through here.
pub(crate) fn closed_parameter_space_area_on_carrier(face: &FaceRecord) -> Result<f64, String> {
    let mut area = parameter_space_area(face)?;
    let mut coincidence: Option<f64> = None;
    for loop_record in &face.loops {
        let count = loop_record.coedges.len();
        for index in 0..count {
            let [_, end] = loop_record.coedges[index].pcurve.domain()?;
            let [start, _] = loop_record.coedges[(index + 1) % count].pcurve.domain()?;
            let here = loop_record.coedges[index].pcurve.evaluate(end)?;
            let next = loop_record.coedges[(index + 1) % count].pcurve.evaluate(start)?;
            if here.x == next.x && here.y == next.y {
                continue;
            }
            let bar = *coincidence.get_or_insert_with(|| {
                SEAM_COINCIDENCE_REL
                    * crate::tolerance::model_scale(
                        face.surface
                            .control_points
                            .iter()
                            .flatten()
                            .filter_map(|control| control.point().ok()),
                    )
            });
            let (image_here, image_next) = (
                face.surface.evaluate_extended(here.x, here.y)?,
                face.surface.evaluate_extended(next.x, next.y)?,
            );
            if image_here.sub(image_next).length() <= bar {
                continue;
            }
            area += 0.5 * (here.x * next.y - next.x * here.y);
        }
    }
    Ok(area)
}

pub(super) fn is_affine(surface: &NurbsSurface) -> Result<bool, String> {
    // Keep exact metric shortcuts consistent with geometry recognition.
    surface.is_affine()
}

pub(crate) fn surface_breaks(surface: &NurbsSurface) -> Result<(Vec<f64>, Vec<f64>), String> {
    let ku = crate::KnotVector::new(surface.knots_u.clone(), surface.degree_u)?;
    let kv = crate::KnotVector::new(surface.knots_v.clone(), surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let mut u = vec![u0];
    u.extend(interior_knots(&surface.knots_u, surface.degree_u));
    u.push(u1);
    let mut v = vec![v0];
    v.extend(interior_knots(&surface.knots_v, surface.degree_v));
    v.push(v1);
    Ok((u, v))
}

pub(super) fn integrand_value(kind: Integrand, point: Vec3, weighted_normal: Vec3) -> f64 {
    let (x, y, z) = (point.x, point.y, point.z);
    match kind {
        Integrand::Area => weighted_normal.length(),
        Integrand::Volume => point.dot(weighted_normal),
        Integrand::VolumeAbout(reference) => point.sub(reference).dot(weighted_normal),
        Integrand::MomentX => 0.5 * x * x * weighted_normal.x,
        Integrand::MomentY => 0.5 * y * y * weighted_normal.y,
        Integrand::MomentZ => 0.5 * z * z * weighted_normal.z,
        Integrand::SecondXX => x * x * x / 3.0 * weighted_normal.x,
        Integrand::SecondYY => y * y * y / 3.0 * weighted_normal.y,
        Integrand::SecondZZ => z * z * z / 3.0 * weighted_normal.z,
        Integrand::ProductXY => 0.5 * x * x * y * weighted_normal.x,
        Integrand::ProductXZ => 0.5 * x * x * z * weighted_normal.x,
        Integrand::ProductYZ => 0.5 * y * y * z * weighted_normal.y,
    }
}


/// Stations per direction in a fixed tensor Gauss block.
pub(super) const GAUSS_COUNT: usize = GAUSS_X.len();

/// The tensor-block station evaluator is on unless `BREP_MASS_TENSOR_BLOCK=0`
/// turns it off. It is bit-identical to the per-station path by construction
/// (`NurbsSurface::deriv1_tensor_each`), so the switch buys nothing but a
/// paired A/B timing of the two in ONE binary under ONE load — which is the
/// only honest way to quote a speed-up on a box that runs eight agents.
pub(super) fn tensor_blocks_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("BREP_MASS_TENSOR_BLOCK").as_deref() != Ok("0"))
}

/// Accumulate every `kind` over ONE parameter cell by tensor Gauss–Legendre,
/// with every station's weight scaled — the trimmed path's full cells carry
/// their winding and the sweep's orientation there. `scale` multiplies FIRST,
/// as every call site has always written the product, so a cell read at the
/// base rule is bit-identical to the one-block rule it replaces.
///
/// Identical arithmetic to the per-station loop it replaces (kept as the
/// tests' reference sweep) — the same stations in the same i-outer/j-inner
/// order, the same `weight_u * weight_v * half_u * half_v * value` product and
/// the same running sum per slot — but each of the block's u and v
/// spans/basis rows is evaluated ONCE rather than once per station it takes
/// part in, which at the base rule is eight rows each instead of sixty-four
/// evaluations.
pub(super) fn integrate_cell_scaled(
    face: &FaceRecord,
    cell: [f64; 4],
    scale: f64,
    kinds: &[Integrand],
    totals: &mut [f64],
    rules: &mut rule::SurfaceRules,
) -> Result<usize, String> {
    // A rational carrier's cell is read on the panels and at the orders its
    // own weights ask for (`rule`); a polynomial one keeps ONE base-order
    // block over the whole cell.
    let (u_panels, v_panels) = rules.panels(&face.surface, cell)?;
    let mut stations = 0;
    for u_panel in &u_panels {
        for v_panel in &v_panels {
            integrate_block(face, *u_panel, *v_panel, scale, kinds, totals)?;
            stations += u_panel.order * v_panel.order;
        }
    }
    Ok(stations)
}

/// One tensor Gauss block: every station of `u_panel × v_panel`, every kind.
fn integrate_block(
    face: &FaceRecord,
    u_panel: rule::Panel,
    v_panel: rule::Panel,
    scale: f64,
    kinds: &[Integrand],
    totals: &mut [f64],
) -> Result<(), String> {
    let half_u = (u_panel.hi - u_panel.lo) * 0.5;
    let middle_u = (u_panel.hi + u_panel.lo) * 0.5;
    let half_v = (v_panel.hi - v_panel.lo) * 0.5;
    let middle_v = (v_panel.hi + v_panel.lo) * 0.5;
    let (u_x, u_w) = u_panel.nodes();
    let (v_x, v_w) = v_panel.nodes();
    let sign = if face.same_sense { 1.0 } else { -1.0 };
    // The base rule, written against the fixed-size tables: its trip count is
    // a constant the compiler unrolls, and reading the nodes through a slice
    // of run-time length instead cost 0.28 µs a station on a 900 000-station
    // import (measured 2026-09-15). Every polynomial carrier is on this path,
    // and it is the arithmetic the one-block rule always did.
    let base_rule = u_x.len() == GAUSS_COUNT && v_x.len() == GAUSS_COUNT;
    if base_rule && tensor_blocks_on() && face.surface.deriv1_tensor_supported() {
        let mut u_stations = [0.0f64; GAUSS_COUNT];
        let mut v_stations = [0.0f64; GAUSS_COUNT];
        for index in 0..GAUSS_COUNT {
            u_stations[index] = middle_u + half_u * GAUSS_X[index];
            v_stations[index] = middle_v + half_v * GAUSS_X[index];
        }
        return face
            .surface
            .deriv1_tensor_each(&u_stations, &v_stations, |i, j, point, su, sv| {
                let weighted_normal = su.cross(sv).scale(sign);
                for (slot, kind) in kinds.iter().enumerate() {
                    totals[slot] += scale
                        * GAUSS_W[i]
                        * GAUSS_W[j]
                        * half_u
                        * half_v
                        * integrand_value(*kind, point, weighted_normal);
                }
                Ok(())
            });
    }
    if base_rule && !face.surface.deriv1_tensor_supported() {
        for i in 0..GAUSS_COUNT {
            for j in 0..GAUSS_COUNT {
                let (point, su, sv) = face
                    .surface
                    .deriv1(middle_u + half_u * GAUSS_X[i], middle_v + half_v * GAUSS_X[j])?;
                let weighted_normal = su.cross(sv).scale(sign);
                for (slot, kind) in kinds.iter().enumerate() {
                    totals[slot] += scale
                        * GAUSS_W[i]
                        * GAUSS_W[j]
                        * half_u
                        * half_v
                        * integrand_value(*kind, point, weighted_normal);
                }
            }
        }
        return Ok(());
    }
    if tensor_blocks_on() && face.surface.deriv1_tensor_supported() {
        let mut u_stations = [0.0f64; rule::MAX_ORDER];
        let mut v_stations = [0.0f64; rule::MAX_ORDER];
        for index in 0..u_x.len() {
            u_stations[index] = middle_u + half_u * u_x[index];
        }
        for index in 0..v_x.len() {
            v_stations[index] = middle_v + half_v * v_x[index];
        }
        return face.surface.deriv1_tensor_each(
            &u_stations[..u_x.len()],
            &v_stations[..v_x.len()],
            |i, j, point, su, sv| {
                let weighted_normal = su.cross(sv).scale(sign);
                for (slot, kind) in kinds.iter().enumerate() {
                    totals[slot] += scale
                        * u_w[i]
                        * v_w[j]
                        * half_u
                        * half_v
                        * integrand_value(*kind, point, weighted_normal);
                }
                Ok(())
            },
        );
    }
    for i in 0..u_x.len() {
        for j in 0..v_x.len() {
            let (point, su, sv) = face
                .surface
                .deriv1(middle_u + half_u * u_x[i], middle_v + half_v * v_x[j])?;
            let weighted_normal = su.cross(sv).scale(sign);
            for (slot, kind) in kinds.iter().enumerate() {
                totals[slot] += scale
                    * u_w[i]
                    * v_w[j]
                    * half_u
                    * half_v
                    * integrand_value(*kind, point, weighted_normal);
            }
        }
    }
    Ok(())
}

/// Every `kind` over the whole (untrimmed) knot-span grid in ONE sweep.
/// `solid_mass_properties` asks an untrimmed face for its area and its volume
/// flux; two single-kind sweeps evaluated every station twice for a second
/// integrand that costs a multiply.
pub(super) fn integrate_untrimmed_multi(
    face: &FaceRecord,
    kinds: &[Integrand],
) -> Result<Vec<f64>, String> {
    let (u_breaks, v_breaks) = surface_breaks(&face.surface)?;
    let mut totals = vec![0.0f64; kinds.len()];
    let mut rules = rule::SurfaceRules::new(&face.surface)?;
    for upair in u_breaks.windows(2) {
        for vpair in v_breaks.windows(2) {
            integrate_cell_scaled(
                face,
                [upair[0], upair[1], vpair[0], vpair[1]],
                1.0,
                kinds,
                &mut totals,
                &mut rules,
            )?;
        }
    }
    Ok(totals)
}

pub(super) fn integrate_untrimmed(face: &FaceRecord, kind: Integrand) -> Result<f64, String> {
    Ok(integrate_untrimmed_multi(face, &[kind])?[0])
}

/// A bi-periodic band face (surface closed in both u and v, exactly two loops
/// each a full-wrap constant-cross-level rim) whose trim is NOT expressed with a
/// seam ruling — the fillet-torus bands OCC/STEP emit as two rim circles. Such a
/// face integrates to ZERO on the trimmed path (both loops are degenerate iso
/// lines in parameter space), so it is handled analytically over the seam-cut
/// rectangle instead. See [`biperiodic_band_range`].
#[derive(Clone, Copy)]
pub(super) struct BiBand {
    /// Which parameter is the periodic (full-wrap) one.
    pub(super) p_is_u: bool,
    /// The two rim cross-levels, ascending (q_lo <= q_hi) in the cross param.
    pub(super) q_lo: f64,
    pub(super) q_hi: f64,
    /// True when the material band is the CROSS-SEAM COMPLEMENT of [q_lo, q_hi]
    /// rather than the between-rims strip (mirrors the tessellator's
    /// `close_periodic_trim_dir` orientation test).
    pub(super) complement: bool,
    /// The larger of the two rims' cross-parameter extents, as a fraction of
    /// the cross domain. Zero (to double noise) for the rim circles OCC/STEP
    /// emit as exact iso lines; a few 1e-3 for a rim PROJECTED from a vendor
    /// 3D curve that sits off the carrier, whose pcurve then wanders in the
    /// cross parameter. Decides between the seam-cut rectangle and the
    /// rim-following strip — see [`biperiodic_band_integral`].
    pub(super) rim_wobble: f64,
}

/// Above this fraction of the cross domain a rim is not an iso line, and the
/// band between the rims is integrated along the rims' own pcurves instead
/// of over the rectangle between their mean levels.
///
/// The bar is double noise, not a geometric tolerance: a rim that IS an iso
/// line measures ~1e-15 here, and the two paths agree on it to quadrature
/// precision, so nothing that reads exactly today moves. Set at 1e-9 rather
/// than at the noise floor so a level rim of a large-domain surface is never
/// pushed onto the pcurve walk by rounding alone. `importTestWorking`
/// SOLID_03 face 90: a torus fillet strip whose upper rim was projected from
/// an edge 2.2e-3 mm off the torus wanders over v ∈ [0.746903, 0.750195] —
/// 3.3e-3 of the period — and the rectangle at the mean level 0.748549 read
/// the strip 9.67e-3 mm² (0.09%) small and its origin flux 0.388 low.
pub(super) const BAND_RIM_LEVEL_TOLERANCE_REL: f64 = 1e-9;

/// Detect the bi-periodic band configuration of `face` and choose which of the
/// two regions its rims bound is material. The choice replicates the watertight
/// tessellator (`watertight_tessellation::close_periodic_trim_dir`): the
/// material is to the LEFT of the boundary traversal, so the between-rims strip
/// is kept unless the cross direction wraps AND the rim senses conclusively name
/// the complement. Returns None for anything that is not a clean two-rim band.
pub(super) fn biperiodic_band_range(face: &FaceRecord) -> Result<Option<BiBand>, String> {
    let surface = &face.surface;
    let (closed_u, closed_v) = surface.closed_directions()?;
    if !(closed_u && closed_v) {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let u_span = (u1 - u0).abs().max(1e-30);
    let v_span = (v1 - v0).abs().max(1e-30);
    // Sample each loop's pcurves (in coedge/traversal order) into (u,v) points.
    // A DEGENERATE loop — a pole/apex vertex whose whole trace collapses to one
    // parameter point (a fat fillet torus touches its outer equator at a single
    // seam point: ABC 00000039 faces 68/70) — is not a band boundary; drop it
    // and keep the two real rims. Anything else leaves the band ambiguous.
    let mut loop_points: Vec<Vec<[f64; 2]>> = Vec::with_capacity(2);
    for loop_record in &face.loops {
        let mut points = Vec::new();
        for coedge in &loop_record.coedges {
            let [d0, d1] = coedge.pcurve.domain()?;
            // 32 per coedge: enough to see a projected rim wander, which a
            // 12-sample read of a smooth pcurve could straddle.
            let samples = 32;
            for k in 0..=samples {
                let t = d0 + (d1 - d0) * k as f64 / samples as f64;
                let p = coedge.pcurve.evaluate(t)?;
                points.push([p.x, p.y]);
            }
        }
        if points.len() < 2 {
            return Ok(None);
        }
        let (mut umin, mut umax, mut vmin, mut vmax) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for pt in &points {
            umin = umin.min(pt[0]);
            umax = umax.max(pt[0]);
            vmin = vmin.min(pt[1]);
            vmax = vmax.max(pt[1]);
        }
        if (umax - umin) <= 1e-3 * u_span && (vmax - vmin) <= 1e-3 * v_span {
            continue; // degenerate pole/apex loop
        }
        loop_points.push(points);
    }
    if loop_points.len() != 2 {
        return Ok(None);
    }
    for p_is_u in [true, false] {
        let (period, _p0, _p1) = if p_is_u {
            (u1 - u0, u0, u1)
        } else {
            (v1 - v0, v0, v1)
        };
        let (q_dom_lo, q_dom_hi) = if p_is_u { (v0, v1) } else { (u0, u1) };
        let q_extent = (q_dom_hi - q_dom_lo).abs().max(1e-30);
        if !(period > 0.0) {
            continue;
        }
        let coord = |pt: &[f64; 2]| -> (f64, f64) {
            if p_is_u {
                (pt[0], pt[1])
            } else {
                (pt[1], pt[0])
            }
        };
        // (level, winding-direction) for each rim; None if any loop is not a
        // clean full-wrap constant-cross-level rim in this p direction.
        let mut rings: Vec<(f64, i32)> = Vec::with_capacity(2);
        let mut clean = true;
        let mut rim_wobble = 0.0f64;
        for points in &loop_points {
            let (mut pmin, mut pmax, mut qmin, mut qmax) = (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            );
            for pt in points {
                let (p, q) = coord(pt);
                pmin = pmin.min(p);
                pmax = pmax.max(p);
                qmin = qmin.min(q);
                qmax = qmax.max(q);
            }
            if (pmax - pmin) < 0.6 * period || (qmax - qmin) > 0.05 * q_extent {
                clean = false;
                break;
            }
            rim_wobble = rim_wobble.max((qmax - qmin) / q_extent);
            let mut net = 0.0;
            for pair in points.windows(2) {
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
            rings.push((0.5 * (qmin + qmax), direction));
        }
        if !clean || rings.len() != 2 {
            continue;
        }
        rings.sort_by(|a, b| a.0.total_cmp(&b.0));
        let (q_lo, lower_dir) = rings[0];
        let (q_hi, upper_dir) = rings[1];
        let inconclusive = lower_dir == 0 || upper_dir == 0 || lower_dir == upper_dir;
        let between_rims_is_ccw_uv = if p_is_u { lower_dir > 0 } else { lower_dir < 0 };
        let complement = !inconclusive && between_rims_is_ccw_uv != face.same_sense;
        return Ok(Some(BiBand {
            p_is_u,
            q_lo,
            q_hi,
            complement,
            rim_wobble,
        }));
    }
    Ok(None)
}

/// Gauss–Legendre integral of `kind` over the axis-aligned parameter rectangle
/// [u_lo,u_hi]×[v_lo,v_hi], subdividing at knot spans clipped to the rectangle.
pub(super) fn integrate_rectangle(
    face: &FaceRecord,
    u_lo: f64,
    u_hi: f64,
    v_lo: f64,
    v_hi: f64,
    kind: Integrand,
) -> Result<f64, String> {
    let (u_full, v_full) = surface_breaks(&face.surface)?;
    let clamp = |breaks: &[f64], lo: f64, hi: f64| -> Vec<f64> {
        let eps = 1e-9 * (hi - lo).abs().max(1e-30);
        let mut out = vec![lo];
        for &b in breaks {
            if b > lo + eps && b < hi - eps {
                out.push(b);
            }
        }
        out.push(hi);
        out
    };
    let u_breaks = clamp(&u_full, u_lo, u_hi);
    let v_breaks = clamp(&v_full, v_lo, v_hi);
    let mut totals = [0.0f64];
    let mut rules = rule::SurfaceRules::new(&face.surface)?;
    for upair in u_breaks.windows(2) {
        for vpair in v_breaks.windows(2) {
            integrate_cell_scaled(
                face,
                [upair[0], upair[1], vpair[0], vpair[1]],
                1.0,
                &[kind],
                &mut totals,
                &mut rules,
            )?;
        }
    }
    Ok(totals[0])
}

/// Integrate each of `kinds` over a bi-periodic band face, or return None when
/// `face` is not such a band. The between-rims strip integrates over the full
/// periodic span × the cross-level strip; the complement is the full-domain
/// integral MINUS that strip (the two tile the closed cross period, so no
/// domain-extended evaluation is needed).
///
/// Two readings of the strip. Rims that ARE iso lines (every rim circle a
/// kernel-built or exactly exported torus carries) integrate over the
/// seam-cut rectangle between their levels. A rim that wanders in the cross
/// parameter — a pcurve PROJECTED from a vendor edge that sits off the
/// carrier — is not at any one level, and the rectangle at its mean level
/// mis-reads the strip by the wander times the rim length; such a band is
/// integrated along the rims' own pcurves by the winding boundary integral
/// (`∮ −G du`, [`super::winding::winding_loops_integral`]), which reads the
/// trim exactly as given. The material choice (`complement`) is the same in
/// both readings; only the strip's quadrature differs.
pub(super) fn biperiodic_band_integral(
    face: &FaceRecord,
    kinds: &[Integrand],
) -> Result<Option<Vec<f64>>, String> {
    let Some(band) = biperiodic_band_range(face)? else {
        return Ok(None);
    };
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    let (u_lo, u_hi, v_lo, v_hi) = if band.p_is_u {
        (u0, u1, band.q_lo, band.q_hi)
    } else {
        (band.q_lo, band.q_hi, v0, v1)
    };
    let strips: Vec<f64> = if band.rim_wobble <= BAND_RIM_LEVEL_TOLERANCE_REL {
        kinds
            .iter()
            .map(|&kind| integrate_rectangle(face, u_lo, u_hi, v_lo, v_hi, kind))
            .collect::<Result<_, _>>()?
    } else {
        super::winding::winding_loops_integral(face, kinds, band.p_is_u, 1)?
    };
    let mut out = Vec::with_capacity(kinds.len());
    for (&kind, strip) in kinds.iter().zip(strips) {
        let value = if band.complement {
            integrate_untrimmed(face, kind)? - strip
        } else {
            strip
        };
        out.push(value);
    }
    Ok(Some(out))
}
