use super::*;

/// Lift each rim continuously to the covering plane before taking a boundary
/// primitive. Integer translations of individual coedges must not change the
/// material. Disjoint cross-coordinate control hulls certify two separate
/// rims; no constant-level or small-wander assumption is needed.
pub(super) fn lifted_band_integral(
    face: &FaceRecord,
    kinds: &[Integrand],
) -> Result<Option<Vec<f64>>, String> {
    if face.surface.closed_directions()? != (true, true) || face.loops.len() != 2 {
        return Ok(None);
    }
    let domains = [face.surface.domain_u()?, face.surface.domain_v()?];
    let periods = [domains[0][1] - domains[0][0], domains[1][1] - domains[1][0]];
    for p_is_u in [true, false] {
        let q = if p_is_u { 1 } else { 0 };
        let p = 1 - q;
        let coordinate = |point: Vec3| [point.x, point.y];
        let mut lifted = face.clone();
        let mut bounds = Vec::new();
        let mut directions = Vec::new();
        let mut eligible = true;
        for rim in &mut lifted.loops {
            let Some(first) = rim.coedges.first() else { eligible = false; break; };
            let original_start = coordinate(first.pcurve.evaluate(first.pcurve.domain()?[0])?);
            let start = [
                domains[0][0] + (original_start[0] - domains[0][0]).rem_euclid(periods[0]),
                domains[1][0] + (original_start[1] - domains[1][0]).rem_euclid(periods[1]),
            ];
            let mut cursor = start;
            let mut lo = f64::INFINITY;
            let mut hi = f64::NEG_INFINITY;
            let mut curves = Vec::new();
            for coedge in &rim.coedges {
                let [a, b] = coedge.pcurve.domain()?;
                let original = coordinate(coedge.pcurve.evaluate(a)?);
                let shift = [
                    ((cursor[0] - original[0]) / periods[0]).round() * periods[0],
                    ((cursor[1] - original[1]) / periods[1]).round() * periods[1],
                ];
                let mut curve = coedge.clone();
                for control in &mut curve.pcurve.control_points {
                    if control.w <= 0.0 { eligible = false; break; }
                    control.x += shift[0] * control.w;
                    control.y += shift[1] * control.w;
                    let value = if q == 0 { control.x / control.w } else { control.y / control.w };
                    lo = lo.min(value);
                    hi = hi.max(value);
                }
                let next = coordinate(curve.pcurve.evaluate(a)?);
                // Close real admitted joins, as the trimmed polygon route does.
                // A seam jump has already disappeared in the integer lift.
                if cursor != next {
                    let mut bridge = coedge.clone();
                    bridge.pcurve = crate::make_line(Vec3::new(cursor[0], cursor[1], 0.0), Vec3::new(next[0], next[1], 0.0))?;
                    curves.push(bridge);
                }
                cursor = coordinate(curve.pcurve.evaluate(b)?);
                curves.push(curve);
            }
            if !eligible { break; }
            let travel = [cursor[0] - start[0], cursor[1] - start[1]];
            let wind = (travel[p] / periods[p]).round();
            if wind.abs() != 1.0 || (travel[p] / periods[p] - wind).abs() > 1e-3
                || (travel[q] / periods[q]).abs() > 1e-3 || hi - lo >= periods[q] {
                eligible = false;
                break;
            }
            let mut end = start;
            end[p] += wind * periods[p];
            if cursor != end {
                let mut bridge = curves[0].clone();
                bridge.pcurve = crate::make_line(Vec3::new(cursor[0], cursor[1], 0.0), Vec3::new(end[0], end[1], 0.0))?;
                curves.push(bridge);
            }
            rim.coedges = curves;
            bounds.push((lo, hi));
            directions.push(wind);
        }
        if !eligible || directions[0] != -directions[1] { continue; }
        // Put the second rim wholly above the first, within one cross period.
        // If their control hulls overlap in every lift, decline this band path.
        let shift = ((bounds[0].1 - bounds[1].0) / periods[q]).ceil() * periods[q];
        if bounds[1].1 + shift >= bounds[0].0 + periods[q] { continue; }
        for coedge in &mut lifted.loops[1].coedges {
            for control in &mut coedge.pcurve.control_points {
                if q == 0 { control.x += shift * control.w; }
                else { control.y += shift * control.w; }
            }
        }
        let between_ccw = if p_is_u { directions[0] > 0.0 } else { directions[0] < 0.0 };
        let complement = between_ccw != face.same_sense;
        let mut previous = super::winding::lifted_loops_integral(&lifted, kinds, p_is_u, 1)?;
        for refine in [2, 4, 8, 16, 32] {
            let values = super::winding::lifted_loops_integral(&lifted, kinds, p_is_u, refine)?;
            let converged = values.iter().zip(&previous).all(|(a, b)| (a - b).abs() <= 1e-12 * a.abs().max(1.0));
            previous = values;
            if converged { break; }
        }
        if complement {
            for (value, &kind) in previous.iter_mut().zip(kinds) {
                *value = super::integration::integrate_untrimmed(face, kind)? - *value;
            }
        }
        return Ok(Some(previous));
    }
    Ok(None)
}
