use super::*;
use crate::env_read::Mode;

/// Lift each rim continuously to the covering plane before taking a boundary
/// primitive. Integer translations of individual coedges must not change the
/// material. Disjoint cross-coordinate control hulls certify two separate
/// rims; no constant-level or small-wander assumption is needed.
pub fn lifted_band_integral(
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
        let mode = group_scale_mode();
        let groups: Vec<u8> = kinds.iter().map(|&kind| convergence_group(kind)).collect();
        // Check mode: where the group bar would have stopped, and its values.
        let mut group_stop: Option<(usize, Vec<f64>)> = None;
        for refine in [2, 4, 8, 16, 32] {
            let values = super::winding::lifted_loops_integral(&lifted, kinds, p_is_u, refine)?;
            let component = values.iter().zip(&previous).all(|(a, b)| (a - b).abs() <= 1e-12 * a.abs().max(1.0));
            let grouped = mode != Mode::Off && group_converged(&values, &previous, &groups);
            previous = values;
            if mode == Mode::Check && grouped && group_stop.is_none() {
                group_stop = Some((refine, previous.clone()));
            }
            if if mode == Mode::On { grouped } else { component } { break; }
        }
        if let Some((refine, stopped)) = group_stop {
            let scales = group_scales(&previous, &groups);
            let worst = stopped.iter().zip(&previous).zip(&scales).map(|((a, b), scale)| (a - b).abs() / scale).fold(0.0f64, f64::max);
            eprintln!("BREP_MASS_TORUS_LIFT_GROUP_SCALE check: face {} group bar stops at refine {refine}, its values differ from the component bar's by {worst:.3e} of their group scale", face.id);
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

/// `BREP_MASS_TORUS_LIFT_GROUP_SCALE` (default on since 2026-11-24; `0|off`
/// restores the per-component bar; `check` keeps that bar and prints where
/// the group bar would have stopped): the lifted band's
/// refinement stops when two readings agree to 1e-12 of each component's
/// GROUP scale, the largest magnitude among the requested kinds of the same
/// unit (the vector area's three components, the first moments, the second
/// moments and products), instead of each component's own value. A vector
/// component near zero carries the rounding of the vector's whole length:
/// on a band whose rim is 218 projected edges, the vector area (-35.4,
/// -46624.4, 0.125) agreed to 1e-14 of its length at the first doubling, but
/// its 0.125 component read 5e-10 apart at every level, so all five levels
/// ran (63 times the first reading's cost) for no change in any value. The
/// group bar is also rotation invariant, which the per-component one is not.
pub fn group_scale_mode() -> Mode {
    crate::mode_switch!("BREP_MASS_TORUS_LIFT_GROUP_SCALE", true)
}


/// The unit group of an integrand: kinds in one group are compared against
/// one scale (area; volume; vector area; first moments; second moments and
/// products).
pub fn convergence_group(kind: Integrand) -> u8 {
    match kind {
        Integrand::Area => 0,
        Integrand::Volume | Integrand::VolumeAbout(_) => 1,
        Integrand::NormalX | Integrand::NormalY | Integrand::NormalZ => 2,
        Integrand::MomentX | Integrand::MomentY | Integrand::MomentZ | Integrand::MomentAbout(..) => 3,
        Integrand::SecondXX | Integrand::SecondYY | Integrand::SecondZZ
        | Integrand::ProductXY | Integrand::ProductXZ | Integrand::ProductYZ
        | Integrand::SecondAbout(..) | Integrand::ProductAbout(..) => 4,
    }
}

/// Each component's group scale: the largest magnitude in its group, at
/// least 1 (the per-component bar's absolute floor).
pub fn group_scales(values: &[f64], groups: &[u8]) -> Vec<f64> {
    groups
        .iter()
        .map(|group| {
            values.iter().zip(groups).filter(|(_, other)| *other == group).map(|(value, _)| value.abs()).fold(1.0f64, f64::max)
        })
        .collect()
}

/// Two readings agree to 1e-12 of every component's group scale. A
/// non-finite reading never agrees.
pub fn group_converged(values: &[f64], previous: &[f64], groups: &[u8]) -> bool {
    let scales = group_scales(values, groups);
    values.iter().zip(previous).zip(&scales).all(|((a, b), scale)| (a - b).abs() <= 1e-12 * scale)
}
