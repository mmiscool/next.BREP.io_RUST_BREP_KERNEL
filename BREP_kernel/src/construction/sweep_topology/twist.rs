use super::*;

/// The prefix of the refusal for a requested twist that does not close a CLOSED
/// path, so a caller can classify one without matching free text.
pub const SWEEP_TWIST_CLOSURE_REFUSAL: &str =
    "sweepSolid: a requested twist does not close this closed path";

/// What a REQUESTED twist does on a closed path, once it is known to close.
///
/// `twist` is the requested angle SNAPPED to the symmetry step it matched at
/// round-off (`2π·n/k`), and `shift` is how far along the section's curve list
/// one lap carries each curve: the wall leaving curve `j` arrives on curve
/// `(j + shift) mod m`. A whole number of turns, or no twist, is shift 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct ClosingTwist {
    pub twist: f64,
    pub shift: usize,
}

/// Which REQUESTED twists close a closed sweep of `section`, and what the one asked
/// for does — or the refusal naming the nearest that close.
///
/// WHAT CLOSES. After one lap the frame's own roll, the path's holonomy, is taken
/// out by the counter-twist, so the section comes back rolled by exactly the
/// requested twist `T` about the PATH at station 0. The sweep closes only if that
/// roll carries the section onto ITSELF. So `T` must be a rotational symmetry of
/// the section about the path's own axis, `T ≡ 2π·n/k (mod 2π)` for a section with
/// `k` such rotations. `k = 1` for any section with no symmetry about the path,
/// including a symmetric one drawn off it, so only whole turns close there.
///
/// READ CURVE FOR CURVE, AS DRAWN. A wall per section curve must land on another
/// curve after the lap, end for end, with the same degree, knots and weights, so the
/// rotation is found on the curves' control points and not on the region they
/// bound. A circle drawn as two half-arcs has `k = 2`, not a continuum; a square
/// drawn with one side split has `k = 1`.
///
/// `axis_point` and `axis` are the path's first point and unit tangent there, and
/// the rotation is right-handed about `axis`, the sense the frame rolls in.
/// `floor` is the lane's own position floor, which is how far a control point may
/// land from its image and still be the same point. The angle itself is matched at
/// ROUND-OFF, not at a size: a twist that misses a closing value by `δ` would put
/// the seam `ρ·δ` apart, which grows with the section, so `90.001°` is refused
/// on a square.
pub(super) fn closing_twist(
    section: &[NurbsCurve],
    axis_point: Vec3,
    axis: Vec3,
    requested: f64,
    floor: f64,
) -> Result<ClosingTwist, KernelRefusal> {
    use std::f64::consts::TAU;
    if requested == 0.0 {
        return Ok(ClosingTwist {
            twist: 0.0,
            shift: 0,
        });
    }
    if !requested.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "twist_finite",
            "sweepSolid: twist angle must be finite",
        ));
    }
    let symmetry = section_symmetry(section, axis_point, axis, floor)?;
    let order = symmetry.len();
    let step = TAU / order as f64;
    let turns = (requested / step).round();
    // Snapped in LOWEST TERMS, so every loop of a profile that closes on the same
    // angle lays down the same bits: a hole with 6 rotations and an outer loop
    // with 3 both read 120° as `TAU·1/3`, and their shares, swept in calls of
    // their own, agree exactly.
    let common = {
        let (mut a, mut b) = ((turns as i64).unsigned_abs(), order as u64);
        while b != 0 {
            (a, b) = (b, a % b);
        }
        a.max(1) as f64
    };
    let snapped = TAU * (turns / common) / (order as f64 / common);
    if (requested - snapped).abs() <= 64.0 * f64::EPSILON * requested.abs().max(1.0) {
        let index = (turns as i64).rem_euclid(order as i64) as usize;
        return Ok(ClosingTwist {
            twist: snapped,
            shift: symmetry[index],
        });
    }
    let below = TAU * (requested / step).floor() / order as f64;
    let above = TAU * (requested / step).ceil() / order as f64;
    let reading = if order == 1 {
        "this section has NO rotational symmetry about the path, so only a whole number of \
         turns carries it onto itself"
            .to_string()
    } else {
        format!(
            "this section maps onto itself under {order} rotations about the path, so a twist \
             closes only in steps of {:.6}°",
            step.to_degrees()
        )
    };
    Err(KernelRefusal::input(
        KernelStage::Classify,
        "twist_closure",
        format!(
        "{SWEEP_TWIST_CLOSURE_REFUSAL}: the section comes back from one lap rolled about the path \
         by the requested {:.6}° (the frame's own roll, its holonomy, is closed separately by the \
         counter-twist), and a roll that does not carry the section onto itself leaves the last \
         wall meeting the first one turned. The symmetry is read off the section's boundary \
         curves as drawn, each curve landing on another end for end, about the path's own axis \
         (a section drawn off the path turns about the path, not about its centroid): {reading}. \
         The nearest twists that close are {:.6}° and {:.6}°",
        requested.to_degrees(),
        below.to_degrees(),
        above.to_degrees(),
        ),
    ))
}

/// The ROTATIONAL SYMMETRY of `section` about the line through `axis_point` along
/// unit `axis`: entry `i` is the curve shift that the rotation by `2π·i/k` applies,
/// for the `k` rotations that carry the section's curve list onto itself. Entry 0
/// is the identity, so the result is never empty.
///
/// A candidate shift `s` must map every curve `j` onto curve `(j + s) mod m` with
/// the same degree, knots and weights. The angle is read off the control point
/// farthest from the axis and then checked on every control point against
/// `floor`. A shift whose angle does not sit on its own multiple of `2π/k` is
/// dropped. A section that is symmetric only to within the floor then reads as
/// less symmetric than it looks, and a twist is refused rather than welded.
fn section_symmetry(
    section: &[NurbsCurve],
    axis_point: Vec3,
    axis: Vec3,
    floor: f64,
) -> Result<Vec<usize>, KernelRefusal> {
    use std::f64::consts::TAU;
    let count = section.len();
    let axis = axis.normalized().map_err(|_| {
        KernelRefusal::internal(
            KernelStage::Classify,
            "symmetry_axis",
            "sweepSolid: the path tangent is degenerate at its start",
        )
    })?;
    let radial = |point: Vec3| {
        let offset = point.sub(axis_point);
        offset.sub(axis.scale(offset.dot(axis)))
    };
    // The control point farthest from the axis fixes each candidate's angle.
    let mut reach = 0.0_f64;
    let mut farthest = (0, 0);
    for (curve_index, curve) in section.iter().enumerate() {
        for (control_index, control) in curve.control_points.iter().enumerate() {
            let distance = radial(control.point().or_refuse(KernelStage::Classify, "control_point")?).length();
            if distance > reach {
                reach = distance;
                farthest = (curve_index, control_index);
            }
        }
    }
    let mut found: Vec<(usize, f64)> = vec![(0, 0.0)];
    if reach <= floor {
        return Ok(vec![0]);
    }
    let rotate = |point: Vec3, angle: f64| {
        let offset = point.sub(axis_point);
        let (sin, cos) = angle.sin_cos();
        axis_point.add(
            offset
                .scale(cos)
                .add(axis.cross(offset).scale(sin))
                .add(axis.scale(axis.dot(offset) * (1.0 - cos))),
        )
    };
    'shift: for shift in 1..count {
        for index in 0..count {
            let (a, b) = (&section[index], &section[(index + shift) % count]);
            if a.degree != b.degree
                || a.control_points.len() != b.control_points.len()
                || a.knots.len() != b.knots.len()
                || a.knots
                    .iter()
                    .zip(&b.knots)
                    .any(|(x, y)| (x - y).abs() > 1e-12 * (1.0 + x.abs()))
                || a.control_points
                    .iter()
                    .zip(&b.control_points)
                    .any(|(x, y)| (x.w - y.w).abs() > 1e-12 * x.w.abs().max(1.0))
            {
                continue 'shift;
            }
        }
        let from = radial(section[farthest.0].control_points[farthest.1].point().or_refuse(KernelStage::Classify, "control_point")?);
        let to = radial(section[(farthest.0 + shift) % count].control_points[farthest.1].point().or_refuse(KernelStage::Classify, "control_point")?);
        let angle = axis.dot(from.cross(to)).atan2(from.dot(to)).rem_euclid(TAU);
        for index in 0..count {
            let (a, b) = (&section[index], &section[(index + shift) % count]);
            for (x, y) in a.control_points.iter().zip(&b.control_points) {
                if rotate(x.point().or_refuse(KernelStage::Classify, "control_point")?, angle).sub(y.point().or_refuse(KernelStage::Classify, "control_point")?).length() > floor {
                    continue 'shift;
                }
            }
        }
        found.push((shift, angle));
    }
    // The rotations form a cyclic group of order k: each must sit on its own
    // multiple of 2π/k, and no two on the same one.
    let order = found.len();
    let mut shifts = vec![None; order];
    for &(shift, angle) in &found {
        let step = (angle * order as f64 / TAU).round();
        let miss = (angle - TAU * step / order as f64).abs() * reach;
        let index = (step as i64).rem_euclid(order as i64) as usize;
        if miss > floor || shifts[index].is_some() {
            return Ok(vec![0]);
        }
        shifts[index] = Some(shift);
    }
    Ok(shifts.into_iter().map(|shift| shift.unwrap_or(0)).collect())
}
