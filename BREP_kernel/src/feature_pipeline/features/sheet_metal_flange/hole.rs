use super::*;

// ---------------------------------------------------------------------------
// Hole-edge grafting
// ---------------------------------------------------------------------------

pub(super) fn graft_hole_flange(
    flat: &mut Flat,
    hole: usize,
    segment: usize,
    geometry: &FlangeParams,
    bend_id: String,
    child_id: String,
) -> Result<(), String> {
    if hole >= flat.holes.len() {
        return Err(format!(
            "sheet-metal flange: hole {hole} not found on flat `{}`",
            flat.id
        ));
    }
    // Only a THROUGH hole has a foldable rim — a blind pocket's rim sits on a
    // floor, folding it is geometric nonsense.
    {
        let record = &flat.holes[hole];
        let half_t = geometry.thickness * 0.5;
        let eps = 1e-9 * geometry.thickness.max(1.0);
        let through = record.z_min.map_or(true, |z| z <= -half_t + eps)
            && record.z_max.map_or(true, |z| z >= half_t - eps);
        if !through {
            return Err(format!(
                "sheet-metal flange: hole {hole} is a blind pocket (span {:?}..{:?}) — only THROUGH hole rims can fold",
                record.z_min, record.z_max
            ));
        }
    }

    // --- Circular loop → collar ---
    if let Some((center, radius)) = sheet_metal::evaluate::circle_of_loop(&flat.holes[hole].outer) {
        if geometry.setback_start > GEOM_EPS || geometry.setback_end > GEOM_EPS {
            return Err(
                "sheet-metal flange: setbacks are not supported on a circular hole flange (no arc-span collars yet)"
                    .into(),
            );
        }
        if flat.hole_bends.iter().any(|hb| hb.hole == hole) {
            return Err(format!(
                "sheet-metal flange: hole {hole} already carries a flange"
            ));
        }
        // The inset/offset shift GROWS the rim radius (inward = into the
        // material = away from the opening), so the default material_inside
        // collar lands its finished bore back on the original hole radius.
        let rim = radius + geometry.shift;
        let clearance = geometry.inside_radius + geometry.thickness;
        if !(rim - clearance > 1e-6) {
            return Err(format!(
                "sheet-metal flange: collar needs bend radius + thickness ({clearance:.4}) smaller than the inset-shifted hole radius ({rim:.4})"
            ));
        }
        if geometry.shift.abs() > GEOM_EPS {
            // Rebuild only the OUTER loop at the shifted rim; islands and the
            // (through) span are untouched.
            flat.holes[hole].outer = vec![crate::make_circle(
                Vec3::new(center[0], center[1], 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                rim,
            )?];
        }
        flat.hole_bends.push(HoleBend {
            id: bend_id,
            hole,
            segment: 0, // a collar wraps the whole loop; the picked segment is irrelevant
            reversed: false,
            angle_deg: geometry.signed_angle,
            inside_radius: geometry.inside_radius,
            k_factor: geometry.k_factor,
            kind: HoleBendKind::Collar { leg: geometry.leg },
        });
        return Ok(());
    }

    // --- Straight window segment → flap into the opening ---
    let curve = flat.holes[hole].outer.get(segment).ok_or_else(|| {
        format!(
            "sheet-metal flange: segment {segment} not found on hole {hole} of flat `{}`",
            flat.id
        )
    })?;
    if !curve_is_straight(curve)? {
        return Err(format!(
            "sheet-metal flange: hole {hole} segment {segment} is not straight — only straight window segments and full circles fold"
        ));
    }
    if flat
        .hole_bends
        .iter()
        .any(|hb| hb.hole == hole && hb.segment == segment)
    {
        return Err(format!(
            "sheet-metal flange: hole {hole} segment {segment} already carries a flange"
        ));
    }
    let domain = curve.domain()?;
    let start = curve.evaluate(domain[0])?;
    let end = curve.evaluate(domain[1])?;
    let a = [start.x, start.y];
    let b = [end.x, end.y];
    let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
    if !(length > GEOM_EPS) {
        return Err(format!(
            "sheet-metal flange: hole {hole} segment {segment} is degenerate"
        ));
    }
    // Setbacks measure along the STORED segment direction.
    let s0 = geometry.setback_start;
    let s1 = length - geometry.setback_end;
    if !(s1 - s0 > MIN_SPAN) {
        return Err(format!(
            "sheet-metal flange: setbacks {} + {} consume the {length:.4}-long hole segment",
            geometry.setback_start, geometry.setback_end
        ));
    }

    // Orientation: the evaluator folds toward the RIGHT of the (a,b) direction
    // (ê × n̂), and the flap must leave through the OPENING — so reverse when
    // the loop winds CCW (opening on the LEFT of stored travel). The material
    // direction m̂ (for the inset notch) is the opposite side.
    let ccw = hole_loop_is_ccw(&flat.holes[hole].outer)?;
    let t = [(b[0] - a[0]) / length, (b[1] - a[1]) / length];
    let left = [-t[1], t[0]];
    let m = if ccw { [-left[0], -left[1]] } else { left };
    let shifted = geometry.shift.abs() > GEOM_EPS;
    let has_rim0 = s0 > GEOM_EPS;
    let has_rim1 = length - s1 > GEOM_EPS;

    let final_segment = if shifted || has_rim0 || has_rim1 {
        // Replace the picked line with the notched/split chain (all exact
        // lines). No vertex absorption here: hole neighbours may be arcs, and
        // a perpendicular jog at a shared vertex stays a simple step.
        let q = |s: f64, d: f64| {
            Vec3::new(
                a[0] + t[0] * s + m[0] * d,
                a[1] + t[1] * s + m[1] * d,
                0.0,
            )
        };
        let mut replacement: Vec<NurbsCurve> = Vec::new();
        let mut cursor = Vec3::new(a[0], a[1], 0.0);
        if has_rim0 {
            let p = q(s0, 0.0);
            replacement.push(make_line(cursor, p)?);
            cursor = p;
        }
        if shifted {
            let p = q(s0, geometry.shift);
            replacement.push(make_line(cursor, p)?);
            cursor = p;
        }
        let bend_offset = replacement.len();
        let bend_end = q(s1, if shifted { geometry.shift } else { 0.0 });
        replacement.push(make_line(cursor, bend_end)?);
        cursor = bend_end;
        if shifted {
            let p = q(s1, 0.0);
            replacement.push(make_line(cursor, p)?);
            cursor = p;
        }
        if has_rim1 {
            replacement.push(make_line(cursor, Vec3::new(b[0], b[1], 0.0))?);
        }
        let added = replacement.len() - 1;
        flat.holes[hole].outer.splice(segment..=segment, replacement);
        // Later segments of this loop shifted — keep other bends anchored.
        for hb in &mut flat.hole_bends {
            if hb.hole == hole && hb.segment > segment {
                hb.segment += added;
            }
        }
        segment + bend_offset
    } else {
        segment
    };

    // Bend relief at setback band ends: the window-rim analogue of the outline
    // case — the slot cuts from the window rim into the plate (m̂ = material
    // side), appended as its own Hole so hole/bend indices stay stable.
    if geometry.relief != ReliefType::None {
        if geometry.setback_start > GEOM_EPS {
            add_relief_slot(flat, geometry, a, t, m, s0, ReliefSide::Start)?;
        }
        if geometry.setback_end > GEOM_EPS {
            add_relief_slot(flat, geometry, a, t, m, s1, ReliefSide::End)?;
        }
    }

    let child = corner_child_flat(
        child_id,
        geometry.leg,
        s1 - s0,
        WallEnd::Flat,
        WallEnd::Flat,
        &bend_id,
    )?;
    flat.hole_bends.push(HoleBend {
        id: bend_id,
        hole,
        segment: final_segment,
        reversed: ccw,
        angle_deg: geometry.signed_angle,
        inside_radius: geometry.inside_radius,
        k_factor: geometry.k_factor,
        kind: HoleBendKind::Straight {
            child: Box::new(child),
        },
    });
    Ok(())
}

/// True iff a curve is (numerically) straight — its mid-domain point lies on
/// the chord between its endpoints.
fn curve_is_straight(curve: &NurbsCurve) -> Result<bool, String> {
    let domain = curve.domain()?;
    let start = curve.evaluate(domain[0])?;
    let end = curve.evaluate(domain[1])?;
    let mid = curve.evaluate((domain[0] + domain[1]) * 0.5)?;
    let chord = end.sub(start);
    let chord_length = chord.length();
    if chord_length < 1e-12 {
        return Ok(false);
    }
    let offset = mid.sub(start).cross(chord.scale(1.0 / chord_length)).length();
    Ok(offset <= 1e-9 * chord_length.max(1.0))
}

/// Loop winding by the shoelace over dense samples: positive area = CCW.
fn hole_loop_is_ccw(curves: &[NurbsCurve]) -> Result<bool, String> {
    let mut points: Vec<[f64; 2]> = Vec::new();
    for curve in curves {
        let domain = curve.domain()?;
        for step in 0..8 {
            let p = curve.evaluate(domain[0] + (domain[1] - domain[0]) * step as f64 / 8.0)?;
            points.push([p.x, p.y]);
        }
    }
    let mut area = 0.0;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        area += a[0] * b[1] - b[0] * a[1];
    }
    Ok(area > 0.0)
}
