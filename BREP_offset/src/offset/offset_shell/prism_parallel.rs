//! The SHRUNK parallel body of a right prism whose side is one convex ruled
//! face between two planar caps: the profile's inward offset, trimmed where it
//! crosses itself, extruded between the caps moved in by the distance.
//!
//! Past the profile's tightest radius of curvature the side's offset folds
//! (a swallowtail: two cusps and a loop on each side of the fold) and the
//! pipeline refuses the band it cannot assemble ("folds in a BAND across its
//! trim"). The erosion is still well defined: every point at least `d` from
//! the outside. Its section is the part of the inward offset curve lying at
//! least `d` from the whole profile, and that part ends where the curve
//! crosses itself, in a sharp edge along the prism's direction.
//!
//! Cause (brep-bench offset/elliptic-cylinder-shrunk-past-its-curvature): the
//! elliptic cylinder x²/25 + y²/9 ≤ 1, 0 ≤ z ≤ 10, shrunk by 2.5 past its
//! tightest radius b²/a = 1.8, was refused from d = 1.8 on. Its section's area
//! is 3.06267694641 by quadrature of the trimmed offset curve (a distance grid
//! agrees), its volume that times 5.
//!
//! It runs only where the pipeline and every other lane refused, and it
//! declines (the pipeline's refusal stands) on anything but the prism above.
//! An erosion with nothing left is refused by name.
use super::*;
use crate::{KernelRefusal, KernelStage, NurbsCurve, OrRefuse};

/// `BREP_OFFSET_PRISM_PARALLEL` (default on; `=0` restores the refusal).
pub(super) fn prism_parallel_on() -> bool {
    switch!("BREP_OFFSET_PRISM_PARALLEL", true)
}

/// How many stations the kept-parameter census reads around the profile.
const STATIONS: usize = 2048;
/// Where a piece's fit stops refining and the lane declines.
const MAX_FIT_STATIONS: usize = 8192;

/// The shrunk parallel body of `source` by `distance` (< 0), or `None` when the
/// lane does not apply. `Err` only for an erosion with nothing left.
pub(super) fn prism_parallel(source: &BrepSolid, distance: f64) -> Result<Option<OffsetShellResultRecord>, KernelRefusal> {
    if !prism_parallel_on() || !(distance < 0.0) || !distance.is_finite() {
        return Ok(None);
    }
    match prism_parallel_impl(source, -distance) {
        Ok(result) => Ok(result),
        Err(error) if error.message.starts_with("offset_solid: the shrink") => Err(error),
        Err(error) => {
            os_debug!("prism parallel lane declined: {}", error.message);
            Ok(None)
        }
    }
}


pub(super) struct Prism<'a> {
    pub(super) profile: NurbsCurve,
    /// The prism's direction, from the bottom cap to the top.
    pub(super) axis: Vec3,
    pub(super) height: f64,
    pub(super) side: &'a FaceRecord,
    pub(super) bottom: &'a FaceRecord,
    pub(super) top: &'a FaceRecord,
}

/// The right prism `source` is (two caps square to one ruled side along one
/// translation, the profile on the bottom cap), or why not.
pub(super) fn recognize_prism(source: &BrepSolid, tolerance: f64) -> Result<Prism<'_>, KernelRefusal> {
    recognize(source, tolerance)
}

fn decline(reason: impl Into<String>) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Refine, "prism_parallel_declined", reason.into())
}

fn recognize(source: &BrepSolid, tolerance: f64) -> Result<Prism<'_>, KernelRefusal> {
    if source.shells.len() != 1 || source.shells[0].faces.len() != 3 {
        return Err(decline("not one shell of three faces"));
    }
    let faces = &source.shells[0].faces;
    let mut caps = Vec::new();
    let mut sides = Vec::new();
    for face in faces {
        if face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
            caps.push(face);
        } else {
            sides.push(face);
        }
    }
    if caps.len() != 2 || sides.len() != 1 {
        return Err(decline("not two planar caps and one curved side"));
    }
    let side = sides[0];
    let surface = &side.surface;
    // Ruled along v: two control rows a constant translation apart.
    if surface.degree_v != 1 || surface.control_points.iter().any(|row| row.len() != 2) {
        return Err(decline("the side is not ruled along v"));
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let translation = surface.evaluate(u0, v1).or_refuse(KernelStage::Refine, "evaluate")?
        .sub(surface.evaluate(u0, v0).or_refuse(KernelStage::Refine, "evaluate")?);
    let height = translation.length();
    if height <= tolerance {
        return Err(decline("the side has no height"));
    }
    for k in 0..=16 {
        let u = u0 + (u1 - u0) * k as f64 / 16.0;
        let run = surface.evaluate(u, v1).or_refuse(KernelStage::Refine, "evaluate")?
            .sub(surface.evaluate(u, v0).or_refuse(KernelStage::Refine, "evaluate")?);
        if run.sub(translation).length() > tolerance {
            return Err(decline("the side's rulings are not one translation"));
        }
    }
    let axis = translation.scale(1.0 / height);
    let profile = surface.iso_curve_v(v0).or_refuse(KernelStage::Refine, "iso_curve_v")?;
    let [s0, s1] = profile.domain().or_refuse(KernelStage::Refine, "domain")?;
    let (start, end) = (
        profile.evaluate(s0).or_refuse(KernelStage::Refine, "evaluate")?,
        profile.evaluate(s1).or_refuse(KernelStage::Refine, "evaluate")?,
    );
    if start.sub(end).length() > tolerance {
        return Err(decline("the profile is not closed"));
    }
    // The caps: one on the profile's plane facing −axis, one `height` along it facing +axis.
    let outward = |face: &FaceRecord| -> Result<(Vec3, Vec3), KernelRefusal> {
        let [a0, a1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [b0, b1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let (u, v) = (0.5 * (a0 + a1), 0.5 * (b0 + b1));
        let normal = face.surface.normal(u, v).or_refuse(KernelStage::Refine, "normal")?;
        let normal = if face.same_sense { normal } else { normal.scale(-1.0) };
        Ok((face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?, normal))
    };
    let mut bottom = None;
    let mut top = None;
    for cap in caps {
        let (point, normal) = outward(cap)?;
        let level = point.sub(start).dot(axis);
        if normal.dot(axis) < -1.0 + 1e-9 && level.abs() <= tolerance {
            bottom = Some(cap);
        } else if normal.dot(axis) > 1.0 - 1e-9 && (level - height).abs() <= tolerance {
            top = Some(cap);
        }
    }
    let (Some(bottom), Some(top)) = (bottom, top) else {
        return Err(decline("the caps are not the side's two ends, square to it"));
    };
    Ok(Prism { profile, axis, height, side, bottom, top })
}

fn prism_parallel_impl(source: &BrepSolid, depth: f64) -> Result<Option<OffsetShellResultRecord>, KernelRefusal> {
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-7 * scale;
    let prism = recognize(source, tolerance)?;
    if prism.height <= 2.0 * depth + tolerance {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_solid_shrinks_to_nothing", format!(
            "offset_solid: the shrink by {depth:.6} leaves nothing: the prism is {:.6} high, so its moved caps meet or cross",
            prism.height
        )));
    }
    let profile = &prism.profile;
    let [s0, s1] = profile.domain().or_refuse(KernelStage::Refine, "domain")?;
    let period = s1 - s0;
    let wrap = |s: f64| s0 + (s - s0).rem_euclid(period);
    // The in-plane unit tangent and the INWARD normal at `s`; convex: the turn
    // (tangent × tangent′)·axis keeps one sign all the way round.
    let mut turn_sign = 0.0f64;
    let frame = |s: f64| -> Result<(Vec3, Vec3, f64), KernelRefusal> {
        let d = profile.derivatives(wrap(s), 2).or_refuse(KernelStage::Refine, "derivatives")?;
        let speed = d[1].length();
        if speed <= 1e-300 {
            return Err(decline("the profile is stationary"));
        }
        let tangent = d[1].scale(1.0 / speed);
        // The CURVATURE (a length's reciprocal), so the reading does not
        // depend on how the profile is parametrized.
        let turn = d[1].cross(d[2]).dot(prism.axis) / (speed * speed * speed);
        Ok((d[0], tangent, turn))
    };
    // The profile's own size: what its curvature and its section are read against.
    let mut diameter = 0.0f64;
    {
        let samples = (0..64)
            .map(|k| profile.evaluate(s0 + period * k as f64 / 64.0).or_refuse(KernelStage::Refine, "evaluate"))
            .collect::<Result<Vec<_>, _>>()?;
        for (i, a) in samples.iter().enumerate() {
            for b in &samples[i + 1..] {
                diameter = diameter.max(a.sub(*b).length());
            }
        }
    }
    if diameter <= tolerance {
        return Err(decline("the profile has no size"));
    }
    // The section's own tolerance: a prism's length must not loosen its fit.
    let section_tolerance = 1e-7 * diameter.max(1.0);
    for k in 0..STATIONS {
        let (_, _, turn) = frame(s0 + period * k as f64 / STATIONS as f64)?;
        if turn.abs() <= 1e-9 / diameter {
            continue;
        }
        if turn_sign == 0.0 {
            turn_sign = turn.signum();
        } else if turn.signum() != turn_sign {
            return Err(decline("the profile is not convex"));
        }
    }
    if turn_sign == 0.0 {
        return Err(decline("the profile does not turn"));
    }
    // Counter-clockwise about the axis turns left, so inward is axis × tangent.
    let offset_point = |s: f64| -> Result<Vec3, KernelRefusal> {
        let (point, tangent, _) = frame(s)?;
        let inward = prism.axis.cross(tangent).scale(turn_sign);
        Ok(point.add(inward.scale(depth)))
    };
    // The kept stations: the offset point at least `depth` from the whole profile.
    let distance_to_profile = |point: Vec3| -> Result<f64, KernelRefusal> {
        Ok(crate::project_point_to_curve(profile, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?.distance)
    };
    let mut kept = Vec::with_capacity(STATIONS);
    for k in 0..STATIONS {
        let point = offset_point(s0 + period * k as f64 / STATIONS as f64)?;
        kept.push(distance_to_profile(point)? >= depth - 1e-6 * diameter.max(1.0));
    }
    if kept.iter().all(|keep| !keep) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_solid_shrinks_to_nothing", format!(
            "offset_solid: the shrink by {depth:.6} leaves nothing: no point of the profile's inward offset lies that far from the profile"
        )));
    }
    let step = period / STATIONS as f64;
    // The kept runs, as station intervals [first, last] round the loop.
    let mut runs: Vec<(usize, usize)> = Vec::new();
    if kept.iter().all(|keep| *keep) {
        runs.push((0, STATIONS));
    } else {
        let begin = (0..STATIONS).find(|&k| !kept[k]).unwrap();
        let mut k = 0;
        while k < STATIONS {
            let index = (begin + k) % STATIONS;
            if kept[index] {
                let first = begin + k;
                let mut last = first;
                while last + 1 < begin + STATIONS && kept[(last + 1) % STATIONS] {
                    last += 1;
                }
                runs.push((first, last));
                k = last - begin + 1;
            } else {
                k += 1;
            }
        }
    }
    // An erosion with no INTERIOR is empty too: past the profile's inradius
    // the kept offset points crowd onto a segment or a point and enclose no
    // area (the ellipse with b = 3 shrunk by 3).
    {
        let mut ring = Vec::new();
        for k in 0..STATIONS {
            if kept[k] {
                ring.push(offset_point(s0 + period * k as f64 / STATIONS as f64)?);
            }
        }
        let reference_axis = prism.axis;
        let enclosed = (0..ring.len())
            .map(|i| ring[i].cross(ring[(i + 1) % ring.len()]).dot(reference_axis))
            .sum::<f64>()
            .abs()
            * 0.5;
        if enclosed <= 1e-9 * diameter * diameter {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_solid_shrinks_to_nothing", format!(
                "offset_solid: the shrink by {depth:.6} leaves nothing: the points of the profile's inward offset that far from it enclose no area ({enclosed:.3e})"
            )));
        }
    }
    // In-plane components, for the crossing solve.
    let reference = if prism.axis.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let e1 = reference.sub(prism.axis.scale(reference.dot(prism.axis))).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let e2 = prism.axis.cross(e1);
    let flat = |p: Vec3| (p.dot(e1), p.dot(e2));
    // Each gap between run i's end and run i+1's start closes at the curve's
    // self-crossing: offset(a) = offset(b), solved by Newton from the runs' ends.
    let count = runs.len();
    let mut piece_ends = vec![(0.0f64, 0.0f64); count];
    let full = count == 1 && runs[0] == (0, STATIONS);
    if !full {
        for index in 0..count {
            let (_, last) = runs[index];
            let (next_first, _) = runs[(index + 1) % count];
            let mut a = s0 + step * last as f64;
            let mut b = s0 + step * next_first as f64;
            if b < a {
                b += period;
            }
            let mut solved = false;
            for _ in 0..80 {
                let (pa, pb) = (flat(offset_point(a)?), flat(offset_point(b)?));
                let gap = (pa.0 - pb.0, pa.1 - pb.1);
                if gap.0.hypot(gap.1) <= 1e-12 * diameter.max(1.0) {
                    solved = true;
                    break;
                }
                let h = 1e-7 * period;
                let (pa_h, pb_h) = (flat(offset_point(a + h)?), flat(offset_point(b + h)?));
                let da = ((pa_h.0 - pa.0) / h, (pa_h.1 - pa.1) / h);
                let db = ((pb_h.0 - pb.0) / h, (pb_h.1 - pb.1) / h);
                // [da, −db] (Δa, Δb) = −gap
                let det = da.0 * (-db.1) - (-db.0) * da.1;
                if det.abs() <= 1e-300 {
                    break;
                }
                let step_a = ((-gap.0) * (-db.1) - (-db.0) * (-gap.1)) / det;
                let step_b = (da.0 * (-gap.1) - (-gap.0) * da.1) / det;
                let limit = (0.25 * step * 8.0) / step_a.abs().max(step_b.abs()).max(0.25 * step * 8.0);
                a += step_a * limit;
                b += step_b * limit;
            }
            if !solved {
                return Err(decline(format!("the offset's crossing after kept run {index} was not solved")));
            }
            piece_ends[index].1 = a;
            piece_ends[(index + 1) % count].0 = b;
        }
    } else {
        piece_ends[0] = (s0, s1);
    }
    // Each kept piece split where the offset stands still (its speed is the
    // profile's times |1 − depth·κ|, zero where the radius of curvature equals
    // the depth: the onset of the fold, a point of the offset with no tangent
    // that no single smooth fit passes), then fitted, ends pinned, lifted.
    let speed_factor = |s: f64| -> Result<f64, KernelRefusal> {
        let d = profile.derivatives(wrap(s), 2).or_refuse(KernelStage::Refine, "derivatives")?;
        let speed = d[1].length();
        let curvature = d[1].cross(d[2]).dot(prism.axis).abs() / (speed * speed * speed);
        Ok((1.0 - depth * curvature).abs())
    };
    let mut spans = Vec::new();
    for &(start, end) in &piece_ends {
        let end = if end <= start { end + period } else { end };
        let mut cuts = vec![start];
        let samples = ((end - start) / step).ceil().max(2.0) as usize;
        let values = (0..=samples)
            .map(|k| speed_factor(start + (end - start) * k as f64 / samples as f64))
            .collect::<Result<Vec<_>, _>>()?;
        for k in 1..samples {
            if values[k] < 1e-3 && values[k] <= values[k - 1] && values[k] <= values[k + 1] {
                // Refine the minimum by golden section between its neighbours.
                let (mut lo, mut hi) = (start + (end - start) * (k - 1) as f64 / samples as f64, start + (end - start) * (k + 1) as f64 / samples as f64);
                let ratio = 0.5 * (5f64.sqrt() - 1.0);
                for _ in 0..80 {
                    let (c, d) = (hi - ratio * (hi - lo), lo + ratio * (hi - lo));
                    if speed_factor(c)? < speed_factor(d)? { hi = d } else { lo = c }
                }
                cuts.push(0.5 * (lo + hi));
            }
        }
        cuts.push(end);
        for pair in cuts.windows(2) {
            spans.push((pair[0], pair[1]));
        }
    }
    let lift = prism.axis.scale(depth);
    let mut curves = Vec::with_capacity(spans.len());
    for &(start, end) in &spans {
        let exact = |fraction: f64| -> Result<Vec3, KernelRefusal> { Ok(offset_point(start + (end - start) * fraction)?.add(lift)) };
        let (from, to) = (exact(0.0)?, exact(1.0)?);
        let mut stations = 16usize;
        let mut fitted = None;
        while stations <= MAX_FIT_STATIONS {
            let parameters = (0..=stations).map(|k| k as f64 / stations as f64).collect::<Vec<_>>();
            let mut points = parameters.iter().map(|&f| exact(f)).collect::<Result<Vec<_>, _>>()?;
            points[0] = from;
            points[stations] = to;
            let curve = crate::interpolate_curve(&points, 3, &parameters).or_refuse(KernelStage::Refine, "interpolate_curve")?;
            let mut worst = 0.0f64;
            for k in 0..stations {
                for quarter in [0.25, 0.5, 0.75] {
                    let f = (k as f64 + quarter) / stations as f64;
                    worst = worst.max(curve.evaluate(f).or_refuse(KernelStage::Refine, "evaluate")?.sub(exact(f)?).length());
                }
            }
            if worst <= section_tolerance {
                fitted = Some(curve);
                break;
            }
            stations *= 2;
        }
        let Some(curve) = fitted else {
            return Err(decline("a kept piece of the offset was not fitted within the tolerance"));
        };
        curves.push(curve);
    }
    let solid = crate::extrude_profile_brep(&curves, prism.axis, prism.height - 2.0 * depth)?;
    if !solid.validate().is_empty() {
        return Err(decline(format!("the extruded erosion does not validate: {:?}", solid.validate())));
    }
    let face_images = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| -> Result<OffsetShellFaceImageRecord, KernelRefusal> {
            let source_face_id = if face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
                let [a0, a1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
                let [b0, b1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
                let point = face.surface.evaluate(0.5 * (a0 + a1), 0.5 * (b0 + b1)).or_refuse(KernelStage::Refine, "evaluate")?;
                let level = point.sub(prism.profile.evaluate(s0).or_refuse(KernelStage::Refine, "evaluate")?).dot(prism.axis);
                if level < 0.5 * prism.height { prism.bottom.id } else { prism.top.id }
            } else {
                prism.side.id
            };
            Ok(OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id })
        })
        .collect::<Result<Vec<_>, _>>()?;
    os_debug!("prism parallel lane built the shrink by {depth:.6} from {} kept piece(s)", curves.len());
    Ok(Some(OffsetShellResultRecord { solid, face_images }))
}
