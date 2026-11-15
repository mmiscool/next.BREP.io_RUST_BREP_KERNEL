//! Which copy of a coincident face a boolean keeps, and where the rims of the
//! copy it drops end.
//!
//! Two operands carry COINCIDENT faces when the imprint reads a face pair as
//! one surface (`ImprintResultRecord::cosurface_pairs`): each is a copy of the
//! other within the weld band, in the same or the opposite sense. Selection
//! holds a fragment on the overlap On, and the result must bound its material
//! with exactly one copy.
//!
//! THE RULE. The result keeps the copy its material lies behind. Let `s` be
//! the offset of the second operand's copy from the first operand's along the
//! first operand's outward normal. A point between the copies is in the
//! result for a union of same-sense copies only on the outer side, so the
//! union keeps the outer copy (the second operand's when `s > 0`); an
//! intersection of same-sense copies keeps the inner one (the second's when
//! `s < 0`); a difference of opposite-sense copies, first − second, keeps the
//! copy nearer the first operand's material (the second's when `s < 0`, where
//! the second operand intrudes). A union or intersection of opposite-sense
//! copies and a difference of same-sense copies keep neither: the material
//! meets, or is removed, on both sides. Where every sampled `|s|` is within
//! the model tolerance the copies are one surface at the model's resolution
//! and the first operand's copy is kept, as before; where samples disagree in
//! sign beyond it the copies cross, and the first operand's copy is kept too.
//! No new tolerance: a pair is coincident by the imprint's own test, a sample
//! counts when it projects onto its partner within the assembler's weld, and
//! the level band is the model tolerance.
//!
//! THE RIMS. A kept face of either operand whose boundary edge is also the
//! boundary of a coincident copy ends on that copy. When the other operand's
//! copy crosses the face between its rim and its interior (a sub-weld strip
//! of the face lies beyond the other copy), the strip is on the other side of
//! that operand's boundary from the rest of the face, so the result does not
//! hold it: the rim is re-sewn onto the other copy, its pcurve moved across
//! the face until its points lie on the other copy's surface, and the
//! neighbouring coedges' ends follow. A strip the other copy does not cross
//! needs nothing. Escape hatches: `BREP_COINCIDENT_KEEPER=0` (operand order
//! decides, the first operand's copy kept), `BREP_RIM_RESEW=0`.

use crate::{KernelRefusal, KernelStage, NurbsSurface, OrRefuse, Vec4};
use super::*;

/// Where the second operand's copy sits against the first operand's, along
/// the first operand's outward normal.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum CopyOffset {
    Outward,
    Inward,
    /// Every sample within the model tolerance, or samples on both sides.
    Level,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct CoincidentPair {
    pub(super) same_sense: bool,
    pub(super) offset: CopyOffset,
}

/// Keyed by (first operand face id, second operand face id).
pub(super) type CoincidentPairs = HashMap<(u64, u64), CoincidentPair>;

fn outward_normal_at(surface: &NurbsSurface, same_sense: bool, u: f64, v: f64) -> Option<Vec3> {
    let normal = surface.normal(u, v).ok()?;
    Some(if same_sense { normal } else { normal.scale(-1.0) })
}

/// The partner copy against a fragment's point: `(offset of the partner's
/// surface along the fragment face's outward normal, own normal · partner
/// normal)` when the point projects onto the partner within the weld and
/// inside its trim.
fn partner_offset(
    fragment: &FaceFragmentRecord,
    partner: &FaceRecord,
    weld: f64,
) -> Result<Option<(f64, f64)>, KernelRefusal> {
    let Some(own) = outward_normal_at(&fragment.surface, fragment.same_sense, fragment.test_uv.x, fragment.test_uv.y)
    else {
        return Ok(None);
    };
    let Ok(projection) = project_point_to_surface(&partner.surface, fragment.test_point) else {
        return Ok(None);
    };
    if projection.distance > weld {
        return Ok(None);
    }
    let uv = Vec2 { x: projection.u, y: projection.v };
    if parameter_point_in_face(partner, uv, 1e-9).or_refuse(KernelStage::Select, "parameter_point_in_face")?
        != PolygonClass::Inside
    {
        return Ok(None);
    }
    let Some(theirs) = outward_normal_at(&partner.surface, partner.same_sense, projection.u, projection.v) else {
        return Ok(None);
    };
    Ok(Some((projection.point.sub(fragment.test_point).dot(own), own.dot(theirs))))
}

fn face_of(solid: &BrepSolid, face_id: u64) -> Option<&FaceRecord> {
    solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == face_id)
}

/// Every cosurface pair's sense and offset, measured at the fragments' own
/// test points on both copies.
pub(super) fn coincident_pairs(
    fragments_a: &[FaceFragmentRecord],
    fragments_b: &[FaceFragmentRecord],
    pristine_a: &BrepSolid,
    pristine_b: &BrepSolid,
    cosurface_pairs: &[(FaceKey, FaceKey)],
    tolerance: f64,
) -> Result<CoincidentPairs, KernelRefusal> {
    let mut pairs = CoincidentPairs::default();
    if std::env::var("BREP_COINCIDENT_KEEPER").as_deref() == Ok("0") {
        return Ok(pairs);
    }
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();
    let weld = assembler_weld(tolerance);
    for (first, second) in cosurface_pairs {
        let (a_key, b_key) = if first.operand == 0 { (first, second) } else { (second, first) };
        if a_key.operand != 0 || b_key.operand != 1 || pairs.contains_key(&(a_key.face_id, b_key.face_id)) {
            continue;
        }
        let (Some(face_a), Some(face_b)) = (face_of(pristine_a, a_key.face_id), face_of(pristine_b, b_key.face_id))
        else {
            continue;
        };
        // (offset of the second operand's copy along the first's outward
        // normal, normals agree)
        let mut samples: Vec<(f64, bool)> = Vec::new();
        for fragment in fragments_a.iter().filter(|fragment| fragment.source_face_id == a_key.face_id) {
            if let Some((offset, dot)) = partner_offset(fragment, face_b, weld)? {
                samples.push((offset, dot > 0.0));
            }
        }
        for fragment in fragments_b.iter().filter(|fragment| fragment.source_face_id == b_key.face_id) {
            if let Some((offset, dot)) = partner_offset(fragment, face_a, weld)? {
                // The first operand's copy sits `offset` along the second's
                // normal; the second's copy then sits at −offset·sign(n_a · n_b)
                // along the first's normal.
                samples.push((-offset * dot.signum(), dot > 0.0));
            }
        }
        if samples.is_empty() {
            continue;
        }
        let outward = samples.iter().any(|(offset, _)| *offset > tolerance);
        let inward = samples.iter().any(|(offset, _)| *offset < -tolerance);
        let offset = match (outward, inward) {
            (true, false) => CopyOffset::Outward,
            (false, true) => CopyOffset::Inward,
            _ => CopyOffset::Level,
        };
        let agreeing = samples.iter().filter(|(_, same)| *same).count();
        let pair = CoincidentPair { same_sense: agreeing * 2 >= samples.len(), offset };
        if debug {
            let (low, high) = samples
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), (offset, _)| (low.min(*offset), high.max(*offset)));
            eprintln!(
                "coincident pair {}x{}: same_sense={} offset={:?} ({} samples, second's copy {low:.3e} .. {high:.3e} along the first's normal)",
                a_key.face_id,
                b_key.face_id,
                pair.same_sense,
                pair.offset,
                samples.len()
            );
        }
        pairs.insert((a_key.face_id, b_key.face_id), pair);
    }
    Ok(pairs)
}

/// The operand whose copy `operation` keeps (see the module's rule), or `None`
/// when it keeps neither.
pub(super) fn keeper(pair: CoincidentPair, operation: BooleanOperation) -> Option<u8> {
    let second_when = |offset: CopyOffset| Some(if pair.offset == offset { 1 } else { 0 });
    match (pair.same_sense, operation) {
        (true, BooleanOperation::Union) => second_when(CopyOffset::Outward),
        (true, BooleanOperation::Intersect) => second_when(CopyOffset::Inward),
        (false, BooleanOperation::Subtract) => second_when(CopyOffset::Inward),
        _ => None,
    }
}

/// The partner face whose trim holds a fragment's point within the weld: the
/// copy the fragment is On against.
pub(super) fn coincident_partner(
    fragment: &FaceFragmentRecord,
    partners: &HashMap<u64, Vec<&FaceRecord>>,
    tolerance: f64,
) -> Result<Option<u64>, KernelRefusal> {
    let Some(faces) = partners.get(&fragment.source_face_id) else {
        return Ok(None);
    };
    let weld = assembler_weld(tolerance);
    for partner in faces {
        if partner_offset(fragment, partner, weld)?.is_some() {
            return Ok(Some(partner.id));
        }
    }
    Ok(None)
}

/// Move a pcurve across its face until its points lie on `target`, and the
/// largest distance of its stations from `target` before and after. A control
/// point whose station sits where the face's chart is degenerate (a pole)
/// stays where it is: a rim running into a pole ends at the pole's vertex
/// whichever copy it follows. A move is held to the face's domain.
/// `None` when the face runs along the target (a move beyond the weld), a
/// station leaves the domain, or the move brings no station closer.
fn pcurve_onto(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    target: &NurbsSurface,
    tolerance: f64,
) -> Result<Option<(NurbsCurve, f64, f64)>, String> {
    let weld = assembler_weld(tolerance);
    let degree = pcurve.degree;
    if degree == 0 {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let greville: Vec<f64> = (0..pcurve.control_points.len())
        .map(|index| pcurve.knots[index + 1..=index + degree].iter().sum::<f64>() / degree as f64)
        .collect();
    let inside = |uv: Vec3| uv.x >= u0 && uv.x <= u1 && uv.y >= v0 && uv.y <= v1;
    // (derivatives, target normal, gap) at a station inside the domain.
    let off = |curve: &NurbsCurve, parameter: f64| -> Result<Option<(Vec3, Vec3, Vec3, f64)>, String> {
        let uv = curve.evaluate(parameter)?;
        if !inside(uv) {
            return Ok(None);
        }
        let (point, along_u, along_v) = surface.deriv1(uv.x, uv.y)?;
        let projection = project_point_to_surface(target, point)?;
        let normal = target.normal(projection.u, projection.v)?;
        Ok(Some((along_u, along_v, normal, point.sub(projection.point).dot(normal))))
    };
    let [t0, t1] = pcurve.domain()?;
    let stations = (2 * pcurve.control_points.len()).max(16);
    let gaps = |curve: &NurbsCurve| -> Result<Option<Vec<f64>>, String> {
        let mut gaps = Vec::with_capacity(stations + 1);
        for station in 0..=stations {
            match off(curve, t0 + (t1 - t0) * station as f64 / stations as f64)? {
                Some((.., gap)) => gaps.push(gap.abs()),
                None => return Ok(None),
            }
        }
        Ok(Some(gaps))
    };
    let Some(before) = gaps(pcurve)? else {
        return Ok(None);
    };
    let mut curve = pcurve.clone();
    for _ in 0..4 {
        let mut controls = curve.control_points.clone();
        for (index, &parameter) in greville.iter().enumerate() {
            let Some((along_u, along_v, normal, gap)) = off(&curve, parameter)? else {
                return Ok(None);
            };
            let duv = curve.derivatives(parameter, 1)?[1];
            let tangent = along_u.scale(duv.x).add(along_v.scale(duv.y));
            let Ok(across) = along_u.cross(along_v).cross(tangent).normalized() else {
                continue;
            };
            let rate = across.dot(normal);
            if rate == 0.0 || (gap / rate).abs() > weld {
                return Ok(None);
            }
            let step = across.scale(-gap / rate);
            let (uu, uv, vv) = (along_u.dot(along_u), along_u.dot(along_v), along_v.dot(along_v));
            let determinant = uu * vv - uv * uv;
            if determinant <= 0.0 {
                continue;
            }
            let (ru, rv) = (along_u.dot(step), along_v.dot(step));
            let du = (ru * vv - rv * uv) / determinant;
            let dv = (rv * uu - ru * uv) / determinant;
            let control = controls[index];
            let Ok(at) = control.point() else {
                continue;
            };
            // A move off the domain stays on its edge: round-off in the step
            // along a rim that lies on the domain's boundary.
            let (u, v) = ((at.x + du).clamp(u0, u1), (at.y + dv).clamp(v0, v1));
            controls[index] = Vec4 { x: u * control.w, y: v * control.w, ..control };
        }
        curve = NurbsCurve::new(curve.degree, curve.knots.clone(), controls)?;
    }
    let Some(after) = gaps(&curve)? else {
        return Ok(None);
    };
    // Accepted only when no station moves further from the target (beyond the
    // model tolerance) and the stations come closer in sum.
    let no_worse = before.iter().zip(&after).all(|(before, after)| *after <= before.max(tolerance));
    if !no_worse || after.iter().sum::<f64>() >= before.iter().sum::<f64>() {
        return Ok(None);
    }
    let worst = |gaps: &[f64]| gaps.iter().copied().fold(0.0f64, f64::max);
    Ok(Some((curve, worst(&before), worst(&after))))
}

/// Set a clamped pcurve's first or last point.
fn with_end(pcurve: &NurbsCurve, at_start: bool, uv: Vec3) -> Result<NurbsCurve, String> {
    let mut controls = pcurve.control_points.clone();
    let index = if at_start { 0 } else { controls.len() - 1 };
    let control = controls[index];
    controls[index] = Vec4 { x: uv.x * control.w, y: uv.y * control.w, ..control };
    NurbsCurve::new(pcurve.degree, pcurve.knots.clone(), controls)
}

/// Adjacent pcurves in the same chart must share the repaired endpoint,
/// including removal of their pre-existing fitting gap. A distant UV image
/// of the same 3D junction (a seam or pole) instead follows the displacement
/// in its own chart representation.
fn followed_rim_end(surface: &NurbsSurface, at: Vec3, old: Vec3, new: Vec3, weld: f64) -> Vec3 {
    let band = crate::classification::surface_uv_band(surface, old.x, old.y, weld);
    let delta = Vec2 { x:at.x-old.x, y:at.y-old.y };
    if delta.length() <= band { new } else { at.add(new.sub(old)) }
}

/// Re-sew every kept fragment's rim that a coincident copy of the other
/// operand crosses (see the module's rims). Returns the rims moved.
pub(super) fn resew_coincident_rims(
    selected: &mut [FaceFragmentRecord],
    split: [&BrepSolid; 2],
    pristine: [&BrepSolid; 2],
    cosurface_pairs: &[(FaceKey, FaceKey)],
    operation: BooleanOperation,
    tolerance: f64,
) -> Result<usize, KernelRefusal> {
    if cosurface_pairs.is_empty() || std::env::var("BREP_RIM_RESEW").as_deref() == Ok("0") {
        return Ok(0);
    }
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();
    let weld = assembler_weld(tolerance);
    // (operand, face id) -> the other operand's coincident copies.
    let mut partners: HashMap<(u8, u64), Vec<&FaceRecord>> = HashMap::default();
    for (first, second) in cosurface_pairs {
        for (own, other) in [(first, second), (second, first)] {
            if own.operand > 1 || other.operand > 1 || own.operand == other.operand {
                continue;
            }
            if let Some(face) = face_of(pristine[other.operand as usize], other.face_id) {
                let entry = partners.entry((own.operand, own.face_id)).or_default();
                if !entry.iter().any(|known| known.id == face.id) {
                    entry.push(face);
                }
            }
        }
    }
    // (operand, edge id) -> the split operand's faces using it.
    let mut edge_faces: HashMap<(u8, u64), Vec<u64>> = HashMap::default();
    for (operand, solid) in split.iter().enumerate() {
        for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
            for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                let entry = edge_faces.entry((operand as u8, coedge.edge_id)).or_default();
                if !entry.contains(&face.id) {
                    entry.push(face.id);
                }
            }
        }
    }
    let mut moved = 0usize;
    for fragment in selected.iter_mut() {
        let operand = fragment.operand;
        if operand > 1 || partners.contains_key(&(operand, fragment.source_face_id)) {
            // A copy itself: its fate is the keeper's, not a rim's.
            continue;
        }
        // Where the rest of this kept fragment lies against the other operand.
        let inside_other = match (operand, operation) {
            (0, BooleanOperation::Intersect) => true,
            (0, _) => false,
            (_, BooleanOperation::Union) => false,
            _ => true,
        };
        for loop_record in &mut fragment.loops {
            let count = loop_record.coedges.len();
            for index in 0..count {
                let FragmentEdgeSource::Boundary { operand: edge_operand, edge_id } = loop_record.coedges[index].source else {
                    continue;
                };
                if edge_operand != operand {
                    continue;
                }
                let Some(faces) = edge_faces.get(&(operand, edge_id)) else {
                    continue;
                };
                let copies: Vec<&FaceRecord> = faces
                    .iter()
                    .filter(|&&face_id| face_id != fragment.source_face_id)
                    .filter_map(|&face_id| partners.get(&(operand, face_id)))
                    .flatten()
                    .copied()
                    .collect();
                for copy in copies {
                    let pcurve = &loop_record.coedges[index].pcurve;
                    let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Select, "domain")?;
                    let uv = pcurve.evaluate((t0 + t1) * 0.5).or_refuse(KernelStage::Select, "evaluate")?;
                    let Ok(point) = fragment.surface.evaluate(uv.x, uv.y) else {
                        continue;
                    };
                    let Ok(projection) = project_point_to_surface(&copy.surface, point) else {
                        continue;
                    };
                    if projection.distance > weld {
                        continue;
                    }
                    let copy_uv = Vec2 { x: projection.u, y: projection.v };
                    if parameter_point_in_face(copy, copy_uv, 1e-9).or_refuse(KernelStage::Select, "parameter_point_in_face")?
                        == PolygonClass::Outside
                    {
                        continue;
                    }
                    let Some(normal) = outward_normal_at(&copy.surface, copy.same_sense, projection.u, projection.v) else {
                        continue;
                    };
                    let gap = point.sub(projection.point).dot(normal);
                    // The strip beyond the copy is inside its operand when the
                    // rim sits behind the copy's outward normal.
                    if gap.abs() <= tolerance || (gap < 0.0) == inside_other {
                        continue;
                    }
                    let moved_onto = pcurve_onto(&fragment.surface, pcurve, &copy.surface, tolerance)
                        .or_refuse(KernelStage::Select, "pcurve_onto")?;
                    let Some((onto, before, after)) = moved_onto else {
                        if debug {
                            eprintln!(
                                "resew: operand {operand} face {} rim edge {edge_id} stands {gap:.3e} beyond copy {} and could not be moved onto it",
                                fragment.source_face_id, copy.id
                            );
                        }
                        continue;
                    };
                    let [s0, s1] = onto.domain().or_refuse(KernelStage::Select, "domain")?;
                    let (start, end) = (
                        onto.evaluate(s0).or_refuse(KernelStage::Select, "evaluate")?,
                        onto.evaluate(s1).or_refuse(KernelStage::Select, "evaluate")?,
                    );
                    let (old_start, old_end) = (
                        pcurve.evaluate(t0).or_refuse(KernelStage::Select, "evaluate")?,
                        pcurve.evaluate(t1).or_refuse(KernelStage::Select, "evaluate")?,
                    );
                    loop_record.coedges[index].pcurve = onto;
                    if count > 1 {
                        let previous = (index + count - 1) % count;
                        let next = (index + 1) % count;
                        for (neighbour, at_start, old, new) in [(previous, false, old_start, start), (next, true, old_end, end)] {
                            let neighbour_pcurve = &loop_record.coedges[neighbour].pcurve;
                            let [n0, n1] = neighbour_pcurve.domain().or_refuse(KernelStage::Select, "domain")?;
                            let at = neighbour_pcurve
                                .evaluate(if at_start { n0 } else { n1 })
                                .or_refuse(KernelStage::Select, "evaluate")?;
                            // The neighbour's end is the rim's when the two
                            // meet in 3D within the weld. Nearby chart points
                            // become one endpoint; distant seam/pole images
                            // follow the displacement in their own chart.
                            let meets = match (fragment.surface.evaluate(at.x, at.y), fragment.surface.evaluate(old.x, old.y)) {
                                (Ok(there), Ok(here)) => there.sub(here).length() <= weld,
                                _ => false,
                            };
                            if meets {
                                loop_record.coedges[neighbour].pcurve =
                                    with_end(neighbour_pcurve, at_start, followed_rim_end(&fragment.surface, at, old, new, weld)).or_refuse(KernelStage::Select, "with_end")?;
                            }
                        }
                    }
                    moved += 1;
                    if debug {
                        eprintln!(
                            "resew: operand {operand} face {} rim edge {edge_id} moved {gap:.3e} onto copy {} (stations off it {before:.3e} -> {after:.3e})",
                            fragment.source_face_id, copy.id
                        );
                    }
                    break;
                }
            }
        }
    }
    Ok(moved)
}

