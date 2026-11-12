//! A loft between two sections along guide curves through their corners.
//!
//! Each wall is the patch its four edges bound: a side of the first section,
//! the two guides from its ends, and the matching side of the last section,
//! filled by the bilinearly blended Coons patch ([`crate::coons_bilinear`]).
//! Where the two sides are straight, that patch is the surface ruled between
//! the two guides at equal parameters. A side of no length (the last section
//! closing on a point) makes its wall three-sided, closed on a pole. The walls
//! and the two flat ends are faces of their own, joined into the solid they
//! bound ([`crate::solid_from_sheets`]).
use crate::{coons_bilinear, face_from_surface, face_from_surface_trimmed, make_plane, solid_from_sheets, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};

/// The prefix of every refusal of this operation.
pub const LOFT_ON_GUIDES_REFUSAL: &str = "loftOnGuides";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{LOFT_ON_GUIDES_REFUSAL}: {message}"))
}

fn ends(curve: &NurbsCurve) -> Result<[Vec3; 2], KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Collect, "loftOnGuides.domain")?;
    Ok([curve.evaluate(t0).or_refuse(KernelStage::Collect, "loftOnGuides.end")?, curve.evaluate(t1).or_refuse(KernelStage::Collect, "loftOnGuides.end")?])
}

fn poles(curves: &[NurbsCurve]) -> Vec<Vec3> {
    curves.iter().flat_map(|curve| &curve.control_points).map(|pole| Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)).collect()
}

/// The flat face inside the closed loop `section`, which must lie in a plane.
fn flat_end(section: &[NurbsCurve], size: f64, which: &str) -> Result<BrepSolid, KernelRefusal> {
    let points = poles(section);
    let origin = points[0];
    let far = |off: Option<Vec3>| {
        points.iter().map(|point| point.sub(origin)).map(|arm| off.map_or(arm, |axis| arm.sub(axis.scale(arm.dot(axis))))).fold(Vec3::new(0.0, 0.0, 0.0), |best, arm| if arm.length() > best.length() { arm } else { best })
    };
    let flat = || input("section_plane", format!("the {which} section does not lie in a plane"));
    let e1 = far(None).normalized().map_err(|_| flat())?;
    let across = far(Some(e1));
    if across.length() <= 1e-9 * size {
        return Err(flat());
    }
    let e2 = across.normalized().map_err(|_| flat())?;
    let normal = e1.cross(e2);
    if points.iter().any(|point| point.sub(origin).dot(normal).abs() > 1e-9 * size) {
        return Err(flat());
    }
    let reach = |axis: Vec3| points.iter().map(|point| point.sub(origin).dot(axis)).fold((0.0f64, 0.0f64), |(low, high), at| (low.min(at), high.max(at)));
    let ((low1, high1), (low2, high2)) = (reach(e1), reach(e2));
    let corner = origin.add(e1.scale(low1 - 0.5 * size)).add(e2.scale(low2 - 0.5 * size));
    let plane = make_plane(corner, e1, e2, high1 - low1 + size, high2 - low2 + size).or_refuse(KernelStage::Collect, "loftOnGuides.plane")?;
    face_from_surface_trimmed(&plane, &[section.to_vec()])
}

/// The wall standing on the straight side `side` between two guides that meet
/// at one point: the surface ruled between the guides at equal parameters,
/// closed on a pole where they meet. (The Coons patch of a straight side, two
/// guides and a point is that ruled surface.)
fn pointed_wall(side: &NurbsCurve, rail: &NurbsCurve, next: &NurbsCurve, k: usize, tolerance: f64) -> Result<crate::NurbsSurface, KernelRefusal> {
    let [from, to] = ends(side)?;
    let straight = poles(std::slice::from_ref(side)).iter().all(|point| point.sub(from).cross(to.sub(from)).length() <= tolerance * to.sub(from).length());
    if !straight {
        return Err(KernelRefusal::unsupported(KernelStage::Fragment, "pointed_wall_side", format!("{LOFT_ON_GUIDES_REFUSAL}: wall {k} closes on a point from a side that is not straight")));
    }
    if rail.degree != next.degree || rail.knots != next.knots {
        return Err(KernelRefusal::unsupported(KernelStage::Fragment, "pointed_wall_guides", format!("{LOFT_ON_GUIDES_REFUSAL}: wall {k} closes on a point between guides of different degree or knots")));
    }
    let net = vec![rail.control_points.clone(), next.control_points.clone()];
    crate::NurbsSurface::new(1, rail.degree, vec![0.0, 0.0, 1.0, 1.0], rail.knots.clone(), net).or_refuse(KernelStage::Fragment, "loftOnGuides.pointed_wall")
}

/// The solid lofted from the closed section `first` to the closed section
/// `last` along `guides`.
///
/// The two sections have the same number of curves, each a closed loop run the
/// same way round, curve `k` of the first matched with curve `k` of the last;
/// each lies in a plane. `guides[k]` runs from the start of `first[k]` to the
/// start of `last[k]` (given in either direction). A curve of `last` may have
/// no length: where every one of them is the same point the loft closes on
/// that point and has no last end.
///
/// Refuses sections of different counts, guides that do not join the matched
/// corners, sections off a plane, and walls the Coons patch refuses.
pub fn loft_profile_brep_on_guides(first: &[NurbsCurve], last: &[NurbsCurve], guides: &[NurbsCurve]) -> Result<BrepSolid, KernelRefusal> {
    let count = first.len();
    if count < 2 || last.len() != count || guides.len() != count {
        return Err(input("counts", format!("the sections and the guides must be equally many, at least 2 (got {count}, {} and {})", last.len(), guides.len())));
    }
    let all = poles(&[first, last, guides].concat());
    let size = all.iter().map(|point| point.sub(all[0]).length()).fold(0.0, f64::max);
    let tolerance = 1e-9 * size;
    let near = |a: Vec3, b: Vec3| a.sub(b).length() <= tolerance;
    // each guide turned to run from the first section to the last
    let mut rails = Vec::with_capacity(count);
    for k in 0..count {
        let ([foot, _], [head, _]) = (ends(&first[k])?, ends(&last[k])?);
        let [from, to] = ends(&guides[k])?;
        let rail = if near(from, foot) && near(to, head) {
            guides[k].clone()
        } else if near(to, foot) && near(from, head) {
            guides[k].reversed().or_refuse(KernelStage::Collect, "loftOnGuides.reverse")?
        } else {
            return Err(input("guide_ends", format!("guide {k} does not join the start of curve {k} of the first section to that of the last")));
        };
        rails.push(rail);
    }
    let mut sheets = Vec::with_capacity(count + 2);
    for k in 0..count {
        let next = (k + 1) % count;
        let far_side = last[k].reversed().or_refuse(KernelStage::Collect, "loftOnGuides.reverse")?;
        let back = rails[k].reversed().or_refuse(KernelStage::Collect, "loftOnGuides.reverse")?;
        let [head, head_next] = ends(&last[k])?;
        let patch = if near(head, head_next) && poles(&last[k..=k]).iter().all(|point| near(*point, head)) {
            pointed_wall(&first[k], &rails[k], &rails[next], k, tolerance)?
        } else {
            coons_bilinear([first[k].clone(), rails[next].clone(), far_side, back])
                .map_err(|error| KernelRefusal::unsupported(KernelStage::Fragment, "wall", format!("{LOFT_ON_GUIDES_REFUSAL}: wall {k} has no Coons patch: {error:?}")))?
        };
        sheets.push(face_from_surface(&patch)?);
    }
    sheets.push(flat_end(first, size, "first")?);
    let apex = ends(&last[0])?[0];
    let pointed = poles(last).iter().all(|point| near(*point, apex));
    if !pointed {
        sheets.push(flat_end(last, size, "last")?);
    }
    // sewing reads each edge's place by projection, whose own error on a cubic is near 1e-8
    solid_from_sheets(&sheets, (1e-8 * size).max(1e-7))
}

