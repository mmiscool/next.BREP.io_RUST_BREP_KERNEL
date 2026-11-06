//! Reuse edge/carrier solves across seeding and section processing in one imprint.
use super::*;
use std::cell::RefCell;
use crate::CurveSurfaceIntersection;

#[derive(Default)]
struct Cache {
    results: HashMap<Vec<u64>, Vec<CurveSurfaceIntersection>>,
    requests: usize,
    solves: usize,
}
thread_local! {
    static CACHES: RefCell<Vec<Cache>> = const { RefCell::new(Vec::new()) };
}

pub(super) struct Scope;
impl Scope {
    pub(super) fn open() -> Self {
        CACHES.with(|stack| stack.borrow_mut().push(Cache::default()));
        Self
    }
}
impl Drop for Scope {
    fn drop(&mut self) {
        let cache = CACHES.with(|stack| stack.borrow_mut().pop().unwrap());
        if profile_enabled() {
            profile_line!("imprint.intersection_cache requests={} solves={} retained={}",
                cache.requests, cache.solves, cache.results.len());
        }
    }
}

// Include complete nets and tolerance, not face/edge IDs: IDs overlap between
// operands, and restricted carriers differ from their parent faces. Surface
// identity also separates objects with distinct lazy projection caches.
fn key(curve: &NurbsCurve, surface: &NurbsSurface, tolerance: f64) -> Vec<u64> {
    let _wall = crate::wall_profile::Scope::enter(crate::wall_profile::LOOKUP);
    fn floats(key: &mut Vec<u64>, values: &[f64]) {
        key.push(values.len() as u64);
        key.extend(values.iter().map(|v| v.to_bits()));
    }
    fn controls(key: &mut Vec<u64>, values: &[Vec4]) {
        key.push(values.len() as u64);
        for p in values { key.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits(), p.w.to_bits()]); }
    }
    let mut key = vec![tolerance.to_bits(), surface as *const NurbsSurface as usize as u64,
        surface.degree_u as u64, surface.degree_v as u64];
    floats(&mut key, &surface.knots_u);
    floats(&mut key, &surface.knots_v);
    key.push(surface.control_points.len() as u64);
    for row in &surface.control_points { controls(&mut key, row); }
    key.push(curve.degree as u64);
    floats(&mut key, &curve.knots);
    controls(&mut key, &curve.control_points);
    key
}

pub(super) fn intersect(
    curve: &NurbsCurve, surface: &NurbsSurface, tolerance: f64,
) -> Result<Vec<CurveSurfaceIntersection>, String> {
    if CACHES.with(|stack| stack.borrow().is_empty()) {
        return solve(curve, surface, tolerance);
    }
    let key = key(curve, surface, tolerance);
    let hit = CACHES.with(|stack| {
        let _wall = crate::wall_profile::Scope::enter(crate::wall_profile::LOOKUP);
        let mut stack = stack.borrow_mut();
        let cache = stack.last_mut().unwrap();
        cache.requests += 1;
        let hit = cache.results.get(&key).cloned();
        if hit.is_none() { cache.solves += 1; }
        hit
    });
    if let Some(result) = hit { return Ok(result); }
    let result = solve(curve, surface, tolerance)?;
    CACHES.with(|stack| {
        let _wall = crate::wall_profile::Scope::enter(crate::wall_profile::LOOKUP);
        stack.borrow_mut().last_mut().unwrap().results.insert(key, result.clone());
    });
    Ok(result)
}

/// Lane O row 15 (2026-10-28), default on since batch 7 (`BREP_IMPRINT_EXACT_PIERCE=0`
/// restores the general search; held back from batch 4 until row 26 made
/// `offset_subtract_fixture`'s joint decision immune to last-bit pierce noise): an edge that
/// is a straight segment against an analytic carrier takes the closed-form
/// roots of `exact_line_surface` as its Newton seeds (the point classifier's
/// exact ray lane, lane B row 1), instead of the general search's 32 sampled
/// starts. The Newton, gap filter, duplicate guard and tangency reading are
/// the general search's own; the lane declines (the general search answers)
/// for a curved edge, an unrecognised carrier, a grazing or parallel segment.
fn solve(curve: &NurbsCurve, surface: &NurbsSurface, tolerance: f64) -> Result<Vec<CurveSurfaceIntersection>, String> {
    let result = solve_pierces(curve, surface, tolerance);
    result
}

fn solve_pierces(curve: &NurbsCurve, surface: &NurbsSurface, tolerance: f64) -> Result<Vec<CurveSurfaceIntersection>, String> {
    if exact_pierce_enabled() {
        if let Some(seeds) = crate::exact_line_surface::exact_line_surface_seeds(curve, surface, tolerance) {
            return crate::curve_surface_intersection::intersect_curve_surface_from_seeds(curve, surface, tolerance, seeds);
        }
    }
    if plane_pierce_enabled() {
        if let Some(seeds) = crate::exact_line_surface::exact_curve_plane_seeds(curve, surface, tolerance) {
            return crate::curve_surface_intersection::intersect_curve_surface_from_seeds(curve, surface, tolerance, seeds);
        }
    }
    if quadric_pierce_enabled() {
        if let Some(seeds) = crate::exact_line_surface::exact_curve_quadric_seeds(curve, surface, tolerance) {
            return crate::curve_surface_intersection::intersect_curve_surface_from_seeds(curve, surface, tolerance, seeds);
        }
    }
    crate::intersect_curve_surface(curve, surface, tolerance)
}

/// Lane O row 27 (2026-10-30), default on since batch 8 (`BREP_IMPRINT_QUADRIC_PIERCE=0`
/// restores the general search): an edge of degree two, a fillet's rational arc included, against a
/// sphere, cylinder or cone carrier is seeded at the closed-form roots of the
/// quadric's equation along it (`exact_curve_quadric_seeds`); same Newton and
/// filters as rows 15 and 16. An arc that misses the carrier seeds nothing.
pub(super) fn quadric_pierce_enabled() -> bool {
    switch!("BREP_IMPRINT_QUADRIC_PIERCE", true)
}


/// Lane O row 16 (2026-10-28), default on since batch 7 (`BREP_IMPRINT_PLANE_PIERCE=0`
/// restores the general search; held back with row 15 until row 26): any edge of
/// degree at most three against an affine plane carrier is seeded at the
/// closed-form roots of its weighted signed distance (`exact_curve_plane_seeds`),
/// a fillet's rational arc included; same Newton and filters as row 15.
pub(super) fn plane_pierce_enabled() -> bool {
    switch!("BREP_IMPRINT_PLANE_PIERCE", true)
}


pub(super) fn exact_pierce_enabled() -> bool {
    switch!("BREP_IMPRINT_EXACT_PIERCE", true)
}



