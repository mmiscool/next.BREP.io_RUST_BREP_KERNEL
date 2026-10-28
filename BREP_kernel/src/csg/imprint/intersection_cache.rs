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
        return crate::intersect_curve_surface(curve, surface, tolerance);
    }
    let key = key(curve, surface, tolerance);
    let hit = CACHES.with(|stack| {
        let mut stack = stack.borrow_mut();
        let cache = stack.last_mut().unwrap();
        cache.requests += 1;
        let hit = cache.results.get(&key).cloned();
        if hit.is_none() { cache.solves += 1; }
        hit
    });
    if let Some(result) = hit { return Ok(result); }
    let result = crate::intersect_curve_surface(curve, surface, tolerance)?;
    CACHES.with(|stack| {
        stack.borrow_mut().last_mut().unwrap().results.insert(key, result.clone());
    });
    Ok(result)
}

