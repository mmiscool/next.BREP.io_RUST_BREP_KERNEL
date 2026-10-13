//! Reuse deterministic fits within one imprint, keyed by complete input nets.
//! A cache hit replays the original report into the normal diagnostic ledger.
use super::*;
use std::cell::RefCell;

#[derive(Default)]
struct FitCache {
    fits: HashMap<Vec<u64>, crate::PcurveFit>,
    requests: usize,
    hits: usize,
    evaluations: usize,
    key_time: Duration,
    lookup_time: Duration,
    fit_time: Duration,
    replay_time: Duration,
}

thread_local! {
    static CACHES: RefCell<Vec<Option<FitCache>>> = const { RefCell::new(Vec::new()) };
}

pub(super) fn enabled() -> bool {
    !matches!(std::env::var("BREP_IMPRINT_PCURVE_CACHE").as_deref(), Ok("0") | Ok("off"))
}

/// Separate nested operations, including error returns, never retain a cache
/// on their caller. The only mutable state is local to this thread and scope.
pub(super) struct FitCacheScope {
    depth: usize,
}
impl FitCacheScope {
    pub(super) fn open(enabled: bool) -> Self {
        let depth = CACHES.with(|stack| {
            let mut stack = stack.borrow_mut();
            let depth = stack.len();
            stack.push(enabled.then(FitCache::default));
            depth
        });
        Self { depth }
    }
}
impl Drop for FitCacheScope {
    fn drop(&mut self) {
        CACHES.with(|stack| {
            let mut stack = stack.borrow_mut();
            debug_assert_eq!(stack.len(), self.depth + 1);
            if let Some(Some(cache)) = stack.pop() {
                if std::env::var_os("BREP_PROFILE").is_some() {
                    eprintln!("imprint.pcurve_cache requests={} hits={} evaluations={} retained={}",
                        cache.requests, cache.hits, cache.evaluations, cache.fits.len());
                    eprintln!("imprint.pcurve_cache_time key={:.3} lookup={:.3} fit={:.3} replay={:.3}",
                        cache.key_time.as_secs_f64() * 1000.0, cache.lookup_time.as_secs_f64() * 1000.0,
                        cache.fit_time.as_secs_f64() * 1000.0, cache.replay_time.as_secs_f64() * 1000.0);
                }
            }
        });
    }
}

/// Lengths delimit every field; equality includes every IEEE bit, including
/// signed zero. Recognition labels and pointer identity cannot alias a net.
fn key(surface: &NurbsSurface, curve: &NurbsCurve) -> Vec<u64> {
    fn floats(key: &mut Vec<u64>, values: &[f64]) {
        key.push(values.len() as u64);
        key.extend(values.iter().map(|v| v.to_bits()));
    }
    fn controls(key: &mut Vec<u64>, values: &[Vec4]) {
        key.push(values.len() as u64);
        for p in values { key.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits(), p.w.to_bits()]); }
    }
    // Imprint face charts are borrowed for the entire scope. Keeping their
    // identity as well as every net bit prevents reuse across distinct
    // objects carrying different lazy projection/recognition cache states.
    let mut key = vec![surface as *const NurbsSurface as usize as u64,
        surface.degree_u as u64, surface.degree_v as u64];
    floats(&mut key, &surface.knots_u);
    floats(&mut key, &surface.knots_v);
    key.push(surface.control_points.len() as u64);
    for row in &surface.control_points { controls(&mut key, row); }
    key.push(curve.degree as u64);
    floats(&mut key, &curve.knots);
    controls(&mut key, &curve.control_points);
    // Other fit-policy switches are process OnceLocks; this one is dynamic.
    key.push((std::env::var("BREP_SECTION_PCURVE_MARCH").as_deref() == Ok("0")) as u64);
    key
}

pub(super) fn cached_build_pcurve(surface: &NurbsSurface, curve: &NurbsCurve) -> Result<NurbsCurve, String> {
    let active = CACHES.with(|stack| stack.borrow().last().is_some_and(Option::is_some));
    if !active {
        return crate::build_pcurve_on_surface_marched(surface, curve);
    }
    let profiling = std::env::var_os("BREP_PROFILE").is_some();
    let mut clock = profiling.then(Instant::now);
    let mut lap = || {
        let now = profiling.then(Instant::now);
        let elapsed = clock.zip(now).map_or(Duration::ZERO, |(before, after)| after - before);
        clock = now;
        elapsed
    };
    let key = key(surface, curve);
    let key_time = lap();
    let hit = CACHES.with(|stack| {
        let mut stack = stack.borrow_mut();
        let cache = stack.last_mut().unwrap().as_mut().unwrap();
        cache.requests += 1;
        let hit = cache.fits.get(&key).cloned();
        if hit.is_some() { cache.hits += 1; } else { cache.evaluations += 1; }
        hit
    });
    let lookup_time = lap();
    let fit = match hit {
        Some(fit) => fit,
        None => {
            let fit = crate::fit_pcurve_on_surface_marched(surface, curve)?;
            CACHES.with(|stack| {
                let mut stack = stack.borrow_mut();
                let cache = stack.last_mut().unwrap().as_mut().unwrap();
                // Bound retained memory; a full cache keeps evaluating misses.
                if cache.fits.len() < 256 { cache.fits.insert(key, fit.clone()); }
            });
            fit
        }
    };
    let fit_time = lap();
    crate::pcurve::record_fit(&fit.report);
    let replay_time = lap();
    if profiling {
        CACHES.with(|stack| {
            let mut stack = stack.borrow_mut();
            let cache = stack.last_mut().unwrap().as_mut().unwrap();
            cache.key_time += key_time;
            cache.lookup_time += lookup_time;
            cache.fit_time += fit_time;
            cache.replay_time += replay_time;
        });
    }
    Ok(fit.curve)
}
