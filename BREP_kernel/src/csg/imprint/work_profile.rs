//! Work counts only. All geometry serialization and history retention are behind
//! BREP_PROFILE. A separate process per history makes previous-imprint counts
//! unambiguous; these are observations, never a geometry-result cache.
use super::*;
use std::cell::RefCell;

#[derive(Clone, Copy, Default, Serialize)]
pub(crate) struct Counts {
    pub sections: u64,
    pub projection_global: u64,
    pub projection_seeded: u64,
    pub projection_from_seed: u64,
    pub pcurve_native_passes: u64,
    pub pcurve_native_samples: u64,
    pub pcurve_probe_passes: u64,
    pub pcurve_probe_samples: u64,
    pub carried_uv_curves: u64,
    pub carried_uv_stations: u64,
    pub minted_pieces: u64,
    pub trace_calls: u64,
    pub trace_curves: u64,
    pub trace_samples: u64,
    pub fit_reports: u64,
    pub fit_calls: u64,
    pub refinement_rounds: u64,
    pub fit_samples: u64,
    pub fit_raised: u64,
    pub newton_iterations: u64,
    pub newton_unused: u64,
    pub newton_unused_first: u64,
}
thread_local! {
    static COUNTS: RefCell<Counts> = RefCell::new(Counts::default());
    static PREVIOUS_FACES: RefCell<HashSet<Vec<u8>>> = RefCell::new(HashSet::default());
    static PREVIOUS: RefCell<HashSet<Vec<u8>>> = RefCell::new(HashSet::default());
}
#[inline]
pub(crate) fn add(update: impl FnOnce(&mut Counts)) {
    if profile_enabled() { COUNTS.with(|c| update(&mut c.borrow_mut())); }
}
/// Successful construction routes, distinct from cached fit report reads.
pub(crate) fn constructed(route: &'static str, samples: usize, controls: usize) {
    if profile_enabled() {
        profile_line!("pcurve.construction route={} samples={} controls={}", route, samples, controls);
    }
}
fn counts() -> Counts { COUNTS.with(|c| *c.borrow()) }

pub(crate) struct Scope { label: &'static str, before: Counts }
impl Scope {
    pub(crate) fn open(label: &'static str) -> Option<Self> {
        profile_enabled().then(|| Self { label, before: counts() })
    }
}
impl Drop for Scope {
    fn drop(&mut self) {
        let a = serde_json::to_value(counts()).unwrap();
        let b = serde_json::to_value(self.before).unwrap();
        let delta: serde_json::Map<String, serde_json::Value> = a.as_object().unwrap().iter()
            .map(|(k, v)| (k.clone(), (v.as_u64().unwrap() - b[k].as_u64().unwrap()).into())).collect();
        profile_line!("{}.work {}", self.label, serde_json::Value::Object(delta));
    }
}

pub(super) struct Pairs {
    current: HashSet<Vec<u8>>,
    repeated: usize,
    productive: usize,
    minted: usize,
    possible: usize,
}
impl Pairs {
    pub(super) fn new(possible: usize) -> Option<Self> {
        profile_enabled().then(|| Self { current: HashSet::default(), repeated: 0, productive: 0, minted: 0, possible })
    }
    pub(super) fn pair(&mut self, first: &FaceRecord, second: &FaceRecord) {
        // Full serialized records, including trims, orientation and IDs: a
        // conservative exact-repeat count, not just matching surface nets.
        let key = serde_json::to_vec(&(first, second)).unwrap();
        self.repeated += usize::from(PREVIOUS.with(|p| p.borrow().contains(&key)));
        self.current.insert(key);
    }
    pub(super) fn produced(&mut self, before: Counts) {
        self.productive += usize::from(counts().sections > before.sections);
        self.minted += usize::from(counts().minted_pieces > before.minted_pieces);
    }
}
impl Drop for Pairs {
    fn drop(&mut self) {
        profile_line!("imprint.work_pairs possible={} overlapping={} section_pairs={} minted_pairs={} repeated_previous={}",
            self.possible, self.current.len(), self.productive, self.minted, self.repeated);
        PREVIOUS.with(|p| *p.borrow_mut() = std::mem::take(&mut self.current));
    }
}
pub(super) struct Pair<'a> { pairs: &'a mut Option<Pairs>, before: Counts }
impl<'a> Pair<'a> {
    pub(super) fn open(pairs: &'a mut Option<Pairs>, first: &FaceRecord, second: &FaceRecord) -> Self {
        if let Some(p) = pairs { p.pair(first, second); }
        Self { before: if pairs.is_some() { counts() } else { Counts::default() }, pairs }
    }
}
impl Drop for Pair<'_> {
    fn drop(&mut self) { if let Some(p) = self.pairs { p.produced(self.before); } }
}

pub(super) fn trace(first: &NurbsSurface, second: &NurbsSurface, options: &SurfaceIntersectionOptions)
    -> Result<Vec<crate::SurfaceIntersectionCurve>, String>
{
    add(|c| c.trace_calls += 1);
    let result = crate::intersect_surfaces(first, second, options);
    if let Ok(curves) = &result {
        add(|c| { c.trace_curves += curves.len() as u64;
            c.trace_samples += curves.iter().map(|v| v.points.len() as u64).sum::<u64>(); });
    }
    result
}

pub(super) fn supplemental(first: &NurbsSurface, second: &NurbsSurface, options: &SurfaceIntersectionOptions)
    -> Result<Vec<crate::SurfaceIntersectionCurve>, String>
{
    add(|c| c.trace_calls += 1);
    let result = crate::intersect_surfaces_supplemental(first, second, options);
    if let Ok(curves) = &result {
        add(|c| { c.trace_curves += curves.len() as u64;
            c.trace_samples += curves.iter().map(|v| v.points.len() as u64).sum::<u64>(); });
    }
    result
}

pub(super) fn faces<'a>(faces: impl Iterator<Item = (&'a FaceRecord, bool)>) {
    if !profile_enabled() { return; }
    let mut current = HashSet::default();
    let (mut inputs, mut with_sections, mut repeated, mut repeated_without_sections) = (0, 0, 0, 0);
    PREVIOUS_FACES.with(|previous| {
        let mut previous = previous.borrow_mut();
        for (face, touched) in faces {
            inputs += 1;
            with_sections += usize::from(touched);
            let key = serde_json::to_vec(face).unwrap();
            if previous.contains(&key) {
                repeated += 1;
                repeated_without_sections += usize::from(!touched);
            }
            current.insert(key);
        }
        *previous = current;
    });
    profile_line!("imprint.work_faces inputs={inputs} with_sections={with_sections} without_sections={} repeated_previous={repeated} repeated_without_sections={repeated_without_sections}", inputs-with_sections);
}
