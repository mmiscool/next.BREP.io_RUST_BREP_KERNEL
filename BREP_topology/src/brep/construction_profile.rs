//! Construction-stage profile lines (`BREP_PROFILE`) shared by primitives, lofts and the soundness grid.


/// Consecutive construction stages, emitted only under the existing profiling switch.
/// Nested builder/acceptance times are hierarchical, not additive.
pub struct ConstructionProfile {
    builder: &'static str,
    started: Option<web_time::Instant>,
}
impl ConstructionProfile {
    pub fn new(builder: &'static str) -> Self {
        let enabled = crate::profile::profile_enabled();
        Self { builder, started: enabled.then(web_time::Instant::now) }
    }
    pub fn lap(&mut self, phase: &str) {
        if let Some(started) = &mut self.started {
            profile_line!("construction.profile builder={} {}_ms={:.6}",
                self.builder, phase, started.elapsed().as_secs_f64() * 1000.0);
            *started = web_time::Instant::now();
        }
    }
}
