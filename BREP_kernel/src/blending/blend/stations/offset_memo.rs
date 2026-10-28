//! Per-solve offset samples, keyed by the exact arguments to each carrier.
use super::{blend_offset, tangency_from_samples, TangencyState};
use crate::{KernelRefusal, KernelStage, NurbsSurface, OffsetSample, OrRefuse, Vec3};

pub(super) fn enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("BREP_STATION_OFFSET_MEMO").ok().as_deref() == Some("1"))
}

#[derive(Clone, Copy)]
struct Reading { key: [u64; 3], sample: OffsetSample }

/// Three readings per mate retain a Newton base and its two ordinary Jacobian
/// probes. A chart probe may evict a reading; a miss always evaluates normally.
/// Carrier identity and the Raw normal convention are fixed for this solve.
pub(super) struct Samples<'a> {
    surfaces: [&'a NurbsSurface; 2],
    readings: [[Option<Reading>; 3]; 2],
    next: [usize; 2],
    evaluations: u64,
    hits: u64,
}

impl<'a> Samples<'a> {
    pub(super) fn new(surfaces: [&'a NurbsSurface; 2]) -> Self {
        Self { surfaces, readings: [[None; 3]; 2], next: [0; 2], evaluations: 0, hits: 0 }
    }

    fn read(&mut self, mate: usize, u: f64, v: f64, rho: f64) -> Result<OffsetSample, KernelRefusal> {
        let key = [u.to_bits(), v.to_bits(), rho.to_bits()];
        for reading in self.readings[mate].iter().flatten() {
            if reading.key == key {
                self.hits += 1;
                return Ok(reading.sample);
            }
        }
        self.evaluations += 1;
        let sample = blend_offset(self.surfaces[mate]).at(u, v, rho).or_refuse(KernelStage::Refine, "at")?;
        self.readings[mate][self.next[mate]] = Some(Reading { key, sample });
        self.next[mate] = (self.next[mate] + 1) % 3;
        Ok(sample)
    }

    pub(super) fn state(&mut self, rho: [f64; 2], uv: [f64; 4], section_point: Vec3, section_tangent: Vec3) -> Result<TangencyState, KernelRefusal> {
        let first = self.read(0, uv[0], uv[1], rho[0])?;
        let second = self.read(1, uv[2], uv[3], rho[1])?;
        Ok(tangency_from_samples(first, second, section_point, section_tangent))
    }
}

impl Drop for Samples<'_> {
    fn drop(&mut self) {
        crate::blend::record_count("count.march.offset_evaluations", self.evaluations);
        crate::blend::record_count("count.march.offset_memo_hits", self.hits);
    }
}


