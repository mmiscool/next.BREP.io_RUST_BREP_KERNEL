//! A sheet of REVOLUTION thickened to the centre its offset collapses to
//! (`BREP_THICKEN_TO_CENTER`, default on; `=0` restores the refusal).
//!
//! A sheet whose every normal segment of length |thickness| ends at ONE point
//! (a spherical sheet thickened by its own radius toward its centre) has no
//! offset face: the offset is that point. The thicken, the union of the normal
//! segments, is then the cone of segments from the sheet to the point. For a
//! surface of revolution about an axis through the point it is the region the
//! meridian's sector (the meridian closed by the two segments from its ends to
//! the point) sweeps, revolved by the sheet's own sweep: a half ball for the
//! hemisphere thickened by 5 to its centre (brep-bench
//! thicken/hemisphere-to-its-center, 2πR³/3 and 3πR²), refused before as
//! "offset ... self-intersects ... curvature radius 5 is not larger than the
//! offset distance".
//!
//! Recognition is measured: on a 9 × 9 grid over the whole domain every
//! offset point lies within 1e-9 of the sheet's size of the first, and that
//! point lies on the revolution axis. Anything else is `None`, and the
//! ordinary thicken runs unchanged.
use crate::{KernelRefusal, KernelStage, NurbsSurface, OrRefuse, Vec3};
use crate::topology::BrepSolid;

pub(super) fn enabled() -> bool {
    switch!("BREP_THICKEN_TO_CENTER", true)
}


/// The solid of the whole-domain `surface` thickened by `thickness` along its
/// normal `Su × Sv` (negative: against it), where that offset is one point on
/// the surface's revolution axis; `None` otherwise.
pub(super) fn build(surface: &NurbsSurface, thickness: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    if !enabled() {
        return Ok(None);
    }
    let Some(crate::AnalyticSurface::Revolution { frame, sweep, .. }) = surface.analytic() else {
        return Ok(None);
    };
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    // The sheet's size, for the coincidence bar.
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    let mut images = Vec::new();
    const STATIONS: usize = 8;
    for i in 0..=STATIONS {
        for j in 0..=STATIONS {
            let u = u0 + (u1 - u0) * i as f64 / STATIONS as f64;
            let v = v0 + (v1 - v0) * j as f64 / STATIONS as f64;
            let point = surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
            low = Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
            high = Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
            // A pole has no normal; its segment is the meridian's, read at the
            // stations beside it.
            let Ok(normal) = surface.normal(u, v) else { continue };
            let Ok(normal) = normal.normalized() else { continue };
            images.push(point.add(normal.scale(thickness)));
        }
    }
    let size = high.sub(low).length().max(1e-9);
    let bar = 1e-9 * size.max(1.0);
    let Some(&centre) = images.first() else {
        return Ok(None);
    };
    if images.len() < (STATIONS + 1) * (STATIONS + 1) / 2 || images.iter().any(|image| image.sub(centre).length() > bar) {
        return Ok(None);
    }
    let along = centre.sub(frame.origin);
    if along.sub(frame.axis.scale(along.dot(frame.axis))).length() > bar {
        return Ok(None);
    }
    let centre = frame.origin.add(frame.axis.scale(along.dot(frame.axis)));
    // The sector: the meridian at u0, closed through the centre.
    let meridian = surface.iso_curve_u(u0).or_refuse(KernelStage::Refine, "iso_curve_u")?;
    let [t0, t1] = meridian.domain().or_refuse(KernelStage::Refine, "domain")?;
    let start = meridian.evaluate(t0).or_refuse(KernelStage::Refine, "evaluate")?;
    let end = meridian.evaluate(t1).or_refuse(KernelStage::Refine, "evaluate")?;
    let mut profile = vec![meridian];
    for (from, to) in [(end, centre), (centre, start)] {
        if from.sub(to).length() > bar {
            profile.push(crate::make_line(from, to).or_refuse(KernelStage::Refine, "make_line")?);
        }
    }
    let solid = crate::revolve_profile_brep(&profile, frame.origin, frame.axis, sweep.min(std::f64::consts::TAU))
        .or_refuse(KernelStage::Refine, "revolve_profile_brep")?;
    // The revolve must bound the sheet's sector: a positive volume, and the
    // sheet's middle on its boundary.
    let volume = crate::solid_mass_properties(&solid).or_refuse(KernelStage::Validate, "solid_mass_properties")?.volume;
    let middle = surface.evaluate(0.5 * (u0 + u1), 0.5 * (v0 + v1)).or_refuse(KernelStage::Refine, "evaluate")?;
    let class = crate::classify_point(middle, &solid, bar * 10.0).or_refuse(KernelStage::Validate, "classify_point")?.class;
    if !(volume > 0.0) || class != crate::PointClass::On || !solid.validate().is_empty() {
        return Ok(None);
    }
    Ok(Some(solid))
}

