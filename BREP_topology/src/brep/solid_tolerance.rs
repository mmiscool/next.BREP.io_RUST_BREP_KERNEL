//! Model scale and size of a solid, for the tolerance bands of `KernelTolerances`.

use crate::topology::BrepSolid;
use crate::*;

impl crate::SolidScale for BrepSolid {
    fn solid_scale(&self) -> f64 {
        solid_scale(self)
    }
}


/// [`model_scale`] of a solid's vertices — the RAW bounding-box diagonal, with
/// no `>= 1.0` floor.  This is what direct-edit wants: a sub-unit part must keep
/// sub-unit bands.  (`BrepSolid::validate` wants the same thing and still
/// hand-rolls it, for the reason its own comment gives — it answers `0.0`, not
/// `1.0`, for a vertexless solid.  Reconciling that contract is a follow-up, not
/// a free ride-along.)  Use [`solid_scale`] for the floored variant the
/// tolerance policy is built on.
pub fn solid_model_scale(solid: &BrepSolid) -> f64 {
    model_scale(solid.vertices.iter().map(|vertex| vertex.point))
}

/// [`solid_model_scale`] floored at 1.0 — the characteristic length
/// [`KernelTolerances::for_solid`] and the boolean/imprint/heal sites are tuned
/// against.  The floor keeps a unit-and-below part on the historical bands those
/// call sites were fitted to; new size-relative sites should prefer the unfloored
/// [`model_scale`] family and floor explicitly where they mean to.
pub fn solid_scale(solid: &BrepSolid) -> f64 {
    solid_model_scale(solid).max(1.0)
}

/// The app's display chord tolerance (`BetterSolid._kernelTessellationOptions`):
/// vertex |coord| extent — falling back to the control-point hull / sqrt(2) for
/// vertex-free solids (full spheres/tori) — times 1.5e-3, times the render-LOD
/// factor (1.0 = the app's "Normal" preset).
pub fn display_chord_tolerance(solid: &BrepSolid, lod_factor: f64) -> f64 {
    let mut extent = 0.0f64;
    for vertex in &solid.vertices {
        extent = extent
            .max(vertex.point.x.abs())
            .max(vertex.point.y.abs())
            .max(vertex.point.z.abs());
    }
    if extent <= 0.0 {
        let mut control_point_extent = 0.0f64;
        for shell in &solid.shells {
            for face in &shell.faces {
                for row in &face.surface.control_points {
                    for cp in row {
                        let w = if cp.w != 0.0 { cp.w } else { 1.0 };
                        control_point_extent = control_point_extent
                            .max((cp.x / w).abs())
                            .max((cp.y / w).abs())
                            .max((cp.z / w).abs());
                    }
                }
            }
        }
        extent = control_point_extent / std::f64::consts::SQRT_2;
    }
    extent.max(1e-9) * 1.5e-3 * lod_factor
}
