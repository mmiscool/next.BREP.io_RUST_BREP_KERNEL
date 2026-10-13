//! Shell-local acceptance certificates. The full repair/refusal path remains
//! the fallback. No geometry, IDs, names, trims or provenance are rewritten.
use crate::{AnalyticSurface, BrepSolid, KernelRefusal};

/// A single recognized spherical support cannot cross another face (there is
/// none) or fold through itself. Recognition reconstructs the full rational
/// sphere, including knots and weights, under the kernel's analytic-carrier
/// contract. A trim only restricts that support. With one loop there
/// is also no hole-containment floor to ask (`first_hole_breach` skips these).
///
/// Re-recognize from the stored net rather than trusting a populated cache:
/// this certificate must stand down if a caller has edited public controls.
/// Topology must validate before granting the certificate. Other quadrics,
/// multiple faces/shells and faces with holes always take full acceptance.
pub(super) fn exact_single_sphere(solid: &BrepSolid) -> bool {
    if solid.shells.len() != 1 || solid.shells[0].faces.len() != 1 {
        return false;
    }
    let face = &solid.shells[0].faces[0];
    if face.loops.len() != 1 || !solid.validate().is_empty() {
        return false;
    }
    matches!(
        crate::analytic_surface::recognize(&face.surface),
        Some(AnalyticSurface::Sphere { radius, .. }) if radius.is_finite() && radius > 0.0
    )
}

/// Enabled by default; `BREP_OFFSET_SPHERE_ACCEPT=0` / `off` restores the full scan.
pub(super) fn accept_offset_result(solid: BrepSolid) -> Result<BrepSolid, KernelRefusal> {
    let enabled = std::env::var("BREP_OFFSET_SPHERE_ACCEPT")
        .map_or(true, |value| value == "1" || value == "on");
    accept_offset_result_with_certificate(solid, enabled)
}

pub(super) fn accept_offset_result_with_certificate(
    solid: BrepSolid,
    enabled: bool,
) -> Result<BrepSolid, KernelRefusal> {
    if enabled && exact_single_sphere(&solid) {
        if std::env::var("BREP_PROFILE").is_ok() {
            eprintln!("offset_accept.profile certificate=single_sphere faces=1");
        }
        return Ok(solid);
    }
    crate::accept_sound(solid, "offsetShell")
}
