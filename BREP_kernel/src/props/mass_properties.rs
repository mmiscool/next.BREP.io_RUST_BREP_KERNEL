use crate::topology::{BrepSolid, EdgeRecord, FaceRecord, ShellRecord};
use crate::{NurbsCurve, NurbsSurface, Vec3};
use serde::Serialize;

/// Eight-station Gauss–Legendre on `[-1, 1]`. `pub(crate)` because
/// `brep/soundness.rs`'s vector-area closure integrates the trims with the
/// same rule the mass integrals use — one table, not a third copy of it.
pub(crate) const GAUSS_X: [f64; 8] = [
    -0.9602898564975363,
    -0.7966664774136267,
    -0.525532409916329,
    -0.18343464249564978,
    0.18343464249564978,
    0.525532409916329,
    0.7966664774136267,
    0.9602898564975363,
];
pub(crate) const GAUSS_W: [f64; 8] = [
    0.10122853629037669,
    0.22238103445337445,
    0.31370664587788727,
    0.362683783378362,
    0.362683783378362,
    0.31370664587788727,
    0.22238103445337445,
    0.10122853629037669,
];

#[derive(Clone, Copy)]
enum Integrand {
    Area,
    Volume,
    VolumeAbout(Vec3),
    /// Divergence-theorem first moments: ∫x dV = ∮ (x²/2)·nx dA, etc.
    MomentX,
    MomentY,
    MomentZ,
    /// Second moments: ∫x² dV = ∮ (x³/3)·nx dA, etc.
    SecondXX,
    SecondYY,
    SecondZZ,
    /// Products: ∫xy dV = ∮ (x²y/2)·nx dA, ∫xz, ∫yz analogously.
    ProductXY,
    ProductXZ,
    ProductYZ,
}

/// A bounded, per-solid memo for exact area and volume. Geometry edits are
/// detected by the face-set content identity; callers never invalidate it.
/// The contents are private so external code cannot supply a fabricated result.
#[derive(Clone, Debug, Default)]
pub struct MassPropertiesCache {
    entry: std::cell::Cell<Option<(FaceSetIdentity, MassProperties)>>,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct MassProperties {
    pub surface_area: f64,
    pub volume: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct FullMassProperties {
    pub surface_area: f64,
    pub volume: f64,
    /// Volume centroid (unit density center of mass).
    pub centroid: Vec3,
    /// Inertia tensor about the centroid for unit density, row-major
    /// [[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]].
    pub inertia: [[f64; 3]; 3],
    /// Principal moments of inertia (eigenvalues of `inertia`) at unit
    /// density, sorted ASCENDING: `principal_moments[0] <= [1] <= [2]`.
    /// Golovanov §8.11.
    pub principal_moments: [f64; 3],
    /// Principal axes of inertia (eigenvectors of `inertia`), one per row:
    /// `principal_axes[i]` is the unit axis whose moment is
    /// `principal_moments[i]`. The three axes are orthonormal and arranged as
    /// a RIGHT-HANDED frame (determinant +1); the sign of the third axis is
    /// fixed to enforce handedness. Density-independent.
    pub principal_axes: [[f64; 3]; 3],
}

/// Mass properties scaled to a physical density (Golovanov §8.11). The
/// underlying `FullMassProperties` is a UNIT-density geometric result; here
/// `mass = density * volume` and every inertia quantity scales linearly with
/// density (`inertia = density * geometric_inertia`). The centroid and the
/// principal AXES are density-independent; only the principal MOMENTS and the
/// mass scale with density.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct DensityMassProperties {
    /// Density used to scale the geometric (unit-density) properties.
    pub density: f64,
    /// `mass = density * volume`.
    pub mass: f64,
    pub surface_area: f64,
    pub volume: f64,
    /// Center of mass (identical to the unit-density centroid).
    pub centroid: Vec3,
    /// Centroidal inertia tensor `= density * geometric_inertia`, row-major.
    pub inertia: [[f64; 3]; 3],
    /// Principal moments `= density * geometric_principal_moments`, ascending.
    pub principal_moments: [f64; 3],
    /// Principal axes (identical to the unit-density axes; density-invariant).
    pub principal_axes: [[f64; 3]; 3],
}

impl FullMassProperties {
    /// Scale the unit-density geometric result to a physical `density`
    /// (Golovanov §8.11): `mass = density * volume`, `inertia` and the
    /// principal moments scale by `density`, while the centroid and principal
    /// axes are unchanged. Panics only on a non-finite density is avoided —
    /// callers should pass a finite, positive density; a zero density yields a
    /// massless (all-zero inertia) result, which is well defined.
    pub fn with_density(&self, density: f64) -> DensityMassProperties {
        let scale = |matrix: [[f64; 3]; 3]| {
            let mut out = [[0.0f64; 3]; 3];
            for row in 0..3 {
                for column in 0..3 {
                    out[row][column] = density * matrix[row][column];
                }
            }
            out
        };
        DensityMassProperties {
            density,
            mass: density * self.volume,
            surface_area: self.surface_area,
            volume: self.volume,
            centroid: self.centroid,
            inertia: scale(self.inertia),
            principal_moments: [
                density * self.principal_moments[0],
                density * self.principal_moments[1],
                density * self.principal_moments[2],
            ],
            principal_axes: self.principal_axes,
        }
    }
}


/// Per-call counters for the trimmed-face integrator, printed to stderr when
/// `BREP_PROFILE` is set (the switch `boolean.<stage>_ms` and `imprint.*`
/// use). One line per `solid_mass_properties` / `solid_signed_volume` /
/// `solid_mass_properties_full` call: which route each trimmed face took,
/// how many quadrature stations the cells, the boundary triangles and the
/// chord-arc correction cost, and the wall time of each part. Native only in
/// practice (no env vars in wasm); when the switch is off every counter is a
/// thread-local flag test.
#[path = "mass_properties/profile.rs"]
mod profile;
pub(crate) use profile::mass_profile;
pub use profile::{mass_caller, MassCaller};

/// The content identity of the face set an integrator call reads — the
/// instrument behind the sign-check pairing, and the key a served value is
/// checked against.
#[path = "mass_properties/identity.rs"]
mod identity;

#[path = "mass_properties/integration.rs"]
mod integration;
#[path = "mass_properties/measures.rs"]
mod measures;
#[path = "mass_properties/polygons.rs"]
mod polygons;
/// Which Gauss–Legendre rule reads one span, and where it is cut: the answer
/// is the span's own WEIGHTS, because a fixed-order rule is exact for a
/// polynomial integrand and a rational one is not.
#[path = "mass_properties/rule.rs"]
mod rule;
#[path = "mass_properties/solid_props.rs"]
mod solid_props;
#[path = "mass_properties/trimmed.rs"]
mod trimmed;
#[path = "mass_properties/winding.rs"]
mod winding;

use integration::*;
use measures::*;
use polygons::*;
use trimmed::*;

pub use integration::parameter_space_area;
// The two span helpers `brep/soundness.rs`'s vector-area closure shares with
// the mass integrals: the surface's knot levels, and a pcurve's spans cut
// wherever it crosses one. A boundary integral read across a knot line is the
// same 0.3%-off reading `winding_loops_integral` documents.
pub(crate) use integration::surface_breaks;
pub(crate) use winding::split_at_surface_breaks;
pub use measures::{
    curve_arc_length, edge_arc_length, face_area, face_boundary_length,
    face_volume_contribution, solid_edge_length_total,
};
pub use solid_props::{solid_mass_properties, solid_mass_properties_full, solid_signed_volume};
pub(crate) use solid_props::shell_signed_volume;
pub use polygons::trim_polygons;
pub use identity::{face_set_identity, FaceSetIdentity};
