//! Sketch documents, constraint solving, interaction state, and overlay geometry.

use crate::json_support::vec3_or as read_vec3;
use crate::geometry3d::{cross3 as cross, dot3 as dot, sub3 as sub};

pub mod constraint_glyphs;
pub mod dimensions;
pub mod doc;
pub mod external_ref;
pub mod handdraw;
pub mod infer;
pub mod session;
pub mod solve;
pub mod spline;
pub mod tessellate;
pub mod trim;

pub use doc::{SketchConstraint, SketchDiagnostics, SketchDoc, SketchGeometry, SketchPoint};
pub use external_ref::{classify_uv, EdgeLink, ExternalRef};
pub use session::{
    constraint_ref, entity_ref_eq, geometry_ref, point_ref, refs_equal, SketchSession,
};
pub use solve::SketchSolverSettings;
pub use tessellate::SketchTessellation;

/// An orthonormal placement frame for a sketch plane — origin + in-plane `x`/`y`
/// axes + the `z` normal, all in world space (`f64`). A plane `(u, v)` coordinate
/// maps to world `origin + u·x + v·y` (via [`to_world`](Self::to_world)).
///
/// Uses double precision to match the sketch solver's coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlaneFrame {
    pub origin: [f64; 3],
    pub x_axis: [f64; 3],
    pub y_axis: [f64; 3],
    pub z_axis: [f64; 3],
}

impl PlaneFrame {
    /// The world XY plane (identity frame): `u → +x`, `v → +y`, normal `+z`.
    pub fn xy() -> Self {
        Self {
            origin: [0.0, 0.0, 0.0],
            x_axis: [1.0, 0.0, 0.0],
            y_axis: [0.0, 1.0, 0.0],
            z_axis: [0.0, 0.0, 1.0],
        }
    }

    /// The world XZ base plane (`datum.rs` normal `(0, -1, 0)`), resolved through
    /// the same worldUp convention the kernel uses (see [`from_normal`](Self::from_normal)).
    pub fn xz() -> Self {
        Self::from_normal([0.0, 0.0, 0.0], [0.0, -1.0, 0.0])
    }

    /// The world YZ base plane (`datum.rs` normal `(1, 0, 0)`), resolved through
    /// the same worldUp convention the kernel uses (see [`from_normal`](Self::from_normal)).
    pub fn yz() -> Self {
        Self::from_normal([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
    }

    /// Derive an orthonormal frame from an `origin` + plane `normal`, a faithful
    /// port of `feature_pipeline::Frame::from_origin_normal` — the kernel's SINGLE
    /// source of truth for how a plane reference becomes in-plane axes:
    ///
    /// ```text
    /// refUp = |n·(0,1,0)| > 0.9 ? (1,0,0) : (0,1,0)
    /// x = norm(refUp × n);  y = norm(n × x);  z = n
    /// ```
    ///
    /// A degenerate (zero / non-finite) normal — or a normal collinear with the
    /// picked `refUp` — returns the XY identity axes (at `origin`) rather than
    /// erroring, so callers always get a usable frame.
    pub fn from_normal(origin: [f64; 3], normal: [f64; 3]) -> Self {
        let identity = Self {
            origin,
            ..Self::xy()
        };
        let Some(z) = normalize(normal) else {
            return identity;
        };
        let world_up = [0.0, 1.0, 0.0];
        let ref_up = if dot(z, world_up).abs() > 0.9 {
            [1.0, 0.0, 0.0]
        } else {
            world_up
        };
        let Some(x) = normalize(cross(ref_up, z)) else {
            return identity;
        };
        let Some(y) = normalize(cross(z, x)) else {
            return identity;
        };
        Self {
            origin,
            x_axis: x,
            y_axis: y,
            z_axis: z,
        }
    }

    /// Read a persisted `persistentData.basis` object (`{origin, x, y, z}`, each a
    /// `[x, y, z]` array) into a frame, mirroring the kernel's `persisted_basis_frame`
    /// (`features/sketch.rs`). Missing keys default to the identity components, so a
    /// partial / absent basis still yields a usable XY-ish frame.
    pub fn from_basis_json(basis: &serde_json::Value) -> Self {
        Self {
            origin: read_vec3(basis.get("origin"), [0.0, 0.0, 0.0]),
            x_axis: read_vec3(basis.get("x"), [1.0, 0.0, 0.0]),
            y_axis: read_vec3(basis.get("y"), [0.0, 1.0, 0.0]),
            z_axis: read_vec3(basis.get("z"), [0.0, 0.0, 1.0]),
        }
    }

    /// Map a plane `(u, v)` coordinate to world `[x, y, z]`.
    pub fn to_world(&self, u: f64, v: f64) -> [f64; 3] {
        [
            self.origin[0] + self.x_axis[0] * u + self.y_axis[0] * v,
            self.origin[1] + self.x_axis[1] * u + self.y_axis[1] * v,
            self.origin[2] + self.x_axis[2] * u + self.y_axis[2] * v,
        ]
    }

    /// Project a world point onto the plane's `(u, v)` frame — the inverse of
    /// [`to_world`](Self::to_world). With orthonormal axes this is plain dot
    /// products against the offset from the origin (`d = world − origin`;
    /// `u = d·x_axis`, `v = d·y_axis`); a point off the plane projects orthogonally
    /// (its normal component is dropped). Mirrors the previous sketcher's
    /// world→UV projection.
    pub fn to_uv(&self, world: [f64; 3]) -> (f64, f64) {
        let d = [
            world[0] - self.origin[0],
            world[1] - self.origin[1],
            world[2] - self.origin[2],
        ];
        (dot(d, self.x_axis), dot(d, self.y_axis))
    }
}

impl Default for PlaneFrame {
    fn default() -> Self {
        Self::xy()
    }
}

/// Intersect a world-space ray (`origin` + `dir`) with a sketch `plane` and return
/// the hit's in-plane `(u, v)` coordinate, or `None` when the ray is parallel to
/// the plane (`|dir·n| < 1e-9`) or the hit is behind the ray origin (`t <= 0`).
///
/// This is the pure pixel→plane math behind
/// [`EngineState::sketch_uv_at`](crate::engine_state::EngineState::sketch_uv_at):
/// the caller supplies the camera ray (`camera.pick_ray(x, y)`); the plane's axes
/// are assumed orthonormal, so the world→uv projection is plain dot products.
pub fn ray_plane_uv(plane: &PlaneFrame, origin: [f64; 3], dir: [f64; 3]) -> Option<(f64, f64)> {
    let n = plane.z_axis;
    let denom = dot(dir, n);
    if denom.abs() < 1e-9 {
        return None; // ray parallel to the plane
    }
    let t = dot(sub(plane.origin, origin), n) / denom;
    if t <= 0.0 {
        return None; // plane is behind the ray origin
    }
    let hit = [
        origin[0] + t * dir[0],
        origin[1] + t * dir[1],
        origin[2] + t * dir[2],
    ];
    let w = sub(hit, plane.origin);
    Some((dot(w, plane.x_axis), dot(w, plane.y_axis)))
}

/// The BASE overlay color for one constraint's annotation — the shared constraint
/// green, or the conflict red while the solver names this constraint in a conflict.
/// The ONE place that decision is made, so a glyph and a dimension leader in the
/// same conflict are never colored differently. Selection / hover emphasis is
/// layered ON TOP of this by
/// [`tessellate::interaction_color`](crate::sketch::tessellate), so a picked
/// conflicting constraint still reads as picked.
pub(crate) fn constraint_base_color(
    colors: &crate::style::SketchColors,
    diag: &SketchDiagnostics,
    id: &serde_json::Value,
) -> u32 {
    if diag.constraint_conflicting(id) {
        colors.conflict
    } else {
        colors.constraint
    }
}

/// Normalize `v`, or `None` when it is (near) zero / non-finite.
fn normalize(v: [f64; 3]) -> Option<[f64; 3]> {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len.is_finite() && len > 1e-12 {
        Some([v[0] / len, v[1] / len, v[2] / len])
    } else {
        None
    }
}

