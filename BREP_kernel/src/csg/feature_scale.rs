//! Feature-scale tolerance: the boolean's identity bands scale with the part,
//! so a feature far thinner than the part's extent is merged away. Under the
//! `BREP_FEATURE_SCALE_*` switches (all default on; `=0` restores each band)
//! a band that would reach across such a feature is capped at [`FRACTION`] of
//! its size.
//!
//! This module measures the pair-derived feature size (`lfs_pair` in the
//! design): two plane faces of the two operands that are almost parallel
//! bound a sliver between them, as thin as their largest separation over
//! the region both patches cover. brep-bench tangent/wedge-a-millionth-steep:
//! the cube's top and the half-space's face 1e-6 steep bound a wedge 5e-6
//! thick at its far end.

use crate::spatial::Aabb;
use crate::{BrepSolid, NurbsSurface, Vec3};

/// `BREP_FEATURE_SCALE_IMPRINT` (default on, `=0` off): the imprint's identity bands
/// (B1″, the edge-split endpoint guard, so far) are capped at the local
/// feature size.
pub(crate) fn imprint_enabled() -> bool {
    crate::switch!("BREP_FEATURE_SCALE_IMPRINT", true)
}

/// `BREP_FEATURE_SCALE_SELECT` (default on, `=0` off): selection's coincident-On
/// rescue band (B6) is capped at the local feature size. It acts only with
/// [`imprint_enabled`]: a sliver kept from the rescue but never imprinted is
/// classified whole, and the wedge then built a valid two-face solid of the
/// wrong volume (all lane switches on but `_IMPRINT`, 2026-11-07).
pub(crate) fn select_enabled() -> bool {
    crate::switch!("BREP_FEATURE_SCALE_SELECT", true) && imprint_enabled()
}

/// `BREP_FEATURE_SCALE_FRAGMENT` (default on, `=0` off): the fragmenter measures a
/// strip between two cuts as it measures one between a cut and the face's
/// boundary, and resolves the arrangement to it (B4).
pub(crate) fn fragment_enabled() -> bool {
    crate::switch!("BREP_FEATURE_SCALE_FRAGMENT", true)
}

/// The fraction of the local feature size a capped band may reach.
pub(crate) const FRACTION: f64 = 0.25;

/// Plane pairs whose normals' cross product is below this record a sliver.
const THIN_ANGLE: f64 = 1e-3;

/// A sliver between two almost parallel plane faces: `height` is the largest
/// separation of the two planes over `region`, the box both patches cover.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ThinPair {
    pub(crate) height: f64,
    pub(crate) region: Aabb,
    /// A slab of one operand's material (two of its faces facing away from
    /// each other), not a gap between the two operands' faces.
    pub(crate) same_operand: bool,
}

/// The local feature sizes of one boolean's operands. A cap is linear in the
/// slivers; they are measured only under a switch.
/// The longest band any capped site applies (the fuzzy-junction clamp,
/// 1.5e-3): a sliver whose [`FRACTION`] of its height is longer cannot limit one.
const LONGEST_BAND: f64 = 1.5e-3;

#[derive(Clone, Debug, Default)]
pub(crate) struct FeatureScale {
    pub(crate) thin_pairs: Vec<ThinPair>,
}

impl FeatureScale {
    /// Every sliver between two almost parallel plane faces (normals' cross
    /// product under [`THIN_ANGLE`], exactly parallel included):
    /// - of the two operands: the wedge's top and the half-space's face 1e-6
    ///   steep. Two such planes no farther apart than the imprint's coincidence
    ///   band (`max(10·tolerance, COINCIDENCE_DISTANCE_FLOOR)`, as
    ///   `coplanar_pair` reads it) over the region are one surface, not a
    ///   sliver: mirrored or transformed coplanar faces differ by round-off
    ///   (cross 1e-16, height 1e-15 on the 2026-10-09 mirrored shell history),
    ///   and the coplanar lanes treat them;
    /// - of one operand, facing away from each other: the two walls of a slab
    ///   of material (the hairline saw, 1e-6 thick). A solid's opposite walls
    ///   are never one surface, so these count once apart by more than the
    ///   tolerance.
    ///
    /// Only slivers thin enough to limit a band are kept ([`FRACTION`] of the
    /// height under [`LONGEST_BAND`]). The region is the box the two patches
    /// share; where they do not overlap along an axis (the saw's walls, at
    /// x = 5 and x = 5.000001) it spans the gap between them, when that gap
    /// is within the longest band's reach.
    pub(crate) fn between(first_solid: &BrepSolid, second_solid: &BrepSolid, tolerance: f64) -> Result<Self, String> {
        let coincident = (tolerance * 10.0).max(crate::tolerance::COINCIDENCE_DISTANCE_FLOOR);
        let mut planes: Vec<(u8, Vec3, Vec3, Aabb, &crate::FaceRecord)> = Vec::new();
        for (operand, solid) in [(0u8, first_solid), (1u8, second_solid)] {
            for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
                if let Some((origin, normal)) = plane_of(&face.surface)? {
                    planes.push((operand, origin, normal, Aabb::from_surface_controls(&face.surface)?, face));
                }
            }
        }
        let reach = 4.0 * LONGEST_BAND;
        let mut thin_pairs = Vec::new();
        for (i, &(a_operand, a_origin, a_normal, a_box, a_face)) in planes.iter().enumerate() {
            for &(b_operand, b_origin, b_normal, b_box, b_face) in &planes[i + 1..] {
                let same_operand = a_operand == b_operand;
                if same_operand && a_normal.dot(b_normal) >= 0.0 {
                    continue;
                }
                if a_normal.cross(b_normal).length() >= THIN_ANGLE {
                    continue;
                }
                let span = |lo_a: f64, hi_a: f64, lo_b: f64, hi_b: f64| -> Option<(f64, f64)> {
                    let (lo, hi) = (lo_a.max(lo_b), hi_a.min(hi_b));
                    if lo <= hi {
                        Some((lo, hi))
                    } else if lo - hi <= reach {
                        Some((hi, lo))
                    } else {
                        None
                    }
                };
                let (Some(x), Some(y), Some(z)) = (
                    span(a_box.minimum.x, a_box.maximum.x, b_box.minimum.x, b_box.maximum.x),
                    span(a_box.minimum.y, a_box.maximum.y, b_box.minimum.y, b_box.maximum.y),
                    span(a_box.minimum.z, a_box.maximum.z, b_box.minimum.z, b_box.maximum.z),
                ) else {
                    continue;
                };
                let region = Aabb { minimum: Vec3::new(x.0, y.0, z.0), maximum: Vec3::new(x.1, y.1, z.1) };
                // The region's corners, put onto each plane, measured from the
                // other: the largest is the sliver's thickness over the region.
                let mut height = 0.0f64;
                for corner in corners(region) {
                    for (on_origin, on_normal, other_origin, other_normal) in [
                        (a_origin, a_normal, b_origin, b_normal),
                        (b_origin, b_normal, a_origin, a_normal),
                    ] {
                        let on_plane = corner.sub(on_normal.scale(corner.sub(on_origin).dot(on_normal)));
                        height = height.max(on_plane.sub(other_origin).dot(other_normal).abs());
                    }
                }
                let floor = if same_operand { tolerance } else { coincident };
                if height > floor
                    && FRACTION * height < LONGEST_BAND
                    && faces_overlap(a_face, b_face, region)?
                    && (same_operand || !crate::imprint::reads_as_one_surface(&a_face.surface, &b_face.surface, tolerance)?)
                {
                    if env_read!("BREP_DEBUG_FEATURE_SCALE").is_some() {
                        eprintln!(
                            "feature_scale: sliver faces {}/{} height={height:.3e} same_operand={same_operand} region={:?}..{:?}",
                            a_face.id, b_face.id, region.minimum, region.maximum
                        );
                    }
                    thin_pairs.push(ThinPair { height, region, same_operand });
                }
            }
        }
        Ok(Self { thin_pairs })
    }

    /// Whether any feature limits a band (no work at the cap sites when not).
    pub(crate) fn is_empty(&self) -> bool {
        self.thin_pairs.is_empty()
    }

    /// `uncapped`, capped at [`FRACTION`] of every sliver whose region lies
    /// within `uncapped` of `point` (a band that cannot reach a sliver is not
    /// limited by it).
    pub(crate) fn cap(&self, point: Vec3, uncapped: f64) -> f64 {
        self.thin_pairs
            .iter()
            .filter(|pair| pair.region.expanded(uncapped).contains(point))
            .fold(uncapped, |cap, pair| cap.min(FRACTION * pair.height))
    }

    /// [`Self::cap`] by the slabs of one operand's material alone. The
    /// residual merge (B3) takes its cap from these: its identity-proof band
    /// (2·weld·extent, 2e-4 on a 10-unit part) is wider than many a gap
    /// between two operands' near-coplanar faces (1.8e-4 on the revolve pole
    /// fixtures), whose near-tangent junctions it exists to merge, while the
    /// hairline saw's walls are one operand's slab.
    pub(crate) fn cap_by_slabs(&self, point: Vec3, uncapped: f64) -> f64 {
        self.thin_pairs
            .iter()
            .filter(|pair| pair.same_operand && pair.region.expanded(uncapped).contains(point))
            .fold(uncapped, |cap, pair| cap.min(FRACTION * pair.height))
    }

    /// `(region, cap)` for every sliver, for the solid classifier's
    /// coincident-On band.
    pub(crate) fn regions(&self) -> Vec<(Aabb, f64)> {
        self.thin_pairs.iter().map(|pair| (pair.region, FRACTION * pair.height)).collect()
    }

    /// The thinnest sliver, for the resolution floor.
    pub(crate) fn thinnest(&self) -> Option<f64> {
        self.thin_pairs.iter().map(|pair| pair.height).min_by(f64::total_cmp)
    }
}

/// Whether the two faces' trimmed material covers a common area of
/// `region`: some point of a 3×3×3 grid over it, projected onto each face's
/// carrier, lies inside that face's trim. Carrier boxes alone overlap for
/// faces that only stand side by side: fixture `10_regression_step269_cone`
/// carries faces 5e-6 apart in z over different areas (a step, not a slab
/// of material), and treating them as a sliver blocked its near-tangent
/// residual merges.
fn faces_overlap(first: &crate::FaceRecord, second: &crate::FaceRecord, region: Aabb) -> Result<bool, String> {
    use crate::classification::{parameter_point_in_face, PolygonClass};
    let inside = |face: &crate::FaceRecord, point: Vec3| -> Result<bool, String> {
        let projection = crate::project_point_to_surface(&face.surface, point)?;
        Ok(matches!(
            parameter_point_in_face(face, crate::Vec2 { x: projection.u, y: projection.v }, 1e-9)?,
            PolygonClass::Inside
        ))
    };
    let (lo, hi) = (region.minimum, region.maximum);
    for fx in [0.1, 0.5, 0.9] {
        for fy in [0.1, 0.5, 0.9] {
            for fz in [0.1, 0.5, 0.9] {
                let point = Vec3::new(lo.x + (hi.x - lo.x) * fx, lo.y + (hi.y - lo.y) * fy, lo.z + (hi.z - lo.z) * fz);
                if inside(first, point)? && inside(second, point)? {
                    return Ok(true);
                }
            }
        }
    }
    Ok(false)
}

/// An affine carrier's plane: a point on it and its unit normal.
fn plane_of(surface: &NurbsSurface) -> Result<Option<(Vec3, Vec3)>, String> {
    if !surface.is_affine()? {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    Ok(Some((surface.evaluate(u, v)?, surface.normal(u, v)?)))
}

fn corners(region: Aabb) -> [Vec3; 8] {
    let (lo, hi) = (region.minimum, region.maximum);
    [
        Vec3::new(lo.x, lo.y, lo.z),
        Vec3::new(hi.x, lo.y, lo.z),
        Vec3::new(lo.x, hi.y, lo.z),
        Vec3::new(hi.x, hi.y, lo.z),
        Vec3::new(lo.x, lo.y, hi.z),
        Vec3::new(hi.x, lo.y, hi.z),
        Vec3::new(lo.x, hi.y, hi.z),
        Vec3::new(hi.x, hi.y, hi.z),
    ]
}

