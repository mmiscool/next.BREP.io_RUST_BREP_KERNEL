//! Recover analytic intersections lost when a triangle-mesh boolean cuts
//! spherical facets. Cut vertices lie inside their parent facet's sphere;
//! fitting those vertices, or treating the cut strip as a new surface, loses
//! the sphere. Use precise, independently segmented carriers as hypotheses
//! and prove their union against the entire closed source mesh before CSG.
use crate::{
    boolean_operation, make_box_brep, make_sphere_brep, transform_brep, AffineTransform,
    BooleanOperation, BooleanOptions, BrepSolid, MeshSegmentation, RegionCarrier, SegmentOptions,
    Vec3,
};

#[derive(Clone)]
struct Sphere {
    center: Vec3,
    radius: f64,
    // Measured chordal deflection of intact facets, not the user's fit bar.
    sag: f64,
}

struct BoxCarrier {
    axes: [Vec3; 3],
    low: [f64; 3],
    high: [f64; 3],
}

impl BoxCarrier {
    fn distance(&self, p: Vec3) -> f64 {
        (0..3)
            .map(|i| {
                let h = p.dot(self.axes[i]);
                (self.low[i] - h).max(h - self.high[i])
            })
            .fold(f64::NEG_INFINITY, f64::max)
    }

    fn point(&self, coordinates: [f64; 3]) -> Vec3 {
        (0..3).fold(Vec3::default(), |p, i| {
            p.add(self.axes[i].scale(coordinates[i]))
        })
    }

    fn solid(&self) -> Result<BrepSolid, String> {
        let local = make_box_brep(
            Vec3::default(),
            self.high[0] - self.low[0],
            self.high[1] - self.low[1],
            self.high[2] - self.low[2],
        )?;
        let origin = self.point(self.low);
        let [x, y, z] = self.axes;
        let transform = AffineTransform::new([
            x.x, y.x, z.x, origin.x, x.y, y.y, z.y, origin.y, x.z, y.z, z.z, origin.z, 0., 0., 0.,
            1.,
        ])?;
        transform_brep(&local, transform, transform.determinant3() < 0.)
    }
}

fn box_carrier(seg: &MeshSegmentation, precision: f64) -> Option<Option<BoxCarrier>> {
    let planes: Vec<_> = seg
        .regions
        .iter()
        .filter_map(|r| match r.carrier {
            RegionCarrier::Plane { origin, normal } => Some((origin, normal)),
            _ => None,
        })
        .collect();
    if planes.is_empty() {
        return Some(None);
    }
    if planes.len() != 6 {
        return None;
    }
    let mut axes = Vec::<Vec3>::new();
    let mut ranges = Vec::<[f64; 2]>::new();
    let mut counts = Vec::<[usize; 2]>::new();
    for (origin, normal) in planes {
        let index = axes
            .iter()
            .position(|axis| axis.cross(normal).length() < 1e-6);
        let i = match index {
            Some(i) => i,
            None => {
                if axes.len() == 3 || axes.iter().any(|axis| axis.dot(normal).abs() > 1e-6) {
                    return None;
                }
                axes.push(normal.normalized().ok()?);
                ranges.push([f64::NEG_INFINITY, f64::INFINITY]);
                counts.push([0, 0]);
                axes.len() - 1
            }
        };
        let side = usize::from(axes[i].dot(normal) > 0.);
        ranges[i][side] = origin.dot(axes[i]);
        counts[i][side] += 1;
    }
    if axes.len() != 3
        || counts.iter().any(|c| *c != [1, 1])
        || ranges.iter().any(|r| r[1] - r[0] <= precision)
    {
        return None;
    }
    Some(Some(BoxCarrier {
        axes: [axes[0], axes[1], axes[2]],
        low: [ranges[0][0], ranges[1][0], ranges[2][0]],
        high: [ranges[0][1], ranges[1][1], ranges[2][1]],
    }))
}

fn sphere_distance(sphere: &Sphere, p: Vec3) -> f64 {
    p.sub(sphere.center).length() - sphere.radius
}

fn union_distance(spheres: &[Sphere], cube: Option<&BoxCarrier>, p: Vec3) -> f64 {
    spheres
        .iter()
        .map(|s| sphere_distance(s, p))
        .fold(cube.map_or(f64::INFINITY, |b| b.distance(p)), f64::min)
}

/// Attempt a small union of outward spheres and, optionally, one orthogonal
/// box. Returns `None` for unsupported geometry or insufficient evidence.
///
/// The caller supplies a consistently wound closed component and its kernel
/// segmentation. This does not relax segmentation tolerances: carriers need
/// near-precision support, all source facets must agree with the exposed
/// union boundary, and that boundary must also be covered by the source.
/// The distance allowance for cut vertices is bounded by the measured
/// chordal deflection of intact sphere facets (at most 2% of the radius).
pub fn reconstruct_spherical_mesh_union(
    positions: &[f64],
    indices: &[u32],
    seg: &MeshSegmentation,
    options: &SegmentOptions,
) -> Option<BrepSolid> {
    let sequential;
    let indices = if indices.is_empty() {
        if positions.len() % 9 != 0 || positions.len() / 9 > 10_000 {
            return None;
        }
        sequential = (0..positions.len() as u32 / 3).collect::<Vec<_>>();
        &sequential
    } else {
        indices
    };
    // Keep this bounded recovery out of general assemblies and noisy scans.
    if !options.normal_tolerance_deg.is_finite()
        || !(0. ..90.).contains(&options.normal_tolerance_deg)
        || positions.len() % 3 != 0
        || indices.len() % 3 != 0
        || indices.len() / 3 > 10_000
        || seg.regions.len() > 16
        || seg.triangle_region_ids.len() != indices.len() / 3
    {
        return None;
    }
    let vertices: Vec<_> = positions
        .chunks_exact(3)
        .map(|p| Vec3::new(p[0], p[1], p[2]))
        .collect();
    if vertices
        .iter()
        .any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite())
    {
        return None;
    }
    let triangles: Vec<[Vec3; 3]> = indices
        .chunks_exact(3)
        .map(|t| {
            Some([
                *vertices.get(t[0] as usize)?,
                *vertices.get(t[1] as usize)?,
                *vertices.get(t[2] as usize)?,
            ])
        })
        .collect::<Option<_>>()?;
    // Validate closed, opposite edge incidence here too: this API must never
    // fill a missing mesh patch merely because its vertices fit a sphere.
    let mut lo = vertices.first().copied()?;
    let mut hi = lo;
    for p in &vertices {
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    let weld_tolerance = if options.weld_tolerance > 0. && options.weld_tolerance.is_finite() {
        options.weld_tolerance
    } else {
        (hi.sub(lo).length() * 1e-6).max(1e-12)
    };
    let mut welded = Vec::new();
    let mut buckets = rustc_hash::FxHashMap::default();
    let vertex_ids: Vec<_> = vertices
        .iter()
        .map(|&p| {
            crate::mesh_weld::weld_vertex(p, weld_tolerance, &mut welded, &mut buckets) as u32
        })
        .collect();
    let mut edges = rustc_hash::FxHashMap::<(u32, u32), (usize, i32)>::default();
    for t in indices.chunks_exact(3) {
        for i in 0..3 {
            let (a, b) = (
                vertex_ids[t[i] as usize],
                vertex_ids[t[(i + 1) % 3] as usize],
            );
            if a == b {
                return None;
            }
            let entry = edges.entry((a.min(b), a.max(b))).or_default();
            entry.0 += 1;
            entry.1 += if a < b { 1 } else { -1 };
        }
    }
    if edges
        .values()
        .any(|&(count, sense)| count != 2 || sense != 0)
    {
        return None;
    }
    let mut spheres = Vec::new();
    for region in &seg.regions {
        match region.carrier {
            RegionCarrier::Sphere {
                center,
                radius,
                sense: 1,
            } => {
                let precision = radius * 2e-6;
                if !radius.is_finite()
                    || radius <= 0.
                    || region.triangle_count < 24
                    || !region.max_deviation.is_finite()
                    || region.max_deviation > precision
                {
                    return None;
                }
                let mut sag = 0_f64;
                for (t, &owner) in triangles.iter().zip(&seg.triangle_region_ids) {
                    if owner != region.id {
                        continue;
                    }
                    let normal = t[1].sub(t[0]).cross(t[2].sub(t[0])).normalized().ok()?;
                    let height = t[0].sub(center).dot(normal);
                    if height <= 0. {
                        return None;
                    }
                    sag = sag.max(radius - height);
                }
                if sag <= 0. || sag > 0.02 * radius {
                    return None;
                }
                spheres.push(Sphere {
                    center,
                    radius,
                    sag: sag + precision,
                });
            }
            RegionCarrier::Plane { .. } | RegionCarrier::Freeform => {}
            _ => return None,
        }
    }
    if spheres.is_empty() || spheres.len() > 2 {
        return None;
    }
    let precision = spheres
        .iter()
        .map(|s| s.radius * 2e-6)
        .fold(0_f64, f64::max);
    let cube = box_carrier(seg, precision)?;
    if spheres.len() == 1 && cube.is_none() {
        return None;
    }
    let sag = spheres.iter().map(|s| s.sag).fold(0_f64, f64::max);
    let cosine = options.normal_tolerance_deg.to_radians().cos();
    for t in &triangles {
        let normal = t[1].sub(t[0]).cross(t[2].sub(t[0])).normalized().ok()?;
        let centroid = t[0].add(t[1]).add(t[2]).scale(1. / 3.);
        let samples = [
            t[0],
            t[1],
            t[2],
            centroid,
            t[0].add(t[1]).scale(0.5),
            t[1].add(t[2]).scale(0.5),
            t[2].add(t[0]).scale(0.5),
        ];
        if samples.iter().any(|&p| {
            let d = union_distance(&spheres, cube.as_ref(), p);
            !d.is_finite() || d < -sag || d > precision
        }) {
            return None;
        }
        let spherical = spheres.iter().any(|s| {
            samples
                .iter()
                .all(|&p| sphere_distance(s, p).abs() <= s.sag)
                && centroid
                    .sub(s.center)
                    .normalized()
                    .is_ok_and(|n| n.dot(normal) >= cosine)
        });
        let planar = cube.as_ref().is_some_and(|b| {
            (0..3).any(|i| {
                [(-1., b.low[i]), (1., b.high[i])].iter().any(|&(sign, h)| {
                    normal.dot(b.axes[i].scale(sign)) >= cosine
                        && samples
                            .iter()
                            .all(|p| (p.dot(b.axes[i]) - h).abs() <= precision)
                })
            })
        });
        if !spherical && !planar {
            return None;
        }
    }
    // Check the reverse direction as well. Source-to-carrier distance alone
    // would admit an incomplete or differently trimmed primitive arrangement.
    let covered = |p: Vec3| -> bool {
        union_distance(&spheres, cube.as_ref(), p) < -precision
            || triangles
                .iter()
                .any(|t| point_triangle_distance(p, *t) <= sag)
    };
    for s in &spheres {
        // Equal-area deterministic samples, including all Cartesian extrema.
        for i in 0..2048 {
            let z = 1. - 2. * (i as f64 + 0.5) / 2048.;
            let angle = i as f64 * std::f64::consts::PI * (3. - 5_f64.sqrt());
            let r = (1. - z * z).sqrt();
            if !covered(
                s.center
                    .add(Vec3::new(r * angle.cos(), r * angle.sin(), z).scale(s.radius)),
            ) {
                return None;
            }
        }
        for axis in [
            Vec3::new(1., 0., 0.),
            Vec3::new(0., 1., 0.),
            Vec3::new(0., 0., 1.),
        ] {
            for sign in [-1., 1.] {
                if !covered(s.center.add(axis.scale(sign * s.radius))) {
                    return None;
                }
            }
        }
    }
    if let Some(b) = &cube {
        for i in 0..3 {
            for h in [b.low[i], b.high[i]] {
                for u in 0..=16 {
                    for v in 0..=16 {
                        let mut coordinates = b.low;
                        coordinates[i] = h;
                        for (j, fraction) in [((i + 1) % 3, u), ((i + 2) % 3, v)] {
                            coordinates[j] += (b.high[j] - b.low[j]) * fraction as f64 / 16.;
                        }
                        if !covered(b.point(coordinates)) {
                            return None;
                        }
                    }
                }
            }
        }
    }
    let mut primitives = Vec::new();
    if let Some(b) = cube {
        primitives.push(b.solid().ok()?);
    }
    for s in spheres {
        primitives.push(make_sphere_brep(s.center, s.radius, Vec3::new(0., 0., 1.)).ok()?);
    }
    let mut result = primitives.remove(0);
    let boolean_options = BooleanOptions {
        tolerance: (precision * 0.05).max(1e-12),
        ..BooleanOptions::default()
    };
    for primitive in primitives {
        result = boolean_operation(
            &result,
            &primitive,
            BooleanOperation::Union,
            &boolean_options,
        )
        .ok()?;
    }
    result.validate().is_empty().then_some(result)
}

fn point_triangle_distance(p: Vec3, [a, b, c]: [Vec3; 3]) -> f64 {
    let ab = b.sub(a);
    let ac = c.sub(a);
    let ap = p.sub(a);
    let n = ab.cross(ac);
    let nn = n.length_squared();
    if nn > 0. {
        let projection = p.sub(n.scale(ap.dot(n) / nn));
        if [(a, b), (b, c), (c, a)]
            .iter()
            .all(|&(x, y)| y.sub(x).cross(projection.sub(x)).dot(n) >= -1e-14 * nn)
        {
            return ap.dot(n).abs() / nn.sqrt();
        }
    }
    [(a, b), (b, c), (c, a)]
        .iter()
        .map(|&(x, y)| {
            let edge = y.sub(x);
            let t = (p.sub(x).dot(edge) / edge.length_squared()).clamp(0., 1.);
            p.sub(x.add(edge.scale(t))).length()
        })
        .fold(f64::INFINITY, f64::min)
}
