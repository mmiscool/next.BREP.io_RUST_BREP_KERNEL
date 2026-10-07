//! Recover fragmented cylinder/cone partitions from their measured carriers.
//! This is a bounded proposal pass; the hybrid builder still validates every
//! shared boundary and demotes unsupported topology in the ordinary way.
use crate::{
    AnalyticSurface, ConeSurface, CylinderSurface, Mesh, MeshAnalysisOptions, RecognitionOptions,
    SamplingMode, SurfaceHint,
};
use brep_kernel::{MeshRegion, MeshSegmentation, RegionCarrier, SegmentOptions, Vec3};

pub(super) fn recover(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
    segmentation: &mut MeshSegmentation,
) {
    // Keep general assemblies on their existing path. No hypotheses or fits
    // are generated from an unbounded collection of possible carriers.
    if segmentation.regions.len() > 32 || segmentation.welded_vertex_count != positions.len() / 3 {
        return;
    }
    let cv = |p: Vec3| crate::Vec3::new(p.x, p.y, p.z);
    let kv = |p: crate::Vec3| Vec3::new(p.x, p.y, p.z);
    let mesh = Mesh::new(
        positions
            .chunks_exact(3)
            .map(|p| crate::Vec3::new(p[0], p[1], p[2]))
            .collect(),
        indices
            .chunks_exact(3)
            .map(|t| [t[0], t[1], t[2]])
            .collect(),
    );
    let Ok(analyzed) = mesh.analyze(&MeshAnalysisOptions::default()) else {
        return;
    };
    let mut candidates = Vec::new();
    for region in &segmentation.regions {
        let surface = match region.carrier {
            RegionCarrier::Cylinder {
                axis_point,
                axis_dir,
                radius,
                ..
            } => AnalyticSurface::Cylinder(CylinderSurface {
                axis_origin: cv(axis_point),
                axis: cv(axis_dir),
                radius,
            }),
            RegionCarrier::Cone {
                apex,
                axis_dir,
                half_angle_rad,
                ..
            } => AnalyticSurface::Cone(ConeSurface {
                apex: cv(apex),
                axis: cv(axis_dir),
                half_angle: half_angle_rad,
            }),
            _ => continue,
        };
        // Bound accuracy by the seed's own scale, not distant geometry in
        // the mesh. Never widen the caller's segmentation acceptance bar.
        let scale = region.bbox_max.sub(region.bbox_min).length();
        let tolerance = (1e-10 + 1e-8 * scale).min(options.fit_tolerance * scale);
        if !tolerance.is_finite() || tolerance <= 0. {
            continue;
        }
        let recognition = RecognitionOptions {
            distance_tolerance: tolerance,
            relative_tolerance: 0.,
            normal_tolerance: options.normal_tolerance_deg.to_radians(),
            sampling: SamplingMode::Vertices,
            ..RecognitionOptions::default()
        };
        // Segment carriers are approximate normal fits. Refine only their own
        // measured support before asking whether neighboring strips agree.
        let ids: Vec<_> = segmentation
            .triangle_region_ids
            .iter()
            .enumerate()
            .filter_map(|(i, &r)| (r == region.id).then_some(i))
            .collect();
        let Ok(fit) = crate::reconstruct_analyzed(
            &analyzed,
            &ids,
            &SurfaceHint::InitialGuess {
                surface,
                trust: crate::MetadataTrust::InitialGuess,
            },
            &recognition,
        ) else {
            continue;
        };
        if fit.surface.surface_type() == surface.surface_type() {
            candidates.push((region.id, ids, fit.surface, recognition));
        }
    }
    let mut proposals: Vec<(Vec<usize>, RegionCarrier, crate::FitMetrics)> = Vec::new();
    for (seed_region, seed_ids, surface, recognition) in candidates {
        let tolerance = recognition.distance_tolerance;
        let ids: Vec<_> = analyzed
            .triangles
            .iter()
            .enumerate()
            .filter_map(|(id, t)| {
                (t.area > 0.
                    && t.vertices
                        .iter()
                        .all(|&v| surface.signed_distance(mesh.vertices[v]).abs() <= tolerance)
                    && surface.normal_at(t.centroid).is_some_and(|n| {
                        n.dot(t.normal).abs() >= recognition.normal_tolerance.cos()
                    }))
                .then_some(id)
            })
            .collect();
        for component in analyzed.connected_components(&ids, false) {
            // Only repair fragmentation: keep complete regions, periodic
            // splits and retry identifiers untouched. The complete seed must
            // survive; recovery cannot discard its inconvenient triangles.
            if !seed_ids
                .iter()
                .all(|id| component.binary_search(id).is_ok())
                || component
                    .iter()
                    .all(|&id| segmentation.triangle_region_ids[id] == seed_region)
                || component.len() < recognition.minimum_support
            {
                continue;
            }
            let result = crate::reconstruct_analyzed(
                &analyzed,
                &component,
                &SurfaceHint::ExactCandidate { surface },
                &recognition,
            );
            let Ok(fit) = result else {
                continue;
            };
            if !fit.diagnostics.exact_parameters_reused || fit.surface != surface {
                continue;
            }
            // Every triangle must agree in sense, including very small slivers;
            // an area-weighted fit alone must not hide an inverted triangle.
            if component.iter().any(|&id| {
                surface
                    .normal_at(analyzed.triangles[id].centroid)
                    .is_none_or(|n| {
                        n.dot(analyzed.triangles[id].normal) * f64::from(fit.orientation) <= 0.
                    })
            }) {
                continue;
            }
            let carrier = match surface {
                AnalyticSurface::Cylinder(c) => RegionCarrier::Cylinder {
                    axis_point: kv(c.axis_origin),
                    axis_dir: kv(c.axis),
                    radius: c.radius,
                    sense: fit.orientation,
                },
                AnalyticSurface::Cone(c) => RegionCarrier::Cone {
                    apex: kv(c.apex),
                    axis_dir: kv(c.axis),
                    half_angle_rad: c.half_angle,
                    sense: fit.orientation,
                },
                _ => unreachable!(),
            };
            // Equivalent seeds for one carrier are one proposal. Merely
            // sharing the same vertex support does not make carriers equal.
            if proposals
                .iter()
                .any(|(ids, old, _)| *ids == component && same_carrier(old, &carrier, tolerance))
            {
                continue;
            }
            proposals.push((component, carrier, fit.metrics));
        }
    }
    // Competing carriers are not evidence for either assignment. Reject the
    // entire proposals when they overlap rather than silently dropping faces.
    let mut counts = vec![0; mesh.triangles.len()];
    for (ids, _, _) in &proposals {
        for &id in ids {
            counts[id] += 1;
        }
    }
    let mut owners = segmentation.triangle_region_ids.clone();
    let mut regions = segmentation.regions.clone();
    for (ids, carrier, metrics) in proposals {
        if ids.iter().any(|&id| counts[id] != 1) {
            continue;
        }
        let id = regions.len() as u32;
        let region = MeshRegion {
            id,
            carrier,
            triangle_count: ids.len(),
            area: metrics.supported_area,
            bbox_min: Vec3::new(0., 0., 0.),
            bbox_max: Vec3::new(0., 0., 0.),
            max_deviation: metrics.max_error,
            rms_deviation: metrics.rms_error,
            max_normal_angle_deg: metrics.max_normal_error.to_degrees(),
        };
        for tid in ids {
            owners[tid] = id;
        }
        regions.push(region);
    }
    if regions.len() == segmentation.regions.len() {
        return;
    }
    let mut compact = Vec::new();
    for mut region in regions {
        let ids: Vec<_> = owners
            .iter()
            .enumerate()
            .filter_map(|(t, &id)| (id == region.id).then_some(t))
            .collect();
        if ids.is_empty() {
            continue;
        }
        region.triangle_count = ids.len();
        region.area = 0.;
        region.bbox_min = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        region.bbox_max = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        let new_id = compact.len() as u32;
        for &t in &ids {
            segmentation.triangle_region_ids[t] = new_id;
            region.area += analyzed.triangles[t].area;
            for &v in &mesh.triangles[t] {
                let p = mesh.vertices[v as usize];
                region.bbox_min = Vec3::new(
                    region.bbox_min.x.min(p.x),
                    region.bbox_min.y.min(p.y),
                    region.bbox_min.z.min(p.z),
                );
                region.bbox_max = Vec3::new(
                    region.bbox_max.x.max(p.x),
                    region.bbox_max.y.max(p.y),
                    region.bbox_max.z.max(p.z),
                );
            }
        }
        region.id = new_id;
        compact.push(region);
    }
    segmentation.regions = compact;
}

fn same_carrier(a: &RegionCarrier, b: &RegionCarrier, tolerance: f64) -> bool {
    match (a, b) {
        (
            RegionCarrier::Cylinder {
                axis_point: p,
                axis_dir: u,
                radius: r,
                sense: s,
            },
            RegionCarrier::Cylinder {
                axis_point: q,
                axis_dir: v,
                radius: t,
                sense: w,
            },
        ) => {
            s == w
                && u.cross(*v).length() <= 1e-10
                && (r - t).abs() <= tolerance
                && p.sub(*q).cross(*u).length() <= tolerance
        }
        (
            RegionCarrier::Cone {
                apex: p,
                axis_dir: u,
                half_angle_rad: r,
                sense: s,
            },
            RegionCarrier::Cone {
                apex: q,
                axis_dir: v,
                half_angle_rad: t,
                sense: w,
            },
        ) => {
            s == w
                && p.sub(*q).length() <= tolerance
                && u.sub(*v).length() <= 1e-10
                && (r - t).abs() <= 1e-10
        }
        _ => false,
    }
}

