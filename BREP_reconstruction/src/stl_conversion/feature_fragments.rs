//! Rejoin tiny tessellation feature fragments only to an unchanged, already
//! fitted carrier that surrounds them. Generic region discovery retains its
//! feature-edge policy; this is a topology-aware whole-primitive completion.
use super::*;

pub(super) fn complete(
    mesh: &Mesh,
    options: &RecognitionOptions,
    regions: &mut [SurfaceRegion],
    unresolved: &mut Vec<usize>,
) -> Result<usize, StlConversionError> {
    if unresolved.is_empty() || regions.is_empty() {
        return Ok(0);
    }
    let analyzed = mesh
        .analyze(&MeshAnalysisOptions {
            feature_angle: options.feature_angle,
            ..Default::default()
        })
        .map_err(|e| StlConversionError(format!("feature fragment analysis failed: {e}")))?;
    let mut owner = vec![None; mesh.triangles.len()];
    for (r, region) in regions.iter().enumerate() {
        for &t in &region.triangle_indices {
            owner[t] = Some(r);
        }
    }
    let fragments = analyzed.connected_components(unresolved, false);
    let mut accepted = BTreeSet::new();
    for fragment in fragments {
        if fragment.len() >= options.minimum_support {
            continue;
        }
        let members = fragment.iter().copied().collect::<BTreeSet<_>>();
        let mut surrounding = BTreeSet::new();
        let mut closed = true;
        for &t in &fragment {
            for neighbor in analyzed.triangles[t].neighbors {
                match neighbor {
                    Some(n) if members.contains(&n) => {}
                    Some(n) => match owner[n] {
                        Some(r) => {
                            surrounding.insert(r);
                        }
                        None => closed = false,
                    },
                    None => closed = false,
                }
            }
        }
        if !closed || surrounding.len() != 1 {
            continue;
        }
        let r = *surrounding.first().unwrap();
        let surface = regions[r].surface;
        let exact = 512.0 * f64::EPSILON * analyzed.diagonal.max(1.0);
        if fragment.iter().any(|&t| {
            let triangle = &analyzed.triangles[t];
            triangle
                .vertices
                .iter()
                .any(|&v| surface.signed_distance(mesh.vertices[v]).abs() > exact)
                || surface.normal_at(triangle.centroid).is_none_or(|n| {
                    n.dot(triangle.normal) * f64::from(regions[r].orientation) <= 0.0
                })
        }) {
            continue;
        }
        let mut ids = regions[r].triangle_indices.clone();
        ids.extend(&fragment);
        ids.sort_unstable();
        let Ok(fit) = crate::reconstruct_analyzed(
            &analyzed,
            &ids,
            &SurfaceHint::ExactCandidate { surface },
            options,
        ) else {
            continue;
        };
        if !fit.diagnostics.exact_parameters_reused
            || fit.surface != surface
            || fit.orientation != regions[r].orientation
        {
            continue;
        }
        // Preserve recognition provenance. The carrier is unchanged; only its
        // support, residuals and confidence have been revalidated together.
        regions[r].triangle_indices = ids;
        regions[r].metrics = fit.metrics;
        regions[r].confidence = fit.confidence;
        regions[r]
            .diagnostics
            .reason
            .push_str("; surrounded feature fragment validated on unchanged carrier");
        accepted.extend(fragment);
    }
    unresolved.retain(|t| !accepted.contains(t));
    Ok(accepted.len())
}

