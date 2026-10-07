//! Restore topological coverage of numerically degenerate triangles without
//! allowing those triangles to vote in an analytic fit.
use super::*;

pub(super) fn complete(
    mesh: &Mesh,
    options: &RecognitionOptions,
    regions: &mut [SurfaceRegion],
) -> Result<usize, StlConversionError> {
    let mut owner = vec![None; mesh.triangles.len()];
    for (region, data) in regions.iter().enumerate() {
        for &triangle in &data.triangle_indices {
            if triangle >= owner.len() || owner[triangle].replace(region).is_some() {
                return Err(StlConversionError("invalid recognition partition".into()));
            }
        }
    }
    if owner.iter().all(Option::is_some) {
        return Ok(0);
    }
    let analyzed = mesh
        .analyze(&MeshAnalysisOptions {
            feature_angle: options.feature_angle,
            ..MeshAnalysisOptions::default()
        })
        .map_err(|e| StlConversionError(format!("degenerate coverage analysis failed: {e}")))?;
    if analyzed.degenerate_triangles.is_empty() {
        return Ok(0);
    }
    let mut edges = BTreeMap::<(u32, u32), Vec<usize>>::new();
    for (t, triangle) in mesh.triangles.iter().enumerate() {
        for side in 0..3 {
            let (a, b) = (triangle[side], triangle[(side + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let mut completed = 0;
    // Do not propagate through an unrecognized chain: all three neighbors
    // must already belong to the same independently fitted region.
    for &t in &analyzed.degenerate_triangles {
        if owner[t].is_some() {
            continue;
        }
        let triangle = mesh.triangles[t];
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0] {
            continue;
        }
        let neighbors = (0..3)
            .map(|side| {
                let (a, b) = (triangle[side], triangle[(side + 1) % 3]);
                let uses = &edges[&(a.min(b), a.max(b))];
                if uses.len() != 2 {
                    return None;
                }
                uses.iter()
                    .find(|&&other| other != t)
                    .and_then(|&other| owner[other])
            })
            .collect::<Vec<_>>();
        let Some(region) = neighbors[0] else {
            continue;
        };
        if neighbors.iter().any(|r| *r != Some(region)) {
            continue;
        }
        let surface = regions[region].surface;
        // The absolute fitting tolerance already includes measured source
        // precision. Require every vertex, not just a centroid, on the carrier.
        if triangle.iter().any(|&v| {
            surface.signed_distance(mesh.vertices[v as usize]).abs() > options.distance_tolerance
        }) {
            continue;
        }
        regions[region].triangle_indices.push(t);
        completed += 1;
    }
    for region in regions {
        region.triangle_indices.sort_unstable();
    }
    Ok(completed)
}

