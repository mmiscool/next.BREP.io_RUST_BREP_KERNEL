pub(super) use crate::mesh_weld::edge_key;
use crate::mesh_weld::weld_vertex;
use super::*;

// ---------------------------------------------------------------------------
// Internal mesh representation
// ---------------------------------------------------------------------------

/// Triangles below this fraction of the mesh's mean triangle area are
/// excluded from region growing and fitting (their normals are numerical
/// noise — e.g. pole slivers of a tessellated sphere) and adopted into an
/// adjacent region afterwards, like fully degenerate triangles.
const MICRO_AREA_FRACTION: f64 = 1e-3;

pub(super) struct TriData {
    pub(super) verts: [usize; 3],
    pub(super) normal: Vec3,
    pub(super) area: f64,
    pub(super) centroid: Vec3,
    /// Degenerate or micro-area triangle: excluded from growing/fitting.
    pub(super) skip: bool,
}

pub(super) struct MeshData {
    pub(super) verts: Vec<Vec3>,
    pub(super) tris: Vec<TriData>,
    pub(super) diag: f64,
    /// Welded (min,max) vertex pair → incident triangle ids.
    pub(super) edges: HashMap<(usize, usize), Vec<u32>>,
}

pub(super) fn build_mesh_data(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
) -> Result<MeshData, String> {
    if positions.is_empty() || positions.len() % 3 != 0 {
        return Err("segment_mesh_faces: positions must be xyz triples".into());
    }
    if positions.iter().any(|value| !value.is_finite()) {
        return Err("segment_mesh_faces: non-finite vertex coordinate".into());
    }
    let point_count = positions.len() / 3;
    let sequential;
    let tri_indices: &[u32] = if indices.is_empty() {
        if point_count % 3 != 0 {
            return Err("segment_mesh_faces: soup positions must form whole triangles".into());
        }
        sequential = (0..point_count as u32).collect::<Vec<_>>();
        &sequential
    } else {
        if indices.len() % 3 != 0 {
            return Err("segment_mesh_faces: index buffer is not triangular".into());
        }
        if indices.iter().any(|index| *index as usize >= point_count) {
            return Err("segment_mesh_faces: index outside position buffer".into());
        }
        indices
    };

    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for point in positions.chunks_exact(3) {
        for axis in 0..3 {
            low[axis] = low[axis].min(point[axis]);
            high[axis] = high[axis].max(point[axis]);
        }
    }
    let diag = (0..3)
        .map(|axis| (high[axis] - low[axis]).powi(2))
        .sum::<f64>()
        .sqrt();
    let weld_tolerance = if options.weld_tolerance > 0.0 && options.weld_tolerance.is_finite() {
        options.weld_tolerance
    } else {
        (diag * 1e-6).max(1e-12)
    };

    let mut verts = Vec::new();
    let mut buckets = HashMap::default();
    let mut point_to_welded = Vec::with_capacity(point_count);
    for point in positions.chunks_exact(3) {
        point_to_welded.push(weld_vertex(
            Vec3::new(point[0], point[1], point[2]),
            weld_tolerance,
            &mut verts,
            &mut buckets,
        ));
    }

    let degenerate_cross = (1e-12 * diag.max(1e-12)).powi(2);
    let mut tris = Vec::with_capacity(tri_indices.len() / 3);
    let mut edges: HashMap<(usize, usize), Vec<u32>> = HashMap::default();
    for triangle in tri_indices.chunks_exact(3) {
        let ids = [
            point_to_welded[triangle[0] as usize],
            point_to_welded[triangle[1] as usize],
            point_to_welded[triangle[2] as usize],
        ];
        let tri_id = tris.len() as u32;
        let a = verts[ids[0]];
        let b = verts[ids[1]];
        let c = verts[ids[2]];
        let cross = b.sub(a).cross(c.sub(a));
        let cross_len = cross.length();
        let repeated = ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2];
        let degenerate = repeated || cross_len <= degenerate_cross;
        let normal = if degenerate {
            Vec3::default()
        } else {
            cross.scale(1.0 / cross_len)
        };
        tris.push(TriData {
            verts: ids,
            normal,
            area: 0.5 * cross_len,
            centroid: a.add(b).add(c).scale(1.0 / 3.0),
            skip: degenerate,
        });
        for corner in 0..3 {
            let first = ids[corner];
            let second = ids[(corner + 1) % 3];
            if first == second {
                continue;
            }
            edges
                .entry(edge_key(first, second))
                .or_default()
                .push(tri_id);
        }
    }
    if tris.is_empty() {
        return Err("segment_mesh_faces: no triangles".into());
    }
    // Mark micro-area slivers for skipping (their normals are noise).
    let mean_area = tris.iter().map(|t| t.area).sum::<f64>() / tris.len() as f64;
    let micro_floor = MICRO_AREA_FRACTION * mean_area;
    for tri in &mut tris {
        if tri.area < micro_floor {
            tri.skip = true;
        }
    }
    if tris.iter().all(|t| t.skip) {
        return Err("segment_mesh_faces: all triangles are degenerate".into());
    }
    Ok(MeshData {
        verts,
        tris,
        diag,
        edges,
    })
}
