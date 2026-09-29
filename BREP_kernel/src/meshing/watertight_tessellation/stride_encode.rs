use super::*;

/// Tessellate only the faces whose global sequential index `i` satisfies
/// `i % stride == offset`, leaving every other face out of the returned mesh.
///
/// This exists so a large solid can be meshed in parallel across several
/// worker WASM instances: dispatch `stride` calls with `offset` 0..stride-1
/// and concatenate the fragments. Two properties make that safe:
///  - Edge samples are recomputed here from the solid and are DETERMINISTIC
///    (interior points from `curve.evaluate`, endpoints pinned to the exact
///    shared vertex-record points), so two workers meshing disjoint face sets
///    produce byte-identical boundary vertices that weld into a watertight
///    whole — the same guarantee the single-call path relies on.
///  - The union over `offset` 0..stride-1 is exactly the set of all faces, so
///    coverage is complete even when the caller's face count is a stale
///    estimate (the caller only needs to pick `stride`, never an exact range).
///
/// Sequential face ids are assigned over ALL faces (not just the meshed ones),
/// so a fragment's `face_ids` match the single-call output for the same solid.
/// Unlike [`tessellate_brep_watertight`] this does NOT validate the mesh — a
/// face subset is not a closed shell; validate after concatenation.
pub fn tessellate_brep_watertight_face_stride(
    solid: &BrepSolid,
    chord_tolerance: f64,
    stride: usize,
    offset: usize,
) -> Result<Mesh, String> {
    if !(chord_tolerance > 0.0) || !chord_tolerance.is_finite() {
        return Err("tessellate_brep_watertight: chord tolerance must be positive".into());
    }
    let started = profile_started();
    let samples = sample_all_edges(solid, chord_tolerance)?;
    let sampling_ms = elapsed_ms(started);
    tess_profile(|p| p.edge_sampling_ms += sampling_ms);
    tessellate_faces_stride(solid, &samples, chord_tolerance, stride, offset)
}

/// As [`tessellate_brep_watertight_face_stride`], but the caller supplies edge
/// samples already computed (and serialized) by [`sample_edges_encoded`]. The
/// worker-pool split uses this so `sample_all_edges` runs ONCE (in the prepare
/// step) instead of redundantly in every face-range worker. The samples are
/// deterministic, so meshing against a shared copy is identical to recomputing
/// them locally.
pub fn tessellate_brep_watertight_face_stride_with_samples(
    solid: &BrepSolid,
    chord_tolerance: f64,
    stride: usize,
    offset: usize,
    encoded_samples: &[f64],
) -> Result<Mesh, String> {
    if !(chord_tolerance > 0.0) || !chord_tolerance.is_finite() {
        return Err("tessellate_brep_watertight: chord tolerance must be positive".into());
    }
    let samples = decode_edge_samples(encoded_samples)?;
    tessellate_faces_stride(solid, &samples, chord_tolerance, stride, offset)
}

/// A `(face_id, &FaceRecord)` that can cross rayon worker threads.
///
/// SAFETY: `FaceRecord` is `!Sync` only because its owned `NurbsSurface`
/// memoizes derived data (analytic carrier, seam closure, projection grid) in
/// `Cell`/`OnceCell` acceleration caches. Those caches live INSIDE each face's
/// own surface (surfaces are stored inline per face, never shared or `Rc`'d),
/// and `tessellate_faces_stride` hands each face to exactly one rayon task, so
/// no cache is ever read or written by two threads at once. The only other
/// shared input is the edge-sample map, which is immutable and free of interior
/// mutability. Under those invariants sharing `&FaceRecord` across threads is
/// sound; if faces ever come to share a surface, revisit this.
#[cfg(feature = "parallel")]
pub(super) struct SyncFace<'a>(u32, &'a FaceRecord);
#[cfg(feature = "parallel")]
unsafe impl Sync for SyncFace<'_> {}

/// Mesh the faces of the stride/offset class against a shared edge-sample map.
///
/// Each face is meshed into its OWN fragment and the fragments are concatenated
/// in face order — bit-identical to folding them one after another into a
/// single mesh (a face's triangle indices are relative to its fragment's own
/// vertex base, which the concat re-bases exactly as the sequential fold did).
/// Because the fragments are independent, the map is a rayon `par_iter` under
/// the `parallel` feature (shared-memory in-process threading), and an ordinary
/// serial map otherwise — the DEFAULT build is unchanged.
pub(super) fn tessellate_faces_stride(
    solid: &BrepSolid,
    samples: &HashMap<u64, EdgeSamples>,
    chord_tolerance: f64,
    stride: usize,
    offset: usize,
) -> Result<Mesh, String> {
    let stride = stride.max(1);
    let offset = offset % stride;
    let mut work: Vec<(u32, &FaceRecord)> = Vec::new();
    let mut sequential_face_id = 0u32;
    for shell in &solid.shells {
        for face in &shell.faces {
            if (sequential_face_id as usize) % stride == offset {
                work.push((sequential_face_id, face));
            }
            sequential_face_id += 1;
        }
    }
    tess_profile(|p| p.faces += work.len() as u64);
    let mesh_one = |face_id: u32, face: &FaceRecord| -> Result<Mesh, String> {
        let mut fragment = Mesh::default();
        tessellate_face_watertight(face, samples, chord_tolerance, face_id, &mut fragment)?;
        Ok(fragment)
    };
    #[cfg(feature = "parallel")]
    let fragments = {
        use rayon::prelude::*;
        let sync_work: Vec<SyncFace> = work.iter().map(|&(id, face)| SyncFace(id, face)).collect();
        sync_work
            .par_iter()
            .map(|face| mesh_one(face.0, face.1))
            .collect::<Result<Vec<Mesh>, String>>()?
    };
    #[cfg(not(feature = "parallel"))]
    let fragments = work
        .iter()
        .map(|&(id, face)| mesh_one(id, face))
        .collect::<Result<Vec<Mesh>, String>>()?;
    Ok(concatenate_meshes(fragments))
}

/// Concatenate mesh fragments in order, re-basing each fragment's triangle
/// indices by the running vertex count (same math as the worker-split weld).
pub(super) fn concatenate_meshes(fragments: Vec<Mesh>) -> Mesh {
    let mut mesh = Mesh::default();
    for fragment in fragments {
        let base = (mesh.positions.len() / 3) as u32;
        mesh.positions.extend_from_slice(&fragment.positions);
        mesh.normals.extend_from_slice(&fragment.normals);
        mesh.indices
            .extend(fragment.indices.iter().map(|index| index + base));
        mesh.face_ids.extend_from_slice(&fragment.face_ids);
    }
    mesh
}

/// Display edge polylines for native in-process consumers (brep-render): every
/// NON-degenerate edge's chord-tolerance samples as `(edge_id, points)`, sorted
/// by edge id so the output is deterministic (the sample map is a HashMap).
/// The samples are the same shared samples the watertight tessellation uses, so
/// the displayed edges lie exactly on the mesh's face boundaries.
pub fn sample_edge_polylines(
    solid: &BrepSolid,
    chord_tolerance: f64,
) -> Result<Vec<(u64, Vec<Vec3>)>, String> {
    if !(chord_tolerance > 0.0) || !chord_tolerance.is_finite() {
        return Err("tessellate_brep_watertight: chord tolerance must be positive".into());
    }
    let samples = sample_all_edges(solid, chord_tolerance)?;
    let degenerate: std::collections::HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| edge.degenerate)
        .map(|edge| edge.id)
        .collect();
    let mut out: Vec<(u64, Vec<Vec3>)> = samples
        .into_iter()
        .filter(|(id, samples)| !degenerate.contains(id) && samples.positions.len() >= 2)
        .map(|(id, samples)| (id, samples.positions))
        .collect();
    out.sort_by_key(|(id, _)| *id);
    Ok(out)
}

/// Compute every edge's shared samples and serialize them into a flat f64
/// buffer for transfer to face-range workers. Layout:
/// `[edge_count, (edge_id, sample_count, frac_0..n, x_0,y_0,z_0, ...), ...]`
/// (`fractions.len() == positions.len()`, so one count per edge). Edge ids ride
/// as f64 exactly as the solid codec already carries them.
pub fn sample_edges_encoded(solid: &BrepSolid, chord_tolerance: f64) -> Result<Vec<f64>, String> {
    if !(chord_tolerance > 0.0) || !chord_tolerance.is_finite() {
        return Err("tessellate_brep_watertight: chord tolerance must be positive".into());
    }
    let samples = sample_all_edges(solid, chord_tolerance)?;
    Ok(encode_edge_samples(&samples))
}

pub(super) fn encode_edge_samples(samples: &HashMap<u64, EdgeSamples>) -> Vec<f64> {
    let mut out = Vec::new();
    out.push(samples.len() as f64);
    for (edge_id, edge) in samples {
        out.push(*edge_id as f64);
        out.push(edge.fractions.len() as f64);
        out.push(edge.worst_sag);
        out.extend_from_slice(&edge.fractions);
        for position in &edge.positions {
            out.push(position.x);
            out.push(position.y);
            out.push(position.z);
        }
    }
    out
}

pub(super) fn decode_edge_samples(data: &[f64]) -> Result<HashMap<u64, EdgeSamples>, String> {
    let mut cursor = 0usize;
    let mut take = |count: usize| -> Result<&[f64], String> {
        let end = cursor
            .checked_add(count)
            .filter(|end| *end <= data.len())
            .ok_or("watertight tessellation: truncated edge-sample buffer")?;
        let slice = &data[cursor..end];
        cursor = end;
        Ok(slice)
    };
    let edge_count = take(1)?[0] as usize;
    let mut samples = HashMap::with_capacity(edge_count);
    for _ in 0..edge_count {
        let header = take(3)?;
        let edge_id = header[0] as u64;
        let sample_count = header[1] as usize;
        let worst_sag = header[2];
        let fractions = take(sample_count)?.to_vec();
        let position_values = take(sample_count * 3)?;
        let positions = position_values
            .chunks_exact(3)
            .map(|chunk| Vec3::new(chunk[0], chunk[1], chunk[2]))
            .collect();
        samples.insert(
            edge_id,
            EdgeSamples {
                fractions,
                positions,
                worst_sag,
            },
        );
    }
    Ok(samples)
}
