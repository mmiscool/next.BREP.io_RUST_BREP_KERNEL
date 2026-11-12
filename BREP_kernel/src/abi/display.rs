use super::*;

/// Everything the native renderer needs to display one resident solid.
pub struct DisplaySolidPayload {
    /// Faces in shell/face order — the index into this Vec IS the watertight
    /// mesh's `face_ids` value for that face's triangles.
    pub faces: Vec<(u64, Option<String>)>,
    /// Watertight display mesh (chord-tolerance driven).
    pub mesh: Mesh,
    /// Non-degenerate edges `(edge_id, name, polyline)`, sorted by edge id.
    pub edges: Vec<(u64, Option<String>, Vec<Vec3>)>,
    /// Topology vertices `(vertex_id, point)`.
    pub vertices: Vec<(u64, Vec3)>,
    /// The chord tolerance actually used.
    pub chord_tolerance: f64,
}

/// The app's display chord tolerance (`BetterSolid._kernelTessellationOptions`):
/// vertex |coord| extent — falling back to the control-point hull / sqrt(2) for
/// vertex-free solids (full spheres/tori) — times 1.5e-3, times the render-LOD
/// factor (1.0 = the app's "Normal" preset).
pub fn display_chord_tolerance(solid: &BrepSolid, lod_factor: f64) -> f64 {
    let mut extent = 0.0f64;
    for vertex in &solid.vertices {
        extent = extent
            .max(vertex.point.x.abs())
            .max(vertex.point.y.abs())
            .max(vertex.point.z.abs());
    }
    if extent <= 0.0 {
        let mut control_point_extent = 0.0f64;
        for shell in &solid.shells {
            for face in &shell.faces {
                for row in &face.surface.control_points {
                    for cp in row {
                        let w = if cp.w != 0.0 { cp.w } else { 1.0 };
                        control_point_extent = control_point_extent
                            .max((cp.x / w).abs())
                            .max((cp.y / w).abs())
                            .max((cp.z / w).abs());
                    }
                }
            }
        }
        extent = control_point_extent / std::f64::consts::SQRT_2;
    }
    extent.max(1e-9) * 1.5e-3 * lod_factor
}

/// Native display payload for a resident solid: watertight mesh + face list (in
/// mesh `face_ids` order) + edge polylines + vertices, all in one registry
/// borrow. `lod_factor` scales the per-solid display chord tolerance (1.0 = the
/// app's "Normal" preset; higher = coarser mesh). Callers pass a sanitized,
/// finite, positive value — [`display_chord_tolerance`] multiplies it in directly.
pub fn display_payload_handle_native(
    handle: u32,
    lod_factor: f64,
) -> Result<DisplaySolidPayload, String> {
    with_registered_solid_str(handle, |solid| display_payload_of_solid(solid, lod_factor))
}

/// [`display_payload_handle_native`] for many resident solids at once, in
/// `handles` order.
///
/// **Instances are meshed once.** Solids that are rigid copies of one another
/// (an imported assembly places each part many times: 857 instances of 174
/// parts) share one tessellation: the copy with the finest display chord is
/// meshed, and each other copy gets that mesh and its edge polylines carried
/// by the rigid map between them, with its OWN face/edge ids and names and its
/// own vertices, so picking, selection and per-instance colour see the solid
/// they always saw. A copy is only drawn this way when everything but point
/// positions is identical (ids, senses, knots, weights, pcurves) and one rigid
/// map carries every vertex and control point of the meshed copy onto its own
/// (`congruent_pose`); anything else, and every solid with no copy, gets the
/// single call's payload exactly.
///
/// Under the `parallel` feature the meshed solids are copied out of this
/// thread's registry (it is thread-local) a chunk at a time and meshed across
/// rayon's pool. Display meshing opens no thread-local mode (the soundness
/// scan's mode is closed on every worker), so a payload does not depend on
/// the thread that computed it.
pub fn display_payloads_handles_native(
    handles: &[u32],
    lod_factor: f64,
) -> Vec<Result<DisplaySolidPayload, String>> {
    // Group by everything a rigid move leaves alone; the first, finest-chord
    // member of a group is meshed, the rest are posed when the map verifies.
    let keys: Vec<Option<(u64, f64)>> = handles
        .iter()
        .map(|&handle| {
            with_registered_solid_str(handle, |solid| {
                Ok((congruence_signature(solid), display_chord_tolerance(solid, lod_factor)))
            })
            .ok()
        })
        .collect();
    let mut base_of_signature: std::collections::HashMap<u64, usize> = std::collections::HashMap::new();
    for (index, key) in keys.iter().enumerate() {
        let Some((signature, chord)) = *key else { continue };
        let base = base_of_signature.entry(signature).or_insert(index);
        if chord < keys[*base].map_or(f64::INFINITY, |(_, base_chord)| base_chord) {
            *base = index;
        }
    }
    let mut posed: Vec<Option<(usize, RigidPose)>> = vec![None; handles.len()];
    for (index, key) in keys.iter().enumerate() {
        let Some((signature, _)) = *key else { continue };
        let base = base_of_signature[&signature];
        if base == index {
            continue;
        }
        posed[index] = with_registered_solid_str(handles[base], |base_solid| {
            with_registered_solid_str(handles[index], |solid| Ok(congruent_pose(base_solid, solid)))
        })
        .ok()
        .flatten()
        .map(|pose| (base, pose));
    }
    // Mesh every solid that is not posed (bases, unmatched copies, failures).
    let meshed_indices: Vec<usize> = (0..handles.len()).filter(|&index| posed[index].is_none()).collect();
    let meshed_handles: Vec<u32> = meshed_indices.iter().map(|&index| handles[index]).collect();
    // BREP_PROFILE: how much of the batch was meshed and how much posed.
    let profile_started = crate::profile::profile_enabled().then(web_time::Instant::now);
    let mut payloads: Vec<Option<Result<DisplaySolidPayload, String>>> = (0..handles.len()).map(|_| None).collect();
    for (index, payload) in meshed_indices.iter().zip(mesh_display_payloads(&meshed_handles, lod_factor)) {
        payloads[*index] = Some(payload);
    }
    let mesh_ms = profile_started.map(|started| started.elapsed().as_secs_f64() * 1_000.0);
    // Pose the copies; a copy whose base failed to mesh is meshed on its own.
    for index in 0..handles.len() {
        let Some((base, pose)) = posed[index].take() else { continue };
        let result = match &payloads[base] {
            Some(Ok(base_payload)) => {
                with_registered_solid_str(handles[index], |solid| Ok(posed_payload(base_payload, &pose, solid)))
            }
            _ => display_payload_handle_native(handles[index], lod_factor),
        };
        payloads[index] = Some(result);
    }
    if let Some(started) = profile_started {
        profile_line!(
            "display.batch solids={} meshed={} posed={} mesh_ms={:.1} total_ms={:.1}",
            handles.len(),
            meshed_indices.len(),
            handles.len() - meshed_indices.len(),
            mesh_ms.unwrap_or(0.0),
            started.elapsed().as_secs_f64() * 1_000.0
        );
    }
    payloads.into_iter().map(|payload| payload.expect("every slot meshed or posed")).collect()
}

/// Mesh each handle's solid on its own (across the pool under `parallel`).
fn mesh_display_payloads(handles: &[u32], lod_factor: f64) -> Vec<Result<DisplaySolidPayload, String>> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        // Bounded copies: a chunk of solids is resident twice at most.
        const CHUNK: usize = 256;
        let mut payloads = Vec::with_capacity(handles.len());
        for chunk in handles.chunks(CHUNK) {
            let solids: Vec<Result<BrepSolid, String>> =
                chunk.iter().map(|&handle| registered_solid_clone(handle)).collect();
            payloads.extend(
                solids
                    .into_par_iter()
                    .with_max_len(1)
                    .map(|solid| solid.and_then(|solid| display_payload_of_solid(&solid, lod_factor)))
                    .collect::<Vec<_>>(),
            );
        }
        payloads
    }
    #[cfg(not(feature = "parallel"))]
    {
        handles.iter().map(|&handle| display_payload_handle_native(handle, lod_factor)).collect()
    }
}

/// A proper rigid map `p -> rotation * p + translation` (rows of `rotation`).
#[derive(Clone, Copy, Debug)]
struct RigidPose {
    rotation: [[f64; 3]; 3],
    translation: [f64; 3],
}

impl RigidPose {
    fn point(&self, p: Vec3) -> Vec3 {
        let r = &self.rotation;
        Vec3::new(
            r[0][0] * p.x + r[0][1] * p.y + r[0][2] * p.z + self.translation[0],
            r[1][0] * p.x + r[1][1] * p.y + r[1][2] * p.z + self.translation[1],
            r[2][0] * p.x + r[2][1] * p.y + r[2][2] * p.z + self.translation[2],
        )
    }

    fn direction(&self, d: Vec3) -> Vec3 {
        let r = &self.rotation;
        Vec3::new(
            r[0][0] * d.x + r[0][1] * d.y + r[0][2] * d.z,
            r[1][0] * d.x + r[1][1] * d.y + r[1][2] * d.z,
            r[2][0] * d.x + r[2][1] * d.y + r[2][2] * d.z,
        )
    }
}

/// A hash of everything a rigid move leaves unchanged: ids, topology, senses,
/// degrees, knots, weights, parameter ranges and pcurves. Names are left out
/// (instances are named apart). Equal signatures only nominate a pair;
/// [`congruent_pose`] decides.
fn congruence_signature(solid: &BrepSolid) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = rustc_hash::FxHasher::default();
    std::mem::discriminant(&solid.kind).hash(&mut h);
    solid.genus.hash(&mut h);
    solid.cells.len().hash(&mut h);
    solid.vertices.len().hash(&mut h);
    for vertex in &solid.vertices {
        vertex.id.hash(&mut h);
    }
    solid.edges.len().hash(&mut h);
    for edge in &solid.edges {
        (edge.id, edge.start_vertex_id, edge.end_vertex_id, edge.degenerate).hash(&mut h);
        (edge.t0.to_bits(), edge.t1.to_bits(), edge.curve.degree).hash(&mut h);
        edge.curve.knots.iter().for_each(|k| k.to_bits().hash(&mut h));
        edge.curve.control_points.iter().for_each(|c| c.w.to_bits().hash(&mut h));
    }
    solid.shells.len().hash(&mut h);
    for shell in &solid.shells {
        (shell.id, shell.faces.len()).hash(&mut h);
        for face in &shell.faces {
            let surface = &face.surface;
            (face.id, face.same_sense, surface.degree_u, surface.degree_v, face.loops.len()).hash(&mut h);
            surface.knots_u.iter().chain(&surface.knots_v).for_each(|k| k.to_bits().hash(&mut h));
            surface.control_points.len().hash(&mut h);
            for row in &surface.control_points {
                row.len().hash(&mut h);
                row.iter().for_each(|c| c.w.to_bits().hash(&mut h));
            }
            for loop_record in &face.loops {
                (loop_record.id, loop_record.coedges.len()).hash(&mut h);
                for coedge in &loop_record.coedges {
                    (coedge.id, coedge.edge_id, coedge.forward, coedge.pcurve.degree).hash(&mut h);
                    coedge.pcurve.knots.iter().for_each(|k| k.to_bits().hash(&mut h));
                    for c in &coedge.pcurve.control_points {
                        [c.x, c.y, c.z, c.w].iter().for_each(|v| v.to_bits().hash(&mut h));
                    }
                }
            }
        }
    }
    h.finish()
}

/// The rigid map taking `base` onto `solid`, when `solid` is `base` moved
/// rigidly: identical in everything but point positions, and the map carries
/// every vertex and every edge-curve and surface control point of `base` to
/// `solid`'s within `1e-9` of the larger coordinate magnitude. `None`
/// otherwise (a scaled, mirrored or reshaped copy, or no three points that
/// fix a frame).
fn congruent_pose(base: &BrepSolid, solid: &BrepSolid) -> Option<RigidPose> {
    if !same_but_positions(base, solid) {
        return None;
    }
    let points = |s: &BrepSolid| -> Option<Vec<(Vec3, f64)>> {
        let mut out: Vec<(Vec3, f64)> = s.vertices.iter().map(|v| (v.point, 1.0)).collect();
        for edge in &s.edges {
            for c in &edge.curve.control_points {
                out.push((c.point().ok()?, c.w));
            }
        }
        for shell in &s.shells {
            for face in &shell.faces {
                for row in &face.surface.control_points {
                    for c in row {
                        out.push((c.point().ok()?, c.w));
                    }
                }
            }
        }
        Some(out)
    };
    let from = points(base)?;
    let to = points(solid)?;
    if from.len() != to.len() || from.len() < 3 {
        return None;
    }
    let pose = frame_pose(&from, &to)?;
    let magnitude = from
        .iter()
        .chain(&to)
        .map(|(p, _)| p.x.abs().max(p.y.abs()).max(p.z.abs()))
        .fold(1.0f64, f64::max);
    let tolerance = 1e-9 * magnitude;
    from.iter()
        .zip(&to)
        .all(|((p, wp), (q, wq))| wp.to_bits() == wq.to_bits() && pose.point(*p).sub(*q).length() <= tolerance)
        .then_some(pose)
}

/// The proper rigid map fixed by three well-spread points of `from` and their
/// partners in `to`: orthonormal frames on (p0, farthest from p0, farthest
/// from that line), rotation = frame_to * frame_fromᵀ.
fn frame_pose(from: &[(Vec3, f64)], to: &[(Vec3, f64)]) -> Option<RigidPose> {
    let p0 = from[0].0;
    let (i1, d1) = from
        .iter()
        .enumerate()
        .map(|(i, (p, _))| (i, p.sub(p0).length()))
        .fold((0, 0.0), |best, next| if next.1 > best.1 { next } else { best });
    if !(d1 > 0.0) {
        return None;
    }
    let axis = from[i1].0.sub(p0).scale(1.0 / d1);
    let (i2, d2) = from
        .iter()
        .enumerate()
        .map(|(i, (p, _))| {
            let d = p.sub(p0);
            (i, d.sub(axis.scale(d.dot(axis))).length())
        })
        .fold((0, 0.0), |best, next| if next.1 > best.1 { next } else { best });
    if !(d2 > 1e-6 * d1) {
        return None;
    }
    let frame = |a: Vec3, b: Vec3, c: Vec3| -> Option<[Vec3; 3]> {
        let e1 = b.sub(a);
        let e1 = e1.scale(1.0 / e1.length());
        let d = c.sub(a);
        let e2 = d.sub(e1.scale(d.dot(e1)));
        let length = e2.length();
        if !(length > 0.0) {
            return None;
        }
        let e2 = e2.scale(1.0 / length);
        Some([e1, e2, e1.cross(e2)])
    };
    let f = frame(p0, from[i1].0, from[i2].0)?;
    let g = frame(to[0].0, to[i1].0, to[i2].0)?;
    // rotation = Σ_k g_k f_kᵀ
    let mut rotation = [[0.0f64; 3]; 3];
    for k in 0..3 {
        let (gk, fk) = ([g[k].x, g[k].y, g[k].z], [f[k].x, f[k].y, f[k].z]);
        for row in 0..3 {
            for col in 0..3 {
                rotation[row][col] += gk[row] * fk[col];
            }
        }
    }
    let mut pose = RigidPose { rotation, translation: [0.0; 3] };
    let moved = pose.point(p0);
    let q0 = to[0].0;
    pose.translation = [q0.x - moved.x, q0.y - moved.y, q0.z - moved.z];
    pose.translation.iter().all(|t| t.is_finite()).then_some(pose)
}

/// Everything but point positions (and names) equal, field by field.
fn same_but_positions(a: &BrepSolid, b: &BrepSolid) -> bool {
    let same_bits = |x: &[f64], y: &[f64]| x.len() == y.len() && x.iter().zip(y).all(|(p, q)| p.to_bits() == q.to_bits());
    let same_weights = |x: &[Vec4], y: &[Vec4]| x.len() == y.len() && x.iter().zip(y).all(|(p, q)| p.w.to_bits() == q.w.to_bits());
    let same_pcurve = |x: &NurbsCurve, y: &NurbsCurve| {
        x.degree == y.degree
            && same_bits(&x.knots, &y.knots)
            && x.control_points.len() == y.control_points.len()
            && x.control_points.iter().zip(&y.control_points).all(|(p, q)| {
                [p.x, p.y, p.z, p.w].iter().zip([q.x, q.y, q.z, q.w]).all(|(u, v)| u.to_bits() == v.to_bits())
            })
    };
    a.kind == b.kind
        && a.genus == b.genus
        && a.cells.len() == b.cells.len()
        && a.cells.iter().zip(&b.cells).all(|(x, y)| x.id == y.id && x.void == y.void && x.faces == y.faces)
        && a.vertices.len() == b.vertices.len()
        && a.vertices.iter().zip(&b.vertices).all(|(x, y)| x.id == y.id)
        && a.edges.len() == b.edges.len()
        && a.edges.iter().zip(&b.edges).all(|(x, y)| {
            x.id == y.id
                && x.start_vertex_id == y.start_vertex_id
                && x.end_vertex_id == y.end_vertex_id
                && x.degenerate == y.degenerate
                && x.t0.to_bits() == y.t0.to_bits()
                && x.t1.to_bits() == y.t1.to_bits()
                && x.curve.degree == y.curve.degree
                && same_bits(&x.curve.knots, &y.curve.knots)
                && same_weights(&x.curve.control_points, &y.curve.control_points)
        })
        && a.shells.len() == b.shells.len()
        && a.shells.iter().zip(&b.shells).all(|(x, y)| {
            x.id == y.id
                && x.faces.len() == y.faces.len()
                && x.faces.iter().zip(&y.faces).all(|(f, g)| {
                    f.id == g.id
                        && f.same_sense == g.same_sense
                        && f.surface.degree_u == g.surface.degree_u
                        && f.surface.degree_v == g.surface.degree_v
                        && same_bits(&f.surface.knots_u, &g.surface.knots_u)
                        && same_bits(&f.surface.knots_v, &g.surface.knots_v)
                        && f.surface.control_points.len() == g.surface.control_points.len()
                        && f.surface.control_points.iter().zip(&g.surface.control_points).all(|(r, s)| same_weights(r, s))
                        && f.loops.len() == g.loops.len()
                        && f.loops.iter().zip(&g.loops).all(|(l, m)| {
                            l.id == m.id
                                && l.coedges.len() == m.coedges.len()
                                && l.coedges.iter().zip(&m.coedges).all(|(c, d)| {
                                    c.id == d.id && c.edge_id == d.edge_id && c.forward == d.forward && same_pcurve(&c.pcurve, &d.pcurve)
                                })
                        })
                })
        })
}

/// `base`'s payload carried by `pose`, with `solid`'s own face and edge ids
/// and names and its own vertices (the meshed copy and `solid` share every id,
/// [`same_but_positions`]).
fn posed_payload(base: &DisplaySolidPayload, pose: &RigidPose, solid: &BrepSolid) -> DisplaySolidPayload {
    let mut mesh = base.mesh.clone();
    for p in mesh.positions.chunks_exact_mut(3) {
        let q = pose.point(Vec3::new(p[0], p[1], p[2]));
        p.copy_from_slice(&[q.x, q.y, q.z]);
    }
    for n in mesh.normals.chunks_exact_mut(3) {
        let q = pose.direction(Vec3::new(n[0], n[1], n[2]));
        n.copy_from_slice(&[q.x, q.y, q.z]);
    }
    let faces = solid
        .shells
        .iter()
        .flat_map(|shell| shell.faces.iter().map(|face| (face.id, face.name.clone())))
        .collect();
    let edge_names: std::collections::HashMap<u64, &String> =
        solid.edges.iter().filter_map(|edge| edge.name.as_ref().map(|name| (edge.id, name))).collect();
    let edges = base
        .edges
        .iter()
        .map(|(id, _, points)| (*id, edge_names.get(id).map(|name| (*name).clone()), points.iter().map(|p| pose.point(*p)).collect()))
        .collect();
    let vertices = solid.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    DisplaySolidPayload { faces, mesh, edges, vertices, chord_tolerance: base.chord_tolerance }
}

/// The display payload of one solid; see [`display_payload_handle_native`].
fn display_payload_of_solid(solid: &BrepSolid, lod_factor: f64) -> Result<DisplaySolidPayload, String> {
    let chord = display_chord_tolerance(solid, lod_factor);
    let mesh = tessellate_brep_watertight(solid, chord)?;
    let mut faces = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            faces.push((face.id, face.name.clone()));
        }
    }
    let edge_names: std::collections::HashMap<u64, String> = solid
        .edges
        .iter()
        .filter_map(|edge| edge.name.as_ref().map(|name| (edge.id, name.clone())))
        .collect();
    let edges = sample_edge_polylines(solid, chord)?
        .into_iter()
        .map(|(id, points)| (id, edge_names.get(&id).cloned(), points))
        .collect();
    let vertices = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    Ok(DisplaySolidPayload {
        faces,
        mesh,
        edges,
        vertices,
        chord_tolerance: chord,
    })
}

/// Native display payload for a solved sketch PROFILE — the SHEET-SOLID view of a
/// committed sketch: a planar FACE mesh + its named boundary EDGES + corner
/// VERTICES, so a sketch is pickable / selectable / measurable through the exact
/// same display path as a real solid. NO solid is registered — the payload is
/// synthesized directly from the profile, so it owns no scene handle (the display
/// carries `source_handle = 0`).
///
/// The FACE triangulates each region (outer boundary minus holes) via the
/// watertight planar path, mapped to world through the profile's frame; each
/// triangle rides face id 0 (one logical sheet face). EDGES are each boundary
/// curve sampled to a world polyline, named from the profile's `{sketchId}:G{gid}`
/// edge names; VERTICES are the loop corners. An empty profile (no closed region)
/// yields an empty payload (no face, no edges) — an open/underconstrained sketch
/// has no sheet.
pub fn sketch_profile_display_payload(
    profile: &crate::feature_pipeline::SketchProfile,
) -> DisplaySolidPayload {
    /// Samples per boundary curve — enough to resolve arcs/circles; straight
    /// segments oversample harmlessly.
    const SEGMENTS: usize = 24;
    let origin = profile.origin;
    let x_axis = profile.x_axis;
    let y_axis = profile.y_axis;
    let normal = profile.z_axis;
    let to_uv = |p: Vec3| {
        let d = p.sub(origin);
        [d.dot(x_axis), d.dot(y_axis)]
    };

    let mut mesh = Mesh::default();
    let mut edges: Vec<(u64, Option<String>, Vec<Vec3>)> = Vec::new();
    let mut vertices: Vec<(u64, Vec3)> = Vec::new();
    let mut next_edge_id: u64 = 0;
    let mut next_vertex_id: u64 = 0;
    let mut sketch_id: Option<String> = None;

    for region in &profile.regions {
        let Some((outer, holes)) = region.split_first() else {
            continue;
        };
        // Boundary vertices (uv, world) for triangulation: sample [t0, t1) per
        // curve so the next curve contributes the shared corner exactly once.
        let boundary = |lp: &crate::feature_pipeline::ProfileLoop| -> Vec<([f64; 2], Vec3)> {
            let mut out = Vec::new();
            for curve in &lp.curves {
                let Ok([t0, t1]) = curve.domain() else { continue };
                for step in 0..SEGMENTS {
                    let t = t0 + (t1 - t0) * step as f64 / SEGMENTS as f64;
                    if let Ok(p) = curve.evaluate(t) {
                        out.push((to_uv(p), p));
                    }
                }
            }
            out
        };
        let outer_b = boundary(outer);
        let holes_b: Vec<Vec<([f64; 2], Vec3)>> = holes.iter().map(boundary).collect();
        for [a, b, c] in watertight_tessellation::triangulate_planar_region(&outer_b, &holes_b) {
            let base = (mesh.positions.len() / 3) as u32;
            for p in [a, b, c] {
                mesh.positions.extend([p.x, p.y, p.z]);
                mesh.normals.extend([normal.x, normal.y, normal.z]);
            }
            mesh.indices.extend([base, base + 1, base + 2]);
            mesh.face_ids.push(0);
        }

        // Named boundary EDGES + corner VERTICES for every loop of this region.
        for lp in region {
            for (index, curve) in lp.curves.iter().enumerate() {
                let Ok([t0, t1]) = curve.domain() else { continue };
                let mut polyline = Vec::with_capacity(SEGMENTS + 1);
                for step in 0..=SEGMENTS {
                    let t = t0 + (t1 - t0) * step as f64 / SEGMENTS as f64;
                    if let Ok(p) = curve.evaluate(t) {
                        polyline.push(p);
                    }
                }
                let name = lp.edge_names.get(index).cloned().flatten();
                if sketch_id.is_none() {
                    // `{sketchId}:G{gid}` → the sheet's face inherits `{sketchId}`.
                    sketch_id = name
                        .as_deref()
                        .and_then(|n| n.split_once(":G").map(|(id, _)| id.to_string()));
                }
                if polyline.len() >= 2 {
                    edges.push((next_edge_id, name, polyline));
                    next_edge_id += 1;
                }
                if let Ok(p) = curve.evaluate(t0) {
                    vertices.push((next_vertex_id, p));
                    next_vertex_id += 1;
                }
            }
        }
    }

    // One logical sheet face (id 0), only if the mesh has triangles.
    let faces = if mesh.face_ids.is_empty() {
        Vec::new()
    } else {
        let face_name = sketch_id.map(|id| format!("{id}:FACE"));
        vec![(0u64, face_name)]
    };
    DisplaySolidPayload {
        faces,
        mesh,
        edges,
        vertices,
        chord_tolerance: 0.0,
    }
}

/// Native display payload for a whole committed SKETCH: its profile SHEET (when
/// the sketch closes a region) PLUS every model SEGMENT the sheet does not
/// already draw.
///
/// [`sketch_profile_display_payload`] draws a sketch through its closed profile,
/// and that is the only display a committed sketch had. A sketch that closes
/// NOTHING — one line, an open chain, a trajectory drawn alongside a closed loop —
/// publishes no profile at all, so it was handed to the sheet builder as `None`
/// and drew nothing: invisible in 3D, and unpickable there (the 2026-09-02 report,
/// *"Sketch with single edge not visible in 3D"*). Its geometry was never missing,
/// only unaccepted — the SKETCH feature already publishes one world curve per
/// model segment under `{sketchId}:G{gid}`, which is exactly what `segments`
/// carries here.
///
/// Each such segment draws as one NAMED edge (the same name a downstream
/// `reference_selection` stores, so picking the drawn line resolves) with its
/// endpoints as vertices. A segment whose name the sheet already drew is SKIPPED,
/// so a closed sketch draws each boundary edge exactly once and a mixed sketch
/// draws its loop and its open chain side by side. Endpoints are deduped against
/// the points already drawn — consecutive segments of one chain share a junction.
///
/// `points` are the sketch's standalone MODEL points (world space): a sketch
/// with points only — a hole-placement sketch — has no segment at all, so it
/// drew nothing and was invisible and unpickable in 3D. Each point draws as a
/// vertex, deduped against the endpoints the segments already drew, so a point
/// that is also a segment corner draws once.
///
/// Like its sheet sibling this registers no solid: the payload is synthesized
/// directly from the profile, the curves and the points.
pub fn sketch_display_payload(
    profile: Option<&crate::feature_pipeline::SketchProfile>,
    segments: &[(String, Vec<NurbsCurve>)],
    points: &[Vec3],
) -> DisplaySolidPayload {
    /// Samples per segment for a simple curve — the sheet builder's own
    /// resolution. A curve with many knot spans (a fitted helix has ~64 per
    /// turn) gets four samples per span instead, so a long fitted curve draws
    /// as itself rather than as a coarse polygon; capped so a pathological
    /// knot vector cannot flood the display.
    const SEGMENTS: usize = 24;
    const SAMPLES_PER_SPAN: usize = 4;
    const MAX_SEGMENTS: usize = 4096;
    /// Two drawn endpoints closer than this are the same corner (a chain's
    /// junction points are the SAME solved sketch point, so they agree exactly).
    const SAME_POINT: f64 = 1e-9;

    let mut payload = match profile {
        Some(profile) => sketch_profile_display_payload(profile),
        None => DisplaySolidPayload {
            faces: Vec::new(),
            mesh: Mesh::default(),
            edges: Vec::new(),
            vertices: Vec::new(),
            chord_tolerance: 0.0,
        },
    };

    let drawn: std::collections::HashSet<String> = payload
        .edges
        .iter()
        .filter_map(|(_, name, _)| name.clone())
        .collect();
    let mut next_edge_id = payload
        .edges
        .iter()
        .map(|(id, _, _)| id + 1)
        .max()
        .unwrap_or(0);
    let mut next_vertex_id = payload.vertices.iter().map(|(id, _)| id + 1).max().unwrap_or(0);

    for (name, curves) in segments {
        if drawn.contains(name) {
            continue;
        }
        for curve in curves {
            let Ok([t0, t1]) = curve.domain() else { continue };
            let spans = curve
                .knots
                .windows(2)
                .filter(|pair| pair[1] > pair[0])
                .count();
            let segments = (spans * SAMPLES_PER_SPAN).clamp(SEGMENTS, MAX_SEGMENTS);
            let mut polyline = Vec::with_capacity(segments + 1);
            for step in 0..=segments {
                let t = t0 + (t1 - t0) * step as f64 / segments as f64;
                if let Ok(point) = curve.evaluate(t) {
                    polyline.push(point);
                }
            }
            if polyline.len() < 2 {
                continue;
            }
            for end in [polyline[0], polyline[polyline.len() - 1]] {
                if payload
                    .vertices
                    .iter()
                    .any(|(_, point)| point.sub(end).length() <= SAME_POINT)
                {
                    continue;
                }
                payload.vertices.push((next_vertex_id, end));
                next_vertex_id += 1;
            }
            payload.edges.push((next_edge_id, Some(name.clone()), polyline));
            next_edge_id += 1;
        }
    }

    // Standalone points: one vertex each, skipping any position a segment
    // endpoint (or an earlier point) already drew.
    for &point in points {
        if payload
            .vertices
            .iter()
            .any(|(_, drawn)| drawn.sub(point).length() <= SAME_POINT)
        {
            continue;
        }
        payload.vertices.push((next_vertex_id, point));
        next_vertex_id += 1;
    }
    payload
}

/// Full mass properties of a resident solid, scaled to `density` (the native,
/// non-wasm sibling of [`mass_properties_handle`] for the in-process renderer):
/// volume + surface area + centroid + centroidal inertia tensor + principal
/// axes/moments (Golovanov §8.11). The underlying geometry is unit-density; here
/// `mass = density * volume` and every inertia quantity scales linearly with
/// density (centroid + principal axes are density-independent). `density` is in
/// mass units per mm³ (the kernel's length convention is millimetres); pass
/// `1.0` for the raw geometric result (`mass == volume`). Reads the solid in one
/// registry borrow; the topology never crosses a boundary.
pub fn mass_properties_handle_native(
    handle: u32,
    density: f64,
) -> Result<DensityMassProperties, String> {
    with_registered_solid_str(handle, |solid| {
        Ok(solid_mass_properties_full(solid)?.with_density(density))
    })
}

/// Surface area and volume of a resident solid — the native sibling of
/// [`mass_properties_handle_native`] for a caller that reads NOTHING but
/// those two numbers (`examples/case_replay.rs` records volume and area per
/// solid and discards the rest). The full entry runs a second station sweep
/// over every trimmed face for the nine moment integrands, which costs as
/// much again as the area/volume sweep: on `anotherBooleanFail`'s imported
/// 32-face body 2.6 s against 1.3 s, and 5.3 s of that case's 12.9 s replay
/// went to moments nobody read. The two numbers come out of the same
/// `solid_mass_properties` call the full entry starts with, so they are
/// bit-identical to its `volume` and `surface_area`; the full entry's
/// refusal of a non-positive volume is kept so a caller switching over sees
/// the same `Err` on the same solids. Reads the solid in one registry borrow.
pub fn mass_measures_handle_native(handle: u32) -> Result<MassProperties, String> {
    with_registered_solid_str(handle, |solid| {
        let _caller = crate::mass_caller("abi.mass_measures");
        let properties = solid_mass_properties(solid)?;
        if properties.volume.abs() <= 1e-30 {
            return Err("solid_mass_properties_full: non-positive volume".into());
        }
        Ok(properties)
    })
}

/// Topology validation issues of a resident solid as `(severity, message)`
/// pairs — the native sibling of the JSON validators, for in-process
/// qualification tooling (`examples/case_replay.rs`). An empty Vec means the
/// incidence checks passed; that is NOT a correctness proof: `validate()` tests
/// neither connectivity nor face-vs-face self-intersection, and a validating
/// solid can still be the wrong solid. Reads the solid in one registry borrow.
pub fn validate_handle_native(handle: u32) -> Result<Vec<(String, String)>, String> {
    with_registered_solid_str(handle, |solid| {
        Ok(solid
            .validate()
            .into_iter()
            .map(|issue| (issue.severity.to_string(), issue.message))
            .collect())
    })
}

/// Topology TOTALS of a resident solid as
/// `(faces, edges, non_degenerate_edges, vertices)`, read straight off the
/// record: every face of every shell, every edge, every edge that is not a
/// collapsed pole boundary, every vertex.
///
/// The two edge counts answer different questions and both are worth
/// recording. The RAW total is what a reader means by "15 edges" when looking
/// at the record, and it is the right thing to diff between two runs. The
/// NON-DEGENERATE count is the one a derivation can state from the intended
/// construction, because it excludes the seam and pole boundaries the kernel
/// chose as scaffolding rather than anything the shape has.
///
/// These are what a case gate compares. A solid can keep its volume to the
/// last digit while gaining or losing edges — `inbox-20260909-tube-joint-edge-splits`
/// carried "7 faces / 17 edges / 11 vertices" in prose for a month with every
/// local assertion in its suite passing — so the totals are recorded and
/// diffed.
///
/// Since 2026-09-20 they can also be STATED. The two checks answer different
/// questions and both are kept — a diff against the baseline says "did this
/// change", a claim says "was it ever right" — and it is the NON-degenerate
/// count that a claim may state, for the reason above. Reads the solid in one
/// registry borrow.
pub fn topology_counts_native(handle: u32) -> Result<(usize, usize, usize, usize), String> {
    with_registered_solid_str(handle, |solid| {
        Ok((
            solid.shells.iter().map(|shell| shell.faces.len()).sum(),
            solid.edges.len(),
            solid.edges.iter().filter(|edge| !edge.degenerate).count(),
            solid.vertices.len(),
        ))
    })
}

/// Connectivity of a resident solid — the check `validate()` does NOT make.
/// Per-shell edge-connected face components, edges shared across shells, and
/// pinch vertices. Pure topology, no geometry, no tolerance; see
/// [`crate::solid_connectivity`] for which part is a gate and which is a
/// diagnostic. Reads the solid in one registry borrow.
pub fn connectivity_handle_native(handle: u32) -> Result<crate::ConnectivityReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::solid_connectivity(solid)))
}

/// Closedness and Euler parity of a resident solid — the THIRD check
/// `validate()` does not make on its own. Its Euler comparison reduces, on a
/// freshly derived solid, to exactly "is the characteristic even", and it is
/// switched off on any solid carrying a degenerate edge; this asks the question
/// directly and on every solid. Pure counting, one registry borrow.
pub fn euler_handle_native(handle: u32) -> Result<crate::EulerReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::solid_euler(solid)))
}

/// The body-kind report of a resident solid. A report beside the Euler
/// one, not a gate: no body carries a kind yet. Pure counting, one
/// registry borrow.
pub fn topology_kind_handle_native(handle: u32) -> Result<crate::KindReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::topology_kind(solid)))
}

/// The kind a resident solid DECLARES (`BrepSolid::kind`), as distinct
/// from the kind the report above reads from the edge census. `Solid`
/// for every body built today.
pub fn body_kind_handle_native(handle: u32) -> Result<crate::BodyKind, String> {
    with_registered_solid_str(handle, |solid| Ok(solid.kind))
}

/// Face-loop self-crossing of a resident solid — the check no OTHER detector
/// here can make. `validate()` is silent (the incidence is perfect), the
/// face-versus-face scan has no pair to confirm (both triangles are on one
/// face), connectivity and Euler parity count the same cells either way; the
/// face's AREA is the only other tell. Pure parameter-space work on the trims
/// the face already carries — no tessellation — so it is cheap enough to be a
/// default. Reads the solid in one registry borrow.
pub fn loop_crossings_handle_native(handle: u32) -> Result<crate::LoopCrossingReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::loop_self_crossings(solid)))
}

/// The vector-area closure of a resident solid: does each closed shell's
/// TRIMS enclose the zero vector area a closed shell owes, and if not, which
/// faces own the residual. One pass over the trims and the edge curves — no
/// tessellation, no projection — so unlike the crossing scan it is cheap
/// enough for a default gate. Reads the solid in one registry borrow.
pub fn vector_area_handle_native(handle: u32) -> Result<crate::VectorAreaReport, String> {
    with_registered_solid_str(handle, |solid| Ok(crate::shell_vector_areas(solid)))
}

/// Face-versus-face self-intersection of a resident solid — the OTHER check
/// `validate()` does not make (its one "self-intersects" string is a uv-wire
/// warning). Opt-in and expensive: it tessellates the solid and walks a
/// triangle BVH, so no default gate calls it. `chord_tolerance` overrides the
/// per-solid display density when positive. Reads the solid in one registry
/// borrow.
pub fn self_intersections_handle_native(
    handle: u32,
    chord_tolerance: f64,
) -> Result<crate::SelfIntersectionReport, String> {
    with_registered_solid_str(handle, |solid| {
        let mut options = crate::SelfIntersectionOptions::for_solid(solid);
        if chord_tolerance > 0.0 && chord_tolerance.is_finite() {
            options.chord_tolerance = chord_tolerance;
        }
        crate::solid_self_intersections(solid, options)
    })
}

/// The Transform feature's BBOX_CENTER pivot for a resident source solid.
/// Share the kernel's vertex-bbox definition with viewport controls; display
/// tessellation bounds would give a different pivot for curved solids.
pub fn transform_pivot_native(handle: u32) -> Result<[f64; 3], String> {
    with_registered_solid_str(handle, |solid| {
        Ok(crate::feature_pipeline::transform_bbox_center(solid))
    })
}

/// Total 3D arc length (mm) of every non-degenerate edge of a resident solid —
/// the Properties panel's "total edge length" measurement. Native sibling of the
/// mass-properties accessors; one short registry borrow, topology never crosses
/// the boundary.
pub fn solid_edge_length_total_native(handle: u32) -> Result<f64, String> {
    with_registered_solid_str(handle, solid_edge_length_total)
}

/// `(area, boundary_edge_total_length, surface_type)` of a resident solid's
/// named face (mm² / mm / classification): the face's surface area, the summed
/// arc length of its boundary edges, and a short label for its underlying
/// carrier surface (`"Plane"`, `"Cylinder"`, `"Cone"`, `"Sphere"`, `"Torus"`,
/// `"Surface of revolution"`, or `"NURBS"` when the exact rational patch is not
/// a recognized analytic carrier). Errs if no face carries `face_name`.
pub fn face_measurements_native(
    handle: u32,
    face_name: &str,
) -> Result<(f64, f64, &'static str), String> {
    with_registered_solid_str(handle, |solid| {
        let face = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.name.as_deref() == Some(face_name))
            .ok_or_else(|| format!("face '{face_name}' not found"))?;
        let surface_type = face
            .surface
            .analytic()
            .map(|analytic| analytic.kind_label())
            .unwrap_or("NURBS");
        Ok((
            face_area(face)?,
            face_boundary_length(solid, face)?,
            surface_type,
        ))
    })
}

/// 3D arc length (mm) of a resident solid's named edge. Errs if no edge carries
/// `edge_name`.
pub fn edge_length_native(handle: u32, edge_name: &str) -> Result<f64, String> {
    with_registered_solid_str(handle, |solid| {
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.name.as_deref() == Some(edge_name))
            .ok_or_else(|| format!("edge '{edge_name}' not found"))?;
        edge_arc_length(edge)
    })
}

/// The carrier dimensions of a resident solid's named face, for the Info
/// panel: a cylinder's radius and diameter, a cone's half-angle, a sphere's
/// radius, a torus's radii (see [`crate::surface_carrier_dimensions`]).
/// `Ok(None)` for a plane or a free-form face; errs if no face carries
/// `face_name`.
pub fn face_carrier_dimensions_native(
    handle: u32,
    face_name: &str,
) -> Result<Option<crate::CarrierDimensions>, String> {
    with_registered_solid_str(handle, |solid| {
        let face = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.name.as_deref() == Some(face_name))
            .ok_or_else(|| format!("face '{face_name}' not found"))?;
        Ok(crate::surface_carrier_dimensions(&face.surface))
    })
}

/// The circle a resident solid's named edge lies on, when it does: radius,
/// diameter and an arc's sweep (see [`crate::curve_carrier_dimensions`]).
/// `Ok(None)` for a line or any other curve; errs if no edge carries
/// `edge_name`.
pub fn edge_carrier_dimensions_native(
    handle: u32,
    edge_name: &str,
) -> Result<Option<crate::CarrierDimensions>, String> {
    with_registered_solid_str(handle, |solid| {
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.name.as_deref() == Some(edge_name))
            .ok_or_else(|| format!("edge '{edge_name}' not found"))?;
        if edge.degenerate {
            return Ok(None);
        }
        Ok(crate::curve_carrier_dimensions(&edge.curve, edge.t0, edge.t1))
    })
}

/// Escape hatch: pull a resident solid's full topology across the boundary (for
/// STEP export or JSON-only lanes during the migration). Prefer handle-native
/// ops — this re-incurs the serialization cost the registry exists to avoid.
#[wasm_bindgen]
pub fn solid_handle_to_buffer(handle: u32) -> Result<WasmSolidBuffer, JsValue> {
    with_registered_solid(handle, |solid| solid_buffer(solid, "{}".into()))
}

/// Native: export the CURRENT resident solids (by handle) to an ISO-10303-21
/// STEP document. Reads each resident solid out of the thread-local registry —
/// the SAME registry [`display_payload_handle_native`] reads — clones them into
/// a `Vec<BrepSolid>`, and hands the batch to [`export_step`]. The engine-native
/// app's Export→STEP lane calls this with the handles the pipeline left resident
/// after the last history run, so the topology never crosses a boundary as JSON.
/// `String` error (JsValue-free — it links + runs on native and wasm alike).
pub fn export_step_handles(
    handles: &[u32],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    if handles.is_empty() {
        return Err("export_step_handles: no solids to export".into());
    }
    let mut solids = Vec::with_capacity(handles.len());
    for &handle in handles {
        let solid: BrepSolid = with_registered_solid_str(handle, |solid| Ok(solid.clone()))?;
        solids.push(solid);
    }
    export_step(&solids, name, unit, timestamp)
}

/// [`export_step_handles`] with each body's SCENE NAME and an optional PMI
/// block: the engine-native Export→STEP lane. Names key the
/// `MANIFOLD_SOLID_BREP`s and the PMI reference resolution.
pub fn export_step_named_handles(
    named: &[(String, u32)],
    name: &str,
    unit: &str,
    timestamp: &str,
    pmi: Option<&crate::StepPmi<'_>>,
    colors: Option<&crate::StepColors>,
) -> Result<crate::StepExportReport, String> {
    if named.is_empty() {
        return Err("export_step_named_handles: no solids to export".into());
    }
    let mut solids = Vec::with_capacity(named.len());
    for (solid_name, handle) in named {
        let solid: BrepSolid = with_registered_solid_str(*handle, |solid| Ok(solid.clone()))?;
        solids.push((solid_name.clone(), solid));
    }
    let borrowed: Vec<(String, &BrepSolid)> = solids
        .iter()
        .map(|(solid_name, solid)| (solid_name.clone(), solid))
        .collect();
    crate::export_step_report_named(&borrowed, name, unit, timestamp, pmi, colors)
}

/// STRUCTURED export: the resident scene PLUS the document's assembly model, as
/// an AP242 file with real product structure — one `PRODUCT` per parts-library
/// entry, one `NEXT_ASSEMBLY_USAGE_OCCURRENCE` per placed instance.
///
/// `named` is the whole resident scene (component-owned bodies included — the
/// tree builder filters them, because their geometry is written unposed in the
/// part's own product instead). `components` is one entry per ACOMP instance of
/// the ROOT document — `(component id, parts-library entry name, live pose)` —
/// which only the caller knows, since a solved or gizmo-moved instance's pose
/// lives in the app's projection of the scene.
pub fn export_step_assembly_handles(
    document_name: &str,
    named: &[(String, u32)],
    components: &[(String, String, crate::Mat4)],
    unit: &str,
    timestamp: &str,
    pmi: Option<&crate::StepPmi<'_>>,
    colors: Option<&crate::StepColors>,
) -> Result<crate::StepExportReport, String> {
    if named.is_empty() {
        return Err("export_step_assembly_handles: no solids to export".into());
    }
    let mut solids = Vec::with_capacity(named.len());
    for (solid_name, handle) in named {
        let solid: BrepSolid = with_registered_solid_str(*handle, |solid| Ok(solid.clone()))?;
        solids.push((solid_name.clone(), solid));
    }
    let assembly = crate::assembly_export_tree(document_name, solids, components)?;
    crate::export_step_assembly_report(&assembly, unit, timestamp, pmi, colors)
}

/// Native: export the CURRENT resident solids (by handle) to an IGES 5.3
/// document of trimmed NURBS surfaces. The IGES analogue of
/// [`export_step_handles`] — reads the resident solids out of the thread-local
/// registry, clones them into a `Vec<BrepSolid>`, and hands the batch to
/// [`export_iges`]. `String` error (JsValue-free — links on native and wasm).
pub fn export_iges_handles(
    handles: &[u32],
    name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<String, String> {
    if handles.is_empty() {
        return Err("export_iges_handles: no solids to export".into());
    }
    let mut solids = Vec::with_capacity(handles.len());
    for &handle in handles {
        let solid: BrepSolid = with_registered_solid_str(handle, |solid| Ok(solid.clone()))?;
        solids.push(solid);
    }
    export_iges(&solids, name, unit, timestamp)
}
