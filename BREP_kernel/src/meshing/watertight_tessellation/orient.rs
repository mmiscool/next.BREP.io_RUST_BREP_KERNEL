use super::*;

/// Watertight tessellation of a closed solid: coincident triangulation along
/// every shared edge, density driven by `chord_tolerance`.
pub fn tessellate_brep_watertight(solid: &BrepSolid, chord_tolerance: f64) -> Result<Mesh, String> {
    // A disconnected shell is not necessarily another exterior component: in
    // a BREP_WITH_VOIDS its authored winding is negative so its normal points
    // away from material and into the cavity. Preserve that sign through the
    // mesh coherence pass. The overwhelmingly common one-shell path remains
    // the historical positive/outward convention without an extra integral.
    let mut face_shell_signs = Vec::new();
    if solid.shells.len() == 1 {
        face_shell_signs.resize(solid.shells[0].faces.len(), 1_i8);
    } else {
        for shell in &solid.shells {
            let volume = crate::mass_properties::shell_signed_volume(shell)?;
            let sign = if volume < 0.0 { -1_i8 } else { 1_i8 };
            face_shell_signs.resize(face_shell_signs.len() + shell.faces.len(), sign);
        }
    }
    // The whole solid is stride 1 (every face). Validation (closed-shell,
    // no odd-use edges) only makes sense on the complete mesh.
    let profile = crate::watertight_tessellation::profile::begin();
    let mut mesh = tessellate_brep_watertight_face_stride(solid, chord_tolerance, 1, 0)?;
    // Final orientation pass over the COMPLETE mesh: make every shared edge
    // coherent and match each component to its source shell's material-side
    // orientation. Faces are emitted
    // with the `same_sense` convention, which is per-face consistent but leaves
    // some faces (CW-in-uv seam bands / folded patches) or whole shells wound
    // inward; this pass repairs both at the mesh level. Positions are never
    // touched, so watertightness is preserved (validated below).
    orient_mesh_coherently(&mut mesh, chord_tolerance, &face_shell_signs)?;
    mesh.validate()?;
    crate::watertight_tessellation::profile::end(profile, chord_tolerance, mesh.indices.len() / 3);
    Ok(mesh)
}

/// Geometric normal of triangle `t` in its CURRENT winding (`(b-a)×(c-a)`),
/// magnitude 2·area. Read-only over the mesh arrays.
pub(super) fn tri_geometric_normal(positions: &[f64], indices: &[u32], t: usize) -> [f64; 3] {
    let p = |i: usize| {
        let i = i * 3;
        [positions[i], positions[i + 1], positions[i + 2]]
    };
    let a = p(indices[3 * t] as usize);
    let b = p(indices[3 * t + 1] as usize);
    let c = p(indices[3 * t + 2] as usize);
    let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ]
}

/// Signed volume of the tetrahedron (origin, a, b, c) for triangle `t` in its
/// CURRENT winding: `a · (b × c) / 6`. Summed over a closed shell it is positive
/// iff the winding faces outward (translation-invariant for a closed surface).
pub(super) fn tetra_signed_volume(positions: &[f64], indices: &[u32], t: usize) -> f64 {
    let p = |i: usize| {
        let i = i * 3;
        [positions[i], positions[i + 1], positions[i + 2]]
    };
    let a = p(indices[3 * t] as usize);
    let b = p(indices[3 * t + 1] as usize);
    let c = p(indices[3 * t + 2] as usize);
    (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0]))
        / 6.0
}

/// Final mesh-level orientation pass: make every 2-manifold shared edge
/// COHERENT (traversed once in each direction) and wind each connected
/// component to its source shell's material orientation. This repairs both
/// within-face incoherence (a folded or
/// trimmed face whose per-vertex analytic normal flips, so the `same_sense`
/// emission winds half of it inside-out) and whole faces / shells wound inward.
///
/// Positions are never modified — only a triangle's 2nd/3rd index may be swapped
/// and a stored normal's sign flipped — so the mesh stays watertight. Runs in
/// O(V + T): hashed welding, a triangle flood-fill over shared edges, then each
/// component oriented by the sign of its centroid-referenced signed volume
/// (fold-immune; the stored analytic normals are NOT trusted for orientation
/// because they are exactly what flips), matched to the source shell sign.
pub(crate) fn orient_mesh_coherently(
    mesh: &mut Mesh,
    chord: f64,
    face_shell_signs: &[i8],
) -> Result<(), String> {
    let tri_count = mesh.indices.len() / 3;
    if tri_count == 0 {
        return Ok(());
    }
    let vcount = mesh.positions.len() / 3;
    if vcount == 0 {
        return Ok(());
    }
    let _timer = std::env::var("BREP_TIME_ORIENT")
        .ok()
        .map(|_| web_time::Instant::now());

    // (a) Weld raw vertices by quantized position, so triangles from adjacent
    // faces that meet along a shared edge share welded endpoints.
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for i in 0..vcount {
        for k in 0..3 {
            let x = mesh.positions[3 * i + k];
            if x < lo[k] {
                lo[k] = x;
            }
            if x > hi[k] {
                hi[k] = x;
            }
        }
    }
    let diag = {
        let d = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
    };
    if !(diag > 0.0) {
        return Ok(());
    }
    // Shared boundary samples are identical by construction. Preserve their
    // exact edge adjacency before considering approximate seam matches: a
    // chord-sized weld grid must not collapse a real, short boundary edge.
    let mut exact: FxHashMap<[u64; 3], u32> = FxHashMap::default();
    let mut welded = Vec::with_capacity(vcount);
    for point in mesh.positions.chunks_exact(3) {
        if point.iter().any(|x| !x.is_finite()) {
            return Err("orient_mesh_coherently: nonfinite vertex".into());
        }
        let key = std::array::from_fn(|i| {
            if point[i] == 0. {
                0
            } else {
                point[i].to_bits()
            }
        });
        let next = exact.len() as u32;
        welded.push(*exact.entry(key).or_insert(next));
    }
    let mut edge_tris: FxHashMap<(u32, u32), Vec<(u32, bool)>> = FxHashMap::default();
    edge_tris.reserve(tri_count * 3);
    for (t, triangle) in mesh.indices.chunks_exact(3).enumerate() {
        for i in 0..3 {
            let a = welded[triangle[i] as usize];
            let b = welded[triangle[(i + 1) % 3] as usize];
            if a == b {
                continue;
            }
            let (key, forward) = if a < b {
                ((a, b), true)
            } else {
                ((b, a), false)
            };
            edge_tris.entry(key).or_default().push((t as u32, forward));
        }
    }
    // Only one-use exact edges need a geometric seam match. Approximate
    // groups use a disjoint ID range and cannot change an exact shared edge.
    let mut unmatched: Vec<_> = edge_tris
        .iter()
        .filter(|(_, uses)| uses.len() == 1)
        .map(|(&key, _)| key)
        .collect();
    unmatched.sort_unstable();
    if !unmatched.is_empty() {
        let quant = (chord * 1e-2).max(1e-12);
        let mut coarse: FxHashMap<[i64; 3], u32> = FxHashMap::default();
        let mut coarse_id = vec![None; exact.len()];
        for (raw, point) in mesh.positions.chunks_exact(3).enumerate() {
            let coordinates: [f64; 3] =
                std::array::from_fn(|i| ((point[i] - lo[i]) / quant).round());
            // Unrepresentable grid coordinates decline only the approximate match;
            // exact adjacency remains available, without saturating integer keys.
            if coordinates
                .iter()
                .any(|x| !x.is_finite() || x.abs() >= i64::MAX as f64)
            {
                continue;
            }
            let key = coordinates.map(|x| x as i64);
            let next = u32::try_from(exact.len() + coarse.len())
                .map_err(|_| "orient_mesh_coherently: too many welded vertices")?;
            coarse_id[welded[raw] as usize] = Some(*coarse.entry(key).or_insert(next));
        }
        let mut approximate: FxHashMap<(u32, u32), Vec<(u32, bool)>> = FxHashMap::default();
        for (a, b) in unmatched {
            let (Some(ca), Some(cb)) = (coarse_id[a as usize], coarse_id[b as usize]) else {
                continue;
            };
            if ca == cb {
                continue;
            } // never erase a finite edge for a seam match
            let uses = edge_tris.remove(&(a, b)).unwrap();
            let key = if ca < cb { (ca, cb) } else { (cb, ca) };
            approximate.entry(key).or_default().extend(
                uses.into_iter()
                    .map(|(t, forward)| (t, if ca < cb { forward } else { !forward })),
            );
        }
        edge_tris.extend(approximate);
    }

    // (c) Triangle adjacency across 2-manifold edges. `same_dir` means both
    // triangles currently traverse the shared edge the SAME way (incoherent), so
    // their flip bits must DIFFER to become coherent. Non-manifold seam / pole
    // edges (welded to > 2 triangles) are NOT propagated across — pairing their
    // triangles is geometrically ambiguous and forcing a pairing injects
    // contradictory constraints. Leaving them unpropagated only splits the mesh
    // into more components; the outward pass below orients each independently.
    let mut adj: Vec<Vec<(u32, bool)>> = vec![Vec::new(); tri_count];
    for (key, uses) in &edge_tris {
        // Establish exact components first. Approximate seam matches may
        // orient whole components, never contradict an exact shared edge.
        if key.0 >= exact.len() as u32 || uses.len() != 2 {
            continue;
        }
        let (t0, d0) = uses[0];
        let (t1, d1) = uses[1];
        let same_dir = d0 == d1;
        adj[t0 as usize].push((t1, same_dir));
        adj[t1 as usize].push((t0, same_dir));
    }
    // Canonical neighbour order so the flood-fill (and thus every flip) is
    // deterministic regardless of hash-map iteration order.
    for a in adj.iter_mut() {
        a.sort_unstable();
    }

    // Flood-fill: propagate a coherent winding across each connected component
    // (fragments split at non-manifold seams) and record each triangle's
    // component index.
    let mut flip = vec![false; tri_count];
    let mut comp_of = vec![u32::MAX; tri_count];
    let mut stack: Vec<u32> = Vec::new();
    let mut ncomp = 0u32;
    for seed in 0..tri_count {
        if comp_of[seed] != u32::MAX {
            continue;
        }
        comp_of[seed] = ncomp;
        stack.push(seed as u32);
        while let Some(t) = stack.pop() {
            let tf = flip[t as usize];
            for &(nb, same_dir) in &adj[t as usize] {
                if comp_of[nb as usize] != u32::MAX {
                    continue;
                }
                flip[nb as usize] = tf ^ same_dir;
                comp_of[nb as usize] = ncomp;
                stack.push(nb);
            }
        }
        ncomp += 1;
    }
    let ncomp = ncomp as usize;

    // (d) Fragments split at non-manifold seams are re-joined into whole SHELLS.
    // A component is internally coherent but its GLOBAL sign is an independent
    // gauge; two fragments meeting at a seam edge must traverse it oppositely, so
    // their relative gauge is fixed by that edge. A parity union-find over the
    // seam edges resolves every fragment's sign relative to its shell. (Open
    // fragments have no reliable inside/outside on their own — this is why a
    // per-fragment vote fails — but a fully assembled shell is closed, so its
    // signed volume gives a robust outward test.)
    let mut parent: Vec<u32> = (0..ncomp as u32).collect();
    let mut prel = vec![false; ncomp]; // parity from node to its parent
    fn find(parent: &mut [u32], prel: &mut [bool], mut c: u32) -> (u32, bool) {
        let mut par = false;
        // Walk to root accumulating parity.
        let mut path = Vec::new();
        while parent[c as usize] != c {
            path.push(c);
            par ^= prel[c as usize];
            c = parent[c as usize];
        }
        // Path-compress: point every visited node straight at the root with its
        // total parity to the root.
        let mut acc = par;
        for &node in path.iter() {
            let this = prel[node as usize];
            parent[node as usize] = c;
            prel[node as usize] = acc;
            acc ^= this;
        }
        (c, par)
    }
    let mut fwd: Vec<u32> = Vec::new();
    let mut rev: Vec<u32> = Vec::new();
    // Process seam edges in a CANONICAL (sorted) order so the union-find gauge is
    // deterministic regardless of hash-map iteration order.
    let mut seam_keys: Vec<(u32, u32)> = edge_tris.keys().copied().collect();
    seam_keys.sort_unstable();
    for key in &seam_keys {
        let uses = &edge_tris[key];
        // A two-use seam has an unambiguous triangle pair regardless of its
        // initial winding. For non-manifold seams retain the legacy pairing
        // of opposite raw directions. A coherent continuation
        // traverses the edge once each way, so the required relative gauge between
        // the two fragments is 1 exactly when they currently traverse it the SAME
        // effective way (effective dir = raw dir XOR flip).
        fwd.clear();
        rev.clear();
        for &(t, d) in uses {
            if d {
                fwd.push(t);
            } else {
                rev.push(t);
            }
        }
        let pairs = if uses.len() == 2 {
            1
        } else {
            fwd.len().min(rev.len())
        };
        for i in 0..pairs {
            let ((ta, da), (tb, db)) = if uses.len() == 2 {
                (uses[0], uses[1])
            } else {
                ((fwd[i], true), (rev[i], false))
            };
            let (ca, cb) = (comp_of[ta as usize], comp_of[tb as usize]);
            if ca == cb {
                continue;
            }
            let eff_a = da ^ flip[ta as usize];
            let eff_b = db ^ flip[tb as usize];
            let rel = eff_a == eff_b;
            let (ra, pa) = find(&mut parent, &mut prel, ca);
            let (rb, pb) = find(&mut parent, &mut prel, cb);
            if ra != rb {
                parent[ra as usize] = rb;
                prel[ra as usize] = pa ^ pb ^ rel;
            }
        }
    }
    // crel[c] = gauge of fragment c relative to its shell root.
    let mut crel = vec![false; ncomp];
    let mut root_of = vec![0u32; ncomp];
    for c in 0..ncomp as u32 {
        let (r, p) = find(&mut parent, &mut prel, c);
        crel[c as usize] = p;
        root_of[c as usize] = r;
    }
    // Bake the shell-relative gauge into flip, so each shell is now consistently
    // oriented (up to one global sign per shell).
    for t in 0..tri_count {
        if crel[comp_of[t] as usize] {
            flip[t] = !flip[t];
        }
    }

    // (e-orient) Match each coherent component to its SOURCE shell's material
    // orientation. Exterior/disconnected material shells target positive
    // volume; cavity shells target negative volume. A welded component may not
    // mix the two roles.
    let mut shell_vol: HashMap<u32, f64> = HashMap::default();
    let mut shell_target: HashMap<u32, i8> = HashMap::default();
    for t in 0..tri_count {
        let v = tetra_signed_volume(&mesh.positions, &mesh.indices, t);
        let v = if flip[t] { -v } else { v };
        let root = root_of[comp_of[t] as usize];
        *shell_vol.entry(root).or_insert(0.0) += v;
        let face_id = mesh.face_ids.get(t).copied().ok_or_else(|| {
            "orient_mesh_coherently: triangle is missing its source face id".to_string()
        })? as usize;
        let target = *face_shell_signs.get(face_id).ok_or_else(|| {
            format!("orient_mesh_coherently: source face id {face_id} is out of range")
        })?;
        if let Some(previous) = shell_target.insert(root, target) {
            if previous != target {
                return Err(
                    "orient_mesh_coherently: one welded component spans oppositely oriented shells"
                        .into(),
                );
            }
        }
    }
    if std::env::var("BREP_DEBUG_ORIENT_SHELLS").is_ok() {
        let mut roots: std::collections::BTreeMap<u32, usize> = Default::default();
        for t in 0..tri_count {
            *roots.entry(root_of[comp_of[t] as usize]).or_insert(0) += 1;
        }
        let mut edge_use: std::collections::BTreeMap<usize, usize> = Default::default();
        for uses in edge_tris.values() {
            *edge_use.entry(uses.len()).or_insert(0) += 1;
        }
        eprintln!(
            "[orient-shells] {ncomp} components -> {} shells | edge-use histogram {edge_use:?}",
            roots.len()
        );
        for (r, n) in roots.iter().filter(|(_, n)| **n > 1) {
            let v = shell_vol.get(r).copied().unwrap_or(0.0);
            let tgt = shell_target.get(r).copied().unwrap_or(1);
            eprintln!("  shell {r}: {n} tris, vol={v:.3e}, target={tgt}");
        }
    }
    for t in 0..tri_count {
        let r = root_of[comp_of[t] as usize];
        let volume_sign = if shell_vol.get(&r).copied().unwrap_or(0.0) < 0.0 {
            -1_i8
        } else {
            1_i8
        };
        if volume_sign != shell_target.get(&r).copied().unwrap_or(1) {
            flip[t] = !flip[t];
        }
    }
    // (f) Make stored normals continuous and material-outward, matching the
    // FINAL winding (into the cavity for a void shell).
    // Reference per raw vertex = sum of the post-flip geometric normals of the
    // triangles touching it; flip any stored normal that opposes it. This turns
    // the folded region's flipped analytic normals continuous (and outward).
    let mut vnref = vec![[0.0f64; 3]; vcount];
    for t in 0..tri_count {
        let mut g = tri_geometric_normal(&mesh.positions, &mesh.indices, t);
        if flip[t] {
            g = [-g[0], -g[1], -g[2]];
        }
        for k in 0..3 {
            let v = mesh.indices[3 * t + k] as usize;
            vnref[v][0] += g[0];
            vnref[v][1] += g[1];
            vnref[v][2] += g[2];
        }
    }
    for v in 0..vcount {
        let r = vnref[v];
        let dot = mesh.normals[3 * v] * r[0]
            + mesh.normals[3 * v + 1] * r[1]
            + mesh.normals[3 * v + 2] * r[2];
        if dot < 0.0 {
            mesh.normals[3 * v] = -mesh.normals[3 * v];
            mesh.normals[3 * v + 1] = -mesh.normals[3 * v + 1];
            mesh.normals[3 * v + 2] = -mesh.normals[3 * v + 2];
        }
    }

    // (e) Apply the winding flips (swap 2nd & 3rd index of each flipped triangle).
    for t in 0..tri_count {
        if flip[t] {
            mesh.indices.swap(3 * t + 1, 3 * t + 2);
        }
    }

    if let Some(start) = _timer {
        let nflip = flip.iter().filter(|&&f| f).count();
        eprintln!(
            "orient_mesh_coherently: {tri_count} tris, {vcount} verts, {ncomp} components, {nflip} flipped in {:?}",
            start.elapsed()
        );
    }
    Ok(())
}

