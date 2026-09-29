use super::*;

/// A detected MIXED-convexity trihedral corner (Golovanov §6.9.7): one
/// CONCAVE incident edge between walls A and B (its fillet cylinder Q sits in
/// the void, axis offset +r from both walls), two CONVEX edges where A and B
/// meet the third wall (the "cap").  Supported when Q's axis is parallel to
/// the cap's outward normal — then the corner closure is an EXACT torus
/// sector (rolling a radius-r ball along the convex rim: its centre keeps
/// distance r from the cap and 2r from Q's axis, a circle).
struct MixedConcaveCorner {
    /// Outward normal of the cap wall.
    cap_normal: Vec3,
    /// A point on the concave fillet's axis.
    axis_point: Vec3,
    /// Unit axis of the concave fillet, oriented along `cap_normal`.
    axis_dir: Vec3,
    /// Tangency vertex of Q with the wall-A/cap convex fillet cylinder
    /// (`(vertex id, position)`); lies ON wall A at distance r from Q's axis.
    v_a: (u64, Vec3),
    v_b: (u64, Vec3),
    /// Cap-side end of the A-junction arc (on the cap plane, 2r from Q axis).
    w_a: (u64, Vec3),
    w_b: (u64, Vec3),
    /// Closest point of the wall-A convex fillet's axis to Q's axis.
    a_axis_closest: Vec3,
    b_axis_closest: Vec3,
}

/// Detect the supported mixed-convexity trihedral corner configuration from
/// the three OUTWARD wall normals and the already-filleted solid: exactly one
/// of the three wall pairs carries a CONCAVE blend.  The proof of the
/// configuration is the existence of the four tangency vertices the
/// sequential fillets must have left behind (Vᵢ = Q∧cylᵢ tangency on wall i,
/// Wᵢ = the cap end of the i-junction arc); a corner whose fillets did not
/// produce them (wrong convexity mix, tilted concave axis, non-composing
/// fillet order) is refused with the collected reasons.
fn detect_mixed_concave_corner(
    solid: &BrepSolid,
    corner: Vec3,
    radius: f64,
    normals: &[Vec3],
) -> Result<MixedConcaveCorner, KernelRefusal> {
    if normals.len() != 3 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "wall_count",
            format!(
                "mixed corner supports exactly 3 planar walls, found {}",
                normals.len()
            ),
        ));
    }
    let find_vertex = |ideal: Vec3| -> Option<(u64, Vec3)> {
        solid
            .vertices
            .iter()
            .find(|v| v.point.sub(ideal).length() < 1e-6)
            .map(|v| (v.id, v.point))
    };
    let mut reasons: Vec<String> = Vec::new();
    for k in 0..3 {
        let (i, j) = match k {
            0 => (1, 2),
            1 => (0, 2),
            _ => (0, 1),
        };
        let (na, nb, nc) = (normals[i], normals[j], normals[k]);
        // Concave fillet axis: the CONCAVE sign, `+r` along both outward
        // normals — offset into the void from both walls. Their intersection
        // LINE is the locus, in closed form; `offset_plane_pair` is the shared
        // seam and `line.direction` is `n_a × n_b` normalized, i.e. the axis
        // this candidate is about (audit §11).
        //
        // The solve now runs BEFORE the perpendicularity gate below, where the
        // hand-written version ran after it. The only configuration that can
        // tell the difference is one whose 3×3 system is singular to
        // `solve_small`'s `1e-13·scale` floor while `|n_a × n_b|` still clears
        // `normalized`'s `1e-12` — a band no fixture occupies — and it would
        // change only which reason string is collected, never a result.
        let line = match offset_plane_pair(corner, na, radius, nb, radius) {
            Ok(line) => line,
            Err(OffsetPairDegeneracy::ParallelPlanes) => {
                reasons.push(format!("walls {i}/{j} are parallel"));
                continue;
            }
            Err(_) => {
                reasons.push(format!("walls {i}/{j} tangency system is singular"));
                continue;
            }
        };
        let axis = line.direction;
        // The exact torus-sector closure needs the concave blend's axis
        // parallel to the cap normal (concave edge perpendicular to the cap).
        if axis.dot(nc).abs() < 1.0 - 1e-9 {
            reasons.push(format!(
                "concave-axis candidate between walls {i}/{j} is not perpendicular \
                 to the third wall (|cos| = {:.6})",
                axis.dot(nc).abs()
            ));
            continue;
        }
        let axis_dir = if axis.dot(nc) > 0.0 {
            axis
        } else {
            axis.scale(-1.0)
        };
        // The pre-slice solve put `axis_dir` — the possibly NEGATED direction —
        // in the anchoring row; the shared form always uses `+(n_a × n_b)`. The
        // two are bit-identical because the row's right-hand side is zero, which
        // `bit_identical_plane_pair_under_a_negated_direction_row` pins and this
        // diagnostic measures across the corpus.
        offset_pair_diag::plane_pair(
            "mixed_concave::concave_axis",
            corner,
            na,
            radius,
            nb,
            radius,
            axis_dir,
            Some(&line),
        );
        let q0 = line.point;
        // Convex fillet axis for the cap edge of wall `w`: the same construction
        // with the CONVEX sign, `−r` — offset into the material from both the
        // wall and the cap. The caller discards this error text, so the
        // degeneracy travels as a value.
        let side = |nw: Vec3| -> Result<(Vec3, Vec3), OffsetPairDegeneracy> {
            let line = offset_plane_pair(corner, nw, -radius, nc, -radius)?;
            offset_pair_diag::plane_pair(
                "mixed_concave::cap_fillet_axis",
                corner,
                nw,
                -radius,
                nc,
                -radius,
                line.direction,
                Some(&line),
            );
            Ok((line.point, line.direction))
        };
        let (Ok((pa, ta)), Ok((pb, tb))) = (side(na), side(nb)) else {
            reasons.push(format!("cap-edge axis solve failed for walls {i}/{j}"));
            continue;
        };
        let tangency = |pw: Vec3, tw: Vec3| -> Option<(Vec3, Vec3, Vec3)> {
            let (qc, ac) = line_closest_points(q0, axis_dir, pw, tw)?;
            // Q (radius r) and the convex cylinder (radius r) must touch:
            // axis separation exactly 2r.
            if (ac.sub(qc).length() - 2.0 * radius).abs() > 1e-6 * (1.0 + radius) {
                return None;
            }
            let v = qc.add(ac).scale(0.5);
            let w = ac.add(nc.scale(radius));
            Some((v, w, ac))
        };
        let (Some((va, wa, aca)), Some((vb, wb, acb))) = (tangency(pa, ta), tangency(pb, tb))
        else {
            reasons.push(format!(
                "concave blend between walls {i}/{j} is not tangent to both cap-edge fillets"
            ));
            continue;
        };
        let (Some(v_a), Some(v_b), Some(w_a), Some(w_b)) = (
            find_vertex(va),
            find_vertex(vb),
            find_vertex(wa),
            find_vertex(wb),
        ) else {
            reasons.push(format!(
                "no fillet tangency vertices for a concave edge between walls {i}/{j} \
                 (expected near {va:?}, {vb:?}, {wa:?}, {wb:?}; fillet the concave edge \
                 before the convex cap edges)"
            ));
            continue;
        };
        return Ok(MixedConcaveCorner {
            cap_normal: nc,
            axis_point: q0,
            axis_dir,
            v_a,
            v_b,
            w_a,
            w_b,
            a_axis_closest: aca,
            b_axis_closest: acb,
        });
    }
    Err(KernelRefusal::unsupported(
        KernelStage::Classify,
        "mixed_concave_config",
        format!(
            "no supported single-concave-edge trihedral configuration: {}",
            reasons.join("; ")
        ),
    ))
}

/// Golovanov §6.9.7 MIXED-convexity trihedral vertex blend: round a corner
/// where ONE of the three incident filleted edges is CONCAVE (its blend ADDS
/// material) and the other two are convex.  Unlike the all-convex corner
/// there is NO corner sphere tangent to all three blend strips along circles
/// (a ball centred on the concave cylinder's axis only touches the two convex
/// cylinders at single points), so the closure is the rolling-ball sweep
/// along the concave blend's rim — an EXACT TORUS sector (major radius 2r,
/// minor r, axis = the concave fillet's axis) when that axis is perpendicular
/// to the third wall.  Pure topology surgery, no booleans:
///
///   * the concave fillet Q is trimmed back to its tangent arc V_A→V_B (the
///     torus' inner equator, r from the axis, r below the cap),
///   * the cap is trimmed back to the torus' outer contact arc W_A→W_B
///     (2r from the axis, on the cap plane),
///   * the two convex fillets ALREADY end on the junction profile arcs
///     Vᵢ→Wᵢ (the sequential fillet trims them there), which become the
///     torus sector's side boundaries,
///   * the leftover sharp notch faces between those curves are deleted and
///     the torus patch is sewn on the four arcs.
///
/// Requires the concave edge to have been filleted BEFORE its two convex cap
/// edges (the composing order — the reverse order does not produce a clean
/// junction topology); refused otherwise via the tangency-vertex probe.
pub(super) fn round_mixed_concave_corner(
    solid: &BrepSolid,
    corner: Vec3,
    radius: f64,
    normals: &[Vec3],
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

    let mc = detect_mixed_concave_corner(solid, corner, radius, normals)?;
    let mut result = solid.clone();

    let vpoint: HashMap<u64, Vec3> = result.vertices.iter().map(|v| (v.id, v.point)).collect();
    let edge_ends: HashMap<u64, (u64, u64)> = result
        .edges
        .iter()
        .map(|e| (e.id, (e.start_vertex_id, e.end_vertex_id)))
        .collect();
    let coedge_ends_3d = |co: &CoedgeRecord| -> Option<(Vec3, Vec3)> {
        let (s, e) = edge_ends.get(&co.edge_id)?;
        Some((*vpoint.get(s)?, *vpoint.get(e)?))
    };
    let plane_tol = 1e-6 * radius.max(1.0);

    // Axis reference points at the two boundary levels: q_v on the axis at
    // the V (inner-equator) level, q_w at the cap level.
    let q_v = mc.axis_point.add(
        mc.axis_dir
            .scale(mc.v_a.1.sub(mc.axis_point).dot(mc.axis_dir)),
    );
    let q_w = mc.axis_point.add(
        mc.axis_dir
            .scale(mc.w_a.1.sub(mc.axis_point).dot(mc.axis_dir)),
    );

    // ---- 1. Locate the two existing junction arcs Vᵢ→Wᵢ (the convex fillet
    //         end profiles; either stored direction), verified by midpoint.
    let arc_a_id = locate_mixed_junction_arc(
        &result,
        "round_mixed_concave_corner",
        mc.v_a.0,
        mc.w_a.0,
        mc.a_axis_closest,
        radius,
    )?;
    let arc_b_id = locate_mixed_junction_arc(
        &result,
        "round_mixed_concave_corner",
        mc.v_b.0,
        mc.w_b.0,
        mc.b_axis_closest,
        radius,
    )?;

    // ---- 2. The concave fillet face Q: the unique non-planar face incident
    //         to BOTH tangency vertices V_A and V_B.
    let touches = |face: &FaceRecord, vid: u64| -> bool {
        face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
            edge_ends
                .get(&co.edge_id)
                .map(|&(s, e)| s == vid || e == vid)
                .unwrap_or(false)
        })
    };
    let is_planar = |face: &FaceRecord| -> bool {
        matches!(
            face.surface.analytic(),
            Some(crate::AnalyticSurface::Plane { .. })
        )
    };
    let mut q_face_id: Option<u64> = None;
    let mut cyl_a_face_id: Option<u64> = None;
    let mut cyl_b_face_id: Option<u64> = None;
    let mut q_shell = 0usize;
    for (si, shell) in result.shells.iter().enumerate() {
        for face in &shell.faces {
            if is_planar(face) {
                continue;
            }
            if touches(face, mc.v_a.0) && touches(face, mc.v_b.0) {
                if q_face_id.is_some() {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "concave_candidates",
                        "round_mixed_concave_corner: multiple concave-fillet candidates",
                    ));
                }
                q_face_id = Some(face.id);
                q_shell = si;
            }
            let uses_arc = |arc: u64| {
                face.loops
                    .iter()
                    .flat_map(|l| &l.coedges)
                    .any(|co| co.edge_id == arc)
            };
            if uses_arc(arc_a_id) && !(touches(face, mc.v_a.0) && touches(face, mc.v_b.0)) {
                cyl_a_face_id = Some(face.id);
            }
            if uses_arc(arc_b_id) && !(touches(face, mc.v_a.0) && touches(face, mc.v_b.0)) {
                cyl_b_face_id = Some(face.id);
            }
        }
    }
    let q_face_id = q_face_id.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "concave_face",
            "round_mixed_concave_corner: concave fillet face not found (no non-planar face \
         touches both tangency vertices)",
        )
    })?;
    let cyl_a_face_id = cyl_a_face_id.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "cap_fillet_a",
            "round_mixed_concave_corner: wall-A cap fillet does not end on its junction arc",
        )
    })?;
    let cyl_b_face_id = cyl_b_face_id.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "cap_fillet_b",
            "round_mixed_concave_corner: wall-B cap fillet does not end on its junction arc",
        )
    })?;

    // The convex fillets' surviving coedge senses on the junction arcs (the
    // torus patch must use the opposite sense).
    let cyl_a_forward = coedge_sense_on(
        &result,
        "round_mixed_concave_corner",
        cyl_a_face_id,
        arc_a_id,
    )?;
    let cyl_b_forward = coedge_sense_on(
        &result,
        "round_mixed_concave_corner",
        cyl_b_face_id,
        arc_b_id,
    )?;

    // id allocator across the whole solid's shared id space.
    let mut next_id = crate::blend::edge::fresh_id_source(&result);

    // ---- 3. Trim Q back to the inner-equator arc: corner side = past the
    //         V level toward the cap (along the axis).
    let q_corner_side = |p: Vec3| -> bool { p.sub(q_v).dot(mc.axis_dir) >= -plane_tol };
    let mut new_loops: HashMap<u64, Vec<CoedgeRecord>> = HashMap::default();
    let mut fresh_edges: Vec<EdgeRecord> = Vec::new();
    let mut fresh_w_arc: Option<EdgeRecord> = None;
    let fresh_v_arc: EdgeRecord = {
        let face = result
            .shells
            .iter()
            .flat_map(|s| &s.faces)
            .find(|f| f.id == q_face_id)
            .unwrap();
        if face.loops.len() != 1 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Fragment,
                "multi_loop",
                format!(
                    "round_mixed_concave_corner: concave fillet face {} has {} loops",
                    face.id,
                    face.loops.len()
                ),
            ));
        }
        let is_corner: Vec<bool> = face.loops[0]
            .coedges
            .iter()
            .map(|co| {
                coedge_ends_3d(co)
                    .map(|(a, b)| q_corner_side(a) && q_corner_side(b))
                    .unwrap_or(false)
            })
            .collect();
        let (rebuilt, edge) = rebuild_mixed_corner_run(
            "round_mixed_concave_corner",
            face,
            &is_corner,
            [mc.v_a.0, mc.v_b.0],
            q_v,
            radius,
            &vpoint,
            &edge_ends,
            &mut next_id,
        )?;
        new_loops.insert(face.id, rebuilt);
        fresh_edges.push(edge.clone());
        edge
    };

    // ---- 4. Trim the cap back to the outer contact arc (2r from the axis).
    //         Cap faces = planar faces on the cap plane through the corner;
    //         all-corner ones are leftover notch junk (deleted below), the
    //         one carrying a partial run gets rebuilt.
    let cap_corner_side = |p: Vec3| -> bool {
        let radial = p
            .sub(q_w)
            .sub(mc.axis_dir.scale(p.sub(q_w).dot(mc.axis_dir)));
        radial.length() <= 2.0 * radius + plane_tol
    };
    let on_cap_plane = |face: &FaceRecord| -> bool {
        let Some(crate::AnalyticSurface::Plane {
            origin,
            u_dir,
            v_dir,
            ..
        }) = face.surface.analytic()
        else {
            return false;
        };
        let Ok(n) = u_dir.cross(*v_dir).normalized() else {
            return false;
        };
        n.dot(mc.cap_normal).abs() > 1.0 - 1e-6
            && mc.cap_normal.dot(corner.sub(*origin)).abs() < 1e-6
    };
    let mut cap_rebuilt = false;
    for shell in &result.shells {
        for face in &shell.faces {
            if !on_cap_plane(face) || face.loops.len() != 1 {
                continue;
            }
            let is_corner: Vec<bool> = face.loops[0]
                .coedges
                .iter()
                .map(|co| {
                    coedge_ends_3d(co)
                        .map(|(a, b)| cap_corner_side(a) && cap_corner_side(b))
                        .unwrap_or(false)
                })
                .collect();
            if !is_corner.iter().any(|&c| c) || is_corner.iter().all(|&c| c) {
                continue; // untouched cap region, or all-junk face handled below
            }
            if cap_rebuilt {
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "cap_faces",
                    "round_mixed_concave_corner: multiple cap faces carry a corner run",
                ));
            }
            let (rebuilt, edge) = rebuild_mixed_corner_run(
                "round_mixed_concave_corner",
                face,
                &is_corner,
                [mc.w_a.0, mc.w_b.0],
                q_w,
                2.0 * radius,
                &vpoint,
                &edge_ends,
                &mut next_id,
            )?;
            new_loops.insert(face.id, rebuilt);
            fresh_w_arc = Some(edge.clone());
            fresh_edges.push(edge);
            cap_rebuilt = true;
        }
    }
    let fresh_w_arc = fresh_w_arc.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "cap_run",
            "round_mixed_concave_corner: no cap face carries the corner run",
        )
    })?;

    // ---- 5. Leftover notch junk: planar faces whose EVERY loop vertex lies
    //         within the corner ball (2r about the cap axis point), connected
    //         to the corner fillet complex across shared edges (flood fill —
    //         the same scheme as the convex path's cap removal).
    let junk_scope = 2.0 * radius + 1e-6 * (1.0 + radius);
    let mut edge_faces: HashMap<u64, Vec<u64>> = HashMap::default();
    for shell in &result.shells {
        for face in &shell.faces {
            for lp in &face.loops {
                for co in &lp.coedges {
                    edge_faces.entry(co.edge_id).or_default().push(face.id);
                }
            }
        }
    }
    let candidate_junk = |face: &FaceRecord| -> bool {
        is_planar(face)
            && face.loops.iter().flat_map(|l| &l.coedges).all(|co| {
                coedge_ends_3d(co)
                    .map(|(a, b)| {
                        a.sub(q_w).length() <= junk_scope && b.sub(q_w).length() <= junk_scope
                    })
                    .unwrap_or(false)
            })
    };
    let seed: HashSet<u64> = [q_face_id, cyl_a_face_id, cyl_b_face_id]
        .into_iter()
        .collect();
    let mut junk_face_ids: HashSet<u64> = HashSet::default();
    loop {
        let mut added = false;
        for shell in &result.shells {
            for face in &shell.faces {
                if junk_face_ids.contains(&face.id) || !candidate_junk(face) {
                    continue;
                }
                let borders = face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
                    edge_faces
                        .get(&co.edge_id)
                        .map(|fs| {
                            fs.iter()
                                .any(|fid| seed.contains(fid) || junk_face_ids.contains(fid))
                        })
                        .unwrap_or(false)
                });
                if borders {
                    junk_face_ids.insert(face.id);
                    added = true;
                }
            }
        }
        if !added {
            break;
        }
    }

    // ---- 6. The torus-sector patch.  Generatrix = the A-junction arc's own
    //         curve (so the u=0 boundary reproduces it EXACTLY); revolved
    //         about the concave fillet's axis by the sector angle to the
    //         B-junction plane.
    let arc_a = result
        .edges
        .iter()
        .find(|e| e.id == arc_a_id)
        .unwrap()
        .clone();
    let arc_b = result
        .edges
        .iter()
        .find(|e| e.id == arc_b_id)
        .unwrap()
        .clone();
    let patch_face = build_mixed_sector_patch(
        "round_mixed_concave_corner",
        &arc_a,
        &arc_b,
        &fresh_v_arc,
        &fresh_w_arc,
        cyl_a_forward,
        cyl_b_forward,
        mc.v_a,
        mc.v_b,
        mc.w_a,
        mc.w_b,
        mc.axis_point,
        mc.axis_dir,
        mc.a_axis_closest,
        mc.b_axis_closest,
        radius,
        name,
        &mut next_id,
    )?;

    // ---- 7. Splice: fresh edges, rebuilt loops, junk removal, patch face.
    result.edges.extend(fresh_edges);
    for (si, shell) in result.shells.iter_mut().enumerate() {
        shell.faces.retain(|f| !junk_face_ids.contains(&f.id));
        for face in shell.faces.iter_mut() {
            if let Some(coedges) = new_loops.get(&face.id) {
                face.loops[0].coedges = coedges.clone();
            }
        }
        if si == q_shell {
            shell.faces.push(patch_face.clone());
        }
    }

    // Remove now-orphaned edges and vertices.
    let mut edge_uses: HashMap<u64, usize> = HashMap::default();
    for shell in &result.shells {
        for face in &shell.faces {
            for lp in &face.loops {
                for co in &lp.coedges {
                    *edge_uses.entry(co.edge_id).or_default() += 1;
                }
            }
        }
    }
    result
        .edges
        .retain(|e| edge_uses.get(&e.id).copied().unwrap_or(0) > 0);
    let referenced: HashSet<u64> = result
        .edges
        .iter()
        .flat_map(|e| [e.start_vertex_id, e.end_vertex_id])
        .collect();
    result.vertices.retain(|v| referenced.contains(&v.id));

    // ---- 8. Validate; surface the issues to the caller if any.
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!("round_mixed_concave_corner: {issues:?}"),
        ));
    }
    Ok(result)
}

/// Close the re-entrant corner of TWO convex rim fillets with the exact
/// horn-torus limit of the mixed-corner rolling-ball patch.
///
/// A concave profile vertex has two selected cap edges plus one unselected,
/// concave wall-wall edge.  The two per-edge cutter volumes only TOUCH at
/// that wall-wall edge, so boolean intersection alone preserves one cutter's
/// triangular end cap.  The rolling-ball centre instead follows a radius-r
/// arc around the sharp wall-wall edge.  Sweeping the radius-r section along
/// that arc is a horn torus (major radius r, minor radius r): its inner rim
/// collapses to the point `corner - r * cap_normal`, while its outer rim is a
/// radius-r contact arc on the cap.
pub(crate) fn round_concave_chain_corner(
    solid: &BrepSolid,
    source: &BrepSolid,
    corner: Vec3,
    selected_edges: [u64; 2],
    radius: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

    let ctx = "round_concave_chain_corner";
    let faces_of = |edge_id: u64| -> Vec<&FaceRecord> {
        source
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| {
                face.loops
                    .iter()
                    .flat_map(|lp| &lp.coedges)
                    .any(|co| co.edge_id == edge_id)
            })
            .collect()
    };
    let first = faces_of(selected_edges[0]);
    let second = faces_of(selected_edges[1]);
    if first.len() != 2 || second.len() != 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "manifold_edges",
            format!("{ctx}: selected corner edges are not manifold"),
        ));
    }
    let cap = first
        .iter()
        .find(|face| second.iter().any(|other| other.id == face.id))
        .copied()
        .ok_or_else(|| {
            KernelRefusal::input(
                KernelStage::Collect,
                "common_cap",
                format!("{ctx}: selected corner edges have no common cap face"),
            )
        })?;
    let wall_a = first
        .iter()
        .find(|face| face.id != cap.id)
        .copied()
        .unwrap();
    let wall_b = second
        .iter()
        .find(|face| face.id != cap.id)
        .copied()
        .unwrap();
    let outward = |face: &FaceRecord| -> Result<Vec3, KernelRefusal> {
        if !matches!(
            face.surface.analytic(),
            Some(crate::AnalyticSurface::Plane { .. })
        ) {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "planar_rim",
                format!("{ctx}: horn-torus closure requires planar cap and walls"),
            ));
        }
        let projection = crate::project_point_to_surface(&face.surface, corner)
            .or_refuse(KernelStage::Classify, "project")?;
        let normal = face
            .surface
            .normal(projection.u, projection.v)
            .or_refuse(KernelStage::Classify, "normal")?;
        Ok(if face.same_sense {
            normal
        } else {
            normal.scale(-1.0)
        })
    };
    let (nc, na, nb) = (outward(cap)?, outward(wall_a)?, outward(wall_b)?);
    if nc.dot(na).abs() > 1e-6 || nc.dot(nb).abs() > 1e-6 || na.dot(nb).abs() > 1e-6 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "orthogonal_rim",
            format!("{ctx}: only orthogonal planar re-entrant rims are supported"),
        ));
    }

    let q = corner.sub(nc.scale(radius));
    let w_a = corner.sub(na.scale(radius));
    let w_b = corner.sub(nb.scale(radius));
    let centre_a = q.sub(na.scale(radius));
    let centre_b = q.sub(nb.scale(radius));
    let find_vertex = |point: Vec3| -> Result<(u64, Vec3), KernelRefusal> {
        solid
            .vertices
            .iter()
            .find(|vertex| vertex.point.sub(point).length() < 1e-6 * (1.0 + radius))
            .map(|vertex| (vertex.id, vertex.point))
            .ok_or_else(|| {
                KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "corner_vertex",
                    format!("{ctx}: expected corner vertex near {point:?}"),
                )
            })
    };
    let qv = find_vertex(q)?;
    let wav = find_vertex(w_a)?;
    let wbv = find_vertex(w_b)?;
    let arc_a_id = locate_mixed_junction_arc(solid, ctx, qv.0, wav.0, centre_a, radius)?;
    let arc_b_id = locate_mixed_junction_arc(solid, ctx, qv.0, wbv.0, centre_b, radius)?;

    let is_planar = |face: &FaceRecord| {
        matches!(
            face.surface.analytic(),
            Some(crate::AnalyticSurface::Plane { .. })
        )
    };
    let uses = |face: &FaceRecord, edge_id: u64| {
        face.loops
            .iter()
            .flat_map(|lp| &lp.coedges)
            .any(|co| co.edge_id == edge_id)
    };
    let blend_for = |edge_id: u64| -> Result<(u64, usize), KernelRefusal> {
        let matches = solid
            .shells
            .iter()
            .enumerate()
            .flat_map(|(si, shell)| shell.faces.iter().map(move |face| (face, si)))
            .filter(|(face, _)| !is_planar(face) && uses(face, edge_id))
            .map(|(face, si)| (face.id, si))
            .collect::<Vec<_>>();
        if matches.len() != 1 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "junction_blend",
                format!(
                    "{ctx}: expected one blend face on junction arc {edge_id}, found {}",
                    matches.len()
                ),
            ));
        }
        Ok(matches[0])
    };
    let (blend_a, patch_shell) = blend_for(arc_a_id)?;
    let (blend_b, _) = blend_for(arc_b_id)?;
    let sense_a = coedge_sense_on(solid, ctx, blend_a, arc_a_id)?;
    let sense_b = coedge_sense_on(solid, ctx, blend_b, arc_b_id)?;

    let mut result = solid.clone();
    let vpoint: HashMap<u64, Vec3> = result.vertices.iter().map(|v| (v.id, v.point)).collect();
    let edge_ends: HashMap<u64, (u64, u64)> = result
        .edges
        .iter()
        .map(|edge| (edge.id, (edge.start_vertex_id, edge.end_vertex_id)))
        .collect();
    let point_ends = |co: &CoedgeRecord| -> Option<(Vec3, Vec3)> {
        let (a, b) = edge_ends.get(&co.edge_id)?;
        Some((*vpoint.get(a)?, *vpoint.get(b)?))
    };

    let mut next_id = crate::blend::edge::fresh_id_source(&result);

    // Replace the little W_a -> sharp corner -> W_b run on the cap with its
    // exact rolling-ball contact arc.
    let cap_face = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| {
            if !is_planar(face) || face.loops.len() != 1 {
                return false;
            }
            let Ok(projection) = crate::project_point_to_surface(&face.surface, corner) else {
                return false;
            };
            let Ok(normal) = face.surface.normal(projection.u, projection.v) else {
                return false;
            };
            projection.distance < 1e-6 * (1.0 + radius) && normal.dot(nc).abs() > 1.0 - 1e-6
        })
        .ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Classify,
                "cap_face",
                format!("{ctx}: trimmed cap face not found"),
            )
        })?;
    let local = 1.01 * radius + 1e-6;
    let is_corner = cap_face.loops[0]
        .coedges
        .iter()
        .map(|co| {
            point_ends(co)
                .map(|(a, b)| a.sub(corner).length() <= local && b.sub(corner).length() <= local)
                .unwrap_or(false)
        })
        .collect::<Vec<_>>();
    let (rebuilt_cap, fresh_w_arc) = rebuild_mixed_corner_run(
        ctx,
        cap_face,
        &is_corner,
        [wav.0, wbv.0],
        corner,
        radius,
        &vpoint,
        &edge_ends,
        &mut next_id,
    )?;
    let cap_face_id = cap_face.id;

    let degenerate = EdgeRecord {
        id: next_id(),
        curve: crate::make_line(qv.1, qv.1).or_refuse(KernelStage::Fragment, "make_line")?,
        t0: 0.0,
        t1: 1.0,
        start_vertex_id: qv.0,
        end_vertex_id: qv.0,
        degenerate: true,
        name: None,
    };
    let arc_a = result
        .edges
        .iter()
        .find(|edge| edge.id == arc_a_id)
        .unwrap()
        .clone();
    let arc_b = result
        .edges
        .iter()
        .find(|edge| edge.id == arc_b_id)
        .unwrap()
        .clone();
    let axis_dir = nc.scale(-1.0);
    let patch = build_mixed_sector_patch(
        ctx,
        &arc_a,
        &arc_b,
        &degenerate,
        &fresh_w_arc,
        sense_a,
        sense_b,
        qv,
        qv,
        wav,
        wbv,
        corner,
        axis_dir,
        centre_a,
        centre_b,
        radius,
        name,
        &mut next_id,
    )?;

    // The only other users of the junction arcs are the planar triangular
    // cutter end-caps.  They are precisely the artefact replaced by the torus.
    let junk: HashSet<u64> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| {
            is_planar(face)
                && face.id != cap_face_id
                && (uses(face, arc_a_id) || uses(face, arc_b_id))
        })
        .map(|face| face.id)
        .collect();
    if junk.is_empty() {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "cutter_end_cap",
            format!("{ctx}: no planar cutter end-cap found"),
        ));
    }

    result.edges.push(fresh_w_arc);
    result.edges.push(degenerate);
    for (si, shell) in result.shells.iter_mut().enumerate() {
        shell.faces.retain(|face| !junk.contains(&face.id));
        if let Some(face) = shell.faces.iter_mut().find(|face| face.id == cap_face_id) {
            face.loops[0].coedges = rebuilt_cap.clone();
        }
        if si == patch_shell {
            shell.faces.push(patch.clone());
        }
    }

    let used_edges: HashSet<u64> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|lp| &lp.coedges)
        .map(|co| co.edge_id)
        .collect();
    result.edges.retain(|edge| used_edges.contains(&edge.id));
    let used_vertices: HashSet<u64> = result
        .edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    result
        .vertices
        .retain(|vertex| used_vertices.contains(&vertex.id));

    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!("{ctx}: {issues:?}"),
        ));
    }
    Ok(result)
}
