use super::*;

/// A detected MIXED-convexity CURVED-WALL trihedral corner (Golovanov
/// §6.9.7): the concave incident edge runs between a PLANAR wall A and a
/// CYLINDRICAL wall B (a boss/bore wall); the two convex edges lie on the
/// planar cap.  Because both the wall plane and the cylinder are extruded
/// along the cap normal, the concave blend's centre path — the intersection
/// of the r-offset wall plane with the (R±r)-offset cylinder — is a straight
/// LINE parallel to the cap normal, so the planar concave-vertex closure
/// carries over: rolling a radius-r ball along the concave blend's rim keeps
/// its centre at distance r from the cap and 2r from that axis (a circle),
/// and the closure is an EXACT torus sector.  The one geometric difference
/// is on the B side, where the cap fillet is a TORUS about the cylinder
/// axis: its junction with the concave blend is its minor profile circle at
/// the station where its centre circle passes closest to the concave blend's
/// axis — centre-circle radius R−r (boss) / R+r (bore) against the blend
/// axis at R+r / R−r, separation exactly 2r, so the tangency holds by
/// construction.
pub(in crate::blend) struct MixedCurvedCorner {
    /// Outward normal of the planar cap.
    cap_normal: Vec3,
    /// A point on the concave blend's axis.
    axis_point: Vec3,
    /// Unit axis of the concave blend, oriented along `cap_normal`.
    axis_dir: Vec3,
    /// Tangency vertex of the concave blend with the wall-A cap fillet
    /// (`(vertex id, position)`); lies ON wall A, r below the cap.
    v_a: (u64, Vec3),
    /// Tangency vertex with the cylinder-cap rim fillet; ON the cylinder
    /// wall, r below the cap.
    v_b: (u64, Vec3),
    /// Cap-side end of the B-junction arc (on the cap plane, 2r from the
    /// concave blend's axis).
    w_b: (u64, Vec3),
    /// Ideal cap-side end of the A-junction arc.  NOT a vertex yet: the
    /// wall-A cap fillet composes LAST and overshoots the corner with an end
    /// bulkhead (its open-blend end trim finds only the TANGENT concave
    /// blend ahead, not a transverse face), so the surgery creates W_A by
    /// splitting the fillet's cap-contact edge here.
    w_a_point: Vec3,
    /// Centre of the A-junction arc = closest point of the wall-A cap
    /// fillet's axis to the concave blend's axis.
    a_axis_closest: Vec3,
    /// Centre of the B-junction arc = the rim fillet's centre-circle point
    /// closest to the concave blend's axis.
    b_axis_closest: Vec3,
    /// Unit normal of the A station plane, pointing INTO the corner wedge.
    a_corner_dir: Vec3,
    /// Unit normal of the B station plane, pointing INTO the corner wedge.
    b_corner_dir: Vec3,
}

/// Detect the supported mixed-convexity CURVED-WALL corner configuration:
/// two planar walls + one cylindrical wall through the corner, the concave
/// edge between the plane and the cylinder.  Tries each planar wall as the
/// cap; the exact torus-sector closure requires the cylinder axis parallel
/// to the cap normal (and the wall plane parallel to the axis).  The
/// configuration is proven by the tangency vertices the sequential fillets
/// left behind: V_A, V_B and W_B must exist (W_A must NOT — see
/// [`MixedCurvedCorner::w_a_point`]).  Refused with the collected reasons
/// otherwise.
#[allow(clippy::too_many_arguments)]
pub(in crate::blend) fn detect_mixed_concave_curved_corner(
    solid: &BrepSolid,
    corner: Vec3,
    radius: f64,
    wall_normals: [Vec3; 2],
    cyl_axis_point: Vec3,
    cyl_axis_dir: Vec3,
    cyl_radius: f64,
    cyl_outward_away: bool,
) -> Result<MixedCurvedCorner, KernelRefusal> {
    let find_vertex = |ideal: Vec3| -> Option<(u64, Vec3)> {
        solid
            .vertices
            .iter()
            .find(|v| v.point.sub(ideal).length() < 1e-6)
            .map(|v| (v.id, v.point))
    };
    let tangency_tol = 1e-6 * (1.0 + radius);
    let mut reasons: Vec<String> = Vec::new();
    for k in 0..2 {
        let nc = wall_normals[k];
        let na = wall_normals[1 - k];
        // The exact revolution closure needs the cylinder axis parallel to
        // the cap normal — then (and only then) the concave blend's centre
        // path is a straight line.
        let cosine = cyl_axis_dir.dot(nc);
        if cosine.abs() < 1.0 - 1e-9 {
            reasons.push(format!(
                "with planar wall {k} as the cap, the cylindrical wall's axis is \
                 tilted against the cap normal (|cos| = {:.6}) — the concave \
                 blend's centre path is not a straight line and the closure is \
                 not an exact surface of revolution",
                cosine.abs()
            ));
            continue;
        }
        let axis_dir = if cosine > 0.0 {
            cyl_axis_dir
        } else {
            cyl_axis_dir.scale(-1.0)
        };
        if na.dot(axis_dir).abs() > 1e-9 {
            reasons.push(format!(
                "planar wall {} is not parallel to the cylindrical wall's axis",
                1 - k
            ));
            continue;
        }
        // Concave blend axis: the CONCAVE offset — `+r` along the wall's
        // outward normal, into the void — meets the like-offset cylinder in two
        // lines parallel to the axis. `axial_plane_meets_offset_cylinder` is the
        // shared closed form (audit §11); it is the `a = 1` lane, which is exact
        // precisely because the wall plane is parallel to the axis, and that is
        // guaranteed by the two gates above.
        let rho_q = match offset_cylinder_radius(cyl_radius, cyl_outward_away, radius) {
            Ok(rho) => rho,
            Err(_) => {
                reasons.push(format!(
                    "blend radius {radius} swallows the cylindrical bore wall (radius {cyl_radius})"
                ));
                continue;
            }
        };
        let axes = axial_plane_meets_offset_cylinder(
            corner,
            na,
            radius,
            cyl_axis_point,
            axis_dir,
            rho_q,
        );
        offset_pair_diag::axial_plane_cylinder(
            "mixed_curved::concave_axis",
            corner,
            na,
            radius,
            cyl_axis_point,
            axis_dir,
            rho_q,
            axes.as_ref().ok(),
        );
        let mut roots = match axes {
            Ok(axes) => axes,
            Err(OffsetPairDegeneracy::PlaneNormalAlongAxis) => {
                reasons.push("wall plane is degenerate against the cylinder axis".to_string());
                continue;
            }
            Err(_) => {
                reasons.push(format!(
                    "no radius-{radius} concave blend axis exists on the wall's offset \
                     plane (the blend cannot touch both the planar and the cylindrical wall)"
                ));
                continue;
            }
        };
        // ROOT SELECTION, and it stays here: the shared form returns both roots
        // in algebraic order because which one is the corner's blend axis is a
        // fact about this corner, not about the quadratic. Nearest the corner
        // first, then each is proven or rejected by its tangency vertices below.
        roots.sort_by(|s, t| {
            s.sub(corner)
                .length()
                .total_cmp(&t.sub(corner).length())
        });
        for q0 in roots {
            // Side A (planar wall × cap): a straight convex fillet whose
            // axis sits r inside both planes — the CONVEX `−r` offset of the
            // same plane-pair construction.
            let (pa, ta) = match offset_plane_pair(corner, na, -radius, nc, -radius) {
                Ok(line) => {
                    offset_pair_diag::plane_pair(
                        "mixed_curved::cap_fillet_axis",
                        corner,
                        na,
                        -radius,
                        nc,
                        -radius,
                        line.direction,
                        Some(&line),
                    );
                    (line.point, line.direction)
                }
                Err(OffsetPairDegeneracy::ParallelPlanes) => {
                    reasons.push("cap and wall planes are parallel".to_string());
                    continue;
                }
                Err(_) => {
                    reasons.push("wall/cap tangency system is singular".to_string());
                    continue;
                }
            };
            let Some((qc, ac)) = line_closest_points(q0, axis_dir, pa, ta) else {
                reasons
                    .push("wall-side cap fillet axis is parallel to the concave axis".to_string());
                continue;
            };
            if (ac.sub(qc).length() - 2.0 * radius).abs() > tangency_tol {
                reasons.push(format!(
                    "the concave blend is not tangent to the planar wall's cap fillet \
                     (axis separation {} vs {})",
                    ac.sub(qc).length(),
                    2.0 * radius
                ));
                continue;
            }
            let v_a_pt = qc.add(ac).scale(0.5);
            let w_a_pt = ac.add(nc.scale(radius));
            // Side B (cylinder × cap): the rim fillet's centre circle sits r
            // below the cap at radial R−r (boss) / R+r (bore); the concave
            // blend's axis lies on the R+r / R−r offset cylinder, so the
            // closest centre-circle point is at distance exactly 2r.
            let rho_rim = if cyl_outward_away {
                cyl_radius - radius
            } else {
                cyl_radius + radius
            };
            if !(rho_rim > 0.0) {
                reasons.push(format!(
                    "blend radius {radius} swallows the cylindrical boss wall \
                     (radius {cyl_radius})"
                ));
                continue;
            }
            let rel = q0.sub(cyl_axis_point);
            let Ok(u_hat) = rel.sub(axis_dir.scale(rel.dot(axis_dir))).normalized() else {
                reasons.push("concave blend axis coincides with the cylinder axis".to_string());
                continue;
            };
            let a_cap =
                cyl_axis_point.add(axis_dir.scale(corner.sub(cyl_axis_point).dot(axis_dir)));
            let c_b = a_cap.add(u_hat.scale(rho_rim)).sub(nc.scale(radius));
            let q_at = q0.add(axis_dir.scale(c_b.sub(q0).dot(axis_dir)));
            if (c_b.sub(q_at).length() - 2.0 * radius).abs() > tangency_tol {
                reasons.push(format!(
                    "the concave blend is not tangent to the cylinder-cap rim fillet \
                     (centre separation {} vs {})",
                    c_b.sub(q_at).length(),
                    2.0 * radius
                ));
                continue;
            }
            let v_b_pt = q_at.add(c_b).scale(0.5);
            let w_b_pt = c_b.add(nc.scale(radius));
            let (Some(v_a), Some(v_b), Some(w_b)) = (
                find_vertex(v_a_pt),
                find_vertex(v_b_pt),
                find_vertex(w_b_pt),
            ) else {
                reasons.push(format!(
                    "no fillet tangency vertices for a concave plane×cylinder edge \
                     (expected near {v_a_pt:?}, {v_b_pt:?}, {w_b_pt:?}; fillet the \
                     concave edge first, then the cylinder-cap rim, then the planar \
                     wall's cap edge)"
                ));
                continue;
            };
            let a_corner_dir = if ta.dot(v_b_pt.sub(ac)) >= 0.0 {
                ta
            } else {
                ta.scale(-1.0)
            };
            let Ok(b_normal) = u_hat.cross(axis_dir).normalized() else {
                reasons.push("degenerate B station plane".to_string());
                continue;
            };
            let b_corner_dir = if b_normal.dot(v_a_pt.sub(c_b)) >= 0.0 {
                b_normal
            } else {
                b_normal.scale(-1.0)
            };
            return Ok(MixedCurvedCorner {
                cap_normal: nc,
                axis_point: q0,
                axis_dir,
                v_a,
                v_b,
                w_b,
                w_a_point: w_a_pt,
                a_axis_closest: ac,
                b_axis_closest: c_b,
                a_corner_dir,
                b_corner_dir,
            });
        }
    }
    Err(KernelRefusal::unsupported(
        KernelStage::Classify,
        "mixed_curved_config",
        format!(
            "no supported mixed-convexity curved-wall (plane×cylinder concave edge) \
         configuration: {}",
            reasons.join("; ")
        ),
    ))
}

/// Golovanov §6.9.7 MIXED-convexity CURVED-WALL trihedral vertex blend:
/// round the corner where a CONCAVE plane×cylinder edge meets two convex cap
/// edges (the semicircular-boss fixture's corner at (10,4,10)).  The concave
/// blend Q is a radius-r cylinder about a straight axis parallel to the cap
/// normal (both walls are extruded along it — see [`MixedCurvedCorner`]), so
/// the planar concave-vertex closure carries over: the corner closes with an
/// EXACT torus sector (major radius 2r, minor r) about Q's axis, between the
/// A station plane (through the wall-A cap fillet's axis ⊥ to it) and the B
/// station plane (through the cylinder axis and Q's axis).  Differences from
/// the planar path, all on the A side:
///
///   * the B junction arc V_B→W_B already exists — the rim fillet's end
///     profile at its closest-approach station doubles as the junction —
///     but the wall-A cap fillet, composed LAST, overshoots the corner by r
///     and closes with a flat END BULKHEAD (its open-blend end trim finds
///     only the tangent Q ahead — a tangent surface is no transverse trim
///     partner), leaving no W_A vertex and no A junction arc;
///   * so the surgery SPLITS the fillet's cap-contact edge at the ideal W_A
///     and trims the overshoot back to a fresh junction arc V_A→W_A (the
///     fillet profile at the A station).  Rebuilding the cap loop back to
///     W_A also RE-COVERS the overshoot chamber (the groove cross-section ×
///     r past the A station outside the rolling-ball flare) that the
///     bulkhead lane had over-removed — pure loop surgery re-closes it as
///     interior material;
///   * the cap's corner run is bounded by the corner WEDGE between the two
///     station planes (apex on Q's axis), not by the 2r cylinder alone: the
///     overshoot trim boundary crosses the 2r circle (the bulkhead's cap
///     corner sits √5·r from the axis).
///
/// Pure topology surgery, no booleans; every fresh boundary is an exact
/// circular arc and the patch is an exact revolution of the A junction arc.
#[allow(clippy::too_many_arguments)]
pub(super) fn round_mixed_concave_curved_corner(
    solid: &BrepSolid,
    corner: Vec3,
    radius: f64,
    wall_normals: [Vec3; 2],
    cyl_axis_point: Vec3,
    cyl_axis_dir: Vec3,
    cyl_radius: f64,
    cyl_outward_away: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
    const CTX: &str = "round_mixed_concave_curved_corner";

    let mcc = detect_mixed_concave_curved_corner(
        solid,
        corner,
        radius,
        wall_normals,
        cyl_axis_point,
        cyl_axis_dir,
        cyl_radius,
        cyl_outward_away,
    )?;
    let mut result = solid.clone();
    let plane_tol = 1e-6 * radius.max(1.0);

    // Axis reference points at the two boundary levels: q_v at the V
    // (inner-equator) level, q_w at the cap level.
    let q_v = mcc.axis_point.add(
        mcc.axis_dir
            .scale(mcc.v_a.1.sub(mcc.axis_point).dot(mcc.axis_dir)),
    );
    let q_w = mcc.axis_point.add(
        mcc.axis_dir
            .scale(mcc.w_b.1.sub(mcc.axis_point).dot(mcc.axis_dir)),
    );

    // ---- 1. The B junction arc must already exist (the rim fillet was
    //         trimmed at its tangency station against Q — the composing
    //         order).
    let arc_b_id = locate_mixed_junction_arc(
        &result,
        CTX,
        mcc.v_b.0,
        mcc.w_b.0,
        mcc.b_axis_closest,
        radius,
    )?;

    // ---- 2. Identify the carrier faces: Q = the unique non-planar face
    //         touching BOTH V_A and V_B; the rim fillet = the non-planar
    //         face on the B junction arc; the wall-A cap fillet = the
    //         remaining non-planar face at V_A.  (The cylindrical WALL
    //         touches V_B only, and wall A is planar.)
    let edge_ends_pre: HashMap<u64, (u64, u64)> = result
        .edges
        .iter()
        .map(|e| (e.id, (e.start_vertex_id, e.end_vertex_id)))
        .collect();
    let touches = |face: &FaceRecord, vid: u64| -> bool {
        face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
            edge_ends_pre
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
            let at_v_a = touches(face, mcc.v_a.0);
            if at_v_a && touches(face, mcc.v_b.0) {
                if q_face_id.is_some() {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "concave_candidates",
                        format!("{CTX}: multiple concave-blend candidates"),
                    ));
                }
                q_face_id = Some(face.id);
                q_shell = si;
                continue;
            }
            let uses_arc_b = face
                .loops
                .iter()
                .flat_map(|l| &l.coedges)
                .any(|co| co.edge_id == arc_b_id);
            if uses_arc_b {
                cyl_b_face_id = Some(face.id);
            } else if at_v_a {
                if cyl_a_face_id.is_some() {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "cap_fillet_a",
                        format!("{CTX}: ambiguous wall-side cap fillet at the corner"),
                    ));
                }
                cyl_a_face_id = Some(face.id);
            }
        }
    }
    let q_face_id = q_face_id.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "concave_face",
            format!(
                "{CTX}: concave blend face not found (no non-planar face touches both \
             tangency vertices)"
            ),
        )
    })?;
    let cyl_b_face_id = cyl_b_face_id.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "cap_fillet_b",
            format!("{CTX}: cylinder-cap rim fillet does not end on its junction arc"),
        )
    })?;
    let cyl_a_face_id = cyl_a_face_id.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Classify,
            "cap_fillet_a_missing",
            format!("{CTX}: wall-side cap fillet not found at its tangency vertex"),
        )
    })?;
    let cyl_b_forward = coedge_sense_on(&result, CTX, cyl_b_face_id, arc_b_id)?;

    // id allocator across the whole solid's shared id space.
    let mut next_id = crate::blend::edge::fresh_id_source(&result);

    // ---- 3. A-side junction vertex W_A: split the wall-A cap fillet's
    //         cap-contact edge at the ideal point (the overshoot lane) —
    //         unless the corner already composed planar-style with a W_A
    //         vertex and junction arc, which are then reused directly.
    let existing_w_a = result
        .vertices
        .iter()
        .find(|v| v.point.sub(mcc.w_a_point).length() < 1e-6)
        .map(|v| v.id);
    let trim_a = existing_w_a.is_none();
    let w_a_id = if let Some(id) = existing_w_a {
        id
    } else {
        let a_face = result
            .shells
            .iter()
            .flat_map(|s| &s.faces)
            .find(|f| f.id == cyl_a_face_id)
            .unwrap();
        let mut split: Option<(u64, f64, u64)> = None;
        for co in a_face.loops.iter().flat_map(|l| &l.coedges) {
            let Some(edge) = result.edges.iter().find(|e| e.id == co.edge_id) else {
                continue;
            };
            let Ok(proj) = crate::project_point_to_curve(&edge.curve, mcc.w_a_point) else {
                continue;
            };
            let lo = edge.t0.min(edge.t1);
            let hi = edge.t0.max(edge.t1);
            let margin = 1e-6 * (hi - lo).max(1e-12);
            let t_at = proj.u;
            if t_at < lo + margin || t_at > hi - margin {
                continue;
            }
            let Ok(at) = edge.curve.evaluate(t_at) else {
                continue;
            };
            if at.sub(mcc.w_a_point).length() > 1e-6 * (1.0 + radius) {
                continue;
            }
            let side = |vid: u64| -> f64 {
                result
                    .vertices
                    .iter()
                    .find(|v| v.id == vid)
                    .map(|v| v.point.sub(mcc.a_axis_closest).dot(mcc.a_corner_dir))
                    .unwrap_or(f64::NEG_INFINITY)
            };
            let (sv, ev) = (edge.start_vertex_id, edge.end_vertex_id);
            let old_vertex = if side(sv) > side(ev) { sv } else { ev };
            if side(old_vertex) < plane_tol {
                continue; // does not actually overshoot into the wedge
            }
            split = Some((edge.id, t_at, old_vertex));
            break;
        }
        let Some((split_edge, t_at, old_vertex)) = split else {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "composition_order",
                format!(
                    "{CTX}: the wall-side cap fillet neither ends on its junction arc nor \
                 crosses the ideal W_A {:?} with its cap-contact edge (unsupported \
                 fillet composition order)",
                    mcc.w_a_point
                ),
            ));
        };
        let id = next_id();
        result.vertices.push(VertexRecord {
            id,
            point: mcc.w_a_point,
        });
        trim_edge_at(&mut result, split_edge, t_at, old_vertex, id)?;
        id
    };

    // Vertex/edge maps AFTER the split (the trim rewrote an edge end and its
    // pcurves).
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

    // Corner wedge between the two station planes (apex on Q's axis), with a
    // 3r reach bound about the cap-level axis point: the overshoot trim
    // reaches √5·r from it, past the 2r circle the planar path uses.
    let wedge = |p: Vec3| -> bool {
        p.sub(mcc.a_axis_closest).dot(mcc.a_corner_dir) >= -plane_tol
            && p.sub(mcc.b_axis_closest).dot(mcc.b_corner_dir) >= -plane_tol
            && p.sub(q_w).length() <= 3.0 * radius + plane_tol
    };

    // ---- 4. Trim Q back to the inner-equator arc (corner side = past the V
    //         level toward the cap), exactly as the planar path.
    let q_corner_side = |p: Vec3| -> bool { p.sub(q_v).dot(mcc.axis_dir) >= -plane_tol };
    let mut new_loops: HashMap<u64, Vec<CoedgeRecord>> = HashMap::default();
    let mut fresh_edges: Vec<EdgeRecord> = Vec::new();
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
                    "{CTX}: concave blend face {} has {} loops",
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
            CTX,
            face,
            &is_corner,
            [mcc.v_a.0, mcc.v_b.0],
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

    // ---- 5. A side: trim the overshoot back to a fresh junction arc
    //         V_A→W_A at the A station (the profile the fillet would have
    //         ended on under a tangent trim), or reuse the existing arc when
    //         the corner composed planar-style.
    let (arc_a, cyl_a_forward) = if trim_a {
        let face = result
            .shells
            .iter()
            .flat_map(|s| &s.faces)
            .find(|f| f.id == cyl_a_face_id)
            .unwrap();
        if face.loops.len() != 1 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Fragment,
                "multi_loop_a",
                format!(
                    "{CTX}: wall-side cap fillet {} has {} loops",
                    face.id,
                    face.loops.len()
                ),
            ));
        }
        let a_side =
            |p: Vec3| -> bool { p.sub(mcc.a_axis_closest).dot(mcc.a_corner_dir) >= -plane_tol };
        let is_corner: Vec<bool> = face.loops[0]
            .coedges
            .iter()
            .map(|co| {
                coedge_ends_3d(co)
                    .map(|(a, b)| a_side(a) && a_side(b))
                    .unwrap_or(false)
            })
            .collect();
        let (rebuilt, edge) = rebuild_mixed_corner_run(
            CTX,
            face,
            &is_corner,
            [mcc.v_a.0, w_a_id],
            mcc.a_axis_closest,
            radius,
            &vpoint,
            &edge_ends,
            &mut next_id,
        )?;
        new_loops.insert(face.id, rebuilt);
        fresh_edges.push(edge.clone());
        (edge, true)
    } else {
        let arc_a_id =
            locate_mixed_junction_arc(&result, CTX, mcc.v_a.0, w_a_id, mcc.a_axis_closest, radius)?;
        let forward = coedge_sense_on(&result, CTX, cyl_a_face_id, arc_a_id)?;
        (
            result
                .edges
                .iter()
                .find(|e| e.id == arc_a_id)
                .unwrap()
                .clone(),
            forward,
        )
    };

    // ---- 6. Trim the cap back to the outer contact arc W_A→W_B (2r about
    //         Q's axis).  Corner side = the corner WEDGE, not the 2r circle.
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
        n.dot(mcc.cap_normal).abs() > 1.0 - 1e-6
            && mcc.cap_normal.dot(corner.sub(*origin)).abs() < 1e-6
    };
    let mut fresh_w_arc: Option<EdgeRecord> = None;
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
                        .map(|(a, b)| wedge(a) && wedge(b))
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
                    format!("{CTX}: multiple cap faces carry a corner run"),
                ));
            }
            let (rebuilt, edge) = rebuild_mixed_corner_run(
                CTX,
                face,
                &is_corner,
                [w_a_id, mcc.w_b.0],
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
            format!("{CTX}: no cap face carries the corner run"),
        )
    })?;

    // ---- 7. Leftover notch junk: planar faces whose EVERY loop vertex lies
    //         in the corner wedge (the two fillet end bulkheads and the
    //         buried wall strip the overshoot resurfaced), connected to the
    //         corner blend complex across shared edges (flood fill).
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
                    .map(|(a, b)| wedge(a) && wedge(b))
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

    // ---- 8. The torus-sector patch (shared builder; generatrix = the A
    //         junction arc revolved about Q's axis to the B station).
    let arc_b = result
        .edges
        .iter()
        .find(|e| e.id == arc_b_id)
        .unwrap()
        .clone();
    let patch_face = build_mixed_sector_patch(
        CTX,
        &arc_a,
        &arc_b,
        &fresh_v_arc,
        &fresh_w_arc,
        cyl_a_forward,
        cyl_b_forward,
        mcc.v_a,
        mcc.v_b,
        (w_a_id, mcc.w_a_point),
        mcc.w_b,
        mcc.axis_point,
        mcc.axis_dir,
        mcc.a_axis_closest,
        mcc.b_axis_closest,
        radius,
        name,
        &mut next_id,
    )?;

    // ---- 9. Splice: fresh edges, rebuilt loops, junk removal, patch face.
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

    // ---- 10. Validate; surface the issues to the caller if any.
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!("{CTX}: {issues:?}"),
        ));
    }
    Ok(result)
}
