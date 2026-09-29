use super::*;

/// Golovanov §6.9.7 GENERAL "star" vertex blend: round a convex corner whose
/// N incident edges are ALREADY filleted but whose N fillet cylinders have NO
/// common inscribed ball (their axes are not concurrent, so no single sphere is
/// tangent to all N along a circle).  `round_convex_corner` routes here after
/// its least-squares tangency solve leaves a residual.  Because N=3 always has
/// a common ball, this path is only ever reached for N≥4; the notch is closed
/// by an N-sided patch that is (approximately) G1-tangent to each fillet
/// cylinder along a boundary arc rather than by a spherical N-gon.
///
/// The N=4 case — the dominant one — is filled by a single bicubic Coons patch
/// whose four boundaries are cubic Bézier arcs on the four cylinders.  N≥5
/// would need recursive fillet-of-fillet subdivision and is refused with a
/// clear message.
///
/// `corner` is the ORIGINAL sharp vertex (already trimmed by the edge fillets);
/// `radius` is the fillet radius.  Surgery is pure topology, no booleans: each
/// cylinder's corner-end coedge run is replaced by one fresh boundary arc, the
/// leftover flat caps are deleted, and the Coons patch is sewn on the four
/// fresh arcs (single shared EdgeRecords, forward=true on the trimmed cylinder
/// and forward=false on the patch).
pub(super) fn round_general_star(
    solid: &BrepSolid,
    corner: Vec3,
    radius: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
    use std::f64::consts::{PI, TAU};

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
    if result.vertices.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "empty_solid",
            "round_general_star: solid has no vertices",
        ));
    }
    let centroid = result
        .vertices
        .iter()
        .fold(Vec3::default(), |acc, v| acc.add(v.point))
        .scale(1.0 / result.vertices.len() as f64);

    // ---- Step A.1: outward wall normals (identical detection to
    //       round_convex_corner so the two paths agree on N). ----
    let mut normals: Vec<Vec3> = Vec::new();
    for shell in &result.shells {
        for face in &shell.faces {
            let Some(crate::AnalyticSurface::Plane {
                origin,
                u_dir,
                v_dir,
                ..
            }) = face.surface.analytic()
            else {
                continue;
            };
            let Ok(n) = u_dir.cross(*v_dir).normalized() else {
                continue;
            };
            if n.dot(corner.sub(*origin)).abs() > 1e-6 {
                continue;
            }
            let near_corner = face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
                coedge_ends_3d(co)
                    .map(|(a, b)| {
                        a.sub(corner).length() < 6.0 * radius
                            || b.sub(corner).length() < 6.0 * radius
                    })
                    .unwrap_or(false)
            });
            if !near_corner {
                continue;
            }
            let outward = if n.dot(corner.sub(centroid)) > 0.0 {
                n
            } else {
                n.scale(-1.0)
            };
            if !normals.iter().any(|m| m.dot(outward) > 1.0 - 1e-6) {
                normals.push(outward);
            }
        }
    }
    let n_faces = normals.len();
    if n_faces < 4 {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "wall_count",
            format!(
                "round_general_star: expected N≥4 walls at a no-common-ball star, found {n_faces}"
            ),
        ));
    }
    if n_faces != 4 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "star_n5",
            format!(
                "round_general_star: only N=4 general stars are supported (found N={n_faces}); \
             N≥5 needs recursive fillet-of-fillet subdivision (Golovanov §6.9.7) which is \
             not yet implemented"
            ),
        ));
    }

    // ---- Step A.2: the N fillet cylinders bordering the corner.  The fillets
    //       are radius-r circular cylinders; after the sequential booleans they
    //       are recognized as PARTIAL revolutions (`Revolution`), not full
    //       `RuledRevolution`, so accept either and verify the radius by
    //       sampling. ----
    struct StarCyl {
        face_id: u64,
        surface: NurbsSurface,
        origin: Vec3,
        axis: Vec3,
        x_axis: Vec3,
        y_axis: Vec3,
        radius: f64,
    }
    let mut cyls: Vec<StarCyl> = Vec::new();
    for shell in &result.shells {
        for face in &shell.faces {
            let frame = match face.surface.analytic() {
                Some(crate::AnalyticSurface::RuledRevolution { frame, .. }) => frame,
                Some(crate::AnalyticSurface::Revolution { frame, .. }) => frame,
                _ => continue,
            };
            let borders = face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
                edge_ends
                    .get(&co.edge_id)
                    .map(|&(s, e)| {
                        vpoint
                            .get(&s)
                            .map(|p| p.sub(corner).length() < 3.0 * radius)
                            .unwrap_or(false)
                            || vpoint
                                .get(&e)
                                .map(|p| p.sub(corner).length() < 3.0 * radius)
                                .unwrap_or(false)
                    })
                    .unwrap_or(false)
            });
            if !borders {
                continue;
            }
            let [u0, u1] = face
                .surface
                .domain_u()
                .or_refuse(KernelStage::Classify, "domain")?;
            let [v0, v1] = face
                .surface
                .domain_v()
                .or_refuse(KernelStage::Classify, "domain")?;
            let sample = face
                .surface
                .evaluate((u0 + u1) * 0.5, (v0 + v1) * 0.5)
                .or_refuse(KernelStage::Classify, "evaluate")?;
            let d = sample.sub(frame.origin);
            let axial = d.dot(frame.axis);
            let rad = d.sub(frame.axis.scale(axial)).length();
            if (rad - radius).abs() > 1e-4 {
                continue;
            }
            cyls.push(StarCyl {
                face_id: face.id,
                surface: face.surface.clone(),
                origin: frame.origin,
                axis: frame.axis,
                x_axis: frame.x_axis,
                y_axis: frame.y_axis,
                radius: rad,
            });
        }
    }
    if cyls.len() != n_faces {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "fillet_cylinders",
            format!(
                "round_general_star: expected {n_faces} fillet cylinders at the corner, found {}",
                cyls.len()
            ),
        ));
    }

    // ---- Step B: corner points P_f (contact-line intersections, one per wall).
    //       Each wall plane is tangent to exactly two of the cylinders (those
    //       whose axis is parallel to the wall, at distance r inside).  The
    //       cylinder's contact line on the wall = the axis projected onto the
    //       wall; the two contact lines meet at P_f, which lies on both
    //       cylinders (radius r from each axis) and coincides with an existing
    //       tangent vertex the edge fillets already created. ----
    struct StarCorner {
        vid: u64,
        point: Vec3,
        cyl_a: usize,
        cyl_b: usize,
    }
    let mut corners: Vec<StarCorner> = Vec::new();
    for n in &normals {
        let mut members: Vec<usize> = Vec::new();
        for (ci, cyl) in cyls.iter().enumerate() {
            if cyl.axis.dot(*n).abs() >= 1e-4 {
                continue;
            }
            let signed = n.dot(cyl.origin.sub(corner));
            if (signed.abs() - cyl.radius).abs() < 1e-3 {
                members.push(ci);
            }
        }
        if members.len() != 2 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "wall_cylinders",
                format!(
                    "round_general_star: wall has {} axis-parallel cylinders (expected 2)",
                    members.len()
                ),
            ));
        }
        // Project each cylinder axis line onto the wall plane (through `corner`,
        // normal n): point o - n·(n·(o-corner)), direction axis - n·(axis·n).
        let project = |cyl: &StarCyl| -> Result<(Vec3, Vec3), KernelRefusal> {
            let o = cyl.origin.sub(n.scale(n.dot(cyl.origin.sub(corner))));
            let d = cyl
                .axis
                .sub(n.scale(cyl.axis.dot(*n)))
                .normalized()
                .or_refuse(KernelStage::Classify, "normalized")?;
            Ok((o, d))
        };
        let (p1, d1) = project(&cyls[members[0]])?;
        let (p2, d2) = project(&cyls[members[1]])?;
        // Intersect the two coplanar lines (least squares closest approach).
        let w = p1.sub(p2);
        let (a, b, c, d, e) = (d1.dot(d1), d1.dot(d2), d2.dot(d2), d1.dot(w), d2.dot(w));
        let den = a * c - b * b;
        if den.abs() < 1e-12 {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "parallel_contact_lines",
                "round_general_star: contact lines on a wall are parallel",
            ));
        }
        let s = (b * e - c * d) / den;
        let pf = p1.add(d1.scale(s));
        // Match to the existing tangent vertex.
        let matched = result
            .vertices
            .iter()
            .min_by(|x, y| {
                x.point
                    .sub(pf)
                    .length()
                    .total_cmp(&y.point.sub(pf).length())
            })
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "corner_vertices",
                    "round_general_star: no vertices to match a corner point",
                )
            })?;
        if matched.point.sub(pf).length() > 1e-4 * (1.0 + radius) {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "tangent_vertex",
                format!(
                    "round_general_star: corner point {pf:?} has no existing tangent vertex \
                 (nearest is {:.3e} away)",
                    matched.point.sub(pf).length()
                ),
            ));
        }
        corners.push(StarCorner {
            vid: matched.id,
            point: matched.point,
            cyl_a: members[0],
            cyl_b: members[1],
        });
    }
    if corners.len() != n_faces {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "corner_count",
            format!(
                "round_general_star: expected {n_faces} corner points, found {}",
                corners.len()
            ),
        ));
    }
    // Every cylinder must appear in exactly two corner pairs.
    for ci in 0..cyls.len() {
        let deg = corners
            .iter()
            .filter(|k| k.cyl_a == ci || k.cyl_b == ci)
            .count();
        if deg != 2 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "cylinder_degree",
                format!(
                    "round_general_star: cylinder {ci} borders {deg} corner points (expected 2)"
                ),
            ));
        }
    }

    // Threshold that separates the near-corner notch (corner points at
    // ~r/tan(θ/2) and the interior fillet-fillet junk closer than that) from the
    // far fillet rims (which recede down the filleted edges, many r away).
    let max_corner_dist = corners
        .iter()
        .fold(0.0f64, |acc, k| acc.max(k.point.sub(corner).length()));
    let scope = max_corner_dist * 1.6;

    // id allocator over the whole shared id space.
    let mut next_id = crate::blend::edge::fresh_id_source(&result);

    // ---- Steps C + E (cylinder side): for each cylinder, replace its
    //       corner-end coedge run with ONE fresh boundary arc (a cubic Bézier
    //       helix on the cylinder joining its two corner points). ----
    struct Boundary {
        edge_id: u64,
        start_vid: u64,  // == t_a (the loop-run predecessor's tangent vertex)
        end_vid: u64,    // == t_b (the loop-run successor's tangent vertex)
        ctrl: [Vec3; 4], // cubic Bézier control points, ctrl[0]=start, ctrl[3]=end
    }
    let mut fresh_edges: Vec<EdgeRecord> = Vec::new();
    let mut new_loops: HashMap<u64, Vec<CoedgeRecord>> = HashMap::default();
    let mut boundaries: Vec<Boundary> = Vec::new();
    let mut patch_shell: Option<usize> = None;

    for (ci, cyl) in cyls.iter().enumerate() {
        let (si, face) = result
            .shells
            .iter()
            .enumerate()
            .flat_map(|(si, s)| s.faces.iter().map(move |f| (si, f)))
            .find(|(_, f)| f.id == cyl.face_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "cylinder_face",
                    "round_general_star: cylinder face vanished",
                )
            })?;
        if patch_shell.is_none() {
            patch_shell = Some(si);
        }
        if face.loops.len() != 1 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Fragment,
                "multi_loop",
                format!(
                    "round_general_star: cylinder face {} has {} loops",
                    face.id,
                    face.loops.len()
                ),
            ));
        }
        let coedges = &face.loops[0].coedges;
        let n = coedges.len();
        let traverse = |co: &CoedgeRecord| -> Option<(u64, u64)> {
            let &(s, e) = edge_ends.get(&co.edge_id)?;
            Some(if co.forward { (s, e) } else { (e, s) })
        };
        let near = |vid: u64| {
            vpoint
                .get(&vid)
                .map(|p| p.sub(corner).length() < scope)
                .unwrap_or(false)
        };
        // A coedge is on the corner run when BOTH its traversal endpoints are
        // near the corner (the two long contact lines and the far rim each have
        // one far endpoint, so they anchor the retained part of the loop).
        let is_corner: Vec<bool> = coedges
            .iter()
            .map(|co| {
                traverse(co)
                    .map(|(a, b)| near(a) && near(b))
                    .unwrap_or(false)
            })
            .collect();
        let anchor = (0..n).find(|&i| !is_corner[i]).ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Fragment,
                "all_corner_end",
                format!(
                    "round_general_star: cylinder face {} is all corner-end",
                    face.id
                ),
            )
        })?;
        let (prefix, run, suffix) = partition_corner_loop(coedges, &is_corner, anchor);
        if run.is_empty() {
            return Err(KernelRefusal::unsupported(
                KernelStage::Fragment,
                "no_corner_run",
                format!(
                    "round_general_star: cylinder face {} has no corner-end coedge",
                    face.id
                ),
            ));
        }
        let before = prefix.last().ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Fragment,
                "run_predecessor",
                format!(
                    "round_general_star: cylinder {} corner run has no predecessor",
                    face.id
                ),
            )
        })?;
        let (bs, be) = *edge_ends.get(&before.edge_id).unwrap();
        let t_a = if before.forward { be } else { bs };
        let after = suffix.first().or_else(|| prefix.first()).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Fragment,
                "run_successor",
                format!(
                    "round_general_star: cylinder {} corner run has no successor",
                    face.id
                ),
            )
        })?;
        let (as_, ae) = *edge_ends.get(&after.edge_id).unwrap();
        let t_b = if after.forward { as_ } else { ae };
        // t_a, t_b must be exactly this cylinder's two corner points.
        let my: Vec<u64> = corners
            .iter()
            .filter(|k| k.cyl_a == ci || k.cyl_b == ci)
            .map(|k| k.vid)
            .collect();
        if !(my.contains(&t_a) && my.contains(&t_b) && t_a != t_b) {
            return Err(KernelRefusal::unsupported(
                KernelStage::Fragment,
                "run_ends",
                format!(
                    "round_general_star: cylinder {} run ends ({t_a},{t_b}) are not its two corner \
                 points {my:?}",
                    face.id
                ),
            ));
        }

        // Build the boundary helix Bézier from t_a to t_b on this cylinder.
        let pa = *vpoint.get(&t_a).unwrap();
        let pb = *vpoint.get(&t_b).unwrap();
        let theta_z = |p: Vec3| -> (f64, f64) {
            let d = p.sub(cyl.origin);
            let z = d.dot(cyl.axis);
            let theta = d.dot(cyl.y_axis).atan2(d.dot(cyl.x_axis));
            (theta, z)
        };
        let (theta_a, z_a) = theta_z(pa);
        let (mut theta_b, z_b) = theta_z(pb);
        while theta_b - theta_a > PI {
            theta_b -= TAU;
        }
        while theta_b - theta_a < -PI {
            theta_b += TAU;
        }
        let helix = |t: f64| -> Vec3 {
            let th = theta_a + (theta_b - theta_a) * t;
            let z = z_a + (z_b - z_a) * t;
            cyl.origin
                .add(cyl.axis.scale(z))
                .add(cyl.x_axis.scale(cyl.radius * th.cos()))
                .add(cyl.y_axis.scale(cyl.radius * th.sin()))
        };
        // Exact vertex points at the ends, helix samples between.
        let samples = [pa, helix(1.0 / 3.0), helix(2.0 / 3.0), pb];
        let bez_points: Vec<Vec4> = samples
            .iter()
            .map(|p| Vec4 {
                x: p.x,
                y: p.y,
                z: p.z,
                w: 1.0,
            })
            .collect();
        let bez = fit::interpolate_homogeneous(&bez_points, 3, &[0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0])
            .or_refuse(KernelStage::Fragment, "interpolate")?;
        if bez.control_points.len() != 4 {
            return Err(KernelRefusal::internal(
                KernelStage::Fragment,
                "bezier_cubic",
                "round_general_star: boundary Bézier is not a 4-point cubic",
            ));
        }
        let ctrl: [Vec3; 4] = [
            Vec3::new(
                bez.control_points[0].x,
                bez.control_points[0].y,
                bez.control_points[0].z,
            ),
            Vec3::new(
                bez.control_points[1].x,
                bez.control_points[1].y,
                bez.control_points[1].z,
            ),
            Vec3::new(
                bez.control_points[2].x,
                bez.control_points[2].y,
                bez.control_points[2].z,
            ),
            Vec3::new(
                bez.control_points[3].x,
                bez.control_points[3].y,
                bez.control_points[3].z,
            ),
        ];

        let arc_id = next_id();
        fresh_edges.push(EdgeRecord {
            id: arc_id,
            curve: bez.clone(),
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: t_a,
            end_vertex_id: t_b,
            degenerate: false,
            name: None,
        });
        boundaries.push(Boundary {
            edge_id: arc_id,
            start_vid: t_a,
            end_vid: t_b,
            ctrl,
        });

        // Fresh cylinder coedge (forward=true, so t_a -> t_b): pcurve fitted by
        // projecting Bézier samples onto the cylinder in the surface's own
        // parameterization (u kept continuous across the short arc).
        const PCURVE_SAMPLES: usize = 13;
        let mut uv_points = Vec::with_capacity(PCURVE_SAMPLES);
        let mut uv_params = Vec::with_capacity(PCURVE_SAMPLES);
        let mut previous_u: Option<f64> = None;
        for k in 0..PCURVE_SAMPLES {
            let fraction = k as f64 / (PCURVE_SAMPLES - 1) as f64;
            let p = bez
                .evaluate(fraction)
                .or_refuse(KernelStage::Fragment, "evaluate")?;
            let projection = crate::project_point_to_surface(&cyl.surface, p)
                .or_refuse(KernelStage::Fragment, "project")?;
            let mut u = projection.u;
            let v = projection.v;
            if let Some(pu) = previous_u {
                while u - pu > 0.5 {
                    u -= 1.0;
                }
                while u - pu < -0.5 {
                    u += 1.0;
                }
            }
            previous_u = Some(u);
            uv_points.push(Vec4 {
                x: u,
                y: v,
                z: 0.0,
                w: 1.0,
            });
            uv_params.push(fraction);
        }
        let pcurve = fit::interpolate_homogeneous(&uv_points, 3, &uv_params)
            .or_refuse(KernelStage::Fragment, "interpolate")?;
        let fresh_co = CoedgeRecord {
            id: next_id(),
            edge_id: arc_id,
            forward: true,
            pcurve,
        };
        let mut rebuilt = prefix;
        rebuilt.push(fresh_co);
        rebuilt.extend(suffix);
        new_loops.insert(face.id, rebuilt);
    }
    let patch_shell = patch_shell.unwrap();

    // ---- Flat caps: the small planar notch faces between the cylinders.
    //       Scope them by the corner (no ball centre here) and grow the set out
    //       from the fillet cylinders across shared edges, exactly as
    //       round_convex_corner does. ----
    let cyl_face_ids: HashSet<u64> = cyls.iter().map(|c| c.face_id).collect();
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
    let candidate_cap = |face: &FaceRecord| -> bool {
        matches!(
            face.surface.analytic(),
            Some(crate::AnalyticSurface::Plane { .. })
        ) && face.loops.iter().flat_map(|l| &l.coedges).all(|co| {
            coedge_ends_3d(co)
                .map(|(a, b)| a.sub(corner).length() <= scope && b.sub(corner).length() <= scope)
                .unwrap_or(false)
        })
    };
    let mut cap_face_ids: HashSet<u64> = HashSet::default();
    loop {
        let mut added = false;
        for shell in &result.shells {
            for face in &shell.faces {
                if cap_face_ids.contains(&face.id) || !candidate_cap(face) {
                    continue;
                }
                let borders = face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
                    edge_faces
                        .get(&co.edge_id)
                        .map(|fs| {
                            fs.iter()
                                .any(|fid| cyl_face_ids.contains(fid) || cap_face_ids.contains(fid))
                        })
                        .unwrap_or(false)
                });
                if borders {
                    cap_face_ids.insert(face.id);
                    added = true;
                }
            }
        }
        if !added {
            break;
        }
    }

    // ---- Step D: the bicubic Coons vertex patch on the four boundary arcs. ----
    // Chain the four boundaries into the cyclic order they are traversed by the
    // patch (each used forward=false, i.e. end_vid -> start_vid), so a
    // successor's end_vid meets the current start_vid.
    let mut order: Vec<usize> = vec![0];
    let mut used = vec![false; boundaries.len()];
    used[0] = true;
    let mut chain_vertex = boundaries[0].start_vid;
    for _ in 0..(boundaries.len() - 1) {
        let nxt = (0..boundaries.len())
            .find(|&j| !used[j] && boundaries[j].end_vid == chain_vertex)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "arc_chain",
                    "round_general_star: boundary arcs do not form a closed quad",
                )
            })?;
        used[nxt] = true;
        order.push(nxt);
        chain_vertex = boundaries[nxt].start_vid;
    }
    if chain_vertex != boundaries[0].end_vid {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "arc_loop",
            "round_general_star: boundary arc loop is not closed",
        ));
    }
    // Traversal vertices W0..W3 where Wk = end_vid(order[k]) = start_vid(order[k-1]).
    // net corners: net[0][0]=W0, net[3][0]=W1, net[3][3]=W2, net[0][3]=W3.
    let seg = |k: usize| &boundaries[order[k]];
    let mut net = [[Vec3::default(); 4]; 4]; // net[i][j], i=u, j=v
                                             // net[i][0] = ctrl(edge0)[3-i]   (W0 -> W1, v=0)
                                             // net[3][j] = ctrl(edge1)[3-j]   (W1 -> W2, u=1)
                                             // net[i][3] = ctrl(edge2)[i]     (W3 -> W2 as i:0->3)
                                             // net[0][j] = ctrl(edge3)[j]     (W0 -> W3 as j:0->3)
    for i in 0..4 {
        net[i][0] = seg(0).ctrl[3 - i];
    }
    for j in 0..4 {
        net[3][j] = seg(1).ctrl[3 - j];
    }
    for i in 0..4 {
        net[i][3] = seg(2).ctrl[i];
    }
    for j in 0..4 {
        net[0][j] = seg(3).ctrl[j];
    }
    // Translational-Coons interior control points (each carries the two
    // incident boundary tangent legs, so the patch stays G1-ish to the
    // cylinders at the corners and interpolates the four arcs exactly).
    net[1][1] = net[1][0].add(net[0][1]).sub(net[0][0]);
    net[2][1] = net[2][0].add(net[3][1]).sub(net[3][0]);
    net[1][2] = net[1][3].add(net[0][2]).sub(net[0][3]);
    net[2][2] = net[2][3].add(net[3][2]).sub(net[3][3]);

    let control_points: Vec<Vec<Vec4>> = (0..4)
        .map(|i| {
            (0..4)
                .map(|j| Vec4 {
                    x: net[i][j].x,
                    y: net[i][j].y,
                    z: net[i][j].z,
                    w: 1.0,
                })
                .collect()
        })
        .collect();
    let surface = NurbsSurface::new(
        3,
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        control_points,
    )
    .or_refuse(KernelStage::Fragment, "surface_new")?;

    // Sanity: the patch must not be collapsed (its four corners are distinct and
    // the mid-point is a real interior point of a smooth cap).
    let pc = normals
        .iter()
        .fold(Vec3::default(), |acc, n| acc.add(*n))
        .normalized()
        .or_refuse(KernelStage::Validate, "normalized")?;
    let mid = surface
        .evaluate(0.5, 0.5)
        .or_refuse(KernelStage::Validate, "evaluate")?;
    let corner_pts = [net[0][0], net[3][0], net[3][3], net[0][3]];
    for a in 0..4 {
        for b in (a + 1)..4 {
            if corner_pts[a].sub(corner_pts[b]).length() < 1e-6 {
                return Err(KernelRefusal::internal(
                    KernelStage::Validate,
                    "patch_corners",
                    "round_general_star: degenerate patch (coincident corners)",
                ));
            }
        }
    }
    // The patch must be a genuine convex cap, not a collapsed sliver or an
    // inward dimple: its centre must stand OFF the flat quad through the four
    // corner points, on the side of the trimmed apex (the outward `pc`
    // direction).  A patch that dips below that quad (away from the apex) has
    // folded inward.
    let quad_centroid = corner_pts
        .iter()
        .fold(Vec3::default(), |acc, p| acc.add(*p))
        .scale(0.25);
    let apex_dir = corner
        .sub(quad_centroid)
        .normalized()
        .or_refuse(KernelStage::Validate, "normalized")?;
    let bulge = mid.sub(quad_centroid).dot(apex_dir);
    if bulge < -1e-9 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Validate,
            "patch_fold",
            format!(
                "round_general_star: patch centre folds inward (bulge {bulge:.3e} below the corner quad)"
            ),
        ));
    }
    let mid_extent = corner_pts
        .iter()
        .fold(f64::INFINITY, |acc, p| acc.min(mid.sub(*p).length()));
    if mid_extent < 1e-6 {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "patch_centre",
            "round_general_star: degenerate patch (centre coincides with a corner)",
        ));
    }
    let surface_normal = surface
        .normal(0.5, 0.5)
        .or_refuse(KernelStage::Validate, "normal")?;
    let same_sense = surface_normal.dot(pc) > 0.0;

    // Patch coedges (all forward=false): W0->W1 (v=0), W1->W2 (u=1), W2->W3
    // (v=1, u decreasing), W3->W0 (u=0, v decreasing).
    let pl = |u0, v0, u1, v1| {
        crate::sweep_topology::parameter_line(u0, v0, u1, v1)
            .or_refuse(KernelStage::Fragment, "parameter_line")
    };
    let patch_loop_id = next_id();
    let patch_coedges = vec![
        CoedgeRecord {
            id: next_id(),
            edge_id: seg(0).edge_id,
            forward: false,
            pcurve: pl(0.0, 0.0, 1.0, 0.0)?,
        },
        CoedgeRecord {
            id: next_id(),
            edge_id: seg(1).edge_id,
            forward: false,
            pcurve: pl(1.0, 0.0, 1.0, 1.0)?,
        },
        CoedgeRecord {
            id: next_id(),
            edge_id: seg(2).edge_id,
            forward: false,
            pcurve: pl(1.0, 1.0, 0.0, 1.0)?,
        },
        CoedgeRecord {
            id: next_id(),
            edge_id: seg(3).edge_id,
            forward: false,
            pcurve: pl(0.0, 1.0, 0.0, 0.0)?,
        },
    ];
    let patch_face = FaceRecord {
        id: next_id(),
        surface,
        same_sense,
        loops: vec![LoopRecord {
            id: patch_loop_id,
            coedges: patch_coedges,
        }],
        name: name.map(|value| value.to_string()),
    };

    // ---- Splice: add fresh arcs, rebuild cylinder loops, drop caps, add patch.
    result.edges.extend(fresh_edges);
    for (si, shell) in result.shells.iter_mut().enumerate() {
        shell.faces.retain(|f| !cap_face_ids.contains(&f.id));
        for face in shell.faces.iter_mut() {
            if let Some(coedges) = new_loops.get(&face.id) {
                face.loops[0].coedges = coedges.clone();
            }
        }
        if si == patch_shell {
            shell.faces.push(patch_face.clone());
        }
    }

    // Remove orphaned edges/vertices.
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

    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!("round_general_star: {issues:?}"),
        ));
    }
    Ok(result)
}
