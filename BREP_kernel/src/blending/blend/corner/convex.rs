use super::*;

/// §6.9.7 vertex ("star") blend: round a convex trihedral corner whose three
/// incident edges are ALREADY filleted, by pure topology surgery — no
/// booleans.  `corner` is the ORIGINAL sharp corner coordinate (already
/// trimmed away by the edge fillets).  A spherical octant of `radius`,
/// tangent to all three cylindrical fillets along their tangent circles, is
/// sewn into the notch; the three fillet loops are closed on those tangent
/// circles and the leftover flat caps are removed.
///
/// Dispatch beyond the convex trihedral fast path:
///   * N>3 walls with a common tangent ball → spherical N-gon patch;
///   * N≥4 walls with NO common ball → [`round_general_star`] (N=4 Coons);
///   * trihedral corner with ONE CONCAVE incident edge (fillet tangent
///     vertices missing) → [`round_mixed_concave_corner`] (torus sector);
///   * CURVED-WALL trihedral corner (two planar walls + one cylindrical
///     wall through the vertex, all three incident edges convex and
///     filleted) → corner ball solved against the offset cylinder, then the
///     same tangent-arc rebuild + spherical-triangle patch;
///   * anything else (all/multi-concave, tilted concave axis, MIXED
///     curved-wall corners, non-cylindrical curved walls, N≥5
///     no-common-ball) refuses with a message naming the configuration.
pub fn round_convex_corner(
    solid: &BrepSolid,
    corner: Vec3,
    radius: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
    if !(radius > 0.0) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "radius",
            "round_convex_corner: radius must be positive",
        ));
    }
    let mut result = solid.clone();

    // Vertex position lookup.
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
            "round_convex_corner: solid has no vertices",
        ));
    }

    // 1. The three planar faces whose plane passes through `corner`; collect
    //    one OUTWARD normal per distinct direction.  Outwardness comes from
    //    the face record itself (geometric plane normal × same_sense): a
    //    vertex-centroid sign heuristic breaks exactly at MIXED-convexity
    //    corners — an L-prism's reentrant corner sits ON the centroid's
    //    mirror plane, where the sign test is degenerate and flips the two
    //    wall normals inward.
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
            // Plane must pass through the corner point.
            if n.dot(corner.sub(*origin)).abs() > 1e-6 {
                continue;
            }
            // Face must border the corner region (skip coincident far faces).
            // The nearest surviving coedge vertex of a corner wall recedes from
            // the corner as the dihedral sharpens: the fillet contact line sits
            // ~r/tan(θ/2) out, so a fixed 3·r window (fine for right/obtuse
            // corners) DROPS the far wall + cap on acute corners (d ≈ 3.1·r at
            // 38°, 3.7·r near 30°) and the corner then looks like it has only
            // one planar face.  Widen the window to 6·r — still only ever
            // matches faces whose plane already passes through the corner (the
            // walls meeting there), so no unrelated far face is pulled in.
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
            let outward = if face.same_sense { n } else { n.scale(-1.0) };
            if !normals.iter().any(|m| m.dot(outward) > 1.0 - 1e-6) {
                normals.push(outward);
            }
        }
    }
    // CURVED-WALL star corners (§6.9.7): one wall is a CYLINDER through the
    // corner (e.g. a bulged prism wall meeting two planes at a vertex).  The
    // corner-ball construction generalizes cleanly — C sits at distance r
    // inside each plane and at axis distance R−r (boss) / R+r (bore) from
    // the cylinder, and every blend face's corner cross-section through C is
    // STILL a radius-r circle centered at C (the corner ball's characteristic
    // circle on each constant-radius canal blend: the plane×plane and
    // plane×cylinder blends have C on their centre path, the rim blend is a
    // canal torus with C on its centre circle).  So after solving C from the
    // two offset planes + offset cylinder (closed-form quadratic on the
    // offset-plane intersection line) and taking the cylinder's contact
    // normal (T_cyl − C)/r as the third "wall normal", the ENTIRE planar
    // machinery below — tangent-vertex reuse, fresh tangent-arc rebuild of
    // each blend's corner end, notch-cap removal, general spherical-triangle
    // patch — applies verbatim.  The wall faces themselves must only be kept
    // out of the blend-face rebuild (a curved WALL borders tangent vertices
    // too; blends never contain the sharp corner, walls do).
    //
    // Support boundary (refusals below name it): exactly TWO planar walls +
    // ONE cylindrical wall.  All three incident edges convex and filleted →
    // the corner-ball root proven by its tangent vertices, sphere patch.
    // ONE CONCAVE plane×cylinder edge (the semicircular-boss fixture) → no
    // convex corner-ball root exists and the solve falls through to
    // `round_mixed_concave_curved_corner`: the concave blend's axis is the
    // offset-plane × offset-cylinder intersection LINE (both walls extruded
    // along the cap normal), and the concave-vertex torus-sector closure
    // applies about it.  Still refused with named messages: a cylinder axis
    // TILTED against the cap normal (the blend's centre path is then a
    // curve, no exact revolution), missing fillets / tangency vertices,
    // unsupported composition orders, ≥2 cylindrical walls, and
    // non-cylindrical curved carriers (cone/sphere/torus/freeform walls).
    let mut curved_wall_face_ids: HashSet<u64> = HashSet::default();
    let mut curved_centre: Option<Vec3> = None;
    // A MIXED curved-wall corner whose LAST-composed cap fillet overshot the
    // corner resurfaces a strip of the wall plane with the OPPOSITE outward
    // normal (the strip fronts the overshoot chamber), so the corner shows
    // THREE plane normals containing an ANTIPARALLEL pair — a configuration
    // no genuine trihedral corner produces (its tangency system is
    // singular).  Route it through the curved-wall lane, trying each way of
    // dropping one antiparallel member (the mixed detector's tangency-vertex
    // proof selects the genuine wall).
    let antiparallel_pair = if normals.len() == 3 {
        (0..3).find_map(|i| {
            ((i + 1)..3)
                .find(|&j| normals[i].dot(normals[j]) < -(1.0 - 1e-6))
                .map(|j| (i, j))
        })
    } else {
        None
    };
    if normals.len() < 3 || antiparallel_pair.is_some() {
        struct CylWall {
            axis_point: Vec3,
            axis_dir: Vec3,
            radius: f64,
            outward_away: bool,
        }
        let mut cylinders: Vec<CylWall> = Vec::new();
        let mut other_curved = 0usize;
        for shell in &result.shells {
            for face in &shell.faces {
                if matches!(
                    face.surface.analytic(),
                    Some(crate::AnalyticSurface::Plane { .. })
                ) {
                    continue;
                }
                let Ok(proj) = crate::project_point_to_surface(&face.surface, corner) else {
                    continue;
                };
                if proj.distance > 1e-6 {
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
                curved_wall_face_ids.insert(face.id);
                let Some((axis_point, axis_dir, cyl_radius)) = cylinder_wall_carrier(&face.surface)
                else {
                    other_curved += 1;
                    continue;
                };
                let Ok(n_geom) = face.surface.normal(proj.u, proj.v) else {
                    other_curved += 1;
                    continue;
                };
                let outward = if face.same_sense {
                    n_geom
                } else {
                    n_geom.scale(-1.0)
                };
                let rel = corner.sub(axis_point);
                let Ok(radial) = rel.sub(axis_dir.scale(rel.dot(axis_dir))).normalized() else {
                    other_curved += 1; // corner on the axis: degenerate carrier
                    continue;
                };
                let outward_away = outward.dot(radial) > 0.0;
                let duplicate = cylinders.iter().any(|cyl| {
                    cyl.axis_dir.cross(axis_dir).length() < 1e-6
                        && {
                            let between = axis_point.sub(cyl.axis_point);
                            between
                                .sub(cyl.axis_dir.scale(between.dot(cyl.axis_dir)))
                                .length()
                                < 1e-6
                        }
                        && (cyl.radius - cyl_radius).abs() < 1e-6
                        && cyl.outward_away == outward_away
                });
                if !duplicate {
                    cylinders.push(CylWall {
                        axis_point,
                        axis_dir,
                        radius: cyl_radius,
                        outward_away,
                    });
                }
            }
        }
        if curved_wall_face_ids.is_empty() && antiparallel_pair.is_none() {
            return Err(KernelRefusal::input(
                KernelStage::Classify,
                "planar_walls",
                format!(
                    "round_convex_corner: expected 3 or more planar faces at the corner, found {}",
                    normals.len()
                ),
            ));
        }
        if !curved_wall_face_ids.is_empty() {
            if cylinders.len() != 1
                || other_curved != 0
                || !(normals.len() == 2 || antiparallel_pair.is_some())
            {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Classify,
                    "curved_wall_mix",
                    format!(
                    "round_convex_corner: star corner with a curved (non-planar) wall carrier is \
                     only supported as two planar walls + one cylindrical wall — found {} planar, \
                     {} cylindrical and {} other curved wall carrier(s) through the corner \
                     (§6.9.7 curved-wall vertex blend)",
                    normals.len(),
                    cylinders.len(),
                    other_curved
                    ),
                ));
            }
            let cyl = &cylinders[0];
            if let Some((i, j)) = antiparallel_pair {
                // Only a mixed-convexity overshoot presents an antiparallel
                // wall-plane pair; no convex corner ball can exist.  Try the
                // curved-mixed closure with either antiparallel member dropped.
                let k = 3 - i - j;
                let mut mixed_reasons: Vec<String> = Vec::new();
                // Both attempts run the same detector; the LAST refusal's
                // class carries the joined text, so the inner class survives
                // the wrap.
                let mut carried: Option<KernelRefusal> = None;
                for wall in [normals[i], normals[j]] {
                    match round_mixed_concave_curved_corner(
                        solid,
                        corner,
                        radius,
                        [wall, normals[k]],
                        cyl.axis_point,
                        cyl.axis_dir,
                        cyl.radius,
                        cyl.outward_away,
                        name,
                    ) {
                        Ok(done) => return Ok(done),
                        Err(why) => {
                            mixed_reasons.push(why.message.clone());
                            carried = Some(why);
                        }
                    }
                }
                let message = format!(
                    "round_convex_corner: star corner with a curved (non-planar) wall \
                     carrier shows an antiparallel wall-plane pair (a mixed-convexity \
                     overshoot signature — no corner-ball root exists for a genuine \
                     trihedral corner here), and the mixed-convexity curved-wall closure \
                     refused: {} (§6.9.7 curved-wall vertex blend)",
                    mixed_reasons.join(" / ")
                );
                return Err(match carried {
                    Some(why) => why.with_message(|_| message),
                    None => KernelRefusal::internal(
                        KernelStage::Classify,
                        "antiparallel_attempts",
                        message,
                    ),
                });
            }
            let candidates = curved_wall_ball_candidates(
                corner,
                radius,
                normals[0],
                normals[1],
                cyl.axis_point,
                cyl.axis_dir,
                cyl.radius,
                cyl.outward_away,
            )
            .map_err(|error| {
                error.with_message(|why| {
                    format!(
                    "round_convex_corner: star corner with a curved (non-planar) wall carrier: \
                     {why} (§6.9.7 curved-wall vertex blend)"
                    )
                })
            })?;
            // The valid root is PROVEN by the tangent vertices the three edge
            // fillets left behind: all three contact points C + r·nᵢ must be
            // existing vertices.  No root qualifying means the corner is not the
            // all-convex filleted configuration (e.g. the boss fixture's MIXED
            // corner with its concave plane×cylinder edge).
            let matched = candidates.iter().find(|(centre, n_contact)| {
                [normals[0], normals[1], *n_contact].iter().all(|n| {
                    let ideal = centre.add(n.scale(radius));
                    result
                        .vertices
                        .iter()
                        .any(|v| v.point.sub(ideal).length() < 1e-6)
                })
            });
            let Some((centre, n_contact)) = matched else {
                // MIXED-convexity curved-wall corner (one CONCAVE plane×cylinder
                // edge): no convex corner ball carries the tangency vertices, but
                // the concave-vertex torus-sector closure generalizes — both
                // walls are extruded along the cap normal, so the concave
                // blend's axis is the offset-plane × offset-cylinder
                // intersection LINE and the closure is an exact revolution.
                // `result` is still an untouched clone of `solid` here.
                return round_mixed_concave_curved_corner(
                    solid,
                    corner,
                    radius,
                    [normals[0], normals[1]],
                    cyl.axis_point,
                    cyl.axis_dir,
                    cyl.radius,
                    cyl.outward_away,
                    name,
                )
                .map_err(|error| {
                    error.with_message(|mixed| {
                        format!(
                        "round_convex_corner: star corner with a curved (non-planar) wall \
                         carrier: no corner-ball root has all three convex-fillet tangent \
                         vertices (not an all-convex curved-wall corner with its three \
                         incident edges filleted), and the corner is not a supported \
                         mixed-convexity curved-wall corner ({mixed}) (§6.9.7 curved-wall \
                         vertex blend)"
                        )
                    })
                });
            };
            curved_centre = Some(*centre);
            normals.push(*n_contact);
        }
        // (antiparallel pair with no curved wall carrier: fall through to
        // the planar solve, which reports the singular tangency system.)
    }
    let n_faces = normals.len();
    // The EXACT iso-parameter octant fast path applies ONLY to an orthogonal
    // TRIHEDRAL corner (3 mutually perpendicular PLANAR faces).  Any other
    // trihedral angle — including every curved-wall corner — uses the general
    // fitted spherical-triangle patch, and N>3 faces (a §6.9.7 "star", e.g. a
    // pyramid apex) use the general spherical N-gon.
    let orthogonal = curved_centre.is_none()
        && n_faces == 3
        && (0..3).all(|i| ((i + 1)..3).all(|j| normals[i].dot(normals[j]).abs() <= 1e-6));

    // 2. Ball centre C: the point at distance r inside every face plane.  Each
    //    plane passes through `corner`, so with d = C − corner the tangency
    //    conditions are nᵢ·d = −r (i = 1..N).  A trihedral corner (N=3) is a
    //    determined 3×3 system with an EXACT solution; N>3 is OVER-determined,
    //    so solve LEAST-SQUARES via the normal equations AᵀA d = Aᵀb (rows of
    //    A = nᵢ, b = −r) and REQUIRE the residual to vanish — a common tangent
    //    ball (a single rolling ball tangent to all N faces) exists only then.
    //    If it does NOT, the N faces form a GENERAL star that would need
    //    Golovanov's fillet-the-fillet-intersection recursion; this vertex
    //    patch cannot sew it, so refuse gracefully (no crash, no bad solid).
    //    A CURVED-WALL corner arrives here with C already solved from the two
    //    offset planes + offset cylinder (its third "normal" is the contact
    //    direction, which depends on C, so the planar linear solve does not
    //    apply).
    let c = if let Some(centre) = curved_centre {
        centre
    } else if n_faces == 3 {
        // Three CONVEX `−r` offset planes meeting in one point — the same
        // closed form as the corner's plane PAIRS, one row wider (audit §11).
        // The audit's Slice 2b table listed three configurations; this is the
        // fourth, and it is in the same directory.
        let normals3 = [normals[0], normals[1], normals[2]];
        let distances = [-radius, -radius, -radius];
        let centre = offset_plane_triple(corner, normals3, distances)
            // The pre-slice line was `solve_small(..)?`, so this refusal text
            // was `solve_small`'s own. Kept verbatim: this slice moves no
            // message.
            .map_err(|_| {
                KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "singular_tangency",
                    "solve_small: singular matrix",
                )
            })?;
        offset_pair_diag::plane_triple(
            "convex::corner_ball",
            corner,
            normals3,
            distances,
            Some(centre),
        );
        centre
    } else {
        let mut ata = [[0.0f64; 3]; 3];
        let mut atb = [0.0f64; 3];
        for nrm in &normals {
            let ni = [nrm.x, nrm.y, nrm.z];
            for a in 0..3 {
                atb[a] += ni[a] * (-radius);
                for b in 0..3 {
                    ata[a][b] += ni[a] * ni[b];
                }
            }
        }
        let d = crate::fit::solve_small::<3>(ata, atb, 3).map_err(|_| {
            KernelRefusal::ill_posed(
                KernelStage::Classify,
                "singular_tangency_lsq",
                "round_convex_corner: tangency normal equations are singular",
            )
        })?;
        let c = corner.add(Vec3::new(d[0], d[1], d[2]));
        let worst = normals.iter().fold(0.0f64, |acc, nrm| {
            acc.max((nrm.dot(c.sub(corner)) + radius).abs())
        });
        if worst > 1e-6 * radius {
            // No single ball is tangent to all N cylinders along a circle (the N
            // fillet-cylinder axes are not concurrent): this is Golovanov's
            // §6.9.7 GENERAL "star". Close the notch with an N-sided patch
            // tangent to each cylinder along a boundary arc instead of a
            // spherical N-gon. `result` is still the untouched edge-filleted
            // clone at this point (nothing above mutates it).
            return round_general_star(&result, corner, radius, name);
        }
        c
    };

    // 3. Tangent point Tᵢ = C + r·nᵢ per face; reuse the matching existing
    //    tangent vertex id created by the edge fillets.  If any tangent
    //    vertex is MISSING the corner is not the all-convex configuration
    //    this sphere-patch path serves — for a trihedral corner, try the
    //    MIXED-convexity path (one concave edge, §6.9.7): its fillets leave
    //    tangency vertices elsewhere (the concave blend's contact points).
    let mut t_ids: Vec<u64> = Vec::with_capacity(n_faces);
    let mut missing_tangent: Option<Vec3> = None;
    for nrm in &normals {
        let ideal = c.add(nrm.scale(radius));
        match result
            .vertices
            .iter()
            .find(|v| v.point.sub(ideal).length() < 1e-6)
        {
            Some(found) => t_ids.push(found.id),
            None => {
                missing_tangent = Some(ideal);
                break;
            }
        }
    }
    if let Some(ideal) = missing_tangent {
        if n_faces == 3 && curved_centre.is_none() {
            // `result` is still an untouched clone of `solid` here.  (A
            // curved-wall corner never lands here — its root was selected by
            // exactly this tangent-vertex existence test — and the planar
            // mixed-concave machinery would misread its cylinder wall.)
            return round_mixed_concave_corner(solid, corner, radius, &normals, name).map_err(
                |error| {
                    error.with_message(|mixed| {
                        format!(
                        "round_convex_corner: no convex tangent vertex near {ideal:?}, and the \
                         corner is not a supported mixed-convexity trihedral corner ({mixed})"
                        )
                    })
                },
            );
        }
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "concave_star_n4",
            format!(
            "round_convex_corner: no existing tangent vertex near {ideal:?} (a star corner \
             with concave incident edges is unsupported for N>3 walls)"
            ),
        ));
    }
    let t_id_set: HashSet<u64> = t_ids.iter().copied().collect();

    // id allocator across the whole solid's shared id space.
    let mut next_id = crate::blend::edge::fresh_id_source(&result);

    // Tolerance for on-plane / corner-side tests, scaled to the ball radius.
    let plane_tol = 1e-6 * radius;

    // Map each edge id to the faces using it (cap adjacency below).
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

    // The curved "fillet" faces at this corner: non-planar faces touching a
    // tangent vertex.  Needed both to scope the flat-cap search and (below) to
    // drive the per-fillet corner rebuild.  A curved WALL also borders tangent
    // vertices (its contact curves with two blends cross at T_cyl on its own
    // boundary), so wall carriers are excluded explicitly.
    let corner_fillet_faces: HashSet<u64> = result
        .shells
        .iter()
        .flat_map(|s| &s.faces)
        .filter(|face| {
            !matches!(
                face.surface.analytic(),
                Some(crate::AnalyticSurface::Plane { .. })
            ) && !curved_wall_face_ids.contains(&face.id)
                && face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
                    edge_ends
                        .get(&co.edge_id)
                        .map(|&(s, e)| t_id_set.contains(&s) || t_id_set.contains(&e))
                        .unwrap_or(false)
                })
        })
        .map(|f| f.id)
        .collect();

    // Flat caps: the small planar faces filling the notch.  For an ACUTE
    // corner the leftover sharp vertex and the fillet-fillet intersection
    // vertices sit OUTSIDE the tangent ball, so a fixed 1.45r window (fine for
    // obtuse corners) MISSES them.  Scope the corner cluster generously by
    // |corner-C| + 2r — which always contains the sharp-corner projections and
    // the fillet-fillet intersections of radius-r fillets while excluding the
    // far model — keep only planar faces fully inside that scope, and grow the
    // set out from the corner fillets across shared edges so every cap that
    // hangs off a fillet (directly or through another cap) is removed.
    //
    // A pure distance-to-C window is NOT enough on its own: at a sharp corner
    // the ball centre C recedes from the corner as ~r/sin(θ/2), so |corner-C|
    // (and hence the scope) grows, while the REAL neighbouring model face (the
    // trimmed top cap / a wall) shrinks — its surviving triangle can fall
    // ENTIRELY inside the scope ball and be swallowed as junk, tearing a hole
    // in the solid (the r=1.5, 43° corner: its top cap's far corners sit 6.3
    // and 7.1 from C, both inside a 7.35 scope).  The notch junk, by contrast,
    // clusters BETWEEN the sharp corner and C: every junk vertex lies on the
    // corner side of the plane through C ⟂ (corner-C), i.e. its signed reach
    // s = (v-C)·(corner-C) is ≥ 0 (tangent points sit at small +s).  A real
    // model face reaches PAST C the other way (s ≈ −(a few)·r at its far
    // vertices).  So additionally require every cap vertex to satisfy
    // s ≥ −0.5r·|corner-C|: keeps all genuine junk removable while protecting
    // any face that extends into the model interior beyond C.
    let corner_scope = corner.sub(c).length() + 2.0 * radius;
    let corner_dir = corner.sub(c);
    let near_side_limit = -0.5 * radius * corner_dir.length();
    let candidate_cap = |face: &FaceRecord| -> bool {
        matches!(
            face.surface.analytic(),
            Some(crate::AnalyticSurface::Plane { .. })
        ) && face.loops.iter().flat_map(|l| &l.coedges).all(|co| {
            coedge_ends_3d(co)
                .map(|(a, b)| {
                    a.sub(c).length() <= corner_scope
                        && b.sub(c).length() <= corner_scope
                        && a.sub(c).dot(corner_dir) >= near_side_limit
                        && b.sub(c).dot(corner_dir) >= near_side_limit
                })
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
                let borders_corner = face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
                    edge_faces
                        .get(&co.edge_id)
                        .map(|fs| {
                            fs.iter().any(|fid| {
                                corner_fillet_faces.contains(fid) || cap_face_ids.contains(fid)
                            })
                        })
                        .unwrap_or(false)
                });
                if borders_corner {
                    cap_face_ids.insert(face.id);
                    added = true;
                }
            }
        }
        if !added {
            break;
        }
    }

    // Fillet faces: curved (non-planar) faces touching a tangent vertex.
    // For each, replace the corner-end coedge run with ONE fresh tangent arc.
    let mut new_loops: HashMap<u64, Vec<CoedgeRecord>> = HashMap::default();
    let mut fresh_edges: Vec<EdgeRecord> = Vec::new();
    let mut fresh_arcs: Vec<(u64, u64, u64)> = Vec::new(); // (edge_id, start_vid, end_vid)
    let mut fillet_shell: Option<usize> = None;
    let mut fillet_count = 0usize;

    for (si, shell) in result.shells.iter().enumerate() {
        for face in &shell.faces {
            let is_plane = matches!(
                face.surface.analytic(),
                Some(crate::AnalyticSurface::Plane { .. })
            );
            if is_plane || curved_wall_face_ids.contains(&face.id) {
                continue;
            }
            let touches_tangent = face.loops.iter().flat_map(|l| &l.coedges).any(|co| {
                let (s, e) = match edge_ends.get(&co.edge_id) {
                    Some(v) => *v,
                    None => return false,
                };
                t_id_set.contains(&s) || t_id_set.contains(&e)
            });
            if !touches_tangent {
                continue;
            }
            if face.loops.len() != 1 {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Fragment,
                    "multi_loop",
                    format!(
                        "round_convex_corner: fillet face {} has {} loops",
                        face.id,
                        face.loops.len()
                    ),
                ));
            }
            let coedges = &face.loops[0].coedges;
            let n = coedges.len();

            // Pair this fillet with the TWO tangent vertices it borders, by
            // GEOMETRY: the tangent vertices that appear as endpoints of its
            // loop (its two longitudinal contact lines reach them).  We must
            // NOT read the messy sequential coedge run's loose ends — for an
            // ACUTE corner those land on a spurious fillet-fillet vertex.
            let mut border_tvs: Vec<u64> = Vec::new();
            for co in coedges {
                if let Some(&(s, e)) = edge_ends.get(&co.edge_id) {
                    for v in [s, e] {
                        if t_id_set.contains(&v) && !border_tvs.contains(&v) {
                            border_tvs.push(v);
                        }
                    }
                }
            }
            if border_tvs.len() != 2 {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Fragment,
                    "tangent_vertex_pair",
                    format!(
                    "round_convex_corner: fillet face {} borders {} tangent vertices \
                     (expected exactly 2); acute geometry too degenerate to rebuild",
                    face.id,
                    border_tvs.len()
                    ),
                ));
            }
            let bpt0 = *vpoint.get(&border_tvs[0]).unwrap();
            let bpt1 = *vpoint.get(&border_tvs[1]).unwrap();
            // Tangent-circle plane of this cylinder fillet: sphere∩cylinder is a
            // radius-r circle in the plane through C ⟂ the cylinder axis, and
            // both tangent points lie on it (each Tᵢ−C ⟂ the axis), so the
            // plane normal is (Ta−C)×(Tb−C).  Orient it TOWARD the sharp corner.
            // Everything on the corner side of this plane is notch junk (flat
            // caps, fillet-fillet intersection vertices/edges) that the fresh
            // tangent arc replaces; everything on the far side is the real
            // fillet body to keep.  This trims/rebuilds the corner end at the
            // tangent circle regardless of how the sequential fillets cut it.
            let mut axis = bpt0.sub(c).cross(bpt1.sub(c)).normalized().map_err(|_| {
                KernelRefusal::ill_posed(
                    KernelStage::Fragment,
                    "colinear_tangents",
                    format!(
                        "round_convex_corner: fillet face {} tangent points are colinear with C",
                        face.id
                    ),
                )
            })?;
            if corner.sub(c).dot(axis) < 0.0 {
                axis = axis.scale(-1.0);
            }
            let corner_side = |p: Vec3| p.sub(c).dot(axis) >= -plane_tol;
            let is_corner: Vec<bool> = coedges
                .iter()
                .map(|co| {
                    coedge_ends_3d(co)
                        .map(|(a, b)| corner_side(a) && corner_side(b))
                        .unwrap_or(false)
                })
                .collect();
            let anchor = (0..n).find(|&i| !is_corner[i]).ok_or_else(|| {
                KernelRefusal::unsupported(
                    KernelStage::Fragment,
                    "all_corner_end",
                    format!(
                        "round_convex_corner: fillet face {} is all corner-end",
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
                        "round_convex_corner: fillet face {} has no corner-end coedge",
                        face.id
                    ),
                ));
            }
            // T_a: traversal-end vertex of the coedge before the run.
            let before = prefix.last().ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "run_predecessor",
                    format!(
                        "round_convex_corner: fillet face {} corner run has no predecessor",
                        face.id
                    ),
                )
            })?;
            let (bs, be) = *edge_ends.get(&before.edge_id).unwrap();
            let t_a = if before.forward { be } else { bs };
            let uv_a = before
                .pcurve
                .evaluate(1.0)
                .or_refuse(KernelStage::Fragment, "evaluate")?;
            // The coedge after the run (wraps to prefix[0] if the run trails).
            let after = suffix.first().or_else(|| prefix.first()).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "run_successor",
                    format!(
                        "round_convex_corner: fillet face {} corner run has no successor",
                        face.id
                    ),
                )
            })?;
            let (as_, ae) = *edge_ends.get(&after.edge_id).unwrap();
            let t_b = if after.forward { as_ } else { ae };
            let uv_b = after
                .pcurve
                .evaluate(0.0)
                .or_refuse(KernelStage::Fragment, "evaluate")?;

            if !t_id_set.contains(&t_a) || !t_id_set.contains(&t_b) || t_a == t_b {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Fragment,
                    "run_ends",
                    format!(
                    "round_convex_corner: fillet face {} corner ends are not two distinct tangent vertices ({t_a},{t_b})",
                    face.id
                    ),
                ));
            }

            // Fresh tangent-circle arc T_a -> T_b (radius-r arc centered at C).
            let ta_pt = *vpoint.get(&t_a).unwrap();
            let tb_pt = *vpoint.get(&t_b).unwrap();
            let x_axis = ta_pt
                .sub(c)
                .normalized()
                .or_refuse(KernelStage::Fragment, "normalized")?;
            let rb = tb_pt.sub(c);
            let y_axis = rb
                .sub(x_axis.scale(rb.dot(x_axis)))
                .normalized()
                .or_refuse(KernelStage::Fragment, "normalized")?;
            // Sweep the ACTUAL angle Tₐ–C–T_b (= angle between the two face
            // normals); only 90° for orthogonal corners.  make_arc segments it
            // (up to a full turn) so any convex angle in (0, π) is exact.
            let sweep = x_axis
                .dot(rb.normalized().or_refuse(KernelStage::Fragment, "normalized")?)
                .clamp(-1.0, 1.0)
                .acos();
            let arc = crate::make_arc(c, x_axis, y_axis, radius, 0.0, sweep)
                .or_refuse(KernelStage::Fragment, "make_arc")?;
            let arc_id = next_id();
            fresh_edges.push(EdgeRecord {
                id: arc_id,
                curve: arc,
                t0: 0.0,
                t1: 1.0,
                start_vertex_id: t_a,
                end_vertex_id: t_b,
                degenerate: false,
                name: None,
            });
            fresh_arcs.push((arc_id, t_a, t_b));

            // Fresh coedge on the fillet: param-line at the tangent v* from the
            // T_a contact to the T_b contact, running the loop direction.
            let fresh_co = CoedgeRecord {
                id: next_id(),
                edge_id: arc_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(uv_a.x, uv_a.y, uv_b.x, uv_b.y)
                    .or_refuse(KernelStage::Fragment, "parameter_line")?,
            };
            let mut rebuilt = prefix;
            rebuilt.push(fresh_co);
            rebuilt.extend(suffix);
            new_loops.insert(face.id, rebuilt);
            fillet_count += 1;
            if fillet_shell.is_none() {
                fillet_shell = Some(si);
            }
        }
    }
    if fillet_count != n_faces {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "fillet_count",
            format!(
            "round_convex_corner: expected {n_faces} fillet faces at the corner, found {fillet_count}"
            ),
        ));
    }
    let octant_shell = fillet_shell.unwrap();

    // 4. Build the vertex patch reusing the fresh arcs + existing tangent
    //    vertices.  An orthogonal trihedral corner uses the EXACT iso-parameter
    //    octant (with its degenerate pole); any other trihedral corner uses the
    //    general fitted spherical-TRIANGLE patch; N>3 faces use the general
    //    spherical N-GON — both are the same builder (N real arc coedges, no
    //    pole edge, fitted pcurves).
    let octant_face = if orthogonal {
        // Right-handed trihedral frame + m-ordered tangent vertices, exactly as
        // the octant fast path expects (unchanged 3-face behaviour).
        let mut m = [normals[0], normals[1], normals[2]];
        if m[0].cross(m[1]).dot(m[2]) < 0.0 {
            m.swap(0, 1);
        }
        let mut oct_ids = [0u64; 3];
        let mut oct_pts = [Vec3::default(); 3];
        for k in 0..3 {
            let ideal = c.add(m[k].scale(radius));
            let found = result
                .vertices
                .iter()
                .find(|v| v.point.sub(ideal).length() < 1e-6)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Fragment,
                        "octant_vertex",
                        format!("round_convex_corner: no octant tangent vertex near {ideal:?}"),
                    )
                })?;
            oct_ids[k] = found.id;
            oct_pts[k] = found.point;
        }
        let (face, pole_edge) = build_octant_face(
            c,
            m,
            radius,
            oct_ids,
            oct_pts,
            &fresh_arcs,
            name,
            &mut next_id,
        )?;
        result.edges.push(pole_edge);
        face
    } else {
        build_general_corner_patch(c, radius, &normals, &fresh_edges, true, name, &mut next_id)?
    };

    // 5. Splice everything in: new edges, rebuilt fillet loops, drop caps, add
    //    the octant face.
    result.edges.extend(fresh_edges);
    for (si, shell) in result.shells.iter_mut().enumerate() {
        shell.faces.retain(|f| !cap_face_ids.contains(&f.id));
        for face in shell.faces.iter_mut() {
            if let Some(coedges) = new_loops.get(&face.id) {
                face.loops[0].coedges = coedges.clone();
            }
        }
        if si == octant_shell {
            shell.faces.push(octant_face.clone());
        }
    }

    // Remove now-orphaned edges (used by no coedge) and vertices (referenced by
    // no surviving edge).
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

    // 6. Validate; surface the issues to the caller if any.
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!("round_convex_corner: {issues:?}"),
        ));
    }
    Ok(result)
}
