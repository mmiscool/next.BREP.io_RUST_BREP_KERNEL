use super::*;

/// Recognize a face surface as a CYLINDER wall carrier: `(axis point, unit
/// axis, radius)`.  Covers both analytic spellings a cylindrical wall takes
/// in this kernel: a full [`AnalyticSurface::RuledRevolution`] with equal end
/// radii, and a partial [`AnalyticSurface::Revolution`] whose generatrix
/// keeps a CONSTANT radial distance from the axis (the extruded-arc wall of
/// the §6.9.7 curved-wall fixtures is recognized as the latter).
pub(super) fn cylinder_wall_carrier(surface: &crate::NurbsSurface) -> Option<(Vec3, Vec3, f64)> {
    match surface.analytic()? {
        crate::AnalyticSurface::RuledRevolution {
            frame, rho0, rho1, ..
        } => {
            let radius = 0.5 * (rho0 + rho1);
            if (rho0 - rho1).abs() > 1e-9 * radius.abs().max(1.0) || !(radius > 0.0) {
                return None;
            }
            Some((frame.origin, frame.axis, radius))
        }
        crate::AnalyticSurface::Revolution {
            frame, generatrix, ..
        } => {
            let [g0, g1] = generatrix.domain().ok()?;
            let mut radius = None;
            for k in 0..=8 {
                let t = g0 + (g1 - g0) * k as f64 / 8.0;
                let p = generatrix.evaluate(t).ok()?;
                let rel = p.sub(frame.origin);
                let radial = rel.sub(frame.axis.scale(rel.dot(frame.axis))).length();
                match radius {
                    None => radius = Some(radial),
                    Some(r0) if (radial - r0).abs() > 1e-9 * r0.max(1.0) => return None,
                    Some(_) => {}
                }
            }
            let radius = radius?;
            if !(radius > 0.0) {
                return None;
            }
            Some((frame.origin, frame.axis, radius))
        }
        _ => None,
    }
}

/// Candidate centres for the §6.9.7 CURVED-WALL corner ball: the radius-`r`
/// ball tangent to two planar walls through `corner` (outward normals `n1`,
/// `n2`) and to one cylindrical wall (`axis_point`/`axis_dir`, radius
/// `cyl_radius`; `outward_away` = the wall's outward normal points away from
/// the axis, i.e. material inside the cylinder).  The centre lies on the line
/// where the two r-offset planes intersect, at axis distance R−r
/// (`outward_away`) or R+r (a bore wall); that quadratic has up to two roots.
/// Returns `(centre, outward contact normal on the cylinder)` per root,
/// nearest-the-corner first.
pub(super) fn curved_wall_ball_candidates(
    corner: Vec3,
    radius: f64,
    n1: Vec3,
    n2: Vec3,
    axis_point: Vec3,
    axis_dir: Vec3,
    cyl_radius: f64,
    outward_away: bool,
) -> Result<Vec<(Vec3, Vec3)>, KernelRefusal> {
    // Three offset carriers, all through the shared closed forms (audit §11):
    // the ball centre sits at the CONVEX offset `−r` — inside the material —
    // from both planar walls and from the cylindrical one. Offsetting the
    // cylinder first keeps this refusal ahead of the wall-pair solve, as it was.
    let rho = offset_cylinder_radius(cyl_radius, outward_away, -radius).map_err(|_| {
        KernelRefusal::input(
            KernelStage::Classify,
            "radius_swallows_wall",
            format!("corner ball radius {radius} swallows the cylindrical wall (radius {cyl_radius})"),
        )
    })?;
    // The two `−r`-offset planes meet in a LINE, anchored at the foot of the
    // perpendicular from the corner.
    let line = offset_plane_pair(corner, n1, -radius, n2, -radius).map_err(|why| match why {
        OffsetPairDegeneracy::ParallelPlanes => KernelRefusal::ill_posed(
            KernelStage::Classify,
            "parallel_walls",
            "the two planar walls at the corner are parallel",
        ),
        // The pre-slice line was `solve_small(..)?`, so this refusal text was
        // `solve_small`'s own. Kept verbatim rather than improved: it is
        // unreachable above the parallel gate, and this slice moves no message.
        _ => KernelRefusal::internal(
            KernelStage::Classify,
            "wall_pair_singular",
            "solve_small: singular matrix",
        ),
    })?;
    offset_pair_diag::plane_pair(
        "mixed_support::wall_pair_line",
        corner,
        n1,
        -radius,
        n2,
        -radius,
        line.direction,
        Some(&line),
    );
    // Axis-distance condition along the line: |(p0 + t·dl − A)⊥axis| = rho.
    // The line is NOT constrained perpendicular to the axis here, so this is
    // the general-leading-coefficient lane; both roots come back and the choice
    // between them is made below, by distance to the corner.
    let roots = line_meets_offset_cylinder(&line, axis_point, axis_dir, rho);
    offset_pair_diag::line_cylinder(
        "mixed_support::ball_centres",
        &line,
        axis_point,
        axis_dir,
        rho,
        roots.as_ref().ok(),
    );
    let roots = roots.map_err(|why| match why {
        OffsetPairDegeneracy::LineParallelToAxis => KernelRefusal::ill_posed(
            KernelStage::Classify,
            "line_along_axis",
            "the wall-intersection line is parallel to the cylindrical wall's axis",
        ),
        _ => KernelRefusal::unsupported(
            KernelStage::Classify,
            "no_curved_wall_ball",
            format!(
                "no ball of radius {radius} is tangent to both planar walls and the cylindrical wall"
            ),
        ),
    })?;
    let mut candidates = Vec::with_capacity(2);
    for centre in roots {
        let rel = centre.sub(axis_point);
        let Ok(radial) = rel.sub(axis_dir.scale(rel.dot(axis_dir))).normalized() else {
            continue;
        };
        let n_contact = if outward_away {
            radial
        } else {
            radial.scale(-1.0)
        };
        candidates.push((centre, n_contact));
    }
    candidates.sort_by(|p, q| {
        p.0.sub(corner)
            .length()
            .total_cmp(&q.0.sub(corner).length())
    });
    candidates.dedup_by(|p, q| p.0.sub(q.0).length() < 1e-9 * radius);
    Ok(candidates)
}

/// Closest points between two (infinite) lines `p1 + s·d1` and `p2 + t·d2`.
/// Returns `None` for (near-)parallel lines.
pub(super) fn line_closest_points(p1: Vec3, d1: Vec3, p2: Vec3, d2: Vec3) -> Option<(Vec3, Vec3)> {
    let w0 = p1.sub(p2);
    let a = d1.dot(d1);
    let b = d1.dot(d2);
    let c = d2.dot(d2);
    let d = d1.dot(w0);
    let e = d2.dot(w0);
    let denom = a * c - b * b;
    if denom.abs() <= 1e-12 * a * c {
        return None;
    }
    let s = (b * e - c * d) / denom;
    let t = (a * e - b * d) / denom;
    Some((p1.add(d1.scale(s)), p2.add(d2.scale(t))))
}

/// Locate an existing mixed-corner junction arc between vertices `vid` and
/// `wid` (either stored direction): a radius-`radius` arc centred at
/// `centre`, verified by its midpoint.
pub(super) fn locate_mixed_junction_arc(
    solid: &BrepSolid,
    ctx: &str,
    vid: u64,
    wid: u64,
    centre: Vec3,
    radius: f64,
) -> Result<u64, KernelRefusal> {
    let point_of = |id: u64| -> Result<Vec3, KernelRefusal> {
        solid
            .vertices
            .iter()
            .find(|v| v.id == id)
            .map(|v| v.point)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "junction_vertex",
                    format!("{ctx}: junction vertex {id} vanished"),
                )
            })
    };
    let vpt = point_of(vid)?;
    let wpt = point_of(wid)?;
    let mid_dir = vpt
        .sub(centre)
        .normalized()
        .or_refuse(KernelStage::Classify, "normalized")?
        .add(
            wpt.sub(centre)
                .normalized()
                .or_refuse(KernelStage::Classify, "normalized")?,
        )
        .normalized()
        .or_refuse(KernelStage::Classify, "normalized")?;
    let expected_mid = centre.add(mid_dir.scale(radius));
    solid
        .edges
        .iter()
        .find(|e| {
            ((e.start_vertex_id == vid && e.end_vertex_id == wid)
                || (e.start_vertex_id == wid && e.end_vertex_id == vid))
                && e.curve
                    .evaluate((e.t0 + e.t1) * 0.5)
                    .map(|p| p.sub(expected_mid).length() < 1e-6 * (1.0 + radius))
                    .unwrap_or(false)
        })
        .map(|e| e.id)
        .ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Classify,
                "junction_arc",
                format!(
                    "{ctx}: no junction arc between vertices {vid} and \
                 {wid} (expected a radius-{radius} arc centred at {centre:?})"
                ),
            )
        })
}

/// The stored traversal sense of the (unique) coedge of face `face_id` on
/// edge `edge_id` — the mixed-corner patch must use the opposite sense on the
/// shared junction arc.
pub(super) fn coedge_sense_on(
    solid: &BrepSolid,
    ctx: &str,
    face_id: u64,
    edge_id: u64,
) -> Result<bool, KernelRefusal> {
    solid
        .shells
        .iter()
        .flat_map(|s| &s.faces)
        .filter(|f| f.id == face_id)
        .flat_map(|f| f.loops.iter().flat_map(|l| &l.coedges))
        .find(|co| co.edge_id == edge_id)
        .map(|co| co.forward)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "junction_coedge",
                format!("{ctx}: junction coedge vanished"),
            )
        })
}

/// Rebuild one face's loop for the mixed-corner surgeries: replace its
/// contiguous corner-side coedge run with ONE fresh arc T_a→T_b about
/// `arc_centre` (same prefix/run/suffix scheme as the convex corner path).
/// Returns the rebuilt coedge list and the fresh EdgeRecord.
///
/// Pcurve lane per carrier: a PLANE maps the arc through its affine
/// parameterization (a straight uv segment would drop the sagitta); an
/// ANALYTIC blend carrier takes a straight parameter line (the fresh arc is
/// an iso-line sharing the carrier's own rational-quadratic angle
/// parameterization); a fitted FREEFORM carrier (a §4.9 march surface — the
/// plane×cylinder concave blend of the curved-wall corner) gets a projection
/// FIT, because its row parameterization is not the arc's angle
/// parameterization and a straight parameter line would misparameterize the
/// boundary.
#[allow(clippy::too_many_arguments)]
pub(super) fn rebuild_mixed_corner_run(
    ctx: &str,
    face: &FaceRecord,
    is_corner: &[bool],
    expected_ends: [u64; 2],
    arc_centre: Vec3,
    arc_radius: f64,
    vpoint: &rustc_hash::FxHashMap<u64, Vec3>,
    edge_ends: &rustc_hash::FxHashMap<u64, (u64, u64)>,
    next_id: &mut dyn FnMut() -> u64,
) -> Result<(Vec<CoedgeRecord>, EdgeRecord), KernelRefusal> {
    let coedges = &face.loops[0].coedges;
    let n = coedges.len();
    let anchor = (0..n).find(|&i| !is_corner[i]).ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Fragment,
            "all_corner_end",
            format!("{ctx}: face {} is all corner-end", face.id),
        )
    })?;
    let order: Vec<usize> = (0..n).map(|k| (anchor + k) % n).collect();
    let mut prefix: Vec<CoedgeRecord> = Vec::new();
    let mut suffix: Vec<CoedgeRecord> = Vec::new();
    let mut run = 0usize;
    let mut seen_run = false;
    for &idx in &order {
        if is_corner[idx] {
            seen_run = true;
            run += 1;
        } else if !seen_run {
            prefix.push(coedges[idx].clone());
        } else {
            suffix.push(coedges[idx].clone());
        }
    }
    if run == 0 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Fragment,
            "no_corner_run",
            format!("{ctx}: face {} has no corner-end coedge", face.id),
        ));
    }
    let before = prefix.last().ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Fragment,
            "run_predecessor",
            format!("{ctx}: face {} corner run has no predecessor", face.id),
        )
    })?;
    let (bs, be) = *edge_ends.get(&before.edge_id).unwrap();
    let t_a = if before.forward { be } else { bs };
    let uv_a = before
        .pcurve
        .evaluate(1.0)
        .or_refuse(KernelStage::Fragment, "evaluate")?;
    let after = suffix
        .first()
        .or_else(|| prefix.first())
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Fragment,
                "run_successor",
                format!("{ctx}: face {} corner run has no successor", face.id),
            )
        })?;
    let (as_, ae) = *edge_ends.get(&after.edge_id).unwrap();
    let t_b = if after.forward { as_ } else { ae };
    let uv_b = after
        .pcurve
        .evaluate(0.0)
        .or_refuse(KernelStage::Fragment, "evaluate")?;
    if !(expected_ends.contains(&t_a) && expected_ends.contains(&t_b)) || t_a == t_b {
        return Err(KernelRefusal::unsupported(
            KernelStage::Fragment,
            "run_ends",
            format!(
                "{ctx}: face {} corner run ends ({t_a},{t_b}) are not \
             the expected boundary vertices {expected_ends:?}",
                face.id
            ),
        ));
    }
    // Fresh arc T_a→T_b about `arc_centre` (standard construction, so its
    // parameterization matches both the trimmed carrier's profile and the
    // torus patch's revolution parameter — pcurves stay parameter lines).
    let ta_pt = *vpoint.get(&t_a).unwrap();
    let tb_pt = *vpoint.get(&t_b).unwrap();
    let x_axis = ta_pt
        .sub(arc_centre)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let rb = tb_pt.sub(arc_centre);
    let y_axis = rb
        .sub(x_axis.scale(rb.dot(x_axis)))
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let sweep = x_axis
        .dot(rb.normalized().or_refuse(KernelStage::Fragment, "normalized")?)
        .clamp(-1.0, 1.0)
        .acos();
    let curve = crate::make_arc(arc_centre, x_axis, y_axis, arc_radius, 0.0, sweep)
        .or_refuse(KernelStage::Fragment, "make_arc")?;
    let pcurve = if matches!(
        face.surface.analytic(),
        Some(crate::AnalyticSurface::Plane { .. })
    ) {
        crate::pcurve::build_pcurve_on_surface(&face.surface, &curve)
            .or_refuse(KernelStage::Fragment, "build_pcurve")?
    } else if face.surface.analytic().is_some() {
        crate::sweep_topology::parameter_line(uv_a.x, uv_a.y, uv_b.x, uv_b.y)
            .or_refuse(KernelStage::Fragment, "parameter_line")?
    } else {
        crate::pcurve::build_pcurve_on_surface_range(
            &face.surface,
            &curve,
            0.0,
            1.0,
            true,
            1e-9 * (1.0 + arc_radius),
        )
        .or_refuse(KernelStage::Fragment, "build_pcurve")?
    };
    let edge = EdgeRecord {
        id: next_id(),
        curve,
        t0: 0.0,
        t1: 1.0,
        start_vertex_id: t_a,
        end_vertex_id: t_b,
        degenerate: false,
        name: None,
    };
    let fresh_co = CoedgeRecord {
        id: next_id(),
        edge_id: edge.id,
        forward: true,
        pcurve,
    };
    let mut rebuilt = prefix;
    rebuilt.push(fresh_co);
    rebuilt.extend(suffix);
    Ok((rebuilt, edge))
}

/// Build the mixed-corner TORUS-SECTOR patch face shared by the planar and
/// curved-wall mixed-convexity vertex blends.  Generatrix = the A-side
/// junction arc's own curve (so the u=0 boundary reproduces it EXACTLY),
/// revolved about the concave blend's axis by the sector angle to the B-side
/// junction plane.  The four boundary coedges reuse the shared EdgeRecords
/// (junction arcs A/B, the fresh inner-equator V arc, the fresh cap-contact
/// W arc) with the sense opposite their carrier neighbours.
#[allow(clippy::too_many_arguments)]
pub(super) fn build_mixed_sector_patch(
    ctx: &str,
    arc_a: &EdgeRecord,
    arc_b: &EdgeRecord,
    fresh_v_arc: &EdgeRecord,
    fresh_w_arc: &EdgeRecord,
    cyl_a_forward: bool,
    cyl_b_forward: bool,
    v_a: (u64, Vec3),
    v_b: (u64, Vec3),
    w_a: (u64, Vec3),
    w_b: (u64, Vec3),
    axis_point: Vec3,
    axis_dir: Vec3,
    a_axis_closest: Vec3,
    b_axis_closest: Vec3,
    radius: f64,
    name: Option<&str>,
    next_id: &mut dyn FnMut() -> u64,
) -> Result<FaceRecord, KernelRefusal> {
    let q_v = axis_point.add(axis_dir.scale(v_a.1.sub(axis_point).dot(axis_dir)));
    let da = a_axis_closest
        .sub(q_v)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let db = b_axis_closest
        .sub(q_v)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    // A regular mixed trihedral has two distinct inner-equator vertices.
    // The two-stripe re-entrant rim closure is the horn-torus limit: both
    // inner vertices collapse to the same pole on the revolution axis.  In
    // that limit use the rolling-centre directions to define the sector.
    let ra = v_a.1.sub(q_v).normalized().unwrap_or(da);
    let rb = v_b.1.sub(q_v).normalized().unwrap_or(db);
    let sweep = ra.dot(rb).clamp(-1.0, 1.0).acos();
    if sweep < 1e-9 {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "sector_sweep",
            format!("{ctx}: degenerate sector sweep"),
        ));
    }
    let revolve_axis = if ra.cross(rb).dot(axis_dir) > 0.0 {
        axis_dir
    } else {
        axis_dir.scale(-1.0)
    };
    let surface = crate::make_revolution(axis_point, revolve_axis, &arc_a.curve, sweep)
        .or_refuse(KernelStage::Fragment, "make_revolution")?;

    // v-parameters of the V (inner) and W (cap) boundary levels on the
    // generatrix, and the arc_a traversal direction.
    let (v_at_v, v_at_w) = if arc_a.start_vertex_id == v_a.0 {
        (arc_a.t0, arc_a.t1)
    } else {
        (arc_a.t1, arc_a.t0)
    };

    // Patch boundary traversal is forced by the neighbours: opposite sense on
    // each shared edge.  Chain the four sides into a closed loop.
    let pl = |u0: f64, v0: f64, u1: f64, v1: f64| {
        crate::sweep_topology::parameter_line(u0, v0, u1, v1)
            .or_refuse(KernelStage::Fragment, "parameter_line")
    };
    let patch_a_forward = !cyl_a_forward;
    let patch_b_forward = !cyl_b_forward;
    // arc_a runs along u=0; its pcurve follows the traversal direction.
    let (a_pc_v0, a_pc_v1) = if patch_a_forward {
        (arc_a.t0, arc_a.t1)
    } else {
        (arc_a.t1, arc_a.t0)
    };
    let a_start = if patch_a_forward {
        arc_a.start_vertex_id
    } else {
        arc_a.end_vertex_id
    };
    let a_end = if patch_a_forward {
        arc_a.end_vertex_id
    } else {
        arc_a.start_vertex_id
    };
    // arc_b runs along u=1; its parameterization is not guaranteed to match
    // the revolved generatrix, so FIT its pcurve from samples.
    let fit_tol = 1e-5 * (1.0 + radius);
    let b_pcurve_forward = crate::pcurve::build_pcurve_on_surface_range(
        &surface,
        &arc_b.curve,
        arc_b.t0.min(arc_b.t1),
        arc_b.t0.max(arc_b.t1),
        patch_b_forward,
        fit_tol,
    )
    .or_refuse(KernelStage::Fragment, "build_pcurve")?;
    let b_start = if patch_b_forward {
        arc_b.start_vertex_id
    } else {
        arc_b.end_vertex_id
    };
    let b_end = if patch_b_forward {
        arc_b.end_vertex_id
    } else {
        arc_b.start_vertex_id
    };
    // Fresh arcs: patch uses the OPPOSITE sense of the trimmed carrier's
    // fresh coedge (which is forward=true), so forward=false — traversal
    // T_b→T_a.  The stored arcs share the standard rational-quadratic angle
    // parameterization with the revolution (same construction), and that
    // parameterization is symmetric under reversal, so straight parameter
    // lines are exact.
    let u_of = |vid: u64, level_pts: [(u64, Vec3); 2]| -> f64 {
        // u=0 at the A-side of the sector, u=1 at the B-side.
        if vid == level_pts[0].0 {
            0.0
        } else {
            1.0
        }
    };
    let v_arc_start = fresh_v_arc.end_vertex_id; // traversal T_b→T_a
    let v_arc_end = fresh_v_arc.start_vertex_id;
    let v_arc_pc = if v_a.0 == v_b.0 {
        // Degenerate horn-torus pole.  The 3D edge has zero length, while its
        // pcurve spans the full sector exactly like a sphere pole edge.
        pl(1.0, v_at_v, 0.0, v_at_v)?
    } else {
        pl(
            u_of(v_arc_start, [v_a, v_b]),
            v_at_v,
            u_of(v_arc_end, [v_a, v_b]),
            v_at_v,
        )?
    };
    let w_arc_start = fresh_w_arc.end_vertex_id;
    let w_arc_end = fresh_w_arc.start_vertex_id;
    let w_arc_pc = pl(
        u_of(w_arc_start, [w_a, w_b]),
        v_at_w,
        u_of(w_arc_end, [w_a, w_b]),
        v_at_w,
    )?;

    struct Side {
        edge_id: u64,
        forward: bool,
        start: u64,
        end: u64,
        pcurve: NurbsCurve,
    }
    let sides = vec![
        Side {
            edge_id: arc_a.id,
            forward: patch_a_forward,
            start: a_start,
            end: a_end,
            pcurve: pl(0.0, a_pc_v0, 0.0, a_pc_v1)?,
        },
        Side {
            edge_id: fresh_v_arc.id,
            forward: false,
            start: v_arc_start,
            end: v_arc_end,
            pcurve: v_arc_pc,
        },
        Side {
            edge_id: arc_b.id,
            forward: patch_b_forward,
            start: b_start,
            end: b_end,
            pcurve: b_pcurve_forward,
        },
        Side {
            edge_id: fresh_w_arc.id,
            forward: false,
            start: w_arc_start,
            end: w_arc_end,
            pcurve: w_arc_pc,
        },
    ];
    // Chain into a closed 4-loop.
    let mut order: Vec<usize> = vec![0];
    let mut used = [true, false, false, false];
    let mut cursor = sides[0].end;
    for _ in 0..3 {
        let next_side = (0..4)
            .find(|&s| !used[s] && sides[s].start == cursor)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "sector_chain",
                    format!("{ctx}: torus patch boundary does not chain"),
                )
            })?;
        used[next_side] = true;
        cursor = sides[next_side].end;
        order.push(next_side);
    }
    if cursor != sides[0].start {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "sector_loop",
            format!("{ctx}: torus patch boundary is not closed"),
        ));
    }
    let coedges: Vec<CoedgeRecord> = order
        .iter()
        .map(|&s| CoedgeRecord {
            id: next_id(),
            edge_id: sides[s].edge_id,
            forward: sides[s].forward,
            pcurve: sides[s].pcurve.clone(),
        })
        .collect();

    // same_sense: outward = away from the rolling ball's centre at the
    // sector midpoint.
    let v_mid = 0.5 * (arc_a.t0 + arc_a.t1);
    let mid_point = surface
        .evaluate(0.5, v_mid)
        .or_refuse(KernelStage::Fragment, "evaluate")?;
    let major_radius = a_axis_closest.sub(q_v).length();
    let mid_centre = q_v.add(
        da.add(db)
            .normalized()
            .or_refuse(KernelStage::Fragment, "normalized")?
            .scale(major_radius),
    );
    let outward = mid_point
        .sub(mid_centre)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let surface_normal = surface
        .normal(0.5, v_mid)
        .or_refuse(KernelStage::Fragment, "normal")?;
    let same_sense = surface_normal.dot(outward) > 0.0;
    let loop_id = next_id();
    Ok(FaceRecord {
        id: next_id(),
        surface,
        same_sense,
        loops: vec![LoopRecord {
            id: loop_id,
            coedges,
        }],
        name: name.map(|value| value.to_string()),
    })
}
