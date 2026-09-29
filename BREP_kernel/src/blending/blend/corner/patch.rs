use super::*;

// ======================================================================
// §6.9.7 vertex ("star") blend — convex trihedral corner.
//
// When three edges meeting at a convex vertex are filleted, the rolling
// ball tangent to all three faces sits at ONE position and its sphere is
// the vertex patch (Golovanov §6.9.7). The patch is a spherical octant:
// with the sphere centered at C = P − r·Σ mᵢ (mᵢ = outward face normals),
// its three corners are the tangent points Tᵢ = C + r·mᵢ and its three
// boundary arcs are the tangent circles sphere∩cylinderᵢ, which lie in
// the coordinate planes through C (arc T₁T₂ = sphere∩(edge-3 cylinder),
// in the plane ⟂ m₃, etc.). Built as a sub-rectangle u∈[0,¼], v∈[½,1] of
// a sphere revolution with polar axis m₃ and x-axis m₁: the three arcs
// are exact iso-parameter curves (two meridians + the equator) and the
// v=1 pole is a degenerate edge at T₃. No booleans — the tangent
// surfaces are never intersected, only constructed and sewn.

/// Build ONLY the spherical-octant `FaceRecord` (plus its degenerate pole
/// edge) for a vertex blend, reusing the three EXISTING tangent vertices
/// `t_ids` and the three FRESH tangent-circle arc edges in `arcs`
/// (`(edge_id, start_vertex, end_vertex)`).  The loop, pcurves and
/// same_sense are exactly the spherical-octant construction described above;
/// each arc coedge's
/// `forward` is derived from the stored edge orientation so the octant use
/// is opposite to the fillet use (shared edge, two coedges, opposite sense).
pub(super) fn build_octant_face(
    center: Vec3,
    normals: [Vec3; 3],
    radius: f64,
    t_ids: [u64; 3],
    t_pts: [Vec3; 3],
    arcs: &[(u64, u64, u64)],
    name: Option<&str>,
    next_id: &mut dyn FnMut() -> u64,
) -> Result<(FaceRecord, EdgeRecord), KernelRefusal> {
    let [m1, m2, m3] = normals;
    if m1.cross(m2).dot(m3) < 0.5 {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "octant_frame",
            "round_convex_corner: octant normals are not right-handed",
        ));
    }
    let half = std::f64::consts::FRAC_PI_2;
    let meridian = crate::make_arc(center, m1, m3, radius, -half, half)
        .or_refuse(KernelStage::Fragment, "make_arc")?;
    let surface = crate::make_revolution(center, m3, &meridian, std::f64::consts::TAU)
        .or_refuse(KernelStage::Fragment, "make_revolution")?;

    let [t1, t2, t3] = t_ids;
    let e_pole = next_id();
    let pole_edge = EdgeRecord {
        id: e_pole,
        curve: crate::make_line(t_pts[2], t_pts[2]).or_refuse(KernelStage::Fragment, "make_line")?,
        t0: 0.0,
        t1: 1.0,
        start_vertex_id: t3,
        end_vertex_id: t3,
        degenerate: true,
        name: None,
    };

    // For a needed traversal `from -> to`, locate the fresh arc joining that
    // pair and report whether its stored orientation already runs that way.
    let find = |from: u64, to: u64| -> Result<(u64, bool), KernelRefusal> {
        for &(eid, s, e) in arcs {
            if s == from && e == to {
                return Ok((eid, true));
            }
            if s == to && e == from {
                return Ok((eid, false));
            }
        }
        Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "tangent_arc",
            format!("round_convex_corner: no tangent arc between vertices {from} and {to}"),
        ))
    };
    let (e12, f12) = find(t1, t2)?; // equator T1->T2  (arc ⟂ m3)
    let (e23, f23) = find(t2, t3)?; // meridian u=0.25 T2->T3 (arc ⟂ m1)
    let (e31, f31) = find(t3, t1)?; // meridian u=0    T3->T1 (arc ⟂ m2)

    let pl = |u0, v0, u1, v1| {
        crate::sweep_topology::parameter_line(u0, v0, u1, v1)
            .or_refuse(KernelStage::Fragment, "parameter_line")
    };
    let loop_id = next_id();
    let coedges = vec![
        CoedgeRecord {
            id: next_id(),
            edge_id: e12,
            forward: f12,
            pcurve: pl(0.0, 0.5, 0.25, 0.5)?,
        },
        CoedgeRecord {
            id: next_id(),
            edge_id: e23,
            forward: f23,
            pcurve: pl(0.25, 0.5, 0.25, 1.0)?,
        },
        CoedgeRecord {
            id: next_id(),
            edge_id: e_pole,
            forward: true,
            pcurve: pl(0.25, 1.0, 0.0, 1.0)?,
        },
        CoedgeRecord {
            id: next_id(),
            edge_id: e31,
            forward: f31,
            pcurve: pl(0.0, 1.0, 0.0, 0.5)?,
        },
    ];

    let mid = surface
        .evaluate(0.125, 0.75)
        .or_refuse(KernelStage::Fragment, "evaluate")?;
    let surface_normal = surface
        .normal(0.125, 0.75)
        .or_refuse(KernelStage::Fragment, "normal")?;
    let outward = mid
        .sub(center)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let same_sense = surface_normal.dot(outward) > 0.0;

    let face = FaceRecord {
        id: next_id(),
        surface,
        same_sense,
        loops: vec![LoopRecord {
            id: loop_id,
            coedges,
        }],
        name: name.map(|value| value.to_string()),
    };
    Ok((face, pole_edge))
}

/// General convex-corner vertex patch for a NON-orthogonal trihedral corner:
/// a spherical triangle on the ball (`center`, `radius`) bounded by the three
/// FRESH tangent-circle arcs in `arc_edges`.  Unlike [`build_octant_face`],
/// the three boundary arcs are great circles that are NOT iso-parameter curves
/// of any single revolution parameterization, so each coedge pcurve is FITTED
/// by projecting samples of its arc onto the sphere.  The revolution frame is
/// chosen to keep the whole patch OFF the u-seam and OFF both poles.
///
/// `outward` says which way the material lies: a CONVEX corner's ball sits
/// inside the material and the patch faces away from the centre; a CONCAVE
/// corner's ball sits in the air and the patch faces toward it.
pub(in crate::blend) fn build_general_corner_patch(
    center: Vec3,
    radius: f64,
    normals: &[Vec3],
    arc_edges: &[EdgeRecord],
    outward: bool,
    name: Option<&str>,
    next_id: &mut dyn FnMut() -> u64,
) -> Result<FaceRecord, KernelRefusal> {
    use std::f64::consts::{FRAC_PI_2, PI, TAU};
    let n_faces = normals.len();
    if n_faces < 3 || arc_edges.len() != n_faces {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "arc_count",
            format!(
                "build_general_corner_patch: expected N≥3 faces with one arc each, \
             found {n_faces} faces and {} arcs",
                arc_edges.len()
            ),
        ));
    }
    // Patch centre direction (radially outward through the middle of the
    // spherical N-gon); every tangent point Tᵢ = C + r·nᵢ lies within 90° of
    // it, so a seam placed opposite pc keeps the whole patch off u=0.
    let pc = normals
        .iter()
        .fold(Vec3::default(), |acc, n| acc.add(*n))
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;

    // Revolution frame: polar axis ⟂ pc so the patch straddles the equator
    // (away from both poles).  Among all such axes pick the one that keeps the
    // N tangent points AND the great-circle arc midpoints (normalize(nᵢ+nⱼ))
    // nearest the equator — minimizing the worst |d·polar|.  x-axis = -pc puts
    // the u=0 seam on the far side (patch lands near u≈0.5).
    let e0 = pc
        .perpendicular()
        .or_refuse(KernelStage::Fragment, "perpendicular")?;
    let e1 = pc
        .cross(e0)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let mut crit: Vec<Vec3> = normals.to_vec();
    for i in 0..n_faces {
        for j in (i + 1)..n_faces {
            if let Ok(mid) = normals[i].add(normals[j]).normalized() {
                crit.push(mid);
            }
        }
    }
    let mut best_phi = 0.0f64;
    let mut best_max = f64::INFINITY;
    for k in 0..360 {
        let phi = PI * k as f64 / 360.0;
        let polar = e0.scale(phi.cos()).add(e1.scale(phi.sin()));
        let worst = crit
            .iter()
            .fold(0.0f64, |acc, d| acc.max(d.dot(polar).abs()));
        if worst < best_max {
            best_max = worst;
            best_phi = phi;
        }
    }
    let polar = e0
        .scale(best_phi.cos())
        .add(e1.scale(best_phi.sin()))
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let x_axis = pc.scale(-1.0);
    let meridian = crate::make_arc(center, x_axis, polar, radius, -FRAC_PI_2, FRAC_PI_2)
        .or_refuse(KernelStage::Fragment, "make_arc")?;
    let surface = crate::make_revolution(center, polar, &meridian, TAU)
        .or_refuse(KernelStage::Fragment, "make_revolution")?;

    // Chain the three arcs into a closed loop traversed OPPOSITE to the fillet
    // use: each arc is stored start->end and used forward=true by its fillet,
    // so the patch reuses it forward=false (end->start).  Following reversed
    // arcs, each successor's stored end meets the current arc's stored start.
    let mut order = vec![0usize];
    let mut used = vec![false; n_faces];
    used[0] = true;
    let mut chain_vertex = arc_edges[0].start_vertex_id;
    for _ in 0..(n_faces - 1) {
        let next = (0..n_faces)
            .find(|&j| !used[j] && arc_edges[j].end_vertex_id == chain_vertex)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "arc_chain",
                    "build_general_corner_patch: tangent arcs do not form a closed N-gon",
                )
            })?;
        used[next] = true;
        order.push(next);
        chain_vertex = arc_edges[next].start_vertex_id;
    }
    if chain_vertex != arc_edges[0].end_vertex_id {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "arc_loop",
            "build_general_corner_patch: tangent arc loop is not closed",
        ));
    }

    // Each coedge's pcurve is the arc's track on the sphere, fitted by the one
    // verified fitter (`track_fit`): broken at the revolution's knot lines,
    // which every arc of an octant-sized patch crosses, and held to the
    // refinement floor at midpoints it did not interpolate. It used to be one
    // cubic through 33 uniform samples: exact at those samples and 1.3e-5 to
    // 1.9e-5 off in uv between them, peaking where the arc crosses a knot line,
    // which left the patch's trims 5.3e-7·r² of vector area off their arcs on a
    // 10-cube star fillet and its volume +1.09e-4 at r = 9 (measured
    // 2026-09-17). Each arc is walked in the coedge's traversal direction
    // (forward = false: fraction 0 is its end vertex).
    const PATCH_TRACK_SAMPLES: usize = 512;
    const PATCH_TRACK_SAMPLES_LIMIT: usize = 4096;
    let floor = crate::pcurve::PCURVE_REFINEMENT_TOLERANCE;
    let mut coedges = Vec::with_capacity(n_faces);
    for &ai in &order {
        let edge = &arc_edges[ai];
        let at = |fraction: f64| edge.curve.evaluate(edge.t1 - (edge.t1 - edge.t0) * fraction);
        let breaks = super::super::track_fit::curve_breaks(&edge.curve, edge.t0, edge.t1, false);
        let fit = super::super::track_fit::fit_curve_track(
            &surface,
            &at,
            &breaks,
            floor,
            PATCH_TRACK_SAMPLES,
            PATCH_TRACK_SAMPLES_LIMIT,
        )?;
        if std::env::var("BREP_DEBUG_TRACK_FIT").is_ok() {
            eprintln!(
                "TRACK_FIT corner patch arc {}: miss {:.3e} at {} samples",
                edge.id, fit.miss, fit.samples
            );
        }
        if !fit.on_floor {
            return Err(KernelRefusal::non_convergence(
                KernelStage::Refine,
                "patch_track",
                format!(
                    "{} the corner patch's pcurve along arc {} misses its arc by {:.3e} at {} \
                 samples, against a floor of {floor:.1e}",
                    crate::blend::PCURVE_OFF_FLOOR,
                    edge.id,
                    fit.miss,
                    fit.samples
                ),
            ));
        }
        coedges.push(CoedgeRecord {
            id: next_id(),
            edge_id: edge.id,
            forward: false,
            pcurve: fit.curve,
        });
    }

    // same_sense so the face normal points radially away from C for a convex
    // corner, toward it for a concave one.
    let centre_projection =
        crate::project_point_to_surface(&surface, center.add(pc.scale(radius)))
            .or_refuse(KernelStage::Fragment, "project")?;
    let surface_normal = surface
        .normal(centre_projection.u, centre_projection.v)
        .or_refuse(KernelStage::Fragment, "normal")?;
    let same_sense = (surface_normal.dot(pc) > 0.0) == outward;

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

/// The CHAMFER counterpart of [`build_general_corner_patch`]: the planar facet
/// a chamfered star closes with.
///
/// §6.11 keeps a chamfered vertex sharp only where the bevels meet in a seam.
/// At a STAR — every edge at the vertex selected — each stripe stops at the
/// station where its own rolling ball IS the corner ball, and there its cross
/// section is the CHORD between the ball's two tangency points rather than the
/// arc between them.  The N chords therefore run tangency point to tangency
/// point around the ball, one per face, and close an N-gon whose vertices all
/// lie on the ball.  For N = 3 that N-gon is a triangle, so it is planar by
/// construction and the facet is the one plane through the three tangency
/// points; the chords lie in it exactly, which is what lets the facet be sewn
/// to the stripes with no intersection anywhere.
///
/// N ≥ 4 is refused: four points on a sphere are not coplanar in general, and
/// the fill that closes a non-planar N-gon is the question
/// `general_star.rs` answers for fillets, not this one.
pub(in crate::blend) fn build_chamfer_corner_facet(
    center: Vec3,
    normals: &[Vec3],
    chord_edges: &[EdgeRecord],
    outward: bool,
    name: Option<&str>,
    next_id: &mut dyn FnMut() -> u64,
) -> Result<FaceRecord, KernelRefusal> {
    let n_faces = normals.len();
    if n_faces < 3 || chord_edges.len() != n_faces {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "chord_count",
            format!(
                "blend network: a chamfered star closes with the facet its end chords bound, which \
             needs N≥3 faces carrying one chord each — this vertex has {n_faces} faces and {} \
             sections",
                chord_edges.len()
            ),
        ));
    }
    // Outward direction through the middle of the corner, exactly as the
    // spherical patch uses it: every tangency point lies within 90 degrees of
    // it, so it fixes the facet's sense.
    let pc = normals
        .iter()
        .fold(Vec3::default(), |acc, n| acc.add(*n))
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;

    // Chain the chords into a closed loop traversed OPPOSITE to the stripes'
    // use, the same walk `build_general_corner_patch` makes over the arcs.
    let mut order = vec![0usize];
    let mut used = vec![false; n_faces];
    used[0] = true;
    let mut chain_vertex = chord_edges[0].start_vertex_id;
    for _ in 0..(n_faces - 1) {
        let next = (0..n_faces)
            .find(|&j| !used[j] && chord_edges[j].end_vertex_id == chain_vertex)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "chord_chain",
                    "blend network: the chamfer facet's sections do not form a closed N-gon",
                )
            })?;
        used[next] = true;
        order.push(next);
        chain_vertex = chord_edges[next].start_vertex_id;
    }
    if chain_vertex != chord_edges[0].end_vertex_id {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "chord_loop",
            "blend network: the chamfer facet's section loop is not closed",
        ));
    }

    // The plane the N-gon bounds. Each tangency point is `center + r·nᵢ` for
    // the one seated radius, and every face at the corner passes through the
    // corner VERTEX V, so `nᵢ·(V − center) = −r` for all of them: the points
    // all lie on the plane with normal `V − center`, whatever N is. That is an
    // identity for PLANAR supports only — a curved support puts its tangency
    // point somewhere the identity does not reach — so the N-gon's flatness is
    // measured rather than assumed, and a skew one is refused by name.
    let corner_point = |index: usize| -> Result<Vec3, KernelRefusal> {
        let edge = &chord_edges[index];
        edge.curve
            .evaluate(edge.t0)
            .or_refuse(KernelStage::Fragment, "evaluate")
    };
    let points = order
        .iter()
        .map(|&index| corner_point(index))
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let [a, b] = [points[0], points[1]];
    let u_dir = b
        .sub(a)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    // Newell's normal over the chained N-gon: exact for a planar polygon at
    // any N, and for N = 3 exactly the triangle's own `(b−a)×(c−a)`.
    let mut newell = Vec3::default();
    for index in 0..n_faces {
        let from = points[index];
        let to = points[(index + 1) % n_faces];
        newell = newell.add(from.sub(a).cross(to.sub(a)));
    }
    let normal = newell
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    let v_dir = normal
        .cross(u_dir)
        .normalized()
        .or_refuse(KernelStage::Fragment, "normalized")?;
    // The seated radius, which every tangency point shares, sets the bar the
    // N-gon's flatness is held to.
    let radius = points
        .iter()
        .fold(0.0f64, |worst, point| worst.max(point.sub(center).length()));
    let flatness = 1e-9 * (1.0 + radius);
    let skew = points
        .iter()
        .fold(0.0f64, |worst, point| worst.max(point.sub(a).dot(normal).abs()));
    if skew > flatness {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "skew_ngon",
            format!(
                "blend network: the {n_faces} chords at a chamfered star bound a SKEW N-gon \
             ({skew:.3e} out of plane against {flatness:.3e}); a non-planar corner fill is not \
             implemented"
            ),
        ));
    }
    // A patch big enough to hold the facet with room to spare, laid out so the
    // facet lands inside the parameter square (the same set-back `sew_cap`
    // uses for its bulkhead).
    let reach = points
        .iter()
        .fold(0.0f64, |worst, point| worst.max(point.sub(a).length()))
        .max(1e-6)
        * 4.0;
    let origin = a.sub(u_dir.scale(reach)).sub(v_dir.scale(reach));
    let plane = crate::make_plane(origin, u_dir, v_dir, 2.0 * reach, 2.0 * reach)
        .or_refuse(KernelStage::Fragment, "make_plane")?;
    let uv_of = |point: Vec3| -> Result<(f64, f64), KernelRefusal> {
        let projection = crate::project_point_to_surface(&plane, point)
            .or_refuse(KernelStage::Fragment, "project")?;
        Ok((projection.u, projection.v))
    };

    // A chord is a straight line and the plane's parameterisation is affine,
    // so each pcurve is the straight uv segment between the chord's ends —
    // exact, with nothing to fit. `forward = false` walks stored end -> start.
    let mut coedges = Vec::with_capacity(n_faces);
    for &index in &order {
        let edge = &chord_edges[index];
        let (u0, v0) = uv_of(
            edge.curve
                .evaluate(edge.t1)
                .or_refuse(KernelStage::Fragment, "evaluate")?,
        )?;
        let (u1, v1) = uv_of(
            edge.curve
                .evaluate(edge.t0)
                .or_refuse(KernelStage::Fragment, "evaluate")?,
        )?;
        coedges.push(CoedgeRecord {
            id: next_id(),
            edge_id: edge.id,
            forward: false,
            pcurve: crate::sweep_topology::parameter_line(u0, v0, u1, v1)
                .or_refuse(KernelStage::Fragment, "parameter_line")?,
        });
    }

    // same_sense so the face normal points away from the ball's centre at a
    // convex corner and toward it at a concave one — the patch's own rule.
    let plane_normal = plane
        .normal(0.5, 0.5)
        .or_refuse(KernelStage::Fragment, "normal")?;
    let same_sense = (plane_normal.dot(pc) > 0.0) == outward;

    let loop_id = next_id();
    Ok(FaceRecord {
        id: next_id(),
        surface: plane,
        same_sense,
        loops: vec![LoopRecord {
            id: loop_id,
            coedges,
        }],
        name: name.map(|value| value.to_string()),
    })
}
