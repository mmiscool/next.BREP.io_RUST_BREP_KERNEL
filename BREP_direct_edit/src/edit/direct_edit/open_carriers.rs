use super::*;
use crate::{ExtendRefusal, KernelRefusal, KernelStage, OrRefuse};

/// Grow a ruled-revolution neighbour along its axis so its domain covers
/// `points` (with a small margin) — the extension is EXACT (same frame, the
/// generatrix prolonged linearly) and every existing pcurve is remapped
/// affinely in v, so the face's trims are untouched geometrically. Planar or
/// already-covering carriers are left alone.
pub(super) fn extend_ruled_neighbour_over(
    solid: &mut BrepSolid,
    face_id: u64,
    points: &[Vec3],
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let op = "delete_face_and_heal";
    let (shell, face_pos) = find_face(solid, face_id).ok_or_else(|| {
        KernelRefusal::internal(KernelStage::Collect, "face", format!("{op}: missing face {face_id}"))
    })?;
    extend_ruled_carrier(&mut solid.shells[shell].faces[face_pos], points, tolerance)
}

/// The face-local half of [`extend_ruled_neighbour_over`]: everything it does
/// happens inside one `FaceRecord`, and the rejoin in `delete_faces.rs` grows a
/// candidate face it has not put into a solid yet.
pub(super) fn extend_ruled_carrier(
    face: &mut FaceRecord,
    points: &[Vec3],
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let Some(AnalyticSurface::RuledRevolution {
        frame,
        rho0,
        rho1,
        height,
    }) = face.surface.analytic().cloned()
    else {
        return Ok(());
    };
    let mut low = 0.0f64;
    let mut high = height;
    for point in points {
        let axial = point.sub(frame.origin).dot(frame.axis);
        low = low.min(axial - tolerance);
        high = high.max(axial + tolerance);
    }
    if low >= -tolerance && high <= height + tolerance {
        return Ok(());
    }
    census_push("ruled_extension", || serde_json::json!({ "low": low, "high": high, "height": height }));
    let rho_at = |z: f64| rho0 + (rho1 - rho0) * z / height;
    let base = frame.origin.add(frame.axis.scale(low));
    let start = base.add(frame.x_axis.scale(rho_at(low)));
    let end = frame
        .origin
        .add(frame.axis.scale(high))
        .add(frame.x_axis.scale(rho_at(high)));
    let generatrix = make_line(start, end).or_refuse(KernelStage::Sew, "make_line")?;
    let extended = make_revolution(base, frame.axis, &generatrix, std::f64::consts::TAU)
        .or_refuse(KernelStage::Sew, "make_revolution")?;
    // v remap old -> new: z = old_v * height; new_v = (z - low)/(high - low).
    let v_scale = height / (high - low);
    let v_offset = -low / (high - low);
    face.surface = extended;
    for coedge in face
        .loops
        .iter_mut()
        .flat_map(|loop_record| &mut loop_record.coedges)
    {
        for control in &mut coedge.pcurve.control_points {
            control.y = control.y * v_scale + control.w * v_offset;
        }
    }
    Ok(())
}

/// Grow a partial-sweep straight-generatrix `Revolution` neighbour (a fillet
/// band / partial cylinder-or-cone wall) along its axis so its domain covers
/// `points`. `extend_ruled_neighbour_over` only knows the full-2π
/// `RuledRevolution` variant and no-ops on a partial `Revolution`; this is its
/// partial-sweep analogue. The straight generatrix is prolonged along its own
/// 3D line — which keeps its azimuth and radius law exactly — to the required
/// axial span, and the band is re-revolved over the SAME sweep. `retrim_ruled_face`
/// rebuilds every pcurve from scratch on the grown surface afterwards, so no
/// v-remap is needed here. A non-`Revolution` surface, or one that already
/// covers the boundary, is left untouched.
pub(super) fn extend_revolution_carrier_over(
    solid: &mut BrepSolid,
    face_id: u64,
    points: &[Vec3],
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let (shell, face_pos) = find_face(solid, face_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "ruled_face",
            format!("move_faces: missing ruled face {face_id}"),
        )
    })?;
    extend_revolution_carrier(&mut solid.shells[shell].faces[face_pos], points, tolerance)
}

/// The face-local half of [`extend_revolution_carrier_over`] — see
/// [`extend_ruled_carrier`] for why the rejoin needs one.
pub(super) fn extend_revolution_carrier(
    face: &mut FaceRecord,
    points: &[Vec3],
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let Some(AnalyticSurface::Revolution {
        frame,
        sweep,
        generatrix,
        ..
    }) = face.surface.analytic().cloned()
    else {
        return Ok(());
    };
    let controls = &generatrix.control_points;
    if generatrix.degree != 1 || controls.len() != 2 {
        return Ok(()); // curved generatrix — not a ruled band, nothing to grow
    }
    let p0 = controls[0].point().or_refuse(KernelStage::Collect, "control_point")?;
    let p1 = controls[1].point().or_refuse(KernelStage::Collect, "control_point")?;
    let axial = |p: Vec3| p.sub(frame.origin).dot(frame.axis);
    let (z0, z1) = (axial(p0), axial(p1));
    let (span_lo, span_hi) = (z0.min(z1), z0.max(z1));
    let mut low = span_lo;
    let mut high = span_hi;
    for &point in points {
        let a = axial(point);
        low = low.min(a - tolerance);
        high = high.max(a + tolerance);
    }
    if low >= span_lo - tolerance && high <= span_hi + tolerance {
        return Ok(()); // the current band already covers the moved boundary
    }
    let denom = z1 - z0;
    if denom.abs() <= tolerance {
        return Ok(());
    }
    // Prolong the generatrix line to the required axial span (same 3D line ⇒
    // same azimuth and radius law). Recognition validated the carrier by exact
    // reconstruction, so `make_revolution(frame.origin, frame.axis, generatrix,
    // sweep)` reproduces the SOURCE surface bit-for-bit — including its normal
    // orientation — PROVIDED the generatrix keeps its control[0]→control[1]
    // direction. So extend along that same sense (not blindly low→high), else
    // the rebuilt surface's normal flips and the face reads as inside-out.
    let point_at = |z: f64| -> Vec3 {
        let t = (z - z0) / denom;
        p0.add(p1.sub(p0).scale(t))
    };
    let (start_axial, end_axial) = if z1 >= z0 { (low, high) } else { (high, low) };
    let extended = make_line(point_at(start_axial), point_at(end_axial)).or_refuse(KernelStage::Sew, "make_line")?;
    let grown = make_revolution(frame.origin, frame.axis, &extended, sweep)
        .or_refuse(KernelStage::Sew, "make_revolution")?;
    face.surface = grown;
    Ok(())
}

/// Grow a face's carrier until it spans `points`, then rebuild EVERY pcurve on
/// the grown carrier from the edge curves.
///
/// The rejoin in `delete_faces.rs` hands one survivor face the loops of its
/// cosurface twin on the far side of a deleted band. Those loops' pcurves were
/// fitted to the TWIN's patch, and the patch that keeps them is a finite piece
/// of surface that stops where its own face stopped — so it has to grow over
/// the twin's extent and the bridges between them, and everything it now
/// carries has to be re-fitted to it. Both halves are the shared re-trim's
/// (`crate::offset_retrim`), taken face-locally because the rejoin is still
/// assembling a candidate face rather than editing one inside a solid.
///
/// The growth is EXACT for every carrier it accepts — a plane rebuilt over the
/// boundary's own box in its own frame, a ruled or partial-sweep revolution
/// prolonged along its axis — and a carrier it does not recognise is left
/// alone, so the refit below refuses if the loops fall outside it.
pub(super) fn regrow_and_refit_carrier(
    face: &mut FaceRecord,
    edges: &HashMap<u64, EdgeRecord>,
    scale: f64,
    op: &str,
) -> Result<(), KernelRefusal> {
    let tolerance = (scale * 1e-7).max(1e-9);
    let points = boundary_samples(face, edges, op)?;
    if let Ok(plane) = plane_of_surface(&face.surface, (scale * 1e-6).max(1e-7), op) {
        // The planar rebuild is `retrim_planar_face`'s, inlined for its
        // SUBRANGE-AWARE refit: a rejoined face keeps pinch-split edges that
        // are strict subranges of their own curve, and a whole-curve pcurve
        // for one of those lands the loop's ends a whole edge away.
        let mut u_min = f64::INFINITY;
        let mut u_max = f64::NEG_INFINITY;
        let mut v_min = f64::INFINITY;
        let mut v_max = f64::NEG_INFINITY;
        for point in &points {
            let delta = point.sub(plane.origin);
            u_min = u_min.min(delta.dot(plane.u_dir));
            u_max = u_max.max(delta.dot(plane.u_dir));
            v_min = v_min.min(delta.dot(plane.v_dir));
            v_max = v_max.max(delta.dot(plane.v_dir));
        }
        if !(u_min.is_finite() && v_min.is_finite()) {
            return Err(KernelRefusal::internal(
                KernelStage::Collect,
                "empty_boundary",
                format!("{op}: empty face boundary"),
            ));
        }
        let margin = ((u_max - u_min).max(v_max - v_min) * 0.25).max(scale * 1e-3);
        let origin = plane
            .origin
            .add(plane.u_dir.scale(u_min - margin))
            .add(plane.v_dir.scale(v_min - margin));
        face.surface = crate::make_plane(
            origin,
            plane.u_dir,
            plane.v_dir,
            (u_max - u_min) + 2.0 * margin,
            (v_max - v_min) + 2.0 * margin,
        )
        .or_refuse(KernelStage::Sew, "make_plane")?;
    } else {
        extend_ruled_carrier(face, &points, tolerance)?;
        extend_revolution_carrier(face, &points, tolerance)?;
    }
    let surface = face.surface.clone();
    crate::offset_retrim::rebuild_loop_pcurves(
        face,
        edges,
        &surface,
        tolerance,
        op,
    )
    
}

// ---------------------------------------------------------------------------
// §6.12 OPEN-chain heal with curved neighbours: curved PRIMARIES (plane ×
// cylinder/ruled revolution) and curved LATERAL CAPS (a strip ending against
// a curved analytic wall) alike.
// ---------------------------------------------------------------------------

/// Which carrier a neighbour of an open transition strip offers.
/// `Curved` is any recognized non-planar analytic (cylinder/cone wall,
/// partial revolution with a straight generatrix, …) — `intersect_analytic_pair`
/// decides below whether the actual PAIR has a closed-form re-intersection.
/// `FreeForm` is a fitted NURBS patch that is not any recognized analytic: it
/// grows by [`extend_freeform_neighbour_over`] and re-intersects through the
/// marched lane of `offset_reintersect::reintersect_carriers`.
pub(super) enum OpenNeighbourCarrier {
    Planar(Plane),
    Curved,
    FreeForm,
}

/// How much of the estimated parameter overshoot each extension round asks
/// for. A first-order estimate off the boundary tangent understates a curving
/// patch, so each round overshoots deliberately and the coverage check below
/// decides whether another round is needed.
const FREEFORM_EXTENSION_MARGIN: f64 = 1.5;

/// How many rounds of grow-and-recheck the free-form extension runs before it
/// gives up. Each round re-measures against the surface it just built, so the
/// estimate converges quadratically in practice; the cap is a guard against a
/// boundary whose tangent is degenerate and never converges at all.
const FREEFORM_EXTENSION_ROUNDS: usize = 5;

/// An overrun below this fraction of the boundary's own domain span is NOISE,
/// not a boundary the strip crosses. Measured 2026-09-12: a strip point sitting
/// on the carrier to within round-off still reports a residual, whose component
/// along the boundary tangent is a 1e-18-scale "overrun". Extending by that
/// appends a span narrower than the band within which two knots are the same
/// knot, and the appended span evaluates to a zero-weight point everywhere —
/// a singular carrier built out of round-off. `extend_natural` refuses such an
/// increment outright; this stops it ever being asked for.
const FREEFORM_OVERRUN_NOISE_FLOOR: f64 = 1e-6;

/// The smallest extension worth asking for, as a fraction of the boundary's own
/// domain span. The measured overrun is the MINIMUM that covers the strip; an
/// extension that only just covers it leaves the recovered branch running along
/// the grown domain's own boundary, in a sliver too thin for the marcher's seed
/// grid to land in. Asking for a floor of 2% of the span costs nothing (the
/// coverage check and `extend_natural`'s growth limit both still hold) and gives
/// the march somewhere to start.
///
/// It is also the UNIT every request is made in: a request is a whole number of
/// floors, the fewest that cover the margined overrun. The overrun is read off
/// the strip's stations, and the same strip read through another
/// parameterisation of its rails puts those stations — and so the overrun — a
/// few ulp elsewhere. A request of exactly `overrun × margin` carried those ulp
/// into the extended carrier, the carrier into the marched branch, and the
/// branch into every healed float; measured on the open heal's r = 1.0 rows,
/// whose overrun (0.0167 and 0.0173 of the span) is the first to ask past one
/// floor, re-parameterised rails healed to the same geometry but not the same
/// bits. A whole number of floors is the same number unless the overrun sits
/// within round-off of a floor boundary.
pub(super) const FREEFORM_MINIMUM_EXTENSION: f64 = 0.02;

/// Grow a FITTED (non-analytic) neighbour so its parameter square covers
/// `points`, by [`NurbsSurface::extend_natural`] on whichever of its four
/// sides the points overrun.
///
/// This is the free-form analogue of [`extend_ruled_neighbour_over`], and the
/// reason the open-chain heal can reach a fitted carrier at all. The heal used
/// to refuse a free-form neighbour outright, because the only thing available
/// past the trim was `evaluate_extended` — a tangent-plane continuation with no
/// bound and no surface, which a Newton solve can converge onto anywhere.
/// `extend_natural` gives a real surface with a real domain instead, so a solve
/// that leaves the extension leaves the surface.
///
/// COVERAGE IS VERIFIED, never assumed: after each round every point is
/// re-projected and must land strictly INSIDE the grown domain. A point whose
/// projection is still pinned to a boundary is a point the carrier does not
/// cover, and another round is asked for. A closed direction wraps and needs no
/// extension. Analytic and already-covering carriers are left untouched.
pub(super) fn extend_freeform_neighbour_over(
    solid: &mut BrepSolid,
    face_id: u64,
    points: &[Vec3],
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let op = "delete_face_and_heal";
    let (shell, face_pos) = find_face(solid, face_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "neighbour",
            format!("{op}: missing neighbour {face_id}"),
        )
    })?;
    let face = &mut solid.shells[shell].faces[face_pos];
    if face.surface.analytic().is_some() {
        return Ok(()); // an analytic carrier grows through its own exact lane
    }
    let debug = std::env::var("BREP_DEBUG_HEAL").is_ok();
    let mut surface = face.surface.clone();
    if debug {
        let weights: Vec<f64> = surface
            .control_points
            .iter()
            .flat_map(|row| row.iter().map(|point| point.w))
            .collect();
        let low = weights.iter().copied().fold(f64::INFINITY, f64::min);
        let high = weights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        eprintln!(
            "HEAL extend: face {face_id} degree {}x{} net {}x{} knots {}+{} weights [{low},{high}] \
             domain u{:?} v{:?} closed {:?}",
            surface.degree_u,
            surface.degree_v,
            surface.control_points.len(),
            surface.control_points[0].len(),
            surface.knots_u.len(),
            surface.knots_v.len(),
            surface.domain_u().or_refuse(KernelStage::Classify, "domain")?,
            surface.domain_v().or_refuse(KernelStage::Classify, "domain")?,
            surface.closed_directions().or_refuse(KernelStage::Classify, "domain")?
        );
    }
    for round in 0..FREEFORM_EXTENSION_ROUNDS {
        let overruns = boundary_overruns(&surface, points, tolerance)?;
        if debug {
            eprintln!("HEAL extend: face {face_id} round {round} overruns {overruns:?}");
        }
        if overruns.iter().all(|overrun| *overrun <= 0.0) {
            face.surface = surface;
            return Ok(());
        }
        for (index, side) in [
            SurfaceSide::UMin,
            SurfaceSide::UMax,
            SurfaceSide::VMin,
            SurfaceSide::VMax,
        ]
        .into_iter()
        .enumerate()
        {
            if overruns[index] <= 0.0 {
                continue;
            }
            let [low, high] = if side.is_u() {
                surface.domain_u().or_refuse(KernelStage::Classify, "domain")?
            } else {
                surface.domain_v().or_refuse(KernelStage::Classify, "domain")?
            };
            let floor = (high - low) * FREEFORM_MINIMUM_EXTENSION;
            let asked = (overruns[index] * FREEFORM_EXTENSION_MARGIN / floor).ceil().max(1.0) * floor;
            census_push("freeform_extension", || {
                serde_json::json!({
                    "face": face_id,
                    "round": round,
                    "side": format!("{side:?}"),
                    "overrun": overruns[index],
                    "floor": floor,
                    "asked": asked,
                })
            });
            // The extension's own verdict classes the refusal: a closed direction,
            // an unclamped end, a weight past its root, a fold or a runaway growth
            // is the carrier declining to be continued that way; an increment
            // this lane floors and a lower-level failure are its own defects.
            surface = surface.extend_natural(side, asked).map_err(|refusal| {
                let message = format!(
                    "{op}: cannot extend the fitted carrier of face {face_id} past its \
                     {side:?} boundary — {}",
                    refusal.describe()
                );
                match refusal {
                    ExtendRefusal::DegenerateIncrement { .. } | ExtendRefusal::Failed(_) => {
                        KernelRefusal::internal(KernelStage::Sew, "extend_natural", message)
                    }
                    _ => KernelRefusal::unsupported(KernelStage::Sew, "extend_natural", message),
                }
            })?;
        }
    }
    Err(KernelRefusal::non_convergence(
        KernelStage::Sew,
        "extension_rounds",
        format!(
            "{op}: the fitted carrier of face {face_id} still does not cover the deleted strip \
             after {FREEFORM_EXTENSION_ROUNDS} extension rounds — refusing rather than solving \
             on an unbounded extrapolation"
        ),
    ))
}

/// A planar patch centred on the deleted strip and big enough to cover its
/// region gate.
///
/// The marched re-intersection CLAMPS to each operand's stored domain
/// (`intersect/surface_surface_intersection.rs`), and an extruded or imported
/// planar face's patch ends exactly at the sharp edge the heal is trying to
/// recover — so the marcher is handed a branch lying along its own domain
/// boundary and finds nothing. A plane has no extent of its own; only its trim
/// does. Widening it is therefore exact, not an approximation, and it is what
/// makes the closed-form `Planar` neighbour usable in the marched lane the
/// free-form pairings need.
pub(super) fn planar_region_patch(
    plane: &Plane,
    center: Vec3,
    reach: f64,
) -> Result<NurbsSurface, KernelRefusal> {
    let on_plane = center.sub(
        plane
            .normal
            .scale(center.sub(plane.origin).dot(plane.normal)),
    );
    let corner = on_plane
        .sub(plane.u_dir.scale(reach))
        .sub(plane.v_dir.scale(reach));
    crate::make_plane(corner, plane.u_dir, plane.v_dir, reach * 2.0, reach * 2.0)
        .or_refuse(KernelStage::Sew, "make_plane")
}

/// How far past each of the four domain boundaries `points` reach, in that
/// boundary's own parameter. Zero means covered. A point that projects into the
/// interior overruns nothing; a point pinned to a boundary overruns it by the
/// component of its residual along the outgoing tangent, converted to parameter
/// through that tangent's own speed.
fn boundary_overruns(
    surface: &NurbsSurface,
    points: &[Vec3],
    tolerance: f64,
) -> Result<[f64; 4], KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Classify, "domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Classify, "domain")?;
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Classify, "domain")?;
    // A projection this close to a boundary is ON it: the Newton clamped, so
    // the true nearest point may be outside.
    let pinned_u = (u1 - u0) * 1e-9;
    let pinned_v = (v1 - v0) * 1e-9;
    let floor_u = (u1 - u0) * FREEFORM_OVERRUN_NOISE_FLOOR;
    let floor_v = (v1 - v0) * FREEFORM_OVERRUN_NOISE_FLOOR;
    let mut overruns = [0.0f64; 4];
    for &point in points {
        let projection =
            crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Classify, "project")?;
        if projection.distance <= tolerance {
            continue; // the point is ON the carrier; nothing overruns
        }
        let derivatives = surface
            .derivatives(projection.u, projection.v, 1)
            .or_refuse(KernelStage::Classify, "derivatives")?;
        let residual = point.sub(projection.point);
        let mut record = |index: usize, tangent: Vec3, sign: f64, floor: f64| {
            let speed = tangent.length();
            if speed <= 0.0 {
                return;
            }
            let along = residual.dot(tangent) / (speed * speed) * sign;
            if along > floor && along > overruns[index] {
                overruns[index] = along;
            }
        };
        if !closed_u {
            if projection.u <= u0 + pinned_u {
                record(0, derivatives[1][0], -1.0, floor_u);
            }
            if projection.u >= u1 - pinned_u {
                record(1, derivatives[1][0], 1.0, floor_u);
            }
        }
        if !closed_v {
            if projection.v <= v0 + pinned_v {
                record(2, derivatives[0][1], -1.0, floor_v);
            }
            if projection.v >= v1 - pinned_v {
                record(3, derivatives[0][1], 1.0, floor_v);
            }
        }
    }
    Ok(overruns)
}

/// The parameter/point where `curve` crosses `plane`, restricted to crossings
/// within `reach` of `center` (the deleted strip's region gate); the nearest
/// such crossing wins. Sampled sign changes refined by bisection — accurate to
/// ~1e-13 of the domain span, which is exact-in-practice for the analytic
/// branches this heal clips.
pub(super) fn curve_plane_crossing_near(
    curve: &NurbsCurve,
    plane: &Plane,
    center: Vec3,
    reach: f64,
) -> Option<(f64, Vec3)> {
    let [t0, t1] = curve.domain().ok()?;
    let height = |t: f64| -> Option<f64> {
        curve
            .evaluate(t)
            .ok()
            .map(|point| point.sub(plane.origin).dot(plane.normal))
    };
    const SAMPLES: usize = 64;
    let mut best: Option<(f64, Vec3, f64)> = None;
    let mut previous_t = t0;
    let mut previous_h = height(previous_t)?;
    for index in 1..=SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / SAMPLES as f64;
        let h = height(t)?;
        if previous_h * h <= 0.0 {
            let (mut low, mut high, mut low_h) = (previous_t, t, previous_h);
            for _ in 0..80 {
                let mid = 0.5 * (low + high);
                let mid_h = height(mid)?;
                if low_h * mid_h <= 0.0 {
                    high = mid;
                } else {
                    low = mid;
                    low_h = mid_h;
                }
            }
            let root = 0.5 * (low + high);
            let point = curve.evaluate(root).ok()?;
            let distance = point.sub(center).length();
            if distance <= reach && best.map(|(_, _, known)| distance < known).unwrap_or(true) {
                best = Some((root, point, distance));
            }
        }
        previous_t = t;
        previous_h = h;
    }
    best.map(|(t, point, _)| (t, point))
}

/// [`curve_plane_crossing_near`], plus the curve's own END when that end lies
/// ON the plane within `tolerance`.
///
/// A sign change is the right test for a branch that passes through a cap and
/// the wrong one for a branch that STOPS on it. A marched re-intersection is
/// clamped to its carriers' domains, and an extruded wall's domain ends exactly
/// in its cap planes, so the branch the open heal recovers routinely ends in
/// the cap it has to be clipped by. Its end height there is round-off: on the
/// D-prism of `open_heal_tests` it read +2.2e-16 on one body and -4.4e-16 on
/// the same body with a strip vertex moved 5e-13, and a strict sign change
/// refused the first and built the second. The end is accepted by the heal's
/// own model tolerance, the band every other incidence in the heal is held to;
/// the nearest candidate to `center` still wins.
pub(super) fn curve_plane_crossing_or_end_near(
    curve: &NurbsCurve,
    plane: &Plane,
    center: Vec3,
    reach: f64,
    tolerance: f64,
) -> Option<(f64, Vec3)> {
    let mut best = curve_plane_crossing_near(curve, plane, center, reach)
        .map(|(t, point)| (t, point, point.sub(center).length()));
    let [t0, t1] = curve.domain().ok()?;
    for t in [t0, t1] {
        let point = curve.evaluate(t).ok()?;
        if point.sub(plane.origin).dot(plane.normal).abs() > tolerance {
            continue;
        }
        let distance = point.sub(center).length();
        if distance <= reach && best.map(|(_, _, known)| distance < known).unwrap_or(true) {
            best = Some((t, point, distance));
        }
    }
    best.map(|(t, point, _)| (t, point))
}

/// Points along an edge's represented range at equal ARC LENGTH, both ends
/// included, ordered the way the coedge that reads them traverses the edge.
///
/// This is how the open heal samples a strip's boundary. A uniform step in the
/// edge's PARAMETER puts the samples wherever the curve's speed law puts them,
/// and the same rail can carry any number of speed laws — reversed, re-knotted,
/// fitted in pieces, re-weighted — so a heal fed parameter samples reads a
/// property of the representation. Equal arc length is a property of the curve,
/// and so is the traversal order: reversing an edge flips its coedge's
/// `forward` flag with it. Each station is solved by a bracketed Newton on the
/// Gauss arc length within one knot span, iterated until the step no longer
/// moves the parameter.
pub(super) fn arc_length_stations(
    edge: &EdgeRecord,
    count: usize,
    forward: bool,
) -> Result<Vec<Vec3>, KernelRefusal> {
    let count = count.max(2);
    let curve = &edge.curve;
    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    let mut breaks: Vec<f64> = vec![low, high];
    breaks.extend(curve.knots.iter().copied().filter(|knot| *knot > low && *knot < high));
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    let mut cumulative = vec![0.0f64];
    for pair in breaks.windows(2) {
        let last = *cumulative.last().unwrap();
        cumulative.push(
            last + crate::curve_arc_length(curve, pair[0], pair[1]).or_refuse(KernelStage::Classify, "arc_length")?,
        );
    }
    let total = *cumulative.last().unwrap();
    let mut stations = Vec::with_capacity(count);
    for index in 0..count {
        let t = if index == 0 {
            low
        } else if index == count - 1 {
            high
        } else if !(total > 0.0) {
            low + (high - low) * index as f64 / (count - 1) as f64
        } else {
            let target = total * index as f64 / (count - 1) as f64;
            let panel = (0..breaks.len() - 1)
                .find(|panel| cumulative[panel + 1] >= target)
                .unwrap_or(breaks.len() - 2);
            let (mut bracket_low, mut bracket_high) = (breaks[panel], breaks[panel + 1]);
            let along = target - cumulative[panel];
            let panel_length = cumulative[panel + 1] - cumulative[panel];
            let mut t = if panel_length > 0.0 {
                bracket_low + (bracket_high - bracket_low) * (along / panel_length).clamp(0.0, 1.0)
            } else {
                bracket_low
            };
            for _ in 0..64 {
                let excess =
                    crate::curve_arc_length(curve, breaks[panel], t).or_refuse(KernelStage::Classify, "arc_length")? - along;
                if excess > 0.0 {
                    bracket_high = t;
                } else {
                    bracket_low = t;
                }
                let speed = curve.deriv1(t).or_refuse(KernelStage::Classify, "derivatives")?.1.length();
                let mut next = if speed > 0.0 { t - excess / speed } else { f64::NAN };
                if !(next > bracket_low && next < bracket_high) {
                    next = 0.5 * (bracket_low + bracket_high);
                }
                if next == t {
                    break;
                }
                t = next;
            }
            t
        };
        stations.push(curve.evaluate(t).or_refuse(KernelStage::Classify, "evaluate")?);
    }
    if !forward {
        stations.reverse();
    }
    Ok(stations)
}

/// The parameter/point where `curve` crosses a lateral cap's ANALYTIC carrier,
/// restricted to crossings within `reach` of `center` (the deleted strip's
/// region gate); the nearest such crossing wins. A PLANAR cap uses the exact
/// bisection solver above, which also accepts a branch that ENDS on the plane;
/// a CURVED analytic cap uses the Newton curve×surface intersector against the
/// cap face's carrier surface, whose Newton clamps to the curve's domain and
/// accepts a hit within `tolerance`, so an end lying on the cap is a hit it can
/// return.
pub(super) fn curve_cap_crossing_near(
    curve: &NurbsCurve,
    cap_carrier: &OpenNeighbourCarrier,
    cap_surface: &NurbsSurface,
    center: Vec3,
    reach: f64,
    tolerance: f64,
) -> Option<(f64, Vec3)> {
    match cap_carrier {
        OpenNeighbourCarrier::Planar(plane) => {
            curve_plane_crossing_or_end_near(curve, plane, center, reach, tolerance)
        }
        OpenNeighbourCarrier::Curved | OpenNeighbourCarrier::FreeForm => {
            let hits = intersect_curve_surface(curve, cap_surface, tolerance).ok()?;
            let mut best: Option<(f64, Vec3, f64)> = None;
            for hit in hits {
                let point = curve.evaluate(hit.t).ok()?;
                let distance = point.sub(center).length();
                if distance <= reach && best.map(|(_, _, known)| distance < known).unwrap_or(true) {
                    best = Some((hit.t, point, distance));
                }
            }
            best.map(|(t, point, _)| (t, point))
        }
    }
}

/// Move a side edge's endpoint(s) onto recovered corner(s). Prefers WIDENING:
/// the corner is projected onto the edge's underlying curve and, when it lands
/// on it (a blend trimmed this very curve back, so growing the represented
/// interval restores it exactly — line, arc, or spline alike), only t0/t1 and
/// the vertex ids change and the geometry is bit-exact. A straight line whose
/// curve stops short of the corner is rebuilt from its resolved endpoints
/// instead. Returns Ok(false) — leaving the edge untouched — when the stored
/// curve stops short and is not a line (booleans re-fit curves to exactly the
/// represented range); the caller may then re-derive the edge in closed form
/// from its flanking carriers. Hard refusals (degenerate edge, zero-length
/// collapse of a line) stay errors.
pub(super) fn relocate_open_side_edge(
    edge: &mut EdgeRecord,
    start_target: Option<(u64, Vec3)>,
    end_target: Option<(u64, Vec3)>,
    on_curve_tolerance: f64,
    tolerance: f64,
    op: &str,
) -> Result<bool, KernelRefusal> {
    if edge.degenerate {
        return Err(KernelRefusal::unsupported(
            KernelStage::Sew,
            "degenerate_side_edge",
            format!(
                "{op}: degenerate side edge {} cannot be relocated (deferred)",
                edge.id
            ),
        ));
    }
    let [d0, d1] = edge.curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let span = (d1 - d0).max(1e-12);
    let snap = |t: f64| -> f64 {
        if (t - d0).abs() <= 1e-9 * span {
            d0
        } else if (t - d1).abs() <= 1e-9 * span {
            d1
        } else {
            t
        }
    };
    let widen = |target: Vec3| -> Option<f64> {
        let projection = project_point_to_curve(&edge.curve, target).ok()?;
        (projection.distance <= on_curve_tolerance).then(|| snap(projection.u))
    };
    let new_t0 = match &start_target {
        Some((_, point)) => widen(*point),
        None => Some(edge.t0),
    };
    let new_t1 = match &end_target {
        Some((_, point)) => widen(*point),
        None => Some(edge.t1),
    };
    if let (Some(new_t0), Some(new_t1)) = (new_t0, new_t1) {
        if new_t1 - new_t0 > 1e-9 * span {
            edge.t0 = new_t0;
            edge.t1 = new_t1;
            if let Some((vertex, _)) = start_target {
                edge.start_vertex_id = vertex;
            }
            if let Some((vertex, _)) = end_target {
                edge.end_vertex_id = vertex;
            }
            return Ok(true);
        }
    }
    // The curve itself does not reach the corner: rebuild straight lines,
    // hand everything else back to the caller for closed-form re-derivation.
    if edge.curve.straight_segment(tolerance).is_some() {
        let start_point = match &start_target {
            Some((_, point)) => *point,
            None => edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?,
        };
        let end_point = match &end_target {
            Some((_, point)) => *point,
            None => edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?,
        };
        if start_point.sub(end_point).length() <= tolerance {
            return Err(KernelRefusal::unsupported(
                KernelStage::Sew,
                "side_edge_collapse",
                format!(
                    "{op}: healing would collapse side edge {} to zero length",
                    edge.id
                ),
            ));
        }
        edge.curve = make_line(start_point, end_point).or_refuse(KernelStage::Sew, "make_line")?;
        edge.t0 = 0.0;
        edge.t1 = 1.0;
        if let Some((vertex, _)) = start_target {
            edge.start_vertex_id = vertex;
        }
        if let Some((vertex, _)) = end_target {
            edge.end_vertex_id = vertex;
        }
        return Ok(true);
    }
    Ok(false)
}

/// Closed-form fallback for a side edge whose STORED curve stops short of the
/// recovered corner (booleans re-fit intersection curves to exactly the
/// represented range, so widening has nothing to widen into). The side edge
/// lies on the intersection of its two flanking faces' analytic carriers:
/// re-derive that intersection with `intersect_analytic_pair`, pick the branch
/// that carries BOTH resolved endpoints AND the existing edge (its midpoint
/// must project onto the branch inside the new interval — this also rejects a
/// periodic branch whose direct parameter interval runs the wrong way around),
/// and re-trim it from the preserved endpoint to the corner. Refused honestly
/// when the flanking pair has no closed form or no branch qualifies.
pub(super) fn rederive_side_edge_on_carriers(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    start_target: Option<(u64, Vec3)>,
    end_target: Option<(u64, Vec3)>,
    on_curve_tolerance: f64,
    tolerance: f64,
    op: &str,
) -> Result<EdgeRecord, KernelRefusal> {
    let mut adjacent: Vec<&NurbsSurface> = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            if face
                .loops
                .iter()
                .any(|loop_record| loop_record.coedges.iter().any(|c| c.edge_id == edge.id))
            {
                adjacent.push(&face.surface);
            }
        }
    }
    if adjacent.len() != 2 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Collect,
            "non_manifold_side_edge",
            format!(
                "{op}: side edge {} is not shared by exactly two faces (found {})",
                edge.id,
                adjacent.len()
            ),
        ));
    }
    let start_point = match &start_target {
        Some((_, point)) => *point,
        None => edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?,
    };
    let end_point = match &end_target {
        Some((_, point)) => *point,
        None => edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?,
    };
    if start_point.sub(end_point).length() <= tolerance {
        return Err(KernelRefusal::unsupported(
            KernelStage::Sew,
            "rederived_edge_collapse",
            format!(
                "{op}: healing would collapse side edge {} to zero length",
                edge.id
            ),
        ));
    }
    let middle_point = edge
        .curve
        .evaluate(0.5 * (edge.t0 + edge.t1))
        .or_refuse(KernelStage::Sew, "evaluate")?;
    let branches =
        intersect_analytic_pair(adjacent[0], adjacent[1], tolerance).ok_or_else(|| {
            KernelRefusal::unsupported(
                KernelStage::Classify,
                "no_closed_form_pair",
                format!(
                    "{op}: side edge {} cannot be extended onto the recovered corner (its curve \
             stops short and its flanking carriers are not a closed-form analytic pair — \
             deferred)",
                    edge.id
                ),
            )
        })?;
    for branch in branches {
        let Ok(projected_start) = project_point_to_curve(&branch, start_point) else {
            continue;
        };
        let Ok(projected_end) = project_point_to_curve(&branch, end_point) else {
            continue;
        };
        let Ok(projected_middle) = project_point_to_curve(&branch, middle_point) else {
            continue;
        };
        if projected_start.distance > on_curve_tolerance
            || projected_end.distance > on_curve_tolerance
            || projected_middle.distance > on_curve_tolerance
        {
            continue;
        }
        let [d0, d1] = branch.domain().or_refuse(KernelStage::Sew, "domain")?;
        let branch_span = (d1 - d0).max(1e-12);
        let (t_start, t_end) = (projected_start.u, projected_end.u);
        if (t_end - t_start).abs() <= 1e-9 * branch_span {
            continue;
        }
        let (low, high) = if t_start < t_end {
            (t_start, t_end)
        } else {
            (t_end, t_start)
        };
        let slack = 1e-6 * branch_span;
        if projected_middle.u < low - slack || projected_middle.u > high + slack {
            continue;
        }
        // Preserve the edge's start→end direction (coedge forward flags and
        // vertex bindings reference it).
        let (curve, new_t0, new_t1) = if t_start < t_end {
            (branch, t_start, t_end)
        } else {
            let reversed = branch.reversed().or_refuse(KernelStage::Sew, "reverse")?;
            (reversed, d0 + d1 - t_start, d0 + d1 - t_end)
        };
        let mut record = edge.clone();
        record.curve = curve;
        record.t0 = new_t0;
        record.t1 = new_t1;
        if let Some((vertex, _)) = start_target {
            record.start_vertex_id = vertex;
        }
        if let Some((vertex, _)) = end_target {
            record.end_vertex_id = vertex;
        }
        return Ok(record);
    }
    Err(KernelRefusal::unsupported(
        KernelStage::Classify,
        "no_branch_reaches",
        format!(
            "{op}: side edge {} cannot be extended onto the recovered corner (no closed-form \
             intersection branch of its flanking carriers reaches both endpoints — deferred)",
            edge.id
        ),
    ))
}

/// Refit the pcurves of every coedge referencing a touched (widened or newly
/// created) edge against the face's CURRENT carrier, sampling exactly the
/// edge's represented interval — so extended pcurve endpoints land on the NEW
/// edge, not the old blend rim (the closed-heal endpoint-station lesson).
/// With `only_subrange`, coedges whose edge represents its curve's full domain
/// are left alone (the affine full-curve pcurve a planar retrim just built is
/// already exact there).
pub(super) fn refit_touched_pcurves(
    face: &mut FaceRecord,
    edges: &HashMap<u64, EdgeRecord>,
    touched: &HashSet<u64>,
    only_subrange: bool,
    tolerance: f64,
    op: &str,
) -> Result<(), KernelRefusal> {
    let surface = face.surface.clone();
    for loop_record in &mut face.loops {
        for coedge in &mut loop_record.coedges {
            if !touched.contains(&coedge.edge_id) {
                continue;
            }
            let edge = edges.get(&coedge.edge_id).ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "edge",
                    format!("{op}: missing edge {}", coedge.edge_id),
                )
            })?;
            if only_subrange {
                let [d0, d1] = edge.curve.domain().or_refuse(KernelStage::Sew, "domain")?;
                let span = (d1 - d0).max(1e-12);
                if (edge.t0 - d0).abs() <= 1e-9 * span && (edge.t1 - d1).abs() <= 1e-9 * span {
                    continue;
                }
            }
            coedge.pcurve = build_pcurve_on_surface_range(
                &surface,
                &edge.curve,
                edge.t0,
                edge.t1,
                coedge.forward,
                tolerance,
            )
            .or_refuse(KernelStage::Sew, "pcurve_fit")?;
        }
    }
    Ok(())
}
