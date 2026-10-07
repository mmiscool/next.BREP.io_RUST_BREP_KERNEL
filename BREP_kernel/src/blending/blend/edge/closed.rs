use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// General closed-edge rolling-ball fillet or chamfer by direct §6.9
/// topology surgery.
pub fn blend_closed_edge(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    if !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelRefusal::input(KernelStage::Collect, "radius", "blend: radius must be positive"));
    }
    blend_closed_edge_impl(solid, edge_id, &|_| radius, Some(radius), chamfer, name)
}

/// Variable-radius blend (4.9.5): radius stops as (edge fraction, radius)
/// pairs, linearly interpolated along the edge parameter and clamped at
/// the ends.  Closed edges must supply matching first/last radii.
pub fn blend_edge_variable(
    solid: &BrepSolid,
    edge_id: u64,
    radii: &[(f64, f64)],
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    if radii.is_empty() || radii.iter().any(|(_, radius)| !(*radius > 0.0)) {
        return Err(KernelRefusal::input(KernelStage::Collect, "radius_stop", "blend: every radius stop must be positive"));
    }
    let mut stops = radii.to_vec();
    stops.sort_by(|a, b| a.0.total_cmp(&b.0));
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| format!("blend: edge {edge_id} not found")).or_refuse(KernelStage::Refine, "ok_or_else")?;
    let closed = edge.start_vertex_id == edge.end_vertex_id;
    if closed && (stops[0].1 - stops[stops.len() - 1].1).abs() > 1e-12 {
        return Err(KernelRefusal::input(KernelStage::Collect, "closed_edge_radii", "blend: closed edges need equal first/last radii"));
    }
    // CONSTANT stops must DEGENERATE to the exact constant-radius
    // machinery (§6.9 exact cylinder cutters where the mates allow them,
    // the same general march otherwise).  The general march's fitted
    // NURBS is geometrically exact for a constant radius, but downstream
    // booleans treat it as a generic surface: marched SSI curves against
    // a tangent mate drift O(√ε) along the tangency (measured 1.2e-3 on
    // a 10-box chain miter), while the exact representation intersects
    // analytically — this is what keeps chain-corner miters composable.
    let first_radius = stops[0].1;
    if stops
        .iter()
        .all(|(_, radius)| (radius - first_radius).abs() <= 1e-12 * (1.0 + first_radius.abs()))
    {
        return if chamfer {
            crate::fillet::chamfer_edge(solid, edge_id, first_radius, name)
        } else {
            crate::fillet::fillet_edge(solid, edge_id, first_radius, name)
        };
    }
    let (t0, t1) = (edge.t0, edge.t1);
    let radius_at = move |t: f64| -> f64 {
        let raw = (t - t0) / (t1 - t0);
        // Closed edges WRAP (the anchored march may start mid-edge and run
        // past t1 once around — clamping misplaced mid-profile stops
        // there); open edges CLAMP flat past the ends, which is only the
        // marched overshoot region beyond the trims.  The blend-end
        // stations are pinned exactly at t0/t1 by the open march, so the
        // profile kink the clamp creates sits ON an interpolation node and
        // the fitted rows still pass through the closed-form end stations.
        let fraction = if closed {
            raw.rem_euclid(1.0)
        } else {
            raw.clamp(0.0, 1.0)
        };
        if fraction <= stops[0].0 {
            return stops[0].1;
        }
        for pair in stops.windows(2) {
            if fraction <= pair[1].0 {
                let width = (pair[1].0 - pair[0].0).max(1e-12);
                let local = (fraction - pair[0].0) / width;
                return pair[0].1 + (pair[1].1 - pair[0].1) * local;
            }
        }
        stops[stops.len() - 1].1
    };
    // A VARIABLE profile (constant stops left above for the constant entry):
    // no constant-radius provenance, so no rolling-ball wall verdict.
    if closed {
        blend_closed_edge_impl(solid, edge_id, &radius_at, None, chamfer, name)
    } else {
        blend_open_edge_impl(solid, edge_id, &radius_at, None, chamfer, name)
    }
}

/// `constant`: the radius when the blend came in through a CONSTANT-radius
/// entry point (`blend_closed_edge`), `None` for a variable profile — the
/// only provenance on which the wall is judged against one rolling ball.
fn blend_closed_edge_impl(
    solid: &BrepSolid,
    edge_id: u64,
    radius_at: &dyn Fn(f64) -> f64,
    constant: Option<f64>,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    if let Some(separated) = separate_spherical_domain_cut(solid, edge_id)? {
        return blend_closed_edge_impl(&separated, edge_id, radius_at, constant, chamfer, name);
    }
    if let Some(simplified) = remove_retraced_cuts(solid, edge_id)? {
        return blend_closed_edge_impl(&simplified, edge_id, radius_at, constant, chamfer, name);
    }
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| format!("blend: edge {edge_id} not found")).or_refuse(KernelStage::Refine, "ok_or_else")?;
    if edge.start_vertex_id != edge.end_vertex_id {
        return Err(KernelRefusal::input(KernelStage::Collect, "closed_edge", "blend: general path currently requires a CLOSED edge"));
    }
    let (face_a, loop_a, coedge_a) = locate_mate(solid, edge_id, None)?;
    let (face_b, loop_b, coedge_b) = locate_mate(solid, edge_id, Some((face_a.id, loop_a)))?;
    if face_a.id == face_b.id && loop_a == loop_b {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "seam_edge", "blend: edge is used twice by one loop (seam edge?)"));
    }

    // Seam-structured loops force the blend seam onto that carrier's seam
    // meridian; at most one mate may need it.
    let seam_a = face_a.loops[loop_a].coedges.len() > 1;
    let seam_b = face_b.loops[loop_b].coedges.len() > 1;
    let (first_face, first_loop, first_coedge, second_face, second_loop, second_coedge) =
        if seam_a || !seam_b {
            (face_a, loop_a, coedge_a, face_b, loop_b, coedge_b)
        } else {
            (face_b, loop_b, coedge_b, face_a, loop_a, coedge_a)
        };
    let anchored = first_face.loops[first_loop].coedges.len() > 1;
    let second_seam = anchored && second_face.loops[second_loop].coedges.len() > 1;
    let anchor_u = if anchored {
        // Lock onto the carrier's seam meridian: the u-domain start.
        Some(first_face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?[0])
    } else {
        None
    };

    // Signed radii from the cross-section seed at the edge midpoint.
    let mid_radius = radius_at(edge.t0 + (edge.t1 - edge.t0) * 0.5);
    let (rho1, rho2) = signed_radii(
        edge,
        first_face,
        first_coedge,
        second_face,
        second_coedge,
        mid_radius,
    )?;

    let first_mate = BlendMate {
        face: first_face,
        coedge: first_coedge,
        loop_index: first_loop,
        rho: rho1,
    };
    let second_mate = BlendMate {
        face: second_face,
        coedge: second_coedge,
        loop_index: second_loop,
        rho: rho2,
    };
    let mid_t = edge.t0 + (edge.t1 - edge.t0) * 0.5;
    // Anchored FIRST, always — everything that already works takes this rung
    // and is unchanged.  An anchored march freezes the first carrier's u on
    // its seam meridian and frees t (`solve_anchored_start`), so it is only
    // solvable when the edge actually reaches that meridian.  A closed edge
    // cut by a plane through a surface of revolution's AXIS is one whole
    // MERIDIAN of that carrier — constant u, a full turn in v — so no station
    // on it can sit at the seam, and the seam may even have been trimmed off
    // the face (the 2026-09-01 reported document: the kept torus was
    // u ∈ [0.1369, 0.8869] while the anchor locked u = 0).  That march reports
    // "anchored seam start did not converge"; the UNANCHORED march solves the
    // same edge exactly, because the edge's own endpoints already sit on the
    // carrier's OTHER (v) seam.  Retrying is what makes this a rung of the
    // existing ladder rather than a predicate with a tolerance to tune: a
    // march that needs its anchor still gets one.
    use crate::blend::chain::FoldPolicy;
    let march_with = |fold: FoldPolicy, depth_budget: usize| -> Result<Vec<Station>, KernelRefusal> {
        match march_stations(edge, &first_mate, &second_mate, radius_at, anchor_u, fold, depth_budget) {
            Ok(stations) => Ok(stations),
            Err(anchored_error) if anchor_u.is_some() => {
                march_stations(edge, &first_mate, &second_mate, radius_at, None, fold, depth_budget).map_err(
                    |retry_error| {
                        if crate::blend::is_wall_fold(&retry_error)
                        || crate::blend::is_marched_fit_off_carriers(&retry_error) {
                            retry_error
                        } else {
                            anchored_error
                        }
                    },
                )
            }
            Err(anchored_error) => Err(anchored_error),
        }
    };
    let rail_bar = 0.5 * crate::KernelTolerances::for_solid(solid, 1e-7).intersection_fit;
    // A FILLET of one constant radius is also judged on its wall between the
    // rails, against that ball's sweep, at the unchanged `intersection_fit`
    // (`ClosedWallCheck`); a chamfer or a varying radius is not.
    let wall_check = {
        // Constant-radius PROVENANCE (the entry point), never sampled
        // constancy: a variable profile can agree at any finite set of samples.
        let radius = constant.unwrap_or(f64::NAN);
        let constant = constant.is_some();
        // The construction request as on the closed chain's plain fillet
        // rungs: `KernelTolerances::model` on the wall between the rails.
        let model = crate::KernelTolerances::for_solid(solid, 1e-7).model;
        let bar = crate::KernelTolerances::for_solid(solid, 1e-7).intersection_fit;
        (!chamfer && constant && radius.is_finite() && radius != 0.0).then(|| crate::blend::stations::ClosedWallCheck {
            radius: radius.abs(),
            bar,
            model: model.is_finite().then_some(model),
        })
    };
    crate::blend::stations::CLOSED_MODEL_UNMET.with(|slot| slot.set(None));
    crate::blend::stations::CLOSED_REQUEST_RAILS.with(|slot| slot.set(None));
    let marched = match march_stations_with_rail_bar(edge, &first_mate, &second_mate, radius_at, anchor_u, FoldPolicy::Refuse, MAX_REFINEMENT_DEPTH, rail_bar, wall_check) {
        Ok(stations) => Ok(stations),
        Err(failure) => {
            // Post-refinement errors return directly, before either the
            // unanchored retry or the legacy edge-preserving construction.
            let anchored_error = failure.into_initial()?;
            if anchor_u.is_some() {
                if std::env::var("BREP_DEBUG_CLOSED_RAILS").is_ok() {
                    eprintln!("closed initial anchor fallback: edge {}: {anchored_error}", edge.id);
                }
                match march_stations_with_rail_bar(edge, &first_mate, &second_mate, radius_at, None, FoldPolicy::Refuse, MAX_REFINEMENT_DEPTH, rail_bar, wall_check) {
                    Ok(stations) => Ok(stations),
                    Err(failure) => {
                        let retry_error = failure.into_initial()?;
                        if crate::blend::is_wall_fold(&retry_error) { Err(retry_error) }
                        else { Err(anchored_error) }
                    }
                }
            } else { Err(anchored_error) }
        }
    };
    // A wall that FOLDS through itself: the lens of the envelope inside the
    // swept ball volume is CARVED (`blend/carve.rs`), the way the closed chain
    // lane carves it, and the crease sewn as an inner loop of the blend face.
    // A fold that reaches a RAIL is not a lens (nothing to keep on one side)
    // and stays the named refusal; so does a chamfer's (a straight section
    // has no ball centre to read the fold locus against).  The march is
    // re-run under `FoldPolicy::Carve` so the folding wall is built; the lane
    // then checks the fitted rows are accurate enough to carve and that the
    // lens does not straddle the wall's own seam before anything is cut.
    // The march's unmet construction request (if any), taken before any other
    // march runs; it is reported only for the plain wall this lane builds
    // from that march's stations.
    let model_unmet = crate::blend::stations::CLOSED_MODEL_UNMET.with(|slot| slot.take());
    let request_rails = crate::blend::stations::CLOSED_REQUEST_RAILS.with(|slot| slot.take());
    let mut carve_fold = false;
    let marched = match marched {
        Err(error)
            if crate::blend::is_wall_fold(&error)
                && !crate::blend::fold::is_rail_fold(&error)
                && !chamfer =>
        {
            crate::blend::carve::carve_trace(format_args!(
                "blend closed-edge: the wall folds ({error}); re-marching under Carve"
            ));
            // The CARVE LADDER: the adaptive march's depth budget is raised
            // rung by rung until the rows fitted through its stations stand
            // on their carriers between stations to the network's rail bar
            // (half `intersection_fit`, read at 0.25/0.5/0.75 of every span)
            // — the same climb the closed chain lane makes per segment
            // (`carve_ladder`).  Measured on the narrowed mouth-over-wall
            // variant 2026-10-03: the refusing march's own budget left the
            // rails 2.7e-4 off their carriers, two decades over the bar.
            let rail_bar = 0.5 * crate::KernelTolerances::for_solid(solid, 1e-7).intersection_fit;
            let mut climbed: Option<Vec<Station>> = None;
            let mut last_off = f64::NAN;
            for depth_budget in MAX_REFINEMENT_DEPTH..=CARVE_MAX_REFINEMENT_DEPTH {
                let stations = match march_with(FoldPolicy::Carve, depth_budget) {
                    Ok(stations) => stations,
                    // The Carve march did not build: the fold stays the answer.
                    Err(_) => break,
                };
                let parameters = station_parameters(&stations);
                let off = match fit_closed_rows(&stations, &parameters, chamfer) {
                    Ok(rows) => rails_off_closed_carriers(&rows, &stations, &parameters, &first_mate, &second_mate)?,
                    Err(_) => break,
                };
                last_off = off;
                crate::blend::carve::carve_trace(format_args!(
                    "blend closed-edge carve ladder: depth budget {depth_budget} ({} stations) leaves the rails {off:.3e} off their carriers (bar {rail_bar:.3e})",
                    stations.len()
                ));
                if off <= rail_bar {
                    climbed = Some(stations);
                    break;
                }
            }
            match climbed {
                Some(stations) => {
                    carve_fold = true;
                    Ok(stations)
                }
                None => Err(crate::blend::fold::wall_fold_refusal(
                    Vec::new(),
                    format!(
                        "{} {} fits this edge: the wall folds, and at depth budget {CARVE_MAX_REFINEMENT_DEPTH} — the top of the closed march's carve ladder — its fitted rails still stand {last_off:.3e} off their carriers between stations against a bar of {rail_bar:.3e}, so there is no wall accurate enough to carve ({error})",
                        crate::blend::WALL_FOLDS,
                        radius_at(mid_t).abs()
                    ),
                )),
            }
        }
        other => other,
    };
    let stations = match marched {
        Ok(stations) => stations,
        // A wall that FOLDS is refused here for the same reason an escaped
        // march is: the edge-preserving construction would build a blend on the
        // same folding centre curve, and if it converged it would return the
        // very surface this refusal exists to stop.  It is not the fallback's
        // question to answer, so the geometry is reported as-is.
        Err(march_error) if crate::blend::is_wall_fold(&march_error)
            || crate::blend::is_marched_fit_off_carriers(&march_error) => return Err(march_error),
        Err(march_error) if crate::blend::is_ball_off_carrier(&march_error) => {
            // An ESCAPED march is not a non-convergence: the tangency system
            // had no solution at all, so no fallback can build the blend the
            // rolling ball never made.  Report the escape itself — routing it
            // to the edge-preserving construction below only reports THAT
            // lane's complaint about a face it was never meant to consume
            // ("ambiguous preserved boundary edge" on the 2026-09-10 report).
            return Err(march_error);
        }
        Err(march_error) => {
            // A non-converging march often means the rolling ball falls off
            // one face — try the edge-preserving construction (4.9.7).
            return blend_closed_edge_keep(
                solid,
                edge,
                &first_mate,
                &second_mate,
                radius_at(mid_t),
                rho1,
                chamfer,
                name,
            )
            .or_else(|_| {
                blend_closed_edge_keep(
                    solid,
                    edge,
                    &second_mate,
                    &first_mate,
                    radius_at(mid_t),
                    rho2,
                    chamfer,
                    name,
                )
            })
            .map_err(|keep_error| {
                format!("{march_error}; edge-preserving blend also failed: {keep_error}")
            }).or_refuse(KernelStage::Refine, "map_err");
        }
    };
    // Support-out-of-trim detection (4.9.7 trigger): a tangency track that
    // leaves its face's trimmed region cannot be trimmed there — the far
    // boundary must be PRESERVED instead.
    let support_exits = |face: &FaceRecord, second_side: bool| -> bool {
        // Adaptive marches vary the station count; keep at least the old fixed
        // grid's ~16-probe floor while inheriting extra density where the
        // march refined (which is where an exit would hide).
        let stride = (stations.len() / 16).max(1);
        stations.iter().step_by(stride).any(|station| {
            let mut uv = if second_side {
                station.uv2
            } else {
                station.uv1
            };
            if let Ok((closed_u, closed_v)) = face.surface.closed_directions() {
                if closed_u {
                    if let Ok([low, high]) = face.surface.domain_u() {
                        uv[0] = low + (uv[0] - low).rem_euclid(high - low);
                    }
                }
                if closed_v {
                    if let Ok([low, high]) = face.surface.domain_v() {
                        uv[1] = low + (uv[1] - low).rem_euclid(high - low);
                    }
                }
            }
            crate::parameter_point_in_face(face, crate::Vec2 { x: uv[0], y: uv[1] }, 1e-6)
                .map(|class| class == crate::PolygonClass::Outside)
                .unwrap_or(true)
        })
    };
    if support_exits(second_face, true) {
        return blend_closed_edge_keep(
            solid,
            edge,
            &first_mate,
            &second_mate,
            radius_at(mid_t),
            rho1,
            chamfer,
            name,
        );
    }
    if support_exits(first_face, false) {
        return blend_closed_edge_keep(
            solid,
            edge,
            &second_mate,
            &first_mate,
            radius_at(mid_t),
            rho2,
            chamfer,
            name,
        );
    }
    if std::env::var("BREP_DEBUG_BLEND_MARCH").is_ok() {
        let last = stations.len() - 1;
        for index in [
            0usize,
            1.min(last),
            2.min(last),
            last.saturating_sub(1),
            last,
        ] {
            let station = &stations[index];
            eprintln!(
                "station {index}: uv1=({:.6},{:.6}) uv2=({:.6},{:.6}) p1=({:.6},{:.6},{:.6}) w={:.6}",
                station.uv1[0], station.uv1[1], station.uv2[0], station.uv2[1],
                station.p1.x, station.p1.y, station.p1.z, station.weight,
            );
        }
    }
    let parameters = station_parameters(&stations);
    let exact_rows = exact_closed_revolution_rows(&stations, &first_mate, &second_mate, chamfer);
    let rows = match exact_rows {
        Some(rows) => rows?,
        None => {
            // The support pcurves interpolate the stations' (u, v) and are
            // read nowhere between them: on the 20-degree crossing's single
            // closed exit edge they stood 1.01e-4 (thin carrier) and
            // 2.6e-5 to 3.8e-5 (fat carrier) off the rails they trim, while the
            // rails stood on their carriers to 1e-6 — the whole of a 5e-6 to
            // 1.1e-5 shell vector-area residual and of a seam-placement-
            // dependent volume (dumps of c5ea8fb80 and 635ef7630, read
            // input-free). Each is refitted to its rail's own projected track
            // at the pcurve floor, by the closed chain's support-piece fitter.
            let mut rows = fit_closed_rows(&stations, &parameters, chamfer)?;
            rows.cr_pcurve = refit_closed_support_pcurve(&rows.cr, &rows.cr_pcurve, &first_mate.face.surface)?;
            rows.cs_pcurve = refit_closed_support_pcurve(&rows.cs, &rows.cs_pcurve, &second_mate.face.surface)?;
            rows
        }
    };
    let crease = if carve_fold {
        Some(carve_closed_wall(solid, &rows, &stations, &parameters, &first_mate, &second_mate, radius_at(mid_t).abs())?)
    } else {
        None
    };

    // Both mates seam-structured: the second support crosses ITS carrier's
    // seam meridian somewhere mid-loop; split cs there so face2's loop can
    // keep its seam-in-one-loop structure.
    let second_split = if second_seam {
        let seam2 = second_face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?[0];
        let period2 = {
            let [d0, d1] = second_face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
            d1 - d0
        };
        // Bracket: unwrapped uv2 track crossing seam2 + k·period.
        let u2_first = stations[0].uv2[0];
        let u2_last = stations[stations.len() - 1].uv2[0];
        let direction = (u2_last - u2_first).signum();
        let mut k = ((u2_first - seam2) / period2).ceil();
        if direction < 0.0 {
            k = ((u2_first - seam2) / period2).floor();
        }
        let target = seam2 + k * period2;
        let inside =
            (target - u2_first) * direction > 1e-6 && (u2_last - target) * direction > 1e-6;
        // ALIGNED seams (both carriers' meridians meet at the shared
        // vertex — the natural coaxial construction): the crossing sits
        // at the march boundary and no mid-loop split is needed; the
        // blend seam vertex already lies on both meridians.
        let aligned = (target - u2_first).abs() <= 1e-6
            || (target - u2_last).abs() <= 1e-6
            || ((u2_first - seam2) / period2).fract().abs() <= 1e-9;
        if !inside && aligned {
            None
        } else if !inside {
            return Err(KernelRefusal::internal(KernelStage::Refine, "seam_crossing_bracket", "blend: second seam crossing not bracketed by the march"));
        } else {
            // Fit-space crossing parameter on the fitted cs (Newton via the
            // fitted pcurve so the split lands exactly where the FITTED track
            // crosses the meridian).
            let [fit_low, fit_high] = rows.u_domain;
            let mut p = fit_low
                + (fit_high - fit_low) * {
                    // Seed from the bracketing stations.
                    let mut seed = 0.5;
                    for pair in 0..stations.len() - 1 {
                        let a = stations[pair].uv2[0];
                        let b = stations[pair + 1].uv2[0];
                        if (target - a) * (target - b) <= 0.0 {
                            let local = (target - a) / (b - a);
                            seed = (parameters[pair]
                                + (parameters[pair + 1] - parameters[pair]) * local)
                                .clamp(0.0, 1.0);
                            break;
                        }
                    }
                    seed
                };
            for _ in 0..NEWTON_ITERATIONS {
                let value = rows.cs_pcurve.evaluate(p).or_refuse(KernelStage::Refine, "evaluate")?.x - target;
                if value.abs() <= 1e-12 {
                    break;
                }
                let step = 1e-8;
                let probed = rows.cs_pcurve.evaluate(p + step).or_refuse(KernelStage::Refine, "evaluate")?.x - target;
                let derivative = (probed - value) / step;
                if derivative.abs() <= 1e-14 {
                    return Err(KernelRefusal::non_convergence(KernelStage::Refine, "seam_crossing_newton", "blend: second seam crossing Newton stalled"));
                }
                p -= value / derivative;
            }
            let crossing_v = rows.cs_pcurve.evaluate(p).or_refuse(KernelStage::Refine, "evaluate")?.y;
            Some(SecondSeamSplit {
                fit_parameter: p,
                crossing_v,
                period: period2 * direction,
            })
        }
    } else {
        None
    };

    // A wall that SHIPS short of its construction request says so, typed, on
    // the blend face it built (exact name and surface); a carved wall or a
    // fallback construction makes no request.
    let model_unmet = if carve_fold { None } else { model_unmet };
    let wall_surface = model_unmet.map(|_| rows.surface.clone());
    let built = build_surgery(
        solid,
        edge,
        &first_mate,
        &second_mate,
        rows,
        second_split,
        name,
        crease,
    )?;
    if let (Some((declared, reason, round, stations, ceiling)), Some(surface)) = (model_unmet, wall_surface) {
        crate::blend::record_wall_model_report(crate::blend::WallModelReport {
            name: name.map(str::to_string),
            surface,
            request: crate::KernelTolerances::for_solid(solid, 1e-7).model,
            residual: declared,
            // The request covers the rails (the shipped support curves) and
            // the wall; the residual is the worse of the two.
            detail: request_rails.map(|(rails, unread, wall)| {
                format!("last request read: rails {rails:.3e} off their carriers at j/16 ({unread} interval(s) unread), wall {wall:.3e}")
            }),
            budget: crate::ApproximationBudget {
                reason,
                rounds_used: round,
                rounds_limit: crate::blend::stations::REFINE_ROUNDS,
                stations,
                station_limit: ceiling,
                mechanism: crate::BudgetMechanism::LocalRounds,
                // The request covers rails and wall: which one the residual
                // reads, and the rail intervals it could not read.
                measured_component: match request_rails {
                    Some((rails, _, wall)) if rails > wall => crate::MeasuredComponent::Rails,
                    _ => crate::MeasuredComponent::Wall,
                },
                unread: request_rails.map_or(0, |(_, unread, _)| unread),
            },
        });
    }
    Ok(built)
}

/// A closed spherical hole may meet the carrier's full-domain seam at its
/// loop vertex. The domain rectangle is only a parameter cut, so moving the
/// hole need not move that cut. Separate the ring from the rectangle and give
/// it its own coincident vertex before surgery. Unlike a retraced slit, the
/// opposite seam branches and collapsed pole edges remain in the solid.
fn separate_spherical_domain_cut(
    solid: &BrepSolid,
    edge_id: u64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id) else {
        return Ok(None);
    };
    if edge.start_vertex_id != edge.end_vertex_id {
        return Ok(None);
    }
    let mut result = solid.clone();
    let mut take_id = crate::blend::edge::fresh_id_source(solid);
    let mut separated = false;
    for face in result.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        let Some(atlas) = crate::sphere_chart::SphereAtlas::of_surface(&face.surface) else {
            continue;
        };
        let [u0, u1] = face
            .surface
            .domain_u()
            .or_refuse(KernelStage::Collect, "domain_cut_u")?;
        let [v0, v1] = face
            .surface
            .domain_v()
            .or_refuse(KernelStage::Collect, "domain_cut_v")?;
        if [u0, u1, v0, v1] != [0.0, 1.0, 0.0, 1.0] {
            continue;
        }
        if face
            .surface
            .closed_directions()
            .or_refuse(KernelStage::Collect, "domain_cut_closed")?
            != (true, false)
        {
            continue;
        }
        let band = 1e-8 * atlas.radius;
        let bottom = face
            .surface
            .evaluate(u0, v0)
            .or_refuse(KernelStage::Collect, "domain_cut_bottom")?;
        let top = face
            .surface
            .evaluate(u0, v1)
            .or_refuse(KernelStage::Collect, "domain_cut_top")?;
        if bottom.add(top).scale(0.5).sub(atlas.centre).length() > band
            || (bottom.sub(top).length() - 2.0 * atlas.radius).abs() > band
        {
            continue;
        }
        let mut additions = Vec::new();
        let mut cut_ids = std::collections::HashSet::new();
        for rim in &mut face.loops {
            if rim.coedges.len() < 5 {
                continue;
            }
            let positions: Vec<_> = rim
                .coedges
                .iter()
                .enumerate()
                .filter_map(|(i, c)| (c.edge_id == edge_id).then_some(i))
                .collect();
            if positions.len() != 1 {
                continue;
            }
            let position = positions[0];
            let ring = &rim.coedges[position];
            let [a, b] = ring
                .pcurve
                .domain()
                .or_refuse(KernelStage::Collect, "domain_cut_ring")?;
            if ring
                .pcurve
                .evaluate(a)
                .or_refuse(KernelStage::Collect, "domain_cut_ring_start")?
                .sub(
                    ring.pcurve
                        .evaluate(b)
                        .or_refuse(KernelStage::Collect, "domain_cut_ring_end")?,
                )
                .length()
                > 1e-7
            {
                continue;
            }
            let rest: Vec<_> = rim
                .coedges
                .iter()
                .filter(|c| c.edge_id != edge_id)
                .collect();
            let mut points = Vec::new();
            let mut domain_cut = true;
            for coedge in &rest {
                let [a, b] = coedge
                    .pcurve
                    .domain()
                    .or_refuse(KernelStage::Collect, "domain_cut_trim")?;
                let start = coedge
                    .pcurve
                    .evaluate(a)
                    .or_refuse(KernelStage::Collect, "domain_cut_start")?;
                let end = coedge
                    .pcurve
                    .evaluate(b)
                    .or_refuse(KernelStage::Collect, "domain_cut_end")?;
                points.push(start);
                let degenerate = solid
                    .edges
                    .iter()
                    .any(|e| e.id == coedge.edge_id && e.degenerate);
                let at = |x: f64, low: f64, high: f64| {
                    (x - low).abs() <= 1e-8 || (x - high).abs() <= 1e-8
                };
                let on_boundary = if degenerate {
                    (start.y - end.y).abs() <= 1e-8
                        && at(start.y, v0, v1)
                        && coedge
                            .pcurve
                            .control_points
                            .iter()
                            .all(|p| (p.y / p.w - start.y).abs() <= 1e-8)
                } else {
                    at(start.x, u0, u1)
                        && (start.x - end.x).abs() <= 1e-8
                        && coedge
                            .pcurve
                            .control_points
                            .iter()
                            .all(|p| (p.x / p.w - start.x).abs() <= 1e-8)
                        && rest.iter().filter(|c| c.edge_id == coedge.edge_id).count() == 2
                        && rest
                            .iter()
                            .any(|c| c.edge_id == coedge.edge_id && c.forward != coedge.forward)
                };
                domain_cut &= on_boundary;
            }
            let twice_area: f64 = (0..points.len())
                .map(|i| {
                    let a = points[i];
                    let b = points[(i + 1) % points.len()];
                    a.x * b.y - a.y * b.x
                })
                .sum();
            let expected = (u1 - u0) * (v1 - v0) * if face.same_sense { 1.0 } else { -1.0 };
            if !domain_cut || (0.5 * twice_area - expected).abs() > 1e-7 * expected.abs() {
                continue;
            }
            // A relocated cut must belong only to this sphere. Its vertices
            // may touch the selected ring or collapsed poles, but no other
            // material edge. Its active parameter must already be sphere v.
            let cuts: std::collections::HashSet<_> = rest
                .iter()
                .filter_map(|c| {
                    solid
                        .edges
                        .iter()
                        .find(|e| e.id == c.edge_id && !e.degenerate)
                        .map(|e| e.id)
                })
                .collect();
            for cut in solid.edges.iter().filter(|e| cuts.contains(&e.id)) {
                let users = solid
                    .shells
                    .iter()
                    .flat_map(|s| &s.faces)
                    .flat_map(|f| &f.loops)
                    .flat_map(|l| &l.coedges)
                    .filter(|c| c.edge_id == cut.id)
                    .count();
                let exclusive_vertices = solid
                    .edges
                    .iter()
                    .filter(|e| {
                        e.start_vertex_id == cut.start_vertex_id
                            || e.end_vertex_id == cut.start_vertex_id
                            || e.start_vertex_id == cut.end_vertex_id
                            || e.end_vertex_id == cut.end_vertex_id
                    })
                    .all(|e| e.degenerate || cuts.contains(&e.id) || e.id == edge_id);
                let mut same_parameter = cut.t0 >= v0 && cut.t1 <= v1 && cut.t0 < cut.t1;
                for i in 0..=16 {
                    let t = cut.t0 + (cut.t1 - cut.t0) * i as f64 / 16.0;
                    let point = cut
                        .curve
                        .evaluate(t)
                        .or_refuse(KernelStage::Collect, "domain_cut_curve")?;
                    let carrier = face
                        .surface
                        .evaluate(u0, t)
                        .or_refuse(KernelStage::Collect, "domain_cut_carrier")?;
                    same_parameter &= point.sub(carrier).length() <= band;
                }
                domain_cut &= users == 2 && exclusive_vertices && same_parameter;
            }
            if !domain_cut {
                continue;
            }
            cut_ids.extend(cuts);
            additions.push(LoopRecord {
                id: take_id(),
                coedges: vec![rim.coedges.remove(position)],
            });
            separated = true;
        }
        if !additions.is_empty() {
            // The freed contact can cross the old parameter seam. Move the
            // artificial cut by a quarter turn, which is an exact shift of
            // this rational circle chart, to keep every real trim in-domain.
            let real = |id: u64| {
                !cut_ids.contains(&id) && !solid.edges.iter().any(|e| e.id == id && e.degenerate)
            };
            let curves: Vec<_> = face
                .loops
                .iter()
                .flat_map(|l| &l.coedges)
                .chain(additions.iter().flat_map(|l| &l.coedges))
                .filter(|c| real(c.edge_id) || c.edge_id == edge_id)
                .map(|c| &c.pcurve)
                .collect();
            let mut candidates = Vec::new();
            for shift in [0.25, 0.5, 0.75] {
                let mut clearance = f64::INFINITY;
                let mut fits = true;
                for curve in &curves {
                    let first = curve.control_points[0].x / curve.control_points[0].w;
                    let offset = -(first - shift).floor();
                    for p in &curve.control_points {
                        let u = p.x / p.w - shift + offset;
                        fits &= u >= 0.0 && u <= 1.0;
                        clearance = clearance.min(u.min(1.0 - u));
                    }
                }
                if fits {
                    candidates.push((clearance, shift));
                }
            }
            let Some((_, shift)) = candidates.into_iter().max_by(|a, b| a.0.total_cmp(&b.0)) else {
                return Err(KernelRefusal::unsupported(
                    KernelStage::Collect,
                    "sphere_domain_cut",
                    "blend: no quarter-turn sphere chart keeps the real trims in-domain",
                ));
            };
            let seam_direction = face
                .surface
                .evaluate(shift, 0.5)
                .or_refuse(KernelStage::Collect, "sphere_cut_direction")?
                .sub(atlas.centre);
            let surface = crate::make_sphere_surface_framed(
                atlas.centre,
                atlas.radius,
                atlas.basis[2],
                Some(seam_direction),
            )
            .or_refuse(KernelStage::Collect, "sphere_cut_surface")?;
            // Rebuilding must be only a chart change, including handedness.
            for i in 0..=16 {
                for v in [0.0, 0.3, 0.5, 0.7, 1.0] {
                    let u = i as f64 / 16.0;
                    let old = face
                        .surface
                        .evaluate(u, v)
                        .or_refuse(KernelStage::Collect, "sphere_cut_old")?;
                    let new = surface
                        .evaluate((u - shift).rem_euclid(1.0), v)
                        .or_refuse(KernelStage::Collect, "sphere_cut_new")?;
                    if old.sub(new).length() > band {
                        return Err(KernelRefusal::unsupported(
                            KernelStage::Collect,
                            "sphere_cut_chart",
                            "blend: the sphere quarter turn is not the same carrier",
                        ));
                    }
                }
            }
            let seam = surface
                .iso_curve_u(0.0)
                .or_refuse(KernelStage::Collect, "sphere_cut_seam")?;
            for coedge in face
                .loops
                .iter_mut()
                .flat_map(|l| &mut l.coedges)
                .chain(additions.iter_mut().flat_map(|l| &mut l.coedges))
            {
                if solid
                    .edges
                    .iter()
                    .any(|e| e.id == coedge.edge_id && e.degenerate)
                {
                    continue;
                }
                if !cut_ids.contains(&coedge.edge_id) {
                    let first =
                        coedge.pcurve.control_points[0].x / coedge.pcurve.control_points[0].w;
                    let offset = -(first - shift).floor() - shift;
                    for p in &mut coedge.pcurve.control_points {
                        p.x += offset * p.w;
                    }
                }
            }
            for cut in result.edges.iter_mut().filter(|e| cut_ids.contains(&e.id)) {
                for (vertex_id, t) in [(cut.start_vertex_id, cut.t0), (cut.end_vertex_id, cut.t1)] {
                    let point = seam
                        .evaluate(t)
                        .or_refuse(KernelStage::Collect, "sphere_cut_vertex")?;
                    let vertex = result
                        .vertices
                        .iter_mut()
                        .find(|v| v.id == vertex_id)
                        .ok_or(KernelRefusal::internal(
                            KernelStage::Collect,
                            "sphere_cut_vertex",
                            "blend: sphere seam vertex is missing",
                        ))?;
                    vertex.point = point;
                }
                cut.curve = seam.clone();
            }
            face.surface = surface;
        }
        face.loops.extend(additions);
    }
    if !separated {
        return Ok(None);
    }
    let old_vertex = solid
        .vertices
        .iter()
        .find(|v| v.id == edge.start_vertex_id)
        .ok_or(KernelRefusal::internal(
            KernelStage::Collect,
            "domain_cut_vertex",
            "blend: closed ring vertex is missing",
        ))?;
    let vertex_id = take_id();
    result.vertices.push(VertexRecord {
        id: vertex_id,
        point: old_vertex.point,
    });
    let ring = result
        .edges
        .iter_mut()
        .find(|e| e.id == edge_id)
        .ok_or(KernelRefusal::internal(
            KernelStage::Collect,
            "domain_cut_ring",
            "blend: closed ring edge is missing",
        ))?;
    ring.start_vertex_id = vertex_id;
    ring.end_vertex_id = vertex_id;
    Ok(Some(result))
}

/// Imported faces can carry a slit from a closed rim to a pole and straight
/// back along the same UV track. It has no area and is not a periodic seam
/// (whose two pcurves differ by a period). Moving the rim must not drag this
/// artificial cut onto a different meridian. Cancel only adjacent opposite
/// uses of an edge with no other users, and prove that their UV tracks agree.
fn remove_retraced_cuts(
    solid: &BrepSolid,
    edge_id: u64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    let mut result = solid.clone();
    let mut removed = Vec::new();
    for face in result.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        for rim in &mut face.loops {
            if !rim.coedges.iter().any(|coedge| coedge.edge_id == edge_id) {
                continue;
            }
            loop {
                let n = rim.coedges.len();
                let mut pair = None;
                for i in 0..n {
                    let j = (i + 1) % n;
                    let (a, b) = (&rim.coedges[i], &rim.coedges[j]);
                    if a.edge_id == edge_id || a.edge_id != b.edge_id || a.forward == b.forward {
                        continue;
                    }
                    let users = solid
                        .shells
                        .iter()
                        .flat_map(|s| &s.faces)
                        .flat_map(|f| &f.loops)
                        .flat_map(|l| &l.coedges)
                        .filter(|c| c.edge_id == a.edge_id)
                        .count();
                    if users != 2 {
                        continue;
                    }
                    let [a0, a1] = a
                        .pcurve
                        .domain()
                        .or_refuse(KernelStage::Collect, "cut_domain")?;
                    let [b0, b1] = b
                        .pcurve
                        .domain()
                        .or_refuse(KernelStage::Collect, "cut_domain")?;
                    // Compare the complete reversed rational representation,
                    // not a few probes that could miss a curved excursion.
                    let agrees =
                        a.pcurve.degree == b.pcurve.degree
                            && a.pcurve.knots.len() == b.pcurve.knots.len()
                            && a.pcurve.control_points.len() == b.pcurve.control_points.len()
                            && a.pcurve.knots.iter().zip(b.pcurve.knots.iter().rev()).all(
                                |(x, y)| {
                                    ((x - a0) / (a1 - a0) - (b1 - y) / (b1 - b0)).abs() <= 1e-9
                                },
                            )
                            && a.pcurve
                                .control_points
                                .iter()
                                .zip(b.pcurve.control_points.iter().rev())
                                .all(|(p, q)| {
                                    p.point()
                                        .ok()
                                        .zip(q.point().ok())
                                        .is_some_and(|(p, q)| p.sub(q).length() <= 1e-9)
                                        && (p.w / a.pcurve.control_points[0].w
                                            - q.w / b.pcurve.control_points.last().unwrap().w)
                                            .abs()
                                            <= 1e-9
                                });
                    if agrees {
                        pair = Some((i, j, a.edge_id));
                        break;
                    }
                }
                let Some((i, j, id)) = pair else {
                    break;
                };
                rim.coedges.remove(i.max(j));
                rim.coedges.remove(i.min(j));
                removed.push(id);
            }
        }
    }
    if removed.is_empty() {
        return Ok(None);
    }
    result.edges.retain(|edge| !removed.contains(&edge.id));
    super::open::prune_orphan_vertices(&mut result);
    Ok(Some(result))
}

/// Where the second support crosses ITS seam-structured carrier's seam
/// meridian (both-seam closed edges).
struct SecondSeamSplit {
    fit_parameter: f64,
    crossing_v: f64,
    /// Signed carrier period travelled by the unwrapped pcurve track.
    period: f64,
}

/// Replace the blended edge in both mating loops, trim the seam edge of an
/// anchored closed carrier, and insert the blend face.
/// The carve of a folding closed-edge wall: measure the fitted rows against
/// the carriers between the stations they were fitted through (the network's
/// own rail bar, half `intersection_fit`), trace the crease on the fitted
/// surface and fit it to `pcurve_consistency`, and refuse by name a lens
/// that straddles the wall's own seam (the closed rows join at u = 0 ≡ 1, and
/// the crease tracer reads one parameter rectangle).
/// Top of the closed march's carve ladder: `SEED_INTERVALS << 8` = 4096
/// intervals, the station count the chain lane's ladder also tops out near
/// (`CHAIN_CARVE_MAX_PER_SEGMENT` per segment).
const CARVE_MAX_REFINEMENT_DEPTH: usize = MAX_REFINEMENT_DEPTH + 4;

/// How far the closed rows' rails stand off their carriers BETWEEN the
/// stations they were fitted through, at 0.25/0.5/0.75 of every span — the
/// open march's `rails_off_carriers` reading on closed rows.
fn rails_off_closed_carriers(
    rows: &FittedRows,
    stations: &[Station],
    parameters: &[f64],
    first: &BlendMate,
    second: &BlendMate,
) -> Result<f64, KernelRefusal> {
    Ok(closed_rail_interval_misses(rows, stations, parameters,
        [&first.face.surface, &second.face.surface])?.into_iter().fold(0.0_f64, f64::max))
}

fn carve_closed_wall(
    solid: &BrepSolid,
    rows: &FittedRows,
    stations: &[Station],
    parameters: &[f64],
    first: &BlendMate,
    second: &BlendMate,
    radius: f64,
) -> Result<crate::blend::carve::CarvedCrease, KernelRefusal> {
    let policy = crate::KernelTolerances::for_solid(solid, 1e-7);
    let rail_bar = 0.5 * policy.intersection_fit;
    let off = rails_off_closed_carriers(rows, stations, parameters, first, second)?;
    crate::blend::carve::carve_trace(format_args!(
        "blend closed-edge carve: {} stations, rails {off:.3e} off their carriers between stations (bar {rail_bar:.3e})",
        parameters.len()
    ));
    if off > rail_bar {
        return Err(crate::blend::fold::wall_fold_refusal(
            Vec::new(),
            format!(
                "{} {radius} fits this edge: the wall folds, and the closed march's fitted rails stand {off:.3e} off their carriers between stations (bar {rail_bar:.3e}), so there is no wall accurate enough to carve",
                crate::blend::WALL_FOLDS
            ),
        ));
    }
    let Some(carved) = crate::blend::carve::trace_wall_crease(&rows.surface, radius)? else {
        return Err(KernelRefusal::internal(KernelStage::Refine, "carve_no_fold", "blend closed-edge carve: the wall re-marched under Carve carries no fold to carve, but the march refused one"));
    };
    let [u0, u1] = rows.u_domain;
    let margin = 1e-3 * (u1 - u0);
    let straddles = carved.ends.iter().any(|end| end[0] <= u0 + margin || end[0] >= u1 - margin);
    if straddles {
        return Err(crate::blend::fold::wall_fold_refusal(
            Vec::new(),
            format!(
                "{} {radius} fits this edge: the wall folds, and the fold band straddles the wall's own seam (lens ends at u = {:.6} and {:.6} of [{u0:.3}, {u1:.3}]), which the crease tracer reads as two half-lenses; carving across the seam is not built",
                crate::blend::WALL_FOLDS, carved.ends[0][0], carved.ends[1][0]
            ),
        ));
    }
    crate::blend::carve::fit_crease(&rows.surface, &carved, policy.pcurve_consistency)
}

/// A support rail's pcurve on its mate `surface` (a closed edge's whole rail,
/// or an open edge's rail trimmed to its crossing window), fitted to the
/// rail's own projected track at the pcurve floor
/// (`chain::project_piece_pcurve`, the closed chain's support-piece fitter,
/// which refuses a fit off that floor), on the rail's parameter domain and in
/// the same carrier period as the interpolated pcurve `old` it replaces. A
/// closed rail around a periodic carrier spans a whole period, so its ends are
/// not pinned by the fitter (both would go to one seam); instead an end whose
/// `old` coordinate lies exactly on a seam keeps that coordinate, so the loop
/// still closes on the seam the surgery built it on.
///
/// The shipped curve is read again at the floor (`read_shipped_pcurve`). The
/// fitter's stencil reads j/16 of each TRACK interval and the shipped read
/// j/16 of each span of the fitted curve's own (averaged) knots, so a fit the
/// stencil reads just under the floor can read just over it as shipped (the
/// 2026-09-27 revolve: 9.923e-8 on the stencil, 1.037e-7 at t 0.4902). Such
/// a fit is refitted, at the SAME floor, with its track refined where the
/// shipped read found the miss, up to `SHIPPED_READ_REPAIRS` times; the floor
/// never moves, and a fit read within it the first time ships as before.
pub(super) fn refit_closed_support_pcurve(rail: &NurbsCurve, old: &NurbsCurve, surface: &NurbsSurface) -> Result<NurbsCurve, KernelRefusal> {
    const SHIPPED_READ_REPAIRS: usize = 3;
    let floor = crate::pcurve::PCURVE_REFINEMENT_TOLERANCE;
    let [d0, d1] = rail.domain().or_refuse(KernelStage::Refine, "domain")?;
    let mut near: Vec<f64> = Vec::new();
    let (mut fitted, mut read) = fit_and_read_support(rail, old, surface, &near)?;
    for _ in 0..SHIPPED_READ_REPAIRS {
        if read.0 <= floor || !(d1 > d0) {
            break;
        }
        near.push((read.3 - d0) / (d1 - d0));
        match fit_and_read_support(rail, old, surface, &near) {
            Ok(repaired) => (fitted, read) = repaired,
            Err(_) => break,
        }
    }
    let (miss, standoff, samples, _) = read;
    if !(miss <= floor) {
        return Err(KernelRefusal::non_convergence(KernelStage::Refine, "closed_support_pcurve_floor", format!(
            "{} a blend's support pcurve, as shipped, misses its rail's track by {miss:.3e} at {samples} samples, \
             against a floor of {floor:.1e}",
            crate::blend::PCURVE_OFF_FLOOR,
        )));
    }
    Ok(fitted)
}

/// [`refit_closed_support_pcurve`]'s fit, its track refined near the
/// fractions `near` first, and the shipped curve's read (miss, standoff,
/// samples, worst rail parameter).
fn fit_and_read_support(
    rail: &NurbsCurve,
    old: &NurbsCurve,
    surface: &NurbsSurface,
    near: &[f64],
) -> Result<(NurbsCurve, (f64, f64, usize, f64)), KernelRefusal> {
    let [d0, d1] = rail.domain().or_refuse(KernelStage::Refine, "domain")?;
    // Every foot of the track is sought from the march's own trim `old` at
    // the same rail parameter (`pcurve_guide`), on the branch it runs on.
    let guide = crate::blend::chain::pcurve_guide(old, surface)?;
    let mut fitted = crate::blend::chain::project_piece_pcurve_within(rail, surface, false, false, Some(&guide), near)?;
    // The fitter's parameter is the track fraction; the surgery reads the
    // pcurve at the rail's own parameter.
    let [f0, f1] = fitted.domain().or_refuse(KernelStage::Refine, "domain")?;
    for knot in fitted.knots.iter_mut() {
        *knot = d0 + (*knot - f0) / (f1 - f0) * (d1 - d0);
    }
    let debug = std::env::var_os("BREP_DEBUG_TRACK_FIT").is_some();
    if debug {
        // Diagnostic only: the same read before the period shift and the seam
        // ends are set (the shift alone moves no point of the carrier).
        match crate::blend::track_fit::read_shipped_pcurve(surface, &|t| rail.evaluate(t), &fitted, d0, d1) {
            Ok((miss, standoff, samples, worst)) => eprintln!(
                "TRACK_FIT closed-edge support: before the seam ends, miss {miss:.3e} at t {worst:.9} ({samples} samples), standoff {standoff:.3e}"),
            Err(error) => eprintln!("TRACK_FIT closed-edge support: before the seam ends, unreadable: {error}"),
        }
    }
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let old_start = old.evaluate(d0).or_refuse(KernelStage::Refine, "evaluate")?;
    let old_end = old.evaluate(d1).or_refuse(KernelStage::Refine, "evaluate")?;
    let new_start = fitted.evaluate(d0).or_refuse(KernelStage::Refine, "evaluate")?;
    // Whole periods only: the same carrier point, in the old pcurve's period.
    let shift = |closed: bool, from: f64, to: f64, period: f64| if closed { ((to - from) / period).round() * period } else { 0.0 };
    let (shift_u, shift_v) = (shift(closed_u, new_start.x, old_start.x, u1 - u0), shift(closed_v, new_start.y, old_start.y, v1 - v0));
    for point in fitted.control_points.iter_mut() {
        point.x += shift_u * point.w;
        point.y += shift_v * point.w;
    }
    let on_seam = |closed: bool, value: f64, low: f64, high: f64| {
        let period = high - low;
        closed && ((value - low) / period - ((value - low) / period).round()).abs() * period <= 1e-12 * period.max(1.0)
    };
    let last = fitted.control_points.len() - 1;
    for (index, end) in [(0, old_start), (last, old_end)] {
        let point = &mut fitted.control_points[index];
        if on_seam(closed_u, end.x, u0, u1) {
            point.x = end.x * point.w;
        }
        if on_seam(closed_v, end.y, v0, v1) {
            point.y = end.y * point.w;
        }
    }
    // The curve SHIPPED is not the one the fitter checked (its domain, period
    // and seam ends were edited), so it is read again, at the rail's own
    // parameter, against the fitter's floor; a miss refuses as the fitter's
    // own would, and an unreadable sample refuses.
    let (miss, standoff, samples, worst) =
        crate::blend::track_fit::read_shipped_pcurve(surface, &|t| rail.evaluate(t), &fitted, d0, d1)?;
    if debug {
        eprintln!("TRACK_FIT closed-edge support: shipped pcurve miss {miss:.3e} at t {worst:.9} of [{d0:.9}, {d1:.9}] ({samples} samples); rail standoff from its carrier {standoff:.3e}");
    }
    Ok((fitted, (miss, standoff, samples, worst)))
}


fn build_surgery(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    rows: FittedRows,
    second_split: Option<SecondSeamSplit>,
    name: Option<&str>,
    crease: Option<crate::blend::carve::CarvedCrease>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut result = solid.clone();
    let mut take_id = crate::blend::edge::fresh_id_source(solid);

    let [u_start, u_end] = rows.u_domain;
    let cr_start = rows.cr.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let cs_start = rows.cs.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let vertex1_id = take_id();
    let vertex2_id = take_id();
    result.vertices.push(VertexRecord {
        id: vertex1_id,
        point: cr_start,
    });
    result.vertices.push(VertexRecord {
        id: vertex2_id,
        point: cs_start,
    });

    let cr_edge_id = take_id();
    result.edges.push(EdgeRecord {
        id: cr_edge_id,
        curve: rows.cr.clone(),
        t0: u_start,
        t1: u_end,
        start_vertex_id: vertex1_id,
        end_vertex_id: vertex1_id,
        degenerate: false,
        name: None,
    });
    // Second support: whole closed edge, or two pieces split where it
    // crosses ITS carrier's seam meridian (both-seam closed edges).  Each
    // piece records (edge id, fit-space window, forward pcurve).
    let mut cs_pieces: Vec<(u64, [f64; 2], NurbsCurve)> = Vec::new();
    let mut second_seam_vertex = vertex2_id;
    let mut second_seam_point = cs_start;
    let mut second_crossing_v = 0.0;
    if let Some(split) = &second_split {
        let p = split.fit_parameter;
        second_crossing_v = split.crossing_v;
        let (cs_a, cs_b) = rows.cs.split(p).or_refuse(KernelStage::Refine, "split")?;
        let (pc_a, pc_b) = rows.cs_pcurve.split(p).or_refuse(KernelStage::Refine, "split")?;
        // Bring the wrapped second piece back into the carrier's domain.
        let mut pc_b = pc_b;
        for point in pc_b.control_points.iter_mut() {
            point.x -= split.period * point.w;
        }
        let w2s = take_id();
        second_seam_vertex = w2s;
        second_seam_point = rows.cs.evaluate(p).or_refuse(KernelStage::Refine, "evaluate")?;
        result.vertices.push(VertexRecord {
            id: w2s,
            point: second_seam_point,
        });
        let edge_a = take_id();
        result.edges.push(EdgeRecord {
            id: edge_a,
            curve: cs_a,
            t0: u_start,
            t1: p,
            start_vertex_id: vertex2_id,
            end_vertex_id: w2s,
            degenerate: false,
            name: None,
        });
        cs_pieces.push((edge_a, [u_start, p], pc_a));
        let edge_b = take_id();
        result.edges.push(EdgeRecord {
            id: edge_b,
            curve: cs_b,
            t0: p,
            t1: u_end,
            start_vertex_id: w2s,
            end_vertex_id: vertex2_id,
            degenerate: false,
            name: None,
        });
        cs_pieces.push((edge_b, [p, u_end], pc_b));
    } else {
        let cs_edge_id = take_id();
        result.edges.push(EdgeRecord {
            id: cs_edge_id,
            curve: rows.cs.clone(),
            t0: u_start,
            t1: u_end,
            start_vertex_id: vertex2_id,
            end_vertex_id: vertex2_id,
            degenerate: false,
            name: None,
        });
        cs_pieces.push((cs_edge_id, [u_start, u_end], rows.cs_pcurve.clone()));
    }
    let seam_curve = rows.surface.iso_curve_u(u_start).or_refuse(KernelStage::Refine, "iso_curve_u")?;
    let [seam_t0, seam_t1] = seam_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let blend_seam_id = take_id();
    result.edges.push(EdgeRecord {
        id: blend_seam_id,
        curve: seam_curve,
        t0: seam_t0,
        t1: seam_t1,
        start_vertex_id: vertex1_id,
        end_vertex_id: vertex2_id,
        degenerate: false,
        name: None,
    });

    let old_vertex = edge.start_vertex_id;
    let replace_in_face = |result: &mut BrepSolid,
                           mate: &BlendMate,
                           pieces: &[(u64, [f64; 2], NurbsCurve)],
                           uv_new: [f64; 2]|
     -> Result<bool, KernelRefusal> {
        let v_new = uv_new[1];
        // (edge id, the START vertex is the removed one, the END vertex is).
        let seam_edge_ends: Vec<(u64, bool, bool)> = result
            .edges
            .iter()
            .filter(|candidate| {
                candidate.id != edge.id
                    && (candidate.start_vertex_id == old_vertex
                        || candidate.end_vertex_id == old_vertex)
            })
            .map(|candidate| {
                (
                    candidate.id,
                    candidate.start_vertex_id == old_vertex,
                    candidate.end_vertex_id == old_vertex,
                )
            })
            .collect();
        let seam_edge_ids: Vec<u64> = seam_edge_ends.iter().map(|(id, ..)| *id).collect();
        let pcurve_band = crate::KernelTolerances::for_solid(result, 1e-7).pcurve_consistency;
        let debug = std::env::var("BREP_DEBUG_NETWORK").is_ok();
        let face = result
            .shells
            .iter_mut()
            .flat_map(|shell| &mut shell.faces)
            .find(|face| face.id == mate.face.id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during surgery"))?;
        let face_id = face.id;
        let FaceRecord { surface, loops, .. } = &mut *face;
        let loop_record = &mut loops[mate.loop_index];
        let position = loop_record
            .coedges
            .iter()
            .position(|coedge| coedge.edge_id == edge.id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "edge_coedge", "blend: edge coedge lost during surgery"))?;
        let old_forward = loop_record.coedges[position].forward;
        let old_seam_v = {
            let pcurve = &loop_record.coedges[position].pcurve;
            let [d0, _] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            pcurve.evaluate(d0).or_refuse(KernelStage::Refine, "evaluate")?.y
        };
        // Replacement coedges: pieces run in u order; a reversed loop use
        // takes them in reverse order, each reversed.
        let mut replacements = Vec::new();
        let ordered: Vec<&(u64, [f64; 2], NurbsCurve)> = if old_forward {
            pieces.iter().collect()
        } else {
            pieces.iter().rev().collect()
        };
        for (index, (edge_id, _, pcurve)) in ordered.iter().enumerate() {
            replacements.push(CoedgeRecord {
                id: if index == 0 {
                    loop_record.coedges[position].id
                } else {
                    0 // patched by the caller with fresh ids
                },
                edge_id: *edge_id,
                forward: old_forward,
                pcurve: if old_forward {
                    pcurve.clone()
                } else {
                    pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?
                },
            });
        }
        loop_record
            .coedges
            .splice(position..=position, replacements);
        let mut repaired = false;
        if loop_record.coedges.len() > 1 {
            // Anchored carrier: the loop's other coedges ride the seam
            // meridian between the old edge and the rest of the trim.
            // Pull the pcurve endpoint that met the old edge down to the
            // support crossing.  (The seam EDGE itself is trimmed after
            // both replacements, outside this borrow.)
            for coedge in &mut loop_record.coedges {
                if pieces
                    .iter()
                    .any(|(edge_id, ..)| *edge_id == coedge.edge_id)
                    || !seam_edge_ids.contains(&coedge.edge_id)
                {
                    continue;
                }
                let controls = &coedge.pcurve.control_points;
                let last_index = controls.len() - 1;
                let first_u = controls[0].x / controls[0].w;
                let last_u = controls[last_index].x / controls[last_index].w;
                let constant_u = (first_u - last_u).abs()
                    <= 1e-9 * (1.0 + first_u.abs().max(last_u.abs()));
                // Which end of the pcurve met the removed vertex, and the uv
                // the crossing sits at on this carrier.
                let (target, target_uv, fraction) = if constant_u {
                    // Seam MERIDIAN pcurve: constant u, runs along the v
                    // direction; the endpoint at the removed edge's v moves
                    // onto the support crossing.
                    let first_v = controls[0].y / controls[0].w;
                    let last_v = controls[last_index].y / controls[last_index].w;
                    let target = if (first_v - old_seam_v).abs() < (last_v - old_seam_v).abs() {
                        0
                    } else {
                        last_index
                    };
                    let span = last_v - first_v;
                    let fraction = if span.abs() > 0.0 { (v_new - first_v) / span } else { 0.0 };
                    (target, [first_u, v_new], fraction)
                } else {
                    // The adjacent trim runs along U at constant v — the
                    // carrier's OTHER (v) seam, which is what a closed edge
                    // lying on one whole MERIDIAN borders (an axial-plane
                    // collar on a torus).  Its endpoint moves in U, not V,
                    // and the v-distance rule above cannot pick the end at
                    // all: both ends share the same v, so it always lands on
                    // `last_index`, which is wrong for one of the seam PAIR
                    // (the v-seam edge appears twice in the loop, at v = 0
                    // and v = 1).  Take the end from the TOPOLOGY.
                    let Some((_, start_is_old, end_is_old)) = seam_edge_ends
                        .iter()
                        .find(|(id, ..)| *id == coedge.edge_id)
                        .copied()
                    else {
                        continue;
                    };
                    if start_is_old == end_is_old {
                        return Err(KernelRefusal::ill_posed(KernelStage::Sew, "neighbour_trim_end", format!(
                            "blend: neighbour edge {} meets the blended edge at both ends; \
                             its trim end is ambiguous",
                            coedge.edge_id
                        )));
                    }
                    // `forward` maps the coedge's pcurve domain start onto the
                    // edge's t0 (its start vertex).
                    let target = if start_is_old == coedge.forward { 0 } else { last_index };
                    let v_end = controls[target].y / controls[target].w;
                    let span = last_u - first_u;
                    let fraction = if span.abs() > 0.0 { (uv_new[0] - first_u) / span } else { 0.0 };
                    (target, [uv_new[0], v_end], fraction)
                };
                // The trim is a SPLIT of the pcurve at the crossing's own
                // parameter, never a moved control point.  Moving the end
                // control point of a pcurve with interior knots re-slopes its
                // end span alone and breaks the pcurve ↔ edge parameter
                // correspondence validate reads (measured 2026-10-03 on the
                // torus-box-edge and axial-collar documents: the torus's
                // v-seam pcurve, a 3-span polyline, read 0.548 mm off its
                // edge at the same parameter while its image was on the edge
                // to 1e-10).  The seam-edge pass below projects the same
                // physical point onto the edge curve for the edge's own
                // parameter, so both sides keep the correspondence they had.
                let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                let target_point = surface
                    .evaluate_extended(target_uv[0], target_uv[1])
                    .or_refuse(KernelStage::Refine, "evaluate_extended")?;
                // Seeded at the AFFINE fraction of the pcurve's extent in the
                // moving direction (the seed `trim_edge_at` uses), not at the
                // pcurve's end: a crossing deep in a long seam would hand the
                // point solver a far seed and turn a blend that built into a
                // refusal.  The solver refines the seed to the physical point.
                let seed = (q0 + (q1 - q0) * fraction).clamp(q0.min(q1), q0.max(q1));
                let q_split = trim_parameter_at_point(
                    surface, &coedge.pcurve, seed, target_point, pcurve_band,
                )?;
                let interior = q_split > q0 + 1e-9 * (q1 - q0) && q_split < q1 - 1e-9 * (q1 - q0);
                if debug {
                    eprintln!(
                        "closed-edge seam trim: face {face_id} edge {} coedge {} ({}): {} at q {q_split:.9} of [{q0:.6}, {q1:.6}], {} control points",
                        coedge.edge_id, coedge.id, if constant_u { "meridian, constant u" } else { "along u" },
                        if interior { "SPLIT" } else { "EXTEND" }, controls.len()
                    );
                }
                if interior {
                    let (low, high) = coedge.pcurve.split(q_split).or_refuse(KernelStage::Refine, "split")?;
                    coedge.pcurve = if target == 0 { high } else { low };
                } else {
                    // The crossing lies at or past the pcurve's end: the seam
                    // EXTENDS to meet it.  Exact for a two-point linear
                    // pcurve, whose one span IS the whole parameterisation;
                    // anything else would need a refit and is refused by name.
                    if coedge.pcurve.degree != 1 || controls.len() != 2 {
                        return Err(KernelRefusal::unsupported(KernelStage::Sew, "neighbour_seam_extend_curved", format!(
                            "blend: neighbour seam edge {} would have to extend past its own end to meet the blend's support crossing, and its pcurve is not a single straight span (degree {}, {} control points)",
                            coedge.edge_id, coedge.pcurve.degree, controls.len()
                        )));
                    }
                    let w = coedge.pcurve.control_points[target].w;
                    coedge.pcurve.control_points[target] = crate::Vec4::from_point(
                        Vec3::new(target_uv[0], target_uv[1], 0.0), w,
                    );
                }
                repaired = true;
            }
        }
        Ok(repaired)
    };
    let uv1_new = rows.cr_pcurve.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let v1_new = uv1_new.y;
    let cr_pieces = vec![(cr_edge_id, [u_start, u_end], rows.cr_pcurve.clone())];
    let mut repaired = replace_in_face(&mut result, first, &cr_pieces, [uv1_new.x, v1_new])?;
    let uv2_new = rows.cs_pcurve.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let v2_new = if second_split.is_some() {
        second_crossing_v
    } else {
        uv2_new.y
    };
    repaired |= replace_in_face(&mut result, second, &cs_pieces, [uv2_new.x, v2_new])?;

    // A contact circle about an axis oblique to a sphere's stored polar axis
    // can wind once around that sphere's U period.  Such a circle is not an
    // ordinary hole in the full-domain sphere rectangle: it bounds a cap with
    // one of the collapsed pole rims.  Keep that pole as the companion loop so
    // containment, integration, and tessellation see the actual cap topology.
    let collapse_winding_sphere_cap =
        |result: &mut BrepSolid, face_id: u64, support_ids: &[u64]| -> Result<(), KernelRefusal> {
            let face = result
                .shells
                .iter_mut()
                .flat_map(|shell| &mut shell.faces)
                .find(|face| face.id == face_id)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "sphere_carrier", "blend: sphere carrier lost during cap surgery"))?;
            if !matches!(
                face.surface.analytic(),
                Some(crate::AnalyticSurface::Sphere { .. })
            ) || face.surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")? != (true, false)
            {
                return Ok(());
            }
            let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
            let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
            let period = u1 - u0;
            let support_loop = face.loops.iter().position(|loop_record| {
                !loop_record.coedges.is_empty()
                    && loop_record
                        .coedges
                        .iter()
                        .all(|coedge| support_ids.contains(&coedge.edge_id))
            });
            let Some(support_loop) = support_loop else {
                return Ok(());
            };
            if face.loops[support_loop].coedges.len() != 1 {
                return Ok(());
            }
            let support = &face.loops[support_loop].coedges[0];
            let [s0, s1] = support.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let support_start = support.pcurve.evaluate(s0).or_refuse(KernelStage::Refine, "evaluate")?;
            let support_end = support.pcurve.evaluate(s1).or_refuse(KernelStage::Refine, "evaluate")?;
            let support_winding = support_end.x - support_start.x;
            if (support_winding.abs() - period).abs() > 0.05 * period {
                return Ok(());
            }

            let mut pole: Option<(usize, CoedgeRecord)> = None;
            for (loop_index, loop_record) in face.loops.iter().enumerate() {
                if loop_index == support_loop {
                    continue;
                }
                for coedge in &loop_record.coedges {
                    let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                    let start = coedge.pcurve.evaluate(p0).or_refuse(KernelStage::Refine, "evaluate")?;
                    let end = coedge.pcurve.evaluate(p1).or_refuse(KernelStage::Refine, "evaluate")?;
                    let winding = end.x - start.x;
                    let at_pole = (start.y - v0).abs() <= 1e-8 || (start.y - v1).abs() <= 1e-8;
                    if at_pole
                        && (end.y - start.y).abs() <= 1e-8
                        && (winding.abs() - period).abs() <= 0.05 * period
                        && winding * support_winding < 0.0
                    {
                        pole = Some((loop_index, coedge.clone()));
                        break;
                    }
                }
                if pole.is_some() {
                    break;
                }
            }
            let Some((pole_loop, pole_coedge)) = pole else {
                return Ok(());
            };
            let support_record = face.loops[support_loop].clone();
            let pole_id = face.loops[pole_loop].id;
            face.loops = vec![
                LoopRecord {
                    id: pole_id,
                    coedges: vec![pole_coedge],
                },
                support_record,
            ];
            Ok(())
        };
    collapse_winding_sphere_cap(&mut result, first.face.id, &[cr_edge_id])?;
    let cs_edge_ids: Vec<u64> = cs_pieces.iter().map(|(edge_id, ..)| *edge_id).collect();
    collapse_winding_sphere_cap(&mut result, second.face.id, &cs_edge_ids)?;
    // Patch the zero coedge ids introduced for extra pieces.
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            for loop_record in &mut face.loops {
                for coedge in &mut loop_record.coedges {
                    if coedge.id == 0 {
                        coedge.id = take_id();
                    }
                }
            }
        }
    }

    // Trim seam EDGES that ended on the removed vertex: their endpoint
    // moves onto the support-curve crossing on THEIR carrier (first
    // mate's seam -> the blend seam vertex; second mate's -> its own
    // seam-crossing vertex).
    let new_edge_ids: Vec<u64> = std::iter::once(cr_edge_id)
        .chain(cs_pieces.iter().map(|(edge_id, ..)| *edge_id))
        .chain(std::iter::once(blend_seam_id))
        .collect();
    let first_face_edges: Vec<u64> = first
        .face
        .loops
        .iter()
        .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.edge_id))
        .collect();
    for seam_edge in result.edges.iter_mut() {
        if new_edge_ids.contains(&seam_edge.id) {
            continue;
        }
        if seam_edge.start_vertex_id != old_vertex && seam_edge.end_vertex_id != old_vertex {
            continue;
        }
        let (target_vertex, target_v, target_point) = if first_face_edges.contains(&seam_edge.id) {
            (vertex1_id, v1_new, cr_start)
        } else {
            (second_seam_vertex, v2_new, second_seam_point)
        };
        let [c0, c1] = seam_edge.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let clamped = target_v.clamp(c0.min(c1), c0.max(c1));
        // Using the support crossing's V as the neighbour's CURVE parameter is
        // only valid when that neighbour is the carrier's seam MERIDIAN, whose
        // 3D curve is parameterized by v.  Verify it against the vertex the
        // endpoint is moving to; a neighbour running along U instead (the
        // carrier's v-seam, bordered by a collar edge that lies on one whole
        // meridian) lands nowhere near it — the 2026-09-01 report trimmed the
        // torus' equator seam to u = 0, 10.885 away from its own vertex, and
        // the solid failed validation with "curve start does not match vertex".
        let band = 1e-6 * (1.0 + target_point.length());
        let split_parameter = if seam_edge.id == edge.id {
            // The BLENDED edge itself still carries the removed vertex at this
            // point.  Its coedges have already been replaced in both mates, so
            // the used-edge sweep below discards it; leave its (meaningless)
            // trim exactly as it was rather than asking a doomed edge to pass
            // through the support start.
            clamped
        } else {
            match seam_edge.curve.evaluate(clamped) {
                Ok(point) if point.sub(target_point).length() <= band => clamped,
                _ => {
                    repaired = true;
                    let projection =
                        crate::project_point_to_curve(&seam_edge.curve, target_point).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
                    if projection.distance > band {
                        return Err(KernelRefusal::unsupported(KernelStage::Sew, "neighbour_off_support", format!(
                            "blend: neighbour edge {} does not pass through the blend's support \
                             start (off by {:.9}); the trim has no parameter to move to",
                            seam_edge.id, projection.distance
                        )));
                    }
                    projection.u.clamp(c0.min(c1), c0.max(c1))
                }
            }
        };
        if seam_edge.start_vertex_id == old_vertex {
            seam_edge.t0 = split_parameter;
            seam_edge.start_vertex_id = target_vertex;
        }
        if seam_edge.end_vertex_id == old_vertex {
            seam_edge.t1 = split_parameter;
            seam_edge.end_vertex_id = target_vertex;
        }
    }

    // Blend face: outward orientation matches F1's outward at the v=0 rim.
    let mid_u = (u_start + u_end) * 0.5;
    let blend_normal = raw_normal(&rows.surface, mid_u, 0.0)?;
    let station_uv = rows.cr_pcurve.evaluate(mid_u).or_refuse(KernelStage::Refine, "evaluate")?;
    let n1 = raw_normal(&first.face.surface, station_uv.x, station_uv.y)?;
    let out1 = if first.face.same_sense {
        n1
    } else {
        n1.scale(-1.0)
    };
    let same_sense = blend_normal.dot(out1) >= 0.0;
    let loop_id = take_id();
    let mut coedges = vec![
        CoedgeRecord {
            id: take_id(),
            edge_id: cr_edge_id,
            forward: true,
            pcurve: crate::sweep_topology::parameter_line(u_start, 0.0, u_end, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
        },
        CoedgeRecord {
            id: take_id(),
            edge_id: blend_seam_id,
            forward: true,
            pcurve: crate::sweep_topology::parameter_line(u_end, 0.0, u_end, 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
        },
    ];
    for (edge_id, window, _) in cs_pieces.iter().rev() {
        coedges.push(CoedgeRecord {
            id: take_id(),
            edge_id: *edge_id,
            forward: false,
            pcurve: crate::sweep_topology::parameter_line(window[1], 1.0, window[0], 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
        });
    }
    coedges.push(CoedgeRecord {
        id: take_id(),
        edge_id: blend_seam_id,
        forward: false,
        pcurve: crate::sweep_topology::parameter_line(u_start, 1.0, u_start, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
    });
    // The coedges above trace the parameter rectangle counter-clockwise in
    // (u, v); that traversal is only the outward boundary when the blend
    // surface normal already points outward (same_sense).  When it does not
    // (a mate whose support rim runs the other way — e.g. the reversed cap
    // of a seam carrier), the loop must wind the OTHER way so its coedges
    // traverse the shared support edges opposite to the mate, and so the
    // classification tangent test (parameter_point_in_face) stays consistent
    // with same_sense.  Reversing the coedge order and each coedge (forward
    // flag + pcurve) flips the winding without touching same_sense.
    if !same_sense {
        coedges.reverse();
        for coedge in &mut coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
    }
    let mut loops = vec![LoopRecord {
        id: loop_id,
        coedges,
    }];
    // The CARVED lens: an INNER loop of two coedges on ONE new edge — the
    // crease — both of them this face's own, the slit pattern the closed
    // chain lane sews (`chain/closed_surgery.rs`): the two sheets of the wall
    // are one surface, so the curve where they meet has one edge and two
    // coedges.  The lens between them is the part of the swept envelope
    // inside the swept ball volume, and dropping it is what makes the wall
    // the blend.  Winding: the inner loop runs AGAINST the outer one, read
    // off both loops' signed areas rather than assumed.
    if let Some(crease) = crease {
        let start_vertex = take_id();
        let end_vertex = take_id();
        let crease_edge = take_id();
        result.vertices.push(VertexRecord { id: start_vertex, point: crease.start });
        result.vertices.push(VertexRecord { id: end_vertex, point: crease.end });
        let [crease_t0, crease_t1] = crease.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        result.edges.push(EdgeRecord {
            id: crease_edge,
            curve: crease.curve,
            t0: crease_t0,
            t1: crease_t1,
            start_vertex_id: start_vertex,
            end_vertex_id: end_vertex,
            degenerate: false,
            name: name.map(|value| format!("{value}:CREASE")),
        });
        let outer = crate::blend::chain::signed_area(&loops[0].coedges)?;
        let forward_first = crate::blend::chain::signed_area(&[
            CoedgeRecord { id: 0, edge_id: crease_edge, forward: true, pcurve: crease.first.clone() },
            CoedgeRecord { id: 0, edge_id: crease_edge, forward: false, pcurve: crease.second.reversed().or_refuse(KernelStage::Refine, "reversed")? },
        ])?;
        let inner = if outer * forward_first < 0.0 {
            vec![
                CoedgeRecord { id: take_id(), edge_id: crease_edge, forward: true, pcurve: crease.first },
                CoedgeRecord { id: take_id(), edge_id: crease_edge, forward: false, pcurve: crease.second.reversed().or_refuse(KernelStage::Refine, "reversed")? },
            ]
        } else {
            vec![
                CoedgeRecord { id: take_id(), edge_id: crease_edge, forward: true, pcurve: crease.second },
                CoedgeRecord { id: take_id(), edge_id: crease_edge, forward: false, pcurve: crease.first.reversed().or_refuse(KernelStage::Refine, "reversed")? },
            ]
        };
        loops.push(LoopRecord { id: take_id(), coedges: inner });
    }
    let blend_face = FaceRecord {
        id: take_id(),
        surface: rows.surface,
        same_sense,
        loops,
        name: name.map(|value| value.to_string()),
    };
    let shell_index = result
        .shells
        .iter()
        .position(|shell| shell.faces.iter().any(|face| face.id == first.face.id))
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_shell", "blend: mate shell lost during surgery"))?;
    result.shells[shell_index].faces.push(blend_face);

    let used_edges: std::collections::HashSet<u64> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    result
        .edges
        .retain(|candidate| used_edges.contains(&candidate.id));
    let used_vertices: std::collections::HashSet<u64> = result
        .edges
        .iter()
        .flat_map(|candidate| [candidate.start_vertex_id, candidate.end_vertex_id])
        .collect();
    result
        .vertices
        .retain(|candidate| used_vertices.contains(&candidate.id));
    // A surgery that had to REPAIR a neighbour trim took a branch no existing
    // fixture exercises, so it does not get the benefit of the doubt: prove the
    // result before returning it.  Paths that never repaired are untouched (the
    // check does not run), so this cannot slow or change anything that already
    // works — and the new branch can never ship a plausible-looking wrong solid
    // the way the un-repaired trim did.
    if repaired {
        let problems = result.validate();
        if !problems.is_empty() {
            return Err(KernelRefusal::internal(KernelStage::Validate, "neighbour_trim_closure", format!(
                "blend: the repaired neighbour trim did not close ({} issue(s), first: {})",
                problems.len(),
                problems
                    .first()
                    .map(|issue| issue.message.as_str())
                    .unwrap_or("unknown")
            )));
        }
    }
    Ok(result)
}
