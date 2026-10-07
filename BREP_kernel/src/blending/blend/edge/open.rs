use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// General OPEN-edge rolling-ball fillet or chamfer: §4.9 march +
/// §6.9 surgery with transverse edges on the two end faces.
pub fn blend_open_edge(
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
    blend_open_edge_impl(solid, edge_id, &|_| radius, Some(radius), chamfer, name)
}

/// `constant`: the radius when the blend came in through a CONSTANT-radius
/// entry point (`blend_open_edge`), `None` for a variable profile — the only
/// provenance on which the wall is judged against one rolling ball.
pub(super) fn blend_open_edge_impl(
    solid: &BrepSolid,
    edge_id: u64,
    radius_at: &dyn Fn(f64) -> f64,
    constant: Option<f64>,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| format!("blend: edge {edge_id} not found")).or_refuse(KernelStage::Refine, "ok_or_else")?;
    if edge.start_vertex_id == edge.end_vertex_id {
        return Err(KernelRefusal::input(KernelStage::Collect, "open_edge", "blend: blend_open_edge requires an open edge"));
    }
    let (first_face, first_loop, first_coedge) = locate_mate(solid, edge_id, None)?;
    let (second_face, second_loop, second_coedge) =
        locate_mate(solid, edge_id, Some((first_face.id, first_loop)))?;
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
    // End topology: boundary edges of both mates at each end vertex, the
    // single face across the corner, and the support crossings.  When a
    // prior fillet has already consumed one of the end corners, the support
    // crossing can land at the very rim of the marched rows (the prior
    // blend's transverse arc meets our contact line right at its base).
    // March with a growing overshoot until every crossing lands strictly
    // inside the fitted-row domain so the surgery can trim cleanly.
    let debug = std::env::var("BREP_DEBUG_BLEND_MARCH").is_ok();
    if debug {
        let sp = edge.curve.evaluate(edge.t0);
        let ep = edge.curve.evaluate(edge.t1);
        eprintln!(
            "OPEN edge {} v{}->v{} first_face {} second_face {} p0={:?} p1={:?}",
            edge.id,
            edge.start_vertex_id,
            edge.end_vertex_id,
            first_face.id,
            second_face.id,
            sp,
            ep
        );
    }
    // The rung's MEASUREMENT (before anything is built on it), over what it
    // ships: each rail against its carrier's extension over its own crossing
    // window (`rails_on_shipped_windows`, at the network lane's rail bar, half
    // `intersection_fit`), and a constant-radius FILLET's wall over those
    // windows' envelope against the rolling ball's own sweep (the declared
    // reader the chains use, at `intersection_fit`).
    let rail_bar = 0.5 * crate::KernelTolerances::for_solid(solid, 1e-7).intersection_fit;
    let wall_bar = crate::KernelTolerances::for_solid(solid, 1e-7).intersection_fit;
    let model = crate::KernelTolerances::for_solid(solid, 1e-7).model;
    // Constant-radius PROVENANCE (the entry point), never sampled constancy:
    // a variable profile can agree at any finite set of samples.
    let wall_radius = constant.filter(|radius| radius.is_finite() && *radius > 0.0);
    let wall_check = !chamfer && wall_radius.is_some() && crate::blend::station_refinement_on();
    let compute = |overshoot_fraction: f64, station_count: usize| -> Result<(FittedRows, Vec<EndSurgery>, bool, OpenRungReading), KernelRefusal> {
        // The open-edge surgery locates its ends by support crossings, not by
        // the marched vertex stations; the snapped indices only break the fit
        // where the carriers' extension starts (`fit_open_rows`).
        let (stations, vertex_indices, refinement, dense) = march_open_stations_core(
            edge,
            &first_mate,
            &second_mate,
            radius_at,
            overshoot_fraction,
            [edge.t0, edge.t1],
            station_count,
            [false, false],
            [false, false],
            crate::blend::station_refinement_on().then_some(rail_bar),
            if wall_check { OPEN_WALL_CENTRES } else { 0 },
        )?;
        let parameters = station_parameters(&stations);
        let rows = fit_open_rows(
            &stations,
            &parameters,
            chamfer,
            Some(vertex_indices),
            extrusion_direction(edge, &first_mate, &second_mate),
        )?;
        let mut ends = Vec::with_capacity(2);
        let mut in_range = true;
        for (vertex, at_start) in [(edge.start_vertex_id, true), (edge.end_vertex_id, false)] {
            let (end, side_in_range) = resolve_free_end(
                solid,
                edge_id,
                &first_mate,
                &second_mate,
                &rows,
                vertex,
                at_start,
            )?;
            if debug {
                eprintln!(
                    "  [os {overshoot_fraction:.2}] end v{vertex} at_start={at_start} end_face={} first_boundary={}(t={:.4}) second_boundary={}(t={:.4}) cr={:.4} cs={:.4} in_range={side_in_range}",
                    end.end_face_id,
                    end.first_edge_id,
                    end.first_edge_parameter,
                    end.second_edge_id,
                    end.second_edge_parameter,
                    end.cr_parameter,
                    end.cs_parameter,
                );
            }
            in_range &= side_in_range;
            ends.push(end);
        }
        // The measurement covers what SHIPS: each rail over its own crossing
        // window (the `cr`/`cs` parameters `build_open_surgery` trims it to,
        // any retained overshoot included), and the wall over the envelope of
        // both windows -- not merely between the vertex stations.
        let windows = [
            shipped_window(&parameters, ends[0].cr_parameter, ends[1].cr_parameter),
            shipped_window(&parameters, ends[0].cs_parameter, ends[1].cs_parameter),
        ];
        let (rails, rail_samples) =
            rails_on_shipped_windows(&rows, &parameters, &stations, [&first_mate.face.surface, &second_mate.face.surface], windows);
        let walls = match (wall_check, windows) {
            (false, _) => Vec::new(),
            (true, [Some(first), Some(second)]) => open_wall_readings_why(
                &rows,
                &parameters,
                &stations,
                [first[0].min(second[0]), first[1].max(second[1])],
                &dense,
                wall_radius.unwrap_or(f64::NAN),
            ),
            // No shipped window to read: no wall coverage, never acceptance.
            (true, _) => vec![Err("no shipped window to read".to_string())],
        };
        let first_unread = walls.iter().find_map(|wall| wall.as_ref().err().cloned());
        let walls: Vec<Option<f64>> = if wall_check && windows.iter().any(|window| window.is_none()) {
            Vec::new()
        } else {
            walls.into_iter().map(|wall| wall.ok()).collect()
        };
        let reading = OpenRungReading {
            station_count,
            stations: stations.len(),
            rails,
            rail_samples,
            walls,
            first_unread,
            refine_exit: refinement.exit,
            inserted: refinement.inserted,
        };
        Ok((rows, ends, in_range, reading))
    };

    // Radius-aware seeded first attempt (occt-filleting-system-study §6(b)
    // lesson 7): OCCT floors corner extensions at 1.5·max_radius
    // (`ExtentTwoCorner`) instead of probing blindly.  Convert that floor to
    // an overshoot fraction of THIS edge: the contact rails sit r·tan(α/2)
    // from the edge (α = sign-adjusted angle between the mates' raw normals
    // at mid-edge — the march's own `cos_alpha`), so seed with 1.5× the
    // widest end radius's rail offset over the edge arc length, clamped to
    // the ladder's proven [0.08, 0.45] envelope.  Deterministic; `None` when
    // it degenerates or merely reproduces the ladder's first rung.
    let seeded_fraction: Option<f64> = (|| {
        let span = edge.t1 - edge.t0;
        let mut length = 0.0f64;
        let mut previous = edge.curve.evaluate(edge.t0).ok()?;
        for index in 1..=16 {
            let t = edge.t0 + span * index as f64 / 16.0;
            let point = edge.curve.evaluate(t).ok()?;
            length += point.sub(previous).length();
            previous = point;
        }
        if !(length > 0.0) || !length.is_finite() {
            return None;
        }
        let mid_t = edge.t0 + span * 0.5;
        let uv1 = edge_uv_on_face(first_mate.coedge, edge, mid_t).ok()?;
        let uv2 = edge_uv_on_face(second_mate.coedge, edge, mid_t).ok()?;
        let n1 = raw_normal(&first_mate.face.surface, uv1[0], uv1[1]).ok()?;
        let n2 = raw_normal(&second_mate.face.surface, uv2[0], uv2[1]).ok()?;
        let cos_alpha =
            (rho1.signum() * rho2.signum() * n1.dot(n2)).clamp(-1.0, 1.0);
        // tan(α/2) = √((1−cosα)/(1+cosα)); the tangent-offset factor from
        // the edge to each rail (box: α = π/2 → offset = r).
        let tan_half = ((1.0 - cos_alpha).max(0.0) / (1.0 + cos_alpha).max(1e-9)).sqrt();
        let end_radius = radius_at(edge.t0).abs().max(radius_at(edge.t1).abs());
        let fraction = (1.5 * end_radius * tan_half / length).clamp(0.08, 0.45);
        if !fraction.is_finite() || (fraction - 0.08).abs() < 1e-12 {
            return None;
        }
        Some(fraction)
    })();

    // Retry with a growing overshoot; keep the first attempt whose crossings
    // are all in range, else fall back to the widest march tried.  The seeded
    // attempt is accepted ONLY when its crossings land in range — otherwise
    // the blind ladder below runs exactly as before (including its
    // fallback-to-first-Ok semantics), so the seed can improve the first
    // landing but never change the fallback behaviour.
    let choose = |station_count: usize| -> Result<(FittedRows, Vec<EndSurgery>, OpenRungReading), KernelRefusal> {
    let mut chosen: Option<(FittedRows, Vec<EndSurgery>, OpenRungReading)> = None;
    let mut last_error: Option<KernelRefusal> = None;
    // A wall that FOLDS is the shape's answer, not this rung's: every wider
    // overshoot marches the same centre curve through the same bend, so the
    // ladder is left at the first fold rather than re-proving it four times.
    if let Some(fraction) = seeded_fraction {
        match compute(fraction, station_count) {
            Ok((rows, ends, true, reading)) => chosen = Some((rows, ends, reading)),
            Ok(_) => {}
            Err(error) if crate::blend::is_wall_fold(&error) => return Err(error),
            Err(error) => last_error = Some(error),
        }
    }
    if chosen.is_none() {
        for &overshoot_fraction in &[0.08f64, 0.16, 0.28, 0.45] {
            match compute(overshoot_fraction, station_count) {
                Ok((rows, ends, in_range, reading)) => {
                    let fallback = chosen.is_none();
                    if in_range {
                        chosen = Some((rows, ends, reading));
                        break;
                    } else if fallback {
                        chosen = Some((rows, ends, reading));
                    }
                }
                Err(error) if crate::blend::is_wall_fold(&error) => return Err(error),
                Err(error) => last_error = Some(error),
            }
        }
    }
    chosen.ok_or_else(|| {
        last_error.unwrap_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Refine,
                "open_march",
                "blend: open march failed at every overshoot",
            )
        })
    })
    };
    // The LADDER: the open march's existing budget — STATIONS doubling to
    // MAX_OPEN_STATIONS, each rung with its local refinement (REFINE_ROUNDS,
    // REFINE_STATION_FACTOR) against the rail bar — climbed until a rung is
    // ACCEPTED (rails and wall inside their bars), then, for a plain fillet,
    // further toward `model` on the wall. A rung the request climbs to that
    // fails acceptance is not taken: the accepted rung ships and says so,
    // typed. A ladder that never accepts ships its top rung with a named note,
    // as the network lane's open march does (a body that built before is not
    // turned into a refusal here) -- but it is NOT an accepted rung: it ships
    // with a typed `blend.wall_acceptance` report of its measured deficit.
    // Acceptance is stated positively: every reading present, finite and
    // inside its bar (a NaN or an empty coverage never passes).
    let accepts = |reading: &OpenRungReading| {
        reading.rails_read()
            && reading.rails <= rail_bar
            && (!wall_check || reading.worst_wall().is_some_and(|wall| wall <= wall_bar))
    };
    let meets_model = |reading: &OpenRungReading| reading.worst_wall().is_some_and(|wall| wall <= model);
    let mut station_count = STATIONS;
    let mut accepted: Option<(FittedRows, Vec<EndSurgery>, OpenRungReading)> = None;
    // The top rung of a ladder that NEVER accepted: shipped as built, kept
    // apart from `accepted` so it is never reported as an accepted parent.
    let mut unaccepted: Option<(FittedRows, Vec<EndSurgery>, OpenRungReading)> = None;
    let mut request_rungs = 0usize;
    let mut unmet: Option<crate::BudgetReason> = None;
    loop {
        // A rung climbed to after acceptance is a request rung ATTEMPTED,
        // whatever it returns.
        if accepted.is_some() {
            request_rungs += 1;
        }
        let chosen = choose(station_count);
        let (rows, ends, reading) = match chosen {
            Ok(chosen) => chosen,
            // A request rung that cannot be built: the accepted rung stands.
            Err(_) if accepted.is_some() => {
                unmet = Some(crate::BudgetReason::Incoherent);
                break;
            }
            Err(error) => return Err(error),
        };
        #[cfg(not(test))]
        let reject_request = false;
        crate::blend::carve::carve_trace(format_args!(
            "blend open edge {}: {} stations a rung ({} in all), rails {:.3e} (bar {rail_bar:.3e}), wall {:.3e} (bar {wall_bar:.3e}){}",
            edge.id, reading.station_count, reading.stations, reading.rails,
            reading.worst_wall().unwrap_or(f64::INFINITY),
            if accepted.is_some() { ", a request rung" } else { "" }
        ));
        let accepted_now = accepts(&reading) && !reject_request;
        if accepted.is_none() {
            if accepted_now {
                let met = !wall_check || meets_model(&reading);
                accepted = Some((rows, ends, reading));
                if met {
                    break;
                }
            } else if station_count < MAX_OPEN_STATIONS {
                station_count *= 2;
                continue;
            } else {
                crate::blend::record_blend_note(format!(
                    "blend: open edge {} at {} stations a rung ({} in all) — the top of the open march's \
                     ladder — still leaves its rails {:.3e} from their carriers (bar {rail_bar:.3e}) or its \
                     wall off the rolling ball (bar {wall_bar:.3e}); the wall ships as built, unaccepted",
                    edge.id, reading.station_count, reading.stations, reading.rails
                ));
                unaccepted = Some((rows, ends, reading));
                break;
            }
        } else {
            if !accepted_now {
                unmet = Some(crate::BudgetReason::Rejected);
                break;
            }
            let met = meets_model(&reading);
            accepted = Some((rows, ends, reading));
            if met {
                unmet = None;
                break;
            }
        }
        // Accepted, short of `model`: one more rung if the budget allows.
        if station_count < MAX_OPEN_STATIONS {
            station_count *= 2;
        } else {
            unmet = Some(crate::BudgetReason::StationCeiling);
            break;
        }
    }
    let (shipped_accepted, (rows, ends, reading)) = match (accepted, unaccepted) {
        (Some(rung), _) => (true, rung),
        (None, Some(rung)) => (false, rung),
        (None, None) => {
            return Err(KernelRefusal::internal(KernelStage::Refine, "open_ladder", "blend: the open ladder ended without a rung"));
        }
    };
    // What the shipped wall owes, typed: an UNACCEPTED rung its measured
    // acceptance deficit (or that a reading could not be taken); an accepted
    // fillet short of `model` its construction request.
    let report: Option<(crate::BudgetReason, f64, f64, String)> = if !shipped_accepted {
        Some(acceptance_deficit(&reading, rail_bar, wall_bar, wall_check, reading.rails_read(), "no rung of the open ladder was accepted"))
    } else if wall_check && !meets_model(&reading) {
        let reason = unmet.unwrap_or(crate::BudgetReason::StationCeiling);
        // An accepted rung's every wall interval was read (acceptance needs it).
        let declared = reading.worst_wall().unwrap_or(f64::INFINITY);
        Some((reason, model, declared, String::new()))
    } else {
        None
    };

    let [start_end, finish_end] = match <[EndSurgery; 2]>::try_from(ends) {
        Ok(pair) => pair,
        Err(_) => return Err(KernelRefusal::internal(KernelStage::Sew, "open_surgery_ends", "blend: open surgery needs exactly two ends")),
    };
    // A crossing that only resolved on a looser rung of the tolerance ladder
    // builds something master refused outright, so it must earn its answer
    // (see [`SpokeCrossings`]).
    let mut crossings = SpokeCrossings::default();
    crossings.note(start_end.escalated || finish_end.escalated);
    let mut result = solid.clone();
    let mut take_id = fresh_id_source(solid);
    let sewn = build_open_surgery(
        solid,
        &mut result,
        &mut take_id,
        edge,
        &first_mate,
        &second_mate,
        rows,
        [EndPlan::Free(start_end), EndPlan::Free(finish_end)],
        name,
    )?;
    prune_orphan_vertices(&mut result);
    // Extended end boundaries can leave a finite planar chart. Rebuild those
    // charts before validating the surgery, rather than rejecting a correct
    // edge because its projected pcurve was clamped to the old rectangle.
    crate::blend::fit_planar_charts_to_trims(solid, &mut result)?;
    check_snap_closure(solid, &result, sewn.snap)?;
    let built = crossings.gate(result)?;
    // The wall AS SEWN: the surgery may transpose the face (an exact
    // extrusion), and a report is matched to its face by exact surface.
    let sewn_surface = built
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == sewn.blend_face_id)
        .map(|face| face.surface.clone())
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "open_blend_face", "blend: the sewn open blend face is not in the built body"))?;
    // A wall that SHIPS unaccepted, or short of its construction request,
    // says so, typed, on the built body's wall (exact name and sewn surface).
    if let Some((reason, bar, measured, deficit)) = report {
        // The budget is the open march's GLOBAL station ladder (STATIONS
        // doubling to MAX_OPEN_STATIONS), not local rounds: for a request,
        // the rungs ATTEMPTED past the accepted one (a refused one included);
        // for an unaccepted rung, the rungs climbed past the first. Each rung
        // also ran its own local rail refinement, named in the detail.
        let rungs_used = if shipped_accepted { request_rungs } else { (reading.station_count / STATIONS).trailing_zeros() as usize };
        let mut detail = format!(
            "the shipped rung's local rail refinement ended {:?} with {} stations inserted",
            reading.refine_exit, reading.inserted
        );
        if !deficit.is_empty() {
            detail = format!("{deficit}; {detail}");
        }
        crate::blend::record_wall_model_report(crate::blend::WallModelReport {
            name: name.map(str::to_string),
            surface: sewn_surface,
            request: bar,
            residual: measured,
            detail: Some(detail),
            budget: crate::ApproximationBudget {
                reason,
                rounds_used: rungs_used,
                rounds_limit: OPEN_REQUEST_RUNGS,
                stations: reading.stations,
                station_limit: REFINE_STATION_FACTOR * (MAX_OPEN_STATIONS + 1),
                mechanism: crate::BudgetMechanism::StationRungs,
                measured_component: crate::MeasuredComponent::Wall,
                unread: 0,
            },
        });
    }
    Ok(built)
}

/// An UNACCEPTED shipped rung's deficit, typed: `Unread` when any reading
/// acceptance needs could not be taken (a non-finite rail, an empty shipped
/// window, an unread or missing wall interval), else `Unaccepted`; the
/// measured value and its bar are the readable component worst against its
/// bar (rails against `rail_bar`, the wall against `wall_bar`; 0 against
/// `rail_bar` when nothing could be read), always finite. The text names both
/// readings against their bars, after `what` (what failed acceptance: the
/// open ladder's every rung, or a network stripe's shipped windows).
/// `rails_read` is the caller's verdict on whether the rails were READ (the
/// open lane: [`OpenRungReading::rails_read`]; a network stripe also counts a
/// validated collapsed rail as read with no samples).
pub(in crate::blend) fn acceptance_deficit(
    reading: &OpenRungReading,
    rail_bar: f64,
    wall_bar: f64,
    wall_check: bool,
    rails_read: bool,
    what: &str,
) -> (crate::BudgetReason, f64, f64, String) {
    let rails = (rails_read && reading.rails.is_finite() && reading.rails >= 0.0).then_some(reading.rails);
    let wall = if wall_check { reading.worst_wall() } else { None };
    let unread = rails.is_none() || (wall_check && wall.is_none());
    let mut worst = (0.0_f64, rail_bar);
    for (value, bar) in [(rails, rail_bar), (wall, wall_bar)] {
        if let Some(value) = value {
            if value / bar > worst.0 / worst.1 {
                worst = (value, bar);
            }
        }
    }
    let rail_text = match rails {
        Some(value) => format!("rails {value:.3e} from their carriers (bar {rail_bar:.3e})"),
        None => format!(
            "rails UNREAD ({:?} over {} + {} samples) against bar {rail_bar:.3e}",
            reading.rails, reading.rail_samples[0], reading.rail_samples[1]
        ),
    };
    let wall_text = match (wall_check, wall) {
        (false, _) => "the wall not judged (no constant-radius fillet provenance)".to_string(),
        (true, Some(value)) => format!("wall {value:.3e} off the rolling ball (bar {wall_bar:.3e})"),
        (true, None) => format!(
            "wall UNREAD on {} of {} intervals against bar {wall_bar:.3e}{}",
            reading.walls.iter().filter(|wall| !matches!(wall, Some(value) if value.is_finite())).count(),
            reading.walls.len(),
            reading.first_unread.as_deref().map(|why| format!(" (first: {why})")).unwrap_or_default()
        ),
    };
    let reason = if unread { crate::BudgetReason::Unread } else { crate::BudgetReason::Unaccepted };
    (reason, worst.1, worst.0, format!("{what}: {rail_text}, {wall_text}"))
}

/// Section probes per open wall interval (k/8 of the section), and centres
/// per interval (probes at the even ones), as the chains read their walls.
const OPEN_WALL_SECTIONS: usize = 8;
pub(in crate::blend) const OPEN_WALL_CENTRES: usize = 32;
/// The rungs the construction request may climb past the first accepted one:
/// the open march's own ladder, STATIONS doubling to MAX_OPEN_STATIONS.
pub(in crate::blend) const OPEN_REQUEST_RUNGS: usize = (MAX_OPEN_STATIONS / STATIONS).trailing_zeros() as usize;

/// One open rung's measurement over what it SHIPS: its rung, its station
/// count (refinement included), the rails' worst distance from their
/// carriers over each rail's own crossing window and the samples read on
/// each (0: no window), each wall interval's declared reading (`None`:
/// unreadable) over the windows' envelope, and how the rung's own LOCAL rail
/// refinement ended (its exit and the stations it inserted).
#[derive(Clone, Debug)]
pub(in crate::blend) struct OpenRungReading {
    pub(in crate::blend) station_count: usize,
    pub(in crate::blend) stations: usize,
    pub(in crate::blend) rails: f64,
    pub(in crate::blend) rail_samples: [usize; 2],
    pub(in crate::blend) walls: Vec<Option<f64>>,
    /// The first unread wall interval and why (diagnostic; `None` when every
    /// interval read or the wall was not judged).
    pub(in crate::blend) first_unread: Option<String>,
    pub(in crate::blend) refine_exit: crate::blend::stations::StationRefineExit,
    pub(in crate::blend) inserted: usize,
}

impl OpenRungReading {
    /// Every rail READ: a finite, nonnegative worst distance from samples on
    /// both rails' shipped windows. A NaN, an infinity (an unreadable foot)
    /// or an empty window is never a reading.
    pub(in crate::blend) fn rails_read(&self) -> bool {
        self.rails.is_finite() && self.rails >= 0.0 && self.rail_samples.iter().all(|&samples| samples > 0)
    }

    /// Every wall interval READ (and at least one): `None` when any is not.
    pub(in crate::blend) fn worst_wall(&self) -> Option<f64> {
        if self.walls.is_empty() {
            return None;
        }
        self.walls.iter().try_fold(0.0_f64, |worst, wall| match wall {
            Some(declared) if declared.is_finite() && *declared >= 0.0 => Some(worst.max(*declared)),
            _ => None,
        })
    }
}

/// The shipped window `[low, high]` of one rail between its two crossing
/// parameters, within the rows' parameters (the surgery refuses a crossing
/// outside them; the reading only never indexes past them). `None` when the
/// window is empty or not finite: nothing ships there that could be read.
pub(in crate::blend) fn shipped_window(parameters: &[f64], a: f64, b: f64) -> Option<[f64; 2]> {
    let (&first, &last) = (parameters.first()?, parameters.last()?);
    let (low, high) = (a.min(b).max(first), a.max(b).min(last));
    (low.is_finite() && high.is_finite() && low < high).then_some([low, high])
}

/// The station interval holding `u` and `u`'s fraction of it.
fn interval_at(parameters: &[f64], u: f64) -> Option<(usize, f64)> {
    let last = parameters.len().checked_sub(2)?;
    let interval = parameters.partition_point(|&parameter| parameter <= u).saturating_sub(1).min(last);
    let (from, to) = (parameters[interval], parameters[interval + 1]);
    (to > from).then(|| (interval, ((u - from) / (to - from)).clamp(0.0, 1.0)))
}

/// Each shipped rail's worst distance from its carrier over ITS OWN window
/// ([`shipped_window`]): both window ends exactly and the quarter points of
/// every station span inside it, each against the carrier's EXTENSION
/// (`extended_foot_distance`, seeded by the stations' own contact uv) --
/// a finite planar chart clamps its projection, so retained overshoot past a
/// vertex would read a miss that is not there. `(worst, samples per rail)`;
/// any unreadable sample (a failed evaluation, an infinite foot) makes the
/// worst NaN, which [`OpenRungReading::rails_read`] never admits.
pub(in crate::blend) fn rails_on_shipped_windows(
    rows: &FittedRows,
    parameters: &[f64],
    stations: &[Station],
    carriers: [&NurbsSurface; 2],
    windows: [Option<[f64; 2]>; 2],
) -> (f64, [usize; 2]) {
    let mut worst = 0.0_f64;
    let mut samples = [0usize; 2];
    for (side, (rail, carrier)) in [(&rows.cr, carriers[0]), (&rows.cs, carriers[1])].into_iter().enumerate() {
        let Some([low, high]) = windows[side] else { continue };
        let mut at = vec![low, high];
        for pair in parameters.windows(2) {
            for fraction in [0.25, 0.5, 0.75] {
                let u = pair[0] + (pair[1] - pair[0]) * fraction;
                if u > low && u < high {
                    at.push(u);
                }
            }
        }
        for u in at {
            let off = interval_at(parameters, u)
                .and_then(|(interval, fraction)| {
                    let (left, right) = (&stations[interval], &stations[interval + 1]);
                    let (a, b) = if side == 0 { (left.uv1, right.uv1) } else { (left.uv2, right.uv2) };
                    let seed = [a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1])];
                    let point = rail.evaluate(u).ok()?;
                    crate::blend::stations::extended_foot_distance(carrier, point, seed).ok()
                })
                .filter(|off| off.is_finite())
                .unwrap_or(f64::NAN);
            samples[side] += 1;
            if !worst.is_nan() && (off.is_nan() || off > worst) {
                worst = off;
            }
        }
    }
    (worst, samples)
}

/// The JUNCTION stencil across the station between interval `left` and
/// `left + 1`: `left_nodes` (33 exact centres of `left`) from node 16 to 32,
/// then `right_nodes` (33 of `left + 1`) from node 1 to 16, the shared
/// station at node 16. Its native abscissae come from `station_ts` (the
/// edge parameters the centres were solved at): `None` when the two native
/// widths are bitwise equal -- uniform nodes, read by the ordinary reader --
/// else each node's t, interior nodes by the dense solves' own arithmetic
/// t_i + (t_{i+1} − t_i)·j/32 and the shared station at its STORED t. Err
/// when a station t is not finite or a width is not strictly positive.
pub(in crate::blend) fn junction_stencil(
    left_nodes: &[Vec3],
    right_nodes: &[Vec3],
    station_ts: &[f64],
    left: usize,
) -> Result<(Vec<Vec3>, Option<Vec<f64>>), String> {
    let half = OPEN_WALL_CENTRES / 2;
    if left_nodes.len() != OPEN_WALL_CENTRES + 1 || right_nodes.len() != OPEN_WALL_CENTRES + 1 || left + 2 >= station_ts.len() {
        return Err(format!("interval {left}: junction needs two {}-node intervals and their three station parameters", OPEN_WALL_CENTRES + 1));
    }
    let (t0, t1, t2) = (station_ts[left], station_ts[left + 1], station_ts[left + 2]);
    let (w_left, w_right) = (t1 - t0, t2 - t1);
    if !(t0.is_finite() && t1.is_finite() && t2.is_finite() && w_left > 0.0 && w_right > 0.0 && w_left.is_finite() && w_right.is_finite()) {
        return Err(format!("interval {left}: junction station parameters {t0}, {t1}, {t2} are not finite and strictly increasing"));
    }
    let nodes: Vec<Vec3> = left_nodes[half..].iter().chain(&right_nodes[1..=half]).copied().collect();
    if w_left.to_bits() == w_right.to_bits() {
        return Ok((nodes, None));
    }
    let abscissae: Vec<f64> = (half..=OPEN_WALL_CENTRES)
        .map(|j| if j == OPEN_WALL_CENTRES { t1 } else { t0 + w_left * j as f64 / OPEN_WALL_CENTRES as f64 })
        .chain((1..=half).map(|j| t1 + w_right * j as f64 / OPEN_WALL_CENTRES as f64))
        .collect();
    Ok((nodes, Some(abscissae)))
}

/// One wall probe on a stencil: the ordinary uniform reader when it has no
/// native abscissae (an interval's own nodes, or a junction of equal native
/// widths), the native-abscissa reader when it has them.
pub(in crate::blend) fn read_on_stencil(point: Vec3, nodes: &[Vec3], abscissae: Option<&[f64]>, step: usize, radius: f64) -> Result<(f64, f64), String> {
    match abscissae {
        None => crate::blend::chain::declared_wall_probe_why(point, nodes, step, radius),
        Some(ts) => crate::blend::chain::declared_wall_probe_at_why(point, nodes, ts, step, radius),
    }
}

/// Every interval's largest DECLARED wall reading (`chain::declared_wall_probe`,
/// reading + uncertainty) on the open fillet `rows`, against the exact ball
/// centres the march solved at j/32 of each interval ([`OpenDenseCentres`]),
/// or, for each unread interval, WHY: its station
/// interval and which component failed (no dense row, a dense row of the
/// wrong size, an exact centre the march did not solve, a wall point the
/// surface would not evaluate, a probe the declared reader declined, no probe
/// in the shipped part, or a non-finite reading). The cause is diagnostic.
///
/// Read on the DECLARED SHIPPED REGION `window` only: an interval it overlaps
/// is probed at its even stencil fractions that lie inside the window and at
/// each window end inside the interval; a probe outside the window is not on
/// the shipped wall and is not read. A window END whose nearest even stencil
/// node falls outside 2..=30 -- an end within a node of the interval's own
/// station, as the original bore's shipped spans were, 1e-16 from their
/// vertex stations -- has its foot at the interval's end node, where
/// [`crate::blend::chain::declared_wall_probe`] cannot read (the stencil
/// cannot follow a foot out of its own interval). It is read on a JUNCTION
/// stencil instead: the neighbouring interval's last half and this one's
/// first half (or this one's last half and the next one's first), the same
/// exact centres meeting at the shared station, with the probe at its
/// middle node. Two intervals of unequal native width (local insertions)
/// give the junction unequal node spacing, so it is then read by the
/// native-abscissa reader (`chain::declared_wall_probe_at_why`) on the
/// edge parameters the centres were solved at; of equal width, by the
/// ordinary reader. With no neighbour it is read as before.
pub(in crate::blend) fn open_wall_readings_why(
    rows: &FittedRows,
    parameters: &[f64],
    stations: &[Station],
    window: [f64; 2],
    dense: &OpenDenseCentres,
    radius: f64,
) -> Vec<Result<f64, String>> {
    let intervals = parameters.len().saturating_sub(1);
    let overlapped: Vec<usize> =
        (0..intervals).filter(|&interval| parameters[interval + 1] > window[0] && parameters[interval] < window[1]).collect();
    let Ok([v_low, v_high]) = rows.surface.domain_v() else {
        return overlapped.iter().map(|interval| Err(format!("interval {interval}: wall surface v domain unreadable"))).collect();
    };
    // The exact centres of one interval at j/32, its stations at both ends.
    let nodes_of = |interval: usize| -> Result<Vec<Vec3>, String> {
        let inner = dense.centres.get(interval).ok_or_else(|| format!("interval {interval}: no dense centre row"))?;
        if dense.subdivisions != OPEN_WALL_CENTRES || inner.len() + 1 != OPEN_WALL_CENTRES {
            return Err(format!("interval {interval}: dense row {} centres at {} subdivisions", inner.len(), dense.subdivisions));
        }
        let mut centres = Vec::with_capacity(OPEN_WALL_CENTRES + 1);
        centres.push(stations[interval].center);
        for (j, centre) in inner.iter().enumerate() {
            centres.push(centre.ok_or_else(|| format!("interval {interval}: exact centre {} of {OPEN_WALL_CENTRES} unsolved", j + 1))?);
        }
        centres.push(stations[interval + 1].center);
        Ok(centres)
    };
    let half = OPEN_WALL_CENTRES / 2;
    // The junction stencil across the station between `left` and `left + 1`:
    // left's nodes half..=32, then right's nodes 1..=half; the station is node
    // `half`. With each node's NATIVE abscissa (the edge parameter its exact
    // centre was solved at, `OpenDenseCentres::station_ts`), and `None` when
    // the two intervals' native widths are bitwise equal: the nodes are then
    // uniform and the ordinary reader reads them, bit for bit.
    let junction = |left: usize| -> Result<(Vec<Vec3>, Option<Vec<f64>>), String> {
        if dense.station_ts.len() != parameters.len() {
            return Err(format!("interval {left}: junction without the native station parameters"));
        }
        junction_stencil(&nodes_of(left)?, &nodes_of(left + 1)?, &dense.station_ts, left)
    };
    overlapped
        .into_iter()
        .map(|interval| -> Result<f64, String> {
            let centres = nodes_of(interval)?;
            let (from, to) = (parameters[interval], parameters[interval + 1]);
            let inside = |u: f64| u >= window[0] && u <= window[1];
            // (u, stencil node, the stencil: None = this interval's own;
            // a junction's nodes with its native abscissae when unequal).
            let mut probes: Vec<(f64, usize, Option<(Vec<Vec3>, Option<Vec<f64>>)>)> = (2..OPEN_WALL_CENTRES)
                .step_by(2)
                .map(|step| (from + (to - from) * step as f64 / OPEN_WALL_CENTRES as f64, step))
                .filter(|(u, _)| inside(*u))
                .map(|(u, step)| (u, step, None))
                .collect();
            for end in window {
                if end > from && end < to {
                    let fraction = (end - from) / (to - from);
                    let nearest = 2 * ((fraction * OPEN_WALL_CENTRES as f64 / 2.0).round() as usize);
                    #[cfg(not(test))]
                    let junction_off = false;
                    if junction_off {
                        probes.push((end, nearest.clamp(2, OPEN_WALL_CENTRES - 2), None));
                    } else if nearest < 2 && interval > 0 {
                        probes.push((end, half, Some(junction(interval - 1)?)));
                    } else if nearest > OPEN_WALL_CENTRES - 2 && interval + 1 < intervals {
                        probes.push((end, half, Some(junction(interval)?)));
                    } else {
                        probes.push((end, nearest.clamp(2, OPEN_WALL_CENTRES - 2), None));
                    }
                }
            }
            if probes.is_empty() {
                return Err(format!("interval {interval}: no probe in its shipped part of [{:.17}, {:.17}]", window[0], window[1]));
            }
            let mut declared = 0.0_f64;
            for (u, step, stencil) in probes {
                let (nodes, abscissae, what) = match &stencil {
                    Some((nodes, None)) => (nodes, None, "junction"),
                    Some((nodes, Some(ts))) => (nodes, Some(ts), "native-abscissa junction"),
                    None => (&centres, None, "interval"),
                };
                for k in 1..OPEN_WALL_SECTIONS {
                    let v = v_low + (v_high - v_low) * k as f64 / OPEN_WALL_SECTIONS as f64;
                    let point = rows.surface.evaluate(u, v).map_err(|error| format!("interval {interval}: wall point (u {u:.9}, v {v:.6}) unreadable: {error}"))?;
                    // The interval's own stencil reads from the probe's node,
                    // then (declined) from where its fraction reaches along the
                    // centre path (a stationary section); a junction keeps its
                    // own dispatch.
                    let read = if stencil.is_none() {
                        crate::blend::chain::declared_wall_probe_located_why(point, nodes, step, (u - from) / (to - from), radius)
                    } else {
                        read_on_stencil(point, nodes, abscissae.map(|ts| ts.as_slice()), step, radius)
                    };
                    let (reading, uncertainty) = read.map_err(|why| {
                        format!(
                            "interval {interval}: declared reader declined probe node {step} of the {what} stencil, section {k}/{OPEN_WALL_SECTIONS} \
                             (u {u:.17}): {why}"
                        )
                    })?;
                    declared = declared.max(reading + uncertainty);
                }
            }
            if declared.is_finite() { Ok(declared) } else { Err(format!("interval {interval}: non-finite reading {declared}")) }
        })
        .collect()
}


/// A fresh id allocator seeded past the vertex, edge, face, loop and coedge IDs.
/// Solid and shell IDs are outside this allocation domain. Shared by
/// a whole group of stripes so their vertices, edges and coedges cannot
/// collide.
pub(in crate::blend) fn fresh_id_source(solid: &BrepSolid) -> impl FnMut() -> u64 {
    let mut next_id = solid
        .vertices
        .iter()
        .map(|vertex| vertex.id)
        .chain(solid.edges.iter().map(|edge| edge.id))
        .chain(
            solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .flat_map(|face| {
                    face.loops
                        .iter()
                        .map(|loop_record| loop_record.id)
                        .chain(face.loops.iter().flat_map(|loop_record| {
                            loop_record.coedges.iter().map(|coedge| coedge.id)
                        }))
                        .chain(std::iter::once(face.id))
                }),
        )
        .max()
        .unwrap_or(0)
        + 1;
    move || {
        let id = next_id;
        next_id += 1;
        id
    }
}


/// May a declined support refit fall back on the march's own trim pcurve?
/// Only when that trim, read independently as shipped
/// (`read_shipped_pcurve`: miss, rail standoff, samples, worst), meets the
/// floor ON its rail's branch: its miss PLUS the rail's standoff from the
/// carrier at the trim's foot within `floor`. A trim on another branch of the
/// carrier tracks its own foot perfectly (miss 0) while that foot stands far
/// off the rail, so a miss alone would ship it; an unreadable trim never
/// stands.
pub(in crate::blend) fn march_trim_stands(read: &Result<(f64, f64, usize, f64), KernelRefusal>, floor: f64) -> bool {
    matches!(read, Ok((miss, standoff, _, _)) if *miss + *standoff <= floor)
}

/// Sew ONE stripe into `result`.
///
/// `solid` is the ORIGINAL topology every stripe was marched against (the
/// row-coincidence detection reads it); `result` is the shared evolving
/// solid, and `take_id` the shared id allocator — a group of stripes meeting
/// at a corner must create their rim vertices and cross-section arcs from one
/// counter, and must see each other's work.
///
/// Each end is either free (terminate on the face across the corner) or a
/// corner stop; see [`EndPlan`].  Pruning orphaned vertices is the caller's
/// job, once, after every stripe of the group is in.
pub(in crate::blend) fn build_open_surgery(
    solid: &BrepSolid,
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    rows: FittedRows,
    ends: [EndPlan; 2],
    name: Option<&str>,
) -> Result<SewnStripe, KernelRefusal> {
    let [start_end, finish_end] = ends;

    // Trim the support rows to the crossing window.
    // A POLE end stops EXACTLY on the row's own end (the march puts the
    // pole's station there and overshoots no further), so there is nothing to
    // split off on that side.  Only equality: a stop PAST the row's end — a
    // free end's crossing the rows never reached — still fails the split and
    // refuses, rather than building a rail out to wherever the rows stop.
    let trim_row = |row: &NurbsCurve, a: f64, b: f64| -> Result<NurbsCurve, KernelRefusal> {
        let [low, high] = row.domain().or_refuse(KernelStage::Refine, "domain")?;
        let tail = if a == low { row.clone() } else { row.split(a).or_refuse(KernelStage::Refine, "split")?.1 };
        if b == high {
            return Ok(tail);
        }
        let (middle, _) = tail.split(b).or_refuse(KernelStage::Refine, "split")?;
        Ok(middle)
    };
    let kind = |plan: &EndPlan| match plan {
        EndPlan::Free(_) => "free",
        EndPlan::Corner(_) => "corner",
        EndPlan::Miter(_) => "miter",
        EndPlan::Cap(_) => "cap",
    };
    let describe_trim = |error: KernelRefusal| {
        error.with_message(|error| {
            format!(
                "{error} (edge {} rows trimmed for a {} start at cr {:.6}/cs {:.6} and a {} finish \
                 at cr {:.6}/cs {:.6})",
                edge.id,
                kind(&start_end),
                start_end.cr_parameter(),
                start_end.cs_parameter(),
                kind(&finish_end),
                finish_end.cr_parameter(),
                finish_end.cs_parameter()
            )
        })
    };
    // A rail whose two stops are one point within `consumed_band` has no strip
    // to trim: the blend takes that mate down to a single point along this
    // edge.  That is a FULL-WIDTH corner — the shared face of a miter whose
    // radius is the side face's width, every face of a star at full width, the
    // middle edge of a channel whose two corner setbacks meet.  No rail edge is
    // built; the mate loses the blended edge instead of having it replaced, and
    // the rail's two rim vertices are one vertex, which the orchestrator
    // identifies (`SewnStripe::identified`) once every stripe is in.  A stripe
    // free at BOTH ends has no corner to collapse onto, so it refuses.
    let band = consumed_band(solid);
    let collapses = |row: &NurbsCurve, a: f64, b: f64| -> Result<bool, KernelRefusal> {
        let from = row.evaluate(a).or_refuse(KernelStage::Refine, "evaluate")?;
        Ok(row.evaluate(b).or_refuse(KernelStage::Refine, "evaluate")?.sub(from).length() <= band
            && row.evaluate(0.5 * (a + b)).or_refuse(KernelStage::Refine, "evaluate")?.sub(from).length() <= band)
    };
    let cr_collapsed = collapses(&rows.cr, start_end.cr_parameter(), finish_end.cr_parameter())?;
    let cs_collapsed = collapses(&rows.cs, start_end.cs_parameter(), finish_end.cs_parameter())?;
    if cr_collapsed || cs_collapsed {
        let unsupported = if start_end.free().is_some() && finish_end.free().is_some() {
            Some("neither end is a corner")
        } else if first.face.id == second.face.id {
            Some("both rails lie on one face")
        } else if matches!(start_end, EndPlan::Cap(_)) || matches!(finish_end, EndPlan::Cap(_)) {
            Some("a capped end has no corner to share the point with")
        } else {
            None
        };
        if let Some(reason) = unsupported {
            return Err(KernelRefusal::unsupported(KernelStage::Sew, super::RAIL_COLLAPSE_WHAT, format!(
                "{RAIL_COLLAPSE_UNSUPPORTED} edge {}'s rail shrinks to a point and {reason}",
                edge.id
            )));
        }
    }
    // The support pcurve the rows carry interpolates the stations' (u, v) and
    // is read nowhere between them: on the 20-degree crossing's notched open
    // exit arc it stood 2.98e-5 off the rail it trims (q1 = 9cd8c3163 + the
    // section-fit fix, read against the exact cylinders). Inside the crossing
    // window — on the face, never on a carrier's extension — it is refitted
    // to the trimmed rail's own projected track at the pcurve floor and read
    // again as shipped (`refit_closed_support_pcurve`, the closed edge's).
    // A rail that overruns a PLANE carrier's chart (the rib-base fin top's
    // rails run 0.303 past it) is on the plane's extension, which the chart
    // clamps: there the refit and both reads would measure the clamp, and the
    // trim is instead rebuilt from its edge on the widened chart and verified
    // at the floor by `fit_planar_charts_to_trims`, which every lane that
    // sews these stripes runs.
    let trim_rail = |collapsed: bool,
                     row: &NurbsCurve,
                     pcurve: &NurbsCurve,
                     surface: &NurbsSurface,
                     a: f64,
                     b: f64|
     -> Result<Option<(NurbsCurve, NurbsCurve)>, KernelRefusal> {
        if collapsed {
            return Ok(None);
        }
        let rail = trim_row(row, a, b).map_err(describe_trim)?;
        let interpolated = trim_row(pcurve, a, b)?;
        {
            let [d0, d1] = rail.domain().or_refuse(KernelStage::Refine, "domain")?;
            if crate::blend::planar_chart_overrun(surface, &|t| rail.evaluate(t), d0, d1)?.is_some_and(|excursion| excursion > band) {
                return Ok(Some((rail, interpolated)));
            }
        }
        // The march's OWN trim pcurve, read independently at the rail's own
        // parameter against the same floor (`read_shipped_pcurve`): what a
        // declined refit may fall back to, and the standoff a refit on the
        // rail's own branch must match.
        let [d0, d1] = rail.domain().or_refuse(KernelStage::Refine, "domain")?;
        let march_read = crate::blend::track_fit::read_shipped_pcurve(surface, &|t| rail.evaluate(t), &interpolated, d0, d1);
        let floor = crate::pcurve::PCURVE_REFINEMENT_TOLERANCE;
        let declined = |why: KernelRefusal| -> Result<Option<(NurbsCurve, NurbsCurve)>, KernelRefusal> {
            // Kept ONLY when the march's trim stands on its own read
            // (`march_trim_stands`); otherwise the refusal stands.
            let stands = march_trim_stands(&march_read, floor);
            if stands {
                Ok(Some((rail.clone(), interpolated.clone())))
            } else {
                Err(why)
            }
        };
        let refit = super::closed::refit_closed_support_pcurve(&rail, &interpolated, surface);
        match refit {
            Ok(fitted) => {
                // A refit tracks ITS OWN projected track; a track that took
                // another branch of the carrier tracks itself perfectly while
                // its foot stands far off the rail (the stationary-start
                // prism's coedge read 6.0 off its edge). Its foot standoff
                // must be the rail's own standoff on the march trim's branch.
                // BOTH reads are required: an unreadable refit read declines
                // the refit (the march trim then stands only on its own read
                // within the floor), and an unreadable march read cannot
                // establish that the refit is on the rail's branch, so it
                // refuses. Never shipped on an unknown.
                // The two are compared POINTWISE, at the same rail
                // parameters (`standoff_excess`), never as two maxima read at
                // each curve's own knots.
                let refit_read = crate::blend::track_fit::read_shipped_pcurve(surface, &|t| rail.evaluate(t), &fitted, d0, d1);
                let (march_standoff, refit_standoff) = match (&march_read, refit_read) {
                    (_, Err(error)) => return declined(error),
                    (Err(error), Ok(_)) => {
                        return Err(KernelRefusal::non_convergence(KernelStage::Refine, "support_refit_branch_unread", format!(
                            "blend: a support trim's refit cannot be shown to stand on its rail's branch: the march's own \
                             trim is unreadable ({})",
                            error.message
                        )));
                    }
                    (Ok((_, march_standoff, _, _)), Ok((_, refit_standoff, _, _))) => (*march_standoff, refit_standoff),
                };
                let excess = match crate::blend::track_fit::standoff_excess(surface, &|t| rail.evaluate(t), &interpolated, &fitted, d0, d1) {
                    Ok(excess) => excess,
                    Err(error) => return declined(error),
                };
                if !(excess <= floor) {
                    return declined(KernelRefusal::non_convergence(KernelStage::Refine, "support_refit_branch", format!(
                        "blend: a support trim's refit stands up to {excess:.3e} farther off its rail than the march's own trim \
                         at the same rail parameter (largest standoffs {refit_standoff:.3e} and {march_standoff:.3e}): it took \
                         another branch of the carrier"
                    )));
                }
                // ABSOLUTE: the comparison above is relative to the march
                // trim, so a refit that follows an off-branch march trim
                // passes it. The refit's foot must also stand within the floor
                // of the rail's own deviation from its carrier, read by the
                // nearest-point projector at the same parameters
                // (`standoff_over_nearest`) -- the rail's approximation the
                // lane reports, never more.
                let absolute = match crate::blend::track_fit::standoff_over_nearest(surface, &|t| rail.evaluate(t), &fitted, d0, d1) {
                    Ok(absolute) => absolute,
                    Err(error) => return declined(error),
                };
                if !(absolute <= floor) {
                    return declined(KernelRefusal::non_convergence(KernelStage::Refine, "support_refit_off_rail", format!(
                        "blend: a support trim's refit stands up to {absolute:.3e} farther off its rail than the rail stands \
                         off its carrier: its foot is not the rail's nearest point"
                    )));
                }
                Ok(Some((rail, fitted)))
            }
            Err(error) => declined(error),
        }
    };
    let cr_rail = trim_rail(
        cr_collapsed,
        &rows.cr,
        &rows.cr_pcurve,
        &first.face.surface,
        start_end.cr_parameter(),
        finish_end.cr_parameter(),
    )?;
    let cs_rail = trim_rail(
        cs_collapsed,
        &rows.cs,
        &rows.cs_pcurve,
        &second.face.surface,
        start_end.cs_parameter(),
        finish_end.cs_parameter(),
    )?;
    // A collapsed rail's domain is its two stops, which are one point.
    let cr_domain = match &cr_rail {
        Some((cr, _)) => cr.domain().or_refuse(KernelStage::Refine, "domain")?,
        None => [start_end.cr_parameter(), finish_end.cr_parameter()],
    };
    let cs_domain = match &cs_rail {
        Some((cs, _)) => cs.domain().or_refuse(KernelStage::Refine, "domain")?,
        None => [start_end.cs_parameter(), finish_end.cs_parameter()],
    };
    // A rail's end points, read off the trimmed rail or, collapsed, the row.
    let cr_at = |parameter: f64| match &cr_rail {
        Some((cr, _)) => cr.evaluate(parameter),
        None => rows.cr.evaluate(parameter),
    };
    let cs_at = |parameter: f64| match &cs_rail {
        Some((cs, _)) => cs.evaluate(parameter),
        None => rows.cs.evaluate(parameter),
    };
    let old_start = edge.start_vertex_id;
    let old_finish = edge.end_vertex_id;

    // Classify each of the four support crossings.  A crossing that lands at
    // the FAR endpoint of the boundary edge it meets (the end away from the
    // corner) means a prior fillet already consumed that whole boundary: the
    // new blend reuses the existing vertex there and the boundary edge is
    // deleted (its role is taken over by the new transverse curve on the
    // prior blend face).  An interior crossing is the pristine case — a fresh
    // vertex with the boundary trimmed to it.
    //
    // Consumed means within `consumed_band` — the slack `check_support_extent`
    // refuses past, so a rail between the two is not possible.
    let classify =
        |boundary_id: u64, boundary_param: f64, corner: u64| -> Result<RimResolution, KernelRefusal> {
            let boundary = result
                .edges
                .iter()
                .find(|candidate| candidate.id == boundary_id)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "end_boundary_edge", "blend: end boundary edge missing during surgery"))?;
            let span = (boundary.t1 - boundary.t0).abs().max(1e-12);
            let far_vertex = if boundary.start_vertex_id == corner {
                Some((boundary.end_vertex_id, boundary.t1, boundary.t0))
            } else if boundary.end_vertex_id == corner {
                Some((boundary.start_vertex_id, boundary.t0, boundary.t1))
            } else {
                None
            };
            if let Some((far_id, far_t, near_t)) = far_vertex {
                // A consumed boundary is a COINCIDENCE — a prior fillet's rim
                // vertex is exactly where this rail crosses — and the operands
                // were healed before the march, so it holds to solver
                // precision.  It is not "near the far end": a fillet whose
                // radius is 1e-3 short of the face's width crosses 1e-3 from
                // the far vertex and must keep that sliver, or the face is
                // snapped a full 1e-3 out of true (the oversized-radius limit
                // cases and issue 1177 measure exactly that).  Measured in
                // model units, never in the boundary's parameter — a unit
                // parameter on a 20-long edge would call 2e-3 a coincidence.
                let far_point = result
                    .vertices
                    .iter()
                    .find(|candidate| candidate.id == far_id)
                    .map(|candidate| candidate.point)
                    .ok_or(KernelRefusal::internal(KernelStage::Sew, "far_vertex", "blend: consumed boundary far vertex missing"))?;
                let crossing = boundary.curve.evaluate_extended(boundary_param).or_refuse(KernelStage::Refine, "evaluate_extended")?;
                let _ = span;
                let past_far = crossing.sub(far_point).length();
                if past_far <= band {
                    // Sanity: the reused vertex must exist.
                    if !result
                        .vertices
                        .iter()
                        .any(|candidate| candidate.id == far_id)
                    {
                        return Err(KernelRefusal::internal(KernelStage::Sew, "far_vertex", "blend: consumed boundary far vertex missing"));
                    }
                    return Ok(RimResolution::Consumed(far_id));
                }
                // A crossing BEYOND the far vertex is a rail that runs off
                // its face: the blend is wider than the face by more than the
                // band a snap may close, and trimming the boundary there would
                // extend it into air.  `check_support_extent` refuses this
                // before the march on every face the exact tool understands;
                // this is the refusal for the rest.  It is not the whole
                // answer: a crossing the solve CLAMPS to the far vertex reads
                // as consumed here, and `detect_row_coincidence` is what sees
                // that row leave the face.  Measured on the census: it fires on
                // a wall wider than its closed chain's arc (0.5 past) and on the
                // r = 10.001 pinch past an earlier round (2e-3 past), both of
                // which refused later in the surgery before.
                if (boundary_param - far_t) * (far_t - near_t) > 0.0 {
                    return Err(KernelRefusal::unsupported(KernelStage::Sew, "blend_wider_than_face", format!(
                        "{BLEND_WIDER_THAN_FACE} its rail crosses edge {boundary_id} {past_far:.3e} \
                         past that edge's far vertex, and a rail within {band:.3e} of it is the \
                         most this surgery snaps onto the edge"
                    )));
                }
            }
            Ok(RimResolution::Fresh)
        };
    // A CORNER end has no boundary to classify: the ball's tangency vertex is
    // the rim, and the boundary edge that used to reach the sharp corner is
    // the neighbouring stripe's own blended edge, which that stripe replaces.
    let resolve_side = |plan: &EndPlan,
                        side_first: bool,
                        corner: u64|
     -> Result<RimResolution, KernelRefusal> {
        if let Some(vertex) = plan.planned_rim(side_first) {
            return Ok(RimResolution::Corner(vertex));
        }
        match plan {
            EndPlan::Free(free) => classify(
                if side_first {
                    free.first_edge_id
                } else {
                    free.second_edge_id
                },
                if side_first {
                    free.first_edge_parameter
                } else {
                    free.second_edge_parameter
                },
                corner,
            ),
            EndPlan::Corner(_) | EndPlan::Miter(_) | EndPlan::Cap(_) => {
                Err(KernelRefusal::internal(KernelStage::Sew, "rim_vertices", "blend: planned end without rim vertices"))
            }
        }
    };
    let start_first = resolve_side(&start_end, true, old_start)?;
    let start_second = resolve_side(&start_end, false, old_start)?;
    let finish_first = resolve_side(&finish_end, true, old_finish)?;
    let finish_second = resolve_side(&finish_end, false, old_finish)?;

    // Manifold pairing: the blend's use of the first support row must oppose
    // F1's use of the blended edge (see the blend-loop construction below).
    let first_use_forward = first.coedge.forward;
    let blend_cr_forward = !first_use_forward;

    // Curve-level coincidence (OCCT PR #1449 lesson 3): when BOTH of a mate's
    // crossings are endpoint-consumed AND the trimmed support row retraces the
    // existing edge joining the two far vertices, that mate face is FULLY
    // consumed — the surgery sews the blend face straight onto the existing
    // edge and drops the zero-width face, instead of creating a coincident
    // fresh support edge that would leave a sliver strip (the r = face-width
    // class).  Each side is detected independently; any ambiguity keeps the
    // pristine path (fail-safe).  `row_traversed_from_start`: the blend loop
    // walks cr in station order iff `blend_cr_forward`, and cs in the
    // OPPOSITE order (see the two loop branches below).
    // A stripe with a CORNER end has no boundary edges at that end to be
    // coincident with, so the detection only runs on stripes that are free at
    // both ends.  (Fail-safe: not detecting a coincidence keeps the pristine
    // path, which is what a corner stop wants anyway.)
    let (sew_first, sew_second) = match (start_end.free(), finish_end.free(), &cr_rail, &cs_rail) {
        (Some(start_free), Some(finish_free), Some((cr, _)), Some((cs, _))) => (
            detect_row_coincidence(
                solid,
                first,
                edge.id,
                start_free.first_edge_id,
                finish_free.first_edge_id,
                start_first.is_consumed(),
                start_first.existing().unwrap_or(0),
                finish_first.is_consumed(),
                finish_first.existing().unwrap_or(0),
                cr,
                blend_cr_forward,
                band,
            )?,
            detect_row_coincidence(
                solid,
                second,
                edge.id,
                start_free.second_edge_id,
                finish_free.second_edge_id,
                start_second.is_consumed(),
                start_second.existing().unwrap_or(0),
                finish_second.is_consumed(),
                finish_second.existing().unwrap_or(0),
                cs,
                !blend_cr_forward,
                band,
            )?,
        ),
        _ => (None, None),
    };
    // One face hosting both sides (the blended edge used twice by one face)
    // cannot be dropped for one side while the other still splices into it —
    // ambiguous, keep the pristine path for both.
    let (sew_first, sew_second): (Option<RowSew>, Option<RowSew>) =
        if first.face.id == second.face.id {
            (None, None)
        } else {
            (sew_first, sew_second)
        };

    // Resolve the four rim vertices: reuse the existing vertex for a consumed
    // side, else create a fresh vertex at the support-row endpoint.
    let mut resolve = |rim: &RimResolution, point: Result<Vec3, String>| -> Result<u64, KernelRefusal> {
        match rim.existing() {
            Some(id) => Ok(id),
            None => {
                let id = take_id();
                result.vertices.push(VertexRecord { id, point: point.or_refuse(KernelStage::Refine, "point")? });
                Ok(id)
            }
        }
    };
    let w1a = resolve(&start_first, cr_at(cr_domain[0]))?;
    let w1b = resolve(&finish_first, cr_at(cr_domain[1]))?;
    let w2a = resolve(&start_second, cs_at(cs_domain[0]))?;
    let w2b = resolve(&finish_second, cs_at(cs_domain[1]))?;
    // How far the consumed rims and sewn rows moved a rail onto what already
    // existed — what `check_snap_closure` measures the body against.
    let mut snap = [&sew_first, &sew_second]
        .into_iter()
        .flatten()
        .fold(0.0f64, |worst, sew| worst.max(sew.deviation));
    for (rim, vertex, row_end) in [
        (&start_first, w1a, cr_at(cr_domain[0]).or_refuse(KernelStage::Refine, "cr_at")?),
        (&finish_first, w1b, cr_at(cr_domain[1]).or_refuse(KernelStage::Refine, "cr_at")?),
        (&start_second, w2a, cs_at(cs_domain[0]).or_refuse(KernelStage::Refine, "cs_at")?),
        (&finish_second, w2b, cs_at(cs_domain[1]).or_refuse(KernelStage::Refine, "cs_at")?),
    ] {
        if !rim.is_consumed() {
            continue;
        }
        let point = result
            .vertices
            .iter()
            .find(|candidate| candidate.id == vertex)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "rim_vertex", "blend: consumed rim vertex missing"))?
            .point;
        snap = snap.max(row_end.sub(point).length());
    }

    // Support edges: a sewn side reuses the EXISTING coincident edge (no
    // fresh edge — OCCT's `SetExistingEdge` move); a pristine side gets the
    // fitted row as a fresh edge.
    // A collapsed side builds no edge at all.
    let cr_edge_id = match (&sew_first, &cr_rail) {
        (Some(sew), _) => Some(sew.edge_id),
        (None, None) => None,
        (None, Some((cr, _))) => {
            let id = take_id();
            result.edges.push(EdgeRecord {
                id,
                curve: cr.clone(),
                t0: cr_domain[0],
                t1: cr_domain[1],
                start_vertex_id: w1a,
                end_vertex_id: w1b,
                degenerate: false,
                name: None,
            });
            Some(id)
        }
    };
    let cs_edge_id = match (&sew_second, &cs_rail) {
        (Some(sew), _) => Some(sew.edge_id),
        (None, None) => None,
        (None, Some((cs, _))) => {
            let id = take_id();
            result.edges.push(EdgeRecord {
                id,
                curve: cs.clone(),
                t0: cs_domain[0],
                t1: cs_domain[1],
                start_vertex_id: w2a,
                end_vertex_id: w2b,
                degenerate: false,
                name: None,
            });
            Some(id)
        }
    };
    // The edge closing each end of the blend face: the §6.9 transverse curve
    // on the end face for a free end, or — at a corner — the end
    // cross-section arc the corner patch was already given, committed by the
    // caller and merely referenced here.
    let mut commit_end_edge = |plan: &EndPlan,
                               first_rim: u64,
                               second_rim: u64|
     -> Result<Option<u64>, KernelRefusal> {
        match plan {
            EndPlan::Corner(_) | EndPlan::Miter(_) | EndPlan::Cap(_) => Ok(None),
            EndPlan::Free(free) => {
                let id = take_id();
                let domain = free.transverse_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
                result.edges.push(EdgeRecord {
                    id,
                    curve: free.transverse_curve.clone(),
                    t0: domain[0],
                    t1: domain[1],
                    start_vertex_id: first_rim,
                    end_vertex_id: second_rim,
                    degenerate: false,
                    name: None,
                });
                Ok(Some(id))
            }
        }
    };
    let transverse_a_id = commit_end_edge(&start_end, w1a, w2a)?;
    let transverse_b_id = commit_end_edge(&finish_end, w1b, w2b)?;

    // Replace the blended edge in each PRISTINE mate's loop — the blend
    // face's loop direction is forced by manifold pairing with F1's use of
    // the blended edge (`first_use_forward`, captured above).  A sewn mate is
    // dropped whole below; nothing to splice.  A COLLAPSED side has no rail to
    // put there: the mate simply loses the blended edge, and its loop closes
    // through the rim vertices the orchestrator identifies.
    for (mate, rail, sewn) in [
        (first, cr_edge_id.zip(cr_rail.as_ref()), sew_first.is_some()),
        (second, cs_edge_id.zip(cs_rail.as_ref()), sew_second.is_some()),
    ] {
        if sewn {
            continue;
        }
        let face = result
            .shells
            .iter_mut()
            .flat_map(|shell| &mut shell.faces)
            .find(|face| face.id == mate.face.id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during surgery"))?;
        let loop_record = &mut face.loops[mate.loop_index];
        let position = loop_record
            .coedges
            .iter()
            .position(|coedge| coedge.edge_id == edge.id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "edge_coedge", "blend: edge coedge lost during surgery"))?;
        let Some((new_edge_id, (_, pcurve_forward))) = rail else {
            loop_record.coedges.remove(position);
            continue;
        };
        let old_forward = loop_record.coedges[position].forward;
        loop_record.coedges[position] = CoedgeRecord {
            id: loop_record.coedges[position].id,
            edge_id: new_edge_id,
            forward: old_forward,
            pcurve: if old_forward {
                pcurve_forward.clone()
            } else {
                pcurve_forward.reversed().or_refuse(KernelStage::Refine, "reversed")?
            },
        };
    }

    // A capped end's legs: on each mate, the leg runs from the rail's rim
    // vertex to the sharp vertex, where the continuing edge still starts.
    // It goes between the rail coedge and that continuing coedge, with the
    // sense that walks rim -> vertex when the rail ends at the rim.
    for (plan, at_start) in [(&start_end, true), (&finish_end, false)] {
        let EndPlan::Cap(cap) = plan else {
            continue;
        };
        for (mate, rail_edge_id, leg, rim) in [
            (first, cr_edge_id, &cap.first_leg, if at_start { w1a } else { w1b }),
            (second, cs_edge_id, &cap.second_leg, if at_start { w2a } else { w2b }),
        ] {
            // Refused above: a capped end never meets a collapsed rail.
            let rail_edge_id = rail_edge_id.ok_or(KernelRefusal::internal(KernelStage::Sew, "end_rail", "blend: a capped end lost its rail"))?;
            let face = result
                .shells
                .iter_mut()
                .flat_map(|shell| &mut shell.faces)
                .find(|face| face.id == mate.face.id)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during cap splice"))?;
            let loop_record = &mut face.loops[mate.loop_index];
            let count = loop_record.coedges.len();
            let rail_at = loop_record
                .coedges
                .iter()
                .position(|coedge| coedge.edge_id == rail_edge_id)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "rail_coedge", "blend: rail coedge lost during cap splice"))?;
            // Which side of the rail coedge is this end?  The rail coedge
            // traverses rim-to-rim; the capped end is at its traversal END
            // when walking forward means station order and this is the
            // finish end, etc.  Decide by geometry: the neighbour whose
            // traversal touches the sharp vertex.
            let rail_forward = loop_record.coedges[rail_at].forward;
            let end_is_traversal_end = if rail_forward { !at_start } else { at_start };
            let (insert_at, forward, pcurve) = if end_is_traversal_end {
                // rail ... rim -> [leg rim->vertex] -> continuing edge
                (rail_at + 1, true, leg.1.clone())
            } else {
                // continuing edge -> [leg vertex->rim] -> rim ... rail
                (rail_at, false, leg.1.reversed().or_refuse(KernelStage::Refine, "reversed")?)
            };
            let _ = (count, rim);
            loop_record.coedges.insert(
                insert_at,
                CoedgeRecord {
                    id: take_id(),
                    edge_id: leg.0,
                    forward,
                    pcurve,
                },
            );
        }
    }

    // Drop consumed boundary edges from the mate loop they share with the
    // blended edge: the fresh support coedge already begins at the reused far
    // vertex, so removing the fully-covered boundary keeps the loop closed.
    // A SEWN mate's whole loop collapses (the face is dropped below), so its
    // consumed boundaries are only recorded for deletion, never spliced.
    let mut consumed_edges: Vec<u64> = Vec::new();
    let start_free = start_end.free();
    let finish_free = finish_end.free();
    for (mate, sewn, sides) in [
        (
            first,
            sew_first.is_some(),
            [
                (&start_first, start_free.map(|free| free.first_edge_id)),
                (&finish_first, finish_free.map(|free| free.first_edge_id)),
            ],
        ),
        (
            second,
            sew_second.is_some(),
            [
                (&start_second, start_free.map(|free| free.second_edge_id)),
                (&finish_second, finish_free.map(|free| free.second_edge_id)),
            ],
        ),
    ] {
        for (cross, boundary_id) in sides {
            if !cross.is_consumed() {
                continue;
            }
            let Some(boundary_id) = boundary_id else {
                continue;
            };
            if !sewn {
                let face = result
                    .shells
                    .iter_mut()
                    .flat_map(|shell| &mut shell.faces)
                    .find(|face| face.id == mate.face.id)
                    .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during surgery"))?;
                face.loops[mate.loop_index]
                    .coedges
                    .retain(|coedge| coedge.edge_id != boundary_id);
            }
            if !consumed_edges.contains(&boundary_id) {
                consumed_edges.push(boundary_id);
            }
        }
    }
    // Lesson 7: zero-span leftovers of a collapsing loop go with their face.
    for sew in [&sew_first, &sew_second].into_iter().flatten() {
        for id in &sew.collapsed_edges {
            if !consumed_edges.contains(id) {
                consumed_edges.push(*id);
            }
        }
    }

    // Locate the corner junction on each end face BEFORE trimming (the two
    // boundary coedges that meet at the old corner vertex, stepping over the
    // pole between them where the end face's apex IS that vertex) — edge ids
    // alone are ambiguous on two-coedge cap loops, so the meeting must be at
    // the corner.
    // (end, transverse_id, corner, first_consumed, second_consumed) -> plan.
    // Only FREE ends appear here: a corner stop rebuilds nothing on a third
    // face, because the corner patch is what closes it.
    let mut end_plans = Vec::with_capacity(2);
    for (end, transverse_id, corner, first_consumed, second_consumed) in [
        (
            start_end.free(),
            transverse_a_id,
            old_start,
            start_first.is_consumed(),
            start_second.is_consumed(),
        ),
        (
            finish_end.free(),
            transverse_b_id,
            old_finish,
            finish_first.is_consumed(),
            finish_second.is_consumed(),
        ),
    ] {
        let (Some(end), Some(transverse_id)) = (end, transverse_id) else {
            continue;
        };
        let face = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == end.end_face_id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "end_face", "blend: end face lost during surgery"))?;
        // Key on stable coedge ids (not indices) so processing one end can't
        // invalidate another that shares this face.
        let Some(located) = locate_end_corner(
            &result,
            face,
            corner,
            end.first_edge_id,
            end.second_edge_id,
        ) else {
            return Err(KernelRefusal::internal(KernelStage::Sew, "end_face_corner", "blend: end-face corner (adjacent boundary coedges) not found"));
        };
        // The apex the transverse curve cuts off goes with the boundaries the
        // blend consumes.
        for pole in &located.pole_edge_ids {
            if !consumed_edges.contains(pole) {
                consumed_edges.push(*pole);
            }
        }
        end_plans.push((
            end.end_face_id,
            transverse_id,
            end.transverse_end_pcurve.clone(),
            located,
            first_consumed,
            second_consumed,
        ));
    }

    // Trim the boundary edges at their support crossings.  A consumed side is
    // deleted instead of trimmed; a CORNER side touches no boundary at all.
    for (rim_resolution, boundary, corner, rim) in [
        (
            &start_first,
            start_free.map(|free| (free.first_edge_id, free.first_edge_parameter)),
            old_start,
            w1a,
        ),
        (
            &start_second,
            start_free.map(|free| (free.second_edge_id, free.second_edge_parameter)),
            old_start,
            w2a,
        ),
        (
            &finish_first,
            finish_free.map(|free| (free.first_edge_id, free.first_edge_parameter)),
            old_finish,
            w1b,
        ),
        (
            &finish_second,
            finish_free.map(|free| (free.second_edge_id, free.second_edge_parameter)),
            old_finish,
            w2b,
        ),
    ] {
        if !rim_resolution.trims_boundary() {
            continue;
        }
        let Some((boundary_id, boundary_param)) = boundary else {
            continue;
        };
        trim_edge_at(result, boundary_id, boundary_param, corner, rim)?;
    }

    // Rebuild each end face's corner: drop the consumed boundary coedge(s) and
    // splice in the transverse coedge between the (trimmed) boundaries.
    for (end_face, transverse_id, transverse_end_pcurve, located, first_consumed, second_consumed) in
        end_plans
    {
        let forward = located.x_is_first;
        let pcurve = if forward {
            transverse_end_pcurve
        } else {
            transverse_end_pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?
        };
        let (x_consumed, y_consumed) = if located.x_is_first {
            (first_consumed, second_consumed)
        } else {
            (second_consumed, first_consumed)
        };
        let transverse_coedge = CoedgeRecord {
            id: take_id(),
            edge_id: transverse_id,
            forward,
            pcurve,
        };
        splice_end_corner(
            result,
            end_face,
            &located,
            transverse_coedge,
            x_consumed,
            y_consumed,
        )?;
    }

    // Blend face: cr fwd -> TB -> cs rev -> TA rev in (t, z) space.  Both rows
    // share one parameterisation, so a collapsed cr reads its sense across the
    // strip the cs rail still spans (a stripe with BOTH rails collapsed has no
    // area; the orchestrator drops it with the edges that bound it).
    let (mid_u, station_uv) = match (&cr_rail, &cs_rail) {
        (Some((_, cr_pcurve)), _) => {
            let mid_u = (cr_domain[0] + cr_domain[1]) * 0.5;
            (mid_u, cr_pcurve.evaluate(mid_u).or_refuse(KernelStage::Refine, "evaluate")?)
        }
        (None, Some(_)) => {
            let mid_u = (cs_domain[0] + cs_domain[1]) * 0.5;
            (mid_u, rows.cr_pcurve.evaluate(mid_u).or_refuse(KernelStage::Refine, "evaluate")?)
        }
        (None, None) => (cr_domain[0], rows.cr_pcurve.evaluate(cr_domain[0]).or_refuse(KernelStage::Refine, "evaluate")?),
    };
    let blend_normal = raw_normal(&rows.surface, mid_u, 0.0)?;
    let n1 = raw_normal(&first.face.surface, station_uv.x, station_uv.y)?;
    let out1 = if first.face.same_sense {
        n1
    } else {
        n1.scale(-1.0)
    };
    let same_sense = blend_normal.dot(out1) >= 0.0;
    let loop_id = take_id();
    // Traversal senses of the blend's support coedges.  A fresh support edge
    // is parameterized in station order, so the sense is the loop's walking
    // direction (`blend_cr_forward` for cr, its opposite for cs); a SEWN
    // side's sense comes from the detection (the existing edge's own
    // parameter direction relative to that same walk — equal to the dropped
    // face's old sense, preserving manifold pairing with the survivor).  The
    // pcurves always follow the LOOP's walking direction and are unchanged.
    let cr_forward = sew_first
        .as_ref()
        .map_or(blend_cr_forward, |sew| sew.forward);
    let cs_forward = sew_second
        .as_ref()
        .map_or(!blend_cr_forward, |sew| sew.forward);
    // The two end slots of the blend loop.  `natural_forward` is the sense the
    // loop walks a transverse edge stored first-rim -> second-rim; a CORNER
    // arc was already committed in the walk direction (so the corner patch can
    // take it the other way round), so it is always used forward.
    // A slot is one coedge for a free end (the transverse curve) or a corner
    // end (the section arc), and one or two for a miter end (the seam, then
    // the sibling's section when the seam left the sibling first).  A corner
    // arc was committed in the walk direction; a miter edge records its own
    // sense along the walk.
    let mut end_slot = |plan: &EndPlan,
                        free_edge_id: Option<u64>,
                        natural_forward: bool|
     -> Result<Vec<CoedgeRecord>, KernelRefusal> {
        match plan {
            EndPlan::Corner(end) => Ok(vec![CoedgeRecord {
                id: take_id(),
                edge_id: end.arc_edge_id,
                forward: true,
                pcurve: end.arc_blend_pcurve.clone(),
            }]),
            EndPlan::Cap(end) => Ok(vec![CoedgeRecord {
                id: take_id(),
                edge_id: end.arc_edge_id,
                forward: true,
                pcurve: end.arc_blend_pcurve.clone(),
            }]),
            EndPlan::Miter(end) => end
                .edges
                .iter()
                .map(|(edge_id, forward, pcurve)| {
                    Ok(CoedgeRecord {
                        id: take_id(),
                        edge_id: *edge_id,
                        forward: *forward,
                        pcurve: pcurve.clone(),
                    })
                })
                .collect(),
            EndPlan::Free(free) => {
                let edge_id =
                    free_edge_id.ok_or(KernelRefusal::internal(KernelStage::Sew, "transverse_edge", "blend: free end without its transverse edge"))?;
                Ok(vec![if natural_forward {
                    CoedgeRecord {
                        id: take_id(),
                        edge_id,
                        forward: true,
                        pcurve: free.transverse_blend_pcurve.clone(),
                    }
                } else {
                    CoedgeRecord {
                        id: take_id(),
                        edge_id,
                        forward: false,
                        pcurve: free.transverse_blend_pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?,
                    }
                }])
            }
        }
    };
    let start_slot = end_slot(&start_end, transverse_a_id, !blend_cr_forward)?;
    let finish_slot = end_slot(&finish_end, transverse_b_id, blend_cr_forward)?;
    // A collapsed rail has no coedge: the end slots on either side of it meet
    // at its one rim point.
    let coedges = if blend_cr_forward {
        let mut coedges = Vec::new();
        if let Some(cr_edge_id) = cr_edge_id {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: cr_edge_id,
                forward: cr_forward,
                pcurve: crate::sweep_topology::parameter_line(
                    cr_domain[0],
                    0.0,
                    cr_domain[1],
                    0.0,
                ).or_refuse(KernelStage::Refine, "parameter_line")?,
            });
        }
        coedges.extend(finish_slot);
        if let Some(cs_edge_id) = cs_edge_id {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: cs_edge_id,
                forward: cs_forward,
                pcurve: cs_blend_pcurve_reversed(&cs_domain)?,
            });
        }
        coedges.extend(start_slot);
        coedges
    } else {
        let mut coedges = Vec::new();
        if let Some(cr_edge_id) = cr_edge_id {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: cr_edge_id,
                forward: cr_forward,
                pcurve: crate::sweep_topology::parameter_line(
                    cr_domain[1],
                    0.0,
                    cr_domain[0],
                    0.0,
                ).or_refuse(KernelStage::Refine, "parameter_line")?,
            });
        }
        coedges.extend(start_slot);
        if let Some(cs_edge_id) = cs_edge_id {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: cs_edge_id,
                forward: cs_forward,
                pcurve: crate::sweep_topology::parameter_line(
                    cs_domain[0],
                    1.0,
                    cs_domain[1],
                    1.0,
                ).or_refuse(KernelStage::Refine, "parameter_line")?,
            });
        }
        coedges.extend(finish_slot);
        coedges
    };
    let blend_face_id = take_id();
    let mut blend_face = FaceRecord {
        id: blend_face_id,
        surface: rows.surface,
        same_sense,
        loops: vec![LoopRecord {
            id: loop_id,
            coedges,
        }],
        name: name.map(|value| value.to_string()),
    };
    if rows.exact_extrusion {
        transpose_face(&mut blend_face)?;
    }
    let shell_index = result
        .shells
        .iter()
        .position(|shell| shell.faces.iter().any(|face| face.id == first.face.id))
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_shell", "blend: mate shell lost during surgery"))?;
    // Drop the fully-consumed mate faces (lesson 3): their loops collapsed
    // between the sewn edge and the blend; the consumed boundaries and
    // lesson-7 leftovers are deleted below, the sewn edge lives on shared by
    // the blend face and the surviving neighbour.
    for (mate, sew) in [(first, &sew_first), (second, &sew_second)] {
        if sew.is_some() {
            for shell in &mut result.shells {
                shell.faces.retain(|face| face.id != mate.face.id);
            }
        }
    }
    result.shells[shell_index].faces.push(blend_face);

    result.edges.retain(|candidate| candidate.id != edge.id);
    // Delete the boundary edges a prior fillet's corner had left behind that
    // this blend fully consumed.
    result
        .edges
        .retain(|candidate| !consumed_edges.contains(&candidate.id));
    // Each collapsed rail's two rims are one vertex.
    let mut identified = Vec::new();
    for (collapsed, from, to) in [(cr_collapsed, w1a, w1b), (cs_collapsed, w2a, w2b)] {
        if collapsed && from != to {
            identified.push((from, to));
        }
    }
    Ok(SewnStripe {
        cr_edge_id,
        cs_edge_id,
        blend_face_id,
        consumed: [sew_first.is_some(), sew_second.is_some()],
        snap,
        identified,
    })
}

/// Swap the two parameters of a face: the surface's control net, every
/// coedge pcurve, and the sense (S_u × S_v changes sign).  The face's
/// geometry is untouched; only its chart is.  Used to hand an exact
/// cylinder patch built with its section along v to the analytic recogniser,
/// which wants the circle along u.
pub(in crate::blend) fn transpose_face(face: &mut FaceRecord) -> Result<(), KernelRefusal> {
    let surface = &face.surface;
    let rows_u = surface.control_points.len();
    let rows_v = surface.control_points.first().map(|row| row.len()).unwrap_or(0);
    let mut transposed = vec![Vec::with_capacity(rows_u); rows_v];
    for row in &surface.control_points {
        for (j, point) in row.iter().enumerate() {
            transposed[j].push(*point);
        }
    }
    face.surface = NurbsSurface::new(
        surface.degree_v,
        surface.degree_u,
        surface.knots_v.clone(),
        surface.knots_u.clone(),
        transposed,
    ).or_refuse(KernelStage::Refine, "new")?;
    for loop_record in &mut face.loops {
        for coedge in &mut loop_record.coedges {
            for control in &mut coedge.pcurve.control_points {
                std::mem::swap(&mut control.x, &mut control.y);
            }
        }
    }
    face.same_sense = !face.same_sense;
    Ok(())
}

/// Drop every vertex no longer referenced by an edge — the sharp corners the
/// blends replaced, and anything freed by a consumed boundary.  Run ONCE after
/// a whole group of stripes is in: a corner vertex is still referenced by the
/// selected edges that have not been sewn yet.
pub(in crate::blend) fn prune_orphan_vertices(result: &mut BrepSolid) {
    let used: rustc_hash::FxHashSet<u64> = result
        .edges
        .iter()
        .flat_map(|candidate| [candidate.start_vertex_id, candidate.end_vertex_id])
        .collect();
    result
        .vertices
        .retain(|candidate| used.contains(&candidate.id));
}

fn cs_blend_pcurve_reversed(cs_domain: &[f64; 2]) -> Result<NurbsCurve, KernelRefusal> {
    crate::sweep_topology::parameter_line(cs_domain[1], 1.0, cs_domain[0], 1.0).or_refuse(KernelStage::Refine, "parameter_line")
}

