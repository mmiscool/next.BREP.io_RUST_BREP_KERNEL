use super::*;

fn finite_point(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

/// A foot that can be compared: a finite distance and a finite point.
fn usable_foot(foot: &crate::SurfaceProjection) -> bool {
    foot.distance.is_finite() && finite_point(foot.point)
}

/// A maximum that cannot lose an unreadable value: once any value is not
/// finite the reduction is NaN, reported as unreadable, never folded into a
/// finite partial maximum by `f64::max`.
fn fold_readable(worst: f64, value: f64) -> f64 {
    if worst.is_nan() || !value.is_finite() {
        f64::NAN
    } else {
        worst.max(value)
    }
}

/// The nearer usable foot of `point` on `surface`: the global projector's, and Newton seeded at the
/// trim's own `uv` (the image's parameter). Nearer means nearer the EDGE POINT; both are points OF
/// the surface, so the nearer is still an upper bound on the true distance. Both feet take one
/// admission: a finite distance and point and a finite (u, v) inside the surface's native domain
/// (the seeded Newton is bounded to it; this checks the answer rather than trusting the bound). A
/// tie keeps the global (legacy) foot. An unusable global foot does not block a usable seeded one;
/// an unusable seed keeps a usable global foot exactly; with no usable foot, or an unreadable query
/// point, the answer is `None` (unreadable). On a carrier that comes back near itself the global
/// search can land on the far sheet; the seeded foot is then the nearer point.
fn nearer_foot(
    surface: &NurbsSurface,
    point: Vec3,
    global: crate::SurfaceProjection,
    uv: Vec3,
) -> Result<Option<crate::SurfaceProjection>, String> {
    if !finite_point(point) {
        return Ok(None);
    }
    // ONE admission for both feet: a finite distance and point, and a finite
    // (u, v) inside the native domain.
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let admitted = |foot: &crate::SurfaceProjection| {
        usable_foot(foot) && foot.u.is_finite() && foot.v.is_finite() && (u0..=u1).contains(&foot.u) && (v0..=v1).contains(&foot.v)
    };
    let global = admitted(&global).then_some(global);
    let mut seeded = None;
    if uv.x.is_finite() && uv.y.is_finite() {
        if let Ok(foot) = crate::project_point_to_surface_seeded(surface, point, uv.x, uv.y) {
            if admitted(&foot) {
                seeded = Some(foot);
            }
        }
    }
    // A tie keeps the global foot: the legacy reading stands unless the seed is
    // strictly nearer.
    Ok(match (global, seeded) {
        (Some(global), Some(seeded)) => Some(if seeded.distance < global.distance { seeded } else { global }),
        (global, seeded) => global.or(seeded),
    })
}

/// The largest distance between `pcurve`'s image and the FOOT of the edge
/// point it stands for, over 1025 evenly spaced fractions, on the nearer
/// usable foot ([`nearer_foot`]); the second value is the same read on the
/// global projector's foot alone (the reading before 2026-10-04). The foot
/// is chosen by its distance to the EDGE point, not to the image, so neither
/// value bounds the other. Any unreadable station (query point, image or
/// foot) makes that reading NaN ([`fold_readable`]). The deviation from the
/// edge point reads a tangential slip `d` at a station `g` off the carrier as
/// only `d²/(2g)`; this reads it whole, and is what tells a file's residual
/// (image on the feet) from a fitter's miss.
fn image_to_foot(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    forward: bool,
) -> Result<(f64, f64), String> {
    let [p0, p1] = pcurve.domain()?;
    let (mut worst, mut worst_global) = (0.0_f64, 0.0_f64);
    for index in 0..=1024 {
        let fraction = index as f64 / 1024.0;
        let t = if forward { t0 + (t1 - t0) * fraction } else { t1 - (t1 - t0) * fraction };
        let point = curve.evaluate(t)?;
        let global = crate::project_point_to_surface(surface, point)?;
        let uv = pcurve.evaluate(p0 + (p1 - p0) * fraction)?;
        let image = surface.evaluate_extended(uv.x, uv.y)?;
        let readable_image = finite_point(image) && finite_point(point);
        let global_read = if readable_image && usable_foot(&global) { image.sub(global.point).length() } else { f64::NAN };
        worst_global = fold_readable(worst_global, global_read);
        let chosen_read = match nearer_foot(surface, point, global, uv)? {
            Some(foot) if readable_image => image.sub(foot.point).length(),
            _ => f64::NAN,
        };
        worst = fold_readable(worst, chosen_read);
    }
    Ok((worst, worst_global))
}

impl<'a> SolidBuilder<'a> {
    /// Build a loop's coedges, deriving pcurves by projection, placing seam
    /// coedges on opposite domain boundaries and re-synthesising collapsed pole
    /// boundaries as degenerate edges so the parameter loop closes.
    pub(super) fn build_loop(
        &mut self,
        surface: &NurbsSurface,
        face_ref: usize,
        surface_ref: usize,
        specs: &[(u64, bool)],
    ) -> Result<LoopRecord, String> {
        let [u0, u1] = surface.domain_u()?;
        let [v0, v1] = surface.domain_v()?;
        let (closed_u, closed_v) = surface.closed_directions()?;
        let u_span = u1 - u0;
        let v_span = v1 - v0;
        let pcurve_tol = loop_pcurve_tolerance(surface)?;

        // Which edges appear twice in this loop → seam pair candidates.
        let mut occurrence: HashMap<u64, usize> = HashMap::default();
        for (edge_id, _) in specs {
            *occurrence.entry(*edge_id).or_insert(0) += 1;
        }

        // Cyclic-rotate the loop to START on a NON-seam coedge.  A seam
        // coedge placed first has no continuity cursor, so its boundary is
        // a blind guess (lo); the rim directions that follow are fixed by
        // geometry and can demand the OPPOSITE boundary — the loop then
        // walks [seam-down, rim-backwards, …], a bowtie polygon whose
        // triangulation cuts straight across the face (Unnamed-io1-ug
        // flange: the center bore rendered as a filled disc).  Starting on
        // a rim grounds the cursor in real geometry before any seam is
        // pinned; loops that are ALL seam pairs keep their order.
        let rotated: Vec<(u64, bool)>;
        let specs: &[(u64, bool)] = if let Some(start) = specs
            .iter()
            .position(|(edge_id, _)| occurrence.get(edge_id).copied().unwrap_or(0) < 2)
        {
            let mut reordered = specs.to_vec();
            reordered.rotate_left(start);
            rotated = reordered;
            &rotated
        } else {
            specs
        };

        #[derive(Clone, Copy, PartialEq)]
        enum Seam {
            None,
            U,
            V,
        }

        struct Placed {
            edge_id: u64,
            forward: bool,
            pcurve: NurbsCurve,
            start: (f64, f64),
            end: (f64, f64),
        }

        let mut placed: Vec<Placed> = Vec::new();
        let mut cursor: Option<(f64, f64)> = None;
        // edge_id → boundary value chosen for its first seam coedge.
        let mut seam_boundary: HashMap<u64, f64> = HashMap::default();
        // Vertices that a degenerate edge collapses onto in THIS loop — the
        // POLES/apexes. A rim→apex seam ruling (synthesized by
        // `stitch_seam_circle_loops` for a pointed cone / sphere cap) attaches
        // one of its ends here; near that pole the projection-fitted pcurve
        // wanders across the domain, so the constant-coordinate seam test below
        // cannot recognise it. Knowing the pole vertices lets us recover the
        // seam classification from topology instead (see the rescue block).
        let pole_vertices: HashSet<u64> = specs
            .iter()
            .filter_map(|(edge_id, _)| {
                let edge = self.edge_record(*edge_id);
                edge.degenerate.then_some(edge.start_vertex_id)
            })
            .collect();

        // Self-overlapping non-periodic carrier (synthesized-trim thread band):
        // assign every coedge's branch JOINTLY around the loop so the UV polygon
        // does not self-cross. `None` on every other face → old path untouched.
        let joint = self.joint_branch_pcurves(surface, specs, pcurve_tol)?;

        for (index, (edge_id, forward)) in specs.iter().enumerate() {
            if let Some(ref joint_pcurves) = joint {
                // Branch chosen by the loop-level pass: bypass the per-coedge
                // seam/periodic logic (inert for this open×open class anyway) and
                // seat the pcurve directly. Consecutive coedges share exact UV
                // vertices by construction, so the gap-fill below mints no bridge.
                let pcurve = joint_pcurves[index].clone();
                let dom = pcurve.domain()?;
                let s = pcurve.evaluate(dom[0])?;
                let e = pcurve.evaluate(dom[1])?;
                cursor = Some((e.x, e.y));
                placed.push(Placed {
                    edge_id: *edge_id,
                    forward: *forward,
                    pcurve,
                    start: (s.x, s.y),
                    end: (e.x, e.y),
                });
                continue;
            }
            // The vendor's own statement of this edge's locus in THIS face's
            // parameter space, when the file supplies one and it verifies.
            // Falls through to the projection fit otherwise, which is what the
            // importer did for every coedge before this lane existed.
            let supplied =
                self.supplied_loop_pcurve(surface, surface_ref, *edge_id, *forward, pcurve_tol)?;
            let edge = self.edge_record(*edge_id).clone();
            let mut pcurve = match supplied {
                Some(pcurve) => pcurve,
                None => {
                    // Every read below (the floor fit's challenger, the
                    // keep's) projects the same edge points: memoized on
                    // exact inputs, same answers.
                    let _feet = crate::pcurve::FootMemoScope::open(surface);
                    // Fitted at the size-derived ask first, as the importer did
                    // until 2026-09-26: that is the refinement the corpus's
                    // off-carrier trims were built and tessellated at. A fit
                    // the detector reads off its bar (over the 1e-7 floor plus
                    // the stations' standoff) is refitted at the floor and the
                    // fit with the smaller residual is kept. Chasing every trim
                    // to the floor instead spent 2000 samples on curves off
                    // their carriers and returned worse fits than the ask did
                    // (`abc_00000012` face #299 edge 179, `abc_00000011` #585:
                    // 2.06e-4 at the ask, 0.250 mm at the floor, measured
                    // 2026-09-26).
                    let mut fit = crate::fit_pcurve_on_surface_range(
                        surface,
                        &edge.curve,
                        edge.t0,
                        edge.t1,
                        *forward,
                        pcurve_tol,
                    )?;
                    let mut refitted = false;
                    let deviation_trigger = fit.report.residual > trim_detector_bar(&fit.report);
                    if deviation_trigger && pcurve_tol > crate::PCURVE_REFINEMENT_TOLERANCE {
                        let floor = crate::fit_pcurve_on_surface_range(
                            surface,
                            &edge.curve,
                            edge.t0,
                            edge.t1,
                            *forward,
                            crate::PCURVE_REFINEMENT_TOLERANCE,
                        )?;
                        if floor.report.residual < fit.report.residual {
                            fit = floor;
                            refitted = true;
                        }
                    }
                    // The band-free read between the fit's stations
                    // (`trim_floor_reading`): the residual at the pcurve's
                    // knot-span quarter points and the edge curve's knot-span
                    // midpoints, taken on EVERY derived trim so a miss the
                    // fit's own grid cannot see (edge 237: 7.4e-4 inside a
                    // 6e-5-wide knot span, read 6.8e-5 by the fit) is booked
                    // as the band it is. The refit trigger above stays on the
                    // fit's own residual: this read reports, it does not yet
                    // act (2026-10-03).
                    let floor = trim_floor_reading(surface, &fit.curve, &edge.curve, edge.t0, edge.t1, *forward, false)?;
                    // NaN-preserving: an unread reading is reported, never
                    // folded into a clean maximum.
                    let residual = fold_reading(fit.report.residual, floor.residual);
                    // A non-finite bar (an unread station standoff) is no bar:
                    // `x <= inf` would suppress an unread trim.
                    let bar = trim_detector_bar(&fit.report);
                    if !(bar.is_finite() && residual <= bar) {
                        let report = fit.report;
                        let (image_to_foot, image_to_global_foot) =
                            image_to_foot(surface, &fit.curve, &edge.curve, edge.t0, edge.t1, *forward)?;
                        // The file curve's standoff BETWEEN the stations, on
                        // the same grid the residual was read on; the bar's
                        // standoff term is the larger of it and the nodes'.
                        let between = trim_floor_reading(
                            surface, &fit.curve, &edge.curve, edge.t0, edge.t1, *forward, true,
                        )?;
                        let standoff = fold_reading(report.off_surface, between.standoff);
                        // Whose miss: the residual is within the floor of the
                        // file curve's own standoff (the file's), or over it
                        // (the fitter's).
                        // Unread (NaN) is not exempted as the file's.
                        let fitter = !(residual <= crate::PCURVE_REFINEMENT_TOLERANCE + standoff);
                        // Accepted whatever it reads, and named: the importer
                        // before the bar returned this fit unmeasured, so a
                        // refusal here loses a body it imported. `abc 00008080`
                        // face #163 edge 476 shipped at 3.794e-3 (a line 3.6e-4
                        // off its 2×1 carrier, 514 nodes) before the bar and
                        // after; the fitter's miss is the next slice's to fix,
                        // not the body's to lose (2026-09-26 ruling).
                        self.bounded.push(readings::BoundedTrim {
                            face_ref,
                            surface_ref,
                            edge_id: *edge_id,
                            residual,
                            standoff,
                            fit_residual: report.residual,
                            fit_standoff: report.off_surface,
                            image_to_foot,
                            image_to_global_foot,
                            samples: report.samples,
                            exit: format!("{:?}", report.exit),
                            fitter,
                            clamped_excursion: report.clamped_excursion,
                            clamped_distance: report.clamped_distance,
                        });
                        self.capture_stage(*edge_id, || {
                            format!(
                                "trim off its bar on face #{face_ref}: {:.3e} against a floor of {:.1e}, the file's curve {:.3e} off the carrier, image {:.3e} off its feet, {:.3e} off the global projector's ({}; {:?} at {} samples{}{}; the fit read {:.3e} with its stations {:.3e} off; the reading is the maximum at {} sampled probes, not a certified maximum)",
                                residual,
                                crate::PCURVE_REFINEMENT_TOLERANCE,
                                standoff,
                                image_to_foot,
                                image_to_global_foot,
                                if fitter { "the fitter's miss" } else { "the file's residual" },
                                report.exit,
                                report.samples,
                                if refitted { ", refitted at the floor" } else { "" },
                                if report.clamped_excursion > 0.0 {
                                    format!(
                                        "; clamped into the chart by {:.3e} in parameter, {:.3e} mm",
                                        report.clamped_excursion, report.clamped_distance
                                    )
                                } else {
                                    String::new()
                                },
                                report.residual,
                                report.off_surface,
                                floor.probes,
                            )
                        });
                    }
                    fit.curve
                }
            };
            if edge.degenerate {
                // Projection at a surface singularity has many equally valid
                // UV answers and sampling a point-curve can jump between them.
                // Pin the whole pcurve to one representative so evaluation
                // remains exactly consistent with the collapsed 3D edge.
                let point = self.vertex_point(edge.start_vertex_id);
                let projection = crate::project_point_to_surface(surface, point)?;
                let uv = Vec3::new(projection.u, projection.v, 0.0);
                pcurve = make_line(uv, uv)?;
            } else if closed_u && edge.start_vertex_id == edge.end_vertex_id {
                // A full circle projected point-by-point crosses the periodic
                // branch cut, which can make a fitted pcurve double back and
                // map to the opposite side of the cone/cylinder. If it is a
                // v-isocurve, represent its trim exactly as one full u period.
                let mut projected_v_lo = f64::INFINITY;
                let mut projected_v_hi = f64::NEG_INFINITY;
                let mut projected_v_sum = 0.0;
                for index in 0..8 {
                    let fraction = (index as f64 + 0.5) / 8.0;
                    let point = edge
                        .curve
                        .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)?;
                    let v = crate::project_point_to_surface(surface, point)?.v;
                    projected_v_lo = projected_v_lo.min(v);
                    projected_v_hi = projected_v_hi.max(v);
                    projected_v_sum += v;
                }
                if projected_v_hi - projected_v_lo <= 1e-3 * v_span.abs().max(1e-9) {
                    let v = projected_v_sum / 8.0;
                    // Select the sense by direct geometric agreement at an
                    // interior quarter turn. This is more robust than a
                    // projected-angle delta when a conic starts on the branch
                    // cut (or its STEP placement uses the opposite normal).
                    let fraction = 0.25;
                    let t = if *forward {
                        edge.t0 + (edge.t1 - edge.t0) * fraction
                    } else {
                        edge.t1 - (edge.t1 - edge.t0) * fraction
                    };
                    let on_curve = edge.curve.evaluate(t)?;
                    let increasing_point = surface.evaluate(u0 + u_span * fraction, v)?;
                    let decreasing_point = surface.evaluate(u1 - u_span * fraction, v)?;
                    let increasing = increasing_point.sub(on_curve).length()
                        <= decreasing_point.sub(on_curve).length();
                    let (from_u, to_u) = if increasing { (u0, u1) } else { (u1, u0) };
                    let linear =
                        make_line(Vec3::new(from_u, v, 0.0), Vec3::new(to_u, v, 0.0))?;
                    // A STEP B-spline can trace an exact full circle with a
                    // non-uniform parameter speed.  Replacing its fitted trim
                    // by a straight u ramp then changes the point pairing even
                    // though both curves describe the same geometric circle.
                    // Keep the historical exact linear representation only
                    // when it actually follows the edge inside the fit band;
                    // otherwise retain the seam-unwrapped fitted pcurve above.
                    if pcurve_tracks_edge(
                        surface,
                        &linear,
                        &edge.curve,
                        edge.t0,
                        edge.t1,
                        *forward,
                        pcurve_tol,
                    )? {
                        pcurve = linear;
                    }
                }
            }

            // Classify: seam only if the edge occurs twice AND rides a closed
            // boundary (one uv coordinate is ~constant across the whole
            // coedge) AND that constant actually sits ON the domain boundary.
            // A face can legitimately touch an INTERIOR meridian twice (a
            // wrap-around loop pinched along a doubled edge away from the
            // seam) — pinning such a pair to u0/u1 tears the loop apart.
            let (u_lo, u_hi, v_lo, v_hi) = pcurve_extent(&pcurve)?;
            let mut seam = Seam::None;
            if occurrence.get(edge_id).copied().unwrap_or(0) >= 2 {
                let u_tol = 1e-3 * u_span.max(1e-9);
                let v_tol = 1e-3 * v_span.max(1e-9);
                let u_const = (u_hi - u_lo) <= u_tol;
                let v_const = (v_hi - v_lo) <= v_tol;
                let u_mid = 0.5 * (u_lo + u_hi);
                let v_mid = 0.5 * (v_lo + v_hi);
                // Pinning tolerance: generous (a wrong-branch projection puts
                // the fit a whole period off, never a fraction), but strict
                // enough to leave interior doubled meridians alone.
                let on_u_boundary = (u_mid - u0).abs() <= 0.05 * u_span.max(1e-9)
                    || (u1 - u_mid).abs() <= 0.05 * u_span.max(1e-9);
                let on_v_boundary = (v_mid - v0).abs() <= 0.05 * v_span.max(1e-9)
                    || (v1 - v_mid).abs() <= 0.05 * v_span.max(1e-9);
                let u_seam = closed_u && u_const && on_u_boundary;
                let v_seam = closed_v && v_const && on_v_boundary;
                if u_seam && !(v_seam && v_span < u_span) {
                    seam = Seam::U;
                } else if v_seam {
                    seam = Seam::V;
                } else if u_seam {
                    seam = Seam::U;
                }
            }

            // Seam-to-apex meridian rescue. `stitch_seam_circle_loops` closes a
            // pointed cone / sphere cap by ruling the periodic (u) seam from a
            // rim vertex down to the apex (a VERTEX_LOOP pole). That ruling's 3D
            // curve rides the u=u0 seam, but its far end is a hair from the pole,
            // where the surface is singular: an INDEPENDENT global inversion of a
            // near-pole sample JUMPS to a distant uv branch, so the fitted pcurve
            // above looks diagonal and the constant-u seam test rejects it.
            // Recover the classification from topology — the edge occurs twice,
            // one endpoint is a pole and the other projects cleanly onto the
            // u-seam — and rebuild the pcurve by SEEDED tracking from the rim end
            // (each sample's inversion seeded from its neighbour's parameters, so
            // it stays on the seam branch instead of snapping to the pole's fold)
            // with u forced onto the seam. Pinning then seats the two instances on
            // the opposite u-boundaries like any other seam pair. Gated on the
            // constant-u test having FAILED: where the fit stayed clean (a wider
            // cap whose samples never near the pole) the pinned pcurve already
            // rides the seam, so we leave it untouched to keep every previously
            // valid apex import byte-identical.
            if seam == Seam::None
                && closed_u
                && occurrence.get(edge_id).copied().unwrap_or(0) >= 2
            {
                let sv = edge.start_vertex_id;
                let ev = edge.end_vertex_id;
                let rim_apex = if pole_vertices.contains(&ev) && !pole_vertices.contains(&sv) {
                    Some((sv, ev))
                } else if pole_vertices.contains(&sv) && !pole_vertices.contains(&ev) {
                    Some((ev, sv))
                } else {
                    None
                };
                if let Some((rim, _apex)) = rim_apex {
                    let rim_proj = crate::project_point_to_surface(surface, self.vertex_point(rim))?;
                    let on_seam = (rim_proj.u - u0).abs().min((rim_proj.u - u1).abs())
                        <= 0.05 * u_span.abs().max(1e-9);
                    if on_seam {
                        // Sample the coedge (fraction 0→1 in coedge sense), but
                        // drive the seed chain from the RELIABLE rim end so every
                        // inversion continues from a good neighbour. `u` is forced
                        // to the seam meridian; only the seeded `v` (which the
                        // near-pole global search gets wrong but continuation
                        // tracks) is kept.
                        let coedge_start = if *forward { sv } else { ev };
                        let rim_is_start = coedge_start == rim;
                        const SAMPLES: usize = 48;
                        let mut vs = vec![0.0f64; SAMPLES + 1];
                        let mut seed = (rim_proj.u, rim_proj.v);
                        let order: Vec<usize> = if rim_is_start {
                            (0..=SAMPLES).collect()
                        } else {
                            (0..=SAMPLES).rev().collect()
                        };
                        for index in order {
                            let fraction = index as f64 / SAMPLES as f64;
                            let edge_fraction = if *forward { fraction } else { 1.0 - fraction };
                            let t = edge.t0 + (edge.t1 - edge.t0) * edge_fraction;
                            let point = edge.curve.evaluate(t)?;
                            let proj = crate::project_point_to_surface_seeded(
                                surface, point, seed.0, seed.1,
                            )?;
                            seed = (proj.u, proj.v);
                            vs[index] = proj.v;
                        }
                        let points: Vec<Vec3> =
                            vs.iter().map(|&v| Vec3::new(u0, v, 0.0)).collect();
                        let parameters: Vec<f64> = (0..=SAMPLES)
                            .map(|index| index as f64 / SAMPLES as f64)
                            .collect();
                        pcurve = crate::interpolate_curve(&points, 3, &parameters)?;
                        seam = Seam::U;
                    }
                }
            }

            if seam != Seam::None {
                let (lo, hi) = if seam == Seam::U { (u0, u1) } else { (v0, v1) };
                let boundary = if let Some(&other) = seam_boundary.get(edge_id) {
                    // Second coedge of the pair → the opposite boundary.
                    if (other - lo).abs() < (other - hi).abs() {
                        hi
                    } else {
                        lo
                    }
                } else if let Some((cu, cv)) = cursor {
                    let c = if seam == Seam::U { cu } else { cv };
                    if (c - lo).abs() <= (c - hi).abs() {
                        lo
                    } else {
                        hi
                    }
                } else {
                    lo
                };
                seam_boundary.entry(*edge_id).or_insert(boundary);
                pin_pcurve(&mut pcurve, seam == Seam::U, boundary)?;
            }

            let dom = pcurve.domain()?;
            let mut s = pcurve.evaluate(dom[0])?;
            let mut e = pcurve.evaluate(dom[1])?;

            // Periodic-aware connection: a non-seam coedge whose start sits a
            // whole period away from the previous coedge's end (the SAME 3D
            // point across the surface seam — e.g. a ruling projected to the
            // wrong branch on an OCC full/partial cylinder) is shifted to meet
            // it, provided the shift keeps the whole pcurve inside the domain.
            if seam == Seam::None {
                if let Some((cu, cv)) = cursor {
                    let du = if closed_u && u_span > 1e-12 {
                        ((cu - s.x) / u_span).round() * u_span
                    } else {
                        0.0
                    };
                    let dv = if closed_v && v_span > 1e-12 {
                        ((cv - s.y) / v_span).round() * v_span
                    } else {
                        0.0
                    };
                    // Prefer the full periodic shift that meets the cursor
                    // exactly. When it leaves the domain — a DIAGONAL corner hop
                    // on a surface periodic in BOTH directions (a torus fillet
                    // band that rides one seam while wrapping the other), whose
                    // opposite-corner rep sends u OR v a whole period out of
                    // range — fall back to the single-axis shift that stays in
                    // domain. That re-seats the coedge onto the seam boundary
                    // the loop is riding, collapsing the diagonal corner gap
                    // into a plain single-direction seam wrap the loop-closer
                    // and the (raw, in-domain) integrator/tessellator handle.
                    for (candidate_du, candidate_dv) in [(du, dv), (du, 0.0), (0.0, dv)] {
                        if candidate_du == 0.0 && candidate_dv == 0.0 {
                            continue;
                        }
                        let shifted = shift_pcurve(&pcurve, candidate_du, candidate_dv)?;
                        if pcurve_in_domain(&shifted, u0, u1, v0, v1) {
                            pcurve = shifted;
                            s = pcurve.evaluate(dom[0])?;
                            e = pcurve.evaluate(dom[1])?;
                            break;
                        }
                    }
                }
            }

            let start = (s.x, s.y);
            let end = (e.x, e.y);
            cursor = Some(end);
            placed.push(Placed {
                edge_id: *edge_id,
                forward: *forward,
                pcurve,
                start,
                end,
            });
        }

        // An explicit collapsed EDGE_CURVE/VERTEX_LOOP must span the two
        // neighboring seam endpoints in UV. Leaving it at one arbitrary pole
        // representative disconnects the parameter wire after seam placement.
        // Only accept a full iso-row whose control hull proves it collapsed;
        // endpoint coincidence alone also occurs on ordinary periodic seams.
        let anchor = surface.control_points[0][0].point()?;
        let mut local_extent = 0.0_f64;
        for control in surface.control_points.iter().flatten() {
            local_extent = local_extent.max(control.point()?.sub(anchor).length());
        }
        let pole_band = (1e-10 * (1.0 + local_extent)).min(1e-7);
        for index in 0..placed.len() {
            let previous = &placed[(index + placed.len() - 1) % placed.len()];
            let next = &placed[(index + 1) % placed.len()];
            let edge = self.edge_record(placed[index].edge_id);
            if !edge.degenerate
                || self.edge_record(previous.edge_id).degenerate
                || self.edge_record(next.edge_id).degenerate
            {
                continue;
            }
            let (from, to) = (previous.end, next.start);
            let row = if from.1 == to.1 {
                surface.iso_curve_v(from.1)?
            } else if from.0 == to.0 {
                surface.iso_curve_u(from.0)?
            } else {
                continue;
            };
            let point = row.control_points[0].point()?;
            // An IDENTITY band (is the row's start this loop's pole vertex?),
            // not a fit ask: the expression the fit ask carried until
            // 2026-09-26, kept here so the pole rescue reads as before.
            let pole_identity = {
                let scale = surface_scale(surface)?;
                (1e-6 * (1.0 + scale))
                    .min(0.5 * crate::KernelTolerances::for_scale(scale, 1e-7).pcurve_consistency)
            };
            if point.sub(self.vertex_point(edge.start_vertex_id)).length() > pole_identity {
                continue;
            }
            let mut collapsed = true;
            for control in &row.control_points {
                if control.point()?.sub(point).length() > pole_band {
                    collapsed = false;
                    break;
                }
            }
            if collapsed {
                placed[index].pcurve =
                    make_line(Vec3::new(from.0, from.1, 0.0), Vec3::new(to.0, to.1, 0.0))?;
                placed[index].start = from;
                placed[index].end = to;
            }
        }

        if std::env::var("BREP_DEBUG_STEP_LOOP").is_ok() {
            for (index, item) in placed.iter().enumerate() {
                let edge = self.edge_record(item.edge_id);
                let p0 = edge.curve.evaluate(edge.t0).unwrap_or_default();
                let p1 = edge.curve.evaluate(edge.t1).unwrap_or_default();
                let mut worst = 0.0f64;
                let mut worst_fraction = 0.0f64;
                let mut worst_project = 0.0f64;
                for sample in 0..=256 {
                    let fraction = sample as f64 / 256.0;
                    let dom = item.pcurve.domain().unwrap_or([0.0, 1.0]);
                    let uv = item
                        .pcurve
                        .evaluate(dom[0] + (dom[1] - dom[0]) * fraction)
                        .unwrap_or_default();
                    let on_surface = surface.evaluate(uv.x, uv.y).unwrap_or_default();
                    let edge_fraction = if item.forward { fraction } else { 1.0 - fraction };
                    let on_edge = edge
                        .curve
                        .evaluate(edge.t0 + (edge.t1 - edge.t0) * edge_fraction)
                        .unwrap_or_default();
                    let deviation = on_surface.sub(on_edge).length();
                    if deviation > worst {
                        worst = deviation;
                        worst_fraction = fraction;
                    }
                    if let Ok(projection) = crate::project_point_to_surface(surface, on_edge) {
                        let nearest = surface
                            .evaluate(projection.u, projection.v)
                            .unwrap_or_default();
                        worst_project = worst_project.max(nearest.sub(on_edge).length());
                    }
                }
                if worst > 1e-4 {
                    eprintln!(
                        "  DEVIATION edge {}: pcurve-vs-edge {:.6} at f={:.4}, edge-to-surface {:.6}",
                        item.edge_id, worst, worst_fraction, worst_project
                    );
                    if worst > 4e-3 {
                        let dom = item.pcurve.domain().unwrap_or([0.0, 1.0]);
                        let uv = item
                            .pcurve
                            .evaluate(dom[0] + (dom[1] - dom[0]) * worst_fraction)
                            .unwrap_or_default();
                        let edge_fraction = if item.forward {
                            worst_fraction
                        } else {
                            1.0 - worst_fraction
                        };
                        let on_edge = edge
                            .curve
                            .evaluate(edge.t0 + (edge.t1 - edge.t0) * edge_fraction)
                            .unwrap_or_default();
                        let mapped = surface.evaluate(uv.x, uv.y).unwrap_or_default();
                        if let Ok(projection) = crate::project_point_to_surface(surface, on_edge) {
                            eprintln!(
                                "    spike: edge3d ({:.4},{:.4},{:.4}) pcurve uv ({:.4},{:.4}) -> ({:.4},{:.4},{:.4}); proj uv ({:.4},{:.4}) dist {:.6}",
                                on_edge.x, on_edge.y, on_edge.z, uv.x, uv.y,
                                mapped.x, mapped.y, mapped.z,
                                projection.u, projection.v, projection.distance
                            );
                        }
                    }
                }
                eprintln!(
                    "LOOP[{index}] edge {} fwd={} uv ({:.4},{:.4})->({:.4},{:.4}) 3d ({:.2},{:.2},{:.2})->({:.2},{:.2},{:.2}) sv={} ev={} degen={}",
                    item.edge_id, item.forward, item.start.0, item.start.1, item.end.0,
                    item.end.1, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z,
                    edge.start_vertex_id, edge.end_vertex_id, edge.degenerate
                );
            }
        }
        // Fill parameter-domain gaps (dropped degenerate pole edges).
        let gap_tol = 1e-6 * u_span.max(v_span).max(1.0);
        // A periodic SEAM wrap is recognised much more loosely than a pole gap.
        // A dropped pole edge leaves a near-ZERO parameter gap (tight gap_tol),
        // but a seam wrap leaves a near-FULL-PERIOD gap: the loop's consecutive
        // coedges sit on OPPOSITE domain boundaries (u_min vs u_max), which are
        // the same 3D locus. Independent pcurve fits can land those endpoints a
        // fit-tolerance off the exact boundary (observed |du - period| up to
        // ~1e-4 of the span on real ABC faces), so the pole band is far too tight
        // to see the wrap and the loop tears open ("unsupported non-pole seam").
        // Because `du ≈ period` can ONLY mean "ends on opposite seam boundaries"
        // (a partial face leaves a SMALL interior gap, never a full-period one),
        // a generous fit-scale band is unambiguous and cannot swallow a real
        // sub-period gap.
        let seam_tol = 1e-3 * u_span.max(v_span).max(1.0);
        let mut coedges: Vec<CoedgeRecord> = Vec::new();
        let count = placed.len();
        for index in 0..count {
            let current = &placed[index];
            let next = &placed[(index + 1) % count];
            coedges.push(CoedgeRecord {
                id: self.fresh(),
                edge_id: current.edge_id,
                forward: current.forward,
                pcurve: current.pcurve.clone(),
            });
            // A STEP VERTEX_LOOP becomes one degenerate kernel edge. Its 3D
            // curve is a point and is therefore closed regardless of which UV
            // representative the projector chooses at a singularity.
            let current_edge = self.edge_record(current.edge_id);
            if current_edge.degenerate {
                continue;
            }
            let gap = distance2(current.end, next.start);
            if gap > gap_tol {
                // The topological vertex the two neighbouring coedges already
                // meet at (if any). Two pcurves fitted independently up to that
                // shared vertex can land it at slightly different uv, so a pure
                // parameter-space gap here is closed by a degenerate edge that
                // REUSES the vertex — keeping the coedge chain connected instead
                // of minting a new vertex by 3D proximity and tearing the loop.
                let next_edge = self.edge_record(next.edge_id);
                let current_end_vertex = if current.forward {
                    current_edge.end_vertex_id
                } else {
                    current_edge.start_vertex_id
                };
                let next_start_vertex = if next.forward {
                    next_edge.start_vertex_id
                } else {
                    next_edge.end_vertex_id
                };
                let bridge_vertex = (current_end_vertex == next_start_vertex)
                    .then_some(current_end_vertex);
                // On a periodic surface, opposite parameter-domain boundaries
                // are the same geometric locus.  Such a loop is already closed
                // topologically and in 3D; no artificial pole edge is needed.
                // Keep the two legal in-domain pcurves on their respective
                // branches and let periodic-aware tessellation unwrap them.
                let du = (current.end.0 - next.start.0).abs();
                let dv = (current.end.1 - next.start.1).abs();
                let wraps_u = closed_u && (du - u_span.abs()).abs() <= seam_tol && dv <= seam_tol;
                let wraps_v = closed_v && (dv - v_span.abs()).abs() <= seam_tol && du <= seam_tol;
                // Diagonal seam on a surface periodic in BOTH directions (torus,
                // sphere-as-torus): the loop steps across the u-seam AND the
                // v-seam at once, landing on the opposite domain corner. All
                // four domain corners denote the SAME 3D point, so this gap is a
                // full period in u and a full period in v simultaneously — the
                // loop is already closed geometrically; only its parameter reps
                // jump corner-to-corner. Treat it like a single-axis seam wrap.
                let wraps_uv = closed_u
                    && closed_v
                    && (du - u_span.abs()).abs() <= seam_tol
                    && (dv - v_span.abs()).abs() <= seam_tol;
                if wraps_u || wraps_v || wraps_uv {
                    // A complete-period gap can also be a collapsed pole row
                    // (sphere/cone). Prefer reconstructing that topological
                    // edge; leave it implicit only for a genuine non-collapsed
                    // periodic seam. The bridged-artefact relaxation is NOT
                    // offered here: a full-period wrap whose row happens to stay
                    // near a pole must remain implicit (bridging it would forge a
                    // spurious degenerate edge and perturb the reconstruction).
                    if let Ok(degenerate) = self.synthesize_pole_edge(
                        surface,
                        current.end,
                        next.start,
                        bridge_vertex,
                        false,
                    ) {
                        coedges.push(degenerate);
                    }
                } else {
                    // A SUB-period gap that shares a vertex is closeable as a
                    // near-pole seam-vertex artefact even when its row is not a
                    // perfectly collapsed pole.
                    coedges.push(self.synthesize_pole_edge(
                        surface,
                        current.end,
                        next.start,
                        bridge_vertex,
                        true,
                    )?);
                }
            }
        }

        let loop_id = self.fresh();
        if std::env::var("BREP_DEBUG_STEP_LOOP").is_ok() {
            let described: Vec<String> = coedges
                .iter()
                .map(|coedge| format!("ce{}→e{}", coedge.id, coedge.edge_id))
                .collect();
            eprintln!("LOOP-DONE id={loop_id} [{}]", described.join(" "));
        }
        Ok(LoopRecord {
            id: loop_id,
            coedges,
        })
    }

    /// A parameter-space gap between consecutive coedges must lie on a collapsed
    /// (pole) boundary — the exporter drops those degenerate edges. Re-create
    /// the degenerate edge + coedge that closes the domain there.
    fn synthesize_pole_edge(
        &mut self,
        surface: &NurbsSurface,
        from: (f64, f64),
        to: (f64, f64),
        bridge_vertex: Option<u64>,
        bridge_artefact_ok: bool,
    ) -> Result<CoedgeRecord, String> {
        let a = surface.evaluate(from.0, from.1)?;
        let b = surface.evaluate(to.0, to.1)?;
        let scale = surface_scale(surface)?;
        // Identity band for "these endpoints denote the same collapsed pole".
        // Two pcurves fitted INDEPENDENTLY up to a shared vertex can each be
        // displaced by up to their fit tolerance, so the projected endpoints
        // (and the collapsed row between them) may sit a few fit-tolerances
        // apart even on a genuine pole — the old 1e-6·(1+scale) band was below
        // that fitting floor and rejected real poles. Stay an order of
        // magnitude under the smallest genuine seam sweep (a full-period row
        // spans millimetres, orders larger) and well under the validator's
        // pcurve-consistency contract, so real periodic seams are never
        // collapsed into a spurious pole edge.
        let tolerance = (1e-5 * (1.0 + scale))
            .min(0.1 * crate::KernelTolerances::for_scale(scale, 1e-7).pcurve_consistency);
        // A true pole gap: the endpoints coincide AND the whole connecting row
        // (sampled between them) collapses to that same point. A seam gap has
        // coinciding endpoints but the row sweeps a full circle — those are
        // handled by periodic connection, never bridged here.
        let row_deviation = (1..8)
            .filter_map(|index| {
                let fraction = index as f64 / 8.0;
                let mid = (
                    from.0 + (to.0 - from.0) * fraction,
                    from.1 + (to.1 - from.1) * fraction,
                );
                surface
                    .evaluate(mid.0, mid.1)
                    .ok()
                    .map(|point| point.sub(a).length())
            })
            .fold(0.0_f64, f64::max);
        let row_collapsed = row_deviation <= tolerance;
        // A non-collapsed row is normally a genuine periodic seam sweep (or a
        // real open loop), never a pole — reject it. Endpoint coincidence is
        // only required when the neighbours do NOT already share a topological
        // vertex; when they do, the topology asserts the 3D meeting and the gap
        // is a pure parameter-space fitting artefact.
        //
        // EXCEPTION: a bridged gap whose ends AND whole connecting row still
        // denote the SHARED VERTEX is a seam-vertex fitting artefact — both uv
        // reps are that one corner and the tiny arc between them never leaves it
        // (e.g. a rim→apex seam meridian that meets its rim neighbour a few
        // thousandths off the exact seam boundary near a pole). Close it with a
        // degenerate edge reusing the vertex. Requiring the ROW (not just the
        // endpoints) to stay within band is what distinguishes this short
        // artefact from a FULL-PERIOD seam wrap, whose ends also coincide but
        // whose row sweeps an entire cross-section away — that wrap must stay
        // implicit, never bridged.
        //
        // The band is measured against the VERTEX, with the identity band
        // `BrepSolid::validate` uses to accept a curve endpoint AS its vertex.
        // Two things follow that comparing `a` to `b` against the raw absolute
        // `pcurve_consistency` field got wrong. First the question: the vertex —
        // not either pcurve end — is the kernel's authority for where the corner
        // is, and each end is an independent vendor approximation of it, so two
        // ends straddling it can be twice the identity band apart while each is
        // well inside it (abc_00000023 face #944: two ends ~4e-3 mm either side
        // of the shared vertex, 8.5e-3 mm apart, refused by a flat 4e-3 mm
        // band). Second the scale: this is an IDENTITY band, which per the
        // tolerance charter size-couples with the part at the site, while the
        // raw `pcurve_consistency` field stays tight because fit targets read
        // it.
        let coincidence_band = crate::KernelTolerances::for_scale(scale, 1e-7)
            .heal_band(2.0 * solid_like_scale(&self.vertices), 2e-5)
            .max(crate::VERTEX_MATCH_FLOOR);
        // Worst excursion of the bridged row (endpoints included) away from the
        // shared vertex it is supposed to collapse onto.
        let vertex_row_deviation = bridge_vertex.map(|vertex_id| {
            let vertex = self.vertex_point(vertex_id);
            (0..=8)
                .filter_map(|index| {
                    let fraction = index as f64 / 8.0;
                    let mid = (
                        from.0 + (to.0 - from.0) * fraction,
                        from.1 + (to.1 - from.1) * fraction,
                    );
                    surface
                        .evaluate(mid.0, mid.1)
                        .ok()
                        .map(|point| point.sub(vertex).length())
                })
                .fold(0.0_f64, f64::max)
        });
        let bridge_seam_artefact = bridge_artefact_ok
            && vertex_row_deviation.is_some_and(|deviation| deviation <= coincidence_band);
        if std::env::var("BREP_DEBUG_STEP_LOOP").is_ok() {
            eprintln!(
                "BRIDGE from={from:?} to={to:?} |a-b|={:.9} row_dev={row_deviation:.9} vertex_row_dev={vertex_row_deviation:?} tol={tolerance:.9} band={coincidence_band:.9} scale={scale:.4} bridge_vertex={bridge_vertex:?} artefact_ok={bridge_artefact_ok}",
                a.sub(b).length()
            );
        }
        if (!row_collapsed && !bridge_seam_artefact)
            || (bridge_vertex.is_none() && a.sub(b).length() > tolerance)
        {
            return Err(format!(
                "step_import: open parameter loop (uv gap {from:?}->{to:?} is not a collapsed \
                 pole; unsupported non-pole seam)"
            ));
        }
        // Reuse the neighbours' shared vertex when they meet there topologically;
        // otherwise reuse an existing vertex at the pole by proximity, else mint
        // one. Reusing the shared vertex keeps the coedge chain connected even
        // when independent pcurve fits land it a few microns apart in 3D.
        let vertex_id = match bridge_vertex {
            Some(vertex_id) => vertex_id,
            None => match self
                .vertices
                .iter()
                .find(|vertex| vertex.point.sub(a).length() <= IMPORT_IDENTITY_TOLERANCE)
            {
                Some(vertex) => vertex.id,
                None => {
                    let id = self.fresh();
                    self.push_vertex(VertexRecord { id, point: a });
                    id
                }
            },
        };
        // Collapse the degenerate edge onto its endpoint VERTEX rather than the
        // surface's own pole-row point `a`. On a non-axis-aligned (e.g. Y-axis)
        // cone/sphere the converted NURBS pole row can sit a fit-tolerance away
        // from the authoritative STEP apex vertex (~1e-4 here); pinning the
        // collapsed point to `a` then trips the validator's edge-start/end vs
        // vertex check. The reused vertex already carries the STEP apex and is
        // shared with the real rim edges, so it is the correct collapse target.
        let point = self
            .vertex_index
            .get(&vertex_id)
            .map_or(a, |&index| self.vertices[index].point);
        let edge_id = self.fresh();
        self.push_edge(EdgeRecord {
            id: edge_id,
            curve: make_line(point, point)?,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: vertex_id,
            end_vertex_id: vertex_id,
            degenerate: true,
            name: None,
        });
        Ok(CoedgeRecord {
            id: self.fresh(),
            edge_id,
            forward: true,
            pcurve: make_line(Vec3::new(from.0, from.1, 0.0), Vec3::new(to.0, to.1, 0.0))?,
        })
    }
}

