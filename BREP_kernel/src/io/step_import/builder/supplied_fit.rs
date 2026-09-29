use super::*;

/// Stations the supplied trim is sampled at before refinement. Matches the
/// derived lane's `base_samples` so an accepted supplied pcurve and the pcurve
/// it replaced are built from the same number of seeds, and a difference
/// between them is the DATA's, not the sampling's.
const SUPPLIED_STATIONS: usize = 64;

/// Relative distance within which an edge's 3D curve counts as LYING ON a
/// face's carrier, so that projecting it is an accurate reading of the trim
/// rather than a guess about a curve floating nearby.
///
/// Scaled by the solid's extent, like every other incidence question in the
/// kernel. The value sits far above the 1e-13 residuals an exact analytic
/// curve projects with, and far below the fit error of a curve that has
/// genuinely drifted off its surface — the constructed fixture's off-carrier
/// coedge measures 1.85e-4 on a 261 mm part. Nothing in between has been
/// observed, and a file that lands there would be one whose two statements
/// disagree by about the fit tolerance, where either answer is defensible.
const CARRIER_INCIDENCE: f64 = 1e-9;

impl<'a> SolidBuilder<'a> {
    /// The pcurve this coedge's supplied trim states, or `None` to fall through
    /// to the projection fit.
    ///
    /// Declining is ordinary: the importer derives a pcurve for every coedge
    /// anyway, so a supplied one is only ever used when it is READ, MAPPED and
    /// VERIFIED. Each decline is counted and reported rather than silently
    /// swallowed — the counts are the measurement the next slice needs.
    pub(super) fn supplied_loop_pcurve(
        &mut self,
        surface: &NurbsSurface,
        surface_ref: usize,
        edge_id: u64,
        forward: bool,
        pcurve_tol: f64,
    ) -> Result<Option<NurbsCurve>, String> {
        if !supplied_pcurves_enabled() {
            return Ok(None);
        }
        let edge = self.edge_record(edge_id);
        if edge.degenerate {
            // A collapsed bound has no locus to state: the loop below pins it
            // to a single representative uv, which no supplied curve improves.
            return Ok(None);
        }
        let Some(&curve_ref) = self.curve_ref_of_edge.get(&edge_id) else {
            return Ok(None);
        };
        let bundle = self.resolver.supplied_pcurves(curve_ref);
        if bundle.is_empty() {
            return Ok(None);
        }
        // A SEAM_CURVE names the SAME basis surface twice, once per branch, and
        // the loop visits its edge twice. Both branches are kept and offered in
        // file order; the seam pinning downstream is what seats the pair on
        // opposite domain boundaries, so taking the first matching branch here
        // states the locus without pre-empting that decision.
        let Some(supplied) = bundle
            .iter()
            .find(|candidate| candidate.basis_ref == surface_ref)
        else {
            return Ok(None);
        };
        self.supplied_report.offered += 1;

        let edge = self.edge_record(edge_id).clone();
        let span = edge.t1 - edge.t0;
        let mut cursor: Option<(f64, f64)> = None;
        let mut stations = Vec::with_capacity(SUPPLIED_STATIONS + 1);
        let mut worst_vs_curve = 0.0f64;
        for index in 0..=SUPPLIED_STATIONS {
            let fraction = index as f64 / SUPPLIED_STATIONS as f64;
            let edge_fraction = if forward { fraction } else { 1.0 - fraction };
            let point = edge.curve.evaluate(edge.t0 + span * edge_fraction)?;
            let (uv, on_trim) = supplied.snap(point, cursor)?;
            cursor = Some(uv);
            worst_vs_curve = worst_vs_curve.max(on_trim.sub(point).length());
            if std::env::var("BREP_DEBUG_SUPPLIED_STATIONS").is_ok_and(|v| v == format!("{curve_ref}")) {
                let (raw_u, raw_v) = supplied.surface.invert(point);
                eprintln!(
                    "  station {index:3} edge=({:.6},{:.6},{:.6}) invert=({:?},{:.6}) snap=({:.6},{:.6}) trim=({:.6},{:.6},{:.6}) gap={:.6e}",
                    point.x, point.y, point.z,
                    raw_u.map(|u| format!("{u:.6}")), raw_v,
                    uv.0, uv.1,
                    on_trim.x, on_trim.y, on_trim.z,
                    on_trim.sub(point).length()
                );
            }
            stations.push(on_trim);
        }

        // The supplied trim has to reach THIS edge's vertices. It does not when
        // the bundle's curve spans more than the edge does — an OCC habit of
        // reusing one long curve across several EDGE_CURVEs trimmed only by
        // their vertices — and a pcurve fitted to the wrong span would close a
        // loop on geometry the edge does not own.
        let traversal_start = if forward {
            edge.start_vertex_id
        } else {
            edge.end_vertex_id
        };
        let traversal_end = if forward {
            edge.end_vertex_id
        } else {
            edge.start_vertex_id
        };
        let scale = 1.0 + stations
            .iter()
            .fold(0.0f64, |extent, point| extent.max(point.length()));
        let endpoint_band = (1e-6 * scale).max(IMPORT_IDENTITY_TOLERANCE);
        let start_gap = stations[0]
            .sub(self.vertex_point(traversal_start))
            .length();
        let end_gap = stations[SUPPLIED_STATIONS]
            .sub(self.vertex_point(traversal_end))
            .length();
        if start_gap > endpoint_band || end_gap > endpoint_band {
            self.supplied_report.declined_endpoints += 1;
            supplied_trace(format_args!(
                "decline endpoints curve=#{curve_ref} basis=#{surface_ref} start_gap={start_gap:.6e} end_gap={end_gap:.6e} band={endpoint_band:.6e} worst_vs_curve={worst_vs_curve:.6e}"
            ));
            return Ok(None);
        }

        // Interpolate the stations so the fitter sees a continuous supplier:
        // its refinement asks for points BETWEEN stations, and asking the
        // supplied trim for those directly would re-solve the snap at each one
        // (the snap is a projection, so a mid-station snap can disagree with
        // its neighbours by a hair and give the refinement a target that moves
        // under it).
        let parameters: Vec<f64> = (0..=SUPPLIED_STATIONS)
            .map(|index| index as f64 / SUPPLIED_STATIONS as f64)
            .collect();
        let trim = crate::interpolate_curve(&stations, 3, &parameters)?;
        let sample_trim = |fraction: f64| trim.evaluate(fraction);
        let Ok(pcurve) = crate::build_pcurve_on_surface_stations(
            surface,
            &sample_trim,
            &[],
            pcurve_tol,
            SUPPLIED_STATIONS,
            3,
            513,
        ) else {
            self.supplied_report.declined_failed += 1;
            return Ok(None);
        };

        // Seat the supplied pcurve only when it is measurably AT LEAST AS GOOD
        // as the one projection would have derived, both judged by the kernel's
        // own coedge-image measurement against the edge's 3D curve.
        //
        // Verifying the supplied pcurve alone is not enough, and assuming it is
        // cost 4800x of round-trip fidelity before this was measured. A vendor
        // pcurve is only more authoritative than the 3D curve when the 3D curve
        // is itself a fit. Our OWN exporter is the counter-example that matters
        // most: it writes exact analytic `LINE`/`CIRCLE`/`ELLIPSE` 3D curves
        // and 2D pcurves fitted to within `export_knit` (4e-3 mm), so on every
        // file we produce the supplied pcurve is the approximation and
        // re-projecting the 3D curve is the accurate answer. Every round trip
        // through our writer goes through this lane, which makes that the
        // largest population it sees — larger than the vendor files it was
        // written for.
        //
        // Comparing rather than type-switching on the 3D curve's entity kind is
        // deliberate: a guard keyed on `LINE`/`CIRCLE` silently no-ops on every
        // fitted curve, and the question here is which curve is more accurate,
        // which is a measurement and not a classification.
        let band = crate::KernelTolerances::for_scale(scale, 1e-7).pcurve_consistency;
        let measure = |candidate: &NurbsCurve| {
            crate::measure_edge_against_pcurve_image(surface, candidate, &edge, forward, band)
        };
        let measured = measure(&pcurve);
        // The derived pcurve this would replace, FITTED rather than built: the
        // fit is read for its report whether or not it is on its bar, because
        // the question below is about the input, not the fit.
        let derived = crate::fit_pcurve_on_surface_range(
            surface,
            &edge.curve,
            edge.t0,
            edge.t1,
            forward,
            pcurve_tol,
        )
        .ok();
        let derived_deviation = derived
            .as_ref()
            .and_then(|fit| measure(&fit.curve).ok())
            .map(|measured| measured.deviation());
        // Does the edge's 3D curve lie on THIS carrier at all?
        //
        // The fit's `off_surface` is the curve's own distance from the surface
        // at every station the fit inverted — the standoff no pcurve can
        // remove, read directly from the input. Until 2026-09-26 this was read
        // off the derived pcurve's deviation instead, which is the same number
        // only while the fit tracks the curve, and a refused derived fit
        // (`None`) read as "on the carrier", the opposite of what it says.
        //
        // Comparing the two candidates instead is CIRCULAR and was measured to
        // be: the derived pcurve is a projection of the 3D curve, so it tracks
        // that curve by construction and wins under any "closer to the 3D
        // curve" test — including where the 3D curve is the thing that is
        // wrong. Scoring the vendor's answer by the projection's own objective
        // punished it for being right, and took the constructed fixture's
        // recovery from 94% to nothing.
        //
        // Where the 3D curve DOES lie on the carrier it is a legitimate trim
        // and projection is the accurate reading of it — that is every file our
        // own exporter writes, and it is why a round trip stays lossless. Where
        // it does NOT, it cannot define a trim on this face at all, and the
        // supplied pcurve, which lies on the carrier by construction, is the
        // better statement of where the boundary runs. A curve the derived
        // lane could not fit at all (`None`) is not known to be on the
        // carrier, and is not treated as if it were.
        let curve_is_on_carrier = derived
            .as_ref()
            .is_some_and(|fit| fit.report.off_surface <= CARRIER_INCIDENCE * scale);
        // Every offered coedge reports its whole measurement row, accepted or
        // not, under the trace switch. A criterion that cleanly separates the
        // cases someone chose to predict, while leaving a populated middle, is
        // a threshold wearing a mechanism's clothes — and only the whole
        // distribution can show which of the two this is. The row is what the
        // 2026-09-13 census reads.
        if supplied_tracing() {
            let (u_span, v_span) = supplied.span_in_periods();
            let span = |value: Option<f64>| {
                value.map_or_else(|| "-".to_string(), |value| format!("{value:.6}"))
            };
            supplied_trace(format_args!(
                "offer curve=#{curve_ref} basis=#{surface_ref} curve_off_carrier={:.6e} \
                 gate={:.6e} supplied={:.6e} snap={worst_vs_curve:.6e} scale={scale:.6e} \
                 band={band:.6e} corroboration={:.6e} branches={} curve3d={} surface={} \
                 uspan={} vspan={}",
                derived_deviation.unwrap_or(f64::NAN),
                CARRIER_INCIDENCE * scale,
                measured.as_ref().map(|m| m.deviation()).unwrap_or(f64::NAN),
                self.corroboration(&bundle, supplied, surface_ref, &edge, forward),
                bundle.len(),
                self.curve3d_kind(curve_ref),
                self.entity_kind(surface_ref),
                span(u_span),
                span(v_span),
            ));
        }
        match measured {
            Ok(measured) if measured.within_band() && !curve_is_on_carrier => {
                self.supplied_report.used += 1;
                self.supplied_report.worst_supplied_vs_curve = self
                    .supplied_report
                    .worst_supplied_vs_curve
                    .max(worst_vs_curve);
                Ok(Some(pcurve))
            }
            Ok(measured) if measured.within_band() => {
                // In band, but the edge's own 3D curve already lies on this
                // carrier, so it is the geometry of record for this trim and
                // projection reads it accurately. This is the ordinary outcome
                // on anything our own writer produced, and it is what keeps a
                // round trip through it lossless.
                self.supplied_report.declined_curve_on_carrier += 1;
                supplied_trace(format_args!(
                    "decline curve-on-carrier curve=#{curve_ref} basis=#{surface_ref} supplied={:.6e} curve_off_carrier={:.6e} gate={:.6e}",
                    measured.deviation(),
                    derived_deviation.unwrap_or(f64::NAN),
                    CARRIER_INCIDENCE * scale
                ));
                Ok(None)
            }
            other => {
                self.supplied_report.declined_residual += 1;
                // Keep the magnitude, not just the count: this is the only
                // place the lane ever sees how far a vendor's stated trim
                // departs from the 3D curve it bounds.
                if let Ok(measured) = other {
                    self.supplied_report.worst_declined_residual = self
                        .supplied_report
                        .worst_declined_residual
                        .max(measured.deviation());
                    self.supplied_report.declined_band = band;
                    supplied_trace(format_args!(
                        "decline residual curve=#{curve_ref} basis=#{surface_ref} deviation={:.6e} band={band:.6e} worst_vs_curve={worst_vs_curve:.6e}",
                        measured.deviation()
                    ));
                } else {
                    supplied_trace(format_args!(
                        "decline measure-failed curve=#{curve_ref} basis=#{surface_ref} worst_vs_curve={worst_vs_curve:.6e}"
                    ));
                }
                Ok(None)
            }
        }
    }

    /// How far this edge's OTHER supplied trim lands from the one being
    /// offered, station for station — the two 2D statements measured against
    /// each other rather than against the 3D curve.
    ///
    /// An edge bounds two faces, so a bundle usually supplies two pcurves on two
    /// different basis surfaces. Asking whether they agree with EACH OTHER
    /// better than either agrees with the 3D curve is the one question no
    /// single-candidate reading can answer, and it was measured as a
    /// discriminator and REFUTED: on our own writer's output the two pcurves are
    /// both fits of the same 3D curve sampled at the same stations, so they
    /// corroborate each other without being independent, and a harmed coedge of
    /// `abc_00000014` corroborates BETTER than the constructed fixture's
    /// recovering one (24.9x against 21.3x). It is a trace column and nothing
    /// more.
    ///
    /// `NaN` when there is no second opinion: a bundle with one pcurve, or a
    /// `SEAM_CURVE`, whose two branches name the SAME basis surface and are the
    /// same locus by construction. "No reading" must not read as "agrees
    /// perfectly", which is why this is NaN and not zero.
    fn corroboration(
        &self,
        bundle: &[SuppliedPcurve],
        supplied: &SuppliedPcurve,
        surface_ref: usize,
        edge: &EdgeRecord,
        forward: bool,
    ) -> f64 {
        let Some(other) = bundle
            .iter()
            .find(|candidate| candidate.basis_ref != surface_ref)
        else {
            return f64::NAN;
        };
        let domain = edge.t1 - edge.t0;
        let mut worst = 0.0f64;
        let mut cursor: Option<(f64, f64)> = None;
        let mut other_cursor: Option<(f64, f64)> = None;
        for index in 0..=SUPPLIED_STATIONS {
            let fraction = index as f64 / SUPPLIED_STATIONS as f64;
            let edge_fraction = if forward { fraction } else { 1.0 - fraction };
            let Ok(point) = edge.curve.evaluate(edge.t0 + domain * edge_fraction) else {
                return f64::NAN;
            };
            let (Ok((uv, on_trim)), Ok((other_uv, on_other))) =
                (supplied.snap(point, cursor), other.snap(point, other_cursor))
            else {
                return f64::NAN;
            };
            cursor = Some(uv);
            other_cursor = Some(other_uv);
            worst = worst.max(on_other.sub(on_trim).length());
        }
        worst
    }

    /// The entity kind of the 3D curve a `SURFACE_CURVE`/`SEAM_CURVE` bundle
    /// wraps — the representation the supplied pcurve is offered AGAINST.
    fn curve3d_kind(&self, curve_ref: usize) -> String {
        let Ok(entity) = self.resolver.get(curve_ref) else {
            return "?".to_string();
        };
        let curve = ["SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"]
            .iter()
            .find_map(|wrapper| entity.find(wrapper))
            .and_then(|args| args.get(1))
            .and_then(|value| value.as_ref_id().ok());
        curve.map_or_else(|| "?".to_string(), |id| self.entity_kind(id))
    }

    /// One name for an entity's record, preferring the geometric type over the
    /// supertypes a complex instance also carries: a rational spline's record
    /// is `(BOUNDED_CURVE()B_SPLINE_CURVE(..)..)`, and "B_SPLINE_CURVE" is the
    /// answer a reader wants from it.
    fn entity_kind(&self, id: usize) -> String {
        const PREFERRED: [&str; 16] = [
            "LINE",
            "CIRCLE",
            "ELLIPSE",
            "PARABOLA",
            "HYPERBOLA",
            "POLYLINE",
            "TRIMMED_CURVE",
            "B_SPLINE_CURVE_WITH_KNOTS",
            "B_SPLINE_CURVE",
            "PLANE",
            "CYLINDRICAL_SURFACE",
            "CONICAL_SURFACE",
            "SPHERICAL_SURFACE",
            "TOROIDAL_SURFACE",
            "B_SPLINE_SURFACE_WITH_KNOTS",
            "B_SPLINE_SURFACE",
        ];
        let Ok(entity) = self.resolver.get(id) else {
            return "?".to_string();
        };
        for name in PREFERRED {
            if entity.has(name) {
                // A rational spline carries both the WITH_KNOTS record and the
                // plain one; the first match in this order is the specific one.
                return name.to_string();
            }
        }
        entity
            .records
            .first()
            .map_or_else(|| "?".to_string(), |(name, _)| name.clone())
    }

    /// Emit the per-body supplied-pcurve tally when asked for it.
    pub(super) fn report_supplied_pcurves(&self) {
        if std::env::var("BREP_DEBUG_SUPPLIED_PCURVE").is_err() {
            return;
        }
        let report = &self.supplied_report;
        eprintln!(
            "SUPPLIED PCURVE offered={} used={} declined(endpoints={} residual={} failed={} curve_on_carrier={}) worst_supplied_vs_curve={:.6e} worst_declined_residual={:.6e} band={:.6e}",
            report.offered,
            report.used,
            report.declined_endpoints,
            report.declined_residual,
            report.declined_failed,
            report.declined_curve_on_carrier,
            report.worst_supplied_vs_curve,
            report.worst_declined_residual,
            report.declined_band,
        );
    }
}

/// One line of per-coedge supplied-pcurve trace, under the same switch the
/// per-body tally uses. Every decline names the STEP entities behind it, so a
/// refusal can be attributed to the file rather than assumed to be the file's.
fn supplied_trace(args: std::fmt::Arguments<'_>) {
    if supplied_tracing() {
        eprintln!("SUPPLIED PCURVE {args}");
    }
}

/// Whether the per-coedge trace is on. Read before COMPUTING a trace column and
/// not merely before printing one: the corroboration reading snaps every station
/// onto a second branch, which is real work in the lane's hot loop and exists
/// only to be measured.
fn supplied_tracing() -> bool {
    std::env::var("BREP_DEBUG_SUPPLIED_PCURVE").is_ok()
}

/// `BREP_SUPPLIED_PCURVES=1` turns this lane ON, `=0` or unset leaves it OFF,
/// and OFF is the measurement rather than caution.
///
/// The lane reads the vendor's stated trim correctly — that part is verified,
/// and on a file whose 3D curve has genuinely drifted off its own carrier it
/// recovers 94% of the volume error the projection fit inherits. What is missing
/// is a way to tell that file from the ones where seating the supplied pcurve
/// makes things worse, and that question has now been asked of a whole corpus
/// rather than of one fixture: 62 STEP inputs through both lanes in two
/// populations, 56,176 offered coedges, and a round trip through our own writer
/// for every one of them, because our writer emits pcurves and is therefore the
/// largest producer of this lane's inputs. 39 of the 62 round trips are WORSE
/// under the lane and 6 are better.
///
/// Every quantity that could decide it was measured per coedge — the 3D curve's
/// distance from the carrier, the supplied trim's distance from the 3D curve,
/// each against the file's own stated precision, the trim's parameter span
/// against the surface domain, the two entity kinds, and whether the edge's two
/// supplied trims corroborate each other better than either matches the 3D
/// curve. On every one of them the benefiting coedge sits INSIDE the harmed
/// distribution, and the tightest rule that keeps it still seats a coedge of
/// `abc_00000014` that corroborates BETTER than it does (24.9x against 21.3x).
/// Zero false positives is reachable only by a conjunction whose constants are
/// that one coedge's own coordinates.
///
/// The harm is not hypothetical and is not confined to vendor files: seating a
/// fitted pcurve over an exact analytic 3D curve cost a round trip a factor of
/// 4800 in volume fidelity before the carrier test was added, and 262,000x on
/// `ttt-CAD323` with it.
///
/// What WOULD decide it is named in the record and is not this module's: our
/// writer states a precision of 1e-6 mm while verifying its pcurves only to
/// `export_knit` (at least 4e-3 mm), and the measured round-trip snap never
/// exceeds that band (3.908e-3 against 4e-3) because a pcurve that misses it is
/// omitted rather than written. A writer that stated its real band would make
/// "the trim disagrees with the 3D curve by more than the file claims" seat zero
/// round-trip coedges with no fitted constant at all — a change to `io/step`.
fn supplied_pcurves_enabled() -> bool {
    std::env::var("BREP_SUPPLIED_PCURVES").is_ok_and(|value| value != "0")
}
