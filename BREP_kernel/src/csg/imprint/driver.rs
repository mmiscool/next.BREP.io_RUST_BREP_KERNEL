use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass, MAX_FIT_STATIONS};

/// The degree a marched section is interpolated at (2026-09-27). The march
/// puts every station on both carriers; the fit decides how far the curve
/// strays BETWEEN them, and a cubic's O(h^4) there left `BadBoolean`'s
/// sphere×bore rim 2.5e-7 off the bore with a mean bias of −1.2e-8 that both
/// trims inherited (+1.14e-6 on the solid). A quintic through the same
/// stations reads 8.5e-9 and +6e-11 with the same knots and control-point
/// count. More stations bought the same accuracy on 2026-09-26 and were
/// reverted for their cost: the watertight tessellator samples every interior
/// knot, so every scan downstream scales with station count, and a degree does
/// not add knots. `BREP_SECTION_FIT_DEGREE=3` restores the cubic. The gate on
/// the mid-span refinement, as a multiple of the fit tolerance (2026-09-27): a
/// marched section whose unrefined fit misses a carrier between stations by
/// more than this is UNDERSAMPLED — the 20° cylinder crossing of
/// `fillet-skew-cylinder-seam-split-20deg` sits 4e-5..8e-5 off, 470 times the
/// tolerance, at any degree — and only such a section is given true
/// intersection points until its mid-spans hold. Refining every section cost
/// more downstream than it returned (the tessellator samples every knot).
/// `BREP_SECTION_REFINE_GATE` sets the multiple (`on` for
/// [`SECTION_REFINE_GATE`]). OFF BY DEFAULT: at k = 100 the sequential
/// blend/direct_edit/imprint/boolean lib set costs +64 % and
/// `oblique_multi_rim_cone_cap_tearing_push_refuses` goes red; k = 1000 costs
/// +32 % and leaves the skew union +5.3e-4 off. This global switch stays off.
/// Independently, a pair of grazing crossings on one trim edge requests a
/// fit-floor check for that face pair below: those two nearby junctions and
/// the intervening cap must survive the fit.
fn section_refine_gate() -> Option<f64> {
    static GATE: std::sync::OnceLock<Option<f64>> = std::sync::OnceLock::new();
    *GATE.get_or_init(|| match std::env::var("BREP_SECTION_REFINE_GATE") {
        Ok(value) if value == "on" => Some(SECTION_REFINE_GATE),
        Ok(value) => value.parse::<f64>().ok().filter(|gate| *gate >= 0.0),
        Err(_) => None,
    })
}

/// The multiple `BREP_SECTION_REFINE_GATE=on` uses: the smallest measured
/// that trips the skew union's undersampled sections and leaves fixture 22 as
/// it was (k = 10 flips it).
const SECTION_REFINE_GATE: f64 = 100.0;

/// A clipped run's END span can be a sliver: the clip endpoint lands beside a
/// march station, a spacing ratio to the run's median of 0.011 on
/// `anotherBooleanFail`, and a global interpolant rings next to such a jump —
/// the mid-spans near the ends miss their carriers by 1e-3, and bisecting them
/// only makes the spacing more uneven (2026-09-27). Below
/// `SLIVER_END_FRACTION` of the median spacing the station beside the endpoint
/// is RE-SOLVED at the middle of the merged gap as a fresh intersection point
/// (`BREP_SECTION_SLIVER_END=resolve`) or dropped (`drop`). OFF BY DEFAULT:
/// resolve evens every `anotherBooleanFail` section but reddens three
/// fixture-25 pins, one a closure residual 2.32e-3 against its 1.41e-4 bar.
/// Returns how many ends were evened.
fn even_sliver_ends(
    section: &mut Vec<Vec3>,
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<usize, String> {
    static MODE: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    let mode = MODE.get_or_init(|| std::env::var("BREP_SECTION_SLIVER_END").unwrap_or_default());
    if !(mode == "drop" || mode == "resolve") {
        return Ok(0);
    }
    let mut evened = 0;
    // One sliver at a time: each removal changes the spans it sits between.
    for _ in 0..4 {
        let count = section.len();
        if count < 6 {
            break;
        }
        let mut chords: Vec<f64> = section.windows(2).map(|pair| pair[1].sub(pair[0]).length()).collect();
        let lengths = chords.clone();
        chords.sort_by(f64::total_cmp);
        let median = chords[chords.len() / 2];
        let spans = lengths.len();
        let near_ends = [0, 1, 2, spans - 3, spans - 2, spans - 1];
        let Some(&sliver) = near_ends
            .iter()
            .filter(|&&span| lengths[span] < sliver_end_fraction() * median)
            .min_by(|a, b| lengths[**a].total_cmp(&lengths[**b]))
        else {
            break;
        };
        // The sliver's bounding stations are `sliver` and `sliver + 1`; a run
        // endpoint is never moved. Of the interior ones, remove the one whose
        // removal leaves the shorter merged span.
        let mut candidates = Vec::new();
        for station in [sliver, sliver + 1] {
            if station == 0 || station == count - 1 {
                continue;
            }
            let merged = section[station + 1].sub(section[station - 1]).length();
            candidates.push((station, merged));
        }
        let Some(&(station, _)) = candidates.iter().min_by(|a, b| a.1.total_cmp(&b.1)) else {
            break;
        };
        if mode == "resolve" {
            let (before, after) = (section[station - 1], section[station + 1]);
            let seed = before.add(after).scale(0.5);
            let reach = 0.5 * after.sub(before).length();
            match crate::surface_surface_intersection::refine_to_intersection(first, second, seed, tolerance, reach)? {
                Some(point) => section[station] = point,
                None => {
                    section.remove(station);
                }
            }
        } else {
            section.remove(station);
        }
        evened += 1;
    }
    Ok(evened)
}

/// At 0.25 six of `anotherBooleanFail`'s eight sections stay NoProgress
/// (worst miss 2.7e-4); at 0.5 all eight are evened and fall below the
/// refinement gate (worst miss 2.8e-3 -> 5.4e-8).
const SLIVER_END_FRACTION: f64 = 0.5;

/// `BREP_SECTION_SLIVER_FRACTION` overrides [`SLIVER_END_FRACTION`] for the
/// measurement that picks it.
fn sliver_end_fraction() -> f64 {
    static FRACTION: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *FRACTION.get_or_init(|| {
        std::env::var("BREP_SECTION_SLIVER_FRACTION")
            .ok()
            .and_then(|value| value.parse::<f64>().ok())
            .filter(|fraction| *fraction > 0.0 && *fraction < 1.0)
            .unwrap_or(SLIVER_END_FRACTION)
    })
}

fn section_fit_degree() -> usize {
    static DEGREE: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
    *DEGREE.get_or_init(|| {
        std::env::var("BREP_SECTION_FIT_DEGREE")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .filter(|degree| (1..=7).contains(degree))
            .unwrap_or(5)
    })
}

/// A marched section is held to the fit tolerance against the CARRIERS, not
/// only against the polyline it was fitted through (2026-09-26). The march
/// refines every station onto both surfaces to `tolerance * 0.01`, but the
/// cubic interpolant between stations is measured by `fit_polyline` against the
/// polyline's own chords, which are further from the true curve than the
/// interpolant is — so a systematic bias of the interpolant (2.8e-8 mean,
/// 4.7e-7 worst, inside the bore on `BadBoolean`'s sphere/bore rim at 135
/// stations) passed every check the fit could make, and both trims inherited it
/// (+1.14e-6 on the solid's volume, against two independent readings agreeing
/// to 1e-8). The check below evaluates the fitted curve at every mid-span,
/// measures it against both carriers, and where it misses `fit tolerance *
/// SECTION_MIDSPAN_FRACTION` inserts a TRUE intersection point there and
/// refits. The interpolant's bias falls as h^4, so one round usually suffices.
/// A round that neither halves the worst miss nor lowers the count of spans
/// missing is not in that regime — a carrier's C0 crease puts a corner in the
/// section (fixture 25's pierce solid), where every round only doubles the
/// stations — so that round is discarded and the loop stops. It is also capped
/// in rounds and stations, and names how it left.
/// `BREP_SECTION_REFINE_GATE=off` switches it off, for the before/after reading
/// and nothing else.
pub(super) const SECTION_MIDSPAN_FRACTION: f64 = 0.1;
const SECTION_MIDSPAN_ROUNDS: usize = 4;
/// A round earns another if it cuts the worst mid-span miss by this factor
/// (h^4 promises 16 for a halved spacing) OR lowers the count of spans
/// missing: near the floor a few isolated spans hold the worst while the count
/// falls, and on a corner the count GROWS round on round.
const SECTION_MIDSPAN_PROGRESS: f64 = 0.5;

/// Squared distance from `point` to the segment `from`-`to`.
fn distance_to_segment_sq(point: Vec3, from: Vec3, to: Vec3) -> f64 {
    let d = to.sub(from);
    let l2 = d.dot(d);
    let t = if l2 > 0.0 { (point.sub(from).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
    let q = from.add(d.scale(t));
    point.sub(q).dot(point.sub(q))
}

/// How a mid-span refinement left (`SECTION_MIDSPAN_*`), for the pair trace.
#[derive(Clone, Copy, Debug, PartialEq)]
enum MidspanExit {
    /// Every mid-span within the floor of both carriers.
    Converged,
    /// The unrefined fit's worst mid-span is within the gate: the section is
    /// sampled well enough that its degree carries it, and stations would
    /// cost more downstream than they return.
    BelowGate,
    /// Some mid-span still misses and no inserted point could improve it
    /// (the Newton failed, wandered off the span, or landed on a station
    /// already present).
    Stalled,
    /// A round neither halved the worst miss nor lowered the count of spans
    /// missing: not the h^4 regime, so the section has a corner the
    /// interpolant cannot follow. That round's fit is discarded.
    NoProgress,
    RoundBudget,
    StationCeiling,
}

/// Measure `fit` at every mid-span against both carriers; return the worst
/// distance, the count of spans that miss `floor` and, for each of them the
/// Newton could place, the refined intersection point to insert (with the
/// span's index).
fn midspan_misses(
    fit: &PolylineFit,
    first: &NurbsSurface,
    second: &NurbsSurface,
    floor: f64,
    tolerance: f64,
) -> Result<(f64, usize, Vec<(usize, Vec3)>), String> {
    let mut worst = 0.0f64;
    let mut missing = 0usize;
    let mut inserts = Vec::new();
    for span in 0..fit.parameters.len().saturating_sub(1) {
        let mid = 0.5 * (fit.parameters[span] + fit.parameters[span + 1]);
        let point = fit.curve.evaluate(mid)?;
        let off = project_point_to_surface(first, point)?
            .distance
            .max(project_point_to_surface(second, point)?.distance);
        worst = worst.max(off);
        if off <= floor {
            continue;
        }
        missing += 1;
        // The refined point must stay on THIS span: half its chord is the reach.
        let reach = 0.5 * fit.kept[span + 1].sub(fit.kept[span]).length();
        if let Some(refined) = crate::surface_surface_intersection::refine_to_intersection(
            first, second, point, tolerance, reach,
        )? {
            inserts.push((span, refined));
        }
    }
    Ok((worst, missing, inserts))
}

/// Refit `section` until its mid-spans sit within `floor` of both carriers,
/// inserting true intersection points where they do not. Returns the fit that
/// stands, the worst mid-span miss it still carries, the unrefined fit's worst
/// (what the gate read), the rounds spent and the exit.
fn refine_section_against_carriers(
    section: &mut Vec<Vec3>,
    fit: PolylineFit,
    first: &NurbsSurface,
    second: &NurbsSurface,
    fit_tolerance: f64,
    tolerance: f64,
    local_fit: bool,
    gate: f64,
) -> Result<(PolylineFit, f64, f64, usize, MidspanExit), String> {
    let floor = fit_tolerance * SECTION_MIDSPAN_FRACTION;
    let mut unrefined = f64::NAN;
    let mut fit = fit;
    let mut rounds = 0usize;
    let mut previous: Option<(PolylineFit, f64, usize)> = None;
    loop {
        let (worst, missing, inserts) = midspan_misses(&fit, first, second, floor, tolerance)?;
        if rounds == 0 {
            unrefined = worst;
        }
        if worst <= floor {
            return Ok((fit, worst, unrefined, rounds, MidspanExit::Converged));
        }
        if rounds == 0 && worst <= gate {
            return Ok((fit, worst, unrefined, rounds, MidspanExit::BelowGate));
        }
        if let Some((earlier, earlier_worst, earlier_missing)) = previous.take() {
            // A round that did not earn its keep is discarded whole: the fit
            // of the last round that did stands (on a corner, the unrefined
            // fit), not whichever of the two happens to read a smaller max.
            if worst > earlier_worst * SECTION_MIDSPAN_PROGRESS && missing >= earlier_missing {
                return Ok((earlier, earlier_worst, unrefined, rounds, MidspanExit::NoProgress));
            }
        }
        if inserts.is_empty() {
            return Ok((fit, worst, unrefined, rounds, MidspanExit::Stalled));
        }
        if rounds >= SECTION_MIDSPAN_ROUNDS {
            return Ok((fit, worst, unrefined, rounds, MidspanExit::RoundBudget));
        }
        if section.len() + inserts.len() > MAX_FIT_STATIONS {
            return Ok((fit, worst, unrefined, rounds, MidspanExit::StationCeiling));
        }
        // Insert each refined point into the run between its span's stations,
        // on the nearest segment of the run there. Positions are located
        // against the run BEFORE any insertion of this round, then applied
        // from the back so earlier indices stay valid. The kept stations are
        // found by a FORWARD search from the previous one: a closed section
        // ends on its first station, and a search from the start would put
        // the last span's far end at index 0 and skip that span forever.
        let station_index = |point: Vec3, from: usize| -> Option<usize> {
            section[from..]
                .iter()
                .position(|p| p.sub(point).length() <= 1e-15 * (1.0 + point.length()))
                .map(|offset| from + offset)
        };
        let mut placed: Vec<(usize, Vec3)> = Vec::new();
        for (span, point) in inserts {
            let Some(lo) = station_index(fit.kept[span], 0) else {
                continue;
            };
            let Some(hi) = station_index(fit.kept[span + 1], lo + 1) else {
                continue;
            };
            if hi <= lo {
                continue;
            }
            if section[lo..=hi].iter().any(|p| p.sub(point).length() <= floor) {
                continue; // already a station: inserting it again refines nothing
            }
            let mut best = (lo, f64::INFINITY);
            for index in lo..hi {
                let d2 = distance_to_segment_sq(point, section[index], section[index + 1]);
                if d2 < best.1 {
                    best = (index, d2);
                }
            }
            placed.push((best.0 + 1, point));
        }
        if placed.is_empty() {
            return Ok((fit, worst, unrefined, rounds, MidspanExit::Stalled));
        }
        placed.sort_by(|a, b| b.0.cmp(&a.0));
        for (at, point) in placed {
            section.insert(at, point);
        }
        // Every station of the denser run is entitled to be an interpolation
        // station: the ladder starts at the run's own count.
        let maximum_points = section.len().min(MAX_FIT_STATIONS);
        let refit = fit_polyline_of_degree(section, fit_tolerance, maximum_points, local_fit, section_fit_degree())?;
        previous = Some((std::mem::replace(&mut fit, refit), worst, missing));
        rounds += 1;
    }
}
use super::*;
use super::gate_census::{census_pair, fit_census_enabled};

pub fn build_imprints(
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    options: &ImprintOptions,
) -> Result<ImprintResultRecord, KernelRefusal> {
    build_imprints_impl(solid_a, solid_b, options, false)
}

/// Imprint temporary open offset sheets. A section riding one sheet's trim
/// still cuts the other sheet when no boundary copy supplied that section.
pub(crate) fn build_carrier_imprints(
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    options: &ImprintOptions,
) -> Result<ImprintResultRecord, KernelRefusal> {
    build_imprints_impl(solid_a, solid_b, options, true)
}

fn build_imprints_impl(
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    options: &ImprintOptions,
    open_carriers: bool,
) -> Result<ImprintResultRecord, KernelRefusal> {
    let mut section_evidence = false;
    let edges = edge_map(solid_a, solid_b);
    let mut builder = ImprintBuilder {
        open_carriers,
        edges,
        tolerance: options.tolerance,
        barrier_edges: HashSet::default(),
        overlap_ridden_edges: HashSet::default(),
        scale: solid_scale(solid_a).max(solid_scale(solid_b)),
        vertices: Vec::new(),
        vertex_radii: HashMap::default(),
        pieces: Vec::new(),
        by_face: HashMap::default(),
        edge_splits: HashMap::default(),
        marched_pieces: HashSet::default(),
        next_id: 1,
    };
    let mut first_faces = faces(solid_a, 0);
    let mut second_faces = faces(solid_b, 1);
    // Per-face trim-window carriers, computed once: tighter BVH bounds, and
    // the seed/march stages walk the window instead of the full carrier
    // (identical geometry and parameterization inside the window).
    //
    // A window LIFTED across a closed direction's seam (`restricted_carrier`)
    // is the exception to "same parameterization": it is the trim's frame, a
    // period off the carrier's on one side. Its face carries it as its
    // `chart`, and every uv the imprint reads for that face — the clip's
    // containment, the sections' pcurves, a seed's normal — is read there.
    let first_restricted: Vec<Option<MarchWindow>> = first_faces
        .iter()
        .map(|tagged| restricted_carrier(tagged.face))
        .collect();
    let second_restricted: Vec<Option<MarchWindow>> = second_faces
        .iter()
        .map(|tagged| restricted_carrier(tagged.face))
        .collect();
    for (tagged, window) in first_faces.iter_mut().zip(&first_restricted) {
        tagged.lifted = window.as_ref().filter(|window| window.lifted).map(|window| &window.surface);
    }
    for (tagged, window) in second_faces.iter_mut().zip(&second_restricted) {
        tagged.lifted = window.as_ref().filter(|window| window.lifted).map(|window| &window.surface);
    }
    let first_faces = first_faces;
    let second_faces = second_faces;
    // The lifted charts, by key, for the post-passes that rebuild a pcurve
    // after the mint (junction canonicalization, the origin dissolve, the
    // truncation bridge). A face absent here is read on its own carrier.
    let charts: FaceCharts<'_> = first_faces
        .iter()
        .chain(second_faces.iter())
        .filter_map(|tagged| tagged.lifted.map(|surface| (tagged.key(), surface)))
        .collect();
    let first_bounds = first_faces
        .iter()
        .zip(&first_restricted)
        .map(|(tagged, restricted)| {
            face_bounds(
                tagged.face,
                restricted.as_ref().map(|window| &window.surface),
                &builder.edges,
                tagged.operand,
                options.tolerance,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let second_bounds = second_faces
        .iter()
        .zip(&second_restricted)
        .map(|(tagged, restricted)| {
            face_bounds(
                tagged.face,
                restricted.as_ref().map(|window| &window.surface),
                &builder.edges,
                tagged.operand,
                options.tolerance,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let second_bvh = Bvh::build(&second_bounds);
    // Per-face classifier data and edge lists/subcurves, computed once and
    // reused across every pair the face participates in.
    let first_classify = first_faces
        .iter()
        .map(|tagged| SurfaceClassifyData::build(&tagged.face.surface, options.tolerance))
        .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
    let second_classify = second_faces
        .iter()
        .map(|tagged| SurfaceClassifyData::build(&tagged.face.surface, options.tolerance))
        .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
    let mut face_edge_lists: HashMap<FaceKey, Vec<&EdgeRecord>> = HashMap::default();
    for tagged in first_faces.iter().chain(second_faces.iter()) {
        face_edge_lists.insert(tagged.key(), face_edges(*tagged, &builder.edges)?);
    }
    let mut subcurves: HashMap<(u8, u64), NurbsCurve> = HashMap::default();
    for (&key, edge) in &builder.edges {
        if !edge.degenerate {
            // Failures fall through: the use sites recompute and surface the
            // original error exactly where the uncached code did.
            if let Ok(curve) = edge_subcurve(edge) {
                subcurves.insert(key, curve);
            }
        }
    }
    let cached_subcurve = |operand: u8, edge: &EdgeRecord| -> Result<NurbsCurve, KernelRefusal> {
        match subcurves.get(&(operand, edge.id)) {
            Some(curve) => Ok(curve.clone()),
            None => edge_subcurve(edge),
        }
    };
    let mut profile = ImprintProfile::new();
    let mut tangent_nodes: Vec<Vec3> = Vec::new();
    let mut cosurface_pairs: Vec<(FaceKey, FaceKey)> = Vec::new();
    let mut paired = Vec::new();
    for (first_index, first) in first_faces.iter().enumerate() {
        let first = *first;
        paired.clear();
        second_bvh.overlapping(
            first_bounds[first_index],
            options.tolerance * 100.0,
            &mut paired,
        );
        paired.sort_unstable();
        let debug_pairs = std::env::var("BREP_DEBUG_PAIRS").is_ok();
        if debug_pairs {
            eprintln!(
                "bvh first_face={} -> {} candidate pairs {:?}",
                first.face.id,
                paired.len(),
                paired
                    .iter()
                    .map(|&index| second_faces[index].face.id)
                    .collect::<Vec<_>>()
            );
        }
        for &second_index in &paired {
            let second = second_faces[second_index];
            profile.pairs += 1;
            // Set when this pair's section passes through an isolated TANGENT
            // NODE inside both trims (see the classification below). The march
            // is attempted anyway — that is the whole point — but a marcher
            // that cannot get through a node must refuse in the tangent-node
            // class naming the node, not leak its own step-budget message.
            let mut tangent_node: Option<Vec3> = None;
            let mut lap_start = None;
            profile.lap(&mut lap_start);
            let pair_classification = classify_surface_pair_cached(
                &first.face.surface,
                &first_classify[first_index],
                &second.face.surface,
                &second_classify[second_index],
                options.tolerance,
                PAIR_ANGULAR_TOLERANCE,
            ).or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
            profile.classify += profile.lap(&mut lap_start);
            if pair_classification.relation == SurfacePairRelation::Disjoint {
                if debug_pairs {
                    eprintln!(
                        "pair {}x{}: DISJOINT-cull sep={:.4e}",
                        first.face.id, second.face.id, pair_classification.minimum_separation
                    );
                }
                continue;
            }
            // Coincident carriers are never marched: their true intersection
            // is a 2D region, not a curve, so anything the marcher traces on
            // them is noise ("similar faces we do not intersect" — Golovanov
            // §6.2).  The sampled classification catches coincident pairs
            // the strict reconstruction test misses (partial overlaps,
            // differing parameterizations); both route to the boundary-curve
            // exchange below.
            let is_cosurface = pair_classification.relation == SurfacePairRelation::Cosurface
                || cosurface_pair(&first.face.surface, &second.face.surface, options.tolerance)?;
            profile.cosurface += profile.lap(&mut lap_start);
            if is_cosurface {
                if debug_pairs {
                    eprintln!("pair {}x{}: cosurface", first.face.id, second.face.id);
                }
                cosurface_pairs.push((first.key(), second.key()));
                for edge in &face_edge_lists[&second.key()] {
                    if !edge.degenerate {
                        builder.process_curve(
                            cached_subcurve(second.operand, edge)?,
                            first,
                            second,
                            &[first],
                            &[first],
                            false,
                        )?;
                    }
                }
                for edge in &face_edge_lists[&first.key()] {
                    if !edge.degenerate {
                        builder.process_curve(
                            cached_subcurve(first.operand, edge)?,
                            first,
                            second,
                            &[second],
                            &[second],
                            false,
                        )?;
                    }
                }
                profile.process_curve += profile.lap(&mut lap_start);
                continue;
            }

            // The exact section lanes below (planar-iso, analytic) mint this
            // pair's whole intersection, the part over either trim included, so
            // a SPAN of an edge lying on the other face would be a second copy
            // of a curve they already mint — which reorders the face's pieces
            // and with them its fragments' names (the 2026-09-10 chamfer groove
            // rim's `Box_PX` / `Box_PX_1`). Where they answer, only a whole
            // edge is exchanged, as before.
            let planar_iso = planar_iso_intersection(
                &first.face.surface,
                &second.face.surface,
                options.tolerance,
            )?;
            profile.planar_iso += profile.lap(&mut lap_start);
            let analytic = crate::intersect_analytic_pair(
                &first.face.surface,
                &second.face.surface,
                options.tolerance,
            );
            profile.analytic += profile.lap(&mut lap_start);
            let exact_section = planar_iso.is_some() || analytic.is_some();

            for edge in &face_edge_lists[&first.key()] {
                if !edge.degenerate {
                    let curve = cached_subcurve(first.operand, edge)?;
                    let spans = curve_spans_on_face(curve, second.face, &face_edge_lists[&second.key()], options.tolerance, exact_section)?;
                    for span in spans {
                        builder.process_curve(span, first, second, &[second], &[second], false)?;
                    }
                }
            }
            for edge in &face_edge_lists[&second.key()] {
                if !edge.degenerate {
                    let curve = cached_subcurve(second.operand, edge)?;
                    let spans = curve_spans_on_face(curve, first.face, &face_edge_lists[&first.key()], options.tolerance, exact_section)?;
                    for span in spans {
                        builder.process_curve(span, first, second, &[first], &[first], false)?;
                    }
                }
            }
            profile.lies_on += profile.lap(&mut lap_start);
            // CENSUS ONLY (`BREP_FIT_CENSUS=1`): the lane this pair takes under
            // the count-keyed and the geometry-keyed gates, and — when it is
            // marched here and answered exactly there — the point-set comparison
            // of the two, printed when `pair_census` is dropped. `imprint/gate_census.rs`.
            let mut pair_census = if fit_census_enabled() {
                census_pair(first, second, options.tolerance)?
            } else {
                None
            };
            if let Some(curve) = planar_iso {
                if debug_pairs {
                    eprintln!("pair {}x{}: planar_iso", first.face.id, second.face.id);
                }
                builder.process_curve(
                    curve,
                    first,
                    second,
                    &[first, second],
                    &[first, second],
                    true,
                )?;
                profile.process_curve += profile.lap(&mut lap_start);
                continue;
            }

            // Recognized analytic pairs produce their exact intersection
            // curves (lines, circles, ellipses) directly — no marching, no
            // polyline fitting, no chord-sag drift. An empty result is a
            // proof of non-intersection and also skips the marcher. (Read
            // above, with the planar-iso curve.)
            if let Some(curves) = analytic {
                if debug_pairs {
                    eprintln!(
                        "pair {}x{}: analytic x{}",
                        first.face.id,
                        second.face.id,
                        curves.len()
                    );
                }
                for curve in curves {
                    builder.process_curve(
                        curve,
                        first,
                        second,
                        &[first, second],
                        &[first, second],
                        true,
                    )?;
                }
                profile.process_curve += profile.lap(&mut lap_start);
                continue;
            }

            // A pair whose every touching sample is TANGENTIAL cannot
            // contain a transverse intersection curve: the marcher would
            // walk the tangency band's noise, which neither closes nor
            // reaches a boundary (the distance-0 glue-extrude runaway).
            // Shared topology at a tangential contact comes from the
            // boundary-curve exchange above ("similar faces we do not
            // intersect" extended to tangential contacts — Golovanov §6.2;
            // where surfaces CROSS through a tangency, off-line samples
            // have non-parallel normals and the pair still marches).
            if pair_classification.relation == SurfacePairRelation::NearTangent
                && pair_classification.tangential_only
            {
                // The coarse 5×5 classifier can flag `tangential_only` off an
                // incidental tangential KISS between two curved carriers and
                // miss the transverse loop where they actually cross (two
                // overlapping tori: their tubes cross while their inner walls
                // just touch). Before honouring the skip, a GATED supplemental
                // detector (denser two-sided seeding, tangency-band seeds
                // rejected, trace-exhaustion swallowed, transverse-only
                // branches) checks whether a genuine transverse curve of
                // meaningful length lies inside BOTH trims.
                //
                // Such a pair's intersection is SINGULAR where a tangency
                // sits on it, and what the imprint can do about that depends
                // entirely on the SHAPE of the tangency — see
                // `imprint/tangent_contact.rs`. An isolated NODE (two branches
                // crossing, as every equal-radius pair produces) is already
                // assembled correctly by the ordinary march plus the 2D
                // arrangement's pinch carving; an EXTENDED contact along a
                // whole curve has no section to imprint at all and tears the
                // shell. Both are classified below and only the second is
                // refused. Skipping either silently would DROP the
                // intersection (torus∪torus double-counts; torus−torus removes
                // nothing — the two structural bugs the semantic oracle
                // found), so nothing here ever falls through quietly.
                let first_march = first_restricted[first_index]
                    .as_ref()
                    .map(|window| &window.surface)
                    .unwrap_or(&first.face.surface);
                let second_march = second_restricted[second_index]
                    .as_ref()
                    .map(|window| &window.surface)
                    .unwrap_or(&second.face.surface);
                let supplemental = intersect_surfaces_supplemental(
                    first_march,
                    second_march,
                    &SurfaceIntersectionOptions {
                        tolerance: options.tolerance,
                        maximum_step: options.maximum_ssi_step,
                        ..Default::default()
                    },
                ).or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
                let mut dropped_length = 0.0f64;
                let mut clipped: Vec<Vec<Vec3>> = Vec::new();
                for branch in &supplemental {
                    for run in clip_branch_to_trims(&branch.points, first, second)? {
                        if run.len() < 2 {
                            continue;
                        }
                        let length: f64 = run
                            .windows(2)
                            .map(|pair| pair[1].sub(pair[0]).length())
                            .sum();
                        dropped_length = dropped_length.max(length);
                        clipped.push(run);
                    }
                }
                if dropped_length <= options.tolerance * 100.0 {
                    if debug_pairs {
                        eprintln!(
                            "pair {}x{}: tangential-only contact, march skipped",
                            first.face.id, second.face.id
                        );
                    }
                    continue;
                }
                // A transverse curve EXISTS. What would make it unimprintable is
                // a tangent NODE on it — and the node need not lie inside the
                // trims. Two equal-radius pipe arms are tangent to each other
                // where their axes' common perpendicular leaves the junction,
                // and a joint ball wider than the arms trims that crotch off
                // both faces; what is left inside the trims is an ordinary
                // crossing. So the question is not "does a transverse curve
                // exist" but "does the curve INSIDE BOTH TRIMS reach a
                // tangency". Only the latter is the checkerboard case.
                let mut tangency_inside_trims = false;
                let mut tangency_witness = Vec3::default();
                'clipped: for run in &clipped {
                    for (index, &point) in run.iter().enumerate() {
                        if pair_normals_parallel_at(first, second, point)? {
                            tangency_inside_trims = true;
                            tangency_witness = point;
                            if debug_pairs {
                                eprintln!(
                                    "pair {}x{}: TANGENCY at run index {}/{} ({:.6},{:.6},{:.6}) dropped_len={:.4e}",
                                    first.face.id, second.face.id, index, run.len(),
                                    point.x, point.y, point.z, dropped_length
                                );
                            }
                            break 'clipped;
                        }
                    }
                }
                if tangency_inside_trims {
                    // WHAT SHAPE is the tangency? The witness is only a marched
                    // sample within the transverse-seed angular gate, so it is
                    // REFINED onto the contact before being classified — at the
                    // raw witness a G1 cylinder/torus join and a genuine
                    // equal-radius node are eighteen-fold apart, which would be
                    // a band; at the refined contact they are thirty orders
                    // apart, which is a rank question. See
                    // `imprint/tangent_contact.rs`.
                    let contact = classify_tangent_contact(
                        &first.face.surface,
                        &second.face.surface,
                        tangency_witness,
                        options.tolerance,
                    )?;
                    if debug_pairs {
                        eprintln!(
                            "pair {}x{}: contact classification {:?}",
                            first.face.id, second.face.id, contact
                        );
                    }
                    match contact {
                        // An isolated node: the section is a curve everywhere
                        // but that one point, so the ordinary march below runs
                        // and the 2D arrangement carves the pinch on both
                        // faces. `tangent_node` records it so a march that
                        // cannot get through still refuses in THIS class rather
                        // than leaking the marcher's own message.
                        Some(contact) if contact.is_isolated_node() => {
                            tangent_node = Some(contact.point);
                            tangent_nodes.push(contact.point);
                        }
                        // An extended contact (rank-deficient) or a witness we
                        // could not refine onto any contact at all (None — so
                        // nothing is proven and the pre-classification refusal
                        // stands). Neither has a section curve the imprint can
                        // represent.
                        other => {
                            let shape = match other {
                                Some(contact) => contact.describe(),
                                None => format!(
                                    "the tangency near ({:.6},{:.6},{:.6}) could not be refined \
                                     onto a contact, so its shape is unproven",
                                    tangency_witness.x, tangency_witness.y, tangency_witness.z
                                ),
                            };
                            return Err(KernelRefusal::new(
                                RefusalClass::TangentNodeSingularity,
                                KernelStage::Intersect,
                                format!(
                                    "boolean: unsupported singular/tangent-node surface intersection \
                                     between faces {} and {}: {shape}",
                                    first.face.id, second.face.id
                                ),
                            ));
                        }
                    }
                }
                if debug_pairs && tangent_node.is_none() {
                    // Transverse the whole way inside both trims: the coarse
                    // 5x5 classifier only saw the tangency the trims cut away.
                    eprintln!(
                        "pair {}x{}: classified tangential-only, but the curve inside both \
                         trims is transverse ({dropped_length:.4}) — marching",
                        first.face.id, second.face.id
                    );
                }
            }

            // The trim-window carriers: same surface and parameterization
            // over the window, so hit (u, v) values remain valid on the
            // originals; out-of-window intersections could never survive
            // clip_branch_to_trims and are not walked at all.
            let first_march = first_restricted[first_index]
                .as_ref()
                .map(|window| &window.surface)
                .unwrap_or(&first.face.surface);
            let second_march = second_restricted[second_index]
                .as_ref()
                .map(|window| &window.surface)
                .unwrap_or(&second.face.surface);
            let mut seed_points = Vec::new();
            // Smallest |edge_tangent · surface_normal| over accepted seeds — the
            // local grazing measure at the trim crossings (near 0 = the edge
            // pierces the other surface tangentially). Gates the near-tangent
            // clip-order rescue below to genuinely grazing pairs.
            let mut min_seed_tangency = f64::INFINITY;
            // Two grazing crossings of one trim edge bound a real, narrow
            // interval. It cannot tolerate a section fit that misses either
            // crossing, even when a coarse whole-model distance looks small.
            let mut paired_grazing_crossings = false;
            for (face, other_march, other) in
                [(first, second_march, second), (second, first_march, first)]
            {
                for edge in &face_edge_lists[&face.key()] {
                    if edge.degenerate {
                        continue;
                    }
                    let mut grazing_crossings: Vec<Vec3> = Vec::new();
                    for hit in intersect_curve_surface(&edge.curve, other_march, options.tolerance).or_refuse(KernelStage::Intersect, "intersect_curve_surface")?
                    {
                        if hit.t < edge.t0 - 1e-9 || hit.t > edge.t1 + 1e-9 {
                            continue;
                        }
                        let derivative = edge.curve.derivatives(hit.t, 1).or_refuse(KernelStage::Intersect, "derivatives")?[1];
                        // A STATIONARY POINT on the edge curve is not a reason
                        // to refuse the boolean. An involute flank starts at
                        // the base circle with an exactly zero tangent, so a
                        // gear tooth's profile carries one cusp per flank, and
                        // a hit landing on that parameter used to propagate
                        // "Vec3.normalized: zero-length vector" out of the
                        // whole union (the 2026-09-14 herringbone report: the
                        // second tooth band's z=0 cap edge meets the first
                        // band's flank exactly at the cusp). The direction is
                        // still defined as the one-sided limit, so read it from
                        // just inside the edge's own range; an edge with no
                        // direction at all simply contributes no seed, exactly
                        // as a hit whose face normal is unavailable already
                        // does two lines below. Seeds are marcher hints, so a
                        // dropped one costs a hint, never an answer.
                        // Escape hatch for tamper-verification:
                        // BREP_CUSP_TANGENT_RESCUE=0 restores the refusal.
                        let tangent = match derivative.normalized() {
                            Ok(tangent) => tangent,
                            Err(error) => {
                                if std::env::var("BREP_CUSP_TANGENT_RESCUE").as_deref() == Ok("0") {
                                    return Err(error)
                                        .or_refuse(KernelStage::Intersect, "normalized");
                                }
                                match edge.curve.stationary_tangent_rescue(hit.t, edge.t0, edge.t1)
                                {
                                    Some(tangent) => tangent,
                                    None => continue,
                                }
                            }
                        };
                        // `hit` was solved on the other face's MARCH WINDOW, so
                        // its uv is in that face's chart — a period off the
                        // carrier's where the window is lifted, and `normal`
                        // clamps a parameter past its domain.
                        let normal = match other.chart().normal(hit.u, hit.v) {
                            Ok(normal) => normal,
                            Err(_) => continue,
                        };
                        if debug_pairs {
                            eprintln!(
                                "pair {}x{}: seed edge {} t={:.6} p=({:.5},{:.5},{:.5}) |tan.n|={:.4} {}",
                                first.face.id,
                                second.face.id,
                                edge.id,
                                hit.t,
                                hit.point.x,
                                hit.point.y,
                                hit.point.z,
                                tangent.dot(normal).abs(),
                                if tangent.dot(normal).abs() >= 0.1 { "ACCEPT" } else { "reject" }
                            );
                        }
                        if tangent.dot(normal).abs() >= 0.1 {
                            seed_points.push(hit.point);
                            min_seed_tangency = min_seed_tangency.min(tangent.dot(normal).abs());
                            section_evidence = true;
                        } else {
                            if grazing_crossings.iter().any(|point| {
                                point.sub(hit.point).length() > assembler_weld(options.tolerance)
                            }) {
                                paired_grazing_crossings = true;
                            }
                            grazing_crossings.push(hit.point);
                        }
                    }
                }
            }
            profile.seeds += profile.lap(&mut lap_start);
            profile.marched_pairs += 1;
            if debug_pairs {
                eprintln!(
                    "pair {}x{}: marching (relation {:?})...",
                    first.face.id, second.face.id, pair_classification.relation
                );
            }
            let marched = intersect_surfaces(
                first_march,
                second_march,
                &SurfaceIntersectionOptions {
                    tolerance: options.tolerance,
                    maximum_step: march_maximum_step(
                        &pair_classification,
                        options.maximum_ssi_step,
                        builder.scale,
                    ),
                    seed_points: seed_points.clone(),
                    ..Default::default()
                },
            );
            // A pair carrying a tangent node keeps its OWN refusal class when
            // the march fails: the node is why the trace cannot close, and the
            // equal-radius torus pair depends on being told so rather than on
            // reading a step-budget message it cannot act on.
            let marched = match (marched, tangent_node) {
                (Err(error), Some(node)) => {
                    return Err(KernelRefusal::new(
                        RefusalClass::TangentNodeSingularity,
                        KernelStage::Intersect,
                        format!(
                            "boolean: unsupported singular/tangent-node surface intersection \
                             between faces {} and {}: the section through the tangent node at \
                             ({:.6},{:.6},{:.6}) could not be marched ({error})",
                            first.face.id, second.face.id, node.x, node.y, node.z
                        ),
                    ));
                }
                (marched, _) => marched,
            };
            let mut branches = marched
            .map_err(|error| {
                format!(
                    "{error} (marching faces {} and {})",
                    first.face.id, second.face.id
                )
            }).or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
            profile.march += profile.lap(&mut lap_start);
            let rescue_started = profile.enabled.then(Instant::now);
            // MARCH-ORDER SWAP RESCUE (hatch BREP_MARCH_SWAP_RESCUE=0).
            // `intersect_surfaces` is not order-symmetric: the coupled Newton
            // trace can fail to start/continue from a valid seed when the two
            // surfaces are presented in one operand order yet succeed in the
            // other. This is the sole reason t217's `subtract(sphere, step)`
            // fails while every other op passes — pair 118x842 (sphere-first)
            // marches ZERO branches, while 842x118 (step-first, the working
            // a\b order) marches the section from the IDENTICAL accepted pierce
            // seeds. When the forward order returns no branch AND an accepted
            // transverse pierce seed exists (so a real section provably crosses
            // both trims), retry the march with the surfaces swapped — i.e.
            // reproduce the exact call the working operand order makes for this
            // pair (measured: seed_only alone does NOT recover it — the section
            // is found from an auto-grid start, not from the near-tangent pierce
            // seeds, which the seed normal-cross gate rejects). The returned
            // branch points are 3D and therefore order-independent, so no
            // parameter remap is needed; they flow through the identical clip /
            // process_curve gates below, which drop any out-of-trim or
            // sub-length run exactly as today. The rescue only ever runs when
            // the forward order found NOTHING, so it cannot alter a pair that
            // already marched.
            if branches.is_empty()
                && !seed_points.is_empty()
                && pair_classification.relation == SurfacePairRelation::Candidate
                && std::env::var("BREP_MARCH_SWAP_RESCUE").as_deref() != Ok("0")
            {
                // FAIL-SOFT: the forward order already returned Ok(empty); a
                // swapped-march error (trace-exhaustion is a real error class —
                // `intersect_surfaces_supplemental` swallows it for exactly this
                // reason) must NOT convert that graceful empty into a hard error.
                // On Err, keep the (empty) forward result and carry on.
                let swapped = intersect_surfaces(
                    second_march,
                    first_march,
                    &SurfaceIntersectionOptions {
                        tolerance: options.tolerance,
                        maximum_step: march_maximum_step(
                            &pair_classification,
                            options.maximum_ssi_step,
                            builder.scale,
                        ),
                        seed_points: seed_points.clone(),
                        ..Default::default()
                    },
                )
                .unwrap_or_default();
                if debug_pairs {
                    eprintln!(
                        "pair {}x{}: MARCH-SWAP RESCUE attempt -> {} branches, pts {:?}",
                        first.face.id,
                        second.face.id,
                        swapped.len(),
                        swapped.iter().map(|b| b.points.len()).collect::<Vec<_>>()
                    );
                }
                if !swapped.is_empty() {
                    branches = swapped;
                }
            }
            // NEAR-TANGENT CLIP-ORDER RESCUE (hatch BREP_MARCH_SWAP_CLIP_RESCUE=0).
            // Companion to the empty-branch MARCH-SWAP RESCUE above:
            // `intersect_surfaces` is order-asymmetric not only in WHETHER it
            // marches, but in the exact sample positions of the section
            // polyline. On a NEAR-TANGENT graze, `clip_branch_to_trims`
            // classifies those samples against the mutual trims, and a sample
            // landing just past the near-tangent boundary is dropped as a
            // false-Outside that TRUNCATES the clipped section. The two operand
            // orders drop DIFFERENT near-tangent tail samples, so one order
            // yields a section ~0.1-0.2mm shorter at ONE endpoint (t660: b\a's
            // step-first order clips pairs 223x105/399x105 ~0.22/0.14mm short of
            // the section a\b's cyl-first order keeps, stranding edge 173
            // one-use). Near-tangent clip errors are almost exclusively
            // false-Outside (a point truly outside a trim rarely projects to an
            // in-trim uv), so the order with the LONGER clipped section suffered
            // fewer drops and is the more complete one. When the swapped order
            // marches the SAME branch structure with a meaningfully longer
            // clipped total, adopt its branches (3D points, so they flow through
            // the identical clip/process_curve gates below). Gated to Candidate
            // pairs (the near-tangent/ambiguous class; clean Transverse
            // crossings clip identically in both orders and never differ) with
            // an accepted GRAZING pierce seed (min |tan·n| < 0.5 — a real
            // section provably crosses, and does so near-tangentially, the only
            // regime where the clip wobbles; this also bounds the extra march to
            // grazing pairs), and requires a >0.1%-of-length margin so
            // raw-sampling jitter (t660: 0.02%) cannot flip a clean pair.
            // Fail-soft: any swapped-march or comparison-clip error keeps the
            // forward result.
            if !branches.is_empty()
                && !seed_points.is_empty()
                && min_seed_tangency < 0.5
                && pair_classification.relation == SurfacePairRelation::Candidate
                && std::env::var("BREP_MARCH_SWAP_CLIP_RESCUE").as_deref() != Ok("0")
            {
                let swapped = intersect_surfaces(
                    second_march,
                    first_march,
                    &SurfaceIntersectionOptions {
                        tolerance: options.tolerance,
                        maximum_step: march_maximum_step(
                            &pair_classification,
                            options.maximum_ssi_step,
                            builder.scale,
                        ),
                        seed_points: seed_points.clone(),
                        ..Default::default()
                    },
                )
                .unwrap_or_default();
                // Only a swap that reproduces the SAME branch count — a
                // refined-endpoint variant of the same section, not a different
                // branch decomposition (guards against adopting a spurious
                // extra branch as "longer").
                if !swapped.is_empty() && swapped.len() == branches.len() {
                    let totals = (|| -> Result<(f64, f64), KernelRefusal> {
                        let mut fwd = 0.0;
                        for b in &branches {
                            let refined = insert_seed_points_into_branch(
                                &b.points,
                                &seed_points,
                                options.tolerance,
                            );
                            for run in clip_branch_to_trims(&refined, first, second)? {
                                fwd += run
                                    .windows(2)
                                    .map(|p| p[1].sub(p[0]).length())
                                    .sum::<f64>();
                            }
                        }
                        let mut swp = 0.0;
                        for b in &swapped {
                            let refined = insert_seed_points_into_branch(
                                &b.points,
                                &seed_points,
                                options.tolerance,
                            );
                            for run in clip_branch_to_trims(&refined, first, second)? {
                                swp += run
                                    .windows(2)
                                    .map(|p| p[1].sub(p[0]).length())
                                    .sum::<f64>();
                            }
                        }
                        Ok((fwd, swp))
                    })();
                    if let Ok((fwd_clip, swp_clip)) = totals {
                        let margin = fwd_clip.max(swp_clip) * 1.0e-3;
                        let adopt = swp_clip > fwd_clip + margin;
                        if debug_pairs {
                            eprintln!(
                                "pair {}x{}: CLIP-SWAP RESCUE fwd_clip={:.6} swp_clip={:.6} margin={:.6}{}",
                                first.face.id,
                                second.face.id,
                                fwd_clip,
                                swp_clip,
                                margin,
                                if adopt { " ADOPT" } else { "" }
                            );
                        }
                        if adopt {
                            branches = swapped;
                        }
                    }
                }
            }
            if let Some(started) = rescue_started {
                profile.rescue_march += started.elapsed();
            }
            if branches.iter().any(|branch| branch.points.len() >= 2) {
                section_evidence = true;
            }
            if debug_pairs {
                eprintln!(
                    "pair {}x{}: MARCH {} branches, pts {:?}",
                    first.face.id,
                    second.face.id,
                    branches.len(),
                    branches.iter().map(|b| b.points.len()).collect::<Vec<_>>()
                );
            }
            for branch in branches {
                // The pierce seeds are the section's exact trim-crossing
                // points — insert them so no trim interval shorter than the
                // march step is invisible to the point-classification clip.
                let refined_points =
                    insert_seed_points_into_branch(&branch.points, &seed_points, options.tolerance);
                let clip_started = profile.enabled.then(Instant::now);
                let runs = clip_branch_to_trims(&refined_points, first, second)?;
                if let Some(started) = clip_started {
                    profile.clip += started.elapsed();
                }
                for run in runs {
                    if debug_pairs {
                        eprintln!(
                            "pair {}x{}: clip run len_pts={} length={:.4e}",
                            first.face.id,
                            second.face.id,
                            run.len(),
                            run.windows(2)
                                .map(|pair| pair[1].sub(pair[0]).length())
                                .sum::<f64>()
                        );
                    }
                    if run.len() < 2 {
                        continue;
                    }
                    let length: f64 = run
                        .windows(2)
                        .map(|pair| pair[1].sub(pair[0]).length())
                        .sum();
                    if length <= options.tolerance * 100.0 {
                        continue;
                    }
                    let shared_started = profile.enabled.then(Instant::now);
                    let follows_shared = branch_follows_shared_boundary(
                        &run,
                        first,
                        second,
                        &builder.edges,
                        options.tolerance,
                        builder.scale,
                    )?;
                    if let Some(started) = shared_started {
                        profile.shared_boundary += started.elapsed();
                    }
                    if follows_shared {
                        if debug_pairs {
                            eprintln!(
                                "pair {}x{}: run dropped (follows shared boundary)",
                                first.face.id, second.face.id
                            );
                        }
                        continue;
                    }
                    if let Some(census) = pair_census.as_mut() {
                        census.record_stations(&run);
                    }
                    let pieces_before = builder.pieces.len();
                    let chunk_points = options.fit_chunk_points.unwrap_or(run.len()).max(2);
                    let mut start = 0;
                    while start + 1 < run.len() {
                        let end = (start + chunk_points - 1).min(run.len() - 1);
                        let fit_started = profile.enabled.then(Instant::now);
                        let fit_tolerance = options.tolerance.max(1e-7);
                        let mut section: Vec<Vec3> = run[start..=end].to_vec();
                        let slivers = even_sliver_ends(&mut section, &first.face.surface, &second.face.surface, options.tolerance)
                            .or_refuse(KernelStage::Intersect, "csg.imprint.driver.sliver_end")?;
                        if slivers > 0 {
                            if let Ok(path) = std::env::var("BREP_SECTION_SLIVER_CENSUS") {
                                use std::io::Write;
                                if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                                    let _ = writeln!(file, "sliver-end census: pair {}x{} slivers {slivers} stations {}", first.face.id, second.face.id, section.len());
                                }
                            }
                        }
                        let (mut fit, recovered) = crate::fit::fit_polyline_of_degree_with_recovery(
                            &section,
                            fit_tolerance,
                            options.maximum_fit_points,
                            options.local_fit,
                            section_fit_degree(),
                        ).or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
                        // A recovered fit may reproduce every march point yet
                        // still wander off the carriers between sparse stations.
                        // Measure it against both supports before accepting that
                        // recovery, and use the existing bounded SSI refiner.
                        let refine_gate = section_refine_gate()
                            .or_else(|| recovered.then_some(1.0)).or_else(|| {
                            (paired_grazing_crossings
                                && std::env::var("BREP_GRAZE_SECTION_REFINE").as_deref() != Ok("0"))
                                .then_some(1.0)
                        });
                        if let Some(gate) = refine_gate {
                            let stations_before = fit.kept.len();
                            let (refined, worst, unrefined, rounds, exit) = refine_section_against_carriers(
                                &mut section,
                                fit,
                                &first.face.surface,
                                &second.face.surface,
                                fit_tolerance,
                                options.tolerance,
                                options.local_fit,
                                gate * fit_tolerance,
                            ).or_refuse(KernelStage::Intersect, "csg.imprint.driver.midspan")?;
                            // `BREP_SECTION_REFINE_CENSUS=<file>`: one line per section,
                            // appended, so a corpus replay (whose children's stderr is
                            // captured) can be read for trip counts and growth.
                            if let Ok(path) = std::env::var("BREP_SECTION_REFINE_CENSUS") {
                                use std::io::Write;
                                if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                                    let _ = writeln!(
                                        file,
                                        "section-refine census: pair {}x{} exit {:?} rounds {} stations {} -> {} unrefined {:.3e} worst {:.3e}",
                                        first.face.id, second.face.id, exit, rounds, stations_before, refined.kept.len(), unrefined, worst
                                    );
                                }
                            }
                            if debug_pairs && (rounds > 0 || !matches!(exit, MidspanExit::Converged | MidspanExit::BelowGate)) {
                                eprintln!(
                                    "pair {}x{}: mid-span refinement {:?} after {} round(s): {} stations, worst {:.3e}",
                                    first.face.id, second.face.id, exit, rounds, section.len(), worst
                                );
                            }
                            fit = refined;
                        }
                        if let Some(started) = fit_started {
                            profile.fit += started.elapsed();
                        }
                        if let Some(census) = pair_census.as_mut() {
                            census.record_fit(&fit);
                        }
                        let process_started = profile.enabled.then(Instant::now);
                        builder.process_curve(
                            fit.curve,
                            first,
                            second,
                            &[first, second],
                            &[first, second],
                            true,
                        )?;
                        if let Some(started) = process_started {
                            profile.process += started.elapsed();
                        }
                        start = end;
                    }
                    for piece in &builder.pieces[pieces_before..] {
                        builder.marched_pieces.insert(piece.id);
                    }
                    if debug_pairs {
                        eprintln!(
                            "pair {}x{}: run -> {} pieces",
                            first.face.id,
                            second.face.id,
                            builder.pieces.len() - pieces_before
                        );
                    }
                }
            }
            profile.clip_and_fit += profile.lap(&mut lap_start);
        }
    }
    profile.report();
    // SELF-TOUCH SPLIT: a face whose loops touch at an edge interior (a hole
    // tangent to a fillet setback) gets a vertex minted at the touch on BOTH
    // edges, so the fragment arrangement's pinch resolution assembles — see
    // `imprint/self_touch.rs`. Escape hatch: BREP_SELF_TOUCH_SPLIT=0.
    let mut tail_lap: Option<Instant> = None;
    let mut tail_ms = Vec::<(&str, f64)>::new();
    let mut lap = |name: &'static str, started: &mut Option<Instant>, tail_ms: &mut Vec<(&str, f64)>| {
        if profile.enabled {
            let now = Instant::now();
            if let Some(previous) = *started {
                tail_ms.push((name, (now - previous).as_secs_f64() * 1_000.0));
            }
            *started = Some(now);
        }
    };
    lap("pairs", &mut tail_lap, &mut tail_ms);
    if std::env::var("BREP_SELF_TOUCH_SPLIT").as_deref() != Ok("0") {
        builder.split_self_touching_loops(&first_faces, &face_edge_lists, &cached_subcurve)?;
        builder.split_self_touching_loops(&second_faces, &face_edge_lists, &cached_subcurve)?;
    }
    // PIECE-ENDPOINT EXCHANGE post-pass: with every pair's process_curve
    // done, the piece set is final for the ridden-edge class — imprint each
    // open riding piece's junction endpoints onto the ridden edges (see the
    // method doc; hatch BREP_OVERLAP_PIECE_ENDPOINT_SPLIT=0).
    builder.exchange_piece_endpoint_junctions()?;
    lap("self_touch", &mut tail_lap, &mut tail_ms);
    let mut edge_splits = builder
        .edge_splits
        .into_iter()
        .map(|((operand, edge_id), mut parameters)| {
            parameters.sort_by(f64::total_cmp);
            EdgeSplitRecord {
                operand,
                edge_id,
                parameters,
            }
        })
        .collect::<Vec<_>>();
    edge_splits.sort_by_key(|record| (record.operand, record.edge_id));
    let mut by_face = builder
        .by_face
        .into_iter()
        .map(|(face, piece_ids)| FaceImprints {
            operand: face.operand,
            face_id: face.face_id,
            piece_ids,
        })
        .collect::<Vec<_>>();
    by_face.sort_by_key(|record| (record.operand, record.face_id));
    let section_evidence = section_evidence || !builder.pieces.is_empty();
    let marched_pieces = std::mem::take(&mut builder.marched_pieces);
    let mut result = ImprintResultRecord {
        tangent_nodes,
        vertices: builder.vertices,
        pieces: builder.pieces,
        by_face,
        edge_splits,
        barrier_edges: builder.barrier_edges.into_iter().collect(),
        section_evidence,
        cosurface_pairs,
    };
    // COINCIDENT-PIECE MERGE (problemInbox equator-tangent, and the generic
    // one-circle-from-many-pairs class): the SAME section curve can be minted
    // by several pairs — a cosurface boundary-edge copy (the cylinder cap
    // ring lying ON the inscribed sphere) AND the cap-plane's analytic
    // section ring are one circle minted twice, each carrying only its own
    // pair's supports/pcurves. Assembly then builds duplicate edges that
    // cannot both be two-use → one-use strands. Merge pieces whose curves
    // coincide along their whole span (bidirectional max deviation within
    // the weld band): keep the first, union the supports/pcurves/by_face
    // registrations of the rest into it. Escape hatch:
    // BREP_COINCIDENT_PIECE_MERGE=0.
    lap("collect", &mut tail_lap, &mut tail_ms);
    if std::env::var("BREP_COINCIDENT_PIECE_MERGE").as_deref() != Ok("0") {
        let weld = assembler_weld(options.tolerance).max(options.tolerance * 10.0);
        // Two pieces that coincide along their WHOLE spans (deviation within
        // the weld both ways) also have matching end points, up to direction
        // and a weld or two of overhang, so a piece whose ends are nowhere
        // near the other's needs no 33-station projection sweep. The sweep is
        // what made this pass quadratic in wall time: 630 pieces cost 3.2 s
        // of a 3.8 s imprint on the 2026-09-12 mesh-import report, all of it
        // rejecting pairs an end-point look already rules out.
        let mut ends: Vec<[Vec3; 2]> = Vec::with_capacity(result.pieces.len());
        for piece in &result.pieces {
            let [d0, d1] = piece.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
            ends.push([
                piece.curve.evaluate(d0).or_refuse(KernelStage::Intersect, "evaluate")?,
                piece.curve.evaluate(d1).or_refuse(KernelStage::Intersect, "evaluate")?,
            ]);
        }
        let end_band = 4.0 * weld;
        // A CLOSED piece (a full ring) has no end points to speak of: two
        // rings of the same circle seamed at different azimuths coincide
        // along their whole spans while their domain ends sit anywhere on
        // the ring, so a closed piece always takes the full sweep.
        let ends_match = |a: &[Vec3; 2], b: &[Vec3; 2]| -> bool {
            let near = |p: Vec3, q: Vec3| p.sub(q).length() <= end_band;
            near(a[0], a[1])
                || near(b[0], b[1])
                || (near(a[0], b[0]) && near(a[1], b[1]))
                || (near(a[0], b[1]) && near(a[1], b[0]))
        };
        let mut removed: Vec<u64> = Vec::new();
        let mut index = 0;
        while index < result.pieces.len() {
            let mut other = index + 1;
            while other < result.pieces.len() {
                let coincide = ends_match(&ends[index], &ends[other]) && {
                    let a = &result.pieces[index];
                    let b = &result.pieces[other];
                    max_curve_deviation(&a.curve, &b.curve)? <= weld
                        && max_curve_deviation(&b.curve, &a.curve)? <= weld
                };
                if coincide {
                    let absorbed = result.pieces.remove(other);
                    ends.remove(other);
                    removed.push(absorbed.id);
                    let keeper = &mut result.pieces[index];
                    for pcurve in absorbed.pcurves {
                        if !keeper
                            .pcurves
                            .iter()
                            .any(|existing| {
                                existing.operand == pcurve.operand
                                    && existing.face_id == pcurve.face_id
                            })
                        {
                            keeper.pcurves.push(pcurve);
                        }
                    }
                    let keeper_id = keeper.id;
                    for record in &mut result.by_face {
                        if let Some(position) =
                            record.piece_ids.iter().position(|&id| id == absorbed.id)
                        {
                            if record.piece_ids.contains(&keeper_id) {
                                record.piece_ids.remove(position);
                            } else {
                                record.piece_ids[position] = keeper_id;
                            }
                        }
                    }
                } else {
                    other += 1;
                }
            }
            index += 1;
        }
        if !removed.is_empty() && std::env::var("BREP_DEBUG_BOOL").is_ok() {
            eprintln!("coincident-piece merge: absorbed {:?}", removed);
        }
    }
    // Rescue near-tangent SSI truncations BEFORE canonicalization so the added
    // bridge pieces' endpoints (existing crossing/stub vertices) fold into the
    // same junction merges as every other section.
    lap("coincident_merge", &mut tail_lap, &mut tail_ms);
    extend_truncated_sections(
        &mut result,
        &face_edge_lists,
        solid_a,
        solid_b,
        &charts,
        options.tolerance,
    )?;
    lap("extend_truncated", &mut tail_lap, &mut tail_ms);
    canonicalize_imprint_junctions(&mut result, solid_a, solid_b, &charts, options.tolerance)?;
    lap("canonicalize", &mut tail_lap, &mut tail_ms);
    // ONE RIM, ONE EDGE: with vertex identity final, dissolve the section
    // vertices that no operand edge passes through — the marcher's own
    // parameterization origin, which `process_curve` cannot tell from a
    // junction because it sees one pair at a time. See the method doc.
    let dissolved = dissolve_section_origin_vertices(
        &mut result,
        &marched_pieces,
        solid_a,
        solid_b,
        &charts,
        options.tolerance,
    )?;
    if dissolved > 0 && std::env::var("BREP_DEBUG_BOOL").is_ok() {
        eprintln!("origin-dissolve: {dissolved} parameterization vertex/vertices removed");
    }
    lap("origin_dissolve", &mut tail_lap, &mut tail_ms);
    // B2: reuse an existing boundary edge as the shared section edge wherever a
    // section coincides with one along its whole span (vertices are final after
    // canonicalization; `face_edge_lists` holds each face's boundary edges on
    // the healed operands). Runs here so both operands reference ONE edge.
    reuse_boundary_section_edges(
        &mut result,
        &face_edge_lists,
        solid_a,
        solid_b,
        options.tolerance,
    )?;
    // Capstone step 1 — instrumentation only, zero behavior change: report
    // every (section piece × boundary edge) contact where the piece runs
    // within the scale-derived band of the edge over a real span. Measuring
    // the bands here first validates the graze-contact model on the
    // acceptance suite before step 2's common-block machinery replaces a
    // grazed overlap with a shared edge.
    lap("reuse_boundary", &mut tail_lap, &mut tail_ms);
    report_graze_contacts(&result, &face_edge_lists, solid_a, solid_b, options.tolerance)?;
    if profile.enabled {
        let line = tail_ms
            .iter()
            .map(|(name, ms)| format!("{name}={ms:.2}"))
            .collect::<Vec<_>>()
            .join(" ");
        eprintln!("imprint.stages {line}");
    }
    Ok(result)
}

/// Debug-only graze-contact survey (`BREP_DEBUG_GRAZE=1`): for each section
/// piece and each boundary edge of its support faces, sample the piece and
/// measure distance to the edge; report contacts whose in-band span exceeds
/// both the weld scale and 4× the minimum deviation (span-wise proximity, not
/// a point touch). `band_cap` reuses the residual-merge `sep_cap` ceiling —
/// measured, never grown. The output is the raw material for capstone step 2
/// (partial-span common-block): which contacts exist, their spans, and their
/// measured bands.
fn report_graze_contacts(
    result: &ImprintResultRecord,
    face_edge_lists: &HashMap<FaceKey, Vec<&EdgeRecord>>,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    if std::env::var("BREP_DEBUG_GRAZE").as_deref() != Ok("1") {
        return Ok(());
    }
    let raw_extent = raw_solid_extent(solid_a).max(raw_solid_extent(solid_b));
    let (_, band_cap) = residual_merge_bands(raw_extent, tolerance);
    const SAMPLES: usize = 17;
    for piece in &result.pieces {
        let [t0, t1] = [piece.t0, piece.t1];
        if !(t1 > t0) {
            continue;
        }
        for key in piece.support_faces {
            let Some(edges) = face_edge_lists.get(&key) else {
                continue;
            };
            for edge in edges {
                if edge.degenerate {
                    continue;
                }
                let mut in_band = 0usize;
                let mut min_dev = f64::INFINITY;
                let mut max_dev_in_band = 0.0f64;
                let mut span = 0.0f64;
                let mut prev: Option<(bool, Vec3)> = None;
                for k in 0..SAMPLES {
                    let t = t0 + (t1 - t0) * k as f64 / (SAMPLES - 1) as f64;
                    let point = piece.curve.evaluate(t).or_refuse(KernelStage::Intersect, "evaluate")?;
                    let deviation = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?.distance;
                    min_dev = min_dev.min(deviation);
                    let inside = deviation <= band_cap;
                    if inside {
                        in_band += 1;
                        max_dev_in_band = max_dev_in_band.max(deviation);
                        if let Some((true, prev_point)) = prev {
                            span += point.sub(prev_point).length();
                        }
                    }
                    prev = Some((inside, point));
                }
                // Span-wise contact: several consecutive samples in band and a
                // span that dwarfs the closest-approach (not a transversal
                // crossing, which dips in and out at one sample).
                if in_band >= 3 && span > (4.0 * min_dev).max(assembler_weld(tolerance)) {
                    eprintln!(
                        "graze: piece {} sup=[{}:{},{}:{}] ~ edge {}:{} span={:.3e} band=[{:.3e},{:.3e}] samples_in_band={}/{}",
                        piece.id,
                        piece.support_faces[0].operand,
                        piece.support_faces[0].face_id,
                        piece.support_faces[1].operand,
                        piece.support_faces[1].face_id,
                        key.operand,
                        edge.id,
                        span,
                        min_dev,
                        max_dev_in_band,
                        in_band,
                        SAMPLES
                    );
                }
            }
        }
    }
    Ok(())
}

/// Are the two carriers TANGENT (normals parallel) where they both pass through
/// `point`?
///
/// The test the tangential-only refusal above needs: a point on the marched
/// intersection is a TANGENT NODE when the two surface normals there are
/// parallel. The threshold is the transversality bound the supplemental
/// detector already accepts seeds by (`TRANSVERSE_SEED_CROSS`), not the
/// far tighter pair-classifier bound — a node the march merely passes CLOSE to
/// still poisons the assembly, so this errs toward calling a pair singular.
fn pair_normals_parallel_at(
    first: TaggedFace<'_>,
    second: TaggedFace<'_>,
    point: Vec3,
) -> Result<bool, KernelRefusal> {
    let mut normals = [Vec3::default(); 2];
    for (slot, face) in normals.iter_mut().zip([first, second]) {
        let projection = project_point_to_surface(&face.face.surface, point)
            .or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
        let Ok(normal) = face.face.surface.normal(projection.u, projection.v) else {
            // A pole/singular parameter point cannot witness transversality;
            // treat it as tangential so the pair stays refused.
            return Ok(true);
        };
        *slot = normal;
    }
    Ok(normals[0].cross(normals[1]).length() <= crate::TRANSVERSE_SEED_CROSS)
}
