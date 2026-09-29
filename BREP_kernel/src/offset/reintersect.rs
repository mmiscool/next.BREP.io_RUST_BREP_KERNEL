//! The ONE definition of "re-intersect two carriers to remake a trim boundary".
//!
//! # What this is, and where it sits in the offset-unification plan
//!
//! The audit splits the shared machinery into three: offsetting a surface
//! pointwise ([`crate::OffsetEvaluator`], `offset/point.rs`, slice 1),
//! re-trimming a face whose boundary has already moved (`offset/retrim.rs`,
//! slice 2), and **intersecting carriers to remake the boundary in the first
//! place** — the step that has to happen *between* those two, and the one
//! push-face did not have in general form. §2.3's matrix is the work list:
//! offset-shell re-intersects arbitrary carriers, push-face refuses most
//! neighbour classes.
//!
//! # Why this is not an extraction of offset-shell's arrangement
//!
//! §5's Slice 3 proposes exposing "replace one face's carrier and re-derive the
//! incident trim by imprint arrangement, extracted from `offset_shell_impl`",
//! and rates its own confidence in a clean extraction low. Read, not assumed:
//! `offset_shell_impl` is a first-attempt/retry driver over *whole solids*
//! whose fragment selection (`point_on_offset_skin`) asserts that every kept
//! point sits at the full offset distance from **every** retained source face —
//! an invariant that is simply false when one face of a solid is pushed. There
//! is nothing in that pipeline a single-face push can consume without first
//! deleting the invariant that makes it work.
//!
//! What offset-shell actually does at the level a push needs is one step lower,
//! in the imprint driver it calls: **try the recognized analytic pair first,
//! fall back to the marched surface/surface intersection** — `driver.rs:225`
//! (`intersect_analytic_pair`) then `driver.rs:398` (`intersect_surfaces`) then
//! `driver.rs:644` (`fit_polyline`). That two-lane structure is the generality;
//! the arrangement around it is offset-shell's own bookkeeping.
//!
//! `intersect_analytic_pair`'s own contract already names the fallback —
//! "*or `None` when the pair is not handled and the caller must fall back to
//! SSI marching*" (`geometry/analytic_surface/intersect.rs:73-75`). Push-face
//! read `None` as a refusal. That misreading **is** the capability gap.
//!
//! # Conventions, made explicit rather than baked in
//!
//! Slice 1 found that a shared helper with one hard-coded convention silently
//! moves a caller (audit §9.6.2), and slices 2 and 2b carried that forward as
//! an explicit pcurve fit lane (since retired: that lane's planar/ruled split
//! turned out to be a defect, not a convention) and `OffsetPairDegeneracy`.
//! The same discipline here:
//!
//! * **The analytic lane is bit-identical to calling
//!   [`crate::intersect_analytic_pair`] directly**, with the same arguments in
//!   the same order. A caller that only ever reaches analytic pairs gets
//!   exactly the curves it got before this module existed — which is what the
//!   bit-identity probe measures.
//! * **Refusals are values, not strings** ([`ReintersectRefusal`]). Every push
//!   path already has its own wording for "the push separated the carriers";
//!   a shared form that emitted its own text would have moved their tests.
//! * **The marched lane is residual-gated and the gate is the caller's.** A
//!   fitted section is an approximation; the caller says how much drift its
//!   operation may ship. There is no default, because the right number is
//!   different for a boolean imprint and for a push that must re-validate a
//!   solid.
//! * **Branch selection is never made here.** Both lanes return every branch
//!   they found, in their own order. Which one is *this* rim, and which way it
//!   must run, is decided at the site from the boundary it is replacing.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::topology::EdgeRecord;
use crate::{
    fit_polyline, intersect_analytic_pair, intersect_surfaces, project_point_to_surface, NurbsCurve,
    NurbsSurface, SurfaceIntersectionOptions, Vec3, Vec4,
};

/// Which lane produced a set of rim curves.
///
/// Recorded rather than hidden because the two lanes have different exactness:
/// [`RimLane::Analytic`] is a closed-form conic with no fit error at all, while
/// [`RimLane::Marched`] is a NURBS fit of a traced polyline whose residual the
/// caller gates. A caller that must stay exact can refuse the marched lane by
/// looking at this; a diagnostic can report which lane a corpus reaches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RimLane {
    /// [`crate::intersect_analytic_pair`] — exact lines/circles/ellipses for a
    /// recognized pair, no marching and no polyline fitting.
    Analytic,
    /// The general marched surface/surface intersection, fitted to NURBS. This
    /// is the lane that makes push-face's neighbour handling surface-type-blind.
    Marched,
}

/// One branch of a re-intersection.
pub(crate) struct RimSection {
    /// The whole branch as a curve: an exact conic on the analytic lane, a fit
    /// of [`RimSection::polyline`] on the marched one.
    pub(crate) curve: NurbsCurve,
    /// The traced polyline the fit came from — **empty on the analytic lane**,
    /// which has no polyline and needs none.
    ///
    /// Carried because a caller that has to cut the branch into ARCS cannot do
    /// it on the curve: a closed fit's parameter origin is wherever the trace
    /// started, so an arc between two of its points routinely wraps that
    /// origin, and cutting a NURBS curve across its own start means splitting
    /// and re-joining with a re-parameterization. Cutting the polyline
    /// cyclically and fitting each run is exact bookkeeping on the data the
    /// marcher actually produced — and it is what offset-shell's imprint driver
    /// does too (`clip_branch_to_trims` then `fit_polyline` per run,
    /// `csg/imprint/driver.rs:600-655`).
    pub(crate) polyline: Vec<Vec3>,
    pub(crate) closed: bool,
}

/// The branches two carriers share, and which lane found them.
pub(crate) struct RimIntersection {
    pub(crate) sections: Vec<RimSection>,
    pub(crate) lane: RimLane,
    /// The worst distance from a sampled point of any returned curve to either
    /// carrier. Exactly `0.0` is not claimed for the analytic lane — it is not
    /// measured there, because a closed-form conic on a recognized pair has no
    /// fit to gate and measuring it would cost a projection per sample for a
    /// number no caller acts on.
    pub(crate) residual: f64,
}

impl RimIntersection {
    /// Every branch as a curve, for a caller that takes whole sections.
    pub(crate) fn curves(&self) -> Vec<NurbsCurve> {
        self.sections
            .iter()
            .map(|section| section.curve.clone())
            .collect()
    }

    /// The branch that best matches a boundary being replaced: the one whose
    /// worst distance to any of `reference` is least. A "nearest by midpoint"
    /// pick — which is right for two well-separated conics — chooses wrongly
    /// between two branches of one section set that pass near each other, and a
    /// wall with two holes against one neighbour SURFACE gets exactly that.
    pub(crate) fn nearest_section(&self, reference: &[Vec3]) -> Result<&RimSection, KernelRefusal> {
        let mut best: Option<(f64, &RimSection)> = None;
        for section in &self.sections {
            let [t0, t1] = section.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
            let mut samples = Vec::with_capacity(SECTION_MATCH_SAMPLES);
            for index in 0..SECTION_MATCH_SAMPLES {
                let fraction = index as f64 / (SECTION_MATCH_SAMPLES - 1) as f64;
                samples.push(section.curve.evaluate(t0 + (t1 - t0) * fraction).or_refuse(KernelStage::Intersect, "evaluate")?);
            }
            let mut worst = 0.0f64;
            for point in reference {
                let nearest = samples
                    .iter()
                    .map(|sample| sample.sub(*point).length())
                    .fold(f64::INFINITY, f64::min);
                worst = worst.max(nearest);
            }
            if best.map(|(score, _)| worst < score).unwrap_or(true) {
                best = Some((worst, section));
            }
        }
        best.map(|(_, section)| section)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Intersect,
                    "reintersect_no_section",
                    "no section to match against the old boundary",
                )
            })
    }
}

/// How densely a section is sampled when matching it to the boundary it
/// replaces. Coarse on purpose: this is a "which branch" question, not a
/// distance measurement.
const SECTION_MATCH_SAMPLES: usize = 24;

/// Why a re-intersection produced nothing usable. Each variant is a distinct
/// geometric situation with a distinct right answer at the call site, so the
/// wording stays there.
#[derive(Clone, Debug)]
pub(crate) enum ReintersectRefusal {
    /// The two carriers do not meet. Either the analytic lane *proved* it (an
    /// empty exact result — `Some(vec![])` is a proof of non-intersection, per
    /// `intersect_analytic_pair`'s contract) or the march found no branch.
    Separated,
    /// The marched lane found a branch whose fit drifts off one of the carriers
    /// by more than the caller's gate. A loud refusal, never a shipped
    /// approximation — the standard slice 1 set when it deleted THICKEN's
    /// full-domain fallback.
    Residual { drift: f64, limit: f64 },
    /// A lower-level failure (the marcher, the fit, an evaluation), carried
    /// verbatim so the site can prefix it with its own operation name.
    Failed(KernelRefusal),
}

impl ReintersectRefusal {
    /// The failure text, for a site that has no more specific wording. Sites
    /// with an established message keep it and match on the variant instead.
    pub(crate) fn describe(&self) -> String {
        match self {
            Self::Separated => "the carriers do not meet".to_string(),
            Self::Residual { drift, limit } => format!(
                "the marched section drifts {drift:.3e} off a carrier (limit {limit:.3e})"
            ),
            Self::Failed(refusal) => refusal.message.clone(),
        }
    }
}

/// What the caller will accept from the marched lane.
pub(crate) struct MarchPolicy {
    /// The marcher's own tolerance — the corrector's convergence radius and the
    /// polyline fit's chord tolerance. Model tolerance, not a UV tolerance.
    pub(crate) tolerance: f64,
    /// The largest distance a fitted section may sit from either carrier before
    /// the whole re-intersection is refused.
    pub(crate) residual_tolerance: f64,
    /// Points on the boundary being REPLACED. The new section runs near the old
    /// one for any push small against the feature, so seeding the march there
    /// finds the branch that matters even when the blind seed grid misses it.
    /// Seeds that do not lie on both carriers are dropped by the marcher.
    pub(crate) seeds: Vec<Vec3>,
}

/// The number of samples the residual gate takes along each fitted section.
/// Matches the free-form push's 15×15 dense gate in spirit — enough to catch a
/// fit that wanders between interpolation points, cheap enough to run per rim.
const RESIDUAL_SAMPLES: usize = 33;

/// Upper bound on control points for a fitted section, mirroring the imprint
/// driver's `default_maximum_fit_points` (`csg/imprint.rs:96`) so a push's rim
/// is fitted exactly as tightly as the same curve would be inside a boolean.
const MAXIMUM_FIT_POINTS: usize = 80;

/// Re-intersect two carriers: the exact analytic lane first, the general
/// marched lane when the pair is not one the closed forms recognise.
///
/// This is the whole of what makes a re-intersection surface-type-blind. The
/// analytic call is byte-for-byte the one every push path already made, so a
/// pair that was answered before is answered identically now; the marched lane
/// is reached only where the old code returned `None` and the caller refused.
pub(crate) fn reintersect_carriers(
    first: &NurbsSurface,
    second: &NurbsSurface,
    policy: &MarchPolicy,
) -> Result<RimIntersection, ReintersectRefusal> {
    // Lane 1 — exact. Identical arguments and order to every existing push-face
    // call site, so its result is bit-identical to the pre-slice tree's.
    match intersect_analytic_pair(first, second, policy.tolerance) {
        Some(curves) if !curves.is_empty() => {
            return Ok(RimIntersection {
                sections: curves
                    .into_iter()
                    .map(|curve| RimSection {
                        curve,
                        polyline: Vec::new(),
                        closed: false,
                    })
                    .collect(),
                lane: RimLane::Analytic,
                residual: 0.0,
            })
        }
        // `Some(vec![])` is a PROOF of non-intersection (the closed form solved
        // the pair and found nothing), so it must not fall through to a march
        // that would only burn steps confirming it.
        Some(_) => return Err(ReintersectRefusal::Separated),
        None => {}
    }

    // Lane 2 — general. The pair has no closed form; march it.
    let branches = intersect_surfaces(
        first,
        second,
        &SurfaceIntersectionOptions {
            tolerance: policy.tolerance,
            seed_points: policy.seeds.clone(),
            ..SurfaceIntersectionOptions::default()
        },
    )
    .map_err(|message| {
        ReintersectRefusal::Failed(KernelRefusal::internal(KernelStage::Intersect, "intersect_surfaces", message))
    })?;

    let mut sections: Vec<RimSection> = Vec::new();
    let mut worst_residual = 0.0f64;
    for branch in &branches {
        if branch.points.len() < 2 {
            continue;
        }
        let length: f64 = branch
            .points
            .windows(2)
            .map(|pair| pair[1].sub(pair[0]).length())
            .sum();
        // The imprint driver drops runs shorter than `tolerance * 100` as
        // marcher noise (`driver.rs:619-622`); a rim that short is not a trim
        // boundary either.
        if length <= policy.tolerance * 100.0 {
            continue;
        }
        let fit = fit_polyline(
            &branch.points,
            policy.tolerance.max(1e-7),
            MAXIMUM_FIT_POINTS,
            false,
        )
        .map_err(|message| {
            ReintersectRefusal::Failed(KernelRefusal::internal(KernelStage::Intersect, "fit_polyline", message))
        })?;
        let curve = if branch.closed {
            close_exactly(&fit.curve, policy.tolerance).map_err(ReintersectRefusal::Failed)?
        } else {
            fit.curve
        };
        let residual =
            section_residual(&curve, first, second).map_err(ReintersectRefusal::Failed)?;
        worst_residual = worst_residual.max(residual);
        sections.push(RimSection {
            curve,
            polyline: branch.points.clone(),
            closed: branch.closed,
        });
    }

    if sections.is_empty() {
        return Err(ReintersectRefusal::Separated);
    }
    if worst_residual > policy.residual_tolerance {
        return Err(ReintersectRefusal::Residual {
            drift: worst_residual,
            limit: policy.residual_tolerance,
        });
    }
    Ok(RimIntersection {
        sections,
        lane: RimLane::Marched,
        residual: worst_residual,
    })
}

/// Cut a marched CLOSED section into the arc that runs `from → to` the way the
/// boundary it replaces ran, picked by the point that boundary passed through.
///
/// A hole cut through a wall by a crossing carrier is ONE closed section that
/// the arrangement stored as several open arcs meeting at vertices of valence
/// two. Those vertices are bookkeeping — no third face meets there, so no
/// triple point determines them — which is exactly why they may be placed at
/// the section's own nearest points to the ones they replace, and why the arcs
/// between them are just cyclic runs of the traced polyline.
///
/// Refuses rather than guesses when the two ends land on the same sample or
/// adjacent ones: an arc that short is not a trim boundary, and fitting it
/// would invent a curve out of two points.
///
/// The one exception is `from` and `to` the SAME point exactly — the caller's single relocated
/// vertex at both ends — which asks for the WHOLE section re-origined at that
/// corner: a hole whose rim is ONE closed edge pinned to the neighbour's seam
/// (the imprint stores a closed marched rim as one edge since 2026-09-14,
/// where it used to cut it again at the march's parameterization origin). A
/// full loop passes every point, so `through` cannot pick its direction the
/// way it picks an arc's; for this case the caller passes a point the old edge
/// reaches EARLY (a quarter of the way along), and the direction that reaches
/// it in the first half of the loop is the one the edge ran.
/// Where a relocated corner vertex goes on a marched section: the section's own
/// sample nearest the vertex it replaces.
///
/// Deliberately a SAMPLE and not a refined projection. Every sample the marcher
/// emitted is converged onto both carriers to the trace tolerance, so it is a
/// point that genuinely lies on the new boundary; a refined projection onto the
/// FITTED curve would land on the fit, which is the approximation. And because
/// [`arc_of_section`] cuts the same ring at the same index, an arc's ends and
/// its vertices agree exactly rather than to within a fit residual.
pub(crate) fn section_corner(section: &RimSection, old: Vec3) -> Result<Vec3, KernelRefusal> {
    if !section.closed || section.polyline.len() < 4 {
        return Err(KernelRefusal::internal(KernelStage::Intersect, "reintersect_open_section", "a section that is not a traced closed loop has no corner samples"));
    }
    let ring = &section.polyline[..section.polyline.len() - 1];
    ring.iter()
        .copied()
        .min_by(|a, b| {
            a.sub(old)
                .length()
                .total_cmp(&b.sub(old).length())
        })
        .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "reintersect_empty_ring", "empty section ring"))
}

pub(crate) fn arc_of_section(
    section: &RimSection,
    from: Vec3,
    to: Vec3,
    through: Vec3,
    tolerance: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    if !section.closed || section.polyline.len() < 4 {
        return Err(KernelRefusal::internal(KernelStage::Intersect, "reintersect_open_section", "a section that is not a traced closed loop cannot be cut into arcs"));
    }
    // `trace` closes a loop by pushing the START point back on, so the last
    // sample duplicates the first; the cyclic ring is everything but that.
    let ring = &section.polyline[..section.polyline.len() - 1];
    let nearest_index = |target: Vec3| -> usize {
        let mut best = 0usize;
        let mut best_distance = f64::INFINITY;
        for (index, point) in ring.iter().enumerate() {
            let distance = point.sub(target).length();
            if distance < best_distance {
                best_distance = distance;
                best = index;
            }
        }
        best
    };
    let start = nearest_index(from);
    let end = nearest_index(to);
    let count = ring.len();
    if from.sub(to).length() == 0.0 {
        let early = (nearest_index(through) + count - start) % count;
        let step: isize = if early <= count / 2 { 1 } else { -1 };
        let mut chosen: Vec<Vec3> = (0..count as isize)
            .map(|offset| ring[(start as isize + step * offset).rem_euclid(count as isize) as usize])
            .collect();
        chosen[0] = from;
        chosen.push(from);
        let fit = fit_polyline(&chosen, tolerance.max(1e-7), MAXIMUM_FIT_POINTS, false).or_refuse(KernelStage::Intersect, "fit_polyline")?;
        return close_exactly(&fit.curve, tolerance);
    }
    let forward_len = (end + count - start) % count;
    if forward_len < 2 || count - forward_len < 2 {
        return Err(KernelRefusal::internal(KernelStage::Intersect, "reintersect_arc_ends_coincide", "the two ends of an arc land on the same place of its section — refusing"));
    }
    let run = |begin: usize, step: isize| -> Vec<Vec3> {
        let mut points = Vec::new();
        let mut index = begin as isize;
        loop {
            points.push(ring[index.rem_euclid(count as isize) as usize]);
            if index.rem_euclid(count as isize) as usize == end && points.len() > 1 {
                break;
            }
            index += step;
        }
        points
    };
    let forward = run(start, 1);
    let backward = run(start, -1);
    let closeness = |points: &[Vec3]| -> f64 {
        points
            .iter()
            .map(|point| point.sub(through).length())
            .fold(f64::INFINITY, f64::min)
    };
    let mut chosen = if closeness(&forward) <= closeness(&backward) {
        forward
    } else {
        backward
    };
    // The arc's ends ARE the relocated vertices, exactly — anything else leaves
    // a gap the topology reads as a tear.
    let last = chosen.len() - 1;
    chosen[0] = from;
    chosen[last] = to;
    if chosen.len() < 3 {
        return Err(KernelRefusal::internal(KernelStage::Intersect, "reintersect_arc_samples", "a rebuilt arc has too few samples to fit — refusing"));
    }
    Ok(fit_polyline(&chosen, tolerance.max(1e-7), MAXIMUM_FIT_POINTS, false).or_refuse(KernelStage::Intersect, "fit_polyline")?.curve)
}

/// Make a fitted CLOSED section close bitwise.
///
/// `trace` terminates a closed branch by pushing the START point back onto the
/// polyline (`intersect/surface_surface_intersection.rs:583`), so the fit's
/// first and last interpolation points are the same `Vec3` — but a fit is not
/// obliged to reproduce them as the same control point, and a rim edge whose
/// `start_vertex_id == end_vertex_id` must evaluate to ONE point at both ends
/// or `validate` reports a gap. Snap the last control point onto the first,
/// after checking they already agree within tolerance (so this can only remove
/// rounding, never bend a curve shut).
fn close_exactly(curve: &NurbsCurve, tolerance: f64) -> Result<NurbsCurve, KernelRefusal> {
    let controls = &curve.control_points;
    let (Some(first), Some(last)) = (controls.first(), controls.last()) else {
        return Ok(curve.clone());
    };
    let gap = last.point().or_refuse(KernelStage::Intersect, "point")?.sub(first.point().or_refuse(KernelStage::Intersect, "point")?).length();
    if gap == 0.0 {
        return Ok(curve.clone());
    }
    if gap > tolerance.max(1e-9) * 10.0 {
        return Err(KernelRefusal::internal(KernelStage::Intersect, "reintersect_closure_gap", format!(
            "a marched section reported itself closed but its fit leaves a {gap:.3e} gap"
        )));
    }
    let mut controls = controls.clone();
    let head = controls[0];
    let tail_weight = controls[controls.len() - 1].w;
    let head_point = head.point().or_refuse(KernelStage::Intersect, "point")?;
    let index = controls.len() - 1;
    controls[index] = Vec4::from_point(head_point, tail_weight);
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls).or_refuse(KernelStage::Intersect, "new")
}

/// The worst distance from a sampled point of `curve` to either carrier.
///
/// A marched-and-fitted section is an approximation twice over — the trace's
/// corrector tolerance and the fit's chord tolerance — and a section that has
/// wandered off a carrier produces a face whose pcurve cannot be built, or
/// worse, one that can. Measuring it is what lets the caller refuse loudly
/// instead of shipping the approximation, which is the standard slice 1 set
/// when it deleted THICKEN's full-domain fallback.
fn section_residual(
    curve: &NurbsCurve,
    first: &NurbsSurface,
    second: &NurbsSurface,
) -> Result<f64, KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let mut worst = 0.0f64;
    for index in 0..RESIDUAL_SAMPLES {
        let fraction = index as f64 / (RESIDUAL_SAMPLES - 1) as f64;
        let point = curve.evaluate(t0 + (t1 - t0) * fraction).or_refuse(KernelStage::Intersect, "evaluate")?;
        worst = worst.max(project_point_to_surface(first, point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.distance);
        worst = worst.max(project_point_to_surface(second, point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.distance);
    }
    Ok(worst)
}

/// Sample an edge's curve over its trimmed range, as march seeds.
///
/// The boundary being replaced is the best possible seed set for the boundary
/// replacing it: for any push small against the feature the two run close
/// together, and the marcher drops a seed that does not land on both carriers,
/// so a seed that has stopped being relevant costs nothing.
pub(crate) fn edge_seeds(edge: &EdgeRecord, count: usize) -> Result<Vec<Vec3>, KernelRefusal> {
    let count = count.max(2);
    (0..count)
        .map(|index| {
            let fraction = index as f64 / (count - 1) as f64;
            edge.curve
                .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)
                .or_refuse(KernelStage::Intersect, "evaluate")
        })
        .collect()
}

/// Orient a rebuilt CLOSED section the way the boundary it replaces ran, when
/// the two need not START at the same place.
///
/// `face_offset.rs::match_closed_rim_direction` decides this on the start
/// tangent, which is exact **because** an analytic conic and the edge it
/// replaces both begin on the carrier's seam. A marched section begins wherever
/// the trace was seeded, so the same comparison would read a random pair of
/// tangents. Compare against the old curve at the parameter nearest the new
/// section's start instead — which reduces to the same question, asked at a
/// place where both curves are actually near each other.
pub(crate) fn match_marched_rim_direction(
    rim: NurbsCurve,
    previous: &EdgeRecord,
) -> Result<NurbsCurve, KernelRefusal> {
    let [d0, _] = rim.domain().or_refuse(KernelStage::Intersect, "domain")?;
    let head = rim.evaluate(d0).or_refuse(KernelStage::Intersect, "evaluate")?;
    let projection = crate::project_point_to_curve(&previous.curve, head).or_refuse(KernelStage::Intersect, "project_point_to_curve")?;
    let parameter = projection.u.clamp(previous.t0.min(previous.t1), previous.t1.max(previous.t0));
    let incoming = previous.curve.derivatives(parameter, 1).or_refuse(KernelStage::Intersect, "derivatives")?;
    let rebuilt = rim.derivatives(d0, 1).or_refuse(KernelStage::Intersect, "derivatives")?;
    if incoming[1].dot(rebuilt[1]) < 0.0 {
        return rim.reversed().or_refuse(KernelStage::Intersect, "reversed");
    }
    Ok(rim)
}

