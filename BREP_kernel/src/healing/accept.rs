//! Is a constructed result SOUND enough to return — and if not, can it be
//! repaired rather than refused?
//!
//! `validate()` is an incidence test and says nothing about a body passing
//! through itself. [`crate::solid_self_intersections`] does, in two forms: two
//! faces CROSSING, and one face FOLDING through itself. Neither changes the
//! volume in a way a closed-form oracle can see — a shape can enclose exactly
//! the right amount of space while passing through itself — so a lane that
//! returns one reports success for a different result. That is the failure
//! mode this module exists to stop.
//!
//! ## Repair first, refuse second
//!
//! A self-intersection is not automatically a dead end. Where two faces of one
//! body cross along a curve and the material beyond that curve is a sliver,
//! the answer is to SPLIT both faces along their carriers' exact intersection
//! and drop the pieces that should not be there —
//! [`resolve_face_crossing`]. Only when that cannot be bounded does the lane
//! refuse.
//!
//! The drop is closed under one rule — a face left holding a boundary no other
//! face answers for bordered the dropped material — and that rule has two
//! answers. DROPPING the face is right when the unanswered boundary is the
//! whole of what it contributed: a tail beside a tail. RE-TRIMMING it — taking
//! the unanswered coedges off and re-linking what is left into closed loops —
//! is right when the body keeps the face and only part of its boundary went,
//! which is the only way a BREAKOUT closes. A bore carried out through a side
//! wall leaves the mouth flat holding the stretch of a corner edge that now
//! runs INSIDE the bore and the arc of its own mouth ring that now runs
//! OUTSIDE the body; dropping the flat is absurd, and taking those two coedges
//! off it merges its mouth loop into its outer loop, which is the topology a
//! breakout has. The enumeration runs under the dropping closure first and
//! reaches the re-trimming one only where that resolved nothing at all, so the
//! second can turn a refusal into a repair and can never change a repair this
//! module already made.
//!
//! A FOLD is different and is refused outright here. A face folded through
//! itself is a TRIM that was never constructed: the answer is to carve the
//! folded part of the trim away at the point the carrier is built
//! (`offset/carve.rs`), not to repair the finished body. Refusing it by name
//! is what sends the caller to the lane that can fix it.
//!
//! ## A hole that left its face
//!
//! The scan reads the body as a SURFACE: it convicts two faces that cross and
//! one face that folds. Neither describes a face whose HOLE loop has wandered
//! off the face it is a hole in — the trim is nonsense but nothing crosses,
//! because the wall the hole opens into has gone with it. A direct edit that
//! carries a bore clean off the body builds exactly that: `validate()` is an
//! incidence test and passes, the scan finds no pair to convict, and the
//! divergence-theorem volume of a cylinder is blind to where its axis sits, so
//! the wrong body measures right to the last bit. [`refuse_holes_leaving_their_faces`]
//! is the floor for it, and it runs LAST — after the repair, on whatever body
//! the repair left — so a crossing the repair can take is still taken.
//!
//! ## And a hole that left its face ONTO A NEIGHBOUR
//!
//! That floor convicts two different things, and only one of them has no right
//! answer. A bore dragged CLEAN OFF the body has none: the carrier misses the
//! solid, and there is nothing to cut at any extent. A bore dragged so that its
//! exit mouth crosses the edge between the face it opened through and the face
//! next door has one, and it is the body the crossing repair already builds for
//! a bore carried out through a side wall — the moved carrier and the
//! neighbour split along their exact section, the overshoot dropped, and the
//! faces left holding an unanswered boundary RE-TRIMMED, which merges the mouth
//! into the outer loops of the two faces it now opens through.
//!
//! [`accept_or_break_out`] tries that before refusing, and the two cases are
//! told apart by GEOMETRY rather than by a threshold: the convicted hole loop
//! is traced in its own face's parameter plane and asked which of that face's
//! outer coedges it crosses ([`breakout_pair`]). A hole that has broken out
//! onto a neighbour crosses one, and the face across that edge is the one to
//! re-trim against. A hole that has left the body altogether crosses none, and
//! there is no second face for the repair to be about — so the runaway refuses
//! before a single face is split, and it refuses with the floor's own words
//! plus the repair's reason for declining.
//!
//! The extent is DERIVED and not invented, which is what makes this the same
//! repair rather than a new operation with a new input. The tool is the moved
//! carrier trimmed against the body, and where the carrier meets the body is
//! what the imprint computes. Measured on the 2026-09-21 exit-hole report: the
//! body this builds removes 4195.389267765 from the filled wedge, against
//! 4195.389268713 for the same wedge with the INFINITE moved carrier
//! subtracted through the boolean and 4195.389267095 for a slice integration
//! that never enters the kernel — three roads, 1.6e-9 apart.
//!
//! ## The order
//!
//! Three separate slices built [`accept_sound`]'s sequence on 2026-09-21 and
//! none of them saw the whole of it, so here it is end to end:
//!
//! 1. SCAN once. One tessellation, one BVH sweep; every verdict below is read
//!    off that single reading.
//! 2. NOT FLAGGED — [`accept_or_break_out`]: the hole-containment floor runs on
//!    the body as built, and the body is returned when it says nothing. When it
//!    convicts, the BREAKOUT is tried before the refusal — the same split and
//!    re-trim step 4 runs, on a pair the floor names instead of one the scan
//!    confirmed — and only a breakout that cannot be built refuses. This is the
//!    ONLY path a bore carried clear of its faces reaches, because a bore that
//!    has left the solid crosses nothing.
//! 3. FLAGGED — the body is dumped under `BREP_HEAL_DUMP`, and a FOLD refuses
//!    here and now: a fold is a trim that was never constructed, not material
//!    to drop.
//! 4. A CROSSING goes to [`resolve_face_crossing`], which returns the body
//!    UNCHANGED when nothing is confirmed, refuses a fold and refuses more than
//!    one crossing pair, and otherwise imprints the two carriers' section onto
//!    both faces and enumerates every way of dropping the resulting pieces —
//!    closed under `Drop` first, and under `Retrim` only where that resolved
//!    nothing at all.
//! 5. The repaired body is SCANNED AGAIN, and a repair that still
//!    self-intersects refuses, naming the original crossing.
//! 6. [`accept_or_break_out`] AGAIN, on the repaired body, so a re-trim that
//!    left a hole off its face is broken out in turn or refused, rather than
//!    shipped.
//!
//! Steps 4 and 6 bound each other, and the boundary is not a gap. The crossing
//! repair needs a section to split along, and a section exists exactly while
//! the moved carrier still MEETS the body; the containment floor decides what
//! is left. Measured on the 2026-09-21 runaway-bore report, the crossing
//! regime ends and the carrier's clearance turns positive in the SAME step of
//! a swept drag, and 13.90 mm past that the repair is handed a body with
//! nothing to imprint and hands it straight back.
//!
//! What step 2's breakout adds is the OTHER side of that boundary: the window
//! where the mouth has crossed its face's edge but the wall's overshoot is
//! still a sliver the tessellation cannot resolve, so the scan confirms
//! nothing and step 4 is never entered. On the reported document that window
//! is `s = 0.99 … 1.03` of the drag, and the scan takes over at `s = 1.04`.
//! The two routes reach the same [`split_and_drop`] and the repair is one
//! repair.
//!
//! ## Cost
//!
//! One tessellation and one BVH sweep per accepted result, and only on the
//! success path. Both verdicts come off that one scan. The containment floor
//! traces the loops of PLANAR faces that have more than one, in their own
//! parameters; a face with a single loop costs nothing.
//!
//! That note used to stop at that sentence, and stopping there was
//! misleading: the floor's cost was never in how MANY faces qualify but in
//! how LARGE one qualifying face's loops can be. A loop is traced at 32
//! stations per KNOT SPAN, so a box's face gives four points and an imported
//! 100-tooth gear's flat gives 1.1 MILLION; testing every hole point against
//! every one of those is a PRODUCT, and three such faces out of 1004 cost 24
//! s of a 30 s replay. The trace is unchanged — decimating it would weaken
//! the predicate — and the product is what went instead: the outer loop is
//! indexed once into bands of its own `v` ([`BoundaryBands`]) and each hole
//! point reads only the band it falls in, for the same answer at `hole
//! points × segments per band`.

use crate::{KernelRefusal, KernelStage, OrRefuse, SoundnessDefect};
use crate::topology::{BrepSolid, LoopRecord};
use crate::{solid_self_intersections, SelfIntersectionOptions, SelfIntersectionReport};

/// Scan a finished result and either accept it, repair it, or refuse it by
/// name.
///
/// `entry` is the lane's own name, so the refusal reads as that lane's
/// refusal rather than as a soundness module's.
pub fn accept_sound(solid: BrepSolid, entry: &str) -> Result<BrepSolid, KernelRefusal> {
    let report = scan(&solid, entry)?;
    if !report.is_flagged() {
        return accept_or_break_out(solid, entry);
    }
    dump_flagged(&solid, entry);
    if let Some(refusal) = fold_refusal(&report, entry) {
        return Err(refusal);
    }
    // Two faces crossing: try to split and trim before refusing.
    let repaired = match resolve_face_crossing(&solid) {
        Ok(repaired) => repaired,
        // The crossing lane declined. Where the containment floor ALSO convicts
        // a hole loop that has left its face, this body may be a BREAKOUT the
        // other route can take — a mouth that has migrated onto a neighbouring
        // face crosses that neighbour, so the scan flags it, and the pair the
        // scan picked is not always the pair the re-trim needs.
        //
        // Guarded three ways, because a second chance is exactly how a body
        // that should refuse gets talked into building. The floor's conviction
        // is the ticket — a body that merely self-intersects never reaches
        // here. The success path above is untouched, so a body that repairs
        // today repairs identically. And the result goes through the same
        // re-scan and the same floor below as any other repair.
        Err(reason) if first_hole_breach(&solid).is_some() => {
            match break_out_until_contained(solid.clone(), entry) {
                Ok(repaired) => repaired,
                Err(second) => {
                    return Err(crossing_refusal(
                        &report,
                        entry,
                        &format!(
                            "{reason}. The breakout route was offered it too and declined: \
                             {second}"
                        ),
                    ))
                }
            }
        }
        Err(reason) => return Err(crossing_refusal(&report, entry, &reason.message)),
    };
    let after = scan(&repaired, entry)?;
    if after.is_flagged() {
        return Err(crossing_refusal(
            &report,
            entry,
            &format!(
                "the repair ran and the result still self-intersects — {}",
                after.summary().unwrap_or_else(|| "no summary".into())
            ),
        ));
    }
    accept_or_break_out(repaired, entry)
}

/// How many hole-containment breaches one acceptance may break out, before it
/// stops and refuses.
///
/// A bore has TWO mouths, and a motion big enough to walk one onto the next
/// face can walk the other too — so one is not enough, and the whole point of a
/// bound is that a repair which does not actually reduce the breaches cannot
/// spin. Four leaves room for a two-mouth migration and a re-trim that opens
/// one more, and stops well short of a body being talked into building by
/// attrition.
const MAX_BREAKOUT_ROUNDS: usize = 4;

/// The containment floor, with the BREAKOUT tried before the refusal.
///
/// A hole loop that has left its face is not one case but two, and they are
/// told apart by where it went. A hole that has left ONTO ANOTHER FACE has a
/// right answer the kernel can build — the moved carrier re-trimmed against the
/// face it now opens through — and this builds it when the mouth STRADDLES its
/// face's boundary, which is the reported case. A hole that has left the body
/// has no such answer, and this refuses exactly as the floor always did, with
/// the repair's own reason for declining carried after the floor's so the
/// refusal says which of the two it was.
///
/// **A mouth that has migrated WHOLLY off its face is built by the CROSSING
/// LANE, not by this route.** Until 2026-09-22 it was built by neither: the
/// departed face kept a hole loop the imprint never cut and the drop
/// enumeration had no move for retiring one, so every candidate face this
/// route offered the carrier to declined. The drop closures have that move now
/// — [`fragment_is_detached`] and [`holds_only_stale_loops`] — and a turned
/// bore whose exit migrates onto the next face is repaired to its closed form
/// through [`resolve_face_crossing`], which `accept_sound` reaches first.
///
/// What is left for this route to decide is the mouth that has arrived on
/// NOTHING: a carrier clear of the body cuts no face of it, every candidate
/// declines, and the refusal names each face it tried and the distance it
/// tried it at. That is measured, not assumed, by
/// `a_mouth_that_has_left_the_body_reaches_the_migration_route_and_is_refused_by_name`.
fn accept_or_break_out(solid: BrepSolid, entry: &str) -> Result<BrepSolid, KernelRefusal> {
    if refuse_holes_leaving_their_faces(&solid, entry).is_ok() {
        return Ok(solid);
    }
    break_out_until_contained(solid, entry)
}

/// Break out every hole the containment floor convicts, one at a time, and hand
/// back a body that BOTH scans clean and passes the floor.
///
/// The rounds are what a two-mouth migration needs: repairing one mouth leaves
/// the other still off its face, and — while it is — the body legitimately
/// still passes through itself, because the second mouth's wall is still
/// standing where the material is not. So an intermediate round is allowed to
/// be flagged; the END of it is not. Nothing is returned that would not have
/// been accepted had it been built that way in the first place.
fn break_out_until_contained(solid: BrepSolid, entry: &str) -> Result<BrepSolid, KernelRefusal> {
    let mut current = solid;
    for _ in 0..MAX_BREAKOUT_ROUNDS {
        let Some(breach) = first_hole_breach(&current) else {
            // Contained. It still has to be SOUND to be returned.
            let report = scan(&current, entry)?;
            if report.is_flagged() {
                return Err(KernelRefusal::unsound(
                    KernelStage::Validate,
                    SoundnessDefect::Crossing,
                    crossing_faces(&report),
                    format!(
                        "{entry}: the breakout repair built a body and it SELF-INTERSECTS — {}",
                        report.summary().unwrap_or_else(|| "no summary".into()),
                    ),
                ));
            }
            return Ok(current);
        };
        // The floor's refusal is the primary one; the breakout's decline is
        // context appended to it, and the class stays the floor's.
        let floor = hole_breach_refusal(&current, &breach, entry);
        current = repair_hole_breakout(&current, &breach).map_err(|reason| {
            floor.with_message(|floor| {
                format!("{floor}. The breakout repair was offered this body and declined it: {reason}")
            })
        })?;
    }
    let breach = first_hole_breach(&current).expect("the loop only falls through on a breach");
    Err(
        hole_breach_refusal(&current, &breach, entry).with_message(|floor| {
            format!(
                "{floor}. {MAX_BREAKOUT_ROUNDS} breakouts were built and a hole is still off its face, so \
                 this stops rather than repairing a body into one nobody asked for"
            )
        }),
    )
}

/// [`accept_sound`] for a lane whose INPUTS may themselves self-intersect.
///
/// A boolean of two bodies that already pass through themselves is entitled to
/// return a result that does: the defect came in with the operand and refusing
/// it here would blame the wrong operation. What must never pass is a crossing
/// the operation ITSELF introduced, so the inputs are scanned only when the
/// output is flagged — the common case pays for one scan, not three.
pub fn accept_sound_against(
    inputs: &[&BrepSolid],
    solid: BrepSolid,
    entry: &str,
) -> Result<BrepSolid, KernelRefusal> {
    let report = scan(&solid, entry)?;
    if !report.is_flagged() {
        return Ok(solid);
    }
    for input in inputs {
        let before = scan(input, entry)?;
        if before.is_flagged() {
            // Said plainly rather than silently: the operand arrived broken.
            return Ok(solid);
        }
    }
    accept_sound(solid, entry)
}

/// The scan, with a read failure reported as a refusal rather than swallowed:
/// a result that could not be scanned is not a result that scanned clean.
fn scan(solid: &BrepSolid, entry: &str) -> Result<SelfIntersectionReport, KernelRefusal> {
    let options = SelfIntersectionOptions::for_solid(solid);
    solid_self_intersections(solid, options).map_err(|message| {
        KernelRefusal::internal(
            KernelStage::Validate,
            "soundness_scan",
            format!("{entry}: the soundness scan could not run: {message}"),
        )
    })
}

/// The faces the scan's confirmed crossings name, each once, in report order.
fn crossing_faces(report: &SelfIntersectionReport) -> Vec<u64> {
    let mut faces = Vec::new();
    for crossing in &report.confirmed {
        for face in [crossing.face_a, crossing.face_b] {
            if !faces.contains(&face) {
                faces.push(face);
            }
        }
    }
    faces
}

/// The refusal for a face folded through itself — named, with both feet, the
/// point and the margins, and pointing at the lane that owns the fix.
fn fold_refusal(report: &SelfIntersectionReport, entry: &str) -> Option<KernelRefusal> {
    let fold = report.folds.first()?;
    let faces = report.folds.iter().map(|fold| fold.face).collect();
    let message = format!(
        "{entry}: the result's face {}{} FOLDS THROUGH ITSELF in 3D — its surface carries \
         ({:.6}, {:.6}) and ({:.6}, {:.6}), {:.3e} apart in its own domain and both inside \
         its trim, to the same point ({:.4}, {:.4}, {:.4}) (residual {:.3e}, branches {:.1} \
         degrees apart){}. The face is wrong in AREA where its volume is not, so nothing that \
         measures volume notices and `validate()` cannot see it at all. A fold is a trim that \
         was never constructed: the folded part of the parameter region has to be carved away \
         where the carrier is built, not repaired out of the finished body.",
        fold.face,
        fold.face_name
            .as_ref()
            .map(|name| format!(" '{name}'"))
            .unwrap_or_default(),
        fold.uv_a[0],
        fold.uv_a[1],
        fold.uv_b[0],
        fold.uv_b[1],
        fold.uv_separation,
        fold.point.x,
        fold.point.y,
        fold.point.z,
        fold.residual,
        fold.normal_angle.to_degrees(),
        if report.folds.len() > 1 {
            format!(" ({} folded faces in all)", report.folds.len())
        } else {
            String::new()
        },
    );
    Some(KernelRefusal::unsound(
        KernelStage::Validate,
        SoundnessDefect::Fold,
        faces,
        message,
    ))
}

/// The refusal for two faces crossing, naming every pair and why the repair
/// did not take it.
fn crossing_refusal(report: &SelfIntersectionReport, entry: &str, reason: &str) -> KernelRefusal {
    let pairs = report
        .confirmed
        .iter()
        .map(|crossing| {
            format!(
                "faces {} and {} near ({:.4}, {:.4}, {:.4}) (straddle {:.3e} against a band of {:.1e})",
                crossing.face_a,
                crossing.face_b,
                crossing.point.x,
                crossing.point.y,
                crossing.point.z,
                crossing.straddle,
                report.options.straddle_band
            )
        })
        .collect::<Vec<_>>()
        .join("; ");
    KernelRefusal::unsound(
        KernelStage::Validate,
        SoundnessDefect::Crossing,
        crossing_faces(report),
        format!(
            "{entry}: the result SELF-INTERSECTS — {pairs}. The enclosed space can still measure \
             right, so neither a volume oracle nor `validate()` would have caught this. The repair \
             that splits both faces along their carriers' intersection and drops the enclosed \
             material did not take it: {reason}"
        ),
    )
}

// ---------------------------------------------------------------------------
// The hole-containment floor
// ---------------------------------------------------------------------------

/// Stations per curved pcurve span when a loop is traced in its face's own
/// parameters. A straight pcurve contributes its start point only — the next
/// coedge begins where it ends.
const LOOP_SAMPLES_PER_SPAN: usize = 32;

/// A loop of a face as a closed polyline in that face's own parameter plane,
/// traced coedge by coedge along each pcurve (already oriented to its coedge).
fn loop_parameter_polyline(loop_record: &LoopRecord) -> Result<Vec<(f64, f64)>, KernelRefusal> {
    pcurve_polyline(loop_record.coedges.iter().map(|coedge| &coedge.pcurve))
}

/// The same walk for a FRAGMENT's loop, which carries the same pcurves under a
/// different record. One sampler, so a loop traced during the repair and the
/// same loop traced by the containment floor are the same polyline.
fn fragment_loop_polyline(
    fragment_loop: &crate::fragment::FragmentLoop,
) -> Result<Vec<(f64, f64)>, KernelRefusal> {
    pcurve_polyline(fragment_loop.coedges.iter().map(|coedge| &coedge.pcurve))
}

fn pcurve_polyline<'a>(
    pcurves: impl Iterator<Item = &'a crate::NurbsCurve>,
) -> Result<Vec<(f64, f64)>, KernelRefusal> {
    let mut points = Vec::new();
    for pcurve in pcurves {
        let [d0, d1] = pcurve.domain().or_refuse(KernelStage::Validate, "domain")?;
        let steps = if pcurve.degree <= 1 && pcurve.control_points.len() == 2 {
            1
        } else {
            let mut spans = 0usize;
            for window in pcurve.knots.windows(2) {
                if window[1] > window[0] {
                    spans += 1;
                }
            }
            spans.max(1) * LOOP_SAMPLES_PER_SPAN
        };
        for step in 0..steps {
            let uv = pcurve.evaluate(d0 + (d1 - d0) * (step as f64 / steps as f64)).or_refuse(KernelStage::Validate, "evaluate")?;
            points.push((uv.x, uv.y));
        }
    }
    Ok(points)
}

fn polyline_area(points: &[(f64, f64)]) -> f64 {
    let mut twice = 0.0;
    for index in 0..points.len() {
        let (x0, y0) = points[index];
        let (x1, y1) = points[(index + 1) % points.len()];
        twice += x0 * y1 - x1 * y0;
    }
    0.5 * twice
}

/// The outer loop's segments, bucketed by the band of `v` each one spans.
///
/// The floor asks a hole point two questions — even-odd parity against the
/// outer loop, and whether the point sits within `tolerance` of it — and both
/// read ONLY the segments whose own `v` range reaches the point's `v`. A
/// segment that does not reach it has both endpoints on the same side of the
/// horizontal ray, so it cannot toggle the parity; and every point of it is
/// further away than `tolerance` in `v` alone, so it cannot be the witness
/// that the point is on the boundary. Answering out of the band is therefore
/// the SAME answer as walking the whole loop, not an approximation of it: the
/// set of segments that can toggle is identical, parity is an XOR and does not
/// care in what order they come, and the nearest segment — if there is one
/// within `tolerance` at all — is among those searched.
///
/// What changes is the COST. The brute-force walk is `hole points × outer
/// points`, which on an imported 100-tooth gear's flat is 1.1 million outer
/// stations against 11 904 hole stations — 1.3e10 pairs on ONE face, 24 s of a
/// 30 s replay. Through the bands it is `hole points × segments per band`,
/// plus one linear pass to build.
///
/// Laid out CSR — one offsets array and one flat segment array — because a
/// loop of a million points would otherwise want a million `Vec` headers.
struct BoundaryBands<'a> {
    points: &'a [(f64, f64)],
    /// The bottom of the first band: the loop's lowest `v`.
    v_lo: f64,
    /// Bands per unit of `v`. Zero when the loop has no `v` extent to divide,
    /// in which case there is one band and everything lands in it.
    bands_per_v: f64,
    /// `starts[band] .. starts[band + 1]` indexes `segments`.
    starts: Vec<u32>,
    /// Segment `i` is `points[i] -> points[(i + 1) % len]`, the closing
    /// segment included.
    segments: Vec<u32>,
}

impl<'a> BoundaryBands<'a> {
    fn build(points: &'a [(f64, f64)]) -> Self {
        let count = points.len();
        debug_assert!(count < u32::MAX as usize, "segment indices are u32");
        let (mut v_lo, mut v_hi) = (f64::INFINITY, f64::NEG_INFINITY);
        let mut variation = 0.0;
        for index in 0..count {
            let (_, y0) = points[index];
            let (_, y1) = points[(index + 1) % count];
            v_lo = v_lo.min(y0);
            v_hi = v_hi.max(y0);
            variation += (y1 - y0).abs();
        }
        // One band per `variation / count` of `v`. That width is what makes the
        // index linear in the loop whatever the loop's shape: a segment lands
        // in `its own v extent / band width + 2` bands, and those sum to about
        // `3 × count` however the vertical variation is distributed — a gear
        // tooth's thousands of short segments and a box's four long ones alike.
        let span = v_hi - v_lo;
        let bands = if span.is_finite() && span > 0.0 && variation > 0.0 {
            let wanted = span * count as f64 / variation;
            if wanted.is_finite() && wanted >= 1.0 {
                (wanted as usize).min(count.max(1))
            } else {
                1
            }
        } else {
            1
        };
        let bands_per_v = if bands > 1 { bands as f64 / span } else { 0.0 };
        let band_of = |v: f64| -> usize {
            if bands_per_v == 0.0 {
                return 0;
            }
            let raw = (v - v_lo) * bands_per_v;
            // Written so a NaN takes the first branch rather than falling
            // through to an `as usize` cast of it.
            if !(raw > 0.0) {
                0
            } else if raw >= (bands - 1) as f64 {
                bands - 1
            } else {
                raw as usize
            }
        };
        let mut starts = vec![0u32; bands + 1];
        for index in 0..count {
            let (_, y0) = points[index];
            let (_, y1) = points[(index + 1) % count];
            let (lo, hi) = (band_of(y0.min(y1)), band_of(y0.max(y1)));
            for start in starts[lo + 1..=hi + 1].iter_mut() {
                *start += 1;
            }
        }
        for band in 1..=bands {
            starts[band] += starts[band - 1];
        }
        let mut cursor = starts.clone();
        let mut segments = vec![0u32; starts[bands] as usize];
        for index in 0..count {
            let (_, y0) = points[index];
            let (_, y1) = points[(index + 1) % count];
            let (lo, hi) = (band_of(y0.min(y1)), band_of(y0.max(y1)));
            for band in lo..=hi {
                segments[cursor[band] as usize] = index as u32;
                cursor[band] += 1;
            }
        }
        Self {
            points,
            v_lo,
            bands_per_v,
            starts,
            segments,
        }
    }

    fn band_count(&self) -> usize {
        self.starts.len() - 1
    }

    /// The band `v` falls in, clamped. A `v` outside the loop's own extent
    /// clamps to an end band, whose segments then fail the tests on their own
    /// terms — nothing straddles a ray the whole loop sits above or below.
    fn band_of(&self, v: f64) -> usize {
        if self.bands_per_v == 0.0 {
            return 0;
        }
        let bands = self.band_count();
        let raw = (v - self.v_lo) * self.bands_per_v;
        if !(raw > 0.0) {
            0
        } else if raw >= (bands - 1) as f64 {
            bands - 1
        } else {
            raw as usize
        }
    }

    fn segment(&self, index: u32) -> ((f64, f64), (f64, f64)) {
        let index = index as usize;
        (
            self.points[index],
            self.points[(index + 1) % self.points.len()],
        )
    }

    /// Even-odd parity: is `(u, v)` inside the closed loop?
    fn contains(&self, point: (f64, f64)) -> bool {
        let (u, v) = point;
        let band = self.band_of(v);
        let mut inside = false;
        for &index in &self.segments[self.starts[band] as usize..self.starts[band + 1] as usize] {
            let ((x0, y0), (x1, y1)) = self.segment(index);
            if (y0 > v) != (y1 > v) && u < x0 + (v - y0) * (x1 - x0) / (y1 - y0) {
                inside = !inside;
            }
        }
        inside
    }

    /// Does `(u, v)` sit within `tolerance` of the loop — is it ON the
    /// boundary rather than off the face?
    ///
    /// The bands searched are widened by one on each side of the `tolerance`
    /// band, so the margin against a segment being excluded by a rounding of
    /// its own `v` extent is a whole band rather than an ulp.
    fn within(&self, point: (f64, f64), tolerance: f64) -> bool {
        let (u, v) = point;
        let lo = self.band_of(v - tolerance).saturating_sub(1);
        let hi = (self.band_of(v + tolerance) + 1).min(self.band_count() - 1);
        for band in lo..=hi {
            for &index in &self.segments[self.starts[band] as usize..self.starts[band + 1] as usize]
            {
                let ((x0, y0), (x1, y1)) = self.segment(index);
                let (dx, dy) = (x1 - x0, y1 - y0);
                let length_squared = dx * dx + dy * dy;
                let t = if length_squared > 0.0 {
                    (((u - x0) * dx + (v - y0) * dy) / length_squared).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let (px, py) = (x0 + t * dx, y0 + t * dy);
                if ((u - px).powi(2) + (v - py).powi(2)).sqrt() <= tolerance {
                    return true;
                }
            }
        }
        false
    }
}

/// THE HOLE-CONTAINMENT FLOOR: a PLANAR face whose hole loop has left that
/// face's own outer loop is refused, naming the hole.
///
/// A hole is a hole IN something. Carry a bore's mouth off the flat it opens
/// through — a direct edit dragging the wall clear of the body — and what the
/// fixed-topology road emits is the mouth loop hanging in the plane beside the
/// face it is supposed to perforate, with the wall it bounds somewhere else
/// entirely. Every other floor is blind to it: `validate()` tests INCIDENCE and
/// the incidences are all intact; the scan needs two faces to CROSS and the
/// runaway bore crosses nothing; and the divergence-theorem volume of a
/// cylinder does not depend on where its axis sits, so the body can measure to
/// the last bit of the right answer while being the wrong body. The correct
/// result needs the hole to break through a neighbouring face — a topology
/// change the direct edits do not make — so this refuses instead of shipping it.
///
/// Decided in the face's own parameter plane, which is affine to 3D for a
/// plane, by parity against the outer loop (the one enclosing the largest
/// parameter area). A point that fails parity but sits within `tolerance` of
/// the outer loop is treated as ON it, so a mouth tangent to its face's own
/// boundary is not convicted by sampling noise. The predicate is only as fine
/// as the sampling: a curved hole poking out by less than its sampled chord sag
/// can still pass.
///
/// Both questions go through [`BoundaryBands`], which is an INDEX over the
/// outer loop and not a coarser reading of it: the answers are the ones a walk
/// of the whole traced polyline gives, off the whole traced polyline, at a
/// cost that is no longer the product of the two loops' lengths.
///
/// This runs at the END of [`accept_sound`], on the body the repair left, so a
/// crossing the repair can take is still taken — the reason an earlier
/// primitive-level copy of this floor was removed.
fn refuse_holes_leaving_their_faces(solid: &BrepSolid, entry: &str) -> Result<(), KernelRefusal> {
    match first_hole_breach(solid) {
        Some(breach) => Err(hole_breach_refusal(solid, &breach, entry)),
        None => Ok(()),
    }
}

/// A hole loop the floor has convicted: WHICH face, which loop of it, and the
/// sample that fell outside, in that face's own parameters.
///
/// The floor's verdict is a yes or no; this is the same verdict with the
/// witness kept, so a lane that can BUILD the right body has something to
/// build it from. Nothing about the predicate changes by recording it.
struct HoleBreach {
    face: u64,
    loop_index: usize,
    loop_id: u64,
    at: (f64, f64),
}

/// THE FLOOR'S PREDICATE, verbatim — the first hole loop on a planar face that
/// has left that face's own outer loop, or `None`.
///
/// This is the whole of the decision [`refuse_holes_leaving_their_faces`] used
/// to make inline; the refusal is now written by [`hole_breach_refusal`] off
/// what this returns. Same loops, same trace, same bands, same tolerance, same
/// first-hit order.
fn first_hole_breach(solid: &BrepSolid) -> Option<HoleBreach> {
    for shell in &solid.shells {
        for face in &shell.faces {
            if face.loops.len() < 2 {
                continue;
            }
            if !matches!(
                face.surface.analytic(),
                Some(crate::AnalyticSurface::Plane { .. })
            ) {
                continue;
            }
            let polylines = face
                .loops
                .iter()
                .map(loop_parameter_polyline)
                .collect::<Result<Vec<_>, KernelRefusal>>()
                // A loop this cannot be traced from is not a loop this can
                // convict: the acceptance's own floors already ran.
                .unwrap_or_default();
            if polylines.len() != face.loops.len() || polylines.iter().any(Vec::is_empty) {
                continue;
            }
            let outer_index = (0..polylines.len())
                .max_by(|&a, &b| {
                    polyline_area(&polylines[a])
                        .abs()
                        .total_cmp(&polyline_area(&polylines[b]).abs())
                })
                .unwrap_or(0);
            let outer = &polylines[outer_index];
            // Every hole point is asked the same two questions against this
            // one loop, so the loop is indexed ONCE and each question then
            // reads the band it falls in rather than the whole polyline. The
            // answers are the ones the whole-polyline walk gives; see
            // [`BoundaryBands`].
            let bands = BoundaryBands::build(outer);
            let tolerance = boundary_tolerance(outer);
            for (index, hole) in polylines.iter().enumerate() {
                if index == outer_index {
                    continue;
                }
                let Some(&at) = hole
                    .iter()
                    .find(|&&point| !bands.contains(point) && !bands.within(point, tolerance))
                else {
                    continue;
                };
                return Some(HoleBreach {
                    face: face.id,
                    loop_index: index,
                    loop_id: face.loops[index].id,
                    at,
                });
            }
        }
    }
    None
}

/// The "on the boundary" band, measured against the outer loop's own parameter
/// extent so it means the same thing on a 1 mm face and on a 1 m one.
fn boundary_tolerance(outer: &[(f64, f64)]) -> f64 {
    let (mut lo, mut hi) = ((f64::MAX, f64::MAX), (f64::MIN, f64::MIN));
    for &(u, v) in outer {
        lo = (lo.0.min(u), lo.1.min(v));
        hi = (hi.0.max(u), hi.1.max(v));
    }
    let extent = (hi.0 - lo.0).max(hi.1 - lo.1);
    (extent * 1e-9).max(1e-12)
}

/// The floor's refusal, written off the breach it convicted.
fn hole_breach_refusal(solid: &BrepSolid, breach: &HoleBreach, entry: &str) -> KernelRefusal {
    let face = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == breach.face);
    let label = match face.and_then(|face| face.name.as_deref()) {
        Some(name) => format!("`{name}` (face {})", breach.face),
        None => format!("face {}", breach.face),
    };
    let (u, v) = breach.at;
    let where_in_space = face
        .and_then(|face| face.surface.evaluate(u, v).ok())
        .map(|point| format!(" — ({:.4}, {:.4}, {:.4}) in space", point.x, point.y, point.z))
        .unwrap_or_default();
    KernelRefusal::unsound(
        KernelStage::Validate,
        SoundnessDefect::HoleBreach,
        vec![breach.face],
        format!(
            "{entry}: hole loop {} of {label} has LEFT that face — it runs outside the \
             face's own outer boundary, at ({u:.6}, {v:.6}) in the face's parameters{}. \
             A hole is a hole IN a face; this one bounds a wall that no longer opens \
             through it, so the right body needs the hole to break through a \
             neighbouring face, a topology change this operation does not make. \
             `validate()` cannot see it (every incidence is intact), the \
             self-intersection scan cannot (nothing crosses), and the volume need not \
             change at all — refusing rather than returning the wrong body",
            breach.loop_id, where_in_space,
        ),
    )
}

// ---------------------------------------------------------------------------
// The breakout route: a hole that has left its face onto a NAMED neighbour
// ---------------------------------------------------------------------------

/// The faces a breakout is between: the WALL the convicted hole loop bounds,
/// and the ONE neighbour of the breached face the hole has run out onto.
struct BreakoutPair {
    wall: u64,
    neighbour: u64,
}

/// WHICH neighbour the runaway hole has gone onto — read in the breached
/// face's own parameter plane, where the question is a segment crossing and
/// not a guess.
///
/// The hole loop bounds a wall (for a bore, the bore's own cylindrical face):
/// exactly one other face answers for its edges, and that is the face whose
/// carrier has to be re-trimmed. The hole has left through some stretch of the
/// breached face's OUTER loop, and each outer coedge is shared with exactly one
/// neighbouring face, so the coedges the hole polyline crosses name the faces
/// it has gone out onto.
///
/// This refuses rather than choosing whenever the answer is not a single pair:
/// a hole bounding more than one face, a hole that crosses no outer coedge at
/// all (it has left the face's own parameter patch entirely — the runaway), and
/// a hole crossing into MORE THAN ONE neighbour, which is a breakout over a
/// corner and needs both re-trims resolved together rather than one at a time.
fn breakout_pair(solid: &BrepSolid, breach: &HoleBreach) -> Result<BreakoutPair, KernelRefusal> {
    let wall = breakout_wall(solid, breach)?;
    match breakout_neighbours_crossed(solid, breach, wall)?[..] {
        [neighbour] => Ok(BreakoutPair { wall, neighbour }),
        [] => Err(KernelRefusal::unsupported(KernelStage::Validate, NOT_A_STRADDLE_WHAT, NOT_A_STRADDLE)),
        ref many => Err(KernelRefusal::unsupported(KernelStage::Validate, "breakout.pair", format!(
            "the runaway hole crosses into {} neighbouring faces at once ({many:?}) — a \
             breakout over a corner, where the two re-trims bound each other and resolving one \
             at a time is a different problem from resolving both",
            many.len(),
        ))),
    }
}

/// The marker [`breakout_pair`] returns when the hole does not STRADDLE its
/// face's boundary. It has either migrated wholly onto another face or left the
/// body; [`repair_hole_breakout`] tells those apart, by trying.
const NOT_A_STRADDLE: &str = "the hole does not straddle its own face's boundary";

/// The slug [`breakout_pair`] mints [`NOT_A_STRADDLE`] under
/// (`UnsupportedGeometry`), shared with [`is_not_a_straddle`].
const NOT_A_STRADDLE_WHAT: &str = "breakout.not_a_straddle";

/// Is this [`breakout_pair`]'s non-straddle answer? Read off the class and
/// slug, so the route's decision does not follow the sentence.
fn is_not_a_straddle(refusal: &KernelRefusal) -> bool {
    matches!(
        &refusal.class,
        crate::RefusalClass::UnsupportedGeometry { what } if what == NOT_A_STRADDLE_WHAT
    )
}

/// The face the convicted hole loop BOUNDS — the moved carrier, for a bore the
/// bore's own wall. Exactly one other face may answer for its edges.
fn breakout_wall(solid: &BrepSolid, breach: &HoleBreach) -> Result<u64, KernelRefusal> {
    let face = face_by_id(solid, breach.face)?;
    let mut walls: Vec<u64> = Vec::new();
    for coedge in &face.loops[breach.loop_index].coedges {
        for other in solid.shells.iter().flat_map(|shell| &shell.faces) {
            if other.id == face.id {
                continue;
            }
            if other
                .loops
                .iter()
                .any(|wire| wire.coedges.iter().any(|use_| use_.edge_id == coedge.edge_id))
                && !walls.contains(&other.id)
            {
                walls.push(other.id);
            }
        }
    }
    match walls[..] {
        [wall] => Ok(wall),
        _ => Err(KernelRefusal::unsupported(KernelStage::Validate, "breakout.wall_count", format!(
            "the runaway hole loop bounds {} face(s) ({walls:?}); this route re-trims ONE moved \
             carrier against the body and cannot decide which of several is the one that moved",
            walls.len(),
        ))),
    }
}

fn face_by_id<'a>(
    solid: &'a BrepSolid,
    id: u64,
) -> Result<&'a crate::FaceRecord, KernelRefusal> {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == id)
        .ok_or_else(|| {
            KernelRefusal::internal(KernelStage::Validate, "face_by_id", format!("face {id} is not on the body"))
        })
}

/// The neighbours the hole has run out ONTO, read as segment crossings in the
/// breached face's own parameter plane.
///
/// This answers the STRADDLE: part of the mouth still inside the face it was
/// recorded on, part outside. It returns nothing for a mouth that has migrated
/// wholly onto another face — such a loop is entirely outside the outer loop
/// and crosses none of its coedges — and nothing for one that has left the body
/// altogether. Those two are told apart downstream, by geometry, in
/// [`repair_hole_breakout`].
fn breakout_neighbours_crossed(
    solid: &BrepSolid,
    breach: &HoleBreach,
    wall: u64,
) -> Result<Vec<u64>, KernelRefusal> {
    let face = face_by_id(solid, breach.face)?;
    let polylines = face
        .loops
        .iter()
        .map(loop_parameter_polyline)
        .collect::<Result<Vec<_>, KernelRefusal>>()
        .map_err(|error| {
            error.with_message(|error| format!("the breached face's loops could not be traced: {error}"))
        })?;
    let outer_index = (0..polylines.len())
        .max_by(|&a, &b| {
            polyline_area(&polylines[a])
                .abs()
                .total_cmp(&polyline_area(&polylines[b]).abs())
        })
        .unwrap_or(0);
    if outer_index == breach.loop_index {
        return Err(KernelRefusal::internal(KernelStage::Validate, "breakout.outer_loop", "the convicted loop is the face's own outer loop"));
    }
    let hole = &polylines[breach.loop_index];

    let mut neighbours: Vec<u64> = Vec::new();
    for coedge in &face.loops[outer_index].coedges {
        let side = coedge_parameter_polyline(coedge)
            .map_err(|error| error.with_message(|error| format!("an outer coedge could not be traced: {error}")))?;
        if !polylines_cross(hole, &side) {
            continue;
        }
        for other in solid.shells.iter().flat_map(|shell| &shell.faces) {
            if other.id == face.id || other.id == wall {
                continue;
            }
            if other
                .loops
                .iter()
                .any(|wire| wire.coedges.iter().any(|use_| use_.edge_id == coedge.edge_id))
                && !neighbours.contains(&other.id)
            {
                neighbours.push(other.id);
            }
        }
    }
    Ok(neighbours)
}

/// Every face across the breached face's OUTER boundary — the faces a mouth
/// leaving that face can have arrived on.
///
/// A migration is a mouth that has walked off one face and onto the next. The
/// next one is over an edge, so this is the candidate set, and it is small: the
/// outer loop's own neighbours, the wall excepted. A carrier that has reached
/// something further away than that is not a case this route claims, and it
/// says so rather than widening until it finds a face it can cut.
fn faces_over_the_outer_boundary(
    solid: &BrepSolid,
    breach: &HoleBreach,
    wall: u64,
) -> Result<Vec<u64>, KernelRefusal> {
    let face = face_by_id(solid, breach.face)?;
    let polylines = face
        .loops
        .iter()
        .map(loop_parameter_polyline)
        .collect::<Result<Vec<_>, KernelRefusal>>()
        .unwrap_or_default();
    let outer_index = (0..polylines.len())
        .max_by(|&a, &b| {
            polyline_area(&polylines[a])
                .abs()
                .total_cmp(&polyline_area(&polylines[b]).abs())
        })
        .unwrap_or(0);
    let mut over: Vec<u64> = Vec::new();
    for coedge in &face.loops[outer_index].coedges {
        for other in solid.shells.iter().flat_map(|shell| &shell.faces) {
            if other.id == face.id || other.id == wall {
                continue;
            }
            if other
                .loops
                .iter()
                .any(|wire| wire.coedges.iter().any(|use_| use_.edge_id == coedge.edge_id))
                && !over.contains(&other.id)
            {
                over.push(other.id);
            }
        }
    }
    Ok(over)
}

/// One coedge as a polyline in its face's parameters, END POINT INCLUDED, so
/// the segments it yields are the whole of that coedge.
///
/// [`loop_parameter_polyline`] deliberately leaves each coedge's end point to
/// the next coedge's start; a single coedge asked about on its own has no next.
fn coedge_parameter_polyline(
    coedge: &crate::topology::CoedgeRecord,
) -> Result<Vec<(f64, f64)>, KernelRefusal> {
    let pcurve = &coedge.pcurve;
    let [d0, d1] = pcurve.domain().or_refuse(KernelStage::Validate, "domain")?;
    let steps = if pcurve.degree <= 1 && pcurve.control_points.len() == 2 {
        1
    } else {
        let mut spans = 0usize;
        for window in pcurve.knots.windows(2) {
            if window[1] > window[0] {
                spans += 1;
            }
        }
        spans.max(1) * LOOP_SAMPLES_PER_SPAN
    };
    let mut points = Vec::with_capacity(steps + 1);
    for step in 0..=steps {
        let uv = pcurve.evaluate(d0 + (d1 - d0) * (step as f64 / steps as f64)).or_refuse(KernelStage::Validate, "evaluate")?;
        points.push((uv.x, uv.y));
    }
    Ok(points)
}

/// Does the CLOSED polyline `loop_points` cross the OPEN polyline `side`?
/// Proper segment intersection, in the face's own parameters.
fn polylines_cross(loop_points: &[(f64, f64)], side: &[(f64, f64)]) -> bool {
    for index in 0..loop_points.len() {
        let a = loop_points[index];
        let b = loop_points[(index + 1) % loop_points.len()];
        for window in side.windows(2) {
            if segments_cross(a, b, window[0], window[1]) {
                return true;
            }
        }
    }
    false
}

fn segments_cross(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let orient = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| {
        (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0)
    };
    let (d1, d2) = (orient(c, d, a), orient(c, d, b));
    let (d3, d4) = (orient(a, b, c), orient(a, b, d));
    ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0))
}

/// THE BREAKOUT, reached from the containment floor instead of from the scan.
///
/// The floor convicts a hole loop that has left the planar face it is recorded
/// on. Where that hole has left ONTO a named neighbour — the case the operator
/// reported, an exit hole dragged across the edge between two faces — the right
/// body is the one the self-crossing repair already builds for a bore carried
/// out through a side wall: the moved carrier and the neighbour split along
/// their exact section, the pieces that ended up on the wrong side dropped, and
/// the faces left holding an unanswered boundary RE-TRIMMED, which merges the
/// mouth into the outer loops of the faces it now opens through.
///
/// What makes it the SAME repair and not a second one is that the extent is
/// still derived and not invented: the tool is the moved carrier trimmed
/// against the body, and where it meets the body is what the imprint computes.
/// The route in is what changes — the scan needs a mesh crossing it can
/// confirm, and at the near edge of a breakout the crossing is a sliver the
/// tessellation does not resolve, so the floor sees it first.
///
/// The genuine runaway is excluded by GEOMETRY and not by a threshold. A bore
/// carried clean off the body crosses no boundary of the face it left, so
/// [`breakout_pair`] finds no neighbour and this refuses before a single face
/// is split; and even given a pair, the imprint of a carrier that misses the
/// body cuts neither trim and [`split_and_drop`] declines on its own terms.
fn repair_hole_breakout(solid: &BrepSolid, breach: &HoleBreach) -> Result<BrepSolid, KernelRefusal> {
    let origin = || RepairOrigin::Breakout {
        face: breach.face,
        loop_id: breach.loop_id,
    };
    // THE STRADDLE: part of the mouth still inside the face it was recorded on,
    // part outside. Which neighbour is read off the crossing, exactly.
    match breakout_pair(solid, breach) {
        Ok(BreakoutPair { wall, neighbour }) => {
            return split_and_drop(solid, wall, neighbour, origin())
        }
        // Every refusal but the non-straddle is this route's own, classed as it
        // always was here; the non-straddle is read off its slug, not its text.
        Err(reason) if !is_not_a_straddle(&reason) => {
            return Err(KernelRefusal::unsupported(
                KernelStage::Validate,
                "breakout.pair",
                reason.message,
            ))
        }
        Err(_) => {}
    }

    // THE MIGRATION, or the RUNAWAY. The mouth is wholly outside the face it
    // was recorded on, so nothing in that face's own parameter plane can say
    // where it went — a mouth that has walked onto the next face and a mouth
    // that has left the solid look the same from there.
    //
    // They do not look the same to the IMPRINT. The question that separates
    // them is whether the moved carrier still meets a face of the body, and
    // `split_and_drop` answers exactly that: handed a face the carrier misses
    // it declines with "the carriers' intersection reaches ... neither", before
    // anything is dropped. So the candidates are tried, and the answer is
    // decided BY THE RESULT — the same principle the drop enumeration inside
    // already runs on, and a discriminator that cannot drift, because a carrier
    // clear of the body cuts nothing anywhere.
    //
    // More than one candidate building is not an answer: the mouth would be on
    // two faces at once with no reason to prefer either, and choosing would be
    // the guess this module exists not to make.
    let wall = breakout_wall(solid, breach).map_err(|refusal| {
        KernelRefusal::internal(KernelStage::Validate, "breakout.wall", refusal.message)
    })?;
    let mut candidates = faces_over_the_outer_boundary(solid, breach, wall)
        .map_err(|refusal| {
            KernelRefusal::internal(KernelStage::Validate, "breakout.candidates", refusal.message)
        })?;
    if candidates.is_empty() {
        return Err(KernelRefusal::unsupported(
            KernelStage::Validate,
            "breakout.runaway",
            RUNAWAY,
        ));
    }

    // WHICH face the mouth moved onto, when its own face's parameter plane can
    // no longer say. The carrier is a whole surface and generally meets the
    // body in more than one place — a through bore leaves by one face and
    // enters by another, and both are faces this carrier can be re-trimmed
    // against — so "some face it cuts" is not the answer. The answer is the
    // face the MIGRATED MOUTH IS AT: the candidates are ordered by the least
    // 3D distance from the convicted hole loop to the face's own trim, and the
    // nearest one that the repair can actually build is taken.
    //
    // A TIE is refused rather than broken. Two faces the same distance from the
    // mouth means the mouth sits on the edge between them, which is a corner
    // breakout — the case whose two re-trims bound each other.
    let mouth = hole_loop_in_space(solid, breach).map_err(|refusal| {
        KernelRefusal::internal(KernelStage::Validate, "breakout.mouth", refusal.message)
    })?;
    let scale = crate::solid_scale(solid);
    let mut ranked: Vec<(f64, u64)> = Vec::new();
    for candidate in candidates.drain(..) {
        ranked.push((
            distance_to_face(solid, candidate, &mouth)
                .map_err(|refusal| {
                    KernelRefusal::internal(KernelStage::Validate, "breakout.distance", refusal.message)
                })?,
            candidate,
        ));
    }
    ranked.sort_by(|a, b| a.0.total_cmp(&b.0));
    if ranked.len() > 1 && (ranked[1].0 - ranked[0].0).abs() <= scale * 1e-9 {
        return Err(KernelRefusal::ill_posed(
            KernelStage::Validate,
            "breakout.corner_tie",
            format!(
                "the migrated mouth is the same distance ({:.6}) from faces {} and {}, so the body \
                 does not say which of them it moved onto — a breakout over the corner between \
                 them, where the two re-trims bound each other",
                ranked[0].0, ranked[0].1, ranked[1].1,
            ),
        ));
    }

    let carried = take_crossing_repairs();
    let mut declined: Vec<String> = Vec::new();
    for (distance, candidate) in ranked {
        match split_and_drop(solid, wall, candidate, origin()) {
            Ok(body) => {
                // The trial that built pushed a record; the ones that declined
                // pushed none. Keep that one and put the caller's ledger back.
                let mine = take_crossing_repairs();
                restore_repairs(carried.into_iter().chain(mine).collect());
                return Ok(body);
            }
            Err(reason) => {
                declined.push(format!("face {candidate} at {distance:.6}: {reason}"))
            }
        }
    }
    restore_repairs(carried);
    Err(KernelRefusal::unsupported(
        KernelStage::Validate,
        "breakout.runaway",
        format!(
            "{RUNAWAY} — the moved carrier was offered every face across that boundary, nearest \
             first, and cut none of them ({})",
            declined.join(" | ")
        ),
    ))
}

/// The convicted hole loop as 3D points: its pcurves are in the breached face's
/// own parameters, so they are read back through that face's surface — the same
/// road the floor takes, and the reason the floor was never wrong about where a
/// loop is.
fn hole_loop_in_space(solid: &BrepSolid, breach: &HoleBreach) -> Result<Vec<crate::Vec3>, KernelRefusal> {
    let face = face_by_id(solid, breach.face)?;
    let uv = loop_parameter_polyline(&face.loops[breach.loop_index])?;
    uv.into_iter()
        .map(|(u, v)| face.surface.evaluate(u, v))
        .collect::<Result<Vec<_>, _>>()
        .map_err(|error| format!("the convicted hole loop could not be read in space: {error}"))
        .or_refuse(KernelStage::Validate, "evaluate")
}

/// The least distance from a set of points to a face's own TRIM — sampled, in
/// the face's parameters, so a point beside a face's carrier but off the end of
/// the face does not read as being on it.
fn distance_to_face(
    solid: &BrepSolid,
    face_id: u64,
    points: &[crate::Vec3],
) -> Result<f64, KernelRefusal> {
    const STATIONS: usize = 32;
    let face = face_by_id(solid, face_id)?;
    let polylines = face
        .loops
        .iter()
        .map(loop_parameter_polyline)
        .collect::<Result<Vec<_>, KernelRefusal>>()?;
    let outer_index = (0..polylines.len())
        .max_by(|&a, &b| {
            polyline_area(&polylines[a])
                .abs()
                .total_cmp(&polyline_area(&polylines[b]).abs())
        })
        .unwrap_or(0);
    let bands = BoundaryBands::build(&polylines[outer_index]);
    let (mut lo, mut hi) = ((f64::MAX, f64::MAX), (f64::MIN, f64::MIN));
    for &(u, v) in &polylines[outer_index] {
        lo = (lo.0.min(u), lo.1.min(v));
        hi = (hi.0.max(u), hi.1.max(v));
    }
    let mut best = f64::INFINITY;
    for iu in 0..=STATIONS {
        for iv in 0..=STATIONS {
            let u = lo.0 + (hi.0 - lo.0) * iu as f64 / STATIONS as f64;
            let v = lo.1 + (hi.1 - lo.1) * iv as f64 / STATIONS as f64;
            if !bands.contains((u, v)) {
                continue;
            }
            if polylines
                .iter()
                .enumerate()
                .any(|(index, hole)| index != outer_index && inside_polyline((u, v), hole))
            {
                continue;
            }
            let Ok(point) = face.surface.evaluate(u, v) else {
                continue;
            };
            for probe in points {
                best = best.min(probe.sub(point).length());
            }
        }
    }
    Ok(best)
}

/// Even-odd parity against a closed polyline, for the hole loops the band index
/// is not built over.
fn inside_polyline(point: (f64, f64), polygon: &[(f64, f64)]) -> bool {
    let (u, v) = point;
    let mut parity = false;
    for index in 0..polygon.len() {
        let (ax, ay) = polygon[index];
        let (bx, by) = polygon[(index + 1) % polygon.len()];
        if (ay > v) != (by > v) {
            let x = ax + (v - ay) / (by - ay) * (bx - ax);
            if x > u {
                parity = !parity;
            }
        }
    }
    parity
}

/// The sentence a hole that has left the BODY earns, as opposed to one that has
/// moved onto another face. It is the one thing standing between the operator
/// and a silently ruined model, so it is a constant and not a phrasing.
const RUNAWAY: &str = "the runaway hole has left the face it was recorded on without arriving \
     on another: the moved carrier meets no face of the body, so there is nothing for it to be \
     re-trimmed against and no cut to perform at any extent";

/// [`resolve_face_crossing`] for a pair this module has named itself rather
/// than read off the scan.
///
/// The scan is one way to find two faces of one body that have to be split
/// along their section, and the containment floor is another. Both hand the
/// same pair to the same repair.
pub fn resolve_face_crossing_between(
    solid: &BrepSolid,
    face_a: u64,
    face_b: u64,
) -> Result<BrepSolid, KernelRefusal> {
    let report = scan(solid, "resolve_face_crossing_between")?;
    split_and_drop(solid, face_a, face_b, RepairOrigin::Scan(&report))
}

// ---------------------------------------------------------------------------
// The repair
// ---------------------------------------------------------------------------

/// WHAT FOUND the pair this repair is about to split.
///
/// Two floors reach the same repair. The scan finds two faces whose meshes
/// cross and can say where and by how much; the hole-containment floor finds a
/// hole loop that has left its face and can say which loop and which face. The
/// repair does the same work either way, and the ledger says which asked for
/// it rather than printing a crossing point that was never measured.
enum RepairOrigin<'a> {
    /// The self-intersection scan confirmed a crossing pair.
    Scan(&'a SelfIntersectionReport),
    /// The containment floor convicted a hole loop that has broken out onto a
    /// neighbouring face.
    Breakout { face: u64, loop_id: u64 },
}

/// The operand tag this repair works under. The solid is its own and only
/// operand: what is being resolved is a body crossing ITSELF, so there is no
/// second operand to be the other side of.
const SELF_OPERAND: u8 = 0;

/// How many pieces the crossing may cut the two faces into before this repair
/// declines. Every way of dropping some of them is REBUILT and re-checked, so
/// the enumeration has to stay small enough to run in full: at eight pieces
/// there are at most 225 candidate drops, nearly all of which the re-trim
/// closure ends before anything is assembled. The variable-radius fillet's
/// crossing cuts four pieces and the two-bore fixture's cuts six.
const MAXIMUM_CROSSING_PIECES: usize = 8;

/// Split two faces of ONE body that cross each other along their carriers'
/// exact intersection, drop the material the crossing encloses, and re-sew.
///
/// This is the bounded half of the self-intersection answer. Where two walls of
/// one body overshoot past each other instead of meeting at an edge, the
/// crossing curve IS the edge they should have met at: imprinting it onto both
/// faces cuts each into the part that belongs and the tail that overshot, and
/// dropping both tails leaves a body sewn along that curve.
///
/// **What it refuses, and why each refusal is a bound rather than a shrug:**
///
/// * A FOLD inside one face. There is no second face to imprint against, and
///   the answer is a trim that was never constructed rather than material to
///   drop — `offset/carve.rs` owns it.
/// * More than one crossing pair. Each repair changes the faces the next one
///   would be measured against, so resolving them together is a different
///   problem from resolving one.
/// * A crossing whose carriers' intersection does not cut BOTH trims. Then the
///   mesh crossing is not a curve the faces share and there is nothing to split
///   along.
/// * More pieces than [`MAXIMUM_CROSSING_PIECES`], because every candidate drop
///   is rebuilt in full and the enumeration has to run to the end rather than
///   be sampled.
/// * Any split whose reassembly does not come back SOUND. Every way of dropping
///   some pieces of each face — keeping at least one of each — is enumerated,
///   closed under the re-trim, rebuilt and re-checked in full (`validate()`
///   clean, connected, and scanning clean), so the piece choice is decided by
///   the result rather than guessed from a normal. If none verifies, the solid
///   is returned untouched and the caller refuses: a repair that cannot prove
///   its own answer must not ship one.
/// * A split that MORE THAN ONE drop resolves. Two sound answers is not an
///   answer, and choosing between them would be the guess this repair exists
///   not to make.
///
/// **The re-trim is the other half of the split.** Dropping a tail also removes
/// that tail's boundary uses on the NEIGHBOURING faces it bordered, and a body
/// cannot close over an edge only one face answers for. So the drop is closed
/// under "a face left holding an unmatched boundary goes too" — which, since
/// the imprint already cut those neighbours where the crossing curve met their
/// edges, drops the part of each that the crossing cut off. The closure is
/// bounded by the faces the imprint TOUCHED: reaching any other face ends that
/// candidate. Nothing here edits a loop by hand; every piece it chooses between
/// came out of the imprint/fragment machinery.
///
/// **And the drop RETIRES what the old configuration left.** The touched bound
/// is about material the crossing encloses, and it does not reach the third
/// thing a moved carrier leaves on a body: a piece, or a whole loop, that the
/// NEW configuration has no room for — a bore's exit ring still recorded on the
/// top face after the exit has migrated onto the side. Neither is material this
/// drop ran past; both are held by nothing the drop keeps, and keeping either
/// cannot close. [`fragment_is_detached`] retires the one the fragmenter
/// returns as its own island and [`holds_only_stale_loops`] the one it returns
/// as a hole loop that has left its face, and the ledger says a repair retired
/// something rather than leaving it to be read off the cell counts.
///
/// **A census has two directions and only one was enforced.** An edge used ONCE
/// is a face left holding an unanswered boundary, above. An edge used THREE
/// times is three faces along one curve, which is not a body either, and the
/// assembly does not refuse it — it puts the third face on a shell of its own
/// and every floor in this repository then passes the result.
/// [`over_used_edge`] refuses that candidate where the third use still exists,
/// which is here. It is what stopped the turned-bore migration returning a body
/// 23% out on area.
pub fn resolve_face_crossing(solid: &BrepSolid) -> Result<BrepSolid, KernelRefusal> {
    let options = SelfIntersectionOptions::for_solid(solid);
    let report = solid_self_intersections(solid, options).map_err(|message| {
        KernelRefusal::internal(
            KernelStage::Validate,
            "soundness_scan",
            format!("the soundness scan could not run: {message}"),
        )
    })?;
    if let Some(fold) = report.folds.first() {
        return Err(KernelRefusal::unsound(
            KernelStage::Validate,
            SoundnessDefect::Fold,
            report.folds.iter().map(|fold| fold.face).collect(),
            format!(
                "face {} folds through ITSELF; a fold is a trim that was never constructed and \
                 belongs to the carrier carve, not to this repair",
                fold.face
            ),
        ));
    }
    if report.confirmed.is_empty() {
        return Ok(solid.clone());
    }
    if report.confirmed.len() > 1 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Validate,
            "crossing.multiple_pairs",
            format!(
                "{} crossing face pairs; each repair changes the faces the next would be measured \
                 against, so this repair takes one pair at a time",
                report.confirmed.len()
            ),
        ));
    }
    let crossing = &report.confirmed[0];
    split_and_drop(solid, crossing.face_a, crossing.face_b, RepairOrigin::Scan(&report))
}

/// One crossing pair, split and resolved. Separated from the scan so the
/// refusals above read as one list.
fn split_and_drop(
    solid: &BrepSolid,
    face_a: u64,
    face_b: u64,
    origin: RepairOrigin<'_>,
) -> Result<BrepSolid, KernelRefusal> {
    use crate::fragment::fragment_solid;
    use crate::imprint::{FaceKey, ImprintOptions};
    use crate::{apply_edge_splits, build_imprints};

    let scale = crate::solid_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    if trace_on() {
        eprintln!("HEAL split_and_drop: faces {face_a} | {face_b}, scale {scale:.6}");
        trace_solid("entry solid", solid);
    }

    // Each face as its own one-face open body, which is what the imprint
    // driver pairs: it walks operand A's faces against operand B's, and a
    // solid paired with itself would pair every face with itself first.
    let open_a = one_face_body(solid, face_a)?;
    let open_b = one_face_body(solid, face_b)?;
    let mut request = ImprintOptions {
        tolerance,
        // The GLOBAL interpolation lane: the section the repair cuts along is
        // the faithful one. On the only row this repair takes
        // (`FilletFailureWith2RasiusValues`) the global lane's section lies
        // 1.6e-6 from its two carriers where the local lane's lies 7.8e-4, and
        // it moves every face the section bounds onto its closed form (`Pin_S`
        // 1.6e-4 -> 4.4e-7). The local lane was asked for until
        // `process_curve` split an edge a section ENDS on whatever the angle:
        // that section is tangent to the bore's mouth ring at both ends, the
        // global fit reproduces the tangency, and the old gates left the ring
        // whole, so the repair succeeded only on the local fit's 1.4-degree
        // end-tangent error. `BREP_HEAL_FIT_LANE` re-measures it.
        local_fit: false,
        // The station budget is the imprint's own (the ladder climbs it to
        // twice that). `96` was `offset_shell`'s: every section this repair
        // reaches is a sparse march (68 points on the fillet row, 7 and 8 on
        // `BadBoolean`) that saturates either budget, bit-identically.
        ..ImprintOptions::default()
    };
    measured_fit_request(&mut request);
    let mut imprint = build_imprints(&open_a, &open_b, &request).map_err(|error| {
        error.with_message(|error| format!("the two carriers could not be imprinted: {error}"))
    })?;

    // Both faces belong to ONE body, so both sides of the imprint are rewritten
    // onto that body's operand. The face ids are the solid's own and cannot
    // collide, so this is a relabel and not a merge.
    for face in &mut imprint.by_face {
        face.operand = SELF_OPERAND;
    }
    for split in &mut imprint.edge_splits {
        split.operand = SELF_OPERAND;
    }
    for piece in &mut imprint.pieces {
        for support in &mut piece.support_faces {
            *support = FaceKey {
                operand: SELF_OPERAND,
                face_id: support.face_id,
            };
        }
        for pcurve in &mut piece.pcurves {
            pcurve.operand = SELF_OPERAND;
        }
    }
    for barrier in &mut imprint.barrier_edges {
        barrier.0 = SELF_OPERAND;
    }

    if trace_on() {
        trace_imprint(&imprint);
    }

    let cut_a = imprint
        .by_face
        .iter()
        .any(|face| face.face_id == face_a && !face.piece_ids.is_empty());
    let cut_b = imprint
        .by_face
        .iter()
        .any(|face| face.face_id == face_b && !face.piece_ids.is_empty());
    if !cut_a || !cut_b {
        return Err(KernelRefusal::unsupported(
            KernelStage::Fragment,
            "crossing.no_shared_curve",
            format!(
                "the carriers' intersection reaches {} of the two trims, so there is no shared \
                 curve to split along",
                match (cut_a, cut_b) {
                    (true, false) => format!("only face {face_a}"),
                    (false, true) => format!("only face {face_b}"),
                    _ => "neither".into(),
                }
            ),
        ));
    }

    let split = apply_edge_splits(solid, SELF_OPERAND, &imprint).map_err(|error| {
        error.with_message(|error| format!("the imprinted edge splits could not be applied: {error}"))
    })?;
    let fragments = fragment_solid(&split, SELF_OPERAND, &imprint).map_err(|error| {
        error.with_message(|error| format!("the split faces could not be fragmented: {error}"))
    })?;

    if trace_on() {
        trace_solid("split solid", &split);
        trace_fragments(&fragments);
    }

    // The faces the imprint TOUCHED: the two that cross, plus every face whose
    // own loop carries an edge the imprint split. Those are exactly the faces
    // the dropped material can border — the crossing curve ends ON a boundary
    // edge of each crossing face, and that edge is shared with a neighbour — so
    // they are the only faces this repair may re-trim. Anything beyond them is
    // material this repair has not been shown to own, and reaching one ends the
    // candidate rather than widening the drop.
    let touched = touched_faces(solid, &imprint, face_a, face_b);
    // A POLE edge is legitimately used once — it is a collapsed boundary, not a
    // shared one — so it never means the face beside it bordered dropped
    // material. Left in the census it would drop the fragment holding it, and
    // the cascade would refuse a body the repair could have resolved.
    let degenerate: rustc_hash::FxHashSet<EdgeKey> = split
        .edges
        .iter()
        .filter(|edge| edge.degenerate)
        .map(|edge| EdgeKey::Boundary(SELF_OPERAND, edge.id))
        .collect();

    let pieces_of = |face: u64| -> Vec<usize> {
        fragments
            .iter()
            .enumerate()
            .filter_map(|(index, fragment)| (fragment.source_face_id == face).then_some(index))
            .collect()
    };
    let pieces_a = pieces_of(face_a);
    let pieces_b = pieces_of(face_b);
    // How many pieces the section has to cut, and it is not the same question
    // on both roads.
    //
    // From the SCAN, two faces overshoot PAST each other and the crossing curve
    // is the edge they should have met at, so it runs across both trims and
    // each has a tail: both sides must come out in two or more pieces, and one
    // piece is a section that grazed a trim rather than cut it.
    //
    // From the containment floor, the shape is different and the difference is
    // the whole of a COMPLETED MIGRATION. When a bore's mouth has walked
    // entirely onto the next face, the moved carrier meets that face in a
    // CLOSED loop inside its trim — the new mouth — which cuts nothing off it;
    // the face comes back as one piece carrying a new hole, and the only tail
    // is the wall's. Demanding two pieces of both would refuse exactly the case
    // the operator asked for, so this road asks for two pieces on ONE side and
    // at least one on the other. Nothing about the enumeration changes: a face
    // with one piece has one drop set, the empty one.
    let (need_a, need_b) = match &origin {
        RepairOrigin::Scan(_) => (2, 2),
        RepairOrigin::Breakout { .. } => {
            if pieces_a.len().max(pieces_b.len()) >= 2 {
                (1, 1)
            } else {
                (2, 2)
            }
        }
    };
    if pieces_a.len() < need_a || pieces_b.len() < need_b {
        return Err(KernelRefusal::unsupported(
            KernelStage::Fragment,
            "crossing.one_sided_cut",
            format!(
                "the crossing cut face {face_a} into {} piece(s) and face {face_b} into {} — this \
                 repair resolves the case where the curve cuts BOTH trims, so one part of each is \
                 the tail that overshot",
                pieces_a.len(),
                pieces_b.len()
            ),
        ));
    }
    if pieces_a.len() + pieces_b.len() > MAXIMUM_CROSSING_PIECES {
        return Err(KernelRefusal::non_convergence(
            KernelStage::Fragment,
            "crossing.pieces",
            format!(
                "the crossing cut face {face_a} into {} piece(s) and face {face_b} into {}, which is \
                 {} pieces against this repair's budget of {MAXIMUM_CROSSING_PIECES} — every way to \
                 drop some of them is rebuilt and re-checked, so the enumeration is bounded rather \
                 than sampled",
                pieces_a.len(),
                pieces_b.len(),
                pieces_a.len() + pieces_b.len(),
            ),
        ));
    }

    // WHICH pieces are the tails is decided by the RESULT, not by a normal.
    // Every way to drop some pieces of each crossing face (keeping at least one
    // of each) is closed under the re-trim below, rebuilt in full and
    // re-checked; the repair is the answer only when exactly ONE of them
    // verifies.
    //
    // The enumeration runs under TWO closures, and the second only where the
    // first found nothing at all. [`DropClosure::Drop`] drops a neighbour the
    // drop leaves holding an unmatched boundary — the rule this repair has
    // always used, and the one every body it already resolves is decided by.
    // [`DropClosure::Retrim`] RE-TRIMS that neighbour instead. Ordering them
    // this way makes the second closure strictly additive: a body the first
    // resolves never reaches it, so no answer this repair already returns can
    // change. The price is the enumeration twice over on a body that refuses.
    let plan = DropPlan {
        split: &split,
        fragments: &fragments,
        imprint: &imprint,
        touched: &touched,
        degenerate: &degenerate,
        pieces_a: &pieces_a,
        pieces_b: &pieces_b,
        face_a,
        face_b,
        scale,
    };
    let (mut verified, refusals) = plan.enumerate(DropClosure::Drop);
    if verified.is_empty() {
        verified = plan.enumerate(DropClosure::Retrim).0;
    }

    if verified.len() > 1 {
        return Err(KernelRefusal::ill_posed(
            KernelStage::Select,
            "crossing.ambiguous_drop",
            format!(
                "{} different drops of the crossing pieces all rebuilt into a sound body, so the \
                 result does not decide which material the crossing encloses; this repair answers \
                 only where one does",
                verified.len()
            ),
        ));
    }
    let Some(Candidate {
        dropped,
        retired,
        retrimmed,
        solid: repaired,
    }) = verified.pop()
    else {
        // The first few reasons, not all of them: eight pieces is up to 225
        // candidates and a refusal nobody can read is a refusal nobody uses.
        let listed = refusals
            .iter()
            .take(4)
            .cloned()
            .collect::<Vec<_>>()
            .join(" | ");
        // The candidate reasons (an over-used edge, a closure that consumed
        // every face, a rebuild that declined) are CONTEXT inside this one
        // refusal, whose class is minted here: the body still crosses itself.
        return Err(KernelRefusal::unsound(
            KernelStage::Sew,
            SoundnessDefect::Crossing,
            vec![face_a, face_b],
            format!(
                "none of the {} ways of dropping the crossing pieces rebuilt into a sound body \
                 ({listed}{}); the worst crossing was {:.3e} past its band, so the overshoot is \
                 real and it is the SPLIT that did not resolve it",
                refusals.len(),
                if refusals.len() > 4 {
                    format!(" | and {} more", refusals.len() - 4)
                } else {
                    String::new()
                },
                match &origin {
                    RepairOrigin::Scan(report) => report
                        .confirmed
                        .first()
                        .map(|crossing| crossing.straddle)
                        .unwrap_or_default(),
                    RepairOrigin::Breakout { .. } => 0.0,
                }
            ),
        ));
    };

    record_repair(
        &fragments,
        &dropped,
        &retired,
        &retrimmed,
        face_a,
        face_b,
        &origin,
    );
    Ok(repaired)
}

/// One candidate answer: the fragments the drop removed, the ones it RETIRED
/// as detached, the faces it RE-TRIMMED rather than removed, and the body
/// those choices rebuilt into.
struct Candidate {
    dropped: Vec<usize>,
    retired: Vec<u64>,
    retrimmed: Vec<u64>,
    solid: BrepSolid,
}

/// How a candidate drop closes over a neighbour it leaves holding a boundary
/// no other face answers for.
#[derive(Clone, Copy, Eq, PartialEq)]
enum DropClosure {
    /// DROP that neighbour too, iterated to a fixed point — [`close_drop_set`].
    Drop,
    /// RE-TRIM it — [`retrim_drop_set`]: take the unmatched coedges off it and
    /// re-link what remains into closed loops on the same carrier, with the
    /// same pcurves. The bound is the same one (a face the imprint never cut
    /// still ends the candidate); what changes is that a face it DID cut is
    /// re-trimmed rather than consumed.
    Retrim,
}

/// One crossing pair's split and everything a candidate drop is measured
/// against. Bundled so the enumeration can run once per closure.
struct DropPlan<'a> {
    split: &'a BrepSolid,
    fragments: &'a [crate::fragment::FaceFragmentRecord],
    imprint: &'a crate::imprint::ImprintResultRecord,
    touched: &'a rustc_hash::FxHashSet<u64>,
    degenerate: &'a rustc_hash::FxHashSet<EdgeKey>,
    pieces_a: &'a [usize],
    pieces_b: &'a [usize],
    face_a: u64,
    face_b: u64,
    scale: f64,
}

impl DropPlan<'_> {
    /// Every way of dropping some pieces of each crossing face, closed under
    /// `closure`, rebuilt in full and re-checked. The candidates that verified
    /// come back first, the reasons the rest did not second.
    fn enumerate(&self, closure: DropClosure) -> (Vec<Candidate>, Vec<String>) {
        let (face_a, face_b) = (self.face_a, self.face_b);
        let mut refusals: Vec<String> = Vec::new();
        let mut verified: Vec<Candidate> = Vec::new();
        let mut seen: Vec<(Vec<usize>, Vec<Vec<usize>>)> = Vec::new();
        for drop_a in proper_subsets(self.pieces_a) {
            for drop_b in proper_subsets(self.pieces_b) {
                let mut dropped: Vec<usize> =
                    drop_a.iter().chain(drop_b.iter()).copied().collect();
                if dropped.is_empty() {
                    continue;
                }
                let label = format!(
                    "dropping piece(s) {:?} of face {face_a} and {:?} of face {face_b}",
                    drop_a
                        .iter()
                        .map(|index| self
                            .pieces_a
                            .iter()
                            .position(|piece| piece == index)
                            .unwrap_or(0))
                        .collect::<Vec<_>>(),
                    drop_b
                        .iter()
                        .map(|index| self
                            .pieces_b
                            .iter()
                            .position(|piece| piece == index)
                            .unwrap_or(0))
                        .collect::<Vec<_>>(),
                );
                // The re-trim: a face that bordered dropped material is left
                // with a boundary use no other face answers, and a body cannot
                // close over one. Closing the drop under that rule is what
                // turns a two-face split into a re-trim of every face the
                // dropped material bordered.
                // The dropping closure leaves every fragment as it was, so it
                // borrows the list rather than copying it per candidate.
                let mut retired: Vec<u64> = Vec::new();
                let closed = match closure {
                    DropClosure::Drop => close_drop_set(
                        self.fragments,
                        &mut dropped,
                        &mut retired,
                        self.touched,
                        self.degenerate,
                    )
                    .map(|()| (None, Vec::new())),
                    DropClosure::Retrim => retrim_drop_set(
                        self.fragments,
                        &mut dropped,
                        &mut retired,
                        self.touched,
                        self.degenerate,
                    )
                    .map(|(working, retrimmed)| (Some(working), retrimmed)),
                };
                let (working, retrimmed) = match closed {
                    Ok(closed) => closed,
                    Err(reason) => {
                        if trace_on() {
                            println!("    candidate died at closure {label}: {reason}");
                        }
                        refusals.push(format!("{label}: {reason}"));
                        continue;
                    }
                };
                let working: &[crate::fragment::FaceFragmentRecord] =
                    working.as_deref().unwrap_or(self.fragments);
                dropped.sort_unstable();
                // Two seeds can close onto the same answer. The signature is
                // the drop AND what the re-trim left of each kept fragment, so
                // a second closure that re-trimmed differently is a different
                // candidate rather than a hidden duplicate.
                let signature = (
                    dropped.clone(),
                    working
                        .iter()
                        .enumerate()
                        .filter(|(index, _)| !dropped.contains(index))
                        .map(|(_, fragment)| {
                            fragment
                                .loops
                                .iter()
                                .map(|fragment_loop| fragment_loop.coedges.len())
                                .collect()
                        })
                        .collect(),
                );
                if seen.contains(&signature) {
                    continue;
                }
                seen.push(signature);
                if dropped.len() == working.len() {
                    refusals.push(format!("{label}: the re-trim consumed every face"));
                    continue;
                }
                // The census's OTHER direction — see [`over_used_edge`].
                if let Some((key, count)) = over_used_edge(working, &dropped, self.degenerate) {
                    refusals.push(format!(
                        "{label}: {} is held {count} times by the pieces this drop keeps, and a \
                         closed body holds every edge exactly twice",
                        match key {
                            EdgeKey::Boundary(_, edge) => format!("edge {edge}"),
                            EdgeKey::Imprint(piece) => format!("imprint piece {piece}"),
                        }
                    ));
                    if trace_on() {
                        println!(
                            "    candidate refused   {label}: {key:?} held {count} times"
                        );
                    }
                    continue;
                }
                let selected: Vec<_> = working
                    .iter()
                    .enumerate()
                    .filter(|(index, _)| !dropped.contains(index))
                    .map(|(_, fragment)| fragment.clone())
                    .collect();
                match rebuild(self.split, selected, self.imprint, self.scale) {
                    // `BREP_HEAL_TRACE=1` prints every candidate the
                    // enumeration considered, whether it verified, and what
                    // volume it produced. This is the instrument that found the
                    // 2026-09-22 stale-fragment defect: the `touched` bound
                    // kills most candidates before `verified.len() == 1` picks
                    // a survivor, and without the list there is no way to see
                    // that the survivor was not the right one.
                    Ok(solid) => {
                        if trace_on() {
                            println!(
                                "    CANDIDATE VERIFIED  {label}  volume {:.6}  dropped {:?}",
                                crate::solid_mass_properties(&solid)
                                    .map(|mass| mass.volume)
                                    .unwrap_or(f64::NAN),
                                dropped
                            );
                        }
                        verified.push(Candidate {
                            dropped,
                            retired,
                            retrimmed,
                            solid,
                        })
                    }
                    Err(reason) => {
                        if trace_on() {
                            println!("    candidate refused   {label}: {reason}");
                        }
                        refusals.push(format!("{label}: {reason}"))
                    }
                }
            }
        }
        (verified, refusals)
    }
}

/// The faces this repair may re-trim: the two that cross, and every face whose
/// own loops use an edge the imprint split.
fn touched_faces(
    solid: &BrepSolid,
    imprint: &crate::imprint::ImprintResultRecord,
    face_a: u64,
    face_b: u64,
) -> rustc_hash::FxHashSet<u64> {
    use rustc_hash::FxHashSet as HashSet;
    let split_edges: HashSet<u64> = imprint
        .edge_splits
        .iter()
        .filter(|split| split.operand == SELF_OPERAND)
        .map(|split| split.edge_id)
        .collect();
    let mut touched: HashSet<u64> = [face_a, face_b].into_iter().collect();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let uses_split = face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .any(|coedge| split_edges.contains(&coedge.edge_id));
        if uses_split {
            touched.insert(face.id);
        }
    }
    touched
}

/// Close a drop set under the rule a closed body cannot break: every edge is
/// used exactly twice. Dropping a tail takes its uses with it, so any face left
/// holding a boundary no other face answers bordered the dropped material and
/// has to be re-trimmed — which, since the imprint already cut it, means
/// dropping the part of it that the crossing cut off. Iterated to a fixed
/// point, and bounded by the touched set: reaching a face the imprint never cut
/// means the drop has left the material the crossing encloses, and the
/// candidate is refused rather than grown.
///
/// The one thing that bound does not own is a fragment joined to NOTHING the
/// drop keeps — see [`fragment_is_detached`]. That is not material the drop ran
/// past; it is what the old configuration left, and it is retired wherever it
/// came from.
fn close_drop_set(
    fragments: &[crate::fragment::FaceFragmentRecord],
    dropped: &mut Vec<usize>,
    retired: &mut Vec<u64>,
    touched: &rustc_hash::FxHashSet<u64>,
    degenerate: &rustc_hash::FxHashSet<EdgeKey>,
) -> Result<(), KernelRefusal> {
    use rustc_hash::FxHashMap as HashMap;
    loop {
        let mut uses: HashMap<EdgeKey, usize> = HashMap::default();
        for (index, fragment) in fragments.iter().enumerate() {
            if dropped.contains(&index) {
                continue;
            }
            for key in fragment
                .loops
                .iter()
                .flat_map(|fragment_loop| &fragment_loop.coedges)
                .filter_map(|coedge| edge_key(&coedge.source))
            {
                *uses.entry(key).or_default() += 1;
            }
        }
        let mut grew = false;
        for (index, fragment) in fragments.iter().enumerate() {
            if dropped.contains(&index) {
                continue;
            }
            // RETIRED, not re-trimmed: a fragment holding no edge that any
            // other kept fragment holds is attached to nothing this drop keeps.
            // The [`touched`] bound does not apply to it, because that bound
            // asks whether the drop has run past the material the crossing
            // encloses — and a piece joined to the body by nothing is not
            // material this drop reached, it is material the OLD configuration
            // left behind. Retiring it cannot cascade: the only uses it removes
            // are uses nothing else holds. See [`fragment_is_detached`].
            if fragment_is_detached(fragment, &uses, degenerate) {
                dropped.push(index);
                retired.push(fragment.source_face_id);
                grew = true;
                continue;
            }
            let unmatched = fragment
                .loops
                .iter()
                .flat_map(|fragment_loop| &fragment_loop.coedges)
                .filter_map(|coedge| edge_key(&coedge.source))
                .filter(|key| !degenerate.contains(key))
                .any(|key| uses.get(&key) == Some(&1));
            if !unmatched {
                continue;
            }
            if !touched.contains(&fragment.source_face_id) {
                return Err(KernelRefusal::internal(KernelStage::Validate, "retrim.uncut_face", format!(
                    "the re-trim reached face {}, which the imprint never cut — the drop has run \
                     past the material the crossing encloses",
                    fragment.source_face_id
                )));
            }
            dropped.push(index);
            grew = true;
        }
        if !grew {
            return Ok(());
        }
    }
}

/// [`close_drop_set`]'s other closure: a face left holding a boundary no other
/// face answers for is RE-TRIMMED rather than dropped.
///
/// Dropping it is the right answer when the unanswered boundary is the whole
/// of what that face contributed to the dropped material — a tail beside a
/// tail. It is the WRONG answer when the face is one the body keeps and only
/// part of its boundary went: a bore carried out through a side wall leaves
/// the mouth flat holding the stretch of its own corner edge that now runs
/// inside the bore, and the arc of the mouth ring that now runs outside the
/// body, and neither is material to drop — they are two coedges to take OFF a
/// face that otherwise stays exactly as it is. Taking them off merges that
/// face's mouth loop into its outer one, which is precisely the topology a
/// breakout has and a drop-only closure cannot reach.
///
/// The bound is [`close_drop_set`]'s: a face the imprint never cut still ends
/// the candidate rather than widening the re-trim, with the two exceptions that
/// are not re-trims at all — a fragment attached to nothing
/// ([`fragment_is_detached`]) and a whole loop that has left its face
/// ([`holds_only_stale_loops`]), both of which are retired. What comes back
/// is the fragment list to rebuild from — the untouched fragments as they were,
/// the re-trimmed ones with their loops re-linked — and the ids of the faces
/// that were re-trimmed, for the ledger. A fragment left with no coedge at all
/// joins `dropped`.
fn retrim_drop_set(
    fragments: &[crate::fragment::FaceFragmentRecord],
    dropped: &mut Vec<usize>,
    retired: &mut Vec<u64>,
    touched: &rustc_hash::FxHashSet<u64>,
    degenerate: &rustc_hash::FxHashSet<EdgeKey>,
) -> Result<(Vec<crate::fragment::FaceFragmentRecord>, Vec<u64>), KernelRefusal> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

    let mut working = fragments.to_vec();
    let mut retrimmed: Vec<u64> = Vec::new();
    loop {
        let mut uses: HashMap<EdgeKey, usize> = HashMap::default();
        for (index, fragment) in working.iter().enumerate() {
            if dropped.contains(&index) {
                continue;
            }
            for key in fragment
                .loops
                .iter()
                .flat_map(|fragment_loop| &fragment_loop.coedges)
                .filter_map(|coedge| edge_key(&coedge.source))
            {
                *uses.entry(key).or_default() += 1;
            }
        }
        // RETIRED FIRST, and before the early return below, because a
        // detached fragment does not have to hold an UNMATCHED edge to be
        // detached — one whose loop walks its own seam twice holds that seam
        // twice itself, and the census reads two. See [`fragment_is_detached`].
        let mut grew = false;
        for index in 0..working.len() {
            if dropped.contains(&index) {
                continue;
            }
            if fragment_is_detached(&working[index], &uses, degenerate) {
                dropped.push(index);
                retired.push(working[index].source_face_id);
                grew = true;
            }
        }
        if grew {
            // The census is stale the moment anything is retired.
            continue;
        }
        // A POLE edge is legitimately used once, as in [`close_drop_set`].
        let unmatched: HashSet<EdgeKey> = uses
            .iter()
            .filter(|(key, count)| **count == 1 && !degenerate.contains(key))
            .map(|(key, _)| *key)
            .collect();
        if unmatched.is_empty() {
            return Ok((working, retrimmed));
        }
        for index in 0..working.len() {
            if dropped.contains(&index) {
                continue;
            }
            let holds = working[index]
                .loops
                .iter()
                .flat_map(|fragment_loop| &fragment_loop.coedges)
                .filter_map(|coedge| edge_key(&coedge.source))
                .any(|key| unmatched.contains(&key));
            if !holds {
                continue;
            }
            // A STALE LOOP is the other way the old configuration leaves
            // something behind, and the [`touched`] bound does not own it —
            // see [`holds_only_stale_loops`].
            let stale = holds_only_stale_loops(&working[index], &unmatched, degenerate);
            if !touched.contains(&working[index].source_face_id) && !stale {
                return Err(KernelRefusal::internal(KernelStage::Validate, "retrim.uncut_face", format!(
                    "the re-trim reached face {}, which the imprint never cut — the drop has run \
                     past the material the crossing encloses",
                    working[index].source_face_id
                )));
            }
            let face = working[index].source_face_id;
            match retrim_fragment(&working[index], &unmatched)? {
                Some(next) => {
                    working[index] = next;
                    if stale {
                        retired.push(face);
                    } else if !retrimmed.contains(&face) {
                        retrimmed.push(face);
                    }
                }
                None => dropped.push(index),
            }
            grew = true;
        }
        if !grew {
            return Ok((working, retrimmed));
        }
    }
}

/// Re-trim ONE fragment: take off every coedge in `remove` and re-link what is
/// left into closed loops.
///
/// The link is made in the face's OWN parameter domain, which is where a trim
/// loop is closed and the only frame that tells the two sides of a seam apart —
/// a cylinder's seam is one curve in 3D and two coedges a period apart here.
/// Nothing is fitted or re-parameterised: each surviving coedge keeps its
/// source, its sense and its pcurve, and only the ORDER and the grouping into
/// loops are rebuilt. A chain that does not close is a re-trim this repair
/// cannot prove, and it refuses rather than emitting an open face.
///
/// The loops come back in the order of their first surviving coedge, so a face
/// whose outer loop has just absorbed an inner one is still outer-loop-first.
/// `Ok(None)` when nothing at all is left to keep.
fn retrim_fragment(
    fragment: &crate::fragment::FaceFragmentRecord,
    remove: &rustc_hash::FxHashSet<EdgeKey>,
) -> Result<Option<crate::fragment::FaceFragmentRecord>, KernelRefusal> {
    use crate::fragment::{FaceFragmentRecord, FragmentCoedge, FragmentLoop};

    let kept: Vec<FragmentCoedge> = fragment
        .loops
        .iter()
        .flat_map(|fragment_loop| &fragment_loop.coedges)
        .filter(|coedge| !edge_key(&coedge.source).is_some_and(|key| remove.contains(&key)))
        .cloned()
        .collect();
    if kept.is_empty() {
        return Ok(None);
    }
    let ends = kept
        .iter()
        .map(|coedge| {
            let [t0, t1] = coedge.pcurve.domain()?;
            Ok((coedge.pcurve.evaluate(t0)?, coedge.pcurve.evaluate(t1)?))
        })
        .collect::<Result<Vec<_>, String>>().or_refuse(KernelStage::Validate, "collect")?;
    // The domain's own diagonal sets the band a junction is matched in: a
    // plane's parameters are millimetres and a cylinder's are radians against
    // millimetres, so one fixed number would mean two different things on the
    // two carriers. The ends being matched were cut from ONE curve by the
    // imprint's own split, so they agree to the fit's precision and the band
    // only has to be small against the face.
    let [u0, u1] = fragment.surface.domain_u().or_refuse(KernelStage::Validate, "domain_u")?;
    let [v0, v1] = fragment.surface.domain_v().or_refuse(KernelStage::Validate, "domain_v")?;
    let tolerance = 1e-4 * (u1 - u0).hypot(v1 - v0);
    let mut used = vec![false; kept.len()];
    let mut loops: Vec<FragmentLoop> = Vec::new();
    for seed in 0..kept.len() {
        if used[seed] {
            continue;
        }
        used[seed] = true;
        let mut chain = vec![seed];
        let start = ends[seed].0;
        let mut cursor = ends[seed].1;
        while cursor.sub(start).length() > tolerance {
            let next = (0..kept.len())
                .filter(|index| !used[*index])
                .map(|index| (index, ends[index].0.sub(cursor).length()))
                .filter(|(_, gap)| *gap <= tolerance)
                .min_by(|left, right| left.1.total_cmp(&right.1))
                .map(|(index, _)| index);
            let Some(next) = next else {
                return Err(KernelRefusal::internal(KernelStage::Validate, "retrim.open_chain", format!(
                    "face {}'s re-trim left a chain open at ({:.6}, {:.6}) in its own parameter \
                     domain — what the drop took off it does not close into a loop",
                    fragment.source_face_id, cursor.x, cursor.y
                )));
            };
            used[next] = true;
            chain.push(next);
            cursor = ends[next].1;
        }
        loops.push(FragmentLoop {
            coedges: chain.into_iter().map(|index| kept[index].clone()).collect(),
        });
    }
    Ok(Some(FaceFragmentRecord {
        loops,
        ..fragment.clone()
    }))
}

/// Is this fragment attached to NOTHING else the drop keeps?
///
/// The census `uses` counts every non-dropped fragment's edge keys, this one's
/// included. So a fragment is detached exactly when, for every non-degenerate
/// key it holds, the census count equals ITS OWN count of that key: nobody else
/// is holding any edge of it.
///
/// It is stated that way rather than as "every key it holds is used once"
/// because a fragment may legitimately hold one key TWICE on its own — a
/// cylinder band walks its seam up one side and down the other — and such a
/// fragment is still detached when the seam's only two uses are both its.
///
/// **What a detached fragment IS.** It is a piece of the old configuration that
/// the new one has no room for: on the turned-bore migration, the top face's
/// hole loop is the exit ring the bore no longer opens through, sitting out at
/// x = 44.6 where the top face's own square ends at 20. The fragmenter returns
/// it as its own island because the ring is disjoint from the square, and the
/// only other face holding that ring is the wall's overshoot tail. Drop the
/// tail and the island is joined to nothing.
///
/// **Why retiring it is bounded.** Keeping it can never produce a closed body:
/// every edge it holds is answered for only by itself, so the assembly is open
/// there however the rest of the drop goes. And retiring it removes only uses
/// nothing else holds, so it cannot leave a third face unmatched — it does not
/// cascade, and no candidate the enumeration would otherwise verify is lost.
fn fragment_is_detached(
    fragment: &crate::fragment::FaceFragmentRecord,
    uses: &rustc_hash::FxHashMap<EdgeKey, usize>,
    degenerate: &rustc_hash::FxHashSet<EdgeKey>,
) -> bool {
    use rustc_hash::FxHashMap as HashMap;
    let mut own: HashMap<EdgeKey, usize> = HashMap::default();
    for key in fragment
        .loops
        .iter()
        .flat_map(|fragment_loop| &fragment_loop.coedges)
        .filter_map(|coedge| edge_key(&coedge.source))
        .filter(|key| !degenerate.contains(key))
    {
        *own.entry(key).or_default() += 1;
    }
    // A fragment carrying no identified edge at all is NOT called detached:
    // its boundary is all DERIVED, which `edge_key` deliberately does not
    // count, and the rebuild's own closure is what answers for those.
    !own.is_empty()
        && own
            .iter()
            .all(|(key, count)| uses.get(key).copied().unwrap_or(0) == *count)
}

/// The OTHER way the old configuration leaves something behind: a WHOLE LOOP
/// of a face that nothing else answers for AND THAT HAS LEFT THAT FACE.
///
/// [`fragment_is_detached`] catches the leftover when the fragmenter returns it
/// as its own island — which it does when the stale ring is far enough from the
/// face's own trim to be plainly disjoint. When it is nearer, the same ring
/// comes back as a HOLE LOOP of the face it left: the turned bore's exit ring
/// at 40 degrees sits at `x = 22.9 - 30.7` against a top face that ends at 20,
/// and the top face is ONE fragment carrying it. There is no island to retire,
/// and taking the loop off is a RE-TRIM of a face the imprint never cut.
///
/// That is the move the earlier records named as missing — *"the departed face
/// keeps a hole loop the imprint never cut, and the drop enumeration has no
/// move for retiring one"*. This is the move, and it is bounded FOUR ways:
///
/// * **Unanswered for.** Every non-degenerate edge of the loop is used once,
///   by this fragment alone, so no other face the drop keeps is standing on it
///   and removing it takes nothing from anyone. A loop only PART of which is
///   unanswered for is a genuine re-trim of a neighbour that bordered the
///   dropped material, and the [`touched`] bound still owns that case.
/// * **It has LEFT the face**, by the containment floor's own test: every
///   sample of it lies outside the face's outer loop in that face's own
///   parameter plane. This is the condition that matters, and it was added
///   after measuring what happens without it. Unanswered-for alone is
///   SYMMETRIC — at 40 degrees it licensed retiring the bore's ENTRY mouth
///   just as readily as its departed exit ring, so the enumeration verified
///   two bodies and refused the pair as ambiguous. A loop still inside its own
///   face is trimming material and is not a leftover, whoever is holding it.
/// * **A loop survives.** This never empties a face; a fragment with nothing
///   left at all is the detached case above.
/// * **A PLANE only**, as the containment floor restricts itself, because on a
///   periodic surface "outside the outer loop" in the parameter plane is a
///   statement about the seam rather than about the trim.
///
/// And the body still has to rebuild, `validate()`, connect, scan clean and
/// pass the containment floor afterwards, exactly as before.
fn holds_only_stale_loops(
    fragment: &crate::fragment::FaceFragmentRecord,
    unmatched: &rustc_hash::FxHashSet<EdgeKey>,
    degenerate: &rustc_hash::FxHashSet<EdgeKey>,
) -> bool {
    if !matches!(
        fragment.surface.analytic(),
        Some(crate::AnalyticSurface::Plane { .. })
    ) {
        return false;
    }
    let mut stale: Vec<usize> = Vec::new();
    let mut surviving = 0usize;
    for (index, fragment_loop) in fragment.loops.iter().enumerate() {
        let keys: Vec<EdgeKey> = fragment_loop
            .coedges
            .iter()
            .filter_map(|coedge| edge_key(&coedge.source))
            .filter(|key| !degenerate.contains(key))
            .collect();
        if !keys.is_empty() && keys.iter().all(|key| unmatched.contains(key)) {
            stale.push(index);
            continue;
        }
        if keys.iter().any(|key| unmatched.contains(key)) {
            return false;
        }
        surviving += 1;
    }
    if stale.is_empty() || surviving == 0 {
        return false;
    }
    let Ok(polylines) = fragment
        .loops
        .iter()
        .map(fragment_loop_polyline)
        .collect::<Result<Vec<_>, KernelRefusal>>()
    else {
        return false;
    };
    if polylines.iter().any(Vec::is_empty) {
        return false;
    }
    let outer_index = (0..polylines.len())
        .max_by(|&a, &b| {
            polyline_area(&polylines[a])
                .abs()
                .total_cmp(&polyline_area(&polylines[b]).abs())
        })
        .unwrap_or(0);
    // The face's own outer loop being the stale one is not a leftover to
    // retire; it is the whole face going, which is the detached case.
    if stale.contains(&outer_index) {
        return false;
    }
    let outer = &polylines[outer_index];
    let bands = BoundaryBands::build(outer);
    let tolerance = boundary_tolerance(outer);
    stale.iter().all(|index| {
        polylines[*index]
            .iter()
            .all(|&point| !bands.contains(point) && !bands.within(point, tolerance))
    })
}

/// The other half of the census rule, and the one that had no enforcement.
///
/// A closed body uses every non-degenerate edge EXACTLY TWICE. The drop
/// closures above are the "fewer than twice" half — a face left holding a
/// boundary nobody answers is dropped or re-trimmed. Nothing checked the other
/// direction, and the direction matters here: an imprint piece is cut into BOTH
/// crossing faces, so before any drop it is held four times, once by each of
/// the four pieces. A drop that takes one piece away leaves it held three
/// times — three faces along one curve, which is not a body.
///
/// That is the candidate the turned-bore migration used to return: dropping the
/// exit mouth's disc alone leaves the wall's kept part, the wall's overshoot
/// tail and the mouth flat's remainder all on the mouth ring. Refusing it is
/// what leaves the one drop that is right.
///
/// **And nothing downstream catches it**, which is why the check belongs here
/// and not there. `assemble_open_fragments` resolves the third use by putting
/// the overshoot on a shell of its OWN: the body comes back with every edge
/// used exactly twice, `validate()` clean, `solid_connectivity` calling it
/// connected (the sheets share edges with the main shell), the scan clean and
/// the containment floor acquitting it — a three-shell solid whose first shell
/// is the right answer and whose other two are sheets stapled to it. Measured
/// on the turned bore at 60 degrees: shell 1 is the 7 correct faces at
/// 7673.516114379 against a closed form of 7673.516114438, shell 2 is the
/// overshoot alone at -536.326313 and shell 3 the departed ring at -6.7e-14.
///
/// Returns the offending key and its count, smallest key first so the refusal
/// reads the same on every run.
fn over_used_edge(
    fragments: &[crate::fragment::FaceFragmentRecord],
    dropped: &[usize],
    degenerate: &rustc_hash::FxHashSet<EdgeKey>,
) -> Option<(EdgeKey, usize)> {
    use rustc_hash::FxHashMap as HashMap;
    let mut uses: HashMap<EdgeKey, usize> = HashMap::default();
    for (index, fragment) in fragments.iter().enumerate() {
        if dropped.contains(&index) {
            continue;
        }
        for key in fragment
            .loops
            .iter()
            .flat_map(|fragment_loop| &fragment_loop.coedges)
            .filter_map(|coedge| edge_key(&coedge.source))
            .filter(|key| !degenerate.contains(key))
        {
            *uses.entry(key).or_default() += 1;
        }
    }
    let mut over: Vec<(EdgeKey, usize)> = uses
        .into_iter()
        .filter(|(_, count)| *count > 2)
        .collect();
    over.sort_unstable();
    over.into_iter().next()
}

/// The identity of a fragment coedge's edge, for the use census. A DERIVED
/// coedge carries a freshly minted curve rather than an identity, so it is not
/// counted — the rebuild's own closure check is what answers for those.
fn edge_key(source: &crate::fragment::FragmentEdgeSource) -> Option<EdgeKey> {
    use crate::fragment::FragmentEdgeSource;
    match source {
        FragmentEdgeSource::Boundary { operand, edge_id }
        | FragmentEdgeSource::SharedBoundary { operand, edge_id } => {
            Some(EdgeKey::Boundary(*operand, *edge_id))
        }
        FragmentEdgeSource::Imprint { piece_id } => Some(EdgeKey::Imprint(*piece_id)),
        FragmentEdgeSource::Derived { .. } => None,
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
enum EdgeKey {
    Boundary(u8, u64),
    Imprint(u64),
}

/// Every subset of `pieces` that leaves at least one behind, the empty one
/// included — the ways one face's pieces can be dropped.
fn proper_subsets(pieces: &[usize]) -> Vec<Vec<usize>> {
    let mut subsets = Vec::new();
    for mask in 0..(1u32 << pieces.len()) {
        if mask == (1u32 << pieces.len()) - 1 {
            // Dropping every piece of a crossing face is not a re-trim of it.
            continue;
        }
        subsets.push(
            pieces
                .iter()
                .enumerate()
                .filter(|(bit, _)| mask & (1 << bit) != 0)
                .map(|(_, piece)| *piece)
                .collect(),
        );
    }
    subsets
}

/// Reassemble one choice of kept fragments and prove it sound, or say why not.
fn rebuild(
    split: &BrepSolid,
    selected: Vec<crate::fragment::FaceFragmentRecord>,
    imprint: &crate::imprint::ImprintResultRecord,
    scale: f64,
) -> Result<BrepSolid, KernelRefusal> {
    use crate::boolean::{assemble_open_fragments, finalize_assembled_solid};
    use rustc_hash::FxHashMap as HashMap;

    let mut sources: HashMap<u8, &BrepSolid> = HashMap::default();
    sources.insert(SELF_OPERAND, split);
    let assembly_tolerance = 2e-3f64.max(scale * 5e-5);
    let solid = assemble_open_fragments(selected, &sources, imprint, assembly_tolerance)
        .map_err(|error| error.with_message(|error| format!("the kept pieces did not assemble: {error}")))?;
    let solid = finalize_assembled_solid(solid, assembly_tolerance)
        .map_err(|error| error.with_message(|error| format!("the assembly did not finalize: {error}")))?;

    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "rebuild.validate", format!(
            "the rebuilt body fails validate(): {}",
            issues[0].message
        )));
    }
    if crate::solid_connectivity(&solid).is_disconnected() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "rebuild.connected", "the rebuilt body is not connected"));
    }
    let options = SelfIntersectionOptions::for_solid(&solid);
    let report = solid_self_intersections(&solid, options)
        .map_err(|message| format!("the rebuilt body could not be scanned: {message}"))
        .or_refuse(KernelStage::Validate, "solid_self_intersections")?;
    if report.is_flagged() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "rebuild.self_intersects", format!(
            "the rebuilt body still self-intersects: {}",
            report.summary().unwrap_or_else(|| "no summary".into())
        )));
    }
    Ok(solid)
}

/// One face of a body as its own open body, carrying the edges and vertices
/// that face uses and nothing else.
fn one_face_body(solid: &BrepSolid, face_id: u64) -> Result<BrepSolid, KernelRefusal> {
    use crate::topology::ShellRecord;
    use rustc_hash::FxHashSet as HashSet;

    let face = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| face.id == face_id)
        .ok_or_else(|| {
            // The class its one caller gave it before this helper was typed.
            KernelRefusal::internal(KernelStage::Fragment, "one_face_body", format!("face {face_id} is not in this body"))
        })?
        .clone();
    let used: HashSet<u64> = face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    let edges: Vec<_> = solid
        .edges
        .iter()
        .filter(|edge| used.contains(&edge.id))
        .cloned()
        .collect();
    let vertices: HashSet<u64> = edges
        .iter()
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    Ok(BrepSolid {
        mass_properties_cache: Default::default(),
        id: solid.id,
        genus: 0,
        vertices: solid
            .vertices
            .iter()
            .filter(|vertex| vertices.contains(&vertex.id))
            .cloned()
            .collect(),
        edges,
        shells: vec![ShellRecord {
            id: 1,
            faces: vec![face],
        }],
    })
}

// ---------------------------------------------------------------------------
// The ledger: a repaired result is reported as repaired
// ---------------------------------------------------------------------------

/// One self-crossing this module REPAIRED rather than refused.
///
/// A repair changes the answer a lane returns, so it is never silent: the
/// feature that ran it carries this in its own result
/// (`FeatureResult::notes`), and the case gate records it per case. The
/// ledger is the carrier between the two — [`accept_sound`] is called from
/// lanes with no diagnostics record to write into, and threading one through
/// every lane exit would be a wider change than the repair itself. Same shape
/// as the boolean's `CONFORMANCE_REPAIRS` counter and the pcurve fit ledger.
#[derive(Clone, Debug)]
pub struct CrossingRepairRecord {
    /// The two faces that crossed.
    pub faces: (u64, u64),
    /// The crossing point the scan confirmed.
    pub point: crate::Vec3,
    /// How far past its band the crossing straddled.
    pub straddle: f64,
    /// Source faces whose pieces were dropped, with how many pieces of each.
    pub dropped: Vec<(u64, usize)>,
    /// Source faces the repair RETIRED a leftover from, with how many: a
    /// piece the drop left joined to nothing, or a whole loop of a face that
    /// nothing else answers for. Either way it is what the OLD configuration
    /// left on the body, and the ledger is the only place that says a repair
    /// took one off — "a piece was dropped" and "a leftover was retired" are
    /// different events.
    pub retired: Vec<(u64, usize)>,
    /// The faces the drop RE-TRIMMED: neighbours of the dropped material,
    /// which are neither of the crossing pair.
    pub retrimmed: Vec<u64>,
    /// Set when the repair was reached from the HOLE-CONTAINMENT FLOOR rather
    /// than from the scan: the face whose hole loop had left it, and that
    /// loop. `point` and `straddle` are then unmeasured and read zero, because
    /// no crossing was confirmed — the floor convicts a containment, not a
    /// crossing.
    pub breakout: Option<(u64, u64)>,
}

impl std::fmt::Display for CrossingRepairRecord {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some((face, loop_id)) = self.breakout {
            return write!(
                formatter,
                "repaired a BREAKOUT: hole loop {loop_id} of face {face} had left that face \
                 onto face {}, so face {} — the wall it bounds — and face {} were split along \
                 their carriers' intersection; dropped {} and re-trimmed {}{}",
                self.faces.1,
                self.faces.0,
                self.faces.1,
                self.pieces_dropped(),
                self.faces_retrimmed(),
                self.pieces_retired(),
            );
        }
        write!(
            formatter,
            "repaired a self-crossing: faces {} and {} crossed near ({:.4}, {:.4}, {:.4}) \
             (straddle {:.3e}); split along their carriers' intersection, dropped {} \
             and re-trimmed {}{}",
            self.faces.0,
            self.faces.1,
            self.point.x,
            self.point.y,
            self.point.z,
            self.straddle,
            self.pieces_dropped(),
            self.faces_retrimmed(),
            self.pieces_retired(),
        )
    }
}

impl CrossingRepairRecord {
    fn pieces_dropped(&self) -> String {
        self.dropped
            .iter()
            .map(|(face, count)| format!("{count} piece(s) of face {face}"))
            .collect::<Vec<_>>()
            .join(", ")
    }

    fn faces_retrimmed(&self) -> String {
        if self.retrimmed.is_empty() {
            "no neighbouring face".to_string()
        } else {
            format!(
                "face(s) {}",
                self.retrimmed
                    .iter()
                    .map(|face| face.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        }
    }

    /// The clause that says the OLD configuration's leftovers were taken off
    /// the body. Empty when nothing was retired, so every message this ledger
    /// already printed is unchanged; a repair that retires something says so
    /// and names the face, which is the only way the case gate's result line
    /// tells a repaired migration from the one it used to return.
    fn pieces_retired(&self) -> String {
        if self.retired.is_empty() {
            return String::new();
        }
        format!(
            ", and RETIRED {} the old configuration left, held by nothing else this repair keeps",
            self.retired
                .iter()
                .map(|(face, count)| format!("{count} leftover(s) of face {face}"))
                .collect::<Vec<_>>()
                .join(", ")
        )
    }
}

thread_local! {
    static CROSSING_REPAIRS: std::cell::RefCell<Vec<CrossingRepairRecord>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// Take every repair made on this thread since the last call, leaving the
/// ledger empty. The history loop drains it after each feature so the repair
/// is reported against the feature that made it.
pub fn take_crossing_repairs() -> Vec<CrossingRepairRecord> {
    CROSSING_REPAIRS.with(|repairs| std::mem::take(&mut *repairs.borrow_mut()))
}

/// Put a ledger back, replacing whatever is there.
///
/// The migration route TRIES candidate faces, and a trial that builds records
/// itself on the way past. Only the candidate whose body is returned may keep
/// its record, and a caller's own entries must survive the trials, so the
/// ledger is taken before and written back after. Nothing else may use this:
/// a repair reports itself once, where it happened.
fn restore_repairs(records: Vec<CrossingRepairRecord>) {
    CROSSING_REPAIRS.with(|repairs| *repairs.borrow_mut() = records);
}

fn record_repair(
    fragments: &[crate::fragment::FaceFragmentRecord],
    dropped: &[usize],
    retired: &[u64],
    retrimmed_faces: &[u64],
    face_a: u64,
    face_b: u64,
    origin: &RepairOrigin<'_>,
) {
    let tally = |faces: &[u64]| {
        let mut by_face: Vec<(u64, usize)> = Vec::new();
        for face in faces {
            match by_face.iter_mut().find(|(existing, _)| existing == face) {
                Some((_, count)) => *count += 1,
                None => by_face.push((*face, 1)),
            }
        }
        by_face.sort_unstable();
        by_face
    };
    let by_face = tally(
        &dropped
            .iter()
            .map(|index| fragments[*index].source_face_id)
            .collect::<Vec<_>>(),
    );
    let retired = tally(retired);
    // A face that lost ONLY a detached piece was not re-trimmed — nothing of
    // it was re-cut, a leftover of the old configuration was taken off the
    // body — so it is reported under `retired` and not counted here twice.
    let mut retrimmed: Vec<u64> = by_face
        .iter()
        .filter(|(face, count)| {
            retired
                .iter()
                .find(|(retired_face, _)| retired_face == face)
                .map(|(_, retired_count)| retired_count < count)
                .unwrap_or(true)
        })
        .map(|(face, _)| *face)
        .chain(retrimmed_faces.iter().copied())
        .filter(|face| *face != face_a && *face != face_b)
        .collect();
    retrimmed.sort_unstable();
    retrimmed.dedup();
    let crossing = match origin {
        RepairOrigin::Scan(report) => report.confirmed.first(),
        RepairOrigin::Breakout { .. } => None,
    };
    let record = CrossingRepairRecord {
        faces: (face_a, face_b),
        point: crossing.map(|crossing| crossing.point).unwrap_or_default(),
        straddle: crossing.map(|crossing| crossing.straddle).unwrap_or_default(),
        dropped: by_face,
        retired,
        retrimmed,
        breakout: match origin {
            RepairOrigin::Scan(_) => None,
            RepairOrigin::Breakout { face, loop_id } => Some((*face, *loop_id)),
        },
    };
    if trace_on() {
        eprintln!("HEAL {record}");
    }
    CROSSING_REPAIRS.with(|repairs| repairs.borrow_mut().push(record));
}

/// Write the flagged body out before the repair touches it, when
/// `BREP_HEAL_DUMP` names a directory. A crossing is measured off the body that
/// carries it — per-face areas, a mesh-arbitrated genus, the volume the repair
/// changes — and replaying a whole document to reach it again costs more than
/// the file does. Diagnostics only: the lane still repairs or refuses exactly
/// as it would have.
fn dump_flagged(solid: &BrepSolid, entry: &str) {
    let Ok(directory) = std::env::var("BREP_HEAL_DUMP") else {
        return;
    };
    if directory.is_empty() {
        return;
    }
    let name: String = entry
        .chars()
        .map(|character| if character.is_alphanumeric() { character } else { '_' })
        .collect();
    let path = std::path::Path::new(&directory).join(format!("flagged-{name}.json"));
    match serde_json::to_string(solid) {
        Ok(text) => {
            if let Err(error) = std::fs::write(&path, text) {
                eprintln!("HEAL dump {}: {error}", path.display());
            } else {
                eprintln!("HEAL dumped the flagged body to {}", path.display());
            }
        }
        Err(error) => eprintln!("HEAL dump: {error}"),
    }
}

// ---------------------------------------------------------------------------
// Trace instrument (BREP_HEAL_TRACE=1)
// ---------------------------------------------------------------------------

fn trace_on() -> bool {
    std::env::var("BREP_HEAL_TRACE").is_ok_and(|value| !value.is_empty() && value != "0")
}

/// Measurement hatch for the section fit the repair's imprint asks for:
/// `BREP_HEAL_FIT_LANE=local|global` picks the lane its ladder starts in and
/// `BREP_HEAL_FIT_POINTS=<n>` its station budget. Both unset is the request as
/// built, so every lane and budget can be read off ONE binary against the
/// shipped one (`crossing_repair_probe body` with `BREP_DEBUG_POLYLINE_FIT=1`).
fn measured_fit_request(request: &mut crate::imprint::ImprintOptions) {
    match std::env::var("BREP_HEAL_FIT_LANE").as_deref() {
        Ok("local") => request.local_fit = true,
        Ok("global") => request.local_fit = false,
        _ => {}
    }
    if let Some(points) = std::env::var("BREP_HEAL_FIT_POINTS")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .filter(|points| *points >= 2)
    {
        request.maximum_fit_points = points;
    }
    if trace_on() {
        eprintln!(
            "HEAL imprint request: {} lane, {} stations, tolerance {:.3e}",
            if request.local_fit { "local" } else { "global" },
            request.maximum_fit_points,
            request.tolerance
        );
    }
}

fn trace_solid(label: &str, solid: &BrepSolid) {
    use rustc_hash::FxHashMap as HashMap;
    let point: HashMap<u64, crate::Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    eprintln!(
        "HEAL {label}: {}F {}E {}V shells={}",
        solid.shells.iter().map(|shell| shell.faces.len()).sum::<usize>(),
        solid.edges.len(),
        solid.vertices.len(),
        solid.shells.len()
    );
    let mut uses: HashMap<u64, usize> = HashMap::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    *uses.entry(coedge.edge_id).or_default() += 1;
                }
            }
        }
    }
    for edge in &solid.edges {
        let start = point.get(&edge.start_vertex_id).copied().unwrap_or_default();
        let end = point.get(&edge.end_vertex_id).copied().unwrap_or_default();
        eprintln!(
            "HEAL   edge {:>3} uses={} v{}->v{} ({:.4},{:.4},{:.4})->({:.4},{:.4},{:.4}) deg={} '{}'",
            edge.id,
            uses.get(&edge.id).copied().unwrap_or(0),
            edge.start_vertex_id,
            edge.end_vertex_id,
            start.x, start.y, start.z, end.x, end.y, end.z,
            edge.degenerate,
            edge.name.as_deref().unwrap_or("")
        );
    }
    for shell in &solid.shells {
        for face in &shell.faces {
            eprintln!(
                "HEAL   face {:>3} loops={} '{}'",
                face.id,
                face.loops.len(),
                face.name.as_deref().unwrap_or("")
            );
            for (index, loop_record) in face.loops.iter().enumerate() {
                let text = loop_record
                    .coedges
                    .iter()
                    .map(|coedge| format!("{}{}", if coedge.forward { '+' } else { '-' }, coedge.edge_id))
                    .collect::<Vec<_>>()
                    .join(" ");
                eprintln!("HEAL     loop {index}: {text}");
            }
        }
    }
}

fn trace_imprint(imprint: &crate::imprint::ImprintResultRecord) {
    eprintln!(
        "HEAL imprint: {} piece(s) {} edge_split(s) by_face={:?}",
        imprint.pieces.len(),
        imprint.edge_splits.len(),
        imprint
            .by_face
            .iter()
            .map(|face| (face.operand, face.face_id, face.piece_ids.clone()))
            .collect::<Vec<_>>()
    );
    for piece in &imprint.pieces {
        let start = piece.curve.evaluate(piece.t0).unwrap_or_default();
        let end = piece.curve.evaluate(piece.t1).unwrap_or_default();
        let middle = piece.curve.evaluate((piece.t0 + piece.t1) / 2.0).unwrap_or_default();
        eprintln!(
            "HEAL   piece {} v{}->v{} ({:.4},{:.4},{:.4}) .. ({:.4},{:.4},{:.4}) .. ({:.4},{:.4},{:.4}) supports={:?} shared={:?}",
            piece.id,
            piece.start_vertex_id,
            piece.end_vertex_id,
            start.x, start.y, start.z,
            middle.x, middle.y, middle.z,
            end.x, end.y, end.z,
            piece.support_faces.iter().map(|key| (key.operand, key.face_id)).collect::<Vec<_>>(),
            piece.shared_edge,
        );
    }
    for split in &imprint.edge_splits {
        eprintln!(
            "HEAL   edge_split operand={} edge={} at {:?}",
            split.operand, split.edge_id, split.parameters
        );
    }
    for vertex in &imprint.vertices {
        eprintln!(
            "HEAL   imprint vertex {} ({:.4},{:.4},{:.4})",
            vertex.id, vertex.point.x, vertex.point.y, vertex.point.z
        );
    }
}

fn trace_fragments(fragments: &[crate::fragment::FaceFragmentRecord]) {
    use crate::fragment::FragmentEdgeSource;
    eprintln!("HEAL fragments: {}", fragments.len());
    for (index, fragment) in fragments.iter().enumerate() {
        eprintln!(
            "HEAL   fragment {index} face {} loops={} test=({:.4},{:.4},{:.4})",
            fragment.source_face_id,
            fragment.loops.len(),
            fragment.test_point.x,
            fragment.test_point.y,
            fragment.test_point.z
        );
        for (loop_index, fragment_loop) in fragment.loops.iter().enumerate() {
            let text = fragment_loop
                .coedges
                .iter()
                .map(|coedge| {
                    let tag = match &coedge.source {
                        FragmentEdgeSource::Boundary { operand, edge_id } => format!("b{operand}:{edge_id}"),
                        FragmentEdgeSource::SharedBoundary { operand, edge_id } => format!("s{operand}:{edge_id}"),
                        FragmentEdgeSource::Imprint { piece_id } => format!("p{piece_id}"),
                        FragmentEdgeSource::Derived { start, end, .. } => format!(
                            "d({:.3},{:.3},{:.3})->({:.3},{:.3},{:.3})",
                            start.x, start.y, start.z, end.x, end.y, end.z
                        ),
                    };
                    format!("{}{tag}", if coedge.forward { '+' } else { '-' })
                })
                .collect::<Vec<_>>()
                .join(" ");
            eprintln!("HEAL     loop {loop_index}: {text}");
        }
    }
}


