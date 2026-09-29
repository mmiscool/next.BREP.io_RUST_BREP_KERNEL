//! The ONE definition of "intersect two OFFSET analytic carriers, in closed
//! form" — the *locus*-level offset seam.
//!
//! # Why this is not the pointwise evaluator, and not `intersect_analytic_pair`
//!
//! Slice 1 built [`crate::OffsetEvaluator`] — "where is the offset of THIS
//! surface at THIS `(u, v)`" — and deliberately refused to route the blend's
//! three corner solves through it, because those solves hold neither a surface
//! nor a parameter and what they need back is a whole LOCUS, not a point
//! (audit §9.4). This module is the shape that does fit.
//!
//! The audit's Slice 2b entry proposed
//! `offset_analytic_pair(a: &AnalyticSurface, da, b: &AnalyticSurface, db)`
//! that "hands the pair to the *existing* [`crate::intersect_analytic_pair`]".
//! Read rather than assumed, that plumbing does not exist to be reused here —
//! three separate misfits, all measured in §11 of the audit:
//!
//! 1. **`intersect_analytic_pair` refuses plane × plane by design**
//!    (`geometry/analytic_surface/intersect.rs`, the `(Plane, Plane) => None`
//!    arm and its comment). Plane × plane is the *headline* case — five of the
//!    seven solves collected here.
//! 2. **It takes `&NurbsSurface`, not `&AnalyticSurface`**, and the corner
//!    solves hold neither: they were handed bare normals, an axis point/dir
//!    and a radius by their caller. Routing through it would mean fabricating
//!    NURBS carriers to intersect them.
//! 3. **It returns `Vec<NurbsCurve>` clipped to the operands' finite domains.**
//!    Even the one lane that does answer this module's question —
//!    `cylinder_parallel_plane_lines` — returns two *segments* cut to the
//!    cylinder's `height` band and goes silently empty at `radius − tolerance`,
//!    where the corner solves need infinite lines and a loud refusal. Recovering
//!    a line's point and direction back out of a fitted `NurbsCurve` is strictly
//!    less exact than the 3×3 solve it replaced.
//!
//! So the seam lives here, at `Vec3` level, in the same closed forms the sites
//! already used — not as a router into the surface intersector.
//!
//! # The sign convention, and what it encodes
//!
//! **`distance` is signed ALONG THE CARRIER'S OUTWARD NORMAL**, the same
//! convention slice 1 fixed for [`crate::OffsetEvaluator`]: the offset carrier
//! is `{ p + distance·n̂(p) }`. For a plane that is the plane translated by
//! `distance·n`; for a cylinder wall whose outward normal points AWAY from its
//! axis it is radius `R + distance`, and `R − distance` for a bore wall whose
//! outward normal points at the axis ([`offset_cylinder_radius`]).
//!
//! That one convention makes the convex/concave split VISIBLE instead of
//! hidden in four hand-written sign choices: a **convex** blend's centre locus
//! is the `−r` offset (inside the material) and a **concave** blend's axis is
//! the `+r` offset (out in the void). `mixed_support`'s ball centre and
//! `mixed_curved`'s concave axis differ by exactly that sign and by nothing
//! else — which is why they used opposite-looking `R ∓ r` expressions before.
//!
//! # Root selection is the CALLER's, always
//!
//! A quadratic has two roots and each site's pick encodes its own geometry, so
//! nothing here picks one. [`line_meets_offset_cylinder`] and
//! [`axial_plane_meets_offset_cylinder`] return **both** roots in a fixed
//! ALGEBRAIC order (`(−b − √Δ)/2a` first), never a geometric one. Sorting them
//! nearest-the-corner first, de-duplicating a tangency, and proving a root by
//! the tangency vertices the sequential fillets left behind all stay at the
//! sites, unchanged.
//!
//! # Degeneracies are values, not strings
//!
//! Every refusal is an [`OffsetPairDegeneracy`] variant and every site renders
//! its own wording from it. The corner refusal messages are asserted on by
//! `corner_concave.rs` (`mixed_curved_wall_corner_refusals`,
//! `all_concave_staircase_corner_refuses`, `tilted_concave_vertex_corner_refuses`),
//! so a shared form that emitted its own text would have moved those tests —
//! the same discipline slice 1 applied to orientation and slice 2 to sampling
//! density.
//!
//! # Exactness
//!
//! Every routine here is the arithmetic its site already performed, in the same
//! order, so the switch is **bit-identical** — pinned by the `bit_identical_*`
//! tests below and measured across the corpus by `BREP_OFFSET_PAIR_DIAG`
//! (audit §11.4). Where two sites used *different* algebra for the same
//! geometry they keep two entry points rather than being merged behind a
//! general one: [`axial_plane_meets_offset_cylinder`] uses the `a = 1`
//! quadratic its plane-parallel-to-axis precondition guarantees, and
//! [`line_meets_offset_cylinder`] the general `a = |d̂⊥|²` form. Folding the
//! first into the second would move its roots in the last bits for no gain.

use crate::Vec3;

/// A straight locus: `point + t·direction`, `direction` unit.
///
/// `point` is the locus point in the plane through the caller's `anchor`
/// perpendicular to `direction` — i.e. the FOOT OF THE PERPENDICULAR from the
/// anchor, not the least-norm point from the world origin. That distinction is
/// load-bearing: `edit/direct_edit/geom.rs::intersect_planes` anchors the other
/// way (`α·n_a + β·n_b`, closest to the origin) and is deliberately NOT a
/// consumer of this module (audit §11.2).
#[derive(Clone, Copy, Debug)]
pub(crate) struct OffsetLine {
    pub(crate) point: Vec3,
    pub(crate) direction: Vec3,
}

/// Why an offset-carrier intersection has no unique locus.
///
/// One variant per degenerate configuration these closed forms can meet. Each
/// site maps the variant to its own message; nothing here formats text.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum OffsetPairDegeneracy {
    /// The two plane normals are parallel or antiparallel, so the two offset
    /// planes are parallel: they meet in nothing (distinct) or in a whole
    /// plane (coincident). Either way there is no line, and the two cases are
    /// NOT distinguished here — no site needs them apart, and telling them
    /// apart needs a tolerance this module refuses to own.
    ParallelPlanes,
    /// The linear system is singular to `solve_small`'s `1e-13·scale` pivot
    /// floor. For a plane PAIR this is all but unreachable — the determinant is
    /// `|n_a × n_b|`, which `ParallelPlanes` already caught above `1e-12` — but
    /// a plane TRIPLE is genuinely singular whenever the three normals are
    /// coplanar (a prismatic "corner" that is really an edge).
    SingularSystem,
    /// The offset radius `R ± d` is zero or negative: the offset swallows the
    /// cylinder's axis, so there is no offset cylinder to intersect.
    CollapsedCylinder,
    /// The line runs (near-)parallel to the cylinder axis, so its projection
    /// into the cross-section is a point: it meets the offset cylinder in
    /// nothing or in the whole line, and the quadratic has no leading term.
    LineParallelToAxis,
    /// The plane's normal is (near-)parallel to the cylinder axis, so the plane
    /// is perpendicular to the axis rather than containing its direction: the
    /// section is a CIRCLE, not a line pair, and this routine does not answer
    /// it. Refused loudly rather than approximated — no site in the tree asks
    /// for that section (audit §11.3).
    PlaneNormalAlongAxis,
    /// The discriminant is negative: the offset plane/line misses the offset
    /// cylinder entirely. A vanishing discriminant is NOT this — tangency
    /// returns the double root twice, and the caller decides whether two equal
    /// roots are one contact or an error.
    NoRealIntersection,
}

/// The radius of a cylinder wall offset by `distance` along its OUTWARD normal.
///
/// `outward_away` = the wall's outward normal points away from the axis
/// (material inside the cylinder, e.g. a boss); `false` is a bore wall whose
/// outward normal points at the axis. So the offset radius is `R + distance`
/// or `R − distance` respectively — and `R + (−r) ≡ R − r` exactly in IEEE, so
/// this reproduces both hand-written `R ∓ r` expressions bit for bit.
///
/// Separate from the intersectors on purpose: `mixed_support` checks this
/// refusal BEFORE it solves for the wall-pair line, and merging the steps
/// would reorder its refusal messages.
pub(crate) fn offset_cylinder_radius(
    radius: f64,
    outward_away: bool,
    distance: f64,
) -> Result<f64, OffsetPairDegeneracy> {
    let rho = if outward_away {
        radius + distance
    } else {
        radius - distance
    };
    if !(rho > 0.0) {
        return Err(OffsetPairDegeneracy::CollapsedCylinder);
    }
    Ok(rho)
}

/// The intersection LINE of two offset planes, both carried through `anchor`.
///
/// Plane `i` is `{ x : n̂_i·(x − anchor) = d_i }` — the plane through `anchor`
/// with unit outward normal `n̂_i`, offset by the signed `d_i` along it. The
/// answer is the line where the two offset planes meet, anchored at the foot of
/// the perpendicular from `anchor` and directed along `n̂_a × n̂_b`.
///
/// **Preconditions:** `normal_a` and `normal_b` are unit. Nothing is
/// re-normalized here — a defensive `.normalized()` on an already-unit vector
/// perturbs its last bits and would break the bit-identity this module claims.
///
/// The third row of the 3×3 system is what pins the anchor: `n̂_a × n̂_b`
/// normalized, with a zero right-hand side. `mixed_concave` passes the
/// AXIS-ORIENTED (possibly negated) direction there today; negating a row whose
/// right-hand side is zero leaves `solve_small`'s answer bit-identical —
/// pivot selection compares magnitudes, and every subsequent operation on that
/// row is an exact IEEE negation of the original — which
/// `bit_identical_plane_pair_under_a_negated_direction_row` pins.
pub(crate) fn offset_plane_pair(
    anchor: Vec3,
    normal_a: Vec3,
    distance_a: f64,
    normal_b: Vec3,
    distance_b: f64,
) -> Result<OffsetLine, OffsetPairDegeneracy> {
    let direction = normal_a
        .cross(normal_b)
        .normalized()
        .map_err(|_| OffsetPairDegeneracy::ParallelPlanes)?;
    let matrix = [
        [normal_a.x, normal_a.y, normal_a.z],
        [normal_b.x, normal_b.y, normal_b.z],
        [direction.x, direction.y, direction.z],
    ];
    let delta = crate::fit::solve_small::<3>(matrix, [distance_a, distance_b, 0.0], 3)
        .map_err(|_| OffsetPairDegeneracy::SingularSystem)?;
    Ok(OffsetLine {
        point: anchor.add(Vec3::new(delta[0], delta[1], delta[2])),
        direction,
    })
}

/// The single POINT where three offset planes meet, all carried through
/// `anchor` — the same construction as [`offset_plane_pair`] with the third
/// plane replacing the anchoring row.
///
/// This is the rolling-ball corner centre: the point at signed distance `d_i`
/// from each of the three wall planes. **Preconditions:** unit normals.
/// Returns [`OffsetPairDegeneracy::SingularSystem`] when the three normals are
/// coplanar, i.e. no such point exists.
///
/// Deliberately only THREE: `convex.rs` handles `N > 3` by least squares on the
/// normal equations and then REQUIRES the residual to vanish, because for `N > 3`
/// the question changes from "solve" to "does a common tangent ball exist at
/// all". That is a different problem and stays where it is.
pub(crate) fn offset_plane_triple(
    anchor: Vec3,
    normals: [Vec3; 3],
    distances: [f64; 3],
) -> Result<Vec3, OffsetPairDegeneracy> {
    let matrix = [
        [normals[0].x, normals[0].y, normals[0].z],
        [normals[1].x, normals[1].y, normals[1].z],
        [normals[2].x, normals[2].y, normals[2].z],
    ];
    let delta = crate::fit::solve_small::<3>(matrix, distances, 3)
        .map_err(|_| OffsetPairDegeneracy::SingularSystem)?;
    Ok(anchor.add(Vec3::new(delta[0], delta[1], delta[2])))
}

/// Where a straight `line` meets the cylinder of radius `rho` about
/// `axis_point`/`axis_dir` — **both** roots, in the fixed algebraic order
/// `(−b − √Δ)/2a` then `(−b + √Δ)/2a`.
///
/// `rho` is the ALREADY-OFFSET radius from [`offset_cylinder_radius`]; the line
/// is typically the offset-plane pair's locus from [`offset_plane_pair`], which
/// makes the whole call a plane × plane × cylinder solve. The general leading
/// coefficient `a = |d̂ − (d̂·â)â|²` is kept because the line is NOT constrained
/// to be perpendicular to the axis here — see
/// [`axial_plane_meets_offset_cylinder`] for the case that is.
///
/// **Preconditions:** `axis_dir` unit. A tangency (`Δ = 0`) comes back as the
/// same point twice, on purpose: the caller decides whether that is one contact
/// or a refusal.
pub(crate) fn line_meets_offset_cylinder(
    line: &OffsetLine,
    axis_point: Vec3,
    axis_dir: Vec3,
    rho: f64,
) -> Result<[Vec3; 2], OffsetPairDegeneracy> {
    let w0 = line.point.sub(axis_point);
    let w0p = w0.sub(axis_dir.scale(w0.dot(axis_dir)));
    let dlp = line
        .direction
        .sub(axis_dir.scale(line.direction.dot(axis_dir)));
    let a = dlp.dot(dlp);
    if a < 1e-12 {
        return Err(OffsetPairDegeneracy::LineParallelToAxis);
    }
    let b = 2.0 * w0p.dot(dlp);
    let c = w0p.dot(w0p) - rho * rho;
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return Err(OffsetPairDegeneracy::NoRealIntersection);
    }
    let sq = disc.sqrt();
    Ok([
        line.point.add(line.direction.scale((-b - sq) / (2.0 * a))),
        line.point.add(line.direction.scale((-b + sq) / (2.0 * a))),
    ])
}

/// Where an offset plane PARALLEL TO THE AXIS meets the cylinder of radius
/// `rho` about `axis_point`/`axis_dir`: two lines parallel to the axis,
/// returned as the two points at which they cross the plane through `anchor`
/// perpendicular to the axis — **both** roots, in the fixed algebraic order
/// `(−b − √Δ)/2` then `(−b + √Δ)/2`.
///
/// The plane is `{ x : n̂·(x − anchor) = distance }`, so its anchored point is
/// `anchor + distance·n̂` and the in-plane direction perpendicular to the axis
/// is `n̂ × â`. Because that direction is a UNIT vector perpendicular to the
/// axis, the quadratic's leading coefficient is exactly `1` in exact arithmetic
/// and the routine uses the `a = 1` form. That is not an approximation and not a
/// duplicate of [`line_meets_offset_cylinder`]: computing `a = |d̂⊥|²` in
/// floating point returns `1` only about half the time and `1 ± 1–2 ULP`
/// otherwise (measured over 200k random frames — see
/// `the_general_and_unit_leading_coefficient_lanes_are_not_bitwise_interchangeable`),
/// which would move every root in its last bits for no gain in accuracy.
///
/// **Preconditions:** `normal` and `axis_dir` unit, and `|n̂·â|` negligible —
/// the plane genuinely parallel to the axis. The CALLER checks that and owns
/// the refusal, because the message ("the axis is tilted against the cap
/// normal — the closure is not an exact surface of revolution") is a statement
/// about the corner, not about this algebra. A plane PERPENDICULAR to the axis
/// is caught here as [`OffsetPairDegeneracy::PlaneNormalAlongAxis`]: its
/// section is a circle, which this routine does not answer.
pub(crate) fn axial_plane_meets_offset_cylinder(
    anchor: Vec3,
    normal: Vec3,
    distance: f64,
    axis_point: Vec3,
    axis_dir: Vec3,
    rho: f64,
) -> Result<[Vec3; 2], OffsetPairDegeneracy> {
    let along = normal
        .cross(axis_dir)
        .normalized()
        .map_err(|_| OffsetPairDegeneracy::PlaneNormalAlongAxis)?;
    // AFTER the perpendicular-plane refusal, never before it: a plane whose
    // normal runs ALONG the axis is a real degeneracy this routine answers with
    // `PlaneNormalAlongAxis`, and asserting first would turn that loud refusal
    // into a debug-build panic. What is left for the assert is the case no
    // caller may pass and no return value can express — a MILD tilt, where the
    // true section is an ellipse and the `a = 1` form would quietly answer the
    // wrong question.
    debug_assert!(
        normal.dot(axis_dir).abs() <= 1e-6,
        "axial_plane_meets_offset_cylinder: the plane must be parallel to the axis"
    );
    let base = anchor.add(normal.scale(distance));
    let w0 = base.sub(axis_point);
    let w0p = w0.sub(axis_dir.scale(w0.dot(axis_dir)));
    let b = 2.0 * w0p.dot(along);
    let c = w0p.dot(w0p) - rho * rho;
    let disc = b * b - 4.0 * c;
    if disc < 0.0 {
        return Err(OffsetPairDegeneracy::NoRealIntersection);
    }
    let sq = disc.sqrt();
    Ok([
        base.add(along.scale((-b - sq) / 2.0)),
        base.add(along.scale((-b + sq) / 2.0)),
    ])
}

// ---------------------------------------------------------------------------
// Migration diagnostic
// ---------------------------------------------------------------------------

/// Per-site agreement between this module's closed form and the hand-written
/// algebra it replaced, aggregated over one thread.
///
/// Enabled by `BREP_OFFSET_PAIR_DIAG=1`. Each converted site calls one `diag::`
/// entry point with its RAW inputs and the shared result; the entry point
/// re-runs the pre-slice formula VERBATIM (see [`diag::legacy`]) and records
/// the worst disagreement and the number of ULPs between them.
///
/// Both readings matter and are reported: a site with calls and zero
/// disagreement is evidence the corpus reaches it and the forms agree there; a
/// site that never appears in the dump was **never reached at all**, and its
/// only coverage is this module's unit tests. Slice 0 (§8.4) and slice 1 (§9.3)
/// both had to say that out loud, so the counts come first.
pub(crate) mod diag {
    use super::*;

    /// The pre-slice algebra, copied verbatim from the sites this slice
    /// converted, kept ONLY so the diagnostic can measure against it. Nothing
    /// outside `diag` may call these.
    pub(super) mod legacy {
        use super::*;

        /// `mixed_concave.rs` / `mixed_support.rs` / `mixed_curved.rs`: the
        /// 3×3 offset-plane pair solve, with the site's own third row.
        pub(crate) fn plane_pair(
            anchor: Vec3,
            normal_a: Vec3,
            distance_a: f64,
            normal_b: Vec3,
            distance_b: f64,
            third_row: Vec3,
        ) -> Option<Vec3> {
            let mat = [
                [normal_a.x, normal_a.y, normal_a.z],
                [normal_b.x, normal_b.y, normal_b.z],
                [third_row.x, third_row.y, third_row.z],
            ];
            let d = crate::fit::solve_small::<3>(mat, [distance_a, distance_b, 0.0], 3).ok()?;
            Some(anchor.add(Vec3::new(d[0], d[1], d[2])))
        }

        /// `convex.rs`: the trihedral corner-ball centre.
        pub(crate) fn plane_triple(
            anchor: Vec3,
            normals: [Vec3; 3],
            distances: [f64; 3],
        ) -> Option<Vec3> {
            let mat = [
                [normals[0].x, normals[0].y, normals[0].z],
                [normals[1].x, normals[1].y, normals[1].z],
                [normals[2].x, normals[2].y, normals[2].z],
            ];
            let d = crate::fit::solve_small::<3>(mat, distances, 3).ok()?;
            Some(anchor.add(Vec3::new(d[0], d[1], d[2])))
        }

        /// `mixed_support.rs::curved_wall_ball_candidates`: the general
        /// line × offset-cylinder quadratic.
        pub(crate) fn line_cylinder(
            p0: Vec3,
            dl: Vec3,
            axis_point: Vec3,
            axis_dir: Vec3,
            rho: f64,
        ) -> Option<[Vec3; 2]> {
            let w0 = p0.sub(axis_point);
            let w0p = w0.sub(axis_dir.scale(w0.dot(axis_dir)));
            let dlp = dl.sub(axis_dir.scale(dl.dot(axis_dir)));
            let a = dlp.dot(dlp);
            if a < 1e-12 {
                return None;
            }
            let b = 2.0 * w0p.dot(dlp);
            let c = w0p.dot(w0p) - rho * rho;
            let disc = b * b - 4.0 * a * c;
            if disc < 0.0 {
                return None;
            }
            let sq = disc.sqrt();
            Some([
                p0.add(dl.scale((-b - sq) / (2.0 * a))),
                p0.add(dl.scale((-b + sq) / (2.0 * a))),
            ])
        }

        /// `mixed_curved.rs::detect_mixed_concave_curved_corner`: the
        /// axis-parallel offset plane × offset-cylinder quadratic.
        pub(crate) fn axial_plane_cylinder(
            corner: Vec3,
            na: Vec3,
            radius: f64,
            axis_point: Vec3,
            axis_dir: Vec3,
            rho_q: f64,
        ) -> Option<[Vec3; 2]> {
            let dl = na.cross(axis_dir).normalized().ok()?;
            let p0 = corner.add(na.scale(radius));
            let w0 = p0.sub(axis_point);
            let w0p = w0.sub(axis_dir.scale(w0.dot(axis_dir)));
            let b = 2.0 * w0p.dot(dl);
            let c = w0p.dot(w0p) - rho_q * rho_q;
            let disc = b * b - 4.0 * c;
            if disc < 0.0 {
                return None;
            }
            let sq = disc.sqrt();
            Some([p0.add(dl.scale((-b - sq) / 2.0)), p0.add(dl.scale((-b + sq) / 2.0))])
        }
    }

    #[derive(Default, Clone, Copy)]
    struct SiteStats {
        /// Calls the site made — the REACH reading.
        calls: u64,
        /// Calls where exactly one of the two forms produced an answer.
        outcome_splits: u64,
        /// Calls where both answered but not bit-identically.
        bit_differences: u64,
        /// Worst absolute distance between the two forms' points.
        worst_delta: f64,
        /// Worst per-coordinate ULP gap between the two forms' points.
        worst_ulps: u64,
    }

    struct DiagnosticSink {
        sites: std::collections::BTreeMap<&'static str, SiteStats>,
    }

    impl Drop for DiagnosticSink {
        fn drop(&mut self) {
            // One `eprintln!` per line: threads exit concurrently under
            // `cargo test` and a line assembled from several `eprint!`s
            // interleaves with another thread's (audit §9.3's lesson).
            for (site, stats) in &self.sites {
                eprintln!(
                    "offset-pair-diag site={site} calls={} split={} bitdiff={} \
                     worst_delta={:.3e} worst_ulps={}",
                    stats.calls,
                    stats.outcome_splits,
                    stats.bit_differences,
                    stats.worst_delta,
                    stats.worst_ulps
                );
            }
        }
    }

    thread_local! {
        static DIAGNOSTIC: std::cell::RefCell<DiagnosticSink> =
            std::cell::RefCell::new(DiagnosticSink {
                sites: std::collections::BTreeMap::new(),
            });
    }

    /// Off unless `BREP_OFFSET_PAIR_DIAG` is set; the only cost on the hot path
    /// when off is one `OnceLock` read.
    pub(crate) fn enabled() -> bool {
        static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        *ON.get_or_init(|| std::env::var("BREP_OFFSET_PAIR_DIAG").is_ok())
    }

    fn ulps(a: f64, b: f64) -> u64 {
        if a == b {
            return 0;
        }
        if !a.is_finite() || !b.is_finite() {
            return u64::MAX;
        }
        // Monotone ordered mapping of f64 bits, so the gap counts
        // representable doubles between the two values across zero.
        let key = |x: f64| -> i64 {
            let bits = x.to_bits() as i64;
            if bits < 0 {
                i64::MIN - bits
            } else {
                bits
            }
        };
        key(a).abs_diff(key(b))
    }

    fn record(site: &'static str, shared: Option<&[Vec3]>, legacy: Option<&[Vec3]>) {
        DIAGNOSTIC.with(|sink| {
            let mut sink = sink.borrow_mut();
            let stats = sink.sites.entry(site).or_default();
            stats.calls += 1;
            match (shared, legacy) {
                (Some(shared), Some(legacy)) if shared.len() == legacy.len() => {
                    let mut differed = false;
                    for (a, b) in shared.iter().zip(legacy.iter()) {
                        let delta = a.sub(*b).length();
                        if delta > stats.worst_delta {
                            stats.worst_delta = delta;
                        }
                        for (x, y) in [(a.x, b.x), (a.y, b.y), (a.z, b.z)] {
                            let gap = ulps(x, y);
                            if gap > 0 {
                                differed = true;
                            }
                            if gap > stats.worst_ulps {
                                stats.worst_ulps = gap;
                            }
                        }
                    }
                    if differed {
                        stats.bit_differences += 1;
                    }
                }
                (None, None) => {}
                _ => stats.outcome_splits += 1,
            }
        });
    }

    /// Record one [`offset_plane_pair`] call against the site's own 3×3 solve.
    /// `third_row` is the direction row the site wrote before this slice.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn plane_pair(
        site: &'static str,
        anchor: Vec3,
        normal_a: Vec3,
        distance_a: f64,
        normal_b: Vec3,
        distance_b: f64,
        third_row: Vec3,
        shared: Option<&OffsetLine>,
    ) {
        if !enabled() {
            return;
        }
        let legacy =
            legacy::plane_pair(anchor, normal_a, distance_a, normal_b, distance_b, third_row);
        let shared = shared.map(|line| [line.point]);
        record(
            site,
            shared.as_ref().map(|p| &p[..]),
            legacy.as_ref().map(std::slice::from_ref),
        );
    }

    /// Record one [`offset_plane_triple`] call against the site's own solve.
    pub(crate) fn plane_triple(
        site: &'static str,
        anchor: Vec3,
        normals: [Vec3; 3],
        distances: [f64; 3],
        shared: Option<Vec3>,
    ) {
        if !enabled() {
            return;
        }
        let legacy = legacy::plane_triple(anchor, normals, distances);
        record(
            site,
            shared.as_ref().map(std::slice::from_ref),
            legacy.as_ref().map(std::slice::from_ref),
        );
    }

    /// Record one [`line_meets_offset_cylinder`] call against the site's own
    /// quadratic.
    pub(crate) fn line_cylinder(
        site: &'static str,
        line: &OffsetLine,
        axis_point: Vec3,
        axis_dir: Vec3,
        rho: f64,
        shared: Option<&[Vec3; 2]>,
    ) {
        if !enabled() {
            return;
        }
        let legacy = legacy::line_cylinder(line.point, line.direction, axis_point, axis_dir, rho);
        record(
            site,
            shared.map(|roots| &roots[..]),
            legacy.as_ref().map(|roots| &roots[..]),
        );
    }

    /// Record one [`axial_plane_meets_offset_cylinder`] call against the site's
    /// own quadratic.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn axial_plane_cylinder(
        site: &'static str,
        anchor: Vec3,
        normal: Vec3,
        distance: f64,
        axis_point: Vec3,
        axis_dir: Vec3,
        rho: f64,
        shared: Option<&[Vec3; 2]>,
    ) {
        if !enabled() {
            return;
        }
        let legacy = legacy::axial_plane_cylinder(anchor, normal, distance, axis_point, axis_dir, rho);
        record(
            site,
            shared.map(|roots| &roots[..]),
            legacy.as_ref().map(|roots| &roots[..]),
        );
    }
}

