//! Region-scoped offset REGULARITY — one definition of "does the equidistant
//! surface fold over THIS trimmed region", shared by `thicken`'s sheet gate and
//! `offset_shell`'s carrier-collapse lane.
//!
//! The condition itself is the classical one and neither caller changes it:
//! moving a surface by `δ` along its parametrization normal `n = Su × Sv`
//! scales the area element, per principal direction, by `1 − δ·κ` with `κ`
//! signed against that same normal ([`NurbsSurface::principal_curvatures`]).
//! The offset folds through the evolute where that factor reaches zero.
//!
//! What this module changes is WHERE the condition is asked.  Both callers used
//! to sweep a fixed grid over the whole surface DOMAIN, which answers about the
//! carrier and not about the face: a cone's apex row rejected a sheet trimmed to
//! the cone's wide end, and a fold narrower than the grid step INSIDE the trim
//! was not seen at all.  Here the samples are laid over the trim's own parameter
//! region, and their density is derived rather than fixed:
//!
//! * **Per knot span.**  A NURBS surface is one rational polynomial patch per
//!   knot span, so curvature cannot spike between spans without the spike living
//!   inside one.  Every span the region touches therefore gets its own samples,
//!   and a span covering 0.2% of the domain gets as many as one covering half of
//!   it.  A grid uniform over the domain is exactly what steps over the narrow
//!   one.
//! * **Per offset distance.**  Inside one span the fold's own scale is the
//!   offset distance — the fold happens where the curvature RADIUS falls to
//!   `|δ|` — so the arc-length step is additionally capped at a fraction of
//!   `|δ|`, converted to parameter steps through the first fundamental form.
//! * **Plus the trim boundary itself.**  The loop pcurves are sampled directly:
//!   the worst point of a region is often on its rim, and a region small enough
//!   to fall between grid nodes still gets a population this way.
//!
//! Refinement around the worst node sharpens the location that goes into a
//! refusal.  It never votes in the sample census, so the census stays an
//! unbiased population — which is what the shell lane's "collapsed EVERYWHERE in
//! this trim" question needs, and what a refinement biased toward the minimum
//! would quietly destroy.
//!
//! # Between the nodes
//!
//! The census cannot see a fold narrower than its node spacing, and that is
//! not hypothetical: on a free-form patch it let the carve keep a piece with a
//! folded ISLAND inside it (factor −2.1e-3, 0.03 across, between nodes 0.028
//! apart), which `thicken` then refused only as a self-intersecting result.
//! [`refine_between_nodes`] looks inside every grid cell whose corners agree but
//! read close to the level against the slope their neighbourhood shows, halving
//! the riskiest cells first. What it finds is reported beside the census
//! (`between_collapsed`, `between_regular`), never in it, and it seeds the fold
//! trace so an island comes back as the closed branch it is.
//!
//! What this does NOT do: no finite sampling can PROVE a region regular, and the
//! refinement's slope is estimated from the very nodes an island hides between.
//! So the claim is a measurement, on the island family in this module's tests
//! (an island is the folded top of a bump, at every width and depth the family
//! reaches, placed on a knot halfway between nodes or at random): with the SHEET
//! budget every island at least **0.02 of the node spacing** wide is caught, where
//! the grid alone missed 14 of those at least 0.1 wide; with SHELL_FACE every
//! island at least **0.25 of the node spacing** is. A narrower island is missed
//! when the nodes round it read far from the level (0.5 and more on the misses
//! measured — nothing computed from those nodes can see it), or when it is
//! narrower than the finest cell, `2⁻⁷` of the node spacing. The refinement has
//! no evaluation budget to run out of: each risky cell costs at most
//! `5·(4⁷ − 1)/3` = 27,305 samples, so it always finishes, and nothing it did
//! not reach is reported regular. What it costs on the lib, integration and
//! case populations is in the 2026-09-17 record. Away from the between-node question
//! the density rules still hold: a fold has to hide inside a single knot span,
//! below the `|δ|`-derived arc-length step, and away from the rim, to be missed.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::topology::FaceRecord;
use crate::{NurbsCurve, NurbsSurface};

/// The offset family's own trace switch, the same `BREP_OS_DEBUG` the shell
/// reads — this module answers for both callers, so it cannot borrow either
/// one's macro.
fn debug_enabled() -> bool {
    std::env::var("BREP_OS_DEBUG").is_ok_and(|value| !value.is_empty() && value != "0")
}

/// The worst sample a scan saw: the fold factor, where it is, and the
/// displacement and curvature that produced it.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FoldSample {
    /// `1 − δ·κ`; at or below the caller's collapse factor the offset folds.
    pub(crate) factor: f64,
    pub(crate) u: f64,
    pub(crate) v: f64,
    /// The principal curvature that produced `factor`, signed against `Su × Sv`.
    pub(crate) kappa: f64,
    /// The displacement along `Su × Sv` that produced `factor`.
    pub(crate) displacement: f64,
}

impl FoldSample {
    /// The concave curvature radius the displacement is measured against.
    pub(crate) fn radius(&self) -> f64 {
        1.0 / self.kappa.abs().max(1e-300)
    }
}

/// What a scan found over a region.
#[derive(Clone, Debug, Default)]
pub(crate) struct RegularityScan {
    /// Census population: grid and trim-boundary samples inside the region at
    /// which the curvature could be read at all.
    pub(crate) sampled: usize,
    /// How many of those collapsed.
    pub(crate) collapsed: usize,
    /// The worst factor seen, refinement included.
    pub(crate) worst: Option<FoldSample>,
    /// What the between-node refinement found that the census could not: cells
    /// whose four nodes all read regular but which hold a collapsed sample, and
    /// cells whose nodes all read collapsed but which hold a regular one. Not
    /// census members — the census stays the unbiased grid — but a region with
    /// either is not what its census says it is. See [`refine_between_nodes`].
    pub(crate) between_collapsed: usize,
    pub(crate) between_regular: usize,
    /// What that refinement cost.
    pub(crate) between_evaluations: usize,
    /// Cells still risky at the finest spacing the refinement reaches.
    pub(crate) between_unresolved: usize,
    /// Dips the between-node refinement DESCENDED: local minima of the node
    /// grid on the regular side, followed to their minimum (see
    /// [`descend_dips`]). A collapsed sample a descent reaches counts in
    /// `between_collapsed`.
    pub(crate) descents: usize,
    pub(crate) descent_evaluations: usize,
    /// The descent budget ran out with prominent dips still undescended.
    pub(crate) descent_capped: bool,
    /// The cells behind `between_unresolved`, kept so the certainty step can
    /// refine them further on demand ([`RegularityScan::certify`]) instead of
    /// every scan paying for that.
    pub(crate) unresolved_cells: Vec<RiskyCell>,
    /// What the certainty step spent, and whether its budget ran out.
    pub(crate) certify_evaluations: usize,
    pub(crate) certify_capped: bool,
}

/// How much deeper than [`REFINE_DEPTH`] the certainty step may halve a cell
/// still risky at the scan's finest spacing: `2⁻¹⁴` of the node spacing.
const CERTIFY_DEPTH: usize = 14;
/// Samples the certainty step may spend on those cells, riskiest first. The
/// cell count can quadruple per level, so this is what makes the step
/// bounded; a region that runs out is reported undecided, not regular.
const CERTIFY_EVALUATIONS: usize = 200_000;

/// What a scan can say about a region it found nothing collapsed in — the
/// CERTAINTY step. "Nothing collapsed" is a census reading; whether the census
/// could have seen a fold is a separate question, and a caller that builds on
/// the first without asking the second ships a folded carrier that validates.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Certainty {
    /// Every risky cell was cleared or refined to a sample of the other sign,
    /// and every prominent dip was descended to a minimum above the level.
    /// As far as sampling can say, the region is regular.
    Regular,
    /// A sample at or below the level: on the grid, on the rim, between the
    /// nodes, or at the bottom of a dip.
    Folded(FoldSample),
    /// The scan could not decide: cells still risky at its finest spacing (the
    /// factor sits within a hair of the level over an area), or the descent
    /// budget ran out with dips still undescended.
    Uncertain {
        /// Cells still risky after the certainty step's own refinement.
        unresolved: usize,
        /// A budget ran out first: the dip descent's, or the certainty step's.
        budget_exhausted: bool,
        /// The least factor seen, refinement and descents included.
        least: Option<FoldSample>,
    },
}

impl RegularityScan {
    /// The CERTAINTY step: what this scan's reading at `level` can be trusted
    /// to mean, refining the cells it left unresolved (bounded by
    /// [`CERTIFY_DEPTH`] and [`CERTIFY_EVALUATIONS`]) before answering.
    ///
    /// `surface`, `region` and `displacements` must be the ones the scan was
    /// taken over. A second call refines nothing more and repeats the answer.
    pub(crate) fn certify(
        &mut self,
        surface: &NurbsSurface,
        region: &TrimRegion,
        displacements: &[f64],
        level: f64,
    ) -> Certainty {
        if let Some(worst) = self.worst.filter(|worst| worst.factor <= level) {
            return Certainty::Folded(worst);
        }
        if !self.unresolved_cells.is_empty() && !self.certify_capped {
            // The scan's own refinement judges a cell by the slope its 3 × 3
            // NEIGHBOURHOOD shows and never lets a child fall below its parent,
            // which is what catches a feature between nodes — and what leaves a
            // cell beside a smooth minimum a hair above the level risky at every
            // depth: a torus grown to 0.999 of its fold onset reads 1e-3 along
            // its inner equator, its nodes a spacing away read 0.13 and 0.37,
            // and that inherited slope (4.5 per unit of v) covers 2.9e-3 across
            // a cell already 2⁻⁷ of the spacing wide, a hundred times what the
            // cell's own corners vary by. Every dip those nodes show has by now
            // been descended, so the certainty step judges each unresolved cell
            // by its OWN corners — the slope across the cell at the finest
            // spacing the scan reached — and halves, under the same local rule,
            // only the ones that still do not clear.
            let work = std::mem::take(&mut self.unresolved_cells)
                .into_iter()
                .filter_map(|mut cell| {
                    cell.slopes = own_slopes(&cell);
                    cell.risk = cell_risk(
                        &cell.corners,
                        cell.u[1] - cell.u[0],
                        cell.v[1] - cell.v[0],
                        cell.slopes,
                    )?;
                    Some(cell)
                })
                .collect::<std::collections::BinaryHeap<RiskyCell>>();
            let mut found = BetweenNodes::default();
            refine_cells(
                surface,
                region,
                displacements,
                level,
                work,
                CERTIFY_DEPTH,
                Some(CERTIFY_EVALUATIONS),
                &mut found,
            );
            self.between_collapsed += found.collapsed;
            self.between_regular += found.regular;
            self.certify_evaluations += found.evaluations;
            self.certify_capped = found.capped;
            self.between_unresolved = found.unresolved_cells.len();
            if let Some(worst) = found.worst {
                self.keep_worst(worst);
            }
            if debug_enabled() {
                eprintln!(
                    "regularity: the certainty step refined {} more sample(s) — {} collapsed and {} \
                     regular sample(s) found, {} cell(s) still risky{}",
                    found.evaluations,
                    found.collapsed,
                    found.regular,
                    found.unresolved_cells.len(),
                    if found.capped { " (budget ran out)" } else { "" }
                );
            }
            self.unresolved_cells = found.unresolved_cells;
            if let Some(worst) = self.worst.filter(|worst| worst.factor <= level) {
                return Certainty::Folded(worst);
            }
        }
        if self.between_unresolved > 0 || self.descent_capped || self.certify_capped {
            return Certainty::Uncertain {
                unresolved: self.between_unresolved,
                budget_exhausted: self.descent_capped || self.certify_capped,
                least: self.worst,
            };
        }
        Certainty::Regular
    }
}

/// The largest slope a cell's own corners show, per direction.
fn own_slopes(cell: &RiskyCell) -> (f64, f64) {
    let [c00, c10, c01, c11] = cell.corners;
    let (du, dv) = (cell.u[1] - cell.u[0], cell.v[1] - cell.v[0]);
    (
        (c10 - c00).abs().max((c11 - c01).abs()) / du,
        (c01 - c00).abs().max((c11 - c10).abs()) / dv,
    )
}

/// How hard a caller wants its region swept.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ScanBudget {
    /// Samples per knot span per direction, before the node cap.
    pub(crate) per_span: usize,
    /// Arc-length steps per offset distance: the step is `|δ| / this`.
    pub(crate) steps_per_distance: f64,
    /// Ceiling on the grid (nodes, before the region test).
    pub(crate) max_nodes: usize,
    /// Refine around the worst node to sharpen the reported location.
    pub(crate) refine: bool,
}

impl ScanBudget {
    /// The sheet gate: one scan per thicken, so it can afford to hunt.  The
    /// per-span floor is what sees a tight bend that occupies a sliver of the
    /// parameter domain, and the refinement is what puts a usable `(u, v)` in
    /// the refusal instead of the nearest node.
    pub(crate) const SHEET: Self = Self {
        per_span: 9,
        steps_per_distance: 4.0,
        max_nodes: 40_000,
        refine: true,
    };

    /// The shell's per-face collapse probe.  It runs on every non-analytic face
    /// of a shell and its question is "is this trim collapsed EVERYWHERE", which
    /// buys coverage rather than spike-hunting — so it takes a coverage budget
    /// of the same order as the 7×7 whole-face grid it replaces, spent over the
    /// trim instead of over the carrier.
    pub(crate) const SHELL_FACE: Self = Self {
        per_span: 3,
        steps_per_distance: 1.0,
        max_nodes: 512,
        refine: false,
    };
}

/// A trimmed parameter region: the trim loops as `(u, v)` polylines, plus the
/// parameter box they span.
///
/// Winding is deliberately not consulted.  Containment is crossing parity over
/// every loop at once, which puts a point in a hole OUTSIDE the region however
/// the hole is wound, and which needs no convexity or nesting assumption.
pub(crate) struct TrimRegion {
    loops: Vec<Vec<[f64; 2]>>,
    bounds: [f64; 4],
}

impl TrimRegion {
    /// The whole carrier: what a face with no loops covers, and the region a
    /// caller means when it has no trim to name.
    pub(crate) fn whole_domain(surface: &NurbsSurface) -> Result<Self, KernelRefusal> {
        let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
        let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
        Ok(Self {
            loops: Vec::new(),
            bounds: [u0, u1, v0, v1],
        })
    }

    /// A region from bare parameter-space loops — `thicken`'s form, where the
    /// trim is passed in rather than stored on a face.
    ///
    /// Malformed loops are not this module's refusal to make: a loop that does
    /// not close is polylined and closed implicitly, so the region is still a
    /// region and the caller's own loop validation keeps the right message.
    pub(crate) fn from_pcurve_loops(
        surface: &NurbsSurface,
        loops: &[Vec<NurbsCurve>],
    ) -> Result<Self, KernelRefusal> {
        let mut polylines = Vec::with_capacity(loops.len());
        for loop_curves in loops {
            let mut polyline = Vec::new();
            for curve in loop_curves {
                append_pcurve_polyline(curve, &mut polyline)?;
            }
            if polyline.len() >= 3 {
                polylines.push(polyline);
            }
        }
        Self::from_polylines(surface, polylines)
    }

    /// A region from a face's own stored trim.
    pub(crate) fn from_face(face: &FaceRecord) -> Result<Self, KernelRefusal> {
        let mut polylines = Vec::with_capacity(face.loops.len());
        for loop_record in &face.loops {
            let mut polyline = Vec::new();
            for coedge in &loop_record.coedges {
                append_pcurve_polyline(&coedge.pcurve, &mut polyline)?;
            }
            if polyline.len() >= 3 {
                polylines.push(polyline);
            }
        }
        Self::from_polylines(&face.surface, polylines)
    }

    fn from_polylines(
        surface: &NurbsSurface,
        loops: Vec<Vec<[f64; 2]>>,
    ) -> Result<Self, KernelRefusal> {
        let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
        let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
        if loops.is_empty() {
            return Self::whole_domain(surface);
        }
        let (mut lo_u, mut hi_u) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut lo_v, mut hi_v) = (f64::INFINITY, f64::NEG_INFINITY);
        for polyline in &loops {
            for point in polyline {
                lo_u = lo_u.min(point[0]);
                hi_u = hi_u.max(point[0]);
                lo_v = lo_v.min(point[1]);
                hi_v = hi_v.max(point[1]);
            }
        }
        // A pcurve may run a rounding step outside its carrier's domain; the
        // region is what the carrier can be evaluated over.
        let bounds = [
            lo_u.max(u0).min(u1),
            hi_u.max(u0).min(u1),
            lo_v.max(v0).min(v1),
            hi_v.max(v0).min(v1),
        ];
        if !(bounds[1] > bounds[0]) || !(bounds[3] > bounds[2]) {
            // A degenerate trim box is not a region to judge: fall back to the
            // carrier so nothing is silently skipped — and SAY so, because
            // "why did this scan the whole carrier again" is otherwise a
            // bisect.
            if debug_enabled() {
                eprintln!(
                    "regularity: trim box degenerate (u[{:.6},{:.6}] v[{:.6},{:.6}]) — scanning \
                     the whole carrier instead",
                    bounds[0], bounds[1], bounds[2], bounds[3]
                );
            }
            return Self::whole_domain(surface);
        }
        Ok(Self { loops, bounds })
    }

    /// `[u0, u1, v0, v1]` — the trim's own parameter box, clamped to the
    /// carrier's domain.
    pub(crate) fn bounds(&self) -> [f64; 4] {
        self.bounds
    }

    /// Crossing parity against every loop at once.
    pub(crate) fn contains(&self, u: f64, v: f64) -> bool {
        if self.loops.is_empty() {
            return u >= self.bounds[0]
                && u <= self.bounds[1]
                && v >= self.bounds[2]
                && v <= self.bounds[3];
        }
        let mut inside = false;
        for polyline in &self.loops {
            let count = polyline.len();
            let mut previous = count - 1;
            for current in 0..count {
                let (a, b) = (polyline[previous], polyline[current]);
                if (a[1] > v) != (b[1] > v) {
                    let t = (v - a[1]) / (b[1] - a[1]);
                    if u < a[0] + t * (b[0] - a[0]) {
                        inside = !inside;
                    }
                }
                previous = current;
            }
        }
        inside
    }

    fn boundary_points(&self) -> impl Iterator<Item = [f64; 2]> + '_ {
        self.loops.iter().flat_map(|polyline| polyline.iter().copied())
    }
}

/// Sample a parameter-space curve into the polyline that stands for it.
///
/// Per knot span, for the same reason the surface grid is: a pcurve's shape is
/// one polynomial per span, so a span is the unit that cannot be stepped over.
/// The last parameter is deliberately omitted — the next pcurve of the loop
/// starts there, and a closed single-pcurve loop closes on its own.
fn append_pcurve_polyline(curve: &NurbsCurve, into: &mut Vec<[f64; 2]>) -> Result<(), KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Classify, "domain")?;
    if !(t1 > t0) {
        return Ok(());
    }
    let spans = breakpoints(&curve.knots, t0, t1).len().saturating_sub(1).max(1);
    let count = (8 * spans).clamp(16, 256);
    for index in 0..count {
        let t = t0 + (t1 - t0) * index as f64 / count as f64;
        let point = curve.evaluate(t).or_refuse(KernelStage::Classify, "evaluate")?;
        into.push([point.x, point.y]);
    }
    Ok(())
}

/// The distinct knot values bounding the spans that `[lo, hi]` touches,
/// `lo` and `hi` included.
fn breakpoints(knots: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    let width = (hi - lo).abs();
    let epsilon = 1e-12 * width.max(1.0);
    let mut points = vec![lo];
    for &knot in knots {
        if knot > lo + epsilon && knot < hi - epsilon && knot > *points.last().unwrap() + epsilon {
            points.push(knot);
        }
    }
    points.push(hi);
    points
}

/// Drop interior breakpoints until the span grid fits under `max_spans`.
fn thin(points: &mut Vec<f64>, max_spans: usize) {
    while points.len() > max_spans.max(1) + 1 {
        let kept: Vec<f64> = points
            .iter()
            .enumerate()
            .filter(|(index, _)| *index == 0 || *index == points.len() - 1 || index % 2 == 0)
            .map(|(_, value)| *value)
            .collect();
        if kept.len() == points.len() {
            break;
        }
        *points = kept;
    }
}

/// Strictly interior samples, `per_span` of them inside every span.
///
/// Interior on purpose: a pole or a seam sits ON a domain boundary, where the
/// parametrization is degenerate and the curvature is unreadable, and a knot is
/// where a one-sided curvature would be read.
fn span_samples(breaks: &[f64], per_span: usize) -> Vec<f64> {
    let per_span = per_span.max(1);
    let mut samples = Vec::with_capacity(breaks.len() * per_span);
    for window in breaks.windows(2) {
        let (lo, hi) = (window[0], window[1]);
        for index in 0..per_span {
            samples.push(lo + (hi - lo) * (index as f64 + 0.5) / per_span as f64);
        }
    }
    samples
}

/// Largest `|Su|` and `|Sv|` over a coarse pass of the region box — the first
/// fundamental form's diagonal, which is what turns a 3D step into a parameter
/// step.
fn metric_scale(surface: &NurbsSurface, bounds: [f64; 4]) -> (f64, f64) {
    const PROBES: usize = 5;
    let (mut su, mut sv) = (0.0f64, 0.0f64);
    for iu in 0..PROBES {
        let u = bounds[0] + (bounds[1] - bounds[0]) * (iu as f64 + 0.5) / PROBES as f64;
        for iv in 0..PROBES {
            let v = bounds[2] + (bounds[3] - bounds[2]) * (iv as f64 + 0.5) / PROBES as f64;
            if let Ok(derivatives) = surface.derivatives(u, v, 1) {
                su = su.max(derivatives[1][0].length());
                sv = sv.max(derivatives[0][1].length());
            }
        }
    }
    (su, sv)
}

/// The sample nodes a scan lays over a region: `per_span` per knot span in each
/// direction, floored by the arc-length cap and capped by the node budget.
///
/// Extracted so the fold TRACE (`offset/fold_locus.rs`) brackets the locus on
/// exactly the nodes the scan censuses.  "The scan can already bracket the fold
/// but does not trace it" is then literally true rather than approximately: a
/// tracer with a grid of its own could seed on a crossing this scan never saw,
/// and the two would disagree about the same face.
pub(crate) struct RegionGrid {
    pub(crate) samples_u: Vec<f64>,
    pub(crate) samples_v: Vec<f64>,
}

pub(crate) fn region_grid(
    surface: &NurbsSurface,
    bounds: [f64; 4],
    reach: f64,
    budget: ScanBudget,
) -> RegionGrid {
    let mut breaks_u = breakpoints(&surface.knots_u, bounds[0], bounds[1]);
    let mut breaks_v = breakpoints(&surface.knots_v, bounds[2], bounds[3]);
    // A fitted carrier can carry hundreds of knots; the span grid alone must
    // still fit the budget before anything is spent inside a span.
    let max_spans = (budget.max_nodes as f64).sqrt().max(2.0) as usize;
    thin(&mut breaks_u, max_spans);
    thin(&mut breaks_v, max_spans);
    let (spans_u, spans_v) = (breaks_u.len() - 1, breaks_v.len() - 1);

    let mut per_u = budget.per_span.max(1);
    let mut per_v = budget.per_span.max(1);
    // Arc-length cap: the fold's own scale is the offset distance, so no step
    // may cover more 3D length than a fraction of it.
    let step = reach / budget.steps_per_distance.max(1.0);
    let (metric_u, metric_v) = metric_scale(surface, bounds);
    let needed = |metric: f64, width: f64, spans: usize| -> usize {
        if metric <= 0.0 || step <= 0.0 {
            return 1;
        }
        ((metric * width / step) / spans as f64).ceil().max(1.0).min(512.0) as usize
    };
    per_u = per_u.max(needed(metric_u, bounds[1] - bounds[0], spans_u));
    per_v = per_v.max(needed(metric_v, bounds[3] - bounds[2], spans_v));
    while spans_u * per_u * spans_v * per_v > budget.max_nodes && (per_u > 1 || per_v > 1) {
        if per_u >= per_v {
            per_u -= 1;
        } else {
            per_v -= 1;
        }
    }
    RegionGrid {
        samples_u: span_samples(&breaks_u, per_u),
        samples_v: span_samples(&breaks_v, per_v),
    }
}

/// Worst `1 − δ·κ` at one parameter, over every requested displacement and both
/// principal directions — the scalar FIELD whose zero set is the fold locus.
///
/// `min` over the two principal factors is continuous (both principal
/// curvatures are continuous functions of the fundamental forms), so the field
/// is continuous wherever the parametrization is regular, and smooth away from
/// the umbilics where the two branches cross.  At the locus itself the branch
/// that vanishes is strictly the smaller one, so the crossing is generically
/// transversal and the trace in `offset/fold_locus.rs` can march it.
///
/// `None` where the curvature cannot be read at all (a pole, a seam, a
/// degenerate parametrization) — not zero, which would be a fold.
pub(crate) fn fold_sample_at(
    surface: &NurbsSurface,
    u: f64,
    v: f64,
    displacements: &[f64],
) -> Option<FoldSample> {
    let (kappa_min, kappa_max) = surface.principal_curvatures(u, v).ok()?;
    let mut worst: Option<FoldSample> = None;
    for &displacement in displacements {
        if displacement == 0.0 {
            continue;
        }
        for kappa in [kappa_min, kappa_max] {
            let factor = 1.0 - displacement * kappa;
            if worst.is_none_or(|current| factor < current.factor) {
                worst = Some(FoldSample {
                    factor,
                    u,
                    v,
                    kappa,
                    displacement,
                });
            }
        }
    }
    worst
}

impl RegularityScan {
    fn keep_worst(&mut self, candidate: FoldSample) {
        if self.worst.is_none_or(|current| candidate.factor < current.factor) {
            self.worst = Some(candidate);
        }
    }
}

/// Sweep `region` for the worst offset fold factor.
///
/// `displacements` are signed moves ALONG the parametrization normal `Su × Sv`
/// — the one convention this module has, so each caller converts its own signed
/// distance once at the call site instead of re-deriving a sign here.
pub(crate) fn scan_offset_regularity(
    surface: &NurbsSurface,
    region: &TrimRegion,
    displacements: &[f64],
    collapse_factor: f64,
    budget: ScanBudget,
) -> Result<RegularityScan, KernelRefusal> {
    let mut scan = RegularityScan::default();
    let reach = displacements
        .iter()
        .fold(0.0f64, |worst, distance| worst.max(distance.abs()));
    // A plane has no curvature to read and no offset that can fold.
    if reach == 0.0 || surface.is_affine().or_refuse(KernelStage::Classify, "is_affine")? {
        return Ok(scan);
    }
    let bounds = region.bounds();
    let RegionGrid {
        samples_u,
        samples_v,
    } = region_grid(surface, bounds, reach, budget);

    let record = |scan: &mut RegularityScan, sample: FoldSample| {
        scan.sampled += 1;
        if sample.factor <= collapse_factor {
            scan.collapsed += 1;
        }
        scan.keep_worst(sample);
    };
    // Every node of the box is read, inside the region or not: the census counts
    // only the ones inside, and the between-node refinement below needs the
    // others as the far corners of the cells the region's own rim passes through.
    let nodes = samples_u
        .iter()
        .map(|&u| {
            samples_v
                .iter()
                .map(|&v| fold_sample_at(surface, u, v, displacements))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    for (iu, &u) in samples_u.iter().enumerate() {
        for (iv, &v) in samples_v.iter().enumerate() {
            if let Some(sample) = nodes[iu][iv] {
                if region.contains(u, v) {
                    record(&mut scan, sample);
                }
            }
        }
    }
    // The rim is part of the region whatever parity says about a point sitting
    // exactly on it, and on a trim too small to catch a grid node it is the
    // whole population.  It is also the one part of the population a caller did
    // not size: a booleaned face carries as many polyline points as it has
    // knotty pcurves, which on the shell lane would outvote the grid the budget
    // was spent on.  Stride it to the same ceiling.
    let rim: Vec<[f64; 2]> = region.boundary_points().collect();
    let stride = (rim.len() / budget.max_nodes.max(1)).max(1);
    for point in rim.iter().step_by(stride) {
        let u = point[0].clamp(bounds[0], bounds[1]);
        let v = point[1].clamp(bounds[2], bounds[3]);
        if let Some(sample) = fold_sample_at(surface, u, v, displacements) {
            record(&mut scan, sample);
        }
    }

    let values = nodes
        .iter()
        .map(|column| {
            column
                .iter()
                .map(|sample| sample.map(|sample| sample.factor - collapse_factor))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let between = refine_between_nodes(
        surface,
        region,
        &samples_u,
        &samples_v,
        &values,
        displacements,
        collapse_factor,
    );
    scan.between_collapsed = between.collapsed;
    scan.between_regular = between.regular;
    scan.between_evaluations = between.evaluations;
    scan.between_unresolved = between.unresolved();
    scan.unresolved_cells = between.unresolved_cells;
    scan.descents = between.descents;
    scan.descent_evaluations = between.descent_evaluations;
    scan.descent_capped = between.descent_capped;
    if let Some(worst) = between.worst {
        scan.keep_worst(worst);
    }
    if debug_enabled() && (between.evaluations > 0 || between.descents > 0) {
        eprintln!(
            "regularity: between-node refinement read {} sample(s) in {} risky cell(s) — {} \
             collapsed and {} regular sample(s) the census's own nodes did not show, {} cell(s) \
             still risky at the finest spacing; {} dip(s) descended in {} sample(s){}",
            between.evaluations,
            between.cells,
            between.collapsed,
            between.regular,
            scan.between_unresolved,
            between.descents,
            between.descent_evaluations,
            if between.descent_capped { " (budget ran out)" } else { "" }
        );
    }

    if budget.refine {
        refine_worst(surface, region, displacements, &mut scan, bounds);
    }
    Ok(scan)
}

/// The least `1 − δ·κ` over a face's own trim, for a caller asking its own
/// question of the same field rather than the census's.
///
/// What it reports is what [`scan_offset_regularity`] read: the grid, the rim,
/// the refinement around the worst node when the budget asks for it, and the
/// between-node refinement. `bound` says how far that reading can be trusted
/// (see [`LeastFoldFactor::bound`]).
// Built for the outward-shell guard (`outward_fold_the_carve_missed`), which
// sweeps the same field on a grid of its own; it moves onto this entry point as
// a separate change, and until then only the tests call it. The callers that
// already hold a scan read the same certainty off it directly
// ([`RegularityScan::certify`]) rather than scanning the face twice.
#[cfg_attr(not(test), allow(dead_code))]
#[derive(Clone, Copy, Debug)]
pub(crate) struct LeastFoldFactor {
    /// The least factor found and where, or `None` when nothing was readable.
    pub(crate) least: Option<FoldSample>,
    /// The scan grid's largest node spacing, per direction.
    pub(crate) node_spacing: [f64; 2],
    /// The finest cell the between-node refinement may reach, per direction:
    /// `node_spacing / 2^REFINE_DEPTH`.
    pub(crate) finest_spacing: [f64; 2],
    /// Every cell whose nodes agreed was either cleared by its neighbourhood's
    /// slope bound or refined to a sample of the other sign, and every
    /// prominent dip was descended. `false` when a cell was still risky at the
    /// finest spacing — near a fold locus that is the normal case, and the
    /// least factor is then the least SEEN at that spacing — or the descent
    /// budget ran out. The same reading as [`RegularityScan::certify`].
    pub(crate) bound: bool,
    pub(crate) evaluations: usize,
}

/// [`LeastFoldFactor`] over `face`'s own trim, with `displacements` along the
/// parametrization normal `Su × Sv` (this module's one convention) and `level`
/// the fold factor at which a sample counts as collapsed.
#[cfg_attr(not(test), allow(dead_code))]
pub(crate) fn least_fold_factor(
    face: &FaceRecord,
    displacements: &[f64],
    level: f64,
    budget: ScanBudget,
) -> Result<LeastFoldFactor, KernelRefusal> {
    let region = TrimRegion::from_face(face)?;
    let reach = displacements
        .iter()
        .fold(0.0f64, |worst, distance| worst.max(distance.abs()));
    let grid = region_grid(&face.surface, region.bounds(), reach, budget);
    let spacing = |samples: &[f64]| {
        samples
            .windows(2)
            .map(|pair| pair[1] - pair[0])
            .fold(0.0f64, f64::max)
    };
    let node_spacing = [spacing(&grid.samples_u), spacing(&grid.samples_v)];
    let mut scan = scan_offset_regularity(&face.surface, &region, displacements, level, budget)?;
    let certainty = scan.certify(&face.surface, &region, displacements, level);
    let finest = (1u64 << REFINE_DEPTH) as f64;
    Ok(LeastFoldFactor {
        least: scan.worst,
        node_spacing,
        finest_spacing: [node_spacing[0] / finest, node_spacing[1] / finest],
        bound: !matches!(certainty, Certainty::Uncertain { .. }),
        evaluations: scan.between_evaluations + scan.descent_evaluations + scan.certify_evaluations,
    })
}

/// How many times the variation a cell's neighbourhood shows across half the
/// cell its own nodes have to clear before the cell is taken to hold no sign
/// change. The slope is ESTIMATED from node differences, which is exactly what
/// underestimates a feature between nodes, so the margin is doubled; the
/// island family in this module's tests measures what that buys.
const REFINE_SAFETY: f64 = 2.0;
/// How many times a risky cell may be halved: 2⁷ = 128 subdivisions per node
/// spacing.
const REFINE_DEPTH: usize = 7;

/// What [`refine_between_nodes`] found.
#[derive(Clone, Debug, Default)]
pub(crate) struct BetweenNodes {
    /// Refined samples inside the region that read at or below the level, in
    /// cells whose four nodes all read above it.
    pub(crate) collapsed: usize,
    /// Refined samples inside the region that read above the level, in cells
    /// whose four nodes all read at or below it.
    pub(crate) regular: usize,
    /// A `(u, v)` pair straddling the level for every such find — one refined
    /// sample and the corner of its cell with the other sign — so the fold
    /// TRACE can seed on a branch the grid never bracketed.
    pub(crate) brackets: Vec<([f64; 2], [f64; 2])>,
    /// The worst refined sample inside the region.
    pub(crate) worst: Option<FoldSample>,
    pub(crate) cells: usize,
    pub(crate) evaluations: usize,
    /// Cells still risky at the depth limit: the bound was not cleared there,
    /// and nothing of the other sign was found either. Kept, not just counted,
    /// so the certainty step can take them further.
    pub(crate) unresolved_cells: Vec<RiskyCell>,
    /// An evaluation cap stopped the refinement with risky cells unprocessed
    /// (those are in `unresolved_cells` too). Never set by the scan's own
    /// refinement, which has no cap; only by the certainty step.
    pub(crate) capped: bool,
    /// Dips descended, what they cost, and whether the budget ran out first
    /// (see [`descend_dips`]).
    pub(crate) descents: usize,
    pub(crate) descent_evaluations: usize,
    pub(crate) descent_capped: bool,
}

impl BetweenNodes {
    /// Cells still risky at the depth limit.
    pub(crate) fn unresolved(&self) -> usize {
        self.unresolved_cells.len()
    }
}

/// A grid cell, or a piece of one, whose four corners agree in sign but which
/// the slope bound could not clear — the unit the between-node refinement and
/// the certainty step work on.
#[derive(Clone, Debug)]
pub(crate) struct RiskyCell {
    pub(crate) u: [f64; 2],
    pub(crate) v: [f64; 2],
    /// Corner values in the order (u0,v0), (u1,v0), (u0,v1), (u1,v1).
    pub(crate) corners: [f64; 4],
    pub(crate) slopes: (f64, f64),
    pub(crate) depth: usize,
    pub(crate) risk: f64,
}
// Ordered so the heap pops the SMALLEST risk ratio — the riskiest cell — first.
impl PartialEq for RiskyCell {
    fn eq(&self, other: &Self) -> bool {
        self.risk.total_cmp(&other.risk).is_eq()
    }
}
impl Eq for RiskyCell {}
impl PartialOrd for RiskyCell {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for RiskyCell {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.risk.total_cmp(&self.risk)
    }
}

/// The risk of a cell with these corner values, spans and slopes: the ratio
/// of its smallest `|f|` to the margin the slopes could cover over half the
/// cell, or `None` when the bound clears.
fn cell_risk(corners: &[f64; 4], du: f64, dv: f64, (lu, lv): (f64, f64)) -> Option<f64> {
    let margin = REFINE_SAFETY * (lu * du + lv * dv) / 2.0;
    let smallest = corners.iter().fold(f64::INFINITY, |m, c| m.min(c.abs()));
    (margin > 0.0 && smallest < margin).then(|| smallest / margin)
}

/// A local minimum of the node grid on the regular side is worth following
/// when its highest neighbour sits at least this far above it. The
/// spikes the slope bound cannot see (2026-09-30 sweep, SHELL_FACE budget:
/// 185 of 375 family members missed, every one with the bound cleared) move
/// their nearest node by 0.05 to 0.5; a fitted carrier's interpolation noise
/// moves nodes by 1e-6 and less, and a dip that shallow hides nothing a
/// descent could find.
const DESCENT_PROMINENCE: f64 = 1e-3;
/// Levels of the windowed descent: the window starts one node spacing wide and
/// halves each level, so the last one is `2⁻⁹` of the spacing — below the cell
/// refinement's own floor.
const DESCENT_LEVELS: usize = 9;
/// Probes either side of the centre per direction per level: a 9 × 9 window.
const DESCENT_PROBES: usize = 4;
/// Samples one scan may spend descending, most prominent dips first. At
/// 80 samples a level, 9 levels, that is about 200 descents; a region with more
/// prominent dips than that is reported capped, not regular.
const DESCENT_EVALUATIONS: usize = 150_000;

/// Follow every prominent DIP of the node grid to its minimum.
///
/// The cell refinement above judges a cell by the slope its neighbourhood
/// shows, and a spike narrower than the node spacing shows almost none: the
/// nodes round the family's missed islands read 0.49 to 0.91 with the level at
/// 1e-6, so no cell is risky and the island is missed with the bound cleared.
/// What those nodes DO show is a dip — a strict local minimum of the sampled
/// field, its nearest node reading lower than every neighbour — and the fold at
/// a spike's tip is the bottom of that dip. So every local minimum on the
/// regular side (ties allowed: a spike on a knot reads the same at the nodes
/// either side of it), with a neighbour at least [`DESCENT_PROMINENCE`] above
/// it, is descended: a window one node spacing wide is probed on a 9 × 9 lattice, the
/// centre moves to the least sample inside the region, and the window halves,
/// [`DESCENT_LEVELS`] times or until a sample at or below the level turns up.
/// A collapsed sample found this way counts as `collapsed` and brackets against
/// the node it descended from, so the fold trace seeds on it like any other.
///
/// Bounded: [`DESCENT_EVALUATIONS`] per scan, most prominent dips first; when
/// it runs out `descent_capped` is set and the caller's certainty step reads
/// the region as undecided rather than regular. Measured, not proved: a spike
/// so far between nodes that it moves none of them by the prominence floor is
/// still invisible, and the family sweep in this module's tests says where the
/// floor sits.
#[allow(clippy::too_many_arguments)]
fn descend_dips(
    surface: &NurbsSurface,
    region: &TrimRegion,
    samples_u: &[f64],
    samples_v: &[f64],
    values: &[Vec<Option<f64>>],
    displacements: &[f64],
    level: f64,
    found: &mut BetweenNodes,
) {
    let (nu, nv) = (samples_u.len(), samples_v.len());
    if nu < 2 || nv < 2 {
        return;
    }
    // Local minima on the regular side — no neighbour reads lower; ties are
    // allowed, because a spike sitting on a knot exactly halfway between nodes
    // reads the SAME at the two or four nodes round it — with their prominence,
    // how far the highest neighbour sits above them.
    let mut dips: Vec<(f64, usize, usize)> = Vec::new();
    for iu in 0..nu {
        for iv in 0..nv {
            let Some(own) = values[iu][iv] else { continue };
            if own <= 0.0 || !region.contains(samples_u[iu], samples_v[iv]) {
                continue;
            }
            let mut highest_neighbour = f64::NEG_INFINITY;
            let mut minimum = true;
            for du in -1i64..=1 {
                for dv in -1i64..=1 {
                    if du == 0 && dv == 0 {
                        continue;
                    }
                    let (ju, jv) = (iu as i64 + du, iv as i64 + dv);
                    if ju < 0 || jv < 0 || ju >= nu as i64 || jv >= nv as i64 {
                        continue;
                    }
                    if let Some(other) = values[ju as usize][jv as usize] {
                        if other < own {
                            minimum = false;
                        }
                        highest_neighbour = highest_neighbour.max(other);
                    }
                }
            }
            if minimum && highest_neighbour.is_finite() && highest_neighbour - own >= DESCENT_PROMINENCE {
                dips.push((highest_neighbour - own, iu, iv));
            }
        }
    }
    dips.sort_by(|a, b| b.0.total_cmp(&a.0));
    let spacing = |samples: &[f64], index: usize| -> f64 {
        let before = if index > 0 { samples[index] - samples[index - 1] } else { 0.0 };
        let after = if index + 1 < samples.len() { samples[index + 1] - samples[index] } else { 0.0 };
        before.max(after)
    };
    for (_, iu, iv) in dips {
        if found.descent_evaluations + (2 * DESCENT_PROBES + 1).pow(2) > DESCENT_EVALUATIONS {
            found.descent_capped = true;
            break;
        }
        found.descents += 1;
        let node = [samples_u[iu], samples_v[iv]];
        let (mut centre_u, mut centre_v) = (node[0], node[1]);
        let (mut radius_u, mut radius_v) = (spacing(samples_u, iu), spacing(samples_v, iv));
        let mut least = values[iu][iv].unwrap_or(f64::INFINITY);
        'levels: for _ in 0..DESCENT_LEVELS {
            let mut best: Option<FoldSample> = None;
            for pu in 0..=2 * DESCENT_PROBES {
                let u = centre_u + radius_u * (pu as f64 / DESCENT_PROBES as f64 - 1.0);
                for pv in 0..=2 * DESCENT_PROBES {
                    let v = centre_v + radius_v * (pv as f64 / DESCENT_PROBES as f64 - 1.0);
                    if (pu == DESCENT_PROBES && pv == DESCENT_PROBES) || !region.contains(u, v) {
                        continue;
                    }
                    found.descent_evaluations += 1;
                    let Some(sample) = fold_sample_at(surface, u, v, displacements) else {
                        continue;
                    };
                    if found.worst.is_none_or(|worst| sample.factor < worst.factor) {
                        found.worst = Some(sample);
                    }
                    let f = sample.factor - level;
                    if f <= 0.0 {
                        // The bottom of the dip is below the level: a fold the
                        // census's nodes did not show, bracketed against the
                        // node the descent started from.
                        found.collapsed += 1;
                        found.brackets.push(([u, v], node));
                        break 'levels;
                    }
                    if best.is_none_or(|current| sample.factor < current.factor) {
                        best = Some(sample);
                    }
                }
            }
            if let Some(sample) = best.filter(|sample| sample.factor - level < least) {
                least = sample.factor - level;
                centre_u = sample.u;
                centre_v = sample.v;
            }
            radius_u *= 0.5;
            radius_v *= 0.5;
        }
    }
}

/// Look BETWEEN the scan's nodes for a sign change the nodes cannot show.
///
/// A cell whose four corners agree can still hold an island of the other sign
/// — a shallow fold narrower than the node spacing, which is how a carved
/// piece was once kept with a folded island inside it. A cell is RISKY when the
/// smallest `|f|` at its corners is below [`REFINE_SAFETY`] times the
/// variation its 3×3 neighbourhood of cells shows over half the cell,
/// `(L_u·Δu + L_v·Δv)/2` with `L` the largest node-difference slope per
/// direction. Risky cells are halved, most risky first, recomputing the bound
/// on each child with its own corners (never below the parent's slopes), until
/// the bound clears, a sample of the other sign turns up, or the depth reaches
/// [`REFINE_DEPTH`]. There is no evaluation budget: at most
/// `5·(4^REFINE_DEPTH − 1)/3` samples per risky cell, so it always finishes.
///
/// Near a real fold locus every cell is risky and the refinement costs up to
/// `REFINE_DEPTH` levels along it; away from one `|f|` is large against the
/// neighbourhood's variation and nothing is read.
///
/// `values[iu][iv]` is `factor − level` at `(samples_u[iu], samples_v[iv])`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn refine_between_nodes(
    surface: &NurbsSurface,
    region: &TrimRegion,
    samples_u: &[f64],
    samples_v: &[f64],
    values: &[Vec<Option<f64>>],
    displacements: &[f64],
    level: f64,
) -> BetweenNodes {
    let mut found = BetweenNodes::default();
    let (nu, nv) = (samples_u.len(), samples_v.len());
    if nu < 2 || nv < 2 {
        return found;
    }
    let value = |iu: usize, iv: usize| values[iu][iv];
    // Largest node-difference slope per direction over the 3×3 cells around
    // cell (iu, iv).
    let slopes = |iu: usize, iv: usize| -> (f64, f64) {
        let (mut lu, mut lv) = (0.0f64, 0.0f64);
        let u_lo = iu.saturating_sub(1);
        let u_hi = (iu + 2).min(nu - 1);
        let v_lo = iv.saturating_sub(1);
        let v_hi = (iv + 2).min(nv - 1);
        for a in u_lo..=u_hi {
            for b in v_lo..=v_hi {
                if a < u_hi {
                    if let (Some(p), Some(q)) = (value(a, b), value(a + 1, b)) {
                        lu = lu.max((q - p).abs() / (samples_u[a + 1] - samples_u[a]));
                    }
                }
                if b < v_hi {
                    if let (Some(p), Some(q)) = (value(a, b), value(a, b + 1)) {
                        lv = lv.max((q - p).abs() / (samples_v[b + 1] - samples_v[b]));
                    }
                }
            }
        }
        (lu, lv)
    };

    let mut work = std::collections::BinaryHeap::<RiskyCell>::new();
    for iu in 0..nu - 1 {
        for iv in 0..nv - 1 {
            let (Some(a), Some(b), Some(c), Some(d)) =
                (value(iu, iv), value(iu + 1, iv), value(iu, iv + 1), value(iu + 1, iv + 1))
            else {
                continue;
            };
            let corners = [a, b, c, d];
            let negative = corners.iter().filter(|x| **x <= 0.0).count();
            if negative != 0 && negative != 4 {
                continue; // the grid already brackets this cell
            }
            let (u0, u1) = (samples_u[iu], samples_u[iu + 1]);
            let (v0, v1) = (samples_v[iv], samples_v[iv + 1]);
            let touches = region.contains(u0, v0)
                || region.contains(u1, v0)
                || region.contains(u0, v1)
                || region.contains(u1, v1)
                || region.contains(0.5 * (u0 + u1), 0.5 * (v0 + v1));
            if !touches {
                continue;
            }
            let cell_slopes = slopes(iu, iv);
            if let Some(risk) = cell_risk(&corners, u1 - u0, v1 - v0, cell_slopes) {
                work.push(RiskyCell {
                    u: [u0, u1],
                    v: [v0, v1],
                    corners,
                    slopes: cell_slopes,
                    depth: 0,
                    risk,
                });
            }
        }
    }
    found.cells = work.len();
    refine_cells(surface, region, displacements, level, work, REFINE_DEPTH, None, &mut found);
    descend_dips(surface, region, samples_u, samples_v, values, displacements, level, &mut found);
    found
}

/// Halve `work`'s cells, riskiest first, until each clears its bound, holds a
/// sample of the other sign, or reaches `max_depth`; with `cap`, stop once
/// that many samples have been read and leave the rest in `unresolved_cells`
/// with `capped` set. Everything found goes into `found`.
#[allow(clippy::too_many_arguments)]
fn refine_cells(
    surface: &NurbsSurface,
    region: &TrimRegion,
    displacements: &[f64],
    level: f64,
    mut work: std::collections::BinaryHeap<RiskyCell>,
    max_depth: usize,
    cap: Option<usize>,
    found: &mut BetweenNodes,
) {
    // The scan's own refinement runs with no evaluation budget: the quadtree
    // below one risky cell has at most (4^REFINE_DEPTH − 1)/3 cells to process
    // at five samples each, so it always finishes, and a cap could only ever
    // stop it early — which is the island escape again with a different
    // trigger. The certainty step's second pass over what that left is the one
    // with a cap, and running out is reported, never read as regular.
    while let Some(cell) = work.pop() {
        if cap.is_some_and(|cap| found.evaluations + 5 > cap) {
            found.capped = true;
            found.unresolved_cells.push(cell);
            found.unresolved_cells.extend(work.into_iter());
            break;
        }
        let collapsed_side = cell.corners[0] <= 0.0;
        let (um, vm) = (0.5 * (cell.u[0] + cell.u[1]), 0.5 * (cell.v[0] + cell.v[1]));
        let points = [
            [um, vm],
            [um, cell.v[0]],
            [cell.u[0], vm],
            [cell.u[1], vm],
            [um, cell.v[1]],
        ];
        let mut read = [None; 5];
        let mut flipped = false;
        for (index, point) in points.iter().enumerate() {
            found.evaluations += 1;
            let Some(sample) = fold_sample_at(surface, point[0], point[1], displacements) else {
                continue;
            };
            let f = sample.factor - level;
            read[index] = Some(f);
            let inside = region.contains(point[0], point[1]);
            if inside && found.worst.is_none_or(|worst| sample.factor < worst.factor) {
                found.worst = Some(sample);
            }
            if (f <= 0.0) != collapsed_side {
                // A sign change inside this cell whether or not the sample is in
                // the region: the trace needs the bracket either way (its branch
                // may enter the region), and a child cell built on this sample
                // would no longer have corners of one sign. Only a sample in
                // the region counts against the region's census.
                flipped = true;
                if inside && f <= 0.0 {
                    found.collapsed += 1;
                } else if inside {
                    found.regular += 1;
                }
                // Straddle the level: this sample and the corner nearest it.
                let corner = [
                    [cell.u[0], cell.v[0]],
                    [cell.u[1], cell.v[0]],
                    [cell.u[0], cell.v[1]],
                    [cell.u[1], cell.v[1]],
                ]
                .into_iter()
                .min_by(|p, q| {
                    let dp = (p[0] - point[0]).hypot(p[1] - point[1]);
                    let dq = (q[0] - point[0]).hypot(q[1] - point[1]);
                    dp.partial_cmp(&dq).unwrap_or(std::cmp::Ordering::Equal)
                })
                .expect("four corners");
                found.brackets.push((*point, corner));
            }
        }
        if flipped {
            continue;
        }
        let [Some(centre), Some(bottom), Some(left), Some(right), Some(top)] = read else {
            continue;
        };
        let [c00, c10, c01, c11] = cell.corners;
        let children = [
            ([cell.u[0], um], [cell.v[0], vm], [c00, bottom, left, centre]),
            ([um, cell.u[1]], [cell.v[0], vm], [bottom, c10, centre, right]),
            ([cell.u[0], um], [vm, cell.v[1]], [left, centre, c01, top]),
            ([um, cell.u[1]], [vm, cell.v[1]], [centre, right, top, c11]),
        ];
        for (u, v, corners) in children {
            let (du, dv) = (u[1] - u[0], v[1] - v[0]);
            let own = (
                ((corners[1] - corners[0]).abs().max((corners[3] - corners[2]).abs())) / du,
                ((corners[2] - corners[0]).abs().max((corners[3] - corners[1]).abs())) / dv,
            );
            let child_slopes = (cell.slopes.0.max(own.0), cell.slopes.1.max(own.1));
            if let Some(risk) = cell_risk(&corners, du, dv, child_slopes) {
                let child = RiskyCell {
                    u,
                    v,
                    corners,
                    slopes: child_slopes,
                    depth: cell.depth + 1,
                    risk,
                };
                if child.depth >= max_depth {
                    found.unresolved_cells.push(child);
                    continue;
                }
                work.push(child);
            }
        }
    }
}

/// Bisect toward the worst node so the refusal names where the fold IS, not the
/// nearest node to it.  Census counts are untouched: refinement hunts a minimum
/// and would bias any population it joined.
fn refine_worst(
    surface: &NurbsSurface,
    region: &TrimRegion,
    displacements: &[f64],
    scan: &mut RegularityScan,
    bounds: [f64; 4],
) {
    const LEVELS: usize = 4;
    const PROBES: usize = 4;
    let Some(start) = scan.worst else { return };
    let mut radius_u = (bounds[1] - bounds[0]) / 16.0;
    let mut radius_v = (bounds[3] - bounds[2]) / 16.0;
    let (mut centre_u, mut centre_v) = (start.u, start.v);
    for _ in 0..LEVELS {
        let mut best: Option<FoldSample> = None;
        for iu in 0..=2 * PROBES {
            let u = (centre_u + radius_u * (iu as f64 / PROBES as f64 - 1.0))
                .clamp(bounds[0], bounds[1]);
            for iv in 0..=2 * PROBES {
                let v = (centre_v + radius_v * (iv as f64 / PROBES as f64 - 1.0))
                    .clamp(bounds[2], bounds[3]);
                if !region.contains(u, v) {
                    continue;
                }
                if let Some(sample) = fold_sample_at(surface, u, v, displacements) {
                    if best.is_none_or(|current| sample.factor < current.factor) {
                        best = Some(sample);
                    }
                }
            }
        }
        if let Some(sample) = best {
            scan.keep_worst(sample);
            centre_u = sample.u;
            centre_v = sample.v;
        }
        radius_u *= 0.5;
        radius_v *= 0.5;
    }
}

