//! Linking (lane C4, row 3): the operation-level entry point every strategy
//! uses to turn its cut passes into a complete, safe motion stream.
//!
//! Strategies hand over ordered passes (polylines of certified cut points).
//! The linker adds the approach, the entry (plunge or ramp, then lead-in),
//! the links between passes (keep-down feed link, minimum retract, retract
//! height or full clearance, by the operation's retract policy), the lead-out
//! and the final retract. Every non-rapid link chord is proven against the
//! complete protected mesh with the drop cutter's conservative segment bound:
//! a chord is safe when its lowest Z is at or above the constant height that
//! clears every triangle under the chord's dilated footprint. An unproven
//! link falls back to the next rung of the ladder and finally to a full
//! clearance retract, and the operation report says so.
//!
//! Rapids are pure-axis moves (XY at clearance, Z straight down or up) so a
//! controller that doglegs G0 cannot leave the proven path. Horizontal
//! traverses below clearance are `Link` motions at a high feed; posts may map
//! them to G0 when the controller interpolates rapids linearly.

use crate::drop::path::{Dilated, Sampling};
use crate::drop::DropCutter;
use crate::model::{
    Diagnostic, Feed, FeedsSpeeds, Heights, LeadKind, LinkingParams, Motion, MotionKind, OperationReport, Point3,
    RampKind, RetractPolicy, Toolpath,
};
use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};
use std::f64::consts::{PI, TAU};

mod stock;
use stock::StockMap;

// The stock before the operation being generated (lane C7's envelope from
// `GenerateContext::pre_op_stock`), scoped to one `dispatch` call. Every
// linker site reads it into `LinkContext::stock` (`with_current_pre_op_stock`),
// so the dozen strategy signatures between dispatch and the linker do not
// carry it. The scope holds a BORROW of the caller's envelope, not a copy (an
// envelope holds one top per stock column).
//
// Thread-local by design: CAM generation runs on the calling thread. The
// crate has no worker threads (`link::tests::cam_generation_never_leaves_the_calling_thread`
// fails if threading appears in `BREP_cam/src`), and `connect` asserts in
// debug builds that a linker inside an active scope used it. A generator that
// ever links on another thread must carry the stock into that closure
// explicitly. `connect` refuses a context built without the stock while a
// scope with one is active on its thread.
thread_local! {
    static PRE_OP_STOCK: std::cell::Cell<*const crate::StockEnvelope> = const { std::cell::Cell::new(std::ptr::null()) };
}

/// Run `f` with `stock` as the pre-operation stock of the operation being
/// generated; the previous value is restored afterwards (also on unwind).
pub fn with_pre_op_stock<R>(stock: Option<&crate::StockEnvelope>, f: impl FnOnce() -> R) -> R {
    struct Restore(*const crate::StockEnvelope);
    impl Drop for Restore {
        fn drop(&mut self) {
            PRE_OP_STOCK.with(|c| c.set(self.0));
        }
    }
    let previous = PRE_OP_STOCK.with(|c| c.replace(stock.map_or(std::ptr::null(), |s| s as *const _)));
    let _restore = Restore(previous);
    f()
}

/// Call `f` with the pre-operation stock of the operation being generated
/// (`None` outside a `with_pre_op_stock` scope or when the caller had none).
/// The reference cannot outlive `f`.
pub fn with_current_pre_op_stock<R>(f: impl for<'s> FnOnce(Option<&'s crate::StockEnvelope>) -> R) -> R {
    let p = PRE_OP_STOCK.with(|c| c.get());
    // SAFETY: the pointer is non-null only while a `with_pre_op_stock` frame
    // on THIS thread's stack holds the `&StockEnvelope` it came from (the
    // guard restores the previous value when that frame ends, also on
    // unwind), and `f` runs synchronously inside that frame; the
    // higher-ranked lifetime keeps the reference from escaping `f`.
    let stock = if p.is_null() { None } else { Some(unsafe { &*p }) };
    f(stock)
}

/// Whether a scope with a stock is active on this thread.
fn scope_has_pre_op_stock() -> bool {
    PRE_OP_STOCK.with(|c| !c.get().is_null())
}

/// One cutting pass: a polyline of certified cut points.
#[derive(Clone, Debug, PartialEq)]
pub struct Pass {
    pub points: Vec<Point3>,
    /// Source path identity for preview grouping.
    pub path_id: Option<u32>,
}

impl Pass {
    pub fn new(points: Vec<Point3>, path_id: Option<u32>) -> Self {
        Self { points, path_id }
    }

    pub fn start(&self) -> Point3 {
        self.points[0]
    }

    pub fn end(&self) -> Point3 {
        self.points[self.points.len() - 1]
    }

    /// Unit XY tangent at the start (first chord with XY length).
    fn start_tangent(&self) -> Option<Point3> {
        self.points.windows(2).find_map(|w| xy_unit(w[1].sub(w[0])))
    }

    /// Unit XY tangent at the end (last chord with XY length).
    fn end_tangent(&self) -> Option<Point3> {
        self.points.windows(2).rev().find_map(|w| xy_unit(w[1].sub(w[0])))
    }
}

fn xy_unit(d: Point3) -> Option<Point3> {
    Point3::new(d.x, d.y, 0.0).normalized()
}

/// Which side of the pass a lead arc lies on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LeadSide {
    /// Left of the cut direction (climb milling of an outer profile keeps the
    /// material on the right).
    #[default]
    Left,
    Right,
}

/// Linker settings beyond the shared [`LinkingParams`] (whose `safe_distance`
/// is the clearance kept above material by minimum retracts, ramp tops and
/// feed heights).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkOptions {
    /// Feed of horizontal traverses below clearance (`Link` motions).
    pub high_feed: f64,
    /// Radial stock allowance to keep off the surface.
    pub radial_allowance: f64,
    /// Axial stock allowance to keep off the surface.
    pub axial_allowance: f64,
    /// Chord tolerance for discretized leads and ramps.
    pub chord_tolerance: f64,
    /// Preferred side of lead arcs; the other side is tried when unproven.
    pub lead_side: LeadSide,
    /// Reorder passes by nearest neighbour (strategies that need their order
    /// leave this off).
    pub reorder: bool,
    /// Passes may be reversed when reordering.
    pub allow_reverse: bool,
    /// Ramp length along the pass for profile and zig-zag ramps (`None`:
    /// twice the helix diameter, else four cutter radii).
    pub ramp_length: Option<f64>,
    /// Work budget: drop samples spent proving links.
    pub max_samples: u64,
    /// Top of the material this operation removes (roughing, any operation
    /// that starts from stock). When set, partial retracts, feed heights and
    /// keep-down links are proven against the part AND a conservative
    /// in-process stock: material up to this top wherever this operation's
    /// earlier cutting moves have not swept the whole cell (`link::stock`).
    /// `None`: only the part (and allowances) remains, as after roughing.
    pub material_top: Option<f64>,
}

impl Default for LinkOptions {
    fn default() -> Self {
        Self {
            high_feed: 5_000.0,
            radial_allowance: 0.0,
            axial_allowance: 0.0,
            chord_tolerance: 0.01,
            lead_side: LeadSide::Left,
            reorder: false,
            allow_reverse: true,
            ramp_length: None,
            max_samples: 1_000_000,
            material_top: None,
        }
    }
}

impl LinkOptions {
    pub fn validate(&self) -> Result<(), CamError> {
        let bad = |m: &str| Err(CamError::InvalidInput(format!("linking: {m}")));
        if !self.high_feed.is_finite() || self.high_feed <= 0.0 {
            return bad("high feed must be finite and positive");
        }
        if !self.radial_allowance.is_finite() || self.radial_allowance < 0.0 || !self.axial_allowance.is_finite() || self.axial_allowance < 0.0 {
            return bad("allowances must be finite and non-negative");
        }
        if !self.chord_tolerance.is_finite() || self.chord_tolerance <= 0.0 {
            return bad("chord tolerance must be finite and positive");
        }
        if let Some(l) = self.ramp_length {
            if !l.is_finite() || l <= 0.0 {
                return bad("ramp length must be finite and positive");
            }
        }
        if self.max_samples < 2 {
            return bad("sample budget too small");
        }
        if self.material_top.is_some_and(|t| !t.is_finite()) {
            return bad("material top must be finite");
        }
        Ok(())
    }
}

/// Everything the linker needs about one operation.
pub struct LinkContext<'a> {
    /// The complete protected mesh (targets and fixtures) in machine coordinates.
    pub mesh: &'a MeshSnapshot,
    pub cutter: &'a Cutter,
    /// Index built from `mesh`, when available.
    pub index: Option<&'a dyn TriangleIndex>,
    pub tolerance: Tolerance,
    pub heights: &'a Heights,
    pub feeds: &'a FeedsSpeeds,
    pub params: &'a LinkingParams,
    pub options: LinkOptions,
    /// The stock as it stands when this operation starts (lane C7's
    /// envelope: the initial stock or the replay of earlier operations), in
    /// link coordinates. It seeds the in-process stock map: a cell starts at
    /// the lower of `options.material_top` and the envelope's top over the
    /// cell's square, and at -inf where the envelope has no material; with no
    /// `material_top` the envelope's highest top bounds the cells. `None`:
    /// today's behaviour (`material_top` alone, or no stock at all).
    pub stock: Option<&'a crate::StockEnvelope>,
}

/// Linked motions plus the report the linker wrote (warnings, counts).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Linked {
    pub motions: Vec<Motion>,
    pub report: OperationReport,
}

/// Proves chords against the protected mesh and counts the work.
struct Prover<'a> {
    mesh: &'a MeshSnapshot,
    index: Option<&'a dyn TriangleIndex>,
    tolerance: Tolerance,
    dilated: Dilated<'a>,
    sampling: Sampling,
    axial: f64,
    /// In-process stock of this operation (`LinkOptions::material_top`).
    stock: std::cell::RefCell<Option<StockMap>>,
}

impl<'a> Prover<'a> {
    fn new(ctx: &LinkContext<'a>) -> Result<Self, CamError> {
        let dilated = Dilated { base: ctx.cutter, pad: ctx.options.radial_allowance };
        // Validates the cutter, the mesh and the index once.
        DropCutter::from_profile(ctx.mesh, &dilated, ctx.index, ctx.tolerance)?;
        let floor = ctx.mesh.bounds.min.z - ctx.cutter.total_height() - 1.0;
        let sampling = Sampling {
            sample_spacing: 1.0,
            min_sample_spacing: 1e-6,
            chord_error: ctx.options.chord_tolerance,
            max_samples: ctx.options.max_samples,
            floor_z: floor,
            ..Sampling::default()
        };
        sampling.validate()?;
        Ok(Self { mesh: ctx.mesh, index: ctx.index, tolerance: ctx.tolerance, dilated, sampling, axial: ctx.options.axial_allowance, stock: std::cell::RefCell::new(None) })
    }

    /// Highest uncut stock the tool footprint meets above `p` (no stock: -inf).
    fn stock_point(&self, p: Point3) -> f64 {
        self.stock.borrow().as_ref().map_or(f64::NEG_INFINITY, |s| s.point(p))
    }

    /// Highest uncut stock the tool footprint meets on the XY move `a`-`b`.
    fn stock_chord(&self, a: Point3, b: Point3) -> f64 {
        self.stock.borrow().as_ref().map_or(f64::NEG_INFINITY, |s| s.chord(a, b))
    }

    fn drop(&self) -> Result<DropCutter<'_>, CamError> {
        DropCutter::from_profile(self.mesh, &self.dilated, self.index, self.tolerance)
    }

    /// Lowest safe Z of the cutter reference point at `p`'s XY.
    fn point_bound(&self, p: Point3, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<f64, CamError> {
        if report.counts.samples >= self.sampling.max_samples {
            return Err(CamError::InvalidInput("linking sample work budget exhausted".into()));
        }
        let r = self.drop()?.point(p.x, p.y, self.sampling.floor_z, runner)?;
        report.counts.samples += 1;
        report.counts.candidates += r.candidate_count;
        report.counts.contacts += r.contact_count;
        Ok(r.point.z + self.axial)
    }

    /// Lowest constant Z at which the cutter can travel straight from `a` to
    /// `b` (XY) without touching protected material.
    fn chord_bound(&self, a: Point3, b: Point3, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<f64, CamError> {
        if a.distance_xy(b) <= 0.0 {
            return self.point_bound(a, report, runner);
        }
        Ok(self.drop()?.segment_bound(a, b, &self.sampling, report, runner)? + self.axial)
    }

    /// A chord from `a` to `b` is proven when its lowest point clears the
    /// chord bound (within numeric distance epsilon).
    fn chord_safe(&self, a: Point3, b: Point3, eps: f64, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<bool, CamError> {
        let bound = self.chord_bound(a, b, report, runner)?;
        Ok(a.z.min(b.z) + eps >= bound)
    }

    fn polyline_safe(&self, points: &[Point3], eps: f64, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<bool, CamError> {
        for w in points.windows(2) {
            if !self.chord_safe(w[0], w[1], eps, report, runner)? {
                return Ok(false);
            }
        }
        Ok(true)
    }
}

/// Pending motions: the linker writes into a scratch list and commits it only
/// when the whole operation is proven, so an error leaves no partial stream.
struct Emitter {
    motions: Vec<Motion>,
    path_id: Option<u32>,
}

impl Emitter {
    fn push(&mut self, mut m: Motion) {
        if m.path_id.is_none() {
            m.path_id = self.path_id;
        }
        self.motions.push(m);
    }
    fn at(&self) -> Option<Point3> {
        self.motions.iter().rev().find_map(|m| m.end())
    }
}

/// Link ordered passes into a complete motion stream for one operation.
///
/// The stream starts with a rapid at clearance (the caller's tool is assumed
/// at or above clearance) and ends with a retract to clearance. Empty passes
/// and passes with non-finite points are errors.
pub fn connect(passes: &[Pass], ctx: &LinkContext<'_>, runner: &mut Runner<'_>) -> Result<Linked, CamError> {
    connect_with(passes, ctx, &[], runner)
}

/// Material the in-process stock map must keep, whatever this operation's
/// cuts say: up to `z` within `reach` of the polyline `points`.
struct KeepOut {
    points: Vec<Point3>,
    reach: f64,
    z: f64,
}

fn connect_with(passes: &[Pass], ctx: &LinkContext<'_>, keep: &[KeepOut], runner: &mut Runner<'_>) -> Result<Linked, CamError> {
    // A linker inside an operation that has a pre-operation stock must use it:
    // a site that builds its context with `stock: None` would prove links
    // against the final part only, the unsafe case. Refused in every build.
    if ctx.stock.is_none() && scope_has_pre_op_stock() {
        return Err(CamError::InvalidInput("linking: LinkContext built without the active pre-operation stock (read it with link::with_current_pre_op_stock)".into()));
    }
    ctx.options.validate()?;
    ctx.params.validate().map_err(CamError::InvalidInput)?;
    ctx.heights.validate().map_err(CamError::InvalidInput)?;
    ctx.feeds.validate().map_err(CamError::InvalidInput)?;
    ctx.tolerance.validate()?;
    ctx.cutter.validate()?;
    if passes.is_empty() {
        return Err(CamError::InvalidInput("linking: no passes to link".into()));
    }
    for (i, p) in passes.iter().enumerate() {
        if p.points.is_empty() {
            return Err(CamError::InvalidInput(format!("linking: pass {i} is empty")));
        }
        if let Some(q) = p.points.iter().find(|q| !q.is_finite()) {
            return Err(CamError::NonFinite { what: format!("pass {i} point {q:?}") });
        }
    }
    if ctx.mesh.is_empty() {
        return Err(CamError::EmptyTarget { operation_id: "linking".into() });
    }
    let heights = ctx.heights;
    // Chord proofs accept the conservative segment bound within the chord
    // tolerance, exactly as the drop cutter certifies cut chords: on a slope
    // the dilated bound exceeds a certified end point by the dilation times
    // the slope, never by more than the chord tolerance for the chords used.
    let eps = ctx.options.chord_tolerance.max(ctx.tolerance.distance_eps);
    let top = ctx.mesh.bounds.max.z + ctx.options.axial_allowance;
    if heights.clearance < top + eps {
        return Err(CamError::InvalidInput("linking: clearance height must be above the full protected mesh and allowance".into()));
    }
    if passes.iter().flat_map(|p| &p.points).any(|p| p.z >= heights.clearance) {
        return Err(CamError::InvalidInput("linking: a pass reaches the clearance height".into()));
    }
    let prover = Prover::new(ctx)?;
    if ctx.options.material_top.is_some() || ctx.stock.is_some() || !keep.is_empty() {
        let extent = passes.iter().flat_map(|p| p.points.iter().copied()).chain(keep.iter().flat_map(|k| k.points.iter().copied()));
        let mut map = StockMap::new(extent, ctx.cutter, ctx.options.material_top, ctx.stock, 4_000_000);
        for k in keep {
            map.raise(&k.points, k.reach, k.z);
        }
        *prover.stock.borrow_mut() = Some(map);
    }
    let mut swept = 0usize;
    let mut report = OperationReport::default();
    report.counts.triangles = ctx.mesh.triangle_count() as u64;
    let mut em = Emitter { motions: Vec::new(), path_id: None };

    let order: Vec<(usize, bool)> = if ctx.options.reorder {
        let start = Point3::new(passes[0].start().x, passes[0].start().y, heights.clearance);
        order_passes(passes, start, ctx.options.allow_reverse)
    } else {
        (0..passes.len()).map(|i| (i, false)).collect()
    };
    runner.stage("link", "Link passes", 0, order.len() as u64)?;
    let mut previous: Option<Pass> = None;
    for (n, &(index, reversed)) in order.iter().enumerate() {
        let mut pass = passes[index].clone();
        if reversed {
            pass.points.reverse();
        }
        em.path_id = pass.path_id;
        // Entry geometry (lead-in) decides where the descent lands.
        let lead_in = plan_lead(&pass, true, &ctx.params.lead_in, ctx, &prover, eps, &mut report, runner)?;
        let entry = lead_in.as_ref().and_then(|l| l.first().copied()).unwrap_or(pass.start());
        match previous.take() {
            None => {
                // First pass: the tool is at clearance.
                em.push(Motion::rapid(Point3::new(entry.x, entry.y, heights.clearance)));
                descend(&pass, entry, heights.clearance, ctx, &prover, eps, &mut em, &mut report, runner)?;
            }
            Some(prev) => {
                // Everything this operation cut so far is gone from the stock.
                if let Some(map) = prover.stock.borrow_mut().as_mut() {
                    let from = em.motions[..swept].iter().rev().find_map(|m| m.end());
                    map.sweep(from, &em.motions[swept..]);
                }
                swept = em.motions.len();
                let exit = em.at().unwrap_or(prev.end());
                link_between(&prev, exit, &pass, entry, ctx, &prover, eps, &mut em, &mut report, runner)?;
            }
        }
        if let Some(lead) = lead_in {
            let feed = ctx.feeds.lead_feed.unwrap_or(ctx.feeds.cutting_feed);
            for p in lead.iter().skip(1) {
                em.push(Motion::new(MotionKind::LeadIn { to: *p }, Feed::Rate(feed)));
            }
        }
        for p in pass.points.iter().skip(1) {
            em.push(Motion::cut(*p, ctx.feeds.cutting_feed));
        }
        if let Some(lead) = plan_lead(&pass, false, &ctx.params.lead_out, ctx, &prover, eps, &mut report, runner)? {
            let feed = ctx.feeds.lead_feed.unwrap_or(ctx.feeds.cutting_feed);
            for p in lead.iter().skip(1) {
                em.push(Motion::new(MotionKind::LeadOut { to: *p }, Feed::Rate(feed)));
            }
        }
        report.counts.passes += 1;
        previous = Some(pass);
        runner.work(1, "link", n as u64 + 1, order.len() as u64)?;
    }
    let last = em.at().expect("a pass was emitted");
    em.push(Motion::retract(Point3::new(last.x, last.y, heights.clearance), retract_feed(ctx)));
    report.counts.motions = em.motions.len() as u64;
    report.finish();
    Ok(Linked { motions: em.motions, report })
}

fn retract_feed(ctx: &LinkContext<'_>) -> Feed {
    ctx.feeds.retract_feed.map_or(Feed::RAPID, Feed::Rate)
}

/// The height at which rapids turn into feed on the way down to `entry`:
/// the configured feed height, raised to clear material under the entry.
fn feed_height(entry: Point3, from_z: f64, ctx: &LinkContext<'_>, prover: &Prover<'_>, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<f64, CamError> {
    // Rapids stop above the part and above any stock still uncut there.
    let bound = prover.point_bound(entry, report, runner)?.max(prover.stock_point(entry));
    let z = ctx.heights.feed.max(bound + ctx.params.safe_distance).max(entry.z);
    Ok(z.min(from_z))
}

/// Descend from `from_z` straight above `entry` down to `entry`: a rapid to
/// the feed height, then the ramp (when proven) or a plunge.
///
/// The tool is at `entry` exactly when this returns: `eps` is the proof
/// tolerance of the chord certificates, never a threshold below which a
/// coordinate change goes unemitted (the next cut starts where the strategy
/// said it does, not within `eps` of it).
#[allow(clippy::too_many_arguments)]
fn descend(pass: &Pass, entry: Point3, from_z: f64, ctx: &LinkContext<'_>, prover: &Prover<'_>, eps: f64, em: &mut Emitter, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<(), CamError> {
    let zf = feed_height(entry, from_z, ctx, prover, report, runner)?;
    if zf < from_z {
        em.push(Motion::rapid(Point3::new(entry.x, entry.y, zf)));
    }
    if zf == entry.z {
        return Ok(());
    }
    if zf > entry.z {
        if let Some(ramp) = plan_ramp(pass, entry, zf, ctx, prover, eps, report, runner)? {
            for m in ramp {
                em.push(m);
            }
            return Ok(());
        }
    }
    em.push(Motion::plunge(entry, ctx.feeds.plunge_feed));
    Ok(())
}

/// Link from the end of `prev` (tool at `exit`) to `entry` of `next` by the
/// retract-policy ladder. Every rung ends with the tool at `entry` exactly;
/// a move is left out only when it would not change the position at all.
#[allow(clippy::too_many_arguments)]
fn link_between(prev: &Pass, exit: Point3, next: &Pass, entry: Point3, ctx: &LinkContext<'_>, prover: &Prover<'_>, eps: f64, em: &mut Emitter, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<(), CamError> {
    let _ = prev;
    let heights = ctx.heights;
    let params = ctx.params;
    // Rung 1: keep the tool down, following the surface.
    if params.keep_down_distance > 0.0 && exit.distance_xy(entry) <= params.keep_down_distance {
        if let Some(path) = keep_down_path(exit, entry, ctx, prover, eps, report, runner)? {
            for p in path {
                em.push(Motion::link(p, ctx.feeds.cutting_feed));
            }
            return Ok(());
        }
        report.warn(Diagnostic::new("link-keep-down-unproven", "A keep-down link could not stay within the safe distance of the surface and retracts instead").at(exit));
    }
    // Rung 2: a partial retract (minimum or retract height), when proven.
    let vertical = exit.distance_xy(entry) == 0.0;
    let partial = match params.retract_policy {
        RetractPolicy::FullRetract => None,
        // Never below either pass end: a link runs level with or above them.
        RetractPolicy::RetractHeight => Some(heights.retract.max(exit.z).max(entry.z)),
        RetractPolicy::MinimumRetract => {
            // Above the part and above the stock this operation has not cut
            // yet. A purely vertical transition (same XY) stays in the tool's
            // own swept column, so the uncut rim beside it does not count.
            let stock = if vertical { f64::NEG_INFINITY } else { prover.stock_chord(exit, entry) };
            let bound = prover.chord_bound(exit, entry, report, runner)?.max(stock);
            Some((bound + ctx.params.safe_distance).max(exit.z).max(entry.z))
        }
    };
    if let Some(z) = partial {
        if z < heights.clearance - eps {
            let a = Point3::new(exit.x, exit.y, z);
            let b = Point3::new(entry.x, entry.y, z);
            let proven = (vertical || prover.stock_chord(a, b) <= z) && prover.chord_safe(a, b, eps, report, runner)?;
            if proven {
                if z > exit.z {
                    em.push(Motion::retract(a, retract_feed(ctx)));
                }
                if a.distance_xy(b) > 0.0 {
                    em.push(Motion::link(b, ctx.options.high_feed));
                }
                descend(next, entry, z, ctx, prover, eps, em, report, runner)?;
                return Ok(());
            }
            report.warn(Diagnostic::new("link-fallback-clearance", format!("A retract to Z={z} could not be proven clear of material; the link uses the clearance height")).at(exit));
        }
    }
    // Rung 3: full clearance, pure-axis rapids.
    em.push(Motion::retract(Point3::new(exit.x, exit.y, heights.clearance), retract_feed(ctx)));
    if exit.distance_xy(entry) > 0.0 {
        em.push(Motion::rapid(Point3::new(entry.x, entry.y, heights.clearance)));
    }
    descend(next, entry, heights.clearance, ctx, prover, eps, em, report, runner)
}

/// A stay-down link from `exit` to `entry` that follows the surface: the
/// chord is sampled at half-radius steps, every sample dropped onto the
/// protected mesh, and every sub-chord lifted to its segment bound, so the
/// result is safe by construction. `None` when the path would have to climb
/// more than the safe distance above the pass ends (a retract is cheaper).
fn keep_down_path(exit: Point3, entry: Point3, ctx: &LinkContext<'_>, prover: &Prover<'_>, eps: f64, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<Option<Vec<Point3>>, CamError> {
    // A keep-down link never runs through stock this operation has not cut.
    if prover.stock_chord(exit, entry) > exit.z.min(entry.z) {
        return Ok(None);
    }
    let length = exit.distance_xy(entry);
    if length <= eps {
        // Vertical or coincident: proven when the end clears the point bound.
        let bound = prover.point_bound(entry, report, runner)?;
        return Ok((exit.z.min(entry.z) + eps >= bound).then(|| vec![entry]));
    }
    let step = (ctx.cutter.radius() * 0.5).max(eps * 10.0);
    let n = ((length / step).ceil() as usize).clamp(1, 10_000);
    let floor = exit.z.min(entry.z);
    // (point, fixed): the pass ends are certified cut points and never move.
    let mut pts: Vec<(Point3, bool)> = Vec::with_capacity(n + 1);
    pts.push((exit, true));
    for i in 1..n {
        let q = exit.lerp(entry, i as f64 / n as f64);
        let z = prover.point_bound(q, report, runner)?;
        pts.push((Point3::new(q.x, q.y, z.max(floor)), false));
    }
    pts.push((entry, true));
    // Certify chord by chord. Raising a free point never breaks a chord
    // already proven (its lowest point only rises), so failing chords lift
    // their free ends to the bound; a chord failing at a fixed end is split
    // at its midpoint until it passes or gets too short to matter.
    let min_len = (eps * 4.0).max(1e-6);
    let mut i = 0usize;
    let mut splits = 0usize;
    while i + 1 < pts.len() {
        let (a, a_fixed) = pts[i];
        let (b, b_fixed) = pts[i + 1];
        let bound = prover.chord_bound(a, b, report, runner)?;
        if a.z.min(b.z) + eps >= bound {
            i += 1;
            continue;
        }
        let mut lifted = false;
        if !a_fixed && a.z < bound {
            pts[i].0.z = bound;
            lifted = true;
        }
        if !b_fixed && b.z < bound {
            pts[i + 1].0.z = bound;
            lifted = true;
        }
        if lifted {
            continue;
        }
        if a.distance_xy(b) <= min_len || splits >= 10_000 {
            return Ok(None);
        }
        let m = a.lerp(b, 0.5);
        let z = prover.point_bound(m, report, runner)?;
        pts.insert(i + 1, (Point3::new(m.x, m.y, z.max(floor)), false));
        splits += 1;
    }
    if pts.iter().any(|(p, _)| p.z > floor + ctx.params.safe_distance + eps) {
        return Ok(None);
    }
    let pts: Vec<Point3> = pts.into_iter().map(|(p, _)| p).collect();
    // Drop interior points that are collinear within the chord tolerance.
    let filtered = crate::filter::filter_points(&pts, ctx.options.chord_tolerance, &[])?;
    Ok(Some(filtered.points.into_iter().skip(1).collect()))
}

/// Plan a lead: the polyline (or arc chords) from the lead's far end to the
/// pass start (lead-in) or from the pass end outwards (lead-out), proven
/// against the mesh. `None` when no lead is configured or none is proven.
#[allow(clippy::too_many_arguments)]
fn plan_lead(pass: &Pass, lead_in: bool, lead: &crate::model::Lead, ctx: &LinkContext<'_>, prover: &Prover<'_>, eps: f64, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<Option<Vec<Point3>>, CamError> {
    if lead.kind == LeadKind::None {
        return Ok(None);
    }
    let (anchor, tangent) = if lead_in { (pass.start(), pass.start_tangent()) } else { (pass.end(), pass.end_tangent()) };
    let Some(t) = tangent else {
        report.warn(Diagnostic::new("lead-no-tangent", "The pass has no horizontal direction for a lead").at(anchor));
        return Ok(None);
    };
    let name = if lead_in { "lead-in" } else { "lead-out" };
    let sides = match ctx.options.lead_side {
        LeadSide::Left => [1.0, -1.0],
        LeadSide::Right => [-1.0, 1.0],
    };
    let mut candidates: Vec<Vec<Point3>> = Vec::new();
    match lead.kind {
        LeadKind::None => {}
        LeadKind::Linear => {
            let len = lead.radius.max(lead.linear_length);
            if len > 0.0 {
                let far = anchor.sub(t.scale(len));
                candidates.push(if lead_in { vec![far, anchor] } else { vec![anchor, anchor.add(t.scale(len))] });
            }
        }
        LeadKind::Arc => {
            if lead.radius > 0.0 && lead.sweep_deg > 0.0 {
                for side in sides {
                    candidates.push(lead_arc(anchor, t, lead.radius, lead.sweep_deg.to_radians(), lead.linear_length, side, lead_in, ctx.options.chord_tolerance));
                }
            }
        }
        LeadKind::Vertical => {
            let r = if lead.vertical_radius > 0.0 { lead.vertical_radius } else { lead.radius };
            if r > 0.0 {
                candidates.push(vertical_lead(anchor, t, r, lead_in, ctx.options.chord_tolerance));
            }
        }
    }
    if candidates.is_empty() {
        report.warn(Diagnostic::new("lead-zero-size", format!("The {name} has no size and was skipped")).at(anchor));
        return Ok(None);
    }
    for c in candidates {
        if c.iter().any(|p| p.z >= ctx.heights.clearance) {
            continue;
        }
        if prover.polyline_safe(&c, eps, report, runner)? {
            return Ok(Some(c));
        }
    }
    report.warn(Diagnostic::new("lead-unproven", format!("No {name} could be proven clear of material; the pass starts without one")).at(anchor));
    Ok(None)
}

/// Horizontal arc lead chords. For a lead-in the chords run from the far end
/// to `anchor`; `side` is +1 for the left of the tangent, -1 for the right.
fn lead_arc(anchor: Point3, t: Point3, radius: f64, sweep: f64, linear: f64, side: f64, lead_in: bool, chord_tol: f64) -> Vec<Point3> {
    // Lead-out is the mirror image of a lead-in along the reversed tangent,
    // on the same side of the pass.
    let (t, side) = if lead_in { (t, side) } else { (t.scale(-1.0), -side) };
    let n = Point3::new(-t.y, t.x, 0.0).scale(side);
    let center = anchor.add(n.scale(radius));
    let steps = arc_steps(radius, sweep, chord_tol);
    let mut pts = Vec::with_capacity(steps + 2);
    // Angle 0 is the anchor; the arc runs back along the tangent.
    // Position at angle a (rotating from the anchor away from the tangent):
    // anchor + n*r - (n cos a - t sin a * side_sign) * r, where the sign makes
    // the arc tangent to t at the anchor.
    let far_at = |a: f64| -> Point3 {
        let radial = n.scale(-a.cos()).add(t.scale(-a.sin()));
        center.add(radial.scale(radius))
    };
    let far = far_at(sweep);
    if linear > 0.0 {
        // Tangent of the arc at its far end, extended straight.
        let dir = n.scale(sweep.sin()).add(t.scale(-sweep.cos()));
        pts.push(far.add(dir.scale(linear)));
    }
    for i in 0..=steps {
        let a = sweep * (1.0 - i as f64 / steps as f64);
        pts.push(far_at(a));
    }
    // The lead joins the pass at the anchor exactly, not within rounding.
    *pts.last_mut().expect("at least the anchor") = anchor;
    if !lead_in {
        pts.reverse();
    }
    pts
}

/// Vertical quarter-circle lead: descends radius `r` along the tangent onto
/// the anchor (lead-in) or climbs away from it (lead-out).
fn vertical_lead(anchor: Point3, t: Point3, r: f64, lead_in: bool, chord_tol: f64) -> Vec<Point3> {
    let t = if lead_in { t } else { t.scale(-1.0) };
    let center = anchor.add(Point3::new(0.0, 0.0, r));
    let steps = arc_steps(r, PI / 2.0, chord_tol);
    let mut pts: Vec<Point3> = (0..=steps)
        .map(|i| {
            let a = (PI / 2.0) * (i as f64 / steps as f64);
            // a = 0: far end (back along t, up r); a = 90 deg: the anchor.
            center.add(t.scale(-r * a.cos())).add(Point3::new(0.0, 0.0, -r * a.sin()))
        })
        .collect();
    *pts.last_mut().expect("at least the anchor") = anchor;
    if !lead_in {
        pts.reverse();
    }
    pts
}

/// Chord count for an arc so the sagitta stays within `chord_tol`.
fn arc_steps(radius: f64, sweep: f64, chord_tol: f64) -> usize {
    let max_step = if chord_tol >= radius { PI } else { 2.0 * (1.0 - chord_tol / radius).acos() };
    ((sweep / max_step.max(1e-6)).ceil() as usize).clamp(1, 3600)
}

/// Plan the ramp from height `zf` straight above `entry` down to `entry`.
/// Returns the motions (helix arcs or lead-in chords) when proven.
#[allow(clippy::too_many_arguments)]
fn plan_ramp(pass: &Pass, entry: Point3, zf: f64, ctx: &LinkContext<'_>, prover: &Prover<'_>, eps: f64, report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<Option<Vec<Motion>>, CamError> {
    let ramp = &ctx.params.ramp;
    if ramp.kind == RampKind::Plunge {
        return Ok(None);
    }
    let feed = ctx.feeds.ramp_feed.unwrap_or(ctx.feeds.cutting_feed);
    let angle = ramp.angle_deg.to_radians();
    if angle <= 0.0 {
        report.warn(Diagnostic::new("ramp-zero-angle", "The ramp angle is zero; plunging instead").at(entry));
        return Ok(None);
    }
    let Some(t) = pass.start_tangent() else {
        report.warn(Diagnostic::new("ramp-no-tangent", "The pass has no horizontal direction for a ramp; plunging instead").at(entry));
        return Ok(None);
    };
    let descent = zf - entry.z;
    let proven: Option<Vec<Motion>> = match ramp.kind {
        RampKind::Helix | RampKind::Smooth => {
            let d = if ramp.helix_diameter > 0.0 { ramp.helix_diameter } else { ctx.cutter.radius() };
            // A counter-clockwise helix centred left of the entry ends at the
            // entry with the pass's tangent.
            let n = Point3::new(-t.y, t.x, 0.0);
            let r = 0.5 * d;
            let center = Point3::new(entry.x + n.x * r, entry.y + n.y * r, 0.0);
            let per_turn = TAU * r * angle.tan();
            let turns = descent / per_turn;
            // Discretize for proof; emit as arcs with pitch.
            let steps = (arc_steps(r, TAU, ctx.options.chord_tolerance) as f64 * turns).ceil().max(1.0) as usize;
            let steps = steps.min(100_000);
            let at = |frac: f64| -> Point3 {
                // frac 0 = top (start), 1 = entry. Angle measured from the entry
                // position (radial -n), going backwards (clockwise) for the start.
                let a = -TAU * turns * (1.0 - frac);
                let radial = n.scale(-a.cos()).add(t.scale(a.sin()));
                Point3::new(center.x + radial.x * r, center.y + radial.y * r, entry.z + descent * (1.0 - frac))
            };
            // The proof covers the traverse from above the entry to the helix
            // start and every chord of the helix down to the entry.
            let mut chords: Vec<Point3> = vec![Point3::new(entry.x, entry.y, zf)];
            chords.extend((0..=steps).map(|i| at(i as f64 / steps as f64)));
            if prover.polyline_safe(&chords, eps, report, runner)? {
                let mut motions = Vec::new();
                let start = chords[1];
                motions.push(Motion::link(Point3::new(start.x, start.y, zf), ctx.options.high_feed));
                // One arc per full turn, then the remainder, all ending exactly
                // on their computed points; the last one on the entry.
                let full = turns.floor() as usize;
                let pitch = -per_turn;
                let c3 = Point3::new(center.x, center.y, entry.z);
                for k in 1..=full {
                    let p = at(k as f64 / turns);
                    motions.push(Motion::new(MotionKind::ArcCcw { to: p, center: c3, plane: Default::default(), pitch }, Feed::Rate(feed)));
                }
                if turns - full as f64 > 1e-9 {
                    motions.push(Motion::new(MotionKind::ArcCcw { to: entry, center: c3, plane: Default::default(), pitch }, Feed::Rate(feed)));
                } else if let Some(last) = motions.last_mut() {
                    if let MotionKind::ArcCcw { to, .. } = &mut last.kind {
                        *to = entry;
                    }
                }
                // The last turn slopes down to the entry, so the disc under it
                // is left on that slope; one flat turn at the entry depth (two
                // half turns, for controllers that refuse 360-degree arcs)
                // clears it, when proven.
                let ring_at = |a: f64| -> Point3 {
                    let radial = n.scale(-a.cos()).add(t.scale(a.sin()));
                    Point3::new(center.x + radial.x * r, center.y + radial.y * r, entry.z)
                };
                let ring_steps = arc_steps(r, TAU, ctx.options.chord_tolerance).max(2);
                let ring: Vec<Point3> = (0..=ring_steps).map(|i| if i == ring_steps { entry } else { ring_at(TAU * i as f64 / ring_steps as f64) }).collect();
                if prover.polyline_safe(&ring, eps, report, runner)? {
                    for to in [ring_at(PI), entry] {
                        motions.push(Motion::new(MotionKind::ArcCcw { to, center: c3, plane: Default::default(), pitch: 0.0 }, Feed::Rate(feed)));
                    }
                } else {
                    report.warn(Diagnostic::new("ramp-floor-ring-unproven", "The flat turn at the bottom of the helix could not be proven clear of the part; the floor under the helix is left on its slope").at(entry));
                }
                Some(motions)
            } else {
                None
            }
        }
        RampKind::Profile | RampKind::ZigZag => {
            let length = ctx.options.ramp_length.unwrap_or(if ramp.helix_diameter > 0.0 { 2.0 * ramp.helix_diameter } else { 4.0 * ctx.cutter.radius() });
            // The ramp runs back and forth from the entry along the pass (profile)
            // or along the entry tangent (zig-zag), descending at the ramp angle.
            let track: Vec<Point3> = if ramp.kind == RampKind::Profile {
                let mut pts = vec![entry];
                let mut acc = 0.0;
                for w in pass.points.windows(2) {
                    let seg = w[1].distance_xy(w[0]);
                    if acc + seg >= length {
                        let f = ((length - acc) / seg).clamp(0.0, 1.0);
                        pts.push(w[0].lerp(w[1], f));
                        break;
                    }
                    acc += seg;
                    pts.push(w[1]);
                }
                pts
            } else {
                vec![entry, entry.add(t.scale(length))]
            };
            let track_len: f64 = track.windows(2).map(|w| w[1].distance_xy(w[0])).sum();
            if track_len <= eps {
                None
            } else {
                // An even leg count brings the ramp back to the entry.
                let mut legs = (descent / (track_len * angle.tan())).ceil().max(1.0) as usize;
                if legs % 2 == 1 {
                    legs += 1;
                }
                let drop_per_leg = descent / legs as f64;
                let mut chords = vec![Point3::new(entry.x, entry.y, zf)];
                let mut z = zf;
                for leg in 0..legs {
                    let forward = leg % 2 == 0;
                    let pts: Vec<Point3> = if forward { track.iter().skip(1).copied().collect() } else { track.iter().rev().skip(1).copied().collect() };
                    let mut travelled = 0.0;
                    let mut at = if forward { track[0] } else { track[track.len() - 1] };
                    for p in pts {
                        travelled += p.distance_xy(at);
                        at = p;
                        chords.push(Point3::new(p.x, p.y, z - drop_per_leg * travelled / track_len));
                    }
                    z -= drop_per_leg;
                }
                if let Some(last) = chords.last_mut() {
                    *last = entry;
                }
                if prover.polyline_safe(&chords, eps, report, runner)? {
                    Some(chords.iter().skip(1).map(|p| Motion::new(MotionKind::LeadIn { to: *p }, Feed::Rate(feed))).collect())
                } else {
                    None
                }
            }
        }
        RampKind::Plunge => None,
    };
    if proven.is_none() {
        report.warn(Diagnostic::new("ramp-unproven", "The ramp could not be proven clear of material; plunging instead").at(entry));
    }
    Ok(proven)
}

/// Deterministic nearest-neighbour ordering. Returns `(pass index, reversed)`
/// in machining order. Ties keep the original order.
pub fn order_passes(passes: &[Pass], start: Point3, allow_reverse: bool) -> Vec<(usize, bool)> {
    let mut remaining: Vec<usize> = (0..passes.len()).collect();
    let mut at = start;
    let mut out = Vec::with_capacity(passes.len());
    while !remaining.is_empty() {
        let mut best: Option<(usize, bool, f64)> = None;
        for (k, &i) in remaining.iter().enumerate() {
            let p = &passes[i];
            let forward = at.distance_xy(p.start());
            let candidates: [(bool, f64); 2] = [(false, forward), (true, if allow_reverse { at.distance_xy(p.end()) } else { f64::INFINITY })];
            for (rev, cost) in candidates {
                let better = match best {
                    None => true,
                    Some((_, _, c)) => cost < c - 1e-12,
                };
                if better {
                    best = Some((k, rev, cost));
                }
            }
        }
        let (k, rev, _) = best.expect("remaining is non-empty");
        let i = remaining.remove(k);
        at = if rev { passes[i].start() } else { passes[i].end() };
        out.push((i, rev));
    }
    out
}

/// Extract the cut passes of an existing stream: every run of `Cut` motions,
/// starting from the position the run began at. Leads, links and rapids are
/// discarded (the linker regenerates them). Arc motions are refused: relink
/// before arc fitting.
pub fn passes_of(path: &Toolpath) -> Result<Vec<Pass>, CamError> {
    let mut passes = Vec::new();
    let mut current: Option<Pass> = None;
    let mut at: Option<Point3> = None;
    for m in &path.motions {
        match &m.kind {
            MotionKind::Cut { to } => {
                let start = at.ok_or_else(|| CamError::InvalidInput("relink: a cut has no start position".into()))?;
                let pass = current.get_or_insert_with(|| Pass::new(vec![start], m.path_id));
                pass.points.push(*to);
                at = Some(*to);
            }
            MotionKind::ArcCw { .. } | MotionKind::ArcCcw { .. } => {
                return Err(CamError::InvalidInput("relink: the stream already contains arcs; relink before arc fitting".into()));
            }
            other => {
                if let Some(p) = current.take() {
                    passes.push(p);
                }
                if let Some(p) = other.end() {
                    at = Some(p);
                }
            }
        }
    }
    if let Some(p) = current.take() {
        passes.push(p);
    }
    Ok(passes)
}

/// Re-link a strategy's toolpath: keep its cut passes and their order, replace
/// every approach, link, lead and retract by this linker's proven ones. The
/// result keeps the operation and tool identity and merges the reports.
pub fn relink(path: &Toolpath, ctx: &LinkContext<'_>, runner: &mut Runner<'_>) -> Result<Toolpath, CamError> {
    let passes = passes_of(path)?;
    if passes.is_empty() {
        return Err(CamError::InvalidInput(format!("relink: operation {} has no cut passes", path.operation)));
    }
    let linked = connect(&passes, ctx, runner)?;
    Ok(merge_linked(path, linked))
}

/// `path` with `linked`'s motions and its report merged in.
fn merge_linked(path: &Toolpath, linked: Linked) -> Toolpath {
    let mut out = path.clone();
    out.motions = linked.motions;
    // Strategy warnings about their own placeholder links are superseded.
    out.report.warnings.retain(|w| w.code != "retract-linking");
    out.report.warnings.extend(linked.report.warnings);
    out.report.counts.samples += linked.report.counts.samples;
    out.report.counts.candidates += linked.report.counts.candidates;
    out.report.counts.contacts += linked.report.counts.contacts;
    out.report.counts.motions = out.motions.len() as u64;
    out.report.finish();
    out
}

/// Why a strategy cannot cut a span of its path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrimReason {
    /// The cutter cannot reach the span at all (`unreachable-trimmed`).
    Unreachable,
    /// The shank or holder meets material there (`holder-trimmed`).
    Holder,
}

impl TrimReason {
    /// The report code, shared with lane C17's multi-axis trim.
    pub fn code(self) -> &'static str {
        match self {
            TrimReason::Unreachable => "unreachable-trimmed",
            TrimReason::Holder => "holder-trimmed",
        }
    }
}

/// Cut motions a strategy has proven it must not make.
#[derive(Clone, Debug, PartialEq)]
pub struct BlockedSpan {
    /// Indices into the path's motions; every one must be a `Cut`.
    pub motions: std::ops::Range<usize>,
    pub reason: TrimReason,
    /// Highest material that may stand at the span. `None`: the operation's
    /// material top, else the pre-operation stock's highest top.
    pub material_top: Option<f64>,
    /// XY distance from the tool axis within which that material may stand
    /// (the blocking shank or holder radius; at least the cutter radius).
    pub reach: f64,
}

/// One trimmed span: where material is left.
#[derive(Clone, Debug, PartialEq)]
pub struct TrimmedSpan {
    pub reason: TrimReason,
    /// The tool positions that were not cut (the span's start, then each removed cut's end).
    pub points: Vec<Point3>,
    /// Where material may be left: the points' XY grown by the reach, from the
    /// lowest point to the material top.
    pub bounds: crate::model::Bounds3,
}

/// A path with its blocked spans removed and the gaps relinked.
#[derive(Clone, Debug, PartialEq)]
pub struct Trimmed {
    pub path: Toolpath,
    pub spans: Vec<TrimmedSpan>,
}

/// Remove the cut motions a strategy proved blocked (shank or holder against
/// the stock, or unreachable) and relink the rest with proven links. Material
/// may stand anywhere over a trimmed span up to its material top, so the
/// gaps' links keep above it there whatever the remaining cuts remove. Each
/// span adds an `empty_regions` entry with its code, and each code one
/// warning at its first span listing where material is left. Without spans
/// the path is returned as it is. Relink before arc fitting.
pub fn trim_blocked(path: &Toolpath, blocked: &[BlockedSpan], ctx: &LinkContext<'_>, runner: &mut Runner<'_>) -> Result<Trimmed, CamError> {
    if blocked.is_empty() {
        return Ok(Trimmed { path: path.clone(), spans: Vec::new() });
    }
    let fallback_top = ctx.options.material_top.or_else(|| ctx.stock.and_then(|e| e.max_top()));
    let mut reason_of: Vec<Option<usize>> = vec![None; path.motions.len()];
    for (k, span) in blocked.iter().enumerate() {
        if span.motions.is_empty() || span.motions.end > path.motions.len() {
            return Err(CamError::InvalidInput(format!("trim: span {k} ({:?}) is empty or outside the {} motions", span.motions, path.motions.len())));
        }
        if !(span.reach.is_finite() && span.reach >= 0.0) || span.material_top.is_some_and(|z| !z.is_finite()) {
            return Err(CamError::InvalidInput(format!("trim: span {k} has a non-finite reach or material top")));
        }
        if span.material_top.or(fallback_top).is_none() {
            return Err(CamError::InvalidInput(format!("trim: span {k} has no material top and the operation has no stock")));
        }
        for i in span.motions.clone() {
            if !matches!(path.motions[i].kind, MotionKind::Cut { .. }) {
                return Err(CamError::InvalidInput(format!("trim: span {k} covers motion {i}, which is not a straight cut")));
            }
            reason_of[i].get_or_insert(k);
        }
    }
    // Passes of the unblocked cuts; each blocked run becomes a trimmed span.
    let mut passes = Vec::new();
    let mut current: Option<Pass> = None;
    let mut spans: Vec<(usize, Vec<Point3>)> = Vec::new();
    let mut open_span: Option<(usize, Vec<Point3>)> = None;
    let mut at: Option<Point3> = None;
    for (i, m) in path.motions.iter().enumerate() {
        match (&m.kind, reason_of[i]) {
            (MotionKind::Cut { to }, Some(k)) => {
                let start = at.ok_or_else(|| CamError::InvalidInput("trim: a cut has no start position".into()))?;
                if let Some(p) = current.take() {
                    passes.push(p);
                }
                match &mut open_span {
                    Some((open, points)) if *open == k => points.push(*to),
                    _ => {
                        if let Some(done) = open_span.take() {
                            spans.push(done);
                        }
                        open_span = Some((k, vec![start, *to]));
                    }
                }
                at = Some(*to);
            }
            (MotionKind::Cut { to }, None) => {
                if let Some(done) = open_span.take() {
                    spans.push(done);
                }
                let start = at.ok_or_else(|| CamError::InvalidInput("trim: a cut has no start position".into()))?;
                current.get_or_insert_with(|| Pass::new(vec![start], m.path_id)).points.push(*to);
                at = Some(*to);
            }
            (MotionKind::ArcCw { .. } | MotionKind::ArcCcw { .. }, _) => {
                return Err(CamError::InvalidInput("trim: the stream already contains arcs; trim before arc fitting".into()));
            }
            (other, _) => {
                if let Some(done) = open_span.take() {
                    spans.push(done);
                }
                if let Some(p) = current.take() {
                    passes.push(p);
                }
                if let Some(p) = other.end() {
                    at = Some(p);
                }
            }
        }
    }
    if let Some(done) = open_span.take() {
        spans.push(done);
    }
    if let Some(p) = current.take() {
        passes.push(p);
    }
    if passes.is_empty() {
        return Err(CamError::InvalidInput(format!("trim: every cut of operation {} is blocked", path.operation)));
    }
    let trimmed: Vec<TrimmedSpan> = spans
        .iter()
        .map(|(k, points)| {
            let span = &blocked[*k];
            let reach = span.reach.max(ctx.cutter.radius());
            let top = span.material_top.or(fallback_top).expect("checked above");
            let mut bounds = crate::model::Bounds3::from_points(points.iter().copied()).expanded(reach);
            bounds.min.z = points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
            bounds.max.z = top.max(points.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max));
            TrimmedSpan { reason: span.reason, points: points.clone(), bounds }
        })
        .collect();
    let keep: Vec<KeepOut> = spans
        .iter()
        .map(|(k, points)| {
            let span = &blocked[*k];
            KeepOut { points: points.clone(), reach: span.reach.max(ctx.cutter.radius()), z: span.material_top.or(fallback_top).expect("checked above") }
        })
        .collect();
    let linked = connect_with(&passes, ctx, &keep, runner)?;
    let mut out = merge_linked(path, linked);
    for reason in [TrimReason::Unreachable, TrimReason::Holder] {
        let of: Vec<&TrimmedSpan> = trimmed.iter().filter(|t| t.reason == reason).collect();
        let Some(first) = of.first() else { continue };
        let shown: Vec<String> = of.iter().take(20).map(|t| { let p = t.points[0]; format!("({:.4}, {:.4}, {:.4})", p.x, p.y, p.z) }).collect();
        let more = if of.len() > shown.len() { format!(" and {} more", of.len() - shown.len()) } else { String::new() };
        let why = match reason {
            TrimReason::Unreachable => "the cutter cannot reach them",
            TrimReason::Holder => "the shank or holder meets material there",
        };
        let spans: Vec<String> = of
            .iter()
            .take(20)
            .map(|t| {
                let (a, b) = (t.points[0], t.points[t.points.len() - 1]);
                format!("({:.3}, {:.3}, {:.3})-({:.3}, {:.3}, {:.3}) material to {:.3}", a.x, a.y, a.z, b.x, b.y, b.z, t.bounds.max.z)
            })
            .collect();
        out.report.warn(
            Diagnostic::new(reason.code(), format!("{} cut spans are left out because {why}: the path breaks, retracts and re-enters; material is left at {}{more}", of.len(), shown.join(", ")))
                .at(first.points[0])
                .detail(format!("spans (from-to): {}{more}", spans.join(", "))),
        );
        for _ in &of {
            out.report.empty_regions.push(crate::model::EmptyRegion { reason: reason.code().into(), target: None });
        }
    }
    out.report.finish();
    Ok(Trimmed { path: out, spans: trimmed })
}

/// The whole post-generation pipeline every strategy's output goes through:
/// relink with proven links, drop redundant points within
/// `filter_tolerance`, then fit arcs when the operation asks for them.
pub fn finish(path: &Toolpath, ctx: &LinkContext<'_>, filter_tolerance: f64, arcs: Option<&crate::arcfit::ArcFitOptions>, runner: &mut Runner<'_>) -> Result<Toolpath, CamError> {
    let linked = relink(path, ctx, runner)?;
    let filtered = if filter_tolerance > 0.0 { crate::filter::filter_toolpath(&linked, filter_tolerance)? } else { linked };
    match arcs {
        Some(opts) => crate::arcfit::fit_toolpath(&filtered, opts),
        None => Ok(filtered),
    }
}

