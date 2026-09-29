//! The ANALYTIC hidden-line pass: what a sheet draws of the model, computed
//! from the B-rep rather than from the display mesh.
//!
//! # The algorithm
//!
//! 1. **Candidate curves.** Every non-degenerate B-rep edge, sampled on its
//!    own curve, and the SILHOUETTE of every curved face — the locus on the
//!    exact surface where the viewing direction is tangent to it: two
//!    generator lines of a cylinder or cone (closed form), one great circle of
//!    a sphere (closed form), and for a torus, a general revolution or a
//!    fitted surface the zero set of `n(u, v) · d` traced over the face's
//!    parameter box and solved onto the surface through its own evaluator.
//!    Each silhouette is trimmed to its face. An edge is INKED when its two
//!    faces meet at a crease past [`EDGE_ANGLE_DEG`](super::project::EDGE_ANGLE_DEG)
//!    (the rule the mesh pass used, read here off the exact normals), when it
//!    is a TANGENT LINE — a smooth edge between two surfaces, a round's or a
//!    blend's, told from a split of one surface by the jump in curvature
//!    across it ([`one_surface`]) — when it bounds only one face, or when more
//!    than two share it; a smooth edge between two fragments of one surface
//!    is kept as a SPLITTER only, and a seam (one face on both sides) is
//!    neither.
//! 2. **Splitting.** Every candidate is split where its projection crosses
//!    another's, and where another's END lands on it — the only places the
//!    number of surfaces in front of a line can change. The crossing test is
//!    the kernel's own `segment_intersection` over the sampled paper chords.
//! 3. **Visibility.** Each piece is decided at its MIDPOINT, evaluated on the
//!    exact curve (a chord's midpoint sits inside a curved surface, and the
//!    ray from there to the eye can cross the very surface it bounds): the
//!    view ray from that point toward the eye is intersected with every face
//!    whose paper box holds the point — a plane in closed form, anything else
//!    through the kernel's `intersect_curve_surface` on the exact carrier —
//!    and the piece is hidden when a hit inside that face's trim is nearer the
//!    eye than the piece by more than the depth floor.
//! 4. **Runs.** Consecutive visible pieces of one candidate are one run, so a
//!    straight edge a hidden line crosses still comes back as its two ends,
//!    and edge runs chain through a vertex exactly two inked edges share.
//!
//! # The two floors
//!
//! Both are the B-rep's own agreement bar, `KernelTolerances::pcurve_consistency`
//! (the largest disagreement a valid in-memory B-rep allows between an edge's
//! curve and the carriers it lies on), carried to the paper at the view's
//! scale:
//!
//! - the **depth floor** — a face must be nearer the eye than the piece by
//!   more than that bar to hide it, because an edge's own curve may sit that
//!   far off the face it bounds;
//! - the **crossing floor** — two projected curves closer than that bar touch,
//!   and two split points closer than it along a curve are one. A touch is
//!   measured against a CHORD, so it also admits the chord's own sag,
//!   [`CHORD_MM`].
//!
//! Visibility is exact to those floors: no raster cell, no bias, no dropped
//! fringe. What it does not see is what the SAMPLING does not: a trim notch
//! or a silhouette loop smaller than one chord.
//!
//! # Before the B-rep arrives
//!
//! Steps 2 to 4 also run on a DISPLAY ([`visible_mesh_lines`]): the display
//! edges and the mesh's silhouettes as candidates, the display triangles as
//! what hides them. It is what a placement draws while its solids' topology is
//! not in the scene, and the drawing says so — a stated approximation, the
//! tessellation's chords in place of the exact curves.

use std::collections::HashMap;

use brep_kernel::{
    containment_lane, intersect_analytic_pair, intersect_curve_surface, intersect_surfaces,
    make_line, make_plane, parameter_point_in_face, project_point_to_curve,
    project_point_to_surface, segment_intersection, trim_sample_count, trim_station,
    AnalyticSurface, BrepSolid, ContainmentLane, FaceRecord, KernelTolerances, NurbsCurve,
    NurbsSurface, PolygonClass, SurfaceIntersectionOptions, Vec2, Vec3,
};

use super::project::{ViewFrame, EDGE_ANGLE_DEG};
use super::PlacedView;

/// How far a sampled paper polyline may sit off its exact curve, paper
/// millimetres. The comparand is the model line's own stroke,
/// [`EDGE_WIDTH`](super::svg::EDGE_WIDTH) = 0.35 mm: seventy times finer, so
/// no printed or zoomed drawing can show the chord.
pub const CHORD_MM: f64 = 0.005;

/// Deepest a sampled interval is halved in search of [`CHORD_MM`].
const MAX_SUBDIVISION: u32 = 14;

/// The kernel's tangency bar on a curve-surface hit (`intersect_curve_surface`
/// flags `|n · t| <= 1e-3`), used for the closed-form plane too: a face the
/// view ray only grazes has no area in front of the piece to hide it with.
const GRAZING_COS: f64 = 1e-3;

/// The fewest stations a silhouette is classified at against its face's trim,
/// whatever its chord sampling: a trim notch shorter than 1/64 of the curve
/// between two chord samples is what this pass does not see of a silhouette.
const TRIM_STATIONS: usize = 64;

/// Bisection steps when a curve's parameter is pinned where it leaves a face,
/// crosses the cutting plane or crosses the detail circle — 2^-40 of one
/// sampled interval, far under every floor.
const BISECTION_STEPS: u32 = 40;


fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

fn a3(p: Vec3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

/// What the drawn model is restricted to before anything is decided.
pub enum Clip<'a> {
    /// A plain placement: the whole model.
    Whole,
    /// A SECTION: the half-space at or beyond the cutting plane, which is the
    /// view's own target plane (depth zero, the camera looking along its
    /// normal). `cap` is the cut region on the paper — the exact section
    /// curves — which hides everything behind it.
    Beyond { cap: &'a [Vec<[f64; 2]>] },
    /// A DETAIL: the cylinder of MODEL radius `radius` along the viewing
    /// direction through the view's target.
    Circle { radius: f64 },
}

/// The floors of one placement, see the module docs.
#[derive(Debug, Clone, Copy)]
pub struct Floors {
    /// The B-rep's agreement bar, model units.
    pub bar: f64,
    /// The depth floor, paper millimetres (`bar × scale`).
    pub depth_mm: f64,
    /// The crossing floor, paper millimetres (`bar × scale`).
    pub crossing_mm: f64,
}

impl Floors {
    pub fn of(scale: f64) -> Floors {
        let bar = KernelTolerances::default().pcurve_consistency;
        Floors { bar, depth_mm: bar * scale, crossing_mm: bar * scale }
    }
}

/// The exact result of one placement: the visible runs on the paper, plus
/// what the A/B comparison and the tests read.
#[derive(Debug, Clone, Default)]
pub struct HlrResult {
    pub runs: Vec<Vec<[f64; 2]>>,
    /// Inked candidate curves before splitting (edges and silhouette pieces).
    pub inked: usize,
    /// Silhouette pieces among them.
    pub silhouettes: usize,
    /// Pieces the splitting produced and visibility decided.
    pub pieces: usize,
    /// How many times the pass asked the kernel whether a point is inside a
    /// face's trim (`parameter_point_in_face`) — the question the projected and
    /// parameter trims exist to answer first.
    pub trim_queries: usize,
    /// How many view rays the pass intersected with a curved face's carrier
    /// (`intersect_curve_surface`).
    pub ray_queries: usize,
}

thread_local! {
    /// The kernel questions of the pass running on this thread, for
    /// [`HlrResult::trim_queries`] and [`HlrResult::ray_queries`].
    static TRIM_QUERIES: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    static RAY_QUERIES: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}



// ---------------------------------------------------------------------------
// Curves
// ---------------------------------------------------------------------------

/// A candidate's exact carrier, evaluated at its own parameter.
#[derive(Clone)]
enum Carrier<'a> {
    /// A B-rep edge's curve.
    Edge(&'a NurbsCurve),
    /// A curve the kernel handed back (an exact section of a carrier).
    Curve(NurbsCurve),
    /// A straight generator from `a` (s = 0) to `b` (s = 1).
    Line { a: Vec3, b: Vec3 },
    /// `centre + e1 cos s + e2 sin s` — `e1`, `e2` carry the radius.
    Circle { centre: Vec3, e1: Vec3, e2: Vec3 },
    /// A silhouette TRACED on a general surface: `uv[floor(s)]` toward
    /// `uv[floor(s) + 1]`, re-solved onto `n · d = 0` at every evaluation.
    Traced { surface: &'a NurbsSurface, uv: Vec<[f64; 2]>, dir: Vec3 },
    /// A marched section polyline (`intersect_surfaces`): `s` indexes the
    /// points, which the marcher already put on both carriers.
    Polyline(Vec<Vec3>),
}

impl Carrier<'_> {
    fn at(&self, s: f64) -> Option<Vec3> {
        match self {
            Carrier::Edge(curve) => curve.evaluate(s).ok(),
            Carrier::Curve(curve) => curve.evaluate(s).ok(),
            Carrier::Line { a, b } => Some(a.add(b.sub(*a).scale(s))),
            Carrier::Circle { centre, e1, e2 } => Some(centre.add(e1.scale(s.cos())).add(e2.scale(s.sin()))),
            Carrier::Traced { surface, uv, dir } => {
                let (point, _) = traced_at(surface, uv, *dir, s)?;
                Some(point)
            }
            Carrier::Polyline(points) => {
                let last = points.len().checked_sub(1)?;
                let i = (s.floor().max(0.0) as usize).min(last.saturating_sub(1));
                let f = (s - i as f64).clamp(0.0, 1.0);
                let (a, b) = (points[i], points[(i + 1).min(last)]);
                Some(a.add(b.sub(a).scale(f)))
            }
        }
    }

    /// Whether the carrier is a straight line, so sampling needs no interior
    /// point to be exact.
    fn straight(&self) -> bool {
        match self {
            Carrier::Line { .. } => true,
            Carrier::Edge(curve) => curve.degree == 1 && curve.control_points.len() == 2,
            Carrier::Curve(curve) => curve.degree == 1 && curve.control_points.len() == 2,
            _ => false,
        }
    }
}

/// `n(u, v) · d` with the normal normalised — the silhouette's defining
/// function, sign-blind (the locus is the same for either orientation).
fn grazing(surface: &NurbsSurface, dir: Vec3, u: f64, v: f64) -> Option<f64> {
    surface.normal(u, v).ok().map(|n| n.dot(dir))
}

/// A traced silhouette at fractional station `s`: the straight uv step
/// between the two bracketing stations, then a root of `n · d` along the uv
/// direction PERPENDICULAR to that step, bracketed within one step's length.
fn traced_at(surface: &NurbsSurface, uv: &[[f64; 2]], dir: Vec3, s: f64) -> Option<(Vec3, [f64; 2])> {
    let last = uv.len().checked_sub(1)?;
    if last == 0 {
        let p = surface.evaluate(uv[0][0], uv[0][1]).ok()?;
        return Some((p, uv[0]));
    }
    let i = (s.floor().max(0.0) as usize).min(last - 1);
    let f = (s - i as f64).clamp(0.0, 1.0);
    let (a, b) = (uv[i], uv[i + 1]);
    let base = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
    let step = [b[0] - a[0], b[1] - a[1]];
    let length = (step[0] * step[0] + step[1] * step[1]).sqrt();
    let solved = if f <= 0.0 || f >= 1.0 || length <= 0.0 {
        base
    } else {
        let across = [-step[1], step[0]];
        root_along(surface, dir, base, across).unwrap_or(base)
    };
    let p = surface.evaluate(solved[0], solved[1]).ok()?;
    Some((p, solved))
}

/// A root of `n · d` on the uv segment `base ± across`, by bisection from the
/// first sign change outward from `base`.
fn root_along(surface: &NurbsSurface, dir: Vec3, base: [f64; 2], across: [f64; 2]) -> Option<[f64; 2]> {
    let at = |t: f64| [base[0] + across[0] * t, base[1] + across[1] * t];
    let g0 = {
        let p = at(0.0);
        grazing(surface, dir, p[0], p[1])?
    };
    if g0 == 0.0 {
        return Some(base);
    }
    for (lo, hi) in [(0.0, 0.5), (0.0, -0.5), (0.5, 1.0), (-0.5, -1.0)] {
        let (p_lo, p_hi) = (at(lo), at(hi));
        let g_lo = grazing(surface, dir, p_lo[0], p_lo[1])?;
        let g_hi = grazing(surface, dir, p_hi[0], p_hi[1])?;
        if g_lo.signum() == g_hi.signum() {
            continue;
        }
        let (mut lo, mut hi, mut g_lo) = (lo, hi, g_lo);
        for _ in 0..BISECTION_STEPS {
            let mid = 0.5 * (lo + hi);
            let p = at(mid);
            let g = grazing(surface, dir, p[0], p[1])?;
            if g.signum() == g_lo.signum() {
                lo = mid;
                g_lo = g;
            } else {
                hi = mid;
            }
        }
        return Some(at(0.5 * (lo + hi)));
    }
    None
}

/// The paper image of a model point, and its view depth in MODEL units.
fn project(frame: &ViewFrame, placed: &PlacedView, p: Vec3) -> ([f64; 2], [f64; 3]) {
    let view = frame.to_view(a3(p));
    ([placed.position[0] + view[0] * placed.scale, placed.position[1] - view[1] * placed.scale], view)
}

fn paper_distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// Distance from `p` to the paper segment `a`–`b`, and the segment parameter
/// of the foot.
fn segment_foot(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> (f64, f64) {
    let d = [b[0] - a[0], b[1] - a[1]];
    let length2 = d[0] * d[0] + d[1] * d[1];
    let t = if length2 > 0.0 {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / length2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let foot = [a[0] + d[0] * t, a[1] + d[1] * t];
    (paper_distance(p, foot), t)
}

/// One sampled curve: parameters, model points and paper points, in order.
#[derive(Debug, Clone, Default)]
struct Samples {
    params: Vec<f64>,
    points: Vec<Vec3>,
    paper: Vec<[f64; 2]>,
    view: Vec<[f64; 3]>,
}

impl Samples {
    fn push(&mut self, s: f64, p: Vec3, frame: &ViewFrame, placed: &PlacedView) {
        let (paper, view) = project(frame, placed, p);
        self.params.push(s);
        self.points.push(p);
        self.paper.push(paper);
        self.view.push(view);
    }

    fn len(&self) -> usize {
        self.params.len()
    }

    fn paper_length(&self) -> f64 {
        self.paper.windows(2).map(|w| paper_distance(w[0], w[1])).sum()
    }
}

/// Sample `carrier` over `[s0, s1]` so every chord is within [`CHORD_MM`] of
/// the exact curve ON THE PAPER, starting from `initial` equal intervals. The
/// test is against the chord as a SEGMENT, so a curve that doubles back in
/// projection (a circle seen edge-on) is still subdivided at its turn.
fn sample(
    carrier: &Carrier,
    s0: f64,
    s1: f64,
    initial: usize,
    frame: &ViewFrame,
    placed: &PlacedView,
) -> Samples {
    let mut out = Samples::default();
    let initial = if carrier.straight() { 1 } else { initial.max(2) };
    let Some(first) = carrier.at(s0) else { return out };
    out.push(s0, first, frame, placed);
    for k in 0..initial {
        let a = s0 + (s1 - s0) * k as f64 / initial as f64;
        let b = s0 + (s1 - s0) * (k + 1) as f64 / initial as f64;
        let (Some(pa), Some(pb)) = (carrier.at(a), carrier.at(b)) else { continue };
        subdivide(carrier, (a, pa), (b, pb), 0, frame, placed, &mut out);
    }
    out
}

fn subdivide(
    carrier: &Carrier,
    (a, pa): (f64, Vec3),
    (b, pb): (f64, Vec3),
    depth: u32,
    frame: &ViewFrame,
    placed: &PlacedView,
    out: &mut Samples,
) {
    if depth < MAX_SUBDIVISION && !carrier.straight() {
        let mid = 0.5 * (a + b);
        if let Some(pm) = carrier.at(mid) {
            let (paper_a, _) = project(frame, placed, pa);
            let (paper_b, _) = project(frame, placed, pb);
            let (paper_m, _) = project(frame, placed, pm);
            if segment_foot(paper_m, paper_a, paper_b).0 > CHORD_MM {
                subdivide(carrier, (a, pa), (mid, pm), depth + 1, frame, placed, out);
                subdivide(carrier, (mid, pm), (b, pb), depth + 1, frame, placed, out);
                return;
            }
        }
    }
    out.push(b, pb, frame, placed);
}

// ---------------------------------------------------------------------------
// Candidates
// ---------------------------------------------------------------------------

/// Which B-rep vertex an edge candidate starts or ends on, for chaining.
type VertexKey = (usize, u64);

struct Candidate<'a> {
    carrier: Carrier<'a>,
    samples: Samples,
    /// Drawn (an inked edge or a silhouette) or only a SPLITTER.
    inked: bool,
    silhouette: bool,
    /// The B-rep edge an edge candidate samples.
    edge: Option<(usize, u64)>,
    ends: Option<(VertexKey, VertexKey)>,
    /// Paper box of the samples.
    bounds: [f64; 4],
}

fn bounds_of(paper: &[[f64; 2]]) -> [f64; 4] {
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for p in paper {
        b[0] = b[0].min(p[0]);
        b[1] = b[1].min(p[1]);
        b[2] = b[2].max(p[0]);
        b[3] = b[3].max(p[1]);
    }
    b
}

/// Initial sampling density of a NURBS curve: a few intervals per span.
fn curve_initial(curve: &NurbsCurve) -> usize {
    let spans = curve.control_points.len().saturating_sub(curve.degree).max(1);
    (spans * (curve.degree + 1)).clamp(2, 256)
}

/// The outward normal of `face` at model point `p` (the carrier's normal,
/// turned by `same_sense`), through the carrier's own projection.
fn outward_normal(face: &FaceRecord, p: Vec3) -> Option<Vec3> {
    let foot = project_point_to_surface(&face.surface, p).ok()?;
    let n = face.surface.normal(foot.u, foot.v).ok()?;
    Some(if face.same_sense { n } else { n.scale(-1.0) })
}

/// Whether an edge between `a` and `b` is a CREASE anywhere along it: the two
/// outward normals more than [`EDGE_ANGLE_DEG`] apart at any of five interior
/// stations. A normal that cannot be evaluated counts as a crease — drawn
/// rather than guessed away.
fn is_crease(curve: &NurbsCurve, t0: f64, t1: f64, a: &FaceRecord, b: &FaceRecord) -> bool {
    let bar = EDGE_ANGLE_DEG.to_radians().cos();
    for k in 1..=5 {
        let t = t0 + (t1 - t0) * k as f64 / 6.0;
        let Ok(p) = curve.evaluate(t) else { return true };
        let (Some(na), Some(nb)) = (outward_normal(a, p), outward_normal(b, p)) else { return true };
        if na.dot(nb) < bar {
            return true;
        }
    }
    false
}

/// Every face's index by the edges its loops use: `edge id -> [(face index)]`.
fn edge_faces(solid: &BrepSolid) -> HashMap<u64, Vec<(usize, usize)>> {
    let mut map: HashMap<u64, Vec<(usize, usize)>> = HashMap::new();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            for lp in &face.loops {
                for coedge in &lp.coedges {
                    map.entry(coedge.edge_id).or_default().push((shell_index, face_index));
                }
            }
        }
    }
    map
}

/// The diagonal of the box two faces' BOUNDARIES span — both ends and the
/// middle of every edge either face's loops use: how far, to the scale
/// [`one_surface`] needs, the pair reaches from an edge it shares. A face that
/// bulges past its own boundary reaches farther than this.
fn pair_extent(solid: &BrepSolid, a: &FaceRecord, b: &FaceRecord, edge_index: &HashMap<u64, usize>) -> f64 {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for coedge in [a, b].into_iter().flat_map(|face| &face.loops).flat_map(|lp| &lp.coedges) {
        let Some(edge) = edge_index.get(&coedge.edge_id).map(|&i| &solid.edges[i]) else { continue };
        for t in [edge.t0, 0.5 * (edge.t0 + edge.t1), edge.t1] {
            if let Ok(q) = edge.curve.evaluate(t) {
                lo = Vec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
                hi = Vec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
            }
        }
    }
    if lo.x <= hi.x {
        hi.sub(lo).length()
    } else {
        0.0
    }
}

/// The normal curvature of `face` at `p` ACROSS the direction `tangent`,
/// signed against the face's outward normal, read off the carrier's second
/// derivatives at the foot of `p`.
fn curvature_across(face: &FaceRecord, p: Vec3, tangent: Vec3) -> Option<f64> {
    let foot = project_point_to_surface(&face.surface, p).ok()?;
    let d = face.surface.derivatives(foot.u, foot.v, 2).ok()?;
    let (su, sv) = (d[1][0], d[0][1]);
    let normal = su.cross(sv).normalized().ok()?;
    let normal = if face.same_sense { normal } else { normal.scale(-1.0) };
    let across = normal.cross(tangent).normalized().ok()?;
    // `across` in the carrier's own tangent basis, then II / I along it.
    let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let det = e * g - f * f;
    if det.abs() <= f64::EPSILON * e * g {
        return None;
    }
    let (cu, cv) = (across.dot(su), across.dot(sv));
    let (a, b) = ((g * cu - f * cv) / det, (e * cv - f * cu) / det);
    let second = d[2][0].dot(normal) * a * a + 2.0 * d[1][1].dot(normal) * a * b + d[0][2].dot(normal) * b * b;
    let first = e * a * a + 2.0 * f * a * b + g * b * b;
    (first > 0.0).then(|| second / first)
}

/// Whether the two faces of a SMOOTH edge are ONE surface — fragments of a
/// split no merge removed, whose shared edge is not a line of the part —
/// rather than two surfaces meeting tangentially, where the edge is a TANGENT
/// LINE (a round's, a blend's) and is drawn.
///
/// Two carriers tangent along an edge whose normal curvatures across it
/// differ by Δκ part by Δκ·w²/2 at a distance w from it. They are one surface
/// when that parting, over `extent` ([`pair_extent`]), is within the B-rep bar at every one of five interior stations —
/// the same stations and the same doubt rule as [`is_crease`]: a curvature
/// that cannot be evaluated draws the line. Read at the edge, where both
/// carriers are inside their own domains, so a fragment's carrier being
/// bounded to its fragment does not matter. What it does not see is a
/// curvature-CONTINUOUS (G2) joint between two surfaces: its edge is drawn
/// as one surface's.
fn one_surface(curve: &NurbsCurve, t0: f64, t1: f64, a: &FaceRecord, b: &FaceRecord, extent: f64, bar: f64) -> bool {
    for k in 1..=5 {
        let t = t0 + (t1 - t0) * k as f64 / 6.0;
        let (Ok(p), Ok(d)) = (curve.evaluate(t), curve.derivatives(t, 1)) else { return false };
        let (Some(ka), Some(kb)) = (curvature_across(a, p, d[1]), curvature_across(b, p, d[1])) else { return false };
        if (ka - kb).abs() * extent * extent * 0.5 > bar {
            return false;
        }
    }
    true
}

fn edge_candidates<'a>(
    solid_index: usize,
    solid: &'a BrepSolid,
    frame: &ViewFrame,
    placed: &PlacedView,
    floors: &Floors,
) -> Vec<Candidate<'a>> {
    let faces = edge_faces(solid);
    let edge_index: HashMap<u64, usize> = solid.edges.iter().enumerate().map(|(i, edge)| (edge.id, i)).collect();
    // How far each smooth pair of faces reaches, read once per pair.
    let mut extents: HashMap<[(usize, usize); 2], f64> = HashMap::new();
    let mut out = Vec::new();
    for edge in &solid.edges {
        if edge.degenerate {
            continue;
        }
        let owners = faces.get(&edge.id).cloned().unwrap_or_default();
        let mut distinct = owners.clone();
        distinct.sort_unstable();
        distinct.dedup();
        let inked = match distinct.len() {
            // A seam: the face meets itself. Not a line of the drawing, and
            // not a place visibility can change (the surface is one surface
            // across it), so it is not even a splitter.
            1 if owners.len() >= 2 => continue,
            // A crease, or a tangent line between two surfaces; an edge
            // between two fragments of one surface is a splitter only.
            2 => {
                let face = |(s, f): (usize, usize)| &solid.shells[s].faces[f];
                let (a, b) = (face(distinct[0]), face(distinct[1]));
                is_crease(&edge.curve, edge.t0, edge.t1, a, b) || {
                    let extent = *extents
                        .entry([distinct[0], distinct[1]])
                        .or_insert_with(|| pair_extent(solid, a, b, &edge_index));
                    !one_surface(&edge.curve, edge.t0, edge.t1, a, b, extent, floors.bar)
                }
            }
            _ => true,
        };
        let carrier = Carrier::Edge(&edge.curve);
        let samples = sample(&carrier, edge.t0, edge.t1, curve_initial(&edge.curve), frame, placed);
        if samples.len() < 2 {
            continue;
        }
        let bounds = bounds_of(&samples.paper);
        out.push(Candidate {
            carrier,
            samples,
            inked,
            silhouette: false,
            edge: Some((solid_index, edge.id)),
            ends: Some(((solid_index, edge.start_vertex_id), (solid_index, edge.end_vertex_id))),
            bounds,
        });
    }
    out
}

/// Whether a face is flat: a recognized plane, or an affine bilinear patch.
fn planar(surface: &NurbsSurface) -> bool {
    matches!(surface.analytic(), Some(AnalyticSurface::Plane { .. })) || surface.is_affine().unwrap_or(false)
}

/// The uv box of a face's trim: the control points of every loop's pcurves,
/// clamped to the carrier's domain (a pcurve lies in its own control hull).
fn trim_box(face: &FaceRecord) -> Option<[f64; 4]> {
    let du = face.surface.domain_u().ok()?;
    let dv = face.surface.domain_v().ok()?;
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for lp in &face.loops {
        for coedge in &lp.coedges {
            for cp in &coedge.pcurve.control_points {
                let w = if cp.w != 0.0 { cp.w } else { 1.0 };
                b[0] = b[0].min(cp.x / w);
                b[1] = b[1].min(cp.y / w);
                b[2] = b[2].max(cp.x / w);
                b[3] = b[3].max(cp.y / w);
            }
        }
    }
    if !(b[0] <= b[2]) {
        return Some([du[0], dv[0], du[1], dv[1]]);
    }
    Some([b[0].max(du[0]), b[1].max(dv[0]), b[2].min(du[1]), b[3].min(dv[1])])
}

/// What a pass learns about every trim of one solid that no camera, scale or
/// clip changes: each face's [`TrimTests`], `[shell][face]`. The projection
/// cache keeps one per solid it draws ([`crate::sheets::project`]), so a
/// re-projection — another scale, a section or a detail of the same part —
/// asks nothing a previous one built.
pub struct SolidTrims {
    faces: Vec<Vec<TrimTests>>,
}

impl SolidTrims {
    pub fn of(solid: &BrepSolid) -> SolidTrims {
        SolidTrims { faces: solid.shells.iter().map(|shell| shell.faces.iter().map(|_| TrimTests::new()).collect()).collect() }
    }
}

/// What a pass learns about a face's trim, so the kernel is asked only what
/// nothing cheaper can answer: which of the kernel's containment lanes the face
/// is in ([`containment_lane`]), its [`UvReach`], and — the first time a
/// question gets past that — its [`UvTrim`]. All are read off the face alone,
/// so they hold for any view of it, and each is learned the first time a
/// question about the face needs it.
struct TrimTests {
    lane: std::cell::OnceCell<Option<ContainmentLane>>,
    reach: std::cell::OnceCell<Option<UvReach>>,
    uv: std::cell::OnceCell<Option<UvTrim>>,
}

impl TrimTests {
    fn new() -> TrimTests {
        TrimTests { lane: std::cell::OnceCell::new(), reach: std::cell::OnceCell::new(), uv: std::cell::OnceCell::new() }
    }

    /// The kernel's lane for the face; `None` where preparing it fails, where
    /// the kernel's own answer is an error and nothing answers ahead of it.
    fn lane(&self, face: &FaceRecord) -> Option<&ContainmentLane> {
        self.lane.get_or_init(|| containment_lane(face).ok()).as_ref()
    }

    /// The face's [`UvReach`].
    fn reach(&self, face: &FaceRecord) -> Option<&UvReach> {
        self.reach.get_or_init(|| self.lane(face).and_then(|lane| UvReach::of(face, lane))).as_ref()
    }

    /// The face's [`UvTrim`], built the first time the kernel would be asked.
    fn uv_trim(&self, face: &FaceRecord) -> Option<&UvTrim> {
        self.uv
            .get_or_init(|| {
                let reach = self.reach(face)?;
                UvTrim::of(face, reach, self.lane(face)?)
            })
            .as_ref()
    }
}

/// A face's trim in its carrier's parameters, as the kernel's containment
/// test reads it, so that a point clear of its chords is classified here and
/// only a point near them is asked of the kernel. Two of the kernel's lanes
/// are mirrored; any other face has none.
///
/// **The plain lane** — the kernel's generic scan, for a face
/// [`containment_lane`] puts there without the sphere chart — samples each
/// pcurve at the kernel's [`trim_sample_count`] stations, [`trim_station`]
/// apart (`max(2, (k + 1)(p + 1)·4)`, `k` the distinct interior knots), and
/// answers `Boundary` (inside) within its band
/// of a chord, counts crossings of that polygon farther than one chord's length
/// from the nearest chord, and takes the side of the pcurve itself nearer than
/// that. This polygon's vertices are a subset of those stations — one per
/// power-of-two stride, halved on the kernel's own stations wherever the
/// chord's middle sits off it by more than the kernel's band at the trim's
/// middle — and `margin` is the largest such middle's distance off its chord
/// (and the gaps where one pcurve's end meets the next one's start). The
/// kernel's polygon and the pcurves both lie within that of this one, so a
/// point farther than the kernel's band plus `margin` from every chord is
/// outside that band of the kernel's polygon and on the same side of it as of
/// this one and of the pcurves: whichever rule the kernel uses there, its
/// answer is this polygon's even-odd count. The side rule assumes the loops
/// run with the material to the left of each pcurve on a same-sense face (to
/// the right otherwise); a face whose loops do not is left to the kernel.
///
/// **The seam-horizon lane**, for a face [`containment_lane`] puts in the
/// kernel's wrapped-horizon lane: the loops the kernel published, unwrapped
/// sample to sample, `Boundary` within the band of a chord at any of the
/// query's three period images, otherwise each loop's even-odd count summed
/// over the images, mod 2. Here the polygon IS the kernel's, every station
/// kept, and a point is asked of the kernel within twice its band of a chord at
/// any image. With the kernel's per-image fallback switched on
/// ([`brep_kernel::horizon_cross_frame`] false) it is not built.
///
/// Not built for a face in any other lane: the seam band, the sphere cap, the
/// covering rim strip, or the generic scan where the sphere chart substitutes
/// for its parity count.
struct UvTrim {
    /// `(from, to, loop)`.
    segments: Vec<([f64; 2], [f64; 2], usize)>,
    loops: usize,
    grid: Grid,
    /// How far beyond the kernel's band a point must be from every chord.
    margin: Margin,
    /// The period images the query is counted at.
    images: Vec<f64>,
}

#[derive(Debug, Clone, Copy)]
enum Margin {
    /// Plus this, in the carrier's parameters.
    Plus(f64),
    /// The kernel's band again.
    Twice,
}

impl UvTrim {
    fn of(face: &FaceRecord, reach: &UvReach, lane: &ContainmentLane) -> Option<UvTrim> {
        match lane {
            ContainmentLane::WrappedHorizon { loops, u_period, cross_frame: true } => {
                let loops = loops.iter().map(|lp| lp.iter().map(|p| [p.x, p.y]).collect()).collect();
                return Some(UvTrim::grid_of(loops, Margin::Twice, vec![-u_period, 0.0, *u_period]));
            }
            ContainmentLane::Generic { sphere_chart: false } => {}
            _ => return None,
        }
        let surface = &face.surface;
        // Chords no coarser than the band the kernel asks with at the trim's
        // middle, where the curve allows it.
        let centre = [0.5 * (reach.lo[0] + reach.hi[0]), 0.5 * (reach.lo[1] + reach.hi[1])];
        let fine = trim_band(surface, centre, KernelTolerances::default().pcurve_consistency);
        let mut rings: Vec<Vec<[f64; 2]>> = Vec::with_capacity(face.loops.len());
        let mut sag: f64 = 0.0;
        for lp in &face.loops {
            let mut ring = Vec::new();
            for coedge in &lp.coedges {
                let curve = &coedge.pcurve;
                let [start, end] = curve.domain().ok()?;
                let count = trim_sample_count(curve);
                let station = |i: usize| if i < count { trim_station(start, end, i, count) } else { end };
                let at = |i: usize| -> Option<[f64; 2]> { curve.evaluate(station(i)).ok().map(|p| [p.x, p.y]) };
                // Every station ours is one of the kernel's, the coarsest a
                // power-of-two stride that still gives two chords.
                let mut stride = 1;
                while count % (stride * 2) == 0 && count / (stride * 2) >= 2 {
                    stride *= 2;
                }
                let mut a = at(0)?;
                // The kernel's polygon runs from a pcurve's last station
                // straight to the next one's first; both polygons meet the
                // pcurves' own ends only to the gap between them.
                if let Some(previous) = ring.last() {
                    sag = sag.max(paper_distance(*previous, a));
                }
                for from in (0..count).step_by(stride) {
                    let to = from + stride;
                    let b = at(to)?;
                    // Halve on the kernel's stations while the middle sits
                    // off the chord by more than `fine`; a single kernel
                    // chord is measured at its parameter middle.
                    let mut pending = vec![(from, a, to, b)];
                    while let Some((i, pi, j, pj)) = pending.pop() {
                        if j - i >= 2 {
                            let m = (i + j) / 2;
                            let pm = at(m)?;
                            let off = segment_foot(pm, pi, pj).0;
                            if off > fine {
                                pending.push((m, pm, j, pj));
                                pending.push((i, pi, m, pm));
                                continue;
                            }
                            sag = sag.max(off);
                        } else {
                            let pm = curve.evaluate(0.5 * (station(i) + station(j))).ok()?;
                            sag = sag.max(segment_foot([pm.x, pm.y], pi, pj).0);
                        }
                        ring.push(pi);
                    }
                    a = b;
                }
                ring.push(a);
            }
            if let (Some(first), Some(last)) = (ring.first(), ring.last()) {
                sag = sag.max(paper_distance(*first, *last));
            }
            if ring.len() >= 2 {
                rings.push(ring);
            }
        }
        let area = |ring: &[[f64; 2]]| -> f64 {
            (0..ring.len()).map(|i| {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                a[0] * b[1] - b[0] * a[1]
            }).sum()
        };
        let areas: Vec<f64> = rings.iter().map(|ring| area(ring)).collect();
        let outer = (0..rings.len()).max_by(|&i, &j| areas[i].abs().partial_cmp(&areas[j].abs()).unwrap_or(std::cmp::Ordering::Equal))?;
        let left = if face.same_sense { 1.0 } else { -1.0 };
        for (index, a) in areas.iter().enumerate() {
            let expected = if index == outer { left } else { -left };
            if !(a * expected > 0.0) {
                return None;
            }
        }
        Some(UvTrim::grid_of(rings, Margin::Plus(sag), vec![0.0]))
    }

    fn grid_of(rings: Vec<Vec<[f64; 2]>>, margin: Margin, images: Vec<f64>) -> UvTrim {
        let mut segments = Vec::new();
        for (index, ring) in rings.iter().enumerate() {
            for i in 0..ring.len() {
                segments.push((ring[i], ring[(i + 1) % ring.len()], index));
            }
        }
        let mut extent = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        let boxed = |(a, b, _): &([f64; 2], [f64; 2], usize)| [a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])];
        for segment in &segments {
            let g = boxed(segment);
            extent = [extent[0].min(g[0]), extent[1].min(g[1]), extent[2].max(g[2]), extent[3].max(g[3])];
        }
        let mut grid = Grid::new(extent, segments.len().max(1));
        for (index, segment) in segments.iter().enumerate() {
            grid.insert(boxed(segment), index);
        }
        UvTrim { segments, loops: rings.len(), grid, margin, images }
    }

    /// Inside the trim, or `None` near enough a chord that only the kernel
    /// can say, `band` being the band the kernel is asked with.
    fn covers(&self, uv: [f64; 2], band: f64) -> Option<bool> {
        let near = match self.margin {
            Margin::Plus(sag) => band + sag,
            Margin::Twice => 2.0 * band,
        };
        let mut odd = vec![0u32; self.loops];
        for shift in &self.images {
            let image = [uv[0] + shift, uv[1]];
            let (x0, y0, x1, y1) = self.grid.range([image[0] - near, image[1] - near, image[0] + near, image[1] + near]);
            for y in y0..=y1 {
                for x in x0..=x1 {
                    for &index in &self.grid.buckets[y * self.grid.width + x] {
                        let (a, b, _) = self.segments[index];
                        if segment_foot(image, a, b).0 <= near {
                            return None;
                        }
                    }
                }
            }
            // The lane's own count: crossings of +u, half-open in v, per loop.
            let (column, row, _, _) = self.grid.range([image[0], image[1], image[0], image[1]]);
            let mut inside = vec![false; self.loops];
            for x in column..self.grid.width {
                for &index in &self.grid.buckets[row * self.grid.width + x] {
                    let (a, b, ring) = self.segments[index];
                    if (a[1] > image[1]) == (b[1] > image[1]) {
                        continue;
                    }
                    let crossing = a[0] + (image[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                    if crossing > image[0] && self.grid.range([crossing, image[1], crossing, image[1]]).0 == x {
                        inside[ring] = !inside[ring];
                    }
                }
            }
            for (ring, inside) in inside.into_iter().enumerate() {
                odd[ring] += inside as u32;
            }
        }
        Some(odd.iter().map(|hits| hits % 2).sum::<u32>() % 2 == 1)
    }
}

/// A flat face's trim on the paper: its loops closed from the paper samples of
/// its boundary edges, each segment signed by its loop's orientation (the
/// loop of largest projected area is the outer one, the rest are holes) and
/// bucketed on a grid.
///
/// A flat face that is not edge-on maps onto the paper one to one, so the view
/// ray through a paper point meets it inside its trim exactly when the
/// projected loops wind round the point. The projected loops are not the trim,
/// and differ from it by at most the band [`PaperTrim::covers`] keeps clear of
/// them: [`CHORD_MM`], the samples' chords against the exact edge curves, plus
/// the crossing floor twice — once for an edge curve sitting the B-rep bar off
/// the trim its pcurves draw (and a loop closed across two edges' ends that
/// agree only to that bar), once for the band the kernel's trim test counts as
/// inside.
///
/// It answers ONE way. A point the loops wind round by more than the band is
/// inside the trim, and the kernel's test says so too: within its band of its
/// chord polygon it answers inside, near the pcurve it takes the pcurve's own
/// side, and farther out its polygon's count agrees with the pcurve's. A point
/// the loops do not wind round is NOT reliably outside for the kernel: it
/// counts anything within its band of its own chords as inside, and a chord
/// can sit well inside a rim it samples coarsely — measured on `booleanIssue`,
/// where a hole's circle is sampled at twelve stations and a point 0.023 paper
/// mm into the hole reads `Boundary`. So "not covered" goes to the trim test in
/// the carrier's parameters ([`UvTrim`], then the kernel).
///
/// This is what made the pass fast: one flat face of `gear-hex-bore-push`
/// carries 562 boundary edges and 36 530 pcurve control points, and the
/// kernel's test scans every sample of that trim for every point it is asked
/// about — 37 of the pass's 43 seconds.
#[derive(Debug, Clone)]
struct PaperTrim {
    /// `(from, to, sign)`: the sign turns every loop to the outer-positive,
    /// holes-negative orientation.
    segments: Vec<([f64; 2], [f64; 2], i32)>,
    /// Each segment, in every cell its box grown by the band touches.
    grid: Grid,
    band: f64,
}

impl PaperTrim {
    /// `None` when a boundary edge has no samples (a seam has none), or a loop
    /// does not close on the paper within the bar the B-rep keeps its edges'
    /// ends to.
    fn of(solid: &BrepSolid, face: &FaceRecord, paper_of_edge: &HashMap<u64, &[[f64; 2]]>, floors: &Floors) -> Option<PaperTrim> {
        let band = CHORD_MM + 2.0 * floors.crossing_mm;
        let gap = 2.0 * floors.crossing_mm + CHORD_MM;
        let mut rings: Vec<Vec<[f64; 2]>> = Vec::with_capacity(face.loops.len());
        for lp in &face.loops {
            let mut pieces: Vec<&[[f64; 2]]> = Vec::with_capacity(lp.coedges.len());
            for coedge in &lp.coedges {
                match paper_of_edge.get(&coedge.edge_id) {
                    Some(paper) if paper.len() >= 2 => pieces.push(paper),
                    _ if solid.edges.iter().any(|edge| edge.id == coedge.edge_id && edge.degenerate) => {}
                    _ => return None,
                }
            }
            let Some(first) = pieces.first() else { continue };
            // Walk the loop, each edge's samples the way round that starts
            // nearest the previous one's end; the first is turned to end
            // nearest the second.
            let ends = |piece: &[[f64; 2]]| (piece[0], piece[piece.len() - 1]);
            let first_forward = pieces.get(1).is_none_or(|next| {
                let (n0, n1) = ends(next);
                let near = |p: [f64; 2]| paper_distance(p, n0).min(paper_distance(p, n1));
                near(ends(first).1) <= near(ends(first).0)
            });
            let mut ring: Vec<[f64; 2]> = if first_forward { first.to_vec() } else { first.iter().rev().copied().collect() };
            for piece in &pieces[1..] {
                let last = ring[ring.len() - 1];
                let (p0, p1) = ends(piece);
                let forward = paper_distance(last, p0) <= paper_distance(last, p1);
                if paper_distance(last, if forward { p0 } else { p1 }) > gap {
                    return None;
                }
                if forward {
                    ring.extend_from_slice(piece);
                } else {
                    ring.extend(piece.iter().rev());
                }
            }
            if paper_distance(ring[0], ring[ring.len() - 1]) > gap {
                return None;
            }
            rings.push(ring);
        }
        // Twice the signed area of each ring, and the outer one.
        let areas: Vec<f64> = rings
            .iter()
            .map(|ring| (0..ring.len()).map(|i| {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                a[0] * b[1] - b[0] * a[1]
            }).sum())
            .collect();
        let outer = (0..rings.len()).max_by(|&i, &j| areas[i].abs().partial_cmp(&areas[j].abs()).unwrap_or(std::cmp::Ordering::Equal))?;
        let mut segments: Vec<([f64; 2], [f64; 2], i32)> = Vec::new();
        for (index, ring) in rings.iter().enumerate() {
            let turned = if areas[index] < 0.0 { -1 } else { 1 };
            let sign = if index == outer { turned } else { -turned };
            segments.extend((0..ring.len()).map(|i| (ring[i], ring[(i + 1) % ring.len()], sign)));
        }
        let mut extent = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        let grown = |(a, b, _): &([f64; 2], [f64; 2], i32)| {
            [a[0].min(b[0]) - band, a[1].min(b[1]) - band, a[0].max(b[0]) + band, a[1].max(b[1]) + band]
        };
        for segment in &segments {
            let g = grown(segment);
            extent = [extent[0].min(g[0]), extent[1].min(g[1]), extent[2].max(g[2]), extent[3].max(g[3])];
        }
        let mut grid = Grid::new(extent, segments.len());
        for (index, segment) in segments.iter().enumerate() {
            grid.insert(grown(segment), index);
        }
        Some(PaperTrim { segments, grid, band })
    }

    /// Whether the face covers paper point `p` — its projected loops wind
    /// round it — or `None` within the band of them, where only the kernel
    /// can say.
    fn covers(&self, p: [f64; 2]) -> Option<bool> {
        // A segment within the band has its grown box round `p`, so it is in
        // `p`'s own cell.
        if self.grid.at(p).iter().any(|&index| {
            let (a, b, _) = self.segments[index];
            segment_foot(p, a, b).0 <= self.band
        }) {
            return None;
        }
        // Signed crossings along +x, half-open in y. A segment the ray
        // crosses is in the cell of the crossing, and is counted there only.
        let (column, row, _, _) = self.grid.range([p[0], p[1], p[0], p[1]]);
        let mut winding = 0;
        for x in column..self.grid.width {
            for &index in &self.grid.buckets[row * self.grid.width + x] {
                let (a, b, sign) = self.segments[index];
                if (a[1] > p[1]) == (b[1] > p[1]) {
                    continue;
                }
                let crossing = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                if crossing > p[0] && self.grid.range([crossing, p[1], crossing, p[1]]).0 == x {
                    winding += if b[1] > p[1] { sign } else { -sign };
                }
            }
        }
        Some(winding != 0)
    }
}

/// The uv coincidence band a trim is asked with: the B-rep bar carried into
/// the carrier's parameters by its own speed at `uv`.
fn trim_band(surface: &NurbsSurface, uv: [f64; 2], bar: f64) -> f64 {
    let speed = surface
        .derivatives(uv[0], uv[1], 1)
        .ok()
        .map(|d| d[1][0].length().max(d[0][1].length()))
        .unwrap_or(0.0);
    if speed > 1e-12 {
        bar / speed
    } else {
        1e-9
    }
}

/// Inside (or on) a face's trim: the kernel's `parameter_point_in_face`,
/// unless what the pass has learned about the trim ([`TrimTests`]) answers
/// first.
fn in_face(face: &FaceRecord, tests: Option<&TrimTests>, uv: [f64; 2], bar: f64) -> bool {
    let band = trim_band(&face.surface, uv, bar);
    if let Some(tests) = tests {
        if tests.reach(face).is_some_and(|reach| reach.outside(uv, band)) {
            return false;
        }
        if let Some(inside) = tests.uv_trim(face).and_then(|trim| trim.covers(uv, band)) {
            return inside;
        }
    }
    TRIM_QUERIES.with(|count| count.set(count.get() + 1));
    !matches!(
        parameter_point_in_face(face, Vec2 { x: uv[0], y: uv[1] }, band),
        Ok(PolygonClass::Outside) | Err(_)
    )
}

/// Where in its carrier's parameters a face's trim can put material: the box
/// of every loop's pcurve control points — a pcurve with positive weights
/// lies in the hull of its own — read modulo the period along a closed
/// direction. A point outside that box by more than the band the trim is
/// asked with is outside every loop and within the band of none, so the
/// kernel's answer there is `Outside` and is not asked for.
///
/// It is the containment question's cheap half: a silhouette generator of a
/// small cylinder patch, or a view ray through its carrier far from the
/// patch, is decided without sampling the trim.
///
/// The box holds for the lanes that answer inside the loops' own polygons: the
/// generic scan without the sphere chart and the wrapped horizon.
#[derive(Debug, Clone)]
struct UvReach {
    lo: [f64; 2],
    hi: [f64; 2],
    period: [Option<f64>; 2],
}

impl UvReach {
    /// `None` where the kernel's lane ([`containment_lane`]) can put material
    /// outside the pcurve box: the sphere cap, which holds a pole the loop does
    /// not enclose in uv; the sphere chart, which decides on the ball; the seam
    /// band, which reads a band of punctures as all material; and the covering
    /// rim strip, whose rims need only span the period to within 2%, so a
    /// point below both can count one crossing.
    fn of(face: &FaceRecord, lane: &ContainmentLane) -> Option<UvReach> {
        if !matches!(lane, ContainmentLane::Generic { sphere_chart: false } | ContainmentLane::WrappedHorizon { .. }) {
            return None;
        }
        let surface = &face.surface;
        let closed = surface.closed_directions().ok()?;
        let (du, dv) = (surface.domain_u().ok()?, surface.domain_v().ok()?);
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for cp in face.loops.iter().flat_map(|lp| &lp.coedges).flat_map(|coedge| &coedge.pcurve.control_points) {
            if !(cp.w > 0.0) {
                return None;
            }
            let (u, v) = (cp.x / cp.w, cp.y / cp.w);
            lo = [lo[0].min(u), lo[1].min(v)];
            hi = [hi[0].max(u), hi[1].max(v)];
        }
        if !(lo[0] <= hi[0] && lo[1] <= hi[1]) {
            return None;
        }
        let period = [closed.0.then_some(du[1] - du[0]), closed.1.then_some(dv[1] - dv[0])];
        Some(UvReach { lo, hi, period })
    }

    fn outside(&self, uv: [f64; 2], band: f64) -> bool {
        (0..2).any(|k| {
            let (lo, hi) = (self.lo[k] - band, self.hi[k] + band);
            match self.period[k] {
                None => uv[k] < lo || uv[k] > hi,
                Some(period) if period > 0.0 && hi - lo < period => {
                    uv[k] - ((uv[k] - lo) / period).floor() * period > hi
                }
                Some(_) => false,
            }
        })
    }
}

/// The uv of a model point that lies ON `face`'s carrier (within `bar`).
fn uv_on(face: &FaceRecord, p: Vec3, bar: f64) -> Option<[f64; 2]> {
    let foot = project_point_to_surface(&face.surface, p).ok()?;
    (foot.distance <= bar).then_some([foot.u, foot.v])
}

/// The silhouette curves of one face, trimmed to it.
/// The silhouette curves of one face, trimmed to it.
fn silhouette_candidates<'a>(
    face: &'a FaceRecord,
    tests: &TrimTests,
    frame: &ViewFrame,
    placed: &PlacedView,
    floors: &Floors,
) -> Vec<Candidate<'a>> {
    let surface = &face.surface;
    if planar(surface) {
        return Vec::new();
    }
    let dir = v3(frame.forward);
    let mut carriers: Vec<(Carrier<'a>, f64, f64, usize)> = Vec::new();
    match surface.analytic() {
        Some(AnalyticSurface::RuledRevolution { frame: rev, rho0, rho1, height }) => {
            // n ∝ h·r(θ) − (ρ1 − ρ0)·axis, so n·d = 0 is A cos θ + B sin θ = C.
            let a = height * dir.dot(rev.x_axis);
            let b = height * dir.dot(rev.y_axis);
            let c = (rho1 - rho0) * dir.dot(rev.axis);
            let r = (a * a + b * b).sqrt();
            if r > 1e-12 * (height.abs() + 1.0) && c.abs() <= r {
                let base = b.atan2(a);
                let spread = (c / r).clamp(-1.0, 1.0).acos();
                let mut angles = vec![base + spread];
                if spread > 1e-12 {
                    angles.push(base - spread);
                }
                for theta in angles {
                    let radial = rev.x_axis.scale(theta.cos()).add(rev.y_axis.scale(theta.sin()));
                    let start = rev.origin.add(radial.scale(*rho0));
                    let end = rev.origin.add(rev.axis.scale(*height)).add(radial.scale(*rho1));
                    carriers.push((Carrier::Line { a: start, b: end }, 0.0, 1.0, 1));
                }
            }
        }
        Some(AnalyticSurface::Sphere { frame: rev, radius }) => {
            let Ok(e1) = dir.perpendicular().and_then(|e| e.normalized()) else { return Vec::new() };
            let e2 = dir.cross(e1);
            carriers.push((
                Carrier::Circle { centre: rev.origin, e1: e1.scale(*radius), e2: e2.scale(*radius) },
                0.0,
                std::f64::consts::TAU,
                16,
            ));
        }
        _ => {
            let traced = trace_silhouette(face, dir);
            for uv in traced {
                let n = uv.len();
                if n < 2 {
                    continue;
                }
                carriers.push((Carrier::Traced { surface, uv, dir }, 0.0, (n - 1) as f64, n - 1));
            }
        }
    }
    let mut out = Vec::new();
    for (carrier, s0, s1, initial) in carriers {
        let samples = sample(&carrier, s0, s1, initial, frame, placed);
        let spans_found = trimmed_spans(&carrier, &samples, face, tests, floors.bar);
        for (lo, hi) in spans_found {
            let piece = sample(&carrier, lo, hi, initial_between(&samples, lo, hi), frame, placed);
            if piece.len() < 2 || piece.paper_length() < floors.crossing_mm {
                continue;
            }
            let bounds = bounds_of(&piece.paper);
            out.push(Candidate {
                carrier: carrier.clone(),
                samples: piece,
                inked: true,
                silhouette: true,
                edge: None,
                ends: None,
                bounds,
            });
        }
    }
    out
}

/// How many initial intervals a re-sample of `[lo, hi]` needs: as many as the
/// full sampling had there, so the chord test starts no coarser.
fn initial_between(samples: &Samples, lo: f64, hi: f64) -> usize {
    samples.params.iter().filter(|s| **s > lo && **s < hi).count() + 1
}

/// The parameter spans of a sampled silhouette that lie on its face, each
/// transition pinned by bisection on the exact curve.
fn trimmed_spans(carrier: &Carrier, samples: &Samples, face: &FaceRecord, tests: &TrimTests, bar: f64) -> Vec<(f64, f64)> {
    // The chord samples, and never fewer than TRIM_STATIONS: a straight
    // generator is two chord samples, and a trim it enters and leaves between
    // them would be invisible to its ends.
    let (s0, s1) = (samples.params[0], *samples.params.last().unwrap());
    let mut stations: Vec<f64> = (0..=TRIM_STATIONS).map(|k| s0 + (s1 - s0) * k as f64 / TRIM_STATIONS as f64).collect();
    stations.extend(samples.params.iter().copied());
    stations.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    stations.dedup();
    let inside = |s: f64| -> bool {
        let uv = match carrier {
            Carrier::Traced { surface, uv, dir } => traced_at(surface, uv, *dir, s).map(|(_, uv)| uv),
            _ => carrier.at(s).and_then(|p| uv_on(face, p, bar)),
        };
        uv.is_some_and(|uv| in_face(face, Some(tests), uv, bar))
    };
    let flags: Vec<bool> = stations.iter().map(|s| inside(*s)).collect();
    let mut spans = Vec::new();
    let mut start: Option<f64> = None;
    for i in 0..flags.len() {
        if i > 0 && flags[i] != flags[i - 1] {
            let (mut lo, mut hi) = (stations[i - 1], stations[i]);
            let lo_flag = flags[i - 1];
            for _ in 0..BISECTION_STEPS {
                let mid = 0.5 * (lo + hi);
                if inside(mid) == lo_flag {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let cut = 0.5 * (lo + hi);
            if flags[i] {
                start = Some(cut);
            } else if let Some(s) = start.take() {
                spans.push((s, cut));
            }
        } else if i == 0 && flags[0] {
            start = Some(stations[0]);
        }
    }
    if let (Some(s), Some(last)) = (start, stations.last()) {
        spans.push((s, *last));
    }
    spans
}

/// The zero set of `n · d` over a face's uv box, as uv polylines: marching
/// squares on a grid of the carrier's own normals, each crossing pinned on
/// its grid edge by bisection, the crossings chained cell to cell.
fn trace_silhouette(face: &FaceRecord, dir: Vec3) -> Vec<Vec<[f64; 2]>> {
    let surface = &face.surface;
    let Some(uv_box) = trim_box(face) else { return Vec::new() };
    let spans = |knots: &[f64], degree: usize| knots.len().saturating_sub(2 * degree + 1).max(1);
    let nu = (spans(&surface.knots_u, surface.degree_u) * (surface.degree_u + 1) * 2).clamp(12, 96);
    let nv = (spans(&surface.knots_v, surface.degree_v) * (surface.degree_v + 1) * 2).clamp(12, 96);
    let at = |i: usize, j: usize| {
        [
            uv_box[0] + (uv_box[2] - uv_box[0]) * i as f64 / nu as f64,
            uv_box[1] + (uv_box[3] - uv_box[1]) * j as f64 / nv as f64,
        ]
    };
    let mut values = vec![None; (nu + 1) * (nv + 1)];
    for j in 0..=nv {
        for i in 0..=nu {
            let p = at(i, j);
            values[j * (nu + 1) + i] = grazing(surface, dir, p[0], p[1]);
        }
    }
    let value = |i: usize, j: usize| values[j * (nu + 1) + i];
    // A crossing on a grid edge, keyed by the edge: (i, j, horizontal).
    let mut nodes: HashMap<(usize, usize, bool), [f64; 2]> = HashMap::new();
    let mut crossing = |i: usize, j: usize, horizontal: bool| -> Option<(usize, usize, bool)> {
        let (i2, j2) = if horizontal { (i + 1, j) } else { (i, j + 1) };
        let (g0, g1) = (value(i, j)?, value(i2, j2)?);
        if (g0 < 0.0) == (g1 < 0.0) {
            return None;
        }
        let key = (i, j, horizontal);
        if !nodes.contains_key(&key) {
            let (a, b) = (at(i, j), at(i2, j2));
            let (mut lo, mut hi, mut g_lo) = (0.0, 1.0, g0);
            for _ in 0..BISECTION_STEPS {
                let mid = 0.5 * (lo + hi);
                let p = [a[0] + (b[0] - a[0]) * mid, a[1] + (b[1] - a[1]) * mid];
                let Some(g) = grazing(surface, dir, p[0], p[1]) else { break };
                if (g < 0.0) == (g_lo < 0.0) {
                    lo = mid;
                    g_lo = g;
                } else {
                    hi = mid;
                }
            }
            let t = 0.5 * (lo + hi);
            nodes.insert(key, [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
        }
        Some(key)
    };
    let mut links: Vec<[(usize, usize, bool); 2]> = Vec::new();
    for j in 0..nv {
        for i in 0..nu {
            let edges = [
                crossing(i, j, true),
                crossing(i + 1, j, false),
                crossing(i, j + 1, true),
                crossing(i, j, false),
            ];
            let hits: Vec<_> = edges.iter().flatten().copied().collect();
            match hits.len() {
                2 => links.push([hits[0], hits[1]]),
                4 => {
                    // A saddle: pair by the centre's sign, the usual rule.
                    let centre = at(i, j);
                    let half = [
                        centre[0] + (uv_box[2] - uv_box[0]) * 0.5 / nu as f64,
                        centre[1] + (uv_box[3] - uv_box[1]) * 0.5 / nv as f64,
                    ];
                    let g = grazing(surface, dir, half[0], half[1]).unwrap_or(0.0);
                    let corner = value(i, j).unwrap_or(0.0);
                    if (g < 0.0) == (corner < 0.0) {
                        links.push([hits[0], hits[1]]);
                        links.push([hits[2], hits[3]]);
                    } else {
                        links.push([hits[0], hits[3]]);
                        links.push([hits[1], hits[2]]);
                    }
                }
                _ => {}
            }
        }
    }
    // Chain the links through shared crossings.
    let mut incident: HashMap<(usize, usize, bool), Vec<usize>> = HashMap::new();
    for (index, link) in links.iter().enumerate() {
        incident.entry(link[0]).or_default().push(index);
        incident.entry(link[1]).or_default().push(index);
    }
    let mut used = vec![false; links.len()];
    let mut order: Vec<usize> = (0..links.len()).collect();
    // Start from an END (a crossing one link touches) so open curves keep
    // their ends; what is left is closed.
    order.sort_by_key(|&i| {
        let open = incident[&links[i][0]].len() == 1 || incident[&links[i][1]].len() == 1;
        (!open, i)
    });
    let mut out = Vec::new();
    for start in order {
        if used[start] {
            continue;
        }
        used[start] = true;
        let [a, b] = links[start];
        let (head, mut tail) = if incident[&a].len() == 1 { (a, b) } else { (b, a) };
        let mut path = vec![head, tail];
        while let Some(next) = incident[&tail].iter().copied().find(|i| !used[*i]) {
            used[next] = true;
            tail = if links[next][0] == tail { links[next][1] } else { links[next][0] };
            path.push(tail);
            if tail == head {
                break;
            }
        }
        out.push(path.into_iter().map(|key| nodes[&key]).collect());
    }
    out
}

// ---------------------------------------------------------------------------
// Occluders
// ---------------------------------------------------------------------------

/// Something that can hide a piece, and its box on the paper.
struct Occluder<'a> {
    cover: Cover<'a>,
    bounds: [f64; 4],
}

/// What an [`Occluder`] is.
enum Cover<'a> {
    /// A B-rep face: the exact pass.
    Face {
        face: &'a FaceRecord,
        /// `(point, unit normal)` of a flat face, for the closed-form ray hit.
        plane: Option<(Vec3, Vec3)>,
        /// The carrier's per-SPAN control hulls that meet the trim's uv box,
        /// each as its paper box, its projected convex hull and its nearest
        /// view depth: a NURBS patch lies inside the hull of the control points
        /// of each of its knot spans, and a projection of a convex hull is the
        /// convex hull of the projections, so a point outside every hull — or
        /// in front of every hull that holds it — is not covered by the face.
        /// Much tighter than one hull for a lofted face, and than a box for a
        /// slanted one.
        spans: Vec<SpanHull>,
        /// What the pass has learned about the trim.
        tests: &'a TrimTests,
        /// A flat face's trim on the paper, when its loops close there.
        trim: Option<PaperTrim>,
    },
    /// A triangle of a display mesh: the mesh approximation
    /// ([`visible_mesh_lines`]). Its corners and unit normal.
    Triangle { corners: [Vec3; 3], normal: Vec3 },
}

/// The paper box a face can cover: its boundary and silhouettes when those
/// were computed in closed form, otherwise the carrier's control hull (a
/// NURBS patch lies inside it, so the box is never short).
fn face_bounds(
    face: &FaceRecord,
    boundary: &[[f64; 4]],
    silhouettes: &[[f64; 4]],
    frame: &ViewFrame,
    placed: &PlacedView,
) -> [f64; 4] {
    let exact_outline = planar(&face.surface)
        || matches!(face.surface.analytic(), Some(AnalyticSurface::RuledRevolution { .. } | AnalyticSurface::Sphere { .. }));
    let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    let mut grow = |c: [f64; 4]| {
        b[0] = b[0].min(c[0]);
        b[1] = b[1].min(c[1]);
        b[2] = b[2].max(c[2]);
        b[3] = b[3].max(c[3]);
    };
    if exact_outline && !boundary.is_empty() {
        for c in boundary.iter().chain(silhouettes) {
            grow(*c);
        }
    } else {
        for row in &face.surface.control_points {
            for cp in row {
                let w = if cp.w != 0.0 { cp.w } else { 1.0 };
                let (paper, _) = project(frame, placed, Vec3::new(cp.x / w, cp.y / w, cp.z / w));
                grow([paper[0], paper[1], paper[0], paper[1]]);
            }
        }
    }
    b
}

/// One knot span's control hull on the paper.
struct SpanHull {
    bounds: [f64; 4],
    /// The projected control points' convex hull, counter-clockwise.
    hull: Vec<[f64; 2]>,
    /// The nearest view depth of its control points.
    near: f64,
}

impl SpanHull {
    /// Whether `p` is inside the hull or within `pad` of it.
    fn holds(&self, p: [f64; 2], pad: f64) -> bool {
        let b = self.bounds;
        if p[0] < b[0] - pad || p[0] > b[2] + pad || p[1] < b[1] - pad || p[1] > b[3] + pad {
            return false;
        }
        let n = self.hull.len();
        if n >= 3 && (0..n).all(|k| {
            let (a, c) = (self.hull[k], self.hull[(k + 1) % n]);
            (c[0] - a[0]) * (p[1] - a[1]) - (c[1] - a[1]) * (p[0] - a[0]) >= 0.0
        }) {
            return true;
        }
        (0..n.max(1)).any(|k| segment_foot(p, self.hull[k], self.hull[(k + 1) % n]).0 <= pad)
    }
}

/// The convex hull of paper points, counter-clockwise (monotone chain).
fn convex_hull(mut points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    points.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    let turn = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut hull: Vec<[f64; 2]> = Vec::with_capacity(points.len() + 1);
    for pass in 0..2 {
        let start = hull.len();
        let ordered: Box<dyn Iterator<Item = &[f64; 2]>> =
            if pass == 0 { Box::new(points.iter()) } else { Box::new(points.iter().rev()) };
        for &p in ordered {
            while hull.len() >= start + 2 && turn(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

/// The control hull on the paper of each knot span of `face`'s carrier, for
/// the spans whose parameter interval meets the trim's uv box.
fn span_hulls(face: &FaceRecord, frame: &ViewFrame, placed: &PlacedView) -> Vec<SpanHull> {
    let surface = &face.surface;
    let uv_box = trim_box(face).unwrap_or([f64::NEG_INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::INFINITY]);
    let spans = |knots: &[f64], degree: usize, count: usize, lo: f64, hi: f64| -> Vec<usize> {
        // Span i covers [knots[i], knots[i + 1]] and is shaped by control
        // points i − degree ..= i.
        (degree..count)
            .filter(|&i| knots[i + 1] > knots[i] && knots[i + 1] >= lo && knots[i] <= hi)
            .collect()
    };
    let rows = surface.control_points.len();
    let columns = surface.control_points.first().map_or(0, Vec::len);
    let su = spans(&surface.knots_u, surface.degree_u, rows, uv_box[0], uv_box[2]);
    let sv = spans(&surface.knots_v, surface.degree_v, columns, uv_box[1], uv_box[3]);
    let mut out = Vec::with_capacity(su.len() * sv.len());
    for &i in &su {
        for &j in &sv {
            let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
            let mut near = f64::INFINITY;
            let mut points = Vec::with_capacity((surface.degree_u + 1) * (surface.degree_v + 1));
            for row in &surface.control_points[i - surface.degree_u..=i] {
                for cp in &row[j - surface.degree_v..=j] {
                    let w = if cp.w != 0.0 { cp.w } else { 1.0 };
                    let (paper, view) = project(frame, placed, Vec3::new(cp.x / w, cp.y / w, cp.z / w));
                    b = [b[0].min(paper[0]), b[1].min(paper[1]), b[2].max(paper[0]), b[3].max(paper[1])];
                    near = near.min(view[2]);
                    points.push(paper);
                }
            }
            out.push(SpanHull { bounds: b, hull: convex_hull(points), near });
        }
    }
    out
}

/// A uniform bucket grid over paper boxes.
#[derive(Debug, Clone)]
struct Grid {
    origin: [f64; 2],
    cell: f64,
    width: usize,
    height: usize,
    buckets: Vec<Vec<usize>>,
}

impl Grid {
    fn new(extent: [f64; 4], count: usize) -> Grid {
        let span = (extent[2] - extent[0]).max(extent[3] - extent[1]).max(1e-9);
        let per_side = ((count as f64).sqrt().ceil() as usize).clamp(1, 256);
        let cell = span / per_side as f64;
        let width = (((extent[2] - extent[0]) / cell).ceil() as usize).clamp(1, 512);
        let height = (((extent[3] - extent[1]) / cell).ceil() as usize).clamp(1, 512);
        Grid { origin: [extent[0], extent[1]], cell, width, height, buckets: vec![Vec::new(); width * height] }
    }

    fn range(&self, b: [f64; 4]) -> (usize, usize, usize, usize) {
        let clamp = |v: f64, n: usize| (v.floor().max(0.0) as usize).min(n - 1);
        (
            clamp((b[0] - self.origin[0]) / self.cell, self.width),
            clamp((b[1] - self.origin[1]) / self.cell, self.height),
            clamp((b[2] - self.origin[0]) / self.cell, self.width),
            clamp((b[3] - self.origin[1]) / self.cell, self.height),
        )
    }

    fn insert(&mut self, b: [f64; 4], id: usize) {
        let (x0, y0, x1, y1) = self.range(b);
        for y in y0..=y1 {
            for x in x0..=x1 {
                self.buckets[y * self.width + x].push(id);
            }
        }
    }

    fn at(&self, p: [f64; 2]) -> &[usize] {
        let (x, y, _, _) = self.range([p[0], p[1], p[0], p[1]]);
        &self.buckets[y * self.width + x]
    }
}

/// Everything the visibility test reads.
struct Scene<'a> {
    frame: ViewFrame,
    placed: &'a PlacedView,
    floors: Floors,
    occluders: Vec<Occluder<'a>>,
    grid: Grid,
    /// Nearest view depth of the model, model units, less a margin: where a
    /// view ray toward the eye can stop looking.
    near: f64,
    clip: &'a Clip<'a>,
}

impl Scene<'_> {
    /// Whether a piece whose exact midpoint is `p` is hidden. `hint` is the
    /// face that hid the previous piece of the same curve, tried first — the
    /// next piece along is usually behind the same face — and is set to the
    /// face that hides this one.
    fn hidden(&self, p: Vec3, hint: &mut Option<usize>) -> bool {
        let (paper, view) = project(&self.frame, self.placed, p);
        let bar = self.floors.bar;
        if let Clip::Beyond { cap } = self.clip {
            if view[2] > bar && inside_region(cap, paper) {
                return true;
            }
        }
        let forward = v3(self.frame.forward);
        // A hit in front of the piece by more than the depth floor, and (in a
        // section) not in the material the cut removed.
        let in_front = |hit: Vec3| {
            let depth = dot3(sub3(a3(hit), self.frame.target), self.frame.forward);
            depth <= view[2] - bar && !(matches!(self.clip, Clip::Beyond { .. }) && depth < -bar)
        };
        // The ray toward the eye against a flat carrier through `origin`.
        let plane_hit = |origin: Vec3, normal: Vec3| -> Option<Vec3> {
            let denominator = normal.dot(forward);
            if denominator.abs() <= GRAZING_COS {
                return None;
            }
            let lambda = normal.dot(origin.sub(p)) / denominator;
            (lambda < -bar).then(|| p.add(forward.scale(lambda)))
        };
        // A curved face: the ray toward the eye against its carrier, a hit in
        // front of the piece and inside the trim. The ray is the same line for
        // every face the piece is asked of, so it is made once.
        let reach = view[2] - self.near;
        let ray = std::cell::OnceCell::new();
        let curved_hides = |face: &FaceRecord, tests: &TrimTests| -> bool {
            if reach <= bar {
                return false;
            }
            let Some(ray) = ray.get_or_init(|| make_line(p.sub(forward.scale(reach)), p.sub(forward.scale(bar))).ok()) else {
                return false;
            };
            RAY_QUERIES.with(|count| count.set(count.get() + 1));
            let fit = KernelTolerances::default().intersection_fit;
            let hits = intersect_curve_surface(ray, &face.surface, fit);
            hits.is_ok_and(|hits| {
                hits.into_iter()
                    .filter(|hit| !hit.tangential)
                    .any(|hit| in_front(hit.point) && in_face(face, Some(tests), [hit.u, hit.v], bar))
            })
        };
        // A flat face's one hit, in front of the piece, asked of the trim test.
        let flat_hides = |face: &FaceRecord, tests: &TrimTests, hit: Vec3| -> bool {
            uv_on(face, hit, bar).is_some_and(|uv| in_face(face, Some(tests), uv, bar))
        };
        // The piece is hidden when ANY occluder hides it, so the order they are
        // asked in is free, and it is chosen for cost. The face that hid the
        // previous piece is asked first, whole. Then every other face the paper
        // or a closed form decides — a triangle, a flat face whose projected
        // trim covers the point — then each curved face's ray, and last a flat
        // face the paper does not cover: near a trim of thousands of edges that
        // question goes to the kernel and costs more than all the others.
        let mut curved: Vec<usize> = Vec::new();
        let mut flat: Vec<(usize, Vec3)> = Vec::new();
        let first = hint.take();
        for index in first.into_iter().chain(self.grid.at(paper).iter().copied().filter(|index| Some(*index) != first)) {
            let occluder = &self.occluders[index];
            let pad = self.floors.crossing_mm + CHORD_MM;
            let holds = |b: &[f64; 4]| {
                paper[0] >= b[0] - pad && paper[0] <= b[2] + pad && paper[1] >= b[1] - pad && paper[1] <= b[3] + pad
            };
            if !holds(&occluder.bounds) {
                continue;
            }
            let hides = match &occluder.cover {
                Cover::Triangle { corners, normal } => {
                    plane_hit(corners[0], *normal).is_some_and(|hit| in_front(hit) && in_triangle(corners, *normal, hit, bar))
                }
                Cover::Face { face, plane, spans, tests, trim } => {
                    if !spans.iter().any(|span| span.near <= view[2] - bar && span.holds(paper, pad)) {
                        continue;
                    }
                    let asked_first = Some(index) == first;
                    match plane {
                        Some((origin, normal)) => {
                            let Some(hit) = plane_hit(*origin, *normal).filter(|hit| in_front(*hit)) else { continue };
                            if trim.as_ref().and_then(|trim| trim.covers(paper)) == Some(true) {
                                true
                            } else if asked_first {
                                flat_hides(face, tests, hit)
                            } else {
                                flat.push((index, hit));
                                continue;
                            }
                        }
                        None if asked_first => curved_hides(face, tests),
                        None => {
                            curved.push(index);
                            continue;
                        }
                    }
                }
            };
            if hides {
                *hint = Some(index);
                return true;
            }
        }
        let face_of = |index: usize| match &self.occluders[index].cover {
            Cover::Face { face, tests, .. } => Some((*face, tests)),
            Cover::Triangle { .. } => None,
        };
        let hider = curved
            .into_iter()
            .find(|&index| face_of(index).is_some_and(|(face, tests)| curved_hides(face, tests)))
            .or_else(|| {
                flat.into_iter()
                    .find(|&(index, hit)| face_of(index).is_some_and(|(face, tests)| flat_hides(face, tests, hit)))
                    .map(|(index, _)| index)
            });
        if hider.is_some() {
            *hint = hider;
        }
        hider.is_some()
    }
}

/// Whether `hit`, a point in the plane of the triangle `corners` (unit normal
/// `normal`), is inside it or within `bar` of it — the band a face's trim is
/// asked with ([`in_face`]), so a hit on the edge two triangles share is
/// inside both rather than in the crack between them.
fn in_triangle(corners: &[Vec3; 3], normal: Vec3, hit: Vec3, bar: f64) -> bool {
    (0..3).all(|k| {
        let (a, b) = (corners[k], corners[(k + 1) % 3]);
        let side = b.sub(a);
        let length = side.length();
        length <= 0.0 || side.cross(hit.sub(a)).dot(normal) / length >= -bar
    })
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Even-odd crossing count of a paper point against a set of polylines —
/// the cut region of a section.
pub fn inside_region(region: &[Vec<[f64; 2]>], p: [f64; 2]) -> bool {
    let mut inside = false;
    for line in region {
        for w in line.windows(2) {
            let (a, b) = (w[0], w[1]);
            if (a[1] <= p[1]) == (b[1] <= p[1]) {
                continue;
            }
            let x = a[0] + (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]);
            if x > p[0] {
                inside = !inside;
            }
        }
    }
    inside
}

// ---------------------------------------------------------------------------
// The pass
// ---------------------------------------------------------------------------

/// The visible model lines of `solids` through `frame` at `placed`'s scale,
/// restricted to `clip`.
pub fn visible_edges(solids: &[&BrepSolid], frame: &ViewFrame, placed: &PlacedView, clip: &Clip) -> HlrResult {
    let trims: Vec<SolidTrims> = solids.iter().map(|solid| SolidTrims::of(solid)).collect();
    let trims: Vec<&SolidTrims> = trims.iter().collect();
    visible_edges_with(solids, &trims, frame, placed, clip)
}

/// [`visible_edges`] with each solid's [`SolidTrims`] kept from before.
pub fn visible_edges_with(
    solids: &[&BrepSolid],
    trims: &[&SolidTrims],
    frame: &ViewFrame,
    placed: &PlacedView,
    clip: &Clip,
) -> HlrResult {
    TRIM_QUERIES.with(|count| count.set(0));
    RAY_QUERIES.with(|count| count.set(0));
    let floors = Floors::of(placed.scale);
    let mut candidates: Vec<Candidate> = Vec::new();
    // Per face: the paper boxes of its boundary edges and its silhouettes.
    let mut face_boxes: HashMap<(usize, usize, usize), (Vec<[f64; 4]>, Vec<[f64; 4]>)> = HashMap::new();
    // Per flat face: its trim on the paper, from its boundary edges' samples.
    let mut paper_trims: HashMap<(usize, usize, usize), Option<PaperTrim>> = HashMap::new();
    for (solid_index, solid) in solids.iter().enumerate() {
        let owners = edge_faces(solid);
        let edges = edge_candidates(solid_index, solid, frame, placed, &floors);
        for candidate in &edges {
            let Some((_, edge_id)) = candidate.edge else { continue };
            for (shell, face) in owners.get(&edge_id).into_iter().flatten() {
                face_boxes.entry((solid_index, *shell, *face)).or_default().0.push(candidate.bounds);
            }
        }
        let paper_of_edge: HashMap<u64, &[[f64; 2]]> =
            edges.iter().filter_map(|c| c.edge.map(|(_, id)| (id, c.samples.paper.as_slice()))).collect();
        let mut silhouettes: Vec<Candidate> = Vec::new();
        for (shell_index, shell) in solid.shells.iter().enumerate() {
            for (face_index, face) in shell.faces.iter().enumerate() {
                let found = silhouette_candidates(face, &trims[solid_index].faces[shell_index][face_index], frame, placed, &floors);
                let entry = face_boxes.entry((solid_index, shell_index, face_index)).or_default();
                entry.1.extend(found.iter().map(|c| c.bounds));
                silhouettes.extend(found);
                if planar(&face.surface) {
                    paper_trims.insert((solid_index, shell_index, face_index), PaperTrim::of(solid, face, &paper_of_edge, &floors));
                }
            }
        }
        candidates.extend(edges);
        candidates.extend(silhouettes);
    }
    let mut occluders: Vec<Occluder> = Vec::new();
    let mut near = f64::INFINITY;
    let mut extent = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for (solid_index, solid) in solids.iter().enumerate() {
        for (shell_index, shell) in solid.shells.iter().enumerate() {
            for (face_index, face) in shell.faces.iter().enumerate() {
                let (boundary, silhouettes) =
                    face_boxes.get(&(solid_index, shell_index, face_index)).cloned().unwrap_or_default();
                let bounds = face_bounds(face, &boundary, &silhouettes, frame, placed);
                if !(bounds[0] <= bounds[2]) {
                    continue;
                }
                let spans = span_hulls(face, frame, placed);
                near = spans.iter().fold(near, |near, span| near.min(span.near));
                extent = [
                    extent[0].min(bounds[0]),
                    extent[1].min(bounds[1]),
                    extent[2].max(bounds[2]),
                    extent[3].max(bounds[3]),
                ];
                let plane = if planar(&face.surface) {
                    let du = face.surface.domain_u().unwrap_or([0.0, 1.0]);
                    let dv = face.surface.domain_v().unwrap_or([0.0, 1.0]);
                    let (u, v) = (0.5 * (du[0] + du[1]), 0.5 * (dv[0] + dv[1]));
                    match (face.surface.evaluate(u, v), face.surface.normal(u, v)) {
                        (Ok(origin), Ok(normal)) => Some((origin, normal)),
                        _ => None,
                    }
                } else {
                    None
                };
                let trim = paper_trims.remove(&(solid_index, shell_index, face_index)).flatten();
                let tests = &trims[solid_index].faces[shell_index][face_index];
                occluders.push(Occluder { cover: Cover::Face { face, plane, spans, tests, trim }, bounds });
            }
        }
    }
    decide(candidates, occluders, extent, near, frame, placed, clip)
}

/// The shared back half of both passes: split every candidate where the
/// projection can change what is in front of it, decide each piece against
/// `occluders`, and join the visible pieces into runs.
fn decide(
    candidates: Vec<Candidate>,
    occluders: Vec<Occluder>,
    extent: [f64; 4],
    near: f64,
    frame: &ViewFrame,
    placed: &PlacedView,
    clip: &Clip,
) -> HlrResult {
    let floors = Floors::of(placed.scale);
    let mut result = HlrResult::default();
    if occluders.is_empty() {
        return result;
    }
    let mut grid = Grid::new(extent, occluders.len());
    for (index, occluder) in occluders.iter().enumerate() {
        grid.insert(occluder.bounds, index);
    }
    let scene = Scene {
        frame: *frame,
        placed,
        floors,
        occluders,
        grid,
        near: near - floors.bar * 10.0 - 1.0,
        clip,
    };

    let stations = split_parameters(&candidates, &scene);
    result.inked = candidates.iter().filter(|c| c.inked).count();
    result.silhouettes = candidates.iter().filter(|c| c.silhouette).count();

    // How many INKED edges meet at each vertex — the model's topology, not
    // what survived visibility: a silhouette corner of a box is where two
    // visible edges and a hidden one meet, and it stays a corner.
    let mut valence: HashMap<VertexKey, usize> = HashMap::new();
    for candidate in candidates.iter().filter(|c| c.inked) {
        if let Some((start, end)) = candidate.ends {
            *valence.entry(start).or_default() += 1;
            *valence.entry(end).or_default() += 1;
        }
    }
    let mut runs: Vec<Run> = Vec::new();
    for (index, candidate) in candidates.iter().enumerate() {
        if !candidate.inked {
            continue;
        }
        let pieces = pieces_of(candidate, &stations[index], &scene);
        result.pieces += pieces.len();
        runs.extend(runs_of(candidate, &pieces, &scene));
    }
    result.runs = chain_runs(runs, &valence, floors.crossing_mm);
    result.trim_queries = TRIM_QUERIES.with(|count| count.get());
    result.ray_queries = RAY_QUERIES.with(|count| count.get());
    result
}

// ---------------------------------------------------------------------------
// The mesh approximation
// ---------------------------------------------------------------------------

/// One solid's DISPLAY, in world coordinates: what a placement is drawn from
/// while its B-rep has not reached the scene.
#[derive(Debug, Clone, Default)]
pub struct MeshSolid {
    pub positions: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
    /// Per triangle, the display face it tessellates.
    pub faces: Vec<u32>,
    /// Every drawn display edge — each B-rep edge the tessellator emitted.
    pub edges: Vec<MeshEdge>,
}

/// One display edge of a [`MeshSolid`]: its polyline, and the topology
/// vertices it starts and ends on — where runs chain, as the exact pass chains
/// through the B-rep's own `start_vertex_id` / `end_vertex_id`. `None` when
/// either end is not recoverably one vertex.
#[derive(Debug, Clone, Default)]
pub struct MeshEdge {
    pub points: Vec<[f64; 3]>,
    pub ends: Option<(u64, u64)>,
}

/// The model lines of `meshes` through `frame` at `placed`'s scale, restricted
/// to `clip` — the STATED APPROXIMATION a placement draws while the exact
/// solids are not in the scene (their topology request is in flight, or the
/// registry could not clone a handle). It is the analytic pass's own split,
/// visibility and run code on different inputs:
///
/// - **candidates** are every display edge (each B-rep edge's tessellated
///   polyline, so a round's tangent lines are drawn like any other edge)
///   except a SEAM — an edge whose every chord is a mesh edge with the same
///   face's triangles on both sides — and the display mesh's SILHOUETTES, the
///   mesh edges inside one face whose two triangles face opposite ways along
///   the view;
/// - **occluders** are the display triangles, each hit in closed form.
///
/// What it gives up is what the display gave up: a curve is its
/// tessellation's chords, and an edge between two fragments of one surface is
/// drawn, because nothing in a display tells a split from a tangent line.
pub fn visible_mesh_lines(meshes: &[MeshSolid], frame: &ViewFrame, placed: &PlacedView, clip: &Clip) -> HlrResult {
    TRIM_QUERIES.with(|count| count.set(0));
    RAY_QUERIES.with(|count| count.set(0));
    let forward = v3(frame.forward);
    let mut candidates: Vec<Candidate> = Vec::new();
    let mut occluders: Vec<Occluder> = Vec::new();
    let mut near = f64::INFINITY;
    let mut extent = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for (solid_index, mesh) in meshes.iter().enumerate() {
        let points: Vec<Vec3> = mesh.positions.iter().map(|p| v3(*p)).collect();
        let welded = WeldedMesh::of(mesh, &points, forward);
        for edge in &mesh.edges {
            let chain: Vec<Vec3> = edge.points.iter().map(|p| v3(*p)).collect();
            if welded.seam(&chain) {
                continue;
            }
            // Runs chain through the topology vertices the edge runs between —
            // the model's adjacency, never the distance between two ends.
            if let Some(candidate) = polyline_candidate(chain, false, frame, placed) {
                candidates.push(Candidate {
                    ends: edge.ends.map(|(start, end)| ((solid_index, start), (solid_index, end))),
                    ..candidate
                });
            }
        }
        for chain in welded.silhouettes() {
            candidates.extend(polyline_candidate(chain, true, frame, placed));
        }
        for triangle in &mesh.triangles {
            let Some(corners) = triangle.iter().map(|&i| points.get(i as usize).copied()).collect::<Option<Vec<Vec3>>>() else {
                continue;
            };
            let corners = [corners[0], corners[1], corners[2]];
            let Ok(normal) = corners[1].sub(corners[0]).cross(corners[2].sub(corners[0])).normalized() else {
                continue;
            };
            let mut bounds = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
            for corner in corners {
                let (paper, view) = project(frame, placed, corner);
                bounds = [bounds[0].min(paper[0]), bounds[1].min(paper[1]), bounds[2].max(paper[0]), bounds[3].max(paper[1])];
                near = near.min(view[2]);
            }
            extent = [extent[0].min(bounds[0]), extent[1].min(bounds[1]), extent[2].max(bounds[2]), extent[3].max(bounds[3])];
            occluders.push(Occluder { cover: Cover::Triangle { corners, normal }, bounds });
        }
    }
    decide(candidates, occluders, extent, near, frame, placed, clip)
}

/// An inked candidate on a model polyline (a display edge, a mesh silhouette
/// chain), or `None` when it has no length.
fn polyline_candidate<'a>(chain: Vec<Vec3>, silhouette: bool, frame: &ViewFrame, placed: &PlacedView) -> Option<Candidate<'a>> {
    if chain.len() < 2 {
        return None;
    }
    let last = (chain.len() - 1) as f64;
    let intervals = chain.len() - 1;
    let carrier = Carrier::Polyline(chain);
    let samples = sample(&carrier, 0.0, last, intervals, frame, placed);
    if samples.len() < 2 || samples.paper_length() <= 0.0 {
        return None;
    }
    let bounds = bounds_of(&samples.paper);
    Some(Candidate { carrier, samples, inked: true, silhouette, edge: None, ends: None, bounds })
}

/// A display mesh welded by its exact coordinates — the tessellator writes a
/// point its triangles share as one value, whatever copies the shading split
/// it into — with the faces and facings of the triangles on each mesh edge.
struct WeldedMesh {
    index: HashMap<[u64; 3], u32>,
    position: Vec<Vec3>,
    /// Per welded mesh edge: `(face, faces the eye)` of each triangle on it.
    owners: HashMap<[u32; 2], Vec<(u32, bool)>>,
}

impl WeldedMesh {
    fn of(mesh: &MeshSolid, points: &[Vec3], forward: Vec3) -> WeldedMesh {
        let key = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let mut index: HashMap<[u64; 3], u32> = HashMap::new();
        let mut position = Vec::new();
        let welded: Vec<u32> = points
            .iter()
            .map(|p| {
                *index.entry(key(*p)).or_insert_with(|| {
                    position.push(*p);
                    (position.len() - 1) as u32
                })
            })
            .collect();
        let mut owners: HashMap<[u32; 2], Vec<(u32, bool)>> = HashMap::new();
        for (t, triangle) in mesh.triangles.iter().enumerate() {
            if triangle.iter().any(|&i| i as usize >= points.len()) {
                continue;
            }
            let [a, b, c] = [points[triangle[0] as usize], points[triangle[1] as usize], points[triangle[2] as usize]];
            let facing = b.sub(a).cross(c.sub(a)).dot(forward);
            if facing == 0.0 {
                continue;
            }
            let face = mesh.faces.get(t).copied().unwrap_or(u32::MAX);
            let w = [welded[triangle[0] as usize], welded[triangle[1] as usize], welded[triangle[2] as usize]];
            for k in 0..3 {
                let (i, j) = (w[k], w[(k + 1) % 3]);
                if i != j {
                    owners.entry([i.min(j), i.max(j)]).or_default().push((face, facing < 0.0));
                }
            }
        }
        WeldedMesh { index, position, owners }
    }

    /// The faces on the mesh edge between two model points, when both are
    /// mesh vertices joined by one.
    fn faces_on(&self, a: Vec3, b: Vec3) -> Option<&[(u32, bool)]> {
        let key = |p: Vec3| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let (i, j) = (*self.index.get(&key(a))?, *self.index.get(&key(b))?);
        self.owners.get(&[i.min(j), i.max(j)]).map(Vec::as_slice)
    }

    /// Whether a display edge is a SEAM: every chord of it a mesh edge with
    /// two triangles of ONE face on it — the surface meeting itself, not a
    /// line of the model. An edge whose chords are not mesh edges is drawn.
    fn seam(&self, chain: &[Vec3]) -> bool {
        chain.windows(2).all(|w| {
            self.faces_on(w[0], w[1]).is_some_and(|faces| faces.len() == 2 && faces[0].0 == faces[1].0)
        })
    }

    /// The silhouette chains: every mesh edge whose two triangles belong to
    /// ONE display face and face opposite ways along the view, chained through
    /// vertices exactly two of them meet at. An edge between two faces is a
    /// display edge already.
    fn silhouettes(&self) -> Vec<Vec<Vec3>> {
        let mut segments: Vec<[u32; 2]> = self
            .owners
            .iter()
            .filter(|(_, list)| list.len() == 2 && list[0].0 == list[1].0 && list[0].1 != list[1].1)
            .map(|(edge, _)| *edge)
            .collect();
        // HashMap order is not a drawing's business.
        segments.sort_unstable();
        let mut incident: HashMap<u32, Vec<usize>> = HashMap::new();
        for (i, segment) in segments.iter().enumerate() {
            incident.entry(segment[0]).or_default().push(i);
            incident.entry(segment[1]).or_default().push(i);
        }
        let valence = |v: u32| incident.get(&v).map_or(0, Vec::len);
        let mut used = vec![false; segments.len()];
        let mut starts: Vec<usize> = (0..segments.len()).collect();
        // From a chain END first, so an open chain keeps its ends.
        starts.sort_by_key(|&i| (valence(segments[i][0]) == 2 && valence(segments[i][1]) == 2, i));
        let mut out = Vec::new();
        for start in starts {
            if used[start] {
                continue;
            }
            used[start] = true;
            let segment = segments[start];
            let (head, mut tail) = if valence(segment[0]) != 2 { (segment[0], segment[1]) } else { (segment[1], segment[0]) };
            let mut path = vec![head, tail];
            while valence(tail) == 2 && tail != head {
                let Some(next) = incident[&tail].iter().copied().find(|i| !used[*i]) else { break };
                used[next] = true;
                tail = if segments[next][0] == tail { segments[next][1] } else { segments[next][0] };
                path.push(tail);
            }
            out.push(path.into_iter().map(|v| self.position[v as usize]).collect());
        }
        out
    }
}

/// Every place a candidate must be split, as sorted curve parameters: where
/// another candidate crosses it or ends on it, and where it leaves the clip.
fn split_parameters(candidates: &[Candidate], scene: &Scene) -> Vec<Vec<f64>> {
    let floors = scene.floors;
    let mut splits: Vec<Vec<f64>> = vec![Vec::new(); candidates.len()];
    // The segment grid.
    let mut extent = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    let mut count = 0;
    for c in candidates {
        extent = [extent[0].min(c.bounds[0]), extent[1].min(c.bounds[1]), extent[2].max(c.bounds[2]), extent[3].max(c.bounds[3])];
        count += c.samples.len();
    }
    if !(extent[0] <= extent[2]) {
        return splits;
    }
    let mut grid = Grid::new(extent, count.max(1));
    let mut segments: Vec<(usize, usize)> = Vec::new();
    for (ci, c) in candidates.iter().enumerate() {
        for si in 0..c.samples.len().saturating_sub(1) {
            let (a, b) = (c.samples.paper[si], c.samples.paper[si + 1]);
            let pad = floors.crossing_mm + CHORD_MM;
            grid.insert(
                [a[0].min(b[0]) - pad, a[1].min(b[1]) - pad, a[0].max(b[0]) + pad, a[1].max(b[1]) + pad],
                segments.len(),
            );
            segments.push((ci, si));
        }
    }
    let param_at = |ci: usize, si: usize, t: f64| {
        let params = &candidates[ci].samples.params;
        params[si] + (params[si + 1] - params[si]) * t
    };
    let tolerance = floors.crossing_mm + CHORD_MM;
    let mut seen: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
    for bucket in &grid.buckets {
        for (k, &first) in bucket.iter().enumerate() {
            for &second in &bucket[k + 1..] {
                let (lo, hi) = (first.min(second), first.max(second));
                let ((ca, sa), (cb, sb)) = (segments[lo], segments[hi]);
                if ca == cb && sa.abs_diff(sb) <= 1 {
                    continue;
                }
                if !candidates[ca].inked && !candidates[cb].inked {
                    continue;
                }
                if !seen.insert((lo, hi)) {
                    continue;
                }
                let pa = &candidates[ca].samples.paper;
                let pb = &candidates[cb].samples.paper;
                let vec2 = |p: [f64; 2]| Vec2 { x: p[0], y: p[1] };
                if let Some([ta, tb]) = segment_intersection(vec2(pa[sa]), vec2(pa[sa + 1]), vec2(pb[sb]), vec2(pb[sb + 1]), tolerance) {
                    splits[ca].push(param_at(ca, sa, ta));
                    splits[cb].push(param_at(cb, sb, tb));
                }
                // An END of one on the other, including along a collinear
                // overlap the crossing test does not report.
                for (from, from_seg, onto, onto_seg) in [(ca, sa, cb, sb), (cb, sb, ca, sa)] {
                    let fp = &candidates[from].samples.paper;
                    let op = &candidates[onto].samples.paper;
                    let last = fp.len() - 1;
                    for (end_seg, end) in [(0usize, fp[0]), (last - 1, fp[last])] {
                        if from_seg != end_seg {
                            continue;
                        }
                        let (distance, t) = segment_foot(end, op[onto_seg], op[onto_seg + 1]);
                        if distance <= tolerance {
                            splits[onto].push(param_at(onto, onto_seg, t));
                        }
                    }
                }
            }
        }
    }
    // The clip's own boundary.
    for (ci, c) in candidates.iter().enumerate() {
        if !c.inked {
            continue;
        }
        let side = |view: [f64; 3]| -> f64 {
            match scene.clip {
                Clip::Whole => 1.0,
                Clip::Beyond { .. } => view[2],
                Clip::Circle { radius } => radius - (view[0] * view[0] + view[1] * view[1]).sqrt(),
            }
        };
        if matches!(scene.clip, Clip::Whole) {
            continue;
        }
        for i in 1..c.samples.len() {
            let (g0, g1) = (side(c.samples.view[i - 1]), side(c.samples.view[i]));
            if (g0 < 0.0) == (g1 < 0.0) {
                continue;
            }
            let (mut lo, mut hi) = (c.samples.params[i - 1], c.samples.params[i]);
            for _ in 0..BISECTION_STEPS {
                let mid = 0.5 * (lo + hi);
                let Some(p) = c.carrier.at(mid) else { break };
                let g = side(scene.frame.to_view(a3(p)));
                if (g < 0.0) == (g0 < 0.0) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            splits[ci].push(0.5 * (lo + hi));
        }
    }
    for list in &mut splits {
        list.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    }
    splits
}

/// One decided piece of a candidate: its parameter span and its paper ends.
struct Piece {
    lo: f64,
    hi: f64,
    visible: bool,
}

/// Cut a candidate at its split parameters (merging any closer together on
/// the paper than the crossing floor) and decide each piece.
fn pieces_of(candidate: &Candidate, splits: &[f64], scene: &Scene) -> Vec<Piece> {
    let samples = &candidate.samples;
    let (s0, s1) = (samples.params[0], *samples.params.last().unwrap());
    let mut cuts = vec![s0];
    let paper_at = |s: f64| candidate.carrier.at(s).map(|p| project(&scene.frame, scene.placed, p).0);
    let mut last_paper = samples.paper[0];
    for &s in splits {
        if s <= s0 || s >= s1 {
            continue;
        }
        let Some(p) = paper_at(s) else { continue };
        if paper_distance(p, last_paper) < scene.floors.crossing_mm {
            continue;
        }
        cuts.push(s);
        last_paper = p;
    }
    let end_paper = *samples.paper.last().unwrap();
    if cuts.len() > 1 && paper_distance(last_paper, end_paper) < scene.floors.crossing_mm {
        cuts.pop();
    }
    cuts.push(s1);
    let mut pieces = Vec::with_capacity(cuts.len() - 1);
    let mut hint = None;
    for w in cuts.windows(2) {
        let (lo, hi) = (w[0], w[1]);
        let Some(mid) = candidate.carrier.at(0.5 * (lo + hi)) else { continue };
        let view = scene.frame.to_view(a3(mid));
        let kept = match scene.clip {
            Clip::Whole => true,
            Clip::Beyond { .. } => view[2] >= -scene.floors.bar,
            Clip::Circle { radius } => (view[0] * view[0] + view[1] * view[1]).sqrt() <= radius + scene.floors.bar,
        };
        let visible = kept && !scene.hidden(mid, &mut hint);
        pieces.push(Piece { lo, hi, visible });
    }
    pieces
}

/// A visible run on the paper, with the B-rep vertices it touches (for
/// chaining through a vertex two inked edges share).
struct Run {
    points: Vec<[f64; 2]>,
    start: Option<VertexKey>,
    end: Option<VertexKey>,
}

/// The visible runs of one candidate: maximal chains of consecutive visible
/// pieces, wrapped round the seam of a closed curve. A run keeps its own ends
/// and the SAMPLES strictly between them, so a straight edge is two points.
fn runs_of(candidate: &Candidate, pieces: &[Piece], scene: &Scene) -> Vec<Run> {
    let samples = &candidate.samples;
    let closed = samples.len() > 2
        && paper_distance(samples.paper[0], *samples.paper.last().unwrap()) < scene.floors.crossing_mm
        && samples.points[0].sub(*samples.points.last().unwrap()).length() <= scene.floors.bar;
    let mut spans: Vec<(f64, f64, bool, bool)> = Vec::new(); // (lo, hi, touches start, touches end)
    let mut i = 0;
    while i < pieces.len() {
        if !pieces[i].visible {
            i += 1;
            continue;
        }
        let first = i;
        while i < pieces.len() && pieces[i].visible {
            i += 1;
        }
        spans.push((pieces[first].lo, pieces[i - 1].hi, first == 0, i == pieces.len()));
    }
    let (s0, s1) = (samples.params[0], *samples.params.last().unwrap());
    // A closed curve visible across its seam is one run, not two.
    let mut wrapped: Option<(f64, f64)> = None;
    if closed && spans.len() >= 2 && spans[0].2 && spans.last().unwrap().3 {
        let head = spans.remove(0);
        let tail = spans.pop().unwrap();
        wrapped = Some((tail.0, head.1));
    }
    let point_at = |s: f64| candidate.carrier.at(s).map(|p| project(&scene.frame, scene.placed, p).0);
    let mut out = Vec::new();
    let mut emit = |pieces: &[(f64, f64)], start: Option<VertexKey>, end: Option<VertexKey>| {
        let mut points = Vec::new();
        for (k, &(lo, hi)) in pieces.iter().enumerate() {
            if k == 0 {
                if let Some(p) = point_at(lo) {
                    points.push(p);
                }
            }
            for (index, s) in samples.params.iter().enumerate() {
                if *s > lo && *s < hi {
                    points.push(samples.paper[index]);
                }
            }
            if let Some(p) = point_at(hi) {
                points.push(p);
            }
        }
        if points.len() >= 2 {
            out.push(Run { points, start, end });
        }
    };
    if let Some((lo, hi)) = wrapped {
        emit(&[(lo, s1), (s0, hi)], None, None);
    }
    for (lo, hi, at_start, at_end) in spans {
        let ends = candidate.ends;
        let whole_closed = closed && at_start && at_end;
        emit(
            &[(lo, hi)],
            if at_start && !whole_closed { ends.map(|e| e.0) } else { None },
            if at_end && !whole_closed { ends.map(|e| e.1) } else { None },
        );
    }
    out
}

/// Chain runs through every vertex exactly two INKED edges meet at — the
/// mesh pass's rule read on the B-rep's own vertices — when both runs reach it.
fn chain_runs(runs: Vec<Run>, valence: &HashMap<VertexKey, usize>, floor: f64) -> Vec<Vec<[f64; 2]>> {
    let mut at_vertex: HashMap<VertexKey, Vec<(usize, bool)>> = HashMap::new();
    for (index, run) in runs.iter().enumerate() {
        if let Some(v) = run.start {
            at_vertex.entry(v).or_default().push((index, true));
        }
        if let Some(v) = run.end {
            at_vertex.entry(v).or_default().push((index, false));
        }
    }
    // Neighbour through each end: (run, whether we enter at its start).
    let mut next: Vec<[Option<(usize, bool)>; 2]> = vec![[None, None]; runs.len()];
    for (vertex, list) in &at_vertex {
        if valence.get(vertex) != Some(&2) || list.len() != 2 || list[0].0 == list[1].0 {
            continue;
        }
        let ((a, a_start), (b, b_start)) = (list[0], list[1]);
        let (pa, pb) = (
            if a_start { runs[a].points[0] } else { *runs[a].points.last().unwrap() },
            if b_start { runs[b].points[0] } else { *runs[b].points.last().unwrap() },
        );
        if paper_distance(pa, pb) > floor + CHORD_MM {
            continue;
        }
        next[a][if a_start { 0 } else { 1 }] = Some((b, b_start));
        next[b][if b_start { 0 } else { 1 }] = Some((a, a_start));
    }
    let mut used = vec![false; runs.len()];
    let mut out = Vec::new();
    // Open chains first (from a run with a free end), then closed rings.
    let mut order: Vec<usize> = (0..runs.len()).collect();
    order.sort_by_key(|&i| (next[i][0].is_some() && next[i][1].is_some(), i));
    for start in order {
        if used[start] {
            continue;
        }
        used[start] = true;
        // Walk forward from the start run's END; a free START end is where
        // the chain begins.
        let mut points = runs[start].points.clone();
        if next[start][0].is_some() && next[start][1].is_none() {
            points.reverse();
            let mut cursor = next[start][0];
            extend_chain(&runs, &next, &mut used, &mut points, &mut cursor);
        } else {
            let mut cursor = next[start][1];
            extend_chain(&runs, &next, &mut used, &mut points, &mut cursor);
        }
        out.push(points);
    }
    out
}

fn extend_chain(
    runs: &[Run],
    next: &[[Option<(usize, bool)>; 2]],
    used: &mut [bool],
    points: &mut Vec<[f64; 2]>,
    cursor: &mut Option<(usize, bool)>,
) {
    while let Some((run, enter_at_start)) = *cursor {
        if used[run] {
            break;
        }
        used[run] = true;
        let mut piece = runs[run].points.clone();
        if !enter_at_start {
            piece.reverse();
        }
        points.extend(piece.into_iter().skip(1));
        *cursor = next[run][if enter_at_start { 1 } else { 0 }];
    }
}

// ---------------------------------------------------------------------------
// The exact section
// ---------------------------------------------------------------------------

/// One cut curve of a section on the paper, ORIENTED so the cut region lies
/// to its left as the section camera sees it (the region's outward normal is
/// toward the eye): a whole set of them sums to the region's signed area.
pub type CutCurve = Vec<[f64; 2]>;

/// The exact section of `solids` by the view's target plane (the camera
/// looks along its normal): per face, the plane's intersection with the
/// carrier — the kernel's closed forms for a plane against a cylinder, cone,
/// sphere or torus (`intersect_analytic_pair`), marched for a fitted surface
/// (`intersect_surfaces`), and for a plane against a plane the line their two
/// equations share, which the kernel's analytic lane leaves to its iso path on
/// purpose — trimmed to the face where its boundary edges cross the plane.
pub fn section_curves(solids: &[&BrepSolid], frame: &ViewFrame, placed: &PlacedView) -> Vec<CutCurve> {
    let trims: Vec<SolidTrims> = solids.iter().map(|solid| SolidTrims::of(solid)).collect();
    let trims: Vec<&SolidTrims> = trims.iter().collect();
    section_curves_with(solids, &trims, frame, placed)
}

/// [`section_curves`] with each solid's [`SolidTrims`] kept from before.
pub fn section_curves_with(solids: &[&BrepSolid], trims: &[&SolidTrims], frame: &ViewFrame, placed: &PlacedView) -> Vec<CutCurve> {
    let floors = Floors::of(placed.scale);
    let tolerances = KernelTolerances::default();
    let normal = v3(frame.forward);
    let target = v3(frame.target);
    let mut out = Vec::new();
    for (solid, trims) in solids.iter().zip(trims) {
        let mut reach: f64 = 1.0;
        for shell in &solid.shells {
            for face in &shell.faces {
                for row in &face.surface.control_points {
                    for cp in row {
                        let w = if cp.w != 0.0 { cp.w } else { 1.0 };
                        reach = reach.max(Vec3::new(cp.x / w, cp.y / w, cp.z / w).sub(target).length());
                    }
                }
            }
        }
        let reach = reach * 1.5;
        let (right, up) = (v3(frame.right), v3(frame.up));
        let Ok(cutting) = make_plane(target.sub(right.scale(reach)).sub(up.scale(reach)), right, up, 2.0 * reach, 2.0 * reach)
        else {
            continue;
        };
        for (shell, shell_trims) in solid.shells.iter().zip(&trims.faces) {
            for (face, tests) in shell.faces.iter().zip(shell_trims) {
                for carrier in section_carriers(&cutting, face, normal, target, reach, &tolerances) {
                    for curve in trim_section(solid, face, Some(tests), carrier, &cutting, frame, placed, &floors) {
                        out.push(curve);
                    }
                }
            }
        }
    }
    out
}

/// The untrimmed intersection of the cutting plane with one face's carrier.
fn section_carriers<'a>(
    cutting: &NurbsSurface,
    face: &'a FaceRecord,
    normal: Vec3,
    target: Vec3,
    reach: f64,
    tolerances: &KernelTolerances,
) -> Vec<(Carrier<'a>, f64, f64)> {
    let surface = &face.surface;
    if planar(surface) {
        let du = surface.domain_u().unwrap_or([0.0, 1.0]);
        let dv = surface.domain_v().unwrap_or([0.0, 1.0]);
        let (u, v) = (0.5 * (du[0] + du[1]), 0.5 * (dv[0] + dv[1]));
        let (Ok(origin), Ok(face_normal)) = (surface.evaluate(u, v), surface.normal(u, v)) else { return Vec::new() };
        let direction = normal.cross(face_normal);
        let Ok(direction) = direction.normalized() else { return Vec::new() }; // parallel or coplanar
        // The point of both planes nearest the target: solve n·x = n·t,
        // m·x = m·o in the span of the two normals.
        let (h1, h2) = (normal.dot(target), face_normal.dot(origin));
        let c = normal.dot(face_normal);
        let det = 1.0 - c * c;
        if det.abs() < 1e-18 {
            return Vec::new();
        }
        let a = (h1 - h2 * c) / det;
        let b = (h2 - h1 * c) / det;
        let point = normal.scale(a).add(face_normal.scale(b));
        let point = point.add(direction.scale(target.sub(point).dot(direction)));
        return vec![(
            Carrier::Line { a: point.sub(direction.scale(reach)), b: point.add(direction.scale(reach)) },
            0.0,
            1.0,
        )];
    }
    if let Some(curves) = intersect_analytic_pair(cutting, surface, tolerances.intersection_fit) {
        return curves
            .into_iter()
            .filter_map(|curve| {
                let [t0, t1] = curve.domain().ok()?;
                Some((Carrier::Curve(curve), t0, t1))
            })
            .collect();
    }
    let options = SurfaceIntersectionOptions { tolerance: tolerances.intersection_fit, ..Default::default() };
    match intersect_surfaces(cutting, surface, &options) {
        Ok(curves) => curves
            .into_iter()
            .filter(|curve| curve.points.len() >= 2)
            .map(|curve| {
                let mut points = curve.points;
                if curve.closed && points.first().map(|p| p.sub(*points.last().unwrap()).length()) > Some(0.0) {
                    points.push(points[0]);
                }
                let last = (points.len() - 1) as f64;
                (Carrier::Polyline(points), 0.0, last)
            })
            .collect(),
        Err(_) => Vec::new(),
    }
}

/// The spans of one cut carrier that lie on `face`: split where the face's
/// boundary edges cross the cutting plane (`intersect_curve_surface`), each
/// span kept when its midpoint classifies inside the trim, then sampled
/// against the chord and oriented.
fn trim_section(
    solid: &BrepSolid,
    face: &FaceRecord,
    tests: Option<&TrimTests>,
    carrier: (Carrier, f64, f64),
    cutting: &NurbsSurface,
    frame: &ViewFrame,
    placed: &PlacedView,
    floors: &Floors,
) -> Vec<CutCurve> {
    let (carrier, s0, s1) = carrier;
    let tolerances = KernelTolerances::default();
    let initial = match &carrier {
        Carrier::Polyline(points) => points.len().saturating_sub(1).max(1),
        Carrier::Curve(curve) => curve_initial(curve),
        _ => 1,
    };
    let base = sample(&carrier, s0, s1, initial, frame, placed);
    if base.len() < 2 {
        return Vec::new();
    }
    // Where the face's boundary crosses the plane, as carrier parameters.
    let mut cuts: Vec<f64> = vec![s0, s1];
    for lp in &face.loops {
        for coedge in &lp.coedges {
            let Some(edge) = solid.edges.iter().find(|e| e.id == coedge.edge_id) else { continue };
            let Ok(hits) = intersect_curve_surface(&edge.curve, cutting, tolerances.intersection_fit) else { continue };
            for hit in hits {
                if hit.t < edge.t0.min(edge.t1) - 1e-12 || hit.t > edge.t0.max(edge.t1) + 1e-12 {
                    continue;
                }
                if let Some(s) = nearest_parameter(&carrier, &base, hit.point) {
                    cuts.push(s);
                }
            }
        }
    }
    cuts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mut out = Vec::new();
    for w in cuts.windows(2) {
        let (lo, hi) = (w[0], w[1]);
        if hi - lo <= 0.0 {
            continue;
        }
        let Some(mid) = carrier.at(0.5 * (lo + hi)) else { continue };
        let Some(uv) = uv_on(face, mid, floors.bar) else { continue };
        if !in_face(face, tests, uv, floors.bar) {
            continue;
        }
        let piece = sample(&carrier, lo, hi, initial_between(&base, lo, hi), frame, placed);
        if piece.len() < 2 || piece.paper_length() < floors.crossing_mm {
            continue;
        }
        let mut paper = piece.paper;
        // Orient: the tangent along m × n_F, m the cap's outward normal
        // (toward the eye) and n_F the face's outward normal — the region is
        // then on the left as the eye sees it.
        let step = (hi - lo) * 1e-4;
        let tangent = match (carrier.at(0.5 * (lo + hi) + step), carrier.at(0.5 * (lo + hi) - step)) {
            (Some(a), Some(b)) => a.sub(b),
            _ => piece.points[piece.points.len() - 1].sub(piece.points[0]),
        };
        if let Some(n_face) = outward_normal(face, mid) {
            let eye = v3(frame.forward).scale(-1.0);
            if tangent.dot(eye.cross(n_face)) < 0.0 {
                paper.reverse();
            }
        }
        out.push(paper);
    }
    out
}

/// The carrier parameter nearest a model point.
fn nearest_parameter(carrier: &Carrier, samples: &Samples, p: Vec3) -> Option<f64> {
    if let Carrier::Edge(curve) = carrier {
        return project_point_to_curve(curve, p).ok().map(|foot| foot.u);
    }
    // Nearest chord, then its foot's parameter.
    let mut best: Option<(f64, f64)> = None;
    for i in 0..samples.len().saturating_sub(1) {
        let (a, b) = (samples.points[i], samples.points[i + 1]);
        let d = b.sub(a);
        let length2 = d.dot(d);
        let t = if length2 > 0.0 { (p.sub(a).dot(d) / length2).clamp(0.0, 1.0) } else { 0.0 };
        let distance = p.sub(a.add(d.scale(t))).length();
        let s = samples.params[i] + (samples.params[i + 1] - samples.params[i]) * t;
        if best.is_none_or(|(bd, _)| distance < bd) {
            best = Some((distance, s));
        }
    }
    best.map(|(_, s)| s)
}

