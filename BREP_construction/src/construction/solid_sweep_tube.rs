//! `BREP_SWEEP_TORUS_TUBE` (default off): the solid a ring (a solid torus)
//! fills under a two-turn motion, as the set of points within its tube radius
//! r of the surface Σ(t, ψ) = M(t)·k(ψ) its core circle k sweeps.
//!
//! Where the core circle at t is nearest a point, the point lies square to
//! the circle and to its velocity, so the boundary lies on the two sheets
//! Σ ± r·N (N the unit normal of Σ) and on the half of the start torus that
//! trails and the half of the end torus that leads. Those four charts close
//! into a surface K, a torus parameterised by (σ, ψ): the sheet Σ + r·N
//! (σ = t), the end torus's leading half, the sheet Σ − r·N back from the
//! end, and the start torus's trailing half.
//!
//! K bounds the solid exactly where no other pose of the ring reaches into it.
//! This first slice builds only that regime: Σ regular (its sheets' r·κ < 1)
//! and every station of K at least r from the core circle at every pose
//! (outside its own time, by the motion's exact speed bound between poses).
//! Anything else (folds, a sheet or cap the motion carries the ring back over)
//! declines to the existing routes. K is cut into patches (σ chunks by ψ
//! quarters), each interpolated bicubically through exact points of K and
//! refined until its error at the cells' middles is within the fit tolerance;
//! neighbouring patches share their boundary stations, so their edges are one
//! curve. The patches are sewn into one shell and validated.
use crate::solid_sweep_trim::TwoTurnMotion;
#[path = "solid_sweep_tube_trim.rs"]
pub mod trim;
#[path = "solid_sweep_tube_fold.rs"]
pub mod fold;
use crate::{interpolate_curve, BrepSolid, KernelRefusal, KernelStage, NurbsSurface, OrRefuse, Vec3, Vec4};

/// The switch, default off.
pub fn torus_tube_on() -> bool {
    crate::switch!("BREP_SWEEP_TORUS_TUBE", false)
}


fn decline(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("sweepSolid torus tube: {message}"))
}

/// The ring and its motion.
pub struct Tube {
    centre: Vec3,
    axis: Vec3,
    x: Vec3,
    y: Vec3,
    major: f64,
    minor: f64,
    motion: TwoTurnMotion,
    from: f64,
    to: f64,
}

/// A point of Σ with its frame: the point, the unit normal N, and the unit
/// leading direction T (Σ_t's part square to the core circle).
struct Frame {
    point: Vec3,
    normal: Vec3,
    lead: Vec3,
}

impl Tube {
    pub fn new(centre: Vec3, axis: Vec3, major: f64, minor: f64, motion: TwoTurnMotion, from: f64, to: f64) -> Option<Tube> {
        let axis = axis.normalized().ok()?;
        let x = axis.perpendicular().ok()?;
        let y = axis.cross(x);
        (major > minor && minor > 0.0 && to > from).then_some(Tube { centre, axis, x, y, major, minor, motion, from, to })
    }

    /// Σ at (t, ψ) with its frame; `None` where Σ is singular.
    fn frame(&self, t: f64, psi: f64) -> Option<Frame> {
        let pose = self.motion.pose(t);
        let (s, c) = psi.sin_cos();
        let local = self.centre.add(self.x.scale(self.major * c)).add(self.y.scale(self.major * s));
        let tangent = self.x.scale(-self.major * s).add(self.y.scale(self.major * c));
        let point = pose.apply(local);
        let along_psi = pose.rotate(tangent);
        let along_t = self.motion.velocity(t, point);
        let normal = along_t.cross(along_psi);
        let length = normal.length();
        if !(length > 1e-9 * along_t.length().max(1e-300) * along_psi.length()) {
            return None;
        }
        let normal = normal.scale(1.0 / length);
        let unit_psi = along_psi.scale(1.0 / along_psi.length());
        let lead = along_t.sub(unit_psi.scale(along_t.dot(unit_psi)));
        let lead = lead.normalized().ok()?;
        Some(Frame { point, normal, lead })
    }

    /// K at chart `chart` (0: Σ + rN over t, 1: end torus's leading half, 2:
    /// Σ − rN back over t, 3: start torus's trailing half), local s in [0, 1].
    fn k_point(&self, chart: usize, s: f64, psi: f64) -> Option<Vec3> {
        let r = self.minor;
        let at = |t: f64| self.frame(t, psi);
        Some(match chart {
            0 => {
                let f = at(self.from + (self.to - self.from) * s)?;
                f.point.add(f.normal.scale(r))
            }
            1 => {
                let f = at(self.to)?;
                let phi = std::f64::consts::PI * s;
                f.point.add(f.normal.scale(r * phi.cos())).add(f.lead.scale(r * phi.sin()))
            }
            2 => {
                let f = at(self.to - (self.to - self.from) * s)?;
                f.point.sub(f.normal.scale(r))
            }
            _ => {
                let f = at(self.from)?;
                let phi = std::f64::consts::PI * s;
                f.point.sub(f.normal.scale(r * phi.cos())).sub(f.lead.scale(r * phi.sin()))
            }
        })
    }

    /// r times the larger principal curvature of Σ at (t, ψ), by central
    /// differences in the range's own fraction τ (t = from + τ·range, step
    /// 1e-5; curvature does not depend on the parameter): the sheets Σ ± r·N
    /// fold where it reaches 1. Infinite where Σ is singular.
    fn fold_ratio(&self, t: f64, psi: f64) -> f64 {
        let range = self.to - self.from;
        let at = |tau: f64, psi: f64| {
            let (s, c) = psi.sin_cos();
            self.motion.pose(self.from + range * tau).apply(self.centre.add(self.x.scale(self.major * c)).add(self.y.scale(self.major * s)))
        };
        let (t, h) = ((t - self.from) / range, 1e-5);
        let p = at(t, psi);
        let st = at(t + h, psi).sub(at(t - h, psi)).scale(0.5 / h);
        let sp = at(t, psi + h).sub(at(t, psi - h)).scale(0.5 / h);
        let stt = at(t + h, psi).sub(p.scale(2.0)).add(at(t - h, psi)).scale(1.0 / (h * h));
        let spp = at(t, psi + h).sub(p.scale(2.0)).add(at(t, psi - h)).scale(1.0 / (h * h));
        let stp = at(t + h, psi + h).sub(at(t + h, psi - h)).sub(at(t - h, psi + h)).add(at(t - h, psi - h)).scale(0.25 / (h * h));
        let normal = st.cross(sp);
        let length = normal.length();
        if !(length > 0.0) {
            return f64::INFINITY;
        }
        let n = normal.scale(1.0 / length);
        // Shape operator I⁻¹·II; its eigenvalues are the principal curvatures.
        let (e, f, g) = (st.dot(st), st.dot(sp), sp.dot(sp));
        let (l, m, nn) = (stt.dot(n), stp.dot(n), spp.dot(n));
        let det = e * g - f * f;
        if !(det > 0.0) {
            return f64::INFINITY;
        }
        let mean = (e * nn - 2.0 * f * m + g * l) / (2.0 * det);
        let gauss = (l * nn - m * m) / det;
        let spread = (mean * mean - gauss).max(0.0).sqrt();
        self.minor * (mean.abs() + spread)
    }

    /// The least distance from `q` to the core circle placed at time t.
    fn reach(&self, q: Vec3, t: f64) -> f64 {
        let local = self.motion.pose(t).invert(q).sub(self.centre);
        let up = local.dot(self.axis);
        let flat = local.sub(self.axis.scale(up)).length();
        (flat - self.major).hypot(up)
    }

    /// The least distance from `q` to the core circle over every pose of the
    /// motion (the swept solid holds `q` where it is at most r): 8192 poses,
    /// the five least local minima refined by golden section to 1e-14 of the
    /// range. A referee for tests.
    #[doc(hidden)]
    pub fn depth(&self, q: Vec3) -> f64 {
        let count = 8192;
        let h = (self.to - self.from) / count as f64;
        let values: Vec<f64> = (0..=count).map(|k| self.reach(q, self.from + h * k as f64)).collect();
        let mut minima: Vec<usize> = (0..=count).filter(|&k| (k == 0 || values[k] <= values[k - 1]) && (k == count || values[k] <= values[k + 1])).collect();
        minima.sort_by(|&x, &y| values[x].total_cmp(&values[y]));
        let mut best = values.iter().copied().fold(f64::INFINITY, f64::min);
        for &k in minima.iter().take(5) {
            let (mut a, mut b) = (self.from + h * k.saturating_sub(1) as f64, (self.from + h * (k + 1) as f64).min(self.to));
            let ratio = 0.5 * (5f64.sqrt() - 1.0);
            // Stops at 1e-14 of the range or after 200 steps (a flat distance).
            for _ in 0..200 {
                if b - a <= 1e-14 * (self.to - self.from) {
                    break;
                }
                let (c, d) = (b - ratio * (b - a), a + ratio * (b - a));
                if self.reach(q, c) < self.reach(q, d) {
                    b = d;
                } else {
                    a = c;
                }
            }
            best = best.min(self.reach(q, 0.5 * (a + b)));
        }
        best
    }

    /// An exact bound on how fast `reach(q, ·)` changes: the speed of the
    /// motion's velocity field at q (each turn's rate times the distance to
    /// its axis point, the first's through the second's frame).
    fn reach_speed(&self, q: Vec3) -> f64 {
        let [p0, p1] = self.motion.points;
        let [w0, w1] = self.motion.rates;
        let to_second = q.sub(p1).length();
        w1.abs() * to_second + w0.abs() * (to_second + p1.sub(p0).length())
    }

    /// Whether some pose other than `own` brings the core circle nearer `q`
    /// than r (less a tie band): poses every `step`, each interval cleared by
    /// the speed bound; near `own` (within `window`) the depth is read on a
    /// fine grid and must not dip.
    fn covered(&self, q: Vec3, own: f64, step: f64, window: f64, tie: f64) -> bool {
        let r = self.minor;
        let speed = self.reach_speed(q);
        let _ = step;
        let count = 64usize;
        let h = (self.to - self.from) / count as f64;
        let mut previous = (self.from, self.reach(q, self.from));
        for k in 1..=count {
            let t = self.from + h * k as f64;
            let value = self.reach(q, t);
            if previous.0 <= own + window && t >= own - window {
                // The interval holding the own time: cleared by the bound
                // only beyond the window about it; inside it the certified
                // own-time read (a tie counts as covered: the trimmed route
                // then reads it again and declines on a tie).
                let (a, b) = (previous.0.max(own - window), t.min(own + window));
                if !matches!(self.own_read(q, own, window, tie), fold::OwnRead::Clear) {
                    return true;
                }
                if previous.0 < a && self.covered_between(q, previous.0, a, speed, tie, 0) {
                    return true;
                }
                if b < t && self.covered_between(q, b, t, speed, tie, 0) {
                    return true;
                }
            } else if previous.1.min(value) - speed * h * 0.5 < r - tie {
                // Not cleared by the bound: look closer, halving.
                if self.covered_between(q, previous.0, t, speed, tie, 0) {
                    return true;
                }
            }
            previous = (t, value);
        }
        false
    }

    fn covered_between(&self, q: Vec3, a: f64, b: f64, speed: f64, tie: f64, level: usize) -> bool {
        let r = self.minor;
        let (fa, fb) = (self.reach(q, a), self.reach(q, b));
        if fa < r - tie || fb < r - tie {
            return true;
        }
        if fa.min(fb) - speed * (b - a) * 0.5 >= r - tie {
            return false;
        }
        if level >= 30 {
            return true; // undecided: count it covered (declines)
        }
        let m = 0.5 * (a + b);
        self.covered_between(q, a, m, speed, tie, level + 1) || self.covered_between(q, m, b, speed, tie, level + 1)
    }
}

/// A tensor-product interpolant through `points[i][j]` at (`us[i]`, `vs[j]`),
/// biquintic (or lower where there are too few points).
fn interpolate_grid(points: &[Vec<Vec3>], us: &[f64], vs: &[f64]) -> Result<NurbsSurface, String> {
    let degree_u = 5.min(us.len() - 1);
    let degree_v = 5.min(vs.len() - 1);
    // Along v for each row, then along u through the rows' controls.
    let rows: Vec<crate::NurbsCurve> = points.iter().map(|row| interpolate_curve(row, degree_v, vs)).collect::<Result<_, _>>()?;
    let knots_v = rows[0].knots.clone();
    let count_v = rows[0].control_points.len();
    let mut columns: Vec<crate::NurbsCurve> = Vec::with_capacity(count_v);
    for j in 0..count_v {
        let column: Vec<Vec3> = rows.iter().map(|row| row.control_points[j].point()).collect::<Result<_, _>>()?;
        columns.push(interpolate_curve(&column, degree_u, us)?);
    }
    let knots_u = columns[0].knots.clone();
    let count_u = columns[0].control_points.len();
    let net: Vec<Vec<Vec4>> = (0..count_u).map(|i| (0..count_v).map(|j| columns[j].control_points[i]).collect()).collect();
    NurbsSurface::new(degree_u, degree_v, knots_u, knots_v, net)
}

/// The body, or a refusal naming why the tube route does not build it.
pub fn sweep_torus_tube(tube: &Tube) -> Result<BrepSolid, KernelRefusal> {
    // Tolerances from the ring's own size, never its distance from the origin;
    // a ring placed farther off than 1e6 of its size is declined (too few
    // digits are left to hold the tube to them).
    let size = tube.major + tube.minor;
    // A range too short for its parameter's magnitude leaves the poses (and
    // every difference across them) too few digits: declined as such.
    if !(tube.from.abs().max(tube.to.abs()) <= 1e6 * (tube.to - tube.from)) {
        return Err(decline("tube_range", "the motion's range is too short for its parameter's magnitude".into()));
    }
    // Where the ring is: its centre at the start, the middle and the end.
    let far = [tube.from, 0.5 * (tube.from + tube.to), tube.to].iter().map(|&t| tube.motion.pose(t).apply(tube.centre).length()).fold(0.0, f64::max);
    if !(far <= 1e6 * size) {
        return Err(decline("tube_far", "the ring lies too far off the origin for its size".into()));
    }
    let fit = 1e-7 * size;
    // Σ regular and its sheets regular: r·κ < 1 is read on the stations below
    // (a fold makes K run inside the solid, which this slice does not build).
    let range = tube.to - tube.from;
    let fastest = tube.motion.rates.iter().fold(0.0f64, |m, w| m.max(w.abs()));
    // σ chunks: a sheet in pieces of at most 15° of the fastest turn; each cap in two.
    let sheet_chunks = ((fastest * range).to_degrees() / 15.0).ceil().max(1.0) as usize;
    let chunks = [sheet_chunks, 2, sheet_chunks, 2];
    let quarters = 4;
    // patches[a][b]: σ chunk a (round K), ψ quarter b, with its domains.
    let mut patches: Vec<Vec<(NurbsSurface, [f64; 2], [f64; 2])>> = Vec::new();
    let step = (range / 4000.0).min(0.5f64.to_radians() / fastest.max(1e-12));
    let tie = 1e-9 * size;
    for chart in 0..4 {
        for c in 0..chunks[chart] {
            let (s0, s1) = (c as f64 / chunks[chart] as f64, (c + 1) as f64 / chunks[chart] as f64);
            let mut ring = Vec::with_capacity(quarters);
            for q in 0..quarters {
                let (p0, p1) = (q as f64 * std::f64::consts::FRAC_PI_2, (q + 1) as f64 * std::f64::consts::FRAC_PI_2);
                let (surface, n, us, vs, grid) = fit_patch(tube, chart, [s0, s1], [p0, p1], fit)?;
                // The sheets stay well short of a fold at every station
                // (r·κ under 0.9: a fold makes K run inside the solid).
                if chart == 0 || chart == 2 {
                    for &u in &us {
                        let t = if chart == 0 { tube.from + range * u } else { tube.to - range * u };
                        for &v in &vs {
                            if !(tube.fold_ratio(t, v) < 0.9) {
                                // A sheet nears a fold: its folded part is covered (the
                                // motion's nearby poses reach it), so the trim removes it.
                                return trimmed_route(tube, chunks, fit, range);
                            }
                        }
                    }
                }
                // Every station of the patch must lie on the boundary.
                let stride = (n / 16).max(1);
                for (i, &u) in us.iter().enumerate().step_by(stride) {
                    let own = match chart {
                        0 => tube.from + range * u,
                        1 => tube.to,
                        2 => tube.to - range * u,
                        _ => tube.from,
                    };
                    for p in grid[i].iter().step_by(stride) {
                        if tube.covered(*p, own, step, 0.02 * range, tie) {
                            // The motion carries the ring back over K: trim it.
                            return trimmed_route(tube, chunks, fit, range);
                        }
                    }
                }
                ring.push((surface, [s0, s1], [p0, p1]));
            }
            patches.push(ring);
        }
    }
    let mut solid = assemble(&patches)?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(decline("tube_validate", format!("the tube does not validate: {:?}", issues.first())));
    }
    // Outward on the sheet Σ + r·N is +N (away from the core surface): the
    // first patch's normal at its middle says whether the faces turn out.
    let (surface, u, v) = &patches[0][0];
    let (um, vm) = (0.5 * (u[0] + u[1]), 0.5 * (v[0] + v[1]));
    let normal = surface.normal(um, vm).or_refuse(KernelStage::Collect, "tube_normal")?;
    let outward = tube.frame(tube.from + range * um, vm).ok_or_else(|| decline("tube_singular", "the core circle's sweep is singular".into()))?.normal;
    let along = normal.dot(outward);
    if along < 0.0 {
        crate::orientation::flip_all_faces(&mut solid)?;
    } else if !(along > 0.0) {
        return Err(decline("tube_orientation", "the tube's normal is undecided".into()));
    }
    Ok(solid)
}

/// The tube where the motion carries the ring back over K (or folds a sheet,
/// whose folded part is covered): K's cover traced into loops on the lattice
/// and K trimmed along them (`trim`).
fn trimmed_route(tube: &Tube, chunks: [usize; 4], fit: f64, range: f64) -> Result<BrepSolid, KernelRefusal> {
    let lattice = trim::Lattice::new(chunks, 8, 16);
    // A fold needs no guard here: its folded part is covered by the motion's
    // nearby poses (the own-time read finds them), so the trim removes it.
    let window = 0.02 * range;
    let loops = trim::trim_loops(tube, &lattice, window, 1e-12 * tube.minor).map_err(|why| decline("tube_trim", why))?;
    if loops.is_empty() {
        return Err(decline("tube_trim", "K is covered at a station but no trim loop crosses the lattice".into()));
    }
    let solid = trim::trimmed_tube(tube, chunks, &lattice, 8, 16, &loops, window, fit).map_err(|why| decline("tube_trim", why))?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(decline("tube_validate", format!("the trimmed tube does not validate: {:?}", issues.first())));
    }
    Ok(solid)
}

/// One patch of K (chart `chart`, local s in `su`, ψ in `pv`): biquintic
/// through exact points on an n × n grid, n doubled from 8 until the error at
/// the cells' middles is within `fit` (stepping 8, 12, 16, 24, 32, 48, 64, 96,
/// 128: a quarter where the normal of Σ turns fast in ψ needs past 64),
/// with its stations and points.
#[allow(clippy::type_complexity)]
pub(crate) fn fit_patch(tube: &Tube, chart: usize, su: [f64; 2], pv: [f64; 2], fit: f64) -> Result<(NurbsSurface, usize, Vec<f64>, Vec<f64>, Vec<Vec<Vec3>>), KernelRefusal> {
    let [s0, s1] = su;
    let [p0, p1] = pv;
    let singular = || decline("tube_singular", "the core circle's sweep is singular".into());
    let step = |n: usize| if n % 3 == 0 { n / 3 * 4 } else { n / 2 * 3 };
    // Stations along σ and along ψ refined apart, in steps of √2 (8, 12, 16,
    // 24, 32, 48, 64, 96, 128): each round steps the direction whose midpoints miss
    // most, so neither carries more stations than it needs (STEP size).
    let (mut nu, mut nv) = (8usize, 8usize);
    loop {
        let us: Vec<f64> = (0..=nu).map(|i| s0 + (s1 - s0) * i as f64 / nu as f64).collect();
        let vs: Vec<f64> = (0..=nv).map(|j| p0 + (p1 - p0) * j as f64 / nv as f64).collect();
        let mut grid = Vec::with_capacity(us.len());
        for &u in &us {
            let mut row = Vec::with_capacity(vs.len());
            for &v in &vs {
                row.push(tube.k_point(chart, u, v).ok_or_else(singular)?);
            }
            grid.push(row);
        }
        let surface = interpolate_grid(&grid, &us, &vs).or_refuse(KernelStage::Refine, "tube_fit")?;
        let miss = |u: f64, v: f64| -> Result<f64, KernelRefusal> {
            let exact = tube.k_point(chart, u, v).ok_or_else(singular)?;
            Ok(surface.evaluate(u, v).or_refuse(KernelStage::Refine, "tube_fit")?.sub(exact).length())
        };
        let (mut along_u, mut along_v, mut centre): (f64, f64, f64) = (0.0, 0.0, 0.0);
        for i in 0..nu {
            for j in 0..nv {
                let (um, vm) = (0.5 * (us[i] + us[i + 1]), 0.5 * (vs[j] + vs[j + 1]));
                along_u = along_u.max(miss(um, vs[j])?);
                along_v = along_v.max(miss(us[i], vm)?);
                centre = centre.max(miss(um, vm)?);
            }
        }
        if along_u.max(along_v).max(centre) <= fit {
            return Ok((surface, nu.max(nv), us, vs, grid));
        }
        let (grow_u, grow_v) = (nu < 128, nv < 128);
        if !grow_u && !grow_v {
            return Err(decline("tube_fit", format!("a patch of K does not fit within {fit:.1e} (worst {:.3e})", along_u.max(along_v).max(centre))));
        }
        // the direction that misses most (the centre counts for both)
        if (along_u >= along_v && grow_u) || !grow_v {
            nu = step(nu);
        } else {
            nv = step(nv);
        }
    }
}

/// The closed shell of a periodic grid of patches (σ chunks round K by ψ
/// quarters), its edges shared by construction: each σ boundary's curve along
/// ψ and each quarter boundary's curve along σ is one edge of the two patches
/// it parts, its pcurves the sides of their parameter rectangles.
fn assemble(patches: &[Vec<(NurbsSurface, [f64; 2], [f64; 2])>]) -> Result<BrepSolid, KernelRefusal> {
    use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
    let (rows, columns) = (patches.len(), patches[0].len());
    let mut next = 1u64;
    let mut id = || {
        next += 1;
        next - 1
    };
    let corner = |a: usize, b: usize| -> Result<Vec3, KernelRefusal> {
        let (surface, u, v) = &patches[a % rows][b % columns];
        surface.evaluate(u[0], v[0]).or_refuse(KernelStage::Collect, "tube_corner")
    };
    let mut vertices = Vec::with_capacity(rows * columns);
    let mut vertex_ids = vec![vec![0u64; columns]; rows];
    for a in 0..rows {
        for b in 0..columns {
            let vid = id();
            vertex_ids[a][b] = vid;
            vertices.push(VertexRecord { id: vid, point: corner(a, b)? });
        }
    }
    let mut edges = Vec::new();
    let make_edge = |edges: &mut Vec<EdgeRecord>, id: u64, curve: crate::NurbsCurve, start: u64, end: u64| -> Result<u64, KernelRefusal> {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Collect, "tube_edge")?;
        edges.push(EdgeRecord { id, curve, t0, t1, start_vertex_id: start, end_vertex_id: end, degenerate: false, name: None });
        Ok(id)
    };
    // σ edges: at the start of chunk a, along ψ over quarter b; ψ edges: at the start of quarter b, along σ over chunk a.
    let mut sigma_edges = vec![vec![0u64; columns]; rows];
    let mut psi_edges = vec![vec![0u64; columns]; rows];
    for a in 0..rows {
        for b in 0..columns {
            let (surface, u, v) = &patches[a][b];
            let along_psi = surface.iso_curve_u(u[0]).or_refuse(KernelStage::Collect, "tube_edge")?;
            let along_sigma = surface.iso_curve_v(v[0]).or_refuse(KernelStage::Collect, "tube_edge")?;
            let (eid, fid) = (id(), id());
            sigma_edges[a][b] = make_edge(&mut edges, eid, along_psi, vertex_ids[a][b], vertex_ids[a][(b + 1) % columns])?;
            psi_edges[a][b] = make_edge(&mut edges, fid, along_sigma, vertex_ids[a][b], vertex_ids[(a + 1) % rows][b])?;
        }
    }
    let line = |from: [f64; 2], to: [f64; 2]| crate::make_line(Vec3::new(from[0], from[1], 0.0), Vec3::new(to[0], to[1], 0.0)).or_refuse(KernelStage::Collect, "tube_pcurve");
    let mut faces = Vec::with_capacity(rows * columns);
    for a in 0..rows {
        for b in 0..columns {
            let (surface, u, v) = &patches[a][b];
            let sides = [
                (psi_edges[a][b], true, line([u[0], v[0]], [u[1], v[0]])?),
                (sigma_edges[(a + 1) % rows][b], true, line([u[1], v[0]], [u[1], v[1]])?),
                (psi_edges[a][(b + 1) % columns], false, line([u[1], v[1]], [u[0], v[1]])?),
                (sigma_edges[a][b], false, line([u[0], v[1]], [u[0], v[0]])?),
            ];
            let coedges = sides.into_iter().map(|(edge_id, forward, pcurve)| CoedgeRecord { id: id(), edge_id, forward, pcurve }).collect();
            faces.push(FaceRecord { id: id(), surface: surface.clone(), same_sense: true, loops: vec![LoopRecord { id: id(), coedges }], name: None });
        }
    }
    Ok(BrepSolid {
        mass_properties_cache: Default::default(),
        id: id(),
        vertices,
        edges,
        shells: vec![ShellRecord { id: id(), faces }],
        genus: 1,
        kind: Default::default(),
        cells: Vec::new(),
    })
}
