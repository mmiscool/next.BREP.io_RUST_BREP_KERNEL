//! Tri-dexel stock (lane C10c, row 1): three dexel grids, one per axis,
//! seeded from the resolved stock (box and cylinder analytically, a mesh by
//! parity ray casts, the previous setup's Z-dexel by conversion) and cut by
//! a revolved tool profile moving with a varying tool axis.
//!
//! Each grid stores, per column line parallel to its axis, the material
//! intervals along that line (`stock_sim::Column`, shared with C7's Z-dexel
//! through its public API). A cut takes the motion from one pose to the next
//! at sub-poses spaced at most `step` along the tip path and
//! `max_axis_step_deg` of axis change; at every sub-pose every column line
//! in the tool's box is intersected EXACTLY with the revolved cutting part
//! (each profile band is a frustum: a quadratic in the line parameter) and
//! the intersection is removed. The union of sub-pose cuts under-approximates
//! the swept volume by slivers no thicker than `step² / (8·r_min)`
//! (`CutReport::sliver_bound`), stated per simulation; undercuts are
//! represented because every column keeps a list of intervals. Work counts
//! the column lines intersected (those with material inside the tool's box
//! along the line) over all sub-poses, and is budgeted.

use crate::cutter::{ToolPart, ToolProfile};
use crate::model::{Bounds3, Diagnostic, Motion, MotionKind, Point3, RigidTransform};
use crate::multiaxis::slerp;
use crate::multiaxis::tilt::ClPose;
use crate::stock::{ResolvedStock, StockBox, StockCylinder};
use crate::stock_sim::{Column, DexelGrid};
use crate::{CamError, MeshSnapshot, ProjectionMode, Runner, Tolerance, TriangleIndex};
use serde::{Deserialize, Serialize};

/// `(radius, height)` of one profile point.
type ProfilePointPair = (f64, f64);

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct TriDexelOptions {
    /// Requested cell size (both transverse axes of every grid).
    pub cell: f64,
    /// Largest number of columns per grid.
    pub max_columns: u64,
    /// Largest tip travel between sub-poses of one cut.
    pub step: f64,
    /// Largest axis change between sub-poses, degrees.
    pub max_axis_step_deg: f64,
    /// Work budget for one simulation (column line cuts).
    pub max_work: u64,
}

impl Default for TriDexelOptions {
    fn default() -> Self {
        Self { cell: 0.5, max_columns: 4_000_000, step: 0.5, max_axis_step_deg: 2.0, max_work: 400_000_000 }
    }
}

impl TriDexelOptions {
    pub fn validate(&self) -> Result<(), CamError> {
        let bad = |m: &str| Err(CamError::InvalidInput(format!("tri-dexel: {m}")));
        if !self.cell.is_finite() || self.cell <= 0.0 {
            return bad("cell must be finite and positive");
        }
        if !self.step.is_finite() || self.step <= 0.0 {
            return bad("step must be finite and positive");
        }
        if !self.max_axis_step_deg.is_finite() || self.max_axis_step_deg <= 0.0 {
            return bad("axis step must be positive");
        }
        if self.max_columns == 0 || self.max_work == 0 {
            return bad("budgets must be positive");
        }
        Ok(())
    }
}

/// One dexel grid: column lines parallel to `axis` (0 = X, 1 = Y, 2 = Z),
/// indexed over the two transverse axes `u = (axis + 1) % 3`, `v = (axis + 2) % 3`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AxisGrid {
    pub axis: usize,
    /// Minimum corner of the grid's box.
    pub origin: Point3,
    /// Cell size along `u` and `v`.
    pub cell: [f64; 2],
    /// Column counts along `u` and `v`.
    pub n: [u32; 2],
    /// Row-major over `v` then `u`: `columns[iv * n[0] + iu]`.
    pub columns: Vec<Column>,
    /// Extent along the axis.
    pub range: [f64; 2],
}

impl AxisGrid {
    pub fn u(&self) -> usize {
        (self.axis + 1) % 3
    }

    pub fn v(&self) -> usize {
        (self.axis + 2) % 3
    }

    /// Transverse centre of a column.
    pub fn centre(&self, iu: u32, iv: u32) -> (f64, f64) {
        (
            self.origin.axis(self.u()) + (iu as f64 + 0.5) * self.cell[0],
            self.origin.axis(self.v()) + (iv as f64 + 0.5) * self.cell[1],
        )
    }

    /// Point on the column line at axis coordinate `s`.
    pub fn point(&self, iu: u32, iv: u32, s: f64) -> Point3 {
        let (cu, cv) = self.centre(iu, iv);
        let mut c = [0.0; 3];
        c[self.axis] = s;
        c[self.u()] = cu;
        c[self.v()] = cv;
        Point3::new(c[0], c[1], c[2])
    }

    pub fn column(&self, iu: u32, iv: u32) -> &Column {
        &self.columns[(iv * self.n[0] + iu) as usize]
    }

    pub fn column_mut(&mut self, iu: u32, iv: u32) -> &mut Column {
        &mut self.columns[(iv * self.n[0] + iu) as usize]
    }

    /// Column under a transverse position, `None` outside the grid.
    pub fn column_of(&self, p: Point3) -> Option<(u32, u32)> {
        let fu = (p.axis(self.u()) - self.origin.axis(self.u())) / self.cell[0];
        let fv = (p.axis(self.v()) - self.origin.axis(self.v())) / self.cell[1];
        if fu < 0.0 || fv < 0.0 || fu >= self.n[0] as f64 || fv >= self.n[1] as f64 {
            return None;
        }
        Some((fu as u32, fv as u32))
    }

    /// Material at `p` according to this grid.
    pub fn contains(&self, p: Point3) -> bool {
        match self.column_of(p) {
            Some((iu, iv)) => {
                let s = p.axis(self.axis);
                self.column(iu, iv).intervals.iter().any(|[a, b]| s >= *a && s <= *b)
            }
            None => false,
        }
    }

    pub fn cell_area(&self) -> f64 {
        self.cell[0] * self.cell[1]
    }

    pub fn volume(&self) -> f64 {
        self.columns.iter().map(|c| c.height()).sum::<f64>() * self.cell_area()
    }

    /// Column index ranges `[u0, u1, v0, v1]` (inclusive) whose cells meet `b`.
    pub fn columns_in(&self, b: &Bounds3) -> Option<[u32; 4]> {
        let (u, v) = (self.u(), self.v());
        let lo_u = ((b.min.axis(u) - self.origin.axis(u)) / self.cell[0]).floor();
        let hi_u = ((b.max.axis(u) - self.origin.axis(u)) / self.cell[0]).floor();
        let lo_v = ((b.min.axis(v) - self.origin.axis(v)) / self.cell[1]).floor();
        let hi_v = ((b.max.axis(v) - self.origin.axis(v)) / self.cell[1]).floor();
        let max_u = self.n[0] as f64 - 1.0;
        let max_v = self.n[1] as f64 - 1.0;
        if hi_u < 0.0 || hi_v < 0.0 || lo_u > max_u || lo_v > max_v {
            return None;
        }
        Some([lo_u.max(0.0) as u32, hi_u.min(max_u) as u32, lo_v.max(0.0) as u32, hi_v.min(max_v) as u32])
    }

    fn layout(axis: usize, bounds: &Bounds3, opts: &TriDexelOptions) -> Result<AxisGrid, CamError> {
        let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
        let size = bounds.size();
        let nu = (size.axis(u) / opts.cell).ceil().max(1.0);
        let nv = (size.axis(v) / opts.cell).ceil().max(1.0);
        if nu * nv > opts.max_columns as f64 {
            return Err(CamError::InvalidInput(format!(
                "tri-dexel: {} columns on axis {axis} exceed the budget of {}",
                nu * nv,
                opts.max_columns
            )));
        }
        let n = [nu as u32, nv as u32];
        Ok(AxisGrid {
            axis,
            origin: bounds.min,
            cell: [size.axis(u) / nu, size.axis(v) / nv],
            n,
            columns: vec![Column::default(); (n[0] * n[1]) as usize],
            range: [bounds.min.axis(axis), bounds.max.axis(axis)],
        })
    }
}

/// The stock as three dexel grids.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TriDexel {
    pub grids: [AxisGrid; 3],
    pub bounds: Bounds3,
}

/// What one cut removed.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CutReport {
    /// Mean of the three grids' removed volumes.
    pub removed_volume: f64,
    pub sub_poses: u64,
    /// Column lines intersected (work units): lines with material inside the
    /// tool's box along the line, summed over sub-poses.
    pub work: u64,
    /// Thickest uncut sliver the sub-pose spacing can leave.
    pub sliver_bound: f64,
}

/// The exact savings a cut may take (see [`TriDexel::cut_with`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct CutPlan {
    pub skip_start: bool,
    pub prune: bool,
    /// Cut a level move of a vertical tool exactly, one visit per line.
    pub sweep: bool,
}

/// What a motion stream removed.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TriDexelReport {
    pub removed_volume: f64,
    pub motions: u64,
    pub cutting_motions: u64,
    pub work: u64,
    pub sliver_bound: f64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<Diagnostic>,
}

/// Merge sorted-by-start intervals, dropping empty ones.
fn merge(mut v: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    v.retain(|[a, b]| b > a);
    v.sort_by(|x, y| x[0].total_cmp(&y[0]));
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(v.len());
    for [a, b] in v {
        match out.last_mut() {
            Some(last) if a <= last[1] => last[1] = last[1].max(b),
            _ => out.push([a, b]),
        }
    }
    out
}

/// Solution set of `a t² + b t + c <= 0` within `[lo, hi]`: at most two intervals.
fn quad_le_zero(a: f64, b: f64, c: f64, lo: f64, hi: f64) -> Vec<[f64; 2]> {
    let eps = 1e-12;
    let keep = |l: f64, h: f64| if h > l { vec![[l, h]] } else { Vec::new() };
    if a.abs() <= eps {
        if b.abs() <= eps {
            return if c <= 0.0 { keep(lo, hi) } else { Vec::new() };
        }
        let t = -c / b;
        return if b > 0.0 { keep(lo, t.min(hi)) } else { keep(t.max(lo), hi) };
    }
    let disc = b * b - 4.0 * a * c;
    if a > 0.0 {
        if disc < 0.0 {
            return Vec::new();
        }
        let s = disc.sqrt();
        let (t0, t1) = ((-b - s) / (2.0 * a), (-b + s) / (2.0 * a));
        keep(t0.max(lo), t1.min(hi))
    } else {
        // Opens downward: <= 0 outside the roots.
        if disc < 0.0 {
            return keep(lo, hi);
        }
        let s = disc.sqrt();
        let (t0, t1) = ((-b + s) / (2.0 * a), (-b - s) / (2.0 * a));
        let mut out = keep(lo, t0.min(hi));
        out.extend(keep(t1.max(lo), hi));
        out
    }
}

/// Intervals of the line `q0 + t d` (tool frame, `d` unit) inside the
/// revolved cutting part of `profile`, in `t`.
/// The point of segment `a`-`b` (in XY, at `a`'s Z) closest to `(x, y)`.
fn closest_on_segment(a: Point3, b: Point3, x: f64, y: f64) -> Point3 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((x - a.x) * dx + (y - a.y) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    Point3::new(a.x + t * dx, a.y + t * dy, a.z)
}

/// Radius of the cutting part at height `h` above the tip (the widest band
/// covering it), `None` outside the cutting part.
fn radius_at(bands: &[(ProfilePointPair, ProfilePointPair)], h: f64) -> Option<f64> {
    let mut best: Option<f64> = None;
    for &((r0, h0), (r1, h1)) in bands {
        if h >= h0 && h <= h1 && h1 > h0 {
            let r = r0 + (r1 - r0) * (h - h0) / (h1 - h0);
            best = Some(best.map_or(r, |b: f64| b.max(r)));
        }
    }
    best.filter(|r| *r > 0.0)
}

/// Union over t in [0, 1] of the chords `s(t) ± sqrt(R² - w(t)²)` of a disc
/// of radius `r` whose centre moves linearly, `s` from `s0` to `s1` along the
/// line and `w` from `w0` to `w1` across it. The t where |w| ≤ R is one
/// interval, so the union is one interval: its lower end minimises the convex
/// `s - sqrt(R² - w²)` and its upper end maximises the concave `s + sqrt(..)`.
fn chord_union(s0: f64, s1: f64, w0: f64, w1: f64, r: f64) -> Option<[f64; 2]> {
    let (ds, dw) = (s1 - s0, w1 - w0);
    // Parameter range where the line meets the disc.
    let (mut t_lo, mut t_hi) = (0.0f64, 1.0f64);
    if dw == 0.0 {
        if w0.abs() > r {
            return None;
        }
    } else {
        let (ta, tb) = ((-r - w0) / dw, (r - w0) / dw);
        t_lo = t_lo.max(ta.min(tb));
        t_hi = t_hi.min(ta.max(tb));
        if t_lo > t_hi {
            return None;
        }
    }
    let at = |t: f64| {
        let w = w0 + dw * t;
        let half = (r * r - w * w).max(0.0).sqrt();
        let s = s0 + ds * t;
        (s - half, s + half)
    };
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut take = |t: f64| {
        if t >= t_lo && t <= t_hi {
            let (l, h) = at(t);
            lo = lo.min(l);
            hi = hi.max(h);
        }
    };
    take(t_lo);
    take(t_hi);
    let l = (ds * ds + dw * dw).sqrt();
    if dw != 0.0 && l > 0.0 {
        // Stationary points: w = -ds·sgn(dw)·R / L for the lower end and
        // +ds·sgn(dw)·R / L for the upper one. Both candidates are taken for
        // both ends, so a stationary point outside the range changes nothing.
        let w_lower = -ds * dw.signum() * r / l;
        let w_upper = -w_lower;
        take((w_lower - w0) / dw);
        take((w_upper - w0) / dw);
    }
    (hi > lo).then_some([lo, hi])
}

pub fn line_in_profile(profile: &ToolProfile, q0: Point3, d: Point3) -> Vec<[f64; 2]> {
    let a_r = d.x * d.x + d.y * d.y;
    let b_r = 2.0 * (q0.x * d.x + q0.y * d.y);
    let c_r = q0.x * q0.x + q0.y * q0.y;
    let mut out = Vec::new();
    for w in profile.points.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        if p0.part != ToolPart::Cutting || p1.height <= p0.height {
            continue;
        }
        // t range of the band by height.
        let (lo, hi) = if d.z.abs() <= 1e-15 {
            if q0.z < p0.height || q0.z > p1.height {
                continue;
            }
            (f64::NEG_INFINITY, f64::INFINITY)
        } else {
            let t0 = (p0.height - q0.z) / d.z;
            let t1 = (p1.height - q0.z) / d.z;
            (t0.min(t1), t0.max(t1))
        };
        // R(t) = alpha + beta t with R(h) linear in h.
        let slope = (p1.radius - p0.radius) / (p1.height - p0.height);
        let alpha = p0.radius + slope * (q0.z - p0.height);
        let beta = slope * d.z;
        // r² <= R²: (a_r - beta²) t² + (b_r - 2 alpha beta) t + (c_r - alpha²) <= 0
        for iv in quad_le_zero(a_r - beta * beta, b_r - 2.0 * alpha * beta, c_r - alpha * alpha, lo, hi) {
            // R must be non-negative over the kept interval; clip where it is not.
            let r_at = |t: f64| alpha + beta * t;
            let mut i = iv;
            if r_at(i[0]) < 0.0 || r_at(i[1]) < 0.0 {
                if beta.abs() > 1e-15 {
                    let tz = -alpha / beta;
                    if r_at(i[0]) < 0.0 {
                        i[0] = i[0].max(tz);
                    }
                    if r_at(i[1]) < 0.0 {
                        i[1] = i[1].min(tz);
                    }
                } else {
                    continue;
                }
            }
            if i[1] > i[0] && i[0].is_finite() && i[1].is_finite() {
                out.push(i);
            }
        }
    }
    merge(out)
}

/// Clip the parameter range of a line against a convex box given as six
/// half-spaces (outward normals and corner points).
fn clip_line_box(p0: Point3, d: Point3, b: &StockBox) -> Option<[f64; 2]> {
    let t = &b.to_machine;
    let axes = [Point3::X, Point3::Y, Point3::Z];
    let mut lo = f64::NEG_INFINITY;
    let mut hi = f64::INFINITY;
    for ax in axes.iter() {
        let n = t.apply_vector(*ax);
        let min_c = t.apply_point(b.bounds.min).dot(n);
        let max_c = t.apply_point(b.bounds.max).dot(n);
        let (c_lo, c_hi) = (min_c.min(max_c), min_c.max(max_c));
        let num = p0.dot(n);
        let den = d.dot(n);
        if den.abs() <= 1e-15 {
            if num < c_lo || num > c_hi {
                return None;
            }
            continue;
        }
        let t0 = (c_lo - num) / den;
        let t1 = (c_hi - num) / den;
        lo = lo.max(t0.min(t1));
        hi = hi.min(t0.max(t1));
    }
    if hi > lo {
        Some([lo, hi])
    } else {
        None
    }
}

/// Parameter range of a line inside a finite cylinder.
fn clip_line_cylinder(p0: Point3, d: Point3, c: &StockCylinder) -> Option<[f64; 2]> {
    let a = c.axis;
    let rel = p0.sub(c.base);
    // Axial clip.
    let along0 = rel.dot(a);
    let dal = d.dot(a);
    let (mut lo, mut hi) = if dal.abs() <= 1e-15 {
        if along0 < 0.0 || along0 > c.height {
            return None;
        }
        (f64::NEG_INFINITY, f64::INFINITY)
    } else {
        let t0 = -along0 / dal;
        let t1 = (c.height - along0) / dal;
        (t0.min(t1), t0.max(t1))
    };
    // Radial: |rel_perp + t d_perp|² <= r².
    let rp = rel.sub(a.scale(along0));
    let dp = d.sub(a.scale(dal));
    let qa = dp.length_squared();
    let qb = 2.0 * rp.dot(dp);
    let qc = rp.length_squared() - c.radius * c.radius;
    let iv = *quad_le_zero(qa, qb, qc, lo, hi).first()?;
    lo = iv[0];
    hi = iv[1];
    if hi > lo {
        Some([lo, hi])
    } else {
        None
    }
}

/// Parity intervals of a line along `axis` through a closed mesh.
fn mesh_intervals(mesh: &MeshSnapshot, index: Option<&dyn TriangleIndex>, axis: usize, p0: Point3, eps: f64) -> Vec<[f64; 2]> {
    let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
    let mode = match axis {
        0 => ProjectionMode::Yz,
        1 => ProjectionMode::Xz,
        _ => ProjectionMode::Xy,
    };
    let mut ids = Vec::new();
    match index {
        Some(i) => {
            let mut lo = p0;
            let mut hi = p0;
            let set = |p: &mut Point3, k: usize, val: f64| match k {
                0 => p.x = val,
                1 => p.y = val,
                _ => p.z = val,
            };
            // The whole mesh extent along the axis (finite, for tree indexes).
            set(&mut lo, axis, mesh.bounds.min.axis(axis) - 1.0);
            set(&mut hi, axis, mesh.bounds.max.axis(axis) + 1.0);
            i.query_aabb(&Bounds3::new(lo.sub(Point3::new(eps, eps, eps)), hi.add(Point3::new(eps, eps, eps))), mode, &mut ids);
            ids.sort_unstable();
            ids.dedup();
        }
        None => ids.extend(0..mesh.triangle_count() as u32),
    }
    let (pu, pv) = (p0.axis(u), p0.axis(v));
    let mut crossings: Vec<f64> = Vec::new();
    for id in ids {
        let t = &mesh.triangles[id as usize];
        let [a, b, c] = t.vertices();
        let (au, av, bu, bv, cu, cv) = (a.axis(u), a.axis(v), b.axis(u), b.axis(v), c.axis(u), c.axis(v));
        let det = (bu - au) * (cv - av) - (cu - au) * (bv - av);
        if det.abs() <= 1e-18 {
            continue;
        }
        let l1 = ((pu - au) * (cv - av) - (cu - au) * (pv - av)) / det;
        let l2 = ((bu - au) * (pv - av) - (pu - au) * (bv - av)) / det;
        if l1 < 0.0 || l2 < 0.0 || l1 + l2 > 1.0 {
            continue;
        }
        crossings.push(a.axis(axis) + l1 * (b.axis(axis) - a.axis(axis)) + l2 * (c.axis(axis) - a.axis(axis)));
    }
    crossings.sort_by(|x, y| x.total_cmp(y));
    // A line through an edge or vertex shared by several triangles yields
    // the same crossing from each of them: keep one.
    crossings.dedup_by(|a, b| (*a - *b).abs() <= 1e-9 * (1.0 + b.abs()));
    let mut out = Vec::new();
    let mut i = 0;
    while i + 1 < crossings.len() {
        out.push([crossings[i], crossings[i + 1]]);
        i += 2;
    }
    merge(out)
}

impl TriDexel {
    /// Lay the three grids over `bounds`, empty.
    pub fn empty(bounds: &Bounds3, opts: &TriDexelOptions) -> Result<TriDexel, CamError> {
        opts.validate()?;
        if bounds.is_empty() || !bounds.is_finite() {
            return Err(CamError::InvalidInput("tri-dexel: stock bounds are empty or not finite".into()));
        }
        Ok(TriDexel {
            grids: [AxisGrid::layout(0, bounds, opts)?, AxisGrid::layout(1, bounds, opts)?, AxisGrid::layout(2, bounds, opts)?],
            bounds: *bounds,
        })
    }

    /// Seed from a resolved stock. `rest` supplies the previous setup's
    /// Z-dexel for `ResolvedStock::Rest`; `index` is built on the mesh stock.
    pub fn from_stock(
        stock: &ResolvedStock,
        opts: &TriDexelOptions,
        rest: Option<&DexelGrid>,
        index: Option<&dyn TriangleIndex>,
    ) -> Result<TriDexel, CamError> {
        opts.validate()?;
        let bounds = match stock {
            ResolvedStock::Rest(_) => {
                let r = rest.ok_or_else(|| CamError::InvalidInput("tri-dexel: rest stock needs the previous setup's grid".into()))?;
                return Self::from_dexel(r, opts);
            }
            other => other.machine_bounds().ok_or_else(|| CamError::InvalidInput("tri-dexel: stock has no bounds".into()))?,
        };
        let mut td = Self::empty(&bounds, opts)?;
        for g in td.grids.iter_mut() {
            let axis = g.axis;
            let mut d = [0.0; 3];
            d[axis] = 1.0;
            let d = Point3::new(d[0], d[1], d[2]);
            let (range0, range1) = (g.range[0], g.range[1]);
            for iv in 0..g.n[1] {
                for iu in 0..g.n[0] {
                    let p0 = g.point(iu, iv, range0);
                    let intervals: Vec<[f64; 2]> = match stock {
                        ResolvedStock::Box(b) => clip_line_box(p0, d, b).into_iter().collect(),
                        ResolvedStock::Cylinder(c) => clip_line_cylinder(p0, d, c).into_iter().collect(),
                        ResolvedStock::Mesh(m) => mesh_intervals(m, index, axis, p0, 1e-9).into_iter().map(|[a, b]| [a - range0, b - range0]).collect(),
                        ResolvedStock::Rest(_) => unreachable!(),
                    };
                    let col = g.column_mut(iu, iv);
                    col.intervals = intervals
                        .into_iter()
                        .map(|[a, b]| [(range0 + a).max(range0), (range0 + b).min(range1)])
                        .filter(|[a, b]| b > a)
                        .collect();
                }
            }
        }
        Ok(td)
    }

    /// Convert C7's Z-dexel: the Z grid is resampled at the nearest column,
    /// the X and Y grids are built by scanning the Z columns (cell-quantised
    /// along their axes, a bound of one cell).
    pub fn from_dexel(grid: &DexelGrid, opts: &TriDexelOptions) -> Result<TriDexel, CamError> {
        opts.validate()?;
        let bounds = grid.bounds();
        let mut td = Self::empty(&bounds, opts)?;
        // Z grid.
        {
            let g = &mut td.grids[2];
            for iv in 0..g.n[1] {
                for iu in 0..g.n[0] {
                    let (cx, cy) = g.centre(iu, iv);
                    let jx = (((cx - grid.origin.x) / grid.cell_x).floor().max(0.0) as u32).min(grid.nx - 1);
                    let jy = (((cy - grid.origin.y) / grid.cell_y).floor().max(0.0) as u32).min(grid.ny - 1);
                    *g.column_mut(iu, iv) = grid.column(jx, jy).clone();
                }
            }
        }
        // X and Y grids from the Z columns.
        for axis in 0..2 {
            let g = &mut td.grids[axis];
            for iv in 0..g.n[1] {
                for iu in 0..g.n[0] {
                    let (cu, cv) = g.centre(iu, iv);
                    // Transverse coordinates: axis X has u = Y, v = Z; axis Y has u = Z, v = X.
                    let (y_or_x, z) = if axis == 0 { (cu, cv) } else { (cv, cu) };
                    let mut intervals: Vec<[f64; 2]> = Vec::new();
                    let n_along = if axis == 0 { grid.nx } else { grid.ny };
                    let mut open: Option<f64> = None;
                    for j in 0..n_along {
                        let (jx, jy) = if axis == 0 {
                            (j, (((y_or_x - grid.origin.y) / grid.cell_y).floor().max(0.0) as u32).min(grid.ny - 1))
                        } else {
                            ((((y_or_x - grid.origin.x) / grid.cell_x).floor().max(0.0) as u32).min(grid.nx - 1), j)
                        };
                        let has = grid.column(jx, jy).has_material(z - 1e-12, z + 1e-12);
                        let cell = if axis == 0 { grid.cell_x } else { grid.cell_y };
                        let start = (if axis == 0 { grid.origin.x } else { grid.origin.y }) + j as f64 * cell;
                        match (has, open) {
                            (true, None) => open = Some(start),
                            (false, Some(s)) => {
                                intervals.push([s, start]);
                                open = None;
                            }
                            _ => {}
                        }
                    }
                    if let Some(s) = open {
                        let end = if axis == 0 { grid.origin.x + grid.nx as f64 * grid.cell_x } else { grid.origin.y + grid.ny as f64 * grid.cell_y };
                        intervals.push([s, end]);
                    }
                    g.column_mut(iu, iv).intervals = merge(intervals);
                }
            }
        }
        Ok(td)
    }

    /// The Z grid as C7's Z-dexel (for its deviation map and stock mesh).
    pub fn to_dexel_grid(&self) -> DexelGrid {
        let g = &self.grids[2];
        DexelGrid {
            origin: g.origin,
            cell_x: g.cell[0],
            cell_y: g.cell[1],
            nx: g.n[0],
            ny: g.n[1],
            columns: g.columns.clone(),
            z_range: g.range,
        }
    }

    /// Volume estimates of the three grids.
    pub fn grid_volumes(&self) -> [f64; 3] {
        [self.grids[0].volume(), self.grids[1].volume(), self.grids[2].volume()]
    }

    /// Mean of the three grids' volumes.
    pub fn volume(&self) -> f64 {
        let v = self.grid_volumes();
        (v[0] + v[1] + v[2]) / 3.0
    }

    /// Material at `p` by every grid (`[x, y, z]`).
    pub fn contains_by_axis(&self, p: Point3) -> [bool; 3] {
        [self.grids[0].contains(p), self.grids[1].contains(p), self.grids[2].contains(p)]
    }

    /// Material at `p`: the majority of the three grids.
    pub fn contains(&self, p: Point3) -> bool {
        self.contains_by_axis(p).iter().filter(|b| **b).count() >= 2
    }

    /// Cut the cutting part of `profile` moving from `from` to `to`.
    pub fn cut(&mut self, profile: &ToolProfile, from: ClPose, to: ClPose, opts: &TriDexelOptions, tolerance: &Tolerance, work_left: &mut u64, runner: &mut Runner<'_>) -> Result<CutReport, CamError> {
        self.cut_with(profile, from, to, CutPlan { skip_start: false, prune: true, sweep: true }, opts, tolerance, work_left, runner)
    }

    /// [`cut`](Self::cut) with its two exact savings named:
    /// - `skip_start`: the start pose was the end of the previous cut, so the
    ///   tool there has already removed everything it reaches (removal is
    ///   idempotent and stock only shrinks): its sub-pose is not cut again.
    /// - `prune`: a column line whose material has no interval inside the
    ///   tool's box along the line cannot change at this sub-pose, so it is
    ///   neither intersected nor counted as work. After roughing, most lines
    ///   through a finishing tool's box are empty there (C19 real plate:
    ///   large-bore-6 removes 12 mm³ while visiting 20.4 M lines).
    /// Neither changes the result: the stock is identical bit for bit.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn cut_with(&mut self, profile: &ToolProfile, from: ClPose, to: ClPose, plan: CutPlan, opts: &TriDexelOptions, tolerance: &Tolerance, work_left: &mut u64, runner: &mut Runner<'_>) -> Result<CutReport, CamError> {
        opts.validate()?;
        // Cutting bands: a band belongs to its lower point's part, so the
        // upper point of the last cutting band may already be tagged Shank.
        let bands: Vec<(ProfilePointPair, ProfilePointPair)> = profile
            .points
            .windows(2)
            .filter(|w| w[0].part == ToolPart::Cutting && w[1].height > w[0].height)
            .map(|w| ((w[0].radius, w[0].height), (w[1].radius, w[1].height)))
            .collect();
        let cutting_top = bands.iter().map(|b| b.1 .1).fold(0.0, f64::max);
        let r_max = bands.iter().flat_map(|b| [b.0 .0, b.1 .0]).fold(0.0, f64::max);
        let r_min_pos = bands.iter().flat_map(|b| [b.0 .0, b.1 .0]).filter(|r| *r > 0.0).fold(f64::INFINITY, f64::min);
        if r_max <= 0.0 || cutting_top <= 0.0 {
            return Err(CamError::InvalidInput("tri-dexel: the profile has no cutting part".into()));
        }
        let a0 = from.axis.normalized().ok_or_else(|| CamError::InvalidInput("tri-dexel: zero tool axis".into()))?;
        let a1 = to.axis.normalized().ok_or_else(|| CamError::InvalidInput("tri-dexel: zero tool axis".into()))?;
        let vertical = |a: Point3| a.x == 0.0 && a.y == 0.0 && a.z > 0.0;
        if plan.sweep && vertical(a0) && vertical(a1) && (from.tip.z - to.tip.z).abs() <= tolerance.distance_eps {
            return self.sweep_level(profile, &bands, cutting_top, r_max, from.tip, to.tip, plan, tolerance, work_left, runner);
        }
        let angle = a0.dot(a1).clamp(-1.0, 1.0).acos();
        let n = ((from.tip.distance(to.tip) / opts.step).ceil().max((angle / opts.max_axis_step_deg.to_radians()).ceil()) as u64).max(1);
        let sliver = if r_min_pos.is_finite() { opts.step * opts.step / (8.0 * r_min_pos) } else { 0.0 };
        let mut report = CutReport { sliver_bound: sliver, ..Default::default() };
        let mut removed = [0.0f64; 3];
        for s in 0..=n {
            if s == 0 && plan.skip_start && n > 0 {
                continue;
            }
            let f = s as f64 / n as f64;
            let tip = from.tip.lerp(to.tip, f);
            let axis = slerp(a0, a1, f);
            let x_ref = if axis.cross(Point3::X).length() <= tolerance.angle_eps.max(1e-12).sqrt() { Point3::Y } else { Point3::X };
            let to_local = RigidTransform::from_frame(tip, x_ref, axis)?;
            // Box of the tool at this pose.
            let top = tip.add(axis.scale(cutting_top));
            let tool_box = Bounds3::from_points([tip, top]).expanded(r_max + tolerance.distance_eps);
            for g in self.grids.iter_mut() {
                let Some([u0, u1, v0, v1]) = g.columns_in(&tool_box) else { continue };
                let axis_idx = g.axis;
                let mut d = [0.0; 3];
                d[axis_idx] = 1.0;
                let d_local = to_local.apply_vector(Point3::new(d[0], d[1], d[2]));
                let range0 = g.range[0];
                // The tool box along this grid's lines: material outside it cannot be reached.
                let along = |q: Point3| [q.x, q.y, q.z][axis_idx];
                let (reach_lo, reach_hi) = (along(tool_box.min), along(tool_box.max));
                let mut count = 0u64;
                for iv in v0..=v1 {
                    for iu in u0..=u1 {
                        if plan.prune && !g.column(iu, iv).intervals.iter().any(|&[a, b]| b > reach_lo && a < reach_hi) {
                            continue;
                        }
                        if *work_left == 0 {
                            return Err(CamError::InvalidInput("tri-dexel: work budget exceeded".into()));
                        }
                        *work_left -= 1;
                        count += 1;
                        let p0 = g.point(iu, iv, range0);
                        let q0 = to_local.apply_point(p0);
                        let intervals = line_in_profile(profile, q0, d_local);
                        if intervals.is_empty() {
                            continue;
                        }
                        let area = g.cell[0] * g.cell[1];
                        let col = g.column_mut(iu, iv);
                        for [t0, t1] in intervals {
                            removed[axis_idx] += col.cut(range0 + t0, range0 + t1) * area;
                        }
                    }
                }
                report.work += count;
                runner.work(count, "tridexel-cut", s, n + 1)?;
            }
            report.sub_poses += 1;
        }
        report.removed_volume = (removed[0] + removed[1] + removed[2]) / 3.0;
        Ok(report)
    }

    /// A LEVEL move of a vertical tool (constant tip Z and axis +Z), cut
    /// exactly with one visit per column line, so no sub-pose sliver is left:
    /// - a Z line meets the tool where the profile radius is at least the
    ///   line's distance from the axis; that set only shrinks as the distance
    ///   grows, so the union over the move is the cut at the closest approach;
    /// - an X or Y line lies at one height, where the tool is a disc of radius
    ///   R; the union of its chords over the move is a single interval whose
    ///   ends are the minimum of `s(t) - sqrt(R² - w(t)²)` and the maximum of
    ///   `s(t) + sqrt(...)` (convex and concave in t, with closed-form
    ///   stationary points), `s` along the line and `w` across it.
    #[allow(clippy::too_many_arguments)]
    fn sweep_level(
        &mut self,
        profile: &ToolProfile,
        bands: &[(ProfilePointPair, ProfilePointPair)],
        cutting_top: f64,
        r_max: f64,
        a: Point3,
        b: Point3,
        plan: CutPlan,
        tolerance: &Tolerance,
        work_left: &mut u64,
        runner: &mut Runner<'_>,
    ) -> Result<CutReport, CamError> {
        let mut report = CutReport { sliver_bound: 0.0, sub_poses: 1, ..Default::default() };
        let mut removed = [0.0f64; 3];
        let swept = Bounds3::from_points([a, b, a.add(Point3::Z.scale(cutting_top)), b.add(Point3::Z.scale(cutting_top))]).expanded(r_max + tolerance.distance_eps);
        let tz = a.z;
        for g in self.grids.iter_mut() {
            let Some([u0, u1, v0, v1]) = g.columns_in(&swept) else { continue };
            let axis_idx = g.axis;
            let along = |q: Point3| [q.x, q.y, q.z][axis_idx];
            let (reach_lo, reach_hi) = (along(swept.min), along(swept.max));
            let range0 = g.range[0];
            let area = g.cell[0] * g.cell[1];
            let mut count = 0u64;
            for iv in v0..=v1 {
                for iu in u0..=u1 {
                    if plan.prune && !g.column(iu, iv).intervals.iter().any(|&[lo, hi]| hi > reach_lo && lo < reach_hi) {
                        continue;
                    }
                    if *work_left == 0 {
                        return Err(CamError::InvalidInput("tri-dexel: work budget exceeded".into()));
                    }
                    *work_left -= 1;
                    count += 1;
                    let p0 = g.point(iu, iv, range0);
                    let cut: Vec<[f64; 2]> = if axis_idx == 2 {
                        // Closest approach of the axis to the line, then the exact line cut there.
                        let c = closest_on_segment(a, b, p0.x, p0.y);
                        line_in_profile(profile, Point3::new(p0.x - c.x, p0.y - c.y, range0 - tz), Point3::Z)
                            .into_iter()
                            .map(|[t0, t1]| [range0 + t0, range0 + t1])
                            .collect()
                    } else {
                        let Some(r) = radius_at(bands, p0.z - tz) else { continue };
                        // Along-line and across-line coordinates of the tool centre.
                        let across = if axis_idx == 0 { 1 } else { 0 };
                        let s_of = |q: Point3| [q.x, q.y, q.z][axis_idx];
                        let w_of = |q: Point3| [q.x, q.y, q.z][across];
                        let w_line = w_of(p0);
                        match chord_union(s_of(a), s_of(b), w_of(a) - w_line, w_of(b) - w_line, r) {
                            Some(iv) => vec![iv],
                            None => Vec::new(),
                        }
                    };
                    if cut.is_empty() {
                        continue;
                    }
                    let col = g.column_mut(iu, iv);
                    for [lo, hi] in cut {
                        removed[axis_idx] += col.cut(lo, hi) * area;
                    }
                }
            }
            report.work += count;
            runner.work(count, "tridexel-cut", 0, 1)?;
        }
        report.removed_volume = (removed[0] + removed[1] + removed[2]) / 3.0;
        Ok(report)
    }

    /// Run a motion stream: the first positional motion places the tool;
    /// every cutting motion cuts from the current pose. Arcs in any plane
    /// are cut along their chords (the Z-dexel removal's chord bound).
    pub fn simulate(&mut self, profile: &ToolProfile, motions: &[Motion], opts: &TriDexelOptions, tolerance: &Tolerance, runner: &mut Runner<'_>) -> Result<TriDexelReport, CamError> {
        opts.validate()?;
        let mut report = TriDexelReport::default();
        let mut work_left = opts.max_work;
        let mut at: Option<ClPose> = None;
        // The pose the last cut ended at: a cut starting there skips its start sub-pose.
        let mut cut_end: Option<ClPose> = None;
        runner.stage("tridexel-sim", "Tri-dexel simulation", 0, motions.len() as u64)?;
        for (i, m) in motions.iter().enumerate() {
            report.motions += 1;
            // An arc (any plane: the program's arc fit emits XZ/YZ arcs too) is
            // cut along its chords with the Z-dexel removal's chord bound and the
            // arc's own tool axis. It used to be refused, and the player then
            // resubmitted the refused chunk every frame (ABC36, the blade).
            if let MotionKind::ArcCw { to, center, plane, pitch } | MotionKind::ArcCcw { to, center, plane, pitch } = &m.kind {
                let axis = m.tool_axis.unwrap_or(Point3::Z);
                let end = ClPose { tip: *to, axis };
                match at {
                    Some(from) if m.kind.is_cutting() => {
                        let cw = matches!(m.kind, MotionKind::ArcCw { .. });
                        let chord = crate::stock_sim::removal::RemovalOptions::default().arc_chord;
                        let mut prev = from;
                        for q in crate::stock_sim::removal::arc_chords_in_plane(from.tip, *to, *center, *plane, cw, *pitch, chord) {
                            let pose = ClPose { tip: q, axis };
                            let plan = CutPlan { skip_start: cut_end == Some(prev), prune: true, sweep: true };
                            let c = self.cut_with(profile, prev, pose, plan, opts, tolerance, &mut work_left, runner)?;
                            report.removed_volume += c.removed_volume;
                            report.work += c.work;
                            report.sliver_bound = report.sliver_bound.max(c.sliver_bound);
                            cut_end = Some(pose);
                            prev = pose;
                        }
                        report.cutting_motions += 1;
                    }
                    None if m.kind.is_cutting() => report.warnings.push(Diagnostic::new("sim-no-start", format!("cutting motion {i} has no start position; not simulated"))),
                    _ => {}
                }
                at = Some(end);
                continue;
            }
            let Some(to) = m.end() else { continue };
            let axis = m.tool_axis.unwrap_or(Point3::Z);
            let pose = ClPose { tip: to, axis };
            if m.kind.is_cutting() {
                if let Some(from) = at {
                    let plan = CutPlan { skip_start: cut_end == Some(from), prune: true, sweep: true };
                    let c = self.cut_with(profile, from, pose, plan, opts, tolerance, &mut work_left, runner)?;
                    cut_end = Some(pose);
                    report.removed_volume += c.removed_volume;
                    report.work += c.work;
                    report.sliver_bound = report.sliver_bound.max(c.sliver_bound);
                    report.cutting_motions += 1;
                } else {
                    report.warnings.push(Diagnostic::new("sim-no-start", format!("cutting motion {i} has no start position; not simulated")));
                }
            }
            at = Some(pose);
        }
        Ok(report)
    }
}
