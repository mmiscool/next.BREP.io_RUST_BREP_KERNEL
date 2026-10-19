//! The CONTROL CAGE of a NURBS solid (`feature_pipeline::features::nurbs_solid`).
//!
//! A cage is a lattice of control points wrapped around a primitive: `M`
//! RINGS of `N` points each, closed around the body's axis (+Y, the polar
//! axis every other primitive uses), plus one POLE point at either end. The body is the bicubic B-spline surface those points
//! control — the smooth "subdivision" of the cage, lying inside it the way a
//! subdivision surface lies inside its control mesh — closed into a solid the
//! way the kernel's sphere is: one face, one seam edge, two degenerate pole
//! edges. Every point is free: dragging it deforms the body smoothly, and new
//! LOOPS of points (a ring, or a column running pole to pole) are inserted by
//! knot insertion, which refines the cage WITHOUT changing the body — the same
//! surface, with more handles on it.
//!
//! # Parameterisation
//!
//! The around direction `u` is PERIODIC: a ring stores `N` distinct points
//! and `N + 1` cumulative knots `t_0 = 0 < … < t_N = period`, extended
//! cyclically (`t_{i+N} = t_i + period`). Point `c` of a ring carries the
//! basis supported on `[t_c, t_{c+4}]`, so the curve over span `[t_k, t_{k+1}]`
//! blends points `k-3 … k` (cyclic). The kernel's surfaces need CLAMPED knot
//! vectors, so [`ControlCage::surface`] converts each row by unwrapping the
//! periodic sequence and inserting the seam knot to full multiplicity (Boehm),
//! which keeps the surface identical and C² across the seam — a deformed cage
//! shows no crease where the seam edge runs.
//!
//! The pole-to-pole direction `v` is OPEN and clamped over `M + 2` rows: the
//! bottom pole, the rings bottom-up, the top pole. A pole row is `N` copies of
//! the pole point, so the surface passes through the pole exactly and the two
//! pole edges are degenerate, exactly as on the sphere.
//!
//! # Point indexing
//!
//! Every editor and script addresses a point by ONE linear index:
//! `0` is the bottom pole, `1 + r·N + c` is column `c` of ring `r` (ring `0`
//! nearest the bottom pole), and `1 + M·N` is the top pole. The wrap column
//! the clamped surface carries and the collapsed pole rows are never exposed —
//! they are derived here, once.

use crate::curve::parameter_line;
use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{make_line, NurbsSurface, Vec3, Vec4};
use serde_json::Value;

/// The B-spline degree in both directions: bicubic, the subdivision-surface
/// smoothness (C² everywhere a knot is simple).
pub const CAGE_DEGREE: usize = 3;
/// The fewest points a ring may have: a cubic periodic ring needs four.
pub const MIN_SEGMENTS: usize = 4;
/// The fewest rings: with the two poles that is `degree + 1` rows, one span.
pub const MIN_RINGS: usize = 2;
/// Caps on the initial cage so a typo cannot ask for a million handles.
pub const MAX_SEGMENTS: usize = 64;
pub const MAX_RINGS: usize = 64;

/// The primitive the initial cage is wrapped around.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CagePrimitive {
    /// Rings at even latitudes of the ellipsoid inscribed in the size box.
    Sphere,
    /// Elliptical rings (in XZ) at even heights along Y, the end rings in
    /// the end planes.
    Cylinder,
    /// Rectangular rings (in XZ) at even heights along Y, corners first.
    Box,
}

impl CagePrimitive {
    /// The schema's option names (case-insensitive on read).
    pub const OPTIONS: [&'static str; 3] = ["BOX", "CYLINDER", "SPHERE"];

    pub fn parse(name: &str) -> Option<Self> {
        match name.trim().to_ascii_uppercase().as_str() {
            "SPHERE" | "ELLIPSOID" => Some(Self::Sphere),
            "CYLINDER" => Some(Self::Cylinder),
            "BOX" | "CUBE" => Some(Self::Box),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Sphere => "SPHERE",
            Self::Cylinder => "CYLINDER",
            Self::Box => "BOX",
        }
    }
}

/// Where a linear point index lands in the cage.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CagePoint {
    /// The bottom (`0`) or top (`1`) pole.
    Pole(usize),
    /// Column `column` of ring `ring` (ring `0` nearest the bottom pole).
    Ring { ring: usize, column: usize },
}

/// The cage: see the module docs for the layout.
#[derive(Clone, Debug)]
pub struct ControlCage {
    /// `N + 1` cumulative periodic knots, `knots_u[0] = 0`, `knots_u[N]` the period.
    pub knots_u: Vec<f64>,
    /// Clamped cubic knots over the `M + 2` rows: `M + 6` values.
    pub knots_v: Vec<f64>,
    /// The bottom and top pole points.
    pub poles: [Vec3; 2],
    /// `M` rings of `N` points, bottom-up.
    pub rings: Vec<Vec<Vec3>>,
}

/// Knots closer than this fraction of the period are one knot.
const KNOT_TOL: f64 = 1e-9;

impl ControlCage {
    // --- construction --------------------------------------------------------

    /// The initial cage around a primitive filling the `size` box (centred on
    /// the origin, poles on the Y axis), with `segments` points per ring and
    /// `rings` rings. Rings run counter-clockwise about +Y seen from above
    /// (from +X towards −Z), so `Su × Sv` points outward.
    pub fn primitive(
        kind: CagePrimitive,
        size: [f64; 3],
        segments: usize,
        rings: usize,
    ) -> Result<Self, String> {
        if !(MIN_SEGMENTS..=MAX_SEGMENTS).contains(&segments) {
            return Err(format!(
                "control cage: segments must be between {MIN_SEGMENTS} and {MAX_SEGMENTS}, got {segments}"
            ));
        }
        if !(MIN_RINGS..=MAX_RINGS).contains(&rings) {
            return Err(format!(
                "control cage: rings must be between {MIN_RINGS} and {MAX_RINGS}, got {rings}"
            ));
        }
        let half = [size[0].abs() * 0.5, size[1].abs() * 0.5, size[2].abs() * 0.5];
        if half.iter().any(|h| !h.is_finite() || *h <= 1e-9) {
            return Err(format!("control cage: every size must be positive, got {size:?}"));
        }
        let n = segments;
        let m = rings;
        let mut out = Vec::with_capacity(m);
        for r in 0..m {
            // Rings are spaced so that ring 0 and ring M-1 are the END rings
            // of a cylinder / box (in the end planes), and even latitudes
            // (poles excluded) on the sphere.
            let fraction = if m == 1 { 0.5 } else { r as f64 / (m - 1) as f64 };
            let ring: Vec<Vec3> = (0..n)
                .map(|c| {
                    let phi = std::f64::consts::TAU * c as f64 / n as f64;
                    let height = -half[1] + 2.0 * half[1] * fraction;
                    match kind {
                        CagePrimitive::Sphere => {
                            let theta = std::f64::consts::PI * (r + 1) as f64 / (m + 1) as f64;
                            Vec3::new(
                                half[0] * theta.sin() * phi.cos(),
                                -half[1] * theta.cos(),
                                -half[2] * theta.sin() * phi.sin(),
                            )
                        }
                        CagePrimitive::Cylinder => {
                            Vec3::new(half[0] * phi.cos(), height, -half[2] * phi.sin())
                        }
                        CagePrimitive::Box => {
                            // Corners first: the first point sits on the +X−Z
                            // corner, so 4 segments give the four corners and 8
                            // add the edge midpoints.
                            let angle = phi + std::f64::consts::FRAC_PI_4;
                            let (s, c) = angle.sin_cos();
                            let scale = 1.0 / c.abs().max(s.abs());
                            Vec3::new(half[0] * c * scale, height, -half[2] * s * scale)
                        }
                    }
                })
                .collect();
            out.push(ring);
        }
        let poles = [Vec3::new(0.0, -half[1], 0.0), Vec3::new(0.0, half[1], 0.0)];
        let knots_u = (0..=n).map(|i| i as f64 / n as f64).collect();
        let knots_v = crate::uniform_clamped_knots(m + 2, CAGE_DEGREE)?;
        let cage = Self {
            knots_u,
            knots_v,
            poles,
            rings: out,
        };
        cage.validate()?;
        Ok(cage)
    }

    /// Every structural invariant: ring sizes, knot counts, monotone knots,
    /// finite points. A persisted cage is checked through this before use.
    pub fn validate(&self) -> Result<(), String> {
        let n = self.segments();
        let m = self.rings.len();
        if n < MIN_SEGMENTS {
            return Err(format!("control cage: a ring needs at least {MIN_SEGMENTS} points, got {n}"));
        }
        if m < MIN_RINGS {
            return Err(format!("control cage: needs at least {MIN_RINGS} rings, got {m}"));
        }
        if self.rings.iter().any(|ring| ring.len() != n) {
            return Err("control cage: every ring must have the same number of points".into());
        }
        if self.knots_u.len() != n + 1 {
            return Err(format!(
                "control cage: expected {} periodic knots for {n} segments, got {}",
                n + 1,
                self.knots_u.len()
            ));
        }
        if self.knots_v.len() != m + 2 + CAGE_DEGREE + 1 {
            return Err(format!(
                "control cage: expected {} clamped knots for {m} rings, got {}",
                m + 2 + CAGE_DEGREE + 1,
                self.knots_v.len()
            ));
        }
        let finite = |knots: &[f64]| knots.iter().all(|k| k.is_finite());
        if !finite(&self.knots_u) || !finite(&self.knots_v) {
            return Err("control cage: knots must be finite".into());
        }
        if self.knots_u[0].abs() > KNOT_TOL || self.period() <= KNOT_TOL {
            return Err("control cage: periodic knots must start at 0 and span a positive period".into());
        }
        if self.knots_u.windows(2).any(|w| w[1] <= w[0] + KNOT_TOL * self.period()) {
            return Err("control cage: periodic knots must be strictly increasing".into());
        }
        if self.knots_v.windows(2).any(|w| w[1] < w[0] - KNOT_TOL) {
            return Err("control cage: ring knots must be non-decreasing".into());
        }
        let [v0, v1] = self.domain_v();
        if v1 - v0 <= KNOT_TOL {
            return Err("control cage: the ring knots span no interval".into());
        }
        for index in 0..=CAGE_DEGREE {
            let last = self.knots_v.len() - 1 - index;
            if (self.knots_v[index] - self.knots_v[0]).abs() > KNOT_TOL
                || (self.knots_v[last] - self.knots_v[self.knots_v.len() - 1]).abs() > KNOT_TOL
            {
                return Err("control cage: ring knots must be clamped".into());
            }
        }
        // A run of more than `degree` equal interior knots breaks the basis.
        let mut run = 1;
        for w in self.knots_v[CAGE_DEGREE + 1..self.knots_v.len() - CAGE_DEGREE - 1].windows(2) {
            run = if (w[1] - w[0]).abs() <= KNOT_TOL { run + 1 } else { 1 };
            if run > CAGE_DEGREE {
                return Err("control cage: an interior ring knot repeats more than the degree".into());
            }
        }
        let finite_point = |p: &Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
        if !self.poles.iter().all(finite_point) || !self.rings.iter().flatten().all(finite_point) {
            return Err("control cage: every point must be finite".into());
        }
        Ok(())
    }

    // --- shape -----------------------------------------------------------------

    /// Points per ring (`N`).
    pub fn segments(&self) -> usize {
        self.rings.first().map_or(0, Vec::len)
    }

    /// Rings (`M`).
    pub fn ring_count(&self) -> usize {
        self.rings.len()
    }

    /// Every free point: the rings' plus the two poles.
    pub fn point_count(&self) -> usize {
        self.rings.len() * self.segments() + 2
    }

    /// The period of the around direction.
    pub fn period(&self) -> f64 {
        self.knots_u[self.knots_u.len() - 1]
    }

    /// The clamped `v` domain.
    pub fn domain_v(&self) -> [f64; 2] {
        [self.knots_v[CAGE_DEGREE], self.knots_v[self.knots_v.len() - 1 - CAGE_DEGREE]]
    }

    /// Where linear index `index` lands.
    pub fn locate(&self, index: usize) -> Option<CagePoint> {
        let n = self.segments();
        let m = self.rings.len();
        if index == 0 {
            return Some(CagePoint::Pole(0));
        }
        if index == 1 + m * n {
            return Some(CagePoint::Pole(1));
        }
        if index > 1 + m * n {
            return None;
        }
        let offset = index - 1;
        Some(CagePoint::Ring {
            ring: offset / n,
            column: offset % n,
        })
    }

    /// The linear index of column `column` of ring `ring`.
    pub fn index_of(&self, ring: usize, column: usize) -> usize {
        1 + ring * self.segments() + column
    }

    /// The linear index of pole `which` (0 bottom, 1 top).
    pub fn pole_index(&self, which: usize) -> usize {
        if which == 0 {
            0
        } else {
            1 + self.rings.len() * self.segments()
        }
    }

    pub fn point(&self, index: usize) -> Option<Vec3> {
        match self.locate(index)? {
            CagePoint::Pole(which) => Some(self.poles[which]),
            CagePoint::Ring { ring, column } => Some(self.rings[ring][column]),
        }
    }

    pub fn set_point(&mut self, index: usize, point: Vec3) -> Result<(), String> {
        match self
            .locate(index)
            .ok_or_else(|| format!("control cage: no point {index}"))?
        {
            CagePoint::Pole(which) => self.poles[which] = point,
            CagePoint::Ring { ring, column } => self.rings[ring][column] = point,
        }
        Ok(())
    }

    /// Every point in linear order.
    pub fn points(&self) -> Vec<Vec3> {
        let mut out = Vec::with_capacity(self.point_count());
        out.push(self.poles[0]);
        for ring in &self.rings {
            out.extend_from_slice(ring);
        }
        out.push(self.poles[1]);
        out
    }

    /// The `M + 2` rows the surface is built from: a pole row is `N` copies of
    /// its pole.
    fn rows(&self) -> Vec<Vec<Vec3>> {
        let n = self.segments();
        let mut rows = Vec::with_capacity(self.rings.len() + 2);
        rows.push(vec![self.poles[0]; n]);
        rows.extend(self.rings.iter().cloned());
        rows.push(vec![self.poles[1]; n]);
        rows
    }

    /// The cyclic periodic knot `t_i` for any integer `i`.
    fn knot_u(&self, i: isize) -> f64 {
        let n = self.segments() as isize;
        let q = i.div_euclid(n);
        let r = i.rem_euclid(n) as usize;
        self.knots_u[r] + q as f64 * self.period()
    }

    /// The Greville abscissa of column `column` — the `u` the point "sits" at.
    pub fn greville_u(&self, column: usize) -> f64 {
        let c = column as isize;
        (self.knot_u(c + 1) + self.knot_u(c + 2) + self.knot_u(c + 3)) / 3.0
    }

    /// The Greville abscissa of row `row` (`0` the bottom pole, `M + 1` the top).
    pub fn greville_v(&self, row: usize) -> f64 {
        (self.knots_v[row + 1] + self.knots_v[row + 2] + self.knots_v[row + 3]) / 3.0
    }

    // --- loop insertion --------------------------------------------------------

    /// Insert a ring of points between rows `row` and `row + 1` (`row` in
    /// `0 ..= M`, row `0` being the bottom pole), by inserting the `v` knot
    /// half-way between their Greville abscissae. The surface is unchanged.
    /// Returns the index (`0`-based, among the rings) of the new ring.
    pub fn insert_ring_between(&mut self, row: usize) -> Result<usize, String> {
        let m = self.rings.len();
        if row > m {
            return Err(format!("control cage: no row pair {row}/{} to insert between", row + 1));
        }
        let v = 0.5 * (self.greville_v(row) + self.greville_v(row + 1));
        self.insert_ring_at(v)
    }

    /// Insert a ring at parameter `v` (strictly inside the `v` domain).
    pub fn insert_ring_at(&mut self, v: f64) -> Result<usize, String> {
        let [v0, v1] = self.domain_v();
        if !(v > v0 + KNOT_TOL && v < v1 - KNOT_TOL) {
            return Err(format!("control cage: ring parameter {v} is outside ({v0}, {v1})"));
        }
        let n = self.segments();
        let rows = self.rows();
        let mut new_rows: Vec<Vec<Vec3>> = Vec::new();
        let mut new_knots = self.knots_v.clone();
        for column in 0..n {
            let column_points: Vec<Vec3> = rows.iter().map(|row| row[column]).collect();
            let (points, knots) = insert_knot_open(&column_points, &self.knots_v, CAGE_DEGREE, v)?;
            if new_rows.is_empty() {
                new_rows = vec![Vec::with_capacity(n); points.len()];
                new_knots = knots;
            }
            for (row, point) in new_rows.iter_mut().zip(points) {
                row.push(point);
            }
        }
        // Row 0 and the last row are the untouched poles (Boehm never blends
        // the first or last control point); every row between is a ring.
        let count = new_rows.len();
        self.rings = new_rows[1..count - 1].to_vec();
        self.knots_v = new_knots;
        // The new handle: the ring whose Greville abscissa is nearest `v`.
        let nearest = (0..self.rings.len())
            .min_by(|a, b| {
                (self.greville_v(a + 1) - v)
                    .abs()
                    .total_cmp(&(self.greville_v(b + 1) - v).abs())
            })
            .unwrap_or(0);
        self.validate()?;
        Ok(nearest)
    }

    /// Insert a column of points (pole to pole) between columns `column` and
    /// `column + 1` (cyclic), half-way between their Greville abscissae. The
    /// surface is unchanged. Returns the new column's index.
    pub fn insert_column_between(&mut self, column: usize) -> Result<usize, String> {
        let n = self.segments();
        if column >= n {
            return Err(format!("control cage: no column {column}"));
        }
        let c = column as isize;
        let g0 = (self.knot_u(c + 1) + self.knot_u(c + 2) + self.knot_u(c + 3)) / 3.0;
        let g1 = (self.knot_u(c + 2) + self.knot_u(c + 3) + self.knot_u(c + 4)) / 3.0;
        let u = 0.5 * (g0 + g1);
        self.insert_column_at(u.rem_euclid(self.period()))
    }

    /// Insert a column at periodic parameter `u` (reduced into `[0, period)`).
    pub fn insert_column_at(&mut self, u: f64) -> Result<usize, String> {
        let n = self.segments();
        let period = self.period();
        let mut u = u.rem_euclid(period);
        // Snap away from an existing knot: a repeated knot would reduce the
        // continuity there, and the new column would sit ON an old one.
        if let Some(k) = self
            .knots_u
            .iter()
            .position(|t| (t - u).abs() <= KNOT_TOL * period)
        {
            let next = self.knot_u(k as isize + 1);
            u = 0.5 * (self.knots_u[k] + next);
            if u >= period {
                u -= period;
            }
        }
        // The span [t_k, t_{k+1}) holding u.
        let k = (0..n)
            .find(|&k| self.knots_u[k] <= u && u < self.knots_u[k + 1])
            .ok_or_else(|| format!("control cage: column parameter {u} is outside the period"))?
            as isize;
        let new_len = n + 1;
        let mut new_rings = Vec::with_capacity(self.rings.len());
        for ring in &self.rings {
            let p = |i: isize| ring[i.rem_euclid(n as isize) as usize];
            let mut out = vec![Vec3::default(); new_len];
            for j in (k - 2)..=(k - 2 + n as isize) {
                let value = if j <= k {
                    let t_j = self.knot_u(j);
                    let alpha = (u - t_j) / (self.knot_u(j + 3) - t_j);
                    p(j).scale(alpha).add(p(j - 1).scale(1.0 - alpha))
                } else {
                    p(j - 1)
                };
                out[j.rem_euclid(new_len as isize) as usize] = value;
            }
            new_rings.push(out);
        }
        let ku = k as usize;
        let mut knots = Vec::with_capacity(n + 2);
        knots.extend_from_slice(&self.knots_u[..=ku]);
        knots.push(u);
        knots.extend_from_slice(&self.knots_u[ku + 1..]);
        self.rings = new_rings;
        self.knots_u = knots;
        self.validate()?;
        // The new handle: the column whose Greville abscissa is nearest `u`.
        let nearest = (0..new_len)
            .min_by(|&a, &b| {
                let da = cyclic_distance(self.greville_u(a), u, period);
                let db = cyclic_distance(self.greville_u(b), u, period);
                da.total_cmp(&db)
            })
            .unwrap_or(0);
        Ok(nearest)
    }

    // --- the surface and the solid --------------------------------------------

    /// One row of the cage as a CLAMPED cubic B-spline around the seam:
    /// `N + 3` points and `N + 7` knots over `[0, period]`, the same curve
    /// as the periodic one (see the module docs).
    fn clamped_row(&self, row: &[Vec3]) -> Result<(Vec<Vec3>, Vec<f64>), String> {
        let n = self.segments();
        let p = CAGE_DEGREE;
        // Unwrap: point j carries P_{(j - p) mod N}, knot j is t_{j - p}.
        let points: Vec<Vec3> = (0..n + p).map(|j| row[(j + n - p) % n]).collect();
        let knots: Vec<f64> = (0..n + 2 * p + 1)
            .map(|j| self.knot_u(j as isize - p as isize))
            .collect();
        let start = self.knot_u(0);
        let end = self.knot_u(n as isize);
        let (points, knots) = insert_knot_open(&points, &knots, p, start)?;
        let (points, knots) = insert_knot_open(&points, &knots, p, start)?;
        let (points, knots) = insert_knot_open(&points, &knots, p, end)?;
        let (mut points, mut knots) = insert_knot_open(&points, &knots, p, end)?;
        // The p - 1 points and knots outside the domain on either side carry
        // nothing of the curve on it; the outermost surviving knot is then the
        // clamp knot by value.
        let drop = p - 1;
        points.drain(..drop);
        points.truncate(points.len() - drop);
        knots.drain(..drop);
        knots.truncate(knots.len() - drop);
        knots[0] = start;
        let last = knots.len() - 1;
        knots[last] = end;
        debug_assert_eq!(points.len(), n + p);
        debug_assert_eq!(knots.len(), n + 2 * p + 1);
        Ok((points, knots))
    }

    /// The bicubic surface the cage controls, over `[0, 1] × [0, 1]`: `u`
    /// around (clamped at the seam), `v` from the bottom pole to the top.
    pub fn surface(&self) -> Result<NurbsSurface, String> {
        self.validate()?;
        let n = self.segments();
        let rows = self.rows();
        let mut knots_u: Option<Vec<f64>> = None;
        let mut columns: Vec<Vec<Vec4>> = vec![Vec::with_capacity(rows.len()); n + CAGE_DEGREE];
        for row in &rows {
            let (points, knots) = self.clamped_row(row)?;
            if knots_u.is_none() {
                let period = self.period();
                knots_u = Some(knots.iter().map(|k| (k / period).clamp(0.0, 1.0)).collect());
            }
            for (column, point) in columns.iter_mut().zip(points) {
                column.push(Vec4::from_point(point, 1.0));
            }
        }
        let [v0, v1] = self.domain_v();
        let knots_v: Vec<f64> = self
            .knots_v
            .iter()
            .map(|k| ((k - v0) / (v1 - v0)).clamp(0.0, 1.0))
            .collect();
        NurbsSurface::new(
            CAGE_DEGREE,
            CAGE_DEGREE,
            knots_u.expect("at least four rows"),
            knots_v,
            columns,
        )
    }

    /// The closed solid: one face carrying `face_name`, a seam edge at `u = 0`
    /// and a degenerate edge at each pole — the sphere's topology on this
    /// surface. Validated before it is returned.
    pub fn solid(&self, face_name: Option<&str>) -> Result<BrepSolid, String> {
        let surface = self.surface()?;
        let south = surface.evaluate(0.0, 0.0)?;
        let north = surface.evaluate(0.0, 1.0)?;
        let seam = surface.iso_curve_u(0.0)?;
        let solid = BrepSolid {
            mass_properties_cache: Default::default(),
            cells: Default::default(),
            id: 120,
            vertices: vec![
                VertexRecord { id: 1, point: south },
                VertexRecord { id: 2, point: north },
            ],
            edges: vec![
                EdgeRecord {
                    id: 10,
                    curve: seam,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 1,
                    end_vertex_id: 2,
                    degenerate: false,
                    name: None,
                },
                EdgeRecord {
                    id: 11,
                    curve: make_line(south, south)?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 1,
                    end_vertex_id: 1,
                    degenerate: true,
                    name: None,
                },
                EdgeRecord {
                    id: 12,
                    curve: make_line(north, north)?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 2,
                    end_vertex_id: 2,
                    degenerate: true,
                    name: None,
                },
            ],
            shells: vec![ShellRecord {
                id: 119,
                faces: vec![FaceRecord {
                    id: 118,
                    surface,
                    same_sense: true,
                    loops: vec![LoopRecord {
                        id: 117,
                        coedges: vec![
                            CoedgeRecord {
                                id: 101,
                                edge_id: 11,
                                forward: true,
                                pcurve: parameter_line(0.0, 0.0, 1.0, 0.0)?,
                            },
                            CoedgeRecord {
                                id: 102,
                                edge_id: 10,
                                forward: true,
                                pcurve: parameter_line(1.0, 0.0, 1.0, 1.0)?,
                            },
                            CoedgeRecord {
                                id: 103,
                                edge_id: 12,
                                forward: true,
                                pcurve: parameter_line(1.0, 1.0, 0.0, 1.0)?,
                            },
                            CoedgeRecord {
                                id: 104,
                                edge_id: 10,
                                forward: false,
                                pcurve: parameter_line(0.0, 1.0, 0.0, 0.0)?,
                            },
                        ],
                    }],
                    name: face_name.map(str::to_string),
                }],
            }],
            genus: 0,
            kind: crate::BodyKind::Solid,
        };
        let issues = solid.validate();
        if !issues.is_empty() {
            return Err(format!("control cage solid has invalid topology: {issues:?}"));
        }
        Ok(solid)
    }

    // --- persistence -----------------------------------------------------------

    /// The persisted shape: `{ knotsU, knotsV, poles: [[x,y,z],[x,y,z]],
    /// rings: [[[x,y,z], …], …] }`.
    pub fn to_json(&self) -> Value {
        let point = |p: &Vec3| serde_json::json!([p.x, p.y, p.z]);
        serde_json::json!({
            "knotsU": self.knots_u,
            "knotsV": self.knots_v,
            "poles": [point(&self.poles[0]), point(&self.poles[1])],
            "rings": self.rings.iter().map(|ring| ring.iter().map(point).collect::<Vec<_>>()).collect::<Vec<_>>(),
        })
    }

    /// Read a persisted cage, validating it.
    pub fn from_json(value: &Value) -> Result<Self, String> {
        let numbers = |key: &str| -> Result<Vec<f64>, String> {
            value
                .get(key)
                .and_then(Value::as_array)
                .ok_or_else(|| format!("control cage: `{key}` must be an array"))?
                .iter()
                .map(|n| n.as_f64().ok_or_else(|| format!("control cage: `{key}` holds a non-number")))
                .collect()
        };
        let point = |p: &Value, what: &str| -> Result<Vec3, String> {
            let xyz = p
                .as_array()
                .filter(|a| a.len() == 3)
                .ok_or_else(|| format!("control cage: {what} must be [x, y, z]"))?;
            let read = |i: usize| xyz[i].as_f64().ok_or_else(|| format!("control cage: {what} holds a non-number"));
            Ok(Vec3::new(read(0)?, read(1)?, read(2)?))
        };
        let poles = value
            .get("poles")
            .and_then(Value::as_array)
            .filter(|a| a.len() == 2)
            .ok_or("control cage: `poles` must hold two points")?;
        let rings = value
            .get("rings")
            .and_then(Value::as_array)
            .ok_or("control cage: `rings` must be an array")?
            .iter()
            .map(|ring| {
                ring.as_array()
                    .ok_or_else(|| "control cage: a ring must be an array".to_string())?
                    .iter()
                    .map(|p| point(p, "a ring point"))
                    .collect::<Result<Vec<_>, _>>()
            })
            .collect::<Result<Vec<_>, _>>()?;
        let cage = Self {
            knots_u: numbers("knotsU")?,
            knots_v: numbers("knotsV")?,
            poles: [point(&poles[0], "a pole")?, point(&poles[1], "a pole")?],
            rings,
        };
        cage.validate()?;
        Ok(cage)
    }
}

/// Boehm knot insertion on an OPEN B-spline (clamped or not): insert `u` once
/// into `knots`, returning the refined points and knots. `u` must lie in the
/// domain `[knots[p], knots[n]]`; an existing multiplicity up to `p` is fine
/// (the affected blends then degenerate to copies, as the algorithm says).
fn insert_knot_open(
    points: &[Vec3],
    knots: &[f64],
    degree: usize,
    u: f64,
) -> Result<(Vec<Vec3>, Vec<f64>), String> {
    let n = points.len();
    let p = degree;
    if knots.len() != n + p + 1 {
        return Err(format!(
            "insert_knot: {} knots do not fit {n} points of degree {p}",
            knots.len()
        ));
    }
    let [start, end] = [knots[p], knots[n]];
    if u < start - KNOT_TOL || u > end + KNOT_TOL {
        return Err(format!("insert_knot: {u} is outside the domain [{start}, {end}]"));
    }
    // The span: the largest k in [p, n - 1] with knots[k] <= u.
    let mut k = p;
    for (i, pair) in knots.windows(2).enumerate().take(n).skip(p) {
        if pair[0] <= u {
            k = i;
        }
    }
    let mut out = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let value = if i + p <= k {
            points[i]
        } else if i <= k {
            let denominator = knots[i + p] - knots[i];
            let alpha = if denominator.abs() <= f64::EPSILON { 0.0 } else { (u - knots[i]) / denominator };
            points[i].scale(alpha).add(points[i - 1].scale(1.0 - alpha))
        } else {
            points[i - 1]
        };
        out.push(value);
    }
    let mut new_knots = Vec::with_capacity(knots.len() + 1);
    new_knots.extend_from_slice(&knots[..=k]);
    new_knots.push(u);
    new_knots.extend_from_slice(&knots[k + 1..]);
    Ok((out, new_knots))
}

fn cyclic_distance(a: f64, b: f64, period: f64) -> f64 {
    let d = (a - b).rem_euclid(period);
    d.min(period - d)
}

