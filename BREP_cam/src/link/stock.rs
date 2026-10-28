//! A conservative in-process stock bound for linking (owner repro from C20:
//! minimum retracts bounded only by the finished part crossed material still
//! to remove).
//!
//! The map covers the passes' XY extent with square cells. Every cell starts
//! at the operation's material top: material may remain anywhere up to it.
//! A cell is lowered only by a cutting motion of this operation whose swept
//! tool disc covers the WHOLE cell, and only to the tool's lower envelope
//! there (the tip height plus the profile height at the cell's farthest
//! corner). So a cell's value is always an upper bound of the material left
//! in it: a link, a feed height or a keep-down path that stays at or above
//! the bound over its whole tool footprint cannot touch uncut stock. Outside
//! the map the bound is the material top. A cut never lowers a cell whose
//! material may stand above the cutter's flutes there: the shank does not cut.
use crate::model::{Motion, MotionKind, Point3};
use crate::{Cutter, CutterProfile};

pub(crate) struct StockMap {
    top: f64,
    radius: f64,
    /// Flute length: material above the tip plus this is not cut.
    cutting_length: f64,
    origin: (f64, f64),
    cell: f64,
    nx: usize,
    ny: usize,
    tops: Vec<f64>,
    /// Lower envelope height at a radial distance, `None` past the radius.
    envelope: Vec<(f64, f64)>,
}

/// Distance in XY from `q` to the segment `a`-`b`.
fn segment_distance(q: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((q.0 - a.0) * dx + (q.1 - a.1) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    let (px, py) = (a.0 + t * dx, a.1 + t * dy);
    ((q.0 - px).powi(2) + (q.1 - py).powi(2)).sqrt()
}

impl StockMap {
    /// A map over `points` (the passes) for `cutter`, at most `max_cells`
    /// cells. Each cell starts at `material_top`, lowered to the `envelope`'s
    /// top over the cell's square (the stock before this operation), or at
    /// -inf where the envelope holds no material; with only an envelope the
    /// map's top is the envelope's highest material.
    pub(crate) fn new(points: impl Iterator<Item = Point3>, cutter: &Cutter, material_top: Option<f64>, envelope: Option<&crate::StockEnvelope>, max_cells: usize) -> StockMap {
        let top = material_top.or_else(|| envelope.and_then(|e| e.max_top())).unwrap_or(f64::NEG_INFINITY);
        let radius = cutter.radius();
        let (mut x0, mut y0, mut x1, mut y1) = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in points {
            x0 = x0.min(p.x);
            y0 = y0.min(p.y);
            x1 = x1.max(p.x);
            y1 = y1.max(p.y);
        }
        let pad = 2.0 * radius;
        let (w, h) = ((x1 - x0) + 2.0 * pad, (y1 - y0) + 2.0 * pad);
        // A cell drops only when ONE swept capsule contains it, so where two
        // passes share the coverage (ring corners) the cell must be smaller
        // than their overlap margin: an eighth of the radius.
        let cell = (radius / 8.0).max((w * h / max_cells as f64).sqrt()).max(1e-6);
        let nx = ((w / cell).ceil() as usize).max(1);
        let ny = ((h / cell).ceil() as usize).max(1);
        // The envelope at the radii the cell test asks for (monotone profiles
        // only: an undercut cutter clears nothing here, conservatively).
        let monotone = matches!(cutter.kind(), crate::cutter::CutterKind::Flat | crate::cutter::CutterKind::Ball | crate::cutter::CutterKind::Bull | crate::cutter::CutterKind::Cone | crate::cutter::CutterKind::BallCone | crate::cutter::CutterKind::Frustum);
        let profile = if monotone {
            (0..=64).filter_map(|k| { let r = radius * k as f64 / 64.0; cutter.height_at_radius(r).map(|h| (r, h)) }).collect()
        } else {
            Vec::new()
        };
        let origin = (x0 - pad, y0 - pad);
        let tops = match envelope {
            None => vec![top; nx * ny],
            Some(e) => {
                // A disc of the half-diagonal about the centre covers the cell's square.
                let half = cell * std::f64::consts::SQRT_2 * 0.5;
                (0..nx * ny)
                    .map(|k| {
                        let c = Point3::new(origin.0 + ((k % nx) as f64 + 0.5) * cell, origin.1 + ((k / nx) as f64 + 0.5) * cell, 0.0);
                        e.top_at(c, half).map_or(f64::NEG_INFINITY, |z| z.min(top))
                    })
                    .collect()
            }
        };
        StockMap { top, radius, cutting_length: cutter.cutting_length(), origin, cell, nx, ny, tops, envelope: profile }
    }

    /// Profile height at radius `d`, rounded UP to the next tabulated radius.
    fn envelope_at(&self, d: f64) -> Option<f64> {
        self.envelope.iter().find(|(r, _)| *r >= d).map(|(_, h)| *h)
    }

    fn cell_range(&self, lo: f64, hi: f64, origin: f64, n: usize) -> std::ops::Range<usize> {
        let a = ((lo - origin) / self.cell).floor().max(0.0) as usize;
        let b = (((hi - origin) / self.cell).ceil().max(0.0) as usize).min(n);
        a.min(n)..b
    }

    /// Lower the cells a straight tool move from `a` to `b` (tip positions)
    /// sweeps completely, with the disc shrunk by `shrink`.
    fn sweep_segment(&mut self, a: Point3, b: Point3, shrink: f64) {
        let r = self.radius - shrink;
        if r <= 0.0 || self.envelope.is_empty() {
            return;
        }
        let zmax = a.z.max(b.z);
        let xs = self.cell_range(a.x.min(b.x) - r, a.x.max(b.x) + r, self.origin.0, self.nx);
        let ys = self.cell_range(a.y.min(b.y) - r, a.y.max(b.y) + r, self.origin.1, self.ny);
        for iy in ys {
            for ix in xs.clone() {
                let cx = self.origin.0 + ix as f64 * self.cell;
                let cy = self.origin.1 + iy as f64 * self.cell;
                // Distance to a segment is convex: its maximum over the square is at a corner.
                let d = [(cx, cy), (cx + self.cell, cy), (cx, cy + self.cell), (cx + self.cell, cy + self.cell)]
                    .iter()
                    .map(|q| segment_distance(*q, (a.x, a.y), (b.x, b.y)))
                    .fold(0.0, f64::max);
                if d >= r {
                    continue;
                }
                let Some(h) = self.envelope_at(d) else { continue };
                let t = &mut self.tops[iy * self.nx + ix];
                // Material that may stand above the flutes is not cut here.
                if *t <= a.z.min(b.z) + self.cutting_length {
                    *t = t.min(zmax + h);
                }
            }
        }
    }

    /// Lower the map by the cutting motions in `motions`, starting at `from`.
    pub(crate) fn sweep(&mut self, mut from: Option<Point3>, motions: &[Motion]) {
        for m in motions {
            let Some(to) = m.end() else { continue };
            if let (Some(a), true) = (from, m.kind.is_cutting()) {
                match &m.kind {
                    MotionKind::ArcCw { .. } | MotionKind::ArcCcw { .. } => {
                        // Chords of an arc, each disc shrunk by its sagitta bound
                        // (the chord capsule of radius R - s lies inside the arc's sweep).
                        let n = 32;
                        let mut prev = a;
                        for k in 1..=n {
                            let q = crate::arcfit::arc_point(a, &m.kind, k as f64 / n as f64).unwrap_or(to);
                            let mid = crate::arcfit::arc_point(a, &m.kind, (k as f64 - 0.5) / n as f64).unwrap_or(q);
                            let sagitta = segment_distance((mid.x, mid.y), (prev.x, prev.y), (q.x, q.y));
                            self.sweep_segment(prev, q, sagitta);
                            prev = q;
                        }
                    }
                    _ => self.sweep_segment(a, to, 0.0),
                }
            }
            from = Some(to);
        }
    }

    /// Highest material the tool footprint (radius) can meet on the straight
    /// XY move from `a` to `b`: cells within reach of the capsule, conservatively.
    pub(crate) fn chord(&self, a: Point3, b: Point3) -> f64 {
        // A cell counts when its square can meet the footprint: its centre
        // within the radius plus half the cell's diagonal.
        let reach = self.radius + self.cell * std::f64::consts::SQRT_2 * 0.5;
        let (lo_x, hi_x, lo_y, hi_y) = (a.x.min(b.x) - reach, a.x.max(b.x) + reach, a.y.min(b.y) - reach, a.y.max(b.y) + reach);
        let (mx1, my1) = (self.origin.0 + self.nx as f64 * self.cell, self.origin.1 + self.ny as f64 * self.cell);
        if lo_x < self.origin.0 || lo_y < self.origin.1 || hi_x > mx1 || hi_y > my1 {
            return self.top;
        }
        let mut z = f64::NEG_INFINITY;
        for iy in self.cell_range(lo_y, hi_y, self.origin.1, self.ny) {
            for ix in self.cell_range(lo_x, hi_x, self.origin.0, self.nx) {
                let c = (self.origin.0 + (ix as f64 + 0.5) * self.cell, self.origin.1 + (iy as f64 + 0.5) * self.cell);
                if segment_distance(c, (a.x, a.y), (b.x, b.y)) <= reach {
                    z = z.max(self.tops[iy * self.nx + ix]);
                }
            }
        }
        z
    }

    /// Material may stand up to `z` within `reach` (XY) of the polyline
    /// `points`: every cell whose square comes that close is raised to at
    /// least `z` (a trimmed span the cutter could not take).
    pub(crate) fn raise(&mut self, points: &[Point3], reach: f64, z: f64) {
        let near = reach + self.cell * std::f64::consts::SQRT_2 * 0.5;
        let segments: Vec<(Point3, Point3)> = if points.len() == 1 { vec![(points[0], points[0])] } else { points.windows(2).map(|w| (w[0], w[1])).collect() };
        for (a, b) in segments {
            let xs = self.cell_range(a.x.min(b.x) - near, a.x.max(b.x) + near, self.origin.0, self.nx);
            let ys = self.cell_range(a.y.min(b.y) - near, a.y.max(b.y) + near, self.origin.1, self.ny);
            for iy in ys {
                for ix in xs.clone() {
                    let c = (self.origin.0 + (ix as f64 + 0.5) * self.cell, self.origin.1 + (iy as f64 + 0.5) * self.cell);
                    if segment_distance(c, (a.x, a.y), (b.x, b.y)) <= near {
                        let t = &mut self.tops[iy * self.nx + ix];
                        *t = t.max(z);
                    }
                }
            }
        }
    }

    pub(crate) fn point(&self, p: Point3) -> f64 {
        self.chord(p, p)
    }
}

