//! The lower envelope of a ball-end mill's moves: the surface it leaves on
//! the stock's top.
//!
//! A ball-end mill's ball of radius `r` whose centre runs a straight segment
//! sweeps a CAPSULE: every point within `r` of the segment. Wherever the mill
//! cuts down into the stock's top and nothing of its shank reaches lower than
//! its ball (an upright mill, or one leaning away from the part as a
//! five-axis finish holds it), what is left of the stock is the stock below
//! the height field `h(x, y)`, the lowest point of any capsule above (x, y):
//! the LOWER ENVELOPE of the capsules. Over each capsule the underside is
//! exact: the lower half of its end balls, or the lower side of the cylinder
//! of radius `r` about its segment, where that point's foot on the segment
//! lies within it.
//!
//! This first piece reads the envelope exactly at a point ([`BallEnvelope::height`],
//! with which capsule and which part of it is lowest there), and integrates the
//! volume under it ([`BallEnvelope::volume_under`], the referee of the solid the
//! construction builds). [`crate::mill_plan`] reads which surface is lowest
//! where as a graph of exact boundary curves; the faces are built on it next.

use crate::Vec3;

/// The part of a capsule that is lowest at a point: its cylinder, or the ball
/// about one end (`false` the start, `true` the end).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Underside {
    Tube,
    Ball(bool),
}

/// Ball-end mill moves: balls of `radius` whose centres run the segments.
#[derive(Clone, Debug)]
pub struct BallEnvelope {
    radius: f64,
    /// Each move's ball centres, start to end.
    segments: Vec<(Vec3, Vec3)>,
    /// A uniform grid over the segments' plan boxes (each grown by the
    /// radius): cell (i, j) lists the segments whose box meets it.
    grid: Grid,
}

#[derive(Clone, Debug)]
struct Grid {
    origin: [f64; 2],
    cell: f64,
    size: [usize; 2],
    cells: Vec<Vec<usize>>,
}

/// The underside of the ball of `radius` about `centre` over (x, y); None
/// past its rim.
pub fn ball_underside(centre: Vec3, radius: f64, x: f64, y: f64) -> Option<f64> {
    let (dx, dy) = (x - centre.x, y - centre.y);
    let d2 = dx * dx + dy * dy;
    (d2 <= radius * radius).then(|| centre.z - (radius * radius - d2).sqrt())
}

impl BallEnvelope {
    /// The balls' radius.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// The moves' ball-centre segments, start to end.
    pub fn segments(&self) -> &[(Vec3, Vec3)] {
        &self.segments
    }

    /// The envelope of `segments` swept by balls of `radius`. Its envelope
    /// (None outside it): a radius in [1e-100, 1e100], and every coordinate
    /// within 1e6 radii of the origin. Within it a read is backward accurate:
    /// the point it returns lies on the capsule's surface to about 1e-10 of
    /// the radius. Its height alone can move further where the capsule's
    /// wall stands near upright, since there the height itself turns on the
    /// plan position that sharply.
    pub fn new(radius: f64, segments: Vec<(Vec3, Vec3)>) -> Option<Self> {
        if !(radius.is_finite() && (1e-100..=1e100).contains(&radius)) || segments.is_empty() {
            return None;
        }
        let bound = 1e6 * radius;
        let within = |p: &Vec3| [p.x, p.y, p.z].iter().all(|c| c.is_finite() && c.abs() <= bound);
        if !segments.iter().all(|(a, b)| within(a) && within(b)) {
            return None;
        }
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for (a, b) in &segments {
            for p in [a, b] {
                lo = [lo[0].min(p.x - radius), lo[1].min(p.y - radius)];
                hi = [hi[0].max(p.x + radius), hi[1].max(p.y + radius)];
            }
        }
        // Cells about the radius across, at most 1024 a side.
        let cell = radius.max((hi[0] - lo[0]).max(hi[1] - lo[1]) / 1024.0);
        // One cell more than the span needs a side, so a point on the boxes' far
        // edge (a capsule's outer reach) falls in a cell.
        let size = [((hi[0] - lo[0]) / cell).floor() as usize + 1, ((hi[1] - lo[1]) / cell).floor() as usize + 1];
        let mut cells = vec![Vec::new(); size[0] * size[1]];
        for (index, (a, b)) in segments.iter().enumerate() {
            let (x0, x1) = (a.x.min(b.x) - radius, a.x.max(b.x) + radius);
            let (y0, y1) = (a.y.min(b.y) - radius, a.y.max(b.y) + radius);
            let span = |v0: f64, v1: f64, axis: usize| {
                let first = (((v0 - lo[axis]) / cell).floor().max(0.0) as usize).min(size[axis] - 1);
                let last = (((v1 - lo[axis]) / cell).floor().max(0.0) as usize).min(size[axis] - 1);
                first..=last
            };
            for i in span(x0, x1, 0) {
                for j in span(y0, y1, 1) {
                    cells[i * size[1] + j].push(index);
                }
            }
        }
        Some(BallEnvelope { radius, segments, grid: Grid { origin: lo, cell, size, cells } })
    }

    /// The lowest point of the capsule `index` above (x, y), and which part
    /// of it that is; None where the capsule does not reach over (x, y).
    pub fn capsule_underside(&self, index: usize, x: f64, y: f64) -> Option<(f64, Underside)> {
        let [start, end, tube] = self.capsule_parts(index, x, y);
        let mut best: Option<(f64, Underside)> = None;
        for (z, part) in [(start, Underside::Ball(false)), (end, Underside::Ball(true)), (tube, Underside::Tube)] {
            if let Some(z) = z {
                if best.is_none_or(|(b, _)| z < b) {
                    best = Some((z, part));
                }
            }
        }
        best
    }

    /// Each part of the capsule `index` over (x, y): the underside of its
    /// start ball, of its end ball, and of its tube (None where that part
    /// does not reach: past a ball's rim, or where the tube's foot leaves the
    /// segment).
    pub fn capsule_parts(&self, index: usize, x: f64, y: f64) -> [Option<f64>; 3] {
        let (a, b) = self.segments[index];
        let tube = self.tube_root(index, x, y).and_then(|(z, foot)| (0.0..=1.0).contains(&foot).then_some(z));
        [ball_underside(a, self.radius, x, y), ball_underside(b, self.radius, x, y), tube]
    }

    /// The lower side of the whole line's cylinder about the capsule
    /// `index`'s segment over (x, y), and the foot of that point on the line
    /// (0 at the segment's start, 1 at its end); None where the line's
    /// cylinder does not reach over (x, y) or the segment has no length.
    pub fn tube_root(&self, index: usize, x: f64, y: f64) -> Option<(f64, f64)> {
        let (a, b) = self.segments[index];
        let r = self.radius;
        // The cylinder about the segment: points P = (x, y, a.z + Z) with
        // |(P − a)⊥| = r, the part square to the axis u. With v = (x − a.x,
        // y − a.y, 0) and e = (0, 0, 1), (P − a)⊥ = v⊥ + Z e⊥, so
        // |e⊥|² Z² + 2 (v⊥·e⊥) Z + |v⊥|² − r² = 0 (each part square to u
        // formed directly, never as a difference of squares); its lower root.
        let d = b.sub(a);
        let length = d.length();
        if !(length > 0.0) {
            return None;
        }
        let u = d.scale(1.0 / length);
        // Read about the axis point nearest the query's vertical line (in
        // plan), so every term below stands at the radius's scale: `shift`
        // along the axis from `a` (none for an axis near upright).
        let plan = u.x * u.x + u.y * u.y;
        let shift = if plan > 1e-24 { ((x - a.x) * u.x + (y - a.y) * u.y) / plan } else { 0.0 };
        let base = a.add(u.scale(shift));
        let v = Vec3::new(x - base.x, y - base.y, 0.0);
        let vu = v.dot(u);
        let v_perp = v.sub(u.scale(vu));
        let e_perp = Vec3::new(-u.z * u.x, -u.z * u.y, 1.0 - u.z * u.z);
        let qa = e_perp.dot(e_perp);
        let qb = 2.0 * v_perp.dot(e_perp);
        let qc = v_perp.dot(v_perp) - r * r;
        // A tangent read negative by rounding alone (short of zero by
        // under 1e-12 of 4·|e⊥|²·r², the radius's own scale) is a touch.
        let disc = qb * qb - 4.0 * qa * qc;
        let disc = if disc < 0.0 && -disc <= 1e-12 * 4.0 * qa * r * r { 0.0 } else { disc };
        if !(qa > 0.0 && disc >= 0.0) {
            return None;
        }
        // The two roots in the form that does not cancel, the lower one;
        // a touch (the discriminant zero, or clamped to it) its double root.
        let z_low = if disc == 0.0 {
            -qb / (2.0 * qa)
        } else {
            let q = -0.5 * (qb + qb.signum() * disc.sqrt());
            let (r1, r2) = (q / qa, if q != 0.0 { qc / q } else { q / qa });
            r1.min(r2)
        };
        Some((base.z + z_low, (shift + vu + z_low * u.z) / length))
    }

    /// The capsules whose plan boxes hold (x, y) (a superset of those that
    /// reach over it); none outside the grid.
    pub fn candidates(&self, x: f64, y: f64) -> &[usize] {
        let Grid { origin, cell, size, cells } = &self.grid;
        let i = ((x - origin[0]) / cell).floor();
        let j = ((y - origin[1]) / cell).floor();
        if !(i >= 0.0 && j >= 0.0 && (i as usize) < size[0] && (j as usize) < size[1]) {
            return &[];
        }
        &cells[i as usize * size[1] + j as usize]
    }

    /// The envelope at (x, y): the lowest point of any capsule above it, the
    /// capsule's index and its part; None where no capsule reaches.
    pub fn height(&self, x: f64, y: f64) -> Option<(f64, usize, Underside)> {
        let mut best: Option<(f64, usize, Underside)> = None;
        for &index in self.candidates(x, y) {
            if let Some((z, part)) = self.capsule_underside(index, x, y) {
                if best.is_none_or(|(b, _, _)| z < b) {
                    best = Some((z, index, part));
                }
            }
        }
        best
    }

    /// The volume of the box `[x0, x1] × [y0, y1] × [bottom, top]` below the
    /// envelope (the stock left where the capsules cut its top), by
    /// Gauss-Legendre on square cells of size `cell`, four
    /// points a side each. The envelope has creases (where two capsules meet),
    /// so the rule converges as the cell's square at best: two sizes and
    /// Richardson are the caller's.
    ///
    /// The box and the cell lie in the capsules' own envelope (None outside
    /// it): every bound within 1e6 radii of the origin, the box of positive
    /// extent, the cell at least 1e-6 radii.
    pub fn volume_under(&self, x0: f64, x1: f64, y0: f64, y1: f64, bottom: f64, top: f64, cell: f64) -> Option<f64> {
        let bound = 1e6 * self.radius;
        if !(cell.is_finite() && cell >= 1e-6 * self.radius) || ![x0, x1, y0, y1, bottom, top].iter().all(|c| c.is_finite() && c.abs() <= bound) || !(x1 > x0 && y1 > y0 && top >= bottom) {
            return None;
        }

        const X: [f64; 4] = [-0.861_136_311_594_052_6, -0.339_981_043_584_856_3, 0.339_981_043_584_856_3, 0.861_136_311_594_052_6];
        const W: [f64; 4] = [0.347_854_845_137_453_9, 0.652_145_154_862_546_1, 0.652_145_154_862_546_1, 0.347_854_845_137_453_9];
        // At most 1e8 cells, counted as they are iterated.
        let (fx, fy) = (((x1 - x0) / cell).ceil().max(1.0), ((y1 - y0) / cell).ceil().max(1.0));
        if !(fx * fy <= 1e8) {
            return None;
        }
        let (nx, ny) = (fx as usize, fy as usize);
        let (hx, hy) = ((x1 - x0) / nx as f64, (y1 - y0) / ny as f64);
        let mut total = 0.0;
        for i in 0..nx {
            for j in 0..ny {
                let (cx, cy) = (x0 + hx * (i as f64 + 0.5), y0 + hy * (j as f64 + 0.5));
                for (a, wa) in X.iter().zip(W) {
                    for (b, wb) in X.iter().zip(W) {
                        let (x, y) = (cx + 0.5 * hx * a, cy + 0.5 * hy * b);
                        let surface = self.height(x, y).map_or(top, |(z, _, _)| z.min(top)).max(bottom);
                        // Each point's share scaled by its cell's area as it is summed.
                        total += wa * wb * 0.25 * hx * hy * (surface - bottom);
                    }
                }
            }
        }
        Some(total)
    }
}

