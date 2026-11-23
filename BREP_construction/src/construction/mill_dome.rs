//! A ball-end mill held square to a dome: what its finishing passes leave,
//! read along rays from the dome's centre.
//!
//! A five-axis finish holds a ball-end mill of radius `r` along the dome's
//! normal with its ball touching the dome of radius `R`: every ball centre
//! lies on the sphere of radius `R + r` about the dome's centre (here the
//! origin), and the mill's shank reaches away from it. Along the ray from
//! the origin in the direction `d`, a ball whose centre lies at the angle θ
//! from `d` is first met at
//!
//! `t(θ) = (R + r) cos θ − √(r² − (R + r)² sin² θ)`,
//!
//! which grows with θ. So the surface the balls leave is `t` of the angle
//! from `d` to the nearest centre: the passes enter only through the curves
//! their centres run on the unit sphere ([`Centres`]: great-circle arcs, as
//! a pass turned about a line through the origin runs, and circles about an
//! axis through it, as a pass turned about the table's axis runs).

use crate::Vec3;

/// A curve the ball centres' directions run on the unit sphere.
#[derive(Clone, Copy, Debug)]
pub enum CentreCurve {
    /// From the unit direction `from` turned right-handed about the unit
    /// `axis` (square to `from`) by `angle`: a great-circle arc.
    Arc { from: Vec3, axis: Vec3, angle: f64 },
    /// The whole circle of the directions at the angle `polar` from the unit
    /// `axis`.
    Circle { axis: Vec3, polar: f64 },
    /// One direction.
    Point(Vec3),
}

impl CentreCurve {
    /// The angle from the unit direction `d` to the nearest point of the curve.
    pub fn angle_to(&self, d: Vec3) -> f64 {
        let between = |a: Vec3, b: Vec3| a.cross(b).length().atan2(a.dot(b));
        match *self {
            CentreCurve::Point(c) => between(d, c),
            CentreCurve::Circle { axis, polar } => {
                let off = between(d, axis);
                (off - polar).abs()
            }
            CentreCurve::Arc { from, axis, angle } => {
                // d's projection on the arc's plane, read as an angle from `from`
                let side = axis.cross(from);
                let (x, y) = (d.dot(from), d.dot(side));
                // the turn from `from`, read the arc's way round, in [0, 2π)
                let at = (y.atan2(x) * angle.signum()).rem_euclid(std::f64::consts::TAU);
                let within = at <= angle.abs();
                if within {
                    let height = d.dot(axis);
                    height.abs().atan2((x * x + y * y).sqrt())
                } else {
                    let end = from.scale(angle.cos()).add(side.scale(angle.sin()));
                    between(d, from).min(between(d, end))
                }
            }
        }
    }
}

/// The finishing passes: the balls' radius, the dome's, and the curves their
/// centres' directions run (an arc turning by at most a whole turn).
#[derive(Clone, Debug)]
pub struct Centres {
    pub ball: f64,
    pub dome: f64,
    pub curves: Vec<CentreCurve>,
}

impl Centres {
    /// Where the ray from the origin along the unit `d` first meets a ball
    /// at the angle `theta` from it; None past the ball's reach.
    pub fn reach(&self, theta: f64) -> Option<f64> {
        let c = self.dome + self.ball;
        let s = c * theta.sin();
        (theta.abs() <= std::f64::consts::FRAC_PI_2 && s.abs() <= self.ball).then(|| c * theta.cos() - (self.ball * self.ball - s * s).sqrt())
    }

    /// The surface the balls leave along the unit `d`: the nearest centre
    /// curve and where its balls first meet the ray; None where none reaches.
    pub fn surface(&self, d: Vec3) -> Option<(f64, usize)> {
        let (index, theta) = self.curves.iter().enumerate().map(|(k, c)| (k, c.angle_to(d))).min_by(|a, b| a.1.total_cmp(&b.1))?;
        self.reach(theta).map(|t| (t, index))
    }
}

/// The chart of the upper half of the sphere of directions: the square
/// [-1, 1]² onto the unit disc (each side onto a quarter of its circle, by
/// the elliptical grid map), then the disc onto the half sphere by the
/// stereographic projection from the lower pole. The square's sides are the
/// directions of the level plane z = 0.
pub fn chart_direction(x: f64, y: f64) -> Vec3 {
    let (u, w) = (x * (1.0 - 0.5 * y * y).max(0.0).sqrt(), y * (1.0 - 0.5 * x * x).max(0.0).sqrt());
    let rho = u * u + w * w;
    let d = Vec3::new(2.0 * u, 2.0 * w, 1.0 - rho).scale(1.0 / (1.0 + rho));
    if x.abs() >= 1.0 || y.abs() >= 1.0 {
        // on the square's side: level, exactly
        let flat = Vec3::new(d.x, d.y, 0.0);
        return flat.scale(1.0 / flat.length());
    }
    d.scale(1.0 / d.length())
}

/// The chart point of the unit direction `d` (z ≥ 0): [`chart_direction`]'s
/// inverse.
pub fn direction_chart(d: Vec3) -> [f64; 2] {
    let (u, w) = (d.x / (1.0 + d.z), d.y / (1.0 + d.z));
    let r2 = 2.0f64.sqrt();
    let root = |a: f64| a.max(0.0).sqrt();
    let x = 0.5 * root(2.0 + u * u - w * w + 2.0 * r2 * u) - 0.5 * root(2.0 + u * u - w * w - 2.0 * r2 * u);
    let y = 0.5 * root(2.0 - u * u + w * w + 2.0 * r2 * w) - 0.5 * root(2.0 - u * u + w * w - 2.0 * r2 * w);
    [x.clamp(-1.0, 1.0), y.clamp(-1.0, 1.0)]
}

/// A finishing program's balls over the dome's upper half, as a field over
/// [`chart_direction`]'s chart: the height is the distance along the ray
/// from the origin. Its tubes are the arcs' tori (about each arc's axis) and
/// each circle's torus about its axis cut into `sectors` stretches (so no
/// face goes all the way round); its balls are each arc's start (an arc's end
/// on a circle lies inside that circle's torus) and each point.
pub struct DomeField {
    centres: Centres,
    arcs: Vec<(Vec3, Vec3, f64)>,
    circles: Vec<(Vec3, f64)>,
    sectors: usize,
    balls: Vec<(Vec3, Option<usize>)>,
    top: f64,
    /// The parts that may be lowest in each degree of latitude (from -10°)
    /// and longitude: those within reach of the bin's middle and its spread.
    bins: Vec<(Vec<usize>, Vec<usize>)>,
}

const BIN_LATITUDES: usize = 101;

use crate::mill_plan::{Field, Part};

impl DomeField {
    /// The field of `centres` (each circle about the z axis), cut at `top`.
    /// Refuses a circle about another axis.
    pub fn new(centres: Centres, sectors: usize, top: f64) -> Result<Self, String> {
        let (mut arcs, mut circles, mut balls) = (Vec::new(), Vec::new(), Vec::new());
        for curve in &centres.curves {
            match *curve {
                CentreCurve::Arc { from, axis, angle } => {
                    if !(angle > 0.0 && angle <= std::f64::consts::TAU) {
                        return Err("an arc must turn by a positive angle, at most a whole turn".into());
                    }
                    arcs.push((from, axis, angle));
                }
                CentreCurve::Circle { axis, polar } => {
                    if !(axis.x == 0.0 && axis.y == 0.0 && axis.z > 0.0) {
                        return Err("a circle must be about the z axis".into());
                    }
                    circles.push((axis, polar));
                }
                CentreCurve::Point(c) => balls.push((c, None)),
            }
        }
        let points = balls.len();
        // each arc's start, its own; an end on a circle is inside that circle's torus
        let mut starts: Vec<(Vec3, Option<usize>)> = arcs.iter().enumerate().map(|(k, &(from, _, _))| (from, Some(k))).collect();
        starts.extend(balls.drain(..points));
        for (k, &(from, axis, angle)) in arcs.iter().enumerate() {
            let end = from.scale(angle.cos()).add(axis.cross(from).scale(angle.sin()));
            if !circles.iter().any(|&(c, polar)| (end.dot(c).clamp(-1.0, 1.0).acos() - polar).abs() < 1e-12) {
                starts.push((end, Some(k)));
            }
        }
        let mut field = DomeField { centres, arcs, circles, sectors: sectors.max(1), balls: starts, top, bins: Vec::new() };
        let mut bins = Vec::with_capacity(BIN_LATITUDES * 360);
        for i in 0..BIN_LATITUDES {
            for j in 0..360 {
                let (lat, lon) = ((i as f64 - 10.0 + 0.5).to_radians(), (j as f64 + 0.5).to_radians());
                let d = Vec3::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin());
                // a bin's points lie within a degree of its middle
                bins.push(field.scan(d, 1f64.to_radians()));
            }
        }
        field.bins = bins;
        Ok(field)
    }

    /// The finishing passes it reads (fixed when it is made: its index of
    /// the parts in reach is built from them).
    pub fn centres(&self) -> &Centres {
        &self.centres
    }

    fn sphere(&self) -> f64 {
        self.centres.dome + self.centres.ball
    }

    /// The arc `k`'s foot angle of `d` (from its start, about its axis) and
    /// the angle of `d` from its plane.
    fn arc_angles(&self, k: usize, d: Vec3) -> (f64, f64) {
        let (from, axis, _) = self.arcs[k];
        let side = axis.cross(from);
        let (x, y) = (d.dot(from), d.dot(side));
        // the foot's turn from the start, in [0, 2π)
        (y.atan2(x).rem_euclid(std::f64::consts::TAU), d.dot(axis).abs().atan2((x * x + y * y).sqrt()))
    }

    fn sector_of(&self, d: Vec3) -> usize {
        let phi = d.y.atan2(d.x).rem_euclid(std::f64::consts::TAU);
        ((phi / std::f64::consts::TAU * self.sectors as f64) as usize).min(self.sectors - 1)
    }

    /// The tube `index`'s height along `d` and whether it is read there.
    fn tube(&self, index: usize, d: Vec3) -> (f64, bool) {
        if index < self.arcs.len() {
            let (foot, off) = self.arc_angles(index, d);
            let t = self.centres.reach(off).unwrap_or(f64::NAN);
            (t, (0.0..=self.arcs[index].2).contains(&foot))
        } else {
            let k = (index - self.arcs.len()) / self.sectors;
            let sector = (index - self.arcs.len()) % self.sectors;
            let (axis, polar) = self.circles[k];
            let off = (d.dot(axis).clamp(-1.0, 1.0).acos() - polar).abs();
            (self.centres.reach(off).unwrap_or(f64::NAN), self.sector_of(d) == sector)
        }
    }

    fn ball(&self, index: usize, d: Vec3) -> (f64, bool) {
        let (c, own) = self.balls[index];
        let angle = d.cross(c).length().atan2(d.dot(c));
        let t = self.centres.reach(angle).unwrap_or(f64::NAN);
        let covered = own.is_some_and(|k| self.tube(k, d).1);
        (t, !covered)
    }

    /// The parts that may be lowest along `d`: the arcs and starts within a
    /// few of the nearest's longitude, every circle and point near the pole.
    fn candidates(&self, d: Vec3) -> (Vec<usize>, Vec<usize>) {
        let lat = d.z.clamp(-1.0, 1.0).asin().to_degrees();
        let lon = d.y.atan2(d.x).to_degrees().rem_euclid(360.0);
        let i = ((lat + 10.0).floor().max(0.0) as usize).min(BIN_LATITUDES - 1);
        let j = (lon.floor() as usize).min(359);
        if lat >= -10.0 {
            return self.bins[i * 360 + j].clone();
        }
        self.scan(d, 0.0)
    }

    /// The parts within reach of `d`, and `spread` more.
    fn scan(&self, d: Vec3, spread: f64) -> (Vec<usize>, Vec<usize>) {
        let reach = (self.centres.ball / self.sphere()).asin() + 1e-9 + spread;
        let tubes: Vec<usize> = (0..self.arcs.len() + self.circles.len() * self.sectors)
            .filter(|&i| {
                if i < self.arcs.len() {
                    let (foot, off) = self.arc_angles(i, d);
                    off <= reach && (foot <= self.arcs[i].2 + reach || foot >= std::f64::consts::TAU - reach)
                } else {
                    let (axis, polar) = self.circles[(i - self.arcs.len()) / self.sectors];
                    (d.dot(axis).clamp(-1.0, 1.0).acos() - polar).abs() <= reach
                }
            })
            .collect();
        let balls: Vec<usize> = (0..self.balls.len()).filter(|&j| { let c = self.balls[j].0; d.cross(c).length().atan2(d.dot(c)) <= reach }).collect();
        (tubes, balls)
    }

    /// The point on the centre curve of `part` nearest `p` (the axis of the
    /// tube there, or the ball's centre).
    fn core(&self, part: Part, p: Vec3) -> Vec3 {
        let c = self.sphere();
        match part {
            Part::Tube(i) if i < self.arcs.len() => {
                let axis = self.arcs[i].1;
                let q = p.sub(axis.scale(p.dot(axis)));
                q.scale(c / q.length())
            }
            Part::Tube(i) => {
                let (_, polar) = self.circles[(i - self.arcs.len()) / self.sectors];
                let plan = (p.x * p.x + p.y * p.y).sqrt();
                let (cx, cy) = if plan > 0.0 { (p.x / plan, p.y / plan) } else { (1.0, 0.0) };
                Vec3::new(c * polar.sin() * cx, c * polar.sin() * cy, c * polar.cos())
            }
            Part::Ball(j) => self.balls[j].0.scale(c),
            _ => p,
        }
    }
}

impl Field for DomeField {
    fn scale(&self) -> f64 {
        self.centres.ball
    }
    fn length(&self) -> f64 {
        self.centres.dome
    }
    fn top(&self) -> f64 {
        self.top
    }
    fn lowest(&self, x: f64, y: f64) -> (f64, Part) {
        let d = chart_direction(x, y);
        let (tubes, balls) = self.candidates(d);
        let mut best = (self.top, Part::Top);
        let mut take = |z: f64, part: Part| {
            if z < best.0 || (z == best.0 && part < best.1) {
                best = (z, part);
            }
        };
        for i in tubes {
            let (t, read) = self.tube(i, d);
            if read && t.is_finite() {
                take(t, Part::Tube(i));
            }
        }
        for j in balls {
            let (t, read) = self.ball(j, d);
            if read && t.is_finite() {
                take(t, Part::Ball(j));
            }
        }
        best
    }
    fn height(&self, part: Part, x: f64, y: f64) -> f64 {
        let d = chart_direction(x, y);
        let (t, read) = match part {
            Part::Tube(i) => self.tube(i, d),
            Part::Ball(j) => self.ball(j, d),
            Part::Top => (self.top, true),
            Part::Outside => (f64::INFINITY, false),
        };
        if read && t.is_finite() { t } else { f64::INFINITY }
    }
    fn continued(&self, part: Part, x: f64, y: f64) -> f64 {
        let d = chart_direction(x, y);
        match part {
            Part::Tube(i) => self.tube(i, d).0,
            Part::Ball(j) => self.ball(j, d).0,
            Part::Top => self.top,
            Part::Outside => f64::NAN,
        }
    }
    /// An arc and its own start meet on the plane through the origin square
    /// to the arc there; two stretches of a circle's torus on the plane
    /// through the z axis between them.
    fn plane(&self, p: Part, q: Part, x: f64, y: f64) -> Option<f64> {
        let d = chart_direction(x, y);
        match (p, q) {
            (Part::Tube(i), Part::Ball(j)) | (Part::Ball(j), Part::Tube(i)) if self.balls[j].1 == Some(i) => {
                let (from, axis, angle) = self.arcs[i];
                let at = if self.balls[j].0.sub(from).length() < 1e-12 { 0.0 } else { angle };
                let tangent = axis.cross(from.scale(at.cos()).add(axis.cross(from).scale(at.sin())));
                Some(self.ball(j, d).0 * d.dot(tangent))
            }
            (Part::Tube(i), Part::Tube(k)) if i >= self.arcs.len() && k >= self.arcs.len() && (i - self.arcs.len()) / self.sectors == (k - self.arcs.len()) / self.sectors => {
                let (a, b) = ((i - self.arcs.len()) % self.sectors, (k - self.arcs.len()) % self.sectors);
                // the boundary between neighbouring stretches (the later's start)
                let later = if (a + 1) % self.sectors == b { b } else { a };
                let phi = std::f64::consts::TAU * later as f64 / self.sectors as f64;
                Some(self.tube(i, d).0 * (d.y * phi.cos() - d.x * phi.sin()))
            }
            _ => None,
        }
    }
    /// Each arc's end on a circle: the arc's torus and the circle's meet
    /// there, at the dome itself, and their boundary turns a corner.
    fn seeds(&self) -> Vec<[f64; 2]> {
        let mut seeds = Vec::new();
        for &(from, axis, angle) in &self.arcs {
            let end = from.scale(angle.cos()).add(axis.cross(from).scale(angle.sin()));
            if self.circles.iter().any(|&(c, polar)| (end.dot(c).clamp(-1.0, 1.0).acos() - polar).abs() < 1e-12) {
                seeds.push(direction_chart(end));
            }
        }
        seeds
    }

    fn witnesses(&self, cell: f64) -> Vec<[f64; 2]> {
        let mut points = Vec::new();
        let step = 0.5 * cell;
        for &(from, axis, angle) in &self.arcs {
            let count = ((angle / step).ceil() as usize).max(1);
            for k in 0..=count {
                let a = angle * k as f64 / count as f64;
                points.push(direction_chart(from.scale(a.cos()).add(axis.cross(from).scale(a.sin()))));
            }
        }
        for &(_, polar) in &self.circles {
            let count = ((std::f64::consts::TAU * polar.sin() / step).ceil() as usize).max(8);
            for k in 0..count {
                let phi = std::f64::consts::TAU * (k as f64 + 0.5) / count as f64;
                points.push(direction_chart(Vec3::new(polar.sin() * phi.cos(), polar.sin() * phi.sin(), polar.cos())));
            }
        }
        for &(c, _) in &self.balls {
            if c.z >= 0.0 {
                points.push(direction_chart(c));
            }
        }
        points
    }
}

use crate::mill_faces::Faces;
use crate::{make_arc, make_revolution, make_sphere_surface_framed, NurbsSurface};

impl Faces for DomeField {
    fn surface(&self, part: Part) -> Result<NurbsSurface, String> {
        let (c, r) = (self.sphere(), self.centres.ball);
        let origin = Vec3::new(0.0, 0.0, 0.0);
        match part {
            // the side of each torus toward the origin: its section within
            // 100° (an arc's) or less (a circle's, kept off its axis) of the
            // inward direction, turned through its arc or stretch
            Part::Tube(i) if i < self.arcs.len() => {
                let (from, axis, angle) = self.arcs[i];
                let section = make_arc(from.scale(c), from.scale(-1.0), axis, r, -100f64.to_radians(), 100f64.to_radians())?;
                make_revolution(origin, axis, &section, angle)
            }
            Part::Tube(i) => {
                let k = (i - self.arcs.len()) / self.sectors;
                let sector = (i - self.arcs.len()) % self.sectors;
                let (axis, polar) = self.circles[k];
                let phi = std::f64::consts::TAU * sector as f64 / self.sectors as f64;
                let n = Vec3::new(polar.sin() * phi.cos(), polar.sin() * phi.sin(), polar.cos());
                let north = Vec3::new(-polar.cos() * phi.cos(), -polar.cos() * phi.sin(), polar.sin());
                // within 100° of inward, short of the axis by a tenth of the
                // centre circle's radius (a spindle torus's section is kept
                // off the axis it would cross; a point past it is refused by
                // the inversion, not built wrong)
                let off = c * polar.sin();
                let clear = |beta: f64| (0..=40).all(|k| {
                    let b = -beta + 2.0 * beta * k as f64 / 40.0;
                    let q = n.scale(c).add(n.scale(-r * b.cos())).add(north.scale(r * b.sin()));
                    (q.x * q.x + q.y * q.y).sqrt() >= 0.1 * off
                });
                let mut limit = 100f64.to_radians();
                while limit > 1f64.to_radians() && !clear(limit) {
                    limit -= 0.5f64.to_radians();
                }
                let section = make_arc(n.scale(c), n.scale(-1.0), north, r, -limit, limit)?;
                make_revolution(origin, axis, &section, std::f64::consts::TAU / self.sectors as f64)
            }
            Part::Ball(j) => {
                let centre = self.balls[j].0;
                let side = centre.perpendicular()?;
                make_sphere_surface_framed(centre.scale(c), r, side, Some(centre))
            }
            Part::Top => Err("the dome's top is not cut".into()),
            Part::Outside => Err("no surface outside".into()),
        }
    }

    fn outward(&self, part: Part, at: Vec3) -> Vec3 {
        self.core(part, at).sub(at)
    }

    fn lift(&self, chart: Vec3) -> Vec3 {
        chart_direction(chart.x, chart.y).scale(chart.z)
    }
}

