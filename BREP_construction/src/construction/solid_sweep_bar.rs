//! A BAR as the tool of a screw sweep ([`crate::SweptTool::Bar`]): the region
//! a closed chain of plane curves bounds, extended along the plane's normal,
//! its sides leaning inward by a draft angle.
//!
//! The sweep's engine needs three things of a tool: its faces as patches over
//! a parameter box, its sharp edges as curves, and a membership test that is
//! also a lower bound on the distance to the tool from outside. Here
//!
//! - the section at height `h` is the given section moved inward by
//!   `d = h·tan(draft)` along its own normals, so a side is
//!   `C(s) − d·ν(s)` lifted to `h` (a straight piece runs between its two
//!   ends' moved joints, mitred where the joint is a corner);
//! - membership is `sd(q) + d ≤ 0` with `sd` the signed distance to the
//!   section in its plane, which is that same inward offset wherever the offset
//!   stays short of the section's centres of curvature (checked), and
//!   `(sd + d)·cos(draft)` is a lower bound on the distance to the leaning
//!   side, since `sd + d` changes by at most `1/cos(draft)` per unit length.
//!
//! The distance in the plane is exact: the nearest chord of a polyline through
//! each curve seeds a Newton iteration for the curve's own nearest point.
use crate::{KernelRefusal, KernelStage, NurbsCurve, Vec3};

/// Chords per knot span of a curved piece (seeds only: every distance is
/// refined on the curve itself).
const CHORDS_PER_SPAN: usize = 32;

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{}: {message}", crate::solid_sweep::SOLID_SWEEP_REFUSAL))
}

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{}: {message}", crate::solid_sweep::SOLID_SWEEP_REFUSAL))
}

struct Piece {
    curve: NurbsCurve,
    domain: [f64; 2],
    /// A straight segment: its two ends are all there is to it.
    straight: bool,
    /// Section points, evenly spaced within each knot span (first and last are
    /// the ends), so no span is too short to be seen.
    samples: Vec<[f64; 2]>,
    /// The samples' places along the piece, in [0, 1].
    places: Vec<f64>,
    /// Bounding box of the samples, grown by the piece's sag.
    low: [f64; 2],
    high: [f64; 2],
}

/// The start of a piece: where it leaves the piece before it.
struct Joint {
    point: [f64; 2],
    /// The joint at height `h` is `point − h·taper·mitre`: the outward normal
    /// where the pieces meet smoothly, the mitre of the two where they do not.
    mitre: [f64; 2],
    corner: bool,
}

pub(crate) struct Bar {
    origin: Vec3,
    n: Vec3,
    xa: Vec3,
    ya: Vec3,
    pub from: f64,
    pub to: f64,
    /// tan(draft): how far the section moves inward per unit of height.
    taper: f64,
    cosine: f64,
    pieces: Vec<Piece>,
    joints: Vec<Joint>,
    /// One closed curve with no corner: its side is one face, periodic round it.
    pub periodic: bool,
    /// +1 where the section runs counter-clockwise in (xa, ya), −1 clockwise.
    turn: f64,
    /// How far a chord can stand off its curve.
    sag: f64,
    /// The section's box, grown by as far as the draft moves it outward.
    low: [f64; 2],
    high: [f64; 2],
}

fn sub2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn length2(a: [f64; 2]) -> f64 {
    a[0].hypot(a[1])
}

/// Distance from `q` to the segment `a b`, and the fraction along it of the
/// nearest point.
fn to_segment(q: [f64; 2], a: [f64; 2], b: [f64; 2]) -> (f64, f64) {
    let d = sub2(b, a);
    let span = dot2(d, d);
    let w = if span > 0.0 { (dot2(sub2(q, a), d) / span).clamp(0.0, 1.0) } else { 0.0 };
    (length2(sub2([a[0] + d[0] * w, a[1] + d[1] * w], q)), w)
}

impl Bar {
    /// The bar on `section` (curves end to end round a closed loop, in the
    /// plane through `origin` square to `normal`), from height `from` to `to`
    /// along `normal`, its sides leaning inward by `draft` radians as the
    /// height grows from the section's own plane.
    pub fn new(origin: Vec3, normal: Vec3, section: &[NurbsCurve], from: f64, to: f64, draft: f64) -> Result<Bar, KernelRefusal> {
        let n = normal.normalized().map_err(|_| input("bar_normal", "the bar's direction is zero".into()))?;
        if section.is_empty() {
            return Err(input("section_empty", "the bar's section is empty".into()));
        }
        if !(from.is_finite() && to.is_finite() && to > from) {
            return Err(input("bar_range", "the bar's heights must be increasing".into()));
        }
        if !(draft.is_finite() && draft.abs() < 1.5) {
            return Err(input("bar_draft", "the bar's draft must be an angle short of a right angle".into()));
        }
        let seed = if n.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
        let xa = seed.sub(n.scale(seed.dot(n))).normalized_or(seed);
        let ya = n.cross(xa);
        let flat = |x: Vec3| -> ([f64; 2], f64) {
            let d = x.sub(origin);
            ([d.dot(xa), d.dot(ya)], d.dot(n))
        };
        // A curve that turns a corner inside itself (a knot where its direction
        // jumps: a polygon as one curve) is as many pieces as it has smooth runs.
        let mut runs: Vec<NurbsCurve> = Vec::with_capacity(section.len());
        for curve in section {
            let domain = curve.domain().map_err(|error| input("section_curve", format!("a section curve has no domain: {error}")))?;
            let width = domain[1] - domain[0];
            let mut inner: Vec<f64> = curve.knots.iter().copied().filter(|k| *k > domain[0] && *k < domain[1]).collect();
            inner.dedup();
            let direction = |t: f64| -> Option<Vec3> { curve.derivatives(t, 1).ok()?.get(1)?.normalized().ok() };
            let mut rest = curve.clone();
            let mut tail: Vec<NurbsCurve> = Vec::new();
            for knot in inner.into_iter().rev() {
                let (Some(before), Some(after)) = (direction(knot - 1e-9 * width), direction(knot + 1e-9 * width)) else { continue };
                if before.cross(after).length() < 1e-6 && before.dot(after) > 0.0 {
                    continue;
                }
                let (left, right) = rest.split(knot).map_err(|error| input("section_curve", error))?;
                tail.push(right);
                rest = left;
            }
            runs.push(rest);
            runs.extend(tail.into_iter().rev());
        }
        let mut pieces = Vec::with_capacity(runs.len());
        let mut size = 0.0f64;
        let mut off_plane = 0.0f64;
        for curve in &runs {
            let domain = curve.domain().map_err(|error| input("section_curve", format!("a section curve has no domain: {error}")))?;
            // the knots that part the curve's spans, ends included
            let mut breaks: Vec<f64> = vec![domain[0]];
            breaks.extend(curve.knots.iter().copied().filter(|k| *k > domain[0] && *k < domain[1]));
            breaks.push(domain[1]);
            breaks.dedup();
            let ends = (
                curve.evaluate(domain[0]).map_err(|error| input("section_curve", error))?,
                curve.evaluate(domain[1]).map_err(|error| input("section_curve", error))?,
            );
            let chord = ends.1.sub(ends.0).length();
            let straight = curve.straight_segment(1e-12 * chord.max(1e-300)).is_some();
            let mut places = vec![0.0];
            if straight {
                places.push(1.0);
            } else {
                for span in breaks.windows(2) {
                    for k in 1..=CHORDS_PER_SPAN {
                        let t = span[0] + (span[1] - span[0]) * k as f64 / CHORDS_PER_SPAN as f64;
                        places.push(((t - domain[0]) / (domain[1] - domain[0])).min(1.0));
                    }
                }
                *places.last_mut().expect("places") = 1.0;
            }
            let mut samples = Vec::with_capacity(places.len());
            for place in &places {
                let t = domain[0] + (domain[1] - domain[0]) * place;
                let (q, h) = flat(curve.evaluate(t).map_err(|error| input("section_curve", error))?);
                off_plane = off_plane.max(h.abs());
                size = size.max(q[0].abs()).max(q[1].abs());
                samples.push(q);
            }
            pieces.push(Piece { curve: curve.clone(), domain, straight, samples, places, low: [0.0; 2], high: [0.0; 2] });
        }
        if off_plane > 1e-9 * size.max(1.0) {
            return Err(input("section_plane", "the bar's section does not lie in the plane through its origin square to its direction".into()));
        }
        let count = pieces.len();
        for k in 0..count {
            let end = *pieces[k].samples.last().expect("samples");
            let start = pieces[(k + 1) % count].samples[0];
            if length2(sub2(end, start)) > 1e-9 * size.max(1.0) {
                return Err(input("outline_open", format!("the bar's section is not closed after curve {k}")));
            }
        }
        let area: f64 = pieces
            .iter()
            .flat_map(|piece| piece.samples.windows(2))
            .map(|pair| 0.5 * cross2(pair[0], pair[1]))
            .sum();
        if area.abs() <= 1e-18 * size * size {
            return Err(input("section_flat", "the bar's section bounds no area".into()));
        }
        let turn = area.signum();
        let taper = draft.tan();
        let mut bar = Bar {
            origin,
            n,
            xa,
            ya,
            from,
            to,
            taper,
            cosine: draft.cos(),
            pieces,
            joints: Vec::new(),
            periodic: false,
            turn,
            sag: 0.0,
            low: [f64::INFINITY; 2],
            high: [f64::NEG_INFINITY; 2],
        };
        // How far the chords stand off: read at each chord's middle, doubled.
        let mut sag = 0.0f64;
        for index in 0..count {
            let piece = &bar.pieces[index];
            if piece.straight {
                continue;
            }
            for k in 0..piece.samples.len() - 1 {
                // a quarter and three quarters along too: a chord can cross its curve at the middle
                for w in [0.25, 0.5, 0.75] {
                    let place = piece.places[k] + (piece.places[k + 1] - piece.places[k]) * w;
                    sag = sag.max(to_segment(bar.point(index, place).0, piece.samples[k], piece.samples[k + 1]).0);
                }
            }
        }
        bar.sag = 2.0 * sag + 1e-12 * size;
        for piece in &mut bar.pieces {
            let mut low = [f64::INFINITY; 2];
            let mut high = [f64::NEG_INFINITY; 2];
            for q in &piece.samples {
                for axis in 0..2 {
                    low[axis] = low[axis].min(q[axis] - bar.sag);
                    high[axis] = high[axis].max(q[axis] + bar.sag);
                }
            }
            piece.low = low;
            piece.high = high;
        }
        // The joints, and what the draft may do at each.
        let (d0, d1) = (taper * from, taper * to);
        let grown = (-d0).max(-d1).max(0.0);
        for index in 0..count {
            let before = (index + count - 1) % count;
            let (point, t1) = bar.point(index, 0.0);
            let (_, t0) = bar.point(before, 1.0);
            let (n0, n1) = (bar.outward(t0), bar.outward(t1));
            let (sine, cosine) = (cross2(t0, t1) / (length2(t0) * length2(t1)), dot2(n0, n1));
            let corner = !(sine.abs() < 1e-6 && cosine > 0.0);
            let mitre = if corner {
                if 1.0 + cosine < 1e-6 {
                    return Err(refuse("section_cusp", format!("the bar's section turns straight back at the start of curve {index}")));
                }
                [(n0[0] + n1[0]) / (1.0 + cosine), (n0[1] + n1[1]) / (1.0 + cosine)]
            } else {
                n1
            };
            if corner && taper != 0.0 {
                // A drafted side is the section's offset only where the offset keeps the
                // corner sharp: inward of a corner that sticks out, outward of one that
                // points in, between two straight sides.
                let convex = sine * turn > 0.0;
                let keeps = if convex { d0 >= 0.0 && d1 >= 0.0 } else { d0 <= 0.0 && d1 <= 0.0 };
                if !(bar.pieces[index].straight && bar.pieces[before].straight && keeps) {
                    return Err(refuse(
                        "drafted_corner",
                        format!("the bar's section has a corner at the start of curve {index} that its draft does not keep sharp between two straight sides"),
                    ));
                }
            }
            bar.joints.push(Joint { point, mitre, corner });
        }
        bar.periodic = count == 1 && !bar.joints[0].corner;
        // The draft must stop short of every centre of curvature it moves toward.
        if taper != 0.0 {
            for index in 0..count {
                let piece = &bar.pieces[index];
                if piece.straight {
                    continue;
                }
                let chords = piece.samples.len() - 1;
                for k in 0..=4 * chords {
                    let (chord, w) = ((k / 4).min(chords - 1), if k == 4 * chords { 1.0 } else { (k % 4) as f64 / 4.0 });
                    let (_, d1v, d2v) = bar.derivatives(index, piece.places[chord] + (piece.places[chord + 1] - piece.places[chord]) * w);
                    let speed = length2(d1v);
                    if speed <= 0.0 {
                        continue;
                    }
                    // signed curvature, positive where the section bulges outward
                    let curvature = turn * cross2(d1v, d2v) / (speed * speed * speed);
                    if 1.0 - d0 * curvature <= 1e-9 || 1.0 - d1 * curvature <= 1e-9 {
                        return Err(refuse(
                            "draft_past_curvature",
                            format!("the bar's draft moves curve {index} of its section past its own centre of curvature"),
                        ));
                    }
                }
            }
        }
        for piece in &bar.pieces {
            for axis in 0..2 {
                bar.low[axis] = bar.low[axis].min(piece.low[axis] - grown);
                bar.high[axis] = bar.high[axis].max(piece.high[axis] + grown);
            }
        }
        Ok(bar)
    }

    pub fn piece_count(&self) -> usize {
        self.pieces.len()
    }

    /// Point, first and second derivative of piece `index` at `s ∈ [0, 1]`, in the plane.
    fn derivatives(&self, index: usize, s: f64) -> ([f64; 2], [f64; 2], [f64; 2]) {
        let piece = &self.pieces[index];
        let width = piece.domain[1] - piece.domain[0];
        let t = piece.domain[0] + width * s.clamp(0.0, 1.0);
        let flat = |v: Vec3| [v.dot(self.xa), v.dot(self.ya)];
        match piece.curve.derivatives(t, 2) {
            Ok(values) if values.len() >= 2 => {
                let second = values.get(2).copied().unwrap_or(Vec3::new(0.0, 0.0, 0.0));
                (
                    flat(values[0].sub(self.origin)),
                    [flat(values[1])[0] * width, flat(values[1])[1] * width],
                    [flat(second)[0] * width * width, flat(second)[1] * width * width],
                )
            }
            _ => {
                // the chord that holds `s`
                let k = piece.places.partition_point(|place| *place <= s).clamp(1, piece.places.len() - 1) - 1;
                let d = sub2(piece.samples[k + 1], piece.samples[k]);
                let step = (piece.places[k + 1] - piece.places[k]).max(1e-300);
                let w = (s - piece.places[k]) / step;
                ([piece.samples[k][0] + d[0] * w, piece.samples[k][1] + d[1] * w], [d[0] / step, d[1] / step], [0.0; 2])
            }
        }
    }

    /// Point and tangent (not unit) of piece `index` at `s`.
    fn point(&self, index: usize, s: f64) -> ([f64; 2], [f64; 2]) {
        let (point, tangent, _) = self.derivatives(index, s);
        (point, tangent)
    }

    /// The unit normal out of the section, across a tangent.
    fn outward(&self, tangent: [f64; 2]) -> [f64; 2] {
        let length = length2(tangent).max(1e-300);
        [self.turn * tangent[1] / length, -self.turn * tangent[0] / length]
    }

    fn lift(&self, q: [f64; 2], h: f64) -> Vec3 {
        self.origin.add(self.xa.scale(q[0])).add(self.ya.scale(q[1])).add(self.n.scale(h))
    }

    fn joint_in_plane(&self, index: usize, h: f64) -> [f64; 2] {
        let joint = &self.joints[index % self.joints.len()];
        let d = self.taper * h;
        [joint.point[0] - d * joint.mitre[0], joint.point[1] - d * joint.mitre[1]]
    }

    /// The upright edge at the start of piece `index`, at height `h`.
    pub fn joint(&self, index: usize, h: f64) -> Vec3 {
        self.lift(self.joint_in_plane(index, h), h)
    }

    /// The side on piece `index` at `s ∈ [0, 1]` round it and height `h`.
    pub fn side(&self, index: usize, s: f64, h: f64) -> Vec3 {
        let q = if self.pieces[index].straight {
            let (a, b) = (self.joint_in_plane(index, h), self.joint_in_plane(index + 1, h));
            [a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s]
        } else {
            let (point, tangent) = self.point(index, s);
            let normal = self.outward(tangent);
            let d = self.taper * h;
            [point[0] - d * normal[0], point[1] - d * normal[1]]
        };
        self.lift(q, h)
    }

    /// Is `q` inside the polyline through the section (even-odd)?
    fn inside_chords(&self, q: [f64; 2]) -> bool {
        let mut crossings = 0usize;
        for piece in &self.pieces {
            for pair in piece.samples.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                if (a[1] > q[1]) != (b[1] > q[1]) {
                    let x = a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
                    if x > q[0] {
                        crossings += 1;
                    }
                }
            }
        }
        crossings % 2 == 1
    }

    /// The nearest point of piece `index` to `q`, from the chord `k` it was
    /// seeded on at fraction `w`: (distance, s).
    fn nearest_on(&self, index: usize, q: [f64; 2], k: usize, w: f64) -> (f64, f64) {
        let places = &self.pieces[index].places;
        let (low, high) = (places[k.saturating_sub(1)], places[(k + 2).min(places.len() - 1)]);
        let seed = places[k] + (places[k + 1] - places[k]) * w;
        let distance = |s: f64| length2(sub2(self.point(index, s).0, q));
        let mut s = seed;
        for _ in 0..24 {
            let (point, d1, d2) = self.derivatives(index, s);
            let r = sub2(point, q);
            let slope = dot2(r, d1);
            let curve = dot2(d1, d1) + dot2(r, d2);
            if !(curve > 0.0) {
                break;
            }
            let next = (s - slope / curve).clamp(low, high);
            let moved = (next - s).abs();
            s = next;
            if moved < 1e-15 {
                break;
            }
        }
        let (found, seeded) = (distance(s), distance(seed));
        if found <= seeded { (found, s) } else { (seeded, seed) }
    }

    /// Signed distance to the section in its plane (negative inside).
    fn section_distance(&self, q: [f64; 2]) -> f64 {
        // nearest chord of each piece
        let mut best: Vec<(f64, usize, usize, f64)> = Vec::with_capacity(self.pieces.len());
        let mut least = f64::INFINITY;
        for (index, piece) in self.pieces.iter().enumerate() {
            let dx = (piece.low[0] - q[0]).max(q[0] - piece.high[0]).max(0.0);
            let dy = (piece.low[1] - q[1]).max(q[1] - piece.high[1]).max(0.0);
            if dx.hypot(dy) > least + 4.0 * self.sag {
                continue;
            }
            let mut own = (f64::INFINITY, 0usize, 0.0f64);
            for (k, pair) in piece.samples.windows(2).enumerate() {
                let (d, w) = to_segment(q, pair[0], pair[1]);
                if d < own.0 {
                    own = (d, k, w);
                }
            }
            least = least.min(own.0);
            best.push((own.0, index, own.1, own.2));
        }
        // the curve's own nearest point, on every piece that could hold it
        let mut nearest = (f64::INFINITY, 0usize, 0.0f64);
        for &(chord, index, k, w) in &best {
            if chord > least + 4.0 * self.sag {
                continue;
            }
            let (distance, s) = if self.pieces[index].straight { (chord, w) } else { self.nearest_on(index, q, k, w) };
            if distance < nearest.0 {
                nearest = (distance, index, s);
            }
        }
        let (distance, index, s) = nearest;
        // Which side: across the curve at an inner foot; by the chords at a corner
        // (where no one normal says) and far from the curve (where they cannot err).
        let at_end = s <= 0.0 || s >= 1.0;
        let end_joint = if s <= 0.0 { index } else { index + 1 };
        let inside = if distance > 8.0 * self.sag || (at_end && self.joints[end_joint % self.joints.len()].corner) {
            self.inside_chords(q)
        } else {
            let (point, tangent) = self.point(index, s);
            dot2(sub2(q, point), self.outward(tangent)) < 0.0
        };
        if inside { -distance } else { distance }
    }

    /// Negative inside the bar; outside, no more than the distance to it.
    pub fn signed_distance(&self, x: Vec3) -> f64 {
        let d = x.sub(self.origin);
        let (q, h) = ([d.dot(self.xa), d.dot(self.ya)], d.dot(self.n));
        let ends = (self.from - h).max(h - self.to);
        // Far from the bar's box its distance to the box is enough.
        let dx = (self.low[0] - q[0]).max(q[0] - self.high[0]).max(0.0);
        let dy = (self.low[1] - q[1]).max(q[1] - self.high[1]).max(0.0);
        let far = dx.hypot(dy).hypot(ends.max(0.0));
        if far > 0.25 * length2(sub2(self.high, self.low)) {
            return far;
        }
        let side = (self.section_distance(q) + self.taper * h) * self.cosine;
        side.max(ends)
    }

    /// Does an end face fold under the screw (turn 1, advance `c` per radian
    /// about the axis through `axis_origin` along `ez`) at a level in
    /// `levels`? An end square to the axis never does; one along the axis
    /// folds on a line of one level, which its rim already carries. A leaning
    /// end folds along a line that crosses levels, a wall of its own.
    pub fn end_folds(&self, axis_origin: Vec3, ez: Vec3, c: f64, levels: [f64; 2]) -> bool {
        let lean = self.n.dot(ez).abs();
        if self.n.cross(ez).length() <= 1e-12 || lean <= 1e-12 {
            return false;
        }
        let graze = |x: Vec3| ez.cross(x.sub(axis_origin)).add(ez.scale(c)).dot(self.n);
        // from the axis, across it
        let radial = |x: Vec3| {
            let r = x.sub(axis_origin);
            r.sub(ez.scale(r.dot(ez)))
        };
        for h in [self.from, self.to] {
            let mut rim: Vec<Vec3> = Vec::new();
            for (index, piece) in self.pieces.iter().enumerate() {
                for place in &piece.places[..piece.places.len() - 1] {
                    rim.push(self.side(index, *place, h));
                }
            }
            // where the grazing line crosses the rim
            let mut crossings: Vec<Vec3> = Vec::new();
            for k in 0..rim.len() {
                let (a, b) = (rim[k], rim[(k + 1) % rim.len()]);
                let (ga, gb) = (graze(a), graze(b));
                if (ga < 0.0) != (gb < 0.0) {
                    crossings.push(a.add(b.sub(a).scale(ga / (ga - gb))));
                }
            }
            // in order along the line, so each pair spans one stretch inside the end
            if let [first, second, ..] = crossings.as_slice() {
                let along = second.sub(*first);
                let base = *first;
                crossings.sort_by(|a, b| a.sub(base).dot(along).total_cmp(&b.sub(base).dot(along)));
            }
            for pair in crossings.chunks(2) {
                if let [a, b] = pair {
                    // the levels the stretch runs through: from its nearest point to the
                    // axis to its farther end
                    let (ra, rb) = (radial(*a), radial(*b));
                    let run = rb.sub(ra);
                    let w = if run.dot(run) > 0.0 { (-ra.dot(run) / run.dot(run)).clamp(0.0, 1.0) } else { 0.0 };
                    let (least, most) = (ra.add(run.scale(w)).length(), ra.length().max(rb.length()));
                    if least <= levels[1] && most >= levels[0] {
                        return true;
                    }
                }
            }
        }
        false
    }
}
