//! The plan of a ball-end mill's lower envelope over a box: which surface is
//! lowest where, as a graph of exact boundary curves (the second piece of the
//! ball-end raster construction; the faces are built on it next).
//!
//! The envelope ([`BallEnvelope`]) over each point of the box is the lowest
//! of the capsules' parts, read by SURFACE: one cylinder for every run of
//! moves along one line ([`Part::Tube`], the run's moves joined end to end),
//! one sphere for every distinct ball centre ([`Part::Ball`], the ball two
//! moves share at their joint once), and the stock's top ([`Part::Top`])
//! wherever nothing reaches below it. A ball is never lower than a tube of its
//! own (a move ending at its centre) where that tube's foot lies on the move,
//! so there the tube is read: the boundary between a tube and its own ball is
//! the plane square to the move through the ball's centre, found exactly, and
//! no rounding tie at the joint makes a ball region of no width.
//!
//! The plan is read on a grid of the given cell: each node labelled by its
//! part, each grid edge whose ends differ crossed where its two parts trade
//! places (bisected to rounding on the pair's own comparison), and the
//! crossings linked through the cells they bound. A cell of two parts and two
//! crossings links them; a cell of two parts crossed four times is read by
//! its centre. A cell where three or more parts meet holds a VERTEX: it is
//! solved by Newton on the pairs' equations (the difference of two heights;
//! for a tube and its own ball the plane between them), or the cell is
//! quartered until it is (down to 1e-11 of the cell, where its centre is
//! taken). A part met inside a grid edge whose ends agree (a region narrower
//! than the cell where the grid crosses it) is refused, never guessed: the
//! caller reads again at a smaller cell. A region the grid misses whole (no
//! node in it, no side through it) is caught where it holds a witness: each
//! move's lowest line (below its centres, at half the cell's spacing) and
//! each ball's lowest point; a part lowest at one that the plan does not
//! name is refused the same way. The cell is the caller's: a region that
//! holds no witness and is narrower than the cell everywhere is not seen.
//!
//! The result is checked before it is returned: at every crossing the two
//! parts' heights agree to 1e-9 of the radius and nothing else is lower
//! there; at every vertex its parts' heights agree and nothing else is lower.

use crate::mill_envelope::{ball_underside, BallEnvelope};
use crate::Vec3;
use std::collections::HashMap;

/// A surface the envelope lies on over part of the plan.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Part {
    /// The cylinder about a run of moves along one line (the run's index).
    Tube(usize),
    /// The sphere about one ball centre (the centre's index).
    Ball(usize),
    /// The stock's top, where nothing cuts below it.
    Top,
    /// Beyond the box (the right side of a wall edge).
    Outside,
}

/// A point where three or more parts meet, or where a boundary meets the
/// box's side, or a corner of the box (lifted to the envelope there).
#[derive(Clone, Debug)]
pub struct PlanVertex {
    pub point: Vec3,
}

/// A boundary curve of the plan from one vertex to another: `left` lies to
/// its left in plan (looking down), `right` to its right (`Outside` along
/// the box's sides, which run counter-clockwise). `points` are on the curve,
/// lifted to the envelope, the vertices' own points first and last.
#[derive(Clone, Debug)]
pub struct PlanEdge {
    pub from: usize,
    pub to: usize,
    pub left: Part,
    pub right: Part,
    pub points: Vec<Vec3>,
}

/// The plan's graph, and the surfaces its parts name.
#[derive(Clone, Debug)]
pub struct Plan {
    pub vertices: Vec<PlanVertex>,
    pub edges: Vec<PlanEdge>,
    /// Each tube's run: the moves (indices into the envelope's segments) along
    /// its line, in order.
    pub runs: Vec<Vec<usize>>,
    /// Each ball's centre.
    pub centres: Vec<Vec3>,
    /// The box [x0, x1] × [y0, y1] and the stock's top.
    pub bounds: [f64; 4],
    pub top: f64,
}

/// What a plan reads: over each point (x, y) of its chart, the part lowest
/// there and its height (the stock's top where nothing cuts below it), and
/// each part's own height and surface (continued past where it is read).
/// A mill's envelope over level ground reads in plan (x, y) and height z; a
/// dome's reads directions in a chart of the sphere, and the distance along
/// each ray.
pub trait Field {
    /// The length the tolerances are shares of (the ball's radius).
    fn scale(&self) -> f64;
    /// The length one unit of the chart spans, about (1 in plan).
    fn length(&self) -> f64;
    /// The stock's top: the height read where nothing cuts below it.
    fn top(&self) -> f64;
    /// The lowest part over (x, y) and its height; ties to the lesser part.
    fn lowest(&self, x: f64, y: f64) -> (f64, Part);
    /// The part's height over (x, y), +∞ where it is not read.
    fn height(&self, part: Part, x: f64, y: f64) -> f64;
    /// The part's surface continued past where it is read (NaN past its
    /// reach), for the boundary equations.
    fn continued(&self, part: Part, x: f64, y: f64) -> f64;
    /// For two parts that meet where one of them stops being read (a tube
    /// and its own ball, two stretches of one surface), the signed distance
    /// of the point from the surface they meet on; None for parts that meet
    /// where their heights agree.
    fn plane(&self, p: Part, q: Part, x: f64, y: f64) -> Option<f64>;
    /// Points (in the chart) each part that is lowest anywhere is lowest at
    /// or near, read at a grid of `cell`: a part lowest at one that the plan
    /// does not name is narrower than the cell.
    fn witnesses(&self, cell: f64) -> Vec<[f64; 2]>;
    /// Points (in the chart) where a boundary turns a corner that no third
    /// part marks (two parts meeting at a point of both their centre
    /// curves): each is made a vertex of the plan, its cell read down to it.
    fn seeds(&self) -> Vec<[f64; 2]> {
        Vec::new()
    }

    /// Whether `p` is read below `q` at (x, y) (the pair's own comparison,
    /// ties to the lesser part).
    fn below(&self, p: Part, q: Part, x: f64, y: f64) -> bool {
        let (hp, hq) = (self.height(p, x, y), self.height(q, x, y));
        hp < hq || (hp == hq && p < q)
    }

    /// The equation of the boundary between `p` and `q` at (x, y): zero on it.
    fn equation(&self, p: Part, q: Part, x: f64, y: f64) -> f64 {
        self.plane(p, q, x, y).unwrap_or_else(|| self.continued(p, x, y) - self.continued(q, x, y))
    }

    /// Whether (x, y) lies on the curve where `p` and `q` meet, to
    /// `tolerance`: their surfaces at one height there, and on the surface
    /// they meet on where they have one.
    fn meets(&self, p: Part, q: Part, x: f64, y: f64, tolerance: f64) -> bool {
        self.plane(p, q, x, y).is_none_or(|d| d.abs() <= tolerance) && (self.continued(p, x, y) - self.continued(q, x, y)).abs() <= tolerance
    }
}

/// The mill's envelope over level ground as a field over its plan, cut at
/// `top`.
pub fn ball_field(envelope: &BallEnvelope, top: f64) -> impl Field + '_ {
    Parts::new(envelope, top)
}

/// The parts of an envelope, by surface.
struct Parts<'a> {
    envelope: &'a BallEnvelope,
    run_of: Vec<usize>,
    runs: Vec<Vec<usize>>,
    /// Each move's start and end centre's index.
    centre_of: Vec<[usize; 2]>,
    centres: Vec<Vec3>,
    top: f64,
}

impl<'a> Parts<'a> {
    fn new(envelope: &'a BallEnvelope, top: f64) -> Self {
        let segments = envelope.segments();
        // Centres equal to the bit are one ball.
        let mut index: HashMap<[u64; 3], usize> = HashMap::new();
        let mut centres = Vec::new();
        let mut centre = |p: Vec3| {
            *index.entry([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).or_insert_with(|| {
                centres.push(p);
                centres.len() - 1
            })
        };
        let centre_of: Vec<[usize; 2]> = segments.iter().map(|&(a, b)| [centre(a), centre(b)]).collect();
        // A move joins the run before it where it starts at that run's end and
        // runs along its line the same way (directions within 1e-12).
        let mut runs: Vec<Vec<usize>> = Vec::new();
        let mut run_of = Vec::with_capacity(segments.len());
        for (k, &(a, b)) in segments.iter().enumerate() {
            let joins = k > 0 && {
                let (pa, pb) = segments[k - 1];
                let (d0, d1) = (pb.sub(pa), b.sub(a));
                let (l0, l1) = (d0.length(), d1.length());
                l0 > 0.0 && l1 > 0.0 && centre_of[k - 1][1] == centre_of[k][0] && {
                    let (u0, u1) = (d0.scale(1.0 / l0), d1.scale(1.0 / l1));
                    u0.dot(u1) > 0.0 && u0.cross(u1).length() <= 1e-12
                }
            };
            if !joins {
                runs.push(Vec::new());
            }
            runs.last_mut().unwrap().push(k);
            run_of.push(runs.len() - 1);
        }
        Parts { envelope, run_of, runs, centre_of, centres, top }
    }

    fn radius(&self) -> f64 {
        self.envelope.radius()
    }

    /// Whether `centre` is an end of the run `run` (its ball is the run's own).
    fn own(&self, run: usize, centre: usize) -> bool {
        let moves = &self.runs[run];
        self.centre_of[moves[0]][0] == centre || self.centre_of[*moves.last().unwrap()][1] == centre
    }

    /// The lowest part over (x, y) and its height (`Top` at the top where
    /// nothing reaches below it). A ball is passed over where a tube of its
    /// own reaches; ties go to the lesser part (a tube before a ball).
    fn lowest(&self, x: f64, y: f64) -> (f64, Part) {
        let mut best = (self.top, Part::Top);
        let mut take = |z: f64, part: Part| {
            if z < best.0 || (z == best.0 && part < best.1) {
                best = (z, part);
            }
        };
        let candidates = self.envelope.candidates(x, y);
        let read: Vec<[Option<f64>; 3]> = candidates.iter().map(|&k| self.envelope.capsule_parts(k, x, y)).collect();
        // the centres a tube of their own covers here
        let covered: Vec<usize> = candidates.iter().zip(&read).filter(|(_, r)| r[2].is_some()).flat_map(|(&k, _)| self.centre_of[k]).collect();
        for (&k, [start, end, tube]) in candidates.iter().zip(&read) {
            if let Some(z) = tube {
                take(*z, Part::Tube(self.run_of[k]));
            }
            for (z, c) in [(start, self.centre_of[k][0]), (end, self.centre_of[k][1])] {
                if let Some(z) = z {
                    if !covered.contains(&c) {
                        take(*z, Part::Ball(c));
                    }
                }
            }
        }
        best
    }

    /// Whether a tube of the ball `c`'s own (a move ending at it) reaches over
    /// (x, y), where that tube is never above the ball.
    fn covered(&self, c: usize, x: f64, y: f64, candidates: &[usize]) -> bool {
        candidates.iter().any(|&k| self.centre_of[k].contains(&c) && self.envelope.capsule_parts(k, x, y)[2].is_some())
    }

    /// The part's height over (x, y), +∞ where it does not reach (a ball
    /// covered by its own tube counted as not reaching).
    fn height(&self, part: Part, x: f64, y: f64) -> f64 {
        match part {
            Part::Top => self.top,
            Part::Outside => f64::INFINITY,
            Part::Tube(run) => self.runs[run].iter().filter_map(|&k| self.envelope.capsule_parts(k, x, y)[2]).fold(f64::INFINITY, f64::min),
            Part::Ball(c) => {
                let candidates = self.envelope.candidates(x, y);
                if self.covered(c, x, y, candidates) {
                    f64::INFINITY
                } else {
                    ball_underside(self.centres[c], self.radius(), x, y).unwrap_or(f64::INFINITY)
                }
            }
        }
    }

    /// The part's surface continued past where it is read (a tube's whole
    /// line's cylinder, a ball inside its rim), for the vertex equations.
    fn continued(&self, part: Part, x: f64, y: f64) -> f64 {
        match part {
            Part::Top => self.top,
            Part::Outside => f64::NAN,
            Part::Tube(run) => self.envelope.tube_root(self.runs[run][0], x, y).map_or(f64::NAN, |(z, _)| z),
            Part::Ball(c) => ball_underside(self.centres[c], self.radius(), x, y).unwrap_or(f64::NAN),
        }
    }

    /// A tube and its own ball meet on the plane through the ball's centre
    /// square to the run: the signed distance of the ball's underside point
    /// from it (NaN where the ball does not reach).
    fn own_plane(&self, p: Part, q: Part, x: f64, y: f64) -> Option<f64> {
        let ((Part::Tube(tube), Part::Ball(ball)) | (Part::Ball(ball), Part::Tube(tube))) = (p, q) else {
            return None;
        };
        if !self.own(tube, ball) {
            return None;
        }
        let moves = &self.runs[tube];
        let (a, b) = (self.envelope.segments()[moves[0]].0, self.envelope.segments()[*moves.last().unwrap()].1);
        let Ok(axis) = b.sub(a).normalized() else { return Some(f64::NAN) };
        let c = self.centres[ball];
        Some(ball_underside(c, self.radius(), x, y).map_or(f64::NAN, |z| Vec3::new(x - c.x, y - c.y, z - c.z).dot(axis)))
    }
}

impl<'a> Field for Parts<'a> {
    fn scale(&self) -> f64 {
        self.radius()
    }
    fn length(&self) -> f64 {
        1.0
    }
    fn top(&self) -> f64 {
        self.top
    }
    fn lowest(&self, x: f64, y: f64) -> (f64, Part) {
        Parts::lowest(self, x, y)
    }
    fn height(&self, part: Part, x: f64, y: f64) -> f64 {
        Parts::height(self, part, x, y)
    }
    fn continued(&self, part: Part, x: f64, y: f64) -> f64 {
        Parts::continued(self, part, x, y)
    }
    fn plane(&self, p: Part, q: Part, x: f64, y: f64) -> Option<f64> {
        self.own_plane(p, q, x, y)
    }
    /// Each run's lowest line (below its moves, at half the cell's spacing)
    /// and each ball's lowest point.
    fn witnesses(&self, cell: f64) -> Vec<[f64; 2]> {
        let mut points: Vec<[f64; 2]> = self.centres.iter().map(|c| [c.x, c.y]).collect();
        for &(a, b) in self.envelope.segments() {
            let length = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
            let count = ((2.0 * length / cell).ceil() as usize).min(1 << 20);
            for k in 1..count {
                let t = k as f64 / count as f64;
                points.push([a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t]);
            }
        }
        points
    }
}

/// A point where a grid edge's two parts trade places.
#[derive(Clone, Debug)]
struct Crossing {
    point: Vec3,
    /// The edge's ends and their parts.
    ends: [([f64; 2], Part); 2],
    /// On a side of the box: 0 bottom, 1 right, 2 top, 3 left.
    wall: Option<u8>,
}

/// A side of a cell: its crossings in order along it, and the parts
/// between them (one more than the crossings).
#[derive(Clone, Debug)]
struct Side {
    crossings: Vec<usize>,
    labels: Vec<Part>,
}

fn mid(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]
}

/// A node of the link graph: a crossing, or a solved vertex.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum Node {
    Crossing(usize),
    Vertex(usize),
}

struct Builder<'a> {
    parts: &'a dyn Field,
    crossings: Vec<Crossing>,
    /// Solved vertices: the point and its parts.
    vertices: Vec<(Vec3, Vec<Part>)>,
    links: HashMap<Node, Vec<Node>>,
    tolerance: f64,
    cell: f64,
    /// A length in the chart small enough to be one point (1e-12 of the scale).
    near: f64,
    seeds: Vec<[f64; 2]>,
}

/// The finest cell a vertex is sought in, as a share of the grid's cell.
const FINEST: f64 = 1e-11;

impl<'a> Builder<'a> {
    fn link(&mut self, a: Node, b: Node) {
        self.links.entry(a).or_default().push(b);
        self.links.entry(b).or_default().push(a);
    }

    /// The crossing between `a` (labelled `p`) and `b` (labelled `q`), and
    /// whether nothing else is lower there.
    fn crossing(&mut self, a: [f64; 2], p: Part, b: [f64; 2], q: Part, wall: Option<u8>) -> (Crossing, bool) {
        let at = |t: f64| [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        let (mut lo, mut hi) = (0.0f64, 1.0f64);
        for _ in 0..64 {
            let m = 0.5 * (lo + hi);
            if m <= lo || m >= hi {
                break;
            }
            let [x, y] = at(m);
            if self.parts.below(p, q, x, y) { lo = m } else { hi = m }
        }
        let [x, y] = at(0.5 * (lo + hi));
        let z = self.parts.height(p, x, y).min(self.parts.height(q, x, y));
        // the point is on the curve where the two parts meet (not where one
        // of them merely stops being read), and nothing else is lower
        let clear = self.parts.meets(p, q, x, y, self.tolerance) && self.parts.lowest(x, y).0 >= z - self.tolerance;
        (Crossing { point: Vec3::new(x, y, z), ends: [(a, p), (b, q)], wall }, clear)
    }

    /// The side from `a` (labelled `p`) to `b` (labelled `q`): one crossing
    /// where its ends differ and nothing else is lower there; else halved at
    /// its midpoint (the point a quartered cell reads too) until each piece's
    /// crossing is clear (down to `finest`). Ends that agree give no crossing:
    /// a part met only inside such a side is found, if at all, when a cell
    /// reads the side's midpoint (and is then refused).
    fn side(&mut self, a: [f64; 2], p: Part, b: [f64; 2], q: Part, wall: Option<u8>, finest: f64) -> Side {
        // near a seed a part may pass inside a side whatever its ends read:
        // the side is read at 63 points more there, and in pieces between
        // the points where it changes
        let length = (b[0] - a[0]).abs().max((b[1] - a[1]).abs());
        if length > 64.0 * finest && self.near_seed(a, b) {
            let mut from = (a, p);
            let mut pieces: Vec<([f64; 2], Part, [f64; 2], Part)> = Vec::new();
            for k in 1..=64 {
                let t = k as f64 / 64.0;
                let m = if k == 64 { b } else { [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t] };
                let lm = if k == 64 { q } else { self.parts.lowest(m[0], m[1]).1 };
                if lm != from.1 || k == 64 {
                    pieces.push((from.0, from.1, m, lm));
                    from = (m, lm);
                }
            }
            if pieces.len() > 1 || pieces[0].0 != a || pieces[0].2 != b {
                let mut side = Side { crossings: Vec::new(), labels: vec![p] };
                for (u, lu, w, lw) in pieces {
                    let piece = self.side_plain(u, lu, w, lw, wall, finest);
                    side.crossings.extend(piece.crossings);
                    side.labels.extend_from_slice(&piece.labels[1..]);
                }
                return side;
            }
        }
        self.side_plain(a, p, b, q, wall, finest)
    }

    /// [`Builder::side`] without the reading near seeds.
    fn side_plain(&mut self, a: [f64; 2], p: Part, b: [f64; 2], q: Part, wall: Option<u8>, finest: f64) -> Side {
        if p == q {
            return Side { crossings: Vec::new(), labels: vec![p] };
        }
        let (crossing, clear) = self.crossing(a, p, b, q, wall);
        let length = (b[0] - a[0]).abs().max((b[1] - a[1]).abs());
        let m = mid(a, b);
        // at the finest, or where rounding no longer moves the midpoint, the
        // crossing is kept as it is (and the check refuses it if it is wrong)
        if clear || length <= finest || m == a || m == b {
            self.crossings.push(crossing);
            return Side { crossings: vec![self.crossings.len() - 1], labels: vec![p, q] };
        }
        let lm = self.parts.lowest(m[0], m[1]).1;
        let mut first = self.side_plain(a, p, m, lm, wall, finest);
        let second = self.side_plain(m, lm, b, q, wall, finest);
        first.crossings.extend(second.crossings);
        first.labels.extend_from_slice(&second.labels[1..]);
        first
    }

    /// Whether a seed lies within eight times the side's length of it (or
    /// two cells).
    fn near_seed(&self, a: [f64; 2], b: [f64; 2]) -> bool {
        let length = (4.0 * (b[0] - a[0]).abs().max((b[1] - a[1]).abs())).max(self.cell);
        self.seeds.iter().any(|s| {
            let (lo, hi) = ([a[0].min(b[0]) - 2.0 * length, a[1].min(b[1]) - 2.0 * length], [a[0].max(b[0]) + 2.0 * length, a[1].max(b[1]) + 2.0 * length]);
            s[0] >= lo[0] && s[0] <= hi[0] && s[1] >= lo[1] && s[1] <= hi[1]
        })
    }

    /// The halves of the side `side` from `from` to `to` at its midpoint
    /// `middle` (labelled `lm`, which must be the label of the stretch it lies
    /// in), along the axis `axis`.
    fn split(&self, side: &Side, from: [f64; 2], to: [f64; 2], middle: [f64; 2], lm: Part, axis: usize) -> Result<[Side; 2], String> {
        let sense = (to[axis] - from[axis]).signum();
        let along = |p: f64| (p - from[axis]) * sense;
        let place = |k: usize| along(if axis == 0 { self.crossings[k].point.x } else { self.crossings[k].point.y });
        let mut before = side.crossings.iter().take_while(|&&k| place(k) <= along(middle[axis])).count();
        // a crossing at the midpoint itself (a boundary through the grid's
        // point, read either side by rounding) goes to the half its label says
        let near = |k: usize| (place(k) - along(middle[axis])).abs() <= 1e-12 * (to[axis] - from[axis]).abs();
        if side.labels[before] != lm {
            if before > 0 && near(side.crossings[before - 1]) && side.labels[before - 1] == lm {
                before -= 1;
            } else if before < side.crossings.len() && near(side.crossings[before]) && side.labels[before + 1] == lm {
                before += 1;
            }
        }
        if side.labels[before] != lm {
            return Err(format!(
                "a part ({lm:?}) inside a cell side read as {:?} near ({:.6}, {:.6}): read at a smaller cell [{from:?} {to:?} crossings {:?}]",
                side.labels, middle[0], middle[1], side.crossings.iter().map(|&k| (self.crossings[k].point.x, self.crossings[k].point.y)).collect::<Vec<_>>()
            ));
        }
        Ok([
            Side { crossings: side.crossings[..before].to_vec(), labels: side.labels[..=before].to_vec() },
            Side { crossings: side.crossings[before..].to_vec(), labels: side.labels[before..].to_vec() },
        ])
    }

    /// Reads the cell with corners `c` (counter-clockwise from its low
    /// corner) labelled `l`, and its sides `s` (bottom and top running along
    /// +x, right and left along +y).
    fn cell(&mut self, c: [[f64; 2]; 4], l: [Part; 4], s: [Side; 4], size: f64, finest: f64) -> Result<(), String> {
        let mut distinct: Vec<Part> = Vec::new();
        for part in l.iter().chain(s.iter().flat_map(|side| side.labels.iter())) {
            if !distinct.contains(part) {
                distinct.push(*part);
            }
        }
        let crossed: Vec<usize> = s.iter().flat_map(|side| side.crossings.iter().copied()).collect();
        let seed = self.seeds.iter().copied().find(|p| p[0] >= c[0][0] && p[0] <= c[2][0] && p[1] >= c[0][1] && p[1] <= c[2][1]);
        if let Some(seed) = seed {
            if size <= 1e-6 * self.cell {
                // the seed itself is the cell's vertex
                let mut parts = distinct;
                let (z, at) = self.parts.lowest(seed[0], seed[1]);
                if !parts.contains(&at) {
                    parts.push(at);
                }
                return self.place(Vec3::new(seed[0], seed[1], z), parts, &crossed);
            }
        }
        if seed.is_none() && distinct.len() == 1 {
            return Ok(());
        }
        if seed.is_none() && distinct.len() == 2 && crossed.len() == 2 {
            self.link(Node::Crossing(crossed[0]), Node::Crossing(crossed[1]));
            return Ok(());
        }
        let centre = mid(c[0], c[2]);
        if seed.is_none() && distinct.len() == 2 && crossed.len() == 4 && s.iter().all(|side| side.crossings.len() == 1) {
            let (_, middle) = self.parts.lowest(centre[0], centre[1]);
            let [b, r, t, lft] = [s[0].crossings[0], s[1].crossings[0], s[2].crossings[0], s[3].crossings[0]];
            if middle == l[0] {
                // corners 1 and 3 cut off
                self.link(Node::Crossing(b), Node::Crossing(r));
                self.link(Node::Crossing(t), Node::Crossing(lft));
                return Ok(());
            }
            if middle == l[1] {
                self.link(Node::Crossing(b), Node::Crossing(lft));
                self.link(Node::Crossing(r), Node::Crossing(t));
                return Ok(());
            }
        }
        // Three or more parts: a vertex, solved here or in a quarter.
        if seed.is_none() && distinct.len() == 3 && crossed.len() == 3 {
            if let Some(point) = self.solve(&distinct, c, centre) {
                return self.place(point, distinct, &crossed);
            }
        }
        if seed.is_none() && size <= finest {
            let (z, _) = self.parts.lowest(centre[0], centre[1]);
            return self.place(Vec3::new(centre[0], centre[1], z), distinct, &crossed);
        }
        // Quarter the cell: the sides' midpoints and its centre.
        let (m0, m1, m2, m3) = (mid(c[0], c[1]), mid(c[1], c[2]), mid(c[3], c[2]), mid(c[0], c[3]));
        let label = |p: [f64; 2]| self.parts.lowest(p[0], p[1]).1;
        let (k0, k1, k2, k3, kc) = (label(m0), label(m1), label(m2), label(m3), label(centre));
        let halves = (|| -> Result<_, String> {
            Ok((
                self.split(&s[0], c[0], c[1], m0, k0, 0)?,
                self.split(&s[1], c[1], c[2], m1, k1, 1)?,
                self.split(&s[2], c[3], c[2], m2, k2, 0)?,
                self.split(&s[3], c[0], c[3], m3, k3, 1)?,
            ))
        })();
        let ([b0, b1], [r0, r1], [t0, t1], [l0, l1]) = match halves {
            Ok(halves) => halves,
            // a part met inside a side this small is rounding at a vertex:
            // the cell's centre is that vertex
            Err(_) if size <= 1e-5 * self.cell || (size <= 1e-4 * self.cell && self.near_seed(c[0], c[2])) => {
                let mut parts = distinct;
                for k in [k0, k1, k2, k3, kc] {
                    if !parts.contains(&k) {
                        parts.push(k);
                    }
                }
                // solved on its parts' equations near the cell where it can be
                let grown = [[c[0][0] - size, c[0][1] - size], c[1], [c[2][0] + size, c[2][1] + size], c[3]];
                let point = if parts.len() >= 3 { self.solve(&parts, grown, centre) } else { None };
                let point = point.unwrap_or_else(|| Vec3::new(centre[0], centre[1], self.parts.lowest(centre[0], centre[1]).0));
                return self.place(point, parts, &crossed);
            }
            Err(error) => return Err(error),
        };
        // inner sides: m0–centre, centre–m2 (along +y); m3–centre, centre–m1 (along +x)
        let down = self.side(m0, k0, centre, kc, None, finest);
        let up = self.side(centre, kc, m2, k2, None, finest);
        let west = self.side(m3, k3, centre, kc, None, finest);
        let east = self.side(centre, kc, m1, k1, None, finest);
        let half = 0.5 * size;
        self.cell([c[0], m0, centre, m3], [l[0], k0, kc, k3], [b0, down.clone(), west.clone(), l0], half, finest)?;
        self.cell([m0, c[1], m1, centre], [k0, l[1], k1, kc], [b1, r0, east.clone(), down], half, finest)?;
        self.cell([centre, m1, c[2], m2], [kc, k1, l[2], k2], [east, r1, t1, up.clone()], half, finest)?;
        self.cell([m3, centre, m2, c[3]], [k3, kc, k2, l[3]], [west, up, t0, l1], half, finest)
    }

    /// Newton on the three parts' two boundary equations from the cell's
    /// centre; the point if it converges inside the cell.
    fn solve(&self, parts: &[Part], c: [[f64; 2]; 4], start: [f64; 2]) -> Option<Vec3> {
        // Of the three pairs' equations, the two most nearly square to each
        // other at the start (a tube and its own ball meet tangent, as do two
        // tubes at a joint's lowest point, so some pairs run tangent at the
        // vertex; the plane between a tube and its own ball never does).
        let mut pairs = Vec::new();
        for (i, &a) in parts.iter().enumerate() {
            for &b in &parts[i + 1..] {
                pairs.push((a, b));
            }
        }
        let gradient = |(a, b): (Part, Part)| {
            let h = 1e-6 * self.parts.scale() / self.parts.length();
            let f = self.parts.equation(a, b, start[0], start[1]);
            [(self.parts.equation(a, b, start[0] + h, start[1]) - f) / h, (self.parts.equation(a, b, start[0], start[1] + h) - f) / h]
        };
        let grads: Vec<[f64; 2]> = pairs.iter().map(|&pair| gradient(pair)).collect();
        let mut choice = (0usize, 1usize);
        let mut best = -1.0f64;
        for (i, j) in (0..pairs.len()).flat_map(|i| (i + 1..pairs.len()).map(move |j| (i, j))) {
            let (u, w) = (grads[i], grads[j]);
            let sine = (u[0] * w[1] - u[1] * w[0]).abs() / ((u[0] * u[0] + u[1] * u[1]).sqrt() * (w[0] * w[0] + w[1] * w[1]).sqrt());
            if sine.is_finite() && sine > best {
                best = sine;
                choice = (i, j);
            }
        }
        let (first, second) = (pairs[choice.0], pairs[choice.1]);
        let g = |x: f64, y: f64| [self.parts.equation(first.0, first.1, x, y), self.parts.equation(second.0, second.1, x, y)];
        let scale = self.parts.scale();
        let step = 1e-7 * scale / self.parts.length();
        let [mut x, mut y] = start;
        for _ in 0..40 {
            let [f0, f1] = g(x, y);
            if !(f0.is_finite() && f1.is_finite()) {
                return None;
            }
            if f0.abs().max(f1.abs()) <= 1e-13 * scale {
                let inside = x >= c[0][0] && x <= c[2][0] && y >= c[0][1] && y <= c[2][1];
                if !inside {
                    return None;
                }
                let (z, _) = self.parts.lowest(x, y);
                return Some(Vec3::new(x, y, z));
            }
            let [ax, ay] = g(x + step, y);
            let [bx, by] = g(x, y + step);
            let (j00, j10, j01, j11) = ((ax - f0) / step, (ay - f1) / step, (bx - f0) / step, (by - f1) / step);
            let det = j00 * j11 - j01 * j10;
            if !(det.is_finite() && det != 0.0) {
                return None;
            }
            x -= (f0 * j11 - f1 * j01) / det;
            y -= (j00 * f1 - j10 * f0) / det;
        }
        None
    }

    fn place(&mut self, point: Vec3, parts: Vec<Part>, crossed: &[usize]) -> Result<(), String> {
        // a vertex on a side two cells share is solved in both: one vertex
        let near = 100.0 * self.near;
        // a vertex solved this close to a seed is the seed's
        let point = match self.seeds.iter().find(|s| ((s[0] - point.x).powi(2) + (s[1] - point.y).powi(2)).sqrt() <= 2e-4 * self.cell) {
            Some(s) => Vec3::new(s[0], s[1], self.parts.lowest(s[0], s[1]).0),
            None => point,
        };
        let found = self.vertices.iter().position(|(p, _)| p.sub(point).length() <= near);
        let vertex = match found {
            Some(k) => {
                for part in parts {
                    if !self.vertices[k].1.contains(&part) {
                        self.vertices[k].1.push(part);
                    }
                }
                Node::Vertex(k)
            }
            None => {
                self.vertices.push((point, parts));
                Node::Vertex(self.vertices.len() - 1)
            }
        };
        for &k in crossed {
            self.link(vertex, Node::Crossing(k));
        }
        Ok(())
    }
}

impl BallEnvelope {
    /// The plan of the envelope over the box [x0, x1] × [y0, y1] cut below
    /// `top`, read on a grid of about `cell` (see the module's notes). Refuses
    /// (saying where) a part the grid passes over between two nodes of one
    /// part, a box or cell outside the envelope's reach (every bound within 1e6
    /// radii of the origin, the cell at least 1e-6 radii, at most 1e8 nodes),
    /// and a result its own check rejects.
    pub fn plan(&self, x0: f64, x1: f64, y0: f64, y1: f64, top: f64, cell: f64) -> Result<Plan, String> {
        let r = self.radius();
        let bound = 1e6 * r;
        if !(cell.is_finite() && cell >= 1e-6 * r) || ![x0, x1, y0, y1, top].iter().all(|c| c.is_finite() && c.abs() <= bound) || !(x1 > x0 && y1 > y0) {
            return Err("the box or the cell lies outside the envelope's reach".into());
        }
        let parts = Parts::new(self, top);
        let mut plan = plan_field(&parts, [x0, x1, y0, y1], cell)?;
        plan.runs = parts.runs.clone();
        plan.centres = parts.centres.clone();
        Ok(plan)
    }
}

/// The plan of `field` over the box `bounds` = [x0, x1] × [y0, y1] of its
/// chart, read on a grid of about `cell` (see the module's notes).
pub fn plan_field(field: &dyn Field, bounds: [f64; 4], cell: f64) -> Result<Plan, String> {
    {
        let [x0, x1, y0, y1] = bounds;
        let top = field.top();
        let r = field.scale();
        if !(cell.is_finite() && cell > 0.0 && x1 > x0 && y1 > y0) {
            return Err("the box or the cell is empty".into());
        }
        let (fx, fy) = (((x1 - x0) / cell).ceil().max(1.0), ((y1 - y0) / cell).ceil().max(1.0));
        if !((fx + 1.0) * (fy + 1.0) <= 1e8) {
            return Err("more than 1e8 grid nodes".into());
        }
        let (nx, ny) = (fx as usize, fy as usize);
        let node = |i: usize, j: usize| [if i == nx { x1 } else { x0 + (x1 - x0) * i as f64 / nx as f64 }, if j == ny { y1 } else { y0 + (y1 - y0) * j as f64 / ny as f64 }];
        let parts = field;
        let labels: Vec<Part> = (0..=nx).flat_map(|i| (0..=ny).map(move |j| (i, j))).map(|(i, j)| { let p = node(i, j); parts.lowest(p[0], p[1]).1 }).collect();
        let label = |i: usize, j: usize| labels[i * (ny + 1) + j];
        let near = 1e-12 * r / field.length();
        let seeds = field.seeds();
        let mut builder = Builder { parts, crossings: Vec::new(), vertices: Vec::new(), links: HashMap::new(), tolerance: 1e-9 * r, cell, near, seeds };
        // The grid's edges' crossings: horizontal (i, j)–(i + 1, j), vertical (i, j)–(i, j + 1).
        let finest = FINEST * cell;
        let empty = Side { crossings: Vec::new(), labels: Vec::new() };
        let mut horizontal = vec![empty.clone(); nx * (ny + 1)];
        let mut vertical = vec![empty; (nx + 1) * ny];
        for i in 0..=nx {
            for j in 0..=ny {
                if i < nx {
                    let wall = if j == 0 { Some(0) } else if j == ny { Some(2) } else { None };
                    horizontal[i * (ny + 1) + j] = builder.side(node(i, j), label(i, j), node(i + 1, j), label(i + 1, j), wall, finest);
                }
                if j < ny {
                    let wall = if i == 0 { Some(3) } else if i == nx { Some(1) } else { None };
                    vertical[i * ny + j] = builder.side(node(i, j), label(i, j), node(i, j + 1), label(i, j + 1), wall, finest);
                }
            }
        }
        for i in 0..nx {
            for j in 0..ny {
                let corners = [node(i, j), node(i + 1, j), node(i + 1, j + 1), node(i, j + 1)];
                let l = [label(i, j), label(i + 1, j), label(i + 1, j + 1), label(i, j + 1)];
                let s = [horizontal[i * (ny + 1) + j].clone(), vertical[(i + 1) * ny + j].clone(), horizontal[i * (ny + 1) + j + 1].clone(), vertical[i * ny + j].clone()];
                let size = (corners[1][0] - corners[0][0]).max(corners[3][1] - corners[0][1]);
                builder.cell(corners, l, s, size, finest)?;
            }
        }
        // The plan's vertices: solved vertices, crossings on the box's sides,
        // the box's corners, and crossings not linked twice.
        let mut vertices: Vec<PlanVertex> = Vec::new();
        let mut vertex_of: HashMap<Node, usize> = HashMap::new();
        for (k, (point, _)) in builder.vertices.iter().enumerate() {
            vertex_of.insert(Node::Vertex(k), vertices.len());
            vertices.push(PlanVertex { point: *point });
        }
        for (k, crossing) in builder.crossings.iter().enumerate() {
            let degree = builder.links.get(&Node::Crossing(k)).map_or(0, Vec::len);
            if crossing.wall.is_some() || degree != 2 {
                if crossing.wall.is_none() {
                    return Err(format!("a boundary ends at ({:.6}, {:.6}) inside the box (linked {degree} times)", crossing.point.x, crossing.point.y));
                }
                if degree != 1 {
                    return Err(format!("a boundary meeting the box's side at ({:.6}, {:.6}) is linked {degree} times", crossing.point.x, crossing.point.y));
                }
                vertex_of.insert(Node::Crossing(k), vertices.len());
                vertices.push(PlanVertex { point: crossing.point });
            }
        }
        // Chains between vertices; then loops of crossings with none.
        let mut edges: Vec<PlanEdge> = Vec::new();
        let mut walked: std::collections::HashSet<(Node, Node)> = std::collections::HashSet::new();
        let mut starts: Vec<Node> = vertex_of.keys().copied().collect();
        starts.sort_by_key(|n| vertex_of[n]);
        let point_of = |b: &Builder, n: Node| match n {
            Node::Crossing(k) => b.crossings[k].point,
            Node::Vertex(k) => b.vertices[k].0,
        };
        let mut chains: Vec<Vec<Node>> = Vec::new();
        for &start in &starts {
            for &first in builder.links.get(&start).map_or(&[][..], |v| &v[..]) {
                if walked.contains(&(start, first)) {
                    continue;
                }
                let mut chain = vec![start, first];
                walked.insert((start, first));
                while !vertex_of.contains_key(chain.last().unwrap()) {
                    let (here, back) = (chain[chain.len() - 1], chain[chain.len() - 2]);
                    let links = &builder.links[&here];
                    let next = if links[0] == back { links[1] } else { links[0] };
                    walked.insert((here, next));
                    chain.push(next);
                }
                let (end, before) = (chain[chain.len() - 1], chain[chain.len() - 2]);
                walked.insert((end, before));
                chains.push(chain);
            }
        }
        let mut on_chain: std::collections::HashSet<Node> = chains.iter().flatten().copied().collect();
        for k in 0..builder.crossings.len() {
            let start = Node::Crossing(k);
            if vertex_of.contains_key(&start) || on_chain.contains(&start) || !builder.links.contains_key(&start) {
                continue;
            }
            // a closed boundary: its first crossing is its vertex
            vertex_of.insert(start, vertices.len());
            vertices.push(PlanVertex { point: builder.crossings[k].point });
            let mut chain = vec![start, builder.links[&start][0]];
            walked.insert((start, chain[1]));
            while chain.last() != Some(&start) {
                let (here, back) = (chain[chain.len() - 1], chain[chain.len() - 2]);
                let links = &builder.links[&here];
                let next = if links[0] == back { links[1] } else { links[0] };
                walked.insert((here, next));
                chain.push(next);
            }
            walked.insert((start, chain[chain.len() - 2]));
            on_chain.extend(chain.iter().copied());
            chains.push(chain);
        }
        for chain in chains {
            // the sides from a crossing on the chain: its edge's ends against the chain's direction there
            // The sides from the chain's crossing most square to its grid
            // edge: the edge's start lies left of the chain's direction there
            // when (direction × edge) < 0, read on the edge's own direction so
            // a crossing at a grid node reads as well as any.
            let mut best: Option<(f64, f64, Part, Part)> = None;
            for i in 0..chain.len() {
                let Node::Crossing(k) = chain[i] else { continue };
                // the nearest points either side that stand apart from it
                let here = point_of(&builder, chain[i]);
                let apart = |n: &Node| { let p = point_of(&builder, *n); p.x != here.x || p.y != here.y };
                let before = chain[..i].iter().rev().find(|n| apart(n)).map_or(here, |&n| point_of(&builder, n));
                let after = chain[i + 1..].iter().find(|n| apart(n)).map_or(here, |&n| point_of(&builder, n));
                let (dx, dy) = (after.x - before.x, after.y - before.y);
                let [(a, pa), (b, pb)] = builder.crossings[k].ends;
                let (ex, ey) = (b[0] - a[0], b[1] - a[1]);
                let cross = dx * ey - dy * ex;
                let square = cross.abs() / ((dx * dx + dy * dy).sqrt() * (ex * ex + ey * ey).sqrt());
                if square.is_finite() && best.is_none_or(|(s, _, _, _)| square > s) {
                    best = Some((square, cross, pa, pb));
                }
            }
            // a boundary of no length (from a vertex to itself through a
            // crossing at it) bounds nothing
            let start_point = point_of(&builder, chain[0]);
            let plan_gap = |a: Vec3, b: Vec3| ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt();
            // (or round a loop through one vertex and back narrower than the
            // tolerance: a crossing linked twice to one vertex)
            let small = (1e3 * near).max(4e-4 * cell);
            if chain.iter().all(|&n| plan_gap(point_of(&builder, n), start_point) <= near) || (chain[0] == *chain.last().unwrap() && chain.iter().all(|&n| plan_gap(point_of(&builder, n), start_point) <= small)) {
                continue;
            }
            let Some((square, side, pa, pb)) = best else {
                let at = point_of(&builder, chain[0]);
                let detail: Vec<String> = chain.iter().map(|&n| { let p = point_of(&builder, n); match n { Node::Crossing(k) => format!("C{k}({:e}, {:e}) ends {:?}", p.x, p.y, builder.crossings[k].ends), Node::Vertex(k) => format!("V{k}({:e}, {:e}) {:?}", p.x, p.y, builder.vertices[k].1) } }).collect();
                return Err(format!("a boundary of no length at ({:.9}, {:.9}), {} nodes: {detail:?}", at.x, at.y, chain.len()));
            };
            if !(square > 1e-6) {
                let at = point_of(&builder, chain[0]);
                return Err(format!("a boundary's sides are not read near ({:.6}, {:.6})", at.x, at.y));
            }
            // the start `a` is left when the edge runs from left to right of the direction
            let side = -side;
            let (left, right) = if side > 0.0 { (pa, pb) } else { (pb, pa) };
            let mut points: Vec<Vec3> = Vec::with_capacity(chain.len());
            for &n in &chain {
                let p = point_of(&builder, n);
                if points.last().is_none_or(|q: &Vec3| q.sub(p).length() > near) {
                    points.push(p);
                }
            }
            // the end vertex's own point last, in place of one too near it
            let end = point_of(&builder, *chain.last().unwrap());
            if points.len() > 1 && points.last().unwrap().sub(end).length() <= near {
                *points.last_mut().unwrap() = end;
            } else {
                points.push(end);
            }
            edges.push(PlanEdge { from: vertex_of[&chain[0]], to: vertex_of[chain.last().unwrap()], left, right, points });
        }
        // The box's sides, counter-clockwise: the corners and the sides' crossings in order.
        let corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
        let mut corner_vertex = [0usize; 4];
        for (k, c) in corners.iter().enumerate() {
            let (z, _) = builder.parts.lowest(c[0], c[1]);
            // a boundary meeting the box at its corner: that crossing is the corner
            let at = builder.crossings.iter().enumerate().find(|(_, x)| x.wall.is_some() && ((x.point.x - c[0]).powi(2) + (x.point.y - c[1]).powi(2)).sqrt() <= 1e3 * near).map(|(j, _)| vertex_of[&Node::Crossing(j)]);
            match at {
                Some(v) => {
                    vertices[v].point = Vec3::new(c[0], c[1], z);
                    corner_vertex[k] = v;
                }
                None => {
                    corner_vertex[k] = vertices.len();
                    vertices.push(PlanVertex { point: Vec3::new(c[0], c[1], z) });
                }
            }
        }
        for edge in &mut edges {
            // the ends of the boundaries that met it, at the corner itself
            if let Some(&k) = corner_vertex.iter().find(|&&k| k == edge.from) {
                edge.points[0] = vertices[k].point;
            }
            if let Some(&k) = corner_vertex.iter().find(|&&k| k == edge.to) {
                *edge.points.last_mut().unwrap() = vertices[k].point;
            }
        }
        for side in 0..4u8 {
            let (from, to) = (corners[side as usize], corners[(side as usize + 1) % 4]);
            let along = |p: [f64; 2]| (p[0] - from[0]) * (to[0] - from[0]) + (p[1] - from[1]) * (to[1] - from[1]);
            let mut stops: Vec<(f64, usize)> = builder.crossings.iter().enumerate().filter(|(_, c)| c.wall == Some(side)).map(|(k, c)| (along([c.point.x, c.point.y]), vertex_of[&Node::Crossing(k)])).collect();
            stops.sort_by(|a, b| a.0.total_cmp(&b.0));
            let mut chain = vec![(f64::NEG_INFINITY, corner_vertex[side as usize])];
            chain.extend(stops);
            chain.push((f64::INFINITY, corner_vertex[(side as usize + 1) % 4]));
            for w in chain.windows(2) {
                if w[0].1 == w[1].1 {
                    continue;
                }
                let (a, b) = (vertices[w[0].1].point, vertices[w[1].1].point);
                // the part along this stretch, read at its middle; points at the grid's spacing
                let (mx, my) = (0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
                let (_, part) = builder.parts.lowest(mx, my);
                let length = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
                let count = ((length / cell).ceil() as usize).max(1);
                let mut points = vec![a];
                for k in 1..count {
                    let t = k as f64 / count as f64;
                    let (x, y) = (a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
                    points.push(Vec3::new(x, y, builder.parts.height(part, x, y)));
                }
                points.push(b);
                edges.push(PlanEdge { from: w[0].1, to: w[1].1, left: part, right: Part::Outside, points });
            }
        }
        let plan = Plan { vertices, edges, runs: Vec::new(), centres: Vec::new(), bounds: [x0, x1, y0, y1], top };
        builder.check(&plan)?;
        Ok(plan)
    }
}

impl Plan {
    /// The points of the plan's edge `index` again, about `spacing` apart in
    /// plan and each on the curve where its two parts meet: the edge's
    /// polyline cut evenly by length, each cut point moved square to its
    /// stretch of the polyline onto the curve (by the secant on the pair's
    /// equation; a side of the box keeps its line and takes its part's
    /// height). Its two ends are its vertices' own points. Refuses a point
    /// the secant does not bring onto the curve (to 1e-9 of the radius)
    /// within half a spacing of the polyline.
    pub fn edge_points(&self, envelope: &BallEnvelope, index: usize, spacing: f64) -> Result<Vec<Vec3>, String> {
        self.edge_points_in(&Parts::new(envelope, self.top), index, spacing)
    }

    /// [`Plan::edge_points`] on any field, `spacing` in its chart.
    pub fn edge_points_in(&self, field: &dyn Field, index: usize, spacing: f64) -> Result<Vec<Vec3>, String> {
        let edge = &self.edges[index];
        let parts = field;
        let r = field.scale();
        let points = &edge.points;
        let mut along = vec![0.0f64];
        for w in points.windows(2) {
            along.push(along.last().unwrap() + ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt());
        }
        let length = *along.last().unwrap();
        if !(spacing.is_finite() && spacing > 0.0) {
            return Err("the spacing must be positive".into());
        }
        // a closed edge at least 16 stretches round (cross review: a short rim
        // came back two coincident points), an open one at least 4
        let fewest = if edge.from == edge.to { 16 } else { 4 };
        let count = ((length / spacing).ceil() as usize).clamp(fewest, 1 << 20);
        let mut out = vec![points[0]];
        let mut stretch = 0usize;
        for k in 1..count {
            let s = length * k as f64 / count as f64;
            while stretch + 1 < points.len() - 1 && along[stretch + 1] < s {
                stretch += 1;
            }
            let (a, b) = (points[stretch], points[stretch + 1]);
            let span = along[stretch + 1] - along[stretch];
            let t = if span > 0.0 { (s - along[stretch]) / span } else { 0.0 };
            let (x, y) = (a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
            if edge.right == Part::Outside {
                out.push(Vec3::new(x, y, parts.height(edge.left, x, y).min(self.top)));
                continue;
            }
            // square to the stretch, onto the curve
            let (nx, ny) = if span > 0.0 { (-(b.y - a.y) / span, (b.x - a.x) / span) } else { (0.0, 0.0) };
            let g = |l: f64| parts.equation(edge.left, edge.right, x + nx * l, y + ny * l);
            let (mut l0, mut l1) = (0.0f64, 1e-6 * r / field.length());
            let (mut g0, mut g1) = (g(l0), g(l1));
            for _ in 0..50 {
                if !(g1.is_finite() && g0.is_finite()) || g1 == g0 || g1.abs() <= 1e-15 * r {
                    break;
                }
                let l2 = l1 - g1 * (l1 - l0) / (g1 - g0);
                (l0, g0, l1) = (l1, g1, l2);
                g1 = g(l1);
            }
            let (px, py) = (x + nx * l1, y + ny * l1);
            if !(l1.abs() <= 0.5 * spacing && parts.meets(edge.left, edge.right, px, py, 1e-9 * r)) {
                return Err(format!("a point of the boundary between {:?} and {:?} near ({x:.6}, {y:.6}) is not brought onto it", edge.left, edge.right));
            }
            let z = parts.height(edge.left, px, py).min(parts.height(edge.right, px, py)).min(self.top);
            let z = if z.is_finite() { z } else { parts.continued(edge.left, px, py) };
            out.push(Vec3::new(px, py, z));
        }
        out.push(*points.last().unwrap());
        Ok(out)
    }
}

impl<'a> Builder<'a> {
    /// The plan's own check: at every crossing the two parts' heights agree
    /// (read just either side of the crossing, where each is read) and
    /// nothing is lower; at every solved vertex its parts' surfaces meet and
    /// nothing is lower.
    fn check(&self, plan: &Plan) -> Result<(), String> {
        let tolerance = self.tolerance;
        for crossing in &self.crossings {
            let [(a, p), (b, q)] = crossing.ends;
            let at = crossing.point;
            if !self.parts.meets(p, q, at.x, at.y, tolerance) {
                return Err(format!(
                    "the crossing ({:.9}, {:.9}) is not where {p:?} and {q:?} meet (they read {} and {}; edge from {a:?} to {b:?})",
                    at.x, at.y, self.parts.continued(p, at.x, at.y), self.parts.continued(q, at.x, at.y)
                ));
            }
            let (lowest, part) = self.parts.lowest(at.x, at.y);
            if lowest < at.z - tolerance {
                return Err(format!("at the crossing ({:.9}, {:.9}) of {p:?} and {q:?} at {} the part {part:?} is lower, at {lowest}", at.x, at.y, at.z));
            }
        }
        for (point, parts) in &self.vertices {
            for &part in parts {
                let z = self.parts.continued(part, point.x, point.y);
                if !((z - point.z).abs() <= tolerance) {
                    return Err(format!("at the vertex ({:.9}, {:.9}) the part {part:?} reads {z}, not {}", point.x, point.y, point.z));
                }
            }
            for (i, &a) in parts.iter().enumerate() {
                for &b in &parts[i + 1..] {
                    if let Some(d) = self.parts.plane(a, b, point.x, point.y).filter(|d| !(d.abs() <= tolerance)) {
                        return Err(format!("the vertex ({:.9}, {:.9}) of {parts:?} is {d:e} off the surface {a:?} and {b:?} meet on", point.x, point.y));
                    }
                }
            }
            let (lowest, part) = self.parts.lowest(point.x, point.y);
            if lowest < point.z - tolerance {
                return Err(format!("at the vertex ({:.9}, {:.9}) the part {part:?} is lower, at {lowest}", point.x, point.y));
            }
        }
        for edge in &plan.edges {
            if edge.left == edge.right {
                return Err(format!("a boundary has {:?} on both sides", edge.left));
            }
        }
        // Witnesses of parts the grid may pass over: each run's lowest line
        // (below its moves, at half the cell's spacing) and each ball's
        // lowest point, inside the box. A part lowest at one of them that the
        // plan does not name is a region narrower than the cell.
        let [x0, x1, y0, y1] = plan.bounds;
        let mut named: std::collections::HashSet<Part> = plan.edges.iter().flat_map(|e| [e.left, e.right]).collect();
        for vertex in &plan.vertices {
            named.insert(self.parts.lowest(vertex.point.x, vertex.point.y).1);
        }
        let inside = |x: f64, y: f64| x >= x0 && x <= x1 && y >= y0 && y <= y1;
        for [x, y] in self.parts.witnesses(self.cell) {
            if inside(x, y) {
                let (_, part) = self.parts.lowest(x, y);
                if !named.contains(&part) {
                    return Err(format!("the part {part:?}, lowest at ({x:.6}, {y:.6}), is narrower than the cell: read at a smaller cell"));
                }
            }
        }
        Ok(())
    }
}

