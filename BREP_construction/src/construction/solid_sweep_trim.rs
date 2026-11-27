//! Self-intersection trims of a solid sweep's envelope, found from exact membership (behind
//! `BREP_SWEEP_TRIM_ENGINE`, default on, `=0` refuses polyhedra as before).
//!
//! The boundary of the set a rigid tool fills along a motion lies on the
//! envelope sheets (families of grazing points) and on the faces of the start
//! and end poses (cap sheets). A sheet point is on the boundary unless some
//! pose holds it strictly. Its own pose touches it (depth 0), so the test runs
//! over the poses outside the OWN-TIME BASIN: walking away from the own time
//! while the depth keeps falling. Where the depth rises at once instead (a
//! fold), the neighbouring poses hold the point. Each sheet also carries an
//! exact LOCAL sign function whose zero is the sheet's own fold edge (the
//! silhouette end of an edge's trace; the trailing/leading split of a cap
//! face), so that edge is found by a smooth function, not by the competitor.
//!
//! This file holds the evaluators: rigid poses of a two-turn motion, the
//! convex tool's depth, the pose grid with the basin walk, and the sheets of a
//! convex polyhedron. The assembly of trimmed faces builds on them.
pub use crate::{KernelRefusal, KernelStage, Vec3};
use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
use crate::{BrepSolid, NurbsCurve, NurbsSurface, OrRefuse, Vec4};

fn frag<T>(result: Result<T, String>, what: &'static str) -> Result<T, KernelRefusal> {
    result.or_refuse(KernelStage::Fragment, what)
}

/// `BREP_SWEEP_TRIM_END_SLIVERS` (default on, `=0` leaves them unmated as before): unmated arcs the imprint
/// leaves within two time steps of the motion's start or end, shorter than
/// the closure band, are contracted and the arcs mated again
/// (`Arrangement::contract_unmated_end_slivers`). Only an arrangement that
/// would not close reaches it.
fn end_slivers_on() -> bool {
    crate::switch!("BREP_SWEEP_TRIM_END_SLIVERS", true)
}

/// `BREP_SWEEP_TRIM_TIP_SPLICE` (on unless =0): a loop through two preimages
/// of one world point (a double curve's end on its sheet: the sheet meets
/// itself there) runs on through both instead of being split there as a
/// pinch ([`Arrangement::split_pinched_loops`]), so each such vertex keeps one
/// link and the sweep is built instead of refused `trim_ring_tips`; such a
/// face is then cut between the two preimages
/// ([`ConvexSweep::cut_self_meeting_loops`]) so no face uses an edge twice;
/// and a ring tumbled through a whole turn is taken (the time cap 360°, not
/// 200°).
pub fn tip_splice_on() -> bool {
    crate::switch!("BREP_SWEEP_TRIM_TIP_SPLICE", true)
}

/// `BREP_SWEEP_SHORT_RETURNING_SPLIT` (default off, `=1` on): a ring
/// envelope's double curve that runs out to a tip and back is split at its tip
/// however short it is ([`Arrangement::split_returning_self_arcs`]). Otherwise
/// only one longer than four tolerances is split, and a short one stays one
/// arc that has no mate (an arc cannot mate itself): the tumbling torus turned
/// the other way over 70..100 degrees leaves one such double curve, 0.014
/// long, where it starts at the motion's first pose, and the trimmed sheets do
/// not close.
pub fn short_returning_split_on() -> bool {
    crate::switch!("BREP_SWEEP_SHORT_RETURNING_SPLIT", false)
}

/// `BREP_SWEEP_TRIM_ONE_SIDED` (default on, `=0` gives the vertex up as before): the vertex solve's Jacobian
/// read one-sided where a sheet ends within the difference step (a strip's
/// zero line leaving its face at the vertex), where it gave the vertex up.
fn one_sided_on() -> bool {
    crate::switch!("BREP_SWEEP_TRIM_ONE_SIDED", true)
}

/// `BREP_SWEEP_TRIM_STRIP_JACOBIAN` (default on, `=0` reads du alone as before): a strip face's chart
/// orientation by its parameter map's whole Jacobian, not its first
/// derivative alone (a strip chart that runs the sheet's time the other way
/// built its face inside out).
fn strip_jacobian_on() -> bool {
    crate::switch!("BREP_SWEEP_TRIM_STRIP_JACOBIAN", true)
}

thread_local! {
    static END_SLIVERS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

/// End slivers contracted on this thread since the last call (the check
/// reading of `BREP_SWEEP_TRIM_END_SLIVERS`).
pub fn trim_end_slivers_take() -> u64 {
    END_SLIVERS.with(|cell| cell.replace(0))
}

/// The engine's switch (default on; `=0` refuses a tumbled polyhedron as before).
pub fn trim_engine_on() -> bool {
    crate::switch!("BREP_SWEEP_TRIM_ENGINE", true)
}

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("sweepSolid: {message}"))
}

/// The whole sweep of a convex polyhedron (`vertices`, `faces` as loops of
/// vertex indices, either winding) along a two-turn motion for t in
/// [`from`, `to`], by the trim engine: the sheets' kept regions are found
/// on a pose grid, closed into one oriented shell, refined onto the exact
/// sheets and built. The grid is doubled once when the first does not
/// resolve the arrangement. The result must validate and its volume must
/// agree with the arrangement's own flux integral.
pub fn sweep_convex_tumbled(vertices: &[Vec3], faces: &[Vec<usize>], motion: &TwoTurnMotion, from: f64, to: f64) -> Result<BrepSolid, KernelRefusal> {
    if !(from.is_finite() && to.is_finite() && to > from) {
        return Err(refuse("trim_range", "the motion's range must run forward".into()));
    }
    let tool = Convex::new(vertices, faces)?;
    if tool.polygons.iter().any(|p| p.len() > 4) {
        return Err(refuse("trim_tool", "the trim engine takes faces of three or four corners".into()));
    }
    // Resolution by the fastest turn's angle: ten poses and two rows a degree.
    let degrees = motion.rates.iter().fold(0.0f64, |a, r| a.max(r.abs())) * (to - from).to_degrees();
    let mut last = None;
    for scale in [1usize, 2] {
        let poses = ((10.0 * degrees).ceil() as usize).clamp(750, 6000) * scale + 1;
        let rows = ((2.0 * degrees).ceil() as usize).clamp(150, 1200) * (1 + scale) / 2 + 1;
        let columns = 40 * (1 + scale) + 1;
        let grid = PoseGrid::new(motion, &tool, from, to, poses);
        let sweep = ConvexSweep::new(&tool, motion, &grid);
        let mut arrangement = sweep.arrangement(rows, columns);
        let closure = arrangement.close(1e-2 * sweep.size, 2e-2 * sweep.size);
        if !closure.is_sphere() {
            last = Some(refuse("trim_arrangement", format!("the trimmed sheets do not close into one oriented shell on a {poses}-pose grid")));
            continue;
        }
        let refined = sweep.refine_arcs(&arrangement, &closure);
        let solved = sweep.solve_vertices(&arrangement, &closure, &refined);
        let body = sweep.build(&arrangement, &closure, &refined, &solved)?;
        if !body.validate().is_empty() {
            return Err(refuse("trim_assembly", "the built sweep does not validate".into()));
        }
        let built = frag(crate::solid_mass_properties(&body).map(|p| p.volume), "trim_volume")?;
        let (flux, _, _) = sweep.flux_volume(&arrangement, &closure, 2000);
        if (built - flux).abs() > 1e-5 * flux.abs().max(sweep.size.powi(3)) {
            return Err(refuse("trim_volume", format!("the built sweep's volume {built:.7} disagrees with its trims' {flux:.7}")));
        }
        return Ok(body);
    }
    Err(last.expect("tried"))
}

/// A rigid motion `x -> m x + b`.
#[derive(Clone, Copy, Debug)]
pub struct Rigid {
    pub m: [[f64; 3]; 3],
    pub b: Vec3,
}

fn mul(a: &[[f64; 3]; 3], v: Vec3) -> Vec3 {
    Vec3::new(
        a[0][0] * v.x + a[0][1] * v.y + a[0][2] * v.z,
        a[1][0] * v.x + a[1][1] * v.y + a[1][2] * v.z,
        a[2][0] * v.x + a[2][1] * v.y + a[2][2] * v.z,
    )
}

fn mul_transposed(a: &[[f64; 3]; 3], v: Vec3) -> Vec3 {
    Vec3::new(
        a[0][0] * v.x + a[1][0] * v.y + a[2][0] * v.z,
        a[0][1] * v.x + a[1][1] * v.y + a[2][1] * v.z,
        a[0][2] * v.x + a[1][2] * v.y + a[2][2] * v.z,
    )
}

impl Rigid {
    /// The turn by `angle` (right-handed) about the line through `point` along
    /// the unit `direction`.
    pub fn turn(point: Vec3, direction: Vec3, angle: f64) -> Rigid {
        let (s, c) = angle.sin_cos();
        let k = direction;
        let kk = [[k.x * k.x, k.x * k.y, k.x * k.z], [k.y * k.x, k.y * k.y, k.y * k.z], [k.z * k.x, k.z * k.y, k.z * k.z]];
        let kx = [[0.0, -k.z, k.y], [k.z, 0.0, -k.x], [-k.y, k.x, 0.0]];
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = (if i == j { c } else { 0.0 }) + s * kx[i][j] + (1.0 - c) * kk[i][j];
            }
        }
        Rigid { m, b: point.sub(mul(&m, point)) }
    }

    /// This motion, then `next`.
    pub fn then(&self, next: &Rigid) -> Rigid {
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = (0..3).map(|k| next.m[i][k] * self.m[k][j]).sum();
            }
        }
        Rigid { m, b: mul(&next.m, self.b).add(next.b) }
    }

    pub fn apply(&self, x: Vec3) -> Vec3 {
        mul(&self.m, x).add(self.b)
    }

    pub fn rotate(&self, v: Vec3) -> Vec3 {
        mul(&self.m, v)
    }

    pub fn invert(&self, y: Vec3) -> Vec3 {
        mul_transposed(&self.m, y.sub(self.b))
    }

    pub fn unrotate(&self, v: Vec3) -> Vec3 {
        mul_transposed(&self.m, v)
    }
}

/// Two turns driven by one parameter: `rates[0]·t` about the first axis, then
/// `rates[1]·t` about the second; both axes fixed in the world.
#[derive(Clone, Copy, Debug)]
pub struct TwoTurnMotion {
    pub points: [Vec3; 2],
    /// Unit directions.
    pub directions: [Vec3; 2],
    pub rates: [f64; 2],
}

impl TwoTurnMotion {
    pub fn pose(&self, t: f64) -> Rigid {
        let first = Rigid::turn(self.points[0], self.directions[0], self.rates[0] * t);
        first.then(&Rigid::turn(self.points[1], self.directions[1], self.rates[1] * t))
    }

    /// The world velocity, at t, of the material point at world position `y`.
    pub fn velocity(&self, t: f64, y: Vec3) -> Vec3 {
        let second = Rigid::turn(self.points[1], self.directions[1], self.rates[1] * t);
        let z = second.invert(y);
        let inner = self.directions[0].cross(z.sub(self.points[0])).scale(self.rates[0]);
        self.directions[1].cross(y.sub(self.points[1])).scale(self.rates[1]).add(second.rotate(inner))
    }
}

/// A convex polyhedron: outward unit normals and offsets (`n·x <= d` inside),
/// each face's polygon counter-clockwise about its normal, and each edge with
/// the faces on its two sides.
#[derive(Clone, Debug)]
pub struct Convex {
    pub vertices: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub offsets: Vec<f64>,
    pub polygons: Vec<Vec<usize>>,
    /// `[a, b, f, g]`: the edge from vertex a to b between faces f and g.
    pub edges: Vec<[usize; 4]>,
}

impl Convex {
    pub fn new(vertices: &[Vec3], faces: &[Vec<usize>]) -> Result<Convex, KernelRefusal> {
        let n = vertices.len() as f64;
        let centre = vertices.iter().fold(Vec3::ZERO, |s, v| s.add(*v)).scale(1.0 / n.max(1.0));
        let size = vertices.iter().map(|v| v.sub(centre).length()).fold(0.0, f64::max);
        let mut normals = Vec::new();
        let mut offsets = Vec::new();
        let mut polygons = Vec::new();
        for face in faces {
            if face.len() < 3 || face.iter().any(|&i| i >= vertices.len()) {
                return Err(refuse("trim_tool", "a face of the tool has fewer than three corners".into()));
            }
            // Newell's normal.
            let mut normal = Vec3::ZERO;
            for (k, &i) in face.iter().enumerate() {
                let (a, b) = (vertices[i], vertices[face[(k + 1) % face.len()]]);
                normal = normal.add(Vec3::new((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y)));
            }
            let mut normal = normal.normalized().map_err(|_| refuse("trim_tool", "a face of the tool has no area".into()))?;
            let mut polygon = face.clone();
            if normal.dot(vertices[face[0]].sub(centre)) < 0.0 {
                normal = normal.scale(-1.0);
                polygon.reverse();
            }
            let offset = normal.dot(vertices[face[0]]);
            if face.iter().any(|&i| (normal.dot(vertices[i]) - offset).abs() > 1e-9 * size) {
                return Err(refuse("trim_tool", "a face of the tool is not planar".into()));
            }
            if vertices.iter().any(|v| normal.dot(*v) > offset + 1e-9 * size) {
                return Err(refuse("trim_tool", "the tool is not convex".into()));
            }
            normals.push(normal);
            offsets.push(offset);
            polygons.push(polygon);
        }
        let mut sides: std::collections::BTreeMap<(usize, usize), Vec<usize>> = Default::default();
        for (f, polygon) in polygons.iter().enumerate() {
            for (k, &a) in polygon.iter().enumerate() {
                let b = polygon[(k + 1) % polygon.len()];
                sides.entry((a.min(b), a.max(b))).or_default().push(f);
            }
        }
        let mut edges = Vec::new();
        for ((a, b), fs) in sides {
            if fs.len() != 2 {
                return Err(refuse("trim_tool", "an edge of the tool does not join exactly two faces".into()));
            }
            edges.push([a, b, fs[0], fs[1]]);
        }
        Ok(Convex { vertices: vertices.to_vec(), normals, offsets, polygons, edges })
    }

    /// How far inside the tool (in its own frame) x lies: positive strictly
    /// inside, zero on the boundary.
    pub fn depth(&self, x: Vec3) -> f64 {
        self.normals.iter().zip(&self.offsets).map(|(n, d)| d - n.dot(x)).fold(f64::INFINITY, f64::min)
    }

    fn centre_and_reach(&self) -> (Vec3, f64) {
        let centre = self.vertices.iter().fold(Vec3::ZERO, |s, v| s.add(*v)).scale(1.0 / self.vertices.len() as f64);
        (centre, self.vertices.iter().map(|v| v.sub(centre).length()).fold(0.0, f64::max))
    }
}

/// A tool the pose grid can test points against: how deep (in its own frame)
/// a point lies, positive strictly inside, zero on the boundary, changing no
/// faster than the point moves (1-Lipschitz, so the competitor's speed bound
/// holds), and a ball (centre, reach) holding the tool. The competitor's ball
/// test passes a pose whose ball's clearance `reach − |x − centre|` cannot beat
/// the best depth so far: exact for positive depths (a held point lies in the
/// ball no deeper than that clearance), which is all a sheet's visibility
/// reads; a negative depth may read lower than its true value past the ball.
pub trait Depth {
    fn depth(&self, x: Vec3) -> f64;
    fn centre_and_reach(&self) -> (Vec3, f64);
    /// The tool's faces as planes (n, d), inside n·x <= d, when it is the
    /// intersection of them (the pose table's case).
    fn planes(&self) -> Option<(&[Vec3], &[f64])> {
        None
    }
}

impl Depth for Convex {
    fn depth(&self, x: Vec3) -> f64 {
        Convex::depth(self, x)
    }
    fn centre_and_reach(&self) -> (Vec3, f64) {
        Convex::centre_and_reach(self)
    }
    fn planes(&self) -> Option<(&[Vec3], &[f64])> {
        Some((&self.normals, &self.offsets))
    }
}

/// A ring torus: the points within `minor` of the circle of radius `major`
/// about `axis` (unit) through `centre`.
#[derive(Clone, Copy, Debug)]
pub struct Torus {
    pub centre: Vec3,
    pub axis: Vec3,
    pub major: f64,
    pub minor: f64,
}

impl Torus {
    pub fn new(centre: Vec3, axis: Vec3, major: f64, minor: f64) -> Result<Torus, KernelRefusal> {
        let finite = |v: Vec3| v.x.is_finite() && v.y.is_finite() && v.z.is_finite();
        if !(finite(centre) && finite(axis)) {
            return Err(refuse("trim_tool", "the ring's centre or axis is not finite".into()));
        }
        // Scaled by its largest component first: a long axis does not
        // overflow its length.
        let big = axis.x.abs().max(axis.y.abs()).max(axis.z.abs());
        let axis = if big > 0.0 { axis.scale(1.0 / big) } else { axis };
        let axis = axis.normalized().map_err(|_| refuse("trim_tool", "the ring's axis has no direction".into()))?;
        if !finite(axis) || (axis.length() - 1.0).abs() > 1e-12 {
            return Err(refuse("trim_tool", "the ring's axis has no direction".into()));
        }
        if !(minor > 0.0 && major > minor && major.is_finite()) {
            return Err(refuse("trim_tool", "the ring is not a ring torus".into()));
        }
        Ok(Torus { centre, axis, major, minor })
    }
}

impl Depth for Torus {
    /// Exactly the signed distance to the surface: the tube's radius less the
    /// distance to the centre circle (1-Lipschitz, so the speed bound holds).
    fn depth(&self, x: Vec3) -> f64 {
        let d = x.sub(self.centre);
        let along = d.dot(self.axis);
        let across = d.sub(self.axis.scale(along)).length();
        self.minor - (across - self.major).hypot(along)
    }
    fn centre_and_reach(&self) -> (Vec3, f64) {
        (self.centre, self.major + self.minor)
    }
}

/// What the poses outside a sheet point's own-time basin make of it.
#[derive(Clone, Copy, Debug)]
pub struct Competitor {
    /// Minus the largest depth of the point in those poses (positive: no pose
    /// holds it), clamped to 1.
    pub clearance: f64,
    /// The neighbouring poses hold it (the depth rises at once from its own
    /// time): it is hidden whatever the far poses do.
    pub fold: bool,
    /// The pose of the largest depth.
    pub at: usize,
    /// The own-time basin's first and last pose (excluded).
    pub basin: (usize, usize),
}

/// The motion's poses at evenly spaced times.
/// (Read-only once made: the pose table is made from the poses.)
pub struct PoseGrid {
    /// The motion the poses sample (the basin jump reads its velocity).
    motion: TwoTurnMotion,
    /// Each pose's second turn alone, as `TwoTurnMotion::velocity` builds it.
    seconds: Vec<Rigid>,
    /// The farthest a pose's centre lies from the origin (for rounding).
    farthest: f64,
    times: Vec<f64>,
    /// Whether the times are the equally spaced ones of [`PoseGrid::new`]
    /// (else increasing, and a pose is found by search).
    uniform: bool,
    poses: Vec<Rigid>,
    centres: Vec<Vec3>,
    reach: f64,
    /// (a, p, b): the motion's speed at a world point y is at most
    /// a·|y − p| + b at every time.
    speed_bound: Option<(f64, Vec3, f64)>,
    /// The tool's faces (n, d) the table was made from, and each pose's faces
    /// turned into the world, (N, e) with e = d − n·c (c the tool's centre),
    /// so the depth of y is min(e − N·(y − C)) with C the pose's centre: no
    /// large terms cancel near the tool however far the pose is moved (when
    /// BREP_SWEEP_POSE_TABLE is on).
    planes: Option<PoseTable>,
}

struct PoseTable {
    faces: Vec<(Vec3, f64)>,
    table: Vec<[f64; 4]>,
}

impl PoseGrid {
    pub fn new<T: Depth>(motion: &TwoTurnMotion, tool: &T, t0: f64, t1: f64, count: usize) -> PoseGrid {
        let count = count.max(2);
        let times: Vec<f64> = (0..count).map(|i| t0 + (t1 - t0) * i as f64 / (count - 1) as f64).collect();
        Self::with_times(motion, tool, times, true)
    }

    /// [`PoseGrid::new`] at the given times (a grid denser where the motion's
    /// sheets change fast): None unless there are two or more, strictly
    /// increasing and finite.
    pub fn new_with_times<T: Depth>(motion: &TwoTurnMotion, tool: &T, times: Vec<f64>) -> Option<PoseGrid> {
        if times.len() < 2 || !times.iter().all(|t| t.is_finite()) || !times.windows(2).all(|w| w[1] > w[0]) {
            return None;
        }
        Some(Self::with_times(motion, tool, times, false))
    }

    /// The grid at `times`; `uniform` when they are equally spaced (a pose
    /// is then found by its index, else by search).
    fn with_times<T: Depth>(motion: &TwoTurnMotion, tool: &T, times: Vec<f64>, uniform: bool) -> PoseGrid {
        let poses: Vec<Rigid> = times.iter().map(|&t| motion.pose(t)).collect();
        let (centre, reach) = tool.centre_and_reach();
        let centres: Vec<Vec3> = poses.iter().map(|p| p.apply(centre)).collect();
        // The outer turn moves y at |r1|·dist(y, its axis) <= |r1|·|y − p1|;
        // the inner one at |r0|·dist(z, its axis) <= |r0|·(|z − p1| + |p1 − p0|),
        // z = y turned back about the outer axis, so |z − p1| = |y − p1|.
        let [r0, r1] = motion.rates;
        let (p0, p1) = (motion.points[0], motion.points[1]);
        let speed_bound = Some((r0.abs() + r1.abs(), p1, r0.abs() * p1.sub(p0).length())).filter(|b| b.0.is_finite() && b.2.is_finite());
        // x = Rᵀ(y − b) and C = R c + b, so n·x = Rn·(y − C) + n·c.
        let planes = tool.planes().filter(|_| pose_table_on()).map(|(normals, offsets)| {
            let faces: Vec<(Vec3, f64)> = normals.iter().copied().zip(offsets.iter().copied()).collect();
            let mut table = Vec::with_capacity(poses.len() * faces.len());
            for pose in &poses {
                for (n, d) in &faces {
                    let w = pose.rotate(*n);
                    table.push([w.x, w.y, w.z, d - n.dot(centre)]);
                }
            }
            PoseTable { faces, table }
        });
        let seconds = if trim_reuse_on() { times.iter().map(|&t| Rigid::turn(motion.points[1], motion.directions[1], motion.rates[1] * t)).collect() } else { Vec::new() };
        let farthest = centres.iter().map(|c: &Vec3| c.length()).fold(0.0, f64::max);
        PoseGrid { motion: *motion, seconds, farthest, times, uniform, poses, centres, reach, speed_bound, planes }
    }

    pub fn times(&self) -> &[f64] {
        &self.times
    }

    pub fn poses(&self) -> &[Rigid] {
        &self.poses
    }

    #[doc(hidden)]
    pub fn speed_bound(&self) -> Option<(f64, Vec3, f64)> {
        self.speed_bound
    }

    /// Whether pose `i`'s bounding ball shows it cannot hold `y` deeper than
    /// `best` (the competitor scan's test, sound for depths above zero).
    #[doc(hidden)]
    pub fn ball_misses(&self, i: usize, y: Vec3, best: f64) -> bool {
        self.reach - y.sub(self.centres[i]).length() <= best
    }

    /// The pose table, when it was made from `tool`'s faces.
    fn table_for<T: Depth>(&self, tool: &T) -> Option<&PoseTable> {
        let table = self.planes.as_ref()?;
        let (normals, offsets) = tool.planes()?;
        let same = table.faces.len() == normals.len()
            && table.faces.iter().zip(normals.iter().zip(offsets)).all(|((n, d), (m, e))| n.x.to_bits() == m.x.to_bits() && n.y.to_bits() == m.y.to_bits() && n.z.to_bits() == m.z.to_bits() && d.to_bits() == e.to_bits());
        same.then_some(table)
    }

    pub fn depth<T: Depth>(&self, tool: &T, i: usize, y: Vec3) -> f64 {
        self.depth_with(self.table_for(tool), tool, i, y)
    }

    /// The depth through `table` when given (made from `tool`), else in the
    /// tool's frame.
    fn depth_with<T: Depth>(&self, table: Option<&PoseTable>, tool: &T, i: usize, y: Vec3) -> f64 {
        match table {
            Some(table) => self.table_depth(table, i, y),
            None => tool.depth(self.poses[i].invert(y)),
        }
    }

    /// Whether the depth of `y` is shown to fall at every step from pose `i`
    /// to pose `j` (either way), by more than rounding: the walk would pass
    /// every pose between. `depth`: the depth read at `i`.
    ///
    /// Each face's term `G = d − n·x(t)` (read centred, as the walk reads it),
    /// x the preimage of y, has
    /// `|G''| <= M`, M the bound on |x''| the two turns give (see `turns_bend`):
    /// over the run (length Δ in t, step h) a face either falls everywhere
    /// (`G' <= s + M·Δ`, s its slope at i in the walk's direction, at most
    /// −2ρ/h, so each step falls by more than twice the rounding ρ) or stays
    /// above the depth at i by more than 3ρ (`G >= g + min(s, 0)·Δ − M·Δ²/2`),
    /// so it is never the least. The least at every pose is then a falling
    /// face, and each read depth is at most the one before.
    fn falls_through(&self, table: &PoseTable, y: Vec3, i: usize, j: usize, depth: f64) -> bool {
        let (ti, tj) = (self.times[i], self.times[j]);
        let delta = (tj - ti).abs();
        let steps = i.abs_diff(j) as f64;
        if !(delta > 0.0 && steps >= 1.0) {
            return false;
        }
        let h = delta / steps;
        // Each face's term read as the walk reads it, centred on the pose's
        // centre (`table_depth`: e − N·(y − C)), so nothing cancels however
        // far the tool lies from the origin; its slope N·v (= −n·x', the
        // preimage's velocity x' = −Rᵀv, v the material velocity at y).
        let faces = table.faces.len();
        let q = y.sub(self.centres[i]);
        let v = match self.seconds.get(i) {
            // `TwoTurnMotion::velocity` at the pose's time, its second turn read once.
            Some(second) => {
                let m = &self.motion;
                let z = second.invert(y);
                let inner = m.directions[0].cross(z.sub(m.points[0])).scale(m.rates[0]);
                m.directions[1].cross(y.sub(m.points[1])).scale(m.rates[1]).add(second.rotate(inner))
            }
            None => self.motion.velocity(ti, y),
        };
        // Along the walk in time (a grid may run backwards in t).
        let v = if tj > ti { v } else { v.scale(-1.0) };
        let bend = turns_bend(&self.motion, y);
        // The rounding of a term: of its parts' sizes (the tool, the offset
        // from the pose's centre), of the velocity read at y, and of the
        // pose's centre and the point themselves, placed in absolute
        // coordinates (the centres are the tool's turned and moved there).
        let absolute = y.length() + self.centres[i].length() + self.motion.points[0].length() + self.motion.points[1].length();
        // And of the poses' angles, rate·t rounded: a pose a little off the
        // motion's own moves the tool by that angle times its arm.
        let [r0, r1] = self.motion.rates;
        let arm = self.reach + q.length() + y.sub(self.motion.points[1]).length() + self.motion.points[1].sub(self.motion.points[0]).length();
        let angles = (r0.abs() + r1.abs()) * (ti.abs() + tj.abs()) * arm;
        let rho = 64.0 * f64::EPSILON * (self.reach + q.length() + absolute + angles + delta * (v.length() + bend * delta));
        if !(bend.is_finite() && rho.is_finite() && rho > 0.0) {
            return false;
        }
        for row in &table.table[i * faces..(i + 1) * faces] {
            let normal = Vec3::new(row[0], row[1], row[2]);
            let g = row[3] - normal.dot(q);
            let slope = normal.dot(v);
            let falls = slope + bend * delta <= -2.0 * rho / h;
            let above = g + slope.min(0.0) * delta - 0.5 * bend * delta * delta > depth + 3.0 * rho;
            if !(falls || above) {
                return false;
            }
        }
        true
    }

    fn table_depth(&self, table: &PoseTable, i: usize, y: Vec3) -> f64 {
        let faces = table.faces.len();
        let q = y.sub(self.centres[i]);
        let mut depth = f64::INFINITY;
        for p in &table.table[i * faces..(i + 1) * faces] {
            depth = depth.min(p[3] - (p[0] * q.x + p[1] * q.y + p[2] * q.z));
        }
        depth
    }

    /// The competitor of the world point `y` whose own pose is `own`. `side`:
    /// 0 for an envelope point (poses on both sides), +1 for a start-cap point
    /// (later poses only), −1 for an end-cap point (earlier poses only).
    pub fn competitor<T: Depth>(&self, tool: &T, y: Vec3, own: usize, side: i32) -> Competitor {
        let table = self.table_for(tool);
        let n = self.times.len();
        // The basin on each side: [lo, hi] excluded.
        let mut lo = own;
        let mut hi = own;
        let mut fold = false;
        for sign in [1i64, -1] {
            if side != 0 && sign != side as i64 {
                if sign > 0 {
                    hi = n - 1;
                } else {
                    lo = 0;
                }
                continue;
            }
            let step = |i: usize| -> Option<usize> {
                let j = i as i64 + sign;
                (j >= 0 && j < n as i64).then_some(j as usize)
            };
            let Some(first) = step(own) else { continue };
            let last;
            let mut previous = self.depth_with(table, tool, first, y);
            if previous > 1e-12 {
                fold = true;
            }
            let mut i = first;
            let jump = table.filter(|_| basin_jump_on());
            let mut quiet = 0usize;
            let mut run = BASIN_RUN;
            // Walk while the depth keeps falling.
            loop {
                // A run of poses the depth is shown to fall through, step by
                // step beyond rounding, is passed at once (`basin_jump`).
                if let Some(t) = jump {
                    if quiet == 0 {
                        let target = if sign > 0 { (i + run).min(n - 1) } else { i.saturating_sub(run) };
                        if target != i && self.falls_through(t, y, i, target, previous) {
                            i = target;
                            previous = self.depth_with(table, tool, i, y);
                            // A run passed: try one twice as long next.
                            run = (2 * run).min(BASIN_RUN_MOST);
                            continue;
                        }
                        if run > BASIN_RUN {
                            // A long run failed: try the short one at once.
                            run = BASIN_RUN;
                            continue;
                        }
                        quiet = BASIN_QUIET;
                    } else {
                        quiet -= 1;
                    }
                }
                let Some(next) = step(i) else {
                    last = i;
                    break;
                };
                let value = self.depth_with(table, tool, next, y);
                if value > previous + 1e-15 {
                    last = i;
                    break;
                }
                previous = value;
                i = next;
            }
            if sign > 0 {
                hi = last;
            } else {
                lo = last;
            }
        }
        if let Some(speed) = self.speed_bound.filter(|_| pose_bound_on()) {
            let (best, at) = self.deepest_with(table, tool, y, speed, own, [(0, lo), (hi + 1, n)]);
            return Competitor { clearance: (-best).min(1.0), fold, at, basin: (lo, hi) };
        }
        let mut best = f64::NEG_INFINITY;
        let mut at = own;
        for i in (0..lo).chain(hi + 1..n) {
            // A pose whose bounding ball misses y holds it no deeper than this.
            if self.reach - y.sub(self.centres[i]).length() <= best {
                continue;
            }
            let value = self.depth_with(table, tool, i, y);
            if value > best {
                best = value;
                at = i;
            }
        }
        Competitor { clearance: (-best).min(1.0), fold, at, basin: (lo, hi) }
    }

    /// The competitor scan over the poses of `ranges` (each [start, end)) in
    /// order: a pose whose bounding ball cannot beat the best so far is passed,
    /// else the deepest so far wins (first of equals); (best, its pose, `own`
    /// when none). The same answer as testing pose by pose, passing runs of
    /// poses none of which could change it: a point's depth moves no faster
    /// than the point moves in the tool's frame, the motion's speed at `y`, at
    /// most `speed.0 · |y − speed.1| + speed.2`; so between two poses of
    /// depths da and db, Δt apart, it stays under (da + db)/2 + speed·Δt/2,
    /// and a run under the best so far changes nothing.
    #[doc(hidden)]
    pub fn deepest<T: Depth>(&self, tool: &T, y: Vec3, speed: (f64, Vec3, f64), own: usize, ranges: [(usize, usize); 2]) -> (f64, usize) {
        self.deepest_with(self.table_for(tool), tool, y, speed, own, ranges)
    }

    fn deepest_with<T: Depth>(&self, table: Option<&PoseTable>, tool: &T, y: Vec3, speed: (f64, Vec3, f64), own: usize, ranges: [(usize, usize); 2]) -> (f64, usize) {
        const RUN: usize = 16;
        let rate = speed.0 * y.sub(speed.1).length() + speed.2;
        // Rounding in a depth (a few ulps of the tool's and the point's size),
        // and of the absolute coordinates the poses' centres are placed at and
        // of their angles (rate·t rounded, times the arm), which far from the
        // origin or at large times exceed it.
        let [r0, r1] = self.motion.rates;
        let (t_first, t_last) = (self.times[0], self.times[self.times.len() - 1]);
        let absolute = y.length() + self.farthest + self.motion.points[0].length() + self.motion.points[1].length();
        let arm = self.reach + y.sub(self.centres[0]).length() + y.sub(self.motion.points[1]).length() + self.motion.points[1].sub(self.motion.points[0]).length();
        let angles = (r0.abs() + r1.abs()) * (t_first.abs().max(t_last.abs())) * arm;
        let slack = 1e-12 * (self.reach + y.sub(self.centres[0]).length() + rate) + 64.0 * f64::EPSILON * (absolute + angles);
        let mut best = f64::NEG_INFINITY;
        let mut at = own;
        // The pose-by-pose rule, `value` the pose's depth when already known.
        let step = |i: usize, value: Option<f64>, best: &mut f64, at: &mut usize| {
            if self.reach - y.sub(self.centres[i]).length() <= *best {
                return;
            }
            let value = value.unwrap_or_else(|| self.depth_with(table, tool, i, y));
            if value > *best {
                *best = value;
                *at = i;
            }
        };
        for (start, end) in ranges {
            if start >= end {
                continue;
            }
            let mut a = start;
            let mut da = self.depth_with(table, tool, a, y);
            step(a, Some(da), &mut best, &mut at);
            while a + 1 < end {
                let b = (a + RUN).min(end - 1);
                let db = self.depth_with(table, tool, b, y);
                let bound = 0.5 * (da + db) + 0.5 * rate * (self.times[b] - self.times[a]).abs() + slack;
                if bound > best {
                    for i in a + 1..b {
                        step(i, None, &mut best, &mut at);
                    }
                }
                step(b, Some(db), &mut best, &mut at);
                (a, da) = (b, db);
            }
        }
        (best, at)
    }
}

/// Whether each polyline lies within `tolerance` of the other, read both
/// ways. The gap to a polyline moves no faster than the point does, so each
/// segment read at steps h (at most a quarter of the tolerance) with every
/// gap within the tolerance less h/2 holds the whole segment within it.
fn polylines_within(a: &[Vec3], b: &[Vec3], tolerance: f64) -> bool {
    if !(tolerance > 0.0) {
        return false;
    }
    let near = |from: &[Vec3], to: &[Vec3]| {
        if from.len() == 1 {
            return polyline_gap(to, from[0]) <= tolerance;
        }
        from.windows(2).all(|w| {
            let length = w[1].sub(w[0]).length();
            let steps = (4.0 * length / tolerance).ceil().clamp(1.0, 1e6) as usize;
            let h = length / steps as f64;
            (0..=steps).all(|k| polyline_gap(to, w[0].add(w[1].sub(w[0]).scale(k as f64 / steps as f64))) + 0.5 * h <= tolerance)
        })
    };
    near(a, b) && near(b, a)
}

/// The distance from `x` to the polyline through `points`.
fn polyline_gap(points: &[Vec3], x: Vec3) -> f64 {
    if points.len() == 1 {
        return x.sub(points[0]).length();
    }
    points.windows(2).fold(f64::INFINITY, |best, w| {
        let d = w[1].sub(w[0]);
        let len2 = d.dot(d);
        let s = if len2 > 0.0 { (x.sub(w[0]).dot(d) / len2).clamp(0.0, 1.0) } else { 0.0 };
        best.min(x.sub(w[0].add(d.scale(s))).length())
    })
}

/// `BREP_SWEEP_TRIM_EDGE_FLOOR` (default off: with it on, the crossed
/// forward tumble's refined edges leave its tessellation two edges unmatched;
/// held until that is understood): the trim engine refines each
/// edge's stations toward its trims meeting it on their faces' surfaces
/// within half the pcurve refinement tolerance, so a boolean the solid enters
/// has fewer trims to refit. Best effort, not a bound: an edge whose doubling
/// stops reading better (or reaches 1536 stations) keeps its best reading,
/// which may be above that, and a boolean refits it as before.
pub fn trim_edge_floor_on() -> bool {
    crate::switch!("BREP_SWEEP_TRIM_EDGE_FLOOR", false)
}

/// Whether the trim grid reads a node's sign alone where its own point is
/// outside its pose, its full value only where a crossing needs it (default
/// on, `=0` reads every value; the same contour).
fn grid_lazy_on() -> bool {
    crate::switch!("BREP_SWEEP_GRID_LAZY", true)
}

/// Whether the trim engine reuses what it has read: each pose's second turn
/// for the velocity, and a search's depths at times it reads again (default
/// on, `=0` reads each again; the same values, bit for bit).
fn trim_reuse_on() -> bool {
    crate::switch!("BREP_SWEEP_TRIM_REUSE", true)
}

/// Poses a basin jump tries to pass at once, and single steps after a failed
/// try before the next.
const BASIN_RUN: usize = 32;
const BASIN_RUN_MOST: usize = 512;
const BASIN_QUIET: usize = 8;

/// A bound on |x''| for the preimage x(t) of a fixed world point y under the
/// two turns: z(t) = y turned back about the second axis, x = z turned back
/// about the first, so |x''| <= r0²·|z − p0| + 2|r0|·|z'| + |z''| with
/// |z − p0| <= |y − p1| + |p1 − p0|, |z'| <= |r1|·|y − p1|, |z''| <= r1²·|y − p1|.
pub(crate) fn turns_bend(motion: &TwoTurnMotion, y: Vec3) -> f64 {
    let [r0, r1] = motion.rates;
    // A rate whose square underflows bends no less for it over a long time:
    // no bound read then (the jump declines).
    if [r0, r1].iter().any(|&r| r != 0.0 && !(r * r).is_normal()) {
        return f64::INFINITY;
    }
    let (p0, p1) = (motion.points[0], motion.points[1]);
    let a = y.sub(p1).length();
    let b = p1.sub(p0).length();
    (r0 * r0 * (a + b) + 2.0 * r0.abs() * r1.abs() * a + r1 * r1 * a) * (1.0 + 1e-12)
}

/// Whether the competitor's basin walk passes runs of poses the depth is
/// shown to fall through (default on, `=0` walks pose by pose; the same basin).
fn basin_jump_on() -> bool {
    crate::switch!("BREP_SWEEP_BASIN_JUMP", true)
}

/// Whether the pose grid tests a point against each pose's faces as world
/// planes (on unless BREP_SWEEP_POSE_TABLE=0; the same depths to rounding).
fn pose_table_on() -> bool {
    crate::switch!("BREP_SWEEP_POSE_TABLE", true)
}

/// Whether the trim engine's competitor search passes runs of poses its
/// speed bound shows cannot hold a point deeper (on unless
/// BREP_SWEEP_POSE_BOUND=0; the same answer either way, only faster).
fn pose_bound_on() -> bool {
    crate::switch!("BREP_SWEEP_POSE_BOUND", true)
}

/// Whether a grid edge's contour crossing is solved by a safeguarded
/// regula falsi (Illinois) instead of 48 halvings (default on;
/// BREP_SWEEP_TRIM_SECANT=0 restores the halvings): the same final bracket width, 2^-48 of the
/// edge, in a few evaluations where the value is smooth across it.
fn trim_secant_on() -> bool {
    crate::switch!("BREP_SWEEP_TRIM_SECANT", true)
}

/// The bracket `(lo, hi)` of [0, 1], at most 2^-48 wide, about where `f`
/// leaves the sign class of `f0 = f(0)` (positive, or not) for that of
/// `f1 = f(1)`: `lo` on `f0`'s side, `hi` on the other, as halving would
/// leave them. Illinois steps; an end creeping onto the root in strides far
/// below the bracket is followed by a probe twice its stride beyond it (across
/// the root where the creep converges). After 12 reads, halvings: at most
/// 12 + 48 reads where the value jumps or kinks, a dozen where it is smooth.
#[doc(hidden)]
pub fn bracket_sign_change(f: impl Fn(f64) -> f64, f0: f64, f1: f64) -> (f64, f64) {
    const WIDTH: f64 = 1.0 / (1u64 << 48) as f64;
    const BUDGET: usize = 12;
    let side = f0 > 0.0;
    let (mut lo, mut hi, mut flo, mut fhi) = (0.0f64, 1.0f64, f0, f1);
    // Which end the last step moved (1: lo, -1: hi): an end kept twice
    // running has its value halved.
    let mut last = 0i32;
    let mut reads = 0usize;
    let mut probe: Option<f64> = None;
    while hi - lo > WIDTH {
        let width = hi - lo;
        let m = if reads >= BUDGET {
            0.5 * (lo + hi)
        } else if let Some(p) = probe {
            p
        } else {
            let secant = lo + width * flo / (flo - fhi);
            if secant > lo && secant < hi {
                secant
            } else if secant <= lo {
                lo + 0.5 * WIDTH
            } else if secant >= hi {
                hi - 0.5 * WIDTH
            } else {
                0.5 * (lo + hi)
            }
        };
        probe = None;
        // Clear of the ends, so every read shrinks the bracket.
        let m = m.clamp(lo + 0.25 * WIDTH, hi - 0.25 * WIDTH);
        let fm = f(m);
        reads += 1;
        let moved_lo = (fm > 0.0) == side;
        let stride = if moved_lo { m - lo } else { hi - m };
        if moved_lo {
            lo = m;
            flo = fm;
            if last == 1 {
                fhi *= 0.5;
            }
            last = 1;
        } else {
            hi = m;
            fhi = fm;
            if last == -1 {
                flo *= 0.5;
            }
            last = -1;
        }
        if stride < 1e-2 * width {
            let reach = (2.0 * stride).max(0.5 * WIDTH);
            probe = Some(if moved_lo { lo + reach } else { hi - reach });
            last = 0;
        }
    }
    (lo, hi)
}

/// One sheet's sample census: its name, how many grid nodes it has, how many
/// of them are hidden.
#[derive(Clone, Debug, PartialEq)]
pub struct SheetCensus {
    pub name: String,
    pub nodes: usize,
    pub hidden: usize,
}

/// A face's characteristic segment at time t (where the normal speed, affine
/// on the face, is zero; a corner of zero speed is an end), in the tool's frame, its ends ordered along the
/// segment's own direction (normal × the speed's gradient) so consecutive
/// times never swap them.
pub fn face_strip(tool: &Convex, motion: &TwoTurnMotion, pose: &Rigid, t: f64, face: usize) -> Option<[Vec3; 2]> {
    let polygon = &tool.polygons[face];
    let normal = pose.rotate(tool.normals[face]);
    let speeds: Vec<f64> = polygon.iter().map(|&i| motion.velocity(t, pose.apply(tool.vertices[i])).dot(normal)).collect();
    let mut ends = Vec::new();
    // Half-open: a corner of zero speed counts as non-negative, so a
    // segment ending exactly at a corner has that corner as one end, once.
    for k in 0..polygon.len() {
        let (sa, sb) = (speeds[k], speeds[(k + 1) % polygon.len()]);
        if (sa >= 0.0) != (sb >= 0.0) {
            let (a, b) = (tool.vertices[polygon[k]], tool.vertices[polygon[(k + 1) % polygon.len()]]);
            ends.push(a.add(b.sub(a).scale(sa / (sa - sb))));
        }
    }
    if ends.len() != 2 {
        return None;
    }
    // The fastest corner lies on the positive side of the segment.
    let top = (0..polygon.len()).max_by(|&a, &b| speeds[a].total_cmp(&speeds[b]))?;
    let side = tool.normals[face].dot(ends[1].sub(ends[0]).cross(tool.vertices[polygon[top]].sub(ends[0])));
    Some(if side > 0.0 { [ends[0], ends[1]] } else { [ends[1], ends[0]] })
}

/// The silhouette sign of an edge point (positive where its two faces' normal
/// speeds have opposite signs, so the edge grazes), scaled by the speed.
pub fn edge_silhouette(tool: &Convex, motion: &TwoTurnMotion, pose: &Rigid, t: f64, edge: usize, x: Vec3) -> f64 {
    let [_, _, f, g] = tool.edges[edge];
    let v = motion.velocity(t, pose.apply(x));
    let (sf, sg) = (v.dot(pose.rotate(tool.normals[f])), v.dot(pose.rotate(tool.normals[g])));
    -sf * sg / v.length_squared().max(1e-300)
}

/// The census of every sheet of a convex tool under a two-turn motion: face
/// strips and edge traces on a grid of `rows` times × `samples` along the
/// feature, caps on a `samples` × `samples` grid over each face's bounding
/// rectangle (nodes outside the polygon are not counted). The grid's times are
/// every `stride`-th pose.
pub fn sheet_census(tool: &Convex, motion: &TwoTurnMotion, grid: &PoseGrid, stride: usize, samples: usize) -> Vec<SheetCensus> {
    let n = grid.times.len();
    let rows: Vec<usize> = (0..n).step_by(stride.max(1)).collect();
    let lams: Vec<f64> = (0..samples).map(|i| i as f64 / (samples - 1) as f64).collect();
    let mut out = Vec::new();
    for face in 0..tool.polygons.len() {
        let (mut nodes, mut hidden) = (0, 0);
        for &row in &rows {
            let (t, pose) = (grid.times[row], grid.poses[row]);
            let Some([a, b]) = face_strip(tool, motion, &pose, t, face) else { continue };
            for &lam in &lams {
                let y = pose.apply(a.add(b.sub(a).scale(lam)));
                let c = grid.competitor(tool, y, row, 0);
                nodes += 1;
                if c.fold || c.clearance <= 0.0 {
                    hidden += 1;
                }
            }
        }
        out.push(SheetCensus { name: format!("face{face}"), nodes, hidden });
    }
    for (e, &[a, b, _, _]) in tool.edges.iter().enumerate() {
        let (mut nodes, mut hidden) = (0, 0);
        for &row in &rows {
            let (t, pose) = (grid.times[row], grid.poses[row]);
            for &lam in &lams {
                let x = tool.vertices[a].add(tool.vertices[b].sub(tool.vertices[a]).scale(lam));
                let local = edge_silhouette(tool, motion, &pose, t, e, x);
                let c = grid.competitor(tool, pose.apply(x), row, 0);
                let value = if c.fold { local.min(-1e-12) } else { c.clearance.min(local) };
                nodes += 1;
                if value <= 0.0 {
                    hidden += 1;
                }
            }
        }
        out.push(SheetCensus { name: format!("edge{a}{b}"), nodes, hidden });
    }
    for (which, own, side) in [("startcap", 0usize, 1i32), ("endcap", n - 1, -1)] {
        let (t, pose) = (grid.times[own], grid.poses[own]);
        for face in 0..tool.polygons.len() {
            let polygon = &tool.polygons[face];
            let normal = tool.normals[face];
            let origin = tool.vertices[polygon[0]];
            let u = tool.vertices[polygon[1]].sub(origin).normalized().unwrap_or(Vec3::X);
            let w = normal.cross(u);
            let coords: Vec<(f64, f64)> = polygon.iter().map(|&i| (tool.vertices[i].sub(origin).dot(u), tool.vertices[i].sub(origin).dot(w))).collect();
            let (u0, u1) = coords.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), c| (a.min(c.0), b.max(c.0)));
            let (w0, w1) = coords.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), c| (a.min(c.1), b.max(c.1)));
            let span = (u1 - u0).max(w1 - w0);
            let (mut nodes, mut hidden) = (0, 0);
            for &p in &lams {
                for &q in &lams {
                    let (cu, cw) = (u0 + (u1 - u0) * p, w0 + (w1 - w0) * q);
                    let inside = (0..coords.len()).all(|k| {
                        let (a, b) = (coords[k], coords[(k + 1) % coords.len()]);
                        (b.0 - a.0) * (cw - a.1) - (b.1 - a.1) * (cu - a.0) >= -1e-12 * span * span
                    });
                    if !inside {
                        continue;
                    }
                    let x = origin.add(u.scale(cu)).add(w.scale(cw));
                    let y = pose.apply(x);
                    let v = motion.velocity(t, y);
                    let local = -(side as f64) * v.dot(pose.rotate(normal)) / v.length().max(1e-300);
                    let c = grid.competitor(tool, y, own, side);
                    let value = if c.fold { local.min(-1e-12) } else { c.clearance.min(local) };
                    nodes += 1;
                    if value <= 0.0 {
                        hidden += 1;
                    }
                }
            }
            out.push(SheetCensus { name: format!("{which}{face}"), nodes, hidden });
        }
    }
    out
}

/// A sheet of a convex tool's sweep.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SheetId {
    /// The characteristic segment of a face: parameters (λ along it, time
    /// fraction).
    Strip(usize),
    /// The trace of an edge, the whole edge: (μ along it, time fraction); the
    /// silhouette sign clips it.
    Trace(usize),
    /// A face of the start (0) or end (1) pose: its bounding rectangle's
    /// fractions (u, w); outside the polygon is outside the sheet.
    Cap(u8, usize),
    /// A ring's envelope: its grazing branch (0 lower, 1 upper edge of the
    /// leading band) over half (0, 1) of the ring's turns: (ψ fraction, time
    /// fraction).
    Envelope(u8, u8),
    /// A ring's band at the start (0: the trailing band) or end (1: the
    /// leading band) pose over half of the ring's turns: (ψ fraction,
    /// fraction across the band from one grazing branch to the other).
    Band(u8, u8),
    /// A ring's collar about a point where the whole tube circle grazes (the
    /// grazing harmonic's a = b = 0; `ring::RingCollar`): both branches as one
    /// sheet through that circle, (λ fraction across [−ε, ε], tube angle
    /// fraction); λ > 0 is the lower branch, λ < 0 the upper.
    Collar(u8),
}

impl SheetId {
    /// A cap-like sheet: an end pose's face, charted (u, w) with no time.
    pub fn is_cap(&self) -> bool {
        matches!(self, SheetId::Cap(..) | SheetId::Band(..))
    }

    pub fn is_ring(&self) -> bool {
        matches!(self, SheetId::Envelope(..) | SheetId::Band(..) | SheetId::Collar(..))
    }
}

/// Who a boundary arc of a kept region borders.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Border {
    Sheet(SheetId),
    /// The track of a tool vertex (an edge trace's end).
    Vertex(usize),
    /// Nothing found (an unresolved label: counted, never matched).
    Unknown,
}

/// A sheet point: its world position, its tool-frame position, its own time,
/// and its local sign (positive where the sheet itself is exposed).
#[derive(Clone, Copy, Debug)]
pub struct SheetPoint {
    pub world: Vec3,
    pub tool: Vec3,
    pub t: f64,
    pub own: usize,
    pub local: f64,
}

/// A piece of a kept region's boundary: its partner, its ends in the
/// sheet's parameters and in the world; the kept side lies on its left.
#[derive(Clone, Copy, Debug)]
pub struct Segment {
    pub border: Border,
    pub pa: [f64; 2],
    pub pb: [f64; 2],
    pub a: Vec3,
    pub b: Vec3,
}

/// The switches forced on the calling thread, for jobs it hands the shared
/// pool (a pool thread does not share the caller's thread-locals); none
/// outside tests.
#[cfg(feature = "parallel")]
fn forced_switches() -> Vec<(&'static str, bool)> {
    let forced: Vec<(&'static str, bool)> = Vec::new();
    forced
}

/// Runs `job` on a pool thread with the caller's `forced` switches, restoring
/// the thread's own after.
#[cfg(feature = "parallel")]
fn with_forced<R>(forced: &[(&'static str, bool)], job: impl FnOnce() -> R) -> R {
    let _ = forced;
    job()
}

/// What decides a sheet point's visibility.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Active {
    Local,
    Fold,
    Competitor(usize),
}

/// The tool a sweep carries.
#[derive(Clone, Copy)]
pub enum Tool<'a> {
    Convex(&'a Convex),
    Ring(&'a Torus),
}

/// The sweep of a convex tool (or a ring) along a two-turn motion, sampled.
pub struct ConvexSweep<'a> {
    tool: Tool<'a>,
    pub motion: &'a TwoTurnMotion,
    pub grid: &'a PoseGrid,
    size: f64,
    /// A ring's seam: the angle about its axis where its sheets' two halves
    /// meet (0 for a polyhedron).
    seam: f64,
    /// A ring's envelope chart stretched about where its grazing branches all
    /// but meet (`ring::RingWarp`; None: the chart is (ψ fraction, τ)).
    warp: Option<ring::RingWarp>,
    /// A ring's collars about the points its whole tube circle grazes
    /// (`BREP_SWEEP_RING_COLLAR`); their discs are cut from the envelopes.
    collars: Vec<ring::RingCollar>,
}

/// One face's frame in its plane (a strip's chart): origin, in-plane axes,
/// the polygon's coordinates.
struct FaceFrame {
    origin: Vec3,
    u: Vec3,
    w: Vec3,
    coords: Vec<(f64, f64)>,
}

impl<'a> ConvexSweep<'a> {
    pub fn new(tool: &'a Convex, motion: &'a TwoTurnMotion, grid: &'a PoseGrid) -> Self {
        let (_, reach) = tool.centre_and_reach();
        ConvexSweep { tool: Tool::Convex(tool), motion, grid, size: reach, seam: 0.0, warp: None, collars: Vec::new() }
    }

    /// The sweep of a ring, where the ring's grazing loop is two branches
    /// all through the motion (each tube circle grazed at two angles at every
    /// time on the grid); refused otherwise (a ring grazed all round, or a
    /// loop whose branches meet: no sheets would stand for it).
    pub fn new_ring(torus: &'a Torus, motion: &'a TwoTurnMotion, grid: &'a PoseGrid) -> Result<Self, KernelRefusal> {
        Self::new_ring_with_seam(torus, motion, grid, 0.0)
    }

    /// As [`ConvexSweep::new_ring`], its halves meeting at the angle `seam`
    /// about the ring's axis (reduced to [0, 2π); a non-finite seam is refused).
    pub fn new_ring_with_seam(torus: &'a Torus, motion: &'a TwoTurnMotion, grid: &'a PoseGrid, seam: f64) -> Result<Self, KernelRefusal> {
        if !seam.is_finite() {
            return Err(refuse("trim_ring_seam", "the ring's seam angle is not finite".into()));
        }
        let seam = seam.rem_euclid(std::f64::consts::TAU);
        let (_, reach) = Depth::centre_and_reach(torus);
        let sweep = ConvexSweep { tool: Tool::Ring(torus), motion, grid, size: reach, seam, warp: None, collars: Vec::new() };
        const ROUND: usize = 64;
        // Under the collar switch a circle grazed all round at an isolated
        // point is the collar's (a pose may land on one by chance, at one or
        // two of the stations); more of a pose's circles grazed all round is
        // not two branches (a motion standing still grazes every circle).
        // An isolated point lands on a station at one pose at most: a station
        // grazed all round at two poses is a circle that keeps grazing, no
        // collar's.
        let allowed = if ring::ring_collar_on() { 2 } else { 0 };
        let mut grazed_before = vec![false; ROUND];
        for &t in &grid.times {
            let grazed: Vec<usize> = (0..ROUND).filter(|&k| sweep.ring_branches(t, std::f64::consts::TAU * k as f64 / ROUND as f64).is_none()).collect();
            if grazed.len() > allowed || grazed.iter().any(|&k| std::mem::replace(&mut grazed_before[k], true)) {
                return Err(refuse("trim_ring_branches", "the ring's grazing loop is not two branches all through the motion".into()));
            }
        }
        Ok(sweep)
    }

    /// The polyhedron this sweep carries (None for a ring).
    pub fn convex_tool(&self) -> Option<&'a Convex> {
        match self.tool {
            Tool::Convex(tool) => Some(tool),
            Tool::Ring(_) => None,
        }
    }

    /// The convex tool (a convex sheet's own).
    fn convex(&self) -> &'a Convex {
        match self.tool {
            Tool::Convex(tool) => tool,
            Tool::Ring(_) => unreachable!("a polyhedron's sheet of a ring's sweep"),
        }
    }

    /// The ring (a ring sheet's own).
    fn ring(&self) -> &'a Torus {
        match self.tool {
            Tool::Ring(torus) => torus,
            Tool::Convex(_) => unreachable!("a ring's sheet of a polyhedron's sweep"),
        }
    }

    /// The tool's depth at a tool-frame point.
    fn depth(&self, x: Vec3) -> f64 {
        match self.tool {
            Tool::Convex(tool) => tool.depth(x),
            Tool::Ring(torus) => Depth::depth(torus, x),
        }
    }

    fn centre_and_reach(&self) -> (Vec3, f64) {
        match self.tool {
            Tool::Convex(tool) => tool.centre_and_reach(),
            Tool::Ring(torus) => Depth::centre_and_reach(torus),
        }
    }

    fn t_of(&self, tau: f64) -> f64 {
        let (t0, t1) = (self.grid.times[0], *self.grid.times.last().expect("poses"));
        t0 + (t1 - t0) * tau
    }

    fn own_index(&self, t: f64) -> usize {
        let n = self.grid.times.len();
        let (t0, t1) = (self.grid.times[0], self.grid.times[n - 1]);
        if self.grid.uniform {
            return (((t - t0) / (t1 - t0) * (n - 1) as f64).round().max(0.0) as usize).min(n - 1);
        }
        // Unequal times (a stretched grid): the nearest of them.
        let times = &self.grid.times;
        let k = times.partition_point(|&s| s < t).min(n - 1);
        if k > 0 && (t - times[k - 1]).abs() <= (times[k] - t).abs() { k - 1 } else { k }
    }

    /// A cap's chart: the bilinear map of the unit square onto its polygon
    /// (a quadrilateral's corners in order; a triangle's last corner taken
    /// twice, that side collapsed to a vertex), so every polygon edge is a
    /// side of the parameter domain. In the tool's frame.
    fn cap_at(&self, face: usize, p: [f64; 2]) -> Vec3 {
        let polygon = &self.convex().polygons[face];
        let c = |k: usize| self.convex().vertices[polygon[k.min(polygon.len() - 1)]];
        let (u, w) = (p[0], p[1]);
        c(0).scale((1.0 - u) * (1.0 - w)).add(c(1).scale(u * (1.0 - w))).add(c(2).scale(u * w)).add(c(3).scale((1.0 - u) * w))
    }

    fn frame(&self, face: usize) -> FaceFrame {
        let tool = self.convex();
        let polygon = &tool.polygons[face];
        let origin = tool.vertices[polygon[0]];
        let u = tool.vertices[polygon[1]].sub(origin).normalized().unwrap_or(Vec3::X);
        let w = tool.normals[face].cross(u);
        let coords: Vec<(f64, f64)> = polygon.iter().map(|&i| (tool.vertices[i].sub(origin).dot(u), tool.vertices[i].sub(origin).dot(w))).collect();
        FaceFrame { origin, u, w, coords }
    }

    /// The sheet's point at parameters `p` (None outside the sheet).
    pub fn point(&self, sheet: SheetId, p: [f64; 2]) -> Option<SheetPoint> {
        if sheet.is_ring() {
            return self.ring_point(sheet, p);
        }
        let tool = self.convex();
        match sheet {
            SheetId::Envelope(..) | SheetId::Band(..) | SheetId::Collar(..) => unreachable!("a ring sheet in a polyhedron's arm"),
            SheetId::Strip(face) => {
                let t = self.t_of(p[1]);
                let pose = self.motion.pose(t);
                let [a, b] = face_strip(tool, self.motion, &pose, t, face)?;
                let x = a.add(b.sub(a).scale(p[0]));
                let y = pose.apply(x);
                // The face plane's depth at times beside t: its second
                // derivative (the first is the zero normal speed) says
                // whether the neighbouring poses hold the strip (a fold).
                // Taken per radian of the fastest turn, so the same sweep
                // driven faster over a shorter range reads the same.
                let rate = self.motion.rates.iter().fold(0.0f64, |a, r| a.max(r.abs())).max(1e-300);
                let h = 1e-4 / rate;
                let plane = |s: f64| tool.offsets[face] - tool.normals[face].dot(self.motion.pose(s).invert(y));
                let local = -(plane(t + h) + plane(t - h)) / 1e-8;
                Some(SheetPoint { world: y, tool: x, t, own: self.own_index(t), local })
            }
            SheetId::Trace(edge) => {
                let t = self.t_of(p[1]);
                let pose = self.motion.pose(t);
                let [a, b, _, _] = tool.edges[edge];
                let x = tool.vertices[a].add(tool.vertices[b].sub(tool.vertices[a]).scale(p[0]));
                let local = edge_silhouette(tool, self.motion, &pose, t, edge, x);
                Some(SheetPoint { world: pose.apply(x), tool: x, t, own: self.own_index(t), local })
            }
            SheetId::Cap(end, face) => {
                let own = if end == 0 { 0 } else { self.grid.times.len() - 1 };
                let (t, pose) = (self.grid.times[own], self.grid.poses[own]);
                let x = self.cap_at(face, p);
                let y = pose.apply(x);
                let v = self.motion.velocity(t, y);
                let side = if end == 0 { 1.0 } else { -1.0 };
                let local = -side * v.dot(pose.rotate(tool.normals[face])) / v.length().max(1e-300);
                Some(SheetPoint { world: y, tool: x, t, own, local })
            }
        }
    }

    /// The sheet point's visibility value (positive: on the sweep's boundary)
    /// and what decides it.
    /// The depth of the world point `y` at time s, `self.depth(pose(s).invert(y))`;
    /// with `BREP_SWEEP_TRIM_REUSE` the last few times read are kept by their
    /// bits, so a golden-section search reading a time again reads it once.
    fn depth_memo(&self, y: Vec3) -> impl Fn(f64) -> f64 + '_ {
        let reuse = trim_reuse_on();
        let kept: std::cell::RefCell<[(u64, f64); 4]> = std::cell::RefCell::new([(u64::MAX, 0.0); 4]);
        let next = std::cell::Cell::new(0usize);
        move |s: f64| {
            if reuse {
                if let Some(&(_, d)) = kept.borrow().iter().find(|(bits, _)| *bits == s.to_bits()) {
                    return d;
                }
            }
            let d = self.depth(self.motion.pose(s).invert(y));
            if reuse {
                let k = next.get();
                kept.borrow_mut()[k] = (s.to_bits(), d);
                next.set((k + 1) % 4);
            }
            d
        }
    }

    pub fn value(&self, sheet: SheetId, point: &SheetPoint) -> (f64, Active) {
        let side = match sheet {
            SheetId::Cap(0, _) | SheetId::Band(0, _) => 1,
            SheetId::Cap(_, _) | SheetId::Band(_, _) => -1,
            _ => 0,
        };
        // Near its own fold a sheet point can be held by poses closer in time
        // than the pose grid's step (a self-intersection ending at a cusp):
        // probe the continuous motion at geometrically shrinking offsets.
        if point.local > 0.0 && point.local < 0.05 {
            let n = self.grid.times.len();
            let (t0, t1) = (self.grid.times[0], self.grid.times[n - 1]);
            let step = (t1 - t0) / (n - 1) as f64;
            for k in 0..30 {
                let delta = 2.0 * step * 0.5f64.powi(k);
                for sign in [1.0, -1.0] {
                    if side != 0 && sign * side as f64 <= 0.0 {
                        continue;
                    }
                    let s = point.t + sign * delta;
                    if s < t0 || s > t1 {
                        continue;
                    }
                    let depth = self.depth(self.motion.pose(s).invert(point.world));
                    if depth > 1e-13 * self.size {
                        return (-depth, Active::Fold);
                    }
                }
            }
        }
        let mut c = match self.tool {
            Tool::Convex(tool) => self.grid.competitor(tool, point.world, point.own, side),
            Tool::Ring(torus) => self.grid.competitor(torus, point.world, point.own, side),
        };
        // Near a trim, the competitor's depth is its continuous maximum, not
        // the pose grid's (the grid's step costs h² in depth).
        let n = self.grid.times.len();
        if !c.fold && c.clearance < 1e-2 * self.size && c.at > 0 && c.at < n - 1 {
            let lo = if c.at - 1 == c.basin.1 { c.at } else { c.at - 1 };
            let hi = if c.at + 1 == c.basin.0 { c.at } else { c.at + 1 };
            let (lo, hi) = (lo.max(if c.at > c.basin.1 { c.basin.1 + 1 } else { 0 }), hi.min(if c.at < c.basin.0 { c.basin.0 - 1 } else { n - 1 }));
            if hi > lo {
                let depth_at = self.depth_memo(point.world);
                let (mut a, mut b) = (self.grid.times[lo], self.grid.times[hi]);
                let g = 0.5 * (5f64.sqrt() - 1.0);
                for _ in 0..60 {
                    let (x, y) = (b - g * (b - a), a + g * (b - a));
                    if depth_at(x) > depth_at(y) {
                        b = y;
                    } else {
                        a = x;
                    }
                }
                let best = depth_at(0.5 * (a + b));
                c.clearance = c.clearance.min(-best).min(1.0);
            }
        }
        if c.fold {
            return (point.local.min(-1e-12), if point.local < 0.0 { Active::Local } else { Active::Fold });
        }
        if point.local <= c.clearance {
            (point.local, Active::Local)
        } else {
            (c.clearance, Active::Competitor(c.at))
        }
    }

    fn eval(&self, sheet: SheetId, p: [f64; 2]) -> f64 {
        match self.point(sheet, p) {
            Some(point) => self.value(sheet, &point).0,
            None => -1.0,
        }
    }

    /// The partner across a contour point: the strip of the face whose speed
    /// vanishes (an edge trace's silhouette end) or of the cap's own face, or
    /// the feature of the competing pose that holds the point's neighbours.
    fn partner(&self, sheet: SheetId, point: &SheetPoint, active: Active) -> Border {
        if sheet.is_ring() {
            return self.ring_partner(sheet, point, active);
        }
        let tool = self.convex();
        match active {
            // Held by poses next to its own: the feature of the near pose
            // that holds it deepest (the sheet itself where it crosses
            // itself; a neighbouring edge's trace at a tool vertex).
            Active::Fold => {
                let n = self.grid.times.len();
                let (t0, t1) = (self.grid.times[0], self.grid.times[n - 1]);
                let step = (t1 - t0) / (n - 1) as f64;
                let depth_at = self.depth_memo(point.world);
                let g = 0.5 * (5f64.sqrt() - 1.0);
                let mut best: Option<(f64, f64)> = None;
                for sign in [1.0, -1.0] {
                    let (mut a, mut b) = (point.t + sign * 1e-9 * step, point.t + sign * 2.0 * step);
                    if sign < 0.0 {
                        std::mem::swap(&mut a, &mut b);
                    }
                    let (a0, b0) = (a.max(t0), b.min(t1));
                    if b0 <= a0 {
                        continue;
                    }
                    let (mut a, mut b) = (a0, b0);
                    for _ in 0..80 {
                        let (c, d) = (b - g * (b - a), a + g * (b - a));
                        if depth_at(c) > depth_at(d) {
                            b = d;
                        } else {
                            a = c;
                        }
                    }
                    let s = 0.5 * (a + b);
                    let depth = depth_at(s);
                    if best.map_or(true, |x| depth > x.1) {
                        best = Some((s, depth));
                    }
                }
                match best {
                    Some((s, _)) => match self.feature_at(point.world, s, None) {
                        Border::Unknown => Border::Sheet(sheet),
                        other => other,
                    },
                    None => Border::Sheet(sheet),
                }
            }
            Active::Local => match sheet {
                SheetId::Trace(edge) => {
                    let [_, _, f, g] = tool.edges[edge];
                    let pose = self.motion.pose(point.t);
                    let v = self.motion.velocity(point.t, point.world);
                    let (sf, sg) = (v.dot(pose.rotate(tool.normals[f])).abs(), v.dot(pose.rotate(tool.normals[g])).abs());
                    Border::Sheet(SheetId::Strip(if sf < sg { f } else { g }))
                }
                SheetId::Cap(_, face) => Border::Sheet(SheetId::Strip(face)),
                SheetId::Strip(_) => Border::Unknown,
                SheetId::Envelope(..) | SheetId::Band(..) | SheetId::Collar(..) => unreachable!("a ring sheet in a polyhedron's arm"),
            },
            Active::Competitor(at) => {
                let n = self.grid.times.len();
                // Refine the competing time: the depth's maximum near `at`.
                let depth_at = self.depth_memo(point.world);
                let s_star = if at == 0 || at == n - 1 {
                    self.grid.times[at]
                } else {
                    let (mut a, mut b) = (self.grid.times[at - 1], self.grid.times[at + 1]);
                    let g = 0.5 * (5f64.sqrt() - 1.0);
                    for _ in 0..80 {
                        let (c, d) = (b - g * (b - a), a + g * (b - a));
                        if depth_at(c) > depth_at(d) {
                            b = d;
                        } else {
                            a = c;
                        }
                    }
                    0.5 * (a + b)
                };
                let end = if at == 0 { Some(0u8) } else if at == n - 1 { Some(1u8) } else { None };
                self.feature_at(point.world, s_star, end)
            }
        }
    }

    /// The feature of the pose at time `s` nearest the world point `y`: its
    /// face (a strip, or the end pose's cap), or the edge of its two
    /// nearest faces (a trace).
    fn feature_at(&self, y: Vec3, s: f64, end: Option<u8>) -> Border {
        if matches!(self.tool, Tool::Ring(_)) {
            return self.ring_feature_at(y, s, end);
        }
        let tool = self.convex();
        let x = self.motion.pose(s).invert(y);
        let mut gaps: Vec<(f64, usize)> = tool.normals.iter().zip(&tool.offsets).enumerate().map(|(k, (nk, dk))| (dk - nk.dot(x), k)).collect();
        gaps.sort_by(|a, b| a.0.total_cmp(&b.0));
        let tol = 1e-6 * self.size;
        let active_faces: Vec<usize> = gaps.iter().filter(|g| g.0 < gaps[0].0 + tol).map(|g| g.1).collect();
        match (end, active_faces.as_slice()) {
            (Some(e), [f, ..]) => Border::Sheet(SheetId::Cap(e, *f)),
            (None, [f]) => Border::Sheet(SheetId::Strip(*f)),
            (None, [f, g]) => match tool.edges.iter().position(|ed| (ed[2] == *f && ed[3] == *g) || (ed[2] == *g && ed[3] == *f)) {
                Some(edge) => Border::Sheet(SheetId::Trace(edge)),
                None => Border::Unknown,
            },
            _ => Border::Unknown,
        }
    }

    /// The sheets of this tool.
    pub fn sheets(&self) -> Vec<SheetId> {
        if matches!(self.tool, Tool::Ring(_)) {
            let mut out: Vec<SheetId> = [0u8, 1].iter().flat_map(|&a| [0u8, 1].map(move |b| (a, b))).flat_map(|(a, b)| [SheetId::Envelope(a, b), SheetId::Band(a, b)]).collect();
            out.extend((0..self.collars.len()).map(|k| SheetId::Collar(k as u8)));
            return out;
        }
        let mut out: Vec<SheetId> = (0..self.convex().polygons.len()).map(SheetId::Strip).collect();
        out.extend((0..self.convex().edges.len()).map(SheetId::Trace));
        for end in 0..2u8 {
            out.extend((0..self.convex().polygons.len()).map(|f| SheetId::Cap(end, f)));
        }
        out
    }

    /// The boundary arcs of a sheet's kept region, as segments with their
    /// partners (3D ends), on a grid of `rows` × `columns` (caps: `columns` ×
    /// `columns`).
    pub fn boundary(&self, sheet: SheetId, rows: usize, columns: usize) -> Vec<(Border, Vec3, Vec3)> {
        self.segments(sheet, rows, columns).into_iter().map(|s| (s.border, s.a, s.b)).collect()
    }

    /// The boundary of a sheet's kept region as segments in its parameters,
    /// each oriented with the kept side on its left, every contour crossing
    /// bisected onto the exact value's zero (one crossing per grid edge, shared
    /// by the cells on both sides), label switches bisected too.
    pub fn segments(&self, sheet: SheetId, rows: usize, columns: usize) -> Vec<Segment> {
        let (nr, nc) = if sheet.is_cap() { (columns, columns) } else { (rows, columns) };
        // Even steps, graded geometrically into the domain's sides, where
        // trims meet the sides at small angles (end bulges, fold corners).
        let graded = |n: usize| -> Vec<f64> {
            let step = 1.0 / (n - 1) as f64;
            let mut v: Vec<f64> = (0..n).map(|i| i as f64 * step).collect();
            let mut d = step / 4.0;
            while d > 1e-5 {
                v.push(d);
                v.push(1.0 - d);
                d /= 4.0;
            }
            v.sort_by(|a, b| a.total_cmp(b));
            v.dedup();
            v
        };
        let (along_r, along_c) = (graded(nr), graded(nc));
        let (nr, nc) = (along_r.len(), along_c.len());
        let param = |i: usize, j: usize| -> [f64; 2] {
            if sheet.is_cap() { [along_r[i], along_c[j]] } else { [along_c[j], along_r[i]] }
        };
        // A node whose own point is outside its pose (`local < 0`) is negative
        // whatever its competitor (`value` returns that or less): its sign is
        // read without the scan, its full value only where a crossing needs
        // it (`BREP_SWEEP_GRID_LAZY`); every other use reads signs alone.
        let lazy = grid_lazy_on();
        // Each node's value and whether only its sign was read; the rows are
        // independent (read on the shared pool under the `parallel` feature,
        // the same values in the same places).
        let node = |i: usize, j: usize| -> (f64, bool) {
            if lazy {
                if let Some(point) = self.point(sheet, param(i, j)) {
                    if point.local < 0.0 {
                        return (-1.0, true);
                    }
                    return (self.value(sheet, &point).0, false);
                }
                return (-1.0, false);
            }
            (self.eval(sheet, param(i, j)), false)
        };
        let row = |i: usize| -> Vec<(f64, bool)> { (0..nc).map(|j| node(i, j)).collect() };
        #[cfg(feature = "parallel")]
        let nodes: Vec<Vec<(f64, bool)>> = {
            use rayon::prelude::*;
            let forced = forced_switches();
            (0..nr).into_par_iter().map(|i| with_forced(&forced, || row(i))).collect()
        };
        #[cfg(not(feature = "parallel"))]
        let nodes: Vec<Vec<(f64, bool)>> = (0..nr).map(row).collect();
        let unread: std::cell::RefCell<Vec<Vec<bool>>> = std::cell::RefCell::new(nodes.iter().map(|r| r.iter().map(|x| x.1).collect()).collect());
        let values: Vec<Vec<f64>> = nodes.iter().map(|r| r.iter().map(|x| x.0).collect()).collect();
        let values = std::cell::RefCell::new(values);
        // A node's full value, read now if its sign alone was.
        let full = |(i, j): (usize, usize)| -> f64 {
            if unread.borrow()[i][j] {
                let v = self.eval(sheet, param(i, j));
                values.borrow_mut()[i][j] = v;
                unread.borrow_mut()[i][j] = false;
            }
            values.borrow()[i][j]
        };
        let label = |p: [f64; 2]| -> Border {
            match self.point(sheet, p) {
                Some(point) => {
                    let (_, active) = self.value(sheet, &point);
                    self.partner(sheet, &point, active)
                }
                None => Border::Unknown,
            }
        };
        // The contour crossing on the grid edge between nodes a and b (values
        // of opposite sign), computed once per edge.
        let mut cache: std::collections::HashMap<((usize, usize), (usize, usize)), ([f64; 2], Border)> = Default::default();
        let mut crossing = |a: (usize, usize), b: (usize, usize)| -> ([f64; 2], Border) {
            let (a, b) = if a <= b { (a, b) } else { (b, a) };
            if let Some(hit) = cache.get(&(a, b)) {
                return *hit;
            }
            let (p, q, vp) = (param(a.0, a.1), param(b.0, b.1), full(a));
            let at = |s: f64| [p[0] + (q[0] - p[0]) * s, p[1] + (q[1] - p[1]) * s];
            let (lo, hi) = if trim_secant_on() {
                bracket_sign_change(|s| self.eval(sheet, at(s)), vp, full(b))
            } else {
                let (mut lo, mut hi) = (0.0f64, 1.0f64);
                for _ in 0..48 {
                    let m = 0.5 * (lo + hi);
                    if (self.eval(sheet, at(m)) > 0.0) == (vp > 0.0) {
                        lo = m;
                    } else {
                        hi = m;
                    }
                }
                (lo, hi)
            };
            let (kept, outside) = if vp > 0.0 { (lo, hi) } else { (hi, lo) };
            // The decisive function is the one that turns negative across the
            // zero: read it just outside, name the partner from just inside.
            let border = match (self.point(sheet, at(kept)), self.point(sheet, at(outside))) {
                (Some(inside), Some(o)) => self.partner(sheet, &inside, self.value(sheet, &o).1),
                // Into a collar's disc cut from an envelope: the collar.
                (Some(inside), None) => self.ring_hole_border(sheet, at(outside)).unwrap_or_else(|| self.partner(sheet, &inside, self.value(sheet, &inside).1)),
                _ => Border::Unknown,
            };
            let hit = (at(kept), border);
            cache.insert((a, b), hit);
            hit
        };
        let mut raw: Vec<([f64; 2], [f64; 2], Border, Border)> = Vec::new();
        for i in 0..nr - 1 {
            for j in 0..nc - 1 {
                let corners = [(i, j), (i, j + 1), (i + 1, j + 1), (i + 1, j)];
                let kept: Vec<bool> = corners.iter().map(|&(a, b)| values.borrow()[a][b] > 0.0).collect();
                let count = kept.iter().filter(|&&k| k).count();
                if count == 0 || count == 4 {
                    continue;
                }
                let mut cuts = Vec::new();
                for k in 0..4 {
                    if kept[k] != kept[(k + 1) % 4] {
                        cuts.push(crossing(corners[k], corners[(k + 1) % 4]));
                    }
                }
                // A saddle: the centre says which diagonal is joined. Joined
                // like corner 0, the contours cut off corners 1 and 3.
                let pairs: Vec<(usize, usize)> = if cuts.len() == 2 {
                    vec![(0, 1)]
                } else {
                    let (p, q) = (param(i, j), param(i + 1, j + 1));
                    let centre = self.eval(sheet, [0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1])]) > 0.0;
                    if centre == kept[0] {
                        vec![(0, 1), (2, 3)]
                    } else {
                        vec![(0, 3), (1, 2)]
                    }
                };
                // The corner each pair cuts off (saddles: cut k lies on side k).
                let cut_side: Vec<usize> = (0..4).filter(|&k| kept[k] != kept[(k + 1) % 4]).collect();
                for (a, b) in pairs {
                    let (mut pa, mut pb) = (cuts[a], cuts[b]);
                    // A corner on the kept side of this segment: for one pair, any
                    // kept corner; for a saddle, the corner between its two sides
                    // if kept, else the corner across.
                    let corner = if cuts.len() == 2 {
                        (0..4).find(|&k| kept[k]).expect("a kept corner")
                    } else {
                        let between = (cut_side[a].max(cut_side[b]) + if cut_side[a].min(cut_side[b]) == 0 && cut_side[a].max(cut_side[b]) == 3 { 1 } else { 0 }) % 4;
                        if kept[between] { between } else { (between + 2) % 4 }
                    };
                    let k = param(corners[corner].0, corners[corner].1);
                    let d = [pb.0[0] - pa.0[0], pb.0[1] - pa.0[1]];
                    let r = [k[0] - pa.0[0], k[1] - pa.0[1]];
                    // Kept on the left.
                    if d[0] * r[1] - d[1] * r[0] < 0.0 {
                        std::mem::swap(&mut pa, &mut pb);
                    }
                    raw.push((pa.0, pb.0, pa.1, pb.1));
                }
            }
        }
        // The domain's sides where kept, counter-clockwise round the domain.
        let sides: Vec<(usize, Vec<(usize, usize)>)> = match sheet {
            _ if sheet.is_cap() => vec![
                (2, (0..nr).map(|i| (i, 0)).collect()),
                (1, (0..nc).map(|j| (nr - 1, j)).collect()),
                (3, (0..nr).rev().map(|i| (i, nc - 1)).collect()),
                (0, (0..nc).rev().map(|j| (0, j)).collect()),
            ],
            _ => vec![
                (0, (0..nc).map(|j| (0, j)).collect()),
                (3, (0..nr).map(|i| (i, nc - 1)).collect()),
                (1, (0..nc).rev().map(|j| (nr - 1, j)).collect()),
                (2, (0..nr).rev().map(|i| (i, 0)).collect()),
            ],
        };
        for (side, nodes) in &sides {
            for pair in nodes.windows(2) {
                let (na, nb) = (pair[0], pair[1]);
                let (va, vb) = (values.borrow()[na.0][na.1], values.borrow()[nb.0][nb.1]);
                if va <= 0.0 && vb <= 0.0 {
                    continue;
                }
                let (mut from, mut to) = (param(na.0, na.1), param(nb.0, nb.1));
                if va <= 0.0 || vb <= 0.0 {
                    let (cut, _) = crossing(na, nb);
                    if va > 0.0 {
                        to = cut;
                    } else {
                        from = cut;
                    }
                }
                let border = |p: [f64; 2]| self.side_border(sheet, *side, p);
                raw.push((from, to, border(from), border(to)));
            }
        }
        // Split where the two ends' labels differ (a vertex inside).
        let mut out = Vec::new();
        for (pa, pb, la, lb) in raw {
            let is_side = |p: [f64; 2]| p[0] <= 0.0 || p[0] >= 1.0 || p[1] <= 0.0 || p[1] >= 1.0;
            let on_side = is_side(pa) && is_side(pb) && (pa[0] == pb[0] || pa[1] == pb[1]);
            let name = |p: [f64; 2]| -> Border {
                if on_side {
                    // which side: the shared coordinate
                    let side = if pa[1] == pb[1] && pa[1] <= 0.0 { if sheet.is_cap() { 2 } else { 0 } }
                        else if pa[1] == pb[1] { if sheet.is_cap() { 3 } else { 1 } }
                        else if pa[0] <= 0.0 { if sheet.is_cap() { 0 } else { 2 } }
                        else if sheet.is_cap() { 1 } else { 3 };
                    self.side_border(sheet, side, p)
                } else {
                    label(p)
                }
            };
            // Read the end labels just inside the segment (the ends are vertices).
            let inner = |p: [f64; 2], q: [f64; 2]| [p[0] + (q[0] - p[0]) * 1e-6, p[1] + (q[1] - p[1]) * 1e-6];
            let (first, last) = if on_side { (name(inner(pa, pb)), name(inner(pb, pa))) } else { (la, lb) };
            if first == last {
                out.push(self.make_segment(sheet, pa, pb, first));
                continue;
            }
            // The switch, found on the curve: both ends stay on the contour
            // and each midpoint is moved onto it across the current chord (the
            // contour can turn sharply at the switch, a triple point).
            let onto = |p: [f64; 2], q: [f64; 2]| -> [f64; 2] {
                let m = [0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1])];
                let d = [q[0] - p[0], q[1] - p[1]];
                let h = (d[0] * d[0] + d[1] * d[1]).sqrt();
                if on_side || h == 0.0 {
                    return m;
                }
                let left = [-d[1] / h, d[0] / h];
                let at = |s: f64| [m[0] + left[0] * s, m[1] + left[1] * s];
                let (mut a, mut b) = (h, -h);
                if !(self.eval(sheet, at(a)) > 0.0 && self.eval(sheet, at(b)) <= 0.0) {
                    return m;
                }
                for _ in 0..48 {
                    let c = 0.5 * (a + b);
                    if self.eval(sheet, at(c)) > 0.0 {
                        a = c;
                    } else {
                        b = c;
                    }
                }
                at(a)
            };
            let (mut lo, mut hi) = (pa, pb);
            for _ in 0..40 {
                let m = onto(lo, hi);
                if name(m) == first {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            let switch = lo;
            out.push(self.make_segment(sheet, pa, switch, first));
            out.push(self.make_segment(sheet, switch, pb, last));
        }
        out.into_iter().flatten().collect()
    }

    fn make_segment(&self, sheet: SheetId, pa: [f64; 2], pb: [f64; 2], border: Border) -> Option<Segment> {
        let a = self.point(sheet, pa)?.world;
        let b = self.point(sheet, pb)?.world;
        Some(Segment { border, pa, pb, a, b })
    }

    /// Who borders a sheet along a side of its domain (side: 0 first row,
    /// 1 last row, 2 first column, 3 last column of the grid).
    fn side_border(&self, sheet: SheetId, side: usize, a: [f64; 2]) -> Border {
        if sheet.is_ring() {
            return self.ring_side_border(sheet, side);
        }
        let _ = a;
        match sheet {
            SheetId::Envelope(..) | SheetId::Band(..) | SheetId::Collar(..) => unreachable!("a ring sheet in a polyhedron's arm"),
            SheetId::Trace(edge) => {
                let [va, vb, f, g] = self.convex().edges[edge];
                match side {
                    0 | 1 => {
                        let Some(point) = self.point(sheet, a) else { return Border::Unknown };
                        let end = if side == 0 { 0u8 } else { 1 };
                        let pose = self.motion.pose(point.t);
                        let v = self.motion.velocity(point.t, point.world);
                        let sign = if end == 0 { 1.0 } else { -1.0 };
                        let trailing = |face: usize| -sign * v.dot(pose.rotate(self.convex().normals[face])) > 0.0;
                        if trailing(f) {
                            Border::Sheet(SheetId::Cap(end, f))
                        } else if trailing(g) {
                            Border::Sheet(SheetId::Cap(end, g))
                        } else {
                            Border::Unknown
                        }
                    }
                    2 => Border::Vertex(va),
                    _ => Border::Vertex(vb),
                }
            }
            SheetId::Strip(face) => match side {
                0 => Border::Sheet(SheetId::Cap(0, face)),
                1 => Border::Sheet(SheetId::Cap(1, face)),
                _ => {
                    let Some(point) = self.point(sheet, a) else { return Border::Unknown };
                    let x = point.tool;
                    let found = self.convex().edges.iter().position(|ed| {
                        (ed[2] == face || ed[3] == face) && x.distance_to_segment(self.convex().vertices[ed[0]], self.convex().vertices[ed[1]], 1e-300) < 1e-9 * self.size
                    });
                    found.map_or(Border::Unknown, |e| Border::Sheet(SheetId::Trace(e)))
                }
            },
            SheetId::Cap(end, face) => {
                let Some(point) = self.point(sheet, a) else { return Border::Unknown };
                let x = point.tool;
                let Some(e) = self.convex().edges.iter().position(|ed| {
                    (ed[2] == face || ed[3] == face) && x.distance_to_segment(self.convex().vertices[ed[0]], self.convex().vertices[ed[1]], 1e-300) < 1e-9 * self.size
                }) else {
                    return Border::Unknown;
                };
                let [_, _, f, g] = self.convex().edges[e];
                let other = if f == face { g } else { f };
                let pose = self.grid.poses[point.own];
                let v = self.motion.velocity(point.t, point.world);
                let (sa, sb) = (v.dot(pose.rotate(self.convex().normals[face])), v.dot(pose.rotate(self.convex().normals[other])));
                if sa * sb < 0.0 {
                    Border::Sheet(SheetId::Trace(e))
                } else {
                    Border::Sheet(SheetId::Cap(end, other))
                }
            }
        }
    }

    /// The enclosed volume by the divergence theorem over the closed
    /// arrangement's faces: each loop's polygon in its sheet's parameters,
    /// integrated by scanlines (midpoint across, 6-point Gauss along each
    /// kept interval), the integrand (x − c)·(x_u × x_v)/3, signed by the
    /// face's orientation; the overall sign makes the volume positive. Also
    /// returns each face's part and the count of samples off the sheet.
    pub fn flux_volume(&self, arrangement: &Arrangement, closure: &Closure, scanlines: usize) -> (f64, Vec<(SheetId, f64)>, usize) {
        let (total, mut parts, misses) = self.flux_raw(arrangement, closure, scanlines);
        let sign = if total < 0.0 { -1.0 } else { 1.0 };
        for p in &mut parts {
            p.1 *= sign;
        }
        (sign * total, parts, misses)
    }

    /// [`ConvexSweep::flux_raw`]'s total: the flux volume as oriented by the
    /// face signs alone.
    pub fn flux_volume_raw(&self, arrangement: &Arrangement, closure: &Closure, scanlines: usize) -> f64 {
        self.flux_raw(arrangement, closure, scanlines).0
    }

    /// The flux volume as oriented by the face signs alone (its sign says
    /// whether those signs point outward).
    fn flux_raw(&self, arrangement: &Arrangement, closure: &Closure, scanlines: usize) -> (f64, Vec<(SheetId, f64)>, usize) {
        const GAUSS: [(f64, f64); 6] = [
            (-0.932_469_514_203_152, 0.171_324_492_379_170),
            (-0.661_209_386_466_265, 0.360_761_573_048_139),
            (-0.238_619_186_083_197, 0.467_913_934_572_691),
            (0.238_619_186_083_197, 0.467_913_934_572_691),
            (0.661_209_386_466_265, 0.360_761_573_048_139),
            (0.932_469_514_203_152, 0.171_324_492_379_170),
        ];
        let (centre, _) = self.centre_and_reach();
        let c = self.grid.poses[self.grid.poses.len() / 2].apply(centre);
        let mut parts = Vec::new();
        let mut misses = 0;
        // Each loop's region, counted by its winding in the sheet's
        // parameters: a hole (clockwise) takes its region away from the face
        // round it, whichever face that is.
        let polygon_of = |l: usize| -> Vec<[f64; 2]> {
            let mut poly: Vec<[f64; 2]> = Vec::new();
            for &a in &arrangement.loops[l].arcs {
                let params = &arrangement.arcs[a].params;
                let skip = usize::from(!poly.is_empty());
                poly.extend(params.iter().skip(skip).copied());
            }
            poly
        };
        for (l, lp) in arrangement.loops.iter().enumerate() {
            if lp.arcs.is_empty() || closure.face_sign[l] == 0 {
                continue;
            }
            let sheet = lp.sheet;
            let poly = polygon_of(l);
            if poly.len() < 3 {
                continue;
            }
            let winding = if lp.area < 0.0 { -1.0 } else { 1.0 };
            let holes: Vec<Vec<[f64; 2]>> = Vec::new();
            let world = |p: [f64; 2]| self.point(sheet, [p[0].clamp(0.0, 1.0), p[1].clamp(0.0, 1.0)]).map(|q| q.world);
            let integrand = |p: [f64; 2]| -> Option<f64> {
                let h = 1e-6;
                let du = |q: [f64; 2]| -> Option<Vec3> {
                    let (a, b) = ((q[0] - h).max(0.0), (q[0] + h).min(1.0));
                    Some(world([b, q[1]])?.sub(world([a, q[1]])?).scale(1.0 / (b - a)))
                };
                let dv = |q: [f64; 2]| -> Option<Vec3> {
                    let (a, b) = ((q[1] - h).max(0.0), (q[1] + h).min(1.0));
                    Some(world([q[0], b])?.sub(world([q[0], a])?).scale(1.0 / (b - a)))
                };
                let x = world(p)?;
                Some(x.sub(c).dot(du(p)?.cross(dv(p)?)) / 3.0)
            };
            let (v0, v1) = poly.iter().chain(holes.iter().flatten()).fold((f64::INFINITY, f64::NEG_INFINITY), |a, q| (a.0.min(q[1]), a.1.max(q[1])));
            let dv = (v1 - v0) / scanlines as f64;
            // One scanline's terms and misses (under the `parallel` feature
            // the scanlines are read on the shared pool and their terms added
            // in order after; without it, one sum as before).
            let line = |k: usize, part: &mut dyn FnMut(f64), misses: &mut usize| {
                let v = v0 + (k as f64 + 0.5) * dv;
                let mut xs: Vec<f64> = Vec::new();
                for ring in std::iter::once(&poly).chain(holes.iter()) {
                    for i in 0..ring.len() {
                        let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                        if (a[1] <= v) != (b[1] <= v) {
                            xs.push(a[0] + (b[0] - a[0]) * (v - a[1]) / (b[1] - a[1]));
                        }
                    }
                }
                xs.sort_by(|a, b| a.total_cmp(b));
                // A ring's sheet turns through half the ring along u: each
                // kept interval in pieces (a polyhedron's sheets need one).
                let pieces = if sheet.is_ring() { 32 } else { 1 };
                for pair in xs.chunks_exact(2) {
                    let (a0, b0) = (pair[0], pair[1]);
                    for k in 0..pieces {
                        // The interval's own ends exactly (one piece: as it was).
                        let at = |j: usize| if j == 0 { a0 } else if j == pieces { b0 } else { a0 + (b0 - a0) * j as f64 / pieces as f64 };
                        let (ua, ub) = (at(k), at(k + 1));
                        for (g, w) in GAUSS {
                            let u = 0.5 * (ua + ub) + 0.5 * (ub - ua) * g;
                            match integrand([u, v]) {
                                Some(f) => part(f * w * 0.5 * (ub - ua) * dv),
                                None => *misses += 1,
                            }
                        }
                    }
                }
            };
            #[cfg(feature = "parallel")]
            let part = {
                use rayon::prelude::*;
                let forced = forced_switches();
                // Each scanline's terms, added after in the order one sum
                // takes them (the same bits).
                let lines: Vec<(Vec<f64>, usize)> = (0..scanlines).into_par_iter().map(|k| {
                    with_forced(&forced, || {
                        let (mut terms, mut m) = (Vec::new(), 0usize);
                        line(k, &mut |t| terms.push(t), &mut m);
                        (terms, m)
                    })
                }).collect();
                misses += lines.iter().map(|x| x.1).sum::<usize>();
                let mut part = 0.0;
                for t in lines.iter().flat_map(|x| &x.0) {
                    part += t;
                }
                part
            };
            #[cfg(not(feature = "parallel"))]
            let part = {
                let mut part = 0.0;
                for k in 0..scanlines {
                    line(k, &mut |t| part += t, &mut misses);
                }
                part
            };
            parts.push((sheet, winding * closure.face_sign[l] as f64 * part));
        }
        let total: f64 = parts.iter().map(|p| p.1).sum();
        (total, parts, misses)
    }

    /// A sheet's world point at parameters `p` with no domain test (a cap's
    /// plane past its polygon, a strip's line past its ends): for Newton
    /// steps that may leave the kept region by a little.
    pub fn raw(&self, sheet: SheetId, p: [f64; 2]) -> Option<Vec3> {
        if sheet.is_ring() {
            return self.ring_point_whole(sheet, p).map(|q| q.world);
        }
        let tool = self.convex();
        match sheet {
            SheetId::Envelope(..) | SheetId::Band(..) | SheetId::Collar(..) => unreachable!("a ring sheet in a polyhedron's arm"),
            SheetId::Strip(face) => {
                let t = self.t_of(p[1]);
                let pose = self.motion.pose(t);
                let [a, b] = face_strip(tool, self.motion, &pose, t, face)?;
                Some(pose.apply(a.add(b.sub(a).scale(p[0]))))
            }
            SheetId::Trace(edge) => {
                let t = self.t_of(p[1]);
                let [a, b, _, _] = tool.edges[edge];
                Some(self.motion.pose(t).apply(tool.vertices[a].add(tool.vertices[b].sub(tool.vertices[a]).scale(p[0]))))
            }
            SheetId::Cap(end, face) => {
                let own = if end == 0 { 0 } else { self.grid.times.len() - 1 };
                Some(self.grid.poses[own].apply(self.cap_at(face, p)))
            }
        }
    }

    /// A sheet's local sign at `p` (zero on a silhouette root or a cap's
    /// own strip), with no domain test.
    fn local_raw(&self, sheet: SheetId, p: [f64; 2]) -> Option<f64> {
        if sheet.is_ring() {
            return self.ring_local_raw(sheet, p);
        }
        match sheet {
            SheetId::Envelope(..) | SheetId::Band(..) | SheetId::Collar(..) => unreachable!("a ring sheet in a polyhedron's arm"),
            SheetId::Trace(edge) => {
                let t = self.t_of(p[1]);
                let pose = self.motion.pose(t);
                let [a, b, _, _] = self.convex().edges[edge];
                let x = self.convex().vertices[a].add(self.convex().vertices[b].sub(self.convex().vertices[a]).scale(p[0]));
                Some(edge_silhouette(self.convex(), self.motion, &pose, t, edge, x))
            }
            SheetId::Cap(end, face) => {
                let y = self.raw(sheet, p)?;
                let own = if end == 0 { 0 } else { self.grid.times.len() - 1 };
                let (t, pose) = (self.grid.times[own], self.grid.poses[own]);
                let v = self.motion.velocity(t, y);
                let side = if end == 0 { 1.0 } else { -1.0 };
                Some(-side * v.dot(pose.rotate(self.convex().normals[face])) / v.length().max(1e-300))
            }
            SheetId::Strip(_) => None,
        }
    }

    /// Cut in two each face whose loop passes one world vertex at two
    /// distinct points of its sheet's parameters (a double curve's end: the
    /// sheet meets itself there, and the double curve's edge bounds the face
    /// on both sides). The cut is a line of one parameter held between them,
    /// across the loop wherever it lies inside (the line as
    /// `self_meeting_cut` chooses it); its arcs border the sheet itself and
    /// mate each other. Each face then passes each such vertex once: a
    /// reader that places a loop on its surface by inverting its edges'
    /// points meets each preimage on a face of its own. The two corners are
    /// found by their points (within `tolerance` in the world, over 1e-6
    /// apart in the parameters), so the loops a cut leaves are read as they
    /// are. Returns the cuts; the arrangement is to be closed again.
    pub fn cut_self_meeting_loops(&self, arrangement: &mut Arrangement, tolerance: f64) -> usize {
        let mut cuts = 0;
        let mut l = 0;
        while l < arrangement.loops.len() && cuts < 64 {
            match self.self_meeting_cut(arrangement, l, tolerance) {
                Some((d, c)) if self.cut_loop(arrangement, l, d, c) => cuts += 1,
                _ => l += 1,
            }
        }
        cuts
    }

    /// The line (held coordinate, value) cutting outer loop `l` between two
    /// of its corners at one world point and distinct parameters (the first
    /// such pair a line serves), if one serves:
    /// of fifteen lines in each coordinate between the two corners, those that
    /// cross no hole of the face, pass no loop sample within 1e-7, cross the
    /// loop an even number of times with each chord meeting itself nowhere in
    /// the world (near a double curve's tip the sheet folds over the strip
    /// between its preimages), and keep every crossing twice `tolerance`
    /// from every vertex and every other crossing; the one keeping the most.
    fn self_meeting_cut(&self, arrangement: &Arrangement, l: usize, tolerance: f64) -> Option<(usize, f64)> {
        let lp = &arrangement.loops[l];
        let m = lp.arcs.len();
        let end = |a: usize| *arrangement.arcs[a].params.last().expect("params");
        let at = |a: usize| *arrangement.arcs[a].points.last().expect("points");
        let pairs: Vec<([f64; 2], [f64; 2])> = (0..m).flat_map(|i| {
            // (Not neighbours: an arc shorter than the tolerance is no meeting.)
            ((i + 2)..m).filter(move |&j| !(i == 0 && j == m - 1)).map(move |j| (i, j))
        }).filter(|&(i, j)| {
            let (p, q) = (end(lp.arcs[i]), end(lp.arcs[j]));
            at(lp.arcs[i]).sub(at(lp.arcs[j])).length() <= tolerance && ((p[0] - q[0]).abs() > 1e-6 || (p[1] - q[1]).abs() > 1e-6)
        }).map(|(i, j)| (end(lp.arcs[i]), end(lp.arcs[j]))).collect();
        // A face's outer loop only (a hole is not a polygon to cut).
        if pairs.is_empty() || !(lp.area > 0.0) {
            return None;
        }
        let owners = arrangement.hole_owners();
        let samples = |k: usize| arrangement.loops[k].arcs.iter().flat_map(|&a| arrangement.arcs[a].params.iter().copied()).collect::<Vec<_>>();
        let own = samples(l);
        // Every vertex so far: a crossing near one leaves a piece too short to
        // imprint on its mate.
        let ends: Vec<Vec3> = arrangement.arcs.iter().filter(|a| !a.points.is_empty()).flat_map(|a| [a.points[0], *a.points.last().expect("points")]).collect();
        // The first pair a line serves.
        for (p, q) in pairs {
            let mut best: Option<(f64, usize, f64)> = None;
            for d in [0usize, 1] {
                let (lo, hi) = (p[d].min(q[d]), p[d].max(q[d]));
                if hi - lo < 1e-6 {
                    continue;
                }
                for k in 1..16 {
                    let c = lo + (hi - lo) * k as f64 / 16.0;
                    if own.iter().any(|s| (s[d] - c).abs() < 1e-7) {
                        continue;
                    }
                    // A hole of the face across the line is not cut with it.
                    let crossed = (0..arrangement.loops.len()).any(|o| {
                        if owners[o] != Some(l) || arrangement.loops[o].arcs.is_empty() {
                            return false;
                        }
                        let s = samples(o);
                        s.iter().any(|x| x[d] < c) && s.iter().any(|x| x[d] > c)
                    });
                    if crossed {
                        continue;
                    }
                    // The crossings, in the world and along the line.
                    let mut hits: Vec<(f64, Vec3)> = Vec::new();
                    for &a in &lp.arcs {
                        let arc = &arrangement.arcs[a];
                        for (w, x) in arc.params.windows(2).zip(arc.points.windows(2)) {
                            let (va, vb) = (w[0][d] - c, w[1][d] - c);
                            if va * vb < 0.0 {
                                let f = va / (va - vb);
                                hits.push((w[0][1 - d] + (w[1][1 - d] - w[0][1 - d]) * f, x[0].add(x[1].sub(x[0]).scale(f))));
                            }
                        }
                    }
                    if hits.len() < 2 || hits.len() % 2 != 0 {
                        continue;
                    }
                    // Clearance: each crossing from every vertex and every other
                    // crossing.
                    let mut clear = f64::INFINITY;
                    for (i, h) in hits.iter().enumerate() {
                        clear = ends.iter().fold(clear, |m, e| m.min(e.sub(h.1).length()));
                        clear = hits[i + 1..].iter().fold(clear, |m, g| m.min(g.1.sub(h.1).length()));
                    }
                    // Each chord meets itself nowhere in the world: near a double
                    // curve's tip the sheet folds over the strip between its two
                    // preimages, and a chord across it would.
                    hits.sort_by(|x, y| x.0.total_cmp(&y.0));
                    let folds = hits.chunks(2).any(|pair| {
                        let world: Vec<Option<Vec3>> = (0..=16).map(|i| {
                            let mut r = [0.0; 2];
                            r[d] = c;
                            r[1 - d] = pair[0].0 + (pair[1].0 - pair[0].0) * i as f64 / 16.0;
                            self.raw(lp.sheet, r)
                        }).collect();
                        if world.iter().any(Option::is_none) {
                            return true;
                        }
                        let world: Vec<Vec3> = world.into_iter().flatten().collect();
                        let along: Vec<f64> = std::iter::once(0.0).chain(world.windows(2).scan(0.0, |t, w| { *t += w[1].sub(w[0]).length(); Some(*t) })).collect();
                        (0..world.len()).any(|i| ((i + 2)..world.len()).any(|j| world[j].sub(world[i]).length() < 0.5 * (along[j] - along[i])))
                    });
                    if folds || clear < 2.0 * tolerance {
                        continue;
                    }
                    if best.map_or(true, |b| clear > b.0) {
                        best = Some((clear, d, c));
                    }
                }
            }
            if let Some(b) = best {
                return Some((b.1, b.2));
            }
        }
        None
    }

    /// Cut loop `l` along the line where coordinate `d` is `c`: the arcs the
    /// line crosses split there, a chord between each pair of crossings in
    /// order along the line, and the loop's runs between crossings joined by
    /// the chords into loops. False (the loop unchanged) unless the line
    /// crosses it an even number of times, at least twice.
    fn cut_loop(&self, arrangement: &mut Arrangement, l: usize, d: usize, c: f64) -> bool {
        let sheet = arrangement.loops[l].sheet;
        let e = 1 - d;
        // Split every crossed arc at its crossings; `seq` the loop's arcs in
        // order, `cross_after[s]` the crossing at the end of seq[s].
        let mut cross_after: Vec<Option<usize>> = Vec::new();
        let mut crossings: Vec<f64> = Vec::new();
        let mut crossing_point: Vec<([f64; 2], Vec3)> = Vec::new();
        let original = arrangement.loops[l].arcs.clone();
        let mut pieces: Vec<ArcRecord> = Vec::new();
        for &id in &original {
            let arc = &arrangement.arcs[id];
            let mut points = vec![arc.points[0]];
            let mut params = vec![arc.params[0]];
            for k in 0..arc.params.len() - 1 {
                let (a, b) = (arc.params[k], arc.params[k + 1]);
                let (va, vb) = (a[d] - c, b[d] - c);
                if va * vb < 0.0 {
                    let f = va / (va - vb);
                    let mut p = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f];
                    p[d] = c;
                    let world = self.raw(sheet, p).unwrap_or_else(|| arc.points[k].add(arc.points[k + 1].sub(arc.points[k]).scale(f)));
                    points.push(world);
                    params.push(p);
                    pieces.push(ArcRecord { sheet, border: arc.border, points: std::mem::take(&mut points), params: std::mem::take(&mut params) });
                    cross_after.push(Some(crossings.len()));
                    crossings.push(p[e]);
                    crossing_point.push((p, world));
                    points = vec![world];
                    params = vec![p];
                }
                points.push(arc.points[k + 1]);
                params.push(arc.params[k + 1]);
            }
            pieces.push(ArcRecord { sheet, border: arc.border, points, params });
            cross_after.push(None);
        }
        let n = crossings.len();
        if n < 2 || n % 2 != 0 {
            return false;
        }
        // Pieces into the arrangement: the first piece of each original arc
        // keeps its id, the rest are new.
        let mut piece_id = vec![usize::MAX; pieces.len()];
        let mut next_original = original.iter();
        let mut first_of_arc = true;
        for (s, piece) in pieces.into_iter().enumerate() {
            if first_of_arc {
                let id = *next_original.next().expect("an original arc per run");
                arrangement.arcs[id] = piece;
                piece_id[s] = id;
            } else {
                piece_id[s] = arrangement.arcs.len();
                arrangement.arcs.push(piece);
            }
            first_of_arc = cross_after[s].is_none();
        }
        let seq = piece_id;
        // Chords: crossings paired in order along the line.
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| crossings[a].total_cmp(&crossings[b]));
        let mut partner = vec![0usize; n];
        for pair in order.chunks(2) {
            partner[pair[0]] = pair[1];
            partner[pair[1]] = pair[0];
        }
        // Position in seq of each crossing (it ends seq[pos]).
        let mut at = vec![0usize; n];
        for (s, x) in cross_after.iter().enumerate() {
            if let Some(q) = x {
                at[*q] = s;
            }
        }
        let len = seq.len();
        let chord = |arrangement: &mut Arrangement, from: usize, to: usize| -> usize {
            let (a, b) = (crossing_point[from], crossing_point[to]);
            const STATIONS: usize = 33;
            let mut points = Vec::with_capacity(STATIONS);
            let mut params = Vec::with_capacity(STATIONS);
            for i in 0..STATIONS {
                let f = i as f64 / (STATIONS - 1) as f64;
                let mut p = [a.0[0] + (b.0[0] - a.0[0]) * f, a.0[1] + (b.0[1] - a.0[1]) * f];
                p[d] = c;
                let world = if i == 0 { a.1 } else if i == STATIONS - 1 { b.1 } else { self.raw(sheet, p).unwrap_or_else(|| a.1.add(b.1.sub(a.1).scale(f))) };
                points.push(world);
                params.push(p);
            }
            arrangement.arcs.push(ArcRecord { sheet, border: Border::Sheet(sheet), points, params });
            arrangement.arcs.len() - 1
        };
        let mut used = vec![false; n];
        let mut new_loops: Vec<Vec<usize>> = Vec::new();
        for q0 in 0..n {
            if used[q0] {
                continue;
            }
            let mut ids = Vec::new();
            let mut q = q0;
            loop {
                used[q] = true;
                // The run after crossing q, to the next crossing.
                let mut s = (at[q] + 1) % len;
                loop {
                    ids.push(seq[s]);
                    if cross_after[s].is_some() {
                        break;
                    }
                    s = (s + 1) % len;
                }
                let reached = cross_after[s].expect("a crossing ends the run");
                let other = partner[reached];
                ids.push(chord(arrangement, reached, other));
                q = other;
                if q == q0 {
                    break;
                }
                if used[q] {
                    return false;
                }
            }
            new_loops.push(ids);
        }
        let closed = arrangement.loops[l].closed;
        let area = |arrangement: &Arrangement, ids: &[usize]| -> f64 {
            ids.iter().flat_map(|&a| arrangement.arcs[a].params.windows(2).map(|w| 0.5 * (w[0][0] * w[1][1] - w[1][0] * w[0][1])).collect::<Vec<_>>()).sum()
        };
        for (k, ids) in new_loops.into_iter().enumerate() {
            let record = LoopRecordT { sheet, area: area(arrangement, &ids), arcs: ids, closed };
            if k == 0 {
                arrangement.loops[l] = record;
            } else {
                arrangement.loops.push(record);
            }
        }
        true
    }

    /// Refine every arc of a closed arrangement onto the exact sheets: each
    /// sample is moved (Gauss–Newton, minimum-norm steps on the parameters
    /// of both sheets, a domain side's coordinate held) until its points on
    /// the arc's sheet and on the partner's agree; an arc along its sheet's
    /// own silhouette root also keeps the local sign zero. Returns, per arc,
    /// the refined samples (world point, own and partner parameters) and the
    /// worst remaining gap and parameter shift. Vertex-track arcs (a domain
    /// side, exact) are kept as they are.
    pub fn refine_arcs(&self, arrangement: &Arrangement, closure: &Closure) -> Vec<RefinedArc> {
        let mut out = Vec::new();
        for (k, arc) in arrangement.arcs.iter().enumerate() {
            if arc.points.is_empty() {
                out.push(RefinedArc::default());
                continue;
            }
            let side_of = |params: &[[f64; 2]]| -> Option<(usize, f64)> {
                (0..2).find_map(|c| {
                    let v = params[0][c];
                    // A domain side; under the tip switch also a line cut
                    // across a face (one coordinate held exactly).
                    let on = (v == 0.0 || v == 1.0 || tip_splice_on()) && params.len() > 1 && params.iter().all(|q| q[c] == v);
                    on.then_some((c, v))
                })
            };
            let own_side = side_of(&arc.params);
            let partner = match arc.border {
                Border::Sheet(b) => b,
                _ => {
                    let samples = arc.points.iter().zip(&arc.params).map(|(x, q)| (*x, *q, *q)).collect();
                    out.push(RefinedArc { samples, gap: 0.0, shift: 0.0, partner: None });
                    continue;
                }
            };
            let Some((mate, _)) = closure.mating.mate[k] else {
                out.push(RefinedArc { samples: Vec::new(), gap: f64::INFINITY, shift: 0.0, partner: Some(partner) });
                continue;
            };
            let mate_arc = &arrangement.arcs[mate];
            let mate_side = side_of(&mate_arc.params);
            // The partner's parameters nearest a world point, along the mate.
            let start_partner = |x: Vec3| -> [f64; 2] {
                let mut best = (f64::INFINITY, mate_arc.params[0]);
                for w in 0..mate_arc.points.len().saturating_sub(1) {
                    let (a, b) = (mate_arc.points[w], mate_arc.points[w + 1]);
                    let d = b.sub(a);
                    let s = if d.dot(d) > 0.0 { (x.sub(a).dot(d) / d.dot(d)).clamp(0.0, 1.0) } else { 0.0 };
                    let gap = x.sub(a.add(d.scale(s))).length();
                    if gap < best.0 {
                        let (pa, pb) = (mate_arc.params[w], mate_arc.params[w + 1]);
                        best = (gap, [pa[0] + (pb[0] - pa[0]) * s, pa[1] + (pb[1] - pa[1]) * s]);
                    }
                }
                best.1
            };
            let local_arc = self.local_raw(arc.sheet, arc.params[arc.params.len() / 2]).is_some_and(|l| l.abs() < 1e-6);
            let mut samples = Vec::new();
            let (mut gap, mut shift) = (0.0f64, 0.0f64);
            for (x0, p0) in arc.points.iter().zip(&arc.params) {
                let q0 = start_partner(*x0);
                match self.refine_point(arc.sheet, partner, *p0, q0, own_side.map(|s| s.0), mate_side.map(|s| s.0), local_arc) {
                    Some((xa, p, q, g)) => {
                        gap = gap.max(g);
                        shift = shift.max(xa.sub(*x0).length());
                        samples.push((xa, p, q));
                    }
                    None => gap = f64::INFINITY,
                }
            }
            out.push(RefinedArc { samples, gap, shift, partner: Some(partner) });
        }
        out
    }

    /// One point refined onto two sheets: own parameters `p0` on `sheet`,
    /// partner parameters `q0` on `partner`; `own_side` / `mate_side` name a
    /// coordinate held (a domain side); `local_arc` also keeps the own
    /// sheet's local sign zero. Returns the point, both parameters and the
    /// remaining gap.
    fn refine_point(&self, sheet: SheetId, partner: SheetId, p0: [f64; 2], q0: [f64; 2], own_side: Option<usize>, mate_side: Option<usize>, local_arc: bool) -> Option<(Vec3, [f64; 2], [f64; 2], f64)> {
        let arc_sheet = sheet;
        {
            {
                let (mut p, mut q) = (p0, q0);
                // Free coordinates: (own 0, own 1, partner 0, partner 1).
                let free: Vec<usize> = (0..4).filter(|&i| {
                    if i < 2 { own_side.map_or(true, |s| s != i) } else { mate_side.map_or(true, |s| s != i - 2) }
                }).collect();
                let mut ok = true;
                for _ in 0..30 {
                    let (Some(xa), Some(xb)) = (self.raw(arc_sheet, p), self.raw(partner, q)) else {
                        ok = false;
                        break;
                    };
                    let mut f = vec![xa.x - xb.x, xa.y - xb.y, xa.z - xb.z];
                    let local = if local_arc { self.local_raw(arc_sheet, p) } else { None };
                    if let Some(l) = local {
                        f.push(l * self.size);
                    }
                    if f.iter().map(|v| v * v).sum::<f64>().sqrt() < 1e-13 * self.size {
                        break;
                    }
                    let h = 1e-7;
                    let mut jac = vec![vec![0.0; free.len()]; f.len()];
                    for (c, &i) in free.iter().enumerate() {
                        let (mut pp, mut qq, mut pm, mut qm) = (p, q, p, q);
                        if i < 2 {
                            pp[i] += h;
                            pm[i] -= h;
                        } else {
                            qq[i - 2] += h;
                            qm[i - 2] -= h;
                        }
                        let (Some(ap), Some(am), Some(bp), Some(bm)) = (self.raw(arc_sheet, pp), self.raw(arc_sheet, pm), self.raw(partner, qq), self.raw(partner, qm)) else {
                            ok = false;
                            break;
                        };
                        let d = ap.sub(bp).sub(am.sub(bm)).scale(0.5 / h);
                        jac[0][c] = d.x;
                        jac[1][c] = d.y;
                        jac[2][c] = d.z;
                        if local.is_some() {
                            let (lp, lm) = (self.local_raw(arc_sheet, pp).unwrap_or(0.0), self.local_raw(arc_sheet, pm).unwrap_or(0.0));
                            jac[3][c] = (lp - lm) * 0.5 / h * self.size;
                        }
                    }
                    if !ok {
                        break;
                    }
                    let Some(mut step) = least_squares_step(&jac, &f) else {
                        ok = false;
                        break;
                    };
                    let biggest = step.iter().fold(0.0f64, |a, v| a.max(v.abs()));
                    if biggest > STEP_CAP {
                        step.iter_mut().for_each(|v| *v *= STEP_CAP / biggest);
                    }
                    let gap_of = |p: [f64; 2], q: [f64; 2]| -> Option<f64> {
                        let d = self.raw(arc_sheet, p)?.sub(self.raw(partner, q)?).length();
                        let l = if local.is_some() { self.local_raw(arc_sheet, p)? * self.size } else { 0.0 };
                        Some(d.hypot(l))
                    };
                    let before = f.iter().map(|v| v * v).sum::<f64>().sqrt();
                    let mut accepted = false;
                    for _ in 0..30 {
                        let (mut pt, mut qt) = (p, q);
                        for (c, &i) in free.iter().enumerate() {
                            if i < 2 {
                                pt[i] += step[c];
                            } else {
                                qt[i - 2] += step[c];
                            }
                        }
                        if gap_of(pt, qt).is_some_and(|g| g < before) {
                            (p, q) = (pt, qt);
                            accepted = true;
                            break;
                        }
                        step.iter_mut().for_each(|v| *v *= 0.5);
                    }
                    if !accepted {
                        break;
                    }
                }
                let (xa, xb) = (self.raw(arc_sheet, p)?, self.raw(partner, q)?);
                let gap = if ok { xa.sub(xb).length() } else { xa.sub(xb).length().max(1e-3 * self.size) };
                Some((xa, p, q, gap))
            }
        }
    }

    /// Solve each vertex of a closed arrangement exactly: its corners (one
    /// per loop passing it) must name one world point on every corner's
    /// sheet. A corner on a domain side holds that coordinate (both at a
    /// domain corner); a corner on a silhouette-root arc keeps the local
    /// sign zero. Starts from the refined arcs' end samples. Returns per
    /// vertex class: the point, the corners' spread, the move.
    pub fn solve_vertices(&self, arrangement: &Arrangement, closure: &Closure, refined: &[RefinedArc]) -> Vec<SolvedVertex> {
        let mut corners: std::collections::BTreeMap<usize, Vec<(SheetId, [f64; 2], [bool; 2], bool)>> = Default::default();
        let side_of = |params: &[[f64; 2]], c: usize| -> bool {
            let v = params[0][c];
            (v == 0.0 || v == 1.0 || tip_splice_on()) && params.len() > 1 && params.iter().all(|q| q[c] == v)
        };
        for lp in &arrangement.loops {
            let m = lp.arcs.len();
            for i in 0..m {
                let (a, b) = (lp.arcs[i], lp.arcs[(i + 1) % m]);
                let class = closure.mating.end_class[2 * a + 1];
                let (pa, pb) = (&arrangement.arcs[a].params, &arrangement.arcs[b].params);
                let start = refined[b].samples.first().map(|s| s.1).unwrap_or(pb[0]);
                let fixed = [side_of(pa, 0) || side_of(pb, 0), side_of(pa, 1) || side_of(pb, 1)];
                // Hold the side value exactly.
                let mut p = start;
                for c in 0..2 {
                    if side_of(pa, c) {
                        p[c] = pa[0][c];
                    } else if side_of(pb, c) {
                        p[c] = pb[0][c];
                    }
                }
                let local = [a, b].iter().any(|&k| {
                    let arc = &arrangement.arcs[k];
                    matches!(arc.border, Border::Sheet(_)) && self.local_raw(arc.sheet, arc.params[arc.params.len() / 2]).is_some_and(|l| l.abs() < 1e-6)
                });
                corners.entry(class).or_default().push((lp.sheet, p, fixed, local));
            }
        }
        let mut out = Vec::new();
        for (class, list) in corners {
            let mut params: Vec<[f64; 2]> = list.iter().map(|c| c.1).collect();
            let start = list.iter().filter_map(|c| self.raw(c.0, c.1)).fold(Vec3::new(0.0, 0.0, 0.0), |a, x| a.add(x)).scale(1.0 / list.len() as f64);
            let free: Vec<(usize, usize)> = list.iter().enumerate().flat_map(|(i, c)| (0..2).filter(move |&k| !c.2[k]).map(move |k| (i, k))).collect();
            let residual = |params: &[[f64; 2]]| -> Option<Vec<f64>> {
                let x0 = self.raw(list[0].0, params[0])?;
                let mut f = Vec::new();
                for (i, c) in list.iter().enumerate().skip(1) {
                    let x = self.raw(c.0, params[i])?;
                    f.extend([x.x - x0.x, x.y - x0.y, x.z - x0.z]);
                }
                for (i, c) in list.iter().enumerate() {
                    if c.3 {
                        if let Some(l) = self.local_raw(c.0, params[i]) {
                            f.push(l * self.size);
                        }
                    }
                }
                Some(f)
            };
            let mut ok = true;
            for _ in 0..40 {
                let Some(f) = residual(&params) else {
                    ok = false;
                    break;
                };
                if f.iter().map(|v| v * v).sum::<f64>().sqrt() < 1e-13 * self.size {
                    break;
                }
                let h = 1e-7;
                let mut jac = vec![vec![0.0; free.len()]; f.len()];
                for (c, &(i, k)) in free.iter().enumerate() {
                    let (mut pp, mut pm) = (params.clone(), params.clone());
                    pp[i][k] += h;
                    pm[i][k] -= h;
                    // One-sided where a sheet ends within the step (a
                    // strip's zero line leaves its face at the vertex).
                    let (fp, fm) = (residual(&pp), residual(&pm));
                    let one_sided = one_sided_on();
                    let (upper, lower, width) = match (&fp, &fm) {
                        (Some(fp), Some(fm)) => (fp, fm, 2.0 * h),
                        (Some(fp), None) if one_sided => (fp, &f, h),
                        (None, Some(fm)) if one_sided => (&f, fm, h),
                        _ => {
                            ok = false;
                            break;
                        }
                    };
                    for r in 0..f.len() {
                        jac[r][c] = (upper[r] - lower[r]) / width;
                    }
                }
                if !ok {
                    break;
                }
                let Some(mut step) = least_squares_step(&jac, &f) else {
                    ok = false;
                    break;
                };
                // A local solve: steps capped and halved until the residual
                // falls (a degenerate vertex, such as an edge pivoting on a
                // point, has a whole curve of parameters and must not slide
                // to another vertex on it).
                let norm = |f: &[f64]| f.iter().map(|v| v * v).sum::<f64>().sqrt();
                let biggest = step.iter().fold(0.0f64, |a, v| a.max(v.abs()));
                if biggest > STEP_CAP {
                    step.iter_mut().for_each(|v| *v *= STEP_CAP / biggest);
                }
                let before = norm(&f);
                let mut accepted = false;
                for _ in 0..30 {
                    let mut trial = params.clone();
                    for (c, &(i, k)) in free.iter().enumerate() {
                        trial[i][k] += step[c];
                    }
                    if residual(&trial).is_some_and(|g| norm(&g) < before) {
                        params = trial;
                        accepted = true;
                        break;
                    }
                    step.iter_mut().for_each(|v| *v *= 0.5);
                }
                if !accepted {
                    break;
                }
            }
            let points: Vec<Vec3> = list.iter().zip(&params).filter_map(|(c, p)| self.raw(c.0, *p)).collect();
            let point = points.first().copied().unwrap_or(start);
            let spread = if ok && points.len() == list.len() { points.iter().map(|x| x.sub(point).length()).fold(0.0, f64::max) } else { f64::INFINITY };
            out.push(SolvedVertex { class, point, spread, moved: point.sub(start).length(), corners: list.iter().zip(&params).map(|(c, p)| (c.0, *p)).collect() });
        }
        out
    }

    /// A face's surface over the time fractions [lo, hi] of its loop, and
    /// the map from the sheet's parameters to the surface's.
    /// - a trace: ruled between its two vertex tracks, (μ, τ) as they are;
    /// - a strip: ruled across the face plane's whole zero line of normal
    ///   speed (from the point nearest the face centre, ± the face's reach),
    ///   so it stays smooth where the strip's clipped ends jump edges;
    /// - a cap: the end pose's face plane, exact.
    /// The time direction is interpolated (quintic) through exact rows,
    /// doubling the rows until the midpoints miss by under 1e-10 of the
    /// tool's size.
    fn sheet_surface(&self, sheet: SheetId, lo: f64, hi: f64) -> Result<SheetSurface, KernelRefusal> {
        if sheet.is_ring() {
            return self.ring_sheet_surface(sheet, lo, hi);
        }
        let tool = self.convex();
        match sheet {
            SheetId::Envelope(..) | SheetId::Band(..) | SheetId::Collar(..) => unreachable!("a ring sheet in a polyhedron's arm"),
            SheetId::Cap(end, face) => {
                let own = if end == 0 { 0 } else { self.grid.times.len() - 1 };
                let pose = self.grid.poses[own];
                // The face's plane in its own frame, over the polygon's box
                // (padded): planar, exact, and regular everywhere. (The
                // arrangement's bilinear chart on the corners collapses a
                // triangle's last side to a vertex, where an edge's pcurve
                // leaving it sweeps across the face.)
                let frame = self.frame(face);
                let (mut lo, mut hi) = ((f64::INFINITY, f64::INFINITY), (f64::NEG_INFINITY, f64::NEG_INFINITY));
                for &(a, b) in &frame.coords {
                    lo = (lo.0.min(a), lo.1.min(b));
                    hi = (hi.0.max(a), hi.1.max(b));
                }
                let pad = 0.01 * (hi.0 - lo.0).max(hi.1 - lo.1);
                let (lo, size) = ((lo.0 - pad, lo.1 - pad), (hi.0 - lo.0 + 2.0 * pad, hi.1 - lo.1 + 2.0 * pad));
                let at = |a: f64, b: f64| Vec4::from_point(pose.apply(frame.origin.add(frame.u.scale(lo.0 + size.0 * a)).add(frame.w.scale(lo.1 + size.1 * b))), 1.0);
                let surface = frag(NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], vec![vec![at(0.0, 0.0), at(0.0, 1.0)], vec![at(1.0, 0.0), at(1.0, 1.0)]]), "cap_surface")?;
                Ok(SheetSurface { surface, strip: None, band: None, cap: Some(CapChart { face, origin: frame.origin, u: frame.u, w: frame.w, lo, size }) })
            }
            SheetId::Trace(edge) => {
                let [a, b, _, _] = tool.edges[edge];
                let rows = |tau: f64| -> [Vec3; 2] {
                    let pose = self.motion.pose(self.t_of(tau));
                    [pose.apply(tool.vertices[a]), pose.apply(tool.vertices[b])]
                };
                let surface = self.ruled_in_time(lo, hi, &rows)?;
                Ok(SheetSurface { surface, strip: None, band: None, cap: None })
            }
            SheetId::Strip(face) => {
                let frame = self.frame(face);
                let centre = frame.coords.iter().fold((0.0, 0.0), |a, c| (a.0 + c.0, a.1 + c.1));
                let centre = (centre.0 / frame.coords.len() as f64, centre.1 / frame.coords.len() as f64);
                let reach = 1.05 * frame.coords.iter().map(|c| (c.0 - centre.0).hypot(c.1 - centre.1)).fold(0.0, f64::max);
                // The zero line at τ: its point nearest the centre and its
                // direction (sign kept continuous from the previous row).
                let line = |tau: f64, previous: Option<(f64, f64)>| -> Option<(Vec3, Vec3, (f64, f64))> {
                    let t = self.t_of(tau);
                    let pose = self.motion.pose(t);
                    let n = pose.rotate(tool.normals[face]);
                    let world = |c: (f64, f64)| pose.apply(frame.origin.add(frame.u.scale(c.0)).add(frame.w.scale(c.1)));
                    let speed = |c: (f64, f64)| self.motion.velocity(t, world(c)).dot(n);
                    let g0 = speed(centre);
                    let (gu, gw) = (speed((centre.0 + 1.0, centre.1)) - g0, speed((centre.0, centre.1 + 1.0)) - g0);
                    let gg = gu * gu + gw * gw;
                    // The gradient is a rate (per unit length): compare it with
                    // the fastest turn's, so a slower or faster drive of the
                    // same sweep reads the same.
                    let rate = self.motion.rates.iter().fold(0.0f64, |a, r| a.max(r.abs()));
                    if gg <= (1e-12 * rate).powi(2) {
                        return None;
                    }
                    let foot = (centre.0 - g0 * gu / gg, centre.1 - g0 * gw / gg);
                    let mut d = (-gw / gg.sqrt(), gu / gg.sqrt());
                    if let Some(p) = previous {
                        if p.0 * d.0 + p.1 * d.1 < 0.0 {
                            d = (-d.0, -d.1);
                        }
                    }
                    let ends = (world((foot.0 - reach * d.0, foot.1 - reach * d.1)), world((foot.0 + reach * d.0, foot.1 + reach * d.1)));
                    Some((ends.0, ends.1, d))
                };
                // Directions: continuous over the fitted span.
                let reference = line(0.5 * (lo + hi), None).ok_or_else(|| refuse("trim_strip_line", format!("face {face}'s normal speed is constant on its plane")))?.2;
                let rows = |tau: f64| -> [Vec3; 2] {
                    match line(tau, Some(reference)) {
                        Some((a, b, _)) => [a, b],
                        None => [Vec3::new(f64::NAN, 0.0, 0.0); 2],
                    }
                };
                let surface = self.ruled_in_time(lo, hi, &rows)?;
                Ok(SheetSurface { surface, strip: Some(StripChart { face, centre, reach, reference }), band: None, cap: None })
            }
        }
    }

    /// A surface ruled between two curves of time (u: 0 → 1 between them,
    /// v = τ over [lo, hi]), interpolated through exact rows.
    fn ruled_in_time(&self, lo: f64, hi: f64, rows: &dyn Fn(f64) -> [Vec3; 2]) -> Result<NurbsSurface, KernelRefusal> {
        let mut count = 16usize;
        loop {
            let taus: Vec<f64> = (0..=count).map(|i| lo + (hi - lo) * i as f64 / count as f64).collect();
            let samples: Vec<[Vec3; 2]> = taus.iter().map(|&t| rows(t)).collect();
            if samples.iter().any(|r| !(r[0].x.is_finite() && r[1].x.is_finite())) {
                return Err(refuse("trim_sheet_rows", "a sheet's rows are undefined in its span".into()));
            }
            let a: Vec<Vec3> = samples.iter().map(|r| r[0]).collect();
            let b: Vec<Vec3> = samples.iter().map(|r| r[1]).collect();
            let ca = frag(crate::interpolate_curve(&a, 5, &taus), "trim_rail")?;
            let cb = frag(crate::interpolate_curve(&b, 5, &taus), "trim_rail")?;
            let mut worst = 0.0f64;
            for i in 0..count {
                let t = 0.5 * (taus[i] + taus[i + 1]);
                let exact = rows(t);
                for (curve, x) in [(&ca, exact[0]), (&cb, exact[1])] {
                    let y = frag(curve.evaluate(t), "trim_rail")?;
                    worst = worst.max(y.sub(x).length());
                }
            }
            if worst <= 1e-10 * self.size || count >= 4096 {
                if worst > 1e-7 * self.size {
                    return Err(refuse("trim_sheet_fit", format!("a sheet's rails miss by {worst:.2e}")));
                }
                let net = vec![ca.control_points.clone(), cb.control_points.clone()];
                return frag(NurbsSurface::new(1, 5, vec![0.0, 0.0, 1.0, 1.0], ca.knots.clone(), net), "trim_sheet_surface");
            }
            count *= 2;
        }
    }

    /// A sheet point's parameters on its face's surface.
    fn surface_uv(&self, sheet: SheetId, chart: &SheetSurface, p: [f64; 2]) -> [f64; 2] {
        if let (Some(band), SheetId::Band(end, half)) = (&chart.band, sheet) {
            return self.band_uv(band, end, half, p);
        }
        if let (Some(cap), SheetId::Cap(_, face)) = (&chart.cap, sheet) {
            let x = self.cap_at(face, p).sub(cap.origin);
            return [(x.dot(cap.u) - cap.lo.0) / cap.size.0, (x.dot(cap.w) - cap.lo.1) / cap.size.1];
        }
        match (&chart.strip, sheet) {
            (Some(c), SheetId::Strip(face)) => {
                let t = self.t_of(p[1]);
                let pose = self.motion.pose(t);
                let frame = self.frame(face);
                let Some([a, b]) = face_strip(self.convex(), self.motion, &pose, t, face) else { return p };
                let x = a.add(b.sub(a).scale(p[0]));
                let q = (x.sub(frame.origin).dot(frame.u), x.sub(frame.origin).dot(frame.w));
                // Distance along the line from its foot, over twice the reach.
                let n = pose.rotate(self.convex().normals[face]);
                let world = |c: (f64, f64)| pose.apply(frame.origin.add(frame.u.scale(c.0)).add(frame.w.scale(c.1)));
                let speed = |c: (f64, f64)| self.motion.velocity(t, world(c)).dot(n);
                let g0 = speed(c.centre);
                let (gu, gw) = (speed((c.centre.0 + 1.0, c.centre.1)) - g0, speed((c.centre.0, c.centre.1 + 1.0)) - g0);
                let gg = gu * gu + gw * gw;
                let foot = (c.centre.0 - g0 * gu / gg, c.centre.1 - g0 * gw / gg);
                let mut d = (-gw / gg.sqrt(), gu / gg.sqrt());
                if c.reference.0 * d.0 + c.reference.1 * d.1 < 0.0 {
                    d = (-d.0, -d.1);
                }
                let along = (q.0 - foot.0) * d.0 + (q.1 - foot.1) * d.1;
                [(along + c.reach) / (2.0 * c.reach), p[1]]
            }
            _ => p,
        }
    }

    /// Build the B-rep of a closed, refined arrangement: one face per loop
    /// (its sheet's surface), one edge per mated pair of arcs (resampled
    /// along the arc, each station refined onto both sheets, the curve and
    /// both pcurves interpolated with the same parameters), one vertex per
    /// vertex class (its solved point). Faces are oriented outward by their
    /// orientation signs and the enclosed volume's sign.
    pub fn build(&self, arrangement: &Arrangement, closure: &Closure, refined: &[RefinedArc], vertices: &[SolvedVertex]) -> Result<BrepSolid, KernelRefusal> {
        self.build_with_sign(arrangement, closure, refined, vertices, None)
    }

    /// [`ConvexSweep::build`] with the face signs' overall orientation given
    /// (the sign of a flux reading the caller has taken, as `flux_raw` gives
    /// it), or read here at 200 scanlines when `None`.
    pub fn build_with_sign(&self, arrangement: &Arrangement, closure: &Closure, refined: &[RefinedArc], vertices: &[SolvedVertex], sign: Option<f64>) -> Result<BrepSolid, KernelRefusal> {
        // A polyhedron's sweep is a sphere; a ring's may be of any genus and
        // meet itself at its double curves' tips.
        let closed = match self.tool {
            Tool::Convex(_) => closure.is_sphere(),
            Tool::Ring(_) => closure.is_closed_surface(),
        };
        if !closed {
            return Err(refuse("trim_arrangement", "the trimmed sheets do not close into one oriented shell".into()));
        }
        let sign = match sign {
            Some(sign) => sign.signum(),
            None => self.flux_raw(arrangement, closure, 200).0.signum(),
        };
        let n = arrangement.arcs.len();
        let mut next = 1u64;
        let mut id = || {
            next += 1;
            next
        };
        // Vertices: one per vertex class; for a ring, one per cycle of its
        // link (where the surface meets itself at a point, each sheet through
        // it has its own vertex there).
        let ring = matches!(self.tool, Tool::Ring(_));
        let vertex_key = |e: usize| -> (usize, usize) { (closure.mating.end_class[e], if ring { closure.link_class[e] } else { 0 }) };
        let mut vertex_records = Vec::new();
        let mut vertex_id: std::collections::BTreeMap<(usize, usize), (u64, Vec3)> = Default::default();
        for v in vertices {
            if !(v.spread <= 1e-8 * self.size) {
                return Err(refuse("trim_vertex", format!("a vertex's sheets meet only within {:.2e}", v.spread)));
            }
            let mut keys: std::collections::BTreeSet<(usize, usize)> = (0..2 * n).filter(|&e| closure.mating.end_class[e] == v.class).map(vertex_key).collect();
            if keys.is_empty() {
                keys.insert((v.class, 0));
            }
            for key in keys {
                let k = id();
                vertex_records.push(VertexRecord { id: k, point: v.point });
                vertex_id.insert(key, (k, v.point));
            }
        }
        let corner_param = |class: usize, sheet: SheetId, near: [f64; 2]| -> [f64; 2] {
            let v = vertices.iter().find(|v| v.class == class);
            v.and_then(|v| v.corners.iter().filter(|c| c.0 == sheet).min_by(|a, b| {
                let da = (a.1[0] - near[0]).hypot(a.1[1] - near[1]);
                let db = (b.1[0] - near[0]).hypot(b.1[1] - near[1]);
                da.total_cmp(&db)
            }).map(|c| c.1)).unwrap_or(near)
        };
        // Faces' surfaces, over the time span of each outer loop and its
        // holes (a hole's arcs take their face's chart).
        let owners = arrangement.hole_owners();
        let face_of = |l: usize| owners[l].unwrap_or(l);
        let mut charts: Vec<Option<SheetSurface>> = vec![None; arrangement.loops.len()];
        let mut loop_of = vec![usize::MAX; n];
        for (l, lp) in arrangement.loops.iter().enumerate() {
            for &a in &lp.arcs {
                loop_of[a] = face_of(l);
            }
        }
        for (l, lp) in arrangement.loops.iter().enumerate() {
            if lp.arcs.is_empty() || owners[l].is_some() {
                continue;
            }
            let members: Vec<usize> = (0..arrangement.loops.len()).filter(|&m| face_of(m) == l).collect();
            let (lo, hi) = members.iter().flat_map(|&m| arrangement.loops[m].arcs.iter()).flat_map(|&a| arrangement.arcs[a].params.iter().map(|p| p[1])).fold((f64::INFINITY, f64::NEG_INFINITY), |acc, t| (acc.0.min(t), acc.1.max(t)));
            let margin = 0.02 * (hi - lo).max(1e-3);
            // A ring's envelope runs on past the motion's ends (its formula
            // holds there): its surface takes the margin both ways, so an edge
            // ending at the end time inverts onto it from either side.
            let (lo, hi) = if lp.sheet.is_cap() {
                (0.0, 1.0)
            } else if lp.sheet.is_ring() {
                (lo - margin, hi + margin)
            } else {
                ((lo - margin).max(0.0), (hi + margin).min(1.0))
            };
            // Under the tip switch a ring envelope's surface also spans only
            // its face's ψ fractions (with the margin): a sheet that meets
            // itself holds its double curve's other preimage elsewhere in
            // ψ, and a reader inverting an edge's points onto a surface
            // holding both may land on the other.
            charts[l] = Some(if matches!(lp.sheet, SheetId::Envelope(..)) && tip_splice_on() {
                let (ulo, uhi) = members.iter().flat_map(|&m| arrangement.loops[m].arcs.iter()).flat_map(|&a| arrangement.arcs[a].params.iter().map(|p| p[0])).fold((f64::INFINITY, f64::NEG_INFINITY), |acc, u| (acc.0.min(u), acc.1.max(u)));
                let pad = 0.02 * (uhi - ulo).max(1e-3);
                self.ring_sheet_surface_over(lp.sheet, lo, hi, ((ulo - pad).max(0.0), (uhi + pad).min(1.0)))?
            } else {
                self.sheet_surface(lp.sheet, lo, hi)?
            });
        }
        // Edges: one per mated pair; each arc's coedge (edge, forward, pcurve).
        let mut edge_records = Vec::new();
        let mut coedge_of: Vec<Option<(u64, bool, NurbsCurve)>> = vec![None; n];
        let side_of = |params: &[[f64; 2]]| -> Option<usize> {
            (0..2).find(|&c| {
                let v = params[0][c];
                (v == 0.0 || v == 1.0) && params.iter().all(|q| q[c] == v)
            })
        };
        let mut worst_gap = 0.0f64;
        for a in 0..n {
            let Some((b, same_way)) = closure.mating.mate[a] else { continue };
            if a > b || arrangement.arcs[a].points.is_empty() {
                continue;
            }
            let (arc, mate) = (&arrangement.arcs[a], &arrangement.arcs[b]);
            let (start, end) = (closure.mating.end_class[2 * a], closure.mating.end_class[2 * a + 1]);
            let (Some(&(vs, xs)), Some(&(ve, xe))) = (vertex_id.get(&vertex_key(2 * a)), vertex_id.get(&vertex_key(2 * a + 1))) else {
                return Err(refuse("trim_edge", "an arc's end has no vertex".into()));
            };
            // Samples along the arc (refined where available) with exact ends.
            let mut pts: Vec<(Vec3, [f64; 2], [f64; 2])> = if refined[a].samples.len() >= 2 && refined[a].partner.is_some() {
                refined[a].samples.clone()
            } else {
                arc.points.iter().zip(&arc.params).map(|(x, p)| (*x, *p, *p)).collect()
            };
            if pts.len() < 2 {
                return Err(refuse("trim_edge", "an arc has fewer than two samples".into()));
            }
            // Partner parameters along the mate for unrefined (vertex-track) arcs.
            let mate_near = |x: Vec3| -> [f64; 2] {
                let mut best = (f64::INFINITY, mate.params[0]);
                for w in 0..mate.points.len().saturating_sub(1) {
                    let (p, q) = (mate.points[w], mate.points[w + 1]);
                    let d = q.sub(p);
                    let s = if d.dot(d) > 0.0 { (x.sub(p).dot(d) / d.dot(d)).clamp(0.0, 1.0) } else { 0.0 };
                    let g = x.sub(p.add(d.scale(s))).length();
                    if g < best.0 {
                        let (pa, pb) = (mate.params[w], mate.params[w + 1]);
                        best = (g, [pa[0] + (pb[0] - pa[0]) * s, pa[1] + (pb[1] - pa[1]) * s]);
                    }
                }
                best.1
            };
            if refined[a].partner.is_none() {
                for s in &mut pts {
                    s.2 = mate_near(s.0);
                }
            }
            let last = pts.len() - 1;
            pts[0] = (xs, corner_param(start, arc.sheet, pts[0].1), corner_param(start, mate.sheet, pts[0].2));
            pts[last] = (xe, corner_param(end, arc.sheet, pts[last].1), corner_param(end, mate.sheet, pts[last].2));
            // Stations at even arc length, refined onto both sheets.
            let mut lengths = vec![0.0];
            for w in pts.windows(2) {
                lengths.push(lengths.last().unwrap() + w[1].0.sub(w[0].0).length());
            }
            let total = *lengths.last().unwrap();
            if !(total > 1e-9 * self.size) {
                return Err(refuse("trim_edge", "an edge has no length".into()));
            }
            let first_stations = ((total / (0.01 * self.size)).ceil() as usize).clamp(8, 96);
            let own_side = side_of(&arc.params);
            let mate_side = side_of(&mate.params);
            let local_arc = matches!(arc.border, Border::Sheet(_)) && self.local_raw(arc.sheet, arc.params[arc.params.len() / 2]).is_some_and(|l| l.abs() < 1e-6);
            let chart_a = charts[loop_of[a]].as_ref().ok_or_else(|| refuse("trim_edge", "an arc's face has no surface".into()))?;
            let chart_b = charts[loop_of[b]].as_ref().ok_or_else(|| refuse("trim_edge", "an arc's face has no surface".into()))?;
            let build_at = |stations: usize| -> Result<(NurbsCurve, NurbsCurve, NurbsCurve, f64), KernelRefusal> {
                let mut gap_seen = 0.0f64;
                let mut world = Vec::with_capacity(stations + 1);
                let mut own_uv = Vec::with_capacity(stations + 1);
                let mut mate_uv = Vec::with_capacity(stations + 1);
                let mut params = Vec::with_capacity(stations + 1);
                for i in 0..=stations {
                    let target = total * i as f64 / stations as f64;
                    let w = lengths.partition_point(|&l| l < target).clamp(1, last);
                    let f = if lengths[w] > lengths[w - 1] { (target - lengths[w - 1]) / (lengths[w] - lengths[w - 1]) } else { 0.0 };
                    let lerp = |x: [f64; 2], y: [f64; 2]| [x[0] + (y[0] - x[0]) * f, x[1] + (y[1] - x[1]) * f];
                    let (x, p, q) = if i == 0 {
                        pts[0]
                    } else if i == stations {
                        pts[last]
                    } else {
                        let (p, q) = (lerp(pts[w - 1].1, pts[w].1), lerp(pts[w - 1].2, pts[w].2));
                        let (xr, pr, qr, gap) = self.refine_point(arc.sheet, mate.sheet, p, q, own_side, mate_side, local_arc).ok_or_else(|| refuse("trim_edge", "an edge station left its sheets".into()))?;
                        gap_seen = gap_seen.max(gap);
                        (xr, pr, qr)
                    };
                    world.push(x);
                    let ua = self.surface_uv(arc.sheet, chart_a, p);
                    let ub = self.surface_uv(mate.sheet, chart_b, q);
                    own_uv.push(Vec3::new(ua[0], ua[1], 0.0));
                    mate_uv.push(Vec3::new(ub[0], ub[1], 0.0));
                    params.push(i as f64 / stations as f64);
                }
                let curve = frag(crate::interpolate_curve(&world, 5, &params), "trim_edge_curve")?;
                let pa = frag(crate::interpolate_curve(&own_uv, 5, &params), "trim_pcurve")?;
                let pb = frag(crate::interpolate_curve(&mate_uv, 5, &params), "trim_pcurve")?;
                Ok((curve, pa, pb, gap_seen))
            };
            let (mut curve, mut pa, mut pb, mut gap) = build_at(first_stations)?;
            // `BREP_SWEEP_TRIM_EDGE_FLOOR`: the stations doubled toward each
            // trim, read on its face's surface, meeting the edge within half
            // the pcurve refinement tolerance at four points inside every
            // interval (a trim further off is refitted by every boolean the
            // solid enters, a global projection onto its whole carrier).
            // Best effort: what cannot get there keeps its best reading.
            if trim_edge_floor_on() {
                let target = 0.5 * crate::PCURVE_REFINEMENT_TOLERANCE;
                let miss = |curve: &NurbsCurve, pa: &NurbsCurve, pb: &NurbsCurve, stations: usize| -> Result<f64, KernelRefusal> {
                    (0..stations * 4).map(|k| (k as f64 + 0.5) / (stations * 4) as f64).try_fold(0.0f64, |worst, s| -> Result<f64, KernelRefusal> {
                        let c = frag(curve.evaluate(s), "trim_edge_curve")?;
                        let ua = frag(pa.evaluate(s), "trim_pcurve")?;
                        let ub = frag(pb.evaluate(s), "trim_pcurve")?;
                        let xa = frag(chart_a.surface.evaluate(ua.x, ua.y), "trim_surface")?;
                        let xb = frag(chart_b.surface.evaluate(ub.x, ub.y), "trim_surface")?;
                        Ok(worst.max(xa.sub(c).length()).max(xb.sub(c).length()))
                    })
                };
                // The best edge read so far is kept: a doubling that reads no
                // better (a station near an end can find the double curve's
                // other preimage) ends the refinement, the edge as it was.
                // Strictly no worse than the fixed stations: a reading that
                // fails, or a refined edge whose stations' sheets meet less
                // closely than the build admits, keeps the edge it had.
                let mut stations = first_stations;
                if let Ok(mut best) = miss(&curve, &pa, &pb, stations) {
                    while best > target && stations < 1536 {
                        let more = 2 * stations;
                        let Ok((c2, a2, b2, g2)) = build_at(more) else { break };
                        if !(g2 <= 1e-8 * self.size) {
                            break;
                        }
                        let Ok(read) = miss(&c2, &a2, &b2, more) else { break };
                        if !(read < best) {
                            break;
                        }
                        (curve, pa, pb, gap, best, stations) = (c2, a2, b2, g2, read, more);
                    }
                }
            }
            worst_gap = worst_gap.max(gap);
            let k = id();
            edge_records.push(EdgeRecord { id: k, curve, t0: 0.0, t1: 1.0, start_vertex_id: vs, end_vertex_id: ve, degenerate: false, name: None });
            coedge_of[a] = Some((k, true, pa));
            // The mate's own traversal runs with the edge when it runs the same way.
            coedge_of[b] = Some((k, same_way, if same_way { pb } else { frag(pb.reversed(), "trim_pcurve")? }));
        }
        if worst_gap > 1e-8 * self.size {
            return Err(refuse("trim_edge", format!("an edge's sheets meet only within {worst_gap:.2e}")));
        }
        // Faces: each outer loop with its holes, oriented outward.
        let mut faces = Vec::new();
        for (l, lp) in arrangement.loops.iter().enumerate() {
            let Some(chart) = charts[l].take() else { continue };
            let outward = sign * closure.face_sign[l] as f64;
            // The surface's own normal against the sheet's: the strip chart
            // may run its line the other way.
            let orientation = self.chart_orientation(lp, arrangement, &chart);
            let mut loops = Vec::new();
            for m in std::iter::once(l).chain((0..arrangement.loops.len()).filter(|&m| owners[m] == Some(l))) {
                let mut coedges = Vec::new();
                for &a in &arrangement.loops[m].arcs {
                    let (edge, forward, pcurve) = coedge_of[a].clone().ok_or_else(|| refuse("trim_face", "a loop's arc has no edge".into()))?;
                    coedges.push((edge, forward, pcurve));
                }
                if outward < 0.0 {
                    coedges.reverse();
                    for c in &mut coedges {
                        c.1 = !c.1;
                        c.2 = frag(c.2.reversed(), "trim_pcurve")?;
                    }
                }
                let records: Vec<CoedgeRecord> = coedges.into_iter().map(|(edge, forward, pcurve)| CoedgeRecord { id: id(), edge_id: edge, forward, pcurve }).collect();
                loops.push(LoopRecord { id: id(), coedges: records });
            }
            faces.push(FaceRecord { id: id(), surface: chart.surface, same_sense: outward * orientation > 0.0, loops, name: None });
        }
        let shell = id();
        let solid_id = id();
        Ok(BrepSolid {
            mass_properties_cache: Default::default(),
            id: solid_id,
            vertices: vertex_records,
            edges: edge_records,
            shells: vec![ShellRecord { id: shell, faces }],
            // One shell: Euler 2 − 2g, each vertex counted once per cycle of
            // its link (holes counted in the closure's).
            genus: (2 - closure.link_euler()) / 2,
            kind: Default::default(),
            cells: Default::default(),
        })
    }

    /// +1 when a face's surface parameters keep its sheet's orientation.
    fn chart_orientation(&self, lp: &LoopRecordT, arrangement: &Arrangement, chart: &SheetSurface) -> f64 {
        if chart.cap.is_some() {
            // The sign of the map's Jacobian (the arrangement's chart to the
            // face's frame) at a loop sample.
            let arc = &arrangement.arcs[lp.arcs[0]];
            let p = arc.params[arc.params.len() / 2];
            let h = 1e-6;
            let (a, b) = (self.surface_uv(lp.sheet, chart, [p[0] - h, p[1]]), self.surface_uv(lp.sheet, chart, [p[0] + h, p[1]]));
            let (c, d) = (self.surface_uv(lp.sheet, chart, [p[0], p[1] - h]), self.surface_uv(lp.sheet, chart, [p[0], p[1] + h]));
            let det = (b[0] - a[0]) * (d[1] - c[1]) - (b[1] - a[1]) * (d[0] - c[0]);
            return if det >= 0.0 { 1.0 } else { -1.0 };
        }
        if chart.strip.is_none() {
            return 1.0;
        }
        // Compare the parameter maps' Jacobian at a loop sample.
        let arc = &arrangement.arcs[lp.arcs[0]];
        let p = arc.params[arc.params.len() / 2];
        let h = 1e-6;
        let a = self.surface_uv(lp.sheet, chart, [p[0] - h, p[1]]);
        let b = self.surface_uv(lp.sheet, chart, [p[0] + h, p[1]]);
        if strip_jacobian_on() {
            // The whole Jacobian's sign (a strip's chart need not keep the
            // sheet's time direction), read at the first loop sample where
            // it is not degenerate.
            for &id in &lp.arcs {
                let params = &arrangement.arcs[id].params;
                for q in [params[params.len() / 2], params[params.len() / 4], params[3 * params.len() / 4]] {
                    let (a, b) = (self.surface_uv(lp.sheet, chart, [q[0] - h, q[1]]), self.surface_uv(lp.sheet, chart, [q[0] + h, q[1]]));
                    let (c, d) = (self.surface_uv(lp.sheet, chart, [q[0], q[1] - h]), self.surface_uv(lp.sheet, chart, [q[0], q[1] + h]));
                    let det = (b[0] - a[0]) * (d[1] - c[1]) - (b[1] - a[1]) * (d[0] - c[0]);
                    if det.abs() > 1e-9 * h * h {
                        return if det > 0.0 { 1.0 } else { -1.0 };
                    }
                }
            }
        }
        if b[0] >= a[0] { 1.0 } else { -1.0 }
    }

    /// The whole arrangement: every sheet's kept region chained into loops,
    /// the loops cut into arcs where their partner changes.
    pub fn arrangement(&self, rows: usize, columns: usize) -> Arrangement {
        let key = |p: [f64; 2]| ((p[0] * 1e11).round() as i64, (p[1] * 1e11).round() as i64);
        let mut arcs = Vec::new();
        let mut loops = Vec::new();
        let mut open_chains = 0;
        for sheet in self.sheets() {
            let segments = self.segments(sheet, rows, columns);
            let mut starts: std::collections::HashMap<(i64, i64), Vec<usize>> = Default::default();
            for (k, seg) in segments.iter().enumerate() {
                starts.entry(key(seg.pa)).or_default().push(k);
            }
            let mut used = vec![false; segments.len()];
            for first in 0..segments.len() {
                if used[first] {
                    continue;
                }
                let mut chain = vec![first];
                used[first] = true;
                let closed;
                loop {
                    let tail = segments[*chain.last().expect("chain")].pb;
                    if key(tail) == key(segments[first].pa) && chain.len() > 1 {
                        closed = true;
                        break;
                    }
                    let next = starts.get(&key(tail)).and_then(|list| list.iter().copied().find(|&k| !used[k]));
                    match next {
                        Some(k) => {
                            used[k] = true;
                            chain.push(k);
                        }
                        None => {
                            closed = key(tail) == key(segments[first].pa);
                            break;
                        }
                    }
                }
                if !closed {
                    open_chains += 1;
                }
                // Signed area in parameters (positive: an outer loop).
                let area: f64 = chain.iter().map(|&k| {
                    let (a, b) = (segments[k].pa, segments[k].pb);
                    0.5 * (a[0] * b[1] - b[0] * a[1])
                }).sum();
                // Start at a partner change, then cut into arcs.
                let n = chain.len();
                let start = (0..n).find(|&i| segments[chain[i]].border != segments[chain[(i + n - 1) % n]].border).unwrap_or(0);
                let mut ids = Vec::new();
                let mut current: Option<ArcRecord> = None;
                for i in 0..n {
                    let seg = &segments[chain[(start + i) % n]];
                    match &mut current {
                        Some(arc) if arc.border == seg.border => {
                            arc.points.push(seg.b);
                            arc.params.push(seg.pb);
                        }
                        _ => {
                            if let Some(arc) = current.take() {
                                ids.push(arcs.len());
                                arcs.push(arc);
                            }
                            current = Some(ArcRecord { sheet, border: seg.border, points: vec![seg.a, seg.b], params: vec![seg.pa, seg.pb] });
                        }
                    }
                }
                if let Some(arc) = current.take() {
                    ids.push(arcs.len());
                    arcs.push(arc);
                }
                loops.push(LoopRecordT { sheet, arcs: ids, area, closed });
            }
        }
        Arrangement { arcs, loops, open_chains, time_step: 1.0 / (rows.max(2) - 1) as f64 }
    }
}

/// A boundary arc of a kept region: its sheet, its partner, its points in the
/// world and in the sheet's parameters (kept side on the left).
#[derive(Clone, Debug)]
pub struct ArcRecord {
    pub sheet: SheetId,
    pub border: Border,
    pub points: Vec<Vec3>,
    pub params: Vec<[f64; 2]>,
}

impl ArcRecord {
    pub fn length(&self) -> f64 {
        self.points.windows(2).map(|w| w[1].sub(w[0]).length()).sum()
    }
}

/// A closed chain of arcs on one sheet.
#[derive(Clone, Debug)]
pub struct LoopRecordT {
    pub sheet: SheetId,
    pub arcs: Vec<usize>,
    pub area: f64,
    pub closed: bool,
}

/// Every sheet's loops and arcs.
#[derive(Clone, Debug)]
pub struct Arrangement {
    pub arcs: Vec<ArcRecord>,
    pub loops: Vec<LoopRecordT>,
    pub open_chains: usize,
    /// The time grid's step (a fraction of the motion): what lies within it
    /// of the motion's start or end is below the grid's resolution there.
    pub time_step: f64,
}

/// An arc refined onto the exact sheets: samples (world point, parameters
/// on its sheet and on its partner), the worst gap between the two sheets'
/// points, and the largest move from the grid's contour.
#[derive(Clone, Debug, Default)]
pub struct RefinedArc {
    pub samples: Vec<(Vec3, [f64; 2], [f64; 2])>,
    pub gap: f64,
    pub shift: f64,
    pub partner: Option<SheetId>,
}

#[derive(Clone, Debug)]
/// A cap's frame chart: the face's plane in its own frame (origin, u, w in
/// the tool's frame), over the box from `lo` of `size`.
struct CapChart {
    #[allow(dead_code)]
    face: usize,
    origin: Vec3,
    u: Vec3,
    w: Vec3,
    lo: (f64, f64),
    size: (f64, f64),
}

/// A face's surface and, for a strip, its chart across the zero line.
#[derive(Clone, Debug)]
struct SheetSurface {
    /// A cap's frame chart (None for every other sheet).
    cap: Option<CapChart>,
    surface: NurbsSurface,
    strip: Option<StripChart>,
    /// A ring's band: its exact torus patch's chart.
    band: Option<BandChart>,
}

/// A ring band's torus patch: ψ from `psi0` through π (`spans_u` rational
/// arcs), tube angle from `v_low` through `theta_v` (`spans_v` arcs).
#[derive(Clone, Copy, Debug)]
struct BandChart {
    psi0: f64,
    v_low: f64,
    theta_v: f64,
    spans_u: usize,
    spans_v: usize,
}

#[derive(Clone, Copy, Debug)]
struct StripChart {
    #[allow(dead_code)]
    face: usize,
    centre: (f64, f64),
    reach: f64,
    reference: (f64, f64),
}

/// A vertex solved onto every sheet that meets there.
#[derive(Clone, Debug)]
pub struct SolvedVertex {
    pub class: usize,
    pub point: Vec3,
    /// The largest distance between its corners' points (zero when exact).
    pub spread: f64,
    /// How far it moved from the grid's corners' mean.
    pub moved: f64,
    pub corners: Vec<(SheetId, [f64; 2])>,
}

/// The largest parameter change one refinement step may take (parameters
/// run over [0, 1]; a trim's grid contour is within a cell of the curve).
const STEP_CAP: f64 = 0.02;

/// One damped Gauss–Newton step: the `dz` minimising |J dz + F| (minimum
/// norm when J has more columns than rows), by the normal equations with a
/// tiny ridge, solved by Gaussian elimination with partial pivoting.
fn least_squares_step(jac: &[Vec<f64>], f: &[f64]) -> Option<Vec<f64>> {
    let rows = jac.len();
    let cols = jac.first()?.len();
    if cols == 0 {
        return Some(Vec::new());
    }
    // Minimum norm through J Jᵀ when under-determined; normal equations else.
    if cols > rows {
        let mut a = vec![vec![0.0; rows + 1]; rows];
        let scale = (0..rows).map(|i| jac[i].iter().map(|v| v * v).sum::<f64>()).fold(0.0, f64::max).max(1e-300);
        for i in 0..rows {
            for k in 0..rows {
                a[i][k] = (0..cols).map(|c| jac[i][c] * jac[k][c]).sum();
            }
            a[i][i] += 1e-14 * scale;
            a[i][rows] = -f[i];
        }
        let y = gauss(a)?;
        Some((0..cols).map(|c| (0..rows).map(|i| jac[i][c] * y[i]).sum()).collect())
    } else {
        let mut a = vec![vec![0.0; cols + 1]; cols];
        let scale = (0..cols).map(|c| (0..rows).map(|i| jac[i][c] * jac[i][c]).sum::<f64>()).fold(0.0, f64::max).max(1e-300);
        for c in 0..cols {
            for d in 0..cols {
                a[c][d] = (0..rows).map(|i| jac[i][c] * jac[i][d]).sum();
            }
            a[c][c] += 1e-14 * scale;
            a[c][cols] = -(0..rows).map(|i| jac[i][c] * f[i]).sum::<f64>();
        }
        gauss(a)
    }
}

fn gauss(mut a: Vec<Vec<f64>>) -> Option<Vec<f64>> {
    let n = a.len();
    for i in 0..n {
        let pivot = (i..n).max_by(|&x, &y| a[x][i].abs().total_cmp(&a[y][i].abs()))?;
        if a[pivot][i].abs() < 1e-300 {
            return None;
        }
        a.swap(i, pivot);
        for r in i + 1..n {
            let m = a[r][i] / a[i][i];
            for c in i..=n {
                a[r][c] -= m * a[i][c];
            }
        }
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        x[i] = (a[i][n] - (i + 1..n).map(|c| a[i][c] * x[c]).sum::<f64>()) / a[i][i];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// The closed arrangement's complex and its checks.
#[derive(Clone, Debug)]
pub struct Closure {
    pub contracted: usize,
    pub imprinted: usize,
    pub pinched: usize,
    pub faces: usize,
    pub holes: usize,
    pub edges: usize,
    pub euler: i64,
    /// Vertices counted by their links (equal to the mating's vertices when
    /// no vertex is pinched).
    pub link_vertices: usize,
    /// Mated arc pairs whose faces cannot be oriented consistently.
    pub orientation_conflicts: Vec<(usize, usize)>,
    /// Each loop's orientation sign (0 for a dropped loop).
    pub face_sign: Vec<i32>,
    /// Each arc end's link cycle (2k: arc k's start, 2k + 1 its end;
    /// `usize::MAX` where none): ends of one vertex class in different cycles
    /// lie on different sheets through a point where the surface meets itself.
    pub link_class: Vec<usize>,
    /// Chains of segments that did not close into loops.
    pub open: usize,
    pub mating: Mating,
}

impl Closure {
    /// The Euler count with every vertex counted once per cycle of its link
    /// (a vertex where the surface meets itself at a point counts once for
    /// each sheet through it).
    pub fn link_euler(&self) -> i64 {
        self.link_vertices as i64 - self.edges as i64 + self.faces as i64 - self.holes as i64
    }

    /// One closed, oriented shell, every arc mated, of any genus (its link
    /// Euler count even and at most 2), its surface allowed to meet itself at
    /// points (a double curve's tip).
    pub fn is_closed_surface(&self) -> bool {
        let chi = self.link_euler();
        self.mating.unmated.is_empty() && self.open == 0 && self.mating.shells == 1 && self.orientation_conflicts.is_empty() && chi <= 2 && chi % 2 == 0
    }

    /// One closed, oriented, manifold shell bounded by a sphere's Euler count.
    pub fn is_sphere(&self) -> bool {
        self.mating.unmated.is_empty()
            && self.open == 0
            && self.mating.shells == 1
            && self.euler == 2
            && self.holes == 0
            && self.link_vertices == self.mating.vertices
            && self.orientation_conflicts.is_empty()
    }
}

/// How the arcs pair up across sheets.
#[derive(Clone, Debug, Default)]
pub struct Mating {
    /// For each arc, its mate (if found) and whether the mate runs the same
    /// way in the world.
    pub mate: Vec<Option<(usize, bool)>>,
    /// The unmated arcs.
    pub unmated: Vec<usize>,
    /// Distinct vertices (arc ends merged within the tolerance).
    pub vertices: usize,
    /// The largest distance between an arc end and its mate's matching end.
    pub worst_end: f64,
    /// Connected components through mates.
    pub shells: usize,
    /// Each component's loops' sheets.
    pub shell_sheets: Vec<Vec<SheetId>>,
    /// The vertex class of each arc end (2k: arc k's start, 2k + 1: its
    /// end; `usize::MAX` for a contracted arc).
    pub end_class: Vec<usize>,
}

impl Arrangement {
    /// Contract the micro-arcs at fold corners: an arc shorter than `below`
    /// whose partner keeps nothing (a folded strip) is a swallowtail point
    /// resolved below the grid; a self-intersection arc is kept unless it
    /// is shorter than a fiftieth of `below`. It becomes the vertex at its middle; its neighbours in
    /// the loop end there.
    pub fn contract_fold_corners(&mut self, below: f64) -> usize {
        let kept_sheets: std::collections::BTreeSet<SheetId> = self.arcs.iter().map(|a| a.sheet).collect();
        let band = self.time_step;
        let mut removed = 0;
        // A loop of no length (a strip born at a vertex) bounds nothing.
        for lp in &mut self.loops {
            let total: f64 = lp.arcs.iter().map(|&a| self.arcs[a].length()).sum();
            if total < 1e-9 {
                removed += lp.arcs.len();
                lp.arcs.clear();
            }
        }
        for lp in &mut self.loops {
            loop {
                let n = lp.arcs.len();
                if n <= 2 {
                    break;
                }
                let found = (0..n).find(|&i| {
                    let arc = &self.arcs[lp.arcs[i]];
                    // A self-intersection is a real edge unless far below
                    // the grid (its swallowtail end); an
                    // arc whose partner keeps nothing ends in a swallowtail
                    // point below the grid; any other arc shorter than a
                    // quarter of `below` is a label switch at a triple point
                    // the grid did not resolve on both sides.
                    let (partner_empty, own) = match arc.border {
                        Border::Sheet(other) => (other != arc.sheet && !kept_sheets.contains(&other), other == arc.sheet),
                        Border::Unknown => (true, false),
                        Border::Vertex(_) => (false, false),
                    };
                    // A ring's envelope folding into its own trim at the
                    // motion's very start or end: a fold corner below the
                    // grid's last row, not an edge.
                    // (Within one step of the time grid of either end.)
                    let at_end_row = matches!(arc.sheet, SheetId::Envelope(..)) && (arc.params.iter().all(|p| p[1] <= band) || arc.params.iter().all(|p| p[1] >= 1.0 - band));
                    if own && at_end_row {
                        arc.length() < below
                    } else if own {
                        arc.length() < 0.02 * below
                    } else {
                        (partner_empty && arc.length() < below) || arc.length() < 0.25 * below
                    }
                });
                let Some(i) = found else { break };
                let id = lp.arcs[i];
                let middle = self.arcs[id].points[0].add(*self.arcs[id].points.last().expect("points")).scale(0.5);
                let (prev, next) = (lp.arcs[(i + n - 1) % n], lp.arcs[(i + 1) % n]);
                *self.arcs[prev].points.last_mut().expect("points") = middle;
                self.arcs[next].points[0] = middle;
                lp.arcs.remove(i);
                removed += 1;
            }
        }
        // Drop the contracted arcs from the list (keep ids stable by marking).
        let live: std::collections::BTreeSet<usize> = self.loops.iter().flat_map(|l| l.arcs.iter().copied()).collect();
        for (k, arc) in self.arcs.iter_mut().enumerate() {
            if !live.contains(&k) {
                arc.points.clear();
            }
        }
        removed
    }

    /// The rim between an envelope and a collar is found twice: exactly by
    /// the collar's sides and by the envelope's grid only to its resolution.
    /// Of the arcs that leaves unmated along a rim:
    /// - an envelope arc bordering a collar whose ends are one vertex (within
    ///   `tolerance`) is a sliver of the grid's rim and is contracted, its
    ///   neighbours meeting at its middle;
    /// - an envelope arc the grid labelled as crossing its own sheet, with
    ///   the ends of an unmated collar side that borders that envelope, is
    ///   that side's rim and is labelled as bordering the collar.
    /// Returns how many arcs were contracted or relabelled.
    fn repair_collar_rims(&mut self, unmated: &[usize], tolerance: f64) -> usize {
        let mut changed = 0;
        for &id in unmated {
            let arc = &self.arcs[id];
            if !matches!(arc.sheet, SheetId::Envelope(..)) || arc.points.len() < 2 {
                continue;
            }
            let (a, b) = (arc.points[0], *arc.points.last().expect("points"));
            match arc.border {
                // A sliver: every point of it within the tolerance of its
                // middle (not a long arc that happens to come back).
                Border::Sheet(SheetId::Collar(_)) if a.sub(b).length() <= tolerance && {
                    let middle = a.add(b).scale(0.5);
                    arc.points.iter().all(|x| x.sub(middle).length() <= tolerance)
                } =>
                {
                    changed += usize::from(self.contract_in_loop(id));
                }
                Border::Sheet(other) if other == arc.sheet => {
                    let sheet = arc.sheet;
                    let side = unmated.iter().copied().find(|&k| {
                        let s = &self.arcs[k];
                        matches!(s.sheet, SheetId::Collar(_))
                            && s.border == Border::Sheet(sheet)
                            && s.points.len() >= 2
                            && (s.params.iter().all(|p| p[0].abs() <= 1e-6) || s.params.iter().all(|p| (p[0] - 1.0).abs() <= 1e-6))
                            && {
                                let (c, d) = (s.points[0], *s.points.last().expect("points"));
                                (c.sub(a).length() <= tolerance && d.sub(b).length() <= tolerance) || (c.sub(b).length() <= tolerance && d.sub(a).length() <= tolerance)
                            }
                            // The same curve, not two meeting at their ends:
                            // each one's points within the tolerance of the
                            // other's polyline.
                            && polylines_within(&arc.points, &s.points, tolerance)
                    });
                    if let Some(k) = side {
                        self.arcs[id].border = Border::Sheet(self.arcs[k].sheet);
                        changed += 1;
                    }
                }
                _ => {}
            }
        }
        changed
    }

    /// Contract arc `id` in its loop (its neighbours meet at its middle), as
    /// the fold corners are; false where its loop is too short to lose it.
    fn contract_in_loop(&mut self, id: usize) -> bool {
        let Some(lp) = self.loops.iter_mut().find(|l| l.arcs.contains(&id)) else { return false };
        let n = lp.arcs.len();
        if n <= 2 {
            return false;
        }
        let i = lp.arcs.iter().position(|&x| x == id).expect("in its loop");
        let middle = self.arcs[id].points[0].add(*self.arcs[id].points.last().expect("points")).scale(0.5);
        let (prev, next) = (lp.arcs[(i + n - 1) % n], lp.arcs[(i + 1) % n]);
        *self.arcs[prev].points.last_mut().expect("points") = middle;
        self.arcs[next].points[0] = middle;
        lp.arcs.remove(i);
        self.arcs[id].points.clear();
        true
    }

    /// Unmated arcs on a collar that cross the collar itself and come back to
    /// where they left (ends one point, shorter than `below`): a double
    /// curve's tip where the collar's λ direction all but vanishes in the
    /// world, a slit below the grid with no arc across to mate. Each is
    /// contracted as the fold corners are (its neighbours meet at its
    /// middle). Returns how many.
    fn contract_unmated_collar_returns(&mut self, unmated: &[usize], below: f64) -> usize {
        let returning = |arc: &ArcRecord| {
            matches!(arc.sheet, SheetId::Collar(_))
                && arc.border == Border::Sheet(arc.sheet)
                && arc.points.len() >= 2
                && arc.length() < below
                && arc.points[0].sub(*arc.points.last().expect("points")).length() <= 1e-3 * below
        };
        let mut removed = 0;
        for &id in unmated {
            if !returning(&self.arcs[id]) {
                continue;
            }
            let Some(lp) = self.loops.iter_mut().find(|l| l.arcs.contains(&id)) else { continue };
            let n = lp.arcs.len();
            if n <= 2 {
                continue;
            }
            let i = lp.arcs.iter().position(|&a| a == id).expect("in its loop");
            let middle = self.arcs[id].points[0].add(*self.arcs[id].points.last().expect("points")).scale(0.5);
            let (prev, next) = (lp.arcs[(i + n - 1) % n], lp.arcs[(i + 1) % n]);
            *self.arcs[prev].points.last_mut().expect("points") = middle;
            self.arcs[next].points[0] = middle;
            lp.arcs.remove(i);
            self.arcs[id].points.clear();
            removed += 1;
        }
        removed
    }

    /// Unmated arcs the imprint left at the motion's very start or end, below
    /// the grid (every parameter within two time steps of an end, shorter
    /// than `below`): a partner sheet's arc there was never
    /// resolved, so the arc is a corner of its loop, contracted as the fold
    /// corners are (its neighbours meet at its middle; the vertex solve
    /// then finds the exact point). Returns how many.
    fn contract_unmated_end_slivers(&mut self, unmated: &[usize], below: f64) -> usize {
        let band = 2.0 * self.time_step;
        let sliver = |arc: &ArcRecord| {
            !arc.points.is_empty()
                && arc.length() < below
                && (arc.params.iter().all(|p| p[1] <= band) || arc.params.iter().all(|p| p[1] >= 1.0 - band))
        };
        let mut removed = 0;
        for &id in unmated {
            if !sliver(&self.arcs[id]) {
                continue;
            }
            let Some(lp) = self.loops.iter_mut().find(|l| l.arcs.contains(&id)) else { continue };
            let n = lp.arcs.len();
            if n <= 2 {
                continue;
            }
            let i = lp.arcs.iter().position(|&a| a == id).expect("in its loop");
            let middle = self.arcs[id].points[0].add(*self.arcs[id].points.last().expect("points")).scale(0.5);
            let (prev, next) = (lp.arcs[(i + n - 1) % n], lp.arcs[(i + 1) % n]);
            *self.arcs[prev].points.last_mut().expect("points") = middle;
            self.arcs[next].points[0] = middle;
            lp.arcs.remove(i);
            self.arcs[id].points.clear();
            removed += 1;
        }
        removed
    }

    /// Each hole's face: for a loop running clockwise in its sheet's
    /// parameters (area <= 0), the smallest counter-clockwise loop on the
    /// same sheet whose polygon holds it; None for an outer loop (or a hole
    /// held by none).
    pub fn hole_owners(&self) -> Vec<Option<usize>> {
        let polygon = |l: usize| -> Vec<[f64; 2]> {
            let mut poly: Vec<[f64; 2]> = Vec::new();
            for &a in &self.loops[l].arcs {
                let skip = usize::from(!poly.is_empty());
                poly.extend(self.arcs[a].params.iter().skip(skip).copied());
            }
            poly
        };
        let inside = |poly: &[[f64; 2]], q: [f64; 2]| -> bool {
            let mut c = false;
            for i in 0..poly.len() {
                let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
                if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
                    c = !c;
                }
            }
            c
        };
        let live = |l: usize| !self.loops[l].arcs.is_empty();
        (0..self.loops.len())
            .map(|h| {
                if !live(h) || self.loops[h].area > 0.0 {
                    return None;
                }
                let hole = polygon(h);
                // The face round a hole holds the hole's own boundary; an
                // island inside the hole holds none of it. Of the outer loops
                // on the sheet, the one holding the most of the boundary's
                // samples (a hole touching its face along a stretch reads
                // either way there), the smallest of equals.
                let samples: Vec<[f64; 2]> = (0..16).map(|k| hole[k * hole.len() / 16]).collect();
                (0..self.loops.len())
                    .filter(|&o| o != h && live(o) && self.loops[o].area > 0.0 && self.loops[o].sheet == self.loops[h].sheet)
                    .map(|o| {
                        let poly = polygon(o);
                        (o, samples.iter().filter(|&&q| inside(&poly, q)).count())
                    })
                    .filter(|&(_, held)| held > 0)
                    .max_by(|a, b| a.1.cmp(&b.1).then(self.loops[b.0].area.total_cmp(&self.loops[a.0].area)))
                    .map(|(o, _)| o)
            })
            .collect()
    }

    /// Split each arc along a ring envelope's own double curve that comes back
    /// to where it started (its ends one point in the world, within
    /// `tolerance`): the double curve ends at a tip inside the sheet, and the
    /// arc runs out along one side of it and back along the other. Split at
    /// the tip (its sample farthest from its ends), the two halves are each
    /// other's mates. Returns the number of splits.
    pub fn split_returning_self_arcs(&mut self, tolerance: f64) -> usize {
        let mut splits = 0;
        for l in 0..self.loops.len() {
            let mut i = 0;
            while i < self.loops[l].arcs.len() {
                let id = self.loops[l].arcs[i];
                let arc = &self.arcs[id];
                let own = matches!(arc.border, Border::Sheet(other) if other == arc.sheet) && matches!(arc.sheet, SheetId::Envelope(..));
                let n = arc.points.len();
                let shortest = if short_returning_split_on() { 0.0 } else { 4.0 * tolerance };
                if !own || n < 3 || arc.points[0].sub(arc.points[n - 1]).length() > tolerance || arc.length() <= shortest {
                    i += 1;
                    continue;
                }
                let start = arc.points[0];
                let tip = (1..n - 1).max_by(|&a, &b| arc.points[a].sub(start).length().total_cmp(&arc.points[b].sub(start).length())).expect("an inner sample");
                // A short one (only under the switch) must turn back: its
                // tip four times farther from its start than its ends are
                // from each other (not a short arc that runs straight on),
                // and its way out and its way back one curve within the
                // tolerance (not a small loop round something).
                let reach = arc.points[tip].sub(start).length();
                if arc.length() <= 4.0 * tolerance && !(reach > 4.0 * start.sub(arc.points[n - 1]).length() && polylines_within(&arc.points[..=tip], &arc.points[tip..], tolerance)) {
                    i += 1;
                    continue;
                }
                let back = ArcRecord { sheet: arc.sheet, border: arc.border, points: arc.points[tip..].to_vec(), params: arc.params[tip..].to_vec() };
                let new_id = self.arcs.len();
                self.arcs.push(back);
                self.arcs[id].points.truncate(tip + 1);
                self.arcs[id].params.truncate(tip + 1);
                self.loops[l].arcs.insert(i + 1, new_id);
                splits += 1;
                i += 2;
            }
        }
        splits
    }

    /// Whether end `which` (0 start, 1 end) of arc `b` is where a trim leaves
    /// a collar's side: `b` runs along the side (its λ fraction 0 or 1
    /// throughout, to 1e-6) and that end is off the tube angle's seam.
    fn collar_rim_crossing(&self, b: usize, which: usize) -> bool {
        let arc = &self.arcs[b];
        if !matches!(arc.sheet, SheetId::Collar(_)) || arc.params.len() < 2 {
            return false;
        }
        let side = |f: f64| if f.abs() <= 1e-6 { Some(0) } else if (f - 1.0).abs() <= 1e-6 { Some(1) } else { None };
        let Some(which_side) = side(arc.params[0][0]) else { return false };
        if arc.params.iter().any(|p| side(p[0]) != Some(which_side)) {
            return false;
        }
        let v = if which == 0 { arc.params[0][1] } else { arc.params[arc.params.len() - 1][1] };
        v > 1e-9 && v < 1.0 - 1e-9
    }

    /// Whether arc `b` borders arc `a` back (the pair a mate needs).
    fn borders_back(&self, a: usize, b: usize) -> bool {
        let (x, y) = (&self.arcs[a], &self.arcs[b]);
        match (x.border, y.border) {
            (Border::Sheet(p), Border::Sheet(q)) => p == y.sheet && q == x.sheet && a != b,
            (Border::Vertex(u), Border::Vertex(v)) => u == v && x.sheet != y.sheet,
            _ => false,
        }
    }

    /// Imprint each arc with its partners' vertices: an event on one side
    /// (a strip's end passing a tool vertex, a trace's cusp where its edge
    /// slides along itself) is an arc end there but an interior point of the
    /// arc across. Each arc is split at every end of an arc bordering it back
    /// that lies on its interior within `tolerance` (and farther than that
    /// from its own ends); the split point is that end, so the pieces mate
    /// end to end. Returns the number of splits.
    pub fn imprint(&mut self, tolerance: f64) -> usize {
        let mut splits = 0;
        for _round in 0..8 {
            let mut changed = false;
            let n = self.arcs.len();
            // With collars, the ends of each envelope's double curves with
            // itself: where one meets another sheet's border it is a vertex of
            // that border however near another, and is imprinted on it.
            let collared = self.arcs.iter().any(|c| matches!(c.sheet, SheetId::Collar(_)) && !c.points.is_empty());
            let self_ends: Vec<(SheetId, Vec3)> = if collared {
                self.arcs.iter().filter(|c| matches!(c.sheet, SheetId::Envelope(..)) && c.border == Border::Sheet(c.sheet) && c.points.len() >= 2).flat_map(|c| [(c.sheet, c.points[0]), (c.sheet, *c.points.last().expect("points"))]).collect()
            } else {
                Vec::new()
            };
            for a in 0..n {
                if self.arcs[a].points.len() < 2 {
                    continue;
                }
                let ends_a = (self.arcs[a].points[0], *self.arcs[a].points.last().expect("points"));
                // The nearest foreign end on this arc's interior.
                let mut best: Option<(f64, usize, Vec3, f64)> = None;
                for b in 0..n {
                    if self.arcs[b].points.len() < 2 || !self.borders_back(a, b) {
                        continue;
                    }
                    for (which, end) in [(0usize, self.arcs[b].points[0]), (1, *self.arcs[b].points.last().expect("points"))] {
                        // A collar side's end where a trim leaves it is an
                        // exact point of the rim it shares with an envelope,
                        // whose own grid finds the same crossing only to its
                        // resolution: it is imprinted however near the arc's
                        // own end, so long as it falls inside the arc.
                        let rim = self.collar_rim_crossing(b, which) || self_ends.iter().any(|&(sheet, x)| sheet == self.arcs[b].sheet && x.sub(end).length() <= 0.05 * tolerance);
                        let clear = if rim { 0.05 * tolerance } else { tolerance };
                        if end.sub(ends_a.0).length() <= clear || end.sub(ends_a.1).length() <= clear {
                            continue;
                        }
                        let points = &self.arcs[a].points;
                        for (k, w) in points.windows(2).enumerate() {
                            let d = w[1].sub(w[0]);
                            let len2 = d.dot(d);
                            let s = if len2 > 0.0 { (end.sub(w[0]).dot(d) / len2).clamp(0.0, 1.0) } else { 0.0 };
                            let foot = w[0].add(d.scale(s));
                            let gap = end.sub(foot).length();
                            if rim && (foot.sub(ends_a.0).length() <= clear || foot.sub(ends_a.1).length() <= clear) {
                                continue;
                            }
                            if gap <= tolerance && best.map_or(true, |x| gap < x.0) {
                                best = Some((gap, k, end, s));
                            }
                        }
                    }
                }
                let Some((_, k, end, s)) = best else { continue };
                // Split after segment k at `end`.
                let arc = &mut self.arcs[a];
                let pk = arc.params[k];
                let pq = arc.params[k + 1];
                let split_param = [pk[0] + (pq[0] - pk[0]) * s, pk[1] + (pq[1] - pk[1]) * s];
                let mut tail_points = vec![end];
                tail_points.extend_from_slice(&arc.points[k + 1..]);
                let mut tail_params = vec![split_param];
                tail_params.extend_from_slice(&arc.params[k + 1..]);
                arc.points.truncate(k + 1);
                arc.points.push(end);
                arc.params.truncate(k + 1);
                arc.params.push(split_param);
                let tail = ArcRecord { sheet: arc.sheet, border: arc.border, points: tail_points, params: tail_params };
                let id = self.arcs.len();
                self.arcs.push(tail);
                for lp in &mut self.loops {
                    if let Some(i) = lp.arcs.iter().position(|&x| x == a) {
                        lp.arcs.insert(i + 1, id);
                        break;
                    }
                }
                splits += 1;
                changed = true;
            }
            if !changed {
                break;
            }
        }
        splits
    }

    /// Close the arrangement into a face/edge/vertex complex: contract the
    /// fold-corner micro-arcs (shorter than `below`), imprint partner
    /// vertices, mate arcs (ends within `tolerance`), split pinched loops,
    /// then count: vertices, edges, faces, Euler characteristic, each
    /// vertex's link (one ring of faces per vertex, else pinched), and the
    /// orientation (a sign per face, opposite across every edge).
    pub fn close(&mut self, below: f64, tolerance: f64) -> Closure {
        let contracted = self.contract_fold_corners(below);
        self.split_returning_self_arcs(tolerance);
        let imprinted = self.imprint(tolerance);
        let mut mating = self.mate(tolerance);
        if !mating.unmated.is_empty() && self.contract_unmated_collar_returns(&mating.unmated, below) > 0 {
            mating = self.mate(tolerance);
        }
        if !mating.unmated.is_empty() && self.repair_collar_rims(&mating.unmated, tolerance) > 0 {
            mating = self.mate(tolerance);
        }
        if !mating.unmated.is_empty() && end_slivers_on() {
            let dropped = self.contract_unmated_end_slivers(&mating.unmated, below);
            if dropped > 0 {
                END_SLIVERS.with(|cell| cell.set(cell.get() + dropped as u64));
                mating = self.mate(tolerance);
            }
        }
        let pinched = self.split_pinched_loops(&mating);
        let n = self.arcs.len();
        let faces = self.loops.iter().filter(|l| !l.arcs.is_empty() && l.area > 0.0).count();
        let holes = self.loops.iter().filter(|l| !l.arcs.is_empty() && l.area <= 0.0).count();
        let live = self.arcs.iter().filter(|a| !a.points.is_empty()).count();
        let mated = live - mating.unmated.len();
        let edges = mated / 2 + mating.unmated.len();
        let euler = mating.vertices as i64 - edges as i64 + faces as i64 - holes as i64;
        fn find(p: &mut [usize], x: usize) -> usize {
            let mut x = x;
            while p[x] != x {
                p[x] = p[p[x]];
                x = p[x];
            }
            x
        }
        // Each vertex's link: corners (one per consecutive arc pair of a
        // loop) joined across mated arcs.
        let mut corner_of = vec![usize::MAX; 2 * n];
        let mut corners = 0;
        for lp in &self.loops {
            let m = lp.arcs.len();
            for i in 0..m {
                corner_of[2 * lp.arcs[i] + 1] = corners;
                corner_of[2 * lp.arcs[(i + 1) % m]] = corners;
                corners += 1;
            }
        }
        let mut parent: Vec<usize> = (0..corners).collect();
        for (a, m) in mating.mate.iter().enumerate() {
            if let Some((b, same_way)) = *m {
                let pairs = if same_way { [(2 * a, 2 * b), (2 * a + 1, 2 * b + 1)] } else { [(2 * a, 2 * b + 1), (2 * a + 1, 2 * b)] };
                for (x, y) in pairs {
                    if corner_of[x] != usize::MAX && corner_of[y] != usize::MAX {
                        let (rx, ry) = (find(&mut parent, corner_of[x]), find(&mut parent, corner_of[y]));
                        parent[rx] = ry;
                    }
                }
            }
        }
        let links: std::collections::BTreeSet<usize> = (0..corners).map(|c| find(&mut parent, c)).collect();
        let link_class: Vec<usize> = (0..2 * n).map(|e| if corner_of[e] == usize::MAX { usize::MAX } else { find(&mut parent, corner_of[e]) }).collect();
        // Orientation: a sign per loop; a mate running the same way in the
        // world needs opposite signs, a reversed one equal signs.
        let mut loop_of = vec![usize::MAX; n];
        for (l, lp) in self.loops.iter().enumerate() {
            for &a in &lp.arcs {
                loop_of[a] = l;
            }
        }
        let mut sign = vec![0i32; self.loops.len()];
        let mut conflicts = Vec::new();
        for start in 0..self.loops.len() {
            if sign[start] != 0 || self.loops[start].arcs.is_empty() {
                continue;
            }
            sign[start] = 1;
            let mut queue = vec![start];
            while let Some(l) = queue.pop() {
                for &a in &self.loops[l].arcs {
                    if let Some((b, same_way)) = mating.mate[a] {
                        let m = loop_of[b];
                        let want = if same_way { -sign[l] } else { sign[l] };
                        if sign[m] == 0 {
                            sign[m] = want;
                            queue.push(m);
                        } else if sign[m] != want && a < b {
                            conflicts.push((a, b));
                        }
                    }
                }
            }
        }
        let open = self.open_chains + self.loops.iter().filter(|l| !l.arcs.is_empty() && !l.closed).count();
        Closure { contracted, imprinted, pinched, faces, holes, edges, euler, link_vertices: links.len(), orientation_conflicts: conflicts, face_sign: sign, link_class, open, mating }
    }

    /// Split each loop that passes one vertex twice into two loops there. A
    /// kept region can pinch to a point: where an edge trace's two silhouette
    /// roots cross (the edge moves along itself), the silhouette side switches
    /// across the crossing, and the two kept quadrants touch only there. The
    /// contour chains them through the saddle as one loop; each is its own
    /// face. Needs the vertex classes of `mating`; returns the splits.
    pub fn split_pinched_loops(&mut self, mating: &Mating) -> usize {
        let class = |a: usize| mating.end_class[2 * a + 1];
        let mut splits = 0;
        let mut l = 0;
        while l < self.loops.len() {
            let arcs = self.loops[l].arcs.clone();
            let m = arcs.len();
            // Corner i: after arc i. A repeat at corners i < j, not adjacent
            // (an arc from a vertex to itself is a closed arc, not a pinch).
            // On a ring's sheet under the tip switch, only where both corners
            // are one point of the sheet's parameters (within 1e-6, a corner's
            // imprint error): two corners farther apart are two preimages of
            // one world point (a double curve's end on the sheet), and the
            // loop runs on through both.
            let end = |a: usize| *self.arcs[a].params.last().expect("params");
            let ring = self.loops[l].sheet.is_ring();
            let one_point = |i: usize, j: usize| {
                let (p, q) = (end(arcs[i]), end(arcs[j]));
                !(ring && tip_splice_on()) || ((p[0] - q[0]).abs() <= 1e-6 && (p[1] - q[1]).abs() <= 1e-6)
            };
            let found = (0..m).find_map(|i| ((i + 2)..m).find(|&j| class(arcs[i]) == class(arcs[j]) && !(i == 0 && j == m - 1) && class(arcs[i]) != usize::MAX && one_point(i, j)).map(|j| (i, j)));
            let Some((i, j)) = found else {
                l += 1;
                continue;
            };
            let inner: Vec<usize> = arcs[i + 1..=j].to_vec();
            let outer: Vec<usize> = arcs[j + 1..].iter().chain(arcs[..=i].iter()).copied().collect();
            let area = |ids: &[usize]| -> f64 {
                ids.iter().flat_map(|&a| self.arcs[a].params.windows(2).map(|w| 0.5 * (w[0][0] * w[1][1] - w[1][0] * w[0][1])).collect::<Vec<_>>()).sum()
            };
            let (a_in, a_out) = (area(&inner), area(&outer));
            let sheet = self.loops[l].sheet;
            let closed = self.loops[l].closed;
            self.loops[l] = LoopRecordT { sheet, arcs: outer, area: a_out, closed };
            self.loops.push(LoopRecordT { sheet, arcs: inner, area: a_in, closed });
            splits += 1;
        }
        splits
    }

    /// Pair every arc with the arc on its partner sheet that borders it back,
    /// by their end points (within `tolerance`).
    pub fn mate(&self, tolerance: f64) -> Mating {
        let arcs = &self.arcs;
        let n = arcs.len();
        let mut mate: Vec<Option<(usize, bool)>> = vec![None; n];
        let mut worst: f64 = 0.0;
        let ends = |k: usize| (arcs[k].points[0], *arcs[k].points.last().expect("points"));
        let dead = |k: usize| arcs[k].points.is_empty();
        for a in 0..n {
            if mate[a].is_some() || dead(a) {
                continue;
            }
            let (a0, a1) = ends(a);
            let wanted = |b: usize| -> bool {
                if b == a || mate[b].is_some() || dead(b) {
                    return false;
                }
                match (arcs[a].border, arcs[b].border) {
                    (Border::Sheet(p), Border::Sheet(q)) => p == arcs[b].sheet && q == arcs[a].sheet,
                    (Border::Vertex(u), Border::Vertex(v)) => u == v && arcs[a].sheet != arcs[b].sheet,
                    _ => false,
                }
            };
            let mut best: Option<(usize, bool, f64)> = None;
            for b in 0..n {
                if !wanted(b) {
                    continue;
                }
                let (b0, b1) = ends(b);
                let reversed = a0.sub(b1).length().max(a1.sub(b0).length());
                let same = a0.sub(b0).length().max(a1.sub(b1).length());
                let (d, same_way) = if reversed <= same { (reversed, false) } else { (same, true) };
                if best.map_or(true, |x| d < x.2) {
                    best = Some((b, same_way, d));
                }
            }
            if let Some((b, same_way, d)) = best {
                if d <= tolerance {
                    mate[a] = Some((b, same_way));
                    mate[b] = Some((a, same_way));
                    worst = worst.max(d);
                }
            }
        }
        let unmated: Vec<usize> = (0..n).filter(|&k| mate[k].is_none() && !dead(k)).collect();
        // Vertices: arc ends identified along loops (an arc's end is the
        // next one's start) and across mates; endpoint 2k is arc k's start,
        // 2k + 1 its end.
        let mut ends_parent: Vec<usize> = (0..2 * n).collect();
        fn root(p: &mut [usize], x: usize) -> usize {
            let mut x = x;
            while p[x] != x {
                p[x] = p[p[x]];
                x = p[x];
            }
            x
        }
        let join = |p: &mut Vec<usize>, a: usize, b: usize| {
            let (x, y) = (root(p, a), root(p, b));
            p[x] = y;
        };
        for lp in &self.loops {
            let m = lp.arcs.len();
            for i in 0..m {
                join(&mut ends_parent, 2 * lp.arcs[i] + 1, 2 * lp.arcs[(i + 1) % m]);
            }
        }
        for a in 0..n {
            if let Some((b, same_way)) = mate[a] {
                if same_way {
                    join(&mut ends_parent, 2 * a, 2 * b);
                    join(&mut ends_parent, 2 * a + 1, 2 * b + 1);
                } else {
                    join(&mut ends_parent, 2 * a, 2 * b + 1);
                    join(&mut ends_parent, 2 * a + 1, 2 * b);
                }
            }
        }
        let live_ends: std::collections::BTreeSet<usize> = (0..n).filter(|&k| !dead(k)).flat_map(|k| [2 * k, 2 * k + 1]).map(|e| root(&mut ends_parent, e)).collect();
        let end_class: Vec<usize> = (0..2 * n).map(|e| if dead(e / 2) { usize::MAX } else { root(&mut ends_parent, e) }).collect();
        let points = live_ends;
        // Shells: loops joined through mated arcs.
        let loop_of: Vec<usize> = {
            let mut v = vec![0; n];
            for (l, lp) in self.loops.iter().enumerate() {
                for &a in &lp.arcs {
                    v[a] = l;
                }
            }
            v
        };
        let mut parent: Vec<usize> = (0..self.loops.len()).collect();
        fn find(parent: &mut [usize], x: usize) -> usize {
            let mut x = x;
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }
        for a in (0..n).filter(|&k| !dead(k)) {
            if let Some((b, _)) = mate[a] {
                let (x, y) = (find(&mut parent, loop_of[a]), find(&mut parent, loop_of[b]));
                parent[x] = y;
            }
        }
        // A hole and the outer loop round it are one face.
        for (h, owner) in self.hole_owners().into_iter().enumerate() {
            if let Some(o) = owner {
                let (x, y) = (find(&mut parent, h), find(&mut parent, o));
                parent[x] = y;
            }
        }
        // Loops on one sheet that touch belong to one face: join loops of a
        // sheet whose arcs share an end.
        let live_loops: Vec<usize> = (0..self.loops.len()).filter(|&l| !self.loops[l].arcs.is_empty()).collect();
        let mut groups: std::collections::BTreeMap<usize, Vec<SheetId>> = Default::default();
        for &l in &live_loops {
            let r = find(&mut parent, l);
            groups.entry(r).or_default().push(self.loops[l].sheet);
        }
        Mating { mate, unmated, vertices: points.len(), worst_end: worst, shells: groups.len(), shell_sheets: groups.into_values().collect(), end_class }
    }
}

#[path = "solid_sweep_trim_ring.rs"]
mod ring;
pub use ring::{next_fit_counts, ring_trim_on, sweep_ring_tumbled, RING_BRANCH_MARGIN};

