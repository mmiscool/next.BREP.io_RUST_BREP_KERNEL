//! The block a ball-end mill leaves: the box below the mill's lower envelope
//! (the third piece of the ball-end raster construction).
//!
//! The top is the plan's regions ([`crate::mill_plan`]), each a face on its
//! part's own surface: a tube's cylinder (its seam along the top of the
//! cylinder, which the underside never reaches), a ball's sphere (its poles
//! on a level diameter and its seam over the top), or the stock's top plane.
//! The regions come from the plan's graph: each edge twice, once with the
//! region on its left; each region's boundary cycles walked by the next edge
//! clockwise at every vertex; a counter-clockwise cycle (in plan, from above)
//! an outer boundary and a clockwise one a hole, in the smallest outer
//! boundary of its part that holds it.
//!
//! Each edge's curve is cubic through its points again (about a 32nd of the
//! radius apart, each on the curve where its parts meet), and each face's
//! image of it in its surface's parameters is cubic through the same points
//! inverted onto the surface at the same parameters, never fitted. The four
//! walls are planes under the envelope's section along the box's sides, the
//! floor a plane. The plan names every edge's two faces, so the solid is
//! assembled directly (no sewing): each edge one record, used once each way;
//! each face's sense set so its normal points out of the stock; the result
//! validated.

use crate::mill_envelope::BallEnvelope;
use crate::mill_plan::{ball_field, Field, Part, Plan};
use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
use crate::{interpolate_curve, make_line, make_plane, make_revolution, make_sphere_surface_framed, BodyKind, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, Vec3};

/// The prefix of every refusal of this module.
pub const MILL_FACES_REFUSAL: &str = "millFaces";

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Collect, what, format!("{MILL_FACES_REFUSAL}: {message}"))
}

/// One use of a plan edge: the edge, and whether it runs the edge's way
/// (its region then on the edge's left).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Half {
    edge: usize,
    forward: bool,
}

/// An edge's points again (in the chart and in space) and its curve, at the
/// curve's parameters.
struct EdgeCurve {
    chart: Vec<Vec3>,
    points: Vec<Vec3>,
    parameters: Vec<f64>,
    curve: NurbsCurve,
}

/// A cubic through `points` at `parameters` (a line through two, a
/// quadratic through three).
fn through(points: &[Vec3], parameters: &[f64]) -> Result<NurbsCurve, String> {
    match points.len() {
        0 | 1 => Err("a curve needs two points".into()),
        2 => make_line(points[0], points[1]),
        n => interpolate_curve(points, (n - 1).min(3), parameters),
    }
}

/// The surfaces a plan's parts lie on: each part's surface, which way is out
/// of the stock at a point of it, and a chart point (x, y, height) lifted
/// into space.
pub trait Faces {
    fn surface(&self, part: Part) -> Result<NurbsSurface, String>;
    fn outward(&self, part: Part, at: Vec3) -> Vec3;
    fn lift(&self, chart: Vec3) -> Vec3;
}

/// What closes a plan's top below: four upright walls over a level floor at
/// `bottom` (a block milled over level ground: the chart is the plan), or one
/// floor on the plane through `origin` square to `normal` (which points out
/// of the stock) that every side of the box lifts onto (a dome's base).
#[derive(Clone, Copy, Debug)]
pub enum Sides {
    Walls { bottom: f64 },
    Floor { origin: Vec3, normal: Vec3 },
}

/// A mill's capsules over level ground: tubes about their runs, balls about
/// their centres, the top plane; the chart is the plan.
struct BallFaces<'a> {
    envelope: &'a BallEnvelope,
    plan: &'a Plan,
}

impl<'a> BallFaces<'a> {
    fn run(&self, run: usize) -> (Vec3, Vec3) {
        let moves = &self.plan.runs[run];
        (self.envelope.segments()[moves[0]].0, self.envelope.segments()[*moves.last().unwrap()].1)
    }
}

impl<'a> Faces for BallFaces<'a> {
    fn surface(&self, part: Part) -> Result<NurbsSurface, String> {
        let r = self.envelope.radius();
        let [x0, x1, y0, y1] = self.plan.bounds;
        match part {
            // the seam along the top of the cylinder, which the underside never reaches
            Part::Tube(run) => {
                let (a, b) = self.run(run);
                let axis = b.sub(a);
                let u = axis.scale(1.0 / axis.length());
                let up = Vec3::new(0.0, 0.0, 1.0);
                let over = up.sub(u.scale(up.dot(u))).normalized()?;
                let start = a.add(over.scale(r));
                make_revolution(a, u, &make_line(start, start.add(axis))?, std::f64::consts::TAU)
            }
            Part::Ball(c) => make_sphere_surface_framed(self.plan.centres[c], r, Vec3::new(0.0, 1.0, 0.0), Some(Vec3::new(0.0, 0.0, 1.0))),
            Part::Top => make_plane(Vec3::new(x0, y0, self.plan.top), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), x1 - x0, y1 - y0),
            Part::Outside => Err("no surface outside".into()),
        }
    }

    /// Up, out of the stock: toward a tube's axis or a ball's centre.
    fn outward(&self, part: Part, at: Vec3) -> Vec3 {
        match part {
            Part::Tube(run) => {
                let (a, b) = self.run(run);
                let u = b.sub(a).scale(1.0 / b.sub(a).length());
                let w = a.sub(at);
                w.sub(u.scale(w.dot(u)))
            }
            Part::Ball(c) => self.plan.centres[c].sub(at),
            _ => Vec3::new(0.0, 0.0, 1.0),
        }
    }

    fn lift(&self, chart: Vec3) -> Vec3 {
        chart
    }
}

/// The solid of the box `plan.bounds` × [`bottom`, the plan's top] below
/// `envelope`, its top read by `plan`.
pub fn milled_block(envelope: &BallEnvelope, plan: &Plan, bottom: f64) -> Result<BrepSolid, KernelRefusal> {
    let r = envelope.radius();
    let [x0, x1, y0, y1] = plan.bounds;
    // The floor below everything the envelope reaches over the box: each
    // capsule whose plan box meets it reaches no lower than its segment's
    // lower end less the radius (cross review: the vertices alone miss a
    // ball's bottom inside a face).
    let reach = envelope
        .segments()
        .iter()
        .filter(|(a, b)| a.x.min(b.x) - r <= x1 && a.x.max(b.x) + r >= x0 && a.y.min(b.y) - r <= y1 && a.y.max(b.y) + r >= y0)
        .map(|(a, b)| a.z.min(b.z) - r)
        .fold(f64::INFINITY, f64::min);
    if !(bottom < plan.top && bottom < reach) {
        return Err(refuse("mill_floor", format!("the floor at {bottom} must lie below everything the mill reaches over the box (down to {reach})")));
    }
    let field = ball_field(envelope, plan.top);
    block_from_plan(&field, &BallFaces { envelope, plan }, plan, Sides::Walls { bottom }, r / 32.0)
}

/// The solid below `plan`'s top (each region a face on its part's surface,
/// read through `faces`), closed by `sides`; each edge's points again about
/// `spacing` apart in the chart.
pub fn block_from_plan(field: &dyn Field, faces: &dyn Faces, plan: &Plan, sides: Sides, spacing: f64) -> Result<BrepSolid, KernelRefusal> {
    let r = field.scale();
    let [x0, x1, y0, y1] = plan.bounds;
    // The edges' curves.
    let mut curves = Vec::with_capacity(plan.edges.len());
    for index in 0..plan.edges.len() {
        let resampled = plan.edge_points_in(field, index, spacing).map_err(|e| refuse("mill_edge", e))?;
        // points one in space (1e-12 of the radius) kept once, the ends kept
        let (mut chart, mut points): (Vec<Vec3>, Vec<Vec3>) = (Vec::with_capacity(resampled.len()), Vec::with_capacity(resampled.len()));
        for (k, &c) in resampled.iter().enumerate() {
            let p = faces.lift(c);
            let last = k + 1 == resampled.len();
            if points.last().is_some_and(|q: &Vec3| q.sub(p).length() <= 1e-12 * r) {
                if last {
                    *chart.last_mut().unwrap() = c;
                    *points.last_mut().unwrap() = p;
                }
                continue;
            }
            chart.push(c);
            points.push(p);
        }
        if points.len() < 2 {
            let e = &plan.edges[index];
            return Err(refuse("mill_edge", format!("the edge {index} ({:?}|{:?}, {} to {}) is one point: {:?}", e.left, e.right, e.from, e.to, resampled)));
        }
        let mut parameters = vec![0.0f64];
        for w in points.windows(2) {
            parameters.push(parameters.last().unwrap() + w[1].sub(w[0]).length());
        }
        let total = *parameters.last().unwrap();
        if !(total > 0.0) {
            return Err(refuse("mill_edge", format!("the edge {index} has no length")));
        }
        for p in &mut parameters {
            *p /= total;
        }
        *parameters.last_mut().unwrap() = 1.0;
        let curve = through(&points, &parameters).map_err(|e| refuse("mill_edge", format!("the edge {index}: {e}")))?;
        curves.push(EdgeCurve { chart, points, parameters, curve });
    }
    let leave = 1e-3 * r / field.length();
    // Every half-edge out of each vertex, by the angle it leaves at.
    let leaving = |half: Half| -> f64 {
        let points = &curves[half.edge].chart;
        let (start, rest): (Vec3, Box<dyn Iterator<Item = &Vec3>>) = if half.forward { (points[0], Box::new(points.iter().skip(1))) } else { (*points.last().unwrap(), Box::new(points.iter().rev().skip(1))) };
        // the first point a thousandth of the radius away (or the far end)
        let mut toward = start;
        for p in rest {
            toward = *p;
            if ((p.x - start.x).powi(2) + (p.y - start.y).powi(2)).sqrt() >= leave {
                break;
            }
        }
        (toward.y - start.y).atan2(toward.x - start.x)
    };
    let mut out: Vec<Vec<(f64, Half)>> = vec![Vec::new(); plan.vertices.len()];
    for (edge, e) in plan.edges.iter().enumerate() {
        for forward in [true, false] {
            let half = Half { edge, forward };
            out[if forward { e.from } else { e.to }].push((leaving(half), half));
        }
    }
    for list in &mut out {
        list.sort_by(|a, b| a.0.total_cmp(&b.0));
    }
    let end_of = |h: Half| if h.forward { plan.edges[h.edge].to } else { plan.edges[h.edge].from };
    let part_of = |h: Half| if h.forward { plan.edges[h.edge].left } else { plan.edges[h.edge].right };
    // The cycles: from each unused half-edge, the next clockwise from its twin at each end.
    let mut used = vec![[false; 2]; plan.edges.len()];
    let mut cycles: Vec<(Part, Vec<Half>)> = Vec::new();
    for edge in 0..plan.edges.len() {
        for forward in [true, false] {
            let first = Half { edge, forward };
            if used[edge][forward as usize] || part_of(first) == Part::Outside {
                continue;
            }
            let mut cycle = Vec::new();
            let mut half = first;
            loop {
                if used[half.edge][half.forward as usize] {
                    if half == first {
                        break;
                    }
                    return Err(refuse("mill_cycle", format!("the boundary walk from edge {edge} meets a used edge {}", half.edge)));
                }
                used[half.edge][half.forward as usize] = true;
                cycle.push(half);
                let vertex = end_of(half);
                let twin = Half { edge: half.edge, forward: !half.forward };
                let list = &out[vertex];
                let at = list.iter().position(|(_, h)| *h == twin).ok_or_else(|| refuse("mill_cycle", "a half-edge is missing at its vertex".into()))?;
                half = list[(at + list.len() - 1) % list.len()].1;
                if part_of(half) != part_of(first) {
                    let around: Vec<String> = list.iter().map(|(angle, h)| { let e = &plan.edges[h.edge]; format!("{angle:.6} e{} {:?}|{:?} fwd {} {}->{} n{} {:?}", h.edge, e.left, e.right, h.forward, e.from, e.to, e.points.len(), &e.points[..e.points.len().min(4)]) }).collect();
                    return Err(refuse("mill_cycle", format!("the boundary of {:?} runs on into {:?} at the vertex {vertex} {:?}: {around:?}", part_of(first), part_of(half), plan.vertices[vertex].point)));
                }
                if cycle.len() > plan.edges.len() * 2 {
                    return Err(refuse("mill_cycle", "a boundary walk does not close".into()));
                }
            }
            cycles.push((part_of(first), cycle));
        }
    }
    // Outer boundaries and holes, by the area each cycle bounds in plan.
    let polygon = |cycle: &[Half]| -> Vec<[f64; 2]> {
        let mut ring = Vec::new();
        for h in cycle {
            let points = &curves[h.edge].chart;
            let ordered: Box<dyn Iterator<Item = &Vec3>> = if h.forward { Box::new(points.iter()) } else { Box::new(points.iter().rev()) };
            for p in ordered.skip(1) {
                ring.push([p.x, p.y]);
            }
        }
        ring
    };
    let area = |ring: &[[f64; 2]]| 0.5 * (0..ring.len()).map(|k| { let (a, b) = (ring[k], ring[(k + 1) % ring.len()]); a[0] * b[1] - b[0] * a[1] }).sum::<f64>();
    let holds = |ring: &[[f64; 2]], p: [f64; 2]| {
        let mut inside = false;
        for k in 0..ring.len() {
            let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
            if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
                inside = !inside;
            }
        }
        inside
    };
    let rings: Vec<Vec<[f64; 2]>> = cycles.iter().map(|(_, c)| polygon(c)).collect();
    let areas: Vec<f64> = rings.iter().map(|ring| area(ring)).collect();
    let mut faces_of: Vec<(Part, Vec<usize>)> = Vec::new();
    let mut face_of = vec![usize::MAX; cycles.len()];
    for k in 0..cycles.len() {
        if areas[k] > 0.0 {
            face_of[k] = faces_of.len();
            faces_of.push((cycles[k].0, vec![k]));
        }
    }
    for k in 0..cycles.len() {
        if areas[k] > 0.0 {
            continue;
        }
        let probe = rings[k][0];
        let host = (0..cycles.len())
            .filter(|&o| areas[o] > 0.0 && cycles[o].0 == cycles[k].0 && holds(&rings[o], probe))
            .min_by(|&a, &b| areas[a].total_cmp(&areas[b]))
            .ok_or_else(|| refuse("mill_hole", format!("a hole in {:?} lies in no outer boundary of its part", cycles[k].0)))?;
        faces_of[face_of[host]].1.push(k);
    }
    // The records: the plan's vertices, then the floor's corners; the plan's
    // edges, then the walls' uprights and the floor's sides.
    let corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
    let corner_vertex = |c: [f64; 2]| plan.vertices.iter().position(|v| v.point.x == c[0] && v.point.y == c[1]).ok_or_else(|| refuse("mill_wall", "a corner of the box is missing".into()));
    let top_corners = [corner_vertex(corners[0])?, corner_vertex(corners[1])?, corner_vertex(corners[2])?, corner_vertex(corners[3])?];
    let bottom = match sides { Sides::Walls { bottom } => bottom, Sides::Floor { .. } => f64::NAN };
    let low = |k: usize| Vec3::new(corners[k][0], corners[k][1], bottom);
    let mut vertices: Vec<VertexRecord> = plan.vertices.iter().enumerate().map(|(k, v)| VertexRecord { id: k as u64 + 1, point: faces.lift(v.point) }).collect();
    let low_vertex = |k: usize| (plan.vertices.len() + k) as u64 + 1;
    let walls = matches!(sides, Sides::Walls { .. });
    if walls {
        for k in 0..4 {
            vertices.push(VertexRecord { id: low_vertex(k), point: low(k) });
        }
    }
    let mut next = vertices.len() as u64;
    let mut id = || {
        next += 1;
        next
    };
    let mut edges: Vec<EdgeRecord> = Vec::with_capacity(plan.edges.len() + 8);
    let record = |curve: NurbsCurve, from: u64, to: u64, edges: &mut Vec<EdgeRecord>, id: u64| -> Result<usize, KernelRefusal> {
        let [t0, t1] = curve.domain().map_err(|e| refuse("mill_edge", e))?;
        edges.push(EdgeRecord { id, curve, t0, t1, start_vertex_id: from, end_vertex_id: to, degenerate: false, name: None });
        Ok(edges.len() - 1)
    };
    for (k, e) in plan.edges.iter().enumerate() {
        let curve = curves[k].curve.clone();
        let edge_id = id();
        record(curve, e.from as u64 + 1, e.to as u64 + 1, &mut edges, edge_id)?;
    }
    // uprights from each floor corner to the top's corner; floor sides corner k to k + 1
    let mut upright = [0usize; 4];
    let mut floor_side = [0usize; 4];
    if walls {
        for k in 0..4 {
            let curve = make_line(low(k), faces.lift(plan.vertices[top_corners[k]].point)).map_err(|e| refuse("mill_wall", e))?;
            let edge_id = id();
            upright[k] = record(curve, low_vertex(k), top_corners[k] as u64 + 1, &mut edges, edge_id)?;
        }
        for k in 0..4 {
            let curve = make_line(low(k), low((k + 1) % 4)).map_err(|e| refuse("mill_floor", e))?;
            let edge_id = id();
            floor_side[k] = record(curve, low_vertex(k), low_vertex((k + 1) % 4), &mut edges, edge_id)?;
        }
    }
    // Each side's chain of edges with Outside on the right, corner to corner.
    let mut chains: Vec<Vec<usize>> = Vec::with_capacity(4);
    for side in 0..4 {
        let (start, finish) = (top_corners[side], top_corners[(side + 1) % 4]);
        let mut chain = Vec::new();
        let mut at = start;
        while at != finish {
            let next = plan.edges.iter().position(|e| e.right == Part::Outside && e.from == at).ok_or_else(|| refuse("mill_wall", format!("the side {side} breaks off")))?;
            chain.push(next);
            at = plan.edges[next].to;
            if chain.len() > plan.edges.len() {
                return Err(refuse("mill_wall", format!("the side {side} does not reach its corner")));
            }
        }
        chains.push(chain);
    }
    // A face: its surface, its outward direction at a point of it, and its
    // loops of (edge index, forward) counter-clockwise about the outward normal.
    let mut face_records: Vec<FaceRecord> = Vec::with_capacity(faces_of.len() + 5);
    let mut make_face = |surface: NurbsSurface, outward: &dyn Fn(Vec3) -> Vec3, loops: Vec<Vec<(usize, bool)>>, id: &mut dyn FnMut() -> u64| -> Result<(), KernelRefusal> {
        let mut loop_records = Vec::with_capacity(loops.len());
        let mut same_sense = None;
        for uses in loops {
            let mut coedges = Vec::with_capacity(uses.len());
            for (edge, forward) in uses {
                // the edge's points (a line's two ends) inverted onto the surface, at the curve's parameters
                let (points, parameters): (Vec<Vec3>, Vec<f64>) = if edge < curves.len() {
                    (curves[edge].points.clone(), curves[edge].parameters.clone())
                } else {
                    let curve = &edges[edge].curve;
                    (vec![curve.evaluate(edges[edge].t0).map_err(|e| refuse("mill_edge", e))?, curve.evaluate(edges[edge].t1).map_err(|e| refuse("mill_edge", e))?], vec![edges[edge].t0, edges[edge].t1])
                };
                let mut uv = Vec::with_capacity(points.len());
                for p in &points {
                    let (at, distance) = crate::pcurve::invert_checked(&surface, *p).map_err(|err| refuse("mill_invert", err))?;
                    if !(distance <= 1e-6 * r) {
                        return Err(refuse("mill_invert", format!("a point {p:?} of the edge {edge} ({:?}|{:?}) lies {distance:e} off its face's surface (at {at:?})", plan.edges.get(edge).map(|e| e.left), plan.edges.get(edge).map(|e| e.right))));
                    }
                    uv.push(Vec3::new(at[0], at[1], 0.0));
                }
                if same_sense.is_none() {
                    let (u, w) = (uv[uv.len() / 2].x, uv[uv.len() / 2].y);
                    let normal = surface.normal(u, w).map_err(|e| refuse("mill_normal", e))?;
                    same_sense = Some(normal.dot(outward(points[points.len() / 2])) > 0.0);
                }
                let pcurve = through(&uv, &parameters).map_err(|err| refuse("mill_pcurve", err))?;
                let pcurve = if forward { pcurve } else { pcurve.reversed().map_err(|err| refuse("mill_reverse", err))? };
                coedges.push(CoedgeRecord { id: id(), edge_id: edges[edge].id, forward, pcurve });
            }
            loop_records.push(LoopRecord { id: id(), coedges });
        }
        face_records.push(FaceRecord { id: id(), surface, same_sense: same_sense.unwrap_or(true), loops: loop_records, name: None });
        Ok(())
    };
    // The top's faces, each sensed out of the stock.
    for (part, loops) in &faces_of {
        let surface = faces.surface(*part).map_err(|e| refuse("mill_surface", format!("{part:?}: {e}")))?;
        let part = *part;
        let loops = loops.iter().map(|&k| cycles[k].1.iter().map(|h| (h.edge, h.forward)).collect()).collect();
        make_face(surface, &move |at: Vec3| faces.outward(part, at), loops, &mut id)?;
    }
    match sides {
        // The walls, seen from outside: the floor's side along, up the far
        // upright, back along the top's side (its edges run the other way),
        // down the near upright. The floor, seen from below.
        Sides::Walls { bottom } => {
            for side in 0..4 {
                let mut uses = vec![(floor_side[side], true), (upright[(side + 1) % 4], true)];
                uses.extend(chains[side].iter().rev().map(|&e| (e, false)));
                uses.push((upright[side], false));
                let along = low((side + 1) % 4).sub(low(side));
                let unit = along.normalized().map_err(|e| refuse("mill_wall", e))?;
                let plane = make_plane(low(side), unit, Vec3::new(0.0, 0.0, 1.0), along.length(), plan.top - bottom).map_err(|e| refuse("mill_wall", e))?;
                let out_of = Vec3::new(unit.y, -unit.x, 0.0);
                make_face(plane, &move |_| out_of, vec![uses], &mut id)?;
            }
            let floor = make_plane(low(0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), x1 - x0, y1 - y0).map_err(|e| refuse("mill_floor", e))?;
            make_face(floor, &|_| Vec3::new(0.0, 0.0, -1.0), vec![vec![(floor_side[3], false), (floor_side[2], false), (floor_side[1], false), (floor_side[0], false)]], &mut id)?;
        }
        // One floor under the whole boundary, seen from outside it: the
        // sides' edges the other way, last first.
        Sides::Floor { origin, normal } => {
            let normal = normal.normalized().map_err(|e| refuse("mill_floor", e))?;
            let u = normal.perpendicular().map_err(|e| refuse("mill_floor", e))?;
            let w = normal.cross(u);
            let reach = curves.iter().flat_map(|c| c.points.iter()).map(|p| p.sub(origin).length()).fold(0.0f64, f64::max) * 1.01 + r;
            let floor = make_plane(origin.sub(u.scale(reach)).sub(w.scale(reach)), u, w, 2.0 * reach, 2.0 * reach).map_err(|e| refuse("mill_floor", e))?;
            let uses: Vec<(usize, bool)> = chains.iter().flatten().rev().map(|&e| (e, false)).collect();
            make_face(floor, &move |_| normal, vec![uses], &mut id)?;
        }
    }
    let shell = ShellRecord { id: id(), faces: face_records };
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: id(),
        vertices,
        edges,
        shells: vec![shell],
        genus: 0,
        kind: BodyKind::Solid,
        cells: Default::default(),
    };
    let issues = solid.validate();
    if let Some(issue) = issues.first() {
        return Err(refuse("mill_invalid", format!("the block is not a valid solid: {issue:?}")));
    }
    Ok(solid)
}

