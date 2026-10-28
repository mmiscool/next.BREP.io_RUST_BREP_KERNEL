//! Thread milling (lane C5 row 7): helical passes of a thread mill along a
//! hole (internal) or boss (external). The tool centre radius is the major
//! radius minus the mill radius (internal) or plus it (external), approached
//! in `passes` radial steps over the thread depth (0.6134 x pitch, ISO 60
//! degree profile). A multi-point mill (cutting length at least the thread
//! length) cuts one turn plus a small overlap; a single-point mill runs the
//! whole length one pitch per turn. Climb: counter-clockwise inside,
//! clockwise outside; the helix hand decides whether that climbs up or down
//! (a right-hand helix rises counter-clockwise). The thread cuts into the
//! wall by design, so fixtures, protected bodies and avoid faces are the
//! gouge check; the floor at the bottom is still checked.

use crate::bore::{helix_arcs, helix_points};
use crate::model::{Feed, Motion, MotionKind, Operation, OperationReport, Point3, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{invalid, OpContext, PlanarInput};
use crate::{CamError, MeshSnapshot, Runner, Tolerance, TriangleIndex};

/// ISO metric thread depth for a pitch (H = 0.866 p, depth = 5/8 H + ...; the
/// usual 0.6134 p).
pub fn thread_depth(pitch: f64) -> f64 {
    0.6134 * pitch
}

#[allow(clippy::too_many_arguments)]
pub fn generate(
    operation: &Operation,
    setup: &Setup,
    tool: &Tool,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    input: &PlanarInput,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    let params = match &operation.strategy {
        Strategy::ThreadMill(p) => p,
        _ => return Err(invalid("threadMill", "requires ThreadMill parameters")),
    };
    // A thread mill has no drop-cutter profile; it is treated as a flat
    // cutter of its diameter for every check, through a flat stand-in tool.
    let mut stand_in = tool.clone();
    if stand_in.kind == crate::model::ToolKind::ThreadMill {
        stand_in.kind = crate::model::ToolKind::Flat;
    }
    let ctx = OpContext::new("threadMill", operation, setup, &stand_in, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if input.circles.is_empty() {
        return Err(invalid("threadMill", "no circular features selected"));
    }
    if params.passes == 0 {
        return Err(invalid("threadMill", "at least one pass is needed"));
    }
    let r = ctx.radius();
    let major = 0.5 * params.diameter;
    let depth = thread_depth(params.pitch);
    let mut report = OperationReport::default();
    report.warn(crate::model::Diagnostic::new("thread-model-not-checked", "thread milling cuts the wall by design; only fixtures, protected bodies, avoid faces and the floor are gouge-checked"));
    let mut motions: Vec<Motion> = Vec::new();
    let mut pass_count = 0u64;
    runner.stage("thread", "Thread passes", 0, input.circles.len() as u64)?;
    for (ci, circle) in input.circles.iter().enumerate() {
        circle.validate("threadMill")?;
        if params.internal != circle.internal {
            return Err(invalid("threadMill", format!("feature {ci} is {} but the operation is {}", if circle.internal { "a hole" } else { "a boss" }, if params.internal { "internal" } else { "external" })));
        }
        let top = params.depths.top.unwrap_or(circle.top);
        let bottom = params.depths.bottom.unwrap_or(circle.bottom);
        let length = top - bottom;
        if length <= 0.0 {
            return Err(invalid("threadMill", format!("feature {ci}: thread length must be positive")));
        }
        let final_rho = if params.internal { major - r } else { major + r };
        if params.internal && final_rho <= tol.sample_eps {
            return Err(invalid("threadMill", format!("feature {ci}: the thread mill does not fit the thread diameter")));
        }
        let c = circle.center;
        let ccw = params.internal; // climb
        let rises = ccw != params.left_hand; // a right-hand helix rises counter-clockwise
        let multi_point = stand_in.geometry.cutting_length + tol.distance_eps >= length;
        let turns = if multi_point { 1.0 + 0.05 } else { length / params.pitch + 1.0 };
        let (z_start, z_end) = if rises { (bottom, bottom + turns * params.pitch) } else { (top, top - turns * params.pitch) };
        let feed = ctx.feeds.cutting_feed;
        let path_id = ci as u32;
        // Approach point: the centre of a hole, or outside a boss along +X.
        let park = if params.internal { Point3::new(c.x, c.y, z_start) } else { Point3::new(c.x + final_rho + 2.0 * r, c.y, z_start) };
        motions.push(Motion::rapid(Point3::new(park.x, park.y, ctx.heights.clearance)).with_path(path_id));
        // Rapids stop above the stock over the hole, not only above its top.
        let feed_z = ctx.heights.feed.max(ctx.material_over(&params.depths, input, top) + operation.linking.safe_distance).min(ctx.heights.clearance);
        if feed_z < ctx.heights.clearance {
            motions.push(Motion::rapid(Point3::new(park.x, park.y, feed_z)).with_path(path_id));
        }
        ctx.certify_3d(&[Point3::new(park.x, park.y, feed_z), park], false, &mut report, runner)?;
        motions.push(Motion::plunge(park, ctx.feeds.plunge_feed).with_path(path_id));
        let mut at = park;
        for k in 0..params.passes {
            let rho = if params.internal { final_rho - depth * (params.passes - 1 - k) as f64 / params.passes as f64 } else { final_rho + depth * (params.passes - 1 - k) as f64 / params.passes as f64 };
            let start = Point3::new(c.x + rho, c.y, z_start);
            // Radial lead to the start, the helix, radial lead back.
            ctx.certify_3d(&[at, start], true, &mut report, runner)?;
            motions.push(Motion::new(MotionKind::LeadIn { to: start }, Feed::Rate(ctx.feeds.lead_feed())).with_path(path_id));
            let pts = helix_points(c, rho, 0.0, turns, ccw, z_start, z_end, operation.tolerance);
            ctx.certify_3d(&pts, true, &mut report, runner)?;
            // The helix stays within the feature's own depth (its floor).
            let low = z_start.min(z_end);
            if low < circle.bottom - tol.distance_eps {
                return Err(invalid("threadMill", format!("feature {ci}: the thread helix reaches Z {low:.4} below the feature bottom {:.4}", circle.bottom)));
            }
            motions.extend(helix_arcs(c, rho, 0.0, turns, ccw, z_start, z_end, feed, path_id));
            let end = motions.last().and_then(|m| m.end()).unwrap();
            let back = Point3::new(park.x, park.y, end.z);
            ctx.certify_3d(&[end, back], true, &mut report, runner)?;
            motions.push(Motion::new(MotionKind::LeadOut { to: back }, Feed::Rate(ctx.feeds.lead_feed())).with_path(path_id));
            // Return to the start height for the next pass.
            if k + 1 < params.passes {
                let next = Point3::new(park.x, park.y, z_start);
                motions.push(Motion::new(MotionKind::Link { to: next }, Feed::Rate(ctx.feeds.plunge_feed)).with_path(path_id));
                at = next;
            } else {
                at = back;
            }
            pass_count += 1;
        }
        motions.push(Motion::retract(Point3::new(at.x, at.y, ctx.heights.clearance), ctx.feeds.retract_feed.map_or(Feed::RAPID, Feed::Rate)).with_path(path_id));
        runner.work(1, "thread", ci as u64 + 1, input.circles.len() as u64)?;
    }
    ctx.direct(motions, pass_count, report)
}
