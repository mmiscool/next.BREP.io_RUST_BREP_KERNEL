//! Bore (lane C5 row 7): helical milling of a hole or boss wall. The tool
//! centre runs on a circle of radius `hole - cutter - radial stock` (hole)
//! or `boss + cutter + radial stock` (boss), descending `pitch` per turn as
//! exact helical arcs, with a full circle at the bottom when a finish pass
//! is asked for; a zero pitch cuts a full circle at every depth level
//! instead. Climb: counter-clockwise inside a hole, clockwise around a boss.
//! Every helix chord is certified against the full protected mesh.

use crate::model::{ArcPlane, Feed, Motion, MotionKind, Operation, OperationReport, Point3, Setup, Strategy, Tool, Toolpath};
use crate::planar::machining::{depth_levels, invalid, OpContext, PlanarInput};
use crate::pocket2d::wall_margin;
use crate::{CamError, MeshSnapshot, Runner, Tolerance, TriangleIndex};
use std::f64::consts::TAU;

/// Points along a helix about `center` at radius `rho` from angle `a0`,
/// `turns` turns in the given sense, from `z0` down/up to `z1`, dense
/// enough for the chord tolerance.
pub fn helix_points(center: crate::model::Point2, rho: f64, a0: f64, turns: f64, ccw: bool, z0: f64, z1: f64, chord_tol: f64) -> Vec<Point3> {
    helix_points_with(center, rho, a0, turns, ccw, z0, z1, chord_tol, false)
}

/// As [`helix_points`]; `outward` puts the vertices at `rho + sagitta` so the
/// chords never come closer to the axis than `rho` (material inside, a boss).
#[allow(clippy::too_many_arguments)]
pub fn helix_points_with(center: crate::model::Point2, rho: f64, a0: f64, turns: f64, ccw: bool, z0: f64, z1: f64, chord_tol: f64, outward: bool) -> Vec<Point3> {
    let per_turn = if chord_tol >= rho { 4.0 } else { (TAU / (2.0 * (1.0 - chord_tol / rho).acos())).ceil().max(4.0) };
    let n = ((per_turn * turns).ceil().max(1.0) as usize).min(1_000_000);
    let sign = if ccw { 1.0 } else { -1.0 };
    if !outward {
        return (0..=n)
            .map(|i| {
                let f = i as f64 / n as f64;
                let a = a0 + sign * TAU * turns * f;
                Point3::new(center.x + rho * a.cos(), center.y + rho * a.sin(), z0 + (z1 - z0) * f)
            })
            .collect();
    }
    // Circumscribed: vertices at the half-step angles on the tangent lines.
    let step = TAU * turns / n as f64;
    let r_vertex = rho / (0.5 * step).cos();
    let mut pts = Vec::with_capacity(n + 2);
    pts.push(Point3::new(center.x + rho * a0.cos(), center.y + rho * a0.sin(), z0));
    for i in 0..n {
        let f = (i as f64 + 0.5) / n as f64;
        let a = a0 + sign * TAU * turns * f;
        pts.push(Point3::new(center.x + r_vertex * a.cos(), center.y + r_vertex * a.sin(), z0 + (z1 - z0) * f));
    }
    let a1 = a0 + sign * TAU * turns;
    pts.push(Point3::new(center.x + rho * a1.cos(), center.y + rho * a1.sin(), z1));
    pts
}

/// Helical arc motions for the same helix: one arc per full turn and one
/// for the remainder, every arc ending exactly on its computed point.
pub fn helix_arcs(center: crate::model::Point2, rho: f64, a0: f64, turns: f64, ccw: bool, z0: f64, z1: f64, feed: f64, path_id: u32) -> Vec<Motion> {
    let mut out = Vec::new();
    let full = turns.floor() as usize;
    let pitch = if turns > 0.0 { (z1 - z0) / turns } else { 0.0 };
    let at = |t: f64| -> Point3 {
        let a = a0 + if ccw { 1.0 } else { -1.0 } * TAU * t;
        Point3::new(center.x + rho * a.cos(), center.y + rho * a.sin(), z0 + pitch * t)
    };
    let c = Point3::new(center.x, center.y, z0);
    let arc = |to: Point3| {
        let kind = if ccw { MotionKind::ArcCcw { to, center: c, plane: ArcPlane::Xy, pitch } } else { MotionKind::ArcCw { to, center: c, plane: ArcPlane::Xy, pitch } };
        Motion::new(kind, Feed::Rate(feed)).with_path(path_id)
    };
    for k in 1..=full {
        out.push(arc(at(k as f64)));
    }
    if turns - full as f64 > 1e-9 {
        out.push(arc(at(turns)));
    }
    if let Some(last) = out.last_mut() {
        if let MotionKind::ArcCcw { to, .. } | MotionKind::ArcCw { to, .. } = &mut last.kind {
            to.z = z1;
        }
    }
    out
}

/// A full circle at `z` as two half-turn arcs.
pub fn full_circle(center: crate::model::Point2, rho: f64, a0: f64, ccw: bool, z: f64, feed: f64, path_id: u32) -> Vec<Motion> {
    helix_arcs(center, rho, a0, 1.0, ccw, z, z, feed, path_id)
        .into_iter()
        .flat_map(|m| {
            // Split the single full-turn arc into two halves for controllers
            // that refuse 360-degree arcs.
            let c = Point3::new(center.x, center.y, z);
            let half = a0 + if ccw { std::f64::consts::PI } else { -std::f64::consts::PI };
            let mid = Point3::new(center.x + rho * half.cos(), center.y + rho * half.sin(), z);
            let first = if ccw { MotionKind::ArcCcw { to: mid, center: c, plane: ArcPlane::Xy, pitch: 0.0 } } else { MotionKind::ArcCw { to: mid, center: c, plane: ArcPlane::Xy, pitch: 0.0 } };
            vec![Motion::new(first, m.feed).with_path(path_id), m]
        })
        .collect()
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
        Strategy::Bore(p) => p,
        _ => return Err(invalid("bore", "requires Bore parameters")),
    };
    let ctx = OpContext::new("bore", operation, setup, tool, mesh, index, tolerance)?;
    let tol = &ctx.tolerance;
    if input.circles.is_empty() {
        return Err(invalid("bore", "no circular features selected"));
    }
    let r = ctx.radius();
    let radial = operation.stock_to_leave.radial;
    let mut report = OperationReport::default();
    let mut motions: Vec<Motion> = Vec::new();
    let mut passes = 0u64;
    runner.stage("bore", "Bore helices", 0, input.circles.len() as u64)?;
    for (ci, circle) in input.circles.iter().enumerate() {
        circle.validate("bore")?;
        let rho = if circle.internal { circle.radius - r - radial - wall_margin(tol, operation.tolerance) } else { circle.radius + r + radial + wall_margin(tol, operation.tolerance) };
        if rho <= tol.sample_eps {
            return Err(invalid("bore", format!("hole {ci}: the cutter (radius {r}) does not fit the hole radius {}", circle.radius)));
        }
        let top = params.depths.top.unwrap_or(circle.top);
        let bottom = params.depths.bottom.unwrap_or(circle.bottom) + operation.stock_to_leave.axial;
        if bottom > top {
            return Err(invalid("bore", format!("hole {ci}: bottom {bottom} above top {top}")));
        }
        // Climb: counter-clockwise inside, clockwise outside.
        let ccw = circle.internal;
        let c = circle.center;
        let start = Point3::new(c.x + rho, c.y, top);
        let feed = ctx.feeds.cutting_feed;
        let path_id = ci as u32;
        let depth = top - bottom;
        // Chords of the whole motion for certification.
        let mut chords: Vec<Point3> = vec![start];
        let mut cut: Vec<Motion> = Vec::new();
        if params.pitch > 0.0 && depth > 0.0 {
            let turns = depth / params.pitch;
            chords.extend(helix_points_with(c, rho, 0.0, turns, ccw, top, bottom, operation.tolerance, !circle.internal).into_iter().skip(1));
            cut.extend(helix_arcs(c, rho, 0.0, turns, ccw, top, bottom, feed, path_id));
            if params.finish_pass {
                let a_end = if ccw { TAU * turns } else { -TAU * turns };
                chords.extend(helix_points_with(c, rho, a_end, 1.0, ccw, bottom, bottom, operation.tolerance, !circle.internal).into_iter().skip(1));
                cut.extend(full_circle(c, rho, a_end, ccw, bottom, feed, path_id));
            }
        } else {
            // Circular passes at every depth level, joined by plunges.
            let levels = depth_levels(top, bottom, &params.depths, "bore")?;
            for (k, &z) in levels.iter().enumerate() {
                let from = if k == 0 { start } else { Point3::new(start.x, start.y, levels[k - 1]) };
                let to = Point3::new(start.x, start.y, z);
                if to.z < from.z - tol.distance_eps {
                    chords.push(to);
                    cut.push(Motion::plunge(to, ctx.feeds.plunge_feed).with_path(path_id));
                }
                chords.extend(helix_points_with(c, rho, 0.0, 1.0, ccw, z, z, operation.tolerance, !circle.internal).into_iter().skip(1));
                cut.extend(full_circle(c, rho, 0.0, ccw, z, feed, path_id));
            }
        }
        ctx.certify_3d(&chords, false, &mut report, runner)?;
        let end = cut.last().and_then(|m| m.end()).unwrap_or(start);
        // Approach: rapid above the start, rapid to the feed height, plunge to the top.
        // Rapids stop above the stock over the hole, not only above its top.
        let feed_z = ctx.heights.feed.max(ctx.material_over(&params.depths, input, top) + operation.linking.safe_distance).min(ctx.heights.clearance);
        motions.push(Motion::rapid(Point3::new(start.x, start.y, ctx.heights.clearance)).with_path(path_id));
        if feed_z < ctx.heights.clearance {
            motions.push(Motion::rapid(Point3::new(start.x, start.y, feed_z)).with_path(path_id));
        }
        ctx.certify_3d(&[Point3::new(start.x, start.y, feed_z), start], false, &mut report, runner)?;
        motions.push(Motion::plunge(start, ctx.feeds.plunge_feed).with_path(path_id));
        motions.extend(cut);
        // Leave the wall: back to the centre (hole) or straight out (boss), then retract.
        let exit = if circle.internal { Point3::new(c.x, c.y, end.z) } else { Point3::new(end.x + (end.x - c.x) / rho * r, end.y + (end.y - c.y) / rho * r, end.z) };
        ctx.certify_3d(&[end, exit], false, &mut report, runner)?;
        motions.push(Motion::new(MotionKind::LeadOut { to: exit }, Feed::Rate(ctx.feeds.lead_feed())).with_path(path_id));
        motions.push(Motion::retract(Point3::new(exit.x, exit.y, ctx.heights.clearance), ctx.feeds.retract_feed.map_or(Feed::RAPID, Feed::Rate)).with_path(path_id));
        passes += 1;
        runner.work(1, "bore", ci as u64 + 1, input.circles.len() as u64)?;
    }
    ctx.direct(motions, passes, report)
}
