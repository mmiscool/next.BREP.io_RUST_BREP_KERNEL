//! Multi-axis finishing (lane C10b, rows 1-3): any 3-axis finishing
//! toolpath (parallel, scallop, contour, flow, ...) re-run with a per-CL
//! tool axis.
//!
//! For every cutting point of the inner toolpath the contact point and
//! surface normal are recovered with the vertical tool-frame drop; the axis
//! comes from the [`AxisMode`] (normal with lead/tilt, toward/away from a
//! point or line, fixed from vertical), is clamped to `max_from_vertical_deg`
//! and smoothed to `max_axis_step_deg` per sample; the cutter-location point
//! is rebuilt from the contact for the cutter's corner radius and then
//! dropped along the tilted axis against the full protected mesh, so the
//! emitted tip is the exact contact position (an `exact-contact` approximation
//! records how far the analytic guess moved). Shank and holder are cleared by
//! tilting for reach (row 2, `avoid_collisions`); with a machine chain every
//! pose is solved: poses outside the rotary limits are re-tilted about the
//! travel tangent until a solution exists, otherwise reported as a
//! `machine-limit` unreachable region; singular poses are counted and
//! reported as a `singular-region` warning (row 3). Heights are distances
//! along the tool axis.

use crate::cutter::{ToolPart, ToolProfile};
use crate::kinematics::sequence::choose_pose;
use crate::kinematics::{rotate, Chain, KinematicsError, KinematicsOptions};
use crate::model::*;
use crate::multiaxis::contact::drop_along;
use crate::multiaxis::orientation::{clamp_from_vertical, smooth_axes};
use crate::multiaxis::links::traverse;
use crate::multiaxis::tilt::{avoid_collisions, ClPose, TiltOptions};
use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};

/// The machine the poses must be reachable on.
pub struct MachineCheck<'a> {
    pub chain: &'a Chain,
    pub options: KinematicsOptions,
}

/// Cutter-location point for a contact point and axis (see the spec, section 7).
fn cl_from_contact(cc: Point3, n: Point3, a: Point3, big_r: f64, r: f64) -> Point3 {
    let m = n.sub(a.scale(n.dot(a))).normalized().unwrap_or(Point3::ZERO);
    cc.add(n.scale(r)).add(m.scale(big_r - r)).sub(a.scale(r))
}

/// Generate multi-axis finishing from `inner`, a usable 3-axis finishing
/// toolpath for the same operation geometry (vertical tool axis).
pub fn generate(
    operation: &Operation,
    params: &MultiAxisFinishParams,
    inner: &Toolpath,
    cutter: &Cutter,
    holder: Option<&Holder>,
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    machine: Option<&MachineCheck<'_>>,
    default_heights: &Heights,
    default_feeds: &FeedsSpeeds,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Toolpath, CamError> {
    operation.validate().map_err(CamError::InvalidInput)?;
    if !operation.enabled {
        return Err(CamError::InvalidInput("operation is disabled".into()));
    }
    params.validate().map_err(CamError::InvalidInput)?;
    cutter.validate()?;
    tolerance.validate()?;
    if !inner.report.is_usable() {
        return Err(CamError::InvalidInput("multi-axis finish: the inner toolpath is not usable".into()));
    }
    let corner = match cutter {
        Cutter::Flat(_) => 0.0,
        Cutter::Ball(_) => cutter.radius(),
        Cutter::Bull(b) => cutter.radius() - b.flat_radius(),
        _ => return Err(CamError::UnsupportedCutter { strategy: "multi-axis finish".into(), cutter: cutter.kind().as_str().into() }),
    };
    let heights = operation.heights.as_ref().unwrap_or(default_heights);
    let feeds = operation.feeds.as_ref().unwrap_or(default_feeds);
    heights.validate().map_err(CamError::InvalidInput)?;
    feeds.validate().map_err(CamError::InvalidInput)?;
    if mesh.is_empty() {
        return Err(CamError::EmptyTarget { operation_id: operation.id.to_string() });
    }
    if let Some(m) = machine {
        m.options.validate()?;
    }
    let big_r = cutter.radius() + operation.stock_to_leave.radial.max(0.0);
    let profile = ToolProfile::new(cutter, holder, operation.tolerance);
    let mut out = Toolpath::new(operation.id.clone(), operation.tool.clone());
    out.report.counts.triangles = mesh.triangle_count() as u64;
    // Runs of consecutive cutting points of the inner path.
    let mut runs: Vec<Vec<Point3>> = Vec::new();
    let mut current: Vec<Point3> = Vec::new();
    let mut at: Option<Point3> = None;
    for m in &inner.motions {
        match &m.kind {
            MotionKind::Cut { to } | MotionKind::Plunge { to } | MotionKind::LeadIn { to } | MotionKind::LeadOut { to } => {
                if current.is_empty() {
                    if let Some(start) = at {
                        if matches!(m.kind, MotionKind::Cut { .. }) {
                            current.push(start);
                        }
                    }
                }
                current.push(*to);
                at = Some(*to);
            }
            MotionKind::ArcCw { .. } | MotionKind::ArcCcw { .. } => {
                return Err(CamError::InvalidInput("multi-axis finish: linearise arcs in the inner toolpath first".into()));
            }
            other => {
                if current.len() >= 2 {
                    runs.push(std::mem::take(&mut current));
                } else {
                    current.clear();
                }
                if let Some(e) = other.end() {
                    at = Some(e);
                }
            }
        }
    }
    if current.len() >= 2 {
        runs.push(current);
    }
    if runs.is_empty() {
        return Err(CamError::InvalidInput("multi-axis finish: the inner toolpath has no cutting runs".into()));
    }
    let mut no_contact = 0u64;
    let mut corrected = 0u64;
    let mut max_correction = 0.0f64;
    let mut smoothed_total = 0usize;
    let mut clamped_total = 0usize;
    let mut degenerate = 0u64;
    let mut singular = 0u64;
    let mut first_singular: Option<Point3> = None;
    let mut limit_blocked = 0u64;
    let mut first_blocked: Option<Point3> = None;
    let mut retilted = 0u64;
    let mut unproven_links = 0u64;
    let mut prev_angles: Option<Vec<f64>> = None;
    let max_step = params.max_axis_step_deg.to_radians();
    let max_vert = params.max_from_vertical_deg.to_radians();
    let top_z = mesh.bounds.max.z + heights.clearance;
    let mut prev_retract: Option<ClPose> = None;
    // The last segment's end pose, its proven exit lengths and the index of
    // its retract motion (a break's link may move that retract).
    let mut prev_exit: Option<(ClPose, Vec<f64>, usize)> = None;
    // Trimmed CL tips (UnreachablePolicy::Trim), in path order.
    let mut trimmed: Vec<Point3> = Vec::new();
    let mut unproven_trim_legs = 0u64;
    let mut refined_chords = 0u64;
    // Pieces past a break no proven link reaches (Trim), by first tip.
    let mut unlinkable: Vec<Point3> = Vec::new();
    let mut split_chords = 0u64;
    let mut fallbacks = 0u64;
    let mut unproven_traverses = 0u64;
    runner.stage("maf-runs", "Multi-axis finishing runs", 0, runs.len() as u64)?;
    for (ri, run) in runs.iter().enumerate() {
        runner.work(1, "maf-runs", ri as u64, runs.len() as u64)?;
        // Contacts and normals from the vertical drop at each inner tip.
        let mut contacts: Vec<(Point3, Point3)> = Vec::with_capacity(run.len());
        for tip in run {
            let d = drop_along(mesh, index, cutter, *tip, Point3::Z, 2.0 * cutter.radius() + operation.tolerance, tolerance, runner)?;
            out.report.counts.samples += 1;
            out.report.counts.candidates += d.candidates;
            match d.contact.and_then(|c| Some((c.point?, c.triangle_id?))) {
                Some((p, id)) => contacts.push((p, mesh.triangles[id as usize].normal)),
                None => no_contact += 1,
            }
        }
        if contacts.len() < 2 {
            continue;
        }
        // Axes by mode, clamped, smoothed.
        let mut axes: Vec<Point3> = Vec::with_capacity(contacts.len());
        for (k, (cc, n)) in contacts.iter().enumerate() {
            let prev = contacts[k.saturating_sub(1)].0;
            let next = contacts[(k + 1).min(contacts.len() - 1)].0;
            let d = next.sub(prev);
            let t = d.sub(n.scale(d.dot(*n))).normalized().unwrap_or_else(|| n.cross(Point3::X).normalized().unwrap_or(Point3::Y));
            let a = match params.mode.axis_at(*cc, *n, t) {
                Some(a) => a,
                None => {
                    degenerate += 1;
                    *n
                }
            };
            let (a, clamped) = clamp_from_vertical(a, max_vert);
            if clamped {
                clamped_total += 1;
            }
            axes.push(a);
        }
        smoothed_total += smooth_axes(&mut axes, max_step);
        // Exact CL along each axis: analytic guess, then the tool-frame drop.
        let mut poses: Vec<ClPose> = Vec::with_capacity(axes.len());
        for ((cc, n), a) in contacts.iter().zip(&axes) {
            let guess = cl_from_contact(*cc, *n, *a, big_r, corner);
            let d = drop_along(mesh, index, cutter, guess, *a, big_r + operation.tolerance, tolerance, runner)?;
            out.report.counts.samples += 1;
            out.report.counts.candidates += d.candidates;
            let tip = guess.add(a.scale(d.offset.max(-big_r)));
            if d.offset.abs() > operation.tolerance {
                corrected += 1;
                max_correction = max_correction.max(d.offset.abs());
            }
            poses.push(ClPose { tip, axis: *a });
        }
        // Reach: shank and holder by tilting about the travel tangent.
        let tilt_opts = TiltOptions {
            max_tilt_deg: params.reach_tilt_deg,
            parts: vec![ToolPart::Shank, ToolPart::Holder],
            gouge_tolerance: Some(operation.tolerance),
            reach: cutter.radius(),
            pivot_offset: corner,
            fail_unreachable: params.unreachable == UnreachablePolicy::Fail,
            ..Default::default()
        };
        let tilted = avoid_collisions(mesh, index, &profile, Some(cutter), &poses, &tilt_opts, tolerance, &mut out.report, runner)?;
        // Trim: an unreachable pose ends the segment; the next starts past
        // it with its own approach, so nothing cuts across the gap.
        let trim = params.unreachable == UnreachablePolicy::Trim;
        // (poses, entered past a trim, left at a trim)
        let mut segments: Vec<(Vec<ClPose>, bool, bool)> = Vec::new();
        let mut cl: Vec<ClPose> = Vec::with_capacity(tilted.len());
        let mut after_trim = false;
        for (k, t) in tilted.iter().enumerate() {
            if t.unreachable {
                if trim {
                    trimmed.push(t.pose.tip);
                    if !cl.is_empty() {
                        segments.push((std::mem::take(&mut cl), after_trim, true));
                    }
                    after_trim = true;
                }
                continue;
            }
            let mut pose = t.pose;
            if let Some(m) = machine {
                // Machine limits and singularities (row 3).
                let tangent = {
                    let prev = poses[k.saturating_sub(1)].tip;
                    let next = poses[(k + 1).min(poses.len() - 1)].tip;
                    let d = next.sub(prev);
                    d.sub(pose.axis.scale(d.dot(pose.axis))).normalized()
                };
                match choose_pose(m.chain, pose.tip, Some(pose.axis), prev_angles.as_deref(), k, &m.options) {
                    Ok((p, _)) => {
                        if p.singular {
                            singular += 1;
                            first_singular.get_or_insert(pose.tip);
                        }
                        prev_angles = Some(p.angles_deg);
                    }
                    Err(KinematicsError::LimitViolation { .. }) | Err(KinematicsError::Unreachable { .. }) => {
                        let mut found: Option<(ClPose, Vec<f64>)> = None;
                        if let Some(tn) = tangent {
                            let steps = (params.reach_tilt_deg / tilt_opts.step_deg).floor() as usize;
                            let pivot = pose.tip.add(pose.axis.scale(corner));
                            'search: for s in 1..=steps {
                                for sign in [1.0, -1.0] {
                                    let a = rotate(tn, (sign * s as f64 * tilt_opts.step_deg).to_radians(), pose.axis);
                                    let guess = pivot.sub(a.scale(corner));
                                    let Ok((p, _)) = choose_pose(m.chain, guess, Some(a), prev_angles.as_deref(), k, &m.options) else { continue };
                                    let d = drop_along(mesh, index, cutter, guess, a, cutter.radius(), tolerance, runner)?;
                                    if d.offset > operation.tolerance {
                                        continue;
                                    }
                                    let c = crate::multiaxis::contact::clearance_along(mesh, index, &profile, &tilt_opts.parts, guess, a, tilt_opts.slab, tolerance)?;
                                    if c.hit.is_some() {
                                        continue;
                                    }
                                    found = Some((ClPose { tip: guess, axis: a }, p.angles_deg));
                                    break 'search;
                                }
                            }
                        }
                        match found {
                            Some((p, angles)) => {
                                retilted += 1;
                                pose = p;
                                prev_angles = Some(angles);
                            }
                            None => {
                                limit_blocked += 1;
                                first_blocked.get_or_insert(pose.tip);
                                out.report.empty_regions.push(EmptyRegion { reason: "machine-limit".into(), target: None });
                                continue;
                            }
                        }
                    }
                    Err(e) => return Err(e.into()),
                }
            }
            cl.push(pose);
        }
        segments.push((cl, after_trim, false));
        // Every chord between consecutive poses is certified against the
        // part: one that dips below the contact surface by more than the
        // tolerance gets a refined pose at its middle (recursively); one that
        // still dips splits the segment there (sweep finding S1).
        let mut certified: Vec<(Vec<ClPose>, bool, bool)> = Vec::with_capacity(segments.len());
        for (cl, entered, left) in segments {
            let pieces = certify_chords(mesh, index, cutter, &cl, operation.tolerance, &mut refined_chords, tolerance, runner)?;
            let n = pieces.len();
            split_chords += n.saturating_sub(1) as u64;
            for (i, piece) in pieces.into_iter().enumerate() {
                certified.push((piece, if i == 0 { entered } else { true }, if i + 1 == n { left } else { true }));
            }
        }
        for (cl, entered_past_trim, left_at_trim) in certified {
        if cl.len() < 2 {
            continue;
        }
        // At a trim break (or a split chord) the tool may sit deep in the
        // part. It leaves and re-enters along its own axis up to the safe Z
        // when that leg is clear, else by the clearance distance (the
        // traverse then lifts vertically and proves its own legs); a leg
        // neither way proves is counted and fails the report.
        // Proven exit lengths (or the first try, unproven: `false`).
        let exits = |p: ClPose, at_trim: bool, runner: &mut Runner<'_>| -> Result<(Vec<f64>, bool), CamError> {
            if !at_trim {
                return Ok((vec![heights.clearance], true));
            }
            let off = ClPose { tip: p.tip.add(p.axis.scale(operation.tolerance.max(tolerance.distance_eps))), axis: p.axis };
            // Up to the safe Z, the clearance, then short hops of a few
            // radii (across a step the source climbed in place the traverse's
            // direct chord between two short exits is often the proven one).
            let mut tries = Vec::new();
            if p.axis.z > 1e-6 {
                tries.push(heights.clearance.max((top_z - p.tip.z) / p.axis.z));
            }
            for d in [heights.clearance, 4.0 * cutter.radius(), 2.0 * cutter.radius()] {
                if !tries.iter().any(|t: &f64| (t - d).abs() < 1e-9) {
                    tries.push(d);
                }
            }
            let mut proven = Vec::new();
            for &d in &tries {
                if crate::multiaxis::links::chord_clear(mesh, index, cutter, off, ClPose { tip: p.tip.add(p.axis.scale(d)), axis: p.axis }, cutter.radius(), tolerance, runner)? {
                    proven.push(d);
                }
            }
            if proven.is_empty() {
                return Ok((vec![tries[0]], false));
            }
            Ok((proven, true))
        };
        // Emit: rapid at clearance along the first axis, plunge, cuts, retract.
        let path_id = out.report.counts.passes as u32;
        let first = cl[0];
        let (ins, ins_proven) = exits(first, entered_past_trim, runner)?;
        let last = cl[cl.len() - 1];
        let (outs, outs_proven) = exits(last, left_at_trim, runner)?;
        // Trim: a piece with no proven way in or out is left uncut.
        if trim && (!ins_proven || !outs_proven) {
            unlinkable.push(first.tip);
            out.report.empty_regions.push(EmptyRegion { reason: "unlinkable".into(), target: None });
            continue;
        }
        let mut approach = ClPose { tip: first.tip.add(first.axis.scale(ins[0])), axis: first.axis };
        // Across a break, the first pair of proven exits whose direct chord
        // is clear: the previous retract (the last motion) is moved to it.
        let mut paired = false;
        if let (true, Some((last, outs, idx))) = (entered_past_trim, prev_exit.as_ref()) {
            'pair: for &o in outs {
                for &i in &ins {
                    let r = ClPose { tip: last.tip.add(last.axis.scale(o)), axis: last.axis };
                    let a = ClPose { tip: first.tip.add(first.axis.scale(i)), axis: first.axis };
                    if crate::multiaxis::links::chord_clear(mesh, index, cutter, r, a, cutter.radius(), tolerance, runner)? {
                        out.motions[*idx].kind = MotionKind::Retract { to: r.tip };
                        prev_retract = Some(r);
                        approach = a;
                        paired = true;
                        break 'pair;
                    }
                }
            }
        }
        match prev_retract {
            None => {
                let mut m = Motion::rapid(approach.tip).with_path(path_id);
                m.tool_axis = Some(first.axis);
                out.motions.push(m);
            }
            Some(prev) => {
                let t = traverse(mesh, index, cutter, prev, approach, cutter.radius(), top_z, path_id, tolerance, runner)?;
                // Trim: a piece no proven link reaches is left uncut and
                // reported, never entered through material.
                if t.unproven && !paired && trim {
                    unlinkable.push(first.tip);
                    out.report.empty_regions.push(EmptyRegion { reason: "unlinkable".into(), target: None });
                    continue;
                }
                if t.fallback {
                    fallbacks += 1;
                }
                if t.unproven {
                    unproven_traverses += 1;
                }
                out.motions.extend(t.motions);
            }
        }
        if !ins_proven {
            unproven_trim_legs += 1;
        }
        let mut m = Motion::plunge(first.tip, feeds.plunge_feed).with_path(path_id);
        m.tool_axis = Some(first.axis);
        out.motions.push(m);
        for p in &cl[1..] {
            let mut m = Motion::cut(p.tip, feeds.cutting_feed).with_path(path_id);
            m.tool_axis = Some(p.axis);
            out.motions.push(m);
        }
        if !outs_proven {
            unproven_trim_legs += 1;
        }
        let retract_at = last.tip.add(last.axis.scale(outs[0]));
        for (p, a) in [(approach.tip, first.axis), (retract_at, last.axis)] {
            let d = drop_along(mesh, index, cutter, p, a, 0.0, tolerance, runner)?;
            if d.offset > tolerance.distance_eps {
                unproven_links += 1;
            }
        }
        let mut m = Motion::retract(retract_at, Feed::RAPID).with_path(path_id);
        m.tool_axis = Some(last.axis);
        out.motions.push(m);
        prev_retract = Some(ClPose { tip: retract_at, axis: last.axis });
        prev_exit = Some((last, outs, out.motions.len() - 1));
        out.report.counts.passes += 1;
        }
    }
    if !unlinkable.is_empty() {
        let shown: Vec<String> = unlinkable.iter().take(20).map(|p| format!("({:.4}, {:.4}, {:.4})", p.x, p.y, p.z)).collect();
        let more = if unlinkable.len() > shown.len() { format!(" and {} more", unlinkable.len() - shown.len()) } else { String::new() };
        out.report.warn(
            Diagnostic::new(
                "unlinkable-trimmed",
                format!("{} path pieces have no proven link in or out and are left uncut, starting at {}{more}", unlinkable.len(), shown.join(", ")),
            )
            .at(unlinkable[0]),
        );
    }
    if split_chords > 0 {
        out.report.warn(Diagnostic::new(
            "chord-split",
            format!("{split_chords} chords between contacts would cut below the surface even refined (a step the source climbs in place): the path breaks there, retracts and re-enters"),
        ));
    }
    if refined_chords > 0 {
        out.report.approximations.push(ApproximationLimit {
            kind: "chord-refined".into(),
            bound: Some(operation.tolerance),
            detail: Some(format!("{refined_chords} poses inserted where a chord between contacts dipped below the surface")),
        });
    }
    if unproven_trim_legs > 0 {
        out.report.fail(Diagnostic::new("link-unproven", format!("{unproven_trim_legs} retract or re-entry moves along the tool axis at a trim break touch material")));
    }
    if !trimmed.is_empty() {
        let shown: Vec<String> = trimmed.iter().take(20).map(|p| format!("({:.4}, {:.4}, {:.4})", p.x, p.y, p.z)).collect();
        let more = if trimmed.len() > shown.len() { format!(" and {} more", trimmed.len() - shown.len()) } else { String::new() };
        let mut d = Diagnostic::new(
            "unreachable-trimmed",
            format!(
                "{} CL points collide with the shank or holder at every tilt within {}\u{b0}: the path breaks there, retracts and re-enters; material is left at {}{more}",
                trimmed.len(),
                params.reach_tilt_deg,
                shown.join(", ")
            ),
        );
        d = d.at(trimmed[0]);
        out.report.warn(d);
    }
    if no_contact > 0 {
        out.report.warn(Diagnostic::new("no-contact", format!("{no_contact} inner points had no surface contact and were dropped")));
    }
    if degenerate > 0 {
        out.report.warn(Diagnostic::new("axis-degenerate", format!("{degenerate} points lie on the axis point/line; the normal was used")));
    }
    if smoothed_total > 0 || clamped_total > 0 {
        out.report.approximations.push(ApproximationLimit {
            kind: "axis-smoothed".into(),
            bound: Some(params.max_axis_step_deg),
            detail: Some(format!("{smoothed_total} axes smoothed, {clamped_total} clamped to {}° from vertical", params.max_from_vertical_deg)),
        });
    }
    if corrected > 0 {
        out.report.approximations.push(ApproximationLimit {
            kind: "exact-contact".into(),
            bound: Some(max_correction),
            detail: Some(format!("{corrected} analytic CL guesses moved along the axis to the mesh contact")),
        });
    }
    if singular > 0 {
        let mut d = Diagnostic::new("singular-region", format!("{singular} poses inside the machine's singular cone (held rotary)"));
        if let Some(p) = first_singular {
            d = d.at(p);
        }
        out.report.warn(d);
    }
    if retilted > 0 {
        out.report.warn(Diagnostic::new("retilted-for-limits", format!("{retilted} poses re-tilted inside the machine's rotary limits")));
    }
    if unproven_links > 0 {
        out.report.warn(Diagnostic::new("link-unproven", format!("{unproven_links} approach/retract positions touch material; retract vertically before them")));
    }
    if fallbacks > 0 {
        out.report.warn(Diagnostic::new("link-fallback-clearance", format!("{fallbacks} traverses between runs were unproven and use the vertical clearance")));
    }
    if unproven_traverses > 0 {
        out.report.fail(Diagnostic::new("link-unproven", format!("{unproven_traverses} vertical-clearance traverses still touch material")));
    }
    if limit_blocked > 0 {
        let mut d = Diagnostic::new("machine-limit", format!("{limit_blocked} poses have no orientation inside the machine's rotary limits"));
        if let Some(p) = first_blocked {
            d = d.at(p);
        }
        out.report.fail(d);
    }
    if out.motions.is_empty() && out.report.errors.is_empty() {
        out.report.fail(Diagnostic::new("empty-after-clipping", "no multi-axis finishing run could be machined"));
    }
    out.report.counts.motions = out.motions.len() as u64;
    out.report.finish();
    Ok(out)
}

/// How deep a dipping chord is halved before the segment is split there.
const CHORD_REFINE_DEPTH: u32 = 8;

/// Whether the straight move from `a` to `b` (axis interpolated) keeps the
/// cutting part within `limit` of the surface: sampled at most `limit` apart
/// (at least 0.01, at most a quarter radius) and held to half of it.
fn chord_dips(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    cutter: &Cutter,
    a: ClPose,
    b: ClPose,
    limit: f64,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<bool, CamError> {
    // At most `limit` apart (at least 0.01, at most a quarter radius), held to
    // half the limit so a dip between two samples stays within it.
    let step = limit.max(0.01).min((0.25 * cutter.radius()).max(0.01));
    let n = ((a.tip.distance(b.tip) / step).ceil() as usize).clamp(1, 10_000);
    for k in 1..n {
        let f = k as f64 / n as f64;
        let axis = crate::multiaxis::slerp(a.axis, b.axis, f);
        let d = drop_along(mesh, index, cutter, a.tip.lerp(b.tip, f), axis, 0.0, tolerance, runner)?;
        if d.offset > 0.5 * limit {
            return Ok(true);
        }
    }
    Ok(false)
}

/// The poses to insert between `a` and `b` so no chord dips, or `None` when
/// halving `depth` times does not get there.
#[allow(clippy::too_many_arguments)]
fn refine_chord(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    cutter: &Cutter,
    a: ClPose,
    b: ClPose,
    limit: f64,
    depth: u32,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Option<Vec<ClPose>>, CamError> {
    if !chord_dips(mesh, index, cutter, a, b, limit, tolerance, runner)? {
        return Ok(Some(Vec::new()));
    }
    if depth == 0 {
        return Ok(None);
    }
    let axis = crate::multiaxis::slerp(a.axis, b.axis, 0.5);
    let guess = a.tip.lerp(b.tip, 0.5);
    let d = drop_along(mesh, index, cutter, guess, axis, 0.0, tolerance, runner)?;
    let mid = ClPose { tip: guess.add(axis.scale(d.offset.max(0.0))), axis };
    let Some(left) = refine_chord(mesh, index, cutter, a, mid, limit, depth - 1, tolerance, runner)? else { return Ok(None) };
    let Some(right) = refine_chord(mesh, index, cutter, mid, b, limit, depth - 1, tolerance, runner)? else { return Ok(None) };
    let mut out = left;
    out.push(mid);
    out.extend(right);
    Ok(Some(out))
}

/// Certify a segment's chords: refined where a midpoint fixes the dip,
/// split where it cannot be. Every returned piece cuts only certified chords.
#[allow(clippy::too_many_arguments)]
fn certify_chords(
    mesh: &MeshSnapshot,
    index: Option<&dyn TriangleIndex>,
    cutter: &Cutter,
    cl: &[ClPose],
    limit: f64,
    refined: &mut u64,
    tolerance: Tolerance,
    runner: &mut Runner<'_>,
) -> Result<Vec<Vec<ClPose>>, CamError> {
    let mut pieces: Vec<Vec<ClPose>> = Vec::new();
    let Some(&first) = cl.first() else { return Ok(pieces) };
    let mut current = vec![first];
    for w in cl.windows(2) {
        match refine_chord(mesh, index, cutter, w[0], w[1], limit, CHORD_REFINE_DEPTH, tolerance, runner)? {
            Some(inserted) => {
                *refined += inserted.len() as u64;
                current.extend(inserted);
            }
            None => pieces.push(std::mem::take(&mut current)),
        }
        current.push(w[1]);
    }
    pieces.push(current);
    Ok(pieces)
}

