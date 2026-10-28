//! Cutter-location line filter (lane C4, row 1).
//!
//! Sampled passes carry many nearly collinear points. The filter removes the
//! redundant ones so every kept chord stays within `tolerance` of every point
//! it replaces, and it never crosses a motion-kind, feed, path or tool-axis
//! boundary: a run of straight motions of one kind is filtered on its own,
//! arcs, cycles and non-moving motions are left untouched, and the first and
//! last point of every run are always kept. The output is a function of the
//! input alone, so the same stream always filters to the same stream.

use crate::model::{ApproximationLimit, Motion, MotionKind, Point3, Toolpath};
use crate::CamError;

/// Result of filtering one polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct Filtered {
    /// The kept points, in order; always starts and ends with the input's ends.
    pub points: Vec<Point3>,
    /// For every kept point, its index in the input.
    pub kept: Vec<usize>,
    /// How many input points were dropped.
    pub removed: usize,
}

/// Distance from `p` to the segment `a`-`b` (point distance to `a` when the
/// chord is degenerate).
pub fn distance_to_chord(p: Point3, a: Point3, b: Point3) -> f64 {
    let d = b.sub(a);
    let len2 = d.length_squared();
    if len2 <= 0.0 {
        return p.distance(a);
    }
    let t = (p.sub(a).dot(d) / len2).clamp(0.0, 1.0);
    p.distance(a.add(d.scale(t)))
}

/// Filter one polyline. `preserve` lists input indices that must be kept
/// (slider snap points, loop closures); the ends are always kept.
///
/// Walks an anchor forward: a candidate end is accepted while every point
/// between the anchor and it lies within `tolerance` of their chord and none
/// of them is preserved. When a candidate fails, the previous candidate is
/// emitted and becomes the new anchor. Each removed point therefore lies
/// within `tolerance` of the emitted chord that replaced it.
pub fn filter_points(points: &[Point3], tolerance: f64, preserve: &[usize]) -> Result<Filtered, CamError> {
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(CamError::InvalidInput("filter tolerance must be finite and non-negative".into()));
    }
    if let Some(p) = points.iter().find(|p| !p.is_finite()) {
        return Err(CamError::NonFinite { what: format!("filter point {p:?}") });
    }
    if let Some(&i) = preserve.iter().find(|&&i| i >= points.len()) {
        return Err(CamError::InvalidInput(format!("preserved index {i} is outside the polyline")));
    }
    let n = points.len();
    if n < 3 {
        return Ok(Filtered { points: points.to_vec(), kept: (0..n).collect(), removed: 0 });
    }
    let mut keep = vec![false; n];
    keep[0] = true;
    keep[n - 1] = true;
    for &i in preserve {
        keep[i] = true;
    }
    let mut kept = vec![0usize];
    let mut anchor = 0usize;
    while anchor + 1 < n {
        // The next point is always an acceptable chord end; try to extend past it.
        let mut end = anchor + 1;
        while end + 1 < n && !keep[end] {
            let candidate = end + 1;
            let ok = (anchor + 1..candidate)
                .all(|k| distance_to_chord(points[k], points[anchor], points[candidate]) <= tolerance);
            if !ok {
                break;
            }
            end = candidate;
        }
        kept.push(end);
        anchor = end;
    }
    let out: Vec<Point3> = kept.iter().map(|&i| points[i]).collect();
    let removed = n - out.len();
    Ok(Filtered { points: out, kept, removed })
}

/// Straight motions that may be merged with a neighbour of the same kind.
fn is_straight(kind: &MotionKind) -> bool {
    matches!(
        kind,
        MotionKind::Rapid { .. }
            | MotionKind::Plunge { .. }
            | MotionKind::Cut { .. }
            | MotionKind::Retract { .. }
            | MotionKind::LeadIn { .. }
            | MotionKind::LeadOut { .. }
            | MotionKind::Link { .. }
    )
}

/// Two consecutive motions belong to one filterable run.
fn same_run(a: &Motion, b: &Motion) -> bool {
    is_straight(&a.kind)
        && is_straight(&b.kind)
        && a.kind.kind_name() == b.kind.kind_name()
        && a.feed == b.feed
        && a.path_id == b.path_id
        && a.tool_axis == b.tool_axis
}

fn with_end(template: &Motion, to: Point3) -> Motion {
    let kind = match &template.kind {
        MotionKind::Rapid { .. } => MotionKind::Rapid { to },
        MotionKind::Plunge { .. } => MotionKind::Plunge { to },
        MotionKind::Cut { .. } => MotionKind::Cut { to },
        MotionKind::Retract { .. } => MotionKind::Retract { to },
        MotionKind::LeadIn { .. } => MotionKind::LeadIn { to },
        MotionKind::LeadOut { .. } => MotionKind::LeadOut { to },
        MotionKind::Link { .. } => MotionKind::Link { to },
        other => other.clone(),
    };
    Motion { kind, feed: template.feed, tool_axis: template.tool_axis, path_id: template.path_id }
}

/// Filter a motion stream run by run. Returns the filtered stream and the
/// number of motions removed. Motion order, kinds, feeds and path identity
/// are preserved; only interior points of straight runs disappear.
pub fn filter_motions(motions: &[Motion], tolerance: f64) -> Result<(Vec<Motion>, usize), CamError> {
    let mut out: Vec<Motion> = Vec::with_capacity(motions.len());
    let mut removed = 0usize;
    // Position before the current run (the end of the last emitted motion).
    let mut position: Option<Point3> = None;
    let mut i = 0usize;
    while i < motions.len() {
        let m = &motions[i];
        if !is_straight(&m.kind) {
            out.push(m.clone());
            if let Some(p) = m.end() {
                position = Some(p);
            }
            i += 1;
            continue;
        }
        let mut j = i + 1;
        while j < motions.len() && same_run(&motions[j - 1], &motions[j]) {
            j += 1;
        }
        let run = &motions[i..j];
        // The run's polyline: the start position (when known) plus every end.
        let ends: Vec<Point3> = run.iter().map(|m| m.end().expect("straight motions end somewhere")).collect();
        match position {
            Some(start) if run.len() >= 2 => {
                let mut points = Vec::with_capacity(ends.len() + 1);
                points.push(start);
                points.extend(ends.iter().copied());
                let filtered = filter_points(&points, tolerance, &[])?;
                // kept[0] is the start; every later kept index k maps to run[k-1].
                for &k in filtered.kept.iter().skip(1) {
                    out.push(with_end(&run[k - 1], points[k]));
                }
                removed += filtered.removed;
            }
            _ => {
                // No known start (stream begins mid-air) or a single motion:
                // the run's first end is the first point and cannot be removed
                // anyway, so filter from it.
                if run.len() >= 3 {
                    let filtered = filter_points(&ends, tolerance, &[])?;
                    for &k in &filtered.kept {
                        out.push(with_end(&run[k], ends[k]));
                    }
                    removed += filtered.removed;
                } else {
                    out.extend(run.iter().cloned());
                }
            }
        }
        position = ends.last().copied();
        i = j;
    }
    Ok((out, removed))
}

/// Filter a toolpath and record the deviation bound on its report.
pub fn filter_toolpath(path: &Toolpath, tolerance: f64) -> Result<Toolpath, CamError> {
    let (motions, removed) = filter_motions(&path.motions, tolerance)?;
    let mut out = path.clone();
    out.motions = motions;
    out.report.counts.motions = out.motions.len() as u64;
    if removed > 0 {
        out.report.approximations.push(ApproximationLimit {
            kind: "line-filter".into(),
            bound: Some(tolerance),
            detail: Some(format!("{removed} redundant points removed within {tolerance}")),
        });
    }
    Ok(out)
}

