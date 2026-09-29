use super::*;
use super::chain::weld_chain;
use super::geometry::{ellipse_point, sample_bezier_polyline, Loop, SegKind, Segment};

/// Move segments into their planned closed loops (each index claimed at most
/// once across the plans; open-chain segments stay behind — their geometry is
/// already emitted as the path or is a secondary open chain, exactly the established
/// `openChains` that never join the profile).
pub(super) fn assemble_loops(segments: Vec<Segment>, plans: Vec<Vec<(usize, bool)>>) -> Vec<Loop> {
    let mut slots: Vec<Option<Segment>> = segments.into_iter().map(Some).collect();
    let mut loops = Vec::with_capacity(plans.len());
    for plan in plans {
        let mut entries = Vec::with_capacity(plan.len());
        for (index, reversed) in plan {
            let segment = slots[index]
                .take()
                .expect("each segment is claimed by exactly one loop");
            entries.push((segment, reversed));
        }
        let polygon = sample_loop_polygon(&entries);
        // `loop_id` is resolved over the whole assembled set once, downstream
        // (`loop_ids::resolve`) — assembly itself decides no identity.
        loops.push(Loop {
            entries,
            polygon,
            loop_id: None,
        });
    }
    loops
}

/// Sample a loop's boundary into a UV polygon (arc interiors approximated) for
/// point-in-polygon containment tests. Uses each directed segment's start plus a
/// few interior samples on arcs.
fn sample_loop_polygon(entries: &[(Segment, bool)]) -> Vec<[f64; 2]> {
    let mut polygon = Vec::new();
    for (segment, reversed) in entries {
        let (from, to) = if *reversed {
            (segment.b, segment.a)
        } else {
            (segment.a, segment.b)
        };
        polygon.push(from);
        match &segment.kind {
            SegKind::Arc { center, radius, .. } => {
                // A few interior points so a bulging arc's extent is represented.
                let a0 = (from[1] - center[1]).atan2(from[0] - center[0]);
                let a1 = (to[1] - center[1]).atan2(to[0] - center[0]);
                let mut delta = a1 - a0;
                while delta > PI {
                    delta -= TAU;
                }
                while delta < -PI {
                    delta += TAU;
                }
                for step in 1..4 {
                    let angle = a0 + delta * (step as f64 / 4.0);
                    polygon.push([
                        center[0] + radius * angle.cos(),
                        center[1] + radius * angle.sin(),
                    ]);
                }
            }
            SegKind::Bezier { controls } => {
                // Interior samples (direction-aware) so the spline's bulge is
                // represented in containment tests.
                let mut samples = sample_bezier_polyline(controls, 8);
                samples.remove(0);
                samples.pop();
                if *reversed {
                    samples.reverse();
                }
                polygon.extend(samples);
            }
            SegKind::EllipseHalf {
                center,
                major,
                minor,
                upper,
            } => {
                // Interior angle samples over the half's span, direction-aware.
                let base = if *upper { PI } else { 0.0 };
                let mut samples: Vec<[f64; 2]> = (1..8)
                    .map(|step| {
                        ellipse_point(*center, *major, *minor, base + PI * (step as f64 / 8.0))
                    })
                    .collect();
                if *reversed {
                    samples.reverse();
                }
                polygon.extend(samples);
            }
            SegKind::Line => {}
        }
    }
    polygon
}

/// Group the closed loops into disjoint REGIONS by even-odd containment depth:
/// a depth-EVEN loop is a region's OUTER boundary; a depth-ODD loop is a hole
/// of the region whose outer directly contains it (so an island inside a hole
/// becomes its own region — the even-odd material rule). Containment is decided
/// by one representative point per loop against the other loops' sampled
/// polygons (valid sketch loops never cross, so one point decides). Regions
/// come back in original loop order, each `[outer, holes…]` (holes in loop
/// order too) — the single-outer case is byte-identical to the old outer+holes
/// classification.
pub(super) fn classify_regions(loops: Vec<Loop>) -> Result<Vec<Vec<Loop>>, String> {
    let count = loops.len();
    if count == 1 {
        return Ok(vec![loops]);
    }
    let representative: Vec<[f64; 2]> = loops
        .iter()
        .map(|l| l.polygon.first().copied().unwrap_or([0.0, 0.0]))
        .collect();
    // contains[i][j]: loop i lies inside loop j.
    let mut contains = vec![vec![false; count]; count];
    for i in 0..count {
        for j in 0..count {
            if i != j && point_in_polygon(representative[i], &loops[j].polygon) {
                contains[i][j] = true;
            }
        }
    }
    let depth: Vec<usize> = (0..count)
        .map(|i| contains[i].iter().filter(|inside| **inside).count())
        .collect();
    // Plan the regions on indices: each depth-even loop opens a region; each
    // depth-odd loop joins the region of its DIRECT parent (the containing
    // loop exactly one level up — unique when the loops properly nest).
    let mut plans: Vec<(usize, Vec<usize>)> = (0..count)
        .filter(|&i| depth[i] % 2 == 0)
        .map(|i| (i, Vec::new()))
        .collect();
    for hole in 0..count {
        if depth[hole] % 2 == 0 {
            continue;
        }
        let parent = (0..count)
            .find(|&j| contains[hole][j] && depth[j] + 1 == depth[hole])
            .ok_or("sketch: loop containment is inconsistent (crossing loops?)")?;
        plans
            .iter_mut()
            .find(|(outer, _)| *outer == parent)
            .expect("a hole's direct parent is depth-even, so it opened a region")
            .1
            .push(hole);
    }
    // Move the loops into their planned regions.
    let mut slots: Vec<Option<Loop>> = loops.into_iter().map(Some).collect();
    let mut regions = Vec::with_capacity(plans.len());
    for (outer, holes) in plans {
        let mut region = Vec::with_capacity(holes.len() + 1);
        region.push(slots[outer].take().expect("each loop lands in one region"));
        for hole in holes {
            region.push(slots[hole].take().expect("each loop lands in one region"));
        }
        regions.push(region);
    }
    Ok(regions)
}

/// Standard ray-cast point-in-polygon (UV).
fn point_in_polygon(point: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let count = polygon.len();
    if count < 3 {
        return false;
    }
    let mut j = count - 1;
    for i in 0..count {
        let pi = polygon[i];
        let pj = polygon[j];
        let intersects = (pi[1] > point[1]) != (pj[1] > point[1])
            && point[0] < (pj[0] - pi[0]) * (point[1] - pi[1]) / (pj[1] - pi[1]) + pi[0];
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Emit a loop's curves in the plane frame, head-to-tail and welded exactly
/// closed ([`weld_chain`]), with the parallel per-curve source edge names.
pub(super) fn emit_loop(loop_data: &Loop, frame: &Frame) -> Result<ProfileLoop, String> {
    let mut curves = Vec::with_capacity(loop_data.entries.len());
    let mut edge_names = Vec::with_capacity(loop_data.entries.len());
    for (segment, reversed) in &loop_data.entries {
        curves.push(segment.to_curve(frame, *reversed)?);
        edge_names.push(segment.name.clone());
    }
    weld_chain(&mut curves, true);
    Ok(ProfileLoop {
        curves,
        edge_names,
        loop_id: loop_data.loop_id,
    })
}
