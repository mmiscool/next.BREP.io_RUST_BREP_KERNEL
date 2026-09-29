//! Closing the boundary a shadowed support's removal VACATED on the neighbour
//! that survives it.
//!
//! When two wall offsets cross — a slot narrower than twice the distance, grown
//! outward — each lies wholly inside the other's material, so neither owns any
//! skin and both supports are omitted (`offset_shell/pipeline.rs`). The crossing
//! is trimmed by that omission. What is left is the rim those faces used to cut
//! in a SURVIVING neighbour's trim: a slot that runs out through a face leaves a
//! notch in that face's own loop, the carrier clones the source's trim notch and
//! all, and an outward carrier grows only across the sides of its parameter box,
//! so nothing closes it. Measured on a 2-wide slot through a 20 cube grown by
//! 1.5: 12 one-use edges where the slot crosses both Y faces.
//!
//! The closure is on the OFFSET side: the CARRIER's trim is bridged, and the
//! source face keeps its notch, because that notch is real material boundary.
//!
//! What it is keyed on is OWNERSHIP, per rim: a run of the loop's coedges whose
//! faces were all omitted as shadowed is replaced by the straight span between
//! its ends. A rim whose face owns skin is never bridged, so a T-shaped slot —
//! a narrow throat over a wide pocket — closes the throat and keeps the pocket,
//! which is what the occupancy integral asks for.
//!
//! Two things it will not do, each refused with what it measured:
//! * a bridge that is not the CONTINUATION of the rims either side of the run
//!   (they are not collinear with the chord in the carrier's parameter space);
//! * a carrier whose trim no longer matches its source loop coedge for coedge,
//!   where the correspondence this reads would be a guess.
//!
//! Planar carriers only. On a curved one the image of a parameter-space segment
//! is not the span of an existing rim, which is the thing being restored.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use crate::{NurbsCurve, NurbsSurface};

/// How close to parallel the rims either side of a vacated run must be with the
/// chord that would replace it: `sin θ` of the worst of the two, against the
/// chord's own length. A notch cut square out of a rectangle reads 0.
const CONTINUATION_BAND: f64 = 1e-6;

/// What [`close_vacated_rims`] did to one carrier.
#[derive(Clone, Debug, Default)]
pub(super) struct VacatedRimReport {
    /// Runs bridged.
    pub(super) closed: usize,
    /// Runs left alone, each with what was measured.
    pub(super) declined: Vec<String>,
}

/// Bridge every vacated run in `carrier`'s trim. `omitted` is the set of source
/// faces whose offsets own no skin.
pub(super) fn close_vacated_rims(
    carrier: &mut BrepSolid,
    source_face: &FaceRecord,
    source_faces: &[&FaceRecord],
    omitted: &HashSet<u64>,
) -> Result<VacatedRimReport, KernelRefusal> {
    let mut report = VacatedRimReport::default();
    if omitted.is_empty() {
        return Ok(report);
    }
    let Some(face) = carrier
        .shells
        .first()
        .and_then(|shell| shell.faces.first())
        .cloned()
    else {
        return Ok(report);
    };
    if !face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(report);
    }
    let mut edited = face.clone();
    let edge_by_id = carrier
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect::<HashMap<_, _>>();
    let mut next_edge_id = carrier.edges.iter().map(|edge| edge.id).max().unwrap_or(0) + 1;
    let mut new_edges = Vec::new();
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        let Some(source_loop) = source_face.loops.get(loop_index) else {
            report.declined.push(format!(
                "carrier loop {loop_index} has no source loop to read ownership from"
            ));
            continue;
        };
        let count = loop_record.coedges.len();
        if count != source_loop.coedges.len() {
            report.declined.push(format!(
                "carrier loop {loop_index} walks {count} coedge(s) where its source walks {} — the \
                 correspondence this reads would be a guess",
                source_loop.coedges.len()
            ));
            continue;
        }
        let vacated = source_loop
            .coedges
            .iter()
            .map(|coedge| {
                junction_mate(source_faces, source_face, coedge.edge_id)
                    .is_some_and(|(mate, _)| omitted.contains(&mate.id))
            })
            .collect::<Vec<_>>();
        if vacated.iter().all(|flag| *flag) {
            if vacated.iter().any(|flag| *flag) {
                report.declined.push(format!(
                    "carrier loop {loop_index}: every one of its {count} rims was vacated, so there \
                     is no rim either side to continue"
                ));
            }
            continue;
        }
        let mut replacements = Vec::new();
        for run in maximal_runs(&vacated) {
            match bridge(&loop_record.coedges, &run, count, &edge_by_id) {
                Ok(bridged) => replacements.push((run, bridged)),
                Err(reason) => report
                    .declined
                    .push(format!("carrier loop {loop_index}: {reason}")),
            }
        }
        if replacements.is_empty() {
            continue;
        }
        let (coedges, edges) = rebuild_loop(
            &loop_record.coedges,
            &replacements,
            &edited.surface,
            &mut next_edge_id,
        )?;
        report.closed += replacements.len();
        edited.loops[loop_index].coedges = coedges;
        new_edges.extend(edges);
    }
    if report.closed == 0 {
        return Ok(report);
    }
    carrier.edges.extend(new_edges);
    carrier.shells[0].faces[0] = edited;
    Ok(report)
}

/// Maximal runs of `true`, walked circularly, as `(start, length)`.
///
/// The walk starts at a `false`, because where a loop's coedge list BEGINS is
/// not something the shape controls: a run of vacated rims may sit across that
/// seam, and it is one run either way. Starting from a surviving rim is what
/// makes that true — walking from index 0 and merely skipping index 0 when the
/// list ends vacated finds the rest of the prefix a SECOND time, as a run of its
/// own (`TT.T` read `[(1, 1), (3, 3)]`).
///
/// An all-vacated loop never arrives here: the caller declines it, there being
/// no rim either side to continue.
pub(super) fn maximal_runs(flags: &[bool]) -> Vec<(usize, usize)> {
    let count = flags.len();
    let Some(origin) = (0..count).find(|index| !flags[*index]) else {
        return Vec::new();
    };
    let mut runs = Vec::new();
    let mut step = 0;
    while step < count {
        let index = (origin + step) % count;
        if !flags[index] {
            step += 1;
            continue;
        }
        let mut length = 0;
        while step + length < count && flags[(origin + step + length) % count] {
            length += 1;
        }
        runs.push((index, length));
        step += length;
    }
    runs
}

/// The parameter-space segment that replaces a run: from the end of the rim
/// before it to the start of the rim after it, and only when both are its
/// CONTINUATION.
fn bridge(
    coedges: &[CoedgeRecord],
    (start, length): &(usize, usize),
    count: usize,
    edge_by_id: &HashMap<u64, EdgeRecord>,
) -> Result<Bridge, KernelRefusal> {
    let before_coedge = &coedges[(start + count - 1) % count];
    let after_coedge = &coedges[(start + length) % count];
    let before = before_coedge.pcurve.clone();
    let after = after_coedge.pcurve.clone();
    let [_, before_end] = before.domain().or_refuse(KernelStage::Refine, "domain")?;
    let [after_start, _] = after.domain().or_refuse(KernelStage::Refine, "domain")?;
    let from = before.evaluate(before_end).or_refuse(KernelStage::Refine, "evaluate")?;
    let to = after.evaluate(after_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let chord = to.sub(from);
    let span = chord.length();
    // The same band, because a chord shorter than it has no direction to test
    // against it: the floor is the continuation test's own precondition rather
    // than a second tolerance.
    if span <= CONTINUATION_BAND {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_vacated_rim_closes", format!(
            "the {length} vacated rim(s) from index {start} close on themselves (chord {span:.3e})"
        )));
    }
    let direction = chord.scale(1.0 / span);
    let worst = [
        tangent_at(&before, before_end, true)?,
        tangent_at(&after, after_start, false)?,
    ]
    .into_iter()
    .map(|tangent| tangent.cross(direction).length())
    .fold(0.0f64, f64::max);
    if worst > CONTINUATION_BAND {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_vacated_rim_chord", format!(
            "the {length} vacated rim(s) from index {start} would be bridged by a chord that is not \
             the continuation of the rims either side (worst sin θ {worst:.3e} against a band of \
             {CONTINUATION_BAND:.0e}, chord {span:.4})"
        )));
    }
    // The bridge joins the vertices those two rims already meet the run at, so
    // it re-closes the loop rather than introducing ends of its own.
    let (Some(before_edge), Some(after_edge)) = (
        edge_by_id.get(&before_coedge.edge_id),
        edge_by_id.get(&after_coedge.edge_id),
    ) else {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_vacated_rim_neighbour", format!(
            "the {length} vacated rim(s) from index {start} sit beside a rim whose edge the carrier \
             does not carry"
        )));
    };
    Ok(Bridge {
        pcurve: crate::make_line(from, to).or_refuse(KernelStage::Refine, "make_line")?,
        start_vertex_id: if before_coedge.forward {
            before_edge.end_vertex_id
        } else {
            before_edge.start_vertex_id
        },
        end_vertex_id: if after_coedge.forward {
            after_edge.start_vertex_id
        } else {
            after_edge.end_vertex_id
        },
    })
}

/// One bridge: the parameter-space span and the vertices it re-joins.
struct Bridge {
    pcurve: NurbsCurve,
    start_vertex_id: u64,
    end_vertex_id: u64,
}

/// Unit tangent of a pcurve at one end, `backwards` reading the end that
/// arrives there.
fn tangent_at(curve: &NurbsCurve, parameter: f64, backwards: bool) -> Result<Vec3, KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let step = ((t1 - t0) * 1e-4).max(1e-12);
    let other = if backwards {
        (parameter - step).max(t0)
    } else {
        (parameter + step).min(t1)
    };
    let from = curve.evaluate(parameter).or_refuse(KernelStage::Refine, "evaluate")?;
    let to = curve.evaluate(other).or_refuse(KernelStage::Refine, "evaluate")?;
    let delta = if backwards { from.sub(to) } else { to.sub(from) };
    let length = delta.length();
    if length <= 0.0 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "offset_shell_vacated_rim_direction", "a rim with no direction at its end"));
    }
    Ok(delta.scale(1.0 / length))
}

/// The loop with each run replaced by its bridge, and the edges those bridges
/// need.
fn rebuild_loop(
    coedges: &[CoedgeRecord],
    replacements: &[((usize, usize), Bridge)],
    surface: &NurbsSurface,
    next_edge_id: &mut u64,
) -> Result<(Vec<CoedgeRecord>, Vec<EdgeRecord>), KernelRefusal> {
    let count = coedges.len();
    let mut replaced_at = HashMap::<usize, &Bridge>::default();
    let mut dropped = HashSet::<usize>::default();
    for ((start, length), bridged) in replacements {
        replaced_at.insert(*start, bridged);
        for step in 0..*length {
            dropped.insert((start + step) % count);
        }
    }
    let mut rebuilt = Vec::with_capacity(count);
    let mut edges = Vec::new();
    for (index, coedge) in coedges.iter().enumerate() {
        if let Some(bridged) = replaced_at.get(&index) {
            let pcurve = &bridged.pcurve;
            let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let start = pcurve.evaluate(t0).or_refuse(KernelStage::Refine, "evaluate")?;
            let end = pcurve.evaluate(t1).or_refuse(KernelStage::Refine, "evaluate")?;
            let curve = crate::make_line(
                surface.evaluate(start.x, start.y).or_refuse(KernelStage::Refine, "evaluate")?,
                surface.evaluate(end.x, end.y).or_refuse(KernelStage::Refine, "evaluate")?,
            ).or_refuse(KernelStage::Refine, "make_line")?;
            let [c0, c1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            edges.push(EdgeRecord {
                id: *next_edge_id,
                curve,
                t0: c0,
                t1: c1,
                start_vertex_id: bridged.start_vertex_id,
                end_vertex_id: bridged.end_vertex_id,
                degenerate: false,
                name: None,
            });
            rebuilt.push(CoedgeRecord {
                id: coedge.id,
                edge_id: *next_edge_id,
                forward: true,
                pcurve: pcurve.clone(),
            });
            *next_edge_id += 1;
            continue;
        }
        if dropped.contains(&index) {
            continue;
        }
        rebuilt.push(coedge.clone());
    }
    Ok((rebuilt, edges))
}
