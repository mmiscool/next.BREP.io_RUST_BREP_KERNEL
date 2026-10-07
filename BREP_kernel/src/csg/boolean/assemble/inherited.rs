//! The DERIVED inherited-trim challenger (D, 2026-10-05; root's authorization
//! "derived inherited challenger", corrected per root's 4a6f508 review). A
//! boolean output use of an operand's own edge carries the operand's trim
//! restricted to the piece (`csg/edge_split.rs::subcurve_by_fraction`):
//! nothing refits it, and when the file's trim does not track its own edge,
//! the output inherits that. On fixture 25 the operand edges 5 and 9 (shipped
//! edges 2 and 3) stand 3.388e-4 / 8.965e-4 off their curved carriers and their
//! four output uses track their edges only to 1.422e-4 .. 4.262e-4 (a4c84871,
//! every-use control).
//!
//! What may change: only the OUTPUT body's coedge pcurves of an inherited
//! edge. Raw operands, edge curves, ranges, senses, carriers, vertices and
//! endpoints are never written. Inherited means: the edge was created from an
//! operand BOUNDARY source (`Assembler::edge_boundary_key`: the operand and
//! the operand-side edge id AFTER `apply_edge_splits`, printed per decision
//! with the edge's `name`, which splits carry with `_1`/`_2` suffixes back to
//! the raw operand edge) and is not a constructed section
//! (`Assembler::imprint_edges`).
//!
//! ELIGIBLE: two uses on distinct faces, BOTH carriers non-affine (an affine
//! carrier has its exact affine lane and is not this challenger's).
//!
//! READ: every use by `pcurve::common_station_read` (one shared admitted foot
//! per probe, finite readings or it errors) at the probes
//! [`use_probes`]: the pcurves' native spans (endpoints, quarters, 16 interior
//! points) and exact knots, and the EDGE's own native spans clipped to its
//! range and mapped through the use's sense (exact knots, quarters, 16
//! interior points).
//!
//! PAIR, atomic: when EITHER use reads over the bar, BOTH uses get a candidate
//! (the range lane on the edge's own range and sense, aligned to the old
//! trim's period branch, on [0, 1]) and BOTH must be admitted by
//! `pcurve::floor_admits` on the common reader's tracking, deviation and
//! pointwise excess (`pcurve::common_station_read_with_excess`): every reading
//! finite, strictly better tracking, and EITHER a deviation no worse than the
//! old trim's OR the candidate within the original 1e-7 floor on both tracking
//! and excess (the standoff itself is diagnostic only), or the pair is left
//! unchanged. Both on the bar: nothing is done.
//!
//! LOOP and BODY, fail closed: per loop the worst joint gap, read finite at
//! every joint ([`checked_loop_gap`]), may not grow past `max(before, bar)`
//! with every accepted override in place; per body every closed shell's trim
//! vector-area residual must be finite before and after and no worse
//! ([`shell_residuals_admit`]), or every refit trim goes back. An unread
//! reading anywhere declines (nothing in here aborts the operation). The
//! caller's stage transaction (validate, then restore) covers the result too.
//! `BREP_INHERITED_TRIM_REFIT=0` is a diagnostic A/B hatch: nothing is read
//! or changed.

use super::joints::{replaced_trim, restore_trims, ReplacedTrim};
use super::vertices::align_period_branch;
use crate::{BrepSolid, KernelRefusal, KernelTolerances, NurbsCurve, Vec3};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};

/// One use's candidate outcome.
#[derive(Debug)]
enum UseRead {
    /// An admitted candidate, with (old tracking, new tracking).
    Admitted(NurbsCurve, f64, f64),
    /// No admitted candidate (named).
    Declined(String),
}

/// The pair decision: every use must carry an admitted candidate, or nothing
/// changes. Returns the indices (into `reads`) to replace.
fn edge_decision(reads: &[UseRead]) -> Option<Vec<usize>> {
    if reads.is_empty() || reads.iter().any(|read| matches!(read, UseRead::Declined(_))) {
        return None;
    }
    Some((0..reads.len()).collect())
}


/// The loop admission: both gaps read finite, and the gap does not grow past
/// `max(before, bar)`.
fn loop_admits(before: Option<f64>, after: Option<f64>, bar: f64) -> bool {
    matches!((before, after), (Some(b), Some(a)) if b.is_finite() && a.is_finite() && a <= b.max(bar))
}

/// The body admission on the closed shells' trim residual lengths, in shell
/// order: the same number, every one finite before and after, none worse.
fn shell_residuals_admit(before: &[f64], after: &[f64]) -> bool {
    before.len() == after.len() && before.iter().zip(after).all(|(b, a)| b.is_finite() && a.is_finite() && a <= b)
}

/// The worst joint gap around `face`'s loop `loop_index` with `overrides`
/// (coedge index -> pcurve) in place, or None when any joint cannot be read
/// finite (unread is never clean).
fn checked_loop_gap(face: &crate::FaceRecord, loop_index: usize, overrides: &HashMap<usize, NurbsCurve>) -> Option<f64> {
    let coedges = &face.loops.get(loop_index)?.coedges;
    let n = coedges.len();
    let mut worst = 0.0f64;
    for i in 0..n {
        let here = overrides.get(&i).unwrap_or(&coedges[i].pcurve);
        let next = overrides.get(&((i + 1) % n)).unwrap_or(&coedges[(i + 1) % n].pcurve);
        let [_, p1] = here.domain().ok()?;
        let [q0, _] = next.domain().ok()?;
        let a = here.evaluate(p1).ok()?;
        let b = next.evaluate(q0).ok()?;
        let ia = face.surface.evaluate_extended(a.x, a.y).ok()?;
        let ib = face.surface.evaluate_extended(b.x, b.y).ok()?;
        let gap = ia.sub(ib).length();
        if !gap.is_finite() {
            return None;
        }
        worst = worst.max(gap);
    }
    Some(worst)
}

/// `curve` reparameterised affinely onto [0, 1] (the same geometry: the
/// knots mapped, the controls kept), so a restricted trim reads at coedge
/// fractions as the common reader assumes.
fn on_unit_domain(curve: &NurbsCurve) -> Option<NurbsCurve> {
    let [q0, q1] = curve.domain().ok()?;
    if !(q1 > q0) {
        return None;
    }
    if q0 == 0.0 && q1 == 1.0 {
        return Some(curve.clone());
    }
    let knots = curve.knots.iter().map(|k| (k - q0) / (q1 - q0)).collect();
    NurbsCurve::new(curve.degree, knots, curve.control_points.clone()).ok()
}

/// The edge's own native probes as coedge fractions: its knots clipped to
/// `[t0, t1]` (with the range ends), and on EVERY span between them the
/// quarter points and 16 interior points; mapped through the use's sense.
fn edge_native_fractions(edge: &crate::topology::EdgeRecord, forward: bool) -> Vec<f64> {
    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    let span = edge.t1 - edge.t0;
    if !(span.abs() > 0.0) {
        return Vec::new();
    }
    let mut knots: Vec<f64> = edge.curve.knots.iter().copied().filter(|k| *k >= low && *k <= high).collect();
    knots.push(low);
    knots.push(high);
    knots.sort_by(f64::total_cmp);
    knots.dedup();
    let locals: Vec<f64> = [0.25, 0.5, 0.75].into_iter().chain((1..=16).map(|i| i as f64 / 17.0)).collect();
    let mut ts: Vec<f64> = knots.clone();
    for pair in knots.windows(2).filter(|w| w[1] > w[0]) {
        for &local in &locals {
            ts.push(pair[0] + (pair[1] - pair[0]) * local);
        }
    }
    let mut fractions: Vec<f64> = ts
        .into_iter()
        .map(|t| {
            let along = (t - edge.t0) / span;
            if forward { along } else { 1.0 - along }
        })
        .filter(|f| f.is_finite() && (0.0..=1.0).contains(f))
        .collect();
    fractions.sort_by(f64::total_cmp);
    fractions.dedup();
    fractions
}

/// The probes a use is read at: `curves`' native spans and exact knots plus
/// the edge's own native fractions; finite, in [0, 1], sorted, deduplicated.
fn use_probes(curves: &[&NurbsCurve], edge: &crate::topology::EdgeRecord, forward: bool) -> Option<Vec<f64>> {
    let mut probes = crate::pcurve::native_probe_fractions(curves).ok()?;
    for curve in curves {
        probes.extend(curve.knots.iter().copied());
    }
    probes.extend(edge_native_fractions(edge, forward));
    probes.retain(|f| f.is_finite() && (0.0..=1.0).contains(f));
    probes.sort_by(f64::total_cmp);
    probes.dedup();
    Some(probes)
}

fn edge_point_at(edge: &crate::topology::EdgeRecord, forward: bool) -> impl Fn(f64) -> Result<Vec3, String> + '_ {
    move |fraction: f64| {
        let t = if forward { edge.t0 + fraction * (edge.t1 - edge.t0) } else { edge.t1 - fraction * (edge.t1 - edge.t0) };
        edge.curve.evaluate(t)
    }
}

/// The old trim's tracking at its use's probes, on [0, 1]; None if unread.
fn old_tracking(face: &crate::FaceRecord, edge: &crate::topology::EdgeRecord, coedge: &crate::topology::CoedgeRecord) -> Option<(NurbsCurve, f64)> {
    let old = on_unit_domain(&coedge.pcurve)?;
    let probes = use_probes(&[&old], edge, coedge.forward)?;
    let evaluate = edge_point_at(edge, coedge.forward);
    let ((tracking, _), _) = crate::pcurve::common_station_read(&face.surface, &evaluate, &old, &old, &probes).ok()?;
    tracking.is_finite().then_some((old, tracking))
}

fn candidate(face: &crate::FaceRecord, edge: &crate::topology::EdgeRecord, coedge: &crate::topology::CoedgeRecord, old: &NurbsCurve, bar: f64) -> UseRead {
    let forward = coedge.forward;
    let Ok(fit) = crate::fit_pcurve_on_surface_range(&face.surface, &edge.curve, edge.t0, edge.t1, forward, bar) else {
        return UseRead::Declined("range lane refused".into());
    };
    if !(fit.report.residual.is_finite() && fit.report.off_surface.is_finite()) {
        return UseRead::Declined("range lane report not finite".into());
    }
    let standoff = fit.report.off_surface;
    let Ok(new) = align_period_branch(&face.surface, fit.curve, old) else { return UseRead::Declined("period branch".into()) };
    if !matches!(new.domain(), Ok([a, b]) if a == 0.0 && b == 1.0) {
        return UseRead::Declined("candidate not on [0, 1]".into());
    }
    let Some(probes) = use_probes(&[old, &new], edge, forward) else { return UseRead::Declined("common probes unreadable".into()) };
    let evaluate = edge_point_at(edge, forward);
    match crate::pcurve::common_station_read_with_excess(&face.surface, &evaluate, old, &new, &probes) {
        Ok((o, n)) if crate::pcurve::floor_admits(o, n) => UseRead::Admitted(new, o[0], n[0]),
        Ok((o, n)) => UseRead::Declined(format!(
            "tracking {:.3e} -> {:.3e}, deviation {:.3e} -> {:.3e}, excess {:.3e} -> {:.3e} (standoff {standoff:.3e}, diagnostic)",
            o[0], n[0], o[1], n[1], o[2], n[2]
        )),
        Err(error) => UseRead::Declined(format!("common read: {error}")),
    }
}

/// Debug only. Per SHELL (by id), an UNWEIGHTED mean of the old and new trim
/// images of that shell's moves in `order`, 33 fractions per move: a rough
/// location of where that shell's strips sit, not an area-weighted centroid.
/// A shell whose images cannot all be read finite maps to None.
fn strip_centroids(solid: &BrepSolid, order: &[(usize, usize, usize)], accepted: &HashMap<(usize, usize), HashMap<usize, NurbsCurve>>) -> HashMap<u64, Option<Vec3>> {
    let mut faces: Vec<(u64, &crate::FaceRecord)> = Vec::new();
    for shell in &solid.shells {
        for face in &shell.faces {
            faces.push((shell.id, face));
        }
    }
    let mut sums: HashMap<u64, Option<(Vec3, usize)>> = HashMap::default();
    for &(face_index, loop_index, coedge_index) in order {
        let Some(&(shell_id, face)) = faces.get(face_index) else { continue };
        let entry = sums.entry(shell_id).or_insert(Some((Vec3::default(), 0)));
        let read = (|| -> Option<(Vec3, usize)> {
            let old = &face.loops.get(loop_index)?.coedges.get(coedge_index)?.pcurve;
            let new = accepted.get(&(face_index, loop_index))?.get(&coedge_index)?;
            let ([o0, o1], [n0, n1]) = (old.domain().ok()?, new.domain().ok()?);
            let (mut sum, mut count) = (Vec3::default(), 0usize);
            for k in 0..=32 {
                let f = k as f64 / 32.0;
                for (curve, a, b) in [(old, o0, o1), (new, n0, n1)] {
                    let uv = curve.evaluate(a + (b - a) * f).ok()?;
                    let point = face.surface.evaluate_extended(uv.x, uv.y).ok()?;
                    if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite()) {
                        return None;
                    }
                    sum = sum.add(point);
                    count += 1;
                }
            }
            Some((sum, count))
        })();
        *entry = match (*entry, read) {
            (Some((total, n)), Some((sum, count))) => Some((total.add(sum), n + count)),
            _ => None,
        };
    }
    let finite = |p: Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    sums.into_iter()
        .map(|(id, sum)| (id, sum.and_then(|(total, n)| (n > 0).then(|| total.scale(1.0 / n as f64))).filter(|mean| finite(*mean))))
        .collect()
}

/// Debug only. One closed shell as the trace reads it at ONE moment: its trim
/// residual vector, edge length (the shell's edge set, for correspondence)
/// and the volume reference `mass_properties::shell_volume_reference` takes
/// (the first face's first trim start; mirrored here, not shared), None when
/// that reference cannot be read finite.
struct ShellTraceRead {
    closed: bool,
    residual: Vec3,
    edge_length: f64,
    reference: Option<Vec3>,
}

/// Debug only. Every shell's [`ShellTraceRead`], by shell id, or None when
/// the scan is unreadable.
fn shell_trace_reads(solid: &BrepSolid) -> Option<HashMap<u64, ShellTraceRead>> {
    let report = crate::shell_vector_areas(solid);
    if !report.unreadable.is_empty() {
        return None;
    }
    let finite = |p: Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    Some(
        report
            .shells
            .iter()
            .map(|shell| {
                // `mass_properties::shell_volume_reference` takes the FIRST
                // face's first trim start when it reads finite, and otherwise
                // falls back (that face's surface midpoint, then a control
                // point) before trying the next face. Only its first branch is
                // mirrored: the reference is claimed only when the first
                // face's first trim start reads finite; otherwise unread.
                let reference = solid.shells.iter().find(|s| s.id == shell.shell).and_then(|s| {
                    let face = s.faces.first()?;
                    let coedge = face.loops.first()?.coedges.first()?;
                    let [q0, _] = coedge.pcurve.domain().ok()?;
                    let uv = coedge.pcurve.evaluate(q0).ok()?;
                    face.surface.evaluate(uv.x, uv.y).ok()
                });
                (shell.shell, ShellTraceRead { closed: shell.closed, residual: shell.trim_residual, edge_length: shell.edge_length, reference: reference.filter(|c| finite(*c)) })
            })
            .collect(),
    )
}

/// Debug only, deciding nothing, and NOT a volume. A body whose trims do not
/// close has a divergence-theorem volume that depends on its reference point
/// c: V(c) = V(0) - c . R / 3, with R the trims' vector-area residual. A move
/// changes R by its strips' vector area dR = R_after - R_before, and V(0) by
/// the strips' own flux, which for thin strips is APPROXIMATELY x . dR / 3
/// with x where the strips sit. So the volume the kernel reports moves by
/// APPROXIMATELY x . dR / 3 - (c_after . R_after - c_before . R_before) / 3.
/// Printed per closed shell, matched BY SHELL ID with the same closedness
/// and the bit-identical edge length (the same edge set: this stage writes
/// pcurves only), every read finite or the shell is reported unread: R
/// before and after, dR, c before and after, x (the UNWEIGHTED image mean of
/// [`strip_centroids`]) and that approximate predictor, labelled as such.
/// Every derived term must be finite, or nothing is predicted. The 'after'
/// read is the TRIAL state, before the challenger's shell gate, which may
/// roll it back. The reference mirrors only the first branch of
/// `shell_volume_reference` (first face, first trim start); otherwise
/// unread. Omitted and named: the strips' flux covariance about x, the
/// closure's chord terms and the quadrature error. It infers nothing about
/// the physical volume.
fn volume_reference_trace(before: &Option<HashMap<u64, ShellTraceRead>>, after: &Option<HashMap<u64, ShellTraceRead>>, centroids: &HashMap<u64, Option<Vec3>>) {
    let (Some(before), Some(after)) = (before, after) else {
        eprintln!("inherited: volume-reference trace: the closure scan is unreadable before or after; nothing predicted");
        return;
    };
    let finite = |p: Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    let mut ids: Vec<u64> = before.keys().copied().collect();
    ids.sort_unstable();
    for id in ids {
        let b = &before[&id];
        if !b.closed {
            continue;
        }
        let Some(a) = after.get(&id) else {
            eprintln!("inherited: volume-reference trace shell {id}: absent after the moves; unread");
            continue;
        };
        if !b.edge_length.is_finite() || a.closed != b.closed || a.edge_length.to_bits() != b.edge_length.to_bits() {
            eprintln!("inherited: volume-reference trace shell {id}: closedness or edge set differs before/after (edge length {:e} vs {:e}); unread", b.edge_length, a.edge_length);
            continue;
        }
        let centroid = centroids.get(&id).copied().flatten();
        match (b.reference, a.reference, centroid) {
            (Some(cb), Some(ca), Some(x)) if finite(b.residual) && finite(a.residual) => {
                let dr = a.residual.sub(b.residual);
                let strips = x.dot(dr) / 3.0;
                let reference_term = (ca.dot(a.residual) - cb.dot(b.residual)) / 3.0;
                let predicted = strips - reference_term;
                if !(finite(dr) && strips.is_finite() && reference_term.is_finite() && predicted.is_finite()) {
                    eprintln!("inherited: volume-reference trace shell {id}: a derived term is non-finite; nothing predicted");
                    continue;
                }
                eprintln!(
                    "inherited: volume-reference trace shell {id} (APPROXIMATE, not a volume; 'after' is the TRIAL state before the shell gate, which may roll it back): R {:?} -> {:?}, dR {dr:?}; reference c {cb:?} -> {ca:?}{}; strip image mean (unweighted) x {x:?}; x.dR/3 {strips:.6e} - (c_after.R_after - c_before.R_before)/3 {reference_term:.6e} = predicted change {predicted:.6e}; NOT included: the strips' flux covariance (sum of (x_i - x).dA_i, uncontrolled), the closure's chord terms and the vector-area quadrature error",
                    b.residual, a.residual,
                    if cb.x.to_bits() == ca.x.to_bits() && cb.y.to_bits() == ca.y.to_bits() && cb.z.to_bits() == ca.z.to_bits() { " (unchanged)" } else { " (MOVED)" }
                );
            }
            _ => eprintln!("inherited: volume-reference trace shell {id}: a residual, reference or strip location is unread or non-finite; nothing predicted"),
        }
    }
}

/// Run the challenger, reading the `BREP_INHERITED_TRIM_REFIT` hatch.
pub(in crate::boolean) fn challenge_inherited_trims(
    solid: &mut BrepSolid,
    tolerance: f64,
    boundary: &HashMap<u64, (u8, u64)>,
    constructed: &HashSet<u64>,
) -> Result<Vec<ReplacedTrim>, KernelRefusal> {
    let enabled = std::env::var("BREP_INHERITED_TRIM_REFIT").as_deref() != Ok("0");
    challenge_inherited_trims_with(solid, tolerance, boundary, constructed, enabled)
}

/// [`challenge_inherited_trims`] with the hatch's value passed in. Returns
/// every replaced trim, for the stage's restore. Never refuses: every unread
/// reading declines.
pub(super) fn challenge_inherited_trims_with(
    solid: &mut BrepSolid,
    tolerance: f64,
    boundary: &HashMap<u64, (u8, u64)>,
    constructed: &HashSet<u64>,
    enabled: bool,
) -> Result<Vec<ReplacedTrim>, KernelRefusal> {
    if !enabled {
        return Ok(Vec::new());
    }
    let policy = KernelTolerances::for_solid(solid, 1e-7);
    let bar = policy.model.min(tolerance.max(1e-12));
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok() || std::env::var("BREP_DEBUG_JOINTS").is_ok();
    let faces: Vec<&crate::FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    let mut uses: HashMap<u64, Vec<(usize, usize, usize)>> = HashMap::default();
    for (face_index, face) in faces.iter().enumerate() {
        for (loop_index, loop_record) in face.loops.iter().enumerate() {
            for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                uses.entry(coedge.edge_id).or_default().push((face_index, loop_index, coedge_index));
            }
        }
    }
    let mut edges: Vec<&crate::topology::EdgeRecord> = solid.edges.iter().collect();
    edges.sort_by_key(|edge| edge.id);
    // Accepted overrides, per loop: (face index, loop index) -> coedge index -> pcurve.
    let mut accepted: HashMap<(usize, usize), HashMap<usize, NurbsCurve>> = HashMap::default();
    let mut order: Vec<(usize, usize, usize)> = Vec::new();
    for edge in edges {
        if edge.degenerate || !(edge.t1 != edge.t0) || constructed.contains(&edge.id) {
            continue;
        }
        let Some(&(operand, source_edge)) = boundary.get(&edge.id) else { continue };
        let Some(edge_uses) = uses.get(&edge.id) else { continue };
        if edge_uses.len() != 2 || edge_uses[0].0 == edge_uses[1].0 {
            continue;
        }
        if edge_uses.iter().any(|&(face_index, _, _)| faces[face_index].surface.is_affine().unwrap_or(true)) {
            continue;
        }
        let olds: Vec<Option<(NurbsCurve, f64)>> = edge_uses.iter().map(|u| old_tracking(faces[u.0], edge, &faces[u.0].loops[u.1].coedges[u.2])).collect();
        if olds.iter().any(|old| old.is_none()) {
            if debug {
                eprintln!("inherited: edge {} {:?} (operand {operand} edge {source_edge}) declined: an old trim is unread", edge.id, edge.name);
            }
            continue;
        }
        if olds.iter().flatten().all(|(_, tracking)| *tracking <= bar) {
            continue;
        }
        // Either use over the bar: BOTH get a candidate and both must be admitted.
        let reads: Vec<UseRead> = edge_uses
            .iter()
            .zip(&olds)
            .map(|(u, old)| candidate(faces[u.0], edge, &faces[u.0].loops[u.1].coedges[u.2], &old.as_ref().expect("read above").0, bar))
            .collect();
        let Some(taken) = edge_decision(&reads) else {
            if debug {
                eprintln!("inherited: edge {} {:?} (operand {operand} edge {source_edge}) pair left unchanged: {reads:?}", edge.id, edge.name);
            }
            continue;
        };
        // The loop gate, with every accepted override plus this pair, read finite.
        let mut trial = accepted.clone();
        for &index in &taken {
            let (face_index, loop_index, coedge_index) = edge_uses[index];
            if let UseRead::Admitted(pcurve, _, _) = &reads[index] {
                trial.entry((face_index, loop_index)).or_default().insert(coedge_index, pcurve.clone());
            }
        }
        let mut opens = None;
        for &index in &taken {
            let (face_index, loop_index, _) = edge_uses[index];
            let before = checked_loop_gap(faces[face_index], loop_index, &HashMap::default());
            let after = checked_loop_gap(faces[face_index], loop_index, &trial[&(face_index, loop_index)]);
            if !loop_admits(before, after, bar) {
                opens = Some((faces[face_index].id, before, after));
                break;
            }
        }
        if let Some((face_id, before, after)) = opens {
            if debug {
                eprintln!("inherited: edge {} {:?} (operand {operand} edge {source_edge}) pair left unchanged: face {face_id}'s loop gap {before:?} -> {after:?}", edge.id, edge.name);
            }
            continue;
        }
        if debug {
            for &index in &taken {
                if let UseRead::Admitted(_, old, new) = &reads[index] {
                    eprintln!("inherited: edge {} {:?} (operand {operand} edge {source_edge}) use on face {}: tracking {old:.3e} -> {new:.3e}", edge.id, edge.name, faces[edge_uses[index].0].id);
                }
            }
        }
        for &index in &taken {
            order.push(edge_uses[index]);
        }
        accepted = trial;
    }
    drop(faces);
    if order.is_empty() {
        return Ok(Vec::new());
    }
    let closed_residuals = |solid: &BrepSolid| -> Option<Vec<f64>> {
        let report = crate::shell_vector_areas(solid);
        report.unreadable.is_empty().then(|| report.shells.iter().filter(|shell| shell.closed).map(|shell| shell.trim_residual.length()).collect())
    };
    let before = closed_residuals(solid);
    // Debug only, deciding nothing: per-shell strip locations and every
    // shell's residual vector and volume reference BEFORE the moves, for the
    // volume-reference trace below.
    let traced = debug.then(|| (strip_centroids(solid, &order, &accepted), shell_trace_reads(solid)));
    let mut replaced = Vec::with_capacity(order.len());
    for (face_index, loop_index, coedge_index) in order {
        let pcurve = accepted[&(face_index, loop_index)][&coedge_index].clone();
        let face = solid.shells.iter_mut().flat_map(|shell| &mut shell.faces).nth(face_index).expect("face index");
        let previous = std::mem::replace(&mut face.loops[loop_index].coedges[coedge_index].pcurve, pcurve);
        replaced.push(replaced_trim(face_index, loop_index, coedge_index, previous));
    }
    let after = closed_residuals(solid);
    if let Some((centroids, reads_before)) = &traced {
        volume_reference_trace(reads_before, &shell_trace_reads(solid), centroids);
    }
    let admitted = matches!((&before, &after), (Some(b), Some(a)) if shell_residuals_admit(b, a));
    if !admitted {
        if debug {
            eprintln!("inherited: shell trim residuals {before:?} -> {after:?} not admitted; the {} refit trim(s) go back", replaced.len());
        }
        restore_trims(solid, replaced);
        return Ok(Vec::new());
    }
    Ok(replaced)
}

