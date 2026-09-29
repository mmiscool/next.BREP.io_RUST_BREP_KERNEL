//! The ONE definition of "re-trim a face whose boundary has already moved".
//!
//! # "Trim" means two different things in this kernel — this is the second one
//!
//! The audit records that "trimming an offset surface to a face" is not one
//! problem but two, with different failure modes and different tolerances:
//!
//! 1. **Parametric-image trimming.** Offset-shell reuses the *source face's own
//!    pcurves verbatim* on the offset carrier (`offset/offset.rs`,
//!    `offset_face_carrier`), which is meaningful only because
//!    `interpolate_tensor` guarantees the offset shares the source's knots and
//!    weights. Where the image is not enough there is a whole rebuild ladder
//!    (`offset/offset_shell/carrier_rebuild.rs`) that re-derives a carrier's
//!    topology from the *surface's own parameter domain* — it never reads an
//!    updated edge curve, and `rebuild_carrier_full_domain` deliberately
//!    *discards* the source trim's interior loops.
//! 2. **Re-trimming after re-intersection.** Push-face, face-move and the
//!    direct-edit heals recompute the 3D boundary first (by closed-form
//!    surface∩surface intersection, or by rigid translation), and then have to
//!    make the carrier and every pcurve agree with the boundary they were
//!    handed.
//!
//! **This module is (2) and only (2).** Nothing here belongs to (1), and the
//! two must not be merged behind one name: (1) trusts a shared basis and works
//! in parameter space; (2) trusts nothing and re-derives parameter space from
//! 3D. `carrier_rebuild.rs` stays where it is for exactly that reason.
//!
//! # The shape all re-trims share
//!
//! Every caller in the tree did the same three things in the same order:
//!
//! * **(a) sample the already-updated boundary** — [`boundary_samples`];
//! * **(b) grow the carrier to cover those samples** — *not* shared, because
//!   the growth is exact per carrier type and each caller owns a different one
//!   (an axis-aligned box in the plane's own frame; an axial prolongation of a
//!   ruled revolution; an axial prolongation of a partial-sweep revolution).
//!   The audit's slice entry asks for exactly this — "unify the *signature*,
//!   keep the per-carrier growth strategy pluggable" — so the solid-side driver
//!   [`retrim_face_in_solid`] takes the growth as a callback;
//! * **(c) rebuild every coedge pcurve from the updated edge curves** —
//!   [`rebuild_loop_pcurves`].
//!
//! # Conventions, made explicit rather than baked in
//!
//! Slice 1 found that a shared evaluator with one hard-coded orientation
//! convention would have silently moved the blend march (audit §9.6.2). The
//! same discipline applies here: every convention these helpers carried
//! implicitly is now a named, documented parameter.
//!
//! * **Boundary sampling is 5 points per edge over `[t0, t1]`, every loop, and
//!   it does NOT skip degenerate edges.** All three original copies agreed on
//!   this; two *other* 5-point boundary loops in the tree
//!   (`feature_pipeline/features/common.rs` and `solvers/assembly_resolve.rs`,
//!   both computing a plane frame's AABB centre) *do* skip degenerate edges.
//!   Those are a different question and are deliberately not routed here.
//! * **Every pcurve is subrange-aware.** A full-domain edge takes the exact
//!   whole-curve build; a strict subrange is fitted over exactly `[t0, t1]`.
//!   The planar lane used to map every edge's WHOLE curve, and this note used
//!   to defend that asymmetry as pre-existing. It was a defect: a cap face
//!   carrying a second blend's trimmed edges got pcurves spanning the whole
//!   curve, and deleting one of two chamfers was refused with the pcurve off
//!   its edge by exactly the chamfer size (2026-09-16). One lane now.
//! * **The refusal prefix is the caller's `op` string**, so every message keeps
//!   the operation name it has always reported under.
//!
//! # Why this lives under `offset/`
//!
//! Most consumers do not offset anything — `delete_face_and_heal`, the open and
//! closed heals, and `move_faces` are the majority. The home follows the
//! offset-unification plan, which names this module and places it beside slice
//! 1's [`crate::OffsetEvaluator`] (`offset/point.rs`); the owner's thesis names
//! *trimming offset surfaces to faces* as the shared machinery, and the
//! cross-feature trim slices that come next all live in `offset/`. A move to a
//! carrier-neutral home is a rename, not a redesign.
//!
//! # Provenance
//!
//! Each item below records the `file:line` whose body it is. Every one is
//! bit-identical to the code it replaced; the only edits were the `op` prefix
//! becoming a parameter and the carrier growth becoming a callback.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::topology::{EdgeRecord, FaceRecord};
use crate::{
    build_pcurve_on_surface, build_pcurve_on_surface_range, make_plane, BrepSolid, KnotVector,
    NurbsSurface, Vec3,
};
use rustc_hash::FxHashMap as HashMap;

/// Below this, a cross product / determinant / direction cosine is treated as
/// degenerate. Absolute, not scale-relative — as it has always been.
///
/// Was `edit/direct_edit/geom.rs:3`, which re-exports this so its plane-pair
/// and line-plane closed forms keep the single definition.
pub(crate) const PARALLEL_EPS: f64 = 1e-9;

/// An oriented orthonormal frame on a planar carrier: `origin` on the plane,
/// `u_dir`/`v_dir` spanning it, and `normal = u_dir × v_dir` **matching the
/// surface's own normal** so a face's `same_sense` survives a rebuild.
///
/// Was `edit/direct_edit/geom.rs:5-11`.
#[derive(Clone, Copy)]
pub(crate) struct Plane {
    pub(crate) origin: Vec3,
    pub(crate) u_dir: Vec3,
    pub(crate) v_dir: Vec3,
    pub(crate) normal: Vec3,
}

fn surface_domain_mid(surface: &NurbsSurface) -> Result<(f64, f64), KernelRefusal> {
    let ku = KnotVector::new(surface.knots_u.clone(), surface.degree_u).or_refuse(KernelStage::Refine, "new")?;
    let kv = KnotVector::new(surface.knots_v.clone(), surface.degree_v).or_refuse(KernelStage::Refine, "new")?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    Ok(((u0 + u1) * 0.5, (v0 + v1) * 0.5))
}

/// Recognise a planar carrier and return an orthonormal frame whose normal
/// matches the surface normal (so face `same_sense` is preserved on rebuild).
/// `op` prefixes the refusal messages so every operation reports under its own
/// name.
///
/// Planarity is decided on the **control net**: every control point must lie
/// within `tolerance` of the plane through the domain-midpoint derivative
/// frame. That is a *structural* test, and it is not the same question as
/// `offset_shell::rim_welds::planar_surface_frame`, which samples a 7×7 grid of
/// evaluated points and returns a normal of arbitrary sign. The two are
/// deliberately NOT merged: this one's normal is orientation-bearing (its whole
/// point is that a rebuilt face keeps its sense), that one's is not, and this
/// one refuses where that one returns `None`.
///
/// Was `edit/direct_edit/geom.rs:99-134`.
pub(crate) fn plane_of_surface(
    surface: &NurbsSurface,
    tolerance: f64,
    op: &str,
) -> Result<Plane, KernelRefusal> {
    let (u_mid, v_mid) = surface_domain_mid(surface)?;
    let derivatives = surface.derivatives(u_mid, v_mid, 1).or_refuse(KernelStage::Refine, "derivatives")?;
    let origin = derivatives[0][0];
    let du = derivatives[1][0];
    let dv = derivatives[0][1];
    let raw_normal = du.cross(dv);
    if raw_normal.length() <= PARALLEL_EPS {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "retrim_degenerate_surface", format!("{op}: degenerate face surface")));
    }
    let normal = raw_normal.normalized().or_refuse(KernelStage::Refine, "normalized")?;
    // Confirm the whole control net is coplanar; otherwise it is a genuinely
    // curved carrier that this planar slice does not extend.
    for row in &surface.control_points {
        for control in row {
            let point = control.point().or_refuse(KernelStage::Refine, "point")?;
            if point.sub(origin).dot(normal).abs() > tolerance {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "retrim_curved_face", format!(
                    "{op}: face is not planar (curved neighbours are deferred in this slice)"
                )));
            }
        }
    }
    let u_dir = du.normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let v_dir = normal.cross(u_dir).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    Ok(Plane {
        origin,
        u_dir,
        v_dir,
        normal,
    })
}

/// The already-updated boundary of `face`, sampled so a carrier can be grown to
/// cover it: **5 points per coedge**, uniformly over the edge's `[t0, t1]`,
/// visiting **every loop** so holes are carried.
///
/// Conventions this pins, because all three original copies shared them and a
/// future caller must not silently pick differently:
///
/// * degenerate edges are **not** skipped — a pole placeholder contributes its
///   (collapsed) samples like any other edge;
/// * the samples come from `edges`, the caller's *updated* edge table, never
///   from the face's current pcurves — that is the whole point of a re-trim;
/// * a coedge whose edge is missing from `edges` is a refusal, prefixed `op`.
///
/// Was the sampling loop of `edit/direct_edit/delete_face.rs:83-95`,
/// `edit/direct_edit/face_offset.rs:892-902` and
/// `edit/direct_edit/face_move.rs:589-599` — three copies whose only textual
/// difference was the `op` literal and whether the samples were folded into a
/// uv box on the spot (the planar one) or collected (the two ruled ones).
/// `min`/`max` are order-independent, so folding after collection is exact.
pub(crate) fn boundary_samples(
    face: &FaceRecord,
    edges: &HashMap<u64, EdgeRecord>,
    op: &str,
) -> Result<Vec<Vec3>, KernelRefusal> {
    let mut points: Vec<Vec3> = Vec::new();
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            let edge = edges
                .get(&coedge.edge_id)
                .ok_or_else(|| format!("{op}: missing edge {}", coedge.edge_id)).or_refuse(KernelStage::Refine, "retrim_missing_edge")?;
            for step in 0..=4 {
                let t = edge.t0 + (edge.t1 - edge.t0) * (step as f64 / 4.0);
                points.push(edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?);
            }
        }
    }
    Ok(points)
}

/// Rebuild every coedge pcurve of `face` on `surface`, from the updated edge
/// curves in `edges`. Every loop is visited, so holes carry.
///
/// A full-domain edge maps its WHOLE curve (`build_pcurve_on_surface`),
/// reversed for a backward coedge — the exact build. An edge that is a strict
/// SUBRANGE of its curve's domain is fitted over exactly `[t0, t1]` with
/// `build_pcurve_on_surface_range`, `tolerance` passed through.
///
/// Both halves are load-bearing. On a ruled wall, a conic ARC that is a partial
/// circle of a full-domain circle curve would otherwise get a pcurve laid across
/// the entire circle and read as an antipodal miss. On a plane, where the affine
/// parameterization makes the whole-curve pcurve the right SHAPE, it is still
/// the wrong RANGE: a cap carrying another blend's trimmed edges failed
/// validation by exactly the trimmed length.
///
/// Was the pcurve loop of `edit/direct_edit/delete_face.rs:107-118` (then
/// whole-curve only), `edit/direct_edit/face_offset.rs:905-935` and
/// `edit/direct_edit/face_move.rs:604-634`.
pub(crate) fn rebuild_loop_pcurves(
    face: &mut FaceRecord,
    edges: &HashMap<u64, EdgeRecord>,
    surface: &NurbsSurface,
    tolerance: f64,
    op: &str,
) -> Result<(), KernelRefusal> {
    for loop_record in &mut face.loops {
        for coedge in &mut loop_record.coedges {
            let edge = edges
                .get(&coedge.edge_id)
                .ok_or_else(|| format!("{op}: missing edge {}", coedge.edge_id)).or_refuse(KernelStage::Refine, "retrim_missing_edge")?;
            let [d0, d1] = edge.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let span = (d1 - d0).max(1e-12);
            let is_subrange =
                (edge.t0 - d0).abs() > 1e-9 * span || (edge.t1 - d1).abs() > 1e-9 * span;
            coedge.pcurve = if is_subrange {
                build_pcurve_on_surface_range(
                    surface,
                    &edge.curve,
                    edge.t0,
                    edge.t1,
                    coedge.forward,
                    tolerance,
                ).or_refuse(KernelStage::Refine, "build_pcurve_on_surface_range")?
            } else {
                let mut pcurve = build_pcurve_on_surface(surface, &edge.curve).or_refuse(KernelStage::Refine, "build_pcurve_on_surface")?;
                if !coedge.forward {
                    pcurve = pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
                }
                pcurve
            };
        }
    }
    Ok(())
}

/// Re-trim a PLANAR face: rebuild its carrier to cover its (possibly grown)
/// boundary loop and recompute every coedge pcurve on the fresh carrier.
///
/// The growth strategy is this function's own, and is the reason it is not
/// expressed through [`retrim_face_in_solid`]: the new carrier is the boundary's
/// axis-aligned box **in the plane's own frame**, outset by
/// `max(0.25 · longest extent, scale · 1e-3)`. `scale` is the model scale
/// (`crate::solid_scale`), so the floor is size-relative and the 0.25 term
/// dominates for any boundary with real extent.
///
/// Takes `&mut FaceRecord` rather than a solid because the plane rebuild needs
/// no other face — every caller that has a solid resolves the face first.
///
/// Was `edit/direct_edit/delete_face.rs:60-120`.
pub(crate) fn retrim_planar_face(
    face: &mut FaceRecord,
    plane: &Plane,
    edges: &HashMap<u64, EdgeRecord>,
    scale: f64,
    op: &str,
) -> Result<(), KernelRefusal> {
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for point in boundary_samples(face, edges, op)? {
        let delta = point.sub(plane.origin);
        let u = delta.dot(plane.u_dir);
        let v = delta.dot(plane.v_dir);
        u_min = u_min.min(u);
        u_max = u_max.max(u);
        v_min = v_min.min(v);
        v_max = v_max.max(v);
    }
    if !(u_min.is_finite() && u_max.is_finite() && v_min.is_finite() && v_max.is_finite()) {
        return Err(KernelRefusal::internal(KernelStage::Refine, "retrim_empty_boundary", format!("{op}: empty face boundary")));
    }
    let margin = ((u_max - u_min).max(v_max - v_min) * 0.25).max(scale * 1e-3);
    let new_origin = plane
        .origin
        .add(plane.u_dir.scale(u_min - margin))
        .add(plane.v_dir.scale(v_min - margin));
    let width = (u_max - u_min) + 2.0 * margin;
    let height = (v_max - v_min) + 2.0 * margin;
    let surface = make_plane(new_origin, plane.u_dir, plane.v_dir, width, height).or_refuse(KernelStage::Refine, "make_plane")?;
    rebuild_loop_pcurves(face, edges, &surface, (scale * 1e-7).max(1e-9), op)?;
    face.surface = surface;
    Ok(())
}

/// Re-trim a face that lives inside a solid and whose carrier must GROW in
/// place to cover a boundary that has already moved: sample the boundary, hand
/// the samples to `grow`, then rebuild every pcurve on the grown carrier.
///
/// `grow` is the pluggable half. It receives the whole solid because the exact
/// carrier prolongations are written against `(solid, face_id)` and remap the
/// face's own surface in place; it must not add or remove faces, because
/// `(shell, face_pos)` is resolved before it runs and reused after — which is
/// what both original copies did.
///
/// Was, twice over, `edit/direct_edit/face_offset.rs:882-937`
/// (`retrim_offset_ruled_face`, growth = `extend_ruled_neighbour_over`) and
/// `edit/direct_edit/face_move.rs:579-636` (`retrim_ruled_face`, growth =
/// `extend_ruled_neighbour_over` **then** `extend_revolution_carrier_over`).
/// Those two bodies were byte-identical apart from the refusal prefix and that
/// second growth call — a duplication the audit's inventory missed, found by
/// grepping the body's shape rather than reading the inventory.
pub(crate) fn retrim_face_in_solid<G>(
    solid: &mut BrepSolid,
    shell: usize,
    face_pos: usize,
    edges: &HashMap<u64, EdgeRecord>,
    grow: G,
    tolerance: f64,
    op: &str,
) -> Result<(), KernelRefusal>
where
    G: FnOnce(&mut BrepSolid, &[Vec3]) -> Result<(), KernelRefusal>,
{
    // Sample the updated boundary so the carrier can grow to cover it.
    let points = boundary_samples(&solid.shells[shell].faces[face_pos], edges, op)?;
    grow(solid, &points)?;
    let surface = solid.shells[shell].faces[face_pos].surface.clone();
    rebuild_loop_pcurves(
        &mut solid.shells[shell].faces[face_pos],
        edges,
        &surface,
        tolerance,
        op,
    )
}

