use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

// ---------------------------------------------------------------------------
// §6.12 sibling operation: move a face group.
// ---------------------------------------------------------------------------

/// How the moved group uses an edge: not at all, on both sides (carried
/// rigidly), or on exactly one side (the seam to re-intersect).
enum EdgeMoveClass {
    Fixed,
    Interior,
    Boundary { moved_face: u64, fixed_face: u64 },
}

enum EdgeMoveAction {
    /// Translate the curve rigidly; parameters and pcurves stay exact.
    Translate,
    /// Replace the curve by the straight line between re-solved endpoints.
    Rebuild { start: Vec3, end: Vec3 },
    /// Map the curve by an exact affine (SM1b: the radial-scale-about-axis of a
    /// rim re-intersecting a ruled carrier under an axis-parallel cap push).
    /// Rational-quadratic circles map exactly, so parametrisation is preserved.
    Transform(AffineTransform),
    /// Replace the curve outright by an exactly RE-BUILT conic arc — the section
    /// `plane ∩ ruled carrier` between two re-solved endpoints
    /// (`conic_arc_on_ruled`). A re-trim cannot serve here: the boolean that
    /// created these edges SPLIT them at the corner, so `domain == [t0, t1]` and
    /// there is no parameter headroom for an outward push (verified on the
    /// flatted-frustum fixture — the flat×cone hyperbolas and the cap's conic
    /// rims alike). Used where an oblique multi-rim cap's affine rim map carries
    /// the whole conic exactly but slides the ARC's endpoints off the fixed wall
    /// the corners must stay on (a cone's homothety about its apex), and for the
    /// curved fixed edge between a flat and a cylinder or cone whose corner the
    /// push moves.
    Replace {
        curve: NurbsCurve,
    },
}

enum FaceMoveAction {
    /// Shift the whole control net; pcurves stay exact for any carrier type.
    TranslateSurface,
    /// Rebuild the (planar) carrier around the new boundary and recompute
    /// every pcurve — the direct-edit equivalent of "extend the neighbour".
    Retrim(Plane),
}

/// `plane_of_surface` with per-face memoisation, because a face is consulted
/// once per boundary edge, once per touched vertex, and once at re-trim time.
pub(super) fn cached_plane(
    cache: &mut HashMap<u64, Plane>,
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    tolerance: f64,
) -> Result<Plane, KernelRefusal> {
    if let Some(plane) = cache.get(&face_id) {
        return Ok(*plane);
    }
    let (shell_index, face_index) = *face_lookup.get(&face_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "face_lookup",
            format!("move_faces: missing face {face_id}"),
        )
    })?;
    // `plane_of_surface` answers "is this carrier a plane"; every caller here has
    // already decided the face SHOULD be one, so its refusal is the planar lane's
    // own deferral of a curved carrier.
    let plane = plane_of_surface(
        &solid.shells[shell_index].faces[face_index].surface,
        tolerance,
        "move_faces",
    )
    .map_err(|message| {
        KernelRefusal::unsupported(KernelStage::Classify, "non_planar_carrier", message)
    })?;
    cache.insert(face_id, plane);
    Ok(plane)
}

/// The carrier kind of a face, in the words a user would use — for refusals
/// that must say WHAT the offending face is.
pub(super) fn carrier_kind_name(surface: &NurbsSurface) -> &'static str {
    match surface.analytic() {
        Some(AnalyticSurface::Plane { .. }) => "plane",
        Some(AnalyticSurface::RuledRevolution { rho0, rho1, .. }) => {
            let scale = rho0.abs().max(rho1.abs()).max(1.0);
            if (rho1 - rho0).abs() <= 1e-9 * scale {
                "cylinder"
            } else {
                "cone"
            }
        }
        Some(AnalyticSurface::Sphere { .. }) => "sphere",
        Some(AnalyticSurface::Torus { .. }) => "torus",
        Some(AnalyticSurface::Revolution { .. }) => "general surface of revolution",
        None => "free-form surface",
    }
}

/// The refusal a face whose carrier this operation cannot use deserves: one
/// that names the FACE, its CARRIER KIND and the ROLE it plays in the push.
///
/// Every call site below reaches `cached_plane` on a face it has already
/// decided is not a supported carrier — so the string a user got was
/// `plane_of_surface`'s own `move_faces: face is not planar (curved neighbours
/// are deferred in this slice)` (`offset/retrim.rs:152`): a message from a
/// shared helper that three features call, naming neither the face nor the
/// neighbour, and carrying slice-scoped wording out of a general predicate. A
/// torus groove, a barrel boss and a NURBS dimple all produced the same
/// sentence, and the earlier hand-read census of these refusals could not tell
/// from the text which of the *nine* such call sites had fired: it attributed
/// the torus groove's refusal to the wrong one.
///
/// This changes no verdict anywhere: it is applied as `map_err`, so a face
/// `plane_of_surface` accepts is still accepted, on exactly the same inputs.
fn unsupported_carrier(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    role: &str,
) -> KernelRefusal {
    let kind = face_lookup
        .get(&face_id)
        .map(|&(shell, face)| carrier_kind_name(&solid.shells[shell].faces[face].surface))
        .unwrap_or("missing");
    KernelRefusal::unsupported(KernelStage::Classify, "carrier_kind", format!(
        "move_faces: {role} (face {face_id}) is a {kind}; the plane push re-intersects only \
         planar and ruled (cylinder / cone) carriers here — refusing"
    ))
}

/// Intersect three-or-more planes in one point: take the best-conditioned
/// pair for the line, then the plane most transverse to that line for the
/// point. The caller checks the residual against EVERY plane afterwards, so
/// this only has to find *a* candidate, not prove consistency.
pub(super) fn solve_corner(planes: &[Plane]) -> Option<Vec3> {
    let mut best_pair: Option<(usize, usize, f64)> = None;
    for first in 0..planes.len() {
        for second in first + 1..planes.len() {
            let spread = planes[first].normal.cross(planes[second].normal).length();
            if best_pair.map(|(_, _, best)| spread > best).unwrap_or(true) {
                best_pair = Some((first, second, spread));
            }
        }
    }
    let (first, second, spread) = best_pair?;
    if spread <= PARALLEL_EPS {
        return None;
    }
    let line = intersect_planes(&planes[first], &planes[second])?;
    let mut best_third: Option<(usize, f64)> = None;
    for third in 0..planes.len() {
        if third == first || third == second {
            continue;
        }
        let transversality = line.dir.dot(planes[third].normal).abs();
        if best_third
            .map(|(_, best)| transversality > best)
            .unwrap_or(true)
        {
            best_third = Some((third, transversality));
        }
    }
    let (third, transversality) = best_third?;
    if transversality <= PARALLEL_EPS {
        return None;
    }
    intersect_line_plane(&line, &planes[third])
}

/// Rebuild a straight edge between two re-solved endpoints. Refuses the
/// degenerate and inverted cases — a zero or reversed chord means the
/// translation drove a moved face onto or past the neighbour this edge
/// belongs to (e.g. pushing a box face through its opposite face).
fn plan_straight_rebuild(
    edge: &EdgeRecord,
    start_old: Vec3,
    end_old: Vec3,
    start_new: Vec3,
    end_new: Vec3,
    tolerance: f64,
) -> Result<EdgeMoveAction, KernelRefusal> {
    if edge.degenerate {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "degenerate_edge", format!(
            "move_faces: degenerate edge {} would need re-stretching (deferred)",
            edge.id
        )));
    }
    if edge.curve.straight_segment(tolerance).is_none() {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "curved_edge_rebuild", format!(
            "move_faces: edge {} must be re-stretched but is not a straight line \
             (curved re-intersection edges are deferred in this slice)",
            edge.id
        )));
    }
    let new_chord = end_new.sub(start_new);
    if new_chord.length() <= tolerance {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "edge_collapse", format!(
            "move_faces: the translation collapses edge {} to zero length (a moved \
             face lands exactly on its neighbour) — refusing",
            edge.id
        )));
    }
    if end_old.sub(start_old).dot(new_chord) <= 0.0 {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, EDGE_INVERSION, format!(
            "move_faces: the translation inverts edge {} (a moved face passes beyond \
             its neighbour) — refusing",
            edge.id
        )));
    }
    Ok(EdgeMoveAction::Rebuild {
        start: start_new,
        end: end_new,
    })
}

/// The (linearly-varying) radius of a ruled revolution at axial coordinate
/// `axial`: `rho0` at the base, `rho1` at `height`. Constant for a cylinder.
fn rho_at(rho0: f64, rho1: f64, height: f64, axial: f64) -> f64 {
    if height == 0.0 {
        rho0
    } else {
        rho0 + (rho1 - rho0) * axial / height
    }
}

/// Frame/radii/height of any revolution with a STRAIGHT (degree-1, unit-weight)
/// generatrix — the full-2π `RuledRevolution` quadrics AND partial-sweep
/// `Revolution`s whose generatrix is a line. The latter is how a fillet band /
/// partial cylinder-or-cone wall recognizes (a straight profile swept less than
/// 2π is the general `Revolution`, not `RuledRevolution`), so treating it as a
/// ruled carrier is exactly what lets a face adjacent to a fillet band be pushed.
///
/// This mirrors `analytic_surface::intersect::ruled_revolution_data`, which
/// already extracts the same `(frame, rho0, rho1, height)` from a partial-sweep
/// `Revolution`; that function is private behind a private module (unreachable
/// from here without editing `analytic_surface.rs`), so the tiny extraction is
/// re-derived from the (public) frame basis instead of shared. A CURVED
/// generatrix (sphere/torus-like, non-ruled) returns None → it must still refuse.
fn ruled_revolution_carrier(
    surface: &NurbsSurface,
) -> Option<(crate::RevolutionFrame, f64, f64, f64)> {
    match surface.analytic() {
        Some(AnalyticSurface::RuledRevolution {
            frame,
            rho0,
            rho1,
            height,
        }) => Some((frame.clone(), *rho0, *rho1, *height)),
        Some(AnalyticSurface::Revolution {
            frame, generatrix, ..
        }) => {
            // Unit-weight straight line only; a rational or higher-degree
            // generatrix is a genuinely curved surface of revolution.
            const UNIT_WEIGHT_TOL: f64 = 1e-9;
            let controls = &generatrix.control_points;
            if generatrix.degree != 1
                || controls.len() != 2
                || (controls[0].w - 1.0).abs() > UNIT_WEIGHT_TOL
                || (controls[1].w - 1.0).abs() > UNIT_WEIGHT_TOL
            {
                return None;
            }
            // Cylindrical decomposition (radius, axial) from the public frame
            // basis — `RevolutionFrame::cylindrical` is module-private.
            let decompose = |point: Vec3| -> (f64, f64) {
                let d = point.sub(frame.origin);
                let axial = d.dot(frame.axis);
                let radial = d.sub(frame.axis.scale(axial)).length();
                (radial, axial)
            };
            let (rho0, z0) = decompose(controls[0].point().ok()?);
            let (rho1, z1) = decompose(controls[1].point().ok()?);
            let height = z1 - z0;
            if height.abs() <= 1e-12 * (1.0 + rho0.abs().max(rho1.abs())) {
                return None;
            }
            // Rebase the origin to the generatrix start's axial position so
            // v = axial / height, exactly as the `RuledRevolution` variant does.
            let origin = frame.origin.add(frame.axis.scale(z0));
            Some((
                crate::RevolutionFrame {
                    origin,
                    ..frame.clone()
                },
                rho0,
                rho1,
                height,
            ))
        }
        _ => None,
    }
}

/// The ruled-revolution carrier (cylinder OR cone, full-2π OR partial-sweep
/// fillet band) of a face resolved by id, for ANY push direction (SM1/SM1b
/// axis-parallel, SM1c oblique). A planar cap re-intersecting such a carrier
/// under a push keeps the SAME surface and only re-trims; the rim maps by the
/// exact affine `rim_ruled_map` (a homothety about the cone apex, or an axis
/// translation for a cylinder), so unlike the earlier `axis_parallel_ruled`
/// predicate this no longer requires the push to be parallel to the axis.
pub(super) fn carrier_ruled(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
) -> Option<(crate::RevolutionFrame, f64, f64, f64)> {
    let &(shell, face) = face_lookup.get(&face_id)?;
    ruled_revolution_carrier(&solid.shells[shell].faces[face].surface)
}

/// The SPHERE carrier (frame + radius) of a face resolved by id, or `None` when
/// it is not an analytic sphere. A planar push that borders a FIXED sphere
/// re-intersects it in an EXACT circle (`intersect_plane_quadric` = plane ×
/// sphere) and re-trims the sphere to that new rim — see
/// `move_planar_face_across_sphere` (backlog #5, Plane × Sphere).
fn carrier_sphere(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
) -> Option<(crate::RevolutionFrame, f64)> {
    let &(shell, face) = face_lookup.get(&face_id)?;
    match solid.shells[shell].faces[face].surface.analytic() {
        Some(AnalyticSurface::Sphere { frame, radius }) => Some((frame.clone(), *radius)),
        _ => None,
    }
}

/// True iff the face's carrier is a plane the push is PARALLEL to — the corner
/// slides within it, carried rigidly by `+translation` (SM1's invariant-plane
/// case, e.g. a box side wall as the top is pushed up).
fn plane_parallel_to(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    translation: Vec3,
    parallel_tol: f64,
) -> bool {
    let Some(&(shell, face)) = face_lookup.get(&face_id) else {
        return false;
    };
    match solid.shells[shell].faces[face].surface.analytic() {
        Some(AnalyticSurface::Plane { u_dir, v_dir, .. }) => {
            let normal = u_dir.cross(*v_dir);
            let len = normal.length();
            len > 0.0 && translation.dot(normal).abs() / len <= parallel_tol
        }
        _ => false,
    }
}

/// True iff a carrier is INVARIANT under `translation` — `S + T == S` as a point
/// set. Read on a FIXED face it says a corner on it rides rigidly by
/// `+translation` and provably stays on it; read on every MOVED face at once it
/// says the whole edit is the identity (`translation_is_identity_on`), because
/// the predicate is about the geometry and not about the role the face plays.
/// Two carriers are invariant:
///   • a plane the push is PARALLEL to (the corner slides in-plane), and
///   • a CYLINDER whose axis the push is PARALLEL to (the point shifts along a
///     generatrix, radius unchanged).
/// A CONE is deliberately excluded: its radius varies with the axial coordinate,
/// so an axis translation moves a point OFF the carrier — a corner there must
/// re-intersect, not ride. This lets a corner shared by a fillet band (an
/// axis-parallel partial cylinder) and a parallel wall ride rigidly under an
/// axis-parallel push, while any oblique push falls through to the (planar-only)
/// re-intersection path and refuses.
fn carrier_invariant_under(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    translation: Vec3,
    parallel_tol: f64,
) -> bool {
    if plane_parallel_to(solid, face_lookup, face_id, translation, parallel_tol) {
        return true;
    }
    if let Some((frame, rho0, rho1, _height)) = carrier_ruled(solid, face_lookup, face_id) {
        let radius_scale = rho0.abs().max(rho1.abs()).max(1.0);
        let is_cylinder = (rho1 - rho0).abs() <= 1e-9 * radius_scale;
        if is_cylinder {
            // Invariant iff the translation is parallel to the axis, i.e. its
            // component perpendicular to the axis is negligible.
            let axis = frame.axis;
            let perpendicular = translation.sub(axis.scale(translation.dot(axis)));
            return perpendicular.length() <= parallel_tol;
        }
    }
    false
}

/// True iff `translation` carries EVERY selected carrier onto ITSELF, which
/// makes the whole edit the IDENTITY: the answer is the input body, exactly.
///
/// The argument in full, because this lane answers before any other guard is
/// asked. `move_faces` edits CARRIERS: a selected face's carrier becomes
/// `S + T`, an unselected one is not touched at all, and every edge of a
/// manifold solid is a section of the two carriers that share it (a periodic
/// face's seam is the exception, and it is pinned by its endpoints, which are
/// such sections). When `S + T == S` for every selected face, every carrier in
/// the output is the same POINT SET as the one in the input — so every section
/// is the same curve, every corner the same point, every trim the same trim,
/// and the body is the input. That is not a guard being relaxed; it is a
/// question the roads below never get to ask, because each of them chooses its
/// lane from a NEIGHBOUR's carrier pair before the motion's magnitude is
/// consulted at all. Nothing is left for the collapse, inversion and tear
/// guards to find: each convicts a moved face for reaching somewhere new, and
/// no point of an invariant carrier is anywhere new. Returning `solid` itself
/// rather than a rebuild of it also means the identity is exact — not exact to
/// a tolerance.
///
/// The invariant carriers are exactly the plane the translation is PARALLEL to
/// and the cylinder whose AXIS it is parallel to; `carrier_invariant_under`
/// already decides that on the geometry alone. A cone is excluded there and
/// must be: an axial translation slides a cone ALONG itself but not ONTO
/// itself, because its radius varies with the axial coordinate. A sphere,
/// torus, general revolution or free-form patch is invariant under no
/// translation but the zero one, and is refused here for want of a proof, which
/// costs nothing: those take the roads below exactly as before.
///
/// EVERY selected face, because one that is not invariant moves the body: a top
/// that slides in its own plane TOGETHER with a band that does not is not an
/// identity, and takes the roads below unchanged.
fn translation_is_identity_on(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    moved: &HashSet<u64>,
    translation: Vec3,
    parallel_tol: f64,
) -> bool {
    moved.iter().all(|&face_id| {
        carrier_invariant_under(solid, face_lookup, face_id, translation, parallel_tol)
    })
}

/// The EXACT affine that re-intersects a planar cap's rim with a ruled
/// revolution under a push of ANY direction — the SM1c generalisation of SM1b's
/// axis-parallel radial-scale map:
///
/// - **Cone/frustum:** a homothety (central dilation) about the apex with ratio
///   `λ = 1 + (n·T)/D₀`, where `n` is the cap-plane unit normal, `T` the
///   translation, and `D₀ = n·(cap_origin − apex)` the signed apex→plane
///   distance along `n`. The dilation maps the cone to itself and the cap plane
///   to the TRANSLATED cap plane, so it maps the old rim exactly to the new one
///   — for any push direction (oblique sections are hyperbola/ellipse arcs, and
///   an affine maps rational curves control-point-wise, so parametrisation is
///   preserved). For an axis-parallel ⊥-cap this reduces to the radial scale
///   `s = rho_at(z+d)/rho_at(z)`.
/// - **Cylinder (`rho0 == rho1`):** the carrier is invariant under axis
///   translation, so the cap plane's shift maps to a pure axis shift `t·axis`
///   with `t = (n·T)/(n·axis)` (SM1's `n = axis` case gives `t = d`).
///
/// Refuses a cap plane through the apex (`D₀ ≈ 0`), a push to/through the apex
/// (`λ ≤ tol`), and a cap plane parallel to a cylinder axis (`n·axis ≈ 0`, a
/// straight generatrix section handled by the straight-rebuild path).
fn rim_ruled_map(
    frame: &crate::RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    moved_plane: &Plane,
    translation: Vec3,
    tolerance: f64,
) -> Result<AffineTransform, KernelRefusal> {
    let n = moved_plane.normal;
    let axis = frame.axis;
    let radius_scale = rho0.abs().max(rho1.abs()).max(1.0);
    // Cylinder: axis-invariant carrier ⇒ the cap shift is a pure axis translation.
    if (rho1 - rho0).abs() <= 1e-9 * radius_scale {
        let axial_component = n.dot(axis);
        if axial_component.abs() <= 1e-9 {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "cap_parallel_to_axis",
                "move_faces: the cap plane is parallel to the cylinder axis \
                 (straight generatrix section) — refusing"));
        }
        let shift = axis.scale(n.dot(translation) / axial_component);
        return AffineTransform::new([
            1.0, 0.0, 0.0, shift.x, //
            0.0, 1.0, 0.0, shift.y, //
            0.0, 0.0, 1.0, shift.z, //
            0.0, 0.0, 0.0, 1.0,
        ])
        .or_refuse(KernelStage::Refine, "affine");
    }
    // Cone/frustum: homothety about the apex (where rho_at → 0).
    let z_apex = rho0 * height / (rho0 - rho1);
    let apex = frame.origin.add(axis.scale(z_apex));
    let d0 = n.dot(moved_plane.origin.sub(apex));
    if d0.abs() <= tolerance {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "cap_through_apex", "move_faces: the cap plane passes through the cone apex — refusing"));
    }
    let lambda = 1.0 + n.dot(translation) / d0;
    if lambda <= tolerance {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "push_past_apex",
            "move_faces: the push drives the cap to or past the cone apex \
             (scale → 0) — refusing"));
    }
    // Y = apex + λ·(X − apex) = λ·X + (1−λ)·apex.
    let offset = apex.scale(1.0 - lambda);
    AffineTransform::new([
        lambda, 0.0, 0.0, offset.x, //
        0.0, lambda, 0.0, offset.y, //
        0.0, 0.0, lambda, offset.z, //
        0.0, 0.0, 0.0, 1.0,
    ])
    .or_refuse(KernelStage::Refine, "affine")
}

/// The checked contract for SM1b (the user's "reapply the trimming" semantics):
/// the constructed rim must lie ON both modified carriers — the fixed ruled
/// carrier (radius `rho_at`) and the translated moved plane. Samples the mapped
/// rim; refuses if any sample drifts off either surface. The map is exact by
/// construction, so this is a fail-safe guard, not the primary computation.
fn verify_rim_on_carriers(
    edge: &EdgeRecord,
    map: &AffineTransform,
    frame: &crate::RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    moved_plane: &Plane,
    translation: Vec3,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let (origin, axis) = (frame.origin, frame.axis);
    let normal = moved_plane.normal;
    let plane_point = moved_plane.origin.add(translation);
    for step in 0..=8 {
        let t = edge.t0 + (edge.t1 - edge.t0) * (step as f64 / 8.0);
        let mapped = map.point(edge.curve.evaluate(t).or_refuse(KernelStage::Validate, "evaluate")?);
        let delta = mapped.sub(origin);
        let axial = delta.dot(axis);
        let radial = delta.sub(axis.scale(axial)).length();
        let off_ruled = (radial - rho_at(rho0, rho1, height, axial)).abs();
        let off_plane = mapped.sub(plane_point).dot(normal).abs();
        if off_ruled > 10.0 * tolerance || off_plane > 10.0 * tolerance {
            return Err(KernelRefusal::internal(KernelStage::Validate, "rim_on_carriers", format!(
                "move_faces: the re-intersected rim does not lie on both modified carriers \
                 (off ruled {off_ruled:.3e}, off plane {off_plane:.3e}) — refusing"
            )));
        }
    }
    Ok(())
}

/// A ruled revolution's frame carried by the push. A translation is rigid and
/// leaves the axis DIRECTION and the radial basis alone, so the shifted frame
/// describes the PUSHED carrier exactly and its radii and height are untouched.
/// That is what lets a bore wall's own section against a fixed flat be
/// re-intersected with the same closed form the mirror case uses.
fn shift_ruled_frame(
    frame: &crate::RevolutionFrame,
    translation: Vec3,
) -> crate::RevolutionFrame {
    crate::RevolutionFrame {
        origin: frame.origin.add(translation),
        axis: frame.axis,
        x_axis: frame.x_axis,
        y_axis: frame.y_axis,
    }
}

/// THE MIRROR MAP — the exact affine carrying the rim a MOVED ruled carrier
/// shares with a FIXED plane: `P ∩ C  ↦  P ∩ (C + T)`.
///
/// Which of the two carriers the push moves is not something the section can
/// see. Read in the CARRIER's own frame, a fixed plane `P` against a carrier
/// pushed by `T` IS the plane pushed by `−T` against the carrier on record — so
/// the map is [`rim_ruled_map`]'s affine for `−T` (a cylinder's axis
/// translation, a cone's homothety about its apex), which lands the rim on
/// `(P − T) ∩ C`, followed by `+ T`, which lands the whole conic back on `P`
/// and on the pushed carrier. Both steps are affine, so a rational-quadratic
/// rim maps EXACTLY, control point by control point, and its parametrisation
/// survives untouched — a CLOSED rim included, where there is no arc to
/// re-anchor and nothing for a corner to disambiguate.
///
/// For a CYLINDER the composite is the pure translation
/// `T − ((n·T)/(n·a))·a`: the push with the part the carrier absorbs (its axial
/// component) removed. The limits are `rim_ruled_map`'s own, and one of them is
/// this direction's too — a push that would lay the axis parallel to the flat
/// (`n·a ≈ 0`) meets it in a pair of rulings rather than a conic, and there is
/// no rim to build.
fn moved_ruled_rim_map(
    frame: &crate::RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    fixed_plane: &Plane,
    translation: Vec3,
    tolerance: f64,
) -> Result<AffineTransform, KernelRefusal> {
    let back = rim_ruled_map(
        frame,
        rho0,
        rho1,
        height,
        fixed_plane,
        translation.scale(-1.0),
        tolerance,
    )?;
    // `translate(T) ∘ back` — a translation only touches the last column.
    let mut elements = back.elements;
    elements[3] += translation.x;
    elements[7] += translation.y;
    elements[11] += translation.z;
    AffineTransform::new(elements).or_refuse(KernelStage::Refine, "affine")
}

/// The PUSHED ruled carrier of the one moved face at a corner, when that is
/// what the corner must be solved against: `Some((face id, the frame on record,
/// rho0, rho1, height))` for exactly one moved face at the vertex whose carrier
/// is a cylinder or cone the translation does NOT leave invariant.
///
/// `None` — and so the roads that existed before — for a planar moved face, for
/// a carrier the push maps onto itself (the rigid regime has already claimed
/// those), and for a group that brings TWO faces to the corner, where "the
/// rim's own affine places it" is no longer a statement about one carrier. A
/// moved sphere, torus, revolution or free-form patch is not ruled, so it falls
/// through to the three-plane solve and refuses there by name, as before.
fn moved_ruled_carrier_at(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    moved: &HashSet<u64>,
    adjacent: &HashSet<u64>,
    translation: Vec3,
    parallel_tolerance: f64,
) -> Option<(u64, crate::RevolutionFrame, f64, f64, f64)> {
    let mut here = adjacent.iter().copied().filter(|id| moved.contains(id));
    let face_id = here.next()?;
    if here.next().is_some() {
        return None;
    }
    if carrier_invariant_under(solid, face_lookup, face_id, translation, parallel_tolerance) {
        return None;
    }
    let (frame, rho0, rho1, height) = carrier_ruled(solid, face_lookup, face_id)?;
    Some((face_id, frame, rho0, rho1, height))
}

/// True iff the face's carrier is a plane (the today path — planar retrim).
pub(super) fn carrier_is_planar(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
) -> bool {
    face_lookup
        .get(&face_id)
        .map(|&(shell, face)| {
            matches!(
                solid.shells[shell].faces[face].surface.analytic(),
                Some(AnalyticSurface::Plane { .. })
            )
        })
        .unwrap_or(false)
}

/// A rebuilt straight edge bordering a curved carrier must still lie ON that
/// carrier (endpoints + midpoint within `10·tol`), else the moved group tore
/// off it — refuse rather than emit a bad solid.
///
/// `carrier_shift` is `Some(T)` when the face is one the push MOVES — the bore
/// wall carrying its own seam generatrix — in which case the chord is measured
/// against the carrier the push LEAVES, not the one on record. Measured against
/// the stored frame it would convict every such push of tearing the seam off
/// its own cylinder. (An invariant carrier measures identically either way:
/// `ruled_offset` reads only the origin and the axis, and an axis-parallel
/// shift moves the origin along the axis.)
fn verify_chord_on_carrier(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    face_id: u64,
    start: Vec3,
    end: Vec3,
    carrier_shift: Option<Vec3>,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let (shell, face) = *face_lookup.get(&face_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "chord_face_lookup",
            format!("move_faces: missing face {face_id}"),
        )
    })?;
    // Measure against the ANALYTIC (unbounded) carrier, not the trimmed NURBS
    // surface: the rebuilt chord routinely lands OUTSIDE the current v-domain
    // (the carrier is grown to cover it afterwards), so a domain-clamped
    // projection would report a false miss. For a cylinder "on the carrier" is
    // just "radial distance from the axis == radius". Accepts a partial-sweep
    // fillet band (`Revolution`) the same way as a full `RuledRevolution`.
    let Some((frame, rho0, rho1, height)) =
        ruled_revolution_carrier(&solid.shells[shell].faces[face].surface)
    else {
        return Err(KernelRefusal::internal(KernelStage::Classify, "ruled_carrier_expected", format!(
            "move_faces: chord-on-carrier check expects a ruled carrier (face {face_id})"
        )));
    };
    let origin = frame.origin.add(carrier_shift.unwrap_or_default());
    let axis = frame.axis;
    let midpoint = start.add(end).scale(0.5);
    for point in [start, midpoint, end] {
        let delta = point.sub(origin);
        let axial = delta.dot(axis);
        let radial = delta.sub(axis.scale(axial)).length();
        // The generatrix radius VARIES along the axis on a cone, so compare
        // against rho_at(axial), not a fixed rho0 (which is cylinder-only).
        let radius = rho_at(rho0, rho1, height, axial);
        if (radial - radius).abs() > 10.0 * tolerance {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, "chord_off_carrier", format!(
                "move_faces: rebuilt edge would leave its curved neighbour \
                 (face {face_id}, off by {:.3e}) — refusing",
                (radial - radius).abs()
            )));
        }
    }
    Ok(())
}

/// Re-trim a FIXED ruled neighbour whose boundary moved under an axis-parallel
/// push: grow the carrier along its axis to cover the new boundary
/// (`extend_ruled_neighbour_over` for a full-2π cylinder/cone,
/// `extend_revolution_carrier_over` for a partial-sweep fillet band — both
/// exact), then recompute every pcurve on the grown carrier from the
/// already-updated edge curves. The direct-edit analogue of `retrim_planar_face`
/// for a translation-invariant cylinder; all loops are visited, so holes on the
/// neighbour are carried.
///
/// The three-phase body is `crate::offset_retrim::retrim_face_in_solid`; this is
/// the two-step growth strategy and the caller's `op` refusal prefix. It differs
/// from `face_offset::retrim_offset_ruled_face` in exactly that second growth
/// call — the partial-sweep `Revolution` prolongation, which the offset push has
/// never run.
///
/// **The growth covers the rim's axial extremes, not only the five stations
/// per coedge `boundary_samples` pins (2026-10-03).** A rim re-intersected on a
/// tilted wall is an ellipse whose axial extremes fall BETWEEN those stations:
/// on the 2026-09-21 runaway-bore document the rotated `Pin_S` was grown to
/// `[0, 53.0875]` along its axis while its mouth ellipses truly ran
/// `[-2.5436, 17.1686]` and `[35.9189, 55.6311]`, so each left the chart by
/// 2.5436 mm and the pcurve fitted against the bounded chart stood 0.3722 /
/// 0.4222 mm off its edge (`validate()` read exactly that; the acceptance's
/// crossing repair happened to rebuild them later). The growth now reads each
/// rim edge's axial extent with [`rim_axial_extent`] — every local extremum of
/// the sampled axial coordinate refined, and a CERTIFIED bound from the control
/// polygons of the pieces between stations — and grows the carrier to the
/// certified bound, so the rim is inside the chart by construction; a carrier
/// the five stations already grew far enough is left exactly as it was.
pub(super) fn retrim_ruled_face(
    solid: &mut BrepSolid,
    face_id: u64,
    final_edges: &HashMap<u64, EdgeRecord>,
    tolerance: f64,
    op: &str,
) -> Result<(), KernelRefusal> {
    let (shell, face_pos) = find_face(solid, face_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Sew,
            "ruled_face_lookup",
            format!("{op}: missing ruled face {face_id}"),
        )
    })?;
    retrim_face_in_solid(
        solid,
        shell,
        face_pos,
        final_edges,
        |solid, points| {
            let mut points = points.to_vec();
            points.extend(rim_axial_growth_points(
                &solid.shells[shell].faces[face_pos],
                final_edges,
                op,
            )?);
            // The carrier helpers are typed; this retrim's closure is still
            // the stringly shape its callee asks for (lossy exit).
            extend_ruled_neighbour_over(solid, face_id, &points, tolerance)?;
            Ok(extend_revolution_carrier_over(solid, face_id, &points, tolerance)?)
        },
        tolerance,
        op,
    )
    
}

/// The points where each rim edge of `face` reaches its lowest and highest
/// axial coordinate on the face's ruled or partial-sweep revolution carrier —
/// the two points per coedge the carrier must be grown to cover, read from
/// the caller's UPDATED edge table like `boundary_samples`. A face on any other
/// carrier contributes nothing (the growth helpers no-op there anyway).
///
/// Each edge's axial coordinate `a(t) = (c(t) − origin) · axis` is swept at 64
/// stations over `[t0, t1]`; the extreme station's two neighbours bracket the
/// true extreme, and golden-section refinement closes that bracket to 1e-13 of
/// the span. A grid alone would leave the chart short by the sampling
/// shortfall (`A·(π/n)²/2`, 5e-5 mm on this document's 10 mm amplitude at
/// n = 1024), and the fitter would read that shortfall back as a pcurve
/// deviation.
/// The two readings of [`rim_axial_extent`].
#[derive(Clone, Copy, Debug)]
pub(super) struct RimAxialExtent {
    pub(super) refined_low: f64,
    pub(super) refined_high: f64,
    pub(super) certified_low: f64,
    pub(super) certified_high: f64,
}

/// A rim's axial extent on a ruled or partial-sweep revolution carrier: the
/// lowest and highest axial coordinate any point of the face's rim edges
/// reaches, read from the caller's UPDATED edge table like `boundary_samples`.
///
/// Two readings, and the growth uses the second:
///
/// * `refined_low` / `refined_high` — the axial coordinate
///   `a(t) = (c(t) − origin) · axis` is sampled at 16 stations per native knot
///   span of each edge (plus the span and trim ends); EVERY sampled local
///   extremum is then refined by golden section inside its two neighbouring
///   stations to 1e-13 of the trim's span. This is where the extremum IS to
///   the parameter's resolution when it lies in a sampled bracket; it is not a
///   proof that no narrower extremum hides between two stations.
/// * `certified_low` / `certified_high` — the control polygons of the curve
///   pieces between consecutive stations (the curve split there): a rational
///   B-spline with positive weights lies in the convex hull of its control
///   points, so the union of the pieces' hulls CONTAINS THE WHOLE RIM. That
///   is the one guarantee: `[certified_low, certified_high]` contains the true
///   extent. The refined reading is a separate, unguaranteed one (a sampled
///   extremum can sit INSIDE the true one), and the certified extent is only
///   widened to contain it. The excess over the true extreme is the hull's,
///   `O(h²)` in the station spacing (read on the fixtures in the record); a
///   chart grown to it covers the rim with certainty and is never short by a
///   sampling miss. A station closer than the curve's own split tolerance
///   (`KNOT_IDENTITY_TOL`, absolute) or 1e-9 of the domain to a piece's end
///   is not split at: the unsplit piece's hull still contains its whole
///   interval, so coverage holds, only looser.
///
/// A rim edge with a non-positive control weight has no hull bound; the
/// reading refuses by name rather than certify what it cannot. A face on any
/// other carrier reads `None` (the growth helpers no-op there anyway).
pub(super) fn rim_axial_extent(
    face: &FaceRecord,
    edges: &HashMap<u64, EdgeRecord>,
    op: &str,
) -> Result<Option<RimAxialExtent>, KernelRefusal> {
    let frame = match face.surface.analytic() {
        Some(AnalyticSurface::RuledRevolution { frame, .. })
        | Some(AnalyticSurface::Revolution { frame, .. }) => frame.clone(),
        _ => return Ok(None),
    };
    const STATIONS_PER_SPAN: usize = 16;
    let mut extent = RimAxialExtent {
        refined_low: f64::INFINITY,
        refined_high: f64::NEG_INFINITY,
        certified_low: f64::INFINITY,
        certified_high: f64::NEG_INFINITY,
    };
    let mut read_any = false;
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            let edge = edges
                .get(&coedge.edge_id)
                .ok_or_else(|| format!("{op}: missing edge {}", coedge.edge_id))
                .or_refuse(KernelStage::Refine, "retrim_missing_edge")?;
            if edge.degenerate {
                continue;
            }
            let curve = &edge.curve;
            let axial_of = |point: Vec3| point.sub(frame.origin).dot(frame.axis);
            let axial = |t: f64| -> Result<f64, KernelRefusal> {
                Ok(axial_of(curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?))
            };
            let (lo, hi) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
            let span = hi - lo;
            if !(span > 0.0) {
                continue;
            }
            // Stations: the trim ends, every native knot inside the trim, and
            // 16 per knot span between them.
            let mut stations: Vec<f64> = vec![lo];
            let mut knots: Vec<f64> = curve.knots.iter().copied().filter(|&k| k > lo && k < hi).collect();
            knots.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * span);
            let mut previous = lo;
            for knot in knots.into_iter().chain(std::iter::once(hi)) {
                if knot - previous <= 1e-12 * span {
                    continue;
                }
                for step in 1..=STATIONS_PER_SPAN {
                    stations.push(previous + (knot - previous) * step as f64 / STATIONS_PER_SPAN as f64);
                }
                previous = knot;
            }
            let readings: Vec<f64> = stations.iter().map(|&t| axial(t)).collect::<Result<_, _>>()?;
            let last = readings.len() - 1;
            // Every sampled local extremum, refined inside its bracket.
            for sign in [1.0f64, -1.0] {
                for index in 0..=last {
                    let value = sign * readings[index];
                    let left_ok = index == 0 || value >= sign * readings[index - 1];
                    let right_ok = index == last || value >= sign * readings[index + 1];
                    if !(left_ok && right_ok) {
                        continue;
                    }
                    let (mut a, mut b) = (stations[index.saturating_sub(1)], stations[(index + 1).min(last)]);
                    let f = |t: f64| -> Result<f64, KernelRefusal> { Ok(-sign * axial(t)?) };
                    let phi = 0.618_033_988_749_894_9_f64;
                    let mut c = b - phi * (b - a);
                    let mut d = a + phi * (b - a);
                    let (mut fc, mut fd) = (f(c)?, f(d)?);
                    for _ in 0..200 {
                        if (b - a).abs() <= 1e-13 * span {
                            break;
                        }
                        if fc < fd {
                            b = d;
                            d = c;
                            fd = fc;
                            c = b - phi * (b - a);
                            fc = f(c)?;
                        } else {
                            a = c;
                            c = d;
                            fc = fd;
                            d = a + phi * (b - a);
                            fd = f(d)?;
                        }
                    }
                    let best = (-fc).max(-fd).max(value) * sign;
                    if sign > 0.0 {
                        extent.refined_high = extent.refined_high.max(best);
                    } else {
                        extent.refined_low = extent.refined_low.min(best);
                    }
                }
            }
            // The certified bound: the control polygon of every piece between
            // consecutive stations, the curve split there.
            let mut rest = curve.clone();
            let [d0, d1] = rest.domain().or_refuse(KernelStage::Refine, "domain")?;
            // `NurbsCurve::split` refuses a parameter within the ABSOLUTE
            // `KNOT_IDENTITY_TOL` of a domain end; the guard is that floor or
            // 1e-9 of the domain, whichever is larger, and a station inside
            // it is skipped (the unsplit piece's hull still covers it).
            let guard = (1e-9 * (d1 - d0)).max(crate::KNOT_IDENTITY_TOL);
            let mut pieces: Vec<NurbsCurve> = Vec::with_capacity(stations.len());
            if lo > d0 + guard {
                rest = rest.split(lo).or_refuse(KernelStage::Refine, "split")?.1;
            }
            for &t in &stations[1..last] {
                let [r0, r1] = rest.domain().or_refuse(KernelStage::Refine, "domain")?;
                if t <= r0 + guard || t >= r1 - guard {
                    continue;
                }
                let (left, right) = rest.split(t).or_refuse(KernelStage::Refine, "split")?;
                pieces.push(left);
                rest = right;
            }
            if hi < d1 - guard {
                let [r0, r1] = rest.domain().or_refuse(KernelStage::Refine, "domain")?;
                if hi > r0 + guard && hi < r1 - guard {
                    rest = rest.split(hi).or_refuse(KernelStage::Refine, "split")?.0;
                }
            }
            pieces.push(rest);
            for piece in &pieces {
                for control in &piece.control_points {
                    if !(control.w > 0.0) {
                        return Err(KernelRefusal::unsupported(
                            KernelStage::Refine,
                            "rim_axial_extent_weights",
                            format!(
                                "{op}: rim edge {} carries a control weight {} that is not positive, so its \
                                 axial extent has no convex-hull bound to certify the carrier's growth against",
                                edge.id, control.w
                            ),
                        ));
                    }
                    let a = axial_of(control.point().or_refuse(KernelStage::Refine, "control_point")?);
                    extent.certified_low = extent.certified_low.min(a);
                    extent.certified_high = extent.certified_high.max(a);
                }
            }
            read_any = true;
        }
    }
    if !read_any {
        return Ok(None);
    }
    // The hull contains the curve, so it contains the refined readings too;
    // taking the union guards the bound against a rounding hair.
    extent.certified_low = extent.certified_low.min(extent.refined_low);
    extent.certified_high = extent.certified_high.max(extent.refined_high);
    Ok(Some(extent))
}

/// The points `retrim_ruled_face` grows a carrier over beyond the five pinned
/// stations: the rim's CERTIFIED axial bound, as two points on the axis.
pub(super) fn rim_axial_growth_points(
    face: &FaceRecord,
    edges: &HashMap<u64, EdgeRecord>,
    op: &str,
) -> Result<Vec<Vec3>, KernelRefusal> {
    let Some(extent) = rim_axial_extent(face, edges, op)? else {
        return Ok(Vec::new());
    };
    let frame = match face.surface.analytic() {
        Some(AnalyticSurface::RuledRevolution { frame, .. })
        | Some(AnalyticSurface::Revolution { frame, .. }) => frame.clone(),
        _ => return Ok(Vec::new()),
    };
    Ok(vec![
        frame.origin.add(frame.axis.scale(extent.certified_low)),
        frame.origin.add(frame.axis.scale(extent.certified_high)),
    ])
}

/// The EXACT corner where a fixed PLANE, a fixed RULED carrier and the
/// TRANSLATED cap plane meet: the line `fixed plane ∩ translated cap plane`
/// intersected with the ruled carrier (a quadratic in the line parameter),
/// taking the root nearest the corner's OLD position so the corner moves
/// continuously.
///
/// This is the carrier-level solve that a CURVED fixed edge needs.
/// `resolve_corner_on_fixed_edge` brackets strictly INSIDE the fixed edge's
/// domain, and the boolean that produced such an edge split it exactly at the
/// corner (`domain == [t0, t1]`), so riding the edge can only ever move a corner
/// INWARD — an outward push would refuse for a purely representational reason.
/// The carriers have no such horizon.
pub(super) fn corner_on_plane_and_ruled(
    fixed_plane: &Plane,
    cap_normal: Vec3,
    cap_c: f64,
    frame: &crate::RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    old_corner: Vec3,
    tolerance: f64,
) -> Result<Vec3, KernelRefusal> {
    let direction = fixed_plane.normal.cross(cap_normal);
    if direction.length() <= PARALLEL_EPS {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "parallel_carriers",
            "move_faces: the pushed cap plane is parallel to a fixed planar neighbour \
             (no corner) — refusing"));
    }
    let direction = direction.normalized().or_refuse(KernelStage::Refine, "normalized")?;
    // A point on both planes, taken in the 2-D span of the two normals.
    let ca = fixed_plane.normal.dot(fixed_plane.origin);
    let naa = fixed_plane.normal.dot(fixed_plane.normal);
    let nab = fixed_plane.normal.dot(cap_normal);
    let nbb = cap_normal.dot(cap_normal);
    let determinant = naa * nbb - nab * nab;
    if determinant.abs() <= PARALLEL_EPS {
        return Err(KernelRefusal::internal(KernelStage::Refine, "carrier_line", "move_faces: cannot place the corner's carrier line — refusing"));
    }
    let alpha = (ca * nbb - cap_c * nab) / determinant;
    let beta = (cap_c * naa - ca * nab) / determinant;
    let base = fixed_plane
        .normal
        .scale(alpha)
        .add(cap_normal.scale(beta));
    // |P − O|² − axial² = rho_at(axial)² along P(s) = base + s·direction.
    let offset = base.sub(frame.origin);
    let axis = frame.axis;
    let slope = if height == 0.0 {
        0.0
    } else {
        (rho1 - rho0) / height
    };
    let a0 = offset.dot(axis);
    let a1 = direction.dot(axis);
    let r0 = rho0 + slope * a0;
    let quad = 1.0 - a1 * a1 - slope * slope * a1 * a1;
    let linear = 2.0 * offset.dot(direction) - 2.0 * a0 * a1 - 2.0 * slope * a1 * r0;
    let constant = offset.dot(offset) - a0 * a0 - r0 * r0;
    let mut roots: Vec<f64> = Vec::new();
    if quad.abs() <= 1e-12 {
        if linear.abs() > 1e-12 {
            roots.push(-constant / linear);
        }
    } else {
        let discriminant = linear * linear - 4.0 * quad * constant;
        if discriminant >= 0.0 {
            let root = discriminant.sqrt();
            roots.push((-linear + root) / (2.0 * quad));
            roots.push((-linear - root) / (2.0 * quad));
        }
    }
    let mut best: Option<(Vec3, f64)> = None;
    for s in roots {
        let point = base.add(direction.scale(s));
        // Only the nappe with a NON-NEGATIVE radius is the real carrier.
        if rho_at(rho0, rho1, height, point.sub(frame.origin).dot(axis)) < -tolerance {
            continue;
        }
        let distance = point.sub(old_corner).length();
        if best.map(|(_, best)| distance < best).unwrap_or(true) {
            best = Some((point, distance));
        }
    }
    let (corner, _) = best.ok_or_else(|| {
        KernelRefusal::unsupported(
            KernelStage::Refine,
            "corner_off_carrier",
            "move_faces: the pushed cap plane no longer meets the fixed ruled neighbour \
         (the push drives the corner off the carrier) — refusing",
        )
    })?;
    Ok(corner)
}

/// True iff `curve[t0..t1]` sweeps in the POSITIVE azimuth sense about `frame`
/// (the sense `make_arc` builds), decided by whether its midpoint's azimuth lies
/// inside the positive sweep from the start's to the end's.
pub(super) fn arc_sweeps_forward(
    frame: &crate::RevolutionFrame,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
) -> Result<bool, String> {
    let azimuth = |point: Vec3| {
        let delta = point.sub(frame.origin);
        delta.dot(frame.y_axis).atan2(delta.dot(frame.x_axis))
    };
    let tau = std::f64::consts::TAU;
    let wrap = |angle: f64| {
        let value = angle % tau;
        if value < 0.0 {
            value + tau
        } else {
            value
        }
    };
    let start = azimuth(curve.evaluate(t0)?);
    let end = azimuth(curve.evaluate(t1)?);
    let middle = azimuth(curve.evaluate(0.5 * (t0 + t1))?);
    Ok(wrap(middle - start) <= wrap(end - start))
}

/// The EXACT arc of `plane ∩ cone` between two endpoints that already lie on
/// both, swept in the given sense.
///
/// A cone's plane section is the PROJECTIVE image, from the apex, of the base
/// circle: the ray `apex → X` meets the plane at `apex + (k/((X−apex)·n))·(X−apex)`
/// with `k = c − n·apex`, which is LINEAR in the homogeneous control point — so
/// the image of a rational-quadratic circular arc is a rational-quadratic conic
/// arc on the SAME knot vector, exactly. Azimuth is constant along a cone ray,
/// so the endpoints' azimuths give the base arc directly. This is exact for
/// ELLIPTIC and HYPERBOLIC sections alike (the full hyperbola cannot be one
/// rational Bezier — its weights change sign — but an arc that stays on one
/// nappe can, which is why the sign check below is the only restriction).
///
/// The kernel's own `intersect_plane_quadric` builds a cone section the same way
/// but only ever for the FULL section, so it refuses exactly the hyperbolic case
/// this arc-restricted form supports.
pub(super) fn conic_arc_on_ruled(
    frame: &crate::RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    plane_normal: Vec3,
    plane_c: f64,
    start: Vec3,
    end: Vec3,
    forward_sweep: bool,
    tolerance: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let radius_scale = rho0.abs().max(rho1.abs()).max(1.0);
    let axis = frame.axis;
    // Azimuth is preserved by BOTH section constructions — a cone's central
    // projection from a point on the axis, and a cylinder's axial shear add no
    // azimuthal component — so the arc between two points on the section is the
    // arc between their azimuths on the base circle, exactly.
    let azimuth = |point: Vec3| {
        let delta = point.sub(frame.origin);
        delta.dot(frame.y_axis).atan2(delta.dot(frame.x_axis))
    };
    let tau = std::f64::consts::TAU;
    let wrap = |angle: f64| {
        let value = angle % tau;
        if value < 0.0 {
            value + tau
        } else {
            value
        }
    };
    let (start_angle, sweep, reverse) = if forward_sweep {
        (azimuth(start), wrap(azimuth(end) - azimuth(start)), false)
    } else {
        (azimuth(end), wrap(azimuth(start) - azimuth(end)), true)
    };
    if sweep <= 1e-9 || sweep >= tau - 1e-9 {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "section_degenerate",
            "move_faces: the re-intersected arc degenerates to a point or a full turn \
             — refusing"));
    }
    if (rho1 - rho0).abs() <= 1e-9 * radius_scale {
        // A CYLINDER's plane section is an ellipse, built by the shared
        // `plane_ruled_section_arc` — the same axial-shear image of a base arc
        // `intersect_plane_quadric` uses for the whole section. This arm used
        // to be an honest refusal ("deferred"), which is what stopped a curved
        // fixed edge on a cylinder carrier from rebuilding.
        let mut curve = plane_ruled_section_arc(
            plane_normal.scale(plane_c / plane_normal.dot(plane_normal)),
            plane_normal,
            frame,
            rho0,
            rho1,
            height,
            start_angle,
            sweep,
            tolerance,
        )
        .map_err(|error| {
            KernelRefusal::unsupported(
                KernelStage::Refine,
                "ruled_section",
                error.replace("plane_ruled_section_arc:", "move_faces:"),
            )
        })?;
        if reverse {
            curve = curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
        verify_section_between(&curve, frame, rho0, rho1, height, plane_normal, plane_c, start, end, tolerance)?;
        return Ok(curve);
    }
    let apex = frame
        .origin
        .add(axis.scale(rho0 * height / (rho0 - rho1)));
    let k = plane_c - plane_normal.dot(apex);
    if k.abs() <= tolerance {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "section_through_apex", "move_faces: the section plane passes through the cone apex — refusing"));
    }
    // Base circle at whichever end has the LARGER radius, so it never degenerates.
    let (reference_rho, reference_axial) = if rho0.abs() >= rho1.abs() {
        (rho0.abs(), 0.0)
    } else {
        (rho1.abs(), height)
    };
    if reference_rho <= tolerance {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "degenerate_cone", "move_faces: the cone carrier degenerates to its apex — refusing"));
    }
    let circle = crate::make_arc(
        frame.origin.add(axis.scale(reference_axial)),
        frame.x_axis,
        frame.y_axis,
        reference_rho,
        start_angle,
        start_angle + sweep,
    ).or_refuse(KernelStage::Refine, "make_arc")?;
    let mut mapped: Vec<crate::Vec4> = Vec::with_capacity(circle.control_points.len());
    let mut sign = 0.0f64;
    for control in &circle.control_points {
        let relative = Vec3::new(
            control.x - control.w * apex.x,
            control.y - control.w * apex.y,
            control.z - control.w * apex.z,
        );
        let weight = relative.dot(plane_normal);
        if weight.abs() <= 1e-9 * radius_scale {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "asymptotic_ruling",
                "move_faces: the re-intersected section runs along an asymptotic ruling \
                 of the cone — refusing"));
        }
        if sign == 0.0 {
            sign = weight.signum();
        } else if weight.signum() != sign {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "both_nappes",
                "move_faces: the re-intersected section crosses the cone's apex plane \
                 (both nappes) — refusing"));
        }
        let scaled = relative.scale(k);
        mapped.push(crate::Vec4 {
            x: apex.x * weight + scaled.x,
            y: apex.y * weight + scaled.y,
            z: apex.z * weight + scaled.z,
            w: weight,
        });
    }
    if sign < 0.0 {
        // Homogeneously identical, but the kernel keeps weights positive.
        for control in &mut mapped {
            control.x = -control.x;
            control.y = -control.y;
            control.z = -control.z;
            control.w = -control.w;
        }
    }
    let mut curve = NurbsCurve::new(circle.degree, circle.knots.clone(), mapped).or_refuse(KernelStage::Refine, "curve_new")?;
    if reverse {
        curve = curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
    }
    verify_section_between(
        &curve, frame, rho0, rho1, height, plane_normal, plane_c, start, end, tolerance,
    )?;
    Ok(curve)
}

/// The "reapply the trimming" contract a rebuilt section is held to, shared by
/// both branches of [`conic_arc_on_ruled`] and by the rotation's rim rebuild:
/// the arc must RUN BETWEEN the two given corners and every sample of it must
/// lie on BOTH carriers. The constructions are exact, so this measures rather
/// than fits — it is the same fail-safe [`verify_rim_on_carriers`] applies to
/// the affine rim map.
pub(super) fn verify_section_between(
    curve: &NurbsCurve,
    frame: &crate::RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    plane_normal: Vec3,
    plane_c: f64,
    start: Vec3,
    end: Vec3,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let axis = frame.axis;
    let [d0, d1] = curve.domain().or_refuse(KernelStage::Validate, "domain")?;
    for (parameter, target) in [(d0, start), (d1, end)] {
        let drift = curve.evaluate(parameter).or_refuse(KernelStage::Validate, "evaluate")?.sub(target).length();
        if drift > 10.0 * tolerance {
            return Err(KernelRefusal::internal(KernelStage::Validate, "section_corner", format!(
                "move_faces: the rebuilt section misses its corner by {drift:.3e} — refusing"
            )));
        }
    }
    for step in 0..=8 {
        let point = curve.evaluate(d0 + (d1 - d0) * (step as f64 / 8.0)).or_refuse(KernelStage::Validate, "evaluate")?;
        let delta = point.sub(frame.origin);
        let axial = delta.dot(axis);
        let off_ruled = (delta.sub(axis.scale(axial)).length()
            - rho_at(rho0, rho1, height, axial))
        .abs();
        let off_plane = (point.dot(plane_normal) - plane_c).abs();
        if off_ruled > 10.0 * tolerance || off_plane > 10.0 * tolerance {
            return Err(KernelRefusal::internal(KernelStage::Validate, "section_on_carriers", format!(
                "move_faces: the rebuilt section does not lie on both carriers \
                 (off ruled {off_ruled:.3e}, off plane {off_plane:.3e}) — refusing"
            )));
        }
    }
    Ok(())
}

/// The fixed PLANE and the fixed RULED carrier an edge is shared by, when it is
/// shared by exactly one of each (the flat×cone hyperbola configuration).
pub(super) fn plane_and_ruled_carriers(
    solid: &BrepSolid,
    face_lookup: &HashMap<u64, (usize, usize)>,
    planes: &mut HashMap<u64, Plane>,
    face_ids: &[u64],
    plane_tolerance: f64,
) -> Option<(Plane, (crate::RevolutionFrame, f64, f64, f64))> {
    let mut plane = None;
    let mut ruled = None;
    for &face_id in face_ids {
        if let Some(carrier) = carrier_ruled(solid, face_lookup, face_id) {
            if ruled.is_some() {
                return None;
            }
            ruled = Some(carrier);
        } else if carrier_is_planar(solid, face_lookup, face_id) {
            if plane.is_some() {
                return None;
            }
            plane = Some(cached_plane(planes, solid, face_lookup, face_id, plane_tolerance).ok()?);
        } else {
            return None;
        }
    }
    Some((plane?, ruled?))
}

/// Re-solve a cap corner that is shared with a FIXED ruled carrier (a split
/// cylinder/cone band), which the planar 3-plane `solve_corner` cannot place
/// (its carrier is not a plane). The corner rides the ONE fixed edge it shares
/// with the body: the new corner is where the TRANSLATED cap plane crosses that
/// fixed edge's curve. The fixed edge itself stays put — only its cap-side trim
/// endpoint slides along it (`plan_straight_rebuild` re-lays a straight
/// generatrix between the new and the untouched endpoints afterwards).
///
/// This is the SM1c multi-rim generalisation: the single-closed-rim oblique cap
/// (no corners) is the `fixed_at.len() == 1` rim-ride; a cap bounded by several
/// conic rims meeting at seam corners needs each corner re-placed here.
///
/// - **Straight generatrix (degree 1):** exact line solve. A line is its own
///   natural extension, so a crossing OUTSIDE the fixed edge's current span
///   (the outward push that lengthens the wall) is exact and accepted.
/// - **Conic (degree 2+):** bracket a sign change WITHIN the domain only and
///   refine — a rational conic extended past its span rides the end tangent,
///   not the conic, so an out-of-domain crossing is refused. The root nearest
///   the corner's own end parameter is chosen so the corner moves continuously.
///   This is now a FALLBACK: booleans split such an edge exactly at the corner
///   (`domain == [t0, t1]`), so an outward push has no room to bracket into.
///   When the corner's fixed carriers are one plane and one ruled surface, the
///   caller solves on the CARRIERS instead (`corner_on_plane_and_ruled`), which
///   has no such horizon; this arm only runs for configurations that solve does
///   not cover.
///
/// Refuses cleanly when the fixed edge runs in the cap plane (no crossing) or
/// the push drives the corner off the edge's reachable span (a tearing push).
pub(super) fn resolve_corner_on_fixed_edge(
    fixed_edge: &EdgeRecord,
    seed_param: f64,
    plane_normal: Vec3,
    plane_c: f64,
    tolerance: f64,
) -> Result<Vec3, KernelRefusal> {
    let curve = &fixed_edge.curve;
    if let Some((p0, p1)) = curve.straight_segment(tolerance) {
        let dir = p1.sub(p0);
        let denom = plane_normal.dot(dir);
        if denom.abs() <= PARALLEL_EPS * (1.0 + dir.length()) {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "edge_parallel_to_cap", format!(
                "move_faces: fixed edge {} runs parallel to the cap plane (no crossing) — refusing",
                fixed_edge.id
            )));
        }
        let s = (plane_c - plane_normal.dot(p0)) / denom;
        return Ok(p0.add(dir.scale(s)));
    }
    // Curved (conic) fixed edge — closed-form-in-spirit numeric solve strictly
    // within the domain. `n·C(u) − c` has the sign of the numerator polynomial
    // (weights are strictly positive), so its roots are the crossings.
    let [d0, d1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let f = |u: f64| -> Result<f64, KernelRefusal> { Ok(plane_normal.dot(curve.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?) - plane_c) };
    const STEPS: usize = 96;
    let mut best: Option<(f64, f64)> = None;
    let mut prev_u = d0;
    let mut prev_f = f(d0)?;
    if prev_f.abs() <= tolerance {
        best = Some((d0, (d0 - seed_param).abs()));
    }
    for i in 1..=STEPS {
        let u = d0 + (d1 - d0) * (i as f64 / STEPS as f64);
        let fu = f(u)?;
        if prev_f * fu < 0.0 {
            let (mut lo, mut hi, mut flo) = (prev_u, u, prev_f);
            for _ in 0..64 {
                let mid = 0.5 * (lo + hi);
                let fm = f(mid)?;
                if flo * fm <= 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                    flo = fm;
                }
            }
            let root = 0.5 * (lo + hi);
            let dist = (root - seed_param).abs();
            if best.map(|(_, bd)| dist < bd).unwrap_or(true) {
                best = Some((root, dist));
            }
        }
        prev_u = u;
        prev_f = fu;
    }
    let (root, _) = best.ok_or_else(|| {
        KernelRefusal::unsupported(KernelStage::Refine, "fixed_edge_span", format!(
            "move_faces: the pushed cap does not re-cross fixed edge {} within its span \
             (curved-fixed-edge extension deferred, or the push tears the face) — refusing",
            fixed_edge.id
        ))
    })?;
    curve.evaluate(root).or_refuse(KernelStage::Refine, "evaluate")
}

/// Golovanov §6.12 direct editing — translate a group of faces rigidly and
/// heal the adjacency with the faces that stay behind.
///
/// The moved carriers translate exactly (every control point shifts by the
/// translation, which is exact for ANY surface type), and each boundary edge
/// between a moved face and a fixed face is recomputed as the intersection of
/// the translated moved carrier with the fixed carrier:
///
/// - When the translation is parallel to every fixed plane a boundary vertex
///   touches, the whole neighbourhood translates rigidly — exact for any
///   moved carrier and any edge curve type. This is the extrude-like case:
///   pushing a face along its own normal slides the side walls in-plane.
/// - Otherwise the new corner is re-solved as the common point of ALL carrier
///   planes meeting at the vertex (moved ones translated), and every affected
///   straight edge is rebuilt between the re-solved corners — the same
///   relocate-onto-recovered-corners move `delete_face_and_heal` performs on
///   its side edges.
///
/// Scope (honest refusals, never a bad solid): the moved faces may be any
/// surface type. A FIXED face that must be re-intersected — the fixed side of a
/// boundary edge, or any face whose boundary edges must be rebuilt — must be
/// PLANAR, OR an AXIS-PARALLEL ruled revolution: a cylinder OR a cone whose axis
/// the push is parallel to (SM1/SM1b). Such a carrier keeps its SAME surface and
/// only re-trims — its rim re-intersects it at a new radius, constructed by the
/// exact radial-scale map `EdgeMoveAction::Transform` (`s = rho_at(z+d)/rho_at(z)`;
/// the cylinder is `s = 1`) and verified to lie on both modified carriers, then
/// grown along the axis (`retrim_ruled_face`). Every straight rebuilt edge (a
/// seam/generatrix) is verified on its carrier at its own `rho_at` radius.
///
/// An OBLIQUE cap push is supported on both carriers. The rim is the affine
/// image `rim_ruled_map` (a cylinder's axis translation, a cone's homothety
/// about the apex) — exact for the whole conic — and where that affine's image
/// of a corner disagrees with the corner itself (a CONE, whose homothety slides
/// the arc's endpoint off the fixed flat), the rim and the CURVED fixed edge it
/// meets are RE-BUILT as exact conic sections between the re-solved corners
/// (`conic_arc_on_ruled`, `EdgeMoveAction::Replace`); the corner itself comes
/// from the carrier-level solve `corner_on_plane_and_ruled`. A curved fixed edge
/// rebuilds on a CYLINDER carrier as its ellipse and on a cone as its conic.
/// Spheres (outside their dedicated route), tori, general revolutions and a cap
/// pushed to/through the apex are refused (SM3 is the general offset path).
/// A translation that collapses an adjacent edge to zero length or reverses
/// its direction (moving a box face onto or past its opposite face) is
/// refused, as is a group that tears away from its neighbours. The input is
/// never mutated; the result is returned only when `validate()` is clean.
pub fn move_faces(
    solid: &BrepSolid,
    face_ids: &[u64],
    translation: Vec3,
) -> Result<BrepSolid, KernelRefusal> {
    if !(translation.x.is_finite() && translation.y.is_finite() && translation.z.is_finite()) {
        return Err(KernelRefusal::input(KernelStage::Collect, "translation", "move_faces: translation must be finite"));
    }
    if face_ids.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "selection", "move_faces: no faces selected"));
    }
    let moved: HashSet<u64> = face_ids.iter().copied().collect();
    // face id -> (shell, face) built once. move_faces never mutates `solid`,
    // so this replaces the O(faces) `find_face` scans in the validation loop
    // below and in `cached_plane` (called up to once per unique fixed face).
    // `or_insert` keeps the first match, mirroring `find_face`.
    let mut face_lookup: HashMap<u64, (usize, usize)> = HashMap::default();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            face_lookup
                .entry(face.id)
                .or_insert((shell_index, face_index));
        }
    }
    for &face_id in face_ids {
        if !face_lookup.contains_key(&face_id) {
            return Err(KernelRefusal::input(KernelStage::Collect, "face_id", format!("move_faces: no face with id {face_id}")));
        }
    }

    // THE PARALLEL ROAD. `BREP_FACE_MOVE_ROAD=carrier` re-intersects the
    // selection's UNBOUNDED carriers against the rest of the solid instead of
    // classifying each boundary edge into one of four actions — the
    // construction the operator asked for, built alongside this one so the two
    // can be compared cell by cell on the capability matrix before anything is
    // rewired. With the variable unset this line is the only trace of it.
    if carrier_road_selected() {
        return move_faces_by_carrier(solid, &moved, translation);
    }

    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);
    // "Parallel to a fixed plane" means the translation's normal component
    // could not move any point off that plane at model precision.
    let parallel_tolerance = (translation.length() * 1e-9).max(1e-12);
    // "Rigid" endpoints moved by exactly the translation (they are assigned
    // `point + translation` verbatim, so this only absorbs rounding noise).
    let rigid_tolerance = (scale * 1e-9).max(1e-12);

    // --- Classify every edge by how the group uses it ----------------------
    let mut faces_of_edge: HashMap<u64, Vec<u64>> = HashMap::default();
    for shell in &solid.shells {
        for face in &shell.faces {
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    faces_of_edge
                        .entry(coedge.edge_id)
                        .or_default()
                        .push(face.id);
                }
            }
        }
    }
    // --- The manifold contract, asked once and early -----------------------
    // Hoisted out of the classification loop below so that the identity lane
    // that follows cannot hand a non-manifold document straight back unread.
    // The message is that loop's own, unchanged, and so is the verdict: this is
    // the same test at an earlier line.
    for edge in &solid.edges {
        let uses = faces_of_edge
            .get(&edge.id)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        let expected = if edge.degenerate { 1 } else { 2 };
        if uses.len() != expected {
            return Err(KernelRefusal::input(KernelStage::Collect, "non_manifold", format!(
                "move_faces: edge {} is used {} times (non-manifold input)",
                edge.id,
                uses.len()
            )));
        }
    }

    // --- THE IDENTITY: a translation no selected carrier can feel ----------
    // A plane slid WITHIN itself and a cylinder pushed along its OWN axis are
    // the same statement — `S + T == S` — and the answer to both is the input
    // body. Answered here, ahead of every road below, because each of those
    // chooses its lane from a neighbour's carrier pair BEFORE the motion is
    // looked at: a slide over a fillet band took the curved-fixed-edge road and
    // a bore wall along its axis took the three-plane corner solve, and both
    // refused a motion that moves nothing. See `translation_is_identity_on` for
    // why no guard below is being relaxed by answering first.
    if translation_is_identity_on(solid, &face_lookup, &moved, translation, parallel_tolerance) {
        return Ok(solid.clone());
    }

    // --- Plane × Sphere fast path (backlog #5) -----------------------------
    // A planar push whose FIXED neighbour across a boundary edge is a SPHERE
    // cannot be healed by the planar/ruled machinery below: the sphere's seam
    // meridian is a CURVED fixed edge (which `plan_straight_rebuild` refuses),
    // and its periodic u=0/u=2π seam pcurves must be patched in parameter space,
    // not refit from scratch. Route those to a dedicated handler that
    // re-intersects the translated plane with the fixed sphere (an EXACT circle)
    // and re-trims the sphere. Every configuration that handler does not support
    // refuses cleanly there — and every such case refuses in the generic path
    // today too, so this routing can only turn a refusal into a heal (it never
    // changes an already-supported case).
    let borders_a_sphere = solid.edges.iter().any(|edge| {
        let uses = faces_of_edge
            .get(&edge.id)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        let moved_uses = uses.iter().filter(|f| moved.contains(*f)).count();
        moved_uses > 0
            && moved_uses < uses.len()
            && uses
                .iter()
                .any(|f| !moved.contains(f) && carrier_sphere(solid, &face_lookup, *f).is_some())
    });
    if borders_a_sphere {
        return move_planar_face_across_sphere(solid, &moved, &face_lookup, &faces_of_edge, translation);
    }

    let mut classes: HashMap<u64, EdgeMoveClass> = HashMap::default();
    let mut planes: HashMap<u64, Plane> = HashMap::default();
    for edge in &solid.edges {
        let uses = faces_of_edge
            .get(&edge.id)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        // Manifold use counts were checked above, before the identity lane.
        let moved_uses = uses
            .iter()
            .filter(|face_id| moved.contains(*face_id))
            .count();
        let class = if moved_uses == 0 {
            EdgeMoveClass::Fixed
        } else if moved_uses == uses.len() {
            EdgeMoveClass::Interior
        } else {
            let moved_face = *uses
                .iter()
                .find(|face_id| moved.contains(*face_id))
                .unwrap();
            let fixed_face = *uses
                .iter()
                .find(|face_id| !moved.contains(*face_id))
                .unwrap();
            // The face left behind across a boundary edge is the carrier we
            // re-intersect against. Planar → cache its plane (today path).
            // Non-planar but a ruled revolution (a cylinder OR a cone) → allowed:
            // the cap rim re-intersects the SAME carrier, re-trimmed as a ruled
            // carrier via the exact `rim_ruled_map` for ANY push direction
            // (SM1/SM1b axis-parallel, SM1c oblique). Any other non-planar
            // carrier still refuses.
            if carrier_is_planar(solid, &face_lookup, fixed_face) {
                cached_plane(&mut planes, solid, &face_lookup, fixed_face, plane_tolerance)?;
            } else if carrier_ruled(solid, &face_lookup, fixed_face).is_none() {
                // A geometrically-planar face the recognizer did not TAG as a
                // plane (an imported B-spline patch, say) still passes here —
                // only its message changes. A genuinely curved carrier refuses,
                // and this is the arm it refuses at: the classification loop,
                // long before the face-action loop the census cites.
                cached_plane(&mut planes, solid, &face_lookup, fixed_face, plane_tolerance)
                    .map_err(|_| {
                        unsupported_carrier(
                            solid,
                            &face_lookup,
                            fixed_face,
                            &format!(
                                "the fixed neighbour across boundary edge {}",
                                edge.id
                            ),
                        )
                    })?;
            }
            EdgeMoveClass::Boundary {
                moved_face,
                fixed_face,
            }
        };
        classes.insert(edge.id, class);
    }

    // --- Relocate every vertex the group touches ---------------------------
    let mut vertex_faces: HashMap<u64, HashSet<u64>> = HashMap::default();
    for edge in &solid.edges {
        if let Some(uses) = faces_of_edge.get(&edge.id) {
            for vertex_id in [edge.start_vertex_id, edge.end_vertex_id] {
                vertex_faces
                    .entry(vertex_id)
                    .or_default()
                    .extend(uses.iter().copied());
            }
        }
    }
    let mut new_vertex: HashMap<u64, Vec3> = HashMap::default();
    for vertex in &solid.vertices {
        let Some(adjacent) = vertex_faces.get(&vertex.id) else {
            continue;
        };
        if !adjacent.iter().any(|face_id| moved.contains(face_id)) {
            continue;
        }
        let fixed_at: Vec<u64> = adjacent
            .iter()
            .copied()
            .filter(|face_id| !moved.contains(face_id))
            .collect();
        if fixed_at.is_empty() {
            // Interior vertex: carried rigidly with the group.
            new_vertex.insert(vertex.id, vertex.point.add(translation));
            continue;
        }
        // SM1b/SM1c: a corner on a single ruled neighbour rides that rim's exact
        // affine map — it stays on the (unchanged) cylinder/cone at the re-
        // intersected position. The map is the cap's homothety about the cone
        // apex (or an axis translation for a cylinder) and no longer requires the
        // push to be axis-parallel, so an OBLIQUE cap push relocates the seam
        // vertex onto the new conic rim exactly.
        if fixed_at.len() == 1 {
            if let Some((frame, rho0, rho1, height)) =
                carrier_ruled(solid, &face_lookup, fixed_at[0])
            {
                // The cap sharing this ruled rim is the moved planar face at the
                // vertex; its plane defines the homothety (D₀ from the apex).
                if let Some(cap) = adjacent.iter().copied().find(|f| moved.contains(f)) {
                    let moved_plane =
                        cached_plane(&mut planes, solid, &face_lookup, cap, plane_tolerance)
                            .map_err(|_| {
                                unsupported_carrier(
                                    solid,
                                    &face_lookup,
                                    cap,
                                    &format!(
                                        "the MOVED cap meeting a ruled neighbour at vertex {}",
                                        vertex.id
                                    ),
                                )
                            })?;
                    let map =
                        rim_ruled_map(&frame, rho0, rho1, height, &moved_plane, translation, tolerance)?;
                    new_vertex.insert(vertex.id, map.point(vertex.point));
                    continue;
                }
            }
        }
        // If every fixed carrier is INVARIANT under the push — a plane parallel
        // to it, or an axis-parallel cylinder (e.g. a fillet band) — the corner
        // rides rigidly (stays on all of them + on every translated moved plane).
        if fixed_at.iter().all(|&face_id| {
            carrier_invariant_under(solid, &face_lookup, face_id, translation, parallel_tolerance)
        }) {
            new_vertex.insert(vertex.id, vertex.point.add(translation));
            continue;
        }
        // THE MIRROR CORNER — a MOVED ruled carrier against a fixed FLAT. This
        // is the bore wall the user drags: the moved face is the cylinder (or
        // cone), the fixed neighbour is the flat it opens through, and the rim
        // they meet in is a conic on that flat. Neither road below can place
        // this corner — the three-plane solve because the moved carrier is not
        // a plane, the multi-rim ride because there it is the FIXED side that
        // is curved. What places it is the rim's OWN exact affine: the corner
        // is the image of the old corner under the map that carries
        // `P ∩ C → P ∩ (C + T)`, and that is the same map the rim edge is then
        // built with, so the corner and its rim cannot disagree.
        if let Some((moved_face, frame, rho0, rho1, height)) = moved_ruled_carrier_at(
            solid,
            &face_lookup,
            &moved,
            adjacent,
            translation,
            parallel_tolerance,
        ) {
            // Two fixed flats at the corner give two different maps (each
            // absorbs its own flat's share of the push along the axis) and no
            // single image for the vertex. It refuses by name rather than
            // picking one.
            if fixed_at.len() != 1 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "mirror_corner_flats", format!(
                    "move_faces: the corner at vertex {} where the MOVED {} meets its \
                     neighbours touches {} fixed faces (a pushed cylinder or cone is \
                     re-intersected against exactly one flat there) — refusing",
                    vertex.id,
                    carrier_kind_name(
                        &solid.shells[face_lookup[&moved_face].0].faces[face_lookup[&moved_face].1]
                            .surface
                    ),
                    fixed_at.len()
                )));
            }
            let fixed_face = fixed_at[0];
            let flat = cached_plane(&mut planes, solid, &face_lookup, fixed_face, plane_tolerance)
                .map_err(|_| {
                    unsupported_carrier(
                        solid,
                        &face_lookup,
                        fixed_face,
                        &format!(
                            "the fixed neighbour of the MOVED ruled carrier (face \
                             {moved_face}) at vertex {}",
                            vertex.id
                        ),
                    )
                })?;
            let map =
                moved_ruled_rim_map(&frame, rho0, rho1, height, &flat, translation, tolerance)?;
            let corner = map.point(vertex.point);
            // Checked, never asserted, exactly as every other corner here is:
            // on the flat it must stay on, and on the carrier the push leaves.
            let off_flat = corner.sub(flat.origin).dot(flat.normal).abs();
            if off_flat > 10.0 * tolerance {
                return Err(KernelRefusal::internal(KernelStage::Validate, "mirror_corner_on_flat", format!(
                    "move_faces: the re-solved corner at vertex {} left its fixed flat \
                     (off {off_flat:.3e}) — refusing",
                    vertex.id
                )));
            }
            let pushed = shift_ruled_frame(&frame, translation);
            let delta = corner.sub(pushed.origin);
            let axial = delta.dot(pushed.axis);
            let radial = delta.sub(pushed.axis.scale(axial)).length();
            let off_carrier = (radial - rho_at(rho0, rho1, height, axial)).abs();
            if off_carrier > 10.0 * tolerance {
                return Err(KernelRefusal::internal(KernelStage::Validate, "mirror_corner_on_carrier", format!(
                    "move_faces: the re-solved corner at vertex {} left the MOVED carrier \
                     (face {moved_face}, off {off_carrier:.3e}) — refusing",
                    vertex.id
                )));
            }
            new_vertex.insert(vertex.id, corner);
            continue;
        }
        // SM1c multi-rim: a corner shared with a FIXED ruled carrier (a split
        // cylinder/cone band) cannot be placed by the planar 3-plane solver — its
        // carrier is not a plane. Ride it along the single fixed edge it shares:
        // the new corner is where the TRANSLATED cap plane crosses that fixed
        // edge's curve. Consistent + valid only when the affine rim map that
        // carries the adjacent conic rim agrees with this ride (an axis-parallel
        // cylinder generatrix); a cone homothety moves the corner off the fixed
        // wall, so the consistency gate below refuses that (curved multi-rim
        // cone deferred to the rim re-trim path).
        if fixed_at
            .iter()
            .any(|&f| carrier_ruled(solid, &face_lookup, f).is_some())
        {
            let moved_here: Vec<u64> = adjacent
                .iter()
                .copied()
                .filter(|f| moved.contains(f))
                .collect();
            if moved_here.len() != 1 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "multi_rim_caps", format!(
                    "move_faces: corner at vertex {} touches {} moved faces against a ruled \
                     neighbour (single-cap multi-rim only) — refusing",
                    vertex.id,
                    moved_here.len()
                )));
            }
            let cap_plane =
                cached_plane(&mut planes, solid, &face_lookup, moved_here[0], plane_tolerance)
                    .map_err(|_| {
                        unsupported_carrier(
                            solid,
                            &face_lookup,
                            moved_here[0],
                            &format!("the MOVED cap at vertex {}", vertex.id),
                        )
                    })?;
            let fixed_edges: Vec<&EdgeRecord> = solid
                .edges
                .iter()
                .filter(|e| {
                    (e.start_vertex_id == vertex.id || e.end_vertex_id == vertex.id)
                        && matches!(classes.get(&e.id), Some(EdgeMoveClass::Fixed))
                })
                .collect();
            if fixed_edges.len() != 1 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "multi_rim_fixed_edges", format!(
                    "move_faces: corner at vertex {} rides {} fixed edges against a ruled \
                     neighbour (exactly one required) — refusing",
                    vertex.id,
                    fixed_edges.len()
                )));
            }
            let fixed_edge = fixed_edges[0];
            let seed = if fixed_edge.start_vertex_id == vertex.id {
                fixed_edge.t0
            } else {
                fixed_edge.t1
            };
            let translated_origin = cap_plane.origin.add(translation);
            let plane_c = cap_plane.normal.dot(translated_origin);
            // A STRAIGHT fixed edge (a cylinder generatrix on a flat, or a cone's
            // seam meridian — which passes through the apex, so the rim homothety
            // agrees with it) rides its own line: exact, and unchanged since SM1c.
            // Straightness is geometric: a line rejoined with an interior knot is
            // still a line, and rides too.
            // A CURVED fixed edge (the hyperbola where a flat cuts a cone) is
            // solved on the CARRIERS instead: the boolean split it exactly at the
            // corner, so riding it could only ever move the corner inward.
            let straight_fixed_edge =
                fixed_edge.curve.straight_segment(tolerance).is_some();
            let carriers = if straight_fixed_edge {
                None
            } else {
                plane_and_ruled_carriers(
                    solid,
                    &face_lookup,
                    &mut planes,
                    &fixed_at,
                    plane_tolerance,
                )
            };
            let corner = match carriers {
                Some((fixed_plane, (frame, rho0, rho1, height))) => corner_on_plane_and_ruled(
                    &fixed_plane,
                    cap_plane.normal,
                    plane_c,
                    &frame,
                    rho0,
                    rho1,
                    height,
                    vertex.point,
                    tolerance,
                )?,
                None => resolve_corner_on_fixed_edge(
                    fixed_edge,
                    seed,
                    cap_plane.normal,
                    plane_c,
                    tolerance,
                )?,
            };
            // The new corner must genuinely sit on EVERY fixed carrier at the
            // vertex (ruled: at its rho_at radius; planar: on the plane), else
            // the group tore away — refuse rather than emit a bad solid.
            for &f in &fixed_at {
                if let Some((frame, rho0, rho1, height)) = carrier_ruled(solid, &face_lookup, f) {
                    let delta = corner.sub(frame.origin);
                    let axial = delta.dot(frame.axis);
                    let radial = delta.sub(frame.axis.scale(axial)).length();
                    let off = (radial - rho_at(rho0, rho1, height, axial)).abs();
                    if off > 10.0 * tolerance {
                        return Err(KernelRefusal::unsupported(KernelStage::Validate, "corner_off_ruled_neighbour", format!(
                            "move_faces: re-solved corner at vertex {} left its ruled neighbour \
                             (off {off:.3e}) — refusing",
                            vertex.id
                        )));
                    }
                } else {
                    let plane = cached_plane(&mut planes, solid, &face_lookup, f, plane_tolerance)
                        .map_err(|_| {
                            unsupported_carrier(
                                solid,
                                &face_lookup,
                                f,
                                &format!("a FIXED neighbour at vertex {}", vertex.id),
                            )
                        })?;
                    if corner.sub(plane.origin).dot(plane.normal).abs() > 10.0 * tolerance {
                        return Err(KernelRefusal::unsupported(KernelStage::Validate, "corner_off_planar_neighbour", format!(
                            "move_faces: re-solved corner at vertex {} left a fixed planar \
                             neighbour — refusing",
                            vertex.id
                        )));
                    }
                }
            }
            // The re-solved corner is TRUTH: it is the only point lying on the
            // fixed wall, the fixed flat AND the translated cap plane at once.
            //
            // The conic rim bordering the fixed ruled carrier is carried by the
            // EXACT affine `rim_ruled_map`, which maps the WHOLE conic correctly
            // (carrier → itself, cap plane → translated cap plane). On a CYLINDER
            // — an axis translation along a generatrix of a flat that contains the
            // axis — its image of the old corner IS this corner, so the rim keeps
            // its affine map untouched. On a CONE it is a homothety about the
            // apex: the rim CURVE is still exact but the arc's ENDPOINT slides off
            // the fixed flat, so the rim edge is RE-BUILT between the re-solved
            // corners instead (`EdgeMoveAction::Replace`, see the Boundary arm).
            // This used to be a consistency gate that refused the cone outright.
            new_vertex.insert(vertex.id, corner);
            continue;
        }
        // Genuine re-intersection: every carrier meeting at the corner must
        // be planar to solve the new corner in closed form.
        let mut corner_planes = Vec::with_capacity(fixed_at.len());
        for &face_id in &fixed_at {
            corner_planes.push(
                cached_plane(&mut planes, solid, &face_lookup, face_id, plane_tolerance).map_err(
                    |_| {
                        unsupported_carrier(
                            solid,
                            &face_lookup,
                            face_id,
                            &format!("a FIXED carrier meeting the moved group at vertex {}", vertex.id),
                        )
                    },
                )?,
            );
        }
        for face_id in adjacent
            .iter()
            .copied()
            .filter(|face_id| moved.contains(face_id))
        {
            let mut plane = cached_plane(&mut planes, solid, &face_lookup, face_id, plane_tolerance)
                .map_err(|_| {
                    unsupported_carrier(
                        solid,
                        &face_lookup,
                        face_id,
                        &format!("a MOVED carrier meeting a fixed neighbour at vertex {}", vertex.id),
                    )
                })?;
            plane.origin = plane.origin.add(translation);
            corner_planes.push(plane);
        }
        let corner = solve_corner(&corner_planes).ok_or_else(|| {
            KernelRefusal::ill_posed(KernelStage::Refine, "corner_underconstrained", format!(
                "move_faces: cannot re-intersect the carriers meeting at vertex {} \
                 (parallel or under-constrained planes)",
                vertex.id
            ))
        })?;
        // The corner must genuinely sit on EVERY carrier; otherwise the group
        // tears away from its fixed neighbours and no manifold heal exists.
        for plane in &corner_planes {
            if corner.sub(plane.origin).dot(plane.normal).abs() > tolerance {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "corner_tear", format!(
                    "move_faces: the moved group tears away from its neighbours at \
                     vertex {} — refusing rather than emitting an invalid solid",
                    vertex.id
                )));
            }
        }
        new_vertex.insert(vertex.id, corner);
    }

    // --- Plan every edge update -------------------------------------------
    let vertex_position: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let mut actions: HashMap<u64, EdgeMoveAction> = HashMap::default();
    for edge in &solid.edges {
        let position = |vertex_id: u64| -> Result<Vec3, KernelRefusal> {
            vertex_position.get(&vertex_id).copied().ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "vertex_lookup",
                    format!("move_faces: missing vertex {vertex_id}"),
                )
            })
        };
        let start_old = position(edge.start_vertex_id)?;
        let end_old = position(edge.end_vertex_id)?;
        let start_new = new_vertex
            .get(&edge.start_vertex_id)
            .copied()
            .unwrap_or(start_old);
        let end_new = new_vertex
            .get(&edge.end_vertex_id)
            .copied()
            .unwrap_or(end_old);
        let rigid = start_new.sub(start_old.add(translation)).length() <= rigid_tolerance
            && end_new.sub(end_old.add(translation)).length() <= rigid_tolerance;
        match &classes[&edge.id] {
            EdgeMoveClass::Fixed => {
                if start_new.sub(start_old).length() == 0.0 && end_new.sub(end_old).length() == 0.0
                {
                    continue; // no endpoint relocated — the edge is untouched
                }
                // A fixed side edge follows its re-solved endpoint, exactly as
                // delete_face_and_heal relocates side edges onto recovered
                // corners. Its faces get re-trimmed: planar faces need their
                // plane; a ruled carrier (the drilled-hole seam, or a cone's
                // extended generatrix) re-trims as a ruled carrier.
                for &face_id in &faces_of_edge[&edge.id] {
                    if carrier_ruled(solid, &face_lookup, face_id).is_none() {
                        cached_plane(&mut planes, solid, &face_lookup, face_id, plane_tolerance)
                            .map_err(|_| {
                                unsupported_carrier(
                                    solid,
                                    &face_lookup,
                                    face_id,
                                    &format!(
                                        "a carrier of fixed edge {}, whose trim the push relocates",
                                        edge.id
                                    ),
                                )
                            })?;
                    }
                }
                if !edge.degenerate && edge.curve.straight_segment(tolerance).is_none() {
                    // CURVED fixed edge — the hyperbola where a flat cuts a cone,
                    // whose cap-side endpoint rides along it. BOTH its carriers
                    // stayed put, so the section is unchanged as a SET, but the
                    // boolean split the curve exactly at the corner (`domain ==
                    // [t0, t1]`), leaving no parameter headroom for an outward
                    // push — so the arc is RE-BUILT between its endpoints. (A
                    // straight-chord rebuild, what a straight generatrix gets,
                    // would leave the cone; that is why this used to refuse.)
                    // "Curved" is asked of the GEOMETRY (`straight_segment`), not
                    // of the pole count: a line an offset shell rejoined out of
                    // two pieces carries an interior knot, and counting poles
                    // sent it down this arm to be refused for having two planar
                    // carriers instead of a plane and a ruled one.
                    // Same collapse/inversion guards as `plan_straight_rebuild`,
                    // so a tearing push still refuses.
                    let new_chord = end_new.sub(start_new);
                    if new_chord.length() <= tolerance {
                        return Err(KernelRefusal::unsupported(KernelStage::Refine, "curved_edge_collapse", format!(
                            "move_faces: the translation collapses edge {} to zero length (a \
                             moved face lands exactly on its neighbour) — refusing",
                            edge.id
                        )));
                    }
                    if end_old.sub(start_old).dot(new_chord) <= 0.0 {
                        return Err(KernelRefusal::unsupported(KernelStage::Refine, CURVED_EDGE_INVERSION, format!(
                            "move_faces: the translation inverts edge {} (a moved face passes \
                             beyond its neighbour) — refusing",
                            edge.id
                        )));
                    }
                    let (fixed_plane, (frame, rho0, rho1, height)) = plane_and_ruled_carriers(
                        solid,
                        &face_lookup,
                        &mut planes,
                        &faces_of_edge[&edge.id],
                        plane_tolerance,
                    )
                    .ok_or_else(|| {
                        KernelRefusal::unsupported(KernelStage::Classify, "curved_fixed_edge_pair", format!(
                            "move_faces: curved fixed edge {} is not shared by exactly one plane \
                             and one ruled carrier — refusing",
                            edge.id
                        ))
                    })?;
                    let forward =
                        arc_sweeps_forward(&frame, &edge.curve, edge.t0, edge.t1).or_refuse(KernelStage::Refine, "arc_sweeps_forward")?;
                    let curve = conic_arc_on_ruled(
                        &frame,
                        rho0,
                        rho1,
                        height,
                        fixed_plane.normal,
                        fixed_plane.normal.dot(fixed_plane.origin),
                        start_new,
                        end_new,
                        forward,
                        tolerance,
                    )?;
                    actions.insert(edge.id, EdgeMoveAction::Replace { curve });
                    continue;
                }
                let action =
                    plan_straight_rebuild(edge, start_old, end_old, start_new, end_new, tolerance)?;
                // A chord rebuilt against a ruled carrier must stay ON it — the
                // seam/generatrix endpoints and midpoint at their own rho_at radius.
                if let EdgeMoveAction::Rebuild { start, end } = &action {
                    for &face_id in &faces_of_edge[&edge.id] {
                        if carrier_ruled(solid, &face_lookup, face_id).is_some() {
                            verify_chord_on_carrier(
                                solid,
                                &face_lookup,
                                face_id,
                                *start,
                                *end,
                                moved.contains(&face_id).then_some(translation),
                                tolerance,
                            )?;
                        }
                    }
                }
                actions.insert(edge.id, action);
            }
            EdgeMoveClass::Interior => {
                if rigid {
                    actions.insert(edge.id, EdgeMoveAction::Translate);
                } else {
                    // A tangential translation left the carriers in place, so
                    // an interior edge must stretch between re-solved corners
                    // instead of riding along (both faces are planar-checked —
                    // except a MOVED ruled carrier's own seam generatrix, whose
                    // straight chord the carrier check below proves instead).
                    for &face_id in &faces_of_edge[&edge.id] {
                        if carrier_ruled(solid, &face_lookup, face_id).is_some() {
                            continue;
                        }
                        cached_plane(&mut planes, solid, &face_lookup, face_id, plane_tolerance)
                            .map_err(|_| {
                                unsupported_carrier(
                                    solid,
                                    &face_lookup,
                                    face_id,
                                    &format!(
                                        "a carrier of interior edge {}, which must stretch between \
                                         re-solved corners",
                                        edge.id
                                    ),
                                )
                            })?;
                    }
                    let action = plan_straight_rebuild(
                        edge, start_old, end_old, start_new, end_new, tolerance,
                    )?;
                    if let EdgeMoveAction::Rebuild { start, end } = &action {
                        for &face_id in &faces_of_edge[&edge.id] {
                            if carrier_ruled(solid, &face_lookup, face_id).is_some() {
                                verify_chord_on_carrier(
                                    solid,
                                    &face_lookup,
                                    face_id,
                                    *start,
                                    *end,
                                    moved.contains(&face_id).then_some(translation),
                                    tolerance,
                                )?;
                            }
                        }
                    }
                    actions.insert(edge.id, action);
                }
            }
            EdgeMoveClass::Boundary {
                moved_face,
                fixed_face,
            } => {
                if let Some((frame, rho0, rho1, height)) =
                    carrier_ruled(solid, &face_lookup, *fixed_face)
                {
                    // The rim re-intersects the (unchanged) ruled carrier. Build
                    // that trim as the EXACT affine map of the rim (= the
                    // intersection of the translated cap plane with the carrier):
                    // a homothety about the cone apex, or an axis translation for
                    // a cylinder — for ANY push direction. Then verify it lies on
                    // both modified carriers (the "reapply the trimming"
                    // contract). The moved side must be planar (a cap);
                    // `cached_plane` refuses otherwise.
                    let moved_plane = cached_plane(
                        &mut planes,
                        solid,
                        &face_lookup,
                        *moved_face,
                        plane_tolerance,
                    )
                    .map_err(|_| {
                        unsupported_carrier(
                            solid,
                            &face_lookup,
                            *moved_face,
                            &format!(
                                "the MOVED side of boundary edge {} against a ruled neighbour",
                                edge.id
                            ),
                        )
                    })?;
                    let map =
                        rim_ruled_map(&frame, rho0, rho1, height, &moved_plane, translation, tolerance)?;
                    verify_rim_on_carriers(
                        edge,
                        &map,
                        &frame,
                        rho0,
                        rho1,
                        height,
                        &moved_plane,
                        translation,
                        tolerance,
                    )?;
                    // The affine carries the WHOLE conic exactly, but a CONE's
                    // homothety about the apex slides the ARC's endpoints off the
                    // fixed flat the corners must stay on — and the boolean left
                    // the rim with no parameter headroom (`domain == [t0, t1]`),
                    // so it cannot simply be re-trimmed either. When the map and
                    // the re-solved corners disagree, RE-BUILD the rim as the
                    // exact section of the TRANSLATED cap plane with the carrier,
                    // between those corners. A closed rim (one vertex, no corner)
                    // and a cylinder (whose map already lands on the corners) take
                    // the untouched affine path.
                    let mut replacement = None;
                    if edge.start_vertex_id != edge.end_vertex_id {
                        let old_start = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
                        let old_end = edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Refine, "evaluate")?;
                        let rim_start = new_vertex
                            .get(&edge.start_vertex_id)
                            .copied()
                            .unwrap_or(old_start);
                        let rim_end = new_vertex
                            .get(&edge.end_vertex_id)
                            .copied()
                            .unwrap_or(old_end);
                        let drift = map
                            .point(old_start)
                            .sub(rim_start)
                            .length()
                            .max(map.point(old_end).sub(rim_end).length());
                        if drift > 10.0 * tolerance {
                            let forward =
                                arc_sweeps_forward(&frame, &edge.curve, edge.t0, edge.t1).or_refuse(KernelStage::Refine, "arc_sweeps_forward")?;
                            let plane_c = moved_plane
                                .normal
                                .dot(moved_plane.origin.add(translation));
                            replacement = Some(conic_arc_on_ruled(
                                &frame,
                                rho0,
                                rho1,
                                height,
                                moved_plane.normal,
                                plane_c,
                                rim_start,
                                rim_end,
                                forward,
                                tolerance,
                            )?);
                        }
                    }
                    actions.insert(
                        edge.id,
                        match replacement {
                            Some(curve) => EdgeMoveAction::Replace { curve },
                            None => EdgeMoveAction::Transform(map),
                        },
                    );
                } else if carrier_is_planar(solid, &face_lookup, *fixed_face) {
                    let fixed_plane = planes[fixed_face];
                    if rigid && translation.dot(fixed_plane.normal).abs() <= parallel_tolerance {
                        // The whole edge slides inside the fixed plane while
                        // staying on the translated moved carrier — exact for any
                        // curve type, no re-intersection needed.
                        actions.insert(edge.id, EdgeMoveAction::Translate);
                    } else if let Some((frame, rho0, rho1, height)) =
                        carrier_ruled(solid, &face_lookup, *moved_face).filter(|_| {
                            !carrier_invariant_under(
                                solid,
                                &face_lookup,
                                *moved_face,
                                translation,
                                parallel_tolerance,
                            )
                        })
                    {
                        // THE MIRROR RIM — the pushed bore's mouth. The same
                        // `plane ∩ quadric` the arm above re-intersects, with the
                        // roles swapped: here the CYLINDER (or cone) is what
                        // moved and the FLAT is what stayed, so the rim's exact
                        // affine is `moved_ruled_rim_map` rather than
                        // `rim_ruled_map`. It carries the whole conic — a closed
                        // mouth included — so the parametrisation and every
                        // coedge's sense survive the push untouched.
                        let map = moved_ruled_rim_map(
                            &frame,
                            rho0,
                            rho1,
                            height,
                            &fixed_plane,
                            translation,
                            tolerance,
                        )?;
                        verify_rim_on_carriers(
                            edge,
                            &map,
                            &shift_ruled_frame(&frame, translation),
                            rho0,
                            rho1,
                            height,
                            &fixed_plane,
                            Vec3::default(),
                            tolerance,
                        )?;
                        // The affine carries the conic, but it only agrees with
                        // the corners the vertex pass solved when both ends of
                        // this rim ride the SAME flat. An open rim whose two ends
                        // sit on different flats has two maps and no single
                        // affine; it refuses by name rather than tearing.
                        if edge.start_vertex_id != edge.end_vertex_id {
                            let drift = map
                                .point(start_old)
                                .sub(start_new)
                                .length()
                                .max(map.point(end_old).sub(end_new).length());
                            if drift > 10.0 * tolerance {
                                return Err(KernelRefusal::unsupported(KernelStage::Refine, "mirror_rim_flats", format!(
                                    "move_faces: the rim of boundary edge {} does not reach the corners the \
                                     push re-solved (off {drift:.3e}) — its two ends ride \
                                     different fixed flats, which no single affine carries — \
                                     refusing",
                                    edge.id
                                )));
                            }
                        }
                        actions.insert(edge.id, EdgeMoveAction::Transform(map));
                    } else {
                        // Real re-intersection: line = translated moved plane ∩
                        // fixed plane, delimited by the re-solved corners. The
                        // moved side must be planar for the chord to stay on it.
                        cached_plane(
                            &mut planes,
                            solid,
                            &face_lookup,
                            *moved_face,
                            plane_tolerance,
                        )
                        .map_err(|_| {
                            unsupported_carrier(
                                solid,
                                &face_lookup,
                                *moved_face,
                                &format!(
                                    "the MOVED side of boundary edge {} against a planar neighbour",
                                    edge.id
                                ),
                            )
                        })?;
                        actions.insert(
                            edge.id,
                            plan_straight_rebuild(
                                edge, start_old, end_old, start_new, end_new, tolerance,
                            )?,
                        );
                    }
                } else {
                    return Err(KernelRefusal::unsupported(KernelStage::Classify, "sm3_carrier", format!(
                        "move_faces: boundary edge {} borders a non-planar, non-axis-parallel \
                         carrier — refusing (SM3 territory)",
                        edge.id
                    )));
                }
            }
        }
    }

    // --- Plan face updates -------------------------------------------------
    // Edges whose SHAPE changed — rebuilt straight OR affine-transformed rim. A
    // moved face bounding one of these must be RE-TRIMMED (its rim moved to a new
    // curve), not merely surface-translated; else its pcurve goes stale.
    let reshaped: HashSet<u64> = actions
        .iter()
        .filter(|(_, action)| {
            matches!(
                action,
                EdgeMoveAction::Rebuild { .. }
                    | EdgeMoveAction::Transform(_)
                    | EdgeMoveAction::Replace { .. }
            )
        })
        .map(|(edge_id, _)| *edge_id)
        .collect();
    let dirty: HashSet<u64> = actions.keys().copied().collect();
    let mut face_actions: Vec<(usize, usize, FaceMoveAction)> = Vec::new();
    // Fixed cylinder neighbours re-trim in a separate post-pass (they grow the
    // carrier via `extend_ruled_neighbour_over`, which needs `&mut solid`).
    let mut ruled_retrim_faces: Vec<u64> = Vec::new();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            let edge_ids = || {
                face.loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .map(|coedge| coedge.edge_id)
            };
            if moved.contains(&face.id) {
                if edge_ids().any(|edge_id| reshaped.contains(&edge_id))
                    && carrier_ruled(solid, &face_lookup, face.id).is_some()
                    && !carrier_invariant_under(
                        solid,
                        &face_lookup,
                        face.id,
                        translation,
                        parallel_tolerance,
                    )
                {
                    // THE PUSHED WALL ITSELF. Its rims were re-intersected, so
                    // the pcurves on record are stale — a bore whose mouths sat
                    // at one station of its own carrier now meets the flats at
                    // another, and in the wall's own parameters that is a shift
                    // in v, not the curve on file. Shifting the control net is
                    // exact (a translated cylinder is the same cylinder moved),
                    // and the re-trim afterwards rebuilds the loops on THAT
                    // carrier, grown along its axis to cover the new rims.
                    face_actions.push((shell_index, face_index, FaceMoveAction::TranslateSurface));
                    ruled_retrim_faces.push(face.id);
                } else if edge_ids().any(|edge_id| reshaped.contains(&edge_id)) {
                    // A boundary edge changed shape (stretched, or a cap rim grown
                    // by the radial-scale map), so the patch must be re-trimmed
                    // around it; the moved cap is planar (translated), so its plane
                    // simply shifts by `translation` before the retrim.
                    let mut plane =
                        cached_plane(&mut planes, solid, &face_lookup, face.id, plane_tolerance)
                            .map_err(|_| {
                                unsupported_carrier(
                                    solid,
                                    &face_lookup,
                                    face.id,
                                    "the MOVED face whose rim the push reshaped",
                                )
                            })?;
                    plane.origin = plane.origin.add(translation);
                    face_actions.push((shell_index, face_index, FaceMoveAction::Retrim(plane)));
                } else {
                    // Every edge of the face rode along rigidly: shifting the
                    // control net keeps surface, curves, and pcurves in exact
                    // agreement for ANY carrier type.
                    face_actions.push((shell_index, face_index, FaceMoveAction::TranslateSurface));
                }
            } else if edge_ids().any(|edge_id| dirty.contains(&edge_id)) {
                if carrier_is_planar(solid, &face_lookup, face.id) {
                    let plane =
                        cached_plane(&mut planes, solid, &face_lookup, face.id, plane_tolerance)?;
                    face_actions.push((shell_index, face_index, FaceMoveAction::Retrim(plane)));
                } else if carrier_ruled(solid, &face_lookup, face.id).is_some() {
                    // Ruled carrier (drilled hole, boss cap, OR a cone whose trim
                    // extended under an oblique cap push): grow it along its axis +
                    // recompute pcurves in the post-pass.
                    ruled_retrim_faces.push(face.id);
                } else {
                    // Non-planar, non-ruled fixed carrier — refuse (the general
                    // offset path does the genuine curved re-intersection).
                    //
                    // MEASURED, and it corrects a just-landed claim: this arm is
                    // NOT the one a curved fixed neighbour reaches. A curved
                    // neighbour that borders the moved group is refused by the
                    // classification loop's own carrier gate (search
                    // `the fixed neighbour across boundary edge`) hundreds of
                    // lines earlier, so this arm can only be entered by a face
                    // whose edges went dirty WITHOUT it sharing a boundary edge
                    // with the moved group — which needs a vertex of valence
                    // four or more. The earlier census, reading the code rather
                    // than running it, attributed the Plane × Torus refusal to
                    // this arm; the probe (`examples/plane_push_refusal_probe.rs`,
                    // `carrier/torus.*`) shows the classification arm firing.
                    cached_plane(&mut planes, solid, &face_lookup, face.id, plane_tolerance)
                        .map_err(|_| {
                            unsupported_carrier(
                                solid,
                                &face_lookup,
                                face.id,
                                "a FIXED face the push must re-trim",
                            )
                        })?;
                }
            }
        }
    }

    // --- Apply to a fresh clone (the input is never touched) ---------------
    let translate = AffineTransform::new([
        1.0,
        0.0,
        0.0,
        translation.x,
        0.0,
        1.0,
        0.0,
        translation.y,
        0.0,
        0.0,
        1.0,
        translation.z,
        0.0,
        0.0,
        0.0,
        1.0,
    ]).or_refuse(KernelStage::Sew, "affine")?;
    let mut result = solid.clone();
    for edge in &mut result.edges {
        match actions.get(&edge.id) {
            Some(EdgeMoveAction::Translate) => {
                edge.curve = transform_curve(&edge.curve, translate).or_refuse(KernelStage::Sew, "transform_curve")?;
            }
            Some(EdgeMoveAction::Rebuild { start, end }) => {
                edge.curve = make_line(*start, *end).or_refuse(KernelStage::Sew, "make_line")?;
                edge.t0 = 0.0;
                edge.t1 = 1.0;
            }
            Some(EdgeMoveAction::Transform(map)) => {
                // Affine-map the rim (radial scale about the axis + translate).
                // Rational-quadratic circles map exactly; parameters unchanged.
                edge.curve = transform_curve(&edge.curve, *map).or_refuse(KernelStage::Sew, "transform_curve")?;
            }
            Some(EdgeMoveAction::Replace { curve }) => {
                // An exactly rebuilt conic arc spans its whole domain by
                // construction, so the trim is the domain.
                let [d0, d1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
                edge.curve = curve.clone();
                edge.t0 = d0;
                edge.t1 = d1;
            }
            None => {}
        }
    }
    for vertex in &mut result.vertices {
        if let Some(point) = new_vertex.get(&vertex.id) {
            vertex.point = *point;
        }
    }
    let final_edges: HashMap<u64, EdgeRecord> = result
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    for (shell_index, face_index, action) in face_actions {
        let face = &mut result.shells[shell_index].faces[face_index];
        match action {
            FaceMoveAction::TranslateSurface => {
                face.surface = transform_surface(&face.surface, translate).or_refuse(KernelStage::Sew, "transform_surface")?;
            }
            FaceMoveAction::Retrim(plane) => {
                retrim_planar_face(face, &plane, &final_edges, scale, "move_faces")?;
                // `retrim_planar_face` maps each edge's WHOLE curve onto the
                // rebuilt plane, so any edge that represents a strict SUBRANGE
                // of its curve (e.g. a box wall edge a fillet trimmed back)
                // needs the range fitter to span exactly [t0, t1]; full-domain
                // edges keep the exact affine pcurve just built. Same pairing
                // the delete/heal planar retrim uses.
                let face_edges: HashSet<u64> = face
                    .loops
                    .iter()
                    .flat_map(|loop_record| loop_record.coedges.iter().map(|c| c.edge_id))
                    .collect();
                refit_touched_pcurves(
                    face,
                    &final_edges,
                    &face_edges,
                    true,
                    tolerance,
                    "move_faces",
                )?;
            }
        }
    }
    // SM1: fixed ruled (cylinder) neighbours grow along their axis and recompute
    // pcurves on the grown carrier — a separate pass because it needs `&mut result`.
    for face_id in ruled_retrim_faces {
        retrim_ruled_face(&mut result, face_id, &final_edges, tolerance, "move_faces")?;
    }

    // Topology (and therefore genus) is untouched — only geometry moved — so
    // validate() re-checks Euler, loop closure, and pcurve agreement.
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "validate", format!(
            "move_faces: moved solid failed validation: {issues:?}"
        )));
    }
    // Belt and braces on top of the per-edge inversion guard: a global
    // inversion flips the signed volume even if every edge kept its direction.
    if let (Ok(before), Ok(after)) = (solid_signed_volume(solid), solid_signed_volume(&result)) {
        if before * after <= 0.0 {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, SOLID_INVERSION,
                "move_faces: the translation inverts the solid (signed volume changed sign) \
                 — refusing"));
        }
    }
    Ok(result)
}

/// Backlog #5 (Plane × Sphere): push a single PLANAR face whose FIXED boundary
/// neighbour(s) are SPHERES — e.g. a flat capping a spherical dome / spherical-
/// bottomed pocket. The moved plane translates; where it borders a fixed sphere
/// the new boundary rim is the EXACT circle `translated-plane ∩ sphere` (built
/// seam-aligned from the sphere's own frame so its pcurve crosses the seam
/// cleanly), the sphere's coupled seam meridian slides its rim endpoint to the
/// new latitude (its pcurve is patched in parameter space — the periodic u=0/u=2π
/// pairing is preserved), and both carriers are re-trimmed to the new rim.
///
/// Honest scope (SPHERE neighbours only, this slice): only the AXIS-PERPENDICULAR
/// single-closed-circle rim is supported. Refused cleanly (never a bad solid): a
/// moved GROUP, a moved face that is not planar, any FIXED neighbour that is not
/// a sphere (planar corner re-solve / torus / general revolution are other
/// slices), an OBLIQUE plane × sphere rim, an open / multi-edge rim, a sphere
/// seam that does not lie where the plane re-intersects it, a vanished / tangent
/// cap, and a push that collapses or inverts the solid.
fn move_planar_face_across_sphere(
    solid: &BrepSolid,
    moved: &HashSet<u64>,
    face_lookup: &HashMap<u64, (usize, usize)>,
    faces_of_edge: &HashMap<u64, Vec<u64>>,
    translation: Vec3,
) -> Result<BrepSolid, KernelRefusal> {
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let plane_tolerance = (scale * 1e-6).max(1e-7);

    // Single moved planar face only (groups deferred — they would need the
    // generic corner re-solve against the remaining planar neighbours).
    if moved.len() != 1 {
        return Err(KernelRefusal::unsupported(KernelStage::Collect, "sphere_group",
            "move_faces: a moved GROUP against a sphere neighbour is deferred \
             (single planar face only) — refusing"));
    }
    let moved_id = *moved.iter().next().unwrap();
    let &(mshell, mface) = face_lookup.get(&moved_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "moved_face_lookup",
            format!("move_faces: missing moved face {moved_id}"),
        )
    })?;
    // The moved face must itself be planar (it translates rigidly).
    let moved_plane = plane_of_surface(
        &solid.shells[mshell].faces[mface].surface,
        plane_tolerance,
        "move_faces",
    )
    .map_err(|message| {
        KernelRefusal::unsupported(KernelStage::Classify, "sphere_moved_non_planar", message)
    })?;
    let translated_origin = moved_plane.origin.add(translation);

    let edge_by_id: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|e| (e.id, e)).collect();

    // --- Build every sphere rim of the moved planar face -------------------
    struct SphereRim {
        edge_id: u64,
        sphere_id: u64,
        closure_vertex: u64,
        new_circle: NurbsCurve,
        closure_point: Vec3,
        v_rim: f64,
    }
    let mut rims: Vec<SphereRim> = Vec::new();
    let mut closure_vertices: HashSet<u64> = HashSet::default();
    let mut sphere_ids: HashSet<u64> = HashSet::default();

    for loop_record in &solid.shells[mshell].faces[mface].loops {
        for coedge in &loop_record.coedges {
            let edge = *edge_by_id
                .get(&coedge.edge_id)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Refine,
                        "rim_edge_lookup",
                        format!("move_faces: missing edge {}", coedge.edge_id),
                    )
                })?;
            let uses = faces_of_edge
                .get(&edge.id)
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            // The one fixed face across this boundary edge.
            let neighbour = uses.iter().copied().find(|f| *f != moved_id);
            let Some(neighbour) = neighbour else {
                return Err(KernelRefusal::internal(KernelStage::Classify, "own_edge", format!(
                    "move_faces: the moved face borders itself along edge {} \
                     (unexpected own edge) — refusing",
                    edge.id
                )));
            };
            let Some((frame, radius)) = carrier_sphere(solid, face_lookup, neighbour) else {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "sphere_mixed_neighbours", format!(
                    "move_faces: the moved planar face borders a non-sphere fixed \
                     neighbour along edge {} (mixed / planar-corner / torus / \
                     revolution neighbours are other slices) — refusing",
                    edge.id
                )));
            };
            // A single CLOSED-circle rim only (start == end vertex).
            if edge.start_vertex_id != edge.end_vertex_id {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "sphere_open_rim", format!(
                    "move_faces: the plane × sphere rim (edge {}) is not a single closed \
                     circle (open / multi-edge rims are deferred) — refusing",
                    edge.id
                )));
            }
            let center = frame.origin;
            let axis = frame.axis;
            // Only the axis-perpendicular cap keeps the rim a fixed-latitude
            // circle we can build seam-aligned; an oblique section crossing the
            // seam is deferred (matches the sphere-pushed path's own refusal).
            if moved_plane.normal.dot(axis).abs() < 1.0 - 1e-6 {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "sphere_oblique_rim", format!(
                    "move_faces: an OBLIQUE plane × sphere rim (edge {}) is deferred \
                     (only axis-perpendicular caps) — refusing",
                    edge.id
                )));
            }
            // Signed axial offset of the TRANSLATED plane from the sphere centre;
            // the new rim is the small circle of radius √(r²−a²) at that height.
            let a = translated_origin.sub(center).dot(axis);
            let rr2 = radius * radius - a * a;
            if rr2 <= tolerance * tolerance {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "sphere_cap_vanishes", format!(
                    "move_faces: the pushed plane no longer meets the sphere (edge {}: the \
                     cap vanishes / is tangent) — refusing",
                    edge.id
                )));
            }
            let radius_new = rr2.sqrt();
            let circle_center = center.add(axis.scale(a));
            let mut new_circle = crate::make_arc(
                circle_center,
                frame.x_axis,
                frame.y_axis,
                radius_new,
                0.0,
                std::f64::consts::TAU,
            ).or_refuse(KernelStage::Refine, "make_arc")?;
            // Match the new rim's traversal to the old edge so the preserved
            // coedge `forward` flags keep both loops' winding consistent. A
            // closed circle's reversal keeps its start point, so the seam-aligned
            // closure point is unaffected.
            let closure_old = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?;
            let old_tan = edge
                .curve
                .evaluate(edge.t0 + 0.01 * (edge.t1 - edge.t0)).or_refuse(KernelStage::Refine, "evaluate")?
                .sub(closure_old);
            let [c0, c1] = new_circle.domain().or_refuse(KernelStage::Refine, "domain")?;
            let new_start = new_circle.evaluate(c0).or_refuse(KernelStage::Refine, "evaluate")?;
            let new_tan = new_circle.evaluate(c0 + 0.01 * (c1 - c0)).or_refuse(KernelStage::Refine, "evaluate")?.sub(new_start);
            if old_tan.dot(new_tan) < 0.0 {
                new_circle = new_circle.reversed().or_refuse(KernelStage::Refine, "reversed")?;
            }
            let closure_point = new_circle
                .evaluate(new_circle.domain().or_refuse(KernelStage::Refine, "domain")?[0])
                .or_refuse(KernelStage::Refine, "evaluate")?;

            // The rim latitude in the (unchanged) sphere's (u, v) space, read off
            // the seam-crossing pcurve (constant v). The coupled seam meridian's
            // rim endpoint slides to this v.
            let (nshell, nface) = find_face(solid, neighbour)
                .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Refine,
                    "sphere_lookup",
                    format!("move_faces: missing sphere neighbour {neighbour}"),
                )
            })?;
            let sphere_surface = &solid.shells[nshell].faces[nface].surface;
            let rim_pcurve = build_pcurve_on_surface(sphere_surface, &new_circle).or_refuse(KernelStage::Refine, "build_pcurve_on_surface")?;
            let [q0, q1] = rim_pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let v_rim = rim_pcurve.evaluate(0.5 * (q0 + q1)).or_refuse(KernelStage::Refine, "evaluate")?.y;

            closure_vertices.insert(edge.start_vertex_id);
            sphere_ids.insert(neighbour);
            rims.push(SphereRim {
                edge_id: edge.id,
                sphere_id: neighbour,
                closure_vertex: edge.start_vertex_id,
                new_circle,
                closure_point,
                v_rim,
            });
        }
    }
    if rims.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Classify, "sphere_rim_missing", "move_faces: no plane × sphere rim found — refusing"));
    }
    // Per closure vertex: where it moves + its new rim latitude (for the meridian).
    let closure_of: HashMap<u64, (Vec3, f64)> = rims
        .iter()
        .map(|r| (r.closure_vertex, (r.closure_point, r.v_rim)))
        .collect();

    // --- Coupled seam meridians (own-only edges of the fixed sphere whose rim
    // endpoint is one of the moved closure vertices) ------------------------
    struct MeridianRetrim {
        edge_id: u64,
        sphere_id: u64,
        moved_end_is_start: bool,
        new_t: f64,
        moved_vertex_old: Vec3,
        moved_vertex_new: Vec3,
        v_rim: f64,
    }
    let mut meridians: Vec<MeridianRetrim> = Vec::new();
    for edge in &solid.edges {
        if edge.degenerate {
            continue; // a pole degeneracy does not move
        }
        let uses = faces_of_edge
            .get(&edge.id)
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        // Own-only to exactly one of the fixed spheres (a seam meridian).
        let owner = uses.first().copied();
        let Some(owner) = owner else { continue };
        if !sphere_ids.contains(&owner) || !uses.iter().all(|f| *f == owner) {
            continue;
        }
        let start_is_closure = closure_vertices.contains(&edge.start_vertex_id);
        let end_is_closure = closure_vertices.contains(&edge.end_vertex_id);
        if !start_is_closure && !end_is_closure {
            continue; // an uncoupled seam meridian: untouched
        }
        if start_is_closure && end_is_closure {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "sphere_zone", format!(
                "move_faces: sphere seam meridian {} moves at BOTH ends (a sphere zone \
                 with two pushed rims) — deferred, refusing",
                edge.id
            )));
        }
        let moved_end_is_start = start_is_closure;
        let closure_vertex = if moved_end_is_start {
            edge.start_vertex_id
        } else {
            edge.end_vertex_id
        };
        let (moved_vertex_new, v_rim) = *closure_of
            .get(&closure_vertex)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Refine,
                    "meridian_pairing",
                    "move_faces: seam meridian is not paired with a rim — refusing",
                )
            })?;
        // Slide the moved endpoint along the (unchanged) meridian curve. The
        // curve is fixed (the sphere is fixed), so this only re-parametrises the
        // trim; project the new rim point onto it and verify it truly lands there
        // (the safety net if `frame.x_axis` were not the seam azimuth).
        let projection = project_point_to_curve(&edge.curve, moved_vertex_new).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
        if projection.distance > 10.0 * tolerance {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "seam_off_rim", format!(
                "move_faces: the re-intersected rim point does not lie on the sphere seam \
                 meridian {} (off {:.3e}) — refusing",
                edge.id, projection.distance
            )));
        }
        let new_t = projection.u;
        let fixed_t = if moved_end_is_start { edge.t1 } else { edge.t0 };
        let old_moved_t = if moved_end_is_start { edge.t0 } else { edge.t1 };
        // The trim must neither collapse nor invert: the moved parameter must
        // stay on the same side of the fixed endpoint as before, with a real span.
        let [dom0, dom1] = edge.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let span = (dom1 - dom0).max(1e-12);
        if (new_t - fixed_t) * (old_moved_t - fixed_t) <= 0.0
            || (new_t - fixed_t).abs() <= 1e-7 * span
            || edge.curve.evaluate(new_t).or_refuse(KernelStage::Refine, "evaluate")?.sub(edge.curve.evaluate(fixed_t).or_refuse(KernelStage::Refine, "evaluate")?).length() <= tolerance
        {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "meridian_collapse", format!(
                "move_faces: the sphere seam meridian {} trim collapses or inverts under \
                 the push — refusing",
                edge.id
            )));
        }
        let moved_vertex_old = edge_point(solid, closure_vertex)?;
        meridians.push(MeridianRetrim {
            edge_id: edge.id,
            sphere_id: owner,
            moved_end_is_start,
            new_t,
            moved_vertex_old,
            moved_vertex_new,
            v_rim,
        });
    }

    // Every moved-face boundary vertex must be an accounted-for rim closure
    // vertex; anything else means an unmodelled corner (refuse rather than leave
    // a vertex un-relocated and fail late).
    for loop_record in &solid.shells[mshell].faces[mface].loops {
        for coedge in &loop_record.coedges {
            let edge = *edge_by_id
                .get(&coedge.edge_id)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Refine,
                        "boundary_edge_lookup",
                        format!("move_faces: missing edge {}", coedge.edge_id),
                    )
                })?;
            for v in [edge.start_vertex_id, edge.end_vertex_id] {
                if !closure_vertices.contains(&v) {
                    return Err(KernelRefusal::unsupported(KernelStage::Classify, "unmodelled_corner", format!(
                        "move_faces: the moved face has a boundary vertex {v} that is not a \
                         sphere-rim closure — refusing",
                    )));
                }
            }
        }
    }

    // --- Apply to a fresh clone (the input is never mutated) ---------------
    let mut result = solid.clone();
    // Rim edges take their new circle.
    for rim in &rims {
        if let Some(edge) = result.edges.iter_mut().find(|e| e.id == rim.edge_id) {
            let [d0, d1] = rim.new_circle.domain().or_refuse(KernelStage::Sew, "domain")?;
            edge.curve = rim.new_circle.clone();
            edge.t0 = d0;
            edge.t1 = d1;
        }
    }
    // Seam meridians keep their curve; only the moved-end trim slides.
    for mer in &meridians {
        if let Some(edge) = result.edges.iter_mut().find(|e| e.id == mer.edge_id) {
            if mer.moved_end_is_start {
                edge.t0 = mer.new_t;
            } else {
                edge.t1 = mer.new_t;
            }
        }
    }
    // Relocate the rim closure vertices.
    for rim in &rims {
        if let Some(v) = result.vertices.iter_mut().find(|v| v.id == rim.closure_vertex) {
            v.point = rim.closure_point;
        }
    }

    // The moved planar face rides the translated plane and re-trims around its
    // new (smaller/larger) rim circle.
    let final_edges: HashMap<u64, EdgeRecord> =
        result.edges.iter().map(|e| (e.id, e.clone())).collect();
    {
        let mut plane = moved_plane;
        plane.origin = plane.origin.add(translation);
        let face = &mut result.shells[mshell].faces[mface];
        retrim_planar_face(face, &plane, &final_edges, scale, "move_faces")?;
    }

    // Re-trim every fixed sphere: rebuild the rim coedge pcurve on the (unchanged)
    // sphere surface, and patch each coupled seam-meridian coedge's pcurve in
    // parameter space (keep u — preserving the periodic u=0/u=2π pairing — and
    // slide only the moved endpoint's v to the new rim latitude).
    for sphere_id in &sphere_ids {
        let (nshell, nface) = find_face(&result, *sphere_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "sphere_retrim_lookup",
                    format!("move_faces: missing sphere neighbour {sphere_id}"),
                )
            })?;
        let sphere_surface = result.shells[nshell].faces[nface].surface.clone();
        for loop_record in &mut result.shells[nshell].faces[nface].loops {
            for coedge in &mut loop_record.coedges {
                if let Some(rim) = rims
                    .iter()
                    .find(|r| r.edge_id == coedge.edge_id && r.sphere_id == *sphere_id)
                {
                    let mut pcurve = build_pcurve_on_surface(&sphere_surface, &rim.new_circle).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?;
                    if !coedge.forward {
                        pcurve = pcurve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
                    }
                    coedge.pcurve = pcurve;
                } else if let Some(mer) = meridians
                    .iter()
                    .find(|m| m.edge_id == coedge.edge_id && m.sphere_id == *sphere_id)
                {
                    coedge.pcurve = patch_seam_meridian_pcurve(
                        &coedge.pcurve,
                        &sphere_surface,
                        mer.moved_vertex_old,
                        mer.moved_vertex_new,
                        mer.v_rim,
                        tolerance,
                    )?;
                }
            }
        }
    }

    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(KernelStage::Validate, "sphere_validate", format!(
            "move_faces: moved solid failed validation: {issues:?}"
        )));
    }
    if let (Ok(before), Ok(after)) = (solid_signed_volume(solid), solid_signed_volume(&result)) {
        if before * after <= 0.0 {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, SPHERE_SOLID_INVERSION,
                "move_faces: the push inverts the solid (signed volume changed sign) — refusing"));
        }
    }
    Ok(result)
}

/// Slide a sphere seam-meridian pcurve's RIM endpoint to the new latitude,
/// keeping its constant u (so the periodic u=0 / u=2π seam pairing survives) and
/// its fixed (pole / other-rim) endpoint. The moved endpoint is identified by
/// which pcurve end maps (through the surface) to the moved vertex's OLD
/// position — not by pole detection — so it makes no assumption about the cap's
/// topology. Endpoint order (domain start→end) is preserved.
fn patch_seam_meridian_pcurve(
    pcurve: &NurbsCurve,
    surface: &NurbsSurface,
    moved_vertex_old: Vec3,
    moved_vertex_new: Vec3,
    v_rim: f64,
    tolerance: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let a = pcurve.evaluate(q0).or_refuse(KernelStage::Sew, "evaluate")?;
    let b = pcurve.evaluate(q1).or_refuse(KernelStage::Sew, "evaluate")?;
    let a3 = surface.evaluate(a.x, a.y).or_refuse(KernelStage::Sew, "evaluate")?;
    let b3 = surface.evaluate(b.x, b.y).or_refuse(KernelStage::Sew, "evaluate")?;
    let da = a3.sub(moved_vertex_old).length();
    let db = b3.sub(moved_vertex_old).length();
    // Guard: one endpoint must genuinely be the moved vertex, and the surface
    // point at the patched (u, v_rim) must land on the relocated vertex.
    let moved_is_a = da <= db;
    let (moved_uv, fixed_uv) = if moved_is_a { (a, b) } else { (b, a) };
    let patched = Vec3::new(moved_uv.x, v_rim, 0.0);
    if surface.evaluate(patched.x, patched.y).or_refuse(KernelStage::Sew, "evaluate")?.sub(moved_vertex_new).length() > 10.0 * tolerance {
        return Err(KernelRefusal::internal(KernelStage::Sew, "seam_patch",
            "move_faces: patched seam-meridian pcurve endpoint does not reach the new rim \
             vertex — refusing"));
    }
    let fixed = Vec3::new(fixed_uv.x, fixed_uv.y, 0.0);
    if moved_is_a {
        make_line(patched, fixed)
    } else {
        make_line(fixed, patched)
    }
    .or_refuse(KernelStage::Sew, "make_line")
}

