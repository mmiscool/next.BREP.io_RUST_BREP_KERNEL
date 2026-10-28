//! The TAPERED CUSP RUNOUT: a constant-radius stripe whose mate narrows to a
//! zero-width CUSP before the edge ends finishes with its radius tapering
//! smoothly to zero at the cusp, instead of on a web and a cap.
//!
//! # The class
//!
//! A selected convex edge E ends at a vertex V where one of its two mates, A,
//! has a CUSP: A's other boundary edge B at V leaves V along E's own
//! direction (unit tangents within [`CUSP_TANGENT`]), so A's width under E
//! goes to zero at V, and a radius-R ball stops fitting before V. Report
//! 6df39293 (F8): the r 0.25 round on the start face's sliver outside F2's
//! r 5 round; A is the sliver (y = 10), B the r 5 arc, V is (5, 10, 20).
//!
//! # The law (the integrator's ruling, 2026-10-19: C1, keep the hairline)
//!
//! In the plane normal to E at each station, the WIDTH w is the radius of the
//! largest ball tangent to the other mate and to A with its contact on A
//! still on A's face — the ball whose A contact sits on B ([`ball_at`] at the
//! point where B crosses that plane). On F8, w(x) = 5 − √(25 − (x − 5)²), the
//! sliver's height under the edge. The radius is a function of the width
//! alone ([`radius_for_width`]):
//!
//! ```text
//! r(w) = R                  for w ≥ 2R
//! r(w) = w − w² / (4R)      for w ≤ 2R
//! ```
//!
//! It is C1 where the pieces meet (r = R, r' = 0 at w = 2R), so the round has
//! no crease across any section — the section arc's normal depends on r' —
//! and r < w for every w > 0: the ball's contact on A stays inside A, leaving
//! a HAIRLINE of A between the round's rail and B, w²/(4R) wide. The
//! quadratic is the lowest-order piece that meets R with zero slope and
//! returns to zero with the width; it starts the taper at w = 2R (F8:
//! x = 5 − √(25 − 4.5²) = 2.8206), before the constant ball stops fitting at
//! w = R (x = 3.43875).
//!
//! Where the hairline narrows below the model tolerance the face would be
//! thinner than the kernel can tell from an edge, so A ENDS there: past that
//! station (H) the round's rail is identified with B, the round borders B's
//! other face along B's piece from H to V, and A keeps B only from H on.
//!
//! The section at each station is the ball's arc between its two contacts;
//! both lie on their mates and the arc's tangent there is in the mate's
//! tangent plane, so the round is G1 to both mates along both rails by
//! construction, whatever r'.
//!
//! # The end at V
//!
//! At V, r = 0: the last row collapses to the vertex and the face closes on
//! a DEGENERATE edge there, the way `plan_pole_end` closes a stripe whose
//! mates run tangent into a pole. The surface is regular on every row before
//! V (r > 0 and the two contacts distinct) and singular only at V itself;
//! no cap is needed, so none is built.
//!
//! # Topology
//!
//! The stripe stops at the taper's first section (w = 2R) exactly as a
//! capped end does (`CapEnd`): its rails end at the section's rims, and a leg
//! on each mate runs from the rim towards V. The leg on the other mate is the
//! tapered round's rail, rim to V. The leg on A is the round's rail on A from
//! the rim to H; from H the round shares B's V-side piece with B's other
//! face, B being split at H. The bulkhead a cap would sew is the round.
//!
//! # Concave edges and both ends (2026-10-21, report b46db89a)
//!
//! The class holds for a CONCAVE edge too ([`ball_at`] reads the depth on
//! each mate's ball side, so the sign of ρ carries it), and at both ends of
//! one stripe, each end against its own cusp line. Report b46db89a's sliver
//! is a square corner minus a quarter disk under an r 1 concave arc: the
//! width `5/cos θ − 5` reaches 2R only within 44.4°–45.6° of the arc, so the
//! two tapers start 0.1 apart about its midpoint and the constant stripe
//! between them is that short. The plane at the midpoint passes through the
//! square's corner, which is each cusp line's far END, so [`width_at`] admits
//! a crossing at B's end within a bar. With the local radius relief on, the
//! network asks this runout BEFORE the free end at a cusped vertex (a free end
//! found a crossing there and shipped a wall overhanging the cusp), and refuses
//! a cusped end whose taper cannot be planned; `fillet/analyze.rs`'s support
//! check admits the overrun of a cusped support as the taper's.
//!
//! `BREP_BLEND_TAPER_RUNOUT=0`/`off` is the escape hatch: the network never
//! asks and the end goes to the free end and the cutter as before. On by
//! default since its census (promoted 2026-10-19).

use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{project_point_to_curve, project_point_to_surface, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4};

use super::edge::{boundary_edge_at_vertex, fit_rows_breaking, interpolate_piece, trim_edge_at, trim_parameter_at_point, CapEnd};
use super::network::tangent_away_from;
use super::stations::{raw_normal, BlendMate, FittedRows, Station};

/// Unit tangents of the selected edge and A's other boundary edge leaving V
/// agree to this (1 − cos): A's interior angle at V is zero — a cusp.
const CUSP_TANGENT: f64 = 1e-6;

/// Stations of the tapered stretch, taper start to cusp inclusive (the
/// hairline's end H is inserted besides).
const TAPER_STATIONS: usize = 97;

/// Samples used to bracket a root along an edge, and bisections after.
const BRACKET_SAMPLES: usize = 64;
const BISECTIONS: usize = 80;

/// Newton steps of the in-plane ball solve.
const SECTION_NEWTON: usize = 60;

/// Samples of B's piece H -> V read onto the round for its pcurve there.
const PIECE_SAMPLES: usize = 17;

/// Promoted 2026-10-19 (lane R7).
const TAPER_RUNOUT_DEFAULT: bool = true;

thread_local! {
    static TAPER_RUNOUT_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

/// `BREP_BLEND_TAPER_RUNOUT=1`/`on` (the default) builds the tapered cusp
/// runout; `=0`/`off` leaves the end to the network's free end and the cutter.
pub(crate) fn taper_runout_on() -> bool {
    if let Some(forced) = TAPER_RUNOUT_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_TAPER_RUNOUT").as_deref() {
        Ok("0") | Ok("off") => false,
        Ok("1") | Ok("on") => true,
        _ => TAPER_RUNOUT_DEFAULT,
    }
}


fn debug_on() -> bool {
    std::env::var("BREP_DEBUG_NETWORK").is_ok()
}

/// The law (module doc): the radius at a section whose width is `width`.
pub(crate) fn radius_for_width(width: f64, radius: f64) -> f64 {
    if width >= 2.0 * radius {
        radius
    } else {
        width - width * width / (4.0 * radius)
    }
}

/// One section's ball: tangent to A at `on_a` and to the other mate at
/// `on_other`.
#[derive(Clone, Copy, Debug)]
pub(in crate::blend) struct TaperBall {
    pub(in crate::blend) radius: f64,
    pub(in crate::blend) center: Vec3,
    pub(in crate::blend) on_a: Vec3,
    pub(in crate::blend) uv_a: [f64; 2],
    pub(in crate::blend) on_other: Vec3,
    pub(in crate::blend) uv_other: [f64; 2],
}

/// The inward unit normal of a mate at `uv`: the side its ball centre sits
/// on (`center = p + ρ·n_raw`, so the sign of ρ).
fn inward_normal(mate: &BlendMate, uv: [f64; 2]) -> Result<Vec3, KernelRefusal> {
    let normal = raw_normal(&mate.face.surface, uv[0], uv[1])?
        .normalized()
        .or_refuse(KernelStage::Refine, "normalized")?;
    Ok(if mate.rho < 0.0 { normal.scale(-1.0) } else { normal })
}

/// The point's depth on a mate's ball side, its foot and the foot's uv.
fn depth(mate: &BlendMate, point: Vec3) -> Result<(f64, Vec3, [f64; 2], Vec3), KernelRefusal> {
    let foot = project_point_to_surface(&mate.face.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let uv = [foot.u, foot.v];
    let inward = inward_normal(mate, uv)?;
    Ok((point.sub(foot.point).dot(inward), foot.point, uv, inward))
}

/// The largest ball tangent to `a` at `point` (on A's carrier) and to
/// `other`: r solves `depth_other(point + r·n_A) = r` by bisection on
/// `[0, ceiling]`. `None` when no root lies in the bracket.
pub(in crate::blend) fn ball_at(
    a: &BlendMate,
    other: &BlendMate,
    point: Vec3,
    ceiling: f64,
) -> Result<Option<TaperBall>, KernelRefusal> {
    let foot_a = project_point_to_surface(&a.face.surface, point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
    let uv_a = [foot_a.u, foot_a.v];
    let inward_a = inward_normal(a, uv_a)?;
    let gap = |radius: f64| -> Result<(f64, Vec3, [f64; 2], Vec3), KernelRefusal> {
        let center = point.add(inward_a.scale(radius));
        let (depth, foot, uv, _) = depth(other, center)?;
        Ok((depth - radius, center, uv, foot))
    };
    let (mut low, mut high) = (0.0, ceiling);
    if gap(low)?.0 < 0.0 || gap(high)?.0 > 0.0 {
        return Ok(None);
    }
    for _ in 0..BISECTIONS {
        let middle = 0.5 * (low + high);
        if gap(middle)?.0 > 0.0 {
            low = middle;
        } else {
            high = middle;
        }
    }
    let radius = 0.5 * (low + high);
    let (_, center, uv_other, on_other) = gap(radius)?;
    Ok(Some(TaperBall { radius, center, on_a: point, uv_a, on_other, uv_other }))
}

/// The ball of radius `radius` tangent to both carriers with its centre in
/// the plane through `origin` normal to `tangent`: Newton on the centre,
/// whose Jacobian rows are the two inward normals (the gradients of the
/// depths) and the plane's normal.
fn section_ball(
    a: &BlendMate,
    other: &BlendMate,
    origin: Vec3,
    tangent: Vec3,
    radius: f64,
    guess: Vec3,
    scale: f64,
) -> Result<Option<TaperBall>, KernelRefusal> {
    let mut center = guess;
    for _ in 0..SECTION_NEWTON {
        let (depth_a, _, _, n_a) = depth(a, center)?;
        let (depth_o, _, _, n_o) = depth(other, center)?;
        let residual = [depth_a - radius, depth_o - radius, center.sub(origin).dot(tangent)];
        if residual.iter().all(|value| value.abs() <= 1e-14 * (1.0 + scale)) {
            break;
        }
        // Solve [n_a; n_o; tangent] · step = residual by Cramer's rule.
        let det = n_a.dot(n_o.cross(tangent));
        if det.abs() < 1e-12 {
            return Ok(None);
        }
        let step = n_o
            .cross(tangent)
            .scale(residual[0])
            .add(tangent.cross(n_a).scale(residual[1]))
            .add(n_a.cross(n_o).scale(residual[2]))
            .scale(1.0 / det);
        center = center.sub(step);
    }
    let (depth_a, on_a, uv_a, _) = depth(a, center)?;
    let (depth_o, on_other, uv_other, _) = depth(other, center)?;
    let bar = 1e-11 * (1.0 + scale);
    if (depth_a - radius).abs() > bar || (depth_o - radius).abs() > bar || center.sub(origin).dot(tangent).abs() > bar {
        return Ok(None);
    }
    Ok(Some(TaperBall { radius, center, on_a, uv_a, on_other, uv_other }))
}

/// Bisect `[near, far]` for the parameter where `rising` crosses `target`
/// (`rising(near) < target <= rising(far)`).
fn bisect(
    mut near: f64,
    mut far: f64,
    target: f64,
    rising: &mut dyn FnMut(f64) -> Result<Option<f64>, KernelRefusal>,
) -> Result<Option<f64>, KernelRefusal> {
    for _ in 0..BISECTIONS {
        let middle = 0.5 * (near + far);
        match rising(middle)? {
            Some(value) if value < target => near = middle,
            Some(_) => far = middle,
            None => return Ok(None),
        }
    }
    Ok(Some(0.5 * (near + far)))
}

/// The recognised class at one stripe end, with its stations' positions.
pub(in crate::blend) struct TaperEnd {
    /// Whether A (the cusped mate) is the stripe's SECOND mate.
    pub(in crate::blend) a_is_second: bool,
    /// A's other boundary edge at V and its parameter at V.
    pub(in crate::blend) boundary: u64,
    pub(in crate::blend) b_vertex: f64,
    /// The selected edge's parameters at V and where the taper starts
    /// (width 2R).
    pub(in crate::blend) e_vertex: f64,
    pub(in crate::blend) e_start: f64,
}

/// A's boundary from V onward, away from the selected edge: B from V to its
/// far end, then each next edge of A's loop from the vertex it shares with
/// the last, up to the selected edge again (degenerate edges skipped), as
/// (edge, parameter at the near end, parameter at the far end).
///
/// The width plane crosses B only while B lasts. Where A's boundary turns
/// (report 80c5d9d1, 2026-10-24: F10's r 1 on the start face's straight
/// edge after F9's r 0.3 tapered round, whose taper rail is B and spans only
/// the 0.27-wide end of the sliver), the width reaches 2R on a LATER edge of
/// A's boundary, F9's constant rail; read on B alone it ran out, the end was
/// not recognised, and the free end and cutter closed the round on a planar
/// web in y = 10 under F2's round: an undercut.
pub(in crate::blend) fn boundary_chain<'a>(
    solid: &'a BrepSolid,
    face: &FaceRecord,
    edge_id: u64,
    boundary: &'a EdgeRecord,
    b_vertex: f64,
) -> Vec<(&'a EdgeRecord, f64, f64)> {
    let b_far = if (boundary.t0 - b_vertex).abs() < (boundary.t1 - b_vertex).abs() { boundary.t1 } else { boundary.t0 };
    let mut chain = vec![(boundary, b_vertex, b_far)];
    let Some(coedges) = face
        .loops
        .iter()
        .map(|ring| &ring.coedges)
        .find(|coedges| coedges.iter().any(|c| c.edge_id == boundary.id) && coedges.iter().any(|c| c.edge_id == edge_id))
    else {
        return chain;
    };
    let count = coedges.len();
    let Some(at_b) = coedges.iter().position(|c| c.edge_id == boundary.id) else {
        return chain;
    };
    // Away from the selected edge: the loop steps from B to E one way.
    let step = if coedges[(at_b + 1) % count].edge_id == edge_id { count - 1 } else { 1 };
    let mut far_vertex = if b_far == boundary.t1 { boundary.end_vertex_id } else { boundary.start_vertex_id };
    let mut index = at_b;
    for _ in 1..count {
        index = (index + step) % count;
        if coedges[index].edge_id == edge_id {
            break;
        }
        let Some(next) = solid.edges.iter().find(|candidate| candidate.id == coedges[index].edge_id) else {
            break;
        };
        if next.degenerate {
            continue;
        }
        let (near, far, vertex) = if next.start_vertex_id == far_vertex {
            (next.t0, next.t1, next.end_vertex_id)
        } else if next.end_vertex_id == far_vertex {
            (next.t1, next.t0, next.start_vertex_id)
        } else {
            break;
        };
        chain.push((next, near, far));
        far_vertex = vertex;
    }
    chain
}

/// Where A's boundary from V ([`boundary_chain`]) first crosses the plane
/// normal to E at `t`, and the width there.
fn width_at(
    edge: &EdgeRecord,
    chain: &[(&EdgeRecord, f64, f64)],
    a: &BlendMate,
    other: &BlendMate,
    t: f64,
    ceiling: f64,
) -> Result<Option<(f64, TaperBall)>, KernelRefusal> {
    let derivatives = edge.curve.derivatives(t, 1).or_refuse(KernelStage::Refine, "derivatives")?;
    let (origin, tangent) = (derivatives[0], derivatives[1].normalized().or_refuse(KernelStage::Refine, "normalized")?);
    let Some(&(first, first_near, _)) = chain.first() else {
        return Ok(None);
    };
    let side = |curve: &EdgeRecord, s: f64| -> Result<f64, KernelRefusal> {
        Ok(curve.curve.evaluate(s).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin).dot(tangent))
    };
    let start = side(first, first_near)?;
    // A plane through B's far END crosses it there, whatever the last bit of
    // the sign reads (report b46db89a: the sliver's two cusp lines meet at the
    // square's corner, which the plane at the arc's midpoint passes through).
    let end_bar = 1e-9 * (1.0 + start.abs());
    for &(boundary, b_near, b_far) in chain {
        let mut previous = b_near;
        let mut bracket = None;
        for index in 1..=BRACKET_SAMPLES {
            let s = b_near + (b_far - b_near) * index as f64 / BRACKET_SAMPLES as f64;
            if side(boundary, s)?.signum() != start.signum() {
                bracket = Some((previous, s));
                break;
            }
            previous = s;
        }
        let bracket = bracket.or_else(|| (side(boundary, b_far).ok()?.abs() <= end_bar).then_some((b_far, b_far)));
        let Some((mut near, mut far)) = bracket else {
            continue;
        };
        for _ in 0..BISECTIONS {
            if near == far {
                break;
            }
            let middle = 0.5 * (near + far);
            if side(boundary, middle)?.signum() == start.signum() {
                near = middle;
            } else {
                far = middle;
            }
        }
        let s = 0.5 * (near + far);
        let point = boundary.curve.evaluate(s).or_refuse(KernelStage::Refine, "evaluate")?;
        return Ok(ball_at(a, other, point, ceiling)?.map(|ball| (s, ball)));
    }
    Ok(None)
}

/// Is `face` CUSPED at `vertex` under the selected `edge` — does its other
/// boundary edge there leave `vertex` along the edge's own direction (the
/// test [`recognise`] admits a mate by)?
pub(crate) fn cusped_at(solid: &BrepSolid, edge: &EdgeRecord, face: &FaceRecord, vertex: u64) -> bool {
    let Ok(own) = tangent_away_from(edge, vertex) else {
        return false;
    };
    let Ok(boundary_id) = boundary_edge_at_vertex(solid, face, vertex, edge.id) else {
        return false;
    };
    let Some(boundary) = solid.edges.iter().find(|candidate| candidate.id == boundary_id) else {
        return false;
    };
    !boundary.degenerate && tangent_away_from(boundary, vertex).is_ok_and(|other| own.dot(other) >= 1.0 - CUSP_TANGENT)
}

/// Recognise the class at `vertex` for a constant-radius FILLET stripe:
/// exactly one mate cusped there, and the width rising from zero at V
/// through 2R along the edge. `None` for every other end.
pub(in crate::blend) fn recognise(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius: f64,
    vertex: u64,
) -> Result<Option<TaperEnd>, KernelRefusal> {
    let own = tangent_away_from(edge, vertex)?;
    let mut cusps = Vec::new();
    for (index, mate) in [first, second].into_iter().enumerate() {
        let Ok(boundary_id) = boundary_edge_at_vertex(solid, mate.face, vertex, edge.id) else {
            continue;
        };
        let Some(boundary) = solid.edges.iter().find(|candidate| candidate.id == boundary_id) else {
            continue;
        };
        if boundary.degenerate {
            continue;
        }
        let Ok(other) = tangent_away_from(boundary, vertex) else {
            continue;
        };
        if own.dot(other) >= 1.0 - CUSP_TANGENT {
            cusps.push((index == 1, boundary));
        }
    }
    let [(a_is_second, boundary)] = cusps[..] else {
        if debug_on() {
            eprintln!("taper runout: vertex {vertex} has {} cusped mates, not one", cusps.len());
        }
        return Ok(None);
    };
    let (a, other) = if a_is_second { (second, first) } else { (first, second) };
    let b_vertex = if boundary.start_vertex_id == vertex { boundary.t0 } else { boundary.t1 };
    let e_vertex = if edge.start_vertex_id == vertex { edge.t0 } else { edge.t1 };
    let e_far = if edge.start_vertex_id == vertex { edge.t1 } else { edge.t0 };
    let ceiling = 8.0 * radius;
    let chain = boundary_chain(solid, a.face, edge.id, boundary, b_vertex);
    let mut width = |t: f64| -> Result<Option<f64>, KernelRefusal> {
        Ok(width_at(edge, &chain, a, other, t, ceiling)?.map(|(_, ball)| ball.radius))
    };
    // Walk away from V until the width reaches 2R; it must rise all the way.
    let mut previous = (e_vertex, 0.0);
    let mut bracket = None;
    for index in 1..=BRACKET_SAMPLES {
        let t = e_vertex + (e_far - e_vertex) * index as f64 / BRACKET_SAMPLES as f64;
        let Some(w) = width(t)? else {
            if debug_on() {
                eprintln!("taper runout: vertex {vertex}: no width at edge t {t}");
            }
            return Ok(None);
        };
        if w <= previous.1 {
            if debug_on() {
                eprintln!("taper runout: vertex {vertex}: width {w} does not rise at edge t {t} (previous {:?})", previous);
            }
            return Ok(None);
        }
        if w >= 2.0 * radius {
            bracket = Some((previous.0, t));
            break;
        }
        previous = (t, w);
    }
    let Some((near, far)) = bracket else {
        return Ok(None);
    };
    let Some(e_start) = bisect(near, far, 2.0 * radius, &mut width)? else {
        return Ok(None);
    };
    Ok(Some(TaperEnd { a_is_second, boundary: boundary.id, b_vertex, e_vertex, e_start }))
}

/// The tapered stretch's stations, taper start (radius R) to V (radius 0,
/// collapsed onto V), uniform in the edge's parameter, with the hairline's
/// end H inserted: (stations, parameters in [0, 1], balls, index of H, B's
/// parameter at H, the hairline's width at every station).
#[allow(clippy::type_complexity)]
pub(in crate::blend) fn taper_stations(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    end: &TaperEnd,
    radius: f64,
    vertex_point: Vec3,
) -> Result<Option<(Vec<Station>, Vec<f64>, Vec<TaperBall>, usize, f64, Vec<f64>)>, KernelRefusal> {
    let boundary = solid
        .edges
        .iter()
        .find(|candidate| candidate.id == end.boundary)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "taper_boundary", "taper runout: boundary edge missing"))?;
    let (a, other) = if end.a_is_second { (second, first) } else { (first, second) };
    let scale = crate::solid_scale(solid).max(1.0);
    let tolerance = crate::KernelTolerances::for_solid(solid, 1e-7).model;
    let ceiling = 8.0 * radius;
    let chain = boundary_chain(solid, a.face, edge.id, boundary, end.b_vertex);
    // The section ball at edge parameter t and its hairline (the distance on
    // A from its contact to B).
    let mut guess = None::<Vec3>;
    let mut ball_at_t = |t: f64, guess: &mut Option<Vec3>| -> Result<Option<(TaperBall, f64)>, KernelRefusal> {
        let Some((_, widest)) = width_at(edge, &chain, a, other, t, ceiling)? else {
            return Ok(None);
        };
        let r = radius_for_width(widest.radius, radius);
        let derivatives = edge.curve.derivatives(t, 1).or_refuse(KernelStage::Refine, "derivatives")?;
        let tangent = derivatives[1].normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let seed = guess.unwrap_or(widest.center);
        let Some(ball) = section_ball(a, other, derivatives[0], tangent, r, seed, scale)? else {
            return Ok(None);
        };
        *guess = Some(ball.center);
        let hairline = project_point_to_curve(&boundary.curve, ball.on_a).or_refuse(KernelStage::Refine, "project_point_to_curve")?.distance;
        Ok(Some((ball, hairline)))
    };
    // H: where the hairline falls to the model tolerance.
    let mut hairline_at = |t: f64| -> Result<Option<f64>, KernelRefusal> {
        let mut seed = None;
        Ok(ball_at_t(t, &mut seed)?.map(|(_, hairline)| -hairline))
    };
    let Some(e_hair) = bisect(end.e_start, end.e_vertex, -tolerance, &mut hairline_at)? else {
        return Ok(None);
    };
    let mut ts: Vec<f64> =
        (0..TAPER_STATIONS).map(|index| end.e_start + (end.e_vertex - end.e_start) * index as f64 / (TAPER_STATIONS - 1) as f64).collect();
    let span = end.e_vertex - end.e_start;
    let h_fraction = (e_hair - end.e_start) / span;
    // Insert H, displacing a station closer than a quarter spacing.
    let spacing = 1.0 / (TAPER_STATIONS - 1) as f64;
    ts.retain(|t| {
        let fraction = (t - end.e_start) / span;
        fraction == 0.0 || fraction == 1.0 || (fraction - h_fraction).abs() > 0.25 * spacing
    });
    ts.push(e_hair);
    ts.sort_by(|x, y| ((x - end.e_start) / span).total_cmp(&((y - end.e_start) / span)));
    let h_index = ts.iter().position(|t| *t == e_hair).ok_or(KernelRefusal::internal(KernelStage::Refine, "taper_h", "taper runout: H lost"))?;

    let station = |ball: &TaperBall, normals: Option<(Vec3, Vec3)>| -> Result<Station, KernelRefusal> {
        let (p1, p2, uv1, uv2) = if end.a_is_second {
            (ball.on_other, ball.on_a, ball.uv_other, ball.uv_a)
        } else {
            (ball.on_a, ball.on_other, ball.uv_a, ball.uv_other)
        };
        // The section's sweep is the angle between the contacts' outward
        // normals; at the collapsed row it is read off the mates' normals.
        let (d1, d2) = match normals {
            Some(pair) => pair,
            None => (
                p1.sub(ball.center).normalized().or_refuse(KernelStage::Refine, "normalized")?,
                p2.sub(ball.center).normalized().or_refuse(KernelStage::Refine, "normalized")?,
            ),
        };
        let sweep = d1.dot(d2).clamp(-1.0, 1.0).acos();
        let weight = (0.5 * sweep).cos();
        let bisector = d1.add(d2).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        Ok(Station { uv1, uv2, p1, p2, center: ball.center, weight, apex: ball.center.add(bisector.scale(ball.radius / weight)) })
    };
    let mut stations = Vec::with_capacity(ts.len());
    let mut parameters = Vec::with_capacity(ts.len());
    let mut balls = Vec::with_capacity(ts.len());
    let mut hairlines = Vec::with_capacity(ts.len());
    for (index, t) in ts.iter().enumerate() {
        parameters.push(if index + 1 == ts.len() { 1.0 } else { (t - end.e_start) / span });
        if index + 1 == ts.len() {
            let foot_a = project_point_to_surface(&a.face.surface, vertex_point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            let foot_o = project_point_to_surface(&other.face.surface, vertex_point).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            let ball = TaperBall {
                radius: 0.0,
                center: vertex_point,
                on_a: vertex_point,
                uv_a: [foot_a.u, foot_a.v],
                on_other: vertex_point,
                uv_other: [foot_o.u, foot_o.v],
            };
            let out_a = inward_normal(a, ball.uv_a)?.scale(-1.0);
            let out_o = inward_normal(other, ball.uv_other)?.scale(-1.0);
            let normals = if end.a_is_second { (out_o, out_a) } else { (out_a, out_o) };
            stations.push(station(&ball, Some(normals))?);
            balls.push(ball);
            hairlines.push(0.0);
            continue;
        }
        let Some((mut ball, hairline)) = ball_at_t(*t, &mut guess)? else {
            return Ok(None);
        };
        if index == 0 {
            ball.radius = radius;
        }
        stations.push(station(&ball, None)?);
        balls.push(ball);
        hairlines.push(hairline);
    }
    // H's parameter on B: where the rail meets it (within the tolerance).
    let b_hair = project_point_to_curve(&boundary.curve, balls[h_index].on_a).or_refuse(KernelStage::Refine, "project_point_to_curve")?.u;
    Ok(Some((stations, parameters, balls, h_index, b_hair, hairlines)))
}

/// What the sew needs of one planned runout once the stripes are in.
pub(in crate::blend) struct PendingTaper {
    surface: NurbsSurface,
    center: NurbsCurve,
    /// The taper's first section: (edge id, stored start vertex, stored end
    /// vertex), stored in the stripe's walk direction.
    arc: (u64, u64, u64),
    /// The rims, first then second mate's.
    rims: [u64; 2],
    /// The other mate's leg (rim -> V), the round's rail on A (rim -> H) and
    /// B's piece H -> V.
    other_leg: u64,
    a_rail: u64,
    b_piece: u64,
    /// The round's u at H, and the piece's pcurve on the round (H -> V).
    u_hair: f64,
    piece_on_round: NurbsCurve,
    a_is_second: bool,
    boundary: u64,
    b_hair: f64,
    hair_vertex: u64,
    vertex: u64,
    name: Option<String>,
    /// The ball rolls OUTSIDE the material (a concave edge): the round's
    /// outward normal faces its ball centre rather than away from it.
    ball_outside: bool,
}

/// Plan the tapered runout at a stripe end, or `None` (switch off, a
/// chamfer, a tapered law, or not this class): the stripe stops at the
/// taper's first section and the runout is sewn by [`sew_taper_ends`].
#[allow(clippy::too_many_arguments)]
pub(in crate::blend) fn plan_taper_end(
    solid: &BrepSolid,
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    rows: &FittedRows,
    constant_radius: Option<f64>,
    chamfer: bool,
    walk_first_to_second: bool,
    name: Option<&str>,
    vertex: u64,
    pending: &mut Vec<PendingTaper>,
) -> Result<Option<CapEnd>, KernelRefusal> {
    if !taper_runout_on() || chamfer {
        return Ok(None);
    }
    let Some(radius) = constant_radius.filter(|radius| radius.is_finite() && *radius > 0.0) else {
        return Ok(None);
    };
    let Some(end) = recognise(solid, edge, first, second, radius, vertex)? else {
        return Ok(None);
    };
    let vertex_point = solid
        .vertices
        .iter()
        .find(|candidate| candidate.id == vertex)
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "taper_vertex", "taper runout: vertex missing"))?
        .point;
    let Some((stations, parameters, balls, h_index, b_hair, hairlines)) =
        taper_stations(solid, edge, first, second, &end, radius, vertex_point)?
    else {
        if debug_on() {
            eprintln!("taper runout: edge {} vertex {vertex}: no section ball along the taper", edge.id);
        }
        return Ok(None);
    };
    let start = balls[0];
    // The stripe's own section through the taper's first ball: its rails
    // must stand on that ball's contacts, between its vertex stations.
    let center_path = rows
        .center
        .as_ref()
        .ok_or(KernelRefusal::internal(KernelStage::Refine, "centre_path", "taper runout: stripe rows carry no centre path"))?;
    let station = project_point_to_curve(center_path, start.center).or_refuse(KernelStage::Refine, "project_point_to_curve")?.u;
    let scale = crate::solid_scale(solid).max(1.0);
    let bar = 1e-6 * (1.0 + scale);
    let (first_point, second_point) = (
        rows.cr.evaluate(station).or_refuse(KernelStage::Refine, "evaluate")?,
        rows.cs.evaluate(station).or_refuse(KernelStage::Refine, "evaluate")?,
    );
    let misses = [
        first_point.sub(stations[0].p1).length(),
        second_point.sub(stations[0].p2).length(),
        center_path.evaluate(station).or_refuse(KernelStage::Refine, "evaluate")?.sub(start.center).length(),
    ];
    let inside = rows.vertex_stations.is_some_and(|[low, high]| station > low.min(high) && station < low.max(high));
    let boundary = solid.edges.iter().find(|candidate| candidate.id == end.boundary).ok_or(KernelRefusal::internal(
        KernelStage::Refine,
        "taper_boundary",
        "taper runout: boundary edge missing",
    ))?;
    if debug_on() {
        eprintln!(
            "taper runout: edge {} vertex {vertex} A={} boundary {} taper from t {:.12} station {station:.9}, H at {:.9} of the runout (B t {b_hair:.12}), misses {:.3e} {:.3e} {:.3e} inside {inside}",
            edge.id,
            if end.a_is_second { "second" } else { "first" },
            end.boundary,
            end.e_start,
            parameters[h_index],
            misses[0],
            misses[1],
            misses[2],
        );
        let widths: Vec<String> = hairlines.iter().zip(&parameters).map(|(w, u)| format!("{u:.4}:{w:.3e}")).collect();
        eprintln!("taper runout: hairline width by runout fraction {}", widths.join(" "));
    }
    if misses.iter().any(|miss| *miss > bar) || !inside {
        return Ok(None);
    }
    let taper = fit_rows_breaking(&stations, &parameters, &[])?;
    let (rail_a, rail_other, a_face, other_face) = if end.a_is_second {
        (&taper.cs, &taper.cr, second.face, first.face)
    } else {
        (&taper.cr, &taper.cs, first.face, second.face)
    };
    // The fitted rails between stations: each on its carrier; A's past H on B.
    let u_hair = parameters[h_index];
    let mut worst = [0.0f64; 3];
    for index in 0..4 * (TAPER_STATIONS - 1) {
        let u = (index as f64 + 0.5) / (4 * (TAPER_STATIONS - 1)) as f64;
        let on_a = rail_a.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
        worst[0] = worst[0].max(project_point_to_surface(&a_face.surface, on_a).or_refuse(KernelStage::Refine, "project_point_to_surface")?.distance);
        if u > u_hair {
            worst[2] = worst[2].max(project_point_to_curve(&boundary.curve, on_a).or_refuse(KernelStage::Refine, "project_point_to_curve")?.distance);
        }
        let on_other = rail_other.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
        worst[1] = worst[1].max(
            project_point_to_surface(&other_face.surface, on_other).or_refuse(KernelStage::Refine, "project_point_to_surface")?.distance,
        );
    }
    let tolerance = crate::KernelTolerances::for_solid(solid, 1e-7).model;
    if debug_on() {
        eprintln!(
            "taper runout: fitted rails off A {:.3e}, off the other mate {:.3e}, A's past H off B {:.3e} (bars {bar:.3e}, B {:.3e})",
            worst[0],
            worst[1],
            worst[2],
            2.0 * tolerance
        );
    }
    if worst[0] > bar || worst[1] > bar || worst[2] > 2.0 * tolerance {
        return Ok(None);
    }

    // Commit: the rims, the first section, the other mate's leg, the round's
    // rail on A to H, H on B and B's piece H -> V.
    let first_vertex = take_id();
    result.vertices.push(VertexRecord { id: first_vertex, point: first_point });
    let second_vertex = take_id();
    result.vertices.push(VertexRecord { id: second_vertex, point: second_point });
    let hair_point = boundary.curve.evaluate(b_hair).or_refuse(KernelStage::Refine, "evaluate")?;
    let hair_vertex = take_id();
    result.vertices.push(VertexRecord { id: hair_vertex, point: hair_point });
    let (from, to, from_vertex, to_vertex) = if walk_first_to_second {
        (first_point, second_point, first_vertex, second_vertex)
    } else {
        (second_point, first_point, second_vertex, first_vertex)
    };
    let x_axis = from.sub(start.center).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let toward = to.sub(start.center).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let y_axis = toward.sub(x_axis.scale(toward.dot(x_axis))).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let sweep = toward.dot(x_axis).clamp(-1.0, 1.0).acos();
    let arc = crate::make_arc(start.center, x_axis, y_axis, radius, 0.0, sweep).or_refuse(KernelStage::Refine, "make_arc")?;
    let arc_domain = arc.domain().or_refuse(KernelStage::Refine, "domain")?;
    let arc_edge_id = take_id();
    result.edges.push(EdgeRecord {
        id: arc_edge_id,
        curve: arc,
        t0: arc_domain[0],
        t1: arc_domain[1],
        start_vertex_id: from_vertex,
        end_vertex_id: to_vertex,
        degenerate: false,
        name: None,
    });
    let arc_blend_pcurve = if walk_first_to_second {
        crate::sweep_topology::parameter_line(station, 0.0, station, 1.0)
    } else {
        crate::sweep_topology::parameter_line(station, 1.0, station, 0.0)
    }
    .or_refuse(KernelStage::Refine, "parameter_line")?;

    let uv_samples = |uvs: Vec<[f64; 2]>, at: &[f64]| -> Result<NurbsCurve, KernelRefusal> {
        let samples: Vec<Vec4> = uvs.iter().map(|uv| Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0)).collect();
        interpolate_piece(&samples, at, &[])
    };
    let (other_rim, a_rim) = if end.a_is_second { (first_vertex, second_vertex) } else { (second_vertex, first_vertex) };
    // The other mate's leg: the round's rail on it, rim -> V.
    let other_leg_id = take_id();
    result.edges.push(EdgeRecord {
        id: other_leg_id,
        curve: rail_other.clone(),
        t0: 0.0,
        t1: 1.0,
        start_vertex_id: other_rim,
        end_vertex_id: vertex,
        degenerate: false,
        name: None,
    });
    let other_leg_pcurve = uv_samples(balls.iter().map(|ball| ball.uv_other).collect(), &parameters)?;
    // A's leg: the round's rail on A, rim -> H.
    let a_rail_id = take_id();
    result.edges.push(EdgeRecord {
        id: a_rail_id,
        curve: rail_a.clone(),
        t0: 0.0,
        t1: u_hair,
        start_vertex_id: a_rim,
        end_vertex_id: hair_vertex,
        degenerate: false,
        name: None,
    });
    let a_rail_pcurve = uv_samples(balls[..=h_index].iter().map(|ball| ball.uv_a).collect(), &parameters[..=h_index])?;
    // B's piece H -> V, on B's own curve (reversed when B runs the other way).
    let b_piece_id = take_id();
    let (piece_curve, piece_t0, piece_t1) = if b_hair < end.b_vertex {
        (boundary.curve.clone(), b_hair, end.b_vertex)
    } else {
        let reversed = boundary.curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        let [low, high] = boundary.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        (reversed, low + high - b_hair, low + high - end.b_vertex)
    };
    // The piece's pcurve on the round: each sample's section (its foot on E)
    // gives u, and it runs along A's side of the rows.
    let v_a = if end.a_is_second { 1.0 } else { 0.0 };
    let mut piece_uv = Vec::with_capacity(PIECE_SAMPLES);
    let mut piece_at = Vec::with_capacity(PIECE_SAMPLES);
    for index in 0..PIECE_SAMPLES {
        let s = piece_t0 + (piece_t1 - piece_t0) * index as f64 / (PIECE_SAMPLES - 1) as f64;
        let u = if index == 0 {
            u_hair
        } else if index + 1 == PIECE_SAMPLES {
            1.0
        } else {
            let point = piece_curve.evaluate(s).or_refuse(KernelStage::Refine, "evaluate")?;
            let foot = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?.u;
            ((foot - end.e_start) / (end.e_vertex - end.e_start)).clamp(u_hair, 1.0)
        };
        piece_uv.push([u, v_a]);
        piece_at.push(s);
    }
    let piece_on_round = uv_samples(piece_uv, &piece_at)?;
    result.edges.push(EdgeRecord {
        id: b_piece_id,
        curve: piece_curve,
        t0: piece_t0,
        t1: piece_t1,
        start_vertex_id: hair_vertex,
        end_vertex_id: vertex,
        degenerate: false,
        name: None,
    });
    let (first_leg, second_leg) = if end.a_is_second {
        ((other_leg_id, other_leg_pcurve), (a_rail_id, a_rail_pcurve))
    } else {
        ((a_rail_id, a_rail_pcurve), (other_leg_id, other_leg_pcurve))
    };
    pending.push(PendingTaper {
        surface: taper.surface.clone(),
        center: taper.center.clone().ok_or(KernelRefusal::internal(KernelStage::Refine, "taper_centre", "taper runout: rows carry no centre path"))?,
        arc: (arc_edge_id, from_vertex, to_vertex),
        rims: [first_vertex, second_vertex],
        other_leg: other_leg_id,
        a_rail: a_rail_id,
        b_piece: b_piece_id,
        u_hair,
        piece_on_round,
        a_is_second: end.a_is_second,
        boundary: end.boundary,
        b_hair,
        hair_vertex,
        vertex,
        name: name.map(String::from),
        // centre = p + ρ·n_raw, and the face's outward normal is n_raw
        // flipped where its sense is: the ball is outside where they agree.
        ball_outside: (first.rho > 0.0) == first.face.same_sense,
    });
    Ok(Some(CapEnd {
        cr_parameter: station,
        cs_parameter: station,
        first_vertex,
        second_vertex,
        arc_edge_id,
        arc_blend_pcurve,
        first_leg,
        second_leg,
        sharp_vertex: vertex,
    }))
}

fn face_using(result: &mut BrepSolid, edge_id: u64, except: Option<u64>) -> Option<&mut FaceRecord> {
    result
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .filter(|face| Some(face.id) != except)
        .find(|face| face.loops.iter().flat_map(|ring| &ring.coedges).any(|coedge| coedge.edge_id == edge_id))
}

/// Sew every planned runout: the tapered round's face, B split at H with its
/// V-side piece moved from A to the round, and the cusp closed on a
/// degenerate edge. Runs after the stripes' surgery and the caps' splice,
/// which put each leg into its mate's loop (A's ends at H).
pub(in crate::blend) fn sew_taper_ends(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    pending: Vec<PendingTaper>,
) -> Result<(), KernelRefusal> {
    let refusal = |what: &'static str, message: &str| KernelRefusal::internal(KernelStage::Sew, what, format!("taper runout: {message}"));
    for taper in pending {
        let hair_point = result
            .vertices
            .iter()
            .find(|vertex| vertex.id == taper.hair_vertex)
            .ok_or(refusal("taper_hair", "H's vertex missing"))?
            .point;
        let a_face_id = face_using(result, taper.a_rail, None).ok_or(refusal("taper_a_face", "A lost its rail"))?.id;
        // B's other face: its coedge on B is split at H, the V-side piece
        // going to the piece edge.
        let (t0, t1, start_is_vertex) = {
            let boundary =
                result.edges.iter().find(|edge| edge.id == taper.boundary).ok_or(refusal("taper_boundary", "boundary edge missing"))?;
            (boundary.t0, boundary.t1, boundary.start_vertex_id == taper.vertex)
        };
        let band = crate::KernelTolerances::for_solid(result, 1e-7).pcurve_consistency;
        let (c_face_id, piece) = {
            let face = face_using(result, taper.boundary, Some(a_face_id)).ok_or(refusal("taper_c_face", "B has no face across from A"))?;
            let (ring_index, position) = face
                .loops
                .iter()
                .enumerate()
                .find_map(|(ring_index, ring)| {
                    ring.coedges.iter().position(|coedge| coedge.edge_id == taper.boundary).map(|position| (ring_index, position))
                })
                .ok_or(refusal("taper_c_coedge", "B's coedge missing"))?;
            let coedge = &face.loops[ring_index].coedges[position];
            let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let fraction = (taper.b_hair - t0) / (t1 - t0);
            let guess = if coedge.forward { q0 + (q1 - q0) * fraction } else { q1 - (q1 - q0) * fraction };
            let split = trim_parameter_at_point(&face.surface, &coedge.pcurve, guess, hair_point, band)?;
            let (low, high) = coedge.pcurve.split(split).or_refuse(KernelStage::Refine, "split")?;
            // Traversal runs q0 -> q1 and starts at V when (V is B's start)
            // == forward. The piece is stored H -> V.
            let (pcurve, forward, insert_at) =
                if start_is_vertex == coedge.forward { (low, false, position) } else { (high, true, position + 1) };
            (face.id, (ring_index, insert_at, forward, pcurve))
        };
        trim_edge_at(result, taper.boundary, taper.b_hair, taper.vertex, taper.hair_vertex)?;
        let c_forward = piece.2;
        {
            let face = result
                .shells
                .iter_mut()
                .flat_map(|shell| &mut shell.faces)
                .find(|face| face.id == c_face_id)
                .ok_or(refusal("taper_c_face", "B's other face lost"))?;
            let id = take_id();
            face.loops[piece.0].coedges.insert(piece.1, CoedgeRecord { id, edge_id: taper.b_piece, forward: c_forward, pcurve: piece.3 });
        }

        // The round's walk, `sew_cap`'s closed across the collapsed row: from
        // V back to the arc's end rim, the arc reversed, from its start rim to
        // V, the degenerate edge at V. A's side is two edges (rail to H, B's
        // piece to V). uv: u along the runout (0 at the first section, 1 at
        // V), v from the first mate (0) to the second (1).
        let (arc_id, arc_start, arc_end) = taper.arc;
        let a_rim = if taper.a_is_second { taper.rims[1] } else { taper.rims[0] };
        let v_of = |rim: u64| if rim == taper.rims[0] { 0.0 } else { 1.0 };
        let line = |a: (f64, f64), b: (f64, f64)| {
            crate::sweep_topology::parameter_line(a.0, a.1, b.0, b.1).or_refuse(KernelStage::Refine, "parameter_line")
        };
        // Rim -> V along one side, as (edge, forward, pcurve) walked forward.
        let side = |rim: u64| -> Result<Vec<CoedgeRecord>, KernelRefusal> {
            let v = v_of(rim);
            Ok(if rim == a_rim {
                vec![
                    CoedgeRecord { id: 0, edge_id: taper.a_rail, forward: true, pcurve: line((0.0, v), (taper.u_hair, v))? },
                    CoedgeRecord { id: 0, edge_id: taper.b_piece, forward: true, pcurve: taper.piece_on_round.clone() },
                ]
            } else {
                vec![CoedgeRecord { id: 0, edge_id: taper.other_leg, forward: true, pcurve: line((0.0, v), (1.0, v))? }]
            })
        };
        let mut coedges = Vec::new();
        // V -> arc end rim: that side reversed.
        for coedge in side(arc_end)?.into_iter().rev() {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: coedge.edge_id,
                forward: false,
                pcurve: coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?,
            });
        }
        let (v_in, v_out) = (v_of(arc_end), v_of(arc_start));
        coedges.push(CoedgeRecord { id: take_id(), edge_id: arc_id, forward: false, pcurve: line((0.0, v_in), (0.0, v_out))? });
        for coedge in side(arc_start)? {
            coedges.push(CoedgeRecord { id: take_id(), ..coedge });
        }
        if coedges.iter().any(|coedge| coedge.edge_id == taper.b_piece && coedge.forward == c_forward) {
            return Err(refusal("taper_orientation", "the round and B's other face walk B's piece the same way"));
        }
        let point = result.vertices.iter().find(|vertex| vertex.id == taper.vertex).ok_or(refusal("taper_vertex", "vertex missing"))?.point;
        let pole_id = take_id();
        result.edges.push(EdgeRecord {
            id: pole_id,
            curve: crate::make_line(point, point).or_refuse(KernelStage::Refine, "make_line")?,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: taper.vertex,
            end_vertex_id: taper.vertex,
            degenerate: true,
            name: None,
        });
        coedges.push(CoedgeRecord { id: take_id(), edge_id: pole_id, forward: true, pcurve: line((1.0, v_out), (1.0, v_in))? });
        // Orientation: outward is away from the ball centre on a convex edge
        // and towards it on a concave one, where the ball rolls in the air
        // (report b46db89a: both tapers read inward, and the body lost 101
        // of volume where its round adds 1.2).
        let surface_point = taper.surface.evaluate(0.5, 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
        let away = surface_point.sub(taper.center.evaluate(0.5).or_refuse(KernelStage::Refine, "evaluate")?);
        let outward = if taper.ball_outside { away.scale(-1.0) } else { away };
        let same_sense = taper.surface.normal(0.5, 0.5).or_refuse(KernelStage::Refine, "normal")?.dot(outward) > 0.0;
        let face = FaceRecord {
            id: take_id(),
            surface: taper.surface,
            same_sense,
            loops: vec![LoopRecord { id: take_id(), coedges }],
            name: taper.name.map(|name| format!("{name}:TAPER")),
        };
        let shell_index = result
            .shells
            .iter()
            .position(|shell| shell.faces.iter().any(|candidate| candidate.id == c_face_id))
            .ok_or(refusal("taper_shell", "no shell to sew the round into"))?;
        result.shells[shell_index].faces.push(face);
    }
    Ok(())
}
