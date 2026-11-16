//! Lane O item 705 (2026-11-02): the ROUND at a TANGENT opening rim.
//!
//! An inward shell through an opening whose neighbour meets it smoothly (a
//! hemisphere cup shelled through its dome; `smOffsetTest_ShellBrep_705`'s
//! lofted prism through one of its arc walls) has no skin that reaches the
//! opening: the neighbour's offset runs parallel to the opening's
//! continuation at exactly `d`. Under the operation's own definition,
//! `source ∩ {dist to retained ≤ d}`, the material near such a rim is the
//! set within `d` of the rim EDGE, so its wall is a round of radius `d`
//! about the rim — a torus about a circular rim — tangent to the
//! neighbour's skin where they meet and cut by the opening where it exits.
//! The normal-ruled band is NOT the answer: for the cup it reproduces the
//! 16.788671 body the suite records as 7.6% short of the 18.164 integral.
//!
//! Switch `BREP_OFFSET_TANGENT_RIM_ROUND`, default off. The carrier is the
//! HALF tube on the opening's side, trimmed so its skin-side boundary is the
//! rim's exact image on the neighbour's skin (the tube's iso-curve toward
//! the rim's axis): the tube then never sections the skin tangentially, and
//! the weld's coincident one-use boundary lane sews that edge. Built from the
//! same template as the apex-cap sphere carrier (`pipeline.rs`).
use crate::{KernelRefusal, KernelStage, NurbsSurface, OrRefuse};
use super::*;
use super::rim_welds::fit_rim_circle;

pub(super) fn tangent_rim_round_enabled() -> bool {
    switch!("BREP_OFFSET_TANGENT_RIM_ROUND", false)
}


/// `BREP_OFFSET_COLLAPSED_RIM_TUBE` (default on; `=0` restores): see [`planar_rim_tube`].
pub(super) fn collapsed_rim_tube_enabled() -> bool {
    switch!("BREP_OFFSET_COLLAPSED_RIM_TUBE", true)
}

/// The wall where a COLLAPSED round meets a PLANAR opening tangentially along
/// a circle: the round's radius is under the shell's distance, so it has no
/// offset (it is omitted), and the material within `distance` of the round
/// near the opening is the set within `distance` of its END circle on the
/// opening's plane. The cavity is bounded there by the torus of tube radius
/// `distance` about that circle, from the mouth it opens on the plane down to
/// where it meets the next face's offset (a rounded frustum's top rim, a
/// rounded puck's: both refused "unwelded rim leaves a non-watertight shell").
/// The carrier is the tube's half on the material side of the plane, from its
/// inward point on the plane (the mouth) to its outward one; the selection
/// keeps the part the next offset leaves on the skin. `None` where the rim is
/// not a circle on the opening's plane, the shell is not inward, or the tube
/// would not fit inside the circle.
pub(super) fn planar_rim_tube(
    rim: &EdgeRecord,
    opening: &FaceRecord,
    distance: f64,
    tolerance: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    let Some(circle) = fit_rim_circle(rim, tolerance)? else {
        return Ok(None);
    };
    // The fit admits a rim within a thousandth of its radius; the tube is
    // exact about the FITTED circle, so the rim itself must lie on it to the
    // model's own accuracy (a fitted round's rim reads a few 1e-7 off), or the
    // wall would come out off its asked thickness (an ellipse 4.004 by 4 would
    // open a round mouth 0.002 off its true intercepts).
    let on_circle = tolerance.max(circle.radius * 1e-6);
    for index in 0..=96 {
        let point = rim.curve.evaluate(rim.t0 + (rim.t1 - rim.t0) * index as f64 / 96.0).or_refuse(KernelStage::Refine, "evaluate")?;
        let offset = point.sub(circle.center);
        let along = offset.dot(circle.axis);
        let radial = offset.sub(circle.axis.scale(along)).length();
        if along.abs() > on_circle || (radial - circle.radius).abs() > on_circle {
            os_debug!("collapsed rim tube declined: rim edge {} leaves its fitted circle by {:.3e}", rim.id, along.abs().max((radial - circle.radius).abs()));
            return Ok(None);
        }
    }
    if !(distance > 0.0) || distance >= circle.radius {
        return Ok(None);
    }
    // The opening must be a plane by its geometry, in any representation
    // (degree-elevated, revolved), holding the circle, with the circle's axis
    // its normal; the face's own normal only gives the side.
    let Some((plane_point, plane_normal)) = super::rim_welds::planar_surface_frame(&opening.surface, tolerance.max(1e-7))? else {
        return Ok(None);
    };
    if circle.axis.cross(plane_normal).length() > 1e-6 || plane_point.sub(circle.center).dot(plane_normal).abs() > tolerance.max(1e-7) {
        return Ok(None);
    }
    let [u0, u1] = opening.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = opening.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let outward = super::face_normal(opening, (u0 + u1) * 0.5, (v0 + v1) * 0.5)?;
    // The tube's section runs outward (v 0), INTO the material (0.25), inward
    // (0.5), so v in [0, 0.5] is the half under the plane and the sheet ends ON
    // the plane: its mouth (v 0.5) is an orphan rim the wall's cap takes in.
    // Turning the section that way (not the torus's own sense) points the
    // sheet's normal at the tube's core circle, the side the round's own
    // outward normal faces, as every inward carrier is oriented: the torus's
    // sense put the cavity inside out (the weld then turned the cavity's
    // other walls to agree with it).
    let axis = if circle.axis.dot(outward) > 0.0 { circle.axis } else { circle.axis.scale(-1.0) };
    let x_axis = axis.perpendicular().or_refuse(KernelStage::Refine, "perpendicular")?;
    let section = crate::make_arc(
        circle.center.add(x_axis.scale(circle.radius)),
        x_axis,
        axis.scale(-1.0),
        distance,
        0.0,
        std::f64::consts::TAU,
    )
    .or_refuse(KernelStage::Refine, "make_arc")?;
    let surface = crate::make_revolution(circle.center, axis, &section, std::f64::consts::TAU).or_refuse(KernelStage::Refine, "make_revolution")?;
    Ok(Some(tube_sheet(surface, 0.0, 0.5)?))
}

/// A tangent rim's round carrier where the rim is a CIRCLE: the torus about
/// it with tube radius `distance`, as the half tube from the skin tangency
/// (toward the circle's axis) through the opening's side of the rim plane
/// to the outward point. `None` where the rim is not a circle, the shell is
/// not inward, the tube would not fit inside the circle, or the opening does
/// not lie to one side of the rim plane.
pub(super) fn circular_rim_round(
    rim: &EdgeRecord,
    opening: &FaceRecord,
    distance: f64,
    tolerance: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    let Some(circle) = fit_rim_circle(rim, tolerance)? else {
        return Ok(None);
    };
    if !(distance > 0.0) || distance >= circle.radius {
        return Ok(None);
    }
    let [u0, u1] = opening.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = opening.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let probe = opening.surface.evaluate((u0 + u1) * 0.5, (v0 + v1) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
    let side = probe.sub(circle.center).dot(circle.axis);
    if side.abs() <= tolerance {
        return Ok(None);
    }
    // The torus's tube angle runs outward (v = 0), +axis (0.25), inward
    // (0.5), −axis (0.75): with the axis chosen so the opening lies on its
    // −axis side, the half tube v ∈ [0.5, 1] runs from the skin tangency
    // through the opening's side to the outward point.
    let axis = if side < 0.0 { circle.axis } else { circle.axis.scale(-1.0) };
    let surface = crate::make_torus_surface(circle.center, axis, circle.radius, distance).or_refuse(KernelStage::Refine, "make_torus_surface")?;
    Ok(Some(tube_sheet(surface, 0.5, 1.0)?))
}

/// A one-face sheet over the closed-in-u surface `surface` between the
/// iso-curves `v_from` and `v_to`: two closed boundary circles and one seam
/// edge used twice, the loop wound so the face's parameter-space area is
/// positive (the apex sphere's and the torus's own topology).
pub(super) fn tube_sheet(surface: NurbsSurface, v_from: f64, v_to: f64) -> Result<BrepSolid, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let near = surface.evaluate(u0, v_from).or_refuse(KernelStage::Refine, "evaluate")?;
    let far = surface.evaluate(u0, v_to).or_refuse(KernelStage::Refine, "evaluate")?;
    let near_circle = surface.iso_curve_v(v_from).or_refuse(KernelStage::Refine, "iso_curve_v")?;
    let far_circle = surface.iso_curve_v(v_to).or_refuse(KernelStage::Refine, "iso_curve_v")?;
    let seam = surface.iso_curve_u(u0).or_refuse(KernelStage::Refine, "iso_curve_u")?;
    let [c0, c1] = near_circle.domain().or_refuse(KernelStage::Refine, "domain")?;
    let [f0, f1] = far_circle.domain().or_refuse(KernelStage::Refine, "domain")?;
    let line = |a: f64, b: f64, c: f64, d: f64| crate::sweep_topology::parameter_line(a, b, c, d).or_refuse(KernelStage::Refine, "parameter_line");
    let mut face = FaceRecord {
        id: 128,
        surface,
        same_sense: true,
        name: None,
        loops: vec![LoopRecord {
            id: 127,
            coedges: vec![
                CoedgeRecord { id: 101, edge_id: 10, forward: true, pcurve: line(u0, v_from, u1, v_from)? },
                CoedgeRecord { id: 102, edge_id: 12, forward: true, pcurve: line(u1, v_from, u1, v_to)? },
                CoedgeRecord { id: 103, edge_id: 11, forward: false, pcurve: line(u1, v_to, u0, v_to)? },
                CoedgeRecord { id: 104, edge_id: 12, forward: false, pcurve: line(u0, v_to, u0, v_from)? },
            ],
        }],
    };
    if parameter_space_area(&face).or_refuse(KernelStage::Refine, "parameter_space_area")? < 0.0 {
        let loop_record = &mut face.loops[0];
        loop_record.coedges.reverse();
        for coedge in &mut loop_record.coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
    }
    Ok(BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: 130,
        genus: 0,
        kind: crate::BodyKind::Sheet,
        vertices: vec![VertexRecord { id: 1, point: near }, VertexRecord { id: 2, point: far }],
        edges: vec![
            EdgeRecord { id: 10, curve: near_circle, t0: c0, t1: c1, start_vertex_id: 1, end_vertex_id: 1, degenerate: false, name: None },
            EdgeRecord { id: 11, curve: far_circle, t0: f0, t1: f1, start_vertex_id: 2, end_vertex_id: 2, degenerate: false, name: None },
            EdgeRecord { id: 12, curve: seam, t0: v_from, t1: v_to, start_vertex_id: 1, end_vertex_id: 2, degenerate: false, name: None },
        ],
        shells: vec![ShellRecord { id: 129, faces: vec![face] }],
    })
}
