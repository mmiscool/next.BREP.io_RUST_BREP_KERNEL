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
