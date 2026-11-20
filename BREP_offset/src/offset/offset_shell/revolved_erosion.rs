//! The SHRUNK rounded parallel body of a solid of revolution whose half-section
//! is a polygon (`offset_solid` by −d with `OffsetCorners::Rounded`): the
//! half-section's erosion by d, revolved a full turn.
//!
//! A solid of revolution's distance to a point is read in that point's
//! meridian plane, and the nearest point of the full meridian section lies in
//! the point's own half-plane (its mirror across the axis is never nearer). So
//! the body's erosion is the revolution of its half-section's erosion, where
//! the run of the section along the axis bounds no material: it is no source of
//! distance, the erosion keeps the part of it it reaches, and the planar
//! eroder of [`super::prism_erosion`] reads it as an edge that is not real.
//!
//! Cause (brep-bench offset/flanged-drum-shrunk): a drum of radius 8 and height
//! 6 with a flange from radius 8 to 11 and z = 2.2 to 3.8, shrunk by 1. The
//! flange, 1.6 thick, is gone but for a ridge between two tori about its inner
//! rims; the pipeline refuses the flange's walls offset across each other, and
//! the by-pieces lane took 0.8 s.
//!
//! Scope: what [`super::revolved_parallel`] reads as a solid of revolution,
//! with every section piece a segment (planes square to the axis, cylinders and
//! cones about it). A segment that meets the axis must be square to it (a disk
//! face), so that the section has no corner on the axis. Anything else, a
//! piece of the erosion on the far side of the axis, and every knife edge of
//! the planar eroder decline. Like the prism lane it runs before the pipeline
//! only where an edge's offset vanishes or the body splits, and again after a
//! pipeline refusal.
use super::prism_erosion::{append_disjoint, eroded_loops, Piece, P2};
use super::revolved_parallel::half_section_polygon;
use super::*;
use crate::{make_arc, make_line, revolve_profile_brep};

/// `BREP_OFFSET_REVOLVED_EROSION` (default on; `=0` restores): [`revolved_erosion`].
pub(super) fn revolved_erosion_on() -> bool {
    switch!("BREP_OFFSET_REVOLVED_EROSION", true)
}


/// The shrunk rounded parallel body of `source` by `distance` (< 0), or `None`
/// when the lane does not apply. With `only_new_topology` it answers only
/// where the erosion drops an edge or leaves more than one body.
pub(super) fn revolved_erosion(source: &BrepSolid, distance: f64, only_new_topology: bool) -> Option<OffsetShellResultRecord> {
    if !revolved_erosion_on() || !(distance < 0.0) || !distance.is_finite() {
        return None;
    }
    match revolved_erosion_impl(source, -distance, only_new_topology) {
        Ok(result) => result,
        Err(reason) => {
            os_debug!("revolved erosion declined: {reason}");
            None
        }
    }
}

fn revolved_erosion_impl(source: &BrepSolid, depth: f64, only_new_topology: bool) -> Result<Option<OffsetShellResultRecord>, String> {
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;
    let section = half_section_polygon(source).ok_or("not a solid of revolution with a polygon section")?;
    let outline: Vec<P2> = section.corners.iter().map(|&(rho, z)| P2 { x: rho, y: z }).collect();
    let real: Vec<bool> = section.on_axis.iter().map(|on| !on).collect();
    let n = outline.len();
    if n < 3 {
        return Err("the section has no area".into());
    }
    // A real segment meeting the axis is square to it: no corner on the axis.
    for k in 0..n {
        let (a, b) = (outline[k], outline[(k + 1) % n]);
        let touches = a.x.abs() <= 1e-7 * scale || b.x.abs() <= 1e-7 * scale;
        if real[k] && touches && (a.y - b.y).abs() > 1e-7 * scale {
            return Err("a section edge meets the axis at a slant".into());
        }
    }
    let (loops, new_topology) = eroded_loops(&outline, &real, depth, tolerance)?;
    if only_new_topology && !new_topology {
        return Ok(None);
    }
    // Every piece stays in the half-plane.
    for piece in loops.iter().flatten() {
        for t in [0.0, 0.5, 1.0] {
            if piece.at(depth, t).x < -1e-7 * scale {
                return Err("a piece of the erosion crosses the axis".into());
            }
        }
    }
    let place = |point: P2| section.origin.add(section.x.scale(point.x)).add(section.axis.scale(point.y));
    let mut body: Option<BrepSolid> = None;
    for chain in &loops {
        let joints: Vec<P2> = chain
            .iter()
            .map(|piece| {
                let mut point = piece.at(depth, 0.0);
                // A joint on the axis sits on it exactly.
                if point.x.abs() <= 1e-7 * scale {
                    point.x = 0.0;
                }
                point
            })
            .collect();
        let mut curves = Vec::with_capacity(chain.len());
        for (k, piece) in chain.iter().enumerate() {
            let end = joints[(k + 1) % chain.len()];
            let curve = match *piece {
                Piece::Line { .. } => {
                    if joints[k].sub(end).len() <= tolerance {
                        continue;
                    }
                    make_line(place(joints[k]), place(end))?
                }
                Piece::Arc { center, start, sweep } if sweep > 0.0 => make_arc(place(center), section.x, section.axis, depth, start, start + sweep)?,
                Piece::Arc { center, start, sweep } => {
                    make_arc(place(center), section.x, section.axis.scale(-1.0), depth, -start, -start - sweep)?
                }
            };
            curves.push(curve);
        }
        let part = revolve_profile_brep(&curves, section.origin, section.axis, std::f64::consts::TAU)?;
        match &mut body {
            None => body = Some(part),
            Some(solid) => append_disjoint(solid, part),
        }
    }
    let solid = body.ok_or("nothing is left")?;
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!("the eroded revolution does not validate: {:?}", issues.first()));
    }
    let face_images = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|_| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: 0 })
        .collect();
    Ok(Some(OffsetShellResultRecord { solid, face_images }))
}
