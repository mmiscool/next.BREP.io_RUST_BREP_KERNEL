//! Face-profile extraction — a resident solid FACE as an extrude/revolve profile.
//!
//! Given a resolved `FaceRef`, order the face's boundary edges into closed
//! head-to-tail loop(s) so the profile-consumers (extrude, revolve) can build
//! from a prior solid's face exactly like from a sketch.
//!
//! Unlike the retired builder (which reconstructs lines/arcs from render polylines and
//! rejects everything else), this reads the EXACT trimmed edge curves off the
//! BREP — every boundary curve type a planar face can carry is supported. The
//! coedges of a loop are already stored in traversal order, so no endpoint
//! matching is needed: a `forward` coedge contributes the edge curve trimmed to
//! `[t0, t1]` as-is, a reversed one contributes it reversed, and the chain is
//! head-to-tail by construction.
//!
//! Loop convention (kernel-wide): `loops[0]` is the OUTER boundary, later loops
//! are holes; the outer loop traversed in coedge order runs CCW around the
//! OUTWARD face normal for either `same_sense`, so the emitted profile winds
//! positively about its `z_axis` and the consumers' name↔face alignment holds
//! without a winding flip. Nothing asserts the ORDER on the way in:
//! `validate_uv_wire` files its winding verdict under `wire_warnings`, which
//! `validate()` does not return — so a producer that emits `[hole, outer]` is
//! read here as an outer loop with a hole enclosing it (the offset shell's
//! completion lane did, and a revolve about the ring's outer edge refused the
//! "hole" for touching the axis; the producer now orders by area).
//!
//! Planar carriers only (the `common::face_frame` guard) — a curved face errors
//! loudly, no fallback.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FaceRef, ProfileLoop, SketchProfile};

/// Extract a [`SketchProfile`] (outer loop first, holes after; plane frame from
/// the face carrier) from a resident face. Errors loudly on a non-planar
/// carrier or a boundary the kernel cannot trim — never a silent drop.
pub fn face_profile(face: FaceRef) -> Result<SketchProfile, String> {
    // Plane frame first (short registry borrow inside): origin at the boundary
    // AABB center, z_axis = OUTWARD face normal. Non-planar carrier → loud error.
    let frame = common::face_frame(face).map_err(|error| format!("face-profile: {error}"))?;

    let loops = crate::with_registered_solid_str(face.handle, |solid| {
        let record = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|candidate| candidate.id == face.face_id)
            .ok_or_else(|| format!("face-profile: face {} not found on solid", face.face_id))?;

        let mut loops: Vec<ProfileLoop> = Vec::with_capacity(record.loops.len());
        for loop_record in &record.loops {
            let mut curves = Vec::with_capacity(loop_record.coedges.len());
            let mut edge_names = Vec::with_capacity(loop_record.coedges.len());
            for coedge in &loop_record.coedges {
                let edge = solid
                    .edges
                    .iter()
                    .find(|candidate| candidate.id == coedge.edge_id)
                    .ok_or_else(|| {
                        format!("face-profile: edge {} not found on solid", coedge.edge_id)
                    })?;
                if edge.degenerate {
                    continue; // pole/apex edges bound no length — skip
                }
                // The edge curve's domain may exceed [t0, t1] — trim to the
                // segment, then orient head-to-tail via the coedge sense.
                let trimmed = common::trimmed_curve(&edge.curve, edge.t0, edge.t1).map_err(|error| {
                    format!("face-profile: edge {} trim failed: {error}", edge.id)
                })?;
                let oriented = if coedge.forward {
                    trimmed
                } else {
                    trimmed.reversed()?
                };
                curves.push(oriented);
                edge_names.push(edge.name.clone());
            }
            if curves.is_empty() {
                continue; // an all-degenerate loop encloses nothing
            }
            // A single closed curve (a full-circle boundary — cylinder cap,
            // drilled hole) must become TWO segments: the extrude/revolve
            // builders require >= 2 curves, and `subtract_hole` silently skips
            // 1-curve loops. Mirror sketch.rs's circle-as-two-arcs convention.
            if curves.len() == 1 {
                let only = curves.pop().ok_or("face-profile: loop vanished")?;
                let [start, end] = only.domain()?;
                let (first, second) = only.split((start + end) * 0.5)?;
                let name = edge_names.pop().flatten();
                curves.push(first);
                curves.push(second);
                edge_names.push(name.clone());
                edge_names.push(name);
            }
            // No sketch behind a FACE profile, so no loop id — `ProfileLoop::key`
            // falls back to the loop's own lowest source edge name.
            loops.push(ProfileLoop {
                curves,
                edge_names,
                loop_id: None,
            });
        }
        if loops.is_empty() {
            return Err(format!(
                "face-profile: face {} has no usable boundary loop",
                face.face_id
            ));
        }
        Ok(loops)
    })?;

    Ok(SketchProfile {
        origin: frame.origin,
        x_axis: frame.x_axis,
        y_axis: frame.y_axis,
        z_axis: frame.z_axis,
        // ONE region: the face's outer loop + its holes (a face's hole loops
        // never nest islands, so the region is flat by construction).
        regions: vec![loops],
    })
}

