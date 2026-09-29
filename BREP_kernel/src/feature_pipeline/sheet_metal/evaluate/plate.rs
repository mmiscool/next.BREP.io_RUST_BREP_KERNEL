use super::*;
use super::frame::Frame;

/// One flat → an exact-BREP plate placed by `frame`: extrude the closed outline
/// (midplane-centered on local z = 0) by `thickness` along local `+Z`, then move
/// it into the frame. A segment whose edge id keys [`Flat::outline_curves`] is
/// emitted from its EXACT flat-local curve (an arc stays a true cylindrical
/// wall — no chording); everything else is the straight chord between its two
/// outline vertices. Segment count, order and side-face naming are identical on
/// both paths, so curved outlines change no downstream name.
///
/// Face names (clean, of our own design): top `{id}:A`, bottom `{id}:B`, side wall
/// for outline segment i `{id}:SIDE:{edge_id}`.
pub(super) fn build_flat_plate(flat: &Flat, thickness: f64, frame: Frame) -> Result<BrepSolid, String> {
    let outline = &flat.outline;
    let n = outline.len();
    if n < 3 {
        return Err(format!(
            "sheet-metal: flat `{}` outline needs >= 3 points, got {n}",
            flat.id
        ));
    }
    let half_t = thickness * 0.5;
    let drop = Vec3::new(0.0, 0.0, -half_t);
    let mut profile = Vec::with_capacity(n);
    for i in 0..n {
        let a = outline[i];
        let b = outline[(i + 1) % n];
        let over = flat
            .edges
            .get(i)
            .and_then(|edge| flat.outline_curves.get(&edge.id));
        if let Some(curve) = over {
            // Desync tripwire: the exact curve must still span THIS segment's
            // two outline vertices. Any surgery that moved/renumbered vertices
            // while keeping the id fails loudly here, never builds skewed walls.
            let domain = curve.domain()?;
            let start = curve.evaluate(domain[0])?;
            let end = curve.evaluate(domain[1])?;
            let scale = (b[0] - a[0]).hypot(b[1] - a[1]).max(1.0);
            let miss = |p: Vec3, v: [f64; 2]| (p.x - v[0]).hypot(p.y - v[1]) > 1e-6 * scale;
            if miss(start, a) || miss(end, b) || start.z.abs().max(end.z.abs()) > 1e-6 {
                return Err(format!(
                    "sheet-metal: flat `{}` segment {i} (`{}`): exact outline curve no longer \
                     spans its outline vertices — the outline was reshaped out from under it",
                    flat.id,
                    flat.edges[i].id
                ));
            }
            profile.push(crate::feature_pipeline::features::common::translate_curve(
                curve, drop,
            )?);
        } else {
            profile.push(make_line(
                Vec3::new(a[0], a[1], -half_t),
                Vec3::new(b[0], b[1], -half_t),
            )?);
        }
    }
    let mut solid = extrude_profile_brep(&profile, Vec3::new(0.0, 0.0, 1.0), thickness)?;

    let faces = &mut solid
        .shells
        .get_mut(0)
        .ok_or("sheet-metal: flat plate produced no shell")?
        .faces;
    if faces.len() != n + 2 {
        return Err(format!(
            "sheet-metal: flat `{}` plate produced {} faces, expected {}",
            flat.id,
            faces.len(),
            n + 2
        ));
    }
    for (index, face) in faces.iter_mut().take(n).enumerate() {
        let edge_id = flat
            .edges
            .get(index)
            .map(|edge| edge.id.clone())
            .filter(|id| !id.is_empty())
            .unwrap_or_else(|| index.to_string());
        face.name = Some(format!("{}:SIDE:{edge_id}", flat.id));
    }
    faces[n].name = Some(format!("{}:B", flat.id));
    faces[n + 1].name = Some(format!("{}:A", flat.id));

    // Cut any holes (SM.CUTOUT bakes them into the flat, so they show folded AND
    // unfolded): subtract a through-thickness prism per hole loop.
    let solid = subtract_holes(solid, flat, thickness, half_t)?;

    transform_brep(&solid, frame.affine()?, false)
}

/// Subtract each [`Hole`] from `plate` as a prism over its exact outer loop.
/// Exact curves in → exact bores out: a circular loop cuts a true cylinder, in
/// the folded solid AND the flat pattern.
///
/// The through-thickness SPAN comes from the hole's `z_min`/`z_max` (midplane-
/// relative): an open (`None`) or beyond-face bound extends a full thickness
/// past that face so the boolean is a clean clearance cut; a strictly interior
/// bound is EXACT — the blind pocket floor/ceiling plane. A hole whose span
/// misses the slab entirely cuts nothing (skipped). Island loops are subtracted
/// from the CUTTER first (slightly taller prisms), so their interiors stay
/// material in the plate.
fn subtract_holes(
    plate: BrepSolid,
    flat: &Flat,
    thickness: f64,
    half_t: f64,
) -> Result<BrepSolid, String> {
    if flat.holes.is_empty() {
        return Ok(plate);
    }
    let options = BooleanOptions {
        merge_coplanar_faces: true,
        ..BooleanOptions::default()
    };
    let mut current = plate;
    for (hole_index, hole) in flat.holes.iter().enumerate() {
        if hole.outer.is_empty() {
            continue; // nothing to cut
        }
        // NOTE: a single CLOSED curve (e.g. one full-circle loop) is a valid
        // hole — it is halved below for the extrude. A genuinely broken loop
        // fails loudly in the extrude rather than being skipped silently.
        // Span → cutter bottom/top in flat-local z. `eps` guards exact-touch
        // bounds (a bound AT the face is "through" on that side, not a zero-
        // thickness skin).
        let eps = 1e-9 * thickness.max(1.0);
        let bottom = match hole.z_min {
            Some(z) if z > -half_t + eps => z,
            _ => -half_t - thickness,
        };
        let top = match hole.z_max {
            Some(z) if z < half_t - eps => z,
            _ => half_t + thickness,
        };
        if bottom >= half_t - eps || top <= -half_t + eps || top - bottom <= eps {
            continue; // span misses the slab — nothing removed on this flat
        }
        let drop = Vec3::new(0.0, 0.0, bottom);
        let mut profile = hole
            .outer
            .iter()
            .map(|curve| crate::feature_pipeline::features::common::translate_curve(curve, drop))
            .collect::<Result<Vec<_>, _>>()?;
        if profile.len() == 1 {
            // The extrude needs >= 2 profile curves; a one-curve loop (a full
            // make_circle) halves at mid-domain — exact, and deterministic for
            // the two `{flat}:CUTOUT:{i}:{0,1}` bore-face names.
            let single = profile.pop().expect("one curve");
            let domain = single.domain()?;
            let (first, second) = single.split((domain[0] + domain[1]) * 0.5)?;
            profile.push(first);
            profile.push(second);
        }
        let mut cutter = extrude_profile_brep(&profile, Vec3::new(0.0, 0.0, 1.0), top - bottom)?;
        // Per-SEGMENT bore-wall identity: extrude emits one side face per
        // profile curve IN ORDER, then the caps (the contract `build_flat_plate`
        // relies on too). Side face k gets the loop-positional name
        // `{flat}:CUTOUT:{hole}:{k}` — the pickable, deterministic anchor
        // hole-rim flanges resolve — while the caps keep the loop-level name.
        let side_count = profile.len();
        for (face_index, face) in cutter
            .shells
            .get_mut(0)
            .map(|shell| shell.faces.iter_mut())
            .into_iter()
            .flatten()
            .enumerate()
        {
            if face.name.is_none() {
                face.name = Some(if face_index < side_count {
                    format!("{}:CUTOUT:{hole_index}:{face_index}", flat.id)
                } else {
                    format!("{}:CUTOUT:{hole_index}", flat.id)
                });
            }
        }
        // Islands: carve each island prism OUT of the cutter (extended half a
        // thickness past both cutter caps so the caps never coincide), leaving
        // the island's material untouched when the cutter hits the plate.
        for (island_index, island) in hole.islands.iter().enumerate() {
            if island.is_empty() {
                continue; // nothing to keep
            }
            let island_drop = Vec3::new(0.0, 0.0, bottom - thickness * 0.5);
            let mut island_profile = island
                .iter()
                .map(|curve| {
                    crate::feature_pipeline::features::common::translate_curve(curve, island_drop)
                })
                .collect::<Result<Vec<_>, _>>()?;
            if island_profile.len() == 1 {
                // A single CLOSED curve (a full-circle island) is a valid island
                // loop — halve it at mid-domain exactly like the outer loop
                // above (the extrude needs >= 2 profile curves). Previously this
                // case was silently SKIPPED, deleting the island's material.
                let single = island_profile.pop().expect("one curve");
                let domain = single.domain()?;
                let (first, second) = single.split((domain[0] + domain[1]) * 0.5)?;
                island_profile.push(first);
                island_profile.push(second);
            }
            let mut island_prism = extrude_profile_brep(
                &island_profile,
                Vec3::new(0.0, 0.0, 1.0),
                (top - bottom) + thickness,
            )?;
            for face in island_prism
                .shells
                .get_mut(0)
                .map(|shell| shell.faces.iter_mut())
                .into_iter()
                .flatten()
            {
                if face.name.is_none() {
                    face.name =
                        Some(format!("{}:CUTOUT:{hole_index}:ISLAND:{island_index}", flat.id));
                }
            }
            let cutter_handle = crate::register_solid_value(cutter);
            let island_handle = crate::register_solid_value(island_prism);
            let carved =
                crate::with_two_registered_solids(cutter_handle, island_handle, |body, tool| {
                    boolean_operation(body, tool, BooleanOperation::Subtract, &options)
                });
            crate::free_registered_solid(cutter_handle);
            crate::free_registered_solid(island_handle);
            cutter =
                carved.map_err(|error| format!("sheet-metal island carve failed: {error}"))?;
        }
        let current_handle = crate::register_solid_value(current);
        let cutter_handle = crate::register_solid_value(cutter);
        let cut = crate::with_two_registered_solids(current_handle, cutter_handle, |body, tool| {
            boolean_operation(body, tool, BooleanOperation::Subtract, &options)
        });
        crate::free_registered_solid(current_handle);
        crate::free_registered_solid(cutter_handle);
        current = cut.map_err(|error| format!("sheet-metal hole cut failed: {error}"))?;
    }
    Ok(current)
}
