use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// Extend a hole-wall offset carrier's trim past the opening planes it
/// pierces, so the arrangement — not a rim weld — closes the opening.
///
/// `offset_face_carrier` trims the offset carrier with the parametric IMAGE
/// of the source trim: every source rim point moves `distance` along the
/// local surface normal. When the hole wall meets the pierced face
/// perpendicularly, that image stays in the opening plane and the coplanar
/// rim welds close the shell. When the wall is OBLIQUE (tilted drill axis)
/// or CONICAL, the normal has a component along the opening normal and the
/// image rim leaves the plane — for a tilted cylinder it oscillates
/// sinusoidally about it (z = z₀ + d·n_z(θ)), for a cone it is a coaxial
/// circle shifted along the axis by d·sin(half-angle). The pair imprint
/// against the opening plane then only exists where the image OVERSHOOTS
/// the plane; on the fall-short side the offset skin ends mid-air inside
/// the material, the source rim and the image rim both stay one-use, and
/// the honesty gate refuses the shell.
///
/// The correct opening wall is the piece of the ORIGINAL pierced face's
/// plane between the source rim and the offset carrier's exact plane
/// section (Parasolid hollow semantics: the cavity is trimmed by the
/// opening surface). That is precisely what the arrangement already builds
/// on its own whenever the offset carrier REACHES through the plane — the
/// carrier × opening-plane imprint is the exact conic (analytic plane ×
/// quadric SSI), the carrier keeps its main fragment, and the wall
/// fragment on the opening plane is the flat annulus between the drilled
/// rim and that conic. So instead of welding the gap after assembly,
/// rebuild the qualifying carrier's trim as the full-period band of its
/// surface extended to the surface's own v-domain ends, which lie beyond
/// the opening planes (drill tools always overshoot the stock). Exactness
/// is free: the new rims are surface isolines and the conic trims come
/// from the analytic intersection.
///
/// Qualification is deliberately narrow so every landed lane keeps its
/// existing path (straight holes: coplanar rim-pair welds; a frustum's outer
/// wall: the carrier is grown to its opening plane and the imprint cuts it,
/// `fallshort_opening_reach`):
/// - the carrier face is a closed band: nothing but closed full-period
///   rims (exactly two), face-local seams, and degenerate placeholders;
/// - each rim edge is shared with a PLANAR opening face as an INNER
///   (hole) loop of that face — an outer-loop rim is the frustum lane,
///   where the opening face ends at the wall and the closure is the annulus
///   the opening plane cuts;
/// - the rim's offset image genuinely leaves the opening plane (beyond
///   the weld tolerance band) — an in-plane image is the straight-hole
///   lane;
/// - both surface v-domain ends clear their opening planes on the far
///   side, so the extended band genuinely reaches through both openings
///   (when the surface itself stops short, nothing can close the shell
///   and the honest refusal stands).
pub(super) fn extend_offset_carriers_past_open_hole_rims(
    carriers: &mut [Carrier],
    source: &BrepSolid,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    smooth_pairs: &HashSet<(usize, usize)>,
    scale: f64,
) -> Result<HashSet<usize>, KernelRefusal> {
    let weld_band = 2e-3f64.max(scale * 5e-5);
    let source_edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let mut extended = HashSet::default();
    'carrier: for index in 0..carriers.len() {
        if !matches!(carriers[index].kind, OffsetFaceRole::Offset) {
            continue;
        }
        // Smooth-synchronized carriers had boundary curves rewritten against
        // their tangent neighbours; leave them to that machinery.
        if smooth_pairs
            .iter()
            .any(|(first, second)| *first == index || *second == index)
        {
            continue;
        }
        let Some(source_face) = source_faces
            .iter()
            .find(|face| face.id == carriers[index].source_face_id)
        else {
            continue;
        };
        let mut local_uses = HashMap::<u64, usize>::default();
        for coedge in source_face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
        {
            *local_uses.entry(coedge.edge_id).or_default() += 1;
        }
        // A closed band trims to nothing but rims + face-local seams.
        let mut rims = Vec::new();
        for (loop_index, loop_record) in source_face.loops.iter().enumerate() {
            for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                let Some(edge) = source_edge_by_id.get(&coedge.edge_id) else {
                    continue 'carrier;
                };
                if edge.degenerate {
                    continue;
                }
                match local_uses[&coedge.edge_id] {
                    2 => {}
                    1 if edge.start_vertex_id == edge.end_vertex_id => {
                        rims.push((loop_index, coedge_index, (*edge).clone()));
                    }
                    _ => continue 'carrier,
                }
            }
        }
        if rims.len() != 2 {
            continue;
        }
        let carrier_face = carriers[index].solid.shells[0].faces[0].clone();
        let surface = carrier_face.surface.clone();
        let [su0, su1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [sv0, sv1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let v_mid = (sv0 + sv1) * 0.5;
        // The band must close in u (a periodic wall around the hole bore).
        if surface
            .evaluate(su0, v_mid).or_refuse(KernelStage::Refine, "evaluate")?
            .sub(surface.evaluate(su1, v_mid).or_refuse(KernelStage::Refine, "evaluate")?)
            .length()
            > weld_band
        {
            continue;
        }
        let mut rim_planes = Vec::new();
        for (loop_index, coedge_index, rim_edge) in &rims {
            let source_coedge = &source_face.loops[*loop_index].coedges[*coedge_index];
            // Full-period rim: its pcurve sweeps the whole u domain.
            let [p0, p1] = source_coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let mut u_low = f64::MAX;
            let mut u_high = f64::MIN;
            let mut v_mean = 0.0f64;
            for sample in 0..=32 {
                let uv = source_coedge
                    .pcurve
                    .evaluate(p0 + (p1 - p0) * sample as f64 / 32.0).or_refuse(KernelStage::Refine, "evaluate")?;
                u_low = u_low.min(uv.x);
                u_high = u_high.max(uv.x);
                v_mean += uv.y / 33.0;
            }
            let u_span = su1 - su0;
            if (u_low - su0).abs() > u_span * 1e-3 || (u_high - su1).abs() > u_span * 1e-3 {
                continue 'carrier;
            }
            // The rim must bound a PLANAR opening face as an INNER loop.
            let Some(opening) = source_faces.iter().find(|face| {
                opening_set.contains(&face.id)
                    && face
                        .loops
                        .iter()
                        .flat_map(|loop_record| &loop_record.coedges)
                        .any(|coedge| coedge.edge_id == rim_edge.id)
            }) else {
                continue 'carrier;
            };
            let Some((plane_point, plane_normal)) =
                planar_surface_frame(&opening.surface, weld_band)?
            else {
                continue 'carrier;
            };
            let mut rim_loop_area = None;
            let mut largest_other = 0.0f64;
            for loop_record in &opening.loops {
                let area = parameter_space_area(&FaceRecord {
                    id: 0,
                    surface: opening.surface.clone(),
                    same_sense: true,
                    loops: vec![loop_record.clone()],
                    name: None,
                }).or_refuse(KernelStage::Refine, "parameter_space_area")?
                .abs();
                if loop_record
                    .coedges
                    .iter()
                    .any(|coedge| coedge.edge_id == rim_edge.id)
                {
                    rim_loop_area = Some(area);
                } else {
                    largest_other = largest_other.max(area);
                }
            }
            let Some(rim_loop_area) = rim_loop_area else {
                continue 'carrier;
            };
            if rim_loop_area >= largest_other {
                continue 'carrier;
            }
            // The offset image of the rim must genuinely leave the plane
            // (the carrier mirrors the source loop/coedge structure 1:1).
            let image_coedge = &carrier_face.loops[*loop_index].coedges[*coedge_index];
            let Some(image_edge) = carriers[index]
                .solid
                .edges
                .iter()
                .find(|edge| edge.id == image_coedge.edge_id)
            else {
                continue 'carrier;
            };
            let mut off_plane = 0.0f64;
            for sample in 0..=32 {
                let point = image_edge.curve.evaluate(
                    image_edge.t0 + (image_edge.t1 - image_edge.t0) * sample as f64 / 32.0,
                ).or_refuse(KernelStage::Refine, "evaluate")?;
                off_plane = off_plane.max(point.sub(plane_point).dot(plane_normal).abs());
            }
            if off_plane <= weld_band {
                continue 'carrier;
            }
            rim_planes.push((v_mean, plane_point, plane_normal));
        }
        // Pair each surface v-domain end with the opening plane of the rim
        // nearer to it, and require the extended isoline to clear that plane
        // on the far side (opposite the band interior).
        rim_planes.sort_by(|first, second| first.0.total_cmp(&second.0));
        let interior = surface.evaluate((su0 + su1) * 0.5, v_mid).or_refuse(KernelStage::Refine, "evaluate")?;
        for (v_end, (_, plane_point, plane_normal)) in [(sv0, rim_planes[0]), (sv1, rim_planes[1])]
        {
            let interior_sign = interior.sub(plane_point).dot(plane_normal).signum();
            let iso = surface.iso_curve_v(v_end).or_refuse(KernelStage::Refine, "iso_curve_v")?;
            let [t0, t1] = iso.domain().or_refuse(KernelStage::Refine, "domain")?;
            for sample in 0..=32 {
                let point = iso.evaluate(t0 + (t1 - t0) * sample as f64 / 32.0).or_refuse(KernelStage::Refine, "evaluate")?;
                if point.sub(plane_point).dot(plane_normal) * interior_sign > -weld_band {
                    continue 'carrier;
                }
            }
        }
        // Rebuild the carrier as the full-domain band with exact isoline
        // rims — structurally the same face a freshly made cylinder/cone
        // side carries, which is the arrangement's best-tested input.
        let winding = parameter_space_area(&carrier_face).or_refuse(KernelStage::Refine, "parameter_space_area")?;
        let bottom_rim = surface.iso_curve_v(sv0).or_refuse(KernelStage::Refine, "iso_curve_v")?;
        let top_rim = surface.iso_curve_v(sv1).or_refuse(KernelStage::Refine, "iso_curve_v")?;
        let seam = surface.iso_curve_u(su0).or_refuse(KernelStage::Refine, "iso_curve_u")?;
        let [bottom_t0, bottom_t1] = bottom_rim.domain().or_refuse(KernelStage::Refine, "domain")?;
        let [top_t0, top_t1] = top_rim.domain().or_refuse(KernelStage::Refine, "domain")?;
        let corner_bottom = surface.evaluate(su0, sv0).or_refuse(KernelStage::Refine, "evaluate")?;
        let corner_top = surface.evaluate(su0, sv1).or_refuse(KernelStage::Refine, "evaluate")?;
        let flat = |u: f64, v: f64| Vec3::new(u, v, 0.0);
        let mut coedges = vec![
            CoedgeRecord {
                id: 1,
                edge_id: 1,
                forward: true,
                pcurve: crate::make_line(flat(su0, sv0), flat(su1, sv0)).or_refuse(KernelStage::Refine, "make_line")?,
            },
            CoedgeRecord {
                id: 2,
                edge_id: 3,
                forward: true,
                pcurve: crate::make_line(flat(su1, sv0), flat(su1, sv1)).or_refuse(KernelStage::Refine, "make_line")?,
            },
            CoedgeRecord {
                id: 3,
                edge_id: 2,
                forward: false,
                pcurve: crate::make_line(flat(su1, sv1), flat(su0, sv1)).or_refuse(KernelStage::Refine, "make_line")?,
            },
            CoedgeRecord {
                id: 4,
                edge_id: 3,
                forward: false,
                pcurve: crate::make_line(flat(su0, sv1), flat(su0, sv0)).or_refuse(KernelStage::Refine, "make_line")?,
            },
        ];
        if winding < 0.0 {
            coedges.reverse();
            for coedge in &mut coedges {
                coedge.forward = !coedge.forward;
                coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
            }
        }
        os_debug!(
            "carrier[{index}] src={} extended past opening planes: band v=({sv0:.4},{sv1:.4})",
            carriers[index].source_face_id,
        );
        carriers[index].solid = BrepSolid {
            mass_properties_cache: Default::default(),
            id: carriers[index].solid.id,
            vertices: vec![
                VertexRecord {
                    id: 1,
                    point: corner_bottom,
                },
                VertexRecord {
                    id: 2,
                    point: corner_top,
                },
            ],
            edges: vec![
                EdgeRecord {
                    id: 1,
                    curve: bottom_rim,
                    t0: bottom_t0,
                    t1: bottom_t1,
                    start_vertex_id: 1,
                    end_vertex_id: 1,
                    degenerate: false,
                    name: None,
                },
                EdgeRecord {
                    id: 2,
                    curve: top_rim,
                    t0: top_t0,
                    t1: top_t1,
                    start_vertex_id: 2,
                    end_vertex_id: 2,
                    degenerate: false,
                    name: None,
                },
                EdgeRecord {
                    id: 3,
                    curve: seam,
                    t0: sv0,
                    t1: sv1,
                    start_vertex_id: 1,
                    end_vertex_id: 2,
                    degenerate: false,
                    name: None,
                },
            ],
            shells: vec![ShellRecord {
                id: 1,
                faces: vec![FaceRecord {
                    id: carrier_face.id,
                    surface,
                    same_sense: carrier_face.same_sense,
                    loops: vec![LoopRecord { id: 1, coedges }],
                    name: carrier_face.name.clone(),
                }],
            }],
            genus: 0,
        };
        extended.insert(index);
    }
    Ok(extended)
}

/// Fall-short curved carriers: the sibling disease to the oblique-bore lane
/// above, on the OTHER side of the opening. When a retained CURVED face meets
/// an opening face and its centre of curvature lies BEYOND the opening
/// surface, the offset image of the shared rim moves INTO the material —
/// radial offsetting scales the trim away from the plane. E.g. a sphere gouge
/// whose centre sits outside the box: cavity radius r offsets to r+d, and a
/// rim point p on the opening plane maps to c + (r+d)/r·(p−c), strictly on
/// the material side whenever c is beyond the plane. The trimmed carrier then
/// NEVER reaches the opening surface, the carrier × opening-wall SSI finds
/// nothing (correctly — the trimmed patches are disjoint), the offset skin
/// ends mid-air, and the honesty gate refuses the shell.
///
/// The cure is the oblique lane's cure: rebuild the qualifying carrier's trim
/// so the surface reaches THROUGH the opening plane and let the arrangement
/// cut it back with the exact plane × quadric section. Here the trim becomes
/// the surface's FULL domain (a gouge rim is a partial arc chain against the
/// opening's outer loop — there is no band structure to preserve); a pole at
/// a v-domain end collapses to a degenerate edge exactly like a freshly made
/// sphere face, the arrangement's best-tested input.
///
/// Qualification (narrow, so every landed lane keeps its path):
/// - Offset carrier, curved, not smooth-paired, not rebuilt by the oblique
///   lane above;
/// - the source face shares a non-degenerate edge with a PLANAR opening face;
/// - that rim's offset image falls ENTIRELY on the material side of the
///   opening plane (beyond the weld band) — an image that reaches or crosses
///   the plane is the arrangement's ordinary imprint case (it needs no help);
/// - the surface's full domain genuinely crosses the opening plane on the
///   outside — a surface that stops at the opening (cone base, frustum caps)
///   keeps its ruled-weld lane;
/// - the surface closes in u, so the full-domain face is seam-gluable.
pub(super) fn extend_fallshort_curved_carriers(
    carriers: &mut [Carrier],
    source: &BrepSolid,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    smooth_pairs: &HashSet<(usize, usize)>,
    already_extended: &HashSet<usize>,
    scale: f64,
    distance: f64,
) -> Result<usize, KernelRefusal> {
    let weld_band = 2e-3f64.max(scale * 5e-5);
    let source_edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let mut extended = 0usize;
    'carrier: for index in 0..carriers.len() {
        if !matches!(carriers[index].kind, OffsetFaceRole::Offset)
            || already_extended.contains(&index)
        {
            continue;
        }
        if smooth_pairs
            .iter()
            .any(|(first, second)| *first == index || *second == index)
        {
            continue;
        }
        let Some(source_face) = source_faces
            .iter()
            .find(|face| face.id == carriers[index].source_face_id)
        else {
            continue;
        };
        // Affine carriers translate along their normal: the shared rim's
        // image stays on the opening surface by construction.
        if source_face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
            continue;
        }
        let carrier_face = carriers[index].solid.shells[0].faces[0].clone();
        let surface = carrier_face.surface.clone();
        let [su0, su1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [sv0, sv1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let v_mid = (sv0 + sv1) * 0.5;
        // The full-domain rebuild needs a seam-gluable (u-closed) surface.
        if surface
            .evaluate(su0, v_mid).or_refuse(KernelStage::Refine, "evaluate")?
            .sub(surface.evaluate(su1, v_mid).or_refuse(KernelStage::Refine, "evaluate")?)
            .length()
            > weld_band
        {
            continue;
        }
        let mut qualifies = false;
        'rims: for (loop_index, loop_record) in source_face.loops.iter().enumerate() {
            for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                let Some(edge) = source_edge_by_id.get(&coedge.edge_id) else {
                    continue;
                };
                if edge.degenerate {
                    continue;
                }
                let Some(opening) = source_faces.iter().find(|face| {
                    opening_set.contains(&face.id)
                        && face
                            .loops
                            .iter()
                            .flat_map(|loop_record| &loop_record.coedges)
                            .any(|other| other.edge_id == coedge.edge_id)
                }) else {
                    continue;
                };
                let Some((plane_point, mut plane_normal)) =
                    planar_surface_frame(&opening.surface, weld_band)?
                else {
                    continue;
                };
                // Orient the plane normal OUTWARD (off the material) via the
                // opening face's own oriented normal. A revolved cap's
                // parameterization can be degenerate at some boundary uv
                // (partials vanish on the axis) — probe coedge midpoints
                // until one yields a normal; none at all disqualifies the rim
                // rather than failing the whole shell.
                let Some(outward) = opening
                    .loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .find_map(|opening_coedge| {
                        let [p0, p1] = opening_coedge.pcurve.domain().ok()?;
                        let opening_uv =
                            opening_coedge.pcurve.evaluate((p0 + p1) * 0.5).ok()?;
                        face_normal(opening, opening_uv.x, opening_uv.y).ok()
                    })
                else {
                    continue;
                };
                if plane_normal.dot(outward) < 0.0 {
                    plane_normal = plane_normal.scale(-1.0);
                }
                // A CLOSED full rim on the opening's OUTER loop is the ruled
                // weld's territory (a dome/frustum lateral ending at its cap:
                // the exact band between the coaxial rims is the landed
                // closure). A closed rim qualifies here only as an INNER
                // (hole) loop — a pocket carved interior to the opening face.
                // Open arc chains (a gouge across the face's outer boundary)
                // always qualify: no band structure exists for them.
                if edge.start_vertex_id == edge.end_vertex_id {
                    let mut rim_loop_area = None;
                    let mut largest_other = 0.0f64;
                    for loop_record in &opening.loops {
                        let area = parameter_space_area(&FaceRecord {
                            id: 0,
                            surface: opening.surface.clone(),
                            same_sense: true,
                            loops: vec![loop_record.clone()],
                            name: None,
                        }).or_refuse(KernelStage::Refine, "parameter_space_area")?
                        .abs();
                        if loop_record
                            .coedges
                            .iter()
                            .any(|other| other.edge_id == coedge.edge_id)
                        {
                            rim_loop_area = Some(area);
                        } else {
                            largest_other = largest_other.max(area);
                        }
                    }
                    let Some(rim_loop_area) = rim_loop_area else {
                        continue;
                    };
                    if rim_loop_area >= largest_other {
                        continue;
                    }
                }
                // Fall-short: the rim's offset image sits strictly on the
                // material side (the carrier mirrors the source loop/coedge
                // structure 1:1 — a missing mirror means another lane already
                // reshaped this carrier).
                let Some(image_coedge) = carrier_face
                    .loops
                    .get(loop_index)
                    .and_then(|loop_record| loop_record.coedges.get(coedge_index))
                else {
                    continue 'carrier;
                };
                let Some(image_edge) = carriers[index]
                    .solid
                    .edges
                    .iter()
                    .find(|edge| edge.id == image_coedge.edge_id)
                else {
                    continue 'carrier;
                };
                let mut falls_short = true;
                for sample in 0..=16 {
                    let point = image_edge.curve.evaluate(
                        image_edge.t0 + (image_edge.t1 - image_edge.t0) * sample as f64 / 16.0,
                    ).or_refuse(KernelStage::Refine, "evaluate")?;
                    if point.sub(plane_point).dot(plane_normal) > -weld_band {
                        falls_short = false;
                        break;
                    }
                }
                if !falls_short {
                    continue;
                }
                // The full domain must genuinely cross the opening plane —
                // otherwise extension cannot reach it either and the honest
                // refusal stands.
                let mut crosses = false;
                'grid: for iu in 0..=16 {
                    for iv in 0..=16 {
                        let point = surface.evaluate(
                            su0 + (su1 - su0) * iu as f64 / 16.0,
                            sv0 + (sv1 - sv0) * iv as f64 / 16.0,
                        ).or_refuse(KernelStage::Refine, "evaluate")?;
                        if point.sub(plane_point).dot(plane_normal) > weld_band {
                            crosses = true;
                            break 'grid;
                        }
                    }
                }
                if crosses {
                    qualifies = true;
                    break 'rims;
                }
            }
        }
        if !qualifies {
            continue;
        }
        if rebuild_carrier_full_domain(
            &mut carriers[index],
            source_face,
            weld_band,
            distance,
            index,
            "a fall-short opening rim",
        )? {
            extended += 1;
        }
    }
    Ok(extended)
}

/// CUT-RIM lane: a retained face whose rim with a PLANAR opening runs ACROSS
/// its surface — a boolean cut of a loft wall or of an extruded free-form
/// wall, rather than an opening at the surface's own v end — has its offset
/// carrier built over the FULL RECTANGLE of its surface's domain, so that the
/// opening plane truncates the offset skin the way it does on every other
/// road. Returns the source solid with that face's trim so rebuilt, for the
/// carrier to be built over (the carve's own substitution), or `None`.
///
/// `offset_face_carrier` trims the carrier with the parametric image of the
/// source trim, so a carrier built over the cut face ends at the cut rim's
/// image. For an inward shell that image lies INSIDE the material wherever
/// the plane rises away from the wall (the free-form prism cut at 0.08, over
/// part of its rim) and outside wherever the plane dips (the rest of it) —
/// the MIXED rim — or inside all round (the twisted loft's oblique cut).
/// Where it is inside the carrier never reaches the opening, the pair imprint
/// finds nothing, the rim dangles and the rim band lane corked it BELOW the
/// plane: the twisted loft's oblique cut validated 14.172505 against the plane
/// bracket 14.242887, 0.49 % short, and the void under its band was material
/// (the cork witness 0.149 from retained material at d = 0.3, 2026-09-18). The
/// mixed rim refused by name.
///
/// Neither `fallshort_opening_reach` (it grows the DOMAIN along the rulings
/// from the surface's own v end, which a cut rim does not sit at) nor
/// `extend_fallshort_curved_carriers` (u-closed surfaces whose image falls
/// short ENTIRELY, rebuilt with a seam after the fact) reaches such a rim.
/// This lane rebuilds the SOURCE trim before the carrier is fitted, so the
/// carrier is the full-domain offset with a four-sided trim — the isolines at
/// u₀, u₁, v₀ and v₁ — which the smooth-junction sync then rewrites against
/// its tangent neighbours exactly as it does any other carrier (the walls of
/// a C1 free-form prism are all smooth-paired, and a carrier reshaped AFTER
/// that sync would lose it). The skin runs through the plane on the inside
/// stretches and past it on the outside ones; the carrier × opening-plane
/// imprint cuts the exact trace, the arrangement keeps the fragments on the
/// material side, and the wall on the plane is the region between the source
/// rim and that trace.
///
/// Qualification, narrow so every landed lane keeps its path: an inward
/// shell; a face that is not u-closed (a tilted drill's wall is the seam
/// lanes' — its full domain is a band with a seam, not a rectangle, and
/// `oblique_through_hole_rim_pair_still_refuses` pins that road); an AFFINE
/// face only where its rim image falls short of the plane by more than the
/// weld band (a drafted wall; a perpendicular wall's image lies in the
/// plane and keeps the ordinary road);
/// a non-degenerate rim shared with a PLANAR opening whose uv trace is NOT a
/// v-isoline at either domain end (an end rim is the reach lane's); and the
/// surface's full domain crossing the plane on the outside, so the rebuilt
/// trim genuinely reaches through it. It is NOT gated on the rim image
/// falling short: the mixed prism's inside stretch sits under the weld band
/// (1e-3 at d = 0.3) and read as "reaches everywhere", and a cut rim whose
/// image lies wholly past the plane is truncated by the same imprint either
/// way (the widening loft cut at 0.08 reads its plane bracket on both roads).
/// A face with more than one loop keeps its own trim: the rectangle would
/// drop the holes.
pub(super) fn cut_rim_full_rectangle_source(
    source: &BrepSolid,
    face: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    distance: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    if distance <= 0.0 || face.loops.len() != 1 {
        return Ok(None);
    }
    let affine = face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")?;
    let scale = crate::solid_scale(source);
    let weld_band = 2e-3f64.max(scale * 5e-5);
    let surface = &face.surface;
    let [su0, su1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [sv0, sv1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let v_span = (sv1 - sv0).abs().max(1.0);
    // A u-closed surface (a tilted drill through a box) is the seam lanes'
    // territory: its full domain is a band with a seam, not a rectangle.
    if surface
        .evaluate(su0, (sv0 + sv1) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?
        .sub(surface.evaluate(su1, (sv0 + sv1) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?)
        .length()
        <= weld_band
    {
        return Ok(None);
    }
    let edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let offsets = face_offsets(face);
    // Every TANGENT junction with a retained neighbour must run along a
    // domain isoline, so the rectangle keeps it as one of its four sides and
    // the smooth-junction sync still finds its image (`carrier_use_imaging`).
    // A sharp junction needs no image — the pair imprint cuts it — and a
    // drafted extrude's trapezoid sides run diagonally across their affine
    // patch. A face whose tangent junction is a curved trim boundary keeps
    // its own trim: the 2026-09-06 fillet-rail document's rebuilt fillet
    // face had six uses in its source loop and four in its carrier's, and
    // the sync indexed the sixth — a panic on master until this decline.
    let v_band = 1e-9 * v_span;
    let u_band = 1e-9 * (su1 - su0).abs().max(1.0);
    for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
        let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        let Some((mate, mate_use)) = junction_mate(source_faces, face, coedge.edge_id) else {
            continue;
        };
        if opening_set.contains(&mate.id) || !uses_are_tangent(face, coedge, mate, mate_use)? {
            continue;
        }
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut on = [true; 4];
        for sample in 0..=8 {
            let uv = coedge.pcurve.evaluate(q0 + (q1 - q0) * sample as f64 / 8.0).or_refuse(KernelStage::Refine, "evaluate")?;
            on[0] &= (uv.x - su0).abs() <= u_band;
            on[1] &= (uv.x - su1).abs() <= u_band;
            on[2] &= (uv.y - sv0).abs() <= v_band;
            on[3] &= (uv.y - sv1).abs() <= v_band;
        }
        if !on.iter().any(|hit| *hit) {
            os_debug!(
                "cut-rim lane: source face {} keeps its trim — its TANGENT junction with retained \
                 face {} (edge {}) runs along no domain isoline, so a rectangle would lose it",
                face.id,
                mate.id,
                coedge.edge_id
            );
            return Ok(None);
        }
    }
    let mut qualifies = false;
    for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
        let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        let Some((opening, _)) = junction_mate(source_faces, face, coedge.edge_id)
            .filter(|(mate, _)| opening_set.contains(&mate.id))
        else {
            continue;
        };
        let Some((plane_point, mut plane_normal)) = planar_surface_frame(&opening.surface, weld_band)? else {
            continue;
        };
        let Some(outward) = opening
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .find_map(|opening_coedge| {
                let [p0, p1] = opening_coedge.pcurve.domain().ok()?;
                let opening_uv = opening_coedge.pcurve.evaluate((p0 + p1) * 0.5).ok()?;
                face_normal(opening, opening_uv.x, opening_uv.y).ok()
            })
        else {
            continue;
        };
        if plane_normal.dot(outward) < 0.0 {
            plane_normal = plane_normal.scale(-1.0);
        }
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut at_v_end = true;
        let mut deepest = f64::NEG_INFINITY;
        for sample in 0..=16 {
            let uv = coedge.pcurve.evaluate(q0 + (q1 - q0) * sample as f64 / 16.0).or_refuse(KernelStage::Refine, "evaluate")?;
            if (uv.y - sv0).abs() > 1e-9 * v_span && (uv.y - sv1).abs() > 1e-9 * v_span {
                at_v_end = false;
            }
            // How far the rim station's pointwise inward offset image sits on
            // the material side of the plane — a report quantity: the mixed
            // prism's inside stretch is under the weld band (1e-3 at d = 0.3),
            // so the rebuild is not gated on it. Every cut rim on a planar
            // opening takes this road, and the plane truncates the skin
            // whether the image was short of it or past it.
            if let Ok(image) = offsets.at(uv.x, uv.y, -distance) {
                deepest = deepest.max(-image.point.sub(plane_point).dot(plane_normal));
            }
        }
        if at_v_end {
            os_debug!("cut-rim lane: source face {} rim edge {} sits at a v end", face.id, coedge.edge_id);
            continue;
        }
        os_debug!(
            "cut-rim lane: source face {} rim edge {} image at most {deepest:.3e} inside the opening plane",
            face.id,
            coedge.edge_id
        );
        // An affine wall's rim image is a straight line a constant distance
        // from the plane: exactly in it for a wall perpendicular to the
        // opening (the ordinary planar case, left alone), d·sin(draft) short
        // for a DRAFTED wall — the drafted extrude's walls are affine
        // patches trimmed to trapezoids, whose carriers stopped 0.150 short
        // of the far cap at d = 0.5 on 17.5° walls. The band gate is safe
        // here because the shortfall does not vary along the rim.
        if affine && deepest <= weld_band {
            continue;
        }
        let mut crosses = false;
        'grid: for iu in 0..=16 {
            for iv in 0..=16 {
                let point = surface.evaluate(
                    su0 + (su1 - su0) * iu as f64 / 16.0,
                    sv0 + (sv1 - sv0) * iv as f64 / 16.0,
                ).or_refuse(KernelStage::Refine, "evaluate")?;
                if point.sub(plane_point).dot(plane_normal) > weld_band {
                    crosses = true;
                    break 'grid;
                }
            }
        }
        if !crosses {
            os_debug!(
                "cut-rim lane: source face {} never crosses opening {} over its full domain",
                face.id,
                opening.id
            );
            continue;
        }
        qualifies = true;
        break;
    }
    if !qualifies {
        return Ok(None);
    }
    os_debug!(
        "cut-rim lane: source face {} rebuilt over its full rectangle for the opening plane to truncate",
        face.id
    );
    Ok(Some(full_rectangle_face(source, face)?))
}

/// Replace one face's trim with its surface's parameter rectangle. Neighbour
/// carriers must subsequently cut this construction surface back to the skin.
pub(super) fn full_rectangle_face(source: &BrepSolid, face: &FaceRecord) -> Result<BrepSolid, KernelRefusal> {
    let surface = &face.surface;
    let [su0, su1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [sv0, sv1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    // The face's trim becomes the full rectangle: four isoline edges on four
    // new vertices appended to a copy of the source, the loop wound the way
    // the face's own trim was.
    let mut solid = source.clone();
    let mut next_id = solid
        .vertices
        .iter()
        .map(|vertex| vertex.id)
        .chain(solid.edges.iter().map(|edge| edge.id))
        .chain(solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| {
            face.loops
                .iter()
                .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.id).chain([loop_record.id]))
        }))
        .max()
        .unwrap_or(0)
        + 1;
    let mut vertex_ids = [0u64; 4];
    for (slot, (u, v)) in [(su0, sv0), (su1, sv0), (su1, sv1), (su0, sv1)].into_iter().enumerate() {
        solid.vertices.push(VertexRecord {
            id: next_id,
            point: surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?,
        });
        vertex_ids[slot] = next_id;
        next_id += 1;
    }
    let flat = |u: f64, v: f64| Vec3::new(u, v, 0.0);
    // Edges 0..4: v0 (0→1), u1 (1→2), v1 (3→2), u0 (0→3).
    let isolines = [
        (surface.iso_curve_v(sv0).or_refuse(KernelStage::Refine, "iso_curve_v")?, vertex_ids[0], vertex_ids[1], flat(su0, sv0), flat(su1, sv0), true),
        (surface.iso_curve_u(su1).or_refuse(KernelStage::Refine, "iso_curve_u")?, vertex_ids[1], vertex_ids[2], flat(su1, sv0), flat(su1, sv1), true),
        (surface.iso_curve_v(sv1).or_refuse(KernelStage::Refine, "iso_curve_v")?, vertex_ids[3], vertex_ids[2], flat(su1, sv1), flat(su0, sv1), false),
        (surface.iso_curve_u(su0).or_refuse(KernelStage::Refine, "iso_curve_u")?, vertex_ids[0], vertex_ids[3], flat(su0, sv1), flat(su0, sv0), false),
    ];
    let mut coedges = Vec::with_capacity(4);
    for (curve, start, end, from, to, forward) in isolines {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let edge_id = next_id;
        next_id += 1;
        solid.edges.push(EdgeRecord {
            id: edge_id,
            curve,
            t0,
            t1,
            start_vertex_id: start,
            end_vertex_id: end,
            degenerate: false,
            name: None,
        });
        coedges.push(CoedgeRecord {
            id: next_id,
            edge_id,
            forward,
            pcurve: crate::make_line(from, to).or_refuse(KernelStage::Refine, "make_line")?,
        });
        next_id += 1;
    }
    if parameter_space_area(face).or_refuse(KernelStage::Refine, "parameter_space_area")? < 0.0 {
        coedges.reverse();
        for coedge in &mut coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
    }
    let mut rebuilt = face.clone();
    rebuilt.loops = vec![LoopRecord { id: next_id, coedges }];
    for shell in &mut solid.shells {
        for existing in &mut shell.faces {
            if existing.id == face.id {
                *existing = rebuilt.clone();
            }
        }
    }
    Ok(solid)
}

/// Rebuild an offset carrier as its surface's FULL-DOMAIN face: seam edge
/// used twice, v-domain ends as closed rims — or degenerate point edges when
/// the iso curve collapses (poles), matching a freshly made sphere face. Any
/// interior loops (holes) in the source trim disappear with the rebuild — the
/// surplus sheet is cut back by the imprint pairs + fragment classification.
/// Returns false (carrier untouched) when the surface is not u-closed.
fn rebuild_carrier_full_domain(
    carrier: &mut Carrier,
    source_face: &FaceRecord,
    weld_band: f64,
    distance: f64,
    index: usize,
    reason: &str,
) -> Result<bool, KernelRefusal> {
    let carrier_face = carrier.solid.shells[0].faces[0].clone();
    let mut surface = carrier_face.surface.clone();
    let [mut su0, mut su1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [mut sv0, mut sv1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let v_mid = (sv0 + sv1) * 0.5;
    // The full-domain rebuild needs a seam-gluable (u-closed) surface.
    if surface
        .evaluate(su0, v_mid).or_refuse(KernelStage::Refine, "evaluate")?
        .sub(surface.evaluate(su1, v_mid).or_refuse(KernelStage::Refine, "evaluate")?)
        .length()
        > weld_band
    {
        return Ok(false);
    }
    // A SPHERE source gets its carrier rebuilt on the EXACT full offset
    // sphere. App-built sphere nets are v-clipped short of a pole, so
    // "full domain" of the fitted offset surface still misses a polar
    // cap; when the offset sphere pokes through the opening plane near
    // that pole (the missing-cap band is only a few degrees wide), the
    // rim arcs the shell needs run through the missing cap, the imprint
    // clips them away, and the loop on this carrier can never close.
    // The analytic rebuild also replaces the Greville fit's pole rows
    // (fit error grows unbounded at a degenerate row) with exact ones.
    if let Some(crate::AnalyticSurface::Sphere { frame, radius }) =
        crate::analytic_surface::recognize(&source_face.surface)
    {
        let mid = surface.evaluate((su0 + su1) * 0.5, (sv0 + sv1) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
        let measured = mid.sub(frame.origin).length();
        let grown = radius + distance.abs();
        let shrunk = (radius - distance.abs()).abs();
        // The fitted carrier disambiguates cavity (grown) vs boss
        // (shrunk); the analytic radius itself stays exact.
        let target = if (measured - grown).abs() <= (measured - shrunk).abs() {
            grown
        } else {
            shrunk
        };
        if (measured - target).abs() <= weld_band.max(distance.abs() * 0.5) && target > weld_band {
            let full = crate::make_sphere_surface(frame.origin, target, frame.axis).or_refuse(KernelStage::Refine, "make_sphere_surface")?;
            let [fu0, fu1] = full.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
            let [fv0, fv1] = full.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
            let old_normal = surface.normal((su0 + su1) * 0.5, (sv0 + sv1) * 0.5).or_refuse(KernelStage::Refine, "normal")?;
            let radial = mid.sub(frame.origin).normalized().or_refuse(KernelStage::Refine, "normalized")?;
            let sample = full.evaluate((fu0 + fu1) * 0.5, (fv0 + fv1) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
            let new_normal = full.normal((fu0 + fu1) * 0.5, (fv0 + fv1) * 0.5).or_refuse(KernelStage::Refine, "normal")?;
            let new_radial = sample.sub(frame.origin).normalized().or_refuse(KernelStage::Refine, "normalized")?;
            // The rebuilt net must agree with the fitted carrier on which
            // side the surface normal faces; a mismatch would silently
            // invert the face, so refuse the rebuild instead (the honest
            // refusal downstream is strictly better than a wrong solid).
            if (old_normal.dot(radial) > 0.0) == (new_normal.dot(new_radial) > 0.0) {
                os_debug!("carrier[{index}] rebuilt on the exact full offset sphere r={target:.6}");
                surface = full;
                (su0, su1, sv0, sv1) = (fu0, fu1, fv0, fv1);
            } else {
                os_debug!("carrier[{index}] sphere rebuild skipped: orientation mismatch");
            }
        } else {
            os_debug!(
                "carrier[{index}] sphere rebuild skipped: measured radius {measured:.6} \
                 matches neither grown nor shrunk offset of {radius:.6}"
            );
        }
    }
    let winding = parameter_space_area(&carrier_face).or_refuse(KernelStage::Refine, "parameter_space_area")?;
    let pole_point = |iso: &crate::NurbsCurve| -> Result<Option<Vec3>, KernelRefusal> {
        let [t0, t1] = iso.domain().or_refuse(KernelStage::Refine, "domain")?;
        let anchor = iso.evaluate(t0).or_refuse(KernelStage::Refine, "evaluate")?;
        let mut deviation = 0.0f64;
        for sample in 1..=8 {
            let point = iso.evaluate(t0 + (t1 - t0) * sample as f64 / 8.0).or_refuse(KernelStage::Refine, "evaluate")?;
            deviation = deviation.max(point.sub(anchor).length());
        }
        os_debug!("  pole probe: max deviation {deviation:.6}");
        Ok((deviation <= weld_band).then_some(anchor))
    };
    let bottom_iso = surface.iso_curve_v(sv0).or_refuse(KernelStage::Refine, "iso_curve_v")?;
    let top_iso = surface.iso_curve_v(sv1).or_refuse(KernelStage::Refine, "iso_curve_v")?;
    let bottom_pole = pole_point(&bottom_iso)?;
    let top_pole = pole_point(&top_iso)?;
    let corner_bottom = surface.evaluate(su0, sv0).or_refuse(KernelStage::Refine, "evaluate")?;
    let corner_top = surface.evaluate(su0, sv1).or_refuse(KernelStage::Refine, "evaluate")?;
    let seam = surface.iso_curve_u(su0).or_refuse(KernelStage::Refine, "iso_curve_u")?;
    let v_end_edge = |id: u64,
                      iso: crate::NurbsCurve,
                      pole: Option<Vec3>,
                      vertex: u64|
     -> Result<EdgeRecord, KernelRefusal> {
        Ok(match pole {
            Some(point) => EdgeRecord {
                id,
                curve: crate::make_line(point, point).or_refuse(KernelStage::Refine, "make_line")?,
                t0: 0.0,
                t1: 1.0,
                start_vertex_id: vertex,
                end_vertex_id: vertex,
                degenerate: true,
                name: None,
            },
            None => {
                let [t0, t1] = iso.domain().or_refuse(KernelStage::Refine, "domain")?;
                EdgeRecord {
                    id,
                    curve: iso,
                    t0,
                    t1,
                    start_vertex_id: vertex,
                    end_vertex_id: vertex,
                    degenerate: false,
                    name: None,
                }
            }
        })
    };
    let flat = |u: f64, v: f64| Vec3::new(u, v, 0.0);
    let mut coedges = vec![
        CoedgeRecord {
            id: 1,
            edge_id: 1,
            forward: true,
            pcurve: crate::make_line(flat(su0, sv0), flat(su1, sv0)).or_refuse(KernelStage::Refine, "make_line")?,
        },
        CoedgeRecord {
            id: 2,
            edge_id: 3,
            forward: true,
            pcurve: crate::make_line(flat(su1, sv0), flat(su1, sv1)).or_refuse(KernelStage::Refine, "make_line")?,
        },
        CoedgeRecord {
            id: 3,
            edge_id: 2,
            forward: false,
            pcurve: crate::make_line(flat(su1, sv1), flat(su0, sv1)).or_refuse(KernelStage::Refine, "make_line")?,
        },
        CoedgeRecord {
            id: 4,
            edge_id: 3,
            forward: false,
            pcurve: crate::make_line(flat(su0, sv1), flat(su0, sv0)).or_refuse(KernelStage::Refine, "make_line")?,
        },
    ];
    if winding < 0.0 {
        coedges.reverse();
        for coedge in &mut coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
    }
    os_debug!(
        "carrier[{index}] src={} extended to full domain past {reason} \
         (poles: bottom={} top={})",
        carrier.source_face_id,
        bottom_pole.is_some(),
        top_pole.is_some(),
    );
    carrier.solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: carrier.solid.id,
        vertices: vec![
            VertexRecord {
                id: 1,
                point: corner_bottom,
            },
            VertexRecord {
                id: 2,
                point: corner_top,
            },
        ],
        edges: vec![
            v_end_edge(1, bottom_iso, bottom_pole, 1)?,
            v_end_edge(2, top_iso, top_pole, 2)?,
            EdgeRecord {
                id: 3,
                curve: seam,
                t0: sv0,
                t1: sv1,
                start_vertex_id: 1,
                end_vertex_id: 2,
                degenerate: false,
                name: None,
            },
        ],
        shells: vec![ShellRecord {
            id: 1,
            faces: vec![FaceRecord {
                id: carrier_face.id,
                surface,
                same_sense: carrier_face.same_sense,
                loops: vec![LoopRecord { id: 1, coedges }],
                name: carrier_face.name.clone(),
            }],
        }],
        genus: 0,
    };
    Ok(true)
}

/// REFLEX-RIM lane: an offset carrier whose (curved, u-closed) source face
/// joins a RETAINED neighbour at a reflex edge cannot keep its source-sized
/// trim — the true offset∩offset junction lies PAST the cloned rim (miter
/// overshoot), and when the rim is an interior loop (a boss piercing the
/// face: cylinder through a cone) or a mid-surface arc, no amount of trim
/// stretching re-covers it (a hole excludes the junction band in uv
/// outright). Rebuild such carriers as full-domain faces: the imprint pairs
/// then carve the true junction into BOTH sheets as one shared cut and the
/// surplus is dropped by fragment classification. Runs on BOTH attempts (not
/// gated behind the extension retry): without it these shells close WRONGLY
/// (membrane caps over each rim copy), which is worse than any refusal.
pub(super) fn rebuild_reflex_rim_carriers(
    carriers: &mut [Carrier],
    source: &BrepSolid,
    source_faces: &[&FaceRecord],
    smooth_pairs: &HashSet<(usize, usize)>,
    scale: f64,
    distance: f64,
) -> Result<HashSet<usize>, KernelRefusal> {
    let weld_band = 2e-3f64.max(scale * 5e-5);
    let mut rebuilt = HashSet::default();
    for index in 0..carriers.len() {
        if !matches!(carriers[index].kind, OffsetFaceRole::Offset) {
            continue;
        }
        if smooth_pairs
            .iter()
            .any(|(first, second)| *first == index || *second == index)
        {
            continue;
        }
        let Some(source_face) = source_faces
            .iter()
            .find(|face| face.id == carriers[index].source_face_id)
        else {
            continue;
        };
        if source_face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
            continue;
        }
        if face_reflex_miter_tan(source, source_face)?.is_none() {
            continue;
        }
        if rebuild_carrier_full_domain(
            &mut carriers[index],
            source_face,
            weld_band,
            distance,
            index,
            "a reflex junction rim",
        )? {
            rebuilt.insert(index);
        }
    }
    Ok(rebuilt)
}

/// One sampled station of the junction between `face` and `mate` along
/// their shared edge: both faces' outward normals read at the SAME 3D
/// point, and `face`'s coedge-oriented tangent there.
pub(super) struct JunctionStation {
    pub normal: Vec3,
    pub mate_normal: Vec3,
    pub tangent: Vec3,
}

impl JunctionStation {
    /// Convex when the material dihedral is below π (a box's every edge),
    /// reflex above it (a pocket's internal corner). Sign convention
    /// verified on a box: all edges convex → cross·tangent > 0 with the
    /// coedge-oriented tangent.
    pub fn is_convex(&self) -> bool {
        self.normal.cross(self.mate_normal).dot(self.tangent) > 0.0
    }

    /// tan(θn/2), θn the angle between the two outward normals: the arc
    /// overshoot, per unit offset distance, at which the two faces' offsets
    /// meet past the source rim (1 for a perpendicular join, unbounded as
    /// the join becomes tangential).
    pub fn miter_tan(&self) -> f64 {
        let cos_normals = self.normal.dot(self.mate_normal).clamp(-1.0, 1.0);
        (cos_normals.acos() * 0.5).tan()
    }
}

/// Sample the sharp junction between `face` and `mate` along their shared
/// `edge` at five stations. Smooth stations — the same band
/// `uses_are_tangent` accepts for boundary synchronization — are left out:
/// a tangent join has no miter at all. The crossing angle varies along a
/// curved junction (a cylinder piercing a cone runs from steep to shallow
/// around the quartic rim), which is why callers keep the worst station.
pub(super) fn junction_stations(
    face: &FaceRecord,
    coedge: &CoedgeRecord,
    edge: &EdgeRecord,
    mate: &FaceRecord,
) -> Result<Vec<JunctionStation>, KernelRefusal> {
    let mut stations = Vec::new();
    for fraction in [0.1f64, 0.3, 0.5, 0.7, 0.9] {
        let t_sample = edge.t0 + (edge.t1 - edge.t0) * fraction;
        let step = ((edge.t1 - edge.t0) * 1e-3).max(1e-9);
        let before = edge.curve.evaluate(t_sample - step).or_refuse(KernelStage::Refine, "evaluate")?;
        let after = edge.curve.evaluate(t_sample + step).or_refuse(KernelStage::Refine, "evaluate")?;
        let mut tangent = after.sub(before);
        if tangent.length() <= 1e-12 {
            continue;
        }
        if !coedge.forward {
            tangent = tangent.scale(-1.0);
        }
        // Both normals are taken at the SAME 3D station, located on
        // each face by projection. Reading the two pcurves at equal
        // fractions assumed they share the edge's parameterization,
        // which a blend's end arc on a sphere does not: the normals
        // were then compared at different points of a TANGENT arc,
        // read as a sharp crossing, and the sign came out reflex for
        // two of a corner's three fillets — full-domain rebuilding
        // the corner sphere into a carrier nothing could trim.
        let station = edge.curve.evaluate(t_sample).or_refuse(KernelStage::Refine, "evaluate")?;
        let uv_this = {
            let projection = project_point_to_surface(&face.surface, station).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            Vec2 {
                x: projection.u,
                y: projection.v,
            }
        };
        let uv_mate = {
            let projection = project_point_to_surface(&mate.surface, station).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
            Vec2 {
                x: projection.u,
                y: projection.v,
            }
        };
        let normal = face_normal(face, uv_this.x, uv_this.y)?;
        let mate_normal = face_normal(mate, uv_mate.x, uv_mate.y)?;
        let cross = normal.cross(mate_normal);
        if cross.length() <= 1e-6 || normal.dot(mate_normal) >= SMOOTH_JUNCTION_COS {
            continue;
        }
        stations.push(JunctionStation {
            normal,
            mate_normal,
            tangent,
        });
    }
    Ok(stations)
}

/// The face that shares `edge_id` with `face`, with its use of that edge.
pub(super) fn junction_mate<'a>(
    faces: &[&'a FaceRecord],
    face: &FaceRecord,
    edge_id: u64,
) -> Option<(&'a FaceRecord, &'a CoedgeRecord)> {
    faces.iter().find_map(|other| {
        if other.id == face.id {
            return None;
        }
        other
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .find(|other_coedge| other_coedge.edge_id == edge_id)
            .map(|other_coedge| (*other, other_coedge))
    })
}

/// When `face` meets a neighbour at a REFLEX (concave) edge — the material
/// dihedral exceeds π (a pocket's internal corner: material wraps 270° around
/// the edge) — returns the WORST miter factor tan(θn/2) over samples along
/// its reflex edges, where θn is the angle between the two faces' outward
/// normals. At such an edge this face's offset skin must GROW past the source
/// footprint to reach the neighbour's offset: the offsets meet at arc
/// overshoot d·tan(θn/2) past the source rim (d for a perpendicular join,
/// unbounded as the join becomes tangential), so a flat d-sized extension
/// only covers joins at 90° or steeper. A smooth/tangent join (|n1×n2| ≈ 0)
/// is neither reflex nor convex. `None` = no reflex edge.
pub(super) fn face_reflex_miter_tan(source: &BrepSolid, face: &FaceRecord) -> Result<Option<f64>, KernelRefusal> {
    let edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let faces = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    let mut worst: Option<f64> = None;
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
                continue;
            };
            if edge.degenerate {
                continue;
            }
            let Some((mate, _)) = junction_mate(&faces, face, coedge.edge_id) else {
                continue;
            };
            for station in junction_stations(face, coedge, edge, mate)? {
                if !station.is_convex() {
                    worst = Some(worst.unwrap_or(0.0).max(station.miter_tan()));
                }
            }
        }
    }
    Ok(worst)
}

/// The factor an OUTWARD pad of |d| grows by so a junction whose offsets
/// meet `worst_tan`·d past the source rim (the worst sampled tan(θn/2),
/// `None` when no such junction bounds the pad) lands strictly INSIDE the
/// padded trim: ×1.5 headroom, as the reflex lane's inward extension, and
/// the same ×12 cap (a near-tangential join would ask for an unbounded
/// trim). Joins at 90° or steeper keep the exact |d| pad — every
/// perpendicular lane (a box's grown corner, a bore through a face) closes
/// on the boundary-coincident cut that pad produces, and the 1e-6 band
/// keeps a numerically-perpendicular join on that lane.
pub(super) fn outward_miter_scale(worst_tan: Option<f64>) -> f64 {
    match worst_tan {
        Some(tan) if tan > 1.0 + 1e-6 => (tan * 1.5).min(12.0),
        _ => 1.0,
    }
}

/// OUTWARD (negative distance) per-side extension of a retained face's
/// carrier: `amount` (= |distance|) across every side of the trim's uv
/// bounding box, EXCEPT a side the face runs along SMOOTHLY into retained
/// neighbours. At a sharp convex junction the two outward offsets separate
/// and meet again past both source rims, so each trim must grow to reach
/// the other (a box's grown corner); across an opening the wall plane cuts
/// the surplus back. At a smooth junction (a plane into its fillet, a
/// fillet's end into the corner blend) the offsets stay joined edge-to-edge,
/// and boundary synchronization pairs the two carriers only when both images
/// of the shared edge coincide — a uniform pad slid the plane's tangent edge
/// 0.6 sideways, no pair was ever made, and the tangent SSI cut nothing.
///
/// A side is smooth when at least one coedge RUNS ALONG it (≥90% of its
/// samples on the side — an edge merely ending there does not count) and
/// every such coedge is a smooth junction with a retained face. Ruled
/// carriers only ever grow along their rulings, so for them this decides
/// the two v-ends; the u sides are read but unused.
///
/// `amount` only reaches the meeting line of a convex junction at 90° or
/// steeper: the two outward offsets meet d·tan(θn/2) past the rim, and a
/// frustum's ACUTE base (material dihedral 90° − α) needs d·tan(45° + α/2)
/// — the cone carrier padded by exactly d stopped 0.13 above the padded
/// base plane, the pair imprint was skipped on bounds, and the cone's
/// bottom rim dangled one-use (the user's 2026-09-07 outward document).
/// So a sharp side grows by `outward_miter_scale` of the worst CONVEX
/// miter among the coedges along it; a side no coedge runs along (a
/// circular outline crosses all four) takes the face-wide worst. Reflex
/// coedges contribute nothing: those offsets cross inside the trims.
pub(super) fn outward_carrier_extension(
    source: &BrepSolid,
    face: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    amount: f64,
) -> Result<crate::CarrierExtension, KernelRefusal> {
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let mut uses: Vec<(Vec<Vec2>, bool, Option<f64>)> = Vec::new();
    let mut low = Vec2 {
        x: f64::INFINITY,
        y: f64::INFINITY,
    };
    let mut high = Vec2 {
        x: f64::NEG_INFINITY,
        y: f64::NEG_INFINITY,
    };
    for coedge in face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
    {
        if edge_by_id
            .get(&coedge.edge_id)
            .is_some_and(|edge| edge.degenerate)
        {
            continue;
        }
        let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let samples = (0..=24)
            .map(|sample| {
                coedge
                    .pcurve
                    .evaluate(p0 + (p1 - p0) * sample as f64 / 24.0)
                    .map(|uv| Vec2 { x: uv.x, y: uv.y })
                    .or_refuse(KernelStage::Refine, "evaluate")
            })
            .collect::<Result<Vec<_>, KernelRefusal>>()?;
        for uv in &samples {
            low.x = low.x.min(uv.x);
            low.y = low.y.min(uv.y);
            high.x = high.x.max(uv.x);
            high.y = high.y.max(uv.y);
        }
        let mate = junction_mate(source_faces, face, coedge.edge_id)
            .filter(|(mate, _)| !opening_set.contains(&mate.id));
        let (smooth, convex_tan) = match mate {
            Some((mate, mate_use)) => {
                let smooth = uses_are_tangent(face, coedge, mate, mate_use)?;
                let convex_tan = match edge_by_id.get(&coedge.edge_id) {
                    Some(edge) if !smooth => junction_stations(face, coedge, edge, mate)?
                        .iter()
                        .filter(|station| station.is_convex())
                        .map(JunctionStation::miter_tan)
                        .fold(None, |worst: Option<f64>, tan| {
                            Some(worst.unwrap_or(0.0).max(tan))
                        }),
                    _ => None,
                };
                (smooth, convex_tan)
            }
            None => (false, None),
        };
        uses.push((samples, smooth, convex_tan));
    }
    if uses.is_empty() || !low.x.is_finite() || !high.x.is_finite() {
        return Ok(crate::CarrierExtension::uniform(amount));
    }
    let worst_of = |first: Option<f64>, tan: Option<f64>| -> Option<f64> {
        match (first, tan) {
            (Some(a), Some(b)) => Some(a.max(b)),
            (a, b) => a.or(b),
        }
    };
    let face_worst = uses.iter().fold(None, |worst, (_, _, convex_tan)| {
        worst_of(worst, *convex_tan)
    });
    let side = |axis: fn(&Vec2) -> f64, value: f64, span: f64| -> f64 {
        let band = (span.abs() * 1e-5).max(1e-9);
        let mut along = 0usize;
        let mut smooth_along = 0usize;
        let mut worst_along: Option<f64> = None;
        for (samples, smooth, convex_tan) in &uses {
            let on = samples
                .iter()
                .filter(|uv| (axis(uv) - value).abs() <= band)
                .count();
            if on * 10 >= samples.len() * 9 {
                along += 1;
                if *smooth {
                    smooth_along += 1;
                }
                worst_along = worst_of(worst_along, *convex_tan);
            }
        }
        if along > 0 && along == smooth_along {
            return 0.0;
        }
        let worst = if along > 0 { worst_along } else { face_worst };
        amount * outward_miter_scale(worst)
    };
    Ok(crate::CarrierExtension {
        u_min: side(|uv| uv.x, low.x, u1 - u0),
        u_max: side(|uv| uv.x, high.x, u1 - u0),
        v_min: side(|uv| uv.y, low.y, v1 - v0),
        v_max: side(|uv| uv.y, high.y, v1 - v0),
    })
}

/// OUTWARD opening walls: how far the opening plane's uniform pad must
/// reach so the wall meets the offset of every retained neighbour where
/// that offset crosses the plane. The wall is the opening face's OWN plane
/// (not an offset), and a neighbour joined to it at material dihedral θ has
/// its outward offset cross the plane d/sin θ past the source rim — d for
/// a perpendicular join, d/cos α for a frustum narrowing toward the
/// opening: padded by exactly d the wall stopped 0.013 short of the offset
/// cone's section, the pair imprint cut nothing, and both the wall outline
/// and the cone carrier's top end dangled one-use (the user's 2026-09-07
/// outward document). Only CONVEX joins count — a reflex neighbour's
/// offset crosses the plane inside the opening's own footprint — and the
/// worst station goes through `outward_miter_scale`'s headroom and cap,
/// so perpendicular openings keep their exact d pad.
pub(super) fn outward_wall_pad(
    source: &BrepSolid,
    opening: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    amount: f64,
) -> Result<f64, KernelRefusal> {
    let (worst, _) =
        opening_wall_extension_factor(source, opening, source_faces, opening_set, true)?;
    Ok(amount * outward_miter_scale(worst))
}

/// An inward offset at a REFLEX opening rim crosses beyond the original
/// opening by d/sin(theta). Grow its construction plane past that intersection,
/// including perpendicular rims, so the cut is interior to the wall carrier.
/// Convex-only openings retain their original trims. As for the offset
/// carriers, cap the extension at 12d near tangency.
pub(super) fn inward_wall_pad(
    source: &BrepSolid,
    opening: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    amount: f64,
) -> Result<Option<(f64, HashSet<u64>)>, KernelRefusal> {
    let (worst, neighbors) =
        opening_wall_extension_factor(source, opening, source_faces, opening_set, false)?;
    Ok(worst.map(|scale| (amount * (scale * 1.5).min(12.0), neighbors)))
}

fn opening_wall_extension_factor(
    source: &BrepSolid,
    opening: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    convex: bool,
) -> Result<(Option<f64>, HashSet<u64>), KernelRefusal> {
    let edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let mut worst: Option<f64> = None;
    let mut neighbors = HashSet::default();
    for coedge in opening
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
    {
        let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        let Some((mate, _)) = junction_mate(source_faces, opening, coedge.edge_id)
            .filter(|(mate, _)| !opening_set.contains(&mate.id))
        else {
            continue;
        };
        for station in junction_stations(opening, coedge, edge, mate)? {
            if station.is_convex() != convex {
                continue;
            }
            let sin_dihedral = station.normal.cross(station.mate_normal).length();
            if sin_dihedral <= 1e-6 {
                continue;
            }
            neighbors.insert(mate.id);
            worst = Some(worst.unwrap_or(0.0).max(1.0 / sin_dihedral));
        }
    }
    Ok((worst, neighbors))
}

/// A curved opening face whose surface is RULED along v — a degree-1 net with
/// two control rows in v and at least three in u, the lateral face every
/// revolved or swept straight profile edge makes (a cone, a cylinder) — over a
/// single loop. An outward shell builds such an opening's wall by
/// [`extended_ruled_wall`].
pub(super) fn ruled_opening(face: &FaceRecord) -> Result<bool, KernelRefusal> {
    let surface = &face.surface;
    Ok(face.loops.len() == 1
        && !surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")?
        && surface.degree_v == 1
        && surface.control_points.len() >= 3
        && surface.control_points.iter().all(|row| row.len() == 2))
}

/// The OUTWARD wall of a ruled opening face: the face's own surface continued
/// past every open side by `pad`, as one face over the whole continuation.
///
/// A planar opening's wall is its plane padded on all four sides, and the pad
/// on the sides it shares with other openings is not decoration. A wall that
/// stops exactly on an adjacent opening's plane meets that plane along its own
/// BOUNDARY, and the imprint rejects a section lying on a face boundary that is
/// not a seam ("invalid-boundary"): measured on the torus wedge with the cone
/// grown only along its rulings, both caps lost the chord's section that way and
/// shredded into 57 fragments each. So the ruled wall grows along u as well.
///
/// The continuation is `NurbsSurface::extend_natural` — the analytic
/// continuation of each terminal Bézier span in homogeneous coordinates, so a
/// cone stays the same cone and a circle the same circle — followed by an AFFINE
/// re-mapping of both knot vectors back onto the original domain. The face's
/// cloned pcurves therefore bound the whole extended sheet, exactly as a padded
/// plane's cloned trim bounds the grown outline. That is only the intended wall
/// when the trim IS the whole face, so anything else declines. Each side's
/// parameter increment is sized from the boundary's own speed and then measured:
/// every station along the new boundary must lie as far from the old one as
/// the side asked. A side asks for `pad` unless that would outgrow the surface
/// it continues (`MAXIMUM_GROWTH_RATIO`), and then for just under the limit.
///
/// The inner `Err` carries the reason the construction does not apply, which
/// the shell refuses with. A closed direction has no sides to grow and is left
/// as it is.
pub(super) fn extended_ruled_wall(
    source: &BrepSolid,
    face: &FaceRecord,
    pad: f64,
    tolerance: f64,
) -> Result<Result<RuledWall, String>, KernelRefusal> {
    use crate::{NurbsSurface, SurfaceSide};
    let surface = &face.surface;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let loops = face
        .loops
        .iter()
        .map(|record| {
            record
                .coedges
                .iter()
                .map(|coedge| coedge.pcurve.clone())
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    if !crate::thicken::band::whole_face(&loops, [u0, u1], [v0, v1])? {
        return Ok(Err("the opening's trim is not the whole face".into()));
    }
    let mut wall = standalone_face(source, face.id)?;
    if wall.edges.iter().any(|edge| edge.degenerate) {
        return Ok(Err("the opening has a degenerate edge".into()));
    }
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    const STATIONS: usize = 8;
    // The largest distance, over stations across the side, between a point on
    // the old boundary and the point `delta` past it: the side's growth.
    let side_growth = |grown: &NurbsSurface, side: SurfaceSide, delta: f64| -> Result<f64, KernelRefusal> {
        let mut least = f64::INFINITY;
        for index in 0..=STATIONS {
            let fraction = index as f64 / STATIONS as f64;
            let (old, new) = match side {
                SurfaceSide::UMin | SurfaceSide::UMax => {
                    let v = v0 + (v1 - v0) * fraction;
                    let u = if matches!(side, SurfaceSide::UMin) { u0 } else { u1 };
                    let step = if matches!(side, SurfaceSide::UMin) { -delta } else { delta };
                    ((u, v), (u + step, v))
                }
                SurfaceSide::VMin | SurfaceSide::VMax => {
                    let u = u0 + (u1 - u0) * fraction;
                    let v = if matches!(side, SurfaceSide::VMin) { v0 } else { v1 };
                    let step = if matches!(side, SurfaceSide::VMin) { -delta } else { delta };
                    ((u, v), (u, v + step))
                }
            };
            let from = grown.evaluate(old.0, old.1).or_refuse(KernelStage::Refine, "evaluate")?;
            let to = grown.evaluate(new.0, new.1).or_refuse(KernelStage::Refine, "evaluate")?;
            least = least.min(to.sub(from).length());
        }
        Ok(least)
    };
    let mut grown = surface.clone();
    let mut deltas = [0.0f64; 4];
    let mut reached = [0.0f64; 4];
    for (slot, side) in [
        SurfaceSide::UMin,
        SurfaceSide::UMax,
        SurfaceSide::VMin,
        SurfaceSide::VMax,
    ]
    .into_iter()
    .enumerate()
    {
        if (side.is_u() && closed_u) || (!side.is_u() && closed_v) {
            continue;
        }
        // First guess from the boundary's speed, then measured: a chord of a
        // continued arc is shorter than its parameter speed says.
        let mut speed = f64::INFINITY;
        for index in 0..=STATIONS {
            let fraction = index as f64 / STATIONS as f64;
            let (u, v) = match side {
                SurfaceSide::UMin => (u0, v0 + (v1 - v0) * fraction),
                SurfaceSide::UMax => (u1, v0 + (v1 - v0) * fraction),
                SurfaceSide::VMin => (u0 + (u1 - u0) * fraction, v0),
                SurfaceSide::VMax => (u0 + (u1 - u0) * fraction, v1),
            };
            let derivatives = surface.derivatives(u, v, 1).or_refuse(KernelStage::Refine, "derivatives")?;
            let along = if side.is_u() { derivatives[1][0] } else { derivatives[0][1] };
            speed = speed.min(along.length());
        }
        if !(speed > 0.0) {
            return Ok(Err(format!("the opening's {side:?} side has no speed to grow along")));
        }
        let mut delta = pad / speed;
        // What this side is asked to reach. It shrinks only where the
        // continuation would outgrow the surface it continues
        // (`MAXIMUM_GROWTH_RATIO`): the pad carries the plane's miter headroom,
        // and a wall reaching less far than the pad but past the grown offset
        // still closes. One that falls short leaves the rim open, and the shell
        // refuses downstream rather than building.
        let mut reach = pad;
        let mut attempt = None;
        for _ in 0..8 {
            let candidate = match grown.extend_natural(side, delta) {
                Ok(candidate) => candidate,
                Err(crate::ExtendRefusal::ExcessiveGrowth { ratio, limit }) if ratio > limit => {
                    let shrink = limit / ratio * 0.95;
                    reach *= shrink;
                    delta *= shrink;
                    continue;
                }
                Err(refusal) => {
                    return Ok(Err(format!(
                        "the opening cannot be continued past its {side:?} side by {reach:.6}: {refusal}"
                    )));
                }
            };
            let growth = side_growth(&candidate, side, delta)?;
            if growth >= reach * (1.0 - 1e-9) {
                attempt = Some((candidate, growth));
                break;
            }
            delta *= (reach / growth.max(reach * 1e-3)) * 1.05;
        }
        let Some((candidate, growth)) = attempt else {
            return Ok(Err(format!(
                "the opening's {side:?} side does not reach {reach:.6} along its continuation"
            )));
        };
        reached[slot] = growth;
        grown = candidate;
        deltas[slot] = delta;
    }
    // Back onto the original domain, affinely: the extended [u0 − δ, u1 + δ]
    // becomes [u0, u1], so the unchanged pcurves bound the whole continuation.
    let remap = |knots: &[f64], low: f64, high: f64, start: f64, end: f64| -> Vec<f64> {
        knots
            .iter()
            .map(|knot| start + (knot - low) * (end - start) / (high - low))
            .collect()
    };
    let surface = NurbsSurface::new(
        grown.degree_u,
        grown.degree_v,
        remap(&grown.knots_u, u0 - deltas[0], u1 + deltas[1], u0, u1),
        remap(&grown.knots_v, v0 - deltas[2], v1 + deltas[3], v0, v1),
        grown.control_points.clone(),
    ).or_refuse(KernelStage::Refine, "new")?;
    // Every edge and vertex is the image of the same pcurves on the grown
    // sheet. An edge runs from its start vertex to its end; a pcurve runs in
    // its coedge's direction, which is the edge's exactly when `forward`.
    let wall_face = &mut wall.shells[0].faces[0];
    wall_face.surface = surface.clone();
    let uses = wall_face
        .loops
        .iter()
        .flat_map(|record| &record.coedges)
        .cloned()
        .collect::<Vec<_>>();
    for edge in &mut wall.edges {
        let Some(coedge) = uses.iter().find(|coedge| coedge.edge_id == edge.id) else {
            continue;
        };
        let image = crate::image_curve(&surface, &coedge.pcurve, tolerance, "offset_shell ruled wall").or_refuse(KernelStage::Refine, "image_curve")?;
        let (mut first, mut last) = (image.t0, image.t1);
        if !coedge.forward {
            std::mem::swap(&mut first, &mut last);
        }
        let mut curve = image.curve;
        if first > last {
            let [start, end] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            curve = curve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
            first = start + end - first;
            last = start + end - last;
        }
        let (start_point, end_point) = (curve.evaluate(first).or_refuse(KernelStage::Refine, "evaluate")?, curve.evaluate(last).or_refuse(KernelStage::Refine, "evaluate")?);
        for vertex in &mut wall.vertices {
            if vertex.id == edge.start_vertex_id {
                vertex.point = start_point;
            } else if vertex.id == edge.end_vertex_id {
                vertex.point = end_point;
            }
        }
        edge.curve = curve;
        edge.t0 = first;
        edge.t1 = last;
    }
    Ok(Ok(RuledWall { solid: wall, reached }))
}

/// A ruled opening's extended wall, and how far each side (u low, u high, v
/// low, v high) actually reaches — 0 for a closed direction.
pub(super) struct RuledWall {
    pub solid: BrepSolid,
    pub reached: [f64; 4],
}

/// The worst fold factor of a retained face's OUTWARD offset over its trim,
/// when it reaches the collapse factor: a fold the carve's own probe did not
/// see.
///
/// The carve lane reads a face through `ScanBudget::SHELL_FACE`, a coverage
/// budget, and acts only on samples it finds collapsed. A fold band narrower
/// than that grid is invisible to it, and the face is then built whole. Measured
/// on the torus wedge grown by 1.001, whose offset folds across its axis over a
/// band 2.9° wide: 0 of 448 samples collapsed, and once the chord cone's wall
/// reached the grown offset the pipeline built a valid single shell holding the
/// REFLECTED lemon's witness beyond the axis, which the answer leaves out. The
/// same torus with a knot inserted — identical geometry that no longer reads as
/// a revolution — built that solid through `offset_shell` too, because the
/// revolved-wedge lane declines a carrier it cannot recognize.
///
/// So this asks the same regularity question with the SAME field the carve
/// reads (`fold_sample_at`, the per-direction area factor `1 − δ·κ`), densely
/// and with refinement toward the worst node, which is where a band's minimum
/// is. The factor is `1 − grow` at the torus's inner equator, so the tangent
/// onset reads zero and refuses too. A caller asks only about faces the carve
/// did not carve and the collapse probe did not omit: a fold the carve DID see
/// is the carve's to build or refuse (the conic crater grown outward crosses its
/// axis near the apex, is carved, and reads its exact volume).
pub(super) fn outward_fold_the_carve_missed(
    face: &FaceRecord,
    distance: f64,
) -> Result<Option<crate::offset_regularity::FoldSample>, KernelRefusal> {
    use crate::offset_regularity::{fold_sample_at, scan_offset_regularity, TrimRegion};
    if distance >= 0.0 || face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(None);
    }
    let region = TrimRegion::from_face(face)?;
    let displacement = [shell_displacement(face, distance)];
    let scan = scan_offset_regularity(
        &face.surface,
        &region,
        &displacement,
        COLLAPSE_FACTOR,
        UNSEEN_FOLD_BUDGET,
    )?;
    let Some(mut worst) = scan.worst else {
        return Ok(None);
    };
    // The scan's refinement stops at a fixed depth, which leaves a smooth
    // minimum a few 1e-6 above its value — enough to read the tangent onset,
    // whose minimum IS zero, as regular. Polish to the minimum itself, one
    // coordinate at a time, on the same field.
    let [u_low, u_high, v_low, v_high] = region.bounds();
    let mut radius = [(u_high - u_low) / 64.0, (v_high - v_low) / 64.0];
    let ratio = 0.5 * (5f64.sqrt() - 1.0);
    for _ in 0..6 {
        for along_u in [true, false] {
            let (centre, low, high, reach) = if along_u {
                (worst.u, u_low, u_high, radius[0])
            } else {
                (worst.v, v_low, v_high, radius[1])
            };
            let sample = |t: f64| {
                let (u, v) = if along_u { (t, worst.v) } else { (worst.u, t) };
                region
                    .contains(u, v)
                    .then(|| fold_sample_at(&face.surface, u, v, &displacement))
                    .flatten()
            };
            let (mut a, mut b) = ((centre - reach).max(low), (centre + reach).min(high));
            for _ in 0..48 {
                let (c, d) = (b - ratio * (b - a), a + ratio * (b - a));
                match (sample(c), sample(d)) {
                    (Some(first), Some(second)) if first.factor < second.factor => b = d,
                    (Some(_), Some(_)) => a = c,
                    _ => break,
                }
            }
            if let Some(polished) = sample(0.5 * (a + b)) {
                if polished.factor < worst.factor {
                    worst = polished;
                }
            }
        }
        radius = [radius[0] * 0.5, radius[1] * 0.5];
    }
    Ok((worst.factor <= COLLAPSE_FACTOR).then_some(worst))
}

/// A fold hunt over one face: the sheet gate's density per span, a coverage cap
/// between the shell probe's and the sheet gate's, and the refinement that
/// walks to the minimum a band's thinness would otherwise hide.
const UNSEEN_FOLD_BUDGET: crate::offset_regularity::ScanBudget =
    crate::offset_regularity::ScanBudget {
        per_span: 9,
        steps_per_distance: 2.0,
        max_nodes: 4096,
        refine: true,
    };

/// How far an INWARD offset carrier must grow along its rulings to reach the
/// PLANAR opening its source face meets, and at which v end: `Some((at_v_max,
/// reach))`.
///
/// An inward offset moves the skin into the material, so the offset image of
/// the rim the opening shares lies short of the opening plane: a frustum's
/// r 2 → 1 h 4 shelled by 0.4 has its offset rim at z = 3.903 against an
/// opening at z = 4. The carrier and the opening wall are then genuinely
/// disjoint, their pair is skipped on bounds, nothing cuts the skin at the
/// plane, and the rim was left orphaned for the ruled cork weld (retired with
/// this change), which ends the wall BELOW the plane it opens. The opening plane truncates
/// the offset skin it opens, so the carrier is grown until it crosses that
/// plane and the ordinary imprint cuts it back exactly.
///
/// The reach is measured, not assumed: for each station on the shared rim, the
/// offset image is walked along the carrier's own ruling until it meets the
/// plane, and the worst station wins. A ruling parallel to the plane never
/// meets it and declines.
pub(super) fn fallshort_opening_reach(
    source: &BrepSolid,
    face: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    distance: f64,
) -> Result<Option<(bool, f64)>, KernelRefusal> {
    if distance <= 0.0 || face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(None);
    }
    let surface = &face.surface;
    // Ruled along v is what `offset_surface`'s ruled extension can grow.
    if surface.degree_v != 1 || surface.control_points.iter().any(|row| row.len() != 2) {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let offsets = face_offsets(face);
    let mut best: Option<(bool, f64)> = None;
    for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
        let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        // The opening this rim is shared with, and its plane.
        let Some((opening, _)) = junction_mate(source_faces, face, coedge.edge_id)
            .filter(|(mate, _)| opening_set.contains(&mate.id))
        else {
            continue;
        };
        if !surface_is_planar(&opening.surface, 1e-6 * crate::solid_scale(source).max(1.0))? {
            continue;
        }
        let [ou0, ou1] = opening.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [ov0, ov1] = opening.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let (ou, ov) = ((ou0 + ou1) * 0.5, (ov0 + ov1) * 0.5);
        let plane_point = opening.surface.evaluate(ou, ov).or_refuse(KernelStage::Refine, "evaluate")?;
        // A pole or an apex has no readable normal, and neither has an offset
        // ruling to walk: such a station is skipped, not refused.
        let Ok(plane_normal) = face_normal(opening, ou, ov) else {
            continue;
        };
        let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        for index in 0..=8 {
            let uv = coedge.pcurve.evaluate(p0 + (p1 - p0) * index as f64 / 8.0).or_refuse(KernelStage::Refine, "evaluate")?;
            // Which v end of the trim this rim sits at, and the ruling that
            // leaves the trim there.
            let at_v_max = (uv.y - v1).abs() < (uv.y - v0).abs();
            let u = uv.x.clamp(u0, u1);
            let (near, far) = if at_v_max { (v0, v1) } else { (v1, v0) };
            let image = |v: f64| offsets.at(u, v, -distance).map(|sample| sample.point).ok();
            let (Some(from), Some(to)) = (image(near), image(far)) else {
                continue;
            };
            let ruling = to.sub(from);
            let length = ruling.length();
            if length <= 1e-12 {
                continue;
            }
            let direction = ruling.scale(1.0 / length);
            let denominator = direction.dot(plane_normal);
            if denominator.abs() <= 1e-9 {
                continue;
            }
            // How far past the rim's own image the plane is, along the ruling.
            let reach = plane_point.sub(to).dot(plane_normal) / denominator;
            if reach <= 0.0 {
                continue;
            }
            if best.is_none_or(|(_, worst)| reach > worst) {
                best = Some((at_v_max, reach));
            }
        }
    }
    Ok(best)
}

/// A retained face whose INWARD offset skin stops short of a CURVED opening it
/// meets: `Some((opening face, the gap))`.
///
/// The planar case is grown to reach ([`fallshort_opening_reach`]). A curved
/// opening has no such construction: the wall would have to be the part of that
/// face between its rim and the offset skin's section with it, and nothing
/// builds that section when the skin never arrives. Left alone the pipeline
/// closes the cavity with whatever the completion reaches for — measured on the
/// hemisphere cup shelled through its DOME by 0.4, a valid single shell of
/// 16.788671 against 18.164 for the operation's own definition (a 400³ grid
/// integral of "the source less every point farther than d from a retained
/// face"), 7.6% short. That is refused by name instead.
pub(super) fn offset_falls_short_of_curved_opening(
    source: &BrepSolid,
    face: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    distance: f64,
) -> Result<Option<(u64, f64)>, KernelRefusal> {
    if distance <= 0.0 {
        return Ok(None);
    }
    let band = 2e-3f64.max(crate::solid_scale(source) * 5e-5);
    let edge_by_id = source
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<HashMap<_, _>>();
    let offsets = face_offsets(face);
    let mut worst: Option<(u64, f64)> = None;
    for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
        let Some(edge) = edge_by_id.get(&coedge.edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        let Some((opening, _)) = junction_mate(source_faces, face, coedge.edge_id)
            .filter(|(mate, _)| opening_set.contains(&mate.id))
        else {
            continue;
        };
        // GEOMETRIC planarity: a disk from a revolve is a rational patch lying
        // in a plane, and `is_affine` reads the net, not the geometry.
        if surface_is_planar(&opening.surface, band)? {
            continue;
        }
        let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        for index in 0..=8 {
            let uv = coedge.pcurve.evaluate(p0 + (p1 - p0) * index as f64 / 8.0).or_refuse(KernelStage::Refine, "evaluate")?;
            let Ok(image) = offsets.at(uv.x, uv.y, -distance) else {
                continue;
            };
            let gap = project_point_to_surface(&opening.surface, image.point).or_refuse(KernelStage::Refine, "project_point_to_surface")?.distance;
            if gap > band && worst.is_none_or(|(_, seen)| gap > seen) {
                worst = Some((opening.id, gap));
            }
        }
    }
    Ok(worst)
}

pub(super) fn source_by_id_lookup<'a>(
    source_faces: &[&'a FaceRecord],
    face_id: u64,
) -> Option<&'a FaceRecord> {
    source_faces.iter().find(|face| face.id == face_id).copied()
}

pub(super) fn sample_separation(first: &[Vec3], second: &[Vec3]) -> f64 {
    first
        .iter()
        .flat_map(|a| second.iter().map(move |b| a.sub(*b).length()))
        .fold(f64::INFINITY, f64::min)
}

pub(super) fn opening_wall_seeds(
    opening: &FaceRecord,
    carrier: &FaceRecord,
    source: &BrepSolid,
    opening_set: &HashSet<u64>,
    distance: f64,
) -> Result<Vec<Vec2>, KernelRefusal> {
    let source_faces = source
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    let mut seeds = Vec::new();
    for opening_use in opening
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
    {
        let Some((neighbor, neighbor_use)) = source_faces.iter().find_map(|face| {
            if face.id == opening.id || opening_set.contains(&face.id) {
                return None;
            }
            face.loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .find(|coedge| coedge.edge_id == opening_use.edge_id)
                .map(|coedge| (*face, coedge))
        }) else {
            continue;
        };
        let [opening_start, opening_end] = opening_use.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let opening_uv = opening_use
            .pcurve
            .evaluate((opening_start + opening_end) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
        let original = opening.surface.evaluate(opening_uv.x, opening_uv.y).or_refuse(KernelStage::Refine, "evaluate")?;
        let [neighbor_start, neighbor_end] = neighbor_use.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let neighbor_uv = neighbor_use
            .pcurve
            .evaluate((neighbor_start + neighbor_end) * 0.5).or_refuse(KernelStage::Refine, "evaluate")?;
        // The opening point stepped off along the NEIGHBOUR's offset direction:
        // the shared evaluator's normal, but anchored at a point on a different
        // face, so only the direction is borrowed.
        let expected = original.add(
            face_offsets(neighbor)
                .normal(neighbor_uv.x, neighbor_uv.y).or_refuse(KernelStage::Refine, "normal")?
                .scale(-distance),
        );
        let halfway = original.add(expected.sub(original).scale(0.5));
        let projection = project_point_to_surface(&carrier.surface, halfway).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        seeds.push(Vec2 {
            x: projection.u,
            y: projection.v,
        });
    }
    Ok(seeds)
}

/// SHARP-SIDE lane: a curved retained face whose u-side meets a retained
/// neighbour at a SHARP junction, in an OUTWARD shell, has its offset carrier
/// built over its surface CONTINUED past that side, so the two offsets meet
/// at their mitre. Returns the source with that face so rebuilt, for the
/// carrier to be built over (the cut-rim lane's substitution), `Ok(None)`
/// where the lane does not apply, or `Err` naming why the continuation
/// cannot be built.
///
/// An outward shell's offsets meet at a SHARP convex corner past both source
/// rims: that is the L∞ convention `l-step-open-bottom` reads. A plane's
/// carrier grows there by sliding its net, but a curved carrier's
/// u-extension is a no-op (`offset_surface_constructed` grows only along
/// v rulings). A circle-segment D (a line closed by a circular arc) shelled
/// outward therefore refused with "connector edge left its plane" at every
/// distance (2026-09-26): the offset cylinder stopped at the arc's end ray,
/// short of the offset plane.
///
/// The continuation is `NurbsSurface::extend_natural`: the analytic
/// continuation of the terminal Bézier span in homogeneous coordinates. It is
/// EXACT on a conic, because a rational arc's continuation stays on its
/// circle, and on a plane or a ruling. On a polynomial free-form wall it is an
/// extrapolation, which the mitre is then defined on. Either way it is bounded:
/// each side grows by `1.5 ×` the extension the pipeline asked of it (headroom
/// so the crossing lands inside the trim, not on it). `extend_natural` refuses
/// a weight root, a fold or excessive growth, and the grown strip's offset
/// image must advance MONOTONICALLY across it (a regular offset, no cusp).
/// Each is a refusal by name, not the connector's symptom. Nothing inside
/// the original domain moves, so every `(u, v)` still names the same point.
///
/// Qualification, narrow: an outward shell; a curved (non-affine) face, not
/// u-closed; one loop of exactly four uses, one along each domain isoline (a
/// full-rectangle trim, every extruded wall's); and a u-side the pipeline
/// asked to grow whose mate is RETAINED and meets it at a sharp junction. The
/// loop is edited IN PLACE: use `i` stays the same side, so
/// `carrier_use_imaging`'s equal-count shortcut still images a tangent
/// v-side neighbour correctly.
pub(super) fn sharp_side_continued_source(
    source: &BrepSolid,
    face: &FaceRecord,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    distance: f64,
    extension: &crate::CarrierExtension,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    use crate::SurfaceSide;
    if distance >= 0.0 || face.loops.len() != 1 || face.loops[0].coedges.len() != 4 {
        return Ok(None);
    }
    if extension.u_min <= 0.0 && extension.u_max <= 0.0 {
        return Ok(None);
    }
    let surface = &face.surface;
    if surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(None);
    }
    let (closed_u, _) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    if closed_u {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let band_u = 1e-9 * (u1 - u0).abs().max(1.0);
    let band_v = 1e-9 * (v1 - v0).abs().max(1.0);
    // Which domain side each use runs along: 0 u0, 1 u1, 2 v0, 3 v1.
    let mut sides = [usize::MAX; 4];
    for (index, coedge) in face.loops[0].coedges.iter().enumerate() {
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut on = [true; 4];
        for sample in 0..=8 {
            let uv = coedge.pcurve.evaluate(q0 + (q1 - q0) * sample as f64 / 8.0).or_refuse(KernelStage::Refine, "evaluate")?;
            on[0] &= (uv.x - u0).abs() <= band_u;
            on[1] &= (uv.x - u1).abs() <= band_u;
            on[2] &= (uv.y - v0).abs() <= band_v;
            on[3] &= (uv.y - v1).abs() <= band_v;
        }
        let Some(side) = on.iter().position(|hit| *hit) else {
            return Ok(None);
        };
        sides[index] = side;
    }
    let mut seen = sides;
    seen.sort_unstable();
    if seen != [0, 1, 2, 3] {
        return Ok(None);
    }
    // The u-sides to continue: asked to grow, mate retained, junction sharp.
    let mut grow = [0.0f64; 2];
    for (index, coedge) in face.loops[0].coedges.iter().enumerate() {
        let side = sides[index];
        if side > 1 {
            continue;
        }
        let asked = if side == 0 { extension.u_min } else { extension.u_max };
        if asked <= 0.0 {
            continue;
        }
        let Some((mate, mate_use)) = junction_mate(source_faces, face, coedge.edge_id) else {
            continue;
        };
        if opening_set.contains(&mate.id) || uses_are_tangent(face, coedge, mate, mate_use)? {
            continue;
        }
        grow[side] = asked * 1.5;
    }
    if grow == [0.0, 0.0] {
        return Ok(None);
    }
    const STATIONS: usize = 8;
    let mut grown = surface.clone();
    let mut reach = [u0, u1];
    for (slot, side) in [SurfaceSide::UMin, SurfaceSide::UMax].into_iter().enumerate() {
        let wanted = grow[slot];
        if wanted <= 0.0 {
            continue;
        }
        let u_end = if slot == 0 { u0 } else { u1 };
        let sense = if slot == 0 { -1.0 } else { 1.0 };
        let mut speed = f64::INFINITY;
        for index in 0..=STATIONS {
            let v = v0 + (v1 - v0) * index as f64 / STATIONS as f64;
            speed = speed.min(surface.derivatives(u_end, v, 1).or_refuse(KernelStage::Refine, "derivatives")?[1][0].length());
        }
        if !(speed > 0.0) {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, refusals::SHARP_SIDE_CONTINUATION, format!(
                "offset_shell: source face {}'s offset carrier cannot be built past its sharp {side:?} \
                 side: the side has no speed to continue along",
                face.id
            )));
        }
        let mut delta = wanted / speed;
        let mut accepted = None;
        for _ in 0..8 {
            let candidate = grown.extend_natural(side, delta).map_err(|refusal| {
                KernelRefusal::unsupported(
                    KernelStage::Refine,
                    refusals::SHARP_SIDE_CONTINUATION,
                    format!(
                        "offset_shell: source face {}'s offset carrier cannot be built past its sharp \
                         {side:?} side by {wanted:.6}: {refusal}",
                        face.id
                    ),
                )
            })?;
            // The OFFSET image's travel across the strip, the least over the
            // stations, and that it advances monotonically at each.
            let offsets = crate::OffsetEvaluator::new(
                "offset_shell",
                &candidate,
                crate::OffsetNormal::Face { same_sense: face.same_sense },
            );
            let mut least = f64::INFINITY;
            for index in 0..=STATIONS {
                let v = v0 + (v1 - v0) * index as f64 / STATIONS as f64;
                let start = offsets.at(u_end, v, -distance).or_refuse(KernelStage::Refine, "at")?.point;
                let far = offsets.at(u_end + sense * delta, v, -distance).or_refuse(KernelStage::Refine, "at")?.point;
                let direction = far.sub(start);
                let mut previous = start;
                for step in 1..=16 {
                    let point = offsets.at(u_end + sense * delta * step as f64 / 16.0, v, -distance).or_refuse(KernelStage::Refine, "at")?.point;
                    if point.sub(previous).dot(direction) <= 0.0 {
                        return Err(KernelRefusal::unsupported(KernelStage::Refine, refusals::SHARP_SIDE_CONTINUATION, format!(
                            "offset_shell: source face {}'s offset carrier cannot be built past its \
                             sharp {side:?} side: the continued surface's offset turns back \
                             {:.3} of the way across the strip (a cusp of the offset)",
                            face.id,
                            step as f64 / 16.0
                        )));
                    }
                    previous = point;
                }
                least = least.min(direction.length());
            }
            if least >= wanted * (1.0 - 1e-9) {
                accepted = Some(candidate);
                break;
            }
            delta *= (wanted / least.max(wanted * 1e-3)) * 1.05;
        }
        let Some(candidate) = accepted else {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, refusals::SHARP_SIDE_CONTINUATION, format!(
                "offset_shell: source face {}'s offset carrier cannot be built past its sharp {side:?} \
                 side: its offset does not travel {wanted:.6} within eight refinements",
                face.id
            )));
        };
        grown = candidate;
        reach[slot] = u_end + sense * delta;
    }
    // The loop, edited in place: the same four uses in the same order and
    // sense, each now running along its side of the continued rectangle.
    let mut solid = source.clone();
    let mut next_id = solid
        .vertices
        .iter()
        .map(|vertex| vertex.id)
        .chain(solid.edges.iter().map(|edge| edge.id))
        .chain(solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| {
            face.loops
                .iter()
                .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.id).chain([loop_record.id]))
        }))
        .max()
        .unwrap_or(0)
        + 1;
    let [ua, ub] = reach;
    let corners = [(ua, v0), (ub, v0), (ub, v1), (ua, v1)];
    let mut corner_ids = [0u64; 4];
    for (slot, (u, v)) in corners.into_iter().enumerate() {
        solid.vertices.push(VertexRecord { id: next_id, point: grown.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")? });
        corner_ids[slot] = next_id;
        next_id += 1;
    }
    // Each side's isoline in increasing parameter, with its corner vertices.
    let side_edge = |side: usize| -> Result<(crate::NurbsCurve, u64, u64, Vec3, Vec3), KernelRefusal> {
        let flat = |u: f64, v: f64| Vec3::new(u, v, 0.0);
        Ok(match side {
            0 => (grown.iso_curve_u(ua).or_refuse(KernelStage::Refine, "iso_curve_u")?, corner_ids[0], corner_ids[3], flat(ua, v0), flat(ua, v1)),
            1 => (grown.iso_curve_u(ub).or_refuse(KernelStage::Refine, "iso_curve_u")?, corner_ids[1], corner_ids[2], flat(ub, v0), flat(ub, v1)),
            2 => (grown.iso_curve_v(v0).or_refuse(KernelStage::Refine, "iso_curve_v")?, corner_ids[0], corner_ids[1], flat(ua, v0), flat(ub, v0)),
            _ => (grown.iso_curve_v(v1).or_refuse(KernelStage::Refine, "iso_curve_v")?, corner_ids[3], corner_ids[2], flat(ua, v1), flat(ub, v1)),
        })
    };
    let mut rebuilt = face.clone();
    rebuilt.surface = grown.clone();
    for (index, coedge) in rebuilt.loops[0].coedges.iter_mut().enumerate() {
        let (curve, start, end, from, to) = side_edge(sides[index])?;
        // The old use's own sense along its isoline.
        let [q0, q1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let (a, b) = (coedge.pcurve.evaluate(q0).or_refuse(KernelStage::Refine, "evaluate")?, coedge.pcurve.evaluate(q1).or_refuse(KernelStage::Refine, "evaluate")?);
        let increasing = if sides[index] <= 1 { b.y > a.y } else { b.x > a.x };
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let edge_id = next_id;
        next_id += 1;
        solid.edges.push(EdgeRecord {
            id: edge_id,
            curve,
            t0,
            t1,
            start_vertex_id: start,
            end_vertex_id: end,
            degenerate: false,
            name: None,
        });
        coedge.edge_id = edge_id;
        coedge.forward = increasing;
        coedge.pcurve = if increasing { crate::make_line(from, to).or_refuse(KernelStage::Refine, "make_line")? } else { crate::make_line(to, from).or_refuse(KernelStage::Refine, "make_line")? };
    }
    for shell in &mut solid.shells {
        for existing in &mut shell.faces {
            if existing.id == face.id {
                *existing = rebuilt.clone();
            }
        }
    }
    os_debug!(
        "sharp-side lane: source face {} continued to u = [{ua:.6}, {ub:.6}] from [{u0:.6}, {u1:.6}] for its \
         sharp outward corner(s)",
        face.id
    );
    Ok(Some(solid))
}
