use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// A closed edge sampled as a circle: center, unit axis, radius. `None` when
/// the samples do not lie on a common circle within tolerance.
pub(super) struct RimCircle {
    center: Vec3,
    axis: Vec3,
    radius: f64,
}

pub(super) fn fit_rim_circle(edge: &EdgeRecord, tolerance: f64) -> Result<Option<RimCircle>, KernelRefusal> {
    let samples = 48;
    let mut points = Vec::with_capacity(samples);
    for index in 0..samples {
        let fraction = index as f64 / samples as f64;
        points.push(
            edge.curve
                .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?,
        );
    }
    let count = points.len() as f64;
    let center = points
        .iter()
        .fold(Vec3::default(), |sum, point| sum.add(*point))
        .scale(1.0 / count);
    // Plane normal from the sample covariance's smallest-spread direction,
    // approximated by the average of consecutive radial cross products.
    let mut axis = Vec3::default();
    for index in 0..points.len() {
        let a = points[index].sub(center);
        let b = points[(index + 1) % points.len()].sub(center);
        axis = axis.add(a.cross(b));
    }
    // A degenerate closed edge (pole placeholder) has all samples coincident:
    // the swept-area axis is zero. That is "not a circle", not a hard error.
    let Ok(axis) = axis.normalized() else {
        return Ok(None);
    };
    let radius = points
        .iter()
        .map(|point| point.sub(center).length())
        .sum::<f64>()
        / count;
    if radius <= tolerance {
        return Ok(None);
    }
    for point in &points {
        let radial = point.sub(center);
        if radial.dot(axis).abs() > tolerance.max(radius * 1e-3)
            || (radial.length() - radius).abs() > tolerance.max(radius * 1e-3)
        {
            return Ok(None);
        }
    }
    Ok(Some(RimCircle {
        center,
        axis,
        radius,
    }))
}

/// Whether a coedge runs along its edge, read from geometry.
///
/// The `forward` flag for a coedge is derived, not assumed: a
/// coedge whose `pcurve` (mapped through the surface) traces its edge's 3D
/// curve in the SAME direction is `forward = true`, otherwise `false`. When a
/// pcurve's direction is pinned by loop connectivity (a full-circle rim isoline
/// can run either u-way), the flag must follow the resulting 3D walk or
/// `validate()` flags the coedge as pcurve-inconsistent with its edge.
pub(super) fn isoline_forward(
    pcurve: &crate::NurbsCurve,
    surface: &crate::NurbsSurface,
    curve: &crate::NurbsCurve,
) -> Result<bool, KernelRefusal> {
    let mut deviation_forward = 0.0f64;
    let mut deviation_reversed = 0.0f64;
    for sample in 0..=32 {
        let fraction = sample as f64 / 32.0;
        let uv = pcurve.evaluate(fraction).or_refuse(KernelStage::Sew, "evaluate")?;
        let on_surface = surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate")?;
        deviation_forward =
            deviation_forward.max(on_surface.sub(curve.evaluate(fraction).or_refuse(KernelStage::Sew, "evaluate")?).length());
        deviation_reversed =
            deviation_reversed.max(on_surface.sub(curve.evaluate(1.0 - fraction).or_refuse(KernelStage::Sew, "evaluate")?).length());
    }
    Ok(deviation_forward <= deviation_reversed)
}

/// Plane frame of a geometrically planar surface: `(origin, unit normal)`,
/// or `None` when the sampled grid leaves one plane by more than `tolerance`.
/// Works for planes in any representation — structural `is_affine` misses
/// planes represented as surfaces of revolution (a revolved radial line is a
/// perfectly flat disk but rational degree (2,1)), which is exactly how
/// revolve-built opening caps arrive here.
pub(super) fn planar_surface_frame(
    surface: &crate::NurbsSurface,
    tolerance: f64,
) -> Result<Option<(Vec3, Vec3)>, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
    let origin = surface.evaluate((u0 + u1) * 0.5, (v0 + v1) * 0.5).or_refuse(KernelStage::Sew, "evaluate")?;
    let mut points = Vec::new();
    for iu in 0..=6 {
        for iv in 0..=6 {
            let u = u0 + (u1 - u0) * iu as f64 / 6.0;
            let v = v0 + (v1 - v0) * iv as f64 / 6.0;
            points.push(surface.evaluate(u, v).or_refuse(KernelStage::Sew, "evaluate")?);
        }
    }
    // A summed cross product cancels on rotationally sampled grids (the
    // per-pair crosses alternate sign about the axis); take the single
    // largest-magnitude cross as the normal estimate instead.
    let mut normal = Vec3::default();
    let mut best = 0.0f64;
    for first in &points {
        for second in &points {
            let cross = first.sub(origin).cross(second.sub(origin));
            let magnitude = cross.length();
            if magnitude > best {
                best = magnitude;
                normal = cross;
            }
        }
    }
    let Ok(normal) = normal.normalized() else {
        return Ok(None);
    };
    if points
        .iter()
        .all(|point| point.sub(origin).dot(normal).abs() <= tolerance)
    {
        Ok(Some((origin, normal)))
    } else {
        Ok(None)
    }
}

/// Geometric planarity: whether every sampled surface point lies on one plane
/// within `tolerance`.
pub(super) fn surface_is_planar(surface: &crate::NurbsSurface, tolerance: f64) -> Result<bool, KernelRefusal> {
    if surface.is_affine().or_refuse(KernelStage::Sew, "is_affine")? {
        return Ok(true);
    }
    Ok(planar_surface_frame(surface, tolerance)?.is_some())
}

/// Weld PAIRS of coaxial, coplanar orphan rims into a NEW planar annulus face.
///
/// A through-hole shelled at both of its openings leaves the retained hole
/// wall and its offset image each ending on a dangling rim in every opening
/// plane. The single-rim strategy does not apply there: no wall fragment covers
/// the hole's surroundings (the wall carriers only produce the opening's
/// border frame), so there is no containing planar face to hole
/// (`weld_coplanar_orphan_rims`). The missing geometry is exactly the flat
/// annulus between the two rims — the through-hole wall's exposed thickness,
/// which lies IN the opening plane, as every closure here does since the ruled
/// cork was retired on 2026-09-17.
///
/// Pairing: two one-use closed rims qualify when they are circles on a common
/// axis, lie in a common plane (an out-of-plane mate would need a twisted
/// band — refused, honest), and do not already bound a common face (the two
/// ends of one tube must never be capped against each other). Ambiguity is
/// resolved smallest-radial-gap-first, which picks each rim's true wall mate
/// (the source/offset images differ by exactly the shell thickness).
///
/// Orientation: each rim is traced opposite to its single existing use. The
/// two owning components were assembled independently, so their senses can
/// disagree — detected as the two loops winding the SAME way in the annulus
/// plane, which would corrupt the trim, not just the orientation. When the
/// owners live in different shell components the inner rim's whole component
/// is flipped (legal: a floating component's global sense is arbitrary until
/// first joined); a same-component winding clash is a genuine inconsistency
/// and skips the pair, leaving the honesty gate to refuse the shell.
pub(super) fn weld_coplanar_rim_pair_annuli(
    solid: &mut BrepSolid,
    face_images: &mut Vec<OffsetShellFaceImageRecord>,
    tolerance: f64,
) -> Result<usize, KernelRefusal> {
    let use_counts = crate::topology::edge_use_counts(solid);
    let mut orphans = Vec::new();
    for edge in &solid.edges {
        if use_counts.get(&edge.id).copied().unwrap_or(0) != 1
            || edge.start_vertex_id != edge.end_vertex_id
        {
            continue;
        }
        if let Some(circle) = fit_rim_circle(edge, tolerance)? {
            orphans.push((edge.clone(), circle));
        }
    }
    if orphans.len() < 2 {
        return Ok(0);
    }
    let find_use = |solid: &BrepSolid, edge_id: u64| -> Option<(usize, usize, bool)> {
        solid
            .shells
            .iter()
            .enumerate()
            .find_map(|(shell_index, shell)| {
                shell
                    .faces
                    .iter()
                    .enumerate()
                    .find_map(|(face_index, face)| {
                        face.loops
                            .iter()
                            .flat_map(|loop_record| &loop_record.coedges)
                            .find(|coedge| coedge.edge_id == edge_id)
                            .map(|coedge| (shell_index, face_index, coedge.forward))
                    })
            })
    };
    let coplanar_band = tolerance.max(1e-4);
    let mut candidates = Vec::new();
    for first in 0..orphans.len() {
        for second in first + 1..orphans.len() {
            let (first_edge, first_circle) = &orphans[first];
            let (second_edge, second_circle) = &orphans[second];
            if first_circle.axis.dot(second_circle.axis).abs() < 0.999 {
                continue;
            }
            let between = second_circle.center.sub(first_circle.center);
            let axial = between.dot(first_circle.axis);
            if between.sub(first_circle.axis.scale(axial)).length() > coplanar_band
                || axial.abs() > coplanar_band
            {
                continue;
            }
            let radial_gap = (first_circle.radius - second_circle.radius).abs();
            if radial_gap <= coplanar_band {
                continue;
            }
            // Rims that already bound one face are that face's two ends, not
            // a wall pair (capping them against each other would close the
            // tube's bore with a phantom membrane).
            let share_face = solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .any(|face| {
                    let mut uses_first = false;
                    let mut uses_second = false;
                    for coedge in face
                        .loops
                        .iter()
                        .flat_map(|loop_record| &loop_record.coedges)
                    {
                        uses_first |= coedge.edge_id == first_edge.id;
                        uses_second |= coedge.edge_id == second_edge.id;
                    }
                    uses_first && uses_second
                });
            if share_face {
                continue;
            }
            candidates.push((radial_gap, first, second));
        }
    }
    candidates.sort_by(|a, b| {
        a.0.total_cmp(&b.0)
            .then_with(|| orphans[a.1].0.id.cmp(&orphans[b.1].0.id))
            .then_with(|| orphans[a.2].0.id.cmp(&orphans[b.2].0.id))
    });
    let mut consumed = HashSet::<u64>::default();
    let mut shell_parent = (0..solid.shells.len()).collect::<Vec<_>>();
    fn shell_root(parent: &mut [usize], index: usize) -> usize {
        if parent[index] != index {
            parent[index] = shell_root(parent, parent[index]);
        }
        parent[index]
    }
    let mut welded = 0usize;
    let mut shell_unions: Vec<(usize, usize)> = Vec::new();
    for (_, first, second) in candidates {
        let (first_edge, first_circle) = &orphans[first];
        let (second_edge, second_circle) = &orphans[second];
        if consumed.contains(&first_edge.id) || consumed.contains(&second_edge.id) {
            continue;
        }
        // Outer = larger radius; the annulus is the region between them.
        let (outer_edge, outer_circle, inner_edge, _inner_circle) =
            if first_circle.radius >= second_circle.radius {
                (first_edge, first_circle, second_edge, second_circle)
            } else {
                (second_edge, second_circle, first_edge, first_circle)
            };
        let Some((outer_shell, outer_owner_face, outer_forward)) = find_use(solid, outer_edge.id)
        else {
            continue;
        };
        let Some((inner_shell, inner_owner_face, inner_forward)) = find_use(solid, inner_edge.id)
        else {
            continue;
        };
        // Provenance for the new annulus: the wall thickness it exposes
        // belongs to the outer rim owner's source face (for a through-hole
        // that is the hole wall whose offset produced the outer rim).
        let annulus_source_face_id = face_images
            .get(
                solid.shells[outer_shell].faces[outer_owner_face]
                    .id
                    .saturating_sub(1) as usize,
            )
            .map(|image| image.source_face_id)
            .ok_or_else(|| "offset_shell: rim pair owner lost provenance".to_string()).or_refuse(KernelStage::Sew, "offset_shell_rim_pair_owner_lost_provenance")?;
        // Exactness upgrade with full fallback: swap each fitted-polyline rim
        // for its owner surface's exact isoline where possible, so the
        // annulus trims (and the shell's volume) are exact, not chord-sagged.
        let outer_id = outer_edge.id;
        let inner_id = inner_edge.id;
        let (outer_edge, outer_forward) = match upgrade_rim_to_owner_isoline(
            solid,
            outer_id,
            outer_shell,
            outer_owner_face,
            tolerance,
        )? {
            Some((updated, forward_new)) => (updated, forward_new),
            None => (outer_edge.clone(), outer_forward),
        };
        let (inner_edge, inner_forward) = match upgrade_rim_to_owner_isoline(
            solid,
            inner_id,
            inner_shell,
            inner_owner_face,
            tolerance,
        )? {
            Some((updated, forward_new)) => (updated, forward_new),
            None => (inner_edge.clone(), inner_forward),
        };
        let (outer_edge, inner_edge) = (&outer_edge, &inner_edge);
        let axis = outer_circle.axis;
        let Ok(u_direction) = axis.perpendicular() else {
            continue;
        };
        let v_direction = axis.cross(u_direction).normalized().or_refuse(KernelStage::Sew, "normalized")?;
        let extent = outer_circle.radius * 3.0;
        let plane = crate::make_plane(
            outer_circle
                .center
                .sub(u_direction.scale(extent * 0.5))
                .sub(v_direction.scale(extent * 0.5)),
            u_direction,
            v_direction,
            extent,
            extent,
        ).or_refuse(KernelStage::Sew, "make_plane")?;
        // Manifold rule: the annulus traces each rim opposite its single use.
        let Some(outer_pcurve) = boundary_pcurve(outer_edge, !outer_forward, &plane, tolerance)?
        else {
            continue;
        };
        let Some(mut inner_pcurve) =
            boundary_pcurve(inner_edge, !inner_forward, &plane, tolerance)?
        else {
            continue;
        };
        let mut next_coedge_id = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
            .map(|coedge| coedge.id)
            .max()
            .unwrap_or(0)
            + 1;
        let loop_area = |pcurve: &crate::NurbsCurve, edge_id: u64, forward: bool| {
            parameter_space_area(&FaceRecord {
                id: 0,
                surface: plane.clone(),
                same_sense: true,
                loops: vec![LoopRecord {
                    id: 1,
                    coedges: vec![CoedgeRecord {
                        id: 1,
                        edge_id,
                        forward,
                        pcurve: pcurve.clone(),
                    }],
                }],
                name: None,
            })
        };
        let outer_area = loop_area(&outer_pcurve, outer_edge.id, !outer_forward).or_refuse(KernelStage::Sew, "loop_area")?;
        let mut inner_area = loop_area(&inner_pcurve, inner_edge.id, !inner_forward).or_refuse(KernelStage::Sew, "loop_area")?;
        let mut inner_wall_forward = !inner_forward;
        if outer_area.signum() == inner_area.signum() {
            // The two owning components disagree about their global sense.
            // A floating inner component may be flipped wholesale; a shared
            // component means the shell is genuinely inconsistent — refuse.
            let outer_root = shell_root(&mut shell_parent, outer_shell);
            let inner_root = shell_root(&mut shell_parent, inner_shell);
            if outer_root == inner_root {
                os_debug!(
                    "PAIR skip: rims {} and {} wind the same way inside one component",
                    outer_edge.id,
                    inner_edge.id
                );
                continue;
            }
            let flip_shells = (0..solid.shells.len())
                .filter(|index| shell_root(&mut shell_parent, *index) == inner_root)
                .collect::<Vec<_>>();
            for shell_index in flip_shells {
                flip_shell_faces(&mut solid.shells[shell_index])?;
            }
            inner_wall_forward = inner_forward;
            inner_pcurve = boundary_pcurve(inner_edge, inner_wall_forward, &plane, tolerance)?
                .ok_or_else(|| "offset_shell: annulus inner rim left its plane".to_string()).or_refuse(KernelStage::Sew, "offset_shell_annulus_inner_rim_left_its")?;
            inner_area = loop_area(&inner_pcurve, inner_edge.id, inner_wall_forward).or_refuse(KernelStage::Sew, "loop_area")?;
            if outer_area.signum() == inner_area.signum() {
                os_debug!("PAIR skip: inner-component flip did not fix the winding");
                continue;
            }
        }
        let mut face = FaceRecord {
            id: solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .map(|face| face.id)
                .max()
                .unwrap_or(0)
                + 1,
            surface: plane,
            same_sense: outer_area + inner_area > 0.0,
            loops: Vec::new(),
            name: None,
        };
        let next_loop_id = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .map(|loop_record| loop_record.id)
            .max()
            .unwrap_or(0)
            + 1;
        for (loop_offset, (edge, forward, pcurve)) in [
            (outer_edge, !outer_forward, outer_pcurve),
            (inner_edge, inner_wall_forward, inner_pcurve),
        ]
        .into_iter()
        .enumerate()
        {
            face.loops.push(LoopRecord {
                id: next_loop_id + loop_offset as u64,
                coedges: vec![CoedgeRecord {
                    id: next_coedge_id,
                    edge_id: edge.id,
                    forward,
                    pcurve,
                }],
            });
            next_coedge_id += 1;
        }
        solid.shells[outer_shell].faces.push(face);
        face_images.push(OffsetShellFaceImageRecord {
            role: OffsetFaceRole::Wall,
            source_face_id: annulus_source_face_id,
        });
        for edge_id in [outer_edge.id, inner_edge.id] {
            if let Some(record) = solid.edges.iter_mut().find(|record| record.id == edge_id) {
                record.degenerate = false;
            }
        }
        if outer_shell != inner_shell {
            let outer_root = shell_root(&mut shell_parent, outer_shell);
            let inner_root = shell_root(&mut shell_parent, inner_shell);
            if outer_root != inner_root {
                shell_parent[inner_root] = outer_root;
            }
            shell_unions.push((outer_shell, inner_shell));
        }
        consumed.insert(outer_edge.id);
        consumed.insert(inner_edge.id);
        welded += 1;
    }
    merge_connected_shells(solid, shell_unions);
    if welded > 0 {
        orient_open_solid_faces(solid)?;
    }
    Ok(welded)
}

/// Give every opening-wall face that is a BORE'S END RING the bore wall's
/// provenance, whichever construction built it.
///
/// A through-bore that meets an opening exposes the wall thickness between
/// the hole wall and its offset as a planar ring in the opening's plane. Two
/// constructions produce that ring: the opening carrier's own fragmentation
/// (a wall fragment, provenance the OPENING face) and
/// [`weld_coplanar_rim_pair_annuli`] (provenance the hole wall). Which one
/// runs depends on whether the rims arrived exact enough for the fragmenter
/// to close the ring — so the same face was `Pin_S_1` from one path and
/// `Box_NY_1` from the other, and a document that referenced the ring by
/// name (the 2026-08-24 `BadBoolean` extrude) broke when the offset carrier
/// started building exact rims (2026-09-06).
///
/// The bore's provenance is the one kept: the ring is determined by the bore
/// alone (there is exactly one per bore end), where "the n-th piece of the
/// opening wall" depends on encounter order. A ring is a bore's end ring when
/// it is a two-loop wall whose neighbours are all images of ONE source face
/// and whose INNER rim is that face's source image — the material lies
/// outside the hole. A can shelled through its lid has its source rim
/// OUTSIDE (the offset rim inside) and keeps the opening's name, as before.
///
/// Runs on the finished solid, after the merge remaps `face_images` into
/// face iteration order. Returns how many rings changed provenance.
pub(super) fn claim_bore_end_rings(
    solid: &BrepSolid,
    face_images: &mut [OffsetShellFaceImageRecord],
) -> Result<usize, KernelRefusal> {
    let faces = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .collect::<Vec<_>>();
    if faces.len() != face_images.len() {
        return Err(KernelRefusal::internal(KernelStage::Sew, "offset_shell_provenance_count", format!(
            "offset_shell: {} faces but {} provenance records",
            faces.len(),
            face_images.len()
        )));
    }
    let mut users = HashMap::<u64, Vec<usize>>::default();
    for (index, face) in faces.iter().enumerate() {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            users.entry(coedge.edge_id).or_default().push(index);
        }
    }
    let mut claimed = 0usize;
    for (index, face) in faces.iter().enumerate() {
        if !matches!(face_images[index].role, OffsetFaceRole::Wall) || face.loops.len() != 2 {
            continue;
        }
        let mut areas = [0.0f64; 2];
        for (slot, loop_record) in face.loops.iter().enumerate() {
            areas[slot] = parameter_space_area(&FaceRecord {
                id: 0,
                surface: face.surface.clone(),
                same_sense: true,
                loops: vec![loop_record.clone()],
                name: None,
            }).or_refuse(KernelStage::Sew, "parameter_space_area")?
            .abs();
        }
        let inner = usize::from(areas[1] < areas[0]);
        let mut source: Option<u64> = None;
        let mut inner_is_source = true;
        let mut outer_is_offset = true;
        let mut neighbours = 0usize;
        let mut consistent = true;
        for (slot, loop_record) in face.loops.iter().enumerate() {
            for coedge in &loop_record.coedges {
                for &other in users.get(&coedge.edge_id).map(Vec::as_slice).unwrap_or(&[]) {
                    if other == index {
                        continue;
                    }
                    neighbours += 1;
                    let image = &face_images[other];
                    match source {
                        None => source = Some(image.source_face_id),
                        Some(known) if known == image.source_face_id => {}
                        Some(_) => consistent = false,
                    }
                    let is_source = matches!(image.role, OffsetFaceRole::Source);
                    let is_offset = matches!(image.role, OffsetFaceRole::Offset);
                    if slot == inner {
                        inner_is_source &= is_source;
                    } else {
                        outer_is_offset &= is_offset;
                    }
                }
            }
        }
        let Some(source) = source else { continue };
        if !consistent
            || neighbours == 0
            || !inner_is_source
            || !outer_is_offset
            || face_images[index].source_face_id == source
        {
            continue;
        }
        face_images[index].source_face_id = source;
        claimed += 1;
    }
    Ok(claimed)
}

/// One opening rim as the assembled solid carries it: the source rim edge (used
/// by its source face and by the mis-built opening cap) and its IMAGE on the
/// offset carrier (used once, by the offset skin), with the uv curve both were
/// built from.
struct RimBandPair {
    rim: u64,
    image: u64,
    cap_face: u64,
    carrier: usize,
    pcurve: crate::NurbsCurve,
}

/// What [`weld_free_form_rim_bands`] measured about the bands it built.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct RimBandResiduals {
    /// Worst distance from a band corner to the vertex its edges meet at.
    pub endpoint: f64,
    /// Worst `max_t ‖C_3d(t) − S_band(p(t))‖` over the band's four edges.
    pub edge_to_carrier: f64,
    /// The bar both were held to: twice the intersection-fit contract, since
    /// each band rail and the edge it meets were each fitted to that contract
    /// against the same locus.
    pub bar: f64,
}

/// What [`weld_free_form_rim_bands`] did: how many bands it built, or — when it
/// found an opening cap it could not replace — why. What the bands measured is
/// gated inside the weld and traced through `BREP_OS_DEBUG`; the fixtures
/// re-measure the delivered solid rather than trusting a number passed out.
#[derive(Clone, Debug, Default)]
pub(super) struct RimBandOutcome {
    pub welded: usize,
    /// Set only when the lane RECOGNIZED its configuration — an opening kept
    /// whole as a cap bounded by source rims, with offset edges dangling — and
    /// could not close it. The shell still fails where it failed before; this
    /// is the name its refusal carries.
    pub decline: Option<String>,
}

impl RimBandOutcome {
    /// A decline is only named when an opening cap bounded by nothing but
    /// source rims is there to be replaced; otherwise the dangling edges are
    /// some other lane's, and this one says nothing.
    fn declined(reason: String, solid: &BrepSolid, face_images: &[OffsetShellFaceImageRecord]) -> Self {
        let use_counts = crate::topology::edge_use_counts(solid);
        let role = |face_id: u64| face_images.get(face_id.saturating_sub(1) as usize).map(|image| image.role);
        let faces = solid.shells.iter().flat_map(|shell| &shell.faces).collect::<Vec<_>>();
        let whole_cap = faces.iter().any(|face| {
            matches!(role(face.id), Some(OffsetFaceRole::Wall))
                && face.loops.len() == 1
                && face.loops[0].coedges.iter().all(|coedge| {
                    use_counts.get(&coedge.edge_id).copied().unwrap_or(0) == 2
                        && faces.iter().any(|other| {
                            other.id != face.id
                                && matches!(role(other.id), Some(OffsetFaceRole::Source))
                                && other
                                    .loops
                                    .iter()
                                    .flat_map(|loop_record| &loop_record.coedges)
                                    .any(|use_| use_.edge_id == coedge.edge_id)
                        })
                })
        });
        os_debug!("RIMBAND decline (named: {whole_cap}): {reason}");
        Self {
            decline: whole_cap.then_some(reason),
            ..Self::default()
        }
    }
}

/// Where two edge curves coincide geometrically: `Some(reversed)`.
fn same_edge_geometry(first: &EdgeRecord, second: &EdgeRecord, tolerance: f64) -> Result<Option<bool>, KernelRefusal> {
    let ends = |edge: &EdgeRecord| -> Result<[Vec3; 3], KernelRefusal> {
        Ok([
            edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?,
            edge.curve.evaluate((edge.t0 + edge.t1) * 0.5).or_refuse(KernelStage::Sew, "evaluate")?,
            edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?,
        ])
    };
    let [a0, am, a1] = ends(first)?;
    let [b0, bm, b1] = ends(second)?;
    if am.sub(bm).length() > tolerance {
        return Ok(None);
    }
    if a0.sub(b0).length() <= tolerance && a1.sub(b1).length() <= tolerance {
        return Ok(Some(false));
    }
    if a0.sub(b1).length() <= tolerance && a1.sub(b0).length() <= tolerance {
        return Ok(Some(true));
    }
    Ok(None)
}

/// Close an opening whose offset rim image falls SHORT of the opening, for any
/// rim a pcurve can name: a free-form (NURBS) rim, or a non-iso one.
///
/// The configuration the retired coaxial-circle weld closed for circles: a
/// retained face leaning over its opening offsets to a skin that ends at the
/// image of the rim, strictly inside the source, so no imprint against the
/// opening cuts it. The pipeline then keeps the opening face whole as a cap
/// bounded by the source rim, and the image chain dangles one-use. The wall
/// that closes it is the NORMAL-RULED band from each rim point to its offset
/// image — the same wall the frustum weld builds, and the convention
/// `frustum_open_both_ends_shells_watertight_with_exact_wall_volume` asserts.
///
/// Each band is built from the uv curve the rim and its image were both made
/// from. The source face's surface and the offset carrier are first put on one
/// basis (exact: the carrier's refined net is a refinement of the source's),
/// then the curve's images on both are fitted over ONE parameter set, so the
/// two rails share degree, knots and weights and the ruled tensor between them
/// reproduces the segment `(1 − t)·rim(s) + t·image(s)` at every `s`. Adjacent
/// bands share the straight ruling at the rim vertex between them.
///
/// Measured, not assumed: every band corner against its vertex and every band
/// edge against the band surface along its pcurve, each held to twice the
/// intersection-fit contract. A cap this lane recognizes but cannot close is
/// refused by name: a rim corner where two retained faces meet SHARPLY leaves
/// two images that do not meet, and closing that needs a miter this lane does
/// not build.
pub(super) fn weld_free_form_rim_bands(
    solid: &mut BrepSolid,
    face_images: &mut Vec<OffsetShellFaceImageRecord>,
    carriers: &[Carrier],
    source: &BrepSolid,
    tolerance: f64,
) -> Result<RimBandOutcome, KernelRefusal> {
    let mut residuals = RimBandResiduals::default();
    let use_counts = crate::topology::edge_use_counts(solid);
    let one_use = solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate && use_counts.get(&edge.id).copied().unwrap_or(0) == 1)
        .cloned()
        .collect::<Vec<_>>();
    if one_use.is_empty() {
        return Ok(RimBandOutcome::default());
    }
    let role_of = |face_id: u64| {
        face_images
            .get(face_id.saturating_sub(1) as usize)
            .map(|image| image.role)
    };
    let faces_using = |solid: &BrepSolid, edge_id: u64| -> Vec<u64> {
        solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| {
                face.loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .any(|coedge| coedge.edge_id == edge_id)
            })
            .map(|face| face.id)
            .collect()
    };
    let source_face = |face_id: u64| {
        source
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == face_id)
    };

    // Pair every dangling image with the carrier coedge it came from, and that
    // coedge's uv curve with the source rim it traces on the source face.
    let mut pairs: Vec<RimBandPair> = Vec::new();
    for image in &one_use {
        let mut found = None;
        'carriers: for (index, carrier) in carriers.iter().enumerate() {
            if !matches!(carrier.kind, OffsetFaceRole::Offset) {
                continue;
            }
            let carrier_face = &carrier.solid.shells[0].faces[0];
            for coedge in carrier_face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                let Some(carrier_edge) = carrier.solid.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
                    continue;
                };
                if carrier_edge.degenerate || same_edge_geometry(image, carrier_edge, tolerance)?.is_none() {
                    continue;
                }
                found = Some((index, coedge.pcurve.clone()));
                break 'carriers;
            }
        }
        let Some((carrier_index, pcurve)) = found else {
            // The image is a PIECE of a carrier's rim image, and what cut it
            // short is the name of the refusal. An end ON the opening surface
            // means the offset crossed the opening there — the rim's image is
            // inside the source along part of the rim and outside along the
            // rest, which is the MIXED rim. Anything else is the miter two
            // retained faces' offsets cut into each other's images.
            let point = image.curve.evaluate((image.t0 + image.t1) * 0.5).or_refuse(KernelStage::Sew, "evaluate")?;
            let on_opening = [image.t0, image.t1]
                .into_iter()
                .map(|parameter| {
                    let end = image.curve.evaluate(parameter).or_refuse(KernelStage::Sew, "evaluate")?;
                    let mut nearest = f64::INFINITY;
                    for carrier in carriers.iter().filter(|carrier| matches!(carrier.kind, OffsetFaceRole::Wall)) {
                        let opening = &carrier.solid.shells[0].faces[0];
                        nearest = nearest.min(project_point_to_surface(&opening.surface, end).or_refuse(KernelStage::Sew, "project_point_to_surface")?.distance);
                    }
                    Ok(nearest)
                })
                .collect::<Result<Vec<_>, KernelRefusal>>()?
                .into_iter()
                .fold(f64::INFINITY, f64::min);
            let reason = if on_opening <= tolerance {
                format!(
                    "the rim's offset image lies INSIDE the source along part of the rim and outside \
                     along the rest: the dangling piece near ({:.3},{:.3},{:.3}) ends {on_opening:.3e} \
                     from the opening surface, where the image crosses it. The wall there is the \
                     opening where the image is outside and the ruled band where it is inside, joined \
                     at the crossing — a MIXED rim, which this lane does not build",
                    point.x, point.y, point.z
                )
            } else {
                format!(
                    "the dangling offset edge near ({:.3},{:.3},{:.3}) is not a whole rim image of an \
                     offset carrier, and its ends are {on_opening:.3e} from any opening — a SHARP rim \
                     corner, where two retained faces' offsets meet in a miter and cut each other's rim \
                     images short, needs a miter this rim band lane does not build",
                    point.x, point.y, point.z
                )
            };
            return Ok(RimBandOutcome::declined(reason, solid, face_images));
        };
        let Some(face) = source_face(carriers[carrier_index].source_face_id) else {
            return Ok(RimBandOutcome::default());
        };
        let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let at = |q: f64| -> Result<Vec3, KernelRefusal> {
            let uv = pcurve.evaluate(q).or_refuse(KernelStage::Sew, "evaluate")?;
            face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate")
        };
        let (start, middle, end) = (at(q0)?, at((q0 + q1) * 0.5)?, at(q1)?);
        let mut rim = None;
        for edge in &solid.edges {
            if edge.degenerate || use_counts.get(&edge.id).copied().unwrap_or(0) != 2 {
                continue;
            }
            let (a, b) = (edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?, edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?);
            let ends_match = (a.sub(start).length() <= tolerance && b.sub(end).length() <= tolerance)
                || (a.sub(end).length() <= tolerance && b.sub(start).length() <= tolerance);
            if ends_match && crate::project_point_to_curve(&edge.curve, middle).or_refuse(KernelStage::Sew, "project_point_to_curve")?.distance <= tolerance {
                rim = Some(edge.id);
                break;
            }
        }
        let Some(rim) = rim else {
            os_debug!("RIMBAND skip: image {} has no two-use source rim", image.id);
            return Ok(RimBandOutcome::default());
        };
        let users = faces_using(solid, rim);
        let caps = users
            .iter()
            .copied()
            .filter(|face_id| matches!(role_of(*face_id), Some(OffsetFaceRole::Wall)))
            .collect::<Vec<_>>();
        let sources = users
            .iter()
            .filter(|face_id| matches!(role_of(**face_id), Some(OffsetFaceRole::Source)))
            .count();
        if caps.len() != 1 || sources != 1 {
            os_debug!("RIMBAND skip: rim {rim} is not between a source face and one cap");
            return Ok(RimBandOutcome::default());
        }
        pairs.push(RimBandPair {
            rim,
            image: image.id,
            cap_face: caps[0],
            carrier: carrier_index,
            pcurve,
        });
    }

    // Every cap must be bounded by paired rims and nothing else.
    let cap_ids = pairs.iter().map(|pair| pair.cap_face).collect::<HashSet<_>>();
    for cap_id in &cap_ids {
        let cap = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == *cap_id)
            .ok_or_else(|| "offset_shell: rim band cap vanished".to_string()).or_refuse(KernelStage::Sew, "offset_shell_rim_band_cap_vanished")?;
        if cap.loops.len() != 1
            || !cap.loops[0]
                .coedges
                .iter()
                .all(|coedge| pairs.iter().any(|pair| pair.rim == coedge.edge_id))
        {
            os_debug!("RIMBAND skip: cap {cap_id} is not bounded by paired rims alone");
            return Ok(RimBandOutcome::default());
        }
    }

    let vertex_point = |solid: &BrepSolid, id: u64| -> Result<Vec3, KernelRefusal> {
        solid
            .vertices
            .iter()
            .find(|vertex| vertex.id == id)
            .map(|vertex| vertex.point)
            .ok_or_else(|| format!("offset_shell: rim band vertex {id} missing"))
            .or_refuse(KernelStage::Sew, "offset_shell_rim_band_vertex_missing")
    };
    let edge_record = |solid: &BrepSolid, id: u64| -> Result<EdgeRecord, KernelRefusal> {
        solid
            .edges
            .iter()
            .find(|edge| edge.id == id)
            .cloned()
            .ok_or_else(|| format!("offset_shell: rim band edge {id} missing"))
            .or_refuse(KernelStage::Sew, "offset_shell_rim_band_edge_missing")
    };

    // A rim vertex's images from its two faces must be ONE vertex of the chain.
    for pair in &pairs {
        let rim = edge_record(solid, pair.rim)?;
        for vertex in [rim.start_vertex_id, rim.end_vertex_id] {
            let point = vertex_point(solid, vertex)?;
            let neighbours = pairs
                .iter()
                .filter(|other| {
                    solid.edges.iter().any(|edge| {
                        edge.id == other.rim && (edge.start_vertex_id == vertex || edge.end_vertex_id == vertex)
                    })
                })
                .map(|other| other.image)
                .collect::<Vec<_>>();
            let image_vertices = neighbours
                .iter()
                .map(|image| {
                    let edge = edge_record(solid, *image)?;
                    let near = |id: u64| vertex_point(solid, id).map(|p| p.sub(point).length());
                    Ok(if near(edge.start_vertex_id)? <= near(edge.end_vertex_id)? {
                        edge.start_vertex_id
                    } else {
                        edge.end_vertex_id
                    })
                })
                .collect::<Result<Vec<_>, KernelRefusal>>()?;
            if image_vertices.len() == 2 && image_vertices[0] != image_vertices[1] {
                let gap = vertex_point(solid, image_vertices[0])?
                    .sub(vertex_point(solid, image_vertices[1])?)
                    .length();
                return Ok(RimBandOutcome::declined(format!(
                    "the opening rim turns a SHARP corner at ({:.3},{:.3},{:.3}), where the offset \
                     images of its two retained faces do not meet (gap {gap:.3e}); a ruled rim band \
                     cannot close it without a miter, which is not built",
                    point.x, point.y, point.z
                ), solid, face_images));
            }
        }
    }

    let scale = crate::solid_model_scale(source);
    let fit_tolerance = crate::KernelTolerances::for_scale(scale, 1e-7).intersection_fit;
    residuals.bar = 2.0 * fit_tolerance;
    let band_measure = crate::offset_construction_band(scale);
    let mut next_edge_id = solid.edges.iter().map(|edge| edge.id).max().unwrap_or(0) + 1;
    let mut next_coedge_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.id)
        .max()
        .unwrap_or(0)
        + 1;
    let mut rulings: HashMap<(u64, u64), u64> = HashMap::default();
    let mut new_edges: Vec<EdgeRecord> = Vec::new();
    let mut bands: Vec<(u64, FaceRecord)> = Vec::new();
    for pair in &pairs {
        let face = source_face(carriers[pair.carrier].source_face_id)
            .ok_or_else(|| "offset_shell: rim band source face vanished".to_string()).or_refuse(KernelStage::Sew, "offset_shell_rim_band_source_face_vanished")?;
        let carrier_surface = &carriers[pair.carrier].solid.shells[0].faces[0].surface;
        let (rim_sheet, image_sheet) =
            crate::offset::unify_offset_sheet_bases(&face.surface, carrier_surface)?;
        let (rail, image_rail) = crate::image_curve::image_curve_pair(
            &rim_sheet,
            &image_sheet,
            &pair.pcurve,
            fit_tolerance,
            "offsetShell rim band",
        ).or_refuse(KernelStage::Sew, "image_curve_pair")?;
        let shared = rail.curve.degree == image_rail.curve.degree
            && rail.curve.knots.len() == image_rail.curve.knots.len()
            && rail.curve.knots.iter().zip(&image_rail.curve.knots).all(|(a, b)| (a - b).abs() <= 1e-12)
            && rail
                .curve
                .control_points
                .iter()
                .zip(&image_rail.curve.control_points)
                .all(|(a, b)| (a.w - b.w).abs() <= 1e-9)
            && (rail.t0 - image_rail.t0).abs() <= 1e-12
            && (rail.t1 - image_rail.t1).abs() <= 1e-12;
        if !shared {
            return Err(KernelRefusal::internal(KernelStage::Sew, "offset_shell_rim_band_basis", "offset_shell: a rim band's two rails do not share a basis"));
        }
        let band = crate::NurbsSurface::new(
            rail.curve.degree,
            1,
            rail.curve.knots.clone(),
            vec![0.0, 0.0, 1.0, 1.0],
            rail.curve
                .control_points
                .iter()
                .zip(&image_rail.curve.control_points)
                .map(|(a, b)| vec![*a, *b])
                .collect(),
        ).or_refuse(KernelStage::Sew, "new")?;
        let (s_a, s_b) = (rail.t0, rail.t1);
        let rim = edge_record(solid, pair.rim)?;
        let image = edge_record(solid, pair.image)?;
        let nearest = |solid: &BrepSolid, edge: &EdgeRecord, point: Vec3| -> Result<u64, KernelRefusal> {
            let start = vertex_point(solid, edge.start_vertex_id)?.sub(point).length();
            let end = vertex_point(solid, edge.end_vertex_id)?.sub(point).length();
            Ok(if start <= end { edge.start_vertex_id } else { edge.end_vertex_id })
        };
        let rim_a = nearest(solid, &rim, band.evaluate(s_a, 0.0).or_refuse(KernelStage::Sew, "evaluate")?)?;
        let rim_b = nearest(solid, &rim, band.evaluate(s_b, 0.0).or_refuse(KernelStage::Sew, "evaluate")?)?;
        let image_a = nearest(solid, &image, band.evaluate(s_a, 1.0).or_refuse(KernelStage::Sew, "evaluate")?)?;
        let image_b = nearest(solid, &image, band.evaluate(s_b, 1.0).or_refuse(KernelStage::Sew, "evaluate")?)?;
        if rim_a == rim_b || image_a == image_b {
            return Err(KernelRefusal::unsupported(KernelStage::Sew, "offset_shell_rim_band_seam", "offset_shell: a closed rim needs a seam this rim band does not build"));
        }
        for (s, t, vertex) in [(s_a, 0.0, rim_a), (s_b, 0.0, rim_b), (s_a, 1.0, image_a), (s_b, 1.0, image_b)] {
            residuals.endpoint = residuals
                .endpoint
                .max(band.evaluate(s, t).or_refuse(KernelStage::Sew, "evaluate")?.sub(vertex_point(solid, vertex)?).length());
        }
        let mut ruling = |from: u64, to: u64| -> Result<u64, KernelRefusal> {
            if let Some(id) = rulings.get(&(from, to)) {
                return Ok(*id);
            }
            let id = next_edge_id;
            next_edge_id += 1;
            new_edges.push(EdgeRecord {
                id,
                curve: crate::make_line(vertex_point(solid, from)?, vertex_point(solid, to)?).or_refuse(KernelStage::Sew, "make_line")?,
                t0: 0.0,
                t1: 1.0,
                start_vertex_id: from,
                end_vertex_id: to,
                degenerate: false,
                name: None,
            });
            rulings.insert((from, to), id);
            Ok(id)
        };
        let ruling_b = ruling(rim_b, image_b)?;
        let ruling_a = ruling(rim_a, image_a)?;
        let line = |u0: f64, v0: f64, u1: f64, v1: f64| {
            crate::make_line(Vec3::new(u0, v0, 0.0), Vec3::new(u1, v1, 0.0))
        };
        let starts_at = |edge: &EdgeRecord, point: Vec3| -> Result<bool, KernelRefusal> {
            Ok(edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?.sub(point).length()
                <= edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?.sub(point).length())
        };
        let rim_forward = starts_at(&rim, band.evaluate(s_a, 0.0).or_refuse(KernelStage::Sew, "evaluate")?)?;
        let image_forward = starts_at(&image, band.evaluate(s_b, 1.0).or_refuse(KernelStage::Sew, "evaluate")?)?;
        let mut coedge = |edge_id: u64, forward: bool, pcurve: crate::NurbsCurve| {
            let record = CoedgeRecord {
                id: next_coedge_id,
                edge_id,
                forward,
                pcurve,
            };
            next_coedge_id += 1;
            record
        };
        let coedges = vec![
            coedge(rim.id, rim_forward, line(s_a, 0.0, s_b, 0.0).or_refuse(KernelStage::Sew, "line")?),
            coedge(ruling_b, true, line(s_b, 0.0, s_b, 1.0).or_refuse(KernelStage::Sew, "line")?),
            coedge(image.id, image_forward, line(s_b, 1.0, s_a, 1.0).or_refuse(KernelStage::Sew, "line")?),
            coedge(ruling_a, false, line(s_a, 1.0, s_a, 0.0).or_refuse(KernelStage::Sew, "line")?),
        ];
        let mut band_face = FaceRecord {
            id: 0,
            surface: band,
            same_sense: true,
            loops: vec![LoopRecord { id: 1, coedges }],
            name: None,
        };
        if parameter_space_area(&band_face).or_refuse(KernelStage::Sew, "parameter_space_area")? < 0.0 {
            band_face.same_sense = false;
        }
        bands.push((pair.cap_face, band_face));
    }

    // Measure every band edge against the band along its pcurve.
    let all_edges = solid.edges.iter().chain(new_edges.iter()).map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
    for (_, band) in &bands {
        for coedge in &band.loops[0].coedges {
            let edge = all_edges[&coedge.edge_id];
            let measured = crate::measure_edge_against_pcurve_image(
                &band.surface,
                &coedge.pcurve,
                edge,
                coedge.forward,
                band_measure,
            )?;
            residuals.edge_to_carrier = residuals.edge_to_carrier.max(measured.deviation());
        }
    }
    os_debug!(
        "RIMBAND {} band(s): endpoint {:.3e}, edge-to-carrier {:.3e}, bar {:.3e}",
        bands.len(),
        residuals.endpoint,
        residuals.edge_to_carrier,
        residuals.bar
    );
    if residuals.endpoint > residuals.bar || residuals.edge_to_carrier > residuals.bar {
        return Err(KernelRefusal::unsupported(KernelStage::Sew, "offset_shell_rim_band_miss", format!(
            "offset_shell: a ruled rim band misses its edges (endpoint {:.3e}, edge-to-band {:.3e}) \
             past twice the intersection-fit contract {:.3e}",
            residuals.endpoint, residuals.edge_to_carrier, residuals.bar
        )));
    }

    // Commit: each cap gives way to its bands, in the cap's shell, and the shells
    // the rims and images belong to join it.
    let shell_of = |solid: &BrepSolid, face_id: u64| {
        solid
            .shells
            .iter()
            .position(|shell| shell.faces.iter().any(|face| face.id == face_id))
    };
    let mut unions = Vec::new();
    for pair in &pairs {
        let cap_shell = shell_of(solid, pair.cap_face).ok_or_else(|| "offset_shell: cap shell missing".to_string()).or_refuse(KernelStage::Sew, "offset_shell_cap_shell_missing")?;
        for edge_id in [pair.rim, pair.image] {
            for face_id in faces_using(solid, edge_id) {
                if let Some(shell) = shell_of(solid, face_id) {
                    if shell != cap_shell {
                        unions.push((cap_shell, shell));
                    }
                }
            }
        }
    }
    let welded = bands.len();
    for (cap_id, mut band) in bands {
        let cap_shell = shell_of(solid, cap_id).ok_or_else(|| "offset_shell: cap shell missing".to_string()).or_refuse(KernelStage::Sew, "offset_shell_cap_shell_missing")?;
        let cap_source = face_images
            .get(cap_id.saturating_sub(1) as usize)
            .map(|image| image.source_face_id)
            .ok_or_else(|| "offset_shell: cap provenance missing".to_string()).or_refuse(KernelStage::Sew, "offset_shell_cap_provenance_missing")?;
        band.id = face_images.len() as u64 + 1;
        face_images.push(OffsetShellFaceImageRecord {
            role: OffsetFaceRole::Wall,
            source_face_id: cap_source,
        });
        solid.shells[cap_shell].faces.push(band);
    }
    for shell in &mut solid.shells {
        shell.faces.retain(|face| !cap_ids.contains(&face.id));
    }
    solid.edges.extend(new_edges);
    merge_connected_shells(solid, unions);
    orient_open_solid_faces(solid)?;
    Ok(RimBandOutcome {
        welded,
        decline: None,
    })
}
