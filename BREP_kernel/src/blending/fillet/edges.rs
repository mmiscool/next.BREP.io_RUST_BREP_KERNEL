use super::*;

/// Constant-radius rolling-ball fillet of one edge (Golovanov §4.9 march,
/// §6.9 surgery; the cutter only where the march refuses).
pub fn fillet_edge(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    check_mixed_concavity(solid, edge_id, "fillet_edge")?;
    check_support_extent(solid, edge_id, radius, "fillet_edge")?;
    fillet_or_chamfer(solid, edge_id, radius, false, name, ToolEnds::default(), Lane::GeneralFirst)
}

/// `fillet_edge` with the cutter first: the input the corner-closure lanes
/// (`round_convex_corner`, the mixed-convexity closures) were written
/// against.  Their tests build it directly; production reaches those lanes
/// only through the group's sequential composition, which is cutter-first
/// for the same reason.
pub(crate) fn fillet_edge_cutter(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    check_mixed_concavity(solid, edge_id, "fillet_edge")?;
    check_support_extent(solid, edge_id, radius, "fillet_edge")?;
    fillet_or_chamfer(solid, edge_id, radius, false, name, ToolEnds::default(), Lane::CutterFirst)
        .map(settle_cutter_scaffold)
}

/// Equal-leg chamfer of one edge (Golovanov §6.11: the same construction
/// with the arc's chord).
pub fn chamfer_edge(
    solid: &BrepSolid,
    edge_id: u64,
    distance: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    check_mixed_concavity(solid, edge_id, "chamfer_edge")?;
    check_support_extent(solid, edge_id, distance, "chamfer_edge")?;
    fillet_or_chamfer(solid, edge_id, distance, true, name, ToolEnds::default(), Lane::GeneralFirst)
}

/// Build and apply the chamfer tool for one straight edge from an already-built
/// cross-section profile (curve index 1 is the chamfer chord / blend wall).
/// Shared by the two-distance and distance-angle asymmetric entries.
fn apply_chamfer_offsets_profile(
    solid: &BrepSolid,
    cross: &EdgeCross,
    profile: &[NurbsCurve],
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut tool = match &cross.path {
        EdgePath::Straight { direction, length } => {
            extrude_profile_brep(profile, *direction, *length)?
        }
        EdgePath::Circular { .. } => {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "asymmetric_curved_edge",
                "chamfer_edge_asymmetric: only straight edges on planar faces are supported \
                 in this slice (asymmetric chamfer on general/curved edges is out of scope)",
            ));
        }
    };
    // Side faces are emitted in input-curve order; the chamfer wall is the
    // second profile curve (index 1). Every other face is the cutter's
    // scaffold, tagged and settled exactly as the symmetric cutter's is.
    let mut side_index = 0usize;
    for shell in &mut tool.shells {
        for face in &mut shell.faces {
            if side_index == 1 {
                if face.name.is_none() {
                    face.name = name.map(str::to_string);
                }
            } else {
                face.name = Some(scaffold_tag(name));
            }
            side_index += 1;
        }
    }
    apply_tool(solid, &tool, cross.convex).map(settle_cutter_scaffold)
}

/// Asymmetric (two-distance) chamfer of one STRAIGHT edge between two planar
/// faces (Golovanov §6.11): setback `d1` along face 1 and `d2` along face 2 —
/// the standard CAD "d1 × d2" bevel.  General/curved edges are out of scope for
/// this slice and return a clear error.
pub fn chamfer_edge_asymmetric(
    solid: &BrepSolid,
    edge_id: u64,
    d1: f64,
    d2: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    if !(d1 > 0.0) || !(d2 > 0.0) || !d1.is_finite() || !d2.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "setback_distances",
            "chamfer_edge_asymmetric: both setback distances must be positive",
        ));
    }
    // `analyze_edge` only uses the radius to size the orientation probe step;
    // the smaller setback keeps that probe inside both faces.
    let cross = analyze_edge(solid, edge_id, d1.min(d2))?;
    let profile = chamfer_cross_section_offsets(&cross, d1, d2)?;
    apply_chamfer_offsets_profile(solid, &cross, &profile, name)
}

/// Distance-angle chamfer of one STRAIGHT edge between two planar faces
/// (Golovanov §6.11): setback `d1` along face 1 and angle `angle_rad` between
/// the chamfer face and face 1.  `d2` is constructed geometrically in the
/// cross-section plane (see `chamfer_angle_second_distance`), then the
/// two-distance builder is applied.
pub fn chamfer_edge_angle(
    solid: &BrepSolid,
    edge_id: u64,
    d1: f64,
    angle_rad: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    if !(d1 > 0.0) || !d1.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "setback_d1",
            "chamfer_edge_angle: setback distance d1 must be positive",
        ));
    }
    let cross = analyze_edge(solid, edge_id, d1)?;
    let d2 = chamfer_angle_second_distance(&cross, d1, angle_rad)?;
    let profile = chamfer_cross_section_offsets(&cross, d1, d2)?;
    apply_chamfer_offsets_profile(solid, &cross, &profile, name)
}

/// Snap small trim/intersection gaps between blend edges and their vertices.
/// The boolean endpoint welder preserves gaps already inside the validation
/// band and limits repairs to `max(|radius| * 1e-3, 1e-4)`.
pub(super) fn heal_edge_vertex_gaps(
    solid: &mut BrepSolid,
    radius: f64,
) -> Result<(), KernelRefusal> {
    let search = (radius.abs() * 1e-3).max(1e-7);
    crate::boolean::commit_nearby_edge_endpoints(solid, search)
}

/// Resolve a point within 1e-3 of a trimmed edge.
fn resolve_edge_by_point(solid: &BrepSolid, point: Vec3) -> Result<u64, KernelRefusal> {
    match crate::topology::nearest_edge(solid, point) {
        Some((edge_id, distance)) if distance <= 1e-3 => Ok(edge_id),
        Some((_, distance)) => Err(KernelRefusal::input(
            KernelStage::Collect,
            "edge_point",
            format!(
                "fillet_edges: no edge within tolerance of the point (nearest {distance:.6})"
            ),
        )),
        None => Err(KernelRefusal::input(
            KernelStage::Collect,
            "no_edges",
            "fillet_edges: solid has no edges",
        )),
    }
}

/// Fillet (or chamfer) a GROUP of edges as ONE operation, and — for fillets —
/// round the convex "star" vertices where three or more of the selected edges
/// meet (Golovanov §6.9.7).  This is the whole multi-edge fillet in a single
/// kernel call: the caller passes the object plus one 3D point on each edge,
/// and the kernel orchestrates the filleting and corner blending against the
/// full topology (so acute corners resolve coherently instead of being
/// stitched edge-by-edge by the app).  A corner the kernel cannot round (e.g.
/// non-orthogonal beyond support, or a general no-common-ball star) is left as
/// the edge fillets rather than failing the whole group.
///
/// When the WHOLE selection cannot be blended as one unit — a shared convex
/// corner where a revolve axis/pole edge meets the adjacent cap edges can
/// defeat the sequential corner surgery even though each edge and every proper
/// SUBSET of the selection blends cleanly (the three fillets converge on the
/// pole with no single end face across the corner) — a maximal-valid-SUBSET
/// search blends the largest subset that works, dropping only the edges that
/// cannot co-blend.  That is a real outcome and [`fillet_edges_reported`]
/// returns it, WITH the list of edges it could not blend.
///
/// This entry point does not.  A solid holding eight of the ten walls that
/// were asked for is not the answer to the question that was asked, and
/// returning it as a plain `Ok` reports success for a different result; so an
/// incomplete selection is an error here, naming every edge that was dropped
/// and the group refusal behind them.  A caller that genuinely wants the
/// partial answer asks for it by name.
///
/// `edge_names` (when `Some`) is the per-edge blend-FACE name parallel to
/// `edge_points` — each grown wall is named after ITS originating edge; `None`
/// names every wall with the single base `name` (the legacy/test behavior,
/// byte-identical to before). The whole `*_edges` family takes the same
/// `edge_names` slot in the same position.
pub fn fillet_edges(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    let entry = if chamfer { "chamfer_edges" } else { "fillet_edges" };
    let (result, selection) =
        fillet_edges_reported(solid, edge_points, edge_names, radius, chamfer, name)?;
    if selection.is_complete() {
        return Ok(result);
    }
    // Every dropped edge BY NAME: the caller's own per-edge name when it gave
    // one, and always its index and the point it was selected by.
    let dropped: Vec<String> = selection
        .rejected
        .iter()
        .map(|index| {
            let point = edge_points[*index];
            let named = edge_names
                .and_then(|names| names.get(*index))
                .map(|name| format!("`{name}` "))
                .unwrap_or_default();
            format!(
                "{named}#{index} at ({:.6}, {:.6}, {:.6})",
                point.x, point.y, point.z
            )
        })
        .collect();
    // The selection, as specified, has no single answer the kernel will
    // stand behind: the group refusal behind the drop travels as TEXT in
    // `BlendSelection::reason`, so this refusal carries its own class.
    Err(KernelRefusal::ill_posed(
        KernelStage::Select,
        "incomplete_selection",
        format!(
            "{entry}: {} of the {} selected edges were blended — {} could not co-blend with the \
             rest of the selection and carry no wall in the result ({}). The group refusal was: \
             {}. Blend them separately, or call the reported entry point to accept the partial \
             selection deliberately.",
            selection.applied.len(),
            selection.requested,
            selection.rejected.len(),
            dropped.join(", "),
            selection
                .reason
                .as_deref()
                .unwrap_or("the whole group built but not every edge grew a wall"),
        ),
    ))
}

/// What a blend selection actually achieved.
///
/// The kernel can answer a group blend with fewer walls than were asked for —
/// the maximal-valid-subset search drops the edges that cannot co-blend at a
/// shared corner rather than failing outright. This says WHICH, so partial
/// fulfilment is a statement instead of a silence.
#[derive(Clone, Debug)]
pub struct BlendSelection {
    /// How many edges the caller selected.
    pub requested: usize,
    /// Indices into that selection that carry a blend wall in the result.
    pub applied: Vec<usize>,
    /// Indices that do not.
    pub rejected: Vec<usize>,
    /// Why the whole selection could not be built, when something was dropped.
    pub reason: Option<String>,
}

impl BlendSelection {
    /// Every requested edge carries a wall.
    pub fn is_complete(&self) -> bool {
        self.rejected.is_empty()
    }
}

/// [`fillet_edges`] with the selection outcome reported rather than refused.
///
/// The result is the same solid `fillet_edges` would build; the difference is
/// that an incomplete selection comes back as `Ok` with the misses named,
/// which is the explicit partial-result contract.  Every named refusal that
/// applies to the WHOLE selection — mixed concavity, a radius the local wall
/// cannot hold, a mixed-convexity corner — is still an `Err`: dropping edges
/// cannot rescue those, so a partial answer would be a different defect.
pub fn fillet_edges_reported(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<(BrepSolid, BlendSelection), KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    if !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "radius",
            "fillet_edges: radius must be positive",
        ));
    }
    if edge_points.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "selection",
            "fillet_edges: no edges selected",
        ));
    }
    // The rolling ball must reach both supports of every SELECTED edge — see
    // `check_support_extent`.  Checked ONCE, here, against the solid the
    // selection was made on: inside the group build the faces have already
    // been eaten into by earlier blends of the same selection, and a fillet
    // legitimately runs off that remainder (two rounds pinching on a shared
    // face).  Dropping edges cannot rescue an oversized radius either, so
    // this runs before the subset search rather than inside it.
    let entry = if chamfer { "chamfer_edges" } else { "fillet_edges" };
    let mut selected_ids = Vec::with_capacity(edge_points.len());
    for point in edge_points {
        let edge_id = resolve_edge_by_point(solid, *point)?;
        check_mixed_concavity(solid, edge_id, entry)?;
        check_support_extent(solid, edge_id, radius, entry)?;
        selected_ids.push(edge_id);
    }
    // A corner where the selection mixes convexity — a convex edge dying into
    // the concave edges it meets — has no rolling-ball closure at all: the
    // blends run out against each other partway along, which is the vertex
    // blend the stripe network does not construct yet.  It is TERMINAL, and
    // for the same reason as the two checks above it runs once, here: neither
    // the cutter composition nor the subset search below can rescue it.  The
    // cutter "succeeds" on this shape by shredding the solid (a sliver end cap
    // per corner, the carrier face split and renamed), which is worse than
    // the named refusal — see the 2026-09-09 rib-spine report.
    //
    // Nor can PARTITIONING the selection at the corner rescue it, which is why
    // this is still a refusal and not a split (measured 2026-09-13, the
    // partition record): the two convexity-pure halves each build on the
    // network, and composing them walls every selected edge and leaves every
    // other face's area identical to twelve digits — but the convex wall comes
    // out as the full untrimmed quarter-round prism (2·(1 − π/4)·r²·L to
    // 2.2e-10, with NO rolled-corner term) because its contact rail runs off
    // its own face at the TRI-TANGENT STATION and the surgery splices the
    // overhang in rather than clipping it. The faces it runs along come back
    // with self-crossing loops that `validate()`, `solid_self_intersections`,
    // the connectivity report and the Euler check all pass — and that
    // `check_loop_self_crossings` REFUSES since 9e50f6d55, so the split is not
    // a route to ten walls either: blend the concave half first and the
    // acceptance refuses every convex edge whose wall runs out into it — on
    // the rib both spines, at that same station, so that feature fails
    // outright; blend the convex half first and the maximal-valid-subset
    // search drops one ring edge and returns NINE walls of ten. The
    // construction that is missing is the runout itself: past that station the
    // ball rolls on one input face and on the concave stripe's own surface, so
    // the march needs a STRIPE as its second carrier.
    //
    // And more than that, measured 2026-09-13 (`blend/runout.rs`, which this
    // refusal now reports): the KEPT mate runs out too, at the mirror-image
    // station, so the FIRST carrier becomes the other concave stripe and the
    // last stretch rolls on two stripes; that stretch ends in a POLE where the
    // two stripes' rails on the face they share meet and the section shrinks
    // to a point. Two carrier switches and a degenerate terminus, not a march
    // with a stop.
    if !chamfer {
        // The class is BUILT where the runout composition applies — the ring
        // first, then one wall per spine from pole to pole (`blend/runout.rs`,
        // the 2026-09-30 record §5) — and refused by name, with the
        // composition's own reason appended, where it does not.
        let corners = crate::blend::mixed_convexity_corners(solid, &selected_ids, radius);
        if !corners.is_empty() {
            let composed = rounded_boss_composition(
                solid, &selected_ids, edge_points, edge_names, radius, name, &corners,
            ).or_else(|_| runout_composition(
                solid, &selected_ids, edge_points, edge_names, radius, name, &corners, entry,
            ));
            match composed {
                Ok(result) => {
                    let missing = walls_missing(
                        solid, &result, edge_points, &(0..edge_points.len()).collect::<Vec<_>>(), radius, chamfer, entry,
                    )?;
                    if !missing.is_empty() {
                        return Err(KernelRefusal::internal(
                            KernelStage::Validate,
                            "runout_walls_missing",
                            format!(
                                "{entry}: the runout composition built a solid with no wall on {}",
                                missing.iter().map(|index| selection_label(*index, edge_points, edge_names, name)).collect::<Vec<_>>().join(", ")
                            ),
                        ));
                    }
                    let n = edge_points.len();
                    return Ok((
                        result,
                        BlendSelection { requested: n, applied: (0..n).collect(), rejected: Vec::new(), reason: None },
                    ));
                }
                Err(error) => {
                    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                        eprintln!("runout composition refused: {error}");
                    }
                    check_mixed_corner_convexity(
                        solid, &selected_ids, edge_points, edge_names, radius, name, Some(&error),
                    )?;
                }
            }
        }
        // A selection whose corner setbacks consume a whole edge — the exact
        // degenerate radius, every face shrunk to a point — has no strip for
        // the network to build and nothing the cutter or the subset search
        // can rescue (the search answered the r = 10 cube with nine walls and
        // a 5174 volume against the sphere's 4189).  Terminal, by name, here.
        check_degenerate_corner_setbacks(
            solid,
            &selected_ids,
            edge_points,
            edge_names,
            radius,
            name,
        )?;
    }

    let n = edge_points.len();
    let complete = |applied: Vec<usize>| BlendSelection {
        requested: n,
        applied,
        rejected: Vec::new(),
        reason: None,
    };
    // Every subset below is built against the SAME solid, and a selection of
    // several components is composed one component at a time in input order,
    // so two subsets that keep the same leading components rebuild the same
    // intermediate solids. The memo keeps each component's result under the
    // chain of components composed before it, which is everything that result
    // depends on here (the solid, radius, chamfer and name are fixed for the
    // call), so the search returns what it did without the memo, only without
    // the rebuilding. On the 2026-09-26 cross-drilled sphere report four bored
    // rims were re-blended ahead of the same failing component in each of the
    // (up to 372) subsets tried: 13 min 25 s for one refusal, 34 s with it.
    let mut memo = ComponentMemo::default();
    let group_err = match fillet_edges_group_in(
        solid,
        edge_points,
        edge_names,
        radius,
        chamfer,
        name,
        Some(&mut memo),
    ) {
        Ok(result) => {
            // The group returned a solid, which is NOT the same as the group
            // having blended every edge.  `validate()` cannot tell the
            // difference — it checks incidence, and a solid missing a wall is
            // perfectly incident — so the walls are COUNTED.
            let missing =
                walls_missing(solid, &result, edge_points, &(0..n).collect::<Vec<_>>(), radius, chamfer, entry)?;
            if missing.is_empty() {
                return Ok((result, complete((0..n).collect())));
            }
            let applied: Vec<usize> = (0..n).filter(|index| !missing.contains(index)).collect();
            return Ok((
                result,
                BlendSelection {
                    requested: n,
                    applied,
                    rejected: missing,
                    reason: None,
                },
            ));
        }
        Err(error) => error,
    };
    // A wall that FOLDS THROUGH ITSELF is terminal for the whole selection, and
    // for the reason the two checks above are: dropping edges cannot rescue it.
    // The subset search would happily answer this document's two collars with
    // the ONE that does not fold, and return a solid whose volume is a blend
    // short — the same silent partial `both_collars_add_independently` exists
    // to catch.  Reported by name instead (`blend/fold.rs`).
    // A miter past the sharp edge's far vertex needs support-face switching,
    // not a smaller selection. Do not let the subset search mask the limit.
    if crate::blend::is_wall_fold(&group_err) || crate::blend::is_miter_support_overrun(&group_err) {
        return Err(group_err);
    }
    // Fewer than two edges: nothing to drop, so the group error is final.
    // Cap the combinatorial search so a large malformed selection cannot
    // explode (the full group carries the common case; the search is a
    // rare fallback).
    if n < 2 || n > 12 {
        return Err(group_err);
    }
    // Drop the fewest edges first (largest surviving subset), trying the
    // drop-sets in lexicographic order so the result is deterministic.
    // Take the first subset that blends to a valid solid IN WHICH EVERY KEPT
    // EDGE GREW A WALL: a subset that itself silently drops one of its own
    // edges is not the maximal subset, it is a smaller one wearing a larger
    // one's label.
    for drop in 1..n {
        for dropped in index_combinations(n, drop) {
            let keep: Vec<usize> = (0..n).filter(|i| !dropped.contains(i)).collect();
            let points: Vec<Vec3> = keep.iter().map(|i| edge_points[*i]).collect();
            // Subset the per-edge blend-face names with the IDENTICAL
            // drop-set so `kept_names[k]` still names `kept[k]`.
            let kept_names: Option<Vec<String>> =
                edge_names.map(|names| keep.iter().map(|i| names[*i].clone()).collect());
            let Ok(result) = fillet_edges_group_in(
                solid,
                &points,
                kept_names.as_deref(),
                radius,
                chamfer,
                name,
                Some(&mut memo),
            ) else {
                continue;
            };
            if !result.validate().is_empty() {
                continue;
            }
            if !walls_missing(solid, &result, edge_points, &keep, radius, chamfer, entry)?.is_empty() {
                continue;
            }
            return Ok((
                result,
                BlendSelection {
                    requested: n,
                    applied: keep,
                    rejected: dropped,
                    reason: Some(group_err.message),
                },
            ));
        }
    }
    Err(group_err)
}

/// Which of `selected` (indices into the caller's own selection) carry NO blend
/// wall in `result` — decided by the MATERIAL, not by a name.
///
/// A blended edge is an edge whose material changed: a convex blend removes
/// the sliver under the edge, so a point of the original edge ends up OUTSIDE
/// the answer; a concave one pads the crease, so it ends up INSIDE. An edge
/// the selection dropped is still the sharp edge it was, and its points still
/// lie ON the skin. That statement holds for every caller and every lane,
/// which the previous attribution did not: it looked for one face per edge
/// carrying the caller's per-edge name, so a caller passing `None` (the wasm
/// ABI, `fillet_edges_json`) got NO count at all, and a caller passing names
/// was told a wall was missing where the network had merged two coaxial walls
/// into one face (the U-channel at half its width keeps one name for two
/// edges) or collapsed the walls of a full-width star into its corner patch
/// (`{base}:CORNER:…`, and no wall name at all). Both were measured
/// 2026-09-26; the named path REFUSED those complete blends.
///
/// The classifier answers `On` inside a band, so the check states how far
/// the blend of THIS size moves the edge at each probed station — `r·(1/cos(α/2)
/// − 1)` for a fillet and `d·sin(α/2)` for a chamfer, `α` the angle between the
/// two faces' outward normals there — and probes the station the blend moves
/// MOST. An edge that is tangent at one end (the miter seam two rounds leave
/// runs to 180° at its pole) is still attributed where it is sharp. Only when
/// no station clears the band is the selection refused by name, rather than
/// an unseeable move read as a missing wall.
pub(super) fn walls_missing(
    original: &BrepSolid,
    result: &BrepSolid,
    edge_points: &[Vec3],
    selected: &[usize],
    size: f64,
    chamfer: bool,
    entry: &str,
) -> Result<Vec<usize>, KernelRefusal> {
    use crate::{PointClass, SolidClassifier};
    if selected.is_empty() {
        return Ok(Vec::new());
    }
    let band = crate::KernelTolerances::for_solid(result, 1e-7).model;
    // `classify` reads `On` within ten times its band; attribution needs the
    // move to clear that by the same margin again.
    let visible = band * 100.0;
    let classifier = SolidClassifier::new(result, band)
        .or_refuse(KernelStage::Validate, "wall_classifier")?;
    const STATIONS: usize = 9;
    let mut missing = Vec::new();
    for &index in selected {
        let point = edge_points[index];
        let edge_id = resolve_edge_by_point(original, point)?;
        let (edge, faces) = super::analyze::edge_and_faces(original, edge_id)?;
        // The station the blend moves most, and how far.
        let mut best: Option<(Vec3, f64)> = None;
        for step in 1..=STATIONS {
            let t = edge.t0 + (edge.t1 - edge.t0) * step as f64 / (STATIONS + 1) as f64;
            let Ok(station) = edge.curve.evaluate(t) else {
                continue;
            };
            let clearance = match faces[..] {
                [first, second] => {
                    let (Ok((_, a)), Ok((_, b))) = (
                        super::analyze::face_plane_normal(first, station),
                        super::analyze::face_plane_normal(second, station),
                    ) else {
                        continue;
                    };
                    let half: f64 = 0.5 * a.dot(b).clamp(-1.0, 1.0).acos();
                    if chamfer {
                        size * half.sin()
                    } else {
                        size * (1.0 / half.cos().max(1e-12) - 1.0)
                    }
                }
                // No dihedral to read: probe anyway, unguarded.
                _ => f64::INFINITY,
            };
            if best.map_or(true, |(_, known)| clearance > known) {
                best = Some((station, clearance));
            }
        }
        let Some((station, clearance)) = best else {
            return Err(KernelRefusal::internal(
                KernelStage::Validate,
                "wall_attribution_stations",
                format!(
                    "{entry}: cannot attribute a wall to edge #{index} at ({:.6}, {:.6}, {:.6}) — no \
                     station of it could be evaluated",
                    point.x, point.y, point.z
                ),
            ));
        };
        if !(clearance > visible) {
            // The faces themselves put the move under the band: the input
            // leaves the attribution undecidable, and it is refused rather
            // than guessed.
            return Err(KernelRefusal::ill_posed(
                KernelStage::Validate,
                "wall_attribution_band",
                format!(
                    "{entry}: cannot attribute a wall to edge #{index} at ({:.6}, {:.6}, {:.6}) — \
                     its faces are so nearly tangent that a size-{size} blend moves the edge by at \
                     most {clearance:.3e}, inside the {visible:.3e} the classifier can see; the \
                     selection is refused rather than reported on a guess",
                    point.x, point.y, point.z
                ),
            ));
        }
        if classifier
            .classify(station)
            .or_refuse(KernelStage::Validate, "wall_classify")?
            .class
            == PointClass::On
        {
            missing.push(index);
        }
    }
    Ok(missing)
}

/// The blend-FACE name for the `i`-th selected edge: its per-edge name when the
/// caller supplied the parallel `edge_names` (feature path — each wall named
/// after its originating edge, `{fid}:BLEND:{edge}`), else the single base
/// `name` for every wall (the legacy/test path, byte-identical to before).
fn per_edge_name<'a>(
    edge_names: Option<&'a [String]>,
    base: Option<&'a str>,
    i: usize,
) -> Option<&'a str> {
    match edge_names {
        Some(names) => names.get(i).map(|value| value.as_str()),
        None => base,
    }
}

/// The star-corner patch name: `{base}:CORNER:{sorted+join of adjacent edge
/// names}` when per-edge names were supplied (feature path), else the single
/// `base` (legacy/test path — the corner keeps the wall name, pre-change
/// behavior).  `base` is `{fid}:BLEND` and each `edge_names[i]` is the composed
/// `{fid}:BLEND:{edge}`, so stripping the `{base}:` prefix recovers the bare
/// originating-edge name for the join.  Unique per corner: two distinct corners
/// never share the same set of >=3 selected edges.
fn corner_face_name(
    edge_names: Option<&[String]>,
    base: Option<&str>,
    adjacent: &[usize],
) -> Option<String> {
    match (edge_names, base) {
        (Some(names), Some(base)) => {
            let prefix = format!("{base}:");
            let mut raws: Vec<&str> = adjacent
                .iter()
                .filter_map(|&i| names.get(i))
                .map(|composed| composed.strip_prefix(&prefix).unwrap_or(composed.as_str()))
                .collect();
            raws.sort_unstable();
            raws.dedup();
            Some(format!("{base}:CORNER:{}", raws.join("+")))
        }
        _ => base.map(|value| value.to_string()),
    }
}

/// Round an isolated convex boss edge first, then blend the concave rim
/// including its new circular bridge across the convex wall. Omitting that
/// bridge leaves the two rim stripes disconnected at the mixed corner.
#[allow(clippy::too_many_arguments)]
fn rounded_boss_composition(
    solid: &BrepSolid,
    selected_ids: &[u64],
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
    corners: &[crate::blend::MixedCorner],
) -> Result<BrepSolid, KernelRefusal> {
    let decline = || {
        KernelRefusal::unsupported(KernelStage::Classify, "rounded_boss_selection",
        "the rounded boss composition requires one isolated convex edge and one orthogonal mixed corner")
    };
    if std::env::var("BREP_NO_NETWORK").is_ok() || corners.len() != 1 {
        return Err(decline());
    }
    let corner = &corners[0];
    if corner.convex.len() != 1 || corner.concave.len() != 2 {
        return Err(decline());
    }
    let convexity =
        crate::blend::selection_convexity(solid, selected_ids, radius).ok_or_else(decline)?;
    if convexity.iter().filter(|convex| **convex).count() != 1 {
        return Err(decline());
    }
    let spine_index = corner.convex[0];
    let spine = solid
        .edges
        .iter()
        .find(|edge| edge.id == selected_ids[spine_index])
        .ok_or_else(decline)?;
    let far = if spine.start_vertex_id == corner.vertex {
        spine.end_vertex_id
    } else {
        spine.start_vertex_id
    };
    if solid.edges.iter().any(|edge| {
        edge.id != spine.id
            && selected_ids.contains(&edge.id)
            && (edge.start_vertex_id == far || edge.end_vertex_id == far)
    }) {
        return Err(decline());
    }
    let uses = |face: &FaceRecord, edge: u64| {
        face.loops
            .iter()
            .flat_map(|rim| &rim.coedges)
            .any(|coedge| coedge.edge_id == edge)
    };
    let cap = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .find(|face| {
            corner
                .concave
                .iter()
                .all(|index| uses(face, selected_ids[*index]))
        })
        .ok_or_else(decline)?;
    let faces: Vec<&FaceRecord> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face.id == cap.id || uses(face, spine.id))
        .collect();
    let normals: Option<Vec<Vec3>> = faces
        .iter()
        .map(|face| match face.surface.analytic() {
            Some(crate::AnalyticSurface::Plane { u_dir, v_dir, .. }) => {
                u_dir.cross(*v_dir).normalized().ok()
            }
            _ => None,
        })
        .collect();
    let normals = normals.ok_or_else(decline)?;
    if normals.len() != 3
        || (0..3).any(|a| (a + 1..3).any(|b| normals[a].dot(normals[b]).abs() > 1e-6))
    {
        return Err(decline());
    }
    // Name the wall even for unnamed callers so the bridge is identified
    // from its carriers rather than its distance to the old vertex.
    let spine_name = per_edge_name(edge_names, name, spine_index)
        .unwrap_or("FILLET:ROUNDED_BOSS_SPINE")
        .to_owned();
    let rounded = fillet_edges_group_in(
        solid,
        &[edge_points[spine_index]],
        Some(&[spine_name.clone()]),
        radius,
        false,
        name,
        None,
    )?;
    let bridges: Vec<&EdgeRecord> = rounded
        .edges
        .iter()
        .filter(|edge| {
            let faces: Vec<&FaceRecord> = rounded
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .filter(|face| uses(face, edge.id))
                .collect();
            faces.iter().any(|face| face.id == cap.id)
                && faces
                    .iter()
                    .any(|face| face.name.as_deref() == Some(&spine_name))
        })
        .collect();
    if bridges.len() != 1 {
        return Err(decline());
    }
    let bridge = bridges[0];
    let rim_ids: Vec<u64> = selected_ids
        .iter()
        .enumerate()
        .filter(|(index, _)| *index != spine_index)
        .map(|(_, id)| *id)
        .collect();
    // Both ends must join requested rim edges: never extend a selection
    // into an unselected perimeter.
    if ![bridge.start_vertex_id, bridge.end_vertex_id]
        .iter()
        .all(|vertex| {
            rounded.edges.iter().any(|edge| {
                rim_ids.contains(&edge.id)
                    && (edge.start_vertex_id == *vertex || edge.end_vertex_id == *vertex)
            })
        })
    {
        return Err(decline());
    }
    let mut points = Vec::new();
    let mut names = Vec::new();
    for (index, id) in selected_ids.iter().enumerate() {
        if index == spine_index {
            continue;
        }
        let edge = rounded
            .edges
            .iter()
            .find(|edge| edge.id == *id)
            .ok_or_else(decline)?;
        points.push(
            edge.curve
                .evaluate(0.5 * (edge.t0 + edge.t1))
                .or_refuse(KernelStage::Collect, "rim_midpoint")?,
        );
        names.push(
            per_edge_name(edge_names, name, index)
                .unwrap_or("FILLET:RIM")
                .to_owned(),
        );
    }
    points.push(
        bridge
            .curve
            .evaluate(0.5 * (bridge.t0 + bridge.t1))
            .or_refuse(KernelStage::Collect, "bridge_midpoint")?,
    );
    let mut adjacent = corner.concave.clone();
    adjacent.push(spine_index);
    names.push(
        corner_face_name(edge_names, name, &adjacent).unwrap_or("FILLET:BOSS_CORNER".to_owned()),
    );
    fillet_edges_group_in(&rounded, &points, Some(&names), radius, false, name, None)
}

/// A corner with two concave edges and a convex one requires composition.
///
/// Where selected edges meet, the rolling ball has to touch every face around
/// the vertex from ONE side. A convex edge running into the concave edges it
/// meets (a rib spine dying into the fillets at its own base) asks the ball to
/// sit under the shared face for one blend and over it for the other, so no
/// ball seats there and the corner has no closure: physically the convex blend
/// runs out against the concave beads partway along the edge, which is a vertex
/// blend the stripe network does not construct.
///
/// The re-entrant vertex of a notch — ONE concave edge, the rest convex — is
/// NOT this: the ball rolls round that single concave edge from one convex
/// blend to the next, and `round_concave_chain_corner` / `round_convex_corner`
/// close it with the horn-torus sector.  Only its mirror is refused.
///
/// This is TERMINAL — deliberately not a fall-through to the cutter
/// composition. The cutter answers this shape with a watertight but shredded
/// solid: a sliver end cap at each unclosed corner and the carrier face split
/// so the original name lands on a fragment, which breaks every downstream
/// reference to it. A named refusal that says which edges to separate is
/// worth more than that solid. The runout composition (the 2026-09-30 record
/// §5): a selection whose convex edges die into concave ones at
/// mixed-convexity corners is built in two stages — the CONCAVE edges first,
/// through the ordinary group path (exactly the ring the user gets selecting
/// them alone), then one runout wall per convex spine on that ring result,
/// marched from pole to pole with its carriers switching onto the ring's own
/// pads. Accepted under the network's own checks.
///
/// The class built here: every convex edge of the selection ends on a mixed
/// corner at BOTH ends and shares no vertex with another convex edge, and
/// every other selected edge is concave.  Anything else keeps the named
/// refusal.
#[allow(clippy::too_many_arguments)]
fn runout_composition(
    solid: &BrepSolid,
    selected_ids: &[u64],
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
    corners: &[crate::blend::MixedCorner],
    entry: &str,
) -> Result<BrepSolid, KernelRefusal> {
    if std::env::var("BREP_NO_NETWORK").is_ok() {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_no_network", "the runout composition is a network construction and BREP_NO_NETWORK is set"));
    }
    let convexity = crate::blend::selection_convexity(solid, selected_ids, radius)
        .ok_or(KernelRefusal::unsupported(KernelStage::Classify, "runout_convexity", "the selection's convexity could not be read edge by edge"))?;
    let mut spine_ends = vec![0usize; selected_ids.len()];
    for corner in corners {
        if corner.convex.len() != 1 {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_corner_shape", format!(
                "the corner at ({:.3}, {:.3}, {:.3}) has {} convex edges; the runout is built for one convex edge dying into concave ones",
                corner.point.x, corner.point.y, corner.point.z, corner.convex.len()
            )));
        }
        spine_ends[corner.convex[0]] += 1;
    }
    let mut spines: Vec<usize> = Vec::new();
    let mut concave: Vec<usize> = Vec::new();
    for (index, ends) in spine_ends.iter().enumerate() {
        match (*ends, convexity[index]) {
            (2, true) => spines.push(index),
            (0, false) => concave.push(index),
            (ends, convex) => {
                return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_selection_shape", format!(
                    "{} is {} and meets a mixed corner at {ends} of its ends; the runout is built for a convex edge running out at both ends among concave edges",
                    selection_label(index, edge_points, edge_names, name),
                    if convex { "convex" } else { "concave" },
                )));
            }
        }
    }
    if concave.is_empty() {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "runout_selection_shape", "no concave edges to build the ring from"));
    }
    // Stage 1: the ring, exactly as the concave edges build on their own.
    let ring_points: Vec<Vec3> = concave.iter().map(|index| edge_points[*index]).collect();
    let ring_names: Option<Vec<String>> = edge_names.map(|names| concave.iter().map(|index| names[*index].clone()).collect());
    let ring = fillet_edges_group_with_corner_policy(solid, &ring_points, ring_names.as_deref(), radius, false, name, None, false)
        .map_err(|error| error.with_message(|error| format!("the concave ring did not build: {error}")))?;
    // Stage 2: the runout walls on it.
    let spine_walls: Vec<(u64, Option<String>)> = spines
        .iter()
        .map(|index| (selected_ids[*index], per_edge_name(edge_names, name, *index).map(str::to_string)))
        .collect();
    let spine_ids: Vec<u64> = spine_walls.iter().map(|(id, _)| *id).collect();
    let mut result = crate::blend::build_runout_walls(&ring, &spine_walls, radius)?;
    heal_edge_vertex_gaps(&mut result, radius)?;
    crate::blend::fit_planar_charts_to_trims(&ring, &mut result)?;
    let issues = result.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::new(
            RefusalClass::InvalidResultTopology { issues: issues.len() as u32 },
            KernelStage::Validate,
            format!("the runout walls do not validate: {issues:?}"),
        ));
    }
    check_blend_interference(&ring, &result, &spine_ids, entry)?;
    check_loop_self_crossings(&result, entry)?;
    crate::accept_sound(result, entry)
}

fn check_mixed_corner_convexity(
    solid: &BrepSolid,
    edge_ids: &[u64],
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
    composition_error: Option<&KernelRefusal>,
) -> Result<(), KernelRefusal> {
    match crate::blend::mixed_convexity_corner(solid, edge_ids, radius) {
        Some(corner) => {
            // The refusal carries the RUNOUT's own numbers when they can be
            // measured — the station, the carrier pair, the stop and the pole
            // — so it says which construction is missing and where, not only
            // that the corner has none.  A measurement that fails leaves the
            // refusal exactly as it was: the corner is refused either way.
            let measured = match crate::blend::plan_runout(solid, edge_ids, radius, &corner) {
                Ok(plan) => Some(plan),
                Err(error) => {
                    if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                        eprintln!("runout not measured: {error}");
                    }
                    None
                }
            };
            // "this kernel does not build that runout here": a named deferral,
            // carrying why the composition declined this selection.
            let mut message = mixed_corner_message(
                &corner,
                edge_points,
                edge_names,
                name,
                measured.as_ref(),
            );
            if let Some(error) = composition_error {
                message.push_str(&format!(" The runout composition was tried on this selection and refused: {error}"));
            }
            Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "mixed_corner_convexity",
                message,
            ))
        }
        None => Ok(()),
    }
}

/// Refuse a selection one of whose edges keeps NO blend strip between the
/// corner balls at its two ends (see `degenerate_corner_setbacks`).
fn check_degenerate_corner_setbacks(
    solid: &BrepSolid,
    edge_ids: &[u64],
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    name: Option<&str>,
) -> Result<(), KernelRefusal> {
    let Some(setback) = crate::blend::degenerate_corner_setbacks(solid, edge_ids, radius) else {
        return Ok(());
    };
    let [from, to] = setback.stops;
    let how = if setback.crossed {
        format!(
            "run {:.3e} PAST each other along it (the radius is beyond the limit)",
            setback.strip
        )
    } else {
        format!(
            "meet there ({:.3e} apart against a bar of {:.3e})",
            setback.strip, setback.bar
        )
    };
    // "The radius is at or past the degenerate limit … reduce it": a
    // parameter the geometry refuses, read at `Classify`.
    Err(KernelRefusal::input(
        KernelStage::Classify,
        "degenerate_corner_setbacks",
        format!(
            "fillet_edges: at r = {radius} the corner setbacks consume the whole of {} — the rolling \
             balls seated at its two corners touch face {} at ({:.4}, {:.4}, {:.4}) and ({:.4}, \
             {:.4}, {:.4}) and {how}, so no blend strip is left between them and the faces it lies \
             on are consumed whole. The radius is at or past the degenerate limit for this \
             selection; reduce it.",
            selection_label(setback.edge, edge_points, edge_names, name),
            setback.face_id,
            from.x,
            from.y,
            from.z,
            to.x,
            to.y,
            to.z,
        ),
    ))
}

/// How a refusal names the `index`-th selected edge: its originating edge
/// name when the feature layer supplied one (with the feature's base prefix
/// stripped), else the picked point.
fn selection_label(
    index: usize,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    name: Option<&str>,
) -> String {
    let named = edge_names.and_then(|names| names.get(index)).map(|composed| match name {
        Some(base) => composed
            .strip_prefix(&format!("{base}:"))
            .unwrap_or(composed)
            .to_string(),
        None => composed.clone(),
    });
    named.unwrap_or_else(|| match edge_points.get(index) {
        Some(point) => format!("the edge at ({:.3}, {:.3}, {:.3})", point.x, point.y, point.z),
        None => format!("selection #{}", index + 1),
    })
}

/// The refusal text for [`check_mixed_corner_convexity`], naming the edges the
/// way the user selected them (their originating edge names when the feature
/// layer supplied them, else the picked point).
fn mixed_corner_message(
    corner: &crate::blend::MixedCorner,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    name: Option<&str>,
    measured: Option<&crate::blend::RunoutPlan>,
) -> String {
    let label = |index: usize| selection_label(index, edge_points, edge_names, name);
    let list = |indices: &[usize]| -> String {
        let parts: Vec<String> = indices.iter().map(|index| label(*index)).collect();
        match parts.split_last() {
            None => "none".to_string(),
            Some((last, [])) => last.clone(),
            Some((last, rest)) => format!("{} and {last}", rest.join(", ")),
        }
    };
    let mut message = format!(
        "fillet_edges: the selection mixes convexity at the corner ({:.3}, {:.3}, {:.3}) — convex \
         {} meets concave {} there. One rolling ball cannot touch the face they share from both \
         sides at once, so that corner has no closure: the convex blend runs out against the \
         concave ones partway along the edge, and this kernel does not build that runout yet. \
         Blending the concave edges in one fillet and the convex ones in another does not reach \
         a wall on every one of them either: the convex wall comes out at FULL length in either \
         order, and its contact leaves the faces it runs along where their own loops would then \
         cross. Blend the concave edges first and the blend acceptance refuses each convex edge \
         whose wall runs out into them; blend the convex ones first and the subset search drops \
         the concave edges it cannot compose instead, leaving those unfilleted.",
        corner.point.x,
        corner.point.y,
        corner.point.z,
        list(&corner.convex),
        list(&corner.concave),
    );
    // The runout, measured: where the ball's carriers switch, what they switch
    // to, and where the last stretch degenerates.  These are the numbers the
    // construction owes, so the refusal states them rather than leaving the
    // reader to derive them (`blend/runout.rs`).
    if let Some(plan) = measured {
        message.push_str(&format!(
            " Measured on this selection: the convex edge {}'s ball becomes tangent to face {} — \
             the face its two concave neighbours share — at s = {:.6} from the corner, centred \
             ({:.6}, {:.6}, {:.6}), where its contact on face {} reaches edge {}'s own rail \
             (ball-to-stripe residual {:.3e}). Past that station the second carrier is edge {}'s \
             STRIPE SURFACE, not a face of the input, and the march holds on it for {} \
             stations that fit rows reproducing their own contacts to {:.3e}. On the way the \
             kept mate's contact walks {:.3e} of a span off that face's own patch — into the \
             strip the concave stripe's pad ADDS to it — which is the shape, not an escape: \
             the bar a march is refused at is a full span.",
            plan.convex_edge,
            plan.shared_face,
            plan.station.s,
            plan.station.center.x,
            plan.station.center.y,
            plan.station.center.z,
            plan.replaced_mate,
            plan.second_carrier,
            plan.station.stripe_residual,
            plan.second_carrier,
            plan.marched,
            plan.fit_residual,
            plan.carrier_excursion,
        ));
        match (&plan.stop, plan.first_carrier) {
            (Some(stop), Some(first)) => message.push_str(&format!(
                " It does not simply stop there: at s = {:.6} the KEPT mate (face {}) runs out too \
                 — the ball's contact on it reaches edge {}'s rail, residual {:.3e} — so the FIRST \
                 carrier becomes that stripe as well and the last stretch rolls on two stripes.",
                stop.s, plan.kept_mate, first, stop.stripe_residual,
            )),
            _ => message.push_str(&format!(
                " Where the kept mate (face {}) runs out was not measurable on this selection.",
                plan.kept_mate
            )),
        }
        if let Some(pole) = plan.pole {
            message.push_str(&format!(
                " That stretch ends at a POLE, ({:.6}, {:.6}, {:.6}), where the two concave \
                 stripes' rails on face {} meet and the ball's two contacts merge, so the blend \
                 section shrinks to a point. Two carrier switches and a degenerate terminus is \
                 more than a stop station on a stripe, and none of it is composed yet.",
                pole.x, pole.y, pole.z, plan.shared_face,
            ));
        }
    }
    message
}

/// All ways to choose `k` distinct indices from `0..n`, in lexicographic order.
fn index_combinations(n: usize, k: usize) -> Vec<Vec<usize>> {
    let mut out = Vec::new();
    if k == 0 || k > n {
        return out;
    }
    let mut idx: Vec<usize> = (0..k).collect();
    loop {
        out.push(idx.clone());
        // Advance to the next combination (like counting with carry).
        let mut i = k;
        loop {
            if i == 0 {
                return out;
            }
            i -= 1;
            if idx[i] != i + n - k {
                break;
            }
        }
        idx[i] += 1;
        for j in (i + 1)..k {
            idx[j] = idx[j - 1] + 1;
        }
    }
}

thread_local! {
    /// Running count (per thread) of group builds that reached the CUTTER
    /// COMPOSITION — every network lane above it refused or was rejected.  A
    /// zero delta across a call is the witness that the network answered it;
    /// a face count or a volume cannot say which lane built a solid, since the
    /// cutter can build the same one.  Test-observable; not part of any result.
    pub(crate) static CUTTER_COMPOSITIONS: std::cell::Cell<u64> =
        const { std::cell::Cell::new(0) };
}

/// Blend the WHOLE selection as one group (the single-shot multi-edge fillet).
/// Errors if any selected edge or the shared-corner surgery cannot compose;
/// `fillet_edges` wraps this with a maximal-valid-subset fallback.
/// [`fillet_edges_group_unchecked`] held to the ONE soundness question every
/// other acceptance in this file passes: no face of the result may have a trim
/// loop that crosses itself ([`check_loop_self_crossings`]).
///
/// It sits here, at the group's single exit, rather than beside each `Ok`
/// inside it, because the group has four of them — the closed rim, the stripe
/// network, the per-component composition and the cutter composition — and the
/// cutter's is the one no other check guards at all. The network and the rim
/// ALSO ask it inside, where a bowtie falls through to the next lane instead of
/// failing the group; here there is no next lane, so it refuses.
///
/// The maximal-valid-subset search in [`fillet_edges_reported`] calls this
/// entry, so a subset that blends to a bowtie is not a valid subset either.
fn fillet_edges_group(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    fillet_edges_group_in(solid, edge_points, edge_names, radius, chamfer, name, None)
}

thread_local! {
    /// Running count (per thread) of [`fillet_edges_group`] builds, nested
    /// component builds included. Test-observable; not part of any result.
    pub(crate) static GROUP_BUILDS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

/// One component of a selection: its edge points (as bits, so the key is
/// exact) and its per-edge wall names.
type ComponentKey = (Vec<[u64; 3]>, Option<Vec<String>>);

/// Component results of one [`fillet_edges_reported`] call, keyed by the chain
/// of components composed up to and including the one built. Valid only for
/// the solid, radius, chamfer flag and name of that call.
#[derive(Default)]
struct ComponentMemo {
    built: std::collections::HashMap<Vec<ComponentKey>, Result<BrepSolid, KernelRefusal>>,
}

/// [`fillet_edges_group`], with the subset search's component memo when
/// `memo` is `Some`. Only the call made on the search's own solid may pass it.
fn fillet_edges_group_in(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    memo: Option<&mut ComponentMemo>,
) -> Result<BrepSolid, KernelRefusal> {
    fillet_edges_group_with_corner_policy(solid, edge_points, edge_names, radius, chamfer, name, memo, true)
}

fn fillet_edges_group_with_corner_policy(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    memo: Option<&mut ComponentMemo>,
    wrap_concave: bool,
) -> Result<BrepSolid, KernelRefusal> {
    GROUP_BUILDS.with(|count| count.set(count.get() + 1));
    let entry = if chamfer { "chamfer_edges" } else { "fillet_edges" };
    let result =
        fillet_edges_group_unchecked(solid, edge_points, edge_names, radius, chamfer, name, memo, wrap_concave)?;
    check_loop_self_crossings(&result, entry)?;
    // The BODY's own soundness, beside the loop's. A blend wall is a FITTED
    // surface and a fit can carry two parameters of one wall to one point in
    // space — measured on `inbox-20260902-fillet-two-torus-box-edges`, whose
    // wall `F8:BLEND:Box_NZ|P.T5_Side[0]` does exactly that while `validate()`,
    // connectivity, Euler parity, the loop scan and the face-versus-face scan
    // are all silent. Two walls crossing each other is the same class and is
    // repaired where it can be. Neither is visible to a volume oracle, so a
    // result carrying one reports success for a different shape.
    crate::accept_sound(result, entry)
}

fn fillet_edges_group_unchecked(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    mut memo: Option<&mut ComponentMemo>,
    wrap_concave: bool,
) -> Result<BrepSolid, KernelRefusal> {
    use rustc_hash::FxHashSet as HashSet;

    // Fuse-first operand heal (Lever A) before the multi-edge surgery: snap the
    // input's near-coincident / off-plane vertices (e.g. a revolve pole apex
    // sitting a few microns off the axis) to exact and re-anchor incident
    // edges.  The selected edges are resolved geometrically below, so a
    // sub-heal_tol vertex move never changes which edges are picked; a clean
    // input is left byte-identical.
    let mut healed_input = solid.clone();
    let heal_policy = crate::KernelTolerances::for_solid(&healed_input, 1e-7);
    crate::heal::heal_operands(&mut healed_input, &heal_policy)?;
    let solid = &healed_input;

    // 1. Detect convex corners from the ORIGINAL solid: a point that is an
    //    endpoint of >=3 of the selected edges (a cube/prism-style vertex).
    let mut endpoints: Vec<(Vec3, usize)> = Vec::with_capacity(edge_points.len() * 2);
    // The extent each selected edge has BEFORE any blend trims it, so the
    // sequential build can run every cutter through the shared corners.
    let mut original_extents: Vec<(Vec3, Vec3)> = Vec::with_capacity(edge_points.len());
    let mut selected_edge_ids: Vec<u64> = Vec::with_capacity(edge_points.len());
    for (i, point) in edge_points.iter().enumerate() {
        let edge_id = resolve_edge_by_point(solid, *point)?;
        let edge = solid
            .edges
            .iter()
            .find(|e| e.id == edge_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "resolved_edge",
                    "fillet_edges: resolved edge vanished",
                )
            })?;
        let (start, end) = (
            edge.curve
                .evaluate(edge.t0)
                .or_refuse(KernelStage::Collect, "edge_start")?,
            edge.curve
                .evaluate(edge.t1)
                .or_refuse(KernelStage::Collect, "edge_end")?,
        );
        endpoints.push((start, i));
        endpoints.push((end, i));
        original_extents.push((start, end));
        selected_edge_ids.push(edge_id);
    }
    let mut corners: Vec<Vec3> = Vec::new();
    // The selected-edge INPUT INDICES meeting at each star corner (parallel to
    // `corners`), sorted — used to name the corner patch after its adjacent
    // edges (`{fid}:BLEND:CORNER:{e_a}+{e_b}+…`), UNIQUE per corner because no
    // two distinct corners share the same set of >=3 selected edges.
    let mut corner_edges: Vec<Vec<usize>> = Vec::new();
    let mut chain_corner_count = 0usize;
    let mut chain_corners: Vec<(Vec3, [usize; 2])> = Vec::new();
    let mut used = vec![false; endpoints.len()];
    for i in 0..endpoints.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let mut edges_here: HashSet<usize> = HashSet::default();
        edges_here.insert(endpoints[i].1);
        for j in (i + 1)..endpoints.len() {
            if used[j] {
                continue;
            }
            if endpoints[i].0.sub(endpoints[j].0).length() < 1e-6 {
                used[j] = true;
                edges_here.insert(endpoints[j].1);
            }
        }
        if edges_here.len() >= 3 {
            corners.push(endpoints[i].0);
            let mut adjacent: Vec<usize> = edges_here.into_iter().collect();
            adjacent.sort_unstable();
            corner_edges.push(adjacent);
        } else if edges_here.len() == 2 {
            chain_corner_count += 1;
            let mut adjacent = edges_here.into_iter().collect::<Vec<_>>();
            adjacent.sort_unstable();
            chain_corners.push((endpoints[i].0, [adjacent[0], adjacent[1]]));
        }
    }

    // A FACE selection may contain several disconnected boundary components
    // (the common example is an outer perimeter plus a circular hole rim).
    // Miter composition is only meaningful inside one connected edge graph.
    // Feeding every component into the same INTERSECT/UNION combines multiple
    // independently filleted copies of otherwise untouched support faces; the
    // boolean then imprints those coincident copies and can leave redundant
    // seams (the reported through-hole cylinder split into three faces).
    //
    // Partition by shared original endpoints and process components in input
    // order.  Each recursive call sees one connected component, so it follows
    // the existing miter/sequential path without recursion cycling.  Distinct
    // components are then composed sequentially on the evolving solid.
    let mut component_of = vec![usize::MAX; edge_points.len()];
    let mut components: Vec<Vec<usize>> = Vec::new();
    for seed in 0..edge_points.len() {
        if component_of[seed] != usize::MAX {
            continue;
        }
        let component_index = components.len();
        component_of[seed] = component_index;
        let mut component = vec![seed];
        let mut cursor = 0;
        while cursor < component.len() {
            let current = component[cursor];
            cursor += 1;
            for candidate in 0..edge_points.len() {
                if component_of[candidate] != usize::MAX {
                    continue;
                }
                let connected = [original_extents[current].0, original_extents[current].1]
                    .into_iter()
                    .any(|a| {
                        [original_extents[candidate].0, original_extents[candidate].1]
                            .into_iter()
                            .any(|b| a.sub(b).length() < 1e-6)
                    });
                if connected {
                    component_of[candidate] = component_index;
                    component.push(candidate);
                }
            }
        }
        component.sort_unstable();
        components.push(component);
    }
    if components.len() > 1 {
        let mut separated = solid.clone();
        let mut chain: Vec<ComponentKey> = Vec::with_capacity(components.len());
        for component in components {
            let points = component
                .iter()
                .map(|index| edge_points[*index])
                .collect::<Vec<_>>();
            let names = edge_names.map(|all| {
                component
                    .iter()
                    .map(|index| all[*index].clone())
                    .collect::<Vec<_>>()
            });
            let Some(memo) = memo.as_deref_mut() else {
                separated = fillet_edges_group_with_corner_policy(
                    &separated,
                    &points,
                    names.as_deref(),
                    radius,
                    chamfer,
                    name,
                    None,
                    wrap_concave,
                )?;
                continue;
            };
            let bits = points.iter().map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
            chain.push((bits.collect(), names.clone()));
            let built = match memo.built.get(&chain) {
                Some(built) => built.clone(),
                None => {
                    let built = fillet_edges_group_with_corner_policy(
                        &separated,
                        &points,
                        names.as_deref(),
                        radius,
                        chamfer,
                        name,
                        None,
                        wrap_concave,
                    );
                    memo.built.insert(chain.clone(), built.clone());
                    built
                }
            };
            separated = built?;
        }
        return Ok(separated);
    }

    // 2a. The stripe network (blend/network.rs) is the lane for EVERY
    //     selection: each stripe is marched against THIS solid, each shared
    //     vertex is solved before anything is cut — a star closed by a patch
    //     of the corner ball, a two-edge corner by the seam between the two
    //     blends, a re-entrant corner by a horn torus, a tangent pair by a
    //     flush join, an unselected tangent continuation by a cap — and
    //     nothing is subtracted, so no cutter can overshoot into a neighbour
    //     and no leftover cap has to be identified afterwards.  It refuses
    //     BY NAME on what it does not yet construct (mixed-convexity corners,
    //     no-common-ball stars, pinched edges), and those fall through to the
    //     cutter composition below.
    //
    //     A CLOSED edge is one of those refusals and it has an answer the
    //     network is simply not the lane for: a closed rim has no corner to
    //     share, so `blend_star_network` says "edge N is closed and has no
    //     corner to share" and the composition took it.  `blend_closed_edge`
    //     builds exactly that case and the single-edge ladder in `tool.rs`
    //     has always tried it first — but this function reached only the
    //     network, so a closed rim inside a GROUP went to a cutter that the
    //     same rim selected on its own never saw.  A bored hole's rim splits
    //     out as its own one-edge component (see the component partition
    //     above), so that was every hole rim chamfered as part of a face
    //     selection.  Try it here too, under the same acceptance the network
    //     result gets, and under the same `BREP_NO_NETWORK` A/B.
    if std::env::var("BREP_NO_NETWORK").is_err() {
        if selected_edge_ids.len() == 1 {
            let edge_id = selected_edge_ids[0];
            let closed = solid
                .edges
                .iter()
                .find(|edge| edge.id == edge_id)
                .is_some_and(|edge| edge.start_vertex_id == edge.end_vertex_id);
            if closed {
                let rim_name = per_edge_name(edge_names, name, 0);
                match crate::blend::blend_closed_edge(solid, edge_id, radius, chamfer, rim_name) {
                    Ok(mut rim) => {
                        let entry = if chamfer { "chamfer_edges" } else { "fillet_edges" };
                        let healed = heal_edge_vertex_gaps(&mut rim, radius);
                        // A trim the surgery ran outside its planar carrier's
                        // chart is a wrong body the three checks below all pass
                        // (`crate::blend::fit_planar_charts_to_trims`), so the
                        // carrier is widened here, after the heal has settled
                        // the edges the trims are rebuilt from and before the
                        // bowtie scan reads them.  Its refusals are TERMINAL:
                        // the composition is not a second opinion on a body
                        // whose trims left their carrier.
                        crate::blend::fit_planar_charts_to_trims(solid, &mut rim)?;
                        let issues = rim.validate();
                        let interference =
                            check_blend_interference(solid, &rim, &selected_edge_ids, entry);
                        let bowtie = check_loop_self_crossings(&rim, entry);
                        if healed.is_ok()
                            && issues.is_empty()
                            && interference.is_ok()
                            && bowtie.is_ok()
                        {
                            return Ok(rim);
                        }
                        if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                            eprintln!(
                                "closed-rim result rejected: heal={healed:?} issues={issues:?} \
                                 interference={interference:?} bowtie={bowtie:?}"
                            );
                        }
                    }
                    Err(refusal) => {
                        if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                            eprintln!("closed-rim refused: {refusal}");
                        }
                    }
                }
            }
        }
        let network_names: Vec<Option<String>> = (0..edge_points.len())
            .map(|index| per_edge_name(edge_names, name, index).map(str::to_string))
            .collect();
        let network_corner_name =
            |adjacent: &[usize]| corner_face_name(edge_names, name, adjacent);
        match crate::blend::blend_star_network_with_corner_policy(
            solid,
            &selected_edge_ids,
            radius,
            chamfer,
            &network_names,
            &network_corner_name,
            wrap_concave,
        ) {
            Ok(mut network) => {
                // Fail-safe like the rest of the ladder: a heal or validation
                // problem in the network result falls through to the cutter,
                // it does not fail the group.  So does an UNTRIMMED result: the
                // network re-trims only each stripe's two mates, and
                // `check_blend_interference` is what notices a third face
                // crossing the swept volume -- `validate` cannot, because a
                // self-intersecting solid is still a watertight one.
                let entry = if chamfer { "chamfer_edges" } else { "fillet_edges" };
                let healed = heal_edge_vertex_gaps(&mut network, radius);
                // See the closed-rim lane above: widen a planar carrier the
                // surgery overran, or refuse — terminally — rather than fall
                // through to a composition that cannot answer that question.
                crate::blend::fit_planar_charts_to_trims(solid, &mut network)?;
                let issues = network.validate();
                let interference =
                    check_blend_interference(solid, &network, &selected_edge_ids, entry);
                // And the fourth question, which the other three all pass: does
                // any face of the result have a loop that crosses ITSELF? A
                // wall whose rail ran off its own face comes back as a bowtie,
                // and a bowtie is watertight, incident and connected.
                let bowtie = check_loop_self_crossings(&network, entry);
                if healed.is_ok() && issues.is_empty() && interference.is_ok() && bowtie.is_ok() {
                    return Ok(network);
                }
                if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                    eprintln!(
                        "network result rejected: heal={healed:?} issues={issues:?} \
                         interference={interference:?} bowtie={bowtie:?}"
                    );
                    dump_loops_debug("INPUT", solid);
                    dump_loops_debug("RESULT", &network);
                }
            }
            Err(refusal) => {
                if std::env::var("BREP_DEBUG_NETWORK").is_ok() {
                    eprintln!("network refused: {refusal}");
                }
                // The fail-safe above is for a refusal that says "not this
                // lane".  A marched fit that left its carriers does not say
                // that, and the cutter's answer to the same selection is not a
                // second opinion on it: on the notched cap's stuck connector
                // the composition reads 1724.506048919, +1.81e-2 against the
                // closed form, and it VALIDATES.  A refusal that falls through
                // to a validating wrong solid is not a refusal.
                //
                // Only that one, not all of `is_terminal_blend_refusal`.  The
                // single-edge tool also reads the WALL FOLD as terminal, and
                // making the group build agree costs
                // `inbox_20260902_corner_torus_collar_fillet::each_collar_has_its_own_fold_threshold`
                // a collar that the cutter composes today (measured
                // 2026-09-16: refuses at fold factor 1.035935 on a radius-2.1
                // wall). Whether a group selection should refuse where a
                // single edge does is a decision with a test standing on the
                // current answer, and it is not this slice's.
                //
                // A blend stopping within a sliver of its face's width that
                // cannot be snapped onto the far edge without opening the shell
                // is terminal for the same reason: the SIZE is the question
                // (`blend::CONSUMED_SNAP_UNSOUND`), and the composition's tool
                // cut a sliver short of an edge leaves the sliver no lane keeps.
                //
                // And a FULL-WIDTH corner the network cannot collapse soundly
                // (`blend::RAIL_COLLAPSE_UNSUPPORTED`,
                // `blend::FULL_WIDTH_COLLAPSE_UNSOUND`): the composition's answer
                // to a full-width corner was measured wrong — a 5F/8E/5V miter
                // 8.0e-4 off its closed form at 1e-5 short of the width, and the
                // picture-frame corner at a full-width chamfer star.
                //
                // And a trim that left its planar carrier's chart
                // (`blend::PLANAR_CHART_EDGE_OFF_PLANE`,
                // `blend::PLANAR_CHART_WIDEN_UNSOUND`): the clamped body the
                // composition would answer with is the defect being refused.
                if crate::blend::is_marched_fit_off_carriers(&refusal)
                    || crate::blend::is_consumed_snap(&refusal)
                    || crate::blend::is_rail_collapse(&refusal)
                    || crate::blend::is_full_width_collapse(&refusal)
                    || crate::blend::is_planar_chart_refusal(&refusal)
                    || crate::blend::is_miter_support_overrun(&refusal)
                {
                    return Err(refusal);
                }
            }
        }
    }

    CUTTER_COMPOSITIONS.with(|count| count.set(count.get() + 1));
    // 2. Build the blends.
    //
    //    CHAIN corners (§6.9.6 — exactly TWO selected edges share a vertex):
    //    the sequential build truncates the later blend where it runs into the
    //    earlier one and closes it with a flat bulkhead across the fillet
    //    channel — a hard step, not a transition.  For selections containing
    //    chain corners, blend each edge FULL-LENGTH on the ORIGINAL solid and
    //    INTERSECT the per-edge results instead: the removal volumes union, so
    //    adjacent blends run through the shared corner and trim each other
    //    along their intersection curve — the standard MITER corner, tangent
    //    to the shared face at the seam's tangency end.  Non-adjacent edges
    //    are unaffected (their removals are disjoint, intersection ≡
    //    sequential).
    //
    //    Selections without chain corners keep the sequential build unchanged
    //    (star corners are rounded in step 3 against exactly the sequential
    //    geometry round_convex_corner was built for).
    //    CONVEXITY GUARD: a convex blend REMOVES material (fillet = orig −
    //    cut), a concave blend ADDS it (orig + pad).  Full-length blends
    //    combine as orig − ∪cuts + ∪pads, so the per-edge results compose by
    //    INTERSECTION when every edge is convex and by UNION when every edge
    //    is concave; a mixed selection has no single composition and falls
    //    back to the sequential build.
    // Fillet stars keep the sequential topology required by their sphere
    // patches. Chamfer stars have no subsequent corner patch: compose their
    // full-length removals too, so all bevel planes meet at the corner.
    let miter_operation = if (chain_corner_count > 0 || !corners.is_empty())
        && (chamfer || corners.is_empty())
    {
        let mut any_convex = false;
        let mut any_concave = false;
        for point in edge_points {
            // Convexity is a DIHEDRAL property, so it is read from the general
            // scan (`scan_dihedral`) — the same one `check_mixed_concavity`
            // has already run over every selected edge.  `analyze_edge` used
            // to answer here and it ALSO refuses an edge the EXACT CUTTER
            // cannot build: a straight edge whose mates are not both planes,
            // a circular one whose mates do not share its axis.  Reading that
            // refusal as "unknown convexity" dropped the §6.9.6 miter for
            // every selection touching a curved carrier — the groove rim of a
            // bored box, say — and the sequential build then left a bulkhead
            // face standing in the shared corner (the 2026-09-10 "chamfer
            // produced a spurious face" report).
            match resolve_edge_by_point(solid, *point)
                .and_then(|edge_id| super::analyze::scan_dihedral(solid, edge_id))
            {
                // A mixed edge has no single composition either; it takes the
                // `_ => None` arm below through both flags.
                Ok(profile) if profile.samples > 0 && !profile.is_mixed() => {
                    any_convex |= profile.any_convex;
                    any_concave |= profile.any_concave;
                }
                // Unknown edge class: let the sequential path produce its own
                // (more specific) error or result.
                _ => {
                    any_convex = true;
                    any_concave = true;
                    break;
                }
            }
        }
        match (any_convex, any_concave) {
            (true, false) => Some(crate::BooleanOperation::Intersect),
            (false, true) => Some(crate::BooleanOperation::Union),
            _ => None,
        }
    } else {
        None
    };

    // Fillet/chamfer each edge in turn, resolving its point on the evolving
    // solid (ids shift as earlier fillets rewrite topology; the midpoint of an
    // edge is untouched by the corner surgery of the others).  This is the
    // baseline build used directly for non-miter selections AND as the
    // fallback when the miter composition below cannot reassemble.
    // The composition's per-edge lane: cutter-first ONLY when a corner
    // closure will run afterwards and read cutter-shaped topology (a star or
    // a chain corner).  A selection with no shared vertex — a lone closed
    // rim, a lone chamfer, disconnected edges — has no such closure, so each
    // edge gets the march first there too.
    let composition_lane = if corners.is_empty() && chain_corner_count == 0 {
        Lane::GeneralFirst
    } else {
        Lane::CutterFirst
    };
    let build_sequential = || -> Result<BrepSolid, KernelRefusal> {
        let mut sequential = solid.clone();
        for (index, point) in edge_points.iter().enumerate() {
            let edge_id = resolve_edge_by_point(&sequential, *point)?;
            // Earlier cutters in this loop TRIM the edges that share a corner
            // with them; extend this cutter back over what they took so the
            // two removal volumes union through the corner (§6.9.6 miter)
            // instead of leaving a wedge of material standing behind a flush
            // end cap.  Untouched edges get a zero pad and the historical
            // flush cutter.
            let ends = tool_ends_to_original_extent(
                &sequential,
                edge_id,
                radius,
                original_extents[index],
                if chamfer { &[] } else { &corners },
            );
            // This edge's blend wall carries the name of THIS input edge
            // (`edge_names[index]`); a smooth chain that engulfs several edges
            // is named after the FIRST such input edge processed here (input
            // order), the chain's representative.
            let edge_name = per_edge_name(edge_names, name, index);
            sequential = fillet_or_chamfer(
                &sequential,
                edge_id,
                radius,
                chamfer,
                edge_name,
                ends,
                composition_lane,
            )?;
        }
        Ok(sequential)
    };

    let mut result = if let Some(operation) = miter_operation {
        // The per-edge results still carry their cutters' standing scaffold,
        // which the group settles once its corners are closed. A scaffold face
        // merged here would hand its tag to whatever it merged with.
        let options = crate::BooleanOptions {
            keep_unmerged_name_substrs: vec![CUTTER_SCAFFOLD_NAME.to_string()],
            ..crate::BooleanOptions::default()
        };
        let mut combined: Result<Option<BrepSolid>, KernelRefusal> = Ok(None);
        for (index, point) in edge_points.iter().enumerate() {
            let edge_id = resolve_edge_by_point(solid, *point)?;
            let edge_name = per_edge_name(edge_names, name, index);
            let blended = {
                let entry = if chamfer { "chamfer_edges" } else { "fillet_edges" };
                check_mixed_concavity(solid, edge_id, entry)?;
                check_support_extent(solid, edge_id, radius, entry)?;
                fillet_or_chamfer(
                    solid,
                    edge_id,
                    radius,
                    chamfer,
                    edge_name,
                    ToolEnds::default(),
                    Lane::CutterFirst,
                )?
            };
            combined = match combined {
                Err(e) => Err(e),
                Ok(None) => Ok(Some(blended)),
                Ok(Some(previous)) => {
                    crate::boolean_operation(&previous, &blended, operation, &options)
                        .map(Some)
                        .map_err(|error| {
                            // The boolean's own class survives the wrap.
                            error.with_message(|error| {
                                format!("fillet_edges: chain-corner miter composition failed: {error}")
                            })
                        })
                }
            };
            if combined.is_err() {
                break;
            }
        }
        // A §6.9.6 miter (per-edge blends intersected/unioned through the
        // shared chain corners) can fail to reassemble on faces whose
        // fragmented boundary does not close — e.g. a planar cap whose ENTIRE
        // perimeter is selected, where fragment_face reports an "incomplete
        // run".  Rather than let `fillet_edges` silently DROP a selected edge
        // to recover a valid subset (the reported defect: not every edge of
        // the face gets a fillet), fall back to the sequential build, which
        // blends EVERY selected edge (chain corners get a flat bulkhead
        // instead of a miter).  Only if that also fails to produce a valid
        // solid do we surface the miter error so the caller's
        // maximal-valid-subset search can still run.
        match combined {
            Ok(Some(mitered)) => mitered,
            Ok(None) => {
                return Err(KernelRefusal::input(
                    KernelStage::Collect,
                    "miter_selection",
                    "fillet_edges: no edges selected",
                ))
            }
            Err(miter_err) => match build_sequential() {
                Ok(seq) if seq.validate().is_empty() => seq,
                _ => return Err(miter_err),
            },
        }
    } else {
        build_sequential()?
    };

    // A re-entrant vertex of a selected face perimeter has two selected,
    // convex cap-wall edges and one UNSELECTED concave wall-wall edge.  The
    // two cutter volumes only touch there, so the boolean miter leaves a
    // triangular planar end-cap instead of carrying the rolling ball around
    // the corner.  Close that exact orthogonal class with its horn-torus
    // sector (major radius = minor radius = fillet radius).
    if !chamfer {
        for (corner, adjacent) in &chain_corners {
            let has_concave_unselected_edge = solid.edges.iter().any(|edge| {
                if selected_edge_ids.contains(&edge.id) {
                    return false;
                }
                let Ok(a) = edge.curve.evaluate(edge.t0) else {
                    return false;
                };
                let Ok(b) = edge.curve.evaluate(edge.t1) else {
                    return false;
                };
                (a.sub(*corner).length() < 1e-6 || b.sub(*corner).length() < 1e-6)
                    && analyze_edge(solid, edge.id, radius)
                        .map(|cross| !cross.convex)
                        .unwrap_or(false)
            });
            if !has_concave_unselected_edge {
                continue;
            }
            let corner_name = corner_face_name(edge_names, name, adjacent);
            if let Ok(rounded) = crate::blend::round_concave_chain_corner(
                &result,
                solid,
                *corner,
                [
                    selected_edge_ids[adjacent[0]],
                    selected_edge_ids[adjacent[1]],
                ],
                radius,
                corner_name.as_deref(),
            ) {
                result = rounded;
            }
        }
    }

    // 3. Round the convex corners (fillets only — chamfers keep sharp
    //    vertices).  A corner that cannot be rounded is left as the edge
    //    fillets so the group still succeeds.  Each corner patch is named after
    //    the selected edges meeting there (`{fid}:BLEND:CORNER:{e_a}+…`) so no
    //    two corners collide and the patch stays under the `{fid}:BLEND` prefix.
    if !chamfer {
        for (ci, corner) in corners.iter().enumerate() {
            let corner_name = corner_face_name(edge_names, name, &corner_edges[ci]);
            if let Ok(rounded) =
                crate::blend::round_convex_corner(&result, *corner, radius, corner_name.as_deref())
            {
                result = rounded;
            }
        }
    }

    // Heal any residual vertex/edge gaps introduced by the corner-rounding
    // surgery (the per-edge results are already healed inside fillet_or_chamfer).
    heal_edge_vertex_gaps(&mut result, radius)?;
    crate::blend::fit_planar_charts_to_trims(solid, &mut result)?;
    Ok(settle_cutter_scaffold(result))
}

/// Variable-radius fillet/chamfer of a GROUP of edges (§4.9.5), the app entry
/// for tapered blends: each selected edge (resolved by a point on it) is
/// blended with the SAME radius profile `radii` — a list of (edge-fraction,
/// radius) stops in [0,1] — applied along that edge's own parameterization.
/// Edges are blended independently (no shared-vertex corner rounding; a
/// variable-radius star has no single tangent ball), so this is the tapered
/// counterpart of `fillet_edges` for the constant case.
pub fn fillet_edges_variable(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    radii: &[(f64, f64)],
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    if edge_points.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "variable_selection",
            "fillet_edges_variable: no edges selected",
        ));
    }
    let per_edge_stops: Vec<Vec<(f64, f64)>> = vec![radii.to_vec(); edge_points.len()];
    fillet_edges_variable_impl(
        solid,
        edge_points,
        edge_names,
        &per_edge_stops,
        chamfer,
        name,
        "fillet_edges_variable",
        true,
    )
}

/// The shared variable-radius group core: each selected edge `i` is blended
/// with ITS OWN stop list `per_edge_stops[i]` (the legacy entry replicates one
/// list; the law entries sample a chain-abscissa [`crate::law::RadiusLaw`] per
/// edge).  `allow_tapered_sequential` keeps the legacy entry's sequential
/// fallback for tapered chains (whose end state is the honest
/// "mismatched radii" validation gate); the law entries pass `false` because
/// their stop fractions are computed against the ORIGINAL edges — the
/// sequential build re-resolves edges on the evolving solid whose shared
/// corners are already TRIMMED by earlier blends, which would silently distort
/// the law's abscissa mapping (constant stops are immune, so they may still
/// fall back).
#[allow(clippy::too_many_arguments)]
fn fillet_edges_variable_impl(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    per_edge_stops: &[Vec<(f64, f64)>],
    chamfer: bool,
    name: Option<&str>,
    entry: &str,
    allow_tapered_sequential: bool,
) -> Result<BrepSolid, KernelRefusal> {
    debug_assert_eq!(per_edge_stops.len(), edge_points.len());
    // A law that is one constant everywhere IS the constant-radius fillet:
    // hand it to the constant group, whose corners are constructed (the
    // stripe network) rather than composed by boolean.
    if let Some(constant) = per_edge_stops
        .first()
        .and_then(|stops| stops.first())
        .map(|(_, radius)| *radius)
    {
        let uniform = per_edge_stops.iter().all(|stops| {
            stops
                .iter()
                .all(|(_, radius)| (radius - constant).abs() <= 1e-12 * (1.0 + constant.abs()))
        });
        if uniform && constant > 0.0 {
            return fillet_edges(solid, edge_points, edge_names, constant, chamfer, name);
        }
    }
    let max_radius = per_edge_stops
        .iter()
        .flat_map(|stops| stops.iter())
        .map(|(_, r)| r.abs())
        .fold(0.0_f64, f64::max);

    // Chain corners miter exactly like the constant-radius group (§6.9.6):
    // per-edge blends on the ORIGINAL solid composed by boolean — Intersect
    // when every edge is convex, Union when every edge is concave.  Mixed or
    // unclassifiable selections keep the sequential build.  Variable blends
    // never round star vertices, so unlike the constant group there is no
    // sequential-only star path to protect.
    let mut chain_corner = false;
    {
        use rustc_hash::FxHashSet as HashSet;
        let mut endpoints: Vec<(Vec3, usize)> = Vec::with_capacity(edge_points.len() * 2);
        for (i, point) in edge_points.iter().enumerate() {
            if let Ok(edge_id) = resolve_edge_by_point(solid, *point) {
                if let Some(edge) = solid.edges.iter().find(|e| e.id == edge_id) {
                    if let (Ok(a), Ok(b)) =
                        (edge.curve.evaluate(edge.t0), edge.curve.evaluate(edge.t1))
                    {
                        endpoints.push((a, i));
                        endpoints.push((b, i));
                    }
                }
            }
        }
        let mut used = vec![false; endpoints.len()];
        for i in 0..endpoints.len() {
            if used[i] {
                continue;
            }
            used[i] = true;
            let mut edges_here: HashSet<usize> = HashSet::default();
            edges_here.insert(endpoints[i].1);
            for j in (i + 1)..endpoints.len() {
                if used[j] {
                    continue;
                }
                if endpoints[i].0.sub(endpoints[j].0).length() < 1e-6 {
                    used[j] = true;
                    edges_here.insert(endpoints[j].1);
                }
            }
            if edges_here.len() == 2 {
                chain_corner = true;
            }
        }
    }
    let miter_operation = if chain_corner {
        let probe_radius = if max_radius > 0.0 { max_radius } else { 1.0 };
        let mut any_convex = false;
        let mut any_concave = false;
        for point in edge_points {
            match resolve_edge_by_point(solid, *point)
                .and_then(|edge_id| analyze_edge(solid, edge_id, probe_radius))
            {
                Ok(cross) if cross.convex => any_convex = true,
                Ok(_) => any_concave = true,
                Err(_) => {
                    any_convex = true;
                    any_concave = true;
                    break;
                }
            }
        }
        match (any_convex, any_concave) {
            (true, false) => Some(crate::BooleanOperation::Intersect),
            (false, true) => Some(crate::BooleanOperation::Union),
            _ => None,
        }
    } else {
        None
    };

    // Try the miter first; the variable blend's FITTED boundary curves are
    // only ~1e-3 accurate at blend-blend tangencies (unlike the exact
    // constant-radius cylinders), so the composition can fail — fall back to
    // the sequential build then, which is never worse than the pre-miter
    // behavior.  Tightening the taper surface's endpoint fitting is the
    // documented follow-up that would make the miter stick.
    let miter_attempt: Option<BrepSolid> = if let Some(operation) = miter_operation {
        let options = crate::BooleanOptions::default();
        let mut combined: Option<BrepSolid> = None;
        let mut failed = false;
        for (index, point) in edge_points.iter().enumerate() {
            let Ok(edge_id) = resolve_edge_by_point(solid, *point) else {
                failed = true;
                break;
            };
            let edge_name = per_edge_name(edge_names, name, index);
            let Ok(blended) = crate::blend::blend_edge_variable(
                solid,
                edge_id,
                &per_edge_stops[index],
                chamfer,
                edge_name,
            ) else {
                failed = true;
                break;
            };
            let next = match combined.take() {
                None => blended,
                Some(previous) => {
                    match crate::boolean_operation(&previous, &blended, operation, &options) {
                        Ok(next) => next,
                        Err(_) => {
                            failed = true;
                            break;
                        }
                    }
                }
            };
            combined = Some(next);
        }
        if failed {
            None
        } else {
            combined.filter(|s| s.validate().is_empty())
        }
    } else {
        None
    };
    let mut result = match miter_attempt {
        Some(mitered) => mitered,
        None => {
            // Any stop list that is NOT radius-uniform (the same 1e-12
            // relative criterion `blend_edge_variable` uses for its exact
            // constant-radius degeneration) makes the sequential build
            // abscissa-distorting on trimmed chain edges; law entries refuse
            // instead of silently shifting the law.
            let tapered = per_edge_stops.iter().any(|stops| {
                stops.first().is_some_and(|(_, first)| {
                    stops
                        .iter()
                        .any(|(_, r)| (r - first).abs() > 1e-12 * (1.0 + first.abs()))
                })
            });
            if chain_corner && tapered && !allow_tapered_sequential {
                // The transition patch that would carry a taper through the
                // corner is not built: a named deferral.
                return Err(KernelRefusal::unsupported(
                    KernelStage::Sew,
                    "tapered_chain_miter",
                    format!(
                        "{entry}: the tapered blends across the selected chain's shared corners \
                         did not compose to a valid solid (the fitted blend boundaries could not \
                         be mitered); fillet fewer edges per operation or reduce the taper"
                    ),
                ));
            }
            let mut sequential = solid.clone();
            for (index, point) in edge_points.iter().enumerate() {
                let edge_id = resolve_edge_by_point(&sequential, *point)?;
                let edge_name = per_edge_name(edge_names, name, index);
                sequential = crate::blend::blend_edge_variable(
                    &sequential,
                    edge_id,
                    &per_edge_stops[index],
                    chamfer,
                    edge_name,
                )?;
            }
            sequential
        }
    };
    // Heal §6.9 surgery so re-trimmed edges meet their vertices exactly; scale
    // the heal bound by the largest radius stop in the taper profile.
    heal_edge_vertex_gaps(&mut result, max_radius)?;
    crate::blend::fit_planar_charts_to_trims(solid, &mut result)?;
    // Final honesty gate: a genuinely TAPERED chain (different radii at the
    // shared corner) has mismatched trim stations there — the blends cannot
    // meet without a transition patch (not implemented), and the sequential
    // surgery silently left broken topology before this gate existed.
    let issues = result.validate();
    if !issues.is_empty() {
        // Two verdicts share the text's shape: the legacy entry names the
        // transition patch it does not build (a deferral); the law entries,
        // whose taper is continuous through the corner, report the
        // composition failing its own validation (a check that tripped).
        let (detail, class) = if allow_tapered_sequential {
            (
                "tapered blends meet at a shared chain vertex with \
                 mismatched radii — the radius-transition corner patch is not implemented; \
                 fillet the edges in separate operations or use matching stop radii",
                RefusalClass::UnsupportedGeometry {
                    what: "taper_transition_patch".into(),
                },
            )
        } else {
            (
                "the composed radius-law blend produced invalid topology — the blends \
                 across a shared chain corner failed to reassemble",
                RefusalClass::Internal {
                    what: "law_chain_topology".into(),
                },
            )
        };
        return Err(KernelRefusal::new(
            class,
            KernelStage::Validate,
            format!(
                "{entry}: {detail} ({} validation issues, first: {})",
                issues.len(),
                issues
                    .first()
                    .map(|issue| issue.message.clone())
                    .unwrap_or_default()
            ),
        ));
    }
    // The variable-radius lane's own soundness acceptance, the same one the
    // constant-radius group exit takes. `validate()` above is an INCIDENCE
    // test: a tapered wall that passes through a neighbour, or through itself,
    // satisfies every check it makes and still measures a volume.
    crate::accept_sound(result, entry)
}

/// One selected edge placed on the ordered chain, with its arc-length
/// parameterization (the caller-side mapping from chain abscissa to the
/// `blend_edge_variable` per-edge parameter-fraction stop seam).
struct ChainLink {
    /// Position of this edge in the caller's `edge_points` selection.
    input_index: usize,
    /// True when the edge's own t0→t1 parameter direction runs WITH the chain.
    forward: bool,
    /// Cumulative chord-length table from the edge's t0 end:
    /// `(parameter fraction, arc length)`, uniformly spaced in fraction.
    arc: Vec<(f64, f64)>,
    /// Total arc length of the edge.
    length: f64,
    /// Chain abscissa at the link's ENTRY vertex (the end reached first when
    /// walking the chain from its start).
    abscissa: f64,
}

/// Cumulative chord-length table of one edge, `(parameter fraction, arc
/// length)` at uniform fractions.  The sample count doubles until two
/// successive total-length estimates agree within `tol` (chord length
/// converges O(N⁻²) for smooth curves, so the agreement of the N and 2N
/// estimates bounds the remaining error at the same order); straight edges
/// converge on the first doubling.  The 16-sample start resolves any
/// single-span arc of up to half a turn to sub-percent before refinement; the
/// 4096 cap (8 doublings) guards adversarial curves — beyond it the table is
/// two orders denser than the blend march's station grid, so finer chords
/// cannot move any station's sampled radius meaningfully.
fn edge_arc_table(
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    tol: f64,
) -> Result<(Vec<(f64, f64)>, f64), KernelRefusal> {
    let evaluate = |t: f64| curve.evaluate(t).or_refuse(KernelStage::Collect, "arc_table");
    let build = |n: usize| -> Result<(Vec<(f64, f64)>, f64), KernelRefusal> {
        let mut table = Vec::with_capacity(n + 1);
        let mut cumulative = 0.0;
        let mut previous = evaluate(t0)?;
        table.push((0.0, 0.0));
        for j in 1..=n {
            let fraction = j as f64 / n as f64;
            let point = evaluate(t0 + (t1 - t0) * fraction)?;
            cumulative += point.sub(previous).length();
            previous = point;
            table.push((fraction, cumulative));
        }
        Ok((table, cumulative))
    };
    let mut n = 16usize;
    let (mut table, mut length) = build(n)?;
    while n < 4096 {
        n *= 2;
        let (next_table, next_length) = build(n)?;
        let converged = (next_length - length).abs() <= tol;
        table = next_table;
        length = next_length;
        if converged {
            break;
        }
    }
    Ok((table, length))
}

/// Arc length from the edge's t0 end at `fraction` of its parameter span,
/// linearly interpolated in the uniform chord table.
fn arc_length_at_fraction(table: &[(f64, f64)], fraction: f64) -> f64 {
    let fraction = fraction.clamp(0.0, 1.0);
    let intervals = table.len() - 1;
    let scaled = fraction * intervals as f64;
    let index = (scaled.floor() as usize).min(intervals - 1);
    let local = scaled - index as f64;
    let (_, a) = table[index];
    let (_, b) = table[index + 1];
    a + (b - a) * local
}

/// Resolve the selected edges and order them into ONE OPEN CHAIN with
/// cumulative arc-length abscissas.  The chain starts at the free endpoint
/// belonging to the EARLIEST-selected end edge (so users get the natural
/// "first pick carries the law start" orientation); a single selected edge is
/// its own chain oriented t0→t1 (which also admits a closed edge — the law's
/// end radii must then match, enforced downstream by `blend_edge_variable`).
/// Branching (a vertex shared by 3+ selected edges), closed rings of several
/// edges, and disconnected selections refuse with named errors.
fn resolve_selected_chain(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    entry: &str,
    tol: f64,
) -> Result<Vec<ChainLink>, KernelRefusal> {
    // Endpoint identity uses the kernel-wide COINCIDENCE_DISTANCE_FLOOR — the
    // same band the constant-radius group's corner detector applies.
    let band = crate::tolerance::COINCIDENCE_DISTANCE_FLOOR;

    struct Resolved {
        edge_id: u64,
        start: Vec3,
        end: Vec3,
        arc: Vec<(f64, f64)>,
        length: f64,
    }
    let mut resolved: Vec<Resolved> = Vec::with_capacity(edge_points.len());
    for point in edge_points {
        let edge_id = resolve_edge_by_point(solid, *point)?;
        if resolved.iter().any(|r| r.edge_id == edge_id) {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "duplicate_edge",
                format!("{entry}: the same edge was selected more than once"),
            ));
        }
        let edge = solid
            .edges
            .iter()
            .find(|e| e.id == edge_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "chain_resolved_edge",
                    format!("{entry}: resolved edge vanished"),
                )
            })?;
        let (arc, length) = edge_arc_table(&edge.curve, edge.t0, edge.t1, tol)?;
        if !(length > 0.0) {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "zero_length_edge",
                format!("{entry}: selected edge has zero length"),
            ));
        }
        resolved.push(Resolved {
            edge_id,
            start: edge
                .curve
                .evaluate(edge.t0)
                .or_refuse(KernelStage::Collect, "chain_edge_start")?,
            end: edge
                .curve
                .evaluate(edge.t1)
                .or_refuse(KernelStage::Collect, "chain_edge_end")?,
            arc,
            length,
        });
    }
    let n = resolved.len();
    if n == 1 {
        let only = resolved.remove(0);
        return Ok(vec![ChainLink {
            input_index: 0,
            forward: true,
            arc: only.arc,
            length: only.length,
            abscissa: 0.0,
        }]);
    }

    // Cluster the 2n endpoints; each entry is (edge index, is_start_end).
    let mut clusters: Vec<(Vec3, Vec<(usize, bool)>)> = Vec::new();
    for (i, r) in resolved.iter().enumerate() {
        for (point, is_start) in [(r.start, true), (r.end, false)] {
            match clusters
                .iter_mut()
                .find(|(anchor, _)| anchor.sub(point).length() < band)
            {
                Some((_, members)) => members.push((i, is_start)),
                None => clusters.push((point, vec![(i, is_start)])),
            }
        }
    }
    if clusters.iter().any(|(_, members)| members.len() > 2) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "chain_branching",
            format!(
                "{entry}: selected edges must form one open chain \
                 (a vertex is shared by three or more selected edges)"
            ),
        ));
    }
    let free: Vec<usize> = clusters
        .iter()
        .enumerate()
        .filter(|(_, (_, members))| members.len() == 1)
        .map(|(c, _)| c)
        .collect();
    if free.len() != 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "chain_open",
            format!(
                "{entry}: selected edges must form one OPEN chain \
                 (closed rings and disconnected selections are not supported)"
            ),
        ));
    }
    // Start at the free end whose edge appears EARLIEST in the selection.
    let start_cluster = *free
        .iter()
        .min_by_key(|&&c| clusters[c].1[0].0)
        .expect("two free ends");

    // Walk the chain.
    let mut links: Vec<ChainLink> = Vec::with_capacity(n);
    let mut visited = vec![false; n];
    let mut abscissa = 0.0_f64;
    let mut cluster = start_cluster;
    for _ in 0..n {
        let Some(&(edge_index, entered_at_start)) = clusters[cluster]
            .1
            .iter()
            .find(|(edge_index, _)| !visited[*edge_index])
        else {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "chain_walk",
                format!(
                    "{entry}: selected edges are not connected into one chain"
                ),
            ));
        };
        visited[edge_index] = true;
        let r = &resolved[edge_index];
        links.push(ChainLink {
            input_index: edge_index,
            forward: entered_at_start,
            arc: r.arc.clone(),
            length: r.length,
            abscissa,
        });
        abscissa += r.length;
        let exit_point = if entered_at_start { r.end } else { r.start };
        cluster = clusters
            .iter()
            .position(|(anchor, _)| anchor.sub(exit_point).length() < band)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Collect,
                    "chain_cluster",
                    format!("{entry}: chain walk lost an endpoint cluster"),
                )
            })?;
    }
    if visited.iter().any(|v| !v) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "chain_connected",
            format!(
                "{entry}: selected edges are not connected into one chain"
            ),
        ));
    }
    Ok(links)
}

/// Sample the law into one edge's `(parameter fraction, radius)` stop list —
/// the caller-side bridge from chain abscissa into the `radius_at` closure
/// seam that `blend_edge_variable` builds over its stops.
///
/// `scale` maps chain abscissa into the law's own abscissa units
/// (`law.total_length() / chain_length` — proportional, so a law built from
/// the measured chain lengths maps 1:1).  The base stop count derives from
/// the law's curvature: piecewise-linear sampling of a C1, piecewise-C2
/// function over step `h` errs at most `h²·max|r''|/8`, so
/// `h = sqrt(8·tol/max|r''|)` holds the sampling error under the SSI fit
/// tolerance the variable lane is built to.  That bound is exact for
/// arc-length-linear (straight) edges; curved edges bend the
/// parameter→abscissa map, so each interval is additionally midpoint-checked
/// against the law and bisected on violation (up to 8 halvings — a 4⁸ ≈ 6·10⁴
/// error reduction, decisive for any C1 law).  The base count is capped at
/// 256 intervals: 4× the blend march's 64-station density
/// (blend/stations.rs), beyond which denser stops cannot move any station's
/// sampled radius by more than the fit tolerance.
fn law_stops_for_link(
    link: &ChainLink,
    law: &crate::law::RadiusLaw,
    scale: f64,
    tol: f64,
) -> Vec<(f64, f64)> {
    let radius_at_fraction = |fraction: f64| -> f64 {
        let arc = arc_length_at_fraction(&link.arc, fraction);
        let chain_s = if link.forward {
            link.abscissa + arc
        } else {
            link.abscissa + (link.length - arc)
        };
        law.radius_at(chain_s * scale)
    };
    // Curvature of the law in edge-fraction units: d²r/df² ≤
    // max|r''|·(scale·length)² for the arc-length-linear map.
    let curvature = law
        .max_second_derivative(link.abscissa * scale, (link.abscissa + link.length) * scale)
        * (scale * link.length).powi(2);
    let base = if curvature * 0.125 <= tol {
        1usize
    } else {
        ((curvature / (8.0 * tol)).sqrt().ceil() as usize).clamp(1, 256)
    };
    let mut stops: Vec<(f64, f64)> = (0..=base)
        .map(|j| {
            let fraction = j as f64 / base as f64;
            (fraction, radius_at_fraction(fraction))
        })
        .collect();
    // Midpoint refinement for curved parameter→abscissa maps.
    let mut depth = 0usize;
    while depth < 8 {
        let mut refined: Vec<(f64, f64)> = Vec::with_capacity(stops.len());
        let mut inserted = false;
        for pair in stops.windows(2) {
            refined.push(pair[0]);
            let mid_fraction = 0.5 * (pair[0].0 + pair[1].0);
            let law_mid = radius_at_fraction(mid_fraction);
            let linear_mid = 0.5 * (pair[0].1 + pair[1].1);
            if (law_mid - linear_mid).abs() > tol {
                refined.push((mid_fraction, law_mid));
                inserted = true;
            }
        }
        refined.push(*stops.last().expect("at least two stops"));
        stops = refined;
        if !inserted {
            break;
        }
        depth += 1;
    }
    stops
}

/// Fillet (or chamfer) a chain of edges under a composable radius law
/// evaluated on the chain's cumulative arc-length abscissa (the OCCT
/// `Law_Composite` model; see [`crate::law::RadiusLaw`]).  The selected edges
/// must form ONE OPEN CHAIN (or be a single edge); the chain starts at the
/// free end of the earliest-selected end edge, and the law's abscissa maps
/// proportionally onto the chain's measured arc length (a law built with the
/// chain's own lengths — e.g. [`crate::law::RadiusLaw::from_vertex_radii`] —
/// maps 1:1).  Endpoint radii are met exactly; radii at shared chain vertices
/// match by the law's continuity, so the per-edge blends miter through the
/// corners; a chain whose blends cannot be mitered REFUSES rather than
/// distorting the law through the sequential rebuild.
pub fn fillet_edges_variable_law(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    law: &crate::law::RadiusLaw,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    const ENTRY: &str = "fillet_edges_variable_law";
    if edge_points.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "law_selection",
            format!("{ENTRY}: no edges selected"),
        ));
    }
    let tolerances = crate::KernelTolerances::for_solid(solid, 1e-7);
    let chain = resolve_selected_chain(solid, edge_points, ENTRY, tolerances.intersection_fit)?;
    fillet_variable_law_on_chain(
        solid,
        edge_points,
        edge_names,
        &chain,
        law,
        chamfer,
        name,
        ENTRY,
        tolerances.intersection_fit,
    )
}

/// The natural per-vertex user model: radius `vertex_radii[i]` at chain
/// vertex `i` (in CHAIN order, starting at the free end of the
/// earliest-selected edge), smoothly interpolated along the chain
/// (monotone C1 — every vertex radius met exactly, no overshoot).  Requires
/// exactly one radius per chain vertex (`edges + 1`).
pub fn fillet_edges_variable_vertex_radii(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    vertex_radii: &[f64],
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    const ENTRY: &str = "fillet_edges_variable_vertex_radii";
    if edge_points.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "vertex_radii_selection",
            format!("{ENTRY}: no edges selected"),
        ));
    }
    let tolerances = crate::KernelTolerances::for_solid(solid, 1e-7);
    let chain = resolve_selected_chain(solid, edge_points, ENTRY, tolerances.intersection_fit)?;
    let lengths: Vec<f64> = chain.iter().map(|link| link.length).collect();
    let law = crate::law::RadiusLaw::from_vertex_radii(&lengths, vertex_radii)
        .map_err(|error| error.with_message(|error| format!("{ENTRY}: {error}")))?;
    fillet_variable_law_on_chain(
        solid,
        edge_points,
        edge_names,
        &chain,
        &law,
        chamfer,
        name,
        ENTRY,
        tolerances.intersection_fit,
    )
}

/// Shared law-entry tail: sample per-edge stop lists from the law over the
/// resolved chain and run the variable group core (miter-or-refuse: no
/// sequential fallback for tapered chains — see `fillet_edges_variable_impl`).
#[allow(clippy::too_many_arguments)]
fn fillet_variable_law_on_chain(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    chain: &[ChainLink],
    law: &crate::law::RadiusLaw,
    chamfer: bool,
    name: Option<&str>,
    entry: &str,
    tol: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let chain_length: f64 = chain.iter().map(|link| link.length).sum();
    let scale = law.total_length() / chain_length;
    let mut per_edge_stops: Vec<Vec<(f64, f64)>> = vec![Vec::new(); edge_points.len()];
    for link in chain {
        per_edge_stops[link.input_index] = law_stops_for_link(link, law, scale, tol);
    }
    fillet_edges_variable_impl(
        solid,
        edge_points,
        edge_names,
        &per_edge_stops,
        chamfer,
        name,
        entry,
        false,
    )
}

/// Asymmetric (two-distance) chamfer of a GROUP of edges, the app entry: each
/// selected edge (resolved by a point on it) gets a `d1 × d2` bevel (§6.11).
/// Edges are chamfered independently — asymmetric chamfers keep sharp vertices,
/// so there is no shared-corner blending.
pub fn chamfer_edges_asymmetric(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    d1: f64,
    d2: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    if edge_points.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "asymmetric_selection",
            "chamfer_edges_asymmetric: no edges selected",
        ));
    }
    let mut result = solid.clone();
    // `edge_names[index]` is keyed by INPUT position; the loop resolves each
    // point on the EVOLVING `result`, but enumerates the input points in order,
    // so the index alignment holds.
    for (index, point) in edge_points.iter().enumerate() {
        let edge_id = resolve_edge_by_point(&result, *point)?;
        let edge_name = per_edge_name(edge_names, name, index);
        result = chamfer_edge_asymmetric(&result, edge_id, d1, d2, edge_name)?;
    }
    Ok(result)
}

/// Distance-angle chamfer of a GROUP of edges, the app entry: each selected
/// edge (resolved by a point on it) gets a setback `d1` on face 1 and a chamfer
/// face at `angle_rad` from face 1 (§6.11); `d2` is constructed per edge.
pub fn chamfer_edges_angle(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    d1: f64,
    angle_rad: f64,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // One blend operation: the reports of an earlier one, never consumed, are
    // dropped as it starts (nested calls keep this operation's).
    let _operation = crate::blend::BlendOperation::enter();
    if edge_points.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "angle_selection",
            "chamfer_edges_angle: no edges selected",
        ));
    }
    let mut result = solid.clone();
    for (index, point) in edge_points.iter().enumerate() {
        let edge_id = resolve_edge_by_point(&result, *point)?;
        let edge_name = per_edge_name(edge_names, name, index);
        result = chamfer_edge_angle(&result, edge_id, d1, angle_rad, edge_name)?;
    }
    Ok(result)
}

/// Debug aid (`BREP_DEBUG_NETWORK`): every face loop as its coedge walk, with
/// each edge's stored vertices, so a loop the surgery left open can be read
/// against the input it was built from.
fn dump_loops_debug(label: &str, solid: &BrepSolid) {
    for shell in &solid.shells {
        for face in &shell.faces {
            for (index, loop_record) in face.loops.iter().enumerate() {
                let walk: Vec<String> = loop_record
                    .coedges
                    .iter()
                    .map(|coedge| {
                        match solid.edges.iter().find(|e| e.id == coedge.edge_id) {
                            Some(edge) => format!(
                                "c{}:e{}{}({}->{})",
                                coedge.id,
                                edge.id,
                                if coedge.forward { "+" } else { "-" },
                                edge.start_vertex_id,
                                edge.end_vertex_id
                            ),
                            None => format!("c{}:e{}?MISSING", coedge.id, coedge.edge_id),
                        }
                    })
                    .collect();
                eprintln!(
                    "{label} face {} ({:?}) loop {index}: {}",
                    face.id,
                    face.name,
                    walk.join(" ")
                );
            }
        }
    }
}
