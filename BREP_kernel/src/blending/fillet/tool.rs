use super::*;

/// Cross-section of the removed (convex) or added (concave) material in
/// the plane perpendicular to the edge: corner at the edge, support
/// tangency points at r/tan(θ/2) along each face, and the rolling-ball
/// arc (fillet) or its chord (chamfer) between them.
fn cross_section_profile(
    cross: &EdgeCross,
    radius: f64,
    chamfer: bool,
) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    let corner = cross.start;
    let cos_theta = cross.into_first.dot(cross.into_second).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();
    if theta <= 1e-6 || theta >= std::f64::consts::PI - 1e-6 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "dihedral_degenerate",
            "fillet: dihedral angle too degenerate",
        ));
    }
    let tangent_offset = radius / (theta * 0.5).tan();
    let first_tangency = corner.add(cross.into_first.scale(tangent_offset));
    let second_tangency = corner.add(cross.into_second.scale(tangent_offset));
    let line = |from: Vec3, to: Vec3| {
        make_line(from, to).or_refuse(KernelStage::Fragment, "profile_line")
    };
    let mut profile = vec![line(corner, first_tangency)?];
    if chamfer {
        profile.push(line(first_tangency, second_tangency)?);
    } else {
        // Ball center: along the corner bisector at distance r/sin(θ/2);
        // the arc runs from the first tangency to the second, curving
        // toward the corner.
        let bisector = cross
            .into_first
            .add(cross.into_second)
            .normalized()
            .or_refuse(KernelStage::Fragment, "profile_bisector")?;
        let center = corner.add(bisector.scale(radius / (theta * 0.5).sin()));
        let x_axis = first_tangency.sub(center).scale(1.0 / radius);
        let sweep = std::f64::consts::PI - theta;
        // y axis chosen so the arc sweeps from the first tangency to the
        // second inside the corner wedge.
        let y_axis_raw = second_tangency.sub(center).scale(1.0 / radius);
        let y_axis = y_axis_raw
            .sub(x_axis.scale(y_axis_raw.dot(x_axis)))
            .normalized()
            .or_refuse(KernelStage::Fragment, "profile_arc_axis")?;
        profile.push(
            make_arc(center, x_axis, y_axis, radius, 0.0, sweep)
                .or_refuse(KernelStage::Fragment, "profile_arc")?,
        );
    }
    profile.push(line(second_tangency, corner)?);
    Ok(profile)
}

/// Asymmetric chamfer cross-section (Golovanov §6.11): the removed triangular
/// sliver has EXPLICIT per-face setbacks — the standard CAD "d1 × d2"
/// convention, `d1` along face 1 (`into_first`) and `d2` along face 2
/// (`into_second`).  Curve order matches `cross_section_profile`: the corner
/// leg on face 1, the chamfer chord (blend wall, index 1), then the face-2 leg
/// back to the corner.
pub(super) fn chamfer_cross_section_offsets(
    cross: &EdgeCross,
    d1: f64,
    d2: f64,
) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    let o = cross.start;
    let p1 = o.add(cross.into_first.scale(d1));
    let p2 = o.add(cross.into_second.scale(d2));
    let line = |from: Vec3, to: Vec3| {
        make_line(from, to).or_refuse(KernelStage::Fragment, "chamfer_profile_line")
    };
    Ok(vec![line(o, p1)?, line(p1, p2)?, line(p2, o)?])
}

/// Given setback `d1` on face 1 and the angle α between the chamfer face and
/// face 1, CONSTRUCT the face-2 setback `d2` in the cross-section plane
/// (§6.11 distance-angle chamfer).  We do not assume a closed form: an
/// orthonormal 2D basis `(e1, e2)` is built in the cross-section plane, the
/// chamfer chord ray leaving `P1 = d1·e1` at angle α from face 1 (tilting
/// toward face 2) is intersected with the face-2 ray `t·into_second`, and
/// `d2 = t`.
pub(super) fn chamfer_angle_second_distance(
    cross: &EdgeCross,
    d1: f64,
    angle_rad: f64,
) -> Result<f64, KernelRefusal> {
    let e1 = cross.into_first;
    let cos_theta = cross.into_first.dot(cross.into_second).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();
    if theta <= 1e-6 || theta >= std::f64::consts::PI - 1e-6 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "angle_dihedral_degenerate",
            "chamfer_edge_angle: dihedral angle too degenerate",
        ));
    }
    if !(angle_rad > 1e-9) || angle_rad >= theta - 1e-9 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "chamfer_angle",
            format!(
                "chamfer_edge_angle: angle {angle_rad} must lie in (0, dihedral θ={theta})"
            ),
        ));
    }
    // Orthonormal basis of the cross-section plane: e1 = into_first, e2 the
    // normalized in-plane perpendicular pointing toward face 2.
    let e2 = cross
        .into_second
        .sub(e1.scale(cross.into_second.dot(e1)))
        .normalized()
        .or_refuse(KernelStage::Classify, "chamfer_basis")?;
    // Project face 2 into (e1, e2): (cosθ, sinθ), sinθ > 0 by construction.
    let f2x = cross.into_second.dot(e1);
    let f2y = cross.into_second.dot(e2);
    // P1 = (d1, 0).  The chamfer chord ray leaves P1 making angle α with face 1
    // and tilting toward face 2, so its 2D direction is (−cosα, sinα).  Intersect
    // P1 + s·(−cosα, sinα) = t·(f2x, f2y), t ≥ 0 (the face-2 ray) ⇒ d2 = t.
    let (ca, sa) = (angle_rad.cos(), angle_rad.sin());
    // [ ca   f2x ] [s]   [d1]      (from  d1 − s·cosα = t·f2x)
    // [ sa  −f2y ] [t] = [ 0]      (from  s·sinα      = t·f2y)
    let det = ca * (-f2y) - f2x * sa;
    // Both remaining refusals sit BEHIND the angle check above, which
    // bounds α strictly inside (0, θ): a parallel ray or a non-positive
    // setback is the construction contradicting its own precondition.
    if det.abs() < 1e-12 {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "chamfer_ray_parallel",
            "chamfer_edge_angle: chamfer ray is parallel to face 2",
        ));
    }
    // Cramer's rule for t (the face-2 setback d2).
    let t = (ca * 0.0 - sa * d1) / det;
    if !(t > 0.0) || !t.is_finite() {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "chamfer_setback",
            format!(
                "chamfer_edge_angle: constructed a non-positive setback d2={t}"
            ),
        ));
    }
    Ok(t)
}

fn tool_boolean_setup(convex: bool) -> (BooleanOperation, BooleanOptions) {
    let operation = if convex {
        BooleanOperation::Subtract
    } else {
        BooleanOperation::Union
    };
    // The tool's side walls lie ON the mating faces (cosurface); the
    // post-boolean same-surface merge currently mishandles the resulting
    // ring topology, so the blend boolean runs without it — callers can
    // merge coplanar faces afterwards if they wish.
    let options = BooleanOptions {
        merge_coplanar_faces: false,
        ..BooleanOptions::default()
    };
    (operation, options)
}

pub(super) fn apply_tool(
    solid: &BrepSolid,
    tool: &BrepSolid,
    convex: bool,
) -> Result<BrepSolid, KernelRefusal> {
    let (operation, options) = tool_boolean_setup(convex);
    boolean_operation(solid, tool, operation, &options)
}

/// Assemble the blend boolean with the EXACT arrangement only — no
/// Simulation-of-Simplicity fallback.
///
/// `boolean_operation` rescues an arrangement degeneracy by rigidly
/// TRANSLATING the second operand (here: the cutter) by a fraction of the model
/// scale and accepting the perturbed result once a point-classification oracle
/// and a 1%-slack volume bound agree.  Those gates are far too coarse to notice
/// that the blend wall has moved off its supports: the cutter is translated
/// bodily, so the fillet surface is no longer tangent to either mating face and
/// the removed sliver is wrong by ~1% of itself.  A blend has an exact
/// construction available (see the overshoot ladder in
/// `fillet_or_chamfer_exact`), so it must exhaust that ladder before it is
/// allowed to settle for a perturbed cutter.
pub(super) fn apply_tool_exact(
    solid: &BrepSolid,
    tool: &BrepSolid,
    convex: bool,
) -> Result<BrepSolid, KernelRefusal> {
    let (operation, options) = tool_boolean_setup(convex);
    Ok(crate::boolean_operation_with_diagnostics(solid, tool, operation, &options)?.value)
}

/// Which construction gets the first turn on one edge.
///
/// The public single-edge entries construct the blend by the march (the
/// rolling ball, the fitted rows, direct surgery) and fall back to the cutter
/// only where the march refuses.  The GROUP's sequential composition keeps
/// the cutter first: its corner closures (`round_convex_corner` and the
/// mixed-convexity lanes) reconstruct the corner from the cutter's output
/// and read its topology, so until the stripe network covers those corners
/// the composition they sit on must stay the one they were written against.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum Lane {
    GeneralFirst,
    CutterFirst,
}

pub(super) fn fillet_or_chamfer(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    ends: ToolEnds,
    lane: Lane,
) -> Result<BrepSolid, KernelRefusal> {
    // Fuse-first operand heal (Lever A) BEFORE the §6.9 surgery, mirroring the
    // output heal below: snap the input's near-coincident / off-plane vertices
    // to exact and re-anchor incident edges, so amplified solver/intersection
    // noise cannot break the blend boolean.  Edge identity (ids) is preserved,
    // so `edge_id` still resolves; a clean input is left byte-identical.
    let mut healed = solid.clone();
    let policy = crate::KernelTolerances::for_solid(&healed, 1e-7);
    crate::heal::heal_operands(&mut healed, &policy)?;
    let mut result = fillet_or_chamfer_inner(&healed, edge_id, radius, chamfer, name, ends, lane)?;
    // Heal the §6.9 surgery so re-trimmed edges meet their corner vertices
    // exactly (the boolean-backed exact path is already committed; this is a
    // no-op there and on any clean blend).
    heal_edge_vertex_gaps(&mut result, radius)?;
    // A trim the §6.9 surgery ran outside a planar carrier's chart is clamped
    // onto the chart's boundary by the pcurve builder, which leaves a wrong
    // region on a body `validate()` passes; widen the carrier to hold it.
    crate::blend::fit_planar_charts_to_trims(&healed, &mut result)?;
    // A blend with no corner closure after it is finished here. A cutter-first
    // composition's cutter keeps its standing scaffold tagged until the group
    // has closed its corners, which read that topology (`settle_cutter_scaffold`).
    if lane == Lane::GeneralFirst {
        result = settle_cutter_scaffold(result);
    }
    Ok(result)
}

/// Is this march refusal about the SHAPE rather than the lane?
///
/// A wall that folds through itself is not something another construction
/// answers: the cutter's tool surface is the same envelope with the same fold,
/// and the edge-preserving fallback builds a blend the rolling ball never made.
/// Reported as-is, so the caller reads the geometry instead of whichever later
/// rung complained last.
///
/// A marched seam or connector whose fit left its carriers is the same kind of
/// refusal for the same reason.  It does not say "not this lane": it says the
/// curve the corner was closed with is not the curve the march solved for,
/// read against the march's own witnesses, and no later rung re-asks that
/// question.  The lane beneath it answers a DIFFERENT question and answers it
/// wrong without saying so -- on the notched cap's stuck connector the cutter
/// composition reads 1724.506048919, +1.81e-2 against the closed form, and it
/// validates.  A refusal that falls through to a validating wrong solid is not
/// a refusal.
///
/// So is a blend that stops within a sliver of its face's width and cannot be
/// snapped onto the far edge without opening the shell
/// (`blend::CONSUMED_SNAP_UNSOUND`): the size is the question, and a tool solid
/// cut a sliver short of an edge answers it with a sliver no lane keeps.
///
/// So is a trim that left its planar carrier's chart
/// (`blend::PLANAR_CHART_EDGE_OFF_PLANE`, `blend::PLANAR_CHART_WIDEN_UNSOUND`):
/// the clamped body another lane would answer with is the defect itself.
pub(super) fn is_terminal_blend_refusal(refusal: &KernelRefusal) -> bool {
    crate::blend::is_wall_fold(refusal)
        || crate::blend::is_marched_fit_off_carriers(refusal)
        || crate::blend::is_consumed_snap(refusal)
        || crate::blend::is_planar_chart_refusal(refusal)
}

fn fillet_or_chamfer_inner(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    ends: ToolEnds,
    lane: Lane,
) -> Result<BrepSolid, KernelRefusal> {
    // The general §4.9 march with direct §6.9 surgery is the PRIMARY lane:
    // it constructs the blend from the rolling ball and re-trims the mates,
    // with no tool solid to run past the edge.  A straight edge between two
    // planes comes out as the exact cylinder patch (`exact_extrusion_rows`),
    // a circular rim on straight-meridian carriers as the exact revolution
    // (`exact_closed_revolution_rows`), so nothing the cutter produced
    // exactly is lost.  The cutter remains only as the fallback for what the
    // march still refuses; a pad on its ends (`ends`) is a group-build
    // instruction that only the cutter understands.
    let closed = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .map(|edge| edge.start_vertex_id == edge.end_vertex_id)
        .unwrap_or(false);
    // `BREP_CUTTER_FIRST=1` restores the pre-network order for A/B
    // comparison; the general lane is otherwise first.
    // The three lane notes below are not refusals of the shape: they say the
    // march was not asked, and only ever reach a caller inside the ladder's
    // combined text ("first attempt: …").  Classed as the deferral they are.
    let lane_note = |what: &'static str, message: &str| {
        KernelRefusal::unsupported(KernelStage::Classify, what, message)
    };
    let general = if ends != ToolEnds::default() {
        Err(lane_note("padded_cutter", "a padded cutter was requested"))
    } else if lane == Lane::CutterFirst {
        Err(lane_note(
            "cutter_first",
            "the cutter has the first turn in this composition",
        ))
    } else if std::env::var("BREP_CUTTER_FIRST").is_ok() {
        Err(lane_note("cutter_first_env", "BREP_CUTTER_FIRST set"))
    } else if closed {
        crate::blend::blend_closed_edge(solid, edge_id, radius, chamfer, name)
    } else {
        // An open edge may be one tangent-continuous arc of a CLOSED smooth
        // chain (e.g. a cyl×cyl saddle split into arcs by the two carriers'
        // seams) — blend the whole rim as one unit (§6.9.5).  An OPEN tangent
        // chain is NOT walked: the network treats the edge as one stripe and
        // caps it where it runs into an unselected tangent neighbour.
        crate::blend::blend_smooth_chain_if_closed(solid, edge_id, radius, chamfer, name)
            .or_else(|error| {
                if is_terminal_blend_refusal(&error) {
                    return Err(error);
                }
                let names = [name.map(str::to_string)];
                crate::blend::blend_star_network(solid, &[edge_id], radius, chamfer, &names, &|_| None)
            })
            .or_else(|error| {
                if is_terminal_blend_refusal(&error) {
                    return Err(error);
                }
                crate::blend::blend_open_edge(solid, edge_id, radius, chamfer, name)
            })
    };
    // A general-lane result is accepted only when it is BOTH a valid solid and
    // a trimmed one: `check_blend_interference` catches the blend wall left
    // running through a third face, which `validate` cannot see.  Either way
    // the ladder carries on to the cutter, whose boolean does split the
    // crossing faces and drop the fragments in the void.
    let entry = if chamfer { "chamfer" } else { "fillet" };
    let general_error = match general {
        Ok(result) if result.validate().is_empty() => {
            match check_blend_interference(solid, &result, &[edge_id], entry) {
                Ok(()) => return Ok(result),
                Err(untrimmed) => untrimmed,
            }
        }
        Ok(_) => KernelRefusal::internal(
            KernelStage::Validate,
            "general_invalid",
            "the general blend assembled an invalid solid",
        ),
        Err(error) => error,
    };
    // The rolling ball does not FIT: TERMINAL, like the mixed-convexity corner
    // (`fillet/edges.rs`).  The march refused because the tangency system has
    // no solution at this radius — the local wall is thinner than the ball —
    // and a tool solid does not answer that question, it only removes whatever
    // its own surface encloses.  Reporting the geometric refusal beats leading
    // the user with the cutter's unrelated complaint about the edge's curve.
    if crate::blend::is_ball_off_carrier(&general_error) {
        return Err(general_error);
    }
    // The wall the rolling ball sweeps FOLDS THROUGH ITSELF: terminal for the
    // same reason, and a step further.  The march converged — every station is
    // on the true centre curve — but the curve turns tighter there than the
    // ball is wide, so the envelope's own section sweeps back through itself
    // and no lane in the kernel trims that.  The cutter builds the identical
    // self-intersecting surface, and its boolean then hides the defect inside a
    // solid that validates; the fold is only visible to the tessellation-level
    // scan (`brep/soundness.rs`).  Reporting the geometry beats shipping it.
    if is_terminal_blend_refusal(&general_error) {
        return Err(general_error);
    }
    match fillet_or_chamfer_exact(solid, edge_id, radius, chamfer, name, ends, lane) {
        Ok(result) => Ok(result),
        Err(exact_error) => {
            // The exact special cases (straight×planar extrude, circular×
            // straight-meridian rotational revolve) cover a subset; every
            // other edge goes through the general §4.9 tangency march with
            // direct §6.9 surgery.
            let general = if closed {
                crate::blend::blend_closed_edge(solid, edge_id, radius, chamfer, name)
            } else {
                crate::blend::blend_smooth_chain(solid, edge_id, radius, chamfer, name)
                    .or_else(|error| {
                        if is_terminal_blend_refusal(&error) {
                            return Err(error);
                        }
                        crate::blend::blend_open_edge(solid, edge_id, radius, chamfer, name)
                    })
            };
            // Last rung of the ladder, and the same contract: an untrimmed
            // blend is a failure here too.  Nothing is left to fall through
            // to, so this one reaches the caller as a named refusal rather
            // than as a self-intersecting solid.
            general
                .and_then(|result| {
                    check_blend_interference(solid, &result, &[edge_id], entry).map(|()| result)
                })
                .map_err(|error| {
                    // Reachable with a fold only in the cutter-first
                    // compositions, where the first attempt never marched: the
                    // refusal is still the shape's, so it is reported alone
                    // rather than wrapped in the cutter's complaint.
                    if is_terminal_blend_refusal(&error) {
                        return error;
                    }
                    // The last rung's own class survives the wrap.
                    error.with_message(|error| {
                        format!(
                            "exact cutter failed: {exact_error}; general blend also failed: {error} \
                             (first attempt: {general_error})"
                        )
                    })
                })
        }
    }
}

/// How far the cutter must run PAST each end of the edge it is built on,
/// measured as arc length along the edge path.  Non-zero only when a
/// neighbouring blend has already TRIMMED this edge back from a shared corner
/// (see `tool_ends_to_original_extent`); a lone blend keeps both ends flush.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub(super) struct ToolEnds {
    start: f64,
    end: f64,
}

/// Build the fillet/chamfer cutter for one edge.  `overshoot` extends a
/// STRAIGHT edge's extruded tool by that much PAST each edge endpoint, so the
/// tool's end caps clear the adjacent end faces instead of sitting coincident
/// on them.  This is what lets the boolean cut the end faces TRANSVERSALLY
/// (one clean end-arc each) at a degenerate terminus — e.g. the two ends of a
/// partial-revolve's axis-junction edge, where each adjacent end face is a
/// planar sector that pinches to a POLE exactly on the edge endpoint: with a
/// coincident end cap the subtract emits duplicate/zero-length edges at the
/// pole (non-integral genus), whereas a small overshoot removes only air past
/// the pole and leaves a valid solid.  `overshoot` is 0 for the common case.
///
/// `ends` extends the SAME tool asymmetrically, one end at a time: it is how
/// far this edge was trimmed back from a shared corner by an earlier blend in
/// the same group, so the channel runs THROUGH the corner instead of stopping
/// at a flush cap inside the material.
fn build_exact_tool(
    cross: &EdgeCross,
    profile: &[NurbsCurve],
    overshoot: f64,
    ends: ToolEnds,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let start_pad = overshoot + ends.start.max(0.0);
    let end_pad = overshoot + ends.end.max(0.0);
    let mut tool = match &cross.path {
        EdgePath::Straight { direction, length } => {
            let shifted: Vec<NurbsCurve> = if start_pad > 0.0 {
                profile
                    .iter()
                    .map(|curve| {
                        let mut shifted = curve.clone();
                        for cp in shifted.control_points.iter_mut() {
                            // Homogeneous control points store (P·w, w); translate
                            // the point P by −start_pad·direction ⇒ subtract
                            // start_pad·direction·w from the (x,y,z) components.
                            let delta = direction.scale(start_pad * cp.w);
                            cp.x -= delta.x;
                            cp.y -= delta.y;
                            cp.z -= delta.z;
                        }
                        shifted
                    })
                    .collect()
            } else {
                profile.to_vec()
            };
            extrude_profile_brep(&shifted, *direction, *length + start_pad + end_pad)?
        }
        EdgePath::Circular {
            center,
            axis,
            sweep,
        } => {
            // The revolved counterpart: arc-length pads become sweep angles at
            // the edge's own radius, and the profile rotates back by the start
            // pad so the tool still begins where the cutter must start.
            let path_radius = {
                let radial = cross.start.sub(*center);
                radial.sub(axis.scale(radial.dot(*axis))).length()
            };
            let (start_angle, end_angle) = if path_radius > 1e-12 {
                (start_pad / path_radius, end_pad / path_radius)
            } else {
                (0.0, 0.0)
            };
            let total = (*sweep + start_angle + end_angle).min(std::f64::consts::TAU);
            let rotated: Vec<NurbsCurve> = if start_angle > 0.0 {
                profile
                    .iter()
                    .map(|curve| rotate_curve_about_axis(curve, *center, *axis, -start_angle))
                    .collect::<Result<_, _>>()?
            } else {
                profile.to_vec()
            };
            revolve_profile_brep(&rotated, *center, *axis, total)
                .or_refuse(KernelStage::Fragment, "revolve_profile")?
        }
    };
    // Side faces are emitted in input-curve order; the blend wall is the
    // second profile curve.  Every OTHER cutter face — the two side walls
    // lying on the mates and the end caps — is scaffolding: it exists to close
    // the tool solid and must not appear on the blended solid.  It is tagged
    // so a survivor can be recognised in the boolean's output
    // (`count_cutter_scaffold`) and settled (`settle_cutter_scaffold`); the
    // tag never reaches a caller.
    let mut side_index = 0usize;
    for shell in &mut tool.shells {
        for face in &mut shell.faces {
            if side_index == 1 {
                if let Some(name) = name {
                    if face.name.is_none() {
                        face.name = Some(name.to_string());
                    }
                }
            } else {
                face.name = Some(scaffold_tag(name));
            }
            side_index += 1;
        }
    }
    Ok(tool)
}

/// Prefix of the name stamped on every cutter face except the blend wall while
/// the blend boolean runs.  A control character leads it so no authored or
/// derived face name can collide with it.
pub(super) const CUTTER_SCAFFOLD_NAME: &str = "\u{1}blend-cutter-scaffold:";

/// The scaffold tag of a cutter whose wall is named `wall`: the prefix, the
/// wall's name and a closing control character, so the wall survives the
/// boolean's `_n` fragment suffix, which lands after it.
pub(super) fn scaffold_tag(wall: Option<&str>) -> String {
    format!("{CUTTER_SCAFFOLD_NAME}{}\u{1}", wall.unwrap_or(""))
}

/// The wall name a scaffold tag carries, or `None` for a face that is not
/// scaffold.
fn scaffold_wall(name: Option<&str>) -> Option<&str> {
    let rest = name?.strip_prefix(CUTTER_SCAFFOLD_NAME)?;
    Some(rest.split('\u{1}').next().unwrap_or(""))
}

/// How many faces of a blend boolean's result carry the scaffold tag — the
/// cutter faces the boolean left standing on the blended solid.  Read only on
/// the general-first lane, whose operand never carries a tag (every earlier
/// blend on that lane settled its own), so every one counted is this cutter's.
///
/// Zero is the shape of a complete blend: the result's boundary is the
/// original solid's faces plus the blend wall, nothing else.  A survivor is a
/// bulkhead — the cutter's end cap (or a side wall) standing INSIDE the
/// material because the cutter stopped where the material did not — or an end
/// cap lying in the plane of the face the blend ends on.
fn count_cutter_scaffold(solid: &BrepSolid) -> usize {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| scaffold_wall(face.name.as_deref()).is_some())
        .count()
}

/// Settle the cutter faces a blend left standing, by whose material each one is.
///
/// A survivor that rides the same carrier as a face it shares an edge with
/// is that face's material: the cutter's end cap lying in the plane of the face
/// a concave blend ends on (the fill's end, bounded by the arc and the two
/// straight legs to the sharp corner the blend replaced) or a side wall on its
/// mate. It merges into that face — the boolean's own same-carrier merge at the
/// blend boolean's model tolerance, which moves no boundary and so no volume —
/// and the two edges its corner vertices split along that face's boundary are
/// joined again, so the blend and a solid that never had the corner agree on
/// F/E/V. A survivor on no neighbour's carrier is a face the blend made, a cap
/// of its own, and is named `{wall}:CAP` after the wall of the edge it came
/// from, as the stripe network names its caps; with no wall name (the legacy
/// unnamed call) it stays unnamed.
///
/// A merge the checks refuse leaves every survivor in place, named as a cap.
pub(super) fn settle_cutter_scaffold(solid: BrepSolid) -> BrepSolid {
    let tagged: rustc_hash::FxHashSet<u64> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| scaffold_wall(face.name.as_deref()).is_some())
        .map(|face| face.id)
        .collect();
    if tagged.is_empty() {
        return solid;
    }
    let tolerance = crate::KernelTolerances::for_solid(&solid, BooleanOptions::default().tolerance).model;
    let corners: rustc_hash::FxHashSet<u64> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| tagged.contains(&face.id))
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .filter_map(|coedge| solid.edges.iter().find(|edge| edge.id == coedge.edge_id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let mut settled = match crate::face_merge::absorb_faces_into_carriers(&solid, tolerance, &tagged) {
        Ok(absorbed) => {
            match crate::coalesce::merge_curve_continuation_edges_at(&absorbed, tolerance, &corners) {
                Ok(joined) if joined.validate().is_empty() => joined,
                _ => absorbed,
            }
        }
        Err(_) => solid,
    };
    for shell in &mut settled.shells {
        for face in &mut shell.faces {
            if let Some(wall) = scaffold_wall(face.name.as_deref()) {
                face.name = (!wall.is_empty()).then(|| format!("{wall}:CAP"));
            }
        }
    }
    settled
}

/// Rigid rotation of a curve's control points about `axis` through `center`
/// (Rodrigues on the euclidean points; weights are unchanged).
fn rotate_curve_about_axis(
    curve: &NurbsCurve,
    center: Vec3,
    axis: Vec3,
    angle: f64,
) -> Result<NurbsCurve, KernelRefusal> {
    let (cosine, sine) = (angle.cos(), angle.sin());
    NurbsCurve::new(
        curve.degree,
        curve.knots.clone(),
        curve
            .control_points
            .iter()
            .map(|cp| {
                let point = Vec3::new(cp.x / cp.w, cp.y / cp.w, cp.z / cp.w);
                let vector = point.sub(center);
                let rotated = center
                    .add(vector.scale(cosine))
                    .add(axis.cross(vector).scale(sine))
                    .add(axis.scale(axis.dot(vector) * (1.0 - cosine)));
                crate::Vec4::from_point(rotated, cp.w)
            })
            .collect(),
    )
    .or_refuse(KernelStage::Fragment, "rotate_curve")
}

/// How far the cutter for `edge_id` must run PAST the edge's CURRENT ends to
/// cover the extent that edge had on the ORIGINAL solid.
///
/// A group blend applies its per-edge cutters in sequence, and every cutter
/// TRIMS the edges that touch it: at a shared corner the next edge in the
/// chain starts (or ends) a tangency setback short of where it did before.  A
/// flush cutter built on the trimmed edge therefore stops INSIDE the material
/// and leaves a wedge of it standing at the corner, capped by the cutter's own
/// end plane — the "one corner is not getting made" artefact.  The removal
/// volumes of adjacent blends are supposed to UNION through the corner (that
/// is what makes the miter), so each cutter is extended back to the corner the
/// edge had before its neighbours were blended.
///
/// `blocked` lists corner points that must keep the flush cutter: the STAR
/// corners (>=3 selected edges at one vertex) are closed afterwards by the
/// corner-rounding surgery, which is built against exactly the sequential
/// geometry a flush cutter leaves.
///
/// Returns zero pads when the edge is untrimmed, when its geometry is not the
/// straight/circular path the exact cutter is built on, or when the original
/// endpoints do not lie on the current edge's carrier (an edge that a boolean
/// SPLIT rather than trimmed) — in every one of those cases the historical
/// flush cutter is what gets built.
pub(super) fn tool_ends_to_original_extent(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
    original: (Vec3, Vec3),
    blocked: &[Vec3],
) -> ToolEnds {
    let zero = ToolEnds::default();
    let Ok(cross) = analyze_edge(solid, edge_id, radius) else {
        return zero;
    };
    let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id) else {
        return zero;
    };
    if edge.start_vertex_id == edge.end_vertex_id {
        // Closed edge (a full circle): no ends to extend.
        return zero;
    }
    let (Ok(current_start), Ok(current_end)) =
        (edge.curve.evaluate(edge.t0), edge.curve.evaluate(edge.t1))
    else {
        return zero;
    };
    // Arc-length station of a point along the edge path, measured from the
    // current start (negative before it, > length past the current end), plus
    // its distance off the carrier.
    let station = |point: Vec3| -> Option<(f64, f64)> {
        match &cross.path {
            EdgePath::Straight { direction, .. } => {
                let offset = point.sub(current_start);
                let along = offset.dot(*direction);
                Some((along, offset.sub(direction.scale(along)).length()))
            }
            EdgePath::Circular { center, axis, .. } => {
                let radial = |p: Vec3| {
                    let v = p.sub(*center);
                    v.sub(axis.scale(v.dot(*axis)))
                };
                let from = radial(current_start);
                let to = radial(point);
                let path_radius = from.length();
                if path_radius <= 1e-12 {
                    return None;
                }
                let mut angle = from.cross(to).dot(*axis).atan2(from.dot(to));
                // The edge sweeps in +axis; a station just before the start
                // reads as a small NEGATIVE angle rather than nearly a turn.
                if angle > std::f64::consts::PI {
                    angle -= std::f64::consts::TAU;
                }
                let off_axis =
                    (point.sub(*center).dot(*axis)) - (current_start.sub(*center).dot(*axis));
                let off_circle = (to.length() - path_radius).hypot(off_axis);
                Some((angle * path_radius, off_circle))
            }
        }
    };
    let length = match &cross.path {
        EdgePath::Straight { length, .. } => *length,
        EdgePath::Circular { .. } => match station(current_end) {
            Some((along, _)) if along > 0.0 => along,
            _ => return zero,
        },
    };
    if !(length > 0.0) {
        return zero;
    }
    let tolerance = 1e-6 * (1.0 + length);
    let (Some((a, a_off)), Some((b, b_off))) = (station(original.0), station(original.1)) else {
        return zero;
    };
    if a_off > tolerance || b_off > tolerance {
        return zero;
    }
    let ((low, low_point), (high, high_point)) = if a <= b {
        ((a, original.0), (b, original.1))
    } else {
        ((b, original.1), (a, original.0))
    };
    // The original extent must CONTAIN the current one: anything else means
    // the point resolved to a different edge (a split), not a trimmed end.
    if low > tolerance || high < length - tolerance {
        return zero;
    }
    let is_blocked = |point: Vec3| {
        blocked
            .iter()
            .any(|corner| corner.sub(point).length() < 1e-6)
    };
    ToolEnds {
        start: if is_blocked(low_point) {
            0.0
        } else {
            (-low).max(0.0)
        },
        end: if is_blocked(high_point) {
            0.0
        } else {
            (high - length).max(0.0)
        },
    }
}

fn fillet_or_chamfer_exact(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    ends: ToolEnds,
    lane: Lane,
) -> Result<BrepSolid, KernelRefusal> {
    let cross = analyze_edge(solid, edge_id, radius)?;
    let profile = cross_section_profile(&cross, radius, chamfer)?;
    // Straight edges: try a flush tool first (the workhorse box/prism case,
    // where the end caps land coincident on transverse end faces and the
    // boolean handles them), then retry with a small overshoot if that
    // subtract cannot assemble a valid solid — the overshoot rescues
    // degenerate termini (revolve axis-junction poles) without disturbing the
    // flush cases.  Circular edges are closed loops with no free ends.
    let overshoots: &[f64] = match &cross.path {
        EdgePath::Straight { .. } => &[0.0, radius],
        EdgePath::Circular { .. } => &[0.0],
    };
    // A LONE blend (the single-edge lane, no group pad on its ends) must run
    // THROUGH whatever bounds the material at its ends: where the flush cutter
    // assembles but leaves its own end cap standing on the result, the
    // material continued past the end face — the slab's top face met a
    // tangent blend a hair before the rail's tangency setback (2026-09-06
    // report: an r = 10.5 round on a 1.5 mm rim whose flat top ends at z = 10
    // and rolls into an r = 10 blend) — and the flush cut is a bulkhead
    // inside the part, not a blend.  The overshoot cutter reaches past that
    // end and is accepted when NOTHING of it but the wall survives: the
    // result is then bounded by the original faces and the blend surface
    // alone, which is the definition of a complete blend.  If the overshoot
    // leaves scaffold standing too, the material really does continue and
    // the flush cut (which at least stops at the end face) is kept, exactly
    // as before.  A group's sequential composition keeps the flush cut
    // unconditionally: its corner closures are built against the bulkhead
    // the next cutter in the chain removes.
    let prefer_through = lane == Lane::GeneralFirst && ends == ToolEnds::default();
    // Walk the ladder TWICE.  The first pass assembles each cutter with the
    // exact arrangement only, so a flush tool that hits a terminus degeneracy
    // falls through to the overshoot tool — whose end caps cut the pole faces
    // transversally — instead of being rescued by a bodily TRANSLATED cutter
    // (the Simulation-of-Simplicity lane inside `boolean_operation`), which
    // would leave the blend wall off its supports.  Only when no rung of the
    // ladder assembles exactly do we allow that perturbed rescue, so every
    // selection that used to succeed still does.
    // Every rung of the ladder below writes this before it is read: the
    // overshoot list is never empty.  The empty seed carries no text.
    let mut last_error = KernelRefusal::internal(KernelStage::Fragment, "cutter", String::new());
    let mut capped: Option<BrepSolid> = None;
    for exact_only in [true, false] {
        for (attempt, &overshoot) in overshoots.iter().enumerate() {
            let tool = build_exact_tool(&cross, &profile, overshoot, ends, name)?;
            let assembled = if exact_only {
                apply_tool_exact(solid, &tool, cross.convex)
            } else {
                apply_tool(solid, &tool, cross.convex)
            };
            match assembled {
                Ok(result) => {
                    let survivors = count_cutter_scaffold(&result);
                    if !prefer_through {
                        return Ok(result);
                    }
                    // The through cut is taken only as a VALID solid; a flush
                    // cut that assembled with a bulkhead is otherwise kept.
                    if survivors == 0 && (capped.is_none() || result.validate().is_empty()) {
                        return Ok(result);
                    }
                    if capped.is_none() {
                        capped = Some(result);
                    }
                    // A bulkhead standing on the result is the cut contradicting
                    // what a complete blend is (the header above): a check that
                    // tripped, never a refusal of the input.
                    last_error = KernelRefusal::internal(
                        KernelStage::Validate,
                        "cutter_scaffold",
                        format!(
                            "the cutter left {survivors} of its own faces standing on the result \
                             (overshoot {overshoot})"
                        ),
                    );
                }
                Err(error) => {
                    last_error = if exact_only && attempt == 0 {
                        error
                    } else {
                        // The retry's own class survives; the earlier rung's
                        // text is kept in front of it.
                        error.with_message(|error| {
                            format!("{last_error}; overshoot retry also failed: {error}")
                        })
                    };
                }
            }
        }
        // An exactly assembled cut, bulkhead and all, beats a perturbed one.
        if let Some(result) = capped.take() {
            return Ok(result);
        }
    }
    Err(last_error)
}
