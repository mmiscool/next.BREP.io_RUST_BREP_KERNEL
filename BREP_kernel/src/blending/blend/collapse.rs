//! Full-width corners: the topology a stripe network leaves when a blend is
//! exactly as wide as a face it meets at a corner.
//!
//! # What collapses, and why it is not snapped
//!
//! At a full-width corner the corner ball touches its faces ON their
//! boundaries: a miter whose radius is the side face's width puts the shared
//! face's tangency point on that face's far vertex, a star at full width puts
//! every tangency on a vertex, and a channel whose middle edge is exactly two
//! setbacks long puts both miters' shared-face points at one point.  Each
//! stripe's rail between two such stops has NO length (`build_open_surgery`
//! builds no edge for it), so its two rim vertices are one vertex, and the
//! faces around it lose area:
//!
//! * a face whose every boundary edge was consumed or collapsed keeps an EMPTY
//!   loop — the shared face of the miter, all three faces of the star;
//! * a face left between two coincident edges has zero area — a side face
//!   between its sewn rail and its far edge, the channel's top face between the
//!   two coincident rails, a star stripe's bevel between its corner section and
//!   its free end's section.
//!
//! Nothing here moves geometry by more than the solid's `consumed_band`
//! ([`crate::blend::consumed_band`], the sliver tolerance): the rims that are
//! identified were decided one point by that band, and edges are merged only
//! when they coincide within it.  The body is then held to its closure bar
//! ([`check_collapse_closure`]), because an identification is a trim moved
//! onto what already existed, which `validate()` cannot see.

use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use crate::topology::BrepSolid;
use crate::Vec3;

/// The refusal for a full-width collapse this construction cannot finish —
/// see [`collapse_full_width`].  TERMINAL in the group lane (like
/// [`crate::blend::is_consumed_snap`]): the cutter composition's answer to
/// a full-width corner the network does not build was measured wrong.
pub(crate) const FULL_WIDTH_COLLAPSE_UNSOUND: &str =
    "blend: the full-width corner does not collapse to a sound body —";

/// The slugs of those refusals: `UnsupportedGeometry`, except the face that
/// uses a missing edge, which is `Internal` (its text is under
/// [`FULL_WIDTH_COLLAPSE_UNSOUND`] too, and the fallbacks treat it the same).
pub(crate) const FULL_WIDTH_COLLAPSE_WHAT: &str = "full_width_collapse";
pub(crate) const FULL_WIDTH_COLLAPSE_EDGE_WHAT: &str = "full_width_collapse_edge";

/// Is this a full-width collapse refusal ([`FULL_WIDTH_COLLAPSE_UNSOUND`])?
/// Read off the class and its slug, which the mint site and this check share.
pub(crate) fn is_full_width_collapse(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, RefusalClass::UnsupportedGeometry { what } if what == FULL_WIDTH_COLLAPSE_WHAT) ||
        matches!(&refusal.class, RefusalClass::Internal { what } if what == FULL_WIDTH_COLLAPSE_EDGE_WHAT)
}

/// What [`collapse_full_width`] changed, for the passes that run after it.
#[derive(Default)]
pub(in crate::blend) struct Collapse {
    /// (merged edge, the edge that replaced it).
    pub(in crate::blend) merged_edges: Vec<(u64, u64)>,
    /// Faces left with no area and dropped.
    pub(in crate::blend) dropped_faces: Vec<u64>,
    /// The farthest an identified vertex stood from the one kept.
    pub(in crate::blend) moved: f64,
}

impl Collapse {
    /// True when the pass changed the body.
    pub(in crate::blend) fn changed(&self) -> bool {
        !self.merged_edges.is_empty() || !self.dropped_faces.is_empty() || self.moved > 0.0
    }

    /// The edge standing for `edge_id` after the merges.
    pub(in crate::blend) fn edge(&self, mut edge_id: u64) -> u64 {
        while let Some((_, kept)) = self.merged_edges.iter().find(|(merged, _)| *merged == edge_id) {
            edge_id = *kept;
        }
        edge_id
    }
}

/// Samples used to decide that two edges between one pair of vertices are the
/// same curve.
const COINCIDENCE_SAMPLES: usize = 16;

/// Identify the `identified` vertex pairs, then drop every face the
/// identification left without area and merge the coincident edges that
/// bounded it.
///
/// `input` is the solid the network started from: a vertex or edge of it is
/// the one KEPT when it is identified with one the network built, so the
/// part's own entities keep their ids.  Two input vertices are never
/// identified with each other, and nothing farther apart than `band` is ever
/// merged; either would be a different construction, and refuses.  Only faces
/// the network changed are read, so a degenerate face in the input is not this
/// pass's to repair.
///
/// `collapsed` says whether anything collapsed at all — a rail (then
/// `identified` is not empty) or a sharp edge consumed whole.  When nothing
/// did, the body is returned untouched without a face being read.
pub(in crate::blend) fn collapse_full_width(
    input: &BrepSolid,
    result: &mut BrepSolid,
    identified: &[(u64, u64)],
    collapsed: bool,
    band: f64,
) -> Result<Collapse, KernelRefusal> {
    let mut collapse = Collapse::default();
    if !collapsed && identified.is_empty() {
        return Ok(collapse);
    }
    let is_input_vertex = |id: u64| input.vertices.iter().any(|vertex| vertex.id == id);
    let point_of = |solid: &BrepSolid, id: u64| -> Result<Vec3, KernelRefusal> {
        solid
            .vertices
            .iter()
            .find(|vertex| vertex.id == id)
            .map(|vertex| vertex.point)
            .ok_or_else(|| format!("{FULL_WIDTH_COLLAPSE_UNSOUND} vertex {id} is missing")).or_refuse(KernelStage::Refine, "ok_or_else")
    };

    // ---- 1. Identify the rims, keeping an input vertex where there is one. ----
    let mut representative: Vec<(u64, u64)> = Vec::new();
    let find = |representative: &[(u64, u64)], mut id: u64| {
        while let Some((_, to)) = representative.iter().find(|(from, _)| *from == id) {
            id = *to;
        }
        id
    };
    for &(a, b) in identified {
        let (a, b) = (find(&representative, a), find(&representative, b));
        if a == b {
            continue;
        }
        let (kept, merged) = match (is_input_vertex(a), is_input_vertex(b)) {
            (true, true) => {
                return Err(KernelRefusal::unsupported(KernelStage::Validate, FULL_WIDTH_COLLAPSE_WHAT, format!(
                    "{FULL_WIDTH_COLLAPSE_UNSOUND} a collapsed rail would identify two vertices \
                     of the part, {a} and {b}"
                )))
            }
            (true, false) => (a, b),
            (false, true) => (b, a),
            (false, false) => (a.min(b), a.max(b)),
        };
        let gap = point_of(result, kept)?.sub(point_of(result, merged)?).length();
        if gap > band {
            return Err(KernelRefusal::unsupported(KernelStage::Validate, FULL_WIDTH_COLLAPSE_WHAT, format!(
                "{FULL_WIDTH_COLLAPSE_UNSOUND} a collapsed rail's two rims stand {gap:.3e} apart, \
                 more than the {band:.3e} band that called the rail a point"
            )));
        }
        collapse.moved = collapse.moved.max(gap);
        representative.push((merged, kept));
    }
    if !representative.is_empty() {
        for edge in &mut result.edges {
            edge.start_vertex_id = find(&representative, edge.start_vertex_id);
            edge.end_vertex_id = find(&representative, edge.end_vertex_id);
        }
        result
            .vertices
            .retain(|vertex| !representative.iter().any(|(merged, _)| *merged == vertex.id));
    }

    // ---- 2. Drop the faces left without area, until none is. ----
    let loop_edges = |face: &crate::topology::FaceRecord| -> Vec<Vec<u64>> {
        face.loops
            .iter()
            .map(|loop_record| loop_record.coedges.iter().map(|c| c.edge_id).collect())
            .collect()
    };
    let original: rustc_hash::FxHashMap<u64, Vec<Vec<u64>>> = input
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| (face.id, loop_edges(face)))
        .collect();
    loop {
        let face_ids: Vec<u64> = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| original.get(&face.id) != Some(&loop_edges(face)))
            .map(|face| face.id)
            .collect();
        let mut progressed = false;
        for face_id in face_ids {
            let Some(face) = result.shells.iter().flat_map(|shell| &shell.faces).find(|f| f.id == face_id)
            else {
                continue;
            };
            let empty = face.loops.iter().filter(|l| l.coedges.is_empty()).count();
            if empty > 0 {
                if empty != face.loops.len() {
                    return Err(KernelRefusal::unsupported(KernelStage::Validate, FULL_WIDTH_COLLAPSE_WHAT, format!(
                        "{FULL_WIDTH_COLLAPSE_UNSOUND} face {face_id} lost {empty} of its \
                         {} loops whole",
                        face.loops.len()
                    )));
                }
                drop_face(result, face_id);
                collapse.dropped_faces.push(face_id);
                progressed = true;
                continue;
            }
            if face.loops.len() != 1 || face.loops[0].coedges.len() != 2 {
                continue;
            }
            let [first, second] = [&face.loops[0].coedges[0], &face.loops[0].coedges[1]];
            if first.edge_id == second.edge_id {
                continue;
            }
            let edge_of = |id: u64| result.edges.iter().find(|edge| edge.id == id);
            let (Some(a), Some(b)) = (edge_of(first.edge_id), edge_of(second.edge_id)) else {
                return Err(KernelRefusal::internal(KernelStage::Validate, FULL_WIDTH_COLLAPSE_EDGE_WHAT, format!(
                    "{FULL_WIDTH_COLLAPSE_UNSOUND} face {face_id} uses a missing edge"
                )));
            };
            let aligned = a.start_vertex_id == b.start_vertex_id && a.end_vertex_id == b.end_vertex_id;
            let opposed = a.start_vertex_id == b.end_vertex_id && a.end_vertex_id == b.start_vertex_id;
            if !(aligned || opposed) || a.start_vertex_id == a.end_vertex_id {
                continue;
            }
            // Coincident as curves: every sample of one within the band of the
            // other.  Whether they also agree PARAMETER for parameter decides
            // whether a moved coedge keeps its pcurve.
            let mut affine = true;
            let mut coincident = true;
            for sample in 0..=COINCIDENCE_SAMPLES {
                let fraction = sample as f64 / COINCIDENCE_SAMPLES as f64;
                let on_b = b.curve.evaluate(b.t0 + (b.t1 - b.t0) * fraction).or_refuse(KernelStage::Refine, "evaluate")?;
                let a_fraction = if aligned { fraction } else { 1.0 - fraction };
                let on_a = a.curve.evaluate(a.t0 + (a.t1 - a.t0) * a_fraction).or_refuse(KernelStage::Refine, "evaluate")?;
                if on_a.sub(on_b).length() <= band {
                    continue;
                }
                affine = false;
                let foot = crate::project_point_to_curve(&a.curve, on_b).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
                let inside = foot.u >= a.t0.min(a.t1) - 1e-9 && foot.u <= a.t0.max(a.t1) + 1e-9;
                if foot.distance > band || !inside {
                    coincident = false;
                    break;
                }
            }
            // Two different curves between one pair of vertices bound a real
            // face (a lens); it is not this pass's.
            if !coincident {
                continue;
            }
            // The part's own edge survives; otherwise the older one.
            let a_input = input.edges.iter().any(|edge| edge.id == a.id);
            let b_input = input.edges.iter().any(|edge| edge.id == b.id);
            let (kept, merged) = if a_input || (!b_input && a.id < b.id) {
                (a.id, b.id)
            } else {
                (b.id, a.id)
            };
            drop_face(result, face_id);
            collapse.dropped_faces.push(face_id);
            merge_edge(result, merged, kept, affine)?;
            collapse.merged_edges.push((merged, kept));
            progressed = true;
        }
        if !progressed {
            break;
        }
    }
    Ok(collapse)
}

fn drop_face(result: &mut BrepSolid, face_id: u64) {
    for shell in &mut result.shells {
        shell.faces.retain(|face| face.id != face_id);
    }
}

/// Replace every use of edge `merged` by edge `kept`, which runs between the
/// same two vertices along the same curve within `band`, and delete `merged`.
///
/// A coedge keeps its own traversal: its sense flips when the two edges are
/// stored in opposite directions, and its pcurve — parameterised along the
/// traversal — is kept when the two curves agree parameter for parameter
/// (`affine`) and refitted onto its face from `kept` otherwise.
fn merge_edge(
    result: &mut BrepSolid,
    merged: u64,
    kept: u64,
    affine: bool,
) -> Result<(), KernelRefusal> {
    let edge_of = |id: u64| {
        result
            .edges
            .iter()
            .find(|edge| edge.id == id)
            .cloned()
            .ok_or_else(|| format!("{FULL_WIDTH_COLLAPSE_UNSOUND} edge {id} is missing"))
    };
    let (merged_edge, kept_edge) = (edge_of(merged).or_refuse(KernelStage::Refine, "edge_of")?, edge_of(kept).or_refuse(KernelStage::Refine, "edge_of")?);
    let flipped = merged_edge.start_vertex_id != kept_edge.start_vertex_id;
    for face in result.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        let surface = face.surface.clone();
        for coedge in face.loops.iter_mut().flat_map(|loop_record| &mut loop_record.coedges) {
            if coedge.edge_id != merged {
                continue;
            }
            coedge.edge_id = kept;
            if flipped {
                coedge.forward = !coedge.forward;
            }
            if !affine {
                // Refitted by the one verified fitter, held to the refinement
                // floor out of sample like every other trim, not to the band
                // the merge was decided under.
                let forward = coedge.forward;
                let at = |fraction: f64| {
                    let along = if forward { fraction } else { 1.0 - fraction };
                    kept_edge.curve.evaluate(kept_edge.t0 + (kept_edge.t1 - kept_edge.t0) * along)
                };
                let floor = crate::pcurve::PCURVE_REFINEMENT_TOLERANCE;
                let breaks = super::track_fit::curve_breaks(
                    &kept_edge.curve,
                    kept_edge.t0,
                    kept_edge.t1,
                    forward,
                );
                let fit =
                    super::track_fit::fit_curve_track(&surface, &at, &breaks, floor, 256, 4096)?;
                if !fit.on_floor {
                    return Err(KernelRefusal::unsupported(KernelStage::Validate, FULL_WIDTH_COLLAPSE_WHAT, format!(
                        "{FULL_WIDTH_COLLAPSE_UNSOUND} the merged edge {kept}'s pcurve misses it by \
                         {:.3e} at {} samples against a floor of {floor:.1e}",
                        fit.miss, fit.samples
                    )));
                }
                coedge.pcurve = fit.curve;
            }
        }
    }
    result.edges.retain(|edge| edge.id != merged);
    Ok(())
}

/// Accept a collapsed body only while its shells still close.  Every
/// identification and merge above is a trim moved onto what already existed,
/// by at most the band; whether that closes depends on the shape, exactly as a
/// consumed snap does (`check_snap_closure`), so it is measured, not assumed.
pub(in crate::blend) fn check_collapse_closure(
    input: &BrepSolid,
    result: &BrepSolid,
) -> Result<(), KernelRefusal> {
    let report = crate::shell_vector_areas(result);
    if !report.is_flagged() && report.unreadable.is_empty() {
        return Ok(());
    }
    if crate::shell_vector_areas(input).is_flagged() {
        return Ok(());
    }
    let (residual, bar) = report
        .worst_shell()
        .map_or((f64::NAN, f64::NAN), |shell| (shell.residual(), shell.bar));
    Err(KernelRefusal::unsupported(KernelStage::Validate, FULL_WIDTH_COLLAPSE_WHAT, format!(
        "{FULL_WIDTH_COLLAPSE_UNSOUND} its trims close to {residual:.3e} against a bar of \
         {bar:.3e}{}",
        if report.unreadable.is_empty() {
            String::new()
        } else {
            format!(" ({} faces unreadable)", report.unreadable.len())
        }
    )))
}
