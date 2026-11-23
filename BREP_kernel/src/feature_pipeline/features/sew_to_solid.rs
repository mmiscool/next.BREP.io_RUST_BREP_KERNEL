//! SEW — Sew to Solid: close faces that enclose a volume into one solid.
//!
//! Semantics:
//!
//! - `inputParams.faces` — ONE `reference_selection` admitting sheets, faces
//!   (of sheets or of solids) and whole solids, `multiple`. A name resolves as
//!   an exact face, then a body (sheet or solid), then a face container.
//! - `inputParams.tolerance` — how far apart two edges may lie and still be
//!   sewn into one. `0` (the default) is automatic: 1e-6 of the picked
//!   geometry's diagonal.
//! - `inputParams.unite` — unite the new volume with the solids whose faces
//!   bound it (default on), giving one solid.
//!
//! What closes the volume:
//!
//! - Every face of a picked sheet, and every individually picked face, is
//!   sewn as it is ([`crate::sew_solid`]: edges paired by where they lie,
//!   within the tolerance).
//! - A picked SOLID is split along the edges the sewn faces leave open on it
//!   (a fill built on a cube's top edges leaves those four edges open, and
//!   they cut the cube into its top face and the other five). Of the face sets
//!   the cut leaves, the one that closes the sewn faces into the SMALLEST
//!   volume is the cap: the volume between the faces and the solid, never the
//!   solid swallowed with it. Individually picked faces of a solid are its
//!   cap as picked.
//! - Unite on: the solid's cap is dropped and the rest of its faces are sewn
//!   to the picked faces directly, so the cap (now inside the result) is gone
//!   and no boolean has to meet two coincident faces. The result must measure
//!   the gap's volume plus each solid's; when it does not (the faces cut into
//!   a solid), the gap volume is united with the solids by the boolean
//!   instead. A picked solid no face touches is united by the boolean.
//! - Unite off: the gap volume alone, its caps copies of the solids' faces;
//!   the solids stay.
//!
//! When the faces do not close the feature refuses by name and lists the
//! open edges (count, total length, the largest gap between an open edge and
//! its nearest open neighbour, which is the tolerance that would sew it), and
//! publishes each open edge as the path `${featureId}:GAP[n]`, which the app
//! draws in the viewport so the user sees which edge to fix.
//!
//! Naming (contract rule 2): the result is `${featureId}`. Faces keep the
//! names they had; a face copied from a body that stays in the scene is
//! suffixed `::${featureId}`. Sheets whose every face went into the result,
//! and with unite on the picked solids, are consumed (`removed`).

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureResult};
use crate::topology::{EdgeRecord, ShellRecord, VertexRecord};
use crate::{BodyKind, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, Vec3};

/// Automatic tolerance, as a fraction of the picked geometry's diagonal.
pub const AUTO_TOLERANCE_FRACTION: f64 = 1e-6;
/// The most cap combinations tried across all picked solids.
const MAX_COMBINATIONS: usize = 256;
/// Samples along an edge when comparing it with another.
const EDGE_SAMPLES: usize = 8;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(failure) => {
            let mut result = ctx.fail(failure.refusal);
            result.paths = failure.gaps;
            result
        }
    }
}

/// A refusal plus the open edges to draw.
struct Failure {
    refusal: KernelRefusal,
    gaps: Vec<(String, Vec<NurbsCurve>)>,
}

impl From<KernelRefusal> for Failure {
    fn from(refusal: KernelRefusal) -> Self {
        Self { refusal, gaps: Vec::new() }
    }
}

fn input(what: &'static str, message: String) -> Failure {
    KernelRefusal::input(KernelStage::Collect, what, format!("Sew to Solid: {message}")).into()
}

/// One picked body: its name in the scene, a copy, and which faces of it the
/// picks named (`whole` when the body itself was picked).
struct Body {
    name: String,
    solid: BrepSolid,
    closed: bool,
    whole: bool,
    faces: BTreeSet<u64>,
}

impl Body {
    fn all_faces(&self) -> BTreeSet<u64> {
        self.solid.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).collect()
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, Failure> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    let names = common::unique_reference_names(ctx.param("faces"));
    // Resolve each name: an exact face, then a body, then a face container.
    let mut picks: BTreeMap<u32, (String, bool, BTreeSet<u64>)> = BTreeMap::new();
    let body_name = |handle: u32| {
        ctx.scene
            .solids
            .iter()
            .find(|(_, known)| **known == handle)
            .map(|(name, _)| name.clone())
            .unwrap_or_else(|| format!("#{handle}"))
    };
    for name in &names {
        if let Some(face) = ctx.scene.faces.get(name).copied() {
            picks.entry(face.handle).or_insert_with(|| (body_name(face.handle), false, BTreeSet::new())).2.insert(face.face_id);
        } else if let Some(handle) = ctx.scene.resolve_solid(name) {
            picks.entry(handle).or_insert_with(|| (name.clone(), false, BTreeSet::new())).1 = true;
        } else {
            let group = ctx.scene.resolve_face_group(name);
            if group.is_empty() {
                result.unresolved.push(name.clone());
            }
            for face in group {
                picks.entry(face.handle).or_insert_with(|| (body_name(face.handle), false, BTreeSet::new())).2.insert(face.face_id);
            }
        }
    }
    if picks.is_empty() {
        return Ok(result); // Soft no-op — nothing picked, or nothing resolved.
    }
    result.note_partial_resolution(&names);

    let mut bodies = Vec::with_capacity(picks.len());
    for (handle, (name, whole, faces)) in picks {
        let solid = crate::registered_solid_clone(handle)
            .map_err(|error| input("sew_to_solid_body", format!("`{name}` is not in the scene: {error}")))?;
        let closed = solid.kind == BodyKind::Solid && open_edges(&solid).is_empty();
        bodies.push(Body { name, solid, closed, whole, faces });
    }

    let tolerance = match ctx.param("tolerance") {
        None | Some(serde_json::Value::Null) => 0.0,
        Some(_) => ctx.number("tolerance").map_err(|error| input("sew_to_solid_tolerance", error))?,
    };
    let tolerance = if tolerance.is_finite() && tolerance > 0.0 {
        tolerance
    } else if tolerance == 0.0 || tolerance.is_nan() {
        auto_tolerance(&bodies)
    } else {
        return Err(input("sew_to_solid_tolerance", format!("the tolerance must not be negative ({tolerance})")));
    };
    let unite = ctx.param("unite").and_then(serde_json::Value::as_bool).unwrap_or(true);

    let closed = close_volume(&bodies, tolerance, unite, &ctx.id)?;
    result.notes.extend(closed.notes);
    let mut solid = closed.solid;
    common::stamp_derived_edge_names(&mut solid);
    for consumed in closed.consumed {
        if !result.removed.contains(&consumed) {
            result.removed.push(consumed);
        }
    }
    let face_names = common::collect_face_names(&solid);
    let edge_names = common::collect_edge_names(&solid);
    let handle = crate::register_solid_value(solid);
    result.added.push(AddedSolid {
        handle,
        name: if ctx.id.is_empty() { ctx.feature_type.clone() } else { ctx.id.clone() },
        face_names,
        edge_names,
        ..AddedSolid::default()
    });
    Ok(result)
}

/// What [`close_volume`] built.
struct Closed {
    solid: BrepSolid,
    /// Scene names of the bodies the result replaces.
    consumed: Vec<String>,
    notes: Vec<String>,
}

/// The faces of `bodies` closed into one solid (see the module documentation).
fn close_volume(bodies: &[Body], tolerance: f64, unite: bool, id: &str) -> Result<Closed, Failure> {
    let suffix = if id.is_empty() { "SEW".to_string() } else { id.to_string() };
    let mut notes = Vec::new();
    // The faces sewn as they are: whole sheets, and faces picked one by one
    // from sheets. Faces picked from a SOLID are that solid's cap.
    let mut free: Vec<BrepSolid> = Vec::new();
    let mut consumed = Vec::new();
    for body in bodies.iter().filter(|body| !body.closed) {
        let faces = if body.whole { body.all_faces() } else { body.faces.clone() };
        let every = faces == body.all_faces();
        let mut piece = extract(&body.solid, &faces);
        if every {
            consumed.push(body.name.clone());
        } else {
            rename(&mut piece, &suffix);
        }
        free.push(piece);
    }

    // Each solid's cap choices: fixed by its picked faces, or every way its
    // faces split along the edges the free faces leave open.
    let solids: Vec<&Body> = bodies.iter().filter(|body| body.closed).collect();
    // The open edges that cut a whole solid: those of the free faces AND of
    // the faces picked one by one from other solids (their caps as picked).
    let mut known = free.clone();
    known.extend(solids.iter().filter(|body| fixed_cap(body)).map(|body| extract(&body.solid, &body.faces)));
    let free_open = if known.is_empty() { Vec::new() } else { sew(&known, tolerance)?.1 };
    let mut choices: Vec<Vec<BTreeSet<u64>>> = Vec::with_capacity(solids.len());
    for body in &solids {
        if fixed_cap(body) {
            choices.push(vec![body.faces.clone()]);
            continue;
        }
        let cut = cut_edges(&body.solid, &free_open, tolerance)?;
        if cut.is_empty() {
            choices.push(Vec::new()); // no face meets it
            continue;
        }
        let parts = components(&body.solid, &cut);
        if parts.len() < 2 {
            // The faces meet it along edges that cut out nothing (a hole
            // beside it): each face on a cut edge is tried as its cap, so a
            // refusal names the hole's own edges, not every sheet edge. The
            // empty cap leaves the solid out (united by the boolean when the
            // faces close without it).
            let mut touching_faces: Vec<BTreeSet<u64>> = vec![BTreeSet::new()];
            touching_faces.extend(body
                .solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .filter(|face| face.loops.iter().flat_map(|record| &record.coedges).any(|coedge| cut.contains(&coedge.edge_id)))
                .map(|face| BTreeSet::from([face.id])));
            choices.push(touching_faces);
            continue;
        }
        let mut options = Vec::new();
        if parts.len() <= 6 {
            for mask in 1u32..(1u32 << parts.len()) - 1 {
                let set: BTreeSet<u64> = parts
                    .iter()
                    .enumerate()
                    .filter(|(index, _)| mask & (1 << index) != 0)
                    .flat_map(|(_, part)| part.iter().copied())
                    .collect();
                options.push(set);
            }
        } else {
            options.extend(parts.iter().cloned());
        }
        choices.push(options);
    }
    let touching: Vec<usize> = (0..solids.len()).filter(|&index| !choices[index].is_empty()).collect();
    if free.is_empty() && touching.is_empty() {
        return Err(input(
            "sew_to_solid_nothing_to_sew",
            "pick the sheets or faces that enclose the volume (with the solids whose edges they are built on)".into(),
        ));
    }
    let combinations = touching
        .iter()
        .try_fold(1usize, |product, &index| product.checked_mul(choices[index].len()))
        .filter(|&count| count <= MAX_COMBINATIONS);
    let Some(combinations) = combinations else {
        return Err(input(
            "sew_to_solid_too_many_caps",
            format!("the open edges cut the picked solids into more than {MAX_COMBINATIONS} ways to close the faces; pick the faces that close the volume instead of the whole solids"),
        ));
    };

    // Try every combination; keep the closed one of least volume.
    let mut best: Option<(f64, BrepSolid, Vec<usize>)> = None;
    let mut fewest_open: Option<(Vec<EdgeRecord>, BrepSolid)> = None;
    let mut first_refusal: Option<KernelRefusal> = None;
    let mut counter = vec![0usize; touching.len()];
    for _ in 0..combinations.max(1) {
        let mut pieces = free.clone();
        for (slot, &index) in touching.iter().enumerate() {
            if choices[index][counter[slot]].is_empty() {
                continue;
            }
            let mut cap = extract(&solids[index].solid, &choices[index][counter[slot]]);
            rename(&mut cap, &suffix);
            pieces.push(cap);
        }
        let (sewn, open) = sew(&pieces, tolerance)?;
        if open.is_empty() {
            match crate::solid_from_sheets(&pieces, tolerance) {
                Ok(solid) => {
                    let volume = crate::solid_signed_volume(&solid).map_err(|error| input("sew_to_solid_volume", error))?;
                    if volume > 0.0 && best.as_ref().map_or(true, |(least, _, _)| volume < *least) {
                        best = Some((volume, solid, counter.clone()));
                    }
                }
                Err(refusal) => {
                    first_refusal.get_or_insert(refusal);
                }
            }
        } else if fewest_open.as_ref().map_or(true, |(known, _)| open.len() < known.len()) {
            fewest_open = Some((open, sewn));
        }
        // next combination
        for slot in 0..counter.len() {
            counter[slot] += 1;
            if counter[slot] < choices[touching[slot]].len() {
                break;
            }
            counter[slot] = 0;
        }
    }
    let Some((gap_volume, gap, chosen)) = best else {
        if let Some((open, sewn)) = fewest_open {
            return Err(open_refusal(&open, &sewn, id));
        }
        let refusal = first_refusal.unwrap_or_else(|| {
            KernelRefusal::unsupported(KernelStage::Sew, "sew_to_solid_no_volume", "the sewn faces enclose no volume")
        });
        let message = format!("Sew to Solid: the faces close but make no valid solid: {}", refusal.message);
        return Err(refusal.with_message(|_| message).into());
    };
    // Several closed shells (separate face sets that each close) may overlap:
    // each becomes its own solid and the boolean unites them, so an overlap
    // is never counted twice.
    let shells = gap.shells.len();
    let (gap_volume, gap) = if shells > 1 {
        let gap = one_volume(&gap, tolerance)?;
        notes.push(format!("the faces close {shells} separate volumes; united by the boolean"));
        (crate::solid_signed_volume(&gap).map_err(|error| input("sew_to_solid_volume", error))?, gap)
    } else {
        (gap_volume, gap)
    };
    notes.push(format!("closed volume {gap_volume:.6e} at tolerance {tolerance:.3e}"));
    let caps: Vec<(usize, &BTreeSet<u64>)> = touching
        .iter()
        .enumerate()
        .map(|(slot, &index)| (index, &choices[index][chosen[slot]]))
        .filter(|(_, cap)| !cap.is_empty())
        .collect();

    if !unite {
        return Ok(Closed { solid: gap, consumed, notes });
    }

    // Unite: sew the free faces to the solids' faces outside their caps.
    let mut united: Option<BrepSolid> = None;
    let mut pieces = free.clone();
    let mut expected = gap_volume;
    for &(index, cap) in &caps {
        let body = solids[index];
        let all = body.all_faces();
        if *cap == all {
            // The whole solid is a shell of the gap volume already.
            pieces.push(extract(&body.solid, cap));
            continue;
        }
        let rest: BTreeSet<u64> = all.difference(cap).copied().collect();
        pieces.push(extract(&body.solid, &rest));
        expected += crate::solid_signed_volume(&body.solid).map_err(|error| input("sew_to_solid_volume", error))?;
    }
    if let Ok(solid) = crate::solid_from_sheets(&pieces, tolerance) {
        let volume = crate::solid_signed_volume(&solid).unwrap_or(f64::NAN);
        // A dent in place of a union is off by twice the gap's volume, so the
        // check is bounded by the gap as well as the total (relative only: an
        // absolute floor would pass a dent on a small part), and the result
        // must be one shell (overlapping shells sum to the same volume).
        let allowance = (1e-6 * expected.abs()).min(0.1 * gap_volume.abs());
        if solid.shells.len() == 1 && (volume - expected).abs() <= allowance {
            united = Some(solid);
        }
    }
    let mut solid = match united {
        Some(solid) => solid,
        None => {
            notes.push("the faces cut into a picked solid; united by the boolean".into());
            let mut operands = vec![gap];
            for &(index, cap) in &caps {
                if *cap != solids[index].all_faces() {
                    operands.push(solids[index].solid.clone());
                }
            }
            common::union_region_solids(operands)
                .map_err(|error| input("sew_to_solid_union", format!("the closed volume does not unite with the solids: {error}")))?
        }
    };
    // Solids no face touches are united by the boolean.
    let untouched: Vec<&Body> = (0..solids.len()).filter(|index| !caps.iter().any(|(capped, _)| capped == index)).map(|index| solids[index]).collect();
    if !untouched.is_empty() {
        let mut operands = vec![solid];
        operands.extend(untouched.iter().map(|body| body.solid.clone()));
        solid = common::union_region_solids(operands)
            .map_err(|error| input("sew_to_solid_union", format!("the closed volume does not unite with the picked solids: {error}")))?;
    }
    consumed.extend(solids.iter().map(|body| body.name.clone()));
    Ok(Closed { solid, consumed, notes })
}

/// A solid whose faces were picked one by one (and not the solid itself):
/// those faces are its cap as picked. A solid picked whole searches its caps,
/// whatever of its faces are picked besides.
fn fixed_cap(body: &Body) -> bool {
    !body.whole && !body.faces.is_empty()
}

/// Each shell of `solid` as its own solid, united by the boolean.
fn one_volume(solid: &BrepSolid, tolerance: f64) -> Result<BrepSolid, Failure> {
    let mut lumps = Vec::with_capacity(solid.shells.len());
    for shell in &solid.shells {
        let faces: BTreeSet<u64> = shell.faces.iter().map(|face| face.id).collect();
        lumps.push(crate::solid_from_sheets(&[extract(solid, &faces)], tolerance)?);
    }
    common::union_region_solids(lumps)
        .map_err(|error| input("sew_to_solid_union", format!("the closed volumes do not unite: {error}")))
}

/// The 1e-6 of the picked geometry's diagonal (never below 1e-9).
fn auto_tolerance(bodies: &[Body]) -> f64 {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in bodies.iter().flat_map(|body| &body.solid.vertices) {
        let p = vertex.point;
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    let diagonal = high.sub(low).length();
    if diagonal.is_finite() {
        (AUTO_TOLERANCE_FRACTION * diagonal).max(1e-9)
    } else {
        1e-6
    }
}

/// Suffix every face name with `::{suffix}` (a copy of a body that stays).
fn rename(piece: &mut BrepSolid, suffix: &str) {
    for face in piece.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        if let Some(name) = face.name.as_ref() {
            face.name = Some(format!("{name}::{suffix}"));
        }
    }
}

/// The faces `faces` of `solid` as one sheet, with only the edges and
/// vertices they use.
pub(crate) fn extract(solid: &BrepSolid, faces: &BTreeSet<u64>) -> BrepSolid {
    let shells: Vec<ShellRecord> = solid
        .shells
        .iter()
        .map(|shell| ShellRecord {
            id: shell.id,
            faces: shell.faces.iter().filter(|face| faces.contains(&face.id)).cloned().collect(),
        })
        .filter(|shell| !shell.faces.is_empty())
        .collect();
    let used: HashSet<u64> = shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|record| &record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    let edges: Vec<EdgeRecord> = solid.edges.iter().filter(|edge| used.contains(&edge.id)).cloned().collect();
    let points: HashSet<u64> = edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect();
    let vertices: Vec<VertexRecord> = solid.vertices.iter().filter(|vertex| points.contains(&vertex.id)).cloned().collect();
    BrepSolid {
        mass_properties_cache: Default::default(),
        id: solid.id,
        vertices,
        edges,
        shells,
        genus: 0,
        kind: BodyKind::Sheet,
        cells: Default::default(),
    }
}

/// `pieces` as one body, every id made new, unsewn.
fn join(pieces: &[BrepSolid]) -> BrepSolid {
    let mut next = 0u64;
    let mut id = || {
        next += 1;
        next
    };
    let (mut vertices, mut edges, mut shells) = (Vec::new(), Vec::new(), Vec::new());
    for piece in pieces {
        let vertex_ids: HashMap<u64, u64> = piece.vertices.iter().map(|vertex| (vertex.id, id())).collect();
        let edge_ids: HashMap<u64, u64> = piece.edges.iter().map(|edge| (edge.id, id())).collect();
        for vertex in &piece.vertices {
            vertices.push(VertexRecord { id: vertex_ids[&vertex.id], point: vertex.point });
        }
        for edge in &piece.edges {
            let mut record = edge.clone();
            record.id = edge_ids[&edge.id];
            record.start_vertex_id = vertex_ids.get(&edge.start_vertex_id).copied().unwrap_or(0);
            record.end_vertex_id = vertex_ids.get(&edge.end_vertex_id).copied().unwrap_or(0);
            edges.push(record);
        }
        for shell in &piece.shells {
            let mut faces = Vec::with_capacity(shell.faces.len());
            for face in &shell.faces {
                let mut record = face.clone();
                record.id = id();
                for loop_record in &mut record.loops {
                    loop_record.id = id();
                    for coedge in &mut loop_record.coedges {
                        coedge.id = id();
                        coedge.edge_id = edge_ids.get(&coedge.edge_id).copied().unwrap_or(0);
                    }
                }
                faces.push(record);
            }
            shells.push(ShellRecord { id: id(), faces });
        }
    }
    BrepSolid {
        mass_properties_cache: Default::default(),
        id: id(),
        vertices,
        edges,
        shells,
        genus: 0,
        kind: BodyKind::Sheet,
        cells: Default::default(),
    }
}

/// `pieces` sewn within `tolerance`, and the edges left bounding one face.
fn sew(pieces: &[BrepSolid], tolerance: f64) -> Result<(BrepSolid, Vec<EdgeRecord>), Failure> {
    let (sewn, _) = crate::sew_solid(&join(pieces), tolerance)?;
    let open = open_edges(&sewn);
    Ok((sewn, open))
}

/// The non-degenerate edges that bound fewer than two faces.
fn open_edges(solid: &BrepSolid) -> Vec<EdgeRecord> {
    let mut uses = HashMap::<u64, usize>::new();
    for coedge in solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|record| &record.coedges)
    {
        *uses.entry(coedge.edge_id).or_default() += 1;
    }
    solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate && uses.get(&edge.id).copied().unwrap_or(0) == 1)
        .cloned()
        .collect()
}

fn edge_point(edge: &EdgeRecord, fraction: f64) -> Option<Vec3> {
    edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).ok()
}

/// Worst distance from `piece`'s samples to `carrier`'s curve (infinite when
/// a sample does not evaluate or project).
fn deviation(piece: &EdgeRecord, carrier: &EdgeRecord) -> f64 {
    let mut worst = 0.0f64;
    for index in 0..=EDGE_SAMPLES {
        let Some(point) = edge_point(piece, index as f64 / EDGE_SAMPLES as f64) else {
            return f64::INFINITY;
        };
        match crate::project_point_to_curve(&carrier.curve, point) {
            Ok(hit) => worst = worst.max(hit.distance),
            Err(_) => return f64::INFINITY,
        }
    }
    worst
}

/// The edges of `solid` that lie on an open edge of the free faces, end to
/// end, within `tolerance`.
fn cut_edges(solid: &BrepSolid, open: &[EdgeRecord], tolerance: f64) -> Result<HashSet<u64>, Failure> {
    let ends = |edge: &EdgeRecord| -> Option<(Vec3, Vec3)> { Some((edge_point(edge, 0.0)?, edge_point(edge, 1.0)?)) };
    let mut cut = HashSet::new();
    for edge in solid.edges.iter().filter(|edge| !edge.degenerate) {
        let Some((start, end)) = ends(edge) else { continue };
        for other in open {
            let Some((other_start, other_end)) = ends(other) else { continue };
            let direct = start.sub(other_start).length() <= tolerance && end.sub(other_end).length() <= tolerance;
            let reversed = start.sub(other_end).length() <= tolerance && end.sub(other_start).length() <= tolerance;
            if (direct || reversed) && deviation(edge, other) <= tolerance && deviation(other, edge) <= tolerance {
                cut.insert(edge.id);
                break;
            }
        }
    }
    Ok(cut)
}

/// `solid`'s faces grouped by the edges that join them, `cut` edges joining
/// nothing.
fn components(solid: &BrepSolid, cut: &HashSet<u64>) -> Vec<BTreeSet<u64>> {
    let faces: Vec<u64> = solid.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).collect();
    let index: HashMap<u64, usize> = faces.iter().enumerate().map(|(position, id)| (*id, position)).collect();
    let mut parent: Vec<usize> = (0..faces.len()).collect();
    fn root(parent: &mut [usize], mut at: usize) -> usize {
        while parent[at] != at {
            parent[at] = parent[parent[at]];
            at = parent[at];
        }
        at
    }
    let mut by_edge: HashMap<u64, Vec<usize>> = HashMap::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
            if !cut.contains(&coedge.edge_id) {
                by_edge.entry(coedge.edge_id).or_default().push(index[&face.id]);
            }
        }
    }
    for users in by_edge.values() {
        for pair in users.windows(2) {
            let (a, b) = (root(&mut parent, pair[0]), root(&mut parent, pair[1]));
            parent[a] = b;
        }
    }
    let mut groups: BTreeMap<usize, BTreeSet<u64>> = BTreeMap::new();
    for (position, id) in faces.iter().enumerate() {
        let top = root(&mut parent, position);
        groups.entry(top).or_default().insert(*id);
    }
    groups.into_values().collect()
}

/// The edge's curve over its own parameter range.
fn edge_curve(edge: &EdgeRecord) -> NurbsCurve {
    let trimmed = || -> Result<NurbsCurve, String> {
        let [low, high] = edge.curve.domain()?;
        let (t0, t1) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
        let mut curve = edge.curve.clone();
        let span = high - low;
        if t1 < high - 1e-12 * span.abs().max(1.0) {
            curve = curve.split(t1)?.0;
        }
        if t0 > low + 1e-12 * span.abs().max(1.0) {
            curve = curve.split(t0)?.1;
        }
        Ok(curve)
    };
    trimmed().unwrap_or_else(|_| edge.curve.clone())
}

fn edge_length(edge: &EdgeRecord) -> f64 {
    const STEPS: usize = 64;
    let mut length = 0.0;
    let mut last = edge_point(edge, 0.0);
    for index in 1..=STEPS {
        let point = edge_point(edge, index as f64 / STEPS as f64);
        if let (Some(a), Some(b)) = (last, point) {
            length += b.sub(a).length();
        }
        last = point;
    }
    length
}

/// The refusal for faces that leave `open` edges, each published as a
/// `{id}:GAP[n]` path to draw.
fn open_refusal(open: &[EdgeRecord], sewn: &BrepSolid, id: &str) -> Failure {
    let _ = sewn;
    let total: f64 = open.iter().map(edge_length).sum();
    // Each open edge's distance to its nearest open neighbour: what the
    // tolerance would have to be to sew it.
    let mut largest = 0.0f64;
    let mut widest: Option<usize> = None;
    for (index, edge) in open.iter().enumerate() {
        let nearest = open
            .iter()
            .enumerate()
            .filter(|(other, _)| *other != index)
            .map(|(_, other)| deviation(edge, other))
            .fold(f64::INFINITY, f64::min);
        if nearest.is_finite() && nearest > largest {
            largest = nearest;
            widest = Some(index);
        }
    }
    let prefix = if id.is_empty() { "SEW".to_string() } else { id.to_string() };
    let gap = match widest {
        Some(index) => format!("largest gap {largest:.3e} (at {prefix}:GAP[{index}])"),
        None => "no open edge has a neighbour to sew to".to_string(),
    };
    let message = format!(
        "Sew to Solid: the faces do not close a volume: {} open edge(s), total length {total:.6}, {gap}; the open edges are drawn as {prefix}:GAP[n]. Fill the hole they bound, pick the solid whose edges they lie on, or raise the tolerance past the gap",
        open.len()
    );
    let gaps = open
        .iter()
        .enumerate()
        .map(|(index, edge)| (format!("{prefix}:GAP[{index}]"), vec![edge_curve(edge)]))
        .collect();
    Failure {
        refusal: KernelRefusal::input(KernelStage::Sew, "sew_to_solid_open", message),
        gaps,
    }
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// two or more selected faces or bodies drive `faces` (one face alone
/// encloses nothing).
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces + probe.solids >= 2
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SEW",
    "shortName": "SEW",
    "longName": "Sew to Solid",
    "ribbonPath": "Home/Modeling/Sew to Solid",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier for the sew to solid feature"
        },
        "faces": {
            "type": "reference_selection",
            "label": "Faces and bodies",
            "selectionFilter": ["FACE", "SOLID"],
            "multiple": true,
            "default_value": [],
            "hint": "The sheets, faces and solids that together enclose a volume. A picked solid closes the volume with its faces where the others meet its edges"
        },
        "tolerance": {
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "How far apart two edges may lie and still be sewn together. 0 is automatic (a millionth of the picked geometry's size)"
        },
        "unite": {
            "type": "boolean",
            "label": "Unite with picked solids",
            "default_value": true,
            "hint": "Unite the closed volume with the picked solids into one solid. Off: the closed volume alone, the solids stay as they are"
        }
    }
})
}

