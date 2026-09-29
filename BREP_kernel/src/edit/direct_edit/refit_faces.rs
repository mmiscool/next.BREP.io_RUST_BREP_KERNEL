//! Refit Faces: replace a connected set of faces of one solid with ONE face on
//! a surface fitted to them — the human-guided cleanup after a mesh import,
//! where the automatic recognition left a curved wall as many small facets.
//!
//! The fit is the mesh import's own single-region pass
//! ([`brep_ransac::reconstruct_surface`]: RANSAC hypotheses, then a
//! least-squares refine), run on the selected faces' vertices (a facet's
//! vertices lie on the surface it approximates; its interior does not) plus
//! the tessellation of any selected face that is already curved.
//!
//! The new face is trimmed by the region's own boundary, rebuilt EXACTLY: every
//! run of boundary edges shared with one neighbour becomes a single edge on the
//! curve where the fitted surface meets that neighbour's carrier
//! ([`crate::intersect_analytic_pair`]), so a faceted cylinder's polygonal cap
//! comes back bounded by a true circle. The neighbours keep their ids, names and
//! carriers; only the loop run they shared with the region is replaced.
//!
//! Scope (v1), each refused by name with nothing changed:
//! - the faces must be edge-connected, on one shell;
//! - the region must be a disk (one boundary loop) or an annulus (two, e.g. a
//!   full cylindrical band between two caps);
//! - the fit's maximum vertex residual must be within the caller's tolerance;
//! - every boundary run must lie on an exact analytic intersection of the
//!   fitted surface with its neighbour's carrier;
//! - the result must pass `validate()` and the soundness floor.

use crate::{KernelRefusal, KernelStage, RefusalClass};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{
    build_pcurve_on_surface_range, intersect_analytic_pair, make_arc, make_line, make_plane,
    make_revolution, project_point_to_curve, solid_model_scale, NurbsCurve, NurbsSurface, Vec3,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// The surface family a Refit Faces fit is asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum RefitSurfaceKind {
    /// The best of the five by the recognition pass's own model ranking.
    Auto,
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
}

impl RefitSurfaceKind {
    /// Parse the feature's `surfaceType` option (case-insensitive).
    pub fn parse(text: &str) -> Option<Self> {
        match text.trim().to_ascii_lowercase().as_str() {
            "" | "auto" => Some(Self::Auto),
            "plane" | "planar" => Some(Self::Plane),
            "cylinder" | "cylindrical" => Some(Self::Cylinder),
            "cone" | "conical" => Some(Self::Cone),
            "sphere" | "spherical" => Some(Self::Sphere),
            "torus" | "toroidal" => Some(Self::Torus),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::Plane => "plane",
            Self::Cylinder => "cylinder",
            Self::Cone => "cone",
            Self::Sphere => "sphere",
            Self::Torus => "torus",
        }
    }
}

/// The phrases a Refit Faces refusal is recognised by (formatted into every
/// refusal that carries it, so a reader and the mint share one definition).
pub mod refit_refusal_phrases {
    pub const NOT_EDGE_CONNECTED: &str = "not edge-connected";
    pub const RESIDUAL_OVER_TOLERANCE: &str = "fit residual";
    pub const NOT_DISK_OR_ANNULUS: &str = "not a disk or an annulus";
    pub const NO_EXACT_BOUNDARY: &str = "no exact boundary";
    pub const NO_FIT: &str = "no surface fits";
    pub const RESULT_INVALID: &str = "result does not validate";
    pub const FACETS_TOO_COARSE: &str = "too coarse";
    pub const CLOSED_SHELL_KIND: &str = "only a sphere or a torus closes on itself";
    pub const CLOSED_SHELL_GENUS: &str = "does not match the shell's topology";
}
use refit_refusal_phrases::*;

/// The fitted surface, in the recognition pass's terms.
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum RefitCarrier {
    Plane { origin: Vec3, normal: Vec3 },
    Cylinder { axis_origin: Vec3, axis: Vec3, radius: f64 },
    Cone { apex: Vec3, axis: Vec3, half_angle: f64 },
    Sphere { center: Vec3, radius: f64 },
    Torus { center: Vec3, axis: Vec3, major_radius: f64, minor_radius: f64 },
}

impl RefitCarrier {
    pub fn kind(&self) -> RefitSurfaceKind {
        match self {
            Self::Plane { .. } => RefitSurfaceKind::Plane,
            Self::Cylinder { .. } => RefitSurfaceKind::Cylinder,
            Self::Cone { .. } => RefitSurfaceKind::Cone,
            Self::Sphere { .. } => RefitSurfaceKind::Sphere,
            Self::Torus { .. } => RefitSurfaceKind::Torus,
        }
    }

    /// Unsigned distance from `point` to the (untrimmed) carrier.
    pub fn distance(&self, point: Vec3) -> f64 {
        match *self {
            Self::Plane { origin, normal } => point.sub(origin).dot(normal).abs(),
            Self::Cylinder { axis_origin, axis, radius } => {
                let rel = point.sub(axis_origin);
                (rel.sub(axis.scale(rel.dot(axis))).length() - radius).abs()
            }
            Self::Cone { apex, axis, half_angle } => {
                let rel = point.sub(apex);
                let along = rel.dot(axis);
                let radial = rel.sub(axis.scale(along)).length();
                // Distance to the generator line in the (along, radial) half-plane.
                let (s, c) = half_angle.sin_cos();
                let normal_offset = radial * c - along * s;
                let foot = along * c + radial * s;
                if foot >= 0.0 {
                    normal_offset.abs()
                } else {
                    rel.length()
                }
            }
            Self::Sphere { center, radius } => (point.sub(center).length() - radius).abs(),
            Self::Torus { center, axis, major_radius, minor_radius } => {
                let rel = point.sub(center);
                let along = rel.dot(axis);
                let radial = rel.sub(axis.scale(along)).length();
                ((radial - major_radius).hypot(along) - minor_radius).abs()
            }
        }
    }

    /// A one-line reading of the parameters, for the feature's report.
    pub fn describe(&self) -> String {
        let v = |p: Vec3| format!("({:.6}, {:.6}, {:.6})", p.x, p.y, p.z);
        match *self {
            Self::Plane { origin, normal } => {
                format!("plane through {} normal {}", v(origin), v(normal))
            }
            Self::Cylinder { axis_origin, axis, radius } => format!(
                "cylinder radius {radius:.6} axis {} through {}",
                v(axis),
                v(axis_origin)
            ),
            Self::Cone { apex, axis, half_angle } => format!(
                "cone half-angle {:.4} deg apex {} axis {}",
                half_angle.to_degrees(),
                v(apex),
                v(axis)
            ),
            Self::Sphere { center, radius } => {
                format!("sphere radius {radius:.6} center {}", v(center))
            }
            Self::Torus { center, axis, major_radius, minor_radius } => format!(
                "torus radii {major_radius:.6}/{minor_radius:.6} center {} axis {}",
                v(center),
                v(axis)
            ),
        }
    }
}

/// What a successful refit measured and built.
#[derive(Clone, Debug, Serialize)]
pub struct RefitReport {
    pub carrier: RefitCarrier,
    /// Root-mean-square distance of the fitted samples from the carrier.
    pub rms_residual: f64,
    /// Largest distance of any fitted sample from the carrier.
    pub max_residual: f64,
    pub tolerance: f64,
    pub samples: usize,
    pub replaced_faces: usize,
    /// Boundary edges of the region before, and the exact edges replacing them.
    pub boundary_edges_before: usize,
    pub boundary_edges_after: usize,
    pub boundary_loops: usize,
    pub new_face_id: u64,
}

fn refuse_input(what: &str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("Refit Faces: {message}"))
}

fn refuse_unsupported(stage: KernelStage, what: &str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(stage, what, format!("Refit Faces: {message}"))
}

fn face_label(face: &FaceRecord) -> String {
    face.name.clone().unwrap_or_else(|| format!("face {}", face.id))
}

#[derive(Clone, Copy, Debug)]
struct BoundaryUse {
    edge_id: u64,
    start: u64,
    end: u64,
    /// The neighbouring (unselected) face index across this edge.
    neighbour: usize,
}

/// One run of boundary coedges shared with a single neighbour.
struct Chain {
    uses: Vec<BoundaryUse>,
    closed: bool,
    neighbour: usize,
}

/// A rebuilt boundary edge: the curve, its range, and how the REGION's
/// coedge runs along it.
struct BuiltEdge {
    curve: NurbsCurve,
    t0: f64,
    t1: f64,
    start: u64,
    end: u64,
    region_forward: bool,
    /// The region-side trim, when it is built exactly here (an iso ring of
    /// the fitted surface: a straight line in its parameter plane).
    region_pcurve: Option<NurbsCurve>,
}

/// Replace the faces `face_ids` of `solid` with one face on a surface of
/// family `kind` fitted to them, within `tolerance` (maximum vertex residual,
/// model units). See the module docs for the scope and the refusals.
pub fn refit_faces_as_one(
    solid: &BrepSolid,
    face_ids: &[u64],
    kind: RefitSurfaceKind,
    tolerance: f64,
) -> Result<(BrepSolid, RefitReport), KernelRefusal> {
    let (result, report) = refit_faces_as_one_impl(solid, face_ids, kind, tolerance)?;
    let result = crate::accept_sound(result, "refitFaces")?;
    Ok((result, report))
}

fn refit_faces_as_one_impl(
    solid: &BrepSolid,
    face_ids: &[u64],
    kind: RefitSurfaceKind,
    tolerance: f64,
) -> Result<(BrepSolid, RefitReport), KernelRefusal> {
    if !(tolerance.is_finite() && tolerance > 0.0) {
        return Err(refuse_input(
            "tolerance",
            format!("tolerance must be a positive number, got {tolerance}"),
        ));
    }
    let mut unique: Vec<u64> = Vec::new();
    for id in face_ids {
        if !unique.contains(id) {
            unique.push(*id);
        }
    }
    if unique.is_empty() {
        return Err(refuse_input("face_ids", "no faces selected".into()));
    }
    // ---- Locate the faces: one shell. --------------------------------------
    let mut shell_index = None;
    let mut selected_indices: Vec<usize> = Vec::new();
    for id in &unique {
        let found = solid.shells.iter().enumerate().find_map(|(s, shell)| {
            shell.faces.iter().position(|face| face.id == *id).map(|f| (s, f))
        });
        let Some((s, f)) = found else {
            return Err(refuse_input("face_ids", format!("no face with id {id}")));
        };
        if *shell_index.get_or_insert(s) != s {
            return Err(refuse_input(
                "face_ids",
                format!("the selected faces are {NOT_EDGE_CONNECTED}: they lie on different shells"),
            ));
        }
        selected_indices.push(f);
    }
    let shell_index = shell_index.unwrap_or(0);
    let shell = &solid.shells[shell_index];
    let selected: HashSet<usize> = selected_indices.iter().copied().collect();
    let scale = solid_model_scale(solid).max(1e-9);
    let fit_tol = (scale * 1e-7).max(1e-9);

    let edge_by_id: HashMap<u64, &EdgeRecord> =
        solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let vertex_by_id: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();

    // ---- Edge uses within the shell. ---------------------------------------
    let mut uses: HashMap<u64, Vec<(usize, usize, usize)>> = HashMap::default();
    for (f, face) in shell.faces.iter().enumerate() {
        for (l, lp) in face.loops.iter().enumerate() {
            for (c, coedge) in lp.coedges.iter().enumerate() {
                uses.entry(coedge.edge_id).or_default().push((f, l, c));
            }
        }
    }

    // ---- Connectivity over shared edges. -----------------------------------
    {
        let mut seen: HashSet<usize> = HashSet::default();
        let mut stack = vec![selected_indices[0]];
        seen.insert(selected_indices[0]);
        while let Some(f) = stack.pop() {
            for lp in &shell.faces[f].loops {
                for coedge in &lp.coedges {
                    for &(g, _, _) in uses.get(&coedge.edge_id).into_iter().flatten() {
                        if selected.contains(&g) && seen.insert(g) {
                            stack.push(g);
                        }
                    }
                }
            }
        }
        if seen.len() != selected.len() {
            let stray: Vec<String> = selected_indices
                .iter()
                .filter(|f| !seen.contains(f))
                .map(|&f| face_label(&shell.faces[f]))
                .collect();
            return Err(refuse_input(
                "face_ids",
                format!(
                    "the selected faces are {NOT_EDGE_CONNECTED}: {} {} no edge with the rest",
                    stray.join(", "),
                    if stray.len() == 1 { "shares" } else { "share" }
                ),
            ));
        }
    }

    // ---- Classify the region's edges; collect its boundary. ----------------
    let mut internal_edges: HashSet<u64> = HashSet::default();
    let mut boundary: Vec<BoundaryUse> = Vec::new();
    let mut region_vertices: HashSet<u64> = HashSet::default();
    let mut loop_total = 0usize;
    for &f in &selected_indices {
        loop_total += shell.faces[f].loops.len();
        for lp in &shell.faces[f].loops {
            for coedge in &lp.coedges {
                let edge = edge_by_id.get(&coedge.edge_id).ok_or_else(|| {
                    refuse_input("solid", format!("coedge names missing edge {}", coedge.edge_id))
                })?;
                region_vertices.insert(edge.start_vertex_id);
                region_vertices.insert(edge.end_vertex_id);
                let edge_uses = &uses[&coedge.edge_id];
                let outside: Vec<usize> = edge_uses
                    .iter()
                    .map(|(g, _, _)| *g)
                    .filter(|g| !selected.contains(g))
                    .collect();
                if outside.is_empty() {
                    internal_edges.insert(coedge.edge_id);
                    continue;
                }
                if edge_uses.len() != 2 || outside.len() != 1 {
                    return Err(refuse_unsupported(
                        KernelStage::Classify,
                        "non_manifold_boundary",
                        format!(
                            "edge {} of the selection is used by {} faces; the region is \
                             {NOT_DISK_OR_ANNULUS}",
                            coedge.edge_id,
                            edge_uses.len()
                        ),
                    ));
                }
                let (start, end) = if coedge.forward {
                    (edge.start_vertex_id, edge.end_vertex_id)
                } else {
                    (edge.end_vertex_id, edge.start_vertex_id)
                };
                boundary.push(BoundaryUse {
                    edge_id: coedge.edge_id,
                    start,
                    end,
                    neighbour: outside[0],
                });
            }
        }
    }
    if boundary.is_empty() {
        // Every face of one closed shell: the shell itself becomes the
        // primitive (v2). Only a sphere or a torus closes on itself.
        let euler = region_vertices.len() as i64 - internal_edges.len() as i64
            + (2 * selected_indices.len() as i64 - loop_total as i64);
        return refit_closed_shell(
            solid,
            shell_index,
            &selected_indices,
            euler,
            kind,
            tolerance,
            scale,
            &edge_by_id,
            &vertex_by_id,
        );
    }

    // ---- Walk the boundary into cycles. ------------------------------------
    let mut outgoing: HashMap<u64, usize> = HashMap::default();
    for (index, use_) in boundary.iter().enumerate() {
        if outgoing.insert(use_.start, index).is_some() {
            return Err(refuse_unsupported(
                KernelStage::Classify,
                "pinched_region",
                format!(
                    "the region's boundary touches itself at a vertex; the region is \
                     {NOT_DISK_OR_ANNULUS}"
                ),
            ));
        }
    }
    let mut cycles: Vec<Vec<BoundaryUse>> = Vec::new();
    let mut visited = vec![false; boundary.len()];
    for first in 0..boundary.len() {
        if visited[first] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut at = first;
        loop {
            if visited[at] {
                break;
            }
            visited[at] = true;
            cycle.push(boundary[at]);
            match outgoing.get(&boundary[at].end) {
                Some(&next) => at = next,
                None => {
                    return Err(refuse_unsupported(
                        KernelStage::Classify,
                        "open_boundary",
                        format!("the region's boundary does not close; it is {NOT_DISK_OR_ANNULUS}"),
                    ))
                }
            }
        }
        if at != first {
            return Err(refuse_unsupported(
                KernelStage::Classify,
                "open_boundary",
                format!("the region's boundary does not close; it is {NOT_DISK_OR_ANNULUS}"),
            ));
        }
        cycles.push(cycle);
    }
    // Euler characteristic of the region as a surface: V - E + sum(2 - loops).
    let boundary_edges: HashSet<u64> = boundary.iter().map(|b| b.edge_id).collect();
    let euler = region_vertices.len() as i64
        - (internal_edges.len() + boundary_edges.len()) as i64
        + (2 * selected_indices.len() as i64 - loop_total as i64);
    if !(cycles.len() == 1 || cycles.len() == 2) || euler != 2 - cycles.len() as i64 {
        return Err(refuse_unsupported(
            KernelStage::Classify,
            "region_topology",
            format!(
                "the selected region has {} boundary loop(s) and Euler characteristic {euler}; \
                 it is {NOT_DISK_OR_ANNULUS}, which one face can take",
                cycles.len()
            ),
        ));
    }

    // ---- Fit. ---------------------------------------------------------------
    let samples = fit_samples(shell, &selected_indices, &edge_by_id, &vertex_by_id)?;
    let fitted = fit_carrier(&samples, kind, tolerance, scale)?;
    let carrier = fitted.carrier;
    let (rms_residual, max_residual) = residuals(&carrier, &samples.points);
    if max_residual > tolerance {
        return Err(KernelRefusal::ill_posed(
            KernelStage::Refine,
            "refit_residual",
            format!(
                "Refit Faces: the {} {RESIDUAL_OVER_TOLERANCE} {max_residual:.6} (rms \
                 {rms_residual:.6}) is over the tolerance {tolerance}; these faces are not \
                 one {} within it",
                carrier.kind().name(),
                carrier.kind().name()
            ),
        ));
    }

    coarse_facet_gate(&carrier, shell, &selected_indices, &edge_by_id, &vertex_by_id)?;

    // Every boundary vertex, too, must lie on the carrier: the boundary is
    // what the new face is trimmed by.
    let surface = carrier_surface(&carrier, &samples.points, scale)?;

    // Orientation: agree with the selected faces' outward sense.
    let same_sense = region_same_sense(&surface, &samples)?;

    // ---- Chains: runs of boundary shared with one neighbour. ---------------
    let mut chains_per_cycle: Vec<Vec<Chain>> = Vec::new();
    for cycle in &cycles {
        chains_per_cycle.push(split_chains(cycle));
    }

    // ---- Rebuild each chain as one exact edge. ------------------------------
    let chain_tol = (tolerance * 2.0).max(fit_tol * 10.0);
    let mut next_edge_id = solid.edges.iter().map(|e| e.id).max().unwrap_or(0) + 1;
    let mut next_vertex_id = solid.vertices.iter().map(|v| v.id).max().unwrap_or(0) + 1;
    let mut moved_vertices: HashMap<u64, Vec3> = HashMap::default();
    let mut new_vertices: Vec<VertexRecord> = Vec::new();

    // Candidate curves per chain.
    let mut chain_curves: Vec<Vec<NurbsCurve>> = Vec::new();
    for chains in &chains_per_cycle {
        for chain in chains {
            let neighbour = &shell.faces[chain.neighbour];
            let points = chain_points(chain, &vertex_by_id);
            // The closed-form intersection when the kernel knows the pair; an
            // iso circle of the fitted surface that provably lies on the
            // neighbour otherwise (a plane square to a torus's axis).
            let mut curves = intersect_analytic_pair(&surface, &neighbour.surface, fit_tol)
                .unwrap_or_default();
            // Prefer the exact intersection. An approximate iso ruling can
            // be closer to a chord's trim vertices while missing the neighbour
            // carrier; choosing it would leave a gap in the rebuilt shell.
            if curves.is_empty() {
                curves = iso_curve_on(&surface, &neighbour.surface, &points, chain_tol);
            }
            if curves.is_empty() {
                return Err(refuse_unsupported(
                    KernelStage::Intersect,
                    "no_exact_boundary",
                    format!(
                        "{NO_EXACT_BOUNDARY} with {}: its carrier is not one whose \
                         intersection with the fitted {} is known in closed form",
                        face_label(neighbour),
                        carrier.kind().name()
                    ),
                ));
            }
            let mut best: Option<(f64, NurbsCurve)> = None;
            for curve in curves {
                let mut worst = 0.0_f64;
                for point in &points {
                    let d = project_point_to_curve(&curve, *point)
                        .map(|p| p.distance)
                        .unwrap_or(f64::INFINITY);
                    worst = worst.max(d);
                }
                if best.as_ref().map_or(true, |(w, _)| worst < *w) {
                    best = Some((worst, curve));
                }
            }
            match best {
                Some((worst, curve)) if worst <= chain_tol => chain_curves.push(vec![curve]),
                Some((worst, _)) => {
                    return Err(refuse_unsupported(
                        KernelStage::Intersect,
                        "no_exact_boundary",
                        format!(
                            "{NO_EXACT_BOUNDARY} with {}: the shared edges sit {worst:.6} off \
                             the curve where the fitted {} meets it (allowed {chain_tol:.6})",
                            face_label(neighbour),
                            carrier.kind().name()
                        ),
                    ))
                }
                None => {
                    return Err(refuse_unsupported(
                        KernelStage::Intersect,
                        "no_exact_boundary",
                        format!(
                            "{NO_EXACT_BOUNDARY} with {}: the fitted {} does not meet it",
                            face_label(neighbour),
                            carrier.kind().name()
                        ),
                    ))
                }
            }
        }
    }

    // Corners: where one chain hands over to the next, the vertex is moved to
    // the meeting point of the two curves.
    {
        let mut flat = 0usize;
        for chains in &chains_per_cycle {
            let base = flat;
            let count = chains.len();
            for index in 0..count {
                let chain = &chains[index];
                flat += 1;
                if chain.closed {
                    continue;
                }
                let next = (index + 1) % count;
                let corner = chain.uses.last().expect("chain has uses").end;
                let original = vertex_by_id[&corner];
                let first = &chain_curves[base + index][0];
                let second = &chain_curves[base + next][0];
                let (point, gap) = curve_meeting(first, second, original)?;
                if gap > chain_tol {
                    return Err(refuse_unsupported(
                        KernelStage::Intersect,
                        "corner_gap",
                        format!(
                            "{NO_EXACT_BOUNDARY}: at a corner the rebuilt edges along {} and \
                             {} miss each other by {gap:.6}",
                            face_label(&shell.faces[chain.neighbour]),
                            face_label(&shell.faces[chains[next].neighbour])
                        ),
                    ));
                }
                moved_vertices.insert(corner, point);
            }
        }
    }

    let mut built: Vec<Vec<BuiltEdge>> = Vec::new();
    {
        let mut flat = 0usize;
        for chains in &chains_per_cycle {
            let mut row = Vec::new();
            for chain in chains {
                let curve = chain_curves[flat][0].clone();
                flat += 1;
                let points = chain_points(chain, &vertex_by_id);
                let edge = if chain.closed {
                    if let Some((curve, pcurve, region_forward)) =
                        iso_ring(&surface, &curve, &points, chain_tol)
                    {
                        let [t0, t1] = curve.domain().map_err(|e| refuse_input("curve", e))?;
                        let point = curve.evaluate(t0).map_err(|e| refuse_input("curve", e))?;
                        let id = next_vertex_id;
                        next_vertex_id += 1;
                        new_vertices.push(VertexRecord { id, point });
                        row.push(BuiltEdge {
                            curve,
                            t0,
                            t1,
                            start: id,
                            end: id,
                            region_forward,
                            region_pcurve: Some(pcurve),
                        });
                        continue;
                    }
                    let start_point = points[0];
                    let (curve, region_forward) = closed_ring_curve(&curve, &points, scale)?;
                    let start_point = curve.evaluate(curve.domain().map_err(|e| {
                        refuse_input("curve", e)
                    })?[0])
                    .unwrap_or(start_point);
                    let id = next_vertex_id;
                    next_vertex_id += 1;
                    new_vertices.push(VertexRecord { id, point: start_point });
                    let [t0, t1] = curve.domain().map_err(|e| refuse_input("curve", e))?;
                    BuiltEdge { curve, t0, t1, start: id, end: id, region_forward, region_pcurve: None }
                } else {
                    let start = chain.uses.first().unwrap().start;
                    let end = chain.uses.last().unwrap().end;
                    let start_point = moved_vertices.get(&start).copied().unwrap_or(points[0]);
                    let end_point =
                        moved_vertices.get(&end).copied().unwrap_or(*points.last().unwrap());
                    let middle = points[points.len() / 2];
                    let mid_point = if points.len() > 2 {
                        middle
                    } else {
                        points[0].add(points[1]).scale(0.5)
                    };
                    open_arc(&curve, start, end, start_point, end_point, mid_point, scale)?
                };
                row.push(edge);
            }
            built.push(row);
        }
    }

    // ---- Assemble the new solid. --------------------------------------------
    let mut out = solid.clone();
    // Edges to drop: internal and replaced boundary edges.
    let mut dropped_edges: HashSet<u64> = internal_edges.clone();
    dropped_edges.extend(boundary_edges.iter().copied());
    let mut new_edges: Vec<EdgeRecord> = Vec::new();
    let mut new_edge_ids: Vec<Vec<u64>> = Vec::new();
    for row in &built {
        let mut ids = Vec::new();
        for edge in row {
            let id = next_edge_id;
            next_edge_id += 1;
            new_edges.push(EdgeRecord {
                id,
                curve: edge.curve.clone(),
                t0: edge.t0,
                t1: edge.t1,
                start_vertex_id: edge.start,
                end_vertex_id: edge.end,
                degenerate: false,
                name: None,
            });
            ids.push(id);
        }
        new_edge_ids.push(ids);
    }

    // Moved corners: update the points, and re-straighten straight edges that
    // stay (their pcurves follow below).
    for vertex in &mut out.vertices {
        if let Some(point) = moved_vertices.get(&vertex.id) {
            vertex.point = *point;
        }
    }
    let point_of = |out: &BrepSolid, id: u64| -> Vec3 {
        out.vertices
            .iter()
            .find(|v| v.id == id)
            .map(|v| v.point)
            .or_else(|| new_vertices.iter().find(|v| v.id == id).map(|v| v.point))
            .unwrap_or(Vec3::new(0.0, 0.0, 0.0))
    };
    let mut restraightened: HashSet<u64> = HashSet::default();
    let snapshot = out.clone();
    for edge in &mut out.edges {
        if dropped_edges.contains(&edge.id) {
            continue;
        }
        let touches = moved_vertices.contains_key(&edge.start_vertex_id)
            || moved_vertices.contains_key(&edge.end_vertex_id);
        if !touches {
            continue;
        }
        if edge.curve.degree == 1 && edge.curve.control_points.len() == 2 {
            let a = point_of(&snapshot, edge.start_vertex_id);
            let b = point_of(&snapshot, edge.end_vertex_id);
            if let Ok(line) = make_line(a, b) {
                edge.curve = line;
                edge.t0 = 0.0;
                edge.t1 = 1.0;
                restraightened.insert(edge.id);
            }
        }
    }

    // A planar neighbour whose WHOLE boundary is one rebuilt ring (a cap) is
    // re-seated onto the ring's own plane. The ring lies on the fitted
    // surface exactly and on the old cap plane only to the fit's residual —
    // float32 mesh vertices put it ~2e-6 off, which the cap's projected trim
    // turns into a closure residual over the bar (the user's 3MF pipe bend).
    for (cycle_index, chains) in chains_per_cycle.iter().enumerate() {
        for (chain_index, chain) in chains.iter().enumerate() {
            if !chain.closed {
                continue;
            }
            let neighbour = &out.shells[shell_index].faces[chain.neighbour];
            let chain_edges: HashSet<u64> = chain.uses.iter().map(|u| u.edge_id).collect();
            let whole_boundary = neighbour.loops.len() == 1
                && neighbour.loops[0].coedges.iter().all(|c| chain_edges.contains(&c.edge_id));
            if !whole_boundary || !neighbour.surface.is_affine().unwrap_or(false) {
                continue;
            }
            let ring = &built[cycle_index][chain_index].curve;
            if let Some(plane) = reseated_plane(&neighbour.surface, ring, scale) {
                out.shells[shell_index].faces[chain.neighbour].surface = plane;
            }
        }
    }

    let shell_out = &mut out.shells[shell_index];
    let mut flat = 0usize;
    for (cycle_index, chains) in chains_per_cycle.iter().enumerate() {
        for (chain_index, chain) in chains.iter().enumerate() {
            let built_edge = &built[cycle_index][chain_index];
            let edge_id = new_edge_ids[cycle_index][chain_index];
            flat += 1;
            let neighbour = &mut shell_out.faces[chain.neighbour];
            let chain_edges: HashSet<u64> = chain.uses.iter().map(|u| u.edge_id).collect();
            let coedge_forward = !built_edge.region_forward;
            let pcurve = build_pcurve_on_surface_range(
                &neighbour.surface,
                &built_edge.curve,
                built_edge.t0,
                built_edge.t1,
                coedge_forward,
                fit_tol,
            )
            .map_err(|e| {
                refuse_unsupported(
                    KernelStage::Refine,
                    "neighbour_pcurve",
                    format!(
                        "{NO_EXACT_BOUNDARY}: no trim curve on {} for the rebuilt edge ({e})",
                        face_label(neighbour)
                    ),
                )
            })?;
            let mut replaced = false;
            for lp in &mut neighbour.loops {
                let positions: Vec<usize> = lp
                    .coedges
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| chain_edges.contains(&c.edge_id))
                    .map(|(i, _)| i)
                    .collect();
                if positions.is_empty() {
                    continue;
                }
                if positions.len() != chain_edges.len() {
                    return Err(refuse_unsupported(
                        KernelStage::Sew,
                        "neighbour_loop",
                        format!(
                            "the edges shared with {} are split across its loops",
                            face_label(neighbour)
                        ),
                    ));
                }
                // Rotate the loop so the run is contiguous at the front.
                let n = lp.coedges.len();
                let run_start = (0..n)
                    .find(|&i| {
                        chain_edges.contains(&lp.coedges[i].edge_id)
                            && !chain_edges.contains(&lp.coedges[(i + n - 1) % n].edge_id)
                    })
                    .unwrap_or(0);
                lp.coedges.rotate_left(run_start);
                if !lp.coedges[..positions.len()]
                    .iter()
                    .all(|c| chain_edges.contains(&c.edge_id))
                {
                    return Err(refuse_unsupported(
                        KernelStage::Sew,
                        "neighbour_loop",
                        format!(
                            "the edges shared with {} are not one run of its loop",
                            face_label(neighbour)
                        ),
                    ));
                }
                let first_id = lp.coedges[0].id;
                lp.coedges.splice(
                    0..positions.len(),
                    [CoedgeRecord { id: first_id, edge_id, forward: coedge_forward, pcurve }],
                );
                replaced = true;
                break;
            }
            if !replaced {
                return Err(refuse_unsupported(
                    KernelStage::Sew,
                    "neighbour_loop",
                    format!("{} does not use the edges it shares", face_label(neighbour)),
                ));
            }
        }
    }
    let _ = flat;

    // Pcurves of re-straightened edges, on every face that uses them.
    if !restraightened.is_empty() {
        let edges: HashMap<u64, EdgeRecord> = out
            .edges
            .iter()
            .filter(|e| restraightened.contains(&e.id))
            .map(|e| (e.id, e.clone()))
            .collect();
        for face in &mut out.shells[shell_index].faces {
            let surface = face.surface.clone();
            for lp in &mut face.loops {
                for coedge in &mut lp.coedges {
                    if let Some(edge) = edges.get(&coedge.edge_id) {
                        coedge.pcurve = build_pcurve_on_surface_range(
                            &surface,
                            &edge.curve,
                            edge.t0,
                            edge.t1,
                            coedge.forward,
                            fit_tol,
                        )
                        .map_err(|e| {
                            refuse_unsupported(
                                KernelStage::Refine,
                                "neighbour_pcurve",
                                format!("a straight edge at a moved corner lost its trim ({e})"),
                            )
                        })?;
                    }
                }
            }
        }
    }

    // The new face.
    let keeper = &shell.faces[selected_indices[0]];
    let mut next_coedge_id = out
        .shells
        .iter()
        .flat_map(|s| s.faces.iter())
        .flat_map(|f| f.loops.iter())
        .flat_map(|l| l.coedges.iter().map(|c| c.id))
        .max()
        .unwrap_or(0)
        + 1;
    let mut next_loop_id = out
        .shells
        .iter()
        .flat_map(|s| s.faces.iter())
        .flat_map(|f| f.loops.iter().map(|l| l.id))
        .max()
        .unwrap_or(0)
        + 1;
    let mut loops = Vec::new();
    for (cycle_index, row) in built.iter().enumerate() {
        let mut coedges = Vec::new();
        for (chain_index, edge) in row.iter().enumerate() {
            let pcurve = match &edge.region_pcurve {
                Some(pcurve) => Ok(pcurve.clone()),
                None => build_pcurve_on_surface_range(
                &surface,
                &edge.curve,
                edge.t0,
                edge.t1,
                edge.region_forward,
                fit_tol,
            ),
            }
            .map_err(|e| {
                refuse_unsupported(
                    KernelStage::Refine,
                    "region_pcurve",
                    format!("{NO_EXACT_BOUNDARY}: no trim curve on the fitted surface ({e})"),
                )
            })?;
            coedges.push(CoedgeRecord {
                id: next_coedge_id,
                edge_id: new_edge_ids[cycle_index][chain_index],
                forward: edge.region_forward,
                pcurve,
            });
            next_coedge_id += 1;
        }
        loops.push(LoopRecord { id: next_loop_id, coedges });
        next_loop_id += 1;
    }
    let new_face = FaceRecord {
        id: keeper.id,
        surface: surface.clone(),
        same_sense,
        loops,
        name: keeper.name.clone(),
    };
    let new_face_id = new_face.id;
    let shell_out = &mut out.shells[shell_index];
    let keep_at = selected_indices[0];
    let mut faces = Vec::with_capacity(shell_out.faces.len());
    for (index, face) in shell_out.faces.drain(..).enumerate() {
        if index == keep_at {
            faces.push(new_face.clone());
        } else if !selected.contains(&index) {
            faces.push(face);
        }
    }
    shell_out.faces = faces;
    out.edges.retain(|edge| !dropped_edges.contains(&edge.id));
    out.edges.extend(new_edges);
    out.vertices.extend(new_vertices);
    // Drop vertices nothing uses any more.
    let used: HashSet<u64> = out
        .edges
        .iter()
        .flat_map(|e| [e.start_vertex_id, e.end_vertex_id])
        .collect();
    out.vertices.retain(|v| used.contains(&v.id));

    let issues = out.validate();
    if !issues.is_empty() {
        let first: Vec<String> = issues.iter().take(3).map(|i| format!("{i:?}")).collect();
        return Err(KernelRefusal::new(
            RefusalClass::InvalidResultTopology {
                issues: issues.len() as u32,
            },
            KernelStage::Validate,
            format!(
                "Refit Faces: the {RESULT_INVALID} ({} issue(s): {})",
                issues.len(),
                first.join("; ")
            ),
        ));
    }

    let report = RefitReport {
        carrier,
        rms_residual,
        max_residual,
        tolerance,
        samples: samples.points.len(),
        replaced_faces: selected_indices.len(),
        boundary_edges_before: boundary_edges.len(),
        boundary_edges_after: built.iter().map(Vec::len).sum(),
        boundary_loops: built.len(),
        new_face_id,
    };
    Ok((out, report))
}

// ---------------------------------------------------------------------------
// Sampling and fitting
// ---------------------------------------------------------------------------

struct Samples {
    /// Points the carrier must pass through (vertices; curved faces'
    /// tessellation; curved edges' samples).
    points: Vec<Vec3>,
    /// A triangle mesh over the selection for the recognition pass: the
    /// planar faces fanned over their vertices, curved faces tessellated.
    mesh_vertices: Vec<Vec3>,
    mesh_triangles: Vec<[u32; 3]>,
    /// Vertices supported by intersecting selected facets, plus curved-face
    /// samples. Boundary-only cuts through a flat chord are not samples of
    /// the curved carrier. All points still participate in the residual gate.
    fit_vertices: Vec<usize>,
    /// Outward normals at a few points of the selection (area-weighted
    /// triangle normals), for the orientation decision.
    oriented: Vec<(Vec3, Vec3)>,
}

fn fit_samples(
    shell: &crate::topology::ShellRecord,
    selected: &[usize],
    edges: &HashMap<u64, &EdgeRecord>,
    vertices: &HashMap<u64, Vec3>,
) -> Result<Samples, KernelRefusal> {
    let mut welded: HashMap<(i64, i64, i64), u32> = HashMap::default();
    let mut mesh_vertices: Vec<Vec3> = Vec::new();
    let mut mesh_triangles: Vec<[u32; 3]> = Vec::new();
    let mut points: Vec<Vec3> = Vec::new();
    let mut oriented = Vec::new();
    let extent = vertices
        .values()
        .fold(0.0_f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs()))
        .max(1.0);
    let quantum = extent * 1e-9;
    let mut weld = |p: Vec3, mesh_vertices: &mut Vec<Vec3>| -> u32 {
        let key = (
            (p.x / quantum).round() as i64,
            (p.y / quantum).round() as i64,
            (p.z / quantum).round() as i64,
        );
        *welded.entry(key).or_insert_with(|| {
            mesh_vertices.push(p);
            (mesh_vertices.len() - 1) as u32
        })
    };
    // A trim can introduce vertices inside a planar facet's chord. They
    // lie below the original curved surface, unlike vertices on the ridges
    // between facets. Including them biases both radius and axis; at a
    // tangent neighbour even a minute axis tilt changes a ruling to an ellipse.
    let mut support: HashMap<u64, Vec<Vec3>> = HashMap::default();
    for &f in selected {
        let face = &shell.faces[f];
        if !face.surface.is_affine().unwrap_or(false) {
            continue;
        }
        let normal = face.surface.domain_u().and_then(|u| {
            face.surface.domain_v().and_then(|v| face.surface.normal(u[0], v[0]))
        });
        let Ok(normal) = normal else { continue };
        for c in face.loops.iter().flat_map(|l| &l.coedges) {
            let Some(edge) = edges.get(&c.edge_id) else { continue };
            for id in [edge.start_vertex_id, edge.end_vertex_id] {
                support.entry(id).or_default().push(normal);
            }
        }
    }
    let ridge_vertices: HashSet<u64> = support
        .iter()
        .filter_map(|(&id, normals)| {
            normals.iter().any(|n| normals[0].cross(*n).length() > 1e-9).then_some(id)
        })
        .collect();
    let mut fit_vertices: HashSet<usize> = HashSet::default();
    for &f in selected {
        let face = &shell.faces[f];
        let affine = face.surface.is_affine().unwrap_or(false);
        // Loop vertex points, in loop order.
        let mut polygon: Vec<Vec3> = Vec::new();
        let mut polygon_ids = Vec::new();
        for lp in &face.loops {
            for coedge in &lp.coedges {
                let Some(edge) = edges.get(&coedge.edge_id) else { continue };
                let start = if coedge.forward { edge.start_vertex_id } else { edge.end_vertex_id };
                if let Some(p) = vertices.get(&start) {
                    polygon.push(*p);
                    polygon_ids.push(start);
                    points.push(*p);
                }
                let is_line = edge.curve.degree == 1 && edge.curve.control_points.len() == 2;
                if !is_line {
                    for k in 1..8 {
                        let t = edge.t0 + (edge.t1 - edge.t0) * k as f64 / 8.0;
                        if let Ok(p) = edge.curve.evaluate(t) {
                            points.push(p);
                        }
                    }
                }
            }
        }
        if affine && face.loops.len() == 1 && polygon.len() >= 3 {
            // Keep the facet mesh intact for support areas and normals;
            // select the fitting vertices independently of its triangulation.
            let ids: Vec<u32> = polygon.iter().map(|p| weld(*p, &mut mesh_vertices)).collect();
            for (&vertex_id, &mesh_id) in polygon_ids.iter().zip(&ids) {
                if ridge_vertices.contains(&vertex_id) {
                    fit_vertices.insert(mesh_id as usize);
                }
            }
            let mut normal = Vec3::new(0.0, 0.0, 0.0);
            for i in 1..polygon.len() - 1 {
                let a = polygon[0];
                let b = polygon[i];
                let c = polygon[i + 1];
                normal = normal.add(b.sub(a).cross(c.sub(a)));
                if ids[0] != ids[i] && ids[i] != ids[i + 1] && ids[0] != ids[i + 1] {
                    mesh_triangles.push([ids[0], ids[i], ids[i + 1]]);
                }
            }
            let centroid = polygon
                .iter()
                .fold(Vec3::new(0.0, 0.0, 0.0), |s, p| s.add(*p))
                .scale(1.0 / polygon.len() as f64);
            // Loop order is counter-clockwise about the face's OUTWARD normal.
            if normal.length() > 0.0 {
                oriented.push((centroid, normal));
            }
        } else {
            let mesh = crate::tessellate_face(face, crate::TessellationOptions::default(), 0)
                .map_err(|e| refuse_input("face", format!("could not sample {}: {e}", face_label(face))))?;
            let base: Vec<u32> = mesh
                .positions
                .chunks_exact(3)
                .map(|p| weld(Vec3::new(p[0], p[1], p[2]), &mut mesh_vertices))
                .collect();
            if !affine {
                fit_vertices.extend(base.iter().map(|&i| i as usize));
                points.extend(mesh.positions.chunks_exact(3).map(|p| Vec3::new(p[0], p[1], p[2])));
            }
            for tri in mesh.indices.chunks_exact(3) {
                let [a, b, c] = [base[tri[0] as usize], base[tri[1] as usize], base[tri[2] as usize]];
                if a != b && b != c && a != c {
                    mesh_triangles.push([a, b, c]);
                    let (pa, pb, pc) =
                        (mesh_vertices[a as usize], mesh_vertices[b as usize], mesh_vertices[c as usize]);
                    let normal = pb.sub(pa).cross(pc.sub(pa));
                    if normal.length() > 0.0 {
                        oriented.push((pa.add(pb).add(pc).scale(1.0 / 3.0), normal));
                    }
                }
            }
        }
    }
    if points.len() < 3 || mesh_triangles.is_empty() {
        return Err(refuse_input(
            "face_ids",
            format!("{NO_FIT}: the selection has too few points to fit ({})", points.len()),
        ));
    }
    let mut fit_vertices: Vec<usize> = fit_vertices.into_iter().collect();
    fit_vertices.sort_unstable();
    Ok(Samples { points, mesh_vertices, mesh_triangles, fit_vertices, oriented })
}

struct Fitted {
    carrier: RefitCarrier,
}

fn to_ransac(p: Vec3) -> brep_ransac::Vec3 {
    brep_ransac::Vec3::new(p.x, p.y, p.z)
}

fn from_ransac(p: brep_ransac::Vec3) -> Vec3 {
    Vec3::new(p.x, p.y, p.z)
}

fn fit_carrier(
    samples: &Samples,
    kind: RefitSurfaceKind,
    tolerance: f64,
    scale: f64,
) -> Result<Fitted, KernelRefusal> {
    use brep_ransac::{AnalyticSurface, RecognitionOptions, SamplingMode, SurfaceHint, SurfaceType};
    let mesh = brep_ransac::Mesh::new(
        samples.mesh_vertices.iter().copied().map(to_ransac).collect(),
        samples.mesh_triangles.clone(),
    );
    let triangles: Vec<usize> = (0..samples.mesh_triangles.len()).collect();
    let hint = match kind {
        RefitSurfaceKind::Auto => SurfaceHint::Unknown,
        RefitSurfaceKind::Plane => SurfaceHint::KnownType { surface_type: SurfaceType::Plane },
        RefitSurfaceKind::Cylinder => {
            SurfaceHint::KnownType { surface_type: SurfaceType::Cylinder }
        }
        RefitSurfaceKind::Cone => SurfaceHint::KnownType { surface_type: SurfaceType::Cone },
        RefitSurfaceKind::Sphere => SurfaceHint::KnownType { surface_type: SurfaceType::Sphere },
        RefitSurfaceKind::Torus => SurfaceHint::KnownType { surface_type: SurfaceType::Torus },
    };
    let options = |distance: f64, normal: f64| RecognitionOptions {
        distance_tolerance: distance,
        relative_tolerance: 0.0,
        // The facets' normals are the chords' and not the surface's: the
        // user asserts the family, the positional residual is the judge.
        normal_tolerance: normal,
        minimum_support: 1,
        minimum_support_area: 0.0,
        discover_regions: false,
        sampling: SamplingMode::Vertices,
        ..RecognitionOptions::default()
    };
    // First at the caller's tolerance; if nothing fits there, again with a
    // loose one so the refusal can quote the residual it missed by.
    // The retry gates on nothing (distance the model's size, normals any
    // angle): its only job is the carrier whose residual the refusal quotes.
    let reconstruct = |options: &RecognitionOptions| {
        // A small selection (or one plane) may have too few ridges to fit.
        // In that case retain the original all-vertex fitting path.
        if !samples.fit_vertices.is_empty() && samples.fit_vertices.len() < mesh.vertices.len() {
            if let Ok(fit) = brep_ransac::reconstruct_surface_from_vertices(
                &mesh,
                &samples.fit_vertices,
                &hint,
                options,
            ) {
                // A few ridges can alias a simpler carrier (two cylinder
                // rulings are coplanar). Fall back if it misses the full region.
                if samples.points.iter().all(|p| {
                    fit.surface.signed_distance(to_ransac(*p)).abs() <= options.distance_tolerance
                }) {
                    return Ok(fit);
                }
            }
        }
        brep_ransac::reconstruct_surface(&mesh, &triangles, &hint, options)
    };
    let fit = reconstruct(&options(tolerance, 60.0_f64.to_radians()))
        .or_else(|_| reconstruct(&options(scale * 10.0, PI)))
        .map_err(|e| {
            refuse_unsupported(
                KernelStage::Refine,
                "refit_no_fit",
                format!("{NO_FIT}: the recognition pass found no {} ({e})", kind.name()),
            )
        })?;
    let carrier = match fit.surface {
        AnalyticSurface::Plane(p) => RefitCarrier::Plane {
            origin: from_ransac(p.origin),
            normal: unit(from_ransac(p.normal))?,
        },
        AnalyticSurface::Cylinder(c) => RefitCarrier::Cylinder {
            axis_origin: from_ransac(c.axis_origin),
            axis: unit(from_ransac(c.axis))?,
            radius: c.radius,
        },
        AnalyticSurface::Cone(c) => RefitCarrier::Cone {
            apex: from_ransac(c.apex),
            axis: unit(from_ransac(c.axis))?,
            half_angle: c.half_angle,
        },
        AnalyticSurface::Sphere(s) => {
            RefitCarrier::Sphere { center: from_ransac(s.center), radius: s.radius }
        }
        AnalyticSurface::Torus(t) => RefitCarrier::Torus {
            center: from_ransac(t.center),
            axis: unit(from_ransac(t.axis))?,
            major_radius: t.major_radius,
            minor_radius: t.minor_radius,
        },
    };
    Ok(Fitted { carrier })
}

fn unit(v: Vec3) -> Result<Vec3, KernelRefusal> {
    v.normalized()
        .map_err(|e| refuse_unsupported(KernelStage::Refine, "refit_no_fit", format!("{NO_FIT}: {e}")))
}

fn residuals(carrier: &RefitCarrier, points: &[Vec3]) -> (f64, f64) {
    let mut sum = 0.0;
    let mut max = 0.0_f64;
    for p in points {
        let d = carrier.distance(*p);
        sum += d * d;
        max = max.max(d);
    }
    ((sum / points.len().max(1) as f64).sqrt(), max)
}

// ---------------------------------------------------------------------------
// The carrier surface, framed so the region avoids its seam and poles
// ---------------------------------------------------------------------------

/// Unit x/y frame perpendicular to `axis`, x pointing at the middle of the
/// largest angular gap the points leave around the axis (so the surface's
/// parameter seam falls where the region is not; a full band has no gap and
/// the seam is arbitrary — its boundary is two closed rings).
fn seam_frame(axis: Vec3, origin: Vec3, points: &[Vec3]) -> Result<(Vec3, Vec3), KernelRefusal> {
    let x0 = axis.perpendicular().map_err(|e| refuse_input("axis", e))?;
    let y0 = axis.cross(x0);
    let mut angles: Vec<f64> = points
        .iter()
        .filter_map(|p| {
            let rel = p.sub(origin);
            let (x, y) = (rel.dot(x0), rel.dot(y0));
            (x.hypot(y) > 1e-12).then(|| y.atan2(x))
        })
        .collect();
    if angles.is_empty() {
        return Ok((x0, y0));
    }
    angles.sort_by(f64::total_cmp);
    let mut best_gap = angles[0] + TAU - angles[angles.len() - 1];
    let mut best_mid = angles[angles.len() - 1] + best_gap * 0.5;
    for pair in angles.windows(2) {
        let gap = pair[1] - pair[0];
        if gap > best_gap {
            best_gap = gap;
            best_mid = pair[0] + gap * 0.5;
        }
    }
    let x = x0.scale(best_mid.cos()).add(y0.scale(best_mid.sin()));
    Ok((x, axis.cross(x)))
}

fn carrier_surface(
    carrier: &RefitCarrier,
    points: &[Vec3],
    scale: f64,
) -> Result<NurbsSurface, KernelRefusal> {
    let fail = |e: String| {
        refuse_unsupported(KernelStage::Refine, "refit_surface", format!("{NO_FIT}: {e}"))
    };
    match *carrier {
        RefitCarrier::Plane { origin, normal } => {
            let u = normal.perpendicular().map_err(fail)?;
            let v = normal.cross(u);
            let (mut u0, mut u1, mut v0, mut v1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for p in points {
                let rel = p.sub(origin);
                u0 = u0.min(rel.dot(u));
                u1 = u1.max(rel.dot(u));
                v0 = v0.min(rel.dot(v));
                v1 = v1.max(rel.dot(v));
            }
            let margin = ((u1 - u0).max(v1 - v0) * 0.1).max(scale * 1e-3);
            let corner = origin.add(u.scale(u0 - margin)).add(v.scale(v0 - margin));
            make_plane(corner, u, v, u1 - u0 + 2.0 * margin, v1 - v0 + 2.0 * margin).map_err(fail)
        }
        RefitCarrier::Cylinder { axis_origin, axis, radius } => {
            let (h0, h1) = axial_range(axis_origin, axis, points);
            let margin = ((h1 - h0) * 0.1).max(scale * 1e-3);
            let (x, _) = seam_frame(axis, axis_origin, points)?;
            let base = axis_origin.add(axis.scale(h0 - margin));
            let start = base.add(x.scale(radius));
            let generatrix =
                make_line(start, start.add(axis.scale(h1 - h0 + 2.0 * margin))).map_err(fail)?;
            make_revolution(base, axis, &generatrix, TAU).map_err(fail)
        }
        RefitCarrier::Cone { apex, axis, half_angle } => {
            let (h0, h1) = axial_range(apex, axis, points);
            if h0 <= 0.0 {
                return Err(refuse_unsupported(
                    KernelStage::Refine,
                    "cone_apex",
                    format!(
                        "the fitted cone's apex lies inside the region; a pointed cone face is \
                         {NOT_DISK_OR_ANNULUS} this edit builds (v1)"
                    ),
                ));
            }
            let margin = ((h1 - h0) * 0.1).max(scale * 1e-3).min(h0 * 0.5);
            let (x, _) = seam_frame(axis, apex, points)?;
            let tan = half_angle.tan();
            let (a, b) = (h0 - margin, h1 + margin);
            let generatrix = make_line(
                apex.add(axis.scale(a)).add(x.scale(a * tan)),
                apex.add(axis.scale(b)).add(x.scale(b * tan)),
            )
            .map_err(fail)?;
            make_revolution(apex, axis, &generatrix, TAU).map_err(fail)
        }
        RefitCarrier::Sphere { center, radius } => {
            // Park the poles 90 degrees from the region's mean direction and
            // the seam opposite it.
            let mean = points
                .iter()
                .fold(Vec3::new(0.0, 0.0, 0.0), |s, p| s.add(p.sub(center).normalized().unwrap_or(*p)));
            let mean = mean.normalized().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
            let pole = mean.perpendicular().map_err(fail)?;
            let limit = (85.0_f64).to_radians().sin();
            // Choose, among directions perpendicular to the mean, the pole the
            // region stays farthest from.
            let other = mean.cross(pole);
            let mut best = (f64::MAX, pole);
            for k in 0..36 {
                let angle = PI * k as f64 / 36.0;
                let candidate = pole.scale(angle.cos()).add(other.scale(angle.sin()));
                let worst = points
                    .iter()
                    .map(|p| (p.sub(center).dot(candidate) / radius).abs())
                    .fold(0.0_f64, f64::max);
                if worst < best.0 {
                    best = (worst, candidate);
                }
            }
            if best.0 > limit {
                return Err(refuse_unsupported(
                    KernelStage::Refine,
                    "sphere_extent",
                    format!(
                        "the spherical region reaches within 5 degrees of every pole this edit \
                         can place; a region that large is {NOT_DISK_OR_ANNULUS} this edit \
                         builds (v1)"
                    ),
                ));
            }
            let pole = best.1;
            let seam = mean.scale(-1.0);
            let x_axis = seam.sub(pole.scale(seam.dot(pole))).normalized().map_err(fail)?;
            let meridian =
                make_arc(center, x_axis, pole, radius, -PI / 2.0, PI / 2.0).map_err(fail)?;
            make_revolution(center, pole, &meridian, TAU).map_err(fail)
        }
        RefitCarrier::Torus { center, axis, major_radius, minor_radius } => {
            if !(minor_radius > 0.0 && major_radius > minor_radius) {
                return Err(refuse_unsupported(
                    KernelStage::Refine,
                    "torus_radii",
                    format!(
                        "the fitted torus (radii {major_radius:.6}/{minor_radius:.6}) \
                         self-intersects; {NO_FIT} as one face (v1)"
                    ),
                ));
            }
            let (x, _) = seam_frame(axis, center, points)?;
            // Tube angle gap, in the (radial, axial) plane.
            let mut angles: Vec<f64> = points
                .iter()
                .map(|p| {
                    let rel = p.sub(center);
                    let along = rel.dot(axis);
                    let radial = rel.sub(axis.scale(along)).length() - major_radius;
                    along.atan2(radial)
                })
                .collect();
            angles.sort_by(f64::total_cmp);
            let mut start = 0.0;
            if !angles.is_empty() {
                let mut best_gap = angles[0] + TAU - angles[angles.len() - 1];
                let mut best_mid = angles[angles.len() - 1] + best_gap * 0.5;
                for pair in angles.windows(2) {
                    if pair[1] - pair[0] > best_gap {
                        best_gap = pair[1] - pair[0];
                        best_mid = pair[0] + best_gap * 0.5;
                    }
                }
                start = best_mid;
            }
            let tube_center = center.add(x.scale(major_radius));
            let tube = make_arc(tube_center, x, axis, minor_radius, start, start + TAU)
                .map_err(fail)?;
            make_revolution(center, axis, &tube, TAU).map_err(fail)
        }
    }
}

fn axial_range(origin: Vec3, axis: Vec3, points: &[Vec3]) -> (f64, f64) {
    points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| {
        let h = p.sub(origin).dot(axis);
        (lo.min(h), hi.max(h))
    })
}

fn region_same_sense(surface: &NurbsSurface, samples: &Samples) -> Result<bool, KernelRefusal> {
    let mut vote = 0.0;
    for (point, outward) in samples.oriented.iter().step_by((samples.oriented.len() / 64).max(1)) {
        let Ok(projection) = crate::project_point_to_surface(surface, *point) else { continue };
        let Ok(normal) = surface.normal(projection.u, projection.v) else { continue };
        vote += normal.dot(*outward).signum() * outward.length();
    }
    if vote == 0.0 {
        return Err(refuse_unsupported(
            KernelStage::Classify,
            "orientation",
            format!("{NO_FIT}: the selection's outward side could not be read"),
        ));
    }
    Ok(vote > 0.0)
}

// ---------------------------------------------------------------------------
// Boundary chains and their exact curves
// ---------------------------------------------------------------------------

fn split_chains(cycle: &[BoundaryUse]) -> Vec<Chain> {
    let n = cycle.len();
    let Some(change) = (0..n).find(|&i| cycle[i].neighbour != cycle[(i + n - 1) % n].neighbour)
    else {
        return vec![Chain { uses: cycle.to_vec(), closed: true, neighbour: cycle[0].neighbour }];
    };
    let mut chains: Vec<Chain> = Vec::new();
    for k in 0..n {
        let use_ = cycle[(change + k) % n];
        match chains.last_mut() {
            Some(chain) if chain.neighbour == use_.neighbour => chain.uses.push(use_),
            _ => chains.push(Chain { uses: vec![use_], closed: false, neighbour: use_.neighbour }),
        }
    }
    chains
}

fn chain_points(chain: &Chain, vertices: &HashMap<u64, Vec3>) -> Vec<Vec3> {
    let mut points: Vec<Vec3> = chain.uses.iter().map(|u| vertices[&u.start]).collect();
    if !chain.closed {
        points.push(vertices[&chain.uses.last().unwrap().end]);
    }
    points
}

/// The point where two curves meet near `seed`, by alternating projection,
/// and the gap left between them there.
fn curve_meeting(
    first: &NurbsCurve,
    second: &NurbsCurve,
    seed: Vec3,
) -> Result<(Vec3, f64), KernelRefusal> {
    let project = |curve: &NurbsCurve, p: Vec3| {
        project_point_to_curve(curve, p).map_err(|e| {
            refuse_unsupported(KernelStage::Intersect, "corner", format!("{NO_EXACT_BOUNDARY}: {e}"))
        })
    };
    let mut p = seed;
    let mut gap = f64::MAX;
    for _ in 0..200 {
        let a = project(first, p)?.point;
        let b = project(second, a)?.point;
        gap = a.sub(b).length();
        let next = a.add(b).scale(0.5);
        let step = next.sub(p).length();
        p = next;
        if step < 1e-14 * (1.0 + p.length()) {
            break;
        }
    }
    let a = project(first, p)?.point;
    let b = project(second, p)?.point;
    gap = gap.min(a.sub(b).length());
    Ok((a.add(b).scale(0.5), gap))
}

/// A circle through `curve`'s points, if the curve is one: (center, normal, radius).
fn as_circle(curve: &NurbsCurve, scale: f64) -> Option<(Vec3, Vec3, f64)> {
    let [t0, t1] = curve.domain().ok()?;
    let samples: Vec<Vec3> = (0..24)
        .filter_map(|k| curve.evaluate(t0 + (t1 - t0) * k as f64 / 24.0).ok())
        .collect();
    if samples.len() < 24 {
        return None;
    }
    let (a, b, c) = (samples[0], samples[8], samples[16]);
    let ab = b.sub(a);
    let ac = c.sub(a);
    let normal = ab.cross(ac);
    let denom = 2.0 * normal.length_squared();
    if denom <= 1e-30 {
        return None;
    }
    let center = a.add(
        normal
            .cross(ab)
            .scale(ac.length_squared())
            .add(ac.cross(normal).scale(ab.length_squared()))
            .scale(1.0 / denom),
    );
    let radius = a.sub(center).length();
    let unit_normal = normal.normalized().ok()?;
    let tol = scale * 1e-9 + radius * 1e-9;
    for p in &samples {
        if (p.sub(center).length() - radius).abs() > tol || p.sub(center).dot(unit_normal).abs() > tol {
            return None;
        }
    }
    Some((center, unit_normal, radius))
}

fn angle_in(center: Vec3, x: Vec3, y: Vec3, p: Vec3) -> f64 {
    let rel = p.sub(center);
    let angle = rel.dot(y).atan2(rel.dot(x));
    if angle < 0.0 {
        angle + TAU
    } else {
        angle
    }
}

/// A closed boundary ring: the exact closed curve, oriented and started so
/// the region traverses it forward from its first chain point when possible.
fn closed_ring_curve(
    curve: &NurbsCurve,
    points: &[Vec3],
    scale: f64,
) -> Result<(NurbsCurve, bool), KernelRefusal> {
    let fail = |e: String| {
        refuse_unsupported(KernelStage::Intersect, "ring", format!("{NO_EXACT_BOUNDARY}: {e}"))
    };
    if let Some((center, normal, radius)) = as_circle(curve, scale) {
        let start = points[0];
        let x = start.sub(center);
        let x = x.sub(normal.scale(x.dot(normal))).normalized().map_err(fail)?;
        let mut n = normal;
        let y = n.cross(x);
        // The region runs from points[0] toward points[1]: keep that sense.
        if angle_in(center, x, y, points[1]) > PI {
            n = n.scale(-1.0);
        }
        let y = n.cross(x);
        let arc = make_arc(center, x, y, radius, 0.0, TAU).map_err(fail)?;
        return Ok((arc, true));
    }
    // Not a circle: keep the curve, read the direction from the chain.
    let p0 = project_point_to_curve(curve, points[0]).map_err(fail)?;
    let p1 = project_point_to_curve(curve, points[1]).map_err(fail)?;
    let [t0, t1] = curve.domain().map_err(fail)?;
    let mut delta = p1.u - p0.u;
    let period = t1 - t0;
    if delta > period * 0.5 {
        delta -= period;
    } else if delta < -period * 0.5 {
        delta += period;
    }
    Ok((curve.clone(), delta > 0.0))
}

/// An open boundary run from `start_point` to `end_point` along `curve`,
/// through `mid_point`.
fn open_arc(
    curve: &NurbsCurve,
    start: u64,
    end: u64,
    start_point: Vec3,
    end_point: Vec3,
    mid_point: Vec3,
    scale: f64,
) -> Result<BuiltEdge, KernelRefusal> {
    let fail = |e: String| {
        refuse_unsupported(KernelStage::Intersect, "arc", format!("{NO_EXACT_BOUNDARY}: {e}"))
    };
    if let Some((center, normal, radius)) = as_circle(curve, scale) {
        let x = start_point.sub(center);
        let x = x.sub(normal.scale(x.dot(normal))).normalized().map_err(fail)?;
        let mut n = normal;
        let mut y = n.cross(x);
        let mut sweep = angle_in(center, x, y, end_point);
        if angle_in(center, x, y, mid_point) > sweep {
            n = n.scale(-1.0);
            y = n.cross(x);
            sweep = angle_in(center, x, y, end_point);
        }
        let arc = make_arc(center, x, y, radius, 0.0, sweep).map_err(fail)?;
        let [t0, t1] = arc.domain().map_err(fail)?;
        return Ok(BuiltEdge { curve: arc, t0, t1, start, end, region_forward: true, region_pcurve: None });
    }
    let a = project_point_to_curve(curve, start_point).map_err(fail)?.u;
    let b = project_point_to_curve(curve, end_point).map_err(fail)?.u;
    let m = project_point_to_curve(curve, mid_point).map_err(fail)?.u;
    if !((a < m && m < b) || (b < m && m < a)) {
        return Err(refuse_unsupported(
            KernelStage::Intersect,
            "arc_seam",
            format!(
                "{NO_EXACT_BOUNDARY}: a boundary run crosses its intersection curve's seam (v1)"
            ),
        ));
    }
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    let trimmed = crate::feature_pipeline::features::common::trimmed_curve(curve, lo, hi)
        .map_err(fail)?;
    let [t0, t1] = trimmed.domain().map_err(fail)?;
    if a < b {
        Ok(BuiltEdge { curve: trimmed, t0, t1, start, end, region_forward: true, region_pcurve: None })
    } else {
        Ok(BuiltEdge {
            curve: trimmed,
            t0,
            t1,
            start: end,
            end: start,
            region_forward: false,
            region_pcurve: None,
        })
    }
}

/// A closed boundary ring that is an iso-parameter circle of the fitted
/// (revolved) surface, closed in that direction: the ring is taken as the
/// surface's own iso curve, starting on its seam, so its trim is a straight
/// line across the whole period — the way the kernel's revolved solids carry
/// their cap circles. `None` unless the iso curve IS the intersection with
/// the neighbour to `tolerance` (a tilted cap cuts an ellipse, not a circle).
fn iso_ring(
    surface: &NurbsSurface,
    intersection: &NurbsCurve,
    points: &[Vec3],
    tolerance: f64,
) -> Option<(NurbsCurve, NurbsCurve, bool)> {
    let uv: Vec<(f64, f64)> = points
        .iter()
        .map(|p| crate::project_point_to_surface(surface, *p).map(|q| (q.u, q.v)))
        .collect::<Result<_, _>>()
        .ok()?;
    if uv.len() < 3 {
        return None;
    }
    let [u0, u1] = [surface.knots_u[0], *surface.knots_u.last()?];
    let [v0, v1] = [surface.knots_v[0], *surface.knots_v.last()?];
    let mean_u = uv.iter().map(|q| q.0).sum::<f64>() / uv.len() as f64;
    let mean_v = uv.iter().map(|q| q.1).sum::<f64>() / uv.len() as f64;
    let wrap = |delta: f64, period: f64| {
        let mut d = delta % period;
        if d > period * 0.5 {
            d -= period;
        } else if d < -period * 0.5 {
            d += period;
        }
        d
    };
    let candidates = [
        // Constant v: the ring runs along u.
        (surface.iso_curve_v(mean_v).ok(), true),
        // Constant u: the ring runs along v.
        (surface.iso_curve_u(mean_u).ok(), false),
    ];
    for (curve, along_u) in candidates {
        let Some(curve) = curve else { continue };
        let [t0, t1] = curve.domain().ok()?;
        let (Ok(start), Ok(end)) = (curve.evaluate(t0), curve.evaluate(t1)) else { continue };
        if start.sub(end).length() > tolerance * 1e-3 {
            continue; // not closed in this direction
        }
        let on_intersection = (0..16).all(|k| {
            curve
                .evaluate(t0 + (t1 - t0) * (k as f64 + 0.5) / 16.0)
                .ok()
                .and_then(|p| project_point_to_curve(intersection, p).ok())
                .is_some_and(|q| q.distance <= tolerance)
        });
        let through_points =
            points.iter().all(|p| project_point_to_curve(&curve, *p).is_ok_and(|q| q.distance <= tolerance));
        if !(on_intersection && through_points) {
            continue;
        }
        let (period, a, b) = if along_u {
            (u1 - u0, uv[0].0, uv[1].0)
        } else {
            (v1 - v0, uv[0].1, uv[1].1)
        };
        let forward = wrap(b - a, period) > 0.0;
        let (from, to) = if along_u {
            ([u0, mean_v], [u1, mean_v])
        } else {
            ([mean_u, v0], [mean_u, v1])
        };
        let (from, to) = if forward { (from, to) } else { (to, from) };
        let pcurve =
            make_line(Vec3::new(from[0], from[1], 0.0), Vec3::new(to[0], to[1], 0.0)).ok()?;
        return Some((curve, pcurve, forward));
    }
    None
}

/// The iso-parameter curve of `surface` through `points` (constant v, then
/// constant u), when it lies on `other` to `tolerance` everywhere sampled —
/// the exact boundary for pairs the closed-form intersector does not list
/// (a plane square to a torus's axis cuts it in iso circles).
fn iso_curve_on(
    surface: &NurbsSurface,
    other: &NurbsSurface,
    points: &[Vec3],
    tolerance: f64,
) -> Vec<NurbsCurve> {
    let Ok(uv) = points
        .iter()
        .map(|p| crate::project_point_to_surface(surface, *p).map(|q| (q.u, q.v)))
        .collect::<Result<Vec<_>, _>>()
    else {
        return Vec::new();
    };
    if uv.is_empty() {
        return Vec::new();
    }
    let mean_u = uv.iter().map(|q| q.0).sum::<f64>() / uv.len() as f64;
    let mean_v = uv.iter().map(|q| q.1).sum::<f64>() / uv.len() as f64;
    let mut out = Vec::new();
    for curve in [surface.iso_curve_v(mean_v), surface.iso_curve_u(mean_u)].into_iter().flatten() {
        let Ok([t0, t1]) = curve.domain() else { continue };
        let through = points
            .iter()
            .all(|p| project_point_to_curve(&curve, *p).is_ok_and(|q| q.distance <= tolerance));
        let on_other = (0..=32).all(|k| {
            curve
                .evaluate(t0 + (t1 - t0) * k as f64 / 32.0)
                .ok()
                .is_some_and(|p| {
                    // Test the carrier, not the finite patch: the fitted
                    // surface's margin extends an iso ruling beyond the
                    // neighbour. open_arc trims it to the shared run later.
                    if let Some(crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) =
                        other.analytic()
                    {
                        u_dir.cross(*v_dir).normalized()
                            .is_ok_and(|n| p.sub(*origin).dot(n).abs() <= tolerance)
                    } else {
                        crate::project_point_to_surface(other, p).is_ok_and(|q| q.distance <= tolerance)
                    }
                })
        });
        if through && on_other {
            out.push(curve);
        }
    }
    out
}

/// The plane of the planar closed `ring`, framed like `old` (same normal
/// side, same in-plane directions as nearly as the tilt allows, a patch
/// covering the ring). `None` when the ring is not planar or already lies
/// in `old`.
fn reseated_plane(old: &NurbsSurface, ring: &NurbsCurve, scale: f64) -> Option<NurbsSurface> {
    let [t0, t1] = ring.domain().ok()?;
    let points: Vec<Vec3> =
        (0..48).filter_map(|k| ring.evaluate(t0 + (t1 - t0) * k as f64 / 48.0).ok()).collect();
    if points.len() < 48 {
        return None;
    }
    let centroid = points
        .iter()
        .fold(Vec3::new(0.0, 0.0, 0.0), |s, p| s.add(*p))
        .scale(1.0 / points.len() as f64);
    // Newell normal of the ring polygon.
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    for (i, a) in points.iter().enumerate() {
        let b = points[(i + 1) % points.len()];
        normal = normal.add(a.sub(centroid).cross(b.sub(centroid)));
    }
    let normal = normal.normalized().ok()?;
    let flat = points.iter().all(|p| p.sub(centroid).dot(normal).abs() <= scale * 1e-10);
    if !flat {
        return None;
    }
    let [u0, u1] = [old.knots_u[0], *old.knots_u.last()?];
    let [v0, v1] = [old.knots_v[0], *old.knots_v.last()?];
    let origin = old.evaluate(u0, v0).ok()?;
    let su = old.evaluate(u1, v0).ok()?.sub(origin);
    let sv = old.evaluate(u0, v1).ok()?.sub(origin);
    let old_normal = su.cross(sv).normalized().ok()?;
    let off = points.iter().map(|p| p.sub(origin).dot(old_normal).abs()).fold(0.0, f64::max);
    if off <= scale * 1e-12 {
        return None;
    }
    let n = if normal.dot(old_normal) >= 0.0 { normal } else { normal.scale(-1.0) };
    let u_dir = su.sub(n.scale(su.dot(n))).normalized().ok()?;
    let v_dir = n.cross(u_dir);
    // Cover the ring and the old patch, with a margin.
    let mut lo = [f64::MAX; 2];
    let mut hi = [f64::MIN; 2];
    let corners = [origin, origin.add(su), origin.add(sv), origin.add(su).add(sv)];
    for p in points.iter().chain(corners.iter()) {
        let rel = p.sub(centroid);
        let (a, b) = (rel.dot(u_dir), rel.dot(v_dir));
        lo = [lo[0].min(a), lo[1].min(b)];
        hi = [hi[0].max(a), hi[1].max(b)];
    }
    let margin = (hi[0] - lo[0]).max(hi[1] - lo[1]) * 0.05;
    let corner = centroid.add(u_dir.scale(lo[0] - margin)).add(v_dir.scale(lo[1] - margin));
    make_plane(corner, u_dir, v_dir, hi[0] - lo[0] + 2.0 * margin, hi[1] - lo[1] + 2.0 * margin).ok()
}

// ---------------------------------------------------------------------------
// v2: a whole closed shell becomes one sphere or torus
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn refit_closed_shell(
    solid: &BrepSolid,
    shell_index: usize,
    selected_indices: &[usize],
    euler: i64,
    kind: RefitSurfaceKind,
    tolerance: f64,
    scale: f64,
    edge_by_id: &HashMap<u64, &EdgeRecord>,
    vertex_by_id: &HashMap<u64, Vec3>,
) -> Result<(BrepSolid, RefitReport), KernelRefusal> {
    let shell = &solid.shells[shell_index];
    if selected_indices.len() != shell.faces.len() {
        return Err(refuse_unsupported(
            KernelStage::Classify,
            "closed_region",
            format!(
                "the selection has no boundary but is not every face of its shell; the region \
                 is {NOT_DISK_OR_ANNULUS}"
            ),
        ));
    }
    // A sphere closes with Euler characteristic 2, a torus with 0. AUTO
    // takes the one the shell's own topology names.
    let kind = match kind {
        RefitSurfaceKind::Auto => match euler {
            2 => RefitSurfaceKind::Sphere,
            0 => RefitSurfaceKind::Torus,
            _ => {
                return Err(refuse_unsupported(
                    KernelStage::Classify,
                    "closed_shell_genus",
                    format!(
                        "the selected closed shell has Euler characteristic {euler}; \
                         {CLOSED_SHELL_KIND} (2 or 0)"
                    ),
                ))
            }
        },
        RefitSurfaceKind::Plane | RefitSurfaceKind::Cylinder | RefitSurfaceKind::Cone => {
            return Err(refuse_unsupported(
                KernelStage::Classify,
                "closed_shell_kind",
                format!(
                    "the selection is a whole closed shell and no {} can be one: \
                     {CLOSED_SHELL_KIND}",
                    kind.name()
                ),
            ))
        }
        other => other,
    };
    let wanted = if kind == RefitSurfaceKind::Sphere { 2 } else { 0 };
    if euler != wanted {
        return Err(refuse_unsupported(
            KernelStage::Classify,
            "closed_shell_genus",
            format!(
                "a {} {CLOSED_SHELL_GENUS}: the selected closed shell has Euler characteristic \
                 {euler}, a {} has {wanted}",
                kind.name(),
                kind.name()
            ),
        ));
    }

    let samples = fit_samples(shell, selected_indices, edge_by_id, vertex_by_id)?;
    let carrier = fit_carrier(&samples, kind, tolerance, scale)?.carrier;
    let (rms_residual, max_residual) = residuals(&carrier, &samples.points);
    if max_residual > tolerance {
        return Err(KernelRefusal::ill_posed(
            KernelStage::Refine,
            "refit_residual",
            format!(
                "Refit Faces: the {} {RESIDUAL_OVER_TOLERANCE} {max_residual:.6} (rms \
                 {rms_residual:.6}) is over the tolerance {tolerance}; these faces are not \
                 one {} within it",
                carrier.kind().name(),
                carrier.kind().name()
            ),
        ));
    }
    coarse_facet_gate(&carrier, shell, selected_indices, edge_by_id, vertex_by_id)?;
    let fail = |e: String| {
        refuse_unsupported(KernelStage::Refine, "refit_surface", format!("{NO_FIT}: {e}"))
    };
    let primitive = match carrier {
        RefitCarrier::Sphere { center, radius } => {
            // Poles on the axis the facets' own layout suggests is irrelevant
            // for a whole ball; z is as good as any.
            crate::make_sphere_brep(center, radius, Vec3::new(0.0, 0.0, 1.0)).map_err(fail)?
        }
        RefitCarrier::Torus { center, axis, major_radius, minor_radius } => {
            if !(minor_radius > 0.0 && major_radius > minor_radius) {
                return Err(refuse_unsupported(
                    KernelStage::Refine,
                    "torus_radii",
                    format!(
                        "the fitted torus (radii {major_radius:.6}/{minor_radius:.6}) \
                         self-intersects; {NO_FIT} as one face (v1)"
                    ),
                ));
            }
            crate::make_torus_brep(center, axis, major_radius, minor_radius).map_err(fail)?
        }
        other => {
            return Err(refuse_unsupported(
                KernelStage::Classify,
                "closed_shell_kind",
                format!(
                    "the fit returned a {}; {CLOSED_SHELL_KIND}",
                    other.kind().name()
                ),
            ))
        }
    };

    // The primitive is an outward solid. The selection's own outward side
    // decides whether its shell bounds material (outer shell) or a cavity.
    let prim_face = &primitive.shells[0].faces[0];
    let vote = region_same_sense(&prim_face.surface, &samples)?;
    let cavity = vote != prim_face.same_sense;

    // Re-id the primitive's records above everything the solid holds.
    let base = solid
        .vertices
        .iter()
        .map(|v| v.id)
        .chain(solid.edges.iter().map(|e| e.id))
        .chain(solid.shells.iter().map(|s| s.id))
        .chain(solid.shells.iter().flat_map(|s| s.faces.iter()).flat_map(|f| {
            std::iter::once(f.id).chain(
                f.loops
                    .iter()
                    .flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id))),
            )
        }))
        .max()
        .unwrap_or(0)
        + 1;
    let keeper = &shell.faces[selected_indices[0]];
    let mut new_shell = primitive.shells[0].clone();
    new_shell.id = shell.id;
    for face in &mut new_shell.faces {
        face.id = keeper.id;
        face.name = keeper.name.clone();
        for lp in &mut face.loops {
            lp.id += base;
            for coedge in &mut lp.coedges {
                coedge.id += base;
                coedge.edge_id += base;
            }
        }
        if cavity {
            flip_face(face).map_err(fail)?;
        }
    }
    let new_edges: Vec<EdgeRecord> = primitive
        .edges
        .iter()
        .map(|e| {
            let mut e = e.clone();
            e.id += base;
            e.start_vertex_id += base;
            e.end_vertex_id += base;
            e
        })
        .collect();
    let new_vertices: Vec<VertexRecord> = primitive
        .vertices
        .iter()
        .map(|v| VertexRecord { id: v.id + base, point: v.point })
        .collect();

    // Drop the old shell's edges and vertices (no other shell shares them).
    let old_edges: HashSet<u64> = shell
        .faces
        .iter()
        .flat_map(|f| f.loops.iter())
        .flat_map(|l| l.coedges.iter().map(|c| c.edge_id))
        .collect();
    let mut out = solid.clone();
    out.edges.retain(|e| !old_edges.contains(&e.id));
    out.edges.extend(new_edges);
    out.shells[shell_index] = new_shell;
    let used: HashSet<u64> =
        out.edges.iter().flat_map(|e| [e.start_vertex_id, e.end_vertex_id]).collect();
    out.vertices.retain(|v| used.contains(&v.id));
    out.vertices.extend(new_vertices);
    // The shell's genus is unchanged (checked above), so `out.genus` stands.

    let issues = out.validate();
    if !issues.is_empty() {
        let first: Vec<String> = issues.iter().take(3).map(|i| format!("{i:?}")).collect();
        return Err(KernelRefusal::new(
            RefusalClass::InvalidResultTopology { issues: issues.len() as u32 },
            KernelStage::Validate,
            format!(
                "Refit Faces: the {RESULT_INVALID} ({} issue(s): {})",
                issues.len(),
                first.join("; ")
            ),
        ));
    }
    let report = RefitReport {
        carrier,
        rms_residual,
        max_residual,
        tolerance,
        samples: samples.points.len(),
        replaced_faces: selected_indices.len(),
        boundary_edges_before: 0,
        boundary_edges_after: 0,
        boundary_loops: 0,
        new_face_id: keeper.id,
    };
    Ok((out, report))
}

/// Turn a face inside out: the other normal side, each loop walked the other
/// way (the `offset_shell::orientation` operation).
pub(super) fn flip_face(face: &mut FaceRecord) -> Result<(), String> {
    face.same_sense = !face.same_sense;
    for lp in &mut face.loops {
        lp.coedges.reverse();
        for coedge in &mut lp.coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed()?;
        }
    }
    Ok(())
}

/// The widest arc, in degrees, any selected flat face may span on the fitted
/// surface: a facet may be no wider than the radius of the surface it
/// approximates. A chord spanning `t` on radius `R` is `2 R sin(t / 2)` long,
/// which is at most `R` exactly when `t <= 60` degrees. The bound is inclusive:
/// a regular hexagonal prism (chord = radius) refits as a cylinder, and a
/// pentagonal one (72 degrees, chord 1.18 R) is refused.
///
/// The fit reads vertices only, and every regular polygon is cyclic, so the
/// corners alone never refute a coarse fit. The 16 corners of an 8-sided can
/// lie exactly on one sphere, whose flat caps would each span about 98
/// degrees of it; a box's four walls lie on a cylinder at 90 degrees each.
pub const MAX_FACET_SPAN_DEGREES: f64 = 60.0;

/// Rounding slack on [`MAX_FACET_SPAN_DEGREES`], so a facet exactly on the
/// bound (a hexagon read through f32 coordinates and a fitted radius) is
/// not refused by the last digit.
pub const FACET_SPAN_SLACK_DEGREES: f64 = 0.1;

/// Refuse when a selected flat face spans more than
/// [`MAX_FACET_SPAN_DEGREES`] of the fitted curved surface, read from how far
/// its vertex centroid sags below the surface: a chord of half-angle `a` on
/// a radius `R` sags `R (1 - cos a)`.
fn coarse_facet_gate(
    carrier: &RefitCarrier,
    shell: &crate::topology::ShellRecord,
    selected: &[usize],
    edges: &HashMap<u64, &EdgeRecord>,
    vertices: &HashMap<u64, Vec3>,
) -> Result<(), KernelRefusal> {
    if matches!(carrier, RefitCarrier::Plane { .. }) {
        return Ok(());
    }
    let mut worst: Option<(f64, &FaceRecord)> = None;
    for &f in selected {
        let face = &shell.faces[f];
        if !face.surface.is_affine().unwrap_or(false) {
            continue;
        }
        let mut sum = Vec3::new(0.0, 0.0, 0.0);
        let mut count = 0usize;
        for coedge in face.loops.iter().flat_map(|l| l.coedges.iter()) {
            let Some(edge) = edges.get(&coedge.edge_id) else { continue };
            let id = if coedge.forward { edge.start_vertex_id } else { edge.end_vertex_id };
            if let Some(p) = vertices.get(&id) {
                sum = sum.add(*p);
                count += 1;
            }
        }
        if count < 3 {
            continue;
        }
        let centroid = sum.scale(1.0 / count as f64);
        let radius = local_radius(carrier, centroid);
        if !(radius > 0.0) {
            continue;
        }
        let sag = carrier.distance(centroid);
        let span = 2.0 * (1.0 - (sag / radius).min(2.0)).clamp(-1.0, 1.0).acos().to_degrees();
        if worst.map_or(true, |(w, _)| span > w) {
            worst = Some((span, face));
        }
    }
    match worst {
        Some((span, face)) if span > MAX_FACET_SPAN_DEGREES + FACET_SPAN_SLACK_DEGREES => {
            Err(KernelRefusal::ill_posed(
            KernelStage::Refine,
            "refit_facets_coarse",
            format!(
                "Refit Faces: the selection is {FACETS_TOO_COARSE} to be one {}: {} spans \
                 {span:.1} degrees of the fitted surface (at most {MAX_FACET_SPAN_DEGREES}); \
                 a facet may be no wider than the radius it approximates",
                carrier.kind().name(),
                face_label(face)
            ),
        ))
        }
        _ => Ok(()),
    }
}

/// The surface's smaller principal radius of curvature near `point`.
fn local_radius(carrier: &RefitCarrier, point: Vec3) -> f64 {
    match *carrier {
        RefitCarrier::Plane { .. } => f64::INFINITY,
        RefitCarrier::Cylinder { radius, .. } => radius,
        RefitCarrier::Sphere { radius, .. } => radius,
        RefitCarrier::Torus { minor_radius, .. } => minor_radius,
        RefitCarrier::Cone { apex, axis, half_angle } => {
            let rel = point.sub(apex);
            let radial = rel.sub(axis.scale(rel.dot(axis))).length();
            radial / half_angle.cos()
        }
    }
}

