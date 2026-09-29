//! IGES trimmed-NURBS-surface → kernel `BrepSolid` import.
//!
//! Reads every **144** Trimmed Surface as one kernel face: the **128** surface,
//! and one loop per **142** Curve-on-Surface. For each loop coedge the primary
//! path reads the parameter-space boundary (`BPTR`) directly as the coedge
//! pcurve and the model-space boundary (`CPTR`) as the 3D edge curve — a
//! lossless round-trip of our own exports. When those disagree (a foreign file
//! whose parameter and model curves don't correspond one-to-one) it falls back
//! to deriving the pcurve by projecting the 3D curve onto the carrier surface
//! with [`crate::build_pcurve_on_surface_range`], mirroring the STEP importer.
//!
//! The reconstructed faces form a face soup that [`crate::sew_solid`] pairs into
//! a coherently oriented, outward-facing solid; unsupported entities (bare
//! untrimmed surfaces, MSBO 186) yield a precise error rather than a panic.

use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{build_pcurve_on_surface_range, sew_solid, NurbsCurve, NurbsSurface, Vec3};

use super::entities::{
    composite_members, curve_from_126, curve_on_surface_from_142, surface_from_128,
    trimmed_surface_from_144,
};
use super::reader::{parse_iges, IgesFile};

/// Parse an IGES document and reconstruct one or more `BrepSolid`s.
pub fn import_iges(text: &str) -> Result<Vec<BrepSolid>, String> {
    let file = parse_iges(text)?;
    let scale = file.global.length_scale_mm()?;

    let mut soup = SoupBuilder::new();
    let mut faces: Vec<FaceRecord> = Vec::new();
    for entity in file.entities_of_type(144) {
        let ts = trimmed_surface_from_144(entity)?;
        let face = soup.build_face(&file, &ts, scale)?;
        faces.push(face);
    }

    if faces.is_empty() {
        // Precise diagnostics for the common unsupported representations.
        if file.entities_of_type(186).next().is_some() {
            return Err(
                "iges_import: file uses Manifold Solid B-Rep Object (entity 186), which is not yet supported"
                    .into(),
            );
        }
        if file.entities_of_type(128).next().is_some() {
            return Err(
                "iges_import: file has untrimmed B-spline surfaces (entity 128) but no trimmed surfaces (entity 144) to form faces"
                    .into(),
            );
        }
        return Err("iges_import: no trimmed NURBS surfaces (entity 144) found".into());
    }

    // Characteristic size for sew tolerance.
    let diag = soup.bounding_diagonal();
    let sew_tol = (1e-6 * diag).max(1e-9);

    // IGES has no body grouping, so the file is a flat face set. Partition it
    // into connected components (faces sharing a welded vertex belong to the
    // same body) and sew each into its own solid — matching the STEP importer's
    // one-solid-per-body contract. A single body is one component, so this is
    // behaviour-identical to a single sew for the common case.
    let mut next_id = soup.next_id;
    let SoupBuilder { vertices, edges, .. } = soup;
    let components = partition_faces_into_bodies(&faces, &edges);

    let mut solids = Vec::with_capacity(components.len());
    for face_indices in components {
        let body_faces: Vec<FaceRecord> =
            face_indices.iter().map(|&i| faces[i].clone()).collect();
        let mut edge_ids: rustc_hash::FxHashSet<u64> = Default::default();
        for face in &body_faces {
            for loop_record in &face.loops {
                for coedge in &loop_record.coedges {
                    edge_ids.insert(coedge.edge_id);
                }
            }
        }
        let body_edges: Vec<EdgeRecord> =
            edges.iter().filter(|e| edge_ids.contains(&e.id)).cloned().collect();
        let mut vertex_ids: rustc_hash::FxHashSet<u64> = Default::default();
        for edge in &body_edges {
            vertex_ids.insert(edge.start_vertex_id);
            vertex_ids.insert(edge.end_vertex_id);
        }
        let body_vertices: Vec<VertexRecord> =
            vertices.iter().filter(|v| vertex_ids.contains(&v.id)).cloned().collect();

        let shell_id = next_id;
        let solid_id = next_id + 1;
        next_id += 2;
        let solid = BrepSolid {
            mass_properties_cache: Default::default(),
            id: solid_id,
            vertices: body_vertices,
            edges: body_edges,
            shells: vec![ShellRecord {
                id: shell_id,
                faces: body_faces,
            }],
            genus: 0,
        };
        let (sewn, _report) = sew_solid(&solid, sew_tol)?;
        let issues = sewn.validate();
        if !issues.is_empty() {
            return Err(format!(
                "iges_import: reconstructed solid failed validation: {issues:?}"
            ));
        }
        solids.push(sewn);
    }
    Ok(solids)
}

/// Group faces into bodies by shared welded vertices (union-find). Each returned
/// vector holds the indices (into `faces`) of one connected component.
fn partition_faces_into_bodies(faces: &[FaceRecord], edges: &[EdgeRecord]) -> Vec<Vec<usize>> {
    use rustc_hash::FxHashMap as HashMap;

    let edge_vertices: HashMap<u64, (u64, u64)> = edges
        .iter()
        .map(|e| (e.id, (e.start_vertex_id, e.end_vertex_id)))
        .collect();

    let mut parent: Vec<usize> = (0..faces.len()).collect();
    fn find(parent: &mut Vec<usize>, mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }

    // First face that referenced a given vertex; union subsequent faces to it.
    let mut vertex_owner: HashMap<u64, usize> = HashMap::default();
    for (face_index, face) in faces.iter().enumerate() {
        for loop_record in &face.loops {
            for coedge in &loop_record.coedges {
                if let Some(&(a, b)) = edge_vertices.get(&coedge.edge_id) {
                    for vertex in [a, b] {
                        match vertex_owner.get(&vertex) {
                            Some(&owner) => {
                                let ra = find(&mut parent, owner);
                                let rb = find(&mut parent, face_index);
                                parent[ra] = rb;
                            }
                            None => {
                                vertex_owner.insert(vertex, face_index);
                            }
                        }
                    }
                }
            }
        }
    }

    let mut groups: HashMap<usize, Vec<usize>> = HashMap::default();
    for face_index in 0..faces.len() {
        let root = find(&mut parent, face_index);
        groups.entry(root).or_default().push(face_index);
    }
    let mut components: Vec<Vec<usize>> = groups.into_values().collect();
    // Deterministic order: by smallest face index in each component.
    components.sort_by_key(|c| c.iter().copied().min().unwrap_or(0));
    components
}

/// Accumulates the shared vertex/edge pools while building faces, welding
/// coincident vertices so loops close and adjacent faces share endpoints.
struct SoupBuilder {
    next_id: u64,
    vertices: Vec<VertexRecord>,
    edges: Vec<EdgeRecord>,
}

impl SoupBuilder {
    fn new() -> Self {
        Self {
            next_id: 1,
            vertices: Vec::new(),
            edges: Vec::new(),
        }
    }

    fn fresh(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn weld_vertex(&mut self, point: Vec3) -> u64 {
        let tol = 1e-7 * (1.0 + point.length());
        if let Some(existing) = self
            .vertices
            .iter()
            .find(|v| v.point.sub(point).length() <= tol)
        {
            return existing.id;
        }
        let id = self.fresh();
        self.vertices.push(VertexRecord { id, point });
        id
    }

    fn bounding_diagonal(&self) -> f64 {
        if self.vertices.is_empty() {
            return 1.0;
        }
        let mut lo = self.vertices[0].point;
        let mut hi = self.vertices[0].point;
        for v in &self.vertices {
            lo.x = lo.x.min(v.point.x);
            lo.y = lo.y.min(v.point.y);
            lo.z = lo.z.min(v.point.z);
            hi.x = hi.x.max(v.point.x);
            hi.y = hi.y.max(v.point.y);
            hi.z = hi.z.max(v.point.z);
        }
        hi.sub(lo).length().max(1.0)
    }

    fn build_face(
        &mut self,
        file: &IgesFile,
        ts: &super::entities::TrimmedSurface,
        scale: f64,
    ) -> Result<FaceRecord, String> {
        let surface_entity = file.entity(ts.surface)?;
        if surface_entity.de.entity_type != 128 {
            return Err(format!(
                "iges_import: trimmed surface points to entity type {} (expected 128)",
                surface_entity.de.entity_type
            ));
        }
        let surface = surface_from_128(surface_entity, scale)?;

        let mut loops: Vec<LoopRecord> = Vec::new();
        // Outer boundary first (kernel convention), then holes. `N1=0` means the
        // outer boundary is the surface's natural parameter domain (no explicit
        // outer loop); reconstructing that would need a synthetic domain-edge
        // loop, so it is refused loudly rather than yielding a hole-only face.
        if ts.outer.is_none() {
            return Err(
                "iges_import: trimmed surface with an implicit (natural-domain) outer boundary (144 N1=0) is not supported"
                    .into(),
            );
        }
        let mut boundary_ptrs: Vec<i64> = Vec::new();
        boundary_ptrs.extend(ts.outer);
        boundary_ptrs.extend(ts.inner.iter().copied());
        for ptr in boundary_ptrs {
            loops.push(self.build_loop(file, ptr, ts.surface, &surface, scale)?);
        }

        // Orientation is encoded by `same_sense` together with loop winding:
        // flipping a face toggles `same_sense` and reverses its loops, so a
        // valid outward face has `same_sense == (outer-loop parameter-space
        // winding is CCW)`. The pcurves are preserved faithfully, so recover
        // the original sense directly rather than leaving it to sew heuristics
        // (which mis-resolve concave inner-loop walls).
        let same_sense = loops
            .first()
            .map(|outer| loop_param_signed_area(outer) >= 0.0)
            .unwrap_or(true);

        let id = self.fresh();
        Ok(FaceRecord {
            id,
            surface,
            same_sense,
            loops,
            name: None,
        })
    }

    fn build_loop(
        &mut self,
        file: &IgesFile,
        boundary_ptr: i64,
        surface_ptr: i64,
        surface: &NurbsSurface,
        scale: f64,
    ) -> Result<LoopRecord, String> {
        let cos_entity = file.entity(boundary_ptr)?;
        if cos_entity.de.entity_type != 142 {
            return Err(format!(
                "iges_import: boundary points to entity type {} (expected 142 curve-on-surface)",
                cos_entity.de.entity_type
            ));
        }
        let cos = curve_on_surface_from_142(cos_entity)?;
        if cos.surface != surface_ptr {
            return Err(format!(
                "iges_import: curve-on-surface references surface DE {} but its trimmed surface is DE {surface_ptr}",
                cos.surface
            ));
        }
        let model_curves = resolve_curve_list(file, cos.model_curve, Some(scale))?;
        let param_curves = resolve_curve_list(file, cos.param_curve, None)?;

        let use_faithful =
            !param_curves.is_empty() && param_curves.len() == model_curves.len() && {
                // Sanity: parameter and model boundary must agree at a midpoint.
                faithful_pcurves_agree(surface, &model_curves[0], &param_curves[0])
            };

        let mut coedges: Vec<CoedgeRecord> = Vec::new();
        for (index, model_curve) in model_curves.iter().enumerate() {
            let pcurve = if use_faithful {
                param_curves[index].clone()
            } else {
                derive_pcurve(surface, model_curve)?
            };
            coedges.push(self.build_coedge(model_curve, pcurve)?);
        }
        let id = self.fresh();
        Ok(LoopRecord { id, coedges })
    }

    fn build_coedge(
        &mut self,
        model_curve: &NurbsCurve,
        pcurve: NurbsCurve,
    ) -> Result<CoedgeRecord, String> {
        let [t0, t1] = model_curve.domain()?;
        let p_start = model_curve.evaluate(t0)?;
        let p_end = model_curve.evaluate(t1)?;
        let start_vertex_id = self.weld_vertex(p_start);
        let end_vertex_id = self.weld_vertex(p_end);
        let degenerate = start_vertex_id == end_vertex_id && curve_collapsed(model_curve, p_start);
        let edge_id = self.fresh();
        self.edges.push(EdgeRecord {
            id: edge_id,
            curve: model_curve.clone(),
            t0,
            t1,
            start_vertex_id,
            end_vertex_id,
            degenerate,
            name: None,
        });
        let coedge_id = self.fresh();
        Ok(CoedgeRecord {
            id: coedge_id,
            edge_id,
            forward: true,
            pcurve,
        })
    }
}

/// Resolve a boundary pointer (a 102 composite or a bare 126) into its ordered
/// list of NURBS curves. `coord_scale` is `Some` for 3D model curves, `None`
/// for parameter-space curves.
fn resolve_curve_list(
    file: &IgesFile,
    ptr: i64,
    coord_scale: Option<f64>,
) -> Result<Vec<NurbsCurve>, String> {
    let entity = file.entity(ptr)?;
    match entity.de.entity_type {
        102 => {
            let members = composite_members(entity)?;
            let mut curves = Vec::with_capacity(members.len());
            for m in members {
                let member = file.entity(m)?;
                if member.de.entity_type != 126 {
                    return Err(format!(
                        "iges_import: composite member is entity type {} (expected 126)",
                        member.de.entity_type
                    ));
                }
                curves.push(curve_from_126(member, coord_scale)?);
            }
            Ok(curves)
        }
        126 => Ok(vec![curve_from_126(entity, coord_scale)?]),
        other => Err(format!(
            "iges_import: boundary curve is entity type {other} (expected 102 or 126)"
        )),
    }
}

/// Check that a parameter-space curve evaluated on the surface reproduces the
/// model-space curve at their common midpoint.
fn faithful_pcurves_agree(surface: &NurbsSurface, model: &NurbsCurve, param: &NurbsCurve) -> bool {
    let evaluate = || -> Result<bool, String> {
        let [pt0, pt1] = param.domain()?;
        let mid = 0.5 * (pt0 + pt1);
        let uv = param.evaluate(mid)?;
        let on_surface = surface.evaluate(uv.x, uv.y)?;
        let [mt0, mt1] = model.domain()?;
        let model_mid = model.evaluate(0.5 * (mt0 + mt1))?;
        let tol = 1e-4 * (1.0 + model_mid.length());
        Ok(on_surface.sub(model_mid).length() <= tol)
    };
    evaluate().unwrap_or(false)
}

/// Derive a pcurve for a foreign file by projecting the 3D curve onto the
/// carrier surface.
fn derive_pcurve(surface: &NurbsSurface, model: &NurbsCurve) -> Result<NurbsCurve, String> {
    let [t0, t1] = model.domain()?;
    let magnitude = model.evaluate(0.5 * (t0 + t1)).map(|p| p.length()).unwrap_or(1.0);
    let tol = 1e-6 * (1.0 + magnitude);
    build_pcurve_on_surface_range(surface, model, t0, t1, true, tol)
}

/// Signed area of a loop in surface parameter space (shoelace over sampled
/// pcurve points). Positive = counter-clockwise in `(u, v)`.
fn loop_param_signed_area(loop_record: &LoopRecord) -> f64 {
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for coedge in &loop_record.coedges {
        let curve = &coedge.pcurve;
        let [t0, t1] = match curve.domain() {
            Ok(d) => d,
            Err(_) => continue,
        };
        // Sample the interior of each pcurve; endpoints are shared with
        // neighbours so the concatenation forms the closed loop polygon.
        const SAMPLES: usize = 8;
        for i in 0..SAMPLES {
            let t = t0 + (t1 - t0) * i as f64 / SAMPLES as f64;
            if let Ok(p) = curve.evaluate(t) {
                pts.push((p.x, p.y));
            }
        }
    }
    if pts.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0;
    for i in 0..pts.len() {
        let (x0, y0) = pts[i];
        let (x1, y1) = pts[(i + 1) % pts.len()];
        area += x0 * y1 - x1 * y0;
    }
    0.5 * area
}

/// Multi-station collapse test: true when the curve stays within tolerance of a
/// single point (a pole/degenerate edge), distinguishing it from a closed loop.
fn curve_collapsed(curve: &NurbsCurve, at: Vec3) -> bool {
    let tol = 1e-7 * (1.0 + at.length());
    let [t0, t1] = match curve.domain() {
        Ok(d) => d,
        Err(_) => return false,
    };
    for index in 1..8 {
        let t = t0 + (t1 - t0) * index as f64 / 8.0;
        match curve.evaluate(t) {
            Ok(p) => {
                if p.sub(at).length() > tol {
                    return false;
                }
            }
            Err(_) => return false,
        }
    }
    true
}
