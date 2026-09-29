use crate::numerical;
use crate::{RecognitionError, SurfaceHint, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
/// An indexed triangle mesh and optional recognition metadata.
pub struct Mesh {
    /// Vertex positions in model coordinates.
    pub vertices: Vec<Vec3>,
    /// Triangle vertex indices; winding defines the geometric normal.
    pub triangles: Vec<[u32; 3]>,
    /// Optional unit-normal hints, one per vertex. These improve curved-surface
    /// fitting without replacing triangle winding as the topology authority.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vertex_normals: Option<Vec<Vec3>>,
    #[serde(default)]
    /// Versioned metadata associated with subsets of triangles.
    pub source_metadata: Vec<SourceMetadata>,
}

impl Mesh {
    /// Creates a mesh without vertex-normal hints or source metadata.
    pub fn new(vertices: Vec<Vec3>, triangles: Vec<[u32; 3]>) -> Self {
        Self {
            vertices,
            triangles,
            vertex_normals: None,
            source_metadata: Vec::new(),
        }
    }

    /// Attach per-vertex normal hints. Length and values are validated by
    /// [`Mesh::analyze`], where finite non-zero vectors are normalized.
    pub fn with_vertex_normals(mut self, normals: Vec<Vec3>) -> Self {
        self.vertex_normals = Some(normals);
        self
    }
    /// Validates the mesh and computes geometry and adjacency information.
    pub fn analyze(&self, options: &MeshAnalysisOptions) -> Result<AnalyzedMesh, RecognitionError> {
        AnalyzedMesh::new(self, options)
    }
}

/// Versioned sidecar metadata. Stable source IDs are deliberately distinct
/// from sequential tessellation face indices.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct SourceMetadata {
    /// Sidecar schema version. Version 1 is currently supported.
    pub version: u32,
    /// Mesh triangles to which this metadata applies.
    pub triangle_indices: Vec<usize>,
    /// Prior knowledge about the source surface.
    pub hint: SurfaceHint,
    /// Stable source-system face identifier, when available.
    pub source_face_id: Option<u64>,
    /// Human-readable source-system face name, when available.
    pub source_face_name: Option<String>,
    /// Stable source-system surface identifier, when available.
    pub source_surface_id: Option<String>,
    /// Source orientation relative to triangle winding, as `-1` or `1`.
    pub orientation: Option<i8>,
    /// Positive source-model geometric tolerance, when available.
    pub source_tolerance: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[serde(default)]
/// Controls mesh validation and feature-edge classification.
pub struct MeshAnalysisOptions {
    /// Triangles at or below this area are treated as degenerate.
    pub minimum_triangle_area: f64,
    /// Dihedral-angle threshold, in radians, for feature edges.
    pub feature_angle: f64,
}
impl Default for MeshAnalysisOptions {
    fn default() -> Self {
        Self {
            minimum_triangle_area: 0.0,
            feature_angle: 30_f64.to_radians(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
/// Geometry and topology derived for one input triangle.
pub struct TriangleData {
    /// Indices of the triangle's three vertices.
    pub vertices: [usize; 3],
    /// Arithmetic mean of the three vertex positions.
    pub centroid: Vec3,
    /// Unit normal derived from winding, or zero for a rejected triangle.
    pub normal: Vec3,
    /// Triangle area, or zero when the triangle is degenerate.
    pub area: f64,
    /// Adjacent triangle across each directed edge, when manifold and present.
    pub neighbors: [Option<usize>; 3],
    /// Whether each edge is a boundary, non-manifold edge, or sharp feature.
    pub feature_edges: [bool; 3],
}

#[derive(Clone, Debug)]
/// A validated mesh with cached geometry, topology, and sampling weights.
pub struct AnalyzedMesh {
    /// Validated vertex positions.
    pub vertices: Vec<Vec3>,
    /// Normalized supplied vertex normals, when present on the source mesh.
    pub vertex_normals: Option<Vec<Vec3>>,
    /// Derived data for every input triangle.
    pub triangles: Vec<TriangleData>,
    /// Component-wise minimum of all vertex positions.
    pub bbox_min: Vec3,
    /// Component-wise maximum of all vertex positions.
    pub bbox_max: Vec3,
    /// Length of the axis-aligned bounding-box diagonal.
    pub diagonal: f64,
    /// One-third of each incident usable triangle's area, accumulated per vertex.
    pub vertex_area_weights: Vec<f64>,
    /// Validated source metadata copied from the input mesh.
    pub source_metadata: Vec<SourceMetadata>,
    /// Indices of triangles rejected as degenerate.
    pub degenerate_triangles: Vec<usize>,
}

impl AnalyzedMesh {
    /// Validates `mesh` and constructs its analyzed representation.
    pub fn new(mesh: &Mesh, options: &MeshAnalysisOptions) -> Result<Self, RecognitionError> {
        if !options.minimum_triangle_area.is_finite() || options.minimum_triangle_area < 0.0 {
            return Err(RecognitionError::InvalidOptions(
                "minimum_triangle_area must be finite and non-negative".into(),
            ));
        }
        if !(0.0..=std::f64::consts::PI).contains(&options.feature_angle) {
            return Err(RecognitionError::InvalidOptions(
                "feature_angle must be finite and in [0, pi]".into(),
            ));
        }
        if mesh.vertices.is_empty() {
            return Err(RecognitionError::InvalidMesh("no vertices".into()));
        }
        if mesh.triangles.is_empty() {
            return Err(RecognitionError::InvalidMesh("no triangles".into()));
        }
        if mesh.vertices.iter().any(|v| !v.is_finite()) {
            return Err(RecognitionError::InvalidMesh(
                "vertex coordinates must be finite".into(),
            ));
        }
        let vertex_normals = match &mesh.vertex_normals {
            None => None,
            Some(normals) => {
                if normals.len() != mesh.vertices.len() {
                    return Err(RecognitionError::InvalidMesh(format!(
                        "vertex-normal buffer has {} entries for {} vertices",
                        normals.len(),
                        mesh.vertices.len()
                    )));
                }
                let mut normalized = Vec::with_capacity(normals.len());
                for (index, &normal) in normals.iter().enumerate() {
                    if !normal.is_finite() {
                        return Err(RecognitionError::InvalidMesh(format!(
                            "vertex normal {index} is not finite"
                        )));
                    }
                    normalized.push(normal.normalized().ok_or_else(|| {
                        RecognitionError::InvalidMesh(format!(
                            "vertex normal {index} has zero length"
                        ))
                    })?);
                }
                Some(normalized)
            }
        };
        let mut bbox_min = mesh.vertices[0];
        let mut bbox_max = mesh.vertices[0];
        for p in &mesh.vertices[1..] {
            bbox_min.x = bbox_min.x.min(p.x);
            bbox_min.y = bbox_min.y.min(p.y);
            bbox_min.z = bbox_min.z.min(p.z);
            bbox_max.x = bbox_max.x.max(p.x);
            bbox_max.y = bbox_max.y.max(p.y);
            bbox_max.z = bbox_max.z.max(p.z);
        }
        let diagonal = (bbox_max - bbox_min).length();
        if diagonal <= 0.0 {
            return Err(RecognitionError::InvalidMesh(
                "zero-size bounding box".into(),
            ));
        }
        let mut triangles = Vec::with_capacity(mesh.triangles.len());
        let mut degenerate = Vec::new();
        let mut vertex_area_weights = vec![0.0; mesh.vertices.len()];
        for (id, raw) in mesh.triangles.iter().enumerate() {
            let vi = raw.map(|x| x as usize);
            if vi.iter().any(|&x| x >= mesh.vertices.len()) {
                return Err(RecognitionError::InvalidMesh(format!(
                    "triangle {id} has an out-of-range vertex"
                )));
            }
            if vi[0] == vi[1] || vi[1] == vi[2] || vi[2] == vi[0] {
                degenerate.push(id);
                triangles.push(TriangleData {
                    vertices: vi,
                    centroid: Vec3::ZERO,
                    normal: Vec3::ZERO,
                    area: 0.0,
                    neighbors: [None; 3],
                    feature_edges: [true; 3],
                });
                continue;
            }
            let [a, b, c] = vi.map(|i| mesh.vertices[i]);
            let ab = b - a;
            let ac = c - a;
            let bc = c - b;
            let cross = ab.cross(ac);
            let area = 0.5 * cross.length();
            // A component's triangles must not become "degenerate" merely
            // because an unrelated component makes the mesh-wide bounding box
            // enormous. Bound cross-product uncertainty from this triangle's
            // own edge scale and the coordinate cancellation involved in its
            // vertex subtractions. This retains valid mixed-scale components
            // while still rejecting geometry smaller than representable
            // precision at a far world-space origin.
            let edge_scale = ab.length().max(ac.length()).max(bc.length());
            let coordinate_scale = [a, b, c]
                .into_iter()
                .map(|point| point.x.abs().max(point.y.abs()).max(point.z.abs()))
                .fold(edge_scale, f64::max);
            let subtraction_error = f64::EPSILON * coordinate_scale;
            let machine_area_floor = numerical::mesh::MACHINE_AREA_FLOOR_MULTIPLIER
                * (edge_scale * subtraction_error + subtraction_error * subtraction_error);
            let area_floor = options.minimum_triangle_area.max(machine_area_floor);
            if area <= area_floor {
                degenerate.push(id);
                triangles.push(TriangleData {
                    vertices: vi,
                    centroid: (a + b + c) / 3.0,
                    normal: Vec3::ZERO,
                    // Downstream topology and selection APIs use zero area as
                    // the canonical marker for a rejected triangle.
                    area: 0.0,
                    neighbors: [None; 3],
                    feature_edges: [true; 3],
                });
                continue;
            }
            let normal = cross / (2.0 * area);
            for &v in &vi {
                vertex_area_weights[v] += area / 3.0;
            }
            triangles.push(TriangleData {
                vertices: vi,
                centroid: (a + b + c) / 3.0,
                normal,
                area,
                neighbors: [None; 3],
                feature_edges: [true; 3],
            });
        }
        if degenerate.len() == mesh.triangles.len() {
            return Err(RecognitionError::DegenerateData(
                "all triangles are degenerate".into(),
            ));
        }
        let mut edges: BTreeMap<(usize, usize), Vec<(usize, usize)>> = BTreeMap::new();
        for (tid, t) in triangles.iter().enumerate() {
            if t.area <= 0.0 {
                continue;
            }
            for edge in 0..3 {
                let a = t.vertices[edge];
                let b = t.vertices[(edge + 1) % 3];
                edges
                    .entry(if a < b { (a, b) } else { (b, a) })
                    .or_default()
                    .push((tid, edge));
            }
        }
        for incidents in edges.values() {
            if incidents.len() == 2 {
                let (ta, ea) = incidents[0];
                let (tb, eb) = incidents[1];
                triangles[ta].neighbors[ea] = Some(tb);
                triangles[tb].neighbors[eb] = Some(ta);
                let cosine = triangles[ta]
                    .normal
                    .dot(triangles[tb].normal)
                    .clamp(-1.0, 1.0);
                let feature = cosine.acos() > options.feature_angle;
                triangles[ta].feature_edges[ea] = feature;
                triangles[tb].feature_edges[eb] = feature;
            }
        }
        for metadata in &mesh.source_metadata {
            if metadata.version != 1 {
                return Err(RecognitionError::InvalidMesh(format!(
                    "unsupported metadata version {}",
                    metadata.version
                )));
            }
            if metadata
                .triangle_indices
                .iter()
                .any(|&t| t >= mesh.triangles.len())
            {
                return Err(RecognitionError::InvalidMesh(
                    "metadata references an out-of-range triangle".into(),
                ));
            }
            if metadata
                .orientation
                .is_some_and(|sense| !matches!(sense, -1 | 1))
            {
                return Err(RecognitionError::InvalidMesh(
                    "metadata orientation must be -1 or +1".into(),
                ));
            }
            if metadata
                .source_tolerance
                .is_some_and(|tolerance| !tolerance.is_finite() || tolerance <= 0.0)
            {
                return Err(RecognitionError::InvalidMesh(
                    "metadata source_tolerance must be finite and positive".into(),
                ));
            }
        }
        Ok(Self {
            vertices: mesh.vertices.clone(),
            vertex_normals,
            triangles,
            bbox_min,
            bbox_max,
            diagonal,
            vertex_area_weights,
            source_metadata: mesh.source_metadata.clone(),
            degenerate_triangles: degenerate,
        })
    }
    /// Returns the indices of all triangles with positive analyzed area.
    pub fn all_non_degenerate(&self) -> Vec<usize> {
        self.triangles
            .iter()
            .enumerate()
            .filter_map(|(i, t)| (t.area > 0.0).then_some(i))
            .collect()
    }
    /// Checks that a selection is non-empty, in range, and contains usable geometry.
    pub fn validate_selection(&self, ids: &[usize]) -> Result<(), RecognitionError> {
        if ids.is_empty() {
            return Err(RecognitionError::InvalidSelection(
                "selection is empty".into(),
            ));
        }
        if ids.iter().any(|&i| i >= self.triangles.len()) {
            return Err(RecognitionError::InvalidSelection(
                "triangle index out of range".into(),
            ));
        }
        if ids.iter().all(|&i| self.triangles[i].area == 0.0) {
            return Err(RecognitionError::DegenerateData(
                "selection contains no usable triangles".into(),
            ));
        }
        Ok(())
    }
    /// Checks that a vertex selection is non-empty, unique, in range, and
    /// incident to usable mesh geometry.
    pub fn validate_vertex_selection(&self, ids: &[usize]) -> Result<(), RecognitionError> {
        if ids.is_empty() {
            return Err(RecognitionError::InvalidSelection(
                "vertex selection is empty".into(),
            ));
        }
        if ids.iter().any(|&i| i >= self.vertices.len()) {
            return Err(RecognitionError::InvalidSelection(
                "vertex index out of range".into(),
            ));
        }
        let mut unique = ids.to_vec();
        unique.sort_unstable();
        if unique.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(RecognitionError::InvalidSelection(
                "vertex selection contains duplicate indices".into(),
            ));
        }
        if ids.iter().any(|&i| self.vertex_area_weights[i] == 0.0) {
            return Err(RecognitionError::DegenerateData(
                "vertex selection contains a vertex with no usable incident geometry".into(),
            ));
        }
        Ok(())
    }
    /// Returns area weights for vertices incident to the selected usable triangles.
    pub fn selected_vertex_weights(&self, ids: &[usize]) -> Vec<(usize, f64)> {
        let mut weights = BTreeMap::<usize, f64>::new();
        for &tid in ids {
            let t = &self.triangles[tid];
            if t.area > 0.0 {
                for &v in &t.vertices {
                    *weights.entry(v).or_default() += t.area / 3.0;
                }
            }
        }
        weights.into_iter().collect()
    }
    /// Returns the area-weighted centroid of selected triangles.
    ///
    /// Returns the zero vector when the selection has no positive-area triangle.
    pub fn selection_centroid(&self, ids: &[usize]) -> Vec3 {
        let mut sum = Vec3::ZERO;
        let mut area = 0.0;
        for &i in ids {
            let t = &self.triangles[i];
            sum += t.centroid * t.area;
            area += t.area;
        }
        if area > 0.0 {
            sum / area
        } else {
            Vec3::ZERO
        }
    }
    /// Partitions selected usable triangles into edge-connected components.
    ///
    /// When `respect_features` is true, traversal does not cross feature edges.
    pub fn connected_components(&self, ids: &[usize], respect_features: bool) -> Vec<Vec<usize>> {
        let mut allowed = vec![false; self.triangles.len()];
        for &i in ids {
            if i < allowed.len() {
                allowed[i] = true;
            }
        }
        let mut seen = vec![false; self.triangles.len()];
        let mut result = Vec::new();
        for &seed in ids {
            if seen[seed] || self.triangles[seed].area == 0.0 {
                continue;
            }
            let mut stack = vec![seed];
            seen[seed] = true;
            let mut part = Vec::new();
            while let Some(i) = stack.pop() {
                part.push(i);
                for edge in 0..3 {
                    if respect_features && self.triangles[i].feature_edges[edge] {
                        continue;
                    }
                    if let Some(n) = self.triangles[i].neighbors[edge] {
                        if allowed[n] && !seen[n] {
                            seen[n] = true;
                            stack.push(n);
                        }
                    }
                }
            }
            part.sort_unstable();
            result.push(part);
        }
        result
    }
}

