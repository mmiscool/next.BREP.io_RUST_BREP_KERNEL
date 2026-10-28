//! The immutable triangle snapshot every CAM algorithm queries.
//!
//! A [`MeshSnapshot`] is built once per generation from the targets' and
//! fixtures' tessellations, in MACHINE coordinates, and never mutated. Each
//! triangle keeps the body and face it came from so drive-face regions and
//! per-face diagnostics resolve by stable identity, and triangle ids are the
//! positions in `triangles`, so an index built on a snapshot is valid for the
//! snapshot's lifetime.

use crate::model::{Bounds3, Point3, RigidTransform, TargetRef};
use crate::numeric::Tolerance;
use serde::{Deserialize, Serialize};

/// Which body and face a triangle came from.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FaceSource {
    /// Persistent body name.
    pub solid: String,
    /// Persistent face name within the body.
    pub face: String,
    /// `true` for fixtures and other bodies that are protected but never machined.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub protected_only: bool,
}

/// One triangle with its cached bounds, unit normal and source face.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Triangle {
    /// Position in the snapshot's `triangles`.
    pub id: u32,
    pub a: Point3,
    pub b: Point3,
    pub c: Point3,
    pub bounds: Bounds3,
    /// Unit normal `(b - a) × (c - a)`, normalized. Degenerate triangles never enter a snapshot.
    pub normal: Point3,
    /// Index into [`MeshSnapshot::faces`].
    pub face: u32,
}

impl Triangle {
    pub fn vertices(&self) -> [Point3; 3] {
        [self.a, self.b, self.c]
    }

    /// Twice the signed area vector; its length is twice the area.
    pub fn area_vector(&self) -> Point3 {
        self.b.sub(self.a).cross(self.c.sub(self.a))
    }

    pub fn area(&self) -> f64 {
        0.5 * self.area_vector().length()
    }

    pub fn centroid(&self) -> Point3 {
        self.a.add(self.b).add(self.c).scale(1.0 / 3.0)
    }

    /// The triangle's plane is vertical (normal perpendicular to Z) within the tolerance.
    pub fn is_vertical(&self, tol: &Tolerance) -> bool {
        tol.is_horizontal(self.normal)
    }

    /// The triangle's plane is horizontal within the tolerance.
    pub fn is_horizontal(&self, tol: &Tolerance) -> bool {
        tol.is_vertical(self.normal)
    }

    pub fn min_z(&self) -> f64 {
        self.bounds.min.z
    }

    pub fn max_z(&self) -> f64 {
        self.bounds.max.z
    }
}

/// Why a triangle or buffer was refused while building a snapshot.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MeshError {
    /// A coordinate was NaN or infinite.
    NonFinite { triangle: usize },
    /// `positions` is not a multiple of three, `indices` not a multiple of three,
    /// or `face_ids` does not have one entry per triangle.
    BufferShape(String),
    /// An index points past the vertex count.
    IndexOutOfRange { triangle: usize, index: u32 },
    /// A per-triangle face id is not in the face table.
    FaceOutOfRange { triangle: usize, face: u32 },
}

impl std::fmt::Display for MeshError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MeshError::NonFinite { triangle } => write!(f, "triangle {triangle} has a non-finite coordinate"),
            MeshError::BufferShape(msg) => write!(f, "mesh buffers: {msg}"),
            MeshError::IndexOutOfRange { triangle, index } => {
                write!(f, "triangle {triangle} references vertex {index} past the buffer")
            }
            MeshError::FaceOutOfRange { triangle, face } => {
                write!(f, "triangle {triangle} references face {face} past the face table")
            }
        }
    }
}

impl std::error::Error for MeshError {}

/// What the snapshot builder dropped or flagged. Zero-area triangles are
/// dropped silently (they contribute no surface); the count is reported so a
/// badly tessellated body is visible in the operation report.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MeshBuildReport {
    pub triangles_in: u64,
    pub triangles_kept: u64,
    pub degenerate_dropped: u64,
}

/// Builds a [`MeshSnapshot`] body by body, face by face.
#[derive(Debug, Default)]
pub struct MeshBuilder {
    faces: Vec<FaceSource>,
    triangles: Vec<Triangle>,
    bounds: Bounds3,
    report: MeshBuildReport,
    tolerance: Tolerance,
}

impl MeshBuilder {
    /// A builder using `tolerance.area_eps` to drop degenerate triangles.
    pub fn new(tolerance: Tolerance) -> Self {
        Self { tolerance, bounds: Bounds3::empty(), ..Default::default() }
    }

    /// Register a face and get its index; triangles are added against it.
    pub fn add_face(&mut self, source: FaceSource) -> u32 {
        self.faces.push(source);
        (self.faces.len() - 1) as u32
    }

    /// Add one triangle against a registered face. Degenerate triangles are
    /// counted and dropped; non-finite coordinates are an error.
    pub fn add_triangle(&mut self, face: u32, a: Point3, b: Point3, c: Point3) -> Result<(), MeshError> {
        let i = self.report.triangles_in as usize;
        self.report.triangles_in += 1;
        if face as usize >= self.faces.len() {
            return Err(MeshError::FaceOutOfRange { triangle: i, face });
        }
        if !a.is_finite() || !b.is_finite() || !c.is_finite() {
            return Err(MeshError::NonFinite { triangle: i });
        }
        let area_vector = b.sub(a).cross(c.sub(a));
        let twice_area = area_vector.length();
        if self.tolerance.is_zero_area(0.5 * twice_area) {
            self.report.degenerate_dropped += 1;
            return Ok(());
        }
        let normal = area_vector.scale(1.0 / twice_area);
        let bounds = Bounds3::from_points([a, b, c]);
        self.bounds = self.bounds.union(&bounds);
        let id = self.triangles.len() as u32;
        self.triangles.push(Triangle { id, a, b, c, bounds, normal, face });
        self.report.triangles_kept += 1;
        Ok(())
    }

    /// Add a whole body from flat buffers: `positions` is `xyz` per vertex,
    /// `indices` three per triangle, `face_ids` one per triangle indexing
    /// `face_names`. This is the shape the kernel's tessellation produces.
    pub fn add_body_buffers(
        &mut self,
        solid: &str,
        protected_only: bool,
        positions: &[f64],
        indices: &[u32],
        face_ids: &[u32],
        face_names: &[String],
    ) -> Result<(), MeshError> {
        if positions.len() % 3 != 0 {
            return Err(MeshError::BufferShape("positions length is not a multiple of three".into()));
        }
        if indices.len() % 3 != 0 {
            return Err(MeshError::BufferShape("indices length is not a multiple of three".into()));
        }
        let triangle_count = indices.len() / 3;
        if face_ids.len() != triangle_count {
            return Err(MeshError::BufferShape(format!(
                "face_ids has {} entries for {} triangles",
                face_ids.len(),
                triangle_count
            )));
        }
        let base_face = self.faces.len() as u32;
        for name in face_names {
            self.add_face(FaceSource { solid: solid.to_string(), face: name.clone(), protected_only });
        }
        let vertex_count = (positions.len() / 3) as u32;
        let point = |index: u32| -> Point3 {
            let i = index as usize * 3;
            Point3::new(positions[i], positions[i + 1], positions[i + 2])
        };
        for (t, tri) in indices.chunks_exact(3).enumerate() {
            for &index in tri {
                if index >= vertex_count {
                    return Err(MeshError::IndexOutOfRange { triangle: self.report.triangles_in as usize, index });
                }
            }
            let face_local = face_ids[t];
            if face_local as usize >= face_names.len() {
                return Err(MeshError::FaceOutOfRange { triangle: self.report.triangles_in as usize, face: face_local });
            }
            self.add_triangle(base_face + face_local, point(tri[0]), point(tri[1]), point(tri[2]))?;
        }
        Ok(())
    }

    pub fn finish(self) -> MeshSnapshot {
        MeshSnapshot { faces: self.faces, triangles: self.triangles, bounds: self.bounds, report: self.report }
    }
}

/// The immutable mesh every algorithm reads. Triangle `id == position`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct MeshSnapshot {
    pub faces: Vec<FaceSource>,
    pub triangles: Vec<Triangle>,
    pub bounds: Bounds3,
    #[serde(default)]
    pub report: MeshBuildReport,
}

impl MeshSnapshot {
    pub fn empty() -> Self {
        Self::default()
    }

    pub fn builder(tolerance: Tolerance) -> MeshBuilder {
        MeshBuilder::new(tolerance)
    }

    /// Same mesh in another coordinate frame. Triangle ids and face indices
    /// are unchanged; normals rotate; bounds are recomputed exactly.
    pub fn transformed(&self, transform: &RigidTransform) -> MeshSnapshot {
        if transform.is_identity() {
            return self.clone();
        }
        let mut bounds = Bounds3::empty();
        let triangles = self
            .triangles
            .iter()
            .map(|t| {
                let a = transform.apply_point(t.a);
                let b = transform.apply_point(t.b);
                let c = transform.apply_point(t.c);
                let tb = Bounds3::from_points([a, b, c]);
                bounds = bounds.union(&tb);
                Triangle { id: t.id, a, b, c, bounds: tb, normal: transform.apply_vector(t.normal), face: t.face }
            })
            .collect();
        MeshSnapshot { faces: self.faces.clone(), triangles, bounds, report: self.report }
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    pub fn triangle(&self, id: u32) -> &Triangle {
        &self.triangles[id as usize]
    }

    pub fn face_of(&self, id: u32) -> &FaceSource {
        &self.faces[self.triangles[id as usize].face as usize]
    }

    /// Face indices whose source matches `target` (whole body: every face of it).
    pub fn resolve_faces(&self, target: &TargetRef) -> Vec<u32> {
        self.faces
            .iter()
            .enumerate()
            .filter(|(_, f)| f.solid == target.solid && target.face.as_ref().map_or(true, |name| *name == f.face))
            .map(|(i, _)| i as u32)
            .collect()
    }

    /// Triangle ids whose face is in `faces` (ascending).
    pub fn triangles_of_faces(&self, faces: &[u32]) -> Vec<u32> {
        let mut mark = vec![false; self.faces.len()];
        for &f in faces {
            if let Some(m) = mark.get_mut(f as usize) {
                *m = true;
            }
        }
        self.triangles.iter().filter(|t| mark[t.face as usize]).map(|t| t.id).collect()
    }

    /// Triangle ids of a body or face, ascending; empty when unresolved.
    pub fn triangles_of(&self, target: &TargetRef) -> Vec<u32> {
        self.triangles_of_faces(&self.resolve_faces(target))
    }

    /// Sorted, de-duplicated body names present in the snapshot.
    pub fn solids(&self) -> Vec<String> {
        let mut names: Vec<String> = self.faces.iter().map(|f| f.solid.clone()).collect();
        names.sort();
        names.dedup();
        names
    }

    /// Bounds of the triangles whose ids are given.
    pub fn bounds_of(&self, ids: &[u32]) -> Bounds3 {
        ids.iter().fold(Bounds3::empty(), |b, &id| b.union(&self.triangles[id as usize].bounds))
    }
}

/// THE place model coordinates become machine coordinates for a mesh: the
/// setup's `model_to_machine` applied to a model-space snapshot. Algorithms
/// never transform geometry themselves; display conversion back goes through
/// [`crate::model::Setup::machine_to_model`].
pub fn to_machine(setup: &crate::model::Setup, model: &MeshSnapshot) -> MeshSnapshot {
    model.transformed(&setup.model_to_machine)
}

/// Read access other lanes code against, so an algorithm can run on a full
/// snapshot or on any other triangle source (a sub-mesh, a test fixture).
pub trait TriangleSource {
    fn triangle_count(&self) -> usize;
    fn triangle(&self, id: u32) -> &Triangle;
    fn bounds(&self) -> Bounds3;
    fn face_source(&self, face: u32) -> &FaceSource;
}

impl TriangleSource for MeshSnapshot {
    fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    fn triangle(&self, id: u32) -> &Triangle {
        &self.triangles[id as usize]
    }

    fn bounds(&self) -> Bounds3 {
        self.bounds
    }

    fn face_source(&self, face: u32) -> &FaceSource {
        &self.faces[face as usize]
    }
}

