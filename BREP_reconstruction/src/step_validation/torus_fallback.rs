//! Trim-aware observation mesh fallbacks for empty per-face tessellations.
//!
//! The kernel's legacy per-face scanline tessellator can legitimately return
//! an empty mesh for torus trims represented by full-wrap parameter rims or
//! collapsed vertex loops.  Replacing such a face with an untrimmed torus
//! would fabricate observations outside the STEP face.  This adapter instead
//! calls the kernel's public watertight face-stride tessellator for the exact
//! source solid and global face index.  That path consumes the authored trim
//! loops, shared edge samples, periodic seam rules, holes, and face sense.
//! This is source-face observation recovery, not an independent ordinary
//! tessellation path: corrections back to the source carrier are accepted only
//! within the configured, scale-aware observation tolerance.

use super::TessellationFallbackMethod;
use crate::numerical::{scalar, step_validation as numerical};
use brep_kernel::{AnalyticSurface as KernelAnalyticSurface, BrepSolid, FaceRecord, Mesh, Vec3};

/// Stable machine-readable name recorded in validation evidence.
pub(super) const METHOD_NAME: &str = "WatertightFaceStride";
/// Stable machine-readable name for the narrow planar trim-curve recovery.
pub(super) const TRIM_CURVE_TRIANGLE_METHOD_NAME: &str = "ProjectedTrimCurveTriangle";

#[derive(Debug)]
pub(super) struct FallbackMesh {
    pub(super) mesh: Mesh,
    pub(super) method: TessellationFallbackMethod,
    pub(super) surface_projected_vertices: usize,
    pub(super) max_surface_projection_distance: f64,
    pub(super) surface_projection_tolerance: f64,
}

/// Lazily sampled, per-solid fallback context.
///
/// Edge samples are shared by every fallback face in the solid so a file with
/// several empty legacy torus meshes does not resample the complete topology
/// for every face.
pub(super) struct ObservationFallback<'a> {
    solid: &'a BrepSolid,
    face_count: usize,
    chord_tolerance: f64,
    distance_tolerance: f64,
    relative_face_tolerance: f64,
    encoded_edge_samples: Option<Result<Vec<f64>, String>>,
}

impl<'a> ObservationFallback<'a> {
    pub(super) fn new(
        solid: &'a BrepSolid,
        distance_tolerance: f64,
        relative_face_tolerance: f64,
    ) -> Self {
        let face_count = solid
            .shells
            .iter()
            .map(|shell| shell.faces.len())
            .sum::<usize>();
        let solid_scale = brep_kernel::solid_scale(solid);
        let minimum_torus_minor_radius = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter_map(|face| match face.surface.analytic() {
                Some(KernelAnalyticSurface::Torus { minor_radius, .. })
                    if minor_radius.is_finite() && *minor_radius > 0.0 =>
                {
                    Some(*minor_radius)
                }
                _ => None,
            })
            .min_by(f64::total_cmp);
        // Keep a stable relative density for the whole part, but do not let a
        // small torus tube on a large body collapse to a handful of facets.
        // The scale-relative floor prevents pathological subnormal requests.
        let chord_tolerance = minimum_torus_minor_radius
            .map_or(
                solid_scale * numerical::FALLBACK_CHORD_SOLID_RELATIVE,
                |radius| {
                    (solid_scale * numerical::FALLBACK_CHORD_SOLID_RELATIVE)
                        .min(radius * numerical::FALLBACK_CHORD_MINOR_RADIUS_RELATIVE)
                },
            )
            .max(solid_scale * numerical::FALLBACK_CHORD_RELATIVE_FLOOR);
        Self {
            solid,
            face_count,
            chord_tolerance,
            distance_tolerance,
            relative_face_tolerance,
            encoded_edge_samples: None,
        }
    }

    pub(super) fn chord_tolerance(&self) -> f64 {
        self.chord_tolerance
    }

    /// Tessellate exactly one globally indexed source face.
    pub(super) fn tessellate(
        &mut self,
        global_face_index: usize,
        face: &FaceRecord,
    ) -> Result<FallbackMesh, String> {
        if self.face_count == 0 || global_face_index >= self.face_count {
            return Err(format!(
                "torus fallback face index {global_face_index} is outside {} source faces",
                self.face_count
            ));
        }
        let samples = self.encoded_edge_samples.get_or_insert_with(|| {
            brep_kernel::sample_edges_encoded(self.solid, self.chord_tolerance)
        });
        let samples = samples.as_ref().map_err(Clone::clone)?;
        let mut mesh = brep_kernel::tessellate_brep_watertight_face_stride_with_samples(
            self.solid,
            self.chord_tolerance,
            self.face_count,
            global_face_index,
            samples,
        )?;
        let expected_face_id = u32::try_from(global_face_index)
            .map_err(|_| "torus fallback face index exceeds u32::MAX".to_owned())?;
        if mesh.face_ids.len() != mesh.indices.len() / 3
            || mesh
                .face_ids
                .iter()
                .any(|&face_id| face_id != expected_face_id)
        {
            return Err("watertight face-stride fallback returned unrelated face ownership".into());
        }
        if !mesh.positions.len().is_multiple_of(3) {
            return Err(
                "watertight face-stride fallback returned a malformed position buffer".into(),
            );
        }
        let vertices = mesh.positions.len() / 3;
        let raw_scale = position_buffer_scale(&mesh.positions)?;
        let surface_projection_tolerance = self.chord_tolerance.min(
            self.distance_tolerance
                .max(self.relative_face_tolerance * raw_scale),
        );
        let max_surface_projection_distance =
            project_vertices_to_source_face(&mut mesh, face, surface_projection_tolerance)?;
        Ok(FallbackMesh {
            mesh,
            method: TessellationFallbackMethod::WatertightFaceStride,
            surface_projected_vertices: vertices,
            max_surface_projection_distance,
            surface_projection_tolerance,
        })
    }

    /// Recover one microscopic triangular plane trim from the retained source
    /// edge curves. The STEP importer may tolerance-weld all three topology
    /// vertices and pcurves to one point while deliberately retaining the raw
    /// edge curves. This path is intentionally narrower than a general polygon
    /// tessellator: it never invents an untrimmed carrier patch.
    pub(super) fn tessellate_projected_trim_curve_triangle(
        &self,
        global_face_index: usize,
        face: &FaceRecord,
    ) -> Result<FallbackMesh, String> {
        if self.face_count == 0 || global_face_index >= self.face_count {
            return Err(format!(
                "trim-curve fallback face index {global_face_index} is outside {} source faces",
                self.face_count
            ));
        }
        if !matches!(
            face.surface.analytic(),
            Some(KernelAnalyticSurface::Plane { .. })
        ) {
            return Err("trim-curve triangle fallback requires a planar source face".into());
        }
        let [loop_record] = face.loops.as_slice() else {
            return Err("trim-curve triangle fallback requires exactly one trim loop".into());
        };
        let [first, second, third] = loop_record.coedges.as_slice() else {
            return Err("trim-curve triangle fallback requires exactly three coedges".into());
        };
        let coedges = [first, second, third];
        let mut segments = Vec::with_capacity(3);
        for coedge in coedges {
            let edge = self
                .solid
                .edges
                .iter()
                .find(|edge| edge.id == coedge.edge_id)
                .ok_or_else(|| {
                    format!(
                        "trim-curve triangle fallback is missing source edge {}",
                        coedge.edge_id
                    )
                })?;
            if edge.curve.degree != 1 || edge.curve.control_points.len() != 2 {
                return Err(format!(
                    "trim-curve triangle fallback edge {} is not one straight NURBS span",
                    edge.id
                ));
            }
            let mut start = edge.curve.evaluate(edge.t0)?;
            let mut end = edge.curve.evaluate(edge.t1)?;
            if !coedge.forward {
                std::mem::swap(&mut start, &mut end);
            }
            if !finite_point(start) || !finite_point(end) {
                return Err("trim-curve triangle fallback edge endpoint was non-finite".into());
            }
            segments.push((start, end));
        }

        let coordinate_scale = segments
            .iter()
            .flat_map(|(start, end)| [start, end])
            .map(|point| point.x.abs().max(point.y.abs()).max(point.z.abs()))
            .fold(scalar::GEOMETRIC_SCALE_FLOOR, f64::max);
        let closure_tolerance = self
            .distance_tolerance
            .max(numerical::COORDINATE_ROUNDOFF_RELATIVE * coordinate_scale);
        for index in 0..3 {
            let gap = segments[index].1.sub(segments[(index + 1) % 3].0).length();
            if !gap.is_finite() || gap > closure_tolerance {
                return Err(format!(
                    "trim-curve triangle fallback boundary gap {gap:.6e} exceeded {closure_tolerance:.6e}"
                ));
            }
        }

        // Average the independently evaluated endpoints at each authored
        // corner. Exact LINE endpoints are normally identical; averaging only
        // absorbs a closure discrepancy already proven below the observation
        // tolerance and treats both incident source curves symmetrically.
        let corners = std::array::from_fn::<_, 3, _>(|index| {
            segments[index]
                .0
                .add(segments[(index + 2) % 3].1)
                .scale(0.5)
        });
        let mut mesh = Mesh {
            positions: corners
                .iter()
                .flat_map(|point| [point.x, point.y, point.z])
                .collect(),
            normals: vec![0.0; 9],
            indices: vec![0, 1, 2],
            face_ids: vec![u32::try_from(global_face_index)
                .map_err(|_| "trim-curve fallback face index exceeds u32::MAX")?],
        };
        let raw_scale = position_buffer_scale(&mesh.positions)?;
        let surface_projection_tolerance = self.chord_tolerance.min(
            self.distance_tolerance
                .max(self.relative_face_tolerance * raw_scale),
        );
        let max_surface_projection_distance =
            project_vertices_to_source_face(&mut mesh, face, surface_projection_tolerance)?;

        let point = |index: usize| {
            Vec3::new(
                mesh.positions[3 * index],
                mesh.positions[3 * index + 1],
                mesh.positions[3 * index + 2],
            )
        };
        let distinct_tolerance = numerical::COORDINATE_ROUNDOFF_RELATIVE * coordinate_scale;
        for (first, second) in [(0, 1), (1, 2), (2, 0)] {
            let separation = point(first).sub(point(second)).length();
            if !separation.is_finite() || separation <= distinct_tolerance {
                return Err(format!(
                    "trim-curve triangle corner separation {separation:.6e} did not exceed numerical resolution {distinct_tolerance:.6e}"
                ));
            }
        }
        let normal = Vec3::new(mesh.normals[0], mesh.normals[1], mesh.normals[2]);
        if point(1)
            .sub(point(0))
            .cross(point(2).sub(point(0)))
            .dot(normal)
            < 0.0
        {
            mesh.indices.swap(1, 2);
        }
        let observation = crate::brep::mesh_from_kernel(&mesh)
            .map_err(|error| format!("trim-curve triangle conversion failed: {error}"))?;
        observation
            .analyze(&crate::MeshAnalysisOptions::default())
            .map_err(|error| {
                format!("trim-curve triangle was not numerically observable: {error}")
            })?;

        Ok(FallbackMesh {
            mesh,
            method: TessellationFallbackMethod::ProjectedTrimCurveTriangle,
            surface_projected_vertices: 3,
            max_surface_projection_distance,
            surface_projection_tolerance,
        })
    }
}

fn finite_point(point: Vec3) -> bool {
    point.x.is_finite() && point.y.is_finite() && point.z.is_finite()
}

fn position_buffer_scale(positions: &[f64]) -> Result<f64, String> {
    if positions.is_empty() || !positions.len().is_multiple_of(3) {
        return Err(
            "observation fallback cannot measure a malformed or empty position buffer".into(),
        );
    }
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for point in positions.chunks_exact(3) {
        if point.iter().any(|value| !value.is_finite()) {
            return Err("observation fallback position buffer was non-finite".into());
        }
        low.x = low.x.min(point[0]);
        low.y = low.y.min(point[1]);
        low.z = low.z.min(point[2]);
        high.x = high.x.max(point[0]);
        high.y = high.y.max(point[1]);
        high.z = high.z.max(point[2]);
    }
    Ok(high.sub(low).length().max(scalar::GEOMETRIC_SCALE_FLOOR))
}

fn project_vertices_to_source_face(
    mesh: &mut Mesh,
    face: &FaceRecord,
    tolerance: f64,
) -> Result<f64, String> {
    if !tolerance.is_finite() || tolerance <= 0.0 {
        return Err(
            "observation fallback surface projection tolerance must be finite and positive".into(),
        );
    }
    let vertices = mesh.positions.len() / 3;
    let mut projected = Vec::with_capacity(vertices);
    let mut max_distance = 0.0f64;
    for point in mesh.positions.chunks_exact(3) {
        let source = Vec3::new(point[0], point[1], point[2]);
        let projection = brep_kernel::project_point_to_surface(&face.surface, source)?;
        if !projection.distance.is_finite() {
            return Err("observation fallback surface projection was non-finite".into());
        }
        max_distance = max_distance.max(projection.distance);
        let mut normal = face.surface.normal(projection.u, projection.v)?;
        if !face.same_sense {
            normal = normal.scale(-1.0);
        }
        projected.push((projection.point, normal));
    }
    if max_distance > tolerance {
        return Err(format!(
            "source-face projection correction {max_distance:.6e} exceeded observation tolerance {tolerance:.6e}"
        ));
    }
    mesh.normals.resize(mesh.positions.len(), 0.0);
    for (index, (point, normal)) in projected.into_iter().enumerate() {
        mesh.positions[3 * index] = point.x;
        mesh.positions[3 * index + 1] = point.y;
        mesh.positions[3 * index + 2] = point.z;
        mesh.normals[3 * index] = normal.x;
        mesh.normals[3 * index + 1] = normal.y;
        mesh.normals[3 * index + 2] = normal.z;
    }
    Ok(max_distance)
}

