//! Lightweight, dependency-free geometry export for recognition diagnostics.

use crate::{Mesh, RecognitionError, RecognitionResult};
use std::fmt::Write;

/// In-memory Wavefront OBJ geometry and its companion material library.
///
/// Save [`obj`](Self::obj) as `analytic_regions.obj` and
/// [`mtl`](Self::mtl) as `analytic_regions.mtl` in the same directory. The OBJ
/// contains every input triangle exactly once, partitioned into one group per
/// recognized region plus `unresolved` and `unassigned` groups when needed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DebugObjExport {
    /// Wavefront OBJ geometry and grouping text.
    pub obj: String,
    /// Companion Wavefront material-library text.
    pub mtl: String,
}

const COLORS: &[[f64; 3]] = &[
    [0.894, 0.102, 0.110],
    [0.216, 0.494, 0.722],
    [0.302, 0.686, 0.290],
    [0.596, 0.306, 0.639],
    [1.000, 0.498, 0.000],
    [1.000, 1.000, 0.200],
    [0.651, 0.337, 0.157],
    [0.969, 0.506, 0.749],
];

/// Export an input mesh and recognition result as Wavefront OBJ/MTL text.
///
/// Region colors are deterministic and intended only for debugging. OBJ groups
/// retain primitive type and region number; unresolved triangles are gray and
/// triangles absent from the result are black. Invalid indices or overlapping
/// result partitions are rejected rather than producing misleading geometry.
pub fn export_debug_obj(
    mesh: &Mesh,
    result: &RecognitionResult,
) -> Result<DebugObjExport, RecognitionError> {
    validate_mesh_indices(mesh)?;

    // `None` means not mentioned by the recognition result, `Some(n)` is a
    // recognized region, and `Some(region_count)` is the unresolved partition.
    let mut owner = vec![None; mesh.triangles.len()];
    for (region_index, region) in result.regions.iter().enumerate() {
        for &triangle in &region.triangle_indices {
            claim(&mut owner, triangle, region_index, "recognized region")?;
        }
    }
    let unresolved_owner = result.regions.len();
    for &triangle in &result.unresolved_triangles {
        claim(
            &mut owner,
            triangle,
            unresolved_owner,
            "unresolved partition",
        )?;
    }

    let mut obj = String::new();
    obj.push_str("# cadmesh-analytic recognition debug export\n");
    obj.push_str("mtllib analytic_regions.mtl\n");
    obj.push_str("o analytic_recognition_debug\n");
    for vertex in &mesh.vertices {
        writeln!(
            &mut obj,
            "v {:.17} {:.17} {:.17}",
            vertex.x, vertex.y, vertex.z
        )
        .unwrap();
    }

    let mut mtl = String::from("# cadmesh-analytic deterministic debug palette\n");
    for (region_index, region) in result.regions.iter().enumerate() {
        let material = format!("region_{region_index:03}");
        let group = format!(
            "region_{region_index:03}_{}",
            region.surface.surface_type().name()
        );
        let color = COLORS[region_index % COLORS.len()];
        write_material(&mut mtl, &material, color);
        writeln!(&mut obj, "\ng {group}\nusemtl {material}").unwrap();
        writeln!(
            &mut obj,
            "# orientation={} triangles={} rms_error={:.17} max_error={:.17}",
            region.orientation,
            region.triangle_indices.len(),
            region.metrics.rms_error,
            region.metrics.max_error
        )
        .unwrap();
        write_faces(&mut obj, mesh, &region.triangle_indices);
    }

    if !result.unresolved_triangles.is_empty() {
        write_material(&mut mtl, "unresolved", [0.55, 0.55, 0.55]);
        obj.push_str("\ng unresolved\nusemtl unresolved\n");
        write_faces(&mut obj, mesh, &result.unresolved_triangles);
    }

    let unassigned: Vec<_> = owner
        .iter()
        .enumerate()
        .filter_map(|(triangle, assigned)| assigned.is_none().then_some(triangle))
        .collect();
    if !unassigned.is_empty() {
        write_material(&mut mtl, "unassigned", [0.08, 0.08, 0.08]);
        obj.push_str("\ng unassigned\nusemtl unassigned\n");
        write_faces(&mut obj, mesh, &unassigned);
    }

    Ok(DebugObjExport { obj, mtl })
}

fn validate_mesh_indices(mesh: &Mesh) -> Result<(), RecognitionError> {
    if mesh.vertices.iter().any(|vertex| !vertex.is_finite()) {
        return Err(RecognitionError::InvalidMesh(
            "debug export requires finite vertex coordinates".into(),
        ));
    }
    for (triangle_index, triangle) in mesh.triangles.iter().enumerate() {
        for &vertex in triangle {
            if vertex as usize >= mesh.vertices.len() {
                return Err(RecognitionError::InvalidMesh(format!(
                    "triangle {triangle_index} references missing vertex {vertex}"
                )));
            }
        }
    }
    Ok(())
}

fn claim(
    owner: &mut [Option<usize>],
    triangle: usize,
    claimant: usize,
    partition: &str,
) -> Result<(), RecognitionError> {
    let slot = owner.get_mut(triangle).ok_or_else(|| {
        RecognitionError::InvalidSelection(format!(
            "{partition} references missing triangle {triangle}"
        ))
    })?;
    if let Some(previous) = *slot {
        return Err(RecognitionError::InvalidSelection(format!(
            "triangle {triangle} is assigned more than once (owners {previous} and {claimant})"
        )));
    }
    *slot = Some(claimant);
    Ok(())
}

fn write_faces(out: &mut String, mesh: &Mesh, triangle_indices: &[usize]) {
    for &triangle_index in triangle_indices {
        let triangle = mesh.triangles[triangle_index];
        writeln!(
            out,
            "f {} {} {}",
            triangle[0] + 1,
            triangle[1] + 1,
            triangle[2] + 1
        )
        .unwrap();
    }
}

fn write_material(out: &mut String, name: &str, color: [f64; 3]) {
    writeln!(
        out,
        "\nnewmtl {name}\nKd {:.3} {:.3} {:.3}\nKa 0.000 0.000 0.000\nd 1.000",
        color[0], color[1], color[2]
    )
    .unwrap();
}

