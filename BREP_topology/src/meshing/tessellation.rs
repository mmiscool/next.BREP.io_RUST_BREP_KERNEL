use crate::curve::interior_knots;
use crate::mass_properties::trim_polygons;
use crate::topology::{BrepSolid, FaceRecord};
use crate::{KnotVector, Mesh, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct TessellationOptions {
    pub slabs_per_span_u: usize,
    pub steps_per_span_v: usize,
}

impl Default for TessellationOptions {
    fn default() -> Self {
        Self {
            slabs_per_span_u: 8,
            steps_per_span_v: 8,
        }
    }
}

fn crossings_at(polygons: &[Vec<[f64; 2]>], u: f64) -> Vec<f64> {
    let mut crossings = Vec::new();
    for polygon in polygons {
        for index in 0..polygon.len() {
            let a = polygon[index];
            let b = polygon[(index + 1) % polygon.len()];
            if (a[0] > u) != (b[0] > u) {
                crossings.push(a[1] + (u - a[0]) / (b[0] - a[0]) * (b[1] - a[1]));
            }
        }
    }
    crossings.sort_by(f64::total_cmp);
    crossings
}

fn emit_vertex(
    mesh: &mut Mesh,
    face: &FaceRecord,
    u: f64,
    v: f64,
    domains: [f64; 4],
) -> Result<u32, String> {
    let (point, su, sv) = face.surface.deriv1(u, v)?;
    let mut normal = su.cross(sv);
    if normal.length() <= 1e-12 {
        let [u0, u1, v0, v1] = domains;
        let epsilon = 1e-4;
        let nudged_u = u.clamp(u0 + epsilon * (u1 - u0), u1 - epsilon * (u1 - u0));
        let nudged_v = v.clamp(v0 + epsilon * (v1 - v0), v1 - epsilon * (v1 - v0));
        let (_, nsu, nsv) = face.surface.deriv1(nudged_u, nudged_v)?;
        normal = nsu.cross(nsv);
        if normal.length() <= 1e-12 {
            normal = Vec3::new(0.0, 0.0, 1.0);
        }
    }
    normal = normal.normalized()?;
    if !face.same_sense {
        normal = normal.scale(-1.0);
    }
    mesh.positions.extend([point.x, point.y, point.z]);
    mesh.normals.extend([normal.x, normal.y, normal.z]);
    Ok((mesh.positions.len() / 3 - 1) as u32)
}

fn push_triangle(mesh: &mut Mesh, a: u32, b: u32, c: u32, ccw: bool, face_id: u32) {
    if ccw {
        mesh.indices.extend([a, b, c]);
    } else {
        mesh.indices.extend([a, c, b]);
    }
    mesh.face_ids.push(face_id);
}

pub fn tessellate_face(
    face: &FaceRecord,
    options: TessellationOptions,
    face_id: u32,
) -> Result<Mesh, String> {
    let ku = KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let polygons = trim_polygons(face)?;
    let slabs = options.slabs_per_span_u.max(1);
    let steps = options.steps_per_span_v.max(1);

    let mut u_knots = vec![u0];
    u_knots.extend(interior_knots(&face.surface.knots_u, face.surface.degree_u));
    u_knots.push(u1);
    let mut breaks = Vec::new();
    for pair in u_knots.windows(2) {
        for index in 0..=slabs {
            breaks.push(pair[0] + (pair[1] - pair[0]) * index as f64 / slabs as f64);
        }
    }
    for polygon in &polygons {
        breaks.extend(polygon.iter().map(|point| point[0].clamp(u0, u1)));
    }
    breaks.sort_by(f64::total_cmp);
    let minimum_gap = (u1 - u0) * 1e-9;
    breaks.dedup_by(|a, b| (*a - *b).abs() <= minimum_gap);

    let v_span_length =
        (v1 - v0) / (interior_knots(&face.surface.knots_v, face.surface.degree_v).len() + 1) as f64;
    let v_step = v_span_length / steps as f64;
    let mut mesh = Mesh::default();
    for pair in breaks.windows(2) {
        let ua = pair[0];
        let ub = pair[1];
        if ub - ua <= minimum_gap {
            continue;
        }
        let midpoint_crossings = crossings_at(&polygons, (ua + ub) * 0.5);
        if midpoint_crossings.len() < 2 {
            continue;
        }
        let mut left_crossings = crossings_at(&polygons, ua + (ub - ua) * 1e-7);
        let mut right_crossings = crossings_at(&polygons, ub - (ub - ua) * 1e-7);
        if left_crossings.len() != midpoint_crossings.len()
            || right_crossings.len() != midpoint_crossings.len()
        {
            left_crossings.clone_from(&midpoint_crossings);
            right_crossings.clone_from(&midpoint_crossings);
        }
        for interval in (0..midpoint_crossings.len() - 1).step_by(2) {
            let lower_left = left_crossings[interval].clamp(v0, v1);
            let upper_left = left_crossings[interval + 1].clamp(v0, v1);
            let lower_right = right_crossings[interval].clamp(v0, v1);
            let upper_right = right_crossings[interval + 1].clamp(v0, v1);
            let span = (upper_left - lower_left).max(upper_right - lower_right);
            if span <= 1e-12 {
                continue;
            }
            let row_steps = ((span / v_step).ceil() as usize).clamp(1, 64);
            let mut left = Vec::with_capacity(row_steps + 1);
            let mut right = Vec::with_capacity(row_steps + 1);
            for index in 0..=row_steps {
                let fraction = index as f64 / row_steps as f64;
                left.push(emit_vertex(
                    &mut mesh,
                    face,
                    ua,
                    lower_left + (upper_left - lower_left) * fraction,
                    [u0, u1, v0, v1],
                )?);
                right.push(emit_vertex(
                    &mut mesh,
                    face,
                    ub,
                    lower_right + (upper_right - lower_right) * fraction,
                    [u0, u1, v0, v1],
                )?);
            }
            for index in 0..row_steps {
                push_triangle(
                    &mut mesh,
                    left[index],
                    right[index],
                    right[index + 1],
                    face.same_sense,
                    face_id,
                );
                push_triangle(
                    &mut mesh,
                    left[index],
                    right[index + 1],
                    left[index + 1],
                    face.same_sense,
                    face_id,
                );
            }
        }
    }
    mesh.validate()?;
    Ok(mesh)
}

pub fn tessellate_brep(solid: &BrepSolid, options: TessellationOptions) -> Result<Mesh, String> {
    let mut mesh = Mesh::default();
    let mut sequential_face_id = 0u32;
    for shell in &solid.shells {
        for face in &shell.faces {
            let face_mesh = tessellate_face(face, options, sequential_face_id)?;
            let base = (mesh.positions.len() / 3) as u32;
            mesh.positions.extend(face_mesh.positions);
            mesh.normals.extend(face_mesh.normals);
            mesh.indices
                .extend(face_mesh.indices.into_iter().map(|index| index + base));
            mesh.face_ids.extend(face_mesh.face_ids);
            sequential_face_id += 1;
        }
    }
    mesh.validate()?;
    Ok(mesh)
}

