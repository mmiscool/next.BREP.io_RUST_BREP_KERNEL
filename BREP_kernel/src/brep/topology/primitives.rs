use crate::curve::parameter_line;
use super::*;

pub fn make_box_brep(
    corner: Vec3,
    size_x: f64,
    size_y: f64,
    size_z: f64,
) -> Result<BrepSolid, String> {
    if size_x <= 0.0 || size_y <= 0.0 || size_z <= 0.0 {
        return Err("makeBoxSolid: sizes must be positive".into());
    }
    let mut vertices = Vec::with_capacity(8);
    for index in 0..8_u64 {
        vertices.push(VertexRecord {
            id: index + 1,
            point: corner.add(Vec3::new(
                if index & 1 != 0 { size_x } else { 0.0 },
                if index & 2 != 0 { size_y } else { 0.0 },
                if index & 4 != 0 { size_z } else { 0.0 },
            )),
        });
    }
    let specifications = [
        [0_usize, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];
    let mut edges = Vec::<EdgeRecord>::new();
    let mut edge_indices = HashMap::<(usize, usize), usize>::default();
    let mut faces = Vec::with_capacity(6);
    let mut next_topology_id = 100_u64;

    for corners in specifications {
        let p0 = vertices[corners[0]].point;
        let p1 = vertices[corners[1]].point;
        let p3 = vertices[corners[3]].point;
        let u_vector = p1.sub(p0);
        let v_vector = p3.sub(p0);
        let u_length = u_vector.length();
        let v_length = v_vector.length();
        let u_direction = u_vector.normalized()?;
        let v_direction = v_vector.normalized()?;
        let p11 = p0
            .add(u_direction.scale(u_length))
            .add(v_direction.scale(v_length));
        let surface = NurbsSurface::new(
            1,
            1,
            vec![0.0, 0.0, u_length, u_length],
            vec![0.0, 0.0, v_length, v_length],
            vec![
                vec![Vec4::from_point(p0, 1.0), Vec4::from_point(p3, 1.0)],
                vec![Vec4::from_point(p1, 1.0), Vec4::from_point(p11, 1.0)],
            ],
        )?;
        let uv = [
            [0.0, 0.0],
            [u_length, 0.0],
            [u_length, v_length],
            [0.0, v_length],
        ];
        let mut coedges = Vec::with_capacity(4);
        for side in 0..4 {
            let start_index = corners[side];
            let end_index = corners[(side + 1) % 4];
            let key = if start_index < end_index {
                (start_index, end_index)
            } else {
                (end_index, start_index)
            };
            let edge_index = match edge_indices.get(&key) {
                Some(index) => *index,
                None => {
                    let id = 10 + edges.len() as u64;
                    let start = vertices[key.0].point;
                    let end = vertices[key.1].point;
                    edges.push(EdgeRecord {
                        id,
                        curve: make_line(start, end)?,
                        t0: 0.0,
                        t1: 1.0,
                        start_vertex_id: vertices[key.0].id,
                        end_vertex_id: vertices[key.1].id,
                        degenerate: false,
                        name: None,
                    });
                    let index = edges.len() - 1;
                    edge_indices.insert(key, index);
                    index
                }
            };
            let edge = &edges[edge_index];
            coedges.push(CoedgeRecord {
                id: next_topology_id,
                edge_id: edge.id,
                forward: edge.start_vertex_id == vertices[start_index].id,
                pcurve: make_line(
                    Vec3::new(uv[side][0], uv[side][1], 0.0),
                    Vec3::new(uv[(side + 1) % 4][0], uv[(side + 1) % 4][1], 0.0),
                )?,
            });
            next_topology_id += 1;
        }
        let loop_record = LoopRecord {
            id: next_topology_id,
            coedges,
        };
        next_topology_id += 1;
        faces.push(FaceRecord {
            id: next_topology_id,
            surface,
            same_sense: true,
            loops: vec![loop_record],
            name: None,
        });
        next_topology_id += 1;
    }
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: next_topology_id + 1,
        vertices,
        edges,
        shells: vec![ShellRecord {
            id: next_topology_id,
            faces,
        }],
        genus: 0,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!(
            "Rust box builder produced invalid topology: {:?}",
            issues
        ));
    }
    Ok(solid)
}

pub fn make_pyramid_brep(
    corner: Vec3,
    side_length: f64,
    height: f64,
    sides: usize,
) -> Result<BrepSolid, String> {
    if side_length <= 0.0 || height <= 0.0 {
        return Err("makePyramidSolid: sizes must be positive".into());
    }
    let side_count = sides.max(3);
    let center = corner.add(Vec3::new(side_length / 2.0, side_length / 2.0, 0.0));
    let radius = side_length / (2.0 * (std::f64::consts::PI / side_count as f64).sin());
    let start_angle = -std::f64::consts::FRAC_PI_2 - std::f64::consts::PI / side_count as f64;
    let mut vertices = Vec::with_capacity(side_count + 1);
    for index in 0..side_count {
        let angle = start_angle + index as f64 * std::f64::consts::TAU / side_count as f64;
        vertices.push(VertexRecord {
            id: index as u64 + 1,
            point: center.add(Vec3::new(radius * angle.cos(), radius * angle.sin(), 0.0)),
        });
    }
    let apex_index = vertices.len();
    vertices.push(VertexRecord {
        id: apex_index as u64 + 1,
        point: center.add(Vec3::new(0.0, 0.0, height)),
    });

    let mut edges = Vec::<EdgeRecord>::new();
    let mut edge_indices = HashMap::<(usize, usize), usize>::default();
    let mut faces = Vec::with_capacity(side_count + 1);
    let mut next_topology_id = 100u64;
    let mut make_face = |corners: &[usize]| -> Result<FaceRecord, String> {
        if corners.len() < 3 {
            return Err("makePyramidSolid: face needs at least three corners".into());
        }
        let p0 = vertices[corners[0]].point;
        let p1 = vertices[corners[1]].point;
        let p2 = vertices[corners[2]].point;
        let x_axis = p1.sub(p0).normalized()?;
        let normal = p1.sub(p0).cross(p2.sub(p0)).normalized()?;
        let y_axis = normal.cross(x_axis).normalized()?;
        let mut minimum_u = f64::INFINITY;
        let mut minimum_v = f64::INFINITY;
        let mut maximum_u = f64::NEG_INFINITY;
        let mut maximum_v = f64::NEG_INFINITY;
        let projected = corners
            .iter()
            .map(|&index| {
                let delta = vertices[index].point.sub(p0);
                let u = delta.dot(x_axis);
                let v = delta.dot(y_axis);
                minimum_u = minimum_u.min(u);
                minimum_v = minimum_v.min(v);
                maximum_u = maximum_u.max(u);
                maximum_v = maximum_v.max(v);
                (u, v)
            })
            .collect::<Vec<_>>();
        let origin = p0.add(x_axis.scale(minimum_u)).add(y_axis.scale(minimum_v));
        let surface = make_plane(
            origin,
            x_axis,
            y_axis,
            maximum_u - minimum_u,
            maximum_v - minimum_v,
        )?;
        let mut coedges = Vec::with_capacity(corners.len());
        for index in 0..corners.len() {
            let start_index = corners[index];
            let end_index = corners[(index + 1) % corners.len()];
            let key = if start_index < end_index {
                (start_index, end_index)
            } else {
                (end_index, start_index)
            };
            let edge_index = if let Some(edge_index) = edge_indices.get(&key) {
                *edge_index
            } else {
                let edge_index = edges.len();
                edges.push(EdgeRecord {
                    id: 10 + edge_index as u64,
                    curve: make_line(vertices[key.0].point, vertices[key.1].point)?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: vertices[key.0].id,
                    end_vertex_id: vertices[key.1].id,
                    degenerate: false,
                    name: None,
                });
                edge_indices.insert(key, edge_index);
                edge_index
            };
            let edge = &edges[edge_index];
            let start_uv = projected[index];
            let end_uv = projected[(index + 1) % corners.len()];
            coedges.push(CoedgeRecord {
                id: next_topology_id,
                edge_id: edge.id,
                forward: edge.start_vertex_id == vertices[start_index].id,
                pcurve: parameter_line(
                    start_uv.0 - minimum_u,
                    start_uv.1 - minimum_v,
                    end_uv.0 - minimum_u,
                    end_uv.1 - minimum_v,
                )?,
            });
            next_topology_id += 1;
        }
        let loop_record = LoopRecord {
            id: next_topology_id,
            coedges,
        };
        next_topology_id += 1;
        let face = FaceRecord {
            id: next_topology_id,
            surface,
            same_sense: true,
            loops: vec![loop_record],
            name: None,
        };
        next_topology_id += 1;
        Ok(face)
    };

    let base = (0..side_count).rev().collect::<Vec<_>>();
    faces.push(make_face(&base)?);
    for index in 0..side_count {
        faces.push(make_face(&[index, (index + 1) % side_count, apex_index])?);
    }
    drop(make_face);
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: next_topology_id + 1,
        vertices,
        edges,
        shells: vec![ShellRecord {
            id: next_topology_id,
            faces,
        }],
        genus: 0,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!(
            "Rust pyramid builder produced invalid topology: {issues:?}"
        ));
    }
    Ok(solid)
}

fn curve_to_plane_parameters(
    curve: &NurbsCurve,
    origin: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
) -> Result<NurbsCurve, String> {
    let mut points = Vec::with_capacity(curve.control_points.len());
    for control_point in &curve.control_points {
        let point = control_point.point()?;
        let delta = point.sub(origin);
        points.push(Vec4 {
            x: delta.dot(x_axis) * control_point.w,
            y: delta.dot(y_axis) * control_point.w,
            z: 0.0,
            w: control_point.w,
        });
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), points)
}

pub fn make_cylinder_brep(
    base: Vec3,
    axis_direction: Vec3,
    radius: f64,
    height: f64,
) -> Result<BrepSolid, String> {
    if radius <= 0.0 || height <= 0.0 {
        return Err("makeCylinderSolid: radius and height must be positive".into());
    }
    let axis = axis_direction.normalized()?;
    let side = make_cylinder_surface(base, axis, radius, height)?;
    let bottom_point = side.evaluate(0.0, 0.0)?;
    let top_point = side.evaluate(0.0, 1.0)?;
    let vertices = vec![
        VertexRecord {
            id: 1,
            point: bottom_point,
        },
        VertexRecord {
            id: 2,
            point: top_point,
        },
    ];
    let bottom_circle = side.iso_curve_v(0.0)?;
    let top_circle = side.iso_curve_v(1.0)?;
    let seam_line = side.iso_curve_u(0.0)?;
    let edges = vec![
        EdgeRecord {
            id: 10,
            curve: bottom_circle.clone(),
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: 1,
            end_vertex_id: 1,
            degenerate: false,
            name: None,
        },
        EdgeRecord {
            id: 11,
            curve: top_circle.clone(),
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: 2,
            end_vertex_id: 2,
            degenerate: false,
            name: None,
        },
        EdgeRecord {
            id: 12,
            curve: seam_line,
            t0: 0.0,
            t1: 1.0,
            start_vertex_id: 1,
            end_vertex_id: 2,
            degenerate: false,
            name: None,
        },
    ];

    let side_loop = LoopRecord {
        id: 100,
        coedges: vec![
            CoedgeRecord {
                id: 101,
                edge_id: 10,
                forward: true,
                pcurve: parameter_line(0.0, 0.0, 1.0, 0.0)?,
            },
            CoedgeRecord {
                id: 102,
                edge_id: 12,
                forward: true,
                pcurve: parameter_line(1.0, 0.0, 1.0, 1.0)?,
            },
            CoedgeRecord {
                id: 103,
                edge_id: 11,
                forward: false,
                pcurve: parameter_line(1.0, 1.0, 0.0, 1.0)?,
            },
            CoedgeRecord {
                id: 104,
                edge_id: 12,
                forward: false,
                pcurve: parameter_line(0.0, 1.0, 0.0, 0.0)?,
            },
        ],
    };
    let side_face = FaceRecord {
        id: 105,
        surface: side,
        same_sense: true,
        loops: vec![side_loop],
        name: None,
    };

    let x_axis = axis.perpendicular()?;
    let y_axis = axis.cross(x_axis).normalized()?;
    let bottom_origin = base.add(x_axis.scale(-radius)).add(y_axis.scale(-radius));
    let bottom_plane = make_plane(bottom_origin, x_axis, y_axis, 2.0 * radius, 2.0 * radius)?;
    let bottom_pcurve =
        curve_to_plane_parameters(&bottom_circle, bottom_origin, x_axis, y_axis)?.reversed()?;
    let bottom_face = FaceRecord {
        id: 108,
        surface: bottom_plane,
        same_sense: false,
        loops: vec![LoopRecord {
            id: 107,
            coedges: vec![CoedgeRecord {
                id: 106,
                edge_id: 10,
                forward: false,
                pcurve: bottom_pcurve,
            }],
        }],
        name: None,
    };

    let top_origin = base
        .add(axis.scale(height))
        .add(x_axis.scale(-radius))
        .add(y_axis.scale(-radius));
    let top_plane = make_plane(top_origin, x_axis, y_axis, 2.0 * radius, 2.0 * radius)?;
    let top_pcurve = curve_to_plane_parameters(&top_circle, top_origin, x_axis, y_axis)?;
    let top_face = FaceRecord {
        id: 111,
        surface: top_plane,
        same_sense: true,
        loops: vec![LoopRecord {
            id: 110,
            coedges: vec![CoedgeRecord {
                id: 109,
                edge_id: 11,
                forward: true,
                pcurve: top_pcurve,
            }],
        }],
        name: None,
    };

    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        id: 114,
        vertices,
        edges,
        shells: vec![ShellRecord {
            id: 113,
            faces: vec![side_face, bottom_face, top_face],
        }],
        genus: 0,
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(format!(
            "Rust cylinder builder produced invalid topology: {:?}",
            issues
        ));
    }
    Ok(solid)
}
