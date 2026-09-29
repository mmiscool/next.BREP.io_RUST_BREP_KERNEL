use crate::curve::{curve_to_plane_parameters, parameter_line};
use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{
    make_cone_surface, make_line, make_plane, make_sphere_surface_framed, make_torus_surface,
    Vec3,
};

fn validated(solid: BrepSolid, label: &str) -> Result<BrepSolid, String> {
    let issues = solid.validate();
    if issues.is_empty() {
        Ok(solid)
    } else {
        Err(format!(
            "Rust {label} builder produced invalid topology: {issues:?}"
        ))
    }
}

pub fn make_sphere_brep(center: Vec3, radius: f64, polar_axis: Vec3) -> Result<BrepSolid, String> {
    make_sphere_brep_framed(center, radius, polar_axis, None)
}

/// A sphere solid whose SEAM direction is chosen as well as its polar axis — see
/// [`make_sphere_surface_framed`]. The topology is identical either way (one
/// face, one pole-to-pole seam edge, two degenerate pole edges); only WHERE on
/// the ball the seam and the poles sit changes. A caller that will trim the ball
/// uses it to hide both inside the removed region, so the surviving face has no
/// edge bordering itself.
pub fn make_sphere_brep_framed(
    center: Vec3,
    radius: f64,
    polar_axis: Vec3,
    seam_direction: Option<Vec3>,
) -> Result<BrepSolid, String> {
    let surface = make_sphere_surface_framed(center, radius, polar_axis, seam_direction)?;
    let south = surface.evaluate(0.0, 0.0)?;
    let north = surface.evaluate(0.0, 1.0)?;
    let seam = surface.iso_curve_u(0.0)?;
    validated(
        BrepSolid {
            mass_properties_cache: Default::default(),
            id: 120,
            vertices: vec![
                VertexRecord {
                    id: 1,
                    point: south,
                },
                VertexRecord {
                    id: 2,
                    point: north,
                },
            ],
            edges: vec![
                EdgeRecord {
                    id: 10,
                    curve: seam,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 1,
                    end_vertex_id: 2,
                    degenerate: false,
                    name: None,
                },
                EdgeRecord {
                    id: 11,
                    curve: make_line(south, south)?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 1,
                    end_vertex_id: 1,
                    degenerate: true,
                    name: None,
                },
                EdgeRecord {
                    id: 12,
                    curve: make_line(north, north)?,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 2,
                    end_vertex_id: 2,
                    degenerate: true,
                    name: None,
                },
            ],
            shells: vec![ShellRecord {
                id: 119,
                faces: vec![FaceRecord {
                    id: 118,
                    surface,
                    same_sense: true,
                    loops: vec![LoopRecord {
                        id: 117,
                        coedges: vec![
                            CoedgeRecord {
                                id: 101,
                                edge_id: 11,
                                forward: true,
                                pcurve: parameter_line(0.0, 0.0, 1.0, 0.0)?,
                            },
                            CoedgeRecord {
                                id: 102,
                                edge_id: 10,
                                forward: true,
                                pcurve: parameter_line(1.0, 0.0, 1.0, 1.0)?,
                            },
                            CoedgeRecord {
                                id: 103,
                                edge_id: 12,
                                forward: true,
                                pcurve: parameter_line(1.0, 1.0, 0.0, 1.0)?,
                            },
                            CoedgeRecord {
                                id: 104,
                                edge_id: 10,
                                forward: false,
                                pcurve: parameter_line(0.0, 1.0, 0.0, 0.0)?,
                            },
                        ],
                    }],
                    name: None,
                }],
            }],
            genus: 0,
        },
        "sphere",
    )
}

pub fn make_torus_brep(
    center: Vec3,
    axis_direction: Vec3,
    major_radius: f64,
    minor_radius: f64,
) -> Result<BrepSolid, String> {
    let surface = make_torus_surface(center, axis_direction, major_radius, minor_radius)?;
    let corner = surface.evaluate(0.0, 0.0)?;
    let major_curve = surface.iso_curve_v(0.0)?;
    let tube_curve = surface.iso_curve_u(0.0)?;
    validated(
        BrepSolid {
            mass_properties_cache: Default::default(),
            id: 130,
            vertices: vec![VertexRecord {
                id: 1,
                point: corner,
            }],
            edges: vec![
                EdgeRecord {
                    id: 10,
                    curve: major_curve,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 1,
                    end_vertex_id: 1,
                    degenerate: false,
                    name: None,
                },
                EdgeRecord {
                    id: 11,
                    curve: tube_curve,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 1,
                    end_vertex_id: 1,
                    degenerate: false,
                    name: None,
                },
            ],
            shells: vec![ShellRecord {
                id: 129,
                faces: vec![FaceRecord {
                    id: 128,
                    surface,
                    same_sense: true,
                    loops: vec![LoopRecord {
                        id: 127,
                        coedges: vec![
                            CoedgeRecord {
                                id: 101,
                                edge_id: 10,
                                forward: true,
                                pcurve: parameter_line(0.0, 0.0, 1.0, 0.0)?,
                            },
                            CoedgeRecord {
                                id: 102,
                                edge_id: 11,
                                forward: true,
                                pcurve: parameter_line(1.0, 0.0, 1.0, 1.0)?,
                            },
                            CoedgeRecord {
                                id: 103,
                                edge_id: 10,
                                forward: false,
                                pcurve: parameter_line(1.0, 1.0, 0.0, 1.0)?,
                            },
                            CoedgeRecord {
                                id: 104,
                                edge_id: 11,
                                forward: false,
                                pcurve: parameter_line(0.0, 1.0, 0.0, 0.0)?,
                            },
                        ],
                    }],
                    name: None,
                }],
            }],
            genus: 1,
        },
        "torus",
    )
}

pub fn make_cone_brep(
    base: Vec3,
    axis_direction: Vec3,
    radius_bottom: f64,
    radius_top: f64,
    height: f64,
) -> Result<BrepSolid, String> {
    if radius_bottom <= 0.0 || radius_top < 0.0 || height <= 0.0 {
        return Err("makeConeSolid: invalid radius or height".into());
    }
    let axis = axis_direction.normalized()?;
    let side = make_cone_surface(base, axis, radius_bottom, radius_top, height)?;
    let bottom_point = side.evaluate(0.0, 0.0)?;
    let bottom_circle = side.iso_curve_v(0.0)?;
    let seam = side.iso_curve_u(0.0)?;
    let x_axis = axis.perpendicular()?;
    let y_axis = axis.cross(x_axis).normalized()?;
    let bottom_origin = base
        .add(x_axis.scale(-radius_bottom))
        .add(y_axis.scale(-radius_bottom));
    let bottom_plane = make_plane(
        bottom_origin,
        x_axis,
        y_axis,
        2.0 * radius_bottom,
        2.0 * radius_bottom,
    )?;
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

    if radius_top <= 1e-7 {
        let apex = side.evaluate(0.0, 1.0)?;
        return validated(
            BrepSolid {
                mass_properties_cache: Default::default(),
                id: 140,
                vertices: vec![
                    VertexRecord {
                        id: 1,
                        point: bottom_point,
                    },
                    VertexRecord { id: 2, point: apex },
                ],
                edges: vec![
                    EdgeRecord {
                        id: 10,
                        curve: bottom_circle,
                        t0: 0.0,
                        t1: 1.0,
                        start_vertex_id: 1,
                        end_vertex_id: 1,
                        degenerate: false,
                        name: None,
                    },
                    EdgeRecord {
                        id: 11,
                        curve: seam,
                        t0: 0.0,
                        t1: 1.0,
                        start_vertex_id: 1,
                        end_vertex_id: 2,
                        degenerate: false,
                        name: None,
                    },
                    EdgeRecord {
                        id: 12,
                        curve: make_line(apex, apex)?,
                        t0: 0.0,
                        t1: 1.0,
                        start_vertex_id: 2,
                        end_vertex_id: 2,
                        degenerate: true,
                        name: None,
                    },
                ],
                shells: vec![ShellRecord {
                    id: 139,
                    faces: vec![
                        FaceRecord {
                            id: 105,
                            surface: side,
                            same_sense: true,
                            loops: vec![LoopRecord {
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
                                        edge_id: 11,
                                        forward: true,
                                        pcurve: parameter_line(1.0, 0.0, 1.0, 1.0)?,
                                    },
                                    CoedgeRecord {
                                        id: 103,
                                        edge_id: 12,
                                        forward: true,
                                        pcurve: parameter_line(1.0, 1.0, 0.0, 1.0)?,
                                    },
                                    CoedgeRecord {
                                        id: 104,
                                        edge_id: 11,
                                        forward: false,
                                        pcurve: parameter_line(0.0, 1.0, 0.0, 0.0)?,
                                    },
                                ],
                            }],
                            name: None,
                        },
                        bottom_face,
                    ],
                }],
                genus: 0,
            },
            "cone",
        );
    }

    let top_point = side.evaluate(0.0, 1.0)?;
    let top_circle = side.iso_curve_v(1.0)?;
    let top_origin = base
        .add(axis.scale(height))
        .add(x_axis.scale(-radius_top))
        .add(y_axis.scale(-radius_top));
    let top_plane = make_plane(
        top_origin,
        x_axis,
        y_axis,
        2.0 * radius_top,
        2.0 * radius_top,
    )?;
    let top_pcurve = curve_to_plane_parameters(&top_circle, top_origin, x_axis, y_axis)?;
    validated(
        BrepSolid {
            mass_properties_cache: Default::default(),
            id: 150,
            vertices: vec![
                VertexRecord {
                    id: 1,
                    point: bottom_point,
                },
                VertexRecord {
                    id: 2,
                    point: top_point,
                },
            ],
            edges: vec![
                EdgeRecord {
                    id: 10,
                    curve: bottom_circle,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 1,
                    end_vertex_id: 1,
                    degenerate: false,
                    name: None,
                },
                EdgeRecord {
                    id: 11,
                    curve: top_circle,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 2,
                    end_vertex_id: 2,
                    degenerate: false,
                    name: None,
                },
                EdgeRecord {
                    id: 12,
                    curve: seam,
                    t0: 0.0,
                    t1: 1.0,
                    start_vertex_id: 1,
                    end_vertex_id: 2,
                    degenerate: false,
                    name: None,
                },
            ],
            shells: vec![ShellRecord {
                id: 149,
                faces: vec![
                    FaceRecord {
                        id: 105,
                        surface: side,
                        same_sense: true,
                        loops: vec![LoopRecord {
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
                        }],
                        name: None,
                    },
                    bottom_face,
                    FaceRecord {
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
                    },
                ],
            }],
            genus: 0,
        },
        "cone frustum",
    )
}

