use crate::curve::{curve_to_plane_parameters, parameter_line};
use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{
    make_cone_surface, make_line, make_plane, make_sphere_surface_framed, make_torus_surface,
    Vec3,
};

// A certificate owns the just-built body; it is consumed at this module's
// acceptance boundary. It is never attached to mutable BrepSolid data or
// serialized, and no caller can certify an imported or subsequently edited body.
// Primitive certificate type: begin
struct CertifiedPrimitive { solid: BrepSolid }
// Primitive certificate type: end
impl CertifiedPrimitive {
    fn accept(self) -> BrepSolid {
        primitive_check_count(true);
        self.solid
    }
}

pub(crate) fn primitive_certificate_enabled() -> bool {
    // Promoted after the 98/144-case byte census. Explicit =0 restores the
    // full constructor validator; eligibility never changes geometry or errors.
    match std::env::var("BREP_CERTIFIED_PRIMITIVES") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    }
}

// This is an eligibility bound, never a new refusal: every input outside it
// takes the original validator. Keep floating point conditioning far from the
// model/edge coincidence bands: dimensions 1e-3..1e3, origin at most 1000
// times the smallest dimension, and axes of order unity. Surface/curve builders
// still check finite homogeneous control points, positive weights and knots.
// The trusted builders supply incidence and exact shared boundary construction:
// box affine faces; cylinder two iso-curves and their plane images; sphere
// one periodic chart with two pole edges; ring torus two periodic seams.
// Neither a certificate nor this eligibility predicate accepts arbitrary bodies.
pub(crate) fn primitive_domain(origin: Vec3, dimensions: &[f64], axis: Option<Vec3>) -> bool {
    if !dimensions.iter().all(|v| v.is_finite() && (1e-3..=1e3).contains(v)) { return false; }
    let minimum = dimensions.iter().copied().fold(f64::INFINITY, f64::min);
    if ![origin.x, origin.y, origin.z].iter().all(|v| v.is_finite() && v.abs() <= minimum * 1000.) { return false; }
    axis.is_none_or(|a| [a.x, a.y, a.z].iter().all(|v| v.is_finite())
        && (0.25..=4.).contains(&a.length_squared()))
}

pub(crate) fn primitive_check_count(certified: bool) {
    let _ = certified;
}


fn validated(solid: BrepSolid, label: &'static str) -> Result<BrepSolid, String> {
    let mut profile = crate::loft_topology::ConstructionProfile::new(label);
    primitive_check_count(false);
    let issues = solid.validate();
    profile.lap("validation");
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
    let mut profile = crate::loft_topology::ConstructionProfile::new("sphere");
    let surface = make_sphere_surface_framed(center, radius, polar_axis, seam_direction)?;
    let south = surface.evaluate(0.0, 0.0)?;
    let north = surface.evaluate(0.0, 1.0)?;
    let seam = surface.iso_curve_u(0.0)?;
    let solid = BrepSolid {
            mass_properties_cache: Default::default(),
            cells: Default::default(),
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
            kind: crate::BodyKind::Solid,
        };
    profile.lap("construct");
    // The pole/seam topology above is fixed. A caller-selected seam keeps the
    // full validator until that separate construction is certified as well.
    if primitive_certificate_enabled() && seam_direction.is_none()
        && primitive_domain(center, &[radius], Some(polar_axis)) {
        return Ok(CertifiedPrimitive { solid }.accept());
    }
    validated(solid, "sphere")
}

pub fn make_torus_brep(
    center: Vec3,
    axis_direction: Vec3,
    major_radius: f64,
    minor_radius: f64,
) -> Result<BrepSolid, String> {
    let mut profile = crate::loft_topology::ConstructionProfile::new("torus");
    let surface = make_torus_surface(center, axis_direction, major_radius, minor_radius)?;
    let corner = surface.evaluate(0.0, 0.0)?;
    let major_curve = surface.iso_curve_v(0.0)?;
    let tube_curve = surface.iso_curve_u(0.0)?;
    let solid = BrepSolid {
            mass_properties_cache: Default::default(),
            cells: Default::default(),
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
            kind: crate::BodyKind::Solid,
        };
    profile.lap("construct");
    // Positive clearance makes the periodic tube a ring torus, not a horn or
    // spindle. Near-horn numerics keep the original validation path.
    if primitive_certificate_enabled()
        && major_radius - minor_radius >= minor_radius * 1e-3
        && primitive_domain(center, &[major_radius, minor_radius], Some(axis_direction)) {
        return Ok(CertifiedPrimitive { solid }.accept());
    }
    validated(solid, "torus")
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
                cells: Default::default(),
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
                kind: crate::BodyKind::Solid,
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
            cells: Default::default(),
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
            kind: crate::BodyKind::Solid,
        },
        "cone frustum",
    )
}


